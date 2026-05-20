# Port `STDKinematics::update_pacejka` from f110-simulator into gymkhana

## Context

The f110-simulator (`race_stack/base_system/f110-simulator/src/std_kinematics.cpp`) implements a **dynamic single-track model with a Pacejka Magic Formula tire law on the lateral axis only** (`update_pacejka`, lines 14–133). This model is exactly what `system_identification/on_track_sys_id` identifies — its output is 8 Pacejka coefficients (`B,C,D,E` per axle) plus a steady-state `(v_x, δ) → a_lat` LUT. The `SIM_pacejka.txt` file already contains a 1/10-scale identification.

Gymkhana (`race_stack/base_system/gymkhana/`) currently offers four models: `KS, ST, MB, STD`. The existing `STD` model (`single_track_drift/single_track_drift.py`) uses the *full PAC2002* tire model with longitudinal slip and wheel-spin states (9-state vector, `R_w, I_y_w, T_sb, T_se`) — heavier than `update_pacejka` and not directly compatible with sysid output. The `ST` model uses linear cornering stiffness with small-angle linearizations and cannot be tire-law-swapped in place because its `ψ̈` and `β̇` formulas are closed-form expansions that only exist *because* `F_y` is linear in `α, β, δ` (see `single_track.py:111–127`).

We want a new gymkhana model that mirrors `update_pacejka` exactly: 7-state ST-shaped vector (no longitudinal slip / wheel-spin), Pacejka lateral tires, full nonlinear ODE (no small-angle approximations), low-speed kinematic blend. Goal: drop in identified Pacejka coefficients from `on_track_sys_id` and have a faithful 1/10 dynamics model in the RL training loop.

## Recommended approach

Add a new dynamics module `STP` (Single Track Pacejka), parallel to `single_track.py` and `single_track_drift/`. Do not modify `ST`, `STD`, or `KS`. The structure mirrors the f110 implementation directly (`F_y` as named intermediates → body-frame `(v̇_x, v̇_y, ψ̈)` → chain-rule into `(V̇, β̇)`), with derivative-level low-speed blending and gymkhana-style constraints applied inside `f`.

Per resolved design decisions:
- **Naming:** `DynamicModel.STP`, `vehicle_dynamics_stp`, folder `single_track_pacejka/`.
- **`mu` kept explicit** as a multiplier on `F_y` (matches f110 `update_pacejka`; lets user disable by setting `mu=1.0`).
- **Initial params:** new YAML seeded from `system_identification/on_track_sys_id/models/SIM/SIM_pacejka.txt`.

### Files to create

1. **`race_stack/base_system/gymkhana/envs/dynamic_models/single_track_pacejka/__init__.py`**
   - Re-export `vehicle_dynamics_stp`, `get_standardized_state_stp`.
   - No `init_stp` needed — 7-state init is identical to ST (handled by the existing zero-init path in `DynamicModel.get_initial_state`).

2. **`race_stack/base_system/gymkhana/envs/dynamic_models/single_track_pacejka/single_track_pacejka.py`**
   - `def vehicle_dynamics_stp(x, u_init, params) -> np.ndarray` returning shape `(7,)`.
   - State: `[X, Y, δ, V, ψ, ψ̇, β]` (same as ST, same `get_standardized_state_st` semantics).
   - Inputs: `[STEER_VEL, ACCL]`.
   - Implementation steps inside the function:
     1. Apply `steering_constraint(...)` and `accl_constraints(...)` from `..utils` (same pattern as `single_track.py:50–69`).
     2. Compute slip angles with `V > v_min` guard (gymkhana sign convention, equivalent to f110's `atan2(-v_y - l_f·ω, v_x) + δ`):
        ```python
        alpha_f = np.arctan((V*sin(β) + ψ̇*l_f) / (V*cos(β))) - δ   if V > v_min else 0.0
        alpha_r = np.arctan((V*sin(β) - ψ̇*l_r) / (V*cos(β)))       if V > v_min else 0.0
        ```
     3. Vertical loads with longitudinal weight transfer (uses `params["h_s"]` to align with STD naming):
        ```python
        F_zf = m * (-ACCL*h_s + g*l_r) / (l_f + l_r)
        F_zr = m * ( ACCL*h_s + g*l_f) / (l_f + l_r)
        ```
     4. Pacejka lateral forces with explicit `mu` multiplier (mirrors `std_kinematics.cpp:69–76`):
        ```python
        F_yf = mu * D_f * F_zf * sin(C_f * arctan(B_f*α_f - E_f*(B_f*α_f - arctan(B_f*α_f))))
        F_yr = mu * D_r * F_zr * sin(C_r * arctan(B_r*α_r - E_r*(B_r*α_r - arctan(B_r*α_r))))
        ```
     5. Body-frame derivatives, **exact, no small-angle** (mirrors `std_kinematics.cpp:84,87,88`), substituting `v_x = V·cos β`, `v_y = V·sin β`:
        ```python
        v_x_dot = ACCL - (1/m)*F_yf*sin(δ) + V*sin(β)*ψ̇
        v_y_dot = (1/m)*(F_yr + F_yf*cos(δ)) - V*cos(β)*ψ̇
        psi_ddot = (1/I_z)*(-F_yr*l_r + F_yf*l_f*cos(δ))
        ```
     6. Chain-rule into `(V̇, β̇)` (this is the gymkhana-vs-f110 parametrization difference; only needed because state stores `V, β` not `v_x, v_y`):
        ```python
        V_dot   = v_x_dot*cos(β) + v_y_dot*sin(β)
        beta_dot = (v_y_dot*cos(β) - v_x_dot*sin(β)) / max(V, v_min)
        ```
     7. Compute kinematic-bicycle derivatives by calling `vehicle_dynamics_ks_cog` from `..kinematic` with the projected 5-state `[X, Y, δ, V, ψ]`. This is the same pattern STD uses (`single_track_drift.py:182`) and avoids reimplementing kinematic equations.
     8. Blend **derivatives** (not states) with `tanh` weights — gymkhana-scale thresholds, *not* the f110 `v_b=3, v_s=1` defaults (those were tuned for full-scale velocities and would leave a 1/10 car kinematic almost always):
        ```python
        v_s = 0.2;  v_b = 0.05            # follow STD's choice (line 69–70)
        w_std = 0.5 * (np.tanh((V - v_s) / v_b) + 1)
        w_ks  = 1 - w_std
        ```
        For the kinematic side: `V_dot_ks = ACCL`, `psi_dot_ks = f_ks[4]`, `beta_dot_ks = (lr*STEER_VEL)/(lwb*cos²δ*(1+(tan²δ·lr/lwb)²))` (closed-form from STD line 184), `psi_ddot_ks` from STD line 187.
     9. Assemble the 7-vector:
        ```python
        f = np.array([
            V*cos(ψ + β),
            V*sin(ψ + β),
            STEER_VEL,
            w_std*V_dot    + w_ks*ACCL,
            ψ̇,                              # PSI_DOT (state pass-through; same in dyn and kin)
            w_std*psi_ddot + w_ks*psi_ddot_ks,
            w_std*beta_dot + w_ks*beta_dot_ks,
        ])
        ```
   - `def get_standardized_state_stp(x)`: identical to `get_standardized_state_st` (same 7-element state, same `(V, β)` → `(v_x, v_y)` decomposition). Re-export from `single_track.py` to avoid duplication, OR define a thin alias.

3. **`race_stack/base_system/gymkhana/envs/params/f1tenth_stp.yaml`**
   - Copy geometry/inertia/constraints from `f1tenth_std.yaml` (mass, `lf`, `lr`, `h_s`, `I_z`, `s_min/max`, `sv_min/max`, `v_switch`, `a_max`, `v_min/max`, `width`, `length`).
   - Add `mu: 1.0489` (carry over from `f1tenth_st.yaml` line 8).
   - Add 8 Pacejka coefficients seeded from `on_track_sys_id/models/SIM/SIM_pacejka.txt`:
     ```yaml
     B_f: 6.6185;  C_f: 1.6102;  D_f: 0.652;   E_f: 0.7826
     B_r: 7.9507;  C_r: 3.692;   D_r: 0.6691;  E_r: 0.5595
     ```
   - Note in a comment that `D` is dimensionless (peak μ-equivalent) and that sysid output sets `mu=1.0` since μ is folded into `D`. With `mu=1.0489` here we are slightly scaling up the identified peak — flag this for the user; they may want to change `mu: 1.0` for sysid-faithful behavior.
   - Drop wheel-dynamics keys (`R_w`, `I_y_w`, `T_sb`, `T_se`) and full-PAC2002 tire keys (`tire_p_cx1`, ..., `tire_r_vy6`) — STP doesn't use them.

### Files to modify

4. **`race_stack/base_system/gymkhana/envs/dynamic_models/__init__.py`**
   - Import `vehicle_dynamics_stp, get_standardized_state_stp` from `.single_track_pacejka`.
   - Add `DynamicModel.STP = 5` enum member with docstring.
   - Extend `DynamicModel.from_string`: accept `"stp"`.
   - Extend `DynamicModel.get_initial_state`: STP uses 7-element zero-init (identical to ST branch — extend the existing ST branch or add a parallel branch). No `init_*` inflation.
   - Extend `DynamicModel.f_dynamics`: return `vehicle_dynamics_stp` for STP.
   - Extend `DynamicModel.get_standardized_state_fn`: return `get_standardized_state_stp` for STP.
   - Update the module docstring's tire-parameter section to list `B_f, C_f, D_f, E_f, B_r, C_r, D_r, E_r` (STP-only).

5. **`race_stack/base_system/gymkhana/envs/gymkhana_env.py`**
   - Add `@classmethod GKEnv.f1tenth_stp_vehicle_params(cls) -> dict: return load_params("f1tenth_stp")`. Mirror the existing `f1tenth_std_vehicle_params` (line 452).
   - No changes to `default_config`, `step`, integrator wiring, or observation handling — STP plugs into the existing `self.integrator.integrate(f=f_dynamics, ...)` path (`base_classes.py:354`) automatically once `f_dynamics` is wired.

### Functions to reuse (do not reimplement)

- `vehicle_dynamics_ks_cog` from `envs/dynamic_models/kinematic.py:76` — kinematic blend partner.
- `steering_constraint`, `accl_constraints` from `envs/dynamic_models/utils.py` — input limits.
- `get_standardized_state_st` from `envs/dynamic_models/single_track.py:135` — state shape is identical, can alias.
- `load_params` (already used in `gymkhana_env.py`) — YAML loader.
- The `RK4Integrator` in `envs/integrator.py:32` — works as-is on any `f(x, u, params)`.

## Critical implementation notes

- **Sign convention for `α`**: Use the gymkhana convention `α_f = atan(...) - δ` (matches `single_track_drift.py:99` and is algebraically equivalent to f110's `atan2(-v_y - l_f·ω, v_x) + δ` for `V > 0`). The Pacejka curve is odd, so `F_y(α)` flips sign accordingly; the body-frame equations as written above already match this convention. Sanity-check after implementation: positive `δ` → vehicle yaws left.
- **Don't apply small-angle approximations** anywhere in the STP function. The whole point of Pacejka is large-slip behavior; linearizing defeats the purpose and the STD f110 code keeps `cos δ`, `sin δ`, `cos β`, `sin β` exact.
- **Mix derivatives, not states.** RK4 evaluates `f` four times per step; mixing post-integration states (as f110 does under Euler) is incompatible with the gymkhana integrator architecture. STD already shows the correct derivative-mix pattern.
- **Guard `1/V` and `1/cos β`** in α and `β_dot`. Use `if V > v_min` early returns where divisions appear, matching STD line 99–100.
- **RK4 may evaluate at slightly non-physical intermediate states** (`V` near zero or marginally negative). The blend tanh and the `V > v_min` guards handle this gracefully — same as STD.
- **Constraints inside `f`** (gymkhana convention), not outside (f110 convention).

## Verification

End-to-end checks, executable from the gymkhana root:

1. **Smoke test — straight line, no steering:**
   ```python
   env = GKEnv(config={"model": "stp", "params": GKEnv.f1tenth_stp_vehicle_params()})
   obs, _ = env.reset()
   for _ in range(200):
       env.step(np.array([0.0, 1.0]))   # zero steering, accel=1 m/s²
   # Expect: V increases linearly, β stays ≈ 0, ψ̇ stays ≈ 0, X grows, Y stays ≈ 0.
   ```

2. **Low-slip equivalence with ST:** at low cornering accelerations and `V ≈ 3 m/s`, STP with sysid params should produce trajectories close to ST (linearization regime). Roll out same `(STEER_VEL, ACCL)` schedule under both `model="st"` and `model="stp"`; expect `|x_st - x_stp| < O(1cm)` over 1 s.

3. **Tire saturation:** apply a step steering input of 0.4 rad at `V = 6 m/s`. Plot `F_yf` and `F_yr` (expose as a debug return or compute offline from state). Expect both to saturate near `D · F_z` rather than growing unboundedly (which is the failure mode of ST in this regime).

4. **Low-speed blend smoothness:** start at `V = 0.05` m/s and accelerate through `V = 0.5` m/s. Plot `β` and `ψ̇` over time — should be C¹ smooth across the threshold (no jumps), confirming the tanh derivative-blend works.

5. **Drift bias check:** with rear-axle Pacejka coefficients reduced (e.g. `D_r *= 0.6`), expect the car to oversteer / spin under hard cornering — confirms rear tires saturate first as physics dictates.

6. **Integration compatibility:** confirm both `integrator: "rk4"` (default) and `integrator: "euler"` produce stable rollouts for the same input schedule (no NaN, no divergence).

7. **Sysid round-trip (optional, longer-term):** drop the LUT generated by `on_track_sys_id` for SIM into `system_identification/steering_lookup/cfg/` and run a MAP-controlled time trial in this gymkhana env with `model="stp"`. Should produce coherent racing behavior — validates the parameter-set compatibility end-to-end.
