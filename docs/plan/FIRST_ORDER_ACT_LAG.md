# Plan: First-Order Steering Actuator Lag (Exact Exponential Discretization)

## Context

For sim2real transfer of RL policies trained in this gym, the actuator response must match the real car. Today, steering behaviour is modeled by:

1. A 2-step dead-time transport delay on the target angle (`base_classes.py:328-334`, `steer_buffer`).
2. A **bang-bang controller** that outputs `±sv_max` or `0` based on the sign of the angle error (`bang_bang_steer` in `dynamic_models/utils.py:88-109`, called from `SteeringAngleAction.act` at `action.py:236`).
3. A rate clip to `[sv_min, sv_max]` and end-stops at `[s_min, s_max]` inside `steering_constraint` (called by all three dynamics models).

This produces a piecewise-linear step response — flat during the buffer delay, then a linear ramp at `sv_max`, then flat at the target. Real servos respond **exponentially** once rate saturation ends, following a first-order ODE:

```
δ̇(t) = (1/T_δ) · (δ_ref(t) − δ(t))
```

We want to replace bang-bang with this first-order lag (keeping dead-time and rate-clip in place, since real servos exhibit all three). This plan implements that change using the **exact zero-order-hold discretization**, gated by a new `T_steer` parameter so existing behaviour is preserved when the parameter is absent/zero.

## Discretization Choice — Why Exact, Not Euler

Two valid ways to encode first-order lag at the action layer, both one-liners:

| Form        | Effective update                              | Source                                                  |
|-------------|-----------------------------------------------|---------------------------------------------------------|
| **Euler**   | `δ_{k+1} = δ_k + (dt/T)·(δ_ref − δ_k)`        | `sv = (δ_ref − δ)/T`; integrator ZOH-applies for `dt`   |
| **Exact**   | `δ_{k+1} = δ_k + α·(δ_ref − δ_k)`, α = 1 − e^(−dt/T) | `sv = α·(δ_ref − δ)/dt`; integrator reproduces the ZOH closed-form |

Both are one-line changes inside `SteeringAngleAction.act`. Comparison at `dt=0.01`:

| T_δ    | α_Euler | α_exact | error   |
|--------|---------|---------|---------|
| 0.100  | 0.100   | 0.0952  | +5%     |
| 0.050  | 0.200   | 0.181   | +10%    |
| 0.025  | 0.400   | 0.330   | +21%    |
| 0.010  | 1.000   | 0.632   | +58%    |
| 0.005  | 2.000   | 0.865   | unstable|

Euler **overshoots the physical time constant** (sim feels faster than the modelled servo), becomes unstable for `dt/T ≥ 2`, and makes the effective lag coupled to the sim timestep rather than the physics. The exact form is always stable, dt-independent (for any fixed ZOH on δ_ref), and reproduces exactly the closed-form solution of δ̇ = (1/T)(δ_ref − δ). For the target `T_δ ≈ 0.025 s`, the error gap is ~21%, enough to matter for sim2real. **Exact is the correct choice.**

The exact α is encoded back to a steering velocity `sv = α·(δ_ref − δ)/dt` so that the existing integrator (`DELTA_DOT = STEER_VEL`, constant sv over the step) reproduces the desired end-of-step δ. This requires zero changes to the dynamics models.

## Why the Action Layer (Not Dynamics Models)

All three models — KS (`kinematic.py:68`), ST (`single_track.py:88`), STD (`single_track_drift.py:209`) — share the identical equation `DELTA_DOT = STEER_VEL` and all funnel through `steering_constraint`. A single branch in `SteeringAngleAction.act` propagates to every model with no duplication.

## Scope Decision — SteeringAngleAction Only

Deliberately scoped to `SteeringAngleAction`. `SteeringSpeedAction` (commands a target steering *velocity* with no lag on sv) and `AcclAction` (commands a target *acceleration* with no lag on a) are also currently instantaneous and would benefit from analogous first-order lag models `sv̇ = (1/T_sv)(sv_ref − sv)` and `ȧ = (1/T_a)(a_ref − a)` respectively.

The asymmetry in implementation cost justifies deferring them:

- **`SteeringAngleAction` is stateless**: the variable being lagged (δ) already lives in every dynamics model's state vector (`x[2]`), so the lag can be encoded as a one-site change inside `act()` by returning an appropriately-scaled `sv`. No changes to `RaceCar`, `Simulator`, or any dynamics model.
- **`SteeringSpeedAction` and `AcclAction` are stateful**: they would need new persistent variables (`sv_actual`, `a_actual`) carried on `RaceCar` between steps, reset logic in `RaceCar.reset`, and filter update logic threaded through `base_classes.py:update_pose`. Both changes are structurally parallel and belong in a single follow-up PR.

This plan explicitly leaves those two for a follow-up. The `T_steer` YAML key introduced here does not conflict with the future `T_steer_speed` and `T_accl` keys.

## Files to Modify

### 1. `gymkhana/envs/action.py`

- In `SteeringAngleAction.__init__`: accept and store `timestep`; pre-compute `self.alpha = 1 − exp(−dt/T_steer)` if `params.get("T_steer", 0.0) > 0`, else leave `self.alpha = None` (sentinel for bang-bang fallback).
- In `SteeringAngleAction.act`:

  ```python
  desired_angle = action * self.scale_factor
  if self.alpha is not None:
      sv = self.alpha * (desired_angle - state[2]) / self.dt
  else:
      sv = bang_bang_steer(desired_angle, state[2], params["sv_max"])
  return sv
  ```

- `CarAction.__init__`: add `timestep: float` argument and forward it only to the `SteeringAngleAction` constructor (others ignore it). Keep positional compat where reasonable; new kwarg is non-breaking if defaulted.

### 2. `gymkhana/envs/gymkhana_env.py`

- Line 100: pass `timestep=self.timestep` into `CarAction(...)`. `self.timestep` is already assigned on line 94, immediately above.

### 3. `gymkhana/envs/params/*.yaml` (all five)

Append to each:

```yaml
T_steer: 0.025  # Steering actuator time constant (s). 0 or omitted → bang-bang.
```

Files:
- `f1tenth_st.yaml`
- `f1tenth_std.yaml`
- `f1tenth_std_drift_bias.yaml`
- `f1fifth.yaml`
- `fullscale.yaml`

Rationale for YAML (vs. gym_config): `T_steer` is a physical servo property, belongs next to `sv_max`, `s_max`. `timestep` stays in gym_config because it's a sim discretization choice, not a vehicle property.

### 4. Tests — `tests/test_steering_lag.py` (new file)

New test module (pytest, consistent with `tests/test_action.py` fixture style). Cover:

1. **Back-compat — bang-bang preserved**: construct `SteeringAngleAction` with `T_steer=0` (or key absent); assert `act(...)` returns the same values as `bang_bang_steer` for a sweep of `(desired, current)` pairs.
2. **Exact exponential step response**: construct with `T_steer=0.05`, `dt=0.01`; feed a constant `δ_ref = 0.4`, step δ forward manually using the returned sv (`δ += dt·sv`) for N steps; assert the sequence matches the closed-form `δ_k = δ_ref · (1 − exp(−k·dt/T))` to within ~1e-10 (it should be exact to float precision since we derive sv from the exact α).
3. **Time-to-63% is T_δ**: from step 2, assert the first sample where `δ ≥ 0.632·δ_ref` lands at `t ≈ T_δ` within `dt/2`.
4. **Rate saturation clipping**: when `α·(δ_ref − δ)/dt > sv_max`, apply `steering_constraint` externally and verify the clipped rate matches `sv_max`; verify the tail (post-saturation) returns to exponential.
5. **End-to-end env step**: create a `GKEnv` with a params override containing `T_steer = 0.025`, commit a constant max-left steering action for ~0.2 s, and assert `state[2]` increases monotonically and approaches `s_max` asymptotically (no overshoot, no oscillation). Mirror the `spielberg_std_env` fixture pattern from `tests/test_normalize_logic.py`.
6. **Dt-independence sanity**: for two env timesteps (e.g. 0.01 and 0.005) with the same `T_steer`, the real-time trajectory of δ matches closely (Euler would not pass this test; exact will).

Keep tests fast — unit-level (cases 1–4) should not instantiate a full env.

## Back-Compat Guarantees

- Params YAML key `T_steer` absent → `params.get("T_steer", 0.0)` → `self.alpha = None` → bang-bang branch runs. Existing models/training runs are unaffected.
- `CarAction` gains a new `timestep=` keyword argument; `SpeedAction`, `AcclAction`, `SteeringSpeedAction` ignore it. External callers (only `gymkhana_env.py:100`) are updated in the same PR.
- `steer_buffer` dead-time (`base_classes.py:328-334`) and `steering_constraint` rate-clip (`utils.py:60`) remain in place. All three still cascade for a realistic dead-time + first-order-lag + rate-saturation servo model.

## Critical Files Summary

| Path                                                           | Change                                              |
|----------------------------------------------------------------|-----------------------------------------------------|
| `gymkhana/envs/action.py`                                      | Modify `SteeringAngleAction.__init__/act`, thread `timestep` through `CarAction` |
| `gymkhana/envs/gymkhana_env.py`                                | Pass `timestep=self.timestep` into `CarAction(...)` at line 100 |
| `gymkhana/envs/params/f1tenth_st.yaml`                         | Add `T_steer` key                                    |
| `gymkhana/envs/params/f1tenth_std.yaml`                        | Add `T_steer` key                                    |
| `gymkhana/envs/params/f1tenth_std_drift_bias.yaml`             | Add `T_steer` key                                    |
| `gymkhana/envs/params/f1fifth.yaml`                            | Add `T_steer` key                                    |
| `gymkhana/envs/params/fullscale.yaml`                          | Add `T_steer` key                                    |
| `tests/test_steering_lag.py`                                   | New pytest module (cases 1–6 above)                  |

Reused utilities (no changes): `bang_bang_steer` (still used in fallback branch), `steering_constraint`, `deep_update`, params `load_params`.

## Verification

1. **Unit tests**: `python3 -m pytest tests/test_steering_lag.py -v` — all six cases pass.
2. **Regression**: `python3 -m pytest tests/` — existing action/dynamics/normalize tests still pass (they should, because `T_steer=0.025` in the YAMLs changes default behaviour; the regression sweep may need the fallback path — verify by also running with a params override `T_steer=0` and confirming identical numerics to `main`).
3. **Visual inspection**: `python3 examples/drift_debug.py` (or equivalent) to visually confirm steering now feels less "snappy" on commanded step changes.
4. **Step-response plot**: the new test module (or a quick script) can log δ(t) under a step command; plot vs. the analytic exponential and confirm overlap.
5. **T_δ identification follow-up** (out-of-scope here but noted): the value `0.025 s` is a reasonable starting estimate derived from a 0→full-lock time of ~0.125 s under the "5·T ≈ settling" rule-of-thumb. A proper bench-ID should command a *small* step (not rate-saturated) and fit the exponential tail — this plan parameterizes the value so refining it later is a one-line YAML edit.

## Future Work — Vicon Bench Identification of `sv_max` and `T_steer`

A single 0→full-lock measurement cannot separate `sv_max` from `T_steer`: the trajectory is rate-saturated for most of the motion and only briefly exponential at the tail, so total settling time lies on a degenerate ridge of `(sv_max, T_steer)` combinations. Disentangling them requires either two excitations (small step + large step) or a richer signal (chirp, PRBS).

**Proposed bench protocol** (chock the wheels, car stationary):

1. **Direct δ measurement via wheel-mounted markers.** Place 3 mocap markers on the steered wheel face (rim or hub) — not on a stick: a 30 cm stick adds ~`(1/3)·m·L²` rotational inertia about the kingpin and can double or triple the assembly inertia, biasing `T_steer` upward. Markers on the wheel itself add negligible inertia. Compute δ from the angle between the wheel-face plane and the chassis-forward direction (use a chassis marker triplet for the reference frame). Mocap angular resolution from a ~10 cm marker baseline at 0.5 mm noise is ~5 mrad single-frame, ~1.5 mrad after light smoothing — comfortable for resolving a 0.05 rad step to a few percent.

2. **Small step (fits `T_steer`).** Command `δ_ref: 0 → 0.05–0.10 rad`, hold 0.5 s, repeat 5–10×. Amplitude chosen so that `α·δ_ref/dt < sv_max` and the rate-clip never binds → response is pure exponential. Fit `T_steer` either from `t_63` (time to cross 63% of step) or by least-squares on `δ(t) = δ_ref(1 − exp(−(t − t_0)/T))`.

3. **Large step (fits `sv_max`).** Command `0 → s_max`, identify the rate-saturated window where `|δ_ref − δ| > 0.5·s_max`, and fit a line through `δ(t)` in that window — the slope is `sv_max`. Take the median across repeats.

4. **Dead-time as a free byproduct.** Cross-correlate logged `δ_ref(t)` against measured `δ(t)`; the lag at peak correlation is the transport delay. This lets the currently-hardcoded 2-step `steer_buffer` be calibrated rather than assumed.

**Excitation cross-check.** A small-amplitude chirp (0.5–10 Hz, ~0.05 rad amplitude) gives the full Bode plot of the servo. The −3 dB corner is `1/(2π·T_steer)`; rolloff steeper than 20 dB/dec means the first-order model is inadequate (the servo has higher-order dynamics worth capturing).

**Optuna's role.** With direct δ measurement, the steering parameters are identified by closed-form fits — no optimizer needed. Reserve Optuna for chassis/tire parameters and feed it the bench-derived `(sv_max, T_steer, dead_time)` as fixed values, otherwise it will explore the degenerate ridge with no information to resolve it.

**Mocap requirements.** ≥ 200 Hz sampling (rise spans ~5 frames at `T = 0.025 s`, 200 Hz). Common timebase between controller-logged `δ_ref(t)` and mocap (NTP sync or hardware sync pulse) — clock skew of even 5 ms biases `T_steer` directly.
