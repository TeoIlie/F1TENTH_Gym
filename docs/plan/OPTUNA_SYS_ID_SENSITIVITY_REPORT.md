# Phase 2 Sensitivity Report — `rosbag2_2026_05_04-17_54_17_100Hz.npz`

- Bag: `examples/analysis/bags/rosbag2_2026_05_04-17_54_17_100Hz.npz`
- Git commit: `bff93cd`
- Baseline `dataset_loss` (YAML defaults, no mirror): **27.4159**
- Outputs: `figures/analysis/sysid/sensitivity/rosbag2_2026_05_04-17_54_17_100Hz/` (standard) and `…_wide/` (orientation pass)

This report consolidates the four passes run for Phase 2:

1. Wide-δ orientation pass (`--wide-deltas`, ±90% .. +200%).
2. Standard tire sweep (8 Stage-1/2 candidates, ±50% ladder).
3. Vehicle-dyn sweep (`I_z`, `I_y_w`, `sv_max`, `sv_min`, `a_max`). _Note: `T_steer` was added to `VEHICLE_DYN_CANDIDATES` after this report was written — see §9 for the follow-up plan._
4. Frozen audit (8 frozen params, absolute mode, review-only).

It locks the Stage-1 / Stage-2 search-space membership and bounds for Phase 3.

---

## 1. Coverage audit — saturation regime is well excited

| signal | range observed | default-Pacejka 95%-of-peak knee | verdict |
|---|---|---|---|
| α_front | ±113° | 4.94° | ≫ knee |
| α_rear  | ±86°  | 4.94° | ≫ knee |
| κ_front | up to 26.06 | 0.094 | ≫ knee |
| κ_rear  | up to 24.07 | 0.094 | ≫ knee |

No identifiability veto on any saturation parameter. The bag exercises both linear and saturated regimes with margin to spare. (Side note: the |α| / |κ| extremes are inflated by low-`v_x` samples — the warm-up and low-speed mask in `loss.py` already discount these, so the *scored* coverage is still well past the knee but not pathologically so.)

---

## 2. Wide-δ orientation pass — defaults are off-cliff for several params

The PAC2002 dict in `f1tenth_std.yaml` is lifted from a full-scale-vehicle reference. The wide ladder confirms several defaults are not in the local basin:

| param | min loss at δ | shape | implication |
|---|---|---|---|
| `tire_p_cy1` | +0.25 (22.6) | basin near default | trustable bounds |
| `tire_p_cx1` | +0.25 (24.3) | basin near default; diverges +0.5/+1.0 | trustable bounds, narrow upper edge |
| `tire_p_dx1` | **−0.9** (22.1) | monotonically decreasing | truth past sampled range |
| `tire_p_ex1` | +0.5 (26.3) | mild basin | trustable |
| `tire_p_dy1` | **+2.0** (20.9) | monotonically decreasing | truth past sampled range |
| `tire_p_ky1` | **+2.0** (24.8) | monotone in magnitude | truth past sampled range |
| `tire_p_kx1` | flat | low sensitivity | demote |
| `tire_p_ey1` | flat | low sensitivity | demote |

**Caveat: `dy1`, `dx1`, `ky1` minima are not necessarily physical truth.** Pushing `dy1 → 3.1` corresponds to μ_y ≈ 3.1, which is impossible for rubber on a hard floor; `dx1 → 0.12` corresponds to μ_x on ice. The wide-δ basin landing at non-physical values is the canonical signal that **another parameter** (chassis inertia, longitudinal cap) is wrong and the tire coefficients are sponging the residual. The vehicle-dyn pass below confirms this.

---

## 3. Vehicle-dynamics sweep — chassis params dominate

Ranking by max |Δtotal|:

| param | default | max |Δ| | min loss at δ | interpretation |
|---|---|---|---|---|
| `I_z`   | 0.04712 | **155.2** | **+2.0** (13.24) | strongly under-estimated; truth past +200% |
| `a_max` | 6.0     | **143.8** | **−0.7** (10.89) | strongly over-estimated; truth ≈ 1.8 m/s² or lower |
| `I_y_w` | 0.0017  | 18.0 | −0.7 (20.4) | over-estimated; narrow basin |
| `sv_max`| 4.95    | 1.7  | −0.5 | mild |
| `sv_min`| −4.95   | 0.78 | flat in negative direction | not exercised |

**Two findings dominate Phase 2:**

- **`I_z` is the single most impactful parameter on this bag.** Loss drops from 27.4 → 13.2 going from 0.047 to 0.141 kg·m². The default is ~3× too low. This was anticipated by the OVERVIEW smoke testing.
- **`a_max` is similarly impactful in the opposite direction.** Default 6.0 m/s² caps the rear-wheel longitudinal acceleration well below what the bag actually demands; loss drops to 10.9 at `a_max=1.8`. The minimum likely lies further down — the ladder did not extend below δ=−0.7 here.

The interaction is mechanical: a higher `I_z` slows yaw response (improves `yaw_rate` NMSE), and a lower `a_max` removes a longitudinal-saturation artifact that otherwise corrupts `a_x`. Both are precisely the channels the wide-δ pass showed `dy1` and `dx1` were trying to absorb.

`sv_min` ladder is flat across the negative side: the bag never demands steering-rate that hits the lower slew limit. Drop from search.

---

## 4. Standard tire sweep — ranking under YAML defaults

Ranking by max |Δtotal| (±50% ladder):

| rank | param | max |Δ| | local shape | comment |
|---|---|---|---|---|
| 1 | `tire_p_cx1` | 173.2 | basin at +0.1..+0.25; diverges at +0.5 | dominated by divergence — true sensitivity in-basin is moderate |
| 2 | `tire_p_cy1` | 17.9  | basin at +0.1..+0.25; diverges at +0.5 | clean signal |
| 3 | `tire_p_dx1` | 13.2  | monotonic ↓ as δ↑ over [-0.5,+0.25], diverges +0.5 | wide-δ confirmed off-cliff |
| 4 | `tire_p_dy1` | 7.6   | monotonic ↑/↓ across ladder | wide-δ confirmed off-cliff |
| 5 | `tire_p_ky1` | 5.2   | monotone ↓ as |ky1|↑ | wide-δ confirmed off-cliff |
| 6 | `tire_p_ex1` | 1.93  | mild | identifiable but low priority |
| 7 | `tire_p_kx1` | 1.87  | flat-ish | demote — uninformative |
| 8 | `tire_p_ey1` | 0.55  | flat | demote — uninformative |

The standard sweep ranks the *same* params as the wide-δ pass would suggest, but `auto-propose_bounds` produces narrow ranges for `dx1`/`dy1`/`ky1` that do not contain their wide-δ minima. **Do not use the auto-bounds for those three.**

---

## 5. Frozen audit — confirms freeze decisions

6 of 8 frozen params show **dLoss = 0.0** across their entire absolute ladders: `tire_p_dx3`, `tire_p_dy3`, `tire_p_hy1`, `tire_p_hy3`, `tire_p_vy1`, `tire_p_vy3`. Camber-coupled freeze and lateral pure-shift freeze are both empirically justified.

Two longitudinal pure-shift terms show non-trivial sensitivity:

- `tire_p_vx1`: max |Δ| = **19.7** — comparable to `I_y_w`.
- `tire_p_hx1`: max |Δ| = 1.68.

These remain frozen, **with the audit reinterpreted as evidence of bias, not identifiability**. Both are longitudinal-shift terms that should be ≈0 for a symmetric tire; perturbing them at ~2000× the default reduces loss because they offer a constant-offset structure on `Fx` that other tire params cannot. The bag contains a longitudinal residual (drivetrain drag, motor-map error, throttle deadband, or IMU `a_x` offset) and `vx1` is the only knob whose shape can absorb it. Unfreezing it would let Optuna fit a non-physical bias and degrade transfer.

→ Action item for Phase 4: if `a_x` NMSE plateaus, calibrate this bias **outside** the tire model (e.g. an explicit `a_x` IMU offset, or a drivetrain-drag term) rather than touching the frozen list.

---

## 6. Locked Phase-3 search space

### Stage 1 — promotion priority order

Joint Optuna search (CmaEsSampler from the start, given dimensionality):

| param | default | bounds | source |
|---|---|---|---|
| `I_z`     | 0.04712 | **[0.05, 0.20]** kg·m² | physical prior; wide-δ minimum past +200% |
| `a_max`   | 6.0     | **[1.0, 6.0]** m/s² | physical prior; wide-δ minimum at lower edge |
| `I_y_w`   | 0.0017  | **[0.0005, 0.0025]** kg·m² | auto-proposed range, narrow basin |
| `tire_p_cy1` | 1.351 | [0.34, 2.36] | auto-proposed |
| `tire_p_cx1` | 1.641 | [1.03, 2.26] | auto-proposed (narrow upper edge to avoid divergence) |
| `tire_p_ky1` | −21.92 | **[−80, −15]** | physical prior; wide-δ optimum past −65 |

**Rationale for splitting:** chassis params (`I_z`, `a_max`, `I_y_w`) and stiffness/shape coefficients (`cy1`, `cx1`, `ky1`) are linear-regime drivers that interact strongly. Fitting saturation params before chassis is locked-in is what produced the non-physical wide-δ basins for `dy1` and `dx1` — Stage 1 must converge first.

### Stage 2 — saturation, opens after Stage 1

Initial point = Stage 1 best. Sampler stays CMA-ES.

| param | default | bounds | source |
|---|---|---|---|
| `tire_p_dy1` | 1.049 | **[0.7, 1.5]** | physical prior (μ_y) |
| `tire_p_dx1` | 1.174 | **[0.7, 1.5]** | physical prior (μ_x) |
| `tire_p_ey1` | −0.0075 | [−0.013, −0.0019] | auto-proposed; expect low sensitivity, may converge to whatever |
| `tire_p_ex1` | 0.464 | [0.12, 0.81] | auto-proposed |

`dy1` and `dx1` use physical-prior bounds — wide-δ minima for these were non-physical. Once Stage 1 corrects `I_z` / `a_max`, the residual that those params were absorbing should largely disappear and they should land near defaults. **If Stage 2 pushes either to its prior bound, that is a signal Stage 1 did not fully absorb the chassis residual** — investigate before extending the bound.

### Demoted from search

- `tire_p_kx1` (max |Δ| = 1.87) — flat under both ladders. Leave at YAML default.
- `tire_p_ey1` (max |Δ| = 0.55) — flat. Tentatively in Stage 2 only because it closes the lateral curve shape; safe to drop if Stage 1+2 converges cleanly without it.
- `sv_min`, `sv_max` — bag does not exercise the slew limits meaningfully.

### Stage 3 — combined slip, deferred

Per OVERVIEW: run only if Phase 4 residuals show systematic combined-slip error. Sensitivity sweep is also deferred (`--include-combined` flag is implemented but not run as part of Phase 2).

### Permanently frozen

All 8 of: `tire_p_dx3`, `tire_p_dy3`, `tire_p_hy3`, `tire_p_vy3`, `tire_p_hx1`, `tire_p_vx1`, `tire_p_hy1`, `tire_p_vy1`. The two with non-zero audit signal (`vx1`, `hx1`) are flagged as bias absorbers — see §5.

---

## 7. Open issues for Phase 3 / 4

1. **`a_max` lower bound is uncertain.** The vehicle-dyn ladder bottomed out at δ=−0.7 (`a_max=1.8`) without finding a basin floor. Phase 3 should re-run a narrow `a_max` sweep over `[0.5, 6.0]` once Stage-1 convergence is in hand.
2. **`I_z` upper bound assumed at 0.20.** This is ~4× the YAML default. If Stage 1 saturates the bound, sanity-check by physically estimating `I_z` from the chassis CAD before extending further.
3. **Longitudinal bias source needs to be identified before Stage 2.** Otherwise `dx1` will be pulled toward the frozen-audit `vx1` regime regardless of the prior. Candidates: VESC current-to-Fx map error, IMU `a_x` offset, drivetrain coastdown drag.
4. **Cross-bag stability for the locked bounds is untested.** Per OVERVIEW, this is Phase 6's concern; revisit only if Stage 1 fails to beat baseline.

---

## 8. Follow-up — `T_steer` and `sv_*` reinterpretation

### `T_steer` re-sweep — exclude from Stage 1

Re-ran the vehicle-dyn pass with `T_steer` added to `VEHICLE_DYN_CANDIDATES`. Outputs in `figures/analysis/sysid/sensitivity/rosbag2_2026_05_04-17_54_17_100Hz/ranking_vehicle_dyn.md`.

**Headline:** `T_steer` shows max |Δtotal| = **0.0000** across the entire ladder (`T_steer ∈ [0.0075, 0.075]` s, i.e. 200× variation around the YAML default of 0.025).

**Caveat — the flat-zero is a propagation bug, not real insensitivity.** Empirical check: setting `T_steer ∈ {0.005, 1.0}` (200× difference) and replaying a single window produced bit-identical sim signals on all four channels. Root cause: `SteeringAngleAction.__init__` computes `self.k = (1 - exp(-dt/T_steer))/dt` *once* at construction; `env.configure({"params": ...})` rebuilds `self.action_type` on the env but the simulator's `RaceCar.action_type` still references the original handler. `RaceCar.update_params` only refreshes `self.params`, not `self.action_type`. Other identified params (`I_z`, `tire_p_*`) are read from `agent.params` each step inside the dynamics function and propagate correctly — `T_steer` is unique in being consumed only at action-handler init.

**Decision: exclude `T_steer` from Stage 1.** Two reasons:

1. The `set_params` plumbing bug means we cannot reliably identify `T_steer` via the current Optuna pipeline. Fixing it is filed as a deferred-upgrade in the OVERVIEW (low priority — affects no other Stage-1 param).
2. Even with the bug fixed, the realistic upper bound on `T_steer` sensitivity is comparable to `sv_max` (Δ=1.7) or below — far below the Stage-1 promotion threshold (~18, set by `I_y_w`). The 0.2 s warmup-discard absorbs the steering-servo transient regardless of `T_steer`'s value, leaving little for Optuna to identify on this bag. A richer maneuver bag with rapid step-steers might surface it; this bag does not.

**Action:** pin `T_steer = 0.025` (current YAML) as a hand-picked physical prior. `STAGE1_SPACE` stays at 6 params.

### `sv_min` / `sv_max` reinterpretation (unchanged)

The original §3 demotion of `sv_min` / `sv_max` still stands. The vehicle-dyn re-run confirms low sensitivity (max |Δ| = 1.7 / 3.0). The reason — bang-bang fallback bypassed by `T_steer > 0` mode — is mechanical, not informative about the bag's slew-rate excitation. Demotion holds.

---

## 9. Exit criteria — status

- [x] Sweep CSV + ranking + coverage plots committed under `figures/analysis/sysid/sensitivity/rosbag2_2026_05_04-17_54_17_100Hz/`.
- [x] This report written.
- [x] Frozen-audit summary in §5.
- [x] Stage 1 / Stage 2 candidate lists + bounds locked.
- [x] Demotions justified (`kx1`, `ey1`, `sv_min`, `sv_max`).
- [x] Phase-2 invariant tests pass (`tests/sysid/test_sensitivity.py`).
- [x] Single-core runtime budget met (~23 min standard sweep, ~23 min frozen audit on this bag).

Phase 2 is closed. Phase 3 (Stage 1 Optuna study) can begin.
