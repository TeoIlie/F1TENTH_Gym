# Optuna System Identification — Master Plan

## Context

This repo's STD (Single-Track Drift) dynamics model uses PAC2002 tire parameters that cannot be physically measured on a 1/10 F1TENTH car. To close the sim2real gap for RL drift policy transfer, we identify these parameters by replaying real Vicon-recorded command sequences through the simulator under candidate parameter sets and minimizing a sim-vs-real trajectory residual via Optuna.

- Model: `gymkhana/envs/dynamic_models/single_track_drift/single_track_drift.py`
- Default params: `gymkhana/envs/params/f1tenth_std.yaml`
- Real bags (NPZ, 100 Hz, Vicon): `examples/analysis/bags/`

## Pipeline

```
load_dataset(bag.npz)          # → list[Window]
        │
        ▼
Rollout(env)                   # one GKEnv; set_params hot-swaps PAC2002 coefficients
        │
        ▼  rollout.run(window) → dict[ch → sim signal]
dataset_loss                   # fixed-scale weighted MSE on {yaw_rate, v_y, a_x, v_x, omega}
        │
        ▼
Optuna study (CMA-ES, JournalStorage)  # 6-D Stage-1, 4-D Stage-2
        │
        ▼
f1tenth_std_optuna_stage{1,2}.yaml
```

## Module map

| File | LoC | Purpose |
|---|---|---|
| `examples/analysis/sysid/dataset.py` | 361 | Load NPZ, slice to windows, mirror. Includes a CLI plot helper for visual validation. |
| `examples/analysis/sysid/loss.py` | 77 | `channel_loss`, `window_loss`, `dataset_loss`. Pure functions. |
| `examples/analysis/sysid/rollout.py` | 361 | `Rollout` (one env, hot-swap params, NaN guard). Includes a CLI sim-vs-real overlay plot. |
| `examples/analysis/sysid/sensitivity.py` | 203 | OAT sweep tool — `run_sweep`, console ranking, CSV. Used to lock the search space; archived for re-runs on new bags. |
| `examples/analysis/sysid/search_spaces.py` | 79 | Stage-1 / Stage-2 distributions, `apply_trial_params`, import-time invariants. |
| `examples/analysis/sysid/study.py` | 166 | `Objective`, `build_study`, `dump_best_params`, CLI. |
| `examples/analysis/sysid/env.py` | 12 | `SYSID_PARAMS = GKEnv.f1tenth_std_vehicle_params()` (canonical base dict). |
| `tests/sysid/` | 6 files | Full coverage; runs in ~30 s. |

## Locked design decisions

| Decision | Choice | Why |
|---|---|---|
| Loss channels | `yaw_rate`, `v_y`, `a_x`, `v_x`, `omega`, `pose`, `yaw`, `beta` | Direct dynamics outputs + drivetrain anchor (sim ω = mean of front/rear vs VESC AWD scalar) + world-frame XY tracking + heading (catches integrated yaw_rate bias) + slip angle (re-weights v_y/v_x residual to emphasize orientation; not new information). |
| Loss formula | Fixed-scale weighted MSE for non-angular channels; **wrapped MSE** for angular channels (`yaw`, `beta`): residual is `arctan2(sin(s−r), cos(s−r))` before squaring, gated by `_ANGULAR_CHANNELS` in `loss.py`. | Plain `(s−r)²` is incorrect near the ±π wrap (a 5° physical error at the wrap boundary becomes a 360° numerical error). Bounding angular contributions to ≤ coeff·π² also keeps divergent trials from blowing up the loss. |
| Channel coefficients | See `CHANNEL_COEFFS` in `loss.py` (**values still being tuned via `validate.py`** — refer to source, not this doc, for the current numbers). | Single per-channel multiplier on MSE — bundles importance with unit-scaling so contributions land on a common magnitude. Replaces prior weight + per-dataset NMSE variance (which collapsed on steady-state bags). `pose` is a 2-D channel `(N+1, 2)`; element-wise MSE gives `½·mean(Δx²+Δy²)` and the ½ is absorbed into the coefficient (rotation-invariant in world frame). |
| Window length / stride | 1.5 s / 0.5 s | Excite saturation; limit chaotic divergence. |
| Warmup discard | 0.2 s | Eats steering-servo transient from `delta_init = cmd_steer[t0]`. |
| Reset state | 9-element STD user state, `omega = rs_core_speed/R_w` (AWD) | `env.reset(options={"states": ...})` accepts 9-wide. |
| Mirroring | Off by default; opt-in via `--mirror` | STD is *near*-symmetric: PAC2002 shift terms (`p_hy1`, `p_vy1`, `p_hx1`, `p_vx1`, plus camber-coupled `p_hy3`/`p_vy3`/`p_dy3`/`p_dx3`) break exact L/R symmetry and are all frozen in the search space, so the model can only fit the symmetric component of the data anyway. On a balanced L+R bag, mirroring is a near-no-op that doubles compute. On a single-handed bag, opt in via `--mirror` to project the fit onto the symmetric subspace and avoid params absorbing the bag's directional bias. |
| Sampler | `CmaEsSampler(seed=...)` | 6-D continuous + parameter coupling — CMA-ES wheelhouse. |
| Pruner | None (v1) | Pruning is a runtime optimization, not correctness. Add only if cost demands. |
| Storage | `JournalStorage` at `<repo>/studies/<name>.journal` | Append-only log file, race-free for parallel workers (no DDL, no schema, no alembic). |
| Resume | `load_if_exists=True` | Same study-name continues; new name = fresh study. |
| Stage chaining | Via `base_params` | Stage 1 / 2 search spaces are disjoint — Stage 2 loads Stage 1 best YAML. |
| NaN handling | Map to `inf` | Trial COMPLETE with infinite value, never FAIL. |

## Search spaces (locked by Phase-2 sensitivity analysis)

Full justification in [`OPTUNA_SYS_ID_SENSITIVITY_REPORT.md`](OPTUNA_SYS_ID_SENSITIVITY_REPORT.md).

**Stage 1 — chassis + linear regime (6 params):**

| param | bounds | source |
|---|---|---|
| `I_z` | `[0.05, 0.20]` | physical prior; wide-δ minimum past +200% |
| `a_max` | `[1.0, 6.0]` | physical prior; wide-δ minimum at lower edge |
| `I_y_w` | `[0.0005, 0.0025]` | auto-proposed |
| `tire_p_cy1` | `[0.34, 2.36]` | auto-proposed |
| `tire_p_cx1` | `[1.03, 2.26]` | auto-proposed |
| `tire_p_ky1` | `[-80, -15]` | physical prior |

**Stage 2 — saturation (4 params, run after Stage 1 with `--base-params <stage1.yaml>`):**

| param | bounds | source |
|---|---|---|
| `tire_p_dy1` | `[0.7, 1.5]` | physical prior (μ_y) |
| `tire_p_dx1` | `[0.7, 1.5]` | physical prior (μ_x) |
| `tire_p_ey1` | `[-0.013, -0.0019]` | auto-proposed |
| `tire_p_ex1` | `[0.12, 0.81]` | auto-proposed |

**Permanently frozen** (camber-coupled + pure-shift, see report §5):
`tire_p_dx3`, `tire_p_dy3`, `tire_p_hy3`, `tire_p_vy3`, `tire_p_hx1`, `tire_p_vx1`, `tire_p_hy1`, `tire_p_vy1`.

`T_steer` was evaluated and excluded — partly because of a `set_params` propagation bug (see Deferred Upgrades), partly because realistic sensitivity is too low for Stage-1 inclusion.

## Why staged?

When two params can both reduce the same residual, joint optimization picks whichever combination minimizes loss — not whichever is *physically correct*. Phase-2's wide-δ pass showed `dy1` / `dx1` wanting to drift to non-physical μ values (>3.0 / <0.15) because they were sponging chassis-inertia error. Locking chassis params (`I_z`, `a_max`, `I_y_w`) first means tire-saturation params have a meaningful residual to fit, not a ghost of the chassis error.

If Stage 1's results show clean chassis correction and Stage 2 isn't needed (or pushes against bounds, indicating the staging assumption broke), reconsider. A single 10-D joint study is the fallback.

## CLI usage

**Run a study:**

```bash
# Stage 1 (default base = SYSID_PARAMS)
python -m examples.analysis.sysid.study \
    --bag examples/analysis/bags/<bag>.npz \
    --stage 1 --n-trials 500

# Stage 2 (uses Stage 1 best as base)
python -m examples.analysis.sysid.study \
    --bag examples/analysis/bags/<bag>.npz \
    --stage 2 --n-trials 500 \
    --base-params gymkhana/envs/params/f1tenth_std_optuna_stage1.yaml
```

Auto-derived defaults: `--study-name` is `<bag_stem>_stage{N}`, `--storage` is `<repo>/studies/<study_name>.journal` (path to the JournalStorage file), `--out-yaml` is `gymkhana/envs/params/f1tenth_std_optuna_stage{N}.yaml`. Re-running the same name continues the existing study.

**Parallel workers** (CMA-ES degrades past ~4):

```bash
mkdir -p studies
for i in 1 2 3 4; do
  python -m examples.analysis.sysid.study --bag <path> --stage 1 \
    --study-name myrun_stage1 --n-trials 125 &
done; wait
```

**Live monitoring via wandb** (always on):

Every study run logs per-trial `value`, per-channel loss contributions (`contrib/yaw_rate`, `contrib/v_y`, `contrib/a_x`, `contrib/v_x`, `contrib/omega`, `contrib/pose`, `contrib/yaw`, `contrib/beta` — each is `CHANNEL_COEFFS[ch] * MSE_ch`, with `yaw`/`beta` using wrapped residuals), and suggested params (`param/I_z`, ...) to wandb project `f1tenth-sysid`. Each parallel worker is a separate wandb run grouped by study name; the wandb UI shows N colored lines per chart with min/mean aggregation. Wandb auth must be set up on the host (same as RL training).

**Re-run sensitivity on a new bag:**

```bash
python -m examples.analysis.sysid.sensitivity --path <bag>.npz --candidates stage12
```

Outputs CSV + console ranking. The richer Phase-2 outputs (markdown report, plots, coverage histograms) were one-shot deliverables; their archived versions live under `figures/analysis/sysid/sensitivity/` and the locked findings are in `OPTUNA_SYS_ID_SENSITIVITY_REPORT.md`.

## Pre-launch verification

1. `python -m pytest tests/sysid/ -q` — all green (~30 s).
2. 5-trial smoke study — confirms plumbing, journal write, CLI completion.
3. Optional: `study.enqueue_trial({k: midpoint(b.low, b.high) for k, b in STAGE1_SPACE.items()})` then run 1 trial; assert it matches a direct `dataset_loss` call (catches `apply_trial_params` / `set_params` no-ops). This is `test_enqueue_trial_matches_direct_rollout` in `test_study.py`.

## Phase-1 baseline (reference)

`circle_Apr6_100Hz.npz`, no mirror, 11 windows, YAML defaults: **total = 5.02**, per-channel `{v_y: 1.85, a_x: 0.33, yaw_rate: 0.30, v_x: 0.15}`.

`rosbag2_2026_05_04-17_54_17_100Hz.npz`, drift bag baseline: **27.4**.

5-trial CMA-ES smoke on the drift bag drove this to 8.83 (3.1× improvement).

## Deferred upgrades

### `set_params` → action-handler propagation bug

`env.configure({"params": ...})` rebuilds `self.action_type` on the env (`gymkhana_env.py:649`), but `self.sim.agents[i].action_type` still references the original `CarAction` from env construction. `RaceCar.update_params(params)` (`base_classes.py:199`) only updates `self.params`. Affects only params consumed in `CarAction.__init__` (currently `T_steer`, plus `s_max`/`sv_max` for normalization scale factors when `normalize_act=True`). Other identified params (`I_z`, `tire_p_*`) are read from `agent.params` each step in the dynamics function and propagate fine.

**Fix sketch:** make `RaceCar.update_params` rebuild `self.action_type` if relevant params changed, or add a `Simulator.set_action_type(...)` propagation. ~10-line change in `base_classes.py`. Trigger to fix: any future sysid effort that needs to identify `T_steer` (e.g. on a richer maneuver bag with rapid step-steers where the warmup discard isn't sufficient).

### Plotting / wandb / multi-bag aggregation

Defer to a Phase-6 "validation" pass once Stage 1 / 2 produce a candidate YAML worth validating on held-out bags.

### Pruning (`MedianPruner`)

Defer until v1 wall-clock pain materializes. Adds per-window iteration in `Objective.__call__` instead of calling `dataset_loss` directly.
