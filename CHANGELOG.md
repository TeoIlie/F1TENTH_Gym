# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/),
and this project adheres to [Semantic Versioning](https://semver.org/).

## [Unreleased]

### Added
- **`drift_config` preset**: public starter config for STD-model drift environments — `from gymkhana import drift_config; gym.make("gymkhana:gymkhana-v0", config=drift_config(map="Drift"))`. Bundles the physics, control, observation, normalization, and reset settings used for drift training. Keyword args override individual fields. Available to PyPI users without access to `train/config/`. `train/config/env_config.py::_base_config` now layers training-specific keys on top of this preset to keep a single source of truth.
- **`render_config` gym option**: pass a dict of field overrides for the packaged `gymkhana/envs/rendering/rendering.yaml` (e.g. `{"window_size": 1200, "render_type": "pygame", "show_ctr_debug": True}`) directly through `gym.make(config={...})`. Omitted fields keep their yaml defaults. Makes renderer fields (window size, render backend, debug toggles, vehicle palette) tunable without editing the packaged file — needed for PyPI users who can't modify the installed source.
- **Explicit YAML packaging**: `pyproject.toml` now declares `include = [{path = "gymkhana/**/*.yaml", ...}]` so vehicle parameter and rendering YAMLs are guaranteed to ship in built wheels regardless of git tracking state.
- **Optimal raceline generation**: `maps/extract_raceline.py` produces an optimized racing line for any map via ForzaETH's fork of TUM's `global_racetrajectory_optimization` (mincurv_iqp). The emitted `<map>_raceline.csv` now carries the optimizer's full corner-aware vx/ax profile (previously overridden with constants), which `Raceline.from_raceline_file` and `examples/waypoint_follow.py` already consume as Pure Pursuit target speeds.
- **STP (Single Track Pacejka) model**: dynamic single-track bicycle with lateral-only Pacejka Magic Formula tire model, ported from the ForzaETH f110-simulator. Shares ST's 7-element state layout. Selectable via `model='stp'` with parameters from `GKEnv.f1tenth_stp_vehicle_params()` (`f1tenth_stp.yaml`). Supports `drift_st` observation type alongside ST.
- Aggregated min/max observation tracking across parallelized environments (works with both `normalize_obs=True` and `normalize_obs=False`) for tuning normalization bounds. Now backed by an `ObsMinMaxSnapshotCallback` that periodically writes merged per-subproc trackers to `outputs/config/<run_id>/obs_min_max.yaml` and streams cumulative per-feature bounds-violation magnitudes to wandb under `obs_bounds/<feature>/over` and `.../under` for live monitoring during training.
- **Configurable instability prevention**: opt-in via `prevent_instability` gym config flag. When enabled, post-RK4 sanity checks on the standardized state revert blow-ups and truncate the episode; cumulative event count is logged to wandb under `instability/total` via a new `InstabilityCountCallback`, with end-of-run per-env breakdown printed to stdout. Detection bounds are exposed as `instability_yaw_rate_bound` and `instability_slip_bound`.

## [1.2.0] - 2026-04-11

### Added
- **Control debug panel**: real-time steering/throttle visualization (PyQt6 renderer)
- **Observation debug overlay**: live observation values overlay on the map (PyQt6 renderer)
- **ONNX export**: convert SB3 models to ONNX for sim-to-real transfer via `OnnxPolicyRunner`
- **Norm bounds extraction**: save normalization bounds to config for reuse in external packages
- **Custom neural network architecture**: configurable layer sizes via `net_arch` in RL config
- **Morris sensitivity analysis**: parameter sensitivity analysis script for STD model
- **Observation sensitivity analysis**: script for analyzing observation feature importance
- **Regression tests**: tests for action ordering and other previously encountered bugs
- **Trajectory comparison script**: compare KS, ST, STD model trajectories for sim2real validation

### Fixed
- Action ordering bug where `control_input` order affected action array mapping
- Rendering `rgb_array` mode bugfix
- Kinematic model yaw_rate bugfix
- Removed unused `abstractmethod` decorator on `CarAction.act` method

### Changed
- Refactored docstrings for consistency, correctness, and RTD readability
- Clarified supported Python versions in README
- Reorganized README for clearer table of contents
- Made unit test suite more efficient
- Moved commonroad submodule to analysis subfolder

### Removed
- Docker support (Poetry and PyPI are sufficient)
- Euclidean reward option for recovery learning

## [1.1.1] - 2026-03-15

### Fixed
- Version bump only (packaging fix)

## [1.1.0] - 2026-03-15

### Added
- **Recovery training**: dedicated training mode (`training_mode: "recover"`) with recovery-specific rewards, resets, and evaluation
- **Curriculum learning**: `CurriculumLearningCallback` that gradually expands recovery state initialization ranges as agent success rate improves
- **Transfer learning**: transfer pretrained model weights to new tasks with optional critic reset and `log_std` re-initialization
- **MPC controllers**: Kinematic MPC (KMPC) and Single-Track MPC (STMPC) via acados integration
- **Multi-map training**: train across multiple maps in parallel environments
- **Sparse width observations**: `sparse_width_obs` config to reduce observation dimensionality when track width varies little
- **P/PD controllers**: simple proportional controllers for benchmarking recovery performance
- **Performance metrics**: beta-r phase plane analysis, recovery trajectory plotting, controller comparison metrics
- PyPI publishing via `publish.yml` GitHub Actions workflow (triggered on tag push)
- ReadTheDocs documentation site with Sphinx RTD theme
- Logos, favicon, and branding assets

### Changed
- Refactored package for PyPI publishing (renamed `f1tenth_gym` to `gymkhana`)
- Migrated from Black to Ruff for linting, formatting, and import sorting with pre-commit hooks
- Refactored controllers with abstract base class
- Refactored training scripts with shared `train_common.py` and `train_utils.py`
- Updated map loading to use maintained source for better compatibility
- Reorganized project structure (analysis, figures, plans into dedicated directories)

### Fixed
- Mexico City track naming issue
- Eval env should use curriculum max ranges

## [1.0.0] - 2026-01-30

### Added
- **Drift dynamics**: Single Track Drift (STD) model with PAC2002 tire physics
- **Reverse/random driving direction**: `track_direction` config option (`normal`, `reverse`, `random`) for balanced cornering training
- **State reset**: `env.reset(options={"states": ...})` for full 7-d state initialization (STD model)
- **Arc-length visualization**: render arc-length annotations along the centerline for Frenet coordinate debugging
- **Observation types**: `FeaturesObservation` (drift) with slip angle, lookahead curvatures/widths; observation and action normalization
- **Predictive collision**: TTC-based collision checking as alternative to Frenet-based
- **Wall deflection mode**: configurable track boundary behavior (boundary vs. wall collision)
- **Progress reward gain**: configurable `progress_gain` multiplier for forward progress reward
- **Frenet projection debugging**: `debug_frenet_projection` visualization option
- **Lookahead curvature visualization**: render lookahead sampling points ahead of the vehicle
- **Wandb integration**: experiment tracking, model logging, download, and resume from wandb
- **Performance metrics**: beta-r phase plane analysis, recovery trajectory plotting
- PPO training infrastructure with SB3 and parallel environments

### Fixed
- Wraparound bug in progress tracking (robust fix with unit tests)
- Normalization logic when both actions and observations are normalized
- Reverse direction reset bug where random reset could point vehicle the wrong way due to cached reference line
- Reverse direction normalization fix where min/max curvature bounds need to be symmetric

### Changed
- Configuration logic simplified and centralized in `train/config/`
- Moved plotting files to `analysis/` folder

[Unreleased]: https://github.com/TeoIlie/Gym-Khana/compare/v1.2.0...HEAD
[1.2.0]: https://github.com/TeoIlie/Gym-Khana/compare/v1.1.1...v1.2.0
[1.1.1]: https://github.com/TeoIlie/Gym-Khana/compare/v1.1.0...v1.1.1
[1.1.0]: https://github.com/TeoIlie/Gym-Khana/compare/v1.0.0...v1.1.0
[1.0.0]: https://github.com/TeoIlie/Gym-Khana/releases/tag/v1.0.0
