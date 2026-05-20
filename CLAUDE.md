# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## About Gym-Khana

Gym-Khana is a high-fidelity simulation platform for 1/10th scale autonomous racing cars, built as a Gymnasium RL environment. The name is a play on words: "Gym" for RL Gymnasium and "Khana" from Gymkhana (drift/precision driving motorsport). It is a fork of the original f1tenth_gym project by UPenn, extended with drift dynamics, recovery training, MPC controllers, and RL training infrastructure.

### Autonomous Racing Context
The simulator models realistic vehicle dynamics, sensor data (LiDAR), and racing scenarios to enable development and testing of:
- **Path Planning**: Algorithms like Pure Pursuit (implemented in examples)
- **Control Systems**: Speed and steering control for high-speed racing
- **Perception**: LiDAR-based environment understanding
- **SLAM**: Simultaneous Localization and Mapping
- **Reinforcement Learning**: Training autonomous racing agents with PPO and other algorithms

### Research Applications
- **Multi-agent Racing**: Support for multiple competing vehicles
- **Safety Systems**: Collision detection and avoidance
- **Vehicle Dynamics**: Multiple models including kinematic, single-track, multi-body, and drift models
- **Sensor Simulation**: Accurate LiDAR ray-casting for perception research
- **Drifting**: STD (Single Track Drift) model with PAC2002 tire model for aggressive driving

## Development Commands

### Installation
```bash
# With pip (recommended for this project)
pip install -e .

# Or with poetry
poetry install
source $(poetry env info -p)/bin/activate
```

### Virtual Environment
This project uses a virtual environment at `rl_env/`. Always activate it before running commands:
```bash
source rl_env/bin/activate
```

### Testing
Run all tests:
```bash
python3 -m pytest
```

Run specific test file:
```bash
python3 -m pytest tests/test_f110_env.py
```

Run specific test function:
```bash
python3 -m pytest tests/test_f110_env.py::test_function_name -v
```

The CI runs pytest for Python versions 3.10-3.12.

### Running Examples
```bash
cd examples
python3 waypoint_follow.py  # Pure Pursuit path following
python3 p_steer_controller.py  # Simple P controller for centerline following
python3 drift_debug.py  # Debug drift behavior with visualization
```

### Training Models
Main training script with multiple modes:
```bash
# Train a new model
python train/ppo_race.py --m t

# Evaluate a local model (uses latest wandb run if --path not specified)
python train/ppo_race.py --m e
python train/ppo_race.py --m e --path /path/to/model.zip

# Download model from wandb and evaluate
python train/ppo_race.py --m d --run_id <wandb_run_id>

# Continue training from existing model
python train/ppo_race.py --m c --path /path/to/model.zip --additional_timesteps 10000000
```

### Formatting/Linting
Uses `ruff` for formatting, linting, import sorting, and unused import removal.

Manual run:
```bash
ruff check --fix .  # lint + auto-fix (unused imports, import sorting)
ruff format .       # format (Black-compatible)
```

Pre-commit hooks run both automatically on `git commit` (configured in `.pre-commit-config.yaml`).

VSCode runs both on save via `.vscode/settings.json` (requires `charliermarsh.ruff` extension).

Configuration (`pyproject.toml` under `[tool.ruff]`):
- Line length: 120 characters
- Target: Python 3.10+
- Lint rules: E (pycodestyle), F (pyflakes/unused imports), W (warnings), I (isort)

## Architecture Overview

### Core Autonomous Vehicle Components

**GKEnv** (`gymkhana/envs/gymkhana_env.py`): Main gymnasium environment implementing F1/10th vehicle simulation with realistic dynamics. This is the entry point for creating environments via `gym.make('gymkhana:gymkhana-v0', config={...})`.

**Dynamic Models** (`gymkhana/envs/dynamic_models/`): Four vehicle dynamics models:
- **KS (Kinematic Single Track)**: `kinematic.py` - Simplest model, pure kinematics without tire forces
- **ST (Single Track)**: `single_track.py` - Single-track dynamics model without explicit tire model
- **MB (Multi-Body)**: `multi_body/` - Most detailed model with full tire modeling, may be overkill for RL
- **STD (Single Track Drift)**: `single_track_drift/` - Single-track dynamics + Pacejka PAC2002 tire model for drift simulation (recommended for RL drift training)

**Observation System** (`gymkhana/envs/observation.py`): Factory pattern for different observation types including "rl" and "drift" observations. Supports lookahead curvature sampling and normalization.

**Action System** (`gymkhana/envs/action.py`): Handles different control input types like `["speed", "steering_angle"]` or `["accl", "steering_angle"]`.

**Laser Models** (`gymkhana/envs/laser_models.py`): LiDAR simulation with ray-casting for autonomous perception. Includes TTC (Time-To-Collision) calculations.

**Collision Models** (`gymkhana/envs/collision_models.py`): Safety-critical collision detection systems with support for predictive TTC-based collision checking or Frenet-based collision checking.

**Base Classes** (`gymkhana/envs/base_classes.py`): Core simulation infrastructure:
- `RaceCar`: Handles physics and laser scan of single vehicle
- `Simulator`: Multi-agent simulation orchestrator
- Step method at line 503 defines core simulation loop

**Track System** (`gymkhana/envs/track/`): Racing circuit definitions with centerline and raceline support. Includes cubic spline interpolation for smooth trajectories.

**Reset System** (`gymkhana/envs/reset/`): Different reset strategies for RL training including random static resets and map-based resets.

**Rendering** (`gymkhana/envs/rendering/`): Visualization support using pygame and PyQt with debug visualization options.

### RL Training Infrastructure

**Configuration** (`train/config/`):
- `env_config.py`: Centralized Gym environment and RL parameter configuration
- `rl_config.yaml`: RL hyperparameters (n_envs, learning rate, batch size, etc.)
- `gym_config.yaml`: Gym environment parameters (map, model, timestep, etc.)
- Key functions:
  - `get_drift_test_config()`: Configuration for testing drift-trained models
  - `get_drift_train_config()`: Configuration for training drift models

**Training Scripts** (`train/`):
- `ppo_race.py`: training script for racing
- `ppo_recover.py`: training script for recovery
- `ppo_example.py`: Simpler example PPO training script
- `train_common.py`: Shared training workflow (TrainingProfile dataclass, train/eval/download/continue logic)
- `train_utils.py`: Shared utilities for output directory management, environment creation, and callbacks

### Package Structure
- **gymkhana/**: Main package following gymnasium RL environment interface
- **examples/**: Reference implementations of autonomous racing algorithms
- **train/**: RL training scripts and configuration
- **tests/**: Comprehensive test suite including model validation
- **maps/**: Racing track definitions (real-world inspired circuits)
- **plan/**: Development planning documents (drift, reward, normalization plans)

### Key Dependencies for Autonomous Systems
- **gymnasium**: RL environment interface for agent training
- **stable-baselines3**: PPO and other RL algorithms
- **wandb**: Experiment tracking and model logging
- **numba**: JIT compilation for real-time performance in control loops
- **pygame/PyQt6**: Real-time visualization for algorithm development and debugging
- **numpy/scipy**: Mathematical foundations for control and estimation algorithms

## Important Configuration Options

### Vehicle Models and Control
```python
env = gym.make('gymkhana:gymkhana-v0', config={
    'model': 'std',  # Use 'std' for drifting with PAC2002 tire model
    'control_input': ['accl', 'steering_angle'],  # Order-independent; action array is always [steer, longitudinal]
    'params': GKEnv.f1tenth_std_vehicle_params(),  # Drift parameters for 1/10 scale
})
```

### Observation Configuration
```python
config = {
    'observation_config': {'type': 'drift'},  # or 'rl' for standard RL observations
    'lookahead_n_points': 10,  # Number of lookahead points (default: 10)
    'lookahead_ds': 0.3,  # Spacing between points in meters (default: 0.3m)
    'normalize_obs': True,  # Enable observation normalization (only for 'drift' obs)
    'normalize_act': True,  # Enable action normalization
}
```

### Collision and Safety
```python
config = {
    'predictive_collision': True,  # True for TTC collision checking, False for Frenet-based
    'wall_deflection': False,  # False treats edges as boundaries, True as walls causing collision
}
```

### Debugging and Visualization
```python
config = {
    'render_mode': 'human',  # Enable visualization
    'render_track_lines': True,  # Render centerline (green) and raceline (red)
    'render_lookahead_curvatures': True,  # Visualize lookahead curvature points (yellow)
    'debug_frenet_projection': True,  # Visualize Frenet coordinate accuracy
    'record_obs_min_max': True,  # Record min/max observations for normalization tuning
}
```

Debug with breakpoints by looping through environment steps (see `tests/drift_debug.py`).

## Branches and Fork History

- Original `f1tenth_gym` branch `main` → renamed to `f1tenth_main_original` in this fork
- Original `f1tenth_gym` branch `rl_example` → renamed to `main` in this fork (active development branch)
- This fork actively extends the RL capabilities of the original project

## Vehicle Parameters

Vehicle parameters are defined in YAML files in `gymkhana/envs/params/`:
- `f1tenth_st.yaml`: 1/10 scale F1TENTH car (ST model)
- `f1tenth_std.yaml`: 1/10 scale F1TENTH car with PAC2002 tire model (STD model for drifting)
- `f1tenth_std_drift_bias.yaml`: STD model tuned for increased drift tendency
- `f1fifth.yaml`: 1/5 scale F1FIFTH car
- `fullscale.yaml`: Full-scale vehicle from CommonRoad

Access parameters via class methods on `GKEnv`:
```python
GKEnv.f1tenth_vehicle_params()      # ST model params
GKEnv.f1tenth_std_vehicle_params()  # STD model params (recommended for drift)
GKEnv.f1fifth_vehicle_params()      # 1/5 scale params
GKEnv.fullscale_vehicle_params()    # Full-scale params
```

Test script `tests/model_validation/test_f1tenth_std_params.py` creates comparison figures and parameter YAML dumps in `figures/tire_params/` for validation.

## Map Management

Maps are managed through a two-tier system:
- **Local `maps/` directory**: git submodule from https://github.com/TeoIlie/F1TENTH_Racetracks — used for development
- **User cache `~/.gymkhana/maps/`**: auto-downloaded from GitHub releases for pip-installed users

The download URL is configured in `gymkhana/envs/track/track_utils.py::MAPS_URL`.

### `maps/` runs in its own Python 3.8 environment

The `maps/` submodule is **not** installable into the parent simulator's poetry env. Its scripts (`extract_centerline.py`, `extract_raceline.py`, `mirror_centerline.py`, `convert_to_grayscale.py`) depend on a strict pin set mirroring [ETH race_stack](https://github.com/ForzaETH/race_stack)'s Docker stack: `trajectory_planning_helpers==0.76`, `scipy==1.6.3`, `quadprog==0.1.7`, `numpy==1.22.4`. None of these have wheels for Python 3.10+, which is why the parent simulator (poetry, Python 3.10–3.12) and `maps/` (pyenv, Python 3.8.18) live in entirely separate environments and never share dependencies.

Dependency management in `maps/`:
- `.python-version` → contents `maps`; pyenv-virtualenv auto-activates the venv whenever a shell is inside `maps/`
- `requirements.txt` → race_stack-style loose pins (with the critical ones — numpy, scipy, quadprog, tph — frozen)
- `requirements.lock.txt` → exact-version lockfile (preferred for reproducibility)
- **No `pyproject.toml`, no `poetry.lock`, no `setup.py`** — the directory is a script bundle, not a Python package. `pip install -e maps/` will fail by design; the correct flow is `cd maps && pip install -r requirements.lock.txt` inside the `maps` pyenv venv.

Full bootstrap (pyenv install, system apt deps, venv creation, smoke test) is documented in `maps/README.md` under the Install section. The short version: install `pyenv` + `pyenv-virtualenv`, install Ubuntu build deps (`build-essential libssl-dev libopenblas-dev gfortran ...`), `pyenv install 3.8.18`, `pyenv virtualenv 3.8.18 maps`, `cd maps`, `pip install -r requirements.lock.txt`.

**Known gotchas** (full troubleshooting in `maps/README.md`):
- `tph 0.76` hard-pins `quadprog==0.1.7`. If pip serves a stale locally-built wheel from cache, `import quadprog` raises `undefined symbol: _Z7qpgen2_...`. Fix: `pip install --no-cache-dir --force-reinstall --no-binary=quadprog quadprog==0.1.7`, then restore `cython<3` and `numpy==1.22.4` (which `--force-reinstall` will clobber).
- When regenerating the lockfile, **always** run `PYTHONPATH= pip freeze --local > requirements.lock.txt`. A sourced ROS environment otherwise leaks ~150 ROS2 packages (`ament-*`, `rclpy`, `nav2-*`, etc.) into the freeze, producing a lockfile that fails on every fresh install.

The post-training script `train/extract_global_track_norm_bounds.py` lives in the parent repo (not `maps/`) because it imports from `train.train_utils`.

## Wandb Integration

Models and training runs are logged to: https://wandb.ai/teo-altum-quinque-queen-s-university/projects

## Known Platform Issues

- **Windows**: Must use Python 3.8 due to compilation dependencies
- **macOS Big Sur+**: May need `pip3 install pyglet==1.5.11` for OpenGL framework compatibility (ignore gym version warning)
