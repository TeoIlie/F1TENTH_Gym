.. image:: assets/logo.png
   :width: 20
   :align: left
   :alt: logo

Architecture
============

Package structure
-----------------

::

   gymkhana/                  Main package (Gymnasium RL environment)
   ├── envs/
   │   ├── gymkhana_env.py    GKEnv — main environment class
   │   ├── base_classes.py    RaceCar, Simulator core classes
   │   ├── observation.py     Observation factory (rl, drift types)
   │   ├── action.py          Action types and normalization
   │   ├── laser_models.py    LiDAR ray-casting simulation
   │   ├── collision_models.py  Collision detection (TTC, Frenet)
   │   ├── dynamic_models/    Vehicle dynamics models
   │   ├── params/            Vehicle parameter YAML files (canonical source)
   │   ├── track/             Track loading, Frenet conversion
   │   ├── reset/             Reset strategies for RL training
   │   └── rendering/         Pygame/PyQt visualization
   examples/                  Reference implementations (Pure Pursuit, etc.)
   train/                     RL training scripts and configuration
   ├── config/                env_config.py, rl_config.yaml, gym_config.yaml
   ├── ppo_race.py            Racing training script
   ├── ppo_recover.py         Recovery training script
   ├── train_common.py        Shared training workflow
   └── train_utils.py         Shared utilities
   tests/                     Test suite including model validation
   maps/                      Racing track definitions (git submodule)

Important files
---------------

- ``gymkhana/envs/base_classes.py:503`` defines the ``step`` method
- See :doc:`api/action` for action space details
- ``gymkhana/envs/dynamic_models/single_track.py`` — basic single-track dynamics without explicit tire model
- ``gymkhana/envs/dynamic_models/single_track_pacejka/`` — STP model: dynamic single-track with lateral-only Pacejka Magic Formula (ported from ForzaETH f110-simulator)
- ``gymkhana/envs/dynamic_models/single_track_drift/`` — STD model with PAC2002 tire model, recommended for drift RL training
- ``gymkhana/envs/dynamic_models/multi_body/`` — most detailed model, but parameters only available for full-scale vehicles

Vehicle parameters
------------------

Vehicle parameters are defined in YAML files in ``gymkhana/envs/params/``:

- ``f1tenth_st.yaml`` — 1/10 scale F1TENTH car (ST model)
- ``f1tenth_stp.yaml`` — 1/10 scale F1TENTH car (STP model, lateral-only Pacejka)
- ``f1tenth_std.yaml`` — 1/10 scale F1TENTH with PAC2002 tire model (STD model for drifting)
- ``f1tenth_std_drift_bias.yaml`` — STD model tuned for increased drift tendency
- ``f1fifth.yaml`` — 1/5 scale F1FIFTH car
- ``fullscale.yaml`` — Full-scale vehicle from CommonRoad

Access parameters via class methods on ``GKEnv``::

   GKEnv.f1tenth_vehicle_params()      # ST model params
   GKEnv.f1tenth_stp_vehicle_params()  # STP model params (lateral-only Pacejka)
   GKEnv.f1tenth_std_vehicle_params()  # STD model params (recommended for drift)
   GKEnv.f1fifth_vehicle_params()      # 1/5 scale params
   GKEnv.fullscale_vehicle_params()    # Full-scale params

Test script ``tests/model_validation/test_f1tenth_std_params.py`` creates comparison figures and parameter YAML dumps in ``figures/tire_params/`` for validation.

Key dependencies
----------------

- **gymnasium** — RL environment interface
- **stable-baselines3** — PPO and other RL algorithms
- **wandb** — experiment tracking and model logging
- **numba** — JIT compilation for real-time physics
- **pygame / PyQt6** — real-time visualization
- **numpy / scipy** — control and estimation math
