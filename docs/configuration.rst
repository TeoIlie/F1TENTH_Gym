.. image:: assets/logo.png
   :width: 20
   :align: left
   :alt: logo

Configuration
=============

All configuration is passed through the ``config`` dictionary to ``gym.make()``.

Default configurations
----------------------

For PyPI users (or anyone who wants a one-liner starter config), the package exposes a ``drift_config`` preset bundling sensible STD-model drift defaults — physics (``model='std'``, ``integrator='rk4'``, ``timestep=0.01``), control (``control_input=['accl', 'steering_angle']``), observation (``observation_config={'type': 'drift'}``), normalization, lookahead sampling, Frenet boundary checking, and STD vehicle parameters.

.. code:: python

   import gymnasium as gym
   from gymkhana import drift_config

   # Defaults out of the box — pick a map and you have a working drift env.
   env = gym.make("gymkhana:gymkhana-v0", config=drift_config(map="Drift"))
   obs, info = env.reset()
   action = env.action_space.sample()
   obs, reward, terminated, truncated, info = env.step(action)
   env.close()

Keyword args override individual fields without touching the rest:

.. code:: python

   # Custom lookahead sampling, swap to drift-bias tyre params, enable wall deflection.
   from gymkhana.envs.gymkhana_env import GKEnv

   cfg = drift_config(
       map="Drift",
       lookahead_n_points=8,
       lookahead_ds=0.4,
       params=GKEnv.f1tenth_std_drift_bias_params(),
       wall_deflection=True,
   )
   env = gym.make("gymkhana:gymkhana-v0", config=cfg)

The preset is a plain ``dict``, so you can also spread it and add other keys (e.g. ``render_config`` from the rendering section below):

.. code:: python

   env = gym.make("gymkhana:gymkhana-v0", config={
       **drift_config(map="Drift"),
       "render_config": {"window_size": 1200, "show_obs_debug": True},
   })

For training workflows, full configurations live in ``train/config/env_config.py``, with parameters loaded from:

- ``train/config/rl_config.yaml`` — RL hyperparameters (n_envs, learning rate, batch size, etc.)
- ``train/config/gym_config.yaml`` — Gym environment parameters (map, model, timestep, etc.)

Convenience functions return ready-made configs (built on top of ``drift_config`` with training-specific keys layered on):

- ``get_drift_test_config()`` / ``get_drift_train_config()``
- ``get_recovery_test_config()`` / ``get_recovery_train_config()``

Vehicle model and control
-------------------------

.. code:: python

   env = gym.make('gymkhana:gymkhana-v0', config={
       'model': 'std',                              # STD for drifting with PAC2002 tire model
       'params': GKEnv.f1tenth_std_vehicle_params(),  # Drift parameters for 1/10 scale
   })

.. note::

   Available models: ``ks`` (kinematic), ``st`` (single-track), ``stp`` (single-track Pacejka, lateral-only Magic Formula), ``std`` (single-track drift, PAC2002), ``mb`` (multi-body). Use ``std`` for drift training. See :doc:`api/dynamic_models` for details.

For action space configuration (``control_input``, ``normalize_act``), see :doc:`api/action`.

Training mode
-------------

Set ``training_mode`` to define the training goal. This modifies the reset, initialization, track, and reward settings:

- ``"race"`` (default) — used by ``train/ppo_race.py`` for training racing policies
- ``"recover"`` — used by ``train/ppo_recover.py`` to train policies for stabilizing an out-of-control vehicle

Track direction
---------------

Set ``track_direction`` to control which direction to drive around the track:

- ``"normal"`` (default) — follow waypoint direction (may be CW or CCW depending on the map)
- ``"reverse"`` — drive in the opposite direction
- ``"random"`` — randomly choose direction at each reset (50/50), useful for learning both left and right cornering

Observation configuration
-------------------------

.. code:: python

   config = {
       'observation_config': {'type': 'drift'},  # or 'rl' for standard observations
       'lookahead_n_points': 10,                  # Number of lookahead points (default: 10)
       'lookahead_ds': 0.3,                       # Spacing between points in meters (default: 0.3m)
       'sparse_width_obs': False,                 # False: all widths, True: only 1st and last
       'normalize_obs': True,                     # Enable observation normalization (only 'drift' obs)
   }

See :doc:`api/observation` for details.

Collision and safety
--------------------

.. code:: python

   config = {
       'predictive_collision': True,   # True: TTC collision checking, False: Frenet-based
       'wall_deflection': False,       # False: edges are boundaries, True: edges cause collision
   }

Note that ``predictive_collision`` also modifies the reward function.

Reward configuration
--------------------

.. code:: python

   config = {
       'progress_gain': 1.0,          # Gain multiplier for forward progress reward (>= 1)
       'out_of_bounds_penalty': ...,   # Penalty for driving off the track boundary
       'negative_vel_penalty': ...,    # Penalty for driving backward
       'max_episode_steps': ...,       # Maximum number of episode steps
   }

Reset options
-------------

Vehicles can be initialized at specific configurations via ``env.reset(options=...)``.

**Poses** — reset agents at a specific ``[x, y, yaw]`` in Cartesian coordinates:

.. code:: python

   # Single agent
   poses = np.array([[x, y, yaw]])
   env.reset(options={"poses": poses})

   # Multiple agents
   poses = np.array([[x1, y1, yaw1],
                     [x2, y2, yaw2]])
   env.reset(options={"poses": poses})

**States** — reset agents to a full 7-d state (only for ``model='std'``):

.. code:: python

   # [x, y, delta, v, yaw, yaw_rate, slip_angle]
   states = np.array([[x, y, delta, v, yaw, yaw_rate, slip_angle]])
   env.reset(options={"states": states})

   # Front & rear angular wheel velocities are automatically initialized
   # to form the full 9-d state for the STD model type

Only one of ``poses`` or ``states`` can be used per reset call. To use Frenet coordinates, convert first using ``frenet_to_cartesian()`` in ``gymkhana/envs/track/track.py``.

Debugging and visualization
---------------------------

.. code:: python

   config = {
       'render_mode': 'human',                    # Enable visualization window
       'render_track_lines': True,                # Centerline (green) and raceline (red)
       'render_lookahead_curvatures': True,       # Lookahead curvature points (yellow)
       'render_arc_length_annotations': True,     # Arc-length points along centerline (orange)
       'arc_length_annotation_interval': 2.0,     # Spacing in metres (default: 2.0)
       'debug_frenet_projection': True,           # Visualize Frenet coordinate accuracy
       'record_obs_min_max': True,                # Record min/max obs for normalization tuning (aggregated across parallel envs; works with normalize_obs on or off). During training, periodic YAML snapshots are written to outputs/config/<run_id>/obs_min_max.yaml and per-feature bounds-violation magnitudes are streamed to wandb under obs_bounds/<feature>/{over,under}
       'prevent_instability': True,               # Sanity-check post-RK4 standardized state; on blow-up, revert agent state and truncate the episode. Cumulative event count is logged to wandb (instability/total) and printed at end-of-run. Disable to recover pre-feature behavior
       'instability_yaw_rate_bound': 12.566,      # |yaw_rate| bound used by the sanity check (default 4*pi rad/s)
       'instability_slip_bound': 1.5708,          # |slip| bound used by the sanity check (default pi/2 rad)
   }

Debug with breakpoints by looping through environment steps (see ``tests/drift_debug.py``).

Rendering configuration
^^^^^^^^^^^^^^^^^^^^^^^

Renderer fields (window size, render backend, debug panel toggles, vehicle palette, ...) are loaded from the packaged ``gymkhana/envs/rendering/rendering.yaml``. To override individual fields without editing the packaged file (essential for PyPI users), pass ``render_config`` in the ``gym.make`` config:

.. code:: python

   env = gym.make('gymkhana:gymkhana-v0', config={
       'render_config': {
           'window_size': 1200,
           'render_type': 'pygame',     # 'pygame' or 'pyqt6'
           'show_ctr_debug': True,
           'show_obs_debug': True,
       },
   })

Any field omitted from ``render_config`` keeps its packaged-yaml default.

Control debug panel
^^^^^^^^^^^^^^^^^^^

Set ``show_ctr_debug: True`` via ``render_config`` (or in ``gymkhana/envs/rendering/rendering.yaml`` for repo users) to enable a real-time control debug panel below the map (PyQt6 renderer only). The panel displays:

- **Actual vehicle state** (white text): current steering angle (``delta``) and longitudinal velocity (``v_x``)
- **Control commands** (colour-coded text): raw steering command in blue, throttle command in green, each with their bounds
- **Zero-centered bar gauges**: two horizontal bars (one per command) spanning the full command range, with the fill extending from zero toward the current value

The panel tracks whichever agent the camera is following (switched via mouse click), defaulting to the ego agent in map view. Disabled by default to avoid overhead during RL training.

Observation debug overlay
^^^^^^^^^^^^^^^^^^^^^^^^^

Set ``show_obs_debug: True`` via ``render_config`` (or in ``gymkhana/envs/rendering/rendering.yaml`` for repo users) to overlay all observation values on top of the map in the top-left corner (PyQt6 renderer only). The overlay displays:

- **Feature names and values**: each observation feature as a key-value pair (e.g., ``linear_vel_x: 2.3451``)
- **Array summaries**: large arrays like LiDAR scans show count, min, max, and mean; small arrays (e.g., lookahead curvatures) show all values
- **Normalization indicator**: shows ``[norm: on]`` when observation normalization is active; values are always displayed in raw physical units regardless of normalization

Works with all observation types (``OriginalObservation``, ``FeaturesObservation``, ``VectorObservation``). For multi-agent environments, the overlay shows the followed agent's observations. Disabled by default to avoid overhead during RL training.

Custom maps
-----------

Maps follow the ROS map convention: a ``.yaml`` metadata file and a single-channel black-and-white image (black = obstacles, white = free space).

The ``resolution`` field (m/pixel) and the first two values of ``origin`` (bottom-left corner in world coordinates) are used by the environment. Place both files in the same directory with the same base name, then:

.. code:: python

   env = gym.make('gymkhana:gymkhana-v0', config={
       'map': '/your/path/to/map',  # without extension
       'map_ext': '.png',
   })

Maps are managed through a two-tier system:

- **Local** ``maps/`` **directory**: git submodule from https://github.com/TeoIlie/F1TENTH_Racetracks — used for development
- **User cache** ``~/.gymkhana/maps/``: auto-downloaded from GitHub releases for pip-installed users

To update the maps submodule: ``git pull --recurse-submodules``
