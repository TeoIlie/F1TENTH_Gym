Gym Environment
===============

``GKEnv`` is the main Gymnasium environment class, implementing the F1/10th vehicle simulation with realistic dynamics.

Source: ``gymkhana/envs/gymkhana_env.py``

Creating an environment
-----------------------

.. code:: python

   import gymnasium as gym
   from gymkhana.envs.gymkhana_env import GKEnv

   env = gym.make('gymkhana:gymkhana-v0', config={
       'map': 'Spielberg',
       'num_agents': 1,
       'model': 'std',
       'control_input': ['accl', 'steering_angle'],
       'params': GKEnv.f1tenth_std_vehicle_params(),
       'render_mode': 'human',
   })

See :doc:`../configuration` for the full list of config options.

Vehicle parameter methods
-------------------------

``GKEnv.f1tenth_std_vehicle_params()``
   Returns the default parameter dictionary for the 1/10 scale F1TENTH car with the STD (single-track drift) model. Includes PAC2002 tire parameters adjusted from a full-scale car.

``GKEnv.f1tenth_std_drift_bias_params()``
   Returns drift-biased parameters for more aggressive drifting behavior.

Step and reset
--------------

``env.step(action)``
   Steps the simulation. Returns ``(obs, reward, terminated, truncated, info)``.

   - ``action``: ``ndarray`` of shape ``(num_agents, 2)`` where each row is ``[steering, longitudinal]`` — see :doc:`../api/action` for control input types

``env.reset(options=None)``
   Resets the environment. Returns ``(obs, info)``.

   - ``options``: optional dict with ``"poses"`` or ``"states"`` keys (see :doc:`../configuration`)

``env.update_params(params_dict, index=None)``
   Update vehicle parameters. If ``index`` is specified, updates only that vehicle; otherwise updates all vehicles.

API reference
-------------

.. autoclass:: gymkhana.envs.gymkhana_env.GKEnv
   :members:
   :undoc-members:
