Observation System
==================

The observation system uses a factory pattern to provide different observation types for different use cases.

Source: ``gymkhana/envs/observation.py``

Observation types
-----------------

``"rl"``
   Standard RL observations suitable for general racing tasks.

``"drift"``
   Extended observations including vehicle slip angle, yaw rate, and other states relevant to drift control. Supports normalization.

Configuration
-------------

.. code:: python

   config = {
       'observation_config': {'type': 'drift'},
       'lookahead_n_points': 10,     # Number of lookahead curvature/width points (default: 10)
       'lookahead_ds': 0.3,          # Spacing between lookahead points in metres (default: 0.3)
       'sparse_width_obs': False,    # False: all width values, True: only 1st and last
       'normalize_obs': True,        # Enable observation normalization (only for 'drift' type)
   }

Lookahead observations
----------------------

The observation can include curvature and track width samples at points ahead of the vehicle along the track centerline. The number of points and their spacing are configurable.

When ``sparse_width_obs`` is ``True``, only the first and last lookahead width values are included in the observation. This is useful when track width varies very little.

Normalization
-------------

When ``normalize_obs`` is ``True``, observation values are scaled to a bounded range. Normalization bounds are defined in the codebase and can be tuned by enabling ``record_obs_min_max`` in the config to record actual min/max values during training runs. With ``record_obs_min_max`` enabled, training runs also write periodic snapshots to ``outputs/config/<run_id>/obs_min_max.yaml`` and stream per-feature bounds-violation magnitudes to wandb (see :doc:`../training`).

API reference
-------------

.. automodule:: gymkhana.envs.observation
   :members:
   :undoc-members:
