Action System
=============

The action system handles different control input types and optional normalization.

Source: ``gymkhana/envs/action.py``

Action space
------------

Actions are an ``ndarray`` of shape ``(num_agents, 2)`` where each row is ``[steering, longitudinal]``:

- ``action[0]`` — steering command (angle or velocity)
- ``action[1]`` — longitudinal command (speed or acceleration)

Control input types
-------------------

``control_input`` is a list of two strings — one longitudinal type and one steering type, in any order:

**Longitudinal**:

``"speed"``
   Target velocity. Internally converted to acceleration via a P controller. Action space: ``[v_min, v_max]``.

``"accl"``
   Direct acceleration command. Action space: ``[-a_max, a_max]``. Recommended for RL drift training for smooth control.

**Steering**:

``"steering_angle"``
   Target steering angle. Internally converted to steering velocity via a bang-bang controller. Action space: ``[s_min, s_max]``.

``"steering_speed"``
   Direct steering velocity command. Action space: ``[sv_min, sv_max]``.

Common combinations:

- ``['speed', 'steering_angle']`` — default, suitable for general use and classical controllers
- ``['accl', 'steering_angle']`` — recommended for RL tasks

Configuration
-------------

.. code:: python

   config = {
       'control_input': ['accl', 'steering_angle'],
       'normalize_act': True,   # Normalize action space to [-1, 1]
   }

Normalization
-------------

When ``normalize_act`` is ``True``, the action space is normalized to ``[-1, 1]`` for each dimension. The mapping depends on the action type:

- **accl**: ``[-1, 1]`` maps to ``[-a_max, a_max]``
- **speed**: ``[-1, 1]`` maps to ``[v_min, v_max]``
- **steering_angle**: ``[-1, 1]`` maps to ``[s_min, s_max]``
- **steering_speed**: ``[-1, 1]`` maps to ``[sv_min, sv_max]``

Normalization is supported for all action types and is generally recommended for RL training.

API reference
-------------

.. automodule:: gymkhana.envs.action
   :members:
   :undoc-members:
