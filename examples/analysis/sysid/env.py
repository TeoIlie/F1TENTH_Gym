"""Shared anchor for the STD parameter dict used across the sysid module.

Both `dataset.py` (needs `R_w` to convert VESC `rs_core_speed` [m/s] into
wheel angular speed [rad/s] for the 9-wide `init_state`) and `rollout.py`
(passes the dict to `GKEnv` as `config["params"]`) must reference the
**same** `R_w`. If they diverge, every reset spawns the sim at
`omega ≠ v/R_w` and tire params silently absorb the synthetic mismatch.
"""

from gymkhana.envs.gymkhana_env import GKEnv

SYSID_PARAMS: dict = GKEnv.f1tenth_std_vehicle_params()
