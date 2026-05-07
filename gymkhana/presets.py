"""Public env-config presets for common Gym-Khana workflows.

These are starter configs intended for users (especially PyPI users who don't
have access to ``train/config/``). Pass the returned dict into
``gym.make("gymkhana:gymkhana-v0", config=...)``. Any keyword overrides
provided to the preset functions take precedence over the bundled defaults.
"""

from .envs.gymkhana_env import GKEnv


def drift_config(**overrides) -> dict:
    """Return a sensible starter config for STD-model drift environments.

    Bundles the physics, control, observation, normalization, and reset
    settings used for drift training. The user picks the map (and any other
    workflow-specific knobs) via ``overrides``::

        env = gym.make("gymkhana:gymkhana-v0", config=drift_config(map="Drift"))

    Args:
        **overrides: keys merged on top of the preset defaults (shallow update).

    Returns:
        Config dict ready to pass to ``gym.make``.
    """
    cfg = {
        "model": "std",
        "integrator": "rk4",
        "timestep": 0.01,
        "num_agents": 1,
        "control_input": ["accl", "steering_angle"],
        "observation_config": {"type": "drift"},
        "reset_config": {"type": "cl_random_static"},
        "lookahead_n_points": 5,
        "lookahead_ds": 0.5,
        "normalize_obs": True,
        "normalize_act": True,
        "predictive_collision": False,  # Frenet-based boundary checking (drift default)
        "wall_deflection": False,  # treat track edges as boundaries, not walls
        "params": GKEnv.f1tenth_std_vehicle_params(),
    }
    cfg.update(overrides)
    return cfg
