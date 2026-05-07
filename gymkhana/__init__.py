__version__ = "1.2.0"

import gymnasium as gym

from .presets import drift_config

gym.register(
    id="gymkhana-v0",
    entry_point="gymkhana.envs:GKEnv",
)

__all__ = ["drift_config"]
