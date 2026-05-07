"""
Centralized F1TENTH Gym Environment Configuration
"""

import multiprocessing
import os

import yaml

from gymkhana.envs.gymkhana_env import GKEnv
from gymkhana.presets import drift_config

# ====================================
# RL config
# ====================================
_rl_config_path = os.path.join(os.path.dirname(__file__), "rl_config.yaml")
with open(_rl_config_path, "r") as f:
    _rl_config = yaml.safe_load(f)

# RL training parameters
N_ENVS = _rl_config["core_mult"] * multiprocessing.cpu_count()  # CPU core count * multiplier
TOTAL_TIMESTEPS = _rl_config["total_timesteps"]
N_STEPS = _rl_config["n_steps"]
START_LEARNING_RATE = _rl_config["start_learning_rate"]
END_LEARNING_RATE = _rl_config["end_learning_rate"]
SEED = _rl_config["seed"]
EVAL_SEED = _rl_config["eval_seed"]
ACT_FUNC_NEG_SLOPE = _rl_config["act_func_neg_slope"]
USE_CUSTOM_RELU = _rl_config["use_custom_relu"]
ACTOR_LAYER_SIZE = _rl_config["actor_layer_size"]
CRITIC_LAYER_SIZE = _rl_config["critic_layer_size"]
ADDITIONAL_TIMESTEPS = _rl_config["additional_timesteps"]
TRANSFER_RESET_LOG_STD = _rl_config["transfer_reset_log_std"]
TRANSFER_RESET_CRITIC = _rl_config["transfer_reset_critic"]


# ====================================
# Gym config
# ====================================
_config_path = os.path.join(os.path.dirname(__file__), "gym_config.yaml")
with open(_config_path, "r") as f:
    _config = yaml.safe_load(f)

# Gym shared parameters. Defaults from the public ``drift_config`` preset are not duplicated
PROJECT_NAME = _config["project_name"]
RACE_TRAINING_MODE = _config["race_training_mode"]
MAP = _config["map"]
TRACK_POOL = _config["track_pool"]
TRACK_DIRECTION = _config["track_direction"]
NUM_BEAMS = _config["num_beams"]  # training-only: minimal LiDAR beams to save compute
SPARSE_WIDTH_OBS = _config["sparse_width_obs"]
RECORD_OBS_MIN_MAX = _config["record_obs_min_max"]
PREVENT_INSTABILITY = _config["prevent_instability"]

# Vehicle parameters
PARAMS = GKEnv.f1tenth_std_drift_bias_params()

# Render/debug params
TEST_DEBUG_RENDER = _config["test_debug_render"]
TRAIN_DEBUG_RENDER = _config["train_debug_render"]

# Callback config
CKPT_SAVE_FREQ = _config["ckpt_save_freq"]
N_EVAL_EPISODES = _config["n_eval_episodes"]
BEST_MODEL = "best_model"

# Recovery-specific configs
RECOVERY_PROJECT_NAME = _config["recovery_project_name"]
RECOVERY_TRAINING_MODE = _config["recovery_training_mode"]
RECOVERY_TRACK_POOL = _config["recovery_track_pool"]

# Curriculum learning config
CURRICULUM_CONFIG = _config.get("curriculum", {})


# ====================================
# Gym config functions
# ===================================
def get_env_id():
    return "gymkhana:gymkhana-v0"


def _base_config(debug_render):
    """
    Returns environment params shared across all environments. Inherits all
    drift defaults from the public ``drift_config`` preset and overrides only:

    - per-experiment knobs (``params``, training-only ``num_beams``)
    - training-workflow-only keys (debug renders, telemetry, instability
      checks, ``sparse_width_obs``)
    """
    return drift_config(
        params=PARAMS,
        num_beams=NUM_BEAMS,
        # training-workflow-specific keys not in drift_config
        render_lookahead_curvatures=debug_render,
        debug_frenet_projection=debug_render,
        render_track_lines=debug_render,
        render_arc_length_annotations=debug_render,
        sparse_width_obs=SPARSE_WIDTH_OBS,
        record_obs_min_max=RECORD_OBS_MIN_MAX,
        prevent_instability=PREVENT_INSTABILITY,
    )


def _drift_overrides():
    """
    Drift/race-specific overrides
    """
    return {
        "training_mode": RACE_TRAINING_MODE,
        "map": MAP,
        "track_pool": TRACK_POOL,
        "track_direction": TRACK_DIRECTION,
    }


def _recovery_overrides():
    """
    Recovery-specific overrides. When curriculum learning is enabled, the recovery ranges
    are set to the curriculum's max_lo/max_hi values.
    """
    overrides = {
        "training_mode": RECOVERY_TRAINING_MODE,
        "track_pool": RECOVERY_TRACK_POOL,
    }

    if CURRICULUM_CONFIG.get("enabled", False):
        for gym_key, curriculum_key in [
            ("recovery_v_range", "v_range"),
            ("recovery_beta_range", "beta_range"),
            ("recovery_r_range", "r_range"),
            ("recovery_yaw_range", "yaw_range"),
        ]:
            vals = CURRICULUM_CONFIG[curriculum_key]
            overrides[gym_key] = [vals[2], vals[3]]  # [max_lo, max_hi]

    return overrides


def get_drift_test_config():
    """
    Returns gym drift TESTING environment config
    """
    return {**_base_config(TEST_DEBUG_RENDER), **_drift_overrides()}


def get_drift_train_config():
    """
    Returns gym drift TRAINING environment config
    """
    return {**_base_config(TRAIN_DEBUG_RENDER), **_drift_overrides()}


def get_recovery_test_config():
    """
    Returns gym recovery TESTING environment config
    """
    return {**_base_config(TEST_DEBUG_RENDER), **_recovery_overrides()}


def get_recovery_train_config():
    """
    Returns gym recovery TRAINING environment config
    """
    return {**_base_config(TRAIN_DEBUG_RENDER), **_recovery_overrides()}


def get_curriculum_config():
    """Returns curriculum learning config dict (empty dict if not configured)."""
    return CURRICULUM_CONFIG
