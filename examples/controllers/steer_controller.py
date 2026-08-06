"""
Simple path-tracking controllers: PD on the centerline, Stanley on the raceline
"""

from typing import Any

import numpy as np

from examples.controllers.base import Controller
from train.config.env_config import get_drift_test_config

# Path-tracking controller
FRENET_N_GAIN = 1.0  # Lateral deviation gain
FRENET_U_GAIN = 0.5  # Heading error gain

# Stability controller gains
BETA_GAIN = 1.0  # Sideslip angle gain
R_GAIN = 0.5  # Yaw rate gain

# Stanley controller constants
K_STANLEY = 1.0  # Cross-track gain
K_SOFT_STANLEY = 0.1  # Velocity softening constant [m/s]
K_HEADING_STANLEY = 2.0  # Heading error gain
STANLEY_FRENET_REFERENCE = "raceline"  # Stanley tracks the raceline; the PD controllers track the centerline

TARGET_SPEED = 2.0  # m/s

# config constants
LOOKAHEAD_N_POINTS = 10
LOOKAHEAD_DS = 0.3
OBS_TYPE = "drift_real"

# obs indices for state extraction
LINEAR_VEL_X_I = 0
FRENET_U_I = 2
FRENET_N_I = 3
R_I = 4
BETA_I = 5


class PDSteerController(Controller):
    """
    Path tracking controller.

    This controller minimized lateral deviation and heading error to track a path.
    """

    def __init__(
        self, Kn: float = FRENET_N_GAIN, Ku: float = FRENET_U_GAIN, target_speed: float = TARGET_SPEED, map: str = "IMS"
    ):
        self.Kn = Kn
        self.Ku = Ku
        self.target_speed = target_speed
        self.map = map

    def get_action(self, obs: np.ndarray) -> np.ndarray:
        frenet_u = obs[FRENET_U_I]
        frenet_n = obs[FRENET_N_I]
        steering_angle = self.compute_steering(frenet_n, frenet_u)
        action = np.array([[steering_angle, self.target_speed]], dtype=np.float32)

        return action

    def get_env_config(self) -> dict[str, Any]:
        return get_config(map=self.map)

    def compute_steering(self, frenet_n, frenet_u):
        steering_angle = -self.Kn * frenet_n - self.Ku * frenet_u
        return steering_angle


class PDStabilityController(Controller):
    """
    Pure Stability Controller for Vehicle Recovery

    This controller directly minimizes beta (sideslip angle) and r (yaw rate)
    to stabilize the vehicle.
    """

    def __init__(
        self, Kbeta: float = BETA_GAIN, Kr: float = R_GAIN, target_speed: float = TARGET_SPEED, map: str = "IMS"
    ):
        """
        Initialize the stability controller.

        Args:
            Kbeta: Gain for sideslip angle correction
            Kr: Gain for yaw rate correction
            target_speed: Constant target speed [m/s]
            map: Map name for environment configuration
        """
        self.Kbeta = Kbeta
        self.Kr = Kr
        self.target_speed = target_speed
        self.map = map

    def get_action(self, obs: np.ndarray) -> np.ndarray:
        beta = obs[BETA_I]
        r = obs[R_I]
        steering_angle = self.compute_steering(beta, r)
        action = np.array([[steering_angle, self.target_speed]], dtype=np.float32)

        return action

    def get_env_config(self) -> dict[str, Any]:
        return get_config(map=self.map)

    def compute_steering(self, beta: float, r: float) -> float:
        steering_angle = -self.Kbeta * beta - self.Kr * r
        return steering_angle


class StanleyController(Controller):
    """
    Stanley path tracking controller with speed-adaptive cross-track error correction.

    Developed at Stanford University, this controller combines heading error
    with speed-adaptive cross-track error correction, for aggressive low-speed corrections
    and gentle high-speed corrections:
        δ = k_heading * θ_e + arctan(k * e / (|v| + k_soft))

    Tuning:
    - Heading error correction (k_heading): How aggressively to align with path
    - Cross-track correction (k): How aggressively to correct lateral deviation
    """

    def __init__(
        self,
        k: float = K_STANLEY,
        k_soft: float = K_SOFT_STANLEY,
        k_heading: float = K_HEADING_STANLEY,
        target_speed: float = TARGET_SPEED,
        map: str = "IMS",
    ):
        """
        Initialize the Stanley controller.

        Args:
            k: Cross-track gain
            k_soft: Velocity softening constant [m/s]
            k_heading: Heading error gain
            target_speed: Constant target speed [m/s]
            map: Map name for environment configuration
        """
        self.k = k
        self.k_soft = k_soft
        self.k_heading = k_heading
        self.target_speed = target_speed
        self.map = map

    def get_action(self, obs: np.ndarray) -> np.ndarray:
        vx = obs[LINEAR_VEL_X_I]
        heading_error = obs[FRENET_U_I]
        cross_track_error = obs[FRENET_N_I]
        steering_angle = self.compute_steering(vx, heading_error, cross_track_error)
        action = np.array([[steering_angle, self.target_speed]], dtype=np.float32)

        return action

    def get_env_config(self) -> dict[str, Any]:
        return get_config(map=self.map, frenet_reference=STANLEY_FRENET_REFERENCE)

    def compute_steering(self, vx: float, heading_error: float, cross_track_error: float) -> float:
        """
        Compute Stanley steering control law
        """
        # Independent heading error correction
        heading_term = self.k_heading * heading_error

        # Speed-adaptive cross-track correction with natural saturation
        cross_track_term = np.arctan(self.k * cross_track_error / (abs(vx) + self.k_soft))

        # Combine with separate gains
        steering_angle = -heading_term - cross_track_term

        return steering_angle


def get_config(
    obs_type=OBS_TYPE,
    lookahead_n_points=LOOKAHEAD_N_POINTS,
    lookahead_ds=LOOKAHEAD_DS,
    map="Drift",
    frenet_reference="centerline",
):
    """
    Helper function to create steering controlelrs

    frenet_reference selects the path to track by setting which line frenet_u/frenet_n are
    measured against. "raceline" requires a map that ships a <map>_raceline.csv.
    """
    config = get_drift_test_config()
    config["map"] = map
    config["control_input"] = ["speed", "steering_angle"]
    config["observation_config"] = {"type": obs_type, "frenet_reference": frenet_reference}
    config["normalize_act"] = False
    config["normalize_obs"] = False
    config["predictive_collision"] = False
    config["wall_deflection"] = False
    config["render_lookahead_curvatures"] = True
    config["render_track_lines"] = True
    config["debug_frenet_projection"] = False
    config["lookahead_n_points"] = lookahead_n_points
    config["lookahead_ds"] = lookahead_ds
    config["record_obs_min_max"] = False
    config["render_arc_length_annotations"] = True
    config["track_direction"] = "normal"
    return config
