import warnings
from typing import Any, Dict, TypeVar

import numpy as np

KeyType = TypeVar("KeyType")

# Global track bounds for normalization, computed across all available tracks.
# When adding a new custom track, regenerate these by running:
#     python train/extract_global_track_norm_bounds.py
# then paste the printed GLOBAL_* values below.
GLOBAL_MAX_CURVATURE = 1.95
GLOBAL_MIN_WIDTH = 1.2
GLOBAL_MAX_WIDTH = 2.2


def deep_update(mapping: Dict[KeyType, Any], *updating_mappings: Dict[KeyType, Any]) -> Dict[KeyType, Any]:
    """
    Dictionary deep update for nested dictionaries from pydantic:
    https://github.com/pydantic/pydantic/blob/fd2991fe6a73819b48c906e3c3274e8e47d0f761/pydantic/utils.py#L200
    """
    updated_mapping = mapping.copy()
    for updating_mapping in updating_mappings:
        for k, v in updating_mapping.items():
            if k in updated_mapping and isinstance(updated_mapping[k], dict) and isinstance(v, dict):
                updated_mapping[k] = deep_update(updated_mapping[k], v)
            else:
                updated_mapping[k] = v
    return updated_mapping


def calculate_norm_bounds(env, features: list[str], frenet_reference: str = "centerline"):
    """
    Calculate normalization bounds for requested observation features.

    Extracts bounds from three sources:
    1. Vehicle parameters (velocity limits, control limits, wheel dynamics)
    2. Track geometry (width, curvature) when available
    3. Physical constants (angle wrapping)

    All features are normalized to [-1, 1] for optimal neural network training.

    Parameters
    ----------
    env : GKEnv
        The environment to extract parameters from
    features : list[str]
        List of feature names that require normalization bounds
    frenet_reference : str
        Line that 'frenet_n' is measured against, "centerline" (default) or "raceline".
        Only affects the 'frenet_n' bound; see Notes.

    Returns
    -------
    dict
        Mapping of requested feature names to (min, max) tuples for normalization

    Raises
    ------
    ValueError
        If any requested feature is unknown or required parameters are missing

    Notes
    -----
    - Only calculates bounds for features in the provided list
    - ValueError raised if required params are missing for requested features
    - Track-dependent bounds require track/centerline to be available
    - Asymmetric physical bounds (e.g., wheel speed ≥ 0) still map to [-1, 1] for best DRL training
    - 'frenet_n' bounds differ by reference line, so a normalized frenet_n is only comparable
      across runs sharing the same reference
    """
    params = env.params
    bounds = {}
    features_set = set(features)  # For efficient lookup

    # ===========================
    # 1. VELOCITY BOUNDS
    # ===========================

    # Longitudinal velocity: [v_min, v_max]
    if (
        "linear_vel_x" in features_set
        or "integrated_vel_cmd" in features_set
        or "linear_vel_y" in features_set
        or "raceline_vxs" in features_set
    ):
        v_max = params.get("v_max", None)  # m/s
        v_min = params.get("v_min", None)  # m/s

        if v_max is None or v_min is None:
            raise ValueError(
                "Features 'linear_vel_x', 'linear_vel_y', 'integrated_vel_cmd' and 'raceline_vxs' require "
                "'v_max' and 'v_min' parameters. Please ensure these are configured in env.params."
            )

        if "linear_vel_x" in features_set:
            bounds["linear_vel_x"] = (v_min, v_max)
        if "integrated_vel_cmd" in features_set:
            bounds["integrated_vel_cmd"] = (v_min, v_max)
        # Same bound as linear_vel_x on purpose: a controller subtracts the measured speed from
        # this reference, so the two must share a scale once normalized.
        if "raceline_vxs" in features_set:
            bounds["raceline_vxs"] = (v_min, v_max)
        if "linear_vel_y" in features_set:
            linear_vel_bound = 0.5 * v_max
            bounds["linear_vel_y"] = (-linear_vel_bound, linear_vel_bound)

    # Yaw rate: experimentally determined bound
    if "ang_vel_z" in features_set:
        experimental_max_yaw = 5.0  # rad/s, found experimentally in sim
        bounds["ang_vel_z"] = (-experimental_max_yaw, experimental_max_yaw)

    # ===========================
    # 2. CONTROL INPUT BOUNDS
    # ===========================

    # Steering angle: [s_min, s_max]
    if "delta" in features_set:
        s_min = params.get("s_min", None)
        s_max = params.get("s_max", None)
        if s_min is None or s_max is None:
            raise ValueError("Feature 'delta' requires 's_min' and 's_max' parameters. Please configure in env.params.")
        bounds["delta"] = (s_min, s_max)

    # Cached once; reused by both cmd blocks below in their unnormalized branches.
    steer_type, long_type = env.unwrapped.action_type.type

    # {prev,curr}_steering_cmd: bounds are [-1,1] under normalize_act, else physical bounds
    # of the active steering action (angle or speed). prev and curr share identical bounds.
    if "prev_steering_cmd" in features_set or "curr_steering_cmd" in features_set:
        if env.unwrapped.normalize_act:
            steer_cmd_bounds = (-1.0, 1.0)
        elif steer_type == "steering_angle":
            s_min = params.get("s_min", None)
            s_max = params.get("s_max", None)
            if s_min is None or s_max is None:
                raise ValueError(
                    "Features 'prev_steering_cmd' / 'curr_steering_cmd' under 'steering_angle' control "
                    "require 's_min' and 's_max'."
                )
            steer_cmd_bounds = (s_min, s_max)
        elif steer_type == "steering_speed":
            sv_min = params.get("sv_min", None)
            sv_max = params.get("sv_max", None)
            if sv_min is None or sv_max is None:
                raise ValueError(
                    "Features 'prev_steering_cmd' / 'curr_steering_cmd' under 'steering_speed' control "
                    "require 'sv_min' and 'sv_max'."
                )
            steer_cmd_bounds = (sv_min, sv_max)
        else:
            raise ValueError(
                f"Unknown steering action type '{steer_type}' for prev_steering_cmd / curr_steering_cmd bounds."
            )
        if "prev_steering_cmd" in features_set:
            bounds["prev_steering_cmd"] = steer_cmd_bounds
        if "curr_steering_cmd" in features_set:
            bounds["curr_steering_cmd"] = steer_cmd_bounds

    # {prev,curr}_throttle_cmd: bounds are [-1,1] under normalize_act, else (-a_max, a_max)
    # in accl mode or (v_min, v_max) in speed mode. prev and curr share identical bounds.
    if "prev_throttle_cmd" in features_set or "curr_throttle_cmd" in features_set:
        if env.unwrapped.normalize_act:
            throttle_cmd_bounds = (-1.0, 1.0)
        elif long_type == "accl":
            a_max = params.get("a_max", None)
            if a_max is None:
                raise ValueError(
                    "Features 'prev_throttle_cmd' / 'curr_throttle_cmd' under 'accl' control require 'a_max'."
                )
            throttle_cmd_bounds = (-a_max, a_max)
        elif long_type == "speed":
            v_min = params.get("v_min", None)
            v_max = params.get("v_max", None)
            if v_min is None or v_max is None:
                raise ValueError(
                    "Features 'prev_throttle_cmd' / 'curr_throttle_cmd' under 'speed' control "
                    "require 'v_min' and 'v_max'."
                )
            throttle_cmd_bounds = (v_min, v_max)
        else:
            raise ValueError(
                f"Unknown longitudinal action type '{long_type}' for prev_throttle_cmd / curr_throttle_cmd bounds."
            )
        if "prev_throttle_cmd" in features_set:
            bounds["prev_throttle_cmd"] = throttle_cmd_bounds
        if "curr_throttle_cmd" in features_set:
            bounds["curr_throttle_cmd"] = throttle_cmd_bounds

    # ===========================
    # 3. WHEEL DYNAMICS BOUNDS
    # ===========================

    # Wheel angular velocity: depends on wheel radius
    if "prev_avg_wheel_omega" in features_set or "curr_avg_wheel_omega" in features_set:
        R_w = params.get("R_w", None)
        v_max = params.get("v_max", None)

        if R_w is None or v_max is None:
            raise ValueError(
                "Features 'prev_avg_wheel_omega' or 'curr_avg_wheel_omega' require 'R_w' and 'v_max' parameters. "
                "Please configure in env.params."
            )

        if R_w <= 0:
            raise ValueError(f"Wheel radius 'R_w' must be positive, got {R_w}")

        omega_max = v_max / R_w  # rad/s
        experimental_gain = 6.4  # True max wheel omega found experimentally to be ~2610
        omega_bounds = (0.0, omega_max * experimental_gain)
        if "prev_avg_wheel_omega" in features_set:
            bounds["prev_avg_wheel_omega"] = omega_bounds
        if "curr_avg_wheel_omega" in features_set:
            bounds["curr_avg_wheel_omega"] = omega_bounds

    # ===========================
    # 4. PHYSICAL CONSTANTS
    # ===========================

    # Heading error: wraps at +/- pi
    if "frenet_u" in features_set:
        bounds["frenet_u"] = (-np.pi, np.pi)

    # Slip angle: conservative bounds for typical drift ranges (±60°)
    if "beta" in features_set:
        bounds["beta"] = (-np.pi / 3, np.pi / 3)

    # ===========================
    # 5. TRACK-DEPENDENT BOUNDS
    # ===========================

    # Check if any track-dependent features are requested
    track_features = {"frenet_n", "lookahead_widths", "lookahead_curvatures"}
    needs_track = bool(features_set & track_features)  # intersection of features_set and track_features is non-empty

    if needs_track:
        # Ensure track exists
        if not hasattr(env, "track") or env.track is None:
            raise ValueError(
                f"Features {features_set & track_features} require track data. "
                "Ensure env.track is initialized before using these features."
            )

        track = env.track

        # Frenet lateral distance and track widths (use global bounds for cross-track consistency)
        if "frenet_n" in features_set or "lookahead_widths" in features_set:
            min_width, max_width = GLOBAL_MIN_WIDTH, GLOBAL_MAX_WIDTH

            if "frenet_n" in features_set:
                if frenet_reference not in ("centerline", "raceline"):
                    raise ValueError(
                        f"Invalid frenet_reference {frenet_reference!r} for 'frenet_n' bounds. "
                        "Must be 'centerline' or 'raceline'."
                    )
                # Centerline is mid-track so |ey| <= half width; a raceline hugs the apex and can
                # sit nearly a full width from the far boundary.
                max_deviation = max_width if frenet_reference == "raceline" else 0.5 * max_width
                bounds["frenet_n"] = (-max_deviation, max_deviation)

            if "lookahead_widths" in features_set:
                bounds["lookahead_widths"] = (min_width, max_width)

        # Lookahead curvatures (use global bounds for cross-track consistency)
        if "lookahead_curvatures" in features_set:
            bounds["lookahead_curvatures"] = (-GLOBAL_MAX_CURVATURE, GLOBAL_MAX_CURVATURE)

    # ===========================
    # 6. VALIDATION
    # ===========================

    # Verify all requested features have bounds
    missing_features = features_set - set(bounds.keys())
    if missing_features:
        raise ValueError(
            f"Cannot calculate bounds for features: {missing_features}. "
            f"These features either don't support normalization or are unknown. "
            f"Supported features: linear_vel_x, linear_vel_y, ang_vel_z, delta, "
            f"prev_steering_cmd, curr_steering_cmd, prev_throttle_cmd, curr_throttle_cmd, "
            f"prev_avg_wheel_omega, curr_avg_wheel_omega, integrated_vel_cmd, "
            f"frenet_u, frenet_n, lookahead_widths, lookahead_curvatures, raceline_vxs, beta"
        )

    # Ensure all bounds have min <= max (allow min == max for constant features)
    for feature_name, (min_val, max_val) in bounds.items():
        if min_val > max_val:
            raise ValueError(
                f"Invalid bounds for feature '{feature_name}': min={min_val} > max={max_val}. "
                f"Bounds must satisfy min <= max."
            )
        if min_val == max_val:
            warnings.warn(f"Feature {feature_name} has equal min, max bounds. Verify this is intentional", UserWarning)

    return bounds


def normalize_feature(
    feature_name: str, feature_value: np.float32 | np.ndarray, norm_bounds: dict
) -> np.float32 | np.ndarray:
    """
    Normalize a feature value using min-max normalization to [-1, 1] range.

    Parameters
    ----------
    feature_name : str
        Name of the feature to normalize
    feature_value : np.float32 | np.ndarray
        The feature value(s) to normalize (can be scalar or array)
    norm_bounds : dict
        Dictionary mapping feature names to (min, max) tuples

    Returns
    -------
    np.float32 | np.ndarray
        Normalized feature value(s) in [-1, 1] range

    Raises
    ------
    ValueError
        If feature_name is not found in norm_bounds
    """
    # Raise error if feature does not have normalization bounds defined
    if feature_name not in norm_bounds:
        available_features = list(norm_bounds.keys())
        raise ValueError(
            f"Cannot normalize feature '{feature_name}': no bounds defined. "
            f"Available features with bounds: {available_features}"
        )

    min_val, max_val = norm_bounds[feature_name]

    # Check if range is too small (constant feature) - prevents division by zero
    range_val = max_val - min_val
    if np.isclose(range_val, 0.0, atol=1e-9):
        # Feature is essentially constant, return 0.0 (center of [-1, 1])
        if isinstance(feature_value, np.ndarray):
            return np.zeros_like(feature_value, dtype=np.float32)
        else:
            return np.float32(0.0)

    # Min-max normalization to [-1, 1]: 2 * (x - min) / (max - min) - 1
    normalized = 2.0 * (feature_value - min_val) / range_val - 1.0

    # Clip to [-1, 1] to handle any values outside expected bounds
    return np.clip(normalized, -1.0, 1.0)
