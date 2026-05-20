"""Kinematic Single Track (KS) vehicle dynamics model."""

import numpy as np
from numba import njit

from .utils import accl_constraints, steering_constraint


@njit(cache=True)
def vehicle_dynamics_ks(x: np.ndarray, u_init: np.ndarray, params) -> np.ndarray:
    """Compute Kinematic Single Track vehicle dynamics.

    Reference: CommonRoad vehicle models, section 5.

    Args:
        x: State vector of shape ``(5,)``:
            ``[x_pos, y_pos, steering_angle, velocity, yaw_angle]``.
        u_init: Control input ``[steering_velocity, acceleration]``.
        params: :class:`gymkhana.envs.params.VehicleParams` NamedTuple.

    Returns:
        Time derivatives of the state vector, shape ``(5,)``.
    """
    # States
    DELTA = x[2]
    V = x[3]
    PSI = x[4]
    # Raw actions
    RAW_STEER_VEL = u_init[0]
    RAW_ACCL = u_init[1]
    # wheelbase
    lwb = params.lf + params.lr

    # constraints
    STEER_VEL = steering_constraint(DELTA, RAW_STEER_VEL, params.s_min, params.s_max, params.sv_min, params.sv_max)
    ACCL = accl_constraints(V, RAW_ACCL, params.v_switch, params.a_max, params.v_min, params.v_max)

    # system dynamics
    f = np.array(
        [
            V * np.cos(PSI),  # X_DOT
            V * np.sin(PSI),  # Y_DOT
            STEER_VEL,  # DELTA_DOT
            ACCL,  # V_DOT
            (V / lwb) * np.tan(DELTA),  # PSI_DOT
        ]
    )
    return f


@njit(cache=True)
def vehicle_dynamics_ks_cog(x: np.ndarray, u_init: np.ndarray, params) -> np.ndarray:
    """Compute Kinematic Single Track dynamics referenced at the centre of gravity.

    Unlike :func:`vehicle_dynamics_ks` (which references the rear axle), this
    variant computes position derivatives at the vehicle's centre of gravity by
    incorporating the kinematic slip angle ``beta``. Used by the STD and STP
    models for the low-speed kinematic blending regime.

    Reference: CommonRoad vehicle models, section 5.
    """
    # States
    DELTA = x[2]
    V = x[3]
    PSI = x[4]
    # Raw actions
    RAW_STEER_VEL = u_init[0]
    RAW_ACCL = u_init[1]
    # wheelbase
    lwb = params.lf + params.lr

    # constraints
    sv = steering_constraint(DELTA, RAW_STEER_VEL, params.s_min, params.s_max, params.sv_min, params.sv_max)
    a = accl_constraints(V, RAW_ACCL, params.v_switch, params.a_max, params.v_min, params.v_max)

    # slip angle (beta) from vehicle kinematics
    beta = np.arctan(np.tan(DELTA) * params.lr / lwb)

    # system dynamics
    f = np.array(
        [
            V * np.cos(beta + PSI),
            V * np.sin(beta + PSI),
            sv,
            a,
            V * np.cos(beta) * np.tan(DELTA) / lwb,
        ]
    )
    return f


@njit(cache=True)
def get_standardized_state_ks(x: np.ndarray) -> dict:
    """Extract standardized state dict from KS model state vector.

    Args:
        x: KS state vector ``[x_pos, y_pos, steering_angle, velocity, yaw_angle]``.

    Returns:
        Dict with keys: ``x``, ``y``, ``delta``, ``v_x``, ``v_y``,
        ``yaw``, ``yaw_rate``, ``slip``.
        ``v_y``, ``yaw_rate``, and ``slip`` are zero (kinematic model).
    """
    d = dict()
    d["x"] = x[0]
    d["y"] = x[1]
    d["delta"] = x[2]
    d["v_x"] = x[3]
    d["v_y"] = 0.0
    d["yaw"] = x[4]
    d["yaw_rate"] = 0.0
    d["slip"] = 0.0
    return d
