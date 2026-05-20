"""Single Track (ST) vehicle dynamics model."""

import numpy as np
from numba import njit

from .utils import accl_constraints, steering_constraint


@njit(cache=True)
def vehicle_dynamics_st(x: np.ndarray, u_init: np.ndarray, params) -> np.ndarray:
    """Compute Single Track vehicle dynamics.

    Reference: CommonRoad vehicle models, section 7.

    Args:
        x: State vector of shape ``(7,)``:
            ``[x_pos, y_pos, steering_angle, velocity, yaw_angle, yaw_rate, slip_angle]``.
        u_init: Control input ``[steering_velocity, acceleration]``.
        params: :class:`gymkhana.envs.params.VehicleParams` NamedTuple.

    Returns:
        Time derivatives of the state vector, shape ``(7,)``.
    """
    # Implementation notes (vs. CommonRoad vehiclemodels/vehicle_dynamics_st.py):
    # 1. Kinematic threshold: switch at V < 0.5 instead of x[3] < 0.1
    # 2. BETA_HAT/BETA_DOT computed differently — BETA_HAT does a modulus op
    # 3. Same dynamic equations, restructured for readability; np replaces math
    # States
    DELTA = x[2]
    V = x[3]
    PSI = x[4]  # yaw angle
    PSI_DOT = x[5]  # yaw rate
    BETA = x[6]  # slip angle
    # BETA = np.arctan2(np.sin(BETA), np.cos(BETA))  # wrap to [-pi, pi] if needed

    # gravity constant m/s^2
    g = 9.81

    # constraints
    STEER_VEL = steering_constraint(DELTA, u_init[0], params.s_min, params.s_max, params.sv_min, params.sv_max)
    ACCL = accl_constraints(V, u_init[1], params.v_switch, params.a_max, params.v_min, params.v_max)

    # switch to kinematic model for small velocities
    if V < 0.5:
        lwb = params.lf + params.lr
        BETA_HAT = np.arctan(np.tan(DELTA) * params.lr / lwb)
        BETA_DOT = (
            (1 / (1 + (np.tan(DELTA) * (params.lr / lwb)) ** 2)) * (params.lr / (lwb * np.cos(DELTA) ** 2)) * STEER_VEL
        )
        f = np.array(
            [
                V * np.cos(PSI + BETA_HAT),  # X_DOT
                V * np.sin(PSI + BETA_HAT),  # Y_DOT
                STEER_VEL,  # DELTA_DOT
                ACCL,  # V_DOT
                V * np.cos(BETA_HAT) * np.tan(DELTA) / lwb,  # PSI_DOT
                (1 / lwb)
                * (
                    ACCL * np.cos(BETA) * np.tan(DELTA)
                    - V * np.sin(BETA) * np.tan(DELTA) * BETA_DOT
                    + ((V * np.cos(BETA) * STEER_VEL) / (np.cos(DELTA) ** 2))
                ),  # PSI_DOT_DOT
                BETA_DOT,  # BETA_DOT
            ]
        )
    else:
        # system dynamics
        glr = g * params.lr - ACCL * params.h  # rear load transfer
        glf = g * params.lf + ACCL * params.h  # front load transfer
        mu_m = params.mu * params.m
        I_lwb = params.I * (params.lf + params.lr)
        f = np.array(
            [
                V * np.cos(PSI + BETA),  # X_DOT
                V * np.sin(PSI + BETA),  # Y_DOT
                STEER_VEL,  # DELTA_DOT
                ACCL,  # V_DOT
                PSI_DOT,
                (mu_m / I_lwb)
                * (
                    params.lf * params.C_Sf * glr * DELTA
                    + (params.lr * params.C_Sr * glf - params.lf * params.C_Sf * glr) * BETA
                    - (params.lf * params.lf * params.C_Sf * glr + params.lr * params.lr * params.C_Sr * glf)
                    * (PSI_DOT / V)
                ),  # PSI_DOT_DOT
                (params.mu / (V * (params.lr + params.lf)))
                * (
                    params.C_Sf * glr * DELTA
                    - (params.C_Sr * glf + params.C_Sf * glr) * BETA
                    + (params.C_Sr * glf * params.lr - params.C_Sf * glr * params.lf) * (PSI_DOT / V)
                )
                - PSI_DOT,  # BETA_DOT
            ]
        )

    return f


@njit(cache=True)
def get_standardized_state_st(x: np.ndarray) -> dict:
    """Extract standardized state dict from ST model state vector.

    Args:
        x: ST state vector ``[x_pos, y_pos, steering_angle, velocity,
           yaw_angle, yaw_rate, slip_angle]``.

    Returns:
        Dict with keys: ``x``, ``y``, ``delta``, ``v_x``, ``v_y``,
        ``yaw``, ``yaw_rate``, ``slip``.
    """
    d = dict()
    d["x"] = x[0]
    d["y"] = x[1]
    d["delta"] = x[2]
    d["v_x"] = x[3] * np.cos(x[6])
    d["v_y"] = x[3] * np.sin(x[6])
    d["yaw"] = x[4]
    d["yaw_rate"] = x[5]
    d["slip"] = x[6]
    return d
