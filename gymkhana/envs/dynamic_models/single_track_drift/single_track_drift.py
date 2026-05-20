"""Single Track Drift (STD) vehicle dynamics model with PAC2002 tire model."""

import numpy as np
from numba import njit

from ..kinematic import vehicle_dynamics_ks_cog
from ..tire_model import formula_lateral, formula_lateral_comb, formula_longitudinal, formula_longitudinal_comb
from ..utils import accl_constraints, steering_constraint


@njit(cache=True)
def vehicle_dynamics_std(x: np.ndarray, u_init: np.ndarray, params) -> np.ndarray:
    """Compute Single Track Drift vehicle dynamics.

    Extends the ST model with a PAC2002 (Pacejka Magic Formula) tire model
    to capture tire force saturation and slip needed for drifting.

    Reference: CommonRoad STD model (vehicle_dynamics_std.py).

    Author: Teodor Ilie (18-October-2025).

    Args:
        x: State vector of shape ``(9,)``:
            ``[x_pos, y_pos, steering_angle, velocity, yaw_angle,
            yaw_rate, slip_angle, front_wheel_omega, rear_wheel_omega]``.
        u_init: Control input ``[steering_velocity, acceleration]``.
        params: :class:`gymkhana.envs.params.VehicleParams` NamedTuple.

    Returns:
        Time derivatives of the state vector, shape ``(9,)``.
    """
    # State vector index map:
    #   x[0]: X  — x position in Cartesian global coordinates
    #   x[1]: Y  — y position in Cartesian global coordinates
    #   x[2]: DELTA     — steering angle of front wheels
    #   x[3]: V         — velocity (speed at CoG)
    #   x[4]: PSI       — yaw angle in Cartesian global coordinates
    #   x[5]: PSI_DOT   — yaw rate
    #   x[6]: BETA      — slip angle at vehicle centre
    #   x[7]: ANG_FRONT — angular speed of the front wheel (rad/s)
    #   x[8]: ANG_REAR  — angular speed of the rear wheel (rad/s)
    #
    # Control input:
    #   u_init[0]: steering angle velocity of front wheels (rad/s)
    #   u_init[1]: longitudinal acceleration (m/s²)

    # Get states from state vector
    X = x[0]
    Y = x[1]
    DELTA = x[2]
    V = x[3]
    PSI = x[4]
    PSI_DOT = x[5]
    BETA = x[6]
    ANG_FRONT = x[7]
    ANG_REAR = x[8]

    # set gravity constant
    g = 9.81  # [m/s^2]

    # wheelbase
    lwb = params.lf + params.lr

    # mix-models parameters
    # TODO consider changing these if they are not realistic for a 1/10 scale vehicle —
    # the car seems to be a bit sluggish starting without a lot of accl
    v_s = 0.2
    v_b = 0.05
    v_min = v_s / 2  # local to the blend; distinct from params.v_min used for steering constraints

    # apply steering, accl constraints same as single_track.py
    STEER_VEL = steering_constraint(DELTA, u_init[0], params.s_min, params.s_max, params.sv_min, params.sv_max)
    ACCL = accl_constraints(V, u_init[1], params.v_switch, params.a_max, params.v_min, params.v_max)

    # compute lateral tire slip angles
    if V > v_min:
        alpha_f = np.arctan((V * np.sin(BETA) + PSI_DOT * params.lf) / (V * np.cos(BETA))) - DELTA
        alpha_r = np.arctan((V * np.sin(BETA) - PSI_DOT * params.lr) / (V * np.cos(BETA)))
    else:
        alpha_f = 0.0
        alpha_r = 0.0

    # compute vertical tire forces
    F_zf = params.m * (-ACCL * params.h_s + g * params.lr) / (params.lr + params.lf)
    F_zr = params.m * (ACCL * params.h_s + g * params.lf) / (params.lr + params.lf)

    # compute front and rear tire speeds
    u_wf = max(0.0, V * np.cos(BETA) * np.cos(DELTA) + (V * np.sin(BETA) + params.lf * PSI_DOT) * np.sin(DELTA))
    u_wr = max(0.0, V * np.cos(BETA))

    # compute longitudinal tire slip
    s_f = 1 - params.R_w * ANG_FRONT / max(u_wf, v_min)
    s_r = 1 - params.R_w * ANG_REAR / max(u_wr, v_min)

    # compute tire forces (Pacejka)
    # pure slip longitudinal forces
    F0_xf = formula_longitudinal(s_f, 0.0, F_zf, params)
    F0_xr = formula_longitudinal(s_r, 0.0, F_zr, params)

    # pure slip lateral forces
    F0_yf, mu_yf = formula_lateral(alpha_f, 0.0, F_zf, params)
    F0_yr, mu_yr = formula_lateral(alpha_r, 0.0, F_zr, params)

    # combined slip longitudinal forces
    F_xf = formula_longitudinal_comb(s_f, alpha_f, F0_xf, params)
    F_xr = formula_longitudinal_comb(s_r, alpha_r, F0_xr, params)

    # combined slip lateral forces
    F_yf = formula_lateral_comb(s_f, alpha_f, 0.0, mu_yf, F_zf, F0_yf, params)
    F_yr = formula_lateral_comb(s_r, alpha_r, 0.0, mu_yr, F_zr, F0_yr, params)

    # convert acceleration input to brake and engine torque
    if ACCL > 0:
        T_B = 0.0
        T_E = params.m * params.R_w * ACCL
    else:
        T_B = params.m * params.R_w * ACCL
        T_E = 0.0

    # system dynamics
    d_v = (
        1
        / params.m
        * (-F_yf * np.sin(DELTA - BETA) + F_yr * np.sin(BETA) + F_xr * np.cos(BETA) + F_xf * np.cos(DELTA - BETA))
    )
    dd_psi = 1 / params.I_z * (F_yf * np.cos(DELTA) * params.lf - F_yr * params.lr + F_xf * np.sin(DELTA) * params.lf)
    if V > v_min:
        d_beta = -PSI_DOT + 1 / (params.m * V) * (
            F_yf * np.cos(DELTA - BETA) + F_yr * np.cos(BETA) - F_xr * np.sin(BETA) + F_xf * np.sin(DELTA - BETA)
        )
    else:
        d_beta = 0.0

    # wheel dynamics (negative wheel spin forbidden)
    if ANG_FRONT >= 0:
        d_omega_f = 1 / params.I_y_w * (-params.R_w * F_xf + params.T_sb * T_B + params.T_se * T_E)
    else:
        d_omega_f = 0.0
    ANG_FRONT = max(0.0, ANG_FRONT)
    if ANG_REAR >= 0:
        d_omega_r = 1 / params.I_y_w * (-params.R_w * F_xr + (1 - params.T_sb) * T_B + (1 - params.T_se) * T_E)
    else:
        d_omega_r = 0.0
    ANG_REAR = max(0.0, ANG_REAR)

    # *** Mix with kinematic model at low speeds ***
    # Due to errors when using the scipy.odeint with a "hard" switch to the kinematic model, we overblend both models
    # around the switching velocity to achieve a "smoother" transition between both models.
    # kinematic system dynamics
    u_constrained = np.array([STEER_VEL, ACCL])
    x_ks = np.array([X, Y, DELTA, V, PSI], dtype=np.float64)
    f_ks = vehicle_dynamics_ks_cog(x_ks, u_constrained, params)
    # derivative of slip angle and yaw rate (kinematic)
    d_beta_ks = (params.lr * STEER_VEL) / (lwb * np.cos(DELTA) ** 2 * (1 + (np.tan(DELTA) ** 2 * params.lr / lwb) ** 2))
    dd_psi_ks = (
        1
        / lwb
        * (
            ACCL * np.cos(BETA) * np.tan(DELTA)
            - V * np.sin(BETA) * d_beta_ks * np.tan(DELTA)
            + V * np.cos(BETA) * STEER_VEL / np.cos(DELTA) ** 2
        )
    )
    # derivative of angular speeds (kinematic)
    d_omega_f_ks = (1 / 0.02) * (u_wf / params.R_w - ANG_FRONT)
    d_omega_r_ks = (1 / 0.02) * (u_wr / params.R_w - ANG_REAR)

    # weights for mixing both models
    w_std = 0.5 * (np.tanh((V - v_s) / v_b) + 1)
    w_ks = 1 - w_std

    # output vector: mix results of dynamic and kinematic model
    f = np.array(
        [
            V * np.cos(BETA + PSI),
            V * np.sin(BETA + PSI),
            STEER_VEL,
            w_std * d_v + w_ks * f_ks[3],
            w_std * PSI_DOT + w_ks * f_ks[4],
            w_std * dd_psi + w_ks * dd_psi_ks,
            w_std * d_beta + w_ks * d_beta_ks,
            w_std * d_omega_f + w_ks * d_omega_f_ks,
            w_std * d_omega_r + w_ks * d_omega_r_ks,
        ]
    )

    # time derivatives of the input state x
    return f


@njit(cache=True)
def get_standardized_state_std(x: np.ndarray) -> dict:
    """Extract standardized state dict from STD model state vector.

    Args:
        x: STD state vector ``[x_pos, y_pos, steering_angle, velocity,
           yaw_angle, yaw_rate, slip_angle, omega_front, omega_rear]``.

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
    d["omega_front"] = x[7]
    d["omega_rear"] = x[8]
    return d
