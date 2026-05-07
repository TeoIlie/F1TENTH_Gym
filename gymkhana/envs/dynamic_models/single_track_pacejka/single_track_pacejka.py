"""Single Track Pacejka (STP) vehicle dynamics model.

Dynamic single-track bicycle with a Pacejka Magic Formula lateral tire model
(no longitudinal slip dynamics). Ported from f110-simulator, and adjusted
for this simulator. For the ref, see ``STDKinematics::update_pacejka`` (std_kinematics.cpp).
https://github.com/ForzaETH/f1tenth_simulator

Deviations from the C++ original:

- State uses ``(V, beta)`` instead of ``(v_x, v_y)``; body-frame derivatives are
  chain-ruled into ``(V_dot, beta_dot)``.
- Returns derivatives only (gymkhana's RK4 integrates externally), so the
  low-speed kinematic blend mixes *derivatives*, not post-integrated states.
- Constraints applied inside ``f``.
- Slip-angle sign flipped (gymkhana convention); ``F_y`` is negated to compensate
  since Pacejka is odd in alpha.
- Blend thresholds match the f110 reference (``v_s=3.0, v_b=1.0,
  v_min_blend=1.0``) and are gated on ``V`` rather than ``v_x``; the hard
  ``w_std=0`` clamp below ``v_min`` is replaced by a sharper tanh plus
  zeroing ``alpha`` and ``beta_dot`` below ``v_min_blend``. Thresholds are
  overridable via params keys ``blend_v_s``, ``blend_v_b``, ``blend_v_min``
  (used by parity tests against the f110 ref).

State extraction: STP shares the ST 7-element state layout, so the dispatch in
``dynamic_models/__init__.py`` reuses ST's ``get_standardized_state_st`` for
STP rather than defining a separate function here.
"""

import numpy as np

from ..kinematic import vehicle_dynamics_ks_cog
from ..utils import accl_constraints, steering_constraint


def vehicle_dynamics_stp(x: np.ndarray, u_init: np.ndarray, params: dict) -> np.ndarray:
    """Compute Single Track Pacejka vehicle dynamics.

    Args:
        x: State vector of shape ``(7,)``:
            ``[x_pos, y_pos, steering_angle, velocity, yaw_angle, yaw_rate, slip_angle]``.
        u_init: Control input ``[steering_velocity, acceleration]``.
        params: Vehicle parameters dict. Required keys:
            ``mu, lf, lr, h_s, m, I_z,
            B_f, C_f, D_f, E_f, B_r, C_r, D_r, E_r,
            s_min, s_max, sv_min, sv_max, v_switch, a_max, v_min, v_max``.

    Returns:
        Time derivatives of the state vector, shape ``(7,)``.
    """
    # State unpacking
    DELTA = x[2]
    V = x[3]
    PSI = x[4]
    PSI_DOT = x[5]
    BETA = x[6]

    # Constants
    g = 9.81

    # Kinematic↔dynamic blend thresholds. Match the original f110-simulator
    # values (std_kinematics.cpp)
    # Overridable via params for parity testing against the f110 reference.
    v_s = params.get("blend_v_s", 3.0)  # blend center
    v_b = params.get("blend_v_b", 1.0)  # blend width tanh scale
    v_min_blend = params.get("blend_v_min", 1.0)  # hard kinematic floor

    # Apply input constraints (idempotent under RK4 sub-stages)
    u = np.array(
        [
            steering_constraint(
                DELTA,
                u_init[0],
                params["s_min"],
                params["s_max"],
                params["sv_min"],
                params["sv_max"],
            ),
            accl_constraints(
                V,
                u_init[1],
                params["v_switch"],
                params["a_max"],
                params["v_min"],
                params["v_max"],
            ),
        ]
    )
    STEER_VEL = u[0]
    ACCL = u[1]

    lf = params["lf"]
    lr = params["lr"]
    lwb = lf + lr
    m = params["m"]
    I_z = params["I_z"]
    h_s = params["h_s"]
    mu = params["mu"]

    # --- Lateral tire slip angles (gymkhana sign convention) ---
    # Equivalent to f110's atan2(-v_y - lf*omega, v_x) + delta with v_y = V*sin(beta)
    if V > v_min_blend:
        denom = V * np.cos(BETA)
        alpha_f = np.arctan((V * np.sin(BETA) + PSI_DOT * lf) / denom) - DELTA
        alpha_r = np.arctan((V * np.sin(BETA) - PSI_DOT * lr) / denom)
    else:
        alpha_f = 0.0
        alpha_r = 0.0

    # --- Vertical loads with longitudinal weight transfer ---
    F_zf = m * (-ACCL * h_s + g * lr) / lwb
    F_zr = m * (ACCL * h_s + g * lf) / lwb

    # --- Pacejka Magic Formula lateral forces ---
    # Note: gymkhana sign convention has alpha = -alpha_f110, so we negate F_y to keep
    # the body-frame equations identical to the f110 implementation.
    Bf, Cf, Df, Ef = params["B_f"], params["C_f"], params["D_f"], params["E_f"]
    Br, Cr, Dr, Er = params["B_r"], params["C_r"], params["D_r"], params["E_r"]
    F_yf = -mu * Df * F_zf * np.sin(Cf * np.arctan(Bf * alpha_f - Ef * (Bf * alpha_f - np.arctan(Bf * alpha_f))))
    F_yr = -mu * Dr * F_zr * np.sin(Cr * np.arctan(Br * alpha_r - Er * (Br * alpha_r - np.arctan(Br * alpha_r))))

    # --- Body-frame derivatives (exact, no small-angle) ---
    # v_x = V*cos(beta), v_y = V*sin(beta)
    sin_beta = np.sin(BETA)
    cos_beta = np.cos(BETA)
    sin_delta = np.sin(DELTA)
    cos_delta = np.cos(DELTA)

    v_x_dot = ACCL - (1.0 / m) * F_yf * sin_delta + V * sin_beta * PSI_DOT
    v_y_dot = (1.0 / m) * (F_yr + F_yf * cos_delta) - V * cos_beta * PSI_DOT
    psi_ddot = (1.0 / I_z) * (-F_yr * lr + F_yf * lf * cos_delta)

    # --- Chain-rule into (V_dot, beta_dot) ---
    V_dot = v_x_dot * cos_beta + v_y_dot * sin_beta
    if V > v_min_blend:
        beta_dot = (v_y_dot * cos_beta - v_x_dot * sin_beta) / V
    else:
        beta_dot = 0.0

    # --- Kinematic-bicycle derivatives for low-speed blend ---
    x_ks = np.array([x[0], x[1], DELTA, V, PSI])
    f_ks = vehicle_dynamics_ks_cog(x_ks, u, params)
    V_dot_ks = f_ks[3]
    psi_dot_ks = f_ks[4]
    beta_dot_ks = (lr * STEER_VEL) / (lwb * cos_delta**2 * (1 + (np.tan(DELTA) ** 2 * lr / lwb) ** 2))
    psi_ddot_ks = (1.0 / lwb) * (
        ACCL * cos_beta * np.tan(DELTA)
        - V * sin_beta * beta_dot_ks * np.tan(DELTA)
        + V * cos_beta * STEER_VEL / cos_delta**2
    )

    # --- Blend derivatives ---
    w_std = 0.5 * (np.tanh((V - v_s) / v_b) + 1.0)
    w_ks = 1.0 - w_std

    f = np.array(
        [
            V * np.cos(PSI + BETA),
            V * np.sin(PSI + BETA),
            STEER_VEL,
            w_std * V_dot + w_ks * V_dot_ks,
            w_std * PSI_DOT + w_ks * psi_dot_ks,
            w_std * psi_ddot + w_ks * psi_ddot_ks,
            w_std * beta_dot + w_ks * beta_dot_ks,
        ]
    )
    return f
