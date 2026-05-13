"""
Compares behaviour of the STP (Single Track Pacejka) model against the ST
(Single Track) model on 1/10 scale F1TENTH parameters. Pure visual comparison.

Scenarios:
  1. Linear-regime cornering: low speed, low steering -> models should agree.
  2. Saturation cornering: higher speed and steering -> STP saturates, ST does not.
  3. Steady-state understeer sweep: yaw rate vs steering target at fixed speed.
  4. Low-speed kinematic blend: ST hard-switch vs STP tanh-blend across the threshold.
"""

import matplotlib.pyplot as plt
import numpy as np
from scipy.integrate import odeint

from gymkhana.envs.dynamic_models import vehicle_dynamics_st_py as vehicle_dynamics_st
from gymkhana.envs.dynamic_models import vehicle_dynamics_stp_py as vehicle_dynamics_stp
from gymkhana.envs.gymkhana_env import GKEnv

# ST and STP take incompatible parameter dicts (linear stiffness vs Pacejka coeffs),
# so each model gets its own param set. Both yamls are tuned for the 1/10 scale car.
p_st = GKEnv.f1tenth_vehicle_params()
p_stp = GKEnv.f1tenth_stp_vehicle_params()


def func_ST(x, t, u, p):
    return vehicle_dynamics_st(x, u, p)


def func_STP(x, t, u, p):
    return vehicle_dynamics_stp(x, u, p)


def _initial_state(vel0):
    # [x, y, delta, V, psi, psi_dot, beta]
    return np.array([0.0, 0.0, 0.0, vel0, 0.0, 0.0, 0.0])


def _plot_pair(t, x_st, x_stp, suptitle):
    _, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(15, 4))

    ax1.set_title("trajectory")
    ax1.plot([s[0] for s in x_st], [s[1] for s in x_st])
    ax1.plot([s[0] for s in x_stp], [s[1] for s in x_stp])
    ax1.set_xlabel("x [m]")
    ax1.set_ylabel("y [m]")
    ax1.set_aspect("equal", adjustable="datalim")
    ax1.legend(["ST", "STP"])

    ax2.set_title("slip angle")
    ax2.plot(t, [s[6] for s in x_st])
    ax2.plot(t, [s[6] for s in x_stp])
    ax2.set_xlabel("t [s]")
    ax2.set_ylabel("beta [rad]")
    ax2.legend(["ST", "STP"])

    ax3.set_title("yaw rate")
    ax3.plot(t, [s[5] for s in x_st])
    ax3.plot(t, [s[5] for s in x_stp])
    ax3.set_xlabel("t [s]")
    ax3.set_ylabel("psi_dot [rad/s]")
    ax3.legend(["ST", "STP"])

    plt.suptitle(suptitle)
    plt.tight_layout()
    plt.show()


def compare_linear_cornering():
    """Low speed, low steering -- STP should track ST closely in the linear tire regime."""
    vel0 = 3.0
    v_delta = 0.05
    a_long = 0.0
    tFinal = 2.0

    t = np.arange(0, tFinal, 0.01)
    u = [v_delta, a_long]
    x0 = _initial_state(vel0)

    x_st = odeint(func_ST, x0, t, args=(u, p_st))
    x_stp = odeint(func_STP, x0, t, args=(u, p_stp))

    _plot_pair(t, x_st, x_stp, f"Linear-regime cornering  (vel0={vel0}, v_delta={v_delta})")


def compare_saturation_cornering():
    """Higher speed and steering -- Pacejka saturation should diverge from ST's linear forces."""
    vel0 = 7.0
    v_delta = 0.4
    a_long = 0.0
    tFinal = 1.5

    t = np.arange(0, tFinal, 0.01)
    u = [v_delta, a_long]
    x0 = _initial_state(vel0)

    x_st = odeint(func_ST, x0, t, args=(u, p_st))
    x_stp = odeint(func_STP, x0, t, args=(u, p_stp))

    _plot_pair(t, x_st, x_stp, f"Saturation cornering  (vel0={vel0}, v_delta={v_delta})")


def compare_understeer_sweep():
    """Steady-state yaw rate vs steering target at fixed speed.

    For each delta target: ramp steering linearly over t_ramp, then hold steer_vel=0
    and integrate to settle. Record final psi_dot. ST yields a near-linear curve;
    STP curls over as the tires saturate -- the textbook understeer signature.
    """
    vel0 = 5.0
    t_ramp = 0.5
    t_settle = 3.0
    delta_targets = np.linspace(0.02, 0.4, 12)

    t1 = np.arange(0, t_ramp, 0.01)
    t2 = np.arange(0, t_settle, 0.01)

    psi_dot_ss_st = []
    psi_dot_ss_stp = []
    for delta_t in delta_targets:
        v_delta = delta_t / t_ramp
        x0 = _initial_state(vel0)

        # Phase 1: ramp steering to target
        x_st_1 = odeint(func_ST, x0, t1, args=([v_delta, 0.0], p_st))
        x_stp_1 = odeint(func_STP, x0, t1, args=([v_delta, 0.0], p_stp))

        # Phase 2: hold steering, let yaw rate settle
        x_st_2 = odeint(func_ST, x_st_1[-1], t2, args=([0.0, 0.0], p_st))
        x_stp_2 = odeint(func_STP, x_stp_1[-1], t2, args=([0.0, 0.0], p_stp))

        psi_dot_ss_st.append(x_st_2[-1, 5])
        psi_dot_ss_stp.append(x_stp_2[-1, 5])

    _, ax = plt.subplots(figsize=(7, 5))
    ax.plot(delta_targets, psi_dot_ss_st, "o-")
    ax.plot(delta_targets, psi_dot_ss_stp, "o-")
    ax.set_xlabel("steady-state delta [rad]")
    ax.set_ylabel("steady-state psi_dot [rad/s]")
    ax.set_title(f"Understeer sweep  (vel0={vel0})")
    ax.legend(["ST", "STP"])
    ax.grid(True)
    plt.tight_layout()
    plt.show()


def compare_low_speed_blend():
    """Low speed accelerating through the kinematic blend region.

    ST hard-switches at V < 0.5; STP tanh-blends around v_s = 0.2. The longitudinal
    acceleration drives V from 0.15 across both thresholds during the run.
    """
    vel0 = 0.15
    v_delta = 0.2
    a_long = 0.3
    tFinal = 2.0

    t = np.arange(0, tFinal, 0.01)
    u = [v_delta, a_long]
    x0 = _initial_state(vel0)

    x_st = odeint(func_ST, x0, t, args=(u, p_st))
    x_stp = odeint(func_STP, x0, t, args=(u, p_stp))

    _plot_pair(t, x_st, x_stp, f"Low-speed kinematic blend  (vel0={vel0}, accl={a_long})")


if __name__ == "__main__":
    compare_linear_cornering()
    compare_saturation_cornering()
    compare_understeer_sweep()
    compare_low_speed_blend()
