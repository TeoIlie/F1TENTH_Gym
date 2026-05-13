"""
Compares behaviour of fullscale parameters with f1tenth 1/10 scale parameters.
"""

import math
import os
from datetime import datetime

import matplotlib.pyplot as plt
import numpy as np
import yaml
from matplotlib.gridspec import GridSpec
from scipy.integrate import odeint

from gymkhana.envs.dynamic_models import vehicle_dynamics_std_py as vehicle_dynamics_std
from gymkhana.envs.dynamic_models.single_track_drift import init_std
from gymkhana.envs.gymkhana_env import GKEnv

# ==================
# SHARED
# ==================

g = 9.81  # [m/s^2]

# set options
tStart = 0  # start time
tFinal = 1  # start time

# create dynamic models for STD and MB
delta0 = 0
vel0_fullscale = 15  # m/s for fullscale vehicle
vel0_f1tenth = 5  # m/s for 1/10 scale vehicle (scaled appropriately)
Psi0 = 0
dotPsi0 = 0
beta0 = 0
sy0 = 0
# We'll create separate initial states for each scale
initialState_fullscale = [0, sy0, delta0, vel0_fullscale, Psi0, dotPsi0, beta0]
initialState_f1tenth = [0, sy0, delta0, vel0_f1tenth, Psi0, dotPsi0, beta0]


# ==================
# PARAMS
# ==================
# fullscale vehicle params
p = GKEnv.fullscale_vehicle_params()
# f1tenth vehicle params (1/10 scale)
p_10th = GKEnv.f1tenth_std_vehicle_params()


def func_STD(x, t, u, p):
    f = vehicle_dynamics_std(x, u, p)
    return f


x0_STD_fullscale = init_std(initialState_fullscale, p)  # initial state for fullscale
x0_STD_f1tenth = init_std(initialState_f1tenth, p_10th)  # initial state for F1/10th

# --------------------------------------------------------------------------


def compare_std_cornering_left(v_delta, a_long, ax1, ax2):
    # steering to left
    t = np.arange(0, tFinal, 0.01)
    u = [v_delta, a_long]

    # simulate multibody
    x_left_fullscale = odeint(func_STD, x0_STD_fullscale, t, args=(u, p))
    x_left_f1tenth = odeint(func_STD, x0_STD_f1tenth, t, args=(u, p_10th))

    # results
    # position
    ax1.set_title("Cornering Left: Position")
    ax1.plot([tmp[0] for tmp in x_left_fullscale], [tmp[1] for tmp in x_left_fullscale])
    ax1.plot([tmp[0] for tmp in x_left_f1tenth], [tmp[1] for tmp in x_left_f1tenth], linestyle="--")
    ax1.legend(["Fullscale", "F1TENTH"])
    ax1.set_xlabel("X Position [m]")
    ax1.set_ylabel("Y Position [m]")

    # slip angle
    ax2.set_title("Cornering Left: Slip Angle")
    ax2.plot(t, [tmp[6] for tmp in x_left_fullscale])
    ax2.plot(t, [tmp[6] for tmp in x_left_f1tenth], linestyle="--")
    ax2.legend(["Fullscale", "F1TENTH"])
    ax2.set_xlabel("Time [s]")
    ax2.set_ylabel("Slip angle β [rad]")


def compare_std_oversteer_understeer(ax1, ax2, ax3):
    t = np.arange(0, tFinal, 0.01)
    v_delta = 0.15

    # coasting
    u = [v_delta, 0]
    x_coast_fullscale = odeint(func_STD, x0_STD_fullscale, t, args=(u, p))
    x_coast_f1tenth = odeint(func_STD, x0_STD_f1tenth, t, args=(u, p_10th))

    # braking
    u = [v_delta, -0.75 * g]
    x_brake_fullscale = odeint(func_STD, x0_STD_fullscale, t, args=(u, p))
    x_brake_f1tenth = odeint(func_STD, x0_STD_f1tenth, t, args=(u, p_10th))

    # accelerating
    u = [v_delta, 0.63 * g]
    x_acc_fullscale = odeint(func_STD, x0_STD_fullscale, t, args=(u, p))
    x_acc_f1tenth = odeint(func_STD, x0_STD_f1tenth, t, args=(u, p_10th))

    # position
    ax1.set_title("Oversteer/Understeer: Position Comparison")
    ax1.plot([tmp[0] for tmp in x_coast_fullscale], [tmp[1] for tmp in x_coast_fullscale])
    ax1.plot([tmp[0] for tmp in x_brake_fullscale], [tmp[1] for tmp in x_brake_fullscale])
    ax1.plot([tmp[0] for tmp in x_acc_fullscale], [tmp[1] for tmp in x_acc_fullscale])
    ax1.plot([tmp[0] for tmp in x_coast_f1tenth], [tmp[1] for tmp in x_coast_f1tenth], linestyle="--")
    ax1.plot([tmp[0] for tmp in x_brake_f1tenth], [tmp[1] for tmp in x_brake_f1tenth], linestyle="--")
    ax1.plot([tmp[0] for tmp in x_acc_f1tenth], [tmp[1] for tmp in x_acc_f1tenth], linestyle="--")
    ax1.legend(["Coast-Full", "Brake-Full", "Accel-Full", "Coast-F1/10", "Brake-F1/10", "Accel-F1/10"], fontsize=8)
    ax1.set_xlabel("X Position [m]")
    ax1.set_ylabel("Y Position [m]")

    # compare slip angles
    ax2.set_title("Oversteer/Understeer: Slip Angle Comparison")
    ax2.plot(t, [tmp[6] for tmp in x_coast_fullscale])
    ax2.plot(t, [tmp[6] for tmp in x_brake_fullscale])
    ax2.plot(t, [tmp[6] for tmp in x_acc_fullscale])
    ax2.plot(t, [tmp[6] for tmp in x_coast_f1tenth], linestyle="--")
    ax2.plot(t, [tmp[6] for tmp in x_brake_f1tenth], linestyle="--")
    ax2.plot(t, [tmp[6] for tmp in x_acc_f1tenth], linestyle="--")
    ax2.legend(["Coast-Full", "Brake-Full", "Accel-Full", "Coast-F1/10", "Brake-F1/10", "Accel-F1/10"], fontsize=8)
    ax2.set_xlabel("Time [s]")
    ax2.set_ylabel("Front wheel ang speed [rad/s]")

    # orientation
    ax3.set_title("Oversteer/Understeer: Orientation Comparison")
    ax3.plot(t, [tmp[4] for tmp in x_coast_fullscale])
    ax3.plot(t, [tmp[4] for tmp in x_brake_fullscale])
    ax3.plot(t, [tmp[4] for tmp in x_acc_fullscale])
    ax3.plot(t, [tmp[4] for tmp in x_coast_f1tenth], linestyle="--")
    ax3.plot(t, [tmp[4] for tmp in x_brake_f1tenth], linestyle="--")
    ax3.plot(t, [tmp[4] for tmp in x_acc_f1tenth], linestyle="--")
    ax3.legend(["Coast-Full", "Brake-Full", "Accel-Full", "Coast-F1/10", "Brake-F1/10", "Accel-F1/10"], fontsize=8)
    ax3.set_xlabel("Time [s]")
    ax3.set_ylabel("Heading angle Ψ [rad]")


def compare_std_braking(ax1, ax2, ax3):
    t = np.arange(0, tFinal, 0.01)
    v_delta = 0.0
    acc = -0.7 * g
    u = [v_delta, acc]

    # simulate car
    x_brake_fullscale = odeint(func_STD, x0_STD_fullscale, t, args=(u, p))
    x_brake_f1tenth = odeint(func_STD, x0_STD_f1tenth, t, args=(u, p_10th))

    # position
    ax1.set_title("Braking: Position")
    ax1.plot([tmp[0] for tmp in x_brake_fullscale], [tmp[1] for tmp in x_brake_fullscale])
    ax1.plot([tmp[0] for tmp in x_brake_f1tenth], [tmp[1] for tmp in x_brake_f1tenth], linestyle="--")
    ax1.legend(["Fullscale", "F1TENTH"])
    ax1.set_xlabel("X Position [m]")
    ax1.set_ylabel("Y Position [m]")

    # velocity
    ax2.set_title("Braking: Velocity")
    ax2.plot(t, [tmp[3] * math.cos(tmp[6]) for tmp in x_brake_fullscale])
    ax2.plot(t, [tmp[3] * math.cos(tmp[6]) for tmp in x_brake_f1tenth], linestyle="--")
    ax2.legend(["Fullscale", "F1TENTH"])
    ax2.set_xlabel("Time [s]")
    ax2.set_ylabel("Longitudinal Velocity [m/s]")

    # wheel spin
    ax3.set_title("Braking: Wheel Spin")
    ax3.plot(t, [tmp[7] for tmp in x_brake_fullscale])
    ax3.plot(t, [tmp[8] for tmp in x_brake_fullscale])
    ax3.plot(t, [tmp[7] for tmp in x_brake_f1tenth], linestyle="--")
    ax3.plot(t, [tmp[8] for tmp in x_brake_f1tenth], linestyle="--")
    ax3.legend(["Full-Front", "Full-Rear", "F1/10-Front", "F1/10-Rear"], fontsize=8)
    ax3.set_xlabel("Time [s]")
    ax3.set_ylabel("Wheel Angular Velocity [rad/s]")


def compare_std_accelerating(ax1, ax2, ax3):
    t = np.arange(0, tFinal, 0.01)
    v_delta = 0.0
    acc = 0.63 * g
    u = [v_delta, acc]

    # simulate car
    x_acc_fullscale = odeint(func_STD, x0_STD_fullscale, t, args=(u, p))
    x_acc_f1tenth = odeint(func_STD, x0_STD_f1tenth, t, args=(u, p_10th))

    # position
    ax1.set_title("Accelerating: Position")
    ax1.plot([tmp[0] for tmp in x_acc_fullscale], [tmp[1] for tmp in x_acc_fullscale])
    ax1.plot([tmp[0] for tmp in x_acc_f1tenth], [tmp[1] for tmp in x_acc_f1tenth], linestyle="--")
    ax1.legend(["Fullscale", "F1TENTH"])
    ax1.set_xlabel("X Position [m]")
    ax1.set_ylabel("Y Position [m]")

    # velocity
    ax2.set_title("Accelerating: Velocity")
    ax2.plot(t, [tmp[3] * math.cos(tmp[6]) for tmp in x_acc_fullscale])
    ax2.plot(t, [tmp[3] * math.cos(tmp[6]) for tmp in x_acc_f1tenth], linestyle="--")
    ax2.legend(["Fullscale", "F1TENTH"])
    ax2.set_xlabel("Time [s]")
    ax2.set_ylabel("Longitudinal Velocity [m/s]")

    # wheel spin
    ax3.set_title("Accelerating: Wheel Spin")
    ax3.plot(t, [tmp[7] for tmp in x_acc_fullscale])
    ax3.plot(t, [tmp[8] for tmp in x_acc_fullscale])
    ax3.plot(t, [tmp[7] for tmp in x_acc_f1tenth], linestyle="--")
    ax3.plot(t, [tmp[8] for tmp in x_acc_f1tenth], linestyle="--")
    ax3.legend(["Full-Wheel1", "Full-Wheel2", "F1/10-Wheel1", "F1/10-Wheel2"], fontsize=8)
    ax3.set_xlabel("Time [s]")
    ax3.set_ylabel("Wheel Angular Velocity [rad/s]")


# run simulations *****************
if __name__ == "__main__":
    # Create a single figure with all subplots
    fig = plt.figure(figsize=(18, 16))
    gs = GridSpec(4, 3, figure=fig, hspace=0.3, wspace=0.3)

    # Row 1: Cornering left (2 subplots)
    ax1_1 = fig.add_subplot(gs[0, 0])
    ax1_2 = fig.add_subplot(gs[0, 1])

    # Row 2: Oversteer/Understeer (3 subplots)
    ax2_1 = fig.add_subplot(gs[1, 0])
    ax2_2 = fig.add_subplot(gs[1, 1])
    ax2_3 = fig.add_subplot(gs[1, 2])

    # Row 3: Braking (3 subplots)
    ax3_1 = fig.add_subplot(gs[2, 0])
    ax3_2 = fig.add_subplot(gs[2, 1])
    ax3_3 = fig.add_subplot(gs[2, 2])

    # Row 4: Accelerating (3 subplots)
    ax4_1 = fig.add_subplot(gs[3, 0])
    ax4_2 = fig.add_subplot(gs[3, 1])
    ax4_3 = fig.add_subplot(gs[3, 2])

    # Run all comparisons
    compare_std_cornering_left(0.15, 0, ax1_1, ax1_2)
    compare_std_oversteer_understeer(ax2_1, ax2_2, ax2_3)
    compare_std_braking(ax3_1, ax3_2, ax3_3)
    compare_std_accelerating(ax4_1, ax4_2, ax4_3)

    # Add overall title
    fig.suptitle("Fullscale vs F1TENTH Vehicle Parameters Comparison - STD Model", fontsize=10, fontweight="bold")

    # Create figures directory if it doesn't exist
    today = datetime.now().strftime("%Y-%m-%d")
    figures_dir = os.path.join(os.path.dirname(__file__), "..", "..", "..", "figures", "analysis", "tire_params", today)
    os.makedirs(figures_dir, exist_ok=True)

    # Save f1tenth parameters to YAML
    params_filepath = os.path.join(figures_dir, "f1tenth_params.yaml")
    with open(params_filepath, "w") as f:
        yaml.dump(p_10th, f, default_flow_style=False, sort_keys=False)

    # Save with fixed filename
    filename = "f1tenth_std_params_comparison.png"
    filepath = os.path.join(figures_dir, filename)
    plt.savefig(filepath, bbox_inches="tight")
    plt.show()
