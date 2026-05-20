"""
Compares behaviour of f110 gym mb model with reference commonroad mb model to confirm matching behaviour.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import matplotlib.pyplot as plt
import numpy as np
from commonroad.vehiclemodels.init_mb import init_mb as init_mb_cr
from commonroad.vehiclemodels.parameters_vehicle1 import parameters_vehicle1
from commonroad.vehiclemodels.vehicle_dynamics_mb import vehicle_dynamics_mb as cr_vehicle_dynamics_mb
from scipy.integrate import odeint

from gymkhana.envs.dynamic_models.multi_body import init_mb
from gymkhana.envs.dynamic_models.multi_body.multi_body import vehicle_dynamics_mb
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
vel0 = 15
Psi0 = 0
dotPsi0 = 0
beta0 = 0
sy0 = 0
initialState = [0, sy0, delta0, vel0, Psi0, dotPsi0, beta0]  # initial state for simulation


# ==================
# COMMONROAD
# ==================
# commonroad params
p_cr = parameters_vehicle1()


def func_MB_cr(x, t, u, p):
    f = cr_vehicle_dynamics_mb(x, u, p)
    return f


x0_MB_cr = init_mb_cr(initialState, p_cr)


# ==================
# F1TENTH
# ==================
# f110 params
p = GKEnv.fullscale_vehicle_params()


def func_MB(x, t, u, p):
    f = vehicle_dynamics_mb(x, u, p)
    return f


x0_MB = init_mb(initialState, p)  # initial state for multi-body model

# --------------------------------------------------------------------------


def compare_mb_cornering_left(v_delta, a_long):
    # steering to left
    t = np.arange(0, tFinal, 0.01)
    u = [v_delta, a_long]

    # simulate multibody
    x_left_MB = odeint(func_MB, x0_MB, t, args=(u, p))
    x_left_MB_cr = odeint(func_MB_cr, x0_MB_cr, t, args=(u, p_cr))

    # Create figure with 2 subplots (1 row, 2 columns)
    _, (ax1, ax2) = plt.subplots(1, 2, figsize=(10, 4))

    # results
    # position
    ax1.set_title("positions turning")
    ax1.plot([tmp[0] for tmp in x_left_MB], [tmp[1] for tmp in x_left_MB])
    ax1.plot([tmp[0] for tmp in x_left_MB_cr], [tmp[1] for tmp in x_left_MB_cr])
    ax1.legend(["MB", "MB_cr"])

    # slip angle
    ax2.set_title("slip angle turning")
    ax2.plot(t, [tmp[10] / tmp[3] for tmp in x_left_MB])
    ax2.plot(t, [tmp[10] / tmp[3] for tmp in x_left_MB_cr])
    ax2.legend(["MB", "MB_cr"])

    plt.tight_layout()
    plt.show()


def compare_mb_oversteer_understeer():
    t = np.arange(0, tFinal, 0.01)
    v_delta = 0.15

    # coasting
    u = [v_delta, 0]
    x_coast_MB = odeint(func_MB, x0_MB, t, args=(u, p))
    x_coast_MB_cr = odeint(func_MB_cr, x0_MB_cr, t, args=(u, p_cr))

    # braking
    u = [v_delta, -0.7 * g]
    x_brake_MB = odeint(func_MB, x0_MB, t, args=(u, p))
    x_brake_MB_cr = odeint(func_MB_cr, x0_MB_cr, t, args=(u, p_cr))

    # accelerating
    u = [v_delta, 0.63 * g]
    x_acc_MB = odeint(func_MB, x0_MB, t, args=(u, p))
    x_acc_MB_cr = odeint(func_MB_cr, x0_MB_cr, t, args=(u, p_cr))

    # Create figure with 4 subplots (1 row, 4 columns)
    _, (ax1, ax2, ax3, ax4) = plt.subplots(1, 4, figsize=(20, 4))

    # position
    ax1.set_title("position comparison MB")
    ax1.plot([tmp[0] for tmp in x_coast_MB], [tmp[1] for tmp in x_coast_MB])
    ax1.plot([tmp[0] for tmp in x_brake_MB], [tmp[1] for tmp in x_brake_MB])
    ax1.plot([tmp[0] for tmp in x_acc_MB], [tmp[1] for tmp in x_acc_MB])
    ax1.plot([tmp[0] for tmp in x_coast_MB_cr], [tmp[1] for tmp in x_coast_MB_cr])
    ax1.plot([tmp[0] for tmp in x_brake_MB_cr], [tmp[1] for tmp in x_brake_MB_cr])
    ax1.plot([tmp[0] for tmp in x_acc_MB_cr], [tmp[1] for tmp in x_acc_MB_cr])
    ax1.legend(["coasting", "braking", "accelerating", "coasting_cr", "braking_cr", "accelerating_cr"])

    # slip angles
    ax2.set_title("slip angle comparison MB")
    ax2.plot(t, [tmp[10] / tmp[3] for tmp in x_coast_MB])
    ax2.plot(t, [tmp[10] / tmp[3] for tmp in x_brake_MB])
    ax2.plot(t, [tmp[10] / tmp[3] for tmp in x_acc_MB])
    ax2.plot(t, [tmp[10] / tmp[3] for tmp in x_coast_MB_cr])
    ax2.plot(t, [tmp[10] / tmp[3] for tmp in x_brake_MB_cr])
    ax2.plot(t, [tmp[10] / tmp[3] for tmp in x_acc_MB_cr])
    ax2.legend(["coasting", "braking", "accelerating", "coasting_cr", "braking_cr", "accelerating_cr"])

    # orientation
    ax3.set_title("orientation comparison MB")
    ax3.plot(t, [tmp[4] for tmp in x_coast_MB])
    ax3.plot(t, [tmp[4] for tmp in x_brake_MB])
    ax3.plot(t, [tmp[4] for tmp in x_acc_MB])
    ax3.plot(t, [tmp[4] for tmp in x_coast_MB_cr])
    ax3.plot(t, [tmp[4] for tmp in x_brake_MB_cr])
    ax3.plot(t, [tmp[4] for tmp in x_acc_MB_cr])
    ax3.legend(["coasting", "braking", "accelerating", "coasting_cr", "braking_cr", "accelerating_cr"])

    # pitch
    ax4.set_title("pitch comparison MB")
    ax4.plot(t, [tmp[8] for tmp in x_coast_MB])
    ax4.plot(t, [tmp[8] for tmp in x_brake_MB])
    ax4.plot(t, [tmp[8] for tmp in x_acc_MB])
    ax4.plot(t, [tmp[8] for tmp in x_coast_MB_cr])
    ax4.plot(t, [tmp[8] for tmp in x_brake_MB_cr])
    ax4.plot(t, [tmp[8] for tmp in x_acc_MB_cr])
    ax4.legend(["coasting", "braking", "accelerating", "coasting_cr", "braking_cr", "accelerating_cr"])

    plt.tight_layout()
    plt.show()


def compare_mb_braking():
    t = np.arange(0, tFinal, 0.01)
    v_delta = 0.0
    acc = -0.7 * g
    u = [v_delta, acc]

    # simulate car
    x_brake_MB = odeint(func_MB, x0_MB, t, args=(u, p))
    x_brake_MB_cr = odeint(func_MB_cr, x0_MB_cr, t, args=(u, p_cr))

    # Create figure with 3 subplots (1 row, 3 columns)
    _, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(15, 4))

    # position
    ax1.set_title("position")
    ax1.plot([tmp[0] for tmp in x_brake_MB], [tmp[1] for tmp in x_brake_MB])
    ax1.plot([tmp[0] for tmp in x_brake_MB_cr], [tmp[1] for tmp in x_brake_MB_cr])
    ax1.legend(["MB", "MB_CR"])

    # velocity
    ax2.set_title("velocity")
    ax2.plot(t, [tmp[3] for tmp in x_brake_MB])
    ax2.plot(t, [tmp[3] for tmp in x_brake_MB_cr])
    ax2.legend(["MB", "MB_CR"])

    # wheel spin
    ax3.set_title("wheel spin MB braking")
    ax3.plot(t, [tmp[23] for tmp in x_brake_MB])
    ax3.plot(t, [tmp[24] for tmp in x_brake_MB])
    ax3.plot(t, [tmp[25] for tmp in x_brake_MB])
    ax3.plot(t, [tmp[26] for tmp in x_brake_MB])
    ax3.plot(t, [tmp[23] for tmp in x_brake_MB_cr])
    ax3.plot(t, [tmp[24] for tmp in x_brake_MB_cr])
    ax3.plot(t, [tmp[25] for tmp in x_brake_MB_cr])
    ax3.plot(t, [tmp[26] for tmp in x_brake_MB_cr])
    ax3.legend(["MB1", "MB2", "MB3", "MB4", "MB_CR1", "MB_CR2", "MB_CR3", "MB_CR4"])

    plt.tight_layout()
    plt.show()


def compare_mb_accelerating():
    t = np.arange(0, tFinal, 0.01)
    v_delta = 0.0
    acc = 0.63 * g
    u = [v_delta, acc]

    # simulate car
    x_acc_MB = odeint(func_MB, x0_MB, t, args=(u, p))
    x_acc_MB_cr = odeint(func_MB_cr, x0_MB_cr, t, args=(u, p_cr))

    # Create figure with 3 subplots (1 row, 3 columns)
    _, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(15, 4))

    # position
    ax1.set_title("position")
    ax1.plot([tmp[0] for tmp in x_acc_MB], [tmp[1] for tmp in x_acc_MB])
    ax1.plot([tmp[0] for tmp in x_acc_MB_cr], [tmp[1] for tmp in x_acc_MB_cr])
    ax1.legend(["MB", "MB_CR"])

    # velocity
    ax2.set_title("velocity")
    ax2.plot(t, [tmp[3] for tmp in x_acc_MB])
    ax2.plot(t, [tmp[3] for tmp in x_acc_MB_cr])
    ax2.legend(["MB", "MB_CR"])

    # wheel spin
    ax3.set_title("wheel spin MB")
    ax3.plot(t, [tmp[23] for tmp in x_acc_MB])
    ax3.plot(t, [tmp[24] for tmp in x_acc_MB])
    ax3.plot(t, [tmp[25] for tmp in x_acc_MB])
    ax3.plot(t, [tmp[26] for tmp in x_acc_MB])
    ax3.plot(t, [tmp[23] for tmp in x_acc_MB_cr])
    ax3.plot(t, [tmp[24] for tmp in x_acc_MB_cr])
    ax3.plot(t, [tmp[25] for tmp in x_acc_MB_cr])
    ax3.plot(t, [tmp[26] for tmp in x_acc_MB_cr])
    ax3.legend(["MB1", "MB2", "MB3", "MB4", "MB_CR1", "MB_CR2", "MB_CR3", "MB_CR4"])

    plt.tight_layout()
    plt.show()


# run simulations *****************
if __name__ == "__main__":
    """
    1. oversteer_understeer_STD() - Currently active (line 303). Tests the STD model under
    coasting, braking, and accelerating conditions.
    2. cornering_left(0.15, 0) - Line 301. Compares all models (MB, ST, KS, STD) during a
    left turn.
    3. braking() - Line 305. Compares MB and STD models during braking.
    4. accelerating() - Line 306. Compares MB and STD models during acceleration.
    """
    compare_mb_cornering_left(0.15, 0)
    compare_mb_oversteer_understeer()
    compare_mb_braking()
    compare_mb_accelerating()
