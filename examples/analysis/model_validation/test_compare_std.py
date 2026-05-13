"""
Compares behaviour of f110 gym std model with reference commonroad std model to confirm matching behaviour.
"""

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import matplotlib.pyplot as plt
import numpy as np
from commonroad.vehiclemodels.init_std import init_std as init_std_cr
from commonroad.vehiclemodels.parameters_vehicle1 import parameters_vehicle1
from commonroad.vehiclemodels.vehicle_dynamics_std import vehicle_dynamics_std as cr_vehicle_dynamics_std
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


def func_STD_cr(x, t, u, p):
    f = cr_vehicle_dynamics_std(x, u, p)
    return f


x0_STD_cr = init_std_cr(initialState, p_cr)


# ==================
# F1TENTH
# ==================
# f110 params
p = GKEnv.fullscale_vehicle_params()


def func_STD(x, t, u, p):
    f = vehicle_dynamics_std(x, u, p)
    return f


x0_STD = init_std(initialState, p)  # initial state for std model

# --------------------------------------------------------------------------


def compare_std_cornering_left(v_delta, a_long):
    # steering to left
    t = np.arange(0, tFinal, 0.01)
    u = [v_delta, a_long]

    # simulate multibody
    x_left_STD = odeint(func_STD, x0_STD, t, args=(u, p))
    x_left_STD_cr = odeint(func_STD_cr, x0_STD_cr, t, args=(u, p_cr))

    # Create figure with 2 subplots (1 row, 2 columns)
    _, (ax1, ax2) = plt.subplots(1, 2, figsize=(10, 4))

    # results
    # position
    ax1.set_title("positions turning")
    ax1.plot([tmp[0] for tmp in x_left_STD], [tmp[1] for tmp in x_left_STD])
    ax1.plot([tmp[0] for tmp in x_left_STD_cr], [tmp[1] for tmp in x_left_STD_cr])
    ax1.legend(["STD", "STD_cr"])

    # slip angle
    ax2.set_title("slip angle turning")
    ax2.plot(t, [tmp[6] for tmp in x_left_STD])
    ax2.plot(t, [tmp[6] for tmp in x_left_STD_cr])
    ax2.legend(["STD", "STD_cr"])

    plt.tight_layout()
    plt.show()


def compare_std_oversteer_understeer():
    t = np.arange(0, tFinal, 0.01)
    v_delta = 0.15

    # coasting
    u = [v_delta, 0]
    x_coast_STD = odeint(func_STD, x0_STD, t, args=(u, p))
    x_coast_STD_cr = odeint(func_STD_cr, x0_STD_cr, t, args=(u, p_cr))

    # braking
    u = [v_delta, -0.75 * g]
    x_brake_STD = odeint(func_STD, x0_STD, t, args=(u, p))
    x_brake_STD_cr = odeint(func_STD_cr, x0_STD_cr, t, args=(u, p_cr))

    # accelerating
    u = [v_delta, 0.63 * g]
    x_acc_STD = odeint(func_STD, x0_STD, t, args=(u, p))
    x_acc_STD_cr = odeint(func_STD_cr, x0_STD_cr, t, args=(u, p_cr))

    # Create figure with 3 subplots (1 row, 3 columns)
    _, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(15, 4))

    # position
    ax1.set_title("position comparison STD")
    ax1.plot([tmp[0] for tmp in x_coast_STD], [tmp[1] for tmp in x_coast_STD])
    ax1.plot([tmp[0] for tmp in x_brake_STD], [tmp[1] for tmp in x_brake_STD])
    ax1.plot([tmp[0] for tmp in x_acc_STD], [tmp[1] for tmp in x_acc_STD])
    ax1.plot([tmp[0] for tmp in x_coast_STD_cr], [tmp[1] for tmp in x_coast_STD_cr])
    ax1.plot([tmp[0] for tmp in x_brake_STD_cr], [tmp[1] for tmp in x_brake_STD_cr])
    ax1.plot([tmp[0] for tmp in x_acc_STD_cr], [tmp[1] for tmp in x_acc_STD_cr])
    ax1.legend(["coasting", "braking", "accelerating", "coasting_cr", "braking_cr", "accelerating_cr"])

    # compare slip angles
    ax2.set_title("slip angle comparison STD")
    ax2.plot(t, [tmp[6] for tmp in x_coast_STD])
    ax2.plot(t, [tmp[6] for tmp in x_brake_STD])
    ax2.plot(t, [tmp[6] for tmp in x_acc_STD])
    ax2.plot(t, [tmp[6] for tmp in x_coast_STD_cr])
    ax2.plot(t, [tmp[6] for tmp in x_brake_STD_cr])
    ax2.plot(t, [tmp[6] for tmp in x_acc_STD_cr])
    ax2.legend(["coasting", "braking", "accelerating", "coasting_cr", "braking_cr", "accelerating_cr"])

    # orientation
    ax3.set_title("orientation comparison STD")
    ax3.plot(t, [tmp[4] for tmp in x_coast_STD])
    ax3.plot(t, [tmp[4] for tmp in x_brake_STD])
    ax3.plot(t, [tmp[4] for tmp in x_acc_STD])
    ax3.plot(t, [tmp[4] for tmp in x_coast_STD_cr])
    ax3.plot(t, [tmp[4] for tmp in x_brake_STD_cr])
    ax3.plot(t, [tmp[4] for tmp in x_acc_STD_cr])
    ax3.legend(["coasting", "braking", "accelerating", "coasting_cr", "braking_cr", "accelerating_cr"])

    plt.tight_layout()
    plt.show()


def compare_std_braking():
    t = np.arange(0, tFinal, 0.01)
    v_delta = 0.0
    acc = -0.7 * g
    u = [v_delta, acc]

    # simulate car
    x_brake_STD = odeint(func_STD, x0_STD, t, args=(u, p))
    x_brake_STD_cr = odeint(func_STD_cr, x0_STD_cr, t, args=(u, p_cr))

    # Create figure with 3 subplots (1 row, 3 columns)
    _, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(15, 4))

    # position
    ax1.set_title("position")
    ax1.plot([tmp[0] for tmp in x_brake_STD], [tmp[1] for tmp in x_brake_STD])
    ax1.plot([tmp[0] for tmp in x_brake_STD_cr], [tmp[1] for tmp in x_brake_STD_cr])
    ax1.legend(["STD", "STD_CR"])

    # velocity
    ax2.set_title("velocity")
    ax2.plot(t, [tmp[3] * math.cos(tmp[6]) for tmp in x_brake_STD])
    ax2.plot(t, [tmp[3] * math.cos(tmp[6]) for tmp in x_brake_STD_cr])
    ax2.legend(["STD", "STD_CR"])

    # wheel spin
    ax3.set_title("wheel spin STD braking")
    ax3.plot(t, [tmp[7] for tmp in x_brake_STD])
    ax3.plot(t, [tmp[8] for tmp in x_brake_STD])
    ax3.plot(t, [tmp[7] for tmp in x_brake_STD_cr])
    ax3.plot(t, [tmp[8] for tmp in x_brake_STD_cr])
    ax3.legend(["STD_1", "STD_2", "STD_cr_1", "STD_cr_2"])

    plt.tight_layout()
    plt.show()


def compare_std_accelerating():
    t = np.arange(0, tFinal, 0.01)
    v_delta = 0.0
    acc = 0.63 * g
    u = [v_delta, acc]

    # simulate car
    x_acc_STD = odeint(func_STD, x0_STD, t, args=(u, p))
    x_acc_STD_cr = odeint(func_STD_cr, x0_STD_cr, t, args=(u, p_cr))

    # Create figure with 3 subplots (1 row, 3 columns)
    _, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(15, 4))

    # position
    ax1.set_title("position")
    ax1.plot([tmp[0] for tmp in x_acc_STD], [tmp[1] for tmp in x_acc_STD])
    ax1.plot([tmp[0] for tmp in x_acc_STD_cr], [tmp[1] for tmp in x_acc_STD_cr])
    ax1.legend(["STD", "STD_CR"])

    # velocity
    ax2.set_title("velocity")
    ax2.plot(t, [tmp[3] * math.cos(tmp[6]) for tmp in x_acc_STD])
    ax2.plot(t, [tmp[3] * math.cos(tmp[6]) for tmp in x_acc_STD_cr])
    ax2.legend(["STD", "STD_CR"])

    # wheel spin
    ax3.set_title("wheel spin")
    ax3.plot(t, [tmp[7] for tmp in x_acc_STD])
    ax3.plot(t, [tmp[8] for tmp in x_acc_STD])
    ax3.plot(t, [tmp[7] for tmp in x_acc_STD_cr])
    ax3.plot(t, [tmp[8] for tmp in x_acc_STD_cr])
    ax3.legend(["STD_1", "STD_2", "STD_CR_1", "STD_CR_2"])

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
    compare_std_cornering_left(0.15, 0)
    compare_std_oversteer_understeer()
    compare_std_braking()
    compare_std_accelerating()
