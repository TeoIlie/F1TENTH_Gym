"""
Functions for testing different vehicle models (see examples in chap. 11 of documentation)
Use:
1. Choose a tire param for p from GKEnv
2. Choose a function to uncomment in main
3. Run the code and observe the plots
"""

import math

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.pyplot import legend, title
from scipy.integrate import odeint

from gymkhana.envs.dynamic_models import vehicle_dynamics_st_py as vehicle_dynamics_st
from gymkhana.envs.dynamic_models import vehicle_dynamics_std_py as vehicle_dynamics_std
from gymkhana.envs.dynamic_models.multi_body import init_mb
from gymkhana.envs.dynamic_models.multi_body.multi_body import vehicle_dynamics_mb
from gymkhana.envs.dynamic_models.single_track_drift import init_std
from gymkhana.envs.gymkhana_env import GKEnv


def func_ST(x, t, u, p):
    f = vehicle_dynamics_st(x, u, p)
    return f


def func_MB(x, t, u, p):
    f = vehicle_dynamics_mb(x, u, p)
    return f


def func_STD(x, t, u, p):
    f = vehicle_dynamics_std(x, u, p)
    return f


# load parameters
p = GKEnv.fullscale_vehicle_params()
g = 9.81  # [m/s^2]

# set options --------------------------------------------------------------
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
x0_MB = init_mb(initialState, p)  # initial state for multi-body model
x0_STD = init_std(initialState, p)  # initial state for single-track drift model
x0_ST = np.array(initialState)
# --------------------------------------------------------------------------


def cornering_left(v_delta, a_long):
    # steering to left
    t = np.arange(0, tFinal, 0.01)
    u = [v_delta, a_long]

    # simulate multibody
    x_left_mb = odeint(func_MB, x0_MB, t, args=(u, p))

    # simulate single-track model
    x_left_st = odeint(func_ST, x0_ST, t, args=(u, p))

    # simulate single-track drift model
    x_left_std = odeint(func_STD, x0_STD, t, args=(u, p))

    # results
    # position
    title("positions turning")
    plt.plot([tmp[0] for tmp in x_left_mb], [tmp[1] for tmp in x_left_mb])
    plt.plot([tmp[0] for tmp in x_left_st], [tmp[1] for tmp in x_left_st])
    plt.plot([tmp[0] for tmp in x_left_std], [tmp[1] for tmp in x_left_std])
    legend(["MB", "ST", "STD"])
    plt.autoscale()
    plt.show()

    # slip angle
    title("slip angle turning")
    plt.plot(t, [tmp[10] / tmp[3] for tmp in x_left_mb])
    plt.plot(t, [tmp[6] for tmp in x_left_st])
    plt.plot(t, [tmp[6] for tmp in x_left_std])
    legend(["MB", "ST", "STD"])
    plt.show()


def oversteer_understeer_MB():
    t = np.arange(0, tFinal, 0.01)
    v_delta = 0.15

    # coasting
    u = [v_delta, 0]
    x_coast = odeint(func_MB, x0_MB, t, args=(u, p))

    # braking
    u = [v_delta, -0.7 * g]
    x_brake = odeint(func_MB, x0_MB, t, args=(u, p))

    # accelerating
    u = [v_delta, 0.63 * g]
    x_acc = odeint(func_MB, x0_MB, t, args=(u, p))

    # position
    title("position comparison MB")
    plt.plot([tmp[0] for tmp in x_coast], [tmp[1] for tmp in x_coast])
    plt.plot([tmp[0] for tmp in x_brake], [tmp[1] for tmp in x_brake])
    plt.plot([tmp[0] for tmp in x_acc], [tmp[1] for tmp in x_acc])
    legend(["coasting", "braking", "accelerating"])
    plt.show()
    # slip angles
    title("slip angle comparison MB")
    plt.plot(t, [tmp[10] / tmp[3] for tmp in x_coast])
    plt.plot(t, [tmp[10] / tmp[3] for tmp in x_brake])
    plt.plot(t, [tmp[10] / tmp[3] for tmp in x_acc])
    legend(["coasting", "braking", "accelerating"])
    plt.show()
    # orientation
    title("orientation comparison MB")
    plt.plot(t, [tmp[4] for tmp in x_coast])
    plt.plot(t, [tmp[4] for tmp in x_brake])
    plt.plot(t, [tmp[4] for tmp in x_acc])
    legend(["coasting", "braking", "accelerating"])
    plt.show()
    # pitch
    title("pitch comparison MB")
    plt.plot(t, [tmp[8] for tmp in x_coast])
    plt.plot(t, [tmp[8] for tmp in x_brake])
    plt.plot(t, [tmp[8] for tmp in x_acc])
    legend(["coasting", "braking", "accelerating"])
    plt.show()


def oversteer_understeer_STD():
    t = np.arange(0, tFinal, 0.01)
    v_delta = 0.15

    # coasting
    u = [v_delta, 0]
    x_coast = odeint(func_STD, x0_STD, t, args=(u, p))

    # braking
    u = [v_delta, -0.75 * g]
    x_brake = odeint(func_STD, x0_STD, t, args=(u, p))

    # accelerating
    u = [v_delta, 0.63 * g]
    x_acc = odeint(func_STD, x0_STD, t, args=(u, p))

    # position
    title("position comparison STD")
    plt.plot([tmp[0] for tmp in x_coast], [tmp[1] for tmp in x_coast])
    plt.plot([tmp[0] for tmp in x_brake], [tmp[1] for tmp in x_brake])
    plt.plot([tmp[0] for tmp in x_acc], [tmp[1] for tmp in x_acc])
    legend(["coasting", "braking", "accelerating"])
    plt.show()
    # compare slip angles
    title("slip angle comparison STD")
    plt.plot(t, [tmp[6] for tmp in x_coast])
    plt.plot(t, [tmp[6] for tmp in x_brake])
    plt.plot(t, [tmp[6] for tmp in x_acc])
    legend(["coasting", "braking", "accelerating"])
    plt.show()
    # orientation
    title("orientation comparison STD")
    plt.plot(t, [tmp[4] for tmp in x_coast])
    plt.plot(t, [tmp[4] for tmp in x_brake])
    plt.plot(t, [tmp[4] for tmp in x_acc])
    legend(["coasting", "braking", "accelerating"])
    plt.show()


def oversteer_understeer_ST():
    t = np.arange(0, tFinal, 0.01)
    v_delta = 0.15

    # coasting
    u = [v_delta, 0]
    x_coast = odeint(func_ST, x0_ST, t, args=(u, p))

    # braking
    u = [v_delta, -0.75 * g]
    x_brake = odeint(func_ST, x0_ST, t, args=(u, p))

    # accelerating
    u = [v_delta, 0.63 * g]
    x_acc = odeint(func_ST, x0_ST, t, args=(u, p))

    # position
    title("position comparison ST")
    plt.plot([tmp[0] for tmp in x_coast], [tmp[1] for tmp in x_coast])
    plt.plot([tmp[0] for tmp in x_brake], [tmp[1] for tmp in x_brake])
    plt.plot([tmp[0] for tmp in x_acc], [tmp[1] for tmp in x_acc])
    legend(["coasting", "braking", "accelerating"])
    plt.show()
    # compare slip angles
    title("slip angle comparison ST")
    plt.plot(t, [tmp[6] for tmp in x_coast])
    plt.plot(t, [tmp[6] for tmp in x_brake])
    plt.plot(t, [tmp[6] for tmp in x_acc])
    legend(["coasting", "braking", "accelerating"])
    plt.show()
    # orientation
    title("orientation comparison ST")
    plt.plot(t, [tmp[4] for tmp in x_coast])
    plt.plot(t, [tmp[4] for tmp in x_brake])
    plt.plot(t, [tmp[4] for tmp in x_acc])
    legend(["coasting", "braking", "accelerating"])
    plt.show()


def braking():
    t = np.arange(0, tFinal, 0.01)
    v_delta = 0.0
    acc = -0.7 * g
    u = [v_delta, acc]

    # simulate car
    x_brake = odeint(func_MB, x0_MB, t, args=(u, p))
    x_brake_STD = odeint(func_STD, x0_STD, t, args=(u, p))

    # position
    plt.plot([tmp[0] for tmp in x_brake], [tmp[1] for tmp in x_brake])
    plt.plot([tmp[0] for tmp in x_brake_STD], [tmp[1] for tmp in x_brake_STD])
    plt.title("position")
    legend(["MB", "STD"])
    plt.show()

    # velocity
    plt.plot(t, [tmp[3] for tmp in x_brake])
    plt.plot(t, [tmp[3] * math.cos(tmp[6]) for tmp in x_brake_STD])
    plt.title("velocity")
    legend(["MB", "STD"])
    plt.show()

    # wheel spin
    title("wheel spin MB braking")
    plt.plot(t, [tmp[23] for tmp in x_brake])
    plt.plot(t, [tmp[24] for tmp in x_brake])
    plt.plot(t, [tmp[25] for tmp in x_brake])
    plt.plot(t, [tmp[26] for tmp in x_brake])
    plt.show()
    title("wheel spin STD braking")
    plt.plot(t, [tmp[7] for tmp in x_brake_STD])
    plt.plot(t, [tmp[8] for tmp in x_brake_STD])
    plt.show()


def accelerating():
    t = np.arange(0, tFinal, 0.01)
    v_delta = 0.0
    acc = 0.63 * g
    u = [v_delta, acc]

    # simulate car
    x_acc = odeint(func_MB, x0_MB, t, args=(u, p))
    x_acc_STD = odeint(func_STD, x0_STD, t, args=(u, p))
    x_acc_ST = odeint(func_ST, x0_ST, t, args=(u, p))

    # position
    plt.plot([tmp[0] for tmp in x_acc], [tmp[1] for tmp in x_acc])
    plt.plot([tmp[0] for tmp in x_acc_STD], [tmp[1] for tmp in x_acc_STD])
    plt.plot([tmp[0] for tmp in x_acc_ST], [tmp[1] for tmp in x_acc_ST])
    plt.title("position")
    legend(["MB", "STD", "ST"])
    plt.show()

    # velocity
    plt.plot(t, [tmp[3] for tmp in x_acc])
    plt.plot(t, [tmp[3] * math.cos(tmp[6]) for tmp in x_acc_STD])
    plt.title("velocity")
    legend(["MB", "STD"])
    plt.show()

    # wheel spin
    title("wheel spin MB")
    plt.plot(t, [tmp[23] for tmp in x_acc])
    plt.plot(t, [tmp[24] for tmp in x_acc])
    plt.plot(t, [tmp[25] for tmp in x_acc])
    plt.plot(t, [tmp[26] for tmp in x_acc])
    plt.show()
    title("wheel spin STD")
    plt.plot(t, [tmp[7] for tmp in x_acc_STD])
    plt.plot(t, [tmp[8] for tmp in x_acc_STD])
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
    cornering_left(0.15, 0)
    # oversteer_understeer_MB()
    # oversteer_understeer_STD()
    # oversteer_understeer_ST()
    # braking()
    # accelerating()
