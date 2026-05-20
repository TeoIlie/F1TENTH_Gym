"""
Single-track drift model initialization function
"""

import numpy as np
from numba import njit

from .single_track_drift import get_standardized_state_std, vehicle_dynamics_std


def init_std(init_state, params: dict, compute_wheel_speeds: bool = True) -> np.ndarray:
    """
    init_std generates the initial state vector for the drift single track model.

    Indices 0-6 of ``init_state`` are the user-facing core states
    ``[x, y, delta, v, yaw, yaw_rate, slip]``; indices 7-8 hold the front
    and rear wheel angular velocities ``omega_f, omega_r`` in rad/s.
    Steering and longitudinal velocity are clamped to the parameter limits
    in either case.

    Syntax:
        x0 = init_std(init_state, p)
        x0 = init_std(init_state, p, compute_wheel_speeds=False)

    Inputs:
        :param init_state: 9-wide pre-allocated initial state vector. Indices
            7-8 are only read when ``compute_wheel_speeds`` is False.
        :param params: parameter vector
        :param compute_wheel_speeds: if True (default), derive ``omega_f`` and
            ``omega_r`` (indices 7, 8) from the no-slip rolling assumption
            using the clamped ``v``, ``delta`` and ``slip``. If False, the
            wheel speeds already in ``init_state[7:9]`` are taken as given

    Outputs:
        :return x0: initial state vector

    Author: Teodor Ilie
    Written: 18-October-2025
    """
    x0 = np.zeros((9,))

    # Steering, vel constraints

    delta0 = init_state[2]  # steering angle of front wheels
    vel0 = init_state[3]  # speed of the car

    ## steering constraints
    s_min = params["s_min"]  # minimum steering angle [rad]
    s_max = params["s_max"]  # maximum steering angle [rad]

    ## longitudinal constraints
    v_min = params["v_min"]  # minimum velocity [m/s]
    v_max = params["v_max"]  # minimum velocity [m/s]

    if delta0 > s_max:
        delta0 = s_max

    if delta0 < s_min:
        delta0 = s_min

    if vel0 > v_max:
        vel0 = v_max

    if vel0 < v_min:
        vel0 = v_min

    # Copy first 7 states as-is with constraints on steering, velocity
    x0[0] = init_state[0]
    x0[1] = init_state[1]
    x0[2] = delta0
    x0[3] = vel0
    x0[4] = init_state[4]
    x0[5] = init_state[5]
    x0[6] = init_state[6]

    # Wheel angular velocities: derive from no-slip rolling, or take as given.
    if compute_wheel_speeds:
        x0[7] = x0[3] * np.cos(x0[6]) * np.cos(x0[2]) / params["R_w"]  # init front wheel angular speed
        x0[8] = x0[3] * np.cos(x0[6]) / params["R_w"]  # init rear wheel angular speed
    else:
        x0[7] = init_state[7]
        x0[8] = init_state[8]

    return x0
