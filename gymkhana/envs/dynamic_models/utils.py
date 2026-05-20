"""Constraint and controller utilities for vehicle dynamics models."""

import numpy as np
from numba import njit


@njit(cache=True)
def upper_accel_limit(vel, a_max, v_switch):
    """Compute the upper acceleration limit based on velocity.

    Above ``v_switch``, the maximum acceleration scales down inversely with
    speed to model the limit at which acceleration can no longer create wheel spin.

    Args:
        vel: Current velocity of the vehicle.
        a_max: Maximum allowed acceleration (symmetric).
        v_switch: Velocity above which the limit begins to scale down.

    Returns:
        Adjusted positive acceleration limit.
    """
    if vel > v_switch:
        pos_limit = a_max * (v_switch / vel)
    else:
        pos_limit = a_max

    return pos_limit


@njit(cache=True)
def accl_constraints(vel, a_long_d, v_switch, a_max, v_min, v_max):
    """Apply acceleration constraints based on velocity bounds and limits.

    Args:
        vel: Current velocity of the vehicle.
        a_long_d: Unconstrained desired acceleration in the direction of travel.
        v_switch: Velocity at which acceleration can no longer create wheel spin.
        a_max: Maximum allowed acceleration (symmetric).
        v_min: Minimum allowed velocity.
        v_max: Maximum allowed velocity.

    Returns:
        Constrained acceleration.
    """

    uac = upper_accel_limit(vel, a_max, v_switch)

    if (vel <= v_min and a_long_d <= 0) or (vel >= v_max and a_long_d >= 0):
        a_long = 0.0
    elif a_long_d <= -a_max:
        a_long = -a_max
    elif a_long_d >= uac:
        a_long = uac
    else:
        a_long = a_long_d

    return a_long


@njit(cache=True)
def steering_constraint(steering_angle, steering_velocity, s_min, s_max, sv_min, sv_max):
    """Apply steering constraints based on angle bounds and velocity limits.

    Args:
        steering_angle: Current steering angle of the vehicle.
        steering_velocity: Unconstrained desired steering velocity.
        s_min: Minimum steering angle.
        s_max: Maximum steering angle.
        sv_min: Minimum steering velocity.
        sv_max: Maximum steering velocity.

    Returns:
        Constrained steering velocity.
    """

    # constraint steering velocity
    if (steering_angle <= s_min and steering_velocity <= 0) or (steering_angle >= s_max and steering_velocity >= 0):
        steering_velocity = 0.0
    elif steering_velocity <= sv_min:
        steering_velocity = sv_min
    elif steering_velocity >= sv_max:
        steering_velocity = sv_max

    return steering_velocity


@njit(cache=True)
def bang_bang_steer(steer, current_steer, max_sv):
    """Bang-bang steering controller.

    Outputs maximum steering velocity in the direction of the error,
    creating an aggressive steering response with deliberate lag.

    Args:
        steer: Desired steering angle.
        current_steer: Current steering angle.
        max_sv: Maximum steering velocity.

    Returns:
        Steering velocity command (``+max_sv``, ``-max_sv``, or ``0``).
    """
    # steering
    steer_diff = steer - current_steer
    if np.fabs(steer_diff) > 1e-4:
        sv = (steer_diff / np.fabs(steer_diff)) * max_sv
    else:
        sv = 0.0

    return sv


@njit(cache=True)
def p_accl(speed, current_speed, max_a, max_v, min_v):
    """Proportional controller converting a target speed to an acceleration command.

    Gain is scaled by ``max_a / max_v`` (or ``max_a / abs(min_v)`` when braking),
    with separate forward and reverse gain schedules.

    Args:
        speed: Desired target speed.
        current_speed: Current vehicle speed.
        max_a: Maximum allowed acceleration.
        max_v: Maximum allowed velocity.
        min_v: Minimum allowed velocity (negative for reverse).

    Returns:
        Acceleration command.
    """
    # accl
    vel_diff = speed - current_speed
    # currently forward
    if current_speed > 0.0:
        if vel_diff > 0:
            # accelerate
            kp = 10.0 * max_a / max_v
            accl = kp * vel_diff
        else:
            # braking
            kp = 10.0 * max_a / (-min_v)
            accl = kp * vel_diff
    # currently backwards
    else:
        if vel_diff > 0:
            # braking
            kp = 2.0 * max_a / max_v
            accl = kp * vel_diff
        else:
            # accelerating
            kp = 2.0 * max_a / (-min_v)
            accl = kp * vel_diff

    return accl
