import numpy as np


def format_float(x):
    """
    Format observation, showing "Approx 0" when very small
    """
    if abs(x) < 1e-5:
        return "Approx 0"
    else:
        return f"{x:.4f}"


def display_drift_obs(step, obs, reward, lookahead_n_points, total_reward=None):
    """
    Format and print "drift" observation type
    """
    vx = obs[0]
    vy = obs[1]
    heading_error_radians = obs[2]
    heading_error_degrees = np.degrees(heading_error_radians)
    lateral_dist = obs[3]
    yaw_rate = obs[4]
    delta = obs[5]
    beta = obs[6]
    prev_steer_cmd = obs[7]
    prev_throttle_cmd = obs[8]
    prev_avg_wheel_omega = obs[9]
    integrated_vel_cmd = obs[10]
    curvatures = obs[11 : 11 + lookahead_n_points]
    widths = obs[11 + lookahead_n_points : 11 + (2 * lookahead_n_points)]

    print(f"\n=====================\nStep {step + 1}:\n=====================\n")

    print(
        f"  vx={vx:6.2f}, vy={vy:6.2f}, yaw_rate={yaw_rate:6.2f}, delta={delta:6.4f}, beta={beta:6.4f}\n"
        f"  heading error (degrees)={heading_error_degrees:6.2f}, lateral distance={lateral_dist:6.2f}\n"
        f"  previous steering cmd={prev_steer_cmd:6.4f}\n"
        f"  previous throttle cmd={prev_throttle_cmd:6.4f}\n"
        f"  previous average wheel ang speed={prev_avg_wheel_omega:6.4f}\n"
        f"  integrated velocity command={integrated_vel_cmd:6.4f}\n"
        f"  curvature lookahead:"
    )
    for i, value in enumerate(curvatures, start=1):
        print(f"    Point {i} = {format_float(value)}")

    print("  widths lookahead:")
    for i, value in enumerate(widths, start=1):
        print(f"    Point {i} = {format_float(value)}")

    print(f"\n  Reward = {reward}")

    if total_reward is not None:
        print(f"  Total episode reward = {total_reward}\n")


def display_kinematic_state_obs(step, obs, reward, total_reward=None):
    """
    Format and print "kinematic_state" observation type
    """
    agent_obs = obs["agent_0"]
    pose_x = float(agent_obs["pose_x"])
    pose_y = float(agent_obs["pose_y"])
    delta = float(agent_obs["delta"])
    vx = float(agent_obs["linear_vel_x"])
    pose_theta = float(agent_obs["pose_theta"])
    pose_theta_degrees = np.degrees(pose_theta)

    print(f"\n=====================\nStep {step + 1}:\n=====================\n")
    print(
        f"  pose_x={pose_x:8.4f}, pose_y={pose_y:8.4f}\n"
        f"  heading={pose_theta:8.4f} rad ({pose_theta_degrees:6.2f} deg)\n"
        f"  vx={vx:6.2f}, delta={delta:6.4f}"
    )
    print(f"\n  Reward = {reward}")

    if total_reward is not None:
        print(f"  Total episode reward = {total_reward}\n")


def display_frenet_dynamic_state_obs(step, obs, reward, total_reward=None):
    """
    Format and print "frenet_dynamic_state" observation type. Use for debugging.
    obs is a dict: {"agent_0": {"pose_x": ..., "pose_y": ..., "delta": ...,
                                "linear_vel_x": ..., "linear_vel_y": ...,
                                "pose_theta": ..., "ang_vel_z": ..., "beta": ...}}
    """
    agent_obs = obs["agent_0"]
    pose_x = float(agent_obs["pose_x"])
    pose_y = float(agent_obs["pose_y"])
    delta = float(agent_obs["delta"])
    vx = float(agent_obs["linear_vel_x"])
    vy = float(agent_obs["linear_vel_y"])
    pose_theta = float(agent_obs["pose_theta"])
    pose_theta_degrees = np.degrees(pose_theta)
    ang_vel_z = float(agent_obs["ang_vel_z"])
    beta = float(agent_obs["beta"])

    print(f"\n=====================\nStep {step + 1}:\n=====================\n")
    print(
        f"  pose_x={pose_x:8.4f}, pose_y={pose_y:8.4f}\n"
        f"  heading={pose_theta:8.4f} rad ({pose_theta_degrees:6.2f} deg)\n"
        f"  vx={vx:6.2f}, vy={vy:6.2f}\n"
        f"  yaw_rate={ang_vel_z:6.4f}, delta={delta:6.4f}, beta={beta:6.4f}"
    )
    print(f"\n  Reward = {reward}")

    if total_reward is not None:
        print(f"  Total episode reward = {total_reward}\n")
