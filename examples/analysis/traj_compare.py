"""Sim2Real trajectory comparison.

Replays control commands from a real car recording (100Hz NPZ) through the
gymkhana simulator and produces a side-by-side comparison plot (XY trajectory,
velocity, steering) saved as a single image.

Usage:
    python examples/analysis/traj_compare.py --path /path/to/bag_100Hz.npz --model ks
    python examples/analysis/traj_compare.py --path /path/to/bag_100Hz.npz --model st
    python examples/analysis/traj_compare.py --path /path/to/bag_100Hz.npz --model std
    python examples/analysis/traj_compare.py --path /path/to/bag_100Hz.npz --model stp

Output:
    figures/analysis/traj_compare/<bag_stem>/plt_<model>.png
"""

import argparse
import os
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import gymnasium as gym
import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import savgol_filter

from gymkhana.envs.gymkhana_env import GKEnv


def main():
    parser = argparse.ArgumentParser(description="Sim2Real trajectory comparison")
    parser.add_argument("--path", required=True, help="Path to 100Hz resampled NPZ file")
    parser.add_argument("--model", required=True, choices=["ks", "st", "std", "stp"], help="Vehicle dynamics model")
    parser.add_argument("--start", type=int, default=None, help="Start timestep index (inclusive, 0-based)")
    parser.add_argument("--end", type=int, default=None, help="End timestep index (inclusive, 0-based)")
    args = parser.parse_args()

    # Load real data
    data = np.load(args.path)
    bag_len = len(data["t"])
    start = args.start if args.start is not None else 0
    end = args.end if args.end is not None else bag_len - 1
    if not (0 <= start < bag_len):
        raise ValueError(f"--start must be in [0, {bag_len - 1}], got {start}")
    if not (start <= end < bag_len):
        raise ValueError(f"--end must be in [{start}, {bag_len - 1}], got {end}")
    sl = slice(start, end + 1)
    is_sliced = (start != 0) or (end != bag_len - 1)

    t = data["t"][sl]
    cmd_speed = data["cmd_speed"][sl]
    cmd_steer = data["cmd_steer"][sl]
    vicon_x = data["vicon_x"][sl]
    vicon_y = data["vicon_y"][sl]
    vicon_yaw = data["vicon_yaw"][sl]
    vicon_body_vx = data["vicon_body_vx"][sl]
    vicon_body_vy = data["vicon_body_vy"][sl]
    vicon_r = data["vicon_r"][sl]
    rs_core_speed = data["rs_core_speed"][sl] if "rs_core_speed" in data.files else None

    # Output path
    stem = Path(args.path).stem
    out_dir = os.path.join("figures", "analysis", "traj_compare", stem)
    os.makedirs(out_dir, exist_ok=True)
    suffix = f"_{start}_{end}" if is_sliced else ""
    out_path = os.path.join(out_dir, f"plt_{args.model}{suffix}.png")

    # Select params based on model
    if args.model == "std":
        params = GKEnv.f1tenth_std_vehicle_params()
    elif args.model == "stp":
        params = GKEnv.f1tenth_stp_vehicle_params()
    else:
        params = GKEnv.f1tenth_vehicle_params()

    # Create environment
    config = {
        "model": args.model,
        "num_agents": 1,
        "timestep": 0.01,
        "map": "Spielberg_blank",
        "control_input": ["speed", "steering_angle"],
        "observation_config": {"type": "dynamic_state"},
        "normalize_obs": False,
        "normalize_act": False,
        "params": params,
    }
    env = gym.make("gymkhana:gymkhana-v0", config=config)
    obs, _ = env.reset(options={"poses": np.array([[vicon_x[0], vicon_y[0], vicon_yaw[0]]])})

    has_wheel_omegas = args.model == "std"
    sim_omega_front = []
    sim_omega_rear = []
    if has_wheel_omegas:
        state = env.unwrapped.sim.agents[0].state
        sim_omega_front.append(float(state[7]))
        sim_omega_rear.append(float(state[8]))

    # Record initial state
    agent_obs = obs["agent_0"]
    sim_x = [float(agent_obs["pose_x"])]
    sim_y = [float(agent_obs["pose_y"])]
    sim_vx = [float(agent_obs["linear_vel_x"])]
    sim_vy = [float(agent_obs["linear_vel_y"])]
    sim_delta = [float(agent_obs["delta"])]
    sim_yaw_rate = [float(agent_obs["ang_vel_z"])]
    sim_beta = [float(agent_obs["beta"])]
    sim_cmd_speed = [cmd_speed[0]]
    sim_cmd_steer = [cmd_steer[0]]

    # Replay commands
    for i in range(len(cmd_speed)):
        action = np.array([[cmd_steer[i], cmd_speed[i]]])
        obs, _, _, _, _ = env.step(action)

        agent_obs = obs["agent_0"]
        sim_x.append(float(agent_obs["pose_x"]))
        sim_y.append(float(agent_obs["pose_y"]))
        sim_vx.append(float(agent_obs["linear_vel_x"]))
        sim_vy.append(float(agent_obs["linear_vel_y"]))
        sim_delta.append(float(agent_obs["delta"]))
        sim_yaw_rate.append(float(agent_obs["ang_vel_z"]))
        sim_beta.append(float(agent_obs["beta"]))
        sim_cmd_speed.append(cmd_speed[i])
        sim_cmd_steer.append(cmd_steer[i])
        if has_wheel_omegas:
            state = env.unwrapped.sim.agents[0].state
            sim_omega_front.append(float(state[7]))
            sim_omega_rear.append(float(state[8]))

    env.close()

    sim_omega_front = np.array(sim_omega_front)
    sim_omega_rear = np.array(sim_omega_rear)

    # Convert to arrays
    sim_x = np.array(sim_x)
    sim_y = np.array(sim_y)
    sim_vx = np.array(sim_vx)
    sim_vy = np.array(sim_vy)
    sim_delta = np.array(sim_delta)
    sim_yaw_rate = np.array(sim_yaw_rate)
    sim_beta = np.array(sim_beta)
    sim_cmd_speed = np.array(sim_cmd_speed)
    sim_cmd_steer = np.array(sim_cmd_steer)

    # Time axis: sim has n points starting from t=0 with 0.01 spacing
    n = len(sim_x)
    sim_t = np.arange(n) * 0.01 + float(t[0])

    # Trim real data to match sim duration
    n_real = min(len(t), n)

    # Summary
    model_label = args.model.upper()
    final_err = np.hypot(vicon_x[n_real - 1] - sim_x[n_real - 1], vicon_y[n_real - 1] - sim_y[n_real - 1])
    print(f"Model: {model_label}")
    print(f"Steps simulated: {n - 1}")
    print(f"Duration: {sim_t[-1]:.2f}s")
    print(f"Final position error (real vs sim): {final_err:.4f} m")

    # Derive longitudinal acceleration from velocity via smoothed differentiation
    real_accl = np.gradient(savgol_filter(vicon_body_vx, window_length=21, polyorder=2), t)
    sim_accl = np.gradient(savgol_filter(sim_vx, window_length=11, polyorder=2), sim_t)

    # Real slip angle from body-frame Vicon velocities, masked at low speed
    real_speed = np.hypot(vicon_body_vx, vicon_body_vy)
    real_beta = np.arctan2(vicon_body_vy, vicon_body_vx)
    real_beta = np.where(real_speed > 0.2, real_beta, np.nan)

    fig, axes_grid = plt.subplots(2, 4, figsize=(32, 14))
    range_label = f" [{start}:{end}]" if is_sliced else ""
    fig.suptitle(f"Sim2Real Comparison — {model_label} model ({stem}){range_label}", fontsize=14)
    axes = [
        axes_grid[0, 0],  # XY
        axes_grid[0, 1],  # vx
        axes_grid[0, 2],  # vy
        axes_grid[1, 0],  # steering
        axes_grid[1, 1],  # yaw rate
        axes_grid[1, 2],  # long. accel
        axes_grid[1, 3],  # slip angle
        axes_grid[0, 3],  # wheel angular velocities
    ]

    # --- Plot 1: XY Trajectory ---
    ax = axes[0]
    ax.plot(vicon_x[:n_real], vicon_y[:n_real], label="Real (Vicon)", linewidth=1.5)
    ax.plot(sim_x[:n_real], sim_y[:n_real], label=f"Sim ({model_label})", linewidth=1.5, linestyle="--")
    ax.plot(vicon_x[0], vicon_y[0], "ko", markersize=8, label="Start")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title("XY Trajectory")
    ax.set_aspect("equal")
    ax.legend()
    ax.grid(True, alpha=0.3)

    # --- Plot 2: Longitudinal velocity (vx) ---
    ax = axes[1]
    ax.plot(t[:n_real], cmd_speed[:n_real], label="Commanded speed", linewidth=1, alpha=0.7)
    ax.plot(t[:n_real], vicon_body_vx[:n_real], label="Real vx (Vicon)", linewidth=1)
    ax.plot(sim_t[:n_real], sim_cmd_speed[:n_real], label="Sim commanded speed", linewidth=1, linestyle="--")
    ax.plot(sim_t[:n_real], sim_vx[:n_real], label="Sim vx", linewidth=1.5)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Velocity (m/s)")
    ax.set_title("Longitudinal Velocity (vx) — Command vs Actual")
    ax.legend()
    ax.grid(True, alpha=0.3)

    # --- Plot 3: Lateral velocity (vy) ---
    ax = axes[2]
    ax.plot(t[:n_real], vicon_body_vy[:n_real], label="Real vy (Vicon)", linewidth=1)
    ax.plot(sim_t[:n_real], sim_vy[:n_real], label=f"Sim vy ({model_label})", linewidth=1.5, linestyle="--")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Velocity (m/s)")
    ax.set_title("Lateral Velocity (vy) — Real vs Sim")
    ax.legend()
    ax.grid(True, alpha=0.3)

    # --- Plot 4: Steering ---
    ax = axes[3]
    ax.plot(t[:n_real], cmd_steer[:n_real], label="Commanded steering", linewidth=1, alpha=0.7)
    ax.plot(sim_t[:n_real], sim_cmd_steer[:n_real], label="Sim commanded steering", linewidth=1, linestyle="--")
    ax.plot(sim_t[:n_real], sim_delta[:n_real], label="Sim actual delta", linewidth=1.5)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Steering (rad)")
    ax.set_title("Steering — Command vs Actual")
    ax.legend()
    ax.grid(True, alpha=0.3)

    # --- Plot 5: Yaw rate ---
    ax = axes[4]
    ax.plot(t[:n_real], vicon_r[:n_real], label="Real yaw rate (Vicon)", linewidth=1)
    ax.plot(sim_t[:n_real], sim_yaw_rate[:n_real], label=f"Sim yaw rate ({model_label})", linewidth=1.5, linestyle="--")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Yaw rate (rad/s)")
    ax.set_title("Yaw Rate — Real vs Sim")
    ax.legend()
    ax.grid(True, alpha=0.3)

    # --- Plot 6: Longitudinal acceleration ---
    ax = axes[5]
    ax.plot(t[:n_real], real_accl[:n_real], label="Real dv/dt (Vicon)", linewidth=1)
    ax.plot(sim_t[:n_real], sim_accl[:n_real], label=f"Sim dv/dt ({model_label})", linewidth=1.5, linestyle="--")
    ax.axhline(params["a_max"], color="red", linewidth=0.5, linestyle=":", label="±a_max")
    ax.axhline(-params["a_max"], color="red", linewidth=0.5, linestyle=":")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Acceleration (m/s²)")
    ax.set_title("Longitudinal Acceleration — Real vs Sim")
    ax.legend()
    ax.grid(True, alpha=0.3)

    # --- Plot 7: Slip angle ---
    ax = axes[6]
    ax.plot(t[:n_real], real_beta[:n_real], label="Real slip (Vicon)", linewidth=1)
    ax.plot(sim_t[:n_real], sim_beta[:n_real], label=f"Sim slip ({model_label})", linewidth=1.5, linestyle="--")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Slip angle β (rad)")
    ax.set_title("Slip Angle — Real vs Sim")
    ax.legend()
    ax.grid(True, alpha=0.3)

    # --- Plot 8: Wheel angular velocities ---
    ax = axes[7]
    if has_wheel_omegas:
        R_w = params["R_w"]
        ax.plot(sim_t[:n_real], sim_omega_front[:n_real], label="Sim ω_front", linewidth=1.5, linestyle="--")
        ax.plot(sim_t[:n_real], sim_omega_rear[:n_real], label="Sim ω_rear", linewidth=1.5, linestyle="--")
        if rs_core_speed is not None:
            real_omega = rs_core_speed / R_w
            ax.plot(t[:n_real], real_omega[:n_real], label=f"VESC ω (rs_core_speed/R_w, R_w={R_w})", linewidth=1)
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Angular velocity (rad/s)")
        ax.set_title("Wheel Angular Velocity — VESC vs Sim")
        ax.legend()
        ax.grid(True, alpha=0.3)
    else:
        ax.axis("off")

    fig.tight_layout()
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close(fig)

    print(f"\nPlot saved to {out_path}")


if __name__ == "__main__":
    main()
