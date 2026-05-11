"""
Phase Plane Vector Field Analysis for Trained PPO Drift Policy

This script loads a trained PPO model and analyzes its behavior by generating
phase plane vector field plots showing how the system state evolves:
1. Heading error (frenet_u) phase plane with velocity vectors
2. Lateral deviation (frenet_n) phase plane with velocity vectors

Arrows show the direction and rate of state evolution:
- Arrow position: Current state (x, dx/dt)
- Arrow direction: Rate of change (dx/dt, d²x/dt²)
- Arrow length: Normalized for clarity
- Arrow color: Velocity magnitude (speed of evolution)

Usage:
    1. Edit MODEL_PATH constant to point to your trained model
    2. Run: python examples/analysis/phase_plane_analysis.py
    3. View generated plots in frenet_u_n_phase_plane.png
"""

import os

import gymnasium as gym
import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import savgol_filter
from stable_baselines3 import PPO

from train.config.env_config import get_drift_train_config, get_env_id
from train.train_utils import get_output_dirs, print_header

# Configuration constants
PROJ_ROOT = get_output_dirs()[0]
NUM_EPISODES = 20  # Number of episodes to collect data
MODEL_PATH = PROJ_ROOT + "/outputs/downloads/ol035sw5/model.zip"  # Edit this to point to your trained model
DT = 0.01  # Timestep from config (0.01 seconds = 100 Hz)
OUTPUT_DIR = PROJ_ROOT + "/figures/analysis"
os.makedirs(OUTPUT_DIR, exist_ok=True)
OUTPUT_FILENAME = OUTPUT_DIR + "/frenet_u_n_phase_plane.png"
SUBSAMPLE_RATE = 50  # Subsample before plotting to reduce clutter
COLOR_PERCENTILE_CLIP = (10, 90)  # Clip outliers for clearer color distribution

# Savitzky-Golay smoothing configuration
APPLY_SMOOTHING = True  # Enable/disable smoothing
SMOOTH_WINDOW_LENGTH = 5  # Must be odd (larger = smoother)
SMOOTH_POLYORDER = 3  # Polynomial order (typically 2-4)


def compute_angle_derivative(angles, dt):
    """
    Compute derivative of angle sequence, handling ±π wrapping.

    Args:
        angles: np.ndarray of angles in radians
        dt: timestep in seconds

    Returns:
        np.ndarray of angular derivatives in rad/s (length = len(angles) - 1)
    """
    derivatives = []
    for i in range(len(angles) - 1):
        # Compute shortest angular difference
        diff = angles[i + 1] - angles[i]
        # Wrap to [-π, π]
        diff = np.arctan2(np.sin(diff), np.cos(diff))
        derivatives.append(diff / dt)
    return np.array(derivatives)


def collect_phase_plane_data(model, env, num_episodes, dt):
    """
    Run trained model for num_episodes and collect frenet_u, frenet_n time series.
    Computes first and second derivatives for vector field visualization.

    Args:
        model: Trained PPO model
        env: Gymnasium environment
        num_episodes: Number of episodes to run
        dt: Timestep for derivative computation

    Returns:
        dict with keys: 'frenet_u', 'frenet_n', 'd_frenet_u', 'd_frenet_n',
                        'd2_frenet_u', 'd2_frenet_n'
    """
    frenet_u_all = []
    frenet_n_all = []
    d_frenet_u_all = []
    d_frenet_n_all = []
    d2_frenet_u_all = []
    d2_frenet_n_all = []

    for episode in range(num_episodes):
        obs, info = env.reset()
        done, trunc = False, False

        # Trajectory storage for this episode
        frenet_u_traj = []
        frenet_n_traj = []

        while not (done or trunc):
            # Get action from trained model
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, done, trunc, info = env.step(action)

            # Extract raw Frenet coordinates from environment
            track = env.unwrapped.track
            agent = env.unwrapped.sim.agents[0]
            std_state = agent.standard_state

            x = std_state["x"]
            y = std_state["y"]
            theta = std_state["yaw"]

            s, ey, ephi = track.cartesian_to_frenet(x, y, theta, use_raceline=False)

            frenet_u_traj.append(ephi)  # heading error
            frenet_n_traj.append(ey)  # lateral deviation

        # Convert to numpy arrays
        frenet_u_traj = np.array(frenet_u_traj)
        frenet_n_traj = np.array(frenet_n_traj)

        # Compute derivatives (with optional smoothing)
        if APPLY_SMOOTHING and len(frenet_u_traj) >= SMOOTH_WINDOW_LENGTH:
            # Use Savitzky-Golay filter with analytical derivatives

            # For lateral deviation: Direct savgol_filter (no angle wrapping issues)
            d_frenet_n = savgol_filter(frenet_n_traj, SMOOTH_WINDOW_LENGTH, SMOOTH_POLYORDER, deriv=1, delta=dt)
            d2_frenet_n = savgol_filter(frenet_n_traj, SMOOTH_WINDOW_LENGTH, SMOOTH_POLYORDER, deriv=2, delta=dt)

            # For heading error: Unwrap angles first to handle ±π discontinuities
            frenet_u_unwrapped = np.unwrap(frenet_u_traj)
            d_frenet_u = savgol_filter(frenet_u_unwrapped, SMOOTH_WINDOW_LENGTH, SMOOTH_POLYORDER, deriv=1, delta=dt)
            d2_frenet_u = savgol_filter(frenet_u_unwrapped, SMOOTH_WINDOW_LENGTH, SMOOTH_POLYORDER, deriv=2, delta=dt)
            # Note: Derivatives are continuous after unwrapping, no need to re-wrap

            # Align array lengths: savgol_filter maintains length, trim last 2 for consistency
            frenet_u_all.extend(frenet_u_traj[:-2])
            frenet_n_all.extend(frenet_n_traj[:-2])
            d_frenet_u_all.extend(d_frenet_u[:-2])
            d_frenet_n_all.extend(d_frenet_n[:-2])
            d2_frenet_u_all.extend(d2_frenet_u[:-2])
            d2_frenet_n_all.extend(d2_frenet_n[:-2])
        else:
            # Original finite difference approach (without smoothing)
            d_frenet_u = compute_angle_derivative(frenet_u_traj, dt)
            d_frenet_n = np.diff(frenet_n_traj) / dt
            d2_frenet_u = compute_angle_derivative(d_frenet_u, dt)
            d2_frenet_n = np.diff(d_frenet_n) / dt

            # Align array lengths (N -> N-1 -> N-2)
            frenet_u_all.extend(frenet_u_traj[:-2])
            frenet_n_all.extend(frenet_n_traj[:-2])
            d_frenet_u_all.extend(d_frenet_u[:-1])
            d_frenet_n_all.extend(d_frenet_n[:-1])
            d2_frenet_u_all.extend(d2_frenet_u)
            d2_frenet_n_all.extend(d2_frenet_n)

        print(f"Episode {episode + 1}/{num_episodes} complete: {len(frenet_u_traj)} steps")

    return {
        "frenet_u": np.array(frenet_u_all),
        "frenet_n": np.array(frenet_n_all),
        "d_frenet_u": np.array(d_frenet_u_all),
        "d_frenet_n": np.array(d_frenet_n_all),
        "d2_frenet_u": np.array(d2_frenet_u_all),
        "d2_frenet_n": np.array(d2_frenet_n_all),
    }


def plot_phase_planes(data, output_filename, subsample_rate=50):
    """
    Create two-panel phase plane vector field plots using quiver.

    Args:
        data: dict with 'frenet_u', 'frenet_n', 'd_frenet_u', 'd_frenet_n',
                       'd2_frenet_u', 'd2_frenet_n'
        output_filename: path to save figure
        subsample_rate: plot every Nth point (default 50)
    """
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 7))

    # Subsample data to avoid overcrowding
    indices = np.arange(0, len(data["frenet_u"]), subsample_rate)

    # ========== PLOT 1: Heading Error Vector Field ==========
    # Extract subsampled data
    u_pos = data["frenet_u"][indices]  # x position
    u_vel = data["d_frenet_u"][indices]  # y position
    u_dx = data["d_frenet_u"][indices]  # u component (horizontal velocity)
    u_dy = data["d2_frenet_u"][indices]  # v component (vertical velocity)

    # Compute vector magnitude for color mapping
    u_magnitude = np.sqrt(u_dx**2 + u_dy**2)

    # Compute percentile-based color limits and clip outliers
    vmin_u = np.percentile(u_magnitude, COLOR_PERCENTILE_CLIP[0])
    vmax_u = np.percentile(u_magnitude, COLOR_PERCENTILE_CLIP[1])
    n_outliers_u = np.sum((u_magnitude < vmin_u) | (u_magnitude > vmax_u))
    u_magnitude_clipped = np.clip(u_magnitude, vmin_u, vmax_u)

    # Normalize arrow lengths (all arrows same length, color shows magnitude)
    u_norm = np.sqrt(u_dx**2 + u_dy**2) + 1e-8  # Avoid division by zero
    u_dx_norm = u_dx / u_norm
    u_dy_norm = u_dy / u_norm

    # Create quiver plot
    quiver1 = ax1.quiver(
        u_pos,
        u_vel,  # Arrow positions (x, y)
        u_dx_norm,
        u_dy_norm,  # Arrow directions (normalized)
        u_magnitude_clipped,  # Color by clipped magnitude
        cmap="viridis",  # Color map
        scale=25,  # Arrow scaling (smaller = longer arrows)
        width=0.003,  # Arrow shaft width
        alpha=0.8,  # Transparency
        headwidth=3,  # Arrow head width
        headlength=4,  # Arrow head length
    )

    # Add colorbar for magnitude
    plt.colorbar(quiver1, ax=ax1, label="Velocity magnitude")

    # Labels and formatting
    ax1.set_xlabel("frenet_u (heading error) [rad]", fontsize=12)
    ax1.set_ylabel("d(frenet_u)/dt [rad/s]", fontsize=12)
    ax1.set_title("Phase Plane Vector Field: Heading Error", fontsize=14, fontweight="bold")
    ax1.grid(True, alpha=0.3)
    ax1.axhline(y=0, color="k", linestyle="--", linewidth=0.8, alpha=0.5)
    ax1.axvline(x=0, color="k", linestyle="--", linewidth=0.8, alpha=0.5)

    # ========== PLOT 2: Lateral Deviation Vector Field ==========
    # Extract subsampled data
    n_pos = data["frenet_n"][indices]  # x position
    n_vel = data["d_frenet_n"][indices]  # y position
    n_dx = data["d_frenet_n"][indices]  # u component (horizontal velocity)
    n_dy = data["d2_frenet_n"][indices]  # v component (vertical velocity)

    # Compute vector magnitude for color mapping
    n_magnitude = np.sqrt(n_dx**2 + n_dy**2)

    # Compute percentile-based color limits and clip outliers
    vmin_n = np.percentile(n_magnitude, COLOR_PERCENTILE_CLIP[0])
    vmax_n = np.percentile(n_magnitude, COLOR_PERCENTILE_CLIP[1])
    n_outliers_n = np.sum((n_magnitude < vmin_n) | (n_magnitude > vmax_n))
    n_magnitude_clipped = np.clip(n_magnitude, vmin_n, vmax_n)

    # Normalize arrow lengths
    n_norm = np.sqrt(n_dx**2 + n_dy**2) + 1e-8
    n_dx_norm = n_dx / n_norm
    n_dy_norm = n_dy / n_norm

    # Create quiver plot
    quiver2 = ax2.quiver(
        n_pos,
        n_vel,  # Arrow positions
        n_dx_norm,
        n_dy_norm,  # Arrow directions (normalized)
        n_magnitude_clipped,  # Color by clipped magnitude
        cmap="plasma",  # Different color map for distinction
        scale=25,
        width=0.003,
        alpha=0.8,
        headwidth=3,
        headlength=4,
    )

    # Add colorbar
    plt.colorbar(quiver2, ax=ax2, label="Velocity magnitude")

    # Labels and formatting
    ax2.set_xlabel("frenet_n (lateral deviation) [m]", fontsize=12)
    ax2.set_ylabel("d(frenet_n)/dt [m/s]", fontsize=12)
    ax2.set_title("Phase Plane Vector Field: Lateral Deviation", fontsize=14, fontweight="bold")
    ax2.grid(True, alpha=0.3)
    ax2.axhline(y=0, color="k", linestyle="--", linewidth=0.8, alpha=0.5)
    ax2.axvline(x=0, color="k", linestyle="--", linewidth=0.8, alpha=0.5)

    plt.tight_layout()
    plt.savefig(output_filename, dpi=300, bbox_inches="tight")
    print(f"\nPhase plane vector field plots saved to: {output_filename}")
    print(f"  Plotted {len(indices)} arrows (subsampled every {subsample_rate} points)")
    print(f"  Color mapping (percentile {COLOR_PERCENTILE_CLIP[0]}-{COLOR_PERCENTILE_CLIP[1]}%):")
    print(f"    Heading error: Clipped {n_outliers_u}/{len(u_magnitude)} outliers (range: {vmin_u:.3f}-{vmax_u:.3f})")
    print(
        f"    Lateral deviation: Clipped {n_outliers_n}/{len(n_magnitude)} outliers (range: {vmin_n:.3f}-{vmax_n:.3f})"
    )
    plt.show()


def main():
    print_header("Phase Plane Analysis for Trained PPO Drift Policy")

    # 1. Load trained model
    print(f"\nLoading model from: {MODEL_PATH}")
    model = PPO.load(MODEL_PATH, print_system_info=True, device="cpu")

    # 2. Create evaluation environment (no rendering for faster data collection)
    print("\nCreating evaluation environment...")
    config = get_drift_train_config()
    config["domain_randomization"] = None  # phase plane wants deterministic physics
    env = gym.make(
        get_env_id(),
        config=config,
        render_mode=None,  # No rendering for faster collection
    )

    # 3. Collect phase plane data
    print(f"\nCollecting data from {NUM_EPISODES} episodes...")
    data = collect_phase_plane_data(model, env, NUM_EPISODES, DT)

    print(f"\nTotal data points collected: {len(data['frenet_u'])}")
    print(f"  frenet_u range: [{np.min(data['frenet_u']):.3f}, {np.max(data['frenet_u']):.3f}] rad")
    print(f"  frenet_n range: [{np.min(data['frenet_n']):.3f}, {np.max(data['frenet_n']):.3f}] m")
    print(f"  d(frenet_u)/dt range: [{np.min(data['d_frenet_u']):.3f}, {np.max(data['d_frenet_u']):.3f}] rad/s")
    print(f"  d(frenet_n)/dt range: [{np.min(data['d_frenet_n']):.3f}, {np.max(data['d_frenet_n']):.3f}] m/s")
    print(f"  d²(frenet_u)/dt² range: [{np.min(data['d2_frenet_u']):.3f}, {np.max(data['d2_frenet_u']):.3f}] rad/s²")
    print(f"  d²(frenet_n)/dt² range: [{np.min(data['d2_frenet_n']):.3f}, {np.max(data['d2_frenet_n']):.3f}] m/s²")

    # 4. Generate plots
    print("\nGenerating phase plane vector field plots...")
    plot_phase_planes(data, OUTPUT_FILENAME, subsample_rate=SUBSAMPLE_RATE)

    # 5. Cleanup
    env.close()
    print("\nAnalysis complete!")
    print("=" * 60)


if __name__ == "__main__":
    main()
