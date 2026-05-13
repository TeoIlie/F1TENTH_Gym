"""
3D Critic Value Visualization for PPO Policy

Creates a 3D visualization of the PPO critic (value function) over the Drift map:
- X, Y axes: Position coordinates on the track (meters)
- Z axis: Critic value V(s) - expected return from that position
- Expected pattern: High values near raceline, decreasing outward, very low values off-track

Usage:
    python examples/analysis/critic_value_3d_plot.py --model-path /path/to/model.zip
"""

import argparse
from pathlib import Path

import gymnasium as gym
import matplotlib.pyplot as plt
import numpy as np
import torch
from stable_baselines3 import PPO

from gymkhana.envs.observation import sample_lookahead_curvatures_fast, sample_lookahead_widths_fast
from gymkhana.envs.utils import normalize_feature
from train.config.env_config import get_drift_test_config, get_env_id

# Constants
X_MIN, X_MAX = -1.0, 6.0  # Track X bounds with margin
Y_MIN, Y_MAX = -3.0, 6.5  # Track Y bounds with margin
DEFAULT_GRID_SPACING = 0.1  # meters
DEFAULT_TARGET_VELOCITY = 4.0  # m/s
WHEEL_RADIUS = 0.049  # meters (from F1TENTH params)


def parse_args():
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(description="Visualize PPO critic value function in 3D")
    parser.add_argument("--model-path", type=str, required=True, help="Path to trained PPO model (.zip file)")
    parser.add_argument(
        "--grid-spacing", type=float, default=DEFAULT_GRID_SPACING, help="Grid spacing in meters (default: 0.1)"
    )
    parser.add_argument(
        "--target-velocity",
        type=float,
        default=DEFAULT_TARGET_VELOCITY,
        help="Target velocity in m/s (default: 4.0)",
    )
    parser.add_argument(
        "--output",
        type=str,
        default="figures/analysis/critic_value_3d_plot.png",
        help="Output file path (default: figures/analysis/critic_value_3d_plot.png)",
    )
    return parser.parse_args()


def create_grid(spacing):
    """Create meshgrid covering the Drift track area."""
    xs = np.arange(X_MIN, X_MAX, spacing)
    ys = np.arange(Y_MIN, Y_MAX, spacing)
    X_grid, Y_grid = np.meshgrid(xs, ys)
    print(f"Grid size: {X_grid.shape}, Total points: {X_grid.size}")
    return X_grid, Y_grid


def construct_observation(x, y, track, norm_bounds, target_velocity, lookahead_n_points=5, lookahead_ds=0.5):
    """
    Construct a normalized observation vector for a given (x, y) position.

    Args:
        x, y: Position coordinates
        track: Track object
        norm_bounds: Normalization bounds dict
        target_velocity: Target velocity in m/s
        lookahead_n_points: Number of lookahead points (default: 5 from training config)
        lookahead_ds: Spacing between lookahead points in meters (default: 0.5 from training config)

    Returns:
        obs_array: Normalized observation vector (21 dims for drift type)
    """
    # Step 1: Get track-aligned heading
    # First pass: estimate phi from track (use phi=0.0 as initial guess)
    s_est, _, _ = track.cartesian_to_frenet(x, y, 0.0, precise=False)
    phi_aligned = track.centerline.spline.calc_yaw(s_est)

    # Step 2: Get Frenet coordinates with aligned heading
    s, ey, ephi = track.cartesian_to_frenet(x, y, phi_aligned, precise=False)

    # Step 3: Sample lookahead features at arc length s
    lookahead_curvs = sample_lookahead_curvatures_fast(track, s, n_points=lookahead_n_points, ds=lookahead_ds)
    lookahead_widths = sample_lookahead_widths_fast(track, s, n_points=lookahead_n_points, ds=lookahead_ds)

    # Step 4: Construct raw observation dict with steady-state racing values
    raw_obs = {
        "linear_vel_x": target_velocity,
        "linear_vel_y": 0.0,
        "frenet_u": ephi,
        "frenet_n": ey,
        "ang_vel_z": 0.0,
        "delta": 0.0,
        "beta": 0.0,
        "prev_steering_cmd": 0.0,
        "prev_accl_cmd": 0.0,
        "prev_avg_wheel_omega": target_velocity / WHEEL_RADIUS,
        "integrated_vel_cmd": target_velocity,
        "lookahead_curvatures": lookahead_curvs,
        "lookahead_widths": lookahead_widths,
    }

    # Step 5: Normalize features and flatten to vector
    vec_obs = []
    feature_names = [
        "linear_vel_x",
        "linear_vel_y",
        "frenet_u",
        "frenet_n",
        "ang_vel_z",
        "delta",
        "beta",
        "prev_steering_cmd",
        "prev_accl_cmd",
        "prev_avg_wheel_omega",
        "integrated_vel_cmd",
        "lookahead_curvatures",
        "lookahead_widths",
    ]

    for feat_name in feature_names:
        val = raw_obs[feat_name]
        val_norm = normalize_feature(feat_name, val, norm_bounds)

        if isinstance(val_norm, np.ndarray):
            vec_obs.extend(val_norm)
        else:
            vec_obs.append(val_norm)

    obs_array = np.array(vec_obs, dtype=np.float32)
    assert obs_array.shape == (21,), f"Wrong obs shape: {obs_array.shape}"

    return obs_array


def construct_grid_observations(X_grid, Y_grid, track, norm_bounds, target_velocity):
    """
    Construct observations for all grid points.

    Returns:
        observations: Array of shape (n_points, 21)
    """
    observations = []
    n_points = X_grid.size

    print("Constructing observations for grid points...")
    for i in range(n_points):
        if i % 500 == 0:
            print(f"  Progress: {i}/{n_points} ({100 * i / n_points:.1f}%)")

        x = X_grid.flat[i]
        y = Y_grid.flat[i]

        obs = construct_observation(x, y, track, norm_bounds, target_velocity)
        observations.append(obs)

    observations = np.stack(observations)
    print(f"Observations constructed: {observations.shape}")
    return observations


def query_critic_batch(model, observations):
    """
    Query critic network for all observations in batch.

    Args:
        model: PPO model
        observations: Array of shape (n_points, 21)

    Returns:
        values: Array of shape (n_points,)
    """
    print("Querying critic network...")
    with torch.no_grad():
        obs_tensor = torch.FloatTensor(observations)
        values = model.policy.predict_values(obs_tensor).cpu().numpy().flatten()

    print(f"Value range: [{values.min():.2f}, {values.max():.2f}]")
    return values


def extract_centerline_values(track, model, norm_bounds, target_velocity):
    """
    Extract critic values along the track centerline.

    Returns:
        centerline_xs, centerline_ys, centerline_ss, centerline_values
    """
    print("Extracting centerline values...")
    centerline_xs = track.centerline.xs
    centerline_ys = track.centerline.ys
    centerline_ss = track.centerline.ss

    centerline_observations = []
    for x, y in zip(centerline_xs, centerline_ys):
        obs = construct_observation(x, y, track, norm_bounds, target_velocity)
        centerline_observations.append(obs)

    centerline_observations = np.stack(centerline_observations)

    # Query critic
    with torch.no_grad():
        obs_tensor = torch.FloatTensor(centerline_observations)
        centerline_values = model.policy.predict_values(obs_tensor).cpu().numpy().flatten()

    print(f"Centerline value range: [{centerline_values.min():.2f}, {centerline_values.max():.2f}]")
    return centerline_xs, centerline_ys, centerline_ss, centerline_values


def create_visualization(
    X_grid, Y_grid, V_grid, track, centerline_xs, centerline_ys, centerline_ss, centerline_values, output_path
):
    """
    Create 3-panel visualization and save to file.
    """
    print("Creating visualization...")

    # Create output directory if needed
    Path(output_path).parent.mkdir(parents=True, exist_ok=True)

    # Create figure with 3 panels
    fig = plt.figure(figsize=(20, 6))

    # Panel 1: 3D Surface Plot
    ax1 = fig.add_subplot(131, projection="3d")
    surf = ax1.plot_surface(
        X_grid, Y_grid, V_grid, cmap="viridis", alpha=0.8, edgecolor="none", vmin=V_grid.min(), vmax=V_grid.max()
    )
    ax1.plot(centerline_xs, centerline_ys, centerline_values, "r-", linewidth=3, label="Centerline")
    ax1.set_xlabel("X [m]")
    ax1.set_ylabel("Y [m]")
    ax1.set_zlabel("Critic Value")
    ax1.set_title("3D Critic Value Surface (Constant Velocity Assumption)")
    ax1.view_init(elev=30, azim=45)
    ax1.legend()
    fig.colorbar(surf, ax=ax1, shrink=0.5)

    # Panel 2: 2D Heatmap (Top-Down View)
    ax2 = fig.add_subplot(132)
    im = ax2.contourf(X_grid, Y_grid, V_grid, levels=30, cmap="viridis")
    ax2.plot(centerline_xs, centerline_ys, "r-", linewidth=2, label="Centerline")

    # Add track boundaries
    centerline_yaws = track.centerline.yaws
    w_lefts = track.centerline.w_lefts
    w_rights = track.centerline.w_rights

    left_bound_xs = centerline_xs - w_lefts * np.sin(centerline_yaws)
    left_bound_ys = centerline_ys + w_lefts * np.cos(centerline_yaws)
    right_bound_xs = centerline_xs + w_rights * np.sin(centerline_yaws)
    right_bound_ys = centerline_ys - w_rights * np.cos(centerline_yaws)

    ax2.plot(left_bound_xs, left_bound_ys, "k--", linewidth=1, alpha=0.5, label="Track bounds")
    ax2.plot(right_bound_xs, right_bound_ys, "k--", linewidth=1, alpha=0.5)

    ax2.set_xlabel("X [m]")
    ax2.set_ylabel("Y [m]")
    ax2.set_title("2D Critic Heatmap")
    ax2.set_aspect("equal")
    ax2.legend()
    fig.colorbar(im, ax=ax2)

    # Panel 3: Arc Length Profile
    ax3 = fig.add_subplot(133)
    ax3.plot(centerline_ss, centerline_values, "b-", linewidth=2)
    ax3.set_xlabel("Arc length s [m]")
    ax3.set_ylabel("Critic Value")
    ax3.set_title("Critic Value Along Centerline")
    ax3.grid(True, alpha=0.3)
    ax3.axhline(y=0, color="k", linestyle="--", linewidth=0.8, alpha=0.5)

    plt.tight_layout()
    plt.savefig(output_path, dpi=300, bbox_inches="tight")
    print(f"Visualization saved to: {output_path}")
    plt.show()


def main():
    """Main execution function."""
    args = parse_args()

    # Step 1: Load model
    print(f"Loading PPO model from: {args.model_path}")
    model = PPO.load(args.model_path, print_system_info=True, device="cpu")

    # Step 2: Create environment with exact training config
    print("Creating environment...")
    config = get_drift_test_config()
    config["render_mode"] = None  # No rendering needed
    env = gym.make(get_env_id(), config=config)
    track = env.unwrapped.track

    # Extract normalization bounds
    obs_handler = env.unwrapped.observation_type
    norm_bounds = obs_handler.bounds
    print(f"Normalization bounds loaded: {len(norm_bounds)} features")

    # Step 3: Create grid
    X_grid, Y_grid = create_grid(args.grid_spacing)

    # Step 4: Construct observations for all grid points
    observations = construct_grid_observations(X_grid, Y_grid, track, norm_bounds, args.target_velocity)

    # Step 5: Query critic in batch
    values = query_critic_batch(model, observations)
    V_grid = values.reshape(X_grid.shape)

    # Step 6: Extract centerline values
    centerline_xs, centerline_ys, centerline_ss, centerline_values = extract_centerline_values(
        track, model, norm_bounds, args.target_velocity
    )

    # Step 7: Create visualization
    create_visualization(
        X_grid, Y_grid, V_grid, track, centerline_xs, centerline_ys, centerline_ss, centerline_values, args.output
    )

    # Optional: Save grid data for later replotting
    data_output_path = Path(args.output).with_suffix(".npz")
    np.savez(
        data_output_path,
        X_grid=X_grid,
        Y_grid=Y_grid,
        V_grid=V_grid,
        centerline_xs=centerline_xs,
        centerline_ys=centerline_ys,
        centerline_ss=centerline_ss,
        centerline_values=centerline_values,
    )
    print(f"Grid data saved to: {data_output_path}")

    env.close()
    print("Done!")


if __name__ == "__main__":
    main()
