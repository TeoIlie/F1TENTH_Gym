"""
Observation Sensitivity Analysis for PPO Policy

Quantifies the sensitivity of the learned policy to each observation feature by computing
the average gradient of policy actions with respect to input features over real rollout data.

Produces a bar chart where:
- X-axis: observation feature indices/labels
- Y-axis: mean gradient magnitude (averaged across timesteps and action dimensions)
- Yellow bars: vehicle state features (vx, vy, yaw rate, steering, slip, commands)
- Blue bars: track features (heading error, lateral error, curvatures, widths)

Usage:
    python examples/analysis/obs_sensitivity.py --run_id 178a1a5l
    python examples/analysis/obs_sensitivity.py --run_id 178a1a5l --path /path/to/model.zip --n-episodes 10
"""

import argparse
from pathlib import Path

import gymnasium as gym
import matplotlib.pyplot as plt
import numpy as np
import torch
from stable_baselines3 import PPO

from train.config.env_config import get_drift_test_config, get_env_id

# Feature labels matching the "drift_real" observation type order (observation.py:911-922)
# With 5 lookahead points and sparse widths (first & last) = 14 total
FEATURE_LABELS = [
    "vx",
    "vy",
    "u (heading)",
    "n (lateral)",
    "r (yaw rate)",
    "beta",
    "curr_omega",
    "curv_0",
    "curv_1",
    "curv_2",
    "curv_3",
    "curv_4",
    "width_first",
    "width_last",
]

# Category for each feature: True = vehicle/control (yellow), False = track (blue)
IS_VEHICLE = [
    True,  # vx
    True,  # vy
    False,  # u (heading error - track-relative)
    False,  # n (lateral error - track-relative)
    True,  # r (yaw rate)
    True,  # beta
    True,  # curr_omega
    False,  # curv_0
    False,  # curv_1
    False,  # curv_2
    False,  # curv_3
    False,  # curv_4
    False,  # width_first
    False,  # width_last
]


def parse_args():
    parser = argparse.ArgumentParser(description="Observation sensitivity analysis for PPO policy")
    parser.add_argument(
        "--run_id", type=str, required=True, help="Wandb run ID (used for default model path and output dir)"
    )
    parser.add_argument(
        "--path",
        type=str,
        default=None,
        help="Path to trained PPO model (.zip file). Default: outputs/downloads/<run_id>/model.zip",
    )
    parser.add_argument("--n-episodes", type=int, default=5, help="Number of rollout episodes (default: 5)")
    parser.add_argument("--output", type=str, default=None, help="Output file path (default: auto-generated)")
    return parser.parse_args()


def collect_observations(env, model, n_episodes):
    """Run rollout episodes and collect observations."""
    all_obs = []
    for ep in range(n_episodes):
        obs, _ = env.reset()
        done = False
        steps = 0
        while not done:
            all_obs.append(obs.copy())
            action, _ = model.predict(obs, deterministic=True)
            obs, _, terminated, truncated, _ = env.step(action)
            done = terminated or truncated
            steps += 1
        print(f"  Episode {ep + 1}/{n_episodes}: {steps} steps")

    observations = np.stack(all_obs)
    print(f"Collected {len(all_obs)} observations across {n_episodes} episodes")
    return observations


def compute_gradients(model, observations):
    """
    Compute d(action)/d(obs) for each observation via PyTorch autograd.

    Returns:
        grad_matrix: array of shape (n_obs, n_actions, n_features) with raw gradients
    """
    policy = model.policy
    policy.set_training_mode(False)

    n_obs = observations.shape[0]
    n_features = observations.shape[1]

    # Determine action dimension from a forward pass
    with torch.no_grad():
        sample = torch.FloatTensor(observations[:1])
        features = policy.extract_features(sample, policy.pi_features_extractor)
        latent_pi = policy.mlp_extractor.forward_actor(features)
        n_actions = policy.action_net(latent_pi).shape[-1]

    print(f"Computing gradients: {n_obs} observations, {n_features} features, {n_actions} actions...")

    grad_matrix = np.zeros((n_obs, n_actions, n_features), dtype=np.float32)

    for i in range(n_obs):
        if i % 500 == 0:
            print(f"  Progress: {i}/{n_obs} ({100 * i / n_obs:.1f}%)")

        obs_tensor = torch.FloatTensor(observations[i]).unsqueeze(0).requires_grad_(True)

        # Forward through policy: extract features -> MLP -> action_net
        features = policy.extract_features(obs_tensor, policy.pi_features_extractor)
        latent_pi = policy.mlp_extractor.forward_actor(features)
        action_output = policy.action_net(latent_pi)

        for a_idx in range(n_actions):
            policy.zero_grad()
            if obs_tensor.grad is not None:
                obs_tensor.grad.zero_()

            action_output[0, a_idx].backward(retain_graph=True)
            assert obs_tensor.grad is not None, "Gradient not computed — check that obs_tensor requires grad"
            grad_matrix[i, a_idx] = obs_tensor.grad[0].detach().numpy()

    print("Gradient computation complete.")
    return grad_matrix


def create_plot(mean_abs_grads, std_abs_grads, feature_labels, is_vehicle, output_path, model_name):
    """Create and save the sensitivity bar chart."""
    Path(output_path).parent.mkdir(parents=True, exist_ok=True)

    n_features = len(feature_labels)
    x = np.arange(n_features)
    colors = ["#DAA520" if v else "#4682B4" for v in is_vehicle]

    _, ax = plt.subplots(figsize=(14, 6))

    ax.bar(x, mean_abs_grads, yerr=std_abs_grads, color=colors, edgecolor="black", linewidth=0.5, capsize=3)

    ax.set_xlabel("Observation Feature", fontsize=12)
    ax.set_ylabel("Mean |d(action)/d(obs)|", fontsize=12)
    ax.set_title(f"Observation Sensitivity — {model_name}", fontsize=13)
    ax.set_xticks(x)
    ax.set_xticklabels(feature_labels, rotation=45, ha="right", fontsize=10)
    ax.grid(axis="y", alpha=0.3)

    # Legend
    from matplotlib.patches import Patch

    legend_elements = [
        Patch(facecolor="#DAA520", edgecolor="black", label="Vehicle State"),
        Patch(facecolor="#4682B4", edgecolor="black", label="Track"),
    ]
    ax.legend(handles=legend_elements, loc="upper right", fontsize=11)

    plt.tight_layout()
    plt.savefig(output_path, dpi=300, bbox_inches="tight")
    print(f"Plot saved to: {output_path}")
    plt.show()


def main():
    args = parse_args()
    run_id = args.run_id
    model_path = args.path if args.path else f"outputs/downloads/{run_id}/model.zip"

    # Load model
    print(f"Loading PPO model from: {model_path}")
    model = PPO.load(model_path, device="cpu")

    # Create environment
    config = get_drift_test_config()
    config["render_mode"] = None
    env = gym.make(get_env_id(), config=config)

    obs_dim = env.observation_space.shape[0]
    print(f"Observation dimension: {obs_dim}")
    assert obs_dim == len(FEATURE_LABELS), (
        f"Observation dimension {obs_dim} does not match expected {len(FEATURE_LABELS)} features. "
        f"Check lookahead_n_points and sparse_width_obs config."
    )

    # Collect observations from rollouts
    print(f"\nCollecting observations from {args.n_episodes} episodes...")
    observations = collect_observations(env, model, args.n_episodes)

    # Compute gradients
    print("\nComputing gradients...")
    grad_matrix = compute_gradients(model, observations)

    # Aggregate: mean absolute gradient per feature, averaged across actions and timesteps
    abs_grads = np.abs(grad_matrix)  # (n_obs, n_actions, n_features)
    per_obs_mean = abs_grads.mean(axis=1)  # (n_obs, n_features) — average across actions
    mean_abs_grads = per_obs_mean.mean(axis=0)  # (n_features,) — average across timesteps
    std_abs_grads = per_obs_mean.std(axis=0)  # (n_features,) — std across timesteps

    # Print ranked features
    print("\nFeature sensitivity ranking:")
    ranked = np.argsort(mean_abs_grads)[::-1]
    for rank, idx in enumerate(ranked):
        print(f"  {rank + 1:2d}. {FEATURE_LABELS[idx]:15s}  {mean_abs_grads[idx]:.4f} ± {std_abs_grads[idx]:.4f}")

    # Plot
    if args.output is None:
        output_path = f"figures/analysis/obs_sensitivity/{run_id}_sensitivity.png"
    else:
        output_path = args.output

    create_plot(mean_abs_grads, std_abs_grads, FEATURE_LABELS, IS_VEHICLE, output_path, run_id)

    env.close()
    print("Done!")


if __name__ == "__main__":
    main()
