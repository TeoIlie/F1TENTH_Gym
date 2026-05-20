"""
Morris Method Parameter Sensitivity Analysis for STD Dynamics Model

Quantifies how sensitive the trained policy's performance is to each dynamics model
parameter using the Morris screening method (Elementary Effects). This identifies which
parameters are most important to calibrate accurately for sim2real transfer.

Produces:
- Bar chart: parameters ranked by mu_star (mean absolute elementary effect).
  Taller bar = bigger impact on episodic return. This is the calibration priority list.
- Scatter plot: mu_star (importance) vs sigma (interaction/nonlinearity).
  Below the diagonal: parameter has a consistent, independent effect — easy to calibrate in isolation.
  Above the diagonal: parameter interacts with others or is nonlinear — needs joint calibration.
  Near the origin: negligible — skip calibration.
- Console table: ranked parameters with mu_star and sigma values

Requires: SALib (`pip install SALib`)

Usage:
    python examples/analysis/morris_param_sensitivity.py --run_id 178a1a5l
    python examples/analysis/morris_param_sensitivity.py --run_id 178a1a5l --n-trajectories 15 --n-episodes 5
    python examples/analysis/morris_param_sensitivity.py --run_id 178a1a5l --perturbation 0.3
    python examples/analysis/morris_param_sensitivity.py --path /path/to/model.zip
    python examples/analysis/morris_param_sensitivity.py --path /path/to/model.zip --n-trajectories 15
"""

import argparse
from pathlib import Path

import gymnasium as gym
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Patch
from stable_baselines3 import PPO

try:
    from SALib.analyze import morris as morris_analyze
    from SALib.sample import morris as morris_sample
except ImportError:
    raise ImportError("SALib is required for Morris sensitivity analysis. Install with: pip install SALib")

from train.config.env_config import PARAMS, get_drift_test_config, get_env_id

# Category mapping for parameters that may appear in a vehicle param set.
# Filtered against the active PARAMS dict at module load so the analysis adapts
# to whichever param set ``env_config.PARAMS`` is using (ST, STD, etc.).
# Excludes known-exact limits/geometry (s_max, v_max, width, ...).
# fmt: off
_PARAM_CATEGORIES = {
    # Vehicle dynamics
    "mu":       "vehicle",
    "C_Sf":     "vehicle",
    "C_Sr":     "vehicle",
    "lf":       "vehicle",
    "lr":       "vehicle",
    "m":        "vehicle",
    "I_z":      "vehicle",
    "h_s":      "vehicle",
    # Tire longitudinal (Pacejka)
    "tire_p_cx1": "tire_long",
    "tire_p_dx1": "tire_long",
    "tire_p_ex1": "tire_long",
    "tire_p_kx1": "tire_long",
    "tire_p_hx1": "tire_long",
    "tire_p_vx1": "tire_long",
    "tire_r_bx1": "tire_long",
    # Tire lateral (Pacejka)
    "tire_p_cy1": "tire_lat",
    "tire_p_dy1": "tire_lat",
    "tire_p_ey1": "tire_lat",
    "tire_p_ky1": "tire_lat",
    "tire_p_hy1": "tire_lat",
    "tire_p_vy1": "tire_lat",
    "tire_r_by1": "tire_lat",
    # Wheel dynamics
    "R_w":      "wheel",
    "I_y_w":    "wheel",
    "T_sb":     "wheel",
    "T_se":     "wheel",
    "a_max":    "wheel",
}
# fmt: on

# Filter to keys actually present in the active param set.
PARAM_DEFINITIONS = {name: cat for name, cat in _PARAM_CATEGORIES.items() if name in PARAMS}

CATEGORY_COLORS = {
    "vehicle": "#DAA520",
    "tire_long": "#4682B4",
    "tire_lat": "#2E8B57",
    "wheel": "#8B4513",
}

CATEGORY_LABELS = {
    "vehicle": "Vehicle Dynamics",
    "tire_long": "Tire Longitudinal",
    "tire_lat": "Tire Lateral",
    "wheel": "Wheel / Drivetrain",
}


def parse_args():
    parser = argparse.ArgumentParser(description="Morris method parameter sensitivity analysis for STD dynamics model")
    parser.add_argument(
        "--run_id", type=str, default=None, help="Wandb run ID (used for default model path and output filenames)"
    )
    parser.add_argument(
        "--path",
        type=str,
        default=None,
        help="Path to trained PPO model (.zip file). Default: outputs/downloads/<run_id>/model.zip",
    )
    parser.add_argument("--n-episodes", type=int, default=3, help="Episodes per sample point (default: 3)")
    parser.add_argument("--n-trajectories", type=int, default=10, help="Morris trajectories r (default: 10)")
    parser.add_argument("--n-levels", type=int, default=4, help="Morris grid levels (default: 4)")
    parser.add_argument(
        "--perturbation",
        type=float,
        default=0.2,
        help="Fractional perturbation range, e.g. 0.2 = +-20%% (default: 0.2)",
    )
    parser.add_argument("--output", type=str, default=None, help="Output directory (default: auto-generated)")
    args = parser.parse_args()
    if args.run_id is None and args.path is None:
        parser.error("must provide either --run_id or --path")
    return args


def define_problem(nominal_params, perturbation):
    """Build SALib problem dict with parameter bounds based on perturbation fraction."""
    param_names = list(PARAM_DEFINITIONS.keys())
    bounds = []
    for name in param_names:
        nominal = nominal_params[name]
        if nominal == 0:
            # For zero-valued params, use a small absolute range
            bounds.append([-1e-6, 1e-6])
        elif nominal > 0:
            bounds.append([nominal * (1 - perturbation), nominal * (1 + perturbation)])
        else:
            # For negative params, flip the perturbation direction
            bounds.append([nominal * (1 + perturbation), nominal * (1 - perturbation)])

    # Clamp T_sb and T_se to [0, 1]
    for i, name in enumerate(param_names):
        if name in ("T_sb", "T_se"):
            bounds[i] = [max(0.0, bounds[i][0]), min(1.0, bounds[i][1])]

    problem = {
        "num_vars": len(param_names),
        "names": param_names,
        "bounds": bounds,
    }
    return problem


def evaluate_samples(env, model, sample_matrix, param_names, nominal_params, n_episodes):
    """
    Run rollouts for each parameter sample and return mean episodic returns.

    Args:
        env: gymnasium environment
        model: trained PPO model
        sample_matrix: (n_samples, n_params) array from Morris sampling
        param_names: list of parameter names matching sample columns
        nominal_params: full nominal parameter dict
        n_episodes: episodes to average per sample

    Returns:
        results: (n_samples,) array of mean episodic returns
    """
    n_samples = sample_matrix.shape[0]
    results = np.zeros(n_samples)

    print(f"Evaluating {n_samples} parameter samples, {n_episodes} episodes each ({n_samples * n_episodes} total)...")

    for i in range(n_samples):
        # Build modified params dict
        modified_params = nominal_params.copy()
        for j, name in enumerate(param_names):
            modified_params[name] = sample_matrix[i, j]

        env.unwrapped.update_params(modified_params)

        # Run episodes with fixed seeds for fair comparison
        episode_returns = []
        for ep in range(n_episodes):
            obs, _ = env.reset(seed=1000 + ep)
            done = False
            total_reward = 0.0
            while not done:
                action, _ = model.predict(obs, deterministic=True)
                obs, reward, terminated, truncated, _ = env.step(action)
                total_reward += reward
                done = terminated or truncated
            episode_returns.append(total_reward)

        results[i] = np.mean(episode_returns)

        if (i + 1) % 10 == 0 or i == 0:
            print(f"  Sample {i + 1}/{n_samples}: mean return = {results[i]:.2f}")

    print(f"Evaluation complete. Return range: [{results.min():.2f}, {results.max():.2f}]")
    return results


def create_bar_plot(si, param_names, output_path, run_id):
    """Create ranked bar chart of mu_star values."""
    mu_star = si["mu_star"]
    sigma = si["sigma"]
    categories = [PARAM_DEFINITIONS[name] for name in param_names]

    # Sort by mu_star descending
    ranked = np.argsort(mu_star)[::-1]

    fig, ax = plt.subplots(figsize=(14, 6))

    x = np.arange(len(param_names))
    colors = [CATEGORY_COLORS[categories[idx]] for idx in ranked]
    labels = [param_names[idx] for idx in ranked]
    values = mu_star[ranked]
    errors = sigma[ranked]

    ax.bar(x, values, yerr=errors, color=colors, edgecolor="black", linewidth=0.5, capsize=3)

    ax.set_xlabel("Dynamics Model Parameter", fontsize=12)
    ax.set_ylabel("$\\mu^*$ (Mean Absolute Elementary Effect)", fontsize=12)
    ax.set_title(f"Morris Parameter Sensitivity — {run_id}", fontsize=13)
    ax.set_xticks(x)
    ax.set_xticklabels(labels, rotation=45, ha="right", fontsize=9)
    ax.grid(axis="y", alpha=0.3)

    legend_elements = [
        Patch(facecolor=color, edgecolor="black", label=CATEGORY_LABELS[cat]) for cat, color in CATEGORY_COLORS.items()
    ]
    ax.legend(handles=legend_elements, loc="upper right", fontsize=10)

    plt.tight_layout()
    plt.savefig(output_path, dpi=300, bbox_inches="tight")
    print(f"Bar plot saved to: {output_path}")
    plt.close(fig)


def create_scatter_plot(si, param_names, output_path, run_id):
    """Create mu_star vs sigma scatter plot (standard Morris visualization)."""
    mu_star = si["mu_star"]
    sigma = si["sigma"]
    categories = [PARAM_DEFINITIONS[name] for name in param_names]

    fig, ax = plt.subplots(figsize=(10, 8))

    for i, name in enumerate(param_names):
        color = CATEGORY_COLORS[categories[i]]
        ax.scatter(mu_star[i], sigma[i], color=color, s=60, edgecolor="black", linewidth=0.5, zorder=3)
        ax.annotate(
            name, (mu_star[i], sigma[i]), fontsize=7, ha="left", va="bottom", xytext=(4, 4), textcoords="offset points"
        )

    # Reference line: sigma = mu_star (above = strong interactions/nonlinearity)
    max_val = max(mu_star.max(), sigma.max()) * 1.1
    ax.plot([0, max_val], [0, max_val], "k--", alpha=0.3, linewidth=1, label="$\\sigma = \\mu^*$")

    ax.set_xlabel("$\\mu^*$ (Importance)", fontsize=12)
    ax.set_ylabel("$\\sigma$ (Interaction / Nonlinearity)", fontsize=12)
    ax.set_title(f"Morris Screening — {run_id}", fontsize=13)
    ax.grid(alpha=0.3)

    legend_elements = [
        Patch(facecolor=color, edgecolor="black", label=CATEGORY_LABELS[cat]) for cat, color in CATEGORY_COLORS.items()
    ]
    ax.legend(handles=legend_elements, loc="upper left", fontsize=10)

    plt.tight_layout()
    plt.savefig(output_path, dpi=300, bbox_inches="tight")
    print(f"Scatter plot saved to: {output_path}")
    plt.close(fig)


def main():
    args = parse_args()
    model_path = args.path or f"outputs/downloads/{args.run_id}/model.zip"
    run_id = args.run_id or Path(args.path).stem  # used for plot titles + output filenames

    # Load model
    print(f"Loading PPO model from: {model_path}")
    model = PPO.load(model_path, device="cpu")

    # Create environment
    config = get_drift_test_config()
    config["render_mode"] = None
    env = gym.make(get_env_id(), config=config)

    # Get nominal params from the environment config
    nominal_params = config["params"]
    print(f"Nominal params loaded ({len(nominal_params)} total, {len(PARAM_DEFINITIONS)} analyzed)")

    # Define Morris problem
    problem = define_problem(nominal_params, args.perturbation)
    print(f"Perturbation range: +-{args.perturbation * 100:.0f}%")

    # Generate Morris samples
    sample_matrix = morris_sample.sample(
        problem,
        N=args.n_trajectories,
        num_levels=args.n_levels,
    )
    print(f"Generated {sample_matrix.shape[0]} samples ({args.n_trajectories} trajectories, {args.n_levels} levels)")

    # Evaluate all samples
    results = evaluate_samples(env, model, sample_matrix, problem["names"], nominal_params, args.n_episodes)

    # Morris analysis
    si = morris_analyze.analyze(problem, sample_matrix, results)

    # Print ranked table
    param_names = problem["names"]
    mu_star = si["mu_star"]
    sigma = si["sigma"]
    ranked = np.argsort(mu_star)[::-1]

    print(f"\n{'Rank':>4}  {'Parameter':>18}  {'Category':>15}  {'mu_star':>10}  {'sigma':>10}")
    print("-" * 65)
    for rank, idx in enumerate(ranked):
        cat = PARAM_DEFINITIONS[param_names[idx]]
        print(f"{rank + 1:4d}  {param_names[idx]:>18}  {cat:>15}  {mu_star[idx]:10.4f}  {sigma[idx]:10.4f}")

    # Create plots
    output_dir = args.output if args.output else "figures/analysis/morris_param_sensitivity"
    Path(output_dir).mkdir(parents=True, exist_ok=True)

    bar_path = f"{output_dir}/{run_id}_morris_bar.png"
    scatter_path = f"{output_dir}/{run_id}_morris_scatter.png"

    create_bar_plot(si, param_names, bar_path, run_id)
    create_scatter_plot(si, param_names, scatter_path, run_id)

    env.close()
    print("Done!")


if __name__ == "__main__":
    main()
