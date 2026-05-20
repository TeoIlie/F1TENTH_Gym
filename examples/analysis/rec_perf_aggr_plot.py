"""
Recovery Performance Aggregate Plot — IEEE Paper Figure

Takes 2–5 grid_results.npz cache files (produced by beta_r_avg_plot.py) and
creates a single publication-ready figure: heatmaps side-by-side with one
shared colorbar, sized for IEEE full page width (figure* environment).

Usage:
    python examples/analysis/rec_perf_aggr_plot.py \
        --npz <path1> <path2> <path3> \
        --labels "Stanley" "PPO (recover)" "PPO (transfer)" \
        --output figures/analysis/recover_heatmap/comparison.pdf
"""

import argparse
import os

import matplotlib.pyplot as plt
import numpy as np

from examples.analysis.fig_format import CMAP, IEEE_FULL_W, apply_ieee_style


def parse_args():
    parser = argparse.ArgumentParser(description="Aggregate recovery heatmap for IEEE paper")
    parser.add_argument("--npz", nargs="+", required=True, metavar="PATH", help="2–5 grid_results.npz cache files")
    parser.add_argument("--labels", nargs="+", required=True, metavar="LABEL", help="Controller label for each subplot")
    parser.add_argument("--output", required=True, help="Output file path (e.g. comparison.pdf)")
    return parser.parse_args()


def load_and_validate(npz_paths):
    """Load .npz cache files and validate that grid parameters match.

    Returns:
        recovery_rates_list: list of recovery_rates arrays (values in [0, 1])
        beta_values: shared beta grid (radians)
        r_values: shared r grid (rad/s)
    """
    n = len(npz_paths)
    if not 2 <= n <= 5:
        raise ValueError(f"Expected 2–5 .npz files, got {n}")

    datasets = [np.load(p) for p in npz_paths]

    # Use first file as reference grid
    ref = datasets[0]
    ref_keys = ("beta_values", "r_values", "v_values", "yaw_values")

    for i, data in enumerate(datasets[1:], start=1):
        for key in ref_keys:
            if not np.allclose(data[key], ref[key]):
                raise ValueError(
                    f"Grid mismatch on '{key}' between {npz_paths[0]} and {npz_paths[i]}. "
                    "All files must be produced with the same BETA/R/V/YAW grid."
                )

    recovery_rates_list = [data["recovery_rates"] for data in datasets]
    return recovery_rates_list, ref["beta_values"], ref["r_values"]


def plot_comparison(recovery_rates_list, beta_values, r_values, labels, output_path):
    """Create and save the N-panel IEEE-ready comparison figure."""
    apply_ieee_style()

    n = len(recovery_rates_list)
    beta_deg = np.rad2deg(beta_values)
    r_deg = np.rad2deg(r_values)

    # Scale height with panel count to keep cells roughly square
    height = {2: 2.8, 3: 2.5, 4: 1.9, 5: 2.0}[n]
    fig, axes = plt.subplots(1, n, figsize=(IEEE_FULL_W, height), constrained_layout=True)
    if n == 1:
        axes = [axes]

    im = None
    for i, (ax, rates, label) in enumerate(zip(axes, recovery_rates_list, labels)):
        pct = rates * 100
        im = ax.imshow(
            pct.T,
            origin="lower",
            aspect="auto",
            cmap=CMAP,
            vmin=0,
            vmax=100,
            interpolation="bilinear",
        )

        ax.set_xticks(range(len(beta_deg)))
        ax.set_xticklabels([f"{v:.0f}" for v in beta_deg])
        ax.set_xlabel(r"$\beta$ [deg]")
        ax.set_title(label)
        ax.tick_params(length=2, width=0.5)

        if i == 0:
            ax.set_yticks(range(len(r_deg)))
            ax.set_yticklabels([f"{v:.0f}" for v in r_deg])
            ax.set_ylabel(r"$r$ [deg/s]")
        else:
            ax.set_yticks([])

    # Single shared colorbar to the right of all subplots
    cbar = fig.colorbar(im, ax=list(axes), shrink=0.85, pad=0.02)
    cbar.set_label("Recovery Rate (%)")
    cbar.ax.tick_params(labelsize=6)

    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    fig.savefig(output_path, dpi=300, bbox_inches="tight")
    print(f"Saved: {output_path}")


def main():
    args = parse_args()
    if len(args.npz) != len(args.labels):
        raise ValueError(f"Got {len(args.npz)} .npz files but {len(args.labels)} labels — counts must match")
    recovery_rates_list, beta_values, r_values = load_and_validate(args.npz)
    plot_comparison(recovery_rates_list, beta_values, r_values, args.labels, args.output)


if __name__ == "__main__":
    main()
