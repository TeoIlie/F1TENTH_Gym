"""
Shared figure formatting for IEEE paper figures.

Usage:
    from examples.analysis.fig_format import IEEE_FULL_W, CMAP, apply_ieee_style
"""

import matplotlib.pyplot as plt

IEEE_FULL_W = 7.16  # inches — full page width (figure* environment)
IEEE_COL_W = 3.45  # inches — single column width

CMAP = "magma"


def apply_ieee_style():
    """Apply clean matplotlib rcParams for IEEE figures."""
    plt.rcParams.update(
        {
            "font.family": "serif",
            "font.size": 7,
            "axes.labelsize": 7,
            "axes.titlesize": 8,
            "xtick.labelsize": 6,
            "ytick.labelsize": 6,
            "axes.linewidth": 0.5,
            "figure.dpi": 300,
        }
    )
