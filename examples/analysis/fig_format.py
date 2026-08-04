"""
Shared figure formatting for IEEE paper and LaTeX thesis figures.

Usage:
    from examples.analysis.fig_format import IEEE_FULL_W, CMAP, apply_ieee_style

    with latex_style():
        ...  # build a thesis figure
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


# Serif style matching a LaTeX document, with TrueType font embedding so the PDF is
# accepted by publishers/thesis checkers.
LATEX_STYLE = {
    "font.family": "serif",
    # cmr10 is matplotlib's bundled Computer Modern, matching default LaTeX body text
    "font.serif": ["cmr10", "CMU Serif", "STIXGeneral", "DejaVu Serif"],
    # For text deliberately set off from the serif body, e.g. labels drawn over a
    # colormap. Liberation Sans is metric-compatible with Arial, so figures look the
    # same on machines without Arial installed.
    "font.sans-serif": ["Arial", "Liberation Sans", "Nimbus Sans", "Helvetica", "DejaVu Sans"],
    "mathtext.fontset": "cm",
    # matplotlib warns unless cmr10 is paired with mathtext-rendered tick labels
    "axes.formatter.use_mathtext": True,
    "axes.unicode_minus": False,  # cmr10 has no U+2212; keeps negative labels drawable
    "axes.labelsize": 13,
    "axes.titlesize": 14,
    "xtick.labelsize": 11,
    "ytick.labelsize": 11,
    "legend.fontsize": 11,
    "pdf.fonttype": 42,
    "ps.fonttype": 42,
}


def latex_style():
    """Context manager applying LATEX_STYLE for the duration of a `with` block.

    Scoped rather than global so building a thesis figure does not restyle any
    other figure drawn later in the same process.
    """
    return plt.rc_context(LATEX_STYLE)
