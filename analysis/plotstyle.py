"""Shared matplotlib style for all figures."""

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

BLUE, ORANGE, AQUA, VIOLET = "#2a78d6", "#eb6834", "#1baf7a", "#4a3aa7"
INK, INK2, GRID = "#0b0b0b", "#52514e", "#e4e3df"
SURFACE = "#fcfcfb"


def apply():
    plt.rcParams.update({
        "figure.facecolor": SURFACE,
        "axes.facecolor": SURFACE,
        "axes.edgecolor": "#b9b8b3",
        "axes.labelcolor": INK2,
        "axes.titlecolor": INK,
        "axes.titlesize": 10.5,
        "axes.titleweight": "bold",
        "axes.titlelocation": "left",
        "axes.labelsize": 9,
        "axes.grid": True,
        "axes.spines.top": False,
        "axes.spines.right": False,
        "grid.color": GRID,
        "grid.linewidth": 0.6,
        "xtick.color": INK2,
        "ytick.color": INK2,
        "xtick.labelsize": 8,
        "ytick.labelsize": 8,
        "lines.linewidth": 1.8,
        "legend.fontsize": 8,
        "legend.frameon": False,
        "font.family": "DejaVu Sans",
        "savefig.dpi": 150,
        "savefig.bbox": "tight",
    })


def save(fig, path):
    fig.savefig(path, facecolor=SURFACE)
    plt.close(fig)
    print(f"wrote {path}")
