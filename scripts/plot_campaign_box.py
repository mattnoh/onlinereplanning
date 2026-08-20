#!/usr/bin/env python3
"""Box plots for the target-motion / uncertainty campaign.

Styling follows the v2 benchmark figure
(ALIPDDP-main/benchmark/.../quad_three_way/v2/drivers/plot_v2_benchmark.py):
serif font, one shared colour legend above the figure, categorical x labels
dropped from every panel (the legend already names each group by colour), and
panel captions carried on the x label as "(a) ...".

Two figures, because the design has only 2 trials per (motion, uncertainty)
cell -- a per-cell box would be two points and is not worth drawing:

  campaign_box_by_uncertainty.png   pools the 6 motions -> n=12 per box
  campaign_box_by_motion.png        pools the 3 uncertainty levels -> n=6 per box

Usage: plot_campaign_box.py <campaign_dir>
"""
import csv
import math
import os
import sys

import matplotlib as mpl
mpl.use("Agg")
mpl.rc("font", family="serif")
mpl.rcParams.update({"font.size": 13, "xtick.labelsize": 10.5,
                     "ytick.labelsize": 10.5, "axes.linewidth": 1.0,
                     "xtick.major.width": 0.9, "ytick.major.width": 0.9})
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
import numpy as np

LEVELS = ["zero", "nominal", "high"]
LEVEL_LABELS = ["no uncertainty", "nominal uncertainty", "high uncertainty"]
LEVEL_COLORS = ["#4c9f70", "#e6a24c", "#c9463f"]

# Ordered by how badly each motion breaks the planner's constant-acceleration
# prediction: straight is exact, random_accel is worst.
MOTIONS = ["straight", "figure8", "circle", "stop_and_go",
           "piecewise_accel", "random_accel"]
MOTION_LABELS = ["straight", "figure-eight", "circular", "stop-and-go",
                 "piecewise-const. accel.", "random accel."]
MOTION_COLORS = ["#2ca02c", "#ff7f0e", "#1f77b4", "#d62728", "#9467bd", "#8c564b"]

PANELS = [
    ("viol_glideslope", "max. glideslope violation [m]",  "(a) Glideslope"),
    ("viol_speed",      "max. speed violation [m/s]",     "(b) Touchdown speed"),
    ("viol_tilt",       "max. tilt violation [deg]",      "(c) Tilt"),
    ("viol_omega",      "max. body-rate violation [rad/s]", "(d) Body rate"),
    ("td_lateral",      "touchdown lateral error [m]",    "(e) Terminal accuracy"),
    ("track_err_med",   "tracking error, median [m]",     "(f) Tracking error"),
    ("solve_ms_med",    "solve time, median [ms]",        "(g) Computation time"),
    ("n_reject",        "rejected solves per run",        "(h) Replanning cost"),
]


def style(ax):
    """Categorical x labels are DROPPED: the shared legend above the figure
    already names every group by colour, so repeating them under eight panels
    would be pure redundancy."""
    ax.grid(True, which="major", axis="y", color="0.86", lw=0.6)
    ax.set_axisbelow(True)
    ax.tick_params(axis="x", length=0, pad=4, labelbottom=False)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)


def boxes(ax, data, colors):
    art = ax.boxplot(data, patch_artist=True, widths=0.55, showfliers=True,
                     medianprops={"color": "white", "linewidth": 2.0},
                     whiskerprops={"color": "0.25", "linewidth": 1.1},
                     capprops={"color": "0.25", "linewidth": 1.1},
                     flierprops={"marker": "o", "markersize": 4.0,
                                 "markerfacecolor": "white",
                                 "markeredgecolor": "0.30", "markeredgewidth": 0.9})
    for box, c in zip(art["boxes"], colors):
        box.set(facecolor=c, edgecolor=c, alpha=0.88, linewidth=0.9)
    # A degenerate box (all values equal, e.g. a violation that is identically
    # zero) collapses to an invisible line; draw it explicitly so "exactly
    # satisfied" is legible rather than looking like missing data.
    for i, (vals, c) in enumerate(zip(data, colors), start=1):
        q1, q3 = np.percentile(vals, [25, 75])
        if math.isclose(q1, q3, abs_tol=1e-12):
            ax.hlines(np.median(vals), i - 0.27, i + 0.27, color=c, lw=4.5, zorder=4)


def vals(rows, key):
    out = []
    for r in rows:
        v = r.get(key, "")
        if v in ("", "nan", None):
            continue
        try:
            out.append(float(v))
        except ValueError:
            pass
    return out


def draw(rows, group_key, groups, labels, colors, subtitle, outfile):
    fig, axes = plt.subplots(2, 4, figsize=(19, 9), constrained_layout=True)
    for ax, (key, ylab, cap) in zip(axes.ravel(), PANELS):
        data, kc = [], []
        for g, c in zip(groups, colors):
            v = vals([r for r in rows if r[group_key] == g], key)
            if v:
                data.append(v)
                kc.append(c)
        if not data:
            ax.set_visible(False)
            continue
        boxes(ax, data, kc)
        # Raw points overlaid: at n=6..12 the underlying spread carries more
        # information than the quartiles.
        for i, v in enumerate(data, start=1):
            ax.plot(np.full(len(v), i), v, ".", color="0.15", ms=4.5, alpha=.55,
                    zorder=5)
        ax.set_ylabel(ylab)
        ax.set_xlabel(cap, labelpad=8)
        style(ax)

    handles = [Patch(facecolor=c, edgecolor=c, alpha=0.88, label=l)
               for c, l in zip(colors, labels)]
    fig.legend(handles=handles, loc="upper center", ncol=min(len(handles), 6),
               frameon=False, fontsize=10.5, bbox_to_anchor=(0.5, 0.995))
    fig.text(0.5, 0.945, subtitle, ha="center", fontsize=11, color="0.30")
    try:
        fig.get_layout_engine().set(rect=(0, 0, 1, 0.925))
    except AttributeError:
        fig.subplots_adjust(top=0.88)
    fig.savefig(outfile, dpi=200)
    plt.close(fig)
    print("wrote", outfile)


def main():
    camp = sys.argv[1]
    rows = list(csv.DictReader(open(os.path.join(camp, "samples.csv"))))
    n_land = sum(int(r["landed"]) for r in rows)
    base = (f"PX4 SITL closed-loop receding-horizon landing — {len(rows)} runs, "
            f"{n_land}/{len(rows)} landed ({100 * n_land / len(rows):.0f}%)")

    draw(rows, "uncertainty", LEVELS, LEVEL_LABELS, LEVEL_COLORS,
         base + "   ·   6 target motions pooled, n = 12 per box",
         os.path.join(camp, "campaign_box_by_uncertainty.png"))

    draw(rows, "motion", MOTIONS, MOTION_LABELS, MOTION_COLORS,
         base + "   ·   3 uncertainty levels pooled, n = 6 per box",
         os.path.join(camp, "campaign_box_by_motion.png"))


if __name__ == "__main__":
    main()
