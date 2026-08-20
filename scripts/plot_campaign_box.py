#!/usr/bin/env python3
"""Box plots for the target-motion / uncertainty campaign.

Two figures, because the design has only 2 trials per (motion, uncertainty)
cell -- a per-cell box would be two points and is not worth drawing:

  campaign_box_by_uncertainty.png   pools the 6 motions -> n=12 per box,
                                    isolating the effect of uncertainty
  campaign_box_by_motion.png        pools the 3 uncertainty levels -> n=6 per
                                    box, isolating the effect of target motion

Usage: plot_campaign_box.py <campaign_dir>
"""
import csv
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

LEVELS = ["zero", "nominal", "high"]
# Ordered by how badly each motion breaks the planner's constant-acceleration
# prediction: straight is exact, random_accel is worst.
MOTIONS = ["straight", "figure8", "circle", "stop_and_go",
           "piecewise_accel", "random_accel"]

METRICS = [
    ("viol_glideslope", "glideslope violation [m]"),
    ("viol_speed",      "speed violation [m/s]"),
    ("viol_tilt",       "tilt violation [deg]"),
    ("viol_omega",      "omega violation [rad/s]"),
    ("td_lateral",      "touchdown lateral error [m]"),
    ("track_err_med",   "tracking error, median [m]"),
    ("solve_ms_med",    "solve time, median [ms]"),
    ("n_reject",        "rejected solves per run"),
]


def load(camp):
    return list(csv.DictReader(open(os.path.join(camp, "samples.csv"))))


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


def draw(rows, group_key, groups, title, outfile, colors):
    fig, axes = plt.subplots(2, 4, figsize=(19, 8.5))
    for ax, (key, label) in zip(axes.ravel(), METRICS):
        data, keep, kc = [], [], []
        for g, c in zip(groups, colors):
            v = vals([r for r in rows if r[group_key] == g], key)
            if v:
                data.append(v)
                keep.append(g)
                kc.append(c)
        if not data:
            ax.set_visible(False)
            continue
        bp = ax.boxplot(data, labels=keep, patch_artist=True, widths=0.6,
                        medianprops=dict(color="black", lw=1.6),
                        flierprops=dict(marker="o", ms=3, alpha=.6))
        for patch, c in zip(bp["boxes"], kc):
            patch.set_facecolor(c)
            patch.set_alpha(0.55)
        # Overlay the raw points: with n=6..12 per box the underlying spread
        # matters more than the quartiles.
        for i, v in enumerate(data, start=1):
            ax.plot([i] * len(v), v, "k.", ms=4, alpha=.55)
        ax.set_title(label, fontsize=11)
        ax.grid(alpha=.3, axis="y")
        if max(len(k) for k in keep) > 8:
            ax.tick_params(axis="x", rotation=30, labelsize=8)
    fig.suptitle(title, fontweight="bold", fontsize=13)
    fig.tight_layout()
    fig.savefig(outfile, dpi=140)
    print("wrote", outfile)


def main():
    camp = sys.argv[1]
    rows = load(camp)
    n_land = sum(int(r["landed"]) for r in rows)
    hdr = f"PX4 SITL closed-loop campaign — {len(rows)} runs, {n_land} landed ({100*n_land/len(rows):.0f}%)"

    draw(rows, "uncertainty", LEVELS,
         hdr + "\nEffect of target-state estimate uncertainty (6 motions pooled, n=12 per box)",
         os.path.join(camp, "campaign_box_by_uncertainty.png"),
         ["#4c9f70", "#e6a24c", "#c9463f"])

    draw(rows, "motion", MOTIONS,
         hdr + "\nEffect of target motion (3 uncertainty levels pooled, n=6 per box)",
         os.path.join(camp, "campaign_box_by_motion.png"),
         ["#2ca02c", "#ff7f0e", "#1f77b4", "#d62728", "#9467bd", "#8c564b"])


if __name__ == "__main__":
    main()
