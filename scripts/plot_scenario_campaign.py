#!/usr/bin/env python3
"""Plot a scenario campaign: land rate and degradation vs target uncertainty.

Usage:  plot_scenario_campaign.py <campaign_dir>
Reads samples.csv (written by scenario_metrics.py) and writes
campaign_summary.png next to it.
"""
import csv
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

LEVELS = ["zero", "nominal", "high"]
COLORS = {"circle": "#1f77b4", "figure8": "#ff7f0e", "straight": "#2ca02c",
          "stop_and_go": "#d62728", "piecewise_accel": "#9467bd",
          "random_accel": "#8c564b"}


def main():
    camp = sys.argv[1]
    rows = list(csv.DictReader(open(os.path.join(camp, "samples.csv"))))
    if not rows:
        print("no samples")
        return
    motions = sorted({r["motion"] for r in rows})

    def cell(mo, lv):
        return [r for r in rows if r["motion"] == mo and r["uncertainty"] == lv]

    def agg(mo, lv, key):
        vals = []
        for r in cell(mo, lv):
            try:
                vals.append(float(r[key]))
            except (ValueError, KeyError):
                pass
        return sum(vals) / len(vals) if vals else None

    panels = [("land rate", None), ("touchdown lateral [m]", "td_lateral"),
              ("max STC violation", "max_stc_viol"), ("tracking err med [m]", "track_err_med"),
              ("solve time med [ms]", "solve_ms_med"), ("deadline miss rate", "deadline_miss_rate")]
    fig, axes = plt.subplots(2, 3, figsize=(16, 8.5))
    x = range(len(LEVELS))

    for ax, (title, key) in zip(axes.ravel(), panels):
        for mo in motions:
            ys = []
            for lv in LEVELS:
                g = cell(mo, lv)
                if not g:
                    ys.append(None); continue
                if key is None:
                    ys.append(100.0 * sum(int(r["landed"]) for r in g) / len(g))
                else:
                    ys.append(agg(mo, lv, key))
            xs = [i for i, y in zip(x, ys) if y is not None]
            yy = [y for y in ys if y is not None]
            if xs:
                ax.plot(xs, yy, "o-", label=mo, color=COLORS.get(mo), ms=5)
        ax.set_xticks(list(x)); ax.set_xticklabels(LEVELS)
        ax.set_title(title); ax.grid(alpha=.3)
        ax.set_xlabel("target-estimate uncertainty")
        if key is None:
            ax.set_ylim(-5, 105); ax.set_ylabel("%")
    axes.ravel()[0].legend(fontsize=8, ncol=2)
    fig.suptitle("Target-motion generalization and target-state uncertainty — PX4 SITL",
                 fontweight="bold")
    fig.tight_layout()
    out = os.path.join(camp, "campaign_summary.png")
    fig.savefig(out, dpi=140)
    print("wrote", out)


if __name__ == "__main__":
    main()
