#!/usr/bin/env python3
"""Figures for the two Monte-Carlo studies, as two SEPARATE deliverables.

  MC1  target-motion generalization  -- does STC activation, replanning and
       landing work across target motions, independent of the circular
       condition? Grouped by motion, at the no-added-uncertainty condition.

  MC2  target-state / acceleration-prediction uncertainty -- how does
       performance degrade from nominal to high uncertainty, with model
       mismatch against the planner's constant-acceleration prediction?
       Grouped by uncertainty level.

Panel structure and styling follow the v2 benchmark figure
(ALIPDDP-main/.../quad_three_way/v2/drivers/plot_v2_benchmark.py): rate bar
with Wilson intervals, filled boxes with white medians, an empirical CDF, one
shared colour legend above the figure, categorical x labels dropped, and panel
captions carried on the x label.

Usage: plot_campaign_mc.py <campaign_dir>
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
from matplotlib.ticker import FuncFormatter
import numpy as np

LEVELS = ["zero", "nominal", "high"]
LEVEL_LABELS = {"zero": "no added uncertainty", "nominal": "nominal uncertainty",
                "high": "high uncertainty"}
LEVEL_COLORS = {"zero": "#4c9f70", "nominal": "#e6a24c", "high": "#c9463f"}

MOTIONS = ["straight", "figure8", "circle", "stop_and_go",
           "piecewise_accel", "random_accel"]
MOTION_LABELS = {"straight": "straight", "figure8": "figure-eight",
                 "circle": "circular", "stop_and_go": "stop-and-go",
                 "piecewise_accel": "piecewise-const. accel.",
                 "random_accel": "random accel."}
MOTION_COLORS = {"straight": "#2ca02c", "figure8": "#ff7f0e", "circle": "#1f77b4",
                 "stop_and_go": "#d62728", "piecewise_accel": "#9467bd",
                 "random_accel": "#8c564b"}

FLOOR = 1e-4          # log-axis floor so an exactly-satisfied constraint is visible


def wilson(successes, total, z=1.96):
    if total == 0:
        return math.nan, math.nan
    p = successes / total
    d = 1.0 + z * z / total
    c = (p + z * z / (2.0 * total)) / d
    r = z * math.sqrt(p * (1 - p) / total + z * z / (4.0 * total * total)) / d
    return max(0.0, c - r), min(1.0, c + r)


def readable_log_tick(value, _pos):
    if value <= 0:
        return ""
    e = math.log10(value)
    if abs(e - round(e)) > 1e-6:
        return ""
    return f"$10^{{{int(round(e))}}}$"


def style(ax):
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
    for i, (vals, c) in enumerate(zip(data, colors), start=1):
        q1, q3 = np.percentile(vals, [25, 75])
        if math.isclose(q1, q3, abs_tol=1e-12):
            ax.hlines(np.median(vals), i - 0.27, i + 0.27, color=c, lw=4.5, zorder=4)


def num(rows, key, floor=None):
    out = []
    for r in rows:
        v = r.get(key, "")
        if v in ("", "nan", None):
            continue
        try:
            x = float(v)
        except ValueError:
            continue
        out.append(max(x, floor) if floor is not None else x)
    return out


def figure(rows, groups, labels, colors, title, subtitle, outfile, difficulty=None):
    fig, axes = plt.subplots(2, 3, figsize=(10.4 + 1.3 * len(groups), 9.0),
                             constrained_layout=True)
    (ax_rate, ax_err, ax_viol), (ax_cdf, ax_time, ax_last) = axes
    G = [[r for r in rows if r["_g"] == g] for g in groups]
    n_per = [len(g) for g in G]

    # (a) landing success rate, Wilson interval
    xs = np.arange(1, len(groups) + 1)
    rates = [sum(int(r["landed"]) for r in g) / len(g) if g else 0 for g in G]
    ci = [wilson(sum(int(r["landed"]) for r in g), len(g)) for g in G]
    ax_rate.bar(xs, rates, width=0.55, color=colors, alpha=0.88,
                edgecolor=colors, linewidth=0.9)
    ax_rate.errorbar(xs, rates,
                     yerr=[[max(0.0, r - lo) for r, (lo, _) in zip(rates, ci)],
                           [max(0.0, hi - r) for r, (_, hi) in zip(rates, ci)]],
                     fmt="none", ecolor="0.25", elinewidth=0.85, capsize=4)
    ax_rate.set_ylim(0, 1.12)
    ax_rate.set_ylabel("landing success rate")
    ax_rate.set_xlabel(f"(a) Landing success  (n={n_per[0]} per group, 95% Wilson)",
                       labelpad=8)
    style(ax_rate)

    # (b) terminal / touchdown accuracy
    boxes(ax_err, [num(g, "td_lateral") or [0] for g in G], colors)
    ax_err.set_ylabel("touchdown lateral error [m]")
    ax_err.set_xlabel("(b) Terminal accuracy", labelpad=8)
    style(ax_err)

    # (c) maximum STC violation. Per-term values have different units, so the
    # glideslope term (metres) is plotted -- a max() across m / m/s / deg /
    # rad/s would just report whichever term is degree-valued.
    boxes(ax_viol, [num(g, "viol_glideslope", FLOOR) or [FLOOR] for g in G], colors)
    ax_viol.set_yscale("log")
    ax_viol.yaxis.set_major_formatter(FuncFormatter(readable_log_tick))
    ax_viol.set_ylabel("max. glideslope violation [m]")
    ax_viol.set_xlabel("(c) Max. STC violation", labelpad=8)
    style(ax_viol)

    # (d) tracking-error CDF
    for g, c, lab in zip(G, colors, labels):
        v = np.sort(num(g, "track_err_med"))
        if v.size:
            ax_cdf.step(v, np.arange(1, v.size + 1) / v.size, where="post",
                        color=c, lw=1.9)
    ax_cdf.set_ylim(0, 1.02)
    ax_cdf.set_xlabel("(d) Tracking-error CDF  [m]", labelpad=8)
    ax_cdf.set_ylabel("empirical CDF")
    ax_cdf.grid(True, color="0.86", lw=0.6)
    ax_cdf.set_axisbelow(True)
    for s in ("top", "right"):
        ax_cdf.spines[s].set_visible(False)

    # (e) computation time, p95 marked
    tim = [num(g, "solve_ms_med") or [0] for g in G]
    boxes(ax_time, tim, colors)
    for i, v in enumerate(tim, start=1):
        ax_time.plot([i], [np.percentile(v, 95)], marker="_", ms=18, mew=2.0,
                     color="0.15", zorder=5)
    ax_time.set_ylabel("solve time, median [ms]")
    ax_time.set_xlabel("(e) Computation time  (bar = p95)", labelpad=8)
    style(ax_time)

    # (f) either the scenario difficulty (MC1) or the replanning cost (MC2)
    if difficulty is not None:
        w = 0.38
        vmax = [difficulty[g]["v_max"] for g in groups]
        amax = [difficulty[g]["a_max"] for g in groups]
        ax_last.bar(xs - w / 2, vmax, width=w, color=colors, alpha=0.88,
                    edgecolor=colors, linewidth=0.9)
        ax_last.bar(xs + w / 2, amax, width=w, color=colors, alpha=0.45,
                    edgecolor=colors, linewidth=0.9, hatch="///")
        for x, g in zip(xs, groups):
            ax_last.annotate(f"{difficulty[g]['path_m']:.0f} m",
                             (x, max(difficulty[g]['v_max'], difficulty[g]['a_max'])),
                             textcoords="offset points", xytext=(0, 5),
                             ha="center", fontsize=8.5, color="0.30")
        ax_last.set_ylabel(r"solid: $|v|_{max}$ [m/s]   hatched: $|a|_{max}$ [m/s$^2$]")
        ax_last.set_xlabel("(f) Target-motion difficulty  (label = path length)",
                           labelpad=8)
        style(ax_last)
    else:
        rej = [num(g, "n_reject") or [0] for g in G]
        boxes(ax_last, rej, colors)
        ax_last.set_ylabel("rejected solves per run")
        ax_last.set_xlabel("(f) Replanning cost", labelpad=8)
        style(ax_last)

    handles = [Patch(facecolor=c, edgecolor=c, alpha=0.88, label=l)
               for c, l in zip(colors, labels)]
    fig.legend(handles=handles, loc="upper center", ncol=min(len(handles), 6),
               frameon=False, fontsize=10.5, bbox_to_anchor=(0.5, 0.997))
    fig.text(0.5, 0.955, title, ha="center", fontsize=13, fontweight="bold")
    fig.text(0.5, 0.930, subtitle, ha="center", fontsize=10.5, color="0.30")
    try:
        fig.get_layout_engine().set(rect=(0, 0, 1, 0.915))
    except AttributeError:
        fig.subplots_adjust(top=0.87)
    fig.savefig(outfile, dpi=200)
    plt.close(fig)
    print("wrote", outfile)


def main():
    camp = sys.argv[1]
    rows = list(csv.DictReader(open(os.path.join(camp, "samples.csv"))))
    diff = {r["motion"]: {k: float(v) for k, v in r.items() if k != "motion"}
            for r in csv.DictReader(open(os.path.join(camp, "scenario_difficulty.csv")))}

    # ---- MC1: motion generalization, at the no-added-uncertainty condition ----
    m1 = [dict(r, _g=r["motion"]) for r in rows if r["uncertainty"] == "zero"]
    n1 = len(m1)
    figure(m1, MOTIONS, [MOTION_LABELS[m] for m in MOTIONS],
           [MOTION_COLORS[m] for m in MOTIONS],
           "MC1 — Target-motion generalization",
           f"PX4 SITL closed-loop receding-horizon landing · {n1} runs, "
           "no added target-state uncertainty",
           os.path.join(camp, "mc1_motion_generalization.png"),
           difficulty=diff)

    # ---- MC2: uncertainty analysis, pooled over motions ----
    m2 = [dict(r, _g=r["uncertainty"]) for r in rows]
    figure(m2, LEVELS, [LEVEL_LABELS[l] for l in LEVELS],
           [LEVEL_COLORS[l] for l in LEVELS],
           "MC2 — Target-state and acceleration-prediction uncertainty",
           f"PX4 SITL closed-loop receding-horizon landing · {len(m2)} runs, "
           "6 target motions pooled · model mismatch vs constant-accel prediction",
           os.path.join(camp, "mc2_uncertainty.png"))


if __name__ == "__main__":
    main()
