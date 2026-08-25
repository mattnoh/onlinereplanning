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

# Two nested success criteria, because the bare one is uninformative on its own.
#
#   landed        the FCU disarmed after having been armed -- a LIVENESS test.
#                 It says the STC block activated, the horizon kept replanning
#                 and the vehicle reached the ground without the run timing
#                 out. It says nothing about WHERE it landed, which is why it
#                 reads 100% everywhere.
#   precision     landed AND touchdown lateral error <= PRECISION_M.
#
# PRECISION_M is cap_radius / 3 (the pad radius is 0.45 m in every run's
# constraints.csv), so it is a fixed fraction of the actual landing target
# rather than a threshold picked to make the bars look interesting.
PRECISION_M = 0.15


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
    # tight_layout(rect=...) rather than constrained_layout: this matplotlib
    # (3.5) has no Figure.set_layout_engine, and mixing subplots_adjust into a
    # constrained_layout figure disables it -- which let panel (a)'s two-line
    # caption run into the row below.
    fig, axes = plt.subplots(2, 3, figsize=(10.4 + 1.3 * len(groups), 9.4))
    (ax_rate, ax_err, ax_viol), (ax_cdf, ax_time, ax_last) = axes
    G = [[r for r in rows if r["_g"] == g] for g in groups]
    n_per = [len(g) for g in G]

    # (a) landing success. Outlined bar = liveness (landed at all); filled bar
    # = precision landing. Showing only the first gives six identical 100%
    # bars; showing only the second hides that every run did in fact complete.
    xs = np.arange(1, len(groups) + 1)
    def _lat(r):
        try:
            return float(r["td_lateral"])
        except (ValueError, KeyError):
            return float("inf")
    live = [sum(int(r["landed"]) for r in g) for g in G]
    prec = [sum(1 for r in g if int(r["landed"]) and _lat(r) <= PRECISION_M)
            for g in G]
    rates = [k / len(g) if g else 0 for k, g in zip(live, G)]
    prates = [k / len(g) if g else 0 for k, g in zip(prec, G)]
    ci = [wilson(k, len(g)) for k, g in zip(prec, G)]
    ax_rate.bar(xs, rates, width=0.55, facecolor="none", edgecolor=colors,
                linewidth=1.4, linestyle=(0, (3, 2)))
    ax_rate.bar(xs, prates, width=0.55, color=colors, alpha=0.88,
                edgecolor=colors, linewidth=0.9)
    ax_rate.errorbar(xs, prates,
                     yerr=[[max(0.0, r - lo) for r, (lo, _) in zip(prates, ci)],
                           [max(0.0, hi - r) for r, (_, hi) in zip(prates, ci)]],
                     fmt="none", ecolor="0.25", elinewidth=0.85, capsize=4)
    ax_rate.set_ylim(0, 1.12)
    ax_rate.set_ylabel("success rate")
    ax_rate.set_xlabel(f"(a) Landing success  (dashed: landed, n={n_per[0]}/group;"
                       f"\n      filled: lateral err $\\leq$ {PRECISION_M:g} m, 95% Wilson)",
                       labelpad=8)
    style(ax_rate)

    # (b) terminal / touchdown accuracy
    boxes(ax_err, [num(g, "td_lateral") or [0] for g in G], colors)
    ax_err.axhline(PRECISION_M, color="0.35", lw=1.0, ls=(0, (4, 3)), zorder=1)
    ax_err.annotate(f"precision criterion {PRECISION_M:g} m", (0.02, PRECISION_M),
                    xycoords=("axes fraction", "data"), xytext=(0, 4),
                    textcoords="offset points", ha="left", va="bottom",
                    fontsize=9, color="0.35")
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

    # Title block FIRST, legend below it. The legend used to be anchored above
    # the title, and with six groups its row ran straight through the title
    # text. Stacking them in reading order removes the collision by
    # construction rather than by tuning two y offsets against each other.
    fig.text(0.5, 0.988, title, ha="center", va="top", fontsize=16,
             fontweight="bold")
    fig.text(0.5, 0.962, subtitle, ha="center", va="top", fontsize=11.5,
             color="0.30")
    handles = [Patch(facecolor=c, edgecolor=c, alpha=0.88, label=l)
               for c, l in zip(colors, labels)]
    fig.legend(handles=handles, loc="upper center", ncol=min(len(handles), 6),
               frameon=False, fontsize=14, handlelength=1.6, handleheight=1.1,
               columnspacing=1.8, bbox_to_anchor=(0.5, 0.945))
    # Reserve the top strip for the title block + legend. set_layout_engine
    # keeps constrained_layout ON; subplots_adjust would silently disable it
    # and let panel (a)'s two-line caption run into the row below.
    fig.tight_layout(rect=(0, 0, 1, 0.885), h_pad=2.4, w_pad=2.0)
    fig.savefig(outfile, dpi=200)
    plt.close(fig)
    print("wrote", outfile)


def main():
    camp = sys.argv[1]
    which = sys.argv[2] if len(sys.argv) > 2 else "auto"
    rows = list(csv.DictReader(open(os.path.join(camp, "samples.csv"))))
    dfile = os.path.join(camp, "scenario_difficulty.csv")
    diff = ({r["motion"]: {k: float(v) for k, v in r.items() if k != "motion"}
             for r in csv.DictReader(open(dfile))} if os.path.exists(dfile) else None)

    levels_present = sorted({r["uncertainty"] for r in rows})
    motions_present = sorted({r["motion"] for r in rows})
    if which == "auto":
        which = "mc1" if len(levels_present) == 1 else "mc2"

    if which == "mc1":
        groups = [m for m in MOTIONS if m in motions_present]
        m1 = [dict(r, _g=r["motion"]) for r in rows]
        figure(m1, groups, [MOTION_LABELS[m] for m in groups],
               [MOTION_COLORS[m] for m in groups],
               "MC1 — Target-motion generalization",
               f"PX4 SITL closed-loop receding-horizon landing · {len(m1)} runs "
               f"({len(m1)//max(1,len(groups))} per motion) · no added target-state uncertainty",
               os.path.join(camp, "mc1_motion_generalization.png"),
               difficulty=diff)
    else:
        groups = [l for l in LEVELS if l in levels_present]
        m2 = [dict(r, _g=r["uncertainty"]) for r in rows]
        base = ", ".join(MOTION_LABELS.get(m, m) for m in motions_present)
        figure(m2, groups, [LEVEL_LABELS[l] for l in groups],
               [LEVEL_COLORS[l] for l in groups],
               "MC2 — Target-state and acceleration-prediction uncertainty",
               f"PX4 SITL closed-loop receding-horizon landing · {len(m2)} runs "
               f"({len(m2)//max(1,len(groups))} per level) · {base} target · "
               "model mismatch vs constant-acceleration prediction",
               os.path.join(camp, "mc2_uncertainty.png"))


if __name__ == "__main__":
    main()
