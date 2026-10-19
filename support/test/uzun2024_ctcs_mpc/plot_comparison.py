"""Compare continuous-time constraint maintenance: {SCP, DDP} x {CTCS, nodal-only}.

Reads only saved campaign outputs (no solving, no re-integration):

  <case>/closed_loop_dense.npz   executed plant replay on the 1 ms audit grid (times, states, g)
  <case>/executed_intervals.csv  per executed interval DOP853 audit (integral, refined max g, ...)
  <case>/summary.json            closed_loop.common_summarize and pass/fail flags
  <case>/config.json             epsilon, delta
  <case>/arrays.npz              per-solve sensed mask (only for the delta=0.05 cross-check)
  <case>/solves.csv              per-solve timing / iteration counts (see PERF_DEFINITIONS)

Physical constraint: g_i = 1 - ||S (r - c_i)||_2 for all five obstacles (common.obstacle_g);
g_i > 0 is inside obstacle i.  Every "g" below is the signed max_i g_i, never the pointwise
sum of excesses (that is reported separately as max_sum_excess, labelled as such).

Outputs (in --output-dir):
  fig_comparison.{png,pdf}    trajectories, max_i g_i(t) with node markers, tracking error,
                              audited per-interval integral / epsilon
  fig_intersample.{png,pdf}   worst observed nodal-only interval: dense g(t), node endpoints
  fig_2x2.{png,pdf}           SCP CTCS | SCP nodal / DDP CTCS | DDP nodal, shared geometry and
                              g axis (only when all four cases are loaded)
  fig_performance.{png,pdf}   tracking error, physical max g, per-solve solver time
  comparison_stats.json       per-case extrema, durations, node-vs-dense evidence, run flags,
                              solver time / iteration statistics
  comparison_stats.csv        flat version of the per-case statistics
  executed_intervals_all.csv  per executed interval, all cases (node, dense and refined g)
  plotted_series.csv          dense series exactly as plotted (deduplicated junctions)
  plotted_nodes.csv           0.5 s executed-node samples exactly as plotted
  closeup_series.csv          dense series of the close-up windows
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import common  # noqa: E402

import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402
from matplotlib.patches import Ellipse, Patch  # noqa: E402

DEFAULT_LOGS = Path(__file__).resolve().parents[3] / "logs" / "uzun2024_ctcs_mpc"

# role: "main" = matched comparison (same delta=0.05 tightening), "separate" = shown but
# not part of the matched comparison, "supplementary" = statistics only.
# nodal: constraint enforced only at nodes (candidate for intersample violation close-up).
CASES = {
    "SCP3FOH": dict(dir="P3_foh_costscale100", label="SCP CTCS, FOH, 3 it. (supplementary FOH)",
                    role="supplementary", nodal=False, color="#56B4E9", ls="-"),
    "SCP10FOH": dict(dir="P4_foh_10iters", label="SCP CTCS, FOH, 10 it. (supplementary FOH)",
                     role="supplementary", nodal=False, color="#E69F00", ls="-"),
    "SCP10ZOH": dict(dir="P5_zoh_10iters", label="SCP CTCS, ZOH, 10 it.",
                     role="main", nodal=False, color="#0072B2", ls="-"),
    # same ZOH / 10 it. / cost scale 100 / delta 0.05 settings as P5, scp.py --constraint-mode stage
    "SCPNodal": dict(dir="P10b_scp_stage_zoh_10iters", label=r"SCP nodal only, ZOH, 10 it., $g+0.05\leq 0$",
                     short=r"SCP nodal, $g+0.05\leq 0$",
                     role="main", nodal=True, color="#882255", ls="-"),
    "DDPCTCS": dict(dir="P7_ddp_zoh_reuse", label="DDP CTCS, ZOH",
                    role="main", nodal=False, color="#009E73", ls="-"),
    "DDPNodal": dict(dir="P8_ddp_stage_tightened", label=r"DDP nodal only, $g+0.05\leq 0$",
                     short=r"DDP nodal, $g+0.05\leq 0$",
                     role="main", nodal=True, color="#D55E00", ls="-"),
    "DDPNodal0": dict(dir="P9_ddp_stage_untightened",
                      label=r"DDP nodal only, $g\leq 0$ (margin 0, not matched)",
                      short=r"DDP nodal, $g\leq 0$ (margin 0)",
                      role="separate", nodal=True, color="#CC79A7", ls="--"),
}
DEFAULT_CASES = list(CASES)
# fig_2x2 layout: (panel name, case key, title); rows = solver, columns = constraint handling
PANELS = [("SCPCTCS", "SCP10ZOH", "SCP, CTCS (ZOH, 10 it.)"),
          ("SCPNodal", "SCPNodal", r"SCP, nodal only, $g+0.05\leq 0$"),
          ("DDPCTCS", "DDPCTCS", "DDP, CTCS (ZOH)"),
          ("DDPNodal", "DDPNodal", r"DDP, nodal only, $g+0.05\leq 0$")]
NODE_DT = 0.5
TIME_TOL = 1e-7


# ---------------------------------------------------------------------------
# Loading
# ---------------------------------------------------------------------------
def _read_csv(path):
    with open(path, newline="") as fh:
        return list(csv.DictReader(fh))


def _f(row, key, default=np.nan):
    v = row.get(key, "")
    if v in ("", None):
        return default
    if v in ("True", "False"):
        return float(v == "True")
    return float(v)


def _required(case_dir):
    return [case_dir / n for n in ("closed_loop_dense.npz", "executed_intervals.csv",
                                   "summary.json", "config.json")]


def dedupe_junctions(times, *arrays):
    """Drop duplicated interval-junction samples (keep the first, as common.concat_dense).

    Returns (times, arrays, n_dropped, max state mismatch at the dropped samples)."""
    times = np.asarray(times, float)
    if np.any(np.diff(times) < -TIME_TOL):
        raise ValueError("dense times are not monotone")
    dup = np.concatenate([[False], np.abs(np.diff(times)) <= 1e-12])
    mismatch = 0.0
    if dup.any():
        idx = np.where(dup)[0]
        for a in arrays:
            if a.ndim > 1 and a.shape[1] in (4, 5):
                mismatch = max(mismatch, float(np.max(np.abs(a[idx] - a[idx - 1]))))
    keep = ~dup
    return times[keep], [a[keep] for a in arrays], int(dup.sum()), mismatch


def load_case(key, spec, case_dir):
    d = np.load(case_dir / "closed_loop_dense.npz")
    t, (X, G), ndup, mism = dedupe_junctions(d["times"], d["states"], d["g"])
    g_recomputed = common.obstacle_g(X[:, 0:2])
    g_model_err = float(np.max(np.abs(g_recomputed - G)))
    ref = common.reference(t)
    if "reference" in d.files:
        ref_saved = dedupe_junctions(d["times"], d["reference"])[1][0]
        ref_err = float(np.max(np.abs(ref_saved - ref)))
    else:
        ref_err = None
    summary = json.loads((case_dir / "summary.json").read_text())
    config = json.loads((case_dir / "config.json").read_text())
    cargs = config.get("args", config)
    rows = _read_csv(case_dir / "executed_intervals.csv")
    if not rows:
        raise ValueError(f"{case_dir}/executed_intervals.csv has no executed intervals")
    masks = None
    if (case_dir / "arrays.npz").exists():
        a = np.load(case_dir / "arrays.npz")
        if "mask" in a.files:
            masks = np.asarray(a["mask"], bool)
    solves = _read_csv(case_dir / "solves.csv") if (case_dir / "solves.csv").exists() else []
    return dict(key=key, spec=spec, dir=case_dir, t=t, X=X, G=G, ref=ref,
                gmax=G.max(axis=1), gobs=G.argmax(axis=1),
                err=np.linalg.norm(X[:, 0:2] - ref, axis=1),
                summary=summary, config=config,
                epsilon=float(cargs.get("epsilon", common.ASSUMED_DEFAULTS["epsilon"])),
                delta=float(cargs.get("delta", common.DELTA_OBS)),
                rows=rows, masks=masks, solves=solves,
                n_junction_duplicates=ndup, junction_state_mismatch=mism,
                g_model_check=g_model_err, reference_check=ref_err)


def _idx_at(t, tq):
    i = int(np.clip(np.searchsorted(t, tq), 0, t.size - 1))
    if i > 0 and abs(t[i - 1] - tq) < abs(t[i] - tq):
        i -= 1
    if abs(t[i] - tq) > TIME_TOL:
        raise ValueError(f"no dense sample at node time {tq:.9f} (nearest {t[i]:.9f})")
    return i


def interval_table(c):
    """Per executed interval: node endpoint g, dense max g, refined audited max g, integrals."""
    t, G, gmax = c["t"], c["G"], c["gmax"]
    out = []
    for r in c["rows"]:
        t0, dur = _f(r, "t0"), _f(r, "duration")
        i0, i1 = _idx_at(t, t0), _idx_at(t, t0 + dur)
        seg = slice(i0, i1 + 1)
        jd = i0 + int(np.argmax(gmax[seg]))
        if np.isfinite(_f(r, "max_g")):
            refined = _f(r, "max_g")
        else:
            refined = -_f(r, "min_clearance")  # common.replay_interval: min_clearance = -max_g
        # independent cross-check: 1 ms trapezoid of the sensed, delta-tightened density
        tr = {}
        solve = int(_f(r, "solve"))
        if c["masks"] is not None and solve < len(c["masks"]):
            m = c["masks"][solve]
            for dl in sorted({c["delta"], common.DELTA_OBS}):
                dens = np.sum(np.where(m, np.maximum(0.0, G[seg] + dl) ** 2, 0.0), axis=1)
                tr[dl] = float(np.trapezoid(dens, t[seg]))
        g0, g1 = float(gmax[i0]), float(gmax[i1])
        row = {
            "case": c["key"], "solve": solve, "k": int(_f(r, "k")), "t0": t0, "duration": dur,
            "node_gmax_start": g0, "node_gmax_end": g1,
            "dense_gmax_1ms": float(gmax[jd]), "dense_gmax_time": float(t[jd]),
            "dense_gmax_obstacle": int(c["gobs"][jd]),
            "refined_audited_max_g": float(refined),
            "intersample_rise_over_nodes": float(refined - max(g0, g1)),
            "audited_violation_duration": _f(r, "violation_duration"),
            "audited_max_physical_excess": _f(r, "max_physical_excess"),
            "audited_integral": _f(r, "integral"),
            "audited_integral_delta": c["delta"],
            "audited_integral_over_epsilon": _f(r, "integral") / c["epsilon"],
            "budget_met_csv": bool(_f(r, "budget_met", 0.0)),
            "nodes_physically_safe": bool(max(g0, g1) <= 0.0),
            "interior_physical_violation": bool(refined > 0.0),
        }
        row["intersample_only_violation"] = row["nodes_physically_safe"] and row["interior_physical_violation"]
        for dl, v in tr.items():
            row[f"trapz1ms_integral_delta{dl:g}"] = v
        out.append(row)
    return out


def _dense_duration(t, mask):
    """Measure of {mask} on the sample grid, midpoint rule on each grid step (estimate)."""
    mid = mask[:-1] & mask[1:]
    half = mask[:-1] ^ mask[1:]
    dt = np.diff(t)
    return float(np.sum(dt[mid]) + 0.5 * np.sum(dt[half]))


def case_stats(c, iv):
    s = c["summary"]
    cl = s.get("closed_loop", {})
    cs = cl.get("common_summarize", {})
    t, gmax = c["t"], c["gmax"]
    node_g = np.array([r["node_gmax_start"] for r in iv] + [iv[-1]["node_gmax_end"]])
    node_t = np.array([r["t0"] for r in iv] + [iv[-1]["t0"] + iv[-1]["duration"]])
    jn, jd = int(np.argmax(node_g)), int(np.argmax(gmax))
    ji = int(np.argmax([r["refined_audited_max_g"] for r in iv]))
    ratio = np.array([r["audited_integral_over_epsilon"] for r in iv])
    exec_budget = [_f(r, "executed_integral_max") for r in c["solves"]]
    exec_budget = [v for v in exec_budget if np.isfinite(v)]
    trap_key = f"trapz1ms_integral_delta{c['delta']:g}"
    trap_err = [abs(r[trap_key] - r["audited_integral"]) for r in iv if trap_key in r]
    st = {
        "case": c["key"], "label": c["spec"]["label"], "role": c["spec"]["role"],
        "nodal_only": c["spec"]["nodal"], "directory": str(c["dir"]),
        "summary_status": s.get("status"), "stage": s.get("stage"),
        "solves_completed": s.get("solves_completed"),
        "physical_execution_pass": s.get("physical_execution_pass"),
        "useful_tracking_pass": s.get("useful_tracking_pass"),
        "interval_budgets_met": cl.get("interval_budgets_met"),
        "solves_converged": s.get("solves_converged"), "solves_certified": s.get("solves_certified"),
        "epsilon": c["epsilon"], "delta_config": c["delta"],
        "simulated_seconds": float(t[-1] - t[0]), "executed_intervals": len(iv),
        # physical signed constraint max_i g_i (all five obstacles)
        "max_g_refined_audited": cs.get("max_g"), "max_g_refined_time": cs.get("max_g_time"),
        "max_g_refined_obstacle": cs.get("max_g_obstacle"),
        "max_g_refined_from_intervals": iv[ji]["refined_audited_max_g"],
        "max_g_dense_1ms": float(gmax[jd]), "max_g_dense_time": float(t[jd]),
        "max_g_dense_obstacle": int(c["gobs"][jd]),
        "max_g_nodes_0p5s": float(node_g[jn]), "max_g_nodes_time": float(node_t[jn]),
        "dense_minus_node_max_g": float(gmax[jd] - node_g[jn]),
        "max_physical_excess": cs.get("max_physical_excess"),
        "max_sum_excess_pointwise_sum": cs.get("max_sum_excess"),
        "physical_integral_all5": cs.get("physical_integral"),
        "violation_duration_audited_s": cl.get("violation_duration_total", cs.get("violation_duration")),
        "violation_duration_dense_estimate_s": _dense_duration(t, gmax > 0.0),
        "margin_intrusion_duration_dense_s": _dense_duration(t, (gmax > -common.DELTA_OBS) & (gmax <= 0.0)),
        "n_zero_crossings": cs.get("n_crossings"),
        "n_intervals_physical_violation": int(sum(r["interior_physical_violation"] for r in iv)),
        "n_intervals_intersample_only_violation": int(sum(r["intersample_only_violation"] for r in iv)),
        "n_nodes_physical_violation": int(np.sum(node_g > 0.0)),
        "n_nodes_violating_tightened": int(np.sum(node_g > -common.DELTA_OBS)),
        "min_clearance_shape": cs.get("min_clearance"),
        "min_geometric_clearance_m": cs.get("min_geometric_clearance"),
        # tracking
        "mean_tracking_error_audited_m": cs.get("mean_tracking_error"),
        "mean_tracking_error_dense_m": float(np.trapezoid(c["err"], t) / (t[-1] - t[0])),
        "max_tracking_error_dense_m": float(c["err"].max()),
        "tracking_cost_sum": cl.get("tracking_cost_sum"),
        "max_control_norm": cs.get("max_control_norm"),
        # CTCS integral (diagnostic for nodal-only cases: not their acceptance criterion)
        "max_interval_integral_audited": float(max(r["audited_integral"] for r in iv)),
        "max_interval_integral_over_epsilon": float(ratio.max()),
        "n_intervals_over_epsilon": int(np.sum(ratio > 1.0)),
        "executed_integral_max_from_solves": max(exec_budget) if exec_budget else None,
        "trapz1ms_vs_audited_integral_max_abs_diff": max(trap_err) if trap_err else None,
        # data-handling checks
        "dense_junction_duplicates_dropped": c["n_junction_duplicates"],
        "dense_junction_state_mismatch": c["junction_state_mismatch"],
        "dense_g_vs_common_obstacle_g_max_abs": c["g_model_check"],
        "saved_reference_vs_common_reference_max_abs": c["reference_check"],
    }
    flags = []
    flags.append("physically safe (audited, all 5 obstacles)" if (st["max_physical_excess"] or 0.0) <= 0.0
                 else f"PHYSICAL VIOLATION: excess {st['max_physical_excess']:.3e}, "
                      f"{st['violation_duration_audited_s']:.4f} s")
    flags.append(("diagnostic integral: " if st["nodal_only"] else "") + ("integral budget met" if st["n_intervals_over_epsilon"] == 0
                 else f"integral budget NOT met in {st['n_intervals_over_epsilon']} intervals "
                      f"(max I/eps={st['max_interval_integral_over_epsilon']:.3g})"))
    flags.append(f"run status={st['summary_status']}")
    st["verdict"] = "; ".join(flags)
    return st


PERF_DEFINITIONS = {
    "solve_time_s (SCP)": "solves.csv solve_seconds: Python perf_counter around problem.reset() + "
                          "problem.solve(max_iters=J) in scp.py (Python + JAX + convex subproblem "
                          "solver, fixed J iterations). Excludes problem.initialize() (JAX tracing / "
                          "compilation, reported as initialize_seconds) and all audits/replays.",
    "solve_time_s (DDP)": "solves.csv solver_seconds: the C++ ALIPDDP solver's own timer (diag.json "
                          "solver.seconds), iterated to tolerance. Excludes process/IPC/file I/O and "
                          "all audits/replays.",
    "wall_time_s (DDP)": "solves.csv wall_seconds: Python perf_counter around the C++ call (process "
                         "spawn or persistent-session IPC, CSV/JSON I/O); excludes audits.",
    "first / warm": "solve 0 (cold seed, plus any residual first-call overhead) is reported alone; "
                    "warm_* statistics exclude solve 0. p95 is numpy linear interpolation over few "
                    "solves (one per executed MPC step), not a tail estimate.",
    "comparability": "SCP (Python/JAX, fixed iteration count) and DDP (C++, converge-to-tolerance) "
                     "times come from different implementations, languages and stopping rules in "
                     "single non-dedicated runs; they show per-method cost scale only and do NOT "
                     "support a speed ranking of the algorithms.",
}


def _dist(v, prefix):
    v = np.asarray(v, float)
    if v.size == 0:
        return {f"{prefix}_{k}": None for k in ("n", "median", "p95", "max", "mean")}
    return {f"{prefix}_n": int(v.size), f"{prefix}_median": float(np.median(v)),
            f"{prefix}_p95": float(np.percentile(v, 95)), f"{prefix}_max": float(v.max()),
            f"{prefix}_mean": float(v.mean())}


def perf_stats(c):
    """Solver timing / iterations from saved solves.csv (audits never included)."""
    rows = c["solves"]
    if rows and "solve_seconds" in rows[0]:
        solver, tkey, ikey = "SCP (Python/JAX, openscvx)", "solve_seconds", "iterations"
        note = PERF_DEFINITIONS["solve_time_s (SCP)"]
    elif rows and "solver_seconds" in rows[0]:
        solver, tkey, ikey = "DDP (C++, ALIPDDP)", "solver_seconds", "solver_iterations"
        note = PERF_DEFINITIONS["solve_time_s (DDP)"]
    else:
        return {"perf_solver": None, "perf_timing_source": "solves.csv missing or without timing"}, None
    ok = [r for r in rows if np.isfinite(_f(r, tkey))]
    t = np.array([_f(r, tkey) for r in ok])
    it = np.array([_f(r, ikey) for r in ok])
    wall = np.array([_f(r, "wall_seconds") for r in ok])
    p = {"perf_solver": solver, "perf_timing_source": f"solves.csv:{tkey}", "perf_timing_note": note,
         "solve_time_first_s": float(t[0]) if t.size else None}
    p.update(_dist(t, "solve_time_all_s"))
    p.update(_dist(t[1:], "solve_time_warm_s"))
    p["solve_time_total_s"] = float(t.sum()) if t.size else None
    init = c["summary"].get("initialize_seconds")
    p["initialize_seconds_excluded"] = init  # SCP JAX setup/compile; None for DDP
    if np.isfinite(wall).any():
        p["wall_time_first_s"] = float(wall[0])
        p.update(_dist(wall[1:], "wall_time_warm_s"))
    p["iterations_first"] = int(it[0]) if it.size and np.isfinite(it[0]) else None
    p.update(_dist(it, "iterations_all"))
    p.update(_dist(it[1:], "iterations_warm"))
    p["iterations_total"] = int(np.nansum(it)) if it.size else None
    series = dict(solve=np.array([int(_f(r, "solve")) for r in ok]), t=t, wall=wall, it=it)
    return p, series


# ---------------------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------------------
def _style():
    plt.rcParams.update({
        "font.size": 8, "axes.labelsize": 8, "axes.titlesize": 8.5, "legend.fontsize": 6.8,
        "xtick.labelsize": 7, "ytick.labelsize": 7, "lines.linewidth": 1.1,
        "axes.grid": True, "grid.alpha": 0.25, "grid.linewidth": 0.5,
        "pdf.fonttype": 42, "ps.fonttype": 42, "savefig.bbox": "tight", "savefig.dpi": 300,
        "mathtext.fontset": "dejavusans",
    })


def _ellipses(ax, only=None, label=True):
    semi = 1.0 / common.SHAPE_DIAG
    for i, cen in enumerate(common.CENTERS):
        if only is not None and i != only:
            continue
        ax.add_patch(Ellipse(cen, 2 * semi[0], 2 * semi[1], fc="0.80", ec="0.25", lw=0.8, zorder=1))
        s = 1.0 + common.DELTA_OBS
        ax.add_patch(Ellipse(cen, 2 * s * semi[0], 2 * s * semi[1], fc="none", ec="0.25", lw=0.7,
                             ls=(0, (3, 2)), zorder=1))
        if label:
            ax.annotate(f"$O_{i + 1}$", cen, ha="center", va="center", fontsize=6.5, zorder=2)


def _violation_overlay(ax, x, y, mask, **kw):
    if not np.any(mask):
        return
    yy = np.where(mask, y, np.nan)
    ax.plot(x, yy, **kw)


def _save(fig, out, stem):
    paths = [out / f"{stem}.png", out / f"{stem}.pdf"]
    fig.savefig(paths[0])
    fig.savefig(paths[1])
    plt.close(fig)
    return paths


def fig_comparison(cases, ivs, out, glow):
    fig, axs = plt.subplots(2, 2, figsize=(7.2, 5.9), constrained_layout=True)
    (ax_xy, ax_g), (ax_e, ax_i) = axs
    _ellipses(ax_xy)
    tr = np.linspace(0.0, common.PAPER_VALUES["duration"], 3001)
    rr = common.reference(tr)
    ax_xy.plot(rr[:, 0], rr[:, 1], color="k", lw=0.9, ls=(0, (1.5, 1.5)), zorder=2)
    for c in cases:
        sp, iv = c["spec"], ivs[c["key"]]
        nt = np.array([r["t0"] for r in iv] + [iv[-1]["t0"] + iv[-1]["duration"]])
        ni = [_idx_at(c["t"], x) for x in nt]
        viol = c["gmax"] > 0.0
        ax_xy.plot(c["X"][:, 0], c["X"][:, 1], color=sp["color"], ls=sp["ls"], zorder=3)
        ax_xy.plot(c["X"][ni, 0], c["X"][ni, 1], "o", ms=2.0, color=sp["color"], zorder=4)
        _violation_overlay(ax_xy, c["X"][:, 0], c["X"][:, 1], viol, color="#C00000", lw=2.6, zorder=5)

        ax_g.plot(c["t"], np.maximum(c["gmax"], glow - 1), color=sp["color"], ls=sp["ls"], zorder=3)
        ax_g.plot(nt, c["gmax"][ni], "o", ms=2.6, mfc="white", mec=sp["color"], mew=0.8, zorder=4)
        ax_g.fill_between(c["t"], 0.0, c["gmax"], where=viol, color="#C00000", alpha=0.45, lw=0,
                          zorder=2)

        ax_e.plot(c["t"], c["err"], color=sp["color"], ls=sp["ls"])

        edges = np.array([r["t0"] for r in iv] + [iv[-1]["t0"] + iv[-1]["duration"]])
        ratio = np.array([r["audited_integral_over_epsilon"] for r in iv])
        ax_i.stairs(ratio, edges, color=sp["color"], ls=sp["ls"], lw=1.0)

    ax_xy.set(xlabel="$x$ [m]", ylabel="$y$ [m]", title="(a) Executed trajectories")
    ax_xy.set_aspect("equal", adjustable="datalim")
    ax_xy.autoscale_view()

    ax_g.axhline(0.0, color="k", lw=0.9)
    ax_g.axhline(-common.DELTA_OBS, color="0.3", lw=0.8, ls=(0, (3, 2)))
    ax_g.axhspan(-common.DELTA_OBS, 0.0, color="#FFD27F", alpha=0.35, lw=0, zorder=0)
    gtop = max(0.05, max(float(c["gmax"].max()) for c in cases) * 1.15 + 0.02)
    ax_g.set_ylim(glow, gtop)
    ax_g.set(xlabel="$t$ [s]", ylabel=r"$\max_i\, g_i(t)$  (all 5 obstacles)",
             title=rf"(b) Physical constraint, executed plant (clipped at {glow:g})")
    ax_g.text(0.3, 0.004, r"$g>0$: inside obstacle", va="bottom", fontsize=6.5, color="#C00000")

    ax_e.set(xlabel="$t$ [s]", ylabel=r"$\|r(t)-r_{\rm ref}(t)\|$ [m]", title="(c) Tracking error")
    ax_e.set_ylim(bottom=0.0)

    ax_i.axhline(1.0, color="k", lw=0.9)
    ax_i.set_yscale("symlog", linthresh=1e-3, linscale=0.6)
    ax_i.set_ylim(0.0, max(3.0, 2.5 * max(r["audited_integral_over_epsilon"]
                                          for c in cases for r in ivs[c["key"]])))
    ax_i.set(xlabel="$t$ [s]", ylabel=r"$I_k/\varepsilon$, executed interval",
             title=r"(d) Audited CTCS integral per interval (diagnostic)")
    ax_i.text(0.01, 0.45, r"$I_k=\int\sum_{\rm sensed}[g_i+\delta]_+^2\,dt$, $\varepsilon=10^{-6}$;"
              "\nnot the acceptance test of nodal-only cases", transform=ax_i.transAxes,
              va="center", fontsize=6.2, bbox=dict(fc="white", ec="none", alpha=0.85))

    handles = []
    for c in cases:
        sp = c["spec"]
        lab = sp["label"]
        if c["delta"] != common.DELTA_OBS:
            lab += rf" [$I_k$ at $\delta$={c['delta']:g}]"
        handles.append(Line2D([], [], color=sp["color"], ls=sp["ls"], label=lab))
    handles += [
        Line2D([], [], color="k", lw=0.9, ls=(0, (1.5, 1.5)), label=r"reference $r_{\rm ref}(t)$"),
        Patch(fc="0.80", ec="0.25", label=r"obstacle $\|S(r-c_i)\|=1$"),
        Line2D([], [], color="0.25", lw=0.7, ls=(0, (3, 2)), label=r"tightened $\|S(r-c_i)\|=1.05$ / $g=-0.05$"),
        Line2D([], [], marker="o", ls="none", ms=3, mfc="white", mec="0.2", label="executed nodes (0.5 s)"),
        Patch(fc="#FFD27F", alpha=0.6, label="inside margin, physically safe"),
        Patch(fc="#C00000", alpha=0.6, label=r"physical violation $g>0$"),
    ]
    fig.legend(handles=handles, loc="outside lower center", ncol=3, frameon=False)
    return _save(fig, out, "fig_comparison")


def _node_times(iv):
    return np.array([r["t0"] for r in iv] + [iv[-1]["t0"] + iv[-1]["duration"]])


def fig_2x2(by_key, ivs, stats, out, glow):
    """Rows: SCP, DDP; columns: CTCS, nodal only.  Identical xy limits and g axis in all panels."""
    cases = [by_key[k] for _, k, _ in PANELS]
    semi = (1.0 + common.DELTA_OBS) / common.SHAPE_DIAG
    tr = np.linspace(0.0, common.PAPER_VALUES["duration"], 3001)
    rr = common.reference(tr)
    xs = np.concatenate([rr[:, 0]] + [c["X"][:, 0] for c in cases]
                        + [common.CENTERS[:, 0] - semi[0], common.CENTERS[:, 0] + semi[0]])
    ys = np.concatenate([rr[:, 1]] + [c["X"][:, 1] for c in cases]
                        + [common.CENTERS[:, 1] - semi[1], common.CENTERS[:, 1] + semi[1]])
    xlim, ylim = (xs.min() - 0.3, xs.max() + 0.3), (ys.min() - 0.3, ys.max() + 0.3)
    gtop = max(0.05, max(float(c["gmax"].max()) for c in cases) * 1.15 + 0.02)
    tmax = max(float(c["t"][-1]) for c in cases)

    fig = plt.figure(figsize=(7.2, 7.4), layout="constrained")
    subs = fig.subfigures(2, 2, wspace=0.02, hspace=0.02)
    for n, ((name, key, title), sf) in enumerate(zip(PANELS, subs.ravel())):
        c, iv, st = by_key[key], ivs[key], stats[key]
        sp = c["spec"]
        ax_xy, ax_g = sf.subplots(2, 1, gridspec_kw={"height_ratios": [2.3, 1.0]})
        nt = _node_times(iv)
        ni = [_idx_at(c["t"], x) for x in nt]
        viol = c["gmax"] > 0.0

        _ellipses(ax_xy)
        ax_xy.plot(rr[:, 0], rr[:, 1], color="k", lw=0.9, ls=(0, (1.5, 1.5)), zorder=2)
        ax_xy.plot(c["X"][:, 0], c["X"][:, 1], color=sp["color"], ls=sp["ls"], zorder=3)
        ax_xy.plot(c["X"][ni, 0], c["X"][ni, 1], "o", ms=2.6, mfc="white", mec=sp["color"], mew=0.7,
                   zorder=4)
        _violation_overlay(ax_xy, c["X"][:, 0], c["X"][:, 1], viol, color="#C00000", lw=2.6, zorder=5)
        ax_xy.set_xlim(*xlim)
        ax_xy.set_ylim(*ylim)
        ax_xy.set_aspect("equal", adjustable="box")
        ax_xy.set(xlabel="$x$ [m]", ylabel="$y$ [m]", title=f"({chr(97 + n)}) {title}")

        mpe = st["max_physical_excess"] or 0.0
        txt = (rf"max $g$: refined {st['max_g_refined_audited']:+.4f}" "\n"
               rf"  dense 1 ms {st['max_g_dense_1ms']:+.4f}, nodes {st['max_g_nodes_0p5s']:+.4f}" "\n"
               + (f"VIOLATION {st['violation_duration_audited_s']:.3f} s, "
                  f"{st['n_intervals_physical_violation']} int. "
                  f"({st['n_intervals_intersample_only_violation']} intersample-only)"
                  if mpe > 0.0 else r"no physical violation ($\max g\leq 0$)")
               + f"\nmean tracking err. {st['mean_tracking_error_audited_m']:.3f} m")
        ax_xy.text(0.02, 0.02, txt, transform=ax_xy.transAxes, fontsize=5.8, va="bottom", ha="left",
                   color="#C00000" if mpe > 0.0 else "k", linespacing=1.3,
                   bbox=dict(fc="white", ec="0.7", lw=0.4, alpha=0.9), zorder=6)

        ax_g.axhspan(-common.DELTA_OBS, 0.0, color="#FFD27F", alpha=0.35, lw=0, zorder=0)
        ax_g.axhline(0.0, color="k", lw=0.9)
        ax_g.axhline(-common.DELTA_OBS, color="0.3", lw=0.8, ls=(0, (3, 2)))
        ax_g.plot(c["t"], np.maximum(c["gmax"], glow - 1), color=sp["color"], ls=sp["ls"], zorder=3)
        ax_g.plot(nt, c["gmax"][ni], "o", ms=2.4, mfc="white", mec=sp["color"], mew=0.7, zorder=4)
        ax_g.fill_between(c["t"], 0.0, c["gmax"], where=viol, color="#C00000", alpha=0.45, lw=0, zorder=2)
        ax_g.set_ylim(glow, gtop)
        ax_g.set_xlim(0.0, tmax)
        ax_g.set(xlabel="$t$ [s]", ylabel=r"$\max_i g_i(t)$")

    handles = [
        Line2D([], [], color="k", lw=0.9, ls=(0, (1.5, 1.5)), label=r"reference $r_{\rm ref}(t)$"),
        Patch(fc="0.80", ec="0.25", label=r"obstacle ($g=0$)"),
        Line2D([], [], color="0.25", lw=0.7, ls=(0, (3, 2)), label=r"tightened $g=-0.05$"),
        Line2D([], [], marker="o", ls="none", ms=3, mfc="white", mec="0.2", label="executed nodes (0.5 s)"),
        Patch(fc="#FFD27F", alpha=0.6, label="inside margin, physically safe"),
        Patch(fc="#C00000", alpha=0.6, label=r"physical violation $g>0$ (dense 1 ms)"),
    ]
    fig.legend(handles=handles, loc="outside lower center", ncol=3, frameon=False,
               title=rf"All five obstacles; $g$ axis clipped below {glow:g}; refined peaks from DOP853 audit",
               title_fontsize=6.2)
    return _save(fig, out, "fig_2x2")


def fig_performance(cases, stats, series, out):
    fig, axs = plt.subplots(1, 3, figsize=(7.2, 2.7), constrained_layout=True,
                            gridspec_kw={"width_ratios": [1.0, 1.0, 1.5]})
    ax_e, ax_g, ax_t = axs
    names = [{"SCP10ZOH": "SCP CTCS", "SCPNodal": "SCP nodal",
              "DDPCTCS": "DDP CTCS", "DDPNodal": "DDP nodal"}.get(c["key"], c["key"]) for c in cases]
    cols = [c["spec"]["color"] for c in cases]
    x = np.arange(len(cases))
    track = [stats[c["key"]]["mean_tracking_error_audited_m"] for c in cases]
    gref = [stats[c["key"]]["max_g_refined_audited"] for c in cases]
    gref = [np.nan if v is None else v for v in gref]
    ax_e.bar(x, track, color=cols)
    ax_e.set(ylabel="mean tracking error [m]", title="(a) Tracking")
    ax_g.bar(x, gref, color=cols)
    ax_g.axhline(0.0, color="k", lw=0.9)
    ax_g.axhline(-common.DELTA_OBS, color="0.3", lw=0.8, ls=(0, (3, 2)))
    for xi, v in zip(x, gref):
        if np.isfinite(v):
            ax_g.annotate(f"{v:+.3f}", (xi, v), ha="center", va="bottom" if v > 0 else "top",
                          fontsize=5.5, color="#C00000" if v > 0 else "k")
    ax_g.set(ylabel=r"physical max $g$ (refined audit)", title=r"(b) Physical max $g$ ($>0$: violation)")
    for ax in (ax_e, ax_g):
        ax.set_xticks(x, names, rotation=35, ha="right")
    for c in cases:
        s = series.get(c["key"])
        if s is None or s["t"].size == 0:
            continue
        sp = c["spec"]
        ax_t.plot(s["solve"], s["t"], marker="o", ms=2.5, color=sp["color"], ls=sp["ls"],
                  label=f"{c['key']} ({stats[c['key']]['perf_timing_source'].split(':')[1]})")
        ax_t.plot(s["solve"][:1], s["t"][:1], marker="s", ms=5, mfc="none", mec=sp["color"], ls="none")
    ax_t.set_yscale("log")
    ax_t.set(xlabel="solve index (0 = first, cold)", ylabel="solver time per solve [s]",
             title="(c) Solver time (audits excluded)")
    ax_t.legend(fontsize=5.5, frameon=False)
    ax_t.text(0.01, 0.01, "SCP: Python/JAX, fixed iterations, init/JIT excluded\n"
              "DDP: C++ solver timer, to tolerance\ndifferent implementations: no speed ranking",
              transform=ax_t.transAxes, fontsize=5.5, va="bottom",
              bbox=dict(fc="white", ec="none", alpha=0.85))
    return _save(fig, out, "fig_performance")


def _closeup_target(c, iv):
    j = int(np.argmax([r["refined_audited_max_g"] for r in iv]))
    return j, iv[j]


def fig_intersample(nodal, others, ivs, out):
    """One row per nodal-only case: dense g(t) around its worst interval + planar zoom."""
    rows_out = []
    nrow = len(nodal)
    fig, axs = plt.subplots(nrow, 3, figsize=(7.2, 2.6 * nrow), constrained_layout=True,
                            squeeze=False, gridspec_kw={"width_ratios": [1.45, 1.0, 0.62]})
    info = []
    for row, c in enumerate(nodal):
        ax_g, ax_xy, ax_t = axs[row]
        ax_t.axis("off")
        sp, iv = c["spec"], ivs[c["key"]]
        j, r = _closeup_target(c, iv)
        t0, t1 = r["t0"], r["t0"] + r["duration"]
        w0, w1 = max(c["t"][0], t0 - NODE_DT), min(c["t"][-1], t1 + NODE_DT)
        sel = (c["t"] >= w0 - 1e-9) & (c["t"] <= w1 + 1e-9)
        tw, gw = c["t"][sel], c["gmax"][sel]
        obs = r["dense_gmax_obstacle"]
        for o in others:
            so = (o["t"] >= w0 - 1e-9) & (o["t"] <= w1 + 1e-9)
            ax_g.plot(o["t"][so], o["gmax"][so], color=o["spec"]["color"], lw=0.8, alpha=0.55,
                      ls=o["spec"]["ls"])
        ax_g.axvspan(t0, t1, color="0.92", lw=0, zorder=0)
        ax_g.axhline(0.0, color="k", lw=0.9)
        ax_g.axhline(-common.DELTA_OBS, color="0.3", lw=0.8, ls=(0, (3, 2)))
        ax_g.plot(tw, gw, color=sp["color"], ls=sp["ls"], lw=1.4, zorder=3)
        ax_g.fill_between(tw, 0.0, gw, where=gw > 0.0, color="#C00000", alpha=0.45, lw=0, zorder=2)
        nts = np.arange(np.ceil((w0 - 1e-9) / NODE_DT), np.floor((w1 + 1e-9) / NODE_DT) + 1) * NODE_DT
        nidx = [_idx_at(c["t"], x) for x in nts]
        ax_g.plot(c["t"][nidx], c["gmax"][nidx], "o", ms=5, mfc="white", mec=sp["color"], mew=1.3,
                  zorder=5)
        ax_g.plot([r["dense_gmax_time"]], [r["refined_audited_max_g"]], "*", ms=9, color="#C00000"
                  if r["refined_audited_max_g"] > 0 else "0.2", mec="k", mew=0.4, zorder=6)
        viol = r["refined_audited_max_g"] > 0.0
        txt = (f"worst interval: solve {r['solve']}, k={r['k']}\n"
               f"t = [{t0:.1f}, {t1:.1f}] s, obstacle $O_{obs + 1}$\n"
               f"node g: {r['node_gmax_start']:+.4f} / {r['node_gmax_end']:+.4f}\n"
               f"dense max g (1 ms): {r['dense_gmax_1ms']:+.5f}\n"
               f"  at t = {r['dense_gmax_time']:.3f} s\n"
               f"refined audited max g: {r['refined_audited_max_g']:+.5f}\n"
               f"rise over nodes: {r['intersample_rise_over_nodes']:+.5f}\n"
               f"violation duration (audited):\n  {r['audited_violation_duration']:.4f} s\n"
               + ("INTERSAMPLE-ONLY VIOLATION\n(nodes physically safe)" if r["intersample_only_violation"]
                  else "physical violation (incl. nodes)" if viol
                  else "no physical violation\nobserved in this run"))
        ax_t.text(0.0, 0.5, txt, transform=ax_t.transAxes, fontsize=6.2, va="center", ha="left",
                  linespacing=1.45)
        lo = min(-common.DELTA_OBS, float(gw.min()))
        hi = max(0.0, float(gw.max()))
        pad = 0.12 * (hi - lo) + 1e-3
        ax_g.set_ylim(max(lo - pad, -0.25), hi + pad)
        ax_g.set_xlim(w0, w1)
        ax_g.set(xlabel="$t$ [s]", ylabel=r"$\max_i\, g_i(t)$",
                 title=f"({chr(97 + 2 * row)}) {sp['short']}: dense $g$ vs. 0.5 s nodes")

        # planar zoom
        _ellipses(ax_xy, only=obs, label=False)
        Xw = c["X"][sel]
        for o in others:
            so = (o["t"] >= w0 - 1e-9) & (o["t"] <= w1 + 1e-9)
            ax_xy.plot(o["X"][so, 0], o["X"][so, 1], color=o["spec"]["color"], lw=0.8, alpha=0.55,
                       ls=o["spec"]["ls"])
        ax_xy.plot(Xw[:, 0], Xw[:, 1], color=sp["color"], ls=sp["ls"], lw=1.4, zorder=3)
        _violation_overlay(ax_xy, Xw[:, 0], Xw[:, 1], gw > 0.0, color="#C00000", lw=2.8, zorder=4)
        ax_xy.plot(c["X"][nidx, 0], c["X"][nidx, 1], "o", ms=5, mfc="white", mec=sp["color"],
                   mew=1.3, zorder=5)
        jp = _idx_at(c["t"], r["dense_gmax_time"])
        ax_xy.plot(c["X"][jp, 0], c["X"][jp, 1], "*", ms=9, color="#C00000" if viol else "0.2",
                   mec="k", mew=0.4, zorder=6)
        rw = common.reference(tw)
        ax_xy.plot(rw[:, 0], rw[:, 1], color="k", lw=0.8, ls=(0, (1.5, 1.5)), zorder=2)
        sx = 1.15 * (1 + common.DELTA_OBS) / common.SHAPE_DIAG
        cen = common.CENTERS[obs]
        xs = np.concatenate([Xw[:, 0], [cen[0] - sx[0], cen[0] + sx[0]]])
        ys = np.concatenate([Xw[:, 1], [Xw[:, 1].min()]])
        ax_xy.set_xlim(xs.min() - 0.2, xs.max() + 0.2)
        ax_xy.set_ylim(ys.min() - 0.3, max(Xw[:, 1].max(), cen[1]) + 0.3)
        ax_xy.set_aspect("equal", adjustable="box")
        ax_xy.set(xlabel="$x$ [m]", ylabel="$y$ [m]", title=f"({chr(98 + 2 * row)}) planar close-up, $O_{obs + 1}$")

        info.append(dict(r, interval_index=j, window=[w0, w1]))
        for tt, gg, xx, gi in zip(tw, gw, Xw, c["G"][sel]):
            rows_out.append({"case": c["key"], "t": tt, "x": xx[0], "y": xx[1], "gmax": gg,
                             **{f"g{i + 1}": v for i, v in enumerate(gi)},
                             "is_node": bool(abs(tt / NODE_DT - round(tt / NODE_DT)) < 1e-6),
                             "in_worst_interval": bool(t0 - 1e-9 <= tt <= t1 + 1e-9)})
    handles = [Line2D([], [], color=c["spec"]["color"], ls=c["spec"]["ls"], lw=1.4,
                      label=c["spec"]["label"]) for c in nodal]
    handles += [Line2D([], [], color=o["spec"]["color"], ls=o["spec"]["ls"], lw=0.8, alpha=0.55,
                       label=o["spec"]["label"] + " (same window)") for o in others]
    handles += [
        Line2D([], [], marker="o", ls="none", ms=5, mfc="white", mec="0.2", label="0.5 s nodes"),
        Line2D([], [], marker="*", ls="none", ms=8, color="0.4", mec="k", label="refined interval peak"),
        Line2D([], [], color="k", lw=0.9, label="$g=0$ (physical boundary)"),
        Line2D([], [], color="0.3", lw=0.8, ls=(0, (3, 2)), label=r"$g=-0.05$ (tightened)"),
        Patch(fc="#C00000", alpha=0.6, label="$g>0$ (violation)"),
        Patch(fc="0.92", label="worst executed interval"),
    ]
    fig.legend(handles=handles, loc="outside lower center", ncol=3, frameon=False)
    return _save(fig, out, "fig_intersample"), rows_out, info


# ---------------------------------------------------------------------------
def _write_csv(path, rows):
    keys = []
    for r in rows:
        for k in r:
            if k not in keys:
                keys.append(k)
    with open(path, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=keys)
        w.writeheader()
        for r in rows:
            w.writerow({k: (repr(v) if isinstance(v, float) else v) for k, v in r.items()})


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("--logs-root", type=Path, default=DEFAULT_LOGS)
    p.add_argument("--output-dir", type=Path, required=True)
    p.add_argument("--cases", nargs="+", default=DEFAULT_CASES, choices=list(CASES),
                   help="case keys to load (default: all). Supplementary cases are statistics only.")
    p.add_argument("--case-dir", action="append", default=[], metavar="KEY=DIR",
                   help="override a case directory (relative to --logs-root or absolute)")
    p.add_argument("--allow-missing", action="store_true",
                   help="skip requested cases whose outputs are missing instead of failing")
    p.add_argument("--g-floor", type=float, default=-0.3,
                   help="lower y-limit of the full-run max_i g_i(t) panel (values below are clipped)")
    a = p.parse_args(argv)

    specs = {k: dict(v) for k, v in CASES.items()}
    for item in a.case_dir:
        k, _, d = item.partition("=")
        if k not in specs or not d:
            p.error(f"--case-dir expects KEY=DIR with KEY in {list(specs)}, got {item!r}")
        specs[k]["dir"] = d

    cases, missing = [], []
    for k in a.cases:
        cdir = Path(specs[k]["dir"])
        cdir = cdir if cdir.is_absolute() else a.logs_root / cdir
        lack = [f.name for f in _required(cdir) if not f.exists()]
        if lack:
            missing.append((k, f"{cdir}: {', '.join(lack)}"))
            continue
        cases.append(load_case(k, specs[k], cdir))
    if missing:
        msg = "\n".join(f"  {k}: missing {l}" for k, l in missing)
        if not a.allow_missing:
            sys.exit(f"error: required outputs missing for {len(missing)} case(s):\n{msg}\n"
                     "Run those scenarios first, restrict --cases, or pass --allow-missing.")
        print(f"warning: skipping missing case(s):\n{msg}", file=sys.stderr)
    plotted = [c for c in cases if c["spec"]["role"] in ("main", "separate")]
    if not plotted:
        sys.exit("error: no main/separate case available to plot")

    a.output_dir.mkdir(parents=True, exist_ok=True)
    _style()
    ivs = {c["key"]: interval_table(c) for c in cases}
    stats, perf_series = [], {}
    for c in cases:
        st = case_stats(c, ivs[c["key"]])
        perf, perf_series[c["key"]] = perf_stats(c)
        st.update(perf)
        stats.append(st)
    by_stat = {s["case"]: s for s in stats}

    files = fig_comparison(plotted, ivs, a.output_dir, a.g_floor)
    by_key = {c["key"]: c for c in cases}
    absent = [k for _, k, _ in PANELS if k not in by_key]
    if absent:
        print(f"note: fig_2x2 not produced, missing case(s): {', '.join(absent)}", file=sys.stderr)
    else:
        files += fig_2x2(by_key, ivs, by_stat, a.output_dir, a.g_floor)
    files += fig_performance([c for c in plotted if c["spec"]["role"] == "main"], by_stat, perf_series, a.output_dir)
    nodal = [c for c in plotted if c["spec"]["nodal"]]
    closeup_info = []
    if nodal:
        others = [c for c in plotted if not c["spec"]["nodal"]]
        f2, crow, closeup_info = fig_intersample(nodal, others, ivs, a.output_dir)
        files += f2
        _write_csv(a.output_dir / "closeup_series.csv", crow)
        files.append(a.output_dir / "closeup_series.csv")
    else:
        print("note: no nodal-only case available; fig_intersample not produced", file=sys.stderr)

    series = []
    nodes = []
    for c in plotted:
        for tt, xx, rr, gg, go, ee in zip(c["t"], c["X"], c["ref"], c["gmax"], c["gobs"], c["err"]):
            series.append({"case": c["key"], "t": tt, "x": xx[0], "y": xx[1], "x_ref": rr[0],
                           "y_ref": rr[1], "gmax": gg, "gmax_obstacle": int(go) + 1,
                           "tracking_error": ee})
        for r in ivs[c["key"]]:
            nodes.append({"case": c["key"], "t": r["t0"], "gmax": r["node_gmax_start"]})
        last = ivs[c["key"]][-1]
        nodes.append({"case": c["key"], "t": last["t0"] + last["duration"], "gmax": last["node_gmax_end"]})
    _write_csv(a.output_dir / "plotted_series.csv", series)
    _write_csv(a.output_dir / "plotted_nodes.csv", nodes)
    _write_csv(a.output_dir / "executed_intervals_all.csv", [r for c in cases for r in ivs[c["key"]]])
    _write_csv(a.output_dir / "comparison_stats.csv", stats)
    common.save_json(a.output_dir / "comparison_stats.json", {
        "definitions": {
            "g": "signed physical residual max_i g_i, g_i = 1 - ||S(r - c_i)||_2 over all 5 obstacles "
                 "(common.obstacle_g); >0 inside an obstacle",
            "max_g_refined_audited": "DOP853 audit with refined intersample peaks (summary.json "
                                     "closed_loop.common_summarize.max_g)",
            "max_g_dense_1ms": "max over the saved 1 ms dense executed replay",
            "max_g_nodes_0p5s": "max over executed interval endpoints (0.5 s grid)",
            "max_sum_excess_pointwise_sum": "max_t sum_i max(0, g_i) (paper Fig. 4 metric), NOT max_i g_i",
            "violation_duration_audited_s": "event-resolved measure of {max_i g_i > 0}",
            "violation_duration_dense_estimate_s": "same set, 1 ms grid estimate",
            "margin_intrusion_duration_dense_s": "1 ms grid estimate of {-0.05 < max_i g_i <= 0}",
            "audited_integral": "per executed interval int sum_sensed [g_i + delta]_+^2 dt with the run's "
                                "config delta (diagnostic for nodal-only cases, not their acceptance)",
            "dense junctions": "duplicated interval-junction samples dropped (first kept, as "
                               "common.concat_dense)",
        },
        "performance_definitions": PERF_DEFINITIONS,
        "roles": {"main": "matched comparison, delta=0.05 tightening",
                  "separate": "plotted, NOT matched (margin 0)",
                  "supplementary": "statistics only (FOH SCP runs)"},
        "panels_2x2": {name: key for name, key, _ in PANELS},
        "logs_root": a.logs_root, "missing_cases": [k for k, _ in missing],
        "cases": stats, "closeups": closeup_info,
    })
    files += [a.output_dir / n for n in ("comparison_stats.json", "comparison_stats.csv",
                                         "executed_intervals_all.csv", "plotted_series.csv",
                                         "plotted_nodes.csv")]
    for s in stats:
        print(f"{s['case']:10s} max g refined={s['max_g_refined_audited']:+.6f} "
              f"dense={s['max_g_dense_1ms']:+.6f} nodes={s['max_g_nodes_0p5s']:+.6f} "
              f"viol={s['violation_duration_audited_s']:.4f}s "
              f"track={s['mean_tracking_error_audited_m']:.4f}m  {s['verdict']}")
        if s.get("solve_time_first_s") is not None:
            print(f"{'':10s} {s['perf_timing_source']}: first={s['solve_time_first_s']:.3f}s "
                  f"warm median={s['solve_time_warm_s_median']}s p95={s['solve_time_warm_s_p95']}s "
                  f"max={s['solve_time_warm_s_max']}s it(first/warm median)="
                  f"{s['iterations_first']}/{s['iterations_warm_median']}")
    for f in files:
        print(f)
    return 0


if __name__ == "__main__":
    sys.exit(main())
