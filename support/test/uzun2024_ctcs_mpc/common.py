"""Shared, solver-independent model and audit for the Uzun et al. (2024) CTCS NMPC benchmark.

Paper: S. Uzun et al., "Successive Convexification for Nonlinear Model Predictive
Control with Continuous-Time Constraint Satisfaction", IFAC PapersOnLine 58-18 (2024)
421-429, Sec. 4 and Table 1 (local PDF, see PAPER_SOURCE).

Everything here is independent of OpenSCvx and ALIPDDP: dynamics, reference,
obstacles, sensing, and a high-accuracy scipy replay that audits a held control
against the *physical* (untightened, all five obstacles) constraint on a 1 ms
grid with exact interval end, zero-crossing events and refined peaks.

Two constraint quantities are kept strictly apart:

* ``obstacle_g``   -- paper shape residual g_i = 1 - ||S_i (r - c_i)||_2 (primary, paper).
                      g_i > 0 means inside obstacle i.  Dimensionless.
* ``ellipse_signed_distance`` -- Euclidean signed distance [m] to the ellipse
                      boundary {||S_i (r - c_i)||_2 = 1} (negative inside).  Secondary,
                      reported only so geometric penetration is not confused with g.

The CTCS density integrated by the solvers is sum_{i in mask} max(0, g_i + delta)^2
(delta-tightened, sensed obstacles only).  The physical audit uses max(0, g_i)
over all five obstacles regardless of the mask.
"""

from __future__ import annotations

import csv
import datetime as _dt
import hashlib
import json
import os
import platform
import subprocess
import sys
from pathlib import Path

import numpy as np
from scipy.integrate import solve_ivp
from scipy.optimize import minimize_scalar

PAPER_SOURCE = (
    "/home/lab/Zotero/storage/NR6G7V8D/Uzun et al. - 2024 - Successive Convexification "
    "for Nonlinear Model Predictive Control with Continuous-Time Constraint Sa.pdf"
)

# ---------------------------------------------------------------------------
# Parameters.  PAPER_VALUES are stated in the paper (Table 1 / Sec. 4).
# ASSUMED_DEFAULTS are NOT stated in the paper; they are numeric choices of
# this benchmark and must never be reported as paper values.
# ---------------------------------------------------------------------------
PAPER_VALUES = {
    "duration": 30.0,          # T [s]
    "horizon": 8.0,            # t_h [s]
    "mpc_intervals": 4,        # K
    "nodes": 17,               # N
    "iterations": 3,           # j_max (SCP iterations per NMPC run)
    "u_max": 5.0,              # [m/s^2], U = {||u||_2 <= u_max}
    "c_d": 0.05,               # drag coefficient [1/m]
    "shape_matrix": [[2.4, 0.0], [0.0, 0.4]],   # S_i, identical for i = 1..5
    "obstacle_centers": [[-6.5, -3.0], [-1.8, 3.5], [0.5, -3.5], [2.7, 1.5], [6.2, -3.0]],
    "delta": 0.05,             # delta_obs constraint tightening
    "r_sens": 6.0,             # sensing range, ||S_i (r - c_i)|| <= r_sens
}

ASSUMED_DEFAULTS = {
    "hold": "FOH",             # paper uses FOH (eq. 8); ZOH supported as a variant
    "epsilon": 1e-6,           # per-interval budget y_{k+1} - y_k <= eps (value not stated)
    "x0": [-10.0, 0.0, 0.0, 0.0],  # initial state [r, v] (not stated)
    "penalty": 1e4,            # w_ep (not stated)
    "prox": 1.0,               # w_prox (not stated)
    "audit_step": 1e-3,        # independent audit sample step [s]
    "terminal_cost": 0.0,      # L_h not stated; taken as zero
}

PAPER_DEFAULTS = {**PAPER_VALUES, **ASSUMED_DEFAULTS}

PARAM_SOURCE = {
    **{k: "paper Table 1 / Sec. 4" for k in PAPER_VALUES},
    **{k: "assumption (not stated in paper)" for k in ASSUMED_DEFAULTS},
}
PARAM_SOURCE["hold"] = "paper eq. (8) is FOH; ZOH is a benchmark variant"

C_D = PAPER_VALUES["c_d"]
U_MAX = PAPER_VALUES["u_max"]
SHAPE = np.array(PAPER_VALUES["shape_matrix"], dtype=float)
SHAPE_DIAG = np.diag(SHAPE).copy()
CENTERS = np.array(PAPER_VALUES["obstacle_centers"], dtype=float)  # (5, 2)
N_OBS = CENTERS.shape[0]
R_SENS = PAPER_VALUES["r_sens"]
DELTA_OBS = PAPER_VALUES["delta"]

# Augmented replay state: [r(2), v(2), cost, integral, physical_integral, tracking_abs]
_IDX_COST, _IDX_INT, _IDX_PHYS, _IDX_TRK = 4, 5, 6, 7


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------
def reference(t):
    """Paper r_ref(t) = [-10 + 2t/3, 5 e^{-0.05 t} sin(pi/2 + t/5 + t^2/36)], shape (..., 2)."""
    t = np.asarray(t, dtype=float)
    rx = -10.0 + 2.0 * t / 3.0
    ry = 5.0 * np.exp(-0.05 * t) * np.sin(np.pi / 2.0 + t / 5.0 + t ** 2 / 36.0)
    return np.stack([rx, ry], axis=-1)


def reference_rate(t):
    """d r_ref / dt, shape (..., 2) (analytic, for parity fixtures)."""
    t = np.asarray(t, dtype=float)
    ph = np.pi / 2.0 + t / 5.0 + t ** 2 / 36.0
    e = np.exp(-0.05 * t)
    vx = np.full_like(t, 2.0 / 3.0)
    vy = 5.0 * e * (-0.05 * np.sin(ph) + np.cos(ph) * (0.2 + t / 18.0))
    return np.stack([vx, vy], axis=-1)


def obstacle_g(position):
    """Untightened paper residual g_i = 1 - ||S (r - c_i)||_2 for all 5 obstacles, shape (..., 5)."""
    p = np.asarray(position, dtype=float)
    d = (p[..., None, :] - CENTERS) * SHAPE_DIAG
    return 1.0 - np.linalg.norm(d, axis=-1)


def obstacle_g_grad(position):
    """d g_i / d r, shape (..., 5, 2).  Undefined at an obstacle center (returns 0 there)."""
    p = np.asarray(position, dtype=float)
    d = (p[..., None, :] - CENTERS) * SHAPE_DIAG
    n = np.linalg.norm(d, axis=-1, keepdims=True)
    safe = np.where(n > 0, n, 1.0)
    return np.where(n > 0, -(d * SHAPE_DIAG) / safe, 0.0)


def sensed_mask(position):
    """Paper O_c: bool[5], ||S (r - c_i)||_2 <= r_sens at the measured root (frozen per horizon)."""
    p = np.asarray(position, dtype=float).reshape(2)
    return np.linalg.norm((p - CENTERS) * SHAPE_DIAG, axis=-1) <= R_SENS


def density(position, mask, delta=DELTA_OBS):
    """CTCS integrand sum_{i in mask} max(0, g_i + delta)^2, shape (...)."""
    g = obstacle_g(position)
    m = np.asarray(mask, dtype=bool)
    return np.sum(np.where(m, np.maximum(0.0, g + delta) ** 2, 0.0), axis=-1)


def rhs(t, x, u):
    """Physical dynamics [v, u - c_d ||v|| v] (t unused; kept for the shared signature)."""
    x = np.asarray(x, dtype=float)
    u = np.asarray(u, dtype=float)
    v = x[..., 2:4]
    sp = np.linalg.norm(v, axis=-1, keepdims=True)
    return np.concatenate([v, u - C_D * sp * v], axis=-1)


def rhs_jacobians(x, u):
    """Analytic (A, B) = (d rhs/dx, d rhs/du) for a single state (parity fixture)."""
    x = np.asarray(x, dtype=float).reshape(4)
    v = x[2:4]
    sp = np.linalg.norm(v)
    A = np.zeros((4, 4))
    A[0:2, 2:4] = np.eye(2)
    dv = sp * np.eye(2) + (np.outer(v, v) / sp if sp > 0 else 0.0)
    A[2:4, 2:4] = -C_D * dv
    B = np.zeros((4, 2))
    B[2:4, :] = np.eye(2)
    return A, B


def density_grad(position, mask, delta=DELTA_OBS):
    """d density / d r for a single position, shape (2,)."""
    p = np.asarray(position, dtype=float).reshape(2)
    g = obstacle_g(p)
    G = obstacle_g_grad(p)
    w = np.where(np.asarray(mask, dtype=bool), 2.0 * np.maximum(0.0, g + delta), 0.0)
    return w @ G


def hold_control(tau, u0, u1, duration, hold):
    """Control at local time tau in [0, duration]; FOH (paper eq. 8) or ZOH (u0)."""
    u0 = np.asarray(u0, dtype=float)
    u1 = np.asarray(u1, dtype=float)
    tau = np.asarray(tau, dtype=float)
    h = _check_hold(hold)
    if h == "ZOH":
        return np.broadcast_to(u0, tau.shape + (2,)).copy()
    s = np.clip(tau / duration, 0.0, 1.0)[..., None]
    return (1.0 - s) * u0 + s * u1


def _check_hold(hold):
    h = str(hold).upper()
    if h not in ("FOH", "ZOH"):
        raise ValueError(f"hold must be FOH or ZOH, got {hold!r}")
    return h


def node_times(tc, horizon=PAPER_VALUES["horizon"], nodes=PAPER_VALUES["nodes"]):
    """Uniform grid t_c = t_1 < ... < t_N = t_c + t_h (paper eq. 7)."""
    return tc + np.linspace(0.0, horizon, nodes)


# ---------------------------------------------------------------------------
# Ellipse geometry (secondary, Euclidean metres)
# ---------------------------------------------------------------------------
def _eberly_bisect(y0, y1, n0, z1, r0, s0, s1, iters):
    for _ in range(iters):
        s = 0.5 * (s0 + s1)
        gs = (n0 / (s + r0)) ** 2 + (z1 / (s + 1.0)) ** 2 - 1.0
        s0 = np.where(gs > 0, s, s0)
        s1 = np.where(gs > 0, s1, s)
    s = 0.5 * (s0 + s1)
    return np.hypot(r0 * y0 / (s + r0) - y0, y1 / (s + 1.0) - y1)


def _dist_to_ellipse_q1(e0, e1, y0, y1, iters=200):
    """Eberly's robust point-to-ellipse distance, first quadrant, e0 >= e1, vectorized."""
    y0 = np.asarray(y0, dtype=float)
    y1 = np.asarray(y1, dtype=float)
    # generic case y0 > 0, y1 > 0
    z0, z1 = y0 / e0, y1 / e1
    gq = z0 ** 2 + z1 ** 2 - 1.0
    r0 = (e0 / e1) ** 2
    n0 = r0 * z0
    s0 = z1 - 1.0
    s1 = np.where(gq < 0, 0.0, np.hypot(n0, z1) - 1.0)
    with np.errstate(divide="ignore", invalid="ignore"):
        d_gen = _eberly_bisect(y0, y1, n0, z1, r0, s0, s1, iters)

    # y1 > 0, y0 == 0
    d_y0zero = np.abs(y1 - e1)
    # y1 == 0
    numer0 = e0 * y0
    denom0 = e0 ** 2 - e1 ** 2
    xde0 = np.clip(numer0 / denom0, 0.0, 1.0)
    d_in = np.hypot(e0 * xde0 - y0, e1 * np.sqrt(1.0 - xde0 ** 2))
    d_y1zero = np.where(numer0 < denom0, d_in, np.abs(y0 - e0))

    return np.where(y1 > 0, np.where(y0 > 0, d_gen, d_y0zero), d_y1zero)


def ellipse_signed_distance(position):
    """Euclidean signed distance [m] to each obstacle ellipse boundary, shape (..., 5); <0 inside."""
    p = np.asarray(position, dtype=float)
    d = p[..., None, :] - CENTERS                       # (..., 5, 2)
    semi = 1.0 / SHAPE_DIAG                             # semi-axes along x, y
    major = int(np.argmax(semi))
    minor = 1 - major
    dist = _dist_to_ellipse_q1(semi[major], semi[minor], np.abs(d[..., major]), np.abs(d[..., minor]))
    inside = obstacle_g(p) > 0.0
    return np.where(inside, -dist, dist)


# ---------------------------------------------------------------------------
# Independent replay / audit
# ---------------------------------------------------------------------------
def _augmented_rhs(t0, u0, u1, duration, hold, mask, delta):
    m = np.asarray(mask, dtype=bool)

    def f(tau, z):
        u = hold_control(tau, u0, u1, duration, hold)
        r = z[0:2]
        dx = rhs(t0 + tau, z[0:4], u)
        e = r - reference(t0 + tau)
        g = obstacle_g(r)
        out = np.empty(8)
        out[0:4] = dx
        out[_IDX_COST] = e @ e
        out[_IDX_INT] = np.sum(np.where(m, np.maximum(0.0, g + delta) ** 2, 0.0))
        out[_IDX_PHYS] = np.sum(np.maximum(0.0, g) ** 2)
        out[_IDX_TRK] = np.sqrt(e @ e)
        return out

    return f


def _sample_times(duration, step):
    n = int(np.floor(duration / step + 1e-9))
    t = np.arange(n + 1) * step
    if duration - t[-1] > 1e-12:
        t = np.append(t, duration)
    else:
        t[-1] = duration
    return t


def replay_interval(x, u0, u1, t0, duration, hold, mask, delta=DELTA_OBS, step=1e-3,
                    rtol=1e-11, atol=1e-12, refine_peaks=True):
    """High-accuracy independent replay of one held-control interval.

    x        physical state [r, v] (4,) at absolute time t0 (augmented integrals start at 0)
    u0, u1   node controls; FOH ramps u0->u1 over ``duration``, ZOH holds u0
    mask     bool[5] sensed obstacles for the tightened density (frozen for the horizon)

    Returns a dict:
      end                (4,) physical state at t0 + duration
      cost               int ||r - r_ref(t)||^2 dt
      integral           int sum_{mask} max(0, g_i + delta)^2 dt      (solver CTCS increment)
      physical_integral  int sum_{all 5} max(0, g_i)^2 dt             (untightened)
      tracking_abs_integral  int ||r - r_ref(t)|| dt                  (paper Fig. 4 metric)
      max_physical_excess    max(0, max_t max_i g_i), all 5 obstacles, refined peaks
      max_g, max_g_obstacle, max_g_time   raw peak residual (may be negative)
      max_sum_excess     max_t sum_i max(0, g_i)   (paper Fig. 4 violation metric)
      violation_duration measure of {t : max_i g_i(t) > 0}, event-resolved
      min_clearance      min_t min_i (-g_i), shape residual (dimensionless, >0 outside)
      min_geometric_clearance  min_t min_i Euclidean signed distance [m]
      times, states, controls, g   dense samples every ``step`` incl. exact end
      crossings          list of (time, obstacle, direction) zero crossings of g_i
      ok, message, nfev
    """
    h = _check_hold(hold)
    x = np.asarray(x, dtype=float).reshape(4)
    u0 = np.asarray(u0, dtype=float).reshape(2)
    u1 = np.asarray(u1, dtype=float).reshape(2)
    mask = np.asarray(mask, dtype=bool).reshape(N_OBS)
    duration = float(duration)
    if not duration > 0.0:
        raise ValueError("duration must be positive")
    f = _augmented_rhs(float(t0), u0, u1, duration, h, mask, float(delta))

    events = []
    for i in range(N_OBS):
        def ev(tau, z, i=i):
            return obstacle_g(z[0:2])[i]
        events.append(ev)

    z0 = np.zeros(8)
    z0[0:4] = x
    sol = solve_ivp(f, (0.0, duration), z0, method="DOP853", rtol=rtol, atol=atol,
                    max_step=min(step, duration), dense_output=True, events=events)
    tau = _sample_times(duration, step)
    Z = sol.sol(tau).T
    Z[-1] = sol.y[:, -1]
    Z[0] = z0
    states = Z[:, 0:4]
    controls = hold_control(tau, u0, u1, duration, h)
    g = obstacle_g(states[:, 0:2])
    gmax = g.max(axis=1)

    # zero crossings (events) of every physical residual
    crossings = []
    for i, te in enumerate(sol.t_events):
        for tt in te:
            gi = lambda s: obstacle_g(sol.sol(s)[0:2])[i]
            eps_t = 1e-7
            direction = "enter" if gi(min(tt + eps_t, duration)) > gi(max(tt - eps_t, 0.0)) else "exit"
            crossings.append((float(t0 + tt), i, direction))

    # violation duration: split at samples and event times, classify by midpoint
    br = np.unique(np.concatenate([tau] + [np.asarray(te) for te in sol.t_events]))
    mids = 0.5 * (br[:-1] + br[1:])
    gmid = obstacle_g(sol.sol(mids)[0:2].T).max(axis=1) if mids.size else np.array([])
    viol_dur = float(np.sum(np.where(gmid > 0.0, np.diff(br), 0.0)))

    # refine per-obstacle peaks between samples (narrow pockets)
    peak_g = g.max(axis=0)
    peak_t = tau[g.argmax(axis=0)]
    if refine_peaks and tau.size >= 3:
        for i in range(N_OBS):
            gi_s = g[:, i]
            idx = np.where((gi_s[1:-1] >= gi_s[:-2]) & (gi_s[1:-1] >= gi_s[2:]))[0] + 1
            # only refine candidates that can matter: within 1e-3 of the sampled peak or near zero
            idx = idx[(gi_s[idx] > peak_g[i] - 1e-3) | (gi_s[idx] > -1e-3)]
            for j in idx[:200]:
                a, b = tau[j - 1], tau[j + 1]
                res = minimize_scalar(lambda s: -obstacle_g(sol.sol(s)[0:2])[i],
                                      bounds=(a, b), method="bounded",
                                      options={"xatol": 1e-10})
                if -res.fun > peak_g[i]:
                    peak_g[i] = -res.fun
                    peak_t[i] = res.x
    i_star = int(np.argmax(peak_g))
    max_g = float(peak_g[i_star])

    sum_exc = np.maximum(0.0, g).sum(axis=1)
    geo = ellipse_signed_distance(states[:, 0:2])
    zend = sol.y[:, -1]
    ok = bool(sol.success) and bool(np.all(np.isfinite(zend)))
    return {
        "ok": ok,
        "message": str(sol.message),
        "nfev": int(sol.nfev),
        "hold": h,
        "t0": float(t0),
        "duration": duration,
        "delta": float(delta),
        "mask": mask.copy(),
        "start": x.copy(),
        "end": zend[0:4].copy(),
        "cost": float(zend[_IDX_COST]),
        "integral": float(zend[_IDX_INT]),
        "physical_integral": float(zend[_IDX_PHYS]),
        "tracking_abs_integral": float(zend[_IDX_TRK]),
        "max_physical_excess": max(0.0, max_g),
        "max_g": max_g,
        "max_g_obstacle": i_star,
        "max_g_time": float(t0 + peak_t[i_star]),
        "max_g_per_obstacle": peak_g.copy(),
        "max_sum_excess": float(sum_exc.max()),
        "violation_duration": viol_dur,
        "min_clearance": -max_g,  # refined peak, so <= the sampled min of -g
        "min_geometric_clearance": float(geo.min()),
        "max_control_norm": float(np.linalg.norm(controls, axis=1).max()),
        "crossings": crossings,
        "times": t0 + tau,
        "states": states,
        "controls": controls,
        "g": g,
    }


def replay_sequence(x0, segments, delta=DELTA_OBS, step=1e-3):
    """Chain replay_interval over applied segments (closed-loop audit).

    segments: iterable of dicts with keys u0, u1, t0, duration, hold, mask.
    Each segment starts from the previous segment's audited end state (the plant),
    so this is an open-loop replay of the *applied* controls, independent of the solver.
    Returns (summary dict, list of per-segment results, dense arrays dict).
    """
    x = np.asarray(x0, dtype=float).reshape(4)
    results = []
    for s in segments:
        r = replay_interval(x, s["u0"], s["u1"], s["t0"], s["duration"], s["hold"], s["mask"],
                            delta=delta, step=step)
        results.append(r)
        x = r["end"]
        if not r["ok"]:
            break
    dense = concat_dense(results)
    return summarize(results), results, dense


def concat_dense(results):
    """Concatenate dense samples of consecutive replays (drops duplicated junction samples)."""
    if not results:
        return {k: np.zeros((0,)) for k in ("times", "states", "controls", "g")}
    keep = [np.ones(results[0]["times"].size, bool)]
    for r in results[1:]:
        k = np.ones(r["times"].size, bool)
        k[0] = False
        keep.append(k)
    out = {}
    for key in ("times", "states", "controls", "g"):
        out[key] = np.concatenate([r[key][k] for r, k in zip(results, keep)], axis=0)
    out["reference"] = reference(out["times"])
    return out


def summarize(results):
    """Aggregate per-interval audits into a run-level summary (all physical metrics)."""
    if not results:
        return {"ok": False, "n_intervals": 0}
    T = sum(r["duration"] for r in results)
    i = int(np.argmax([r["max_g"] for r in results]))
    return {
        "ok": all(r["ok"] for r in results),
        "n_intervals": len(results),
        "duration": T,
        "cost": sum(r["cost"] for r in results),
        "integral": sum(r["integral"] for r in results),
        "max_interval_integral": max(r["integral"] for r in results),
        "physical_integral": sum(r["physical_integral"] for r in results),
        "mean_tracking_error": sum(r["tracking_abs_integral"] for r in results) / T,
        "max_physical_excess": max(r["max_physical_excess"] for r in results),
        "max_g": results[i]["max_g"],
        "max_g_time": results[i]["max_g_time"],
        "max_g_obstacle": results[i]["max_g_obstacle"],
        "max_sum_excess": max(r["max_sum_excess"] for r in results),
        "violation_duration": sum(r["violation_duration"] for r in results),
        "min_clearance": min(r["min_clearance"] for r in results),
        "min_geometric_clearance": min(r["min_geometric_clearance"] for r in results),
        "max_control_norm": max(r["max_control_norm"] for r in results),
        "n_crossings": sum(len(r["crossings"]) for r in results),
        "end": results[-1]["end"],
    }


def interval_budget_check(increments, epsilon):
    """Per-interval CTCS budget y_{k+1} - y_k <= epsilon (paper eq. 11). Returns dict."""
    inc = np.asarray(increments, dtype=float)
    return {
        "epsilon": float(epsilon),
        "max_increment": float(inc.max()) if inc.size else 0.0,
        "n_over_budget": int(np.sum(inc > epsilon)),
        "ok": bool(np.all(inc <= epsilon)),
    }


def zero_control_initial_guess(x_root, tc, horizon=PAPER_VALUES["horizon"],
                               nodes=PAPER_VALUES["nodes"], mask=None, hold="FOH",
                               delta=DELTA_OBS, step=1e-3):
    """First-solve initial guess: zero control integrated from the measured root.

    Returns node states X (N, 4), augmented node values l (N,) and y (N,) consistent with
    the integrated flow (l_1 = y_1 = 0), controls U (N, 2) = 0, and per-interval increments.
    """
    if mask is None:
        mask = sensed_mask(np.asarray(x_root)[0:2])
    t = node_times(tc, horizon, nodes)
    X = np.zeros((nodes, 4))
    X[0] = np.asarray(x_root, dtype=float)
    l = np.zeros(nodes)
    y = np.zeros(nodes)
    zero = np.zeros(2)
    for k in range(nodes - 1):
        r = replay_interval(X[k], zero, zero, t[k], t[k + 1] - t[k], hold, mask,
                            delta=delta, step=step, refine_peaks=False)
        X[k + 1] = r["end"]
        l[k + 1] = l[k] + r["cost"]
        y[k + 1] = y[k] + r["integral"]
    return {"t": t, "X": X, "U": np.zeros((nodes, 2)), "l": l, "y": y,
            "increments": np.diff(y), "mask": np.asarray(mask, bool)}


# ---------------------------------------------------------------------------
# CLI defaults, provenance and persistence helpers
# ---------------------------------------------------------------------------
def add_common_args(parser):
    """Register the shared CLI flags with the shared defaults."""
    d = PAPER_DEFAULTS
    parser.add_argument("--horizon", type=float, default=d["horizon"])
    parser.add_argument("--nodes", type=int, default=d["nodes"])
    parser.add_argument("--mpc-intervals", type=int, default=d["mpc_intervals"])
    parser.add_argument("--duration", type=float, default=d["duration"])
    parser.add_argument("--hold", type=str.upper, choices=("FOH", "ZOH"), default=d["hold"])
    parser.add_argument("--epsilon", type=float, default=d["epsilon"])
    parser.add_argument("--delta", type=float, default=d["delta"])
    parser.add_argument("--x0", type=float, nargs=4, default=list(d["x0"]))
    parser.add_argument("--penalty", type=float, default=d["penalty"])
    parser.add_argument("--prox", type=float, default=d["prox"])
    parser.add_argument("--iterations", type=int, default=d["iterations"])
    parser.add_argument("--audit-step", type=float, default=d["audit_step"])
    parser.add_argument("--output-dir", type=str, required=True)
    return parser


def config_provenance(args_dict):
    """Annotate each config entry with whether it equals the paper value or is an assumption."""
    prov = {}
    for k, v in args_dict.items():
        if k in PAPER_VALUES:
            same = _jsonable(v) == _jsonable(PAPER_VALUES[k])
            prov[k] = "paper" if same else "override of paper value"
        elif k in ASSUMED_DEFAULTS:
            same = _jsonable(v) == _jsonable(ASSUMED_DEFAULTS[k])
            prov[k] = "assumption (default)" if same else "assumption (overridden)"
        else:
            prov[k] = "run option"
    return prov


def _sha256(path):
    try:
        with open(path, "rb") as fh:
            return hashlib.sha256(fh.read()).hexdigest()
    except OSError:
        return None


def _git_rev(path):
    try:
        out = subprocess.run(["git", "-C", str(path), "rev-parse", "HEAD"],
                             capture_output=True, text=True, timeout=5)
        return out.stdout.strip() or None
    except Exception:
        return None


def source_provenance(extra_files=()):
    here = Path(__file__).resolve().parent
    files = [Path(__file__).resolve()] + [Path(f).resolve() for f in extra_files]
    import scipy
    return {
        "paper": PAPER_SOURCE,
        "paper_sha256": _sha256(PAPER_SOURCE),
        "files": {str(f): _sha256(f) for f in files},
        "git_head": _git_rev(here),
        "argv": list(sys.argv),
        "python": sys.version,
        "numpy": np.__version__,
        "scipy": scipy.__version__,
        "platform": platform.platform(),
        "timestamp": _dt.datetime.now().isoformat(timespec="seconds"),
    }


def _jsonable(o):
    if isinstance(o, dict):
        return {str(k): _jsonable(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [_jsonable(v) for v in o]
    if isinstance(o, np.ndarray):
        return _jsonable(o.tolist())
    if isinstance(o, (np.bool_,)):
        return bool(o)
    if isinstance(o, np.integer):
        return int(o)
    if isinstance(o, np.floating):
        o = float(o)
    if isinstance(o, float) and not np.isfinite(o):
        return str(o)
    if isinstance(o, Path):
        return str(o)
    return o


def save_json(path, obj):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w") as fh:
        json.dump(_jsonable(obj), fh, indent=2)
    return path


def save_csv(path, rows):
    """rows: list of flat dicts (union of keys becomes the header)."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    keys = []
    for r in rows:
        for k in r:
            if k not in keys:
                keys.append(k)
    with open(path, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=keys)
        w.writeheader()
        for r in rows:
            w.writerow({k: _jsonable(v) for k, v in r.items()})
    return path


def save_npz(path, **arrays):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(path, **{k: np.asarray(v) for k, v in arrays.items()})
    return path


def write_manifest(output_dir, name, config, status, files=(), summary=None, extra_files=()):
    """Machine-readable manifest.json; ``status`` must be 'pass' or 'fail'. Returns exit code."""
    if status not in ("pass", "fail"):
        raise ValueError("status must be 'pass' or 'fail'")
    out = Path(output_dir)
    manifest = {
        "experiment": name,
        "status": status,
        "config": config,
        "config_provenance": config_provenance(config),
        "paper_values": PAPER_VALUES,
        "assumed_defaults": ASSUMED_DEFAULTS,
        "source": source_provenance(extra_files),
        "files": [str(f) for f in files],
        "summary": summary,
    }
    save_json(out / "manifest.json", manifest)
    return 0 if status == "pass" else 1
