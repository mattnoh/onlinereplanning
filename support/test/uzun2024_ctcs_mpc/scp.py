"""Uzun et al. 2024 CTCS NMPC (original 2-D obstacle problem) in native OpenSCvx.

Run with /home/lab/miniforge3/envs/openscvx/bin/python. The library is not
modified: the horizon OCP is an ordinary ``ox.Problem`` with user states
(position, velocity, running-cost integrator ``l``, constraint-violation
integrator ``y``), native convex cross-node rows ``y.at(k+1) - y.at(k) <= eps``
(paper eq. 12c) and a terminal ``Minimize`` on ``l`` (paper eq. 6a). Every
replan is ``reset()`` + ``solve(max_iters=J, continuous=True)``.

Plant time is frozen during solves (synchronous): after each solve the first K
intervals are applied with the same hold, through ``common.replay_interval``.
Each solve and each executed interval is re-audited independently at 1 ms.
With ``--constraint-mode stage``, obstacle rows are ordinary nonconvex
inequalities at all nodes including terminal; the obstacle y state and its
increment constraints are absent. Integral values are then audit diagnostics.
See README.md for the representation and explicit assumptions.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import platform
import subprocess
import sys
import time
import traceback

os.environ.setdefault("JAX_PLATFORMS", "cpu")
os.environ.setdefault("XDG_CACHE_HOME", "/tmp/openscvx-uzun2024-cache")
os.environ.setdefault("OMP_NUM_THREADS", "1")
HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
OPENSCVX = Path(os.environ.get("OPENSCVX_ROOT", str(Path.home() / "projects/solver/SCP/OpenSCvx")))
sys.path.insert(0, str(OPENSCVX))
sys.path.insert(0, str(HERE))

import numpy as np
from scipy.integrate import solve_ivp

import common  # shared contract (reference, obstacle_g, sensed_mask, rhs, replay_interval)
import openscvx as ox

PAPER_PDF = ("/home/lab/Zotero/storage/NR6G7V8D/Uzun et al. - 2024 - Successive Convexification "
             "for Nonlinear Model Predictive Control with Continuous-Time Constraint Sa.pdf")
# Paper Table 1. Kept locally so the OpenSCvx expressions are built from the
# paper, then cross-checked numerically against common.py at startup.
CD = 0.05
SHAPE = np.array([2.4, 0.4])          # S_i = diag(2.4, 0.4), i = 1..5
CENTERS = np.array([[-6.5, -3.0], [-1.8, 3.5], [0.5, -3.5], [2.7, 1.5], [6.2, -3.0]])
N_OBS = len(CENTERS)

ASSUMPTIONS = {
    "drag_norm_regularization": "||v|| lowered as sqrt(v.v + drag_eps2), drag_eps2=1e-30, because "
                                "jnp.linalg.norm has a NaN gradient at v=0 (the stationary initial "
                                "guess); value change <= cd*1e-15*||v||. Not a paper value.",
    "scaling": "physical x,u,y have identity scaling; l uses cost_scale, with lam_cost compensated "
               "to preserve the physical objective. Proximal weight on physical l is reduced by cost_scale^2 "
               "and its virtual-control weight by cost_scale. Paper scaling/weights are unreported.",
    "lam_prox": "OpenSCvx proximal term is lam_prox*||z-z_ref||^2; paper (15) is w_prox/2*||.||^2, "
                "so lam_prox = w_prox/2.",
    "lam_vb": "Nodal nonconvex-row slack L1 weight equals penalty (1e4 by default); OpenSCvx default zero would leave node rows unenforced. CTCS mode has no nonconvex obstacle nodal rows, so this adds no cost there.",
    "lam_vc": "L1 virtual-control weight on scaled defects; physical l weight is penalty/cost_scale.",
    "terminal": "no terminal cost/constraint besides l(N) (paper L_h, P_h unspecified for the example).",
    "state_bounds": "CTCS mode only has the obstacle y state. Generous redundant nodal boxes: |r|,|v|<=1e3, |l|<=1e6, |y|<=1e3; y has NO "
                    "global <=eps bound; only the cross-node increment rows.",
    "loop_termination": "Execute [0,T): solve at tc<T, clip the last execution to T; avoid an extra step beyond the reported simulation.",
    "ode": "Dopri8, rtol=atol=ode_tol (default 1e-10) inside OpenSCvx; warm-start tails / augmented "
           "rebase use scipy DOP853 rtol=atol=1e-12.",
    "conic_solver": "CLARABEL tol 1e-10 (paper used ECOS/MOSEK).",
    "library_time_ctcs": "openscvx builder.py:232 unconditionally adds CTCS(time in [min,max]) -> "
                         "state _ctcs_aug_0 in [0, licq_max=1e-4]. Time bounds are [-1, t_h+1], so its "
                         "integrand is identically 0 on the fixed 0..t_h clock; audited per solve "
                         "(library_time_ctcs_aug_max). It is not the paper y.",
    "zoh_tail":"under ZOH the last node control is unused by the dynamics; the tail control is "
                "u_{N-2} and the final node control is set equal to it.",
}


# ----------------------------------------------------------------------------
# Numpy reference model (independent of the OpenSCvx lowering)
# ----------------------------------------------------------------------------
def ref_np(t):
    t = np.asarray(t, dtype=float)
    return np.stack([-10. + 2. * t / 3.,
                     5. * np.exp(-0.05 * t) * np.sin(np.pi / 2 + t / 5 + t**2 / 36)], axis=-1)


def g_np(r):
    r = np.asarray(r, dtype=float)
    return 1. - np.linalg.norm(SHAPE * (r[..., None, :] - CENTERS), axis=-1)


def check_common(args):
    """Fail loudly if common.py disagrees with the paper constants used here."""
    rng = np.random.default_rng(0)
    ts = np.linspace(0., args.duration + args.horizon + 2., 97)
    r = rng.uniform(-12., 12., size=(64, 2))
    errs = {
        "reference": float(np.max(abs(np.asarray(common.reference(ts)) - ref_np(ts)))),
        "obstacle_g": float(np.max(abs(np.asarray(common.obstacle_g(r)) - g_np(r)))),
    }
    xs = rng.uniform(-5., 5., size=(16, 4)); us = rng.uniform(-5., 5., size=(16, 2))
    rhs_ref = np.c_[xs[:, 2:], us - CD * np.linalg.norm(xs[:, 2:], axis=1, keepdims=True) * xs[:, 2:]]
    errs["rhs"] = float(max(np.max(abs(np.asarray(common.rhs(0., x, u)) - f)) for x, u, f in zip(xs, us, rhs_ref)))
    mask_ok = all(np.array_equal(np.asarray(common.sensed_mask(p), bool),
                                 np.linalg.norm(SHAPE * (p - CENTERS), axis=1) <= 6.) for p in r)
    bad = {k: v for k, v in errs.items() if not v <= 1e-12}
    if bad or not mask_ok:
        raise RuntimeError(f"common.py disagrees with paper constants: {errs}, sensed_mask_ok={mask_ok}")
    return {**errs, "sensed_mask_ok": mask_ok}


def u_at(s, u0, u1, hold):
    return u0 if hold == "ZOH" else (1. - s) * u0 + s * u1


def aug_rhs(t_abs, z, u, mask, delta):
    xi = z[:4]
    dxi = np.asarray(common.rhs(t_abs, xi, u), dtype=float)
    dl = float(np.sum((xi[:2] - ref_np(t_abs))**2))
    dy = float(np.sum(mask * np.maximum(g_np(xi[:2]) + delta, 0.)**2))
    return np.r_[dxi, dl, dy]


def shoot(xi, u0, u1, t0_abs, dt, hold, mask, delta, tol=1e-12):
    """Integrate [xi, l, y] over one interval from l=y=0. Returns end (6,)."""
    def f(s, z):
        return dt * aug_rhs(t0_abs + s * dt, z, u_at(s, u0, u1, hold), mask, delta)
    sol = solve_ivp(f, (0., 1.), np.r_[xi, 0., 0.], method="DOP853", rtol=tol, atol=tol)
    if not sol.success:
        raise RuntimeError(f"guess integration failed: {sol.message}")
    return sol.y[:, -1]


def interval_controls(U, k, hold):
    return (U[k], U[k] if hold == "ZOH" else U[k + 1])


def rebase(xi_nodes, U, tc, dt, hold, mask, delta):
    """Recompute l, y node values from 0 with per-interval shooting from xi nodes."""
    n = len(xi_nodes)
    l = np.zeros(n); y = np.zeros(n)
    for k in range(n - 1):
        end = shoot(xi_nodes[k], *interval_controls(U, k, hold), tc + k * dt, dt, hold, mask, delta)
        l[k + 1] = l[k] + end[4]; y[k + 1] = y[k] + end[5]
    return l, y


def initial_guess(x0, args, mask, dt):
    """Stationary / zero-control guess integrated through the dynamics."""
    n = args.nodes
    U = np.zeros((n, 2)); xi = [np.asarray(x0, float)]
    for k in range(n - 1):
        xi.append(shoot(xi[-1], U[k], U[k + 1], k * dt, dt, args.hold, mask, args.delta)[:4])
    xi = np.asarray(xi)
    l, y = rebase(xi, U, 0., dt, args.hold, mask, args.delta)
    return xi, U, l, y


def warm_start(xi_prev, U_prev, root, tc_new, args, mask, dt):
    """Paper Sec. 3 shift by K; tail integrated from the last node with the last control."""
    n, K = args.nodes, args.mpc_intervals
    xi = np.empty_like(xi_prev); U = np.empty_like(U_prev)
    xi[:n - K] = xi_prev[K:]; U[:n - K] = U_prev[K:]
    u_last = U_prev[-1] if args.hold == "FOH" else U_prev[-2]
    U[n - K:] = u_last
    if args.hold == "ZOH":
        U[n - K - 1] = U_prev[-2]
    t_last = tc_new + (n - K - 1) * dt
    for j in range(1, K + 1):
        xi[n - K - 1 + j] = shoot(xi[n - K - 2 + j], u_last, u_last, t_last + (j - 1) * dt,
                                  dt, "ZOH", mask, args.delta)[:4]
    xi[0] = root  # measured root replaces the shifted first node
    l, y = rebase(xi, U, tc_new, dt, args.hold, mask, args.delta)
    return xi, U, l, y


# ----------------------------------------------------------------------------
# Native OpenSCvx horizon problem
# ----------------------------------------------------------------------------
def build(args):
    n = args.nodes
    big = 1e3
    pos = ox.State("position", (2,)); vel = ox.State("velocity", (2,))
    l = ox.State("l", (1,)); y = ox.State("y", (1,))
    for s, b in ((pos, big), (vel, big), (l, 1e6), (y, big)):
        s.min = [-b] * s.shape[0]; s.max = [b] * s.shape[0]
        s.scaling_min = [-1.] * s.shape[0]; s.scaling_max = [1.] * s.shape[0]
    l.scaling_min = [-args.cost_scale]; l.scaling_max = [args.cost_scale]
    x0 = np.asarray(args.x0, float)
    pos.initial = x0[:2]; vel.initial = x0[2:]
    pos.final = [ox.Free(0.)] * 2; vel.final = [ox.Free(0.)] * 2
    l.initial = [0.]; l.final = [ox.Minimize(0.)]
    y.initial = [0.]; y.final = [ox.Free(0.)]
    for s in (pos, vel, l, y):
        s.guess = np.zeros((n, s.shape[0]))
    u = ox.Control("u", (2,), parameterization=args.hold)
    u.min = [-big, -big]; u.max = [big, big]
    u.scaling_min = [-1., -1.]; u.scaling_max = [1., 1.]
    u.guess = np.zeros((n, 2))
    t = ox.Time(initial=0., final=args.horizon, min=-1., max=args.horizon + 1.,
                time_dilation_min=args.horizon, time_dilation_max=args.horizon,
                uniform_time_grid=True)
    t.scaling_min = [-1.]; t.scaling_max = [1.]

    tc = ox.Parameter("tc", shape=(), value=0.)
    mask = ox.Parameter("obs_mask", shape=(N_OBS,), value=np.ones(N_OBS))
    ta = tc + t[0]
    ref = ox.Concat(-10. + (2. / 3.) * ta,
                    5. * ox.Exp(-0.05 * ta) * ox.Sin(np.pi / 2 + ta / 5. + ta * ta / 36.))
    speed = ox.Sqrt(ox.Sum(vel * vel) + args.drag_eps2)
    terms = []
    for i, c in enumerate(CENTERS):
        g = 1. - ox.linalg.Norm((pos - c) * SHAPE)
        terms.append(mask[i] * ox.Square(ox.PositivePart(g + args.delta)))
    density = terms[0]
    for term in terms[1:]:
        density = density + term
    dynamics = {"position": vel, "velocity": u - CD * speed * vel,
                "l": ox.Sum((pos - ref) * (pos - ref))}
    states = [pos, vel, l]
    constraints = [(ox.linalg.Norm(u) <= args.umax).convex()]
    if args.constraint_mode == "ctcs":
        dynamics["y"] = density
        states.append(y)
        # Paper (12c): one horizon accumulator, per-interval increment budgets.
        constraints += [(y.at(k + 1) - y.at(k) <= args.epsilon).convex() for k in range(n - 1)]
    else:
        # Ordinary NONCONVEX nodal rows: linearized by SCP, not CTCS-wrapped.
        # Mask is frozen from the measured root, identically to the CTCS case.
        # Include the terminal node. No y state or obstacle integral constraint.
        for i, c in enumerate(CENTERS):
            g = 1. - ox.linalg.Norm((pos - c) * SHAPE)
            constraints.append((mask[i] * (g + args.delta) <= 0.).at(list(range(n))))

    ode_kwargs = {"rtol": args.ode_tol, "atol": args.ode_tol, "num_substeps": 30}
    problem = ox.Problem(
        dynamics=dynamics, states=states, controls=[u], time=t,
        constraints=constraints, N=n, float_dtype="float64",
        algorithm={"k_max": args.iterations, "lam_cost": args.cost_scale, "lam_prox": args.prox / 2.,
                   "lam_vc": args.penalty, "lam_vb": args.penalty, "ep_tr": 0., "ep_vc": 0., "ep_vb": 0.,
                   "autotuner": ox.ConstantProximalWeight()},
        discretizer={"type": "VectorizeDiscretizeLinearize", "ode_solver": "Dopri8",
                     "diffrax_kwargs": ode_kwargs},
        solver={"cvx_solver": "CLARABEL", "solver_args": {
            "tol_gap_abs": 1e-10, "tol_gap_rel": 1e-10, "tol_feas": 1e-10,
            "max_iter": 300, "enforce_dpp": True}})
    problem.settings.dev.printing = args.verbose
    return problem, {"position": pos, "velocity": vel, "l": l, "y": y, "u": u, "time": t}


def set_guess(h, xi, U, l, y, args):
    h["position"].guess = xi[:, :2].copy(); h["velocity"].guess = xi[:, 2:].copy()
    h["l"].guess = l[:, None].copy()
    if args.constraint_mode == "ctcs":
        h["y"].guess = y[:, None].copy()
    h["u"].guess = U.copy()
    h["time"].guess = np.linspace(0., args.horizon, args.nodes)[:, None]


# ----------------------------------------------------------------------------
# Independent audits (common.replay_interval, 1 ms)
# ----------------------------------------------------------------------------
def replay(x, u0, u1, t0, dt, mask, args):
    rep = common.replay_interval(np.asarray(x, float), np.asarray(u0, float), np.asarray(u1, float),
                                 float(t0), float(dt), args.hold, np.asarray(mask, bool),
                                 delta=args.delta, step=args.audit_step)
    if not rep["ok"]:
        raise RuntimeError(f"1 ms audit integration failed at t0={t0}: {rep['message']}")
    return rep


def audit_plan(Xi, L, Y, U, tc, dt, mask, args):
    """Multiple-shooting (node-wise) and single-shooting audit of one horizon plan."""
    rows = []; state = Xi[0].copy()
    for k in range(args.nodes - 1):
        u0, u1 = interval_controls(U, k, args.hold)
        ms = replay(Xi[k], u0, u1, tc + k * dt, dt, mask, args)
        ss = replay(state, u0, u1, tc + k * dt, dt, mask, args)
        state = np.asarray(ss["end"], float)
        rows.append({
            "k": k, "t0": tc + k * dt,
            "solver_y_increment": float(Y[k + 1] - Y[k]) if Y is not None else None,
            "solver_l_increment": float(L[k + 1] - L[k]),
            "ms_integral": float(ms["integral"]), "ms_cost": float(ms["cost"]),
            "ms_phys_defect_inf": float(np.max(abs(np.asarray(ms["end"]) - Xi[k + 1]))),
            "ms_y_defect": float(abs(Y[k + 1] - Y[k] - ms["integral"])) if Y is not None else None,
            "ms_l_defect": float(abs(L[k + 1] - L[k] - ms["cost"])),
            "ms_max_physical_excess": float(ms["max_physical_excess"]),
            "ss_integral": float(ss["integral"]),
            "ss_max_physical_excess": float(ss["max_physical_excess"]),
            "ss_violation_duration": float(ss["violation_duration"]),
            "ss_node_drift_inf": float(np.max(abs(state - Xi[k + 1]))),
        })
    return rows


# ----------------------------------------------------------------------------
def provenance(args):
    def sha(p):
        p = Path(p)
        return hashlib.sha256(p.read_bytes()).hexdigest() if p.exists() else None
    try:
        commit = subprocess.check_output(["git", "-C", str(OPENSCVX), "rev-parse", "HEAD"], text=True).strip()
        dirty = subprocess.check_output(["git", "-C", str(OPENSCVX), "status", "--porcelain", "openscvx"],
                                        text=True).strip()
    except Exception as e:  # noqa: BLE001
        commit, dirty = f"unavailable: {e}", None
    return {
        "args": vars(args), "command": sys.argv, "python": sys.executable,
        "python_version": platform.python_version(),
        "scp_py_sha256": sha(__file__), "common_py_sha256": sha(Path(common.__file__)),
        "common_py_path": str(Path(common.__file__).resolve()),
        "openscvx_path": str(Path(ox.__file__).resolve()),
        "openscvx_commit": commit, "openscvx_expected_commit_prefix": "17a1e9f4",
        "openscvx_library_dirty": dirty, "paper_pdf": PAPER_PDF, "paper_pdf_sha256": sha(PAPER_PDF),
        "paper_values": {"T": 30., "t_h": 8., "K": 4, "N": 17, "j_max": 3, "u_max": 5., "c_d": CD,
                         "S": SHAPE.tolist(), "r_obs": CENTERS.tolist(), "delta_obs": .05, "r_sens": 6.,
                         "control_hold": "FOH"},
        "common_PAPER_DEFAULTS": {k: (np.asarray(v).tolist() if isinstance(v, np.ndarray) else v)
                                  for k, v in getattr(common, "PAPER_DEFAULTS", {}).items()},
        "assumptions_not_paper_values": {**ASSUMPTIONS,
                                         "epsilon": "paper does not report epsilon; CLI default 1e-6",
                                         "penalty": "paper does not report w_ep; CLI default 1e4",
                                         "prox": "paper does not report w_prox; CLI default 1",
                                         "x0": "paper does not report x0 numerically; default [-10,0,0,0]"},
        "constraint_mode": args.constraint_mode, "interval_budget_enforced": args.constraint_mode == "ctcs",
        "representation": ("single user state y, dy = sum_i mask_i*pos(1-||S(r-c_i)||+delta)^2, y(0)=0, "
                          "native convex CrossNode rows y.at(k+1)-y.at(k)<=epsilon, k=0..N-2; "
                          "no ox.ctcs augmentation; l(N) minimized." if args.constraint_mode == "ctcs" else
                          "Nonconvex mask_i*(g_i+delta)<=0 at all N nodes including terminal, "
                          "no obstacle y state, no integral constraint; l(N) minimized."),
    }


def write_csv(path, rows):
    if not rows:
        path.write_text("")
        return
    fields = list(dict.fromkeys(k for r in rows for k in r))
    with path.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields); w.writeheader(); w.writerows(rows)


def run(args, out):
    cfg = provenance(args)
    (out / "config.json").write_text(json.dumps(cfg, indent=2, default=str))
    source_dir = out / "source"
    source_dir.mkdir(exist_ok=True)
    for source in (Path(__file__), Path(common.__file__)):
        (source_dir / source.name).write_bytes(source.read_bytes())
    summary = {"status": "fail", "stage": "startup", "solves_completed": 0}
    solves, plan_rows, exec_rows = [], [], []
    arrays = {"X": [], "U": [], "guess_X": [], "guess_U": [], "tc": [], "mask": [], "root": []}
    executed = []  # common.replay_interval results of every applied interval (the plant)
    try:
        summary["common_check"] = check_common(args)
        dt = args.horizon / (args.nodes - 1)
        if args.mpc_intervals < 1 or args.mpc_intervals > args.nodes - 1:
            raise ValueError("mpc_intervals must be in [1, nodes-1]")
        dt_mpc = args.mpc_intervals * dt
        summary["stage"] = "build"
        problem, h = build(args)
        t_init = time.perf_counter(); problem.initialize()
        summary["initialize_seconds"] = time.perf_counter() - t_init
        sl = problem.slices
        idx = {k: sl[k] for k in ("position", "velocity", "l", "u")}
        if args.constraint_mode == "ctcs":
            idx["y"] = sl["y"]

        plant = np.asarray(args.x0, float); tc = 0.; i = 0
        prev = None
        while tc < args.duration - 1e-9:
            summary["stage"] = f"solve {i} tc={tc:.6g}"
            root = plant.copy()  # measured root (synchronous: plant frozen during solve)
            mask = np.asarray(common.sensed_mask(root[:2]), bool)
            if prev is None:
                gxi, gU, gl, gy = initial_guess(root, args, mask.astype(float), dt)
            else:
                gxi, gU, gl, gy = warm_start(prev[0], prev[1], root, tc, args, mask.astype(float), dt)
            h["position"].initial = root[:2]; h["velocity"].initial = root[2:]
            h["l"].initial = [0.]
            if args.constraint_mode == "ctcs":
                h["y"].initial = [0.]
            set_guess(h, gxi, gU, gl, gy, args)
            problem.parameters["tc"] = float(tc)
            problem.parameters["obs_mask"] = mask.astype(float)
            rec = {"solve": i, "tc": tc, "root": root.tolist(), "mask": "".join("1" if m else "0" for m in mask),
                   "guess_y_increment_max": float(np.max(np.diff(gy))), "status": "exception"}
            t0 = time.perf_counter()
            try:
                problem.reset()
                result = problem.solve(max_iters=args.iterations, continuous=True)
            except Exception as e:  # persist and fail, never silently pass
                rec["error"] = repr(e); solves.append(rec)
                raise
            rec["solve_seconds"] = time.perf_counter() - t0
            np.savez_compressed(out / f"iterations_{i:03d}.npz", X=np.asarray(result.X), U=np.asarray(result.U),
                                root=root, tc=tc, mask=mask)
            iters = int(problem._state.k) - 1
            cvx = getattr(getattr(problem, "_solver", None), "_problem", None)
            X = np.asarray(result.x, float); U = np.asarray(result.u, float)
            Xi = np.c_[X[:, idx["position"]], X[:, idx["velocity"]]]
            L = X[:, idx["l"]].ravel()
            Y = X[:, idx["y"]].ravel() if args.constraint_mode == "ctcs" else None
            Uc = U[:, idx["u"]]
            # Saved for every returned solve, including one that fails a later check.
            for key, v in (("X", X), ("U", U), ("guess_X", np.c_[gxi, gl, gy] if args.constraint_mode == "ctcs" else np.c_[gxi, gl]), ("guess_U", gU),
                           ("tc", tc), ("mask", mask), ("root", root)):
                arrays[key].append(v)
            vc = np.asarray(result.VC_history[-1]) if len(result.VC_history) else np.array([np.nan])
            rec.update({
                "iterations": iters, "iterations_requested": args.iterations,
                "openscvx_converged_flag": bool(result.converged),
                "last_cvx_status": getattr(cvx, "status", None),
                "J_tr": float(np.ravel(result.J_tr_history)[-1]) if np.size(result.J_tr_history) else None,
                "J_vc": float(np.ravel(result.J_vc_history)[-1]) if np.size(result.J_vc_history) else None,
                "J_vb": float(np.ravel(result.J_vb_history)[-1]) if np.size(result.J_vb_history) else None,
                "max_abs_virtual_control": float(np.nanmax(abs(vc))),
                "l_N": float(L[-1]), "y_N": float(Y[-1]) if Y is not None else None,
                "solver_y_increment_max": float(np.max(np.diff(Y))) if Y is not None else None,
                "max_u_norm": float(np.max(np.linalg.norm(Uc, axis=1))),
                # builder.py always adds CTCS(time in [min, max]); with bounds [-1, t_h+1]
                # its integrand is identically zero on the 0..t_h clock, so this must be 0.
                "library_time_ctcs_aug_max": float(max(
                    (np.max(abs(X[:, v])) for k, v in sl.items() if k.startswith("_ctcs_aug_")),
                    default=0.)),
                "time_grid_error": float(np.max(abs(X[:, sl["time"]].ravel()
                                                    - np.linspace(0., args.horizon, args.nodes)))),
            })
            finite = bool(np.all(np.isfinite(X)) and np.all(np.isfinite(U)))
            if not finite or iters != args.iterations:
                rec["status"] = "nonfinite" if not finite else "iteration_count_mismatch"
                solves.append(rec)
                raise RuntimeError(f"solve {i}: {rec['status']} (iterations={iters})")
            rows = audit_plan(Xi, L, Y, Uc, tc, dt, mask, args)
            for r in rows:
                r["solve"] = i
            plan_rows += rows
            ms_int = max(r["ms_integral"] for r in rows)
            rec.update({
                "ms_integral_max": ms_int,
                "ms_budget_met": bool(ms_int <= args.epsilon * (1 + 1e-6)),
                "ms_phys_defect_max": max(r["ms_phys_defect_inf"] for r in rows),
                "ms_y_defect_max": max(r["ms_y_defect"] for r in rows) if Y is not None else None,
                "ms_l_defect_max": max(r["ms_l_defect"] for r in rows),
                "ss_integral_max": max(r["ss_integral"] for r in rows),
                "ss_max_physical_excess": max(r["ss_max_physical_excess"] for r in rows),
                "ss_node_drift_max": max(r["ss_node_drift_inf"] for r in rows),
                "u_norm_excess": float(np.max(np.linalg.norm(Uc, axis=1)) - args.umax),
            })
            rec["defects_ok"] = bool(rec["ms_phys_defect_max"] <= args.defect_tol
                                     and (Y is None or rec["ms_y_defect_max"] <= args.defect_tol * args.epsilon)
                                     and rec["ms_l_defect_max"] <= args.defect_tol)
            rec["certified"] = bool(rec["defects_ok"] and rec["ms_budget_met"]
                                    and rec["ss_max_physical_excess"] <= args.excess_tol
                                    and rec["u_norm_excess"] <= 1e-8)
            node_rows = np.where(mask[None, :], g_np(Xi[:, :2]) + args.delta, -np.inf)
            rec["max_modeled_nodal_constraint"] = float(np.max(node_rows)) if mask.any() else -1.
            rec["root_defect_inf"] = float(np.max(abs(Xi[0] - root)))
            if args.constraint_mode == "stage":
                rec["certified"] = bool(rec["defects_ok"]
                    and rec["root_defect_inf"] <= args.defect_tol
                    and rec["max_modeled_nodal_constraint"] <= args.defect_tol
                    and rec["u_norm_excess"] <= args.defect_tol)
            rec["constraint_mode"] = args.constraint_mode
            rec["interval_budget_enforced"] = args.constraint_mode == "ctcs"
            rec["status"] = "solved"

            # Apply first K intervals to the plant with the matching hold.
            summary["stage"] = f"evolve {i}"
            for k in range(args.mpc_intervals):
                if tc + k * dt >= args.duration - 1e-9:
                    break
                u0, u1 = interval_controls(Uc, k, args.hold)
                applied_dt = min(dt, args.duration - (tc + k * dt))
                applied_u1 = u_at(applied_dt / dt, u0, u1, args.hold)
                rep = replay(plant, u0, applied_u1, tc + k * dt, applied_dt, mask, args)
                end = np.asarray(rep["end"], float)
                exec_rows.append({
                    "solve": i, "k": k, "t0": tc + k * dt, "duration": applied_dt,
                    "integral": float(rep["integral"]), "cost": float(rep["cost"]),
                    "budget_met": bool(rep["integral"] <= args.epsilon * (1 + 1e-6)),
                    "max_physical_excess": float(rep["max_physical_excess"]),
                    "violation_duration": float(rep["violation_duration"]),
                    "min_clearance": float(rep["min_clearance"]),
                    "predicted_minus_plant_inf": float(np.max(abs(Xi[k + 1] - end))),
                    "solver_y_increment": float(Y[k + 1] - Y[k]) if Y is not None else None,
                    **{f"x_end_{j}": float(v) for j, v in enumerate(end)},
                })
                executed.append(rep)
                plant = end
            ex = exec_rows[-args.mpc_intervals:]
            rec["executed_max_physical_excess"] = max(r["max_physical_excess"] for r in ex)
            rec["executed_integral_max"] = max(r["integral"] for r in ex)
            solves.append(rec)
            summary["solves_completed"] = i + 1
            print(f"[solve {i:2d}] tc={tc:6.2f} mask={rec['mask']} l_N={rec['l_N']:.4g} "
                  f"vc={rec['max_abs_virtual_control']:.2e} msdef={rec['ms_phys_defect_max']:.2e} "
                  f"ms_int={ms_int:.2e} exec_excess={rec['executed_max_physical_excess']:.3e} "
                  f"cert={rec['certified']} {rec['solve_seconds']:.2f}s", flush=True)
            prev = (Xi, Uc)
            i += 1
            tc = i * dt_mpc
        summary["stage"] = "done"
    except Exception:
        (out / "error.txt").write_text(traceback.format_exc())
        summary["error"] = traceback.format_exc().splitlines()[-1]
    finally:
        write_csv(out / "solves.csv", solves)
        write_csv(out / "plan_intervals.csv", plan_rows)
        write_csv(out / "executed_intervals.csv", exec_rows)
        np.savez_compressed(out / "arrays.npz", **{k: np.asarray(v) for k, v in arrays.items() if v},
                            slices=json.dumps({k: [v.start, v.stop] for k, v in problem.slices.items()})
                            if "problem" in locals() else "{}")
        if executed:
            common.save_npz(out / "closed_loop_dense.npz", **common.concat_dense(executed))
        closed = {
            "common_summarize": common.summarize(executed),
            "executed_intervals": len(exec_rows),
            "max_physical_excess": max((r["max_physical_excess"] for r in exec_rows), default=None),
            "violation_duration_total": sum(r["violation_duration"] for r in exec_rows),
            "min_clearance": min((r["min_clearance"] for r in exec_rows), default=None),
            "max_interval_integral": max((r["integral"] for r in exec_rows), default=None),
            "interval_budgets_met": all(r["budget_met"] for r in exec_rows) if exec_rows else False,
            "max_predicted_minus_plant": max((r["predicted_minus_plant_inf"] for r in exec_rows), default=None),
            "tracking_cost_sum": sum(r["cost"] for r in exec_rows),
            "final_state": plant.tolist() if "plant" in locals() else None,
        }
        summary["closed_loop"] = closed
        summary["solves_certified"] = sum(bool(s.get("certified")) for s in solves)
        summary["first_uncertified_solve"] = next((s["solve"] for s in solves if not s.get("certified")), None)
        summary["first_physical_violation_solve"] = next(
            (r["solve"] for r in exec_rows if r["max_physical_excess"] > args.excess_tol), None)
        metrics = closed["common_summarize"]
        summary["physical_execution_pass"] = bool(exec_rows and closed["max_physical_excess"] <= args.excess_tol
            and metrics.get("max_control_norm", float("inf")) <= args.umax + 1e-6)
        summary["useful_tracking_pass"] = bool(exec_rows and metrics.get("mean_tracking_error", float("inf")) <= args.tracking_tol)
        ok = (summary["stage"] == "done" and exec_rows
              and all(s.get("status") == "solved" for s in solves)
              and summary["physical_execution_pass"] and summary["useful_tracking_pass"]
              and (args.constraint_mode == "stage" or closed["interval_budgets_met"]))
        summary["status"] = "pass" if ok else "fail"
        summary["pass_definition"] = ("all solves completed with exactly the requested iterations, finite, "
                                      "AND closed-loop physical/input limits, executed interval budgets ONLY for CTCS mode, and mean tracking error <= tracking_tol. "
                                      "Per-solve 'certified' and interval budgets are reported separately.")
        common.save_json(out / "summary.json", summary)
        files = [out / f for f in ("config.json", "summary.json", "solves.csv", "plan_intervals.csv",
                                   "executed_intervals.csv", "arrays.npz", "closed_loop_dense.npz",
                                   "error.txt") if (out / f).exists()]
        summary["exit_code"] = common.write_manifest(
            out, "uzun2024_scp_openscvx", {k: v for k, v in vars(args).items() if k != "output_dir"},
            summary["status"], files=files, summary={k: summary[k] for k in (
                "status", "stage", "solves_completed", "solves_certified", "closed_loop")},
            extra_files=[__file__])
        print("SCP_SUMMARY " + json.dumps({k: summary[k] for k in
              ("status", "stage", "solves_completed", "solves_certified", "first_uncertified_solve",
               "first_physical_violation_solve")} | {"closed_loop": closed}, default=str), flush=True)
    return summary


def parse(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    common.add_common_args(ap)  # shared defaults; --penalty -> lam_vc, --prox -> 2*lam_prox
    ap.add_argument("--constraint-mode", choices=["ctcs", "stage"], default="ctcs",
                    help="CTCS interval budgets, or ordinary nonconvex obstacle rows at all nodes (no y state)")
    ap.add_argument("--cost-scale", type=float, default=100., help="integrated cost state scale; lam_cost compensated to preserve physical objective")
    ap.add_argument("--umax", type=float, default=common.U_MAX)
    ap.add_argument("--ode-tol", type=float, default=1e-10)
    ap.add_argument("--drag-eps2", type=float, default=1e-30)
    ap.add_argument("--defect-tol", type=float, default=1e-6)
    ap.add_argument("--excess-tol", type=float, default=1e-6)
    ap.add_argument("--tracking-tol", type=float, default=1., help="useful tracking threshold [m], benchmark acceptance rather than paper value")
    ap.add_argument("--verbose", action="store_true", help="OpenSCvx iteration printing")
    args = ap.parse_args(argv)
    args.output_dir = Path(args.output_dir)
    return args


def main(argv=None):
    args = parse(argv)
    out = args.output_dir.resolve()
    if out.exists() and any(out.iterdir()):
        sys.exit(f"output dir {out} exists and is not empty")
    out.mkdir(parents=True, exist_ok=True)
    summary = run(args, out)
    sys.exit(summary.get("exit_code", 1))


if __name__ == "__main__":
    main()
