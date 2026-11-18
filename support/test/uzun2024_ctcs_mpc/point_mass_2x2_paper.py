"""Paper point-mass 2x2: {folded DDP, CT-SCvx} x {node-only, CTCS}.

One declared problem per (time mode, mesh): identical dynamics, objective,
input/speed constraints, initial guess and acceptance tolerances for all four
arms. CTCS arms use Elango et al. eq. (24c) per-interval budgets with the same
physical budget B and the same a-priori Theorem-14 tightening. The SCP arm is
OpenSCvx with constant proximal/penalty/cost weights, i.e. the CT-SCvx
prox-linear loop (fixed w_px, fixed l1 defect weight w_ep). Every returned
policy is graded by the exact rational replay in point_mass_certificate.
"""
import argparse
import hashlib
import json
import os
from concurrent.futures import ThreadPoolExecutor
from fractions import Fraction as F
from pathlib import Path
import subprocess
import sys
import time
import traceback

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from point_mass_certificate import certify, certify_input, flow, q  # noqa: E402

SCP = Path('/home/lab/projects/solver/SCP/OpenSCvx')
SCP_PYTHON = '/home/lab/miniforge3/envs/openscvx/bin/python'
DDP_BINARY = Path('/home/lab/mavros_ws/devel/.private/comando_planner/lib/comando_planner/point_mass_2x2_ddp')
G = 9.81
TAN20 = float(np.tan(np.deg2rad(20.)))
ARMS = ('ddp_node', 'ddp_ctcs', 'scp_node', 'scp_ctcs')


def protocol():
    return dict(
        name='point_mass_2x2_paper_v1',
        obstacle=dict(center=[2.85355, 2.14645, 5.5], radius=2.),
        glide_tan=1., accel_max=1.5*G, cone_deg=20., speed_max=10.,
        initial_state=[5., 5., 10., -.2, -.2, -.3], terminal_state=[0., 0., 1., 0., 0., 0.],
        objective='sum_k dt_k (1 + 1e-5 |a_k|^2)',
        # [3, 30] s total, i.e. dt_k in [3/N, 30/N]: the bounds of the original 2x2 for both solvers.
        fixed_tf_s=4., free_tf_bounds_s=[3., 30.], free_initial_tf_s=4.,
        meshes=[6, 10, 20, 40], time_modes=['fixed', 'free'],  # N=100 dropped 2026-10-20
        # Elango eq. (24c): one budget per interval, physical time measure.
        # B from Theorem 14: delta=5 mm is sufficient iff B <= (delta/R)^3 R/(4 omega) = 5.5e-10
        # (glide, omega=sqrt(2)*10 m/s); B=1e-10 keeps ~5x margin and is 10x less ill-scaled than 1e-11.
        budget_per_interval=1e-10, residual_scale_m=2.,
        density='sum_i [(g_i + delta_i)/R]_+^2, g = (obstacle, glide residual) in metres',
        tightening_m=dict(obstacle=.005, glide=.005),
        seed='straight line p0->pf at constant velocity (pf-p0)/4, node 0 = x0, hover a=(0,0,g), dt=4/N',
        node_arms='obstacle and glide residual <= 0 at every node, untightened (node-only baseline)',
        acceptance=dict(terminal_error_inf_m=1e-6, input_tolerance_m_s2=1e-8, speed_certificate='strict',
                        node_arms='obstacle and glide residual <= 1e-6 m at every propagated node',
                        ctcs_physical='certified pass on propagated policy',
                        ctcs_integral='certified I_k <= B on propagated policy'),
        ddp_profile=json.loads((HERE/'point_mass_ddp_frozen_profile.json').read_text()),
        ddp_row='linear',  # folded row coordinates: 'linear' I/B-1 or 'log' log(1+I/B)-log 2
        scp_profile=None)


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':')).encode()).hexdigest()


def theorem14_tightening(P):
    """A-priori sufficient margins (Elango Thm 14 / Rem 15) from the speed bound.

    v=(g+delta)/R has |dv/dt| <= omega/R with omega=V (obstacle) and sqrt(1+tan^2) V
    (glide). Integral <= B then gives [g+delta]_+ <= R (4 B omega/R)^(1/3) whenever
    B <= (omega/R)^2 dt_min^3 / 4.
    """
    B, R, V = P['budget_per_interval'], P['residual_scale_m'], P['speed_max']
    dt_min = min(P['fixed_tf_s']/max(P['meshes']), P['free_tf_bounds_s'][0]/max(P['meshes']))
    rows = {}
    for name, omega in (('obstacle', V), ('glide', float(np.sqrt(1+P['glide_tan']**2))*V)):
        L = omega/R
        rows[name] = dict(omega_m_s=omega, required_m=R*(4*B*L)**(1/3),
                          configured_m=P['tightening_m'][name],
                          small_budget_condition=bool(B <= L*L*dt_min**3/4), dt_min_s=dt_min)
        rows[name]['sufficient'] = bool(rows[name]['small_budget_condition'] and rows[name]['configured_m'] >= rows[name]['required_m'])
        # Largest per-interval integral for which the configured margin still suffices.
        Bmax = (rows[name]['configured_m']/R)**3/(4*L)
        rows[name]['admissible_budget'] = Bmax if Bmax <= L*L*dt_min**3/4 else L*L*dt_min**3/4
    return rows


def seed(P, N, mode='fixed'):
    p0, pf = np.array(P['initial_state'][:3]), np.array(P['terminal_state'][:3])
    tf = P['fixed_tf_s'] if mode == 'fixed' else P['free_initial_tf_s']; v = (pf-p0)/tf
    X = [np.r_[p0+k/N*(pf-p0), v] for k in range(N+1)]
    X[0] = np.array(P['initial_state'], float)
    return dict(X=np.array(X).tolist(), U=[[0., 0., G]]*N, durations_s=[tf/N]*N)


# ---------------------------------------------------------------- DDP arms
def run_ddp(P, mode, arm, N, out):
    s = seed(P, N, mode); fixed = mode == 'fixed'; prof = P['ddp_profile']
    U = np.asarray(s['U'])
    if not fixed: U = np.c_[U, s['durations_s']]
    lo, hi = P['free_tf_bounds_s']
    head = [N, int(fixed), prof['optimizer_rk4_substeps'], P['budget_per_interval'], P['fixed_tf_s'], lo, hi,
            P['tightening_m']['obstacle'], P['tightening_m']['glide'], int(arm == 'ctcs'), P['speed_max'],
            int(P.get('ddp_row', 'linear') == 'log')]
    fmt = lambda vals: ' '.join('%.17g' % float(v) for v in vals)
    text = fmt(head)+'\n'+fmt(np.ravel(s['X']))+'\n'+fmt(U.ravel())+'\n'
    for k, v in prof['settings'].items():
        if k == 'feasibility_ms_exact_rows': continue
        text += f"{k} {int(v) if isinstance(v, bool) else v}\n"
    (out/'case.txt').write_text(text)
    t0 = time.perf_counter()
    r = subprocess.run([str(DDP_BINARY), str(out/'case.txt'), str(out/'ddp')], capture_output=True, text=True, timeout=1800)
    (out/'stderr.log').write_text(r.stderr)
    if r.returncode: raise RuntimeError(f'ddp exit {r.returncode}: {r.stderr[-500:]}')
    native = json.loads(r.stdout.strip().splitlines()[-1])
    for k, v in prof['settings'].items():
        if k != 'feasibility_ms_exact_rows' and float(native['settings'][k]) != float(v):
            raise ValueError(f'runtime DDP setting {k} differs from frozen profile')
    Uo = np.asarray(native['U'])
    return dict(X=native['X'], U=Uo[:, :3].tolist(),
                durations_s=(s['durations_s'] if fixed else Uo[:, 3].tolist()),
                native_status=native['termination'], native_iterations=native['iterations'],
                native_optimality=native['optimality'], solve_seconds=native['solve_seconds'],
                wall_seconds=time.perf_counter()-t0,
                numerical_intervals=[(c+1)*P['budget_per_interval'] for c in native['normalized_interval_rows']])


# ---------------------------------------------------------------- SCP arms
def run_scp(P, mode, arm, N, out):
    os.environ.setdefault('JAX_PLATFORMS', 'cpu'); os.environ.setdefault('OMP_NUM_THREADS', '1')
    # One XLA thread per worker: parallel grid jobs must not oversubscribe the CPU.
    os.environ.setdefault('XLA_FLAGS', '--xla_cpu_multi_thread_eigen=false intra_op_parallelism_threads=1')
    sys.path.insert(0, str(SCP)); import openscvx as ox; import diffrax
    cfg = P['scp_profile']; s = seed(P, N, mode); n = N+1; fixed = mode == 'fixed'
    X = np.asarray(s['X']); U = np.asarray(s['U']); dt = np.asarray(s['durations_s']); tf = float(dt.sum())
    o = np.asarray(P['obstacle']['center']); R = P['obstacle']['radius']; B = P['budget_per_interval']
    pos = ox.State('position', (3,)); vel = ox.State('velocity', (3,)); obj = ox.State('objective', (1,))
    for st, b in ((pos, 1e3), (vel, 1e3)):
        st.min = [-b]*3; st.max = [b]*3; st.scaling_min = [-10.]*3; st.scaling_max = [10.]*3
    pos.initial = X[0, :3]; pos.final = P['terminal_state'][:3]; pos.guess = X[:, :3]
    vel.initial = X[0, 3:]; vel.final = P['terminal_state'][3:]; vel.guess = X[:, 3:]
    obj.min = [0.]; obj.max = [100.]; obj.scaling_min = [0.]; obj.scaling_max = [10.]
    obj.initial = [0.]; obj.final = [ox.Minimize(tf)]
    obj.guess = np.r_[0., np.cumsum(dt*(1.+1e-5*np.sum(U*U, axis=1)))][:, None]
    a = ox.Control('acceleration', (3,), parameterization='ZOH')
    a.min = [-P['accel_max']]*2+[0.]; a.max = [P['accel_max']]*3; a.guess = np.vstack([U, U[-1]])
    lo, hi = P['free_tf_bounds_s']
    clock = ox.Time(initial=0., final=tf if fixed else ox.Free(tf), min=0., max=hi,
                    guess=np.r_[0., np.cumsum(dt)][:, None], time_dilation_min=tf if fixed else lo,
                    time_dilation_max=tf if fixed else hi, time_dilation_guess=np.r_[N*dt, N*dt[-1]][:, None])
    states = [pos, vel, obj]
    dyn = {'position': vel, 'velocity': a-np.array([0., 0., G]), 'objective': 1.+1e-5*(a@a)}
    cons = [(ox.linalg.Norm(a) <= P['accel_max']).convex(),
            (ox.linalg.Norm(a[:2]) <= TAN20*a[2]).convex(),
            (ox.linalg.Norm(vel) <= P['speed_max']).convex()]
    obs_res = R-ox.linalg.Norm(pos-o); glide_res = ox.Sqrt(pos[0]**2+pos[1]**2+1e-12)-P['glide_tan']*pos[2]
    if arm == 'ctcs':
        y = ox.State('y', (1,)); y.min = [-1e3]; y.max = [1e3]; y.scaling_min = [-1.]; y.scaling_max = [1.]
        y.initial = [0.]; y.final = [ox.Free(0.)]; y.guess = np.zeros((n, 1)); states.append(y)
        # y measures ys*I (default ys=1/B: y in budget units, row y_{k+1}-y_k <= 1). y_scale
        # is a change of variables only; the feasible set {I_k <= B} is unchanged.
        ys = cfg.get('y_scale', 1./B); k = np.sqrt(ys)/R if 'y_scale' in cfg else 1./(R*np.sqrt(B))
        dyn['y'] = (ox.Square(ox.PositivePart(k*(obs_res+P['tightening_m']['obstacle']))) +
                    ox.Square(ox.PositivePart(k*(glide_res+P['tightening_m']['glide']))))
        cons += [(y.at(j+1)-y.at(j) <= (B*ys if 'y_scale' in cfg else 1.)).convex() for j in range(N)]
    else:
        cons += [(obs_res <= 0.).at(list(range(n))), (glide_res <= 0.).at(list(range(n)))]
    # Re-canonicalize every subproblem (ignore_dpp). Compiled DPP is ~6x faster per
    # iteration but, with the 1/B-scaled accumulator row, Clarabel reported an
    # infeasible CTCS subproblem that the non-DPP path solves; keep one mode for all.
    dpp = {'ignore_dpp': True}
    # Paper setting (cvx_backend='clarabel_dpp' with y_scale): with y = 1e5*I instead of I/B,
    # Clarabel solves the compiled DPP problem, so it is built once instead of every iteration.
    if cfg.get('cvx_backend') == 'clarabel_dpp': dpp = {'enforce_dpp': True}
    tol = cfg.get('cvx_tol', 1e-10)
    solver = {'cvx_solver': 'CLARABEL', 'solver_args': {'tol_gap_abs': tol, 'tol_gap_rel': tol, 'tol_feas': tol, 'max_iter': 300, **dpp}}
    # CT-SCvx penalizes defects AND nonconvex constraint buffers with one exact-penalty
    # weight w_ep. ConstantProximalWeight never adapts lam_vb (default 0), so set it.
    lam_vb = cfg.get('lam_vb', cfg['lam_vc'])
    ode = {'rtol': cfg['ode_tol'], 'atol': cfg['ode_tol'], 'num_substeps': 30,
           'stepsize_controller': diffrax.PIDController(rtol=cfg['ode_tol'], atol=cfg['ode_tol'],
                                                        dtmax=1./(N*cfg['max_step_divisor']))}
    problem = ox.Problem(states=states, controls=[a], time=clock, N=n, float_dtype='float64',
        dynamics=dyn, constraints=cons,
        algorithm={'k_max': cfg['max_iter'], 'lam_cost': cfg['lam_cost'], 'lam_prox': cfg['lam_prox'],
                   'lam_vc': cfg['lam_vc'], 'lam_vb': lam_vb, 'ep_tr': cfg['ep_tr'], 'ep_vc': cfg['ep_vc'], 'ep_vb': cfg['ep_vb'],
                   # 'al': OpenSCvx's default acceptance-ratio weight update (Elango Remark 24 /
                   # Fig. 3 "Update Hyperparameters"); default stays the constant-weight prox-linear loop.
                   'autotuner': ox.AugmentedLagrangian() if cfg.get('autotuner') == 'al' else ox.ConstantProximalWeight()},
        discretizer={'type': 'VectorizeDiscretizeLinearize', 'ode_solver': 'Dopri8', 'diffrax_kwargs': ode},
        solver=solver)
    if not cfg.get('printing', True): problem.settings.dev.printing = False  # per-iteration console table
    problem.initialize(); sl = problem.slices
    def physical(st):
        x = np.asarray(st.x); u = np.asarray(st.u)
        return (np.c_[x[:, sl['position']], x[:, sl['velocity']]], u[:-1, sl['acceleration']],
                u[:-1, sl['_time_dilation']].ravel()/N)
    X0, U0, d0 = physical(problem.state)
    if not (np.allclose(X0, s['X'], atol=1e-12) and np.allclose(U0, s['U'], atol=1e-12) and np.allclose(d0, dt, atol=1e-12)):
        raise ValueError('SCP did not receive the shared seed')
    t0 = time.perf_counter(); result = problem.solve(); wall = time.perf_counter()-t0
    st = problem.state; Xo, Uo, do = physical(st)
    extra = dict(lam_vb_used=lam_vb)
    if arm == 'ctcs':  # optimized y increments vs the actual numerical map, in physical units
        import jax
        rawX, rawU = np.asarray(st.x), np.asarray(st.u); yc = sl['y'].start
        prop = np.asarray(jax.jit(problem.discretizer.get_solver(problem.lowered.dynamics, problem.settings))(rawX, rawU, problem._parameters)[3])
        ys = cfg.get('y_scale', 1./B)
        if 'y_scale' not in cfg: inc = np.diff(rawX[:, yc])*B; native = (prop[:, yc]-rawX[:-1, yc])*B
        else: inc = np.diff(rawX[:, yc])/ys; native = (prop[:, yc]-rawX[:-1, yc])/ys
        extra.update(optimized_accumulator_increments=inc.tolist(), numerical_intervals=native.tolist(),
                     accumulator_defects=(inc-native).tolist())
    k_hist = len(getattr(problem.history, 'J_tr', []) or [])
    return dict(X=Xo.tolist(), U=Uo.tolist(), durations_s=do.tolist(),
                native_status='converged' if result.converged else 'iteration_limit',
                native_iterations=int(np.asarray(st.k)) if hasattr(st, 'k') else k_hist,
                native_J_tr=float(st.J_tr), native_J_vc=float(st.J_vc), native_J_vb=float(st.J_vb),
                solve_seconds=wall, wall_seconds=wall, cvxpy_mode=next(iter(dpp)), **extra)


# ---------------------------------------------------------------- audit
def audit(P, mode, arm, N, pol):
    X = np.asarray(pol['X'], float); U = np.asarray(pol['U'], float); dt = np.asarray(pol['durations_s'], float)
    if X.shape != (N+1, 6) or U.shape != (N, 3) or dt.shape != (N,) or not all(np.isfinite(a).all() for a in (X, U, dt)) or (dt <= 0).any():
        return dict(status=dict(valid=False))
    B = P['budget_per_interval']; m = (P['tightening_m']['obstacle'], P['tightening_m']['glide'])
    prop = list(map(q, P['initial_state'])); rows = []; gaps = []
    for k in range(N):
        c = certify(prop, U[k], dt[k], B, margins=m, obstacle=tuple(P['obstacle']['center']),
                    radius=P['obstacle']['radius'], glide_tan=P['glide_tan'])
        gaps.append(float(np.max(np.abs(np.asarray(flow(X[k], U[k], dt[k]), float)-X[k+1]))))
        rows.append(dict(interval=k, physical=c['physical_status'], integral=c['integral_status'],
                         integral_over_B=[float(F(c['integral']['lower_rational'])/q(B)), float(F(c['integral']['upper_rational'])/q(B))],
                         residual_max_m=[[float(F(r['lower_rational'])), float(F(r['upper_rational']))] for r in c['physical_residual_max_m']],
                         speed_upper=float(F(c['speed_upper_m_s']['upper_rational'])), input=certify_input(U[k])['status'],
                         input_residual_upper=max(float(F(v['upper_rational'])) for v in certify_input(U[k])['residuals'].values())))
        prop = flow(prop, U[k], dt[k])
    terminal = float(np.max(np.abs(np.asarray(prop, float)-P['terminal_state'])))
    comb = lambda v: 'fail' if 'fail' in v else 'uncertified' if 'uncertified' in v else 'pass'
    speed_ok = max(r['speed_upper'] for r in rows) <= P['speed_max']
    lo, hi = P['free_tf_bounds_s']
    time_ok = abs(dt.sum()-P['fixed_tf_s']) <= 1e-9 if mode == 'fixed' else bool((dt >= lo/N-1e-12).all() and (dt <= hi/N+1e-12).all())
    st = dict(valid=True, physical=comb([r['physical'] for r in rows]), integral=comb([r['integral'] for r in rows]),
              input=comb([r['input'] for r in rows]), speed=bool(speed_ok), time_bounds=bool(time_ok),
              terminal=bool(terminal <= P['acceptance']['terminal_error_inf_m']))
    # Inputs: declared tolerance 1e-8 m/s^2 (conic solvers meet cones to ~1e-10); strict status kept as st['input'].
    st['input_within_tolerance'] = bool(max(r['input_residual_upper'] for r in rows) <= 1e-8)
    st['primal_feasible'] = st['input_within_tolerance'] and speed_ok and time_ok and st['terminal']
    st['continuous_time_safe'] = st['primal_feasible'] and st['physical'] == 'pass'
    # Node-only arms must satisfy their own constraints at every node of the propagated policy.
    o = np.asarray(P['obstacle']['center']); R = P['obstacle']['radius']; nodes = [np.asarray(P['initial_state'], float)]
    for k in range(N): nodes.append(np.asarray(flow(nodes[-1], U[k], dt[k]), float))
    pn = np.array(nodes)[:, :3]
    nodal_obs = float(np.max(R-np.linalg.norm(pn-o, axis=1)))
    nodal_glide = float(np.max(np.sqrt(pn[:, 0]**2+pn[:, 1]**2+1e-12)-P['glide_tan']*pn[:, 2]))
    st['nodal_constraints'] = bool(max(nodal_obs, nodal_glide) <= 1e-6)
    # B is the solver's target; Theorem 14 only needs I_k <= B_adm (the budget the configured
    # margins admit). At an active row I_k = B the certified enclosure straddles B, so the
    # gate is the theorem's budget, with B < B_adm as the designed tolerance gap.
    B_adm = min(r['admissible_budget'] for r in theorem14_tightening(P).values())
    st['integral_theorem14'] = bool(max(r['integral_over_B'][1] for r in rows)*B <= B_adm)
    st['accepted'] = st['primal_feasible'] and (st['physical'] == 'pass' and st['integral_theorem14'] if arm == 'ctcs' else st['nodal_constraints'])
    worst = max(r['residual_max_m'][0][1] for r in rows)  # obstacle residual upper bound, m
    worst_glide = max(r['residual_max_m'][1][1] for r in rows)
    return dict(status=st, objective=float(np.sum(dt*(1+1e-5*np.sum(U*U, axis=1)))), final_time_s=float(dt.sum()),
                terminal_error_inf=terminal, shooting_gap_inf=max(gaps), max_obstacle_penetration_upper_m=worst,
                max_glide_excess_upper_m=worst_glide, max_integral_over_B=max(r['integral_over_B'][1] for r in rows), admissible_budget_over_B=B_adm/B,
                max_speed=max(r['speed_upper'] for r in rows), nodal_obstacle_residual_m=nodal_obs,
                nodal_glide_residual_m=nodal_glide, intervals=rows)


# ---------------------------------------------------------------- orchestration
def worker(root, mode, solver_arm, N):
    P = json.loads((root/'protocol.json').read_text())
    solver, arm = solver_arm.split('_'); out = root/f'{mode}_{solver_arm}_N{N}'; out.mkdir(exist_ok=False)
    try:
        pol = (run_ddp if solver == 'ddp' else run_scp)(P, mode, arm, N, out)
        (out/'policy.json').write_text(json.dumps(pol)+'\n')
        (out/'audit.json').write_text(json.dumps(audit(P, mode, arm, N, pol), indent=1)+'\n')
        (out/'status.json').write_text(json.dumps(dict(status='completed'))+'\n')
    except Exception:
        (out/'error.txt').write_text(traceback.format_exc())
        (out/'status.json').write_text(json.dumps(dict(status='failed'))+'\n'); raise


def launch(root, jobs, workers):
    def one(job):
        mode, arm, N = job
        py = SCP_PYTHON if arm.startswith('scp') else sys.executable
        log = root/f'{mode}_{arm}_N{N}.log'
        with log.open('w') as fh:
            try: code = subprocess.run([py, __file__, '--worker', str(root), mode, arm, str(N)], stdout=fh, stderr=subprocess.STDOUT, timeout=7200).returncode
            except subprocess.TimeoutExpired: code = 'timeout'
        print(mode, arm, N, 'exit', code, flush=True); return dict(mode=mode, arm=arm, N=N, exit=code)
    with ThreadPoolExecutor(workers) as ex: return list(ex.map(one, jobs))


def prepare(root, scp_profile, angle=None, ddp_settings=None, ddp_row='linear', budget=None, free_tf=None, extra=None, diagnostic=False):
    root.mkdir(parents=True, exist_ok=False); P = protocol(); P['scp_profile'] = scp_profile; P['ddp_row'] = ddp_row
    if budget is not None: P['budget_per_interval'] = budget
    if free_tf is not None: P['free_initial_tf_s'] = free_tf
    for key, val in (extra or {}).items():
        if val is None: continue
        if key == 'free_bounds': P['free_tf_bounds_s'] = list(val)
        if key == 'tightening': P['tightening_m'] = dict(obstacle=val[0], glide=val[1])
        if key == 'substeps': P['ddp_profile']['optimizer_rk4_substeps'] = val
    if ddp_settings:  # calibrated override of the frozen profile's settings (recorded)
        P['ddp_profile']['settings'].update(ddp_settings); P['ddp_profile']['profile_id'] += '+calibrated'
        P['ddp_profile']['overrides'] = ddp_settings
    if angle is not None:  # held-out start: rotate position and horizontal velocity about the obstacle
        from point_mass_paper_protocol import approach_case
        P['initial_state'] = approach_case('rotated', angle)['initial_state']; P['start_angle_rad'] = angle
    P['theorem14'] = theorem14_tightening(P)
    if not diagnostic and not all(r['sufficient'] for r in P['theorem14'].values()): raise ValueError('declared tightening is not sufficient a priori')
    P['diagnostic'] = diagnostic
    for label, repo in (('ddp_solver', HERE.parents[3]/'ALIPDDP-main'), ('openscvx', SCP), ('planner', HERE.parents[2])):
        P.setdefault('source_pins', {})[label] = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=repo, text=True).strip()
    P['ddp_binary_sha256'] = hashlib.sha256(DDP_BINARY.read_bytes()).hexdigest()
    (root/'protocol.json').write_text(json.dumps(P, indent=1)+'\n')
    (root/'protocol.sha256').write_text(digest(P)+'\n')
    return P


SCP_DEFAULT = dict(max_iter=300, lam_prox=1., lam_vc=1e3, lam_cost=1., ep_tr=1e-6, ep_vc=1e-8, ep_vb=1e-8,
                   ode_tol=1e-10, max_step_divisor=32, algorithm='CT-SCvx prox-linear: ConstantProximalWeight')

if __name__ == '__main__':
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--worker', nargs=4)
    ap.add_argument('--reaudit', type=Path, help='recompute audit.json for every saved policy under a grid')
    ap.add_argument('--run', type=Path, help='new output directory')
    ap.add_argument('--scp-profile', type=json.loads, default=None)
    ap.add_argument('--modes', nargs='+', default=['fixed', 'free'])
    ap.add_argument('--meshes', nargs='+', type=int, default=None)
    ap.add_argument('--arms', nargs='+', default=list(ARMS))
    ap.add_argument('--workers', type=int, default=6)
    ap.add_argument('--angle', type=float, default=None, help='rotate the start about the obstacle (rad)')
    ap.add_argument('--ddp-settings', type=json.loads, default=None)
    ap.add_argument('--ddp-row', choices=['linear', 'log'], default='linear')
    ap.add_argument('--budget', type=float, default=None, help='per-interval physical budget B')
    ap.add_argument('--free-initial-tf', type=float, default=None)
    ap.add_argument('--free-bounds', type=float, nargs=2, default=None, help='total free tf bounds [s]')
    ap.add_argument('--tightening', type=float, nargs=2, default=None, help='obstacle glide margins [m]')
    ap.add_argument('--substeps', type=int, default=None, help='DDP RK4 substeps per interval')
    ap.add_argument('--diagnostic', action='store_true', help='skip the a-priori tightening check (not a paper run)')
    ap.add_argument('--prepare-only', action='store_true', help='write protocol.json; jobs are launched by a pool')
    a = ap.parse_args()
    if a.reaudit:
        for pol in sorted(q for q in a.reaudit.glob('*/*_N*/policy.json') if '.stale' not in q.parent.name):
            P = json.loads((pol.parents[1]/'protocol.json').read_text()); mode, solver, arm, n = pol.parent.name.split('_')
            (pol.parent/'audit.json').write_text(json.dumps(audit(P, mode, arm, int(n[1:]), json.loads(pol.read_text())), indent=1)+'\n')
    elif a.worker: worker(Path(a.worker[0]), a.worker[1], a.worker[2], int(a.worker[3]))
    elif a.run:
        P = prepare(a.run, a.scp_profile or SCP_DEFAULT, a.angle, a.ddp_settings, a.ddp_row, a.budget, a.free_initial_tf,
                    dict(free_bounds=a.free_bounds, tightening=a.tightening, substeps=a.substeps), a.diagnostic)
        jobs = [(m, arm, N) for N in sorted(a.meshes or P['meshes'], reverse=True) for m in a.modes for arm in a.arms]
        if not a.prepare_only: print(json.dumps(launch(a.run, jobs, a.workers)))
