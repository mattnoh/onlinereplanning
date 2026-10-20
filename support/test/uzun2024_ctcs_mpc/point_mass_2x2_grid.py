"""Per-cell tuning grid for the paper 2x2: identical 16-candidate grids per solver.

DDP: base {frozen profile, original example defaults} x mu {1e-3, 1e-1} x rho=rhoT {1, 100}
     x reg2_min {1e-6, 1}.
SCP (CT-SCvx, constant weights): lam_prox {0.03, 0.1, 0.3, 1} x lam_vc {1e3, 1e4} x lam_cost {1, 10}.
Free-time SCP extension (scp_ar_*, free mode only, both arms): the v2 ratios lam_cost/lam_prox
(1..333) lie far outside the authors' CT-SCvx runs (w_px=1, cost_factor=0.003-0.01) and free
time never converged there; lam_prox {1, 3, 10} x lam_cost/lam_prox {0.1, 0.3, 1}, lam_vc 1e3,
1000 iterations.
All profiles share one protocol (point_mass_2x2_paper.protocol); every job runs in a
single worker pool, longest meshes first. Selection happens afterwards
(point_mass_2x2_paper_figure.py).
"""
import argparse
import itertools
import json
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

HERE = Path(__file__).resolve().parent
PY = '/home/lab/miniforge3/envs/openscvx/bin/python'
HARNESS = HERE/'point_mass_2x2_paper.py'
SCP_BASE = dict(max_iter=300, ep_tr=1e-6, ep_vc=1e-9, ep_vb=1e-8, ode_tol=1e-10, max_step_divisor=32,
                algorithm='CT-SCvx prox-linear: ConstantProximalWeight')
DDP_BASES = {'frozen': dict(max_iter=3000),
             'example': dict(max_iter=3000, mu_min=1e-8, rho_max=1e8, max_inner_iter=50)}


def profiles():
    out = []
    for base, mu, rho, reg2 in itertools.product(DDP_BASES, (1e-3, 1e-1), (1., 100.), (1e-6, 1.)):
        out.append((f'ddp_{base}_mu{mu:g}_rho{rho:g}_reg{reg2:g}', 'ddp',
                    dict(DDP_BASES[base], mu=mu, muT=mu, rho=rho, rhoT=rho, reg2_min=reg2)))
    for lp, vc, lc in itertools.product((0.03, 0.1, 0.3, 1.), (1e3, 1e4), (1., 10.)):
        out.append((f'scp_p{lp:g}_vc{vc:g}_c{lc:g}', 'scp', dict(SCP_BASE, lam_prox=lp, lam_vc=vc, lam_cost=lc)))
    for lp, r in itertools.product((1., 3., 10.), (0.1, 0.3, 1.)):
        out.append((f'scp_ar_p{lp:g}_c{lp*r:g}', 'scp', dict(SCP_BASE, max_iter=1000, lam_prox=lp, lam_vc=1e3, lam_cost=lp*r),
                    ('free',)))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--out', type=Path, required=True); ap.add_argument('--workers', type=int, default=18)
    ap.add_argument('--solvers', nargs='+', default=['ddp', 'scp'])
    a = ap.parse_args(); a.out.mkdir(parents=True, exist_ok=False)
    jobs = []
    for name, solver, cfg, *modes in profiles():
        if solver not in a.solvers: continue
        root = a.out/name
        flag = ['--ddp-settings', json.dumps(cfg)] if solver == 'ddp' else ['--scp-profile', json.dumps(cfg)]
        subprocess.run([PY, str(HARNESS), '--run', str(root), '--prepare-only', *flag], check=True)
        for N in (40, 20, 10, 6):  # N=100 dropped (user decision)
            for mode in (modes[0] if modes else ('fixed', 'free')):
                for arm in ('node', 'ctcs'):
                    jobs.append((N, solver, root, mode, f'{solver}_{arm}'))
    jobs.sort(key=lambda j: (-j[0], j[1] != 'scp'))  # long SCP meshes first

    def one(job):
        N, solver, root, mode, arm = job
        with (root/f'{mode}_{arm}_N{N}.log').open('w') as fh:
            try: code = subprocess.run([PY, str(HARNESS), '--worker', str(root), mode, arm, str(N)],
                                       stdout=fh, stderr=subprocess.STDOUT, timeout=7200).returncode
            except subprocess.TimeoutExpired: code = 'timeout'
        print(root.name, mode, arm, N, code, flush=True); return code
    with ThreadPoolExecutor(a.workers) as ex: list(ex.map(one, jobs))
    print('DONE', flush=True)


if __name__ == '__main__':
    sys.exit(main())
