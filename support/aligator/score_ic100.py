#!/usr/bin/env python3
"""Score landing_ic100 runs with the paper's rule (Sec. VII-A), importing the
original single-horizon certificate and consequence audit read-only.

rho = max(pos_err/0.10 m, vel_err/0.30 m/s, hard_path/1e-2,
          triggered consequence excess on dense samples / medium tolerance)
success iff rho <= 1.

  score_ic100.py RUN_DIR [RUN_DIR ...]
"""
import csv
import statistics
import sys
from pathlib import Path

CASE = Path('/home/lab/projects/solver/DDP/ALIPDDP-main/benchmark/results/stc/comparisons/quad_three_way')
sys.path.insert(0, str(CASE / 'v1/scripts/core'))
sys.path.insert(0, str(CASE / 'v1/scripts/sh'))
import summarize_quad_sh_own_constraints as ownc  # noqa: E402
from sh_certification import LAND_POS_CAP, LAND_VEL_CAP, PATH_TOL, certify_csv  # noqa: E402

ownc.set_consequence_margin('medium')


def score(run_dir):
    run_dir = Path(run_dir)
    out = []
    for r in csv.DictReader(open(run_dir / 'results.csv')):
        d = run_dir / r['artifact_dir']
        cert = certify_csv(str(d / 'nodes.csv'), str(d / 'dense.csv'))
        if not cert.get('finite'):
            out.append(dict(r, rho=float('inf'), dominant='nonfinite'))
            continue
        rows = list(csv.DictReader(open(d / 'dense.csv')))
        terms = {'terminal position': cert['pos_err'] / LAND_POS_CAP,
                 'terminal velocity': cert['vel_err'] / LAND_VEL_CAP,
                 'hard path': cert['path_excess'] / PATH_TOL}
        terms.update({n: e / ownc.constraint_tolerance(n)
                      for (n, _), e in ownc.cstc_constraints(rows).items() if n.startswith('triggered')})
        dom = max(terms, key=terms.get)
        out.append(dict(r, rho=terms[dom], dominant=dom, pos_err=cert['pos_err']))
    return out


if __name__ == '__main__':
    for run in sys.argv[1:]:
        rec = score(run)
        ok = [x for x in rec if x['rho'] <= 1]
        fails = {}
        for x in rec:
            if x['rho'] > 1:
                fails[x['dominant']] = fails.get(x['dominant'], 0) + 1
        t = sorted(float(x['solver_only_s']) for x in rec)
        print(f"{run}: success {len(ok)}/{len(rec)}  solve median {statistics.median(t):.2f} s "
              f"p95 {t[int(0.95 * (len(t) - 1))]:.2f} s  failures {fails}")
