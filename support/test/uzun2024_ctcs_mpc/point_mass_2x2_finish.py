"""Finish a per-cell grid: rerun interrupted / pre-fix jobs, re-audit, select and plot.

A job is rerun when its folder has no completed policy, or when it is an SCP
node-only run solved before lam_vb was set (no 'lam_vb_used' in its policy).
Incomplete folders are renamed *.stale_<n>, never deleted.
"""
import argparse
import json
import subprocess
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

HERE = Path(__file__).resolve().parent
PY = '/home/lab/miniforge3/envs/openscvx/bin/python'
HARNESS = HERE/'point_mass_2x2_paper.py'


def needs_rerun(run):
    pol = run/'policy.json'
    if not pol.exists(): return True
    return '_scp_node_' in run.name and 'lam_vb_used' not in json.loads(pol.read_text())


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--grid', type=Path, required=True); ap.add_argument('--out', type=Path, required=True)
    ap.add_argument('--workers', type=int, default=20)
    a = ap.parse_args()
    jobs = []
    for prof in sorted(p for p in a.grid.iterdir() if p.is_dir() and not p.name.endswith('_stale')):
        solver = prof.name.split('_')[0]
        for N in (40, 20, 10, 6):  # N=100 dropped (user decision): CTCS matters at coarse meshes
            for mode in ('fixed', 'free'):
                for arm in ('node', 'ctcs'):
                    run = prof/f'{mode}_{solver}_{arm}_N{N}'
                    if run.exists() and not needs_rerun(run): continue
                    if run.exists():
                        k = 0
                        while run.with_name(run.name+f'.stale_{k}').exists(): k += 1
                        run.rename(run.with_name(run.name+f'.stale_{k}'))
                    jobs.append((N, prof, mode, f'{solver}_{arm}'))
    jobs.sort(key=lambda j: -j[0])
    print('rerunning', len(jobs), flush=True)

    def one(j):
        N, prof, mode, arm = j
        with (prof/f'{mode}_{arm}_N{N}.finish.log').open('w') as fh:
            subprocess.run([PY, str(HARNESS), '--worker', str(prof), mode, arm, str(N)], stdout=fh, stderr=subprocess.STDOUT)
        print(prof.name, mode, arm, N, flush=True)
    with ThreadPoolExecutor(a.workers) as ex: list(ex.map(one, jobs))
    subprocess.run([PY, str(HARNESS), '--reaudit', str(a.grid)], check=True)
    subprocess.run([PY, str(HERE/'point_mass_2x2_paper_figure.py'), '--grid', str(a.grid), '--out', str(a.out)], check=True)
    print('DONE', flush=True)


if __name__ == '__main__':
    main()
