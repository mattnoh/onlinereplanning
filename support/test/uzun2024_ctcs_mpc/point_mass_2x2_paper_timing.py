"""Serial timing reruns of the selected per-cell settings (one job at a time).

Each selected (mode, solver, arm, N) is rerun `--repeats` times with the exact
protocol of its grid profile; the median solve time is reported. DDP time is the
C++ solve() wall time; SCP time is problem.solve() wall time after
problem.initialize() (JAX/CVXPY construction excluded, first-solve canonicalization
included). Different languages: report, do not convert into a method ranking.
"""
import argparse
import json
import shutil
import subprocess
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
SCP_PYTHON = '/home/lab/miniforge3/envs/openscvx/bin/python'


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--selection', type=Path, required=True)
    ap.add_argument('--out', type=Path, required=True)
    ap.add_argument('--repeats', type=int, default=3)
    ap.add_argument('--long-seconds', type=float, default=60., help='cells slower than this in the grid run once')
    a = ap.parse_args(); a.out.mkdir(parents=True, exist_ok=False)
    rows = [r for r in json.loads(a.selection.read_text()) if r['tier'] < 3]
    timing, records = {}, []
    for r in rows:
        key = f"{r['mode']}_{r['solver']}_{r['arm']}_N{r['N']}"; samples = []
        for rep in range(a.repeats if r['wall_seconds_in_grid'] <= a.long_seconds else 1):
            root = a.out/f'{key}_rep{rep}'; root.mkdir()
            shutil.copy2(Path(r['run']).parent/'protocol.json', root/'protocol.json')
            py = SCP_PYTHON if r['solver'] == 'scp' else sys.executable
            code = subprocess.run([py, str(HERE/'point_mass_2x2_paper.py'), '--worker', str(root), r['mode'],
                                   f"{r['solver']}_{r['arm']}", str(r['N'])], capture_output=True, text=True).returncode
            run = root/f"{r['mode']}_{r['solver']}_{r['arm']}_N{r['N']}"
            if code or not (run/'policy.json').exists(): samples.append(None); continue
            pol = json.loads((run/'policy.json').read_text())
            same = np.allclose(pol['U'], json.loads((Path(r['run'])/'policy.json').read_text())['U'], atol=1e-9)
            samples.append(pol['solve_seconds']); records.append(dict(cell=key, rep=rep, seconds=pol['solve_seconds'], same_policy=bool(same)))
        good = [s for s in samples if s is not None]
        if good: timing[key] = float(np.median(good))
        print(key, timing.get(key), flush=True)
    (a.out/'timing.json').write_text(json.dumps(timing, indent=1)+'\n')
    (a.out/'timing_records.json').write_text(json.dumps(records, indent=1)+'\n')


if __name__ == '__main__':
    main()
