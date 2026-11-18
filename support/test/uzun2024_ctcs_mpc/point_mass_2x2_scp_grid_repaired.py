"""Rerun the paper_2x2 per-cell SCP grid with the repaired backend, one process at a time.

Every SCP profile directory of the source grid is copied with its protocol unchanged
except scp_profile += REPAIR (accumulator y = 1e5*I, compiled DPP with Clarabel at the
unchanged 1e-10 tolerance, OpenSCvx acceptance-ratio weight update, printing off); every N<=40 cell that profile ran is rerun and
graded by the unchanged audit(). DDP is not rerun. Resumable: finished cells are skipped.
Usage: python point_mass_2x2_scp_grid_repaired.py SRC_GRID OUT_GRID
"""
import json
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
SCP_PYTHON = '/home/lab/miniforge3/envs/openscvx/bin/python'
# y=I/B (1e10 scale) made Clarabel fail on compiled DPP, so the historical runs re-canonicalized
# every iteration; y=1e5*I is a change of variables with the same feasible set. 'al' is
# OpenSCvx's acceptance-ratio weight update (Elango Remark 24) in place of constant weights.
REPAIR = dict(cvx_backend='clarabel_dpp', y_scale=1e5, autotuner='al', printing=False)


def main(src, out):
    out.mkdir(parents=True, exist_ok=True)
    for prof in sorted(p for p in src.iterdir() if p.is_dir() and p.name.startswith('scp_')):
        dst = out/prof.name; dst.mkdir(exist_ok=True)
        if not (dst/'protocol.json').exists():
            P = json.loads((prof/'protocol.json').read_text())
            P['repair'] = dict(source_protocol=str(prof/'protocol.json'), original_scp_profile=P['scp_profile'], **REPAIR)
            P['scp_profile'] = dict(P['scp_profile'], **REPAIR)
            (dst/'protocol.json').write_text(json.dumps(P, indent=1)+'\n')
        cells = sorted(d.name for d in prof.iterdir() if d.is_dir() and '.stale' not in d.name and int(d.name.rsplit('_N', 1)[1]) <= 40)
        for cell in cells:
            if (dst/cell/'status.json').exists(): continue
            if (dst/cell).exists(): subprocess.run(['mv', str(dst/cell), str(dst/f'{cell}.stale{int(time.time())}')], check=True)
            mode, solver, arm, n = cell.split('_')
            t = time.perf_counter()
            with (dst/f'{cell}.log').open('w') as fh:
                try:
                    code = subprocess.run(['nice', '-n', '19', SCP_PYTHON, str(HERE/'point_mass_2x2_paper.py'), '--worker', str(dst),
                                           mode, f'{solver}_{arm}', n[1:]], stdout=fh, stderr=subprocess.STDOUT, timeout=3600).returncode
                except subprocess.TimeoutExpired:
                    code = 'timeout'
            print(prof.name, cell, 'exit', code, '%.1fs' % (time.perf_counter()-t), flush=True)


if __name__ == '__main__':
    main(Path(sys.argv[1]), Path(sys.argv[2]))
