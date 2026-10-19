"""Audit the exact upper-triangle matrix used by failed DDP Cholesky calls."""
import argparse
import json
from pathlib import Path
import numpy as np


def upper_matrix(a):
    a = np.asarray(a)
    return np.triu(a)+np.triu(a, 1).T


def analyze(log):
    records = []
    for line in log.read_text().splitlines():
        if not line.startswith('[QUU_MATRIX] '):
            continue
        row = json.loads(line.split(' ', 1)[1])
        base, factored = np.array(row['base']), np.array(row['factored'])
        average = .5*(base+base.T)
        components = {name: float(np.max(np.abs(np.array(row[name])-np.array(row[name]).T)))
                      for name in ('uncondensed', 'ipm', 'alm') if name in row}
        try:
            np.linalg.cholesky(upper_matrix(factored))
            reproduced = False
        except np.linalg.LinAlgError:
            reproduced = True
        records.append(dict(stage=row['stage'],
            asymmetry_inf=float(np.max(np.abs(base-base.T))),
            averaged_min_eigenvalue=float(np.linalg.eigvalsh(average)[0]),
            upper_min_eigenvalue=float(np.linalg.eigvalsh(upper_matrix(base))[0]),
            actual_factored_min_eigenvalue=float(np.linalg.eigvalsh(upper_matrix(factored))[0]),
            numpy_cholesky_failure=reproduced, component_asymmetry=components))
    return dict(failures_captured=len(records),
                failures_reproduced=sum(r['numpy_cholesky_failure'] for r in records),
                averaged_positive_but_upper_negative=sum(r['averaged_min_eigenvalue'] > 0 and
                                                         r['upper_min_eigenvalue'] < 0 for r in records),
                records=records)


def compare_traces(reference, actual):
    a = [json.loads(line) for line in reference.read_text().splitlines()]
    b = [json.loads(line) for line in actual.read_text().splitlines()]
    same = len(a) == len(b) and all(
        all(x[key] == y[key] for key in ('iteration', 'X', 'U', 'mu', 'rho', 'opterror'))
        for x, y in zip(a, b))
    return dict(reference_records=len(a), actual_records=len(b),
                identical_optimizer_records_excluding_timing=same)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--log', required=True, type=Path)
    parser.add_argument('--reference-trace', required=True, type=Path)
    parser.add_argument('--trace', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    result = analyze(args.log)
    result['trace_comparison'] = compare_traces(args.reference_trace, args.trace)
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    print(json.dumps({k: v for k, v in result.items() if k != 'records'}, indent=2))
