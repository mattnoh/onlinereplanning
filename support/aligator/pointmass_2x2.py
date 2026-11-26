#!/usr/bin/env python3
"""Point-mass 2x2 DDP arms (paper Table III / Fig. 3) on ProxDDP.

Uses the paper protocol, seed, run_ddp and exact rational audit of
support/test/uzun2024_ctcs_mpc/point_mass_2x2_paper.py unchanged (imported
read-only); only the DDP binary and its settings are this branch's.

  pointmass_2x2.py OUT_DIR [--modes fixed free] [--meshes 6 10 20 40] [--arms ddp_node ddp_ctcs]
                   [--substeps 4] [--settings '{"tol":1e-8,...}']
"""
import argparse
import json
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / 'test/uzun2024_ctcs_mpc'))
import point_mass_2x2_paper as pm  # noqa: E402

BINARY = Path('/home/lab/opt/build/planner_aligator/pm_2x2_aligator')
DEFAULT = dict(tol=1e-8, mu_init=1e-2, max_iters=2000, max_al_iters=100, exact_hessian=0, nonlinear_rollout=0,
               step_acceptance=0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('out', type=Path)
    ap.add_argument('--modes', nargs='+', default=['fixed', 'free'])
    ap.add_argument('--meshes', nargs='+', type=int, default=[6, 10, 20, 40])
    ap.add_argument('--arms', nargs='+', default=['ddp_node', 'ddp_ctcs'])
    ap.add_argument('--substeps', type=int, default=4)
    ap.add_argument('--settings', type=json.loads, default={})
    a = ap.parse_args()
    pm.DDP_BINARY = BINARY
    P = pm.protocol()
    P['ddp_profile'] = dict(profile_id='aligator_proxddp', optimizer_rk4_substeps=a.substeps,
                            settings=dict(DEFAULT, **a.settings))
    P['theorem14'] = pm.theorem14_tightening(P)
    a.out.mkdir(parents=True, exist_ok=False)
    (a.out / 'protocol.json').write_text(json.dumps(P, indent=1) + '\n')
    rows = []
    for mode in a.modes:
        for N in a.meshes:
            for arm_name in a.arms:
                arm = arm_name.split('_')[1]
                out = a.out / f'{mode}_{arm_name}_N{N}'
                out.mkdir()
                t0 = time.perf_counter()
                try:
                    pol = pm.run_ddp(P, mode, arm, N, out)
                    au = pm.audit(P, mode, arm, N, pol)
                except Exception as e:  # recorded, not hidden
                    (out / 'error.txt').write_text(repr(e))
                    rows.append(dict(mode=mode, arm=arm_name, N=N, error=repr(e)[:120]))
                    print(mode, arm_name, N, 'ERROR', repr(e)[:120], flush=True)
                    continue
                (out / 'policy.json').write_text(json.dumps(pol) + '\n')
                (out / 'audit.json').write_text(json.dumps(au, indent=1) + '\n')
                st = au['status']
                row = dict(mode=mode, arm=arm_name, N=N, accepted=st['accepted'], physical=st['physical'],
                           primal_feasible=st['primal_feasible'], terminal_error=au['terminal_error_inf'],
                           clearance_mm=-1e3 * au['max_obstacle_penetration_upper_m'],
                           min_clearance_mm=-1e3 * max(au['max_obstacle_penetration_upper_m'], au['max_glide_excess_upper_m']),
                           tf=au['final_time_s'], objective=au['objective'], solve_s=pol['solve_seconds'],
                           iterations=pol['native_iterations'], status=pol['native_status'],
                           wall_s=time.perf_counter() - t0)
                rows.append(row)
                print(f"{mode:5} {arm_name:8} N={N:2}  clear {row['clearance_mm']:8.2f} mm  accepted {row['accepted']} "
                      f"phys {row['physical']} term {row['terminal_error']:.1e} tf {row['tf']:.2f}  "
                      f"{row['solve_s']:.3f} s {row['iterations']} it {row['status']}", flush=True)
    (a.out / 'summary.json').write_text(json.dumps(rows, indent=1) + '\n')


if __name__ == '__main__':
    main()
