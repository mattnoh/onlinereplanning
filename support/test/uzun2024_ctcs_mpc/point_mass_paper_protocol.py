"""Prepare a matched CTCS campaign; never select winners from historical pools.

This generates a candidate protocol and disjoint calibration/test cases. It
does not launch solvers or declare the candidate settings publication ready.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path


def frozen_ddp_profile():
    return json.loads((Path(__file__).parent/'point_mass_ddp_frozen_profile.json').read_text())


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, allow_nan=False,
                                    separators=(',', ':')).encode()).hexdigest()


def physical_row_coordinates(budget, duration, fixed_time):
    """Return legacy epsilon and c-to-normalized-row multiplier.

    Fixed-time legacy c=I/dt-epsilon; free-time c=I-epsilon.
    In either coordinate system the result is exactly I/B-1.
    """
    if not all(math.isfinite(v) and v > 0 for v in (budget, duration)):
        raise ValueError('budget and duration must be finite and positive')
    epsilon = budget / duration if fixed_time else budget
    return dict(epsilon=epsilon, residual_multiplier=1 / epsilon)


def approach_case(case_id, angle):
    # Rotate position and horizontal velocity together about the obstacle.
    ox, oy = 2.85355, 2.14645
    dx, dy = 5 - ox, 5 - oy
    c, s = math.cos(angle), math.sin(angle)
    return dict(id=case_id, angle_rad=angle,
                initial_state=[ox+c*dx-s*dy, oy+s*dx+c*dy, 10.,
                               -.2*c+.2*s, -.2*s-.2*c, -.3],
                terminal_state=[0., 0., 1., 0., 0., 0.])


def candidate():
    return dict(
        schema_version=1, status='candidate_not_frozen',
        methods=['folded_ddp', 'lifted_openscvx', 'direct_nonaug_prox_ablation'],
        meshes=[6, 10, 20, 50, 100], time_modes=['fixed', 'free'],
        physical_budget_per_interval=1e-11,
        budget_units='seconds times squared dimensionless residual',
        tightening_m={'obstacle': .005, 'glide_residual': .005},
        obstacle={'center': [2.85355, 2.14645, 5.5], 'radius': 2.},
        fixed_tf_s=4., free_tf_bounds_s=[1., 30.], free_initial_tf_s=4.,
        row='I_physical/B - 1', density_length_scale_m=2., glide_tan=1.,
        initialization='shared_hover_rollout',
        initialization_details='u=(0,0,9.81); exact rollout from each initial state; '
                               'same X/U/durations for every arm; terminal defect allowed',
        audit={'shooting_gap_inf': 1e-8, 'terminal_error_inf': 1e-8,
               'physical_violation_m': 1e-8, 'input_violation': 1e-8,
               'cone_violation': 1e-8, 'integral_certificate': 'pass',
               'physical_certificate': 'pass', 'tightening_certificate': 'pass',
               'max_integral_error_over_budget': 1e-4},
        calibration_cases=[approach_case(f'calibration_{i}', a)
                           for i, a in enumerate([0., math.pi/3, math.pi])],
        test_cases=[approach_case(f'test_{i:02d}', 2*math.pi*(i+.5)/32)
                    for i in range(32)],
        selection='one returned policy per method/time/mesh/case; no retries or best-of-pool',
        profiles={'folded_ddp': frozen_ddp_profile()}, source_pins=None,
        optimizer_quadrature={'folded_ddp_rk4_substeps': 512},
        stopping='method-specific stationarity plus common certified feasibility; '
                 'Outer Max/Min and iteration limits are not optimality',
        timing='excluded from primary claims',
        release_blockers=[
            'Calibrate profiles once on calibration cases; archive every trial and work budget.',
            'DDP profile fixed by user: NT, legacy gate, CTCS GN off; solver/globalization research stopped.',
            'Choose fixed optimizer quadrature by accuracy calibration, not substep count alone.',
            'Verify shared cold seed and physical objective in all three adapters, fixed and free.',
            'Pin clean tracked sources, build settings, binaries, profiles and seed arrays.',
            'Freeze protocol hash before opening any test-case solver outcomes.'])


def shared_seed(case, mesh, tf=4.):
    dt = tf / mesh
    x0 = case['initial_state']
    states = [[*(x0[j] + k*dt*x0[j+3] for j in range(3)), *x0[3:]]
              for k in range(mesh+1)]
    return dict(X=states, U=[[0., 0., 9.81] for _ in range(mesh)],
                durations_s=[dt]*mesh)


def acceptance(metrics, protocol):
    """Fail closed on missing/nonfinite independent evidence, not native status.

    Certificates refer to the full propagated nominal policy; interval-local
    checks, accumulator defects and solver convergence remain separate fields.
    """
    failures = []
    limits = protocol['audit']
    for name in ('shooting_gap_inf', 'terminal_error_inf', 'physical_violation_m',
                 'input_violation', 'cone_violation', 'max_integral_error_over_budget'):
        value = metrics.get(name)
        if (isinstance(value, bool) or not isinstance(value, (int, float)) or
                not math.isfinite(value) or value < 0 or value > limits[name]):
            failures.append(name)
    for name in ('integral_certificate', 'physical_certificate', 'tightening_certificate'):
        if metrics.get(name) != limits[name]:
            failures.append(name)
    return dict(feasible=not failures, failed_checks=failures,
                # A native success flag alone cannot establish convergence.
                convergence='unassessed_requires_stationarity_evidence')


def barrier_trace_check(values):
    values = list(values)
    return bool(values) and all(math.isfinite(v) and v > 0 for v in values) and all(
        b <= a for a, b in zip(values, values[1:]))


def prepare(out):
    out.mkdir(parents=True, exist_ok=False)
    protocol = candidate()
    protocol_hash = digest(protocol)
    (out/'protocol.json').write_text(json.dumps(protocol, indent=2, allow_nan=False)+'\n')
    seeds = []
    for split in ('calibration', 'test'):
        for case in protocol[split+'_cases']:
            for mesh in protocol['meshes']:
                # Free-time initial duration equals fixed time: one shared seed.
                seed = shared_seed(case, mesh, protocol['free_initial_tf_s'])
                name = f"{case['id']}_N{mesh}.json"
                (out/name).write_text(json.dumps(seed, allow_nan=False)+'\n')
                seeds.append(dict(split=split, case=case['id'], mesh=mesh,
                                  file=name, seed_sha256=digest(seed)))
    manifest = dict(status='not_ready_for_paper_runs', protocol_sha256=protocol_hash,
                    planned_test_runs=len(protocol['test_cases'])*len(protocol['meshes'])*
                                      len(protocol['time_modes'])*len(protocol['methods']),
                    seeds=seeds, blockers=protocol['release_blockers'])
    (out/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    return manifest


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prepare', type=Path, required=True,
                        help='new output directory; existing evidence is never overwritten')
    args = parser.parse_args()
    manifest = prepare(args.prepare)
    print(json.dumps({k: v for k, v in manifest.items() if k != 'seeds'}, indent=2))
