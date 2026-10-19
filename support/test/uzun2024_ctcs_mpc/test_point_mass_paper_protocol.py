"""Checks for errors that would silently invalidate the matched experiment."""
import math
import unittest
from point_mass_paper_protocol import (acceptance, approach_case, barrier_trace_check,
                                      candidate, physical_row_coordinates, shared_seed)


class ProtocolTest(unittest.TestCase):
    def test_same_physical_budget_across_mesh_and_time_coordinates(self):
        b, integral = 1e-11, 1.3e-11
        for n in (6, 10, 20, 50, 100):
            dt = 4/n
            for fixed in (False, True):
                coordinates = physical_row_coordinates(b, dt, fixed)
                legacy = integral/dt if fixed else integral
                row = (legacy-coordinates['epsilon'])*coordinates['residual_multiplier']
                self.assertAlmostEqual(row, .3)
            # Holding legacy mean-square epsilon fixed would fail this identity.
            self.assertAlmostEqual(physical_row_coordinates(b, dt, True)['epsilon']*dt, b)

    def test_shared_seed_exact_flow_and_disjoint_bearings(self):
        p = candidate()
        calibration = [c['angle_rad'] for c in p['calibration_cases']]
        for case in p['test_cases']:
            self.assertTrue(all(abs(case['angle_rad']-a) > 1e-8 for a in calibration))
            x0 = case['initial_state']
            self.assertLess(math.hypot(*x0[:2]), x0[2]-.005)
            self.assertAlmostEqual(sum(v*v for v in x0[3:]), .17)
            seed = shared_seed(case, 6)
            for k, x in enumerate(seed['X']):
                for j in range(3):
                    self.assertAlmostEqual(x[j], x0[j]+k*4/6*x0[j+3])
        self.assertEqual(len({tuple(c['initial_state']) for c in p['test_cases']}), 32)

    def test_native_success_and_nan_cannot_pass_audit(self):
        p = candidate()
        metrics = {k: (v if isinstance(v, str) else 0.) for k, v in p['audit'].items()}
        self.assertTrue(acceptance(metrics, p)['feasible'])
        for field, bad in [('integral_certificate', 'uncertified'),
                           ('shooting_gap_inf', 1e-6),
                           ('max_integral_error_over_budget', math.nan),
                           ('terminal_error_inf', True),
                           ('physical_violation_m', -1.)]:
            with self.subTest(field=field):
                damaged = dict(metrics, native_status='Optimal Solution')
                damaged[field] = bad
                self.assertFalse(acceptance(damaged, p)['feasible'])
        self.assertFalse(acceptance({'native_status': 'Optimal Solution'}, p)['feasible'])

    def test_barrier_increase_is_visible_even_with_unchanged_endpoints(self):
        self.assertTrue(barrier_trace_check([.001, .001, .0002, 1e-10]))
        self.assertFalse(barrier_trace_check([.001, .0001, .0005, 1e-10]))
        self.assertFalse(barrier_trace_check([]))
        self.assertFalse(barrier_trace_check([.001, math.nan]))


if __name__ == '__main__':
    unittest.main()
