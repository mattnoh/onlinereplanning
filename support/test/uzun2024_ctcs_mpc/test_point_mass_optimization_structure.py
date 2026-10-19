"""Mechanism tests for the local optimization comparison; no trajectory solves."""
import importlib.util
from pathlib import Path
import unittest
import numpy as np
from point_mass_optimization_structure import convex_experiment, grazing_flatness, stopping_metric_probe


@unittest.skipUnless(importlib.util.find_spec('cvxpy'),'requires the research CVXPY environment')
class LocalOptimizationTests(unittest.TestCase):
    def test_exact_elimination_preserves_step(self):
        for metric,penalty in ((0.,None),(.5,None),(.5,.01)):
            r=convex_experiment(metric,penalty)
            self.assertLess(r['step_difference'],2e-6)

    def test_accumulator_metric_changes_physical_step(self):
        plain=convex_experiment()['lifted']['step']
        weighted=convex_experiment(metric=50.)['lifted']['step']
        self.assertGreater(np.linalg.norm(np.array(plain)-weighted),.05)

    def test_virtual_control_can_satisfy_increment_while_true_linear_row_violates(self):
        soft=convex_experiment(virtual_penalty=.01)['lifted']
        self.assertGreater(max(soft['linearized_true_integral_row']),.05)
        self.assertLess(max(soft['increment_row']),1e-8)
        hard=convex_experiment(virtual_penalty=10.)['lifted']
        self.assertLess(max(hard['linearized_true_integral_row']),1e-7)

    def test_scaling_requires_proximal_and_virtual_penalty_compensation(self):
        a=convex_experiment(.5,.01,1.)['lifted']['step']
        b=convex_experiment(.005,.001,10.)['lifted']['step']
        self.assertLess(np.max(abs(np.array(a)-b)),1e-6)


class SharedFlatnessTests(unittest.TestCase):
    def test_grazing_integral_and_derivative_have_different_flatness_orders(self):
        row=grazing_flatness()[-1]
        self.assertAlmostEqual(row['observed_integral_log_slope'],2.5,places=4)
        self.assertAlmostEqual(row['observed_gradient_log_slope'],1.5,places=4)


@unittest.skipUnless(importlib.util.find_spec('jax') and importlib.util.find_spec('cvxpy') and
                    Path('/home/lab/projects/solver/SCP/OpenSCvx').exists(),'requires pinned SCP research environment')
class StoppingMetricTests(unittest.TestCase):
    def test_pinned_source_applies_scaling_twice_to_solver_nu(self):
        probe=stopping_metric_probe(Path('/home/lab/projects/solver/SCP/OpenSCvx'))
        row=probe['cases'][1]
        self.assertAlmostEqual(row['solver_scaled_nu'],.1)
        self.assertAlmostEqual(row['actual_source_metric'],.01)
        self.assertTrue(row['actual_source_converged_at_ep_vc_005'])
        self.assertFalse(row['once_scaled_converged_at_ep_vc_005'])


if __name__=='__main__': unittest.main()
