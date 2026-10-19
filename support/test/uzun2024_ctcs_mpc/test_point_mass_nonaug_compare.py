"""Direct non-augmented penalty/barrier and point-mass-map checks."""
import importlib.util
from pathlib import Path
import json
import unittest
import numpy as np
from point_mass_nonaug_compare import rollout, physical_objective, ProxProblem, ROOT_STATE, GOAL, Oracle


class PointMassSeedTests(unittest.TestCase):
    def test_control_blending_commutes_with_exact_linear_rollout(self):
        a=np.tile([0.,0.,9.81],(6,1));b=a+np.array([.2,-.1,.4])
        self.assertLess(np.max(abs(rollout(.9*a+.1*b)-(.9*rollout(a)+.1*rollout(b)))),1e-13)
        self.assertAlmostEqual(physical_objective(a),4.+4e-5*9.81**2)


@unittest.skipUnless(importlib.util.find_spec('cvxpy'),'requires research CVXPY environment')
class NonaugOptimizationTests(unittest.TestCase):
    def test_hinge_penalty_caps_constraint_multiplier(self):
        import cvxpy as cp
        for gamma in (.25,1.5):
            z=cp.Variable();e=cp.Variable(nonneg=True);row=z<=e
            problem=cp.Problem(cp.Minimize(.5*cp.square(z-1)+gamma*e),[row])
            problem.solve(solver='CLARABEL',tol_gap_abs=1e-11,tol_gap_rel=1e-11,tol_feas=1e-11)
            self.assertAlmostEqual(float(z.value),max(0.,1-gamma),places=5)
            self.assertLessEqual(float(row.dual_value),gamma+1e-7)
        # At the hard-constraint optimum the multiplier is one. Finite barrier
        # parameter gives an interior point rather than a capped penalty force.
        mu=.01;z=(1-np.sqrt(1+4*mu))/2;s=-z;dual=mu/s
        self.assertGreater(s,0);self.assertAlmostEqual(s*dual,mu)
        self.assertAlmostEqual(z-1+dual,0.)

    def test_reference_subproblem_preserves_hard_affine_dynamics_and_endpoints(self):
        U=np.tile([0.,0.,9.81],(6,1));X=rollout(U)
        problem=ProxProblem(6)
        xp,up,_=problem.solve(X,U,-np.ones(6),np.zeros((6,9)),1000.,1e-6)
        self.assertLess(np.max(abs(xp[0]-ROOT_STATE)),1e-8)
        self.assertLess(np.max(abs(xp[-1]-GOAL)),1e-8)
        self.assertLess(np.max(abs(xp-rollout(up))),1e-7)

    def test_shared_cpp_numerical_map_jacobian(self):
        here=Path(__file__).resolve().parents[3]
        snapshot=here/'logs/point_mass/audits/folded_lifted_certificate_20261020_v7'
        binary=here/'logs/point_mass/audits/optimization_structure_20261020_v2/current_evaluator'
        case=here/'logs/point_mass/audits/direct_nonaug_n6_20261020_v3'
        if not binary.exists() or not case.exists(): self.skipTest('requires frozen numerical-map evidence')
        row=json.loads((snapshot/'inputs/fixed_DDP_ctcs_N6/selected_row.json').read_text())
        values=np.loadtxt(case/'seed.txt');X=values[:42].reshape(7,6);U=values[42:].reshape(6,3)
        oracle=Oracle(binary,row,512)
        try:
            c,J=oracle.evaluate(X,U);k=int(np.argmax(c));xp=X.copy();xm=X.copy()
            xp[k,0]+=1e-6;xm[k,0]-=1e-6
            fd=(oracle.evaluate(xp,U)[0][k]-oracle.evaluate(xm,U)[0][k])/2e-6
            self.assertLess(abs(fd-J[k,0])/max(1.,abs(J[k,0])),1e-4)
        finally: oracle.close()


if __name__=='__main__': unittest.main()
