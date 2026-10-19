"""Adapter checks against actual OpenSCvx construction, before optimization."""
import importlib.util
import sys
import unittest
import numpy as np
from point_mass_paper_protocol import candidate, shared_seed
from point_mass_matched import SCP, scp_profile, build_scp, evaluation_error_certificate

sys.path.insert(0,str(SCP))


class EvaluationCertificateTest(unittest.TestCase):
    def test_budget_pass_does_not_imply_accuracy(self):
        cert={'integral':{'lower_rational':'0','upper_rational':'1/200000000000'}}
        self.assertEqual(evaluation_error_certificate(cert,0.,1e-11)['status'],'uncertified')
        zero={'integral':{'lower_rational':'0','upper_rational':'0'}}
        self.assertEqual(evaluation_error_certificate(zero,0.,1e-11)['status'],'pass')
        self.assertEqual(evaluation_error_certificate(zero,1e-12,1e-11)['status'],'fail')


@unittest.skipUnless(importlib.util.find_spec('openscvx'),'requires pinned local OpenSCvx')
class MatchedSeedTest(unittest.TestCase):
    def test_all_calibration_roots_and_both_duration_coordinates(self):
        protocol=candidate();protocol['profiles']['lifted_openscvx']=scp_profile()
        for case in protocol['calibration_cases']:
            for mode in ('fixed','free'):
                with self.subTest(case=case['id'],mode=mode):
                    seed=shared_seed(case,6)
                    problem=build_scp(protocol,dict(N=6,mode=mode,case=case,seed=seed))
                    states={s.name:s for s in problem.symbolic.states}
                    controls={s.name:s for s in problem.symbolic.controls}
                    self.assertTrue(np.array_equal(states['position'].guess,np.asarray(seed['X'])[:,:3]))
                    self.assertTrue(np.array_equal(states['velocity'].guess,np.asarray(seed['X'])[:,3:]))
                    self.assertTrue(np.array_equal(controls['acceleration'].guess[:-1],seed['U']))
                    self.assertTrue(np.allclose(controls['_time_dilation'].guess[:-1].ravel()/6,
                                               seed['durations_s'],atol=1e-15,rtol=0))
                    cost=sum(dt*(1+1e-5*np.dot(u,u)) for u,dt in zip(seed['U'],seed['durations_s']))
                    self.assertAlmostEqual(float(states['objective'].guess[-1,0]),cost,places=12)
                    self.assertTrue(np.all(states['y'].guess==0))


if __name__=='__main__':unittest.main()
