import json
from pathlib import Path
import tempfile
import unittest
import numpy as np
from point_mass_factorization_audit import analyze, compare_traces


class FactorizationAuditTest(unittest.TestCase):
    def test_positive_averaged_spectrum_does_not_certify_actual_factorization(self):
        # The existing diagnostic would report min eigenvalue 1. LLT actually
        # sees [[1,1.5],[1.5,1]], which has min eigenvalue -0.5.
        matrix = [[1., 1.5], [-1.5, 1.]]
        with tempfile.TemporaryDirectory() as d:
            log = Path(d)/'native.log'
            log.write_text('[QUU_MATRIX] '+json.dumps(dict(stage=0, base=matrix,
                factored=matrix, uncondensed=np.eye(2).tolist(), ipm=matrix))+'\n')
            result = analyze(log)
        self.assertEqual(result['failures_reproduced'], 1)
        self.assertEqual(result['averaged_positive_but_upper_negative'], 1)

    def test_logging_replay_comparison_ignores_time_but_detects_policy_change(self):
        with tempfile.TemporaryDirectory() as d:
            a, b = Path(d)/'reference', Path(d)/'actual'
            record = dict(iteration=1, X=[[1.]], U=[[2.]], mu=.01, rho=1., opterror=.2)
            a.write_text(json.dumps(dict(record, wall_seconds=1))+'\n')
            b.write_text(json.dumps(dict(record, wall_seconds=99))+'\n')
            self.assertTrue(compare_traces(a, b)['identical_optimizer_records_excluding_timing'])
            record['U'] = [[2.00001]]
            b.write_text(json.dumps(record)+'\n')
            self.assertFalse(compare_traces(a, b)['identical_optimizer_records_excluding_timing'])


if __name__ == '__main__':
    unittest.main()
