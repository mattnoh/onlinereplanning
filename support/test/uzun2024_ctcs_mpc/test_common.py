"""Unit tests for common.py (no solver runs).  Run: python -m unittest test_common -v"""

import json
import tempfile
import unittest
from pathlib import Path

import numpy as np
from scipy.integrate import quad

import common as C


class TestParameters(unittest.TestCase):
    def test_paper_table1(self):
        p = C.PAPER_VALUES
        self.assertEqual((p["duration"], p["horizon"], p["mpc_intervals"], p["nodes"], p["iterations"]),
                         (30.0, 8.0, 4, 17, 3))
        self.assertEqual((p["u_max"], p["c_d"], p["delta"], p["r_sens"]), (5.0, 0.05, 0.05, 6.0))
        np.testing.assert_array_equal(C.CENTERS, [[-6.5, -3], [-1.8, 3.5], [0.5, -3.5], [2.7, 1.5], [6.2, -3]])
        np.testing.assert_array_equal(C.SHAPE, [[2.4, 0], [0, 0.4]])

    def test_assumptions_separate(self):
        self.assertFalse(set(C.PAPER_VALUES) & set(C.ASSUMED_DEFAULTS))
        for k in set(C.ASSUMED_DEFAULTS) - {"hold"}:
            self.assertIn("assumption", C.PARAM_SOURCE[k])
        self.assertEqual(C.PAPER_DEFAULTS["epsilon"], 1e-6)
        self.assertEqual(C.PAPER_DEFAULTS["x0"], [-10.0, 0.0, 0.0, 0.0])


class TestModel(unittest.TestCase):
    def test_reference(self):
        np.testing.assert_allclose(C.reference(0.0), [-10.0, 5.0], atol=1e-14)
        t = np.linspace(0, 30, 7).reshape(7, 1)
        r = C.reference(t)
        self.assertEqual(r.shape, (7, 1, 2))
        t1 = 12.3
        expect = [-10 + 2 * t1 / 3, 5 * np.exp(-0.05 * t1) * np.sin(np.pi / 2 + t1 / 5 + t1 ** 2 / 36)]
        np.testing.assert_allclose(C.reference(t1), expect, rtol=1e-14)
        h = 1e-6
        np.testing.assert_allclose(C.reference_rate(t1), (C.reference(t1 + h) - C.reference(t1 - h)) / (2 * h),
                                   rtol=1e-7)

    def test_rhs(self):
        x = np.array([1.0, 2.0, 3.0, -4.0])
        u = np.array([0.5, -0.5])
        np.testing.assert_allclose(C.rhs(0.0, x, u), [3, -4, 0.5 - 0.05 * 5 * 3, -0.5 + 0.05 * 5 * 4])

    def test_jacobians_fd(self):
        x = np.array([0.3, -0.2, 1.2, -0.7])
        u = np.array([0.4, 1.1])
        A, B = C.rhs_jacobians(x, u)
        h = 1e-7
        Afd = np.stack([(C.rhs(0, x + h * e, u) - C.rhs(0, x - h * e, u)) / (2 * h) for e in np.eye(4)], 1)
        Bfd = np.stack([(C.rhs(0, x, u + h * e) - C.rhs(0, x, u - h * e)) / (2 * h) for e in np.eye(2)], 1)
        np.testing.assert_allclose(A, Afd, atol=1e-7)
        np.testing.assert_allclose(B, Bfd, atol=1e-7)

    def test_obstacle_g_and_grad(self):
        c = C.CENTERS[2]
        self.assertAlmostEqual(C.obstacle_g(c)[2], 1.0)
        # boundary along x at 1/2.4, along y at 1/0.4 = 2.5
        self.assertAlmostEqual(C.obstacle_g(c + [1 / 2.4, 0])[2], 0.0, places=14)
        self.assertAlmostEqual(C.obstacle_g(c + [0, 2.5])[2], 0.0, places=14)
        self.assertEqual(C.obstacle_g(np.zeros((3, 4, 2))).shape, (3, 4, 5))
        p = np.array([0.9, -1.2])
        G = C.obstacle_g_grad(p)
        h = 1e-7
        Gfd = np.stack([(C.obstacle_g(p + h * e) - C.obstacle_g(p - h * e)) / (2 * h) for e in np.eye(2)], 1)
        np.testing.assert_allclose(G, Gfd, atol=1e-7)
        m = np.array([True, False, True, True, False])
        p = C.CENTERS[3] + [0.3, 0.5]
        dg = C.density_grad(p, m)
        dfd = np.array([(C.density(p + h * e, m) - C.density(p - h * e, m)) / (2 * h) for e in np.eye(2)])
        np.testing.assert_allclose(dg, dfd, atol=1e-6)

    def test_sensing(self):
        c = C.CENTERS[0]
        m = C.sensed_mask(c + [6.0 / 2.4 - 1e-9, 0])
        self.assertTrue(m[0])
        self.assertFalse(C.sensed_mask(c + [6.0 / 2.4 + 1e-6, 0])[0])
        self.assertTrue(C.sensed_mask(c + [0, 15.0 - 1e-9])[0])   # 6 / 0.4 = 15 along y
        self.assertEqual(C.sensed_mask(np.array([-10.0, 0.0])).dtype, bool)

    def test_hold(self):
        u0, u1 = np.array([1.0, -2.0]), np.array([3.0, 4.0])
        np.testing.assert_allclose(C.hold_control(0.5, u0, u1, 1.0, "FOH"), [2.0, 1.0])
        np.testing.assert_allclose(C.hold_control(0.5, u0, u1, 1.0, "zoh"), u0)
        with self.assertRaises(ValueError):
            C.hold_control(0.5, u0, u1, 1.0, "PWL")

    def test_ellipse_distance(self):
        c = C.CENTERS[1]
        a, b = 1 / 2.4, 2.5
        th = np.linspace(0, 2 * np.pi, 400001)
        bnd = np.stack([a * np.cos(th), b * np.sin(th)], 1) + c
        pts = c + np.array([[1.0, 0.0], [0.0, 0.0], [0.0, 2.0], [0.1, 1.0], [-2.0, 3.0], [0.2, -0.1], [0, -4]])
        d = C.ellipse_signed_distance(pts)[:, 1]
        brute = np.min(np.linalg.norm(pts[:, None, :] - bnd[None], axis=-1), axis=1)
        sign = np.where(C.obstacle_g(pts)[:, 1] > 0, -1, 1)
        np.testing.assert_allclose(d, sign * brute, atol=1e-5)
        self.assertAlmostEqual(d[0], 1 - a, places=10)
        self.assertAlmostEqual(d[1], -a, places=10)


class TestReplay(unittest.TestCase):
    def test_drag_analytic(self):
        # 1D quadratic drag: v = v0/(1+c v0 t), x = x0 + ln(1+c v0 t)/c
        x = np.array([-20.0, 30.0, 2.0, 0.0])
        T = 1.7
        r = C.replay_interval(x, [0, 0], [0, 0], 0.0, T, "FOH", np.zeros(5, bool))
        v0, c = 2.0, C.C_D
        np.testing.assert_allclose(r["end"], [-20 + np.log(1 + c * v0 * T) / c, 30, v0 / (1 + c * v0 * T), 0],
                                   rtol=1e-10, atol=1e-11)
        self.assertTrue(r["ok"])
        self.assertAlmostEqual(r["times"][-1], T, places=14)
        self.assertEqual(r["states"].shape, (r["times"].size, 4))

    def test_exact_end_sample(self):
        r = C.replay_interval([30, 30, 0, 0], [0, 0], [0, 0], 1.0, 0.0105, "ZOH", np.zeros(5, bool))
        self.assertAlmostEqual(r["times"][-1], 1.0105, places=14)
        self.assertTrue(np.all(np.diff(r["times"]) > 0))

    def test_cost_quadrature(self):
        p = np.array([1.0, 1.0])
        t0, T = 2.0, 0.5
        r = C.replay_interval(np.r_[p, 0, 0], [0, 0], [0, 0], t0, T, "FOH", np.zeros(5, bool))
        ref = quad(lambda t: np.sum((p - C.reference(t)) ** 2), t0, t0 + T, epsabs=1e-13)[0]
        self.assertAlmostEqual(r["cost"], ref, places=9)

    def test_foh_zoh_constant_equivalence(self):
        x = np.array([-10.0, 0.0, 0.5, 0.2])
        u = np.array([1.0, 0.7])
        m = C.sensed_mask(x[:2])
        a = C.replay_interval(x, u, u, 0.0, 0.5, "FOH", m)
        b = C.replay_interval(x, u, [9.0, -9.0], 0.0, 0.5, "ZOH", m)
        np.testing.assert_allclose(a["end"], b["end"], rtol=1e-12, atol=1e-12)
        self.assertAlmostEqual(a["cost"], b["cost"], places=10)
        c = C.replay_interval(x, u, [9.0, -9.0], 0.0, 0.5, "FOH", m)
        self.assertGreater(np.linalg.norm(c["end"] - b["end"]), 1e-3)

    def test_tightening_quadrature_stationary(self):
        # stationary just outside obstacle 3: g = -0.03 -> physical clear, tightened density (0.02)^2
        c = C.CENTERS[3]
        p = c + [0.0, 1.03 / 0.4]
        self.assertAlmostEqual(C.obstacle_g(p)[3], -0.03, places=12)
        m = np.zeros(5, bool)
        m[3] = True
        T = 0.4
        r = C.replay_interval(np.r_[p, 0, 0], [0, 0], [0, 0], 0.0, T, "ZOH", m)
        self.assertAlmostEqual(r["integral"], T * 0.02 ** 2, places=12)
        self.assertEqual(r["max_physical_excess"], 0.0)
        self.assertEqual(r["physical_integral"], 0.0)
        self.assertEqual(r["violation_duration"], 0.0)
        self.assertAlmostEqual(r["min_clearance"], 0.03, places=10)
        self.assertAlmostEqual(r["min_geometric_clearance"], 0.03 / 0.4, places=8)
        r0 = C.replay_interval(np.r_[p, 0, 0], [0, 0], [0, 0], 0.0, T, "ZOH", np.zeros(5, bool))
        self.assertEqual(r0["integral"], 0.0)

    def test_physical_audit_all_obstacles_and_events(self):
        # drive through obstacle 2 along x with mask empty: physical audit must still see it
        c = C.CENTERS[2]
        x = np.r_[c + [-1.0, 0.0], 2.0, 0.0]
        T = 1.0
        r = C.replay_interval(x, [0, 0], [0, 0], 5.0, T, "FOH", np.zeros(5, bool))
        self.assertEqual(r["integral"], 0.0)
        self.assertGreater(r["max_physical_excess"], 0.9)
        self.assertEqual(r["max_g_obstacle"], 2)
        self.assertEqual(len(r["crossings"]), 2)
        self.assertEqual([d for _, _, d in r["crossings"]], ["enter", "exit"])
        # duration inside = 2a / speed (speed nearly constant, check against events)
        (t_in, _, _), (t_out, _, _) = r["crossings"]
        self.assertAlmostEqual(r["violation_duration"], t_out - t_in, places=9)
        self.assertGreater(r["physical_integral"], 0.0)
        self.assertLess(r["min_clearance"], -0.9)

    def test_narrow_pocket_peak_refined(self):
        # graze obstacle 0 tip at high lateral speed: peak between samples must be refined
        c = C.CENTERS[0]
        y_tip = c[1] + 2.5 - 1e-4          # crosses the top tip by 1e-4 in g-units * 2.5
        x = np.r_[c[0] - 0.3, y_tip, 40.0, 0.0]
        r = C.replay_interval(x, [0, 0], [0, 0], 0.0, 0.02, "ZOH", np.zeros(5, bool), step=1e-3)
        sampled = r["g"][:, 0].max()
        self.assertGreaterEqual(r["max_g_per_obstacle"][0], sampled)
        self.assertAlmostEqual(r["max_g_per_obstacle"][0], 1e-4 / 2.5, delta=2e-7)
        self.assertGreater(r["violation_duration"], 0.0)

    def test_sequence_and_initial_guess(self):
        x0 = C.PAPER_DEFAULTS["x0"]
        ig = C.zero_control_initial_guess(x0, 0.0, horizon=1.0, nodes=5)
        np.testing.assert_allclose(ig["X"], np.tile(x0, (5, 1)))   # stationary under zero control
        self.assertEqual(ig["l"][0], 0.0)
        self.assertTrue(np.all(np.diff(ig["l"]) > 0))
        segs = [dict(u0=[1, 0], u1=[1, 0], t0=0.25 * k, duration=0.25, hold="ZOH",
                     mask=C.sensed_mask(np.array(x0[:2]))) for k in range(4)]
        summ, res, dense = C.replay_sequence(x0, segs)
        single = C.replay_interval(x0, [1, 0], [1, 0], 0.0, 1.0, "ZOH", segs[0]["mask"])
        np.testing.assert_allclose(summ["end"], single["end"], rtol=1e-9, atol=1e-10)
        self.assertAlmostEqual(summ["cost"], single["cost"], places=8)
        self.assertEqual(dense["times"].size, single["times"].size)
        bud = C.interval_budget_check([r["integral"] for r in res], 1e-6)
        self.assertIn("ok", bud)


class TestPersistence(unittest.TestCase):
    def test_manifest_fail_status(self):
        with tempfile.TemporaryDirectory() as d:
            cfg = dict(C.PAPER_DEFAULTS, epsilon=1e-4)
            code = C.write_manifest(d, "unit", cfg, "fail", summary={"x": np.float64(1.0)})
            self.assertEqual(code, 1)
            m = json.loads((Path(d) / "manifest.json").read_text())
            self.assertEqual(m["status"], "fail")
            self.assertEqual(m["config_provenance"]["epsilon"], "assumption (overridden)")
            self.assertEqual(m["config_provenance"]["nodes"], "paper")
            C.save_csv(Path(d) / "a.csv", [{"a": 1, "b": np.array([1, 2])}, {"c": 3}])
            C.save_npz(Path(d) / "a.npz", t=np.arange(3))
            with self.assertRaises(ValueError):
                C.write_manifest(d, "unit", cfg, "ok")


if __name__ == "__main__":
    unittest.main()
