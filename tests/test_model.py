import unittest

import numpy as np
from numpy.testing import assert_allclose

from ramzy3r.benchmarks import dynamic_benchmark, static_benchmark
from ramzy3r.control import simulate_control
from ramzy3r.dynamics import _mass_and_gravity, _velocity_bias, simulate_dynamics
from ramzy3r.kinematics import (L1, L2, L3, determinant, fk, frames, inverse_kinematics,
                                jacobian, jacobian_fd, singular_cases)


class KinematicsTests(unittest.TestCase):
    def test_reference_pose(self):
        q = np.deg2rad([30., 60., -60.])
        assert_allclose(fk(q), [.333419780457, .1925, .291147367097], atol=1e-12)

    def test_independent_formulations(self):
        for q in np.random.default_rng(5285).uniform(-np.pi, np.pi, (64, 3)):
            t = frames(q)[-1]
            r = t[:3, :3]
            assert_allclose(t[:3, 3], fk(q), atol=1e-12)
            assert_allclose(t[3], [0., 0., 0., 1.])
            assert_allclose(r.T @ r, np.eye(3), atol=1e-12)
            self.assertAlmostEqual(np.linalg.det(r), 1.)
            assert_allclose(jacobian(q), jacobian_fd(q), atol=1e-8)
            self.assertAlmostEqual(np.linalg.det(jacobian(q)), determinant(q), places=12)

    def test_inverse_all_branches_and_quadrants(self):
        for q1 in np.deg2rad([-150., -45., 45., 150.]):
            target = fk([q1, .35, -.55])
            solutions = inverse_kinematics(target)
            self.assertEqual(len(solutions), 4)
            self.assertEqual({(sol.radial_sign, sol.elbow_sign) for sol in solutions},
                             {(-1, -1), (-1, 1), (1, -1), (1, 1)})
            for sol in solutions:
                assert_allclose(fk(sol.q), target, atol=1e-10)

    def test_inverse_axis_boundaries_and_unreachable(self):
        solutions = inverse_kinematics([0., 0., L1 + .2])
        self.assertTrue(all(sol.base_free for sol in solutions))
        for sol in solutions:
            assert_allclose(fk(sol.q), [0., 0., L1 + .2], atol=1e-10)
        for radius in (L2 + L3, L2 - L3):
            solutions = inverse_kinematics([radius, 0., L1])
            self.assertEqual(len(solutions), 2)
            for sol in solutions:
                assert_allclose(fk(sol.q), [radius, 0., L1], atol=1e-10)
        self.assertEqual(inverse_kinematics([.6, 0., L1]), [])
        self.assertEqual(inverse_kinematics([0., 0., L1]), [])

    def test_singular_families(self):
        cases = singular_cases()
        ranks = {name: np.linalg.matrix_rank(jacobian(q), tol=1e-10)
                 for name, q in cases.items()}
        self.assertEqual(ranks, {"regular": 3, "codo_extend": 2, "codo_plegado": 2,
                                 "eje_base": 2, "interseccion": 1})


class ExperimentTests(unittest.TestCase):
    def test_published_sample_masks(self):
        rows, _, stats = static_benchmark()
        self.assertEqual(len(rows), 26)
        m = stats["nominal_vs_experimental"]
        self.assertEqual([m[a]["n"] for a in "xyz"], [7, 26, 19])
        assert_allclose([m[a]["mae_mm"] for a in "xyz"], [1.94752, 2.09453, 6.33824],
                        atol=1e-5)
        _, dynamic = dynamic_benchmark()
        self.assertEqual([dynamic[a]["n"] for a in "xyz"], [26, 26, 26])

    def test_control_uses_matched_scenario_and_rate_cap(self):
        a, b = [simulate_control(fk, jacobian, "near_singular", method, measurement_bias_m=.001)
                for method in ("pinv", "dls")]
        assert_allclose(a["q"][0], b["q"][0])
        assert_allclose(a["desired"], b["desired"])
        assert_allclose(a["measurement_bias"], b["measurement_bias"])
        for d in (a, b):
            self.assertTrue(np.all(np.isfinite(d["actual"])))
            self.assertLessEqual(np.max(np.abs(d["qdot"])), d["speed_limit"])
            self.assertGreater(d["desired_sigma_min"].min(), 0.)
        self.assertGreater(np.count_nonzero(a["saturated"]), np.count_nonzero(b["saturated"]))

    def test_dynamic_gradient_and_power(self):
        for q in np.random.default_rng(42).normal(size=(8, 3)):
            velocity = np.array([.15, -.32, .41])
            m, g, _ = _mass_and_gravity(q)
            assert_allclose(m, m.T, atol=1e-13)
            self.assertGreater(np.linalg.eigvalsh(m)[0], 0.)
            d = 1e-6
            offsets = d * np.eye(3)
            grad = [(_mass_and_gravity(q + u)[2] - _mass_and_gravity(q - u)[2]) / (2*d)
                    for u in offsets]
            assert_allclose(g, grad, atol=1e-8)
            mdot = (_mass_and_gravity(q + d*velocity)[0]
                    - _mass_and_gravity(q - d*velocity)[0]) / (2*d)
            self.assertAlmostEqual(velocity @ _velocity_bias(q, velocity),
                                   .5 * velocity @ mdot @ velocity, places=9)

    def test_energy_balance(self):
        for damping in (0., 1.):
            d = simulate_dynamics(damping_scale=damping, duration=.4)
            self.assertLess(np.max(np.abs(d["balance_residual_j"])), 1e-6)
            self.assertGreater(d["min_mass_eigenvalue"], 0.)
            if damping:
                self.assertGreater(d["dissipated_j"][-1], 0.)


if __name__ == "__main__":
    unittest.main()
