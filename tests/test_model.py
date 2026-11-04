import unittest

import numpy as np
from numpy.testing import assert_allclose

from ramzy3r.benchmarks import dynamic_benchmark, static_benchmark
from ramzy3r.control import DLS_DAMPING, simulate_control
from ramzy3r.dynamics import _mass_and_gravity, _velocity_bias, simulate_dynamics
from ramzy3r.kinematics import (L1, L2, L3, SINGULAR_RTOL, determinant, fk, frames,
                                inverse_kinematics, jacobian, jacobian_fd, numerical_rank,
                                singular_cases, singular_value_families, singular_values,
                                singularity_measures)


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
        expected = {"regular": 3, "codo_extend": 2, "codo_plegado": 2,
                    "eje_base": 2, "interseccion": 1}
        svd_ranks = {name: numerical_rank(np.linalg.svd(jacobian(q), compute_uv=False))
                     for name, q in cases.items()}
        measures = {name: singularity_measures(q) for name, q in cases.items()}
        self.assertEqual(svd_ranks, expected)
        self.assertEqual({name: m["rango"] for name, m in measures.items()}, expected)
        self.assertEqual({name: m["familia"] for name, m in measures.items()},
                         {"regular": "ninguna", "codo_extend": "codo", "codo_plegado": "codo",
                          "eje_base": "eje_base", "interseccion": "interseccion"})

    def test_closed_form_singular_values(self):
        for q in np.random.default_rng(3).uniform(-np.pi, np.pi, (2000, 3)):
            sigma = singular_values(q)
            assert_allclose(sigma, np.linalg.svd(jacobian(q), compute_uv=False), atol=1e-15)
            self.assertAlmostEqual(sigma.prod(), abs(determinant(q)), places=15)
            sigma_base, _, sigma_elbow = singular_value_families(q)
            r = L2 * np.cos(q[1]) + L3 * np.cos(q[1] + q[2])
            self.assertAlmostEqual(sigma_base, abs(r), places=15)
            # La dirección perdida en el eje de la base es la tangencial (-s1, c1, 0).
            tangent = np.array([-np.sin(q[0]), np.cos(q[0]), 0.])
            assert_allclose(jacobian(q).T @ tangent, [r, 0., 0.], atol=1e-15)
            # El bloque plano solo depende de q3: no cambia al mover q1 o q2.
            self.assertAlmostEqual(singular_value_families([0., 0., q[2]])[2], sigma_elbow,
                                   places=15)

    def test_closed_form_keeps_relative_accuracy_near_elbow(self):
        slope = L2 * L3 / np.hypot(L2 + L3, L3)
        for q3 in (1e-6, 1e-10, 1e-14):
            assert_allclose(singular_values([.2, .3, q3])[-1], slope * q3, rtol=1e-6)
        q_axis = singular_cases()["eje_base"].copy()
        q_axis[1] += 1e-12
        assert_allclose(singular_values(q_axis)[-1], np.hypot(L2, L3) * 1e-12, rtol=1e-3)

    def test_rank_tolerance_is_relative_and_shared(self):
        self.assertEqual(numerical_rank([1., .5, 2e-11]), 2)
        self.assertEqual(numerical_rank(1e-6 * np.array([1., .5, 2e-11])), 2)
        self.assertEqual(numerical_rank([0., 0., 0.]), 0)
        d = simulate_control(fk, jacobian, "near_singular", "dls")
        self.assertEqual(d["relative_svd_cutoff"], SINGULAR_RTOL)
        self.assertEqual(d["damping"], DLS_DAMPING)
        near = singularity_measures([.2, .3, .1])
        self.assertLess(near["sigma_min"], DLS_DAMPING)
        self.assertEqual((near["rango"], near["familia"]), (3, "codo"))


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
