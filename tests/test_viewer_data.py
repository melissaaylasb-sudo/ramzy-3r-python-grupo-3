"""Comprueba que el visor muestra las ramas de la corrida con su geometría."""

import unittest
from pathlib import Path

from launch_viewer import build_data


class InverseViewerDataTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        results = Path(__file__).resolve().parents[1] / "resultados"
        cls.data = build_data(results)
        cls.cases = {case["id"]: case for case in cls.data["inverse_cases"]}

    def test_counts_and_spatial_postures(self):
        expected = {
            "regular": (4, 2, "ABBA"),
            "codo_extend": (2, 1, "AA"),
            "codo_plegado": (2, 1, "AA"),
            "eje_base": (2, 2, "AB"),
            "interseccion": (1, 1, "A"),
            "inalcanzable_exterior": (0, 0, ""),
            "inalcanzable_interior": (0, 0, ""),
        }
        for name, (count, postures, groups) in expected.items():
            with self.subTest(name=name):
                case = self.cases[name]
                self.assertEqual(len(case["solutions"]), count)
                self.assertEqual(case["posture_count"], postures)
                self.assertEqual("".join(solution["posture"] for solution in case["solutions"]), groups)

    def test_unreachable_limit_pose_is_not_an_ik_solution(self):
        for name, gap in (("inalcanzable_exterior", .07),
                          ("inalcanzable_interior", .05)):
            with self.subTest(name=name):
                case = self.cases[name]
                self.assertEqual(case["solutions"], [])
                self.assertAlmostEqual(case["limit_pose"]["distance_to_target_m"], gap)
        self.assertTrue(self.cases["eje_base"]["base_free"])
        self.assertTrue(self.cases["interseccion"]["base_free"])
        self.assertFalse(self.cases["inalcanzable_interior"]["base_free"])


if __name__ == "__main__":
    unittest.main()
