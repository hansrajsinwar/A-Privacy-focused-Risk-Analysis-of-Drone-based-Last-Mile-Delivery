"""Regression tests: python -m unittest -v   (run `python make_feeds.py` first)"""
import unittest

from scenarios import ALL
from sim import Simulation


def run(name, seed=1):
    return Simulation(ALL[name], seed=seed).run()


class TestScenarios(unittest.TestCase):
    def test_baseline_passes(self):
        m = run("baseline")
        self.assertEqual(m["outcome"], "PASS")
        self.assertLess(m["max_est_error_m"], 2.0)

    def test_short_gps_dropout_tolerated(self):
        m = run("gps_dropout_short")
        self.assertEqual(m["outcome"], "PASS")
        self.assertIsNone(m["first_failsafe"])

    def test_long_gps_dropout_lands_safely(self):
        m = run("gps_dropout_long")
        self.assertEqual(m["outcome"], "SAFE_ABORT")
        self.assertLess(m["detection_latency_s"], 3.0)

    def test_gps_jump_detected(self):
        m = run("gps_step_jump")
        self.assertIsNotNone(m["first_failsafe"])
        self.assertNotEqual(m["outcome"], "FAIL")

    def test_slow_drift_caught_by_detector_but_not_without_it(self):
        with_det = run("gps_drift_slow")
        without = run("gps_drift_slow_no_detector")
        self.assertIsNotNone(with_det["first_failsafe"])
        self.assertIsNone(without["first_failsafe"])

    def test_consistent_spoof_defeats_detector(self):
        m = run("gps_drift_consistent")
        self.assertIsNone(m["first_failsafe"])
        self.assertEqual(m["outcome"], "DEGRADED")

    def test_false_high_battery_crashes(self):
        self.assertEqual(run("battery_false_high")["outcome"], "FAIL")

    def test_false_low_battery_aborts(self):
        self.assertEqual(run("battery_false_low")["outcome"], "SAFE_ABORT")

    def test_two_drones_avoid_each_other(self):
        m = run("two_drones_baseline")
        self.assertEqual(m["outcome"], "PASS")
        self.assertGreater(m["min_separation_m"], 3.0)

    def test_v2v_attacks_cause_collision(self):
        for name in ("two_drones_no_avoidance", "two_drones_v2v_freeze", "two_drones_v2v_drop"):
            self.assertEqual(run(name)["outcome"], "FAIL", name)

    def test_ghost_causes_denial_of_service(self):
        self.assertEqual(run("two_drones_v2v_ghost")["outcome"], "HUNG")

    def test_deterministic_for_same_seed(self):
        a, b = run("gps_freeze", 3), run("gps_freeze", 3)
        self.assertEqual(a["max_est_error_m"], b["max_est_error_m"])


if __name__ == "__main__":
    unittest.main()
