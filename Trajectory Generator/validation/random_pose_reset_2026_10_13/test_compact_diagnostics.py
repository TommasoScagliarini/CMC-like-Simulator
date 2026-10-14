"""Synthetic diagnostic checks only; no new outcome files or native imports."""
import importlib.util
import math
from pathlib import Path
import unittest

HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location("compact_pose_diagnostics", HERE / "compact_diagnostics.py")
M = importlib.util.module_from_spec(spec)
spec.loader.exec_module(M)


class TestDescriptiveDiagnostics(unittest.TestCase):
    def test_clipping_per_joint_and_any_step(self):
        rows = [{"raw_policy_action": [[1.2, -.3]], "applied_policy_action": [[1, -.3]]},
                {"raw_policy_action": [.2, -1.1], "applied_policy_action": [.2, -1]}]
        result = M.clipping(rows)
        self.assertEqual(result["any_joint_raw_clipped_steps"], 2)
        self.assertEqual(result["per_joint"]["pros_knee_angle"]["raw_outside_normalized_bounds_steps"], 1)
        self.assertAlmostEqual(result["per_joint"]["pros_ankle_angle"]["raw_minus_applied_rms"], .1 / math.sqrt(2))

    def test_terminal_recycled_fsm_payload_is_not_new_event(self):
        event = {"event": "heel_strike", "event_time_s": 1, "confirmed_time_s": 1.005,
                 "delivered_time_s": 1.01, "cycle_valid": -1}
        rows = [{"phase_fsm": {"accepted_transitions_this_step": [event]}},
                {"phase_fsm": {"accepted_transitions_this_step": [event]}}]
        result = M.event_times(rows, 0, [2])
        self.assertEqual(result["accepted_hs_count"], 1)
        self.assertIsNone(result["first_cycle_close"])
        self.assertAlmostEqual(result["first_hs"]["confirmed_elapsed_s"], 1.005)

    def test_censored_late_window_remains_empty_not_zero(self):
        times = [i / 1000 for i in range(320)]
        table = {"time": times, "pelvis_ty_reserve_torque": [10] * len(times),
                 "pelvis_tilt_reserve_torque": [20] * len(times), "pros_knee_angle_reserve_torque": [0] * len(times)}
        result = M.reserve_diagnostics(table, 0)
        self.assertEqual(result["startup_000_050"]["samples"], 320)
        self.assertEqual(result["late_400_500"]["samples"], 0)
        self.assertIsNone(result["late_400_500"]["per_coordinate"]["pelvis_ty"]["rms"])
        self.assertEqual(result["startup_000_050"]["per_coordinate"]["pelvis_ty"]["unit"], "N")
        self.assertEqual(result["startup_000_050"]["biological_rotational_reserve_norm_Nm"]["rms"], 20)
        self.assertEqual(result["startup_000_050"]["pelvis_translation_residual_force_norm_N"]["rms"], 10)
        self.assertNotIn("pros_knee_angle", result["startup_000_050"]["per_coordinate"])

    def test_primary_contact_and_detector_strata_are_separate(self):
        receipt = {"initial_state": {"actual_q": {"pros_knee_angle": -.5, "pros_ankle_angle": .1},
                                      "original_prosthetic_ik": {"pros_knee_angle": {"q": -.2}, "pros_ankle_angle": {"q": .1}}},
                   "static_contact_gate": {"force_guard_n": 3500, "primary_grf": {"sides": {"left": {"in_contact": False, "normal_force": 0}}},
                                           "binary_detector_baseline": {"contacts": {"left_heel": True, "left_toe": False}}}}
        result = M.initial_diagnostics(receipt, None)
        self.assertEqual(result["detector"]["word"], "HEEL")
        self.assertFalse(result["primary_contact"]["in_contact"])
        self.assertAlmostEqual(result["euclidean_distance_from_nominal_rad"], .3)

    def test_same_clock_unit_conversion_without_phase_shift(self):
        times = [i / 1000 for i in range(5000)]
        actual = {"time": times, "pelvis_ty": [1.01] * 5000, "hip_flexion_l": [math.radians(3)] * 5000,
                  "pros_knee_angle": [-.5] * 5000}
        control = {"time": times, "pelvis_ty": [1] * 5000, "hip_flexion_l": [0] * 5000, "pros_knee_angle": [-.5] * 5000}
        result = M.compare_same_clock(actual, control, 0)
        self.assertEqual(result["startup_000_050"]["overlap_samples"], 500)
        self.assertEqual(result["late_400_500"]["overlap_samples"], 1000)
        self.assertAlmostEqual(result["middle_200_400"]["biological_q_rmse"]["hip_flexion_l"]["rmse"], 3)
        self.assertAlmostEqual(result["middle_200_400"]["biological_q_rmse"]["pelvis_ty"]["rmse"], .01)


if __name__ == "__main__":
    unittest.main()
