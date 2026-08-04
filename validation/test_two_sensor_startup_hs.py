"""Pure and lightweight tests for prescribed startup-HS validation."""

from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

import numpy as np

from validation import validate_two_sensor_startup_hs as subject


class TwoSensorStartupHSTest(unittest.TestCase):
    def test_detector_contract_is_exactly_the_two_sphere_v9_candidate(self) -> None:
        contract = subject.detector_contract(subject.DEFAULT_DETECTOR_PROFILE)
        self.assertEqual(contract["candidate_id"], subject.V9_CANDIDATE_ID)
        self.assertEqual(contract["sphere_count"], 2)
        self.assertEqual(set(contract["roles"]), {"left_heel", "left_toe"})
        self.assertTrue(contract["detector_only"])
        self.assertFalse(contract["generates_grf"])

    def test_evaluation_grid_is_endpoint_inclusive_at_both_cadences(self) -> None:
        for dt_s, expected_count in ((0.01, 501), (0.001, 5001)):
            times = subject.evaluation_times(13.25, 5.0, dt_s)
            self.assertEqual(times.size, expected_count)
            self.assertAlmostEqual(float(times[0]), 13.25, places=12)
            self.assertAlmostEqual(float(times[-1]), 18.25, places=12)
            self.assertTrue(np.all(np.diff(times) > 0.0))

    def test_time_grid_rejects_non_integral_duration(self) -> None:
        with self.assertRaises(ValueError):
            subject.evaluation_times(0.0, 1.005, 0.01)

    def test_reference_crop_is_derived_and_has_expected_cycle_shape(self) -> None:
        reference = {
            "heel_strike": np.asarray([0.5, 1.5, 2.5, 3.5, 4.5, 5.5]),
            "toe_off": np.asarray([1.0, 2.0, 3.0, 4.0, 5.0]),
        }
        crop = subject.crop_reference_events(
            reference,
            start_s=1.5,
            end_s=4.5,
        )
        np.testing.assert_array_equal(crop["heel_strike"], [1.5, 2.5, 3.5, 4.5])
        np.testing.assert_array_equal(crop["toe_off"], [2.0, 3.0, 4.0])
        shape = subject.reference_shape(crop)
        self.assertTrue(shape["strict_hs_to_toe_off_to_hs_order"])
        self.assertTrue(shape["supports_expected_4_hs_3_to_3_cycles"])

    def test_reference_shape_rejects_misordered_events(self) -> None:
        shape = subject.reference_shape(
            {
                "heel_strike": np.asarray([1.0, 2.0, 3.0, 4.0]),
                "toe_off": np.asarray([1.5, 2.5, 4.5]),
            }
        )
        self.assertFalse(shape["strict_hs_to_toe_off_to_hs_order"])
        self.assertFalse(shape["supports_expected_4_hs_3_to_3_cycles"])

    def test_ordered_timing_gate_uses_established_tolerance(self) -> None:
        passing = subject._ordered_timing_diagnostic(
            np.asarray([1.0, 2.0]),
            np.asarray([1.05, 1.96]),
            tolerance_s=subject.HS_TIMING_TOLERANCE_S,
        )
        failing = subject._ordered_timing_diagnostic(
            np.asarray([1.0, 2.0]),
            np.asarray([1.051, 1.96]),
            tolerance_s=subject.HS_TIMING_TOLERANCE_S,
        )
        self.assertTrue(passing["all_within_tolerance"])
        self.assertFalse(failing["all_within_tolerance"])

    def test_cli_refuses_to_overwrite_existing_summary(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            output_dir = Path(directory)
            (output_dir / "summary.json").write_text("owned\n", encoding="utf-8")
            result = subject.main(["--output-dir", str(output_dir)])
            self.assertEqual(result, 2)
            self.assertEqual(
                (output_dir / "summary.json").read_text(encoding="utf-8"),
                "owned\n",
            )


if __name__ == "__main__":
    unittest.main()
