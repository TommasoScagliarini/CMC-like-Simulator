"""Tests for the V9 two-sensor morphology plot pipeline."""

from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

import numpy as np

from validation import plot_morphology_corridor_two_sensor_v9 as subject


class V9MorphologyPlotTests(unittest.TestCase):
    def test_detector_profile_is_exactly_two_v9_sensor_spheres(self) -> None:
        contract = subject.detector_contract(
            subject.DEFAULT_DETECTOR_PROFILE,
            subject.DEFAULT_V9_MANIFEST,
        )
        self.assertEqual(contract["candidate_id"], subject.V9_CANDIDATE_ID)
        self.assertEqual(contract["sphere_count"], 2)
        self.assertEqual(set(contract["roles"]), {"left_heel", "left_toe"})
        self.assertTrue(contract["detector_only"])
        self.assertFalse(contract["generates_grf"])

    def test_alignment_selects_last_native_sample_before_policy_end(self) -> None:
        state_times = 10.0 + np.arange(51, dtype=float) * 0.001
        trace_times = 10.01 + np.arange(5, dtype=float) * 0.01
        indices, report = subject.aligned_pre_step_indices(
            trace_times,
            state_times,
        )
        np.testing.assert_array_equal(indices, [9, 19, 29, 39, 49])
        self.assertAlmostEqual(report["lag_s_median"], 0.001, places=12)

    def test_complete_cycles_excludes_partial_bootstrap(self) -> None:
        accepted = [
            {
                "event": "toe_off",
                "event_time_s": 0.20,
                "segment_valid": 0.0,
                "cycle_valid": -1.0,
            },
            {
                "event": "heel_strike",
                "event_time_s": 0.60,
                "segment_valid": 1.0,
                "cycle_valid": 0.0,
            },
            {
                "event": "toe_off",
                "event_time_s": 1.10,
                "segment_valid": 1.0,
                "cycle_valid": -1.0,
            },
            {
                "event": "heel_strike",
                "event_time_s": 1.55,
                "segment_valid": 1.0,
                "cycle_valid": 1.0,
            },
        ]
        cycles = subject.complete_cycles(accepted)
        self.assertEqual(len(cycles), 1)
        self.assertEqual(cycles[0].heel_strike_s, 0.60)
        self.assertEqual(cycles[0].toe_off_s, 1.10)
        self.assertEqual(cycles[0].next_heel_strike_s, 1.55)

    def test_frozen_recorded_trace_matches_plotted_trace(self) -> None:
        served = subject._trace_rows(
            subject.MORPHOLOGY_RUN / "rollout_policy_trace.json"
        )
        recorded = subject._trace_rows(
            subject.REFERENCE_RUN / "rollout_policy_trace.json"
        )
        report = subject.trace_invariance(served, recorded)
        self.assertEqual(report["row_count"], 500)
        self.assertTrue(report["exact_for_plotted_fields"])

    def test_plot_helper_writes_nonempty_png(self) -> None:
        profile = subject.load_profile(subject.DEFAULT_MORPHOLOGY_PROFILE)
        times = np.linspace(1.0, 2.0, 101)
        rows = [
            {
                "time": float(time),
                "prosthetic_state": {
                    "pros_knee_angle_served_ref": -0.3,
                    "pros_ankle_angle_served_ref": 0.1,
                },
            }
            for time in times
        ]
        cycle = subject.CompletedCycle(0, 1.1, 1.5, 1.9)
        accepted = [
            {
                "event": "heel_strike",
                "event_time_s": 1.1,
                "confirmed_time_s": 1.13,
            },
            {
                "event": "toe_off",
                "event_time_s": 1.5,
                "confirmed_time_s": 1.53,
            },
            {
                "event": "heel_strike",
                "event_time_s": 1.9,
                "confirmed_time_s": 1.93,
            },
        ]
        reward = {
            "morphology_std_multiplier_knee": 1.6,
            "morphology_std_multiplier_ankle": 0.6,
            "morphology_margin_knee_deg": 7.5,
            "morphology_margin_ankle_deg": 7.5,
        }
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "plot.png"
            subject._plot(
                trace_rows=rows,
                cycles=[cycle],
                accepted=accepted,
                profile=profile,
                reward_config=reward,
                output_path=output,
            )
            self.assertTrue(output.is_file())
            self.assertGreater(output.stat().st_size, 10_000)


if __name__ == "__main__":
    unittest.main()
