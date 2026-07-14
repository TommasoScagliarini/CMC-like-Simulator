from __future__ import annotations

import sys
import unittest
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from validation.validate_online_grf_events import (
    contact_metrics,
    cycles_from_events,
    event_metrics,
    match_events,
    strict_event_pass,
)


class OnlineGRFEventMatchingTests(unittest.TestCase):
    def test_match_events_perfect_match(self) -> None:
        metrics = match_events([1.0, 2.0, 3.0], [1.0, 2.0, 3.0], 0.05)
        self.assertEqual(metrics["matched_count"], 3)
        self.assertEqual(metrics["false_positives"], 0)
        self.assertEqual(metrics["false_negatives"], 0)
        self.assertEqual(metrics["precision"], 1.0)
        self.assertEqual(metrics["recall"], 1.0)
        self.assertEqual(metrics["timing_max_abs_s"], 0.0)

    def test_match_events_detects_spurious_heel_strike(self) -> None:
        metrics = match_events([1.0, 2.0], [0.7, 1.0, 2.0], 0.05)
        self.assertEqual(metrics["matched_count"], 2)
        self.assertEqual(metrics["false_positives"], 1)
        self.assertEqual(metrics["false_negatives"], 0)
        self.assertLess(metrics["precision"], 1.0)
        self.assertEqual(metrics["recall"], 1.0)
        self.assertEqual(metrics["unmatched_predicted"], [0.7])

    def test_event_metrics_detects_missing_toe_off(self) -> None:
        reference = {
            "heel_strikes": np.asarray([1.0, 2.0]),
            "toe_offs": np.asarray([1.5, 2.5]),
        }
        predicted = {
            "heel_strikes": np.asarray([1.0, 2.0]),
            "toe_offs": np.asarray([1.5]),
        }
        metrics = event_metrics(
            reference,
            predicted,
            hs_tolerance_s=0.05,
            to_tolerance_s=0.08,
        )
        self.assertTrue(
            strict_event_pass(
                {"heel_strike": metrics["heel_strike"], "toe_off": metrics["toe_off"]},
                hs_tolerance_s=0.05,
                to_tolerance_s=0.08,
            )
            is False
        )
        self.assertEqual(metrics["toe_off"]["false_negatives"], 1)
        self.assertLess(metrics["toe_off"]["recall"], 1.0)

    def test_shift_outside_tolerance_is_unmatched(self) -> None:
        metrics = match_events([1.0], [1.061], 0.05)
        self.assertEqual(metrics["matched_count"], 0)
        self.assertEqual(metrics["false_positives"], 1)
        self.assertEqual(metrics["false_negatives"], 1)
        self.assertFalse(np.isfinite(metrics["timing_max_abs_s"]))

    def test_strict_gate_accepts_timing_inside_limits(self) -> None:
        reference = {
            "heel_strikes": np.asarray([1.0, 2.0]),
            "toe_offs": np.asarray([1.6, 2.6]),
        }
        predicted = {
            "heel_strikes": np.asarray([1.02, 1.99]),
            "toe_offs": np.asarray([1.55, 2.67]),
        }
        metrics = event_metrics(
            reference,
            predicted,
            hs_tolerance_s=0.05,
            to_tolerance_s=0.08,
        )
        self.assertTrue(
            strict_event_pass(
                metrics,
                hs_tolerance_s=0.05,
                to_tolerance_s=0.08,
            )
        )

    def test_cycles_from_events_reports_stance_period_and_duty(self) -> None:
        rows = cycles_from_events(
            {
                "heel_strikes": np.asarray([1.0, 2.0, 3.0]),
                "toe_offs": np.asarray([1.6, 2.55]),
            }
        )
        self.assertEqual(len(rows), 3)
        self.assertAlmostEqual(rows[0]["stance_duration_s"], 0.6)
        self.assertAlmostEqual(rows[0]["period_s"], 1.0)
        self.assertAlmostEqual(rows[0]["duty_factor"], 0.6)
        self.assertTrue(np.isnan(rows[-1]["period_s"]))

    def test_contact_metrics_are_secondary_shape_scores(self) -> None:
        metrics = contact_metrics(
            np.asarray([0.0, 30.0, 30.0, 0.0]),
            np.asarray([0.0, 25.0, 0.0, 25.0]),
            reference_threshold_n=20.0,
            predicted_threshold_n=20.0,
        )
        self.assertAlmostEqual(metrics["precision"], 0.5)
        self.assertAlmostEqual(metrics["recall"], 0.5)
        self.assertAlmostEqual(metrics["iou"], 1.0 / 3.0)


if __name__ == "__main__":
    unittest.main()
