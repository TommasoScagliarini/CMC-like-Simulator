"""Offline scientific matching checks; never import or run OpenSim."""

from __future__ import annotations

import importlib.util
import json
import tempfile
import unittest
from pathlib import Path


SPEC = importlib.util.spec_from_file_location(
    "ab07_analysis", Path(__file__).with_name("analyze_prescribed_v26.py")
)
assert SPEC is not None and SPEC.loader is not None
ANALYSIS = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(ANALYSIS)


class QualificationAnalysisTests(unittest.TestCase):
    def test_duplicate_predictions_cannot_credit_one_reference_twice(self) -> None:
        matching = ANALYSIS.existing_helper(ANALYSIS.MATCH_SOURCE, "match_events")
        result = matching([1.0, 2.0], [1.03, 1.04, 2.2], 0.05)
        self.assertEqual(result["matched_count"], 1)
        self.assertEqual(result["false_positives"], 2)
        self.assertEqual(result["false_negatives"], 1)
        self.assertEqual(result["pairs"][0]["predicted_time_s"], 1.03)

    def test_far_delayed_hs_are_unmatched_instead_of_becoming_valid_cycles(self) -> None:
        matching = ANALYSIS.existing_helper(ANALYSIS.MATCH_SOURCE, "match_events")
        result = matching([12.59174052, 14.08770469], [13.269, 14.908], 0.12)
        self.assertEqual(result["matched_count"], 0)
        self.assertEqual(result["false_negatives"], 2)
        self.assertEqual(result["false_positives"], 2)
        # Existing helper emits NaN for undefined matched-event error; receipts
        # convert it to null, while preserving the zero matches and missing events.
        encoded = json.dumps(ANALYSIS.json_value(result), allow_nan=False)
        self.assertIsNone(json.loads(encoded)["timing_mae_s"])

    def test_complete_cycle_csv_has_unique_hs_and_no_unanchored_partial_stance(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            source = Path(temporary) / "events.csv"
            source.write_text(
                "side,cycle_start,cycle_end,contact_duration_s\n"
                "left,1.0,2.0,0.6\nleft,2.0,3.0,0.7\nright,1.5,2.5,0.5\n",
                encoding="utf-8",
            )
            cycles, events = ANALYSIS.oracle_cycles(source, 0.5, 2.5)
        self.assertEqual(len(cycles), 1)
        self.assertEqual(events["heel_strike"], [1.0, 2.0])
        self.assertEqual(events["toe_off"], [1.6])

    def test_timing_tolerances_come_from_inherited_v26_source(self) -> None:
        values = ANALYSIS._v26_tolerances()
        self.assertEqual(values["HS_TOLERANCE_S"], 0.05)
        self.assertEqual(values["TO_TOLERANCE_S"], 0.08)
        self.assertEqual(values["DELIVERED_HS_TOLERANCE_S"], 0.06)
        self.assertEqual(values["DELIVERED_TO_TOLERANCE_S"], 0.09)


if __name__ == "__main__":
    unittest.main()
