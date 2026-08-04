"""Pure tests for the read-only V7 diagnostic plot post-processor."""

from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path


VALIDATION_ROOT = Path(__file__).resolve().parent
if str(VALIDATION_ROOT) not in sys.path:
    sys.path.insert(0, str(VALIDATION_ROOT))
import plot_two_sensor_timing_placement_v7_diagnostic as subject  # noqa: E402


class TwoSensorV7DiagnosticPlotTest(unittest.TestCase):
    def test_extracts_multiresolution_values_without_999_sentinel(self) -> None:
        diagnostic = subject.extract_diagnostic(subject.load_frozen_manifest())

        self.assertEqual(diagnostic["status"], "FAIL")
        self.assertEqual(diagnostic["sensors_per_pair"], 2)
        self.assertEqual(diagnostic["sensor_contract"], "1 heel + 1 forefoot")
        self.assertEqual(len(diagnostic["candidates"]), 10)

        by_id = {
            candidate["candidate_id"]: candidate
            for candidate in diagnostic["candidates"]
        }
        x25 = by_id["H02_X2p50_F76_P34p75"]["cadences"]
        self.assertEqual(x25["10 ms"]["accepted_hs"], 51)
        self.assertEqual(x25["1 ms"]["valid_cycles"], 50)
        self.assertAlmostEqual(x25["10 ms"]["hs_max_ms"], 130.39884275681857)
        self.assertAlmostEqual(x25["1 ms"]["to_max_ms"], 16.099656930371964)
        self.assertEqual(x25["1 ms"]["both_off_samples"], 2)
        self.assertEqual(x25["10 ms"]["causal_clear_ms"], 0.0)
        self.assertEqual(x25["1 ms"]["recontact_episodes"], 1)

        missing = by_id["H02_X3p00_F76_P34p75"]["cadences"]
        self.assertIsNone(missing["10 ms"]["hs_max_ms"])
        self.assertEqual(missing["10 ms"]["missing_hs"], 1)
        current = by_id["current_geometry"]["cadences"]
        self.assertIsNone(current["10 ms"]["hs_max_ms"])
        self.assertIsNone(current["1 ms"]["to_max_ms"])
        self.assertEqual(current["10 ms"]["missing_hs"], 4)
        self.assertEqual(current["1 ms"]["missing_to"], 2)

        numeric = [
            value
            for candidate in diagnostic["candidates"]
            for cadence in candidate["cadences"].values()
            for value in cadence.values()
            if isinstance(value, (int, float)) and not isinstance(value, bool)
        ]
        self.assertNotIn(999.0, numeric)
        self.assertNotIn(999000.0, numeric)

    def test_existing_output_directory_is_never_clobbered(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            output_dir = Path(temporary) / "already_exists"
            output_dir.mkdir()
            marker = output_dir / "owned.txt"
            marker.write_text("preserve", encoding="utf-8")

            with self.assertRaises(subject.NoClobberError):
                subject.plot_diagnostic(
                    {"candidates": [], "thresholds": {}}, output_dir
                )
            self.assertEqual(marker.read_text(encoding="utf-8"), "preserve")


if __name__ == "__main__":
    unittest.main()
