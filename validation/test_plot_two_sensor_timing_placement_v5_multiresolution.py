"""Pure tests for the read-only V5 multiresolution post-processor."""

from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path


VALIDATION_ROOT = Path(__file__).resolve().parent
if str(VALIDATION_ROOT) not in sys.path:
    sys.path.insert(0, str(VALIDATION_ROOT))
import plot_two_sensor_timing_placement_v5_multiresolution as subject  # noqa: E402


class TwoSensorV5MultiresolutionPlotTest(unittest.TestCase):
    def test_extracts_frozen_p35_values_and_thresholds(self) -> None:
        manifest = subject.load_frozen_manifest()
        comparison = subject.extract_comparison(manifest)

        self.assertEqual(comparison["candidate_id"], "H02_X2_F80_P35p00")
        self.assertEqual(comparison["sensors_per_pair"], 2)
        rows = comparison["rows"]
        self.assertEqual([row["sample_dt_ms"] for row in rows], [10.0, 1.0])
        self.assertAlmostEqual(rows[0]["hs_max_ms"], 46.53345938879738)
        self.assertAlmostEqual(rows[1]["hs_max_ms"], 52.46885229447429)
        self.assertEqual([row["both_off_samples"] for row in rows], [0, 6])
        self.assertEqual([row["early_to_count"] for row in rows], [0, 1])
        self.assertEqual([row["valid_cycles"] for row in rows], [50, 50])
        self.assertEqual([row["gate_pass"] for row in rows], [True, False])
        self.assertEqual(
            comparison["thresholds"],
            {
                "hs_max_ms": 50.0,
                "to_max_ms": 80.0,
                "both_off_duration_ms": 0.0,
                "both_off_samples": 0,
                "early_to_count": 0,
                "release_margin_ms": 250.0,
                "valid_cycles": 50,
                "accepted_hs": 51,
                "accepted_to": 50,
            },
        )

    def test_existing_output_directory_is_never_clobbered(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            output_dir = Path(temporary) / "already_exists"
            output_dir.mkdir()
            marker = output_dir / "owned.txt"
            marker.write_text("preserve", encoding="utf-8")

            with self.assertRaises(subject.NoClobberError):
                subject.plot_comparison(
                    {"rows": [], "thresholds": {}}, output_dir
                )
            self.assertEqual(marker.read_text(encoding="utf-8"), "preserve")


if __name__ == "__main__":
    unittest.main()
