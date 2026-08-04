"""Synthetic tests for the isolated EPIC AB06 recovery converter."""

from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np

from tools import convert_epic_ab06_tables_recovery as subject


def _convert_synthetic(
    root: Path,
    *,
    dataset_ik_time: np.ndarray,
    force_plate_time: np.ndarray,
    marker_time: np.ndarray,
    required_time_range_s: tuple[float, float] | None,
) -> tuple[dict[str, object], Path]:
    coordinate_data = subject.original.CoordinateData(
        time=dataset_ik_time,
        labels=subject.original.TARGET_COORDINATES,
        values=np.zeros(
            (dataset_ik_time.size, len(subject.original.TARGET_COORDINATES))
        ),
    )
    grf_data = subject.original.GrfData(
        time=force_plate_time,
        labels=subject.original.GRF_LABELS,
        values=np.zeros(
            (force_plate_time.size, len(subject.original.GRF_LABELS))
        ),
    )
    marker_data = subject.original.MarkerData(
        time=marker_time,
        names=subject.original.MARKER_NAMES,
        values=np.zeros(
            (marker_time.size, len(subject.original.MARKER_NAMES), 3)
        ),
        units="mm",
        rate_hz=100.0,
    )
    sources = []
    for folder in ("ik", "fp", "markers"):
        source_dir = root / folder
        source_dir.mkdir()
        source = source_dir / "synthetic_01.mat"
        source.write_bytes(folder.encode("ascii"))
        sources.append(source)
    output = root / "converted"

    with (
        patch.object(
            subject.original,
            "decode_mcos_table",
            side_effect=[object(), object(), object()],
        ),
        patch.object(
            subject.original,
            "extract_coordinate_data",
            return_value=coordinate_data,
        ),
        patch.object(
            subject.original,
            "extract_grf_data",
            return_value=grf_data,
        ),
        patch.object(
            subject.original,
            "extract_marker_data",
            return_value=marker_data,
        ),
    ):
        result = subject.convert_trial(
            ik_mat=sources[0],
            fp_mat=sources[1],
            markers_mat=sources[2],
            output_dir=output,
            trial="synthetic_01",
            required_time_range_s=required_time_range_s,
        )
    return result, output


class RecoveryTimeCoverageTests(unittest.TestCase):
    def test_mismatched_source_endpoints_covering_protocol_are_accepted(self) -> None:
        coverage = subject.validate_source_time_coverage(
            dataset_ik_time=np.asarray([9.998, 20.0]),
            force_plate_time=np.asarray([9.999, 15.0, 20.001]),
            marker_time=np.asarray([10.0, 20.002]),
            required_time_range_s=(10.0, 20.0),
        )

        self.assertEqual(
            coverage["streams"]["force_plate"]["time_range_s"],
            [9.999, 20.001],
        )
        self.assertTrue(coverage["all_sources_cover_required_range"])
        self.assertFalse(coverage["dataset_ik_used_downstream"])

    def test_source_not_covering_required_range_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            output = root / "converted"
            with self.assertRaisesRegex(
                subject.TableSchemaError,
                "does not fully cover",
            ) as caught:
                _convert_synthetic(
                    root,
                    dataset_ik_time=np.asarray([9.999, 20.0]),
                    force_plate_time=np.asarray([9.999, 15.0, 20.001]),
                    marker_time=np.asarray([10.0, 20.0]),
                    required_time_range_s=(9.999, 20.0),
                )
            self.assertIn("uncovered=['markers']", str(caught.exception))
            self.assertFalse(output.exists())

    def test_equal_endpoint_legacy_inference_is_preserved(self) -> None:
        coverage = subject.validate_source_time_coverage(
            dataset_ik_time=np.asarray([10.0, 20.0]),
            force_plate_time=np.asarray([10.0, 15.0, 20.0]),
            marker_time=np.asarray([10.0, 20.0]),
            required_time_range_s=None,
        )
        self.assertEqual(
            coverage["downstream_required_time_range_s"],
            [10.0, 20.0],
        )
        self.assertEqual(coverage["range_origin"], "legacy_equal_source_endpoints")

    def test_conversion_records_schema1_coverage_without_clipping_grf(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            force_plate_time = np.asarray([9.999, 15.0, 20.001])
            result, _output = _convert_synthetic(
                root,
                dataset_ik_time=np.asarray([9.998, 20.0]),
                force_plate_time=force_plate_time,
                marker_time=np.asarray([10.0, 20.002]),
                required_time_range_s=(10.0, 20.0),
            )

            manifest = json.loads(
                Path(result["conversion_manifest"]).read_text(encoding="utf-8")
            )
            coverage = manifest["source_time_coverage"]
            self.assertEqual(
                manifest["schema_version"],
                subject.original.CONVERSION_MANIFEST_SCHEMA_VERSION,
            )
            self.assertEqual(
                manifest["recovery_lineage"]["original_converter"]["sha256"],
                subject.ORIGINAL_CONVERTER_SHA256,
            )
            self.assertEqual(
                coverage["downstream_required_time_range_s"],
                [10.0, 20.0],
            )
            self.assertTrue(coverage["all_sources_cover_required_range"])
            self.assertFalse(coverage["dataset_ik_used_downstream"])
            self.assertEqual(
                coverage["streams"],
                {
                    "dataset_ik": {
                        "time_range_s": [9.998, 20.0],
                        "rows": 2,
                        "covers_required_range": True,
                    },
                    "force_plate": {
                        "time_range_s": [9.999, 20.001],
                        "rows": 3,
                        "covers_required_range": True,
                    },
                    "markers": {
                        "time_range_s": [10.0, 20.002],
                        "rows": 2,
                        "covers_required_range": True,
                    },
                },
            )
            lines = Path(result["grf_mot"]).read_text(encoding="utf-8").splitlines()
            header = lines.index("endheader")
            labels = next(
                index
                for index in range(header + 1, len(lines))
                if lines[index].strip()
            )
            grf_times = np.loadtxt(lines[labels + 1 :], ndmin=2)[:, 0]
            np.testing.assert_array_equal(grf_times, force_plate_time)

    def test_original_converter_identity_drift_fails_before_output(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            output = root / "converted"
            with (
                patch.object(subject, "ORIGINAL_CONVERTER_SHA256", "0" * 64),
                self.assertRaisesRegex(
                    subject.ConversionError,
                    "original converter identity drifted",
                ),
            ):
                _convert_synthetic(
                    root,
                    dataset_ik_time=np.asarray([10.0, 20.0]),
                    force_plate_time=np.asarray([10.0, 15.0, 20.0]),
                    marker_time=np.asarray([10.0, 20.0]),
                    required_time_range_s=(10.0, 20.0),
                )
            self.assertFalse(output.exists())


if __name__ == "__main__":
    unittest.main()
