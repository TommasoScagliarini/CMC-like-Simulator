"""Pure and synthetic tests for the V14 metadata-only trial audit."""

from __future__ import annotations

import copy
import hashlib
import json
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np


VALIDATION_ROOT = Path(__file__).resolve().parent
if str(VALIDATION_ROOT) not in sys.path:
    sys.path.insert(0, str(VALIDATION_ROOT))

import audit_two_sensor_v14_trial_metadata as subject  # noqa: E402


def _frozen_metadata(trial_id: str) -> subject.ConditionMetadata:
    frozen = subject.FROZEN_CONDITIONS[trial_id]
    trial_start, trial_end = frozen["trial_interval_s"]
    table_start, table_end = frozen["table_time_interval_s"]
    return subject.ConditionMetadata(
        trial_id=trial_id,
        trial_start_s=trial_start,
        trial_end_s=trial_end,
        columns=subject.EXPECTED_COLUMNS,
        units=subject.EXPECTED_UNITS,
        table_sample_count=frozen["sample_count"],
        table_time_start_s=table_start,
        table_time_end_s=table_end,
        table_sample_dt_s=subject.EXPECTED_SAMPLE_DT_S,
        table_time_grid_uniform=True,
        plateaus=frozen["plateaus"],
    )


def _make_opaque_inventory(root: Path) -> None:
    for trial_id in subject.TRIAL_IDS:
        for stream in subject.INVENTORY_STREAMS:
            path = root / stream / f"treadmill_{trial_id}_01.mat"
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(f"opaque:{stream}:{trial_id}".encode("ascii"))


class FrozenContractTests(unittest.TestCase):
    def test_exact_split_is_disjoint_complete_and_interpolative(self) -> None:
        checks = subject.validate_split(subject.FROZEN_SPLIT)
        self.assertTrue(all(checks.values()))
        self.assertEqual(subject.FROZEN_SPLIT["DEVELOPMENT"], ("02", "04", "08"))
        self.assertEqual(subject.FROZEN_SPLIT["VALIDATION"], ("05",))
        self.assertEqual(subject.FROZEN_SPLIT["SEALED"], ("06",))
        self.assertEqual(subject.FROZEN_SPLIT["RESERVE"], ("03", "07"))
        self.assertEqual(subject.FROZEN_SPLIT["CONSUMED"], ("01",))

        conditions = {
            trial_id: _frozen_metadata(trial_id)
            for trial_id in subject.SEMANTIC_CONDITION_TRIAL_IDS
        }
        interpolation, rows = subject._cross_speed_interpolation_checks(conditions)
        self.assertTrue(all(interpolation.values()))
        self.assertEqual(len(rows), 4)
        self.assertTrue(
            all(
                row["development_mps"]["04"]
                < row["validation_05_mps"]
                < row["sealed_06_mps"]
                < row["development_mps"]["08"]
                for row in rows
            )
        )

    def test_all_frozen_condition_records_pass(self) -> None:
        for trial_id in subject.SEMANTIC_CONDITION_TRIAL_IDS:
            with self.subTest(trial_id=trial_id):
                checks = subject.validate_condition_metadata(
                    _frozen_metadata(trial_id)
                )
                self.assertTrue(all(checks.values()))

    def test_speed_or_unit_drift_fails_closed(self) -> None:
        original = _frozen_metadata("05")
        changed_plateaus = list(original.plateaus)
        first = changed_plateaus[0]
        changed_plateaus[0] = subject.Plateau(
            start_s=first.start_s,
            end_s=first.end_s,
            speed_mps=first.speed_mps + 0.01,
            sample_count=first.sample_count,
        )
        speed_drift = subject.ConditionMetadata(
            **{
                **original.__dict__,
                "plateaus": tuple(changed_plateaus),
            }
        )
        self.assertFalse(
            subject.validate_condition_metadata(speed_drift)[
                "exact_plateau_intervals_and_speeds"
            ]
        )

        unit_drift = subject.ConditionMetadata(
            **{**original.__dict__, "units": ("s", "km/h")}
        )
        self.assertFalse(
            subject.validate_condition_metadata(unit_drift)["exact_units"]
        )

    def test_constant_plateau_extractor_ignores_ramps_and_zero_runs(self) -> None:
        times = np.arange(0.0, 100.001, 0.001)
        speeds = np.zeros(times.size)
        ramp = (times >= 5.0) & (times < 10.0)
        speeds[ramp] = np.linspace(
            0.0, 0.5, int(np.count_nonzero(ramp)), endpoint=False
        )
        plateau = (times >= 10.0) & (times <= 45.0)
        speeds[plateau] = 0.5
        short = (times >= 50.0) & (times <= 55.0)
        speeds[short] = 0.8

        observed = subject._constant_nonzero_plateaus(times, speeds)
        self.assertEqual(len(observed), 1)
        self.assertAlmostEqual(observed[0].start_s, 10.0)
        self.assertAlmostEqual(observed[0].end_s, 45.0)
        self.assertAlmostEqual(observed[0].speed_mps, 0.5)


class AccessBoundaryTests(unittest.TestCase):
    def test_builder_semantically_reads_only_conditions_02_08(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary) / "treadmill"
            _make_opaque_inventory(root)
            decoded: list[Path] = []

            def reader(path: str | Path) -> subject.ConditionMetadata:
                source = Path(path)
                decoded.append(source)
                self.assertEqual(source.parent.name, "conditions")
                trial_id = subject._trial_id_from_path(source)
                self.assertIn(trial_id, subject.SEMANTIC_CONDITION_TRIAL_IDS)
                return _frozen_metadata(trial_id)

            payload = subject.build_metadata_audit(
                root,
                repo_root=Path(temporary),
                condition_reader=reader,
            )

            self.assertTrue(payload["ok"])
            self.assertEqual(
                [subject._trial_id_from_path(path) for path in decoded],
                list(subject.SEMANTIC_CONDITION_TRIAL_IDS),
            )
            self.assertEqual(
                payload["inventory"]["01"]["streams"]["conditions"][
                    "access_mode"
                ],
                "opaque_stat_and_sha256_only",
            )
            for trial_id in subject.TRIAL_IDS:
                for stream in subject.INVENTORY_STREAMS:
                    mode = payload["inventory"][trial_id]["streams"][stream][
                        "access_mode"
                    ]
                    semantic = (
                        stream == "conditions"
                        and trial_id in subject.SEMANTIC_CONDITION_TRIAL_IDS
                    )
                    self.assertEqual(
                        mode,
                        (
                            "conditions_metadata_semantic_allowlist"
                            if semantic
                            else (
                                "opaque_stat_and_sha256_only_"
                                "forbidden_as_downstream_kinematics"
                                if stream in {"ik", "ik_offset"}
                                else "opaque_stat_and_sha256_only"
                            )
                        ),
                    )

    def test_semantic_reader_rejects_condition_01_and_noncondition_stream(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            condition_01 = root / "conditions/treadmill_01_01.mat"
            condition_01.parent.mkdir(parents=True)
            condition_01.write_bytes(b"not parsed")
            with self.assertRaises(subject.MetadataAuditError):
                subject.read_condition_metadata(condition_01)

            fp_02 = root / "fp/treadmill_02_01.mat"
            fp_02.parent.mkdir(parents=True)
            fp_02.write_bytes(b"not parsed")
            with self.assertRaises(subject.MetadataAuditError):
                subject.read_condition_metadata(fp_02)

    def test_inventory_hashes_opaque_bytes_without_decoding(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            path = root / "ik/treadmill_02_01.mat"
            path.parent.mkdir(parents=True)
            payload = b"opaque-ik-values-must-not-be-decoded"
            path.write_bytes(payload)
            record = subject._identity_record(
                path, root, "opaque_stat_and_sha256_only"
            )
            self.assertEqual(record["size_bytes"], len(payload))
            self.assertEqual(record["sha256"], hashlib.sha256(payload).hexdigest())

    def test_missing_inventory_fails_required_gate(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary) / "treadmill"
            _make_opaque_inventory(root)
            (root / "ik/treadmill_04_01.mat").unlink()
            payload = subject.build_metadata_audit(
                root,
                repo_root=Path(temporary),
                condition_reader=lambda path: _frozen_metadata(
                    subject._trial_id_from_path(Path(path))
                ),
            )
            self.assertFalse(payload["ok"])
            self.assertFalse(payload["checks"]["required"]["inventory_complete"])
            self.assertEqual(len(payload["missing_inventory_paths"]), 1)


class OutputTests(unittest.TestCase):
    def test_downstream_ik_and_toe_radius_contracts_are_unambiguous(self) -> None:
        ik = subject.DOWNSTREAM_IK_CONTRACT
        self.assertEqual(ik["opensim_version"], "4.5.2")
        self.assertEqual(ik["marker_count"], 28)
        self.assertTrue(ik["all_marker_tasks_apply"])
        self.assertEqual(ik["all_marker_task_weights"], 1.0)
        self.assertEqual(ik["accuracy"], 1.0e-5)
        self.assertFalse(ik["dataset_ik_allowed"])
        self.assertFalse(ik["ik_smoke_allowed"])

        detector = subject.DOWNSTREAM_DETECTOR_CONTRACT
        self.assertEqual(detector["toe_radius_parameter"], "reduction_from_V13")
        self.assertEqual(
            detector["toe_radius_reduction_grid_mm"],
            [0.0, 0.05, 0.10, 0.15, 0.20, 0.25],
        )
        self.assertFalse(detector["toe_radius_increase_allowed"])
        self.assertEqual(detector["expected_detector_stations"], 15)
        self.assertEqual(detector["expected_total_sampled_stations"], 23)

    def test_json_output_is_no_clobber_and_records_non_actions(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary) / "audit/metadata.json"
            payload = {
                "ok": True,
                "non_actions": {"ik_semantically_decoded": False},
            }
            written = subject.write_json_no_clobber(output, payload)
            self.assertEqual(json.loads(written.read_text(encoding="utf-8")), payload)
            with self.assertRaises(subject.MetadataAuditError):
                subject.write_json_no_clobber(output, copy.deepcopy(payload))


if __name__ == "__main__":
    unittest.main()
