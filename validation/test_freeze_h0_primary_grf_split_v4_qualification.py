from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path
from unittest import mock

from validation import freeze_h0_primary_grf_split_v4_qualification as freezer
from validation import h0_primary_grf_split_v4_qualification_contract as contract


class QualificationFreezerTests(unittest.TestCase):
    def test_payload_freezes_v4_lineage_order_and_closed_authority(self) -> None:
        record = {"path": "fixture", "sha256": "a" * 64, "size_bytes": 1}
        prerequisites = {
            "post_holdout": {
                "status": "PASS_H0_PRIMARY_SPLIT_V4_POST_HOLDOUT_PREREQUISITE"
            },
            "noise": {"status": "PASS"},
        }
        with (
            mock.patch.object(
                freezer,
                "_validate_prerequisites",
                return_value=prerequisites,
            ),
            mock.patch.object(
                freezer,
                "_source_paths",
                return_value={"source": Path("source")},
            ),
            mock.patch.object(
                freezer,
                "_input_paths",
                return_value={"input": Path("input")},
            ),
            mock.patch.object(freezer, "_record", return_value=record),
        ):
            payload = freezer.build_payload(require_unoccupied=False)

        self.assertEqual(payload["schema_version"], 4)
        self.assertEqual(payload["protocol_id"], contract.PROTOCOL_ID)
        self.assertEqual(payload["source_protocol_id"], contract.SOURCE_PROTOCOL_ID)
        self.assertEqual(payload["candidate_id"], contract.CANDIDATE_ID)
        self.assertEqual(
            payload["execution_order"],
            [
                "all_six_condition_matched_analog_h0_baselines",
                "baseline_receipt",
                "baseline_tolerance_decision_receipt",
                "all_six_primary_split_v4_candidate_rollouts",
            ],
        )
        self.assertEqual(payload["post_holdout_prerequisite"], prerequisites["post_holdout"])
        self.assertFalse(payload["authority"]["zero_update_port_authorized"])
        self.assertFalse(payload["authority"]["protected_trial_access_authorized"])
        self.assertEqual(payload["protected_trials_opened"], [])

    def test_existing_lock_is_rejected_before_prerequisite_reads(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            destination = root / "qualification_lock.json"
            destination.write_text("occupied\n", encoding="utf-8")
            with (
                mock.patch.object(freezer, "REPO_ROOT", root),
                mock.patch.object(freezer, "DESTINATION", destination),
                mock.patch.object(freezer, "_validate_prerequisites") as validate,
            ):
                with self.assertRaisesRegex(
                    freezer.QualificationFreezeError,
                    "refusing to clobber",
                ):
                    freezer.build_payload(require_unoccupied=True)
            validate.assert_not_called()

    def test_freeze_preallocates_exactly_twelve_empty_rollout_directories(self) -> None:
        destinations = [
            contract.rollout_destination(role, case_id).as_posix()
            for role in ("baseline", "candidate")
            for case_id in contract.CASE_IDS
        ]
        payload = {"destinations": destinations, "status": contract.LOCK_STATUS}

        def write_exclusive(path: Path, value: object) -> Path:
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(
                json.dumps(value, sort_keys=True, allow_nan=False) + "\n",
                encoding="utf-8",
            )
            return path

        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            destination = root / contract.LOCK_PATH
            with (
                mock.patch.object(freezer, "REPO_ROOT", root),
                mock.patch.object(freezer, "DESTINATION", destination),
                mock.patch.object(freezer, "build_payload", return_value=payload),
                mock.patch.object(
                    freezer.v4,
                    "_write_json_exclusive",
                    side_effect=write_exclusive,
                ),
            ):
                observed = freezer.freeze()

            self.assertEqual(observed, payload)
            self.assertTrue(destination.is_file())
            self.assertEqual(len(destinations), 12)
            for relative in destinations:
                rollout = root / relative
                self.assertTrue(rollout.is_dir())
                self.assertEqual(list(rollout.iterdir()), [])
            self.assertTrue((root / contract.RUN_ROOT / "gates").is_dir())


if __name__ == "__main__":
    unittest.main()
