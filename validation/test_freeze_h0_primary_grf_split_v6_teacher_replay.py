"""Pure tests for V6 teacher-replay preflight and freeze closure."""

from __future__ import annotations

import tempfile
import unittest
from pathlib import Path
from unittest import mock

from validation import (
    build_h0_primary_grf_split_v6_teacher_replay_preflight as preflight,
)
from validation import freeze_h0_primary_grf_split_v6_teacher_replay as freezer
from validation import h0_primary_grf_split_v6_teacher_replay_contract as contract


class FrozenPrerequisiteTests(unittest.TestCase):
    def test_v5_terminal_and_six_baselines_validate_read_only(self) -> None:
        terminal = preflight.validate_v5_terminal()
        self.assertEqual(
            terminal["status"],
            "FAIL_H0_PRIMARY_SPLIT_V5_AUTONOMOUS_QUALIFICATION",
        )
        for case_id in contract.CASE_IDS:
            with self.subTest(case_id=case_id):
                result = preflight.validate_baseline_case(case_id)
                self.assertEqual(result["rows"], contract.EXPECTED_STEPS)
                self.assertEqual(
                    result["reclassification"],
                    "V5_PASS_BASELINE_TO_V6_DEVELOPMENT_ONLY",
                )

    def test_h0_v25_and_hash_closures_are_complete(self) -> None:
        source_h0 = preflight.validate_source_h0_layout()
        v25 = preflight.validate_v25_active()
        self.assertEqual(source_h0["source_h0_id"], contract.SOURCE_H0_ID)
        self.assertEqual(v25["candidate_id"], "v25_4b351f67b5b86ab0")
        sources = preflight.source_paths()
        inputs = preflight.input_paths()
        self.assertEqual(set(sources), set(contract.SOURCE_RELATIVE_PATHS))
        self.assertGreater(len(inputs), len(contract.INPUT_RELATIVE_PATHS))
        self.assertIn("residual_module", sources)
        self.assertIn("residual_module_tests", sources)
        for path in (*sources.values(), *inputs.values()):
            self.assertTrue(path.is_file(), path)
            self.assertFalse(path.is_symlink(), path)


class NoClobberTests(unittest.TestCase):
    def test_preflight_publish_refuses_existing_receipt(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            target = Path(temporary) / "preflight.json"
            target.write_text('{"existing": true}\n', encoding="utf-8")
            original = target.read_bytes()
            with (
                mock.patch.object(preflight, "PREFLIGHT_PATH", target),
                mock.patch.object(
                    preflight,
                    "build_payload",
                    return_value={"status": "PASS", "passed": True},
                ),
                self.assertRaises(Exception),
            ):
                preflight.publish()
            self.assertEqual(target.read_bytes(), original)

    def test_freeze_refuses_an_existing_lock_before_other_work(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            lock = Path(temporary) / "lock.json"
            lock.write_text('{"existing": true}\n', encoding="utf-8")
            with (
                mock.patch.object(freezer, "LOCK_PATH", lock),
                mock.patch.object(freezer, "build_payload") as build_payload,
                self.assertRaises(freezer.V6TeacherReplayFreezeError),
            ):
                freezer.freeze()
            build_payload.assert_not_called()

    def test_lock_payload_carries_authority_and_persist_before_gate(self) -> None:
        fake_receipt = {
            "sources": {"source": {"path": "source", "sha256": "a", "size_bytes": 1}},
            "inputs": {"input": {"path": "input", "sha256": "b", "size_bytes": 1}},
            "v5_terminal": {"status": "terminal"},
            "v25": {"candidate_id": "v25_4b351f67b5b86ab0"},
            "source_h0": {"source_h0_id": contract.SOURCE_H0_ID},
        }
        with (
            mock.patch.object(
                freezer,
                "verify_preflight",
                return_value=fake_receipt,
            ),
            mock.patch.object(
                freezer.preflight,
                "source_record",
                return_value={"path": "preflight", "sha256": "c", "size_bytes": 1},
            ),
        ):
            payload = freezer.build_payload()
        self.assertEqual(
            payload["gate"]["persist_before_gate"],
            ["trace.json", "partial_summary.json", "summary.json"],
        )
        self.assertEqual(payload["authority"], contract.AUTHORITY)
        self.assertEqual(
            payload["execution_claim"],
            contract.EXECUTION_CLAIM_PATH.as_posix(),
        )
        self.assertEqual(payload["execution_order"], list(contract.CASE_IDS))
        self.assertFalse(payload["candidate_created"])
        self.assertEqual(payload["protected_trials_opened"], [])


if __name__ == "__main__":
    unittest.main()
