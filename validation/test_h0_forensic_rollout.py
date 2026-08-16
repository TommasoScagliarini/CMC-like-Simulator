"""Unit tests for the crash-safe H0 forensic rollout writer."""

from __future__ import annotations

import json
import math
import tempfile
import unittest
from pathlib import Path
from unittest import mock

from validation import h0_forensic_rollout as forensic


class StrictJsonTests(unittest.TestCase):
    def test_writer_rejects_nested_nan_and_infinity_without_artifact(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            for name, value in (
                ("nan", math.nan),
                ("positive_infinity", math.inf),
                ("negative_infinity", -math.inf),
            ):
                with self.subTest(name=name):
                    target = root / name / "value.json"
                    with self.assertRaises(forensic.ForensicRolloutError):
                        forensic.write_json_exclusive(
                            target,
                            {"nested": [{"value": value}]},
                        )
                    self.assertFalse(target.exists())
                    self.assertFalse(target.parent.exists())

    def test_loader_rejects_nonfinite_constants_and_duplicate_keys(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            nonfinite = root / "nonfinite.json"
            duplicate = root / "duplicate.json"
            nonfinite.write_text('{"value": NaN}\n', encoding="utf-8")
            duplicate.write_text('{"value": 1, "value": 2}\n', encoding="utf-8")
            with self.assertRaises(forensic.ForensicRolloutError):
                forensic.strict_json_load(nonfinite)
            with self.assertRaises(forensic.ForensicRolloutError):
                forensic.strict_json_load(duplicate)

    def test_publish_uses_fsync_and_atomic_replace(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            target = Path(temporary) / "artifact.json"
            original_replace = forensic.os.replace
            original_fsync = forensic.os.fsync
            with (
                mock.patch.object(
                    forensic.os,
                    "replace",
                    wraps=original_replace,
                ) as replace,
                mock.patch.object(
                    forensic.os,
                    "fsync",
                    wraps=original_fsync,
                ) as fsync,
            ):
                forensic.write_json_exclusive(target, {"value": 1})
            self.assertEqual(replace.call_count, 1)
            self.assertGreaterEqual(fsync.call_count, 1)
            self.assertEqual(forensic.strict_json_load(target), {"value": 1})


class RolloutLifecycleTests(unittest.TestCase):
    def _writer(self, root: Path) -> forensic.ForensicRolloutWriter:
        return forensic.ForensicRolloutWriter(
            root / "run",
            artifact_root=root,
        )

    def test_run_start_and_steps_are_strictly_no_clobber_and_contiguous(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            writer = self._writer(root)
            start_record = writer.start({"status": "STARTED", "seed": 129})
            self.assertEqual(start_record["path"], "run/run_start.json")
            original = writer.run_start_path.read_bytes()
            with self.assertRaises(forensic.ForensicRolloutError):
                writer.start({"status": "DIFFERENT"})
            self.assertEqual(writer.run_start_path.read_bytes(), original)

            with self.assertRaises(forensic.ForensicRolloutError):
                writer.write_step(2, {"time_s": 0.02})
            first = writer.write_step(1, {"time_s": 0.01})
            self.assertEqual(first["path"], "run/steps/000001.json")
            self.assertEqual(writer.last_completed_step, 1)
            first_bytes = (writer.steps_directory / "000001.json").read_bytes()
            with self.assertRaises(forensic.ForensicRolloutError):
                writer.write_step(1, {"time_s": 99.0})
            self.assertEqual(
                (writer.steps_directory / "000001.json").read_bytes(),
                first_bytes,
            )
            with self.assertRaises(forensic.ForensicRolloutError):
                writer.write_step(2, {"step": 3, "time_s": 0.02})
            writer.write_step(2, {"step": 2, "time_s": 0.02})
            self.assertEqual(writer.last_completed_step, 2)

    def test_partial_crash_receipt_survives_new_writer_instance(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            writer = self._writer(root)
            writer.start({"status": "STARTED"})
            writer.write_step(1, {"time_s": 0.01, "value": 10})
            writer.write_step(2, {"time_s": 0.02, "value": 20})

            recovered = self._writer(root)
            failure_record = recovered.publish_failure(
                end_reason="worker_exception",
                error=RuntimeError("simulated crash"),
                details={"worker_exit_code": 2},
            )
            self.assertEqual(failure_record["path"], "run/failure.json")
            failure = forensic.strict_json_load(recovered.failure_path)
            self.assertEqual(failure["last_completed_step"], 2)
            self.assertEqual(failure["end_reason"], "worker_exception")
            self.assertEqual(failure["error"]["type"], "RuntimeError")
            self.assertEqual(failure["error"]["message"], "simulated crash")
            self.assertEqual(failure["details"], {"worker_exit_code": 2})
            self.assertEqual(len(failure["artifacts"]["steps"]), 2)
            for record in failure["artifacts"]["steps"]:
                artifact = root / record["path"]
                self.assertEqual(
                    record,
                    forensic.artifact_record(artifact, artifact_root=root),
                )
            self.assertNotIn("trace", failure["artifacts"])
            self.assertNotIn("summary", failure["artifacts"])
            with self.assertRaises(forensic.ForensicRolloutError):
                recovered.write_step(3, {"time_s": 0.03})
            with self.assertRaises(forensic.ForensicRolloutError):
                recovered.publish_failure(
                    end_reason="second_failure",
                    error="must not overwrite",
                )

    def test_finalize_prevalidates_all_payloads_before_first_write(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            writer = self._writer(root)
            writer.start({"status": "STARTED"})
            writer.write_step(1, {"time_s": 0.01})
            with self.assertRaises(forensic.ForensicRolloutError):
                writer.finalize_before_gate(
                    partial_summary={"steps": 1},
                    summary={"metric": math.nan},
                )
            self.assertFalse(writer.trace_path.exists())
            self.assertFalse(writer.partial_summary_path.exists())
            self.assertFalse(writer.summary_path.exists())

    def test_gate_runs_only_after_trace_and_both_summaries(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            writer = self._writer(root)
            writer.start({"status": "STARTED"})
            writer.write_step(1, {"time_s": 0.01, "value": 10})
            writer.write_step(2, {"time_s": 0.02, "value": 20})
            called = False

            def gate(records: dict) -> dict:
                nonlocal called
                called = True
                self.assertTrue(writer.trace_path.is_file())
                self.assertTrue(writer.partial_summary_path.is_file())
                self.assertTrue(writer.summary_path.is_file())
                self.assertEqual(
                    [
                        row["step"]
                        for row in forensic.strict_json_load(writer.trace_path)
                    ],
                    [1, 2],
                )
                self.assertEqual(
                    set(records),
                    {"trace", "partial_summary", "summary"},
                )
                return {"status": "PASS_TEST_GATE", "passed": True}

            with self.assertRaises(forensic.ForensicRolloutError):
                writer.run_gate(gate)
            self.assertFalse(called)
            finalized = writer.finalize_before_gate(
                partial_summary={"steps": 2, "complete": True},
                summary={"steps": 2, "end_reason": "episode_time_limit"},
            )
            self.assertEqual(
                set(finalized),
                {"trace", "partial_summary", "summary"},
            )
            gate_record = writer.run_gate(gate)
            self.assertTrue(called)
            self.assertEqual(gate_record["path"], "run/gate.json")
            self.assertEqual(
                forensic.strict_json_load(writer.gate_path),
                {"status": "PASS_TEST_GATE", "passed": True},
            )
            with self.assertRaises(forensic.ForensicRolloutError):
                writer.finalize_before_gate(
                    partial_summary={"steps": 2},
                    summary={"steps": 2},
                )
            with self.assertRaises(forensic.ForensicRolloutError):
                writer.publish_gate({"status": "SECOND_GATE"})

    def test_failure_after_gate_records_every_visible_artifact(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            writer = self._writer(root)
            writer.start({"status": "STARTED"})
            writer.write_step(1, {"time_s": 0.01})
            writer.finalize_before_gate(
                partial_summary={"steps": 1},
                summary={"steps": 1},
            )
            writer.publish_gate({"status": "FAIL_TEST_GATE", "passed": False})
            writer.publish_failure(
                end_reason="gate_failed",
                error={"type": "GateFailure", "message": "gate returned false"},
            )
            failure = forensic.strict_json_load(writer.failure_path)
            self.assertEqual(
                set(failure["artifacts"]),
                {
                    "run_start",
                    "steps",
                    "trace",
                    "partial_summary",
                    "summary",
                    "gate",
                },
            )
            self.assertEqual(failure["last_completed_step"], 1)

    def test_gate_cannot_mutate_finalized_inputs(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            writer = self._writer(root)
            writer.start({"status": "STARTED"})
            writer.write_step(1, {"time_s": 0.01})
            writer.finalize_before_gate(
                partial_summary={"steps": 1},
                summary={"steps": 1},
            )

            def mutating_gate(_records: dict) -> dict:
                writer.summary_path.write_text(
                    '{"steps": 1, "tampered": true}\n',
                    encoding="utf-8",
                )
                return {"status": "MUST_NOT_PUBLISH", "passed": True}

            with self.assertRaises(forensic.ForensicRolloutError):
                writer.run_gate(mutating_gate)
            self.assertFalse(writer.gate_path.exists())

    def test_supplied_trace_must_equal_the_step_journal(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            writer = self._writer(root)
            writer.start({"status": "STARTED"})
            writer.write_step(1, {"time_s": 0.01, "value": 10})
            with self.assertRaises(forensic.ForensicRolloutError):
                writer.finalize_before_gate(
                    trace=[{"step": 1, "time_s": 0.01, "value": 11}],
                    partial_summary={"steps": 1},
                    summary={"steps": 1},
                )
            self.assertFalse(writer.trace_path.exists())


class ArtifactRecordTests(unittest.TestCase):
    def test_artifact_record_rejects_path_outside_declared_root(self) -> None:
        with (
            tempfile.TemporaryDirectory() as root_temporary,
            tempfile.TemporaryDirectory() as other_temporary,
        ):
            other = Path(other_temporary) / "artifact.json"
            other.write_text(json.dumps({"value": 1}), encoding="utf-8")
            with self.assertRaises(forensic.ForensicRolloutError):
                forensic.artifact_record(
                    other,
                    artifact_root=Path(root_temporary),
                )

    def test_zero_step_failure_receipt_is_supported(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            writer = forensic.ForensicRolloutWriter(
                root / "run",
                artifact_root=root,
            )
            writer.start({"status": "STARTED"})
            writer.publish_failure(
                end_reason="reset_failed",
                error="environment reset failed",
            )
            failure = forensic.strict_json_load(writer.failure_path)
            self.assertEqual(failure["last_completed_step"], 0)
            self.assertEqual(failure["artifacts"]["steps"], [])


if __name__ == "__main__":
    unittest.main()
