from __future__ import annotations

import io
import json
import tempfile
import unittest
from contextlib import redirect_stdout
from pathlib import Path
from unittest import mock

from validation import h0_primary_grf_split_v3_qualification_contract as contract
from validation import run_h0_primary_grf_split_v3_qualification as runner


class QualificationRunnerTests(unittest.TestCase):
    def test_claim_requires_exact_preallocated_empty_destination(self) -> None:
        case_id = contract.CASE_IDS[0]
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            destination = root / contract.rollout_destination("baseline", case_id)
            destination.mkdir(parents=True)
            with mock.patch.object(runner, "REPO_ROOT", root):
                self.assertEqual(
                    runner._claim_destination("baseline", case_id, destination),
                    destination.resolve(),
                )
                (destination / "consumed.json").write_text("{}\n", encoding="utf-8")
                with self.assertRaisesRegex(
                    runner.QualificationExecutionError, "already consumed"
                ):
                    runner._claim_destination("baseline", case_id, destination)

    def test_candidate_stage_requires_all_baselines_and_scaffold_decision(self) -> None:
        with (
            mock.patch.object(runner, "_require_baseline_complete") as baselines,
            mock.patch.object(
                runner.scaffold, "validate_qualification_prerequisites"
            ) as scaffold_check,
        ):
            runner._enforce_predecessors("candidate")
        baselines.assert_called_once_with()
        scaffold_check.assert_called_once_with(repo_root=runner.REPO_ROOT)

    def test_baseline_stage_is_closed_after_decision_publication(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            decision = root / contract.DECISION_RECEIPT_PATH
            decision.parent.mkdir(parents=True)
            decision.write_text("{}\n", encoding="utf-8")
            with (
                mock.patch.object(runner, "REPO_ROOT", root),
                self.assertRaisesRegex(
                    runner.QualificationExecutionError, "cannot run after decision"
                ),
            ):
                runner._enforce_predecessors("baseline")

    def test_supervisor_completes_every_baseline_before_any_candidate(self) -> None:
        events: list[tuple[str, str] | tuple[str]] = []

        def worker(role: str, case_id: str) -> None:
            events.append((role, case_id))
            receipt = root / contract.rollout_destination(role, case_id) / "receipt.json"
            receipt.parent.mkdir(parents=True, exist_ok=True)
            receipt.write_text("{}\n", encoding="utf-8")

        def publish() -> tuple[dict[str, object], dict[str, object]]:
            events.append(("publish",))
            return {}, {}

        def write_exclusive(path: Path, value: object) -> Path:
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(
                json.dumps(value, sort_keys=True, allow_nan=False) + "\n",
                encoding="utf-8",
            )
            return path

        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            with (
                mock.patch.object(runner, "REPO_ROOT", root),
                mock.patch.object(runner, "verify_lock", return_value={}),
                mock.patch.object(runner, "_run_worker_process", side_effect=worker),
                mock.patch.object(
                    runner, "_publish_baseline_and_decision", side_effect=publish
                ),
                mock.patch.object(runner.v3, "_strict_mapping", return_value={}),
                mock.patch.object(
                    runner.gates,
                    "condition_matched_gate",
                    side_effect=lambda _baseline, _candidate, *, case_id: {
                        "status": "PASS_H0_PRIMARY_SPLIT_V3_QUALIFICATION_CASE",
                        "passed": True,
                        "case_id": case_id,
                    },
                ),
                mock.patch.object(
                    runner.v3, "_write_json_exclusive", side_effect=write_exclusive
                ),
                mock.patch.object(
                    runner,
                    "_record",
                    return_value={
                        "path": "qualification_lock.json",
                        "sha256": "a" * 64,
                        "size_bytes": 1,
                    },
                ),
            ):
                with redirect_stdout(io.StringIO()):
                    ledger = runner.execute()

        expected = [("baseline", case_id) for case_id in contract.CASE_IDS]
        expected.append(("publish",))
        expected.extend(("candidate", case_id) for case_id in contract.CASE_IDS)
        self.assertEqual(events, expected)
        self.assertTrue(ledger["passed"])
        self.assertEqual(ledger["baseline_rollouts_completed"], 6)
        self.assertEqual(ledger["candidate_rollouts_completed"], 6)
        self.assertEqual(ledger["next_stage"], "TRAINER_ZERO_UPDATE_PORT")
        self.assertEqual(ledger["protected_trials_opened"], [])

    def test_supervisor_stops_without_retry_after_first_worker_failure(self) -> None:
        calls: list[tuple[str, str]] = []

        def worker(role: str, case_id: str) -> None:
            calls.append((role, case_id))
            if len(calls) == 2:
                raise runner.QualificationExecutionError("synthetic failure")

        def write_exclusive(path: Path, value: object) -> Path:
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(
                json.dumps(value, sort_keys=True, allow_nan=False) + "\n",
                encoding="utf-8",
            )
            return path

        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            with (
                mock.patch.object(runner, "REPO_ROOT", root),
                mock.patch.object(runner, "verify_lock", return_value={}),
                mock.patch.object(runner, "_run_worker_process", side_effect=worker),
                mock.patch.object(
                    runner.v3, "_write_json_exclusive", side_effect=write_exclusive
                ),
                mock.patch.object(
                    runner,
                    "_record",
                    return_value={
                        "path": "qualification_lock.json",
                        "sha256": "a" * 64,
                        "size_bytes": 1,
                    },
                ),
                self.assertRaisesRegex(
                    runner.QualificationExecutionError, "synthetic failure"
                ),
            ):
                with redirect_stdout(io.StringIO()):
                    runner.execute()

        self.assertEqual(
            calls,
            [
                ("baseline", contract.CASE_IDS[0]),
                ("baseline", contract.CASE_IDS[1]),
            ],
        )


if __name__ == "__main__":
    unittest.main()
