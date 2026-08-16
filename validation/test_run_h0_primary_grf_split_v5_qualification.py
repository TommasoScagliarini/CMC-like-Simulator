from __future__ import annotations

import io
import json
import tempfile
import unittest
from contextlib import redirect_stdout
from pathlib import Path
from unittest import mock

from validation import h0_primary_grf_split_v5_qualification_contract as contract
from validation import run_h0_primary_grf_split_v5_qualification as runner


def _artifact() -> dict[str, object]:
    return {"path": "fixture", "sha256": "a" * 64, "size_bytes": 1}


class QualificationRunnerTests(unittest.TestCase):
    def _post_holdout_fixture(self, root: Path) -> tuple[dict[str, object], ...]:
        receipt = {
            "schema_version": 5,
            "status": "PASS_H0_PRIMARY_SPLIT_V5_FINAL_HOLDOUT",
            "passed": True,
            "gate": _artifact(),
            "candidate_freeze": _artifact(),
            "holdout_access_claim": _artifact(),
            "holdout_replay_receipt": _artifact(),
            "v4_preexecution_failure": _artifact(),
            "actor_updates": 1,
            "additional_actor_updates": 0,
            "critic_updates": 0,
            "ppo_updates": 0,
            "protected_trials_opened": [],
        }
        gate = {
            "schema_version": 5,
            "status": receipt["status"],
            "passed": True,
            "checks": {
                "candidate_frozen_before_holdout": True,
                "candidate_module_unchanged": True,
                "no_updates_during_holdout": True,
            },
            "metrics": {},
            "offline_thresholds": {},
            "candidate_freeze": receipt["candidate_freeze"],
            "holdout_access_claim": receipt["holdout_access_claim"],
            "holdout_replay_receipt": receipt["holdout_replay_receipt"],
            "v4_preexecution_failure": receipt["v4_preexecution_failure"],
            "actor_updates": 1,
            "additional_actor_updates": 0,
            "critic_updates": 0,
            "ppo_updates": 0,
            "protected_trials_opened": [],
        }
        ledger = {
            "schema_version": 5,
            "status": receipt["status"],
            "passed": True,
            "terminal_stage": "final_holdout_complete",
            "error": None,
            "started_unix_s": 1.0,
            "completed_unix_s": 2.0,
            "execution_lock": _artifact(),
            "attempt_claim": _artifact(),
            "v3_terminal_ledger": _artifact(),
            "v4_preexecution_failure": _artifact(),
            "v3_corpus_reused": True,
            "v3_failed_candidate_reused": False,
            "v4_candidate_reused": False,
            "candidate_created": True,
            "candidate_frozen_before_holdout": True,
            "holdout_access_claimed": True,
            "holdout_replay_completed": True,
            "final_holdout_completed": True,
            "actor_update_candidates": 1,
            "critic_updates": 0,
            "ppo_updates": 0,
            "protected_trials_opened": [],
            "retry_or_retuning_allowed": False,
            "next_stage": "CANONICAL_CLOSED_LOOP_QUALIFICATION",
        }
        freeze = {
            "schema_version": 5,
            "status": "H0_PRIMARY_SPLIT_V5_CANDIDATE_FROZEN_BEFORE_HOLDOUT",
            "protocol_id": contract.SOURCE_PROTOCOL_ID,
            "candidate_id": contract.CANDIDATE_ID,
            "candidate_receipt": _artifact(),
            "candidate_module_state": _artifact(),
            "candidate_module_ctor": _artifact(),
            "candidate_module_metadata": _artifact(),
            "actor_feature_manifest": _artifact(),
            "candidate_actor_digest": "b" * 64,
            "v4_preexecution_failure": _artifact(),
            "holdout_accessed_before_freeze": False,
            "actor_updates": 1,
            "critic_updates": 0,
            "ppo_updates": 0,
            "protected_trials_opened": [],
        }
        candidate_receipt = {"actor_feature_manifest": freeze["actor_feature_manifest"]}
        manifest = {
            "candidate_id": contract.CANDIDATE_ID,
            "observation_contract_id": "primary_grf_split_v1",
            "event_contract_id": "legacy_events_v1",
            "actor_feature_count": 35,
            "trainable_scope": "full_mean_network",
            "logstd_policy": "frozen_bit_exact",
            "actor_digest": freeze["candidate_actor_digest"],
        }
        candidate = root / contract.CANDIDATE_MODULE_PATH
        candidate.mkdir(parents=True)
        (candidate / "actor_feature_manifest.json").write_text("{}\n", encoding="utf-8")
        return receipt, gate, ledger, freeze, candidate_receipt, manifest

    def test_post_holdout_validator_requires_exact_v5_lineage(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            receipt, gate, ledger, freeze, candidate_receipt, manifest = (
                self._post_holdout_fixture(root)
            )

            def strict(path: Path) -> dict[str, object]:
                resolved = Path(path)
                if resolved == root / contract.HOLDOUT_RECEIPT_PATH:
                    return receipt
                if resolved == root / contract.HOLDOUT_GATE_PATH:
                    return gate
                if resolved == root / contract.V5_EXECUTION_LEDGER_PATH:
                    return ledger
                if resolved.name == "actor_feature_manifest.json":
                    return manifest
                raise AssertionError(f"unexpected strict read: {resolved}")

            with (
                mock.patch.object(runner, "REPO_ROOT", root),
                mock.patch.object(runner.v5, "verify_lock", return_value={}),
                mock.patch.object(
                    runner.v5,
                    "_validate_candidate_receipt",
                    return_value=(
                        candidate_receipt,
                        root / contract.CANDIDATE_MODULE_PATH,
                    ),
                ),
                mock.patch.object(
                    runner.v5,
                    "verify_candidate_freeze",
                    return_value=freeze,
                ),
                mock.patch.object(runner.v5, "_strict_mapping", side_effect=strict),
                mock.patch.object(runner, "_record_matches", return_value=True),
                mock.patch.object(runner, "_record", return_value=_artifact()),
            ):
                result = runner.validate_v5_post_holdout()

        self.assertEqual(
            result["status"],
            "PASS_H0_PRIMARY_SPLIT_V5_POST_HOLDOUT_PREREQUISITE",
        )
        self.assertEqual(result["candidate_actor_digest"], "b" * 64)

    def test_post_holdout_validator_rejects_additional_actor_update(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            receipt, gate, ledger, freeze, candidate_receipt, manifest = (
                self._post_holdout_fixture(root)
            )
            receipt["additional_actor_updates"] = 1

            def strict(path: Path) -> dict[str, object]:
                resolved = Path(path)
                if resolved == root / contract.HOLDOUT_RECEIPT_PATH:
                    return receipt
                if resolved == root / contract.HOLDOUT_GATE_PATH:
                    return gate
                if resolved == root / contract.V5_EXECUTION_LEDGER_PATH:
                    return ledger
                return manifest

            with (
                mock.patch.object(runner, "REPO_ROOT", root),
                mock.patch.object(runner.v5, "verify_lock", return_value={}),
                mock.patch.object(
                    runner.v5,
                    "_validate_candidate_receipt",
                    return_value=(candidate_receipt, root / contract.CANDIDATE_MODULE_PATH),
                ),
                mock.patch.object(
                    runner.v5,
                    "verify_candidate_freeze",
                    return_value=freeze,
                ),
                mock.patch.object(runner.v5, "_strict_mapping", side_effect=strict),
                mock.patch.object(runner, "_record_matches", return_value=True),
                self.assertRaisesRegex(
                    runner.QualificationExecutionError,
                    "holdout receipt",
                ),
            ):
                runner.validate_v5_post_holdout()

    def test_v3_rollout_engine_binding_is_scoped_and_uses_v5_contract(self) -> None:
        original_contract = runner.engine.contract
        original_gates = runner.engine.gates
        with runner._bind_v3_qualification_engine():
            self.assertEqual(runner.engine.contract.PROTOCOL_ID, contract.PROTOCOL_ID)
            self.assertEqual(runner.engine.contract.LOCK_PATH, contract.LOCK_PATH)
            self.assertIs(runner.engine.gates, runner.gates)
            self.assertEqual(runner.engine.LOCK, runner.LOCK)
            self.assertEqual(runner.engine.CANDIDATE_MODULE, runner.CANDIDATE_MODULE)
        self.assertIs(runner.engine.contract, original_contract)
        self.assertIs(runner.engine.gates, original_gates)

    def test_candidate_stage_requires_all_baselines_and_decision(self) -> None:
        with (
            mock.patch.object(runner, "_require_baseline_complete") as baselines,
            mock.patch.object(runner, "_validate_baseline_decision") as decision,
        ):
            runner._enforce_predecessors("candidate")
        baselines.assert_called_once_with()
        decision.assert_called_once_with()

    def test_supervisor_runs_all_baselines_before_any_candidate(self) -> None:
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
                    runner,
                    "_publish_baseline_and_decision",
                    side_effect=publish,
                ),
                mock.patch.object(runner.v5, "_strict_mapping", return_value={}),
                mock.patch.object(
                    runner.gates,
                    "condition_matched_gate",
                    side_effect=lambda _baseline, _candidate, *, case_id: {
                        "schema_version": 5,
                        "status": contract.CASE_PASS_STATUS,
                        "passed": True,
                        "case_id": case_id,
                    },
                ),
                mock.patch.object(
                    runner.v5,
                    "_write_json_exclusive",
                    side_effect=write_exclusive,
                ),
                mock.patch.object(runner, "_record", return_value=_artifact()),
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

    def test_supervisor_stops_without_retry_after_first_failure(self) -> None:
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
                    runner.v5,
                    "_write_json_exclusive",
                    side_effect=write_exclusive,
                ),
                mock.patch.object(runner, "_record", return_value=_artifact()),
                self.assertRaisesRegex(
                    runner.QualificationExecutionError,
                    "synthetic failure",
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

