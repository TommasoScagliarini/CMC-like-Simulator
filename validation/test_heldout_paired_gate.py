from __future__ import annotations

import hashlib
import json
import pickle
import tempfile
import unittest
from pathlib import Path

import numpy as np

import heldout_paired_gate as heldout
from compare_policy_checkpoints import _actor_digest, _load_state


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _write_json(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2) + "\n", encoding="utf-8")


def _module(path: Path, marker: float) -> str:
    path.mkdir(parents=True, exist_ok=True)
    state = {
        "pi.0.0.weight": np.full((3, 2), marker, dtype=np.float32),
        "pi.0.0.bias": np.full((3,), marker, dtype=np.float32),
        "pi.0.2.weight": np.full((3, 3), marker + 0.1, dtype=np.float32),
        "pi.0.2.bias": np.full((3,), marker + 0.1, dtype=np.float32),
        "pi.1.weight": np.full((4, 3), marker + 0.2, dtype=np.float32),
        "pi.1.bias": np.full((4,), marker + 0.2, dtype=np.float32),
    }
    (path / "module_state.pkl").write_bytes(pickle.dumps(state))
    (path / "class_and_ctor_args.pkl").write_bytes(b"fixture")
    _write_json(path / "metadata.json", {"fixture": True})
    return _actor_digest(_load_state(path))


class HeldoutPairedGateTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        self.h0 = self.root / "h0" / "rl_module_last"
        self.h0_digest = _module(self.h0, 0.0)
        self.run = self.root / "training_run"
        self.candidate2 = self.run / "milestone_iteration_000002" / "rl_module_last"
        self.candidate3 = self.run / "milestone_iteration_000003" / "rl_module_last"
        self.digest2 = _module(self.candidate2, 0.01)
        self.digest3 = _module(self.candidate3, 0.02)
        self.summary = self.run / "summary.json"
        _write_json(self.summary, {"ok": True, "stop_reason": "completed"})
        self.protocol_path = self.root / "protocol.json"
        self.evidence_path = self.root / "evidence.json"
        self.protocol = {
            "source": {"actor_digest": self.h0_digest},
            "development_gate": {"reference_checkpoint": str(self.h0)},
            "candidate_selection": {
                "eligible_set": heldout.EXPECTED_ELIGIBLE_SET,
                "primary_order": heldout.EXPECTED_PRIMARY_ORDER,
                "tie_breakers": list(heldout.EXPECTED_TIE_BREAKERS),
                "maximum_candidates_opened_on_held_out": 1,
                "fallback_after_held_out_failure": False,
                "checkpoint_best_is_not_a_selector": True,
            },
            "held_out_gate": {
                "status": "sealed_until_one_candidate_digest_is_fixed",
                "seeds": [126, 127, 128],
                "start_offsets_s": list(heldout.EXPECTED_START_OFFSETS_S),
                "sigma": 0.005,
                "paired_reference": "H0 in every seed/start cell",
                "reserve_mode": "condition_matched_non_regression",
                "open_once": True,
            },
        }
        _write_json(self.protocol_path, self.protocol)
        self.other_inputs: dict[str, Path] = {}
        for name in (
            "selection_addendum",
            "pilot_screen",
            "policy_drift",
        ):
            path = self.root / f"{name}.json"
            _write_json(path, {"fixture": name})
            self.other_inputs[name] = path
        self.training_history = self.run / "train_iterations.jsonl"
        self.training_history.write_text('{"iteration": 2}\n', encoding="utf-8")
        self.other_inputs["training_history"] = self.training_history
        self.evidence = {
            "schema_version": 1,
            "table": "ppo_pilot_candidate_evidence",
            "ok": True,
            "status": "COMPLETE",
            "output": str(self.evidence_path),
            "contract": {
                "screening_complete": True,
                "held_out_data_read": False,
                "rollouts_launched": False,
                "ranking_performed": False,
                "checkpoint_selected": False,
            },
            "inputs": {
                "protocol": {
                    "path": str(self.protocol_path),
                    "sha256": _sha256(self.protocol_path),
                },
                "training_summary": {
                    "path": str(self.summary),
                    "sha256": _sha256(self.summary),
                },
                **{
                    name: {"path": str(path), "sha256": _sha256(path)}
                    for name, path in self.other_inputs.items()
                },
            },
            "preregistered_candidate_ordering": {
                "eligible_set": heldout.EXPECTED_ELIGIBLE_SET,
                "primary_order": heldout.EXPECTED_PRIMARY_ORDER,
                "tie_breakers": list(heldout.EXPECTED_TIE_BREAKERS),
                "ordering_performed": False,
                "selected_logical_iteration": None,
            },
            "eligible_logical_iterations": [2, 3],
            "row_count": 2,
            "rows": [
                self._row(2, self.digest2, reserve=0.98, penetration=0.9, kl=0.001),
                self._row(3, self.digest3, reserve=0.95, penetration=0.95, kl=0.002),
            ],
        }
        _write_json(self.evidence_path, self.evidence)
        self.restart_audit_path = self.root / "env_runner_restart_audit.json"
        _write_json(
            self.restart_audit_path,
            {
                "schema_version": 1,
                "ok": True,
                "status": "PASS",
                "run_dir": str(self.run),
                "summary_path": str(self.summary),
                "history_path": str(self.training_history),
                "contract": {
                    "post_run_only": True,
                    "require_completed_summary": True,
                    "require_scoped_driver_log": True,
                    "require_no_supervisor_restart_or_skip": True,
                    "require_no_ray_restart_evidence": True,
                    "fault_tolerance_metrics_optional_but_fail_if_nonzero": True,
                },
                "checks": {"training_completed_ok": True, "no_restarts": True},
                "failed_checks": [],
            },
        )

    def tearDown(self) -> None:
        self.temporary.cleanup()

    @staticmethod
    def _row(
        iteration: int,
        digest: str,
        *,
        reserve: float,
        penetration: float,
        kl: float,
    ) -> dict[str, object]:
        return {
            "logical_iteration": iteration,
            "pilot_update_index": iteration - 1,
            "screen_eligibility": "ELIGIBLE",
            "policy_drift_from_h0": {
                "audit_status": "PASS",
                "logstd_bit_exact": True,
                "candidate_actor_digest": digest,
            },
            "preregistered_ordering_keys": {
                "eligible_for_ordering": True,
                "primary_worst_condition_matched_reserve_ratio_vs_h0": reserve,
                "tie_breaker_worst_penetration_ratio_vs_h0": penetration,
                "tie_breaker_cumulative_empirical_kl_from_h0_mean": kl,
                "tie_breaker_earlier_pilot_update": iteration - 1,
            },
        }

    def _rewrite_protocol_and_evidence(self) -> None:
        _write_json(self.protocol_path, self.protocol)
        self.evidence["inputs"]["protocol"]["sha256"] = _sha256(self.protocol_path)
        _write_json(self.evidence_path, self.evidence)

    def test_seal_binds_unique_preregistered_winner_without_opening(self) -> None:
        path, seal = heldout.build_candidate_seal(
            protocol_path=self.protocol_path,
            evidence_path=self.evidence_path,
            restart_audit_path=self.restart_audit_path,
            candidate_logical_iteration=3,
        )
        self.assertEqual(path, (self.root / heldout.SEAL_FILENAME).resolve())
        self.assertEqual(seal["status"], "SEALED")
        self.assertFalse(seal["held_out_data_read"])
        self.assertFalse(seal["held_out_opened"])
        self.assertEqual(seal["candidate_count"], 1)
        self.assertFalse(seal["fallback_allowed"])
        self.assertEqual(seal["candidate"]["actor_digest"], self.digest3)
        self.assertEqual(seal["selection"]["rank"], 1)
        self.assertFalse((self.root / heldout.OPEN_RECEIPT_FILENAME).exists())
        self.assertFalse((self.root / heldout.REPORT_FILENAME).exists())
        self.assertFalse((self.root / heldout.ROLLOUT_ROOT_NAME).exists())

    def test_non_winner_candidate_is_refused(self) -> None:
        with self.assertRaisesRegex(heldout.HeldoutRefusal, "not rank 1"):
            heldout.build_candidate_seal(
                protocol_path=self.protocol_path,
                evidence_path=self.evidence_path,
                restart_audit_path=self.restart_audit_path,
                candidate_logical_iteration=2,
            )

    def test_unsealed_protocol_is_refused(self) -> None:
        self.protocol["held_out_gate"]["status"] = "OPEN"
        self._rewrite_protocol_and_evidence()
        with self.assertRaisesRegex(heldout.HeldoutRefusal, "not SEALED"):
            heldout.build_candidate_seal(
                protocol_path=self.protocol_path,
                evidence_path=self.evidence_path,
                restart_audit_path=self.restart_audit_path,
                candidate_logical_iteration=3,
            )

    def test_fallback_or_multiple_candidates_are_refused(self) -> None:
        self.protocol["candidate_selection"]["fallback_after_held_out_failure"] = True
        self.protocol["candidate_selection"]["maximum_candidates_opened_on_held_out"] = 2
        self._rewrite_protocol_and_evidence()
        with self.assertRaises(heldout.HeldoutRefusal):
            heldout.build_candidate_seal(
                protocol_path=self.protocol_path,
                evidence_path=self.evidence_path,
                restart_audit_path=self.restart_audit_path,
                candidate_logical_iteration=3,
            )

    def test_actor_digest_mismatch_is_refused(self) -> None:
        self.evidence["rows"][1]["policy_drift_from_h0"]["candidate_actor_digest"] = "f" * 64
        _write_json(self.evidence_path, self.evidence)
        with self.assertRaisesRegex(heldout.HeldoutRefusal, "actor digest differs"):
            heldout.build_candidate_seal(
                protocol_path=self.protocol_path,
                evidence_path=self.evidence_path,
                restart_audit_path=self.restart_audit_path,
                candidate_logical_iteration=3,
            )

    def test_failed_restart_audit_is_refused(self) -> None:
        audit = json.loads(self.restart_audit_path.read_text(encoding="utf-8"))
        audit["ok"] = False
        audit["status"] = "FAIL"
        audit["failed_checks"] = ["no_restarts"]
        _write_json(self.restart_audit_path, audit)
        with self.assertRaisesRegex(heldout.HeldoutRefusal, "restart audit did not pass"):
            heldout.build_candidate_seal(
                protocol_path=self.protocol_path,
                evidence_path=self.evidence_path,
                restart_audit_path=self.restart_audit_path,
                candidate_logical_iteration=3,
            )

    def test_second_seal_is_refused(self) -> None:
        heldout.build_candidate_seal(
            protocol_path=self.protocol_path,
            evidence_path=self.evidence_path,
            restart_audit_path=self.restart_audit_path,
            candidate_logical_iteration=3,
        )
        with self.assertRaisesRegex(heldout.HeldoutRefusal, "already exists"):
            heldout.build_candidate_seal(
                protocol_path=self.protocol_path,
                evidence_path=self.evidence_path,
                restart_audit_path=self.restart_audit_path,
                candidate_logical_iteration=3,
            )

    def test_readiness_matrix_is_exactly_nine_pairs(self) -> None:
        seal_path, _ = heldout.build_candidate_seal(
            protocol_path=self.protocol_path,
            evidence_path=self.evidence_path,
            restart_audit_path=self.restart_audit_path,
            candidate_logical_iteration=3,
        )
        readiness = heldout.validate_open_readiness(seal_path)
        runs = heldout.planned_matrix(readiness)
        self.assertEqual(len(runs), 18)
        self.assertEqual({run.seed for run in runs}, {126, 127, 128})
        self.assertEqual(
            {run.offset_s for run in runs}, set(heldout.EXPECTED_START_OFFSETS_S)
        )
        for seed in heldout.EXPECTED_SEEDS:
            for start_index in range(3):
                paired = [
                    run
                    for run in runs
                    if run.seed == seed and run.start_index == start_index
                ]
                self.assertEqual([run.role for run in paired], ["h0", "candidate"])
                self.assertEqual(paired[0].offset_s, paired[1].offset_s)
        command = heldout._rollout_command(
            runs[0],
            output_dir=self.root / "unused",
            python_executable="python",
            rollout_script=self.root / "rollout_eval.py",
            timeout_s=900.0,
        )
        self.assertEqual(command[command.index("--action-selection") + 1], "stochastic")
        self.assertEqual(command[command.index("--seed") + 1], "126")
        self.assertEqual(command[command.index("--episode-start-offset-s") + 1], repr(heldout.EXPECTED_START_OFFSETS_S[0]))
        self.assertIn("--no-record-policy-trace", command)

    def test_bound_artifact_mutation_is_refused_before_opening(self) -> None:
        seal_path, _ = heldout.build_candidate_seal(
            protocol_path=self.protocol_path,
            evidence_path=self.evidence_path,
            restart_audit_path=self.restart_audit_path,
            candidate_logical_iteration=3,
        )
        (self.candidate3 / "metadata.json").write_text("changed", encoding="utf-8")
        with self.assertRaisesRegex(heldout.HeldoutRefusal, "changed after"):
            heldout.validate_open_readiness(seal_path)

    def test_existing_open_receipt_prevents_second_open(self) -> None:
        seal_path, _ = heldout.build_candidate_seal(
            protocol_path=self.protocol_path,
            evidence_path=self.evidence_path,
            restart_audit_path=self.restart_audit_path,
            candidate_logical_iteration=3,
        )
        _write_json(self.root / heldout.OPEN_RECEIPT_FILENAME, {"status": "OPENED"})
        with self.assertRaisesRegex(heldout.HeldoutRefusal, "already opened"):
            heldout.validate_open_readiness(seal_path)

    def test_open_parser_has_no_candidate_or_output_override(self) -> None:
        parser = heldout.build_parser()
        args = parser.parse_args(["open", "--seal", str(self.root / "seal.json")])
        self.assertFalse(hasattr(args, "checkpoint"))
        self.assertFalse(hasattr(args, "output_dir"))
        self.assertFalse(hasattr(args, "candidate"))


if __name__ == "__main__":
    unittest.main()
