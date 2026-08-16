from __future__ import annotations

import hashlib
import json
import tempfile
import unittest
from pathlib import Path
from unittest import mock

from validation import h0_primary_grf_split_v3_qualification_scaffold as scaffold


def _write_json(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(value, indent=2, sort_keys=False, allow_nan=False) + "\n",
        encoding="utf-8",
    )


def _record(path: Path, root: Path) -> dict[str, object]:
    return {
        "path": path.relative_to(root).as_posix(),
        "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        "size_bytes": path.stat().st_size,
    }


class CanonicalCaseTests(unittest.TestCase):
    def test_exact_six_cases_are_zero_update_legacy_events(self) -> None:
        cases = scaffold.canonical_cases()
        self.assertEqual(tuple(case["case_id"] for case in cases), scaffold.CASE_IDS)
        self.assertEqual(len(cases), 6)
        self.assertEqual(
            [case["offset_delta_s"] for case in cases[:3]], [-0.2, 0.0, 0.2]
        )
        self.assertEqual([case["seed"] for case in cases[3:]], [126, 127, 128])
        for case in cases:
            self.assertEqual(case["event_contract_id"], scaffold.EVENT_CONTRACT_ID)
            self.assertEqual(case["phase_fsm_input_mode"], "legacy_events")
            self.assertEqual(case["morphology_weight"], 0.0)
            self.assertEqual(case["actor_updates"], 0)
            self.assertEqual(case["critic_updates"], 0)
            self.assertEqual(case["ppo_updates"], 0)

    def test_later_stages_are_explicitly_separate_and_pending(self) -> None:
        manifest = scaffold.scaffold_manifest()
        stages = {item["stage"]: item for item in manifest["stages"]}
        self.assertEqual(
            stages["trainer_zero_update_port"]["state"],
            "pending_after_autonomous_qualification",
        )
        self.assertFalse(
            stages["trainer_zero_update_port"]["included_in_this_scaffold"]
        )
        self.assertEqual(
            stages["v25_abc_preflight"]["state"],
            "pending_after_zero_update_port",
        )
        self.assertFalse(stages["v25_abc_preflight"]["included_in_this_scaffold"])
        self.assertFalse(manifest["authority"]["execution_authorized"])


class FrozenNoiseTests(unittest.TestCase):
    def test_repository_seed_126_to_128_tapes_match_frozen_manifest(self) -> None:
        result = scaffold.validate_noise_tapes()
        self.assertEqual(result["seeds"], [126, 127, 128])
        self.assertTrue(result["deterministic_zero_tape"])


class PrerequisiteTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)

    def tearDown(self) -> None:
        self.temporary.cleanup()

    def _create_holdout(self) -> Path:
        freeze_path = self.root / scaffold.CANDIDATE_FREEZE_RELATIVE
        _write_json(
            freeze_path,
            {
                "schema_version": 3,
                "status": "H0_PRIMARY_SPLIT_V3_CANDIDATE_FROZEN_BEFORE_HOLDOUT",
                "protocol_id": scaffold.PROTOCOL_ID,
                "holdout_accessed_before_freeze": False,
                "actor_updates": 1,
                "critic_updates": 0,
                "ppo_updates": 0,
                "protected_trials_opened": [],
            },
        )
        gate_path = self.root / scaffold.HOLDOUT_GATE_RELATIVE
        _write_json(
            gate_path,
            {
                "schema_version": 3,
                "status": "PASS_H0_PRIMARY_SPLIT_V3_FINAL_HOLDOUT",
                "passed": True,
                "checks": {
                    "candidate_frozen_before_holdout": True,
                    "candidate_module_unchanged": True,
                    "no_updates_during_holdout": True,
                },
                "metrics": {},
                "candidate_freeze": _record(freeze_path, self.root),
                "holdout_access_claim": {},
                "holdout_replay_receipt": {},
                "actor_updates": 1,
                "critic_updates": 0,
                "ppo_updates": 0,
                "protected_trials_opened": [],
            },
        )
        receipt_path = self.root / scaffold.HOLDOUT_RECEIPT_RELATIVE
        _write_json(
            receipt_path,
            {
                "schema_version": 3,
                "status": "PASS_H0_PRIMARY_SPLIT_V3_FINAL_HOLDOUT",
                "passed": True,
                "gate": _record(gate_path, self.root),
                "candidate_freeze": _record(freeze_path, self.root),
                "holdout_access_claim": {},
                "holdout_replay_receipt": {},
                "actor_updates": 1,
                "critic_updates": 0,
                "ppo_updates": 0,
                "protected_trials_opened": [],
            },
        )
        return receipt_path

    def _create_decision(self, holdout_receipt: Path) -> Path:
        baseline_path = (
            self.root / scaffold.RUN_ROOT_RELATIVE / "qualification" / "baseline.json"
        )
        case_metrics = {
            case_id: {
                "sea": {"sea_tau_rms_nm": 1.0},
                "reserve": {"reserve_rms_nm": 2.0},
            }
            for case_id in scaffold.CASE_IDS
        }
        _write_json(
            baseline_path,
            {
                "schema_version": 1,
                "status": "H0_PRIMARY_SPLIT_V3_QUALIFICATION_BASELINE_FROZEN",
                "passed": True,
                "baseline_id": "explicit_condition_matched_baseline_v1",
                "case_metrics": case_metrics,
                "actor_updates": 0,
                "critic_updates": 0,
                "ppo_updates": 0,
                "protected_trials_opened": [],
            },
        )
        decision_path = self.root / scaffold.DECISION_RECEIPT_RELATIVE
        _write_json(
            decision_path,
            {
                "schema_version": 1,
                "status": (
                    "H0_PRIMARY_SPLIT_V3_QUALIFICATION_BASELINE_TOLERANCE_DECIDED"
                ),
                "passed": True,
                "protocol_id": scaffold.PROTOCOL_ID,
                "decision_authority": "EXPLICIT_USER_DECISION",
                "candidate_holdout_receipt": _record(holdout_receipt, self.root),
                "baseline": {
                    "baseline_id": "explicit_condition_matched_baseline_v1",
                    "comparison_scope": "condition_matched_six_cases",
                    "receipt": _record(baseline_path, self.root),
                },
                "tolerances": {
                    "comparison_formula": (
                        "candidate <= baseline + max(absolute_tolerance, "
                        "relative_tolerance * abs(baseline))"
                    ),
                    "sea": [
                        {
                            "metric": "sea_tau_rms_nm",
                            "absolute_tolerance": 0.1,
                            "relative_tolerance": 0.05,
                        }
                    ],
                    "reserve": [
                        {
                            "metric": "reserve_rms_nm",
                            "absolute_tolerance": 0.2,
                            "relative_tolerance": 0.05,
                        }
                    ],
                },
                "fixed_gates": {
                    "expected_steps": 500,
                    "minimum_valid_cycles": 2,
                    "penetration_limit_m": 0.025,
                    "penetration_comparison": "strict_less_than",
                    "zero_count_fields": list(scaffold.ZERO_COUNT_FIELDS),
                },
                "runtime_contract": {
                    "event_contract_id": scaffold.EVENT_CONTRACT_ID,
                    "phase_fsm_input_mode": "legacy_events",
                    "morphology_weight": 0.0,
                },
                "canonical_case_ids": list(scaffold.CASE_IDS),
                "authority": dict(scaffold._DECISION_AUTHORITY),
                "actor_updates": 0,
                "critic_updates": 0,
                "ppo_updates": 0,
                "protected_trials_opened": [],
            },
        )
        return decision_path

    def test_holdout_fails_closed_when_absent(self) -> None:
        with self.assertRaisesRegex(scaffold.QualificationScaffoldError, "missing"):
            scaffold.validate_v3_holdout_pass(self.root)

    def test_explicit_decision_is_required_after_holdout(self) -> None:
        holdout = self._create_holdout()
        scaffold.validate_v3_holdout_pass(self.root)
        with self.assertRaisesRegex(
            scaffold.QualificationScaffoldError,
            "explicit baseline/tolerance decision receipt is missing",
        ):
            scaffold.validate_baseline_tolerance_decision(repo_root=self.root)
        self.assertTrue(holdout.is_file())

    def test_complete_prerequisites_do_not_claim_qualification_execution(self) -> None:
        holdout = self._create_holdout()
        decision = self._create_decision(holdout)
        with mock.patch.object(
            scaffold,
            "validate_noise_tapes",
            return_value={"status": "PASS", "seeds": [126, 127, 128]},
        ):
            result = scaffold.validate_qualification_prerequisites(
                repo_root=self.root,
                decision_path=decision,
            )
        self.assertEqual(result["status"], "AUTONOMOUS_QUALIFICATION_INPUTS_READY")
        self.assertFalse(result["qualification_executed"])
        self.assertEqual(result["actor_updates"], 0)
        self.assertEqual(
            result["later_stages"]["trainer_zero_update_port"],
            "PENDING_AFTER_QUALIFICATION_PASS",
        )

    def test_decision_cannot_authorize_ppo_or_change_event_contract(self) -> None:
        holdout = self._create_holdout()
        decision_path = self._create_decision(holdout)
        decision = json.loads(decision_path.read_text(encoding="utf-8"))
        decision["authority"]["ppo_updates_authorized"] = True
        decision["runtime_contract"]["phase_fsm_input_mode"] = "binary_active"
        _write_json(decision_path, decision)
        with self.assertRaises(scaffold.QualificationScaffoldError):
            scaffold.validate_baseline_tolerance_decision(
                decision_path,
                repo_root=self.root,
            )


if __name__ == "__main__":
    unittest.main()
