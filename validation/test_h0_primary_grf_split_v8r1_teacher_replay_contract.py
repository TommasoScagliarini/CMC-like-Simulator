"""Pure tests for the V8R1 procedural correction overlay."""

from __future__ import annotations

import copy
import unittest

from validation import h0_primary_grf_split_v8r1_teacher_replay_contract as contract
from validation import h0_primary_grf_split_v8_teacher_replay_contract as v8
from validation import run_h0_primary_grf_split_v8r1_teacher_replay as runner
from validation.test_h0_primary_grf_split_v8_teacher_replay_contract import (
    valid_summary as v8_valid_summary,
)


class V8R1ContractTests(unittest.TestCase):
    def test_only_compatibility_name_changes_semantic_binding(self) -> None:
        self.assertEqual(
            contract.V25_ACTIVE_EVENT_CONTRACT_ID,
            contract.V26_ACTIVE_EVENT_CONTRACT_ID,
        )
        self.assertEqual(
            contract.V26_ACTIVE_EVENT_CONTRACT_ID,
            v8.V26_ACTIVE_EVENT_CONTRACT_ID,
        )
        self.assertEqual(
            contract.TARGET_OBSERVATION_CONTRACT_ID,
            v8.TARGET_OBSERVATION_CONTRACT_ID,
        )
        self.assertNotEqual(contract.RUN_ROOT, v8.RUN_ROOT)
        self.assertNotEqual(contract.LOCK_PATH, v8.LOCK_PATH)

    def test_all_inherited_engine_attributes_are_present(self) -> None:
        audit = runner.audit_inherited_engine_contract()
        self.assertTrue(audit["passed"])
        self.assertEqual(audit["missing_attributes"], [])
        self.assertIn(
            "V25_ACTIVE_EVENT_CONTRACT_ID", audit["required_attributes"]
        )

    def test_scientific_gate_is_identical_after_identity_projection(self) -> None:
        summary = v8_valid_summary()
        summary["schema_version"] = contract.SCHEMA_VERSION
        summary["status"] = contract.ROLLOUT_COLLECTED_STATUS
        summary["protocol_id"] = contract.PROTOCOL_ID
        summary["collector_id"] = contract.COLLECTOR_ID
        summary["case_id"] = contract.CASE_IDS[0]
        observed = contract.replay_gate(summary)
        expected = v8.replay_gate(v8_valid_summary())
        self.assertTrue(observed["passed"])
        self.assertEqual(observed["checks"], expected["checks"])
        drifted = copy.deepcopy(summary)
        drifted["binary_phase_event_contract_id"] = (
            "binary_point_v25+functional_contact_fsm_v1"
        )
        self.assertFalse(contract.replay_gate(drifted)["passed"])


class V8R1RunnerTests(unittest.TestCase):
    def test_terminal_v8_is_preserved_at_zero_steps(self) -> None:
        history = runner.validate_terminal_v8_history()
        self.assertEqual(history["completed_policy_steps"], 0)
        self.assertEqual(
            history["failure_class"], "PROCEDURAL_COMPATIBILITY_BINDING"
        )
        self.assertEqual(history["history_policy"], "TERMINAL_NO_REINTERPRETATION")

    def test_worker_subprocess_reenters_fresh_overlay(self) -> None:
        token = "v8r1-worker-token-long-enough-123456"
        command = runner._worker_command(contract.CASE_IDS[0], token)
        self.assertEqual(command[1], str(runner.Path(runner.__file__).resolve()))
        self.assertEqual(command[-2:], ["--execution-token", token])
        self.assertIs(runner.engine.contract, contract)
        self.assertIs(runner.engine.verify_lock, runner.verify_lock)

    def test_build_env_config_routes_exact_v26(self) -> None:
        config = runner.build_env_config(
            contract.canonical_case(contract.CASE_IDS[0])
        )
        self.assertEqual(config["binary_phase_fsm_mode"], "binary_active")
        self.assertEqual(
            config["binary_phase_event_contract_id"],
            contract.V26_ACTIVE_EVENT_CONTRACT_ID,
        )
        self.assertEqual(config["online_grf_applied_sides"], ["left"])
        self.assertEqual(config["reward"]["morphology_weight"], 0.0)


if __name__ == "__main__":
    unittest.main()
