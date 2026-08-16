from __future__ import annotations

import unittest

from validation import h0_primary_grf_split_v8r1p1_teacher_replay_contract as contract
from validation import run_h0_primary_grf_split_v8r1p1_teacher_replay as runner


class V8R1P1Tests(unittest.TestCase):
    def test_engine_contract_and_frozen_literal_are_exact(self) -> None:
        audit = runner.audit_compatibility()
        self.assertTrue(audit["passed"])
        self.assertEqual(audit["missing_attributes"], [])
        self.assertEqual(
            contract.V25_ACTIVE_EVENT_CONTRACT_ID,
            contract.V26_ACTIVE_EVENT_CONTRACT_ID,
        )
        self.assertEqual(
            runner.FROZEN_VERIFIER_NEXT_STAGE,
            "EXECUTE_SIX_V26_ACTIVE_DEVELOPMENT_REPLAYS_ONCE",
        )

    def test_fresh_paths_and_worker_entrypoint(self) -> None:
        self.assertIn("v8r1p1", contract.RUN_ROOT.as_posix())
        token = "v8r1p1-token-long-enough-123456789"
        command = runner._worker_command(contract.CASE_IDS[0], token)
        self.assertEqual(command[1], str(runner.Path(runner.__file__).resolve()))
        self.assertIs(runner.engine.contract, contract)
        self.assertIs(runner.engine.verify_lock, runner.verify_lock)

    def test_build_config_is_still_exact_v26(self) -> None:
        cfg = runner.build_env_config(contract.canonical_case(contract.CASE_IDS[0]))
        self.assertEqual(cfg["binary_phase_fsm_mode"], "binary_active")
        self.assertEqual(
            cfg["binary_phase_event_contract_id"],
            contract.V26_ACTIVE_EVENT_CONTRACT_ID,
        )
        self.assertEqual(cfg["online_grf_applied_sides"], ["left"])
        self.assertEqual(cfg["reward"]["morphology_weight"], 0.0)


if __name__ == "__main__":
    unittest.main()
