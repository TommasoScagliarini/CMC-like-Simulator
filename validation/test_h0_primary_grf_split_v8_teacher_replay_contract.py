"""Focused pure tests for the fresh V8/V26 teacher-replay binding."""

from __future__ import annotations

import copy
import unittest
from unittest import mock

from validation import h0_primary_grf_split_v8_teacher_replay_contract as contract
from validation import run_h0_primary_grf_split_v8_teacher_replay as runner


def valid_summary() -> dict:
    case = contract.canonical_case(contract.CASE_IDS[0])
    events = []
    for index, name in enumerate(
        ("heel_strike", "toe_off", "heel_strike", "toe_off")
    ):
        event_time = 0.5 + 0.5 * index
        events.append(
            {
                "event": name,
                "source": contract.V26_FSM_SOURCE,
                "event_contract_id": contract.V26_ACTIVE_EVENT_CONTRACT_ID,
                "event_time_s": event_time,
                "confirmed_time_s": event_time + 0.005,
                "delivered_time_s": event_time + 0.010,
                "startup_partial_stance": False,
            }
        )
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.ROLLOUT_COLLECTED_STATUS,
        "protocol_id": contract.PROTOCOL_ID,
        "collector_id": contract.COLLECTOR_ID,
        "case_id": case["case_id"],
        "action_selection": case["action_selection"],
        "episode_start_offset_s": case["episode_start_offset_s"],
        "action_seed": case["action_seed"],
        "runtime_seed": case["runtime_seed"],
        "source_h0_id": contract.SOURCE_H0_ID,
        "source_observation_contract_id": contract.SOURCE_OBSERVATION_CONTRACT_ID,
        "target_observation_contract_id": contract.TARGET_OBSERVATION_CONTRACT_ID,
        "behavior": "FROZEN_V5_RAW_ACTION_REPLAY",
        "binary_phase_fsm_mode": "binary_active",
        "binary_phase_event_contract_id": contract.V26_ACTIVE_EVENT_CONTRACT_ID,
        "morphology_weight": 0.0,
        "steps": 500,
        "control_window_count": 5000,
        "v25_raw_sensor_sample_count": 5000,
        "end_reason": "episode_time_limit",
        "terminated": False,
        "truncated": True,
        "phase_valid_cycle_count": 2,
        "grf_penetration_max_m": 0.024,
        "invariant_mismatch_count": 0,
        "teacher_mean_mismatch_count": 0,
        "time_mismatch_count": 0,
        "step_contract_failure_count": 0,
        "action_clipped_values": 0,
        "timeout_count": 0,
        "safety_stop_count": 0,
        "invalid_event_count": 0,
        "hard_invalid_count": 0,
        "nonfinite_count": 0,
        "so_solver_unaccepted_count": 0,
        "sea_plugin_fallback_count": 0,
        "binary_phase_event_gate": {
            "passed": True,
            "sample_count": 5000,
            "event_count": 4,
            "events": events,
            "duplicate_event_count": 0,
            "out_of_order_event_count": 0,
            "left_non_v26_source_count": 0,
            "fallback_count": 0,
            "hard_invalid_count": 0,
        },
        "n_actor": 35,
        "n_observation": 84,
        "observation_dtype": "float32",
        "actor_feature_names": list(contract.EXPECTED_ACTOR_FEATURE_NAMES),
        "observation_feature_names": list(
            contract.EXPECTED_OBSERVATION_FEATURE_NAMES
        ),
        "invariant_columns": list(contract.INVARIANT_COLUMNS),
        "candidate_created": False,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
    }


class V8ContractTests(unittest.TestCase):
    def test_identity_is_fresh_and_development_only(self) -> None:
        self.assertEqual(contract.SCHEMA_VERSION, 8)
        self.assertIn("V8_V26", contract.PROTOCOL_ID)
        self.assertEqual(
            contract.V26_ACTIVE_EVENT_CONTRACT_ID,
            "binary_point_v25+heel_qualified_fsm_v2",
        )
        self.assertEqual(len(contract.CASES), 6)
        self.assertTrue(contract.AUTHORITY["v8_protocol_authorized"])
        self.assertFalse(contract.AUTHORITY["protected_trial_access_authorized"])
        self.assertFalse(contract.AUTHORITY["actor_updates_authorized"])
        for case in contract.CASES:
            self.assertEqual(
                case["destination"],
                (contract.RUN_ROOT / str(case["case_id"])).as_posix(),
            )

    def test_gate_accepts_v26_and_rejects_v20_provenance(self) -> None:
        summary = valid_summary()
        self.assertTrue(contract.replay_gate(summary)["passed"])
        drifted = copy.deepcopy(summary)
        drifted["binary_phase_event_gate"]["events"][0]["source"] = (
            "binary_phase_fsm_v20"
        )
        gate = contract.replay_gate(drifted)
        self.assertFalse(gate["passed"])
        self.assertFalse(gate["checks"]["binary_event_gate"])


class V8RunnerBindingTests(unittest.TestCase):
    def test_subprocess_reenters_v8_entrypoint(self) -> None:
        token = "v8-token-long-enough-for-the-worker-1234"
        command = runner._worker_command(contract.CASE_IDS[0], token)
        self.assertEqual(command[1], str(runner.Path(runner.__file__).resolve()))
        self.assertEqual(command[-2:], ["--execution-token", token])
        self.assertIs(runner.engine.contract, contract)
        self.assertIs(runner.engine.verify_lock, runner.verify_lock)
        self.assertIs(runner.engine.build_env_config, runner.build_env_config)

    def test_environment_builder_overrides_only_to_exact_v26_contract(self) -> None:
        case = contract.canonical_case(contract.CASE_IDS[0])
        legacy = runner.engine.legacy
        frozen_globals = (legacy.H0_CONFIG, legacy.V25_PROFILE, legacy.ANALOG_PROFILE)
        old_contract = "binary_point_v25+functional_contact_fsm_v1"

        def fake_builder(*, case_id, condition):
            self.assertEqual(case_id, "C")
            self.assertEqual(condition["id"], case["case_id"])
            return {
                "binary_phase_fsm_mode": "binary_active",
                "binary_phase_event_contract_id": old_contract,
                "phase_fsm_input_mode": "legacy_events",
                "online_grf_applied_sides": ["left"],
                "reward": {"morphology_weight": 0.0},
                "detector_sample_dt_s": 0.001,
                "segment_duration": 0.010,
                "episode_duration": 5.0,
            }

        with mock.patch.object(legacy, "build_env_config", side_effect=fake_builder):
            result = runner.build_env_config(case)
        self.assertEqual(
            result["binary_phase_event_contract_id"],
            contract.V26_ACTIVE_EVENT_CONTRACT_ID,
        )
        self.assertEqual(
            (legacy.H0_CONFIG, legacy.V25_PROFILE, legacy.ANALOG_PROFILE),
            frozen_globals,
        )

    def test_v26_accumulator_rejects_old_actor_source(self) -> None:
        accumulator = {
            "expected_event": "heel_strike",
            "identities": set(),
            "events": [],
            "duplicate_event_count": 0,
            "out_of_order_event_count": 0,
            "fallback_count": 0,
            "hard_invalid_count": 0,
        }
        event = {
            "side": "left",
            "event": "heel_strike",
            "event_time_s": 1.000,
            "confirmed_time_s": 1.005,
            "delivered_time_s": 1.010,
            "source": contract.V26_FSM_SOURCE,
            "event_contract_id": contract.V26_ACTIVE_EVENT_CONTRACT_ID,
        }
        runner._accumulate_binary_events_v26(
            accumulator,
            info={
                "binary_phase_fsm": {"events_this_step": [event]},
                "online_events": [{"side": "left", "source": "v25_fsm_v20"}],
            },
            boundary_s=1.010,
        )
        gate = runner._finalize_binary_event_gate_v26(accumulator, 5000)
        self.assertFalse(gate["passed"])
        self.assertEqual(gate["left_non_v26_source_count"], 1)


if __name__ == "__main__":
    unittest.main()
