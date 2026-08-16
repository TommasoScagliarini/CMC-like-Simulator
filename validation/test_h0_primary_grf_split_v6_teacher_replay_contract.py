"""Pure tests for the V6 V25-active teacher-replay contract."""

from __future__ import annotations

import copy
import unittest

from validation import h0_primary_grf_split_v6_teacher_replay_contract as contract


def valid_summary(case_id: str = contract.CASE_IDS[0]) -> dict:
    case = contract.canonical_case(case_id)
    events = []
    for index, name in enumerate(("heel_strike", "toe_off", "heel_strike", "toe_off")):
        event_time = 0.5 + index * 0.5
        events.append(
            {
                "event": name,
                "source": "binary_phase_fsm_v20",
                "event_contract_id": contract.V25_ACTIVE_EVENT_CONTRACT_ID,
                "event_time_s": event_time,
                "confirmed_time_s": event_time + 0.005,
                "delivered_time_s": event_time + 0.01,
                "startup_partial_stance": False,
            }
        )
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.ROLLOUT_COLLECTED_STATUS,
        "protocol_id": contract.PROTOCOL_ID,
        "collector_id": contract.COLLECTOR_ID,
        "case_id": case_id,
        "action_selection": case["action_selection"],
        "episode_start_offset_s": case["episode_start_offset_s"],
        "action_seed": case["action_seed"],
        "runtime_seed": case["runtime_seed"],
        "source_h0_id": contract.SOURCE_H0_ID,
        "source_observation_contract_id": (contract.SOURCE_OBSERVATION_CONTRACT_ID),
        "target_observation_contract_id": (contract.TARGET_OBSERVATION_CONTRACT_ID),
        "binary_phase_fsm_mode": "binary_active",
        "binary_phase_event_contract_id": (contract.V25_ACTIVE_EVENT_CONTRACT_ID),
        "morphology_weight": contract.MORPHOLOGY_WEIGHT,
        "behavior": "FROZEN_V5_RAW_ACTION_REPLAY",
        "steps": contract.EXPECTED_STEPS,
        "control_window_count": contract.EXPECTED_CONTROL_WINDOWS,
        "v25_raw_sensor_sample_count": contract.EXPECTED_RAW_SENSOR_SAMPLES,
        "end_reason": "episode_time_limit",
        "terminated": False,
        "truncated": True,
        "phase_valid_cycle_count": contract.MINIMUM_VALID_CYCLES,
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
            "sample_count": contract.EXPECTED_RAW_SENSOR_SAMPLES,
            "event_count": len(events),
            "events": events,
            "duplicate_event_count": 0,
            "out_of_order_event_count": 0,
            "left_non_v25_source_count": 0,
            "fallback_count": 0,
            "hard_invalid_count": 0,
        },
        "n_actor": contract.EXPECTED_ACTOR_FEATURES,
        "n_observation": contract.EXPECTED_FULL_FEATURES,
        "observation_dtype": contract.EXPECTED_OBSERVATION_DTYPE,
        "actor_feature_names": list(contract.EXPECTED_ACTOR_FEATURE_NAMES),
        "observation_feature_names": list(contract.EXPECTED_OBSERVATION_FEATURE_NAMES),
        "invariant_columns": list(contract.INVARIANT_COLUMNS),
        "candidate_created": False,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
    }


class ContractIdentityTests(unittest.TestCase):
    def test_lineage_cases_destinations_and_invariants_are_exact(self) -> None:
        self.assertEqual(
            contract.PROTOCOL_ID,
            "AB06_H0_PRIMARY_SPLIT_V6_V25_RESIDUAL_DAGGER",
        )
        self.assertEqual(len(contract.CASES), 6)
        self.assertEqual(
            contract.CASE_IDS,
            (
                "deterministic_offset_minus_0p20",
                "deterministic_offset_nominal",
                "deterministic_offset_plus_0p20",
                "stochastic_nominal_seed_126",
                "stochastic_nominal_seed_127",
                "stochastic_nominal_seed_128",
            ),
        )
        self.assertEqual(
            contract.INVARIANT_COLUMNS,
            (*range(2, 10), *range(25, 35)),
        )
        self.assertEqual(len(contract.EXPECTED_OBSERVATION_FEATURE_NAMES), 84)
        self.assertEqual(
            contract.EXPECTED_OBSERVATION_FEATURE_NAMES[:35],
            contract.EXPECTED_ACTOR_FEATURE_NAMES,
        )
        for case in contract.CASES:
            self.assertEqual(
                case["destination"],
                (contract.RUN_ROOT / case["case_id"]).as_posix(),
            )
            self.assertIn("/qualification/baseline/", case["baseline_trace"])

    def test_authority_is_development_only(self) -> None:
        self.assertTrue(
            contract.AUTHORITY["development_teacher_action_replay_authorized"]
        )
        self.assertTrue(contract.AUTHORITY["binary_active_v25_execution_authorized"])
        for key in (
            "residual_candidate_creation_authorized",
            "actor_updates_authorized",
            "critic_updates_authorized",
            "ppo_updates_authorized",
            "training_authorized",
            "protected_trial_access_authorized",
            "reserve_trial_access_authorized",
            "runtime_promotion_authorized",
            "primary_grf_modification_authorized",
            "detector_retuning_authorized",
            "sea_semantic_modification_authorized",
        ):
            self.assertFalse(contract.AUTHORITY[key], key)

    def test_unknown_case_fails_closed(self) -> None:
        with self.assertRaises(ValueError):
            contract.canonical_case("trial_05")


class ReplayGateTests(unittest.TestCase):
    def test_complete_summary_passes(self) -> None:
        gate = contract.replay_gate(valid_summary())
        self.assertTrue(gate["passed"])
        self.assertEqual(gate["status"], contract.ROLLOUT_PASS_STATUS)
        self.assertTrue(all(gate["checks"].values()))

    def test_each_required_scalar_fails_closed(self) -> None:
        mutations = {
            "schema_version": 5,
            "status": "wrong",
            "protocol_id": "wrong",
            "collector_id": "wrong",
            "case_id": "trial_05",
            "action_selection": "wrong",
            "source_h0_id": "wrong",
            "source_observation_contract_id": "wrong",
            "target_observation_contract_id": "wrong",
            "behavior": "AUTONOMOUS_POLICY",
            "binary_phase_fsm_mode": "binary_shadow",
            "binary_phase_event_contract_id": "wrong",
            "morphology_weight": 0.05,
            "steps": 499,
            "control_window_count": 4999,
            "v25_raw_sensor_sample_count": 4999,
            "end_reason": "failure",
            "terminated": True,
            "truncated": False,
            "phase_valid_cycle_count": 1,
            "grf_penetration_max_m": 0.025,
            "invariant_mismatch_count": 1,
            "teacher_mean_mismatch_count": 1,
            "time_mismatch_count": 1,
            "step_contract_failure_count": 1,
            "action_clipped_values": 1,
            "timeout_count": 1,
            "safety_stop_count": 1,
            "invalid_event_count": 1,
            "hard_invalid_count": 1,
            "nonfinite_count": 1,
            "so_solver_unaccepted_count": 1,
            "sea_plugin_fallback_count": 1,
            "n_actor": 34,
            "n_observation": 83,
            "observation_dtype": "float64",
            "actor_feature_names": [],
            "observation_feature_names": [],
            "invariant_columns": [],
            "candidate_created": True,
            "actor_updates": 1,
            "critic_updates": 1,
            "ppo_updates": 1,
            "protected_trials_opened": [5],
        }
        for field, invalid in mutations.items():
            with self.subTest(field=field):
                summary = valid_summary()
                summary[field] = invalid
                self.assertFalse(contract.replay_gate(summary)["passed"])

    def test_zero_counters_reject_booleans(self) -> None:
        for field in (
            "invariant_mismatch_count",
            "teacher_mean_mismatch_count",
            "time_mismatch_count",
            "step_contract_failure_count",
            "action_clipped_values",
            "timeout_count",
            "safety_stop_count",
            "invalid_event_count",
            "hard_invalid_count",
            "nonfinite_count",
            "so_solver_unaccepted_count",
            "sea_plugin_fallback_count",
            "actor_updates",
            "critic_updates",
            "ppo_updates",
        ):
            with self.subTest(field=field):
                summary = valid_summary()
                summary[field] = False
                self.assertFalse(contract.replay_gate(summary)["passed"])

    def test_binary_event_gate_is_structural_and_fail_closed(self) -> None:
        bad_gates = []
        for field, invalid in (
            ("passed", False),
            ("sample_count", 4999),
            ("event_count", 3),
            ("events", []),
            ("duplicate_event_count", 1),
            ("out_of_order_event_count", 1),
            ("left_non_v25_source_count", 1),
            ("fallback_count", 1),
            ("hard_invalid_count", 1),
        ):
            gate = copy.deepcopy(valid_summary()["binary_phase_event_gate"])
            gate[field] = invalid
            bad_gates.append((field, gate))
        bad_gates.append(("not_mapping", True))
        malformed_event = copy.deepcopy(valid_summary()["binary_phase_event_gate"])
        malformed_event["events"][1]["event"] = "heel_strike"
        bad_gates.append(("non_alternating_events", malformed_event))
        for label, bad_gate in bad_gates:
            with self.subTest(label=label):
                summary = valid_summary()
                summary["binary_phase_event_gate"] = bad_gate
                self.assertFalse(contract.replay_gate(summary)["passed"])


if __name__ == "__main__":
    unittest.main()
