from __future__ import annotations

import copy
import unittest

from validation import compare_h0_primary_split_v6_qualification as gates
from validation import h0_primary_split_v6_qualification_contract as contract


CANDIDATE_ID = "H0_PRIMARY_SPLIT_V6_P1_0123456789abcdef"


def _metric(value: float, samples: int = 500) -> dict[str, float | int]:
    return {"sample_count": samples, "rms": value, "abs_max": value * 1.1}


def _summary(role: str, case_id: str) -> dict[str, object]:
    case = next(row for row in contract.canonical_cases() if row["case_id"] == case_id)
    role_spec = contract.role_contract(role)
    sea: dict[str, object] = {}
    for joint in contract.JOINTS:
        joint_metrics: dict[str, object] = {
            signal: _metric(
                1.0,
                samples=5000 if signal != "tau_spring_rate_nm_s" else 4500,
            )
            for signal in contract.SEA_SIGNALS
        }
        joint_metrics["tau_input_saturated"] = {
            "sample_count": 5000,
            "count": 0,
            "fraction": 0.0,
        }
        sea[joint] = joint_metrics
    result: dict[str, object] = {
        "schema_version": contract.SCHEMA_VERSION,
        "protocol_id": contract.PROTOCOL_ID,
        "candidate_id": CANDIDATE_ID,
        "role": role,
        "actor_id": (
            contract.BASELINE_ACTOR_ID if role == contract.BASELINE_ROLE else CANDIDATE_ID
        ),
        "contract_id": role_spec["contract_id"],
        "actor_input_view": role_spec["actor_input_view"],
        "observation_semantics": role_spec["observation_semantics"],
        "primary_load_contract_id": role_spec["primary_load_contract_id"],
        "phase_fsm_input_mode": role_spec["phase_fsm_input_mode"],
        "event_contract_id": role_spec["event_contract_id"],
        "binary_phase_fsm_mode": role_spec["binary_phase_fsm_mode"],
        "case_id": case_id,
        "action_selection": case["action_selection"],
        "episode_start_offset_s": case["episode_start_offset_s"],
        "action_seed": case["action_seed"],
        "runtime_seed": case["runtime_seed"],
        "sigma": case["sigma"],
        "noise_tape_file_sha256": "b" * 64,
        "noise_tape_array_sha256": "a" * 64,
        "steps": contract.EXPECTED_STEPS,
        "end_reason": "episode_time_limit",
        "terminated": False,
        "truncated": True,
        "phase_valid_cycle_count": 3,
        "grf_penetration_max_m": 0.024,
        "binary_phase_sensor_sample_count": contract.EXPECTED_CONTROL_WINDOWS,
        "raw_so_fallback_count": 3,
        "policy_step_terminal_so_fallback_count": 1,
        "so_policy_id": contract.SO_POLICY_ID,
        "so_solver_control_window_count": contract.EXPECTED_CONTROL_WINDOWS,
        "so_solver_bounded_ls_invocation_count": 3,
        "so_solver_verified_bounded_ls_count": 3,
        "so_solver_verified_status0_max_iter_count": 2,
        "so_solver_hard_fallback_count": 2,
        "so_solver_bounded_ls_unsuccessful_count": 2,
        "n_actor": contract.EXPECTED_ACTOR_FEATURES,
        "n_observation": contract.EXPECTED_FULL_FEATURES,
        "observation_dtype": contract.EXPECTED_DTYPE,
        "action_shape": list(contract.EXPECTED_ACTION_SHAPE),
        "action_dtype": contract.EXPECTED_DTYPE,
        "online_grf_applied_sides": ["left"],
        "morphology_weight": contract.MORPHOLOGY_WEIGHT,
        "episode_metrics": {
            "reserve_norm_nm": _metric(100.0),
            "residual_norm_nm": _metric(1.0),
        },
        "sea_episode_metrics": sea,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "runtime_contract_mismatch_count": 0,
    }
    for name in contract.ZERO_REQUIRED_COUNTS:
        result[name] = 0
    return result


class QualificationComparatorTests(unittest.TestCase):
    CASE = contract.CASE_IDS[1]

    def test_both_role_specific_contracts_pass(self) -> None:
        for role in (contract.BASELINE_ROLE, contract.CANDIDATE_ROLE):
            with self.subTest(role=role):
                result = gates.common_rollout_gate(
                    _summary(role, self.CASE),
                    role=role,
                    case_id=self.CASE,
                    candidate_id=CANDIDATE_ID,
                )
                self.assertTrue(result["passed"])
                self.assertEqual(result["status"], contract.ROLLOUT_PASS_STATUS)

    def test_candidate_cannot_claim_legacy_routing(self) -> None:
        summary = _summary(contract.CANDIDATE_ROLE, self.CASE)
        summary["event_contract_id"] = "legacy_events"
        summary["binary_phase_fsm_mode"] = "disabled"
        result = gates.common_rollout_gate(
            summary,
            role=contract.CANDIDATE_ROLE,
            case_id=self.CASE,
            candidate_id=CANDIDATE_ID,
        )
        self.assertFalse(result["passed"])

    def test_v25_requires_all_five_thousand_raw_samples(self) -> None:
        summary = _summary(contract.CANDIDATE_ROLE, self.CASE)
        summary["binary_phase_sensor_sample_count"] = 4999
        result = gates.common_rollout_gate(
            summary,
            role=contract.CANDIDATE_ROLE,
            case_id=self.CASE,
            candidate_id=CANDIDATE_ID,
        )
        self.assertFalse(result["passed"])

    def test_invalid_event_is_absolute_zero_gate(self) -> None:
        summary = _summary(contract.BASELINE_ROLE, self.CASE)
        summary["invalid_event_count"] = 1
        result = gates.common_rollout_gate(
            summary,
            role=contract.BASELINE_ROLE,
            case_id=self.CASE,
            candidate_id=CANDIDATE_ID,
        )
        self.assertFalse(result["passed"])

    def test_condition_matched_caps_and_saturation_are_applied(self) -> None:
        baseline = _summary(contract.BASELINE_ROLE, self.CASE)
        candidate = _summary(contract.CANDIDATE_ROLE, self.CASE)
        candidate["episode_metrics"]["reserve_norm_nm"]["rms"] = 104.9
        candidate["episode_metrics"]["reserve_norm_nm"]["abs_max"] = 110.0
        result = gates.condition_matched_gate(
            baseline,
            candidate,
            case_id=self.CASE,
            candidate_id=CANDIDATE_ID,
        )
        self.assertTrue(result["passed"])

        failing = copy.deepcopy(candidate)
        failing["episode_metrics"]["reserve_norm_nm"]["rms"] = 105.1
        failing["episode_metrics"]["reserve_norm_nm"]["abs_max"] = 110.0
        result = gates.condition_matched_gate(
            baseline,
            failing,
            case_id=self.CASE,
            candidate_id=CANDIDATE_ID,
        )
        self.assertFalse(result["passed"])

    def test_condition_identity_includes_both_tape_hashes(self) -> None:
        baseline = _summary(contract.BASELINE_ROLE, self.CASE)
        candidate = _summary(contract.CANDIDATE_ROLE, self.CASE)
        candidate["noise_tape_array_sha256"] = "c" * 64
        result = gates.condition_matched_gate(
            baseline,
            candidate,
            case_id=self.CASE,
            candidate_id=CANDIDATE_ID,
        )
        self.assertFalse(result["passed"])

    def test_baseline_metric_schema_matches_contract_order(self) -> None:
        metrics = gates.baseline_case_metrics(
            _summary(contract.BASELINE_ROLE, self.CASE)
        )
        self.assertEqual(list(metrics["sea"]), [row[0] for row in contract.SEA_TOLERANCES])
        self.assertEqual(
            list(metrics["reserve"]),
            [row[0] for row in contract.RESERVE_TOLERANCES],
        )

    def test_nonfinite_metric_fails_closed(self) -> None:
        summary = _summary(contract.BASELINE_ROLE, self.CASE)
        summary["episode_metrics"]["reserve_norm_nm"]["rms"] = float("nan")
        with self.assertRaises(gates.QualificationGateError):
            gates.common_rollout_gate(
                summary,
                role=contract.BASELINE_ROLE,
                case_id=self.CASE,
                candidate_id=CANDIDATE_ID,
            )


if __name__ == "__main__":
    unittest.main()
