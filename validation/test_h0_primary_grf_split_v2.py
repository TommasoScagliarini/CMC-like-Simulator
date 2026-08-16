from __future__ import annotations

import copy
import sys
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
VALIDATION_ROOT = REPO_ROOT / "validation"
BASELINE_ROOT = REPO_ROOT / "Trajectory Generator" / "baseline_MLP"
for path in (VALIDATION_ROOT, BASELINE_ROOT, REPO_ROOT / "Trajectory Generator"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

import compare_h0_primary_grf_split_v2 as gates  # noqa: E402
import run_h0_primary_grf_split_v2_adaptation as runner  # noqa: E402


def _metric(value: float, samples: int = 500) -> dict[str, float | int]:
    return {"rms": value, "abs_max": value, "sample_count": samples}


def _summary(*, role: str, counterfactual_rmse: float = 0.0) -> dict:
    sea = {}
    for joint in gates.JOINTS:
        sea[joint] = {signal: _metric(1.0, 5000) for signal in gates.SEA_SIGNALS}
        sea[joint]["tau_input_saturated"] = {
            "count": 0,
            "fraction": 0.0,
            "sample_count": 5000,
        }
    return {
        "trial_id": "02",
        "plateau_id": "04",
        "behavior_role": role,
        "action_selection": "deterministic",
        "seed": 126,
        "episode_start_time_s": 129.578,
        "episode_start_offset_s": 117.88,
        "sigma": 0.0,
        "noise_tape_sha256": "a" * 64,
        "steps": 500,
        "end_reason": "episode_time_limit",
        "terminated": False,
        "truncated": True,
        "phase_valid_cycle_count": 2,
        "invalid_event_count": 0,
        "grf_penetration_max_m": 0.024,
        "action_clipped_values": 0,
        "timeout_count": 0,
        "safety_stop_count": 0,
        "fallback_count": 0,
        "hard_invalid_count": 0,
        "nonfinite_count": 0,
        "n_actor": 35,
        "n_observation": 84,
        "observation_dtype": "float32",
        "action_shape": [2],
        "action_dtype": "float32",
        "morphology_weight": 0.0,
        "episode_metrics": {
            "reserve_norm_nm": _metric(100.0),
            "residual_norm_nm": _metric(1.0e-8),
        },
        "sea_episode_metrics": sea,
        "prescribed_counterfactual_error": {
            "sample_count": 500,
            "rmse": counterfactual_rmse,
            "max_abs_error": counterfactual_rmse,
            "served_action_teacher_dependency_count": 0,
            "action_selected_before_teacher_diagnostic": True,
        },
        "actor_feature_names": [f"actor_{index}" for index in range(35)],
        "observation_feature_names": [f"obs_{index}" for index in range(84)],
        "event_contract_id": runner.EVENT_CONTRACT,
        "binary_phase_fsm_mode": "disabled",
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "preregistered_provenance": {"passed": True},
    }


class H0PrimarySplitV2GateTests(unittest.TestCase):
    def test_pair_passes_exact_candidate(self) -> None:
        reference = _summary(role="prescribed_reference")
        candidate = _summary(role="candidate")
        self.assertTrue(gates.condition_matched_gate(reference, candidate)["passed"])

    def test_counterfactual_rmse_is_fail_closed(self) -> None:
        reference = _summary(role="prescribed_reference")
        candidate = _summary(role="candidate", counterfactual_rmse=0.031)
        self.assertFalse(gates.condition_matched_gate(reference, candidate)["passed"])

    def test_penetration_equal_to_limit_fails(self) -> None:
        summary = _summary(role="candidate")
        summary["grf_penetration_max_m"] = 0.025
        self.assertFalse(gates.common_rollout_gate(summary)["passed"])

    def test_teacher_dependency_fails(self) -> None:
        summary = _summary(role="candidate")
        summary["prescribed_counterfactual_error"][
            "served_action_teacher_dependency_count"
        ] = 1
        self.assertFalse(gates.prescribed_counterfactual_gate(summary)["passed"])

    def test_condition_identity_mismatch_fails(self) -> None:
        reference = _summary(role="prescribed_reference")
        candidate = _summary(role="candidate")
        candidate["seed"] = 999
        self.assertFalse(gates.condition_matched_gate(reference, candidate)["passed"])

    def test_reserve_tolerance_boundary(self) -> None:
        reference = _summary(role="prescribed_reference")
        candidate = _summary(role="candidate")
        candidate["episode_metrics"]["reserve_norm_nm"] = _metric(125.0)
        self.assertTrue(gates.condition_matched_gate(reference, candidate)["passed"])
        candidate["episode_metrics"]["reserve_norm_nm"] = _metric(125.0001)
        self.assertFalse(gates.condition_matched_gate(reference, candidate)["passed"])

    def test_fallback_fails_common_gate(self) -> None:
        summary = _summary(role="candidate")
        summary["fallback_count"] = 1
        self.assertFalse(gates.common_rollout_gate(summary)["passed"])

    def test_fit_contract_is_frozen(self) -> None:
        self.assertEqual(runner.FIT["learning_rate"], 5.0e-5)
        self.assertEqual(runner.FIT["anchor_weight"], 1.0e-2)
        self.assertEqual(runner.FIT["epochs"], 400)

    def test_v25_is_not_part_of_target_contract(self) -> None:
        self.assertEqual(
            runner.EVENT_CONTRACT,
            "primary_grf_split_v1+legacy_events_v1",
        )

    def test_nonregression_cannot_mutate_inputs(self) -> None:
        reference = _summary(role="prescribed_reference")
        candidate = _summary(role="candidate")
        original_reference = copy.deepcopy(reference)
        original_candidate = copy.deepcopy(candidate)
        gates.condition_matched_gate(reference, candidate)
        self.assertEqual(reference, original_reference)
        self.assertEqual(candidate, original_candidate)


if __name__ == "__main__":
    unittest.main()
