"""Tests for the pure H0 primary-GRF-split closed-loop comparator."""

from __future__ import annotations

import copy
import math
import tempfile
import unittest
from pathlib import Path

from validation import compare_h0_primary_grf_split as comparator


def metric(value: float = 1.0, sample_count: int = 500) -> dict:
    return {"sample_count": sample_count, "rms": value, "abs_max": value}


def condition() -> dict:
    return {
        "condition_id": "trial02_deterministic",
        "trial_id": "02",
        "plateau_id": "04",
        "episode_start_time_s": 129.578,
        "episode_start_offset_s": 117.880,
        "seed": 126,
        "action_selection": "deterministic_mean",
        "sigma": 0.0,
        "noise_tape_sha256": None,
    }


def summary(value: float = 1.0) -> dict:
    sea = {}
    for joint in comparator.JOINTS:
        sea[joint] = {signal: metric(value, 5000) for signal in comparator.SEA_SIGNALS}
        sea[joint]["tau_input_saturated"] = {
            "sample_count": 5000,
            "count": 0,
            "fraction": 0.0,
        }
    return {
        **condition(),
        "behavior_role": "teacher_reference",
        "steps": 500,
        "end_reason": "episode_time_limit",
        "terminated": False,
        "truncated": True,
        "phase_valid_cycle_count": 2,
        "grf_penetration_max_m": 0.01,
        "action_clipped_values": 0,
        "timeout_count": 0,
        "safety_stop_count": 0,
        "fallback_count": 0,
        "hard_invalid_count": 0,
        "invalid_event_count": 0,
        "nonfinite_count": 0,
        "n_actor": 35,
        "n_observation": 84,
        "observation_dtype": "float32",
        "action_shape": [2],
        "action_dtype": "float32",
        "morphology_weight": 0.0,
        "event_contract_id": comparator.EXPECTED_EVENT_CONTRACT,
        "binary_phase_fsm_mode": "disabled",
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "preregistered_provenance": {"passed": True},
        "actor_feature_names": [f"actor_{index}" for index in range(35)],
        "observation_feature_names": [f"obs_{index}" for index in range(84)],
        "episode_metrics": {
            "reserve_norm_nm": metric(value),
            "residual_norm_nm": metric(value),
        },
        "sea_episode_metrics": sea,
        "counterfactual_teacher_error": {
            "sample_count": 500,
            "rmse": 0.015,
            "max_abs_error": 0.10,
            "served_action_teacher_dependency_count": 0,
            "action_selected_before_teacher_diagnostic": True,
        },
    }


class CommonRolloutGateTests(unittest.TestCase):
    def test_complete_valid_summary_passes(self) -> None:
        result = comparator.common_rollout_gate(summary())
        self.assertTrue(result["passed"])
        self.assertEqual(result["status"], "PASS_COMMON_ROLLOUT")
        self.assertEqual(comparator.common_rollout_checks(summary()), result["checks"])

    def test_each_mandatory_physical_gate_fails_closed(self) -> None:
        mutations = {
            "short": ("steps", 499),
            "wrong_end": ("end_reason", "safety_stop"),
            "too_few_cycles": ("phase_valid_cycle_count", 1),
            "penetration_at_strict_limit": (
                "grf_penetration_max_m",
                comparator.PENETRATION_LIMIT_M,
            ),
            "clipping": ("action_clipped_values", 1),
            "timeout": ("timeout_count", 1),
            "safety": ("safety_stop_count", 1),
            "fallback": ("fallback_count", 1),
            "hard_invalid": ("hard_invalid_count", 1),
            "invalid_event": ("invalid_event_count", 1),
            "nonfinite_ledger": ("nonfinite_count", 1),
            "actor_layout": ("n_actor", 34),
            "full_layout": ("n_observation", 83),
            "dtype": ("observation_dtype", "float64"),
            "morphology": ("morphology_weight", 0.05),
        }
        for name, (field, value) in mutations.items():
            with self.subTest(name=name):
                candidate = summary()
                candidate[field] = value
                self.assertFalse(comparator.common_rollout_gate(candidate)["passed"])

    def test_missing_or_nonfinite_metric_is_contract_error(self) -> None:
        missing = summary()
        del missing["episode_metrics"]["reserve_norm_nm"]["rms"]
        with self.assertRaises(comparator.H0PrimaryGRFSplitGateError):
            comparator.common_rollout_gate(missing)

        nonfinite = summary()
        nonfinite["sea_episode_metrics"][comparator.JOINTS[0]][
            comparator.SEA_SIGNALS[0]
        ]["rms"] = math.inf
        with self.assertRaises(comparator.H0PrimaryGRFSplitGateError):
            comparator.common_rollout_gate(nonfinite)

    def test_saturation_statistics_must_be_consistent(self) -> None:
        candidate = summary()
        saturation = candidate["sea_episode_metrics"][comparator.JOINTS[0]][
            "tau_input_saturated"
        ]
        saturation["count"] = 1
        self.assertFalse(comparator.common_rollout_gate(candidate)["passed"])

    def test_rate_signal_may_have_one_fewer_sample_per_segment(self) -> None:
        candidate = summary()
        for joint in comparator.JOINTS:
            candidate["sea_episode_metrics"][joint]["tau_spring_rate_nm_s"][
                "sample_count"
            ] = 4500
        self.assertTrue(comparator.common_rollout_gate(candidate)["passed"])


class ConditionMatchedGateTests(unittest.TestCase):
    def test_exact_addendum_caps_pass_and_epsilon_above_fails(self) -> None:
        reference = summary(1.0)
        candidate = summary(1.0)

        # Reserve: 1 + max(5%, 5 N m) = 6.
        candidate["episode_metrics"]["reserve_norm_nm"]["rms"] = 6.0
        candidate["episode_metrics"]["reserve_norm_nm"]["abs_max"] = 6.0
        # Residual and SEA: 1 + max(5%, 1e-6) = 1.05.
        candidate["episode_metrics"]["residual_norm_nm"]["rms"] = 1.05
        candidate["episode_metrics"]["residual_norm_nm"]["abs_max"] = 1.05
        candidate["sea_episode_metrics"][comparator.JOINTS[0]][
            comparator.SEA_SIGNALS[0]
        ]["rms"] = 1.05
        candidate["sea_episode_metrics"][comparator.JOINTS[0]][
            comparator.SEA_SIGNALS[0]
        ]["abs_max"] = 1.05
        self.assertTrue(
            comparator.condition_matched_gate(reference, candidate)["passed"]
        )

        candidate["episode_metrics"]["reserve_norm_nm"]["abs_max"] = 6.000001
        self.assertFalse(
            comparator.condition_matched_gate(reference, candidate)["passed"]
        )

    def test_relative_tolerance_dominates_for_large_reference(self) -> None:
        reference = summary(200.0)
        candidate = summary(210.0)
        result = comparator.condition_matched_gate(reference, candidate)
        self.assertTrue(result["passed"])

        candidate["episode_metrics"]["reserve_norm_nm"]["rms"] = 210.000001
        candidate["episode_metrics"]["reserve_norm_nm"]["abs_max"] = 210.000001
        self.assertFalse(
            comparator.condition_matched_gate(reference, candidate)["passed"]
        )

    def test_condition_identity_drift_fails(self) -> None:
        reference = summary()
        candidate = summary()
        candidate["noise_tape_sha256"] = "a" * 64
        result = comparator.condition_matched_gate(reference, candidate)
        self.assertFalse(result["passed"])
        failed = {item["name"] for item in result["checks"] if item["status"] == "FAIL"}
        self.assertIn("condition.noise_tape_sha256", failed)

    def test_stochastic_condition_requires_noise_tape_digest(self) -> None:
        reference = summary()
        candidate = summary()
        reference["sigma"] = candidate["sigma"] = 0.005
        with self.assertRaisesRegex(
            comparator.H0PrimaryGRFSplitGateError, "noise_tape_sha256"
        ):
            comparator.condition_matched_gate(reference, candidate)

    def test_saturation_has_no_tolerance(self) -> None:
        reference = summary()
        candidate = summary()
        saturation = candidate["sea_episode_metrics"][comparator.JOINTS[0]][
            "tau_input_saturated"
        ]
        saturation.update({"count": 1, "fraction": 1.0 / 5000.0})
        self.assertFalse(
            comparator.condition_matched_gate(reference, candidate)["passed"]
        )

    def test_adaptive_sea_sample_counts_need_not_match(self) -> None:
        reference = summary()
        candidate = summary()
        for joint in comparator.JOINTS:
            for signal in comparator.SEA_SIGNALS:
                candidate["sea_episode_metrics"][joint][signal][
                    "sample_count"
                ] -= 7
            candidate["sea_episode_metrics"][joint]["tau_input_saturated"][
                "sample_count"
            ] -= 7
        self.assertTrue(
            comparator.condition_matched_gate(reference, candidate)["passed"]
        )


class CounterfactualAndAggregateTests(unittest.TestCase):
    def test_counterfactual_limits_are_inclusive(self) -> None:
        metrics = {
            "sample_count": 500,
            "rmse": 0.015,
            "max_abs_error": 0.10,
            "served_action_teacher_dependency_count": 0,
            "action_selected_before_teacher_diagnostic": True,
        }
        self.assertTrue(comparator.gate_counterfactual(metrics)["passed"])
        metrics["rmse"] = 0.015000001
        self.assertFalse(comparator.gate_counterfactual(metrics)["passed"])

    def test_teacher_dependency_fails_counterfactual_gate(self) -> None:
        metrics = {
            "sample_count": 500,
            "rmse": 0.0,
            "max_abs_error": 0.0,
            "served_action_teacher_dependency_count": 1,
            "action_selected_before_teacher_diagnostic": True,
        }
        self.assertFalse(comparator.gate_counterfactual(metrics)["passed"])

    def test_complete_pair_passes_and_hashes_inputs(self) -> None:
        reference = summary()
        candidate = copy.deepcopy(reference)
        reference["behavior_role"] = "teacher_reference"
        candidate["behavior_role"] = "autonomous_candidate"
        result = comparator.compare_h0_primary_grf_split(reference, candidate)
        self.assertTrue(result["passed"])
        self.assertEqual(
            result["reference_summary_sha256"], comparator.payload_sha256(reference)
        )
        self.assertEqual(
            result["candidate_summary_sha256"], comparator.payload_sha256(candidate)
        )
        self.assertTrue(comparator.compare_rollouts(reference, candidate)["passed"])
        self.assertTrue(comparator.gate_candidate(reference, candidate)["passed"])


class StrictJsonTests(unittest.TestCase):
    def test_writer_is_atomic_strict_and_no_clobber(self) -> None:
        with tempfile.TemporaryDirectory() as raw:
            target = Path(raw) / "gate.json"
            comparator.write_json_exclusive(target, {"value": 1.0})
            self.assertEqual(comparator.strict_json_load(target), {"value": 1.0})
            with self.assertRaisesRegex(
                comparator.H0PrimaryGRFSplitGateError, "clobber"
            ):
                comparator.write_json_exclusive(target, {"value": 2.0})
            with self.assertRaises(comparator.H0PrimaryGRFSplitGateError):
                comparator.write_json_exclusive(Path(raw) / "nan.json", {"x": math.nan})

    def test_loader_rejects_duplicate_keys_and_nonfinite_constants(self) -> None:
        with tempfile.TemporaryDirectory() as raw:
            duplicate = Path(raw) / "duplicate.json"
            duplicate.write_text('{"x": 1, "x": 2}\n', encoding="utf-8")
            with self.assertRaisesRegex(
                comparator.H0PrimaryGRFSplitGateError, "duplicate"
            ):
                comparator.strict_json_load(duplicate)

            nonfinite = Path(raw) / "nonfinite.json"
            nonfinite.write_text('{"x": NaN}\n', encoding="utf-8")
            with self.assertRaisesRegex(
                comparator.H0PrimaryGRFSplitGateError, "non-finite"
            ):
                comparator.strict_json_load(nonfinite)


if __name__ == "__main__":
    unittest.main()
