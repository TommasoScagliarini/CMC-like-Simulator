"""Focused tests for target-domain actor imitation helpers."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

import numpy as np


BASELINE_DIR = (
    Path(__file__).resolve().parents[1] / "Trajectory Generator" / "baseline_MLP"
)
if str(BASELINE_DIR) not in sys.path:
    sys.path.insert(0, str(BASELINE_DIR))

import target_domain_imitation
import target_domain_markov_adaptation
import target_domain_noise_adaptation


class TargetDomainImitationTests(unittest.TestCase):
    def test_absolute_action_encoding(self) -> None:
        values = np.asarray([[-0.75, 0.0], [0.0, 0.7]], dtype=float)
        action = target_domain_imitation.encode_absolute_action(
            values,
            ("knee", "ankle"),
            {"knee": (-1.5, 0.0), "ankle": (-0.7, 0.7)},
        )
        np.testing.assert_allclose(action, [[0.0, 0.0], [1.0, 1.0]])

    def test_absolute_action_encoding_clips_outside_bounds(self) -> None:
        action = target_domain_imitation.encode_absolute_action(
            np.asarray([[-2.0, 1.0]], dtype=float),
            ("knee", "ankle"),
            {"knee": (-1.5, 0.0), "ankle": (-0.7, 0.7)},
        )
        np.testing.assert_array_equal(action, [[-1.0, 1.0]])

    def test_target_contract_requires_disabled_clock_and_enabled_slew(self) -> None:
        valid = {
            "action_mode": "absolute",
            "policy_knots": 1,
            "random_init": False,
            "gait_clock_enable": False,
            "asymmetric_actor_critic": True,
            "pros_knee_target_slew_rate_limit_rad_s": 2.5,
            "pros_ankle_target_slew_rate_limit_rad_s": 2.0,
        }
        target_domain_imitation._validate_target_contract(valid)

        invalid = dict(valid, gait_clock_enable=True)
        with self.assertRaisesRegex(ValueError, "gait_clock_enable"):
            target_domain_imitation._validate_target_contract(invalid)

        invalid = dict(valid, pros_ankle_target_slew_rate_limit_rad_s=0.0)
        with self.assertRaisesRegex(ValueError, "ankle target slew"):
            target_domain_imitation._validate_target_contract(invalid)

    def test_dagger_trace_uses_time_aligned_teacher_labels(self) -> None:
        teacher = {
            "observations": np.asarray([[1.0, 2.0], [3.0, 4.0]], np.float32),
            "actions": np.asarray([[0.1, 0.2], [0.3, 0.4]], np.float32),
            "times": np.asarray([10.0, 10.01], np.float64),
            "actor_feature_names": np.asarray(["a", "b"]),
        }
        trace = [
            {
                "step": 1,
                "actor_observation_vector_before": [1.5, 2.5],
                "raw_policy_action": [0.0, 0.0],
            },
            {
                "step": 2,
                "actor_observation_vector_before": [3.5, 4.5],
                "raw_policy_action": [0.2, 0.1],
            },
        ]

        aggregate, summary = target_domain_imitation.aggregate_dagger_traces(
            teacher, [trace], trace_repeat=2
        )

        np.testing.assert_allclose(
            aggregate["observations"],
            [[1.0, 2.0], [3.0, 4.0], [1.5, 2.5], [3.5, 4.5], [1.5, 2.5], [3.5, 4.5]],
        )
        np.testing.assert_allclose(
            aggregate["actions"],
            [[0.1, 0.2], [0.3, 0.4], [0.1, 0.2], [0.3, 0.4], [0.1, 0.2], [0.3, 0.4]],
        )
        self.assertEqual(summary["aggregate_samples"], 6)
        self.assertEqual(summary["dagger_training_samples"], 4)

    def test_dagger_interpolation_preserves_discrete_features(self) -> None:
        teacher = {
            "observations": np.asarray([[0.0, 0.0]], np.float32),
            "actions": np.asarray([[0.1]], np.float32),
            "times": np.asarray([10.0], np.float64),
            "actor_feature_names": np.asarray(
                ["pros_knee_angle", "phase_fsm_wait_hs"]
            ),
        }
        trace = [
            {
                "step": 1,
                "actor_observation_vector_before": [2.0, 1.0],
                "raw_policy_action": [0.0],
            }
        ]

        aggregate, summary = target_domain_imitation.aggregate_dagger_traces(
            teacher, [trace], trace_repeat=1, interpolation_steps=1
        )

        np.testing.assert_allclose(
            aggregate["observations"],
            [[0.0, 0.0], [2.0, 1.0], [1.0, 1.0]],
        )
        self.assertEqual(summary["interpolated_samples"], 1)

    def test_recovery_dataset_uses_nominal_actions_for_disturbed_states(self) -> None:
        nominal = [
            {
                "step": 1,
                "time": 10.0,
                "actor_observation_vector_before": [0.0, 0.0],
                "raw_policy_action": [0.1],
            },
            {
                "step": 2,
                "time": 10.01,
                "actor_observation_vector_before": [1.0, 1.0],
                "raw_policy_action": [0.2],
            },
        ]
        disturbed = [
            {
                "step": 1,
                "time": 10.0,
                "actor_observation_vector_before": [0.4, 1.0],
                "raw_policy_action": [0.8],
            },
            {
                "step": 2,
                "time": 10.01,
                "actor_observation_vector_before": [1.4, 0.0],
                "raw_policy_action": [-0.8],
            },
        ]
        dataset, report = target_domain_noise_adaptation.build_recovery_dataset(
            nominal,
            [disturbed],
            ["pros_knee_angle", "phase_fsm_wait_hs"],
            nominal_repeat=1,
            trace_repeat=1,
            interpolation_steps=1,
        )

        np.testing.assert_allclose(
            dataset["actions"], [[0.1], [0.2], [0.1], [0.1], [0.2], [0.2]]
        )
        np.testing.assert_allclose(
            dataset["observations"][3], [0.2, 1.0]
        )
        np.testing.assert_allclose(
            dataset["observations"][5], [1.2, 0.0]
        )
        self.assertEqual(report["aggregate_samples"], 6)
        self.assertEqual(report["unique_recovery_samples"], 4)

    def test_noise_recovery_stops_before_discrete_phase_mismatch(self) -> None:
        nominal = [
            {"actor_observation_vector_before": [0.0, 1.0]},
            {"actor_observation_vector_before": [0.1, 1.0]},
            {"actor_observation_vector_before": [0.2, 0.0]},
        ]
        disturbed = [
            {"actor_observation_vector_before": [0.0, 1.0]},
            {"actor_observation_vector_before": [0.3, 0.0]},
            {"actor_observation_vector_before": [0.4, 0.0]},
        ]

        filtered, report = (
            target_domain_noise_adaptation.truncate_before_discrete_mismatch(
                nominal,
                disturbed,
                ["pros_knee_angle", "phase_fsm_stance_after_hs"],
            )
        )

        self.assertEqual(len(filtered), 1)
        self.assertEqual(report["first_discrete_mismatch_step"], 2)
        self.assertEqual(report["retained_steps"], 1)

    def test_markov_dataset_anchors_nominal_and_labels_recovery_with_teacher(self) -> None:
        names = [
            "sensor",
            "pros_knee_angle_previous_endpoint",
            "pros_knee_angle_served_ref",
            "pros_knee_angle_served_ref_vel",
            "pros_knee_angle_served_ref_accel",
            "pros_knee_angle_sea_u",
            "pros_ankle_angle_previous_endpoint",
            "pros_ankle_angle_served_ref",
            "pros_ankle_angle_served_ref_vel",
            "pros_ankle_angle_served_ref_accel",
            "pros_ankle_angle_sea_u",
        ]

        def row(step, offset, mean):
            values = [offset + index for index in range(len(names))]
            return {
                "step": step,
                "actor_observation_before": dict(zip(names, values)),
                "actor_observation_vector_before": values,
                "policy_action_mean": mean,
            }

        nominal = [row(1, 0.0, [0.1, 0.2]), row(2, 1.0, [0.3, 0.4])]
        recovery = [row(1, 2.0, [0.8, 0.9]), row(2, 3.0, [0.7, 0.6])]
        teacher = np.asarray([[0.11, 0.22], [0.33, 0.44]], np.float32)
        dataset, report = (
            target_domain_markov_adaptation.build_markov_recovery_dataset(
                nominal,
                [recovery],
                teacher,
                nominal_repeat=2,
                recovery_repeat=1,
            )
        )

        np.testing.assert_allclose(
            dataset["actions"],
            [[0.1, 0.2], [0.3, 0.4], [0.1, 0.2], [0.3, 0.4], [0.11, 0.22], [0.33, 0.44]],
        )
        self.assertEqual(report["aggregate_samples"], 6)
        self.assertEqual(len(report["trainable_markov_features"]), 10)

    def test_markov_dataset_accepts_rollout_raw_action_fallback(self) -> None:
        names = [
            "sensor",
            "pros_knee_angle_previous_endpoint",
            "pros_knee_angle_served_ref",
            "pros_knee_angle_served_ref_vel",
            "pros_knee_angle_served_ref_accel",
            "pros_knee_angle_sea_u",
            "pros_ankle_angle_previous_endpoint",
            "pros_ankle_angle_served_ref",
            "pros_ankle_angle_served_ref_vel",
            "pros_ankle_angle_served_ref_accel",
            "pros_ankle_angle_sea_u",
        ]
        values = list(range(len(names)))
        row = {
            "step": 1,
            "actor_observation_before": dict(zip(names, values)),
            "actor_observation_vector_before": values,
            "policy_action_mean": None,
            "raw_policy_action": [0.1, 0.2],
        }

        dataset, _ = target_domain_markov_adaptation.build_markov_recovery_dataset(
            [row],
            [[row]],
            np.asarray([[0.3, 0.4]], np.float32),
            nominal_repeat=1,
            recovery_repeat=1,
        )

        np.testing.assert_allclose(dataset["actions"], [[0.1, 0.2], [0.3, 0.4]])

    def test_markov_update_audit_rejects_changes_outside_selected_columns(self) -> None:
        keys = (
            "pi_encoder.0.weight",
            "pi.0.0.weight",
            "pi_encoder.0.bias",
            "pi_encoder.2.weight",
            "pi_encoder.2.bias",
            "pi.0.0.bias",
            "pi.0.2.weight",
            "pi.0.2.bias",
            "pi.1.weight",
            "pi.1.bias",
        )
        source = {key: np.zeros((2, 3), np.float32) for key in keys}
        adapted = {key: value.copy() for key, value in source.items()}
        adapted["pi_encoder.0.weight"][:, 2] = 1.0
        adapted["pi.0.0.weight"][:, 2] = 1.0
        audit = target_domain_markov_adaptation.selected_column_update_audit(
            source, adapted, ["a", "b", "markov"], ["markov"]
        )
        self.assertTrue(audit["only_selected_columns_changed"])

        adapted["pi.1.bias"][0, 0] = 0.5
        audit = target_domain_markov_adaptation.selected_column_update_audit(
            source, adapted, ["a", "b", "markov"], ["markov"]
        )
        self.assertFalse(audit["only_selected_columns_changed"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
