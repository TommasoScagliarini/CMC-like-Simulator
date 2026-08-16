"""Unit tests for the H0 primary-GRF split adaptation contract."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

import numpy as np


BASELINE_ROOT = (
    Path(__file__).resolve().parents[1] / "Trajectory Generator" / "baseline_MLP"
)
if str(BASELINE_ROOT) not in sys.path:
    sys.path.insert(0, str(BASELINE_ROOT))

import primary_grf_split_adaptation as split  # noqa: E402


def actor_names() -> list[str]:
    names = [f"feature_{index}" for index in range(split.ACTOR_FEATURE_COUNT)]
    names[split.LOAD_FEATURE_INDEX] = split.PRIMARY_LOAD_FEATURE
    names[split.CONTACT_FEATURE_INDEX] = split.PRIMARY_CONTACT_FEATURE
    for index, name in zip(split.PHASE_FEATURE_INDICES, split.PHASE_FEATURES):
        names[index] = name
    return names


def phase_observation(value: float = 0.0) -> dict[str, float]:
    return {name: value for name in split.PHASE_FEATURES}


def boundary_info(primary_n: float, detector_n: float) -> dict:
    return {
        "online_grf": {
            "left": {"normal_force": primary_n, "in_contact": primary_n > 1.0}
        },
        "online_grf_detector": {
            "left": {
                "normal_force": detector_n,
                "in_contact": detector_n > 1.0,
            }
        },
    }


class PairedViewTests(unittest.TestCase):
    def test_only_load_and_contact_are_replaced(self) -> None:
        observation = np.linspace(-0.2, 0.2, 84, dtype=np.float32)
        observation[10] = np.float32(100.0 / 800.0)
        observation[11] = np.float32(1.0)
        paired = split.build_paired_views(
            observation,
            actor_names(),
            boundary_info(100.0, 20.0),
            body_weight_n=800.0,
            teacher_phase_observation=phase_observation(0.5),
        )
        expected = paired.student.copy()
        expected[10] = np.float32(20.0 / 800.0)
        expected[11] = np.float32(1.0)
        expected[17:25] = np.float32(0.5)
        np.testing.assert_array_equal(paired.teacher, expected)
        mask = np.ones(split.ACTOR_FEATURE_COUNT, dtype=bool)
        mask[[10, 11, *split.PHASE_FEATURE_INDICES]] = False
        self.assertEqual(
            paired.student[mask].tobytes(), paired.teacher[mask].tobytes()
        )

    def test_primary_mismatch_fails_closed(self) -> None:
        observation = np.zeros(84, dtype=np.float32)
        with self.assertRaisesRegex(
            split.PrimaryGRFSplitContractError, "primary GRF"
        ):
            split.build_paired_views(
                observation,
                actor_names(),
                boundary_info(100.0, 20.0),
                body_weight_n=800.0,
                teacher_phase_observation=phase_observation(),
            )

    def test_reset_requires_both_streams_empty(self) -> None:
        observation = np.zeros(84, dtype=np.float32)
        paired = split.build_paired_views(
            observation,
            actor_names(),
            {"online_grf": {}, "online_grf_detector": {}},
            body_weight_n=800.0,
            reset_boundary=True,
        )
        np.testing.assert_array_equal(paired.student, paired.teacher)

        with self.assertRaisesRegex(
            split.PrimaryGRFSplitContractError, "both be empty"
        ):
            split.build_paired_views(
                observation,
                actor_names(),
                {
                    "online_grf": {},
                    "online_grf_detector": {
                        "left": {"normal_force": 0.0, "in_contact": False}
                    },
                },
                body_weight_n=800.0,
                reset_boundary=True,
            )


class DatasetTests(unittest.TestCase):
    @staticmethod
    def group(trial: str, states: int) -> dict:
        student = np.zeros((states, 35), dtype=np.float32)
        teacher = student.copy()
        teacher[:, 10] = np.float32(0.25)
        labels = np.full((states, 2), 0.1, dtype=np.float32)
        return {
            "trial_id": trial,
            "student_views": student,
            "teacher_views": teacher,
            "teacher_means": labels,
            "actor_feature_names": np.asarray(actor_names()),
        }

    def test_group_split_has_two_records_and_no_leakage(self) -> None:
        dataset, train, validation, summary = split.assemble_grouped_dataset(
            [self.group("02", 2), self.group("04", 2), self.group("08", 2)],
            training_trials=("02", "04"),
            validation_trials=("08",),
        )
        self.assertEqual(dataset["observations"].shape, (12, 35))
        self.assertEqual(len(train), 8)
        self.assertEqual(len(validation), 4)
        self.assertEqual(summary["training_group_count"], 4)
        self.assertEqual(summary["validation_group_count"], 2)
        self.assertFalse(
            set(dataset["group_ids"][train])
            & set(dataset["group_ids"][validation])
        )

    def test_train_only_corpus_has_no_validation_rows(self) -> None:
        dataset, train, validation, summary = (
            split.assemble_train_only_dataset(
                [self.group("123", 2), self.group("124", 2)],
                training_trials=("123", "124"),
            )
        )
        self.assertEqual(dataset["observations"].shape, (8, 35))
        np.testing.assert_array_equal(train, np.arange(8))
        self.assertEqual(validation.size, 0)
        self.assertEqual(summary["partition_mode"], "fixed_final_epoch_train_only")
        self.assertEqual(summary["training_trials"], ["123", "124"])
        self.assertEqual(summary["validation_trials"], [])
        self.assertEqual(summary["training_records"], 8)
        self.assertEqual(summary["validation_records"], 0)

    def test_offline_gate_is_view_specific(self) -> None:
        dataset, _train, validation, _summary = split.assemble_grouped_dataset(
            [self.group("02", 2), self.group("04", 2), self.group("08", 2)],
            training_trials=("02", "04"),
            validation_trials=("08",),
        )
        targets = dataset["actions"]
        source = targets.copy()
        source[dataset["view_roles"] == split.STUDENT_VIEW] += np.float32(0.04)
        adapted = targets.copy()
        adapted[dataset["view_roles"] == split.STUDENT_VIEW] += np.float32(0.004)
        adapted[dataset["view_roles"] == split.TEACHER_VIEW] += np.float32(0.002)
        logits = np.concatenate(
            [adapted, np.full_like(adapted, -5.2983174)], axis=1
        )
        result = split.offline_adaptation_gate(
            source_predictions=source,
            adapted_predictions=adapted,
            targets=targets,
            validation_indices=validation,
            view_roles=dataset["view_roles"],
            all_adapted_logits=logits,
        )
        self.assertTrue(result["passed"])
        self.assertGreater(result["student_rmse_reduction_fraction"], 0.5)


if __name__ == "__main__":
    unittest.main()
