"""Focused tests for preregistered actor-adaptation row splits."""

from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
BASELINE_DIR = REPO_ROOT / "Trajectory Generator" / "baseline_MLP"
if str(BASELINE_DIR) not in sys.path:
    sys.path.insert(0, str(BASELINE_DIR))

import target_domain_imitation  # noqa: E402


class AdaptationSplitTests(unittest.TestCase):
    def resolve(
        self,
        sample_count: int,
        *,
        training_indices=None,
        validation_indices=None,
        seed: int = 123,
        validation_fraction: float = 0.2,
    ):
        return target_domain_imitation._resolve_adaptation_split(
            sample_count,
            validation_fraction=validation_fraction,
            rng=np.random.default_rng(seed),
            training_indices=training_indices,
            validation_indices=validation_indices,
        )

    def test_default_split_preserves_seeded_random_fraction_behavior(self) -> None:
        sample_count = 10
        seed = 321
        validation_fraction = 0.3
        legacy_rng = np.random.default_rng(seed)
        legacy_indices = legacy_rng.permutation(sample_count)
        validation_count = max(
            1,
            int(round(sample_count * validation_fraction)),
        )
        expected_validation = np.sort(legacy_indices[:validation_count])
        expected_training = np.asarray(legacy_indices[validation_count:], dtype=int)

        training, validation, report = self.resolve(
            sample_count,
            seed=seed,
            validation_fraction=validation_fraction,
        )

        np.testing.assert_array_equal(training, expected_training)
        np.testing.assert_array_equal(validation, expected_validation)
        self.assertEqual(report["mode"], "seeded_random_fraction")
        self.assertEqual(
            report["validation_fraction_applied"],
            validation_fraction,
        )
        self.assertTrue(report["indices_disjoint"])
        self.assertTrue(report["coverage_complete"])

    def test_explicit_split_is_sorted_and_has_order_independent_digests(self) -> None:
        first_training, first_validation, first_report = self.resolve(
            6,
            training_indices=[4, 0, 2, 5],
            validation_indices=[3, 1],
        )
        second_training, second_validation, second_report = self.resolve(
            6,
            training_indices=[5, 2, 0, 4],
            validation_indices=[1, 3],
        )

        np.testing.assert_array_equal(first_training, [0, 2, 4, 5])
        np.testing.assert_array_equal(first_validation, [1, 3])
        np.testing.assert_array_equal(first_training, second_training)
        np.testing.assert_array_equal(first_validation, second_validation)
        self.assertEqual(first_report["mode"], "explicit_indices")
        self.assertIsNone(first_report["validation_fraction_applied"])
        self.assertEqual(first_report["training_count"], 4)
        self.assertEqual(first_report["validation_count"], 2)
        self.assertEqual(
            first_report["training_indices_sha256"],
            second_report["training_indices_sha256"],
        )
        self.assertEqual(
            first_report["validation_indices_sha256"],
            second_report["validation_indices_sha256"],
        )
        self.assertEqual(
            first_report["assignment_sha256"],
            second_report["assignment_sha256"],
        )
        for key in (
            "training_indices_sha256",
            "validation_indices_sha256",
            "assignment_sha256",
        ):
            self.assertEqual(len(first_report[key]), 64)

    def test_explicit_indices_must_be_supplied_as_a_pair(self) -> None:
        with self.assertRaisesRegex(ValueError, "provided together"):
            self.resolve(3, training_indices=[0, 1])
        with self.assertRaisesRegex(ValueError, "provided together"):
            self.resolve(3, validation_indices=[2])

    def test_duplicate_indices_are_rejected_in_each_partition(self) -> None:
        cases = (
            ([0, 0, 1], [2, 3], "training_indices"),
            ([0, 1], [2, 2, 3], "validation_indices"),
        )
        for training, validation, expected_name in cases:
            with self.subTest(partition=expected_name):
                with self.assertRaisesRegex(
                    ValueError,
                    rf"{expected_name} contains duplicate",
                ):
                    self.resolve(
                        4,
                        training_indices=training,
                        validation_indices=validation,
                    )

    def test_out_of_bounds_and_non_integer_indices_are_rejected(self) -> None:
        cases = (
            ([-1, 0], [1, 2], "out-of-bounds"),
            ([0, 1], [2, 3], "out-of-bounds"),
            ([0.0, 1.0], [2], "only integer"),
            (np.asarray([[0, 1]]), [2], "one-dimensional"),
        )
        sample_counts = (3, 3, 3, 3)
        for (training, validation, message), sample_count in zip(
            cases,
            sample_counts,
        ):
            with self.subTest(message=message):
                with self.assertRaisesRegex(ValueError, message):
                    self.resolve(
                        sample_count,
                        training_indices=training,
                        validation_indices=validation,
                    )

    def test_overlap_is_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "overlap"):
            self.resolve(
                4,
                training_indices=[0, 1, 2],
                validation_indices=[2, 3],
            )

    def test_incomplete_coverage_is_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "missing indices: \\[2\\]"):
            self.resolve(
                4,
                training_indices=[0, 1],
                validation_indices=[3],
            )

    def test_empty_partition_is_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "training_indices must not be empty"):
            self.resolve(
                2,
                training_indices=[],
                validation_indices=[0, 1],
            )
        with self.assertRaisesRegex(
            ValueError,
            "validation_indices must not be empty",
        ):
            self.resolve(
                2,
                training_indices=[0, 1],
                validation_indices=[],
            )

    def test_adaptation_json_writer_is_strict_and_no_clobber(self) -> None:
        with tempfile.TemporaryDirectory() as raw:
            target = Path(raw) / "receipt.json"
            with mock.patch.object(
                target_domain_imitation.os,
                "link",
                side_effect=AssertionError("hard links are forbidden"),
            ):
                target_domain_imitation._write_strict_json_exclusive(
                    target, {"finite": 1.0}
                )
            with self.assertRaises(FileExistsError):
                target_domain_imitation._write_strict_json_exclusive(
                    target, {"finite": 2.0}
                )
            with self.assertRaises(ValueError):
                target_domain_imitation._write_strict_json_exclusive(
                    Path(raw) / "nan.json", {"value": float("nan")}
                )

    def test_fixed_final_epoch_split_is_train_only_and_complete(self) -> None:
        training, validation, report = (
            target_domain_imitation._resolve_fixed_final_epoch_split(
                2000,
                validation_fraction=0.0,
                patience=0,
                training_indices=None,
                validation_indices=None,
            )
        )

        np.testing.assert_array_equal(training, np.arange(2000))
        self.assertEqual(validation.size, 0)
        self.assertEqual(report["mode"], "fixed_final_epoch_all_rows")
        self.assertEqual(report["training_count"], 2000)
        self.assertEqual(report["validation_count"], 0)
        self.assertTrue(report["coverage_complete"])

    def test_fixed_final_epoch_rejects_model_selection_inputs(self) -> None:
        invalid = (
            {"validation_fraction": 0.1, "patience": 0},
            {"validation_fraction": 0.0, "patience": 1},
            {
                "validation_fraction": 0.0,
                "patience": 0,
                "training_indices": [0],
            },
            {
                "validation_fraction": 0.0,
                "patience": 0,
                "validation_indices": [0],
            },
        )
        for overrides in invalid:
            arguments = {
                "validation_fraction": 0.0,
                "patience": 0,
                "training_indices": None,
                "validation_indices": None,
                **overrides,
            }
            with self.subTest(arguments=arguments):
                with self.assertRaisesRegex(ValueError, "fixed_final_epoch"):
                    target_domain_imitation._resolve_fixed_final_epoch_split(
                        10,
                        **arguments,
                    )


if __name__ == "__main__":
    unittest.main(verbosity=2)
