"""Tests for the exact deterministic multi-start sampling contract."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path


_BASELINE = (
    Path(__file__).resolve().parents[1]
    / "Trajectory Generator"
    / "baseline_MLP"
)
sys.path.insert(0, str(_BASELINE))

from start_sampling import build_exact_start_sampling_contract  # noqa: E402


_OFFSETS = [1.756870983805102, 1.956870983805102, 2.156870983805102]


def _build(**overrides):
    values = {
        "offsets_s": _OFFSETS,
        "random_init": False,
        "num_env_runners": 12,
        "train_batch_size": 4608,
        "minibatch_size": 512,
    }
    values.update(overrides)
    return build_exact_start_sampling_contract(**values)


class ExactStartSamplingContractTests(unittest.TestCase):
    def test_balanced_12_runner_contract(self):
        contract = _build()

        self.assertEqual(contract.rollout_fragment_length, 384)
        self.assertEqual(contract.expected_steps_per_start, 1536)
        self.assertEqual(contract.runners_per_start, 4)
        self.assertEqual(contract.offsets_s, tuple(_OFFSETS))

    def test_thirteen_runners_are_rejected_for_three_starts(self):
        with self.assertRaisesRegex(ValueError, "num_env_runners must be divisible"):
            _build(num_env_runners=13, train_batch_size=4096)

    def test_4096_batch_is_rejected_for_twelve_runners(self):
        with self.assertRaisesRegex(
            ValueError, "train_batch_size must be divisible by num_env_runners"
        ):
            _build(train_batch_size=4096)

    def test_random_init_is_incompatible(self):
        with self.assertRaisesRegex(ValueError, "random_init is incompatible"):
            _build(random_init=True)

    def test_duplicate_offsets_are_rejected(self):
        with self.assertRaisesRegex(ValueError, "distinct"):
            _build(offsets_s=[1.75, 1.95, 1.75])

    def test_metric_label_collision_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "six-decimal"):
            _build(offsets_s=[1.75, 1.7500001, 2.15])

    def test_non_finite_and_negative_offsets_are_rejected(self):
        for invalid in (float("nan"), float("inf"), -0.1):
            with self.subTest(invalid=invalid):
                with self.assertRaisesRegex(ValueError, "finite, non-negative"):
                    _build(offsets_s=[1.75, 1.95, invalid])

    def test_partial_minibatch_is_rejected(self):
        with self.assertRaisesRegex(
            ValueError, "train_batch_size must be divisible by minibatch_size"
        ):
            _build(minibatch_size=500)


if __name__ == "__main__":
    unittest.main()
