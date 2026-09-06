"""Regression tests for independent knee/ankle morphology weighting.

OpenSim-free: these tests exercise only the production reward composer and
configuration serialization.
"""

from __future__ import annotations

import math
import struct
import sys
import unittest
from pathlib import Path


REPO = Path(__file__).resolve().parents[1]
BASELINE = REPO / "Trajectory Generator" / "baseline_MLP"
if str(BASELINE) not in sys.path:
    sys.path.insert(0, str(BASELINE))

import reward_function  # noqa: E402


def _terms(knee: float, ankle: float) -> dict[str, float]:
    return {
        "morphology_loss": 0.5 * (knee + ankle),
        "morphology_knee_loss": knee,
        "morphology_ankle_loss": ankle,
    }


class MorphologyJointWeightTests(unittest.TestCase):
    def test_config_round_trip_preserves_optional_split(self) -> None:
        legacy = reward_function.RewardConfig.from_mapping(
            {"morphology_weight": 1.05}
        )
        self.assertIsNone(legacy.morphology_knee_weight)
        self.assertIsNone(legacy.morphology_ankle_weight)
        self.assertIsNone(legacy.to_dict()["morphology_knee_weight"])
        split = reward_function.RewardConfig.from_mapping(
            {
                "morphology_weight": 1.05,
                "morphology_knee_weight": 0.75,
                "morphology_ankle_weight": 2.5,
            }
        )
        self.assertEqual(split.morphology_knee_weight, 0.75)
        self.assertEqual(split.morphology_ankle_weight, 2.5)
        self.assertEqual(
            reward_function.RewardConfig.from_mapping(split.to_dict()), split
        )

    def test_equal_explicit_weights_are_legacy_reward_exact(self) -> None:
        terms = _terms(0.125, 0.75)
        legacy = reward_function.RewardConfig(morphology_weight=1.05)
        split = reward_function.RewardConfig(
            morphology_weight=1.05,
            morphology_knee_weight=1.05,
            morphology_ankle_weight=1.05,
        )
        legacy_reward, legacy_components = reward_function.compute_reward(
            terms, legacy
        )
        split_reward, split_components = reward_function.compute_reward(terms, split)
        self.assertEqual(
            struct.pack("!d", legacy_reward), struct.pack("!d", split_reward)
        )
        self.assertEqual(
            struct.pack("!d", legacy_components["morphology_term"]),
            struct.pack("!d", split_components["morphology_term"]),
        )
        self.assertEqual(split_components["morphology_split_explicit"], 1.0)
        self.assertEqual(split_components["morphology_knee_weight_effective"], 1.05)
        self.assertEqual(split_components["morphology_ankle_weight_effective"], 1.05)

    def test_asymmetric_weights_compose_independent_terms(self) -> None:
        cfg = reward_function.RewardConfig(
            morphology_weight=0.0,
            morphology_knee_weight=0.5,
            morphology_ankle_weight=2.0,
        )
        _reward, components = reward_function.compute_reward(_terms(0.2, 0.6), cfg)
        self.assertAlmostEqual(components["morphology_knee_term"], 0.05)
        self.assertAlmostEqual(components["morphology_ankle_term"], 0.6)
        self.assertAlmostEqual(components["morphology_term"], 0.65)

    def test_zero_weight_joint_does_not_evaluate_nan(self) -> None:
        cfg = reward_function.RewardConfig(
            morphology_weight=0.0,
            morphology_knee_weight=0.0,
            morphology_ankle_weight=2.0,
        )
        reward, components = reward_function.compute_reward(
            {
                "morphology_loss": float("nan"),
                "morphology_knee_loss": float("nan"),
                "morphology_ankle_loss": 0.6,
            },
            cfg,
        )
        self.assertTrue(math.isfinite(reward))
        self.assertEqual(components["morphology_knee_term"], 0.0)
        self.assertAlmostEqual(components["morphology_ankle_term"], 0.6)
        self.assertAlmostEqual(components["morphology_term"], 0.6)

    def test_enabled_joint_requires_finite_loss(self) -> None:
        cfg = reward_function.RewardConfig(
            morphology_weight=0.0,
            morphology_knee_weight=0.0,
            morphology_ankle_weight=1.0,
        )
        with self.assertRaisesRegex(ValueError, "morphology_ankle_loss"):
            reward_function.compute_reward(
                {
                    "morphology_loss": float("nan"),
                    "morphology_knee_loss": 0.0,
                    "morphology_ankle_loss": float("nan"),
                },
                cfg,
            )

    def test_invalid_joint_weight_fails_closed(self) -> None:
        for value in (-1.0, float("nan"), float("inf")):
            cfg = reward_function.RewardConfig(
                morphology_weight=0.0,
                morphology_ankle_weight=value,
            )
            with self.assertRaisesRegex(ValueError, "morphology_ankle_weight"):
                reward_function.compute_reward(_terms(0.0, 0.0), cfg)


if __name__ == "__main__":
    unittest.main()
