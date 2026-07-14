"""Unit tests for the closed-loop actor fragility diagnostic."""

from __future__ import annotations

import unittest
import sys
from pathlib import Path

import numpy as np

VALIDATION_DIR = Path(__file__).resolve().parent
if str(VALIDATION_DIR) not in sys.path:
    sys.path.insert(0, str(VALIDATION_DIR))

from diagnose_actor_fragility import actor_logits, actor_mean_jacobian


class ActorFragilityDiagnosticTests(unittest.TestCase):
    def setUp(self) -> None:
        self.state = {
            "pi.0.0.weight": np.asarray([[0.4, -0.2], [0.1, 0.3]]),
            "pi.0.0.bias": np.asarray([0.05, -0.1]),
            "pi.0.2.weight": np.asarray([[0.2, 0.5], [-0.4, 0.1]]),
            "pi.0.2.bias": np.asarray([0.01, 0.02]),
            "pi.1.weight": np.asarray(
                [[0.3, -0.2], [0.1, 0.4], [0.0, 0.0], [0.0, 0.0]]
            ),
            "pi.1.bias": np.asarray([0.1, -0.2, -5.0, -5.0]),
        }

    def test_jacobian_matches_central_difference(self) -> None:
        observation = np.asarray([[0.25, -0.4]], dtype=float)
        analytical = actor_mean_jacobian(self.state, observation, action_dim=2)[0]
        numerical = np.empty_like(analytical)
        epsilon = 1e-6
        for feature in range(observation.shape[1]):
            plus = observation.copy()
            minus = observation.copy()
            plus[0, feature] += epsilon
            minus[0, feature] -= epsilon
            numerical[:, feature] = (
                actor_logits(self.state, plus)[0, :2]
                - actor_logits(self.state, minus)[0, :2]
            ) / (2.0 * epsilon)
        np.testing.assert_allclose(analytical, numerical, rtol=1e-5, atol=1e-7)


if __name__ == "__main__":
    unittest.main(verbosity=2)
