"""Focused tests for offline stochastic-rollout noise analysis."""

from __future__ import annotations

import unittest

import numpy as np

from validation import analyze_stochastic_rollouts


class AnalyzeStochasticRolloutsTests(unittest.TestCase):
    def test_noise_is_recovered_from_sampled_actions_and_means(self) -> None:
        means = np.asarray([[1.0, -1.0], [2.0, -2.0]], dtype=float)
        noise = np.asarray([[0.1, -0.2], [-0.1, 0.2]], dtype=float)
        sigma = np.full_like(noise, 0.1)

        metrics = analyze_stochastic_rollouts._noise_metrics(
            means + noise,
            means,
            sigma,
        )

        self.assertAlmostEqual(metrics["realized_noise_rms"], np.sqrt(0.025))
        np.testing.assert_allclose(
            metrics["realized_noise_rms_per_action"],
            [0.1, 0.2],
        )
        np.testing.assert_allclose(metrics["realized_noise_mean_per_action"], 0.0)
        np.testing.assert_allclose(
            metrics["standardized_noise_rms_per_action"],
            [1.0, 2.0],
        )


if __name__ == "__main__":
    unittest.main(verbosity=2)
