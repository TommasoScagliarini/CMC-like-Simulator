from __future__ import annotations

import unittest

import numpy as np

from validation.compare_policy_checkpoints import (
    _fixed_observation_metrics,
    _parameter_comparison,
)


def _state() -> dict[str, np.ndarray]:
    return {
        "pi.0.0.weight": np.asarray([[0.2, -0.1, 0.3], [-0.4, 0.2, 0.1]]),
        "pi.0.0.bias": np.asarray([0.0, 0.1]),
        "pi.0.2.weight": np.asarray([[0.3, -0.2], [0.1, 0.4]]),
        "pi.0.2.bias": np.asarray([0.0, -0.1]),
        "pi.1.weight": np.asarray(
            [[0.2, 0.1], [-0.1, 0.3], [0.0, 0.0], [0.0, 0.0]]
        ),
        "pi.1.bias": np.asarray([0.0, 0.0, np.log(0.005), np.log(0.005)]),
    }


class ComparePolicyCheckpointsTest(unittest.TestCase):
    def test_identical_states_have_zero_shift(self) -> None:
        state = _state()
        observations = np.asarray([[0.1, 0.2, -0.1], [0.0, -0.2, 0.3]])
        metrics = _fixed_observation_metrics(state, state, observations, 2)

        self.assertEqual(metrics["mean_delta_abs_max"], 0.0)
        self.assertEqual(metrics["logstd_delta_abs_max"], 0.0)
        self.assertAlmostEqual(
            metrics["empirical_kl_reference_to_candidate_mean"], 0.0
        )
        self.assertTrue(_parameter_comparison(state, state)["exact"])

    def test_mean_shift_produces_expected_equal_variance_kl(self) -> None:
        reference = _state()
        candidate = {key: value.copy() for key, value in reference.items()}
        candidate["pi.1.bias"][:2] += np.asarray([0.001, -0.002])
        observations = np.asarray([[0.1, 0.2, -0.1], [0.0, -0.2, 0.3]])

        metrics = _fixed_observation_metrics(
            reference, candidate, observations, action_dim=2
        )
        expected_kl = (0.001**2 + 0.002**2) / (2.0 * 0.005**2)

        self.assertAlmostEqual(
            metrics["empirical_kl_reference_to_candidate_mean"], expected_kl
        )
        self.assertEqual(metrics["logstd_delta_abs_max"], 0.0)
        self.assertAlmostEqual(
            _parameter_comparison(reference, candidate)["max_abs_diff"], 0.002
        )


if __name__ == "__main__":
    unittest.main()
