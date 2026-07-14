"""Tests for diagnostic held Gaussian action noise."""

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

import exploration_noise


class ExplorationNoiseTests(unittest.TestCase):
    def test_scalar_sigma_is_broadcast(self) -> None:
        np.testing.assert_array_equal(
            exploration_noise.broadcast_sigma([0.005], 2),
            [0.005, 0.005],
        )

    def test_noise_is_repeated_for_the_requested_hold(self) -> None:
        process = exploration_noise.HeldStandardNormal(
            np.random.default_rng(123),
            (2,),
            hold_steps=3,
        )
        samples = [process.next() for _ in range(4)]

        np.testing.assert_array_equal(samples[0], samples[1])
        np.testing.assert_array_equal(samples[0], samples[2])
        self.assertFalse(np.array_equal(samples[0], samples[3]))

    def test_invalid_hold_is_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "hold_steps"):
            exploration_noise.HeldStandardNormal(
                np.random.default_rng(123),
                (2,),
                hold_steps=0,
            )


if __name__ == "__main__":
    unittest.main(verbosity=2)
