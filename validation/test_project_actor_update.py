"""Tests for nominal trust-region projection of actor updates."""

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

from project_actor_update import interpolate_actor_states


class ProjectActorUpdateTests(unittest.TestCase):
    def test_interpolation_changes_only_actor_tensors(self) -> None:
        source = {
            "pi.1.bias": np.asarray([0.0, 1.0]),
            "vf.bias": np.asarray([3.0]),
        }
        candidate = {
            "pi.1.bias": np.asarray([2.0, 5.0]),
            "vf.bias": np.asarray([9.0]),
        }

        projected = interpolate_actor_states(source, candidate, 0.25)

        np.testing.assert_allclose(projected["pi.1.bias"], [0.5, 2.0])
        np.testing.assert_array_equal(projected["vf.bias"], source["vf.bias"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
