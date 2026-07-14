"""Regression tests for target-to-target policy slew limiting."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np


TRAJECTORY_DIR = Path(__file__).resolve().parents[1] / "Trajectory Generator"
if str(TRAJECTORY_DIR) not in sys.path:
    sys.path.insert(0, str(TRAJECTORY_DIR))

from osim_trj_cmc_like import CMCLikeProsthesisTrajectoryEnv


class _FilteredReferenceStub:
    def get(self, _time: float):
        q = {"knee": 0.0, "ankle": 0.0}
        qdot = {"knee": 0.0, "ankle": 0.0}
        return q, qdot, dict(qdot)


class TargetSlewLimiterTests(unittest.TestCase):
    def test_next_target_is_limited_from_previous_target_not_filter_output(self):
        env = object.__new__(CMCLikeProsthesisTrajectoryEnv)
        env.t = 0.0
        env.cfg = SimpleNamespace(pros_coords=("knee", "ankle"), dt=0.001)
        env.env_cfg = SimpleNamespace(
            policy_knots=1,
            action_mode="absolute",
            absolute_bounds_rad={"knee": (0.0, 1.0), "ankle": (0.0, 1.0)},
        )
        env.kin = _FilteredReferenceStub()
        env._last_policy_endpoint = np.asarray([0.5, 0.5], dtype=float)
        env._target_slew_rate_limit = np.asarray([1.0, 1.0], dtype=float)
        env._last_target_slew_terms = {}

        times, values, _derivatives = env._action_to_segment(
            np.ones((1, 2), dtype=float),
            target_t=0.1,
        )

        np.testing.assert_allclose(times, [0.0, 0.1])
        np.testing.assert_allclose(values[0], [0.0, 0.0])
        np.testing.assert_allclose(values[1], [0.6, 0.6])
        self.assertEqual(
            env._last_target_slew_terms["target_slew_limited_fraction"],
            1.0,
        )


if __name__ == "__main__":
    unittest.main(verbosity=2)
