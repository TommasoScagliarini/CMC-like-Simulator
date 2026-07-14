from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

from validation.compare_rollout_traces import compare


def _row(action: list[float], q: list[float], target: list[float]) -> dict:
    return {
        "raw_policy_action": action,
        "applied_policy_action": action,
        "prosthetic_state": {
            "pros_knee_angle": q[0],
            "pros_ankle_angle": q[1],
            "pros_knee_angle_served_ref": q[0],
            "pros_ankle_angle_served_ref": q[1],
        },
        "imitation_target_q": {
            "pros_knee_angle": target[0],
            "pros_ankle_angle": target[1],
        },
    }


def _summary(episode_return: float) -> dict:
    return {
        "steps": 2,
        "episode_return": episode_return,
        "reward_mean": episode_return / 2.0,
        "phase_valid_cycle_count": 1,
        "grf_penetration_max_m": 0.02,
        "action_clipped_fraction": 0.0,
        "end_reason": "episode_time_limit",
    }


class CompareRolloutTracesTest(unittest.TestCase):
    def test_reports_aligned_closed_loop_differences(self) -> None:
        reference = [
            _row([0.1, 0.2], [-0.2, 0.1], [-0.3, 0.0]),
            _row([0.2, 0.3], [-0.1, 0.2], [-0.2, 0.1]),
        ]
        candidate = [
            _row([0.2, 0.2], [-0.1, 0.1], [-0.3, 0.0]),
            _row([0.3, 0.3], [0.0, 0.2], [-0.2, 0.1]),
        ]
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            paths = {
                "reference_trace": root / "reference_trace.json",
                "candidate_trace": root / "candidate_trace.json",
                "reference_summary": root / "reference_summary.json",
                "candidate_summary": root / "candidate_summary.json",
            }
            paths["reference_trace"].write_text(json.dumps(reference))
            paths["candidate_trace"].write_text(json.dumps(candidate))
            paths["reference_summary"].write_text(json.dumps(_summary(2.0)))
            paths["candidate_summary"].write_text(json.dumps(_summary(3.0)))

            result = compare(**{f"{key}_path": value for key, value in paths.items()})

        self.assertEqual(result["aligned_steps"], 2)
        self.assertAlmostEqual(
            result["candidate_minus_reference"]["raw_policy_action"]["rmse"],
            2**-0.5 * 0.1,
        )
        self.assertAlmostEqual(result["summary_delta"]["episode_return"], 1.0)


if __name__ == "__main__":
    unittest.main()
