"""Regression tests for always-readable training Return/Len cosmetics."""

from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock


ROOT = Path(__file__).resolve().parents[2]
BASELINE = ROOT / "Trajectory Generator" / "baseline_MLP"
sys.path.insert(0, str(BASELINE))

import train_ppo_mlp  # noqa: E402


class ConsoleEpisodeMetricTests(unittest.TestCase):
    def test_completed_episode_metrics_are_reported_without_estimation(self) -> None:
        result = train_ppo_mlp._console_episode_metrics(
            51.25,
            500.0,
            {"env_runners/reward/reward": 0.01},
            episode_duration_s=5.0,
            segment_duration_s=0.01,
        )

        self.assertEqual(result["episode_return"], 51.25)
        self.assertEqual(result["episode_length"], 500.0)
        self.assertEqual(result["source"], "completed_episode")
        self.assertFalse(result["estimated"])

    def test_missing_completed_episode_uses_current_reward_window(self) -> None:
        result = train_ppo_mlp._console_episode_metrics(
            None,
            None,
            {
                "env_runners/console/reward_sum_current": 472.32,
                "env_runners/console/reward_count_current": 4608.0,
            },
            episode_duration_s=5.0,
            segment_duration_s=0.01,
        )

        self.assertAlmostEqual(result["episode_return"], 51.25)
        self.assertEqual(result["episode_length"], 500.0)
        self.assertEqual(result["source"], "sampled_batch_projection")
        self.assertTrue(result["estimated"])

    def test_missing_reward_window_stays_explicitly_unavailable(self) -> None:
        result = train_ppo_mlp._console_episode_metrics(
            None,
            None,
            {},
            episode_duration_s=5.0,
            segment_duration_s=0.01,
        )

        self.assertIsNone(result["episode_return"])
        self.assertIsNone(result["episode_length"])
        self.assertEqual(result["source"], "unavailable")
        self.assertFalse(result["estimated"])

    def test_monitor_marks_estimates_in_bar_and_permanent_line(self) -> None:
        class _Progress:
            def __init__(self):
                self.updates = []
                self.lines = []

            def update(self, **kwargs):
                self.updates.append(kwargs)

            def log(self, message):
                self.lines.append(message)

            def finish(self, message=None):
                return None

        progress = _Progress()
        with tempfile.TemporaryDirectory() as tmp, mock.patch.object(
            train_ppo_mlp.progress_display,
            "LiveProgress",
            return_value=progress,
        ):
            monitor = train_ppo_mlp._TrainingMonitor(
                Path(tmp), total_iterations=2, progress=True
            )
            monitor.on_iteration_complete(
                2,
                37.4278,
                500.0,
                8704.0,
                565.0,
                "sampled_batch_projection",
            )

        self.assertIn("return 37.43~", progress.updates[-1]["suffix"])
        self.assertIn("len 500~", progress.updates[-1]["suffix"])
        self.assertIn(
            "metric_source=sampled_batch_projection", progress.lines[-1]
        )


if __name__ == "__main__":
    unittest.main()
