"""Regression tests for manual fine-tuning resume checkpoint selection."""

from __future__ import annotations

import sys
from pathlib import Path
from unittest import mock
import unittest


ROOT = Path(__file__).resolve().parents[1]
BASELINE = ROOT / "Trajectory Generator" / "baseline_MLP"
sys.path.insert(0, str(BASELINE))

import train_ppo_mlp  # noqa: E402


class ResumeSelectionTests(unittest.TestCase):
    def test_iteration_history_upsert_replaces_duplicate_logical_iteration(self) -> None:
        old = {"iteration": 38, "episode_return_mean": 1.0}
        newer = {"iteration": 38, "episode_return_mean": 2.0}
        following = {"iteration": 39, "episode_return_mean": 3.0}

        history = train_ppo_mlp._merge_iteration_history(
            [old, following],
            newer,
        )

        self.assertEqual([row["iteration"] for row in history], [38, 39])
        self.assertEqual(history[0]["episode_return_mean"], 2.0)

    def test_newer_output_checkpoint_prevents_manual_resume_rollback(self) -> None:
        output_dir = Path("finetune")
        requested = Path("seed/checkpoint_best")
        latest = Path("finetune/checkpoint_last")
        with (
            mock.patch.object(
                train_ppo_mlp,
                "_last_checkpoint",
                return_value=(latest, 38),
            ),
            mock.patch.object(
                train_ppo_mlp,
                "_checkpoint_logical_iteration",
                return_value=37,
            ),
        ):
            resume, iteration_start, message = (
                train_ppo_mlp._prefer_output_checkpoint_on_resume(
                    output_dir,
                    requested,
                    0,
                )
            )

        self.assertEqual(resume, latest)
        self.assertEqual(iteration_start, 39)
        self.assertIn("avoid rollback and duplicate iterations", message or "")

    def test_requested_checkpoint_is_kept_when_output_has_no_progress(self) -> None:
        requested = Path("seed/checkpoint_best")
        with mock.patch.object(
            train_ppo_mlp,
            "_last_checkpoint",
            return_value=(None, None),
        ):
            resume, iteration_start, message = (
                train_ppo_mlp._prefer_output_checkpoint_on_resume(
                    Path("empty-output"),
                    requested,
                    0,
                )
            )

        self.assertEqual(resume, requested)
        self.assertEqual(iteration_start, 0)
        self.assertIsNone(message)

    def test_explicit_iteration_start_disables_automatic_upgrade(self) -> None:
        requested = Path("seed/checkpoint_best")
        with mock.patch.object(train_ppo_mlp, "_last_checkpoint") as last_checkpoint:
            resume, iteration_start, message = (
                train_ppo_mlp._prefer_output_checkpoint_on_resume(
                    Path("finetune"), requested, 50
                )
            )

        last_checkpoint.assert_not_called()
        self.assertEqual(resume, requested)
        self.assertEqual(iteration_start, 50)
        self.assertIsNone(message)


if __name__ == "__main__":
    unittest.main()
