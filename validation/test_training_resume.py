"""Regression tests for manual fine-tuning resume checkpoint selection."""

from __future__ import annotations

import json
import sys
import tempfile
from pathlib import Path
from unittest import mock
import unittest


ROOT = Path(__file__).resolve().parents[1]
BASELINE = ROOT / "Trajectory Generator" / "baseline_MLP"
sys.path.insert(0, str(BASELINE))

import train_ppo_mlp  # noqa: E402


class ResumeSelectionTests(unittest.TestCase):
    @staticmethod
    def _parse_initialization_args(*extra: str):
        temp_dir = tempfile.TemporaryDirectory()
        config = Path(temp_dir.name) / "config.yaml"
        config.write_text("{}\n", encoding="utf-8")
        argv = [
            "train_ppo_mlp.py",
            "--config",
            str(config),
            "--output-dir",
            "unused",
            "--asymmetric-actor-critic",
            *extra,
        ]
        with mock.patch.object(sys, "argv", argv):
            args = train_ppo_mlp.parse_args()
        temp_dir.cleanup()
        return args

    @staticmethod
    def _fake_algo(iteration: int = 7):
        class FakeAlgo:
            def __init__(self):
                self.iteration = iteration
                self.save_calls = []

            def save_to_path(self, path):
                path = Path(path)
                self.save_calls.append(path)
                path.mkdir(parents=True)
                (path / "rllib_checkpoint.json").write_text(
                    "{}\n", encoding="utf-8"
                )

        return FakeAlgo()

    @staticmethod
    def _save_fake_module(_algo, path):
        path = Path(path)
        path.mkdir(parents=True)
        (path / "module_state.pkl").write_bytes(b"module")

    def test_resume_learning_rate_override_updates_actual_optimizer(self) -> None:
        class Optimizer:
            lr = 1e-4

        class Learner:
            optimizer = Optimizer()

            def get_optimizers_for_module(self, module_id):
                self.module_id = module_id
                return [("default_optimizer", self.optimizer)]

            @staticmethod
            def _get_optimizer_lr(optimizer):
                return optimizer.lr

            @staticmethod
            def _set_optimizer_lr(optimizer, lr):
                optimizer.lr = lr

        learner = Learner()
        report = train_ppo_mlp._set_optimizer_learning_rate_on_learner(
            learner,
            learning_rate=5e-7,
        )

        self.assertEqual(learner.module_id, "default_policy")
        self.assertEqual(report[0]["before"], 1e-4)
        self.assertEqual(report[0]["after"], 5e-7)
        self.assertEqual(learner.optimizer.lr, 5e-7)

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

    def test_iteration_checkpoint_retention_is_explicit_and_defaults_off(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            config = Path(tmp) / "config.yaml"
            config.write_text("{}\n", encoding="utf-8")
            base_argv = [
                "train_ppo_mlp.py",
                "--config",
                str(config),
                "--output-dir",
                "unused",
            ]
            with mock.patch.object(sys, "argv", base_argv):
                default_args = train_ppo_mlp.parse_args()
            with mock.patch.object(
                sys,
                "argv",
                [*base_argv, "--retain-iteration-checkpoints"],
            ):
                retained_args = train_ppo_mlp.parse_args()

        self.assertFalse(default_args.retain_iteration_checkpoints)
        self.assertTrue(retained_args.retain_iteration_checkpoints)

    def test_warm_start_selects_canonical_full_h0_checkpoint(self) -> None:
        args = self._parse_initialization_args("--warm-start")

        train_ppo_mlp._validate_warm_start_args(args)

        self.assertTrue(args.warm_start)
        self.assertFalse(args.warm_start_raw)
        self.assertEqual(
            Path(args.resume_from),
            train_ppo_mlp._CANONICAL_H0_CHECKPOINT.resolve(),
        )
        self.assertIsNone(args.warm_start_source)

    def test_warm_start_raw_preserves_actor_only_source_semantics(self) -> None:
        args = self._parse_initialization_args(
            "--warm-start-raw",
            "--warm-start-raw-source",
            "CUSTOM_RL_MODULE",
        )

        train_ppo_mlp._validate_warm_start_args(args)

        self.assertFalse(args.warm_start)
        self.assertTrue(args.warm_start_raw)
        self.assertIsNone(args.resume_from)
        self.assertEqual(args.warm_start_source, "CUSTOM_RL_MODULE")

    def test_legacy_raw_source_alias_remains_available(self) -> None:
        args = self._parse_initialization_args(
            "--warm-start-raw",
            "--warm-start-source",
            "CUSTOM_RL_MODULE",
        )

        train_ppo_mlp._validate_warm_start_args(args)

        self.assertEqual(args.warm_start_source, "CUSTOM_RL_MODULE")

    def test_warm_start_modes_are_mutually_exclusive(self) -> None:
        args = self._parse_initialization_args(
            "--warm-start",
            "--warm-start-raw",
        )

        with self.assertRaisesRegex(
            SystemExit,
            "--warm-start and --warm-start-raw are mutually exclusive",
        ):
            train_ppo_mlp._validate_warm_start_args(args)

    def test_h0_warm_start_rejects_user_supplied_resume(self) -> None:
        args = self._parse_initialization_args(
            "--warm-start",
            "--resume-from",
            "OTHER_CHECKPOINT",
        )

        with self.assertRaisesRegex(
            SystemExit,
            "cannot be combined with a user-supplied --resume-from",
        ):
            train_ppo_mlp._validate_warm_start_args(args)

    def test_supervised_h0_worker_can_resume_run_checkpoint(self) -> None:
        args = self._parse_initialization_args(
            "--warm-start",
            "--resume-from",
            "RUN_CHECKPOINT_LAST",
            "--worker-process",
        )

        train_ppo_mlp._validate_warm_start_args(args)

        self.assertEqual(args.resume_from, "RUN_CHECKPOINT_LAST")

    def test_h0_warm_start_rejects_actor_source_options(self) -> None:
        args = self._parse_initialization_args(
            "--warm-start",
            "--warm-start-source",
            "OLD_STYLE_SOURCE",
        )

        with self.assertRaisesRegex(SystemExit, "use --warm-start-raw"):
            train_ppo_mlp._validate_warm_start_args(args)

    def test_iteration_milestone_has_stable_autoconfig_layout(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            run_dir = Path(tmp).resolve()
            (run_dir / train_ppo_mlp.training_config.RESOLVED_CONFIG_NAME).write_text(
                "model:\n  freeze_logstd: true\n",
                encoding="utf-8",
            )
            canonical_last = run_dir / "checkpoint_last"
            canonical_last.mkdir()
            (canonical_last / "sentinel").write_text("unchanged", encoding="utf-8")
            algo = self._fake_algo(iteration=7)

            with mock.patch.object(
                train_ppo_mlp,
                "_save_module",
                side_effect=self._save_fake_module,
            ):
                milestone = train_ppo_mlp._save_iteration_milestone(
                    algo,
                    run_dir,
                    logical_iteration=7,
                )

            self.assertEqual(milestone, run_dir / "milestone_iteration_000007")
            self.assertTrue((milestone / "checkpoint_last").is_dir())
            self.assertTrue((milestone / "rl_module_last").is_dir())
            checkpoint_meta = json.loads(
                (milestone / "checkpoint_last_meta.json").read_text(
                    encoding="utf-8"
                )
            )
            module_meta = json.loads(
                (milestone / "rl_module_last_meta.json").read_text(
                    encoding="utf-8"
                )
            )
            self.assertEqual(checkpoint_meta["logical_iteration"], 7)
            self.assertEqual(checkpoint_meta["rllib_training_iteration"], 7)
            self.assertEqual(
                checkpoint_meta["checkpoint"],
                str(milestone / "checkpoint_last"),
            )
            self.assertEqual(module_meta["logical_iteration"], 7)
            self.assertEqual(
                module_meta["rl_module"],
                str(milestone / "rl_module_last"),
            )
            resolved = (
                train_ppo_mlp.training_config.load_resolved_for_checkpoint(
                    milestone / "rl_module_last"
                )
            )
            self.assertTrue(resolved["model"]["freeze_logstd"])
            self.assertEqual(
                (canonical_last / "sentinel").read_text(encoding="utf-8"),
                "unchanged",
            )
            self.assertFalse(
                any(path.name.startswith(".milestone_iteration_000007.tmp-")
                    for path in run_dir.iterdir())
            )

    def test_iteration_milestone_publish_failure_leaves_no_partial_artifact(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            run_dir = Path(tmp).resolve()
            (run_dir / train_ppo_mlp.training_config.RESOLVED_CONFIG_NAME).write_text(
                "{}\n", encoding="utf-8"
            )
            algo = self._fake_algo(iteration=8)
            with (
                mock.patch.object(
                    train_ppo_mlp,
                    "_save_module",
                    side_effect=self._save_fake_module,
                ),
                mock.patch.object(
                    train_ppo_mlp.os,
                    "replace",
                    side_effect=OSError("publish failed"),
                ),
                self.assertRaisesRegex(OSError, "publish failed"),
            ):
                train_ppo_mlp._save_iteration_milestone(
                    algo,
                    run_dir,
                    logical_iteration=8,
                )

            self.assertFalse((run_dir / "milestone_iteration_000008").exists())
            self.assertFalse(
                any(path.name.startswith(".milestone_iteration_000008.tmp-")
                    for path in run_dir.iterdir())
            )

    def test_existing_iteration_milestone_is_never_overwritten(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            run_dir = Path(tmp).resolve()
            (run_dir / train_ppo_mlp.training_config.RESOLVED_CONFIG_NAME).write_text(
                "{}\n", encoding="utf-8"
            )
            milestone = run_dir / "milestone_iteration_000009"
            milestone.mkdir()
            sentinel = milestone / "sentinel"
            sentinel.write_text("original", encoding="utf-8")
            algo = self._fake_algo(iteration=9)

            with self.assertRaisesRegex(FileExistsError, "refusing to overwrite"):
                train_ppo_mlp._save_iteration_milestone(
                    algo,
                    run_dir,
                    logical_iteration=9,
                )

            self.assertEqual(sentinel.read_text(encoding="utf-8"), "original")
            self.assertEqual(algo.save_calls, [])


if __name__ == "__main__":
    unittest.main()
