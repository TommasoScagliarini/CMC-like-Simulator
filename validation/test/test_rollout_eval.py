"""Focused checks for rollout checkpoint/observation compatibility guards."""

from __future__ import annotations

import math
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock


BASELINE_DIR = (
    Path(__file__).resolve().parents[2] / "Trajectory Generator" / "baseline_MLP"
)
if str(BASELINE_DIR) not in sys.path:
    sys.path.insert(0, str(BASELINE_DIR))

import rollout_eval


class _Module:
    _n_actor = 31
    _n_full = 80


class RolloutObservationContractTests(unittest.TestCase):
    def test_nonnegative_rollout_metric_accepts_finite_values(self) -> None:
        self.assertEqual(
            rollout_eval._validated_nonnegative_rollout_metric(
                0.0, field="metric"
            ),
            0.0,
        )
        self.assertEqual(
            rollout_eval._validated_nonnegative_rollout_metric(
                1.25, field="metric"
            ),
            1.25,
        )

    def test_nonnegative_rollout_metric_rejects_corrupt_values(self) -> None:
        for value in (-1.0, math.nan, math.inf, -math.inf, True, None):
            with self.subTest(value=value):
                with self.assertRaisesRegex(RuntimeError, "non-negative"):
                    rollout_eval._validated_nonnegative_rollout_metric(
                        value, field="metric"
                    )

    def test_matching_contract_is_accepted(self) -> None:
        rollout_eval._validate_module_observation_contract(
            _Module(),
            tuple(f"actor_{index}" for index in range(31)),
            tuple(f"obs_{index}" for index in range(80)),
        )

    def test_schema_drift_is_rejected(self) -> None:
        with self.assertRaisesRegex(RuntimeError, "Prefix slicing"):
            rollout_eval._validate_module_observation_contract(
                _Module(),
                tuple(f"actor_{index}" for index in range(43)),
                tuple(f"obs_{index}" for index in range(88)),
            )

    def test_action_selection_cli_is_explicit(self) -> None:
        source = (BASELINE_DIR / "rollout_eval.py").read_text(encoding="utf-8")
        self.assertIn(
            'choices=("deterministic", "stochastic", "stochastic_held")',
            source,
        )
        self.assertIn("module.forward_exploration", source)
        self.assertIn("get_exploration_action_dist_cls", source)


class RolloutConfigurationTests(unittest.TestCase):
    @staticmethod
    def _parse(checkpoint: Path, *extra: str):
        with mock.patch.object(
            sys,
            "argv",
            [
                "rollout_eval.py",
                "--checkpoint",
                str(checkpoint),
                "--output-dir",
                str(checkpoint.parent / "unused-output"),
                *extra,
            ],
        ):
            return rollout_eval.parse_args()

    @staticmethod
    def _write_base_config(directory: Path) -> Path:
        config = directory / "default.yaml"
        config.write_text(
            "simulation:\n"
            "  episode_duration: 1.25\n"
            "  episode_start_offset_s: 0.375\n"
            "  policy_knots: 1\n"
            "reward:\n"
            "  reward_mode: ex_novo\n"
            "  morphology_weight: 1.0\n"
            "  morphology_knee_weight: 5.0\n"
            "  morphology_ankle_weight: 5.0\n"
            "  sea_motor_speed_weight: 0.125\n"
            "run:\n"
            "  output_dir: must-not-be-used-by-rollout\n",
            encoding="utf-8",
        )
        return config

    @staticmethod
    def _write_snapshot(run_dir: Path) -> None:
        (run_dir / rollout_eval.training_config.RESOLVED_CONFIG_NAME).write_text(
            "simulation:\n"
            "  episode_duration: 2.25\n"
            "reward:\n"
            "  morphology_weight: 0.125\n"
            "  sea_motor_speed_weight: 0.25\n",
            encoding="utf-8",
        )

    def test_latest_checkpoint_without_snapshot_uses_current_exnovo_config(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            run_dir = Path(tmp)
            checkpoint = run_dir / "rl_module_best"
            checkpoint.mkdir()
            with (
                mock.patch.object(
                    rollout_eval, "_latest_training_checkpoint", return_value=checkpoint
                ),
                mock.patch.object(
                    sys,
                    "argv",
                    ["rollout_eval.py", "--output-dir", str(run_dir / "output")],
                ),
            ):
                args = rollout_eval.parse_args()

        self.assertEqual(Path(args.checkpoint), checkpoint)
        self.assertEqual(args.episode_duration, 5.0)
        self.assertEqual(args.segment_duration, 0.01)
        self.assertEqual(args.policy_knots, 1)
        self.assertEqual(args.binary_phase_actor_fsm_version, "v3")
        self.assertEqual(args._cfg_reward["reward_mode"], "ex_novo")
        for field in (
            "morphology_weight", "morphology_knee_weight", "morphology_ankle_weight"
        ):
            with self.subTest(field=field):
                self.assertEqual(args._cfg_reward[field], 5.0)

    def test_explicit_checkpoint_without_snapshot_uses_default_config(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            run_dir = Path(tmp)
            checkpoint = run_dir / "rl_module_best"
            checkpoint.mkdir()
            default = self._write_base_config(run_dir)
            with mock.patch.object(
                rollout_eval.training_config, "DEFAULT_CONFIG_PATH", default
            ):
                args = self._parse(checkpoint)

        self.assertEqual(args.episode_duration, 1.25)
        self.assertEqual(args.episode_start_offset_s, 0.375)
        self.assertEqual(args._cfg_reward["morphology_weight"], 1.0)

    def test_older_checkpoint_snapshot_replaces_default_without_new_split_weights(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            run_dir = Path(tmp)
            checkpoint = run_dir / "rl_module_best"
            checkpoint.mkdir()
            default = self._write_base_config(run_dir)
            self._write_snapshot(run_dir)
            with mock.patch.object(
                rollout_eval.training_config, "DEFAULT_CONFIG_PATH", default
            ):
                args = self._parse(checkpoint)

        self.assertEqual(args.episode_duration, 2.25)
        self.assertEqual(args.episode_start_offset_s, 0.0)
        self.assertEqual(args.policy_knots, 3)
        self.assertEqual(args._cfg_reward["morphology_weight"], 0.125)
        self.assertNotIn("morphology_knee_weight", args._cfg_reward)
        self.assertNotIn("morphology_ankle_weight", args._cfg_reward)
        self.assertEqual(args._cfg_reward["sea_motor_speed_weight"], 0.25)
        self.assertEqual(Path(args.output_dir), run_dir / "unused-output")

    def test_explicit_config_and_cli_override_snapshot_in_order(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            run_dir = Path(tmp)
            checkpoint = run_dir / "rl_module_best"
            checkpoint.mkdir()
            default = self._write_base_config(run_dir)
            self._write_snapshot(run_dir)
            override = run_dir / "override.yaml"
            override.write_text(
                "simulation:\n"
                "  episode_duration: 3.25\n"
                "reward:\n"
                "  morphology_weight: 3.0\n"
                "  reward_mode: imitation\n",
                encoding="utf-8",
            )
            with mock.patch.object(
                rollout_eval.training_config, "DEFAULT_CONFIG_PATH", default
            ):
                configured = self._parse(checkpoint, "--config", str(override))
                overridden = self._parse(
                    checkpoint,
                    "--config", str(override),
                    "--episode-duration", "4.25",
                    "--reward-mode", "ex_novo",
                )

        self.assertEqual(configured.episode_duration, 3.25)
        self.assertEqual(configured._cfg_reward["morphology_weight"], 3.0)
        self.assertNotIn("morphology_knee_weight", configured._cfg_reward)
        self.assertEqual(configured._cfg_reward["sea_motor_speed_weight"], 0.25)
        self.assertEqual(overridden.episode_duration, 4.25)
        self.assertEqual(overridden.reward_mode, "ex_novo")
        self.assertEqual(overridden._cfg_reward["reward_mode"], "imitation")

    def test_no_auto_config_disables_snapshot_but_keeps_default_config(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            run_dir = Path(tmp)
            checkpoint = run_dir / "rl_module_best"
            checkpoint.mkdir()
            default = self._write_base_config(run_dir)
            self._write_snapshot(run_dir)
            with mock.patch.object(
                rollout_eval.training_config, "DEFAULT_CONFIG_PATH", default
            ):
                args = self._parse(checkpoint, "--no-auto-config")

        self.assertEqual(args.episode_duration, 1.25)
        self.assertEqual(args._cfg_reward["morphology_weight"], 1.0)
        self.assertEqual(args._cfg_reward["morphology_knee_weight"], 5.0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
