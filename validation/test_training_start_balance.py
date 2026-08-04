"""Trainer-level checks for exact start coverage and per-start diagnostics."""

from __future__ import annotations

import argparse
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock


BASELINE = (
    Path(__file__).resolve().parents[1]
    / "Trajectory Generator"
    / "baseline_MLP"
)
sys.path.insert(0, str(BASELINE))

import start_condition_metrics  # noqa: E402
import start_sampling  # noqa: E402
import tb_logging  # noqa: E402
import train_ppo_mlp  # noqa: E402


OFFSETS = [1.756870983805102, 1.956870983805102, 2.156870983805102]


class TrainingStartBalanceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        train_ppo_mlp.tb_logging = tb_logging
        train_ppo_mlp.start_condition_metrics = start_condition_metrics
        train_ppo_mlp.start_sampling = start_sampling

    def _args(self):
        return argparse.Namespace(
            episode_start_offset_choices_s=list(OFFSETS),
            num_epochs=1,
            minibatch_size=512,
            _start_sampling_contract=(
                start_sampling.build_exact_start_sampling_contract(
                    offsets_s=OFFSETS,
                    random_init=False,
                    num_env_runners=12,
                    train_batch_size=4608,
                    minibatch_size=512,
                )
            ),
        )

    @staticmethod
    def _coverage(value=1536.0):
        return {
            "env_runners/episode_start_steps_current/"
            f"{tb_logging.start_offset_metric_label(offset)}": value
            for offset in OFFSETS
        }

    @staticmethod
    def _learner_metrics(
        *,
        connector_out=4622.0,
        pre_rows=4622.0,
        removed_rows=14.0,
        compacted_rows=4608.0,
        trained=4608.0,
        max_kl=0.008,
        min_kl=0.001,
        kl_minibatches=9.0,
        kl_nonfinite=0.0,
    ):
        metrics = {
            "learners/__all_modules__/learner_connector_sum_episodes_length_in": 4608.0,
            "learners/__all_modules__/learner_connector_sum_episodes_length_out": connector_out,
            "learners/__all_modules__/num_module_steps_trained": trained,
            "learners/default_policy/num_module_steps_trained": trained,
            "learners/default_policy/start_condition_batch/pre_rows": pre_rows,
            "learners/default_policy/start_condition_batch/removed_rows": removed_rows,
            "learners/default_policy/start_condition_batch/compacted_rows": compacted_rows,
            "learners/default_policy/start_condition_batch/interleaved_rows": 4608.0,
            "learners/default_policy/start_condition_batch/interleaved_start_conditions": 3.0,
            "learners/default_policy/start_condition_batch/interleaved_rows_per_start": 1536.0,
            "learners/default_policy/start_condition_batch/max_start_run_length": 1.0,
            "learners/default_policy/kl_update/max_minibatch_mean": max_kl,
            "learners/default_policy/kl_update/min_minibatch_mean": min_kl,
            "learners/default_policy/kl_update/minibatch_count": kl_minibatches,
            "learners/default_policy/kl_update/nonfinite_count": kl_nonfinite,
        }
        for offset in OFFSETS:
            label = tb_logging.start_offset_metric_label(offset)
            metrics[
                f"learners/default_policy/start_condition/{label}/advantage_count"
            ] = 1536.0
        return metrics

    def test_exact_balance_accepts_1536_steps_per_start(self):
        report = train_ppo_mlp._exact_start_balance_report(
            self._args(), self._coverage(), self._learner_metrics()
        )

        self.assertTrue(report["pass"])
        self.assertTrue(report["learner_batch_pass"])
        self.assertEqual(report["rollout_fragment_length"], 384)
        self.assertEqual(report["expected_kl_minibatches"], 9)
        self.assertEqual(report["interleaved_rows"], 4608.0)
        self.assertEqual(report["max_start_run_length"], 1.0)

    def test_exact_balance_rejects_one_missing_step(self):
        metrics = self._coverage()
        label = tb_logging.start_offset_metric_label(OFFSETS[-1])
        metrics[f"env_runners/episode_start_steps_current/{label}"] = 1535.0

        report = train_ppo_mlp._exact_start_balance_report(
            self._args(), metrics, self._learner_metrics()
        )

        self.assertFalse(report["pass"])
        self.assertIn(label, report["mismatched"])

    def test_exact_balance_rejects_post_gae_cyclic_reuse(self):
        report = train_ppo_mlp._exact_start_balance_report(
            self._args(),
            self._coverage(),
            self._learner_metrics(
                removed_rows=0.0,
                compacted_rows=4622.0,
                trained=5120.0,
            ),
        )

        self.assertFalse(report["pass"])
        self.assertFalse(report["learner_batch_pass"])
        self.assertFalse(report["learner_checks"]["post_gae_compaction"])
        self.assertFalse(report["learner_checks"]["module_steps_trained"])

    def test_exact_balance_requires_nine_finite_kl_minibatches(self):
        wrong_count = train_ppo_mlp._exact_start_balance_report(
            self._args(),
            self._coverage(),
            self._learner_metrics(kl_minibatches=8.0),
        )
        self.assertFalse(wrong_count["pass"])
        self.assertFalse(wrong_count["learner_checks"]["kl_minibatch_count"])

        nonfinite = train_ppo_mlp._exact_start_balance_report(
            self._args(),
            self._coverage(),
            self._learner_metrics(
                max_kl=float("inf"),
                min_kl=float("-inf"),
                kl_nonfinite=1.0,
            ),
        )
        self.assertFalse(nonfinite["pass"])
        self.assertFalse(nonfinite["learner_checks"]["kl_values_finite"])

    def test_exact_balance_rejects_missing_or_clustered_interleaving(self):
        missing = self._learner_metrics()
        del missing[
            "learners/default_policy/start_condition_batch/interleaved_rows"
        ]
        report = train_ppo_mlp._exact_start_balance_report(
            self._args(), self._coverage(), missing
        )
        self.assertFalse(report["pass"])
        self.assertFalse(report["learner_checks"]["start_interleaving"])

        clustered = self._learner_metrics()
        clustered[
            "learners/default_policy/start_condition_batch/max_start_run_length"
        ] = 1536.0
        report = train_ppo_mlp._exact_start_balance_report(
            self._args(), self._coverage(), clustered
        )
        self.assertFalse(report["pass"])
        self.assertFalse(report["learner_checks"]["start_interleaving"])

    def test_exact_mode_preflight_requires_exactly_three_starts(self):
        args = argparse.Namespace(
            exact_start_sampling=True,
            episode_start_offset_choices_s=OFFSETS[:2],
            random_init=False,
            num_env_runners=12,
            train_batch_size=4608,
            minibatch_size=512,
            num_epochs=1,
        )
        with self.assertRaisesRegex(SystemExit, "exactly three"):
            train_ppo_mlp._validate_start_sampling_args(args)

    def test_exact_mode_preflight_requires_one_epoch(self):
        args = argparse.Namespace(
            exact_start_sampling=True,
            episode_start_offset_choices_s=list(OFFSETS),
            random_init=False,
            num_env_runners=12,
            train_batch_size=4608,
            minibatch_size=512,
            num_epochs=2,
        )
        with self.assertRaisesRegex(SystemExit, "requires --num-epochs 1"):
            train_ppo_mlp._validate_start_sampling_args(args)

    def test_runtime_audit_rejects_bypassed_multi_epoch_contract(self):
        args = self._args()
        args.num_epochs = 2
        report = train_ppo_mlp._exact_start_balance_report(
            args,
            self._coverage(),
            self._learner_metrics(trained=9216.0, kl_minibatches=18.0),
        )

        self.assertFalse(report["pass"])
        self.assertFalse(report["learner_checks"]["single_epoch_contract"])

    def test_kl_update_metrics_expose_jsonl_field_names(self):
        metrics = train_ppo_mlp._kl_update_metrics(self._learner_metrics())
        self.assertEqual(
            metrics,
            {
                "max_minibatch_mean_kl_loss": 0.008,
                "min_minibatch_mean_kl_loss": 0.001,
                "kl_minibatch_count": 9.0,
                "kl_nonfinite_count": 0.0,
            },
        )

    def test_kl_update_guard_passes_complete_safe_audit(self):
        report = train_ppo_mlp._enforce_kl_update_guard(
            self._learner_metrics(max_kl=0.01, min_kl=-1.0e-7),
            max_minibatch_mean_kl_loss=0.01,
            logical_iteration=2,
        )

        self.assertTrue(report["pass"])
        self.assertEqual(report["logical_iteration"], 2)
        self.assertEqual(report["failed_checks"], [])

    def test_kl_update_guard_rejects_limit_exceedance(self):
        with self.assertRaisesRegex(RuntimeError, "max_within_limit"):
            train_ppo_mlp._enforce_kl_update_guard(
                self._learner_metrics(max_kl=0.0100001),
                max_minibatch_mean_kl_loss=0.01,
                logical_iteration=3,
            )

    def test_kl_update_guard_rejects_missing_or_nonfinite_audit(self):
        missing_cases = (
            "learners/default_policy/kl_update/max_minibatch_mean",
            "learners/default_policy/kl_update/min_minibatch_mean",
            "learners/default_policy/kl_update/nonfinite_count",
        )
        for missing_key in missing_cases:
            with self.subTest(missing=missing_key):
                metrics = self._learner_metrics()
                del metrics[missing_key]
                with self.assertRaisesRegex(RuntimeError, "present_and_finite"):
                    train_ppo_mlp._enforce_kl_update_guard(
                        metrics,
                        max_minibatch_mean_kl_loss=0.01,
                    )

        nonfinite_cases = (
            {"max_kl": float("nan")},
            {"min_kl": float("-inf")},
            {"kl_nonfinite": float("nan")},
            {"kl_nonfinite": 1.0},
        )
        for overrides in nonfinite_cases:
            with self.subTest(nonfinite=overrides):
                with self.assertRaisesRegex(RuntimeError, "hard KL update guard"):
                    train_ppo_mlp._enforce_kl_update_guard(
                        self._learner_metrics(**overrides),
                        max_minibatch_mean_kl_loss=0.01,
                    )

    def test_kl_update_guard_cli_and_config_are_opt_in(self):
        with tempfile.TemporaryDirectory() as tmp:
            config = Path(tmp) / "config.yaml"
            base_argv = [
                "train_ppo_mlp.py",
                "--config",
                str(config),
                "--output-dir",
                "unused",
            ]
            config.write_text("{}\n", encoding="utf-8")
            with mock.patch.object(sys, "argv", base_argv):
                default_args = train_ppo_mlp.parse_args()

            config.write_text(
                "supervision:\n  max_minibatch_mean_kl_loss: 0.01\n",
                encoding="utf-8",
            )
            with mock.patch.object(sys, "argv", base_argv):
                configured_args = train_ppo_mlp.parse_args()
            with mock.patch.object(
                sys,
                "argv",
                [*base_argv, "--max-minibatch-mean-kl-loss", "0.02"],
            ):
                overridden_args = train_ppo_mlp.parse_args()

        self.assertIsNone(default_args.max_minibatch_mean_kl_loss)
        self.assertEqual(configured_args.max_minibatch_mean_kl_loss, 0.01)
        self.assertEqual(overridden_args.max_minibatch_mean_kl_loss, 0.02)

    def test_per_start_metrics_derive_returns_and_advantages(self):
        label = tb_logging.start_offset_metric_label(OFFSETS[0])
        env_metrics = self._coverage()
        env_metrics.update(
            {
                f"env_runners/episode_start_return_sum/{label}": 12.0,
                f"env_runners/episode_start_length_sum/{label}": 400.0,
                f"env_runners/episode_start_episode_count/{label}": 2.0,
            }
        )
        learner_metrics = {
            f"learners/default_policy/start_condition/{label}/advantage_sum": 4.0,
            f"learners/default_policy/start_condition/{label}/advantage_sumsq": 10.0,
            f"learners/default_policy/start_condition/{label}/advantage_positive_count": 2.0,
            f"learners/default_policy/start_condition/{label}/advantage_count": 4.0,
        }

        result = train_ppo_mlp._per_start_training_metrics(
            self._args(), env_metrics, learner_metrics
        )[label]

        self.assertEqual(result["episode_return_mean"], 6.0)
        self.assertEqual(result["episode_length_mean"], 200.0)
        self.assertEqual(result["advantage_mean"], 1.0)
        self.assertEqual(result["advantage_positive_fraction"], 0.5)


if __name__ == "__main__":
    unittest.main()
