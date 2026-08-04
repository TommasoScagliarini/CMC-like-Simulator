"""Focused tests for per-start-condition PPO advantage metrics."""

from __future__ import annotations

import math
import pickle
import sys
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np
import torch
from ray.rllib.algorithms.ppo.ppo import LEARNER_RESULTS_KL_KEY
from ray.rllib.algorithms.ppo.torch.ppo_torch_learner import PPOTorchLearner
from ray.rllib.core import DEFAULT_MODULE_ID
from ray.rllib.core.columns import Columns
from ray.rllib.env.single_agent_episode import SingleAgentEpisode
from ray.rllib.evaluation.postprocessing import Postprocessing
from ray.rllib.policy.sample_batch import MultiAgentBatch, SampleBatch
from ray.rllib.utils.minibatch_utils import MiniBatchCyclicIterator
from ray.rllib.utils.metrics.metrics_logger import MetricsLogger


BASELINE_DIR = (
    Path(__file__).resolve().parents[1] / "Trajectory Generator" / "baseline_MLP"
)
if str(BASELINE_DIR) not in sys.path:
    sys.path.insert(0, str(BASELINE_DIR))

import start_condition_metrics as metrics_module


class _RecordingMetricsLogger:
    def __init__(self) -> None:
        self.calls: list[tuple[tuple[str, ...], float | int, dict]] = []

    def log_value(self, key, value, **kwargs) -> None:
        self.calls.append((tuple(key), value, dict(kwargs)))


class StartConditionLearnerKlMetricTests(unittest.TestCase):
    @staticmethod
    def _new_unbuilt_learner():
        learner = object.__new__(
            metrics_module.StartConditionMetricsPPOTorchLearner
        )
        learner.metrics = MetricsLogger()
        return learner

    @staticmethod
    def _run_minibatches(learner, values):
        pending = iter(values)

        def fake_parent_update(instance, batch):
            del batch
            kl = torch.tensor(next(pending), dtype=torch.float32)
            instance.metrics.log_value(
                (DEFAULT_MODULE_ID, LEARNER_RESULTS_KL_KEY),
                kl,
                window=1,
            )
            return {}, {DEFAULT_MODULE_ID: torch.tensor(0.0)}, {}

        with patch.object(PPOTorchLearner, "_update", fake_parent_update):
            for _ in values:
                learner._update({})
        return MetricsLogger.peek_results(learner.metrics.reduce())

    def test_update_kl_range_covers_nine_minibatches_and_resets(self) -> None:
        learner = self._new_unbuilt_learner()
        first_values = [
            0.001,
            0.009,
            -0.002,
            0.013,
            0.004,
            0.005,
            0.006,
            0.007,
            0.008,
        ]
        first = self._run_minibatches(learner, first_values)[DEFAULT_MODULE_ID]
        update = first[metrics_module.KL_UPDATE_METRICS_PREFIX]

        # RLlib's legacy metric remains the final minibatch value.  The custom
        # range retains the maximum in the middle and exposes the negative KL.
        self.assertAlmostEqual(first[LEARNER_RESULTS_KL_KEY], 0.008)
        self.assertAlmostEqual(
            update[metrics_module.KL_UPDATE_MAX_MINIBATCH_MEAN_KEY], 0.013
        )
        self.assertAlmostEqual(
            update[metrics_module.KL_UPDATE_MIN_MINIBATCH_MEAN_KEY], -0.002
        )
        self.assertEqual(
            update[metrics_module.KL_UPDATE_MINIBATCH_COUNT_KEY], 9
        )
        self.assertEqual(
            update[metrics_module.KL_UPDATE_NONFINITE_COUNT_KEY], 0
        )

        second_values = [value / 100.0 for value in range(1, 10)]
        second = self._run_minibatches(learner, second_values)[DEFAULT_MODULE_ID]
        second_update = second[metrics_module.KL_UPDATE_METRICS_PREFIX]
        self.assertAlmostEqual(
            second_update[metrics_module.KL_UPDATE_MAX_MINIBATCH_MEAN_KEY],
            0.09,
        )
        self.assertAlmostEqual(
            second_update[metrics_module.KL_UPDATE_MIN_MINIBATCH_MEAN_KEY],
            0.01,
        )
        self.assertEqual(
            second_update[metrics_module.KL_UPDATE_MINIBATCH_COUNT_KEY], 9
        )

    def test_nonfinite_kl_cannot_be_hidden_by_max_or_min_reduction(self) -> None:
        learner = self._new_unbuilt_learner()
        result = self._run_minibatches(
            learner,
            [0.001, float("nan"), 0.003, 0.004, 0.005, 0.006, 0.007, 0.008, 0.009],
        )[DEFAULT_MODULE_ID]
        update = result[metrics_module.KL_UPDATE_METRICS_PREFIX]

        max_kl = update[metrics_module.KL_UPDATE_MAX_MINIBATCH_MEAN_KEY]
        min_kl = update[metrics_module.KL_UPDATE_MIN_MINIBATCH_MEAN_KEY]
        self.assertTrue(math.isinf(max_kl) and max_kl > 0.0)
        self.assertTrue(math.isinf(min_kl) and min_kl < 0.0)
        self.assertEqual(
            update[metrics_module.KL_UPDATE_MINIBATCH_COUNT_KEY], 9
        )
        self.assertEqual(
            update[metrics_module.KL_UPDATE_NONFINITE_COUNT_KEY], 1
        )


class StartConditionPureHelperTests(unittest.TestCase):
    def test_masked_moments_and_derived_statistics(self) -> None:
        moments = metrics_module.aggregate_advantage_moments_by_start(
            [1.75, 1.75, 1.95, 1.95, 2.15, 2.15],
            [-1.0, 1.0, 2.0, 4.0, -3.0, 100.0],
            [True, True, True, True, True, False],
        )

        self.assertEqual(
            moments["offset_1p750000s"],
            {
                "episode_start_offset_s": 1.75,
                "advantage_sum": 0.0,
                "advantage_sumsq": 2.0,
                "advantage_positive_count": 1,
                "advantage_count": 2,
            },
        )
        self.assertEqual(moments["offset_2p150000s"]["advantage_count"], 1)
        self.assertEqual(moments["offset_2p150000s"]["advantage_sum"], -3.0)

        statistics = metrics_module.derive_advantage_statistics_by_start(moments)
        self.assertEqual(statistics["offset_1p750000s"]["advantage_mean"], 0.0)
        self.assertEqual(statistics["offset_1p750000s"]["advantage_std"], 1.0)
        self.assertEqual(
            statistics["offset_1p750000s"]["advantage_positive_fraction"],
            0.5,
        )
        self.assertEqual(statistics["offset_1p950000s"]["advantage_mean"], 3.0)
        self.assertEqual(statistics["offset_1p950000s"]["advantage_std"], 1.0)
        self.assertEqual(
            statistics["offset_2p150000s"]["advantage_positive_fraction"],
            0.0,
        )

        # Learner result trees contain the label and additive moments; they do
        # not need to repeat the numeric offset as another reducible metric.
        logged_moments = {
            label: {
                key: value
                for key, value in values.items()
                if key != "episode_start_offset_s"
            }
            for label, values in moments.items()
        }
        logged_statistics = metrics_module.derive_advantage_statistics_by_start(
            logged_moments
        )
        self.assertEqual(
            logged_statistics["offset_1p950000s"]["episode_start_offset_s"],
            1.95,
        )

    def test_start_offset_label_round_trip_including_negative_value(self) -> None:
        label = metrics_module.format_start_offset_label(-0.2)
        self.assertEqual(label, "offset_m0p200000s")
        self.assertEqual(metrics_module.parse_start_offset_label(label), -0.2)

    def test_masked_nonfinite_bootstrap_value_is_ignored(self) -> None:
        moments = metrics_module.aggregate_advantage_moments_by_start(
            [1.95, float("nan")],
            [2.0, float("nan")],
            [True, False],
        )
        self.assertEqual(moments["offset_1p950000s"]["advantage_sum"], 2.0)

    def test_active_nonfinite_advantage_is_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "advantages must all be finite"):
            metrics_module.aggregate_advantage_moments_by_start(
                [1.95],
                [float("nan")],
            )

    def test_info_offset_must_be_present_and_consistent(self) -> None:
        self.assertEqual(
            metrics_module.episode_start_offset_from_infos(
                [
                    {"episode_start_offset_s": 1.95},
                    {"episode_start_offset_s": 1.95},
                ]
            ),
            1.95,
        )
        with self.assertRaisesRegex(ValueError, "inconsistent start offsets"):
            metrics_module.episode_start_offset_from_infos(
                [
                    {"episode_start_offset_s": 1.95},
                    {"episode_start_offset_s": 2.15},
                ]
            )

    def test_standardize_real_advantages_uses_population_moments(self) -> None:
        normalized, mean, std = metrics_module.standardize_real_advantages(
            np.asarray([1.0, 2.0, 3.0], dtype=np.float32),
            expected_rows=3,
        )

        self.assertEqual(mean, 2.0)
        self.assertAlmostEqual(std, np.sqrt(2.0 / 3.0))
        self.assertEqual(normalized.dtype, np.float32)
        self.assertAlmostEqual(float(np.mean(normalized)), 0.0, places=6)
        self.assertAlmostEqual(float(np.std(normalized)), 1.0, places=6)

    def test_compaction_is_transactional_on_nested_shape_mismatch(self) -> None:
        original_advantages = torch.tensor([1.0, 2.0, 100.0])
        batch = {
            metrics_module.EPISODE_START_OFFSET_COLUMN: np.asarray(
                [1.75, 1.75, 1.75]
            ),
            Postprocessing.ADVANTAGES: original_advantages,
            Columns.LOSS_MASK: torch.tensor([True, True, False]),
            Columns.OBS: {
                "actor": np.zeros((3, 2), dtype=np.float32),
                "critic": torch.zeros((2, 4)),
            },
        }

        with self.assertRaisesRegex(ValueError, r"obs\.critic has 2 rows"):
            metrics_module.compact_and_normalize_module_batch(batch)

        # A late malformed leaf must not leave earlier columns half-compacted.
        self.assertIs(batch[Postprocessing.ADVANTAGES], original_advantages)
        self.assertEqual(batch[Postprocessing.ADVANTAGES].shape[0], 3)

    def test_compaction_rejects_non_boolean_numeric_mask(self) -> None:
        batch = {
            metrics_module.EPISODE_START_OFFSET_COLUMN: np.asarray([1.75, 1.75]),
            Postprocessing.ADVANTAGES: np.asarray([1.0, 2.0]),
            Columns.LOSS_MASK: np.asarray([1, 2]),
        }
        with self.assertRaisesRegex(ValueError, "boolean, zero, or one"):
            metrics_module.compact_and_normalize_module_batch(batch)


class StartConditionConnectorTests(unittest.TestCase):
    def test_episode_connector_adds_numeric_offset_for_each_timestep(self) -> None:
        episode = SingleAgentEpisode(
            id_="episode-a",
            observations=[0.0, 1.0, 2.0, 3.0],
            infos=[
                {"episode_start_offset_s": 1.95},
                {"episode_start_offset_s": 1.95},
                {"episode_start_offset_s": 1.95},
                {"episode_start_offset_s": 1.95},
            ],
            actions=[0.0, 0.0, 0.0],
            rewards=[0.0, 0.0, 0.0],
            len_lookback_buffer=0,
        )
        connector = metrics_module.build_start_condition_learner_connector(None, None)
        batch = connector(
            rl_module=None,
            batch={},
            episodes=[episode],
            shared_data={},
        )

        stored = batch[metrics_module.EPISODE_START_OFFSET_COLUMN][("episode-a",)]
        self.assertEqual(len(stored), 1)
        np.testing.assert_array_equal(stored[0], np.full(3, 1.95))
        self.assertTrue(np.issubdtype(stored[0].dtype, np.number))

    def test_post_gae_connector_compacts_nested_batch_normalizes_and_logs(self) -> None:
        logger = _RecordingMetricsLogger()
        batch = {
            "default_policy": {
                metrics_module.EPISODE_START_OFFSET_COLUMN: np.asarray(
                    [1.75, 1.75, 1.95, 1.95, 2.15, 2.15, 9.99]
                ),
                Postprocessing.ADVANTAGES: torch.tensor(
                    [-1.0, 1.0, -1.0, 1.0, -1.0, 1.0, 100.0]
                ),
                Postprocessing.VALUE_TARGETS: np.asarray(
                    [10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 999.0]
                ),
                Columns.OBS: {
                    "actor": np.arange(14, dtype=np.float32).reshape(7, 2),
                    "critic": torch.arange(21, dtype=torch.float32).reshape(7, 3),
                },
                Columns.INFOS: [
                    {"row": 0},
                    {"row": 1},
                    {"row": 2},
                    {"row": 3},
                    {"row": 4},
                    {"row": 5},
                    {"artificial": True},
                ],
                Columns.LOSS_MASK: torch.tensor(
                    [True, True, True, True, True, True, False]
                ),
                "scalar_metadata": "preserved",
            }
        }
        original = batch
        returned = metrics_module.LogStartConditionAdvantageMoments()(
            rl_module=None,
            batch=batch,
            episodes=[],
            metrics=logger,
        )

        self.assertIs(returned, original)
        module_batch = batch["default_policy"]
        self.assertEqual(module_batch[Postprocessing.ADVANTAGES].shape[0], 6)
        self.assertEqual(module_batch[Columns.OBS]["actor"].shape, (6, 2))
        self.assertEqual(module_batch[Columns.OBS]["critic"].shape, (6, 3))
        self.assertEqual(
            module_batch[Columns.INFOS],
            [{"row": i} for i in (0, 2, 4, 1, 3, 5)],
        )
        self.assertEqual(module_batch["scalar_metadata"], "preserved")
        self.assertTrue(torch.all(module_batch[Columns.LOSS_MASK]))
        self.assertEqual(
            module_batch[Postprocessing.VALUE_TARGETS].tolist(),
            [10.0, 12.0, 14.0, 11.0, 13.0, 15.0],
        )
        torch.testing.assert_close(
            module_batch[Postprocessing.ADVANTAGES],
            torch.tensor([-1.0, -1.0, -1.0, 1.0, 1.0, 1.0]),
        )
        np.testing.assert_array_equal(
            module_batch[metrics_module.EPISODE_START_OFFSET_COLUMN],
            [1.75, 1.95, 2.15, 1.75, 1.95, 2.15],
        )

        recorded = {key: (value, kwargs) for key, value, kwargs in logger.calls}
        prefix = ("default_policy", "start_condition")
        self.assertEqual(
            recorded[prefix + ("offset_1p750000s", "advantage_sum")][0],
            0.0,
        )
        self.assertEqual(
            recorded[prefix + ("offset_1p750000s", "advantage_sumsq")][0],
            2.0,
        )
        self.assertEqual(
            recorded[prefix + ("offset_2p150000s", "advantage_count")][0],
            2,
        )
        batch_prefix = ("default_policy", "start_condition_batch")
        self.assertEqual(recorded[batch_prefix + ("pre_rows",)][0], 7)
        self.assertEqual(recorded[batch_prefix + ("removed_rows",)][0], 1)
        self.assertEqual(recorded[batch_prefix + ("compacted_rows",)][0], 6)
        self.assertEqual(recorded[batch_prefix + ("interleaved_rows",)][0], 6)
        self.assertEqual(
            recorded[batch_prefix + ("interleaved_start_conditions",)][0], 3
        )
        self.assertEqual(
            recorded[batch_prefix + ("interleaved_rows_per_start",)][0], 2
        )
        self.assertEqual(recorded[batch_prefix + ("max_start_run_length",)][0], 1)
        for key, (_, kwargs) in recorded.items():
            expected_reduce = (
                "max" if key == batch_prefix + ("max_start_run_length",) else "sum"
            )
            self.assertEqual(kwargs, {"reduce": expected_reduce})

    def test_post_gae_connector_fails_without_offset_column(self) -> None:
        with self.assertRaisesRegex(KeyError, "missing start-offset column"):
            metrics_module.LogStartConditionAdvantageMoments()(
                rl_module=None,
                batch={
                    "default_policy": {
                        Postprocessing.ADVANTAGES: np.asarray([1.0]),
                        Columns.LOSS_MASK: np.asarray([True]),
                    }
                },
                episodes=[],
                metrics=_RecordingMetricsLogger(),
            )

    def test_post_gae_connector_numpy_standardization_excludes_fake_row(self) -> None:
        batch = {
            "default_policy": {
                metrics_module.EPISODE_START_OFFSET_COLUMN: np.asarray(
                    [1.95, 1.95, 1.95, 1.95]
                ),
                Postprocessing.ADVANTAGES: np.asarray(
                    [1.0, 2.0, 3.0, -1000.0], dtype=np.float64
                ),
                Columns.LOSS_MASK: np.asarray([1, 1, 1, 0], dtype=np.int8),
            }
        }

        metrics_module.compact_and_normalize_module_batch(
            batch["default_policy"],
            interleave_starts=False,
        )

        advantages = batch["default_policy"][Postprocessing.ADVANTAGES]
        self.assertEqual(advantages.shape, (3,))
        self.assertAlmostEqual(float(np.mean(advantages)), 0.0, places=12)
        self.assertAlmostEqual(float(np.std(advantages)), 1.0, places=12)
        self.assertEqual(
            batch["default_policy"][Columns.LOSS_MASK].dtype,
            np.dtype(bool),
        )

    def test_post_gae_connector_returns_exactly_4608_real_rows(self) -> None:
        input_rows = 4622
        real_rows = 4608
        mask = np.ones(input_rows, dtype=bool)
        artificial_indices = np.linspace(
            7,
            input_rows - 8,
            input_rows - real_rows,
            dtype=int,
        )
        mask[artificial_indices] = False

        offsets = np.full(input_rows, np.nan, dtype=np.float64)
        offsets[mask] = np.repeat([1.75, 1.95, 2.15], 1536)
        advantages = torch.full((input_rows,), float("nan"))
        advantages[torch.as_tensor(mask)] = torch.linspace(-3.0, 3.0, real_rows)
        logger = _RecordingMetricsLogger()
        batch = {
            "default_policy": {
                metrics_module.EPISODE_START_OFFSET_COLUMN: offsets,
                Postprocessing.ADVANTAGES: advantages,
                Postprocessing.VALUE_TARGETS: torch.arange(
                    input_rows, dtype=torch.float32
                ),
                Columns.OBS: np.zeros((input_rows, 35), dtype=np.float32),
                Columns.LOSS_MASK: torch.as_tensor(mask),
            }
        }

        metrics_module.LogStartConditionAdvantageMoments()(
            rl_module=None,
            batch=batch,
            episodes=[],
            metrics=logger,
        )

        module_batch = batch["default_policy"]
        for column in (
            metrics_module.EPISODE_START_OFFSET_COLUMN,
            Postprocessing.ADVANTAGES,
            Postprocessing.VALUE_TARGETS,
            Columns.OBS,
            Columns.LOSS_MASK,
        ):
            self.assertEqual(module_batch[column].shape[0], real_rows)

        recorded = {key: value for key, value, _ in logger.calls}
        batch_prefix = ("default_policy", "start_condition_batch")
        self.assertEqual(recorded[batch_prefix + ("pre_rows",)], 4622)
        self.assertEqual(recorded[batch_prefix + ("removed_rows",)], 14)
        self.assertEqual(recorded[batch_prefix + ("compacted_rows",)], 4608)
        self.assertEqual(recorded[batch_prefix + ("interleaved_rows",)], 4608)
        self.assertEqual(
            recorded[batch_prefix + ("interleaved_start_conditions",)], 3
        )
        self.assertEqual(
            recorded[batch_prefix + ("interleaved_rows_per_start",)], 1536
        )
        self.assertEqual(recorded[batch_prefix + ("max_start_run_length",)], 1)
        for label in (
            "offset_1p750000s",
            "offset_1p950000s",
            "offset_2p150000s",
        ):
            self.assertEqual(
                recorded[
                    (
                        "default_policy",
                        "start_condition",
                        label,
                        "advantage_count",
                    )
                ],
                1536,
            )

    def test_compacted_batch_yields_nine_minibatches_without_cyclic_reuse(self) -> None:
        real_rows = 4608
        input_rows = 4622
        mask = np.concatenate(
            [
                np.ones(real_rows, dtype=bool),
                np.zeros(input_rows - real_rows, dtype=bool),
            ]
        )
        module_batch = {
            metrics_module.EPISODE_START_OFFSET_COLUMN: np.concatenate(
                [
                    np.repeat([1.75, 1.95, 2.15], 1536),
                    np.full(input_rows - real_rows, 99.0),
                ]
            ),
            Postprocessing.ADVANTAGES: np.linspace(
                -2.0, 2.0, input_rows, dtype=np.float32
            ),
            Columns.LOSS_MASK: mask,
            "row_id": np.concatenate(
                [
                    np.arange(real_rows, dtype=np.int64),
                    np.full(input_rows - real_rows, -1, dtype=np.int64),
                ]
            ),
        }
        report = metrics_module.compact_and_normalize_module_batch(
            module_batch,
            interleave_starts=True,
        )
        self.assertEqual(report.real_rows, real_rows)
        self.assertEqual(report.interleaved_rows, real_rows)
        self.assertEqual(report.interleaved_start_conditions, 3)
        self.assertEqual(report.interleaved_rows_per_start, 1536)
        self.assertEqual(report.max_start_run_length, 1)

        compacted = MultiAgentBatch(
            {"default_policy": SampleBatch(module_batch)},
            env_steps=real_rows,
        )
        compacted_minibatches = list(
            MiniBatchCyclicIterator(
                compacted,
                num_epochs=1,
                minibatch_size=512,
                shuffle_batch_per_epoch=False,
            )
        )
        compacted_ids = np.concatenate(
            [
                minibatch.policy_batches["default_policy"]["row_id"]
                for minibatch in compacted_minibatches
            ]
        )
        self.assertEqual(len(compacted_minibatches), 9)
        self.assertTrue(
            all(
                len(minibatch.policy_batches["default_policy"]) == 512
                for minibatch in compacted_minibatches
            )
        )
        self.assertEqual(compacted_ids.size, real_rows)
        self.assertEqual(np.unique(compacted_ids).size, real_rows)
        np.testing.assert_array_equal(np.sort(compacted_ids), np.arange(real_rows))

        minibatch_start_counts = np.asarray(
            [
                [
                    np.count_nonzero(
                        np.isclose(
                            minibatch.policy_batches["default_policy"][
                                metrics_module.EPISODE_START_OFFSET_COLUMN
                            ],
                            offset,
                            rtol=0.0,
                            atol=1e-12,
                        )
                    )
                    for offset in (1.75, 1.95, 2.15)
                ]
                for minibatch in compacted_minibatches
            ],
            dtype=np.int64,
        )
        self.assertEqual(minibatch_start_counts.shape, (9, 3))
        self.assertTrue(np.all((minibatch_start_counts == 170) | (minibatch_start_counts == 171)))
        np.testing.assert_array_equal(
            np.sum(minibatch_start_counts, axis=0),
            [1536, 1536, 1536],
        )

        interleaved_offsets = module_batch[
            metrics_module.EPISODE_START_OFFSET_COLUMN
        ]
        self.assertTrue(np.all(interleaved_offsets[1:] != interleaved_offsets[:-1]))

        # This is the Ray cyclic-iterator failure mode the compaction prevents:
        # 4622 is not divisible by 512, so one epoch emits 10 full minibatches and
        # silently reuses the first 498 rows to complete the last one.
        raw = MultiAgentBatch(
            {
                "default_policy": SampleBatch(
                    {"row_id": np.arange(input_rows, dtype=np.int64)}
                )
            },
            env_steps=input_rows,
        )
        raw_minibatches = list(
            MiniBatchCyclicIterator(
                raw,
                num_epochs=1,
                minibatch_size=512,
                shuffle_batch_per_epoch=False,
            )
        )
        raw_ids = np.concatenate(
            [
                minibatch.policy_batches["default_policy"]["row_id"]
                for minibatch in raw_minibatches
            ]
        )
        self.assertEqual(len(raw_minibatches), 10)
        self.assertEqual(raw_ids.size, 5120)
        self.assertEqual(np.unique(raw_ids).size, input_rows)
        self.assertEqual(raw_ids.size - np.unique(raw_ids).size, 498)

    def test_interleaving_rejects_unequal_groups_without_mutating_batch(self) -> None:
        offsets = np.asarray([1.75, 1.75, 1.95, 1.95, 2.15], dtype=np.float64)
        row_ids = torch.arange(5, dtype=torch.int64)
        actor_rows = row_ids.reshape(5, 1)
        batch = {
            metrics_module.EPISODE_START_OFFSET_COLUMN: offsets,
            Postprocessing.ADVANTAGES: np.arange(5, dtype=np.float32),
            Columns.LOSS_MASK: np.ones(5, dtype=bool),
            Columns.OBS: {"actor": actor_rows},
        }

        with self.assertRaisesRegex(ValueError, "equal rows"):
            metrics_module.compact_and_normalize_module_batch(
                batch,
                interleave_starts=True,
            )

        # Compaction, normalization, and interleaving are one transaction in exact
        # mode: even the earlier valid transformations must not leak on failure.
        self.assertIs(batch[metrics_module.EPISODE_START_OFFSET_COLUMN], offsets)
        self.assertIs(batch[Columns.OBS]["actor"], actor_rows)
        np.testing.assert_array_equal(
            batch[Postprocessing.ADVANTAGES],
            np.arange(5, dtype=np.float32),
        )

    def test_public_builder_and_classes_are_pickleable_by_module_path(self) -> None:
        builder = pickle.loads(
            pickle.dumps(metrics_module.build_start_condition_learner_connector)
        )
        learner_class = pickle.loads(
            pickle.dumps(metrics_module.StartConditionMetricsPPOTorchLearner)
        )
        connector = pickle.loads(
            pickle.dumps(metrics_module.build_start_condition_learner_connector())
        )
        post_gae_connector = pickle.loads(
            pickle.dumps(metrics_module.LogStartConditionAdvantageMoments())
        )

        self.assertIs(builder, metrics_module.build_start_condition_learner_connector)
        self.assertIs(
            learner_class,
            metrics_module.StartConditionMetricsPPOTorchLearner,
        )
        self.assertIsInstance(
            connector,
            metrics_module.AddEpisodeStartOffsetToTrainBatch,
        )
        self.assertIsInstance(
            post_gae_connector,
            metrics_module.LogStartConditionAdvantageMoments,
        )


if __name__ == "__main__":
    unittest.main(verbosity=2)
