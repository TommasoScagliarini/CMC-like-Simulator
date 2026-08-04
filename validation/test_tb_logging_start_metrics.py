"""Focused tests for per-start sampling and completed-episode metrics."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path


BASELINE = (
    Path(__file__).resolve().parents[1]
    / "Trajectory Generator"
    / "baseline_MLP"
)
sys.path.insert(0, str(BASELINE))

import tb_logging  # noqa: E402


class _MetricsLogger:
    def __init__(self):
        self.calls = []

    def log_value(self, key, value, **kwargs):
        self.calls.append((key, value, kwargs))


class _Chunk:
    def __init__(self, *, offset, rewards, info=True):
        self._offset = offset
        self._rewards = list(rewards)
        self._info = info

    def get_infos(self, index):
        if not self._info:
            return {}
        return {"episode_start_offset_s": self._offset}

    def get_return(self):
        return sum(self._rewards)

    def __len__(self):
        return len(self._rewards)


class StartMetricTests(unittest.TestCase):
    def test_step_logs_current_and_lifetime_counts(self):
        logger = _MetricsLogger()
        callback = tb_logging.RewardComponentsCallback()

        callback.on_episode_step(
            episode=_Chunk(offset=1.956870983805102, rewards=[1.0]),
            metrics_logger=logger,
        )

        by_key = {key: kwargs for key, _, kwargs in logger.calls}
        label = "offset_1p956871s"
        self.assertEqual(
            by_key[f"episode_start_steps/{label}"]["reduce"],
            "lifetime_sum",
        )
        self.assertEqual(
            by_key[f"episode_start_steps_current/{label}"]["reduce"],
            "sum",
        )

    def test_episode_end_combines_rollout_fragments(self):
        logger = _MetricsLogger()
        callback = tb_logging.RewardComponentsCallback()
        previous = _Chunk(offset=2.156870983805102, rewards=[1.0, 2.0])
        final = _Chunk(offset=2.156870983805102, rewards=[3.0])

        callback.on_episode_end(
            episode=final,
            prev_episode_chunks=[previous],
            metrics_logger=logger,
        )

        by_key = {key: (value, kwargs) for key, value, kwargs in logger.calls}
        label = "offset_2p156871s"
        self.assertEqual(by_key[f"episode_start_return_sum/{label}"][0], 6.0)
        self.assertEqual(by_key[f"episode_start_length_sum/{label}"][0], 3.0)
        self.assertEqual(by_key[f"episode_start_episode_count/{label}"][0], 1.0)
        for value, kwargs in by_key.values():
            self.assertEqual(kwargs["reduce"], "sum")


if __name__ == "__main__":
    unittest.main()
