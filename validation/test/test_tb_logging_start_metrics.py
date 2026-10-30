"""Focused tests for per-start sampling and completed-episode metrics."""

from __future__ import annotations

import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


BASELINE = (
    Path(__file__).resolve().parents[2]
    / "Trajectory Generator"
    / "baseline_MLP"
)
sys.path.insert(0, str(BASELINE))

import _bootstrap  # noqa: E402

_bootstrap.ensure_sim_paths()
import win_runtime  # noqa: E402,F401 - Apply Windows shim before Torch/Ray.
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
    def test_saved_callback_global_resolves_after_worker_setup(self):
        # A spawned worker has neither this process's imports nor its sys.path.
        # Resolve the canonical class name retained in saved Ray checkpoints.
        script = r"""
import pickle
import sys
from pathlib import Path

baseline_dir = Path(sys.argv[1]).resolve()
sys.path.insert(0, str(baseline_dir))
import train_ppo_mlp
train_ppo_mlp._worker_setup(str(baseline_dir))
assert "tb_logging" not in sys.modules
callback_class = pickle.loads(b"ctb_logging\nRewardComponentsCallback\n.")
assert callback_class.__module__ == "tb_logging"
assert Path(sys.modules["tb_logging"].__file__).resolve() == (
    baseline_dir / "utility" / "tb_logging.py"
).resolve()
callback = callback_class()
assert isinstance(pickle.loads(pickle.dumps(callback)), callback_class)
"""
        with tempfile.TemporaryDirectory(prefix="cmc_tb_logging_worker_") as workdir:
            result = subprocess.run(
                [sys.executable, "-B", "-I", "-c", script, str(BASELINE)],
                cwd=workdir,
                capture_output=True,
                text=True,
                timeout=120,
            )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

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

    def test_step_logs_unsmoothed_current_reward_sum_and_count(self):
        logger = _MetricsLogger()
        callback = tb_logging.RewardComponentsCallback()
        episode = _Chunk(offset=1.956870983805102, rewards=[1.0])
        episode.get_infos = lambda index: {
            "episode_start_offset_s": 1.956870983805102,
            "reward_components": {"reward": 0.125},
        }

        callback.on_episode_step(episode=episode, metrics_logger=logger)

        by_key = {key: (value, kwargs) for key, value, kwargs in logger.calls}
        self.assertEqual(by_key["console/reward_sum_current"][0], 0.125)
        self.assertEqual(by_key["console/reward_sum_current"][1]["reduce"], "sum")
        self.assertEqual(by_key["console/reward_count_current"][0], 1.0)
        self.assertEqual(by_key["console/reward_count_current"][1]["reduce"], "sum")

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
