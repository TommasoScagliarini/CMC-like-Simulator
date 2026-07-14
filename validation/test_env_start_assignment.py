"""Unit tests for deterministic multi-start assignment across RLlib workers."""

import sys
import unittest
from pathlib import Path


_BASELINE = (
    Path(__file__).resolve().parents[1]
    / "Trajectory Generator"
    / "baseline_MLP"
)
sys.path.insert(0, str(_BASELINE))

import env_factory  # noqa: E402


class _EnvContext(dict):
    def __init__(self, worker_index, vector_index=0):
        super().__init__(
            episode_start_offset_s=1.95,
            episode_start_offset_choices_s=[1.75, 1.95, 2.15],
        )
        self.worker_index = worker_index
        self.vector_index = vector_index


class _BaseEnv:
    def __init__(self):
        self.env_cfg = env_factory.CMCEnvConfig(episode_start_offset_s=1.75)


class _WrappedEnv:
    def __init__(self, base_env):
        self.unwrapped = base_env


class _VectorEnv:
    def __init__(self, envs):
        self.envs = envs


class MultiStartAssignmentTests(unittest.TestCase):
    def test_remote_workers_are_assigned_round_robin(self):
        actual = [
            env_factory.build_env_config(_EnvContext(index)).episode_start_offset_s
            for index in range(1, 7)
        ]
        self.assertEqual(actual, [1.75, 1.95, 2.15, 1.75, 1.95, 2.15])

    def test_fixed_offset_is_preserved_without_choices(self):
        cfg = env_factory.build_env_config({"episode_start_offset_s": 1.95})
        self.assertEqual(cfg.episode_start_offset_s, 1.95)

    def test_invalid_choice_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "finite, non-negative"):
            env_factory.build_env_config(
                {"episode_start_offset_choices_s": [1.75, float("nan")]}
            )

    def test_environment_created_hook_recovers_lost_worker_context(self):
        base_env = _BaseEnv()
        vector_env = _VectorEnv([_WrappedEnv(base_env)])
        assigned = env_factory.assign_episode_start_offset_for_runner(
            vector_env,
            _EnvContext(worker_index=3),
        )
        self.assertEqual(assigned, 2.15)
        self.assertEqual(base_env.env_cfg.episode_start_offset_s, 2.15)


if __name__ == "__main__":
    unittest.main()
