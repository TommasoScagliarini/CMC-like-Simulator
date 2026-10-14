"""Synthetic checks: no OpenSim, physical rollouts, training or Ray cluster."""
from __future__ import annotations

import copy
import json
import unittest

import numpy as np

from exploration_analysis import (JOINTS, analyze_records, reconstruct_actions,
                                  records_from_episode_states)


def record(episode, step, *, completed=True, reward=1.0):
    obs = {JOINTS[0]: .2, JOINTS[1]: .4,
           JOINTS[0] + "_served_ref": .21, JOINTS[1] + "_served_ref": .41}
    return {"episode_id": episode, "episode_t": step,
        "time_before": step * .01, "reward": reward, "terminated": not completed,
        "truncated": False, "observation_before": dict(obs), "observation_after": dict(obs),
        "actions": {"raw": [.1, .2], "applied_reconstructed": [.1, .2]},
        "distribution": {"innovation_raw": [.1, .2]},
        "info": {"time": (step + int(completed)) * .01, "segment_completed": completed,
            "target_availability": {"visible": True},
            "imitation_target": {"source": "LEFT", "time_alignment": "prescribed_left",
                "q": {JOINTS[0]: .1, JOINTS[1]: .5}, "phase": {JOINTS[0]: .05, JOINTS[1]: .15}},
            "reward_terms": {}, "reward_components": {}}}


class ExplorationChecks(unittest.TestCase):
    def test_connector_reconstruction_matches_installed_rllib(self):
        from gymnasium.spaces import Box
        from ray.rllib.utils.spaces.space_utils import unsquash_action, clip_action
        rng = np.random.default_rng(4)
        for bounds in ((-1, 1), (-2, 3)):
            space = Box(*bounds, shape=(2,), dtype=np.float32)
            for dtype in (np.float32, np.float64):
                for normalize, clip in ((True, False), (False, True), (False, False)):
                    for action in rng.normal(0, 2, (20, 2)).astype(dtype):
                        result = reconstruct_actions(action, space.low, space.high,
                                                     normalize_actions=normalize, clip_actions=clip)
                        expected = unsquash_action(action, space) if normalize else (
                            clip_action(action, space) if clip else action)
                        np.testing.assert_array_equal(result["to_env_reconstructed"], expected)
                        np.testing.assert_array_equal(result["applied_reconstructed"],
                                                      np.clip(expected.astype(np.float32), space.low, space.high))

    def test_connector_can_hide_clipping_from_reward_wrapper(self):
        action = np.asarray([1.5, -2], np.float32)
        result = reconstruct_actions(action, -np.ones(2), np.ones(2),
                                     normalize_actions=True, clip_actions=False)
        self.assertTrue(result["connector_clipped"].all())
        self.assertFalse(result["wrapper_clipped"].any())
        np.testing.assert_array_equal(result["applied_reconstructed"], [1, -1])

    def test_partial_and_gaps_cannot_bridge_lags(self):
        rows = [record("a", 0), record("a", 1), record("a", 2, completed=False, reward=-10),
                record("a", 3), record("a", 5), record("b", 0), record("b", 1)]
        metrics = analyze_records(rows, max_lag=2)
        self.assertEqual(metrics["n_completed_physical_segments"], 6)
        self.assertEqual(metrics["episodes"][0]["sampled_fragment_reward_sum"], -6)
        self.assertFalse(metrics["episodes"][0]["full_horizon_eligible"])
        lags = metrics["strata"][0]["observational_lags"]
        self.assertEqual(lags[2]["innovation_raw_vs_future"]["reward"]["n"], 2)
        self.assertEqual(lags[4]["innovation_raw_vs_future"]["reward"]["n"], 0)
        self.assertAlmostEqual(metrics["strata"][0]["kinematics"][JOINTS[0]]["actual_minus_target"]["rms"], .1)
        json.dumps(metrics, allow_nan=False)

    def test_duplicate_transition_rejected(self):
        r = record("a", 0)
        with self.assertRaises(ValueError):
            analyze_records([r, copy.deepcopy(r)])

    def test_target_strata_and_full_horizon(self):
        a, b = record("a", 0), record("b", 0)
        a["info"]["end_reason"] = "episode_time_limit"
        a["truncated"] = True
        b["info"]["imitation_target"]["source"] = "RIGHT"
        metrics = analyze_records([a, b], max_lag=0, expected_episode_steps=1)
        self.assertEqual(len(metrics["strata"]), 2)
        self.assertTrue(metrics["episodes"][0]["full_horizon_eligible"])
        self.assertIsNone(metrics["episodes"][1]["full_horizon_return"])

    def test_native_boundary_alignment_and_lookback_exclusion(self):
        from gymnasium.spaces import Box
        from ray.rllib.env.single_agent_episode import SingleAgentEpisode
        names = [JOINTS[0], JOINTS[1], JOINTS[0] + "_served_ref", JOINTS[1] + "_served_ref"]
        infos = [{"time": .01*i, "segment_completed": True, "observation_feature_names": names,
                  "imitation_target": {"source": "LEFT", "q": dict(zip(JOINTS, [.2, .4]))}}
                 for i in range(4)]
        episode = SingleAgentEpisode(id_="native", observations=[np.full(4, i, np.float32) for i in range(4)],
            actions=[np.array([i, -i], np.float32) for i in range(3)], rewards=[10., 11., 12.], infos=infos,
            observation_space=Box(-10, 10, (4,), dtype=np.float32),
            action_space=Box(-1, 1, (2,), dtype=np.float32),
            extra_model_outputs={"action_dist_inputs": [np.zeros(4, np.float32) for _ in range(3)]},
            len_lookback_buffer=1, t_started=7)
        rows = records_from_episode_states([episode.get_state()], normalize_actions=True, clip_actions=False)
        self.assertEqual(len(rows), 2)
        self.assertEqual(rows[0]["episode_t"], 7)
        self.assertEqual(rows[0]["reward"], 11.)
        self.assertEqual(rows[0]["observation_before"][JOINTS[0]], 1.)
        self.assertEqual(rows[0]["observation_after"][JOINTS[0]], 2.)
        self.assertEqual(rows[0]["time_before"], .01)
        self.assertEqual(rows[0]["info"]["time"], .02)
        self.assertEqual(rows[1]["actions"]["raw"], [2., -2.])
        self.assertEqual(rows[1]["actions"]["applied_reconstructed"], [1., -1.])
        json.dumps(rows, allow_nan=False)


if __name__ == "__main__":
    unittest.main()
