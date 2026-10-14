"""Offline arithmetic and boundary tests; no training, Ray cluster or simulation."""
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np
import torch
from ray.rllib.core.distribution.torch.torch_distribution import TorchDiagGaussian
from ray.rllib.env.single_agent_episode import SingleAgentEpisode

spec = importlib.util.spec_from_file_location("learner_analysis", Path(__file__).with_name("learner_analysis.py"))
analysis = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = analysis
spec.loader.exec_module(analysis)


class DistributionModule:
    @staticmethod
    def get_train_action_dist_cls():
        return TorchDiagGaussian

    get_exploration_action_dist_cls = get_train_action_dist_cls


class Tests(unittest.TestCase):
    def test_raw_likelihood_mask_and_clipped_surrogate(self):
        logits = np.zeros((3, 4), dtype=np.float32)
        actions = np.array([[1.4, -.2], [-.3, .1], [100., 100.]], dtype=np.float32)
        logp = TorchDiagGaussian.from_logits(torch.from_numpy(logits)).logp(torch.from_numpy(actions)).numpy()
        batch = {"actions": actions, "action_dist_inputs": logits, "action_logp": logp,
                 "advantages": np.array([1., -1., 999.]), "value_targets": np.array([0., 0., 999.])}
        config = {"clip_param": .2, "vf_clip_param": 10., "vf_loss_coeff": 1.}
        mask = np.array([True, True, False])
        metrics, out = analysis.snapshot_metrics(DistributionModule(), logits, np.array([0., 4., -999.]),
                                                 batch, mask, config, kl_coeff=.3, entropy_coeff=0.)
        np.testing.assert_array_equal(out["ratio"], np.ones(3))
        self.assertEqual(metrics["ratio"]["count"], 2)
        self.assertEqual(metrics["policy_loss_fixed_advantages"], 0.)
        self.assertEqual(metrics["critic_fixed_gae_targets"]["cap_fraction"], .5)
        self.assertEqual(metrics["critic_fixed_gae_targets"]["capped_mse"], 5.)
        self.assertEqual(metrics["critic_fixed_gae_targets"]["squared_error"]["mean"], 8.)
        changed = logits.copy()
        changed[:, 0] = .4
        changed_metrics, changed_out = analysis.snapshot_metrics(
            DistributionModule(), changed, np.zeros(3), batch, mask, config, kl_coeff=.3, entropy_coeff=0.)
        self.assertGreater(changed_out["ratio"][0], 1.2)
        self.assertGreater(changed_metrics["surrogate_actually_clipped_fraction"], 0.)

    def test_real_episode_boundaries_terminal_vs_fragment_bootstrap(self):
        episodes = []
        for number, terminated in enumerate((True, False)):
            observations = [np.full(88, number * 3 + index, dtype=np.float32) for index in range(3)]
            episodes.append(SingleAgentEpisode(
                id_=f"episode_{number}", observations=observations,
                actions=[np.zeros(2)] * 2, rewards=[1. + 2 * number, 2. + 2 * number],
                infos=[{"time": float(index), "end_reason": "guard" if terminated and index == 2 else None} for index in range(3)],
                terminated=terminated, len_lookback_buffer=0, t_started=0))
        gamma, lam = .99, .9
        values = np.array([2., 3., 20., 4., 5., 10.])
        target1 = np.array([1 + gamma * ((1 - lam) * 3 + lam * 2), 2.])
        last2 = 4 + gamma * 10
        target2 = np.array([3 + gamma * ((1 - lam) * 5 + lam * last2), last2])
        batch = {"obs": np.concatenate([np.asarray(episode.get_observations()) for episode in episodes]),
                 "rewards": np.array([1., 2., 0., 3., 4., 0.]),
                 "loss_mask": np.array([True, True, False, True, True, False]),
                 "terminateds": np.array([False, True, True, False, False, True]),
                 "truncateds": np.array([False, False, False, False, False, True]),
                 "value_targets": np.r_[target1, 0., target2, 0.],
                 "advantages": np.zeros(6)}
        summary, rows, independent = analysis.reconstruct_boundaries(
            [episode.get_state() for episode in episodes], batch, values, gamma, lam)
        self.assertTrue(summary["episode_to_batch_order_exact"])
        self.assertTrue(summary["mask_boundaries_all_correct"])
        self.assertLess(summary["independent_real_target_max_abs_error"], 1e-12)
        self.assertEqual(rows[0]["independent_bootstrap_value_used"], 0.)
        self.assertEqual(rows[1]["independent_bootstrap_value_used"], 10.)
        self.assertEqual(rows[1]["stock_bootstrap_value_used"], 0.)
        self.assertEqual(summary["unfinished_fragments"], 1)
        self.assertEqual(summary["full_true_terminal_episodes_in_batch"], 1)
        np.testing.assert_allclose(independent[:2], target1)

    def test_finite_and_bootstrapped_returns_are_distinct(self):
        reward = np.array([1., 2., 3.])
        finite = analysis.discounted_returns(reward, .9)
        boot = analysis.discounted_returns(reward, .9, 10.)
        np.testing.assert_allclose(finite, [5.23, 4.7, 3.])
        np.testing.assert_allclose(boot - finite, [.9 ** 3 * 10, .9 ** 2 * 10, .9 * 10])

    def test_partial_manifest_refused_before_any_model_load(self):
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            (folder / "manifest.json").write_text(json.dumps({"complete": False}))
            with self.assertRaisesRegex(ValueError, "not complete"):
                analysis.run(folder, folder / "output", folder / "plots")

    def test_installed_full_pipeline_reproduces_boundary_defect(self):
        result = analysis.native_pipeline_bootstrap_reproducer()
        for name in ("unfinished_fragment", "time_truncation"):
            np.testing.assert_allclose(result[name]["stock_targets_real"], [3.079, 2.], atol=1e-6)
            np.testing.assert_allclose(result[name]["ordinary_td_lambda_targets_real"], [6.60736, 5.96])
        np.testing.assert_allclose(result["true_termination"]["stock_targets_real"], [2.782, 2.], atol=1e-6)


if __name__ == "__main__":
    unittest.main()
