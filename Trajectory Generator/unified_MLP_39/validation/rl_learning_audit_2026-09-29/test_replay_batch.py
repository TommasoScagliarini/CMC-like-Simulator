"""Arithmetic and immutability checks for the isolated replay correction."""
from pathlib import Path
import sys
import unittest

import numpy as np
import torch
from ray.rllib.env.single_agent_episode import SingleAgentEpisode

sys.path.insert(0, str(Path(__file__).resolve().parent))
from replay_batch import corrected_real_targets, compare_trees


class Tests(unittest.TestCase):
    def test_only_real_targets_change_and_masked_normalization_is_preserved(self):
        episode = SingleAgentEpisode(
            id_="open", observations=[np.full(88, value, dtype=np.float32) for value in (2., 3., 4.)],
            actions=[np.zeros(2)] * 2, rewards=[1., 2.], infos=[{}, {}, {}], len_lookback_buffer=0)
        batch = {"obs": torch.tensor(np.asarray(episode.get_observations())),
                 "rewards": torch.tensor([1., 2., 0.]), "loss_mask": torch.tensor([True, True, False]),
                 "value_targets": torch.tensor([3.079, 2., 1.234])}
        values = torch.tensor([2., 3., 4.])
        original = batch["value_targets"].clone()
        targets, advantages = corrected_real_targets(batch, values, [episode], .99, .9)
        np.testing.assert_allclose(targets[:2], [6.60736, 5.96], atol=5e-7)
        self.assertEqual(targets[2], original[2].item())
        self.assertTrue(torch.equal(batch["value_targets"], original))
        raw = targets - values.numpy()
        np.testing.assert_array_equal(advantages, (raw - raw.mean()) / raw.std())
        self.assertGreater(abs(advantages[:2].mean()), .1)

    def test_true_terminal_has_no_bootstrap_and_previous_value_is_retained(self):
        episode = SingleAgentEpisode(
            id_="terminal", observations=[np.full(88, value, dtype=np.float32) for value in (2., 3., 4.)],
            actions=[np.zeros(2)] * 2, rewards=[1., 2.], infos=[{}, {}, {}], terminated=True, len_lookback_buffer=0)
        batch = {"obs": torch.tensor(np.asarray(episode.get_observations())), "rewards": torch.tensor([1., 2., 0.]),
                 "loss_mask": torch.tensor([True, True, False]), "value_targets": torch.tensor([2.782, 2., 0.])}
        targets, _ = corrected_real_targets(batch, torch.tensor([2., 3., 4.]), [episode], .99, .9)
        np.testing.assert_allclose(targets, [3.079, 2., 0.], atol=2e-7)

    def test_exact_comparison_rejects_small_numeric_displacement(self):
        left = {"weights": torch.tensor([1., 2.]), "state": [0, np.array([3.])], "lr": .0001}
        right = {"weights": torch.tensor([1., 2.]), "state": [0, np.array([3.])], "lr": .0001}
        self.assertEqual(compare_trees(left, right), [])
        right["weights"][0] += 1e-6
        failures = compare_trees(left, right)
        self.assertEqual(len(failures), 1)
        self.assertGreater(failures[0]["max_abs_error"], 0.)


if __name__ == "__main__":
    unittest.main()
