"""The diagnostic changes sigma only and preserves paired independent draws."""
import unittest
import numpy as np
import torch
from ray.rllib.core.distribution.torch.torch_distribution import TorchDiagGaussian
from unified_MLP_39.validation.completion_noise_rollout import paired_selector


class Module:
    def forward_exploration(self, batch):
        return {"action_dist_inputs": torch.tensor([[.25, -.3, -.1, .2]])}

    forward_inference = forward_exploration

    def get_exploration_action_dist_cls(self):
        return TorchDiagGaussian


class PairedNoiseTests(unittest.TestCase):
    def test_matches_rllib_exploration_sample(self):
        module = Module()
        torch.manual_seed(125)
        expected = TorchDiagGaussian.from_logits(
            module.forward_exploration(None)["action_dist_inputs"]).sample().numpy().reshape(2)
        actual = paired_selector(torch, 1., 125, [])(module, np.zeros(39), (2,))[0]
        np.testing.assert_array_equal(actual, expected)

    def test_paired_scaling_and_global_rng_unchanged(self):
        before = torch.get_rng_state().clone()
        records = {scale: [] for scale in (0., .5, 1.)}
        selectors = {s: paired_selector(torch, s, 123, records[s]) for s in records}
        for _ in range(500):
            values = {s: f(Module(), np.zeros(39), (2,)) for s, f in selectors.items()}
            self.assertEqual(records[.5][-1]['standard_normal'], records[1.][-1]['standard_normal'])
            np.testing.assert_array_equal(values[.5][3], values[1.][3] * .5)
            np.testing.assert_array_equal(values[0.][0], values[0.][1])
        self.assertTrue(torch.equal(before, torch.get_rng_state()))


if __name__ == '__main__':
    unittest.main()
