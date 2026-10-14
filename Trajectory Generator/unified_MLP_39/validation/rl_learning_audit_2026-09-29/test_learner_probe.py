"""Synthetic CPU tests only: no Ray cluster, OpenSim, sampling or policy training run."""
import copy
import importlib.util
import json
from pathlib import Path
import pickle
import random
import sys
import tempfile
import types
import unittest

import numpy as np
import torch
from ray.rllib.algorithms.ppo.torch.ppo_torch_learner import PPOTorchLearner
from ray.rllib.core.distribution.torch.torch_distribution import TorchDiagGaussian
from ray.rllib.core.rl_module.apis.value_function_api import ValueFunctionAPI
from ray.rllib.connectors.learner.general_advantage_estimation import GeneralAdvantageEstimation
from ray.rllib.env.single_agent_episode import SingleAgentEpisode
from ray.rllib.utils.postprocessing.value_predictions import compute_value_targets

spec = importlib.util.spec_from_file_location("learner_probe", Path(__file__).with_name("learner_probe.py"))
probe = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = probe
spec.loader.exec_module(probe)


class Module(torch.nn.Module, ValueFunctionAPI):
    def __init__(self):
        super().__init__()
        self.actor = torch.nn.Linear(88, 2)
        self.critic = torch.nn.Linear(88, 1)
        self.logstd = torch.nn.Parameter(torch.tensor([-.4, -.7]))

    def unwrapped(self):
        return self

    def is_stateful(self):
        return False

    def get_train_action_dist_cls(self):
        return TorchDiagGaussian

    get_exploration_action_dist_cls = get_train_action_dist_cls

    def compute_values(self, batch, embeddings=None):
        return self.critic(batch["obs"]).squeeze(-1)

    def forward(self, batch):
        mean = self.actor(batch["obs"])
        return {"action_dist_inputs": torch.cat((mean, self.logstd.expand_as(mean)), dim=-1)}

    def save_to_path(self, path):
        Path(path).mkdir()
        torch.save(self.state_dict(), Path(path) / "state.pt")


class Metrics:
    def log_dict(self, *args, **kwargs):
        pass


class Config(types.SimpleNamespace):
    def get_config_for_module(self, mid):
        return self


class Scheduler:
    def get_current_value(self):
        return 0.0


class Episode:
    id_ = "synthetic"
    t = 7
    t_started = 0
    is_terminated = False
    is_truncated = True
    module_id = "default_policy"

    def __len__(self):
        return 7

    def get_state(self):
        return {"infos": [{"time": i * .01} for i in range(8)],
                "observations": np.zeros((8, 88)), "id_": self.id_}


class Learner:
    """Tiny synchronous harness using the installed, unmodified PPO loss."""
    compute_loss_for_module = PPOTorchLearner.compute_loss_for_module

    def __init__(self):
        self.module = {"default_policy": Module()}
        self.config = Config(gamma=.99, lambda_=.9, clip_param=.2, use_kl_loss=True,
                             use_critic=True, vf_clip_param=10., vf_loss_coeff=1.)
        self.curr_kl_coeffs_per_module = {"default_policy": torch.tensor(.3)}
        self.entropy_coeff_schedulers_per_module = {"default_policy": Scheduler()}
        self._named_optimizers = {"default_policy_default_optimizer":
                                  torch.optim.Adam(self.module["default_policy"].parameters(), lr=1e-4)}
        self.metrics = Metrics()

    def get_state(self):
        return {"rl_module": {k: v.state_dict() for k, v in self.module.items()},
                "optimizer": {k: v.state_dict() for k, v in self._named_optimizers.items()}}

    def _make_batch_if_necessary(self, training_data):
        batch = copy.deepcopy(training_data.batch)
        values = self.module["default_policy"].compute_values(batch).detach().numpy()
        targets = compute_value_targets(values, batch["rewards"].numpy(), batch["terminateds"].numpy(),
                                        batch["truncateds"].numpy(), .99, .9)
        adv = targets - values
        batch["advantages"] = torch.from_numpy((adv - adv.mean()) / max(1e-4, adv.std()))
        batch["value_targets"] = torch.from_numpy(targets)
        return types.SimpleNamespace(policy_batches={"default_policy": batch})

    def _update(self, batch):
        module = self.module["default_policy"]
        mb = batch["default_policy"]
        fwd = module(mb)
        loss = self.compute_loss_for_module(module_id="default_policy", config=self.config, batch=mb, fwd_out=fwd)
        optimizer = next(iter(self._named_optimizers.values()))
        optimizer.zero_grad(set_to_none=True)
        loss.backward()
        optimizer.step()
        return fwd, {"default_policy": loss}, {}

    def update(self, training_data, *, num_epochs=2, minibatch_size=4):
        batch = self._make_batch_if_necessary(training_data).policy_batches["default_policy"]
        for _ in range(num_epochs):
            for start in range(0, len(batch["actions"]), minibatch_size):
                self._update({"default_policy": {key: value[start:start + minibatch_size] for key, value in batch.items()}})
        # Demonstrate that normal scalar scheduling is observed, not suppressed.
        self.curr_kl_coeffs_per_module["default_policy"] *= .5
        return {"done": True}


def inputs(learner):
    obs = torch.linspace(-.6, .9, 8 * 88).reshape(8, 88)
    logits = learner.module["default_policy"]({"obs": obs})["action_dist_inputs"].detach()
    actions = torch.tensor([[1.2, -.4], [0., .1], [-.5, 1.7], [.2, .3],
                            [-1.3, 0.], [.2, -.3], [.7, -.1], [.7, -.1]])
    logp = TorchDiagGaussian.from_logits(logits).logp(actions)
    batch = {"obs": obs, "actions": actions, "action_dist_inputs": logits,
             "action_logp": logp, "rewards": torch.tensor([.1, .2, .3, .4, .2, -.3, .7, 0.]),
             "terminateds": torch.zeros(8, dtype=torch.bool),
             "truncateds": torch.tensor([False] * 7 + [True]),
             "loss_mask": torch.tensor([True] * 7 + [False])}
    return types.SimpleNamespace(episodes=[Episode()], batch=batch)


def assert_trees_equal(test, a, b):
    if isinstance(a, torch.Tensor):
        test.assertTrue(torch.equal(a, b))
    elif isinstance(a, dict):
        test.assertEqual(a.keys(), b.keys())
        for key in a:
            assert_trees_equal(test, a[key], b[key])
    elif isinstance(a, (list, tuple)):
        test.assertEqual(len(a), len(b))
        for left, right in zip(a, b):
            assert_trees_equal(test, left, right)
    else:
        test.assertEqual(a, b)


class Tests(unittest.TestCase):
    def test_probe_exactly_preserves_stock_loss_updates_optimizer_and_rng(self):
        torch.manual_seed(19)
        plain = Learner()
        observed = Learner()
        observed.module["default_policy"].load_state_dict(plain.module["default_policy"].state_dict())
        source = inputs(plain)
        initial_rng = probe._rng_state()
        expected = plain.update(source)
        plain_rng = probe._rng_hash(probe._rng_state())
        random.setstate(initial_rng["python"])
        np.random.set_state(initial_rng["numpy"])
        torch.set_rng_state(initial_rng["torch_cpu"])
        with tempfile.TemporaryDirectory() as directory:
            probe.attach_probe(observed, directory, snapshot_minibatches=(2,))
            actual = observed.update(source)
            manifest = probe.detach_probe(observed)
            self.assertEqual(expected, actual)
            self.assertEqual(plain_rng, probe._rng_hash(probe._rng_state()))
            assert_trees_equal(self, plain.get_state(), observed.get_state())
            self.assertTrue(torch.equal(plain.curr_kl_coeffs_per_module["default_policy"], observed.curr_kl_coeffs_per_module["default_policy"]))
            self.assertNotIn("compute_values", observed.module["default_policy"].__dict__)
            self.assertNotIn("_update", observed.__dict__)
            self.assertTrue(manifest["complete"])
            self.assertEqual(manifest["minibatches_completed"], 4)
            self.assertTrue(manifest["snapshots"]["first_pass"]["exact_epoch_boundary"])
            diagnostics = manifest["batch_diagnostics"]["default_policy"]
            self.assertEqual(diagnostics["rows_real"], 7)
            self.assertLess(diagnostics["stored_logp_vs_raw_gaussian_max_abs_error"], 1e-6)
            self.assertGreater(diagnostics["stored_logp_vs_clipped_gaussian_max_abs_error"], .1)
            self.assertEqual(diagnostics["gae_recomputed"]["targets_max_abs_error"], 0.)
            self.assertEqual(diagnostics["gae_recomputed"]["advantages_max_abs_error"], 0.)
            records = [json.loads(line) for line in (Path(directory) / "minibatch_metrics.jsonl").read_text().splitlines()]
            self.assertEqual(records[0]["ratio_max_abs_deviation_from_one"], 0.)
            with (Path(directory) / "episodes_pre_gae.pkl").open("rb") as stream:
                self.assertEqual(len(pickle.load(stream)[0]["infos"]), 8)
            self.assertTrue((Path(directory) / "artifact_hashes.json").exists())

    def test_gae_matches_rllib_for_terminated_and_truncated_chunks(self):
        values = np.array([1., 2., 3., 4., 5., 6.], dtype=np.float32)
        rewards = np.array([.2, .4, 0., .1, -.3, 0.], dtype=np.float32)
        terminated = np.array([False, False, True, False, False, False])
        truncated = np.array([False, False, False, False, False, True])
        expected = compute_value_targets(values, rewards, terminated, truncated, .99, .9)
        actual = probe._gae_targets(values, rewards, terminated, truncated, .99, .9)
        np.testing.assert_array_equal(actual, expected)
        # Artificial terminal rows are loss-masked. Their target need not be
        # numerically zero; only terminal value bootstrapping must be zeroed.
        changed = probe._gae_targets(values + np.array([0, 0, 99, 0, 0, 0]), rewards, terminated, truncated, .99, .9)
        np.testing.assert_array_equal(actual[:3], changed[:3])

    def test_capture_against_installed_general_advantage_connector(self):
        class Multi(dict):
            framework = "torch"
            def foreach_module(self, func, return_dict=False):
                return {mid: func(mid, module) for mid, module in self.items()}
        torch.manual_seed(47)
        module = Module()
        data = inputs(types.SimpleNamespace(module={"default_policy": module})).batch
        values = module.compute_values(data).detach().clone()
        episode = SingleAgentEpisode(
            id_="elongated", observations=[np.zeros(88, dtype=np.float32) for _ in range(9)],
            actions=[np.zeros(2, dtype=np.float32) for _ in range(8)],
            rewards=[0.] * 8, infos=[{} for _ in range(9)], truncated=True,
            len_lookback_buffer=0)
        before = copy.deepcopy(data)
        batch = GeneralAdvantageEstimation(gamma=.99, lambda_=.9)(
            rl_module=Multi(default_policy=module), episodes=[episode], batch={"default_policy": data})
        diagnostics = probe.batch_diagnostics(batch["default_policy"], gamma=.99, lam=.9,
                                               gae_values=values, gae_batch=before)
        self.assertEqual(diagnostics["gae_recomputed"]["targets_max_abs_error"], 0.)
        self.assertEqual(diagnostics["gae_recomputed"]["advantages_max_abs_error"], 0.)


if __name__ == "__main__":
    unittest.main()
