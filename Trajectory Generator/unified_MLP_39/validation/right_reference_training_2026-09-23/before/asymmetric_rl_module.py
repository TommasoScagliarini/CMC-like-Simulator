"""Unified V1 asymmetric actor-critic RLModule for PPO (Ray RLlib new API stack, 2.55.x).

The policy (pi) reads only the REALISTIC actor prefix ``obs[:, :n_actor]`` -- the
signals a real instrumented prosthesis could sense -- while the value function
(vf) reads the FULL ``obs[:, :n_full]`` superset, which in simulation also carries
privileged state (true pelvis/contralateral kinematics, IK reference, sound-side
load). This is the standard "privileged critic": a better value estimate yields
lower-variance advantages, while the deployed policy stays realistic because the
privileged features never enter its input.

The env (``osim_trj_cmc_like.CMCLikeProsthesisTrajectoryEnv`` with
``critic_privileged_observation=True``) lays the observation out so the actor
features are a contiguous prefix ``obs[:n_actor]`` and the privileged features the
suffix. ``n_actor`` is passed in via ``model_config["n_actor"]``; ``n_full`` is the
observation-space dimension. See the 2026-06-10 observation-space report.

This module lives in an importable file (NOT ``__main__``) so that
``RLModule.from_checkpoint`` can reconstruct it on remote EnvRunners and in
``rollout_eval``. The custom ``n_actor`` rides inside ``model_config``, which RLlib
serialises with the checkpoint, so no custom serialisation is needed.

Inference contract: ``forward_inference({"obs": t})`` works whether ``t`` is the
full vector (the policy slices ``[:, :n_actor]`` internally) or already exactly
``n_actor`` wide (the slice is then a no-op) -- the latter is what a future
realistic real-world deployment would feed.
"""

from __future__ import annotations

from typing import Any, Dict, List, Optional

import numpy as np

from ray.rllib.algorithms.ppo.torch.default_ppo_torch_rl_module import (
    DefaultPPOTorchRLModule,
)
from ray.rllib.core.columns import Columns
from ray.rllib.core.rl_module.apis.value_function_api import ValueFunctionAPI
from ray.rllib.core.rl_module.rl_module import RLModule
from ray.rllib.core.rl_module.torch import TorchRLModule
from ray.rllib.utils.annotations import override
from ray.rllib.utils.framework import try_import_torch

torch, nn = try_import_torch()


_ACTIVATIONS = {
    "tanh": nn.Tanh,
    "relu": nn.ReLU,
    "elu": nn.ELU,
    "swish": nn.SiLU,
    "silu": nn.SiLU,
}


def _cfg_get(model_config: Any, key: str, default: Any) -> Any:
    """Read a key from a model_config that may be a dict or a dataclass."""
    if isinstance(model_config, dict):
        return model_config.get(key, default)
    return getattr(model_config, key, default)


def _mlp(in_dim: int, hiddens: List[int], activation: str) -> nn.Sequential:
    act_cls = _ACTIVATIONS.get(str(activation).lower(), nn.Tanh)
    layers: List[nn.Module] = []
    prev = in_dim
    for h in hiddens:
        layers.append(nn.Linear(prev, h))
        layers.append(act_cls())
        prev = h
    return nn.Sequential(*layers)


def _detach_logstd_gradient(
    logits: "torch.Tensor", action_dim: int, freeze: bool
) -> "torch.Tensor":
    """Keep Gaussian values unchanged while blocking log-std gradients."""
    if not freeze:
        return logits
    return torch.cat(
        (logits[..., :action_dim], logits[..., action_dim:].detach()),
        dim=-1,
    )


def _detach_actor_gradient(logits: "torch.Tensor", freeze: bool) -> "torch.Tensor":
    """Keep the policy distribution unchanged while blocking all actor gradients."""
    return logits.detach() if freeze else logits


class AsymmetricActorCriticTorchRLModule(DefaultPPOTorchRLModule):
    """PPO RLModule with a realistic policy and a privileged value function.

    Builds its own MLPs in ``setup()`` (the catalog encoder cannot give actor and
    critic different input dims), so ``catalog_class`` is forced to ``None`` and the
    ``TorchRLModule`` ``Box`` -> ``TorchDiagGaussian`` fallback supplies the action
    distribution.
    """

    def __init__(self, *args, **kwargs):
        # Bypass DefaultPPOTorchRLModule.__init__ (which would install PPOCatalog).
        # With catalog_class=None, self.action_dist_cls stays None and
        # TorchRLModule.get_inference_action_dist_cls() falls back to TorchDiagGaussian
        # for our flat Box action space.
        kwargs["catalog_class"] = None
        TorchRLModule.__init__(self, *args, **kwargs)

    # ------------------------------------------------------------------
    @override(RLModule)
    def setup(self) -> None:
        n_full = int(np.prod(self.observation_space.shape))
        n_actor = int(_cfg_get(self.model_config, "n_actor", n_full))
        if not (0 < n_actor <= n_full):
            raise ValueError(
                f"n_actor ({n_actor}) must be in (0, n_full={n_full}]. Pass it via "
                "model_config['n_actor'] from env.n_actor."
            )
        self._n_actor = n_actor
        self._n_full = n_full

        action_dim = int(np.prod(self.action_space.shape))
        self._action_dim = action_dim
        self._freeze_logstd = bool(
            _cfg_get(self.model_config, "freeze_logstd", False)
        )
        self._freeze_actor = bool(_cfg_get(self.model_config, "freeze_actor", False))
        initial_phase = 1 if self._freeze_actor else (2 if self._freeze_logstd else 0)
        self.register_buffer("_unified_training_phase", torch.tensor(initial_phase, dtype=torch.int64))
        self.register_buffer("_unified_freeze_logstd", torch.tensor(self._freeze_logstd or self._freeze_actor))
        hiddens = list(_cfg_get(self.model_config, "fcnet_hiddens", [256, 256]))
        activation = _cfg_get(self.model_config, "fcnet_activation", "tanh")
        pi_out = 2 * action_dim  # Gaussian mean + log-std (TorchDiagGaussian).

        # Policy tower: realistic actor prefix only.
        self.pi_encoder = _mlp(n_actor, hiddens, activation)
        self.pi = nn.Sequential(
            self.pi_encoder,
            nn.Linear(hiddens[-1] if hiddens else n_actor, pi_out),
        )
        # Value tower (privileged): full observation. Kept as separate attributes
        # (vf_encoder + vf) so the inference-only export can strip both.
        self.vf_encoder = _mlp(n_full, hiddens, activation)
        self.vf = nn.Linear(hiddens[-1] if hiddens else n_full, 1)

    def set_training_phase(self, phase: str, *, freeze_logstd: bool | None = None) -> None:
        """Select the objective phase without rebuilding any actor parameter.

        Phase flags are registered buffers, so learner/EnvRunner synchronization
        and RLlib checkpoint restoration preserve them independently of the
        constructor's original model_config. The transition trainer constructs a
        fresh optimizer: this method deliberately does not transplant moments.
        """
        codes = {"imitation": 0, "critic_warmup": 1, "ex_novo": 2}
        if phase not in codes:
            raise ValueError(f"unknown unified training phase: {phase!r}")
        frozen = phase != "imitation" if freeze_logstd is None else bool(freeze_logstd)
        if phase == "critic_warmup":
            frozen = True
        self._unified_training_phase.fill_(codes[phase])
        self._unified_freeze_logstd.fill_(frozen)
        # Clear any previous gradient before entering the critic-only iteration.
        if phase == "critic_warmup":
            for parameter in self.pi.parameters():
                parameter.grad = None

    def get_training_phase_state(self) -> dict:
        codes = {0: "imitation", 1: "critic_warmup", 2: "ex_novo"}
        code = int(self._unified_training_phase.item())
        if code not in codes:
            raise ValueError(f"invalid persisted unified training phase: {code}")
        return {
            "phase": codes[code],
            "freeze_actor": code == 1,
            "freeze_logstd": bool(self._unified_freeze_logstd.item()),
        }

    def restore_training_phase_state(self, state: dict) -> None:
        self.set_training_phase(state["phase"], freeze_logstd=state["freeze_logstd"])

    # ------------------------------------------------------------------
    def _masked_observation(self, batch):
        obs = batch[Columns.OBS]
        # Persisted phase buffer is shared by learner, sampler and checkpoint.
        # Defense in depth: ex-novo/warmup cannot reopen target channels even
        # if the caller bypasses the collector and supplies unmasked values.
        if int(self._unified_training_phase.item()) != 0:
            obs = obs.clone()
            obs[..., :2] = 0.0
            obs[..., 35:39] = 0.0
        return obs

    def _actor_in(self, batch: Dict[str, Any]) -> "torch.Tensor":
        obs = self._masked_observation(batch)
        # Slice the realistic prefix. Works for full (n_full) or already-trimmed
        # (n_actor) inference inputs alike.
        return obs[..., : self._n_actor]

    def _critic_in(self, batch: Dict[str, Any]) -> "torch.Tensor":
        return self._masked_observation(batch)[..., : self._n_full]

    def _policy_logits(self, batch: Dict[str, Any]) -> "torch.Tensor":
        logits = self.pi(self._actor_in(batch))
        logits = _detach_actor_gradient(logits, int(self._unified_training_phase.item()) == 1)
        return _detach_logstd_gradient(
            logits,
            self._action_dim,
            bool(self._unified_freeze_logstd.item()),
        )

    @override(DefaultPPOTorchRLModule)
    def _forward(self, batch: Dict[str, Any], **kwargs) -> Dict[str, Any]:
        return {Columns.ACTION_DIST_INPUTS: self._policy_logits(batch)}

    @override(DefaultPPOTorchRLModule)
    def _forward_train(self, batch: Dict[str, Any], **kwargs) -> Dict[str, Any]:
        # Emit the critic embedding so the PPO loss can reuse it in compute_values.
        return {
            Columns.ACTION_DIST_INPUTS: self._policy_logits(batch),
            Columns.EMBEDDINGS: self.vf_encoder(self._critic_in(batch)),
        }

    @override(ValueFunctionAPI)
    def compute_values(
        self,
        batch: Dict[str, Any],
        embeddings: Optional[Any] = None,
    ) -> "torch.Tensor":
        if embeddings is None:
            embeddings = self.vf_encoder(self._critic_in(batch))
        return self.vf(embeddings).squeeze(-1)

    # ------------------------------------------------------------------
    @override(RLModule)
    def get_initial_state(self) -> dict:
        # Non-recurrent: no encoder state. (The default implementation references
        # self.encoder, which this module does not build.)
        return {}

    @override(DefaultPPOTorchRLModule)
    def get_non_inference_attributes(self) -> List[str]:
        # Strip the whole privileged value tower on inference-only export.
        return ["vf", "vf_encoder"]
