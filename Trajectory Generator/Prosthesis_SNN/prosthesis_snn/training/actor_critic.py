"""skrl actor-critic wrapper for the prosthesis SNN.

The structure follows the shared policy/value SNN model from
TommasoScagliarini/SNN-Colangelo-Mardaru-Scagliarini:

- one SNN backbone is shared by policy and value;
- membrane tensors are exposed as skrl ``rnn`` state;
- policy and value calls with the same state reuse a small forward-pass cache;
- policy/value can be passed to ``PPO_SNN`` as the same object.
"""

from __future__ import annotations

from dataclasses import replace
from typing import Any, Sequence

import torch
import torch.nn as nn
from torch.distributions import Normal

from ..config import SNNConfig
from ..model import NoSpikingLIF, ProsthesisReferenceSNN

try:  # pragma: no cover - exercised only with the training extra installed.
    from skrl.models.torch import Model

    _SKRL_IMPORT_ERROR: ModuleNotFoundError | None = None
except ModuleNotFoundError as exc:  # pragma: no cover - import guard.
    Model = object  # type: ignore[assignment,misc]
    _SKRL_IMPORT_ERROR = exc


class ProsthesisSNNActorCritic(Model):  # type: ignore[misc]
    """Shared-backbone SNN actor-critic for ``PPO_SNN``.

    The policy head is the same non-spiking output head used by
    ``ProsthesisReferenceSNN`` so its weights can be exported directly for
    inference through ``ReferenceGenerator``. The value head is training-only.
    """

    def __init__(
        self,
        observation_space: Any,
        action_space: Any,
        device: str | torch.device | None = None,
        cfg: SNNConfig | None = None,
        num_envs: int = 1,
        sequence_length: int = 1,
        log_std_init: float = -1.0,
        min_log_std: float = -20.0,
        max_log_std: float = 0.0,
        clip_actions: bool = True,
        clip_mean_actions: bool = False,
    ) -> None:
        if _SKRL_IMPORT_ERROR is not None:
            raise ModuleNotFoundError(
                "ProsthesisSNNActorCritic requires the optional training "
                "dependency 'skrl'. Install this package with the training "
                "extra before using it."
            ) from _SKRL_IMPORT_ERROR

        Model.__init__(
            self,
            observation_space=observation_space,
            action_space=action_space,
            device=device,
        )

        self.cfg = self._coerce_config(cfg)
        self.num_envs = int(num_envs)
        self.sequence_length = int(sequence_length)
        self.min_log_std = float(min_log_std)
        self.max_log_std = float(max_log_std)
        self.clip_actions = bool(clip_actions)
        self.clip_mean_actions = bool(clip_mean_actions)

        self.reference_model = ProsthesisReferenceSNN(self.cfg)
        self.value_lif = NoSpikingLIF(
            self.cfg.hidden_size,
            1,
            self.cfg.beta,
            self.cfg.learn_beta,
        )
        self.log_std_parameter = nn.Parameter(
            torch.full((self.num_actions,), float(log_std_init))
        )

        min_actions, max_actions = self._action_limits(action_space)
        if min_actions is None or max_actions is None:
            self._min_actions = None
            self._max_actions = None
        else:
            self.register_buffer("_min_actions", min_actions.to(self.device))
            self.register_buffer("_max_actions", max_actions.to(self.device))

        self._distribution: Normal | None = None
        self._last_firing_rate = 0.0
        self._cache_states_ptr: int | None = None
        self._cache_mem_ptr: int | None = None
        self._cache_result: tuple[torch.Tensor, torch.Tensor, list[torch.Tensor]] | None = None

        self.to(self.device)

    @property
    def last_firing_rate(self) -> float:
        return self._last_firing_rate

    def reset_hidden_states(self, env_ids: torch.Tensor | Sequence[int]) -> None:
        """Compatibility hook for skrl trainers.

        ``PPO_SNN`` owns the actual membrane reset logic and zeroes finished
        environments when termination/truncation is observed.
        """

    def get_specification(self) -> dict[str, Any]:
        hidden = self.cfg.hidden_size
        return {
            "rnn": {
                "sequence_length": self.sequence_length,
                "sizes": (
                    [(1, self.num_envs, hidden)] * (self.cfg.num_layers + 1)
                    + [(1, self.num_envs, self.num_actions)]
                    + [(1, self.num_envs, 1)]
                ),
            }
        }

    def act(
        self,
        inputs: dict[str, Any],
        *,
        role: str = "",
    ) -> tuple[torch.Tensor, torch.Tensor | None, dict[str, Any]]:
        if role == "value":
            value, _, outputs = self.compute(inputs, role=role)
            return value, None, outputs

        mean_actions, log_std, outputs = self.compute(inputs, role="policy")
        log_std = torch.clamp(log_std, min=self.min_log_std, max=self.max_log_std)
        if log_std.dim() == 1:
            log_std = log_std.expand_as(mean_actions)

        if self.clip_mean_actions:
            mean_actions = self._clip_to_action_space(mean_actions)

        self._distribution = Normal(mean_actions, log_std.exp())
        actions = self._distribution.rsample()
        if self.clip_actions:
            actions = self._clip_to_action_space(actions)

        taken_actions = inputs.get("taken_actions", actions)
        log_prob = self._distribution.log_prob(taken_actions).sum(dim=-1, keepdim=True)
        outputs.update(
            {
                "log_std": log_std,
                "log_prob": log_prob,
                "mean_actions": mean_actions,
            }
        )
        return actions, log_prob, outputs

    def random_act(
        self,
        inputs: dict[str, Any],
        *,
        role: str = "",
    ) -> tuple[torch.Tensor, torch.Tensor | None, dict[str, Any]]:
        # Keep PPO bookkeeping coherent: the sampled action and log-probability
        # come from the same Gaussian behavior policy.
        return self.act(inputs, role=role)

    def compute(
        self,
        inputs: dict[str, Any],
        role: str = "",
    ) -> tuple[torch.Tensor, torch.Tensor, dict[str, Any]]:
        states = self._states_from_inputs(inputs).to(self.device)
        mem = self._prepare_mem(inputs.get("rnn"), states)

        states_ptr = states.data_ptr()
        mem_ptr = mem[0].data_ptr()
        if (
            self._cache_result is not None
            and self._cache_states_ptr == states_ptr
            and self._cache_mem_ptr == mem_ptr
        ):
            mean_actions, value, stored_mem = self._cache_result
        else:
            mean_actions, value, stored_mem = self._forward_shared(states, mem)
            self._cache_states_ptr = states_ptr
            self._cache_mem_ptr = mem_ptr
            self._cache_result = (mean_actions, value, stored_mem)

        if role == "value":
            return value, self.log_std_parameter, {"rnn": stored_mem}
        return mean_actions, self.log_std_parameter, {"rnn": stored_mem}

    def get_entropy(self, *, role: str = "") -> torch.Tensor:
        if self._distribution is None:
            return torch.zeros(1, 1, device=self.device)
        return self._distribution.entropy().sum(dim=-1, keepdim=True)

    def distribution(self, *, role: str = "") -> Normal | None:
        return self._distribution

    def reference_state_dict(self) -> dict[str, torch.Tensor]:
        """Return the policy/reference weights in ``ProsthesisReferenceSNN`` format."""
        return self.reference_model.state_dict()

    def load_reference_state_dict(self, state_dict: dict[str, torch.Tensor]) -> None:
        """Load policy/reference weights from a ``ProsthesisReferenceSNN`` state dict."""
        self.reference_model.load_state_dict(state_dict)
        self._clear_cache()

    def _coerce_config(self, cfg: SNNConfig | None) -> SNNConfig:
        if cfg is None:
            cfg = SNNConfig(input_size=self.num_observations)
        elif cfg.input_size != self.num_observations:
            cfg = replace(cfg, input_size=self.num_observations)

        if cfg.output_size != self.num_actions:
            raise ValueError(
                "action_space size must match SNN output size: "
                f"{self.num_actions} != {cfg.output_size}"
            )
        return cfg

    def _prepare_mem(
        self,
        mem_raw: Any,
        states: torch.Tensor,
    ) -> list[torch.Tensor]:
        if mem_raw is None or len(mem_raw) == 0:
            batch_size = states.shape[0]
            if batch_size % self.num_envs == 0:
                num_envs = self.num_envs
            else:
                num_envs = batch_size
            return self._init_mem(num_envs)

        mem: list[torch.Tensor] = []
        for item in mem_raw:
            tensor = item.to(self.device)
            if tensor.dim() == 3 and tensor.shape[0] == 1:
                tensor = tensor.squeeze(0)
            mem.append(tensor)
        return mem

    def _init_mem(self, num_envs: int) -> list[torch.Tensor]:
        hidden = self.cfg.hidden_size
        return (
            [torch.zeros(num_envs, hidden, device=self.device) for _ in range(self.cfg.num_layers + 1)]
            + [torch.zeros(num_envs, self.num_actions, device=self.device)]
            + [torch.zeros(num_envs, 1, device=self.device)]
        )

    def _forward_shared(
        self,
        states: torch.Tensor,
        mem: list[torch.Tensor],
    ) -> tuple[torch.Tensor, torch.Tensor, list[torch.Tensor]]:
        policy_out_mem = mem[-2]
        value_out_mem = mem[-1]
        backbone_mem = mem[:-2]
        num_envs = backbone_mem[0].shape[0]
        batch_size = states.shape[0]

        if batch_size != num_envs:
            states_seq = self._reshape_for_update(states, num_envs)
            features, new_backbone_mem = self.reference_model.backbone(
                states_seq,
                backbone_mem,
            )
            policy_seq, policy_out_mem = self.reference_model.output_lif(
                features,
                policy_out_mem,
            )
            value_seq, value_out_mem = self.value_lif(features, value_out_mem)
            mean_actions = policy_seq.transpose(0, 1).reshape(batch_size, self.num_actions)
            value = value_seq.transpose(0, 1).reshape(batch_size, 1)
        else:
            features, new_backbone_mem = self.reference_model.backbone(
                states,
                backbone_mem,
            )
            mean_actions, policy_out_mem = self.reference_model.output_lif(
                features,
                policy_out_mem,
            )
            value, value_out_mem = self.value_lif(features, value_out_mem)

        self._last_firing_rate = self.reference_model.backbone.last_firing_rate
        stored_mem = (
            [item.unsqueeze(0) for item in new_backbone_mem]
            + [policy_out_mem.unsqueeze(0)]
            + [value_out_mem.unsqueeze(0)]
        )
        return mean_actions, value, stored_mem

    def _reshape_for_update(self, states: torch.Tensor, num_envs: int) -> torch.Tensor:
        if states.shape[0] % num_envs != 0:
            raise ValueError(
                "sampled state batch must be a multiple of the RNN state batch "
                f"size ({states.shape[0]} vs {num_envs})"
            )
        sequence_length = states.shape[0] // num_envs
        return states.view(num_envs, sequence_length, -1).transpose(0, 1)

    def _states_from_inputs(self, inputs: dict[str, Any]) -> torch.Tensor:
        if "states" in inputs:
            return inputs["states"]
        if "observations" in inputs:
            return inputs["observations"]
        raise KeyError("inputs must contain 'states' or 'observations'")

    def _clip_to_action_space(self, values: torch.Tensor) -> torch.Tensor:
        if self._min_actions is None or self._max_actions is None:
            return values
        return torch.max(torch.min(values, self._max_actions), self._min_actions)

    def _action_limits(
        self,
        action_space: Any,
    ) -> tuple[torch.Tensor | None, torch.Tensor | None]:
        low = getattr(action_space, "low", None)
        high = getattr(action_space, "high", None)
        if low is None or high is None:
            return None, None

        min_actions = torch.as_tensor(low, dtype=torch.float32)
        max_actions = torch.as_tensor(high, dtype=torch.float32)
        if not torch.isfinite(min_actions).all() or not torch.isfinite(max_actions).all():
            return None, None
        return min_actions.reshape(1, -1), max_actions.reshape(1, -1)

    def _clear_cache(self) -> None:
        self._cache_states_ptr = None
        self._cache_mem_ptr = None
        self._cache_result = None


def build_actor_critic(
    observation_space: Any,
    action_space: Any,
    device: str | torch.device | None = None,
    cfg: SNNConfig | None = None,
    num_envs: int = 1,
    sequence_length: int = 1,
    **kwargs: Any,
) -> tuple[ProsthesisSNNActorCritic, ProsthesisSNNActorCritic]:
    """Build a shared actor-critic pair for ``PPO_SNN``."""

    model = ProsthesisSNNActorCritic(
        observation_space=observation_space,
        action_space=action_space,
        device=device,
        cfg=cfg,
        num_envs=num_envs,
        sequence_length=sequence_length,
        **kwargs,
    )
    return model, model
