"""skrl PPO agent adapted for SNN policies.

This module is intentionally training-only. It mirrors the useful idea from
TommasoScagliarini/SNN-Colangelo-Mardaru-Scagliarini: treat the SNN membrane
state as recurrent state, carry it across steps inside each episode, and reset
it when an environment finishes. The implementation avoids upstream project
imports and keeps skrl as an optional dependency.
"""

from __future__ import annotations

import copy
import itertools
from typing import Any, Mapping

import torch
import torch.nn as nn
import torch.nn.functional as F

from .entropy_scheduler import LinearEntropyDecay

try:  # pragma: no cover - exercised only when the training extra is installed.
    from skrl.agents.torch import Agent
    from skrl.memories.torch import Memory
    from skrl.models.torch import Model
    try:
        from skrl.resources.schedulers.torch import KLAdaptiveLR
    except ImportError:  # pragma: no cover - compatibility with older skrl.
        from skrl.resources.schedulers.torch import KLAdaptiveRL as KLAdaptiveLR

    _SKRL_IMPORT_ERROR: ModuleNotFoundError | None = None
except ModuleNotFoundError as exc:  # pragma: no cover - import guard.
    Agent = object  # type: ignore[assignment,misc]
    Memory = Any  # type: ignore[assignment]
    Model = Any  # type: ignore[assignment]
    KLAdaptiveLR = ()  # type: ignore[assignment]
    _SKRL_IMPORT_ERROR = exc


PPO_SNN_DEFAULT_CONFIG: dict[str, Any] = {
    "rollouts": 16,
    "learning_epochs": 8,
    "mini_batches": 2,
    "discount_factor": 0.99,
    "lambda": 0.95,
    "learning_rate": 1e-3,
    "learning_rate_scheduler": None,
    "learning_rate_scheduler_kwargs": {},
    "state_preprocessor": None,
    "state_preprocessor_kwargs": {},
    "value_preprocessor": None,
    "value_preprocessor_kwargs": {},
    "random_timesteps": 0,
    "learning_starts": 0,
    "grad_norm_clip": 0.5,
    "ratio_clip": 0.2,
    "value_clip": 0.2,
    "clip_predicted_values": False,
    "entropy_loss_scale": 0.0,
    "entropy_loss_scale_end": 0.0,
    "value_loss_scale": 1.0,
    "kl_threshold": 0,
    "rewards_shaper": None,
    "net_activity": False,
    "experiment": {
        "directory": "",
        "experiment_name": "",
        "write_interval": 250,
        "checkpoint_interval": 1000,
        "store_separately": False,
        "wandb": False,
        "wandb_kwargs": {},
    },
}


class _PPOConfig(dict[str, Any]):
    """dict-compatible config with the attribute hooks expected by skrl 2.x."""

    def __init__(self, data: dict[str, Any]) -> None:
        super().__init__(data)
        self.experiment = self["experiment"]

    def expand(self) -> None:
        pass


class PPO_SNN(Agent):  # type: ignore[misc]
    """PPO agent for skrl-compatible SNN policy/value models.

    Expected model contract:
    - ``models["policy"]`` and ``models["value"]`` are skrl torch ``Model``s.
    - Their ``get_specification()["rnn"]`` entries describe membrane-state
      tensors. Those states are stored in memory and carried manually.
    - A shared actor-critic module can be passed by using the same instance for
      policy and value.
    """

    def __init__(
        self,
        models: Mapping[str, Model],
        memory: Memory | tuple[Memory, ...] | None = None,
        observation_space: Any = None,
        action_space: Any = None,
        device: str | torch.device | None = None,
        cfg: dict[str, Any] | None = None,
        num_envs: int = 1,
    ) -> None:
        if _SKRL_IMPORT_ERROR is not None:
            raise ModuleNotFoundError(
                "PPO_SNN requires the optional training dependency 'skrl'. "
                "Install this package with the training extra before using it."
            ) from _SKRL_IMPORT_ERROR

        merged_cfg = copy.deepcopy(PPO_SNN_DEFAULT_CONFIG)
        merged_cfg.update(cfg or {})

        super().__init__(
            models=models,
            memory=memory,
            observation_space=observation_space,
            action_space=action_space,
            device=device,
            cfg=_PPOConfig(merged_cfg),
        )

        self.policy = self.models.get("policy")
        self.value = self.models.get("value")
        if self.policy is None or self.value is None:
            raise ValueError("PPO_SNN requires 'policy' and 'value' models")

        self.num_envs = num_envs
        self.checkpoint_modules["policy"] = self.policy
        self.checkpoint_modules["value"] = self.value

        self._learning_epochs = self.cfg["learning_epochs"]
        self._mini_batches = self.cfg["mini_batches"]
        self._rollouts = self.cfg["rollouts"]
        self._rollout = 0
        self._grad_norm_clip = self.cfg["grad_norm_clip"]
        self._ratio_clip = self.cfg["ratio_clip"]
        self._value_clip = self.cfg["value_clip"]
        self._clip_predicted_values = self.cfg["clip_predicted_values"]
        self._value_loss_scale = self.cfg["value_loss_scale"]
        self._entropy_loss_scale = self.cfg["entropy_loss_scale"]
        self._kl_threshold = self.cfg["kl_threshold"]
        self._learning_rate = self.cfg["learning_rate"]
        self._learning_rate_scheduler = self.cfg["learning_rate_scheduler"]
        self._state_preprocessor = self.cfg["state_preprocessor"]
        self._value_preprocessor = self.cfg["value_preprocessor"]
        self._discount_factor = self.cfg["discount_factor"]
        self._lambda = self.cfg["lambda"]
        self._random_timesteps = self.cfg["random_timesteps"]
        self._learning_starts = self.cfg["learning_starts"]
        self._rewards_shaper = self.cfg["rewards_shaper"]

        self._entropy_scheduler = LinearEntropyDecay(
            start=self.cfg.get("entropy_loss_scale", 0.0),
            end=self.cfg.get("entropy_loss_scale_end", 0.0),
            total_steps=self.cfg.get("timesteps", 800_000),
        )

        parameters = (
            self.policy.parameters()
            if self.policy is self.value
            else itertools.chain(self.policy.parameters(), self.value.parameters())
        )
        self.optimizer = torch.optim.Adam(parameters, lr=self._learning_rate)
        if self._learning_rate_scheduler is not None:
            self.scheduler = self._learning_rate_scheduler(
                self.optimizer,
                **self.cfg["learning_rate_scheduler_kwargs"],
            )
        self.checkpoint_modules["optimizer"] = self.optimizer

        if self._state_preprocessor:
            self._state_preprocessor = self._state_preprocessor(
                **self.cfg["state_preprocessor_kwargs"]
            )
            self.checkpoint_modules["state_preprocessor"] = self._state_preprocessor
        else:
            self._state_preprocessor = self._empty_preprocessor

        if self._value_preprocessor:
            self._value_preprocessor = self._value_preprocessor(
                **self.cfg["value_preprocessor_kwargs"]
            )
            self.checkpoint_modules["value_preprocessor"] = self._value_preprocessor
        else:
            self._value_preprocessor = self._empty_preprocessor

    def init(self, trainer_cfg: dict[str, Any] | None = None) -> None:
        # skrl 2.x calls dataclasses.asdict(trainer_cfg), while older examples
        # often passed a plain dict. A dict carries no extra information we need
        # here because this agent uses explicit numeric write/checkpoint
        # intervals by default, so keep both call styles accepted.
        super().init(trainer_cfg=None if isinstance(trainer_cfg, dict) else trainer_cfg)
        self.set_mode("eval")

        if self.memory is not None:
            self.memory.create_tensor(
                name="states",
                size=self.observation_space,
                dtype=torch.float32,
            )
            self.memory.create_tensor(
                name="actions",
                size=self.action_space,
                dtype=torch.float32,
            )
            self.memory.create_tensor(name="rewards", size=1, dtype=torch.float32)
            self.memory.create_tensor(name="terminated", size=1, dtype=torch.bool)
            self.memory.create_tensor(name="truncated", size=1, dtype=torch.bool)
            self.memory.create_tensor(name="log_prob", size=1, dtype=torch.float32)
            self.memory.create_tensor(name="values", size=1, dtype=torch.float32)
            self.memory.create_tensor(name="returns", size=1, dtype=torch.float32)
            self.memory.create_tensor(name="advantages", size=1, dtype=torch.float32)
            self._tensors_names = [
                "states",
                "actions",
                "terminated",
                "log_prob",
                "values",
                "returns",
                "advantages",
            ]

        self._rnn = False
        self._rnn_tensors_names: list[str] = []
        self._rnn_final_states: dict[str, list[torch.Tensor]] = {
            "policy": [],
            "value": [],
        }
        self._rnn_initial_states: dict[str, list[torch.Tensor]] = {
            "policy": [],
            "value": [],
        }
        self._rnn_sequence_length = (
            self.policy.get_specification()
            .get("rnn", {})
            .get("sequence_length", 1)
        )

        self._init_model_rnn_state("policy", self.policy)
        if self.policy is self.value:
            self._rnn_initial_states["value"] = self._rnn_initial_states["policy"]
        else:
            self._init_model_rnn_state("value", self.value)

        self._current_log_prob: torch.Tensor | None = None
        self._current_next_states: torch.Tensor | None = None

    def set_mode(self, mode: str) -> None:
        train = mode == "train"
        seen: set[int] = set()
        for model in self.models.values():
            if model is None or id(model) in seen:
                continue
            model.train(train)
            seen.add(id(model))

    def act(
        self,
        observations: torch.Tensor,
        states: torch.Tensor | None = None,
        *,
        timestep: int,
        timesteps: int,
    ) -> tuple[torch.Tensor, Mapping[str, Any]]:
        model_states = states if states is not None else observations
        rnn = {"rnn": self._rnn_initial_states["policy"]} if self._rnn else {}
        inputs = {"states": self._state_preprocessor(model_states), **rnn}

        if timestep < self._random_timesteps:
            actions, log_prob, outputs = self.policy.random_act(
                inputs,
                role="policy",
            )
        else:
            actions, log_prob, outputs = self.policy.act(inputs, role="policy")

        self._current_log_prob = log_prob
        if self._rnn:
            self._rnn_final_states["policy"] = outputs.get("rnn", [])
        outputs = dict(outputs)
        outputs["log_prob"] = log_prob
        return actions, outputs

    def record_transition(
        self,
        observations: torch.Tensor,
        states: torch.Tensor | None,
        actions: torch.Tensor,
        rewards: torch.Tensor,
        next_observations: torch.Tensor,
        next_states: torch.Tensor | None,
        terminated: torch.Tensor,
        truncated: torch.Tensor,
        infos: Any,
        *,
        timestep: int,
        timesteps: int,
    ) -> None:
        model_states = states if states is not None else observations
        model_next_states = next_states if next_states is not None else next_observations

        super().record_transition(
            observations=observations,
            states=model_states,
            actions=actions,
            rewards=rewards,
            next_observations=next_observations,
            next_states=model_next_states,
            terminated=terminated,
            truncated=truncated,
            infos=infos,
            timestep=timestep,
            timesteps=timesteps,
        )

        self._entropy_loss_scale = self._entropy_scheduler.get(timestep)
        self._track_info_scalars(infos)

        if hasattr(self.policy, "_last_firing_rate"):
            self.track_data("SNN/firing_rate_mean", self.policy._last_firing_rate)
        elif hasattr(self.policy, "last_firing_rate"):
            self.track_data("SNN/firing_rate_mean", self.policy.last_firing_rate)

        value_outputs: Mapping[str, Any] = {}
        if self.memory is not None:
            self._current_next_states = model_next_states
            if self._rewards_shaper is not None:
                rewards = self._rewards_shaper(rewards, timestep, timesteps)

            rnn = {"rnn": self._rnn_initial_states["value"]} if self._rnn else {}
            values, _, value_outputs = self.value.act(
                {"states": self._state_preprocessor(model_states), **rnn},
                role="value",
            )
            values = self._value_preprocessor(values, inverse=True)
            rnn_states = self._memory_rnn_states()

            self.memory.add_samples(
                states=model_states,
                actions=actions,
                rewards=rewards,
                next_states=model_next_states,
                terminated=terminated,
                truncated=truncated,
                log_prob=self._current_log_prob,
                values=values,
                **rnn_states,
            )
            for memory in getattr(self, "secondary_memories", ()):
                memory.add_samples(
                    states=model_states,
                    actions=actions,
                    rewards=rewards,
                    next_states=model_next_states,
                    terminated=terminated,
                    truncated=truncated,
                    log_prob=self._current_log_prob,
                    values=values,
                    **rnn_states,
                )

        if self._rnn:
            if self.policy is self.value:
                self._rnn_final_states["value"] = self._rnn_final_states["policy"]
            else:
                self._rnn_final_states["value"] = value_outputs.get("rnn", [])
            self._zero_finished_rnn_states(terminated, truncated)
            self._rnn_initial_states = self._rnn_final_states

    def pre_interaction(self, *, timestep: int, timesteps: int) -> None:
        pass

    def post_interaction(self, *, timestep: int, timesteps: int) -> None:
        self._rollout += 1
        if not self._rollout % self._rollouts and timestep >= self._learning_starts:
            self.set_mode("train")
            self.update(timestep=timestep, timesteps=timesteps)
            self.set_mode("eval")
        super().post_interaction(timestep=timestep, timesteps=timesteps)

    def update(self, *, timestep: int, timesteps: int) -> None:
        self._update(timestep, timesteps)

    def _update(self, timestep: int, timesteps: int) -> None:
        if self._current_next_states is None:
            return

        with torch.no_grad():
            self.value.train(False)
            rnn = {"rnn": self._rnn_initial_states["value"]} if self._rnn else {}
            last_values, _, _ = self.value.act(
                {
                    "states": self._state_preprocessor(
                        self._current_next_states.float()
                    ),
                    **rnn,
                },
                role="value",
            )
            self.value.train(True)
        last_values = self._value_preprocessor(last_values, inverse=True)

        values = self.memory.get_tensor_by_name("values")
        returns, advantages = self._compute_gae(
            rewards=self.memory.get_tensor_by_name("rewards"),
            terminated=self.memory.get_tensor_by_name("terminated"),
            values=values,
            last_values=last_values,
        )

        self.memory.set_tensor_by_name(
            "values",
            self._value_preprocessor(values, train=True),
        )
        self.memory.set_tensor_by_name(
            "returns",
            self._value_preprocessor(returns, train=True),
        )
        self.memory.set_tensor_by_name("advantages", advantages)

        sampled_batches = self.memory.sample_all(
            names=self._tensors_names,
            mini_batches=self._mini_batches,
            sequence_length=self._rnn_sequence_length,
        )
        sampled_rnn_batches = None
        if self._rnn:
            sampled_rnn_batches = self.memory.sample_all(
                names=self._rnn_tensors_names,
                mini_batches=self._mini_batches,
                sequence_length=self._rnn_sequence_length,
            )

        cumulative_policy_loss = 0.0
        cumulative_entropy_loss = 0.0
        cumulative_value_loss = 0.0
        updates = 0

        for epoch in range(self._learning_epochs):
            kl_divergences = []
            for batch_index, batch in enumerate(sampled_batches):
                (
                    sampled_states,
                    sampled_actions,
                    sampled_dones,
                    sampled_log_prob,
                    sampled_values,
                    sampled_returns,
                    sampled_advantages,
                ) = batch

                rnn_policy, rnn_value = self._sampled_rnn_inputs(
                    sampled_rnn_batches,
                    batch_index,
                    sampled_dones,
                )
                sampled_states = self._state_preprocessor(
                    sampled_states,
                    train=(epoch == 0),
                )

                _, next_log_prob, _ = self.policy.act(
                    {
                        "states": sampled_states,
                        "taken_actions": sampled_actions,
                        **rnn_policy,
                    },
                    role="policy",
                )

                ratio_log = next_log_prob - sampled_log_prob
                with torch.no_grad():
                    kl_divergence = ((torch.exp(ratio_log) - 1.0) - ratio_log).mean()
                    kl_divergences.append(kl_divergence)

                if self._kl_threshold and kl_divergence > self._kl_threshold:
                    break

                entropy_loss = self._entropy_loss(next_log_prob)
                ratio = torch.exp(ratio_log)
                surrogate = sampled_advantages * ratio
                surrogate_clipped = sampled_advantages * torch.clip(
                    ratio,
                    1.0 - self._ratio_clip,
                    1.0 + self._ratio_clip,
                )
                policy_loss = -torch.min(surrogate, surrogate_clipped).mean()

                predicted_values, _, _ = self.value.act(
                    {"states": sampled_states, **rnn_value},
                    role="value",
                )
                if self._clip_predicted_values:
                    predicted_values = sampled_values + torch.clip(
                        predicted_values - sampled_values,
                        min=-self._value_clip,
                        max=self._value_clip,
                    )
                value_loss = self._value_loss_scale * F.mse_loss(
                    sampled_returns,
                    predicted_values,
                )

                loss = policy_loss + entropy_loss + value_loss
                self.optimizer.zero_grad()
                loss.backward()
                self._clip_gradients()
                self.optimizer.step()

                cumulative_policy_loss += float(policy_loss.item())
                cumulative_value_loss += float(value_loss.item())
                cumulative_entropy_loss += float(entropy_loss.item())
                updates += 1

            if self._learning_rate_scheduler:
                if isinstance(self.scheduler, KLAdaptiveLR):
                    self.scheduler.step(torch.stack(kl_divergences).mean())
                else:
                    self.scheduler.step()

        denominator = max(1, updates)
        self.track_data("Loss / Policy loss", cumulative_policy_loss / denominator)
        self.track_data("Loss / Value loss", cumulative_value_loss / denominator)
        self.track_data("Loss / Entropy loss", cumulative_entropy_loss / denominator)
        self.track_data(
            "Policy / Standard deviation",
            self.policy.distribution(role="policy").stddev.mean().item(),
        )
        if self._learning_rate_scheduler:
            self.track_data("Learning / Learning rate", self.scheduler.get_last_lr()[0])

    def _init_model_rnn_state(self, role: str, model: Model) -> None:
        sizes = model.get_specification().get("rnn", {}).get("sizes", [])
        for index, size in enumerate(sizes):
            self._rnn = True
            if self.memory is not None:
                tensor_name = f"rnn_{role}_{index}"
                self.memory.create_tensor(
                    name=tensor_name,
                    size=(size[0], size[2]),
                    dtype=torch.float32,
                    keep_dimensions=True,
                )
                self._rnn_tensors_names.append(tensor_name)
            self._rnn_initial_states[role].append(
                torch.zeros(size, dtype=torch.float32, device=self.device)
            )

    def _memory_rnn_states(self) -> dict[str, torch.Tensor]:
        if not self._rnn:
            return {}
        states = {
            f"rnn_policy_{index}": state.transpose(0, 1)
            for index, state in enumerate(self._rnn_initial_states["policy"])
        }
        if self.policy is not self.value:
            states.update(
                {
                    f"rnn_value_{index}": state.transpose(0, 1)
                    for index, state in enumerate(self._rnn_initial_states["value"])
                }
            )
        return states

    def _sampled_rnn_inputs(
        self,
        sampled_rnn_batches: Any,
        batch_index: int,
        sampled_dones: torch.Tensor,
    ) -> tuple[dict[str, Any], dict[str, Any]]:
        if not self._rnn:
            return {}, {}
        batch = sampled_rnn_batches[batch_index]
        if self.policy is self.value:
            rnn_policy = {
                "rnn": [state.transpose(0, 1) for state in batch],
                "terminated": sampled_dones,
            }
            return rnn_policy, rnn_policy

        rnn_policy = {
            "rnn": [
                state.transpose(0, 1)
                for state, name in zip(batch, self._rnn_tensors_names)
                if "policy" in name
            ],
            "terminated": sampled_dones,
        }
        rnn_value = {
            "rnn": [
                state.transpose(0, 1)
                for state, name in zip(batch, self._rnn_tensors_names)
                if "value" in name
            ],
            "terminated": sampled_dones,
        }
        return rnn_policy, rnn_value

    def _compute_gae(
        self,
        rewards: torch.Tensor,
        terminated: torch.Tensor,
        values: torch.Tensor,
        last_values: torch.Tensor,
    ) -> tuple[torch.Tensor, torch.Tensor]:
        advantages = torch.zeros_like(rewards)
        advantage = torch.zeros_like(last_values)
        for index in reversed(range(rewards.shape[0])):
            next_values = last_values if index == rewards.shape[0] - 1 else values[index + 1]
            non_terminal = terminated[index].logical_not().to(values.dtype)
            delta = rewards[index] + self._discount_factor * next_values * non_terminal - values[index]
            advantage = delta + self._discount_factor * self._lambda * non_terminal * advantage
            advantages[index] = advantage

        returns = advantages + values
        advantages = (advantages - advantages.mean()) / (advantages.std() + 1e-8)
        return returns, advantages

    def _entropy_loss(self, next_log_prob: torch.Tensor) -> torch.Tensor:
        if not self._entropy_loss_scale:
            return next_log_prob.new_zeros(())
        entropy = self.policy.get_entropy(role="policy").mean()
        return -float(self._entropy_loss_scale) * entropy

    def _clip_gradients(self) -> None:
        if self._grad_norm_clip <= 0:
            return
        parameters = (
            self.policy.parameters()
            if self.policy is self.value
            else itertools.chain(self.policy.parameters(), self.value.parameters())
        )
        nn.utils.clip_grad_norm_(parameters, self._grad_norm_clip)

    def _zero_finished_rnn_states(
        self,
        terminated: torch.Tensor,
        truncated: torch.Tensor,
    ) -> None:
        finished_mask = torch.logical_or(terminated, truncated)
        finished = finished_mask.nonzero(as_tuple=False)
        if not finished.numel():
            return

        env_indices = finished[:, 0]
        for state in self._rnn_final_states["policy"]:
            state[:, env_indices] = 0.0
        if self.policy is not self.value:
            for state in self._rnn_final_states["value"]:
                state[:, env_indices] = 0.0

    def _track_info_scalars(self, infos: Any) -> None:
        if not hasattr(infos, "get") or not infos.get("log"):
            return
        for key, value in infos["log"].items():
            if isinstance(value, (int, float)):
                self.track_data(key, value)
            elif hasattr(value, "item"):
                self.track_data(key, value.item())
