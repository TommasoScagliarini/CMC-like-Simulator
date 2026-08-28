"""Primary-GRF compatibility adapter for the frozen H0 policy.

The primary load/contact split changes actor columns 10 and 11.  This module
keeps the original actor and privileged critic as ordinary RLlib submodules and
adds a small, separately trainable input translator in front of the actor.  It
is intentionally *not* allowed to read event/FSM columns 12:24, so changing the
event source to the V25 binary detector does not change the translator input.

The adapter predicts only a non-negative load proxy and a binary contact proxy.
The original H0 actor is evaluated on both the untouched and translated actor
observations.  Its mean correction is bounded per action, while the Gaussian
log-standard-deviation always comes byte-for-byte from the untouched H0 path.
The full observation is passed unchanged to the privileged critic.

Reset bypass is deliberately unsupported.  The current 35-column actor
observation has no causal, unambiguous reset marker: the same apparent state can
occur after reset and during normal execution.  A caller asking for an inferred
reset bypass therefore gets an explicit error instead of a heuristic.  If a
future environment adds a documented reset bit to the observation contract,
that contract must be implemented and validated separately.
"""

from __future__ import annotations

import copy
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence

import numpy as np

from ray.rllib.core.rl_module.rl_module import RLModule
from ray.rllib.utils.framework import try_import_torch

from asymmetric_rl_module import (
    AsymmetricActorCriticTorchRLModule,
    _cfg_get,
    _detach_actor_gradient,
    _detach_logstd_gradient,
)


torch, nn = try_import_torch()


ACTOR_FEATURE_COUNT = 35
LOAD_FEATURE_INDEX = 10
CONTACT_FEATURE_INDEX = 11

# Physical/SEA/current primary fields plus served-controller state.  In
# particular, this excludes the disabled clock (0:2) and all detector/event/FSM
# fields (12:25).  Keep the tuple literal and ordered: its order is part of the
# checkpoint contract and of the stored normalization buffers.
ADAPTER_INPUT_INDICES = (
    2,
    3,
    4,
    5,
    6,
    7,
    8,
    9,
    10,
    11,
    25,
    26,
    27,
    28,
    29,
    30,
    31,
    32,
    33,
    34,
)
EVENT_AND_FSM_FEATURE_INDICES = tuple(range(12, 25))
ADAPTER_INPUT_COUNT = len(ADAPTER_INPUT_INDICES)
ADAPTER_HIDDEN_DIMS = (64, 64)
DEFAULT_RESIDUAL_LIMITS = (0.05, 0.13)
DEFAULT_LOAD_UPPER_BOUND_BW = 2.0
DEFAULT_INIT_SEED = 20260806

RESET_BYPASS_SUPPORTED = False
RESET_BYPASS_UNAVAILABLE_REASON = (
    "reset bypass is unavailable: the actor observation has no causal, "
    "unambiguous reset marker"
)


if len(set(ADAPTER_INPUT_INDICES)) != ADAPTER_INPUT_COUNT:
    raise RuntimeError("primary-split adapter input indices are not unique")
if set(ADAPTER_INPUT_INDICES) & set(EVENT_AND_FSM_FEATURE_INDICES):
    raise RuntimeError("primary-split adapter must not consume event/FSM inputs")


def _finite_vector(
    value: Any,
    *,
    label: str,
    length: int,
    default: Sequence[float],
    strictly_positive: bool = False,
) -> tuple[float, ...]:
    """Resolve one finite, fixed-width vector from serializable model config."""

    raw = default if value is None else value
    if isinstance(raw, (str, bytes)):
        raise ValueError(f"{label} must be a numeric sequence")
    try:
        result = tuple(float(item) for item in raw)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label} must be a numeric sequence") from exc
    if len(result) != length:
        raise ValueError(f"{label} must contain exactly {length} values")
    array = np.asarray(result, dtype=np.float64)
    if not np.all(np.isfinite(array)):
        raise ValueError(f"{label} must contain only finite values")
    if strictly_positive and np.any(array <= 0.0):
        raise ValueError(f"{label} values must be strictly positive")
    return result


def _clone_state_value(value: Any) -> Any:
    if hasattr(value, "detach") and hasattr(value, "clone"):
        return value.detach().clone()
    return copy.deepcopy(value)


def _state_values_equal(left: Any, right: Any) -> bool:
    def array(value: Any) -> np.ndarray:
        if hasattr(value, "detach"):
            value = value.detach()
        if hasattr(value, "cpu"):
            value = value.cpu()
        return np.asarray(value)

    return bool(np.array_equal(array(left), array(right)))


class PrimarySplitInputAdapterTorchRLModule(AsymmetricActorCriticTorchRLModule):
    """PPO-compatible actor/critic with a primary-only H0 input translator.

    The base actor and critic retain the exact names and shapes used by
    :class:`AsymmetricActorCriticTorchRLModule`.  New adapter parameters live
    exclusively below ``primary_split_input_adapter``.  A custom adaptation
    driver should call :meth:`prepare_adapter_fit` and optimize only the
    returned parameters.  A subsequently reconstructed checkpoint restores
    ordinary ``requires_grad=True`` parameters and remains usable by PPO.
    """

    def setup(self) -> None:
        super().setup()
        if self._n_actor != ACTOR_FEATURE_COUNT:
            raise ValueError(
                "primary-split adapter requires the frozen 35-column actor "
                f"layout, got n_actor={self._n_actor}"
            )
        if self._action_dim != len(DEFAULT_RESIDUAL_LIMITS):
            raise ValueError(
                "primary-split adapter requires the two-action H0 contract, "
                f"got action_dim={self._action_dim}"
            )
        if bool(
            _cfg_get(
                self.model_config,
                "primary_split_adapter_reset_bypass",
                False,
            )
        ):
            raise ValueError(RESET_BYPASS_UNAVAILABLE_REASON)

        mean = _finite_vector(
            _cfg_get(
                self.model_config,
                "primary_split_adapter_input_mean",
                None,
            ),
            label="primary_split_adapter_input_mean",
            length=ADAPTER_INPUT_COUNT,
            default=(0.0,) * ADAPTER_INPUT_COUNT,
        )
        std = _finite_vector(
            _cfg_get(
                self.model_config,
                "primary_split_adapter_input_std",
                None,
            ),
            label="primary_split_adapter_input_std",
            length=ADAPTER_INPUT_COUNT,
            default=(1.0,) * ADAPTER_INPUT_COUNT,
            strictly_positive=True,
        )
        residual_limits = _finite_vector(
            _cfg_get(
                self.model_config,
                "primary_split_adapter_residual_limits",
                None,
            ),
            label="primary_split_adapter_residual_limits",
            length=self._action_dim,
            default=DEFAULT_RESIDUAL_LIMITS,
            strictly_positive=True,
        )
        load_upper_bound = float(
            _cfg_get(
                self.model_config,
                "primary_split_adapter_load_upper_bound_bw",
                DEFAULT_LOAD_UPPER_BOUND_BW,
            )
        )
        if not np.isfinite(load_upper_bound) or load_upper_bound <= 0.0:
            raise ValueError(
                "primary_split_adapter_load_upper_bound_bw must be finite and "
                "strictly positive"
            )
        init_seed = int(
            _cfg_get(
                self.model_config,
                "primary_split_adapter_init_seed",
                DEFAULT_INIT_SEED,
            )
        )
        if init_seed < 0:
            raise ValueError("primary_split_adapter_init_seed must be non-negative")

        self.register_buffer(
            "primary_split_adapter_input_mean",
            torch.as_tensor(mean, dtype=torch.float32),
        )
        self.register_buffer(
            "primary_split_adapter_input_std",
            torch.as_tensor(std, dtype=torch.float32),
        )
        self.register_buffer(
            "primary_split_adapter_residual_limits",
            torch.as_tensor(residual_limits, dtype=torch.float32),
        )
        self.register_buffer(
            "primary_split_adapter_input_indices",
            torch.as_tensor(ADAPTER_INPUT_INDICES, dtype=torch.long),
            persistent=False,
        )
        self.register_buffer(
            "primary_split_adapter_load_upper_bound_bw",
            torch.as_tensor(load_upper_bound, dtype=torch.float32),
        )

        # Local RNG isolation makes a fresh adapter deterministic without
        # changing the caller's global torch RNG state.  A restored checkpoint
        # overwrites these initial values through its state dict.
        with torch.random.fork_rng(devices=[]):
            torch.manual_seed(init_seed)
            self.primary_split_input_adapter = nn.Sequential(
                nn.Linear(ADAPTER_INPUT_COUNT, ADAPTER_HIDDEN_DIMS[0]),
                nn.Tanh(),
                nn.Linear(ADAPTER_HIDDEN_DIMS[0], ADAPTER_HIDDEN_DIMS[1]),
                nn.Tanh(),
                nn.Linear(ADAPTER_HIDDEN_DIMS[1], 2),
            )
        output = self.primary_split_input_adapter[-1]
        with torch.no_grad():
            output.weight.zero_()
            output.bias.copy_(torch.as_tensor((-2.0, 0.0), dtype=output.bias.dtype))

    def _adapter_proxy(self, actor_obs: "torch.Tensor") -> "torch.Tensor":
        selected = torch.index_select(
            actor_obs,
            dim=-1,
            index=self.primary_split_adapter_input_indices,
        )
        normalized = (
            selected - self.primary_split_adapter_input_mean
        ) / self.primary_split_adapter_input_std
        raw = self.primary_split_input_adapter(normalized)
        load = torch.nn.functional.softplus(raw[..., 0]).clamp(
            min=0.0,
            max=self.primary_split_adapter_load_upper_bound_bw,
        )
        probability = torch.sigmoid(raw[..., 1])
        hard_contact = (probability >= 0.5).to(dtype=actor_obs.dtype)
        if self.training and torch.is_grad_enabled():
            # Binary forward value with the probability's straight-through
            # gradient during the dedicated adapter fit.
            contact = hard_contact.detach() + probability - probability.detach()
        else:
            contact = hard_contact
        return torch.stack((load, contact), dim=-1)

    def adapt_actor_observation(
        self, actor_obs: "torch.Tensor"
    ) -> tuple["torch.Tensor", "torch.Tensor"]:
        """Return a translated copy and its ``[load, contact]`` proxy.

        Only columns 10 and 11 are written.  No reset heuristic is applied.
        """

        if actor_obs.shape[-1] != ACTOR_FEATURE_COUNT:
            raise ValueError(
                "adapter input must end in the complete 35-column actor layout"
            )
        proxy = self._adapter_proxy(actor_obs)
        adapted = actor_obs.clone()
        adapted[..., LOAD_FEATURE_INDEX] = proxy[..., 0]
        adapted[..., CONTACT_FEATURE_INDEX] = proxy[..., 1]
        return adapted, proxy

    def _policy_logits(self, batch: Mapping[str, Any]) -> "torch.Tensor":
        actor_obs = self._actor_in(batch)
        adapted_obs, _proxy = self.adapt_actor_observation(actor_obs)
        raw_logits = self.pi(actor_obs)
        adapted_logits = self.pi(adapted_obs)

        raw_mean = raw_logits[..., : self._action_dim]
        adapted_mean = adapted_logits[..., : self._action_dim]
        residual = adapted_mean - raw_mean
        limits = self.primary_split_adapter_residual_limits
        bounded_mean = raw_mean + torch.maximum(
            torch.minimum(residual, limits),
            -limits,
        )
        # The translated path never supplies log-std.  This makes adapter-only
        # fitting incapable of changing exploration even through its forward
        # graph, and keeps inference bit-exact to the base H0 head.
        logits = torch.cat(
            (bounded_mean, raw_logits[..., self._action_dim :]),
            dim=-1,
        )
        logits = _detach_actor_gradient(logits, self._freeze_actor)
        return _detach_logstd_gradient(
            logits,
            self._action_dim,
            self._freeze_logstd,
        )

    def adapter_parameters(self) -> tuple["torch.nn.Parameter", ...]:
        """Return the only parameters authorized for V6 adapter fitting."""

        return tuple(self.primary_split_input_adapter.parameters())

    def prepare_adapter_fit(self) -> tuple["torch.nn.Parameter", ...]:
        """Freeze base actor/critic parameters and enable only the adapter.

        ``requires_grad`` is not part of a torch state dict.  Consequently a
        module reconstructed from the saved checkpoint again exposes ordinary
        trainable parameters and can participate in a later PPO Algorithm.
        """

        self.train()
        adapter_ids = {id(parameter) for parameter in self.adapter_parameters()}
        for parameter in self.parameters():
            parameter.requires_grad_(id(parameter) in adapter_ids)
        # ``freeze_actor`` detaches the complete policy result and would also
        # suppress adapter gradients.  It is a runtime switch, not a tensor.
        self._freeze_actor = False
        return self.adapter_parameters()

    def prepare_for_ppo(self) -> None:
        """Restore gradients for every registered actor/critic/adapter tensor."""

        for parameter in self.parameters():
            parameter.requires_grad_(True)
        self._freeze_actor = False


def build_adapter_module_from_checkpoint(
    source_checkpoint: str | Path,
    *,
    input_mean: Sequence[float],
    input_std: Sequence[float],
    residual_limits: Sequence[float] = DEFAULT_RESIDUAL_LIMITS,
    load_upper_bound_bw: float = DEFAULT_LOAD_UPPER_BOUND_BW,
    init_seed: int = DEFAULT_INIT_SEED,
) -> PrimarySplitInputAdapterTorchRLModule:
    """Wrap one asymmetric H0 checkpoint without changing its base tensors.

    The returned module has the same inference/full-module form as the source.
    Thus an inference-only H0 remains inference-only, while a full PPO module
    retains its privileged critic.  Use the existing zero-update port to create
    a fresh full critic when the source itself is inference-only.
    """

    checkpoint = Path(source_checkpoint).expanduser().resolve()
    source = RLModule.from_checkpoint(checkpoint)
    if not isinstance(source, AsymmetricActorCriticTorchRLModule):
        raise TypeError(
            "source checkpoint must contain AsymmetricActorCriticTorchRLModule"
        )
    if isinstance(source, PrimarySplitInputAdapterTorchRLModule):
        raise TypeError("source checkpoint is already a primary-split adapter")

    model_config = dict(source.model_config)
    model_config.update(
        {
            "primary_split_adapter_input_mean": list(input_mean),
            "primary_split_adapter_input_std": list(input_std),
            "primary_split_adapter_residual_limits": list(residual_limits),
            "primary_split_adapter_load_upper_bound_bw": float(load_upper_bound_bw),
            "primary_split_adapter_init_seed": int(init_seed),
            "primary_split_adapter_reset_bypass": False,
            # Adapter fitting needs a differentiable policy path.  The helper
            # below still freezes all base tensors before optimization.
            "freeze_actor": False,
        }
    )
    wrapped = PrimarySplitInputAdapterTorchRLModule(
        observation_space=source.observation_space,
        action_space=source.action_space,
        inference_only=source.inference_only,
        learner_only=source.learner_only,
        model_config=model_config,
        catalog_class=None,
    )

    source_state = source.get_state()
    wrapped_state = wrapped.get_state()
    absent = sorted(set(source_state) - set(wrapped_state))
    if absent:
        raise RuntimeError(
            "adapter wrapper cannot represent source state keys: " + ", ".join(absent)
        )
    for key, value in source_state.items():
        wrapped_state[key] = _clone_state_value(value)
    wrapped.set_state(wrapped_state)

    restored = wrapped.get_state()
    drifted = [
        key
        for key, value in source_state.items()
        if not _state_values_equal(value, restored[key])
    ]
    if drifted:
        raise RuntimeError(
            "adapter construction changed source tensors: " + ", ".join(drifted)
        )
    return wrapped


def base_state_keys(module: PrimarySplitInputAdapterTorchRLModule) -> tuple[str, ...]:
    """Return checkpoint keys belonging to H0/critic rather than the adapter."""

    return tuple(
        key
        for key in module.get_state()
        if not key.startswith("primary_split_input_adapter.")
        and not key.startswith("primary_split_adapter_")
    )


def adapter_named_parameters(
    module: PrimarySplitInputAdapterTorchRLModule,
) -> Iterable[tuple[str, "torch.nn.Parameter"]]:
    """Yield stable, auditable names for the separately trainable adapter."""

    yield from module.primary_split_input_adapter.named_parameters(
        prefix="primary_split_input_adapter"
    )
