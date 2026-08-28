"""V25-active residual policy head around the frozen H0 actor.

The two-dimensional load/contact proxy is intentionally not reused here: V25
changes discrete event/FSM semantics as well as the primary load observation.
This RLModule instead learns a bounded correction to the two H0 action means
from the deployable actor fields 2:35.  No prescribed or historical analog
detector signal exists in this input contract.

The base asymmetric actor and privileged critic retain their ordinary RLlib
names and shapes.  During the dedicated residual fit only
``primary_split_v25_residual`` is optimized.  The Gaussian log-standard-
deviation is always copied from the untouched H0 forward path, and the final
means are clipped to the physical normalized action interval ``[-1, 1]``.

No reset heuristic is implemented.  The current actor vector has no causal,
unambiguous reset bit; requesting a reset bypass fails explicitly.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence

from ray.rllib.core.rl_module.rl_module import RLModule
from ray.rllib.utils.framework import try_import_torch

from asymmetric_rl_module import (
    AsymmetricActorCriticTorchRLModule,
    _cfg_get,
    _detach_actor_gradient,
    _detach_logstd_gradient,
)
from primary_split_input_adapter import (
    ACTOR_FEATURE_COUNT,
    _clone_state_value,
    _finite_vector,
    _state_values_equal,
)


torch, nn = try_import_torch()


RESIDUAL_INPUT_INDICES = tuple(range(2, ACTOR_FEATURE_COUNT))
RESIDUAL_INPUT_COUNT = len(RESIDUAL_INPUT_INDICES)
RESIDUAL_HIDDEN_DIMS = (128, 128)
DEFAULT_RESIDUAL_LIMITS = (0.175, 0.12)
DEFAULT_INIT_SEED = 20260806

# Ordered names for the frozen 35-column actor layout at indices 2:35.  This is
# an audit surface as well as documentation: no ``online_grf_detector`` analog
# field is available to the residual network.
RESIDUAL_INPUT_FEATURE_NAMES = (
    "pros_knee_angle",
    "pros_knee_angle_vel",
    "pros_ankle_angle",
    "pros_ankle_angle_vel",
    "SEA_Knee_motor_angle",
    "SEA_Knee_motor_speed",
    "SEA_Ankle_motor_angle",
    "SEA_Ankle_motor_speed",
    "online_left_normal_grf_bw",
    "online_left_in_contact",
    "online_left_heel_strike",
    "online_left_toe_off",
    "online_left_gait_phase_sin",
    "online_left_gait_phase_cos",
    "online_left_cycle_duration_s",
    "phase_fsm_wait_hs",
    "phase_fsm_stance_after_hs",
    "phase_fsm_swing_after_to",
    "phase_expected_hs",
    "phase_expected_to",
    "phase_stance_elapsed_norm",
    "phase_swing_elapsed_norm",
    "phase_cycle_progress_credit",
    "pros_knee_angle_previous_endpoint",
    "pros_knee_angle_served_ref",
    "pros_knee_angle_served_ref_vel",
    "pros_knee_angle_served_ref_accel",
    "pros_knee_angle_sea_u",
    "pros_ankle_angle_previous_endpoint",
    "pros_ankle_angle_served_ref",
    "pros_ankle_angle_served_ref_vel",
    "pros_ankle_angle_served_ref_accel",
    "pros_ankle_angle_sea_u",
)

RESET_BYPASS_SUPPORTED = False
RESET_BYPASS_UNAVAILABLE_REASON = (
    "V25 residual reset bypass is unavailable: the actor observation has no "
    "causal, unambiguous reset marker"
)


if len(RESIDUAL_INPUT_INDICES) != 33:
    raise RuntimeError("V25 residual input contract must contain 33 columns")
if len(RESIDUAL_INPUT_FEATURE_NAMES) != len(RESIDUAL_INPUT_INDICES):
    raise RuntimeError("V25 residual feature names do not match its input width")
if any("detector" in name.lower() for name in RESIDUAL_INPUT_FEATURE_NAMES):
    raise RuntimeError("V25 residual must not consume an analog detector signal")


class PrimarySplitV25ResidualTorchRLModule(AsymmetricActorCriticTorchRLModule):
    """PPO-compatible frozen-H0 policy with a bounded V25 residual mean."""

    def setup(self) -> None:
        super().setup()
        if self._n_actor != ACTOR_FEATURE_COUNT:
            raise ValueError(
                "V25 residual requires the frozen 35-column actor layout, "
                f"got n_actor={self._n_actor}"
            )
        if self._action_dim != len(DEFAULT_RESIDUAL_LIMITS):
            raise ValueError(
                "V25 residual requires the two-action H0 contract, "
                f"got action_dim={self._action_dim}"
            )
        if bool(
            _cfg_get(
                self.model_config,
                "primary_split_v25_residual_reset_bypass",
                False,
            )
        ):
            raise ValueError(RESET_BYPASS_UNAVAILABLE_REASON)

        mean = _finite_vector(
            _cfg_get(
                self.model_config,
                "primary_split_v25_residual_input_mean",
                None,
            ),
            label="primary_split_v25_residual_input_mean",
            length=RESIDUAL_INPUT_COUNT,
            default=(0.0,) * RESIDUAL_INPUT_COUNT,
        )
        std = _finite_vector(
            _cfg_get(
                self.model_config,
                "primary_split_v25_residual_input_std",
                None,
            ),
            label="primary_split_v25_residual_input_std",
            length=RESIDUAL_INPUT_COUNT,
            default=(1.0,) * RESIDUAL_INPUT_COUNT,
            strictly_positive=True,
        )
        limits = _finite_vector(
            _cfg_get(
                self.model_config,
                "primary_split_v25_residual_limits",
                None,
            ),
            label="primary_split_v25_residual_limits",
            length=self._action_dim,
            default=DEFAULT_RESIDUAL_LIMITS,
            strictly_positive=True,
        )
        init_seed = int(
            _cfg_get(
                self.model_config,
                "primary_split_v25_residual_init_seed",
                DEFAULT_INIT_SEED,
            )
        )
        if init_seed < 0:
            raise ValueError(
                "primary_split_v25_residual_init_seed must be non-negative"
            )

        self.register_buffer(
            "primary_split_v25_residual_input_mean",
            torch.as_tensor(mean, dtype=torch.float32),
        )
        self.register_buffer(
            "primary_split_v25_residual_input_std",
            torch.as_tensor(std, dtype=torch.float32),
        )
        self.register_buffer(
            "primary_split_v25_residual_limits",
            torch.as_tensor(limits, dtype=torch.float32),
        )
        self.register_buffer(
            "primary_split_v25_residual_input_indices",
            torch.as_tensor(RESIDUAL_INPUT_INDICES, dtype=torch.long),
            persistent=False,
        )

        with torch.random.fork_rng(devices=[]):
            torch.manual_seed(init_seed)
            self.primary_split_v25_residual = nn.Sequential(
                nn.Linear(RESIDUAL_INPUT_COUNT, RESIDUAL_HIDDEN_DIMS[0]),
                nn.Tanh(),
                nn.Linear(RESIDUAL_HIDDEN_DIMS[0], RESIDUAL_HIDDEN_DIMS[1]),
                nn.Tanh(),
                nn.Linear(RESIDUAL_HIDDEN_DIMS[1], self._action_dim),
            )
        output = self.primary_split_v25_residual[-1]
        with torch.no_grad():
            output.weight.zero_()
            output.bias.zero_()

    def residual_delta(self, actor_obs: "torch.Tensor") -> "torch.Tensor":
        """Return the bounded mean correction for a complete actor vector."""

        if actor_obs.shape[-1] != ACTOR_FEATURE_COUNT:
            raise ValueError(
                "V25 residual input must end in the complete 35-column actor layout"
            )
        selected = torch.index_select(
            actor_obs,
            dim=-1,
            index=self.primary_split_v25_residual_input_indices,
        )
        normalized = (
            selected - self.primary_split_v25_residual_input_mean
        ) / self.primary_split_v25_residual_input_std
        raw = self.primary_split_v25_residual(normalized)
        return torch.tanh(raw) * self.primary_split_v25_residual_limits

    def _policy_logits(self, batch: Mapping[str, Any]) -> "torch.Tensor":
        actor_obs = self._actor_in(batch)
        raw_logits = self.pi(actor_obs)
        raw_mean = raw_logits[..., : self._action_dim]
        corrected_mean = torch.clamp(
            raw_mean + self.residual_delta(actor_obs),
            min=-1.0,
            max=1.0,
        )
        logits = torch.cat(
            (corrected_mean, raw_logits[..., self._action_dim :]),
            dim=-1,
        )
        logits = _detach_actor_gradient(logits, self._freeze_actor)
        return _detach_logstd_gradient(
            logits,
            self._action_dim,
            self._freeze_logstd,
        )

    def residual_parameters(self) -> tuple["torch.nn.Parameter", ...]:
        """Return the only parameters authorized for V25 residual fitting."""

        return tuple(self.primary_split_v25_residual.parameters())

    def prepare_residual_fit(self) -> tuple["torch.nn.Parameter", ...]:
        """Freeze H0/critic and enable gradients only for the residual MLP."""

        self.train()
        residual_ids = {id(parameter) for parameter in self.residual_parameters()}
        for parameter in self.parameters():
            parameter.requires_grad_(id(parameter) in residual_ids)
        self._freeze_actor = False
        return self.residual_parameters()

    def prepare_for_ppo(self) -> None:
        """Restore gradients for all actor, critic and residual parameters."""

        for parameter in self.parameters():
            parameter.requires_grad_(True)
        self._freeze_actor = False


def build_v25_residual_module_from_checkpoint(
    source_checkpoint: str | Path,
    *,
    input_mean: Sequence[float],
    input_std: Sequence[float],
    residual_limits: Sequence[float] = DEFAULT_RESIDUAL_LIMITS,
    init_seed: int = DEFAULT_INIT_SEED,
) -> PrimarySplitV25ResidualTorchRLModule:
    """Wrap one asymmetric H0 checkpoint without changing base tensors."""

    checkpoint = Path(source_checkpoint).expanduser().resolve()
    source = RLModule.from_checkpoint(checkpoint)
    if not isinstance(source, AsymmetricActorCriticTorchRLModule):
        raise TypeError(
            "source checkpoint must contain AsymmetricActorCriticTorchRLModule"
        )
    if isinstance(source, PrimarySplitV25ResidualTorchRLModule):
        raise TypeError("source checkpoint is already a V25 residual module")

    model_config = dict(source.model_config)
    model_config.update(
        {
            "primary_split_v25_residual_input_mean": list(input_mean),
            "primary_split_v25_residual_input_std": list(input_std),
            "primary_split_v25_residual_limits": list(residual_limits),
            "primary_split_v25_residual_init_seed": int(init_seed),
            "primary_split_v25_residual_reset_bypass": False,
            "freeze_actor": False,
        }
    )
    wrapped = PrimarySplitV25ResidualTorchRLModule(
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
            "V25 residual wrapper cannot represent source state keys: "
            + ", ".join(absent)
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
            "V25 residual construction changed source tensors: " + ", ".join(drifted)
        )
    return wrapped


def base_state_keys(
    module: PrimarySplitV25ResidualTorchRLModule,
) -> tuple[str, ...]:
    """Return checkpoint keys belonging to H0/critic rather than V25 residual."""

    return tuple(
        key
        for key in module.get_state()
        if not key.startswith("primary_split_v25_residual")
    )


def residual_named_parameters(
    module: PrimarySplitV25ResidualTorchRLModule,
) -> Iterable[tuple[str, "torch.nn.Parameter"]]:
    """Yield stable names for the separately trainable residual parameters."""

    yield from module.primary_split_v25_residual.named_parameters(
        prefix="primary_split_v25_residual"
    )
