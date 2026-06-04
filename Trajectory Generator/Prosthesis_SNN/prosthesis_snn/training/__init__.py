"""Optional training utilities for the prosthesis SNN package."""

from .actor_critic import ProsthesisSNNActorCritic, build_actor_critic
from .checkpoint import (
    REFERENCE_CHECKPOINT_FORMAT,
    REFERENCE_CHECKPOINT_VERSION,
    build_reference_checkpoint,
    save_reference_checkpoint,
)
from .entropy_scheduler import LinearEntropyDecay
from .ppo_snn import PPO_SNN, PPO_SNN_DEFAULT_CONFIG

__all__ = [
    "LinearEntropyDecay",
    "PPO_SNN",
    "PPO_SNN_DEFAULT_CONFIG",
    "REFERENCE_CHECKPOINT_FORMAT",
    "REFERENCE_CHECKPOINT_VERSION",
    "ProsthesisSNNActorCritic",
    "build_actor_critic",
    "build_reference_checkpoint",
    "save_reference_checkpoint",
]
