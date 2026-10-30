"""Pure validation for exactly balanced deterministic multi-start sampling.

The contract intentionally does not import Ray, OpenSim, or project runtime
modules.  Training code can therefore validate its sampling arithmetic before
starting worker processes on either macOS or Windows.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable


@dataclass(frozen=True)
class ExactStartSamplingContract:
    """Resolved arithmetic for one exactly balanced sampling batch."""

    offsets_s: tuple[float, ...]
    rollout_fragment_length: int
    expected_steps_per_start: int
    runners_per_start: int


def _positive_int(value: int, *, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise ValueError(f"{name} must be a positive integer")
    return value


def build_exact_start_sampling_contract(
    *,
    offsets_s: Iterable[float],
    random_init: bool,
    num_env_runners: int,
    train_batch_size: int,
    minibatch_size: int,
) -> ExactStartSamplingContract:
    """Validate and resolve an exactly balanced multi-start sampling layout.

    Every runner is assigned one fixed start.  Exact balance is guaranteed by
    requiring an equal number of runners per start and an equal integer rollout
    fragment per runner.  In exact mode the Learner compacts the post-GAE batch
    back to these real timesteps; requiring complete minibatches then prevents
    RLlib's cyclic iterator from reusing rows to fill a partial minibatch.
    """

    try:
        offsets = tuple(float(value) for value in offsets_s)
    except (TypeError, ValueError) as exc:
        raise ValueError("offsets_s must contain numeric values") from exc

    if not offsets:
        raise ValueError("offsets_s must contain at least one start offset")
    if any(not math.isfinite(value) or value < 0.0 for value in offsets):
        raise ValueError("offsets_s must contain finite, non-negative values")
    if len(set(offsets)) != len(offsets):
        raise ValueError("offsets_s must contain distinct values")
    if len({round(value, 6) for value in offsets}) != len(offsets):
        raise ValueError(
            "offsets_s must remain distinct at six-decimal metric precision"
        )
    if random_init:
        raise ValueError(
            "random_init is incompatible with deterministic multi-start sampling"
        )

    runners = _positive_int(num_env_runners, name="num_env_runners")
    batch_size = _positive_int(train_batch_size, name="train_batch_size")
    minibatch = _positive_int(minibatch_size, name="minibatch_size")

    num_starts = len(offsets)
    if runners % num_starts != 0:
        raise ValueError(
            "num_env_runners must be divisible by the number of start offsets"
        )
    if batch_size % runners != 0:
        raise ValueError("train_batch_size must be divisible by num_env_runners")
    if batch_size % minibatch != 0:
        raise ValueError("train_batch_size must be divisible by minibatch_size")

    rollout_fragment_length = batch_size // runners
    runners_per_start = runners // num_starts
    expected_steps_per_start = rollout_fragment_length * runners_per_start

    return ExactStartSamplingContract(
        offsets_s=offsets,
        rollout_fragment_length=rollout_fragment_length,
        expected_steps_per_start=expected_steps_per_start,
        runners_per_start=runners_per_start,
    )
