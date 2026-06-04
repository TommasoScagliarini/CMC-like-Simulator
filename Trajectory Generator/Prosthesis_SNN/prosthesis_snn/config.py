"""Configuration objects for the portable prosthesis SNN."""

from __future__ import annotations

from dataclasses import dataclass, fields
from typing import Any, Mapping, Tuple


@dataclass(frozen=True)
class SNNConfig:
    """Architecture and I/O settings for the prosthesis reference SNN.

    The output layout is `(coord_i, channel_j)` flattened row-major:
    index `i * len(output_channels) + j` of the network output corresponds to
    `(output_coords[i], output_channels[j])`. For the default config this gives
    `[knee_q, knee_qdot, knee_qddot, ankle_q, ankle_qdot, ankle_qddot]`.
    """

    input_size: int = 2
    hidden_size: int = 128
    num_layers: int = 1
    beta: float = 0.5
    threshold: float = 0.5
    reset_mechanism: str = "zero"
    encoding: str = "latency"
    encoding_timesteps: int = 1
    learn_beta: bool = False
    learn_threshold: bool = False
    output_coords: Tuple[str, ...] = (
        "pros_knee_angle",
        "pros_ankle_angle",
    )
    output_channels: Tuple[str, ...] = (
        "q",
        "qdot",
        "qddot",
    )
    feature_names: Tuple[str, ...] = (
        "phase_sin",
        "phase_cos",
    )

    @property
    def output_size(self) -> int:
        return len(self.output_coords) * len(self.output_channels)

    def iter_output_keys(self):
        """Yield (coord, channel) pairs in the same order as the output vector."""
        for coord in self.output_coords:
            for channel in self.output_channels:
                yield coord, channel

    @classmethod
    def from_mapping(cls, data: Mapping[str, Any] | None) -> "SNNConfig":
        """Create a config from a dict, ignoring unknown keys."""
        if data is None:
            return cls()

        valid = {item.name for item in fields(cls)}
        kwargs = {key: value for key, value in data.items() if key in valid}

        for key in ("output_coords", "output_channels", "feature_names"):
            if key in kwargs and not isinstance(kwargs[key], tuple):
                kwargs[key] = tuple(kwargs[key])

        return cls(**kwargs)

    def to_dict(self) -> dict[str, Any]:
        result = {}
        for item in fields(self):
            value = getattr(self, item.name)
            result[item.name] = list(value) if isinstance(value, tuple) else value
        return result
