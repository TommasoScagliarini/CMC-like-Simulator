"""Configuration objects for the portable prosthesis SNN."""

from __future__ import annotations

from dataclasses import dataclass, fields
from typing import Any, Mapping, Tuple


@dataclass(frozen=True)
class SNNConfig:
    """Architecture and I/O settings for the prosthesis reference SNN.

    The output layout is `(output_coords[i], output_channels[j])` flattened
    row-major. The default contract mirrors the CMC-like trajectory env action:
    `(policy_knots, prosthetic_coords)`, e.g.
    `[knot_1_knee, knot_1_ankle, knot_2_knee, ...]`.

    Direct kinematic-reference output `(q, qdot, qddot) x coord` is still
    supported through ``for_kinematic_reference`` for legacy provider use.
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
    output_contract: str = "env_action"
    output_coords: Tuple[str, ...] = (
        "knot_1",
        "knot_2",
        "knot_3",
    )
    output_channels: Tuple[str, ...] = (
        "pros_knee_angle",
        "pros_ankle_angle",
    )
    feature_names: Tuple[str, ...] = (
        "phase_sin",
        "phase_cos",
    )

    @property
    def output_size(self) -> int:
        return len(self.output_coords) * len(self.output_channels)

    @property
    def action_shape(self) -> Tuple[int, int]:
        """Env action shape represented by this config.

        Valid only for ``output_contract == "env_action"``.
        """
        if self.output_contract != "env_action":
            raise ValueError("action_shape is defined only for env_action configs.")
        return (len(self.output_coords), len(self.output_channels))

    def iter_output_keys(self):
        """Yield (coord, channel) pairs in the same order as the output vector."""
        for coord in self.output_coords:
            for channel in self.output_channels:
                yield coord, channel

    @classmethod
    def for_env_action(
        cls,
        *,
        input_size: int = 2,
        policy_knots: int = 3,
        pros_coords: Tuple[str, ...] = (
            "pros_knee_angle",
            "pros_ankle_angle",
        ),
        feature_names: Tuple[str, ...] = (
            "phase_sin",
            "phase_cos",
        ),
        **kwargs: Any,
    ) -> "SNNConfig":
        """Create a config whose flat output matches ``env.action_space``."""
        if policy_knots < 1:
            raise ValueError("policy_knots must be >= 1.")
        return cls(
            input_size=input_size,
            output_contract="env_action",
            output_coords=tuple(
                f"knot_{index + 1}" for index in range(policy_knots)
            ),
            output_channels=tuple(pros_coords),
            feature_names=tuple(feature_names),
            **kwargs,
        )

    @classmethod
    def for_kinematic_reference(
        cls,
        *,
        input_size: int = 2,
        pros_coords: Tuple[str, ...] = (
            "pros_knee_angle",
            "pros_ankle_angle",
        ),
        output_channels: Tuple[str, ...] = (
            "q",
            "qdot",
            "qddot",
        ),
        feature_names: Tuple[str, ...] = (
            "phase_sin",
            "phase_cos",
        ),
        **kwargs: Any,
    ) -> "SNNConfig":
        """Create a legacy config that emits direct q/qdot/qddot references."""
        return cls(
            input_size=input_size,
            output_contract="kinematic_reference",
            output_coords=tuple(pros_coords),
            output_channels=tuple(output_channels),
            feature_names=tuple(feature_names),
            **kwargs,
        )

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

        if "output_contract" not in kwargs:
            channels = tuple(kwargs.get("output_channels", ()))
            coords = tuple(kwargs.get("output_coords", ()))
            if {"q", "qdot", "qddot"}.intersection(channels):
                kwargs["output_contract"] = "kinematic_reference"
            elif coords and all(str(item).startswith("knot_") for item in coords):
                kwargs["output_contract"] = "env_action"

        return cls(**kwargs)

    def to_dict(self) -> dict[str, Any]:
        result = {}
        for item in fields(self):
            value = getattr(self, item.name)
            result[item.name] = list(value) if isinstance(value, tuple) else value
        return result
