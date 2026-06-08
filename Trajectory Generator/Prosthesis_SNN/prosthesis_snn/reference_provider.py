"""Reference and action-provider adapters for simulator integration."""

from __future__ import annotations

from abc import ABC, abstractmethod
from collections.abc import Callable, Mapping
from typing import Any

import numpy as np

from .features import phase_features
from .generator import ReferenceGenerator

ReferenceTriple = tuple[dict[str, float], dict[str, float], dict[str, float]]


class ReferenceProvider(ABC):
    """Simulator-facing kinematic-reference provider."""

    @abstractmethod
    def get(self, t: float, state: Any = None) -> ReferenceTriple:
        """Return q_ref, qdot_ref, qddot_ref dictionaries."""


class DictReferenceProvider(ReferenceProvider):
    """Small static provider useful for tests and examples."""

    def __init__(
        self,
        q_ref: Mapping[str, float],
        qdot_ref: Mapping[str, float] | None = None,
        qddot_ref: Mapping[str, float] | None = None,
    ) -> None:
        self._q_ref = dict(q_ref)
        self._qdot_ref = dict(qdot_ref or {})
        self._qddot_ref = dict(qddot_ref or {})

    def get(self, t: float, state: Any = None) -> ReferenceTriple:
        return dict(self._q_ref), dict(self._qdot_ref), dict(self._qddot_ref)


class SNNProsthesisReferenceProvider(ReferenceProvider):
    """
    Provider for legacy SNNs that emit direct `(q, qdot, qddot)` references.

    The current env-aligned SNN contract emits trajectory-env actions with
    shape `(policy_knots, n_prosthetic_coords)`. Use
    `SNNProsthesisActionProvider` or `ReferenceGenerator.predict_action` for
    that path. This provider rejects env-action configs so a trajectory
    checkpoint cannot silently fail to override prosthetic refs.
    """

    def __init__(
        self,
        generator: ReferenceGenerator,
        feature_builder: Callable[[float, Any], Mapping[str, float]] | None = None,
    ) -> None:
        if generator.cfg.output_contract != "kinematic_reference":
            raise ValueError(
                "SNNProsthesisReferenceProvider requires a "
                "kinematic_reference config. Env-aligned checkpoints emit "
                "trajectory actions; use SNNProsthesisActionProvider instead."
            )
        self.generator = generator
        self.feature_builder = feature_builder or (
            lambda t, state: phase_features(t)
        )

    def reset(self) -> None:
        self.generator.reset()

    def get(self, t: float, state: Any = None) -> ReferenceTriple:
        features = self.feature_builder(t, state)
        predicted = self.generator.predict(features)
        q_ref = dict(predicted.get("q", {}))
        qdot_ref = dict(predicted.get("qdot", {}))
        qddot_ref = dict(predicted.get("qddot", {}))
        return q_ref, qdot_ref, qddot_ref


class SNNProsthesisActionProvider:
    """
    Provider for env-aligned SNNs that emit `env.step(action)` matrices.

    The returned action has shape `(policy_knots, n_prosthetic_coords)`, exactly
    matching `CMCLikeProsthesisTrajectoryEnv.action_space`.
    """

    def __init__(
        self,
        generator: ReferenceGenerator,
        feature_builder: Callable[
            [float, Any], Mapping[str, float] | np.ndarray
        ] | None = None,
        action_shape: tuple[int, int] | None = None,
    ) -> None:
        if generator.cfg.output_contract != "env_action":
            raise ValueError(
                "SNNProsthesisActionProvider requires an env_action config."
            )
        self.generator = generator
        self.feature_builder = feature_builder or (
            lambda t, state: phase_features(t)
        )
        self.action_shape = action_shape or generator.cfg.action_shape

    def reset(self) -> None:
        self.generator.reset()

    def get_action(self, t: float, state: Any = None) -> np.ndarray:
        features = self.feature_builder(t, state)
        return self.generator.predict_action(features, self.action_shape)


class HybridReferenceProvider(ReferenceProvider):
    """
    Keep the base provider for biological coordinates and override prosthetics.
    """

    def __init__(
        self,
        base_provider: ReferenceProvider,
        prosthesis_provider: ReferenceProvider,
        prosthetic_coords: tuple[str, ...] = (
            "pros_knee_angle",
            "pros_ankle_angle",
        ),
    ) -> None:
        self.base_provider = base_provider
        self.prosthesis_provider = prosthesis_provider
        self.prosthetic_coords = prosthetic_coords

    def get(self, t: float, state: Any = None) -> ReferenceTriple:
        q_ref, qdot_ref, qddot_ref = self.base_provider.get(t, state)
        q_pros, qdot_pros, qddot_pros = self.prosthesis_provider.get(t, state)

        q_ref = dict(q_ref)
        qdot_ref = dict(qdot_ref)
        qddot_ref = dict(qddot_ref)

        for coord in self.prosthetic_coords:
            if coord in q_pros:
                q_ref[coord] = q_pros[coord]
            if coord in qdot_pros:
                qdot_ref[coord] = qdot_pros[coord]
            if coord in qddot_pros:
                qddot_ref[coord] = qddot_pros[coord]

        return q_ref, qdot_ref, qddot_ref
