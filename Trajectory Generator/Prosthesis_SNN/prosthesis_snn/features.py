"""Feature helpers for provisional runtime inputs."""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from typing import Any

import numpy as np


def phase_features(t: float, period: float = 1.0, t0: float = 0.0) -> dict[str, float]:
    """Return a minimal cyclic feature set from time."""
    if period <= 0:
        raise ValueError("period must be positive")
    phase = ((float(t) - t0) / period) % 1.0
    angle = 2.0 * math.pi * phase
    return {
        "phase_sin": math.sin(angle),
        "phase_cos": math.cos(angle),
    }


def as_feature_vector(
    features: Mapping[str, Any] | Sequence[float] | np.ndarray,
    feature_names: Sequence[str],
) -> np.ndarray:
    """Convert mapping or sequence features into a float32 vector."""
    if isinstance(features, Mapping):
        missing = [name for name in feature_names if name not in features]
        if missing:
            raise KeyError(f"missing feature(s): {missing}")
        values = [features[name] for name in feature_names]
    else:
        values = list(features)

    arr = np.asarray(values, dtype=np.float32)
    if arr.ndim != 1:
        raise ValueError("features must be a 1D vector")
    if arr.shape[0] != len(feature_names):
        raise ValueError(
            f"expected {len(feature_names)} features, got {arr.shape[0]}"
        )
    return arr
