"""Small deterministic helpers for diagnostic Gaussian action noise."""

from __future__ import annotations

from typing import Any, Sequence


def broadcast_sigma(value: float | Sequence[float], action_dim: int):
    """Return one finite, non-negative sigma per action component."""
    import numpy as np

    if action_dim <= 0:
        raise ValueError("action_dim must be > 0")
    sigma = np.asarray(value, dtype=float)
    if sigma.ndim == 0 or sigma.shape == (1,):
        sigma = np.full(action_dim, float(sigma.reshape(-1)[0]), dtype=float)
    if sigma.shape != (action_dim,):
        raise ValueError(f"sigma must be scalar or have {action_dim} values")
    if not np.all(np.isfinite(sigma)) or np.any(sigma < 0.0):
        raise ValueError("sigma values must be finite and >= 0")
    return sigma


class HeldStandardNormal:
    """Draw standard-normal noise once every ``hold_steps`` calls."""

    def __init__(self, rng: Any, shape: Sequence[int], hold_steps: int):
        if int(hold_steps) < 1:
            raise ValueError("hold_steps must be >= 1")
        self._rng = rng
        self._shape = tuple(int(value) for value in shape)
        self._hold_steps = int(hold_steps)
        self._calls = 0
        self._sample = None

    def next(self):
        if self._sample is None or self._calls % self._hold_steps == 0:
            self._sample = self._rng.standard_normal(self._shape)
        self._calls += 1
        return self._sample.copy()
