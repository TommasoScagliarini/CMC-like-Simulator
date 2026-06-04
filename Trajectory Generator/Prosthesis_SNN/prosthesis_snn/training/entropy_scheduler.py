"""Small schedules used by training-side agents."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class LinearEntropyDecay:
    """Linearly anneal an entropy coefficient over environment steps."""

    start: float
    end: float
    total_steps: int

    def get(self, current_step: int) -> float:
        if self.total_steps <= 0:
            return float(self.end)
        progress = min(max(float(current_step) / float(self.total_steps), 0.0), 1.0)
        return float(self.start + (self.end - self.start) * progress)
