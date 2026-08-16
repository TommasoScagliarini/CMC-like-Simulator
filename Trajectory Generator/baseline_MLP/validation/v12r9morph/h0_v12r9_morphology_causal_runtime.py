"""Additive strict-delay runtime for the V12R9 morphology stage.

The R9 lineage freezes ``experimental_morphology_corridor.py`` byte-for-byte,
so the correction cannot edit that source.  This module installs a versioned
subclass at process start.  It removes terminal-flush samples younger than the
frozen 40 ms causal delay and records them as explicit drops; no early sample
can enter reward.  The launcher/site hook installs the same class in the
supervisor, worker child and Ray worker processes used by final training.
"""

from __future__ import annotations

import dataclasses
import sys
from pathlib import Path
from typing import Any


HERE = Path(__file__).resolve().parent
BASELINE_ROOT = HERE.parents[1]
if str(BASELINE_ROOT) not in sys.path:
    sys.path.insert(0, str(BASELINE_ROOT))

import experimental_morphology_corridor as corridor  # noqa: E402


RUNTIME_ID = "h0_v12r9_morph_strict_terminal_delay_v1"
SITE_MARKER_ENV = "H0_V12R9_MORPH_CAUSAL_RUNTIME"
_TIME_EPS = 1.0e-12
_ORIGINAL_BUFFER = corridor.CausalDelayedMorphologyBuffer


class StrictDelayCausalMorphologyBuffer(_ORIGINAL_BUFFER):
    """Preserve the frozen buffer except for fail-safe terminal delay handling."""

    runtime_id = RUNTIME_ID

    def update(self, *args: Any, **kwargs: Any) -> Any:
        previous_last_emitted = getattr(self, "_last_emitted_sample_time_s", None)
        result = super().update(*args, **kwargs)
        early = tuple(
            item
            for item in result.resolved_samples
            if float(item.delay_s) + _TIME_EPS < float(self.delay_s)
        )
        if not early:
            return result
        retained = tuple(item for item in result.resolved_samples if item not in early)
        if not result.terminal_flushed:
            # A non-terminal early emission is an implementation violation, not
            # an expected drop.  Return failed-closed evidence and no samples.
            count = len(result.resolved_samples)
            self._total_resolved_sample_count -= count
            pending_count = len(self._samples)
            self._samples = []
            self._total_dropped_sample_count += count + pending_count
            self._failed_reason = "causal_delay_guard_nonterminal"
            self._last_emitted_sample_time_s = previous_last_emitted
            return dataclasses.replace(
                result,
                resolved_samples=(),
                dropped_sample_count=(
                    result.dropped_sample_count + count + pending_count
                ),
                drop_reason="causal_delay_guard_nonterminal",
                pending_sample_count=0,
                total_resolved_sample_count=self._total_resolved_sample_count,
                total_dropped_sample_count=self._total_dropped_sample_count,
                failed_closed=True,
                failure_reason=self._failed_reason,
            )
        count = len(early)
        self._total_resolved_sample_count -= count
        self._total_dropped_sample_count += count
        reasons = [item for item in str(result.drop_reason).split("|") if item]
        reasons.append("episode_end_before_delay")
        retained_times = [float(item.sample.time_s) for item in retained]
        if previous_last_emitted is not None:
            retained_times.append(float(previous_last_emitted))
        self._last_emitted_sample_time_s = (
            max(retained_times) if retained_times else None
        )
        return dataclasses.replace(
            result,
            resolved_samples=retained,
            dropped_sample_count=result.dropped_sample_count + count,
            drop_reason="|".join(dict.fromkeys(reasons)),
            total_resolved_sample_count=self._total_resolved_sample_count,
            total_dropped_sample_count=self._total_dropped_sample_count,
        )


def install() -> dict[str, Any]:
    """Install idempotently in both the source module and imported reward module."""

    corridor.CausalDelayedMorphologyBuffer = StrictDelayCausalMorphologyBuffer
    reward = sys.modules.get("reward_function")
    if reward is not None:
        reward.CausalDelayedMorphologyBuffer = StrictDelayCausalMorphologyBuffer
    return {
        "runtime_id": RUNTIME_ID,
        "corridor_installed": (
            corridor.CausalDelayedMorphologyBuffer is StrictDelayCausalMorphologyBuffer
        ),
        "reward_module_loaded": reward is not None,
        "reward_installed": (
            reward is None
            or reward.CausalDelayedMorphologyBuffer is StrictDelayCausalMorphologyBuffer
        ),
    }


def assert_installed() -> dict[str, Any]:
    import reward_function

    checks = {
        "runtime_id": StrictDelayCausalMorphologyBuffer.runtime_id == RUNTIME_ID,
        "corridor": corridor.CausalDelayedMorphologyBuffer
        is StrictDelayCausalMorphologyBuffer,
        "reward": reward_function.CausalDelayedMorphologyBuffer
        is StrictDelayCausalMorphologyBuffer,
    }
    if not all(checks.values()):
        raise RuntimeError(f"strict morphology runtime is not installed: {checks}")
    return {"passed": True, "runtime_id": RUNTIME_ID, "checks": checks}


__all__ = [
    "RUNTIME_ID",
    "SITE_MARKER_ENV",
    "StrictDelayCausalMorphologyBuffer",
    "assert_installed",
    "install",
]
