#!/usr/bin/env python3
"""Offline validation of legacy and event-anchored morphology corridors.

The audit is intentionally read-only with respect to a rollout.  It rebuilds a
minimal ``phase_fsm`` payload from the flattened fields saved in
``reward_terms`` and delegates phase/corridor/reward calculations to the runtime
helpers in ``baseline_MLP/reward_function.py``.  Three phase contracts are
compared:

* ``legacy``: the phase and availability actually logged by the rollout;
* ``event_anchored_causal``: fixed TO anchor, clipped segment progress and only
  past duration evidence (nominal bootstrap, then reconstructed robust median);
* ``oracle_retrospective``: exact HS--TO--HS durations for complete valid FSM
  cycles.  This last scheme is diagnostic and is not deployable.

The script writes one machine-readable JSON report, one Markdown report and two
PNG figures.  It never changes checkpoints, traces, profiles or reward config.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import sys
import tempfile
from collections import deque
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any

os.environ.setdefault(
    "MPLCONFIGDIR", str(Path(tempfile.gettempdir()) / "cmc_like_matplotlib")
)
os.environ.setdefault(
    "XDG_CACHE_HOME", str(Path(tempfile.gettempdir()) / "cmc_like_cache")
)

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent
BASELINE_DIR = REPO_ROOT / "Trajectory Generator" / "baseline_MLP"
if str(BASELINE_DIR) not in sys.path:
    sys.path.insert(0, str(BASELINE_DIR))

from reward_function import (  # noqa: E402
    RewardConfig,
    _fsm_morphology_phase,
    _load_morphology_profile,
    _morphology_canonical_to_phase,
    _morphology_corridor_at,
    _morphology_interval_losses,
    compute_reward,
)


DEFAULT_ROLLOUT_DIR = (
    REPO_ROOT
    / "Trajectory Generator"
    / "runs"
    / "rollout"
    / "2026-07-15_h0_exact_interleaved_lr5e-7_pilot50_best_"
    "deterministic_nominal_recorded"
)
DEFAULT_LEGACY_PROFILE = (
    BASELINE_DIR / "morphology_profiles" / "ab06_prosthetic_mean_std_corridor.json"
)
DEFAULT_EVENT_PROFILE = (
    BASELINE_DIR
    / "morphology_profiles"
    / "ab06_prosthetic_event_warped_mean_std_corridor.json"
)
DEFAULT_OUTPUT_DIR = SCRIPT_DIR / "event_anchored_morphology_runs" / "2026-07-20_best"
OUTPUT_NAMES = (
    "event_anchored_morphology_summary.json",
    "event_anchored_morphology_summary.md",
    "phase_alignment.png",
    "corridor_coverage.png",
)


@dataclass(frozen=True)
class JointSpec:
    coord: str
    short: str
    title: str
    value_key: str
    logged_min_key: str
    logged_max_key: str
    logged_loss_key: str
    display_sign: float


JOINTS = (
    JointSpec(
        coord="pros_knee_angle",
        short="knee",
        title="Prosthetic knee flexion",
        value_key="pros_knee_angle_served_ref",
        logged_min_key="morphology_knee_min_rad",
        logged_max_key="morphology_knee_max_rad",
        logged_loss_key="morphology_knee_loss",
        display_sign=-1.0,
    ),
    JointSpec(
        coord="pros_ankle_angle",
        short="ankle",
        title="Prosthetic ankle angle",
        value_key="pros_ankle_angle_served_ref",
        logged_min_key="morphology_ankle_min_rad",
        logged_max_key="morphology_ankle_max_rad",
        logged_loss_key="morphology_ankle_loss",
        display_sign=1.0,
    ),
)


@dataclass
class SchemeSeries:
    name: str
    label: str
    profile: Mapping[str, Any]
    phase: np.ndarray
    available: np.ndarray
    source_id: np.ndarray
    low: dict[str, np.ndarray]
    high: dict[str, np.ndarray]
    excursion: dict[str, np.ndarray]
    joint_loss: dict[str, np.ndarray]
    loss: np.ndarray


def _json_load(path: Path) -> Any:
    try:
        with path.open("r", encoding="utf-8") as stream:
            return json.load(stream)
    except json.JSONDecodeError as exc:
        raise ValueError(f"Invalid JSON in {path}: {exc}") from exc


def _finite_float(value: Any, *, context: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"Expected a number for {context}, got {value!r}") from exc
    if not np.isfinite(result):
        raise ValueError(f"Non-finite number for {context}: {value!r}")
    return result


def _optional_finite(value: Any) -> float | None:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if np.isfinite(result) else None


def _resolve_cli_path(path: Path) -> Path:
    path = path.expanduser()
    if not path.is_absolute():
        path = Path.cwd() / path
    return path.resolve()


def _require_file(path: Path, label: str) -> Path:
    path = _resolve_cli_path(path)
    if not path.is_file():
        raise FileNotFoundError(f"{label} not found: {path}")
    return path


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _ordered_sum(values: Sequence[float] | np.ndarray) -> float:
    total = 0.0
    for value in values:
        total += float(value)
    return total


def _percentile(values: np.ndarray, percentile: float) -> float | None:
    finite = values[np.isfinite(values)]
    if finite.size == 0:
        return None
    return float(np.percentile(finite, percentile))


def _safe_mean(values: np.ndarray) -> float | None:
    finite = values[np.isfinite(values)]
    if finite.size == 0:
        return None
    return float(np.mean(finite))


def _safe_max(values: np.ndarray) -> float | None:
    finite = values[np.isfinite(values)]
    if finite.size == 0:
        return None
    return float(np.max(finite))


def _circular_distance(a: np.ndarray | float, b: np.ndarray | float) -> np.ndarray:
    delta = np.abs(np.asarray(a, dtype=float) - np.asarray(b, dtype=float))
    delta = np.mod(delta, 1.0)
    return np.minimum(delta, 1.0 - delta)


def _load_rollout(
    trace_path: Path, summary_path: Path
) -> tuple[list[Mapping[str, Any]], Mapping[str, Any], np.ndarray, np.ndarray]:
    trace_raw = _json_load(trace_path)
    summary = _json_load(summary_path)
    if not isinstance(trace_raw, list) or not trace_raw:
        raise ValueError(f"Trace must be a non-empty JSON list: {trace_path}")
    if not isinstance(summary, Mapping):
        raise ValueError(f"Summary must be a JSON object: {summary_path}")

    rows: list[Mapping[str, Any]] = []
    steps: list[float] = []
    times: list[float] = []
    for index, item in enumerate(trace_raw):
        if not isinstance(item, Mapping):
            raise ValueError(f"trace[{index}] is not an object")
        terms = item.get("reward_terms")
        if not isinstance(terms, Mapping):
            raise ValueError(f"trace[{index}].reward_terms is not an object")
        steps.append(_finite_float(item.get("step"), context=f"trace[{index}].step"))
        times.append(_finite_float(item.get("time"), context=f"trace[{index}].time"))
        rows.append(item)

    step_array = np.asarray(steps, dtype=float)
    time_array = np.asarray(times, dtype=float)
    if np.any(np.diff(step_array) <= 0.0):
        raise ValueError("Trace steps must be strictly increasing")
    if np.any(np.diff(time_array) <= 0.0):
        raise ValueError("Trace times must be strictly increasing")
    expected_steps = summary.get("steps")
    if expected_steps is not None and int(expected_steps) != len(rows):
        raise ValueError(
            f"Trace has {len(rows)} rows but summary reports {expected_steps} steps"
        )
    return rows, summary, step_array, time_array


def _served_values(rows: Sequence[Mapping[str, Any]]) -> dict[str, np.ndarray]:
    result: dict[str, np.ndarray] = {}
    for joint in JOINTS:
        values: list[float] = []
        for index, row in enumerate(rows):
            state = row.get("prosthetic_state")
            value = state.get(joint.value_key) if isinstance(state, Mapping) else None
            if _optional_finite(value) is None:
                terms = row["reward_terms"]
                value = terms.get(f"morphology_{joint.short}_value_rad")
            values.append(
                _finite_float(value, context=f"trace[{index}] served {joint.short}")
            )
        result[joint.short] = np.asarray(values, dtype=float)
    return result


def _counter(terms: Mapping[str, Any], key: str, index: int) -> int:
    value = _finite_float(terms.get(key, 0.0), context=f"trace[{index}].{key}")
    rounded = int(round(value))
    if abs(value - rounded) > 1e-9 or rounded < 0:
        raise ValueError(f"{key} must be a non-negative integer at trace[{index}]")
    return rounded


def _recover_pseudo_fsm(
    rows: Sequence[Mapping[str, Any]],
    times: np.ndarray,
    *,
    history_window: int,
) -> tuple[list[dict[str, float]], list[dict[str, Any]]]:
    """Recover causal FSM payloads and robust duration medians.

    Event times are recovered from the accepted-event counter transition and the
    new state's elapsed clock: ``event_time = row.time - segment_elapsed``.
    Therefore rejected raw detector events never enter the reconstruction.
    """

    if history_window < 1:
        raise ValueError("history_window must be at least 1")
    stance_history: deque[float] = deque(maxlen=history_window)
    swing_history: deque[float] = deque(maxlen=history_window)
    payloads: list[dict[str, float]] = []
    events: list[dict[str, Any]] = []
    previous_hs_count = 0
    previous_to_count = 0
    active_hs_time: float | None = None
    active_to_time: float | None = None

    for index, row in enumerate(rows):
        terms = row["reward_terms"]
        hs_count = _counter(terms, "phase_valid_hs_count", index)
        to_count = _counter(terms, "phase_valid_to_count", index)
        if hs_count < previous_hs_count or to_count < previous_to_count:
            raise ValueError(f"FSM event counters decrease at trace[{index}]")
        if hs_count - previous_hs_count > 1 or to_count - previous_to_count > 1:
            raise ValueError(f"FSM event counter skips more than one at trace[{index}]")
        if hs_count > previous_hs_count and to_count > previous_to_count:
            raise ValueError(f"HS and TO counters both change at trace[{index}]")

        completed_stance: float | None = None
        completed_swing: float | None = None
        if to_count > previous_to_count:
            elapsed = _finite_float(
                terms.get("phase_swing_elapsed_s", 0.0),
                context=f"trace[{index}].phase_swing_elapsed_s",
            )
            event_time = float(times[index] - max(0.0, elapsed))
            if active_hs_time is None or event_time <= active_hs_time:
                raise ValueError(
                    f"Accepted TO without an earlier accepted HS at trace[{index}]"
                )
            active_to_time = event_time
            completed_stance = event_time - active_hs_time
            events.append(
                {
                    "type": "TO",
                    "time_s": event_time,
                    "confirmation_index": index,
                    "confirmation_time_s": float(times[index]),
                    "confirmation_latency_s": float(times[index] - event_time),
                    "valid_hs_count": hs_count,
                    "valid_to_count": to_count,
                    "completed_stance_duration_s": completed_stance,
                }
            )

        if hs_count > previous_hs_count:
            elapsed = _finite_float(
                terms.get("phase_stance_elapsed_s", 0.0),
                context=f"trace[{index}].phase_stance_elapsed_s",
            )
            event_time = float(times[index] - max(0.0, elapsed))
            if active_hs_time is not None:
                if active_to_time is None or not (
                    active_hs_time < active_to_time < event_time
                ):
                    raise ValueError(
                        f"Accepted HS does not close an HS-TO-HS cycle at trace[{index}]"
                    )
                completed_stance = active_to_time - active_hs_time
                completed_swing = event_time - active_to_time
                stance_history.append(completed_stance)
                swing_history.append(completed_swing)
            active_hs_time = event_time
            active_to_time = None
            events.append(
                {
                    "type": "HS",
                    "time_s": event_time,
                    "confirmation_index": index,
                    "confirmation_time_s": float(times[index]),
                    "confirmation_latency_s": float(times[index] - event_time),
                    "valid_hs_count": hs_count,
                    "valid_to_count": to_count,
                    "completed_stance_duration_s": completed_stance,
                    "completed_swing_duration_s": completed_swing,
                }
            )

        logged_history_count = _optional_finite(
            terms.get("phase_duration_history_count")
        )
        logged_robust_stance = _optional_finite(
            terms.get("phase_robust_stance_duration_s")
        )
        logged_robust_swing = _optional_finite(
            terms.get("phase_robust_swing_duration_s")
        )
        reconstructed_count = min(len(stance_history), len(swing_history))
        if (
            logged_history_count is not None
            and logged_history_count > 0.0
            and logged_robust_stance is not None
            and logged_robust_stance > 0.0
            and logged_robust_swing is not None
            and logged_robust_swing > 0.0
        ):
            history_count = logged_history_count
            robust_stance = logged_robust_stance
            robust_swing = logged_robust_swing
        else:
            history_count = float(reconstructed_count)
            robust_stance = (
                float(np.median(tuple(stance_history))) if stance_history else 0.0
            )
            robust_swing = (
                float(np.median(tuple(swing_history))) if swing_history else 0.0
            )

        payloads.append(
            {
                "state_id": _finite_float(
                    terms.get("phase_fsm_state_id", 0.0),
                    context=f"trace[{index}].phase_fsm_state_id",
                ),
                "stance_elapsed_s": max(
                    0.0,
                    _finite_float(
                        terms.get("phase_stance_elapsed_s", 0.0),
                        context=f"trace[{index}].phase_stance_elapsed_s",
                    ),
                ),
                "swing_elapsed_s": max(
                    0.0,
                    _finite_float(
                        terms.get("phase_swing_elapsed_s", 0.0),
                        context=f"trace[{index}].phase_swing_elapsed_s",
                    ),
                ),
                "valid_cycle_count": _finite_float(
                    terms.get("phase_valid_cycle_count", 0.0),
                    context=f"trace[{index}].phase_valid_cycle_count",
                ),
                "last_period_s": _finite_float(
                    terms.get("phase_last_period_s", 0.0),
                    context=f"trace[{index}].phase_last_period_s",
                ),
                "last_stance_fraction": _finite_float(
                    terms.get("phase_last_stance_fraction", 0.0),
                    context=f"trace[{index}].phase_last_stance_fraction",
                ),
                "robust_stance_duration_s": robust_stance,
                "robust_swing_duration_s": robust_swing,
                "duration_history_count": history_count,
                "timeout_side": _finite_float(
                    terms.get("phase_timeout_side", 0.0),
                    context=f"trace[{index}].phase_timeout_side",
                ),
            }
        )
        previous_hs_count = hs_count
        previous_to_count = to_count

    return payloads, events


def _legacy_phase(
    rows: Sequence[Mapping[str, Any]],
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    phase = np.full(len(rows), np.nan, dtype=float)
    available = np.zeros(len(rows), dtype=bool)
    source = np.zeros(len(rows), dtype=float)
    for index, row in enumerate(rows):
        terms = row["reward_terms"]
        availability = _optional_finite(terms.get("morphology_available"))
        value = _optional_finite(terms.get("morphology_phase"))
        source_value = _optional_finite(terms.get("morphology_phase_source_id"))
        if availability is not None and availability > 0.5 and value is not None:
            phase[index] = float(np.clip(value, 0.0, 1.0))
            available[index] = True
        source[index] = float(source_value or 0.0)
    return phase, available, source


def _causal_event_phase(
    payloads: Sequence[Mapping[str, float]],
    cfg: RewardConfig,
    canonical_to_phase: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    phase = np.full(len(payloads), np.nan, dtype=float)
    available = np.zeros(len(payloads), dtype=bool)
    source = np.zeros(len(payloads), dtype=float)
    for index, payload in enumerate(payloads):
        value, is_available, source_id = _fsm_morphology_phase(
            {"phase_fsm": payload},
            cfg,
            canonical_to_phase=canonical_to_phase,
        )
        if value is not None and is_available > 0.5:
            phase[index] = value
            available[index] = True
        source[index] = source_id
    return phase, available, source


def _complete_cycles(events: Sequence[Mapping[str, Any]]) -> list[dict[str, float]]:
    cycles: list[dict[str, float]] = []
    for start_index in range(len(events) - 2):
        first, second, third = events[start_index : start_index + 3]
        if first["type"] != "HS" or second["type"] != "TO" or third["type"] != "HS":
            continue
        hs_start = float(first["time_s"])
        toe_off = float(second["time_s"])
        hs_end = float(third["time_s"])
        if hs_start < toe_off < hs_end:
            cycles.append(
                {
                    "hs_start_s": hs_start,
                    "toe_off_s": toe_off,
                    "hs_end_s": hs_end,
                    "stance_duration_s": toe_off - hs_start,
                    "swing_duration_s": hs_end - toe_off,
                    "period_s": hs_end - hs_start,
                }
            )
    return cycles


def _oracle_phase(
    times: np.ndarray,
    cycles: Sequence[Mapping[str, float]],
    canonical_to_phase: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    phase = np.full(times.size, np.nan, dtype=float)
    available = np.zeros(times.size, dtype=bool)
    source = np.zeros(times.size, dtype=float)
    for cycle_index, cycle in enumerate(cycles):
        hs_start = float(cycle["hs_start_s"])
        toe_off = float(cycle["toe_off_s"])
        hs_end = float(cycle["hs_end_s"])
        final_cycle = cycle_index == len(cycles) - 1
        mask = (times >= hs_start) & (
            (times <= hs_end) if final_cycle else (times < hs_end)
        )
        stance = mask & (times <= toe_off)
        swing = mask & (times > toe_off)
        phase[stance] = canonical_to_phase * (
            (times[stance] - hs_start) / max(1e-12, toe_off - hs_start)
        )
        phase[swing] = canonical_to_phase + (1.0 - canonical_to_phase) * (
            (times[swing] - toe_off) / max(1e-12, hs_end - toe_off)
        )
        available[mask] = True
        source[mask] = 9.0
    phase[available] = np.clip(phase[available], 0.0, 1.0)
    return phase, available, source


def _evaluate_scheme(
    *,
    name: str,
    label: str,
    profile: Mapping[str, Any],
    phase: np.ndarray,
    available: np.ndarray,
    source_id: np.ndarray,
    cfg: RewardConfig,
    served: Mapping[str, np.ndarray],
) -> SchemeSeries:
    count = phase.size
    low = {joint.short: np.full(count, np.nan) for joint in JOINTS}
    high = {joint.short: np.full(count, np.nan) for joint in JOINTS}
    excursion = {joint.short: np.full(count, np.nan) for joint in JOINTS}
    joint_loss = {joint.short: np.full(count, np.nan) for joint in JOINTS}
    loss = np.full(count, np.nan)

    for index in np.flatnonzero(available):
        corridor = _morphology_corridor_at(profile, float(phase[index]), cfg)
        current_losses: list[float] = []
        for joint in JOINTS:
            band = corridor[joint.coord]
            low[joint.short][index] = band["min_rad"]
            high[joint.short][index] = band["max_rad"]
            value_loss, _, value_excursion = _morphology_interval_losses(
                float(served[joint.short][index]),
                band["min_rad"],
                band["max_rad"],
            )
            excursion[joint.short][index] = value_excursion
            joint_loss[joint.short][index] = value_loss
            current_losses.append(value_loss)
        loss[index] = 0.5 * sum(current_losses)

    return SchemeSeries(
        name=name,
        label=label,
        profile=profile,
        phase=phase,
        available=available,
        source_id=source_id,
        low=low,
        high=high,
        excursion=excursion,
        joint_loss=joint_loss,
        loss=loss,
    )


def _scheme_metrics(
    scheme: SchemeSeries,
    served: Mapping[str, np.ndarray],
    common_mask: np.ndarray,
) -> dict[str, Any]:
    mask = scheme.available
    count = int(np.count_nonzero(mask))
    total = int(mask.size)
    joint_metrics: dict[str, Any] = {}
    joint_inside: list[np.ndarray] = []
    for joint in JOINTS:
        excursion = scheme.excursion[joint.short]
        inside = mask & (excursion <= 1e-15)
        joint_inside.append(inside)
        outside_values = excursion[mask & ~inside]
        joint_metrics[joint.short] = {
            "inside_count": int(np.count_nonzero(inside)),
            "inside_fraction": float(np.mean(inside[mask])) if count else None,
            "outside_count": int(np.count_nonzero(mask & ~inside)),
            "mean_excursion_rad": _safe_mean(excursion[mask]),
            "mean_outside_excursion_rad": _safe_mean(outside_values),
            "p95_excursion_rad": _percentile(excursion[mask], 95.0),
            "max_excursion_rad": _safe_max(excursion[mask]),
            "served_min_rad": (
                float(np.min(served[joint.short][mask])) if count else None
            ),
            "served_max_rad": (
                float(np.max(served[joint.short][mask])) if count else None
            ),
            "loss_sum": _ordered_sum(scheme.joint_loss[joint.short][mask]),
            "loss_mean": _safe_mean(scheme.joint_loss[joint.short][mask]),
            "loss_p95": _percentile(scheme.joint_loss[joint.short][mask], 95.0),
            "loss_max": _safe_max(scheme.joint_loss[joint.short][mask]),
        }
    both_inside = joint_inside[0] & joint_inside[1]
    common_available = common_mask & mask
    return {
        "coverage_count": count,
        "coverage_fraction": float(count / total),
        "source_counts": {
            str(int(source)): int(np.count_nonzero(mask & (scheme.source_id == source)))
            for source in np.unique(scheme.source_id[mask])
        },
        "both_joints_inside_count": int(np.count_nonzero(both_inside)),
        "both_joints_inside_fraction": (
            float(np.mean(both_inside[mask])) if count else None
        ),
        "morphology_loss_sum": _ordered_sum(scheme.loss[mask]),
        "morphology_loss_mean": _safe_mean(scheme.loss[mask]),
        "morphology_loss_p95": _percentile(scheme.loss[mask], 95.0),
        "morphology_loss_max": _safe_max(scheme.loss[mask]),
        "common_support_count": int(np.count_nonzero(common_available)),
        "common_support_loss_sum": _ordered_sum(scheme.loss[common_available]),
        "common_support_loss_mean": _safe_mean(scheme.loss[common_available]),
        "common_support_loss_p95": _percentile(scheme.loss[common_available], 95.0),
        "common_support_loss_max": _safe_max(scheme.loss[common_available]),
        "joints": joint_metrics,
    }


def _logged_legacy_errors(
    rows: Sequence[Mapping[str, Any]], legacy: SchemeSeries
) -> dict[str, float | None]:
    mask = legacy.available
    errors: dict[str, float | None] = {}
    for joint in JOINTS:
        logged_low = np.asarray(
            [
                _optional_finite(row["reward_terms"].get(joint.logged_min_key))
                for row in rows
            ],
            dtype=float,
        )
        logged_high = np.asarray(
            [
                _optional_finite(row["reward_terms"].get(joint.logged_max_key))
                for row in rows
            ],
            dtype=float,
        )
        logged_loss = np.asarray(
            [
                _optional_finite(row["reward_terms"].get(joint.logged_loss_key))
                for row in rows
            ],
            dtype=float,
        )
        errors[f"{joint.short}_low_max_abs_error_rad"] = _safe_max(
            np.abs(logged_low[mask] - legacy.low[joint.short][mask])
        )
        errors[f"{joint.short}_high_max_abs_error_rad"] = _safe_max(
            np.abs(logged_high[mask] - legacy.high[joint.short][mask])
        )
        errors[f"{joint.short}_loss_max_abs_error"] = _safe_max(
            np.abs(logged_loss[mask] - legacy.joint_loss[joint.short][mask])
        )
    logged_combined = np.asarray(
        [_optional_finite(row["reward_terms"].get("morphology_loss")) for row in rows],
        dtype=float,
    )
    errors["combined_loss_max_abs_error"] = _safe_max(
        np.abs(logged_combined[mask] - legacy.loss[mask])
    )
    return errors


def _phase_only_jump(
    scheme: SchemeSeries,
    before_index: int,
    after_index: int,
    served: Mapping[str, np.ndarray],
    cfg: RewardConfig,
) -> dict[str, float]:
    before_phase = float(scheme.phase[before_index])
    after_phase = float(scheme.phase[after_index])
    before_corridor = _morphology_corridor_at(scheme.profile, before_phase, cfg)
    after_corridor = _morphology_corridor_at(scheme.profile, after_phase, cfg)
    max_band_jump = 0.0
    max_loss_jump = 0.0
    for joint in JOINTS:
        before_band = before_corridor[joint.coord]
        after_band = after_corridor[joint.coord]
        max_band_jump = max(
            max_band_jump,
            abs(before_band["min_rad"] - after_band["min_rad"]),
            abs(before_band["max_rad"] - after_band["max_rad"]),
        )
        value = float(served[joint.short][after_index])
        loss_before = _morphology_interval_losses(
            value, before_band["min_rad"], before_band["max_rad"]
        )[0]
        loss_after = _morphology_interval_losses(
            value, after_band["min_rad"], after_band["max_rad"]
        )[0]
        max_loss_jump = max(max_loss_jump, abs(loss_after - loss_before))
    return {
        "max_corridor_bound_jump_rad": float(max_band_jump),
        "max_same_value_joint_loss_jump": float(max_loss_jump),
    }


def _event_jump_report(
    events: Sequence[Mapping[str, Any]],
    schemes: Sequence[SchemeSeries],
    served: Mapping[str, np.ndarray],
    cfg: RewardConfig,
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    details: list[dict[str, Any]] = []
    summaries: dict[str, Any] = {}
    for event_number, event in enumerate(events):
        index = int(event["confirmation_index"])
        event_row: dict[str, Any] = {
            "event_number": event_number,
            **dict(event),
            "schemes": {},
        }
        for scheme in schemes:
            before = index - 1
            if (
                before < 0
                or not scheme.available[before]
                or not scheme.available[index]
            ):
                event_row["schemes"][scheme.name] = {"available": False}
                continue
            raw_delta = float(scheme.phase[index] - scheme.phase[before])
            circular_jump = float(
                _circular_distance(scheme.phase[index], scheme.phase[before])
            )
            phase_only = _phase_only_jump(scheme, before, index, served, cfg)
            event_row["schemes"][scheme.name] = {
                "available": True,
                "phase_before": float(scheme.phase[before]),
                "phase_after": float(scheme.phase[index]),
                "raw_phase_delta": raw_delta,
                "circular_phase_jump": circular_jump,
                **phase_only,
            }
        details.append(event_row)

    for scheme in schemes:
        comparable = [
            event["schemes"][scheme.name]
            for event in details
            if event["schemes"][scheme.name].get("available")
        ]
        summaries[scheme.name] = {
            "comparable_event_count": len(comparable),
            "max_circular_phase_jump": max(
                (item["circular_phase_jump"] for item in comparable),
                default=None,
            ),
            "max_corridor_bound_jump_rad": max(
                (item["max_corridor_bound_jump_rad"] for item in comparable),
                default=None,
            ),
            "max_same_value_joint_loss_jump": max(
                (item["max_same_value_joint_loss_jump"] for item in comparable),
                default=None,
            ),
        }
    return details, summaries


def _shadow_rewards(
    rows: Sequence[Mapping[str, Any]],
    summary: Mapping[str, Any],
    cfg: RewardConfig,
    schemes: Sequence[SchemeSeries],
    weights: Sequence[float],
    common_mask: np.ndarray,
) -> tuple[dict[str, Any], dict[str, Any]]:
    baseline_rewards = np.asarray(
        [
            compute_reward(
                row["reward_terms"], cfg, reference=row.get("policy_segment_values")
            )[0]
            for row in rows
        ],
        dtype=float,
    )
    baseline_return = _ordered_sum(baseline_rewards)
    reported_return = _finite_float(
        summary.get("episode_return"), context="summary.episode_return"
    )
    zero_cfg = replace(cfg, morphology_weight=0.0)
    original_zero_rewards = np.asarray(
        [
            compute_reward(
                row["reward_terms"],
                zero_cfg,
                reference=row.get("policy_segment_values"),
            )[0]
            for row in rows
        ],
        dtype=float,
    )

    gates: dict[str, Any] = {
        "rollout_morphology_weight_is_zero": {
            "pass": float(cfg.morphology_weight) == 0.0,
            "value": float(cfg.morphology_weight),
        },
        "runtime_reward_reconstruction_exact": {
            "pass": baseline_return == reported_return,
            "reconstructed_return": baseline_return,
            "reported_return": reported_return,
            "absolute_difference": abs(baseline_return - reported_return),
        },
        "original_weight_zero_per_step_exact": {
            "pass": bool(np.array_equal(original_zero_rewards, baseline_rewards)),
            "max_abs_difference": float(
                np.max(np.abs(original_zero_rewards - baseline_rewards))
            ),
        },
    }
    shadow: dict[str, Any] = {
        "baseline_return": baseline_return,
        "reported_return": reported_return,
        "weights": {},
    }

    for scheme in schemes:
        scheme_losses = np.where(scheme.available, scheme.loss, 0.0)
        scheme_zero_rewards: list[float] = []
        for row, loss in zip(rows, scheme_losses):
            terms = dict(row["reward_terms"])
            terms["morphology_loss"] = float(loss)
            scheme_zero_rewards.append(
                compute_reward(
                    terms,
                    zero_cfg,
                    reference=row.get("policy_segment_values"),
                )[0]
            )
        zero_array = np.asarray(scheme_zero_rewards, dtype=float)
        gate_name = f"{scheme.name}_weight_zero_per_step_exact"
        gates[gate_name] = {
            "pass": bool(np.array_equal(zero_array, original_zero_rewards)),
            "max_abs_difference": float(
                np.max(np.abs(zero_array - original_zero_rewards))
            ),
            "return_difference": _ordered_sum(zero_array)
            - _ordered_sum(original_zero_rewards),
        }

        entries: dict[str, Any] = {}
        for weight in weights:
            weighted_cfg = replace(cfg, morphology_weight=float(weight))
            weighted_rewards: list[float] = []
            for row, loss in zip(rows, scheme_losses):
                terms = dict(row["reward_terms"])
                terms["morphology_loss"] = float(loss)
                weighted_rewards.append(
                    compute_reward(
                        terms,
                        weighted_cfg,
                        reference=row.get("policy_segment_values"),
                    )[0]
                )
            weighted_return = _ordered_sum(weighted_rewards)
            expected_return = baseline_return - float(weight) * _ordered_sum(
                scheme_losses
            )
            entries[f"{weight:.10g}"] = {
                "weight": float(weight),
                "episode_return": weighted_return,
                "return_delta": weighted_return - baseline_return,
                "cumulative_morphology_penalty": float(weight)
                * _ordered_sum(scheme_losses),
                "common_support_penalty": float(weight)
                * _ordered_sum(scheme.loss[common_mask & scheme.available]),
                "max_per_step_morphology_penalty": float(weight)
                * float(np.nanmax(scheme_losses)),
                "postclip_linear_identity_abs_error": abs(
                    weighted_return - expected_return
                ),
            }
        shadow["weights"][scheme.name] = entries

    gates["all_weight_zero_gates"] = {
        "pass": all(
            bool(value["pass"])
            for key, value in gates.items()
            if key != "all_weight_zero_gates"
        )
    }
    return shadow, gates


def _phase_comparison(
    schemes: Sequence[SchemeSeries], oracle: SchemeSeries
) -> dict[str, Any]:
    comparisons: dict[str, Any] = {}
    for scheme in schemes:
        if scheme.name == oracle.name:
            continue
        mask = scheme.available & oracle.available
        error = _circular_distance(scheme.phase[mask], oracle.phase[mask])
        comparisons[f"{scheme.name}_vs_{oracle.name}"] = {
            "sample_count": int(np.count_nonzero(mask)),
            "circular_phase_error_mean": _safe_mean(error),
            "circular_phase_error_p95": _percentile(error, 95.0),
            "circular_phase_error_max": _safe_max(error),
        }
    return comparisons


def _profile_grid(
    profile: Mapping[str, Any], cfg: RewardConfig, joint: JointSpec
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    phase = np.asarray(profile["phase_grid"], dtype=float)
    mean = np.asarray(profile["coordinates"][joint.coord]["mean_rad"], dtype=float)
    low = np.empty_like(phase)
    high = np.empty_like(phase)
    for index, value in enumerate(phase):
        band = _morphology_corridor_at(profile, float(value), cfg)[joint.coord]
        low[index] = band["min_rad"]
        high[index] = band["max_rad"]
    return phase, mean, low, high


def _plot_phase_alignment(
    output_path: Path,
    times: np.ndarray,
    events: Sequence[Mapping[str, Any]],
    schemes: Sequence[SchemeSeries],
    oracle: SchemeSeries,
) -> None:
    colors = {
        "legacy": "#4c78a8",
        "event_anchored_causal": "#f58518",
        "oracle_retrospective": "#54a24b",
    }
    fig, axes = plt.subplots(2, 1, figsize=(14, 9), sharex=True)
    for scheme in schemes:
        axes[0].plot(
            times,
            scheme.phase,
            label=scheme.label,
            color=colors[scheme.name],
            linewidth=1.45,
        )
    for event in events:
        color = "#7f3c8d" if event["type"] == "TO" else "#111111"
        linestyle = ":" if event["type"] == "TO" else "--"
        axes[0].axvline(
            float(event["time_s"]), color=color, linestyle=linestyle, alpha=0.23
        )
    axes[0].set_ylabel("Morphology phase [cycle]")
    axes[0].set_ylim(-0.03, 1.03)
    axes[0].set_title("Legacy, causal event-anchored and retrospective FSM oracle")
    axes[0].grid(alpha=0.25)
    axes[0].legend(loc="upper right")

    for scheme in schemes:
        if scheme.name == oracle.name:
            continue
        mask = scheme.available & oracle.available
        error = np.full(times.size, np.nan)
        error[mask] = _circular_distance(scheme.phase[mask], oracle.phase[mask])
        axes[1].plot(
            times,
            error,
            label=f"{scheme.label} vs oracle",
            color=colors[scheme.name],
            linewidth=1.35,
        )
    axes[1].set_xlabel("Simulation time [s]")
    axes[1].set_ylabel("Circular phase error [cycle]")
    axes[1].grid(alpha=0.25)
    axes[1].legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(output_path, dpi=180, bbox_inches="tight")
    plt.close(fig)


def _plot_corridor_coverage(
    output_path: Path,
    schemes: Sequence[SchemeSeries],
    served: Mapping[str, np.ndarray],
    cfg: RewardConfig,
    metrics: Mapping[str, Any],
    canonical_to_phase: float,
) -> None:
    fig, axes = plt.subplots(2, 3, figsize=(18, 10), sharex=True)
    for column, scheme in enumerate(schemes):
        for row_index, joint in enumerate(JOINTS):
            axis = axes[row_index, column]
            phase, mean, low, high = _profile_grid(scheme.profile, cfg, joint)
            display_low = np.minimum(
                joint.display_sign * low, joint.display_sign * high
            )
            display_high = np.maximum(
                joint.display_sign * low, joint.display_sign * high
            )
            axis.fill_between(
                phase * 100.0,
                np.rad2deg(display_low),
                np.rad2deg(display_high),
                color="#9ecae1",
                alpha=0.55,
                linewidth=0.0,
                label="corridor",
            )
            axis.plot(
                phase * 100.0,
                np.rad2deg(joint.display_sign * mean),
                color="#37474f",
                linestyle="--",
                linewidth=1.25,
                label="profile mean",
            )
            mask = scheme.available
            outside = mask & (scheme.excursion[joint.short] > 1e-15)
            inside = mask & ~outside
            axis.scatter(
                scheme.phase[inside] * 100.0,
                np.rad2deg(joint.display_sign * served[joint.short][inside]),
                s=8,
                color="#2ca02c",
                alpha=0.48,
                linewidths=0.0,
                label="served inside",
            )
            axis.scatter(
                scheme.phase[outside] * 100.0,
                np.rad2deg(joint.display_sign * served[joint.short][outside]),
                s=10,
                marker="x",
                color="#d62728",
                alpha=0.65,
                linewidths=0.65,
                label="served outside",
            )
            if scheme.name != "legacy":
                axis.axvline(
                    canonical_to_phase * 100.0,
                    color="#7f3c8d",
                    linestyle=":",
                    linewidth=1.1,
                    alpha=0.8,
                    label="fixed TO anchor",
                )
            joint_metric = metrics[scheme.name]["joints"][joint.short]
            inside_fraction = joint_metric["inside_fraction"]
            axis.set_title(
                f"{scheme.label}\n{joint.title}: inside {inside_fraction:.1%}"
                if inside_fraction is not None
                else f"{scheme.label}\n{joint.title}: no samples"
            )
            axis.grid(alpha=0.2)
            if column == 0:
                axis.set_ylabel("Angle [deg]")
            if row_index == 1:
                axis.set_xlabel("Morphology phase [%]")
            if row_index == 0 and column == 0:
                axis.legend(loc="best", fontsize=8)
    fig.suptitle("Served trajectories against morphology corridors", fontsize=17)
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    fig.savefig(output_path, dpi=180, bbox_inches="tight")
    plt.close(fig)


def _format_value(value: Any, digits: int = 6) -> str:
    if value is None:
        return "n/a"
    if isinstance(value, bool):
        return "PASS" if value else "FAIL"
    if isinstance(value, int):
        return str(value)
    if isinstance(value, float):
        return f"{value:.{digits}g}"
    return str(value)


def _markdown_report(report: Mapping[str, Any]) -> str:
    lines = [
        "# Validation of event-anchored morphology",
        "",
        "This is an offline audit. The retrospective oracle uses the next HS and must not be used by a deployed policy or online reward.",
        "",
        "## Inputs",
        "",
        "| Input | Path | SHA-256 |",
        "|---|---|---|",
    ]
    for name, item in report["inputs"].items():
        lines.append(f"| {name} | `{item['path']}` | `{item['sha256']}` |")

    lines.extend(
        [
            "",
            "## Phase contracts",
            "",
            f"Fixed event anchor: alpha = **{report['phase_contract']['canonical_to_phase']:.9f}** at valid prosthetic TO.",
            "",
            "- Legacy: phase recorded in the original trace and legacy profile.",
            "- Causal event-anchored: clipped stance/swing progress; nominal timing before the first complete cycle, then past-only robust medians reconstructed with the runtime window.",
            "- Retrospective oracle: exact durations of complete valid FSM HS-TO-HS cycles and the event-warped profile.",
            "",
            f"Recovered events: {report['events']['hs_count']} HS, {report['events']['to_count']} TO, {report['events']['complete_cycle_count']} complete cycles.",
            "",
            "## Coverage and corridor loss",
            "",
            "| Scheme | Coverage | Both inside | Loss sum | Loss mean | Loss p95 | Knee max excursion [rad] | Ankle max excursion [rad] |",
            "|---|---:|---:|---:|---:|---:|---:|---:|",
        ]
    )
    for name in ("legacy", "event_anchored_causal", "oracle_retrospective"):
        metric = report["metrics"][name]
        lines.append(
            "| {label} | {coverage:.1%} | {inside:.1%} | {loss_sum} | {loss_mean} | {loss_p95} | {knee} | {ankle} |".format(
                label=report["scheme_labels"][name],
                coverage=metric["coverage_fraction"],
                inside=metric["both_joints_inside_fraction"] or 0.0,
                loss_sum=_format_value(metric["morphology_loss_sum"]),
                loss_mean=_format_value(metric["morphology_loss_mean"]),
                loss_p95=_format_value(metric["morphology_loss_p95"]),
                knee=_format_value(metric["joints"]["knee"]["max_excursion_rad"]),
                ankle=_format_value(metric["joints"]["ankle"]["max_excursion_rad"]),
            )
        )

    lines.extend(
        [
            "",
            f"Common support: **{report['common_support']['count']} / {report['trace']['sample_count']}** samples ({report['common_support']['fraction']:.1%}).",
            "",
            "## Event continuity",
            "",
            "| Scheme | Comparable events | Max circular phase jump | Max corridor-bound jump [rad] | Max same-value joint-loss jump |",
            "|---|---:|---:|---:|---:|",
        ]
    )
    for name in ("legacy", "event_anchored_causal", "oracle_retrospective"):
        item = report["event_jumps"]["summary"][name]
        lines.append(
            f"| {report['scheme_labels'][name]} | {item['comparable_event_count']} | {_format_value(item['max_circular_phase_jump'])} | {_format_value(item['max_corridor_bound_jump_rad'])} | {_format_value(item['max_same_value_joint_loss_jump'])} |"
        )

    lines.extend(
        [
            "",
            "## Shadow reward",
            "",
            "Unavailable morphology samples contribute zero, matching the runtime contract. Penalties on common support are also reported in JSON.",
            "",
            "| Scheme | Weight | Cumulative penalty | Shadow return | Delta | Max step penalty |",
            "|---|---:|---:|---:|---:|---:|",
        ]
    )
    for name in ("legacy", "event_anchored_causal", "oracle_retrospective"):
        for entry in report["shadow_reward"]["weights"][name].values():
            lines.append(
                f"| {report['scheme_labels'][name]} | {entry['weight']:.4g} | {_format_value(entry['cumulative_morphology_penalty'])} | {_format_value(entry['episode_return'])} | {_format_value(entry['return_delta'])} | {_format_value(entry['max_per_step_morphology_penalty'])} |"
            )

    lines.extend(
        [
            "",
            "## Validation gates",
            "",
            "| Gate | Result | Max/absolute difference |",
            "|---|---:|---:|",
        ]
    )
    for name, gate in report["gates"].items():
        difference = gate.get(
            "max_abs_difference",
            gate.get(
                "absolute_difference",
                gate.get(
                    "value",
                    gate.get(
                        "minimum_raw_to_phase_delta",
                        gate.get(
                            "causal_p95",
                            gate.get(
                                "reduction_fraction",
                                gate.get("causal_max_rad"),
                            ),
                        ),
                    ),
                ),
            ),
        )
        lines.append(
            f"| `{name}` | {_format_value(bool(gate['pass']))} | {_format_value(difference, 12)} |"
        )
    lines.extend(
        [
            "",
            f"Overall gate: **{'PASS' if report['gate_pass'] else 'FAIL'}**.",
            "",
            "## Figures",
            "",
            "- `phase_alignment.png`: causal phase and oracle error over simulation time.",
            "- `corridor_coverage.png`: served knee/ankle references against each evaluated corridor.",
            "",
        ]
    )
    return "\n".join(lines)


def _sanitize_json(value: Any) -> Any:
    if isinstance(value, Mapping):
        return {str(key): _sanitize_json(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_sanitize_json(item) for item in value]
    if isinstance(value, np.generic):
        value = value.item()
    if isinstance(value, float) and not np.isfinite(value):
        return None
    return value


def run_audit(args: argparse.Namespace) -> tuple[dict[str, Any], dict[str, Path]]:
    rollout_dir = _resolve_cli_path(args.rollout_dir)
    trace_path = _require_file(
        args.trace or rollout_dir / "rollout_policy_trace.json", "rollout trace"
    )
    summary_path = _require_file(
        args.summary or rollout_dir / "rollout_summary.json", "rollout summary"
    )
    legacy_profile_path = _require_file(args.legacy_profile, "legacy profile")
    event_profile_path = _require_file(args.event_profile, "event-warped profile")
    output_dir = _resolve_cli_path(args.output_dir)
    output_paths = {name: output_dir / name for name in OUTPUT_NAMES}
    if args.no_clobber:
        existing = [path for path in output_paths.values() if path.exists()]
        if existing:
            raise FileExistsError(
                "Refusing to overwrite existing outputs: "
                + ", ".join(str(path) for path in existing)
            )
    output_dir.mkdir(parents=True, exist_ok=True)

    rows, summary, steps, times = _load_rollout(trace_path, summary_path)
    reward_mapping = summary.get("reward_config")
    if not isinstance(reward_mapping, Mapping):
        raise ValueError("rollout_summary.json has no reward_config mapping")
    cfg = RewardConfig.from_mapping(reward_mapping)
    event_cfg = replace(cfg, morphology_phase_mode="event_anchored")
    legacy_profile = _load_morphology_profile(str(legacy_profile_path))
    event_profile = _load_morphology_profile(str(event_profile_path))
    if legacy_profile is None or event_profile is None:
        raise RuntimeError("Both morphology profiles must be enabled")
    canonical_to_phase = _morphology_canonical_to_phase(event_profile)

    served = _served_values(rows)
    payloads, events = _recover_pseudo_fsm(
        rows, times, history_window=args.history_window
    )
    cycles = _complete_cycles(events)
    legacy_phase, legacy_available, legacy_source = _legacy_phase(rows)
    event_phase, event_available, event_source = _causal_event_phase(
        payloads, event_cfg, canonical_to_phase
    )
    oracle_phase, oracle_available, oracle_source = _oracle_phase(
        times, cycles, canonical_to_phase
    )

    schemes = [
        _evaluate_scheme(
            name="legacy",
            label="Legacy logged phase",
            profile=legacy_profile,
            phase=legacy_phase,
            available=legacy_available,
            source_id=legacy_source,
            cfg=cfg,
            served=served,
        ),
        _evaluate_scheme(
            name="event_anchored_causal",
            label="Event-anchored causal",
            profile=event_profile,
            phase=event_phase,
            available=event_available,
            source_id=event_source,
            cfg=cfg,
            served=served,
        ),
        _evaluate_scheme(
            name="oracle_retrospective",
            label="FSM oracle retrospective",
            profile=event_profile,
            phase=oracle_phase,
            available=oracle_available,
            source_id=oracle_source,
            cfg=cfg,
            served=served,
        ),
    ]
    common_mask = np.logical_and.reduce([scheme.available for scheme in schemes])
    metrics = {
        scheme.name: _scheme_metrics(scheme, served, common_mask) for scheme in schemes
    }
    legacy_reconstruction = _logged_legacy_errors(rows, schemes[0])
    event_details, event_summary = _event_jump_report(events, schemes, served, cfg)
    shadow, gates = _shadow_rewards(
        rows, summary, cfg, schemes, args.weights, common_mask
    )
    phase_comparison = _phase_comparison(schemes, schemes[2])
    legacy_vs_oracle = phase_comparison[
        "legacy_vs_oracle_retrospective"
    ]
    causal_vs_oracle = phase_comparison[
        "event_anchored_causal_vs_oracle_retrospective"
    ]
    causal_to_deltas = [
        item["schemes"]["event_anchored_causal"]["raw_phase_delta"]
        for item in event_details
        if item["type"] == "TO"
        and item["schemes"]["event_anchored_causal"].get("available")
    ]
    common_support_fraction = float(np.mean(common_mask))
    legacy_common_loss = float(metrics["legacy"]["common_support_loss_sum"])
    causal_common_loss = float(
        metrics["event_anchored_causal"]["common_support_loss_sum"]
    )
    common_loss_reduction = (
        1.0 - causal_common_loss / legacy_common_loss
        if legacy_common_loss > 0.0
        else 0.0
    )
    mapping_gates = {
        "event_anchored_to_never_regresses": {
            "pass": bool(causal_to_deltas)
            and min(causal_to_deltas) >= -1e-12,
            "minimum_raw_to_phase_delta": min(causal_to_deltas, default=None),
        },
        "causal_phase_p95_improves_vs_legacy": {
            "pass": (
                causal_vs_oracle["circular_phase_error_p95"]
                < legacy_vs_oracle["circular_phase_error_p95"]
                and causal_vs_oracle["circular_phase_error_p95"] <= 0.10
            ),
            "causal_p95": causal_vs_oracle["circular_phase_error_p95"],
            "legacy_p95": legacy_vs_oracle["circular_phase_error_p95"],
            "absolute_ceiling": 0.10,
        },
        "causal_common_support_loss_reduced": {
            "pass": common_loss_reduction >= 0.25,
            "reduction_fraction": common_loss_reduction,
            "minimum_reduction_fraction": 0.25,
        },
        "causal_event_bound_jump_improves_vs_legacy": {
            "pass": (
                event_summary["event_anchored_causal"][
                    "max_corridor_bound_jump_rad"
                ]
                < event_summary["legacy"]["max_corridor_bound_jump_rad"]
            ),
            "causal_max_rad": event_summary["event_anchored_causal"][
                "max_corridor_bound_jump_rad"
            ],
            "legacy_max_rad": event_summary["legacy"][
                "max_corridor_bound_jump_rad"
            ],
        },
        "common_support_is_sufficient": {
            "pass": common_support_fraction >= 0.90,
            "value": common_support_fraction,
            "threshold": 0.90,
        },
    }
    gates.update(mapping_gates)
    gates["all_mapping_gates"] = {
        "pass": all(item["pass"] for item in mapping_gates.values())
    }
    gates["all_gates"] = {
        "pass": bool(gates["all_weight_zero_gates"]["pass"])
        and bool(gates["all_mapping_gates"]["pass"])
    }
    gate_pass = bool(gates["all_gates"]["pass"])

    report: dict[str, Any] = {
        "schema_version": 1,
        "audit": "event_anchored_morphology",
        "inputs": {
            "trace": {"path": str(trace_path), "sha256": _sha256(trace_path)},
            "summary": {"path": str(summary_path), "sha256": _sha256(summary_path)},
            "legacy_profile": {
                "path": str(legacy_profile_path),
                "sha256": _sha256(legacy_profile_path),
            },
            "event_warped_profile": {
                "path": str(event_profile_path),
                "sha256": _sha256(event_profile_path),
            },
        },
        "trace": {
            "sample_count": len(rows),
            "first_step": float(steps[0]),
            "last_step": float(steps[-1]),
            "start_time_s": float(times[0]),
            "end_time_s": float(times[-1]),
        },
        "phase_contract": {
            "canonical_to_phase": canonical_to_phase,
            "history_window_cycles": int(args.history_window),
            "causal_duration_priority": [
                "nominal",
                "last_completed_cycle",
                "robust_median_of_past_complete_cycles",
            ],
            "segment_progress_clipped": True,
            "oracle_uses_future_next_hs": True,
        },
        "scheme_labels": {scheme.name: scheme.label for scheme in schemes},
        "events": {
            "hs_count": sum(event["type"] == "HS" for event in events),
            "to_count": sum(event["type"] == "TO" for event in events),
            "complete_cycle_count": len(cycles),
            "complete_cycles": cycles,
        },
        "common_support": {
            "count": int(np.count_nonzero(common_mask)),
            "fraction": common_support_fraction,
        },
        "metrics": metrics,
        "legacy_runtime_reconstruction": legacy_reconstruction,
        "phase_comparison": phase_comparison,
        "event_jumps": {"summary": event_summary, "events": event_details},
        "shadow_reward": shadow,
        "gates": gates,
        "gate_pass": gate_pass,
    }
    report = _sanitize_json(report)

    _plot_phase_alignment(
        output_paths["phase_alignment.png"], times, events, schemes, schemes[2]
    )
    _plot_corridor_coverage(
        output_paths["corridor_coverage.png"],
        schemes,
        served,
        cfg,
        metrics,
        canonical_to_phase,
    )
    output_paths["event_anchored_morphology_summary.json"].write_text(
        json.dumps(report, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    output_paths["event_anchored_morphology_summary.md"].write_text(
        _markdown_report(report), encoding="utf-8"
    )
    return report, output_paths


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Compare logged legacy, causal event-anchored and retrospective FSM "
            "oracle morphology phases on a recorded rollout."
        )
    )
    parser.add_argument("--rollout-dir", type=Path, default=DEFAULT_ROLLOUT_DIR)
    parser.add_argument("--trace", type=Path, default=None)
    parser.add_argument("--summary", type=Path, default=None)
    parser.add_argument("--legacy-profile", type=Path, default=DEFAULT_LEGACY_PROFILE)
    parser.add_argument("--event-profile", type=Path, default=DEFAULT_EVENT_PROFILE)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument(
        "--weights",
        type=float,
        nargs="+",
        default=(0.0025, 0.005, 0.01),
        help="Non-negative morphology weights for shadow reward evaluation.",
    )
    parser.add_argument(
        "--history-window",
        type=int,
        default=5,
        help="Past complete cycles retained by the reconstructed robust median.",
    )
    parser.add_argument(
        "--no-clobber",
        action="store_true",
        help="Fail before writing if any expected output already exists.",
    )
    parser.add_argument(
        "--fail-on-gate",
        action="store_true",
        help="Return exit status 2 when an exact weight-zero gate fails.",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = _parser()
    args = parser.parse_args(argv)
    if any(not np.isfinite(weight) or weight < 0.0 for weight in args.weights):
        parser.error("--weights must contain finite non-negative values")
    report, output_paths = run_audit(args)
    print(
        json.dumps(
            {
                "gate_pass": report["gate_pass"],
                "canonical_to_phase": report["phase_contract"]["canonical_to_phase"],
                "complete_cycle_count": report["events"]["complete_cycle_count"],
                "outputs": {name: str(path) for name, path in output_paths.items()},
            },
            indent=2,
        )
    )
    if args.fail_on_gate and not report["gate_pass"]:
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
