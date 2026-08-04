#!/usr/bin/env python3
"""Plot the exact morphology corridor using the V9 two-sensor detector.

The policy trajectory is never regenerated or edited.  The script samples the
diagnostic V9 heel and forefoot spheres on the full states recorded by the
frozen checkpoint-best rollout, replays those two loads through the production
``ProstheticPhaseFSM`` in ``two_sensor`` mode, and time-warps the morphology
corridor only over complete accepted HS-TO-HS cycles.

The detector remains sensor-only: primary load/contact evidence is read from
the rollout's recorded primary online-GRF stream.  Detector event onsets anchor
the retrospective corridor; debounce confirmation times are plotted only as a
causal diagnostic.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import sys
import tempfile
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Mapping, Sequence

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
TRAJECTORY_ROOT = REPO_ROOT / "Trajectory Generator"
BASELINE_DIR = TRAJECTORY_ROOT / "baseline_MLP"
for path in (REPO_ROOT, TRAJECTORY_ROOT):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from online_grf import load_online_grf_profile  # noqa: E402
from output import _read_storage_table  # noqa: E402
from path_resolver import resolve_repo_path  # noqa: E402
from prosthetic_phase_fsm import (  # noqa: E402
    ProstheticPhaseFSM,
    ProstheticPhaseFSMConfig,
)
from setup_io import read_setup_xml  # noqa: E402
from validation.validate_experimental_retrospective_morphology import (  # noqa: E402
    ProfileData,
    _corridor_at,
    load_profile,
)
from validation.validate_online_grf import (  # noqa: E402
    _sample_spheres_from_coordinate_states,
)
from validation.validate_two_sensor_prescribed_replay import (  # noqa: E402
    DEFAULT_SEA_PLUGIN,
    DEFAULT_SETUP,
    _left_sensor_spheres,
    _model_body_weight_n,
    _regional_loads_and_penetrations,
)
from validation.validate_two_sensor_rollout_trace import (  # noqa: E402
    _fsm_overrides_from_summary,
)


REFERENCE_RUN = (
    TRAJECTORY_ROOT
    / "runs"
    / "rollout"
    / "2026-07-15_h0_exact_interleaved_lr5e-7_pilot50_best_deterministic_nominal_recorded"
)
MORPHOLOGY_RUN = (
    TRAJECTORY_ROOT
    / "runs"
    / "rollout"
    / "validation"
    / "event_anchored_morphology_runs"
    / "2026-07-20_weight0_live_rollout_final"
)
DEFAULT_DETECTOR_PROFILE = (
    SCRIPT_DIR
    / "experimental_detector_profiles"
    / "two_sensor_v9_H2p50_X3p25_F79p0_P35p00.json"
)
DEFAULT_V9_MANIFEST = (
    SCRIPT_DIR
    / "two_sensor_timing_placement_sweep_runs"
    / "2026-07-22_ab06_50_100_heel_micro_v9"
    / "manifest.json"
)
DEFAULT_MORPHOLOGY_PROFILE = (
    BASELINE_DIR
    / "morphology_profiles"
    / "ab06_prosthetic_event_warped_mean_std_corridor.json"
)
DEFAULT_OUTPUT_DIR = (
    REPO_ROOT
    / "plot"
    / "07_22_2026_morphology_two_sensor_v9_experimental"
)
DEFAULT_OUTPUT = DEFAULT_OUTPUT_DIR / "01_morphology_corridor_two_sensor_v9_hs_to_hs.png"
DEFAULT_SIDECAR = DEFAULT_OUTPUT_DIR / "01_morphology_corridor_two_sensor_v9_hs_to_hs.json"

V9_CANDIDATE_ID = "H2p50_X3p25_F79p0_P35p00"
SENSOR_ON_THRESHOLD_N = 0.5
SENSOR_OFF_THRESHOLD_N = 0.25
SENSOR_DWELL_S = 0.03
TIME_TOL_S = 2.0e-7
VALUE_TOL = 1.0e-12


@dataclass(frozen=True)
class CompletedCycle:
    cycle_index: int
    heel_strike_s: float
    toe_off_s: float
    next_heel_strike_s: float

    @property
    def period_s(self) -> float:
        return float(self.next_heel_strike_s - self.heel_strike_s)

    @property
    def stance_fraction(self) -> float:
        return float((self.toe_off_s - self.heel_strike_s) / self.period_s)


def _load_json(path: Path) -> Any:
    with path.open("r", encoding="utf-8") as stream:
        return json.load(stream)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _source(path: Path) -> dict[str, str]:
    return {"path": str(path.resolve()), "sha256": _sha256(path)}


def _finite(value: Any, *, label: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label} must be numeric") from exc
    if not math.isfinite(result):
        raise ValueError(f"{label} must be finite")
    return result


def _trace_rows(path: Path) -> list[Mapping[str, Any]]:
    raw = _load_json(path)
    if not isinstance(raw, list) or not raw:
        raise ValueError(f"rollout trace must be a non-empty JSON list: {path}")
    if not all(isinstance(row, Mapping) for row in raw):
        raise ValueError(f"rollout trace contains a non-object row: {path}")
    return list(raw)


def trace_invariance(
    served_rows: Sequence[Mapping[str, Any]],
    state_rows: Sequence[Mapping[str, Any]],
) -> dict[str, Any]:
    """Require the recorded-state rollout to match the plotted served trace."""
    if len(served_rows) != len(state_rows):
        raise ValueError("served and recorded-state traces have different lengths")
    scalar_keys = (
        "pros_knee_angle",
        "pros_ankle_angle",
        "pros_knee_angle_served_ref",
        "pros_ankle_angle_served_ref",
    )
    max_time_error = 0.0
    max_state_error = 0.0
    max_action_error = 0.0
    for index, (served, recorded) in enumerate(zip(served_rows, state_rows)):
        max_time_error = max(
            max_time_error,
            abs(
                _finite(served.get("time"), label=f"served[{index}].time")
                - _finite(recorded.get("time"), label=f"recorded[{index}].time")
            ),
        )
        served_state = served.get("prosthetic_state")
        recorded_state = recorded.get("prosthetic_state")
        if not isinstance(served_state, Mapping) or not isinstance(
            recorded_state, Mapping
        ):
            raise ValueError("both traces require prosthetic_state mappings")
        for key in scalar_keys:
            max_state_error = max(
                max_state_error,
                abs(
                    _finite(served_state.get(key), label=f"served[{index}].{key}")
                    - _finite(
                        recorded_state.get(key), label=f"recorded[{index}].{key}"
                    )
                ),
            )
        for key in ("raw_policy_action", "applied_policy_action"):
            left = np.asarray(served.get(key), dtype=float)
            right = np.asarray(recorded.get(key), dtype=float)
            if left.shape != right.shape or left.ndim != 1:
                raise ValueError(f"trace action shape mismatch for {key}")
            max_action_error = max(
                max_action_error,
                float(np.max(np.abs(left - right))),
            )
    result = {
        "row_count": len(served_rows),
        "max_time_abs_error_s": max_time_error,
        "max_prosthetic_state_abs_error_rad": max_state_error,
        "max_action_abs_error": max_action_error,
        "exact_for_plotted_fields": bool(
            max_time_error <= VALUE_TOL
            and max_state_error <= VALUE_TOL
            and max_action_error <= VALUE_TOL
        ),
    }
    if not result["exact_for_plotted_fields"]:
        raise ValueError(f"frozen rollout invariance failed: {result}")
    return result


def aligned_pre_step_indices(
    trace_times: np.ndarray,
    state_times: np.ndarray,
) -> tuple[np.ndarray, dict[str, Any]]:
    """Align each policy row with its last 1 ms detector sample.

    A policy trace row is emitted at the segment end.  The detector payload in
    that row comes from the final integration sample one native step earlier.
    """
    trace = np.asarray(trace_times, dtype=float)
    states = np.asarray(state_times, dtype=float)
    if trace.ndim != 1 or states.ndim != 1 or trace.size == 0 or states.size < 2:
        raise ValueError("trace/state times must be non-empty 1-D arrays")
    if not np.all(np.isfinite(trace)) or not np.all(np.isfinite(states)):
        raise ValueError("trace/state times must be finite")
    if np.any(np.diff(trace) <= 0.0) or np.any(np.diff(states) <= 0.0):
        raise ValueError("trace/state times must be strictly increasing")
    native_dt = float(np.median(np.diff(states)))
    targets = trace - native_dt
    insertion = np.searchsorted(states, targets, side="left")
    upper = np.clip(insertion, 0, states.size - 1)
    lower = np.clip(insertion - 1, 0, states.size - 1)
    choose_lower = np.abs(states[lower] - targets) <= np.abs(states[upper] - targets)
    indices = np.where(choose_lower, lower, upper).astype(int)
    errors = states[indices] - targets
    if np.any(np.diff(indices) <= 0):
        raise ValueError("trace rows do not map to distinct increasing state samples")
    max_error = float(np.max(np.abs(errors)))
    if max_error > TIME_TOL_S:
        raise ValueError(
            f"trace/state alignment error {max_error:.9g}s exceeds {TIME_TOL_S:g}s"
        )
    lags = trace - states[indices]
    return indices, {
        "method": "nearest saved state to trace_time - native_dt",
        "native_dt_s": native_dt,
        "selected_count": int(indices.size),
        "max_target_alignment_error_s": max_error,
        "lag_s_min": float(np.min(lags)),
        "lag_s_median": float(np.median(lags)),
        "lag_s_max": float(np.max(lags)),
    }


def _find_candidate(value: Any, candidate_id: str) -> Mapping[str, Any] | None:
    if isinstance(value, Mapping):
        if str(value.get("candidate_id", "")) == candidate_id:
            return value
        for item in value.values():
            found = _find_candidate(item, candidate_id)
            if found is not None:
                return found
    elif isinstance(value, list):
        for item in value:
            found = _find_candidate(item, candidate_id)
            if found is not None:
                return found
    return None


def detector_contract(profile_path: Path, manifest_path: Path) -> dict[str, Any]:
    profile = load_online_grf_profile(profile_path, required_sides=("left",))
    roles = _left_sensor_spheres(profile)
    if len(profile.spheres) != 2 or set(roles) != {"left_heel", "left_toe"}:
        raise ValueError("V9 plot requires exactly one heel and one forefoot sphere")
    manifest = _load_json(manifest_path)
    candidate = _find_candidate(manifest, V9_CANDIDATE_ID)
    if candidate is None:
        raise ValueError(f"V9 manifest does not contain {V9_CANDIDATE_ID}")
    coordinates = {
        role: [float(value) for value in sphere.location]
        for role, sphere in roles.items()
    }
    for role, key in (
        ("left_heel", "heel_location_m"),
        ("left_toe", "forefoot_location_m"),
    ):
        raw = candidate.get(key)
        if raw is None:
            raise ValueError(f"V9 manifest candidate is missing {key}")
        if not np.allclose(
            np.asarray(raw, dtype=float),
            np.asarray(coordinates[role], dtype=float),
            rtol=0.0,
            atol=1.0e-12,
        ):
            raise ValueError(f"profile {role} coordinates disagree with V9 manifest")
    return {
        "candidate_id": V9_CANDIDATE_ID,
        "candidate_status": "diagnostic_not_promoted",
        "sphere_count": len(profile.spheres),
        "roles": {role: sphere.name for role, sphere in roles.items()},
        "coordinates_m": coordinates,
        "radii_m": {role: float(sphere.radius) for role, sphere in roles.items()},
        "detector_only": True,
        "generates_grf": False,
    }


def replay_two_sensor_fsm(
    *,
    trace_rows: Sequence[Mapping[str, Any]],
    heel_load_n: np.ndarray,
    toe_load_n: np.ndarray,
    primary_normal_force_n: np.ndarray,
    primary_in_contact: np.ndarray,
    body_weight_n: float,
    summary: Mapping[str, Any],
) -> dict[str, Any]:
    count = len(trace_rows)
    arrays = (
        heel_load_n,
        toe_load_n,
        primary_normal_force_n,
        primary_in_contact,
    )
    if any(np.asarray(value).shape != (count,) for value in arrays):
        raise ValueError("replay inputs must have one sample per trace row")
    config_values = _fsm_overrides_from_summary(summary)
    config_values.update(
        {
            "event_source": "two_sensor",
            "sensor_on_threshold_n": SENSOR_ON_THRESHOLD_N,
            "sensor_off_threshold_n": SENSOR_OFF_THRESHOLD_N,
            "sensor_dwell_s": SENSOR_DWELL_S,
        }
    )
    config = ProstheticPhaseFSMConfig(**config_values)
    fsm = ProstheticPhaseFSM(config)
    accepted: list[dict[str, Any]] = []
    candidates: list[dict[str, Any]] = []
    invalid_steps: list[dict[str, Any]] = []
    heel_latch = np.zeros(count, dtype=float)
    toe_latch = np.zeros(count, dtype=float)
    state_id = np.zeros(count, dtype=float)

    for index, row in enumerate(trace_rows):
        time_s = _finite(row.get("time"), label=f"trace[{index}].time")
        state = row.get("prosthetic_state")
        if not isinstance(state, Mapping):
            raise ValueError(f"trace[{index}] has no prosthetic_state")
        payload = fsm.update(
            time_s=time_s,
            events=(),
            normal_force_bw=float(primary_normal_force_n[index] / body_weight_n),
            in_contact=bool(primary_in_contact[index] >= 0.5),
            prosthetic_knee_angle_rad=_finite(
                state.get("pros_knee_angle"), label=f"trace[{index}].knee"
            ),
            prosthetic_ankle_angle_rad=_finite(
                state.get("pros_ankle_angle"), label=f"trace[{index}].ankle"
            ),
            heel_normal_force_n=float(heel_load_n[index]),
            toe_normal_force_n=float(toe_load_n[index]),
        )
        heel_latch[index] = float(payload.get("sensor_heel_contact", 0.0))
        toe_latch[index] = float(payload.get("sensor_toe_contact", 0.0))
        state_id[index] = float(payload.get("state_id", 0.0))
        step_candidates = [
            {**dict(item), "confirmed_time_s": time_s}
            for item in payload.get("sensor_events_this_step", [])
            if isinstance(item, Mapping)
        ]
        candidates.extend(step_candidates)
        for transition_raw in payload.get("accepted_transitions_this_step", []):
            if not isinstance(transition_raw, Mapping):
                continue
            transition = dict(transition_raw)
            transition["confirmed_time_s"] = time_s
            transition["trace_row_index"] = index
            transition["matched_sensor_candidate"] = any(
                str(candidate.get("event")) == str(transition.get("event"))
                and abs(
                    float(candidate.get("time", math.inf))
                    - float(transition.get("event_time_s", -math.inf))
                )
                <= 1.0e-10
                for candidate in step_candidates
            )
            accepted.append(transition)
        if float(payload.get("invalid_event_this_step", 0.0)) != 0.0:
            invalid_steps.append(
                {
                    "trace_row_index": index,
                    "time_s": time_s,
                    "type": str(payload.get("invalid_event_type", "")),
                }
            )

    return {
        "config": config,
        "accepted": accepted,
        "candidates": candidates,
        "invalid_steps": invalid_steps,
        "heel_latch": heel_latch,
        "toe_latch": toe_latch,
        "state_id": state_id,
        "final_payload": fsm.payload(),
    }


def complete_cycles(accepted: Sequence[Mapping[str, Any]]) -> list[CompletedCycle]:
    """Extract only accepted, valid HS-TO-HS cycles; ignore partial bootstrap."""
    active_hs: float | None = None
    active_to: float | None = None
    cycles: list[CompletedCycle] = []
    previous_time = -math.inf
    for transition in accepted:
        event = str(transition.get("event", "")).strip().lower()
        if event not in {"heel_strike", "toe_off"}:
            continue
        event_time = _finite(transition.get("event_time_s"), label="event_time_s")
        if event_time < previous_time - VALUE_TOL:
            raise ValueError("accepted transition timestamps are not monotonic")
        previous_time = event_time
        segment_valid = float(transition.get("segment_valid", 1.0)) > 0.5
        if event == "toe_off":
            if active_hs is not None and event_time > active_hs and segment_valid:
                active_to = event_time
            continue

        if active_hs is not None and active_to is not None:
            cycle_valid = float(transition.get("cycle_valid", 0.0)) > 0.5
            if cycle_valid and active_hs < active_to < event_time:
                cycles.append(
                    CompletedCycle(
                        cycle_index=len(cycles),
                        heel_strike_s=active_hs,
                        toe_off_s=active_to,
                        next_heel_strike_s=event_time,
                    )
                )
        active_hs = event_time
        active_to = None
    return cycles


def _merged_cycle_intervals(cycles: Sequence[CompletedCycle]) -> list[tuple[float, float]]:
    intervals: list[tuple[float, float]] = []
    for cycle in cycles:
        start = cycle.heel_strike_s
        end = cycle.next_heel_strike_s
        if intervals and start <= intervals[-1][1] + VALUE_TOL:
            intervals[-1] = (intervals[-1][0], max(intervals[-1][1], end))
        else:
            intervals.append((start, end))
    return intervals


def _plot(
    *,
    trace_rows: Sequence[Mapping[str, Any]],
    cycles: Sequence[CompletedCycle],
    accepted: Sequence[Mapping[str, Any]],
    profile: ProfileData,
    reward_config: Mapping[str, Any],
    output_path: Path,
) -> None:
    times = np.asarray([float(row["time"]) for row in trace_rows], dtype=float)
    knee = np.rad2deg(
        [float(row["prosthetic_state"]["pros_knee_angle_served_ref"]) for row in trace_rows]
    )
    ankle = np.rad2deg(
        [float(row["prosthetic_state"]["pros_ankle_angle_served_ref"]) for row in trace_rows]
    )
    knee_multiplier = float(reward_config["morphology_std_multiplier_knee"])
    ankle_multiplier = float(reward_config["morphology_std_multiplier_ankle"])
    knee_margin_deg = float(reward_config["morphology_margin_knee_deg"])
    ankle_margin_deg = float(reward_config["morphology_margin_ankle_deg"])

    figure, axes = plt.subplots(2, 1, figsize=(15, 9), sharex=True)
    specs = (
        (axes[0], "pros_knee_angle", knee, "Prosthetic knee — raw OpenSim sign"),
        (
            axes[1],
            "pros_ankle_angle",
            ankle,
            "Prosthetic ankle — raw OpenSim sign (positive = dorsiflexion)",
        ),
    )
    complete_intervals = _merged_cycle_intervals(cycles)
    gaps: list[tuple[float, float]] = []
    cursor = float(times[0])
    for start, end in complete_intervals:
        start = max(start, float(times[0]))
        end = min(end, float(times[-1]))
        if start > cursor + VALUE_TOL:
            gaps.append((cursor, start))
        cursor = max(cursor, end)
    if cursor < float(times[-1]) - VALUE_TOL:
        gaps.append((cursor, float(times[-1])))

    confirmation_by_anchor = {
        (str(item.get("event", "")), float(item["event_time_s"])): float(
            item["confirmed_time_s"]
        )
        for item in accepted
        if str(item.get("event", "")) in {"heel_strike", "toe_off"}
        and item.get("event_time_s") is not None
        and item.get("confirmed_time_s") is not None
    }
    startup_anchors = {
        (str(item.get("event", "")), float(item["event_time_s"]))
        for item in accepted
        if bool(item.get("startup_contact", False))
        and item.get("event_time_s") is not None
    }
    event_anchors: set[tuple[str, float]] = set()
    for axis, coord, served, panel_title in specs:
        first_band = True
        first_mean = True
        first_gap = True
        for gap_start, gap_end in gaps:
            axis.axvspan(
                gap_start,
                gap_end,
                facecolor="#b0b0b0",
                edgecolor="#8a8a8a",
                hatch="///",
                linewidth=0.0,
                alpha=0.13,
                label=("No complete V9 HS-TO-HS corridor" if first_gap else None),
                zorder=0,
            )
            first_gap = False
        for cycle in cycles:
            for segment_type, start, end in (
                ("stance", cycle.heel_strike_s, cycle.toe_off_s),
                ("swing", cycle.toe_off_s, cycle.next_heel_strike_s),
            ):
                dense_time = np.linspace(start, end, 201, dtype=float)
                progress = (dense_time - start) / (end - start)
                if segment_type == "stance":
                    phase = profile.canonical_to_phase * progress
                    colour = "#4c78a8"
                    event_anchors.add(("heel_strike", start))
                    event_anchors.add(("toe_off", end))
                else:
                    phase = profile.canonical_to_phase + (
                        1.0 - profile.canonical_to_phase
                    ) * progress
                    colour = "#f58518"
                    event_anchors.add(("toe_off", start))
                    event_anchors.add(("heel_strike", end))
                corridors = [
                    _corridor_at(
                        profile,
                        float(value),
                        knee_multiplier=knee_multiplier,
                        ankle_multiplier=ankle_multiplier,
                        knee_margin_deg=knee_margin_deg,
                        ankle_margin_deg=ankle_margin_deg,
                    )[coord]
                    for value in phase
                ]
                mean = np.rad2deg([item["mean_rad"] for item in corridors])
                low = np.rad2deg([item["min_rad"] for item in corridors])
                high = np.rad2deg([item["max_rad"] for item in corridors])
                axis.fill_between(
                    dense_time,
                    low,
                    high,
                    color=colour,
                    alpha=0.20,
                    label="V9 two-sensor event-warped corridor" if first_band else None,
                    zorder=1,
                )
                axis.plot(
                    dense_time,
                    mean,
                    color=colour,
                    linewidth=1.2,
                    linestyle="--",
                    alpha=0.85,
                    label="Corridor mean" if first_mean else None,
                    zorder=2,
                )
                first_band = False
                first_mean = False
        axis.plot(
            times,
            served,
            color="#111111",
            linewidth=1.8,
            label="Policy served reference — frozen complete trace",
            zorder=4,
        )
        axis.set_title(panel_title)
        axis.set_ylabel("Angle [deg]")
        axis.grid(True, alpha=0.25)

    for axis in axes:
        labelled: set[str] = set()
        for event, event_time in sorted(event_anchors, key=lambda item: item[1]):
            is_hs = event == "heel_strike"
            is_startup = (event, event_time) in startup_anchors
            colour = "#2ca02c" if is_hs else "#d62728"
            short = "HS" if is_hs else "TO"
            onset_label = (
                "HS startup heel-only sample"
                if is_startup
                else f"{short} accepted onset"
            )
            axis.axvline(
                event_time,
                color=colour,
                linewidth=1.0,
                linestyle="-." if is_startup else "-",
                alpha=0.62,
                label=(onset_label if onset_label not in labelled else None),
                zorder=3,
            )
            labelled.add(onset_label)
            confirmed = confirmation_by_anchor.get((event, event_time))
            if confirmed is not None and confirmed > event_time + VALUE_TOL:
                confirmation_label = f"{short} debounce confirmation"
                axis.axvline(
                    confirmed,
                    color=colour,
                    linewidth=0.8,
                    linestyle=":",
                    alpha=0.42,
                    label=(
                        confirmation_label
                        if confirmation_label not in labelled
                        else None
                    ),
                    zorder=3,
                )
                labelled.add(confirmation_label)
        handles, labels = axis.get_legend_handles_labels()
        unique: dict[str, Any] = {}
        for handle, label in zip(handles, labels):
            if label and label not in unique:
                unique[label] = handle
        axis.legend(unique.values(), unique.keys(), loc="best", fontsize=8.5)

    axes[-1].set_xlabel("Simulation time [s]")
    figure.suptitle(
        "Experimental retrospective morphology corridor — V9 two-sensor detector\n"
        "Accepted-FSM HS-TO-HS warp with heel-only startup rule; dotted lines "
        "= causal confirmation; diagnostic candidate, not promoted",
        fontsize=13,
    )
    figure.text(
        0.5,
        0.925,
        "Exactly 1 heel sphere + 1 forefoot sphere; detector-only; primary GRF "
        "retained for load/contact evidence",
        ha="center",
        va="center",
        fontsize=9.5,
        color="#555555",
    )
    figure.tight_layout(rect=(0.0, 0.0, 1.0, 0.91))
    output_path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output_path, dpi=200, bbox_inches="tight")
    plt.close(figure)


def _json_safe(value: Any) -> Any:
    if isinstance(value, Mapping):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    if isinstance(value, np.ndarray):
        return [_json_safe(item) for item in value.tolist()]
    if isinstance(value, np.generic):
        value = value.item()
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def run(args: argparse.Namespace) -> dict[str, Any]:
    served_trace_path = resolve_repo_path(args.served_trace).resolve()
    state_trace_path = resolve_repo_path(args.state_trace).resolve()
    summary_path = resolve_repo_path(args.summary).resolve()
    states_path = resolve_repo_path(args.states_sto).resolve()
    primary_grf_path = resolve_repo_path(args.primary_grf_sto).resolve()
    detector_profile_path = resolve_repo_path(args.detector_profile).resolve()
    manifest_path = resolve_repo_path(args.v9_manifest).resolve()
    morphology_profile_path = resolve_repo_path(args.morphology_profile).resolve()
    setup_path = resolve_repo_path(args.setup).resolve()
    sea_plugin = str(resolve_repo_path(args.sea_plugin).resolve())
    output_path = resolve_repo_path(args.output).resolve()
    sidecar_path = resolve_repo_path(args.sidecar).resolve()
    if not args.overwrite:
        for destination in (output_path, sidecar_path):
            if destination.exists():
                raise FileExistsError(f"refusing to overwrite {destination}")

    served_rows = _trace_rows(served_trace_path)
    state_rows = _trace_rows(state_trace_path)
    invariance = trace_invariance(served_rows, state_rows)
    summary = _load_json(summary_path)
    if not isinstance(summary, Mapping) or not isinstance(
        summary.get("reward_config"), Mapping
    ):
        raise ValueError("rollout summary requires reward_config")
    reward_config = summary["reward_config"]
    contract = detector_contract(detector_profile_path, manifest_path)

    setup = read_setup_xml(setup_path)
    detector_profile = load_online_grf_profile(
        detector_profile_path,
        required_sides=("left",),
    )
    state_times, _state_columns, _state_data = _read_storage_table(str(states_path))
    sampled_times, samples = _sample_spheres_from_coordinate_states(
        setup,
        detector_profile,
        states_path,
        sea_plugin,
    )
    if not np.array_equal(state_times, sampled_times):
        raise ValueError("state parser and sensor sampler returned different grids")
    trace_times = np.asarray([float(row["time"]) for row in served_rows], dtype=float)
    indices, alignment = aligned_pre_step_indices(trace_times, state_times)
    loads, _penetrations, _detector_aggregate = _regional_loads_and_penetrations(
        detector_profile,
        samples,
    )

    primary_times, primary_columns, primary_data = _read_storage_table(
        str(primary_grf_path)
    )
    if not np.array_equal(primary_times, state_times):
        raise ValueError("primary GRF and states grids differ")
    column_index = {name: index for index, name in enumerate(primary_columns)}
    for required in ("left_normal_force", "left_in_contact"):
        if required not in column_index:
            raise ValueError(f"primary GRF recording is missing {required}")
    primary_normal_force = np.asarray(
        primary_data[indices, column_index["left_normal_force"]], dtype=float
    )
    primary_in_contact = np.asarray(
        primary_data[indices, column_index["left_in_contact"]], dtype=float
    )

    replay = replay_two_sensor_fsm(
        trace_rows=served_rows,
        heel_load_n=np.asarray(loads["left_heel"], dtype=float)[indices],
        toe_load_n=np.asarray(loads["left_toe"], dtype=float)[indices],
        primary_normal_force_n=primary_normal_force,
        primary_in_contact=primary_in_contact,
        body_weight_n=_model_body_weight_n(setup.model_file),
        summary=summary,
    )
    cycles = complete_cycles(replay["accepted"])
    if not cycles:
        raise ValueError("V9 detector produced no complete accepted HS-TO-HS cycle")
    if replay["invalid_steps"]:
        raise ValueError(
            "V9 detector replay produced invalid FSM events: "
            f"{replay['invalid_steps'][:5]}"
        )
    if not all(bool(item.get("matched_sensor_candidate")) for item in replay["accepted"]):
        raise ValueError("an accepted transition lacks a matching sensor candidate")

    morphology_profile = load_profile(morphology_profile_path)
    _plot(
        trace_rows=served_rows,
        cycles=cycles,
        accepted=replay["accepted"],
        profile=morphology_profile,
        reward_config=reward_config,
        output_path=output_path,
    )
    sources = {
        "served_trace": _source(served_trace_path),
        "recorded_state_trace": _source(state_trace_path),
        "states_sto": _source(states_path),
        "primary_grf_sto": _source(primary_grf_path),
        "rollout_summary": _source(summary_path),
        "detector_profile": _source(detector_profile_path),
        "v9_manifest": _source(manifest_path),
        "morphology_profile": _source(morphology_profile_path),
        "setup": _source(setup_path),
        "production_fsm": _source(TRAJECTORY_ROOT / "prosthetic_phase_fsm.py"),
        "plot_script": _source(Path(__file__)),
    }
    accepted_events = [
        {
            "event": str(item.get("event")),
            "event_time_s": float(item["event_time_s"]),
            "confirmed_time_s": float(item["confirmed_time_s"]),
            "confirmation_delay_s": float(
                item["confirmed_time_s"] - item["event_time_s"]
            ),
            "segment_valid": float(item.get("segment_valid", 0.0)),
            "cycle_valid": float(item.get("cycle_valid", -1.0)),
            "startup_contact": bool(item.get("startup_contact", False)),
        }
        for item in replay["accepted"]
        if str(item.get("event")) in {"heel_strike", "toe_off"}
    ]
    result = {
        "schema_version": 2,
        "status": "PASS_DIAGNOSTIC_NOT_PROMOTED",
        "objective": (
            "plot the frozen checkpoint-best served trajectory against the exact "
            "completed-segment morphology corridor anchored by the diagnostic V9 "
            "two-sensor detector"
        ),
        "plot": str(output_path),
        "sidecar": str(sidecar_path),
        "sources": sources,
        "detector_contract": contract,
        "fsm_config": asdict(replay["config"]),
        "timing_semantics": {
            "corridor_anchor": (
                "accepted transition event_time_s; physical contact onset for "
                "ordinary edges, first available heel-ON/toe-OFF sample for "
                "the startup-classified HS"
            ),
            "confirmation": "trace row time after causal dwell; dotted diagnostic",
            "onset_is_not_available_online_until_confirmation": True,
            "startup_hs_is_an_observed_edge": False,
        },
        "primary_evidence_routing": {
            "heel_toe_guards": "V9 detector spheres",
            "normal_force_bw": "recorded primary online GRF",
            "in_contact": "recorded primary online GRF",
            "detector_aggregate_used_as_grf": False,
        },
        "frozen_trace_invariance": invariance,
        "state_sample_alignment": alignment,
        "accepted_event_count": len(accepted_events),
        "accepted_events": accepted_events,
        "complete_cycle_count": len(cycles),
        "complete_cycles": [asdict(cycle) for cycle in cycles],
        "final_fsm": {
            "state_name": replay["final_payload"].get("state_name"),
            "valid_hs_count": replay["final_payload"].get("valid_hs_count"),
            "valid_to_count": replay["final_payload"].get("valid_to_count"),
            "valid_cycle_count": replay["final_payload"].get("valid_cycle_count"),
            "invalid_event_count": replay["final_payload"].get(
                "invalid_event_count"
            ),
        },
        "notes": [
            "The V9 candidate remains a formal 1 ms timing FAIL and is not promoted.",
            "No training, policy update, served-reference edit, or GRF generation was performed.",
            "The first HS is classified from a heel-ON/toe-OFF startup pattern stable for the configured dwell; its anchor is the first available sample, not an observed OFF-to-ON edge.",
            "Only complete accepted HS-TO-HS cycles receive a corridor; boundary-censored regions are hatched.",
        ],
    }
    sidecar_path.parent.mkdir(parents=True, exist_ok=True)
    sidecar_path.write_text(
        json.dumps(_json_safe(result), indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    return result


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--served-trace",
        default=str(MORPHOLOGY_RUN / "rollout_policy_trace.json"),
    )
    parser.add_argument(
        "--state-trace",
        default=str(REFERENCE_RUN / "rollout_policy_trace.json"),
    )
    parser.add_argument(
        "--summary",
        default=str(MORPHOLOGY_RUN / "rollout_summary.json"),
    )
    parser.add_argument(
        "--states-sto",
        default=str(REFERENCE_RUN / "sim_outputs" / "rollout_episode_states.sto"),
    )
    parser.add_argument(
        "--primary-grf-sto",
        default=str(
            REFERENCE_RUN / "sim_outputs" / "rollout_episode_online_grf.sto"
        ),
    )
    parser.add_argument("--detector-profile", default=str(DEFAULT_DETECTOR_PROFILE))
    parser.add_argument("--v9-manifest", default=str(DEFAULT_V9_MANIFEST))
    parser.add_argument(
        "--morphology-profile", default=str(DEFAULT_MORPHOLOGY_PROFILE)
    )
    parser.add_argument("--setup", default=DEFAULT_SETUP)
    parser.add_argument("--sea-plugin", default=DEFAULT_SEA_PLUGIN)
    parser.add_argument("--output", default=str(DEFAULT_OUTPUT))
    parser.add_argument("--sidecar", default=str(DEFAULT_SIDECAR))
    parser.add_argument("--overwrite", action="store_true")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    result = run(build_parser().parse_args(argv))
    print(
        json.dumps(
            {
                "status": result["status"],
                "plot": result["plot"],
                "sidecar": result["sidecar"],
                "accepted_event_count": result["accepted_event_count"],
                "complete_cycle_count": result["complete_cycle_count"],
                "final_fsm": result["final_fsm"],
            },
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
