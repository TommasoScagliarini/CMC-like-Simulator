"""Fixed prescribed comparison of current and experimental V4 geometry.

This validator is intentionally narrower than a threshold sweep.  It evaluates
exactly two hash-pinned detector profiles with exactly one detector contract:
ON/OFF 0.5/0.25 N, 30 ms dwell, confirmed-time semantics.  The already-open
50--100 s AB06 block is replayed at the runtime 10 ms cadence and at 1 ms as a
sampling-sensitivity diagnostic/gate.  No profile, threshold, policy, reward,
FSM, or runtime configuration is tuned or modified here.

The final block beginning at 100 s remains sealed.  The generated time grids
are fail-closed and contain no sample at or after 100 s.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import sys
import traceback
from dataclasses import replace
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
TRAJECTORY_ROOT = REPO_ROOT / "Trajectory Generator"
for path in (REPO_ROOT, TRAJECTORY_ROOT):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from online_grf import (  # noqa: E402
    OnlineGRFProfile,
    OnlineGRFSphere,
    load_online_grf_profile,
)
from path_resolver import resolve_repo_path  # noqa: E402
from setup_io import read_setup_xml  # noqa: E402
from validation.sweep_two_sensor_prescribed_thresholds import (  # noqa: E402
    Candidate,
    _evaluate_candidate,
    evaluate_holdout_gate,
)
from validation.validate_online_grf import (  # noqa: E402
    _calculate_wrench,
    _external_grf,
    _sample_spheres,
)
from validation.validate_two_sensor_prescribed_replay import (  # noqa: E402
    _current_runtime_fsm_config,
    _json_safe,
    _left_sensor_spheres,
    _model_body_weight_n,
    _prescribed_prosthetic_kinematics,
    _reference_events_from_prescribed_grf,
    _run_production_fsm,
)


DEFAULT_PROTOCOL = (
    REPO_ROOT
    / "validation/two_sensor_mesh_profile_v4_fixed_replay_protocol_v4.json"
)
DEFAULT_OUTPUT_DIR = (
    REPO_ROOT
    / "validation/two_sensor_mesh_profile_v4_fixed_replay_runs/"
    "2026-07-21_ab06_50_100_fixed_v4"
)
DEFAULT_PLOT_DIR = (
    REPO_ROOT / "plot/07_21_2026_two_sensor_v4_mesh_fixed_replay"
)
PROTOCOL_ID = "AB06_TWO_SENSOR_V4_FIXED_GEOMETRY_50_100_2026-07-21_V4"
CURRENT_PROFILE_ID = "current_geometry"
V4_PROFILE_ID = "v4_mesh_geometry"
BLOCK_START_S = 50.0
SEALED_START_S = 100.0
SEALED_END_S = 155.045
SENSOR_ON_N = 0.5
SENSOR_OFF_N = 0.25
SENSOR_DWELL_S = 0.030
PRIMARY_DT_S = 0.010
SENSITIVITY_DT_S = 0.001
EVENT_TIME_FIELD = "confirmed_time_s"


class ProtocolError(ValueError):
    """Raised when the frozen comparison contract has drifted."""


def _portable_path(path: Path) -> str:
    resolved = path.resolve()
    try:
        return resolved.relative_to(REPO_ROOT.resolve()).as_posix()
    except ValueError:
        return resolved.as_posix()


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _source_record(path: Path) -> dict[str, Any]:
    resolved = path.resolve()
    if not resolved.is_file():
        raise FileNotFoundError(resolved)
    return {
        "path": _portable_path(resolved),
        "sha256": _sha256(resolved),
        "bytes": int(resolved.stat().st_size),
    }


def _finite(value: Any, label: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ProtocolError(f"{label} must be numeric") from exc
    if not math.isfinite(result):
        raise ProtocolError(f"{label} must be finite")
    return result


def load_and_validate_protocol(
    path: str | Path = DEFAULT_PROTOCOL,
) -> dict[str, Any]:
    """Load a hash-pinned protocol and reject any scope/contract drift."""
    protocol_path = resolve_repo_path(path).resolve()
    try:
        raw = json.loads(protocol_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ProtocolError(f"cannot load protocol: {protocol_path}") from exc
    if not isinstance(raw, dict) or raw.get("schema_version") != 1:
        raise ProtocolError("unsupported V4 fixed-replay protocol schema")
    if raw.get("protocol_id") != PROTOCOL_ID:
        raise ProtocolError("unexpected V4 fixed-replay protocol id")
    if raw.get("frozen_before_execution") is not True:
        raise ProtocolError("protocol must be frozen before execution")

    access = raw.get("data_access")
    if not isinstance(access, dict):
        raise ProtocolError("data_access contract is required")
    if access.get("already_open_block_s") != [BLOCK_START_S, SEALED_START_S]:
        raise ProtocolError("comparison must remain in the opened 50--100 s block")
    if access.get("upper_bound_is_exclusive") is not True:
        raise ProtocolError("100 s must be an exclusive sample boundary")
    if access.get("sealed_block_s") != [SEALED_START_S, SEALED_END_S]:
        raise ProtocolError("sealed-block bounds drifted")
    if access.get("allow_samples_at_or_after_100_s") is not False:
        raise ProtocolError("protocol must forbid samples at or after 100 s")
    if access.get("right_boundary_cycle_policy") != (
        "exclude complete GRF cycles whose closing HS cannot also receive "
        "dwell plus primary-10ms confirmation margin before 100 s; reuse "
        "that common cycle set at both cadences"
    ):
        raise ProtocolError("right-boundary cycle policy drifted")
    if access.get("expected_right_boundary_cycles_excluded") != 1:
        raise ProtocolError("V2 must exclude exactly one right-boundary cycle")

    replay = raw.get("replay")
    if not isinstance(replay, dict):
        raise ProtocolError("replay contract is required")
    exact_values = {
        "sensor_on_threshold_n": SENSOR_ON_N,
        "sensor_off_threshold_n": SENSOR_OFF_N,
        "sensor_dwell_s": SENSOR_DWELL_S,
        "primary_sample_dt_s": PRIMARY_DT_S,
        "sensitivity_sample_dt_s": SENSITIVITY_DT_S,
    }
    for key, expected in exact_values.items():
        if _finite(replay.get(key), key) != expected:
            raise ProtocolError(f"frozen replay value drifted: {key}")
    if replay.get("event_source") != "two_sensor":
        raise ProtocolError("event source must remain production two_sensor")
    if replay.get("primary_event_time_field") != EVENT_TIME_FIELD:
        raise ProtocolError("confirmed_time_s must remain the primary timestamp")
    if replay.get("diagnostic_event_time_field") != "event_time_s":
        raise ProtocolError("event_time_s must remain diagnostic only")
    if replay.get("phase_reference_mode") != "validated_event_intervals":
        raise ProtocolError("phase reference semantics drifted")

    profiles = raw.get("profiles")
    if not isinstance(profiles, dict) or set(profiles) != {
        CURRENT_PROFILE_ID,
        V4_PROFILE_ID,
    }:
        raise ProtocolError("exactly current and V4 profiles are required")
    if profiles[CURRENT_PROFILE_ID].get("role") != "comparator":
        raise ProtocolError("current profile must remain the comparator")
    if profiles[V4_PROFILE_ID].get("role") != "candidate":
        raise ProtocolError("V4 profile must remain the candidate")

    gate = raw.get("sealed_validation_gate")
    if not isinstance(gate, dict):
        raise ProtocolError("fixed acceptance gate is required")
    frozen_gate = {
        "precision": 1.0,
        "recall": 1.0,
        "max_abs_hs_error_s": 0.05,
        "max_abs_toe_off_error_s": 0.08,
        "minimum_confirmed_fsm_stance_f1": 0.95,
        "minimum_confirmed_fsm_stance_iou": 0.90,
        "maximum_confirmed_fsm_iou_regression_vs_baseline": 0.01,
        "maximum_confirmed_time_worst_timing_regression_vs_baseline": 0.0,
    }
    for key, expected in frozen_gate.items():
        if _finite(gate.get(key), key) != expected:
            raise ProtocolError(f"frozen gate value drifted: {key}")
    if gate.get("primary_10ms_and_sensitivity_1ms_must_both_pass") is not True:
        raise ProtocolError("both frozen sampling cadences must gate V4")

    sources = raw.get("sources")
    if not isinstance(sources, dict) or not sources:
        raise ProtocolError("hash-pinned sources are required")
    for label, record in sources.items():
        if not isinstance(record, dict):
            raise ProtocolError(f"invalid source record: {label}")
        source_path = resolve_repo_path(str(record.get("path", ""))).resolve()
        if not source_path.is_file():
            raise ProtocolError(f"missing pinned source {label}: {source_path}")
        observed_hash = _sha256(source_path)
        if observed_hash != record.get("sha256"):
            raise ProtocolError(
                f"source hash drift for {label}: {observed_hash} != "
                f"{record.get('sha256')}"
            )

    raw["_protocol_path"] = protocol_path.as_posix()
    raw["_protocol_sha256"] = _sha256(protocol_path)
    raw["_primary_event_time_field"] = EVENT_TIME_FIELD
    raw["_diagnostic_event_time_field"] = "event_time_s"
    raw["_phase_reference_mode"] = "validated_event_intervals"
    return raw


def _profile_paths(protocol: Mapping[str, Any]) -> dict[str, Path]:
    return {
        profile_id: resolve_repo_path(str(record["path"])).resolve()
        for profile_id, record in protocol["profiles"].items()
    }


def _profile_geometry_invariants(
    current: OnlineGRFProfile,
    candidate: OnlineGRFProfile,
) -> dict[str, Any]:
    """Prove that V4 changes only left_toe.location plus provenance."""
    current_raw = current.to_dict()
    candidate_raw = candidate.to_dict()
    shared_top_level = ("version", "ground", "material")
    equal_top_level = {
        key: current_raw.get(key) == candidate_raw.get(key)
        for key in shared_top_level
    }
    current_spheres = {item["name"]: item for item in current_raw["spheres"]}
    candidate_spheres = {item["name"]: item for item in candidate_raw["spheres"]}
    same_names = set(current_spheres) == set(candidate_spheres)
    sphere_checks: dict[str, bool] = {}
    changed_fields: list[str] = []
    if same_names:
        for name in sorted(current_spheres):
            before = dict(current_spheres[name])
            after = dict(candidate_spheres[name])
            if name == "left_toe":
                before_location = before.pop("location", None)
                after_location = after.pop("location", None)
                sphere_checks[name] = before == after
                if before_location != after_location:
                    changed_fields.append("spheres[left_toe].location")
            else:
                sphere_checks[name] = before == after
    metadata = candidate_raw.get("metadata", {})
    experimental_status = (
        isinstance(metadata, dict)
        and metadata.get("status") == "experimental_not_promoted"
    )
    ok = bool(
        all(equal_top_level.values())
        and same_names
        and sphere_checks
        and all(sphere_checks.values())
        and changed_fields == ["spheres[left_toe].location"]
        and experimental_status
    )
    if not ok:
        raise ProtocolError(
            "V4 profile invariant failed: only left_toe.location may differ"
        )
    before = np.asarray(current_spheres["left_toe"]["location"], dtype=float)
    after = np.asarray(candidate_spheres["left_toe"]["location"], dtype=float)
    return {
        "ok": True,
        "equal_top_level": equal_top_level,
        "same_sphere_names": same_names,
        "sphere_non_location_fields_equal": sphere_checks,
        "changed_runtime_fields": changed_fields,
        "candidate_is_explicitly_experimental_not_promoted": experimental_status,
        "left_toe_location_current_m": before.tolist(),
        "left_toe_location_v4_m": after.tolist(),
        "left_toe_location_delta_m": (after - before).tolist(),
    }


def _sampling_profile(
    current: OnlineGRFProfile,
    candidate: OnlineGRFProfile,
) -> tuple[OnlineGRFProfile, dict[str, dict[str, OnlineGRFSphere]]]:
    """Build one three-sphere sampler shared by both fixed geometries."""
    current_roles = _left_sensor_spheres(current)
    candidate_roles = _left_sensor_spheres(candidate)
    if current_roles["left_heel"] != candidate_roles["left_heel"]:
        raise ProtocolError("V4 heel sensor must remain exactly unchanged")

    heel = current_roles["left_heel"]
    current_toe = replace(
        current_roles["left_toe"], name="comparison_current_left_toe"
    )
    candidate_toe = replace(
        candidate_roles["left_toe"], name="comparison_v4_left_toe"
    )
    sampler = replace(
        current,
        source="validation_fixed_geometry_comparison_sampler",
        spheres=(heel, current_toe, candidate_toe),
    )
    return sampler, {
        CURRENT_PROFILE_ID: {"heel": heel, "toe": current_toe},
        V4_PROFILE_ID: {"heel": heel, "toe": candidate_toe},
    }


def _contact_inputs(
    profile: OnlineGRFProfile,
    spheres: Mapping[str, OnlineGRFSphere],
    samples: Mapping[str, Any],
) -> tuple[dict[str, np.ndarray], dict[str, np.ndarray], np.ndarray]:
    normal = np.asarray(profile.ground.normal, dtype=float)
    normal /= np.linalg.norm(normal)
    origin = np.asarray(profile.ground.origin, dtype=float)
    loads: dict[str, np.ndarray] = {}
    penetrations: dict[str, np.ndarray] = {}
    for short_role, output_role in (("heel", "left_heel"), ("toe", "left_toe")):
        sphere = spheres[short_role]
        one_sphere_profile = replace(profile, spheres=(sphere,))
        loads[output_role] = np.asarray(
            _calculate_wrench(one_sphere_profile, dict(samples))["left"][
                "normal_force"
            ],
            dtype=float,
        )
        center = np.asarray(samples["centers"][sphere.name], dtype=float)
        penetrations[output_role] = np.maximum(
            0.0,
            float(sphere.radius) - (center - origin) @ normal,
        )
    aggregate = loads["left_heel"] + loads["left_toe"]
    if any(
        not np.all(np.isfinite(values))
        for values in (*loads.values(), *penetrations.values(), aggregate)
    ):
        raise FloatingPointError("non-finite fixed-comparison contact signal")
    return loads, penetrations, aggregate


def _build_time_grid(
    reference_events: Mapping[str, np.ndarray],
    *,
    sample_dt_s: float,
) -> tuple[np.ndarray, float]:
    """Create the established complete-cycle grid, strictly below 100 s."""
    if sample_dt_s not in {PRIMARY_DT_S, SENSITIVITY_DT_S}:
        raise ProtocolError("only frozen 10 ms and 1 ms cadences are allowed")
    heel_strikes = np.asarray(reference_events["heel_strike"], dtype=float)
    toe_offs = np.asarray(reference_events["toe_off"], dtype=float)
    if heel_strikes.size != toe_offs.size + 1 or toe_offs.size < 10:
        raise ProtocolError("50--100 s must contain at least ten complete cycles")
    if (
        np.any(heel_strikes < BLOCK_START_S)
        or np.any(toe_offs < BLOCK_START_S)
        or np.any(heel_strikes >= SEALED_START_S)
        or np.any(toe_offs >= SEALED_START_S)
    ):
        raise ProtocolError("reference event escaped the opened 50--100 s block")
    effective_end_s = float(
        heel_strikes[-1] + SENSOR_DWELL_S + sample_dt_s
    )
    if effective_end_s >= SEALED_START_S:
        raise ProtocolError(
            "right-boundary confirmation would touch sealed data at/after 100 s"
        )
    sample_count = int(
        math.floor((effective_end_s - BLOCK_START_S) / sample_dt_s + 1e-12)
    ) + 1
    times = BLOCK_START_S + np.arange(sample_count, dtype=float) * sample_dt_s
    if (
        times.size < 2
        or float(times[0]) != BLOCK_START_S
        or np.any(times >= SEALED_START_S)
        or float(times[-1]) > effective_end_s + 1e-12
    ):
        raise ProtocolError("time grid violated the fail-closed unsealed boundary")
    return times, effective_end_s


def _exclude_unconfirmable_right_boundary_cycles(
    reference_events: Mapping[str, np.ndarray],
    *,
    sample_dt_s: float,
) -> tuple[dict[str, np.ndarray], dict[str, Any]]:
    """Censor only cycles whose closing HS cannot be causally confirmed.

    A prescribed cycle is usable only when its closing HS plus the frozen
    debounce and the primary 10 ms interval remains strictly below 100 s.
    That conservative set is reused at 1 ms.  Since events are chronological,
    eligible cycles must form one prefix.
    """
    heel_strikes = np.asarray(reference_events["heel_strike"], dtype=float)
    toe_offs = np.asarray(reference_events["toe_off"], dtype=float)
    if heel_strikes.size != toe_offs.size + 1:
        raise ProtocolError("reference events are not complete HS-TO-HS cycles")
    # Freeze one common cycle set with the more conservative runtime cadence.
    # The 1 ms sensitivity run must not recover an additional boundary cycle.
    selection_margin_s = SENSOR_DWELL_S + PRIMARY_DT_S
    confirmable = heel_strikes[1:] + selection_margin_s < SEALED_START_S
    if np.any(np.diff(confirmable.astype(int)) > 0):
        raise ProtocolError("right-boundary confirmability is not a prefix")
    kept_cycles = int(np.count_nonzero(confirmable))
    excluded_cycles = int(toe_offs.size - kept_cycles)
    if kept_cycles < 10:
        raise ProtocolError(
            "fewer than ten causally confirmable cycles remain below 100 s"
        )
    if excluded_cycles != 1:
        raise ProtocolError(
            "V4 preregisters exactly one common right-boundary exclusion; observed "
            f"{excluded_cycles}"
        )
    censored = {
        "heel_strike": heel_strikes[: kept_cycles + 1].copy(),
        "toe_off": toe_offs[:kept_cycles].copy(),
    }
    audit = {
        "original_complete_cycle_count": int(toe_offs.size),
        "causally_confirmable_cycle_count": kept_cycles,
        "right_boundary_cycles_excluded": excluded_cycles,
        "excluded_closing_hs_s": heel_strikes[kept_cycles + 1 :].tolist(),
        "common_cycle_selection_margin_s": float(selection_margin_s),
        "evaluation_sample_dt_s": float(sample_dt_s),
        "last_included_closing_hs_s": float(censored["heel_strike"][-1]),
        "last_included_confirmation_limit_s": float(
            censored["heel_strike"][-1] + selection_margin_s
        ),
        "policy": (
            "exclude one common right-boundary cycle set using dwell plus "
            "the primary 10 ms margin, then reuse it at both cadences"
        ),
    }
    return censored, audit


def _prepare_inputs(
    protocol: Mapping[str, Any],
    profiles: Mapping[str, OnlineGRFProfile],
    *,
    sample_dt_s: float,
) -> tuple[dict[str, dict[str, Any]], dict[str, Any]]:
    setup_path = resolve_repo_path(str(protocol["setup"])).resolve()
    setup_for_reference = replace(
        read_setup_xml(setup_path),
        t_start=BLOCK_START_S,
        t_end=float(np.nextafter(SEALED_START_S, -np.inf)),
    )
    replay_cfg = protocol["replay"]
    raw_reference_events, reference_provenance = _reference_events_from_prescribed_grf(
        setup_for_reference,
        threshold_n=float(replay_cfg["prescribed_contact_threshold_n"]),
        min_contact_duration_s=float(
            replay_cfg["reference_min_contact_duration_s"]
        ),
        min_cycle_duration_s=float(
            replay_cfg["reference_min_cycle_duration_s"]
        ),
    )
    reference_events, boundary_cycle_audit = (
        _exclude_unconfirmable_right_boundary_cycles(
            raw_reference_events, sample_dt_s=sample_dt_s
        )
    )
    times, effective_end_s = _build_time_grid(
        reference_events, sample_dt_s=sample_dt_s
    )
    setup = replace(setup_for_reference, t_end=float(times[-1]))
    sampler, spheres_by_profile = _sampling_profile(
        profiles[CURRENT_PROFILE_ID], profiles[V4_PROFILE_ID]
    )
    samples = _sample_spheres(
        setup,
        sampler,
        times,
        str(replay_cfg["sea_plugin"]),
    )
    kinematics = _prescribed_prosthetic_kinematics(setup, times)
    prescribed_vertical_n = np.asarray(
        _external_grf(setup, times)["left"][:, 1], dtype=float
    )
    shared = {
        "setup": setup,
        "times": times,
        "kinematics": kinematics,
        "prescribed_vertical_n": prescribed_vertical_n,
        "reference_events": reference_events,
        "reference_provenance": reference_provenance,
        "body_weight_n": _model_body_weight_n(setup.model_file),
    }
    inputs: dict[str, dict[str, Any]] = {}
    load_summaries: dict[str, Any] = {}
    for profile_id, profile in profiles.items():
        loads, penetrations, aggregate = _contact_inputs(
            profile,
            spheres_by_profile[profile_id],
            samples,
        )
        inputs[profile_id] = {
            **shared,
            "profile": profile,
            "loads": loads,
            "penetrations": penetrations,
            "aggregate": aggregate,
        }
        load_summaries[profile_id] = {
            role: {
                "minimum_n": float(np.min(values)),
                "median_n": float(np.median(values)),
                "maximum_n": float(np.max(values)),
            }
            for role, values in {
                **loads,
                "left_aggregate": aggregate,
            }.items()
        }
    access_audit = {
        "requested_block_s": [BLOCK_START_S, SEALED_START_S],
        "upper_bound_exclusive": True,
        "effective_replay_end_s": effective_end_s,
        "first_sample_s": float(times[0]),
        "last_sample_s": float(times[-1]),
        "sample_count": int(times.size),
        "samples_at_or_after_100_s": int(np.count_nonzero(times >= SEALED_START_S)),
        "all_reference_events_below_100_s": bool(
            all(
                np.all(np.asarray(values, dtype=float) < SEALED_START_S)
                for values in reference_events.values()
            )
        ),
        "sealed_block_opened": False,
        "right_boundary_cycle_audit": boundary_cycle_audit,
        "load_summary_n": load_summaries,
    }
    if (
        access_audit["samples_at_or_after_100_s"] != 0
        or not access_audit["all_reference_events_below_100_s"]
    ):
        raise ProtocolError("sealed-boundary access audit failed")
    return inputs, access_audit


def _evaluate_profiles(
    protocol: Mapping[str, Any],
    inputs: Mapping[str, Mapping[str, Any]],
    *,
    sample_dt_s: float,
) -> tuple[dict[str, dict[str, Any]], dict[str, dict[str, Any]]]:
    rows: dict[str, dict[str, Any]] = {}
    details: dict[str, dict[str, Any]] = {}
    for profile_id in (CURRENT_PROFILE_ID, V4_PROFILE_ID):
        candidate = Candidate(
            candidate_id=profile_id,
            on_threshold_n=SENSOR_ON_N,
            off_threshold_n=SENSOR_OFF_N,
        )
        row, detail = _evaluate_candidate(
            candidate,
            inputs[profile_id],
            protocol,
            sample_dt_s=sample_dt_s,
        )
        continuity = _regional_continuity_diagnostics(
            inputs[profile_id], sample_dt_s=sample_dt_s
        )
        row["profile_id"] = profile_id
        row["diagnostic_max_interior_both_off_gap_s"] = continuity["summary"][
            "maximum_interior_both_off_gap_s"
        ]
        row["diagnostic_max_heel_off_to_toe_on_gap_s"] = continuity["summary"][
            "maximum_heel_off_to_toe_on_gap_s"
        ]
        row["diagnostic_min_heel_toe_overlap_s"] = continuity["summary"][
            "minimum_heel_toe_overlap_s"
        ]
        row["diagnostic_min_toe_peak_n"] = continuity["summary"][
            "minimum_toe_peak_n"
        ]
        row["diagnostic_min_toe_contact_duration_s"] = continuity["summary"][
            "minimum_toe_contact_duration_s"
        ]
        rows[profile_id] = row
        detail["regional_continuity_diagnostics_not_gate"] = continuity
        details[profile_id] = detail
    return rows, details


def _maximum_true_run_duration(mask: np.ndarray, sample_dt_s: float) -> float:
    longest = 0
    current = 0
    for value in np.asarray(mask, dtype=bool):
        if value:
            current += 1
            longest = max(longest, current)
        else:
            current = 0
    return float(longest * sample_dt_s)


def _regional_continuity_diagnostics(
    inputs: Mapping[str, Any],
    *,
    sample_dt_s: float,
) -> dict[str, Any]:
    """Describe heel/toe support continuity without adding a new gate.

    All intervals use prescribed HS-to-TO stance bounds.  Contacts are the
    debounced production-FSM heel/toe latches; peaks use the corresponding raw
    virtual regional load.  A positive signed heel-off-to-toe-on value is a
    gap and a negative value is overlap.
    """
    fsm_cfg = replace(
        _current_runtime_fsm_config(),
        sensor_on_threshold_n=SENSOR_ON_N,
        sensor_off_threshold_n=SENSOR_OFF_N,
        sensor_dwell_s=SENSOR_DWELL_S,
    )
    replay = _run_production_fsm(
        np.asarray(inputs["times"], dtype=float),
        dict(inputs["loads"]),
        dict(inputs["penetrations"]),
        np.asarray(inputs["aggregate"], dtype=float),
        dict(inputs["kinematics"]),
        body_weight_n=float(inputs["body_weight_n"]),
        fsm_config=fsm_cfg,
    )
    times = np.asarray(inputs["times"], dtype=float)
    heel = np.asarray(replay["heel_contact"], dtype=float) > 0.5
    toe = np.asarray(replay["toe_contact"], dtype=float) > 0.5
    toe_load = np.asarray(inputs["loads"]["left_toe"], dtype=float)
    reference_hs = np.asarray(
        inputs["reference_events"]["heel_strike"], dtype=float
    )
    reference_to = np.asarray(inputs["reference_events"]["toe_off"], dtype=float)
    confirmed_to = [
        float(item["confirmed_time_s"])
        for item in replay["accepted"]
        if str(item.get("event", "")) == "toe_off"
        and float(item.get("segment_valid", 1.0)) != 0.0
    ]

    cycles: list[dict[str, Any]] = []
    for index, (hs_s, to_s) in enumerate(zip(reference_hs[:-1], reference_to)):
        indices = np.flatnonzero((times >= hs_s) & (times < to_s))
        if indices.size == 0:
            raise ProtocolError(f"empty prescribed stance interval at cycle {index}")
        local_heel = heel[indices]
        local_toe = toe[indices]
        local_union = local_heel | local_toe
        toe_on_indices = np.flatnonzero(local_toe)
        toe_on_s = (
            float(times[indices[toe_on_indices[0]]])
            if toe_on_indices.size
            else None
        )
        # The first confirmed heel-latch falling edge within stance represents
        # heel clearance.  If heel is already clear at the reference HS, use
        # that boundary explicitly rather than hiding the missing support.
        global_prev = np.concatenate(([False], heel[:-1]))
        falling = indices[global_prev[indices] & ~heel[indices]]
        if falling.size:
            heel_off_s = float(times[falling[0]])
        elif not bool(local_heel[0]):
            heel_off_s = float(hs_s)
        else:
            heel_off_s = None
        signed_transition_s = (
            float(toe_on_s - heel_off_s)
            if toe_on_s is not None and heel_off_s is not None
            else None
        )

        active_union = np.flatnonzero(local_union)
        if active_union.size:
            interior = ~local_union[active_union[0] : active_union[-1] + 1]
            max_interior_gap_s = _maximum_true_run_duration(
                interior, sample_dt_s
            )
            leading_union_delay_s = max(
                0.0, float(times[indices[active_union[0]]] - hs_s)
            )
        else:
            max_interior_gap_s = float(len(indices) * sample_dt_s)
            leading_union_delay_s = float(len(indices) * sample_dt_s)
        toe_duration_s = float(np.count_nonzero(local_toe) * sample_dt_s)
        toe_peak_n = float(np.max(toe_load[indices]))
        predicted_to_s = confirmed_to[index] if index < len(confirmed_to) else None
        cycles.append(
            {
                "cycle_index": index,
                "reference_hs_s": float(hs_s),
                "reference_to_s": float(to_s),
                "toe_on_after_hs_s": (
                    float(toe_on_s - hs_s) if toe_on_s is not None else None
                ),
                "toe_contact_duration_s": toe_duration_s,
                "toe_peak_load_n": toe_peak_n,
                "heel_off_s": heel_off_s,
                "toe_on_s": toe_on_s,
                "heel_off_to_toe_on_signed_s": signed_transition_s,
                "heel_off_to_toe_on_gap_s": (
                    max(0.0, signed_transition_s)
                    if signed_transition_s is not None
                    else None
                ),
                "heel_toe_overlap_s": (
                    max(0.0, -signed_transition_s)
                    if signed_transition_s is not None
                    else None
                ),
                "leading_stable_union_delay_after_hs_s": leading_union_delay_s,
                "maximum_interior_both_off_gap_s": max_interior_gap_s,
                "confirmed_to_s": predicted_to_s,
                "confirmed_to_error_s": (
                    float(predicted_to_s - to_s)
                    if predicted_to_s is not None
                    else None
                ),
            }
        )

    finite = lambda key: [  # noqa: E731
        float(item[key]) for item in cycles if item[key] is not None
    ]
    gaps = finite("heel_off_to_toe_on_gap_s")
    overlaps = finite("heel_toe_overlap_s")
    toe_errors = finite("confirmed_to_error_s")
    return {
        "role": "diagnostic_only_not_part_of_frozen_acceptance_gate",
        "contact_definition": "production FSM debounced regional sensor latch",
        "stance_definition": "prescribed validated HS-to-TO interval",
        "transition_sign_convention": (
            "heel_off_to_toe_on_signed_s > 0 is unsupported gap; < 0 is overlap"
        ),
        "cycles": cycles,
        "summary": {
            "cycle_count": len(cycles),
            "cycles_missing_stable_toe_contact": int(
                sum(item["toe_on_s"] is None for item in cycles)
            ),
            "maximum_interior_both_off_gap_s": max(
                finite("maximum_interior_both_off_gap_s"), default=None
            ),
            "maximum_heel_off_to_toe_on_gap_s": max(gaps, default=None),
            "minimum_heel_toe_overlap_s": min(overlaps, default=None),
            "minimum_toe_peak_n": min(finite("toe_peak_load_n"), default=None),
            "maximum_toe_peak_n": max(finite("toe_peak_load_n"), default=None),
            "minimum_toe_contact_duration_s": min(
                finite("toe_contact_duration_s"), default=None
            ),
            "maximum_toe_contact_duration_s": max(
                finite("toe_contact_duration_s"), default=None
            ),
            "maximum_abs_confirmed_to_error_s": max(
                (abs(value) for value in toe_errors), default=None
            ),
        },
    }


def gate_profile_comparison(
    protocol: Mapping[str, Any],
    rows_by_cadence: Mapping[str, Mapping[str, Mapping[str, Any]]],
) -> dict[str, Any]:
    """Apply the same frozen holdout gate independently at both cadences."""
    expected = {"primary_10ms", "sensitivity_1ms"}
    if set(rows_by_cadence) != expected:
        raise ProtocolError(f"both frozen cadences are required: {expected}")
    cadence_gates = {
        cadence: evaluate_holdout_gate(
            rows[V4_PROFILE_ID],
            rows[CURRENT_PROFILE_ID],
            protocol,
        )
        for cadence, rows in rows_by_cadence.items()
    }
    return {
        "ok": bool(all(item["ok"] for item in cadence_gates.values())),
        "both_cadences_required": True,
        "cadences": cadence_gates,
    }


def _write_metrics_csv(
    path: Path,
    rows_by_cadence: Mapping[str, Mapping[str, Mapping[str, Any]]],
) -> None:
    rows = [
        {"cadence": cadence, **dict(row)}
        for cadence, profiles in rows_by_cadence.items()
        for row in profiles.values()
    ]
    fieldnames = sorted({str(key) for row in rows for key in row})
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def _plot_event_comparison(
    path: Path,
    details: Mapping[str, Mapping[str, Any]],
    *,
    sample_dt_s: float,
) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(2, 1, figsize=(11.0, 7.0), sharex=False)
    colors = {CURRENT_PROFILE_ID: "#4C78A8", V4_PROFILE_ID: "#E45756"}
    tolerances = {"heel_strike": 0.05, "toe_off": 0.08}
    for axis, event in zip(axes, ("heel_strike", "toe_off")):
        for profile_id in (CURRENT_PROFILE_ID, V4_PROFILE_ID):
            events = details[profile_id]["events"]
            reference = np.asarray(events["reference"][event], dtype=float)
            predicted = np.asarray(events["primary"][event], dtype=float)
            if reference.size == predicted.size:
                axis.plot(
                    np.arange(reference.size),
                    1000.0 * (predicted - reference),
                    marker="o",
                    linewidth=1.3,
                    color=colors[profile_id],
                    label=profile_id,
                )
            else:
                axis.text(
                    0.01,
                    0.88 if profile_id == CURRENT_PROFILE_ID else 0.76,
                    f"{profile_id}: count reference={reference.size}, detected={predicted.size}",
                    transform=axis.transAxes,
                    color=colors[profile_id],
                )
        limit_ms = 1000.0 * tolerances[event]
        axis.axhline(limit_ms, color="black", linestyle="--", linewidth=0.9)
        axis.axhline(-limit_ms, color="black", linestyle="--", linewidth=0.9)
        axis.axhline(0.0, color="0.45", linewidth=0.8)
        axis.set_ylabel(f"{event} error [ms]")
        axis.grid(alpha=0.25)
        axis.legend(loc="best")
    axes[-1].set_xlabel("ordered prescribed event index")
    fig.suptitle(
        "Fixed geometry comparison — confirmed-time detector errors — "
        f"dt={1000.0 * sample_dt_s:g} ms"
    )
    fig.tight_layout()
    fig.savefig(path, dpi=170)
    plt.close(fig)


def _artifact_record(path: Path) -> dict[str, Any]:
    return _source_record(path)


def run_comparison(
    protocol: Mapping[str, Any],
    output_dir: Path,
    plot_dir: Path,
) -> dict[str, Any]:
    if output_dir.exists() and any(output_dir.iterdir()):
        raise FileExistsError(f"no-clobber output directory is not empty: {output_dir}")
    if plot_dir.exists() and any(plot_dir.iterdir()):
        raise FileExistsError(f"no-clobber plot directory is not empty: {plot_dir}")
    output_dir.mkdir(parents=True, exist_ok=True)
    plot_dir.mkdir(parents=True, exist_ok=True)

    paths = _profile_paths(protocol)
    profiles = {
        profile_id: load_online_grf_profile(path)
        for profile_id, path in paths.items()
    }
    invariants = _profile_geometry_invariants(
        profiles[CURRENT_PROFILE_ID], profiles[V4_PROFILE_ID]
    )

    cadence_specs = {
        "primary_10ms": PRIMARY_DT_S,
        "sensitivity_1ms": SENSITIVITY_DT_S,
    }
    rows_by_cadence: dict[str, dict[str, dict[str, Any]]] = {}
    details_by_cadence: dict[str, dict[str, dict[str, Any]]] = {}
    access_by_cadence: dict[str, Any] = {}
    plot_paths: dict[str, Path] = {}
    for cadence, sample_dt_s in cadence_specs.items():
        inputs, access = _prepare_inputs(
            protocol, profiles, sample_dt_s=sample_dt_s
        )
        rows, details = _evaluate_profiles(
            protocol, inputs, sample_dt_s=sample_dt_s
        )
        rows_by_cadence[cadence] = rows
        details_by_cadence[cadence] = details
        access_by_cadence[cadence] = access
        plot_path = plot_dir / f"{cadence}_confirmed_event_errors.png"
        _plot_event_comparison(
            plot_path, details, sample_dt_s=sample_dt_s
        )
        plot_paths[cadence] = plot_path

    gate = gate_profile_comparison(protocol, rows_by_cadence)
    csv_path = output_dir / "profile_comparison_metrics.csv"
    _write_metrics_csv(csv_path, rows_by_cadence)
    artifact_records = {
        "metrics_csv": _artifact_record(csv_path),
        **{
            f"{cadence}_event_plot": _artifact_record(path)
            for cadence, path in plot_paths.items()
        },
    }
    manifest = {
        "schema_version": 1,
        "status": "PASS" if gate["ok"] else "FAIL",
        "ok": bool(gate["ok"]),
        "objective": (
            "fixed, no-tuning prescribed replay of current versus V4 mesh "
            "detector geometry on the already-open AB06 50--100 s block"
        ),
        "protocol": {
            "path": _portable_path(Path(str(protocol["_protocol_path"]))),
            "sha256": str(protocol["_protocol_sha256"]),
            "protocol_id": str(protocol["protocol_id"]),
            "frozen_before_execution": True,
        },
        "data_access": {
            "already_open_block_s": [BLOCK_START_S, SEALED_START_S],
            "sealed_block_s": [SEALED_START_S, SEALED_END_S],
            "sealed_block_opened": False,
            "cadences": access_by_cadence,
        },
        "detector_contract": {
            "production_fsm": "prosthetic_phase_fsm.ProstheticPhaseFSM",
            "event_source": "two_sensor",
            "sensor_on_threshold_n": SENSOR_ON_N,
            "sensor_off_threshold_n": SENSOR_OFF_N,
            "sensor_dwell_s": SENSOR_DWELL_S,
            "primary_event_time_field": EVENT_TIME_FIELD,
            "diagnostic_event_time_field": "event_time_s",
            "threshold_or_geometry_tuning_performed": False,
        },
        "profiles": {
            profile_id: {
                "role": str(protocol["profiles"][profile_id]["role"]),
                "source": _source_record(paths[profile_id]),
                "declared_source": str(profiles[profile_id].source),
            }
            for profile_id in (CURRENT_PROFILE_ID, V4_PROFILE_ID)
        },
        "profile_invariants": invariants,
        "metrics": rows_by_cadence,
        "details": details_by_cadence,
        "acceptance_gate": gate,
        "artifacts": artifact_records,
        "conclusion": (
            "V4_PRESCRIBED_FIXED_GATE_PASS"
            if gate["ok"]
            else "V4_PRESCRIBED_FIXED_GATE_FAIL_NOT_PROMOTABLE"
        ),
        "interpretation_limits": [
            "This comparison neither trains nor evaluates a policy.",
            "It does not modify the production FSM, runtime config, or either profile.",
            "A PASS would support only prescribed AB06 detector promotion review, not hardware validity.",
            "The 100--155.045 s block remains sealed and has no sampled timestamps in this run.",
        ],
    }
    safe_manifest = _json_safe(manifest)
    manifest_path = output_dir / "manifest.json"
    manifest_path.write_text(
        json.dumps(safe_manifest, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    return safe_manifest


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Run the frozen 50--100 s prescribed comparison of current and "
            "experimental V4 two-sensor geometry."
        )
    )
    parser.add_argument("--protocol", default=str(DEFAULT_PROTOCOL))
    parser.add_argument("--output-dir", default=str(DEFAULT_OUTPUT_DIR))
    parser.add_argument("--plot-dir", default=str(DEFAULT_PLOT_DIR))
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_arg_parser().parse_args(argv)
    output_dir = resolve_repo_path(args.output_dir).resolve()
    plot_dir = resolve_repo_path(args.plot_dir).resolve()
    try:
        protocol = load_and_validate_protocol(args.protocol)
        manifest = run_comparison(protocol, output_dir, plot_dir)
    except Exception as exc:  # pragma: no cover - CLI fail-closed path
        output_dir.mkdir(parents=True, exist_ok=True)
        failure = {
            "schema_version": 1,
            "status": "ERROR",
            "ok": False,
            "sealed_block_opened": False,
            "error": f"{type(exc).__name__}: {exc}",
            "traceback": traceback.format_exc(),
        }
        failure_path = output_dir / "failure.json"
        if not failure_path.exists():
            failure_path.write_text(
                json.dumps(failure, indent=2, allow_nan=False) + "\n",
                encoding="utf-8",
            )
        print(json.dumps(failure, indent=2))
        return 2
    concise = {
        "status": manifest["status"],
        "conclusion": manifest["conclusion"],
        "sealed_block_opened": manifest["data_access"]["sealed_block_opened"],
        "acceptance_gate": manifest["acceptance_gate"],
        "manifest": _portable_path(output_dir / "manifest.json"),
    }
    print(json.dumps(concise, indent=2, allow_nan=False))
    return 0 if manifest["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
