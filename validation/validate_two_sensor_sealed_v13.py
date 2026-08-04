"""One-shot sealed validation of the single frozen V12 detector candidate.

V13 opens the AB06 ``[100.0, 155.045)`` block exactly once after binding the
V12 primary candidate by manifest, candidate-record digest, and exact sphere
geometry.  The 0.70 mm toe-depth candidate and the historical V9 geometry are
sampled in the same OpenSim pass, but are diagnostics only: neither can rescue,
replace, veto, or otherwise alter the primary 0.75 mm PASS/FAIL decision.

The output directory is created and an access receipt is written before the
first semantic decode of sealed prescribed-GRF values.  Cryptographic source
hashing alone is an integrity check, not event/metric access.  A crash after
access starts leaves an occupied destination and fails closed on any rerun.
"""

from __future__ import annotations

import argparse
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
VALIDATION_ROOT = REPO_ROOT / "validation"
for path in (REPO_ROOT, VALIDATION_ROOT, REPO_ROOT / "Trajectory Generator"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

import sweep_two_sensor_toe_compensation_prescribed_v12 as v12  # noqa: E402


v11 = v12.v11
v10 = v12.v10
v6 = v10.v6
v1 = v12.v1
dual = v12.dual

DEFAULT_PROTOCOL = VALIDATION_ROOT / "two_sensor_sealed_protocol_v13.json"
DEFAULT_LOCK = VALIDATION_ROOT / "two_sensor_sealed_candidate_lock_v13.json"
DEFAULT_OUTPUT_DIR = (
    VALIDATION_ROOT
    / "two_sensor_sealed_runs/2026-07-22_ab06_100_155p045_v13"
)
DEFAULT_PLOT_DIR = REPO_ROOT / "plot/07_22_2026_two_sensor_sealed_v13"

PROTOCOL_ID = "AB06_TWO_SENSOR_V12_PRIMARY_SEALED_2026-07-22_V13"
LOCK_ID = "AB06_TWO_SENSOR_V12_PRIMARY_SEALED_LOCK_2026-07-22_V13"
SCHEMA_VERSION = 13
SEALED_START_S = 100.0
SEALED_END_S = 155.045
PRIMARY_DT_S = 0.010
FINE_DT_S = 0.001
EXPECTED_REFERENCE_HS = 49
EXPECTED_REFERENCE_TO = 48
EXPECTED_CYCLES = 48
PRIMARY_ID = "toe_down_p0p75mm__heel_x_p3p5mm"
SENSITIVITY_ID = "toe_down_p0p7mm__heel_x_p3p5mm"
V9_ID = "v9_baseline"
PAIR_COUNT = 3
DETECTOR_STATIONS = 5
PRIMARY_LOAD_SPHERES = 8
TOTAL_STATIONS = DETECTOR_STATIONS + PRIMARY_LOAD_SPHERES
NUMERIC_TOLERANCE = 1.0e-12
HEEL_OFF_CADENCE_TOLERANCE_S = PRIMARY_DT_S

V12_PROTOCOL_PATH = "validation/two_sensor_toe_compensation_sweep_protocol_v12.json"
V12_MANIFEST_PATH = (
    "validation/two_sensor_toe_compensation_sweep_runs/"
    "2026-07-22_ab06_open_13p946_100_v12/manifest.json"
)
V12_RUNTIME_CSV = (
    "validation/two_sensor_toe_compensation_sweep_runs/"
    "2026-07-22_ab06_open_13p946_100_v12/"
    "toe_compensation_v12_runtime_10ms_metrics.csv"
)
V12_FINE_CSV = (
    "validation/two_sensor_toe_compensation_sweep_runs/"
    "2026-07-22_ab06_open_13p946_100_v12/"
    "toe_compensation_v12_fine_1ms_metrics.csv"
)

OBJECTIVE = (
    "Open the AB06 100-155.045 s sealed block exactly once and validate only "
    "the already-frozen V12 primary geometry at both 10 ms and 1 ms under the "
    "unchanged event, FSM, causal, transfer, mesh, threshold, and dwell gates; "
    "report the 0.70 mm and V9 geometries as non-decision diagnostics only."
)
INTERPRETATION_LIMITS = [
    "V13 is a one-shot chronological holdout on AB06 treadmill_01_01, not a new geometry search.",
    "Only the frozen 0.75 mm V12 primary determines PASS/FAIL; there is no selector, ranking, fallback, or replacement.",
    "The 0.70 mm candidate is sensitivity-only and the V9 pair is heel-off diagnostic-only.",
    "All three pairs are sampled unconditionally at 10 ms and 1 ms in the same pass per cadence.",
    "Whole-file cryptographic hashing is permitted as source-identity verification while sealed; timestamps, events, and metrics cannot be decoded before the receipt.",
    "Confirmed_time_s is authoritative; event_time_s remains diagnostic only.",
    "Thresholds, dwell, radii, FSM, load routing, policy, reward, training, and runtime configuration remain unchanged.",
    "Heel-off cross-cadence and V9-shift results are robustness diagnostics and cannot change the sealed event verdict.",
    "A PASS does not establish cross-subject, cross-task, hardware, or noise robustness and does not automatically promote a profile.",
]

REQUIRED_SOURCE_PATHS = {
    "v13_validator": "validation/validate_two_sensor_sealed_v13.py",
    "v13_tests": "validation/test_two_sensor_sealed_v13.py",
    "v13_candidate_lock": "validation/two_sensor_sealed_candidate_lock_v13.json",
    "v12_protocol": V12_PROTOCOL_PATH,
    "v12_manifest": V12_MANIFEST_PATH,
    "v12_validator": "validation/sweep_two_sensor_toe_compensation_prescribed_v12.py",
    "v12_tests": "validation/test_two_sensor_toe_compensation_sweep_v12.py",
    "v12_runtime_csv": V12_RUNTIME_CSV,
    "v12_fine_csv": V12_FINE_CSV,
}


class ProtocolError(v11.ProtocolError):
    """Raised before or during V13 when the frozen contract is violated."""


NoClobberError = v11.NoClobberError


def _canonical_sha256(value: Any) -> str:
    payload = json.dumps(
        value,
        sort_keys=True,
        separators=(",", ":"),
        allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def _expected_data_access() -> dict[str, Any]:
    return {
        "sealed_interval_s": [SEALED_START_S, SEALED_END_S],
        "lower_bound_is_inclusive": True,
        "upper_bound_is_exclusive": True,
        "single_authorized_open": True,
        "time_grid_start_s": SEALED_START_S,
        "right_observation_margin_s": 0.060,
        "right_observation_margin_rule": "hs_tolerance_50ms_plus_runtime_sample_10ms",
        "common_cycle_set_at_both_cadences": True,
        "expected_raw_complete_cycles": EXPECTED_CYCLES,
        "expected_retained_complete_cycles": EXPECTED_CYCLES,
        "expected_reference_hs": EXPECTED_REFERENCE_HS,
        "expected_reference_to": EXPECTED_REFERENCE_TO,
        "minimum_complete_cycles": 10,
    }


def _sealed_gate_contract() -> dict[str, Any]:
    gate = dict(v10._gate_contract())
    gate["require_exact_reference_counts"] = {
        "heel_strike": EXPECTED_REFERENCE_HS,
        "toe_off": EXPECTED_REFERENCE_TO,
    }
    gate["require_exact_detector_counts"] = {
        "heel_strike": EXPECTED_REFERENCE_HS,
        "toe_off": EXPECTED_REFERENCE_TO,
    }
    gate["require_exact_valid_cycles"] = EXPECTED_CYCLES
    gate["require_exact_causal_swing_intervals"] = EXPECTED_CYCLES
    return gate


def _expected_sampling() -> dict[str, Any]:
    cadence = {
        "method": "shared_station_sampling_direct_opensim",
        "expected_unique_detector_stations": DETECTOR_STATIONS,
        "expected_primary_load_spheres": PRIMARY_LOAD_SPHERES,
        "expected_total_sampled_stations": TOTAL_STATIONS,
        "evaluated_pair_count": PAIR_COUNT,
    }
    return {
        "runtime_10ms": dict(cadence),
        "fine_1ms": dict(cadence),
        "affine_reconstruction": False,
    }


def _expected_candidate_scope() -> dict[str, Any]:
    return {
        "primary_candidate_id": PRIMARY_ID,
        "sensitivity_candidate_id": SENSITIVITY_ID,
        "historical_comparator_id": V9_ID,
        "primary_count": 1,
        "diagnostic_count": 2,
        "total_pair_count": PAIR_COUNT,
        "primary_alone_determines_pass_fail": True,
        "sensitivity_can_rescue_or_replace_primary": False,
        "historical_comparator_can_affect_decision": False,
        "ranking_or_fallback_allowed": False,
    }


def _expected_decision_contract() -> dict[str, Any]:
    return {
        "sealed_validation_allowed": True,
        "single_use_output_receipt_required_before_data_read": True,
        "primary_must_pass_runtime_10ms_and_fine_1ms": True,
        "sensitivity_is_nonselectable": True,
        "v9_is_nonselectable": True,
        "automatic_profile_creation_allowed": False,
        "automatic_profile_promotion_allowed": False,
        "runtime_modification_allowed": False,
        "training_allowed": False,
        "v9_v10_v11_v12_files_must_remain_immutable": True,
    }


def load_and_validate_protocol(
    path: str | Path = DEFAULT_PROTOCOL,
) -> dict[str, Any]:
    protocol_path = v1.resolve_repo_path(path).resolve()
    try:
        raw = json.loads(protocol_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ProtocolError(f"cannot load V13 protocol: {protocol_path}") from exc
    if not isinstance(raw, dict):
        raise ProtocolError("V13 protocol root must be an object")

    expected_top = {
        "schema_version": SCHEMA_VERSION,
        "protocol_id": PROTOCOL_ID,
        "freeze_date": "2026-07-22",
        "frozen_before_execution": True,
        "stage": "single_frozen_candidate_sealed_validation",
        "objective": OBJECTIVE,
        "interpretation_limits": INTERPRETATION_LIMITS,
        "candidate_lock": "validation/two_sensor_sealed_candidate_lock_v13.json",
        "setup": (
            "models/AB06_SEASEA_Threadmill/"
            "AB06_SEASEA_stiff321_500_pi_setup.xml"
        ),
        "detector_template_profile": (
            "validation/experimental_detector_profiles/"
            "two_sensor_v9_H2p50_X3p25_F79p0_P35p00.json"
        ),
        "geometry_reference_profile": v10.GEOMETRY_REFERENCE_PROFILE,
        "load_evidence_profile": dual.LOAD_EVIDENCE_PROFILE,
    }
    for key, expected in expected_top.items():
        if raw.get(key) != expected:
            raise ProtocolError(f"V13 frozen field drifted: {key}")
    if raw.get("data_access") != _expected_data_access():
        raise ProtocolError("V13 data-access contract drifted")
    if raw.get("replay") != v11._expected_replay():
        raise ProtocolError("V13 replay contract drifted")
    if raw.get("candidate_scope") != _expected_candidate_scope():
        raise ProtocolError("V13 candidate scope drifted")
    if raw.get("sampling") != _expected_sampling():
        raise ProtocolError("V13 sampling contract drifted")
    if raw.get("runtime_gate_10ms") != _sealed_gate_contract():
        raise ProtocolError("V13 runtime gate drifted")
    if raw.get("fine_gate_1ms") != _sealed_gate_contract():
        raise ProtocolError("V13 fine gate drifted")
    if raw.get("decision_contract") != _expected_decision_contract():
        raise ProtocolError("V13 decision contract drifted")
    expected_robustness = {
        "heel_off_cross_cadence_tolerance_s": HEEL_OFF_CADENCE_TOLERANCE_S,
        "heel_off_cross_cadence_role": "promotion_readiness_diagnostic_not_sealed_event_gate",
        "heel_off_vs_v9_role": "diagnostic_only",
        "can_change_primary_sealed_event_verdict": False,
    }
    if raw.get("robustness_diagnostics") != expected_robustness:
        raise ProtocolError("V13 robustness diagnostic contract drifted")

    sources = raw.get("sources")
    if not isinstance(sources, dict) or set(sources) != set(REQUIRED_SOURCE_PATHS):
        raise ProtocolError("V13 hash-pinned source set drifted")
    for label, record in sources.items():
        if not isinstance(record, dict):
            raise ProtocolError(f"invalid V13 source record: {label}")
        if record.get("path") != REQUIRED_SOURCE_PATHS[label]:
            raise ProtocolError(f"V13 source path drifted: {label}")
        source_path = v1.resolve_repo_path(str(record["path"])).resolve()
        if not source_path.is_file():
            raise ProtocolError(f"missing V13 source: {label}")
        if v1._sha256(source_path) != record.get("sha256"):
            raise ProtocolError(f"V13 source hash drifted: {label}")

    raw["_protocol_path"] = protocol_path.as_posix()
    raw["_protocol_sha256"] = v1._sha256(protocol_path)
    raw["_primary_event_time_field"] = "confirmed_time_s"
    raw["_diagnostic_event_time_field"] = "event_time_s"
    raw["_phase_reference_mode"] = "validated_event_intervals"
    raw["profile_paths"] = {
        v1.CURRENT_PROFILE_ID: str(raw["geometry_reference_profile"])
    }
    raw["development_gate"] = raw["runtime_gate_10ms"]
    return raw


def load_and_validate_lock(
    path: str | Path = DEFAULT_LOCK,
) -> dict[str, Any]:
    lock_path = v1.resolve_repo_path(path).resolve()
    try:
        lock = json.loads(lock_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ProtocolError(f"cannot load V13 candidate lock: {lock_path}") from exc
    if not isinstance(lock, dict):
        raise ProtocolError("V13 candidate lock root must be an object")
    if not (
        lock.get("schema_version") == 1
        and lock.get("lock_id") == LOCK_ID
        and lock.get("frozen_before_sealed_access") is True
        and lock.get("primary", {}).get("candidate_id") == PRIMARY_ID
        and lock.get("sensitivity", {}).get("candidate_id") == SENSITIVITY_ID
        and lock.get("historical_comparator", {}).get("candidate_id") == V9_ID
        and lock.get("decision_contract", {}).get(
            "primary_alone_determines_sealed_pass_fail"
        )
        is True
        and lock.get("decision_contract", {}).get(
            "ranking_or_post_hoc_selection_allowed"
        )
        is False
    ):
        raise ProtocolError("V13 candidate lock semantic contract drifted")
    for record in lock.get("lineage", {}).values():
        if not isinstance(record, Mapping):
            raise ProtocolError("invalid candidate-lock lineage record")
        source = v1.resolve_repo_path(str(record.get("path", ""))).resolve()
        if not source.is_file() or v1._sha256(source) != record.get("sha256"):
            raise ProtocolError("candidate-lock lineage hash drifted")
    lock["_lock_path"] = lock_path.as_posix()
    lock["_lock_sha256"] = v1._sha256(lock_path)
    return lock


def _validate_lock_binding(
    protocol: Mapping[str, Any], lock: Mapping[str, Any]
) -> None:
    expected_path = v1.resolve_repo_path(str(protocol["candidate_lock"])).resolve()
    observed_path = Path(str(lock.get("_lock_path", ""))).resolve()
    expected_sha256 = str(
        protocol["sources"]["v13_candidate_lock"]["sha256"]
    )
    if not (
        observed_path == expected_path
        and str(lock.get("_lock_sha256", "")) == expected_sha256
    ):
        raise ProtocolError("V13 CLI candidate lock is not protocol-bound")


def _candidate_record_digest(candidate: Any) -> str:
    return _canonical_sha256(v11._candidate_record(candidate))


def _geometry_matches_lock(candidate: Any, record: Mapping[str, Any]) -> bool:
    return bool(
        list(candidate.heel_location) == record.get("heel_location_m")
        and list(candidate.forefoot_location) == record.get("toe_location_m")
        and float(candidate.geometry["heel_radius_m"])
        == float(record.get("heel_radius_m"))
        and float(candidate.geometry["toe_radius_m"])
        == float(record.get("toe_radius_m"))
        and float(candidate.geometry["heel_x_shift_mm"])
        == float(record.get("heel_x_shift_from_v9_mm"))
        and float(candidate.geometry["toe_center_down_mm"])
        == float(record.get("toe_center_down_from_v9_mm"))
    )


def build_frozen_candidates(
    protocol: Mapping[str, Any], lock: Mapping[str, Any]
) -> tuple[Any, list[Any], dict[str, Any]]:
    v12_protocol = v12.load_and_validate_protocol(protocol["sources"]["v12_protocol"]["path"])
    manifest_path = v1.resolve_repo_path(V12_MANIFEST_PATH).resolve()
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if not (
        manifest.get("schema_version") == 12
        and manifest.get("status") == "PASS"
        and manifest.get("ok") is True
        and manifest.get("selection", {}).get("strict_winner_id") == PRIMARY_ID
        and manifest.get("reported_candidate", {}).get("candidate_id") == PRIMARY_ID
        and manifest.get("reported_candidate", {}).get("strict_multiresolution_pass")
        is True
        and manifest.get("data_access", {}).get("sealed_block_opened") is False
    ):
        raise ProtocolError("V12 prerequisite does not bind one passing primary")

    base, v12_candidates, _boundary, _geometry = v12.build_candidates(v12_protocol)
    by_id = {item.candidate_id: item for item in v12_candidates}
    if PRIMARY_ID not in by_id or SENSITIVITY_ID not in by_id:
        raise ProtocolError("frozen V12 primary or sensitivity candidate is missing")
    primary_v12 = by_id[PRIMARY_ID]
    sensitivity_v12 = by_id[SENSITIVITY_ID]
    manifest_records = {
        str(item["candidate_id"]): item for item in manifest.get("candidates", [])
    }
    for candidate, role in (
        (primary_v12, "primary"),
        (sensitivity_v12, "sensitivity"),
    ):
        expected_record = lock[role]
        current_record = v11._candidate_record(candidate)
        if manifest_records.get(candidate.candidate_id) != current_record:
            raise ProtocolError(f"V12 manifest geometry drifted: {candidate.candidate_id}")
        if _candidate_record_digest(candidate) != expected_record.get(
            "v12_candidate_record_sha256"
        ):
            raise ProtocolError(f"V12 candidate digest drifted: {candidate.candidate_id}")
        if not _geometry_matches_lock(candidate, expected_record):
            raise ProtocolError(f"V12 candidate lock geometry drifted: {candidate.candidate_id}")

    _base_v9, stage_a, _geometry_v9 = v11.build_stage_a_candidates(v12_protocol)
    v9_v12 = next(item for item in stage_a if item.candidate_id == V9_ID)
    if not _geometry_matches_lock(v9_v12, lock["historical_comparator"]):
        raise ProtocolError("V9 diagnostic comparator geometry drifted")

    def bind(candidate: Any, *, selectable: bool, role: str) -> Any:
        geometry = dict(candidate.geometry)
        geometry["v13_frozen_role"] = role
        return replace(
            candidate,
            selectable=selectable,
            role=role,
            geometry=geometry,
        )

    candidates = [
        bind(
            primary_v12,
            selectable=True,
            role="single_frozen_primary_sealed_candidate",
        ),
        bind(
            sensitivity_v12,
            selectable=False,
            role="nonselectable_sensitivity_only",
        ),
        bind(
            v9_v12,
            selectable=False,
            role="nonselectable_heel_off_diagnostic_only",
        ),
    ]
    if [item.candidate_id for item in candidates] != [
        PRIMARY_ID,
        SENSITIVITY_ID,
        V9_ID,
    ]:
        raise ProtocolError("V13 candidate order or scope drifted")
    if sum(bool(item.selectable) for item in candidates) != 1:
        raise ProtocolError("V13 must expose exactly one primary candidate")
    geometry = {
        "primary_id": PRIMARY_ID,
        "sensitivity_id": SENSITIVITY_ID,
        "historical_comparator_id": V9_ID,
        "primary_v12_record_sha256": _candidate_record_digest(primary_v12),
        "sensitivity_v12_record_sha256": _candidate_record_digest(sensitivity_v12),
        "candidate_lock": v1._source_record(
            Path(str(lock["_lock_path"])).resolve()
        ),
        "only_primary_is_selectable": True,
        "radii_unchanged": True,
    }
    return base, candidates, geometry


def _reference_bundle(
    protocol: Mapping[str, Any], *, sample_dt_s: float
) -> tuple[Any, dict[str, np.ndarray], dict[str, Any], np.ndarray]:
    if sample_dt_s not in {PRIMARY_DT_S, FINE_DT_S}:
        raise ProtocolError("V13 permits only the frozen 10 ms and 1 ms cadences")
    setup = replace(
        v1.read_setup_xml(v1.resolve_repo_path(str(protocol["setup"])).resolve()),
        t_start=SEALED_START_S,
        t_end=float(np.nextafter(SEALED_END_S, -np.inf)),
    )
    replay = protocol["replay"]
    raw_events, provenance = v1._reference_events_from_prescribed_grf(
        setup,
        threshold_n=float(replay["prescribed_contact_threshold_n"]),
        min_contact_duration_s=float(replay["reference_min_contact_duration_s"]),
        min_cycle_duration_s=float(replay["reference_min_cycle_duration_s"]),
    )
    raw_hs = np.asarray(raw_events["heel_strike"], dtype=float)
    raw_to = np.asarray(raw_events["toe_off"], dtype=float)
    if raw_hs.size != EXPECTED_REFERENCE_HS or raw_to.size != EXPECTED_REFERENCE_TO:
        raise ProtocolError(
            "V13 sealed reference count drifted before right-boundary censoring"
        )
    observation_margin_s = float(protocol["data_access"]["right_observation_margin_s"])
    confirmable = raw_hs[1:] + observation_margin_s < SEALED_END_S
    if np.any(np.diff(confirmable.astype(int)) > 0):
        raise ProtocolError("V13 right-boundary confirmability is not a prefix")
    retained_cycles = int(np.count_nonzero(confirmable))
    if retained_cycles != EXPECTED_CYCLES:
        raise ProtocolError(
            f"V13 retained sealed cycle count drifted: {retained_cycles}"
        )
    events = {
        "heel_strike": raw_hs[: retained_cycles + 1].copy(),
        "toe_off": raw_to[:retained_cycles].copy(),
    }
    excluded = raw_hs[retained_cycles + 1 :].tolist()
    target_end_s = float(events["heel_strike"][-1] + observation_margin_s)
    intervals = int(
        math.floor((target_end_s - SEALED_START_S) / sample_dt_s + NUMERIC_TOLERANCE)
    )
    times = SEALED_START_S + np.arange(intervals + 1, dtype=float) * sample_dt_s
    latest_required_s = float(
        events["heel_strike"][-1] + float(replay["hs_tolerance_s"])
    )
    if not (
        times.size >= 2
        and math.isclose(float(times[0]), SEALED_START_S, abs_tol=NUMERIC_TOLERANCE)
        and float(times[-1]) >= latest_required_s - NUMERIC_TOLERANCE
        and np.all(times >= SEALED_START_S)
        and np.all(times < SEALED_END_S)
    ):
        raise ProtocolError("V13 time grid violated the sealed interval contract")
    setup = replace(setup, t_end=float(times[-1]))
    reference_digest = _canonical_sha256(
        {
            "heel_strike": events["heel_strike"].tolist(),
            "toe_off": events["toe_off"].tolist(),
        }
    )
    access = {
        "sample_dt_s": float(sample_dt_s),
        "first_sample_s": float(times[0]),
        "last_sample_s": float(times[-1]),
        "sample_count": int(times.size),
        "samples_below_100_s": int(np.count_nonzero(times < SEALED_START_S)),
        "samples_at_or_after_155p045_s": int(
            np.count_nonzero(times >= SEALED_END_S)
        ),
        "sealed_block_opened": True,
        "reference_hs_count": int(events["heel_strike"].size),
        "reference_to_count": int(events["toe_off"].size),
        "reference_cycle_count": int(events["toe_off"].size),
        "reference_event_sha256": reference_digest,
        "right_boundary_audit": {
            "raw_complete_cycle_count": int(raw_to.size),
            "retained_complete_cycle_count": retained_cycles,
            "observation_margin_s": observation_margin_s,
            "excluded_closing_hs_s": excluded,
        },
        "reference_provenance": provenance,
    }
    return setup, events, access, times


def sample_streams_once(
    protocol: Mapping[str, Any],
    base: Any,
    candidates: Sequence[Any],
    *,
    sample_dt_s: float,
    stage_label: str,
) -> tuple[dict[str, dict[str, Any]], dict[str, Any]]:
    setup, events, access, times = _reference_bundle(
        protocol, sample_dt_s=sample_dt_s
    )
    detector_sampler, sphere_pairs, detector_profiles = v11._sampling_bundle(
        base,
        candidates,
        stage_label=stage_label,
        expected_detector_stations=DETECTOR_STATIONS,
    )
    primary_path = v1.resolve_repo_path(
        str(protocol["load_evidence_profile"])
    ).resolve()
    primary_full = dual.load_online_grf_profile(primary_path)
    primary_spheres = tuple(
        sphere for sphere in primary_full.spheres if sphere.side == "left"
    )
    if len(primary_spheres) != PRIMARY_LOAD_SPHERES:
        raise ProtocolError("V13 primary load-sphere count drifted")
    primary_left = replace(
        primary_full,
        source=f"validation_v13_{stage_label}_primary_left",
        spheres=primary_spheres,
    )
    combined = tuple(detector_sampler.spheres) + primary_spheres
    if len(combined) != TOTAL_STATIONS:
        raise ProtocolError("V13 combined sampled-station count drifted")
    if len({sphere.name for sphere in combined}) != TOTAL_STATIONS:
        raise ProtocolError("V13 sampled station names are not unique")
    sampler = replace(
        base,
        source=f"validation_v13_{stage_label}_sealed_sampler",
        spheres=combined,
    )
    samples = v1._sample_spheres(
        setup,
        sampler,
        times,
        str(protocol["replay"]["sea_plugin"]),
    )
    primary_aggregate = np.asarray(
        dual._calculate_wrench(primary_left, dict(samples))["left"]["normal_force"],
        dtype=float,
    )
    primary_penetration = dual._primary_physical_penetration(
        primary_left, samples
    )
    kinematics = v1._prescribed_prosthetic_kinematics(setup, times)
    prescribed_vertical_n = np.asarray(
        v1._external_grf(setup, times)["left"][:, 1], dtype=float
    )
    shared = {
        "setup": setup,
        "times": times,
        "kinematics": kinematics,
        "prescribed_vertical_n": prescribed_vertical_n,
        "reference_events": events,
        "body_weight_n": v1._model_body_weight_n(setup.model_file),
        "primary_aggregate": primary_aggregate,
        "primary_penetration": primary_penetration,
    }
    streams: dict[str, dict[str, Any]] = {}
    for candidate in candidates:
        loads, penetrations, aggregate = v1._contact_inputs(
            detector_profiles[candidate.candidate_id],
            sphere_pairs[candidate.candidate_id],
            samples,
        )
        streams[candidate.candidate_id] = {
            **shared,
            "detector_profile": detector_profiles[candidate.candidate_id],
            "detector_loads": loads,
            "detector_penetrations": penetrations,
            "detector_aggregate": aggregate,
        }
    access.update(
        {
            "stage": stage_label,
            "single_opensim_station_sampling_pass": True,
            "direct_sampling_without_affine_reconstruction": True,
            "sampled_unique_detector_stations": len(detector_sampler.spheres),
            "sampled_primary_load_spheres": len(primary_spheres),
            "sampled_total_stations": len(combined),
            "evaluated_pair_count": len(candidates),
            "load_evidence_profile": v1._source_record(primary_path),
        }
    )
    if not (
        access["samples_below_100_s"] == 0
        and access["samples_at_or_after_155p045_s"] == 0
        and access["reference_hs_count"] == EXPECTED_REFERENCE_HS
        and access["reference_to_count"] == EXPECTED_REFERENCE_TO
        and access["reference_cycle_count"] == EXPECTED_CYCLES
        and access["sampled_unique_detector_stations"] == DETECTOR_STATIONS
        and access["sampled_primary_load_spheres"] == PRIMARY_LOAD_SPHERES
        and access["sampled_total_stations"] == TOTAL_STATIONS
        and access["evaluated_pair_count"] == PAIR_COUNT
    ):
        raise ProtocolError("V13 sealed sampling audit failed")
    return streams, access


def evaluate_candidate(
    protocol: Mapping[str, Any],
    candidate: Any,
    common: Mapping[str, Any],
    *,
    sample_dt_s: float,
    stage_label: str,
) -> tuple[dict[str, Any], dict[str, Any]]:
    row, detail = v6.evaluate_placement(
        protocol,
        candidate,
        common,
        sample_dt_s=sample_dt_s,
    )
    global_window = v10._window_event_metrics(
        detail, start_s=SEALED_START_S, end_s=SEALED_END_S
    )
    row.update(
        {
            "v13_stage": stage_label,
            "heel_x_shift_mm": float(candidate.geometry["heel_x_shift_mm"]),
            "toe_center_down_mm": float(candidate.geometry["toe_center_down_mm"]),
            "v13_decision_role": candidate.role,
            "global_max_signed_hs_error_s": global_window[
                "heel_strike_confirmed_max_signed_error_s"
            ],
            "global_min_signed_hs_error_s": global_window[
                "heel_strike_confirmed_min_signed_error_s"
            ],
            "global_mean_signed_hs_error_s": global_window[
                "heel_strike_confirmed_mean_signed_error_s"
            ],
            "global_mean_abs_hs_error_s": global_window[
                "heel_strike_confirmed_mean_abs_error_s"
            ],
            "global_max_signed_toe_off_error_s": global_window[
                "toe_off_confirmed_max_signed_error_s"
            ],
            "global_min_signed_toe_off_error_s": global_window[
                "toe_off_confirmed_min_signed_error_s"
            ],
            "global_mean_signed_toe_off_error_s": global_window[
                "toe_off_confirmed_mean_signed_error_s"
            ],
            "global_mean_abs_toe_off_error_s": global_window[
                "toe_off_confirmed_mean_abs_error_s"
            ],
        }
    )
    detail["v13_geometry"] = {
        "heel_x_shift_mm": float(candidate.geometry["heel_x_shift_mm"]),
        "toe_center_down_mm": float(candidate.geometry["toe_center_down_mm"]),
        "decision_role": candidate.role,
    }
    detail["v13_window_diagnostics"] = {
        "global_sealed_100_155p045": global_window
    }
    return row, detail


def _required_primary_metrics_finite(row: Mapping[str, Any]) -> bool:
    fields = (
        "precision",
        "recall",
        "max_abs_hs_error_s",
        "max_abs_toe_off_error_s",
        "confirmed_fsm_stance_f1",
        "confirmed_fsm_stance_iou",
        "minimum_causal_toe_clear_before_next_hs_onset_s",
        "global_mean_signed_hs_error_s",
        "global_mean_signed_toe_off_error_s",
    )
    try:
        return all(math.isfinite(float(row[field])) for field in fields)
    except (KeyError, TypeError, ValueError):
        return False


def decide_primary(
    rows_by_dt: Mapping[str, Sequence[Mapping[str, Any]]],
    protocol: Mapping[str, Any],
) -> dict[str, Any]:
    expected_labels = {"runtime_10ms", "fine_1ms"}
    if set(rows_by_dt) != expected_labels:
        raise ProtocolError("V13 decision requires exactly two frozen cadences")
    gates: dict[str, Any] = {}
    diagnostic_gates: dict[str, Any] = {}
    for label, gate_key in (
        ("runtime_10ms", "runtime_gate_10ms"),
        ("fine_1ms", "fine_gate_1ms"),
    ):
        rows = {str(row["candidate_id"]): row for row in rows_by_dt[label]}
        if set(rows) != {PRIMARY_ID, SENSITIVITY_ID, V9_ID}:
            raise ProtocolError(f"V13 candidate scope drifted at {label}")
        if sum(bool(row.get("selectable")) for row in rows.values()) != 1:
            raise ProtocolError(f"V13 selectable scope drifted at {label}")
        primary_gate = v10.strict_gate(rows[PRIMARY_ID], protocol[gate_key])
        finite = _required_primary_metrics_finite(rows[PRIMARY_ID])
        primary_gate = {
            **primary_gate,
            "checks": {**primary_gate["checks"], "required_metrics_finite": finite},
            "ok": bool(primary_gate["ok"] and finite),
        }
        gates[label] = primary_gate
        diagnostic_gates[label] = {
            SENSITIVITY_ID: v10.strict_gate(
                rows[SENSITIVITY_ID], protocol[gate_key]
            ),
            V9_ID: v10.strict_gate(rows[V9_ID], protocol[gate_key]),
            "role": "diagnostic_only_cannot_change_primary_verdict",
        }
    ok = bool(gates["runtime_10ms"]["ok"] and gates["fine_1ms"]["ok"])
    return {
        "status": "PRIMARY_SEALED_PASS" if ok else "PRIMARY_SEALED_FAIL",
        "ok": ok,
        "primary_candidate_id": PRIMARY_ID,
        "primary_gates": gates,
        "diagnostic_gates": diagnostic_gates,
        "selector_used": False,
        "fallback_used": False,
        "sensitivity_can_rescue_primary": False,
        "v9_can_affect_primary": False,
        "automatic_promotion_allowed": False,
    }


def _heel_off_cross_cadence(
    details_by_dt: Mapping[str, Mapping[str, Any]], candidate_id: str
) -> dict[str, Any]:
    cycles_10 = details_by_dt["runtime_10ms"][candidate_id][
        "heel_to_forefoot_transfer"
    ]["cycles"]
    cycles_1 = details_by_dt["fine_1ms"][candidate_id][
        "heel_to_forefoot_transfer"
    ]["cycles"]
    by_10 = {int(item["cycle_index"]): item for item in cycles_10}
    by_1 = {int(item["cycle_index"]): item for item in cycles_1}
    shared = sorted(set(by_10) & set(by_1))
    rows: list[dict[str, Any]] = []
    missing = 0
    for index in shared:
        value_10 = by_10[index].get("heel_off_s")
        value_1 = by_1[index].get("heel_off_s")
        if value_10 is None or value_1 is None:
            missing += 1
            continue
        delta = float(value_10) - float(value_1)
        rows.append(
            {
                "cycle_index": index,
                "heel_off_10ms_s": float(value_10),
                "heel_off_1ms_s": float(value_1),
                "signed_delta_10ms_minus_1ms_s": delta,
                "absolute_delta_s": abs(delta),
            }
        )
    maximum = max((item["absolute_delta_s"] for item in rows), default=float("inf"))
    above = sum(
        item["absolute_delta_s"]
        > HEEL_OFF_CADENCE_TOLERANCE_S + NUMERIC_TOLERANCE
        for item in rows
    )
    ok = bool(
        len(rows) == EXPECTED_CYCLES
        and missing == 0
        and maximum
        <= HEEL_OFF_CADENCE_TOLERANCE_S + NUMERIC_TOLERANCE
    )
    return {
        "candidate_id": candidate_id,
        "role": "promotion_readiness_diagnostic_not_sealed_event_gate",
        "shared_cycle_count": len(shared),
        "comparable_cycle_count": len(rows),
        "missing_heel_off_count": missing,
        "tolerance_s": HEEL_OFF_CADENCE_TOLERANCE_S,
        "cycles_above_tolerance": int(above),
        "maximum_absolute_delta_s": float(maximum),
        "ok": ok,
        "cycles": rows,
    }


def _run_cadences(
    protocol: Mapping[str, Any], base: Any, candidates: Sequence[Any]
) -> tuple[
    dict[str, list[dict[str, Any]]],
    dict[str, dict[str, Any]],
    dict[str, Any],
]:
    rows_by_dt: dict[str, list[dict[str, Any]]] = {}
    details_by_dt: dict[str, dict[str, Any]] = {}
    access_by_dt: dict[str, Any] = {}
    for label, sample_dt_s in (
        ("runtime_10ms", PRIMARY_DT_S),
        ("fine_1ms", FINE_DT_S),
    ):
        streams, access = sample_streams_once(
            protocol,
            base,
            candidates,
            sample_dt_s=sample_dt_s,
            stage_label=f"sealed_{label}",
        )
        rows: list[dict[str, Any]] = []
        details: dict[str, Any] = {}
        for candidate in candidates:
            row, detail = evaluate_candidate(
                protocol,
                candidate,
                streams[candidate.candidate_id],
                sample_dt_s=sample_dt_s,
                stage_label=f"sealed_{label}",
            )
            rows.append(row)
            details[candidate.candidate_id] = detail
        rows_by_dt[label] = rows
        details_by_dt[label] = details
        access_by_dt[label] = access
    if (
        access_by_dt["runtime_10ms"]["reference_event_sha256"]
        != access_by_dt["fine_1ms"]["reference_event_sha256"]
    ):
        raise ProtocolError("V13 cadences did not reuse one common cycle set")
    v11._augment_stage_diagnostics(
        rows_by_dt, details_by_dt, comparator_id=V9_ID
    )
    return rows_by_dt, details_by_dt, access_by_dt


def _write_receipt(
    output_dir: Path,
    protocol: Mapping[str, Any],
    lock: Mapping[str, Any],
    candidate_ids: Sequence[str],
) -> Path:
    try:
        output_dir.mkdir(parents=True, exist_ok=False)
    except FileExistsError as exc:
        raise NoClobberError(
            f"refusing to modify existing path: {v1._portable_path(output_dir)}"
        ) from exc
    receipt_path = output_dir / "sealed_access_receipt.json"
    payload = {
        "schema_version": SCHEMA_VERSION,
        "status": "OPENED_FOR_SINGLE_AUTHORIZED_V13_RUN",
        "sealed_access_started": True,
        "sealed_interval_s": [SEALED_START_S, SEALED_END_S],
        "protocol": {
            "path": v1._portable_path(Path(str(protocol["_protocol_path"]))),
            "sha256": protocol["_protocol_sha256"],
            "protocol_id": protocol["protocol_id"],
        },
        "candidate_lock": {
            "path": v1._portable_path(Path(str(lock["_lock_path"]))),
            "sha256": lock["_lock_sha256"],
            "lock_id": lock["lock_id"],
        },
        "candidate_ids_in_fixed_order": list(candidate_ids),
        "primary_candidate_id": PRIMARY_ID,
        "no_rerun_without_new_explicit_recovery_authorization": True,
    }
    receipt_path.write_text(
        json.dumps(payload, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    return receipt_path


def run_sealed_validation(
    protocol: Mapping[str, Any],
    lock: Mapping[str, Any],
    output_dir: Path,
    plot_dir: Path,
) -> dict[str, Any]:
    v11._preflight_no_clobber(output_dir, plot_dir)
    _validate_lock_binding(protocol, lock)

    # The receipt precedes even the transitive V12 integrity check, whose
    # source hashing reads the full-span GRF file without decoding events.
    receipt_path = _write_receipt(
        output_dir,
        protocol,
        lock,
        (PRIMARY_ID, SENSITIVITY_ID, V9_ID),
    )
    base, candidates, geometry = build_frozen_candidates(protocol, lock)
    rows, details, access = _run_cadences(protocol, base, candidates)
    decision = decide_primary(rows, protocol)
    heel_off_cross = _heel_off_cross_cadence(details, PRIMARY_ID)
    heel_off_vs_v9 = {
        label: v11._heel_off_shift_diagnostics(
            details[label][V9_ID], details[label][PRIMARY_ID]
        )
        for label in ("runtime_10ms", "fine_1ms")
    }
    for item in heel_off_vs_v9.values():
        item["role"] = "diagnostic_only_cannot_change_primary_verdict"

    plot_dir.mkdir(parents=True, exist_ok=False)
    csv_10 = output_dir / "sealed_v13_runtime_10ms_metrics.csv"
    csv_1 = output_dir / "sealed_v13_fine_1ms_metrics.csv"
    plot_path = plot_dir / "01_sealed_v13_multiresolution.png"
    v11._write_csv(csv_10, rows["runtime_10ms"])
    v11._write_csv(csv_1, rows["fine_1ms"])
    v11._plot_stage(
        plot_path,
        rows["runtime_10ms"],
        rows["fine_1ms"],
        x_key="toe_center_down_mm",
        x_label="toe center local-y down from V9 [mm]",
        title="V13 — frozen-primary sealed validation (confirmed time)",
        highlighted_ids=(PRIMARY_ID,),
    )

    event_ok = bool(decision["ok"])
    robustness_ok = bool(heel_off_cross["ok"])
    conclusion = (
        "PRIMARY_SEALED_PASS_HEEL_OFF_ROBUSTNESS_PASS"
        if event_ok and robustness_ok
        else "PRIMARY_SEALED_PASS_HEEL_OFF_ROBUSTNESS_FAIL"
        if event_ok
        else "PRIMARY_SEALED_FAIL"
    )
    manifest = {
        "schema_version": SCHEMA_VERSION,
        "status": "PASS" if event_ok else "FAIL",
        "ok": event_ok,
        "stage": protocol["stage"],
        "objective": protocol["objective"],
        "conclusion": conclusion,
        "protocol": {
            "path": v1._portable_path(Path(str(protocol["_protocol_path"]))),
            "sha256": protocol["_protocol_sha256"],
            "protocol_id": protocol["protocol_id"],
            "frozen_before_execution": True,
        },
        "candidate_lock": {
            "path": v1._portable_path(Path(str(lock["_lock_path"]))),
            "sha256": lock["_lock_sha256"],
            "lock_id": lock["lock_id"],
        },
        "sealed_access": {
            "sealed_block_s": [SEALED_START_S, SEALED_END_S],
            "sealed_block_opened": True,
            "single_authorized_open": True,
            "receipt": v1._source_record(receipt_path),
            "runtime_10ms": access["runtime_10ms"],
            "fine_1ms": access["fine_1ms"],
            "common_reference_cycle_set": bool(
                access["runtime_10ms"]["reference_event_sha256"]
                == access["fine_1ms"]["reference_event_sha256"]
            ),
        },
        "detector_contract": {
            "sensors_per_pair": 2,
            "sensor_roles": ["heel", "forefoot"],
            "event_guard_source": "candidate_two_sensor_forces_only",
            "normal_force_bw_source": "primary_online_grf_left_aggregate",
            "in_contact_source": "primary_online_grf_left_union_physical_penetration",
            "detector_spheres_generate_grf": False,
            "primary_event_time_field": "confirmed_time_s",
            "diagnostic_event_time_field": "event_time_s",
            "thresholds_dwell_fsm_routing_and_radii_unchanged": True,
        },
        "geometry": geometry,
        "candidates": [v11._candidate_record(item) for item in candidates],
        "runtime_10ms": {
            "rows": rows["runtime_10ms"],
            "details": details["runtime_10ms"],
        },
        "fine_1ms": {
            "rows": rows["fine_1ms"],
            "details": details["fine_1ms"],
        },
        "decision": decision,
        "robustness": {
            "heel_off_cross_cadence": heel_off_cross,
            "heel_off_primary_vs_v9": heel_off_vs_v9,
            "affects_primary_sealed_event_verdict": False,
            "promotion_readiness": (
                "REQUIRES_SEPARATE_PROMOTION_DECISION"
                if event_ok and robustness_ok
                else "BLOCKED_BY_HEEL_OFF_CADENCE_INSTABILITY"
                if event_ok
                else "BLOCKED_BY_PRIMARY_SEALED_FAILURE"
            ),
        },
        "artifacts": {
            "runtime_metrics_csv": v1._source_record(csv_10),
            "fine_metrics_csv": v1._source_record(csv_1),
            "multiresolution_plot": v1._source_record(plot_path),
        },
        "source_identity": {
            label: v1._source_record(
                v1.resolve_repo_path(str(record["path"])).resolve()
            )
            for label, record in protocol["sources"].items()
        },
        "non_actions": {
            "prior_v9_v10_v11_v12_modified": False,
            "threshold_or_fsm_changed": False,
            "policy_or_training_run": False,
            "runtime_configuration_modified": False,
            "candidate_profile_created": False,
            "candidate_profile_promoted": False,
            "fallback_or_post_hoc_selection_used": False,
        },
        "interpretation_limits": protocol["interpretation_limits"],
    }
    safe = v1._json_safe(manifest)
    manifest_path = output_dir / "manifest.json"
    manifest_path.write_text(
        json.dumps(safe, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    return safe


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run the one-shot frozen-primary V13 sealed validation."
    )
    parser.add_argument("--protocol", default=str(DEFAULT_PROTOCOL))
    parser.add_argument("--candidate-lock", default=str(DEFAULT_LOCK))
    parser.add_argument("--output-dir", default=str(DEFAULT_OUTPUT_DIR))
    parser.add_argument("--plot-dir", default=str(DEFAULT_PLOT_DIR))
    return parser


def _write_failure_after_open(output_dir: Path, exc: Exception) -> None:
    if not output_dir.is_dir():
        return
    failure_path = output_dir / "failure.json"
    if failure_path.exists():
        return
    failure_path.write_text(
        json.dumps(
            {
                "schema_version": SCHEMA_VERSION,
                "status": "ERROR_AFTER_SEALED_ACCESS_STARTED",
                "ok": False,
                "sealed_block_opened": True,
                "rerun_allowed": False,
                "error": f"{type(exc).__name__}: {exc}",
                "traceback": traceback.format_exc(),
            },
            indent=2,
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )


def main(argv: Sequence[str] | None = None) -> int:
    args = build_arg_parser().parse_args(argv)
    output_dir = v1.resolve_repo_path(args.output_dir).resolve()
    plot_dir = v1.resolve_repo_path(args.plot_dir).resolve()
    try:
        v11._preflight_no_clobber(output_dir, plot_dir)
        protocol = load_and_validate_protocol(args.protocol)
        lock = load_and_validate_lock(args.candidate_lock)
        manifest = run_sealed_validation(protocol, lock, output_dir, plot_dir)
    except NoClobberError as exc:
        print(
            json.dumps(
                {
                    "schema_version": SCHEMA_VERSION,
                    "status": "ERROR",
                    "ok": False,
                    "no_clobber": True,
                    "filesystem_mutated": False,
                    "error": f"{type(exc).__name__}: {exc}",
                },
                indent=2,
            )
        )
        return 2
    except Exception as exc:  # pragma: no cover - integration failure path.
        _write_failure_after_open(output_dir, exc)
        print(
            json.dumps(
                {
                    "schema_version": SCHEMA_VERSION,
                    "status": "ERROR",
                    "ok": False,
                    "sealed_block_opened": output_dir.exists(),
                    "rerun_allowed": not output_dir.exists(),
                    "error": f"{type(exc).__name__}: {exc}",
                    "traceback": traceback.format_exc(),
                },
                indent=2,
            )
        )
        return 2
    print(
        json.dumps(
            {
                "status": manifest["status"],
                "conclusion": manifest["conclusion"],
                "primary_candidate_id": PRIMARY_ID,
                "sealed_block_opened": True,
                "manifest": v1._portable_path(output_dir / "manifest.json"),
            },
            indent=2,
        )
    )
    return 0 if manifest["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
