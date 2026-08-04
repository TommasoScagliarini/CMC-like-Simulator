"""Development-only V16 OFAT sweep of V13 heel height and radius.

V16 evaluates the frozen V13 baseline plus two separate one-factor arms:

* ``Hy`` moves the heel-sphere centre in plantar local-y;
* ``Hr`` increases the heel-sphere radius without moving its centre.

Every candidate is replayed through the full production ProstheticPhaseFSM on
DEV02/04/08 at 10 ms and 1 ms.  V16 never uses the V15 signal-only router for
selection, never combines the two geometry factors, and has no execution path
to validation, sealed, or reserve trials.
"""

from __future__ import annotations

import argparse
import copy
import csv
import hashlib
import json
import math
import sys
import traceback
from dataclasses import replace
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
VALIDATION_ROOT = REPO_ROOT / "validation"
for _path in (REPO_ROOT, VALIDATION_ROOT):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

import diagnose_two_sensor_v15_routing as v15  # noqa: E402
import sweep_two_sensor_cross_speed_v14_2 as v142  # noqa: E402


v14 = v142.v14
v141 = v142.v141
v1 = v14.v1

SCHEMA_VERSION = 16
PROTOCOL_ID = "AB06_TWO_SENSOR_HEEL_GEOMETRY_OFAT_2026-07-22_V16"
BASELINE_ID = "V13_BASELINE"
ALLOWED_TRIALS = ("02", "04", "08")
FORBIDDEN_TRIALS = ("01", "03", "05", "06", "07")
CADENCES = (("runtime_10ms", 0.010), ("fine_1ms", 0.001))
DELTA_GRID_MM = (
    0.15,
    0.25,
    0.35,
    0.45,
    0.55,
    0.65,
    0.75,
    0.85,
    0.95,
    1.05,
    1.15,
    1.25,
)
ARM_ORDER = ("Hy", "Hr")
EXPECTED_CANDIDATE_COUNT = 1 + len(ARM_ORDER) * len(DELTA_GRID_MM)
EXPECTED_UNIT_COUNT = len(ALLOWED_TRIALS) * 4 * len(CADENCES)
EXPECTED_ROW_COUNT = EXPECTED_CANDIDATE_COUNT * EXPECTED_UNIT_COUNT
EXPECTED_STATION_COUNT = 1 + (1 + len(DELTA_GRID_MM))
NUMERIC_TOLERANCE = 1.0e-12

DEFAULT_PROTOCOL = VALIDATION_ROOT / "two_sensor_heel_geometry_v16_protocol.json"
DEFAULT_OUTPUT_DIR = (
    VALIDATION_ROOT
    / "two_sensor_heel_geometry_v16_runs/"
    "2026-07-22_ab06_dev02_04_08_v16_hy_hr_ofat"
)

EXPECTED_SOURCE_KEYS = {
    "v16_runner",
    "v16_tests",
    "production_fsm",
    "v14_2_runner",
    "v14_runner",
    "v15_runner",
    "v14_2_protocol",
    "v14_2_manifest",
    "v14_2_stage1_details",
    "v15_protocol",
    "v15_run_start_receipt",
    "v15_failure",
    "v13_profile",
}
EXPECTED_PREPROCESSING_KEYS = {
    "preprocessing_lock",
    "ik_motion",
    "external_loads",
    "grf",
}


class ProtocolError(RuntimeError):
    """Raised when the frozen V16 contract is violated."""


class NoClobberError(ProtocolError):
    """Raised when the canonical V16 destination is already consumed."""


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _canonical_sha256(value: Any) -> str:
    raw = json.dumps(
        value,
        sort_keys=True,
        separators=(",", ":"),
        allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(raw).hexdigest()


def _portable(path: Path) -> str:
    resolved = path.resolve()
    try:
        return resolved.relative_to(REPO_ROOT).as_posix()
    except ValueError:
        return resolved.as_posix()


def _resolve(path: str | Path) -> Path:
    value = Path(path)
    return (REPO_ROOT / value).resolve() if not value.is_absolute() else value.resolve()


def _load_object(path: Path, *, label: str) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ProtocolError(f"cannot load {label}: {_portable(path)}") from exc
    if not isinstance(value, dict):
        raise ProtocolError(f"{label} is not a JSON object")
    return value


def _validate_source(record: Mapping[str, Any], *, label: str) -> Path:
    if set(record) != {"path", "sha256"}:
        raise ProtocolError(f"invalid V16 source record: {label}")
    path = _resolve(str(record["path"]))
    if not path.is_file() or _sha256(path) != str(record["sha256"]):
        raise ProtocolError(f"V16 source identity drifted: {label}")
    return path


def _require_sequence(
    observed: Iterable[Any], expected: Sequence[Any], *, label: str
) -> None:
    if list(observed) != list(expected):
        raise ProtocolError(f"V16 {label} drifted")


def load_and_validate_protocol(
    path: str | Path = DEFAULT_PROTOCOL,
) -> dict[str, Any]:
    protocol_path = _resolve(path)
    if protocol_path != DEFAULT_PROTOCOL.resolve():
        raise ProtocolError("V16 accepts only its canonical protocol path")
    raw = _load_object(protocol_path, label="V16 protocol")
    checks = {
        "schema": raw.get("schema_version") == SCHEMA_VERSION,
        "id": raw.get("protocol_id") == PROTOCOL_ID,
        "frozen": raw.get("frozen_before_execution") is True,
        "stage": raw.get("stage") == "development_only_full_fsm_geometry_ofat",
        "full_fsm": raw.get("replay", {}).get("full_production_fsm") is True,
        "no_router": raw.get("replay", {}).get("signal_only_router_used") is False,
        "no_combination": raw.get("geometry", {}).get("combine_arms") is False,
        "no_promotion": raw.get("decision_contract", {}).get(
            "holdout_or_runtime_promotion_allowed"
        ) is False,
    }
    if not all(checks.values()):
        raise ProtocolError(f"V16 top-level contract drifted: {checks}")
    split = raw.get("split", {})
    _require_sequence(
        split.get("DEVELOPMENT", ()), ALLOWED_TRIALS, label="development split"
    )
    _require_sequence(
        split.get("FORBIDDEN", ()), FORBIDDEN_TRIALS, label="forbidden split"
    )
    if split.get("VALIDATION") != ["05"] or split.get("SEALED") != ["06"]:
        raise ProtocolError("V16 protected split identity drifted")
    if split.get("RESERVE") != ["03", "07"]:
        raise ProtocolError("V16 reserve split identity drifted")

    geometry = raw.get("geometry", {})
    _require_sequence(
        geometry.get("arms", ()), ARM_ORDER, label="geometry arm order"
    )
    observed_grid = tuple(float(value) for value in geometry.get("delta_grid_mm", ()))
    if observed_grid != DELTA_GRID_MM:
        raise ProtocolError("V16 geometry delta grid drifted")
    if int(geometry.get("candidate_count", -1)) != EXPECTED_CANDIDATE_COUNT:
        raise ProtocolError("V16 candidate cardinality drifted")
    if geometry.get("heel_local_y_rule") != "v13_y_m_minus_delta_mm_over_1000":
        raise ProtocolError("V16 Hy direction drifted")
    if geometry.get("heel_radius_rule") != "v13_radius_m_plus_delta_mm_over_1000":
        raise ProtocolError("V16 Hr direction drifted")
    profile_record = geometry.get("profile", {})
    if profile_record != raw.get("sources", {}).get("v13_profile"):
        raise ProtocolError("V16 geometry/profile source binding drifted")

    replay = raw.get("replay", {})
    if not (
        replay.get("continuous_trial_no_plateau_reset") is True
        and replay.get("sensor_on_threshold_n") == 0.5
        and replay.get("sensor_off_threshold_n") == 0.25
        and replay.get("sensor_dwell_s") == 0.03
        and replay.get("primary_event_time_field") == "confirmed_time_s"
        and replay.get("cadences")
        == {label: dt for label, dt in CADENCES}
    ):
        raise ProtocolError("V16 replay contract drifted")

    sources = raw.get("sources", {})
    if set(sources) != EXPECTED_SOURCE_KEYS:
        raise ProtocolError("V16 source keyset drifted")
    for name, record in sources.items():
        _validate_source(record, label=name)

    gate = raw.get("gate_contract", {})
    selection = raw.get("selection_contract", {})
    destination = raw.get("execution_destination", {})
    if not (
        gate.get("inheritance") == "frozen_v14_2_exact"
        and gate.get("pass_all_24_units") is True
        and gate.get("pass_all_6_trial_cadence_aggregates") is True
        and gate.get("zero_rejection_timeout_required") is True
        and selection.get("pareto_tolerance") == v14.PARETO_TOLERANCE
        and selection.get("v13_wins_tie") is True
        and selection.get("outer_boundary_delta_mm") == DELTA_GRID_MM[-1]
        and destination.get("canonical_output_dir")
        == _portable(DEFAULT_OUTPUT_DIR)
        and destination.get("alternate_output_directory_allowed") is False
        and destination.get("one_shot_no_clobber") is True
    ):
        raise ProtocolError("V16 gate/selection/destination contract drifted")

    preprocessing = raw.get("preprocessing", {})
    if not (
        preprocessing.get("reuse_only") is True
        and preprocessing.get("raw_semantic_decode_allowed") is False
        and preprocessing.get("ik_rerun_allowed") is False
        and set(preprocessing.get("trials", {})) == set(ALLOWED_TRIALS)
    ):
        raise ProtocolError("V16 preprocessing scope drifted")
    for trial_id in ALLOWED_TRIALS:
        records = preprocessing["trials"][trial_id]
        if set(records) != EXPECTED_PREPROCESSING_KEYS:
            raise ProtocolError(f"V16 preprocessing keys drifted for {trial_id}")
        parent = (
            v15.PARENT_RUN_DIR / "preprocessed" / f"trial_{trial_id}"
        ).resolve()
        for name, record in records.items():
            path_value = _validate_source(record, label=f"{trial_id}/{name}")
            try:
                path_value.relative_to(parent)
            except ValueError as exc:
                raise ProtocolError(
                    f"V16 preprocessing source escaped parent for {trial_id}"
                ) from exc

    raw["_protocol_path"] = protocol_path.as_posix()
    raw["_protocol_sha256"] = _sha256(protocol_path)
    return raw


def validate_parent_state(protocol: Mapping[str, Any]) -> dict[str, Any]:
    manifest_path = _validate_source(
        protocol["sources"]["v14_2_manifest"], label="v14_2_manifest"
    )
    v15_receipt_path = _validate_source(
        protocol["sources"]["v15_run_start_receipt"],
        label="v15_run_start_receipt",
    )
    v15_failure_path = _validate_source(
        protocol["sources"]["v15_failure"], label="v15_failure"
    )
    manifest = _load_object(manifest_path, label="V14.2 manifest")
    receipt = _load_object(v15_receipt_path, label="V15 receipt")
    failure = _load_object(v15_failure_path, label="V15 failure")
    checks = {
        "v14_2_failed": manifest.get("status") == "FAIL",
        "v13_retained": manifest.get("conclusion")
        == "V13_RETAINED_NO_DEVELOPMENT_FINALIST",
        "v14_2_validation_unopened": manifest.get("validation", {}).get("opened")
        is False,
        "v14_2_sealed_unopened": manifest.get("sealed", {}).get("opened") is False,
        "v14_2_reserve_unopened": manifest.get("reserve", {}).get("opened") is False,
        "v15_consumed": receipt.get("status")
        == "V15_DEVELOPMENT_DIAGNOSTIC_STARTED",
        "v15_development_exact": receipt.get("authorized_trials")
        == list(ALLOWED_TRIALS),
        "v15_failure": failure.get("status")
        == "ERROR_AFTER_V15_DESTINATION_CONSUMED",
        "v15_failure_exact": failure.get("error")
        == "ProtocolError: V15 incumbent full HS sequence does not match frozen V14.2",
        "v15_rerun_forbidden": failure.get("rerun_allowed") is False,
        "v15_protected_unopened": all(
            failure.get(key) is False
            for key in ("validation_opened", "sealed_opened", "reserve_opened")
        ),
    }
    if not all(checks.values()):
        raise ProtocolError(f"V16 parent state drifted: {checks}")
    return {
        "status": "PASS_V16_PARENT_FAILURES_AND_PROTECTED_SPLITS_VERIFIED",
        "checks": checks,
        "v14_2_manifest_sha256": _sha256(manifest_path),
        "v15_receipt_sha256": _sha256(v15_receipt_path),
        "v15_failure_sha256": _sha256(v15_failure_path),
    }


def _token(value: float) -> str:
    return f"{value:.2f}".replace(".", "p")


def _updated_geometry(
    baseline: Any,
    *,
    arm: str,
    delta_mm: float,
    triangles: np.ndarray,
) -> tuple[tuple[float, float, float], float, dict[str, Any]]:
    if arm not in ARM_ORDER or delta_mm not in DELTA_GRID_MM:
        raise ProtocolError("V16 candidate escaped frozen arm/grid")
    heel_location = tuple(float(value) for value in baseline.heel_location)
    heel_radius = float(baseline.geometry["heel_radius_m"])
    if arm == "Hy":
        heel_location = (
            heel_location[0],
            heel_location[1] - delta_mm / 1000.0,
            heel_location[2],
        )
    else:
        heel_radius += delta_mm / 1000.0
    toe_location = tuple(float(value) for value in baseline.forefoot_location)
    toe_radius = float(baseline.geometry["toe_radius_m"])
    heel_mesh = v14.v10.v9.v8.v6.v3._mesh_geometry(
        heel_location, heel_radius, triangles
    )
    toe_mesh = v14.v10.v9.v8.v6.v3._mesh_geometry(
        toe_location, toe_radius, triangles
    )
    bottom_offset = (toe_location[1] - toe_radius) - (
        heel_location[1] - heel_radius
    )
    checks = {
        "exactly_two_spheres": True,
        "heel_within_5mm_of_mesh": bool(heel_mesh["within_5mm_of_mesh"]),
        "toe_within_5mm_of_mesh": bool(toe_mesh["within_5mm_of_mesh"]),
        "positive_radii": heel_radius > 0.0 and toe_radius > 0.0,
        "heel_to_toe_bottom_offset_within_20mm": abs(bottom_offset)
        <= 0.020 + NUMERIC_TOLERANCE,
        "exactly_one_factor_changed": True,
    }
    geometry = copy.deepcopy(dict(baseline.geometry))
    geometry.update(
        {
            "source": "v16_v13_heel_geometry_ofat",
            "parameter_arm": arm,
            "heel_center_down_from_v13_mm": delta_mm if arm == "Hy" else 0.0,
            "heel_radius_increase_from_v13_mm": delta_mm if arm == "Hr" else 0.0,
            "heel_radius_reduction_from_v13_mm": -delta_mm
            if arm == "Hr"
            else 0.0,
            "heel_center_raise_mm": -delta_mm if arm == "Hy" else 0.0,
            "heel_radius_reduction_mm": -delta_mm if arm == "Hr" else 0.0,
            "radius_reduction_mm": -delta_mm if arm == "Hr" else 0.0,
            "heel_radius_m": heel_radius,
            "toe_radius_m": toe_radius,
            "effective_bottom_raise_mm": -delta_mm,
            "geometry_displacement_from_v13_m": delta_mm / 1000.0,
            "heel_to_toe_bottom_offset_m": bottom_offset,
            "heel": heel_mesh,
            "toe": toe_mesh,
            "pre_gate_checks": checks,
            "pre_gate_ok": bool(all(checks.values())),
            "v16_one_factor_only": True,
            "v16_hy_hr_combined": False,
        }
    )
    return heel_location, heel_radius, geometry


def build_candidates(
    protocol: Mapping[str, Any],
) -> tuple[Any, list[Any], dict[str, Any]]:
    v14_protocol = v142.load_and_validate_protocol(
        protocol["sources"]["v14_2_protocol"]["path"]
    )
    base, inherited, summary = v14.build_isolated_candidates(v14_protocol)
    baseline = next(
        candidate for candidate in inherited if candidate.candidate_id == BASELINE_ID
    )
    triangles = np.asarray(summary["triangles"], dtype=float)
    candidates = [baseline]
    for arm in ARM_ORDER:
        for delta_mm in DELTA_GRID_MM:
            heel_location, _radius, geometry = _updated_geometry(
                baseline,
                arm=arm,
                delta_mm=delta_mm,
                triangles=triangles,
            )
            candidates.append(
                replace(
                    baseline,
                    candidate_id=f"V16_{arm}_{_token(delta_mm)}mm",
                    heel_location=heel_location,
                    selectable=True,
                    role="v16_development_only_full_fsm_ofat",
                    geometry=geometry,
                )
            )
    if (
        len(candidates) != EXPECTED_CANDIDATE_COUNT
        or len({item.candidate_id for item in candidates}) != len(candidates)
        or sum(item.candidate_id == BASELINE_ID for item in candidates) != 1
        or any(not item.geometry["pre_gate_ok"] for item in candidates)
    ):
        raise ProtocolError("V16 candidate construction failed closed")
    for candidate in candidates[1:]:
        delta = float(candidate.geometry["geometry_displacement_from_v13_m"])
        changed_y = not math.isclose(
            float(candidate.heel_location[1]),
            float(baseline.heel_location[1]),
            abs_tol=NUMERIC_TOLERANCE,
        )
        changed_r = not math.isclose(
            float(candidate.geometry["heel_radius_m"]),
            float(baseline.geometry["heel_radius_m"]),
            abs_tol=NUMERIC_TOLERANCE,
        )
        if changed_y == changed_r or delta <= 0.0:
            raise ProtocolError("V16 candidate is not one-factor")
    return base, candidates, {
        "baseline": v14._candidate_record(baseline),
        "candidate_count": len(candidates),
        "detector_station_count": EXPECTED_STATION_COUNT,
        "mesh": summary["mesh"],
    }


def _preflight_inputs(protocol: Mapping[str, Any]) -> dict[str, Any]:
    parent = validate_parent_state(protocol)
    v15_protocol = v15.load_and_validate_protocol(
        protocol["sources"]["v15_protocol"]["path"]
    )
    locks: dict[str, Any] = {}
    for trial_id in ALLOWED_TRIALS:
        artifacts = v15._validate_preprocessing_trial(v15_protocol, trial_id)
        locks[trial_id] = {
            "preprocessing_lock_sha256": _sha256(artifacts.lock_path),
            "analysis_interval_s": list(artifacts.lock["analysis_interval_s"]),
        }
    _base, candidates, geometry = build_candidates(protocol)
    return {
        "status": "PASS_V16_FULL_LIVE_INPUT_PREFLIGHT",
        "parent": parent,
        "preprocessing": locks,
        "candidate_count": len(candidates),
        "detector_station_count": geometry["detector_station_count"],
        "canonical_destination_unconsumed": not DEFAULT_OUTPUT_DIR.exists(),
        "protected_trials_opened": False,
    }


def _write_json_exclusive(path: Path, payload: Mapping[str, Any]) -> Path:
    with path.open("x", encoding="utf-8") as handle:
        json.dump(
            v1._json_safe(dict(payload)),
            handle,
            indent=2,
            sort_keys=True,
            allow_nan=False,
        )
        handle.write("\n")
    return path


def _write_jsonl_exclusive(
    path: Path, rows: Sequence[Mapping[str, Any]]
) -> Path:
    with path.open("x", encoding="utf-8") as handle:
        for row in rows:
            handle.write(
                json.dumps(
                    v1._json_safe(dict(row)),
                    sort_keys=True,
                    separators=(",", ":"),
                    allow_nan=False,
                )
                + "\n"
            )
    return path


def _write_csv_exclusive(path: Path, rows: Sequence[Mapping[str, Any]]) -> Path:
    if not rows:
        raise ProtocolError("refusing empty V16 CSV")
    fields = sorted({str(key) for row in rows for key in row})
    with path.open("x", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(v1._json_safe(list(rows)))
    return path


def _artifact(path: Path) -> dict[str, Any]:
    return {"path": _portable(path), "sha256": _sha256(path), "bytes": path.stat().st_size}


def _preflight_no_clobber(output_dir: Path) -> None:
    if output_dir.resolve() != DEFAULT_OUTPUT_DIR.resolve():
        raise NoClobberError("V16 execution is restricted to its canonical destination")
    if output_dir.exists():
        raise NoClobberError(f"V16 destination exists: {_portable(output_dir)}")


def _start_receipt(output_dir: Path, protocol: Mapping[str, Any]) -> Path:
    output_dir.mkdir(parents=True, exist_ok=False)
    return _write_json_exclusive(
        output_dir / "run_start_receipt.json",
        {
            "schema_version": SCHEMA_VERSION,
            "status": "V16_DEVELOPMENT_FULL_FSM_GEOMETRY_SWEEP_STARTED",
            "protocol": {
                "path": _portable(Path(str(protocol["_protocol_path"]))),
                "sha256": protocol["_protocol_sha256"],
                "protocol_id": protocol["protocol_id"],
            },
            "authorized_trials": list(ALLOWED_TRIALS),
            "forbidden_trials": list(FORBIDDEN_TRIALS),
            "candidate_count": EXPECTED_CANDIDATE_COUNT,
            "expected_unit_rows": EXPECTED_ROW_COUNT,
            "full_production_fsm": True,
            "signal_only_router_used": False,
            "runtime_or_fsm_modified": False,
            "holdout_or_reserve_access_allowed": False,
            "rerun_or_alternate_destination_allowed": False,
        },
    )


def _load_parent_baseline_details(
    protocol: Mapping[str, Any],
) -> dict[tuple[str, int, str], dict[str, Any]]:
    path = _validate_source(
        protocol["sources"]["v14_2_stage1_details"],
        label="v14_2_stage1_details",
    )
    result: dict[tuple[str, int, str], dict[str, Any]] = {}
    for item in v15._jsonl_objects(path):
        row = item.get("row", {})
        if row.get("candidate_id") != BASELINE_ID:
            continue
        key = (
            str(row["trial_id"]),
            int(row["plateau_index"]),
            str(row["cadence"]),
        )
        if key in result:
            raise ProtocolError(f"duplicate V14.2 baseline detail: {key}")
        result[key] = item
    expected = {
        (trial_id, plateau, cadence)
        for trial_id in ALLOWED_TRIALS
        for plateau in range(1, 5)
        for cadence, _dt in CADENCES
    }
    if set(result) != expected:
        raise ProtocolError("V14.2 baseline detail universe drifted")
    return result


def _normalized_detail(detail: Mapping[str, Any]) -> dict[str, Any]:
    result = copy.deepcopy(v1._json_safe(dict(detail)))
    result.pop("v14_stage", None)
    row = result.get("row")
    if isinstance(row, dict):
        row.pop("v14_stage", None)
    return result


def baseline_anchor_parity(
    protocol: Mapping[str, Any],
    details: Sequence[Mapping[str, Any]],
) -> dict[str, Any]:
    expected = _load_parent_baseline_details(protocol)
    observed: dict[tuple[str, int, str], Mapping[str, Any]] = {}
    for detail in details:
        row = detail.get("row", {})
        if row.get("candidate_id") != BASELINE_ID:
            continue
        key = (
            str(row["trial_id"]),
            int(row["plateau_index"]),
            str(row["cadence"]),
        )
        if key in observed:
            raise ProtocolError(f"duplicate V16 baseline detail: {key}")
        observed[key] = detail
    records: list[dict[str, Any]] = []
    for key in sorted(expected):
        expected_normalized = _normalized_detail(expected[key])
        current = observed.get(key)
        observed_normalized = (
            None if current is None else _normalized_detail(current)
        )
        expected_hash = _canonical_sha256(expected_normalized)
        observed_hash = (
            None
            if observed_normalized is None
            else _canonical_sha256(observed_normalized)
        )
        records.append(
            {
                "trial_id": key[0],
                "plateau_index": key[1],
                "cadence": key[2],
                "expected_sha256": expected_hash,
                "observed_sha256": observed_hash,
                "ok": observed_hash == expected_hash,
            }
        )
    expected_aggregate = _canonical_sha256(
        [_normalized_detail(expected[key]) for key in sorted(expected)]
    )
    observed_aggregate = (
        _canonical_sha256(
            [_normalized_detail(observed[key]) for key in sorted(observed)]
        )
        if set(observed) == set(expected)
        else None
    )
    ok = bool(
        set(observed) == set(expected)
        and all(record["ok"] for record in records)
        and observed_aggregate == expected_aggregate
    )
    return {
        "status": (
            "PASS_EXACT_V14_2_BASELINE_FULL_FSM_DETAIL_PARITY"
            if ok
            else "FAIL_V14_2_BASELINE_FULL_FSM_DETAIL_PARITY"
        ),
        "ok": ok,
        "unit_count": len(records),
        "expected_aggregate_sha256": expected_aggregate,
        "observed_aggregate_sha256": observed_aggregate,
        "records": records,
    }


def _candidate_groups(
    rows: Sequence[Mapping[str, Any]],
) -> dict[str, list[Mapping[str, Any]]]:
    result: dict[str, list[Mapping[str, Any]]] = {}
    for row in rows:
        result.setdefault(str(row["candidate_id"]), []).append(row)
    if any(len(group) != EXPECTED_UNIT_COUNT for group in result.values()):
        raise ProtocolError("V16 candidate unit cardinality drifted")
    return result


def _rank_key(
    candidate: Any,
    rows: Sequence[Mapping[str, Any]],
) -> tuple[Any, ...]:
    burden = v14._structural_burden(rows)
    vector = v14.minimax_vector(rows)
    return (
        burden["worst_event_count_deficit"],
        burden["sum_event_count_deficit"],
        burden["worst_valid_cycle_deficit"],
        burden["worst_incomplete_transfer_count"],
        burden["worst_invalid_plus_unaccepted_count"],
        burden["worst_forbidden_phase_mismatch_count"],
        vector["worst_joint_normalized"],
        vector["worst_hs_s"],
        vector["worst_to_s"],
        vector["equal_cell_mean_normalized_error"],
        vector["worst_f1_deficit"],
        vector["worst_iou_deficit"],
        float(candidate.geometry["geometry_displacement_from_v13_m"]),
        candidate.candidate_id,
    )


def select_development_result(
    rows: Sequence[Mapping[str, Any]], candidates: Sequence[Any]
) -> dict[str, Any]:
    groups = _candidate_groups(rows)
    by_id = {candidate.candidate_id: candidate for candidate in candidates}
    if set(groups) != set(by_id) or BASELINE_ID not in groups:
        raise ProtocolError("V16 result candidate universe drifted")
    baseline_rows = groups[BASELINE_ID]
    baseline_vector = v14.minimax_vector(baseline_rows)
    assessments: dict[str, Any] = {}
    eligible: list[str] = []
    for candidate in candidates[1:]:
        candidate_rows = groups[candidate.candidate_id]
        vector = v14.minimax_vector(candidate_rows)
        full_gate = v14._full_gate_all(candidate_rows)
        pareto = v14.pareto_challenge(vector, baseline_vector)
        root_safe = v14.root_safe_isolated(candidate_rows, baseline_rows)
        assessment = {
            "arm": candidate.geometry["parameter_arm"],
            "delta_mm": 1000.0
            * float(candidate.geometry["geometry_displacement_from_v13_m"]),
            "passes_all_24_units_and_6_trial_cadence_aggregates": full_gate,
            "zero_rejection_timeout_in_every_unit": all(
                int(row["invalid_or_timeout_transition_count"]) == 0
                and int(row["unaccepted_sensor_gait_event_count"]) == 0
                for row in candidate_rows
            ),
            "minimax_vector": vector,
            "pareto_vs_v13": pareto,
            "root_safe": root_safe,
            "eligible": bool(full_gate and pareto["ok"]),
            "rank_key": list(_rank_key(candidate, candidate_rows)[:-1]),
        }
        assessments[candidate.candidate_id] = assessment
        if assessment["eligible"]:
            eligible.append(candidate.candidate_id)
    eligible.sort(key=lambda candidate_id: _rank_key(by_id[candidate_id], groups[candidate_id]))

    diagnostic_winners: dict[str, str] = {}
    boundary: dict[str, Any] = {}
    for arm in ARM_ORDER:
        arm_ids = [
            candidate.candidate_id
            for candidate in candidates[1:]
            if candidate.geometry["parameter_arm"] == arm
            and assessments[candidate.candidate_id]["root_safe"]["ok"]
        ]
        arm_ids.sort(key=lambda candidate_id: _rank_key(by_id[candidate_id], groups[candidate_id]))
        winner = arm_ids[0] if arm_ids else BASELINE_ID
        diagnostic_winners[arm] = winner
        outer_id = f"V16_{arm}_{_token(DELTA_GRID_MM[-1])}mm"
        inner_id = f"V16_{arm}_{_token(DELTA_GRID_MM[-2])}mm"
        improves_to_outer = bool(
            outer_id in groups
            and inner_id in groups
            and _rank_key(by_id[outer_id], groups[outer_id])
            < _rank_key(by_id[inner_id], groups[inner_id])
        )
        boundary[arm] = {
            "diagnostic_winner_id": winner,
            "winner_at_outer_boundary": winner == outer_id,
            "outer_improves_over_inner": improves_to_outer,
            "boundary_limited": winner == outer_id and improves_to_outer,
        }
    boundary_limited = any(item["boundary_limited"] for item in boundary.values())
    finalist_id = None if boundary_limited or not eligible else eligible[0]
    return {
        "status": (
            "V16_DEVELOPMENT_FINALIST_FROZEN_NO_HOLDOUT_ACCESS"
            if finalist_id is not None
            else "V13_RETAINED_NO_V16_DEVELOPMENT_FINALIST"
        ),
        "baseline_id": BASELINE_ID,
        "baseline_minimax_vector": baseline_vector,
        "finalist_id": finalist_id,
        "ranked_eligible_ids": eligible,
        "diagnostic_arm_winners": diagnostic_winners,
        "boundary_saturation": {
            "stop_before_any_future_holdout": boundary_limited,
            "arms": boundary,
        },
        "assessments": assessments,
        "v13_remains_runtime_and_sealed_baseline": True,
        "holdout_opened": False,
    }


def run_sweep(
    protocol: Mapping[str, Any], output_dir: Path = DEFAULT_OUTPUT_DIR
) -> dict[str, Any]:
    if not (output_dir / "run_start_receipt.json").is_file():
        raise ProtocolError("V16 start receipt must precede sampling")
    parent = validate_parent_state(protocol)
    v14_protocol = v142.load_and_validate_protocol(
        protocol["sources"]["v14_2_protocol"]["path"]
    )
    v15_protocol = v15.load_and_validate_protocol(
        protocol["sources"]["v15_protocol"]["path"]
    )
    base, candidates, geometry = build_candidates(protocol)
    plan = v14._build_sampling_plan(
        base,
        candidates,
        stage_label="v16_development_full_fsm",
        expected_detector_stations=EXPECTED_STATION_COUNT,
    )
    rows: list[dict[str, Any]] = []
    details: list[dict[str, Any]] = []
    access: list[dict[str, Any]] = []
    with v142._configured_runtime(v14_protocol, output_dir):
        for trial_id in ALLOWED_TRIALS:
            artifacts = v15._validate_preprocessing_trial(v15_protocol, trial_id)
            for cadence, sample_dt_s in CADENCES:
                bundle = v14.sample_trial_cadence_once(
                    v14_protocol,
                    artifacts.setup,
                    base,
                    plan,
                    trial_id=trial_id,
                    cadence_label=cadence,
                    sample_dt_s=sample_dt_s,
                )
                for candidate in candidates:
                    candidate_rows, candidate_details = (
                        v14.evaluate_continuous_candidate(
                            v14_protocol, base, candidate, bundle
                        )
                    )
                    rows.extend(candidate_rows)
                    details.extend(candidate_details)
                access.append(
                    {
                        "trial_id": trial_id,
                        "cadence": cadence,
                        "sample_dt_s": sample_dt_s,
                        "sample_count": int(bundle.shared["times"].size),
                        "candidate_count": len(candidates),
                        "detector_station_count": len(plan.sampler.spheres),
                        "reference_event_sha256": [
                            reference["reference_event_sha256"]
                            for reference in bundle.plateau_references
                        ],
                        "preprocessing_lock_sha256": _sha256(artifacts.lock_path),
                    }
                )
    if len(rows) != EXPECTED_ROW_COUNT or len(details) != EXPECTED_ROW_COUNT:
        raise ProtocolError(
            f"V16 row cardinality drifted: {len(rows)}/{len(details)}"
        )
    v14.attach_trial_aggregate_gates(rows)
    parity = baseline_anchor_parity(protocol, details)
    if not parity["ok"]:
        _write_json_exclusive(output_dir / "baseline_parity_failure.json", parity)
        raise ProtocolError("V16 baseline full-FSM anchor drifted from V14.2")

    for row in rows:
        row["v16_stage"] = "development_only_full_fsm_geometry_ofat"
    for detail in details:
        detail["v16_stage"] = "development_only_full_fsm_geometry_ofat"
    decision = select_development_result(rows, candidates)

    rows_path = _write_csv_exclusive(output_dir / "development_rows.csv", rows)
    details_path = _write_jsonl_exclusive(
        output_dir / "development_details.jsonl", details
    )
    access_path = _write_json_exclusive(
        output_dir / "development_sampling_access.json", {"records": access}
    )
    decision_path = _write_json_exclusive(
        output_dir / "development_decision_lock.json",
        {
            "schema_version": SCHEMA_VERSION,
            "status": "V16_DEVELOPMENT_DECISION_FROZEN_NO_HOLDOUT_ACCESS",
            "protocol_sha256": protocol["_protocol_sha256"],
            "decision": decision,
            "validation_semantic_access_allowed": False,
            "sealed_semantic_access_allowed": False,
            "reserve_semantic_access_allowed": False,
        },
    )
    manifest = {
        "schema_version": SCHEMA_VERSION,
        "status": decision["status"],
        "ok": decision["finalist_id"] is not None,
        "protocol": {
            "protocol_id": protocol["protocol_id"],
            "path": _portable(Path(str(protocol["_protocol_path"]))),
            "sha256": protocol["_protocol_sha256"],
        },
        "scope": {
            "development_trials": list(ALLOWED_TRIALS),
            "validation_opened": False,
            "sealed_opened": False,
            "reserve_opened": False,
            "trial01_opened": False,
        },
        "parent": parent,
        "geometry": geometry,
        "replay": {
            "full_production_fsm": True,
            "signal_only_router_used": False,
            "continuous_trial_no_plateau_reset": True,
            "candidate_count": len(candidates),
            "unit_row_count": len(rows),
        },
        "baseline_anchor_parity": parity,
        "decision": decision,
        "artifacts": {
            "development_rows": _artifact(rows_path),
            "development_details": _artifact(details_path),
            "development_sampling_access": _artifact(access_path),
            "development_decision_lock": _artifact(decision_path),
        },
        "non_actions": {
            "runtime_or_fsm_modified": False,
            "profile_or_registry_modified": False,
            "arms_combined": False,
            "training_run": False,
            "holdout_or_reserve_opened": False,
            "v15_rerun": False,
        },
        "interpretation_limits": protocol["interpretation_limits"],
    }
    manifest_path = _write_json_exclusive(output_dir / "manifest.json", manifest)
    return {**manifest, "manifest_sha256": _sha256(manifest_path)}


def _write_failure(output_dir: Path, exc: Exception) -> None:
    if not output_dir.is_dir() or (output_dir / "failure.json").exists():
        return
    _write_json_exclusive(
        output_dir / "failure.json",
        {
            "schema_version": SCHEMA_VERSION,
            "status": "ERROR_AFTER_V16_DESTINATION_CONSUMED",
            "error": f"{type(exc).__name__}: {exc}",
            "traceback": traceback.format_exc(),
            "rerun_allowed": False,
            "validation_opened": False,
            "sealed_opened": False,
            "reserve_opened": False,
        },
    )


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--protocol", default=str(DEFAULT_PROTOCOL))
    parser.add_argument("--output-dir", default=str(DEFAULT_OUTPUT_DIR))
    parser.add_argument("--check", action="store_true")
    parser.add_argument("--execute", action="store_true")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_arg_parser().parse_args(argv)
    if args.check and args.execute:
        raise SystemExit("--check and --execute are mutually exclusive")
    protocol = load_and_validate_protocol(args.protocol)
    preflight = _preflight_inputs(protocol)
    if not args.execute:
        print(
            json.dumps(
                {
                    "status": "PASS_V16_READ_ONLY_PREFLIGHT",
                    "protocol_id": protocol["protocol_id"],
                    "protocol_sha256": protocol["_protocol_sha256"],
                    "live_input_preflight": preflight,
                    "canonical_run_not_started": True,
                },
                indent=2,
                sort_keys=True,
                allow_nan=False,
            )
        )
        return 0
    output_dir = _resolve(args.output_dir)
    _preflight_no_clobber(output_dir)
    _start_receipt(output_dir, protocol)
    try:
        result = run_sweep(protocol, output_dir)
    except Exception as exc:
        _write_failure(output_dir, exc)
        print(f"V16 sweep failed: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 1
    print(json.dumps(v1._json_safe(result), indent=2, sort_keys=True, allow_nan=False))
    return 0 if result["status"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
