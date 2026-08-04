"""V12 isolated extension of the V11 toe-center compensation arm.

V11 established that heel local-x +3.5 mm satisfies the frozen HS timing and
count contract, while toe-center down 0.4--0.5 mm leaves one 1 ms transfer gap.
V12 keeps that heel, both radii, thresholds, dwell, FSM, evidence routing, and
all non-toe geometry fixed.  It evaluates only toe local-y down values beyond
the frozen V11 boundary, at both 10 ms and 1 ms, on the already-open interval.

The V11 0.5 mm boundary is replayed as a non-selectable comparator and must be
bit-identical in events, transfer diagnostics, and strict gates to the frozen
V11 manifest.  No profile can be created or promoted and no sample at or after
100 s is permitted.
"""

from __future__ import annotations

import argparse
import json
import sys
import traceback
from pathlib import Path
from typing import Any, Mapping, Sequence

REPO_ROOT = Path(__file__).resolve().parents[1]
VALIDATION_ROOT = REPO_ROOT / "validation"
for path in (REPO_ROOT, VALIDATION_ROOT, REPO_ROOT / "Trajectory Generator"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

import sweep_two_sensor_heel_x_prescribed_v11 as v11  # noqa: E402


v1 = v11.v1
v10 = v11.v10
dual = v11.dual
DEFAULT_PROTOCOL = VALIDATION_ROOT / "two_sensor_toe_compensation_sweep_protocol_v12.json"
DEFAULT_OUTPUT_DIR = (
    VALIDATION_ROOT
    / "two_sensor_toe_compensation_sweep_runs/"
    "2026-07-22_ab06_open_13p946_100_v12"
)
DEFAULT_PLOT_DIR = REPO_ROOT / "plot/07_22_2026_two_sensor_toe_compensation_v12"
V11_MANIFEST_PATH = (
    "validation/two_sensor_heel_x_sweep_runs/"
    "2026-07-22_ab06_open_13p946_100_v11/manifest.json"
)
V11_STAGE_B_RUNTIME_CSV = (
    "validation/two_sensor_heel_x_sweep_runs/"
    "2026-07-22_ab06_open_13p946_100_v11/"
    "heel_x_v11_stage_b_runtime_10ms_metrics.csv"
)
V11_STAGE_B_FINE_CSV = (
    "validation/two_sensor_heel_x_sweep_runs/"
    "2026-07-22_ab06_open_13p946_100_v11/"
    "heel_x_v11_stage_b_fine_1ms_metrics.csv"
)

PROTOCOL_ID = "AB06_TWO_SENSOR_TOE_COMPENSATION_DEVELOPMENT_2026-07-22_V12"
SCHEMA_VERSION = 12
HEEL_ANCHOR_ID = "heel_x_p3p5mm"
V11_BOUNDARY_ID = "toe_down_p0p5mm__heel_x_p3p5mm"
COMPARATOR_ID = "v11_boundary__toe_down_p0p5mm__heel_x_p3p5mm"
HEEL_X_SHIFT_MM = 3.5
COMPARATOR_TOE_DOWN_MM = 0.5
TOE_DOWN_GRID_MM = (0.55, 0.60, 0.65, 0.70, 0.75, 0.80, 0.90, 1.00)
SELECTABLE_COUNT = len(TOE_DOWN_GRID_MM)
PAIR_COUNT = SELECTABLE_COUNT + 1
DETECTOR_STATIONS = PAIR_COUNT + 1
TOTAL_STATIONS = DETECTOR_STATIONS + v10.EXPECTED_PRIMARY_SPHERES
OPEN_START_S = v11.OPEN_START_S
OPEN_END_S = v11.OPEN_END_S
PRIMARY_DT_S = v11.PRIMARY_DT_S
FINE_DT_S = v11.FINE_DT_S
NUMERIC_TOLERANCE = v11.NUMERIC_TOLERANCE
if not (
    SELECTABLE_COUNT == v11.STAGE_B_SELECTABLE_COUNT
    and PAIR_COUNT == v11.STAGE_B_PAIR_COUNT
):
    raise RuntimeError("V12 inherited selector cardinality contract drifted")

OBJECTIVE = (
    "Extend only the V11 toe-center local-y plantar compensation beyond its "
    "0.5 mm boundary while fixing heel local-x at +3.5 mm, identify the "
    "strict multiresolution candidate selected by the frozen structural, "
    "timing, and minimum-displacement ranking, remove the residual 1 ms "
    "heel-to-toe gap, and keep the sealed block closed."
)
INTERPRETATION_LIMITS = [
    "V12 is an adaptive development iteration motivated by the V11 boundary result and is not sealed holdout validation.",
    "The V11 heel local-x +3.5 mm anchor, both sphere radii, heel local-y/local-z, and toe local-x/local-z remain fixed.",
    "Each selectable candidate changes only the V9 toe local-y center in the plantar direction, with 0.5 < toe-down <= 1.0 mm.",
    "The V11 0.5 mm boundary comparator is non-selectable and must replay identically before any V12 result is accepted.",
    "Thresholds, dwell, FSM, load evidence, event routing, policy, reward, and training remain unchanged.",
    "Timing acceptance uses confirmed_time_s; event_time_s remains diagnostic only.",
    "Selection prioritizes structural integrity and timing/FSM quality; toe-center displacement is the final tie-break, not the primary objective.",
    "Heel-off shift remains diagnostic; strict acceptance uses the frozen event, FSM, causal, and transfer gates.",
    "No profile can be created or promoted, and the 100-155.045 s sealed block remains closed.",
]
REQUIRED_SOURCE_PATHS = {
    "v12_validator": "validation/sweep_two_sensor_toe_compensation_prescribed_v12.py",
    "v12_tests": "validation/test_two_sensor_toe_compensation_sweep_v12.py",
    "v11_protocol": "validation/two_sensor_heel_x_sweep_protocol_v11.json",
    "v11_manifest": V11_MANIFEST_PATH,
    "v11_stage_b_runtime_csv": V11_STAGE_B_RUNTIME_CSV,
    "v11_stage_b_fine_csv": V11_STAGE_B_FINE_CSV,
    **v11.REQUIRED_SOURCE_PATHS,
}


ProtocolError = v11.ProtocolError
NoClobberError = v11.NoClobberError


def _expected_access() -> dict[str, Any]:
    return {
        "already_open_interval_s": [OPEN_START_S, OPEN_END_S],
        "upper_bound_is_exclusive": True,
        "allow_samples_at_or_after_100_s": False,
        "sealed_block_s": [100.0, 155.045],
        "sealed_block_status": "CLOSED_UNEVALUATED",
        "expected_reference_hs": v11.EXPECTED_REFERENCE_HS,
        "expected_reference_to": v11.EXPECTED_REFERENCE_TO,
        "expected_complete_cycles": v11.EXPECTED_CYCLES,
        "excluded_closing_hs_s": [99.96878691565038],
    }


def load_and_validate_protocol(
    path: str | Path = DEFAULT_PROTOCOL,
) -> dict[str, Any]:
    protocol_path = v1.resolve_repo_path(path).resolve()
    try:
        raw = json.loads(protocol_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ProtocolError(f"cannot load V12 protocol: {protocol_path}") from exc
    if not isinstance(raw, dict):
        raise ProtocolError("V12 protocol root must be an object")

    expected_top = {
        "schema_version": SCHEMA_VERSION,
        "protocol_id": PROTOCOL_ID,
        "frozen_before_execution": True,
        "stage": "development_v11_toe_down_boundary_extension",
        "objective": OBJECTIVE,
        "interpretation_limits": INTERPRETATION_LIMITS,
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
        "v11_manifest": V11_MANIFEST_PATH,
    }
    for key, expected in expected_top.items():
        if raw.get(key) != expected:
            raise ProtocolError(f"V12 frozen field drifted: {key}")
    if raw.get("data_access") != _expected_access():
        raise ProtocolError("V12 data-access contract drifted")
    if raw.get("replay") != v11._expected_replay():
        raise ProtocolError("V12 replay contract drifted")

    extension = raw.get("toe_center_extension", {})
    expected_extension = {
        "heel_anchor_id": HEEL_ANCHOR_ID,
        "heel_x_shift_from_v9_mm": HEEL_X_SHIFT_MM,
        "v11_boundary_candidate_id": V11_BOUNDARY_ID,
        "comparator_toe_center_local_y_down_from_v9_mm": COMPARATOR_TOE_DOWN_MM,
        "selectable_toe_center_local_y_down_from_v9_mm": list(TOE_DOWN_GRID_MM),
        "selectable_candidate_count": SELECTABLE_COUNT,
        "comparator_count": 1,
        "total_pair_count": PAIR_COUNT,
        "sensors_per_pair": 2,
        "only_toe_local_y_changes": True,
        "heel_and_all_radii_fixed": True,
        "selectable_domain_mm": [0.5, 1.0],
        "selectable_lower_bound_is_exclusive": True,
    }
    if extension != expected_extension:
        raise ProtocolError("V12 toe-center extension drifted")

    expected_sampling = {
        "runtime_10ms": {
            "method": "shared_station_sampling_direct_opensim",
            "expected_unique_detector_stations": DETECTOR_STATIONS,
            "expected_primary_load_spheres": v10.EXPECTED_PRIMARY_SPHERES,
            "expected_total_sampled_stations": TOTAL_STATIONS,
            "evaluated_pair_count": PAIR_COUNT,
        },
        "fine_1ms": {
            "method": "shared_station_sampling_direct_opensim",
            "expected_unique_detector_stations": DETECTOR_STATIONS,
            "expected_primary_load_spheres": v10.EXPECTED_PRIMARY_SPHERES,
            "expected_total_sampled_stations": TOTAL_STATIONS,
            "evaluated_pair_count": PAIR_COUNT,
        },
        "affine_reconstruction": False,
    }
    if raw.get("sampling") != expected_sampling:
        raise ProtocolError("V12 sampling contract drifted")
    for key in ("runtime_gate_10ms", "fine_gate_1ms"):
        if raw.get(key) != v10._gate_contract():
            raise ProtocolError(f"V12 {key} drifted")

    expected_selection = {
        "strict_winner_requires_both_resolutions": True,
        "comparator_is_never_selectable": True,
        "structural_failures_rank_before_timing_for_diagnostics": True,
        "selector_implementation": "v11_select_stage_b_with_equal_8_plus_1_cardinality",
        "ranking": v11.STAGE_B_RANKING,
        "minimum_displacement_is_final_tie_break": True,
        "diagnostic_candidate_is_never_promotable": True,
    }
    if raw.get("selection") != expected_selection:
        raise ProtocolError("V12 selection contract drifted")
    expected_decision = {
        "candidate_selection_allowed": True,
        "profile_creation_allowed": False,
        "profile_promotion_allowed": False,
        "sealed_validation_allowed": False,
        "training_allowed": False,
        "runtime_modification_allowed": False,
        "v9_v10_v11_files_must_remain_immutable": True,
    }
    if raw.get("decision_contract") != expected_decision:
        raise ProtocolError("V12 decision contract drifted")

    sources = raw.get("sources")
    if not isinstance(sources, dict) or set(sources) != set(REQUIRED_SOURCE_PATHS):
        raise ProtocolError("V12 hash-pinned source set drifted")
    for label, record in sources.items():
        if not isinstance(record, dict):
            raise ProtocolError(f"invalid V12 source record: {label}")
        if record.get("path") != REQUIRED_SOURCE_PATHS[label]:
            raise ProtocolError(f"V12 source path drift: {label}")
        source_path = v1.resolve_repo_path(str(record["path"])).resolve()
        if not source_path.is_file():
            raise ProtocolError(f"missing V12 source: {label}")
        if v1._sha256(source_path) != record.get("sha256"):
            raise ProtocolError(f"V12 source hash drift: {label}")

    raw["replay"]["sample_dt_s"] = PRIMARY_DT_S
    raw["development_gate"] = raw["runtime_gate_10ms"]
    raw["profile_paths"] = {
        v1.CURRENT_PROFILE_ID: str(raw["geometry_reference_profile"])
    }
    raw["_protocol_path"] = protocol_path.as_posix()
    raw["_protocol_sha256"] = v1._sha256(protocol_path)
    raw["_primary_event_time_field"] = "confirmed_time_s"
    raw["_diagnostic_event_time_field"] = "event_time_s"
    raw["_phase_reference_mode"] = "validated_event_intervals"
    return raw


def _token(value: float) -> str:
    magnitude = f"{float(value):.3f}".rstrip("0").rstrip(".")
    return magnitude.replace(".", "p")


def build_candidates(
    protocol: Mapping[str, Any],
) -> tuple[
    v1.OnlineGRFProfile,
    list[v1.PlacementCandidate],
    v1.PlacementCandidate,
    dict[str, Any],
]:
    base, stage_a, _ = v11.build_stage_a_candidates(protocol)
    anchor = next(item for item in stage_a if item.candidate_id == HEEL_ANCHOR_ID)
    _, _, toe, triangles, mesh_path, profile_path = v11._base_geometry(protocol)
    toe_base = tuple(float(value) for value in toe.location)
    heel_location = tuple(float(value) for value in anchor.heel_location)
    heel_radius = float(anchor.geometry["heel_radius_m"])
    toe_radius = float(anchor.geometry["toe_radius_m"])

    def make(candidate_id: str, down_mm: float, *, selectable: bool) -> v1.PlacementCandidate:
        toe_location = (
            toe_base[0],
            toe_base[1] - down_mm / 1000.0,
            toe_base[2],
        )
        geometry = v11._candidate_geometry(
            arm="heel_x_anchor_toe_down",
            heel_location=heel_location,
            toe_location=toe_location,
            heel_radius=heel_radius,
            toe_radius=toe_radius,
            heel_x_shift_mm=HEEL_X_SHIFT_MM,
            toe_center_down_mm=down_mm,
            triangles=triangles,
            selectable=True,
        )
        geometry.update(
            {
                "source": "v11_boundary_extension",
                "stage_a_anchor_id": HEEL_ANCHOR_ID,
                "v11_boundary_candidate_id": V11_BOUNDARY_ID,
                "v12_comparator": not selectable,
            }
        )
        return v1.PlacementCandidate(
            candidate_id=candidate_id,
            heel_location=heel_location,
            forefoot_location=toe_location,
            heel_offset_below_current_mm=None,
            forefoot_fraction_mesh_x=None,
            forefoot_protrusion_mm=None,
            selectable=selectable,
            role=(
                "v11_boundary_comparator"
                if not selectable
                else "v12_toe_center_extension"
            ),
            geometry=geometry,
        )

    candidates = [
        make(COMPARATOR_ID, COMPARATOR_TOE_DOWN_MM, selectable=False),
        *(
            make(
                f"toe_down_p{_token(down)}mm__{HEEL_ANCHOR_ID}",
                down,
                selectable=True,
            )
            for down in TOE_DOWN_GRID_MM
        ),
    ]
    if len(candidates) != PAIR_COUNT:
        raise ProtocolError("V12 candidate count drifted")
    if len({item.candidate_id for item in candidates}) != PAIR_COUNT:
        raise ProtocolError("V12 candidate IDs are not unique")
    for item in candidates:
        down = float(item.geometry["toe_center_down_mm"])
        if not (
            bool(item.geometry["pre_gate_ok"])
            and tuple(item.heel_location) == heel_location
            and item.forefoot_location[0] == toe_base[0]
            and item.forefoot_location[2] == toe_base[2]
            and item.forefoot_location[1] == toe_base[1] - down / 1000.0
            and float(item.geometry["heel_radius_m"]) == heel_radius
            and float(item.geometry["toe_radius_m"]) == toe_radius
            and (
                (not item.selectable and down == COMPARATOR_TOE_DOWN_MM)
                or (item.selectable and 0.5 < down <= 1.0)
            )
        ):
            raise ProtocolError("V12 changed an undeclared geometry axis")

    _, v11_boundary_candidates, _ = v11.build_stage_b_candidates(protocol, anchor)
    v11_boundary = next(
        item for item in v11_boundary_candidates
        if item.candidate_id == V11_BOUNDARY_ID
    )
    geometry = {
        "mesh": v1._source_record(mesh_path),
        "v9_profile": v1._source_record(profile_path),
        "heel_anchor_id": HEEL_ANCHOR_ID,
        "heel_location_m": list(heel_location),
        "heel_x_shift_from_v9_mm": HEEL_X_SHIFT_MM,
        "v9_toe_location_m": list(toe_base),
        "heel_radius_m": heel_radius,
        "toe_radius_m": toe_radius,
        "comparator_toe_down_mm": COMPARATOR_TOE_DOWN_MM,
        "selectable_toe_down_mm": list(TOE_DOWN_GRID_MM),
        "only_toe_local_y_changes": True,
        "all_radii_fixed": True,
        "selectable_candidate_count": SELECTABLE_COUNT,
        "total_pair_count": PAIR_COUNT,
    }
    offsets_mm = [
        1000.0 * abs(float(item.geometry["heel_to_toe_bottom_offset_m"]))
        for item in candidates
    ]
    geometry["maximum_abs_heel_to_toe_bottom_offset_mm"] = max(offsets_mm)
    geometry["minimum_bottom_offset_margin_to_20mm"] = 20.0 - max(offsets_mm)
    return base, candidates, v11_boundary, geometry


def _load_v11_reference(protocol: Mapping[str, Any]) -> dict[str, Any]:
    manifest_path = v1.resolve_repo_path(str(protocol["v11_manifest"])).resolve()
    try:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ProtocolError("cannot load frozen V11 manifest") from exc
    runtime_ids = {
        str(row.get("candidate_id"))
        for row in manifest.get("stage_b", {}).get("runtime_10ms", {}).get(
            "rows", []
        )
    }
    fine_ids = {
        str(row.get("candidate_id"))
        for row in manifest.get("stage_b", {}).get("fine_1ms", {}).get(
            "rows", []
        )
    }
    if not (
        manifest.get("schema_version") == 11
        and manifest.get("status") == "FAIL"
        and manifest.get("stage_b", {}).get("executed") is True
        and manifest.get("stage_b", {}).get("selection", {}).get(
            "diagnostic_best_id"
        )
        == "toe_down_p0p4mm__heel_x_p3p5mm"
        and manifest.get("data_access", {}).get("sealed_block_opened") is False
        and V11_BOUNDARY_ID in runtime_ids
        and V11_BOUNDARY_ID in fine_ids
    ):
        raise ProtocolError("V11 reference manifest contract drifted")
    return manifest


def _v11_manifest_geometry_identity(
    reference: Mapping[str, Any],
    comparator: v1.PlacementCandidate,
) -> dict[str, Any]:
    try:
        frozen = next(
            item for item in reference["stage_b"]["candidates"]
            if item["candidate_id"] == V11_BOUNDARY_ID
        )
    except (KeyError, StopIteration, TypeError) as exc:
        raise ProtocolError("missing frozen V11 boundary geometry record") from exc
    current = v11._candidate_record(comparator)
    comparisons = {
        "heel_location_m": frozen.get("heel_location_m")
        == current.get("heel_location_m"),
        "forefoot_location_m": frozen.get("forefoot_location_m")
        == current.get("forefoot_location_m"),
        "heel_radius_m": frozen.get("heel_radius_m")
        == current.get("heel_radius_m"),
        "forefoot_radius_m": frozen.get("forefoot_radius_m")
        == current.get("forefoot_radius_m"),
        "heel_x_shift_mm": frozen.get("heel_x_shift_mm")
        == current.get("heel_x_shift_mm"),
        "toe_center_down_mm": frozen.get("toe_center_down_mm")
        == current.get("toe_center_down_mm"),
        "sensor_count": frozen.get("sensor_count") == current.get("sensor_count"),
        "heel_to_toe_bottom_offset_m": frozen.get("geometry", {}).get(
            "heel_to_toe_bottom_offset_m"
        )
        == current.get("geometry", {}).get("heel_to_toe_bottom_offset_m"),
        "frozen_pre_gate_ok": frozen.get("geometry", {}).get("pre_gate_ok")
        is True,
        "current_pre_gate_ok": current.get("geometry", {}).get("pre_gate_ok")
        is True,
    }
    return {
        "v11_candidate_id": V11_BOUNDARY_ID,
        "v12_comparator_id": comparator.candidate_id,
        "comparisons": comparisons,
        "ok": bool(all(comparisons.values())),
    }


def run_sweep(
    protocol: Mapping[str, Any], output_dir: Path, plot_dir: Path
) -> dict[str, Any]:
    v11._preflight_no_clobber(output_dir, plot_dir)
    base, candidates, v11_boundary, geometry = build_candidates(protocol)
    rows, details, access = v11._run_stage(
        protocol,
        base,
        candidates,
        stage_label="v12_toe_center_extension",
        expected_detector_stations=DETECTOR_STATIONS,
        expected_total_stations=TOTAL_STATIONS,
    )
    for label in ("runtime_10ms", "fine_1ms"):
        for row in rows[label]:
            row["v12_stage"] = "v12_toe_center_extension"
            row["inherited_v11_evaluator_stage_label"] = row["v11_stage"]
        for detail in details[label].values():
            detail["v12_geometry"] = dict(detail["v11_geometry"])
            detail["v12_geometry"]["provenance"] = "v12_boundary_extension"
    diagnostics = v11._augment_stage_diagnostics(
        rows, details, comparator_id=COMPARATOR_ID
    )
    reference = _load_v11_reference(protocol)
    manifest_geometry_identity = _v11_manifest_geometry_identity(
        reference, candidates[0]
    )
    identity = v11._stage_b_anchor_identity_audit(
        v11_boundary,
        candidates[0],
        protocol,
        {
            "runtime_10ms": reference["stage_b"]["runtime_10ms"]["rows"],
            "fine_1ms": reference["stage_b"]["fine_1ms"]["rows"],
        },
        {
            "runtime_10ms": reference["stage_b"]["runtime_10ms"]["details"],
            "fine_1ms": reference["stage_b"]["fine_1ms"]["details"],
        },
        rows,
        details,
    )
    identity["frozen_manifest_geometry"] = manifest_geometry_identity
    identity["ok"] = bool(identity["ok"] and manifest_geometry_identity["ok"])
    if not identity["ok"]:
        raise ProtocolError("V12 comparator does not replay identically to V11")

    selection = v11.select_stage_b(
        rows["runtime_10ms"], rows["fine_1ms"], protocol
    )
    selection["inherited_selector_status"] = selection["status"]
    selection["status"] = (
        "STRICT_V12_WINNER"
        if selection["strict_winner_id"] is not None
        else "NO_STRICT_V12_WINNER"
    )
    selection["selector_implementation"] = protocol["selection"][
        "selector_implementation"
    ]
    selection["diagnostics"] = diagnostics
    winner_id = selection["strict_winner_id"]
    ok = winner_id is not None
    reported_id = str(winner_id or selection["diagnostic_best_id"])

    all_access = list(access.values())
    access_ok = bool(
        all_access
        and all(
            float(record["last_sample_s"]) < OPEN_END_S
            and int(record["samples_at_or_after_100_s"]) == 0
            and not bool(record["sealed_block_opened"])
            for record in all_access
        )
    )
    if not access_ok:
        raise ProtocolError("V12 aggregate data-access audit failed")

    output_dir.mkdir(parents=True, exist_ok=False)
    plot_dir.mkdir(parents=True, exist_ok=False)
    csv_10 = output_dir / "toe_compensation_v12_runtime_10ms_metrics.csv"
    csv_1 = output_dir / "toe_compensation_v12_fine_1ms_metrics.csv"
    plot_path = plot_dir / "01_toe_compensation_v12_multiresolution.png"
    v11._write_csv(csv_10, rows["runtime_10ms"])
    v11._write_csv(csv_1, rows["fine_1ms"])
    v11._plot_stage(
        plot_path,
        rows["runtime_10ms"],
        rows["fine_1ms"],
        x_key="toe_center_down_mm",
        x_label="toe center local-y down from V9 [mm]",
        title="V12 — isolated toe-center boundary extension (confirmed time)",
        highlighted_ids=(reported_id,),
    )

    manifest = {
        "schema_version": SCHEMA_VERSION,
        "status": "PASS" if ok else "FAIL",
        "ok": ok,
        "stage": protocol["stage"],
        "objective": protocol["objective"],
        "protocol": {
            "path": v1._portable_path(Path(str(protocol["_protocol_path"]))),
            "sha256": protocol["_protocol_sha256"],
            "protocol_id": protocol["protocol_id"],
            "frozen_before_execution": True,
        },
        "v11_boundary_identity_audit": identity,
        "data_access": {
            "opened_interval_s": [OPEN_START_S, OPEN_END_S],
            "upper_bound_is_exclusive": True,
            "maximum_sample_time_is_below_100_s": access_ok,
            "sealed_block_s": [OPEN_END_S, 155.045],
            "sealed_block_opened": False,
            **access,
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
            "thresholds_dwell_fsm_and_all_radii_unchanged": True,
            "heel_off_advance_role": "diagnostic_only",
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
        "selection": selection,
        "reported_candidate": {
            "candidate_id": reported_id,
            "strict_multiresolution_pass": ok,
            "diagnostic_only": not ok,
            "profile_created": False,
            "promotable": False,
            "requires_future_frozen_validation": bool(ok),
        },
        "conclusion": (
            "STRICT_V12_DEVELOPMENT_WINNER"
            if ok
            else "NO_STRICT_V12_WINNER_DIAGNOSTIC_ONLY"
        ),
        "artifacts": {
            "runtime_metrics_csv": v1._source_record(csv_10),
            "fine_metrics_csv": v1._source_record(csv_1),
            "multiresolution_plot": v1._source_record(plot_path),
        },
        "non_actions": {
            "v9_modified": False,
            "v10_modified": False,
            "v11_modified": False,
            "threshold_or_fsm_sweep_run": False,
            "policy_or_training_run": False,
            "runtime_configuration_modified": False,
            "production_fsm_modified": False,
            "candidate_profile_created_or_promoted": False,
            "sealed_block_opened": False,
        },
        "interpretation_limits": protocol["interpretation_limits"],
    }
    safe = v1._json_safe(manifest)
    (output_dir / "manifest.json").write_text(
        json.dumps(safe, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    return safe


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run preregistered V12 toe-center boundary extension."
    )
    parser.add_argument("--protocol", default=str(DEFAULT_PROTOCOL))
    parser.add_argument("--output-dir", default=str(DEFAULT_OUTPUT_DIR))
    parser.add_argument("--plot-dir", default=str(DEFAULT_PLOT_DIR))
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_arg_parser().parse_args(argv)
    output_dir = v1.resolve_repo_path(args.output_dir).resolve()
    plot_dir = v1.resolve_repo_path(args.plot_dir).resolve()
    try:
        v11._preflight_no_clobber(output_dir, plot_dir)
        protocol = load_and_validate_protocol(args.protocol)
        manifest = run_sweep(protocol, output_dir, plot_dir)
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
    except Exception as exc:  # pragma: no cover
        print(
            json.dumps(
                {
                    "schema_version": SCHEMA_VERSION,
                    "status": "ERROR",
                    "ok": False,
                    "sealed_block_opened": False,
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
                "reported_candidate": manifest["reported_candidate"],
                "sealed_block_opened": False,
                "manifest": v1._portable_path(output_dir / "manifest.json"),
            },
            indent=2,
        )
    )
    return 0 if manifest["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
