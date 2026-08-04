"""V11 staged heel-local-x and conditional toe-compensation experiment.

V9 and V10 are immutable comparators.  Stage A changes only the local-x
coordinate of the V9 heel sphere.  If, and only if, no non-zero Stage-A
candidate passes strictly at both cadences but at least one candidate reaches
the frozen HS timing/count contract, Stage B keeps that preregistered anchor
and moves only the toe center in the local plantar direction to recover
heel-to-toe overlap.  Sphere radii, thresholds, dwell, FSM, evidence routing,
policy, reward, and training remain fixed.

All timing gates use ``confirmed_time_s``.  The complete already-open interval
13.946870983805102--<100 s is replayed directly at 10 ms and 1 ms.  No sample
at or beyond 100 s is permitted and this harness cannot create or promote a
runtime detector profile.
"""

from __future__ import annotations

import argparse
import csv
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

import sweep_two_sensor_center_radius_prescribed_v10 as v10  # noqa: E402


v1 = v10.v1
dual = v10.dual
DEFAULT_PROTOCOL = VALIDATION_ROOT / "two_sensor_heel_x_sweep_protocol_v11.json"
DEFAULT_OUTPUT_DIR = (
    VALIDATION_ROOT
    / "two_sensor_heel_x_sweep_runs/"
    "2026-07-22_ab06_open_13p946_100_v11"
)
DEFAULT_PLOT_DIR = REPO_ROOT / "plot/07_22_2026_two_sensor_heel_x_v11"

PROTOCOL_ID = "AB06_TWO_SENSOR_HEEL_X_DEVELOPMENT_2026-07-22_V11"
SCHEMA_VERSION = 11
BASELINE_ID = "v9_baseline"
STAGE_A_X_SHIFTS_MM = (
    -0.500,
    -0.250,
    -0.150,
    0.150,
    0.250,
    0.500,
    0.750,
    1.000,
    1.250,
    1.500,
    1.750,
    2.000,
    2.250,
    2.500,
    2.750,
    3.000,
    3.250,
    3.500,
    4.000,
)
STAGE_B_TOE_DOWN_MM = (0.050, 0.100, 0.150, 0.200, 0.250, 0.300, 0.400, 0.500)
STAGE_A_SELECTABLE_COUNT = len(STAGE_A_X_SHIFTS_MM)
STAGE_A_PAIR_COUNT = STAGE_A_SELECTABLE_COUNT + 1
STAGE_A_DETECTOR_STATIONS = STAGE_A_SELECTABLE_COUNT + 2
STAGE_A_TOTAL_STATIONS = STAGE_A_DETECTOR_STATIONS + v10.EXPECTED_PRIMARY_SPHERES
STAGE_B_SELECTABLE_COUNT = len(STAGE_B_TOE_DOWN_MM)
STAGE_B_PAIR_COUNT = STAGE_B_SELECTABLE_COUNT + 1
STAGE_B_DETECTOR_STATIONS = STAGE_B_PAIR_COUNT + 1
STAGE_B_TOTAL_STATIONS = STAGE_B_DETECTOR_STATIONS + v10.EXPECTED_PRIMARY_SPHERES
OPEN_START_S = v10.OPEN_START_S
OPEN_END_S = v10.OPEN_END_S
PRIMARY_DT_S = v10.PRIMARY_DT_S
FINE_DT_S = v10.FINE_DT_S
EXPECTED_REFERENCE_HS = v10.EXPECTED_REFERENCE_HS
EXPECTED_REFERENCE_TO = v10.EXPECTED_REFERENCE_TO
EXPECTED_CYCLES = v10.EXPECTED_CYCLES
NUMERIC_TOLERANCE = v10.NUMERIC_TOLERANCE
STAGE_B_TRIGGER = (
    "no_nonzero_stage_a_strict_winner_and_at_least_one_nonzero_stage_a_"
    "candidate_passes_hs_timing_and_exact_counts_at_both_resolutions_with_"
    "only_toe_compensable_structural_failures"
)
STAGE_A_RANKING = [
    "multiresolution_structural_gate_failure_count:min",
    "multiresolution_event_count_deficit:min",
    "multiresolution_valid_cycle_deficit:min",
    "multiresolution_invalid_timeout_unaccepted:min",
    "multiresolution_transfer_gap_samples:min",
    "multiresolution_incomplete_transfer:min",
    "multiresolution_early_to_candidates:min",
    "multiresolution_forbidden_phase_mismatch:min",
    "multiresolution_unknown_phase_samples:min",
    "multiresolution_causal_interval_deficit:min",
    "multiresolution_causal_clear_failure:min",
    "multiresolution_confirmation_latency_failure:min",
    "multiresolution_exact_order_failure:min",
    "multiresolution_mesh_pre_gate_failure:min",
    "multiresolution_two_window_max_abs_hs_error:min",
    "multiresolution_global_max_abs_hs_error:min",
    "multiresolution_global_mean_abs_hs_error:min",
    "multiresolution_global_max_abs_toe_off_error:min",
    "multiresolution_fsm_score_deficit:min",
    "absolute_heel_x_shift_mm:min",
    "candidate_id:lexicographic",
]
STAGE_B_RANKING = [
    *STAGE_A_RANKING[:-2],
    "toe_center_down_mm:min",
    "candidate_id:lexicographic",
]
STRUCTURAL_GATE_CHECKS = (
    "exact_reference_counts",
    "exact_detector_counts",
    "exact_valid_cycles",
    "precision",
    "recall",
    "zero_transfer_gap_samples",
    "zero_incomplete_transfers",
    "zero_early_to_candidates",
    "zero_invalid_timeout",
    "zero_unaccepted",
    "zero_forbidden_phase_mismatch",
    "zero_unknown_phase",
    "causal_clear",
    "exact_causal_intervals",
    "confirmation_latency",
    "exact_order_cycles",
    "mesh_pre_gate",
)
TOE_COMPENSABLE_ROOT_GATE_FAILURES = frozenset(
    {
        "zero_transfer_gap_samples",
        "zero_incomplete_transfers",
        "zero_invalid_timeout",
        "zero_unaccepted",
    }
)
TOE_COMPENSABLE_DERIVED_GATE_FAILURES = frozenset(
    {
        "precision",
        "recall",
        "fsm_f1",
        "fsm_iou",
        "zero_early_to_candidates",
        "zero_forbidden_phase_mismatch",
        "exact_order_cycles",
    }
)
TOE_COMPENSABLE_GATE_FAILURES = (
    TOE_COMPENSABLE_ROOT_GATE_FAILURES
    | TOE_COMPENSABLE_DERIVED_GATE_FAILURES
)
OBJECTIVE = (
    "Evaluate an isolated local-x sweep of the immutable V9 heel sphere on "
    "the complete already-open AB06 interval, then conditionally evaluate a "
    "preregistered toe-center plantar compensation arm only when a non-zero "
    "heel-x candidate satisfies the exact multiresolution HS timing/count "
    "contract, has only toe-compensable structural failures, and no non-zero "
    "Stage-A candidate passes every strict gate."
)
INTERPRETATION_LIMITS = [
    "V11 is a development experiment on data that were already open and is not sealed holdout validation.",
    "Stage A changes only the V9 heel local-x coordinate; every other detector parameter remains byte-equivalent in value to V9.",
    "Stage B is sampled only when its preregistered trigger is true and changes only the V9 toe local-y center relative to the selected Stage-A anchor.",
    "Thresholds, dwell, FSM, load evidence, event routing, policy, reward, training, and all sphere radii remain unchanged.",
    "Timing acceptance uses confirmed_time_s; event_time_s remains diagnostic only.",
    "Heel-off shift is recorded as a causal diagnostic; strict acceptance is governed by the frozen event, FSM, and transfer gates.",
    "No profile can be created or promoted, and the 100-155.045 s sealed block remains closed.",
]
REQUIRED_SOURCE_PATHS = {
    "v11_validator": "validation/sweep_two_sensor_heel_x_prescribed_v11.py",
    "v11_tests": "validation/test_two_sensor_heel_x_sweep_v11.py",
    "v10_validator": "validation/sweep_two_sensor_center_radius_prescribed_v10.py",
    "v10_protocol": "validation/two_sensor_center_radius_sweep_protocol_v10.json",
    "v10_manifest": (
        "validation/two_sensor_center_radius_sweep_runs/"
        "2026-07-22_ab06_open_13p946_100_v10/manifest.json"
    ),
    "v9_validator": "validation/sweep_two_sensor_timing_placements_prescribed_v9.py",
    "v9_profile": (
        "validation/experimental_detector_profiles/"
        "two_sensor_v9_H2p50_X3p25_F79p0_P35p00.json"
    ),
    "v9_protocol": "validation/two_sensor_timing_placement_sweep_protocol_v9.json",
    "v9_manifest": (
        "validation/two_sensor_timing_placement_sweep_runs/"
        "2026-07-22_ab06_50_100_heel_micro_v9/manifest.json"
    ),
    "v8_validator": "validation/sweep_two_sensor_timing_placements_prescribed_v8.py",
    "v7_validator": "validation/sweep_two_sensor_timing_placements_prescribed_v7.py",
    "v6_evaluator": "validation/sweep_two_sensor_timing_placements_prescribed_v6.py",
    "v4_evaluator": "validation/sweep_two_sensor_timing_placements_prescribed_v4.py",
    "v3_evaluator": "validation/sweep_two_sensor_timing_placements_prescribed_v3.py",
    "placement_evaluator": "validation/sweep_two_sensor_mesh_placements_prescribed.py",
    "profile_comparator": "validation/compare_two_sensor_mesh_profiles_prescribed.py",
    "dual_stream_router": "validation/diagnose_two_sensor_dual_stream_prescribed.py",
    "event_evaluator": "validation/sweep_two_sensor_prescribed_thresholds.py",
    "replay_validator": "validation/validate_two_sensor_prescribed_replay.py",
    "open_split_protocol": "validation/two_sensor_prescribed_threshold_sweep_protocol_v3.json",
    "production_fsm": "Trajectory Generator/prosthetic_phase_fsm.py",
    "online_grf": "online_grf.py",
    "training_fsm_config": "Trajectory Generator/baseline_MLP/training_exnovo_cfg.yaml",
    "setup": "models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi_setup.xml",
    "model": "models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi.osim",
    "ik": "models/AB06_SEASEA_Threadmill/data/IK_results_AB06_SEASEA.mot",
    "prescribed_grf": "models/AB06_SEASEA_Threadmill/data/AB06_SEASEA_GRF_FullSpan.mot",
    "external_forces": "models/AB06_SEASEA_Threadmill/data/ExternalForces.xml",
    "foot_mesh": "Geometry/AM_foot_l.STL",
    "primary_load_profile": "online_grf_profiles/AB06_SEASEA_stiff321_500_pi_grf_correct_magnitude.json",
    "geometry_reference_profile": "online_grf_profiles/AB06_SEASEA_stiff321_500_pi_grf_detector_HS-TO.json",
}


class ProtocolError(v10.ProtocolError):
    """Raised before sampling if the frozen V11 contract drifts."""


class NoClobberError(RuntimeError):
    """Raised when any V11 destination already exists, even if empty."""


def _preflight_no_clobber(output_dir: Path, plot_dir: Path) -> None:
    output_dir = output_dir.resolve()
    plot_dir = plot_dir.resolve()
    if (
        output_dir == plot_dir
        or output_dir in plot_dir.parents
        or plot_dir in output_dir.parents
    ):
        raise NoClobberError(
            "output and plot destinations must be distinct, non-nested paths"
        )
    occupied = [path for path in (output_dir, plot_dir) if path.exists()]
    if occupied:
        joined = ", ".join(v1._portable_path(path) for path in occupied)
        raise NoClobberError(f"refusing to modify existing path(s): {joined}")


def _numeric_list(value: Any, expected: Sequence[float], label: str) -> None:
    try:
        observed = [float(item) for item in value]
    except (TypeError, ValueError) as exc:
        raise ProtocolError(f"{label} must be a numeric list") from exc
    if observed != [float(item) for item in expected]:
        raise ProtocolError(f"{label} drifted: {observed}")


def _expected_replay() -> dict[str, Any]:
    return {
        "sea_plugin": "plugins/SEA_Plugin_BlackBox_mCMC_impedence_ff",
        "event_source": "two_sensor",
        "sensor_on_threshold_n": 0.5,
        "sensor_off_threshold_n": 0.25,
        "sensor_dwell_s": 0.03,
        "runtime_sample_dt_s": PRIMARY_DT_S,
        "fine_sample_dt_s": FINE_DT_S,
        "evaluate_all_candidates_at_both_resolutions": True,
        "primary_event_time_field": "confirmed_time_s",
        "diagnostic_event_time_field": "event_time_s",
        "phase_reference_mode": "validated_event_intervals",
        "prescribed_contact_threshold_n": 20.0,
        "reference_min_contact_duration_s": 0.05,
        "reference_min_cycle_duration_s": 0.3,
        "hs_tolerance_s": 0.05,
        "toe_off_tolerance_s": 0.08,
        "fsm_min_stance_duration_s": 0.3,
        "fsm_min_stance_contact_fraction": 0.2,
        "fsm_min_stance_load_bw_s": 0.04,
        "threshold_fsm_routing_or_radius_changed": False,
    }


def _expected_sampling(
    *, detector_stations: int, total_stations: int, pair_count: int
) -> dict[str, Any]:
    cadence = {
        "method": "shared_station_sampling_direct_opensim",
        "expected_unique_detector_stations": detector_stations,
        "expected_primary_load_spheres": v10.EXPECTED_PRIMARY_SPHERES,
        "expected_total_sampled_stations": total_stations,
        "evaluated_pair_count": pair_count,
    }
    return {
        "runtime_10ms": cadence,
        "fine_1ms": cadence,
        "affine_reconstruction": False,
    }


def load_and_validate_protocol(
    path: str | Path = DEFAULT_PROTOCOL,
) -> dict[str, Any]:
    protocol_path = v1.resolve_repo_path(path).resolve()
    try:
        raw = json.loads(protocol_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ProtocolError(f"cannot load V11 protocol: {protocol_path}") from exc
    if not isinstance(raw, dict):
        raise ProtocolError("V11 protocol root must be an object")

    expected_top = {
        "schema_version": SCHEMA_VERSION,
        "protocol_id": PROTOCOL_ID,
        "frozen_before_execution": True,
        "stage": "development_staged_heel_x_then_conditional_toe_compensation",
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
            raise ProtocolError(f"V11 frozen field drifted: {key}")
    if raw.get("objective") != OBJECTIVE:
        raise ProtocolError("V11 objective drifted")
    if raw.get("interpretation_limits") != INTERPRETATION_LIMITS:
        raise ProtocolError("V11 interpretation limits drifted")

    expected_access = {
        "already_open_interval_s": [OPEN_START_S, OPEN_END_S],
        "upper_bound_is_exclusive": True,
        "allow_samples_at_or_after_100_s": False,
        "sealed_block_s": [100.0, 155.045],
        "sealed_block_status": "CLOSED_UNEVALUATED",
        "expected_reference_hs": EXPECTED_REFERENCE_HS,
        "expected_reference_to": EXPECTED_REFERENCE_TO,
        "expected_complete_cycles": EXPECTED_CYCLES,
        "excluded_closing_hs_s": [99.96878691565038],
    }
    if raw.get("data_access") != expected_access:
        raise ProtocolError("V11 data-access contract drifted")
    if raw.get("replay") != _expected_replay():
        raise ProtocolError("V11 replay contract drifted")

    stage_a = raw.get("stage_a_heel_x", {})
    _numeric_list(
        stage_a.get("heel_local_x_shift_from_v9_mm"),
        STAGE_A_X_SHIFTS_MM,
        "Stage-A heel x grid",
    )
    for key, expected in {
        "selectable_candidate_count": STAGE_A_SELECTABLE_COUNT,
        "baseline_comparator_count": 1,
        "total_pair_count": STAGE_A_PAIR_COUNT,
        "sensors_per_pair": 2,
        "heel_y_radius_and_toe_geometry": "exact_v9_fixed",
        "only_heel_local_x_changes": True,
    }.items():
        if stage_a.get(key) != expected:
            raise ProtocolError(f"V11 Stage-A grid drifted: {key}")

    stage_b = raw.get("stage_b_conditional_toe_compensation", {})
    _numeric_list(
        stage_b.get("toe_center_local_y_down_from_v9_mm"),
        STAGE_B_TOE_DOWN_MM,
        "Stage-B toe compensation grid",
    )
    for key, expected in {
        "trigger": STAGE_B_TRIGGER,
        "anchor_rule": "best_preregistered_timing_qualified_stage_a_candidate",
        "selectable_candidate_count": STAGE_B_SELECTABLE_COUNT,
        "anchor_comparator_count": 1,
        "total_pair_count": STAGE_B_PAIR_COUNT,
        "sensors_per_pair": 2,
        "heel_anchor_and_all_radii": "fixed",
        "only_toe_local_y_changes": True,
    }.items():
        if stage_b.get(key) != expected:
            raise ProtocolError(f"V11 Stage-B contract drifted: {key}")

    sampling = raw.get("sampling", {})
    if sampling != {
        "stage_a": _expected_sampling(
            detector_stations=STAGE_A_DETECTOR_STATIONS,
            total_stations=STAGE_A_TOTAL_STATIONS,
            pair_count=STAGE_A_PAIR_COUNT,
        ),
        "stage_b_if_triggered": _expected_sampling(
            detector_stations=STAGE_B_DETECTOR_STATIONS,
            total_stations=STAGE_B_TOTAL_STATIONS,
            pair_count=STAGE_B_PAIR_COUNT,
        ),
        "stage_b_sampling_forbidden_when_trigger_false": True,
    }:
        raise ProtocolError("V11 sampling contract drifted")

    for key in ("runtime_gate_10ms", "fine_gate_1ms"):
        if raw.get(key) != v10._gate_contract():
            raise ProtocolError(f"V11 {key} drifted")

    expected_selection = {
        "strict_winner_requires_both_resolutions": True,
        "structural_failures_rank_before_timing_for_diagnostics": True,
        "stage_a_anchor_requires_exact_counts_and_hs_timing": True,
        "stage_a_anchor_allows_only_toe_compensable_structural_failures": True,
        "stage_a_anchor_required_root_gate_failures": sorted(
            TOE_COMPENSABLE_ROOT_GATE_FAILURES
        ),
        "stage_a_anchor_allowed_derived_gate_failures": sorted(
            TOE_COMPENSABLE_DERIVED_GATE_FAILURES
        ),
        "baseline_allowed_as_diagnostic_fallback": True,
        "diagnostic_candidate_is_never_promotable": True,
        "stage_a_ranking": STAGE_A_RANKING,
        "stage_b_ranking": STAGE_B_RANKING,
    }
    if raw.get("selection") != expected_selection:
        raise ProtocolError("V11 selection contract drifted")

    expected_decision = {
        "candidate_selection_allowed": True,
        "profile_creation_allowed": False,
        "profile_promotion_allowed": False,
        "sealed_validation_allowed": False,
        "training_allowed": False,
        "runtime_modification_allowed": False,
        "v9_and_v10_files_must_remain_immutable": True,
    }
    if raw.get("decision_contract") != expected_decision:
        raise ProtocolError("V11 decision contract drifted")

    sources = raw.get("sources")
    if not isinstance(sources, dict) or set(sources) != set(REQUIRED_SOURCE_PATHS):
        raise ProtocolError("V11 hash-pinned source set drifted")
    for label, record in sources.items():
        if not isinstance(record, dict):
            raise ProtocolError(f"invalid V11 source record: {label}")
        if record.get("path") != REQUIRED_SOURCE_PATHS[label]:
            raise ProtocolError(f"V11 source path drift: {label}")
        source_path = v1.resolve_repo_path(str(record.get("path", ""))).resolve()
        if not source_path.is_file():
            raise ProtocolError(f"missing V11 source: {label}")
        if v1._sha256(source_path) != record.get("sha256"):
            raise ProtocolError(f"V11 source hash drift: {label}")

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
    sign = "p" if value > 0.0 else "m"
    magnitude = f"{abs(float(value)):.3f}".rstrip("0").rstrip(".")
    return f"{sign}{magnitude.replace('.', 'p')}"


def _candidate_geometry(
    *,
    arm: str,
    heel_location: tuple[float, float, float],
    toe_location: tuple[float, float, float],
    heel_radius: float,
    toe_radius: float,
    heel_x_shift_mm: float,
    toe_center_down_mm: float,
    triangles: np.ndarray,
    selectable: bool,
) -> dict[str, Any]:
    heel_mesh = v10.v9.v8.v6.v3._mesh_geometry(
        heel_location, heel_radius, triangles
    )
    toe_mesh = v10.v9.v8.v6.v3._mesh_geometry(
        toe_location, toe_radius, triangles
    )
    bottom_offset_m = (
        toe_location[1] - toe_radius
    ) - (heel_location[1] - heel_radius)
    checks = {
        "exactly_two_spheres": True,
        "heel_within_5mm_of_mesh": bool(heel_mesh["within_5mm_of_mesh"]),
        "toe_within_5mm_of_mesh": bool(toe_mesh["within_5mm_of_mesh"]),
        "heel_to_toe_bottom_offset_within_20mm": abs(bottom_offset_m)
        <= 0.020 + NUMERIC_TOLERANCE,
        "positive_radii": heel_radius > 0.0 and toe_radius > 0.0,
        "one_declared_geometry_axis": bool(
            (
                arm == "v9_baseline_comparator"
                and not selectable
                and heel_x_shift_mm == 0.0
                and toe_center_down_mm == 0.0
            )
            or (
                arm == "heel_x_only"
                and selectable
                and heel_x_shift_mm != 0.0
                and toe_center_down_mm == 0.0
            )
            or (
                arm == "heel_x_anchor_toe_down"
                and selectable
                and heel_x_shift_mm != 0.0
                and toe_center_down_mm > 0.0
            )
            or (
                arm == "stage_b_anchor_comparator"
                and not selectable
                and heel_x_shift_mm != 0.0
                and toe_center_down_mm == 0.0
            )
        ),
    }
    return {
        "source": "v9_experimental_baseline",
        "parameter_arm": arm,
        "target_sensor": (
            "heel" if arm == "heel_x_only" else
            "toe_compensation" if arm == "heel_x_anchor_toe_down" else
            "stage_a_anchor" if arm == "stage_b_anchor_comparator" else
            "baseline"
        ),
        "heel_x_shift_mm": float(heel_x_shift_mm),
        "toe_center_down_mm": float(toe_center_down_mm),
        # Compatibility fields consumed by the frozen V10 evaluator wrapper.
        "center_raise_mm": float(-toe_center_down_mm),
        "radius_reduction_mm": 0.0,
        "heel_center_raise_mm": 0.0,
        "heel_radius_reduction_mm": 0.0,
        "toe_center_raise_mm": float(-toe_center_down_mm),
        "toe_radius_reduction_mm": 0.0,
        "heel_radius_m": float(heel_radius),
        "toe_radius_m": float(toe_radius),
        "v9_heel_radius_m": float(heel_radius),
        "v9_toe_radius_m": float(toe_radius),
        "effective_bottom_raise_mm": float(-toe_center_down_mm),
        "heel_to_toe_bottom_offset_m": float(bottom_offset_m),
        "heel": heel_mesh,
        "toe": toe_mesh,
        "pre_gate_checks": checks,
        "pre_gate_ok": bool(all(checks.values())),
        "detector_representation": "two_spheres_only",
    }


def _base_geometry(
    protocol: Mapping[str, Any],
) -> tuple[
    v1.OnlineGRFProfile,
    Any,
    Any,
    np.ndarray,
    Path,
    Path,
]:
    profile_path = v1.resolve_repo_path(
        str(protocol["detector_template_profile"])
    ).resolve()
    base = v1.load_online_grf_profile(profile_path, required_sides=("left",))
    sensors = v1._left_sensor_spheres(base)
    setup = v1.read_setup_xml(
        v1.resolve_repo_path(str(protocol["setup"])).resolve()
    )
    mesh_path = v1._resolve_left_foot_mesh(setup.model_file.resolve())
    triangles = v1._load_stl_triangles(mesh_path)
    return (
        base,
        sensors["left_heel"],
        sensors["left_toe"],
        triangles,
        mesh_path,
        profile_path,
    )


def build_stage_a_candidates(
    protocol: Mapping[str, Any],
) -> tuple[v1.OnlineGRFProfile, list[v1.PlacementCandidate], dict[str, Any]]:
    base, heel, toe, triangles, mesh_path, profile_path = _base_geometry(protocol)
    heel_base = tuple(float(value) for value in heel.location)
    toe_base = tuple(float(value) for value in toe.location)
    heel_radius = float(heel.radius)
    toe_radius = float(toe.radius)

    def make(
        candidate_id: str,
        shift_mm: float,
        *,
        selectable: bool,
        arm: str,
    ) -> v1.PlacementCandidate:
        heel_location = (
            heel_base[0] + shift_mm / 1000.0,
            heel_base[1],
            heel_base[2],
        )
        geometry = _candidate_geometry(
            arm=arm,
            heel_location=heel_location,
            toe_location=toe_base,
            heel_radius=heel_radius,
            toe_radius=toe_radius,
            heel_x_shift_mm=shift_mm,
            toe_center_down_mm=0.0,
            triangles=triangles,
            selectable=selectable,
        )
        return v1.PlacementCandidate(
            candidate_id=candidate_id,
            heel_location=heel_location,
            forefoot_location=toe_base,
            heel_offset_below_current_mm=None,
            forefoot_fraction_mesh_x=None,
            forefoot_protrusion_mm=None,
            selectable=selectable,
            role=arm,
            geometry=geometry,
        )

    candidates = [
        make(
            BASELINE_ID,
            0.0,
            selectable=False,
            arm="v9_baseline_comparator",
        )
    ]
    candidates.extend(
        make(
            f"heel_x_{_token(shift)}mm",
            shift,
            selectable=True,
            arm="heel_x_only",
        )
        for shift in STAGE_A_X_SHIFTS_MM
    )
    if len(candidates) != STAGE_A_PAIR_COUNT:
        raise ProtocolError("V11 Stage-A candidate count drifted")
    if len({item.candidate_id for item in candidates}) != len(candidates):
        raise ProtocolError("V11 Stage-A candidate IDs are not unique")
    if not all(bool(item.geometry["pre_gate_ok"]) for item in candidates):
        raise ProtocolError("a V11 Stage-A geometry failed its pre-gate")
    for item in candidates:
        expected_x = heel_base[0] + float(item.geometry["heel_x_shift_mm"]) / 1000.0
        if not (
            item.heel_location[0] == expected_x
            and item.heel_location[1:] == heel_base[1:]
            and tuple(item.forefoot_location) == toe_base
            and float(item.geometry["toe_center_down_mm"]) == 0.0
            and float(item.geometry["heel_radius_m"]) == heel_radius
            and float(item.geometry["toe_radius_m"]) == toe_radius
        ):
            raise ProtocolError("V11 Stage-A changed an undeclared geometry axis")
    return base, candidates, {
        "mesh": v1._source_record(mesh_path),
        "v9_profile": v1._source_record(profile_path),
        "v9_heel_location_m": list(heel_base),
        "v9_toe_location_m": list(toe_base),
        "v9_heel_radius_m": heel_radius,
        "v9_toe_radius_m": toe_radius,
        "selectable_candidate_count": STAGE_A_SELECTABLE_COUNT,
        "total_pair_count": STAGE_A_PAIR_COUNT,
        "only_heel_local_x_changes": True,
    }


def build_stage_b_candidates(
    protocol: Mapping[str, Any],
    anchor: v1.PlacementCandidate,
) -> tuple[v1.OnlineGRFProfile, list[v1.PlacementCandidate], dict[str, Any]]:
    base, heel, toe, triangles, mesh_path, profile_path = _base_geometry(protocol)
    if not (
        bool(anchor.selectable)
        and anchor.role == "heel_x_only"
        and anchor.geometry.get("parameter_arm") == "heel_x_only"
        and anchor.geometry.get("target_sensor") == "heel"
        and float(anchor.geometry.get("heel_x_shift_mm", 0.0)) != 0.0
        and float(anchor.geometry.get("toe_center_down_mm", 0.0)) == 0.0
    ):
        raise ProtocolError("V11 Stage-B anchor is not a non-zero Stage-A heel-x candidate")
    toe_base = tuple(float(value) for value in toe.location)
    heel_radius = float(heel.radius)
    toe_radius = float(toe.radius)
    heel_location = tuple(float(value) for value in anchor.heel_location)
    anchor_shift = float(anchor.geometry["heel_x_shift_mm"])
    expected_heel_location = (
        float(heel.location[0]) + anchor_shift / 1000.0,
        float(heel.location[1]),
        float(heel.location[2]),
    )
    if heel_location != expected_heel_location:
        raise ProtocolError("V11 Stage-B anchor heel geometry drifted")
    if tuple(float(value) for value in anchor.forefoot_location) != toe_base:
        raise ProtocolError("V11 Stage-B anchor toe geometry drifted")
    if not (
        float(anchor.geometry["heel_radius_m"]) == heel_radius
        and float(anchor.geometry["toe_radius_m"]) == toe_radius
    ):
        raise ProtocolError("V11 Stage-B anchor radius drifted")

    def make(
        candidate_id: str,
        toe_down_mm: float,
        *,
        selectable: bool,
        arm: str,
    ) -> v1.PlacementCandidate:
        toe_location = (
            toe_base[0],
            toe_base[1] - toe_down_mm / 1000.0,
            toe_base[2],
        )
        geometry = _candidate_geometry(
            arm=arm,
            heel_location=heel_location,
            toe_location=toe_location,
            heel_radius=heel_radius,
            toe_radius=toe_radius,
            heel_x_shift_mm=anchor_shift,
            toe_center_down_mm=toe_down_mm,
            triangles=triangles,
            selectable=selectable,
        )
        geometry["stage_a_anchor_id"] = anchor.candidate_id
        return v1.PlacementCandidate(
            candidate_id=candidate_id,
            heel_location=heel_location,
            forefoot_location=toe_location,
            heel_offset_below_current_mm=None,
            forefoot_fraction_mesh_x=None,
            forefoot_protrusion_mm=None,
            selectable=selectable,
            role=arm,
            geometry=geometry,
        )

    candidates = [
        make(
            f"stage_b_anchor__{anchor.candidate_id}",
            0.0,
            selectable=False,
            arm="stage_b_anchor_comparator",
        )
    ]
    candidates.extend(
        make(
            f"toe_down_{_token(delta)}mm__{anchor.candidate_id}",
            delta,
            selectable=True,
            arm="heel_x_anchor_toe_down",
        )
        for delta in STAGE_B_TOE_DOWN_MM
    )
    if len(candidates) != STAGE_B_PAIR_COUNT:
        raise ProtocolError("V11 Stage-B candidate count drifted")
    if len({item.candidate_id for item in candidates}) != len(candidates):
        raise ProtocolError("V11 Stage-B candidate IDs are not unique")
    if not all(bool(item.geometry["pre_gate_ok"]) for item in candidates):
        raise ProtocolError("a V11 Stage-B geometry failed its pre-gate")
    for item in candidates:
        if tuple(item.heel_location) != heel_location:
            raise ProtocolError("V11 Stage-B heel changed within toe arm")
        if not (
            item.forefoot_location[0] == toe_base[0]
            and item.forefoot_location[2] == toe_base[2]
            and item.forefoot_location[1]
            == toe_base[1] - float(item.geometry["toe_center_down_mm"]) / 1000.0
            and float(item.geometry["heel_radius_m"]) == heel_radius
            and float(item.geometry["toe_radius_m"]) == toe_radius
        ):
            raise ProtocolError("V11 Stage-B changed an undeclared geometry axis")
    return base, candidates, {
        "mesh": v1._source_record(mesh_path),
        "v9_profile": v1._source_record(profile_path),
        "stage_a_anchor_id": anchor.candidate_id,
        "stage_a_anchor_heel_location_m": list(heel_location),
        "stage_a_anchor_heel_x_shift_mm": anchor_shift,
        "selectable_candidate_count": STAGE_B_SELECTABLE_COUNT,
        "total_pair_count": STAGE_B_PAIR_COUNT,
        "only_toe_local_y_changes": True,
        "all_radii_fixed_to_v9": True,
    }


def _sampling_bundle(
    base: v1.OnlineGRFProfile,
    candidates: Sequence[v1.PlacementCandidate],
    *,
    stage_label: str,
    expected_detector_stations: int,
) -> tuple[
    v1.OnlineGRFProfile,
    dict[str, dict[str, Any]],
    dict[str, v1.OnlineGRFProfile],
]:
    sensors = v1._left_sensor_spheres(base)
    templates = {"heel": sensors["left_heel"], "toe": sensors["left_toe"]}
    cache: dict[tuple[str, tuple[float, float, float]], Any] = {}
    sampled_spheres: list[Any] = []

    def station(role: str, location: tuple[float, float, float]) -> Any:
        key = (role, tuple(float(value) for value in location))
        if key not in cache:
            sampled = replace(
                templates[role],
                name=f"v11_{stage_label}_{role}_{len(cache):02d}",
                location=key[1],
            )
            cache[key] = sampled
            sampled_spheres.append(sampled)
        return cache[key]

    pairs: dict[str, dict[str, Any]] = {}
    profiles: dict[str, v1.OnlineGRFProfile] = {}
    for candidate in candidates:
        heel_station = station("heel", tuple(candidate.heel_location))
        toe_station = station("toe", tuple(candidate.forefoot_location))
        heel_variant = replace(
            heel_station, radius=float(candidate.geometry["heel_radius_m"])
        )
        toe_variant = replace(
            toe_station, radius=float(candidate.geometry["toe_radius_m"])
        )
        pairs[candidate.candidate_id] = {
            "heel": heel_variant,
            "toe": toe_variant,
        }
        profiles[candidate.candidate_id] = replace(
            base,
            source=f"validation_v11_{stage_label}_{candidate.candidate_id}",
            spheres=(heel_variant, toe_variant),
        )
    sampler = replace(
        base,
        source=f"validation_v11_{stage_label}_station_sampler",
        spheres=tuple(sampled_spheres),
    )
    if len(sampler.spheres) != expected_detector_stations:
        raise ProtocolError(
            f"V11 {stage_label} detector-station count drifted: "
            f"{len(sampler.spheres)}"
        )
    if any(len(profile.spheres) != 2 for profile in profiles.values()):
        raise ProtocolError("a V11 profile is not exactly heel + toe")
    return sampler, pairs, profiles


def sample_streams_once(
    protocol: Mapping[str, Any],
    base: v1.OnlineGRFProfile,
    candidates: Sequence[v1.PlacementCandidate],
    *,
    sample_dt_s: float,
    stage_label: str,
    expected_detector_stations: int,
    expected_total_stations: int,
) -> tuple[dict[str, dict[str, Any]], dict[str, Any]]:
    setup, events, access, times = v10._reference_bundle(
        protocol, sample_dt_s=sample_dt_s
    )
    detector_sampler, sphere_pairs, detector_profiles = _sampling_bundle(
        base,
        candidates,
        stage_label=stage_label,
        expected_detector_stations=expected_detector_stations,
    )
    primary_path = v1.resolve_repo_path(
        str(protocol["load_evidence_profile"])
    ).resolve()
    primary_full = dual.load_online_grf_profile(primary_path)
    primary_spheres = tuple(
        sphere for sphere in primary_full.spheres if sphere.side == "left"
    )
    if len(primary_spheres) != v10.EXPECTED_PRIMARY_SPHERES:
        raise ProtocolError("V11 primary sphere count drifted")
    primary_left = replace(
        primary_full,
        source=f"validation_v11_{stage_label}_primary_left",
        spheres=primary_spheres,
    )
    combined = tuple(detector_sampler.spheres) + primary_spheres
    if len(combined) != expected_total_stations:
        raise ProtocolError(f"V11 {stage_label} total station count drifted")
    if len({sphere.name for sphere in combined}) != len(combined):
        raise ProtocolError("V11 sampled station names are not unique")
    sampler = replace(
        base,
        source=f"validation_v11_{stage_label}_detector_primary_sampler",
        spheres=combined,
    )
    samples = v1._sample_spheres(
        setup,
        sampler,
        times,
        str(protocol["replay"]["sea_plugin"]),
    )
    primary_aggregate = np.asarray(
        dual._calculate_wrench(primary_left, dict(samples))["left"][
            "normal_force"
        ],
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
            "sampled_unique_detector_stations": len(detector_sampler.spheres),
            "sampled_primary_load_spheres": len(primary_spheres),
            "sampled_total_stations": len(combined),
            "evaluated_pair_count": len(candidates),
            "load_evidence_profile": v1._source_record(primary_path),
        }
    )
    if not (
        float(access["first_sample_s"]) >= OPEN_START_S - NUMERIC_TOLERANCE
        and float(access["last_sample_s"]) < OPEN_END_S
        and int(access["samples_at_or_after_100_s"]) == 0
        and not bool(access["sealed_block_opened"])
        and math.isclose(
            float(access["sample_dt_s"]),
            float(sample_dt_s),
            rel_tol=0.0,
            abs_tol=NUMERIC_TOLERANCE,
        )
    ):
        raise ProtocolError(f"V11 {stage_label} data-access audit failed")
    return streams, access


def evaluate_candidate(
    protocol: Mapping[str, Any],
    candidate: v1.PlacementCandidate,
    common: Mapping[str, Any],
    *,
    sample_dt_s: float,
    stage_label: str,
) -> tuple[dict[str, Any], dict[str, Any]]:
    row, detail = v10.evaluate_candidate(
        protocol, candidate, common, sample_dt_s=sample_dt_s
    )
    heel_x_shift_mm = float(candidate.geometry["heel_x_shift_mm"])
    toe_down_mm = float(candidate.geometry["toe_center_down_mm"])
    row.update(
        {
            "v11_stage": stage_label,
            "parameter_arm": candidate.geometry["parameter_arm"],
            "heel_x_shift_mm": heel_x_shift_mm,
            "toe_center_down_mm": toe_down_mm,
            "geometry_displacement_from_v9_m": (
                abs(heel_x_shift_mm) + abs(toe_down_mm)
            )
            / 1000.0,
            "geometry_displacement_from_current_m": (
                abs(heel_x_shift_mm) + abs(toe_down_mm)
            )
            / 1000.0,
        }
    )
    detail["v11_geometry"] = {
        "stage": stage_label,
        "heel_x_shift_mm": heel_x_shift_mm,
        "toe_center_down_mm": toe_down_mm,
        "parameter_arm": candidate.geometry["parameter_arm"],
    }
    return row, detail


def _finite_metric(row: Mapping[str, Any], key: str) -> float:
    value = row.get(key)
    if value is None:
        return float("inf")
    try:
        number = float(value)
    except (TypeError, ValueError):
        return float("inf")
    return number if math.isfinite(number) else float("inf")


def _int_metric(row: Mapping[str, Any], key: str, default: int = 0) -> int:
    try:
        return int(row.get(key, default))
    except (TypeError, ValueError):
        return default


def _structural_key(
    row_10: Mapping[str, Any],
    row_1: Mapping[str, Any],
    protocol: Mapping[str, Any],
) -> tuple[int, ...]:
    gate_10 = v10.strict_gate(row_10, protocol["runtime_gate_10ms"])
    gate_1 = v10.strict_gate(row_1, protocol["fine_gate_1ms"])
    structural_failure_count = sum(
        int(
            not bool(gate_10["checks"][check])
            or not bool(gate_1["checks"][check])
        )
        for check in STRUCTURAL_GATE_CHECKS
    )

    def maximum(key: str) -> int:
        return max(_int_metric(row_10, key), _int_metric(row_1, key))

    valid_cycle_deficit = max(
        abs(EXPECTED_CYCLES - _int_metric(row_10, "observed_valid_cycle_count")),
        abs(EXPECTED_CYCLES - _int_metric(row_1, "observed_valid_cycle_count")),
    )
    causal_interval_deficit = max(
        abs(EXPECTED_CYCLES - _int_metric(row_10, "causal_swing_interval_count")),
        abs(EXPECTED_CYCLES - _int_metric(row_1, "causal_swing_interval_count")),
    )
    return (
        structural_failure_count,
        maximum("event_count_deficit"),
        valid_cycle_deficit,
        maximum("invalid_timeout_plus_unaccepted_count"),
        maximum("transfer_both_latches_off_sample_count"),
        maximum("incomplete_heel_to_forefoot_transfer_count"),
        maximum("to_candidates_before_min_stance_count"),
        maximum("forbidden_phase_mismatch_count"),
        maximum("unknown_fsm_phase_samples"),
        causal_interval_deficit,
        int(
            not bool(gate_10["checks"]["causal_clear"])
            or not bool(gate_1["checks"]["causal_clear"])
        ),
        int(
            not bool(gate_10["checks"]["confirmation_latency"])
            or not bool(gate_1["checks"]["confirmation_latency"])
        ),
        int(
            not bool(gate_10["checks"]["exact_order_cycles"])
            or not bool(gate_1["checks"]["exact_order_cycles"])
        ),
        int(
            not bool(gate_10["checks"]["mesh_pre_gate"])
            or not bool(gate_1["checks"]["mesh_pre_gate"])
        ),
    )


def _timing_key(
    row_10: Mapping[str, Any], row_1: Mapping[str, Any]
) -> tuple[float, ...]:
    def score_deficit(row: Mapping[str, Any], key: str) -> float:
        value = _finite_metric(row, key)
        return max(0.0, 1.0 - value) if math.isfinite(value) else float("inf")

    return (
        max(
            _finite_metric(row_10, "early_max_abs_hs_error_s"),
            _finite_metric(row_1, "early_max_abs_hs_error_s"),
            _finite_metric(row_10, "long_max_abs_hs_error_s"),
            _finite_metric(row_1, "long_max_abs_hs_error_s"),
        ),
        max(
            _finite_metric(row_10, "max_abs_hs_error_s"),
            _finite_metric(row_1, "max_abs_hs_error_s"),
        ),
        max(
            _finite_metric(row_10, "global_mean_abs_hs_error_s"),
            _finite_metric(row_1, "global_mean_abs_hs_error_s"),
        ),
        max(
            _finite_metric(row_10, "max_abs_toe_off_error_s"),
            _finite_metric(row_1, "max_abs_toe_off_error_s"),
        ),
        max(
            score_deficit(row_10, "confirmed_fsm_stance_f1"),
            score_deficit(row_1, "confirmed_fsm_stance_f1"),
            score_deficit(row_10, "confirmed_fsm_stance_iou"),
            score_deficit(row_1, "confirmed_fsm_stance_iou"),
        ),
    )


def _hs_timing_and_counts_qualified(
    row_10: Mapping[str, Any],
    row_1: Mapping[str, Any],
    protocol: Mapping[str, Any],
) -> bool:
    for row, gate_key in (
        (row_10, "runtime_gate_10ms"),
        (row_1, "fine_gate_1ms"),
    ):
        gate = protocol[gate_key]
        if not (
            int(row["reference_hs_count"]) == EXPECTED_REFERENCE_HS
            and int(row["reference_to_count"]) == EXPECTED_REFERENCE_TO
            and int(row["predicted_hs_count"]) == EXPECTED_REFERENCE_HS
            and int(row["predicted_to_count"]) == EXPECTED_REFERENCE_TO
            and int(row["observed_valid_cycle_count"]) == EXPECTED_CYCLES
            and _finite_metric(row, "max_abs_hs_error_s")
            <= float(gate["max_abs_hs_error_s"]) + NUMERIC_TOLERANCE
        ):
            return False
    return True


def _stage_b_anchor_qualified(
    row_10: Mapping[str, Any],
    row_1: Mapping[str, Any],
    protocol: Mapping[str, Any],
) -> tuple[bool, dict[str, Any]]:
    timing_counts_ok = _hs_timing_and_counts_qualified(row_10, row_1, protocol)
    gate_10 = v10.strict_gate(row_10, protocol["runtime_gate_10ms"])
    gate_1 = v10.strict_gate(row_1, protocol["fine_gate_1ms"])
    failures_by_resolution: dict[str, list[str]] = {}
    for label, gate in (("runtime_10ms", gate_10), ("fine_1ms", gate_1)):
        failures_by_resolution[label] = sorted(
            key for key, passed in gate["checks"].items() if not bool(passed)
        )
    all_failures = set(failures_by_resolution["runtime_10ms"]) | set(
        failures_by_resolution["fine_1ms"]
    )
    noncompensable = all_failures - TOE_COMPENSABLE_GATE_FAILURES
    has_compensable_root_failure = bool(
        all_failures & TOE_COMPENSABLE_ROOT_GATE_FAILURES
    )
    qualified = bool(
        timing_counts_ok
        and has_compensable_root_failure
        and not noncompensable
    )
    return qualified, {
        "hs_timing_and_exact_counts_at_both": timing_counts_ok,
        "failures_by_resolution": failures_by_resolution,
        "toe_compensable_failures": sorted(
            all_failures & TOE_COMPENSABLE_GATE_FAILURES
        ),
        "toe_compensable_root_failures": sorted(
            all_failures & TOE_COMPENSABLE_ROOT_GATE_FAILURES
        ),
        "toe_compensable_derived_failures": sorted(
            all_failures & TOE_COMPENSABLE_DERIVED_GATE_FAILURES
        ),
        "noncompensable_failures": sorted(noncompensable),
        "has_toe_compensable_root_failure": has_compensable_root_failure,
        "qualified_for_stage_b_anchor": qualified,
    }


def select_stage_a(
    rows_10: Sequence[Mapping[str, Any]],
    rows_1: Sequence[Mapping[str, Any]],
    protocol: Mapping[str, Any],
) -> dict[str, Any]:
    runtime = {str(row["candidate_id"]): row for row in rows_10}
    fine = {str(row["candidate_id"]): row for row in rows_1}
    if set(runtime) != set(fine) or len(runtime) != STAGE_A_PAIR_COUNT:
        raise ProtocolError("V11 Stage-A multiresolution sets differ")
    selectable_ids = sorted(
        candidate_id for candidate_id, row in runtime.items()
        if bool(row.get("selectable"))
    )
    if len(selectable_ids) != STAGE_A_SELECTABLE_COUNT:
        raise ProtocolError("V11 Stage-A selectable count drifted")
    gates: dict[str, Any] = {}
    eligible: list[str] = []
    timing_qualified: list[str] = []
    stage_b_qualified: list[str] = []
    for candidate_id in selectable_ids:
        gate_10 = v10.strict_gate(
            runtime[candidate_id], protocol["runtime_gate_10ms"]
        )
        gate_1 = v10.strict_gate(
            fine[candidate_id], protocol["fine_gate_1ms"]
        )
        passes_both = bool(gate_10["ok"] and gate_1["ok"])
        timing_ok = _hs_timing_and_counts_qualified(
            runtime[candidate_id], fine[candidate_id], protocol
        )
        stage_b_ok, stage_b_audit = _stage_b_anchor_qualified(
            runtime[candidate_id], fine[candidate_id], protocol
        )
        gates[candidate_id] = {
            "runtime_10ms": gate_10,
            "fine_1ms": gate_1,
            "passes_both": passes_both,
            "hs_timing_and_exact_counts_at_both": timing_ok,
            "stage_b_anchor_audit": stage_b_audit,
        }
        if passes_both:
            eligible.append(candidate_id)
        if timing_ok:
            timing_qualified.append(candidate_id)
        if stage_b_ok:
            stage_b_qualified.append(candidate_id)

    def rank(candidate_id: str) -> tuple[Any, ...]:
        row_10 = runtime[candidate_id]
        row_1 = fine[candidate_id]
        return (
            *_structural_key(row_10, row_1, protocol),
            *_timing_key(row_10, row_1),
            abs(_finite_metric(row_1, "heel_x_shift_mm")),
            candidate_id,
        )

    eligible.sort(key=rank)
    timing_qualified.sort(key=rank)
    stage_b_qualified.sort(key=rank)
    diagnostic_ranked = sorted([BASELINE_ID, *selectable_ids], key=rank)
    strict_winner = eligible[0] if eligible else None
    anchor_id = (
        stage_b_qualified[0]
        if strict_winner is None and stage_b_qualified
        else None
    )
    if strict_winner is not None:
        stage_b_trigger = "SKIPPED_STAGE_A_STRICT_WINNER"
    elif anchor_id is not None:
        stage_b_trigger = (
            "RUN_HS_QUALIFIED_X_WITH_TOE_COMPENSABLE_STRUCTURAL_FAILURES"
        )
    else:
        stage_b_trigger = "SKIPPED_NO_TOE_COMPENSABLE_STAGE_A_ANCHOR"
    return {
        "status": (
            "STRICT_STAGE_A_WINNER"
            if strict_winner is not None
            else "NO_STRICT_STAGE_A_WINNER"
        ),
        "strict_winner_id": strict_winner,
        "diagnostic_best_id": diagnostic_ranked[0],
        "eligible_ids_ranked": eligible,
        "timing_qualified_ids_ranked": timing_qualified,
        "stage_b_qualified_anchor_ids_ranked": stage_b_qualified,
        "stage_b_anchor_id": anchor_id,
        "stage_b_trigger_decision": stage_b_trigger,
        "all_candidates_diagnostic_ranked": diagnostic_ranked,
        "candidate_gates": gates,
        "automatic_promotion_allowed": False,
    }


def select_stage_b(
    rows_10: Sequence[Mapping[str, Any]],
    rows_1: Sequence[Mapping[str, Any]],
    protocol: Mapping[str, Any],
) -> dict[str, Any]:
    runtime = {str(row["candidate_id"]): row for row in rows_10}
    fine = {str(row["candidate_id"]): row for row in rows_1}
    if set(runtime) != set(fine) or len(runtime) != STAGE_B_PAIR_COUNT:
        raise ProtocolError("V11 Stage-B multiresolution sets differ")
    selectable_ids = sorted(
        candidate_id for candidate_id, row in runtime.items()
        if bool(row.get("selectable"))
    )
    comparator_ids = sorted(set(runtime) - set(selectable_ids))
    if len(selectable_ids) != STAGE_B_SELECTABLE_COUNT or len(comparator_ids) != 1:
        raise ProtocolError("V11 Stage-B candidate roles drifted")
    gates: dict[str, Any] = {}
    eligible: list[str] = []
    for candidate_id in selectable_ids:
        gate_10 = v10.strict_gate(
            runtime[candidate_id], protocol["runtime_gate_10ms"]
        )
        gate_1 = v10.strict_gate(
            fine[candidate_id], protocol["fine_gate_1ms"]
        )
        passes = bool(gate_10["ok"] and gate_1["ok"])
        gates[candidate_id] = {
            "runtime_10ms": gate_10,
            "fine_1ms": gate_1,
            "passes_both": passes,
        }
        if passes:
            eligible.append(candidate_id)

    def rank(candidate_id: str) -> tuple[Any, ...]:
        row_10 = runtime[candidate_id]
        row_1 = fine[candidate_id]
        return (
            *_structural_key(row_10, row_1, protocol),
            *_timing_key(row_10, row_1),
            _finite_metric(row_1, "toe_center_down_mm"),
            candidate_id,
        )

    eligible.sort(key=rank)
    diagnostic_ranked = sorted([*comparator_ids, *selectable_ids], key=rank)
    winner = eligible[0] if eligible else None
    return {
        "status": (
            "STRICT_STAGE_B_WINNER"
            if winner is not None
            else "NO_STRICT_STAGE_B_WINNER"
        ),
        "strict_winner_id": winner,
        "diagnostic_best_id": diagnostic_ranked[0],
        "eligible_ids_ranked": eligible,
        "all_candidates_diagnostic_ranked": diagnostic_ranked,
        "candidate_gates": gates,
        "automatic_promotion_allowed": False,
    }


def _critical_hs_diagnostics(detail: Mapping[str, Any]) -> dict[str, Any]:
    events = detail["events"]
    reference = np.asarray(events["reference"]["heel_strike"], dtype=float)
    confirmed = np.asarray(events["confirmed"]["heel_strike"], dtype=float)
    onset = np.asarray(events["onset"]["heel_strike"], dtype=float)
    if not (reference.size == confirmed.size == onset.size):
        return {
            "equal_counts": False,
            "reference_count": int(reference.size),
            "confirmed_count": int(confirmed.size),
            "onset_count": int(onset.size),
            "out_of_50ms_count": None,
            "out_of_50ms": [],
        }
    errors = confirmed - reference
    critical = np.flatnonzero(np.abs(errors) > 0.05 + NUMERIC_TOLERANCE)
    return {
        "equal_counts": True,
        "reference_count": int(reference.size),
        "confirmed_count": int(confirmed.size),
        "onset_count": int(onset.size),
        "out_of_50ms_count": int(critical.size),
        "out_of_50ms": [
            {
                "event_index": int(index),
                "reference_time_s": float(reference[index]),
                "onset_time_s": float(onset[index]),
                "confirmed_time_s": float(confirmed[index]),
                "signed_error_s": float(errors[index]),
                "absolute_error_s": float(abs(errors[index])),
                "window": (
                    "early_13p946_18p946"
                    if reference[index] <= 18.946870983805102
                    else "middle_18p946_50"
                    if reference[index] < 50.0
                    else "long_50_100"
                ),
            }
            for index in critical
        ],
    }


def _heel_off_shift_diagnostics(
    comparator_detail: Mapping[str, Any],
    candidate_detail: Mapping[str, Any],
) -> dict[str, Any]:
    comparator_cycles = comparator_detail["heel_to_forefoot_transfer"]["cycles"]
    candidate_cycles = candidate_detail["heel_to_forefoot_transfer"]["cycles"]
    comparator = {int(item["cycle_index"]): item for item in comparator_cycles}
    candidate = {int(item["cycle_index"]): item for item in candidate_cycles}
    shared = sorted(set(comparator) & set(candidate))
    shifts: list[float] = []
    missing = 0
    for index in shared:
        baseline_value = comparator[index].get("heel_off_s")
        candidate_value = candidate[index].get("heel_off_s")
        if baseline_value is None or candidate_value is None:
            missing += 1
            continue
        shifts.append(float(candidate_value) - float(baseline_value))
    tolerance = 1e-12
    earlier = [value for value in shifts if value < -tolerance]
    later = [value for value in shifts if value > tolerance]
    same = [value for value in shifts if abs(value) <= tolerance]
    return {
        "comparator_cycle_count": len(comparator),
        "candidate_cycle_count": len(candidate),
        "shared_cycle_count": len(shared),
        "comparable_cycle_count": len(shifts),
        "missing_heel_off_count": missing,
        "earlier_count": len(earlier),
        "same_count": len(same),
        "later_count": len(later),
        "maximum_advance_s": max((-value for value in earlier), default=0.0),
        "maximum_delay_s": max(later, default=0.0),
        "minimum_signed_shift_s": min(shifts) if shifts else None,
        "maximum_signed_shift_s": max(shifts) if shifts else None,
        "mean_signed_shift_s": float(np.mean(shifts)) if shifts else None,
        "role": "diagnostic_only",
    }


def _augment_stage_diagnostics(
    rows_by_dt: Mapping[str, list[dict[str, Any]]],
    details_by_dt: Mapping[str, dict[str, Any]],
    *,
    comparator_id: str,
) -> dict[str, Any]:
    summary: dict[str, Any] = {}
    for label, rows in rows_by_dt.items():
        details = details_by_dt[label]
        if comparator_id not in details:
            raise ProtocolError(f"missing V11 heel-off comparator: {comparator_id}")
        comparator_detail = details[comparator_id]
        per_candidate: dict[str, Any] = {}
        for row in rows:
            candidate_id = str(row["candidate_id"])
            critical = _critical_hs_diagnostics(details[candidate_id])
            heel_off = _heel_off_shift_diagnostics(
                comparator_detail, details[candidate_id]
            )
            row.update(
                {
                    "critical_hs_outside_50ms_count": critical[
                        "out_of_50ms_count"
                    ],
                    "heel_off_comparable_cycles": heel_off[
                        "comparable_cycle_count"
                    ],
                    "heel_off_earlier_count_vs_comparator": heel_off[
                        "earlier_count"
                    ],
                    "heel_off_same_count_vs_comparator": heel_off["same_count"],
                    "heel_off_later_count_vs_comparator": heel_off["later_count"],
                    "heel_off_max_advance_s_vs_comparator": heel_off[
                        "maximum_advance_s"
                    ],
                    "v11_structural_burden": (
                        _int_metric(
                            row, "transfer_both_latches_off_sample_count"
                        )
                        + _int_metric(
                            row, "invalid_timeout_plus_unaccepted_count"
                        )
                        + _int_metric(
                            row, "incomplete_heel_to_forefoot_transfer_count"
                        )
                    ),
                }
            )
            details[candidate_id]["v11_critical_hs"] = critical
            details[candidate_id]["v11_heel_off_vs_comparator"] = heel_off
            per_candidate[candidate_id] = {
                "critical_hs": critical,
                "heel_off_vs_comparator": heel_off,
            }
        summary[label] = {
            "comparator_id": comparator_id,
            "candidates": per_candidate,
        }
    return summary


def _write_csv(path: Path, rows: Sequence[Mapping[str, Any]]) -> None:
    if not rows:
        raise ProtocolError(f"refusing to write empty V11 metrics table: {path.name}")
    fields = sorted({str(key) for row in rows for key in row})
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def _plot_stage(
    path: Path,
    rows_10: Sequence[Mapping[str, Any]],
    rows_1: Sequence[Mapping[str, Any]],
    *,
    x_key: str,
    x_label: str,
    title: str,
    highlighted_ids: Sequence[str],
) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    runtime = {str(row["candidate_id"]): row for row in rows_10}
    fine = {str(row["candidate_id"]): row for row in rows_1}
    if set(runtime) != set(fine):
        raise ProtocolError(f"V11 plot candidate sets differ: {path.name}")
    ordered = sorted(
        runtime,
        key=lambda item: (_finite_metric(runtime[item], x_key), item),
    )
    x = np.asarray([float(runtime[item][x_key]) for item in ordered], dtype=float)
    fig, axes = plt.subplots(2, 2, figsize=(13, 9), sharex=True)
    specs = (
        ("max_abs_hs_error_s", "global max |HS| [ms]", 1000.0, 50.0),
        ("max_abs_toe_off_error_s", "global max |TO| [ms]", 1000.0, 80.0),
        (
            "v11_structural_burden",
            "gap + invalid/unaccepted + incomplete",
            1.0,
            0.0,
        ),
        (
            "heel_off_max_advance_s_vs_comparator",
            "max heel-off advance vs comparator [ms]",
            1000.0,
            0.0,
        ),
    )

    def values(rows: Mapping[str, Mapping[str, Any]], metric: str, scale: float) -> list[float]:
        result: list[float] = []
        for candidate_id in ordered:
            value = _finite_metric(rows[candidate_id], metric)
            result.append(value * scale if math.isfinite(value) else float("nan"))
        return result

    for axis, (metric, ylabel, scale, limit) in zip(axes.flat, specs):
        axis.plot(x, values(runtime, metric, scale), "o-", label="10 ms")
        axis.plot(x, values(fine, metric, scale), "s-", label="1 ms")
        axis.axhline(limit, color="#E45756", linestyle="--", alpha=0.8)
        for candidate_id in highlighted_ids:
            if candidate_id in runtime:
                axis.axvline(
                    float(runtime[candidate_id][x_key]),
                    color="#F2CF5B",
                    linewidth=5,
                    alpha=0.22,
                )
        axis.set_ylabel(ylabel)
        axis.grid(alpha=0.25)
    axes[0, 0].legend(loc="best")
    for axis in axes[-1, :]:
        axis.set_xlabel(x_label)
    fig.suptitle(title)
    fig.tight_layout()
    fig.savefig(path, dpi=170)
    plt.close(fig)


def _candidate_record(candidate: v1.PlacementCandidate) -> dict[str, Any]:
    return {
        "candidate_id": candidate.candidate_id,
        "selectable": candidate.selectable,
        "role": candidate.role,
        "heel_location_m": list(candidate.heel_location),
        "forefoot_location_m": list(candidate.forefoot_location),
        "heel_radius_m": candidate.geometry["heel_radius_m"],
        "forefoot_radius_m": candidate.geometry["toe_radius_m"],
        "heel_x_shift_mm": candidate.geometry["heel_x_shift_mm"],
        "toe_center_down_mm": candidate.geometry["toe_center_down_mm"],
        "sensor_count": 2,
        "geometry": dict(candidate.geometry),
    }


def _stage_b_anchor_identity_audit(
    anchor_a: v1.PlacementCandidate,
    comparator_b: v1.PlacementCandidate,
    protocol: Mapping[str, Any],
    rows_a: Mapping[str, Sequence[Mapping[str, Any]]],
    details_a: Mapping[str, Mapping[str, Any]],
    rows_b: Mapping[str, Sequence[Mapping[str, Any]]],
    details_b: Mapping[str, Mapping[str, Any]],
) -> dict[str, Any]:
    geometry_equal = bool(
        tuple(anchor_a.heel_location) == tuple(comparator_b.heel_location)
        and tuple(anchor_a.forefoot_location) == tuple(comparator_b.forefoot_location)
        and float(anchor_a.geometry["heel_radius_m"])
        == float(comparator_b.geometry["heel_radius_m"])
        and float(anchor_a.geometry["toe_radius_m"])
        == float(comparator_b.geometry["toe_radius_m"])
    )
    metric_keys = (
        "reference_hs_count",
        "reference_to_count",
        "predicted_hs_count",
        "predicted_to_count",
        "observed_valid_cycle_count",
        "max_abs_hs_error_s",
        "max_abs_toe_off_error_s",
        "early_max_abs_hs_error_s",
        "long_max_abs_hs_error_s",
        "confirmed_fsm_stance_f1",
        "confirmed_fsm_stance_iou",
        "transfer_both_latches_off_sample_count",
        "incomplete_heel_to_forefoot_transfer_count",
        "invalid_timeout_plus_unaccepted_count",
        "forbidden_phase_mismatch_count",
        "unknown_fsm_phase_samples",
    )
    resolution_audits: dict[str, Any] = {}
    for label in ("runtime_10ms", "fine_1ms"):
        row_a = next(
            row for row in rows_a[label]
            if str(row["candidate_id"]) == anchor_a.candidate_id
        )
        row_b = next(
            row for row in rows_b[label]
            if str(row["candidate_id"]) == comparator_b.candidate_id
        )
        metric_equal = {
            key: bool(
                np.array_equal(
                    np.asarray(row_a[key]),
                    np.asarray(row_b[key]),
                    equal_nan=True,
                )
            )
            for key in metric_keys
        }
        gate_key = (
            "runtime_gate_10ms" if label == "runtime_10ms" else "fine_gate_1ms"
        )
        strict_gate_a = v10.strict_gate(row_a, protocol[gate_key])
        strict_gate_b = v10.strict_gate(row_b, protocol[gate_key])
        strict_gate_equal = strict_gate_a == strict_gate_b
        events_a = details_a[label][anchor_a.candidate_id]["events"]
        events_b = details_b[label][comparator_b.candidate_id]["events"]
        transfer_equal = bool(
            v1._json_safe(
                details_a[label][anchor_a.candidate_id][
                    "heel_to_forefoot_transfer"
                ]
            )
            == v1._json_safe(
                details_b[label][comparator_b.candidate_id][
                    "heel_to_forefoot_transfer"
                ]
            )
        )
        event_equal: dict[str, bool] = {}
        for source in ("reference", "onset", "confirmed", "predicted"):
            for event in ("heel_strike", "toe_off"):
                key = f"{source}.{event}"
                event_equal[key] = bool(
                    np.array_equal(
                        np.asarray(events_a[source][event], dtype=float),
                        np.asarray(events_b[source][event], dtype=float),
                        equal_nan=True,
                    )
                )
        resolution_audits[label] = {
            "metric_fields_equal": metric_equal,
            "event_arrays_equal": event_equal,
            "strict_gate_equal": strict_gate_equal,
            "strict_gate_a": strict_gate_a,
            "strict_gate_b": strict_gate_b,
            "heel_to_forefoot_transfer_equal": transfer_equal,
            "ok": bool(
                all(metric_equal.values())
                and all(event_equal.values())
                and strict_gate_equal
                and transfer_equal
            ),
        }
    ok = bool(
        geometry_equal
        and all(item["ok"] for item in resolution_audits.values())
    )
    return {
        "stage_a_anchor_id": anchor_a.candidate_id,
        "stage_b_comparator_id": comparator_b.candidate_id,
        "geometry_equal": geometry_equal,
        "resolutions": resolution_audits,
        "ok": ok,
    }


def _run_stage(
    protocol: Mapping[str, Any],
    base: v1.OnlineGRFProfile,
    candidates: Sequence[v1.PlacementCandidate],
    *,
    stage_label: str,
    expected_detector_stations: int,
    expected_total_stations: int,
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
            stage_label=stage_label,
            expected_detector_stations=expected_detector_stations,
            expected_total_stations=expected_total_stations,
        )
        rows: list[dict[str, Any]] = []
        details: dict[str, Any] = {}
        for candidate in candidates:
            row, detail = evaluate_candidate(
                protocol,
                candidate,
                streams[candidate.candidate_id],
                sample_dt_s=sample_dt_s,
                stage_label=stage_label,
            )
            rows.append(row)
            details[candidate.candidate_id] = detail
        rows_by_dt[label] = rows
        details_by_dt[label] = details
        access_by_dt[label] = access
    return rows_by_dt, details_by_dt, access_by_dt


def run_sweep(
    protocol: Mapping[str, Any], output_dir: Path, plot_dir: Path
) -> dict[str, Any]:
    _preflight_no_clobber(output_dir, plot_dir)
    base_a, candidates_a, geometry_a = build_stage_a_candidates(protocol)
    rows_a, details_a, access_a = _run_stage(
        protocol,
        base_a,
        candidates_a,
        stage_label="stage_a_heel_x",
        expected_detector_stations=STAGE_A_DETECTOR_STATIONS,
        expected_total_stations=STAGE_A_TOTAL_STATIONS,
    )
    diagnostics_a = _augment_stage_diagnostics(
        rows_a, details_a, comparator_id=BASELINE_ID
    )
    selection_a = select_stage_a(
        rows_a["runtime_10ms"], rows_a["fine_1ms"], protocol
    )
    selection_a["diagnostics"] = diagnostics_a

    candidates_b: list[v1.PlacementCandidate] = []
    geometry_b: dict[str, Any] | None = None
    rows_b: dict[str, list[dict[str, Any]]] | None = None
    details_b: dict[str, dict[str, Any]] | None = None
    access_b: dict[str, Any] | None = None
    selection_b: dict[str, Any] | None = None
    anchor_identity_audit: dict[str, Any] | None = None
    anchor_id = selection_a["stage_b_anchor_id"]
    if anchor_id is not None:
        by_id = {item.candidate_id: item for item in candidates_a}
        if anchor_id not in by_id:
            raise ProtocolError("V11 Stage-B anchor is absent from Stage A")
        base_b, candidates_b, geometry_b = build_stage_b_candidates(
            protocol, by_id[str(anchor_id)]
        )
        rows_b, details_b, access_b = _run_stage(
            protocol,
            base_b,
            candidates_b,
            stage_label="stage_b_toe_compensation",
            expected_detector_stations=STAGE_B_DETECTOR_STATIONS,
            expected_total_stations=STAGE_B_TOTAL_STATIONS,
        )
        comparator_b_id = candidates_b[0].candidate_id
        diagnostics_b = _augment_stage_diagnostics(
            rows_b, details_b, comparator_id=comparator_b_id
        )
        anchor_identity_audit = _stage_b_anchor_identity_audit(
            by_id[str(anchor_id)],
            candidates_b[0],
            protocol,
            rows_a,
            details_a,
            rows_b,
            details_b,
        )
        if not anchor_identity_audit["ok"]:
            raise ProtocolError("V11 Stage-A anchor and Stage-B comparator differ")
        selection_b = select_stage_b(
            rows_b["runtime_10ms"], rows_b["fine_1ms"], protocol
        )
        selection_b["diagnostics"] = diagnostics_b

    strict_id = selection_a["strict_winner_id"]
    strict_stage = "stage_a_heel_x" if strict_id is not None else None
    if strict_id is None and selection_b is not None:
        strict_id = selection_b["strict_winner_id"]
        if strict_id is not None:
            strict_stage = "stage_b_toe_compensation"
    ok = strict_id is not None
    if ok:
        reported_id = str(strict_id)
        reported_stage = str(strict_stage)
    elif selection_b is not None:
        reported_id = str(selection_b["diagnostic_best_id"])
        reported_stage = "stage_b_toe_compensation"
    else:
        reported_id = str(selection_a["diagnostic_best_id"])
        reported_stage = "stage_a_heel_x"

    # No destination is created until both scientific stages have completed.
    output_dir.mkdir(parents=True, exist_ok=False)
    plot_dir.mkdir(parents=True, exist_ok=False)
    artifacts: dict[str, Any] = {}
    csv_a_10 = output_dir / "heel_x_v11_stage_a_runtime_10ms_metrics.csv"
    csv_a_1 = output_dir / "heel_x_v11_stage_a_fine_1ms_metrics.csv"
    plot_a = plot_dir / "01_heel_x_v11_stage_a_multiresolution.png"
    _write_csv(csv_a_10, rows_a["runtime_10ms"])
    _write_csv(csv_a_1, rows_a["fine_1ms"])
    _plot_stage(
        plot_a,
        rows_a["runtime_10ms"],
        rows_a["fine_1ms"],
        x_key="heel_x_shift_mm",
        x_label="heel local-x shift from V9 [mm]",
        title="V11 Stage A — isolated heel local-x (confirmed time)",
        highlighted_ids=(
            str(selection_a["diagnostic_best_id"]),
            *(
                [str(selection_a["strict_winner_id"])]
                if selection_a["strict_winner_id"] is not None
                else []
            ),
            *(
                [str(selection_a["stage_b_anchor_id"])]
                if selection_a["stage_b_anchor_id"] is not None
                else []
            ),
        ),
    )
    artifacts.update(
        {
            "stage_a_runtime_metrics_csv": v1._source_record(csv_a_10),
            "stage_a_fine_metrics_csv": v1._source_record(csv_a_1),
            "stage_a_multiresolution_plot": v1._source_record(plot_a),
        }
    )
    if rows_b is not None and selection_b is not None:
        csv_b_10 = output_dir / "heel_x_v11_stage_b_runtime_10ms_metrics.csv"
        csv_b_1 = output_dir / "heel_x_v11_stage_b_fine_1ms_metrics.csv"
        plot_b = plot_dir / "02_heel_x_v11_stage_b_toe_compensation.png"
        _write_csv(csv_b_10, rows_b["runtime_10ms"])
        _write_csv(csv_b_1, rows_b["fine_1ms"])
        _plot_stage(
            plot_b,
            rows_b["runtime_10ms"],
            rows_b["fine_1ms"],
            x_key="toe_center_down_mm",
            x_label="toe center local-y down from V9 [mm]",
            title="V11 Stage B — conditional toe compensation (confirmed time)",
            highlighted_ids=(
                str(selection_b["diagnostic_best_id"]),
                *(
                    [str(selection_b["strict_winner_id"])]
                    if selection_b["strict_winner_id"] is not None
                    else []
                ),
            ),
        )
        artifacts.update(
            {
                "stage_b_runtime_metrics_csv": v1._source_record(csv_b_10),
                "stage_b_fine_metrics_csv": v1._source_record(csv_b_1),
                "stage_b_multiresolution_plot": v1._source_record(plot_b),
            }
        )

    stage_b_payload: dict[str, Any]
    if rows_b is None:
        stage_b_payload = {
            "executed": False,
            "reason": selection_a["stage_b_trigger_decision"],
            "sampling_performed": False,
            "geometry": None,
            "candidates": [],
            "selection": None,
            "anchor_identity_audit": None,
            "runtime_10ms": None,
            "fine_1ms": None,
            "data_access": None,
        }
    else:
        stage_b_payload = {
            "executed": True,
            "reason": selection_a["stage_b_trigger_decision"],
            "sampling_performed": True,
            "geometry": geometry_b,
            "candidates": [_candidate_record(item) for item in candidates_b],
            "selection": selection_b,
            "anchor_identity_audit": anchor_identity_audit,
            "runtime_10ms": {
                "rows": rows_b["runtime_10ms"],
                "details": details_b["runtime_10ms"],
            },
            "fine_1ms": {
                "rows": rows_b["fine_1ms"],
                "details": details_b["fine_1ms"],
            },
            "data_access": access_b,
        }

    executed_access_records = list(access_a.values())
    if access_b is not None:
        executed_access_records.extend(access_b.values())
    maximum_sample_time_is_below_100_s = bool(
        executed_access_records
        and all(
            float(record["last_sample_s"]) < OPEN_END_S
            and int(record["samples_at_or_after_100_s"]) == 0
            and not bool(record["sealed_block_opened"])
            for record in executed_access_records
        )
    )
    if not maximum_sample_time_is_below_100_s:
        raise ProtocolError("V11 aggregate data-access audit failed")

    manifest = {
        "schema_version": SCHEMA_VERSION,
        "status": "PASS" if ok else "FAIL",
        "ok": ok,
        "stage": protocol["stage"],
        "objective": protocol.get("objective"),
        "protocol": {
            "path": v1._portable_path(Path(str(protocol["_protocol_path"]))),
            "sha256": protocol["_protocol_sha256"],
            "protocol_id": protocol["protocol_id"],
            "frozen_before_execution": True,
        },
        "data_access": {
            "opened_interval_s": [OPEN_START_S, OPEN_END_S],
            "upper_bound_is_exclusive": True,
            "maximum_sample_time_is_below_100_s": (
                maximum_sample_time_is_below_100_s
            ),
            "sealed_block_s": [OPEN_END_S, 155.045],
            "sealed_block_opened": False,
            "stage_a": access_a,
            "stage_b": access_b,
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
        "stage_a": {
            "executed": True,
            "geometry": geometry_a,
            "candidates": [_candidate_record(item) for item in candidates_a],
            "selection": selection_a,
            "runtime_10ms": {
                "rows": rows_a["runtime_10ms"],
                "details": details_a["runtime_10ms"],
            },
            "fine_1ms": {
                "rows": rows_a["fine_1ms"],
                "details": details_a["fine_1ms"],
            },
        },
        "stage_b": stage_b_payload,
        "reported_candidate": {
            "candidate_id": reported_id,
            "source_stage": reported_stage,
            "strict_multiresolution_pass": ok,
            "diagnostic_only": not ok,
            "profile_created": False,
            "promotable": False,
            "requires_future_frozen_validation": bool(ok),
        },
        "conclusion": (
            "STRICT_V11_STAGE_A_WINNER"
            if strict_stage == "stage_a_heel_x"
            else "STRICT_V11_STAGE_B_WINNER"
            if strict_stage == "stage_b_toe_compensation"
            else "NO_STRICT_V11_WINNER_STAGE_B_DIAGNOSTIC_ONLY"
            if rows_b is not None
            else "NO_STRICT_V11_WINNER_STAGE_B_SKIPPED"
        ),
        "artifacts": artifacts,
        "non_actions": {
            "v9_modified": False,
            "v10_modified": False,
            "threshold_or_fsm_sweep_run": False,
            "policy_or_training_run": False,
            "runtime_configuration_modified": False,
            "production_fsm_modified": False,
            "candidate_profile_created_or_promoted": False,
            "sealed_block_opened": False,
        },
        "interpretation_limits": protocol.get("interpretation_limits", []),
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
        description=(
            "Run preregistered V11 heel local-x sweep and conditional toe "
            "compensation."
        )
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
        _preflight_no_clobber(output_dir, plot_dir)
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
                "stage_b_executed": manifest["stage_b"]["executed"],
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
