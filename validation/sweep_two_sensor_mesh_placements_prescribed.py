"""Preregistered development sweep for a two-sensor forefoot placement.

The heel sensor is frozen.  Every selectable candidate contains exactly that
heel plus one forefoot sphere derived from the prosthetic-foot mesh by a
protocol-declared geometric rule.  All candidate spheres are sampled in one
OpenSim pass at 10 ms, then evaluated independently by the production
``ProstheticPhaseFSM`` with the frozen 0.5/0.25 N, 30 ms detector contract.
Only a preregistered 10 ms winner may be sampled again at 1 ms.

This is development, not holdout validation.  It uses only the already-open
50--100 s block and the same common 50-cycle set as the final V4 fixed replay.
The 100--155.045 s block remains sealed.  The script never trains or evaluates
a policy and never edits a detector profile, runtime config, or FSM.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import sys
import traceback
from dataclasses import dataclass, replace
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
from validation.audit_two_sensor_prescribed_geometry import (  # noqa: E402
    _load_stl_triangles,
    _minimum_mesh_distance,
    _resolve_left_foot_mesh,
)
from validation.build_two_sensor_mesh_profile_v4 import (  # noqa: E402
    _section_z_bounds_at_x,
    _vertical_surface_intersections_y,
)
from validation.compare_two_sensor_mesh_profiles_prescribed import (  # noqa: E402
    BLOCK_START_S,
    CURRENT_PROFILE_ID,
    PRIMARY_DT_S,
    SEALED_END_S,
    SEALED_START_S,
    SENSOR_DWELL_S,
    SENSOR_OFF_N,
    SENSOR_ON_N,
    SENSITIVITY_DT_S,
    V4_PROFILE_ID,
    _build_time_grid,
    _contact_inputs,
    _exclude_unconfirmable_right_boundary_cycles,
    _regional_continuity_diagnostics,
)
from validation.sweep_two_sensor_prescribed_thresholds import (  # noqa: E402
    Candidate,
    _evaluate_candidate,
    evaluate_holdout_gate,
)
from validation.validate_online_grf import (  # noqa: E402
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
    REPO_ROOT / "validation/two_sensor_mesh_placement_sweep_protocol.json"
)
DEFAULT_OUTPUT_DIR = (
    REPO_ROOT
    / "validation/two_sensor_mesh_placement_sweep_runs/"
    "2026-07-22_ab06_50_100_development"
)
DEFAULT_PLOT_DIR = (
    REPO_ROOT / "plot/07_22_2026_two_sensor_mesh_placement_development"
)
PROTOCOL_ID = "AB06_TWO_SENSOR_MESH_PLACEMENT_DEVELOPMENT_2026-07-22_V1"
STRICT_WINNER_RANKING = [
    {"metric": "worst_event_normalized_max_abs_error", "direction": "min"},
    {"metric": "mean_event_normalized_mean_abs_error", "direction": "min"},
    {"metric": "confirmed_fsm_stance_iou", "direction": "max"},
    {"metric": "geometry_displacement_from_current_m", "direction": "min"},
]
DIAGNOSTIC_BEST_RANKING = [
    {"metric": "observed_valid_cycle_count", "direction": "max"},
    {"metric": "event_count_deficit", "direction": "min"},
    {
        "metric": "invalid_timeout_plus_unaccepted_count",
        "direction": "min",
    },
    {"metric": "confirmed_fsm_stance_f1", "direction": "max"},
    {"metric": "confirmed_fsm_stance_iou", "direction": "max"},
    {"metric": "maximum_heel_off_to_forefoot_gap_s", "direction": "min"},
    {"metric": "geometry_displacement_from_current_m", "direction": "min"},
]


class ProtocolError(ValueError):
    """Raised when the development protocol or execution drifts."""


@dataclass(frozen=True)
class PlacementCandidate:
    candidate_id: str
    heel_location: tuple[float, float, float]
    forefoot_location: tuple[float, float, float]
    heel_offset_below_current_mm: float | None
    forefoot_fraction_mesh_x: float | None
    forefoot_protrusion_mm: float | None
    selectable: bool
    role: str
    geometry: Mapping[str, Any]


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
    protocol_path = resolve_repo_path(path).resolve()
    try:
        raw = json.loads(protocol_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ProtocolError(f"cannot load protocol: {protocol_path}") from exc
    if not isinstance(raw, dict) or raw.get("schema_version") != 1:
        raise ProtocolError("unsupported placement-sweep protocol schema")
    if raw.get("protocol_id") != PROTOCOL_ID:
        raise ProtocolError("unexpected placement-sweep protocol id")
    if raw.get("frozen_before_execution") is not True:
        raise ProtocolError("protocol must be frozen before execution")
    if raw.get("stage") != "development":
        raise ProtocolError("this validator is development-only")

    access = raw.get("data_access")
    if not isinstance(access, dict):
        raise ProtocolError("data_access is required")
    if access.get("already_open_block_s") != [BLOCK_START_S, SEALED_START_S]:
        raise ProtocolError("only the already-open 50--100 s block is allowed")
    if access.get("upper_bound_is_exclusive") is not True:
        raise ProtocolError("the 100 s boundary must remain exclusive")
    if access.get("allow_samples_at_or_after_100_s") is not False:
        raise ProtocolError("samples at or after 100 s are forbidden")
    if access.get("sealed_block_s") != [SEALED_START_S, SEALED_END_S]:
        raise ProtocolError("sealed block drifted")
    expected_common = access.get("expected_common_cycle_set")
    if not isinstance(expected_common, dict):
        raise ProtocolError("expected common-cycle set is required")
    if expected_common.get("original_complete_cycles") != 51:
        raise ProtocolError("expected 51 original complete cycles")
    if expected_common.get("retained_complete_cycles") != 50:
        raise ProtocolError("expected exactly 50 retained common cycles")
    if expected_common.get("excluded_closing_hs_s") != [99.96878691565038]:
        raise ProtocolError("common-cycle exclusion drifted")

    replay = raw.get("replay")
    if not isinstance(replay, dict):
        raise ProtocolError("replay is required")
    frozen_numbers = {
        "sensor_on_threshold_n": SENSOR_ON_N,
        "sensor_off_threshold_n": SENSOR_OFF_N,
        "sensor_dwell_s": SENSOR_DWELL_S,
        "primary_sample_dt_s": PRIMARY_DT_S,
        "winner_sensitivity_sample_dt_s": SENSITIVITY_DT_S,
    }
    for key, expected in frozen_numbers.items():
        if _finite(replay.get(key), key) != expected:
            raise ProtocolError(f"frozen replay value drifted: {key}")
    if replay.get("event_source") != "two_sensor":
        raise ProtocolError("event_source must be two_sensor")
    if replay.get("primary_event_time_field") != "confirmed_time_s":
        raise ProtocolError("confirmed_time_s must remain primary")
    if replay.get("diagnostic_event_time_field") != "event_time_s":
        raise ProtocolError("event_time_s must remain diagnostic")
    if replay.get("phase_reference_mode") != "validated_event_intervals":
        raise ProtocolError("phase reference semantics drifted")
    if replay.get("primary_evaluate_all_candidates") is not True:
        raise ProtocolError("10 ms must evaluate every candidate")
    if replay.get("sensitivity_evaluate_winner_only") is not True:
        raise ProtocolError("1 ms must evaluate only the locked winner")

    grid = raw.get("placement_grid")
    if not isinstance(grid, dict):
        raise ProtocolError("placement_grid is required")
    if grid.get("x_rule") != "mesh_bbox_longitudinal_fraction":
        raise ProtocolError("x_rule must remain mesh-bbox fraction")
    if grid.get("z_rule") != "midpoint_of_mesh_section_at_candidate_x":
        raise ProtocolError("z_rule must remain mesh-section midpoint")
    if grid.get("forefoot_height_rule") != (
        "absolute_sphere_bottom_below_local_plantar_surface"
    ):
        raise ProtocolError("forefoot height rule drifted")
    if grid.get("heel_offset_rule") != "local_vertical_below_current_center":
        raise ProtocolError("heel offset rule drifted")
    heel_offsets = [
        _finite(item, "heel offset")
        for item in grid.get("heel_vertical_offsets_below_current_mm", [])
    ]
    if heel_offsets != [0.0, 2.0]:
        raise ProtocolError("heel-offset axis must remain [0,2] mm")
    fractions = [
        _finite(item, "forefoot fraction")
        for item in grid.get("forefoot_longitudinal_fractions_mesh_x", [])
    ]
    if fractions != [0.60, 0.70, 0.80]:
        raise ProtocolError("forefoot fraction axis must remain [0.60,0.70,0.80]")
    protrusions = [
        _finite(item, "forefoot protrusion")
        for item in grid.get("forefoot_absolute_local_plantar_protrusion_mm", [])
    ]
    if protrusions != [25.0, 30.0]:
        raise ProtocolError("forefoot protrusion axis must remain [25,30] mm")
    if grid.get("cartesian_candidate_count") != 12:
        raise ProtocolError("final grid must contain exactly 12 pairs")
    prescreen = grid.get("development_prescreen")
    if not isinstance(prescreen, dict) or prescreen.get("role") != (
        "non_gating_grid_design_only"
    ):
        raise ProtocolError("development pre-screen provenance is required")
    if prescreen.get("rejected_height_rules") != [
        "same_plantar_protrusion_as_selected_heel",
        "flat_sensor_bottom_with_selected_heel",
    ]:
        raise ProtocolError("pre-screen rejected-rule record drifted")

    selection = raw.get("selection")
    if not isinstance(selection, dict):
        raise ProtocolError("selection contract is required")
    if selection.get("strict_gate") != "V3_confirmed_time_holdout_gate":
        raise ProtocolError("strict gate must reuse the V3 confirmed-time gate")
    physical_gate = selection.get("physical_continuity_pre_gate")
    expected_physical_gate = {
        "maximum_heel_off_to_forefoot_gap_s": 0.010,
        "maximum_interior_both_off_gap_s": 0.010,
        "require_forefoot_present_every_stance": True,
        "require_forefoot_off_observed_every_swing": True,
        "require_forefoot_off_before_next_hs": True,
        "require_exact_event_counts_order_and_cycles": True,
        "maximum_invalid_or_timeout_transitions": 0,
        "maximum_unaccepted_sensor_events": 0,
    }
    if physical_gate != expected_physical_gate:
        raise ProtocolError("physical continuity pre-gate drifted")
    if selection.get("strict_winner_ranking") != STRICT_WINNER_RANKING:
        raise ProtocolError("strict-winner ranking drifted")
    if selection.get("diagnostic_best_ranking") != DIAGNOSTIC_BEST_RANKING:
        raise ProtocolError("diagnostic-best ranking drifted")
    if selection.get("tie_breaker") != "lexicographically_lowest_candidate_id":
        raise ProtocolError("candidate-id tie breaker must be deterministic")
    if selection.get("no_strict_winner_policy") != (
        "lock_diagnostic_best_without_promotion_and_run_1ms_diagnostic"
    ):
        raise ProtocolError("no-strict-winner policy drifted")
    if selection.get("diagnostic_1ms_cannot_convert_fail_to_pass") is not True:
        raise ProtocolError("diagnostic 1 ms must never convert a 10 ms failure")
    if selection.get("winner_1ms_gate") != "V3_confirmed_time_holdout_gate":
        raise ProtocolError("winner 1 ms must reuse the V3 gate")

    comparators = raw.get("comparators")
    if not isinstance(comparators, dict):
        raise ProtocolError("comparators mapping is required")
    if set(comparators) - {CURRENT_PROFILE_ID, V4_PROFILE_ID}:
        raise ProtocolError("only current and rejected V4 comparators are supported")
    if comparators != {CURRENT_PROFILE_ID: True, V4_PROFILE_ID: True}:
        raise ProtocolError("both current and rejected-V4 comparators are required")

    gate = raw.get("sealed_validation_gate")
    if not isinstance(gate, dict):
        raise ProtocolError("V3 strict gate payload is required")
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
        if _finite(gate.get(key), f"sealed_validation_gate.{key}") != expected:
            raise ProtocolError(f"V3 strict gate drifted: {key}")

    sources = raw.get("sources")
    if not isinstance(sources, dict) or not sources:
        raise ProtocolError("hash-pinned sources are required")
    for label, record in sources.items():
        if not isinstance(record, dict):
            raise ProtocolError(f"invalid source record: {label}")
        source_path = resolve_repo_path(str(record.get("path", ""))).resolve()
        if not source_path.is_file():
            raise ProtocolError(f"missing pinned source {label}: {source_path}")
        observed = _sha256(source_path)
        if observed != record.get("sha256"):
            raise ProtocolError(
                f"source hash drift for {label}: {observed} != "
                f"{record.get('sha256')}"
            )

    raw["_protocol_path"] = protocol_path.as_posix()
    raw["_protocol_sha256"] = _sha256(protocol_path)
    raw["_primary_event_time_field"] = "confirmed_time_s"
    raw["_diagnostic_event_time_field"] = "event_time_s"
    raw["_phase_reference_mode"] = "validated_event_intervals"
    return raw


def _derive_forefoot_location(
    triangles: np.ndarray,
    forefoot_template: OnlineGRFSphere,
    *,
    fraction: float,
    protrusion_mm: float,
) -> tuple[tuple[float, float, float], dict[str, Any]]:
    vertices = np.asarray(triangles, dtype=float).reshape(-1, 3)
    bounds_min = np.min(vertices, axis=0)
    bounds_max = np.max(vertices, axis=0)
    radius = float(forefoot_template.radius)
    anterior_center_x = float(bounds_max[0] - radius)
    x_value = float(bounds_min[0] + fraction * (bounds_max[0] - bounds_min[0]))
    if x_value > anterior_center_x + 1e-12:
        raise ProtocolError("candidate sphere center exceeds mesh anterior allowance")

    section_z_min, section_z_max = _section_z_bounds_at_x(triangles, x_value)
    z_value = float(0.5 * (section_z_min + section_z_max))
    surface_hits = _vertical_surface_intersections_y(
        triangles, x_value, z_value
    )
    plantar_y = min(surface_hits)
    protrusion = float(protrusion_mm) / 1000.0
    y_value = float(plantar_y - protrusion + radius)
    location = (x_value, y_value, z_value)
    mesh_distance = _minimum_mesh_distance(location, triangles)
    if mesh_distance > radius + 0.005:
        raise ProtocolError("derived forefoot sphere is more than 5 mm from mesh")
    return location, {
        "x_rule": "mesh_bbox_longitudinal_fraction",
        "longitudinal_fraction": float(fraction),
        "absolute_local_plantar_protrusion_mm": float(protrusion_mm),
        "mesh_bounds_min_m": bounds_min.tolist(),
        "mesh_bounds_max_m": bounds_max.tolist(),
        "anterior_center_limit_x_m": anterior_center_x,
        "section_z_bounds_m": [section_z_min, section_z_max],
        "plantar_surface_y_m": plantar_y,
        "sphere_center_to_mesh_m": float(mesh_distance),
        "sphere_surface_gap_to_mesh_m": max(0.0, mesh_distance - radius),
        "forefoot_bottom_y_m": float(y_value - radius),
    }


def build_placement_candidates(
    protocol: Mapping[str, Any],
) -> tuple[
    OnlineGRFProfile,
    list[PlacementCandidate],
    dict[str, Any],
]:
    base_path = resolve_repo_path(
        str(protocol["profile_paths"][CURRENT_PROFILE_ID])
    ).resolve()
    v4_path = resolve_repo_path(
        str(protocol["profile_paths"][V4_PROFILE_ID])
    ).resolve()
    base = load_online_grf_profile(base_path)
    rejected_v4 = load_online_grf_profile(v4_path)
    base_sensors = _left_sensor_spheres(base)
    v4_sensors = _left_sensor_spheres(rejected_v4)
    if base_sensors["left_heel"] != v4_sensors["left_heel"]:
        raise ProtocolError("current and V4 profiles must share the frozen heel")

    setup = read_setup_xml(resolve_repo_path(str(protocol["setup"])).resolve())
    mesh_path = _resolve_left_foot_mesh(setup.model_file.resolve())
    triangles = _load_stl_triangles(mesh_path)
    heel = base_sensors["left_heel"]
    template = base_sensors["left_toe"]
    if not math.isclose(heel.radius, template.radius, rel_tol=0.0, abs_tol=1e-12):
        raise ProtocolError("heel and forefoot radii must remain equal")
    grid = protocol["placement_grid"]
    candidates: list[PlacementCandidate] = []
    for heel_offset_raw in grid["heel_vertical_offsets_below_current_mm"]:
        heel_offset_mm = float(heel_offset_raw)
        heel_location = (
            float(heel.location[0]),
            float(heel.location[1] - heel_offset_mm / 1000.0),
            float(heel.location[2]),
        )
        for fraction_raw in grid["forefoot_longitudinal_fractions_mesh_x"]:
            fraction = float(fraction_raw)
            for protrusion_raw in grid[
                "forefoot_absolute_local_plantar_protrusion_mm"
            ]:
                protrusion_mm = float(protrusion_raw)
                forefoot_location, geometry = _derive_forefoot_location(
                    triangles,
                    template,
                    fraction=fraction,
                    protrusion_mm=protrusion_mm,
                )
                candidate_id = (
                    f"H{int(round(heel_offset_mm)):02d}_"
                    f"F{int(round(100.0 * fraction)):02d}_"
                    f"P{int(round(protrusion_mm)):02d}"
                )
                geometry = {
                    **geometry,
                    "heel_vertical_offset_below_current_mm": heel_offset_mm,
                    "heel_location_m": list(heel_location),
                }
                candidates.append(
                    PlacementCandidate(
                        candidate_id=candidate_id,
                        heel_location=heel_location,
                        forefoot_location=forefoot_location,
                        heel_offset_below_current_mm=heel_offset_mm,
                        forefoot_fraction_mesh_x=fraction,
                        forefoot_protrusion_mm=protrusion_mm,
                        selectable=True,
                        role="development_candidate",
                        geometry=geometry,
                    )
                )
    if len(candidates) != 12:
        raise ProtocolError(f"expected exactly 12 Cartesian pairs, got {len(candidates)}")

    comparators = protocol["comparators"]
    if comparators.get(CURRENT_PROFILE_ID, False):
        candidates.append(
            PlacementCandidate(
                candidate_id=CURRENT_PROFILE_ID,
                heel_location=tuple(base_sensors["left_heel"].location),
                forefoot_location=tuple(base_sensors["left_toe"].location),
                heel_offset_below_current_mm=None,
                forefoot_fraction_mesh_x=None,
                forefoot_protrusion_mm=None,
                selectable=False,
                role="nonselectable_comparator",
                geometry={"source": "current_profile"},
            )
        )
    if comparators.get(V4_PROFILE_ID, False):
        candidates.append(
            PlacementCandidate(
                candidate_id=V4_PROFILE_ID,
                heel_location=tuple(v4_sensors["left_heel"].location),
                forefoot_location=tuple(v4_sensors["left_toe"].location),
                heel_offset_below_current_mm=None,
                forefoot_fraction_mesh_x=None,
                forefoot_protrusion_mm=None,
                selectable=False,
                role="nonselectable_rejected_comparator",
                geometry={"source": "rejected_v4_profile"},
            )
        )
    ids = [item.candidate_id for item in candidates]
    if len(ids) != len(set(ids)):
        raise ProtocolError("placement candidate ids are not unique")
    return base, candidates, {
        "mesh": _source_record(mesh_path),
        "heel_location_m": list(heel.location),
        "heel_radius_m": float(heel.radius),
        "candidate_count": len(candidates),
        "selectable_candidate_count": sum(item.selectable for item in candidates),
        "direct_unique_sphere_sampling": True,
        "affine_reconstruction_used": False,
    }


def _sampling_bundle(
    base: OnlineGRFProfile,
    candidates: Sequence[PlacementCandidate],
) -> tuple[
    OnlineGRFProfile,
    dict[str, dict[str, OnlineGRFSphere]],
    dict[str, OnlineGRFProfile],
]:
    sensors = _left_sensor_spheres(base)
    heel_template = sensors["left_heel"]
    forefoot_template = sensors["left_toe"]
    sampling_spheres: list[OnlineGRFSphere] = []
    sphere_cache: dict[tuple[str, tuple[float, float, float]], OnlineGRFSphere] = {}
    sphere_pairs: dict[str, dict[str, OnlineGRFSphere]] = {}
    profiles: dict[str, OnlineGRFProfile] = {}

    def unique_sphere(
        kind: str,
        template: OnlineGRFSphere,
        location: tuple[float, float, float],
    ) -> OnlineGRFSphere:
        key = (kind, tuple(float(item) for item in location))
        if key not in sphere_cache:
            sphere = replace(
                template,
                name=f"placement_{kind}_{len(sphere_cache):02d}",
                location=key[1],
            )
            sphere_cache[key] = sphere
            sampling_spheres.append(sphere)
        return sphere_cache[key]

    for candidate in candidates:
        heel = unique_sphere(
            "heel", heel_template, tuple(candidate.heel_location)
        )
        forefoot = unique_sphere(
            "forefoot", forefoot_template, tuple(candidate.forefoot_location)
        )
        sphere_pairs[candidate.candidate_id] = {
            "heel": heel,
            "toe": forefoot,
        }
        profiles[candidate.candidate_id] = replace(
            base,
            source=f"validation_placement_{candidate.candidate_id}",
            spheres=(heel, forefoot),
        )
    sampler = replace(
        base,
        source="validation_all_forefoot_candidates_single_pass",
        spheres=tuple(sampling_spheres),
    )
    return sampler, sphere_pairs, profiles


def _reference_bundle(
    protocol: Mapping[str, Any],
    *,
    sample_dt_s: float,
) -> tuple[Any, dict[str, np.ndarray], dict[str, Any], np.ndarray]:
    setup = replace(
        read_setup_xml(resolve_repo_path(str(protocol["setup"])).resolve()),
        t_start=BLOCK_START_S,
        t_end=float(np.nextafter(SEALED_START_S, -np.inf)),
    )
    replay = protocol["replay"]
    raw_events, provenance = _reference_events_from_prescribed_grf(
        setup,
        threshold_n=float(replay["prescribed_contact_threshold_n"]),
        min_contact_duration_s=float(replay["reference_min_contact_duration_s"]),
        min_cycle_duration_s=float(replay["reference_min_cycle_duration_s"]),
    )
    events, boundary_audit = _exclude_unconfirmable_right_boundary_cycles(
        raw_events, sample_dt_s=sample_dt_s
    )
    expected = protocol["data_access"]["expected_common_cycle_set"]
    if int(boundary_audit["original_complete_cycle_count"]) != int(
        expected["original_complete_cycles"]
    ):
        raise ProtocolError("original common-cycle count drifted")
    if int(boundary_audit["causally_confirmable_cycle_count"]) != int(
        expected["retained_complete_cycles"]
    ):
        raise ProtocolError("retained common-cycle count drifted")
    if boundary_audit["excluded_closing_hs_s"] != expected["excluded_closing_hs_s"]:
        raise ProtocolError("excluded common-cycle HS drifted")
    times, effective_end_s = _build_time_grid(events, sample_dt_s=sample_dt_s)
    if np.any(times >= SEALED_START_S):
        raise ProtocolError("time grid touched the sealed block")
    setup = replace(setup, t_end=float(times[-1]))
    access = {
        "common_cycle_audit": boundary_audit,
        "effective_end_s": effective_end_s,
        "first_sample_s": float(times[0]),
        "last_sample_s": float(times[-1]),
        "sample_count": int(times.size),
        "samples_at_or_after_100_s": int(np.count_nonzero(times >= SEALED_START_S)),
        "sealed_block_opened": False,
        "reference_provenance": provenance,
    }
    return setup, events, access, times


def _sample_all_candidates(
    protocol: Mapping[str, Any],
    base: OnlineGRFProfile,
    candidates: Sequence[PlacementCandidate],
    *,
    sample_dt_s: float,
) -> tuple[dict[str, dict[str, Any]], dict[str, Any]]:
    setup, events, access, times = _reference_bundle(
        protocol, sample_dt_s=sample_dt_s
    )
    sampler, sphere_pairs, profiles = _sampling_bundle(base, candidates)
    samples = _sample_spheres(
        setup,
        sampler,
        times,
        str(protocol["replay"]["sea_plugin"]),
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
        "reference_events": events,
        "body_weight_n": _model_body_weight_n(setup.model_file),
    }
    result: dict[str, dict[str, Any]] = {}
    for candidate in candidates:
        candidate_id = candidate.candidate_id
        loads, penetrations, aggregate = _contact_inputs(
            profiles[candidate_id], sphere_pairs[candidate_id], samples
        )
        result[candidate_id] = {
            **shared,
            "profile": profiles[candidate_id],
            "loads": loads,
            "penetrations": penetrations,
            "aggregate": aggregate,
        }
    access["single_opensim_sphere_sampling_pass"] = True
    access["direct_sphere_sampling_without_affine_reconstruction"] = True
    access["sampled_unique_sphere_count"] = len(sampler.spheres)
    access["evaluated_pair_count"] = len(candidates)
    return result, access


def _median_cycle_metric(continuity: Mapping[str, Any], name: str) -> float:
    values = [
        float(item[name])
        for item in continuity["cycles"]
        if item.get(name) is not None
    ]
    return float(np.median(values)) if values else float("inf")


def _forefoot_swing_diagnostics(
    inputs: Mapping[str, Any],
    *,
    sample_dt_s: float,
) -> dict[str, Any]:
    """Check that the debounced forefoot latch releases during every swing."""
    fsm_config = replace(
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
        fsm_config=fsm_config,
    )
    times = np.asarray(inputs["times"], dtype=float)
    forefoot = np.asarray(replay["toe_contact"], dtype=float) > 0.5
    hs = np.asarray(inputs["reference_events"]["heel_strike"], dtype=float)
    toe_off = np.asarray(inputs["reference_events"]["toe_off"], dtype=float)
    swings: list[dict[str, Any]] = []
    for index, (toe_off_s, next_hs_s) in enumerate(zip(toe_off, hs[1:])):
        indices = np.flatnonzero((times >= toe_off_s) & (times < next_hs_s))
        if indices.size == 0:
            raise ProtocolError(f"empty prescribed swing interval at cycle {index}")
        active = forefoot[indices]
        off_observed = bool(np.any(~active))
        off_before_next_hs = bool(not active[-1])
        swings.append(
            {
                "cycle_index": index,
                "reference_to_s": float(toe_off_s),
                "next_reference_hs_s": float(next_hs_s),
                "forefoot_off_observed": off_observed,
                "forefoot_off_at_last_sample_before_next_hs": off_before_next_hs,
                "forefoot_active_sample_count": int(np.count_nonzero(active)),
                "forefoot_active_duration_s": float(
                    np.count_nonzero(active) * sample_dt_s
                ),
            }
        )
    return {
        "role": "physical_continuity_pre_gate",
        "swing_count": len(swings),
        "swings_without_forefoot_off_count": int(
            sum(not item["forefoot_off_observed"] for item in swings)
        ),
        "forefoot_contact_crossing_next_hs_count": int(
            sum(
                not item["forefoot_off_at_last_sample_before_next_hs"]
                for item in swings
            )
        ),
        "swings": swings,
    }


def _evaluate_placement(
    protocol: Mapping[str, Any],
    candidate: PlacementCandidate,
    inputs: Mapping[str, Any],
    *,
    sample_dt_s: float,
) -> tuple[dict[str, Any], dict[str, Any]]:
    row, detail = _evaluate_candidate(
        Candidate(candidate.candidate_id, SENSOR_ON_N, SENSOR_OFF_N),
        inputs,
        protocol,
        sample_dt_s=sample_dt_s,
    )
    continuity = _regional_continuity_diagnostics(
        inputs, sample_dt_s=sample_dt_s
    )
    swing_diagnostics = _forefoot_swing_diagnostics(
        inputs, sample_dt_s=sample_dt_s
    )
    summary = continuity["summary"]
    interior_gap = summary["maximum_interior_both_off_gap_s"]
    heel_forefoot_gap = summary["maximum_heel_off_to_toe_on_gap_s"]
    current_sensors = _left_sensor_spheres(
        load_online_grf_profile(
            resolve_repo_path(
                str(protocol["profile_paths"][CURRENT_PROFILE_ID])
            ).resolve()
        )
    )
    geometry_displacement = float(
        np.linalg.norm(
            np.asarray(candidate.heel_location)
            - np.asarray(current_sensors["left_heel"].location)
        )
        + np.linalg.norm(
            np.asarray(candidate.forefoot_location)
            - np.asarray(current_sensors["left_toe"].location)
        )
    )
    row.update(
        {
            "candidate_id": candidate.candidate_id,
            "selectable": bool(candidate.selectable),
            "candidate_role": candidate.role,
            "heel_offset_below_current_mm": candidate.heel_offset_below_current_mm,
            "forefoot_fraction_mesh_x": candidate.forefoot_fraction_mesh_x,
            "forefoot_protrusion_mm": candidate.forefoot_protrusion_mm,
            "heel_x_m": float(candidate.heel_location[0]),
            "heel_y_m": float(candidate.heel_location[1]),
            "heel_z_m": float(candidate.heel_location[2]),
            "forefoot_x_m": float(candidate.forefoot_location[0]),
            "forefoot_y_m": float(candidate.forefoot_location[1]),
            "forefoot_z_m": float(candidate.forefoot_location[2]),
            "geometry_displacement_from_current_m": geometry_displacement,
            "event_count_deficit": int(
                abs(row["reference_hs_count"] - row["predicted_hs_count"])
                + abs(row["reference_to_count"] - row["predicted_to_count"])
            ),
            "invalid_timeout_plus_unaccepted_count": int(
                row["invalid_or_timeout_transition_count"]
                + row["unaccepted_sensor_gait_event_count"]
            ),
            "forefoot_missing_stance_count": int(
                summary["cycles_missing_stable_toe_contact"]
            ),
            "swings_without_forefoot_off_count": int(
                swing_diagnostics["swings_without_forefoot_off_count"]
            ),
            "forefoot_contact_crossing_next_hs_count": int(
                swing_diagnostics["forefoot_contact_crossing_next_hs_count"]
            ),
            "maximum_interior_both_off_gap_s": summary[
                "maximum_interior_both_off_gap_s"
            ] if interior_gap is not None else 999.0,
            "maximum_heel_off_to_forefoot_gap_s": (
                heel_forefoot_gap if heel_forefoot_gap is not None else 999.0
            ),
            "minimum_forefoot_contact_duration_s": summary[
                "minimum_toe_contact_duration_s"
            ],
            "maximum_forefoot_contact_duration_s": summary[
                "maximum_toe_contact_duration_s"
            ],
            "median_forefoot_contact_duration_s": _median_cycle_metric(
                continuity, "toe_contact_duration_s"
            ),
            "median_forefoot_onset_after_hs_s": _median_cycle_metric(
                continuity, "toe_on_after_hs_s"
            ),
        }
    )
    detail["placement"] = {
        "candidate_id": candidate.candidate_id,
        "selectable": candidate.selectable,
        "heel_location_m": list(candidate.heel_location),
        "forefoot_location_m": list(candidate.forefoot_location),
        "heel_offset_below_current_mm": candidate.heel_offset_below_current_mm,
        "forefoot_fraction_mesh_x": candidate.forefoot_fraction_mesh_x,
        "forefoot_protrusion_mm": candidate.forefoot_protrusion_mm,
        "geometry": dict(candidate.geometry),
    }
    detail["regional_continuity"] = continuity
    detail["forefoot_swing_release"] = swing_diagnostics
    return row, detail


def _rank_key(
    row: Mapping[str, Any],
    ranking: Sequence[Mapping[str, str]],
) -> tuple[Any, ...]:
    values: list[Any] = []
    for item in ranking:
        metric = str(item["metric"])
        value = _finite(row.get(metric), f"rank metric {metric}")
        values.append(value if item["direction"] == "min" else -value)
    values.append(str(row["candidate_id"]))
    return tuple(values)


def physical_continuity_pre_gate(
    row: Mapping[str, Any],
    protocol: Mapping[str, Any],
) -> dict[str, Any]:
    gate = protocol["selection"]["physical_continuity_pre_gate"]
    exact_cycles = bool(
        row["exact_reference_and_detector_event_counts"]
        and row["exact_hs_to_toe_off_to_hs_order_and_cycle_count"]
        and int(row["observed_valid_cycle_count"])
        == int(row["reference_to_count"])
    )
    checks = {
        "heel_off_to_forefoot_gap": float(
            row["maximum_heel_off_to_forefoot_gap_s"]
        )
        <= float(gate["maximum_heel_off_to_forefoot_gap_s"]),
        "interior_both_off_gap": float(row["maximum_interior_both_off_gap_s"])
        <= float(gate["maximum_interior_both_off_gap_s"]),
        "forefoot_present_every_stance": int(
            row["forefoot_missing_stance_count"]
        )
        == 0,
        "forefoot_off_observed_every_swing": int(
            row["swings_without_forefoot_off_count"]
        )
        == 0,
        "forefoot_off_before_next_hs": int(
            row["forefoot_contact_crossing_next_hs_count"]
        )
        == 0,
        "exact_event_counts_order_and_cycles": exact_cycles,
        "no_invalid_or_timeout": int(row["invalid_or_timeout_transition_count"])
        <= int(gate["maximum_invalid_or_timeout_transitions"]),
        "no_unaccepted_sensor_events": int(
            row["unaccepted_sensor_gait_event_count"]
        )
        <= int(gate["maximum_unaccepted_sensor_events"]),
    }
    return {"ok": bool(all(checks.values())), "checks": checks}


def select_development_outcome(
    rows: Sequence[Mapping[str, Any]],
    protocol: Mapping[str, Any],
) -> tuple[str, str, dict[str, Any]]:
    current_rows = [
        row for row in rows if row["candidate_id"] == CURRENT_PROFILE_ID
    ]
    if len(current_rows) != 1:
        raise ProtocolError("exactly one current comparator row is required")
    current = current_rows[0]
    selectable = [row for row in rows if row.get("selectable")]
    if len(selectable) != 12:
        raise ProtocolError("selection requires exactly 12 selectable pairs")

    assessed: list[dict[str, Any]] = []
    eligible_rows: list[Mapping[str, Any]] = []
    for row in selectable:
        physical_gate = physical_continuity_pre_gate(row, protocol)
        v3_gate = evaluate_holdout_gate(row, current, protocol)
        assessed.append(
            {
                "candidate_id": row["candidate_id"],
                "physical_continuity_pre_gate": physical_gate,
                "v3_gate": v3_gate,
                "strict_eligible": bool(physical_gate["ok"] and v3_gate["ok"]),
            }
        )
        if physical_gate["ok"] and v3_gate["ok"]:
            eligible_rows.append(row)
    if eligible_rows:
        ranking = protocol["selection"]["strict_winner_ranking"]
        ordered = sorted(eligible_rows, key=lambda row: _rank_key(row, ranking))
        selected_id = str(ordered[0]["candidate_id"])
        return selected_id, "strict_winner", {
            "status": "STRICT_WINNER_LOCKED_AT_10MS",
            "strict_winner_id": selected_id,
            "diagnostic_best_id": None,
            "strict_ranking": ranking,
            "ranked_strict_eligible_ids": [
                str(row["candidate_id"]) for row in ordered
            ],
            "candidate_v3_gates": assessed,
        }

    diagnostic_ranking = protocol["selection"]["diagnostic_best_ranking"]
    diagnostic_ordered = sorted(
        selectable, key=lambda row: _rank_key(row, diagnostic_ranking)
    )
    selected_id = str(diagnostic_ordered[0]["candidate_id"])
    return selected_id, "diagnostic_best", {
        "status": "NO_STRICT_WINNER_DIAGNOSTIC_BEST_LOCKED",
        "strict_winner_id": None,
        "diagnostic_best_id": selected_id,
        "diagnostic_ranking": diagnostic_ranking,
        "ranked_diagnostic_ids": [
            str(row["candidate_id"]) for row in diagnostic_ordered
        ],
        "candidate_v3_gates": assessed,
        "diagnostic_best_is_promotable": False,
    }


def build_sensitivity_assessment(
    selection_mode: str,
    selected_row: Mapping[str, Any],
    current_row: Mapping[str, Any],
    protocol: Mapping[str, Any],
) -> dict[str, Any]:
    if selection_mode not in {"strict_winner", "diagnostic_best"}:
        raise ProtocolError(f"unsupported selection mode: {selection_mode}")
    v3_gate = evaluate_holdout_gate(selected_row, current_row, protocol)
    physical_gate = physical_continuity_pre_gate(selected_row, protocol)
    return {
        "role": (
            "gating_strict_winner"
            if selection_mode == "strict_winner"
            else "diagnostic_only_cannot_convert_primary_fail"
        ),
        "physical_continuity_pre_gate": physical_gate,
        "v3_gate": v3_gate,
        "can_convert_10ms_fail_to_pass": False,
        "ok": bool(
            selection_mode == "strict_winner"
            and physical_gate["ok"]
            and v3_gate["ok"]
        ),
    }


def _write_rows_csv(path: Path, rows: Sequence[Mapping[str, Any]]) -> None:
    fieldnames = sorted({str(key) for row in rows for key in row})
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def _plot_development(
    path: Path,
    rows: Sequence[Mapping[str, Any]],
    selected_id: str,
) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    selectable = [row for row in rows if row.get("selectable")]
    positions = np.arange(len(selectable), dtype=float)
    labels = [str(row["candidate_id"]) for row in selectable]
    fig, axes = plt.subplots(3, 1, figsize=(10.5, 9.0), sharex=True)
    metrics = (
        ("event_count_deficit", "event-count deficit"),
        ("maximum_interior_both_off_gap_s", "max both-off gap [s]"),
        ("confirmed_fsm_stance_iou", "confirmed FSM stance IoU"),
    )
    for axis, (metric, label) in zip(axes, metrics):
        values = np.asarray([float(row[metric]) for row in selectable], dtype=float)
        axis.plot(positions, values, marker="o", color="#4C78A8")
        selected_index = next(
            index
            for index, row in enumerate(selectable)
            if row["candidate_id"] == selected_id
        )
        axis.scatter(
            [positions[selected_index]],
            [float(selectable[selected_index][metric])],
            color="#E45756",
            marker="*",
            s=140,
            zorder=4,
            label="10 ms selected pair",
        )
        axis.legend(loc="best")
        axis.set_ylabel(label)
        axis.grid(alpha=0.25)
    axes[-1].set_xticks(positions, labels, rotation=45, ha="right")
    axes[-1].set_xlabel("preregistered heel + forefoot pair")
    fig.suptitle("Two-sensor forefoot placement — prescribed development 50–100 s")
    fig.tight_layout()
    fig.savefig(path, dpi=170)
    plt.close(fig)


def run_sweep(
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

    base, candidates, geometry_summary = build_placement_candidates(protocol)
    primary_inputs, primary_access = _sample_all_candidates(
        protocol, base, candidates, sample_dt_s=PRIMARY_DT_S
    )
    primary_rows: list[dict[str, Any]] = []
    primary_details: dict[str, Any] = {}
    for candidate in candidates:
        row, detail = _evaluate_placement(
            protocol,
            candidate,
            primary_inputs[candidate.candidate_id],
            sample_dt_s=PRIMARY_DT_S,
        )
        primary_rows.append(row)
        primary_details[candidate.candidate_id] = detail

    selected_id, selection_mode, selection = select_development_outcome(
        primary_rows, protocol
    )
    selected = next(item for item in candidates if item.candidate_id == selected_id)
    current = next(
        item for item in candidates if item.candidate_id == CURRENT_PROFILE_ID
    )
    sensitivity_inputs, sensitivity_access = _sample_all_candidates(
        protocol, base, [selected, current], sample_dt_s=SENSITIVITY_DT_S
    )
    sensitivity_rows: dict[str, dict[str, Any]] = {}
    sensitivity_details: dict[str, dict[str, Any]] = {}
    for candidate in (selected, current):
        row, detail = _evaluate_placement(
            protocol,
            candidate,
            sensitivity_inputs[candidate.candidate_id],
            sample_dt_s=SENSITIVITY_DT_S,
        )
        sensitivity_rows[candidate.candidate_id] = row
        sensitivity_details[candidate.candidate_id] = detail
    sensitivity_gate = build_sensitivity_assessment(
        selection_mode,
        sensitivity_rows[selected_id],
        sensitivity_rows[CURRENT_PROFILE_ID],
        protocol,
    )

    rows_path = output_dir / "placement_primary_10ms_metrics.csv"
    _write_rows_csv(rows_path, primary_rows)
    plot_path = plot_dir / "placement_development_primary_10ms.png"
    _plot_development(plot_path, primary_rows, selected_id)
    sensitivity_path = output_dir / "selected_plus_current_sensitivity_1ms_metrics.csv"
    _write_rows_csv(
        sensitivity_path,
        [
            sensitivity_rows[selected_id],
            sensitivity_rows[CURRENT_PROFILE_ID],
        ],
    )

    ok = bool(
        selection_mode == "strict_winner" and sensitivity_gate["ok"]
    )
    if selection_mode == "strict_winner":
        conclusion = (
            "DEVELOPMENT_STRICT_WINNER_PASSES_10MS_AND_1MS"
            if ok
            else "STRICT_10MS_WINNER_FAILED_1MS_GATE"
        )
    else:
        conclusion = "NO_STRICT_WINNER_DIAGNOSTIC_BEST_ONLY"
    manifest = {
        "schema_version": 1,
        "status": "PASS" if ok else "FAIL",
        "ok": ok,
        "stage": "development",
        "objective": protocol["objective"],
        "protocol": {
            "path": _portable_path(Path(str(protocol["_protocol_path"]))),
            "sha256": protocol["_protocol_sha256"],
            "protocol_id": protocol["protocol_id"],
            "frozen_before_execution": True,
        },
        "data_access": {
            "already_open_block_s": [BLOCK_START_S, SEALED_START_S],
            "sealed_block_s": [SEALED_START_S, SEALED_END_S],
            "sealed_block_opened": False,
            "primary_10ms": primary_access,
            "selected_plus_current_sensitivity_1ms": sensitivity_access,
        },
        "detector_contract": {
            "sensors_per_candidate": 2,
            "sensor_roles": ["candidate_heel", "candidate_forefoot"],
            "sensor_on_threshold_n": SENSOR_ON_N,
            "sensor_off_threshold_n": SENSOR_OFF_N,
            "sensor_dwell_s": SENSOR_DWELL_S,
            "event_source": "two_sensor",
            "primary_event_time_field": "confirmed_time_s",
        },
        "geometry": geometry_summary,
        "candidates": [
            {
                "candidate_id": item.candidate_id,
                "selectable": item.selectable,
                "role": item.role,
                "heel_offset_below_current_mm": item.heel_offset_below_current_mm,
                "forefoot_fraction_mesh_x": item.forefoot_fraction_mesh_x,
                "forefoot_protrusion_mm": item.forefoot_protrusion_mm,
                "heel_location_m": list(item.heel_location),
                "forefoot_location_m": list(item.forefoot_location),
                "geometry": dict(item.geometry),
            }
            for item in candidates
        ],
        "primary_10ms": {
            "rows": primary_rows,
            "details": primary_details,
            "selection": selection,
        },
        "selected_pair": {
            "candidate_id": selected_id,
            "selection_mode": selection_mode,
            "promotable_from_10ms": selection_mode == "strict_winner",
        },
        "selected_plus_current_sensitivity_1ms": {
            "rows": sensitivity_rows,
            "details": sensitivity_details,
            "gate": sensitivity_gate,
            "evaluated_pair_count": 2,
        },
        "conclusion": conclusion,
        "artifacts": {
            "primary_metrics_csv": _source_record(rows_path),
            "primary_plot": _source_record(plot_path),
            "selected_plus_current_sensitivity_csv": _source_record(
                sensitivity_path
            ),
        },
        "non_actions": {
            "policy_or_training_run": False,
            "runtime_configuration_modified": False,
            "fsm_modified": False,
            "current_or_v4_profile_modified": False,
            "sealed_block_opened": False,
        },
        "interpretation_limits": protocol["interpretation_limits"],
    }
    safe_manifest = _json_safe(manifest)
    (output_dir / "manifest.json").write_text(
        json.dumps(safe_manifest, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    return safe_manifest


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run the preregistered AB06 forefoot-placement development sweep."
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
        manifest = run_sweep(protocol, output_dir, plot_dir)
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
        "selected_pair": manifest["selected_pair"],
        "sealed_block_opened": False,
        "manifest": _portable_path(output_dir / "manifest.json"),
    }
    print(json.dumps(concise, indent=2, allow_nan=False))
    return 0 if manifest["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
