"""Preregistered timing-placement sweep for a simple two-sensor detector.

Each candidate is exactly one heel sphere plus one forefoot sphere.  The
selectable grid varies only two small placement axes supported by the previous
development results: heel height H1/H2 and forefoot plantar depth P30..P32 at
the fixed F80 longitudinal station.  Heel-strike/toe-off guards always use the
two candidate spheres; FSM load/contact evidence always comes from the frozen
primary ``grf_correct_magnitude`` profile.

This harness is development-only on the already-open ``50 <= t < 100 s``
block.  It never reads the sealed block, trains a policy, edits runtime state,
or promotes a detector profile.
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

import diagnose_two_sensor_dual_stream_prescribed as dual  # noqa: E402
import sweep_two_sensor_mesh_placements_prescribed as v1  # noqa: E402
from sweep_two_sensor_prescribed_thresholds import (  # noqa: E402
    evaluate_holdout_gate,
)


DEFAULT_PROTOCOL = (
    VALIDATION_ROOT / "two_sensor_timing_placement_sweep_protocol_v3.json"
)
DEFAULT_OUTPUT_DIR = (
    VALIDATION_ROOT
    / "two_sensor_timing_placement_sweep_runs/"
    "2026-07-22_ab06_50_100_h1_h2_f80_depth_v3"
)
DEFAULT_PLOT_DIR = (
    REPO_ROOT / "plot/07_22_2026_two_sensor_timing_placement_v3"
)
PROTOCOL_ID = "AB06_TWO_SENSOR_TIMING_PLACEMENT_DEVELOPMENT_2026-07-22_V3"
HEEL_OFFSETS_MM = (1.0, 2.0)
FOREFOOT_FRACTION = 0.80
PROTRUSIONS_MM = (30.0, 30.5, 31.0, 31.5, 32.0)
CURRENT_COMPARATOR_ID = v1.CURRENT_PROFILE_ID
EXPECTED_SELECTABLE_COUNT = 10
EXPECTED_PAIR_COUNT = 11
NUMERIC_TOLERANCE = 1e-12


class ProtocolError(v1.ProtocolError):
    """Raised before sampling when the frozen V3 contract drifts."""


class NoClobberError(RuntimeError):
    """Raised before writes when a destination is already occupied."""


def _occupied(path: Path) -> bool:
    return path.exists() and (not path.is_dir() or any(path.iterdir()))


def _preflight_no_clobber(output_dir: Path, plot_dir: Path) -> None:
    occupied = [path for path in (output_dir, plot_dir) if _occupied(path)]
    if occupied:
        joined = ", ".join(v1._portable_path(path) for path in occupied)
        raise NoClobberError(f"refusing to modify occupied path(s): {joined}")


def _candidate_id(heel_offset_mm: float, protrusion_mm: float) -> str:
    heel = int(round(float(heel_offset_mm)))
    depth = f"{float(protrusion_mm):.1f}".replace(".", "p")
    return f"H{heel:02d}_F80_P{depth}"


def _require_float_list(value: Any, expected: Sequence[float], label: str) -> None:
    try:
        observed = [float(item) for item in value]
    except (TypeError, ValueError) as exc:
        raise ProtocolError(f"{label} must be a numeric list") from exc
    if observed != [float(item) for item in expected]:
        raise ProtocolError(f"{label} drifted: {observed}")


def load_and_validate_protocol(
    path: str | Path = DEFAULT_PROTOCOL,
) -> dict[str, Any]:
    protocol_path = v1.resolve_repo_path(path).resolve()
    try:
        raw = json.loads(protocol_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ProtocolError(f"cannot load protocol: {protocol_path}") from exc
    if not isinstance(raw, dict) or raw.get("schema_version") != 3:
        raise ProtocolError("unsupported timing-placement V3 schema")
    frozen = {
        "protocol_id": PROTOCOL_ID,
        "frozen_before_execution": True,
        "stage": "development_timing_placement",
        "setup": (
            "models/AB06_SEASEA_Threadmill/"
            "AB06_SEASEA_stiff321_500_pi_setup.xml"
        ),
        "detector_template_profile": (
            "online_grf_profiles/"
            "AB06_SEASEA_stiff321_500_pi_grf_detector_HS-TO.json"
        ),
        "load_evidence_profile": dual.LOAD_EVIDENCE_PROFILE,
    }
    for key, expected in frozen.items():
        if raw.get(key) != expected:
            raise ProtocolError(f"frozen protocol field drifted: {key}")
    if raw.get("profile_paths") != {
        CURRENT_COMPARATOR_ID: frozen["detector_template_profile"]
    }:
        raise ProtocolError("current comparator profile mapping drifted")

    access = raw.get("data_access")
    if not isinstance(access, dict):
        raise ProtocolError("data_access is required")
    expected_access = {
        "already_open_block_s": [v1.BLOCK_START_S, v1.SEALED_START_S],
        "upper_bound_is_exclusive": True,
        "allow_samples_at_or_after_100_s": False,
        "sealed_block_s": [v1.SEALED_START_S, v1.SEALED_END_S],
        "sealed_block_status": "CLOSED_UNEVALUATED",
    }
    for key, expected in expected_access.items():
        if access.get(key) != expected:
            raise ProtocolError(f"data-access field drifted: {key}")
    common = access.get("expected_common_cycle_set")
    expected_common = {
        "original_complete_cycles": 51,
        "retained_complete_cycles": 50,
        "reference_hs_count": 51,
        "reference_to_count": 50,
        "excluded_closing_hs_s": [99.96878691565038],
        "last_sample_s": 99.12,
    }
    if not isinstance(common, dict):
        raise ProtocolError("common-cycle lineage is required")
    for key, expected in expected_common.items():
        if common.get(key) != expected:
            raise ProtocolError(f"common-cycle field drifted: {key}")

    replay = raw.get("replay")
    expected_replay = {
        "sea_plugin": "plugins/SEA_Plugin_BlackBox_mCMC_impedence_ff",
        "event_source": "two_sensor",
        "sensor_on_threshold_n": v1.SENSOR_ON_N,
        "sensor_off_threshold_n": v1.SENSOR_OFF_N,
        "sensor_dwell_s": v1.SENSOR_DWELL_S,
        "sample_dt_s": v1.PRIMARY_DT_S,
        "primary_event_time_field": "confirmed_time_s",
        "diagnostic_event_time_field": "event_time_s",
        "phase_reference_mode": "validated_event_intervals",
        "prescribed_contact_threshold_n": 20.0,
        "reference_min_contact_duration_s": 0.05,
        "reference_min_cycle_duration_s": 0.3,
        "hs_tolerance_s": 0.05,
        "toe_off_tolerance_s": 0.08,
        "fsm_min_stance_duration_s": 0.30,
        "fsm_min_stance_contact_fraction": 0.20,
        "fsm_min_stance_load_bw_s": 0.04,
    }
    if not isinstance(replay, dict):
        raise ProtocolError("replay is required")
    for key, expected in expected_replay.items():
        observed = replay.get(key)
        if isinstance(expected, float):
            try:
                matches = math.isclose(
                    float(observed), expected, rel_tol=0.0, abs_tol=1e-12
                )
            except (TypeError, ValueError):
                matches = False
        else:
            matches = observed == expected
        if not matches:
            raise ProtocolError(f"frozen replay field drifted: {key}")
    if replay.get("threshold_geometry_or_fsm_tuning_allowed_during_run") is not False:
        raise ProtocolError("in-run tuning must remain forbidden")

    grid = raw.get("placement_grid")
    if not isinstance(grid, dict):
        raise ProtocolError("placement_grid is required")
    _require_float_list(
        grid.get("heel_vertical_offsets_below_current_mm"),
        HEEL_OFFSETS_MM,
        "heel offsets",
    )
    _require_float_list(
        grid.get("forefoot_longitudinal_fractions_mesh_x"),
        [FOREFOOT_FRACTION],
        "forefoot fraction",
    )
    _require_float_list(
        grid.get("forefoot_absolute_local_plantar_protrusion_mm"),
        PROTRUSIONS_MM,
        "forefoot protrusions",
    )
    expected_grid = {
        "heel_offset_rule": "local_vertical_below_current_center",
        "forefoot_x_rule": "mesh_bbox_longitudinal_fraction",
        "forefoot_z_rule": "midpoint_of_mesh_section_at_candidate_x",
        "forefoot_height_rule": (
            "absolute_sphere_bottom_below_local_plantar_surface"
        ),
        "selectable_candidate_count": EXPECTED_SELECTABLE_COUNT,
        "nonselectable_current_comparator_count": 1,
        "total_pair_count": EXPECTED_PAIR_COUNT,
        "sensors_per_pair": 2,
        "maximum_forefoot_protrusion_mm": 32.0,
    }
    for key, expected in expected_grid.items():
        if grid.get(key) != expected:
            raise ProtocolError(f"placement-grid field drifted: {key}")

    routing = raw.get("evidence_routing")
    if routing != {
        "heel_toe_event_guards": "candidate_two_sensor_forces_only",
        "normal_force_bw": "primary_online_grf_left_aggregate",
        "in_contact": "primary_online_grf_left_union_physical_penetration",
        "detector_spheres_generate_grf": False,
    }:
        raise ProtocolError("detector/load evidence separation drifted")

    sampling = raw.get("sampling")
    if sampling != {
        "method": "all_required_spheres_directly_in_one_opensim_pass",
        "affine_reconstruction": False,
        "expected_unique_detector_spheres": 9,
        "expected_primary_load_spheres": 8,
        "expected_total_unique_spheres": 17,
        "evaluated_pair_count": EXPECTED_PAIR_COUNT,
    }:
        raise ProtocolError("single-pass sampling contract drifted")

    gate = raw.get("development_gate")
    required_gate = {
        "maximum_transfer_both_latches_off_samples": 0,
        "maximum_incomplete_heel_to_forefoot_transfers": 0,
        "maximum_to_candidates_before_min_stance": 0,
        "require_exact_reference_counts": {"heel_strike": 51, "toe_off": 50},
        "require_exact_detector_counts": {"heel_strike": 51, "toe_off": 50},
        "require_exact_valid_cycles": 50,
        "precision": 1.0,
        "recall": 1.0,
        "max_abs_hs_error_s": 0.05,
        "max_abs_toe_off_error_s": 0.08,
        "minimum_confirmed_fsm_stance_f1": 0.95,
        "minimum_confirmed_fsm_stance_iou": 0.90,
        "maximum_confirmed_fsm_iou_regression_vs_current": 0.01,
        "maximum_confirmed_time_worst_timing_regression_vs_current": 0.0,
        "maximum_invalid_or_timeout_transitions": 0,
        "maximum_unaccepted_sensor_events": 0,
        "require_forefoot_present_every_stance": True,
        "require_forefoot_off_observed_every_swing": True,
        "require_forefoot_off_before_next_hs": True,
        "require_mesh_proximity_pre_gate": True,
    }
    if gate != required_gate:
        raise ProtocolError("development gate drifted")
    # Imported V3 gate helper reads this historical key name.
    sealed_gate = raw.get("sealed_validation_gate")
    expected_sealed_gate = {
        "precision": 1.0,
        "recall": 1.0,
        "max_abs_hs_error_s": 0.05,
        "max_abs_toe_off_error_s": 0.08,
        "minimum_confirmed_fsm_stance_f1": 0.95,
        "minimum_confirmed_fsm_stance_iou": 0.90,
        "maximum_confirmed_fsm_iou_regression_vs_baseline": 0.01,
        "maximum_confirmed_time_worst_timing_regression_vs_baseline": 0.0,
    }
    if sealed_gate != expected_sealed_gate:
        raise ProtocolError("imported strict-gate payload drifted")

    decision = raw.get("decision_contract")
    if decision != {
        "role": "development_geometry_screen_only",
        "candidate_selection_allowed": True,
        "profile_creation_allowed": False,
        "profile_promotion_allowed": False,
        "sealed_validation_allowed": False,
        "training_allowed": False,
        "runtime_modification_allowed": False,
    }:
        raise ProtocolError("development-only decision contract drifted")

    sources = raw.get("sources")
    if not isinstance(sources, dict) or not sources:
        raise ProtocolError("hash-pinned sources are required")
    for label, record in sources.items():
        if not isinstance(record, dict):
            raise ProtocolError(f"invalid source record: {label}")
        source_path = v1.resolve_repo_path(str(record.get("path", ""))).resolve()
        if not source_path.is_file():
            raise ProtocolError(f"missing pinned source {label}: {source_path}")
        observed = v1._sha256(source_path)
        if observed != record.get("sha256"):
            raise ProtocolError(f"source hash drift for {label}: {observed}")

    raw["_protocol_path"] = protocol_path.as_posix()
    raw["_protocol_sha256"] = v1._sha256(protocol_path)
    raw["_primary_event_time_field"] = "confirmed_time_s"
    raw["_diagnostic_event_time_field"] = "event_time_s"
    raw["_phase_reference_mode"] = "validated_event_intervals"
    return raw


def _mesh_geometry(
    location: Sequence[float],
    radius_m: float,
    triangles: np.ndarray,
) -> dict[str, Any]:
    distance = float(v1._minimum_mesh_distance(location, triangles))
    return {
        "center_to_mesh_m": distance,
        "sphere_surface_gap_to_mesh_m": max(0.0, distance - radius_m),
        "sphere_mesh_overlap_depth_m": max(0.0, radius_m - distance),
        "within_5mm_of_mesh": bool(distance <= radius_m + 0.005 + 1e-12),
    }


def build_placement_candidates(
    protocol: Mapping[str, Any],
) -> tuple[v1.OnlineGRFProfile, list[v1.PlacementCandidate], dict[str, Any]]:
    """Build ten simple two-sphere pairs plus the current comparator."""
    base_path = v1.resolve_repo_path(
        str(protocol["detector_template_profile"])
    ).resolve()
    base = v1.load_online_grf_profile(base_path)
    sensors = v1._left_sensor_spheres(base)
    heel_template = sensors["left_heel"]
    toe_template = sensors["left_toe"]
    if not math.isclose(
        heel_template.radius, toe_template.radius, rel_tol=0.0, abs_tol=1e-12
    ):
        raise ProtocolError("heel/toe radii must remain equal")

    setup = v1.read_setup_xml(
        v1.resolve_repo_path(str(protocol["setup"])).resolve()
    )
    mesh_path = v1._resolve_left_foot_mesh(setup.model_file.resolve())
    triangles = v1._load_stl_triangles(mesh_path)
    radius = float(heel_template.radius)
    forefeet: dict[float, tuple[tuple[float, float, float], dict[str, Any]]] = {}
    for protrusion_mm in PROTRUSIONS_MM:
        forefeet[protrusion_mm] = v1._derive_forefoot_location(
            triangles,
            toe_template,
            fraction=FOREFOOT_FRACTION,
            protrusion_mm=protrusion_mm,
        )

    candidates: list[v1.PlacementCandidate] = []
    for heel_offset_mm in HEEL_OFFSETS_MM:
        heel_location = (
            float(heel_template.location[0]),
            float(heel_template.location[1] - heel_offset_mm / 1000.0),
            float(heel_template.location[2]),
        )
        heel_geometry = _mesh_geometry(heel_location, radius, triangles)
        for protrusion_mm in PROTRUSIONS_MM:
            forefoot_location, derived = forefeet[protrusion_mm]
            forefoot_geometry = _mesh_geometry(
                forefoot_location, radius, triangles
            )
            bottom_offset_mm = 1000.0 * (
                float(forefoot_location[1] - radius)
                - float(heel_location[1] - radius)
            )
            checks = {
                "heel_within_5mm_of_mesh": heel_geometry[
                    "within_5mm_of_mesh"
                ],
                "forefoot_within_5mm_of_mesh": forefoot_geometry[
                    "within_5mm_of_mesh"
                ],
                "heel_forefoot_bottoms_within_20mm": bool(
                    abs(bottom_offset_mm) <= 20.0 + 1e-9
                ),
                "forefoot_depth_at_or_below_32mm": bool(
                    protrusion_mm <= 32.0 + 1e-12
                ),
                "exactly_two_spheres": True,
            }
            geometry = {
                **derived,
                "heel_location_m": list(heel_location),
                "forefoot_location_m": list(forefoot_location),
                "heel_offset_below_current_mm": heel_offset_mm,
                "heel": heel_geometry,
                "forefoot": forefoot_geometry,
                "signed_forefoot_minus_heel_bottom_offset_mm": bottom_offset_mm,
                "pre_gate_checks": checks,
                "pre_gate_ok": bool(all(checks.values())),
                "detector_representation": "two_spheres_only",
            }
            candidates.append(
                v1.PlacementCandidate(
                    candidate_id=_candidate_id(heel_offset_mm, protrusion_mm),
                    heel_location=heel_location,
                    forefoot_location=forefoot_location,
                    heel_offset_below_current_mm=heel_offset_mm,
                    forefoot_fraction_mesh_x=FOREFOOT_FRACTION,
                    forefoot_protrusion_mm=protrusion_mm,
                    selectable=True,
                    role="development_timing_placement_candidate",
                    geometry=geometry,
                )
            )

    current_locations = (
        tuple(heel_template.location),
        tuple(toe_template.location),
    )
    current_checks = {
        "heel": _mesh_geometry(current_locations[0], radius, triangles),
        "forefoot": _mesh_geometry(current_locations[1], radius, triangles),
    }
    candidates.append(
        v1.PlacementCandidate(
            candidate_id=CURRENT_COMPARATOR_ID,
            heel_location=current_locations[0],
            forefoot_location=current_locations[1],
            heel_offset_below_current_mm=None,
            forefoot_fraction_mesh_x=None,
            forefoot_protrusion_mm=None,
            selectable=False,
            role="nonselectable_current_geometry_comparator",
            geometry={
                "source": "current_detector_profile",
                "detector_representation": "two_spheres_only",
                "mesh_proximity_diagnostic": current_checks,
            },
        )
    )
    expected_ids = {
        _candidate_id(heel, depth)
        for heel in HEEL_OFFSETS_MM
        for depth in PROTRUSIONS_MM
    } | {CURRENT_COMPARATOR_ID}
    observed_ids = {candidate.candidate_id for candidate in candidates}
    if observed_ids != expected_ids or len(candidates) != EXPECTED_PAIR_COUNT:
        raise ProtocolError("candidate grid does not match the frozen 10+1 set")
    if not all(
        bool(candidate.geometry.get("pre_gate_ok"))
        for candidate in candidates
        if candidate.selectable
    ):
        raise ProtocolError("a selectable placement failed the mesh pre-gate")
    return base, candidates, {
        "mesh": v1._source_record(mesh_path),
        "selectable_candidate_count": EXPECTED_SELECTABLE_COUNT,
        "total_pair_count": EXPECTED_PAIR_COUNT,
        "sensors_per_pair": 2,
        "unique_detector_sphere_count": 9,
        "affine_reconstruction_used": False,
        "detector_representation": "two_spheres_per_pair",
    }


def _maximum_true_run_samples(mask: np.ndarray) -> int:
    longest = 0
    current = 0
    for value in np.asarray(mask, dtype=bool):
        if value:
            current += 1
            longest = max(longest, current)
        else:
            current = 0
    return int(longest)


def heel_to_forefoot_transfer_diagnostics(
    times: np.ndarray,
    heel_contact: np.ndarray,
    toe_contact: np.ndarray,
    reference_events: Mapping[str, np.ndarray],
) -> dict[str, Any]:
    """Count stable both-off samples only during heel-to-forefoot transfer."""
    sample_times = np.asarray(times, dtype=float)
    heel = np.asarray(heel_contact, dtype=float) > 0.5
    toe = np.asarray(toe_contact, dtype=float) > 0.5
    if not (sample_times.shape == heel.shape == toe.shape):
        raise ValueError("transfer streams must have identical shapes")
    reference_hs = np.asarray(reference_events["heel_strike"], dtype=float)
    reference_to = np.asarray(reference_events["toe_off"], dtype=float)
    cycles: list[dict[str, Any]] = []
    for cycle_index, (hs_s, to_s) in enumerate(zip(reference_hs[:-1], reference_to)):
        indices = np.flatnonzero((sample_times >= hs_s) & (sample_times < to_s))
        if indices.size == 0:
            raise ProtocolError(f"empty reference stance at cycle {cycle_index}")
        local_heel = heel[indices]
        local_toe = toe[indices]
        heel_on = np.flatnonzero(local_heel)
        toe_on = np.flatnonzero(local_toe)
        heel_falls = np.flatnonzero(
            np.concatenate(([False], local_heel[:-1])) & ~local_heel
        )
        complete = bool(heel_on.size and toe_on.size and heel_falls.size)
        both_off_samples = 0
        longest_both_off_run = 0
        heel_off_s: float | None = None
        toe_on_s: float | None = None
        if complete:
            # Ignore a boundary fall before the first observed heel contact.
            valid_falls = heel_falls[heel_falls > heel_on[0]]
            complete = bool(valid_falls.size)
            if complete:
                heel_off_local = int(valid_falls[0])
                first_toe_local = int(toe_on[0])
                heel_off_s = float(sample_times[indices[heel_off_local]])
                toe_on_s = float(sample_times[indices[first_toe_local]])
                if first_toe_local > heel_off_local:
                    transfer = slice(heel_off_local, first_toe_local)
                    both_off = ~(local_heel[transfer] | local_toe[transfer])
                    both_off_samples = int(np.count_nonzero(both_off))
                    longest_both_off_run = _maximum_true_run_samples(both_off)
        cycles.append(
            {
                "cycle_index": cycle_index,
                "reference_hs_s": float(hs_s),
                "reference_to_s": float(to_s),
                "heel_contact_observed": bool(heel_on.size),
                "forefoot_contact_observed": bool(toe_on.size),
                "complete_heel_to_forefoot_transfer": complete,
                "heel_off_s": heel_off_s,
                "forefoot_on_s": toe_on_s,
                "both_latches_off_sample_count": both_off_samples,
                "longest_both_latches_off_run_samples": longest_both_off_run,
            }
        )
    return {
        "cycle_count": len(cycles),
        "incomplete_transfer_cycle_count": int(
            sum(not row["complete_heel_to_forefoot_transfer"] for row in cycles)
        ),
        "both_latches_off_sample_count": int(
            sum(row["both_latches_off_sample_count"] for row in cycles)
        ),
        "maximum_both_latches_off_run_samples": int(
            max(
                (row["longest_both_latches_off_run_samples"] for row in cycles),
                default=0,
            )
        ),
        "cycles": cycles,
    }


def early_to_candidate_diagnostics(
    candidates: Sequence[Mapping[str, Any]],
    *,
    minimum_stance_duration_s: float,
) -> dict[str, Any]:
    records: list[dict[str, Any]] = []
    missing_duration: list[dict[str, Any]] = []
    for item in candidates:
        if str(item.get("event", "")) != "toe_off":
            continue
        duration = item.get("contact_duration_s")
        record = {
            "event_time_s": float(item["time"]),
            "observed_at_s": float(item.get("observed_at_s", item["time"])),
            "contact_duration_s": None if duration is None else float(duration),
        }
        if duration is None:
            missing_duration.append(record)
        elif float(duration) + NUMERIC_TOLERANCE < minimum_stance_duration_s:
            records.append(record)
    return {
        "minimum_stance_duration_s": float(minimum_stance_duration_s),
        "toe_off_candidate_count": int(
            sum(str(item.get("event", "")) == "toe_off" for item in candidates)
        ),
        "early_toe_off_candidate_count": len(records),
        "toe_off_candidates_missing_duration_count": len(missing_duration),
        "early_toe_off_candidates": records,
        "toe_off_candidates_missing_duration": missing_duration,
    }


def evaluate_placement(
    protocol: Mapping[str, Any],
    candidate: v1.PlacementCandidate,
    common: Mapping[str, Any],
) -> tuple[dict[str, Any], dict[str, Any]]:
    """Evaluate two detector guards with primary FSM load/contact evidence."""
    inputs, sources = dual.compose_branch_inputs(common, "B_primary_load")
    row, detail = v1._evaluate_placement(
        protocol,
        candidate,
        inputs,
        sample_dt_s=v1.PRIMARY_DT_S,
    )
    runtime_cfg = replace(
        v1._current_runtime_fsm_config(),
        sensor_on_threshold_n=v1.SENSOR_ON_N,
        sensor_off_threshold_n=v1.SENSOR_OFF_N,
        sensor_dwell_s=v1.SENSOR_DWELL_S,
    )
    if not math.isclose(
        float(runtime_cfg.min_stance_duration_s),
        float(protocol["replay"]["fsm_min_stance_duration_s"]),
        rel_tol=0.0,
        abs_tol=1e-12,
    ):
        raise ProtocolError("production FSM min-stance duration drifted")
    replay = v1._run_production_fsm(
        np.asarray(inputs["times"], dtype=float),
        dict(inputs["loads"]),
        dict(inputs["penetrations"]),
        np.asarray(inputs["aggregate"], dtype=float),
        dict(inputs["kinematics"]),
        body_weight_n=float(inputs["body_weight_n"]),
        fsm_config=runtime_cfg,
    )
    transfer = heel_to_forefoot_transfer_diagnostics(
        np.asarray(inputs["times"], dtype=float),
        np.asarray(replay["heel_contact"], dtype=float),
        np.asarray(replay["toe_contact"], dtype=float),
        inputs["reference_events"],
    )
    early_to = early_to_candidate_diagnostics(
        replay["candidates"],
        minimum_stance_duration_s=float(runtime_cfg.min_stance_duration_s),
    )
    row.update(
        {
            "heel_offset_below_current_mm": candidate.heel_offset_below_current_mm,
            "forefoot_fraction_mesh_x": candidate.forefoot_fraction_mesh_x,
            "forefoot_protrusion_mm": candidate.forefoot_protrusion_mm,
            "normal_force_bw_source": sources["normal_force_bw"],
            "in_contact_source": sources["in_contact"],
            "heel_toe_event_guard_source": sources[
                "heel_toe_event_guards"
            ],
            "mesh_geometry_pre_gate_ok": bool(
                candidate.geometry.get("pre_gate_ok", not candidate.selectable)
            ),
            "transfer_both_latches_off_sample_count": transfer[
                "both_latches_off_sample_count"
            ],
            "maximum_transfer_both_latches_off_run_samples": transfer[
                "maximum_both_latches_off_run_samples"
            ],
            "incomplete_heel_to_forefoot_transfer_count": transfer[
                "incomplete_transfer_cycle_count"
            ],
            "to_candidates_before_min_stance_count": early_to[
                "early_toe_off_candidate_count"
            ],
            "to_candidates_missing_stance_duration_count": early_to[
                "toe_off_candidates_missing_duration_count"
            ],
        }
    )
    detail.update(
        {
            "stream_sources": sources,
            "heel_to_forefoot_transfer": transfer,
            "early_toe_off_candidates": early_to,
            "final_fsm_payload": replay["fsm"].payload(),
        }
    )
    return row, detail


def strict_development_gate(
    row: Mapping[str, Any],
    current_row: Mapping[str, Any],
    protocol: Mapping[str, Any],
) -> dict[str, Any]:
    imported = evaluate_holdout_gate(row, current_row, protocol)
    gate = protocol["development_gate"]
    checks = {
        "imported_event_timing_phase_gate": bool(imported["ok"]),
        "mesh_geometry_pre_gate": bool(row["mesh_geometry_pre_gate_ok"]),
        "primary_load_evidence": row["normal_force_bw_source"]
        == "primary_online_grf_left_aggregate",
        "primary_contact_evidence": row["in_contact_source"]
        == "primary_online_grf_left_union_physical_penetration",
        "two_sensor_event_guards": row["heel_toe_event_guard_source"]
        == "candidate_two_sensor_forces",
        "zero_transfer_both_off_samples": int(
            row["transfer_both_latches_off_sample_count"]
        )
        <= int(gate["maximum_transfer_both_latches_off_samples"]),
        "all_transfers_complete": int(
            row["incomplete_heel_to_forefoot_transfer_count"]
        )
        <= int(gate["maximum_incomplete_heel_to_forefoot_transfers"]),
        "zero_early_to_candidates": int(
            row["to_candidates_before_min_stance_count"]
        )
        <= int(gate["maximum_to_candidates_before_min_stance"]),
        "all_to_candidates_have_duration": int(
            row["to_candidates_missing_stance_duration_count"]
        )
        == 0,
        "exact_51_reference_hs": int(row["reference_hs_count"]) == 51,
        "exact_50_reference_to": int(row["reference_to_count"]) == 50,
        "exact_51_detector_hs": int(row["predicted_hs_count"]) == 51,
        "exact_50_detector_to": int(row["predicted_to_count"]) == 50,
        "exact_50_valid_cycles": int(row["observed_valid_cycle_count"]) == 50,
        "forefoot_present_every_stance": int(
            row["forefoot_missing_stance_count"]
        )
        == 0,
        "forefoot_releases_every_swing": int(
            row["swings_without_forefoot_off_count"]
        )
        == 0,
        "forefoot_off_before_next_hs": int(
            row["forefoot_contact_crossing_next_hs_count"]
        )
        == 0,
    }
    return {
        "ok": bool(all(checks.values())),
        "checks": checks,
        "imported_confirmed_time_gate": imported,
    }


def select_development_candidate(
    rows: Sequence[Mapping[str, Any]],
    protocol: Mapping[str, Any],
) -> tuple[str | None, dict[str, Any]]:
    current = [row for row in rows if row["candidate_id"] == CURRENT_COMPARATOR_ID]
    if len(current) != 1:
        raise ProtocolError("exactly one current comparator row is required")
    selectable = [row for row in rows if bool(row.get("selectable"))]
    if len(selectable) != EXPECTED_SELECTABLE_COUNT:
        raise ProtocolError("selection requires all ten frozen candidates")
    assessments = {
        str(row["candidate_id"]): strict_development_gate(
            row, current[0], protocol
        )
        for row in selectable
    }
    eligible = [
        row
        for row in selectable
        if assessments[str(row["candidate_id"])]["ok"]
    ]
    eligible.sort(
        key=lambda row: (
            float(row["worst_event_normalized_max_abs_error"]),
            float(row["mean_event_normalized_mean_abs_error"]),
            -float(row["confirmed_fsm_stance_iou"]),
            float(row["geometry_displacement_from_current_m"]),
            str(row["candidate_id"]),
        )
    )
    selected_id = str(eligible[0]["candidate_id"]) if eligible else None
    return selected_id, {
        "status": (
            "STRICT_DEVELOPMENT_CANDIDATE_IDENTIFIED"
            if selected_id is not None
            else "NO_STRICT_DEVELOPMENT_CANDIDATE"
        ),
        "selected_candidate_id": selected_id,
        "candidate_gates": assessments,
        "selection_ranking": [
            "worst_event_normalized_max_abs_error:min",
            "mean_event_normalized_mean_abs_error:min",
            "confirmed_fsm_stance_iou:max",
            "geometry_displacement_from_current_m:min",
            "candidate_id:lexicographic",
        ],
        "automatic_promotion_allowed": False,
    }


def _write_rows_csv(path: Path, rows: Sequence[Mapping[str, Any]]) -> None:
    fields = sorted({str(key) for row in rows for key in row})
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def _plot_grid(
    path: Path,
    rows: Sequence[Mapping[str, Any]],
    selected_id: str | None,
) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    selectable = [row for row in rows if bool(row.get("selectable"))]
    fig, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
    colors = {1.0: "#4C78A8", 2.0: "#F58518"}
    metrics = (
        ("observed_valid_cycle_count", "valid cycles", 50.0),
        ("max_abs_hs_error_s", "max |HS error| [s]", 0.05),
        ("max_abs_toe_off_error_s", "max |TO error| [s]", 0.08),
    )
    for heel_offset in HEEL_OFFSETS_MM:
        group = sorted(
            (
                row
                for row in selectable
                if math.isclose(
                    float(row["heel_offset_below_current_mm"]),
                    heel_offset,
                    rel_tol=0.0,
                    abs_tol=1e-12,
                )
            ),
            key=lambda row: float(row["forefoot_protrusion_mm"]),
        )
        x = [float(row["forefoot_protrusion_mm"]) for row in group]
        for axis, (metric, ylabel, threshold) in zip(axes, metrics):
            axis.plot(
                x,
                [float(row[metric]) for row in group],
                marker="o",
                color=colors[heel_offset],
                label=f"heel H{int(heel_offset)}",
            )
            axis.axhline(threshold, color="#E45756", linestyle="--", alpha=0.8)
            axis.set_ylabel(ylabel)
            axis.grid(alpha=0.25)
    if selected_id is not None:
        selected = next(row for row in selectable if row["candidate_id"] == selected_id)
        for axis, (metric, _ylabel, _threshold) in zip(axes, metrics):
            axis.scatter(
                [float(selected["forefoot_protrusion_mm"])],
                [float(selected[metric])],
                marker="*",
                color="#54A24B",
                edgecolor="black",
                s=170,
                zorder=5,
            )
    axes[0].legend(loc="best")
    axes[-1].set_xlabel("forefoot protrusion below plantar surface [mm]")
    fig.suptitle("Two-sensor timing placement V3 — AB06 prescribed 50–100 s")
    fig.tight_layout()
    fig.savefig(path, dpi=170)
    plt.close(fig)


def run_sweep(
    protocol: Mapping[str, Any],
    output_dir: Path,
    plot_dir: Path,
) -> dict[str, Any]:
    _preflight_no_clobber(output_dir, plot_dir)
    base, candidates, geometry = build_placement_candidates(protocol)
    common_inputs, access = dual.sample_all_streams_once(
        protocol, base, candidates
    )
    rows: list[dict[str, Any]] = []
    details: dict[str, Any] = {}
    for candidate in candidates:
        row, detail = evaluate_placement(
            protocol, candidate, common_inputs[candidate.candidate_id]
        )
        rows.append(row)
        details[candidate.candidate_id] = detail
    selected_id, selection = select_development_candidate(rows, protocol)

    # Create artifacts only after all fail-closed evaluation has completed.
    output_dir.mkdir(parents=True, exist_ok=False)
    plot_dir.mkdir(parents=True, exist_ok=False)
    csv_path = output_dir / "timing_placement_primary_10ms_metrics.csv"
    plot_path = plot_dir / "timing_placement_primary_10ms.png"
    _write_rows_csv(csv_path, rows)
    _plot_grid(plot_path, rows, selected_id)

    manifest = {
        "schema_version": 3,
        "status": "PASS" if selected_id is not None else "FAIL",
        "ok": selected_id is not None,
        "stage": "development_timing_placement",
        "objective": protocol["objective"],
        "protocol": {
            "path": v1._portable_path(Path(str(protocol["_protocol_path"]))),
            "sha256": protocol["_protocol_sha256"],
            "protocol_id": protocol["protocol_id"],
            "frozen_before_execution": True,
        },
        "data_access": {
            "already_open_block_s": [v1.BLOCK_START_S, v1.SEALED_START_S],
            "sealed_block_s": [v1.SEALED_START_S, v1.SEALED_END_S],
            "sealed_block_opened": False,
            "primary_10ms_sampling": access,
        },
        "detector_contract": {
            "sensors_per_pair": 2,
            "sensor_roles": ["heel", "forefoot"],
            "selectable_pair_count": EXPECTED_SELECTABLE_COUNT,
            "total_pair_count": EXPECTED_PAIR_COUNT,
            "event_guard_source": "candidate_two_sensor_forces_only",
            "normal_force_bw_source": "primary_online_grf_left_aggregate",
            "in_contact_source": (
                "primary_online_grf_left_union_physical_penetration"
            ),
            "detector_spheres_generate_grf": False,
            "fsm_thresholds_and_dwell_unchanged": True,
        },
        "geometry": geometry,
        "candidates": [
            {
                "candidate_id": candidate.candidate_id,
                "selectable": candidate.selectable,
                "role": candidate.role,
                "heel_location_m": list(candidate.heel_location),
                "forefoot_location_m": list(candidate.forefoot_location),
                "heel_offset_below_current_mm": (
                    candidate.heel_offset_below_current_mm
                ),
                "forefoot_fraction_mesh_x": candidate.forefoot_fraction_mesh_x,
                "forefoot_protrusion_mm": candidate.forefoot_protrusion_mm,
                "sensor_count": 2,
                "geometry": dict(candidate.geometry),
            }
            for candidate in candidates
        ],
        "primary_10ms": {"rows": rows, "details": details},
        "selection": selection,
        "selected_pair": {
            "candidate_id": selected_id,
            "development_screen_passed": selected_id is not None,
            "profile_created": False,
            "promotable": False,
            "requires_future_1ms_and_holdout_validation": True,
        },
        "artifacts": {
            "metrics_csv": v1._source_record(csv_path),
            "plot": v1._source_record(plot_path),
        },
        "non_actions": {
            "policy_or_training_run": False,
            "runtime_configuration_modified": False,
            "production_fsm_modified": False,
            "current_profile_modified": False,
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
        description="Run preregistered two-sensor timing-placement sweep V3."
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
    except NoClobberError as exc:
        print(
            json.dumps(
                {
                    "schema_version": 3,
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
    try:
        protocol = load_and_validate_protocol(args.protocol)
        manifest = run_sweep(protocol, output_dir, plot_dir)
    except NoClobberError as exc:
        print(
            json.dumps(
                {
                    "schema_version": 3,
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
    except Exception as exc:  # pragma: no cover - fail-closed CLI path
        failure = {
            "schema_version": 3,
            "status": "ERROR",
            "ok": False,
            "sealed_block_opened": False,
            "error": f"{type(exc).__name__}: {exc}",
            "traceback": traceback.format_exc(),
        }
        print(json.dumps(failure, indent=2))
        return 2
    print(
        json.dumps(
            {
                "status": manifest["status"],
                "selected_pair": manifest["selected_pair"],
                "sealed_block_opened": False,
                "manifest": v1._portable_path(output_dir / "manifest.json"),
            },
            indent=2,
            allow_nan=False,
        )
    )
    return 0 if manifest["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
