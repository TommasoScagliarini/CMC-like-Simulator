"""V10 isolated center-versus-radius prescribed sweep for heel and toe.

V9 is an immutable experimental comparator.  V10 changes one heel-sphere
or forefoot-sphere geometric quantity at a time while keeping the other sensor,
thresholds, dwell, FSM, primary-load evidence, policy, and reward fixed:

* ``heel_center_only`` raises the local heel center with radius fixed;
* ``heel_radius_only`` reduces heel radius with center fixed;
* ``toe_center_only`` raises the local forefoot center with radius fixed;
* ``toe_radius_only`` reduces forefoot radius with center fixed.

All candidates remain exactly one heel sphere plus one forefoot sphere.  The
entire already-open AB06 interval from the first complete HS at 13.946870984 s
through the last causally confirmable cycle below 100 s is evaluated directly
at 10 ms and 1 ms.  ``confirmed_time_s`` is authoritative for timing gates;
``event_time_s`` is retained as onset diagnostics.  The sealed block is never
sampled and this script cannot create or promote a detector profile.  The best
strict heel component and best strict toe component are then combined and
replayed at both resolutions using the already sampled station trajectories.
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

import sweep_two_sensor_timing_placements_prescribed_v9 as v9  # noqa: E402


v6 = v9.v8.v6
v4 = v6.v4
v1 = v6.v1
dual = v6.dual

DEFAULT_PROTOCOL = VALIDATION_ROOT / "two_sensor_center_radius_sweep_protocol_v10.json"
DEFAULT_OUTPUT_DIR = (
    VALIDATION_ROOT
    / "two_sensor_center_radius_sweep_runs/"
    "2026-07-22_ab06_open_13p946_100_v10"
)
DEFAULT_PLOT_DIR = REPO_ROOT / "plot/07_22_2026_two_sensor_center_radius_v10"
GEOMETRY_REFERENCE_PROFILE = (
    "online_grf_profiles/AB06_SEASEA_stiff321_500_pi_grf_detector_HS-TO.json"
)

PROTOCOL_ID = "AB06_TWO_SENSOR_CENTER_RADIUS_DEVELOPMENT_2026-07-22_V10"
SCHEMA_VERSION = 10
OPEN_START_S = 13.946870983805102
OPEN_END_S = 100.0
PRIMARY_DT_S = 0.01
FINE_DT_S = 0.001
DELTAS_MM = (
    0.150,
    0.210,
    0.215,
    0.220,
    0.225,
    0.250,
    0.275,
    0.325,
    0.340,
    0.341,
    0.500,
    1.000,
    2.000,
    4.000,
)
PARAMETER_ARMS = (
    "heel_center_only",
    "heel_radius_only",
    "toe_center_only",
    "toe_radius_only",
)
BASELINE_ID = "v9_baseline"
SELECTABLE_COUNT = len(PARAMETER_ARMS) * len(DELTAS_MM)
PAIR_COUNT = SELECTABLE_COUNT + 1
EXPECTED_UNIQUE_DETECTOR_SPHERES = 2 * (len(DELTAS_MM) + 1)
EXPECTED_PRIMARY_SPHERES = 8
EXPECTED_TOTAL_SPHERES = (
    EXPECTED_UNIQUE_DETECTOR_SPHERES + EXPECTED_PRIMARY_SPHERES
)
EXPECTED_REFERENCE_HS = 75
EXPECTED_REFERENCE_TO = 74
EXPECTED_CYCLES = 74
NUMERIC_TOLERANCE = 1.0e-12


class ProtocolError(v1.ProtocolError):
    """Raised before sampling if the frozen V10 contract drifts."""


class NoClobberError(RuntimeError):
    """Raised before writes when an output destination is occupied."""


def _occupied(path: Path) -> bool:
    return path.exists() and (not path.is_dir() or any(path.iterdir()))


def _preflight_no_clobber(output_dir: Path, plot_dir: Path) -> None:
    occupied = [path for path in (output_dir, plot_dir) if _occupied(path)]
    if occupied:
        joined = ", ".join(v1._portable_path(path) for path in occupied)
        raise NoClobberError(f"refusing to modify occupied path(s): {joined}")


def _numeric_list(value: Any, expected: Sequence[float], label: str) -> None:
    try:
        observed = [float(item) for item in value]
    except (TypeError, ValueError) as exc:
        raise ProtocolError(f"{label} must be a numeric list") from exc
    if observed != [float(item) for item in expected]:
        raise ProtocolError(f"{label} drifted: {observed}")


def _gate_contract() -> dict[str, Any]:
    return {
        "require_exact_reference_counts": {
            "heel_strike": EXPECTED_REFERENCE_HS,
            "toe_off": EXPECTED_REFERENCE_TO,
        },
        "require_exact_detector_counts": {
            "heel_strike": EXPECTED_REFERENCE_HS,
            "toe_off": EXPECTED_REFERENCE_TO,
        },
        "require_exact_valid_cycles": EXPECTED_CYCLES,
        "precision": 1.0,
        "recall": 1.0,
        "max_abs_hs_error_s": 0.05,
        "max_abs_toe_off_error_s": 0.08,
        "minimum_confirmed_fsm_stance_f1": 0.95,
        "minimum_confirmed_fsm_stance_iou": 0.90,
        "maximum_transfer_both_latches_off_samples": 0,
        "maximum_incomplete_heel_to_forefoot_transfers": 0,
        "maximum_to_candidates_before_min_stance": 0,
        "maximum_invalid_or_timeout_transitions": 0,
        "maximum_unaccepted_sensor_events": 0,
        "maximum_forbidden_phase_mismatches": 0,
        "maximum_unknown_fsm_phase_samples": 0,
        "minimum_causal_toe_clear_before_next_hs_onset_s": 0.03,
        "require_exact_causal_swing_intervals": EXPECTED_CYCLES,
        "require_confirmation_latency_in_range": True,
        "require_exact_hs_to_toe_off_order_and_cycles": True,
        "require_mesh_proximity_pre_gate": True,
    }


def load_and_validate_protocol(
    path: str | Path = DEFAULT_PROTOCOL,
) -> dict[str, Any]:
    protocol_path = v1.resolve_repo_path(path).resolve()
    try:
        raw = json.loads(protocol_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ProtocolError(f"cannot load V10 protocol: {protocol_path}") from exc
    if not isinstance(raw, dict):
        raise ProtocolError("V10 protocol root must be an object")

    expected_top = {
        "schema_version": SCHEMA_VERSION,
        "protocol_id": PROTOCOL_ID,
        "frozen_before_execution": True,
        "stage": "development_open_span_isolated_heel_toe_center_radius",
        "setup": (
            "models/AB06_SEASEA_Threadmill/"
            "AB06_SEASEA_stiff321_500_pi_setup.xml"
        ),
        "detector_template_profile": (
            "validation/experimental_detector_profiles/"
            "two_sensor_v9_H2p50_X3p25_F79p0_P35p00.json"
        ),
        "geometry_reference_profile": GEOMETRY_REFERENCE_PROFILE,
        "load_evidence_profile": dual.LOAD_EVIDENCE_PROFILE,
    }
    for key, expected in expected_top.items():
        if raw.get(key) != expected:
            raise ProtocolError(f"V10 frozen field drifted: {key}")

    access = raw.get("data_access", {})
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
    if access != expected_access:
        raise ProtocolError("V10 data-access contract drifted")

    replay = raw.get("replay", {})
    expected_replay = {
        "sea_plugin": "plugins/SEA_Plugin_BlackBox_mCMC_impedence_ff",
        "event_source": "two_sensor",
        "sensor_on_threshold_n": 0.5,
        "sensor_off_threshold_n": 0.25,
        "sensor_dwell_s": 0.03,
        "runtime_sample_dt_s": PRIMARY_DT_S,
        "fine_sample_dt_s": FINE_DT_S,
        "evaluate_all_pairs_at_both_resolutions": True,
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
        "threshold_fsm_routing_or_forefoot_changed": False,
    }
    if replay != expected_replay:
        raise ProtocolError("V10 replay contract drifted")

    grid = raw.get("isolated_parameter_grid", {})
    _numeric_list(grid.get("delta_mm"), DELTAS_MM, "center/radius delta")
    for key, expected in {
        "parameter_arms": list(PARAMETER_ARMS),
        "baseline_comparator_count": 1,
        "selectable_candidate_count": SELECTABLE_COUNT,
        "total_pair_count": PAIR_COUNT,
        "sensors_per_pair": 2,
        "opposite_sensor_geometry": "exact_v9_fixed",
        "center_arm_radius": "exact_v9_fixed",
        "radius_arm_center": "exact_v9_fixed",
        "maximum_effective_bottom_raise_mm": 4.0,
        "matched_effective_bottom_pairs": True,
    }.items():
        if grid.get(key) != expected:
            raise ProtocolError(f"V10 isolated grid drifted: {key}")

    expected_sample = {
        "method": "shared_station_sampling_radius_evaluated_offline",
        "expected_unique_detector_stations": EXPECTED_UNIQUE_DETECTOR_SPHERES,
        "expected_primary_load_spheres": EXPECTED_PRIMARY_SPHERES,
        "expected_total_sampled_stations": EXPECTED_TOTAL_SPHERES,
        "evaluated_pair_count": PAIR_COUNT,
    }
    if raw.get("sampling") != {
        "runtime_10ms": expected_sample,
        "fine_1ms": expected_sample,
        "radius_does_not_change_station_kinematics": True,
        "affine_reconstruction": False,
    }:
        raise ProtocolError("V10 sampling contract drifted")

    for key in ("runtime_gate_10ms", "fine_gate_1ms"):
        if raw.get(key) != _gate_contract():
            raise ProtocolError(f"V10 {key} drifted")

    if raw.get("selection") != {
        "component_eligibility": "full_strict_pass_at_both_10ms_and_1ms",
        "heel_ranking": [
            "multiresolution_event_count_deficit:min",
            "multiresolution_invalid_timeout_unaccepted:min",
            "multiresolution_transfer_gap_samples:min",
            "multiresolution_two_window_max_abs_hs_error:min",
            "multiresolution_global_max_abs_hs_error:min",
            "multiresolution_global_mean_abs_hs_error:min",
            "multiresolution_global_max_abs_toe_off_error:min",
            "effective_bottom_raise_mm:min",
            "candidate_id:lexicographic",
        ],
        "toe_ranking": [
            "multiresolution_event_count_deficit:min",
            "multiresolution_invalid_timeout_unaccepted:min",
            "multiresolution_transfer_gap_samples:min",
            "multiresolution_two_window_max_abs_toe_off_error:min",
            "multiresolution_global_max_abs_toe_off_error:min",
            "multiresolution_global_mean_abs_toe_off_error:min",
            "multiresolution_global_max_abs_hs_error:min",
            "effective_bottom_raise_mm:min",
            "candidate_id:lexicographic",
        ],
        "combined_candidate_must_pass_both_resolutions": True,
        "diagnostic_best_when_no_strict_winner": True,
        "v9_baseline_allowed_as_constraint_preserving_diagnostic_fallback": True,
    }:
        raise ProtocolError("V10 selection contract drifted")

    if raw.get("decision_contract") != {
        "candidate_selection_allowed": True,
        "profile_creation_allowed": False,
        "profile_promotion_allowed": False,
        "sealed_validation_allowed": False,
        "training_allowed": False,
        "runtime_modification_allowed": False,
        "v9_files_must_remain_immutable": True,
    }:
        raise ProtocolError("V10 decision contract drifted")

    sources = raw.get("sources")
    if not isinstance(sources, dict) or not sources:
        raise ProtocolError("V10 hash-pinned sources are required")
    for label, record in sources.items():
        source_path = v1.resolve_repo_path(str(record.get("path", ""))).resolve()
        if not source_path.is_file():
            raise ProtocolError(f"missing V10 source: {label}")
        if v1._sha256(source_path) != record.get("sha256"):
            raise ProtocolError(f"V10 source hash drift: {label}")

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
    return f"{float(value):.3f}".rstrip("0").rstrip(".").replace(".", "p")


def build_candidates(
    protocol: Mapping[str, Any],
) -> tuple[v1.OnlineGRFProfile, list[v1.PlacementCandidate], dict[str, Any]]:
    base_path = v1.resolve_repo_path(
        str(protocol["detector_template_profile"])
    ).resolve()
    base = v1.load_online_grf_profile(base_path, required_sides=("left",))
    sensors = v1._left_sensor_spheres(base)
    heel = sensors["left_heel"]
    toe = sensors["left_toe"]
    setup = v1.read_setup_xml(v1.resolve_repo_path(str(protocol["setup"])).resolve())
    mesh_path = v1._resolve_left_foot_mesh(setup.model_file.resolve())
    triangles = v1._load_stl_triangles(mesh_path)
    heel_base_radius = float(heel.radius)
    toe_base_radius = float(toe.radius)
    heel_base_location = tuple(float(value) for value in heel.location)
    toe_base_location = tuple(float(value) for value in toe.location)

    candidates: list[v1.PlacementCandidate] = []

    def make_candidate(
        candidate_id: str,
        *,
        arm: str,
        heel_center_raise_mm: float = 0.0,
        heel_radius_reduction_mm: float = 0.0,
        toe_center_raise_mm: float = 0.0,
        toe_radius_reduction_mm: float = 0.0,
        selectable: bool,
    ) -> v1.PlacementCandidate:
        changes = (
            heel_center_raise_mm,
            heel_radius_reduction_mm,
            toe_center_raise_mm,
            toe_radius_reduction_mm,
        )
        changed_count = sum(float(value) > 0.0 for value in changes)
        expected_changed_count = 1 if selectable else 0
        if changed_count != expected_changed_count:
            raise ProtocolError("V10 candidate must change exactly one parameter")
        heel_location = (
            heel_base_location[0],
            heel_base_location[1] + heel_center_raise_mm / 1000.0,
            heel_base_location[2],
        )
        toe_location = (
            toe_base_location[0],
            toe_base_location[1] + toe_center_raise_mm / 1000.0,
            toe_base_location[2],
        )
        heel_radius = heel_base_radius - heel_radius_reduction_mm / 1000.0
        toe_radius = toe_base_radius - toe_radius_reduction_mm / 1000.0
        if heel_radius <= 0.0 or toe_radius <= 0.0:
            raise ProtocolError("V10 produced a non-positive radius")
        heel_mesh = v9.v8.v6.v3._mesh_geometry(
            heel_location, heel_radius, triangles
        )
        toe_mesh = v9.v8.v6.v3._mesh_geometry(toe_location, toe_radius, triangles)
        center_raise_mm = heel_center_raise_mm + toe_center_raise_mm
        radius_reduction_mm = (
            heel_radius_reduction_mm + toe_radius_reduction_mm
        )
        effective_raise = center_raise_mm + radius_reduction_mm
        target_sensor = (
            "heel" if arm.startswith("heel_") else
            "toe" if arm.startswith("toe_") else
            "baseline"
        )
        bottom_offset_m = (
            toe_location[1] - toe_radius
        ) - (heel_location[1] - heel_radius)
        checks = {
            "exactly_two_spheres": True,
            "only_one_sensor_parameter_changed": changed_count
            == expected_changed_count,
            "heel_within_5mm_of_mesh": bool(heel_mesh["within_5mm_of_mesh"]),
            "toe_within_5mm_of_mesh": bool(toe_mesh["within_5mm_of_mesh"]),
            "positive_radii": heel_radius > 0.0 and toe_radius > 0.0,
            "effective_bottom_raise_within_4mm": effective_raise <= 4.0,
            "heel_to_toe_bottom_offset_within_20mm": abs(bottom_offset_m)
            <= 0.020 + NUMERIC_TOLERANCE,
        }
        return v1.PlacementCandidate(
            candidate_id=candidate_id,
            heel_location=heel_location,
            forefoot_location=toe_location,
            heel_offset_below_current_mm=None,
            forefoot_fraction_mesh_x=None,
            forefoot_protrusion_mm=None,
            selectable=selectable,
            role=arm,
            geometry={
                "source": "v9_experimental_baseline",
                "parameter_arm": arm,
                "target_sensor": target_sensor,
                "center_raise_mm": float(center_raise_mm),
                "radius_reduction_mm": float(radius_reduction_mm),
                "heel_center_raise_mm": float(heel_center_raise_mm),
                "heel_radius_reduction_mm": float(heel_radius_reduction_mm),
                "toe_center_raise_mm": float(toe_center_raise_mm),
                "toe_radius_reduction_mm": float(toe_radius_reduction_mm),
                "heel_radius_m": float(heel_radius),
                "toe_radius_m": float(toe_radius),
                "v9_heel_radius_m": float(heel_base_radius),
                "v9_toe_radius_m": float(toe_base_radius),
                "effective_bottom_raise_mm": float(effective_raise),
                "heel_to_toe_bottom_offset_m": float(bottom_offset_m),
                "heel": heel_mesh,
                "toe": toe_mesh,
                "pre_gate_checks": checks,
                "pre_gate_ok": bool(all(checks.values())),
                "detector_representation": "two_spheres_only",
            },
        )

    candidates.append(
        make_candidate(
            BASELINE_ID,
            arm="v9_baseline_comparator",
            selectable=False,
        )
    )
    for delta in DELTAS_MM:
        token = _token(delta)
        candidates.extend(
            (
                make_candidate(
                    f"heel_center_up_{token}mm",
                    arm="heel_center_only",
                    heel_center_raise_mm=delta,
                    selectable=True,
                ),
                make_candidate(
                    f"heel_radius_down_{token}mm",
                    arm="heel_radius_only",
                    heel_radius_reduction_mm=delta,
                    selectable=True,
                ),
                make_candidate(
                    f"toe_center_up_{token}mm",
                    arm="toe_center_only",
                    toe_center_raise_mm=delta,
                    selectable=True,
                ),
                make_candidate(
                    f"toe_radius_down_{token}mm",
                    arm="toe_radius_only",
                    toe_radius_reduction_mm=delta,
                    selectable=True,
                ),
            )
        )

    if len(candidates) != PAIR_COUNT:
        raise ProtocolError("V10 candidate count drifted")
    if len({item.candidate_id for item in candidates}) != PAIR_COUNT:
        raise ProtocolError("V10 candidate IDs are not unique")
    selectable = [item for item in candidates if item.selectable]
    if len(selectable) != SELECTABLE_COUNT:
        raise ProtocolError("V10 selectable count drifted")
    if not all(bool(item.geometry["pre_gate_ok"]) for item in candidates):
        raise ProtocolError("a V10 candidate failed its geometry pre-gate")
    if any(
        sum(
            float(item.geometry[key]) > 0.0
            for key in (
                "heel_center_raise_mm",
                "heel_radius_reduction_mm",
                "toe_center_raise_mm",
                "toe_radius_reduction_mm",
            )
        ) != (1 if item.selectable else 0)
        for item in candidates
    ):
        raise ProtocolError("V10 parameter separation was violated")

    return base, candidates, {
        "mesh": v1._source_record(mesh_path),
        "v9_profile": v1._source_record(base_path),
        "v9_heel_location_m": list(heel_base_location),
        "v9_forefoot_location_m": list(toe_base_location),
        "v9_heel_radius_m": heel_base_radius,
        "v9_forefoot_radius_m": toe_base_radius,
        "selectable_candidate_count": SELECTABLE_COUNT,
        "total_pair_count": PAIR_COUNT,
        "sensors_per_pair": 2,
        "parameter_arms": list(PARAMETER_ARMS),
        "opposite_sensor_fixed_exactly_to_v9": True,
        "no_candidate_changes_more_than_one_sensor_parameter": True,
    }


def _sampling_bundle(
    base: v1.OnlineGRFProfile,
    candidates: Sequence[v1.PlacementCandidate],
) -> tuple[
    v1.OnlineGRFProfile,
    dict[str, dict[str, v1.OnlineGRFSphere]],
    dict[str, v1.OnlineGRFProfile],
]:
    sensors = v1._left_sensor_spheres(base)
    heel_template = sensors["left_heel"]
    toe_template = sensors["left_toe"]
    sampled_by_location: dict[
        tuple[str, tuple[float, float, float]], v1.OnlineGRFSphere
    ] = {}
    sampled_spheres: list[v1.OnlineGRFSphere] = []

    def sampled_station(
        template: v1.OnlineGRFSphere,
        location: tuple[float, float, float],
        role: str,
    ) -> v1.OnlineGRFSphere:
        key = (role, tuple(float(value) for value in location))
        if key not in sampled_by_location:
            sampled = replace(
                template,
                name=f"v10_station_{role}_{len(sampled_by_location):02d}",
                location=key[1],
            )
            sampled_by_location[key] = sampled
            sampled_spheres.append(sampled)
        return sampled_by_location[key]

    pairs: dict[str, dict[str, v1.OnlineGRFSphere]] = {}
    profiles: dict[str, v1.OnlineGRFProfile] = {}
    for candidate in candidates:
        heel_station = sampled_station(
            heel_template, tuple(candidate.heel_location), "heel"
        )
        toe_station = sampled_station(
            toe_template, tuple(candidate.forefoot_location), "toe"
        )
        heel_variant = replace(
            heel_station,
            radius=float(candidate.geometry["heel_radius_m"]),
        )
        toe_variant = replace(
            toe_station,
            radius=float(candidate.geometry["toe_radius_m"]),
        )
        pairs[candidate.candidate_id] = {
            "heel": heel_variant,
            "toe": toe_variant,
        }
        profiles[candidate.candidate_id] = replace(
            base,
            source=f"validation_v10_{candidate.candidate_id}",
            spheres=(heel_variant, toe_variant),
        )
    sampler = replace(
        base,
        source="validation_v10_shared_station_sampler",
        spheres=tuple(sampled_spheres),
    )
    if len(sampler.spheres) != EXPECTED_UNIQUE_DETECTOR_SPHERES:
        raise ProtocolError("V10 unique detector-station count drifted")
    if any(len(profile.spheres) != 2 for profile in profiles.values()):
        raise ProtocolError("V10 profile is not exactly heel + forefoot")
    return sampler, pairs, profiles


def _reference_bundle(
    protocol: Mapping[str, Any], *, sample_dt_s: float
) -> tuple[Any, dict[str, np.ndarray], dict[str, Any], np.ndarray]:
    if sample_dt_s not in {PRIMARY_DT_S, FINE_DT_S}:
        raise ProtocolError("V10 permits only 10 ms and 1 ms")
    setup = replace(
        v1.read_setup_xml(v1.resolve_repo_path(str(protocol["setup"])).resolve()),
        t_start=OPEN_START_S,
        t_end=float(np.nextafter(OPEN_END_S, -np.inf)),
    )
    replay = protocol["replay"]
    raw_events, provenance = v1._reference_events_from_prescribed_grf(
        setup,
        threshold_n=float(replay["prescribed_contact_threshold_n"]),
        min_contact_duration_s=float(replay["reference_min_contact_duration_s"]),
        min_cycle_duration_s=float(replay["reference_min_cycle_duration_s"]),
    )
    events, boundary = v1._exclude_unconfirmable_right_boundary_cycles(
        raw_events, sample_dt_s=sample_dt_s
    )
    if len(events["heel_strike"]) != EXPECTED_REFERENCE_HS:
        raise ProtocolError("V10 reference HS count drifted")
    if len(events["toe_off"]) != EXPECTED_REFERENCE_TO:
        raise ProtocolError("V10 reference TO count drifted")
    if boundary["excluded_closing_hs_s"] != [99.96878691565038]:
        raise ProtocolError("V10 right-boundary exclusion drifted")

    effective_end_s = float(
        events["heel_strike"][-1] + float(replay["sensor_dwell_s"]) + PRIMARY_DT_S
    )
    intervals = int(math.floor((effective_end_s - OPEN_START_S) / sample_dt_s))
    times = OPEN_START_S + np.arange(intervals + 1, dtype=float) * sample_dt_s
    if times.size < 2 or np.any(times >= OPEN_END_S):
        raise ProtocolError("V10 time grid touched sealed data")
    setup = replace(setup, t_end=float(times[-1]))
    return setup, events, {
        "sample_dt_s": float(sample_dt_s),
        "first_sample_s": float(times[0]),
        "last_sample_s": float(times[-1]),
        "sample_count": int(times.size),
        "samples_at_or_after_100_s": int(np.count_nonzero(times >= OPEN_END_S)),
        "sealed_block_opened": False,
        "reference_provenance": provenance,
        "right_boundary_audit": boundary,
    }, times


def sample_streams_once(
    protocol: Mapping[str, Any],
    base: v1.OnlineGRFProfile,
    candidates: Sequence[v1.PlacementCandidate],
    *,
    sample_dt_s: float,
) -> tuple[dict[str, dict[str, Any]], dict[str, Any]]:
    setup, events, access, times = _reference_bundle(
        protocol, sample_dt_s=sample_dt_s
    )
    detector_sampler, sphere_pairs, detector_profiles = _sampling_bundle(
        base, candidates
    )
    primary_path = v1.resolve_repo_path(
        str(protocol["load_evidence_profile"])
    ).resolve()
    primary_full = dual.load_online_grf_profile(primary_path)
    primary_spheres = tuple(
        sphere for sphere in primary_full.spheres if sphere.side == "left"
    )
    if len(primary_spheres) != EXPECTED_PRIMARY_SPHERES:
        raise ProtocolError("V10 primary sphere count drifted")
    primary_left = replace(
        primary_full,
        source="validation_v10_primary_left_load_evidence",
        spheres=primary_spheres,
    )
    combined = tuple(detector_sampler.spheres) + primary_spheres
    if len(combined) != EXPECTED_TOTAL_SPHERES:
        raise ProtocolError("V10 combined sampled-station count drifted")
    if len({sphere.name for sphere in combined}) != len(combined):
        raise ProtocolError("V10 sampled station names are not unique")
    sampler = replace(
        base,
        source="validation_v10_detector_primary_sampler",
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
            "single_opensim_station_sampling_pass": True,
            "radius_variants_reuse_identical_station_kinematics": True,
            "sampled_unique_detector_stations": len(detector_sampler.spheres),
            "sampled_primary_load_spheres": len(primary_spheres),
            "sampled_total_stations": len(combined),
            "evaluated_pair_count": len(candidates),
            "load_evidence_profile": v1._source_record(primary_path),
        }
    )
    return streams, access


def _window_event_metrics(
    detail: Mapping[str, Any], *, start_s: float, end_s: float
) -> dict[str, float | int | None]:
    events = detail["events"]
    result: dict[str, float | int | None] = {}
    for event in ("heel_strike", "toe_off"):
        reference = np.asarray(events["reference"][event], dtype=float)
        confirmed = np.asarray(events["confirmed"][event], dtype=float)
        onset = np.asarray(events["onset"][event], dtype=float)
        if not (reference.size == confirmed.size == onset.size):
            result[f"{event}_count"] = 0
            result[f"{event}_confirmed_max_abs_error_s"] = None
            result[f"{event}_confirmed_mean_abs_error_s"] = None
            result[f"{event}_confirmed_mean_signed_error_s"] = None
            result[f"{event}_confirmed_max_signed_error_s"] = None
            result[f"{event}_confirmed_min_signed_error_s"] = None
            result[f"{event}_onset_max_abs_error_s"] = None
            continue
        mask = (reference >= start_s - 1.0e-9) & (reference <= end_s + 1.0e-9)
        confirmed_errors = confirmed[mask] - reference[mask]
        onset_errors = onset[mask] - reference[mask]
        result[f"{event}_count"] = int(np.count_nonzero(mask))
        result[f"{event}_confirmed_max_abs_error_s"] = (
            float(np.max(np.abs(confirmed_errors)))
            if confirmed_errors.size
            else None
        )
        result[f"{event}_confirmed_mean_abs_error_s"] = (
            float(np.mean(np.abs(confirmed_errors)))
            if confirmed_errors.size
            else None
        )
        result[f"{event}_confirmed_mean_signed_error_s"] = (
            float(np.mean(confirmed_errors)) if confirmed_errors.size else None
        )
        result[f"{event}_confirmed_max_signed_error_s"] = (
            float(np.max(confirmed_errors)) if confirmed_errors.size else None
        )
        result[f"{event}_confirmed_min_signed_error_s"] = (
            float(np.min(confirmed_errors)) if confirmed_errors.size else None
        )
        result[f"{event}_onset_max_abs_error_s"] = (
            float(np.max(np.abs(onset_errors))) if onset_errors.size else None
        )
    return result


def evaluate_candidate(
    protocol: Mapping[str, Any],
    candidate: v1.PlacementCandidate,
    common: Mapping[str, Any],
    *,
    sample_dt_s: float,
) -> tuple[dict[str, Any], dict[str, Any]]:
    row, detail = v6.evaluate_placement(
        protocol, candidate, common, sample_dt_s=sample_dt_s
    )
    global_window = _window_event_metrics(
        detail, start_s=OPEN_START_S, end_s=OPEN_END_S
    )
    early = _window_event_metrics(
        detail, start_s=OPEN_START_S, end_s=18.946870983805102
    )
    long_window = _window_event_metrics(detail, start_s=50.0, end_s=OPEN_END_S)
    row.update(
        {
            "parameter_arm": candidate.geometry["parameter_arm"],
            "center_raise_mm": candidate.geometry["center_raise_mm"],
            "radius_reduction_mm": candidate.geometry["radius_reduction_mm"],
            "heel_radius_m": candidate.geometry["heel_radius_m"],
            "toe_radius_m": candidate.geometry["toe_radius_m"],
            "target_sensor": candidate.geometry["target_sensor"],
            "heel_center_raise_mm": candidate.geometry["heel_center_raise_mm"],
            "heel_radius_reduction_mm": candidate.geometry[
                "heel_radius_reduction_mm"
            ],
            "toe_center_raise_mm": candidate.geometry["toe_center_raise_mm"],
            "toe_radius_reduction_mm": candidate.geometry[
                "toe_radius_reduction_mm"
            ],
            "effective_bottom_raise_mm": candidate.geometry[
                "effective_bottom_raise_mm"
            ],
            "geometry_displacement_from_current_m": (
                float(candidate.geometry["center_raise_mm"])
                + float(candidate.geometry["radius_reduction_mm"])
            )
            / 1000.0,
            "mesh_geometry_pre_gate_ok": candidate.geometry["pre_gate_ok"],
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
            "early_max_abs_hs_error_s": early[
                "heel_strike_confirmed_max_abs_error_s"
            ],
            "early_mean_signed_hs_error_s": early[
                "heel_strike_confirmed_mean_signed_error_s"
            ],
            "early_max_signed_hs_error_s": early[
                "heel_strike_confirmed_max_signed_error_s"
            ],
            "early_onset_max_abs_hs_error_s": early[
                "heel_strike_onset_max_abs_error_s"
            ],
            "early_max_abs_toe_off_error_s": early[
                "toe_off_confirmed_max_abs_error_s"
            ],
            "early_mean_signed_toe_off_error_s": early[
                "toe_off_confirmed_mean_signed_error_s"
            ],
            "early_max_signed_toe_off_error_s": early[
                "toe_off_confirmed_max_signed_error_s"
            ],
            "early_onset_max_abs_toe_off_error_s": early[
                "toe_off_onset_max_abs_error_s"
            ],
            "long_max_abs_hs_error_s": long_window[
                "heel_strike_confirmed_max_abs_error_s"
            ],
            "long_mean_signed_hs_error_s": long_window[
                "heel_strike_confirmed_mean_signed_error_s"
            ],
            "long_max_signed_hs_error_s": long_window[
                "heel_strike_confirmed_max_signed_error_s"
            ],
            "long_onset_max_abs_hs_error_s": long_window[
                "heel_strike_onset_max_abs_error_s"
            ],
            "long_max_abs_toe_off_error_s": long_window[
                "toe_off_confirmed_max_abs_error_s"
            ],
            "long_mean_signed_toe_off_error_s": long_window[
                "toe_off_confirmed_mean_signed_error_s"
            ],
            "long_max_signed_toe_off_error_s": long_window[
                "toe_off_confirmed_max_signed_error_s"
            ],
            "long_onset_max_abs_toe_off_error_s": long_window[
                "toe_off_onset_max_abs_error_s"
            ],
        }
    )
    detail["v10_window_diagnostics"] = {
        "global_13p946_100": global_window,
        "early_13p946_18p946": early,
        "long_50_100": long_window,
    }
    return row, detail


def strict_gate(row: Mapping[str, Any], gate: Mapping[str, Any]) -> dict[str, Any]:
    reference_counts = gate["require_exact_reference_counts"]
    detector_counts = gate["require_exact_detector_counts"]
    checks = {
        "exact_reference_counts": bool(
            int(row["reference_hs_count"]) == int(reference_counts["heel_strike"])
            and int(row["reference_to_count"]) == int(reference_counts["toe_off"])
        ),
        "exact_detector_counts": bool(
            int(row["predicted_hs_count"]) == int(detector_counts["heel_strike"])
            and int(row["predicted_to_count"]) == int(detector_counts["toe_off"])
        ),
        "exact_valid_cycles": int(row["observed_valid_cycle_count"])
        == int(gate["require_exact_valid_cycles"]),
        "precision": float(row["precision"]) >= float(gate["precision"]),
        "recall": float(row["recall"]) >= float(gate["recall"]),
        "hs_timing": float(row["max_abs_hs_error_s"])
        <= float(gate["max_abs_hs_error_s"]) + NUMERIC_TOLERANCE,
        "toe_off_timing": float(row["max_abs_toe_off_error_s"])
        <= float(gate["max_abs_toe_off_error_s"]) + NUMERIC_TOLERANCE,
        "fsm_f1": float(row["confirmed_fsm_stance_f1"])
        >= float(gate["minimum_confirmed_fsm_stance_f1"]),
        "fsm_iou": float(row["confirmed_fsm_stance_iou"])
        >= float(gate["minimum_confirmed_fsm_stance_iou"]),
        "zero_transfer_gap_samples": int(
            row["transfer_both_latches_off_sample_count"]
        )
        <= int(gate["maximum_transfer_both_latches_off_samples"]),
        "zero_incomplete_transfers": int(
            row["incomplete_heel_to_forefoot_transfer_count"]
        )
        <= int(gate["maximum_incomplete_heel_to_forefoot_transfers"]),
        "zero_early_to_candidates": int(row["to_candidates_before_min_stance_count"])
        <= int(gate["maximum_to_candidates_before_min_stance"]),
        "zero_invalid_timeout": int(row["invalid_or_timeout_transition_count"])
        <= int(gate["maximum_invalid_or_timeout_transitions"]),
        "zero_unaccepted": int(row["unaccepted_sensor_gait_event_count"])
        <= int(gate["maximum_unaccepted_sensor_events"]),
        "zero_forbidden_phase_mismatch": int(row["forbidden_phase_mismatch_count"])
        <= int(gate["maximum_forbidden_phase_mismatches"]),
        "zero_unknown_phase": int(row["unknown_fsm_phase_samples"])
        <= int(gate["maximum_unknown_fsm_phase_samples"]),
        "causal_clear": float(
            row["minimum_causal_toe_clear_before_next_hs_onset_s"]
        )
        >= float(gate["minimum_causal_toe_clear_before_next_hs_onset_s"])
        - NUMERIC_TOLERANCE,
        "exact_causal_intervals": int(row["causal_swing_interval_count"])
        == int(gate["require_exact_causal_swing_intervals"]),
        "confirmation_latency": bool(row["confirmation_latency_in_range"])
        is bool(gate["require_confirmation_latency_in_range"]),
        "exact_order_cycles": bool(
            row["exact_hs_to_toe_off_to_hs_order_and_cycle_count"]
        )
        is bool(gate["require_exact_hs_to_toe_off_order_and_cycles"]),
        "mesh_pre_gate": bool(row["mesh_geometry_pre_gate_ok"])
        is bool(gate["require_mesh_proximity_pre_gate"]),
    }
    return {"ok": bool(all(checks.values())), "checks": checks}


def select_components(
    rows_10: Sequence[Mapping[str, Any]],
    rows_1: Sequence[Mapping[str, Any]],
    protocol: Mapping[str, Any],
) -> tuple[str, str, dict[str, Any]]:
    runtime = {str(row["candidate_id"]): row for row in rows_10}
    fine = {str(row["candidate_id"]): row for row in rows_1}
    if set(runtime) != set(fine) or len(runtime) != PAIR_COUNT:
        raise ProtocolError("V10 multiresolution candidate sets differ")
    selectable_ids = sorted(
        candidate_id for candidate_id, row in runtime.items()
        if bool(row.get("selectable"))
    )
    gates: dict[str, Any] = {}
    for candidate_id in selectable_ids:
        gate_10 = strict_gate(runtime[candidate_id], protocol["runtime_gate_10ms"])
        gate_1 = strict_gate(fine[candidate_id], protocol["fine_gate_1ms"])
        gates[candidate_id] = {
            "runtime_10ms": gate_10,
            "fine_1ms": gate_1,
            "passes_both": bool(gate_10["ok"] and gate_1["ok"]),
        }

    def finite_rank_metric(row: Mapping[str, Any], key: str) -> float:
        value = row.get(key)
        if value is None:
            return float("inf")
        try:
            number = float(value)
        except (TypeError, ValueError):
            return float("inf")
        return number if math.isfinite(number) else float("inf")

    def rank_key(candidate_id: str, target_sensor: str) -> tuple[Any, ...]:
        row_10 = runtime[candidate_id]
        row_1 = fine[candidate_id]
        structural = (
            max(
                int(row_10["event_count_deficit"]),
                int(row_1["event_count_deficit"]),
            ),
            max(
                int(row_10["invalid_timeout_plus_unaccepted_count"]),
                int(row_1["invalid_timeout_plus_unaccepted_count"]),
            ),
            max(
                int(row_10["transfer_both_latches_off_sample_count"]),
                int(row_1["transfer_both_latches_off_sample_count"]),
            ),
        )
        if target_sensor == "heel":
            return (*structural,
                max(
                    finite_rank_metric(row_10, "early_max_abs_hs_error_s"),
                    finite_rank_metric(row_1, "early_max_abs_hs_error_s"),
                    finite_rank_metric(row_10, "long_max_abs_hs_error_s"),
                    finite_rank_metric(row_1, "long_max_abs_hs_error_s"),
                ),
                max(
                    finite_rank_metric(row_10, "max_abs_hs_error_s"),
                    finite_rank_metric(row_1, "max_abs_hs_error_s"),
                ),
                max(
                    finite_rank_metric(row_10, "global_mean_abs_hs_error_s"),
                    finite_rank_metric(row_1, "global_mean_abs_hs_error_s"),
                ),
                max(
                    finite_rank_metric(row_10, "max_abs_toe_off_error_s"),
                    finite_rank_metric(row_1, "max_abs_toe_off_error_s"),
                ),
                finite_rank_metric(row_1, "effective_bottom_raise_mm"),
                candidate_id,
            )
        return (*structural,
            max(
                finite_rank_metric(row_10, "early_max_abs_toe_off_error_s"),
                finite_rank_metric(row_1, "early_max_abs_toe_off_error_s"),
                finite_rank_metric(row_10, "long_max_abs_toe_off_error_s"),
                finite_rank_metric(row_1, "long_max_abs_toe_off_error_s"),
            ),
            max(
                finite_rank_metric(row_10, "max_abs_toe_off_error_s"),
                finite_rank_metric(row_1, "max_abs_toe_off_error_s"),
            ),
            max(
                finite_rank_metric(row_10, "global_mean_abs_toe_off_error_s"),
                finite_rank_metric(row_1, "global_mean_abs_toe_off_error_s"),
            ),
            max(
                finite_rank_metric(row_10, "max_abs_hs_error_s"),
                finite_rank_metric(row_1, "max_abs_hs_error_s"),
            ),
            finite_rank_metric(row_1, "effective_bottom_raise_mm"),
            candidate_id,
        )

    components: dict[str, Any] = {}
    chosen: dict[str, str] = {}
    for target_sensor in ("heel", "toe"):
        target_ids = [
            candidate_id for candidate_id in selectable_ids
            if runtime[candidate_id]["target_sensor"] == target_sensor
        ]
        if len(target_ids) != 2 * len(DELTAS_MM):
            raise ProtocolError(f"V10 {target_sensor} component count drifted")
        eligible = [
            candidate_id for candidate_id in target_ids
            if gates[candidate_id]["passes_both"]
        ]
        eligible.sort(key=lambda item: rank_key(item, target_sensor))
        diagnostic_ranked = sorted(
            [BASELINE_ID, *target_ids],
            key=lambda item: rank_key(item, target_sensor),
        )
        strict_winner = eligible[0] if eligible else None
        diagnostic_best = diagnostic_ranked[0]
        chosen_id = strict_winner or diagnostic_best
        chosen[target_sensor] = chosen_id
        components[target_sensor] = {
            "status": (
                "STRICT_MULTIRESOLUTION_COMPONENT"
                if strict_winner is not None
                else "NO_STRICT_COMPONENT_DIAGNOSTIC_BEST_USED_FOR_COMBINATION"
            ),
            "strict_winner_id": strict_winner,
            "diagnostic_best_id": diagnostic_best,
            "chosen_for_combination_id": chosen_id,
            "eligible_ids_ranked": eligible,
            "all_candidates_diagnostic_ranked": diagnostic_ranked,
            "ranking": protocol["selection"][f"{target_sensor}_ranking"],
        }

    return chosen["heel"], chosen["toe"], {
        "status": "COMPONENTS_SELECTED_PENDING_COMBINED_REPLAY",
        "components": components,
        "candidate_gates": gates,
        "automatic_promotion_allowed": False,
    }


def _combined_candidate(
    candidates: Sequence[v1.PlacementCandidate],
    heel_id: str,
    toe_id: str,
) -> v1.PlacementCandidate:
    by_id = {item.candidate_id: item for item in candidates}
    heel = by_id[heel_id]
    toe = by_id[toe_id]
    if heel.geometry["target_sensor"] not in {"heel", "baseline"}:
        raise ProtocolError("combined V10 heel component has wrong target")
    if toe.geometry["target_sensor"] not in {"toe", "baseline"}:
        raise ProtocolError("combined V10 toe component has wrong target")
    bottom_offset_m = (
        toe.forefoot_location[1] - float(toe.geometry["toe_radius_m"])
    ) - (
        heel.heel_location[1] - float(heel.geometry["heel_radius_m"])
    )
    checks = {
        "exactly_two_spheres": True,
        "heel_component_pre_gate": bool(heel.geometry["pre_gate_ok"]),
        "toe_component_pre_gate": bool(toe.geometry["pre_gate_ok"]),
        "heel_to_toe_bottom_offset_within_20mm": abs(bottom_offset_m)
        <= 0.020 + NUMERIC_TOLERANCE,
    }
    return v1.PlacementCandidate(
        candidate_id=f"combined__{heel_id}__{toe_id}",
        heel_location=tuple(heel.heel_location),
        forefoot_location=tuple(toe.forefoot_location),
        heel_offset_below_current_mm=None,
        forefoot_fraction_mesh_x=None,
        forefoot_protrusion_mm=None,
        selectable=False,
        role="combined_selected_components",
        geometry={
            "source": "v10_selected_isolated_components",
            "parameter_arm": "combined_selected_components",
            "target_sensor": "heel_and_toe",
            "component_candidate_ids": {"heel": heel_id, "toe": toe_id},
            "center_raise_mm": float(heel.geometry["center_raise_mm"])
            + float(toe.geometry["center_raise_mm"]),
            "radius_reduction_mm": float(heel.geometry["radius_reduction_mm"])
            + float(toe.geometry["radius_reduction_mm"]),
            "heel_center_raise_mm": heel.geometry["heel_center_raise_mm"],
            "heel_radius_reduction_mm": heel.geometry[
                "heel_radius_reduction_mm"
            ],
            "toe_center_raise_mm": toe.geometry["toe_center_raise_mm"],
            "toe_radius_reduction_mm": toe.geometry["toe_radius_reduction_mm"],
            "heel_radius_m": heel.geometry["heel_radius_m"],
            "toe_radius_m": toe.geometry["toe_radius_m"],
            "effective_bottom_raise_mm": max(
                float(heel.geometry["effective_bottom_raise_mm"]),
                float(toe.geometry["effective_bottom_raise_mm"]),
            ),
            "heel_to_toe_bottom_offset_m": float(bottom_offset_m),
            "heel": heel.geometry["heel"],
            "toe": toe.geometry["toe"],
            "pre_gate_checks": checks,
            "pre_gate_ok": bool(all(checks.values())),
            "detector_representation": "two_spheres_only",
        },
    )


def _combined_common(
    heel_common: Mapping[str, Any],
    toe_common: Mapping[str, Any],
    candidate: v1.PlacementCandidate,
) -> dict[str, Any]:
    for key in (
        "times",
        "prescribed_vertical_n",
        "primary_aggregate",
        "primary_penetration",
    ):
        if not np.array_equal(np.asarray(heel_common[key]), np.asarray(toe_common[key])):
            raise ProtocolError(f"V10 component streams differ for shared {key}")
    heel_load = np.asarray(heel_common["detector_loads"]["left_heel"])
    toe_load = np.asarray(toe_common["detector_loads"]["left_toe"])
    heel_penetration = np.asarray(
        heel_common["detector_penetrations"]["left_heel"]
    )
    toe_penetration = np.asarray(
        toe_common["detector_penetrations"]["left_toe"]
    )
    heel_sphere = heel_common["detector_profile"].spheres[0]
    toe_sphere = toe_common["detector_profile"].spheres[1]
    profile = replace(
        heel_common["detector_profile"],
        source=f"validation_v10_{candidate.candidate_id}",
        spheres=(heel_sphere, toe_sphere),
    )
    return {
        **heel_common,
        "detector_profile": profile,
        "detector_loads": {
            "left_heel": heel_load,
            "left_toe": toe_load,
        },
        "detector_penetrations": {
            "left_heel": heel_penetration,
            "left_toe": toe_penetration,
        },
        "detector_aggregate": heel_load + toe_load,
    }


def _write_csv(path: Path, rows: Sequence[Mapping[str, Any]]) -> None:
    fields = sorted({str(key) for row in rows for key in row})
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def _plot(
    path: Path,
    rows_10: Sequence[Mapping[str, Any]],
    rows_1: Sequence[Mapping[str, Any]],
    highlighted_ids: Sequence[str],
) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    runtime = {str(row["candidate_id"]): row for row in rows_10}
    fine = {str(row["candidate_id"]): row for row in rows_1}
    ids = [
        str(row["candidate_id"])
        for row in rows_10
        if str(row["candidate_id"]) != BASELINE_ID
    ]
    x = np.arange(len(ids), dtype=float)
    width = 0.36
    fig, axes = plt.subplots(2, 2, figsize=(17, 10), sharex=True)
    specs = (
        ("max_abs_hs_error_s", "global max |HS| [ms]", 50.0, 1000.0),
        ("max_abs_toe_off_error_s", "global max |TO| [ms]", 80.0, 1000.0),
        ("early_max_abs_hs_error_s", "13.95-18.95 max |HS| [ms]", 50.0, 1000.0),
        ("long_max_abs_hs_error_s", "50-100 max |HS| [ms]", 50.0, 1000.0),
    )
    def plot_value(row: Mapping[str, Any], metric: str, scale: float) -> float:
        value = row.get(metric)
        if value is None:
            return float("nan")
        number = float(value)
        return scale * number if math.isfinite(number) else float("nan")

    for axis, (metric, ylabel, threshold, scale) in zip(axes.flat, specs):
        axis.bar(
            x - width / 2,
            [plot_value(runtime[item], metric, scale) for item in ids],
            width,
            label="10 ms",
        )
        axis.bar(
            x + width / 2,
            [plot_value(fine[item], metric, scale) for item in ids],
            width,
            label="1 ms",
        )
        axis.axhline(threshold, color="#E45756", linestyle="--", alpha=0.8)
        axis.set_ylabel(ylabel)
        axis.grid(axis="y", alpha=0.25)
        for selected in highlighted_ids:
            if selected in ids:
                index = ids.index(selected)
                axis.axvspan(
                    index - 0.48, index + 0.48, color="gold", alpha=0.12
                )
    axes[0, 0].legend(loc="best")
    for axis in axes[-1, :]:
        axis.set_xticks(x, ids, rotation=48, ha="right")
    fig.suptitle(
        "V10 isolated heel/toe center vs radius — confirmed-time timing"
    )
    fig.tight_layout()
    fig.savefig(path, dpi=170)
    plt.close(fig)


def run_sweep(
    protocol: Mapping[str, Any], output_dir: Path, plot_dir: Path
) -> dict[str, Any]:
    _preflight_no_clobber(output_dir, plot_dir)
    base, candidates, geometry = build_candidates(protocol)
    rows_by_dt: dict[str, list[dict[str, Any]]] = {}
    details_by_dt: dict[str, dict[str, Any]] = {}
    access_by_dt: dict[str, Any] = {}
    streams_by_dt: dict[str, dict[str, dict[str, Any]]] = {}
    for label, sample_dt_s in (
        ("runtime_10ms", PRIMARY_DT_S),
        ("fine_1ms", FINE_DT_S),
    ):
        streams, access = sample_streams_once(
            protocol, base, candidates, sample_dt_s=sample_dt_s
        )
        rows: list[dict[str, Any]] = []
        details: dict[str, Any] = {}
        for candidate in candidates:
            row, detail = evaluate_candidate(
                protocol,
                candidate,
                streams[candidate.candidate_id],
                sample_dt_s=sample_dt_s,
            )
            rows.append(row)
            details[candidate.candidate_id] = detail
        rows_by_dt[label] = rows
        details_by_dt[label] = details
        access_by_dt[label] = access
        streams_by_dt[label] = streams

    heel_id, toe_id, selection = select_components(
        rows_by_dt["runtime_10ms"], rows_by_dt["fine_1ms"], protocol
    )
    combined = _combined_candidate(candidates, heel_id, toe_id)
    combined_gates: dict[str, Any] = {}
    for label, sample_dt_s in (
        ("runtime_10ms", PRIMARY_DT_S),
        ("fine_1ms", FINE_DT_S),
    ):
        common = _combined_common(
            streams_by_dt[label][heel_id],
            streams_by_dt[label][toe_id],
            combined,
        )
        row, detail = evaluate_candidate(
            protocol, combined, common, sample_dt_s=sample_dt_s
        )
        rows_by_dt[label].append(row)
        details_by_dt[label][combined.candidate_id] = detail
        gate_key = (
            "runtime_gate_10ms" if label == "runtime_10ms" else "fine_gate_1ms"
        )
        combined_gates[label] = strict_gate(
            row, protocol[gate_key]
        )
    combined_ok = bool(
        combined_gates["runtime_10ms"]["ok"]
        and combined_gates["fine_1ms"]["ok"]
    )
    selection["combined_candidate"] = {
        "candidate_id": combined.candidate_id,
        "component_candidate_ids": {"heel": heel_id, "toe": toe_id},
        "runtime_10ms": combined_gates["runtime_10ms"],
        "fine_1ms": combined_gates["fine_1ms"],
        "passes_both": combined_ok,
    }
    selection["status"] = (
        "STRICT_COMBINED_MULTIRESOLUTION_WINNER"
        if combined_ok
        else "NO_STRICT_COMBINED_WINNER_DIAGNOSTIC_ONLY"
    )
    output_dir.mkdir(parents=True, exist_ok=False)
    plot_dir.mkdir(parents=True, exist_ok=False)
    csv_10 = output_dir / "center_radius_v10_runtime_10ms_metrics.csv"
    csv_1 = output_dir / "center_radius_v10_fine_1ms_metrics.csv"
    plot_path = plot_dir / "01_center_radius_v10_multiresolution.png"
    _write_csv(csv_10, rows_by_dt["runtime_10ms"])
    _write_csv(csv_1, rows_by_dt["fine_1ms"])
    _plot(
        plot_path,
        rows_by_dt["runtime_10ms"],
        rows_by_dt["fine_1ms"],
        (heel_id, toe_id, combined.candidate_id),
    )

    ok = combined_ok
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
        },
        "data_access": {
            "opened_interval_s": [OPEN_START_S, OPEN_END_S],
            "sealed_block_s": [OPEN_END_S, 155.045],
            "sealed_block_opened": False,
            **access_by_dt,
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
            "thresholds_dwell_fsm_unchanged": True,
        },
        "geometry": geometry,
        "candidates": [
            {
                "candidate_id": item.candidate_id,
                "selectable": item.selectable,
                "role": item.role,
                "heel_location_m": list(item.heel_location),
                "forefoot_location_m": list(item.forefoot_location),
                "heel_radius_m": item.geometry["heel_radius_m"],
                "forefoot_radius_m": item.geometry["toe_radius_m"],
                "center_raise_mm": item.geometry["center_raise_mm"],
                "radius_reduction_mm": item.geometry["radius_reduction_mm"],
                "effective_bottom_raise_mm": item.geometry[
                    "effective_bottom_raise_mm"
                ],
                "sensor_count": 2,
                "geometry": dict(item.geometry),
            }
            for item in (*candidates, combined)
        ],
        "runtime_10ms": {
            "rows": rows_by_dt["runtime_10ms"],
            "details": details_by_dt["runtime_10ms"],
        },
        "fine_1ms": {
            "rows": rows_by_dt["fine_1ms"],
            "details": details_by_dt["fine_1ms"],
        },
        "selection": selection,
        "selected_candidate": {
            "candidate_id": combined.candidate_id,
            "heel_component_id": heel_id,
            "toe_component_id": toe_id,
            "strict_multiresolution_pass": combined_ok,
            "profile_created": False,
            "promotable": False,
            "requires_future_frozen_validation": True,
        },
        "conclusion": (
            "STRICT_V10_COMBINED_DEVELOPMENT_WINNER"
            if ok
            else "NO_STRICT_V10_COMBINED_WINNER_DIAGNOSTIC_ONLY"
        ),
        "artifacts": {
            "runtime_metrics_csv": v1._source_record(csv_10),
            "fine_metrics_csv": v1._source_record(csv_1),
            "multiresolution_plot": v1._source_record(plot_path),
        },
        "non_actions": {
            "v9_modified": False,
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
        description="Run preregistered V10 isolated center/radius sweep."
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
                "selected_candidate": manifest["selected_candidate"],
                "sealed_block_opened": False,
                "manifest": v1._portable_path(output_dir / "manifest.json"),
            },
            indent=2,
        )
    )
    return 0 if manifest["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
