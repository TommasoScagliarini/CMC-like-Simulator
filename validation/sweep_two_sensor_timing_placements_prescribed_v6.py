"""Preregistered multiresolution placement sweep for a two-sensor detector.

Every candidate is exactly one heel sphere plus one forefoot sphere.  The
offline sampler shares identical sphere locations between candidates only to
avoid repeated OpenSim work; it does not create a multi-sensor detector.
Candidate forces generate HS/TO guards only.  FSM load/contact evidence comes
from the frozen primary online-GRF profile.

All candidates are evaluated on the already-open 50--100 s development block
at both 10 ms and 1 ms.  A selectable candidate must pass the same strict
event/phase/continuity gate at both resolutions.  The sealed block remains
closed, and this script cannot create or promote a detector profile.
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
import sweep_two_sensor_timing_placements_prescribed_v3 as v3  # noqa: E402
import sweep_two_sensor_timing_placements_prescribed_v4 as v4  # noqa: E402


DEFAULT_PROTOCOL = (
    VALIDATION_ROOT / "two_sensor_timing_placement_sweep_protocol_v6.json"
)
DEFAULT_OUTPUT_DIR = (
    VALIDATION_ROOT
    / "two_sensor_timing_placement_sweep_runs/"
    "2026-07-22_ab06_50_100_multires_geometry_v6"
)
DEFAULT_PLOT_DIR = REPO_ROOT / "plot/07_22_2026_two_sensor_timing_placement_v6"
PROTOCOL_ID = "AB06_TWO_SENSOR_TIMING_PLACEMENT_DEVELOPMENT_2026-07-22_V6"
HEEL_OFFSET_MM = 2.0
HEEL_X_DELTAS_MM = (2.5, 3.0)
FOREFOOT_FRACTIONS = (0.76, 0.78)
PROTRUSIONS_MM = (34.75, 35.0)
PRIMARY_DT_S = 0.01
FINE_DT_S = 0.001
MINIMUM_CAUSAL_CLEAR_S = 0.03
SELECTABLE_COUNT = 8
V5_COMPARATOR_ID = "v5_h02_x2_f80_p35_comparator"
CURRENT_COMPARATOR_ID = v1.CURRENT_PROFILE_ID
PAIR_COUNT = 10
EXPECTED_DETECTOR_SPHERES = 10
EXPECTED_PRIMARY_SPHERES = 8
EXPECTED_TOTAL_SPHERES = 18
NUMERIC_TOLERANCE = 1e-12


class ProtocolError(v1.ProtocolError):
    """Raised before sampling when the frozen V6 contract drifts."""


class NoClobberError(RuntimeError):
    """Raised before writes when a destination is occupied."""


def _occupied(path: Path) -> bool:
    return path.exists() and (not path.is_dir() or any(path.iterdir()))


def _preflight_no_clobber(output_dir: Path, plot_dir: Path) -> None:
    occupied = [path for path in (output_dir, plot_dir) if _occupied(path)]
    if occupied:
        joined = ", ".join(v1._portable_path(path) for path in occupied)
        raise NoClobberError(f"refusing to modify occupied path(s): {joined}")


def _token(value: float, digits: int = 2) -> str:
    return f"{float(value):.{digits}f}".replace(".", "p")


def _candidate_id(x_mm: float, fraction: float, depth_mm: float) -> str:
    return (
        f"H02_X{_token(x_mm)}_F{int(round(100.0 * fraction)):02d}_"
        f"P{_token(depth_mm)}"
    )


def _numeric_list(value: Any, expected: Sequence[float], label: str) -> None:
    try:
        observed = [float(item) for item in value]
    except (TypeError, ValueError) as exc:
        raise ProtocolError(f"{label} must be a numeric list") from exc
    if observed != [float(item) for item in expected]:
        raise ProtocolError(f"{label} drifted: {observed}")


def _gate_contract() -> dict[str, Any]:
    return {
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
        "minimum_causal_toe_clear_before_next_hs_onset_s": (
            MINIMUM_CAUSAL_CLEAR_S
        ),
        "require_exact_causal_swing_intervals": 50,
    }


def load_and_validate_protocol(
    path: str | Path = DEFAULT_PROTOCOL,
) -> dict[str, Any]:
    protocol_path = v1.resolve_repo_path(path).resolve()
    try:
        raw = json.loads(protocol_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ProtocolError(f"cannot load protocol: {protocol_path}") from exc
    if not isinstance(raw, dict):
        raise ProtocolError("protocol root must be an object")
    expected_top = {
        "schema_version": 6,
        "protocol_id": PROTOCOL_ID,
        "frozen_before_execution": True,
        "stage": "development_multiresolution_geometry",
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
    for key, expected in expected_top.items():
        if raw.get(key) != expected:
            raise ProtocolError(f"frozen protocol field drifted: {key}")
    if raw.get("profile_paths") != {
        CURRENT_COMPARATOR_ID: expected_top["detector_template_profile"]
    }:
        raise ProtocolError("current comparator mapping drifted")

    access = raw.get("data_access", {})
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
    common = access.get("expected_common_cycle_set", {})
    for key, expected in {
        "original_complete_cycles": 51,
        "retained_complete_cycles": 50,
        "reference_hs_count": 51,
        "reference_to_count": 50,
        "excluded_closing_hs_s": [99.96878691565038],
        "last_sample_10ms_s": 99.12,
        "last_sample_1ms_s": 99.12,
    }.items():
        if common.get(key) != expected:
            raise ProtocolError(f"common-cycle field drifted: {key}")

    replay = raw.get("replay", {})
    expected_replay = {
        "sea_plugin": "plugins/SEA_Plugin_BlackBox_mCMC_impedence_ff",
        "event_source": "two_sensor",
        "sensor_on_threshold_n": v1.SENSOR_ON_N,
        "sensor_off_threshold_n": v1.SENSOR_OFF_N,
        "sensor_dwell_s": v1.SENSOR_DWELL_S,
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
        "fsm_min_stance_duration_s": 0.30,
        "fsm_min_stance_contact_fraction": 0.20,
        "fsm_min_stance_load_bw_s": 0.04,
        "threshold_geometry_or_fsm_tuning_allowed_during_run": False,
    }
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
            raise ProtocolError(f"replay field drifted: {key}")

    grid = raw.get("placement_grid", {})
    _numeric_list(grid.get("heel_vertical_offsets_below_current_mm"), [2.0], "heel")
    _numeric_list(grid.get("heel_anterior_x_deltas_mm"), HEEL_X_DELTAS_MM, "heel x")
    _numeric_list(
        grid.get("forefoot_longitudinal_fractions_mesh_x"),
        FOREFOOT_FRACTIONS,
        "forefoot fraction",
    )
    _numeric_list(
        grid.get("forefoot_absolute_local_plantar_protrusion_mm"),
        PROTRUSIONS_MM,
        "forefoot depth",
    )
    for key, expected in {
        "selectable_candidate_count": SELECTABLE_COUNT,
        "nonselectable_comparator_count": 2,
        "total_pair_count": PAIR_COUNT,
        "sensors_per_pair": 2,
        "maximum_forefoot_protrusion_mm": 35.0,
        "maximum_absolute_heel_forefoot_bottom_offset_mm": 20.0,
    }.items():
        if grid.get(key) != expected:
            raise ProtocolError(f"placement-grid field drifted: {key}")

    if raw.get("evidence_routing") != {
        "heel_toe_event_guards": "candidate_two_sensor_forces_only",
        "normal_force_bw": "primary_online_grf_left_aggregate",
        "in_contact": "primary_online_grf_left_union_physical_penetration",
        "detector_spheres_generate_grf": False,
    }:
        raise ProtocolError("evidence routing drifted")
    expected_sampling = {
        "method": "all_required_spheres_directly_in_one_opensim_pass",
        "expected_unique_detector_spheres": EXPECTED_DETECTOR_SPHERES,
        "expected_primary_load_spheres": EXPECTED_PRIMARY_SPHERES,
        "expected_total_unique_spheres": EXPECTED_TOTAL_SPHERES,
        "evaluated_pair_count": PAIR_COUNT,
    }
    if raw.get("sampling") != {
        "runtime_10ms": expected_sampling,
        "fine_1ms": expected_sampling,
        "affine_reconstruction": False,
        "shared_spheres_are_offline_sampling_optimization_only": True,
    }:
        raise ProtocolError("sampling contract drifted")
    for key in ("runtime_gate_10ms", "fine_gate_1ms"):
        if raw.get(key) != _gate_contract():
            raise ProtocolError(f"{key} drifted")
    if raw.get("recontact_diagnostic") != {
        "role": "diagnostic_only_not_detector_gate",
        "interval": "accepted_to_confirmed_to_next_accepted_hs_onset",
        "signal": "debounced_forefoot_latch",
        "reason": "separate foot-clearance/scuff from gait-event detection",
    }:
        raise ProtocolError("recontact diagnostic contract drifted")
    if raw.get("selection") != {
        "eligibility": "full_strict_pass_at_both_10ms_and_1ms",
        "ranking": [
            "multiresolution_worst_event_normalized_max_abs_error:min",
            "multiresolution_mean_event_normalized_mean_abs_error:min",
            "minimum_multiresolution_confirmed_fsm_stance_iou:max",
            "geometry_displacement_from_current_m:min",
            "candidate_id:lexicographic",
        ],
    }:
        raise ProtocolError("selection contract drifted")
    if raw.get("decision_contract") != {
        "role": "development_multiresolution_geometry_screen_only",
        "candidate_selection_allowed": True,
        "profile_creation_allowed": False,
        "profile_promotion_allowed": False,
        "sealed_validation_allowed": False,
        "training_allowed": False,
        "runtime_modification_allowed": False,
    }:
        raise ProtocolError("decision contract drifted")

    sources = raw.get("sources")
    if not isinstance(sources, dict) or not sources:
        raise ProtocolError("hash-pinned sources are required")
    for label, record in sources.items():
        source_path = v1.resolve_repo_path(str(record.get("path", ""))).resolve()
        if not source_path.is_file():
            raise ProtocolError(f"missing pinned source: {label}")
        if v1._sha256(source_path) != record.get("sha256"):
            raise ProtocolError(f"source hash drift: {label}")

    raw["replay"]["sample_dt_s"] = PRIMARY_DT_S
    raw["development_gate"] = raw["runtime_gate_10ms"]
    raw["_protocol_path"] = protocol_path.as_posix()
    raw["_protocol_sha256"] = v1._sha256(protocol_path)
    raw["_primary_event_time_field"] = "confirmed_time_s"
    raw["_diagnostic_event_time_field"] = "event_time_s"
    raw["_phase_reference_mode"] = "validated_event_intervals"
    return raw


def build_placement_candidates(
    protocol: Mapping[str, Any],
) -> tuple[v1.OnlineGRFProfile, list[v1.PlacementCandidate], dict[str, Any]]:
    base = v1.load_online_grf_profile(
        v1.resolve_repo_path(str(protocol["detector_template_profile"])).resolve()
    )
    sensors = v1._left_sensor_spheres(base)
    heel_template = sensors["left_heel"]
    toe_template = sensors["left_toe"]
    setup = v1.read_setup_xml(v1.resolve_repo_path(str(protocol["setup"])).resolve())
    mesh_path = v1._resolve_left_foot_mesh(setup.model_file.resolve())
    triangles = v1._load_stl_triangles(mesh_path)
    radius = float(heel_template.radius)
    candidates: list[v1.PlacementCandidate] = []
    bottom_offsets: list[float] = []
    for x_mm in HEEL_X_DELTAS_MM:
        heel_location = (
            float(heel_template.location[0] + x_mm / 1000.0),
            float(heel_template.location[1] - HEEL_OFFSET_MM / 1000.0),
            float(heel_template.location[2]),
        )
        heel_geometry = v3._mesh_geometry(heel_location, radius, triangles)
        for fraction in FOREFOOT_FRACTIONS:
            for depth_mm in PROTRUSIONS_MM:
                toe_location, derived = v1._derive_forefoot_location(
                    triangles,
                    toe_template,
                    fraction=fraction,
                    protrusion_mm=depth_mm,
                )
                toe_geometry = v3._mesh_geometry(toe_location, radius, triangles)
                bottom_offset_mm = 1000.0 * (
                    float(toe_location[1] - radius)
                    - float(heel_location[1] - radius)
                )
                bottom_offsets.append(bottom_offset_mm)
                checks = {
                    "heel_within_5mm_of_mesh": heel_geometry["within_5mm_of_mesh"],
                    "forefoot_within_5mm_of_mesh": toe_geometry[
                        "within_5mm_of_mesh"
                    ],
                    "heel_forefoot_bottoms_within_20mm": bool(
                        abs(bottom_offset_mm) <= 20.0 + 1e-9
                    ),
                    "forefoot_depth_at_or_below_35mm": bool(depth_mm <= 35.0),
                    "exactly_two_spheres": True,
                }
                candidates.append(
                    v1.PlacementCandidate(
                        candidate_id=_candidate_id(x_mm, fraction, depth_mm),
                        heel_location=heel_location,
                        forefoot_location=toe_location,
                        heel_offset_below_current_mm=HEEL_OFFSET_MM,
                        forefoot_fraction_mesh_x=fraction,
                        forefoot_protrusion_mm=depth_mm,
                        selectable=True,
                        role="development_multiresolution_geometry_candidate",
                        geometry={
                            **derived,
                            "heel": heel_geometry,
                            "forefoot": toe_geometry,
                            "heel_anterior_x_delta_mm": x_mm,
                            "signed_forefoot_minus_heel_bottom_offset_mm": (
                                bottom_offset_mm
                            ),
                            "pre_gate_checks": checks,
                            "pre_gate_ok": bool(all(checks.values())),
                            "detector_representation": "two_spheres_only",
                        },
                    )
                )

    v5_heel = (
        float(heel_template.location[0] + 0.002),
        float(heel_template.location[1] - 0.002),
        float(heel_template.location[2]),
    )
    v5_toe, v5_derived = v1._derive_forefoot_location(
        triangles, toe_template, fraction=0.80, protrusion_mm=35.0
    )
    candidates.append(
        v1.PlacementCandidate(
            candidate_id=V5_COMPARATOR_ID,
            heel_location=v5_heel,
            forefoot_location=v5_toe,
            heel_offset_below_current_mm=2.0,
            forefoot_fraction_mesh_x=0.80,
            forefoot_protrusion_mm=35.0,
            selectable=False,
            role="nonselectable_v5_locked_comparator",
            geometry={
                **v5_derived,
                "heel_anterior_x_delta_mm": 2.0,
                "source": "V5_locked_H02_X2_F80_P35",
                "detector_representation": "two_spheres_only",
            },
        )
    )
    candidates.append(
        v1.PlacementCandidate(
            candidate_id=CURRENT_COMPARATOR_ID,
            heel_location=tuple(heel_template.location),
            forefoot_location=tuple(toe_template.location),
            heel_offset_below_current_mm=None,
            forefoot_fraction_mesh_x=None,
            forefoot_protrusion_mm=None,
            selectable=False,
            role="nonselectable_current_geometry_comparator",
            geometry={
                "source": "current_detector_profile",
                "detector_representation": "two_spheres_only",
            },
        )
    )
    selectable = [candidate for candidate in candidates if candidate.selectable]
    if len(selectable) != SELECTABLE_COUNT or len(candidates) != PAIR_COUNT:
        raise ProtocolError("V6 candidate grid drifted")
    if len({candidate.candidate_id for candidate in candidates}) != PAIR_COUNT:
        raise ProtocolError("V6 candidate IDs are not unique")
    if not all(bool(item.geometry.get("pre_gate_ok")) for item in selectable):
        raise ProtocolError("a selectable V6 geometry failed its pre-gate")
    maximum_offset = max(abs(value) for value in bottom_offsets)
    if maximum_offset > 20.0 + 1e-9:
        raise ProtocolError("V6 bottom-offset pre-gate failed")
    return base, candidates, {
        "mesh": v1._source_record(mesh_path),
        "selectable_candidate_count": SELECTABLE_COUNT,
        "nonselectable_comparator_count": 2,
        "total_pair_count": PAIR_COUNT,
        "sensors_per_pair": 2,
        "maximum_absolute_bottom_offset_mm": maximum_offset,
        "all_bottom_offsets_within_20mm": True,
        "detector_representation": "two_spheres_per_pair",
        "shared_spheres_are_offline_sampling_optimization_only": True,
        "affine_reconstruction_used": False,
    }


def _replay_for_common(common: Mapping[str, Any]) -> tuple[np.ndarray, dict[str, Any]]:
    inputs, _sources = dual.compose_branch_inputs(common, "B_primary_load")
    runtime_cfg = replace(
        v1._current_runtime_fsm_config(),
        sensor_on_threshold_n=v1.SENSOR_ON_N,
        sensor_off_threshold_n=v1.SENSOR_OFF_N,
        sensor_dwell_s=v1.SENSOR_DWELL_S,
    )
    replay = v1._run_production_fsm(
        np.asarray(inputs["times"], dtype=float),
        dict(inputs["loads"]),
        dict(inputs["penetrations"]),
        np.asarray(inputs["aggregate"], dtype=float),
        dict(inputs["kinematics"]),
        body_weight_n=float(inputs["body_weight_n"]),
        fsm_config=runtime_cfg,
    )
    return np.asarray(inputs["times"], dtype=float), replay


def causal_swing_clearance_diagnostics(
    common: Mapping[str, Any], *, sample_dt_s: float
) -> dict[str, Any]:
    """Measure toe-latch clearance against the next accepted heel onset."""
    times, replay = _replay_for_common(common)
    toe_contact = np.asarray(replay["toe_contact"], dtype=float) > 0.5
    accepted = [
        dict(item)
        for item in replay["accepted"]
        if str(item.get("event", "")) in {"heel_strike", "toe_off"}
        and not (
            str(item.get("event", "")) == "toe_off"
            and float(item.get("segment_valid", 1.0)) == 0.0
        )
    ]
    heel = [item for item in accepted if item["event"] == "heel_strike"]
    toe = [item for item in accepted if item["event"] == "toe_off"]
    cycles: list[dict[str, Any]] = []
    total_recontacts = 0
    for cycle_index, toe_event in enumerate(toe):
        to_confirmed_s = float(toe_event["confirmed_time_s"])
        next_heel = next(
            (
                item
                for item in heel
                if float(item["event_time_s"]) > to_confirmed_s + NUMERIC_TOLERANCE
            ),
            None,
        )
        if next_heel is None:
            continue
        heel_onset_s = float(next_heel["event_time_s"])
        indices = np.flatnonzero(
            (times >= to_confirmed_s - NUMERIC_TOLERANCE)
            & (times < heel_onset_s - NUMERIC_TOLERANCE)
        )
        active_indices = indices[toe_contact[indices]]
        if active_indices.size:
            last_active_s = float(times[active_indices[-1]])
            first_guaranteed_clear_s = last_active_s + float(sample_dt_s)
        else:
            last_active_s = None
            first_guaranteed_clear_s = to_confirmed_s
        local = toe_contact[indices]
        previous = False
        episode_count = 0
        for active in local:
            is_active = bool(active)
            if is_active and not previous:
                episode_count += 1
            previous = is_active
        total_recontacts += episode_count
        margin_s = heel_onset_s - first_guaranteed_clear_s
        cycles.append(
            {
                "cycle_index": cycle_index,
                "accepted_to_confirmed_s": to_confirmed_s,
                "next_accepted_hs_onset_s": heel_onset_s,
                "last_active_toe_latch_sample_s": last_active_s,
                "first_guaranteed_toe_latch_clear_s": first_guaranteed_clear_s,
                "causal_clear_margin_s": margin_s,
                "toe_latch_recontact_episode_count": episode_count,
            }
        )
    margins = [float(item["causal_clear_margin_s"]) for item in cycles]
    return {
        "definition": (
            "next accepted HS onset minus the first sample boundary after the "
            "last active debounced toe-latch sample following accepted TO"
        ),
        "interval_count": len(cycles),
        "minimum_causal_clear_s": min(margins) if margins else -1.0,
        "toe_latch_recontact_episode_count": total_recontacts,
        "recontact_role": "diagnostic_only_not_detector_gate",
        "cycles": cycles,
    }


def evaluate_placement(
    protocol: Mapping[str, Any],
    candidate: v1.PlacementCandidate,
    common: Mapping[str, Any],
    *,
    sample_dt_s: float,
) -> tuple[dict[str, Any], dict[str, Any]]:
    row, detail = v4.evaluate_placement(
        protocol, candidate, common, sample_dt_s=sample_dt_s
    )
    causal = causal_swing_clearance_diagnostics(common, sample_dt_s=sample_dt_s)
    row.update(
        {
            "causal_swing_interval_count": causal["interval_count"],
            "minimum_causal_toe_clear_before_next_hs_onset_s": causal[
                "minimum_causal_clear_s"
            ],
            "toe_latch_recontact_episode_count": causal[
                "toe_latch_recontact_episode_count"
            ],
        }
    )
    detail["causal_swing_clearance"] = causal
    return row, detail


def strict_gate(
    row: Mapping[str, Any],
    current: Mapping[str, Any],
    protocol: Mapping[str, Any],
    *,
    gate_key: str,
) -> dict[str, Any]:
    inherited = v4.strict_gate(row, current, protocol, gate_key=gate_key)
    gate = protocol[gate_key]
    interval_ok = int(row["causal_swing_interval_count"]) == int(
        gate["require_exact_causal_swing_intervals"]
    )
    clear_ok = float(
        row["minimum_causal_toe_clear_before_next_hs_onset_s"]
    ) >= float(
        gate["minimum_causal_toe_clear_before_next_hs_onset_s"]
    ) - NUMERIC_TOLERANCE
    checks = dict(inherited["checks"])
    checks["exact_causal_swing_intervals"] = interval_ok
    checks["minimum_causal_toe_clear_before_next_hs_onset"] = clear_ok
    return {
        **inherited,
        "ok": bool(inherited["ok"] and interval_ok and clear_ok),
        "checks": checks,
        "minimum_causal_clear_s": float(
            gate["minimum_causal_toe_clear_before_next_hs_onset_s"]
        ),
        "recontact_is_diagnostic_only": True,
    }


def select_multiresolution_winner(
    runtime_rows: Sequence[Mapping[str, Any]],
    fine_rows: Sequence[Mapping[str, Any]],
    protocol: Mapping[str, Any],
) -> tuple[str | None, dict[str, Any]]:
    runtime = {str(row["candidate_id"]): row for row in runtime_rows}
    fine = {str(row["candidate_id"]): row for row in fine_rows}
    expected_ids = set(runtime)
    if expected_ids != set(fine) or len(expected_ids) != PAIR_COUNT:
        raise ProtocolError("10 ms and 1 ms candidate sets differ")
    current_10 = runtime[CURRENT_COMPARATOR_ID]
    current_1 = fine[CURRENT_COMPARATOR_ID]
    selectable_ids = sorted(
        candidate_id
        for candidate_id, row in runtime.items()
        if bool(row.get("selectable"))
    )
    if len(selectable_ids) != SELECTABLE_COUNT:
        raise ProtocolError("multiresolution selection requires eight candidates")
    gates: dict[str, Any] = {}
    eligible: list[str] = []
    for candidate_id in selectable_ids:
        gate_10 = strict_gate(
            runtime[candidate_id],
            current_10,
            protocol,
            gate_key="runtime_gate_10ms",
        )
        gate_1 = strict_gate(
            fine[candidate_id],
            current_1,
            protocol,
            gate_key="fine_gate_1ms",
        )
        gates[candidate_id] = {
            "runtime_10ms": gate_10,
            "fine_1ms": gate_1,
            "passes_both": bool(gate_10["ok"] and gate_1["ok"]),
        }
        if gate_10["ok"] and gate_1["ok"]:
            eligible.append(candidate_id)

    def rank_key(candidate_id: str) -> tuple[Any, ...]:
        row_10 = runtime[candidate_id]
        row_1 = fine[candidate_id]
        return (
            max(
                float(row_10["worst_event_normalized_max_abs_error"]),
                float(row_1["worst_event_normalized_max_abs_error"]),
            ),
            0.5
            * (
                float(row_10["mean_event_normalized_mean_abs_error"])
                + float(row_1["mean_event_normalized_mean_abs_error"])
            ),
            -min(
                float(row_10["confirmed_fsm_stance_iou"]),
                float(row_1["confirmed_fsm_stance_iou"]),
            ),
            float(row_1["geometry_displacement_from_current_m"]),
            candidate_id,
        )

    eligible.sort(key=rank_key)
    winner = eligible[0] if eligible else None
    return winner, {
        "status": (
            "STRICT_MULTIRESOLUTION_WINNER_LOCKED"
            if winner is not None
            else "NO_STRICT_MULTIRESOLUTION_WINNER"
        ),
        "strict_winner_id": winner,
        "candidate_gates": gates,
        "ranked_eligible_ids": eligible,
        "winner_rule": protocol["selection"],
        "automatic_promotion_allowed": False,
    }


def _write_csv(path: Path, rows: Sequence[Mapping[str, Any]]) -> None:
    fields = sorted({str(key) for row in rows for key in row})
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def _plot(
    path: Path,
    runtime_rows: Sequence[Mapping[str, Any]],
    fine_rows: Sequence[Mapping[str, Any]],
    winner: str | None,
) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    runtime = {str(row["candidate_id"]): row for row in runtime_rows}
    fine = {str(row["candidate_id"]): row for row in fine_rows}
    ids = [
        str(row["candidate_id"])
        for row in runtime_rows
        if bool(row.get("selectable"))
    ]
    positions = np.arange(len(ids), dtype=float)
    width = 0.36
    fig, axes = plt.subplots(2, 2, figsize=(15, 10), sharex=True)
    metrics = (
        ("max_abs_hs_error_s", "max |HS error| [ms]", 50.0, 1000.0),
        ("max_abs_toe_off_error_s", "max |TO error| [ms]", 80.0, 1000.0),
        (
            "transfer_both_latches_off_sample_count",
            "heel+toe both OFF [samples]",
            0.0,
            1.0,
        ),
        (
            "minimum_causal_toe_clear_before_next_hs_onset_s",
            "minimum causal toe-clear [ms]",
            30.0,
            1000.0,
        ),
    )
    for axis, (metric, ylabel, threshold, scale) in zip(axes.flat, metrics):
        values_10 = [scale * float(runtime[item][metric]) for item in ids]
        values_1 = [scale * float(fine[item][metric]) for item in ids]
        axis.bar(positions - width / 2, values_10, width, label="10 ms")
        axis.bar(positions + width / 2, values_1, width, label="1 ms")
        axis.axhline(threshold, color="#E45756", linestyle="--", alpha=0.8)
        axis.set_ylabel(ylabel)
        axis.grid(axis="y", alpha=0.25)
        if winner is not None:
            selected = ids.index(winner)
            axis.axvspan(selected - 0.48, selected + 0.48, color="gold", alpha=0.16)
    axes[0, 0].legend(loc="best")
    for axis in axes[-1, :]:
        axis.set_xticks(positions, ids, rotation=45, ha="right")
    fig.suptitle(
        "Two-sensor V6 — direct 10 ms / 1 ms prescribed development replay"
    )
    fig.tight_layout()
    fig.savefig(path, dpi=170)
    plt.close(fig)


def run_sweep(
    protocol: Mapping[str, Any], output_dir: Path, plot_dir: Path
) -> dict[str, Any]:
    _preflight_no_clobber(output_dir, plot_dir)
    base, candidates, geometry = build_placement_candidates(protocol)
    rows_by_dt: dict[str, list[dict[str, Any]]] = {}
    details_by_dt: dict[str, dict[str, Any]] = {}
    access_by_dt: dict[str, Any] = {}
    for label, sample_dt_s in (("runtime_10ms", PRIMARY_DT_S), ("fine_1ms", FINE_DT_S)):
        streams, access = v4.sample_streams_once(
            protocol,
            base,
            candidates,
            sample_dt_s=sample_dt_s,
            expected_detector_spheres=EXPECTED_DETECTOR_SPHERES,
            expected_total_spheres=EXPECTED_TOTAL_SPHERES,
        )
        rows: list[dict[str, Any]] = []
        details: dict[str, Any] = {}
        for candidate in candidates:
            row, detail = evaluate_placement(
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

    winner_id, selection = select_multiresolution_winner(
        rows_by_dt["runtime_10ms"], rows_by_dt["fine_1ms"], protocol
    )
    output_dir.mkdir(parents=True, exist_ok=False)
    plot_dir.mkdir(parents=True, exist_ok=False)
    csv_10 = output_dir / "timing_placement_v6_runtime_10ms_metrics.csv"
    csv_1 = output_dir / "timing_placement_v6_fine_1ms_metrics.csv"
    plot_path = plot_dir / "01_timing_placement_v6_multiresolution.png"
    _write_csv(csv_10, rows_by_dt["runtime_10ms"])
    _write_csv(csv_1, rows_by_dt["fine_1ms"])
    _plot(
        plot_path,
        rows_by_dt["runtime_10ms"],
        rows_by_dt["fine_1ms"],
        winner_id,
    )
    ok = winner_id is not None
    manifest = {
        "schema_version": 6,
        "status": "PASS" if ok else "FAIL",
        "ok": ok,
        "stage": "development_multiresolution_geometry",
        "objective": protocol["objective"],
        "protocol": {
            "path": v1._portable_path(Path(str(protocol["_protocol_path"]))),
            "sha256": protocol["_protocol_sha256"],
            "protocol_id": protocol["protocol_id"],
        },
        "data_access": {
            "already_open_block_s": [v1.BLOCK_START_S, v1.SEALED_START_S],
            "sealed_block_s": [v1.SEALED_START_S, v1.SEALED_END_S],
            "sealed_block_opened": False,
            **access_by_dt,
        },
        "detector_contract": {
            "sensors_per_pair": 2,
            "sensor_roles": ["heel", "forefoot"],
            "event_guard_source": "candidate_two_sensor_forces_only",
            "normal_force_bw_source": "primary_online_grf_left_aggregate",
            "in_contact_source": (
                "primary_online_grf_left_union_physical_penetration"
            ),
            "detector_spheres_generate_grf": False,
            "fsm_thresholds_and_dwell_unchanged": True,
            "shared_batch_spheres_are_not_runtime_sensors": True,
        },
        "geometry": geometry,
        "candidates": [
            {
                "candidate_id": item.candidate_id,
                "selectable": item.selectable,
                "role": item.role,
                "heel_location_m": list(item.heel_location),
                "forefoot_location_m": list(item.forefoot_location),
                "heel_anterior_x_delta_mm": item.geometry.get(
                    "heel_anterior_x_delta_mm"
                ),
                "forefoot_fraction_mesh_x": item.forefoot_fraction_mesh_x,
                "forefoot_protrusion_mm": item.forefoot_protrusion_mm,
                "sensor_count": 2,
                "geometry": dict(item.geometry),
            }
            for item in candidates
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
        "selected_pair": {
            "candidate_id": winner_id,
            "passed_10ms_and_1ms": ok,
            "profile_created": False,
            "promotable": False,
            "requires_future_sealed_holdout": True,
        },
        "conclusion": (
            "STRICT_MULTIRESOLUTION_DEVELOPMENT_WINNER"
            if ok
            else "NO_STRICT_MULTIRESOLUTION_DEVELOPMENT_WINNER"
        ),
        "artifacts": {
            "runtime_metrics_csv": v1._source_record(csv_10),
            "fine_metrics_csv": v1._source_record(csv_1),
            "multiresolution_plot": v1._source_record(plot_path),
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
        description="Run preregistered two-sensor V6 multiresolution sweep."
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
                    "schema_version": 6,
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
                    "schema_version": 6,
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
                "selected_pair": manifest["selected_pair"],
                "sealed_block_opened": False,
                "manifest": v1._portable_path(output_dir / "manifest.json"),
            },
            indent=2,
        )
    )
    return 0 if manifest["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
