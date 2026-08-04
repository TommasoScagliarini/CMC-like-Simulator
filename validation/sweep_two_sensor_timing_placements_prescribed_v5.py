"""Preregistered V5 depth micro-sweep for a simple two-sensor detector.

The heel is frozen at H2 and 2 mm anterior to its original x coordinate.  The
single F80 forefoot sensor varies only from P34.25 to P36.00.  Every candidate
therefore remains exactly one heel sphere plus one forefoot sphere.  Event
guards use only those two forces, while FSM load/contact evidence comes from
the frozen primary online-GRF profile.

The already-open 50--100 s block is sampled at 10 ms.  A 1 ms replay is
allowed only for one strict 10 ms winner plus current geometry.  The sealed
block remains closed and the harness cannot create or promote a profile.
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
    VALIDATION_ROOT / "two_sensor_timing_placement_sweep_protocol_v5.json"
)
DEFAULT_OUTPUT_DIR = (
    VALIDATION_ROOT
    / "two_sensor_timing_placement_sweep_runs/"
    "2026-07-22_ab06_50_100_x2_depth_micro_v5"
)
DEFAULT_PLOT_DIR = (
    REPO_ROOT / "plot/07_22_2026_two_sensor_timing_placement_v5"
)
PROTOCOL_ID = "AB06_TWO_SENSOR_TIMING_PLACEMENT_DEVELOPMENT_2026-07-22_V5"
HEEL_OFFSET_MM = 2.0
HEEL_ANTERIOR_X_DELTA_MM = 2.0
FOREFOOT_FRACTION = 0.80
PROTRUSIONS_MM = (34.25, 34.5, 34.75, 35.0, 35.25, 35.5, 35.75, 36.0)
CURRENT_COMPARATOR_ID = v1.CURRENT_PROFILE_ID
PRIMARY_DT_S = 0.01
SENSITIVITY_DT_S = 0.001
SELECTABLE_COUNT = 8
PAIR_COUNT = 9
NUMERIC_TOLERANCE = 1e-12


class ProtocolError(v1.ProtocolError):
    """Raised before sampling when the frozen V5 contract drifts."""


class NoClobberError(RuntimeError):
    """Raised before writes when a destination is occupied."""


def _occupied(path: Path) -> bool:
    return path.exists() and (not path.is_dir() or any(path.iterdir()))


def _preflight_no_clobber(output_dir: Path, plot_dir: Path) -> None:
    occupied = [path for path in (output_dir, plot_dir) if _occupied(path)]
    if occupied:
        joined = ", ".join(v1._portable_path(path) for path in occupied)
        raise NoClobberError(f"refusing to modify occupied path(s): {joined}")


def _candidate_id(depth_mm: float) -> str:
    return f"H02_X2_F80_P{float(depth_mm):.2f}".replace(".", "p")


def _float_list(value: Any, expected: Sequence[float], label: str) -> None:
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
    expected_top = {
        "schema_version": 5,
        "protocol_id": PROTOCOL_ID,
        "frozen_before_execution": True,
        "stage": "development_depth_micro_with_conditional_sensitivity",
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
    if not isinstance(raw, dict):
        raise ProtocolError("protocol root must be an object")
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
        "primary_sample_dt_s": PRIMARY_DT_S,
        "winner_sensitivity_sample_dt_s": SENSITIVITY_DT_S,
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
        "sensitivity_requires_strict_10ms_winner": True,
        "sensitivity_evaluates_selected_and_current_only": True,
        "one_ms_cannot_convert_10ms_fail_to_pass": True,
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
    _float_list(grid.get("heel_vertical_offsets_below_current_mm"), [2.0], "heel")
    _float_list(grid.get("heel_anterior_x_deltas_mm"), [2.0], "heel x")
    _float_list(grid.get("forefoot_longitudinal_fractions_mesh_x"), [0.8], "x")
    _float_list(
        grid.get("forefoot_absolute_local_plantar_protrusion_mm"),
        PROTRUSIONS_MM,
        "depth",
    )
    expected_grid = {
        "selectable_candidate_count": SELECTABLE_COUNT,
        "nonselectable_current_comparator_count": 1,
        "total_pair_count": PAIR_COUNT,
        "sensors_per_pair": 2,
        "maximum_forefoot_protrusion_mm": 36.0,
        "maximum_absolute_heel_forefoot_bottom_offset_mm": 20.0,
    }
    for key, expected in expected_grid.items():
        if grid.get(key) != expected:
            raise ProtocolError(f"placement grid drifted: {key}")

    if raw.get("evidence_routing") != {
        "heel_toe_event_guards": "candidate_two_sensor_forces_only",
        "normal_force_bw": "primary_online_grf_left_aggregate",
        "in_contact": "primary_online_grf_left_union_physical_penetration",
        "detector_spheres_generate_grf": False,
    }:
        raise ProtocolError("evidence routing drifted")
    if raw.get("sampling") != {
        "primary_10ms": {
            "method": "all_required_spheres_directly_in_one_opensim_pass",
            "expected_unique_detector_spheres": 11,
            "expected_primary_load_spheres": 8,
            "expected_total_unique_spheres": 19,
            "evaluated_pair_count": PAIR_COUNT,
        },
        "conditional_winner_current_1ms": {
            "method": "winner_and_current_directly_in_one_opensim_pass",
            "expected_unique_detector_spheres": 4,
            "expected_primary_load_spheres": 8,
            "expected_total_unique_spheres": 12,
            "evaluated_pair_count": 2,
        },
        "affine_reconstruction": False,
    }:
        raise ProtocolError("sampling contract drifted")
    for key, label in (
        ("development_gate_10ms", "10 ms gate"),
        ("sensitivity_gate_1ms", "1 ms gate"),
    ):
        gate = raw.get(key)
        if not isinstance(gate, dict):
            raise ProtocolError(f"{label} is required")
        if not math.isclose(
            float(gate.get("minimum_forefoot_release_margin_s", -1.0)),
            0.25,
            rel_tol=0.0,
            abs_tol=1e-12,
        ):
            raise ProtocolError(f"{label} release margin drifted")
        inherited = dict(gate)
        inherited.pop("minimum_forefoot_release_margin_s", None)
        try:
            v4._validate_gate(inherited, label)
        except v1.ProtocolError as exc:
            raise ProtocolError(str(exc)) from exc
    if raw.get("operational_reference_noise_diagnostic") != {
        "role": "secondary_diagnostic_only",
        "expected_reference_hs_count": 51,
        "minimum_hs_within_50ms_count": 50,
        "within_tolerance_s": 0.05,
        "maximum_abs_hs_error_s": 0.06,
        "can_replace_strict_gate": False,
        "can_make_candidate_selectable": False,
        "can_make_candidate_promotable": False,
    }:
        raise ProtocolError("operational reference-noise diagnostic drifted")
    if raw.get("selection") != {
        "winner_rule": "shallowest_full_strict_pass_then_timing_tiebreakers",
        "primary_order": "minimum_forefoot_protrusion_mm",
        "tie_breakers": [
            "worst_event_normalized_max_abs_error:min",
            "mean_event_normalized_mean_abs_error:min",
            "confirmed_fsm_stance_iou:max",
            "geometry_displacement_from_current_m:min",
            "candidate_id:lexicographic",
        ],
        "maximum_selectable_depth_mm": 36.0,
    }:
        raise ProtocolError("selection contract drifted")
    if raw.get("sealed_validation_gate") != {
        "precision": 1.0,
        "recall": 1.0,
        "max_abs_hs_error_s": 0.05,
        "max_abs_toe_off_error_s": 0.08,
        "minimum_confirmed_fsm_stance_f1": 0.95,
        "minimum_confirmed_fsm_stance_iou": 0.90,
        "maximum_confirmed_fsm_iou_regression_vs_baseline": 0.01,
        "maximum_confirmed_time_worst_timing_regression_vs_baseline": 0.0,
    }:
        raise ProtocolError("strict confirmed-time gate drifted")
    if raw.get("decision_contract") != {
        "role": "development_depth_micro_screen_only",
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
    raw["development_gate"] = raw["development_gate_10ms"]
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
    setup = v1.read_setup_xml(
        v1.resolve_repo_path(str(protocol["setup"])).resolve()
    )
    mesh_path = v1._resolve_left_foot_mesh(setup.model_file.resolve())
    triangles = v1._load_stl_triangles(mesh_path)
    radius = float(heel_template.radius)
    heel_location = (
        float(heel_template.location[0] + HEEL_ANTERIOR_X_DELTA_MM / 1000.0),
        float(heel_template.location[1] - HEEL_OFFSET_MM / 1000.0),
        float(heel_template.location[2]),
    )
    heel_geometry = v3._mesh_geometry(heel_location, radius, triangles)
    candidates: list[v1.PlacementCandidate] = []
    bottom_offsets: list[float] = []
    for depth_mm in PROTRUSIONS_MM:
        toe_location, derived = v1._derive_forefoot_location(
            triangles,
            toe_template,
            fraction=FOREFOOT_FRACTION,
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
            "forefoot_within_5mm_of_mesh": toe_geometry["within_5mm_of_mesh"],
            "heel_forefoot_bottoms_within_20mm": bool(
                abs(bottom_offset_mm) <= 20.0 + 1e-9
            ),
            "forefoot_depth_at_or_below_36mm": bool(depth_mm <= 36.0),
            "exactly_two_spheres": True,
        }
        candidates.append(
            v1.PlacementCandidate(
                candidate_id=_candidate_id(depth_mm),
                heel_location=heel_location,
                forefoot_location=toe_location,
                heel_offset_below_current_mm=HEEL_OFFSET_MM,
                forefoot_fraction_mesh_x=FOREFOOT_FRACTION,
                forefoot_protrusion_mm=depth_mm,
                selectable=True,
                role="development_x2_depth_micro_candidate",
                geometry={
                    **derived,
                    "heel": heel_geometry,
                    "forefoot": toe_geometry,
                    "heel_anterior_x_delta_mm": HEEL_ANTERIOR_X_DELTA_MM,
                    "signed_forefoot_minus_heel_bottom_offset_mm": bottom_offset_mm,
                    "pre_gate_checks": checks,
                    "pre_gate_ok": bool(all(checks.values())),
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
    if len(candidates) != PAIR_COUNT or len(
        {candidate.candidate_id for candidate in candidates}
    ) != PAIR_COUNT:
        raise ProtocolError("V5 candidate grid drifted")
    if not all(
        bool(candidate.geometry.get("pre_gate_ok"))
        for candidate in candidates
        if candidate.selectable
    ):
        raise ProtocolError("a selectable V5 geometry failed its pre-gate")
    maximum_offset = max(abs(value) for value in bottom_offsets)
    if maximum_offset > 20.0 + 1e-9:
        raise ProtocolError("V5 bottom-offset pre-gate failed")
    return base, candidates, {
        "mesh": v1._source_record(mesh_path),
        "selectable_candidate_count": SELECTABLE_COUNT,
        "total_pair_count": PAIR_COUNT,
        "sensors_per_pair": 2,
        "maximum_absolute_bottom_offset_mm": maximum_offset,
        "all_bottom_offsets_within_20mm": True,
        "detector_representation": "two_spheres_per_pair",
        "affine_reconstruction_used": False,
    }


def forefoot_release_margin_diagnostics(
    common: Mapping[str, Any],
    *,
    sample_dt_s: float,
) -> dict[str, Any]:
    """Measure clearance from the final active toe-latch sample to next HS."""
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
    times = np.asarray(inputs["times"], dtype=float)
    toe_contact = np.asarray(replay["toe_contact"], dtype=float) > 0.5
    reference_hs = np.asarray(
        inputs["reference_events"]["heel_strike"], dtype=float
    )
    reference_to = np.asarray(
        inputs["reference_events"]["toe_off"], dtype=float
    )
    cycles: list[dict[str, Any]] = []
    for cycle_index, (toe_off_s, next_hs_s) in enumerate(
        zip(reference_to, reference_hs[1:])
    ):
        indices = np.flatnonzero((times >= toe_off_s) & (times < next_hs_s))
        if indices.size == 0:
            raise ProtocolError(
                f"empty reference swing interval at cycle {cycle_index}"
            )
        active_indices = indices[toe_contact[indices]]
        if active_indices.size:
            last_active_sample_s = float(times[active_indices[-1]])
            first_guaranteed_off_s = last_active_sample_s + float(sample_dt_s)
        else:
            last_active_sample_s = None
            first_guaranteed_off_s = float(toe_off_s)
        margin_s = float(next_hs_s) - first_guaranteed_off_s
        cycles.append(
            {
                "cycle_index": cycle_index,
                "reference_to_s": float(toe_off_s),
                "next_reference_hs_s": float(next_hs_s),
                "last_active_toe_latch_sample_s": last_active_sample_s,
                "first_guaranteed_toe_latch_off_s": first_guaranteed_off_s,
                "release_margin_s": margin_s,
            }
        )
    margins = [float(item["release_margin_s"]) for item in cycles]
    return {
        "definition": (
            "next reference HS minus the first sample boundary guaranteed "
            "to follow the final active debounced toe-latch sample"
        ),
        "minimum_s": min(margins) if margins else -1.0,
        "per_cycle_s": margins,
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
        protocol,
        candidate,
        common,
        sample_dt_s=sample_dt_s,
    )
    release_margin = forefoot_release_margin_diagnostics(
        common,
        sample_dt_s=sample_dt_s,
    )
    row["minimum_forefoot_release_margin_s"] = release_margin["minimum_s"]
    detail["forefoot_release_margin"] = release_margin
    return row, detail


def strict_gate(
    row: Mapping[str, Any],
    current: Mapping[str, Any],
    protocol: Mapping[str, Any],
    *,
    gate_key: str,
) -> dict[str, Any]:
    inherited = v4.strict_gate(
        row,
        current,
        protocol,
        gate_key=gate_key,
    )
    minimum = float(protocol[gate_key]["minimum_forefoot_release_margin_s"])
    release_ok = (
        float(row["minimum_forefoot_release_margin_s"])
        >= minimum - NUMERIC_TOLERANCE
    )
    checks = dict(inherited["checks"])
    checks["minimum_forefoot_release_margin"] = release_ok
    return {
        **inherited,
        "ok": bool(inherited["ok"] and release_ok),
        "checks": checks,
        "minimum_forefoot_release_margin_s": minimum,
    }


def select_10ms_winner(
    rows: Sequence[Mapping[str, Any]],
    protocol: Mapping[str, Any],
) -> tuple[str | None, dict[str, Any]]:
    current_rows = [
        row for row in rows if row["candidate_id"] == CURRENT_COMPARATOR_ID
    ]
    selectable = [row for row in rows if bool(row.get("selectable"))]
    if len(current_rows) != 1 or len(selectable) != SELECTABLE_COUNT:
        raise ProtocolError("10 ms selection requires frozen 8+1 rows")
    current = current_rows[0]
    assessments = {
        str(row["candidate_id"]): strict_gate(
            row,
            current,
            protocol,
            gate_key="development_gate_10ms",
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
            float(row["forefoot_protrusion_mm"]),
            float(row["worst_event_normalized_max_abs_error"]),
            float(row["mean_event_normalized_mean_abs_error"]),
            -float(row["confirmed_fsm_stance_iou"]),
            float(row["geometry_displacement_from_current_m"]),
            str(row["candidate_id"]),
        )
    )
    winner = str(eligible[0]["candidate_id"]) if eligible else None
    return winner, {
        "status": (
            "STRICT_10MS_WINNER_LOCKED"
            if winner is not None
            else "NO_STRICT_10MS_WINNER"
        ),
        "strict_winner_id": winner,
        "candidate_gates": assessments,
        "winner_rule": "shallowest_full_strict_pass_then_timing_tiebreakers",
        "operational_reference_noise_diagnostics": {
            str(row["candidate_id"]): v4.operational_reference_noise_diagnostic(
                row, protocol
            )
            for row in selectable
        },
        "automatic_promotion_allowed": False,
    }


def assess_1ms(
    winner_row: Mapping[str, Any],
    current_row: Mapping[str, Any],
    protocol: Mapping[str, Any],
) -> dict[str, Any]:
    gate = strict_gate(
        winner_row,
        current_row,
        protocol,
        gate_key="sensitivity_gate_1ms",
    )
    return {
        "ok": bool(gate["ok"]),
        "gate": gate,
        "evaluated_pair_ids": [
            str(winner_row["candidate_id"]),
            CURRENT_COMPARATOR_ID,
        ],
        "same_gate_semantics_as_10ms": True,
        "can_convert_10ms_fail_to_pass": False,
    }


def _write_csv(path: Path, rows: Sequence[Mapping[str, Any]]) -> None:
    fields = sorted({str(key) for row in rows for key in row})
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def _plot(path: Path, rows: Sequence[Mapping[str, Any]], winner: str | None) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    selectable = sorted(
        (row for row in rows if bool(row.get("selectable"))),
        key=lambda row: float(row["forefoot_protrusion_mm"]),
    )
    x = [float(row["forefoot_protrusion_mm"]) for row in selectable]
    fig, axes = plt.subplots(4, 1, figsize=(10, 10), sharex=True)
    metrics = (
        ("transfer_both_latches_off_sample_count", "both-off samples", 0.0),
        ("to_candidates_before_min_stance_count", "early TO candidates", 0.0),
        ("max_abs_hs_error_s", "max |HS error| [s]", 0.05),
        ("max_abs_toe_off_error_s", "max |TO error| [s]", 0.08),
    )
    for axis, (metric, ylabel, threshold) in zip(axes, metrics):
        axis.plot(
            x,
            [float(row[metric]) for row in selectable],
            marker="o",
            color="#4C78A8",
        )
        axis.axhline(threshold, color="#E45756", linestyle="--", alpha=0.8)
        axis.set_ylabel(ylabel)
        axis.grid(alpha=0.25)
        if winner is not None:
            selected = next(row for row in selectable if row["candidate_id"] == winner)
            axis.scatter(
                [float(selected["forefoot_protrusion_mm"])],
                [float(selected[metric])],
                marker="*",
                s=170,
                color="gold",
                edgecolor="black",
                zorder=5,
            )
    axes[-1].set_xlabel("forefoot protrusion [mm]")
    fig.suptitle("Two-sensor V5 X+2 depth micro-sweep — prescribed 50–100 s")
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
    primary_inputs, primary_access = v4.sample_streams_once(
        protocol,
        base,
        candidates,
        sample_dt_s=PRIMARY_DT_S,
        expected_detector_spheres=11,
        expected_total_spheres=19,
    )
    primary_rows: list[dict[str, Any]] = []
    primary_details: dict[str, Any] = {}
    for candidate in candidates:
        row, detail = evaluate_placement(
            protocol,
            candidate,
            primary_inputs[candidate.candidate_id],
            sample_dt_s=PRIMARY_DT_S,
        )
        primary_rows.append(row)
        primary_details[candidate.candidate_id] = detail
    winner_id, selection = select_10ms_winner(primary_rows, protocol)

    sensitivity_access: dict[str, Any] | None = None
    sensitivity_rows: dict[str, dict[str, Any]] = {}
    sensitivity_details: dict[str, Any] = {}
    sensitivity_assessment: dict[str, Any] | None = None
    if winner_id is not None:
        winner, current = v4.sensitivity_pair(candidates, winner_id)
        sensitivity_inputs, sensitivity_access = v4.sample_streams_once(
            protocol,
            base,
            [winner, current],
            sample_dt_s=SENSITIVITY_DT_S,
            expected_detector_spheres=4,
            expected_total_spheres=12,
        )
        for candidate in (winner, current):
            row, detail = evaluate_placement(
                protocol,
                candidate,
                sensitivity_inputs[candidate.candidate_id],
                sample_dt_s=SENSITIVITY_DT_S,
            )
            sensitivity_rows[candidate.candidate_id] = row
            sensitivity_details[candidate.candidate_id] = detail
        sensitivity_assessment = assess_1ms(
            sensitivity_rows[winner_id],
            sensitivity_rows[CURRENT_COMPARATOR_ID],
            protocol,
        )

    output_dir.mkdir(parents=True, exist_ok=False)
    plot_dir.mkdir(parents=True, exist_ok=False)
    primary_csv = output_dir / "timing_placement_v5_primary_10ms_metrics.csv"
    plot_path = plot_dir / "timing_placement_v5_primary_10ms.png"
    _write_csv(primary_csv, primary_rows)
    _plot(plot_path, primary_rows, winner_id)
    sensitivity_csv: Path | None = None
    if sensitivity_rows:
        sensitivity_csv = output_dir / "winner_current_sensitivity_1ms_metrics.csv"
        _write_csv(sensitivity_csv, list(sensitivity_rows.values()))

    ok = bool(
        winner_id is not None
        and sensitivity_assessment is not None
        and sensitivity_assessment["ok"]
    )
    conclusion = (
        "NO_STRICT_10MS_WINNER_NO_1MS_RUN"
        if winner_id is None
        else (
            "LOCKED_10MS_WINNER_PASSES_SAME_GATE_AT_1MS"
            if ok
            else "LOCKED_10MS_WINNER_FAILS_SAME_GATE_AT_1MS"
        )
    )
    manifest = {
        "schema_version": 5,
        "status": "PASS" if ok else "FAIL",
        "ok": ok,
        "stage": "development_depth_micro_with_conditional_sensitivity",
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
            "primary_10ms": primary_access,
            "conditional_winner_current_1ms": sensitivity_access,
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
                "heel_anterior_x_delta_mm": candidate.geometry.get(
                    "heel_anterior_x_delta_mm"
                ),
                "forefoot_protrusion_mm": candidate.forefoot_protrusion_mm,
                "sensor_count": 2,
                "geometry": dict(candidate.geometry),
            }
            for candidate in candidates
        ],
        "primary_10ms": {
            "rows": primary_rows,
            "details": primary_details,
            "selection": selection,
        },
        "conditional_winner_current_1ms": {
            "executed": winner_id is not None,
            "evaluated_only_after_strict_10ms_winner": True,
            "rows": sensitivity_rows,
            "details": sensitivity_details,
            "assessment": sensitivity_assessment,
        },
        "selected_pair": {
            "candidate_id": winner_id,
            "passed_10ms": winner_id is not None,
            "passed_1ms": bool(
                sensitivity_assessment and sensitivity_assessment["ok"]
            ),
            "profile_created": False,
            "promotable": False,
            "requires_future_sealed_holdout": True,
        },
        "conclusion": conclusion,
        "artifacts": {
            "primary_metrics_csv": v1._source_record(primary_csv),
            "primary_plot": v1._source_record(plot_path),
            "sensitivity_metrics_csv": (
                v1._source_record(sensitivity_csv)
                if sensitivity_csv is not None
                else None
            ),
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
        description="Run preregistered two-sensor timing-placement V5."
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
                    "schema_version": 5,
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
                    "schema_version": 5,
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
