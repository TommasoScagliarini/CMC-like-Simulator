"""Preregistered V2 micro-sweep for two virtual gait-event sensors.

Every selectable geometry contains exactly one heel sphere and one forefoot
sphere.  The heel and forefoot longitudinal placement are frozen from the
closest physical-continuity result of V1; only the forefoot plantar protrusion
is varied by at most 2 mm.  All five candidates plus the two nonselectable
comparators are sampled directly at 10 ms.  A 1 ms replay evaluates only the
shallowest strict 10 ms winner and the current comparator needed by the frozen
V3 gate.

This is development on the already-open 50 <= t < 100 s block.  It does not
train a policy, modify the production FSM, tune detector thresholds, promote a
runtime profile, or access the sealed 100--155.045 s block.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
import traceback
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
VALIDATION_ROOT = REPO_ROOT / "validation"
for path in (REPO_ROOT, VALIDATION_ROOT):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

import sweep_two_sensor_mesh_placements_prescribed as v1  # noqa: E402


DEFAULT_PROTOCOL = (
    VALIDATION_ROOT / "two_sensor_mesh_placement_sweep_protocol_v2.json"
)
DEFAULT_OUTPUT_DIR = (
    VALIDATION_ROOT
    / "two_sensor_mesh_placement_sweep_runs/"
    "2026-07-22_ab06_50_100_depth_micro_v2"
)
DEFAULT_PLOT_DIR = (
    REPO_ROOT / "plot/07_22_2026_two_sensor_mesh_placement_depth_micro_v2"
)
PROTOCOL_ID = "AB06_TWO_SENSOR_MESH_PLACEMENT_DEVELOPMENT_2026-07-22_V2"
PROTRUSIONS_MM = (30.0, 30.5, 31.0, 31.5, 32.0)
HEEL_OFFSET_MM = 2.0
FOREFOOT_FRACTION = 0.70
NUMERIC_TOLERANCE_S = 1e-12
PHYSICAL_PRE_GATE = {
    "maximum_heel_off_to_forefoot_gap_s": 0.010,
    "maximum_interior_both_off_gap_s": 0.010,
    "require_forefoot_present_every_stance": True,
    "require_forefoot_off_observed_every_swing": True,
    "require_forefoot_off_before_next_hs": True,
    "require_exact_event_counts_order_and_cycles": True,
    "maximum_invalid_or_timeout_transitions": 0,
    "maximum_unaccepted_sensor_events": 0,
}


class ProtocolError(v1.ProtocolError):
    """Raised when the V2 protocol or frozen lineage drifts."""


class NoClobberError(RuntimeError):
    """Raised before any write when an output location is already occupied."""


def _occupied(path: Path) -> bool:
    return path.exists() and (not path.is_dir() or any(path.iterdir()))


def _preflight_no_clobber(output_dir: Path, plot_dir: Path) -> None:
    occupied = [path for path in (output_dir, plot_dir) if _occupied(path)]
    if occupied:
        joined = ", ".join(v1._portable_path(path) for path in occupied)
        raise NoClobberError(f"refusing to modify occupied path(s): {joined}")


def _require_float_list(
    observed: Any,
    expected: Sequence[float],
    label: str,
) -> None:
    values = [v1._finite(value, label) for value in observed or []]
    if values != [float(value) for value in expected]:
        raise ProtocolError(f"{label} drifted: {values}")


def load_and_validate_protocol(
    path: str | Path = DEFAULT_PROTOCOL,
) -> dict[str, Any]:
    protocol_path = v1.resolve_repo_path(path).resolve()
    try:
        raw = json.loads(protocol_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ProtocolError(f"cannot load protocol: {protocol_path}") from exc
    if not isinstance(raw, dict) or raw.get("schema_version") != 2:
        raise ProtocolError("unsupported V2 placement-sweep schema")
    if raw.get("protocol_id") != PROTOCOL_ID:
        raise ProtocolError("unexpected V2 protocol id")
    if raw.get("frozen_before_execution") is not True:
        raise ProtocolError("V2 protocol must be frozen before execution")
    if raw.get("stage") != "development":
        raise ProtocolError("V2 is development-only")
    expected_profile_paths = {
        v1.CURRENT_PROFILE_ID: (
            "online_grf_profiles/"
            "AB06_SEASEA_stiff321_500_pi_grf_detector_HS-TO.json"
        ),
        v1.V4_PROFILE_ID: (
            "online_grf_profiles/"
            "AB06_SEASEA_stiff321_500_pi_grf_detector_HS-TO_"
            "v4_mesh_experimental.json"
        ),
    }
    if raw.get("profile_paths") != expected_profile_paths:
        raise ProtocolError("current/V4 profile-path mapping drifted")

    access = raw.get("data_access")
    if not isinstance(access, dict):
        raise ProtocolError("data_access is required")
    if access.get("already_open_block_s") != [v1.BLOCK_START_S, v1.SEALED_START_S]:
        raise ProtocolError("only the already-open 50--100 s block is allowed")
    if access.get("upper_bound_is_exclusive") is not True:
        raise ProtocolError("the 100 s boundary must remain exclusive")
    if access.get("allow_samples_at_or_after_100_s") is not False:
        raise ProtocolError("samples at or after 100 s are forbidden")
    if access.get("sealed_block_s") != [v1.SEALED_START_S, v1.SEALED_END_S]:
        raise ProtocolError("sealed block drifted")
    common = access.get("expected_common_cycle_set")
    expected_common = {
        "original_complete_cycles": 51,
        "retained_complete_cycles": 50,
        "reference_hs_count": 51,
        "reference_to_count": 50,
        "excluded_closing_hs_s": [99.96878691565038],
    }
    if not isinstance(common, dict):
        raise ProtocolError("common-cycle lineage is required")
    for key, expected in expected_common.items():
        if common.get(key) != expected:
            raise ProtocolError(f"common-cycle field drifted: {key}")

    replay = raw.get("replay")
    if not isinstance(replay, dict):
        raise ProtocolError("replay is required")
    frozen_replay = {
        "sensor_on_threshold_n": v1.SENSOR_ON_N,
        "sensor_off_threshold_n": v1.SENSOR_OFF_N,
        "sensor_dwell_s": v1.SENSOR_DWELL_S,
        "primary_sample_dt_s": v1.PRIMARY_DT_S,
        "winner_sensitivity_sample_dt_s": v1.SENSITIVITY_DT_S,
    }
    for key, expected in frozen_replay.items():
        if v1._finite(replay.get(key), key) != expected:
            raise ProtocolError(f"frozen replay value drifted: {key}")
    required_replay = {
        "event_source": "two_sensor",
        "primary_event_time_field": "confirmed_time_s",
        "diagnostic_event_time_field": "event_time_s",
        "phase_reference_mode": "validated_event_intervals",
        "primary_evaluate_all_candidates": True,
        "sensitivity_requires_strict_10ms_winner": True,
        "sensitivity_evaluates_selected_and_current_only": True,
    }
    for key, expected in required_replay.items():
        if replay.get(key) != expected:
            raise ProtocolError(f"frozen replay semantic drifted: {key}")

    grid = raw.get("placement_grid")
    if not isinstance(grid, dict):
        raise ProtocolError("placement_grid is required")
    _require_float_list(
        grid.get("heel_vertical_offsets_below_current_mm"),
        [HEEL_OFFSET_MM],
        "heel offset",
    )
    _require_float_list(
        grid.get("forefoot_longitudinal_fractions_mesh_x"),
        [FOREFOOT_FRACTION],
        "forefoot fraction",
    )
    _require_float_list(
        grid.get("forefoot_absolute_local_plantar_protrusion_mm"),
        PROTRUSIONS_MM,
        "forefoot protrusion",
    )
    expected_grid_fields = {
        "heel_offset_rule": "local_vertical_below_current_center",
        "x_rule": "mesh_bbox_longitudinal_fraction",
        "z_rule": "midpoint_of_mesh_section_at_candidate_x",
        "forefoot_height_rule": (
            "absolute_sphere_bottom_below_local_plantar_surface"
        ),
        "candidate_count": len(PROTRUSIONS_MM),
        "sensors_per_candidate": 2,
        "maximum_absolute_heel_forefoot_bottom_offset_mm": 20.0,
    }
    for key, expected in expected_grid_fields.items():
        if grid.get(key) != expected:
            raise ProtocolError(f"placement-grid field drifted: {key}")

    selection = raw.get("selection")
    if not isinstance(selection, dict):
        raise ProtocolError("selection is required")
    if selection.get("physical_continuity_pre_gate") != PHYSICAL_PRE_GATE:
        raise ProtocolError("physical continuity pre-gate drifted")
    required_selection = {
        "strict_gate": "V3_confirmed_time_holdout_gate",
        "winner_rule": "shallowest_full_pass_at_10ms",
        "depth_order_mm": list(PROTRUSIONS_MM),
        "release_failure_censors_candidate_and_all_deeper": True,
        "no_strict_winner_policy": (
            "fail_depth_micro_adjustment_without_1ms_or_diagnostic_winner"
        ),
        "one_ms_cannot_convert_10ms_fail_to_pass": True,
        "gap_monotonicity_role": "diagnostic_not_acceptance_gate",
    }
    for key, expected in required_selection.items():
        if selection.get(key) != expected:
            raise ProtocolError(f"selection field drifted: {key}")
    if v1._finite(
        selection.get("numeric_comparison_abs_tolerance_s"),
        "numeric comparison tolerance",
    ) != NUMERIC_TOLERANCE_S:
        raise ProtocolError("numeric comparison tolerance drifted")

    if raw.get("comparators") != {
        v1.CURRENT_PROFILE_ID: True,
        v1.V4_PROFILE_ID: True,
    }:
        raise ProtocolError("current and rejected V4 comparators are required")
    gate = raw.get("sealed_validation_gate")
    if not isinstance(gate, dict):
        raise ProtocolError("frozen V3 gate is required")
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
        if v1._finite(gate.get(key), f"sealed_validation_gate.{key}") != expected:
            raise ProtocolError(f"V3 strict gate drifted: {key}")

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
            raise ProtocolError(
                f"source hash drift for {label}: {observed} != "
                f"{record.get('sha256')}"
            )

    raw["_protocol_path"] = protocol_path.as_posix()
    raw["_protocol_sha256"] = v1._sha256(protocol_path)
    raw["_primary_event_time_field"] = "confirmed_time_s"
    raw["_diagnostic_event_time_field"] = "event_time_s"
    raw["_phase_reference_mode"] = "validated_event_intervals"
    return raw


def _candidate_id(protrusion_mm: float) -> str:
    token = f"{float(protrusion_mm):.1f}".replace(".", "p")
    return f"H02_F70_P{token}"


def build_placement_candidates(
    protocol: Mapping[str, Any],
) -> tuple[v1.OnlineGRFProfile, list[v1.PlacementCandidate], dict[str, Any]]:
    base_path = v1.resolve_repo_path(
        str(protocol["profile_paths"][v1.CURRENT_PROFILE_ID])
    ).resolve()
    v4_path = v1.resolve_repo_path(
        str(protocol["profile_paths"][v1.V4_PROFILE_ID])
    ).resolve()
    base = v1.load_online_grf_profile(base_path)
    rejected_v4 = v1.load_online_grf_profile(v4_path)
    base_sensors = v1._left_sensor_spheres(base)
    v4_sensors = v1._left_sensor_spheres(rejected_v4)
    if base_sensors["left_heel"] != v4_sensors["left_heel"]:
        raise ProtocolError("current and V4 profiles must share the heel")

    setup = v1.read_setup_xml(
        v1.resolve_repo_path(str(protocol["setup"])).resolve()
    )
    mesh_path = v1._resolve_left_foot_mesh(setup.model_file.resolve())
    triangles = v1._load_stl_triangles(mesh_path)
    heel_template = base_sensors["left_heel"]
    forefoot_template = base_sensors["left_toe"]
    if not math.isclose(
        heel_template.radius,
        forefoot_template.radius,
        rel_tol=0.0,
        abs_tol=1e-12,
    ):
        raise ProtocolError("heel and forefoot radii must remain equal")
    heel_location = (
        float(heel_template.location[0]),
        float(heel_template.location[1] - HEEL_OFFSET_MM / 1000.0),
        float(heel_template.location[2]),
    )
    radius = float(heel_template.radius)
    heel_bottom_y = float(heel_location[1] - radius)
    candidates: list[v1.PlacementCandidate] = []
    bottom_offsets: list[float] = []
    for protrusion_mm in PROTRUSIONS_MM:
        forefoot_location, geometry = v1._derive_forefoot_location(
            triangles,
            forefoot_template,
            fraction=FOREFOOT_FRACTION,
            protrusion_mm=protrusion_mm,
        )
        forefoot_bottom_y = float(forefoot_location[1] - radius)
        signed_bottom_offset_mm = 1000.0 * (
            forefoot_bottom_y - heel_bottom_y
        )
        if abs(signed_bottom_offset_mm) > 20.0 + 1e-9:
            raise ProtocolError("candidate exceeds the 20 mm bottom-offset limit")
        bottom_offsets.append(signed_bottom_offset_mm)
        candidates.append(
            v1.PlacementCandidate(
                candidate_id=_candidate_id(protrusion_mm),
                heel_location=heel_location,
                forefoot_location=forefoot_location,
                heel_offset_below_current_mm=HEEL_OFFSET_MM,
                forefoot_fraction_mesh_x=FOREFOOT_FRACTION,
                forefoot_protrusion_mm=protrusion_mm,
                selectable=True,
                role="development_depth_micro_candidate",
                geometry={
                    **geometry,
                    "heel_location_m": list(heel_location),
                    "heel_bottom_y_m": heel_bottom_y,
                    "forefoot_bottom_y_m": forefoot_bottom_y,
                    "signed_forefoot_minus_heel_bottom_offset_mm": (
                        signed_bottom_offset_mm
                    ),
                    "absolute_bottom_offset_within_20mm": True,
                    "derived_directly_from_mesh": True,
                },
            )
        )

    candidates.extend(
        [
            v1.PlacementCandidate(
                candidate_id=v1.CURRENT_PROFILE_ID,
                heel_location=tuple(base_sensors["left_heel"].location),
                forefoot_location=tuple(base_sensors["left_toe"].location),
                heel_offset_below_current_mm=None,
                forefoot_fraction_mesh_x=None,
                forefoot_protrusion_mm=None,
                selectable=False,
                role="nonselectable_comparator",
                geometry={"source": "current_profile"},
            ),
            v1.PlacementCandidate(
                candidate_id=v1.V4_PROFILE_ID,
                heel_location=tuple(v4_sensors["left_heel"].location),
                forefoot_location=tuple(v4_sensors["left_toe"].location),
                heel_offset_below_current_mm=None,
                forefoot_fraction_mesh_x=None,
                forefoot_protrusion_mm=None,
                selectable=False,
                role="nonselectable_rejected_comparator",
                geometry={"source": "rejected_v4_profile"},
            ),
        ]
    )
    ids = [candidate.candidate_id for candidate in candidates]
    if len(ids) != len(set(ids)) or len(candidates) != 7:
        raise ProtocolError("V2 must contain five candidates and two comparators")
    return base, candidates, {
        "mesh": v1._source_record(mesh_path),
        "heel_location_m": list(heel_location),
        "heel_radius_m": radius,
        "heel_bottom_y_m": heel_bottom_y,
        "selectable_candidate_count": 5,
        "total_pair_count_including_comparators": 7,
        "direct_unique_sphere_sampling": True,
        "affine_reconstruction_used": False,
        "signed_forefoot_minus_heel_bottom_offsets_mm": bottom_offsets,
        "all_absolute_bottom_offsets_within_20mm": True,
    }


def _gap_monotonicity(rows: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    ordered = sorted(rows, key=lambda row: float(row["forefoot_protrusion_mm"]))
    gaps = [float(row["maximum_heel_off_to_forefoot_gap_s"]) for row in ordered]
    nonincreasing = all(
        later <= earlier + NUMERIC_TOLERANCE_S
        for earlier, later in zip(gaps, gaps[1:])
    )
    return {
        "role": "diagnostic_not_acceptance_gate",
        "protrusions_mm": [float(row["forefoot_protrusion_mm"]) for row in ordered],
        "maximum_heel_off_to_forefoot_gap_s": gaps,
        "monotonic_nonincreasing": bool(nonincreasing),
        "any_candidate_at_or_below_10ms": bool(
            any(gap <= 0.010 + NUMERIC_TOLERANCE_S for gap in gaps)
        ),
    }


def physical_continuity_pre_gate(
    row: Mapping[str, Any],
    protocol: Mapping[str, Any],
) -> dict[str, Any]:
    """Apply the frozen physical gate with float-only boundary tolerance."""
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
        <= float(gate["maximum_heel_off_to_forefoot_gap_s"])
        + NUMERIC_TOLERANCE_S,
        "interior_both_off_gap": float(row["maximum_interior_both_off_gap_s"])
        <= float(gate["maximum_interior_both_off_gap_s"])
        + NUMERIC_TOLERANCE_S,
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
    return {
        "ok": bool(all(checks.values())),
        "checks": checks,
        "numeric_comparison_abs_tolerance_s": NUMERIC_TOLERANCE_S,
    }


def build_sensitivity_assessment(
    selected_row: Mapping[str, Any],
    current_row: Mapping[str, Any],
    protocol: Mapping[str, Any],
) -> dict[str, Any]:
    physical = physical_continuity_pre_gate(selected_row, protocol)
    v3_gate = v1.evaluate_holdout_gate(selected_row, current_row, protocol)
    return {
        "role": "gating_strict_winner",
        "physical_continuity_pre_gate": physical,
        "v3_gate": v3_gate,
        "can_convert_10ms_fail_to_pass": False,
        "ok": bool(physical["ok"] and v3_gate["ok"]),
    }


def select_development_outcome(
    rows: Sequence[Mapping[str, Any]],
    protocol: Mapping[str, Any],
) -> tuple[str | None, dict[str, Any]]:
    current_rows = [row for row in rows if row["candidate_id"] == v1.CURRENT_PROFILE_ID]
    if len(current_rows) != 1:
        raise ProtocolError("exactly one current comparator row is required")
    current = current_rows[0]
    selectable = sorted(
        (row for row in rows if row.get("selectable")),
        key=lambda row: float(row["forefoot_protrusion_mm"]),
    )
    observed_depths = [float(row["forefoot_protrusion_mm"]) for row in selectable]
    if observed_depths != list(PROTRUSIONS_MM):
        raise ProtocolError("V2 selection requires the frozen five-depth grid")

    assessed: list[dict[str, Any]] = []
    release_cutoff_depth_mm: float | None = None
    eligible: list[Mapping[str, Any]] = []
    for row in selectable:
        depth = float(row["forefoot_protrusion_mm"])
        release_failure = bool(
            int(row["swings_without_forefoot_off_count"]) > 0
            or int(row["forefoot_contact_crossing_next_hs_count"]) > 0
        )
        if release_cutoff_depth_mm is None and release_failure:
            release_cutoff_depth_mm = depth
        allowed_by_release_cutoff = bool(
            release_cutoff_depth_mm is None or depth < release_cutoff_depth_mm
        )
        physical = physical_continuity_pre_gate(row, protocol)
        v3_gate = v1.evaluate_holdout_gate(row, current, protocol)
        strict_eligible = bool(
            allowed_by_release_cutoff and physical["ok"] and v3_gate["ok"]
        )
        if strict_eligible:
            eligible.append(row)
        assessed.append(
            {
                "candidate_id": row["candidate_id"],
                "forefoot_protrusion_mm": depth,
                "physical_continuity_pre_gate": physical,
                "v3_gate": v3_gate,
                "release_failure": release_failure,
                "allowed_by_release_cutoff": allowed_by_release_cutoff,
                "strict_eligible": strict_eligible,
            }
        )
    monotonicity = _gap_monotonicity(selectable)
    if eligible:
        winner = min(eligible, key=lambda row: float(row["forefoot_protrusion_mm"]))
        selected_id = str(winner["candidate_id"])
        status = "SHALLOWEST_STRICT_WINNER_LOCKED_AT_10MS"
    else:
        selected_id = None
        status = "NO_STRICT_WINNER_DEPTH_MICRO_ADJUSTMENT_FAILED"
    return selected_id, {
        "status": status,
        "strict_winner_id": selected_id,
        "winner_rule": "shallowest_full_pass_at_10ms",
        "release_cutoff_depth_mm": release_cutoff_depth_mm,
        "gap_monotonicity": monotonicity,
        "candidate_gates": assessed,
        "diagnostic_winner_id": None,
    }


def _write_rows_csv(path: Path, rows: Sequence[Mapping[str, Any]]) -> None:
    fieldnames = sorted({str(key) for row in rows for key in row})
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def _plot_depth_micro(
    path: Path,
    rows: Sequence[Mapping[str, Any]],
    selected_id: str | None,
) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    selectable = sorted(
        (row for row in rows if row.get("selectable")),
        key=lambda row: float(row["forefoot_protrusion_mm"]),
    )
    x = np.asarray(
        [float(row["forefoot_protrusion_mm"]) for row in selectable], dtype=float
    )
    fig, axes = plt.subplots(3, 1, figsize=(9.5, 8.5), sharex=True)
    metrics = (
        ("observed_valid_cycle_count", "valid cycles", 50.0),
        ("maximum_heel_off_to_forefoot_gap_s", "max heel→forefoot gap [s]", 0.010),
        ("confirmed_fsm_stance_iou", "confirmed FSM stance IoU", 0.90),
    )
    for axis, (metric, ylabel, threshold) in zip(axes, metrics):
        y = np.asarray([float(row[metric]) for row in selectable], dtype=float)
        axis.plot(x, y, marker="o", color="#4C78A8")
        axis.axhline(threshold, color="#E45756", linestyle="--", alpha=0.85)
        if selected_id is not None:
            winner = next(row for row in selectable if row["candidate_id"] == selected_id)
            axis.scatter(
                [float(winner["forefoot_protrusion_mm"])],
                [float(winner[metric])],
                marker="*",
                s=150,
                color="#54A24B",
                zorder=4,
                label="shallowest strict winner",
            )
            axis.legend(loc="best")
        axis.set_ylabel(ylabel)
        axis.grid(alpha=0.25)
    axes[-1].set_xlabel("forefoot protrusion below local plantar surface [mm]")
    fig.suptitle("Two-sensor depth micro-sweep — prescribed development 50–100 s")
    fig.tight_layout()
    fig.savefig(path, dpi=170)
    plt.close(fig)


def run_sweep(
    protocol: Mapping[str, Any],
    output_dir: Path,
    plot_dir: Path,
) -> dict[str, Any]:
    _preflight_no_clobber(output_dir, plot_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    plot_dir.mkdir(parents=True, exist_ok=True)

    base, candidates, geometry_summary = build_placement_candidates(protocol)
    primary_inputs, primary_access = v1._sample_all_candidates(
        protocol,
        base,
        candidates,
        sample_dt_s=v1.PRIMARY_DT_S,
    )
    primary_rows: list[dict[str, Any]] = []
    primary_details: dict[str, Any] = {}
    for candidate in candidates:
        row, detail = v1._evaluate_placement(
            protocol,
            candidate,
            primary_inputs[candidate.candidate_id],
            sample_dt_s=v1.PRIMARY_DT_S,
        )
        primary_rows.append(row)
        primary_details[candidate.candidate_id] = detail

    selected_id, selection = select_development_outcome(primary_rows, protocol)
    sensitivity_access: dict[str, Any] | None = None
    sensitivity_rows: dict[str, dict[str, Any]] = {}
    sensitivity_details: dict[str, dict[str, Any]] = {}
    sensitivity_gate: dict[str, Any] | None = None
    if selected_id is not None:
        selected = next(
            candidate for candidate in candidates if candidate.candidate_id == selected_id
        )
        current = next(
            candidate
            for candidate in candidates
            if candidate.candidate_id == v1.CURRENT_PROFILE_ID
        )
        sensitivity_inputs, sensitivity_access = v1._sample_all_candidates(
            protocol,
            base,
            [selected, current],
            sample_dt_s=v1.SENSITIVITY_DT_S,
        )
        for candidate in (selected, current):
            row, detail = v1._evaluate_placement(
                protocol,
                candidate,
                sensitivity_inputs[candidate.candidate_id],
                sample_dt_s=v1.SENSITIVITY_DT_S,
            )
            sensitivity_rows[candidate.candidate_id] = row
            sensitivity_details[candidate.candidate_id] = detail
        sensitivity_gate = build_sensitivity_assessment(
            sensitivity_rows[selected_id],
            sensitivity_rows[v1.CURRENT_PROFILE_ID],
            protocol,
        )

    rows_path = output_dir / "placement_depth_micro_primary_10ms_metrics.csv"
    _write_rows_csv(rows_path, primary_rows)
    plot_path = plot_dir / "placement_depth_micro_primary_10ms.png"
    _plot_depth_micro(plot_path, primary_rows, selected_id)
    sensitivity_path: Path | None = None
    if sensitivity_rows:
        sensitivity_path = output_dir / "winner_plus_current_sensitivity_1ms_metrics.csv"
        _write_rows_csv(sensitivity_path, list(sensitivity_rows.values()))

    ok = bool(selected_id is not None and sensitivity_gate and sensitivity_gate["ok"])
    if ok:
        conclusion = "SHALLOWEST_DEPTH_WINNER_PASSES_10MS_AND_1MS"
    elif selected_id is not None:
        conclusion = "SHALLOWEST_10MS_WINNER_FAILED_1MS_GATE"
    elif not selection["gap_monotonicity"]["monotonic_nonincreasing"]:
        conclusion = "DEPTH_RESPONSE_NON_MONOTONIC_NO_WINNER"
    else:
        conclusion = "NO_STRICT_WINNER_DEPTH_MICRO_ADJUSTMENT_FAILED"

    manifest = {
        "schema_version": 2,
        "status": "PASS" if ok else "FAIL",
        "ok": ok,
        "stage": "development",
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
            "primary_10ms": primary_access,
            "strict_winner_plus_current_sensitivity_1ms": sensitivity_access,
        },
        "detector_contract": {
            "sensors_per_candidate": 2,
            "sensor_roles": ["heel", "forefoot"],
            "sensor_on_threshold_n": v1.SENSOR_ON_N,
            "sensor_off_threshold_n": v1.SENSOR_OFF_N,
            "sensor_dwell_s": v1.SENSOR_DWELL_S,
            "event_source": "two_sensor",
            "primary_event_time_field": "confirmed_time_s",
        },
        "geometry": geometry_summary,
        "candidates": [
            {
                "candidate_id": candidate.candidate_id,
                "selectable": candidate.selectable,
                "role": candidate.role,
                "heel_location_m": list(candidate.heel_location),
                "forefoot_location_m": list(candidate.forefoot_location),
                "forefoot_protrusion_mm": candidate.forefoot_protrusion_mm,
                "geometry": dict(candidate.geometry),
            }
            for candidate in candidates
        ],
        "primary_10ms": {
            "rows": primary_rows,
            "details": primary_details,
            "selection": selection,
            "continuity_helper_role_override": (
                "regional continuity metrics are gating in this V2 protocol, "
                "despite the imported V1 helper's historical diagnostic label"
            ),
        },
        "winner_plus_current_sensitivity_1ms": {
            "rows": sensitivity_rows,
            "details": sensitivity_details,
            "gate": sensitivity_gate,
            "evaluated_only_after_strict_10ms_winner": True,
        },
        "selected_pair": {
            "candidate_id": selected_id,
            "promotable_from_development": ok,
            "promotion_requires_future_holdout": True,
        },
        "conclusion": conclusion,
        "artifacts": {
            "primary_metrics_csv": v1._source_record(rows_path),
            "primary_plot": v1._source_record(plot_path),
            "sensitivity_metrics_csv": (
                v1._source_record(sensitivity_path)
                if sensitivity_path is not None
                else None
            ),
        },
        "non_actions": {
            "policy_or_training_run": False,
            "runtime_configuration_modified": False,
            "fsm_modified": False,
            "current_or_v4_profile_modified": False,
            "candidate_profile_created_or_promoted": False,
            "sealed_block_opened": False,
        },
        "interpretation_limits": protocol["interpretation_limits"],
    }
    safe = v1._json_safe(manifest)
    (output_dir / "manifest.json").write_text(
        json.dumps(safe, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    return safe


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run the preregistered AB06 two-sensor depth micro-sweep V2."
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
                    "schema_version": 2,
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
    except NoClobberError as exc:  # race-safe second preflight
        print(
            json.dumps(
                {
                    "schema_version": 2,
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
    except Exception as exc:  # pragma: no cover - CLI fail-closed path
        output_dir.mkdir(parents=True, exist_ok=True)
        failure = {
            "schema_version": 2,
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
        "manifest": v1._portable_path(output_dir / "manifest.json"),
    }
    print(json.dumps(concise, indent=2, allow_nan=False))
    return 0 if manifest["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
