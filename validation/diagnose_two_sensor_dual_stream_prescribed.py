"""Causal A/B diagnostic for two-sensor gait events and load evidence.

The two detector spheres remain the *only* source of heel-strike/toe-off
guards in both branches.  Branch A also uses their aggregate force and union
penetration as the FSM stance-load evidence, matching the historical replay.
Branch B instead supplies stance-load evidence from the hash-pinned primary
online-GRF profile while leaving the detector streams byte-for-byte identical.

This is a development diagnostic on the already-open ``50 <= t < 100 s``
block.  It has no geometry acceptance gate, does not select or promote a
profile, does not train a policy, and never opens the sealed block.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import sys
import traceback
from collections import Counter
from dataclasses import replace
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
VALIDATION_ROOT = REPO_ROOT / "validation"
TRAJECTORY_ROOT = REPO_ROOT / "Trajectory Generator"
for path in (REPO_ROOT, VALIDATION_ROOT, TRAJECTORY_ROOT):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

import sweep_two_sensor_mesh_placements_prescribed as v1  # noqa: E402
import sweep_two_sensor_mesh_placements_prescribed_v2 as v2  # noqa: E402
from online_grf import OnlineGRFProfile, load_online_grf_profile  # noqa: E402
from sweep_two_sensor_prescribed_thresholds import (  # noqa: E402
    Candidate,
    _evaluate_candidate,
)
from validate_online_grf import _calculate_wrench  # noqa: E402


DEFAULT_PROTOCOL = (
    VALIDATION_ROOT / "two_sensor_dual_stream_prescribed_protocol.json"
)
DEFAULT_OUTPUT_DIR = (
    VALIDATION_ROOT
    / "two_sensor_dual_stream_diagnostic_runs/"
    "2026-07-22_ab06_50_100_h02_f70_p315_p320"
)
DEFAULT_PLOT_DIR = (
    REPO_ROOT / "plot/07_22_2026_two_sensor_dual_stream_causal_diagnostic"
)
PROTOCOL_ID = "AB06_TWO_SENSOR_DUAL_STREAM_CAUSAL_DIAGNOSTIC_2026-07-22_V1"
CANDIDATE_IDS = ("H02_F70_P31p5", "H02_F70_P32p0")
BRANCH_IDS = ("A_detector_load", "B_primary_load")
LOAD_EVIDENCE_PROFILE = (
    "online_grf_profiles/"
    "AB06_SEASEA_stiff321_500_pi_grf_correct_magnitude.json"
)


class ProtocolError(ValueError):
    """Raised before OpenSim sampling when the frozen diagnostic drifts."""


class NoClobberError(RuntimeError):
    """Raised before writes when an output location is already occupied."""


def _occupied(path: Path) -> bool:
    return path.exists() and (not path.is_dir() or any(path.iterdir()))


def _preflight_no_clobber(output_dir: Path, plot_dir: Path) -> None:
    occupied = [path for path in (output_dir, plot_dir) if _occupied(path)]
    if occupied:
        joined = ", ".join(v1._portable_path(path) for path in occupied)
        raise NoClobberError(f"refusing to modify occupied path(s): {joined}")


def _canonical_hash(payload: Any) -> str:
    encoded = json.dumps(
        v1._json_safe(payload),
        sort_keys=True,
        separators=(",", ":"),
        allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def load_and_validate_protocol(
    path: str | Path = DEFAULT_PROTOCOL,
) -> dict[str, Any]:
    protocol_path = v1.resolve_repo_path(path).resolve()
    try:
        raw = json.loads(protocol_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ProtocolError(f"cannot load protocol: {protocol_path}") from exc
    if not isinstance(raw, dict) or raw.get("schema_version") != 1:
        raise ProtocolError("unsupported dual-stream protocol schema")
    expected_top_level = {
        "protocol_id": PROTOCOL_ID,
        "frozen_before_execution": True,
        "stage": "development_causal_diagnostic",
        "setup": "models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi_setup.xml",
        "load_evidence_profile": LOAD_EVIDENCE_PROFILE,
        "candidate_ids": list(CANDIDATE_IDS),
    }
    for key, expected in expected_top_level.items():
        if raw.get(key) != expected:
            raise ProtocolError(f"frozen protocol field drifted: {key}")

    expected_profiles = {
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
    if raw.get("profile_paths") != expected_profiles:
        raise ProtocolError("detector profile paths drifted")

    access = raw.get("data_access")
    expected_common = {
        "original_complete_cycles": 51,
        "retained_complete_cycles": 50,
        "reference_hs_count": 51,
        "reference_to_count": 50,
        "excluded_closing_hs_s": [99.96878691565038],
    }
    if not isinstance(access, dict):
        raise ProtocolError("data_access is required")
    if access.get("already_open_block_s") != [v1.BLOCK_START_S, v1.SEALED_START_S]:
        raise ProtocolError("only the already-open 50--100 s block is allowed")
    if access.get("upper_bound_is_exclusive") is not True:
        raise ProtocolError("100 s must remain an exclusive boundary")
    if access.get("allow_samples_at_or_after_100_s") is not False:
        raise ProtocolError("sampling at or after 100 s is forbidden")
    if access.get("sealed_block_s") != [v1.SEALED_START_S, v1.SEALED_END_S]:
        raise ProtocolError("sealed block drifted")
    common = access.get("expected_common_cycle_set")
    if not isinstance(common, dict):
        raise ProtocolError("common-cycle lineage is required")
    for key, expected in expected_common.items():
        if common.get(key) != expected:
            raise ProtocolError(f"common-cycle field drifted: {key}")

    replay = raw.get("replay")
    if not isinstance(replay, dict):
        raise ProtocolError("replay configuration is required")
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
        "fsm_min_stance_contact_fraction": 0.20,
        "fsm_min_stance_load_bw_s": 0.04,
    }
    for key, expected in expected_replay.items():
        value = replay.get(key)
        if isinstance(expected, float):
            try:
                matches = math.isclose(
                    float(value), expected, rel_tol=0.0, abs_tol=1e-12
                )
            except (TypeError, ValueError):
                matches = False
        else:
            matches = value == expected
        if not matches:
            raise ProtocolError(f"frozen replay field drifted: {key}")

    expected_branches = {
        "A_detector_load": {
            "heel_toe_event_guards": "candidate_two_sensor_forces",
            "normal_force_bw": "candidate_two_sensor_aggregate",
            "in_contact": "candidate_two_sensor_union_physical_penetration",
        },
        "B_primary_load": {
            "heel_toe_event_guards": "candidate_two_sensor_forces",
            "normal_force_bw": "primary_online_grf_left_aggregate",
            "in_contact": "primary_online_grf_left_union_physical_penetration",
        },
    }
    if raw.get("branches") != expected_branches:
        raise ProtocolError("A/B causal branch definition drifted")

    decision = raw.get("decision_contract")
    expected_decision = {
        "role": "causal_diagnostic_only",
        "primary_question": "does_stance_load_too_low_disappear",
        "geometry_acceptance_applied": False,
        "candidate_selection_allowed": False,
        "profile_promotion_allowed": False,
        "sealed_validation_allowed": False,
        "training_allowed": False,
    }
    if decision != expected_decision:
        raise ProtocolError("diagnostic-only decision contract drifted")

    sampling = raw.get("sampling")
    expected_sampling = {
        "method": "all_required_spheres_directly_in_one_opensim_pass",
        "affine_reconstruction": False,
        "left_primary_spheres_only": True,
        "expected_unique_detector_spheres": 3,
        "expected_primary_load_spheres": 8,
        "expected_total_unique_spheres": 11,
    }
    if sampling != expected_sampling:
        raise ProtocolError("single-pass sampling contract drifted")

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


def build_candidates(
    protocol: Mapping[str, Any],
) -> tuple[OnlineGRFProfile, list[v1.PlacementCandidate], dict[str, Any]]:
    base, all_candidates, v2_summary = v2.build_placement_candidates(protocol)
    by_id = {candidate.candidate_id: candidate for candidate in all_candidates}
    try:
        candidates = [by_id[candidate_id] for candidate_id in CANDIDATE_IDS]
    except KeyError as exc:
        raise ProtocolError(f"missing frozen candidate: {exc.args[0]}") from exc
    if any(not candidate.selectable for candidate in candidates):
        raise ProtocolError("causal candidates must originate from the V2 grid")
    observed_depths = [candidate.forefoot_protrusion_mm for candidate in candidates]
    if observed_depths != [31.5, 32.0]:
        raise ProtocolError("frozen candidate depths drifted")
    summary = {
        "derived_by": "V2 mesh placement builder",
        "candidate_count": len(candidates),
        "sensors_per_candidate": 2,
        "candidate_ids": list(CANDIDATE_IDS),
        "heel_location_m": v2_summary["heel_location_m"],
        "heel_radius_m": v2_summary["heel_radius_m"],
        "affine_reconstruction_used": False,
    }
    return base, candidates, summary


def _primary_physical_penetration(
    profile: OnlineGRFProfile,
    samples: Mapping[str, Any],
) -> np.ndarray:
    normal = np.asarray(profile.ground.normal, dtype=float)
    normal /= np.linalg.norm(normal)
    origin = np.asarray(profile.ground.origin, dtype=float)
    values: list[np.ndarray] = []
    for sphere in profile.spheres:
        center = np.asarray(samples["centers"][sphere.name], dtype=float)
        values.append(
            np.maximum(
                0.0,
                float(sphere.radius) - (center - origin) @ normal,
            )
        )
    if not values:
        raise ProtocolError("primary load profile has no left spheres")
    result = np.maximum.reduce(values)
    if result.ndim != 1 or not np.all(np.isfinite(result)):
        raise FloatingPointError("invalid primary physical penetration")
    return result


def sample_all_streams_once(
    protocol: Mapping[str, Any],
    base: OnlineGRFProfile,
    candidates: Sequence[v1.PlacementCandidate],
) -> tuple[dict[str, dict[str, Any]], dict[str, Any]]:
    setup, events, access, times = v1._reference_bundle(
        protocol, sample_dt_s=v1.PRIMARY_DT_S
    )
    detector_sampler, sphere_pairs, detector_profiles = v1._sampling_bundle(
        base, candidates
    )
    primary_path = v1.resolve_repo_path(
        str(protocol["load_evidence_profile"])
    ).resolve()
    primary_full = load_online_grf_profile(primary_path)
    primary_left_spheres = tuple(
        sphere for sphere in primary_full.spheres if sphere.side == "left"
    )
    primary_left = replace(
        primary_full,
        source="validation_primary_left_load_evidence",
        spheres=primary_left_spheres,
    )
    if len(primary_left_spheres) != int(
        protocol["sampling"]["expected_primary_load_spheres"]
    ):
        raise ProtocolError("primary left-sphere count drifted")
    combined = tuple(detector_sampler.spheres) + primary_left_spheres
    names = [sphere.name for sphere in combined]
    if len(names) != len(set(names)):
        raise ProtocolError("combined sampler sphere names are not unique")
    if len(detector_sampler.spheres) != int(
        protocol["sampling"]["expected_unique_detector_spheres"]
    ):
        raise ProtocolError("detector unique-sphere count drifted")
    if len(combined) != int(protocol["sampling"]["expected_total_unique_spheres"]):
        raise ProtocolError("combined single-pass sphere count drifted")
    sampler = replace(
        base,
        source="validation_dual_stream_single_pass_sampler",
        spheres=combined,
    )
    samples = v1._sample_spheres(
        setup,
        sampler,
        times,
        str(protocol["replay"]["sea_plugin"]),
    )

    primary_wrench = _calculate_wrench(primary_left, dict(samples))["left"]
    primary_aggregate = np.asarray(primary_wrench["normal_force"], dtype=float)
    primary_penetration = _primary_physical_penetration(primary_left, samples)
    if not np.all(np.isfinite(primary_aggregate)):
        raise FloatingPointError("non-finite primary aggregate normal force")

    kinematics = v1._prescribed_prosthetic_kinematics(setup, times)
    prescribed_vertical_n = np.asarray(
        v1._external_grf(setup, times)["left"][:, 1], dtype=float
    )
    body_weight_n = v1._model_body_weight_n(setup.model_file)
    shared = {
        "setup": setup,
        "times": times,
        "kinematics": kinematics,
        "prescribed_vertical_n": prescribed_vertical_n,
        "reference_events": events,
        "body_weight_n": body_weight_n,
        "primary_aggregate": primary_aggregate,
        "primary_penetration": primary_penetration,
    }
    result: dict[str, dict[str, Any]] = {}
    for candidate in candidates:
        loads, penetrations, aggregate = v1._contact_inputs(
            detector_profiles[candidate.candidate_id],
            sphere_pairs[candidate.candidate_id],
            samples,
        )
        result[candidate.candidate_id] = {
            **shared,
            "detector_profile": detector_profiles[candidate.candidate_id],
            "detector_loads": loads,
            "detector_penetrations": penetrations,
            "detector_aggregate": aggregate,
        }
    access.update(
        {
            "single_opensim_sphere_sampling_pass": True,
            "direct_sphere_sampling_without_affine_reconstruction": True,
            "sampled_unique_detector_sphere_count": len(detector_sampler.spheres),
            "sampled_primary_load_sphere_count": len(primary_left_spheres),
            "sampled_total_unique_sphere_count": len(combined),
            "evaluated_detector_pair_count": len(candidates),
            "load_evidence_profile": v1._source_record(primary_path),
        }
    )
    return result, access


def compose_branch_inputs(
    common: Mapping[str, Any],
    branch_id: str,
) -> tuple[dict[str, Any], dict[str, str]]:
    detector_loads = dict(common["detector_loads"])
    if branch_id == "A_detector_load":
        aggregate = np.asarray(common["detector_aggregate"], dtype=float)
        penetrations = {
            key: np.asarray(value, dtype=float)
            for key, value in common["detector_penetrations"].items()
        }
        sources = {
            "heel_toe_event_guards": "candidate_two_sensor_forces",
            "normal_force_bw": "candidate_two_sensor_aggregate",
            "in_contact": "candidate_two_sensor_union_physical_penetration",
        }
    elif branch_id == "B_primary_load":
        aggregate = np.asarray(common["primary_aggregate"], dtype=float)
        union = np.asarray(common["primary_penetration"], dtype=float)
        # The production replay API accepts two regional penetrations and ORs
        # them.  Store the already-aggregated primary union in one slot and an
        # exact zero in the other; this preserves the intended boolean stream.
        penetrations = {
            "left_heel": union,
            "left_toe": np.zeros_like(union),
        }
        sources = {
            "heel_toe_event_guards": "candidate_two_sensor_forces",
            "normal_force_bw": "primary_online_grf_left_aggregate",
            "in_contact": "primary_online_grf_left_union_physical_penetration",
        }
    else:
        raise ValueError(f"unsupported branch: {branch_id}")
    inputs = {
        key: common[key]
        for key in (
            "setup",
            "times",
            "kinematics",
            "prescribed_vertical_n",
            "reference_events",
            "body_weight_n",
        )
    }
    inputs.update(
        {
            "loads": detector_loads,
            "penetrations": penetrations,
            "aggregate": aggregate,
        }
    )
    return inputs, sources


def _reference_stance_mask(
    times: np.ndarray,
    reference_events: Mapping[str, np.ndarray],
) -> np.ndarray:
    mask = np.zeros(times.shape, dtype=bool)
    heel_strikes = np.asarray(reference_events["heel_strike"], dtype=float)
    toe_offs = np.asarray(reference_events["toe_off"], dtype=float)
    for heel_strike_s, toe_off_s in zip(heel_strikes, toe_offs):
        mask |= (times >= heel_strike_s) & (times < toe_off_s)
    return mask


def _reference_cycle_load_integrals_bw_s(
    times: np.ndarray,
    aggregate_normal_force_n: np.ndarray,
    body_weight_n: float,
    reference_events: Mapping[str, np.ndarray],
    *,
    sample_dt_s: float,
) -> list[float]:
    """Integrate load evidence separately over every reference HS-to-TO."""
    heel_strikes = np.asarray(reference_events["heel_strike"], dtype=float)
    toe_offs = np.asarray(reference_events["toe_off"], dtype=float)
    force_bw = np.asarray(aggregate_normal_force_n, dtype=float) / float(
        body_weight_n
    )
    values: list[float] = []
    for cycle_index, (heel_strike_s, toe_off_s) in enumerate(
        zip(heel_strikes, toe_offs)
    ):
        indices = np.flatnonzero(
            (times >= heel_strike_s) & (times < toe_off_s)
        )
        if indices.size == 0:
            raise ProtocolError(
                f"empty reference stance interval at cycle {cycle_index}"
            )
        values.append(float(np.sum(force_bw[indices]) * sample_dt_s))
    return values


def _reason_counts(replay: Mapping[str, Any], detail: Mapping[str, Any]) -> dict[str, Any]:
    invalid = Counter(str(item["type"]) for item in replay["invalid_steps"])
    semantic = detail["semantic_gate"]
    timeouts = Counter(
        str(item.get("cycle_reject_reason") or item.get("event") or "timeout")
        for item in semantic["timeout_transitions"]
    )
    return {
        "invalid_step_reasons": dict(sorted(invalid.items())),
        "timeout_reasons": dict(sorted(timeouts.items())),
        "stance_load_too_low": int(invalid.get("stance_load_too_low", 0)),
        "stance_contact_too_low": int(invalid.get("stance_contact_too_low", 0)),
        "phase_timeout": int(sum(timeouts.values())),
        "total_invalid_steps": int(sum(invalid.values())),
        "total_timeout_transitions": int(sum(timeouts.values())),
    }


def evaluate_branch(
    protocol: Mapping[str, Any],
    candidate_id: str,
    common: Mapping[str, Any],
    branch_id: str,
) -> tuple[dict[str, Any], dict[str, Any]]:
    inputs, sources = compose_branch_inputs(common, branch_id)
    detector = Candidate(candidate_id, v1.SENSOR_ON_N, v1.SENSOR_OFF_N)
    row, detail = _evaluate_candidate(
        detector,
        inputs,
        protocol,
        sample_dt_s=v1.PRIMARY_DT_S,
    )
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
    reasons = _reason_counts(replay, detail)
    times = np.asarray(inputs["times"], dtype=float)
    aggregate = np.asarray(inputs["aggregate"], dtype=float)
    body_weight_n = float(inputs["body_weight_n"])
    in_contact = (
        np.asarray(inputs["penetrations"]["left_heel"], dtype=float) > 0.0
    ) | (np.asarray(inputs["penetrations"]["left_toe"], dtype=float) > 0.0)
    reference_stance = _reference_stance_mask(times, inputs["reference_events"])
    stance_load = aggregate[reference_stance] / body_weight_n
    cycle_load_integrals = _reference_cycle_load_integrals_bw_s(
        times,
        aggregate,
        body_weight_n,
        inputs["reference_events"],
        sample_dt_s=v1.PRIMARY_DT_S,
    )
    minimum_stance_load_bw_s = float(
        protocol["replay"]["fsm_min_stance_load_bw_s"]
    )
    load_metrics = {
        "normal_force_source": sources["normal_force_bw"],
        "in_contact_source": sources["in_contact"],
        "normal_force_n_min": float(np.min(aggregate)),
        "normal_force_n_mean": float(np.mean(aggregate)),
        "normal_force_n_max": float(np.max(aggregate)),
        "reference_stance_mean_load_bw": float(np.mean(stance_load)),
        "reference_stance_integrated_load_bw_s": float(
            np.sum(stance_load) * v1.PRIMARY_DT_S
        ),
        "reference_cycle_load_integrals_bw_s": cycle_load_integrals,
        "reference_cycle_load_integral_min_bw_s": float(
            np.min(cycle_load_integrals)
        ),
        "reference_cycle_load_integral_median_bw_s": float(
            np.median(cycle_load_integrals)
        ),
        "reference_cycle_load_integral_max_bw_s": float(
            np.max(cycle_load_integrals)
        ),
        "reference_cycles_below_fsm_min_stance_load_count": int(
            np.count_nonzero(
                np.asarray(cycle_load_integrals, dtype=float)
                < minimum_stance_load_bw_s
            )
        ),
        "fsm_min_stance_load_bw_s": minimum_stance_load_bw_s,
        "in_contact_sample_count": int(np.count_nonzero(in_contact)),
        "in_contact_fraction": float(np.mean(in_contact)),
    }
    guard_trace = {
        "sensor_edges": replay["sensor_edges"],
        "sensor_event_candidates": replay["candidates"],
        "heel_contact": np.asarray(replay["heel_contact"], dtype=float).tolist(),
        "toe_contact": np.asarray(replay["toe_contact"], dtype=float).tolist(),
    }
    row.update(
        {
            "branch_id": branch_id,
            "diagnostic_role": "causal_load_evidence_ab_only",
            "heel_toe_event_guard_source": sources["heel_toe_event_guards"],
            "normal_force_bw_source": sources["normal_force_bw"],
            "in_contact_source": sources["in_contact"],
            "stance_load_too_low_count": reasons["stance_load_too_low"],
            "stance_contact_too_low_count": reasons["stance_contact_too_low"],
            "phase_timeout_count": reasons["phase_timeout"],
            "invalid_reason_counts_json": json.dumps(
                reasons["invalid_step_reasons"], sort_keys=True
            ),
            "timeout_reason_counts_json": json.dumps(
                reasons["timeout_reasons"], sort_keys=True
            ),
            "reference_stance_mean_load_bw": load_metrics[
                "reference_stance_mean_load_bw"
            ],
            "reference_stance_integrated_load_bw_s": load_metrics[
                "reference_stance_integrated_load_bw_s"
            ],
            "load_evidence_in_contact_fraction": load_metrics[
                "in_contact_fraction"
            ],
            "reference_cycle_load_integral_min_bw_s": load_metrics[
                "reference_cycle_load_integral_min_bw_s"
            ],
            "reference_cycle_load_integral_median_bw_s": load_metrics[
                "reference_cycle_load_integral_median_bw_s"
            ],
            "reference_cycle_load_integral_max_bw_s": load_metrics[
                "reference_cycle_load_integral_max_bw_s"
            ],
            "reference_cycles_below_fsm_min_stance_load_count": load_metrics[
                "reference_cycles_below_fsm_min_stance_load_count"
            ],
            "detector_guard_trace_sha256": _canonical_hash(guard_trace),
        }
    )
    detail.update(
        {
            "branch_id": branch_id,
            "stream_sources": sources,
            "invalid_reasons": reasons,
            "load_evidence": load_metrics,
            "detector_guard_trace_sha256": row[
                "detector_guard_trace_sha256"
            ],
            "final_fsm_payload": replay["fsm"].payload(),
        }
    )
    return row, detail


def compare_branches(
    rows: Sequence[Mapping[str, Any]],
) -> list[dict[str, Any]]:
    comparisons: list[dict[str, Any]] = []
    for candidate_id in CANDIDATE_IDS:
        by_branch = {
            str(row["branch_id"]): row
            for row in rows
            if row["candidate_id"] == candidate_id
        }
        if set(by_branch) != set(BRANCH_IDS):
            raise ProtocolError(f"missing A/B row for {candidate_id}")
        branch_a = by_branch["A_detector_load"]
        branch_b = by_branch["B_primary_load"]
        a_load_rejections = int(branch_a["stance_load_too_low_count"])
        b_load_rejections = int(branch_b["stance_load_too_low_count"])
        comparisons.append(
            {
                "candidate_id": candidate_id,
                "detector_guard_trace_identical": (
                    branch_a["detector_guard_trace_sha256"]
                    == branch_b["detector_guard_trace_sha256"]
                ),
                "branch_a_stance_load_too_low_count": a_load_rejections,
                "branch_b_stance_load_too_low_count": b_load_rejections,
                "stance_load_too_low_count_delta_b_minus_a": (
                    b_load_rejections - a_load_rejections
                ),
                "stance_load_too_low_disappears": bool(
                    a_load_rejections > 0 and b_load_rejections == 0
                ),
                "branch_a_predicted_hs_count": int(
                    branch_a["predicted_hs_count"]
                ),
                "branch_b_predicted_hs_count": int(
                    branch_b["predicted_hs_count"]
                ),
                "branch_a_predicted_to_count": int(
                    branch_a["predicted_to_count"]
                ),
                "branch_b_predicted_to_count": int(
                    branch_b["predicted_to_count"]
                ),
                "branch_a_valid_cycle_count": int(
                    branch_a["observed_valid_cycle_count"]
                ),
                "branch_b_valid_cycle_count": int(
                    branch_b["observed_valid_cycle_count"]
                ),
                "branch_a_invalid_reasons": json.loads(
                    str(branch_a["invalid_reason_counts_json"])
                ),
                "branch_b_invalid_reasons": json.loads(
                    str(branch_b["invalid_reason_counts_json"])
                ),
            }
        )
    return comparisons


def _write_rows_csv(path: Path, rows: Sequence[Mapping[str, Any]]) -> None:
    fieldnames = sorted({str(key) for row in rows for key in row})
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def _plot_comparison(path: Path, rows: Sequence[Mapping[str, Any]]) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    labels = list(CANDIDATE_IDS)
    x = np.arange(len(labels), dtype=float)
    width = 0.34
    by_key = {
        (str(row["candidate_id"]), str(row["branch_id"])): row
        for row in rows
    }
    fig, axes = plt.subplots(3, 1, figsize=(9.5, 9.0), sharex=True)
    metrics = (
        ("stance_load_too_low_count", "stance_load_too_low [count]"),
        ("invalid_or_timeout_transition_count", "invalid + timeout [count]"),
        ("observed_valid_cycle_count", "valid cycles [count]"),
    )
    colors = {"A_detector_load": "#E45756", "B_primary_load": "#4C78A8"}
    for axis, (metric, ylabel) in zip(axes, metrics):
        for offset, branch_id in ((-width / 2, BRANCH_IDS[0]), (width / 2, BRANCH_IDS[1])):
            values = [float(by_key[(label, branch_id)][metric]) for label in labels]
            axis.bar(
                x + offset,
                values,
                width,
                label=branch_id,
                color=colors[branch_id],
            )
        axis.set_ylabel(ylabel)
        axis.grid(axis="y", alpha=0.25)
    axes[0].legend(loc="best")
    axes[-1].set_xticks(x, labels)
    axes[-1].set_xlabel("fixed two-sensor candidate")
    fig.suptitle(
        "Causal dual-stream diagnostic — identical heel/toe guards, different load evidence"
    )
    fig.tight_layout()
    fig.savefig(path, dpi=170)
    plt.close(fig)


def _diagnostic_conclusion(comparisons: Sequence[Mapping[str, Any]]) -> str:
    if not all(bool(item["detector_guard_trace_identical"]) for item in comparisons):
        return "INVALID_AB_DETECTOR_GUARD_STREAM_DRIFT"
    a_counts = [int(item["branch_a_stance_load_too_low_count"]) for item in comparisons]
    b_counts = [int(item["branch_b_stance_load_too_low_count"]) for item in comparisons]
    if all(a > 0 and b == 0 for a, b in zip(a_counts, b_counts)):
        return "PRIMARY_LOAD_REMOVES_STANCE_LOAD_TOO_LOW_FOR_ALL_CANDIDATES"
    if any(a > 0 and b == 0 for a, b in zip(a_counts, b_counts)):
        return "PRIMARY_LOAD_REMOVES_STANCE_LOAD_TOO_LOW_FOR_SOME_CANDIDATES"
    if any(a > 0 for a in a_counts) and any(b > 0 for b in b_counts):
        return "STANCE_LOAD_TOO_LOW_PERSISTS_WITH_PRIMARY_LOAD"
    return "NO_BRANCH_A_STANCE_LOAD_TOO_LOW_TO_EXPLAIN"


def run_diagnostic(
    protocol: Mapping[str, Any],
    output_dir: Path,
    plot_dir: Path,
) -> dict[str, Any]:
    _preflight_no_clobber(output_dir, plot_dir)
    base, candidates, geometry = build_candidates(protocol)
    inputs, access = sample_all_streams_once(protocol, base, candidates)

    rows: list[dict[str, Any]] = []
    details: dict[str, dict[str, Any]] = {}
    for candidate in candidates:
        candidate_details: dict[str, Any] = {}
        common = inputs[candidate.candidate_id]
        for branch_id in BRANCH_IDS:
            row, detail = evaluate_branch(
                protocol, candidate.candidate_id, common, branch_id
            )
            rows.append(row)
            candidate_details[branch_id] = detail
        details[candidate.candidate_id] = candidate_details
    comparisons = compare_branches(rows)
    conclusion = _diagnostic_conclusion(comparisons)

    output_dir.mkdir(parents=True, exist_ok=False)
    plot_dir.mkdir(parents=True, exist_ok=False)
    csv_path = output_dir / "dual_stream_branch_metrics.csv"
    plot_path = plot_dir / "dual_stream_causal_comparison.png"
    _write_rows_csv(csv_path, rows)
    _plot_comparison(plot_path, rows)

    manifest = {
        "schema_version": 1,
        "status": (
            "INVALID"
            if conclusion == "INVALID_AB_DETECTOR_GUARD_STREAM_DRIFT"
            else "COMPLETE"
        ),
        "ok": conclusion != "INVALID_AB_DETECTOR_GUARD_STREAM_DRIFT",
        "stage": "development_causal_diagnostic",
        "protocol": {
            "path": v1._portable_path(Path(str(protocol["_protocol_path"]))),
            "sha256": protocol["_protocol_sha256"],
            "protocol_id": protocol["protocol_id"],
            "frozen_before_execution": True,
        },
        "question": protocol["decision_contract"]["primary_question"],
        "data_access": {
            "already_open_block_s": [v1.BLOCK_START_S, v1.SEALED_START_S],
            "sealed_block_s": [v1.SEALED_START_S, v1.SEALED_END_S],
            "sealed_block_opened": False,
            "sampling": access,
        },
        "detector_contract": {
            "candidate_ids": list(CANDIDATE_IDS),
            "sensors_per_candidate": 2,
            "sensor_roles": ["heel", "forefoot"],
            "sensor_on_threshold_n": v1.SENSOR_ON_N,
            "sensor_off_threshold_n": v1.SENSOR_OFF_N,
            "sensor_dwell_s": v1.SENSOR_DWELL_S,
            "fsm_and_thresholds_unchanged": True,
        },
        "geometry": geometry,
        "branches": protocol["branches"],
        "branch_metrics": rows,
        "branch_details": details,
        "causal_comparisons": comparisons,
        "conclusion": conclusion,
        "decision": {
            **protocol["decision_contract"],
            "acceptance_decision": "NOT_APPLICABLE_CAUSAL_DIAGNOSTIC",
            "selected_candidate": None,
            "promotable": False,
        },
        "artifacts": {
            "branch_metrics_csv": v1._source_record(csv_path),
            "causal_comparison_plot": v1._source_record(plot_path),
        },
        "non_actions": {
            "policy_or_training_run": False,
            "runtime_configuration_modified": False,
            "production_fsm_modified": False,
            "detector_profile_modified": False,
            "load_evidence_profile_modified": False,
            "candidate_profile_created_or_promoted": False,
            "sealed_block_opened": False,
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
        description=(
            "Run the frozen two-sensor/primary-load causal A/B diagnostic."
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
    except NoClobberError as exc:
        print(
            json.dumps(
                {
                    "schema_version": 1,
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
        manifest = run_diagnostic(protocol, output_dir, plot_dir)
    except NoClobberError as exc:  # race-safe second preflight
        print(
            json.dumps(
                {
                    "schema_version": 1,
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
    except Exception as exc:  # fail closed with a machine-readable terminal
        output_dir.mkdir(parents=True, exist_ok=True)
        failure = {
            "schema_version": 1,
            "status": "ERROR",
            "ok": False,
            "error": f"{type(exc).__name__}: {exc}",
            "traceback": traceback.format_exc(),
            "sealed_block_opened": False,
        }
        (output_dir / "failure.json").write_text(
            json.dumps(failure, indent=2) + "\n", encoding="utf-8"
        )
        print(json.dumps(failure, indent=2))
        return 2
    print(
        json.dumps(
            {
                "status": manifest["status"],
                "conclusion": manifest["conclusion"],
                "output_dir": v1._portable_path(output_dir),
                "plot_dir": v1._portable_path(plot_dir),
            },
            indent=2,
        )
    )
    return 0 if manifest["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
