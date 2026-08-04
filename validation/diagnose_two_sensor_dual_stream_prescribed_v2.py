"""V2 causal A/B diagnostic with a strictly pre-FSM detector invariant.

V1 incorrectly hashed ``replay['candidates']`` together with the raw detector
trace.  Those candidate events are conditioned on the current FSM state, so a
load-evidence change can legitimately change them even when heel/toe forces,
debounced latches, and detector edges are identical.  V2 gates A/B integrity
only on those pre-FSM quantities and retains state-conditioned candidates as a
separate diagnostic.

The sampling, two fixed detector geometries, production FSM, thresholds,
already-open 50--100 s data block, and diagnostic-only decision contract are
inherited unchanged from the hash-pinned V1 protocol.  V1 artifacts are never
overwritten.
"""

from __future__ import annotations

import argparse
import json
import sys
import traceback
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

import diagnose_two_sensor_dual_stream_prescribed as v1  # noqa: E402


DEFAULT_PROTOCOL = (
    VALIDATION_ROOT / "two_sensor_dual_stream_prescribed_protocol_v2.json"
)
DEFAULT_OUTPUT_DIR = (
    VALIDATION_ROOT
    / "two_sensor_dual_stream_diagnostic_runs/"
    "2026-07-22_ab06_50_100_h02_f70_p315_p320_v2_raw_guard"
)
DEFAULT_PLOT_DIR = (
    REPO_ROOT
    / "plot/07_22_2026_two_sensor_dual_stream_causal_diagnostic_v2_raw_guard"
)
PROTOCOL_ID = (
    "AB06_TWO_SENSOR_DUAL_STREAM_CAUSAL_DIAGNOSTIC_2026-07-22_V2_RAW_GUARD"
)
V1_PROTOCOL_PATH = VALIDATION_ROOT / "two_sensor_dual_stream_prescribed_protocol.json"
V1_MANIFEST_PATH = (
    VALIDATION_ROOT
    / "two_sensor_dual_stream_diagnostic_runs/"
    "2026-07-22_ab06_50_100_h02_f70_p315_p320/manifest.json"
)


class ProtocolError(ValueError):
    """Raised before sampling when the V2 correction contract drifts."""


def load_and_validate_protocol(
    path: str | Path = DEFAULT_PROTOCOL,
) -> dict[str, Any]:
    protocol_path = v1.v1.resolve_repo_path(path).resolve()
    try:
        raw = json.loads(protocol_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ProtocolError(f"cannot load V2 protocol: {protocol_path}") from exc
    if not isinstance(raw, dict) or raw.get("schema_version") != 2:
        raise ProtocolError("unsupported dual-stream V2 protocol schema")
    expected_top_level = {
        "protocol_id": PROTOCOL_ID,
        "frozen_before_execution": True,
        "stage": "development_causal_diagnostic",
    }
    for key, expected in expected_top_level.items():
        if raw.get(key) != expected:
            raise ProtocolError(f"frozen V2 field drifted: {key}")

    derived = raw.get("derived_from")
    if not isinstance(derived, dict):
        raise ProtocolError("V1 lineage is required")
    expected_lineage_paths = {
        "protocol_path": v1.v1._portable_path(V1_PROTOCOL_PATH),
        "invalid_manifest_path": v1.v1._portable_path(V1_MANIFEST_PATH),
    }
    for key, expected in expected_lineage_paths.items():
        if derived.get(key) != expected:
            raise ProtocolError(f"V1 lineage path drifted: {key}")
    expected_hashes = {
        "protocol_sha256": v1.v1._sha256(V1_PROTOCOL_PATH),
        "invalid_manifest_sha256": v1.v1._sha256(V1_MANIFEST_PATH),
    }
    for key, expected in expected_hashes.items():
        if derived.get(key) != expected:
            raise ProtocolError(f"V1 lineage hash drifted: {key}")
    if derived.get("invalid_conclusion") != "INVALID_AB_DETECTOR_GUARD_STREAM_DRIFT":
        raise ProtocolError("V1 invalid conclusion lineage drifted")

    frozen_v1 = v1.load_and_validate_protocol(V1_PROTOCOL_PATH)
    inherited_keys = (
        "setup",
        "profile_paths",
        "load_evidence_profile",
        "candidate_ids",
        "candidate_lineage",
        "data_access",
        "replay",
        "branches",
        "sampling",
        "decision_contract",
    )
    expected_inheritance = {
        "mode": "reuse_hash_pinned_v1_without_changes",
        "keys": list(inherited_keys),
    }
    if raw.get("inherited_contract") != expected_inheritance:
        raise ProtocolError("V2 inherited-contract declaration drifted")

    expected_correction = {
        "v1_error": (
            "state_conditioned_sensor_event_candidates_were_included_in_"
            "the_detector_guard_hash"
        ),
        "ab_integrity_gate_fields": [
            "raw_heel_force_trace",
            "raw_toe_force_trace",
            "debounced_heel_latch_trace",
            "debounced_toe_latch_trace",
            "raw_detector_edge_trace",
        ],
        "fsm_conditioned_sensor_event_candidates_role": "diagnostic_only",
        "fsm_conditioned_candidate_drift_invalidates_ab": False,
        "accepted_transitions_in_ab_integrity_hash": False,
    }
    if raw.get("guard_trace_correction") != expected_correction:
        raise ProtocolError("raw detector guard correction drifted")

    expected_execution = {
        "default_output_dir": v1.v1._portable_path(DEFAULT_OUTPUT_DIR),
        "default_plot_dir": v1.v1._portable_path(DEFAULT_PLOT_DIR),
        "new_versioned_run_required": True,
        "v1_artifacts_must_remain_immutable": True,
    }
    if raw.get("execution") != expected_execution:
        raise ProtocolError("versioned V2 execution contract drifted")

    sources = raw.get("sources")
    if not isinstance(sources, dict) or not sources:
        raise ProtocolError("hash-pinned V2 sources are required")
    for label, record in sources.items():
        if not isinstance(record, dict):
            raise ProtocolError(f"invalid V2 source record: {label}")
        source_path = v1.v1.resolve_repo_path(str(record.get("path", ""))).resolve()
        if not source_path.is_file():
            raise ProtocolError(f"missing V2 source {label}: {source_path}")
        observed = v1.v1._sha256(source_path)
        if observed != record.get("sha256"):
            raise ProtocolError(
                f"V2 source hash drift for {label}: {observed} != "
                f"{record.get('sha256')}"
            )

    for key in inherited_keys:
        raw[key] = frozen_v1[key]

    raw["_protocol_path"] = protocol_path.as_posix()
    raw["_protocol_sha256"] = v1.v1._sha256(protocol_path)
    raw["_primary_event_time_field"] = frozen_v1["_primary_event_time_field"]
    raw["_diagnostic_event_time_field"] = frozen_v1[
        "_diagnostic_event_time_field"
    ]
    raw["_phase_reference_mode"] = frozen_v1["_phase_reference_mode"]
    return raw


def _detector_trace_diagnostics(
    inputs: Mapping[str, Any],
    replay: Mapping[str, Any],
) -> dict[str, Any]:
    force_trace = {
        "heel_normal_force_n": np.asarray(
            inputs["loads"]["left_heel"], dtype=float
        ).tolist(),
        "toe_normal_force_n": np.asarray(
            inputs["loads"]["left_toe"], dtype=float
        ).tolist(),
    }
    latch_trace = {
        "heel_contact": np.asarray(replay["heel_contact"], dtype=float).tolist(),
        "toe_contact": np.asarray(replay["toe_contact"], dtype=float).tolist(),
    }
    edge_trace = [dict(item) for item in replay["sensor_edges"]]
    conditioned_candidates = [dict(item) for item in replay["candidates"]]
    raw_guard = {
        "force_trace_sha256": v1._canonical_hash(force_trace),
        "latch_trace_sha256": v1._canonical_hash(latch_trace),
        "edge_trace_sha256": v1._canonical_hash(edge_trace),
    }
    return {
        "raw_detector_force_trace_sha256": raw_guard["force_trace_sha256"],
        "debounced_detector_latch_trace_sha256": raw_guard[
            "latch_trace_sha256"
        ],
        "raw_detector_edge_trace_sha256": raw_guard["edge_trace_sha256"],
        "raw_detector_guard_trace_sha256": v1._canonical_hash(raw_guard),
        "fsm_conditioned_candidate_trace_sha256": v1._canonical_hash(
            conditioned_candidates
        ),
        "fsm_conditioned_candidate_count": len(conditioned_candidates),
        "fsm_conditioned_sensor_event_candidates": conditioned_candidates,
    }


def evaluate_branch(
    protocol: Mapping[str, Any],
    candidate_id: str,
    common: Mapping[str, Any],
    branch_id: str,
) -> tuple[dict[str, Any], dict[str, Any]]:
    row, detail = v1.evaluate_branch(protocol, candidate_id, common, branch_id)
    v1_mixed_hash = row.pop("detector_guard_trace_sha256")
    detail.pop("detector_guard_trace_sha256", None)
    inputs, _sources = v1.compose_branch_inputs(common, branch_id)
    runtime_cfg = replace(
        v1.v1._current_runtime_fsm_config(),
        sensor_on_threshold_n=v1.v1.SENSOR_ON_N,
        sensor_off_threshold_n=v1.v1.SENSOR_OFF_N,
        sensor_dwell_s=v1.v1.SENSOR_DWELL_S,
    )
    replay = v1.v1._run_production_fsm(
        np.asarray(inputs["times"], dtype=float),
        dict(inputs["loads"]),
        dict(inputs["penetrations"]),
        np.asarray(inputs["aggregate"], dtype=float),
        dict(inputs["kinematics"]),
        body_weight_n=float(inputs["body_weight_n"]),
        fsm_config=runtime_cfg,
    )
    traces = _detector_trace_diagnostics(inputs, replay)
    row.update(
        {
            key: value
            for key, value in traces.items()
            if key != "fsm_conditioned_sensor_event_candidates"
        }
    )
    row["v1_mixed_state_conditioned_trace_sha256_diagnostic_only"] = (
        v1_mixed_hash
    )
    detail.update(
        {
            "raw_detector_integrity": {
                key: value
                for key, value in traces.items()
                if key != "fsm_conditioned_sensor_event_candidates"
            },
            "fsm_conditioned_candidate_diagnostic": {
                "role": "diagnostic_only_not_ab_integrity_gate",
                "trace_sha256": traces[
                    "fsm_conditioned_candidate_trace_sha256"
                ],
                "count": traces["fsm_conditioned_candidate_count"],
                "events": traces["fsm_conditioned_sensor_event_candidates"],
            },
            "v1_mixed_state_conditioned_trace_sha256_diagnostic_only": (
                v1_mixed_hash
            ),
        }
    )
    return row, detail


def compare_branches(rows: Sequence[Mapping[str, Any]]) -> list[dict[str, Any]]:
    comparisons: list[dict[str, Any]] = []
    for candidate_id in v1.CANDIDATE_IDS:
        by_branch = {
            str(row["branch_id"]): row
            for row in rows
            if row["candidate_id"] == candidate_id
        }
        if set(by_branch) != set(v1.BRANCH_IDS):
            raise ProtocolError(f"missing V2 A/B row for {candidate_id}")
        branch_a = by_branch["A_detector_load"]
        branch_b = by_branch["B_primary_load"]
        equality_fields = (
            "raw_detector_force_trace_sha256",
            "debounced_detector_latch_trace_sha256",
            "raw_detector_edge_trace_sha256",
            "raw_detector_guard_trace_sha256",
        )
        equality = {
            field.removesuffix("_sha256") + "_identical": (
                branch_a[field] == branch_b[field]
            )
            for field in equality_fields
        }
        a_load_rejections = int(branch_a["stance_load_too_low_count"])
        b_load_rejections = int(branch_b["stance_load_too_low_count"])
        comparisons.append(
            {
                "candidate_id": candidate_id,
                **equality,
                "raw_detector_ab_integrity_ok": bool(all(equality.values())),
                "fsm_conditioned_candidate_trace_identical": (
                    branch_a["fsm_conditioned_candidate_trace_sha256"]
                    == branch_b["fsm_conditioned_candidate_trace_sha256"]
                ),
                "fsm_conditioned_candidate_trace_role": (
                    "diagnostic_only_not_ab_integrity_gate"
                ),
                "branch_a_fsm_conditioned_candidate_count": int(
                    branch_a["fsm_conditioned_candidate_count"]
                ),
                "branch_b_fsm_conditioned_candidate_count": int(
                    branch_b["fsm_conditioned_candidate_count"]
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


def _diagnostic_conclusion(comparisons: Sequence[Mapping[str, Any]]) -> str:
    if not all(bool(item["raw_detector_ab_integrity_ok"]) for item in comparisons):
        return "INVALID_AB_RAW_DETECTOR_GUARD_STREAM_DRIFT"
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
    v1._preflight_no_clobber(output_dir, plot_dir)
    base_profile, candidates, geometry = v1.build_candidates(protocol)
    inputs, access = v1.sample_all_streams_once(
        protocol, base_profile, candidates
    )
    rows: list[dict[str, Any]] = []
    details: dict[str, dict[str, Any]] = {}
    for candidate in candidates:
        candidate_details: dict[str, Any] = {}
        for branch_id in v1.BRANCH_IDS:
            row, detail = evaluate_branch(
                protocol,
                candidate.candidate_id,
                inputs[candidate.candidate_id],
                branch_id,
            )
            rows.append(row)
            candidate_details[branch_id] = detail
        details[candidate.candidate_id] = candidate_details
    comparisons = compare_branches(rows)
    conclusion = _diagnostic_conclusion(comparisons)

    output_dir.mkdir(parents=True, exist_ok=False)
    plot_dir.mkdir(parents=True, exist_ok=False)
    csv_path = output_dir / "dual_stream_branch_metrics_v2_raw_guard.csv"
    plot_path = plot_dir / "dual_stream_causal_comparison_v2_raw_guard.png"
    v1._write_rows_csv(csv_path, rows)
    v1._plot_comparison(plot_path, rows)

    invalid = conclusion == "INVALID_AB_RAW_DETECTOR_GUARD_STREAM_DRIFT"
    manifest = {
        "schema_version": 2,
        "status": "INVALID" if invalid else "COMPLETE",
        "ok": not invalid,
        "stage": "development_causal_diagnostic",
        "protocol": {
            "path": v1.v1._portable_path(Path(str(protocol["_protocol_path"]))),
            "sha256": protocol["_protocol_sha256"],
            "protocol_id": protocol["protocol_id"],
            "frozen_before_execution": True,
        },
        "derived_from_invalid_v1": protocol["derived_from"],
        "question": protocol["decision_contract"]["primary_question"],
        "guard_trace_correction": protocol["guard_trace_correction"],
        "data_access": {
            "already_open_block_s": [v1.v1.BLOCK_START_S, v1.v1.SEALED_START_S],
            "sealed_block_s": [v1.v1.SEALED_START_S, v1.v1.SEALED_END_S],
            "sealed_block_opened": False,
            "sampling": access,
        },
        "detector_contract": {
            "candidate_ids": list(v1.CANDIDATE_IDS),
            "sensors_per_candidate": 2,
            "sensor_roles": ["heel", "forefoot"],
            "sensor_on_threshold_n": v1.v1.SENSOR_ON_N,
            "sensor_off_threshold_n": v1.v1.SENSOR_OFF_N,
            "sensor_dwell_s": v1.v1.SENSOR_DWELL_S,
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
            "branch_metrics_csv": v1.v1._source_record(csv_path),
            "causal_comparison_plot": v1.v1._source_record(plot_path),
        },
        "non_actions": {
            "v1_artifacts_modified_or_overwritten": False,
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
    safe = v1.v1._json_safe(manifest)
    (output_dir / "manifest.json").write_text(
        json.dumps(safe, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    return safe


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run V2 dual-stream diagnostic with raw detector integrity."
    )
    parser.add_argument("--protocol", default=str(DEFAULT_PROTOCOL))
    parser.add_argument("--output-dir", default=str(DEFAULT_OUTPUT_DIR))
    parser.add_argument("--plot-dir", default=str(DEFAULT_PLOT_DIR))
    return parser


def _no_clobber_payload(exc: Exception) -> dict[str, Any]:
    return {
        "schema_version": 2,
        "status": "ERROR",
        "ok": False,
        "no_clobber": True,
        "filesystem_mutated": False,
        "error": f"{type(exc).__name__}: {exc}",
    }


def main(argv: Sequence[str] | None = None) -> int:
    args = build_arg_parser().parse_args(argv)
    output_dir = v1.v1.resolve_repo_path(args.output_dir).resolve()
    plot_dir = v1.v1.resolve_repo_path(args.plot_dir).resolve()
    try:
        v1._preflight_no_clobber(output_dir, plot_dir)
    except v1.NoClobberError as exc:
        print(json.dumps(_no_clobber_payload(exc), indent=2))
        return 2
    try:
        protocol = load_and_validate_protocol(args.protocol)
        manifest = run_diagnostic(protocol, output_dir, plot_dir)
    except v1.NoClobberError as exc:
        print(json.dumps(_no_clobber_payload(exc), indent=2))
        return 2
    except Exception as exc:  # fail closed without touching V1 artifacts
        output_dir.mkdir(parents=True, exist_ok=True)
        failure = {
            "schema_version": 2,
            "status": "ERROR",
            "ok": False,
            "error": f"{type(exc).__name__}: {exc}",
            "traceback": traceback.format_exc(),
            "sealed_block_opened": False,
            "v1_artifacts_modified_or_overwritten": False,
        }
        failure_path = output_dir / "failure.json"
        if not failure_path.exists():
            failure_path.write_text(
                json.dumps(failure, indent=2) + "\n", encoding="utf-8"
            )
        print(json.dumps(failure, indent=2))
        return 2
    print(
        json.dumps(
            {
                "status": manifest["status"],
                "conclusion": manifest["conclusion"],
                "output_dir": v1.v1._portable_path(output_dir),
                "plot_dir": v1.v1._portable_path(plot_dir),
                "v1_artifacts_modified_or_overwritten": False,
            },
            indent=2,
        )
    )
    return 0 if manifest["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
