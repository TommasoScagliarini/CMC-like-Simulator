"""Pure fail-closed gates for independent V12R3-P1 qualification."""

from __future__ import annotations

import math
from collections.abc import Mapping
from pathlib import PurePosixPath
from typing import Any

try:
    from . import h0_v12r3_p1_qualification_contract as contract
except ImportError:  # Flat import when this directory is on sys.path.
    import h0_v12r3_p1_qualification_contract as contract


def _mapping(value: Any) -> dict[str, Any]:
    return dict(value) if isinstance(value, Mapping) else {}


def _sha256(value: Any) -> bool:
    return (
        isinstance(value, str)
        and len(value) == 64
        and all(character in "0123456789abcdef" for character in value)
    )


def _counter(value: Any) -> int | None:
    return value if type(value) is int and value >= 0 else None


def _finite(value: Any, *, minimum: float | None = None) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    result = float(value)
    if not math.isfinite(result) or (minimum is not None and result < minimum):
        return None
    return result


def artifact_record_matches(value: Any, expected_path: str | PurePosixPath) -> bool:
    record = _mapping(value)
    return (
        set(record) == {"path", "sha256", "size_bytes"}
        and record.get("path") == PurePosixPath(expected_path).as_posix()
        and _sha256(record.get("sha256"))
        and type(record.get("size_bytes")) is int
        and record["size_bytes"] > 0
    )


def _source_h0_tree_matches(value: Any) -> bool:
    tree = _mapping(value)
    return (
        tree.get("path") == contract.SOURCE_H0_MODULE["path"]
        and tree.get("tree_sha256") == contract.SOURCE_H0_MODULE["tree_sha256"]
        and type(tree.get("file_count")) is int
        and tree["file_count"] > 0
        and isinstance(tree.get("files"), list)
        and len(tree["files"]) == tree["file_count"]
    )


def _continuous_metric(value: Any, expected_samples: int) -> dict[str, float] | None:
    metric = _mapping(value)
    if set(metric) != {"sample_count", "rms", "abs_max"}:
        return None
    rms = _finite(metric.get("rms"), minimum=0.0)
    absolute = _finite(metric.get("abs_max"), minimum=0.0)
    if (
        metric.get("sample_count") != expected_samples
        or rms is None
        or absolute is None
        or absolute < rms
    ):
        return None
    return {"rms": rms, "abs_max": absolute}


def _diagnostic_metrics(summary: Mapping[str, Any]) -> dict[str, Any]:
    episode = _mapping(summary.get("episode_metrics"))
    reserve_valid = set(episode) == {"reserve_norm_nm", "residual_norm_nm"}
    reserve: dict[str, float] = {}
    if reserve_valid:
        for family in ("reserve_norm_nm", "residual_norm_nm"):
            metric = _continuous_metric(episode.get(family), contract.EXPECTED_STEPS)
            if metric is None:
                reserve_valid = False
                break
            for aggregation in contract.CONTINUOUS_AGGREGATIONS:
                reserve[f"{family}.{aggregation}"] = metric[aggregation]

    sea_data = _mapping(summary.get("sea_episode_metrics"))
    sea_valid = set(sea_data) == set(contract.JOINTS)
    saturation_zero = sea_valid
    sea: dict[str, float] = {}
    saturation: dict[str, int] = {}
    if sea_valid:
        for joint in contract.JOINTS:
            joint_data = _mapping(sea_data.get(joint))
            if set(joint_data) != {*contract.SEA_SIGNALS, "tau_input_saturated"}:
                sea_valid = False
                saturation_zero = False
                break
            for signal in contract.SEA_SIGNALS:
                metric = _continuous_metric(
                    joint_data.get(signal),
                    contract.SEA_EXPECTED_SAMPLE_COUNTS[signal],
                )
                if metric is None:
                    sea_valid = False
                    break
                for aggregation in contract.CONTINUOUS_AGGREGATIONS:
                    sea[f"{joint}.{signal}.{aggregation}"] = metric[aggregation]
            sat = _mapping(joint_data.get("tau_input_saturated"))
            sat_count = _counter(sat.get("count"))
            sat_fraction = _finite(sat.get("fraction"), minimum=0.0)
            sat_ok = (
                set(sat) == {"sample_count", "count", "fraction"}
                and sat.get("sample_count") == contract.EXPECTED_RAW_SENSOR_SAMPLES
                and sat_count == 0
                and sat_fraction == 0.0
            )
            saturation_zero = saturation_zero and sat_ok
            if sat_count is not None:
                saturation[joint] = sat_count
    return {
        "reserve_valid": reserve_valid,
        "sea_valid": sea_valid,
        "saturation_zero": saturation_zero,
        "reserve": reserve,
        "sea": sea,
        "saturation": saturation,
    }


def prerequisite_gate(payload: Any) -> dict[str, Any]:
    """Validate four future salvage records only after terminal six-of-six PASS."""

    data = _mapping(payload)
    rows = data.get("prerequisites")
    requirements = contract.prerequisite_requirements()
    rows_pass = isinstance(rows, list) and len(rows) == len(requirements)
    if rows_pass:
        for row, requirement in zip(rows, requirements, strict=True):
            record = _mapping(row)
            if (
                set(record)
                != {
                    "name",
                    "status",
                    "passed",
                    "candidate_id",
                    "candidate_module_tree_sha256",
                    "artifact",
                }
                or record.get("name") != requirement["name"]
                or record.get("status") != requirement["required_status"]
                or record.get("passed") is not True
                or record.get("candidate_id") != contract.P1_CANDIDATE_ID
                or record.get("candidate_module_tree_sha256")
                != contract.P1_CANDIDATE_MODULE["tree_sha256"]
                or not artifact_record_matches(
                    record.get("artifact"), requirement["path"]
                )
            ):
                rows_pass = False
                break
    checks = {
        "schema": data.get("schema_version") == contract.SCHEMA_VERSION,
        "ungated_status": data.get("status") == contract.PREREQUISITE_COMPLETE_STATUS,
        "protocol": data.get("protocol_id") == contract.PROTOCOL_ID,
        "candidate_exact": data.get("candidate_id") == contract.P1_CANDIDATE_ID
        and data.get("candidate_module") == contract.P1_CANDIDATE_MODULE,
        "four_future_records_bound_after_pass": rows_pass
        and data.get("hashes_bound_after_salvage_terminal_pass") is True,
        "same_salvage_stage_order": data.get("salvage_stage_order")
        == list(contract.SALVAGE_STAGE_IDS)
        and data.get("salvage_completed_stages") == list(contract.SALVAGE_STAGE_IDS),
        "salvage_six_of_six": data.get("salvage_rollout_count") == 6
        and data.get("salvage_passing_rollout_count") == 6
        and data.get("salvage_failed_rollout_count") == 0
        and data.get("salvage_terminal_ledger_passed") is True,
        "qualification_unopened": data.get("qualification_root_preexisting") is False
        and data.get("noise_root_preexisting") is False
        and data.get("qualification_rollouts_opened") == 0,
        "zero_updates": data.get("actor_updates") == 0
        and data.get("critic_updates") == 0
        and data.get("ppo_updates") == 0,
        "no_retry_tuning_promotion": data.get("retry_authorized") is False
        and data.get("rescue_authorized") is False
        and data.get("sweep_authorized") is False
        and data.get("post_hoc_tuning_authorized") is False
        and data.get("runtime_promoted") is False,
    }
    passed = all(checks.values())
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": (
            contract.PREREQUISITE_PASS_STATUS
            if passed
            else contract.PREREQUISITE_FAIL_STATUS
        ),
        "passed": passed,
        "protocol_id": contract.PROTOCOL_ID,
        "candidate_id": contract.P1_CANDIDATE_ID,
        "checks": checks,
        "qualification_execution_authorized": False,
        "runtime_promoted": False,
        "next_stage": "FREEZE_QUALIFICATION_PROTOCOL" if passed else "STOP",
    }


def common_rollout_gate(summary: Any, *, role: str, case_id: str) -> dict[str, Any]:
    """Apply exact role routing and common absolute physical gates."""

    data = _mapping(summary)
    try:
        expected = contract.canonical_rollout(role, case_id)
    except ValueError:
        expected = None
    root = (
        PurePosixPath(expected["destination"])
        if expected is not None
        else contract.RUN_ROOT / "invalid"
    )
    expected_draws = (
        contract.EXPECTED_STEPS
        if expected is not None and expected["action_selection"] == "stochastic"
        else 0
    )
    diagnostics = _diagnostic_metrics(data)
    binary_gate = _mapping(data.get("binary_phase_event_gate"))
    candidate_v26 = role == contract.CANDIDATE_ROLE and (
        binary_gate.get("passed") is True
        and binary_gate.get("sample_count") == contract.EXPECTED_RAW_SENSOR_SAMPLES
        and binary_gate.get("duplicate_event_count") == 0
        and binary_gate.get("out_of_order_event_count") == 0
        and binary_gate.get("left_non_v26_source_count") == 0
        and binary_gate.get("fallback_count") == 0
        and binary_gate.get("hard_invalid_count") == 0
    )
    role_event_integrity = (
        candidate_v26
        if role == contract.CANDIDATE_ROLE
        else data.get("legacy_event_integrity_passed") is True
    )
    actor_module_exact = (
        data.get("actor_module") == contract.P1_CANDIDATE_MODULE
        if role == contract.CANDIDATE_ROLE
        else _source_h0_tree_matches(data.get("actor_module"))
    )
    fixed_fields = (
        "actor_id",
        "candidate_id",
        "contract_id",
        "actor_input_view",
        "observation_semantics",
        "primary_load_contract_id",
        "phase_fsm_input_mode",
        "event_contract_id",
        "target_contract_id",
        "binary_phase_fsm_mode",
        "morphology_weight",
    )
    checks = {
        "schema": data.get("schema_version") == contract.SCHEMA_VERSION,
        "ungated_status": data.get("status") == contract.ROLLOUT_COMPLETE_STATUS,
        "protocol": data.get("protocol_id") == contract.PROTOCOL_ID,
        "known_role_and_case": expected is not None
        and data.get("role") == role
        and data.get("case_id") == case_id,
        "condition_exact": expected is not None
        and data.get("action_selection") == expected["action_selection"]
        and data.get("episode_start_offset_s") == expected["episode_start_offset_s"]
        and data.get("action_seed") == expected["action_seed"]
        and data.get("runtime_seed") == expected["runtime_seed"]
        and data.get("sigma") == expected["sigma"],
        "role_routing_exact": expected is not None
        and all(data.get(field) == expected[field] for field in fixed_fields)
        and actor_module_exact,
        "layout": data.get("n_actor") == contract.EXPECTED_ACTOR_FEATURES
        and data.get("n_observation") == contract.EXPECTED_FULL_FEATURES
        and data.get("observation_dtype") == contract.EXPECTED_DTYPE
        and data.get("action_shape") == list(contract.EXPECTED_ACTION_SHAPE)
        and data.get("action_dtype") == contract.EXPECTED_DTYPE,
        "pure_autonomy": data.get("teacher_enabled") is False
        and data.get("teacher_loaded_during_rollout") is False
        and data.get("blending_enabled") is False
        and data.get("safety_latch_enabled") is False
        and data.get("actor_query_count") == contract.EXPECTED_STEPS,
        "physical": data.get("steps") == contract.EXPECTED_STEPS
        and data.get("trace_step_count") == contract.EXPECTED_STEPS
        and data.get("control_window_count") == contract.EXPECTED_CONTROL_WINDOWS
        and data.get("raw_sensor_sample_count") == contract.EXPECTED_RAW_SENSOR_SAMPLES
        and data.get("binary_phase_sensor_sample_count")
        == contract.EXPECTED_RAW_SENSOR_SAMPLES
        and data.get("end_reason") == "episode_time_limit"
        and data.get("terminated") is False
        and data.get("truncated") is True
        and _counter(data.get("phase_valid_cycle_count")) is not None
        and data["phase_valid_cycle_count"] >= contract.MINIMUM_VALID_CYCLES
        and _finite(data.get("grf_penetration_max_m"), minimum=0.0) is not None
        and float(data["grf_penetration_max_m"]) < contract.PENETRATION_LIMIT_M,
        "role_event_integrity": role_event_integrity,
        "zero_invalids_fallbacks_updates": all(
            _counter(data.get(field)) == 0 for field in contract.ZERO_REQUIRED_COUNTS
        ),
        "diagnostic_metrics": diagnostics["reserve_valid"] is True
        and diagnostics["sea_valid"] is True,
        "sea_saturation_zero": diagnostics["saturation_zero"] is True,
        "noise_exactly_once": data.get("random_noise_draw_count") == expected_draws
        and data.get("single_noise_application_count") == contract.EXPECTED_STEPS
        and artifact_record_matches(
            data.get("noise_tape"), expected["noise_tape"] if expected else "invalid"
        )
        and _sha256(data.get("noise_tape_array_sha256")),
        "persisted_before_gate": artifact_record_matches(
            data.get("protocol_freeze"), contract.PROTOCOL_FREEZE_PATH
        )
        and artifact_record_matches(
            data.get("execution_lock"), contract.EXECUTION_LOCK_PATH
        )
        and artifact_record_matches(
            data.get("pipeline_claim"), contract.PIPELINE_CLAIM_PATH
        )
        and artifact_record_matches(data.get("run_start"), root / "run_start.json")
        and artifact_record_matches(data.get("trace"), root / "trace.json")
        and artifact_record_matches(
            data.get("partial_summary"), root / "partial_summary.json"
        )
        and expected is not None
        and artifact_record_matches(
            data.get("worker_claim"),
            contract.worker_claim_path(f"rollout__{role}__{case_id}"),
        ),
        "prerequisite_bound": data.get("prerequisite_gate_passed") is True,
        "closed_and_unpromoted": data.get("protected_trials_opened") == []
        and data.get("reserve_trials_opened") == []
        and data.get("runtime_promoted") is False
        and data.get("checkpoint_zero_created") is False
        and data.get("retry_authorized") is False
        and data.get("rescue_authorized") is False
        and data.get("sweep_authorized") is False
        and data.get("post_hoc_tuning_authorized") is False,
    }
    passed = all(checks.values())
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": (
            contract.ROLLOUT_PASS_STATUS if passed else contract.ROLLOUT_FAIL_STATUS
        ),
        "passed": passed,
        "protocol_id": contract.PROTOCOL_ID,
        "candidate_id": contract.P1_CANDIDATE_ID,
        "role": role,
        "case_id": case_id,
        "checks": checks,
        "continuous_metrics": {
            "reserve": diagnostics["reserve"],
            "sea": diagnostics["sea"],
            "saturation": diagnostics["saturation"],
        },
        "runtime_promoted": False,
    }


def condition_matched_gate(
    baseline: Any, candidate: Any, *, case_id: str
) -> dict[str, Any]:
    """Compare P1 only with its exact condition-matched original-H0 rollout."""

    baseline_data = _mapping(baseline)
    candidate_data = _mapping(candidate)
    baseline_gate = common_rollout_gate(
        baseline_data, role=contract.BASELINE_ROLE, case_id=case_id
    )
    candidate_gate = common_rollout_gate(
        candidate_data, role=contract.CANDIDATE_ROLE, case_id=case_id
    )
    condition_fields = (
        "case_id",
        "action_selection",
        "episode_start_offset_s",
        "action_seed",
        "runtime_seed",
        "sigma",
        "noise_tape_array_sha256",
    )
    condition_exact = all(
        baseline_data.get(field) == candidate_data.get(field)
        for field in condition_fields
    ) and _mapping(baseline_data.get("noise_tape")).get("sha256") == _mapping(
        candidate_data.get("noise_tape")
    ).get(
        "sha256"
    )

    baseline_metrics = baseline_gate["continuous_metrics"]
    candidate_metrics = candidate_gate["continuous_metrics"]
    rows: list[dict[str, Any]] = []
    for family, tolerances in (
        ("reserve", contract.RESERVE_TOLERANCES),
        ("sea", contract.SEA_TOLERANCES),
    ):
        for metric, absolute, relative in tolerances:
            reference = baseline_metrics[family].get(metric)
            observed = candidate_metrics[family].get(metric)
            valid = (
                _finite(reference, minimum=0.0) is not None
                and _finite(observed, minimum=0.0) is not None
            )
            tolerance = (
                max(float(absolute), float(relative) * abs(float(reference)))
                if valid
                else None
            )
            cap = (
                float(reference) + tolerance
                if valid and tolerance is not None
                else None
            )
            # The contract threshold is inclusive.  Permit exactly one ULP on
            # the computed cap so a mathematically equal decimal value is not
            # rejected solely because ``reference + tolerance`` rounded down.
            comparison_cap = math.nextafter(cap, math.inf) if cap is not None else None
            row_passed = (
                valid
                and comparison_cap is not None
                and float(observed) <= comparison_cap
            )
            rows.append(
                {
                    "family": family,
                    "metric": metric,
                    "baseline": reference,
                    "candidate": observed,
                    "absolute_tolerance": float(absolute),
                    "relative_tolerance": float(relative),
                    "tolerance": tolerance,
                    "cap": cap,
                    "comparison_cap": comparison_cap,
                    "passed": row_passed,
                }
            )
    saturation_rows = []
    for joint in contract.JOINTS:
        baseline_count = baseline_metrics["saturation"].get(joint)
        candidate_count = candidate_metrics["saturation"].get(joint)
        passed = (
            baseline_count == 0
            and candidate_count == 0
            and candidate_count <= baseline_count
        )
        saturation_rows.append(
            {
                "joint": joint,
                "baseline": baseline_count,
                "candidate": candidate_count,
                "passed": passed,
            }
        )
    checks = {
        "baseline_common": baseline_gate["passed"] is True,
        "candidate_common": candidate_gate["passed"] is True,
        "condition_exact": condition_exact,
        "all_noninferiority_rows": bool(rows)
        and all(row["passed"] is True for row in rows),
        "saturation_nonincreasing_and_zero": all(
            row["passed"] is True for row in saturation_rows
        ),
        "same_exact_candidate": baseline_data.get("candidate_id")
        == contract.P1_CANDIDATE_ID
        and candidate_data.get("candidate_id") == contract.P1_CANDIDATE_ID,
    }
    passed = all(checks.values())
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.PAIR_PASS_STATUS if passed else contract.PAIR_FAIL_STATUS,
        "passed": passed,
        "protocol_id": contract.PROTOCOL_ID,
        "candidate_id": contract.P1_CANDIDATE_ID,
        "case_id": case_id,
        "checks": checks,
        "baseline_common_gate": baseline_gate,
        "candidate_common_gate": candidate_gate,
        "noninferiority_rows": rows,
        "saturation_rows": saturation_rows,
        "compensation_or_averaging_used": False,
        "runtime_promoted": False,
    }


def aggregate_qualification_gate(summary: Any) -> dict[str, Any]:
    """Require all six pair gates PASS; no case may compensate another."""

    data = _mapping(summary)
    bindings = data.get("pair_bindings")
    bindings_pass = isinstance(bindings, list) and len(bindings) == len(
        contract.CASE_IDS
    )
    if bindings_pass:
        for binding, case_id in zip(bindings, contract.CASE_IDS, strict=True):
            row = _mapping(binding)
            if (
                set(row)
                != {
                    "case_id",
                    "passed",
                    "pair_gate",
                    "baseline_receipt",
                    "candidate_receipt",
                }
                or row.get("case_id") != case_id
                or row.get("passed") is not True
                or not artifact_record_matches(
                    row.get("pair_gate"), contract.pair_gate_path(case_id)
                )
                or not artifact_record_matches(
                    row.get("baseline_receipt"),
                    contract.rollout_receipt_path(contract.BASELINE_ROLE, case_id),
                )
                or not artifact_record_matches(
                    row.get("candidate_receipt"),
                    contract.rollout_receipt_path(contract.CANDIDATE_ROLE, case_id),
                )
            ):
                bindings_pass = False
                break
    checks = {
        "schema": data.get("schema_version") == contract.SCHEMA_VERSION,
        "ungated_status": data.get("status") == contract.AGGREGATE_COMPLETE_STATUS,
        "protocol": data.get("protocol_id") == contract.PROTOCOL_ID,
        "candidate_exact": data.get("candidate_id") == contract.P1_CANDIDATE_ID
        and data.get("candidate_module") == contract.P1_CANDIDATE_MODULE,
        "prerequisite_bound": data.get("prerequisite_gate_passed") is True,
        "twelve_rollouts": data.get("rollout_matrix") == list(contract.ROLLOUT_MATRIX)
        and data.get("baseline_rollout_count") == 6
        and data.get("candidate_rollout_count") == 6
        and data.get("total_rollout_count") == 12,
        "six_of_six_pairs": bindings_pass
        and data.get("pair_count") == 6
        and data.get("passing_pair_count") == 6
        and data.get("failed_pair_count") == 0,
        "exclusive_claims": artifact_record_matches(
            data.get("protocol_freeze"), contract.PROTOCOL_FREEZE_PATH
        )
        and artifact_record_matches(
            data.get("execution_lock"), contract.EXECUTION_LOCK_PATH
        )
        and artifact_record_matches(
            data.get("pipeline_claim"), contract.PIPELINE_CLAIM_PATH
        )
        and artifact_record_matches(
            data.get("worker_claim"),
            contract.worker_claim_path("finalize_qualification"),
        ),
        "zero_updates": data.get("actor_updates") == 0
        and data.get("critic_updates") == 0
        and data.get("ppo_updates") == 0,
        "no_compensation": data.get("compensation_or_averaging_used") is False,
        "no_retry_tuning_promotion": data.get("retry_authorized") is False
        and data.get("rescue_authorized") is False
        and data.get("sweep_authorized") is False
        and data.get("post_hoc_tuning_authorized") is False
        and data.get("runtime_promoted") is False
        and data.get("checkpoint_zero_created") is False
        and data.get("positive_morphology_enabled") is False,
        "closed_data": data.get("protected_trials_opened") == []
        and data.get("reserve_trials_opened") == [],
    }
    passed = all(checks.values())
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": (
            contract.AGGREGATE_PASS_STATUS if passed else contract.AGGREGATE_FAIL_STATUS
        ),
        "passed": passed,
        "protocol_id": contract.PROTOCOL_ID,
        "candidate_id": contract.P1_CANDIDATE_ID,
        "checks": checks,
        "independent_qualification_passed": passed,
        "candidate_promoted": False,
        "runtime_promoted": False,
        "next_stage": "WAIT_SEPARATE_ZERO_UPDATE_PROTOCOL" if passed else "STOP",
    }


__all__ = [
    "aggregate_qualification_gate",
    "artifact_record_matches",
    "common_rollout_gate",
    "condition_matched_gate",
    "prerequisite_gate",
]
