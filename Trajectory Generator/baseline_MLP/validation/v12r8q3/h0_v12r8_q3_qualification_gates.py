"""Pure, independent, fail-closed gates for the V12R8-Q3 scaffold."""

from __future__ import annotations

import copy
import math
from collections.abc import Mapping
from pathlib import PurePosixPath
from typing import Any

try:
    from . import h0_v12r8_q3_artifacts as artifacts
    from . import h0_v12r8_q3_prerequisites as prerequisites
    from . import h0_v12r8_q3_qualification_contract as contract
except ImportError:  # Direct test execution with this directory on sys.path.
    import h0_v12r8_q3_artifacts as artifacts
    import h0_v12r8_q3_prerequisites as prerequisites
    import h0_v12r8_q3_qualification_contract as contract


def _mapping(value: Any) -> dict[str, Any]:
    return dict(value) if isinstance(value, Mapping) else {}


def _type_strict_equal(value: Any, expected: Any) -> bool:
    """Compare JSON-like values without Python's bool/int/float aliasing."""

    if isinstance(expected, Mapping):
        return (
            isinstance(value, Mapping)
            and set(value) == set(expected)
            and all(_type_strict_equal(value[key], expected[key]) for key in expected)
        )
    if isinstance(expected, (list, tuple)):
        return (
            type(value) is type(expected)
            and len(value) == len(expected)
            and all(
                _type_strict_equal(current, target)
                for current, target in zip(value, expected, strict=True)
            )
        )
    return type(value) is type(expected) and value == expected


def _counter(value: Any) -> int | None:
    return value if type(value) is int and value >= 0 else None


def _finite(value: Any, *, minimum: float | None = None) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    result = float(value)
    if not math.isfinite(result) or (minimum is not None and result < minimum):
        return None
    return result


def _failure(status: str, check: str) -> dict[str, Any]:
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": status,
        "passed": False,
        "protocol_id": contract.PROTOCOL_ID,
        "candidate_id": None,
        "checks": {check: False},
    }


def morphology_zero_ab_gate(payload: Any) -> dict[str, Any]:
    """Prove the live causal corridor is reward/action/state inert at weight zero."""

    data = _mapping(payload)
    reward_a = data.get("baseline_reward_bytes_sha256")
    reward_b = data.get("candidate_reward_bytes_sha256")
    action_a = data.get("baseline_action_bytes_sha256")
    action_b = data.get("candidate_action_bytes_sha256")
    observation_a = data.get("baseline_observation_bytes_sha256")
    observation_b = data.get("candidate_observation_bytes_sha256")
    corridor_evaluations = _counter(data.get("corridor_evaluation_count"))
    corridor_unavailable = _counter(data.get("corridor_unavailable_count"))
    checks = {
        "contract": _type_strict_equal(
            data.get("contract"), contract.MORPHOLOGY_ZERO_AB_CONTRACT
        ),
        "live_causal_corridor": data.get("profile_loaded") is True
        and data.get("causal_buffer_active") is True
        and data.get("phase_mode") == contract.MORPHOLOGY_PHASE_MODE
        and data.get("event_contract_id") == contract.EVENT_CONTRACT_ID,
        "sample_counts": _counter(data.get("detector_sample_count"))
        == contract.EXPECTED_RAW_SENSOR_SAMPLES
        and _counter(data.get("reward_sample_count"))
        == contract.EXPECTED_REWARD_SAMPLES
        and _counter(data.get("action_sample_count"))
        == contract.EXPECTED_REWARD_SAMPLES
        and _counter(data.get("observation_sample_count"))
        == contract.EXPECTED_REWARD_SAMPLES
        and corridor_evaluations is not None
        and 0 < corridor_evaluations <= contract.EXPECTED_REWARD_SAMPLES
        and corridor_unavailable is not None
        and corridor_evaluations + corridor_unavailable
        == contract.EXPECTED_REWARD_SAMPLES,
        "reward_bit_identity": artifacts.is_sha256(reward_a)
        and reward_a == reward_b
        and _counter(data.get("reward_bit_mismatch_count")) == 0,
        "action_bit_identity": artifacts.is_sha256(action_a)
        and action_a == action_b
        and _counter(data.get("action_bit_mismatch_count")) == 0,
        "observation_bit_identity": artifacts.is_sha256(observation_a)
        and observation_a == observation_b
        and _counter(data.get("observation_bit_mismatch_count")) == 0,
        "zero_effects": type(data.get("morphology_weight")) is float
        and data.get("morphology_weight") == 0.0
        and type(data.get("morphology_causal_allow_effects")) is float
        and data.get("morphology_causal_allow_effects") == 0.0
        and type(data.get("morphology_hard_termination_enabled")) is float
        and data.get("morphology_hard_termination_enabled") == 0.0
        and _counter(data.get("morphology_term_nonzero_count")) == 0
        and _counter(data.get("morphology_hard_termination_count")) == 0,
        "finite_corridor": _counter(data.get("corridor_finite_count"))
        == corridor_evaluations
        and _counter(data.get("corridor_nonfinite_count")) == 0,
        "self_reported_pass": data.get("passed") is True,
    }
    passed = all(checks.values())
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": "PASS_H0_V12R8_Q3_MORPHOLOGY_ZERO_AB"
        if passed
        else "FAIL_H0_V12R8_Q3_MORPHOLOGY_ZERO_AB",
        "passed": passed,
        "checks": checks,
    }


def v26_event_gate(payload: Any) -> dict[str, Any]:
    """Validate exactly 5,000 samples of the resolved V26 detector route."""

    data = _mapping(payload)
    checks = {
        "passed": data.get("passed") is True,
        "samples": _counter(data.get("sample_count"))
        == contract.EXPECTED_RAW_SENSOR_SAMPLES,
        "profile": data.get("detector_profile")
        == contract.DETECTOR_PROFILE_PATH.as_posix()
        and data.get("detector_profile_sha256") == contract.DETECTOR_PROFILE_SHA256,
        "event_contract": data.get("event_contract_id") == contract.EVENT_CONTRACT_ID
        and data.get("actor_event_source") == contract.V26_ACTOR_EVENT_SOURCE
        and data.get("binary_phase_fsm_mode") == contract.V26_BINARY_MODE,
        "strict_zero_failures": all(
            _counter(data.get(name)) == 0
            for name in (
                "duplicate_event_count",
                "out_of_order_event_count",
                "left_non_v26_source_count",
                "fallback_count",
                "hard_invalid_count",
            )
        ),
        "terminal_pending_policy": data.get("terminal_pending_rule")
        == "drop_samples_at_or_after_pending_onset"
        and _counter(data.get("terminal_pending_sample_drop_count")) is not None,
    }
    passed = all(checks.values())
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": "PASS_H0_V12R8_Q3_V26_EVENT_GATE"
        if passed
        else "FAIL_H0_V12R8_Q3_V26_EVENT_GATE",
        "passed": passed,
        "checks": checks,
    }


def common_rollout_gate(summary: Any, *, role: str, case_id: str) -> dict[str, Any]:
    """Apply absolute physical, routing, detector, and morphology gates."""

    try:
        binding = prerequisites.current_candidate_binding()
        expected = contract.canonical_rollout(role, case_id)
    except (ValueError, prerequisites.V12R8Q3PrerequisiteError):
        return _failure(contract.ROLLOUT_FAIL_STATUS, "r8_candidate_bound")

    data = _mapping(summary)
    root = PurePosixPath(expected["destination"])
    expected_draws = (
        contract.EXPECTED_STEPS if expected["action_selection"] == "stochastic" else 0
    )
    actor_ok = False
    if role == contract.CANDIDATE_ROLE:
        try:
            actor_ok = (
                prerequisites.validate_candidate_tree(
                    data.get("candidate_id"), data.get("actor_module")
                )
                == binding
            )
        except (TypeError, ValueError):
            actor_ok = False
    else:
        module = _mapping(data.get("actor_module"))
        actor_ok = (
            module.get("path") == contract.SOURCE_H0_MODULE["path"]
            and module.get("tree_sha256") == contract.SOURCE_H0_MODULE["tree_sha256"]
        )

    event_result = v26_event_gate(data.get("binary_phase_event_gate"))
    morphology_result = morphology_zero_ab_gate(data.get("morphology_zero_ab"))
    expected_noise = PurePosixPath(expected["noise_tape"])
    resolved_config_ok = _type_strict_equal(
        data.get("resolved_env_config"), expected["resolved_env_config"]
    )
    checks = {
        "schema": data.get("schema_version") == contract.SCHEMA_VERSION,
        "status": data.get("status") == contract.ROLLOUT_COMPLETE_STATUS,
        "protocol": data.get("protocol_id") == contract.PROTOCOL_ID,
        "candidate_binding": data.get("candidate_id") == binding["candidate_id"]
        and actor_ok,
        "role_case": data.get("role") == role and data.get("case_id") == case_id,
        "condition": all(
            data.get(name) == expected[name]
            for name in (
                "action_selection",
                "episode_start_offset_s",
                "action_seed",
                "runtime_seed",
                "sigma",
            )
        ),
        "resolved_env_config": resolved_config_ok,
        "standard_layout": data.get("n_actor") == contract.EXPECTED_ACTOR_FEATURES
        and data.get("n_observation") == contract.EXPECTED_FULL_FEATURES
        and data.get("observation_dtype") == contract.EXPECTED_DTYPE
        and data.get("action_shape") == list(contract.EXPECTED_ACTION_SHAPE)
        and data.get("action_dtype") == contract.EXPECTED_DTYPE
        and (
            role == contract.BASELINE_ROLE
            or data.get("fcnet_hiddens") == list(contract.EXPECTED_HIDDENS)
        ),
        "pure_autonomy": data.get("teacher_enabled") is False
        and data.get("teacher_loaded_during_rollout") is False
        and data.get("blending_enabled") is False
        and data.get("safety_latch_enabled") is False
        and _counter(data.get("actor_query_count")) == contract.EXPECTED_STEPS,
        "full_physical_horizon": _counter(data.get("steps")) == contract.EXPECTED_STEPS
        and _counter(data.get("trace_step_count")) == contract.EXPECTED_STEPS
        and _counter(data.get("control_window_count"))
        == contract.EXPECTED_CONTROL_WINDOWS
        and _counter(data.get("raw_sensor_sample_count"))
        == contract.EXPECTED_RAW_SENSOR_SAMPLES
        and data.get("end_reason") == "episode_time_limit"
        and data.get("terminated") is False
        and data.get("truncated") is True,
        "physical_safety": _counter(data.get("phase_valid_cycle_count")) is not None
        and data["phase_valid_cycle_count"] >= contract.MINIMUM_VALID_CYCLES
        and _finite(data.get("grf_penetration_max_m"), minimum=0.0) is not None
        and float(data["grf_penetration_max_m"]) < contract.PENETRATION_LIMIT_M,
        "zero_failures_updates": all(
            _counter(data.get(name)) == 0 for name in contract.ZERO_REQUIRED_COUNTS
        ),
        "v26_gate": event_result.get("passed") is True
        if role == contract.CANDIDATE_ROLE
        else data.get("legacy_event_integrity_passed") is True,
        "morphology_zero_ab": morphology_result.get("passed") is True
        if role == contract.CANDIDATE_ROLE
        else data.get("morphology_disabled_bit_identity_passed") is True,
        "noise_once": _counter(data.get("random_noise_draw_count")) == expected_draws
        and _counter(data.get("single_noise_application_count"))
        == contract.EXPECTED_STEPS
        and artifacts.artifact_record_matches(data.get("noise_tape"), expected_noise)
        and data.get("noise_tape_array_sha256")
        == contract.EXPECTED_TAPE_ARRAY_SHA256[expected_noise.name],
        "persisted_before_gate": artifacts.artifact_record_matches(
            data.get("protocol_freeze"), contract.PROTOCOL_FREEZE_PATH
        )
        and artifacts.artifact_record_matches(
            data.get("execution_lock"), contract.EXECUTION_LOCK_PATH
        )
        and artifacts.artifact_record_matches(
            data.get("pipeline_claim"), contract.PIPELINE_CLAIM_PATH
        )
        and artifacts.artifact_record_matches(
            data.get("run_start"), root / "run_start.json"
        )
        and artifacts.artifact_record_matches(data.get("trace"), root / "trace.json"),
        "closed_unpromoted": data.get("runtime_promoted") is False
        and data.get("checkpoint_zero_created") is False
        and data.get("morphology_weight") == 0.0
        and data.get("positive_morphology_enabled") is False
        and data.get("retry_authorized") is False
        and data.get("resume_authorized") is False
        and data.get("rescue_authorized") is False
        and data.get("sweep_authorized") is False
        and data.get("post_hoc_tuning_authorized") is False,
    }
    passed = all(checks.values())
    metrics = _mapping(data.get("comparison_metrics"))
    metrics_valid = set(metrics) == {
        row[0] for row in contract.NONINFERIORITY_TOLERANCES
    } and all(_finite(value, minimum=0.0) is not None for value in metrics.values())
    checks["comparison_metrics_complete"] = metrics_valid
    passed = all(checks.values())
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.ROLLOUT_PASS_STATUS
        if passed
        else contract.ROLLOUT_FAIL_STATUS,
        "passed": passed,
        "protocol_id": contract.PROTOCOL_ID,
        "candidate_id": binding["candidate_id"],
        "role": role,
        "case_id": case_id,
        "checks": checks,
        "v26_event_gate": event_result,
        "morphology_zero_ab_gate": morphology_result,
        "comparison_metrics": copy.deepcopy(metrics) if metrics_valid else {},
        "runtime_promoted": False,
    }


def condition_matched_gate(
    baseline: Any, candidate: Any, *, case_id: str
) -> dict[str, Any]:
    """Require absolute PASS plus condition-matched physical noninferiority."""

    baseline_data = _mapping(baseline)
    candidate_data = _mapping(candidate)
    baseline_gate = common_rollout_gate(
        baseline_data, role=contract.BASELINE_ROLE, case_id=case_id
    )
    candidate_gate = common_rollout_gate(
        candidate_data, role=contract.CANDIDATE_ROLE, case_id=case_id
    )
    rows: list[dict[str, Any]] = []
    baseline_metrics = _mapping(baseline_gate.get("comparison_metrics"))
    candidate_metrics = _mapping(candidate_gate.get("comparison_metrics"))
    for name, absolute, relative in contract.NONINFERIORITY_TOLERANCES:
        reference = _finite(baseline_metrics.get(name), minimum=0.0)
        observed = _finite(candidate_metrics.get(name), minimum=0.0)
        tolerance = (
            max(float(absolute), float(relative) * abs(reference))
            if reference is not None
            else None
        )
        cap = (
            reference + tolerance
            if reference is not None and tolerance is not None
            else None
        )
        comparison_cap = math.nextafter(cap, math.inf) if cap is not None else None
        passed = (
            observed is not None
            and comparison_cap is not None
            and observed <= comparison_cap
        )
        rows.append(
            {
                "metric": name,
                "baseline": reference,
                "candidate": observed,
                "absolute_tolerance": absolute,
                "relative_tolerance": relative,
                "cap": cap,
                "passed": passed,
            }
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
    checks = {
        "baseline_common": baseline_gate.get("passed") is True,
        "candidate_common": candidate_gate.get("passed") is True,
        "condition_exact": all(
            baseline_data.get(name) == candidate_data.get(name)
            for name in condition_fields
        ),
        "all_noninferiority_rows": bool(rows)
        and all(row["passed"] is True for row in rows),
        "same_candidate": baseline_data.get("candidate_id")
        == candidate_data.get("candidate_id")
        == baseline_gate.get("candidate_id"),
        "no_compensation": baseline_data.get("compensation_or_averaging_used") is False
        and candidate_data.get("compensation_or_averaging_used") is False,
    }
    passed = all(checks.values())
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.PAIR_PASS_STATUS if passed else contract.PAIR_FAIL_STATUS,
        "passed": passed,
        "protocol_id": contract.PROTOCOL_ID,
        "candidate_id": baseline_gate.get("candidate_id"),
        "case_id": case_id,
        "checks": checks,
        "noninferiority_rows": rows,
        "compensation_or_averaging_used": False,
        "runtime_promoted": False,
    }


def aggregate_qualification_gate(summary: Any) -> dict[str, Any]:
    """Require baseline-first 12/12 rollouts and six uncompensated pair passes."""

    try:
        binding = prerequisites.current_candidate_binding()
    except prerequisites.V12R8Q3PrerequisiteError:
        return _failure(contract.AGGREGATE_FAIL_STATUS, "r8_candidate_bound")
    data = _mapping(summary)
    rows = data.get("pair_bindings")
    rows_ok = isinstance(rows, list) and len(rows) == len(contract.CASE_IDS)
    if rows_ok:
        for row, case_id in zip(rows, contract.CASE_IDS, strict=True):
            current = _mapping(row)
            if (
                set(current)
                != {
                    "case_id",
                    "passed",
                    "pair_gate",
                    "baseline_receipt",
                    "candidate_receipt",
                }
                or current.get("case_id") != case_id
                or current.get("passed") is not True
                or not artifacts.artifact_record_matches(
                    current.get("pair_gate"), contract.pair_gate_path(case_id)
                )
                or not artifacts.artifact_record_matches(
                    current.get("baseline_receipt"),
                    contract.rollout_receipt_path(contract.BASELINE_ROLE, case_id),
                )
                or not artifacts.artifact_record_matches(
                    current.get("candidate_receipt"),
                    contract.rollout_receipt_path(contract.CANDIDATE_ROLE, case_id),
                )
            ):
                rows_ok = False
                break
    checks = {
        "schema": data.get("schema_version") == contract.SCHEMA_VERSION,
        "status": data.get("status") == contract.AGGREGATE_COMPLETE_STATUS,
        "protocol": data.get("protocol_id") == contract.PROTOCOL_ID,
        "candidate": data.get("candidate_id") == binding["candidate_id"]
        and data.get("candidate_module") == binding["candidate_module"],
        "baseline_first": _type_strict_equal(
            data.get("rollout_matrix"), list(contract.ROLLOUT_MATRIX)
        )
        and _counter(data.get("baseline_rollout_count")) == 6
        and _counter(data.get("candidate_rollout_count")) == 6
        and _counter(data.get("total_rollout_count")) == 12,
        "six_of_six": rows_ok
        and _counter(data.get("pair_count")) == 6
        and _counter(data.get("passing_pair_count")) == 6
        and _counter(data.get("failed_pair_count")) == 0
        and data.get("compensation_or_averaging_used") is False,
        "zero_updates": _counter(data.get("actor_updates")) == 0
        and _counter(data.get("critic_updates")) == 0
        and _counter(data.get("ppo_updates")) == 0,
        "closed_unpromoted": data.get("runtime_promoted") is False
        and data.get("checkpoint_zero_created") is False
        and type(data.get("morphology_weight")) is float
        and data.get("morphology_weight") == 0.0
        and data.get("positive_morphology_enabled") is False
        and data.get("retry_authorized") is False
        and data.get("resume_authorized") is False
        and data.get("rescue_authorized") is False
        and data.get("sweep_authorized") is False
        and data.get("post_hoc_tuning_authorized") is False,
    }
    passed = all(checks.values())
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.AGGREGATE_PASS_STATUS
        if passed
        else contract.AGGREGATE_FAIL_STATUS,
        "passed": passed,
        "protocol_id": contract.PROTOCOL_ID,
        "candidate_id": binding["candidate_id"],
        "checks": checks,
        "candidate_promoted": False,
        "runtime_promoted": False,
        "next_stage": contract.NEXT_STAGE_AFTER_Q3_PASS if passed else "STOP",
    }


__all__ = [
    "aggregate_qualification_gate",
    "common_rollout_gate",
    "condition_matched_gate",
    "morphology_zero_ab_gate",
    "v26_event_gate",
]
