"""Pure fail-closed gates for the latent-live V12R9 morphology A/B."""

from __future__ import annotations

import hashlib
import json
import math
import struct
from collections.abc import Mapping, Sequence
from pathlib import PurePosixPath
from typing import Any

try:
    from . import h0_v12r9_morphology_contract as contract
except ImportError:
    import h0_v12r9_morphology_contract as contract


_ARTIFACT_KEYS = frozenset({"path", "sha256", "size_bytes"})
_TREE_KEYS = frozenset({"path", "tree_sha256", "file_count", "files"})
_TREE_FILE_KEYS = frozenset({"path", "sha256", "size_bytes"})
_STREAM_KEYS = frozenset({"sha256", "size_bytes", "sample_count", "encoding"})
_BINDING_KEYS = frozenset(
    {
        "candidate_id",
        "candidate_module",
        "actor_digest",
        "checkpoint",
        "resolved_config",
    }
)
_REWARD_ROW_KEYS = frozenset(
    {
        "step",
        "actual_reward",
        "recomputed_reward",
        "reward_without_morphology",
        "morphology_loss",
        "morphology_term",
    }
)
_CAUSAL_ROW_KEYS = frozenset({"step", "samples", "diagnostics"})
_ARM_ARTIFACT_KEYS = frozenset(
    {"trace", "reward_ledger", "causal_ledger", "noise_tape", "module_export"}
)


def _mapping(value: Any) -> dict[str, Any]:
    return dict(value) if isinstance(value, Mapping) else {}


def _sequence(value: Any) -> list[Any]:
    if isinstance(value, Sequence) and not isinstance(value, (str, bytes, bytearray)):
        return list(value)
    return []


def _strict_equal(value: Any, expected: Any) -> bool:
    if isinstance(expected, Mapping):
        return (
            isinstance(value, Mapping)
            and set(value) == set(expected)
            and all(_strict_equal(value[key], expected[key]) for key in expected)
        )
    if isinstance(expected, (list, tuple)):
        return (
            type(value) is type(expected)
            and len(value) == len(expected)
            and all(
                _strict_equal(item, target)
                for item, target in zip(value, expected, strict=True)
            )
        )
    return type(value) is type(expected) and value == expected


def canonical_json_bytes(value: Any) -> bytes:
    return json.dumps(
        value,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=False,
        allow_nan=False,
    ).encode("utf-8")


def canonical_json_sha256(value: Any) -> str:
    try:
        return hashlib.sha256(canonical_json_bytes(value)).hexdigest()
    except (TypeError, ValueError):
        return ""


def _sha(value: Any) -> bool:
    return (
        isinstance(value, str)
        and len(value) == 64
        and value == value.lower()
        and all(character in "0123456789abcdef" for character in value)
    )


def _zero_int(value: Any) -> bool:
    return type(value) is int and value == 0


def _nonnegative_int(value: Any) -> bool:
    return type(value) is int and value >= 0


def _positive_int(value: Any) -> bool:
    return type(value) is int and value > 0


def _finite(value: Any, *, minimum: float | None = None) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    result = float(value)
    if not math.isfinite(result) or (minimum is not None and result < minimum):
        return None
    return result


def _float_bits_equal(left: Any, right: Any) -> bool:
    if type(left) is not float or type(right) is not float:
        return False
    if not math.isfinite(left) or not math.isfinite(right):
        return False
    return struct.pack("!d", left) == struct.pack("!d", right)


def _canonical_relative_path(value: Any) -> bool:
    if (
        not isinstance(value, str)
        or not value
        or value == "."
        or "\\" in value
        or "\x00" in value
        or ":" in value
    ):
        return False
    path = PurePosixPath(value)
    return (
        not path.is_absolute()
        and path.as_posix() == value
        and all(part not in {"", ".", ".."} for part in path.parts)
    )


def artifact_record_valid(value: Any, *, expected_path: str | None = None) -> bool:
    data = _mapping(value)
    return (
        set(data) == _ARTIFACT_KEYS
        and _canonical_relative_path(data.get("path"))
        and (expected_path is None or data.get("path") == expected_path)
        and _sha(data.get("sha256"))
        and _positive_int(data.get("size_bytes"))
    )


def _tree_digest(rows: Sequence[Mapping[str, Any]]) -> str:
    digest = hashlib.sha256()
    for row in rows:
        digest.update(str(row["path"]).encode("utf-8"))
        digest.update(b"\0")
        digest.update(str(row["sha256"]).encode("ascii"))
        digest.update(b"\0")
        digest.update(str(row["size_bytes"]).encode("ascii"))
        digest.update(b"\n")
    return digest.hexdigest()


def tree_record_valid(
    value: Any,
    *,
    expected_path: str | None = None,
    required_files: Sequence[str] | None = None,
) -> bool:
    data = _mapping(value)
    rows = _sequence(data.get("files"))
    if (
        set(data) != _TREE_KEYS
        or not _canonical_relative_path(data.get("path"))
        or (expected_path is not None and data.get("path") != expected_path)
        or not _sha(data.get("tree_sha256"))
        or not _positive_int(data.get("file_count"))
        or data.get("file_count") != len(rows)
    ):
        return False
    normalized: list[dict[str, Any]] = []
    for raw in rows:
        row = _mapping(raw)
        if (
            set(row) != _TREE_FILE_KEYS
            or not _canonical_relative_path(row.get("path"))
            or not _sha(row.get("sha256"))
            or not _positive_int(row.get("size_bytes"))
        ):
            return False
        normalized.append(row)
    paths = [row["path"] for row in normalized]
    return (
        paths == sorted(paths)
        and len(paths) == len(set(paths))
        and _tree_digest(normalized) == data["tree_sha256"]
        and (required_files is None or set(paths) == set(required_files))
    )


def stream_record_valid(value: Any, *, sample_count: int | None = None) -> bool:
    data = _mapping(value)
    return (
        set(data) == _STREAM_KEYS
        and _sha(data.get("sha256"))
        and _positive_int(data.get("size_bytes"))
        and _positive_int(data.get("sample_count"))
        and (sample_count is None or data.get("sample_count") == sample_count)
        and data.get("encoding") == contract.STREAM_ENCODING
    )


def candidate_module_valid(value: Any) -> bool:
    return tree_record_valid(
        value,
        expected_path=contract.CANDIDATE_MODULE_PATH.as_posix(),
        required_files=contract.CANDIDATE_REQUIRED_FILES,
    )


def checkpoint_valid(value: Any) -> bool:
    data = _mapping(value)
    paths = {
        row.get("path")
        for row in _sequence(data.get("files"))
        if isinstance(row, Mapping)
    }
    return tree_record_valid(
        data, expected_path=contract.CHECKPOINT_ZERO_PATH.as_posix()
    ) and contract.zero.CHECKPOINT_REQUIRED_SUFFIXES.issubset(paths)


def binding_valid(value: Any) -> bool:
    data = _mapping(value)
    candidate = _mapping(data.get("candidate_module"))
    return (
        set(data) == _BINDING_KEYS
        and candidate_module_valid(candidate)
        and data.get("candidate_id")
        == contract.candidate_id_for_tree(candidate.get("tree_sha256", ""))
        and _sha(data.get("actor_digest"))
        and checkpoint_valid(data.get("checkpoint"))
        and artifact_record_valid(
            data.get("resolved_config"),
            expected_path=contract.TRAINING_CONFIG_PATH.as_posix(),
        )
    )


def _semantic_attestation_valid(
    value: Any, *, endpoint: Mapping[str, Any], result: Mapping[str, Any]
) -> bool:
    data = _mapping(value)
    return (
        set(data)
        == {
            "endpoint",
            "artifact",
            "verifier_module",
            "verifier",
            "verified_result_sha256",
            "verifier_returned_mapping",
        }
        and _strict_equal(data.get("endpoint"), endpoint)
        and artifact_record_valid(
            data.get("artifact"), expected_path=str(endpoint["path"])
        )
        and data.get("verifier_module") == endpoint["verifier_module"]
        and data.get("verifier") == endpoint["verifier"]
        and data.get("verified_result_sha256") == canonical_json_sha256(result)
        and data.get("verifier_returned_mapping") is True
    )


def upstream_terminal_gate(payload: Any) -> dict[str, Any]:
    """Bind three native semantic PASS results to one candidate/checkpoint."""

    data = _mapping(payload)
    r9_result = _mapping(data.get("v12r9_terminal"))
    q3_result = _mapping(data.get("v12r9_q3_terminal"))
    zero_result = _mapping(data.get("v12r9_zero_terminal"))
    attestations = _mapping(data.get("semantic_attestations"))
    r9_module = _mapping(r9_result.get("candidate_module"))
    q3_module = _mapping(q3_result.get("candidate_module"))
    zero_binding = _mapping(zero_result.get("binding"))
    zero_module = _mapping(zero_binding.get("candidate_module"))
    resolved = _mapping(data.get("resolved_config"))
    r9_endpoint = contract.R9_TERMINAL_ENDPOINT
    q3_endpoint = contract.Q3_TERMINAL_ENDPOINT
    zero_endpoint = contract.ZERO_TERMINAL_ENDPOINT
    checks = {
        "semantic_r9": _semantic_attestation_valid(
            attestations.get("v12r9_terminal"),
            endpoint=r9_endpoint,
            result=r9_result,
        ),
        "semantic_q3": _semantic_attestation_valid(
            attestations.get("v12r9_q3_terminal"),
            endpoint=q3_endpoint,
            result=q3_result,
        ),
        "semantic_zero": _semantic_attestation_valid(
            attestations.get("v12r9_zero_terminal"),
            endpoint=zero_endpoint,
            result=zero_result,
        ),
        "r9_pass": r9_result.get("schema_version") == r9_endpoint["schema_version"]
        and r9_result.get("status") == r9_endpoint["required_status"]
        and r9_result.get("passed") is True
        and r9_result.get("terminal") is True
        and r9_result.get("protocol_id") == r9_endpoint["protocol_id"]
        and r9_result.get("pipeline_id") == r9_endpoint["pipeline_id"]
        and r9_result.get("error") is None
        and r9_result.get("actor_updates") == 1
        and r9_result.get("critic_updates") == 0
        and r9_result.get("ppo_updates") == 0
        and r9_result.get("runtime_promoted") is False
        and r9_result.get("qualification_executed") is False
        and candidate_module_valid(r9_module),
        "q3_pass": q3_result.get("schema_version") == q3_endpoint["schema_version"]
        and q3_result.get("status") == q3_endpoint["required_status"]
        and q3_result.get("passed") is True
        and q3_result.get("terminal") is True
        and q3_result.get("protocol_id") == q3_endpoint["protocol_id"]
        and q3_result.get("pipeline_id") == q3_endpoint["pipeline_id"]
        and q3_result.get("error") is None
        and q3_result.get("error_type") is None
        and q3_result.get("actor_updates") == 0
        and q3_result.get("critic_updates") == 0
        and q3_result.get("ppo_updates") == 0
        and q3_result.get("morphology_weight") == 0.0
        and q3_result.get("positive_morphology_enabled") is False
        and q3_result.get("checkpoint_zero_created") is False
        and q3_result.get("runtime_promoted") is False
        and candidate_module_valid(q3_module),
        "zero_pass": zero_result.get("schema_version")
        == zero_endpoint["schema_version"]
        and zero_result.get("status") == zero_endpoint["required_status"]
        and zero_result.get("passed") is True
        and zero_result.get("terminal") is True
        and zero_result.get("error") is None
        and zero_result.get("protocol_id") == zero_endpoint["protocol_id"]
        and zero_result.get("pipeline_id") == zero_endpoint["pipeline_id"]
        and zero_result.get("actor_updates") == 0
        and zero_result.get("critic_updates") == 0
        and zero_result.get("ppo_updates") == 0
        and zero_result.get("environment_samples") == 0
        and zero_result.get("training_executed") is False
        and zero_result.get("required_working_directory")
        == contract.REQUIRED_WORKING_DIRECTORY
        and zero_result.get("positive_live_config_restore_smoke_passed") is True
        and zero_result.get("training_authorized") is False
        and zero_result.get("training_command_published") is False
        and zero_result.get("next_stage") == contract.zero.NEXT_STAGE_AFTER_ZERO_PASS
        and candidate_module_valid(zero_module)
        and _sha(zero_binding.get("actor_digest"))
        and checkpoint_valid(zero_result.get("checkpoint")),
        "same_candidate": r9_result.get("candidate_id")
        == q3_result.get("candidate_id")
        == zero_binding.get("candidate_id")
        and _strict_equal(r9_module, q3_module)
        and _strict_equal(r9_module, zero_module),
        "resolved_config": artifact_record_valid(
            resolved, expected_path=contract.TRAINING_CONFIG_PATH.as_posix()
        ),
    }
    passed = all(checks.values())
    binding = (
        {
            "candidate_id": zero_binding.get("candidate_id"),
            "candidate_module": zero_module,
            "actor_digest": zero_binding.get("actor_digest"),
            "checkpoint": copy_mapping(zero_result.get("checkpoint")),
            "resolved_config": resolved,
        }
        if passed
        else None
    )
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": (
            "PASS_H0_V12R9_MORPH_UPSTREAMS"
            if passed
            else "FAIL_H0_V12R9_MORPH_UPSTREAMS"
        ),
        "passed": passed,
        "checks": checks,
        "binding": binding,
    }


def copy_mapping(value: Any) -> dict[str, Any]:
    return json.loads(json.dumps(_mapping(value), allow_nan=False))


def q3_control_capability_gate(value: Any) -> dict[str, Any]:
    """Require the observed current Q3 trace ABI and force paired reruns."""

    data = _mapping(value)
    case_ids = data.get("case_ids")
    trace_artifacts = _mapping(data.get("trace_artifacts"))
    observed = _mapping(data.get("observed_streams"))
    missing = _mapping(data.get("missing_required_streams"))
    exact_missing = [
        name for name in contract.Q3_REQUIRED_REUSE_STREAMS if name != "actions"
    ]
    checks = {
        "identity": data.get("status") == "INSPECTED_H0_V12R9_Q3_CONTROL_CAPABILITY"
        and data.get("q3_schema_version") == contract.q3.SCHEMA_VERSION
        and data.get("q3_protocol_id") == contract.q3.PROTOCOL_ID,
        "six_cases": case_ids == list(contract.CASE_IDS)
        and set(trace_artifacts) == set(contract.CASE_IDS)
        and set(observed) == set(contract.CASE_IDS)
        and set(missing) == set(contract.CASE_IDS),
        "trace_records": all(
            artifact_record_valid(
                trace_artifacts[case_id],
                expected_path=(
                    PurePosixPath(
                        contract.q3.canonical_rollout(
                            contract.q3.CANDIDATE_ROLE, case_id
                        )["destination"]
                    )
                    / "trace.json"
                ).as_posix(),
            )
            for case_id in contract.CASE_IDS
        )
        if set(trace_artifacts) == set(contract.CASE_IDS)
        else False,
        "observed_abi": all(
            set(_mapping(observed.get(case_id)))
            == set(contract.Q3_REQUIRED_REUSE_STREAMS)
            and all(
                type(flag) is bool for flag in _mapping(observed.get(case_id)).values()
            )
            and _mapping(observed[case_id]).get("actions") is True
            and missing.get(case_id) == exact_missing
            for case_id in contract.CASE_IDS
        ),
        "insufficient": data.get("q3_trace_sufficient") is False
        and all(
            type(missing.get(case_id)) is list and missing[case_id] == exact_missing
            for case_id in contract.CASE_IDS
        ),
        "forced_fallback": data.get("selected_control_source_mode")
        == contract.CONTROL_SOURCE_MODE
        == "paired_rerun",
    }
    passed = all(checks.values())
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": (
            "PASS_H0_V12R9_MORPH_Q3_CAPABILITY_FALLBACK"
            if passed
            else "FAIL_H0_V12R9_MORPH_Q3_CAPABILITY_FALLBACK"
        ),
        "passed": passed,
        "checks": checks,
        "selected_control_source_mode": (
            contract.CONTROL_SOURCE_MODE if passed else None
        ),
    }


def condition_valid(value: Any) -> bool:
    data = _mapping(value)
    if set(data) != {"holdout_case", "noise_record", "condition_sha256"}:
        return False
    holdout = _mapping(data.get("holdout_case"))
    try:
        expected = contract.canonical_case(str(holdout.get("case_id")))
    except ValueError:
        return False
    noise = _mapping(data.get("noise_record"))
    artifact = _mapping(noise.get("artifact"))
    expected_hash = contract.EXPECTED_TAPE_ARRAY_SHA256.get(
        PurePosixPath(expected["noise_tape"]).name
    )
    core = {"holdout_case": holdout, "noise_record": noise}
    return (
        _strict_equal(holdout, expected)
        and set(noise) == {"artifact", "array_sha256"}
        and artifact_record_valid(artifact, expected_path=expected["noise_tape"])
        and noise.get("array_sha256") == expected_hash
        and data.get("condition_sha256") == canonical_json_sha256(core)
    )


def source_closure_gate(value: Any) -> dict[str, Any]:
    data = _mapping(value)
    sources = _mapping(data.get("sources"))
    inputs = _mapping(data.get("inputs"))
    checks = {
        "sources": set(sources) == set(contract.SOURCE_RELATIVE_PATHS)
        and all(
            artifact_record_valid(
                sources[name], expected_path=contract.SOURCE_RELATIVE_PATHS[name]
            )
            for name in contract.SOURCE_RELATIVE_PATHS
        )
        and _mapping(sources.get("causal_corridor")).get("sha256")
        == contract.FROZEN_CORRIDOR_SHA256,
        "inputs": set(inputs) == set(contract.INPUT_RELATIVE_PATHS)
        and all(
            artifact_record_valid(
                inputs[name], expected_path=contract.INPUT_RELATIVE_PATHS[name]
            )
            for name in contract.INPUT_RELATIVE_PATHS
        ),
        "candidate": candidate_module_valid(data.get("candidate_module")),
        "checkpoint": checkpoint_valid(data.get("checkpoint")),
    }
    return {"passed": all(checks.values()), "checks": checks}


def exact_config_delta_gate(control: Any, positive: Any) -> bool:
    first = _mapping(control)
    second = _mapping(positive)
    changed = {name for name in first if second.get(name) != first[name]}
    return (
        _strict_equal(first, contract.CONTROL_REWARD_CONFIG)
        and _strict_equal(second, contract.POSITIVE_REWARD_CONFIG)
        and set(first) == set(second)
        and changed == set(contract.POSITIVE_CONFIG_DELTA_FIELDS)
    )


def detector_audit_gate(value: Any) -> dict[str, Any]:
    data = _mapping(value)
    checks = {
        "config": _strict_equal(
            data.get("runtime_config"), contract.V26_RUNTIME_CONFIG
        ),
        "samples": data.get("sample_count")
        == contract.EXPECTED_RAW_SAMPLES_PER_ROLLOUT,
        "zero_anomalies": all(
            _zero_int(data.get(name))
            for name in (
                "fallback_count",
                "non_v26_source_count",
                "duplicate_event_count",
                "out_of_order_event_count",
                "hard_invalid_count",
            )
        ),
        "passed": data.get("passed") is True,
    }
    return {"passed": all(checks.values()), "checks": checks}


def causal_runtime_gate(value: Any) -> dict[str, Any]:
    data = _mapping(value)
    attestation = _mapping(data.pop("live_attestation", None))
    runtime_checks = _mapping(attestation.get("checks"))
    checks = {
        "config": _strict_equal(data, contract.CAUSAL_RUNTIME_CONFIG),
        "live": attestation.get("passed") is True
        and attestation.get("runtime_id") == contract.CAUSAL_RUNTIME_ID
        and set(runtime_checks) == {"runtime_id", "corridor", "reward"}
        and all(item is True for item in runtime_checks.values()),
    }
    return {"passed": all(checks.values()), "checks": checks}


def reward_ledger_gate(rows: Any, *, role: str) -> dict[str, Any]:
    values = _sequence(rows)
    failed: list[int] = []
    nonzero = 0
    total_loss = 0.0
    total_penalty = 0.0
    for index, raw in enumerate(values):
        row = _mapping(raw)
        if set(row) != _REWARD_ROW_KEYS or row.get("step") != index + 1:
            failed.append(index + 1)
            continue
        numeric = [row[name] for name in _REWARD_ROW_KEYS if name != "step"]
        if any(type(item) is not float or not math.isfinite(item) for item in numeric):
            failed.append(index + 1)
            continue
        loss = row["morphology_loss"]
        if loss < 0.0:
            failed.append(index + 1)
            continue
        expected_term = 0.0 if role == contract.CONTROL_ROLE else 0.0025 * loss
        expected_reward = row["reward_without_morphology"] - expected_term
        if not (
            _float_bits_equal(row["morphology_term"], float(expected_term))
            and _float_bits_equal(row["actual_reward"], float(expected_reward))
            and _float_bits_equal(row["recomputed_reward"], float(expected_reward))
        ):
            failed.append(index + 1)
            continue
        nonzero += int(expected_term > 0.0)
        total_loss += loss
        total_penalty += expected_term
    passed = (
        role in contract.PAIR_ROLE_ORDER
        and len(values) == contract.EXPECTED_STEPS_PER_ROLLOUT
        and not failed
    )
    return {
        "passed": passed,
        "row_count": len(values),
        "failed_steps": failed,
        "nonzero_effect_sample_count": nonzero,
        "total_morphology_loss": float(total_loss),
        "total_morphology_penalty": float(total_penalty),
    }


def causal_ledger_gate(rows: Any, *, reward_rows: Any) -> dict[str, Any]:
    values = _sequence(rows)
    rewards = _sequence(reward_rows)
    failed: list[int] = []
    sample_count = 0
    failed_closed_count = 0
    unexpected_drop_count = 0
    drop_reason_counts: dict[str, int] = {}
    minimum_delay = float("inf")
    maximum_delivery_latency = 0.0
    for index, raw in enumerate(values):
        row = _mapping(raw)
        if set(row) != _CAUSAL_ROW_KEYS or row.get("step") != index + 1:
            failed.append(index + 1)
            continue
        samples = _sequence(row.get("samples"))
        diagnostics = _mapping(row.get("diagnostics"))
        counter_names = (
            "dropped_sample_count",
            "dropped_pending_sample_count",
            "dropped_wait_hs_sample_count",
            "pending_sample_count",
            "resolved_sample_count",
            "total_resolved_sample_count",
            "total_dropped_sample_count",
            "cancelled_transition_count",
            "total_cancelled_transition_count",
            "timeout_transition_count",
        )
        if (
            set(diagnostics) != set(contract.CAUSAL_DIAGNOSTIC_FIELDS)
            or diagnostics.get("event_contract_id") != contract.V26_EVENT_CONTRACT_ID
            or not _float_bits_equal(
                diagnostics.get("delay_s"), contract.MORPHOLOGY_REWARD_DELAY_S
            )
            or type(diagnostics.get("failed_closed")) is not bool
            or diagnostics.get("failure_reason") != ""
            or not isinstance(diagnostics.get("drop_reason"), str)
            or any(
                not _nonnegative_int(diagnostics.get(name)) for name in counter_names
            )
            or diagnostics.get("resolved_sample_count") != len(samples)
            or type(diagnostics.get("terminal_flushed")) is not bool
            or not isinstance(diagnostics.get("actor_state_name"), str)
            or type(diagnostics.get("partial_stance_active")) is not bool
        ):
            failed.append(index + 1)
            continue
        failed_closed_count += int(diagnostics["failed_closed"])
        dropped = diagnostics.get("dropped_sample_count")
        drop_reason = diagnostics.get("drop_reason")
        if not _nonnegative_int(dropped) or not isinstance(drop_reason, str):
            failed.append(index + 1)
            continue
        reasons = [item for item in drop_reason.split("|") if item]
        if dropped and not reasons:
            unexpected_drop_count += dropped
        for reason in reasons:
            if reason not in contract.EXPECTED_CAUSAL_DROP_REASONS:
                unexpected_drop_count += dropped
            else:
                drop_reason_counts[reason] = drop_reason_counts.get(reason, 0) + dropped
        knee_loss_sum = 0.0
        ankle_loss_sum = 0.0
        for raw_sample in samples:
            sample = _mapping(raw_sample)
            if set(sample) != set(contract.CAUSAL_SAMPLE_FIELDS):
                failed.append(index + 1)
                continue
            numeric_names = set(contract.CAUSAL_SAMPLE_FIELDS) - {
                "terminal_flush",
                "segment_type",
            }
            if any(_finite(sample.get(name)) is None for name in numeric_names):
                failed.append(index + 1)
                continue
            if type(sample.get("terminal_flush")) is not bool or sample.get(
                "segment_type"
            ) not in {"stance", "swing"}:
                failed.append(index + 1)
                continue
            emission_delay = float(sample["emitted_time_s"] - sample["time_s"])
            delivery_latency = float(
                sample["anchor_delivered_time_s"] - sample["anchor_confirmed_time_s"]
            )
            if (
                emission_delay + 1.0e-12 < contract.MORPHOLOGY_REWARD_DELAY_S
                or abs(float(sample["delay_s"]) - emission_delay) > 1.0e-12
                or delivery_latency < -1.0e-12
                or delivery_latency
                > contract.MORPHOLOGY_MAX_DELIVERY_LATENCY_S + 1.0e-12
                or float(sample["knee_loss"]) < 0.0
                or float(sample["ankle_loss"]) < 0.0
            ):
                failed.append(index + 1)
                continue
            minimum_delay = min(minimum_delay, emission_delay)
            maximum_delivery_latency = max(maximum_delivery_latency, delivery_latency)
            # Match the production accumulator order exactly before applying
            # the final 0.5 factor; an algebraically equivalent per-sample sum
            # can differ by one ULP and is not acceptable evidence here.
            knee_loss_sum += float(sample["knee_loss"])
            ankle_loss_sum += float(sample["ankle_loss"])
            sample_count += 1
        loss_from_samples = 0.5 * (knee_loss_sum + ankle_loss_sum)
        if index >= len(rewards):
            failed.append(index + 1)
        else:
            reward = _mapping(rewards[index])
            if type(
                reward.get("morphology_loss")
            ) is not float or not _float_bits_equal(
                reward["morphology_loss"], float(loss_from_samples)
            ):
                failed.append(index + 1)
    passed = (
        len(values) == contract.EXPECTED_STEPS_PER_ROLLOUT
        and len(rewards) == contract.EXPECTED_STEPS_PER_ROLLOUT
        and sample_count > 0
        and failed_closed_count == 0
        and unexpected_drop_count == 0
        and not failed
    )
    return {
        "passed": passed,
        "row_count": len(values),
        "sample_count": sample_count,
        "failed_steps": sorted(set(failed)),
        "failed_closed_count": failed_closed_count,
        "unexpected_drop_count": unexpected_drop_count,
        "drop_reason_counts": drop_reason_counts,
        "minimum_emission_delay_s": (
            float(minimum_delay) if math.isfinite(minimum_delay) else None
        ),
        "maximum_delivery_latency_s": float(maximum_delivery_latency),
        "ledger_rows_sha256": canonical_json_sha256(values),
    }


def arm_gate(
    value: Any,
    *,
    expected_binding: Mapping[str, Any],
    expected_condition: Mapping[str, Any],
    reward_rows: Any,
    causal_rows: Any,
) -> dict[str, Any]:
    data = _mapping(value)
    role = data.get("role")
    streams = _mapping(data.get("streams"))
    artifacts = _mapping(data.get("artifacts"))
    reward = reward_ledger_gate(reward_rows, role=str(role))
    causal = causal_ledger_gate(causal_rows, reward_rows=reward_rows)
    checks = {
        "identity": data.get("schema_version") == contract.SCHEMA_VERSION
        and data.get("status") == contract.ARM_COMPLETE_STATUS
        and data.get("protocol_id") == contract.PROTOCOL_ID
        and data.get("case_id") in contract.CASE_IDS
        and role in contract.PAIR_ROLE_ORDER,
        "binding_condition": _strict_equal(data.get("binding"), expected_binding)
        and binding_valid(expected_binding)
        and _strict_equal(data.get("condition"), expected_condition)
        and condition_valid(expected_condition),
        "config": _strict_equal(
            data.get("reward_config"), contract.reward_config_for_role(str(role))
        ),
        "artifacts": set(artifacts) == _ARM_ARTIFACT_KEYS
        and all(
            artifact_record_valid(artifacts[name])
            for name in ("trace", "reward_ledger", "causal_ledger", "noise_tape")
        )
        and tree_record_valid(
            artifacts.get("module_export"),
            expected_path=contract.MODULE_EXPORT_PATH.as_posix(),
        ),
        "streams": set(streams) == set(contract.STREAM_NAMES)
        and all(
            stream_record_valid(
                streams[name], sample_count=contract.EXPECTED_STEPS_PER_ROLLOUT
            )
            for name in contract.STREAM_NAMES
        ),
        "detector": detector_audit_gate(data.get("detector_audit"))["passed"] is True,
        "reward": reward["passed"] is True,
        "causal": causal["passed"] is True
        and _strict_equal(data.get("causal_audit"), causal),
        "causal_runtime": causal_runtime_gate(data.get("causal_runtime"))["passed"]
        is True,
        "activity": data.get("local_execution_index")
        in range(contract.EXPECTED_LOCAL_ROLLOUT_COUNT)
        and _zero_int(data.get("actor_updates"))
        and _zero_int(data.get("critic_updates"))
        and _zero_int(data.get("ppo_updates"))
        and data.get("training_executed") is False,
    }
    passed = all(checks.values())
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.ARM_PASS_STATUS if passed else "FAIL_H0_V12R9_MORPH_ARM",
        "passed": passed,
        "checks": checks,
        "reward": reward,
        "causal": causal,
    }


def pair_gate(
    payload: Any,
    *,
    expected_binding: Mapping[str, Any],
    expected_condition: Mapping[str, Any],
    control_reward_rows: Any,
    positive_reward_rows: Any,
    control_causal_rows: Any,
    positive_causal_rows: Any,
) -> dict[str, Any]:
    data = _mapping(payload)
    try:
        case_index = contract.CASE_IDS.index(data.get("case_id"))
    except ValueError:
        case_index = -1
    control = _mapping(data.get("control"))
    positive = _mapping(data.get("positive"))
    control_streams = _mapping(control.get("streams"))
    positive_streams = _mapping(positive.get("streams"))
    control_gate = arm_gate(
        control,
        expected_binding=expected_binding,
        expected_condition=expected_condition,
        reward_rows=control_reward_rows,
        causal_rows=control_causal_rows,
    )
    positive_gate = arm_gate(
        positive,
        expected_binding=expected_binding,
        expected_condition=expected_condition,
        reward_rows=positive_reward_rows,
        causal_rows=positive_causal_rows,
    )
    control_rewards = _sequence(control_reward_rows)
    positive_rewards = _sequence(positive_reward_rows)
    reward_rows_equal = len(control_rewards) == len(positive_rewards)
    recomposition_exact = reward_rows_equal
    for control_row, positive_row in zip(
        control_rewards, positive_rewards, strict=False
    ):
        left = _mapping(control_row)
        right = _mapping(positive_row)
        if not (
            _float_bits_equal(
                left.get("reward_without_morphology"),
                right.get("reward_without_morphology"),
            )
            and _float_bits_equal(
                left.get("morphology_loss"), right.get("morphology_loss")
            )
            and _float_bits_equal(
                left.get("actual_reward"), left.get("recomputed_reward")
            )
            and _float_bits_equal(
                right.get("actual_reward"), right.get("recomputed_reward")
            )
        ):
            recomposition_exact = False
            break
    checks = {
        "identity": data.get("schema_version") == contract.SCHEMA_VERSION
        and data.get("status") == contract.PAIR_COMPLETE_STATUS
        and data.get("protocol_id") == contract.PROTOCOL_ID
        and data.get("case_id") in contract.CASE_IDS
        and data.get("control_source_mode") == contract.CONTROL_SOURCE_MODE,
        "control_before_positive": _finite(control.get("completed_unix_s"), minimum=0.0)
        is not None
        and _finite(positive.get("started_unix_s"), minimum=0.0) is not None
        and control["completed_unix_s"] < positive["started_unix_s"]
        and case_index >= 0
        and control.get("local_execution_index") == 2 * case_index
        and positive.get("local_execution_index") == 2 * case_index + 1,
        "same_binding_condition": _strict_equal(
            control.get("binding"), expected_binding
        )
        and _strict_equal(positive.get("binding"), expected_binding)
        and _strict_equal(control.get("condition"), expected_condition)
        and _strict_equal(positive.get("condition"), expected_condition),
        "exact_config_delta": exact_config_delta_gate(
            control.get("reward_config"), positive.get("reward_config")
        ),
        "artifacts_hash_bound": control_gate["checks"]["artifacts"]
        and positive_gate["checks"]["artifacts"],
        "v26_integrity": control_gate["checks"]["detector"]
        and positive_gate["checks"]["detector"]
        and _strict_equal(
            control.get("detector_audit"), positive.get("detector_audit")
        ),
        "observations_identical": _strict_equal(
            control_streams.get("observations"), positive_streams.get("observations")
        ),
        "actions_identical": _strict_equal(
            control_streams.get("actions"), positive_streams.get("actions")
        ),
        "dynamics_identical": _strict_equal(
            control_streams.get("dynamics"), positive_streams.get("dynamics")
        ),
        "events_identical": _strict_equal(
            control_streams.get("events"), positive_streams.get("events")
        ),
        "base_reward_and_loss_identical": _strict_equal(
            control_streams.get("reward_without_morphology"),
            positive_streams.get("reward_without_morphology"),
        )
        and _strict_equal(
            control_streams.get("morphology_loss"),
            positive_streams.get("morphology_loss"),
        ),
        "reward_recomposition_exact": control_gate["checks"]["reward"]
        and positive_gate["checks"]["reward"]
        and recomposition_exact,
        "causal_per_sample_valid": control_gate["checks"]["causal"]
        and positive_gate["checks"]["causal"],
        "causal_diagnostics_identical": _strict_equal(
            control.get("causal_audit"), positive.get("causal_audit")
        )
        and canonical_json_sha256(control_causal_rows)
        == canonical_json_sha256(positive_causal_rows),
        "causal_runtime_identical": control_gate["checks"]["causal_runtime"]
        and positive_gate["checks"]["causal_runtime"]
        and _strict_equal(
            control.get("causal_runtime"), positive.get("causal_runtime")
        ),
        "zero_updates": control_gate["checks"]["activity"]
        and positive_gate["checks"]["activity"],
    }
    if tuple(checks) != contract.REQUIRED_PAIR_CHECKS:
        raise RuntimeError("pair check contract drifted")
    passed = all(checks.values())
    positive_reward = positive_gate["reward"]
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.PAIR_PASS_STATUS if passed else contract.PAIR_FAIL_STATUS,
        "passed": passed,
        "protocol_id": contract.PROTOCOL_ID,
        "case_id": data.get("case_id"),
        "checks": checks,
        "control_arm_gate": control_gate,
        "positive_arm_gate": positive_gate,
        "effect": {
            "nonzero_effect_sample_count": positive_reward[
                "nonzero_effect_sample_count"
            ],
            "total_morphology_loss": positive_reward["total_morphology_loss"],
            "total_morphology_penalty": positive_reward["total_morphology_penalty"],
            "observed": positive_reward["nonzero_effect_sample_count"] > 0
            and positive_reward["total_morphology_penalty"] > 0.0,
        },
    }


def aggregate_gate(
    pair_results: Any, *, expected_binding: Mapping[str, Any]
) -> dict[str, Any]:
    values = _sequence(pair_results)
    effects = [_mapping(_mapping(item).get("effect")) for item in values]
    effects_valid = all(
        _nonnegative_int(effect.get("nonzero_effect_sample_count"))
        and _finite(effect.get("total_morphology_loss"), minimum=0.0) is not None
        and _finite(effect.get("total_morphology_penalty"), minimum=0.0) is not None
        and type(effect.get("observed")) is bool
        for effect in effects
    )
    nonzero = (
        sum(effect["nonzero_effect_sample_count"] for effect in effects)
        if effects_valid
        else 0
    )
    total_loss = (
        sum(float(effect["total_morphology_loss"]) for effect in effects)
        if effects_valid
        else 0.0
    )
    total_penalty = (
        sum(float(effect["total_morphology_penalty"]) for effect in effects)
        if effects_valid
        else 0.0
    )
    checks = {
        "binding": binding_valid(expected_binding),
        "forced_paired_rerun": contract.CONTROL_SOURCE_MODE == "paired_rerun",
        "six_exact_pairs": len(values) == contract.EXPECTED_PAIR_COUNT
        and [_mapping(item).get("case_id") for item in values]
        == list(contract.CASE_IDS),
        "all_pairs_pass": len(values) == contract.EXPECTED_PAIR_COUNT
        and all(_mapping(item).get("passed") is True for item in values),
        "twelve_local_rollouts": contract.EXPECTED_LOCAL_ROLLOUT_COUNT == 12,
        "effect_observed": nonzero >= contract.MIN_AGGREGATE_NONZERO_EFFECT_SAMPLES
        and effects_valid
        and total_loss > 0.0
        and total_penalty > 0.0,
        "single_weight": contract.MORPHOLOGY_POSITIVE_WEIGHTS == (0.0025,),
    }
    passed = all(checks.values())
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": (
            contract.AGGREGATE_PASS_STATUS if passed else contract.AGGREGATE_FAIL_STATUS
        ),
        "passed": passed,
        "terminal": True,
        "protocol_id": contract.PROTOCOL_ID,
        "binding": copy_mapping(expected_binding)
        if binding_valid(expected_binding)
        else None,
        "checks": checks,
        "pair_results": values,
        "effect": {
            "nonzero_effect_sample_count": nonzero,
            "total_morphology_loss": float(total_loss),
            "total_morphology_penalty": float(total_penalty),
        },
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "training_authorized": passed,
        "next_stage": (
            contract.NEXT_STAGE_AFTER_PASS if passed else contract.NEXT_STAGE_AFTER_FAIL
        ),
    }


def terminal_receipt_gate(value: Any) -> dict[str, Any]:
    data = _mapping(value)
    aggregate = _mapping(data.get("aggregate"))
    checks = {
        "identity": data.get("schema_version") == contract.SCHEMA_VERSION
        and data.get("status") == contract.PIPELINE_TERMINAL_PASS_STATUS
        and data.get("passed") is True
        and data.get("terminal") is True
        and data.get("error") is None
        and data.get("protocol_id") == contract.PROTOCOL_ID
        and data.get("pipeline_id") == contract.PIPELINE_ID,
        "binding": binding_valid(data.get("binding")),
        "aggregate": aggregate.get("status") == contract.AGGREGATE_PASS_STATUS
        and aggregate.get("passed") is True
        and bool(_mapping(aggregate.get("checks")))
        and all(item is True for item in _mapping(aggregate.get("checks")).values()),
        "counts": data.get("pair_count") == contract.EXPECTED_PAIR_COUNT
        and data.get("local_rollout_count") == contract.EXPECTED_LOCAL_ROLLOUT_COUNT
        and data.get("policy_step_count") == contract.EXPECTED_TOTAL_POLICY_STEPS,
        "zero_updates": _zero_int(data.get("actor_updates"))
        and _zero_int(data.get("critic_updates"))
        and _zero_int(data.get("ppo_updates"))
        and data.get("training_executed") is False,
        "authority": data.get("training_authorized") is True
        and data.get("training_command_publication_authorized") is True
        and data.get("training_command_published") is False
        and data.get("required_working_directory")
        == contract.REQUIRED_WORKING_DIRECTORY
        and data.get("next_stage") == contract.NEXT_STAGE_AFTER_PASS,
    }
    return {"passed": all(checks.values()), "checks": checks}


__all__ = [
    "aggregate_gate",
    "arm_gate",
    "artifact_record_valid",
    "binding_valid",
    "candidate_module_valid",
    "canonical_json_bytes",
    "canonical_json_sha256",
    "causal_ledger_gate",
    "causal_runtime_gate",
    "checkpoint_valid",
    "condition_valid",
    "detector_audit_gate",
    "exact_config_delta_gate",
    "pair_gate",
    "q3_control_capability_gate",
    "reward_ledger_gate",
    "source_closure_gate",
    "stream_record_valid",
    "terminal_receipt_gate",
    "tree_record_valid",
    "upstream_terminal_gate",
]
