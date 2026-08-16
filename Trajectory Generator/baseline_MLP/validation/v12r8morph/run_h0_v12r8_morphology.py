"""Latent-live one-shot V12R8 positive morphology qualification.

Importing this module is inert.  The live path becomes reachable only after
the native R8, R8-Q3 and R8-zero semantic verifiers all return terminal PASS
for the same candidate and full checkpoint-zero.  Execution then performs six
strict physical control->positive pairs (twelve rollouts), with no learning,
and publishes a resume handoff only after every byte/hash and causal gate PASS.
"""

from __future__ import annotations

import copy
import json
import math
import os
import sys
import time
from collections.abc import Callable, Mapping, Sequence
from pathlib import Path, PurePosixPath
from typing import Any

try:
    from . import h0_v12r8_morphology_contract as contract
    from . import h0_v12r8_morphology_gates as gates
except ImportError:
    import h0_v12r8_morphology_contract as contract
    import h0_v12r8_morphology_gates as gates


HERE = Path(__file__).resolve().parent
VALIDATION_ROOT = HERE.parent
BASELINE_ROOT = VALIDATION_ROOT.parent
REPO_ROOT = HERE.parents[3]
for _root in (
    HERE,
    BASELINE_ROOT,
    VALIDATION_ROOT / "v12r8",
    VALIDATION_ROOT / "v12r8q3",
    VALIDATION_ROOT / "v12r8q3" / "runtime",
    VALIDATION_ROOT / "v12r8zero",
    REPO_ROOT / "validation",
):
    if str(_root) not in sys.path:
        sys.path.insert(0, str(_root))

import run_h0_v12r8_zero_checkpoint as zero_runner  # noqa: E402


class MorphologyExecutionError(RuntimeError):
    """Raised when a frozen one-shot invariant cannot be proved."""


resolve_relative = zero_runner.resolve_relative
artifact_record = zero_runner.artifact_record
tree_record = zero_runner.tree_record
write_json_exclusive = zero_runner.write_json_exclusive

LOCK = resolve_relative(contract.EXECUTION_LOCK_PATH)
RUN_ROOT = resolve_relative(contract.RUN_ROOT)

LOCK_KEYS = frozenset(
    {
        "schema_version",
        "status",
        "protocol_id",
        "pipeline_id",
        "revision",
        "source_state",
        "protocol_freeze",
        "upstream_endpoints",
        "upstream_payload",
        "upstream_gate",
        "binding",
        "source_closure",
        "source_closure_gate",
        "conditions",
        "q3_control_capabilities",
        "q3_control_capability_gate",
        "control_source_mode",
        "checkpoint_module",
        "case_ids",
        "pair_role_order",
        "expected_pair_count",
        "expected_local_rollout_count",
        "expected_steps_per_rollout",
        "expected_total_policy_steps",
        "control_reward_config",
        "positive_reward_config",
        "v26_runtime_config",
        "causal_runtime_config",
        "profile_attestations",
        "required_pair_checks",
        "actor_updates",
        "critic_updates",
        "ppo_updates",
        "training_authorized",
        "training_command_published",
        "authority",
    }
)

ACTIVITY_TEMPLATE = {
    "environment_reset_calls": 0,
    "environment_step_calls": 0,
    "baseline_actor_queries": 0,
    "candidate_actor_queries": 0,
    "teacher_queries": 0,
    "blend_count": 0,
    "latch_count": 0,
    "actor_updates": 0,
    "critic_updates": 0,
    "ppo_updates": 0,
}
EXPECTED_TERMINAL_ACTIVITY = {
    **ACTIVITY_TEMPLATE,
    "environment_reset_calls": contract.EXPECTED_LOCAL_ROLLOUT_COUNT,
    "environment_step_calls": contract.EXPECTED_TOTAL_POLICY_STEPS,
    "candidate_actor_queries": contract.EXPECTED_TOTAL_POLICY_STEPS,
}

ArmCollector = Callable[..., Mapping[str, Any]]


def _mapping(value: Any) -> dict[str, Any]:
    return dict(value) if isinstance(value, Mapping) else {}


def _sequence(value: Any) -> list[Any]:
    if isinstance(value, Sequence) and not isinstance(value, (str, bytes, bytearray)):
        return list(value)
    return []


def _same_json(left: Any, right: Any) -> bool:
    try:
        return gates.canonical_json_bytes(left) == gates.canonical_json_bytes(right)
    except (TypeError, ValueError):
        return False


def _finite(value: Any) -> bool:
    return type(value) is float and math.isfinite(value)


def strict_json_any(path: str | Path) -> Any:
    """Read strict JSON after rejecting link/reparse traversal and torn reads."""

    target = Path(path)
    zero_runner.reject_link_or_reparse_ancestors(target, include_leaf=True)
    before = zero_runner._regular_file_status(target)  # noqa: SLF001

    def reject_duplicates(pairs: Sequence[tuple[str, Any]]) -> dict[str, Any]:
        output: dict[str, Any] = {}
        for key, child in pairs:
            if key in output:
                raise ValueError(f"duplicate JSON key: {key}")
            output[key] = child
        return output

    try:
        raw = target.read_bytes()
        value = json.loads(
            raw,
            parse_constant=lambda token: (_ for _ in ()).throw(
                ValueError(f"non-finite JSON token: {token}")
            ),
            object_pairs_hook=reject_duplicates,
        )
    except (OSError, UnicodeError, ValueError, json.JSONDecodeError) as exc:
        raise MorphologyExecutionError(f"invalid strict JSON: {target}") from exc
    after = zero_runner._regular_file_status(target)  # noqa: SLF001
    identity_before = (
        before.st_dev,
        before.st_ino,
        before.st_size,
        before.st_mtime_ns,
    )
    identity_after = (
        after.st_dev,
        after.st_ino,
        after.st_size,
        after.st_mtime_ns,
    )
    if identity_before != identity_after or len(raw) != before.st_size:
        raise MorphologyExecutionError(f"JSON changed while reading: {target}")
    zero_runner._require_finite_json(value, os.fspath(target))  # noqa: SLF001
    return value


def strict_mapping(path: str | Path) -> dict[str, Any]:
    value = strict_json_any(path)
    if not isinstance(value, Mapping):
        raise MorphologyExecutionError(f"JSON root is not an object: {path}")
    return dict(value)


def _attestation(
    endpoint: Mapping[str, Any], result: Mapping[str, Any]
) -> dict[str, Any]:
    return {
        "endpoint": copy.deepcopy(dict(endpoint)),
        "artifact": artifact_record(resolve_relative(str(endpoint["path"]))),
        "verifier_module": endpoint["verifier_module"],
        "verifier": endpoint["verifier"],
        "verified_result_sha256": gates.canonical_json_sha256(result),
        "verifier_returned_mapping": True,
    }


def semantic_upstream_payload() -> dict[str, Any]:
    """Invoke all three stable native terminal verifiers, never trust flags."""

    try:
        import run_h0_v12r8_q3_qualification as q3_runner
        import run_h0_v12r8_recovery as r8_runner

        r8_result = r8_runner.verify_terminal_ledger()
        q3_result = q3_runner.verify_terminal_ledger()
        zero_result = zero_runner.verify_terminal_pass()
    except BaseException as exc:
        raise MorphologyExecutionError(
            f"native R8/Q3/zero terminal verification failed: {exc}"
        ) from exc
    if not all(
        isinstance(item, Mapping) for item in (r8_result, q3_result, zero_result)
    ):
        raise MorphologyExecutionError("native terminal verifier returned non-mapping")
    r8_payload = dict(r8_result)
    q3_payload = dict(q3_result)
    zero_payload = dict(zero_result)
    payload = {
        "v12r8_terminal": r8_payload,
        "v12r8_q3_terminal": q3_payload,
        "v12r8_zero_terminal": zero_payload,
        "semantic_attestations": {
            "v12r8_terminal": _attestation(
                contract.R8_TERMINAL_ENDPOINT, r8_payload
            ),
            "v12r8_q3_terminal": _attestation(
                contract.Q3_TERMINAL_ENDPOINT, q3_payload
            ),
            "v12r8_zero_terminal": _attestation(
                contract.ZERO_TERMINAL_ENDPOINT, zero_payload
            ),
        },
        "resolved_config": artifact_record(
            resolve_relative(contract.TRAINING_CONFIG_PATH)
        ),
    }
    gate = gates.upstream_terminal_gate(payload)
    if gate.get("passed") is not True:
        failed = sorted(
            name
            for name, passed in _mapping(gate.get("checks")).items()
            if passed is not True
        )
        raise MorphologyExecutionError(f"upstream semantic gate failed: {failed}")
    return payload


def condition_snapshot() -> dict[str, Any]:
    """Rehash the six Q3 tapes and bind their decoded float32 array digests."""

    try:
        import prepare_h0_v12r8_q3_noise_tapes as noise
    except BaseException as exc:
        raise MorphologyExecutionError("Q3 noise loader is unavailable") from exc
    conditions: dict[str, Any] = {}
    for case_id in contract.CASE_IDS:
        case = contract.canonical_case(case_id)
        _array, loaded_record, array_sha = noise.load_case_tape(case_id)
        observed_record = artifact_record(resolve_relative(case["noise_tape"]))
        if not _same_json(loaded_record, observed_record):
            raise MorphologyExecutionError(f"Q3 noise record drifted: {case_id}")
        noise_record = {
            "artifact": observed_record,
            "array_sha256": array_sha,
        }
        core = {"holdout_case": case, "noise_record": noise_record}
        conditions[case_id] = {
            **core,
            "condition_sha256": gates.canonical_json_sha256(core),
        }
        if not gates.condition_valid(conditions[case_id]):
            raise MorphologyExecutionError(f"Q3 condition gate failed: {case_id}")
    return conditions


def inspect_q3_control_capabilities() -> dict[str, Any]:
    """Inspect the persisted Q3 candidate traces and force the proven fallback."""

    trace_artifacts: dict[str, Any] = {}
    observed: dict[str, Any] = {}
    missing: dict[str, Any] = {}
    for case_id in contract.CASE_IDS:
        destination = PurePosixPath(
            contract.q3.canonical_rollout(contract.q3.CANDIDATE_ROLE, case_id)[
                "destination"
            ]
        )
        path = resolve_relative(destination / "trace.json")
        rows = strict_json_any(path)
        if not isinstance(rows, list) or len(rows) != contract.EXPECTED_STEPS_PER_ROLLOUT:
            raise MorphologyExecutionError(f"Q3 trace length drifted: {case_id}")
        if not all(isinstance(row, Mapping) for row in rows):
            raise MorphologyExecutionError(f"Q3 trace row drifted: {case_id}")
        flags = {
            "observations": all(
                "full_observation_before" in row and "full_observation_after" in row
                for row in rows
            ),
            "actions": all(
                "raw_action" in row and "applied_action" in row for row in rows
            ),
            "dynamics": all("dynamics" in row for row in rows),
            "events": all(
                "binary_phase_sensor_samples" in row and "phase_fsm" in row
                for row in rows
            ),
            "reward_without_morphology": all(
                "reward_without_morphology" in row for row in rows
            ),
            "morphology_loss": all("morphology_loss" in row for row in rows),
            "causal_samples": all(
                "morphology_causal_samples" in row
                and "morphology_causal_diagnostics" in row
                for row in rows
            ),
        }
        observed[case_id] = flags
        missing[case_id] = [name for name, present in flags.items() if not present]
        trace_artifacts[case_id] = artifact_record(path)
    payload = {
        "status": "INSPECTED_H0_V12R8_Q3_CONTROL_CAPABILITY",
        "q3_schema_version": contract.q3.SCHEMA_VERSION,
        "q3_protocol_id": contract.q3.PROTOCOL_ID,
        "case_ids": list(contract.CASE_IDS),
        "trace_artifacts": trace_artifacts,
        "observed_streams": observed,
        "missing_required_streams": missing,
        "q3_trace_sufficient": all(not row for row in missing.values()),
        "selected_control_source_mode": contract.CONTROL_SOURCE_MODE,
    }
    gate = gates.q3_control_capability_gate(payload)
    if gate.get("passed") is not True:
        raise MorphologyExecutionError("Q3 capability fallback gate failed")
    return payload


def source_closure_snapshot() -> dict[str, Any]:
    snapshot = {
        "sources": {
            name: artifact_record(resolve_relative(path))
            for name, path in contract.SOURCE_RELATIVE_PATHS.items()
        },
        "inputs": {
            name: artifact_record(resolve_relative(path))
            for name, path in contract.INPUT_RELATIVE_PATHS.items()
        },
        "candidate_module": tree_record(
            resolve_relative(contract.CANDIDATE_MODULE_PATH)
        ),
        "checkpoint": tree_record(resolve_relative(contract.CHECKPOINT_ZERO_PATH)),
    }
    gate = gates.source_closure_gate(snapshot)
    if gate.get("passed") is not True:
        raise MorphologyExecutionError("source/input closure gate failed")
    return snapshot


def checkpoint_module_snapshot(binding: Mapping[str, Any]) -> dict[str, Any]:
    """Bind the audited post-restore RLModule directly to checkpoint-zero."""

    audit_path = resolve_relative(contract.zero.AUDIT_PATH)
    audit = strict_mapping(audit_path)
    restored = _mapping(audit.get("restored_export"))
    locked_tree = _mapping(restored.get("tree"))
    observed_tree = tree_record(resolve_relative(contract.MODULE_EXPORT_PATH))
    manifest_path = resolve_relative(contract.MODULE_EXPORT_PATH) / (
        "actor_feature_manifest.json"
    )
    manifest = strict_mapping(manifest_path)
    evidence = {
        "zero_audit": artifact_record(audit_path),
        "checkpoint": copy.deepcopy(_mapping(binding.get("checkpoint"))),
        "actor_digest": binding.get("actor_digest"),
        "module_export": observed_tree,
        "module_manifest": artifact_record(manifest_path),
    }
    zero_binding = {
        "candidate_id": binding.get("candidate_id"),
        "candidate_module": copy.deepcopy(_mapping(binding.get("candidate_module"))),
        "actor_digest": binding.get("actor_digest"),
    }
    checks = {
        "audit_binding": _same_json(audit.get("binding"), zero_binding),
        "checkpoint": _same_json(audit.get("checkpoint_tree"), binding.get("checkpoint")),
        "tree": gates.tree_record_valid(
            observed_tree, expected_path=contract.MODULE_EXPORT_PATH.as_posix()
        )
        and _same_json(locked_tree, observed_tree),
        "actor": _mapping(restored.get("actor")).get("actor_digest")
        == binding.get("actor_digest")
        and manifest.get("actor_digest") == binding.get("actor_digest"),
        "manifest": _same_json(restored.get("manifest"), evidence["module_manifest"])
        and manifest.get("rl_module_kind") == contract.STANDARD_RL_MODULE_KIND
        and manifest.get("actor_updates") == 0
        and manifest.get("critic_updates") == 0
        and manifest.get("ppo_updates") == 0
        and manifest.get("environment_samples") == 0,
    }
    if not all(checks.values()):
        failed = sorted(name for name, passed in checks.items() if not passed)
        raise MorphologyExecutionError(
            f"checkpoint-derived module evidence failed: {failed}"
        )
    return evidence


def verify_lock_payload(
    value: Any,
    *,
    observed_upstream_payload: Mapping[str, Any] | None = None,
    observed_source_closure: Mapping[str, Any] | None = None,
    observed_conditions: Mapping[str, Any] | None = None,
    observed_capabilities: Mapping[str, Any] | None = None,
    observed_checkpoint_module: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    """Recompute every frozen semantic gate and optional live closure."""

    lock = _mapping(value)
    upstream = gates.upstream_terminal_gate(lock.get("upstream_payload"))
    closure = gates.source_closure_gate(lock.get("source_closure"))
    capability = gates.q3_control_capability_gate(
        lock.get("q3_control_capabilities")
    )
    binding = _mapping(lock.get("binding"))
    conditions = _mapping(lock.get("conditions"))
    module = _mapping(lock.get("checkpoint_module"))
    module_tree = _mapping(module.get("module_export"))
    closure_inputs = _mapping(_mapping(lock.get("source_closure")).get("inputs"))
    checks = {
        "schema": set(lock) == LOCK_KEYS
        and lock.get("schema_version") == contract.SCHEMA_VERSION
        and lock.get("status") == contract.LOCK_STATUS
        and lock.get("protocol_id") == contract.PROTOCOL_ID
        and lock.get("pipeline_id") == contract.PIPELINE_ID
        and lock.get("revision") == contract.REVISION
        and lock.get("source_state") == contract.SOURCE_STATE,
        "protocol_freeze": gates.artifact_record_valid(
            lock.get("protocol_freeze"),
            expected_path=contract.PROTOCOL_FREEZE_PATH.as_posix(),
        ),
        "upstream": upstream.get("passed") is True
        and _same_json(lock.get("upstream_gate"), upstream)
        and _same_json(lock.get("upstream_endpoints"), contract.UPSTREAM_ENDPOINTS),
        "binding": gates.binding_valid(binding)
        and _same_json(binding, upstream.get("binding")),
        "closure": closure.get("passed") is True
        and _same_json(lock.get("source_closure_gate"), closure)
        and _same_json(
            _mapping(lock.get("source_closure")).get("candidate_module"),
            binding.get("candidate_module"),
        )
        and _same_json(
            _mapping(lock.get("source_closure")).get("checkpoint"),
            binding.get("checkpoint"),
        ),
        "conditions": set(conditions) == set(contract.CASE_IDS)
        and all(gates.condition_valid(conditions[name]) for name in contract.CASE_IDS),
        "fallback": capability.get("passed") is True
        and _same_json(lock.get("q3_control_capability_gate"), capability)
        and lock.get("control_source_mode") == contract.CONTROL_SOURCE_MODE,
        "checkpoint_module": set(module)
        == {
            "zero_audit",
            "checkpoint",
            "actor_digest",
            "module_export",
            "module_manifest",
        }
        and gates.artifact_record_valid(
            module.get("zero_audit"), expected_path=contract.zero.AUDIT_PATH.as_posix()
        )
        and _same_json(module.get("checkpoint"), binding.get("checkpoint"))
        and module.get("actor_digest") == binding.get("actor_digest")
        and gates.tree_record_valid(
            module_tree, expected_path=contract.MODULE_EXPORT_PATH.as_posix()
        )
        and gates.artifact_record_valid(
            module.get("module_manifest"),
            expected_path=(
                contract.MODULE_EXPORT_PATH / "actor_feature_manifest.json"
            ).as_posix(),
        ),
        "fixed_contract": lock.get("case_ids") == list(contract.CASE_IDS)
        and lock.get("pair_role_order") == list(contract.PAIR_ROLE_ORDER)
        and lock.get("expected_pair_count") == contract.EXPECTED_PAIR_COUNT
        and lock.get("expected_local_rollout_count")
        == contract.EXPECTED_LOCAL_ROLLOUT_COUNT
        and lock.get("expected_steps_per_rollout")
        == contract.EXPECTED_STEPS_PER_ROLLOUT
        and lock.get("expected_total_policy_steps")
        == contract.EXPECTED_TOTAL_POLICY_STEPS
        and lock.get("required_pair_checks") == list(contract.REQUIRED_PAIR_CHECKS),
        "configs": _same_json(
            lock.get("control_reward_config"), contract.CONTROL_REWARD_CONFIG
        )
        and _same_json(
            lock.get("positive_reward_config"), contract.POSITIVE_REWARD_CONFIG
        )
        and gates.exact_config_delta_gate(
            lock.get("control_reward_config"), lock.get("positive_reward_config")
        )
        and _same_json(lock.get("v26_runtime_config"), contract.V26_RUNTIME_CONFIG)
        and _same_json(
            lock.get("causal_runtime_config"), contract.CAUSAL_RUNTIME_CONFIG
        )
        and _same_json(lock.get("profile_attestations"), contract.PROFILE_ATTESTATIONS),
        "profile_closure": all(
            _mapping(closure_inputs.get(name)).get("path") == expected["path"]
            and _mapping(closure_inputs.get(name)).get("sha256")
            == expected["sha256"]
            for name, expected in contract.PROFILE_ATTESTATIONS.items()
        ),
        "closed": lock.get("actor_updates") == 0
        and lock.get("critic_updates") == 0
        and lock.get("ppo_updates") == 0
        and lock.get("training_authorized") is False
        and lock.get("training_command_published") is False
        and _same_json(lock.get("authority"), contract.AUTHORITY),
        "upstream_rehash": observed_upstream_payload is None
        or _same_json(observed_upstream_payload, lock.get("upstream_payload")),
        "source_rehash": observed_source_closure is None
        or _same_json(observed_source_closure, lock.get("source_closure")),
        "condition_rehash": observed_conditions is None
        or _same_json(observed_conditions, conditions),
        "capability_rehash": observed_capabilities is None
        or _same_json(observed_capabilities, lock.get("q3_control_capabilities")),
        "module_rehash": observed_checkpoint_module is None
        or _same_json(observed_checkpoint_module, module),
    }
    if not all(checks.values()):
        failed = sorted(name for name, passed in checks.items() if passed is not True)
        raise MorphologyExecutionError(f"morphology execution lock failed: {failed}")
    return lock


def verify_lock() -> dict[str, Any]:
    """Verify the published lock and re-run all live semantic closures."""

    try:
        import freeze_h0_v12r8_morphology as freezer

        freezer.verify_protocol_freeze(live=False)
    except BaseException as exc:
        raise MorphologyExecutionError(f"protocol freeze verification failed: {exc}") from exc
    lock = strict_mapping(LOCK)
    upstream = semantic_upstream_payload()
    binding = _mapping(gates.upstream_terminal_gate(upstream).get("binding"))
    return verify_lock_payload(
        lock,
        observed_upstream_payload=upstream,
        observed_source_closure=source_closure_snapshot(),
        observed_conditions=condition_snapshot(),
        observed_capabilities=inspect_q3_control_capabilities(),
        observed_checkpoint_module=checkpoint_module_snapshot(binding),
    )


def build_execution_plan(lock_payload: Mapping[str, Any]) -> dict[str, Any]:
    lock = verify_lock_payload(lock_payload)
    runs = []
    for case_index, case_id in enumerate(contract.CASE_IDS):
        for role_index, role in enumerate(contract.PAIR_ROLE_ORDER):
            runs.append(
                {
                    "case_id": case_id,
                    "role": role,
                    "local_execution_index": 2 * case_index + role_index,
                    "reward_config": contract.reward_config_for_role(role),
                    "condition": copy.deepcopy(lock["conditions"][case_id]),
                }
            )
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": "PLANNED_H0_V12R8_MORPH_PHYSICAL_AB",
        "protocol_id": contract.PROTOCOL_ID,
        "control_source_mode": contract.CONTROL_SOURCE_MODE,
        "runs": runs,
        "local_rollout_count": len(runs),
        "training_executed": False,
    }


def _arm_paths(case_id: str, role: str) -> dict[str, Path]:
    root = resolve_relative(contract.arm_root(case_id, role))
    return {
        "root": root,
        "trace": root / "trace.json",
        "reward": root / "reward_ledger.json",
        "causal": root / "causal_ledger.json",
        "summary": root / "summary.json",
        "gate": root / "gate.json",
        "receipt": root / "receipt.json",
    }


def _stream_record(rows: Sequence[Any]) -> dict[str, Any]:
    encoded = gates.canonical_json_bytes(list(rows))
    import hashlib

    return {
        "sha256": hashlib.sha256(encoded).hexdigest(),
        "size_bytes": len(encoded),
        "sample_count": len(rows),
        "encoding": contract.STREAM_ENCODING,
    }


def _streams_from_trace(rows: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    observations = [row.get("observation") for row in rows]
    actions = [row.get("action") for row in rows]
    dynamics = [row.get("dynamics") for row in rows]
    events = [row.get("events") for row in rows]
    rewards = [_mapping(row.get("reward")) for row in rows]
    return {
        "observations": _stream_record(observations),
        "actions": _stream_record(actions),
        "dynamics": _stream_record(dynamics),
        "events": _stream_record(events),
        "reward_without_morphology": _stream_record(
            [row.get("reward_without_morphology") for row in rewards]
        ),
        "morphology_loss": _stream_record(
            [row.get("morphology_loss") for row in rewards]
        ),
    }


def _arm_receipt(summary: Mapping[str, Any], gate: Mapping[str, Any]) -> dict[str, Any]:
    case_id = str(summary["case_id"])
    role = str(summary["role"])
    paths = _arm_paths(case_id, role)
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.ARM_PASS_STATUS,
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "case_id": case_id,
        "role": role,
        "summary": artifact_record(paths["summary"]),
        "gate": artifact_record(paths["gate"]),
        "gate_sha256": gates.canonical_json_sha256(gate),
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
    }


def _persist_arm(
    *,
    lock: Mapping[str, Any],
    case_id: str,
    role: str,
    local_execution_index: int,
    collected: Mapping[str, Any],
) -> dict[str, Any]:
    paths = _arm_paths(case_id, role)
    if os.path.lexists(paths["root"]):
        raise MorphologyExecutionError(f"refusing arm retry/clobber: {case_id}/{role}")
    paths["root"].mkdir(parents=True, exist_ok=False)
    trace_rows = _sequence(collected.get("trace_rows"))
    reward_rows = _sequence(collected.get("reward_rows"))
    causal_rows = _sequence(collected.get("causal_rows"))
    write_json_exclusive(paths["trace"], {"rows": trace_rows})
    write_json_exclusive(paths["reward"], {"rows": reward_rows})
    write_json_exclusive(paths["causal"], {"rows": causal_rows})
    module_tree = copy.deepcopy(lock["checkpoint_module"]["module_export"])
    summary = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.ARM_COMPLETE_STATUS,
        "protocol_id": contract.PROTOCOL_ID,
        "case_id": case_id,
        "role": role,
        "local_execution_index": local_execution_index,
        "binding": copy.deepcopy(lock["binding"]),
        "condition": copy.deepcopy(lock["conditions"][case_id]),
        "reward_config": contract.reward_config_for_role(role),
        "streams": copy.deepcopy(collected.get("streams")),
        "detector_audit": copy.deepcopy(collected.get("detector_audit")),
        "causal_audit": copy.deepcopy(collected.get("causal_audit")),
        "causal_runtime": copy.deepcopy(collected.get("causal_runtime")),
        "artifacts": {
            "trace": artifact_record(paths["trace"]),
            "reward_ledger": artifact_record(paths["reward"]),
            "causal_ledger": artifact_record(paths["causal"]),
            "noise_tape": copy.deepcopy(collected.get("noise_tape")),
            "module_export": module_tree,
        },
        "started_unix_s": collected.get("started_unix_s"),
        "completed_unix_s": collected.get("completed_unix_s"),
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "training_executed": False,
    }
    if not _same_json(summary["streams"], _streams_from_trace(trace_rows)):
        raise MorphologyExecutionError(f"collector stream digest drifted: {case_id}/{role}")
    write_json_exclusive(paths["summary"], summary)
    gate = gates.arm_gate(
        summary,
        expected_binding=lock["binding"],
        expected_condition=lock["conditions"][case_id],
        reward_rows=reward_rows,
        causal_rows=causal_rows,
    )
    write_json_exclusive(paths["gate"], gate)
    if gate.get("passed") is not True:
        raise MorphologyExecutionError(f"arm gate failed: {case_id}/{role}")
    write_json_exclusive(paths["receipt"], _arm_receipt(summary, gate))
    return verify_arm(lock=lock, case_id=case_id, role=role)


def verify_arm(
    *, lock: Mapping[str, Any], case_id: str, role: str
) -> dict[str, Any]:
    paths = _arm_paths(case_id, role)
    trace_wrapper = strict_mapping(paths["trace"])
    reward_wrapper = strict_mapping(paths["reward"])
    causal_wrapper = strict_mapping(paths["causal"])
    if set(trace_wrapper) != {"rows"} or set(reward_wrapper) != {"rows"} or set(
        causal_wrapper
    ) != {"rows"}:
        raise MorphologyExecutionError(f"arm ledger wrapper drifted: {case_id}/{role}")
    trace_rows = _sequence(trace_wrapper["rows"])
    reward_rows = _sequence(reward_wrapper["rows"])
    causal_rows = _sequence(causal_wrapper["rows"])
    summary = strict_mapping(paths["summary"])
    expected_artifacts = {
        "trace": artifact_record(paths["trace"]),
        "reward_ledger": artifact_record(paths["reward"]),
        "causal_ledger": artifact_record(paths["causal"]),
        "noise_tape": artifact_record(
            resolve_relative(contract.canonical_case(case_id)["noise_tape"])
        ),
        "module_export": tree_record(resolve_relative(contract.MODULE_EXPORT_PATH)),
    }
    trace_rewards = [_mapping(row).get("reward") for row in trace_rows]
    trace_causal = [_mapping(row).get("causal") for row in trace_rows]
    checks = {
        "artifacts": _same_json(summary.get("artifacts"), expected_artifacts),
        "streams": _same_json(summary.get("streams"), _streams_from_trace(trace_rows)),
        "reward_trace": _same_json(trace_rewards, reward_rows),
        "causal_trace": _same_json(trace_causal, causal_rows),
    }
    expected_gate = gates.arm_gate(
        summary,
        expected_binding=lock["binding"],
        expected_condition=lock["conditions"][case_id],
        reward_rows=reward_rows,
        causal_rows=causal_rows,
    )
    observed_gate = strict_mapping(paths["gate"])
    checks["gate"] = _same_json(observed_gate, expected_gate) and expected_gate.get(
        "passed"
    ) is True
    expected_receipt = _arm_receipt(summary, expected_gate)
    checks["receipt"] = _same_json(
        strict_mapping(paths["receipt"]), expected_receipt
    )
    if not all(checks.values()):
        failed = sorted(name for name, passed in checks.items() if not passed)
        raise MorphologyExecutionError(
            f"arm verification failed: {case_id}/{role}: {failed}"
        )
    return summary


def _pair_payload(
    case_id: str, control: Mapping[str, Any], positive: Mapping[str, Any]
) -> dict[str, Any]:
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.PAIR_COMPLETE_STATUS,
        "protocol_id": contract.PROTOCOL_ID,
        "case_id": case_id,
        "control_source_mode": contract.CONTROL_SOURCE_MODE,
        "control": copy.deepcopy(dict(control)),
        "positive": copy.deepcopy(dict(positive)),
    }


def _pair_receipt(case_id: str, gate: Mapping[str, Any]) -> dict[str, Any]:
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.PAIR_PASS_STATUS,
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "case_id": case_id,
        "control_receipt": artifact_record(
            _arm_paths(case_id, contract.CONTROL_ROLE)["receipt"]
        ),
        "positive_receipt": artifact_record(
            _arm_paths(case_id, contract.POSITIVE_ROLE)["receipt"]
        ),
        "pair_gate": artifact_record(resolve_relative(contract.pair_gate_path(case_id))),
        "effect": copy.deepcopy(_mapping(gate.get("effect"))),
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
    }


def _ledger_rows(case_id: str, role: str, name: str) -> list[Any]:
    wrapper = strict_mapping(_arm_paths(case_id, role)[name])
    if set(wrapper) != {"rows"}:
        raise MorphologyExecutionError(f"ledger wrapper drifted: {case_id}/{role}")
    return _sequence(wrapper["rows"])


def _compute_pair_gate(
    lock: Mapping[str, Any], case_id: str, control: Mapping[str, Any], positive: Mapping[str, Any]
) -> dict[str, Any]:
    return gates.pair_gate(
        _pair_payload(case_id, control, positive),
        expected_binding=lock["binding"],
        expected_condition=lock["conditions"][case_id],
        control_reward_rows=_ledger_rows(case_id, contract.CONTROL_ROLE, "reward"),
        positive_reward_rows=_ledger_rows(case_id, contract.POSITIVE_ROLE, "reward"),
        control_causal_rows=_ledger_rows(case_id, contract.CONTROL_ROLE, "causal"),
        positive_causal_rows=_ledger_rows(case_id, contract.POSITIVE_ROLE, "causal"),
    )


def verify_pair(lock: Mapping[str, Any], case_id: str) -> dict[str, Any]:
    control = verify_arm(lock=lock, case_id=case_id, role=contract.CONTROL_ROLE)
    positive = verify_arm(lock=lock, case_id=case_id, role=contract.POSITIVE_ROLE)
    expected = _compute_pair_gate(lock, case_id, control, positive)
    observed = strict_mapping(resolve_relative(contract.pair_gate_path(case_id)))
    receipt = strict_mapping(resolve_relative(contract.pair_receipt_path(case_id)))
    if (
        not _same_json(observed, expected)
        or expected.get("passed") is not True
        or not _same_json(receipt, _pair_receipt(case_id, expected))
    ):
        raise MorphologyExecutionError(f"pair verification failed: {case_id}")
    return expected


def build_terminal_receipt(
    *, lock_payload: Mapping[str, Any], pair_results: Sequence[Mapping[str, Any]]
) -> dict[str, Any]:
    lock = verify_lock_payload(lock_payload)
    aggregate = gates.aggregate_gate(pair_results, expected_binding=lock["binding"])
    passed = aggregate.get("passed") is True
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": (
            contract.PIPELINE_TERMINAL_PASS_STATUS
            if passed
            else contract.PIPELINE_TERMINAL_FAIL_STATUS
        ),
        "passed": passed,
        "terminal": True,
        "error": None if passed else "morphology_positive_qualification_failed",
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "binding": copy.deepcopy(lock["binding"]),
        "aggregate": aggregate,
        "pair_count": contract.EXPECTED_PAIR_COUNT,
        "local_rollout_count": contract.EXPECTED_LOCAL_ROLLOUT_COUNT,
        "policy_step_count": contract.EXPECTED_TOTAL_POLICY_STEPS,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "training_executed": False,
        "training_authorized": passed,
        "training_command_publication_authorized": passed,
        "training_command_published": False,
        "next_stage": (
            contract.NEXT_STAGE_AFTER_PASS if passed else contract.NEXT_STAGE_AFTER_FAIL
        ),
    }


def build_training_handoff(
    receipt: Mapping[str, Any], *, receipt_artifact: Mapping[str, Any]
) -> dict[str, Any]:
    if gates.terminal_receipt_gate(receipt).get("passed") is not True:
        raise MorphologyExecutionError("terminal morphology PASS required for handoff")
    if not gates.artifact_record_valid(
        receipt_artifact, expected_path=contract.FINAL_RECEIPT_PATH.as_posix()
    ):
        raise MorphologyExecutionError("terminal morphology receipt is not hash-bound")
    platforms = {}
    for platform_id in ("macos_arm64", "windows_x86_64"):
        argv = contract.final_training_argv(platform_id)
        platforms[platform_id] = {
            "argv": list(argv),
            "command": contract.render_command(argv, platform_id),
        }
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.HANDOFF_PASS_STATUS,
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "authorized_by_terminal_receipt": copy.deepcopy(dict(receipt_artifact)),
        "binding": copy.deepcopy(dict(receipt["binding"])),
        "checkpoint": copy.deepcopy(dict(receipt["binding"]["checkpoint"])),
        "initialization_mode": "resume_from_full_checkpoint_zero",
        "required_flag": "--resume-from",
        "forbidden_flags": ["--warm-start", "--warm-start-raw"],
        "target_training_iterations": contract.FINAL_TRAINING_ITERATIONS,
        "reward_config": copy.deepcopy(contract.POSITIVE_REWARD_CONFIG),
        "causal_runtime": copy.deepcopy(contract.CAUSAL_RUNTIME_CONFIG),
        "platforms": platforms,
        "training_authorized": True,
        "training_command_published": True,
        "training_execution_performed": False,
        "next_stage": contract.NEXT_STAGE_AFTER_PASS,
    }


def _claim_payload(lock: Mapping[str, Any]) -> dict[str, Any]:
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.CLAIM_STATUS,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "binding": copy.deepcopy(lock["binding"]),
        "execution_lock": artifact_record(LOCK),
        "control_source_mode": contract.CONTROL_SOURCE_MODE,
        "local_rollout_count": contract.EXPECTED_LOCAL_ROLLOUT_COUNT,
        "retry_authorized": False,
        "resume_authorized": False,
        "training_executed": False,
    }


def _terminal_ledger(
    *,
    lock: Mapping[str, Any],
    claim_path: Path,
    started_unix_s: float,
    activity: Mapping[str, Any],
    completed_pairs: Sequence[str],
    passed: bool,
    error: BaseException | None,
) -> dict[str, Any]:
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": (
            contract.PIPELINE_TERMINAL_PASS_STATUS
            if passed
            else contract.PIPELINE_TERMINAL_FAIL_STATUS
        ),
        "passed": passed,
        "terminal": True,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "binding": copy.deepcopy(lock["binding"]),
        "started_unix_s": float(started_unix_s),
        "completed_unix_s": float(time.time()),
        "error_type": type(error).__name__ if error is not None else None,
        "error": str(error) if error is not None else None,
        "execution_lock": artifact_record(LOCK),
        "pipeline_claim": artifact_record(claim_path),
        "completed_pairs": list(completed_pairs),
        "pair_receipts": {
            case_id: artifact_record(resolve_relative(contract.pair_receipt_path(case_id)))
            for case_id in completed_pairs
        },
        "final_receipt": (
            artifact_record(resolve_relative(contract.FINAL_RECEIPT_PATH))
            if passed
            else None
        ),
        "training_handoff": (
            artifact_record(resolve_relative(contract.TRAINING_HANDOFF_PATH))
            if passed
            else None
        ),
        "activity": copy.deepcopy(dict(activity)),
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "training_executed": False,
        "training_authorized": passed,
        "training_command_published": passed,
        "retry_authorized": False,
        "resume_authorized": False,
        "next_stage": (
            contract.NEXT_STAGE_AFTER_PASS if passed else contract.NEXT_STAGE_AFTER_FAIL
        ),
    }


def execute(*, collector: ArmCollector | None = None) -> dict[str, Any]:
    """Claim and execute the canonical physical A/B exactly once."""

    lock = verify_lock()
    if os.path.lexists(RUN_ROOT):
        raise MorphologyExecutionError(f"refusing retry/clobber: {RUN_ROOT}")
    selected = collector
    if selected is None:
        from h0_v12r8_morphology_physical_rollout import collect_physical_arm

        selected = collect_physical_arm
    started = time.time()
    activity = copy.deepcopy(ACTIVITY_TEMPLATE)
    completed_pairs: list[str] = []
    RUN_ROOT.mkdir(parents=True, exist_ok=False)
    claim_path = write_json_exclusive(
        resolve_relative(contract.PIPELINE_CLAIM_PATH), _claim_payload(lock)
    )
    try:
        pair_results = []
        for case_index, case_id in enumerate(contract.CASE_IDS):
            summaries = {}
            for role_index, role in enumerate(contract.PAIR_ROLE_ORDER):
                result = selected(
                    role=role,
                    case=contract.canonical_case(case_id),
                    module_export=lock["checkpoint_module"]["module_export"],
                    activity=activity,
                )
                summaries[role] = _persist_arm(
                    lock=lock,
                    case_id=case_id,
                    role=role,
                    local_execution_index=2 * case_index + role_index,
                    collected=result,
                )
            pair_gate = _compute_pair_gate(
                lock,
                case_id,
                summaries[contract.CONTROL_ROLE],
                summaries[contract.POSITIVE_ROLE],
            )
            write_json_exclusive(
                resolve_relative(contract.pair_gate_path(case_id)), pair_gate
            )
            if pair_gate.get("passed") is not True:
                raise MorphologyExecutionError(f"pair gate failed: {case_id}")
            write_json_exclusive(
                resolve_relative(contract.pair_receipt_path(case_id)),
                _pair_receipt(case_id, pair_gate),
            )
            pair_results.append(verify_pair(lock, case_id))
            completed_pairs.append(case_id)
        if activity != EXPECTED_TERMINAL_ACTIVITY:
            raise MorphologyExecutionError(f"physical activity drifted: {activity}")
        receipt = build_terminal_receipt(
            lock_payload=lock, pair_results=pair_results
        )
        if gates.terminal_receipt_gate(receipt).get("passed") is not True:
            raise MorphologyExecutionError("terminal morphology receipt gate failed")
        final_root = resolve_relative(contract.FINAL_ROOT)
        final_root.mkdir(parents=True, exist_ok=False)
        receipt_path = write_json_exclusive(
            resolve_relative(contract.FINAL_RECEIPT_PATH), receipt
        )
        handoff = build_training_handoff(
            receipt, receipt_artifact=artifact_record(receipt_path)
        )
        write_json_exclusive(
            resolve_relative(contract.TRAINING_HANDOFF_PATH), handoff
        )
        write_json_exclusive(
            resolve_relative(contract.TERMINAL_LEDGER_PATH),
            _terminal_ledger(
                lock=lock,
                claim_path=claim_path,
                started_unix_s=started,
                activity=activity,
                completed_pairs=completed_pairs,
                passed=True,
                error=None,
            ),
        )
        return verify_terminal_ledger()
    except BaseException as exc:
        ledger_path = resolve_relative(contract.TERMINAL_LEDGER_PATH)
        if not os.path.lexists(ledger_path):
            try:
                write_json_exclusive(
                    ledger_path,
                    _terminal_ledger(
                        lock=lock,
                        claim_path=claim_path,
                        started_unix_s=started,
                        activity=activity,
                        completed_pairs=completed_pairs,
                        passed=False,
                        error=exc,
                    ),
                )
            except BaseException:
                pass
        raise


def verify_terminal_ledger() -> dict[str, Any]:
    """Re-run all live closures and verify the immutable terminal PASS chain."""

    lock = verify_lock()
    claim_path = resolve_relative(contract.PIPELINE_CLAIM_PATH)
    claim = strict_mapping(claim_path)
    if not _same_json(claim, _claim_payload(lock)):
        raise MorphologyExecutionError("pipeline claim drifted")
    pair_results = [verify_pair(lock, case_id) for case_id in contract.CASE_IDS]
    expected_receipt = build_terminal_receipt(
        lock_payload=lock, pair_results=pair_results
    )
    receipt_path = resolve_relative(contract.FINAL_RECEIPT_PATH)
    receipt = strict_mapping(receipt_path)
    if (
        not _same_json(receipt, expected_receipt)
        or gates.terminal_receipt_gate(receipt).get("passed") is not True
    ):
        raise MorphologyExecutionError("terminal receipt drifted")
    expected_handoff = build_training_handoff(
        receipt, receipt_artifact=artifact_record(receipt_path)
    )
    handoff_path = resolve_relative(contract.TRAINING_HANDOFF_PATH)
    if not _same_json(strict_mapping(handoff_path), expected_handoff):
        raise MorphologyExecutionError("training handoff drifted")
    ledger = strict_mapping(resolve_relative(contract.TERMINAL_LEDGER_PATH))
    checks = {
        "identity": ledger.get("schema_version") == contract.SCHEMA_VERSION
        and ledger.get("status") == contract.PIPELINE_TERMINAL_PASS_STATUS
        and ledger.get("passed") is True
        and ledger.get("terminal") is True
        and ledger.get("protocol_id") == contract.PROTOCOL_ID
        and ledger.get("pipeline_id") == contract.PIPELINE_ID
        and ledger.get("error_type") is None
        and ledger.get("error") is None,
        "binding": _same_json(ledger.get("binding"), lock.get("binding")),
        "records": ledger.get("execution_lock") == artifact_record(LOCK)
        and ledger.get("pipeline_claim") == artifact_record(claim_path)
        and ledger.get("final_receipt") == artifact_record(receipt_path)
        and ledger.get("training_handoff") == artifact_record(handoff_path)
        and ledger.get("pair_receipts")
        == {
            case_id: artifact_record(
                resolve_relative(contract.pair_receipt_path(case_id))
            )
            for case_id in contract.CASE_IDS
        },
        "pairs_activity": ledger.get("completed_pairs") == list(contract.CASE_IDS)
        and ledger.get("activity") == EXPECTED_TERMINAL_ACTIVITY,
        "time": _finite(ledger.get("started_unix_s"))
        and _finite(ledger.get("completed_unix_s"))
        and ledger["completed_unix_s"] >= ledger["started_unix_s"],
        "closed": ledger.get("actor_updates") == 0
        and ledger.get("critic_updates") == 0
        and ledger.get("ppo_updates") == 0
        and ledger.get("training_executed") is False
        and ledger.get("training_authorized") is True
        and ledger.get("training_command_published") is True
        and ledger.get("retry_authorized") is False
        and ledger.get("resume_authorized") is False
        and ledger.get("next_stage") == contract.NEXT_STAGE_AFTER_PASS,
    }
    if not all(checks.values()):
        failed = sorted(name for name, passed in checks.items() if not passed)
        raise MorphologyExecutionError(f"terminal ledger drifted: {failed}")
    return ledger


if __name__ == "__main__":
    execute()


__all__ = [
    "MorphologyExecutionError",
    "build_execution_plan",
    "build_terminal_receipt",
    "build_training_handoff",
    "checkpoint_module_snapshot",
    "condition_snapshot",
    "execute",
    "inspect_q3_control_capabilities",
    "semantic_upstream_payload",
    "source_closure_snapshot",
    "strict_json_any",
    "verify_arm",
    "verify_lock",
    "verify_lock_payload",
    "verify_pair",
    "verify_terminal_ledger",
]
