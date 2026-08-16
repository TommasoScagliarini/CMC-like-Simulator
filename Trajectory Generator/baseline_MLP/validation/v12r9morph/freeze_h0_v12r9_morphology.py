"""Live exclusive freezer for the V12R9 morphology physical A/B.

Import is inert.  ``freeze`` first invokes all native terminal verifiers and
rehashes every source/input, Q3 condition, inspected trace and checkpoint-
derived RLModule.  Only then can it publish the protocol and execution lock by
exclusive no-clobber writes.  No environment or training call occurs here.
"""

from __future__ import annotations

import copy
import hashlib
import json
import os
from collections.abc import Mapping
from pathlib import Path, PurePath
from typing import Any

try:
    from . import h0_v12r9_morphology_contract as contract
    from . import h0_v12r9_morphology_gates as gates
    from . import run_h0_v12r9_morphology as runner
except ImportError:
    import h0_v12r9_morphology_contract as contract
    import h0_v12r9_morphology_gates as gates
    import run_h0_v12r9_morphology as runner


class MorphologyFreezeError(RuntimeError):
    """Raised before any unsafe or ambiguous freeze publication."""


PROTOCOL_KEYS = frozenset(
    {
        "schema_version",
        "status",
        "protocol_id",
        "pipeline_id",
        "revision",
        "source_state",
        "required_working_directory",
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
        "training_validation_endpoint",
        "required_pair_checks",
        "actor_updates",
        "critic_updates",
        "ppo_updates",
        "training_authorized",
        "training_command_published",
        "authority",
    }
)


def _mapping(value: Any) -> dict[str, Any]:
    return dict(value) if isinstance(value, Mapping) else {}


def _same_json(left: Any, right: Any) -> bool:
    try:
        return gates.canonical_json_bytes(left) == gates.canonical_json_bytes(right)
    except (TypeError, ValueError):
        return False


def _prospective_record(path: Path, payload: Mapping[str, Any]) -> dict[str, Any]:
    encoded = (
        json.dumps(
            dict(payload),
            indent=2,
            sort_keys=True,
            ensure_ascii=False,
            allow_nan=False,
        ).encode("utf-8")
        + b"\n"
    )
    return {
        "path": runner.zero_runner.repo_relative(path),
        "sha256": hashlib.sha256(encoded).hexdigest(),
        "size_bytes": len(encoded),
    }


def build_protocol_freeze_payload(
    *,
    upstream_payload: Mapping[str, Any],
    source_closure: Mapping[str, Any],
    conditions: Mapping[str, Any],
    q3_capabilities: Mapping[str, Any],
    checkpoint_module: Mapping[str, Any],
) -> dict[str, Any]:
    upstream_gate = gates.upstream_terminal_gate(upstream_payload)
    source_gate = gates.source_closure_gate(source_closure)
    capability_gate = gates.q3_control_capability_gate(q3_capabilities)
    if not (
        upstream_gate.get("passed") is True
        and source_gate.get("passed") is True
        and capability_gate.get("passed") is True
    ):
        raise MorphologyFreezeError("live prerequisite gate failed")
    binding = _mapping(upstream_gate.get("binding"))
    payload = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.FREEZE_STATUS,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "revision": contract.REVISION,
        "source_state": contract.SOURCE_STATE,
        "required_working_directory": contract.REQUIRED_WORKING_DIRECTORY,
        "upstream_endpoints": copy.deepcopy(contract.UPSTREAM_ENDPOINTS),
        "upstream_payload": copy.deepcopy(dict(upstream_payload)),
        "upstream_gate": upstream_gate,
        "binding": copy.deepcopy(binding),
        "source_closure": copy.deepcopy(dict(source_closure)),
        "source_closure_gate": source_gate,
        "conditions": copy.deepcopy(dict(conditions)),
        "q3_control_capabilities": copy.deepcopy(dict(q3_capabilities)),
        "q3_control_capability_gate": capability_gate,
        "control_source_mode": contract.CONTROL_SOURCE_MODE,
        "checkpoint_module": copy.deepcopy(dict(checkpoint_module)),
        "case_ids": list(contract.CASE_IDS),
        "pair_role_order": list(contract.PAIR_ROLE_ORDER),
        "expected_pair_count": contract.EXPECTED_PAIR_COUNT,
        "expected_local_rollout_count": contract.EXPECTED_LOCAL_ROLLOUT_COUNT,
        "expected_steps_per_rollout": contract.EXPECTED_STEPS_PER_ROLLOUT,
        "expected_total_policy_steps": contract.EXPECTED_TOTAL_POLICY_STEPS,
        "control_reward_config": copy.deepcopy(contract.CONTROL_REWARD_CONFIG),
        "positive_reward_config": copy.deepcopy(contract.POSITIVE_REWARD_CONFIG),
        "v26_runtime_config": copy.deepcopy(contract.V26_RUNTIME_CONFIG),
        "causal_runtime_config": copy.deepcopy(contract.CAUSAL_RUNTIME_CONFIG),
        "profile_attestations": copy.deepcopy(contract.PROFILE_ATTESTATIONS),
        "training_validation_endpoint": copy.deepcopy(
            contract.TRAINING_VALIDATION_ENDPOINT
        ),
        "required_pair_checks": list(contract.REQUIRED_PAIR_CHECKS),
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "training_authorized": False,
        "training_command_published": False,
        "authority": copy.deepcopy(contract.AUTHORITY),
    }
    return verify_protocol_freeze_payload(payload)


def build_lock_payload(
    protocol_payload: Mapping[str, Any], *, protocol_artifact: Mapping[str, Any]
) -> dict[str, Any]:
    protocol = verify_protocol_freeze_payload(protocol_payload)
    if not gates.artifact_record_valid(
        protocol_artifact, expected_path=contract.PROTOCOL_FREEZE_PATH.as_posix()
    ):
        raise MorphologyFreezeError("protocol artifact record is invalid")
    lock = copy.deepcopy(protocol)
    lock["status"] = contract.LOCK_STATUS
    lock["protocol_freeze"] = copy.deepcopy(dict(protocol_artifact))
    return runner.verify_lock_payload(lock)


def verify_protocol_freeze_payload(
    value: Any,
    *,
    observed_upstream_payload: Mapping[str, Any] | None = None,
    observed_source_closure: Mapping[str, Any] | None = None,
    observed_conditions: Mapping[str, Any] | None = None,
    observed_capabilities: Mapping[str, Any] | None = None,
    observed_checkpoint_module: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    data = _mapping(value)
    upstream = gates.upstream_terminal_gate(data.get("upstream_payload"))
    source = gates.source_closure_gate(data.get("source_closure"))
    capability = gates.q3_control_capability_gate(data.get("q3_control_capabilities"))
    binding = _mapping(data.get("binding"))
    conditions = _mapping(data.get("conditions"))
    module = _mapping(data.get("checkpoint_module"))
    closure_inputs = _mapping(_mapping(data.get("source_closure")).get("inputs"))
    checks = {
        "schema": set(data) == PROTOCOL_KEYS
        and data.get("schema_version") == contract.SCHEMA_VERSION
        and data.get("status") == contract.FREEZE_STATUS
        and data.get("protocol_id") == contract.PROTOCOL_ID
        and data.get("pipeline_id") == contract.PIPELINE_ID
        and data.get("revision") == contract.REVISION
        and data.get("source_state") == contract.SOURCE_STATE
        and data.get("required_working_directory")
        == contract.REQUIRED_WORKING_DIRECTORY,
        "upstream": upstream.get("passed") is True
        and _same_json(data.get("upstream_gate"), upstream)
        and _same_json(data.get("upstream_endpoints"), contract.UPSTREAM_ENDPOINTS)
        and _same_json(binding, upstream.get("binding")),
        "closure": source.get("passed") is True
        and _same_json(data.get("source_closure_gate"), source)
        and _same_json(
            _mapping(data.get("source_closure")).get("candidate_module"),
            binding.get("candidate_module"),
        )
        and _same_json(
            _mapping(data.get("source_closure")).get("checkpoint"),
            binding.get("checkpoint"),
        ),
        "conditions": set(conditions) == set(contract.CASE_IDS)
        and all(gates.condition_valid(conditions[name]) for name in contract.CASE_IDS),
        "fallback": capability.get("passed") is True
        and _same_json(data.get("q3_control_capability_gate"), capability)
        and data.get("control_source_mode") == contract.CONTROL_SOURCE_MODE,
        "checkpoint_module": _same_json(
            module.get("checkpoint"), binding.get("checkpoint")
        )
        and module.get("actor_digest") == binding.get("actor_digest")
        and gates.tree_record_valid(
            module.get("module_export"),
            expected_path=contract.MODULE_EXPORT_PATH.as_posix(),
        ),
        "fixed": data.get("case_ids") == list(contract.CASE_IDS)
        and data.get("pair_role_order") == list(contract.PAIR_ROLE_ORDER)
        and data.get("expected_pair_count") == contract.EXPECTED_PAIR_COUNT
        and data.get("expected_local_rollout_count")
        == contract.EXPECTED_LOCAL_ROLLOUT_COUNT
        and data.get("expected_steps_per_rollout")
        == contract.EXPECTED_STEPS_PER_ROLLOUT
        and data.get("expected_total_policy_steps")
        == contract.EXPECTED_TOTAL_POLICY_STEPS
        and data.get("required_pair_checks") == list(contract.REQUIRED_PAIR_CHECKS),
        "configs": gates.exact_config_delta_gate(
            data.get("control_reward_config"), data.get("positive_reward_config")
        )
        and _same_json(data.get("v26_runtime_config"), contract.V26_RUNTIME_CONFIG)
        and _same_json(
            data.get("causal_runtime_config"), contract.CAUSAL_RUNTIME_CONFIG
        )
        and _same_json(data.get("profile_attestations"), contract.PROFILE_ATTESTATIONS),
        "training_validation": _same_json(
            data.get("training_validation_endpoint"),
            contract.TRAINING_VALIDATION_ENDPOINT,
        ),
        "profile_closure": all(
            _mapping(closure_inputs.get(name)).get("path") == expected["path"]
            and _mapping(closure_inputs.get(name)).get("sha256") == expected["sha256"]
            for name, expected in contract.PROFILE_ATTESTATIONS.items()
        ),
        "closed": data.get("actor_updates") == 0
        and data.get("critic_updates") == 0
        and data.get("ppo_updates") == 0
        and data.get("training_authorized") is False
        and data.get("training_command_published") is False
        and _same_json(data.get("authority"), contract.AUTHORITY),
        "upstream_rehash": observed_upstream_payload is None
        or _same_json(observed_upstream_payload, data.get("upstream_payload")),
        "source_rehash": observed_source_closure is None
        or _same_json(observed_source_closure, data.get("source_closure")),
        "conditions_rehash": observed_conditions is None
        or _same_json(observed_conditions, conditions),
        "capability_rehash": observed_capabilities is None
        or _same_json(observed_capabilities, data.get("q3_control_capabilities")),
        "module_rehash": observed_checkpoint_module is None
        or _same_json(observed_checkpoint_module, module),
    }
    if not all(checks.values()):
        failed = sorted(name for name, passed in checks.items() if not passed)
        raise MorphologyFreezeError(f"protocol freeze failed: {failed}")
    return data


def _live_observations() -> dict[str, Any]:
    upstream = runner.semantic_upstream_payload()
    binding = _mapping(gates.upstream_terminal_gate(upstream).get("binding"))
    return {
        "upstream_payload": upstream,
        "source_closure": runner.source_closure_snapshot(),
        "conditions": runner.condition_snapshot(),
        "q3_capabilities": runner.inspect_q3_control_capabilities(),
        "checkpoint_module": runner.checkpoint_module_snapshot(binding),
    }


def verify_protocol_freeze(*, live: bool = True) -> dict[str, Any]:
    payload = runner.strict_mapping(resolve_relative(contract.PROTOCOL_FREEZE_PATH))
    if not live:
        return verify_protocol_freeze_payload(payload)
    observed = _live_observations()
    return verify_protocol_freeze_payload(
        payload,
        observed_upstream_payload=observed["upstream_payload"],
        observed_source_closure=observed["source_closure"],
        observed_conditions=observed["conditions"],
        observed_capabilities=observed["q3_capabilities"],
        observed_checkpoint_module=observed["checkpoint_module"],
    )


def resolve_relative(value: str | PurePath) -> Path:
    return runner.resolve_relative(value)


def verify_execution_lock() -> dict[str, Any]:
    return runner.verify_lock()


def freeze() -> dict[str, Any]:
    """Publish the protocol and execution lock, but execute no rollout."""

    runner.require_repository_root_cwd()
    if contract.contract_self_check().get("passed") is not True:
        raise MorphologyFreezeError("contract self-check failed")
    targets = (
        resolve_relative(contract.PROTOCOL_FREEZE_PATH),
        resolve_relative(contract.EXECUTION_LOCK_PATH),
        resolve_relative(contract.RUN_ROOT),
    )
    existing = [os.fspath(path) for path in targets if os.path.lexists(path)]
    if existing:
        raise MorphologyFreezeError(f"refusing freeze retry/clobber: {existing}")
    observed = _live_observations()
    protocol = build_protocol_freeze_payload(**observed)
    protocol_path = resolve_relative(contract.PROTOCOL_FREEZE_PATH)
    protocol_record = _prospective_record(protocol_path, protocol)
    lock = build_lock_payload(protocol, protocol_artifact=protocol_record)
    runner.verify_lock_payload(
        lock,
        observed_upstream_payload=observed["upstream_payload"],
        observed_source_closure=observed["source_closure"],
        observed_conditions=observed["conditions"],
        observed_capabilities=observed["q3_capabilities"],
        observed_checkpoint_module=observed["checkpoint_module"],
    )
    runner.write_json_exclusive(protocol_path, protocol)
    if runner.artifact_record(protocol_path) != protocol_record:
        raise MorphologyFreezeError("published protocol bytes drifted")
    runner.write_json_exclusive(resolve_relative(contract.EXECUTION_LOCK_PATH), lock)
    return verify_execution_lock()


if __name__ == "__main__":
    freeze()


__all__ = [
    "MorphologyFreezeError",
    "build_lock_payload",
    "build_protocol_freeze_payload",
    "freeze",
    "resolve_relative",
    "verify_execution_lock",
    "verify_protocol_freeze",
    "verify_protocol_freeze_payload",
]
