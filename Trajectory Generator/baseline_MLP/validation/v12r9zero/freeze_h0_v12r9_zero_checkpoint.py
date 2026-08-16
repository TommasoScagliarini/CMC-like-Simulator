"""No-clobber freezer for the latent V12R9/Q3 checkpoint-zero runtime.

The module is inert on import.  ``freeze`` invokes both stable upstream
semantic verifiers and cannot publish a lock before exact terminal PASS for
the same five-file candidate.  Source/input/candidate closure is rehashed
around lock construction to close the pre-build TOCTOU window.
"""

from __future__ import annotations

import copy
import os
from collections.abc import Mapping
from typing import Any

try:
    from . import h0_v12r9_zero_checkpoint_contract as contract
    from . import h0_v12r9_zero_checkpoint_gates as gates
    from . import run_h0_v12r9_zero_checkpoint as runner
except ImportError:  # Direct execution with this directory on ``sys.path``.
    import h0_v12r9_zero_checkpoint_contract as contract
    import h0_v12r9_zero_checkpoint_gates as gates
    import run_h0_v12r9_zero_checkpoint as runner


class ZeroCheckpointFreezeDeferredError(RuntimeError):
    """Raised before I/O while R9/Q3 terminal PASS evidence is unavailable."""


def _fail(message: str) -> None:
    raise ZeroCheckpointFreezeDeferredError(message)


def build_lock_payload(
    *,
    upstream_payload: Mapping[str, Any],
    actor_manifest: Mapping[str, Any],
    source_closure: Mapping[str, Any],
) -> dict[str, Any]:
    """Build a future lock in memory from semantic verifier return values."""

    upstream = gates.upstream_terminal_gate(upstream_payload)
    if upstream.get("passed") is not True:
        _fail("R9 and Q3 terminal semantic prerequisite gate failed")
    candidate_module = upstream["candidate_module"]
    manifest = gates.actor_manifest_gate(
        actor_manifest,
        candidate_module=candidate_module,
    )
    if manifest.get("passed") is not True:
        _fail("qualified actor manifest is not the exact standard W512 ABI")
    closure = gates.source_closure_gate(source_closure)
    if closure.get("passed") is not True:
        _fail("source/input/candidate closure is incomplete")
    if source_closure.get("candidate_module") != candidate_module:
        _fail("source closure candidate differs from terminal R9/Q3 candidate")

    binding = {
        "candidate_id": upstream["candidate_id"],
        "candidate_module": copy.deepcopy(candidate_module),
        "actor_digest": actor_manifest["actor_digest"],
    }
    if not gates.candidate_binding_valid(binding):
        _fail("candidate binding is invalid")
    payload = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.LOCK_STATUS,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "revision": contract.REVISION,
        "source_state": contract.SOURCE_STATE,
        "candidate_binding_state": "BOUND_AFTER_R9_AND_Q3_TERMINAL_PASS",
        "binding": binding,
        "candidate_selection_rule": contract.CANDIDATE_SELECTION_RULE,
        "source_topology_id": contract.SOURCE_TOPOLOGY_ID,
        "upstream_endpoints": copy.deepcopy(contract.UPSTREAM_ENDPOINTS),
        "upstream_payload": copy.deepcopy(dict(upstream_payload)),
        "upstream_gate": upstream,
        "actor_manifest": copy.deepcopy(dict(actor_manifest)),
        "actor_manifest_gate": manifest,
        "source_closure": copy.deepcopy(dict(source_closure)),
        "source_closure_gate": closure,
        "target_fixed_config": copy.deepcopy(contract.TARGET_FIXED_CONFIG),
        "zero_reward_config": copy.deepcopy(contract.ZERO_REWARD_CONFIG),
        "positive_restore_reward_config": copy.deepcopy(
            contract.POSITIVE_RESTORE_REWARD_CONFIG
        ),
        "profile_attestations": copy.deepcopy(contract.PROFILE_ATTESTATIONS),
        "required_audit_checks": list(contract.REQUIRED_AUDIT_CHECKS),
        "actor_transplants": 1,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "environment_samples": 0,
        "training_authorized": False,
        "training_command_published": False,
        "authority": copy.deepcopy(contract.AUTHORITY),
    }
    return payload


def validate_prerequisites() -> dict[str, Any]:
    """Return the live semantic R9/Q3 prerequisite payload without publication."""

    try:
        payload = runner.semantic_upstream_payload()
    except runner.ZeroCheckpointDeferredError as exc:
        _fail(str(exc))
    gate = gates.upstream_terminal_gate(payload)
    if gate.get("passed") is not True:
        _fail("R9/Q3 semantic terminal gate failed")
    return payload


def build_lock() -> dict[str, Any]:
    """Build the live lock in memory and prove source closure stayed stable."""

    upstream = validate_prerequisites()
    try:
        closure_before = runner.closure_snapshot()
        upstream_gate = gates.upstream_terminal_gate(upstream)
        candidate_module = upstream_gate["candidate_module"]
        manifest = runner.candidate_actor_manifest(candidate_module)
        payload = build_lock_payload(
            upstream_payload=upstream,
            actor_manifest=manifest,
            source_closure=closure_before,
        )
        runner.verify_closure(closure_before)
        runner.verify_lock_payload(
            payload,
            observed_source_closure=runner.closure_snapshot(),
        )
    except runner.ZeroCheckpointDeferredError as exc:
        _fail(str(exc))
    return payload


def freeze() -> dict[str, Any]:
    """Publish the one-shot lock only after exact terminal semantic PASS."""

    runner.require_repository_root_cwd()
    if os.path.lexists(runner.LOCK):
        _fail(f"refusing to clobber checkpoint-zero lock: {runner.LOCK}")
    if os.path.lexists(runner.OUTPUT_ROOT):
        _fail(f"checkpoint-zero output is already claimed: {runner.OUTPUT_ROOT}")
    payload = build_lock()
    try:
        runner.write_json_exclusive(runner.LOCK, payload)
        return runner.verify_lock()
    except runner.ZeroCheckpointDeferredError as exc:
        _fail(str(exc))


def verify_execution_lock() -> dict[str, Any]:
    """Verify the published lock against live upstreams and closure."""

    try:
        return runner.verify_lock()
    except runner.ZeroCheckpointDeferredError as exc:
        _fail(str(exc))


if __name__ == "__main__":
    try:
        freeze()
    except ZeroCheckpointFreezeDeferredError as exc:
        raise SystemExit(str(exc)) from exc


__all__ = [
    "ZeroCheckpointFreezeDeferredError",
    "build_lock",
    "build_lock_payload",
    "freeze",
    "validate_prerequisites",
    "verify_execution_lock",
]
