"""Historical no-clobber freezer scaffold for the V12R7/Q3 zero port.

The freezer is deliberately inert on import.  V12R7 is terminal FAIL, so
``freeze()`` is permanently blocked and can publish no execution lock.
Internal construction helpers remain covered as a source template for a
separately rebound V12R8 namespace; mere file presence or ``passed`` booleans
are insufficient there.
"""

from __future__ import annotations

import json
import os
import sys
from typing import Any

try:
    from . import h0_v12r7_zero_checkpoint_contract as contract
    from . import run_h0_v12r7_zero_checkpoint as driver
except ImportError:  # Direct ``python validation/v12r7zero/freeze_*.py``.
    import h0_v12r7_zero_checkpoint_contract as contract
    import run_h0_v12r7_zero_checkpoint as driver


class ZeroCheckpointFreezeError(RuntimeError):
    """Raised when future checkpoint-zero authority cannot be frozen."""


def _fail(message: str) -> None:
    raise ZeroCheckpointFreezeError(message)


def _same_json(left: Any, right: Any) -> bool:
    try:
        return driver._canonical_json_bytes(  # noqa: SLF001
            left
        ) == driver._canonical_json_bytes(  # noqa: SLF001
            right
        )
    except driver.ZeroCheckpointError as exc:
        raise ZeroCheckpointFreezeError(str(exc)) from exc


def validate_prerequisites() -> dict[str, Any]:
    """Return the exact terminal R7+Q3 binding without publication."""

    try:
        return driver.validate_prerequisites()
    except driver.ZeroCheckpointError as exc:
        raise ZeroCheckpointFreezeError(str(exc)) from exc


def build_lock() -> dict[str, Any]:
    """Build the future execution lock in memory and close the TOCTOU window."""

    try:
        closure_before = driver.closure_snapshot()
    except driver.ZeroCheckpointError as exc:
        raise ZeroCheckpointFreezeError(str(exc)) from exc
    prerequisites = validate_prerequisites()
    if not _same_json(
        prerequisites.get("candidate_module"), closure_before["candidate_module"]
    ):
        _fail("candidate changed between closure snapshot and semantic checks")
    try:
        driver.verify_closure(closure_before)
    except driver.ZeroCheckpointError as exc:
        raise ZeroCheckpointFreezeError(str(exc)) from exc
    payload = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.LOCK_STATUS,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "qualification_protocol_id": contract.QUALIFICATION_PROTOCOL_ID,
        "revision": contract.REVISION,
        "candidate_binding_state": "BOUND_AFTER_R7_AND_Q3_TERMINAL_PASS",
        "candidate_id": prerequisites["candidate_id"],
        "candidate_module": prerequisites["candidate_module"],
        "candidate_selection_rule": contract.CANDIDATE_SELECTION_RULE,
        "source_topology_id": contract.SOURCE_TOPOLOGY_ID,
        "target_contract_id": contract.TARGET_CONTRACT_ID,
        "target_fixed_config": dict(contract.TARGET_FIXED_CONFIG),
        "target_reward_config": dict(contract.TARGET_REWARD_CONFIG),
        "output_root": driver.repo_relative(driver.OUTPUT_ROOT),
        "authority": dict(contract.AUTHORITY),
        "required_checks": list(contract.REQUIRED_CHECKS),
        "prerequisite_audit": prerequisites,
        "sources": closure_before["sources"],
        "inputs": closure_before["inputs"],
        "actor_transplants": 1,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "environment_samples": 0,
    }
    if set(payload) != driver.LOCK_KEYS:
        _fail("freezer and runner lock schemas disagree")
    return payload


def freeze() -> dict[str, Any]:
    """Reject publication from the terminally failed historical R7 lineage."""

    if contract.CANONICAL_ENTRYPOINTS_BLOCKED:
        _fail(
            f"{contract.RETIREMENT_REASON}; create and bind "
            f"{contract.SUCCESSOR_ZERO_NAMESPACE}"
        )

    if os.path.lexists(driver.LOCK):
        _fail(f"refusing to clobber execution lock: {driver.LOCK}")
    if os.path.lexists(driver.OUTPUT_ROOT):
        _fail(f"checkpoint-zero output already exists: {driver.OUTPUT_ROOT}")
    try:
        driver.reject_link_or_reparse_ancestors(driver.LOCK, include_leaf=False)
        driver.reject_link_or_reparse_ancestors(driver.OUTPUT_ROOT, include_leaf=False)
    except driver.ZeroCheckpointError as exc:
        raise ZeroCheckpointFreezeError(str(exc)) from exc
    payload = build_lock()
    closure = {
        "sources": payload["sources"],
        "inputs": payload["inputs"],
        "candidate_module": payload["candidate_module"],
    }
    try:
        driver.verify_closure(closure)
    except driver.ZeroCheckpointError as exc:
        raise ZeroCheckpointFreezeError(str(exc)) from exc
    if os.path.lexists(driver.LOCK) or os.path.lexists(driver.OUTPUT_ROOT):
        _fail("lock/output appeared during freezer TOCTOU window")
    try:
        driver.write_json_exclusive(driver.LOCK, payload)
        driver.verify_lock()
    except driver.ZeroCheckpointError as exc:
        raise ZeroCheckpointFreezeError(str(exc)) from exc
    if os.path.lexists(driver.OUTPUT_ROOT):
        _fail("freezer unexpectedly created the output root")
    return payload


if __name__ == "__main__":
    try:
        print(json.dumps(freeze(), indent=2, sort_keys=True, allow_nan=False))
    except Exception as exc:
        print(f"{type(exc).__name__}: {exc}", file=sys.stderr, flush=True)
        raise SystemExit(2) from exc
