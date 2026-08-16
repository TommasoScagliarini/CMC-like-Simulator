"""Future freezer for the candidate-deferred V12R4/Q2 zero-update port.

The module is source-only until the future R4 and Q2 artifacts exist.  It does
not run on import.  ``freeze()`` is deliberately fail-closed and may publish
only one no-clobber lock after terminal R4 and Q2 PASS records bind the exact
same actor-only standard full-mean candidate tree.
"""

from __future__ import annotations

import json
import os
import sys
from typing import Any, Mapping

try:
    from . import h0_v12r4_zero_update_contract as contract
    from . import run_h0_v12r4_zero_update_port as driver
except ImportError:  # Direct ``python validation/v12r4zero/freeze_*.py``.
    import h0_v12r4_zero_update_contract as contract
    import run_h0_v12r4_zero_update_port as driver


q2 = contract.q2


class ZeroUpdateFreezeError(RuntimeError):
    """Raised when future zero-update execution authority cannot be frozen."""


def _fail(message: str) -> None:
    raise ZeroUpdateFreezeError(message)


def _strict(name: str) -> dict[str, Any]:
    try:
        return driver.strict_json(driver.INPUT_PATHS[name])
    except driver.ZeroUpdatePortError as exc:
        raise ZeroUpdateFreezeError(str(exc)) from exc


def _same_json(left: Any, right: Any) -> bool:
    try:
        return driver._canonical_json_bytes(
            left
        ) == driver._canonical_json_bytes(  # noqa: SLF001
            right
        )
    except driver.ZeroUpdatePortError as exc:
        raise ZeroUpdateFreezeError(str(exc)) from exc


def _zero_updates(payload: Mapping[str, Any], label: str) -> None:
    if (
        payload.get("critic_updates") != 0
        or payload.get("ppo_updates") != 0
        or payload.get("protected_trials_opened", []) != []
    ):
        _fail(f"{label} update/protected-trial footer drifted")


def _candidate_binding(payload: Mapping[str, Any], label: str) -> tuple[str, Any]:
    candidate_id = payload.get("candidate_id")
    candidate_module = payload.get("candidate_module")
    if not isinstance(candidate_id, str) or not candidate_id:
        _fail(f"{label} has no candidate_id")
    if not isinstance(candidate_module, Mapping):
        _fail(f"{label} has no exact candidate_module tree")
    return candidate_id, candidate_module


def _validate_r4() -> tuple[str, dict[str, Any]]:
    freeze = _strict("r4_candidate_freeze_receipt")
    final = _strict("r4_final_development_receipt")
    ledger = _strict("r4_pipeline_ledger")
    expected = (
        (freeze, q2.R4_CANDIDATE_FREEZE_PASS_STATUS, "R4 candidate freeze"),
        (final, q2.R4_FINAL_DEVELOPMENT_PASS_STATUS, "R4 development receipt"),
        (ledger, q2.R4_PIPELINE_PASS_STATUS, "R4 terminal ledger"),
    )
    for payload, status, label in expected:
        if payload.get("status") != status or payload.get("passed") is not True:
            _fail(f"{label} is not the required PASS")
        if payload.get("candidate_selection_rule") != q2.R4_CANDIDATE_SELECTION_RULE:
            _fail(f"{label} candidate selection rule drifted")
        _zero_updates(payload, label)
    if ledger.get("terminal") is not True or ledger.get("error") is not None:
        _fail("R4 pipeline ledger is not a clean terminal PASS")
    if ledger.get("next_stage") != "WAIT_SEPARATE_Q2_PROTOCOL":
        _fail("R4 pipeline did not stop at the separate Q2 boundary")
    if ledger.get("qualification_executed") is not False:
        _fail("R4 development improperly executed qualification")
    if (
        ledger.get("runtime_promoted") is not False
        or ledger.get("checkpoint_zero_created") is not False
        or ledger.get("positive_morphology_enabled") is not False
    ):
        _fail("R4 development promoted or checkpointed the candidate")
    if freeze.get("critic_present") is not False:
        _fail("R4 candidate freeze is not actor-only")
    if (
        freeze.get("source_h0_byte_exact") is not True
        or freeze.get("logstd_byte_exact") is not True
        or freeze.get("save_reload_exact") is not True
    ):
        _fail("R4 full-mean preservation audit is incomplete")
    if any(payload.get("q2_paths_opened", []) != [] for payload in (freeze, ledger)):
        _fail("R4 opened Q2 paths before terminal development")

    bindings = [_candidate_binding(payload, label) for payload, _, label in expected]
    candidate_ids = {candidate_id for candidate_id, _ in bindings}
    if len(candidate_ids) != 1:
        _fail("R4 artifacts bind different candidate ids")
    modules = [module for _, module in bindings]
    if any(not _same_json(module, modules[0]) for module in modules[1:]):
        _fail("R4 artifacts bind different candidate trees")
    observed_tree = driver.tree_record(driver.CANDIDATE_DIR)
    if not _same_json(modules[0], observed_tree):
        _fail("R4 declared candidate tree differs from disk")
    candidate_id = next(iter(candidate_ids))
    expected_id = contract.candidate_id_for_tree(observed_tree["tree_sha256"])
    if candidate_id != expected_id:
        _fail("R4 candidate id is not derived from its exact tree hash")
    return candidate_id, observed_tree


def _validate_q2(candidate_id: str, candidate_module: Mapping[str, Any]) -> None:
    protocol = _strict("q2_protocol_freeze")
    receipt = _strict("q2_final_receipt")
    ledger = _strict("q2_pipeline_ledger")
    expected = (
        (protocol, q2.PROTOCOL_FREEZE_PASS_STATUS, "Q2 protocol freeze"),
        (receipt, q2.AGGREGATE_PASS_STATUS, "Q2 final receipt"),
        (ledger, q2.AGGREGATE_PASS_STATUS, "Q2 terminal ledger"),
    )
    for payload, status, label in expected:
        if (
            payload.get("status") != status
            or payload.get("passed") is not True
            or payload.get("protocol_id") != q2.PROTOCOL_ID
        ):
            _fail(f"{label} is not the required Q2 PASS")
        bound_id, bound_module = _candidate_binding(payload, label)
        if bound_id != candidate_id or not _same_json(bound_module, candidate_module):
            _fail(f"{label} does not bind the exact R4 candidate")
        _zero_updates(payload, label)
    if protocol.get("candidate_binding_state") != "BOUND":
        _fail("Q2 protocol freeze did not complete deferred candidate binding")
    if protocol.get("candidate_selection_rule") != q2.R4_CANDIDATE_SELECTION_RULE:
        _fail("Q2 protocol freeze candidate rule drifted")
    if ledger.get("terminal") is not True or ledger.get("error") is not None:
        _fail("Q2 ledger is not a clean terminal PASS")
    if ledger.get("actor_updates") != 0:
        _fail("Q2 qualification performed an actor update")
    if (
        ledger.get("runtime_promoted") is not False
        or ledger.get("checkpoint_zero_created") is not False
        or ledger.get("positive_morphology_enabled") is not False
    ):
        _fail("Q2 improperly promoted/checkpointed/enabled morphology")
    next_stage = receipt.get("next_stage", ledger.get("next_stage"))
    if next_stage != q2.NEXT_STAGE_AFTER_Q2_PASS:
        _fail("Q2 terminal PASS did not open only the separate zero port")


def validate_prerequisites() -> tuple[str, dict[str, Any]]:
    """Read and bind the exact future R4/Q2 candidate without publishing."""

    if contract.CANDIDATE_ID is not None or contract.CANDIDATE_MODULE is not None:
        _fail("source contract candidate must remain deferred")
    candidate_id, candidate_module = _validate_r4()
    _validate_q2(candidate_id, candidate_module)
    try:
        driver.validate_v26_target_config()
    except driver.ZeroUpdatePortError as exc:
        raise ZeroUpdateFreezeError(str(exc)) from exc
    return candidate_id, candidate_module


def build_lock() -> dict[str, Any]:
    """Build a bound payload in memory and perform a final TOCTOU rehash."""

    try:
        closure_before = driver.closure_snapshot()
    except driver.ZeroUpdatePortError as exc:
        raise ZeroUpdateFreezeError(str(exc)) from exc
    candidate_id, candidate_module = validate_prerequisites()
    if not _same_json(candidate_module, closure_before["candidate_module"]):
        _fail("candidate changed between closure snapshot and prerequisite checks")
    try:
        driver.verify_closure(closure_before)
    except driver.ZeroUpdatePortError as exc:
        raise ZeroUpdateFreezeError(str(exc)) from exc
    payload = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.LOCK_STATUS,
        "protocol_id": contract.PROTOCOL_ID,
        "qualification_protocol_id": contract.QUALIFICATION_PROTOCOL_ID,
        "revision": contract.REVISION,
        "candidate_binding_state": "BOUND_AFTER_Q2_TERMINAL_PASS",
        "candidate_id": candidate_id,
        "candidate_module": candidate_module,
        "candidate_selection_rule": contract.CANDIDATE_SELECTION_RULE,
        "source_topology_id": contract.SOURCE_TOPOLOGY_ID,
        "target_contract_id": contract.TARGET_CONTRACT_ID,
        "target_fixed_config": dict(contract.TARGET_FIXED_CONFIG),
        "target_reward_config": dict(contract.TARGET_REWARD_CONFIG),
        "output_root": driver.repo_relative(driver.OUTPUT_ROOT),
        "authority": dict(contract.AUTHORITY),
        "required_checks": list(contract.REQUIRED_CHECKS),
        "sources": closure_before["sources"],
        "inputs": closure_before["inputs"],
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "environment_samples": 0,
        "protected_trials_opened": [],
    }
    if set(payload) != driver.LOCK_KEYS:
        _fail("freezer and runner lock schemas disagree")
    return payload


def freeze() -> dict[str, Any]:
    """Future no-clobber publication entrypoint; not invoked in this task."""

    if os.path.lexists(driver.LOCK):
        _fail(f"refusing to clobber execution lock: {driver.LOCK}")
    if os.path.lexists(driver.OUTPUT_ROOT):
        _fail(f"zero-update output already exists: {driver.OUTPUT_ROOT}")
    try:
        driver.reject_link_or_reparse_ancestors(driver.LOCK, include_leaf=False)
        driver.reject_link_or_reparse_ancestors(driver.OUTPUT_ROOT, include_leaf=False)
    except driver.ZeroUpdatePortError as exc:
        raise ZeroUpdateFreezeError(str(exc)) from exc
    payload = build_lock()
    closure = {
        "sources": payload["sources"],
        "inputs": payload["inputs"],
        "candidate_module": payload["candidate_module"],
    }
    try:
        driver.verify_closure(closure)
    except driver.ZeroUpdatePortError as exc:
        raise ZeroUpdateFreezeError(str(exc)) from exc
    if os.path.lexists(driver.LOCK) or os.path.lexists(driver.OUTPUT_ROOT):
        _fail("lock/output appeared during freezer TOCTOU window")
    try:
        driver.write_json_exclusive(driver.LOCK, payload)
        driver.verify_lock()
    except driver.ZeroUpdatePortError as exc:
        raise ZeroUpdateFreezeError(str(exc)) from exc
    if os.path.lexists(driver.OUTPUT_ROOT):
        _fail("freezer unexpectedly created the execution output root")
    return payload


if __name__ == "__main__":
    try:
        print(json.dumps(freeze(), indent=2, sort_keys=True, allow_nan=False))
    except Exception as exc:
        print(f"{type(exc).__name__}: {exc}", file=sys.stderr, flush=True)
        raise SystemExit(2) from exc
