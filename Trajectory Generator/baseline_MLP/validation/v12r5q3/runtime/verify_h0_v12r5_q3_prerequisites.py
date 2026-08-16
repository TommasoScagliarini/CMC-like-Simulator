"""Bind Q3 only through the five official live V12R5 verifiers."""

from __future__ import annotations

import copy
import os
import sys
from collections.abc import Callable, Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import h0_v12r5_q3_artifacts as artifacts
import h0_v12r5_q3_runtime_contract as contract


R5_ROOT = (
    artifacts.REPO_ROOT
    / "Trajectory Generator"
    / "baseline_MLP"
    / "validation"
    / "v12r5"
)
if str(R5_ROOT) not in sys.path:
    sys.path.insert(0, str(R5_ROOT))

import freeze_h0_v12r5_case_balanced as r5_freezer  # noqa: E402
import run_h0_v12r5_case_balanced as r5_runner  # noqa: E402


class V12R5Q3PrerequisiteError(RuntimeError):
    """Raised unless all five official R5 verifiers agree on one candidate."""


@dataclass(frozen=True)
class R5VerifierBundle:
    protocol_freeze: Callable[[], Mapping[str, Any]]
    execution_lock: Callable[[], Mapping[str, Any]]
    candidate_freeze_receipt: Callable[[], Mapping[str, Any]]
    final_development_receipt: Callable[[], Mapping[str, Any]]
    terminal_pass_ledger: Callable[[], Mapping[str, Any]]


def official_verifiers() -> R5VerifierBundle:
    """Return the exact five verifier entry points preregistered by Q3."""

    return R5VerifierBundle(
        protocol_freeze=r5_freezer.verify_protocol_freeze,
        execution_lock=lambda: r5_runner.verify_execution_lock(
            require_run_root_absent=False
        ),
        candidate_freeze_receipt=lambda: r5_runner._verify_stage_receipt_claim_bindings(
            "freeze_case_balanced_candidate"
        ),
        final_development_receipt=lambda: r5_runner._verify_stage_receipt_claim_bindings(
            "finalize_development"
        ),
        terminal_pass_ledger=r5_runner.verify_terminal_ledger,
    )


def _selection_exact(value: Any) -> bool:
    selection = dict(value) if isinstance(value, Mapping) else {}
    return (
        selection.get("rule") == contract.R5_CANDIDATE_SELECTION_RULE
        and selection.get("module_path") == contract.R5_CANDIDATE_MODULE_PATH.as_posix()
        and selection.get("candidate_id") == "DEFERRED_UNTIL_CASE_BALANCED_FIT"
        and selection.get("candidate_tree_sha256") == "DEFERRED_UNTIL_CASE_BALANCED_FIT"
    )


def validate_verified_payloads(
    payloads: Mapping[str, Mapping[str, Any]],
    records: Mapping[str, Mapping[str, Any]],
    *,
    enforce_canonical_record_paths: bool = True,
) -> dict[str, Any]:
    """Cross-bind already verified payloads as a pure total function."""

    requirements = contract.prerequisite_requirements()
    names = [row["name"] for row in requirements]
    checks: dict[str, bool] = {
        "exact_five_official_verifier_results": list(payloads) == names
        and list(records) == names,
    }
    if not checks["exact_five_official_verifier_results"]:
        return {
            "passed": False,
            "checks": checks,
            "candidate_id": None,
            "candidate_module": None,
            "prerequisites": [],
        }

    protocol, lock, candidate, final, ledger = (payloads[name] for name in names)
    candidate_id = candidate.get("candidate_id")
    candidate_module = candidate.get("candidate_module")
    try:
        binding = contract.validate_candidate_binding(candidate_id, candidate_module)
    except (TypeError, ValueError):
        binding = None

    rows: list[dict[str, Any]] = []
    for index, (name, requirement) in enumerate(zip(names, requirements, strict=True)):
        payload = payloads[name]
        record = records[name]
        record_ok = artifacts.artifact_record_matches(record, record.get("path", ""))
        if enforce_canonical_record_paths:
            record_ok = record_ok and artifacts.artifact_record_matches(
                record, requirement["path"]
            )
        checks[f"{name}_status_and_record"] = bool(
            payload.get("status") == requirement["required_status"]
            and payload.get("passed") is True
            and record_ok
        )
        rows.append(
            {
                "name": name,
                "status": payload.get("status"),
                "passed": payload.get("passed"),
                "candidate_id": None if index < 2 else payload.get("candidate_id"),
                "candidate_module_tree_sha256": (
                    None
                    if index < 2
                    or not isinstance(payload.get("candidate_module"), Mapping)
                    else payload["candidate_module"].get("tree_sha256")
                ),
                "artifact": copy.deepcopy(dict(record)),
            }
        )

    candidate_record = records[names[2]]
    final_record = records[names[3]]
    protocol_record = records[names[0]]
    lock_record = records[names[1]]
    checks.update(
        {
            "protocol_and_lock_selection_exact": _selection_exact(
                protocol.get("candidate_selection")
            )
            and _selection_exact(lock.get("candidate_selection")),
            "terminal_candidate_binding_exact": binding is not None
            and final.get("candidate_id") == candidate_id
            and final.get("candidate_module") == candidate_module
            and ledger.get("candidate_id") == candidate_id
            and ledger.get("candidate_module") == candidate_module,
            "candidate_and_final_chain_exact": final.get("candidate_freeze")
            == candidate_record
            and ledger.get("candidate_freeze") == candidate_record
            and ledger.get("final_development_receipt") == final_record,
            "protocol_lock_terminal_chain_exact": ledger.get("protocol_freeze")
            == protocol_record
            and ledger.get("execution_lock") == lock_record
            and ledger.get("terminal") is True,
            "selection_rule_remains_exact": candidate.get("candidate_selection_rule")
            == contract.R5_CANDIDATE_SELECTION_RULE
            and final.get("candidate_selection_rule")
            == contract.R5_CANDIDATE_SELECTION_RULE
            and ledger.get("candidate_selection_rule")
            == contract.R5_CANDIDATE_SELECTION_RULE,
            "r5_terminal_pass_exact": ledger.get("terminal") is True
            and ledger.get("passed") is True
            and ledger.get("status") == contract.design.R5_PIPELINE_PASS_STATUS,
            "r5_did_not_execute_q3": ledger.get("q3_paths_opened") == [],
            "no_r5_runtime_promotion": final.get("runtime_promoted") is False
            and ledger.get("runtime_promoted") is False,
        }
    )
    normalized_rows: list[dict[str, Any]] = []
    for requirement, row in zip(requirements, rows, strict=True):
        normalized = copy.deepcopy(row)
        normalized["artifact"]["path"] = requirement["path"]
        normalized_rows.append(normalized)
    passed = all(checks.values())
    if passed and binding is not None:
        contract.bind_candidate(binding["candidate_id"], binding["candidate_module"])
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": (
            contract.PREREQUISITE_PASS_STATUS
            if passed
            else contract.PREREQUISITE_FAIL_STATUS
        ),
        "passed": passed,
        "protocol_id": contract.PROTOCOL_ID,
        "candidate_id": candidate_id if passed else None,
        "candidate_module": copy.deepcopy(candidate_module) if passed else None,
        "prerequisites": normalized_rows,
        "checks": checks,
        "official_verifier_count": 5,
        "official_verifiers": [
            "verify_protocol_freeze",
            "verify_execution_lock",
            "_verify_stage_receipt_claim_bindings:freeze_case_balanced_candidate",
            "_verify_stage_receipt_claim_bindings:finalize_development",
            "verify_terminal_ledger",
        ],
        "qualification_rollouts_opened": 0,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "runtime_promoted": False,
    }


def _canonical_paths() -> dict[str, Path]:
    return {
        row["name"]: artifacts.resolve_relative(row["path"])
        for row in contract.prerequisite_requirements()
    }


def load_and_verify_r5_prerequisites(
    *,
    verifier_bundle: R5VerifierBundle | None = None,
    prerequisite_paths: Mapping[str, Path] | None = None,
    require_q3_unopened: bool = True,
) -> dict[str, Any]:
    """Call each official verifier once, then cross-bind its live artifacts."""

    if require_q3_unopened:
        occupied = [
            artifacts.resolve_relative(path)
            for path in (
                contract.PROTOCOL_FREEZE_PATH,
                contract.EXECUTION_LOCK_PATH,
                contract.NOISE_ROOT,
                contract.RUN_ROOT,
            )
            if os.path.lexists(artifacts.resolve_relative(path))
        ]
        if occupied:
            raise V12R5Q3PrerequisiteError(
                f"Q3 runtime is already occupied: {occupied}"
            )
    paths = (
        _canonical_paths() if prerequisite_paths is None else dict(prerequisite_paths)
    )
    names = [row["name"] for row in contract.prerequisite_requirements()]
    if list(paths) != names or len({path.absolute() for path in paths.values()}) != 5:
        raise V12R5Q3PrerequisiteError(
            "R5 prerequisites must be five distinct ordered artifacts"
        )
    bundle = official_verifiers() if verifier_bundle is None else verifier_bundle
    callbacks = (
        bundle.protocol_freeze,
        bundle.execution_lock,
        bundle.candidate_freeze_receipt,
        bundle.final_development_receipt,
        bundle.terminal_pass_ledger,
    )
    payloads: dict[str, Mapping[str, Any]] = {}
    verifier_failures: dict[str, BaseException] = {}
    for name, callback in zip(names, callbacks, strict=True):
        try:
            value = callback()
        except BaseException as exc:
            verifier_failures[name] = exc
            continue
        if not isinstance(value, Mapping):
            verifier_failures[name] = TypeError(
                f"official R5 verifier returned a non-object: {name}"
            )
            continue
        payloads[name] = dict(value)
    terminal = payloads.get("r5_terminal_pass_ledger")
    if terminal is not None and (
        terminal.get("passed") is not True
        or terminal.get("status") != contract.design.R5_PIPELINE_PASS_STATUS
        or terminal.get("terminal") is not True
    ):
        raise V12R5Q3PrerequisiteError(
            "R5_TERMINAL_NOT_PASS: Q3 remains unopened and candidate-unbound"
        )
    if verifier_failures:
        failed = ", ".join(verifier_failures)
        first = next(iter(verifier_failures.values()))
        raise V12R5Q3PrerequisiteError(
            f"official R5 verifier closure failed: {failed}"
        ) from first
    records = {
        name: artifacts.record(
            paths[name],
            logical_path=(
                contract.prerequisite_requirements()[index]["path"]
                if prerequisite_paths is None
                else paths[name].absolute().as_posix()
            ),
        )
        for index, name in enumerate(names)
    }
    gate = validate_verified_payloads(
        payloads,
        records,
        enforce_canonical_record_paths=prerequisite_paths is None,
    )
    if gate.get("passed") is not True:
        failed = [name for name, value in gate["checks"].items() if value is not True]
        raise V12R5Q3PrerequisiteError(
            f"R5 prerequisite cross-binding failed: {failed}"
        )
    return {"payloads": payloads, "records": records, "gate": gate}


__all__ = [
    "R5VerifierBundle",
    "V12R5Q3PrerequisiteError",
    "load_and_verify_r5_prerequisites",
    "official_verifiers",
    "validate_verified_payloads",
]
