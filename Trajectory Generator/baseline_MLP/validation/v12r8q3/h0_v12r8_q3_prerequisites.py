"""Deferred R8 binding and semantic prerequisite verifier hooks for Q3.

Nothing in this module reads the filesystem.  Future runtime code must provide
five live verifier callbacks plus records of the exact artifacts they verified.
The candidate and final-development receipts are rechecked semantically here;
mere ``passed: true`` payloads are insufficient.
"""

from __future__ import annotations

import copy
from collections.abc import Callable, Mapping
from dataclasses import dataclass
from pathlib import PurePosixPath
from typing import Any

try:
    from . import h0_v12r8_q3_artifacts as artifacts
    from . import h0_v12r8_q3_qualification_contract as contract
except ImportError:  # Direct test execution with this directory on sys.path.
    import h0_v12r8_q3_artifacts as artifacts
    import h0_v12r8_q3_qualification_contract as contract


class V12R8Q3PrerequisiteError(RuntimeError):
    """Raised unless the exact terminal-PASS R8 lineage is verified."""


@dataclass(frozen=True)
class R8SemanticVerifierHooks:
    """The only five live R8 verifier entry points accepted by this scaffold."""

    verify_protocol_freeze: Callable[[], Mapping[str, Any]]
    verify_execution_lock: Callable[[], Mapping[str, Any]]
    verify_candidate_freeze_receipt: Callable[[], Mapping[str, Any]]
    verify_final_development_receipt: Callable[[], Mapping[str, Any]]
    verify_terminal_ledger: Callable[[], Mapping[str, Any]]


R8_CANDIDATE_ID: str | None = None
R8_CANDIDATE_MODULE: dict[str, Any] | None = None


def _zero_int(value: Any) -> bool:
    return type(value) is int and value == 0


def _one_int(value: Any) -> bool:
    return type(value) is int and value == 1


def validate_candidate_tree(candidate_id: Any, candidate_module: Any) -> dict[str, Any]:
    """Validate the exact five-file standard R8 actor tree and identity."""

    module = artifacts.validate_exact_tree(
        candidate_module,
        expected_path=contract.CANDIDATE_MODULE_PATH,
        expected_files=contract.CANDIDATE_REQUIRED_FILES,
    )
    expected_id = contract.r8.candidate_id(module["tree_sha256"])
    if candidate_id != expected_id:
        raise artifacts.V12R8Q3ArtifactError(
            "R8 candidate identity does not bind the exact five-file tree"
        )
    return {"candidate_id": candidate_id, "candidate_module": module}


def validate_actor_feature_manifest(
    manifest: Any,
    *,
    manifest_record: Any,
    candidate_module: Any,
) -> dict[str, Any]:
    """Validate the R8 standard 35->512->512->2 actor feature contract."""

    data = dict(manifest) if isinstance(manifest, Mapping) else {}
    module = artifacts.validate_exact_tree(
        candidate_module,
        expected_path=contract.CANDIDATE_MODULE_PATH,
        expected_files=contract.CANDIDATE_REQUIRED_FILES,
    )
    rows = {row["path"]: row for row in module["files"]}
    feature_row = rows["actor_feature_manifest.json"]
    state_row = rows["module_state.pkl"]
    full_feature_path = (
        PurePosixPath(module["path"]) / "actor_feature_manifest.json"
    ).as_posix()
    record = dict(manifest_record) if isinstance(manifest_record, Mapping) else {}
    record_ok = (
        artifacts.artifact_record_matches(record, full_feature_path)
        and record.get("sha256") == feature_row["sha256"]
        and record.get("size_bytes") == feature_row["size_bytes"]
    )
    semantic_ok = (
        set(data) == set(contract.ACTOR_FEATURE_MANIFEST_KEYS)
        and type(data.get("schema_version")) is int
        and data.get("schema_version") == 1
        and data.get("status") == contract.r8.ACTOR_FEATURE_MANIFEST_STATUS
        and data.get("topology_id") == contract.r8.TOPOLOGY_ID
        and data.get("fit_contract_id") == contract.r8.FIT_CONTRACT_ID
        and type(data.get("actor_feature_count")) is int
        and data.get("actor_feature_count") == contract.EXPECTED_ACTOR_FEATURES
        and data.get("actor_feature_names") == list(contract.ACTOR_FEATURE_NAMES)
        and data.get("fcnet_hiddens") == list(contract.EXPECTED_HIDDENS)
        and data.get("disabled_clock_columns") == list(contract.DISABLED_CLOCK_COLUMNS)
        and artifacts.is_sha256(data.get("actor_digest"))
        and data.get("module_state_sha256") == state_row["sha256"]
    )
    if not (record_ok and semantic_ok):
        raise artifacts.V12R8Q3ArtifactError(
            "R8 actor feature manifest is not the exact standard W512 contract"
        )
    return copy.deepcopy(data)


def candidate_freeze_semantics(
    payload: Any,
    *,
    actor_manifest: Any,
    actor_manifest_record: Any,
) -> dict[str, Any]:
    """Pure semantic verifier for the R8 candidate-freeze receipt."""

    data = dict(payload) if isinstance(payload, Mapping) else {}
    try:
        binding = validate_candidate_tree(
            data.get("candidate_id"), data.get("candidate_module")
        )
        manifest = validate_actor_feature_manifest(
            actor_manifest,
            manifest_record=actor_manifest_record,
            candidate_module=binding["candidate_module"],
        )
    except (TypeError, ValueError):
        binding = None
        manifest = None
    checks = {
        "status": data.get("status") == contract.r8.CANDIDATE_FREEZE_PASS_STATUS
        and data.get("passed") is True,
        "identity": data.get("protocol_id") == contract.r8.PROTOCOL_ID
        and data.get("pipeline_id") == contract.r8.PIPELINE_ID
        and data.get("candidate_selection_rule") == contract.CANDIDATE_SELECTION_RULE,
        "candidate_tree": binding is not None,
        "actor_manifest": manifest is not None
        and data.get("actor_feature_manifest") == actor_manifest_record
        and data.get("actor_digest") == manifest.get("actor_digest"),
        "fit_semantics": data.get("candidate_frozen") is True
        and data.get("fit_gate_passed") is True
        and data.get("standard_actor") is True
        and data.get("warm_start_target_512_compatible") is True,
        "one_fit_no_rl_updates": _one_int(data.get("actor_fit_count"))
        and _one_int(data.get("actor_updates"))
        and _zero_int(data.get("critic_updates"))
        and _zero_int(data.get("ppo_updates")),
        "qualification_closed": data.get("q3_paths_opened") == []
        and data.get("runtime_promoted") is False,
    }
    passed = all(checks.values())
    return {
        "status": contract.r8.CANDIDATE_FREEZE_PASS_STATUS
        if passed
        else contract.PREREQUISITE_FAIL_STATUS,
        "passed": passed,
        "checks": checks,
        "candidate_id": binding["candidate_id"] if passed and binding else None,
        "candidate_module": (
            copy.deepcopy(binding["candidate_module"]) if passed and binding else None
        ),
    }


def final_development_semantics(
    payload: Any,
    *,
    binding: Mapping[str, Any],
    candidate_freeze_record: Any,
) -> dict[str, Any]:
    """Pure semantic verifier for the six-case R8 final-development receipt."""

    data = dict(payload) if isinstance(payload, Mapping) else {}
    rows = data.get("case_gates")
    rows_ok = isinstance(rows, list) and len(rows) == len(
        contract.r8.DEVELOPMENT_CASE_IDS
    )
    if rows_ok:
        for row, case_id in zip(rows, contract.r8.DEVELOPMENT_CASE_IDS, strict=True):
            current = dict(row) if isinstance(row, Mapping) else {}
            if (
                set(current) != {"case_id", "passed"}
                or current.get("case_id") != case_id
                or current.get("passed") is not True
            ):
                rows_ok = False
                break
    checks = {
        "status": data.get("status") == contract.r8.DEVELOPMENT_PASS_STATUS
        and data.get("passed") is True,
        "identity": data.get("protocol_id") == contract.r8.PROTOCOL_ID
        and data.get("pipeline_id") == contract.r8.PIPELINE_ID
        and data.get("candidate_id") == binding.get("candidate_id")
        and data.get("candidate_module") == binding.get("candidate_module"),
        "candidate_freeze_chain": data.get("candidate_freeze")
        == candidate_freeze_record,
        "six_of_six": rows_ok
        and data.get("collection_round_count") == 6
        and _one_int(data.get("actor_fit_count"))
        and data.get("development_count") == 6
        and data.get("passing_development_count") == 6
        and data.get("failed_development_count") == 0,
        "pure_development": _zero_int(data.get("development_actor_updates"))
        and _zero_int(data.get("development_critic_updates"))
        and _zero_int(data.get("development_ppo_updates"))
        and _zero_int(data.get("teacher_query_count"))
        and _zero_int(data.get("mean_blend_count"))
        and _zero_int(data.get("safety_intervention_count")),
        "qualification_closed": data.get("q3_paths_opened") == []
        and data.get("runtime_promoted") is False,
    }
    passed = all(checks.values())
    return {
        "status": contract.r8.DEVELOPMENT_PASS_STATUS
        if passed
        else contract.PREREQUISITE_FAIL_STATUS,
        "passed": passed,
        "checks": checks,
    }


def terminal_ledger_semantics(
    payload: Any,
    *,
    binding: Mapping[str, Any],
    protocol_record: Any,
    lock_record: Any,
    candidate_freeze_record: Any,
    final_development_record: Any,
) -> dict[str, Any]:
    """Pure semantic verifier for the immutable terminal-PASS R8 ledger."""

    data = dict(payload) if isinstance(payload, Mapping) else {}
    checks = {
        "terminal_pass": data.get("status") == contract.r8.PIPELINE_TERMINAL_PASS_STATUS
        and data.get("passed") is True
        and data.get("terminal") is True,
        "identity": data.get("protocol_id") == contract.r8.PROTOCOL_ID
        and data.get("pipeline_id") == contract.r8.PIPELINE_ID
        and data.get("candidate_id") == binding.get("candidate_id")
        and data.get("candidate_module") == binding.get("candidate_module"),
        "artifact_chain": data.get("protocol_freeze") == protocol_record
        and data.get("execution_lock") == lock_record
        and data.get("candidate_freeze") == candidate_freeze_record
        and data.get("final_development_receipt") == final_development_record,
        "one_fit_no_rl_updates": _one_int(data.get("actor_fit_count"))
        and _one_int(data.get("actor_updates"))
        and data.get("collection_round_count") == 6
        and _zero_int(data.get("critic_updates"))
        and _zero_int(data.get("ppo_updates")),
        "q3_unopened": data.get("q3_paths_opened") == [],
        "not_promoted": data.get("runtime_promoted") is False,
    }
    passed = all(checks.values())
    return {
        "status": contract.r8.PIPELINE_TERMINAL_PASS_STATUS
        if passed
        else contract.PREREQUISITE_FAIL_STATUS,
        "passed": passed,
        "checks": checks,
    }


def validate_source_closure(records: Any) -> dict[str, Any]:
    """Require records for every source/runtime hook/input declared by Q3."""

    supplied = dict(records) if isinstance(records, Mapping) else {}
    expected = {
        **contract.QUALIFICATION_SOURCE_PATHS,
        **contract.DEFERRED_RUNTIME_SOURCE_PATHS,
        **contract.QUALIFICATION_INPUT_PATHS,
    }
    checks: dict[str, bool] = {"exact_keys": set(supplied) == set(expected)}
    for name, path in expected.items():
        checks[f"record::{name}"] = artifacts.artifact_record_matches(
            supplied.get(name), path
        )
    for name, expected_sha256 in contract.QUALIFICATION_INPUT_SHA256.items():
        record = supplied.get(name)
        checks[f"input_hash::{name}"] = (
            isinstance(record, Mapping) and record.get("sha256") == expected_sha256
        )
    passed = all(checks.values())
    return {"passed": passed, "checks": checks, "records": copy.deepcopy(supplied)}


def deferred_prerequisite_state() -> dict[str, Any]:
    """Expose the source-only state without pretending declared files are absent.

    This pure helper deliberately performs no filesystem access. Runtime-source
    presence and byte closure are reported by ``source_closure_readiness`` in the
    freezer, so the names here are declarations awaiting that independent check,
    not missing files.
    """

    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.PREREQUISITE_FAIL_STATUS,
        "passed": False,
        "candidate_binding_state": contract.CANDIDATE_BINDING_STATE,
        "candidate_id": None,
        "candidate_module": None,
        "missing_prerequisites": [
            row["name"] for row in contract.PREREQUISITE_REQUIREMENTS
        ],
        "declared_runtime_sources": list(contract.DEFERRED_RUNTIME_SOURCE_PATHS),
        "runtime_source_verification": "DEFERRED_TO_SOURCE_CLOSURE_READINESS",
        "qualification_execution_authorized": False,
    }


def validate_verified_payloads(
    payloads: Any,
    records: Any,
    *,
    actor_manifest: Any,
    actor_manifest_record: Any,
) -> dict[str, Any]:
    """Cross-bind five live verifier payloads and their exact artifact records."""

    raw_payloads = dict(payloads) if isinstance(payloads, Mapping) else {}
    payload_map = {
        name: dict(value) if isinstance(value, Mapping) else {}
        for name, value in raw_payloads.items()
    }
    record_map = dict(records) if isinstance(records, Mapping) else {}
    requirements = contract.prerequisite_requirements()
    names = [row["name"] for row in requirements]
    checks: dict[str, bool] = {
        "exact_five_ordered_payloads": list(payload_map) == names,
        "exact_five_ordered_records": list(record_map) == names,
    }
    if not all(checks.values()):
        return {
            "schema_version": contract.SCHEMA_VERSION,
            "status": contract.PREREQUISITE_FAIL_STATUS,
            "passed": False,
            "checks": checks,
            "candidate_id": None,
            "candidate_module": None,
        }

    for requirement in requirements:
        name = requirement["name"]
        payload = payload_map[name]
        checks[f"record::{name}"] = artifacts.artifact_record_matches(
            record_map[name], requirement["path"]
        )
        checks[f"status::{name}"] = (
            isinstance(payload, Mapping)
            and payload.get("status") == requirement["required_status"]
            and payload.get("passed") is True
        )

    candidate_result = candidate_freeze_semantics(
        payload_map[names[2]],
        actor_manifest=actor_manifest,
        actor_manifest_record=actor_manifest_record,
    )
    binding = {
        "candidate_id": candidate_result.get("candidate_id"),
        "candidate_module": candidate_result.get("candidate_module"),
    }
    final_result = final_development_semantics(
        payload_map[names[3]],
        binding=binding,
        candidate_freeze_record=record_map[names[2]],
    )
    terminal_result = terminal_ledger_semantics(
        payload_map[names[4]],
        binding=binding,
        protocol_record=record_map[names[0]],
        lock_record=record_map[names[1]],
        candidate_freeze_record=record_map[names[2]],
        final_development_record=record_map[names[3]],
    )
    checks.update(
        {
            "candidate_receipt_semantic": candidate_result.get("passed") is True,
            "final_receipt_semantic": final_result.get("passed") is True,
            "terminal_ledger_semantic": terminal_result.get("passed") is True,
            "protocol_selection_exact": payload_map[names[0]].get(
                "candidate_selection_rule"
            )
            == contract.CANDIDATE_SELECTION_RULE
            and payload_map[names[0]].get("candidate_module_path")
            == contract.CANDIDATE_MODULE_PATH.as_posix(),
            "lock_selection_exact": payload_map[names[1]].get(
                "candidate_selection_rule"
            )
            == contract.CANDIDATE_SELECTION_RULE
            and payload_map[names[1]].get("candidate_module_path")
            == contract.CANDIDATE_MODULE_PATH.as_posix(),
        }
    )
    passed = all(checks.values())
    if passed:
        bind_candidate(binding["candidate_id"], binding["candidate_module"])
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.PREREQUISITE_PASS_STATUS
        if passed
        else contract.PREREQUISITE_FAIL_STATUS,
        "passed": passed,
        "checks": checks,
        "candidate_id": binding["candidate_id"] if passed else None,
        "candidate_module": (
            copy.deepcopy(binding["candidate_module"]) if passed else None
        ),
        "official_verifier_count": 5,
        "official_verifiers": [row["semantic_verifier"] for row in requirements],
        "qualification_execution_authorized": False,
    }


def invoke_verifier_hooks(hooks: R8SemanticVerifierHooks) -> dict[str, Any]:
    """Call the five named hooks once; artifact records remain caller-owned."""

    callbacks = (
        hooks.verify_protocol_freeze,
        hooks.verify_execution_lock,
        hooks.verify_candidate_freeze_receipt,
        hooks.verify_final_development_receipt,
        hooks.verify_terminal_ledger,
    )
    names = [row["name"] for row in contract.PREREQUISITE_REQUIREMENTS]
    payloads: dict[str, dict[str, Any]] = {}
    for name, callback in zip(names, callbacks, strict=True):
        value = callback()
        if not isinstance(value, Mapping):
            raise V12R8Q3PrerequisiteError(
                f"R8 semantic verifier returned a non-object: {name}"
            )
        payloads[name] = dict(value)
    return payloads


def bind_candidate(candidate_id: Any, candidate_module: Any) -> dict[str, Any]:
    """Bind one exact terminal-PASS R8 candidate once per process."""

    global R8_CANDIDATE_ID, R8_CANDIDATE_MODULE
    binding = validate_candidate_tree(candidate_id, candidate_module)
    if R8_CANDIDATE_ID is not None and (
        R8_CANDIDATE_ID != binding["candidate_id"]
        or R8_CANDIDATE_MODULE != binding["candidate_module"]
    ):
        raise V12R8Q3PrerequisiteError("R8-Q3 candidate binding is immutable")
    R8_CANDIDATE_ID = str(binding["candidate_id"])
    R8_CANDIDATE_MODULE = copy.deepcopy(binding["candidate_module"])
    return current_candidate_binding()


def current_candidate_binding() -> dict[str, Any]:
    if R8_CANDIDATE_ID is None or R8_CANDIDATE_MODULE is None:
        raise V12R8Q3PrerequisiteError(
            "R8-Q3 remains fail-closed: terminal R8 candidate is not bound"
        )
    return {
        "candidate_id": R8_CANDIDATE_ID,
        "candidate_module": copy.deepcopy(R8_CANDIDATE_MODULE),
    }


def clear_candidate_binding_for_tests() -> None:
    global R8_CANDIDATE_ID, R8_CANDIDATE_MODULE
    R8_CANDIDATE_ID = None
    R8_CANDIDATE_MODULE = None


__all__ = [
    "R8SemanticVerifierHooks",
    "V12R8Q3PrerequisiteError",
    "bind_candidate",
    "candidate_freeze_semantics",
    "clear_candidate_binding_for_tests",
    "current_candidate_binding",
    "deferred_prerequisite_state",
    "final_development_semantics",
    "invoke_verifier_hooks",
    "terminal_ledger_semantics",
    "validate_actor_feature_manifest",
    "validate_candidate_tree",
    "validate_source_closure",
    "validate_verified_payloads",
]
