"""Read-only final ABI/hash audit before the V12R9 morphology freeze.

Importing this module performs no filesystem read, publication, rollout, or
training action.  The live audit remains deferred while any required upstream
artifact is absent.  Once the full lineage exists, it reconstructs the exact
prospective protocol and lock in memory and publishes only a JSON result to
stdout.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import importlib.util
import json
import os
import sys
from collections.abc import Callable, Mapping, Sequence
from pathlib import Path, PurePosixPath
from typing import Any

try:
    from . import freeze_h0_v12r9_morphology as freezer
    from . import h0_v12r9_morphology_contract as contract
    from . import h0_v12r9_morphology_gates as gates
    from . import run_h0_v12r9_morphology as runner
except ImportError:
    import freeze_h0_v12r9_morphology as freezer
    import h0_v12r9_morphology_contract as contract
    import h0_v12r9_morphology_gates as gates
    import run_h0_v12r9_morphology as runner


READY_STATUS = "READY_H0_V12R9_MORPH_FINAL_ABI_HASH_AUDIT"
DEFERRED_STATUS = "DEFERRED_H0_V12R9_MORPH_FINAL_ABI_HASH_AUDIT"
AUDIT_ID = "h0_v12r9_morphology_final_read_only_abi_hash_v1"

FilePredicate = Callable[[Path], bool]


def _same_json(left: Any, right: Any) -> bool:
    try:
        return gates.canonical_json_bytes(left) == gates.canonical_json_bytes(right)
    except (TypeError, ValueError):
        return False


def _prospective_record(
    path: PurePosixPath, payload: Mapping[str, Any]
) -> dict[str, Any]:
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
        "path": path.as_posix(),
        "sha256": hashlib.sha256(encoded).hexdigest(),
        "size_bytes": len(encoded),
    }


def required_file_paths() -> dict[str, str]:
    """Return every regular file needed by the final read-only audit."""

    paths = {
        **{
            f"source::{name}": path
            for name, path in contract.SOURCE_RELATIVE_PATHS.items()
        },
        **{
            f"input::{name}": path
            for name, path in contract.INPUT_RELATIVE_PATHS.items()
        },
        "training_validation::verifier": contract.TRAINING_VALIDATION_ENDPOINT[
            "verifier_module"
        ],
        "training_validation::contract": (
            contract.TRAINING_VALIDATION_ENDPOINT["namespace_root"]
            + "/h0_v12r9_training_contract.py"
        ),
        "training_validation::tests": (
            contract.TRAINING_VALIDATION_ENDPOINT["namespace_root"]
            + "/test_h0_v12r9_training.py"
        ),
        "checkpoint_module::manifest": (
            contract.MODULE_EXPORT_PATH / "actor_feature_manifest.json"
        ).as_posix(),
    }
    for case_id in contract.CASE_IDS:
        destination = PurePosixPath(
            contract.q3.canonical_rollout(contract.q3.CANDIDATE_ROLE, case_id)[
                "destination"
            ]
        )
        paths[f"q3_trace::{case_id}"] = (destination / "trace.json").as_posix()
    return dict(sorted(paths.items()))


def required_directory_paths() -> dict[str, str]:
    return {
        "candidate_module": contract.CANDIDATE_MODULE_PATH.as_posix(),
        "checkpoint_zero": contract.CHECKPOINT_ZERO_PATH.as_posix(),
        "checkpoint_module_export": contract.MODULE_EXPORT_PATH.as_posix(),
    }


def availability_snapshot(
    *,
    file_predicate: FilePredicate | None = None,
    directory_predicate: FilePredicate | None = None,
) -> dict[str, Any]:
    """Inspect only presence/type; do not invoke any semantic verifier."""

    is_file = Path.is_file if file_predicate is None else file_predicate
    is_directory = Path.is_dir if directory_predicate is None else directory_predicate
    files = {
        name: {
            "path": path,
            "present_regular_file": bool(is_file(runner.resolve_relative(path))),
        }
        for name, path in required_file_paths().items()
    }
    directories = {
        name: {
            "path": path,
            "present_directory": bool(is_directory(runner.resolve_relative(path))),
        }
        for name, path in required_directory_paths().items()
    }
    missing = sorted(
        [name for name, value in files.items() if not value["present_regular_file"]]
        + [
            name
            for name, value in directories.items()
            if not value["present_directory"]
        ]
    )
    return {
        "passed": not missing,
        "files": files,
        "directories": directories,
        "missing": missing,
    }


def static_abi_audit() -> dict[str, Any]:
    """Validate immutable ABI/config facts that require no run artifacts."""

    corridor_path = runner.resolve_relative(
        contract.SOURCE_RELATIVE_PATHS["causal_corridor"]
    )
    corridor_record = runner.artifact_record(corridor_path)
    source_texts = {
        name: runner.resolve_relative(path).read_text(encoding="utf-8")
        for name, path in contract.SOURCE_RELATIVE_PATHS.items()
    }
    local_source_texts = {
        name: text
        for name, text in source_texts.items()
        if contract.SOURCE_RELATIVE_PATHS[name].startswith(
            contract.ROOT.as_posix() + "/"
        )
    }
    stale_lower = "v12" + "r8"
    stale_upper = "V12" + "R8"
    expected_cases = (
        "deterministic_offset_minus_0p30",
        "deterministic_offset_plus_0p30",
        "stochastic_nominal_seed_130",
        "stochastic_nominal_seed_131",
        "stochastic_nominal_seed_132",
        "stochastic_nominal_seed_133",
    )
    checks = {
        "contract_self_check": contract.contract_self_check().get("passed") is True,
        "exclusive_r9_endpoints": set(contract.UPSTREAM_ENDPOINTS)
        == {"v12r9_terminal", "v12r9_q3_terminal", "v12r9_zero_terminal"}
        and contract.R9_TERMINAL_ENDPOINT["schema_version"]
        == contract.r9.SCHEMA_VERSION
        and contract.Q3_TERMINAL_ENDPOINT["schema_version"]
        == contract.q3.SCHEMA_VERSION
        and contract.ZERO_TERMINAL_ENDPOINT["schema_version"]
        == contract.zero.SCHEMA_VERSION,
        "no_stale_predecessor_source_reference": all(
            stale_lower not in text and stale_upper not in text
            for text in local_source_texts.values()
        ),
        "independent_q3_matrix": contract.CASE_IDS == expected_cases
        and not set(contract.CASE_IDS).intersection(contract.r9.COLLECTION_CASE_IDS)
        and not set(contract.CASE_IDS).intersection(contract.r9.DEVELOPMENT_CASE_IDS),
        "strict_causal_runtime": contract.MORPHOLOGY_REWARD_DELAY_S == 0.04
        and contract.CAUSAL_RUNTIME_CONFIG["terminal_samples_younger_than_delay"]
        == "drop_fail_safe"
        and "episode_end_before_delay" in contract.EXPECTED_CAUSAL_DROP_REASONS,
        "v26_active": contract.V26_RUNTIME_CONFIG["binary_phase_fsm_mode"]
        == "binary_active"
        and contract.V26_RUNTIME_CONFIG["actor_event_source"] == "binary_active_v26",
        "positive_only": gates.exact_config_delta_gate(
            contract.CONTROL_REWARD_CONFIG, contract.POSITIVE_REWARD_CONFIG
        )
        and contract.CONTROL_REWARD_CONFIG["morphology_weight"] == 0.0
        and contract.POSITIVE_REWARD_CONFIG["morphology_weight"] == 0.0025,
        "one_shot": contract.AUTHORITY["retry_authorized"] is False
        and contract.AUTHORITY["actor_updates_authorized"] is False
        and contract.AUTHORITY["critic_updates_authorized"] is False
        and contract.AUTHORITY["ppo_updates_authorized"] is False,
        "repository_root_cwd": contract.REQUIRED_WORKING_DIRECTORY == "repository_root",
        "corridor_hash": corridor_record["sha256"] == contract.FROZEN_CORRIDOR_SHA256,
        "training_handoff_abi": contract.TRAINING_VALIDATION_ENDPOINT[
            "expected_new_updates"
        ]
        == contract.FINAL_TRAINING_ITERATIONS
        == 50
        and contract.TRAINING_VALIDATION_ENDPOINT["preflight_required_before_training"]
        is True
        and contract.TRAINING_VALIDATION_ENDPOINT["postrun_audit_required"] is True,
        "morph_outputs_unclaimed": all(
            not os.path.lexists(runner.resolve_relative(path))
            for path in (
                contract.PROTOCOL_FREEZE_PATH,
                contract.EXECUTION_LOCK_PATH,
                contract.RUN_ROOT,
            )
        ),
    }
    return {
        "passed": all(checks.values()),
        "checks": checks,
        "corridor": corridor_record,
        "source_count": len(source_texts),
        "local_source_count": len(local_source_texts),
    }


def training_validation_source_audit() -> dict[str, Any]:
    """Hash and invoke only the public source-check ABI of v12r9training."""

    endpoint = contract.TRAINING_VALIDATION_ENDPOINT
    path = runner.resolve_relative(endpoint["verifier_module"])
    contract_path = runner.resolve_relative(
        endpoint["namespace_root"] + "/h0_v12r9_training_contract.py"
    )
    before = runner.artifact_record(path)
    contract_before = runner.artifact_record(contract_path)
    text = path.read_text(encoding="utf-8")
    spec = importlib.util.spec_from_file_location(
        "_h0_v12r9_training_validation_read_only", path
    )
    if spec is None or spec.loader is None:
        raise runner.MorphologyExecutionError("training validator import spec failed")
    module = importlib.util.module_from_spec(spec)
    parent = str(path.parent)
    inserted = parent not in sys.path
    if inserted:
        sys.path.insert(0, parent)
    try:
        spec.loader.exec_module(module)
        source_result = module.source_check()
    finally:
        if inserted:
            sys.path.remove(parent)
    after = runner.artifact_record(path)
    contract_after = runner.artifact_record(contract_path)
    literals = (
        endpoint["protocol_id"],
        endpoint["preflight_receipt_path"],
        endpoint["preflight_required_status"],
        endpoint["postrun_audit_path"],
        endpoint["postrun_required_status"],
    )
    checks = {
        "stable_read": before == after,
        "stable_contract": contract_before == contract_after,
        "path": before["path"] == endpoint["verifier_module"],
        "public_literals": all(str(value) in text for value in literals),
        "public_literal_mapping": module.VALIDATION_ABI_LITERALS
        == {name: endpoint[name] for name in module.VALIDATION_ABI_LITERALS},
        "public_source_check": isinstance(source_result, Mapping)
        and source_result.get("passed") is True
        and source_result.get("source_only") is True
        and source_result.get("training_ready") is False,
        "preflight_required": endpoint["preflight_required_before_training"] is True,
        "postrun_required": endpoint["postrun_audit_required"] is True,
        "fifty_updates": endpoint["expected_new_updates"] == 50,
    }
    return {
        "passed": all(checks.values()),
        "checks": checks,
        "verifier": before,
        "contract": contract_before,
        "source_check": copy.deepcopy(dict(source_result)),
    }


def _deferred_payload(
    static: Mapping[str, Any], availability: Mapping[str, Any]
) -> dict[str, Any]:
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": DEFERRED_STATUS,
        "passed": False,
        "ready": False,
        "audit_id": AUDIT_ID,
        "protocol_id": contract.PROTOCOL_ID,
        "static_abi": copy.deepcopy(dict(static)),
        "availability": copy.deepcopy(dict(availability)),
        "semantic_verifiers_invoked": False,
        "write_operations": 0,
        "freeze_published": False,
        "lock_published": False,
        "rollouts_executed": 0,
        "training_updates_executed": 0,
    }


def audit_live(
    *,
    file_predicate: FilePredicate | None = None,
    directory_predicate: FilePredicate | None = None,
) -> dict[str, Any]:
    """Return DEFERRED or a complete in-memory pre-freeze attestation."""

    runner.require_repository_root_cwd()
    static = static_abi_audit()
    if static.get("passed") is not True:
        raise runner.MorphologyExecutionError("static morphology ABI audit failed")
    availability = availability_snapshot(
        file_predicate=file_predicate,
        directory_predicate=directory_predicate,
    )
    if availability.get("passed") is not True:
        return _deferred_payload(static, availability)

    training_validation = training_validation_source_audit()
    if training_validation.get("passed") is not True:
        raise runner.MorphologyExecutionError("training validation source ABI failed")
    upstream = runner.semantic_upstream_payload()
    upstream_gate = gates.upstream_terminal_gate(upstream)
    if upstream_gate.get("passed") is not True:
        raise runner.MorphologyExecutionError("upstream semantic gate failed")
    binding = dict(upstream_gate["binding"])
    source_closure = runner.source_closure_snapshot()
    conditions = runner.condition_snapshot()
    capabilities = runner.inspect_q3_control_capabilities()
    checkpoint_module = runner.checkpoint_module_snapshot(binding)
    protocol = freezer.build_protocol_freeze_payload(
        upstream_payload=upstream,
        source_closure=source_closure,
        conditions=conditions,
        q3_capabilities=capabilities,
        checkpoint_module=checkpoint_module,
    )
    protocol_record = _prospective_record(contract.PROTOCOL_FREEZE_PATH, protocol)
    lock = freezer.build_lock_payload(
        protocol,
        protocol_artifact=protocol_record,
    )
    plan = runner.build_execution_plan(lock)
    after = static_abi_audit()
    if after.get("passed") is not True or not _same_json(after, static):
        raise runner.MorphologyExecutionError("static ABI changed during audit")

    hashes = {
        "upstream_payload_sha256": gates.canonical_json_sha256(upstream),
        "source_closure_sha256": gates.canonical_json_sha256(source_closure),
        "conditions_sha256": gates.canonical_json_sha256(conditions),
        "q3_capabilities_sha256": gates.canonical_json_sha256(capabilities),
        "checkpoint_module_sha256": gates.canonical_json_sha256(checkpoint_module),
        "prospective_protocol_sha256": protocol_record["sha256"],
        "prospective_lock_sha256": gates.canonical_json_sha256(lock),
        "execution_plan_sha256": gates.canonical_json_sha256(plan),
        "training_validator_sha256": training_validation["verifier"]["sha256"],
        "training_contract_sha256": training_validation["contract"]["sha256"],
    }
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": READY_STATUS,
        "passed": True,
        "ready": True,
        "audit_id": AUDIT_ID,
        "protocol_id": contract.PROTOCOL_ID,
        "binding": binding,
        "static_abi": static,
        "availability": availability,
        "training_validation": training_validation,
        "hashes": hashes,
        "prospective_protocol": protocol,
        "prospective_execution_lock": lock,
        "execution_plan": plan,
        "semantic_verifiers_invoked": True,
        "write_operations": 0,
        "freeze_published": False,
        "lock_published": False,
        "rollouts_executed": 0,
        "training_updates_executed": 0,
    }


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--audit", action="store_true", required=True)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    _parser().parse_args(argv)
    result = audit_live()
    print(json.dumps(result, indent=2, sort_keys=True, allow_nan=False))
    return 0 if result.get("passed") is True else 2


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = [
    "AUDIT_ID",
    "DEFERRED_STATUS",
    "READY_STATUS",
    "audit_live",
    "availability_snapshot",
    "required_directory_paths",
    "required_file_paths",
    "static_abi_audit",
    "training_validation_source_audit",
]
