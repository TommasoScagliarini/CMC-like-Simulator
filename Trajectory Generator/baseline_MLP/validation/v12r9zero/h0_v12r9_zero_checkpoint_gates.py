"""Pure fail-closed gates for the deferred V12R9 checkpoint-zero port."""

from __future__ import annotations

import hashlib
import json
from collections.abc import Mapping, Sequence
from pathlib import PurePosixPath
from typing import Any

try:
    from . import h0_v12r9_zero_checkpoint_contract as contract
except ImportError:  # Direct execution with this directory on ``sys.path``.
    import h0_v12r9_zero_checkpoint_contract as contract


_ARTIFACT_KEYS = frozenset({"path", "sha256", "size_bytes"})
_TREE_KEYS = frozenset({"path", "tree_sha256", "file_count", "files"})
_TREE_FILE_KEYS = frozenset({"path", "sha256", "size_bytes"})
_BINDING_KEYS = frozenset({"candidate_id", "candidate_module", "actor_digest"})
_ACTOR_SURFACE_KEYS = frozenset(
    {"actor_digest", "actor_state_sha256", "actor_key_count", "actor_byte_count"}
)
_CRITIC_SURFACE_KEYS = frozenset(
    {"critic_state_sha256", "critic_key_count", "critic_byte_count"}
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


def canonical_json_sha256(value: Any) -> str:
    """Return a deterministic digest or an empty string for invalid JSON."""

    try:
        encoded = json.dumps(
            value,
            sort_keys=True,
            separators=(",", ":"),
            allow_nan=False,
        ).encode("utf-8")
    except (TypeError, ValueError):
        return ""
    return hashlib.sha256(encoded).hexdigest()


def _sha(value: Any) -> bool:
    return (
        isinstance(value, str)
        and len(value) == 64
        and value == value.lower()
        and all(character in "0123456789abcdef" for character in value)
    )


def _zero_int(value: Any) -> bool:
    return type(value) is int and value == 0


def _one_int(value: Any) -> bool:
    return type(value) is int and value == 1


def _positive_int(value: Any) -> bool:
    return type(value) is int and value > 0


def _canonical_relative_path(value: Any) -> bool:
    if not isinstance(value, str) or value in {"", "."} or "\\" in value:
        return False
    path = PurePosixPath(value)
    return (
        not path.is_absolute() and ".." not in path.parts and path.as_posix() == value
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
        or type(data.get("file_count")) is not int
        or data["file_count"] <= 0
        or data["file_count"] != len(rows)
    ):
        return False
    paths: list[str] = []
    for raw in rows:
        row = _mapping(raw)
        if (
            set(row) != _TREE_FILE_KEYS
            or not _canonical_relative_path(row.get("path"))
            or _sha(row.get("sha256")) is False
            or _positive_int(row.get("size_bytes")) is False
        ):
            return False
        paths.append(row["path"])
    if paths != sorted(paths) or len(paths) != len(set(paths)):
        return False
    return required_files is None or set(paths) == set(required_files)


def candidate_binding_valid(value: Any) -> bool:
    data = _mapping(value)
    module = data.get("candidate_module")
    return (
        set(data) == _BINDING_KEYS
        and tree_record_valid(
            module,
            expected_path=contract.CANDIDATE_MODULE_PATH.as_posix(),
            required_files=contract.CANDIDATE_REQUIRED_FILES,
        )
        and data.get("candidate_id")
        == contract.candidate_id_for_tree(_mapping(module).get("tree_sha256", ""))
        and _sha(data.get("actor_digest"))
    )


def actor_manifest_gate(
    value: Any, *, candidate_module: Mapping[str, Any]
) -> dict[str, Any]:
    """Validate the exact standard 35->512->512->2 actor ABI."""

    data = _mapping(value)
    rows = {
        row.get("path"): row
        for row in _sequence(_mapping(candidate_module).get("files"))
        if isinstance(row, Mapping)
    }
    module_state = _mapping(rows.get("module_state.pkl"))
    checks = {
        "exact_schema": set(data) == set(contract.q3.ACTOR_FEATURE_MANIFEST_KEYS),
        "identity": data.get("schema_version") == 1
        and data.get("status") == contract.r9.ACTOR_FEATURE_MANIFEST_STATUS
        and data.get("topology_id") == contract.r9.TOPOLOGY_ID
        and data.get("fit_contract_id") == contract.r9.FIT_CONTRACT_ID,
        "features": data.get("actor_feature_count") == contract.EXPECTED_ACTOR_FEATURES
        and data.get("actor_feature_names") == list(contract.q3.ACTOR_FEATURE_NAMES),
        "w512": data.get("fcnet_hiddens") == list(contract.EXPECTED_HIDDENS),
        "disabled_clock": data.get("disabled_clock_columns")
        == list(contract.DISABLED_CLOCK_COLUMNS),
        "state": _sha(data.get("module_state_sha256"))
        and data.get("module_state_sha256") == module_state.get("sha256"),
        "actor_digest": _sha(data.get("actor_digest")),
    }
    return {"passed": all(checks.values()), "checks": checks}


def _semantic_attestation_valid(
    value: Any,
    *,
    endpoint: Mapping[str, Any],
    result: Mapping[str, Any],
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
    """Require semantic terminal PASS from R9 and Q3 for one exact candidate."""

    data = _mapping(payload)
    r9_ledger = _mapping(data.get("v12r9_terminal"))
    q3_ledger = _mapping(data.get("v12r9_q3_terminal"))
    attestations = _mapping(data.get("semantic_attestations"))
    r9_module = _mapping(r9_ledger.get("candidate_module"))
    q3_module = _mapping(q3_ledger.get("candidate_module"))
    candidate_id = r9_ledger.get("candidate_id")
    r9_endpoint = contract.R9_TERMINAL_ENDPOINT
    q3_endpoint = contract.Q3_TERMINAL_ENDPOINT
    checks = {
        "r9_semantic_verifier": _semantic_attestation_valid(
            attestations.get("v12r9_terminal"),
            endpoint=r9_endpoint,
            result=r9_ledger,
        ),
        "q3_semantic_verifier": _semantic_attestation_valid(
            attestations.get("v12r9_q3_terminal"),
            endpoint=q3_endpoint,
            result=q3_ledger,
        ),
        "r9_terminal_pass": r9_ledger.get("schema_version")
        == r9_endpoint["schema_version"]
        and r9_ledger.get("status") == r9_endpoint["required_status"]
        and r9_ledger.get("passed") is True
        and r9_ledger.get("terminal") is True
        and r9_ledger.get("protocol_id") == r9_endpoint["protocol_id"]
        and r9_ledger.get("pipeline_id") == r9_endpoint["pipeline_id"]
        and r9_ledger.get("error") is None
        and r9_ledger.get("candidate_selection_rule")
        == contract.CANDIDATE_SELECTION_RULE
        and r9_ledger.get("actor_updates") == r9_endpoint["expected_actor_updates"]
        and r9_ledger.get("critic_updates") == r9_endpoint["expected_critic_updates"]
        and r9_ledger.get("ppo_updates") == r9_endpoint["expected_ppo_updates"]
        and r9_ledger.get("checkpoint_zero_created") is False
        and r9_ledger.get("positive_morphology_enabled") is False
        and r9_ledger.get("runtime_promoted") is False
        and r9_ledger.get("qualification_executed") is False
        and r9_ledger.get("q3_paths_opened") == []
        and r9_ledger.get("next_stage") == r9_endpoint["required_next_stage"]
        and artifact_record_valid(
            r9_ledger.get("candidate_freeze"),
            expected_path=contract.R9_CANDIDATE_FREEZE_ENDPOINT["path"],
        )
        and artifact_record_valid(
            r9_ledger.get("final_development_receipt"),
            expected_path=contract.R9_FINAL_DEVELOPMENT_ENDPOINT["path"],
        ),
        "q3_terminal_pass": q3_ledger.get("schema_version")
        == q3_endpoint["schema_version"]
        and q3_ledger.get("status") == q3_endpoint["required_status"]
        and q3_ledger.get("passed") is True
        and q3_ledger.get("terminal") is True
        and q3_ledger.get("protocol_id") == q3_endpoint["protocol_id"]
        and q3_ledger.get("pipeline_id") == q3_endpoint["pipeline_id"]
        and q3_ledger.get("error") is None
        and q3_ledger.get("error_type") is None
        and q3_ledger.get("actor_updates") == q3_endpoint["expected_actor_updates"]
        and q3_ledger.get("critic_updates") == q3_endpoint["expected_critic_updates"]
        and q3_ledger.get("ppo_updates") == q3_endpoint["expected_ppo_updates"]
        and q3_ledger.get("checkpoint_zero_created") is False
        and q3_ledger.get("morphology_weight") == 0.0
        and q3_ledger.get("positive_morphology_enabled") is False
        and q3_ledger.get("runtime_promoted") is False
        and q3_ledger.get("aggregate_requires_6_of_6") is True
        and q3_ledger.get("retry_authorized") is False
        and q3_ledger.get("resume_authorized") is False
        and q3_ledger.get("rescue_authorized") is False
        and q3_ledger.get("sweep_authorized") is False
        and q3_ledger.get("post_hoc_tuning_authorized") is False
        and q3_ledger.get("compensation_authorized") is False
        and q3_ledger.get("next_stage") == q3_endpoint["required_next_stage"]
        and artifact_record_valid(
            q3_ledger.get("final_receipt"),
            expected_path=contract.Q3_FINAL_ENDPOINT["path"],
        ),
        "same_candidate": isinstance(candidate_id, str)
        and candidate_id == q3_ledger.get("candidate_id")
        and _strict_equal(r9_module, q3_module),
        "five_file_candidate": tree_record_valid(
            r9_module,
            expected_path=contract.CANDIDATE_MODULE_PATH.as_posix(),
            required_files=contract.CANDIDATE_REQUIRED_FILES,
        )
        and candidate_id
        == contract.candidate_id_for_tree(r9_module.get("tree_sha256", "")),
    }
    passed = all(checks.values())
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": (
            "PASS_H0_V12R9_ZERO_UPSTREAM_PREREQUISITES"
            if passed
            else "FAIL_H0_V12R9_ZERO_UPSTREAM_PREREQUISITES"
        ),
        "passed": passed,
        "checks": checks,
        "candidate_id": candidate_id if passed else None,
        "candidate_module": r9_module if passed else None,
    }


def source_closure_gate(value: Any) -> dict[str, Any]:
    data = _mapping(value)
    sources = _mapping(data.get("sources"))
    inputs = _mapping(data.get("inputs"))
    candidate = data.get("candidate_module")
    checks = {
        "sources": set(sources) == set(contract.SOURCE_RELATIVE_PATHS)
        and all(
            artifact_record_valid(
                sources[name], expected_path=contract.SOURCE_RELATIVE_PATHS[name]
            )
            for name in contract.SOURCE_RELATIVE_PATHS
        ),
        "inputs": set(inputs) == set(contract.INPUT_RELATIVE_PATHS)
        and all(
            artifact_record_valid(
                inputs[name], expected_path=contract.INPUT_RELATIVE_PATHS[name]
            )
            for name in contract.INPUT_RELATIVE_PATHS
        ),
        "candidate": tree_record_valid(
            candidate,
            expected_path=contract.CANDIDATE_MODULE_PATH.as_posix(),
            required_files=contract.CANDIDATE_REQUIRED_FILES,
        ),
    }
    return {"passed": all(checks.values()), "checks": checks}


def _actor_surface_valid(value: Any, *, actor_digest: str) -> bool:
    data = _mapping(value)
    return (
        set(data) == _ACTOR_SURFACE_KEYS
        and data.get("actor_digest") == actor_digest
        and _sha(data.get("actor_state_sha256"))
        and data.get("actor_key_count") == 10
        and _positive_int(data.get("actor_byte_count"))
    )


def _critic_surface_valid(value: Any) -> bool:
    data = _mapping(value)
    return (
        set(data) == _CRITIC_SURFACE_KEYS
        and _sha(data.get("critic_state_sha256"))
        and _positive_int(data.get("critic_key_count"))
        and _positive_int(data.get("critic_byte_count"))
    )


def _progress_surface_valid(value: Any) -> bool:
    data = _mapping(value)
    return set(data) == set(contract.ZERO_COUNTER_NAMES) and all(
        _zero_int(data[name]) for name in contract.ZERO_COUNTER_NAMES
    )


def _optimizer_snapshot_valid(value: Any) -> bool:
    data = _mapping(value)
    names = data.get("trainable_parameter_names")
    optimizers = _sequence(data.get("optimizers"))
    return (
        set(data)
        == {
            "optimizer_state_empty",
            "trainable_parameter_count",
            "trainable_parameter_names",
            "all_trainable_parameters_registered_once",
            "optimizers",
        }
        and data.get("optimizer_state_empty") is True
        and _positive_int(data.get("trainable_parameter_count"))
        and type(names) is list
        and len(names) == data.get("trainable_parameter_count")
        and names == sorted(names)
        and len(names) == len(set(names))
        and data.get("all_trainable_parameters_registered_once") is True
        and bool(optimizers)
        and all(
            isinstance(item, Mapping)
            and item.get("state_entry_count") == 0
            and isinstance(item.get("optimizer_name"), str)
            and bool(item["optimizer_name"])
            and isinstance(item.get("optimizer_type"), str)
            and bool(item["optimizer_type"])
            and type(item.get("param_groups")) is list
            and bool(item["param_groups"])
            for item in optimizers
        )
    )


def checkpoint_tree_gate(value: Any) -> dict[str, Any]:
    data = _mapping(value)
    paths = {
        row.get("path")
        for row in _sequence(data.get("files"))
        if isinstance(row, Mapping)
    }
    checks = {
        "tree": tree_record_valid(
            data,
            expected_path=contract.CHECKPOINT_PATH.as_posix(),
        ),
        "full_rllib": contract.CHECKPOINT_REQUIRED_SUFFIXES.issubset(paths),
    }
    return {"passed": all(checks.values()), "checks": checks}


def runtime_audit_gate(
    value: Any, *, expected_binding: Mapping[str, Any]
) -> dict[str, Any]:
    """Validate the complete zero-progress save/reload and positive smoke audit."""

    data = _mapping(value)
    binding = _mapping(data.get("binding"))
    checks_payload = _mapping(data.get("checks"))
    actors = _mapping(data.get("actor_surfaces"))
    critics = _mapping(data.get("critic_surfaces"))
    optimizers = _mapping(data.get("optimizer_surfaces"))
    progress = _mapping(data.get("progress_surfaces"))
    positive = _mapping(data.get("positive_restore_smoke"))
    actor_digest = binding.get("actor_digest")
    critic_values = [critics.get(name) for name in contract.CRITIC_SURFACE_NAMES]
    optimizer_values = [
        optimizers.get(name) for name in contract.OPTIMIZER_SURFACE_NAMES
    ]
    checks = {
        "identity": data.get("schema_version") == contract.SCHEMA_VERSION
        and data.get("status") == "COMPLETE_H0_V12R9_ZERO_CHECKPOINT_AUDIT"
        and data.get("protocol_id") == contract.PROTOCOL_ID,
        "working_directory": _strict_equal(
            data.get("working_directory"),
            {
                "passed": True,
                "required_working_directory": contract.REQUIRED_WORKING_DIRECTORY,
                "observed_working_directory": "repository_root",
            },
        ),
        "binding": candidate_binding_valid(binding)
        and _strict_equal(binding, expected_binding),
        "required_checks": set(checks_payload) == set(contract.REQUIRED_AUDIT_CHECKS)
        and all(
            checks_payload[name] is True for name in contract.REQUIRED_AUDIT_CHECKS
        ),
        "live_config": _strict_equal(
            data.get("target_fixed_config"), contract.TARGET_FIXED_CONFIG
        )
        and _strict_equal(data.get("zero_reward_config"), contract.ZERO_REWARD_CONFIG),
        "actor_surfaces": set(actors) == set(contract.ACTOR_SURFACE_NAMES)
        and _sha(actor_digest)
        and all(
            _actor_surface_valid(actors[name], actor_digest=actor_digest)
            for name in contract.ACTOR_SURFACE_NAMES
        ),
        "fresh_critic": set(critics) == set(contract.CRITIC_SURFACE_NAMES)
        and all(_critic_surface_valid(item) for item in critic_values)
        and all(_strict_equal(item, critic_values[0]) for item in critic_values[1:])
        and data.get("candidate_source_non_actor_key_count") == 0
        and data.get("source_critic_restored") is False,
        "optimizer": set(optimizers) == set(contract.OPTIMIZER_SURFACE_NAMES)
        and all(_optimizer_snapshot_valid(item) for item in optimizer_values)
        and all(
            _strict_equal(item, optimizer_values[0]) for item in optimizer_values[1:]
        )
        and data.get("source_optimizer_restored") is False,
        "zero_progress": set(progress) == set(contract.PROGRESS_SURFACE_NAMES)
        and all(
            _progress_surface_valid(progress[name])
            for name in contract.PROGRESS_SURFACE_NAMES
        ),
        "checkpoint": checkpoint_tree_gate(data.get("checkpoint_tree"))["passed"]
        is True,
        "positive_restore": _strict_equal(
            positive.get("target_fixed_config"), contract.TARGET_FIXED_CONFIG
        )
        and _strict_equal(
            positive.get("reward_config"), contract.POSITIVE_RESTORE_REWARD_CONFIG
        )
        and positive.get("restore_from") == contract.CHECKPOINT_PATH.as_posix()
        and positive.get("restore_completed") is True
        and positive.get("actor_digest") == actor_digest
        and _strict_equal(positive.get("critic_surface"), critic_values[0])
        and _strict_equal(positive.get("optimizer_snapshot"), optimizer_values[0])
        and _progress_surface_valid(positive.get("progress"))
        and positive.get("algorithm_train_calls") == 0
        and positive.get("environment_samples") == 0,
        "closure": _strict_equal(
            data.get("closure_before_build"), data.get("closure_before_restore")
        )
        and _strict_equal(
            data.get("closure_before_build"), data.get("closure_after_restore")
        )
        and source_closure_gate(data.get("closure_before_build"))["passed"] is True,
        "activity": _one_int(data.get("actor_transplants"))
        and _zero_int(data.get("actor_updates"))
        and _zero_int(data.get("critic_updates"))
        and _zero_int(data.get("ppo_updates"))
        and _zero_int(data.get("environment_samples"))
        and _zero_int(data.get("algorithm_train_calls"))
        and data.get("training_executed") is False,
    }
    passed = all(checks.values())
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": (
            "PASS_H0_V12R9_ZERO_CHECKPOINT_AUDIT"
            if passed
            else "FAIL_H0_V12R9_ZERO_CHECKPOINT_AUDIT"
        ),
        "passed": passed,
        "checks": checks,
    }


def terminal_receipt_gate(value: Any) -> dict[str, Any]:
    data = _mapping(value)
    binding = _mapping(data.get("binding"))
    audit_gate = _mapping(data.get("audit_gate"))
    checkpoint_gate = checkpoint_tree_gate(data.get("checkpoint"))
    checks = {
        "identity": data.get("schema_version") == contract.SCHEMA_VERSION
        and data.get("status") == contract.PASS_STATUS
        and data.get("passed") is True
        and data.get("terminal") is True
        and data.get("error") is None
        and data.get("protocol_id") == contract.PROTOCOL_ID
        and data.get("pipeline_id") == contract.PIPELINE_ID,
        "binding": candidate_binding_valid(binding),
        "audit": audit_gate.get("passed") is True
        and audit_gate.get("status") == "PASS_H0_V12R9_ZERO_CHECKPOINT_AUDIT"
        and bool(_mapping(audit_gate.get("checks")))
        and all(item is True for item in _mapping(audit_gate.get("checks")).values()),
        "checkpoint": checkpoint_gate["passed"] is True,
        "zero": _one_int(data.get("actor_transplants"))
        and _zero_int(data.get("actor_updates"))
        and _zero_int(data.get("critic_updates"))
        and _zero_int(data.get("ppo_updates"))
        and _zero_int(data.get("environment_samples"))
        and data.get("training_executed") is False,
        "handoff": data.get("positive_live_config_restore_smoke_passed") is True
        and data.get("training_authorized") is False
        and data.get("training_command_published") is False
        and data.get("required_working_directory")
        == contract.REQUIRED_WORKING_DIRECTORY
        and data.get("next_stage") == contract.NEXT_STAGE_AFTER_ZERO_PASS,
    }
    return {"passed": all(checks.values()), "checks": checks}


__all__ = [
    "actor_manifest_gate",
    "artifact_record_valid",
    "canonical_json_sha256",
    "candidate_binding_valid",
    "checkpoint_tree_gate",
    "runtime_audit_gate",
    "source_closure_gate",
    "terminal_receipt_gate",
    "tree_record_valid",
    "upstream_terminal_gate",
]
