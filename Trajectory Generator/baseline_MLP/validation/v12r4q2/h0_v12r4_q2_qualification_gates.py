"""Fail-closed Q2 gates with safe reuse of the byte-frozen Q1 comparators."""

from __future__ import annotations

import copy
import importlib.util
import sys
from collections.abc import Mapping
from pathlib import Path, PurePosixPath
from typing import Any

import h0_v12r4_q2_runtime_contract as contract


def _discover_repo_root(source: Path) -> Path:
    for candidate in source.resolve().parents:
        if (candidate / "AGENTS.md").is_file() and (candidate / "validation").is_dir():
            return candidate
    raise RuntimeError("repository root could not be discovered")


REPO_ROOT = _discover_repo_root(Path(__file__))
Q1_ROOT = REPO_ROOT / "Trajectory Generator" / "baseline_MLP" / "validation" / "v12p1q"
Q1_GATES_PATH = Q1_ROOT / "h0_v12r3_p1_qualification_gates.py"
if str(Q1_ROOT) not in sys.path:
    sys.path.insert(0, str(Q1_ROOT))


def _load_q1_gate_engine() -> Any:
    spec = importlib.util.spec_from_file_location(
        "_v12r4q2_frozen_q1_gate_engine", Q1_GATES_PATH
    )
    if spec is None or spec.loader is None:
        raise RuntimeError("frozen Q1 gate engine could not be loaded")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.contract = contract
    return module


_Q1_ENGINE = _load_q1_gate_engine()


def artifact_record_matches(value: Any, expected_path: str | PurePosixPath) -> bool:
    """Reuse the generic Q1 record-shape validator unchanged."""

    return bool(_Q1_ENGINE.artifact_record_matches(value, expected_path))


def _sha256(value: Any) -> bool:
    return (
        isinstance(value, str)
        and len(value) == 64
        and all(character in "0123456789abcdef" for character in value)
    )


def _tree(value: Any) -> dict[str, Any] | None:
    if not isinstance(value, Mapping):
        return None
    result = dict(value)
    if (
        result.get("path") != contract.R4_CANDIDATE_MODULE_PATH.as_posix()
        or not _sha256(result.get("tree_sha256"))
        or type(result.get("file_count")) is not int
        or result["file_count"] < 1
        or not isinstance(result.get("files"), list)
        or len(result["files"]) != result["file_count"]
    ):
        return None
    return copy.deepcopy(result)


def r4_prerequisite_gate(payload: Any) -> dict[str, Any]:
    """Validate the frozen five-record R4-to-Q2 handoff as a total function."""

    value = dict(payload) if isinstance(payload, Mapping) else {}
    candidate_module = _tree(value.get("candidate_module"))
    candidate_id = value.get("candidate_id")
    prerequisites = value.get("prerequisites")
    expected_names = [row["name"] for row in contract.prerequisite_requirements()]
    rows_ok = bool(
        isinstance(prerequisites, list)
        and len(prerequisites) == 5
        and all(isinstance(row, Mapping) for row in prerequisites)
        and [row.get("name") for row in prerequisites] == expected_names
    )
    record_paths_ok = rows_ok and all(
        artifact_record_matches(row.get("artifact"), requirement["path"])
        and row.get("status") == requirement["required_status"]
        and row.get("passed") is True
        for row, requirement in zip(
            prerequisites, contract.prerequisite_requirements(), strict=True
        )
    )
    binding_ok = False
    if candidate_module is not None:
        try:
            binding_ok = contract.validate_candidate_binding(
                candidate_id, candidate_module
            ) == {
                "candidate_id": candidate_id,
                "candidate_module": candidate_module,
            }
        except (TypeError, ValueError):
            pass
    exact_three = rows_ok and all(
        row.get("candidate_id") == candidate_id
        and row.get("candidate_module_tree_sha256")
        == (candidate_module or {}).get("tree_sha256")
        for row in prerequisites[2:]
    )
    first_two_deferred = rows_ok and all(
        row.get("candidate_id") is None
        and row.get("candidate_module_tree_sha256") is None
        and row.get("candidate_selection_rule") == contract.R4_CANDIDATE_SELECTION_RULE
        and row.get("candidate_module_path")
        == contract.R4_CANDIDATE_MODULE_PATH.as_posix()
        for row in prerequisites[:2]
    )
    checks = {
        "schema": value.get("schema_version") == contract.SCHEMA_VERSION,
        "status": value.get("status") == contract.PREREQUISITE_COMPLETE_STATUS,
        "protocol": value.get("protocol_id") == contract.PROTOCOL_ID,
        "five_ordered_records": rows_ok and record_paths_ok,
        "pre_fit_selection_rule_exact": first_two_deferred,
        "terminal_candidate_binding_exact": binding_ok and exact_three,
        "r4_terminal_pass": value.get("r4_terminal_passed") is True,
        "same_candidate_unique": value.get("candidate_identity_unique_count") == 1,
        "q2_unopened_before_binding": value.get("q2_protocol_preexisting") is False
        and value.get("q2_execution_lock_preexisting") is False
        and value.get("q2_noise_root_preexisting") is False
        and value.get("q2_run_root_preexisting") is False,
        "no_qualification_activity": value.get("qualification_rollouts_opened") == 0
        and value.get("actor_updates") == 0
        and value.get("critic_updates") == 0
        and value.get("ppo_updates") == 0
        and value.get("runtime_promoted") is False,
    }
    passed = all(checks.values())
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
        "checks": checks,
        "qualification_execution_authorized": False,
        "runtime_promoted": False,
    }


def _binding_failure(status: str) -> dict[str, Any]:
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": status,
        "passed": False,
        "protocol_id": contract.PROTOCOL_ID,
        "checks": {"candidate_bound_from_r4_terminal_pass": False},
    }


def common_rollout_gate(summary: Any, *, role: str, case_id: str) -> dict[str, Any]:
    try:
        contract.current_candidate_binding()
    except contract.Q2CandidateBindingError:
        return _binding_failure(contract.ROLLOUT_FAIL_STATUS)
    return dict(_Q1_ENGINE.common_rollout_gate(summary, role=role, case_id=case_id))


def condition_matched_gate(
    baseline: Any, candidate: Any, *, case_id: str
) -> dict[str, Any]:
    try:
        contract.current_candidate_binding()
    except contract.Q2CandidateBindingError:
        return _binding_failure(contract.PAIR_FAIL_STATUS)
    return dict(_Q1_ENGINE.condition_matched_gate(baseline, candidate, case_id=case_id))


def aggregate_qualification_gate(summary: Any) -> dict[str, Any]:
    try:
        contract.current_candidate_binding()
    except contract.Q2CandidateBindingError:
        return _binding_failure(contract.AGGREGATE_FAIL_STATUS)
    return dict(_Q1_ENGINE.aggregate_qualification_gate(summary))


__all__ = [
    "aggregate_qualification_gate",
    "artifact_record_matches",
    "common_rollout_gate",
    "condition_matched_gate",
    "r4_prerequisite_gate",
]
