"""Freeze and execute the H0 V9 event-causal privileged relabel stage."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import secrets
import sys
import time
from pathlib import Path, PurePosixPath
from typing import Any, Mapping, Sequence


REPO_ROOT = Path(__file__).resolve().parents[1]
VALIDATION_ROOT = REPO_ROOT / "validation"
BASELINE_ROOT = REPO_ROOT / "Trajectory Generator" / "baseline_MLP"
for root in (REPO_ROOT, VALIDATION_ROOT, BASELINE_ROOT):
    if str(root) not in sys.path:
        sys.path.insert(0, str(root))

import h0_forensic_rollout as forensic  # noqa: E402
import h0_primary_split_v9_causal_teacher_contract as contract  # noqa: E402
import run_h0_primary_grf_split_v8r1p1_teacher_replay as source_collector  # noqa: E402
from h0_primary_split_v9_causal_teacher import (  # noqa: E402
    CAUSAL_INDICES,
    PRIVILEGED_INDICES,
    assert_causal_pair,
    from_replay_views,
)


class V9CausalTeacherExecutionError(RuntimeError):
    pass


def resolve_relative(path: str | PurePosixPath) -> Path:
    raw = path.as_posix() if isinstance(path, PurePosixPath) else str(path)
    pure = PurePosixPath(raw)
    if not raw or pure.is_absolute() or ".." in pure.parts or pure.as_posix() != raw:
        raise V9CausalTeacherExecutionError(
            f"non-canonical repository path: {raw!r}"
        )
    return REPO_ROOT.joinpath(*pure.parts)


PREFLIGHT = resolve_relative(contract.PREFLIGHT_PATH)
LOCK = resolve_relative(contract.LOCK_PATH)
RELABEL_ROOT = resolve_relative(contract.RELABEL_ROOT)
CLAIM = resolve_relative(contract.CLAIM_PATH)
LEDGER = resolve_relative(contract.LEDGER_PATH)
SOURCE_H0 = resolve_relative(contract.SOURCE_H0_MODULE_PATH)


def _mapping(path: str | Path) -> dict[str, Any]:
    value = forensic.strict_json_load(path)
    if not isinstance(value, Mapping):
        raise V9CausalTeacherExecutionError(f"expected JSON object: {path}")
    return dict(value)


def _sequence(path: str | Path) -> list[Any]:
    value = forensic.strict_json_load(path)
    if isinstance(value, (str, bytes)) or not isinstance(value, Sequence):
        raise V9CausalTeacherExecutionError(f"expected JSON array: {path}")
    return list(value)


def _record(path: str | Path) -> dict[str, Any]:
    return forensic.artifact_record(path, artifact_root=REPO_ROOT)


def _tree_record(path: str | Path) -> dict[str, Any]:
    root = Path(path).expanduser().resolve()
    files = sorted(item for item in root.rglob("*") if item.is_file())
    if not files:
        raise V9CausalTeacherExecutionError(f"empty artifact tree: {root}")
    digest = hashlib.sha256()
    rows: list[dict[str, Any]] = []
    for item in files:
        relative = item.relative_to(root).as_posix()
        sha = forensic.sha256_file(item)
        size = item.stat().st_size
        rows.append({"path": relative, "sha256": sha, "size_bytes": size})
        digest.update(relative.encode("utf-8"))
        digest.update(b"\0")
        digest.update(sha.encode("ascii"))
        digest.update(b"\0")
        digest.update(str(size).encode("ascii"))
        digest.update(b"\n")
    return {
        "path": root.relative_to(REPO_ROOT).as_posix(),
        "tree_sha256": digest.hexdigest(),
        "file_count": len(rows),
        "files": rows,
    }


def _source_records() -> dict[str, dict[str, Any]]:
    return {
        name: _record(resolve_relative(path))
        for name, path in contract.SOURCE_RELATIVE_PATHS.items()
    }


def _input_records() -> dict[str, dict[str, Any]]:
    return {
        name: _record(resolve_relative(path))
        for name, path in contract.INPUT_RELATIVE_PATHS.items()
    }


def _source_ledger() -> dict[str, Any]:
    ledger = _mapping(resolve_relative(contract.SOURCE_LEDGER_PATH))
    if (
        ledger.get("status") != source_collector.contract.PROTOCOL_PASS_STATUS
        or ledger.get("passed") is not True
        or ledger.get("completed_cases") != list(contract.CASE_IDS)
        or ledger.get("actor_updates") != 0
        or ledger.get("critic_updates") != 0
        or ledger.get("ppo_updates") != 0
        or ledger.get("protected_trials_opened") != []
    ):
        raise V9CausalTeacherExecutionError("V8R1P1 source ledger is not PASS")
    return ledger


def build_preflight() -> dict[str, Any]:
    if any(os.path.lexists(path) for path in (PREFLIGHT, LOCK, RELABEL_ROOT)):
        raise V9CausalTeacherExecutionError("V9 causal relabel path occupied")
    source_ledger = _source_ledger()
    for case_id in contract.CASE_IDS:
        source_collector.verify_case_receipt(case_id)
    checks = {
        "source_v8_pass": source_ledger.get("passed") is True,
        "six_source_cases": tuple(source_ledger["completed_cases"])
        == contract.CASE_IDS,
        "v26_event_contract": contract.EVENT_CONTRACT_ID
        == "binary_point_v25+heel_qualified_fsm_v2",
        "only_load_contact_privileged": contract.PRIVILEGED_INDICES == (10, 11),
        "source_h0_exists": SOURCE_H0.is_dir(),
        "protected_closed": True,
        "reserve_closed": True,
    }
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.PREFLIGHT_STATUS if all(checks.values()) else "FAIL",
        "passed": all(checks.values()),
        "protocol_id": contract.PROTOCOL_ID,
        "collector_id": contract.COLLECTOR_ID,
        "target_id": contract.TARGET_ID,
        "checks": checks,
        "source_ledger": _record(resolve_relative(contract.SOURCE_LEDGER_PATH)),
        "source_h0": _tree_record(SOURCE_H0),
        "sources": _source_records(),
        "inputs": _input_records(),
        "environment_step_calls": 0,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
        "next_stage": "FREEZE_V9_CAUSAL_RELABEL" if all(checks.values()) else "STOP",
    }


def prepare() -> dict[str, Any]:
    preflight = build_preflight()
    if not preflight["passed"]:
        raise V9CausalTeacherExecutionError("V9 causal relabel preflight failed")
    forensic.write_json_exclusive(PREFLIGHT, preflight)
    lock = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.LOCK_STATUS,
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "collector_id": contract.COLLECTOR_ID,
        "target_id": contract.TARGET_ID,
        "preflight": _record(PREFLIGHT),
        "sources": _source_records(),
        "inputs": _input_records(),
        "source_h0": _tree_record(SOURCE_H0),
        "case_ids": list(contract.CASE_IDS),
        "causal_indices": list(contract.CAUSAL_INDICES),
        "teacher_privileged_indices": list(contract.PRIVILEGED_INDICES),
        "authority": dict(contract.AUTHORITY),
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
        "next_stage": "EXECUTE_V9_CAUSAL_RELABEL_ONCE",
    }
    forensic.write_json_exclusive(LOCK, lock)
    return {"preflight": preflight, "lock": lock}


def verify_lock() -> dict[str, Any]:
    lock = _mapping(LOCK)
    expected = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.LOCK_STATUS,
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "collector_id": contract.COLLECTOR_ID,
        "target_id": contract.TARGET_ID,
        "preflight": _record(PREFLIGHT),
        "sources": _source_records(),
        "inputs": _input_records(),
        "source_h0": _tree_record(SOURCE_H0),
        "case_ids": list(contract.CASE_IDS),
        "causal_indices": list(contract.CAUSAL_INDICES),
        "teacher_privileged_indices": list(contract.PRIVILEGED_INDICES),
        "authority": dict(contract.AUTHORITY),
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
        "next_stage": "EXECUTE_V9_CAUSAL_RELABEL_ONCE",
    }
    if forensic.canonical_json_bytes(lock) != forensic.canonical_json_bytes(expected):
        raise V9CausalTeacherExecutionError("V9 causal relabel lock drifted")
    return lock


def _query(module: Any, observations: Any, *, np: Any, torch: Any) -> tuple[Any, Any]:
    from ray.rllib.core.columns import Columns

    tensor = torch.as_tensor(observations, dtype=torch.float32)
    with torch.no_grad():
        logits = module._policy_logits({Columns.OBS: tensor}).detach().cpu().numpy()
    logits = np.ascontiguousarray(logits, dtype=np.float32)
    if logits.shape != (len(observations), 4) or not np.all(np.isfinite(logits)):
        raise V9CausalTeacherExecutionError("teacher logits are malformed")
    means = np.ascontiguousarray(logits[:, :2], dtype=np.float32)
    std = np.ascontiguousarray(np.exp(logits[:, 2:]), dtype=np.float32)
    return means, std


def _execute_case(case_id: str, *, module: Any, np: Any, torch: Any) -> dict[str, Any]:
    case = contract.canonical_case(case_id)
    destination = resolve_relative(case["destination"])
    if os.path.lexists(destination):
        raise V9CausalTeacherExecutionError(f"case destination occupied: {case_id}")
    source_collector.verify_case_receipt(case_id)
    source_trace_path = resolve_relative(case["source_trace"])
    source_rows = _sequence(source_trace_path)
    if len(source_rows) != contract.EXPECTED_STEPS:
        raise V9CausalTeacherExecutionError(f"source trace drifted: {case_id}")

    students: list[Any] = []
    teachers: list[Any] = []
    for index, row in enumerate(source_rows):
        if not isinstance(row, Mapping) or row.get("step") != index + 1:
            raise V9CausalTeacherExecutionError(
                f"source row order drifted: {case_id}/{index + 1}"
            )
        student = np.ascontiguousarray(
            np.asarray(row.get("v25_observation"), dtype=np.float32)
        )
        reference = np.ascontiguousarray(
            np.asarray(row.get("baseline_teacher_observation"), dtype=np.float32)
        )
        teacher = from_replay_views(student, reference)
        students.append(student)
        teachers.append(teacher)
    student_array = np.ascontiguousarray(students, dtype=np.float32)
    teacher_array = np.ascontiguousarray(teachers, dtype=np.float32)
    means, std = _query(module, teacher_array, np=np, torch=torch)
    repeated_means, repeated_std = _query(module, teacher_array, np=np, torch=torch)
    base_means, _base_std = _query(module, student_array, np=np, torch=torch)

    causal_mismatches = 0
    query_mismatches = 0
    nonfinite = 0
    out_of_bounds = 0
    target_equals_base = 0
    rows: list[dict[str, Any]] = []
    for index in range(contract.EXPECTED_STEPS):
        student = student_array[index]
        teacher = teacher_array[index]
        assert_causal_pair(student, teacher)
        causal_exact = (
            student[list(CAUSAL_INDICES)].tobytes(order="C")
            == teacher[list(CAUSAL_INDICES)].tobytes(order="C")
        )
        query_exact = (
            means[index].tobytes(order="C")
            == repeated_means[index].tobytes(order="C")
            and std[index].tobytes(order="C")
            == repeated_std[index].tobytes(order="C")
        )
        finite = bool(
            np.all(np.isfinite(student))
            and np.all(np.isfinite(teacher))
            and np.all(np.isfinite(means[index]))
            and np.all(np.isfinite(std[index]))
        )
        bounded = bool(np.all(np.abs(means[index]) <= 1.0))
        same_base = means[index].tobytes(order="C") == base_means[index].tobytes(
            order="C"
        )
        causal_mismatches += int(not causal_exact)
        query_mismatches += int(not query_exact)
        nonfinite += int(not finite)
        out_of_bounds += int(not bounded)
        target_equals_base += int(same_base)
        rows.append(
            {
                "schema_version": contract.SCHEMA_VERSION,
                "protocol_id": contract.PROTOCOL_ID,
                "case_id": case_id,
                "step": index + 1,
                "v25_observation": student.tolist(),
                "causal_teacher_observation": teacher.tolist(),
                "queried_teacher_mean": means[index].tolist(),
                "teacher_std": std[index].tolist(),
                "base_h0_mean_on_student": base_means[index].tolist(),
                "teacher_privileged_values": teacher[
                    list(PRIVILEGED_INDICES)
                ].tolist(),
                "causal_columns_byte_exact": causal_exact,
                "teacher_query_byte_exact": query_exact,
                "target_equals_base": same_base,
            }
        )

    trace_path = forensic.write_json_exclusive(destination / "trace.json", rows)
    summary = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": "H0_PRIMARY_SPLIT_V9_CAUSAL_RELABEL_CASE_COMPLETE_UNGATED",
        "protocol_id": contract.PROTOCOL_ID,
        "collector_id": contract.COLLECTOR_ID,
        "target_id": contract.TARGET_ID,
        "event_contract_id": contract.EVENT_CONTRACT_ID,
        "case_id": case_id,
        "sample_count": len(rows),
        "observation_shape": [35],
        "action_shape": [2],
        "dtype": "float32",
        "teacher_privileged_indices": list(PRIVILEGED_INDICES),
        "causal_column_mismatch_count": causal_mismatches,
        "teacher_query_mismatch_count": query_mismatches,
        "nonfinite_count": nonfinite,
        "out_of_bounds_target_count": out_of_bounds,
        "target_equals_base_count": target_equals_base,
        "teacher_sigma": float(std[0, 0]),
        "source_trace": _record(source_trace_path),
        "trace": _record(trace_path),
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
    }
    summary_path = forensic.write_json_exclusive(destination / "summary.json", summary)
    gate = contract.case_gate(summary)
    gate_path = forensic.write_json_exclusive(destination / "gate.json", gate)
    receipt = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": gate["status"],
        "passed": gate["passed"],
        "protocol_id": contract.PROTOCOL_ID,
        "case_id": case_id,
        "trace": _record(trace_path),
        "summary": _record(summary_path),
        "gate": _record(gate_path),
        "source_receipt": _record(resolve_relative(case["source_receipt"])),
        "execution_claim": _record(CLAIM),
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
    }
    receipt_path = forensic.write_json_exclusive(destination / "receipt.json", receipt)
    if not gate["passed"]:
        raise V9CausalTeacherExecutionError(f"causal relabel gate failed: {case_id}")
    return {**receipt, "receipt": _record(receipt_path)}


def verify_case_receipt(case_id: str) -> dict[str, Any]:
    case = contract.canonical_case(case_id)
    destination = resolve_relative(case["destination"])
    receipt = _mapping(destination / "receipt.json")
    summary = _mapping(destination / "summary.json")
    gate = _mapping(destination / "gate.json")
    expected_gate = contract.case_gate(summary)
    if (
        receipt.get("status") != contract.CASE_PASS_STATUS
        or receipt.get("passed") is not True
        or receipt.get("protocol_id") != contract.PROTOCOL_ID
        or receipt.get("case_id") != case_id
        or receipt.get("trace") != _record(destination / "trace.json")
        or receipt.get("summary") != _record(destination / "summary.json")
        or receipt.get("gate") != _record(destination / "gate.json")
        or receipt.get("source_receipt")
        != _record(resolve_relative(case["source_receipt"]))
        or receipt.get("execution_claim") != _record(CLAIM)
        or forensic.canonical_json_bytes(gate)
        != forensic.canonical_json_bytes(expected_gate)
        or gate.get("passed") is not True
    ):
        raise V9CausalTeacherExecutionError(f"case receipt drifted: {case_id}")
    return receipt


def execute() -> dict[str, Any]:
    verify_lock()
    if os.path.lexists(RELABEL_ROOT):
        raise V9CausalTeacherExecutionError("V9 causal relabel already claimed")
    token_sha = hashlib.sha256(secrets.token_bytes(32)).hexdigest()
    claim = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": "H0_PRIMARY_SPLIT_V9_CAUSAL_RELABEL_EXECUTION_CLAIMED",
        "protocol_id": contract.PROTOCOL_ID,
        "collector_id": contract.COLLECTOR_ID,
        "target_id": contract.TARGET_ID,
        "execution_token_sha256": token_sha,
        "lock": _record(LOCK),
        "case_ids": list(contract.CASE_IDS),
        "retry_authorized": False,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
    }
    forensic.write_json_exclusive(CLAIM, claim)
    import numpy as np
    import torch
    from ray.rllib.core.rl_module.rl_module import RLModule

    module = RLModule.from_checkpoint(SOURCE_H0)
    module.eval()
    started = time.time()
    attempted: list[str] = []
    passed_cases: list[str] = []
    receipts: list[dict[str, Any]] = []
    error: str | None = None
    for case_id in contract.CASE_IDS:
        attempted.append(case_id)
        try:
            _execute_case(case_id, module=module, np=np, torch=torch)
            verify_case_receipt(case_id)
            passed_cases.append(case_id)
            receipts.append(
                {"case_id": case_id, "receipt": _record(
                    resolve_relative(contract.RELABEL_ROOT / case_id / "receipt.json")
                )}
            )
        except Exception as exc:
            error = f"{type(exc).__name__}: {exc}"
            break
    passed = passed_cases == list(contract.CASE_IDS)
    ledger = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": (
            contract.PROTOCOL_PASS_STATUS if passed else contract.PROTOCOL_FAIL_STATUS
        ),
        "passed": passed,
        "protocol_id": contract.PROTOCOL_ID,
        "collector_id": contract.COLLECTOR_ID,
        "target_id": contract.TARGET_ID,
        "started_unix_s": started,
        "completed_unix_s": time.time(),
        "attempted_cases": attempted,
        "completed_cases": passed_cases,
        "completed_receipts": receipts,
        "error": error,
        "candidate_created": False,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
        "retry_authorized": False,
        "next_stage": "FIT_V9_P0" if passed else "STOP_WITHOUT_RETRY",
    }
    forensic.write_json_exclusive(LEDGER, ledger)
    print(json.dumps(ledger, indent=2, sort_keys=True, allow_nan=False))
    if not passed:
        raise V9CausalTeacherExecutionError(error or contract.PROTOCOL_FAIL_STATUS)
    return ledger


def build_env_config(case: Mapping[str, Any]) -> dict[str, Any]:
    """Reuse only the already-gated V26 environment configuration builder."""

    return source_collector.build_env_config(case)


_sea_fallback_count = source_collector.engine._sea_fallback_count


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--prepare", action="store_true")
    mode.add_argument("--execute", action="store_true")
    mode.add_argument("--verify", action="store_true")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    if args.prepare:
        result = prepare()
    elif args.execute:
        result = execute()
    else:
        result = {
            "lock": verify_lock(),
            "cases": [verify_case_receipt(case_id) for case_id in contract.CASE_IDS],
            "ledger": _mapping(LEDGER),
        }
    if args.prepare or args.verify:
        print(json.dumps(result, indent=2, sort_keys=True, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
