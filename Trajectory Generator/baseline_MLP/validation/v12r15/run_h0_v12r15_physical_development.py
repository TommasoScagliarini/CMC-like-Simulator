"""Prepare, execute, or verify the one-shot V12R15 P3 physical protocol.

``--describe`` and ``--preflight`` are read-only.  ``--prepare-lock`` is
available only after the canonical P2 gate/receipt PASS and publishes the P3
freeze plus full source-closure lock without importing the environment.
``--execute`` lazily imports the frozen V12R6 physical runtime and stops at the
first failed case.  Existing P3 output is never resumed or retried.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import os
import sys
from pathlib import Path, PurePosixPath
from typing import Any, Callable, Mapping, Sequence


LOCAL_ROOT = Path(__file__).resolve().parent
if str(LOCAL_ROOT) not in sys.path:
    sys.path.insert(0, str(LOCAL_ROOT))

import h0_v12r15_physical_development as contract  # noqa: E402


PREPARE_ACKNOWLEDGEMENT = "V12R15_P3_FREEZE_AFTER_P2_PASS"
EXECUTE_ACKNOWLEDGEMENT = "V12R15_P3_ONE_SHOT_NO_RETRY"

Activity = dict[str, int]
CaseRunner = Callable[..., Mapping[str, Any]]


def _record(path: Path, paths: contract.DevelopmentPaths) -> dict[str, Any]:
    return contract.artifact_record(path, artifact_root=paths.artifact_root)


def _tree(path: Path, paths: contract.DevelopmentPaths) -> dict[str, Any]:
    return contract.tree_record(path, artifact_root=paths.artifact_root)


def _write(
    path: Path,
    payload: Mapping[str, Any] | Sequence[Any],
    paths: contract.DevelopmentPaths,
) -> dict[str, Any]:
    contract.write_json_exclusive(path, payload)
    return _record(path, paths)


def _mapping(path: Path) -> dict[str, Any]:
    value = contract.strict_json_load(path)
    if not isinstance(value, dict):
        raise contract.V12R15PhysicalDevelopmentError(f"expected JSON mapping: {path}")
    return value


def _sequence(path: Path) -> list[Any]:
    value = contract.strict_json_load(path)
    if not isinstance(value, list):
        raise contract.V12R15PhysicalDevelopmentError(f"expected JSON array: {path}")
    return value


def _source_record(relative: PurePosixPath) -> dict[str, Any]:
    raw = relative.as_posix()
    if relative.is_absolute() or ".." in relative.parts or raw != str(relative):
        raise contract.V12R15PhysicalDevelopmentError(
            f"non-canonical source path: {raw!r}"
        )
    return contract.artifact_record(
        contract.REPO_ROOT.joinpath(*relative.parts),
        artifact_root=contract.REPO_ROOT,
    )


def _r10_source_closure() -> tuple[dict[str, Any], dict[str, Any]]:
    lock_path = contract.REPO_ROOT.joinpath(*contract.R10_EXECUTION_LOCK.parts)
    lock_record = contract.artifact_record(lock_path, artifact_root=contract.REPO_ROOT)
    if lock_record.get("sha256") != contract.R10_EXECUTION_LOCK_SHA256:
        raise contract.V12R15PhysicalDevelopmentError(
            "R10 source-closure anchor hash drifted"
        )
    lock = _mapping(lock_path)
    closure = lock.get("production_source_closure")
    if (
        not isinstance(closure, Mapping)
        or len(closure) != contract.R10_SOURCE_CLOSURE_COUNT
    ):
        raise contract.V12R15PhysicalDevelopmentError(
            "R10 transitive production source closure is malformed"
        )
    result: dict[str, Any] = {}
    for relative, expected in sorted(closure.items()):
        if not isinstance(relative, str) or not isinstance(expected, Mapping):
            raise contract.V12R15PhysicalDevelopmentError(
                "R10 source closure entry is malformed"
            )
        observed = _source_record(PurePosixPath(relative))
        if observed != dict(expected):
            raise contract.V12R15PhysicalDevelopmentError(
                f"R10 source closure drifted: {relative}"
            )
        result[relative] = copy.deepcopy(dict(expected))
    return lock_record, result


def build_source_closure() -> dict[str, Any]:
    """Build the complete inherited runtime plus V12R15 source closure."""

    _anchor, closure = _r10_source_closure()
    for relative in (
        *contract.OWN_PRODUCTION_SOURCES,
        *contract.GOVERNANCE_SOURCE_PATHS,
        *contract.LINEAGE_SOURCE_PATHS,
    ):
        closure[relative.as_posix()] = _source_record(relative)
    required = {
        relative.as_posix() for relative in contract.REQUIRED_TRANSITIVE_RUNTIME_SOURCES
    }
    if not required.issubset(closure):
        raise contract.V12R15PhysicalDevelopmentError(
            "constructed source closure lacks a required runtime dependency"
        )
    return dict(sorted(closure.items()))


def prepare_lock(
    paths: contract.DevelopmentPaths = contract.PRODUCTION_PATHS,
) -> dict[str, Any]:
    """Publish the P3 freeze/lock only after canonical P2 is terminal PASS."""

    if paths.artifact_root.resolve() != contract.REPO_ROOT.resolve():
        raise contract.V12R15PhysicalDevelopmentError(
            "production governance can only be prepared in the repository root"
        )
    if os.path.lexists(paths.protocol_freeze) or os.path.lexists(paths.execution_lock):
        raise contract.V12R15PhysicalDevelopmentError(
            "P3 freeze/lock path occupied; governance publication is no-clobber"
        )
    p2 = contract.verify_p2(paths)
    anchor, _inherited = _r10_source_closure()
    closure = build_source_closure()
    closure_sha256 = hashlib.sha256(contract.canonical_json_bytes(closure)).hexdigest()
    freeze = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.FREEZE_PASS_STATUS,
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "development_id": contract.DEVELOPMENT_ID,
        "design": contract.protocol_design(),
        "design_sha256": contract.protocol_design_sha256(),
        "p2_receipt": p2["receipt"],
        "p2_gate": p2["gate"],
        "p2_summary": p2["summary"],
        "p2_candidate": p2["candidate_module"],
        "p2_candidate_id": p2["candidate_id"],
        "p0_eligible": False,
        "r10_source_closure_anchor": anchor,
        "production_source_file_count": len(closure),
        "production_source_closure_sha256": closure_sha256,
        "one_shot": True,
        "retry_authorized": False,
        "resume_authorized": False,
        "environment_calls": 0,
        "policy_queries": 0,
    }
    freeze_record = _write(paths.protocol_freeze, freeze, paths)
    lock = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.LOCK_PASS_STATUS,
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "development_id": contract.DEVELOPMENT_ID,
        "design_sha256": contract.protocol_design_sha256(),
        "protocol_freeze": freeze_record,
        "p2_receipt": p2["receipt"],
        "p2_gate": p2["gate"],
        "p2_summary": p2["summary"],
        "p2_candidate": p2["candidate_module"],
        "p2_candidate_id": p2["candidate_id"],
        "p0_eligible": False,
        "r10_source_closure_anchor": anchor,
        "production_source_closure": closure,
        "production_source_file_count": len(closure),
        "production_source_closure_sha256": closure_sha256,
        "one_shot": True,
        "retry_authorized": False,
        "resume_authorized": False,
        "rescue_authorized": False,
        "environment_calls": 0,
        "policy_queries": 0,
    }
    lock_record = _write(paths.execution_lock, lock, paths)
    verified = contract.verify_execution_lock(paths, p2=p2)
    return {
        "passed": True,
        "status": "PASS_H0_V12R15_P3_GOVERNANCE_PREPARED",
        "protocol_freeze": freeze_record,
        "execution_lock": lock_record,
        "production_source_file_count": len(closure),
        "production_source_closure_sha256": closure_sha256,
        "verified": verified,
        "environment_calls": 0,
        "policy_queries": 0,
    }


def _new_activity() -> Activity:
    return {
        "environment_reset_calls": 0,
        "environment_step_calls": 0,
        "raw_sensor_sample_count": 0,
        "actor_query_count": 0,
        **{name: 0 for name in contract.PURE_POLICY_COUNTER_FIELDS},
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
    }


def _increment(activity: Activity, name: str, amount: int = 1) -> None:
    if name not in activity or type(amount) is not int or amount < 0:
        raise contract.V12R15PhysicalDevelopmentError(
            f"invalid P3 activity update: {name}={amount!r}"
        )
    activity[name] += amount


def _load_physical_runtime() -> Any:
    """Lazy import boundary: describe/preflight/tests do not load the env stack."""

    physical_root = (
        contract.REPO_ROOT
        / "Trajectory Generator"
        / "baseline_MLP"
        / "validation"
        / "v12r6"
    )
    for root in (
        contract.REPO_ROOT / "validation",
        physical_root,
    ):
        if str(root) not in sys.path:
            sys.path.insert(0, str(root))
    try:
        import h0_v12r6_physical_development as physical
    except Exception as exc:  # pragma: no cover - production dependency boundary.
        raise contract.V12R15PhysicalDevelopmentError(
            "frozen V12R6 physical runtime is unavailable"
        ) from exc
    return physical


def _physical_config(physical: Any, paths: contract.DevelopmentPaths) -> Any:
    return physical.PhysicalDevelopmentConfig(
        protocol_id=contract.PROTOCOL_ID,
        start_status=contract.START_STATUS,
        partial_status=contract.PARTIAL_STATUS,
        complete_status=contract.COMPLETE_STATUS,
        schema_version=contract.SCHEMA_VERSION,
        artifact_root=paths.artifact_root,
        progress_label="V12R15 P3 pure development",
        progress_every=25,
    )


def _case_inventory(
    root: Path, paths: contract.DevelopmentPaths, *, require_gate: bool
) -> dict[str, Any]:
    required = {
        "run_start": root / "run_start.json",
        "trace": root / "trace.json",
        "partial_summary": root / "partial_summary.json",
        "summary": root / "summary.json",
    }
    if require_gate:
        required["gate"] = root / "gate.json"
    result = {name: _record(path, paths) for name, path in required.items()}
    steps_root = root / "steps"
    step_paths = sorted(steps_root.glob("*.json")) if steps_root.is_dir() else []
    if [path.name for path in step_paths] != [
        f"{index:06d}.json" for index in range(1, len(step_paths) + 1)
    ]:
        raise contract.V12R15PhysicalDevelopmentError(
            f"case step journal is not contiguous: {root}"
        )
    result["steps"] = [_record(path, paths) for path in step_paths]
    return result


def _load_step_prefix(root: Path) -> list[dict[str, Any]]:
    steps_root = root / "steps"
    paths = sorted(steps_root.glob("*.json")) if steps_root.is_dir() else []
    expected = [f"{index:06d}.json" for index in range(1, len(paths) + 1)]
    if [path.name for path in paths] != expected:
        raise contract.V12R15PhysicalDevelopmentError(
            "durable physical step prefix is non-contiguous"
        )
    rows: list[dict[str, Any]] = []
    for index, path in enumerate(paths, start=1):
        row = _mapping(path)
        if row.get("step") != index:
            raise contract.V12R15PhysicalDevelopmentError(
                f"durable step identity drifted: {path}"
            )
        rows.append(row)
    return rows


def _early_summary(
    *,
    case: Mapping[str, Any],
    rows: Sequence[Mapping[str, Any]],
    error: BaseException,
    binding: Mapping[str, Any],
) -> dict[str, Any]:
    penetrations = [
        float(row["grf_penetration_m"])
        for row in rows
        if isinstance(row, Mapping)
        and isinstance(row.get("grf_penetration_m"), (int, float))
        and not isinstance(row.get("grf_penetration_m"), bool)
        and float("-inf") < float(row["grf_penetration_m"]) < float("inf")
    ]
    last = rows[-1] if rows else {}
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.EARLY_COMPLETE_STATUS,
        "protocol_id": contract.PROTOCOL_ID,
        "development_id": contract.DEVELOPMENT_ID,
        "case": copy.deepcopy(dict(case)),
        "steps": len(rows),
        "control_window_count": len(rows) * 10,
        "raw_sensor_sample_count": sum(
            int(row.get("raw_sensor_sample_count", 0))
            for row in rows
            if isinstance(row, Mapping)
            and type(row.get("raw_sensor_sample_count")) is int
        ),
        "phase_valid_cycle_count": 0,
        "grf_penetration_max_m": max(penetrations) if penetrations else 0.0,
        "end_reason": (last.get("end_reason") or "physical_runtime_exception"),
        "terminated": bool(last.get("terminated", False)),
        "truncated": bool(last.get("truncated", False)),
        **{name: 0 for name in contract.ZERO_INVALID_SUMMARY_FIELDS},
        "sea_reserve_gate_passed": False,
        "binary_phase_event_gate": {
            "passed": False,
            "sample_count": len(rows) * 10,
            "duplicate_event_count": 0,
            "out_of_order_event_count": 0,
            "left_non_v26_source_count": 0,
            "fallback_count": 0,
            "hard_invalid_count": 0,
        },
        "event_contract_id": contract.EVENT_CONTRACT_ID,
        "target_contract_id": contract.TARGET_CONTRACT_ID,
        "detector_or_fsm_modified": False,
        "morphology_weight": contract.MORPHOLOGY_WEIGHT,
        "teacher_enabled": False,
        "blending_enabled": False,
        "safety_latch_enabled": False,
        **{name: 0 for name in contract.PURE_POLICY_COUNTER_FIELDS},
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "q2_paths_opened": [],
        "q3_paths_opened": [],
        "p2_candidate_only": True,
        "p0_used": False,
        "p2_binding": copy.deepcopy(dict(binding)),
        "pure_policy_trace_audit": {
            "passed": False,
            "row_count": len(rows),
        },
        "error": {"type": type(error).__name__, "message": str(error)},
    }


def _close_early_case(
    *,
    root: Path,
    case: Mapping[str, Any],
    error: BaseException,
    binding: Mapping[str, Any],
    paths: contract.DevelopmentPaths,
) -> dict[str, Any]:
    rows = _load_step_prefix(root)
    trace_path = root / "trace.json"
    partial_path = root / "partial_summary.json"
    summary_path = root / "summary.json"
    early = _early_summary(case=case, rows=rows, error=error, binding=binding)
    if trace_path.is_file():
        persisted_trace = _sequence(trace_path)
        if contract.canonical_json_bytes(
            persisted_trace
        ) != contract.canonical_json_bytes(list(rows)):
            raise contract.V12R15PhysicalDevelopmentError(
                "published trace disagrees with the durable step prefix"
            ) from error
    elif os.path.lexists(trace_path):
        raise contract.V12R15PhysicalDevelopmentError(
            "unsafe trace artifact after physical failure"
        ) from error
    else:
        _write(trace_path, list(rows), paths)
    if partial_path.is_file():
        _mapping(partial_path)
    elif os.path.lexists(partial_path):
        raise contract.V12R15PhysicalDevelopmentError(
            "unsafe partial summary after physical failure"
        ) from error
    else:
        _write(
            partial_path,
            {
                "schema_version": contract.SCHEMA_VERSION,
                "status": contract.PARTIAL_STATUS,
                "protocol_id": contract.PROTOCOL_ID,
                "stage_id": f"development__{case['case_id']}",
                "steps": len(rows),
                "gate_evaluated": False,
            },
            paths,
        )
    if summary_path.is_file():
        summary = _mapping(summary_path)
    elif os.path.lexists(summary_path):
        raise contract.V12R15PhysicalDevelopmentError(
            "unsafe summary artifact after physical failure"
        ) from error
    else:
        summary = early
        _write(summary_path, summary, paths)
    gate = contract.development_gate(summary, case_id=str(case["case_id"]), trace=rows)
    gate["checks"] = {
        **dict(gate.get("checks", {})),
        "physical_runtime_completed_without_exception": False,
    }
    gate.update(
        {
            "status": contract.CASE_FAIL_STATUS,
            "passed": False,
            "terminal": True,
            "next_stage": "STOP_TERMINAL_NO_RETRY",
            "runtime_error": {
                "type": type(error).__name__,
                "message": str(error),
            },
        }
    )
    gate["inputs"] = _case_inventory(root, paths, require_gate=False)
    _write(root / "gate.json", gate, paths)
    inventory = _case_inventory(root, paths, require_gate=True)
    receipt = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.CASE_FAIL_STATUS,
        "passed": False,
        "terminal": True,
        "protocol_id": contract.PROTOCOL_ID,
        "development_id": contract.DEVELOPMENT_ID,
        "case_id": case["case_id"],
        "p2_binding": copy.deepcopy(dict(binding)),
        "artifacts": inventory,
        "error": {"type": type(error).__name__, "message": str(error)},
        "retry_authorized": False,
        "resume_authorized": False,
    }
    _write(root / "receipt.json", receipt, paths)
    return receipt


def _run_case(
    *,
    case_id: str,
    paths: contract.DevelopmentPaths,
    binding: Mapping[str, Any],
    activity: Activity,
    case_runner: CaseRunner | None,
) -> dict[str, Any]:
    case = contract.canonical_case(case_id)
    destination = paths.case_root(case_id)
    physical = _load_physical_runtime() if case_runner is None else None
    runner = physical.run_case if case_runner is None else case_runner
    physical_config = (
        _physical_config(physical, paths)
        if physical is not None
        else {
            "schema_version": contract.SCHEMA_VERSION,
            "protocol_id": contract.PROTOCOL_ID,
        }
    )
    before = contract.verify_execution_lock(paths, p2=binding["p2"])
    try:
        result = runner(
            config=physical_config,
            case=case,
            destination=destination,
            module_path=paths.p2_candidate,
            activity_callback=lambda name, amount: _increment(activity, name, amount),
            start_metadata={
                "development_id": contract.DEVELOPMENT_ID,
                "pipeline_claim": _record(paths.pipeline_claim, paths),
                "candidate_freeze": _record(paths.candidate_freeze, paths),
                "p2_candidate_only": True,
                "p0_used": False,
                "production_source_closure_sha256": before[
                    "production_source_closure_sha256"
                ],
            },
            summary_metadata={
                "development_id": contract.DEVELOPMENT_ID,
                "candidate_id": binding["p2"]["candidate_id"],
                "candidate_module": binding["p2"]["candidate_module"],
                "p2_receipt": binding["p2"]["receipt"],
                "p2_gate": binding["p2"]["gate"],
                "p2_candidate_only": True,
                "p0_used": False,
                "target_contract_id": contract.TARGET_CONTRACT_ID,
                "production_source_closure_sha256": before[
                    "production_source_closure_sha256"
                ],
            },
        )
    except BaseException as exc:
        if destination.is_dir() and (destination / "run_start.json").is_file():
            _close_early_case(
                root=destination,
                case=case,
                error=exc,
                binding=binding["p2"],
                paths=paths,
            )
            raise contract.V12R15PhysicalDevelopmentError(
                f"P3 case stopped terminally: {case_id}; receipt={_record(destination / 'receipt.json', paths)['sha256']}"
            ) from exc
        raise
    if not isinstance(result, Mapping):
        raise contract.V12R15PhysicalDevelopmentError(
            f"physical runtime returned malformed result: {case_id}"
        )
    after = contract.verify_execution_lock(paths, p2=binding["p2"])
    if before != after:
        raise contract.V12R15PhysicalDevelopmentError(
            "locked sources or P2 changed during physical rollout"
        )
    trace = result.get("trace")
    summary = result.get("summary")
    if not isinstance(trace, Sequence) or isinstance(trace, (str, bytes)):
        raise contract.V12R15PhysicalDevelopmentError("physical trace is malformed")
    if not isinstance(summary, Mapping):
        raise contract.V12R15PhysicalDevelopmentError("physical summary is malformed")
    gate = contract.development_gate(summary, case_id=case_id, trace=trace)
    gate["inputs"] = _case_inventory(destination, paths, require_gate=False)
    _write(destination / "gate.json", gate, paths)
    inventory = _case_inventory(destination, paths, require_gate=True)
    receipt = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.CASE_PASS_STATUS
        if gate["passed"]
        else contract.CASE_FAIL_STATUS,
        "passed": gate["passed"],
        "terminal": gate["passed"] is not True,
        "protocol_id": contract.PROTOCOL_ID,
        "development_id": contract.DEVELOPMENT_ID,
        "case_id": case_id,
        "candidate_id": binding["p2"]["candidate_id"],
        "candidate_module": binding["p2"]["candidate_module"],
        "p2_receipt": binding["p2"]["receipt"],
        "p2_gate": binding["p2"]["gate"],
        "source_closure_before": before,
        "source_closure_after": after,
        "artifacts": inventory,
        "trace_audit": gate["trace_audit"],
        "retry_authorized": False,
        "resume_authorized": False,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
    }
    _write(destination / "receipt.json", receipt, paths)
    if receipt["passed"] is not True:
        raise contract.V12R15PhysicalDevelopmentError(
            f"P3 physical gate failed terminally: {case_id}"
        )
    activity["actor_query_count"] += contract.EXPECTED_STEPS
    return receipt


def _finalize(
    *,
    paths: contract.DevelopmentPaths,
    binding: Mapping[str, Any],
    activity: Activity,
) -> dict[str, Any]:
    case_bindings: list[dict[str, Any]] = []
    candidate_trees: set[str] = set()
    for case_id in contract.DEVELOPMENT_CASE_IDS:
        root = paths.case_root(case_id)
        receipt = _mapping(root / "receipt.json")
        gate = _mapping(root / "gate.json")
        if (
            receipt.get("passed") is not True
            or gate.get("passed") is not True
            or receipt.get("case_id") != case_id
            or receipt.get("candidate_module") != binding["p2"]["candidate_module"]
            or receipt.get("artifacts")
            != _case_inventory(root, paths, require_gate=True)
        ):
            raise contract.V12R15PhysicalDevelopmentError(
                f"P3 case binding drifted before aggregate: {case_id}"
            )
        candidate_trees.add(str(receipt["candidate_module"]["tree_sha256"]))
        case_bindings.append(
            {
                "case_id": case_id,
                "passed": True,
                "receipt": _record(root / "receipt.json", paths),
                "gate": _record(root / "gate.json", paths),
                "summary": _record(root / "summary.json", paths),
                "trace": _record(root / "trace.json", paths),
            }
        )
    summary = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": "COMPLETE_H0_V12R15_P3_PURE_PHYSICAL_DEVELOPMENT",
        "protocol_id": contract.PROTOCOL_ID,
        "development_id": contract.DEVELOPMENT_ID,
        "candidate_id": binding["p2"]["candidate_id"],
        "candidate_module": binding["p2"]["candidate_module"],
        "p2_receipt": binding["p2"]["receipt"],
        "p2_gate": binding["p2"]["gate"],
        "p0_used": False,
        "case_bindings": case_bindings,
        "candidate_tree_unique_count": len(candidate_trees),
        "development_count": len(case_bindings),
        **activity,
        "retry_authorized": False,
        "resume_authorized": False,
        "rescue_authorized": False,
        "q3_paths_opened": [],
        "checkpoint_zero_created": False,
        "positive_morphology_enabled": False,
    }
    _write(paths.aggregate_summary, summary, paths)
    gate = contract.aggregate_gate(summary)
    gate["inputs"] = {
        "aggregate_summary": _record(paths.aggregate_summary, paths),
        "case_receipts": [item["receipt"] for item in case_bindings],
    }
    _write(paths.aggregate_gate, gate, paths)
    if gate.get("passed") is not True:
        raise contract.V12R15PhysicalDevelopmentError(
            "P3 aggregate physical gate failed"
        )
    receipt = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.AGGREGATE_PASS_STATUS,
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "development_id": contract.DEVELOPMENT_ID,
        "candidate_id": binding["p2"]["candidate_id"],
        "candidate_module": binding["p2"]["candidate_module"],
        "candidate_freeze": _record(paths.candidate_freeze, paths),
        "case_bindings": case_bindings,
        "summary": _record(paths.aggregate_summary, paths),
        "gate": _record(paths.aggregate_gate, paths),
        "next_stage": "RUN_SEPARATE_Q3_PROTOCOL",
        "retry_authorized": False,
        "resume_authorized": False,
        "q3_paths_opened": [],
        "checkpoint_zero_created": False,
        "positive_morphology_enabled": False,
    }
    _write(paths.final_receipt, receipt, paths)
    return receipt


def _terminal_payload(
    *,
    passed: bool,
    paths: contract.DevelopmentPaths,
    binding: Mapping[str, Any],
    completed: Sequence[Mapping[str, Any]],
    attempted_case: str | None,
    activity: Mapping[str, int],
    error: BaseException | None,
) -> dict[str, Any]:
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": (
            contract.PIPELINE_PASS_STATUS if passed else contract.PIPELINE_FAIL_STATUS
        ),
        "passed": passed,
        "terminal": True,
        "protocol_id": contract.PROTOCOL_ID,
        "development_id": contract.DEVELOPMENT_ID,
        "design_sha256": contract.protocol_design_sha256(),
        "case_order": list(contract.DEVELOPMENT_CASE_IDS),
        "first_and_only_discriminator": contract.DISCRIMINATOR_CASE_ID,
        "completed_cases": list(completed),
        "completed_case_count": len(completed),
        "attempted_case": attempted_case,
        "attempted_case_receipt": (
            _record(paths.case_root(attempted_case) / "receipt.json", paths)
            if attempted_case is not None
            and (paths.case_root(attempted_case) / "receipt.json").is_file()
            else None
        ),
        "activity_totals": dict(activity),
        "candidate_id": binding["p2"]["candidate_id"],
        "candidate_module": binding["p2"]["candidate_module"],
        "p2_receipt": binding["p2"]["receipt"],
        "p2_gate": binding["p2"]["gate"],
        "p0_used": False,
        "protocol_freeze": _record(paths.protocol_freeze, paths),
        "execution_lock": _record(paths.execution_lock, paths),
        "pipeline_claim": _record(paths.pipeline_claim, paths),
        "candidate_freeze": _record(paths.candidate_freeze, paths),
        "final_receipt": (
            _record(paths.final_receipt, paths)
            if paths.final_receipt.is_file()
            else None
        ),
        "retry_authorized": False,
        "resume_authorized": False,
        "rescue_authorized": False,
        "q3_paths_opened": [],
        "checkpoint_zero_created": False,
        "positive_morphology_enabled": False,
        "error": (
            None
            if error is None
            else {"type": type(error).__name__, "message": str(error)}
        ),
        "next_stage": (
            "RUN_SEPARATE_Q3_PROTOCOL" if passed else "STOP_TERMINAL_NO_RETRY"
        ),
    }


def verify_terminal_ledger(
    paths: contract.DevelopmentPaths = contract.PRODUCTION_PATHS,
) -> dict[str, Any]:
    ledger = _mapping(paths.pipeline_ledger)
    passed = ledger.get("passed")
    if type(passed) is not bool:
        raise contract.V12R15PhysicalDevelopmentError(
            "P3 terminal ledger passed field is malformed"
        )
    expected_status = (
        contract.PIPELINE_PASS_STATUS if passed else contract.PIPELINE_FAIL_STATUS
    )
    if (
        ledger.get("status") != expected_status
        or ledger.get("terminal") is not True
        or ledger.get("protocol_id") != contract.PROTOCOL_ID
        or ledger.get("design_sha256") != contract.protocol_design_sha256()
        or ledger.get("retry_authorized") is not False
        or ledger.get("resume_authorized") is not False
        or ledger.get("p0_used") is not False
    ):
        raise contract.V12R15PhysicalDevelopmentError(
            "P3 terminal ledger identity drifted"
        )
    completed = ledger.get("completed_cases")
    if not isinstance(completed, list) or ledger.get("completed_case_count") != len(
        completed
    ):
        raise contract.V12R15PhysicalDevelopmentError(
            "P3 terminal completed prefix is malformed"
        )
    for index, item in enumerate(completed):
        if (
            not isinstance(item, Mapping)
            or item.get("case_id") != contract.DEVELOPMENT_CASE_IDS[index]
            or item.get("receipt")
            != _record(
                paths.case_root(str(item.get("case_id"))) / "receipt.json", paths
            )
        ):
            raise contract.V12R15PhysicalDevelopmentError(
                "P3 terminal case binding drifted"
            )
    if passed:
        if (
            len(completed) != len(contract.DEVELOPMENT_CASE_IDS)
            or ledger.get("attempted_case") is not None
            or ledger.get("error") is not None
            or ledger.get("final_receipt") != _record(paths.final_receipt, paths)
        ):
            raise contract.V12R15PhysicalDevelopmentError(
                "P3 terminal PASS closure drifted"
            )
    else:
        attempted_case = ledger.get("attempted_case")
        if not isinstance(ledger.get("error"), Mapping):
            raise contract.V12R15PhysicalDevelopmentError(
                "P3 terminal FAIL lacks error evidence"
            )
        if attempted_case is not None:
            if attempted_case not in contract.DEVELOPMENT_CASE_IDS:
                raise contract.V12R15PhysicalDevelopmentError(
                    "P3 terminal attempted case is unknown"
                )
            receipt_path = paths.case_root(attempted_case) / "receipt.json"
            expected_receipt = (
                _record(receipt_path, paths) if receipt_path.is_file() else None
            )
            if ledger.get("attempted_case_receipt") != expected_receipt:
                raise contract.V12R15PhysicalDevelopmentError(
                    "P3 attempted-case failure receipt binding drifted"
                )
    return ledger


def execute(
    paths: contract.DevelopmentPaths = contract.PRODUCTION_PATHS,
    *,
    case_runner: CaseRunner | None = None,
) -> dict[str, Any]:
    """Execute P3 exactly once; an existing ledger is verification-only."""

    if os.path.lexists(paths.pipeline_ledger):
        return verify_terminal_ledger(paths)
    ready = contract.preflight(paths)
    try:
        os.mkdir(paths.p3_root, 0o700)
        os.mkdir(paths.cases_root, 0o700)
    except OSError as exc:
        raise contract.V12R15PhysicalDevelopmentError(
            "cannot exclusively claim P3 output root"
        ) from exc
    binding = ready["locked_inputs"]
    claim = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": "CLAIMED_H0_V12R15_P3_PURE_PIPELINE_ONCE",
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "development_id": contract.DEVELOPMENT_ID,
        "design_sha256": contract.protocol_design_sha256(),
        "case_order": list(contract.DEVELOPMENT_CASE_IDS),
        "first_and_only_discriminator": contract.DISCRIMINATOR_CASE_ID,
        "p2": binding["p2"],
        "protocol_freeze": binding["freeze"],
        "execution_lock": binding["execution_lock"],
        "one_shot": True,
        "retry_authorized": False,
        "resume_authorized": False,
        "rescue_authorized": False,
    }
    _write(paths.pipeline_claim, claim, paths)
    candidate_freeze = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": "PASS_H0_V12R15_P3_P2_CANDIDATE_FREEZE",
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "development_id": contract.DEVELOPMENT_ID,
        "candidate_id": binding["p2"]["candidate_id"],
        "candidate_module": binding["p2"]["candidate_module"],
        "p2_receipt": binding["p2"]["receipt"],
        "p2_gate": binding["p2"]["gate"],
        "p2_only": True,
        "p0_eligible": False,
        "execution_lock": binding["execution_lock"],
        "runtime_promoted": False,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
    }
    _write(paths.candidate_freeze, candidate_freeze, paths)

    activity = _new_activity()
    completed: list[dict[str, Any]] = []
    attempted_case: str | None = None
    error: BaseException | None = None
    try:
        for case_id in contract.DEVELOPMENT_CASE_IDS:
            attempted_case = case_id
            _run_case(
                case_id=case_id,
                paths=paths,
                binding=binding,
                activity=activity,
                case_runner=case_runner,
            )
            completed.append(
                {
                    "case_id": case_id,
                    "receipt": _record(
                        paths.case_root(case_id) / "receipt.json", paths
                    ),
                    "gate": _record(paths.case_root(case_id) / "gate.json", paths),
                }
            )
            attempted_case = None
        _finalize(paths=paths, binding=binding, activity=activity)
    except BaseException as exc:
        error = exc
        # The frozen physical runtime performs exactly one actor query before
        # every environment step, including a terminal partial prefix.
        activity["actor_query_count"] = activity["environment_step_calls"]
    passed = error is None and len(completed) == len(contract.DEVELOPMENT_CASE_IDS)
    ledger = _terminal_payload(
        passed=passed,
        paths=paths,
        binding=binding,
        completed=completed,
        attempted_case=attempted_case,
        activity=activity,
        error=error,
    )
    _write(paths.pipeline_ledger, ledger, paths)
    verified = verify_terminal_ledger(paths)
    if not passed:
        assert error is not None
        raise contract.V12R15PhysicalDevelopmentError(
            f"V12R15 P3 stopped terminally at {attempted_case}: {error}"
        ) from error
    return verified


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    operation = parser.add_mutually_exclusive_group(required=True)
    operation.add_argument("--describe", action="store_true")
    operation.add_argument("--prepare-lock", action="store_true")
    operation.add_argument("--preflight", action="store_true")
    operation.add_argument("--execute", action="store_true")
    operation.add_argument("--verify", action="store_true")
    parser.add_argument("--acknowledge", default=None)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        if args.describe:
            result = contract.describe_protocol()
        elif args.prepare_lock:
            if args.acknowledge != PREPARE_ACKNOWLEDGEMENT:
                raise contract.V12R15PhysicalDevelopmentError(
                    "--prepare-lock requires the exact P2-pass acknowledgement"
                )
            result = prepare_lock()
        elif args.preflight:
            result = contract.preflight()
        elif args.execute:
            if args.acknowledge != EXECUTE_ACKNOWLEDGEMENT:
                raise contract.V12R15PhysicalDevelopmentError(
                    "--execute requires the exact one-shot/no-retry acknowledgement"
                )
            result = execute()
        else:
            result = verify_terminal_ledger()
        print(json.dumps(result, indent=2, sort_keys=True, allow_nan=False))
        return 0 if result.get("passed") is True else 2
    except contract.V12R15PhysicalDevelopmentError as exc:
        print(
            json.dumps(
                {
                    "status": "FAIL_H0_V12R15_P3_PHYSICAL_CLI",
                    "passed": False,
                    "error_type": type(exc).__name__,
                    "error": str(exc),
                },
                indent=2,
                sort_keys=True,
            ),
            file=sys.stderr,
        )
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
