"""Run the candidate-bound V12R7-Q3 qualification exactly once.

The runtime is deliberately inert on import.  ``execute_qualification`` first
re-runs the five official R7 semantic verifiers, verifies the immutable Q3
protocol/lock, source closure and noise manifest, and then claims the canonical
run root.  The only execution order is six baselines, six condition-matched
candidates, and one aggregate stage.  Any failure is terminal and cannot be
resumed or retried.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import os
import platform
import secrets
import sys
from collections.abc import Callable, Mapping, Sequence
from pathlib import Path, PurePosixPath
from typing import Any


RUNTIME_ROOT = Path(__file__).resolve().parent
Q3_ROOT = RUNTIME_ROOT.parent
REPO_ROOT = Q3_ROOT.parents[3]
for _root in (Q3_ROOT, RUNTIME_ROOT, REPO_ROOT / "validation"):
    if str(_root) not in sys.path:
        sys.path.insert(0, str(_root))

import h0_forensic_rollout as forensic  # noqa: E402
import h0_v12r7_q3_artifacts as artifacts  # noqa: E402
import h0_v12r7_q3_prerequisites as prerequisites  # noqa: E402
import h0_v12r7_q3_qualification_contract as contract  # noqa: E402
import h0_v12r7_q3_qualification_gates as gates  # noqa: E402
import freeze_h0_v12r7_q3_qualification_protocol as freezer  # noqa: E402
import h0_v12r7_q3_physical_rollout as physical_runtime  # noqa: E402
import prepare_h0_v12r7_q3_noise_tapes as noise  # noqa: E402


class V12R7Q3QualificationExecutionError(RuntimeError):
    """Raised when the immutable one-shot Q3 pipeline cannot continue."""


PIPELINE_CLAIM_STATUS = "CLAIM_H0_V12R7_Q3_QUALIFICATION_PIPELINE"
ROLLOUT_STARTED_STATUS = "STARTED_H0_V12R7_Q3_QUALIFICATION_ROLLOUT"
ROLLOUT_PERSISTED_STATUS = "PERSISTED_H0_V12R7_Q3_BEFORE_GATE"
STAGE_FAILURE_STATUS = "FAIL_H0_V12R7_Q3_QUALIFICATION_STAGE"
PIPELINE_TERMINAL_FAIL_STATUS = "FAIL_H0_V12R7_Q3_PIPELINE_TERMINAL"

STAGE_IDS = (
    *tuple(
        f"rollout__{role}__{case_id}"
        for role in contract.ROLE_ORDER
        for case_id in contract.CASE_IDS
    ),
    "finalize_qualification",
)

EXECUTION_AUTHORITY = {
    "one_shot": True,
    "rollout_stages": 12,
    "pair_gate_count": 6,
    "aggregate_stages": 1,
    "baseline_first": True,
    "retry_authorized": False,
    "resume_authorized": False,
    "rescue_authorized": False,
    "sweep_authorized": False,
    "post_hoc_tuning_authorized": False,
    "fit_authorized": False,
    "teacher_authorized": False,
    "blending_authorized": False,
    "safety_latch_authorized": False,
    "actor_updates_authorized": False,
    "critic_updates_authorized": False,
    "ppo_updates_authorized": False,
    "checkpoint_zero_authorized": False,
    "positive_morphology_authorized": False,
    "runtime_promotion_authorized": False,
}

ACTIVITY_TEMPLATE = {
    "environment_reset_calls": 0,
    "environment_step_calls": 0,
    "baseline_actor_queries": 0,
    "candidate_actor_queries": 0,
    "teacher_queries": 0,
    "blend_count": 0,
    "latch_count": 0,
    "actor_updates": 0,
    "critic_updates": 0,
    "ppo_updates": 0,
}
EXPECTED_TERMINAL_ACTIVITY = {
    "environment_reset_calls": 12,
    "environment_step_calls": 12 * contract.EXPECTED_STEPS,
    "baseline_actor_queries": 6 * contract.EXPECTED_STEPS,
    "candidate_actor_queries": 6 * contract.EXPECTED_STEPS,
    "teacher_queries": 0,
    "blend_count": 0,
    "latch_count": 0,
    "actor_updates": 0,
    "critic_updates": 0,
    "ppo_updates": 0,
}

RolloutCollector = Callable[..., Mapping[str, Any]]


def _path(value: str | PurePosixPath) -> Path:
    return freezer.resolve_relative(value)


def _record(value: str | PurePosixPath | Path) -> dict[str, Any]:
    return freezer._record(value)


def _strict_mapping(value: str | PurePosixPath | Path) -> dict[str, Any]:
    path = value if isinstance(value, Path) else _path(value)
    try:
        payload = forensic.strict_json_load(path)
    except BaseException as exc:
        raise V12R7Q3QualificationExecutionError(
            f"cannot load strict JSON: {path}"
        ) from exc
    if not isinstance(payload, Mapping):
        raise V12R7Q3QualificationExecutionError(f"expected JSON object: {path}")
    result = dict(payload)
    if path.read_bytes() != forensic.canonical_json_bytes(result):
        raise V12R7Q3QualificationExecutionError(f"JSON is not canonical: {path}")
    return result


def _tree_record(value: str | PurePosixPath) -> dict[str, Any]:
    root = _path(value).absolute()
    if not root.is_dir() or root.is_symlink():
        raise V12R7Q3QualificationExecutionError(f"unsafe artifact tree: {root}")
    entries = sorted(root.rglob("*"), key=lambda item: item.as_posix())
    if any(item.is_symlink() for item in entries):
        raise V12R7Q3QualificationExecutionError(
            f"artifact tree contains a symbolic link: {root}"
        )
    files = [item for item in entries if item.is_file()]
    if not files:
        raise V12R7Q3QualificationExecutionError(f"artifact tree is empty: {root}")
    rows = []
    for item in files:
        record = _record(item)
        rows.append(
            {
                "path": item.relative_to(root).as_posix(),
                "sha256": record["sha256"],
                "size_bytes": record["size_bytes"],
            }
        )
    return {
        "path": root.relative_to(REPO_ROOT).as_posix(),
        "tree_sha256": artifacts.tree_digest(rows),
        "file_count": len(rows),
        "files": rows,
    }


def _prospective_record(path: Path, payload: Any) -> dict[str, Any]:
    encoded = forensic.canonical_json_bytes(payload)
    try:
        relative = path.absolute().relative_to(REPO_ROOT)
    except ValueError as exc:
        raise V12R7Q3QualificationExecutionError(
            f"prospective artifact escaped repository: {path}"
        ) from exc
    return {
        "path": relative.as_posix(),
        "sha256": hashlib.sha256(encoded).hexdigest(),
        "size_bytes": len(encoded),
    }


def _stage_root(stage_id: str) -> Path:
    if stage_id == STAGE_IDS[-1]:
        return _path(contract.FINAL_ROOT)
    try:
        prefix, role, case_id = stage_id.split("__", 2)
    except ValueError as exc:
        raise V12R7Q3QualificationExecutionError(
            f"invalid Q3 stage: {stage_id}"
        ) from exc
    if prefix != "rollout":
        raise V12R7Q3QualificationExecutionError(f"invalid Q3 stage: {stage_id}")
    return _path(contract.canonical_rollout(role, case_id)["destination"])


def _rollout_receipt_path(role: str, case_id: str) -> Path:
    return _path(contract.rollout_receipt_path(role, case_id))


def _pair_path(case_id: str) -> Path:
    return _path(contract.pair_gate_path(case_id))


def _runtime_inputs(
    lock: Mapping[str, Any], snapshot: Mapping[str, Any]
) -> dict[str, Any]:
    gate = snapshot.get("gate")
    if not isinstance(gate, Mapping):
        raise V12R7Q3QualificationExecutionError("live R7 gate is malformed")
    if (
        gate.get("passed") is not True
        or gate.get("candidate_id") != lock.get("candidate_id")
        or gate.get("candidate_module") != lock.get("candidate_module")
    ):
        raise V12R7Q3QualificationExecutionError(
            "live R7 candidate no longer matches the Q3 lock"
        )
    candidate = _tree_record(contract.CANDIDATE_MODULE_PATH)
    if candidate != lock.get("candidate_module"):
        raise V12R7Q3QualificationExecutionError("R7 candidate tree bytes drifted")
    prerequisites.validate_candidate_tree(lock.get("candidate_id"), candidate)
    source_h0 = _tree_record(contract.SOURCE_H0_MODULE["path"])
    if (
        source_h0.get("tree_sha256") != contract.SOURCE_H0_MODULE["tree_sha256"]
        or source_h0.get("path") != contract.SOURCE_H0_MODULE["path"]
    ):
        raise V12R7Q3QualificationExecutionError("source H0 tree bytes drifted")
    closure = lock.get("source_closure")
    if not isinstance(closure, Mapping):
        raise V12R7Q3QualificationExecutionError("Q3 lock source closure is malformed")
    result = {
        "candidate_module": candidate,
        "source_h0_module": source_h0,
        "source_h0_config": copy.deepcopy(closure.get("source_h0_config")),
        "historical_analog_profile": copy.deepcopy(
            closure.get("historical_analog_profile")
        ),
        "baseline_shadow_v25_profile": copy.deepcopy(
            closure.get("baseline_shadow_v25_profile")
        ),
    }
    for name in (
        "source_h0_config",
        "historical_analog_profile",
        "baseline_shadow_v25_profile",
    ):
        expected = contract.QUALIFICATION_INPUT_PATHS[name]
        if not artifacts.artifact_record_matches(result[name], expected):
            raise V12R7Q3QualificationExecutionError(
                f"runtime input record drifted: {name}"
            )
        if result[name] != _record(expected):
            raise V12R7Q3QualificationExecutionError(
                f"runtime input bytes drifted: {name}"
            )
    return result


def build_execution_preflight(
    *, require_run_root_absent: bool = True
) -> dict[str, Any]:
    """Verify every live prerequisite without opening the Q3 run root."""

    if contract.HISTORICAL_TERMINAL_FAILURE is True:
        raise V12R7Q3QualificationExecutionError(
            "V12R7 terminal FAIL permanently closes this historical Q3 runner; "
            f"use a fresh {contract.SUCCESSOR_NAMESPACE} namespace"
        )
    protocol = freezer.verify_protocol_freeze()
    lock = freezer.verify_execution_lock()
    snapshot = freezer.live_r7_prerequisite_snapshot(require_q3_unopened=False)
    closure = freezer.source_closure()
    manifest = noise.verify_manifest()
    runtime_inputs = _runtime_inputs(lock, snapshot)
    run_absent = not os.path.lexists(_path(contract.RUN_ROOT))
    ledger_absent = not os.path.lexists(_path(contract.PIPELINE_LEDGER_PATH))
    checks = {
        "protocol_pass": protocol.get("passed") is True,
        "lock_pass": lock.get("passed") is True,
        "five_live_r7_verifiers": snapshot.get("gate", {}).get("passed") is True
        and snapshot.get("gate", {}).get("official_verifier_count") == 5,
        "same_candidate": protocol.get("candidate_id")
        == lock.get("candidate_id")
        == snapshot.get("gate", {}).get("candidate_id")
        == manifest.get("candidate_id"),
        "source_closure_current": protocol.get("source_closure")
        == lock.get("source_closure")
        == closure,
        "noise_manifest_pass": manifest.get("passed") is True,
        "runtime_inputs_exact": set(runtime_inputs)
        == {
            "candidate_module",
            "source_h0_module",
            "source_h0_config",
            "historical_analog_profile",
            "baseline_shadow_v25_profile",
        },
        "baseline_first": tuple(
            (row["role"], row["case_id"]) for row in contract.ROLLOUT_MATRIX[:6]
        )
        == tuple((contract.BASELINE_ROLE, case_id) for case_id in contract.CASE_IDS)
        and tuple(row["role"] for row in contract.ROLLOUT_MATRIX[6:])
        == (contract.CANDIDATE_ROLE,) * 6,
        "thirteen_stages": len(STAGE_IDS) == 13
        and STAGE_IDS[-1] == "finalize_qualification",
        "run_root_absent": run_absent,
        "terminal_ledger_absent": ledger_absent,
        "closed_authority": all(
            EXECUTION_AUTHORITY[name] is False
            for name in (
                "fit_authorized",
                "teacher_authorized",
                "blending_authorized",
                "safety_latch_authorized",
                "actor_updates_authorized",
                "critic_updates_authorized",
                "ppo_updates_authorized",
                "checkpoint_zero_authorized",
                "positive_morphology_authorized",
                "runtime_promotion_authorized",
            )
        ),
    }
    if require_run_root_absent and not all(checks.values()):
        failed = sorted(name for name, passed in checks.items() if passed is not True)
        raise V12R7Q3QualificationExecutionError(
            f"Q3 execution preflight failed: {failed}"
        )
    if not require_run_root_absent:
        checks.pop("run_root_absent")
        checks.pop("terminal_ledger_absent")
        if not all(checks.values()):
            failed = sorted(
                name for name, passed in checks.items() if passed is not True
            )
            raise V12R7Q3QualificationExecutionError(
                f"Q3 execution closure failed: {failed}"
            )
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": "PASS_H0_V12R7_Q3_EXECUTION_PREFLIGHT",
        "passed": True,
        "checks": checks,
        "candidate_id": lock["candidate_id"],
        "candidate_module": copy.deepcopy(lock["candidate_module"]),
        "protocol_freeze": _record(contract.PROTOCOL_FREEZE_PATH),
        "execution_lock": _record(contract.EXECUTION_LOCK_PATH),
        "noise_manifest": _record(contract.NOISE_ROOT / "manifest.json"),
        "runtime_inputs": runtime_inputs,
        "stage_order": list(STAGE_IDS),
        "rollout_matrix": list(contract.ROLLOUT_MATRIX),
        "publication_performed": False,
    }


def _pipeline_claim_payload(
    preflight: Mapping[str, Any], token_hash: str
) -> dict[str, Any]:
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": PIPELINE_CLAIM_STATUS,
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "one_shot": True,
        "execution_token_sha256": token_hash,
        "candidate_id": preflight["candidate_id"],
        "candidate_module": copy.deepcopy(preflight["candidate_module"]),
        "protocol_freeze": copy.deepcopy(preflight["protocol_freeze"]),
        "execution_lock": copy.deepcopy(preflight["execution_lock"]),
        "noise_manifest": copy.deepcopy(preflight["noise_manifest"]),
        "runtime_inputs": copy.deepcopy(preflight["runtime_inputs"]),
        "stage_order": list(STAGE_IDS),
        "rollout_matrix": list(contract.ROLLOUT_MATRIX),
        "authority": copy.deepcopy(EXECUTION_AUTHORITY),
        "platform": {
            "system": platform.system(),
            "machine": platform.machine(),
            "python": platform.python_version(),
            "executable": str(Path(sys.executable).resolve()),
        },
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "checkpoint_zero_created": False,
        "positive_morphology_enabled": False,
        "runtime_promoted": False,
        "retry_authorized": False,
        "resume_authorized": False,
    }


def _claim_pipeline(preflight: Mapping[str, Any]) -> str:
    run_root = _path(contract.RUN_ROOT)
    if os.path.lexists(run_root):
        raise V12R7Q3QualificationExecutionError("Q3 run root exists/no retry")
    token = secrets.token_hex(32)
    token_hash = hashlib.sha256(token.encode("utf-8")).hexdigest()
    run_root.mkdir(parents=True, exist_ok=False)
    forensic.write_json_exclusive(
        _path(contract.PIPELINE_CLAIM_PATH),
        _pipeline_claim_payload(preflight, token_hash),
    )
    return token


def _verify_pipeline_claim() -> dict[str, Any]:
    claim = _strict_mapping(contract.PIPELINE_CLAIM_PATH)
    token_hash = claim.get("execution_token_sha256")
    if not artifacts.is_sha256(token_hash):
        raise V12R7Q3QualificationExecutionError("pipeline token hash drifted")
    preflight = build_execution_preflight(require_run_root_absent=False)
    expected = _pipeline_claim_payload(preflight, token_hash)
    if claim != expected:
        raise V12R7Q3QualificationExecutionError("pipeline claim drifted")
    return claim


def build_rollout_summary(
    *,
    role: str,
    case_id: str,
    candidate_id: str,
    actor_module: Mapping[str, Any],
    physical: Mapping[str, Any],
    noise_tape: Mapping[str, Any],
    noise_tape_array_sha256: str,
    evidence: Mapping[str, Mapping[str, Any]],
) -> dict[str, Any]:
    """Compose the persisted-before-gate summary as a pure function."""

    expected = contract.canonical_rollout(role, case_id)
    summary = {
        **copy.deepcopy(expected),
        **copy.deepcopy(dict(physical)),
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.ROLLOUT_COMPLETE_STATUS,
        "protocol_id": contract.PROTOCOL_ID,
        "candidate_id": candidate_id,
        "actor_module": copy.deepcopy(dict(actor_module)),
        "role": role,
        "case_id": case_id,
        "action_selection": expected["action_selection"],
        "episode_start_offset_s": expected["episode_start_offset_s"],
        "action_seed": expected["action_seed"],
        "runtime_seed": expected["runtime_seed"],
        "sigma": expected["sigma"],
        "resolved_env_config": copy.deepcopy(expected["resolved_env_config"]),
        "noise_tape": copy.deepcopy(dict(noise_tape)),
        "noise_tape_array_sha256": noise_tape_array_sha256,
        "checkpoint_zero_created": False,
        "positive_morphology_enabled": False,
        "runtime_promoted": False,
        "retry_authorized": False,
        "resume_authorized": False,
        "rescue_authorized": False,
        "sweep_authorized": False,
        "post_hoc_tuning_authorized": False,
        "compensation_or_averaging_used": False,
        **{name: copy.deepcopy(dict(record)) for name, record in evidence.items()},
    }
    for name in contract.ZERO_REQUIRED_COUNTS:
        summary.setdefault(name, 0)
    return summary


def _run_start_payload(stage_id: str, role: str, case_id: str) -> dict[str, Any]:
    binding = prerequisites.current_candidate_binding()
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": ROLLOUT_STARTED_STATUS,
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "stage_id": stage_id,
        "stage_index": STAGE_IDS.index(stage_id),
        "role": role,
        "case_id": case_id,
        "candidate_id": binding["candidate_id"],
        "pipeline_claim": _record(contract.PIPELINE_CLAIM_PATH),
        "one_shot": True,
        "retry_authorized": False,
        "resume_authorized": False,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "runtime_promoted": False,
    }


def _rollout_receipt_payload(
    *, role: str, case_id: str, gate: Mapping[str, Any]
) -> dict[str, Any]:
    root = _path(contract.canonical_rollout(role, case_id)["destination"])
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": gate["status"],
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "stage_id": f"rollout__{role}__{case_id}",
        "role": role,
        "case_id": case_id,
        "candidate_id": prerequisites.current_candidate_binding()["candidate_id"],
        "summary": _record(root / "summary.json"),
        "gate": _record(root / "gate.json"),
        "trace": _record(root / "trace.json"),
        "pair_gate": (
            _record(contract.pair_gate_path(case_id))
            if role == contract.CANDIDATE_ROLE
            else None
        ),
        "pipeline_claim": _record(contract.PIPELINE_CLAIM_PATH),
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "retry_authorized": False,
        "resume_authorized": False,
        "runtime_promoted": False,
    }


def _run_rollout(
    stage_id: str,
    *,
    runtime_inputs: Mapping[str, Mapping[str, Any]],
    activity: dict[str, int],
    collector: RolloutCollector,
) -> dict[str, Any]:
    _prefix, role, case_id = stage_id.split("__", 2)
    root = _stage_root(stage_id)
    if os.path.lexists(root):
        raise V12R7Q3QualificationExecutionError(f"stage exists/no retry: {stage_id}")
    writer = forensic.ForensicRolloutWriter(root, artifact_root=REPO_ROOT)
    writer.start(_run_start_payload(stage_id, role, case_id))
    try:
        collected = collector(
            role=role,
            case=contract.canonical_case(case_id),
            runtime_inputs=runtime_inputs,
            activity=activity,
            persist_step=writer.write_step,
        )
        rows = collected.get("rows")
        physical = collected.get("physical_summary")
        tape_record = collected.get("noise_tape")
        tape_hash = collected.get("noise_tape_array_sha256")
        if (
            not isinstance(rows, list)
            or not isinstance(physical, Mapping)
            or not isinstance(tape_record, Mapping)
            or not artifacts.is_sha256(tape_hash)
        ):
            raise V12R7Q3QualificationExecutionError(
                f"collector result malformed: {stage_id}"
            )
        partial = {
            "schema_version": contract.SCHEMA_VERSION,
            "status": ROLLOUT_PERSISTED_STATUS,
            "passed": True,
            "protocol_id": contract.PROTOCOL_ID,
            "stage_id": stage_id,
            "role": role,
            "case_id": case_id,
            "steps": len(rows),
            "gate_evaluated": False,
            "actor_updates": 0,
            "critic_updates": 0,
            "ppo_updates": 0,
            "retry_authorized": False,
        }
        evidence = {
            "protocol_freeze": _record(contract.PROTOCOL_FREEZE_PATH),
            "execution_lock": _record(contract.EXECUTION_LOCK_PATH),
            "pipeline_claim": _record(contract.PIPELINE_CLAIM_PATH),
            "run_start": _record(writer.run_start_path),
            "trace": _prospective_record(writer.trace_path, rows),
        }
        input_name = (
            "source_h0_module" if role == contract.BASELINE_ROLE else "candidate_module"
        )
        binding = prerequisites.current_candidate_binding()
        summary = build_rollout_summary(
            role=role,
            case_id=case_id,
            candidate_id=binding["candidate_id"],
            actor_module=runtime_inputs[input_name],
            physical=physical,
            noise_tape=tape_record,
            noise_tape_array_sha256=tape_hash,
            evidence=evidence,
        )
        persisted = writer.finalize_before_gate(
            trace=rows, partial_summary=partial, summary=summary
        )
        if persisted["trace"] != evidence["trace"]:
            raise V12R7Q3QualificationExecutionError(
                f"persist-before-gate trace drifted: {stage_id}"
            )
        common = gates.common_rollout_gate(summary, role=role, case_id=case_id)
        writer.publish_gate(common)
        if common.get("passed") is not True:
            failed = sorted(
                name
                for name, passed in common.get("checks", {}).items()
                if passed is not True
            )
            raise V12R7Q3QualificationExecutionError(
                f"common rollout gate failed: {stage_id}: {failed}"
            )
        if role == contract.CANDIDATE_ROLE:
            baseline = _strict_mapping(
                _stage_root(f"rollout__{contract.BASELINE_ROLE}__{case_id}")
                / "summary.json"
            )
            pair = gates.condition_matched_gate(baseline, summary, case_id=case_id)
            forensic.write_json_exclusive(_pair_path(case_id), pair)
            if pair.get("passed") is not True:
                raise V12R7Q3QualificationExecutionError(
                    f"condition-matched gate failed: {case_id}"
                )
        receipt = _rollout_receipt_payload(role=role, case_id=case_id, gate=common)
        forensic.write_json_exclusive(root / "receipt.json", receipt)
        return verify_rollout_receipt(role, case_id)
    except BaseException as exc:
        if writer.run_start_path.is_file() and not os.path.lexists(writer.failure_path):
            try:
                writer.publish_failure(
                    end_reason="q3_rollout_failed_terminal_no_retry",
                    error=exc,
                    status=STAGE_FAILURE_STATUS,
                    details={
                        "stage_id": stage_id,
                        "role": role,
                        "case_id": case_id,
                        "activity": copy.deepcopy(activity),
                        "retry_authorized": False,
                        "resume_authorized": False,
                        "runtime_promoted": False,
                    },
                )
            except BaseException:
                pass
        raise


def verify_rollout_receipt(role: str, case_id: str) -> dict[str, Any]:
    """Recompute one rollout gate and its immutable artifact chain."""

    contract.canonical_rollout(role, case_id)
    root = _path(contract.canonical_rollout(role, case_id)["destination"])
    writer = forensic.ForensicRolloutWriter(root, artifact_root=REPO_ROOT)
    persisted = writer.finalized_artifact_records()
    rows = writer._step_rows()
    trace = forensic.strict_json_load(writer.trace_path)
    if trace != rows or writer.last_completed_step != contract.EXPECTED_STEPS:
        raise V12R7Q3QualificationExecutionError(
            f"rollout trace/journal drifted: {role}/{case_id}"
        )
    summary = _strict_mapping(root / "summary.json")
    observed_gate = _strict_mapping(root / "gate.json")
    expected_gate = gates.common_rollout_gate(summary, role=role, case_id=case_id)
    if observed_gate != expected_gate or expected_gate.get("passed") is not True:
        raise V12R7Q3QualificationExecutionError(
            f"rollout gate drifted: {role}/{case_id}"
        )
    expected_pair = None
    if role == contract.CANDIDATE_ROLE:
        baseline = _strict_mapping(
            _stage_root(f"rollout__{contract.BASELINE_ROLE}__{case_id}")
            / "summary.json"
        )
        pair = _strict_mapping(_pair_path(case_id))
        recalculated = gates.condition_matched_gate(baseline, summary, case_id=case_id)
        if pair != recalculated or pair.get("passed") is not True:
            raise V12R7Q3QualificationExecutionError(f"pair gate drifted: {case_id}")
        expected_pair = _record(contract.pair_gate_path(case_id))
    evidence_ok = all(
        summary.get(name) == record
        for name, record in {
            "protocol_freeze": _record(contract.PROTOCOL_FREEZE_PATH),
            "execution_lock": _record(contract.EXECUTION_LOCK_PATH),
            "pipeline_claim": _record(contract.PIPELINE_CLAIM_PATH),
            "run_start": _record(writer.run_start_path),
            "trace": persisted["trace"],
            "noise_tape": _record(contract.canonical_case(case_id)["noise_tape"]),
        }.items()
    )
    receipt = _strict_mapping(root / "receipt.json")
    expected_receipt = _rollout_receipt_payload(
        role=role, case_id=case_id, gate=expected_gate
    )
    if (
        receipt != expected_receipt
        or receipt.get("pair_gate") != expected_pair
        or not evidence_ok
    ):
        raise V12R7Q3QualificationExecutionError(
            f"rollout receipt closure drifted: {role}/{case_id}"
        )
    return receipt


def _aggregate_summary() -> dict[str, Any]:
    binding = prerequisites.current_candidate_binding()
    pair_bindings = [
        {
            "case_id": case_id,
            "passed": True,
            "pair_gate": _record(contract.pair_gate_path(case_id)),
            "baseline_receipt": _record(
                contract.rollout_receipt_path(contract.BASELINE_ROLE, case_id)
            ),
            "candidate_receipt": _record(
                contract.rollout_receipt_path(contract.CANDIDATE_ROLE, case_id)
            ),
        }
        for case_id in contract.CASE_IDS
    ]
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.AGGREGATE_COMPLETE_STATUS,
        "protocol_id": contract.PROTOCOL_ID,
        "candidate_id": binding["candidate_id"],
        "candidate_module": copy.deepcopy(binding["candidate_module"]),
        "rollout_matrix": list(contract.ROLLOUT_MATRIX),
        "baseline_rollout_count": 6,
        "candidate_rollout_count": 6,
        "total_rollout_count": 12,
        "pair_bindings": pair_bindings,
        "pair_count": 6,
        "passing_pair_count": 6,
        "failed_pair_count": 0,
        "protocol_freeze": _record(contract.PROTOCOL_FREEZE_PATH),
        "execution_lock": _record(contract.EXECUTION_LOCK_PATH),
        "pipeline_claim": _record(contract.PIPELINE_CLAIM_PATH),
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "compensation_or_averaging_used": False,
        "retry_authorized": False,
        "resume_authorized": False,
        "rescue_authorized": False,
        "sweep_authorized": False,
        "post_hoc_tuning_authorized": False,
        "checkpoint_zero_created": False,
        "morphology_weight": 0.0,
        "positive_morphology_enabled": False,
        "runtime_promoted": False,
    }


def _final_receipt_payload(gate: Mapping[str, Any]) -> dict[str, Any]:
    binding = prerequisites.current_candidate_binding()
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": gate["status"],
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "stage_id": "finalize_qualification",
        "candidate_id": binding["candidate_id"],
        "candidate_module": copy.deepcopy(binding["candidate_module"]),
        "case_count": 6,
        "summary": _record(contract.FINAL_ROOT / "summary.json"),
        "gate": _record(contract.FINAL_ROOT / "gate.json"),
        "pair_gates": [
            _record(contract.pair_gate_path(case_id)) for case_id in contract.CASE_IDS
        ],
        "rollout_receipts": [
            _record(contract.rollout_receipt_path(role, case_id))
            for role in contract.ROLE_ORDER
            for case_id in contract.CASE_IDS
        ],
        "pipeline_claim": _record(contract.PIPELINE_CLAIM_PATH),
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "retry_authorized": False,
        "resume_authorized": False,
        "checkpoint_zero_created": False,
        "morphology_weight": 0.0,
        "positive_morphology_enabled": False,
        "runtime_promoted": False,
        "next_stage": contract.NEXT_STAGE_AFTER_Q3_PASS,
    }


def _run_aggregate() -> dict[str, Any]:
    root = _path(contract.FINAL_ROOT)
    if os.path.lexists(root):
        raise V12R7Q3QualificationExecutionError("aggregate root exists/no retry")
    for role in contract.ROLE_ORDER:
        for case_id in contract.CASE_IDS:
            verify_rollout_receipt(role, case_id)
    root.mkdir(parents=True, exist_ok=False)
    try:
        summary = _aggregate_summary()
        forensic.write_json_exclusive(root / "summary.json", summary)
        gate = gates.aggregate_qualification_gate(summary)
        forensic.write_json_exclusive(root / "gate.json", gate)
        if gate.get("passed") is not True:
            raise V12R7Q3QualificationExecutionError("aggregate six-of-six gate failed")
        forensic.write_json_exclusive(
            root / "receipt.json", _final_receipt_payload(gate)
        )
        return verify_final_receipt()
    except BaseException as exc:
        failure = root / "failure.json"
        if not os.path.lexists(failure):
            forensic.write_json_exclusive(
                failure,
                {
                    "schema_version": contract.SCHEMA_VERSION,
                    "status": STAGE_FAILURE_STATUS,
                    "passed": False,
                    "protocol_id": contract.PROTOCOL_ID,
                    "stage_id": "finalize_qualification",
                    "candidate_id": prerequisites.current_candidate_binding()[
                        "candidate_id"
                    ],
                    "error_type": type(exc).__name__,
                    "error": str(exc),
                    "retry_authorized": False,
                    "resume_authorized": False,
                    "runtime_promoted": False,
                },
            )
        raise


def verify_final_receipt() -> dict[str, Any]:
    """Verify all twelve rollouts, six pairs, and the aggregate receipt."""

    for role in contract.ROLE_ORDER:
        for case_id in contract.CASE_IDS:
            verify_rollout_receipt(role, case_id)
    summary = _strict_mapping(contract.FINAL_ROOT / "summary.json")
    observed_gate = _strict_mapping(contract.FINAL_ROOT / "gate.json")
    expected_summary = _aggregate_summary()
    expected_gate = gates.aggregate_qualification_gate(summary)
    receipt = _strict_mapping(contract.FINAL_RECEIPT_PATH)
    expected_receipt = _final_receipt_payload(expected_gate)
    if (
        summary != expected_summary
        or observed_gate != expected_gate
        or expected_gate.get("passed") is not True
        or receipt != expected_receipt
        or receipt.get("status") != contract.AGGREGATE_PASS_STATUS
    ):
        raise V12R7Q3QualificationExecutionError("Q3 aggregate receipt closure drifted")
    return receipt


def _ledger_payload(
    *,
    passed: bool,
    attempted_stage: str | None,
    completed_stages: Sequence[str],
    activity: Mapping[str, int],
    error: BaseException | None,
) -> dict[str, Any]:
    binding = prerequisites.current_candidate_binding()
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": (
            contract.PIPELINE_TERMINAL_PASS_STATUS
            if passed
            else PIPELINE_TERMINAL_FAIL_STATUS
        ),
        "passed": passed,
        "terminal": True,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "candidate_id": binding["candidate_id"],
        "candidate_module": copy.deepcopy(binding["candidate_module"]),
        "stage_order": list(STAGE_IDS),
        "attempted_stage": attempted_stage,
        "completed_stages": list(completed_stages),
        "completed_receipts": [
            {
                "stage_id": stage,
                "receipt": _record(
                    contract.FINAL_RECEIPT_PATH
                    if stage == "finalize_qualification"
                    else _stage_root(stage) / "receipt.json"
                ),
            }
            for stage in completed_stages
        ],
        "error_type": type(error).__name__ if error is not None else None,
        "error": str(error) if error is not None else None,
        "protocol_freeze": _record(contract.PROTOCOL_FREEZE_PATH),
        "execution_lock": _record(contract.EXECUTION_LOCK_PATH),
        "pipeline_claim": _record(contract.PIPELINE_CLAIM_PATH),
        "final_receipt": _record(contract.FINAL_RECEIPT_PATH) if passed else None,
        "activity": copy.deepcopy(dict(activity)),
        "actor_updates": int(activity.get("actor_updates", -1)),
        "critic_updates": int(activity.get("critic_updates", -1)),
        "ppo_updates": int(activity.get("ppo_updates", -1)),
        "aggregate_requires_6_of_6": True,
        "compensation_authorized": False,
        "retry_authorized": False,
        "resume_authorized": False,
        "rescue_authorized": False,
        "sweep_authorized": False,
        "post_hoc_tuning_authorized": False,
        "checkpoint_zero_created": False,
        "morphology_weight": 0.0,
        "positive_morphology_enabled": False,
        "runtime_promoted": False,
        "next_stage": contract.NEXT_STAGE_AFTER_Q3_PASS if passed else "STOP_TERMINAL",
    }


def verify_terminal_ledger() -> dict[str, Any]:
    """Verify the terminal-PASS Q3 ledger and checkpoint-zero handoff."""

    final_receipt = verify_final_receipt()
    ledger = _strict_mapping(contract.PIPELINE_LEDGER_PATH)
    expected = _ledger_payload(
        passed=True,
        attempted_stage="finalize_qualification",
        completed_stages=STAGE_IDS,
        activity=EXPECTED_TERMINAL_ACTIVITY,
        error=None,
    )
    checks = {
        "exact_payload": ledger == expected,
        "terminal_pass": ledger.get("status") == contract.PIPELINE_TERMINAL_PASS_STATUS
        and ledger.get("passed") is True
        and ledger.get("terminal") is True,
        "final_receipt": ledger.get("final_receipt")
        == _record(contract.FINAL_RECEIPT_PATH)
        and final_receipt.get("status") == contract.AGGREGATE_PASS_STATUS,
        "zero_updates": all(
            type(ledger.get(name)) is int and ledger[name] == 0
            for name in ("actor_updates", "critic_updates", "ppo_updates")
        ),
        "closed": ledger.get("checkpoint_zero_created") is False
        and ledger.get("positive_morphology_enabled") is False
        and ledger.get("runtime_promoted") is False
        and ledger.get("next_stage") == contract.NEXT_STAGE_AFTER_Q3_PASS,
    }
    if not all(checks.values()):
        failed = sorted(name for name, passed in checks.items() if passed is not True)
        raise V12R7Q3QualificationExecutionError(
            f"Q3 terminal ledger drifted: {failed}"
        )
    return ledger


def execute_qualification(
    *, collector: RolloutCollector | None = None
) -> dict[str, Any]:
    """Claim and execute the canonical Q3 pipeline; never resume or retry."""

    selected_collector = (
        physical_runtime.collect_physical_rollout if collector is None else collector
    )
    preflight = build_execution_preflight(require_run_root_absent=True)
    _claim_pipeline(preflight)
    activity = copy.deepcopy(ACTIVITY_TEMPLATE)
    completed: list[str] = []
    attempted: str | None = None
    try:
        _verify_pipeline_claim()
        for stage_id in STAGE_IDS:
            attempted = stage_id
            if stage_id == "finalize_qualification":
                _run_aggregate()
            else:
                _run_rollout(
                    stage_id,
                    runtime_inputs=preflight["runtime_inputs"],
                    activity=activity,
                    collector=selected_collector,
                )
            completed.append(stage_id)
        if activity != EXPECTED_TERMINAL_ACTIVITY:
            raise V12R7Q3QualificationExecutionError(
                f"terminal activity drifted: {activity}"
            )
        ledger = _ledger_payload(
            passed=True,
            attempted_stage=attempted,
            completed_stages=completed,
            activity=activity,
            error=None,
        )
        forensic.write_json_exclusive(_path(contract.PIPELINE_LEDGER_PATH), ledger)
        return verify_terminal_ledger()
    except BaseException as exc:
        ledger_path = _path(contract.PIPELINE_LEDGER_PATH)
        if not os.path.lexists(ledger_path):
            try:
                forensic.write_json_exclusive(
                    ledger_path,
                    _ledger_payload(
                        passed=False,
                        attempted_stage=attempted,
                        completed_stages=completed,
                        activity=activity,
                        error=exc,
                    ),
                )
            except BaseException:
                pass
        raise


def deferred_preflight() -> dict[str, Any]:
    """Return the source-only state without invoking R7 or publishing Q3."""

    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": "BLOCKED_H0_V12R7_Q3_HISTORICAL_R7_TERMINAL_FAIL",
        "passed": False,
        "lineage_state": contract.LINEAGE_STATE,
        "successor_namespace": contract.SUCCESSOR_NAMESPACE,
        "candidate_binding_state": contract.CANDIDATE_BINDING_STATE,
        "stage_order": list(STAGE_IDS),
        "five_live_r7_verifiers_required": True,
        "protocol_freeze_required": True,
        "noise_manifest_required": True,
        "publication_performed": False,
        "qualification_execution_authorized_now": False,
    }


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    action = parser.add_mutually_exclusive_group(required=True)
    action.add_argument("--deferred-preflight", action="store_true")
    action.add_argument("--live-preflight", action="store_true")
    action.add_argument("--execute", action="store_true")
    action.add_argument("--verify", action="store_true")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    if args.deferred_preflight:
        result = deferred_preflight()
        print(result["status"])
        return 0
    if args.live_preflight:
        result = build_execution_preflight()
    elif args.execute:
        result = execute_qualification()
    else:
        result = verify_terminal_ledger()
    print(result["status"])
    return 0 if result.get("passed") is True else 1


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())


__all__ = [
    "ACTIVITY_TEMPLATE",
    "EXPECTED_TERMINAL_ACTIVITY",
    "PIPELINE_CLAIM_STATUS",
    "PIPELINE_TERMINAL_FAIL_STATUS",
    "STAGE_IDS",
    "V12R7Q3QualificationExecutionError",
    "build_execution_preflight",
    "build_rollout_summary",
    "deferred_preflight",
    "execute_qualification",
    "verify_final_receipt",
    "verify_rollout_receipt",
    "verify_terminal_ledger",
]
