"""Execute Q3 exactly once: six baselines, six candidates, then six-of-six.

The execution engine is local to Q3 and does not import the Q1 runner.  Every
stage is claimed once, every rollout persists its full step journal before its
gate, failure is terminal, and PASS stops before any checkpoint-zero/update or
positive-morphology action.
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
from pathlib import Path
from typing import Any

import h0_v12r5_q3_artifacts as artifacts
import h0_v12r5_q3_physical_rollout as physical_runtime
import h0_v12r5_q3_qualification_gates as gates
import h0_v12r5_q3_runtime_contract as contract
import prepare_h0_v12r5_q3_qualification_noise_tapes as noise


ROOT_VALIDATION = artifacts.REPO_ROOT / "validation"
if str(ROOT_VALIDATION) not in sys.path:
    sys.path.insert(0, str(ROOT_VALIDATION))

import freeze_h0_v12r5_q3_qualification_protocol as protocol_freezer  # noqa: E402
import h0_forensic_rollout as forensic  # noqa: E402


class V12R5Q3QualificationExecutionError(RuntimeError):
    """Raised when the terminal one-shot Q3 runtime cannot continue."""


LOCK_PASS_STATUS = contract.EXECUTION_LOCK_PASS_STATUS
PIPELINE_CLAIM_STATUS = "CLAIM_H0_V12R5_Q3_QUALIFICATION_PIPELINE"
WORKER_CLAIM_STATUS = "CLAIM_H0_V12R5_Q3_QUALIFICATION_WORKER"
ROLLOUT_STARTED_STATUS = "STARTED_H0_V12R5_Q3_QUALIFICATION_ROLLOUT"
ROLLOUT_PERSISTED_STATUS = "PERSISTED_H0_V12R5_Q3_QUALIFICATION_BEFORE_GATE"
STAGE_FAILURE_STATUS = "FAIL_H0_V12R5_Q3_QUALIFICATION_STAGE"
LEDGER_PASS_STATUS = "PASS_H0_V12R5_Q3_QUALIFICATION_PIPELINE_TERMINAL"
LEDGER_FAIL_STATUS = "FAIL_H0_V12R5_Q3_QUALIFICATION_PIPELINE_TERMINAL"
NEXT_STAGE_AFTER_PASS = contract.design.NEXT_STAGE_AFTER_Q3_PASS

RUN_ROOT = artifacts.resolve_relative(contract.RUN_ROOT)
LOCK_PATH = artifacts.resolve_relative(contract.EXECUTION_LOCK_PATH)
PROTOCOL_FREEZE_PATH = artifacts.resolve_relative(contract.PROTOCOL_FREEZE_PATH)
PIPELINE_CLAIM_PATH = artifacts.resolve_relative(contract.PIPELINE_CLAIM_PATH)
PIPELINE_LEDGER_PATH = artifacts.resolve_relative(contract.PIPELINE_LEDGER_PATH)
NOISE_ROOT = artifacts.resolve_relative(contract.NOISE_ROOT)
WORKER_CLAIMS_ROOT = artifacts.resolve_relative(contract.WORKER_CLAIMS_ROOT)

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
    "offline_teacher_labeling_authorized": False,
    "teacher_authorized": False,
    "blending_authorized": False,
    "safety_latch_authorized": False,
    "actor_updates_authorized": False,
    "critic_updates_authorized": False,
    "ppo_updates_authorized": False,
    "positive_morphology_authorized": False,
    "runtime_promotion_authorized": False,
    "checkpoint_zero_authorized": False,
}

_ACTIVITY = {
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


def _strict_mapping(path: Path) -> dict[str, Any]:
    artifacts.assert_no_link_components(path)
    try:
        value = forensic.strict_json_load(path)
    except BaseException as exc:
        raise V12R5Q3QualificationExecutionError(
            f"cannot load strict JSON: {path}"
        ) from exc
    if not isinstance(value, Mapping):
        raise V12R5Q3QualificationExecutionError(f"expected JSON object: {path}")
    result = dict(value)
    if path.read_bytes() != forensic.canonical_json_bytes(result):
        raise V12R5Q3QualificationExecutionError(f"JSON is not canonical: {path}")
    return result


def _token_sha256(token: str) -> str:
    return hashlib.sha256(token.encode("utf-8")).hexdigest()


def _stage_root(stage_id: str) -> Path:
    if stage_id == "finalize_qualification":
        return artifacts.resolve_relative(contract.FINAL_ROOT)
    try:
        _, role, case_id = stage_id.split("__", 2)
    except ValueError as exc:
        raise V12R5Q3QualificationExecutionError(
            f"invalid Q3 stage: {stage_id}"
        ) from exc
    return artifacts.resolve_relative(contract.rollout_root(role, case_id))


def _stage_receipt_path(stage_id: str) -> Path:
    return _stage_root(stage_id) / "receipt.json"


def _stage_failure_path(stage_id: str) -> Path:
    return _stage_root(stage_id) / "failure.json"


def _claim_path(stage_id: str) -> Path:
    return artifacts.resolve_relative(contract.worker_claim_path(stage_id))


def _pair_path(case_id: str) -> Path:
    return artifacts.resolve_relative(contract.pair_gate_path(case_id))


def _sync_candidate_from_protocol() -> dict[str, Any]:
    try:
        freeze = protocol_freezer.verify_protocol_freeze()
        return contract.bind_candidate(
            freeze["selected_candidate_id"], freeze["selected_candidate"]
        )
    except BaseException as exc:
        raise V12R5Q3QualificationExecutionError(
            "Q3 protocol/candidate binding is not ready"
        ) from exc


def _current_runtime_sources(freeze: Mapping[str, Any]) -> dict[str, Any]:
    frozen = freeze.get("runtime_sources")
    if not isinstance(frozen, Mapping) or set(frozen) != set(
        protocol_freezer.RUNTIME_SOURCE_RELATIVE_PATHS
    ):
        raise V12R5Q3QualificationExecutionError(
            "protocol runtime source closure is malformed"
        )
    current = {
        name: artifacts.record(artifacts.resolve_relative(path))
        for name, path in protocol_freezer.RUNTIME_SOURCE_RELATIVE_PATHS.items()
    }
    if current != dict(frozen):
        raise V12R5Q3QualificationExecutionError("runtime source closure drifted")
    return current


def _current_runtime_inputs(freeze: Mapping[str, Any]) -> dict[str, Any]:
    frozen = freeze.get("runtime_inputs")
    if not isinstance(frozen, Mapping):
        raise V12R5Q3QualificationExecutionError("runtime input closure is malformed")
    observed = protocol_freezer._input_gate()
    if observed.get("passed") is not True or observed.get("records") != frozen:
        raise V12R5Q3QualificationExecutionError("runtime input closure drifted")
    return copy.deepcopy(dict(frozen))


def _runtime_record() -> dict[str, Any]:
    source = artifacts.record(Path(physical_runtime.__file__).resolve())
    return {
        "physical_rollout_source": source,
        "inference_stack_ready": True,
        "platform": {
            "system": platform.system(),
            "machine": platform.machine(),
            "python": platform.python_version(),
            "executable": str(Path(sys.executable).resolve()),
        },
    }


def build_execution_lock(*, require_unoccupied: bool = True) -> dict[str, Any]:
    """Build the no-update lock without claiming the qualification root."""

    for path in (LOCK_PATH, RUN_ROOT, PIPELINE_CLAIM_PATH, PIPELINE_LEDGER_PATH):
        artifacts.assert_no_link_components(path)
    if require_unoccupied:
        for path, label in (
            (LOCK_PATH, "execution lock"),
            (RUN_ROOT, "qualification run root"),
            (PIPELINE_CLAIM_PATH, "pipeline claim"),
            (PIPELINE_LEDGER_PATH, "pipeline ledger"),
        ):
            if os.path.lexists(path):
                raise V12R5Q3QualificationExecutionError(
                    f"{label} already exists/no-clobber"
                )
    freeze = protocol_freezer.verify_protocol_freeze()
    binding = _sync_candidate_from_protocol()
    sources = _current_runtime_sources(freeze)
    inputs = _current_runtime_inputs(freeze)
    manifest = noise.verify_manifest()
    runtime = _runtime_record()
    checks = {
        "protocol_freeze_pass": freeze.get("status")
        == contract.PROTOCOL_FREEZE_PASS_STATUS
        and freeze.get("passed") is True,
        "design_freeze_exact": freeze.get("qualification_design_freeze")
        == noise.DESIGN_FREEZE_RECORD,
        "same_exact_r5_candidate": freeze.get("selected_candidate_id")
        == binding["candidate_id"]
        and freeze.get("selected_candidate") == binding["candidate_module"],
        "noise_manifest_current": freeze.get("noise_manifest")
        == artifacts.record(artifacts.resolve_relative(contract.NOISE_MANIFEST_PATH))
        and manifest.get("passed") is True
        and manifest.get("candidate_id") == binding["candidate_id"],
        "runtime_sources_current": sources == freeze.get("runtime_sources"),
        "runtime_inputs_current": inputs == freeze.get("runtime_inputs"),
        "runtime_ready": runtime.get("inference_stack_ready") is True,
        "matrix_exact_baseline_first": freeze.get("rollout_matrix")
        == list(contract.ROLLOUT_MATRIX)
        and list(contract.STAGE_IDS[:6])
        == [f"rollout__baseline__{case_id}" for case_id in contract.CASE_IDS]
        and list(contract.STAGE_IDS[6:12])
        == [f"rollout__candidate__{case_id}" for case_id in contract.CASE_IDS],
        "thirteen_stages_exact": len(contract.STAGE_IDS) == 13
        and contract.STAGE_IDS[-1] == "finalize_qualification",
        "no_fit_teacher_blend_latch_update_authority": all(
            EXECUTION_AUTHORITY[name] is False
            for name in (
                "fit_authorized",
                "offline_teacher_labeling_authorized",
                "teacher_authorized",
                "blending_authorized",
                "safety_latch_authorized",
                "actor_updates_authorized",
                "critic_updates_authorized",
                "ppo_updates_authorized",
            )
        ),
        "prepublication_unoccupied": True,
    }
    if not all(checks.values()):
        failed = [name for name, value in checks.items() if value is not True]
        raise V12R5Q3QualificationExecutionError(
            f"execution lock checks failed: {failed}"
        )
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": LOCK_PASS_STATUS,
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "authority": copy.deepcopy(EXECUTION_AUTHORITY),
        "checks": checks,
        "protocol_freeze": artifacts.record(PROTOCOL_FREEZE_PATH),
        "qualification_design_freeze": copy.deepcopy(noise.DESIGN_FREEZE_RECORD),
        "noise_manifest": artifacts.record(
            artifacts.resolve_relative(contract.NOISE_MANIFEST_PATH)
        ),
        "candidate_id": binding["candidate_id"],
        "candidate_module": copy.deepcopy(binding["candidate_module"]),
        "runtime_sources": sources,
        "runtime_inputs": inputs,
        "runtime": runtime,
        "rollout_matrix": list(contract.ROLLOUT_MATRIX),
        "stage_order": list(contract.STAGE_IDS),
        "run_root": contract.RUN_ROOT.as_posix(),
        "update_activity": {
            "actor_updates": 0,
            "critic_updates": 0,
            "ppo_updates": 0,
        },
        "retry_authorized": False,
        "resume_authorized": False,
        "runtime_promoted": False,
        "checkpoint_zero_created": False,
    }


def prepare_execution_lock() -> dict[str, Any]:
    if os.path.lexists(LOCK_PATH):
        raise V12R5Q3QualificationExecutionError("execution lock exists/no-clobber")
    payload = build_execution_lock(require_unoccupied=True)
    forensic.write_json_exclusive(LOCK_PATH, payload)
    return verify_execution_lock(require_run_root_absent=True)


def verify_execution_lock(*, require_run_root_absent: bool = False) -> dict[str, Any]:
    observed = _strict_mapping(LOCK_PATH)
    expected = build_execution_lock(require_unoccupied=False)
    if observed != expected or LOCK_PATH.read_bytes() != forensic.canonical_json_bytes(
        expected
    ):
        raise V12R5Q3QualificationExecutionError("execution lock drifted")
    if require_run_root_absent and os.path.lexists(RUN_ROOT):
        raise V12R5Q3QualificationExecutionError("Q3 run root is already claimed")
    return observed


def _pipeline_claim_payload(token_sha256: str) -> dict[str, Any]:
    binding = contract.current_candidate_binding()
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": PIPELINE_CLAIM_STATUS,
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "one_shot": True,
        "execution_token_sha256": token_sha256,
        "candidate_id": binding["candidate_id"],
        "candidate_module": copy.deepcopy(binding["candidate_module"]),
        "protocol_freeze": artifacts.record(PROTOCOL_FREEZE_PATH),
        "execution_lock": artifacts.record(LOCK_PATH),
        "stage_order": list(contract.STAGE_IDS),
        "rollout_matrix": list(contract.ROLLOUT_MATRIX),
        "retry_authorized": False,
        "resume_authorized": False,
        "rescue_authorized": False,
        "sweep_authorized": False,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "runtime_promoted": False,
        "checkpoint_zero_created": False,
    }


def _claim_run_root() -> str:
    token = secrets.token_hex(32)
    token_hash = _token_sha256(token)
    if os.path.lexists(RUN_ROOT):
        raise V12R5Q3QualificationExecutionError("Q3 run root exists/no retry")
    RUN_ROOT.mkdir(parents=True, exist_ok=False)
    WORKER_CLAIMS_ROOT.mkdir(exist_ok=False)
    forensic.write_json_exclusive(
        PIPELINE_CLAIM_PATH, _pipeline_claim_payload(token_hash)
    )
    return token


def _verify_pipeline_claim() -> dict[str, Any]:
    observed = _strict_mapping(PIPELINE_CLAIM_PATH)
    token_hash = observed.get("execution_token_sha256")
    if not isinstance(token_hash, str) or len(token_hash) != 64:
        raise V12R5Q3QualificationExecutionError("pipeline token hash drifted")
    expected = _pipeline_claim_payload(token_hash)
    if observed != expected:
        raise V12R5Q3QualificationExecutionError("pipeline claim drifted")
    return observed


def _previous_receipts(stage_id: str) -> list[dict[str, Any]]:
    index = contract.STAGE_IDS.index(stage_id)
    return [
        {"stage_id": prior, "receipt": artifacts.record(_stage_receipt_path(prior))}
        for prior in contract.STAGE_IDS[:index]
    ]


def _worker_claim_payload(stage_id: str, token_sha256: str) -> dict[str, Any]:
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": WORKER_CLAIM_STATUS,
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "one_shot": True,
        "stage_id": stage_id,
        "stage_index": contract.STAGE_IDS.index(stage_id),
        "execution_token_sha256": token_sha256,
        "pipeline_claim": artifacts.record(PIPELINE_CLAIM_PATH),
        "previous_receipts": _previous_receipts(stage_id),
        "retry_authorized": False,
        "resume_authorized": False,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "runtime_promoted": False,
    }


def _write_worker_claim(stage_id: str, token_sha256: str) -> None:
    path = _claim_path(stage_id)
    if os.path.lexists(path) or os.path.lexists(_stage_receipt_path(stage_id)):
        raise V12R5Q3QualificationExecutionError(f"stage already consumed: {stage_id}")
    forensic.write_json_exclusive(path, _worker_claim_payload(stage_id, token_sha256))


def _verify_worker_claim(stage_id: str) -> dict[str, Any]:
    observed = _strict_mapping(_claim_path(stage_id))
    pipeline = _verify_pipeline_claim()
    expected = _worker_claim_payload(stage_id, pipeline["execution_token_sha256"])
    if observed != expected:
        raise V12R5Q3QualificationExecutionError(f"worker claim drifted: {stage_id}")
    return observed


def _prospective_record(path: Path, payload: Any) -> dict[str, Any]:
    encoded = forensic.canonical_json_bytes(payload)
    return {
        "path": artifacts.portable_path(path),
        "sha256": hashlib.sha256(encoded).hexdigest(),
        "size_bytes": len(encoded),
    }


def _build_rollout_summary(
    *,
    role: str,
    case_id: str,
    physical: Mapping[str, Any],
    evidence: Mapping[str, Mapping[str, Any]],
    noise_tape: Mapping[str, Any],
    noise_tape_array_sha256: str,
    actor_module: Mapping[str, Any],
) -> dict[str, Any]:
    expected = contract.canonical_rollout(role, case_id)
    summary = {
        **copy.deepcopy(expected),
        **copy.deepcopy(dict(physical)),
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.ROLLOUT_COMPLETE_STATUS,
        "protocol_id": contract.PROTOCOL_ID,
        "actor_module": copy.deepcopy(dict(actor_module)),
        "trace_step_count": int(physical.get("steps", -1)),
        "teacher_enabled": False,
        "teacher_loaded_during_rollout": False,
        "blending_enabled": False,
        "safety_latch_enabled": False,
        "actor_query_count": int(physical.get("steps", -1)),
        "random_noise_draw_count": (
            contract.EXPECTED_STEPS
            if expected["action_selection"] == "stochastic"
            else 0
        ),
        "single_noise_application_count": int(physical.get("steps", -1)),
        "noise_tape": copy.deepcopy(dict(noise_tape)),
        "noise_tape_array_sha256": noise_tape_array_sha256,
        "prerequisite_gate_passed": True,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
        "runtime_promoted": False,
        "checkpoint_zero_created": False,
        "retry_authorized": False,
        "resume_authorized": False,
        "rescue_authorized": False,
        "sweep_authorized": False,
        "post_hoc_tuning_authorized": False,
        **{name: copy.deepcopy(dict(record)) for name, record in evidence.items()},
    }
    procedural_zero = {
        "safety_latch_activation_count",
        "safety_latch_release_count",
        "safety_intervention_count",
        "physical_gate_bypass_count",
        "multiple_noise_application_count",
        "noise_application_mismatch_count",
        "served_action_teacher_dependency_count",
        "teacher_query_count",
        "mean_blend_count",
        "actor_updates",
        "critic_updates",
        "ppo_updates",
    }
    for field in procedural_zero:
        if summary.get(field, 0) != 0 or type(summary.get(field, 0)) is not int:
            raise V12R5Q3QualificationExecutionError(
                f"non-zero procedural counter: {field}"
            )
        summary[field] = 0
    missing = [field for field in contract.ZERO_REQUIRED_COUNTS if field not in summary]
    if missing:
        raise V12R5Q3QualificationExecutionError(
            f"physical summary lacks required counters: {missing}"
        )
    return summary


RolloutCollector = Callable[..., Mapping[str, Any]]


def _run_rollout(
    stage_id: str,
    *,
    collector: RolloutCollector = physical_runtime.collect_physical_rollout,
) -> dict[str, Any]:
    _, role, case_id = stage_id.split("__", 2)
    case = contract.canonical_case(case_id)
    destination = _stage_root(stage_id)
    if os.path.lexists(destination):
        raise V12R5Q3QualificationExecutionError(
            f"rollout destination exists: {stage_id}"
        )
    _verify_pipeline_claim()
    _verify_worker_claim(stage_id)
    lock = verify_execution_lock(require_run_root_absent=False)
    writer = forensic.ForensicRolloutWriter(
        destination, artifact_root=artifacts.REPO_ROOT
    )
    writer.start(
        {
            "schema_version": contract.SCHEMA_VERSION,
            "status": ROLLOUT_STARTED_STATUS,
            "protocol_id": contract.PROTOCOL_ID,
            "pipeline_id": contract.PIPELINE_ID,
            "stage_id": stage_id,
            "role": role,
            "case": case,
            "candidate_id": contract.R5_CANDIDATE_ID,
            "protocol_freeze": artifacts.record(PROTOCOL_FREEZE_PATH),
            "execution_lock": artifacts.record(LOCK_PATH),
            "pipeline_claim": artifacts.record(PIPELINE_CLAIM_PATH),
            "worker_claim": artifacts.record(_claim_path(stage_id)),
            "teacher_enabled": False,
            "blending_enabled": False,
            "safety_latch_enabled": False,
            "retry_authorized": False,
            "actor_updates": 0,
            "critic_updates": 0,
            "ppo_updates": 0,
        }
    )
    try:
        collected = collector(
            role=role,
            case=case,
            runtime_inputs=lock["runtime_inputs"],
            activity=_ACTIVITY,
            persist_step=writer.write_step,
        )
        rows = collected["rows"]
        partial = {
            "schema_version": contract.SCHEMA_VERSION,
            "status": ROLLOUT_PERSISTED_STATUS,
            "protocol_id": contract.PROTOCOL_ID,
            "stage_id": stage_id,
            "role": role,
            "case_id": case_id,
            "steps": len(rows),
            "gate_evaluated": False,
            "update_activity": {
                "actor_updates": 0,
                "critic_updates": 0,
                "ppo_updates": 0,
            },
            "retry_authorized": False,
        }
        evidence = {
            "protocol_freeze": artifacts.record(PROTOCOL_FREEZE_PATH),
            "execution_lock": artifacts.record(LOCK_PATH),
            "pipeline_claim": artifacts.record(PIPELINE_CLAIM_PATH),
            "worker_claim": artifacts.record(_claim_path(stage_id)),
            "run_start": artifacts.record(writer.run_start_path),
            "trace": _prospective_record(writer.trace_path, rows),
            "partial_summary": _prospective_record(
                writer.partial_summary_path, partial
            ),
        }
        input_name = (
            "source_h0_module" if role == contract.BASELINE_ROLE else "candidate_module"
        )
        summary = _build_rollout_summary(
            role=role,
            case_id=case_id,
            physical=collected["physical_summary"],
            evidence=evidence,
            noise_tape=collected["noise_tape"],
            noise_tape_array_sha256=collected["noise_tape_array_sha256"],
            actor_module=lock["runtime_inputs"][input_name],
        )
        persisted = writer.finalize_before_gate(
            trace=rows, partial_summary=partial, summary=summary
        )
        for name in ("trace", "partial_summary"):
            if persisted.get(name) != evidence[name]:
                raise V12R5Q3QualificationExecutionError(
                    f"persist-before-gate record drifted: {stage_id}/{name}"
                )
        common_gate = gates.common_rollout_gate(summary, role=role, case_id=case_id)
        writer.publish_gate(common_gate)
        if common_gate.get("passed") is not True:
            failed = [
                name
                for name, value in common_gate.get("checks", {}).items()
                if value is not True
            ]
            raise V12R5Q3QualificationExecutionError(
                f"common rollout gate failed: {stage_id}: {failed}"
            )
        pair_gate = None
        if role == contract.CANDIDATE_ROLE:
            baseline = _strict_mapping(
                artifacts.resolve_relative(
                    contract.rollout_root(contract.BASELINE_ROLE, case_id)
                )
                / "summary.json"
            )
            pair_gate = gates.condition_matched_gate(baseline, summary, case_id=case_id)
            forensic.write_json_exclusive(_pair_path(case_id), pair_gate)
            if pair_gate.get("passed") is not True:
                raise V12R5Q3QualificationExecutionError(
                    f"condition-matched pair failed: {case_id}"
                )
        receipt = {
            "schema_version": contract.SCHEMA_VERSION,
            "status": common_gate["status"],
            "passed": True,
            "protocol_id": contract.PROTOCOL_ID,
            "pipeline_id": contract.PIPELINE_ID,
            "stage_id": stage_id,
            "role": role,
            "case_id": case_id,
            "candidate_id": contract.R5_CANDIDATE_ID,
            "summary": artifacts.record(writer.summary_path),
            "gate": artifacts.record(writer.gate_path),
            "trace": artifacts.record(writer.trace_path),
            "pair_gate": (
                artifacts.record(_pair_path(case_id))
                if role == contract.CANDIDATE_ROLE
                else None
            ),
            "pipeline_claim": artifacts.record(PIPELINE_CLAIM_PATH),
            "worker_claim": artifacts.record(_claim_path(stage_id)),
            "retry_authorized": False,
            "resume_authorized": False,
            "update_activity": {
                "actor_updates": 0,
                "critic_updates": 0,
                "ppo_updates": 0,
            },
            "protected_trials_opened": [],
            "reserve_trials_opened": [],
            "runtime_promoted": False,
        }
        forensic.write_json_exclusive(_stage_receipt_path(stage_id), receipt)
        return _verify_rollout_receipt(stage_id)
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
                        "activity": copy.deepcopy(_ACTIVITY),
                        "update_activity": {
                            "actor_updates": 0,
                            "critic_updates": 0,
                            "ppo_updates": 0,
                        },
                        "retry_authorized": False,
                        "runtime_promoted": False,
                    },
                )
            except BaseException:
                pass
        raise


def _verify_rollout_receipt(stage_id: str) -> dict[str, Any]:
    _, role, case_id = stage_id.split("__", 2)
    root = _stage_root(stage_id)
    writer = forensic.ForensicRolloutWriter(root, artifact_root=artifacts.REPO_ROOT)
    persisted = writer.finalized_artifact_records()
    rows = writer._step_rows()
    trace = forensic.strict_json_load(writer.trace_path)
    if trace != rows or writer.last_completed_step != contract.EXPECTED_STEPS:
        raise V12R5Q3QualificationExecutionError(
            f"trace/journal closure drifted: {stage_id}"
        )
    summary = _strict_mapping(root / "summary.json")
    observed_gate = _strict_mapping(root / "gate.json")
    expected_gate = gates.common_rollout_gate(summary, role=role, case_id=case_id)
    pair_record = None
    if role == contract.CANDIDATE_ROLE:
        baseline = _strict_mapping(
            artifacts.resolve_relative(
                contract.rollout_root(contract.BASELINE_ROLE, case_id)
            )
            / "summary.json"
        )
        pair = _strict_mapping(_pair_path(case_id))
        expected_pair = gates.condition_matched_gate(baseline, summary, case_id=case_id)
        if pair != expected_pair or pair.get("passed") is not True:
            raise V12R5Q3QualificationExecutionError(
                f"pair gate closure drifted: {case_id}"
            )
        pair_record = artifacts.record(_pair_path(case_id))
    receipt = _strict_mapping(_stage_receipt_path(stage_id))
    expected_receipt = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": expected_gate["status"],
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "stage_id": stage_id,
        "role": role,
        "case_id": case_id,
        "candidate_id": contract.R5_CANDIDATE_ID,
        "summary": artifacts.record(writer.summary_path),
        "gate": artifacts.record(writer.gate_path),
        "trace": artifacts.record(writer.trace_path),
        "pair_gate": pair_record,
        "pipeline_claim": artifacts.record(PIPELINE_CLAIM_PATH),
        "worker_claim": artifacts.record(_claim_path(stage_id)),
        "retry_authorized": False,
        "resume_authorized": False,
        "update_activity": {
            "actor_updates": 0,
            "critic_updates": 0,
            "ppo_updates": 0,
        },
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
        "runtime_promoted": False,
    }
    evidence_exact = all(
        summary.get(name) == record
        for name, record in {
            "protocol_freeze": artifacts.record(PROTOCOL_FREEZE_PATH),
            "execution_lock": artifacts.record(LOCK_PATH),
            "pipeline_claim": artifacts.record(PIPELINE_CLAIM_PATH),
            "worker_claim": artifacts.record(_claim_path(stage_id)),
            "run_start": artifacts.record(writer.run_start_path),
            "trace": persisted["trace"],
            "partial_summary": persisted["partial_summary"],
            "noise_tape": artifacts.record(
                artifacts.resolve_relative(
                    contract.canonical_case(case_id)["noise_tape"]
                )
            ),
        }.items()
    )
    if (
        receipt != expected_receipt
        or observed_gate != expected_gate
        or observed_gate.get("passed") is not True
        or not evidence_exact
    ):
        raise V12R5Q3QualificationExecutionError(
            f"rollout receipt/gate closure drifted: {stage_id}"
        )
    return receipt


def _aggregate_summary(stage_id: str) -> dict[str, Any]:
    if stage_id != "finalize_qualification":
        raise V12R5Q3QualificationExecutionError(f"invalid aggregate stage: {stage_id}")
    binding = contract.current_candidate_binding()
    pair_bindings = [
        {
            "case_id": case_id,
            "passed": True,
            "pair_gate": artifacts.record(_pair_path(case_id)),
            "baseline_receipt": artifacts.record(
                artifacts.resolve_relative(
                    contract.rollout_receipt_path(contract.BASELINE_ROLE, case_id)
                )
            ),
            "candidate_receipt": artifacts.record(
                artifacts.resolve_relative(
                    contract.rollout_receipt_path(contract.CANDIDATE_ROLE, case_id)
                )
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
        "prerequisite_gate_passed": True,
        "rollout_matrix": list(contract.ROLLOUT_MATRIX),
        "baseline_rollout_count": 6,
        "candidate_rollout_count": 6,
        "total_rollout_count": 12,
        "pair_bindings": pair_bindings,
        "pair_count": 6,
        "passing_pair_count": 6,
        "failed_pair_count": 0,
        "protocol_freeze": artifacts.record(PROTOCOL_FREEZE_PATH),
        "execution_lock": artifacts.record(LOCK_PATH),
        "pipeline_claim": artifacts.record(PIPELINE_CLAIM_PATH),
        "worker_claim": artifacts.record(_claim_path(stage_id)),
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "update_activity": {
            "actor_updates": 0,
            "critic_updates": 0,
            "ppo_updates": 0,
        },
        "compensation_or_averaging_used": False,
        "retry_authorized": False,
        "resume_authorized": False,
        "rescue_authorized": False,
        "sweep_authorized": False,
        "post_hoc_tuning_authorized": False,
        "runtime_promoted": False,
        "checkpoint_zero_created": False,
        "positive_morphology_enabled": False,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
    }


def _aggregate_receipt(stage_id: str, gate: Mapping[str, Any]) -> dict[str, Any]:
    root = _stage_root(stage_id)
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": gate["status"],
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "stage_id": stage_id,
        "candidate_id": contract.R5_CANDIDATE_ID,
        "case_count": 6,
        "summary": artifacts.record(root / "summary.json"),
        "gate": artifacts.record(root / "gate.json"),
        "pair_gates": [
            artifacts.record(_pair_path(case_id)) for case_id in contract.CASE_IDS
        ],
        "pipeline_claim": artifacts.record(PIPELINE_CLAIM_PATH),
        "worker_claim": artifacts.record(_claim_path(stage_id)),
        "retry_authorized": False,
        "resume_authorized": False,
        "update_activity": {
            "actor_updates": 0,
            "critic_updates": 0,
            "ppo_updates": 0,
        },
        "runtime_promoted": False,
    }


def _run_aggregate(stage_id: str) -> dict[str, Any]:
    root = _stage_root(stage_id)
    if os.path.lexists(root):
        raise V12R5Q3QualificationExecutionError("aggregate root exists/no retry")
    _verify_worker_claim(stage_id)
    for rollout_stage in contract.STAGE_IDS[:12]:
        _verify_rollout_receipt(rollout_stage)
    root.mkdir(parents=True, exist_ok=False)
    try:
        summary = _aggregate_summary(stage_id)
        forensic.write_json_exclusive(root / "summary.json", summary)
        gate = gates.aggregate_qualification_gate(summary)
        forensic.write_json_exclusive(root / "gate.json", gate)
        if gate.get("passed") is not True:
            raise V12R5Q3QualificationExecutionError("aggregate six-of-six gate failed")
        receipt = _aggregate_receipt(stage_id, gate)
        forensic.write_json_exclusive(root / "receipt.json", receipt)
        return _verify_aggregate_receipt(stage_id)
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
                    "pipeline_id": contract.PIPELINE_ID,
                    "stage_id": stage_id,
                    "candidate_id": contract.R5_CANDIDATE_ID,
                    "error_type": type(exc).__name__,
                    "error": str(exc),
                    "activity": copy.deepcopy(_ACTIVITY),
                    "update_activity": {
                        "actor_updates": 0,
                        "critic_updates": 0,
                        "ppo_updates": 0,
                    },
                    "retry_authorized": False,
                    "resume_authorized": False,
                    "runtime_promoted": False,
                },
            )
        raise


def _verify_aggregate_receipt(
    stage_id: str = "finalize_qualification",
) -> dict[str, Any]:
    _verify_worker_claim(stage_id)
    for rollout_stage in contract.STAGE_IDS[:12]:
        _verify_rollout_receipt(rollout_stage)
    root = _stage_root(stage_id)
    summary = _strict_mapping(root / "summary.json")
    gate = _strict_mapping(root / "gate.json")
    expected_summary = _aggregate_summary(stage_id)
    expected_gate = gates.aggregate_qualification_gate(summary)
    receipt = _strict_mapping(root / "receipt.json")
    expected_receipt = _aggregate_receipt(stage_id, expected_gate)
    if (
        summary != expected_summary
        or gate != expected_gate
        or gate.get("passed") is not True
        or receipt != expected_receipt
    ):
        raise V12R5Q3QualificationExecutionError(
            "aggregate summary/gate/receipt closure drifted"
        )
    return receipt


def _run_stage(
    stage_id: str,
    *,
    collector: RolloutCollector = physical_runtime.collect_physical_rollout,
) -> dict[str, Any]:
    if stage_id.startswith("rollout__"):
        return _run_rollout(stage_id, collector=collector)
    if stage_id == "finalize_qualification":
        return _run_aggregate(stage_id)
    raise V12R5Q3QualificationExecutionError(f"unauthorized stage: {stage_id}")


def _ledger_payload(
    *,
    passed: bool,
    attempted_stage: str | None,
    completed_stages: Sequence[str],
    error: BaseException | None,
) -> dict[str, Any]:
    binding = contract.current_candidate_binding()
    attempted_claim = (
        artifacts.record(_claim_path(attempted_stage))
        if attempted_stage is not None and _claim_path(attempted_stage).is_file()
        else None
    )
    attempted_failure = (
        artifacts.record(_stage_failure_path(attempted_stage))
        if attempted_stage is not None
        and _stage_failure_path(attempted_stage).is_file()
        else None
    )
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": LEDGER_PASS_STATUS if passed else LEDGER_FAIL_STATUS,
        "passed": passed,
        "terminal": True,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "candidate_id": binding["candidate_id"],
        "candidate_module": copy.deepcopy(binding["candidate_module"]),
        "stage_order": list(contract.STAGE_IDS),
        "attempted_stage": attempted_stage,
        "attempted_stage_claim": attempted_claim,
        "attempted_stage_failure": attempted_failure,
        "completed_stages": list(completed_stages),
        "completed_receipts": [
            {"stage_id": stage, "receipt": artifacts.record(_stage_receipt_path(stage))}
            for stage in completed_stages
        ],
        "error_type": type(error).__name__ if error is not None else None,
        "error": str(error) if error is not None else None,
        "protocol_freeze": artifacts.record(PROTOCOL_FREEZE_PATH),
        "execution_lock": artifacts.record(LOCK_PATH),
        "pipeline_claim": artifacts.record(PIPELINE_CLAIM_PATH),
        "activity": copy.deepcopy(_ACTIVITY),
        "update_activity": {
            "actor_updates": _ACTIVITY["actor_updates"],
            "critic_updates": _ACTIVITY["critic_updates"],
            "ppo_updates": _ACTIVITY["ppo_updates"],
        },
        "aggregate_requires_6_of_6": True,
        "compensation_authorized": False,
        "retry_authorized": False,
        "resume_authorized": False,
        "rescue_authorized": False,
        "sweep_authorized": False,
        "post_hoc_tuning_authorized": False,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
        "runtime_promoted": False,
        "checkpoint_zero_created": False,
        "positive_morphology_enabled": False,
        "next_stage": NEXT_STAGE_AFTER_PASS if passed else "STOP_TERMINAL",
    }


def verify_pipeline_ledger() -> dict[str, Any]:
    ledger = _strict_mapping(PIPELINE_LEDGER_PATH)
    completed = ledger.get("completed_stages")
    receipts = ledger.get("completed_receipts")
    if not isinstance(completed, list) or not isinstance(receipts, list):
        raise V12R5Q3QualificationExecutionError("terminal ledger stages are malformed")
    checks = {
        "schema": ledger.get("schema_version") == contract.SCHEMA_VERSION,
        "terminal": ledger.get("terminal") is True,
        "protocol": ledger.get("protocol_id") == contract.PROTOCOL_ID,
        "stage_order": ledger.get("stage_order") == list(contract.STAGE_IDS),
        "completed_prefix": completed == list(contract.STAGE_IDS[: len(completed)]),
        "receipt_count": len(receipts) == len(completed),
        "receipt_bindings": all(
            row
            == {
                "stage_id": stage,
                "receipt": artifacts.record(_stage_receipt_path(stage)),
            }
            for row, stage in zip(receipts, completed, strict=True)
        ),
        "candidate": ledger.get("candidate_id") == contract.R5_CANDIDATE_ID
        and ledger.get("candidate_module") == contract.R5_CANDIDATE_MODULE,
        "zero_updates": ledger.get("update_activity")
        == {"actor_updates": 0, "critic_updates": 0, "ppo_updates": 0},
        "one_shot_no_promotion": ledger.get("retry_authorized") is False
        and ledger.get("resume_authorized") is False
        and ledger.get("runtime_promoted") is False
        and ledger.get("checkpoint_zero_created") is False
        and ledger.get("positive_morphology_enabled") is False,
    }
    if ledger.get("passed") is True:
        checks.update(
            {
                "pass_status": ledger.get("status") == LEDGER_PASS_STATUS,
                "all_thirteen": completed == list(contract.STAGE_IDS),
                "no_attempt": ledger.get("attempted_stage") is None
                and ledger.get("attempted_stage_claim") is None
                and ledger.get("attempted_stage_failure") is None,
                "activity_exact": ledger.get("activity") == EXPECTED_TERMINAL_ACTIVITY,
                "next_stage": ledger.get("next_stage") == NEXT_STAGE_AFTER_PASS,
            }
        )
        _verify_aggregate_receipt()
    else:
        checks.update(
            {
                "fail_status": ledger.get("status") == LEDGER_FAIL_STATUS,
                "attempted_next": ledger.get("attempted_stage")
                == contract.STAGE_IDS[len(completed)]
                if len(completed) < len(contract.STAGE_IDS)
                else ledger.get("attempted_stage") is None,
                "stop": ledger.get("next_stage") == "STOP_TERMINAL",
            }
        )
    if not all(checks.values()):
        failed = [name for name, value in checks.items() if value is not True]
        raise V12R5Q3QualificationExecutionError(f"terminal ledger drifted: {failed}")
    return ledger


def execute_qualification_once(
    *,
    collector: RolloutCollector = physical_runtime.collect_physical_rollout,
) -> dict[str, Any]:
    """Consume the frozen 13-stage matrix exactly once and terminalize."""

    verify_execution_lock(require_run_root_absent=True)
    for name in _ACTIVITY:
        _ACTIVITY[name] = 0
    completed: list[str] = []
    attempted: str | None = None
    claimed = False
    try:
        token = _claim_run_root()
        claimed = True
        token_hash = _token_sha256(token)
        token = ""
        for stage_id in contract.STAGE_IDS:
            attempted = stage_id
            _write_worker_claim(stage_id, token_hash)
            receipt = _run_stage(stage_id, collector=collector)
            if receipt.get("passed") is not True:
                raise V12R5Q3QualificationExecutionError(
                    f"stage returned non-PASS: {stage_id}"
                )
            completed.append(stage_id)
        attempted = None
        if completed != list(contract.STAGE_IDS):
            raise V12R5Q3QualificationExecutionError("thirteen-stage order drifted")
        if _ACTIVITY != EXPECTED_TERMINAL_ACTIVITY:
            raise V12R5Q3QualificationExecutionError(
                f"terminal activity counters drifted: {_ACTIVITY}"
            )
        ledger = _ledger_payload(
            passed=True,
            attempted_stage=None,
            completed_stages=completed,
            error=None,
        )
        forensic.write_json_exclusive(PIPELINE_LEDGER_PATH, ledger)
        return verify_pipeline_ledger()
    except BaseException as exc:
        if claimed and not os.path.lexists(PIPELINE_LEDGER_PATH):
            forensic.write_json_exclusive(
                PIPELINE_LEDGER_PATH,
                _ledger_payload(
                    passed=False,
                    attempted_stage=attempted,
                    completed_stages=completed,
                    error=exc,
                ),
            )
            verify_pipeline_ledger()
        raise V12R5Q3QualificationExecutionError(
            "Q3 qualification failed terminally; retry/resume are forbidden"
        ) from exc


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    action = parser.add_mutually_exclusive_group(required=True)
    action.add_argument("--prepare-execution-lock", action="store_true")
    action.add_argument("--verify-execution-lock", action="store_true")
    action.add_argument("--verify-ledger", action="store_true")
    action.add_argument("--execute-once", action="store_true")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    if args.prepare_execution_lock:
        payload = prepare_execution_lock()
    elif args.verify_execution_lock:
        payload = verify_execution_lock()
    elif args.verify_ledger:
        payload = verify_pipeline_ledger()
    else:
        payload = execute_qualification_once()
    print(payload["status"])
    return 0 if payload.get("passed") is True else 1


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())


__all__ = [
    "EXECUTION_AUTHORITY",
    "EXPECTED_TERMINAL_ACTIVITY",
    "LEDGER_FAIL_STATUS",
    "LEDGER_PASS_STATUS",
    "NEXT_STAGE_AFTER_PASS",
    "V12R5Q3QualificationExecutionError",
    "build_execution_lock",
    "execute_qualification_once",
    "prepare_execution_lock",
    "verify_execution_lock",
    "verify_pipeline_ledger",
]
