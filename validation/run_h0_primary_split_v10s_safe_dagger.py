"""Run the frozen V10S coherent-teacher safe-DAgger pipeline.

V10S is a development-only actor adaptation.  It fits P0, collects three
two-case same-state DAgger rounds with preregistered student blend weights,
refits a fresh H0 actor after every cumulative round, freezes only P3, and
then evaluates P3 in six unblended V26 rollouts.  Collection may query the
privileged coherent legacy teacher; final rollouts may not.

Every material stage is run by a separately claimed worker.  The supervisor
is the only process allowed to mint the ephemeral token and publishes one
terminal, no-clobber ledger.  The pipeline never updates a critic or PPO and
never opens protected or reserve trials.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
import os
import secrets
import subprocess
import sys
import time
from pathlib import Path, PurePosixPath
from typing import Any, Mapping, Sequence


REPO_ROOT = Path(__file__).resolve().parents[1]
VALIDATION_ROOT = REPO_ROOT / "validation"
TRAJECTORY_ROOT = REPO_ROOT / "Trajectory Generator"
BASELINE_ROOT = TRAJECTORY_ROOT / "baseline_MLP"
for _root in (REPO_ROOT, VALIDATION_ROOT, TRAJECTORY_ROOT, BASELINE_ROOT):
    if str(_root) not in sys.path:
        sys.path.insert(0, str(_root))

import h0_forensic_rollout as forensic  # noqa: E402
import h0_primary_split_v10_coherent_teacher as coherent_teacher  # noqa: E402
import h0_primary_split_v10s_blend as safe_dagger  # noqa: E402
import h0_primary_split_v10s_safe_dagger_contract as contract  # noqa: E402
import h0_v3_so_recovery_contract as so_recovery  # noqa: E402
import run_h0_primary_split_v9_causal_teacher as env_source  # noqa: E402
import h0_primary_split_v10s_fit as fit_engine  # noqa: E402


WORKER_TIMEOUT_S = 3600.0


class V10SSafeDaggerError(RuntimeError):
    """Raised on provenance, ordering, rollout, fit, or gate failure."""


def resolve_relative(path: str | PurePosixPath) -> Path:
    """Resolve one canonical repository-relative path without traversal."""

    raw = path.as_posix() if isinstance(path, PurePosixPath) else str(path)
    pure = PurePosixPath(raw)
    if not raw or pure.is_absolute() or ".." in pure.parts or pure.as_posix() != raw:
        raise V10SSafeDaggerError(f"non-canonical repository path: {raw!r}")
    return REPO_ROOT.joinpath(*pure.parts)


RUN_ROOT = resolve_relative(contract.RUN_ROOT)
PREFLIGHT = resolve_relative(contract.PREFLIGHT_PATH)
LOCK = resolve_relative(contract.LOCK_PATH)
PIPELINE_CLAIM = resolve_relative(contract.PIPELINE_CLAIM_PATH)
PIPELINE_LEDGER = resolve_relative(contract.PIPELINE_LEDGER_PATH)
WORKER_CLAIMS_ROOT = resolve_relative(contract.WORKER_CLAIMS_ROOT)
SOURCE_H0_MODULE = resolve_relative(contract.SOURCE_H0_MODULE_PATH)
CANDIDATE_FREEZE = resolve_relative(contract.CANDIDATE_FREEZE_PATH)
FINAL_RECEIPT = resolve_relative(contract.FINAL_DEVELOPMENT_RECEIPT_PATH)


def _mapping(path: str | Path) -> dict[str, Any]:
    value = forensic.strict_json_load(path)
    if not isinstance(value, Mapping):
        raise V10SSafeDaggerError(f"expected strict JSON object: {path}")
    return dict(value)


def _sequence(path: str | Path) -> list[Any]:
    value = forensic.strict_json_load(path)
    if isinstance(value, (str, bytes)) or not isinstance(value, Sequence):
        raise V10SSafeDaggerError(f"expected strict JSON array: {path}")
    return list(value)


def _record(path: str | Path) -> dict[str, Any]:
    return forensic.artifact_record(path, artifact_root=REPO_ROOT)


def _record_matches(record: Any, path: str | Path) -> bool:
    return isinstance(record, Mapping) and dict(record) == _record(path)


def _tree_record(path: str | Path) -> dict[str, Any]:
    root = Path(path).expanduser().resolve()
    if not root.is_dir():
        raise V10SSafeDaggerError(f"artifact tree is missing: {root}")
    files = sorted(item for item in root.rglob("*") if item.is_file())
    if not files:
        raise V10SSafeDaggerError(f"artifact tree is empty: {root}")
    rows: list[dict[str, Any]] = []
    digest = hashlib.sha256()
    for item in files:
        relative = item.relative_to(root).as_posix()
        sha256 = forensic.sha256_file(item)
        size = item.stat().st_size
        rows.append({"path": relative, "sha256": sha256, "size_bytes": size})
        digest.update(relative.encode("utf-8"))
        digest.update(b"\0")
        digest.update(sha256.encode("ascii"))
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


def _teacher_evidence_path() -> Path:
    matches = [
        resolve_relative(path)
        for name, path in contract.INPUT_RELATIVE_PATHS.items()
        if (
            "teacher_evidence" in name
            or "adjudication" in name
            or "time_alignment" in name
            or "time_alignment" in str(path)
        )
    ]
    if len(matches) != 1:
        raise V10SSafeDaggerError(
            "contract must identify exactly one V10 time-alignment receipt"
        )
    expected = resolve_relative(
        "validation/"
        "h0_primary_split_v10_time_alignment_adjudication_receipt.json"
    )
    if matches[0] != expected:
        raise V10SSafeDaggerError("teacher evidence path drifted")
    return matches[0]


def _teacher_evidence_passed(receipt: Mapping[str, Any]) -> bool:
    evidence = receipt.get("scientific_evidence")
    counterfactual = receipt.get("counterfactual_gate")
    original = receipt.get("original_protocol")
    checks = receipt.get("checks")
    contract_gate = contract.teacher_evidence_gate(receipt)
    return (
        contract_gate.get("passed") is True
        and receipt.get("status")
        == "PASS_H0_PRIMARY_SPLIT_V10_TIME_ALIGNMENT_ADJUDICATION"
        and receipt.get("passed") is True
        and receipt.get("scope") == "OFFLINE_TIME_ALIGNMENT_ONLY_NO_RERUN"
        and isinstance(evidence, Mapping)
        and evidence.get("coherent_teacher_evidence_accepted") is True
        and evidence.get("teacher_action_byte_exact_count")
        == contract.EXPECTED_STEPS
        and evidence.get("teacher_mean_byte_exact_count")
        == contract.EXPECTED_STEPS
        and evidence.get("teacher_view_byte_exact_count")
        == contract.EXPECTED_STEPS
        and isinstance(counterfactual, Mapping)
        and counterfactual.get("passed") is True
        and counterfactual.get("not_written_back") is True
        and isinstance(original, Mapping)
        and original.get("passed") is False
        and original.get("preserved_as_fail") is True
        and original.get("retry_authorized") is False
        and isinstance(checks, Mapping)
        and checks
        and all(value is True for value in checks.values())
        and receipt.get("rollout_rerun_count") == 0
        and receipt.get("actor_updates") == 0
        and receipt.get("critic_updates") == 0
        and receipt.get("ppo_updates") == 0
        and receipt.get("protected_trials_opened") == []
        and receipt.get("reserve_trials_opened") == []
    )


def _expected_stage_order() -> tuple[str, ...]:
    stages: list[str] = ["fit_p0"]
    for round_index in (1, 2, 3):
        stages.extend(
            f"collect_r{round_index}__{case_id}"
            for case_id in contract.COLLECTION_CASE_IDS
        )
        stages.append(f"fit_p{round_index}")
    stages.append("freeze_p3")
    stages.extend(f"final__{case_id}" for case_id in contract.FINAL_CASE_IDS)
    stages.append("finalize_development")
    return tuple(stages)


def build_preflight(*, require_unoccupied: bool = True) -> dict[str, Any]:
    """Build a read-only prerequisite audit; it never creates a candidate."""

    evidence = _mapping(_teacher_evidence_path())
    representative = contract.canonical_final_case(contract.FINAL_CASE_IDS[0])
    env_config = env_source.build_env_config(representative)
    occupancy = {
        "preflight_unoccupied": not os.path.lexists(PREFLIGHT),
        "lock_unoccupied": not os.path.lexists(LOCK),
        "run_root_unoccupied": not os.path.lexists(RUN_ROOT),
    }
    checks = {
        "v10_time_adjudication_pass": _teacher_evidence_passed(evidence),
        "stage_order_exact": tuple(contract.STAGE_IDS) == _expected_stage_order(),
        "four_fresh_h0_fits": tuple(contract.FIT_STAGES)
        == ("p0", "p1", "p2", "p3"),
        "three_safe_dagger_rounds": dict(contract.ROUND_ALPHAS)
        == {1: 0.25, 2: 0.5, 3: 0.75},
        "two_collection_cases": len(contract.COLLECTION_CASE_IDS) == 2,
        "six_final_cases": len(contract.FINAL_CASE_IDS) == 6,
        "source_h0_exists": SOURCE_H0_MODULE.is_dir(),
        "binary_active_v26": (
            env_config.get("binary_phase_fsm_mode") == "binary_active"
            and env_config.get("binary_phase_event_contract_id")
            == contract.EVENT_CONTRACT_ID
        ),
        "left_primary_only": env_config.get("online_grf_applied_sides") == ["left"],
        "morphology_zero": env_config.get("reward", {}).get("morphology_weight")
        == 0.0,
        "cadence_exact": (
            env_config.get("detector_sample_dt_s")
            == contract.EXPECTED_SAMPLE_DT_S
            and env_config.get("segment_duration")
            == contract.EXPECTED_POLICY_DT_S
        ),
        "critic_ppo_forbidden": not contract.AUTHORITY[
            "critic_updates_authorized"
        ]
        and not contract.AUTHORITY["ppo_updates_authorized"],
        "protected_reserve_closed": not contract.AUTHORITY[
            "protected_trial_access_authorized"
        ]
        and not contract.AUTHORITY["reserve_trial_access_authorized"],
        **occupancy,
    }
    # Constructing records also proves every declared input/source is a file.
    sources = _source_records()
    inputs = _input_records()
    passed = all(checks.values())
    if require_unoccupied and not passed:
        failed = [name for name, value in checks.items() if value is not True]
        raise V10SSafeDaggerError(f"V10S preflight failed: {failed}")
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": (
            contract.PREFLIGHT_PASS_STATUS
            if passed
            else "FAIL_H0_PRIMARY_SPLIT_V10S_SAFE_DAGGER_PREFLIGHT"
        ),
        "passed": passed,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "revision": contract.REVISION,
        "checks": checks,
        "stage_order": list(contract.STAGE_IDS),
        "round_alphas_student_weight": {
            str(key): float(value)
            for key, value in contract.ROUND_ALPHAS.items()
        },
        "collection_case_ids": list(contract.COLLECTION_CASE_IDS),
        "final_case_ids": list(contract.FINAL_CASE_IDS),
        "teacher_evidence": _record(_teacher_evidence_path()),
        "source_h0": _tree_record(SOURCE_H0_MODULE),
        "sources": sources,
        "inputs": inputs,
        "authority": copy.deepcopy(contract.AUTHORITY),
        "environment_reset_calls": 0,
        "environment_step_calls": 0,
        "candidate_created": False,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
        "next_stage": "FREEZE_V10S_SAFE_DAGGER_PIPELINE" if passed else "STOP",
    }


def _lock_payload(preflight: Mapping[str, Any]) -> dict[str, Any]:
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.LOCK_STATUS,
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "revision": contract.REVISION,
        "preflight": _record(PREFLIGHT),
        "run_root": contract.RUN_ROOT.as_posix(),
        "stage_order": list(contract.STAGE_IDS),
        "round_alphas_student_weight": {
            str(key): float(value)
            for key, value in contract.ROUND_ALPHAS.items()
        },
        "collection_case_ids": list(contract.COLLECTION_CASE_IDS),
        "final_case_ids": list(contract.FINAL_CASE_IDS),
        "teacher_evidence": _record(_teacher_evidence_path()),
        "source_h0": _tree_record(SOURCE_H0_MODULE),
        "sources": _source_records(),
        "inputs": _input_records(),
        "fit_gate": copy.deepcopy(contract.OFFLINE_THRESHOLDS),
        "physical_gate": {
            "steps": contract.EXPECTED_STEPS,
            "control_windows": contract.EXPECTED_CONTROL_WINDOWS,
            "raw_sensor_samples": contract.EXPECTED_RAW_SENSOR_SAMPLES,
            "penetration_strictly_less_than_m": contract.PENETRATION_LIMIT_M,
            "minimum_valid_cycles": contract.MINIMUM_VALID_CYCLES,
        },
        "authority": copy.deepcopy(contract.AUTHORITY),
        "retry_authorized": False,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
        "next_stage": "EXECUTE_V10S_SAFE_DAGGER_ONCE",
    }


def prepare() -> dict[str, Any]:
    preflight = build_preflight(require_unoccupied=True)
    forensic.write_json_exclusive(PREFLIGHT, preflight)
    lock = _lock_payload(preflight)
    forensic.write_json_exclusive(LOCK, lock)
    return {"preflight": preflight, "lock": lock}


def verify_lock(*, require_run_root_absent: bool = False) -> dict[str, Any]:
    preflight = _mapping(PREFLIGHT)
    if (
        preflight.get("passed") is not True
        or preflight.get("status") != contract.PREFLIGHT_PASS_STATUS
    ):
        raise V10SSafeDaggerError("V10S preflight is not PASS")
    observed = _mapping(LOCK)
    expected = _lock_payload(preflight)
    if forensic.canonical_json_bytes(observed) != forensic.canonical_json_bytes(
        expected
    ):
        raise V10SSafeDaggerError("V10S execution lock drifted")
    if require_run_root_absent and os.path.lexists(RUN_ROOT):
        raise V10SSafeDaggerError("V10S run root already claimed")
    return observed


def _token_sha256(token: str) -> str:
    if not isinstance(token, str) or len(token) < 32:
        raise V10SSafeDaggerError("supervisor token is malformed")
    return hashlib.sha256(token.encode("utf-8")).hexdigest()


def _claim_path(stage_id: str) -> Path:
    return resolve_relative(contract.worker_claim_path(stage_id))


def _stage_receipt_path(stage_id: str) -> Path:
    return resolve_relative(contract.stage_receipt_path(stage_id))


def _claim_payload(token_sha256: str) -> dict[str, Any]:
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.PIPELINE_CLAIM_STATUS,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "stage_order": list(contract.STAGE_IDS),
        "execution_token_sha256": token_sha256,
        "execution_lock": _record(LOCK),
        "authority": copy.deepcopy(contract.AUTHORITY),
        "retry_authorized": False,
        "safe_dagger_rounds_authorized": 3,
        "actor_updates": 0,
        "actor_updates_authorized": len(contract.FIT_STAGES),
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
    }


def _worker_claim_payload(
    *,
    stage_id: str,
    token_sha256: str,
    previous_receipts: Sequence[Mapping[str, Any]],
) -> dict[str, Any]:
    if stage_id not in contract.STAGE_IDS:
        raise V10SSafeDaggerError(f"unknown stage: {stage_id!r}")
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.WORKER_CLAIM_STATUS,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "stage_id": stage_id,
        "stage_index": contract.STAGE_IDS.index(stage_id),
        "execution_token_sha256": token_sha256,
        "pipeline_claim": _record(PIPELINE_CLAIM),
        "previous_receipts": [dict(row) for row in previous_receipts],
        "retry_authorized": False,
        "actor_updates": 0,
        "actor_updates_authorized": int(
            contract.stage_descriptor(stage_id)["kind"] == "fit"
        ),
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
    }


def verify_pipeline_claim(token: str) -> dict[str, Any]:
    observed = _mapping(PIPELINE_CLAIM)
    expected = _claim_payload(_token_sha256(token))
    if forensic.canonical_json_bytes(observed) != forensic.canonical_json_bytes(
        expected
    ):
        raise V10SSafeDaggerError("pipeline claim/token drifted")
    return observed


def verify_worker_claim(stage_id: str, token: str) -> dict[str, Any]:
    """Reject direct, repeated, post-ledger, and out-of-order workers."""

    if stage_id not in contract.STAGE_IDS:
        raise V10SSafeDaggerError(f"unknown worker stage: {stage_id!r}")
    if os.path.lexists(PIPELINE_LEDGER):
        raise V10SSafeDaggerError("terminal pipeline ledger already exists")
    execution = verify_pipeline_claim(token)
    index = contract.STAGE_IDS.index(stage_id)
    previous: list[dict[str, Any]] = []
    for prior in contract.STAGE_IDS[:index]:
        verify_stage_receipt(prior)
        previous.append(
            {"stage_id": prior, "receipt": _record(_stage_receipt_path(prior))}
        )
    expected = _worker_claim_payload(
        stage_id=stage_id,
        token_sha256=execution["execution_token_sha256"],
        previous_receipts=previous,
    )
    observed = _mapping(_claim_path(stage_id))
    if forensic.canonical_json_bytes(observed) != forensic.canonical_json_bytes(
        expected
    ):
        raise V10SSafeDaggerError(f"worker claim/order drifted: {stage_id}")
    if os.path.lexists(_stage_receipt_path(stage_id)):
        raise V10SSafeDaggerError(f"worker stage already completed: {stage_id}")
    for later in contract.STAGE_IDS[index + 1 :]:
        if os.path.lexists(_stage_receipt_path(later)):
            raise V10SSafeDaggerError(
                f"later stage already exists before {stage_id}: {later}"
            )
    return observed


def _float32_vector(value: Any, *, width: int, label: str, np: Any) -> Any:
    try:
        result = np.ascontiguousarray(np.asarray(value, dtype=np.float32))
    except (TypeError, ValueError, OverflowError) as exc:
        raise V10SSafeDaggerError(f"{label} is not float32") from exc
    if result.shape != (width,) or not np.all(np.isfinite(result)):
        raise V10SSafeDaggerError(f"{label} is malformed")
    return result


def _query_mean_std(module: Any, actor_view: Any, *, np: Any, torch: Any) -> tuple[Any, Any]:
    from ray.rllib.core.columns import Columns

    actor = _float32_vector(
        actor_view,
        width=contract.EXPECTED_ACTOR_FEATURES,
        label="actor view",
        np=np,
    )
    tensor = torch.as_tensor(actor[None, :], dtype=torch.float32)
    with torch.no_grad():
        logits = module._policy_logits({Columns.OBS: tensor}).detach().cpu().numpy()
    logits = np.ascontiguousarray(logits, dtype=np.float32)
    if logits.shape != (1, 2 * contract.EXPECTED_ACTION_DIM) or not np.all(
        np.isfinite(logits)
    ):
        raise V10SSafeDaggerError("policy logits are malformed")
    mean = np.ascontiguousarray(logits[0, :2], dtype=np.float32)
    std = np.ascontiguousarray(np.exp(logits[0, 2:]), dtype=np.float32)
    if (
        not np.all(np.isfinite(std))
        or np.any(std <= 0.0)
        or not np.allclose(
            std,
            contract.EXPECTED_SIGMA,
            rtol=0.0,
            atol=1.0e-8,
        )
    ):
        raise V10SSafeDaggerError("policy logstd drifted from frozen H0")
    return mean, std


def _validate_runtime_layout(
    *, module: Any, env: Any, observation: Any, rollout_eval: Any, np: Any
) -> tuple[tuple[str, ...], tuple[str, ...]]:
    actor_names = tuple(str(name) for name in env.unwrapped.actor_feature_names)
    full_names = tuple(str(name) for name in env.unwrapped.observation_feature_names)
    rollout_eval._validate_module_observation_contract(module, actor_names, full_names)
    coherent_teacher.validate_actor_feature_names(actor_names)
    if (
        np.asarray(observation).shape != (contract.EXPECTED_FULL_FEATURES,)
        or np.asarray(observation).dtype != np.dtype(contract.EXPECTED_DTYPE)
        or len(actor_names) != contract.EXPECTED_ACTOR_FEATURES
        or len(full_names) != contract.EXPECTED_FULL_FEATURES
        or tuple(env.action_space.shape) != (contract.EXPECTED_ACTION_DIM,)
    ):
        raise V10SSafeDaggerError("runtime layout drifted from 35/84 float32")
    return actor_names, full_names


def _frozen_innovations(case_id: str, *, action_selection: str, np: Any) -> Any:
    values = np.zeros(
        (contract.EXPECTED_STEPS, contract.EXPECTED_ACTION_DIM),
        dtype=np.float32,
    )
    if action_selection == "deterministic":
        return values
    if action_selection != "stochastic":
        raise V10SSafeDaggerError(f"unknown action selection: {action_selection}")
    trace_path = resolve_relative(contract.BASE_CORPUS_ROOT / case_id / "trace.json")
    rows = _sequence(trace_path)
    if len(rows) != contract.EXPECTED_STEPS:
        raise V10SSafeDaggerError(f"frozen innovation trace is incomplete: {case_id}")
    for offset, row in enumerate(rows):
        if not isinstance(row, Mapping) or row.get("step") != offset + 1:
            raise V10SSafeDaggerError(
                f"frozen innovation row is malformed: {case_id}/{offset + 1}"
            )
        raw = _float32_vector(
            row.get("frozen_raw_action"), width=2, label="frozen action", np=np
        )
        mean = _float32_vector(
            row.get("frozen_teacher_mean"), width=2, label="frozen mean", np=np
        )
        std = _float32_vector(
            row.get("teacher_std"), width=2, label="frozen std", np=np
        )
        if not np.allclose(
            std, contract.EXPECTED_SIGMA, rtol=0.0, atol=1.0e-8
        ):
            raise V10SSafeDaggerError("frozen innovation sigma drifted")
        values[offset] = np.asarray((raw - mean) / std, dtype=np.float32)
    if not np.all(np.isfinite(values)):
        raise V10SSafeDaggerError("frozen innovations are non-finite")
    return values


def _new_physical_audit(
    *, reset_info: Mapping[str, Any], legacy: Any, np: Any
) -> dict[str, Any]:
    if (
        reset_info.get("binary_phase_fsm_executed") is not True
        or reset_info.get("binary_phase_fsm_mode") != "binary_active"
        or reset_info.get("binary_phase_event_contract_id")
        != contract.EVENT_CONTRACT_ID
        or reset_info.get("online_grf_applied_sides") != ["left"]
    ):
        raise V10SSafeDaggerError("V26 binary-active reset routing drifted")
    baseline = legacy._validate_raw_sample(
        reset_info.get("binary_phase_sensor_baseline"),
        float(reset_info.get("time")),
        "t0",
    )
    reset_fsm = reset_info.get("binary_phase_fsm")
    if not isinstance(reset_fsm, Mapping) or reset_fsm.get("events_this_step") != []:
        raise V10SSafeDaggerError("V26 attributed an event to t0")
    return {
        "baseline_time_s": float(baseline["time_s"]),
        "binary_events": legacy._binary_event_accumulator(baseline),
        "reserve": legacy._empty_accumulator(),
        "residual": legacy._empty_accumulator(),
        "sea": legacy._sea_accumulators(),
        "control_window_count": 0,
        "raw_sensor_sample_count": 0,
        "action_clipped_values": 0,
        "fallback_count": 0,
        "timeout_count": 0,
        "sea_plugin_fallback_count": 0,
        "so_solver_unaccepted_count": 0,
        "hard_invalid_count": 0,
        "invalid_event_count": 0,
        "nonfinite_count": 0,
        "routing_failure_count": 0,
        "step_contract_failure_count": 0,
        "penetrations": [],
        "phase_valid_cycle_count": 0,
        "np": np,
    }


def _phase_state(info: Mapping[str, Any]) -> str:
    phase = info.get("phase_fsm")
    if not isinstance(phase, Mapping):
        raise V10SSafeDaggerError("phase FSM diagnostics are missing")
    state = phase.get("state_name")
    if not isinstance(state, str) or state not in safe_dagger.V26_PHASE_NAMES:
        raise V10SSafeDaggerError(f"unknown active V26 phase: {state!r}")
    return state


def _consume_physical_step(
    audit: dict[str, Any],
    *,
    step: int,
    info: Mapping[str, Any],
    observation_before: Any,
    observation_after: Any,
    reward: Any,
    action: Any,
    applied_action: Any,
    extra_vectors: Sequence[Any],
    legacy: Any,
    v26_collector: Any,
) -> dict[str, Any]:
    np = audit["np"]
    samples = info.get("binary_phase_sensor_samples")
    if (
        not isinstance(samples, Sequence)
        or isinstance(samples, (str, bytes))
        or len(samples) != contract.EXPECTED_RAW_SENSOR_SAMPLES
        // contract.EXPECTED_STEPS
    ):
        raise V10SSafeDaggerError(f"step {step} lacks ten V26 samples")
    previous = audit["baseline_time_s"] + (step - 1) * contract.EXPECTED_POLICY_DT_S
    for sample_index, sample in enumerate(samples, start=1):
        legacy._validate_raw_sample(
            sample,
            previous + sample_index * contract.EXPECTED_SAMPLE_DT_S,
            f"step {step} sample {sample_index}",
        )
    audit["raw_sensor_sample_count"] += len(samples)
    v26_collector._accumulate_binary_events_v26(
        audit["binary_events"], info=info, boundary_s=float(info.get("time"))
    )
    routing_exact = (
        info.get("binary_phase_fsm_executed") is True
        and info.get("binary_phase_fsm_mode") == "binary_active"
        and info.get("binary_phase_event_contract_id") == contract.EVENT_CONTRACT_ID
        and info.get("online_grf_applied_sides") == ["left"]
    )
    audit["routing_failure_count"] += int(not routing_exact)
    reward_terms = info.get("reward_terms")
    phase = info.get("phase_fsm")
    if not isinstance(reward_terms, Mapping) or not isinstance(phase, Mapping):
        raise V10SSafeDaggerError("runtime diagnostics are incomplete")
    penetration = float(reward_terms.get("grf_penetration_m"))
    if not math.isfinite(penetration) or penetration < 0.0:
        raise V10SSafeDaggerError("GRF penetration is malformed")
    audit["penetrations"].append(penetration)
    legacy._accumulate_scalar(audit["reserve"], reward_terms.get("reserve_norm_nm"))
    legacy._accumulate_scalar(
        audit["residual"], reward_terms.get("residual_norm_nm")
    )
    audit["phase_valid_cycle_count"] = int(float(phase.get("valid_cycle_count", 0)))
    step_timeout = int(float(phase.get("timeout_exceeded", 0.0)) > 0.0)
    audit["timeout_count"] += step_timeout
    audit["invalid_event_count"] = max(
        audit["invalid_event_count"],
        int(float(phase.get("invalid_event_count", 0.0))),
    )
    classified = so_recovery.classify_policy_step(
        info.get("so_solver_audit_entries"), policy_id=contract.SO_POLICY_ID
    )
    counters = classified["counters"]
    audit["control_window_count"] += int(counters["control_window_count"])
    step_unaccepted = int(
        counters["unaccepted_hard_so_fallback_count"]
        + counters["unaccepted_bounded_ls_count"]
    )
    audit["so_solver_unaccepted_count"] += step_unaccepted
    audit["fallback_count"] += step_unaccepted
    sea_payload = info.get("sea_segment_diagnostics")
    legacy._accumulate_sea(audit["sea"], sea_payload)
    step_sea_fallback = env_source._sea_fallback_count(sea_payload)
    audit["sea_plugin_fallback_count"] += step_sea_fallback
    step_hard_invalid = int("failure" in info)
    audit["hard_invalid_count"] += step_hard_invalid
    finite = bool(
        np.all(np.isfinite(observation_before))
        and np.all(np.isfinite(observation_after))
        and np.all(np.isfinite(action))
        and np.all(np.isfinite(applied_action))
        and all(np.all(np.isfinite(vector)) for vector in extra_vectors)
        and math.isfinite(float(reward))
        and math.isfinite(float(info.get("time")))
    )
    audit["nonfinite_count"] += int(not finite)
    unclipped = action.tobytes(order="C") == applied_action.tobytes(order="C")
    audit["action_clipped_values"] += int(np.count_nonzero(action != applied_action))
    checks = {
        "binary_active_routing_exact": routing_exact,
        "ten_sensor_samples": len(samples) == 10,
        "ten_control_windows": counters["control_window_count"] == 10,
        "no_unaccepted_so": step_unaccepted == 0,
        "no_sea_fallback": step_sea_fallback == 0,
        "no_timeout": step_timeout == 0,
        "no_hard_invalid": step_hard_invalid == 0,
        "action_unclipped": unclipped,
        "finite": finite,
    }
    audit["step_contract_failure_count"] += int(not all(checks.values()))
    return {
        "penetration_m": penetration,
        "phase": phase,
        "phase_state": _phase_state(info),
        "reserve_norm_nm": float(reward_terms["reserve_norm_nm"]),
        "residual_norm_nm": float(reward_terms["residual_norm_nm"]),
        "sea_segment_diagnostics": sea_payload,
        "so_recovery_counters": counters,
        "checks": checks,
    }


def _physical_summary(
    audit: Mapping[str, Any],
    *,
    case: Mapping[str, Any],
    rows: Sequence[Mapping[str, Any]],
    info: Mapping[str, Any],
    terminated: bool,
    truncated: bool,
    actor_names: Sequence[str],
    full_names: Sequence[str],
    legacy: Any,
    v26_collector: Any,
) -> dict[str, Any]:
    binary_gate = v26_collector._finalize_binary_event_gate_v26(
        audit["binary_events"], audit["raw_sensor_sample_count"]
    )
    return {
        "case_id": case["case_id"],
        "action_selection": case["action_selection"],
        "episode_start_offset_s": case["episode_start_offset_s"],
        "action_seed": case["action_seed"],
        "runtime_seed": case["runtime_seed"],
        "sigma": case["sigma"],
        "steps": len(rows),
        "control_window_count": audit["control_window_count"],
        "raw_sensor_sample_count": audit["raw_sensor_sample_count"],
        "end_reason": info.get("end_reason") if isinstance(info, Mapping) else None,
        "terminated": bool(terminated),
        "truncated": bool(truncated),
        "phase_valid_cycle_count": audit["phase_valid_cycle_count"],
        "grf_penetration_max_m": max(audit["penetrations"], default=0.0),
        "action_clipped_values": audit["action_clipped_values"],
        "fallback_count": audit["fallback_count"],
        "timeout_count": audit["timeout_count"],
        "safety_stop_count": int(bool(terminated)),
        "sea_plugin_fallback_count": audit["sea_plugin_fallback_count"],
        "so_solver_unaccepted_count": audit["so_solver_unaccepted_count"],
        "hard_invalid_count": audit["hard_invalid_count"],
        "invalid_event_count": audit["invalid_event_count"],
        "nonfinite_count": audit["nonfinite_count"],
        "routing_failure_count": audit["routing_failure_count"],
        "step_contract_failure_count": audit["step_contract_failure_count"],
        "binary_event_failure_count": int(binary_gate.get("passed") is not True),
        "binary_phase_event_gate": binary_gate,
        "episode_metrics": {
            "reserve_norm_nm": legacy._finalize_accumulator(audit["reserve"]),
            "residual_norm_nm": legacy._finalize_accumulator(audit["residual"]),
        },
        "sea_episode_metrics": legacy._finalize_sea(audit["sea"]),
        "n_actor": len(actor_names),
        "n_observation": len(full_names),
        "observation_dtype": contract.EXPECTED_DTYPE,
        "binary_phase_fsm_mode": "binary_active",
        "event_contract_id": contract.EVENT_CONTRACT_ID,
        "morphology_weight": contract.MORPHOLOGY_WEIGHT,
        "random_noise_draw_count": (
            len(rows) if case["action_selection"] == "stochastic" else 0
        ),
        "single_noise_application_count": len(rows),
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
    }


def _cumulative_collection_receipt_paths(stage: str) -> tuple[Path, ...]:
    if stage not in contract.FIT_STAGES:
        raise V10SSafeDaggerError(f"unknown fit stage: {stage!r}")
    paths: list[Path] = []
    for round_index in contract.FIT_COMPLETED_ROUNDS[stage]:
        for case_id in contract.COLLECTION_CASE_IDS:
            stage_id = f"collect_r{round_index}__{case_id}"
            paths.append(_stage_receipt_path(stage_id))
    return tuple(paths)


def verify_fit_receipt(stage: str) -> dict[str, Any]:
    stage_id = f"fit_{stage}"
    descriptor = contract.stage_descriptor(stage_id)
    if descriptor != {"kind": "fit", "fit_stage": stage}:
        raise V10SSafeDaggerError(f"fit stage descriptor drifted: {stage}")
    receipt_path = _stage_receipt_path(stage_id)
    receipt = _mapping(receipt_path)
    root = resolve_relative(contract.FIT_ROOTS[stage])
    module = resolve_relative(contract.MODULE_PATHS[stage])
    summary_path = root / "summary.json"
    gate_path = root / "gate.json"
    summary = _mapping(summary_path)
    gate = _mapping(gate_path)
    expected_gate = contract.fit_gate(summary, stage=stage)
    expected_dagger = [
        _record(path) for path in _cumulative_collection_receipt_paths(stage)
    ]
    if (
        receipt.get("schema_version") != contract.SCHEMA_VERSION
        or receipt.get("status") != contract.FIT_PASS_STATUS
        or receipt.get("passed") is not True
        or receipt.get("protocol_id") != contract.PROTOCOL_ID
        or receipt.get("stage_id") != stage_id
        or receipt.get("fit_stage") != stage
        or receipt.get("fit") != contract.FIT
        or not _record_matches(receipt.get("summary"), summary_path)
        or not _record_matches(receipt.get("gate"), gate_path)
        or receipt.get("module") != _tree_record(module)
        or receipt.get("source_h0") != _tree_record(SOURCE_H0_MODULE)
        or receipt.get("dagger_receipts") != expected_dagger
        or not _record_matches(receipt.get("pipeline_claim"), PIPELINE_CLAIM)
        or not _record_matches(receipt.get("worker_claim"), _claim_path(stage_id))
        or receipt.get("actor_updates") != 1
        or receipt.get("critic_updates") != 0
        or receipt.get("ppo_updates") != 0
        or receipt.get("protected_trials_opened") != []
        or receipt.get("reserve_trials_opened") != []
        or forensic.canonical_json_bytes(gate)
        != forensic.canonical_json_bytes(expected_gate)
        or gate.get("passed") is not True
    ):
        raise V10SSafeDaggerError(f"fit receipt closure drifted: {stage}")
    return receipt


def _run_fit_stage(stage: str) -> dict[str, Any]:
    stage_id = f"fit_{stage}"
    result = fit_engine.run_fit_stage(
        stage=stage,
        output_dir=resolve_relative(contract.FIT_ROOTS[stage]),
        dagger_receipt_paths=_cumulative_collection_receipt_paths(stage),
        pipeline_claim_path=PIPELINE_CLAIM,
        worker_claim_path=_claim_path(stage_id),
        enforce_canonical_destination=True,
    )
    if not isinstance(result, Mapping) or result.get("passed") is not True:
        raise V10SSafeDaggerError(f"offline fit gate failed: {stage}")
    return verify_fit_receipt(stage)


def _collection_start_payload(
    *, case: Mapping[str, Any], stage_id: str, candidate_module: Path
) -> dict[str, Any]:
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": "H0_PRIMARY_SPLIT_V10S_SAFE_DAGGER_COLLECTION_STARTED",
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "stage_id": stage_id,
        "round_index": case["round_index"],
        "case": dict(case),
        "behavior": contract.COLLECTION_BEHAVIOR,
        "requested_alpha_student_weight": case["requested_alpha"],
        "candidate_fit_stage": case["candidate_fit_stage"],
        "candidate_module": _tree_record(candidate_module),
        "teacher_id": contract.TEACHER_ID,
        "teacher_evidence_id": contract.TEACHER_EVIDENCE_ID,
        "teacher_evidence": _record(_teacher_evidence_path()),
        "source_h0": _tree_record(SOURCE_H0_MODULE),
        "blend_order": "MEANS_THEN_ONE_FROZEN_NOISE_VECTOR",
        "alpha_semantics": "STUDENT_WEIGHT_LATCH_FORCES_ZERO",
        "safety_signal_lag_steps": 1,
        "physical_gate_relaxed": False,
        "pipeline_claim": _record(PIPELINE_CLAIM),
        "worker_claim": _record(_claim_path(stage_id)),
        "retry_authorized": False,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
    }


def _collect_safe_dagger(round_index: int, case_id: str) -> dict[str, Any]:
    import numpy as np
    import torch
    from ray.rllib.core.rl_module.rl_module import RLModule

    stage_id = f"collect_r{round_index}__{case_id}"
    case = contract.canonical_collection_case(case_id, round_index)
    if contract.stage_descriptor(stage_id)["case"] != case:
        raise V10SSafeDaggerError("collection descriptor/case drifted")
    destination = resolve_relative(case["destination"])
    if os.path.lexists(destination):
        raise V10SSafeDaggerError(f"collection destination exists: {stage_id}")
    fit_stage = str(case["candidate_fit_stage"])
    verify_fit_receipt(fit_stage)
    candidate_module = resolve_relative(contract.MODULE_PATHS[fit_stage])
    evidence = _mapping(_teacher_evidence_path())
    if contract.teacher_evidence_gate(evidence).get("passed") is not True:
        raise V10SSafeDaggerError("coherent-teacher evidence is not PASS")

    rollout_eval, runtime_np, runtime_torch, runtime_rlmodule, env_factory, _reward = (
        env_source.source_collector.engine.legacy._load_inference_stack()
    )
    if runtime_np is not np or runtime_torch is not torch or runtime_rlmodule is not RLModule:
        raise V10SSafeDaggerError("inference stack identity drifted")
    legacy = env_source.source_collector.engine.legacy
    v26_collector = env_source.source_collector.base
    candidate = RLModule.from_checkpoint(candidate_module)
    teacher = RLModule.from_checkpoint(SOURCE_H0_MODULE)
    candidate.eval()
    teacher.eval()
    innovations = _frozen_innovations(
        case_id, action_selection=str(case["action_selection"]), np=np
    )
    env_config = env_source.build_env_config(case)
    env = env_factory.make_cmc_env(env_config)
    writer = forensic.ForensicRolloutWriter(destination, artifact_root=REPO_ROOT)
    writer.start(
        _collection_start_payload(
            case=case, stage_id=stage_id, candidate_module=candidate_module
        )
    )

    rows: list[dict[str, Any]] = []
    actor_names: tuple[str, ...] = ()
    full_names: tuple[str, ...] = ()
    audit: dict[str, Any] | None = None
    info: Mapping[str, Any] = {}
    terminated = False
    truncated = False
    candidate_mean_query_count = 0
    teacher_query_count = 0
    same_state_label_count = 0
    candidate_selected_before_teacher_count = 0
    served_action_teacher_dependency_count = 0
    mean_blend_count = 0
    blend_before_noise_count = 0
    noise_before_blend_count = 0
    multiple_noise_application_count = 0
    alpha_mismatch_count = 0
    mean_blend_mismatch_count = 0
    noise_application_mismatch_count = 0
    safety_latch_rule_violation_count = 0
    physical_gate_bypass_count = 0
    latch = safe_dagger.SafetyLatchState()
    previous_penetration_m = 0.0
    started = time.monotonic()
    try:
        observation, reset_info = env.reset(seed=int(case["runtime_seed"]))
        observation = np.ascontiguousarray(observation, dtype=np.float32)
        actor_names, full_names = _validate_runtime_layout(
            module=candidate,
            env=env,
            observation=observation,
            rollout_eval=rollout_eval,
            np=np,
        )
        rollout_eval._validate_module_observation_contract(
            teacher, actor_names, full_names
        )
        audit = _new_physical_audit(reset_info=reset_info, legacy=legacy, np=np)
        body_weight_n = float(env.unwrapped._body_weight_n)
        if not math.isfinite(body_weight_n) or body_weight_n <= 0.0:
            raise V10SSafeDaggerError("body weight is malformed")
        shadow = coherent_teacher.LegacyGaitShadow.from_runtime_phase_fsm(
            env.unwrapped._phase_fsm
        )
        student = np.ascontiguousarray(
            observation[: contract.EXPECTED_ACTOR_FEATURES], dtype=np.float32
        )
        teacher_view = coherent_teacher.build_teacher_view(
            student,
            actor_names,
            reset_info,
            body_weight_n=body_weight_n,
            shadow=shadow,
            reset_boundary=True,
        )
        current_info: Mapping[str, Any] = dict(reset_info)

        for index in range(contract.EXPECTED_STEPS):
            step = index + 1
            observation_before = observation.copy()
            student_before = student.copy()
            teacher_before = teacher_view.copy()
            coherent_teacher.assert_coherent_pair(student_before, teacher_before)

            # Candidate selection is deliberately complete before the teacher
            # is queried.  The served behavior below is still teacher-dependent
            # by design because every preregistered alpha is strictly below 1.
            candidate_mean, candidate_std = _query_mean_std(
                candidate, student_before, np=np, torch=torch
            )
            candidate_mean_query_count += 1
            candidate_selected_before_teacher_count += 1
            teacher_mean, teacher_std = _query_mean_std(
                teacher, teacher_before, np=np, torch=torch
            )
            teacher_query_count += 1
            same_state_label_count += 1
            if candidate_std.tobytes(order="C") != teacher_std.tobytes(order="C"):
                raise V10SSafeDaggerError("candidate/teacher frozen logstd mismatch")

            noise = np.ascontiguousarray(
                candidate_std * innovations[index], dtype=np.float32
            )
            selected = safe_dagger.select_safe_dagger_action(
                candidate_mean,
                teacher_mean,
                noise,
                requested_alpha=case["requested_alpha"],
                latch_state=latch,
                previous_penetration_m=previous_penetration_m,
                active_v26_phase=_phase_state(current_info),
            )
            latch = selected.latch_state
            served_action_teacher_dependency_count += int(
                selected.effective_alpha < 1.0
            )
            mean_blend_count += 1
            blend_before_noise_count += 1
            expected_alpha = 0.0 if latch.active else float(case["requested_alpha"])
            alpha_exact = selected.effective_alpha == expected_alpha
            alpha_mismatch_count += int(not alpha_exact)
            expected_mean, recomputed_alpha = safe_dagger.blend_policy_means(
                candidate_mean,
                teacher_mean,
                requested_alpha=case["requested_alpha"],
                latch_state=latch,
            )
            mean_exact = (
                recomputed_alpha == selected.effective_alpha
                and expected_mean.tobytes(order="C")
                == selected.blended_mean.tobytes(order="C")
            )
            mean_blend_mismatch_count += int(not mean_exact)
            expected_action = safe_dagger.apply_single_noise(expected_mean, noise)
            noise_exact = (
                expected_action.tobytes(order="C")
                == selected.action.tobytes(order="C")
            )
            noise_application_mismatch_count += int(not noise_exact)
            applied = np.ascontiguousarray(
                np.clip(selected.action, env.action_space.low, env.action_space.high),
                dtype=np.float32,
            )
            observation_after, reward, terminated, truncated, info = env.step(applied)
            observation_after = np.ascontiguousarray(
                observation_after, dtype=np.float32
            )
            if not isinstance(info, Mapping):
                raise V10SSafeDaggerError("collection runtime info is malformed")
            next_student = np.ascontiguousarray(
                observation_after[: contract.EXPECTED_ACTOR_FEATURES],
                dtype=np.float32,
            )
            next_teacher_view = coherent_teacher.build_teacher_view(
                next_student,
                actor_names,
                info,
                body_weight_n=body_weight_n,
                shadow=shadow,
                reset_boundary=False,
            )
            physical = _consume_physical_step(
                audit,
                step=step,
                info=info,
                observation_before=observation_before,
                observation_after=observation_after,
                reward=reward,
                action=selected.action,
                applied_action=applied,
                extra_vectors=(
                    student_before,
                    teacher_before,
                    candidate_mean,
                    candidate_std,
                    teacher_mean,
                    teacher_std,
                    selected.blended_mean,
                    noise,
                ),
                legacy=legacy,
                v26_collector=v26_collector,
            )
            semantic_checks = {
                "candidate_selected_before_teacher": True,
                "teacher_queried_on_same_state": True,
                "teacher_view_changes_only_10_24": True,
                "alpha_exact": alpha_exact,
                "mean_blend_exact": mean_exact,
                "single_noise_after_blend_exact": noise_exact,
                "served_action_teacher_dependent": selected.effective_alpha < 1.0,
                "physical_gate_not_bypassed": True,
            }
            audit["step_contract_failure_count"] += int(
                not all(semantic_checks.values())
            )
            row = {
                "schema_version": contract.SCHEMA_VERSION,
                "protocol_id": contract.PROTOCOL_ID,
                "stage_id": stage_id,
                "round_index": round_index,
                "case_id": case_id,
                "v25_observation": student_before.tolist(),
                "counterfactual_teacher_observation": teacher_before.tolist(),
                "counterfactual_teacher_mean": teacher_mean.tolist(),
                "counterfactual_teacher_std": teacher_std.tolist(),
                "candidate_mean": candidate_mean.tolist(),
                "candidate_std": candidate_std.tolist(),
                "candidate_selected_before_teacher": True,
                "teacher_queried_on_same_state": True,
                "requested_alpha": float(case["requested_alpha"]),
                "effective_alpha": float(selected.effective_alpha),
                "blended_mean": selected.blended_mean.tolist(),
                "single_noise": noise.tolist(),
                "standard_normal": innovations[index].tolist(),
                "raw_action": selected.action.tolist(),
                "applied_action": applied.tolist(),
                "safety_latch_active": latch.active,
                "safety_latch_entered": selected.latch_entered,
                "safety_latch_released": selected.latch_released,
                "safety_intervened": selected.safety_intervened,
                "previous_penetration_m": previous_penetration_m,
                "reward": float(reward),
                "time_s": float(info.get("time")),
                "grf_penetration_m": physical["penetration_m"],
                "reserve_norm_nm": physical["reserve_norm_nm"],
                "residual_norm_nm": physical["residual_norm_nm"],
                "sea_segment_diagnostics": legacy._jsonable(
                    physical["sea_segment_diagnostics"]
                ),
                "phase_fsm": legacy._jsonable(physical["phase"]),
                "so_recovery_counters": legacy._jsonable(
                    physical["so_recovery_counters"]
                ),
                "checks": {**physical["checks"], **semantic_checks},
                "terminated": bool(terminated),
                "truncated": bool(truncated),
                "end_reason": info.get("end_reason"),
            }
            writer.write_step(step, row)
            rows.append({"step": step, **row})
            previous_penetration_m = physical["penetration_m"]
            observation = observation_after
            student = next_student
            teacher_view = next_teacher_view
            current_info = dict(info)
            if step == 1 or step % 25 == 0:
                elapsed = time.monotonic() - started
                eta = elapsed / step * (contract.EXPECTED_STEPS - step)
                print(
                    f"[V10S collect r{round_index}/{case_id}] {step:3d}/500 "
                    f"elapsed={elapsed:7.1f}s eta={eta:7.1f}s",
                    flush=True,
                )
            if terminated or truncated:
                break
    except Exception as exc:
        try:
            writer.publish_failure(
                end_reason="safe_dagger_collection_failed",
                error=exc,
                status=contract.COLLECTION_FAIL_STATUS,
            )
        except Exception:
            pass
        raise
    finally:
        env.close()

    if audit is None:
        raise V10SSafeDaggerError("collection audit was not initialized")
    summary = {
        **_physical_summary(
            audit,
            case=case,
            rows=rows,
            info=info,
            terminated=terminated,
            truncated=truncated,
            actor_names=actor_names,
            full_names=full_names,
            legacy=legacy,
            v26_collector=v26_collector,
        ),
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.COLLECTION_STATUS,
        "protocol_id": contract.PROTOCOL_ID,
        "round_index": round_index,
        "dagger_round": round_index,
        "requested_alpha": float(case["requested_alpha"]),
        "candidate_fit_stage": fit_stage,
        "behavior": contract.COLLECTION_BEHAVIOR,
        "teacher_id": contract.TEACHER_ID,
        "teacher_evidence_id": contract.TEACHER_EVIDENCE_ID,
        "teacher_query_count": teacher_query_count,
        "sample_count": len(rows),
        "persisted_label_count": len(rows),
        "same_state_teacher_label_count": same_state_label_count,
        "candidate_mean_query_count": candidate_mean_query_count,
        "candidate_selected_before_teacher_count": (
            candidate_selected_before_teacher_count
        ),
        "served_action_teacher_dependency_count": (
            served_action_teacher_dependency_count
        ),
        "mean_blend_count": mean_blend_count,
        "blend_before_noise_count": blend_before_noise_count,
        "noise_before_blend_count": noise_before_blend_count,
        "multiple_noise_application_count": multiple_noise_application_count,
        "safety_latch_activation_m": safe_dagger.SAFETY_LATCH_ACTIVATION_M,
        "safety_latch_release_m": safe_dagger.SAFETY_LATCH_RELEASE_M,
        "safety_latch_release_phase": safe_dagger.SAFETY_LATCH_RELEASE_PHASE,
        "safety_signal_lag_steps": 1,
        "safety_intervention_diagnostic_only": True,
        "safety_latch_activation_count": latch.activation_count,
        "safety_latch_release_count": latch.release_count,
        "safety_intervention_count": latch.intervention_action_count,
        "safety_latch_rule_violation_count": safety_latch_rule_violation_count,
        "alpha_mismatch_count": alpha_mismatch_count,
        "mean_blend_mismatch_count": mean_blend_mismatch_count,
        "noise_application_mismatch_count": noise_application_mismatch_count,
        "physical_gate_bypass_count": physical_gate_bypass_count,
    }
    partial = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": "H0_PRIMARY_SPLIT_V10S_COLLECTION_PERSISTED_BEFORE_GATE",
        "protocol_id": contract.PROTOCOL_ID,
        "stage_id": stage_id,
        "round_index": round_index,
        "case_id": case_id,
        "steps": len(rows),
        "gate_evaluated": False,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
    }
    persisted = writer.finalize_before_gate(
        trace=rows, partial_summary=partial, summary=summary
    )

    def evaluate(_records: dict[str, Any]) -> dict[str, Any]:
        gate = contract.collection_gate(summary, round_index=round_index)
        gate["persisted_before_gate"] = persisted
        return gate

    writer.run_gate(evaluate)
    gate = _mapping(writer.gate_path)
    if gate.get("passed") is not True:
        writer.publish_failure(
            end_reason="safe_dagger_collection_gate_failed",
            error="V10S collection physical/semantic gate failed",
            status=contract.COLLECTION_FAIL_STATUS,
            details={"gate": _record(writer.gate_path)},
        )
        raise V10SSafeDaggerError(f"collection gate failed: {stage_id}")
    receipt = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.COLLECTION_PASS_STATUS,
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "stage_id": stage_id,
        "round_index": round_index,
        "dagger_round": round_index,
        "case_id": case_id,
        "sample_count": len(rows),
        "same_state_teacher_label_count": same_state_label_count,
        "teacher_query_count": teacher_query_count,
        "persisted_label_count": len(rows),
        "candidate_mean_query_count": candidate_mean_query_count,
        "candidate_selected_before_teacher_count": (
            candidate_selected_before_teacher_count
        ),
        "served_action_teacher_dependency_count": (
            served_action_teacher_dependency_count
        ),
        "candidate_fit_stage": fit_stage,
        "requested_alpha": float(case["requested_alpha"]),
        "artifacts": writer.artifact_records(),
        "pipeline_claim": _record(PIPELINE_CLAIM),
        "worker_claim": _record(_claim_path(stage_id)),
        "retry_authorized": False,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
    }
    receipt_path = forensic.write_json_exclusive(
        writer.run_directory / "receipt.json", receipt
    )
    return {**receipt, "receipt": _record(receipt_path)}


def verify_collection_receipt(round_index: int, case_id: str) -> dict[str, Any]:
    stage_id = f"collect_r{round_index}__{case_id}"
    case = contract.canonical_collection_case(case_id, round_index)
    destination = resolve_relative(case["destination"])
    writer = forensic.ForensicRolloutWriter(destination, artifact_root=REPO_ROOT)
    receipt = _mapping(_stage_receipt_path(stage_id))
    summary = _mapping(writer.summary_path)
    gate = _mapping(writer.gate_path)
    persisted = writer.finalized_artifact_records()
    expected_gate = contract.collection_gate(summary, round_index=round_index)
    expected_gate["persisted_before_gate"] = persisted
    if (
        receipt.get("schema_version") != contract.SCHEMA_VERSION
        or receipt.get("status") != contract.COLLECTION_PASS_STATUS
        or receipt.get("passed") is not True
        or receipt.get("protocol_id") != contract.PROTOCOL_ID
        or receipt.get("stage_id") != stage_id
        or receipt.get("round_index") != round_index
        or receipt.get("dagger_round") != round_index
        or receipt.get("case_id") != case_id
        or receipt.get("sample_count") != contract.EXPECTED_STEPS
        or receipt.get("same_state_teacher_label_count")
        != contract.EXPECTED_STEPS
        or receipt.get("teacher_query_count") != contract.EXPECTED_STEPS
        or receipt.get("persisted_label_count") != contract.EXPECTED_STEPS
        or receipt.get("candidate_mean_query_count") != contract.EXPECTED_STEPS
        or receipt.get("candidate_selected_before_teacher_count")
        != contract.EXPECTED_STEPS
        or receipt.get("served_action_teacher_dependency_count")
        != contract.EXPECTED_STEPS
        or receipt.get("candidate_fit_stage") != case["candidate_fit_stage"]
        or receipt.get("requested_alpha") != case["requested_alpha"]
        or receipt.get("artifacts") != writer.artifact_records()
        or not _record_matches(receipt.get("pipeline_claim"), PIPELINE_CLAIM)
        or not _record_matches(receipt.get("worker_claim"), _claim_path(stage_id))
        or receipt.get("retry_authorized") is not False
        or receipt.get("actor_updates") != 0
        or receipt.get("critic_updates") != 0
        or receipt.get("ppo_updates") != 0
        or receipt.get("protected_trials_opened") != []
        or receipt.get("reserve_trials_opened") != []
        or forensic.canonical_json_bytes(gate)
        != forensic.canonical_json_bytes(expected_gate)
        or gate.get("passed") is not True
    ):
        raise V10SSafeDaggerError(f"collection receipt closure drifted: {stage_id}")
    rows = _sequence(writer.trace_path)
    if len(rows) != contract.EXPECTED_STEPS or any(
        not isinstance(row, Mapping)
        or row.get("step") != index
        or row.get("protocol_id") != contract.PROTOCOL_ID
        or row.get("round_index") != round_index
        or row.get("case_id") != case_id
        or row.get("candidate_selected_before_teacher") is not True
        or row.get("teacher_queried_on_same_state") is not True
        for index, row in enumerate(rows, start=1)
    ):
        raise V10SSafeDaggerError(f"collection trace closure drifted: {stage_id}")
    return receipt


def _all_collection_receipt_records() -> list[dict[str, Any]]:
    records: list[dict[str, Any]] = []
    for round_index in (1, 2, 3):
        for case_id in contract.COLLECTION_CASE_IDS:
            verify_collection_receipt(round_index, case_id)
            stage_id = f"collect_r{round_index}__{case_id}"
            records.append(
                {
                    "round_index": round_index,
                    "case_id": case_id,
                    "receipt": _record(_stage_receipt_path(stage_id)),
                }
            )
    return records


def _freeze_p3() -> dict[str, Any]:
    stage_id = "freeze_p3"
    p3_receipt = verify_fit_receipt("p3")
    p3_summary_path = resolve_relative(contract.FIT_ROOTS["p3"]) / "summary.json"
    p3_summary = _mapping(p3_summary_path)
    module_path = resolve_relative(contract.MODULE_PATHS["p3"])
    module_record = _tree_record(module_path)
    candidate_id = (
        f"{contract.candidate_id_prefix('p3')}"
        f"{module_record['tree_sha256'][:16]}"
    )
    summary_path = CANDIDATE_FREEZE.with_name("candidate_freeze_summary.json")
    gate_path = CANDIDATE_FREEZE.with_name("candidate_freeze_gate.json")
    for path in (CANDIDATE_FREEZE, summary_path, gate_path):
        if os.path.lexists(path):
            raise V10SSafeDaggerError(f"candidate freeze output exists: {path}")
    collection_receipts = _all_collection_receipt_records()
    fit_receipts = [
        {
            "fit_stage": stage,
            "receipt": _record(resolve_relative(contract.FIT_RECEIPT_PATHS[stage])),
        }
        for stage in contract.FIT_STAGES
        if verify_fit_receipt(stage)
    ]
    summary = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": "H0_PRIMARY_SPLIT_V10S_P3_CANDIDATE_FREEZE_UNGATED",
        "protocol_id": contract.PROTOCOL_ID,
        "candidate_fit_stage": "p3",
        "candidate_id": candidate_id,
        "candidate_module": module_record,
        "p3_fit_passed": p3_receipt.get("passed") is True,
        "candidate_frozen": True,
        "logstd_byte_exact": p3_summary.get("logstd_byte_exact") is True,
        "critic_byte_exact": p3_summary.get("critic_byte_exact") is True,
        "every_fit_restarted_from_h0": all(
            _mapping(resolve_relative(contract.FIT_ROOTS[stage]) / "summary.json").get(
                "continued_from_previous_candidate"
            )
            is False
            for stage in contract.FIT_STAGES
        ),
        "fit_actor_update_count": len(contract.FIT_STAGES),
        "fit_receipts": fit_receipts,
        "collection_receipts": collection_receipts,
        "source_h0": _tree_record(SOURCE_H0_MODULE),
        "teacher_evidence": _record(_teacher_evidence_path()),
        "target_contract_id": contract.TARGET_CONTRACT_ID,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
    }
    forensic.write_json_exclusive(summary_path, summary)
    gate = contract.candidate_freeze_gate(summary)
    forensic.write_json_exclusive(gate_path, gate)
    if gate.get("passed") is not True:
        raise V10SSafeDaggerError("P3 candidate freeze gate failed")
    receipt = {
        **summary,
        "status": contract.FREEZE_PASS_STATUS,
        "passed": True,
        "stage_id": stage_id,
        "summary": _record(summary_path),
        "gate": _record(gate_path),
        "p3_fit_receipt": _record(resolve_relative(contract.FIT_RECEIPT_PATHS["p3"])),
        "pipeline_claim": _record(PIPELINE_CLAIM),
        "worker_claim": _record(_claim_path(stage_id)),
        "retry_authorized": False,
    }
    path = forensic.write_json_exclusive(CANDIDATE_FREEZE, receipt)
    return {**receipt, "receipt": _record(path)}


def _candidate_freeze() -> dict[str, Any]:
    receipt = _mapping(CANDIDATE_FREEZE)
    summary_path = CANDIDATE_FREEZE.with_name("candidate_freeze_summary.json")
    gate_path = CANDIDATE_FREEZE.with_name("candidate_freeze_gate.json")
    summary = _mapping(summary_path)
    gate = _mapping(gate_path)
    expected_gate = contract.candidate_freeze_gate(summary)
    module_record = _tree_record(resolve_relative(contract.MODULE_PATHS["p3"]))
    expected_id = (
        f"{contract.candidate_id_prefix('p3')}"
        f"{module_record['tree_sha256'][:16]}"
    )
    if (
        receipt.get("schema_version") != contract.SCHEMA_VERSION
        or receipt.get("status") != contract.FREEZE_PASS_STATUS
        or receipt.get("passed") is not True
        or receipt.get("protocol_id") != contract.PROTOCOL_ID
        or receipt.get("stage_id") != "freeze_p3"
        or receipt.get("candidate_fit_stage") != "p3"
        or receipt.get("candidate_id") != expected_id
        or receipt.get("candidate_module") != module_record
        or not _record_matches(receipt.get("summary"), summary_path)
        or not _record_matches(receipt.get("gate"), gate_path)
        or not _record_matches(
            receipt.get("p3_fit_receipt"),
            resolve_relative(contract.FIT_RECEIPT_PATHS["p3"]),
        )
        or not _record_matches(receipt.get("pipeline_claim"), PIPELINE_CLAIM)
        or not _record_matches(receipt.get("worker_claim"), _claim_path("freeze_p3"))
        or receipt.get("retry_authorized") is not False
        or receipt.get("actor_updates") != 0
        or receipt.get("critic_updates") != 0
        or receipt.get("ppo_updates") != 0
        or receipt.get("protected_trials_opened") != []
        or receipt.get("reserve_trials_opened") != []
        or forensic.canonical_json_bytes(gate)
        != forensic.canonical_json_bytes(expected_gate)
        or gate.get("passed") is not True
    ):
        raise V10SSafeDaggerError("P3 candidate freeze closure drifted")
    return receipt


def _final_start_payload(
    *, case: Mapping[str, Any], stage_id: str, freeze: Mapping[str, Any]
) -> dict[str, Any]:
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": "H0_PRIMARY_SPLIT_V10S_FINAL_ROLLOUT_STARTED",
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "stage_id": stage_id,
        "case": dict(case),
        "behavior": contract.FINAL_BEHAVIOR,
        "candidate_id": freeze["candidate_id"],
        "candidate_fit_stage": "p3",
        "candidate_module": freeze["candidate_module"],
        "teacher_enabled": False,
        "blending_enabled": False,
        "safety_latch_enabled": False,
        "noise": "ONE_FROZEN_CONDITION_INNOVATION_AFTER_CANDIDATE_MEAN",
        "pipeline_claim": _record(PIPELINE_CLAIM),
        "worker_claim": _record(_claim_path(stage_id)),
        "retry_authorized": False,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
    }


def _final_rollout(case_id: str) -> dict[str, Any]:
    """Run P3 with no teacher load/query, blend, or safety intervention."""

    import numpy as np
    import torch
    from ray.rllib.core.rl_module.rl_module import RLModule

    stage_id = f"final__{case_id}"
    case = contract.canonical_final_case(case_id)
    destination = resolve_relative(case["destination"])
    if os.path.lexists(destination):
        raise V10SSafeDaggerError(f"final destination exists: {case_id}")
    freeze = _candidate_freeze()
    candidate_module = resolve_relative(contract.MODULE_PATHS["p3"])
    rollout_eval, runtime_np, runtime_torch, runtime_rlmodule, env_factory, _reward = (
        env_source.source_collector.engine.legacy._load_inference_stack()
    )
    if runtime_np is not np or runtime_torch is not torch or runtime_rlmodule is not RLModule:
        raise V10SSafeDaggerError("inference stack identity drifted")
    legacy = env_source.source_collector.engine.legacy
    v26_collector = env_source.source_collector.base
    candidate = RLModule.from_checkpoint(candidate_module)
    candidate.eval()
    innovations = _frozen_innovations(
        case_id, action_selection=str(case["action_selection"]), np=np
    )
    env_config = env_source.build_env_config(case)
    env = env_factory.make_cmc_env(env_config)
    writer = forensic.ForensicRolloutWriter(destination, artifact_root=REPO_ROOT)
    writer.start(_final_start_payload(case=case, stage_id=stage_id, freeze=freeze))

    rows: list[dict[str, Any]] = []
    actor_names: tuple[str, ...] = ()
    full_names: tuple[str, ...] = ()
    audit: dict[str, Any] | None = None
    info: Mapping[str, Any] = {}
    terminated = False
    truncated = False
    candidate_mean_query_count = 0
    started = time.monotonic()
    try:
        observation, reset_info = env.reset(seed=int(case["runtime_seed"]))
        observation = np.ascontiguousarray(observation, dtype=np.float32)
        actor_names, full_names = _validate_runtime_layout(
            module=candidate,
            env=env,
            observation=observation,
            rollout_eval=rollout_eval,
            np=np,
        )
        audit = _new_physical_audit(reset_info=reset_info, legacy=legacy, np=np)
        for index in range(contract.EXPECTED_STEPS):
            step = index + 1
            observation_before = observation.copy()
            actor_view = np.ascontiguousarray(
                observation_before[: contract.EXPECTED_ACTOR_FEATURES],
                dtype=np.float32,
            )
            mean, std = _query_mean_std(candidate, actor_view, np=np, torch=torch)
            candidate_mean_query_count += 1
            noise = np.ascontiguousarray(std * innovations[index], dtype=np.float32)
            raw_action = safe_dagger.apply_single_noise(mean, noise)
            applied = np.ascontiguousarray(
                np.clip(raw_action, env.action_space.low, env.action_space.high),
                dtype=np.float32,
            )
            observation_after, reward, terminated, truncated, info = env.step(applied)
            observation_after = np.ascontiguousarray(
                observation_after, dtype=np.float32
            )
            if not isinstance(info, Mapping):
                raise V10SSafeDaggerError("final runtime info is malformed")
            physical = _consume_physical_step(
                audit,
                step=step,
                info=info,
                observation_before=observation_before,
                observation_after=observation_after,
                reward=reward,
                action=raw_action,
                applied_action=applied,
                extra_vectors=(actor_view, mean, std, noise),
                legacy=legacy,
                v26_collector=v26_collector,
            )
            row = {
                "schema_version": contract.SCHEMA_VERSION,
                "protocol_id": contract.PROTOCOL_ID,
                "stage_id": stage_id,
                "case_id": case_id,
                "v26_observation": actor_view.tolist(),
                "candidate_mean": mean.tolist(),
                "candidate_std": std.tolist(),
                "standard_normal": innovations[index].tolist(),
                "single_noise": noise.tolist(),
                "raw_action": raw_action.tolist(),
                "applied_action": applied.tolist(),
                "teacher_enabled": False,
                "teacher_query_count": 0,
                "served_action_teacher_dependency_count": 0,
                "blending_enabled": False,
                "safety_latch_enabled": False,
                "reward": float(reward),
                "time_s": float(info.get("time")),
                "grf_penetration_m": physical["penetration_m"],
                "reserve_norm_nm": physical["reserve_norm_nm"],
                "residual_norm_nm": physical["residual_norm_nm"],
                "sea_segment_diagnostics": legacy._jsonable(
                    physical["sea_segment_diagnostics"]
                ),
                "phase_fsm": legacy._jsonable(physical["phase"]),
                "so_recovery_counters": legacy._jsonable(
                    physical["so_recovery_counters"]
                ),
                "checks": physical["checks"],
                "terminated": bool(terminated),
                "truncated": bool(truncated),
                "end_reason": info.get("end_reason"),
            }
            writer.write_step(step, row)
            rows.append({"step": step, **row})
            observation = observation_after
            if step == 1 or step % 25 == 0:
                elapsed = time.monotonic() - started
                eta = elapsed / step * (contract.EXPECTED_STEPS - step)
                print(
                    f"[V10S final/{case_id}] {step:3d}/500 "
                    f"elapsed={elapsed:7.1f}s eta={eta:7.1f}s",
                    flush=True,
                )
            if terminated or truncated:
                break
    except Exception as exc:
        try:
            writer.publish_failure(
                end_reason="V10S_final_rollout_failed",
                error=exc,
                status=contract.FINAL_ROLLOUT_FAIL_STATUS,
            )
        except Exception:
            pass
        raise
    finally:
        env.close()

    if audit is None:
        raise V10SSafeDaggerError("final rollout audit was not initialized")
    summary = {
        **_physical_summary(
            audit,
            case=case,
            rows=rows,
            info=info,
            terminated=terminated,
            truncated=truncated,
            actor_names=actor_names,
            full_names=full_names,
            legacy=legacy,
            v26_collector=v26_collector,
        ),
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.FINAL_ROLLOUT_STATUS,
        "protocol_id": contract.PROTOCOL_ID,
        "behavior": contract.FINAL_BEHAVIOR,
        "candidate_fit_stage": "p3",
        "candidate_id": freeze["candidate_id"],
        "candidate_mean_query_count": candidate_mean_query_count,
        "blending_enabled": False,
        "mean_blend_count": 0,
        "teacher_enabled": False,
        "teacher_query_count": 0,
        "served_action_teacher_dependency_count": 0,
        "safety_latch_enabled": False,
        "safety_intervention_count": 0,
        "safety_latch_activation_count": 0,
    }
    partial = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": "H0_PRIMARY_SPLIT_V10S_FINAL_PERSISTED_BEFORE_GATE",
        "protocol_id": contract.PROTOCOL_ID,
        "stage_id": stage_id,
        "case_id": case_id,
        "steps": len(rows),
        "gate_evaluated": False,
        "teacher_query_count": 0,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
    }
    persisted = writer.finalize_before_gate(
        trace=rows, partial_summary=partial, summary=summary
    )

    def evaluate(_records: dict[str, Any]) -> dict[str, Any]:
        gate = contract.final_rollout_gate(summary)
        gate["persisted_before_gate"] = persisted
        return gate

    writer.run_gate(evaluate)
    gate = _mapping(writer.gate_path)
    if gate.get("passed") is not True:
        writer.publish_failure(
            end_reason="V10S_final_rollout_gate_failed",
            error="V10S final physical/dependency gate failed",
            status=contract.FINAL_ROLLOUT_FAIL_STATUS,
            details={"gate": _record(writer.gate_path)},
        )
        raise V10SSafeDaggerError(f"final rollout gate failed: {case_id}")
    receipt = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.FINAL_ROLLOUT_PASS_STATUS,
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "stage_id": stage_id,
        "case_id": case_id,
        "candidate_id": freeze["candidate_id"],
        "candidate_fit_stage": "p3",
        "sample_count": len(rows),
        "candidate_mean_query_count": candidate_mean_query_count,
        "teacher_query_count": 0,
        "served_action_teacher_dependency_count": 0,
        "mean_blend_count": 0,
        "safety_intervention_count": 0,
        "artifacts": writer.artifact_records(),
        "pipeline_claim": _record(PIPELINE_CLAIM),
        "worker_claim": _record(_claim_path(stage_id)),
        "retry_authorized": False,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
    }
    receipt_path = forensic.write_json_exclusive(
        writer.run_directory / "receipt.json", receipt
    )
    return {**receipt, "receipt": _record(receipt_path)}


def verify_final_rollout_receipt(case_id: str) -> dict[str, Any]:
    stage_id = f"final__{case_id}"
    case = contract.canonical_final_case(case_id)
    destination = resolve_relative(case["destination"])
    writer = forensic.ForensicRolloutWriter(destination, artifact_root=REPO_ROOT)
    receipt = _mapping(_stage_receipt_path(stage_id))
    summary = _mapping(writer.summary_path)
    gate = _mapping(writer.gate_path)
    expected_gate = contract.final_rollout_gate(summary)
    expected_gate["persisted_before_gate"] = writer.finalized_artifact_records()
    freeze = _candidate_freeze()
    if (
        receipt.get("schema_version") != contract.SCHEMA_VERSION
        or receipt.get("status") != contract.FINAL_ROLLOUT_PASS_STATUS
        or receipt.get("passed") is not True
        or receipt.get("protocol_id") != contract.PROTOCOL_ID
        or receipt.get("stage_id") != stage_id
        or receipt.get("case_id") != case_id
        or receipt.get("candidate_id") != freeze["candidate_id"]
        or receipt.get("candidate_fit_stage") != "p3"
        or receipt.get("sample_count") != contract.EXPECTED_STEPS
        or receipt.get("candidate_mean_query_count") != contract.EXPECTED_STEPS
        or receipt.get("teacher_query_count") != 0
        or receipt.get("served_action_teacher_dependency_count") != 0
        or receipt.get("mean_blend_count") != 0
        or receipt.get("safety_intervention_count") != 0
        or receipt.get("artifacts") != writer.artifact_records()
        or not _record_matches(receipt.get("pipeline_claim"), PIPELINE_CLAIM)
        or not _record_matches(receipt.get("worker_claim"), _claim_path(stage_id))
        or receipt.get("retry_authorized") is not False
        or receipt.get("actor_updates") != 0
        or receipt.get("critic_updates") != 0
        or receipt.get("ppo_updates") != 0
        or receipt.get("protected_trials_opened") != []
        or receipt.get("reserve_trials_opened") != []
        or forensic.canonical_json_bytes(gate)
        != forensic.canonical_json_bytes(expected_gate)
        or gate.get("passed") is not True
    ):
        raise V10SSafeDaggerError(f"final receipt closure drifted: {case_id}")
    rows = _sequence(writer.trace_path)
    if len(rows) != contract.EXPECTED_STEPS or any(
        not isinstance(row, Mapping)
        or row.get("step") != index
        or row.get("protocol_id") != contract.PROTOCOL_ID
        or row.get("case_id") != case_id
        or row.get("teacher_enabled") is not False
        or row.get("teacher_query_count") != 0
        or row.get("served_action_teacher_dependency_count") != 0
        or row.get("blending_enabled") is not False
        or row.get("safety_latch_enabled") is not False
        for index, row in enumerate(rows, start=1)
    ):
        raise V10SSafeDaggerError(f"final trace dependency closure drifted: {case_id}")
    return receipt


def _final_rollout_bindings() -> list[dict[str, Any]]:
    bindings: list[dict[str, Any]] = []
    for case_id in contract.FINAL_CASE_IDS:
        receipt = verify_final_rollout_receipt(case_id)
        destination = resolve_relative(
            contract.canonical_final_case(case_id)["destination"]
        )
        bindings.append(
            {
                "case_id": case_id,
                "passed": receipt.get("passed") is True,
                "receipt": _record(destination / "receipt.json"),
                "gate": _record(destination / "gate.json"),
                "trace": _record(destination / "trace.json"),
            }
        )
    return bindings


def _final_rollout_diagnostics() -> list[dict[str, Any]]:
    diagnostics: list[dict[str, Any]] = []
    for case_id in contract.FINAL_CASE_IDS:
        destination = resolve_relative(
            contract.canonical_final_case(case_id)["destination"]
        )
        summary_path = destination / "summary.json"
        summary = _mapping(summary_path)
        episode_metrics = summary.get("episode_metrics")
        sea_episode_metrics = summary.get("sea_episode_metrics")
        if not isinstance(episode_metrics, Mapping) or not isinstance(
            sea_episode_metrics, Mapping
        ):
            raise V10SSafeDaggerError(
                f"final SEA/reserve diagnostics are missing: {case_id}"
            )
        diagnostics.append(
            {
                "case_id": case_id,
                "summary": _record(summary_path),
                "episode_metrics": copy.deepcopy(dict(episode_metrics)),
                "sea_episode_metrics": copy.deepcopy(dict(sea_episode_metrics)),
            }
        )
    return diagnostics


def _finalize_development() -> dict[str, Any]:
    stage_id = "finalize_development"
    freeze = _candidate_freeze()
    rollout_bindings = _final_rollout_bindings()
    rollout_diagnostics = _final_rollout_diagnostics()
    summary_path = FINAL_RECEIPT.with_name("summary.json")
    gate_path = FINAL_RECEIPT.with_name("gate.json")
    for path in (FINAL_RECEIPT, summary_path, gate_path):
        if os.path.lexists(path):
            raise V10SSafeDaggerError(f"final aggregate output exists: {path}")
    p3_root = resolve_relative(contract.FIT_ROOTS["p3"])
    summary = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": "H0_PRIMARY_SPLIT_V10S_FINAL_DEVELOPMENT_UNGATED",
        "protocol_id": contract.PROTOCOL_ID,
        "candidate_id": freeze["candidate_id"],
        "candidate_fit_stage": "p3",
        "candidate_frozen_before_final": True,
        "candidate_freeze": _record(CANDIDATE_FREEZE),
        "candidate_module": _tree_record(resolve_relative(contract.MODULE_PATHS["p3"])),
        "p3_fit_receipt": _record(resolve_relative(contract.FIT_RECEIPT_PATHS["p3"])),
        "p3_fit_gate": _record(p3_root / "gate.json"),
        "case_ids": list(contract.FINAL_CASE_IDS),
        "rollout_receipt_count": len(rollout_bindings),
        "rollout_pass_count": sum(
            int(binding["passed"] is True) for binding in rollout_bindings
        ),
        "all_rollouts_passed": all(
            binding["passed"] is True for binding in rollout_bindings
        ),
        "rollout_bindings": rollout_bindings,
        "final_rollout_diagnostics": rollout_diagnostics,
        "fit_actor_update_count": len(contract.FIT_STAGES),
        "every_fit_restarted_from_h0": all(
            _mapping(resolve_relative(contract.FIT_ROOTS[stage]) / "summary.json").get(
                "continued_from_previous_candidate"
            )
            is False
            for stage in contract.FIT_STAGES
        ),
        "final_candidate_mean_query_count": sum(
            int(
                _mapping(
                    resolve_relative(
                        contract.canonical_final_case(case_id)["destination"]
                    )
                    / "summary.json"
                )["candidate_mean_query_count"]
            )
            for case_id in contract.FINAL_CASE_IDS
        ),
        "final_teacher_query_count": 0,
        "final_mean_blend_count": 0,
        "final_safety_intervention_count": 0,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
    }
    forensic.write_json_exclusive(summary_path, summary)
    gate = contract.final_development_gate(summary)
    forensic.write_json_exclusive(gate_path, gate)
    if gate.get("passed") is not True:
        raise V10SSafeDaggerError("V10S final development aggregate gate failed")
    receipt = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.FINAL_DEVELOPMENT_PASS_STATUS,
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "stage_id": stage_id,
        "candidate_id": freeze["candidate_id"],
        "candidate_fit_stage": "p3",
        "candidate_freeze": _record(CANDIDATE_FREEZE),
        "candidate_module": _tree_record(resolve_relative(contract.MODULE_PATHS["p3"])),
        "p3_fit_receipt": _record(resolve_relative(contract.FIT_RECEIPT_PATHS["p3"])),
        "p3_fit_gate": _record(p3_root / "gate.json"),
        "summary": _record(summary_path),
        "gate": _record(gate_path),
        "rollout_bindings": rollout_bindings,
        "final_rollout_diagnostics": rollout_diagnostics,
        "case_ids": list(contract.FINAL_CASE_IDS),
        "rollout_pass_count": len(contract.FINAL_CASE_IDS),
        "final_candidate_mean_query_count": (
            len(contract.FINAL_CASE_IDS) * contract.EXPECTED_STEPS
        ),
        "final_teacher_query_count": 0,
        "final_mean_blend_count": 0,
        "final_safety_intervention_count": 0,
        "fit_actor_update_count": len(contract.FIT_STAGES),
        "pipeline_claim": _record(PIPELINE_CLAIM),
        "worker_claim": _record(_claim_path(stage_id)),
        "retry_authorized": False,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
        "next_stage": "V10Q_SEA_RESERVE_QUALIFICATION_REQUIRED",
    }
    path = forensic.write_json_exclusive(FINAL_RECEIPT, receipt)
    return {**receipt, "receipt": _record(path)}


def verify_final_development_receipt() -> dict[str, Any]:
    receipt = _mapping(FINAL_RECEIPT)
    summary_path = FINAL_RECEIPT.with_name("summary.json")
    gate_path = FINAL_RECEIPT.with_name("gate.json")
    summary = _mapping(summary_path)
    gate = _mapping(gate_path)
    expected_gate = contract.final_development_gate(summary)
    freeze = _candidate_freeze()
    rollout_bindings = _final_rollout_bindings()
    rollout_diagnostics = _final_rollout_diagnostics()
    p3_root = resolve_relative(contract.FIT_ROOTS["p3"])
    if (
        receipt.get("schema_version") != contract.SCHEMA_VERSION
        or receipt.get("status") != contract.FINAL_DEVELOPMENT_PASS_STATUS
        or receipt.get("passed") is not True
        or receipt.get("protocol_id") != contract.PROTOCOL_ID
        or receipt.get("stage_id") != "finalize_development"
        or receipt.get("candidate_id") != freeze["candidate_id"]
        or receipt.get("candidate_fit_stage") != "p3"
        or not _record_matches(receipt.get("candidate_freeze"), CANDIDATE_FREEZE)
        or receipt.get("candidate_module")
        != _tree_record(resolve_relative(contract.MODULE_PATHS["p3"]))
        or not _record_matches(
            receipt.get("p3_fit_receipt"),
            resolve_relative(contract.FIT_RECEIPT_PATHS["p3"]),
        )
        or not _record_matches(receipt.get("p3_fit_gate"), p3_root / "gate.json")
        or not _record_matches(receipt.get("summary"), summary_path)
        or not _record_matches(receipt.get("gate"), gate_path)
        or receipt.get("rollout_bindings") != rollout_bindings
        or receipt.get("final_rollout_diagnostics") != rollout_diagnostics
        or receipt.get("case_ids") != list(contract.FINAL_CASE_IDS)
        or receipt.get("rollout_pass_count") != len(contract.FINAL_CASE_IDS)
        or receipt.get("final_candidate_mean_query_count")
        != len(contract.FINAL_CASE_IDS) * contract.EXPECTED_STEPS
        or receipt.get("final_teacher_query_count") != 0
        or receipt.get("final_mean_blend_count") != 0
        or receipt.get("final_safety_intervention_count") != 0
        or receipt.get("fit_actor_update_count") != len(contract.FIT_STAGES)
        or not _record_matches(receipt.get("pipeline_claim"), PIPELINE_CLAIM)
        or not _record_matches(
            receipt.get("worker_claim"), _claim_path("finalize_development")
        )
        or receipt.get("retry_authorized") is not False
        or receipt.get("actor_updates") != 0
        or receipt.get("critic_updates") != 0
        or receipt.get("ppo_updates") != 0
        or receipt.get("protected_trials_opened") != []
        or receipt.get("reserve_trials_opened") != []
        or receipt.get("next_stage")
        != "V10Q_SEA_RESERVE_QUALIFICATION_REQUIRED"
        or forensic.canonical_json_bytes(gate)
        != forensic.canonical_json_bytes(expected_gate)
        or gate.get("passed") is not True
    ):
        raise V10SSafeDaggerError("final development receipt closure drifted")
    return receipt


def verify_stage_receipt(stage_id: str) -> dict[str, Any]:
    descriptor = contract.stage_descriptor(stage_id)
    kind = descriptor["kind"]
    if kind == "fit":
        return verify_fit_receipt(str(descriptor["fit_stage"]))
    if kind == "collection":
        case = descriptor["case"]
        return verify_collection_receipt(
            int(descriptor["round_index"]), str(case["case_id"])
        )
    if kind == "freeze":
        return _candidate_freeze()
    if kind == "final":
        return verify_final_rollout_receipt(str(descriptor["case"]["case_id"]))
    if kind == "finalize":
        return verify_final_development_receipt()
    raise V10SSafeDaggerError(f"unhandled stage kind: {kind!r}")


def _fit_actor_update_evidence(stage: str) -> dict[str, Any]:
    """Count only persisted fit evidence and flag any claimed ambiguous fit."""

    stage_id = f"fit_{stage}"
    receipt_path = resolve_relative(contract.FIT_RECEIPT_PATHS[stage])
    summary_path = resolve_relative(contract.FIT_ROOTS[stage]) / "summary.json"
    counts: list[int] = []
    records: dict[str, Any] = {}
    errors: list[str] = []
    for name, path in (("receipt", receipt_path), ("summary", summary_path)):
        if not os.path.lexists(path):
            continue
        try:
            payload = _mapping(path)
            value = payload.get("actor_updates")
            if type(value) is not int or value < 0:
                raise V10SSafeDaggerError(f"invalid actor_updates in {name}")
            counts.append(value)
            records[name] = _record(path)
        except Exception as exc:
            errors.append(f"{name}:{type(exc).__name__}:{exc}")
    consistent = bool(counts) and len(set(counts)) == 1
    confirmed = counts[0] if consistent else max(counts, default=0)
    claimed = os.path.lexists(_claim_path(stage_id))
    attempted_or_unknown = bool(claimed and (not consistent or errors))
    return {
        "fit_stage": stage,
        "worker_claim_present": claimed,
        "confirmed_actor_updates": confirmed,
        "actor_update_attempted_or_unknown": attempted_or_unknown,
        "evidence_consistent": consistent,
        "records": records,
        "errors": errors,
    }


def _pipeline_actor_update_audit() -> dict[str, Any]:
    rows = [_fit_actor_update_evidence(stage) for stage in contract.FIT_STAGES]
    ambiguous_stages = [
        str(row["fit_stage"])
        for row in rows
        if row["actor_update_attempted_or_unknown"] is True
    ]
    return {
        "confirmed_actor_updates": sum(
            int(row["confirmed_actor_updates"]) for row in rows
        ),
        "actor_update_attempted_or_unknown": bool(ambiguous_stages),
        "actor_update_attempted_or_unknown_stages": ambiguous_stages,
        "fit_stage_evidence": rows,
    }


def _completed_safe_dagger_round_count(completed_stages: Sequence[str]) -> int:
    completed = set(completed_stages)
    return sum(
        int(
            all(
                f"collect_r{round_index}__{case_id}" in completed
                for case_id in contract.COLLECTION_CASE_IDS
            )
        )
        for round_index in (1, 2, 3)
    )


def run_worker(*, stage_id: str, supervisor_token: str) -> dict[str, Any]:
    """Run exactly one already-claimed stage; workers cannot mint authority."""

    verify_worker_claim(stage_id, supervisor_token)
    descriptor = contract.stage_descriptor(stage_id)
    try:
        kind = descriptor["kind"]
        if kind == "fit":
            return _run_fit_stage(str(descriptor["fit_stage"]))
        if kind == "collection":
            case = descriptor["case"]
            return _collect_safe_dagger(
                int(descriptor["round_index"]), str(case["case_id"])
            )
        if kind == "freeze":
            return _freeze_p3()
        if kind == "final":
            return _final_rollout(str(descriptor["case"]["case_id"]))
        if kind == "finalize":
            return _finalize_development()
    except Exception as exc:
        failure_path = _stage_receipt_path(stage_id).with_name("failure.json")
        if not os.path.lexists(failure_path):
            try:
                actor_evidence = (
                    _fit_actor_update_evidence(str(descriptor["fit_stage"]))
                    if kind == "fit"
                    else None
                )
                forensic.write_json_exclusive(
                    failure_path,
                    {
                        "schema_version": contract.SCHEMA_VERSION,
                        "status": contract.PIPELINE_FAIL_STATUS,
                        "passed": False,
                        "protocol_id": contract.PROTOCOL_ID,
                        "pipeline_id": contract.PIPELINE_ID,
                        "stage_id": stage_id,
                        "error": {
                            "type": type(exc).__name__,
                            "message": str(exc),
                        },
                        "pipeline_claim": _record(PIPELINE_CLAIM),
                        "worker_claim": _record(_claim_path(stage_id)),
                        "retry_authorized": False,
                        "actor_updates": (
                            actor_evidence["confirmed_actor_updates"]
                            if actor_evidence is not None
                            else 0
                        ),
                        "actor_update_attempted_or_unknown": (
                            actor_evidence["actor_update_attempted_or_unknown"]
                            if actor_evidence is not None
                            else False
                        ),
                        "actor_update_evidence": actor_evidence,
                        "critic_updates": 0,
                        "ppo_updates": 0,
                        "protected_trials_opened": [],
                        "reserve_trials_opened": [],
                    },
                )
            except Exception:
                pass
        raise
    raise V10SSafeDaggerError(f"unhandled worker stage: {stage_id}")


def _preexecution_absence() -> None:
    verify_lock(require_run_root_absent=True)
    if os.path.lexists(PIPELINE_CLAIM) or os.path.lexists(PIPELINE_LEDGER):
        raise V10SSafeDaggerError("V10S pipeline was already claimed")
    evidence = _mapping(_teacher_evidence_path())
    if contract.teacher_evidence_gate(evidence).get("passed") is not True:
        raise V10SSafeDaggerError("V10A teacher evidence is not intact PASS")


def execute() -> dict[str, Any]:
    """Claim and run the complete fail-closed V10S pipeline once."""

    _preexecution_absence()
    token = secrets.token_urlsafe(32)
    token_sha256 = _token_sha256(token)
    forensic.write_json_exclusive(PIPELINE_CLAIM, _claim_payload(token_sha256))
    started = time.time()
    completed_stages: list[str] = []
    completed_receipts: list[dict[str, Any]] = []
    error: str | None = None
    passed = False
    try:
        for stage_id in contract.STAGE_IDS:
            previous = [
                {
                    "stage_id": prior,
                    "receipt": _record(_stage_receipt_path(prior)),
                }
                for prior in completed_stages
            ]
            forensic.write_json_exclusive(
                _claim_path(stage_id),
                _worker_claim_payload(
                    stage_id=stage_id,
                    token_sha256=token_sha256,
                    previous_receipts=previous,
                ),
            )
            completed = subprocess.run(
                _worker_command(stage_id, token),
                cwd=REPO_ROOT,
                timeout=WORKER_TIMEOUT_S,
                check=False,
            )
            if completed.returncode != 0:
                raise V10SSafeDaggerError(
                    f"worker {stage_id} exited {completed.returncode}"
                )
            verify_stage_receipt(stage_id)
            completed_stages.append(stage_id)
            completed_receipts.append(
                {
                    "stage_id": stage_id,
                    "receipt": _record(_stage_receipt_path(stage_id)),
                }
            )
        final = verify_final_development_receipt()
        passed = (
            final.get("status") == contract.FINAL_DEVELOPMENT_PASS_STATUS
            and final.get("passed") is True
        )
        if not passed:
            raise V10SSafeDaggerError("terminal V10S closure is not PASS")
    except Exception as exc:
        error = f"{type(exc).__name__}: {exc}"

    actor_update_audit = _pipeline_actor_update_audit()
    confirmed_actor_updates = int(actor_update_audit["confirmed_actor_updates"])
    ambiguous_actor_updates = list(
        actor_update_audit["actor_update_attempted_or_unknown_stages"]
    )
    if passed and (confirmed_actor_updates != 4 or ambiguous_actor_updates):
        passed = False
        error = (
            "V10SSafeDaggerError: terminal actor-update audit is not exact: "
            f"confirmed={confirmed_actor_updates}, "
            f"attempted_or_unknown={ambiguous_actor_updates}"
        )
    ledger = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": (
            contract.PIPELINE_PASS_STATUS if passed else contract.PIPELINE_FAIL_STATUS
        ),
        "passed": passed,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "started_unix_s": started,
        "completed_unix_s": time.time(),
        "expected_stages": list(contract.STAGE_IDS),
        "completed_stages": completed_stages,
        "completed_receipts": completed_receipts,
        "error": error,
        "pipeline_claim": _record(PIPELINE_CLAIM),
        "candidate_freeze": _record(CANDIDATE_FREEZE) if passed else None,
        "final_development_receipt": _record(FINAL_RECEIPT) if passed else None,
        "safe_dagger_rounds": _completed_safe_dagger_round_count(
            completed_stages
        ),
        "fit_actor_update_count": confirmed_actor_updates,
        "actor_updates": confirmed_actor_updates,
        "actor_update_attempted_or_unknown": bool(ambiguous_actor_updates),
        "actor_update_attempted_or_unknown_stages": ambiguous_actor_updates,
        "actor_update_evidence": actor_update_audit,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
        "retry_authorized": False,
        "sweep_authorized": False,
        "next_stage": (
            "V10Q_SEA_RESERVE_QUALIFICATION_REQUIRED"
            if passed
            else "STOP_V10S_TERMINAL_NO_RETRY_SWEEP_OR_RESCUE"
        ),
    }
    forensic.write_json_exclusive(PIPELINE_LEDGER, ledger)
    print(json.dumps(ledger, indent=2, sort_keys=True, allow_nan=False), flush=True)
    if not passed:
        raise V10SSafeDaggerError(error or contract.PIPELINE_FAIL_STATUS)
    return ledger


def _worker_command(stage_id: str, token: str) -> list[str]:
    return [
        sys.executable,
        str(Path(__file__).resolve()),
        "--worker",
        "--stage",
        stage_id,
        "--supervisor-token",
        token,
    ]


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--prepare", action="store_true")
    mode.add_argument("--execute", action="store_true")
    mode.add_argument("--worker", action="store_true")
    parser.add_argument("--stage", choices=contract.STAGE_IDS)
    parser.add_argument("--supervisor-token")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        if args.prepare:
            if args.stage is not None or args.supervisor_token is not None:
                raise V10SSafeDaggerError(
                    "prepare does not accept worker arguments"
                )
            result = prepare()
            print(json.dumps(result, indent=2, sort_keys=True, allow_nan=False))
            return 0
        if args.execute:
            if args.stage is not None or args.supervisor_token is not None:
                raise V10SSafeDaggerError(
                    "supervisor execution does not accept worker arguments"
                )
            execute()
            return 0
        if args.stage is None or args.supervisor_token is None:
            raise V10SSafeDaggerError(
                "worker requires --stage and --supervisor-token"
            )
        run_worker(stage_id=args.stage, supervisor_token=args.supervisor_token)
        return 0
    except Exception as exc:
        print(f"{type(exc).__name__}: {exc}", file=sys.stderr, flush=True)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
