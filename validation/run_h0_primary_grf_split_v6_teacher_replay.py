"""Collect frozen V5 teacher-action trajectories under V25 active semantics.

The collector is development-only.  It never creates a candidate and never
updates actor, critic, optimizer, or PPO state.  Each physical rollout is
driven by the exact V5 baseline ``raw_action`` sequence while the source H0 is
queried only for a teacher mean.  V25-active observations and teacher labels
are persisted per step before the scientific gate is evaluated.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import secrets
import subprocess
import sys
import time
from pathlib import Path
from typing import Any, Mapping, Sequence


REPO_ROOT = Path(__file__).resolve().parents[1]
VALIDATION_ROOT = REPO_ROOT / "validation"
TRAJECTORY_ROOT = REPO_ROOT / "Trajectory Generator"
BASELINE_ROOT = TRAJECTORY_ROOT / "baseline_MLP"
for import_root in (VALIDATION_ROOT, BASELINE_ROOT, TRAJECTORY_ROOT, REPO_ROOT):
    if str(import_root) not in sys.path:
        sys.path.insert(0, str(import_root))

import build_h0_primary_grf_split_v6_teacher_replay_preflight as preflight  # noqa: E402
import h0_forensic_rollout as forensic  # noqa: E402
import h0_primary_grf_split_v6_teacher_replay_contract as contract  # noqa: E402
import h0_v3_so_recovery_contract as so_recovery  # noqa: E402
import run_h0_v25_abc_preflight as legacy  # noqa: E402
from primary_grf_split_adaptation import array_sha256  # noqa: E402


LOCK = preflight.resolve_relative(contract.LOCK_PATH)
RUN_ROOT = preflight.resolve_relative(contract.RUN_ROOT)
EXECUTION_LEDGER = preflight.resolve_relative(contract.EXECUTION_LEDGER_PATH)
EXECUTION_CLAIM = preflight.resolve_relative(contract.EXECUTION_CLAIM_PATH)
WORKER_CLAIMS_ROOT = preflight.resolve_relative(contract.WORKER_CLAIMS_ROOT)
INPUT_PATHS = preflight.input_paths()
SOURCE_PATHS = preflight.source_paths()
SOURCE_H0_MODULE = INPUT_PATHS["source_h0_module_state"].parent
SOURCE_H0_CONFIG = INPUT_PATHS["source_h0_config"]
WORKER_TIMEOUT_S = 2400.0
TIME_TOLERANCE_S = 1.0e-12


class V6TeacherReplayExecutionError(RuntimeError):
    """Raised on any provenance, runtime, persistence, or gate failure."""


def _mapping(path: Path) -> dict[str, Any]:
    value = forensic.strict_json_load(path)
    if not isinstance(value, Mapping):
        raise V6TeacherReplayExecutionError(f"expected JSON object: {path}")
    return dict(value)


def _sequence(path: Path) -> list[Any]:
    value = forensic.strict_json_load(path)
    if isinstance(value, (str, bytes)) or not isinstance(value, Sequence):
        raise V6TeacherReplayExecutionError(f"expected JSON array: {path}")
    return list(value)


def _record(path: Path) -> dict[str, Any]:
    return preflight.source_record(path)


def _record_matches(record: Any, path: Path) -> bool:
    return (
        isinstance(record, Mapping)
        and set(record) == {"path", "sha256", "size_bytes"}
        and dict(record) == _record(path)
    )


def verify_lock(*, require_run_root_absent: bool = False) -> dict[str, Any]:
    lock = _mapping(LOCK)
    preflight_path = preflight.resolve_relative(contract.PREFLIGHT_RECEIPT_PATH)
    preflight_receipt = _mapping(preflight_path)
    expected_keys = {
        "schema_version",
        "status",
        "protocol_id",
        "collector_id",
        "revision",
        "run_root",
        "execution_ledger",
        "execution_claim",
        "worker_claims_root",
        "execution_order",
        "cases",
        "matrix",
        "invariant_columns",
        "gate",
        "preflight_receipt",
        "sources",
        "inputs",
        "v5_terminal",
        "v25",
        "source_h0",
        "development_reclassification",
        "authority",
        "simulations_executed_at_freeze",
        "candidate_created",
        "actor_updates",
        "critic_updates",
        "ppo_updates",
        "protected_trials_opened",
        "next_stage",
    }
    if (
        set(lock) != expected_keys
        or lock.get("schema_version") != contract.SCHEMA_VERSION
        or lock.get("status") != contract.LOCK_STATUS
        or lock.get("protocol_id") != contract.PROTOCOL_ID
        or lock.get("collector_id") != contract.COLLECTOR_ID
        or lock.get("revision") != contract.REVISION
        or lock.get("run_root") != contract.RUN_ROOT.as_posix()
        or lock.get("execution_ledger") != contract.EXECUTION_LEDGER_PATH.as_posix()
        or lock.get("execution_claim") != contract.EXECUTION_CLAIM_PATH.as_posix()
        or lock.get("worker_claims_root") != contract.WORKER_CLAIMS_ROOT.as_posix()
        or lock.get("execution_order") != list(contract.CASE_IDS)
        or lock.get("cases") != [dict(case) for case in contract.CASES]
        or lock.get("authority") != contract.AUTHORITY
        or lock.get("simulations_executed_at_freeze") != 0
        or lock.get("candidate_created") is not False
        or lock.get("actor_updates") != 0
        or lock.get("critic_updates") != 0
        or lock.get("ppo_updates") != 0
        or lock.get("protected_trials_opened") != []
        or lock.get("next_stage") != "EXECUTE_SIX_V25_ACTIVE_DEVELOPMENT_REPLAYS_ONCE"
        or lock.get("matrix")
        != {
            "rollout_count": len(contract.CASES),
            "destinations": [str(case["destination"]) for case in contract.CASES],
            "behavior": "FROZEN_V5_RAW_ACTION_REPLAY",
            "observation": contract.TARGET_OBSERVATION_CONTRACT_ID,
            "teacher": contract.SOURCE_H0_ID,
        }
        or lock.get("development_reclassification")
        != {
            "source": "V5_PASS_CONDITION_MATCHED_BASELINES",
            "case_ids": list(contract.CASE_IDS),
            "new_role": "V6_DEVELOPMENT_TEACHER_ACTION_REPLAY",
            "eligible_as_future_holdout": False,
        }
        or lock.get("v5_terminal") != preflight_receipt.get("v5_terminal")
        or lock.get("v25") != preflight_receipt.get("v25")
        or lock.get("source_h0") != preflight_receipt.get("source_h0")
    ):
        raise V6TeacherReplayExecutionError("V6 execution lock identity drifted")
    if (
        preflight_receipt.get("status") != contract.PREFLIGHT_STATUS
        or preflight_receipt.get("passed") is not True
        or not isinstance(preflight_receipt.get("checks"), Mapping)
        or not all(value is True for value in preflight_receipt["checks"].values())
        or preflight_receipt.get("authority") != contract.AUTHORITY
        or preflight_receipt.get("simulations_executed") != 0
        or preflight_receipt.get("candidate_created") is not False
        or preflight_receipt.get("actor_updates") != 0
        or preflight_receipt.get("critic_updates") != 0
        or preflight_receipt.get("ppo_updates") != 0
        or preflight_receipt.get("protected_trials_opened") != []
        or lock.get("sources") != preflight_receipt.get("sources")
        or lock.get("inputs") != preflight_receipt.get("inputs")
    ):
        raise V6TeacherReplayExecutionError("V6 preflight closure drifted")
    if not _record_matches(lock.get("preflight_receipt"), preflight_path):
        raise V6TeacherReplayExecutionError("V6 preflight record drifted")
    for label, paths in (("sources", SOURCE_PATHS), ("inputs", INPUT_PATHS)):
        records = lock.get(label)
        if not isinstance(records, Mapping) or set(records) != set(paths):
            raise V6TeacherReplayExecutionError(f"lock {label} closure drifted")
        for name, path in paths.items():
            if not _record_matches(records[name], path):
                raise V6TeacherReplayExecutionError(f"lock {label}.{name} drifted")
    invariant = lock.get("invariant_columns")
    gate = lock.get("gate")
    expected_invariant = {
        "ranges_half_open": [
            list(bounds) for bounds in contract.INVARIANT_COLUMN_RANGES
        ],
        "indices": list(contract.INVARIANT_COLUMNS),
        "comparison": "FLOAT32_C_CONTIGUOUS_BYTES_EXACT",
    }
    if invariant != expected_invariant:
        raise V6TeacherReplayExecutionError("invariant-column lock drifted")
    expected_gate = {
        "expected_steps": contract.EXPECTED_STEPS,
        "expected_control_windows": contract.EXPECTED_CONTROL_WINDOWS,
        "expected_v25_raw_sensor_samples": contract.EXPECTED_RAW_SENSOR_SAMPLES,
        "minimum_valid_cycles": contract.MINIMUM_VALID_CYCLES,
        "penetration_limit_m": contract.PENETRATION_LIMIT_M,
        "penetration_comparison": "strict_less_than",
        "zero_mismatch_fields": [
            "invariant_mismatch_count",
            "teacher_mean_mismatch_count",
            "time_mismatch_count",
            "step_contract_failure_count",
        ],
        "persist_before_gate": [
            "trace.json",
            "partial_summary.json",
            "summary.json",
        ],
    }
    if gate != expected_gate:
        raise V6TeacherReplayExecutionError("V6 gate lock drifted")
    if require_run_root_absent and os.path.lexists(RUN_ROOT):
        raise V6TeacherReplayExecutionError(f"run root already claimed: {RUN_ROOT}")
    return lock


def canonical_destination(case_id: str) -> Path:
    case = contract.canonical_case(case_id)
    return preflight.resolve_relative(case["destination"])


def worker_claim_path(case_id: str) -> Path:
    try:
        index = contract.CASE_IDS.index(case_id)
    except ValueError as exc:
        raise V6TeacherReplayExecutionError(
            f"unknown worker claim case: {case_id!r}"
        ) from exc
    return WORKER_CLAIMS_ROOT / f"{index + 1:02d}_{case_id}.json"


def _token_sha256(token: str) -> str:
    if not isinstance(token, str) or len(token) < 32:
        raise V6TeacherReplayExecutionError("execution token is malformed")
    return hashlib.sha256(token.encode("utf-8")).hexdigest()


def verify_case_receipt(case_id: str) -> dict[str, Any]:
    """Verify a completed worker receipt and every referenced artifact."""

    lock = verify_lock()
    destination = canonical_destination(case_id)
    receipt_path = destination / "receipt.json"
    receipt = _mapping(receipt_path)
    artifacts = receipt.get("artifacts")
    persisted = receipt.get("persisted_before_gate")
    if (
        set(receipt)
        != {
            "schema_version",
            "status",
            "passed",
            "protocol_id",
            "case_id",
            "artifacts",
            "persisted_before_gate",
            "execution_lock",
            "execution_claim",
            "worker_claim",
            "candidate_created",
            "actor_updates",
            "critic_updates",
            "ppo_updates",
            "protected_trials_opened",
        }
        or receipt.get("schema_version") != contract.SCHEMA_VERSION
        or receipt.get("status") != contract.ROLLOUT_PASS_STATUS
        or receipt.get("passed") is not True
        or receipt.get("protocol_id") != contract.PROTOCOL_ID
        or receipt.get("case_id") != case_id
        or not _record_matches(receipt.get("execution_lock"), LOCK)
        or not _record_matches(receipt.get("execution_claim"), EXECUTION_CLAIM)
        or not _record_matches(receipt.get("worker_claim"), worker_claim_path(case_id))
        or receipt.get("candidate_created") is not False
        or type(receipt.get("actor_updates")) is not int
        or receipt.get("actor_updates") != 0
        or type(receipt.get("critic_updates")) is not int
        or receipt.get("critic_updates") != 0
        or type(receipt.get("ppo_updates")) is not int
        or receipt.get("ppo_updates") != 0
        or receipt.get("protected_trials_opened") != []
        or not isinstance(artifacts, Mapping)
        or set(artifacts)
        != {
            "run_start",
            "steps",
            "trace",
            "partial_summary",
            "summary",
            "gate",
        }
        or not isinstance(persisted, Mapping)
        or set(persisted) != {"trace", "partial_summary", "summary"}
    ):
        raise V6TeacherReplayExecutionError(
            f"worker receipt identity drifted: {case_id}"
        )
    expected_paths = {
        "run_start": destination / "run_start.json",
        "trace": destination / "trace.json",
        "partial_summary": destination / "partial_summary.json",
        "summary": destination / "summary.json",
        "gate": destination / "gate.json",
    }
    if os.path.lexists(destination / "failure.json"):
        raise V6TeacherReplayExecutionError(
            f"PASS worker also has a failure receipt: {case_id}"
        )
    for name, path in expected_paths.items():
        if not _record_matches(artifacts.get(name), path):
            raise V6TeacherReplayExecutionError(
                f"worker artifact record drifted: {case_id}/{name}"
            )
    observed_start = _mapping(expected_paths["run_start"])
    expected_start = _worker_start_payload(
        case=contract.canonical_case(case_id),
        lock=lock,
    )
    if forensic.canonical_json_bytes(observed_start) != forensic.canonical_json_bytes(
        expected_start
    ):
        raise V6TeacherReplayExecutionError(
            f"worker start provenance drifted: {case_id}"
        )
    steps = artifacts.get("steps")
    if not isinstance(steps, list) or len(steps) != contract.EXPECTED_STEPS:
        raise V6TeacherReplayExecutionError(
            f"worker step record count drifted: {case_id}"
        )
    for step, record in enumerate(steps, start=1):
        path = destination / "steps" / f"{step:06d}.json"
        if not _record_matches(record, path):
            raise V6TeacherReplayExecutionError(
                f"worker step record drifted: {case_id}/{step}"
            )
    for name in ("trace", "partial_summary", "summary"):
        if dict(persisted[name]) != dict(artifacts[name]):
            raise V6TeacherReplayExecutionError(
                f"persist-before-gate record drifted: {case_id}/{name}"
            )
    writer = forensic.ForensicRolloutWriter(
        destination,
        artifact_root=REPO_ROOT,
    )
    if writer.finalized_artifact_records() != dict(persisted):
        raise V6TeacherReplayExecutionError(
            f"step journal/final aggregate closure drifted: {case_id}"
        )
    summary = _mapping(expected_paths["summary"])
    partial_summary = _mapping(expected_paths["partial_summary"])
    expected_partial_summary = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": "H0_V6_V25_TEACHER_REPLAY_PERSISTED_BEFORE_GATE",
        "protocol_id": contract.PROTOCOL_ID,
        "case_id": case_id,
        "steps": summary.get("steps"),
        "expected_steps": contract.EXPECTED_STEPS,
        "end_reason": summary.get("end_reason"),
        "terminated": summary.get("terminated"),
        "truncated": summary.get("truncated"),
        "invariant_mismatch_count": summary.get("invariant_mismatch_count"),
        "teacher_mean_mismatch_count": summary.get("teacher_mean_mismatch_count"),
        "time_mismatch_count": summary.get("time_mismatch_count"),
        "step_contract_failure_count": summary.get("step_contract_failure_count"),
        "gate_evaluated": False,
        "candidate_created": False,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
    }
    if partial_summary != expected_partial_summary:
        raise V6TeacherReplayExecutionError(
            f"worker partial summary closure drifted: {case_id}"
        )
    observed_gate = _mapping(expected_paths["gate"])
    expected_gate = contract.replay_gate(summary)
    expected_gate["persisted_before_gate"] = dict(persisted)
    if (
        forensic.canonical_json_bytes(observed_gate)
        != forensic.canonical_json_bytes(expected_gate)
        or observed_gate.get("passed") is not True
    ):
        raise V6TeacherReplayExecutionError(
            f"worker gate/summary closure drifted: {case_id}"
        )
    return {
        "case_id": case_id,
        "receipt": _record(receipt_path),
        "gate": dict(artifacts["gate"]),
        "summary": dict(artifacts["summary"]),
    }


def _execution_claim_payload(token_sha256: str) -> dict[str, Any]:
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": "H0_V6_V25_TEACHER_REPLAY_EXECUTION_CLAIMED",
        "protocol_id": contract.PROTOCOL_ID,
        "collector_id": contract.COLLECTOR_ID,
        "execution_order": list(contract.CASE_IDS),
        "execution_token_sha256": token_sha256,
        "execution_lock": _record(LOCK),
        "authority": dict(contract.AUTHORITY),
        "candidate_created": False,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
    }


def verify_execution_claim(execution_token: str) -> dict[str, Any]:
    observed = _mapping(EXECUTION_CLAIM)
    expected = _execution_claim_payload(_token_sha256(execution_token))
    if forensic.canonical_json_bytes(observed) != forensic.canonical_json_bytes(
        expected
    ):
        raise V6TeacherReplayExecutionError("execution claim/token drifted")
    return observed


def _worker_claim_payload(
    *,
    case_id: str,
    execution_token_sha256: str,
    previous_receipts: Sequence[Mapping[str, Any]],
) -> dict[str, Any]:
    index = contract.CASE_IDS.index(case_id)
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": "H0_V6_V25_TEACHER_REPLAY_WORKER_CLAIMED",
        "protocol_id": contract.PROTOCOL_ID,
        "collector_id": contract.COLLECTOR_ID,
        "case_id": case_id,
        "case_index": index,
        "execution_token_sha256": execution_token_sha256,
        "execution_claim": _record(EXECUTION_CLAIM),
        "execution_lock": _record(LOCK),
        "previous_receipts": [dict(item) for item in previous_receipts],
        "candidate_created": False,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
    }


def verify_worker_claim(case_id: str, execution_token: str) -> dict[str, Any]:
    """Reject direct, out-of-order, post-failure, and post-ledger workers."""

    if os.path.lexists(EXECUTION_LEDGER):
        raise V6TeacherReplayExecutionError(
            "execution ledger already exists; no worker may run"
        )
    execution_claim = verify_execution_claim(execution_token)
    index = contract.CASE_IDS.index(case_id)
    previous_receipts = [
        verify_case_receipt(previous_id) for previous_id in contract.CASE_IDS[:index]
    ]
    expected = _worker_claim_payload(
        case_id=case_id,
        execution_token_sha256=execution_claim["execution_token_sha256"],
        previous_receipts=previous_receipts,
    )
    observed = _mapping(worker_claim_path(case_id))
    if forensic.canonical_json_bytes(observed) != forensic.canonical_json_bytes(
        expected
    ):
        raise V6TeacherReplayExecutionError(f"worker claim/order drifted: {case_id}")
    for later_case in contract.CASE_IDS[index + 1 :]:
        if os.path.lexists(canonical_destination(later_case)):
            raise V6TeacherReplayExecutionError(
                f"later worker destination already occupied: {later_case}"
            )
    return observed


def load_frozen_baseline(case_id: str) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    case = contract.canonical_case(case_id)
    preflight.validate_baseline_case(case_id)
    trace_path = preflight.resolve_relative(case["baseline_trace"])
    raw_rows = _sequence(trace_path)
    rows = [dict(row) for row in raw_rows if isinstance(row, Mapping)]
    if len(rows) != len(raw_rows) or len(rows) != contract.EXPECTED_STEPS:
        raise V6TeacherReplayExecutionError("frozen V5 baseline trace drifted")
    return case, rows


def exact_float32_columns(
    observed: Any,
    expected: Any,
    *,
    columns: Sequence[int],
    np: Any,
) -> tuple[bool, str, str]:
    """Compare selected float32 columns byte-for-byte and return digests."""

    left = np.asarray(observed, dtype=np.float32)
    right = np.asarray(expected, dtype=np.float32)
    indices = np.asarray(list(columns), dtype=np.int64)
    if (
        left.shape != (contract.EXPECTED_ACTOR_FEATURES,)
        or right.shape != (contract.EXPECTED_ACTOR_FEATURES,)
        or indices.shape != (len(columns),)
        or not np.all(np.isfinite(left))
        or not np.all(np.isfinite(right))
    ):
        raise V6TeacherReplayExecutionError(
            "cannot compare malformed actor observations"
        )
    left_slice = np.ascontiguousarray(left[indices], dtype=np.float32)
    right_slice = np.ascontiguousarray(right[indices], dtype=np.float32)
    return (
        left_slice.tobytes(order="C") == right_slice.tobytes(order="C"),
        array_sha256(left_slice),
        array_sha256(right_slice),
    )


def exact_float32_vector(
    observed: Any,
    expected: Any,
    *,
    length: int,
    np: Any,
) -> tuple[bool, str, str]:
    left = np.ascontiguousarray(np.asarray(observed, dtype=np.float32))
    right = np.ascontiguousarray(np.asarray(expected, dtype=np.float32))
    if (
        left.shape != (length,)
        or right.shape != (length,)
        or not np.all(np.isfinite(left))
        or not np.all(np.isfinite(right))
    ):
        raise V6TeacherReplayExecutionError("cannot compare malformed vectors")
    return (
        left.tobytes(order="C") == right.tobytes(order="C"),
        array_sha256(left),
        array_sha256(right),
    )


def build_env_config(case: Mapping[str, Any]) -> dict[str, Any]:
    translated = {
        "id": case["case_id"],
        "action_selection": case["action_selection"],
        "offset_s": case["episode_start_offset_s"],
        "seed": case["runtime_seed"],
    }
    previous_config = legacy.H0_CONFIG
    previous_profile = legacy.V25_PROFILE
    previous_analog = legacy.ANALOG_PROFILE
    try:
        legacy.H0_CONFIG = SOURCE_H0_CONFIG
        legacy.V25_PROFILE = INPUT_PATHS["v25_profile"]
        legacy.ANALOG_PROFILE = INPUT_PATHS["analog_teacher_profile"]
        result = legacy.build_env_config(case_id="C", condition=translated)
    finally:
        legacy.H0_CONFIG = previous_config
        legacy.V25_PROFILE = previous_profile
        legacy.ANALOG_PROFILE = previous_analog
    if (
        result.get("binary_phase_fsm_mode") != "binary_active"
        or result.get("binary_phase_event_contract_id")
        != contract.V25_ACTIVE_EVENT_CONTRACT_ID
        or result.get("phase_fsm_input_mode") != "legacy_events"
        or result.get("online_grf_applied_sides") != ["left"]
        or result.get("reward", {}).get("morphology_weight")
        != contract.MORPHOLOGY_WEIGHT
        or result.get("detector_sample_dt_s") != contract.EXPECTED_SAMPLE_DT_S
        or result.get("segment_duration") != contract.EXPECTED_POLICY_DT_S
        or result.get("episode_duration") != contract.EXPECTED_EPISODE_DURATION_S
    ):
        raise V6TeacherReplayExecutionError("V25-active environment routing drifted")
    return result


def _sea_fallback_count(payload: Any) -> int:
    if not isinstance(payload, Mapping) or not isinstance(
        payload.get("joints"), Mapping
    ):
        raise V6TeacherReplayExecutionError("SEA diagnostics are malformed")
    total = 0
    for joint in ("pros_knee_angle", "pros_ankle_angle"):
        values = payload["joints"].get(joint)
        if not isinstance(values, Mapping):
            raise V6TeacherReplayExecutionError(f"SEA diagnostics missing {joint}")
        for field in (
            "tau_input_plugin_fallback_count",
            "motor_accel_plugin_fallback_count",
        ):
            value = values.get(field)
            if type(value) is not int or value < 0:
                raise V6TeacherReplayExecutionError(
                    f"SEA fallback counter is malformed: {joint}.{field}"
                )
            total += value
    return total


def _step_failure_count(checks: Mapping[str, Any]) -> int:
    return int(not checks or not all(value is True for value in checks.values()))


def _worker_start_payload(
    *, case: Mapping[str, Any], lock: Mapping[str, Any]
) -> dict[str, Any]:
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": "H0_V6_V25_TEACHER_REPLAY_STARTED",
        "protocol_id": contract.PROTOCOL_ID,
        "collector_id": contract.COLLECTOR_ID,
        "case": dict(case),
        "behavior": "FROZEN_V5_RAW_ACTION_REPLAY",
        "observation": contract.TARGET_OBSERVATION_CONTRACT_ID,
        "teacher": contract.SOURCE_H0_ID,
        "invariant_columns": list(contract.INVARIANT_COLUMNS),
        "execution_lock": _record(LOCK),
        "execution_claim": _record(EXECUTION_CLAIM),
        "worker_claim": _record(worker_claim_path(str(case["case_id"]))),
        "baseline_trace": _record(preflight.resolve_relative(case["baseline_trace"])),
        "baseline_receipt": _record(
            preflight.resolve_relative(case["baseline_receipt"])
        ),
        "source_h0_module_state": lock["inputs"]["source_h0_module_state"],
        "v25_profile": lock["inputs"]["v25_profile"],
        "authority": dict(contract.AUTHORITY),
        "candidate_created": False,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
    }


def _execute_worker(
    *,
    case: Mapping[str, Any],
    baseline_rows: Sequence[Mapping[str, Any]],
    writer: forensic.ForensicRolloutWriter,
) -> dict[str, Any]:
    rollout_eval, np, torch, RLModule, env_factory, _reward = (
        legacy._load_inference_stack()
    )
    runtime_seed = int(case["runtime_seed"])
    np.random.seed(runtime_seed)
    torch.manual_seed(runtime_seed)
    module = RLModule.from_checkpoint(SOURCE_H0_MODULE.resolve())
    env_config = build_env_config(case)
    env = env_factory.make_cmc_env(env_config)
    rows: list[dict[str, Any]] = []
    raw_sample_count = 0
    control_window_count = 0
    unaccepted_so_count = 0
    sea_fallback_count = 0
    timeout_count = 0
    invalid_event_count = 0
    hard_invalid_count = 0
    nonfinite_count = 0
    action_clipped_values = 0
    invariant_mismatch_count = 0
    teacher_mean_mismatch_count = 0
    time_mismatch_count = 0
    step_contract_failure_count = 0
    penetrations: list[float] = []
    terminated = False
    truncated = False
    info: Mapping[str, Any] = {}
    actor_names: tuple[str, ...] = ()
    full_names: tuple[str, ...] = ()
    binary_events: dict[str, Any] | None = None
    started = time.monotonic()
    try:
        observation, reset_info = env.reset(seed=runtime_seed)
        observation = np.asarray(observation, dtype=np.float32)
        actor_names = tuple(str(name) for name in env.unwrapped.actor_feature_names)
        full_names = tuple(
            str(name) for name in env.unwrapped.observation_feature_names
        )
        rollout_eval._validate_module_observation_contract(
            module,
            actor_names,
            full_names,
        )
        if (
            observation.shape != (contract.EXPECTED_FULL_FEATURES,)
            or observation.dtype != np.dtype("float32")
            or actor_names != contract.EXPECTED_ACTOR_FEATURE_NAMES
            or full_names != contract.EXPECTED_OBSERVATION_FEATURE_NAMES
            or tuple(env.action_space.shape) != (contract.EXPECTED_ACTION_DIM,)
        ):
            raise V6TeacherReplayExecutionError(
                "runtime layout is not the frozen 35/84 float32 contract"
            )
        if (
            reset_info.get("binary_phase_fsm_executed") is not True
            or reset_info.get("binary_phase_fsm_mode") != "binary_active"
            or reset_info.get("binary_phase_event_contract_id")
            != contract.V25_ACTIVE_EVENT_CONTRACT_ID
        ):
            raise V6TeacherReplayExecutionError(
                "V25 active adapter was not executed at reset"
            )
        baseline_sensor = legacy._validate_raw_sample(
            reset_info.get("binary_phase_sensor_baseline"),
            float(reset_info.get("time")),
            "t0",
        )
        reset_fsm = reset_info.get("binary_phase_fsm")
        if (
            not isinstance(reset_fsm, Mapping)
            or reset_fsm.get("events_this_step") != []
        ):
            raise V6TeacherReplayExecutionError("V25 attributed an event to t0")
        binary_events = legacy._binary_event_accumulator(baseline_sensor)

        for step_index, baseline_row_raw in enumerate(baseline_rows):
            step = step_index + 1
            baseline_row = dict(baseline_row_raw)
            obs_before = np.ascontiguousarray(
                observation[: contract.EXPECTED_ACTOR_FEATURES],
                dtype=np.float32,
            )
            teacher_observation = np.ascontiguousarray(
                np.asarray(baseline_row["actor_observation"], dtype=np.float32)
            )
            frozen_teacher_mean = np.ascontiguousarray(
                np.asarray(baseline_row["mean_action"], dtype=np.float32)
            )
            raw_action = np.ascontiguousarray(
                np.asarray(baseline_row["raw_action"], dtype=np.float32)
            )
            teacher_full = np.asarray(observation, dtype=np.float32).copy()
            teacher_full[: contract.EXPECTED_ACTOR_FEATURES] = teacher_observation
            _raw, queried_teacher_mean, teacher_std, _innovation = (
                legacy._policy_values(
                    module=module,
                    obs=teacher_full,
                    action_shape=tuple(env.action_space.shape),
                    standard_normal=None,
                    stochastic=False,
                    rollout_eval=rollout_eval,
                )
            )
            queried_teacher_mean = np.ascontiguousarray(
                np.asarray(queried_teacher_mean, dtype=np.float32)
            )
            teacher_std = np.ascontiguousarray(
                np.asarray(teacher_std, dtype=np.float32)
            )
            invariant_exact, observed_invariant_sha, expected_invariant_sha = (
                exact_float32_columns(
                    obs_before,
                    teacher_observation,
                    columns=contract.INVARIANT_COLUMNS,
                    np=np,
                )
            )
            mean_exact, queried_mean_sha, frozen_mean_sha = exact_float32_vector(
                queried_teacher_mean,
                frozen_teacher_mean,
                length=contract.EXPECTED_ACTION_DIM,
                np=np,
            )
            action_finite = raw_action.shape == (2,) and bool(
                np.all(np.isfinite(raw_action))
            )
            if not action_finite:
                raise V6TeacherReplayExecutionError(
                    f"frozen raw action is malformed at step {step}"
                )
            applied_action = np.ascontiguousarray(
                np.clip(raw_action, env.action_space.low, env.action_space.high),
                dtype=np.float32,
            )
            action_exact, raw_action_sha, applied_action_sha = exact_float32_vector(
                raw_action,
                applied_action,
                length=contract.EXPECTED_ACTION_DIM,
                np=np,
            )
            action_clipped_values += int(np.count_nonzero(raw_action != applied_action))
            observation_after, reward, terminated, truncated, info = env.step(
                raw_action
            )
            observation_after = np.asarray(observation_after, dtype=np.float32)
            if not isinstance(info, Mapping):
                raise V6TeacherReplayExecutionError(
                    f"environment info is malformed at step {step}"
                )
            runtime_time_s = float(info.get("time"))
            baseline_time_s = float(baseline_row["time_s"])
            time_exact = (
                math.isfinite(runtime_time_s)
                and abs(runtime_time_s - baseline_time_s) <= TIME_TOLERANCE_S
            )
            samples = info.get("binary_phase_sensor_samples")
            if (
                not isinstance(samples, Sequence)
                or isinstance(samples, (str, bytes))
                or len(samples) != contract.EXPECTED_SAMPLES_PER_STEP
            ):
                raise V6TeacherReplayExecutionError(
                    f"step {step} lacks ten V25 sensor samples"
                )
            previous = (
                float(baseline_sensor["time_s"])
                + step_index * contract.EXPECTED_POLICY_DT_S
            )
            validated_samples = [
                legacy._validate_raw_sample(
                    sample,
                    previous + sample_index * contract.EXPECTED_SAMPLE_DT_S,
                    f"step {step} sample {sample_index}",
                )
                for sample_index, sample in enumerate(samples, start=1)
            ]
            raw_sample_count += len(validated_samples)
            legacy._accumulate_binary_events(
                binary_events,
                info=info,
                boundary_s=runtime_time_s,
            )
            routing_exact = (
                info.get("binary_phase_fsm_executed") is True
                and info.get("binary_phase_fsm_mode") == "binary_active"
                and info.get("binary_phase_event_contract_id")
                == contract.V25_ACTIVE_EVENT_CONTRACT_ID
                and info.get("online_grf_applied_sides") == ["left"]
            )
            reward_terms = info.get("reward_terms")
            if not isinstance(reward_terms, Mapping):
                raise V6TeacherReplayExecutionError("reward terms are missing")
            penetration = float(reward_terms.get("grf_penetration_m"))
            if not math.isfinite(penetration) or penetration < 0.0:
                raise V6TeacherReplayExecutionError("penetration is malformed")
            penetrations.append(penetration)
            phase = info.get("phase_fsm")
            if not isinstance(phase, Mapping):
                raise V6TeacherReplayExecutionError("phase FSM payload is missing")
            timeout_count += int(float(phase.get("timeout_exceeded", 0.0)) > 0.0)
            invalid_event_count = max(
                invalid_event_count,
                int(float(phase.get("invalid_event_count", 0.0))),
            )
            try:
                classified = so_recovery.classify_policy_step(
                    info.get("so_solver_audit_entries"),
                    policy_id=contract.SO_POLICY_ID,
                )
            except so_recovery.SORecoveryContractError as exc:
                raise V6TeacherReplayExecutionError(str(exc)) from exc
            counters = classified["counters"]
            control_window_count += int(counters["control_window_count"])
            unaccepted_so_count += int(
                counters["unaccepted_hard_so_fallback_count"]
                + counters["unaccepted_bounded_ls_count"]
            )
            sea_fallback_count += _sea_fallback_count(
                info.get("sea_segment_diagnostics")
            )
            hard_invalid_count += int("failure" in info)
            finite = bool(
                np.all(np.isfinite(obs_before))
                and np.all(np.isfinite(observation_after))
                and np.all(np.isfinite(queried_teacher_mean))
                and np.all(np.isfinite(teacher_std))
                and math.isfinite(float(reward))
            )
            nonfinite_count += int(not finite)
            checks = {
                "v25_observation_finite": finite,
                "invariant_columns_byte_exact": invariant_exact,
                "teacher_mean_byte_exact": mean_exact,
                "teacher_logstd_exact": bool(
                    teacher_std.shape == (2,)
                    and np.allclose(
                        teacher_std,
                        contract.STOCHASTIC_SIGMA,
                        rtol=0.0,
                        atol=1.0e-8,
                    )
                ),
                "frozen_raw_action_unclipped": action_exact,
                "runtime_time_matches_baseline": time_exact,
                "binary_active_routing_exact": routing_exact,
                "ten_v25_samples": len(validated_samples)
                == contract.EXPECTED_SAMPLES_PER_STEP,
                "ten_control_windows": counters["control_window_count"] == 10,
                "no_unaccepted_so": (
                    counters["unaccepted_hard_so_fallback_count"]
                    + counters["unaccepted_bounded_ls_count"]
                )
                == 0,
                "no_sea_fallback": _sea_fallback_count(
                    info.get("sea_segment_diagnostics")
                )
                == 0,
            }
            invariant_mismatch_count += int(not invariant_exact)
            teacher_mean_mismatch_count += int(not mean_exact)
            time_mismatch_count += int(not time_exact)
            step_contract_failure_count += _step_failure_count(checks)
            forensic_row = {
                "schema_version": contract.SCHEMA_VERSION,
                "case_id": case["case_id"],
                "baseline_time_s": baseline_time_s,
                "runtime_time_s": runtime_time_s,
                "v25_observation": obs_before.tolist(),
                "baseline_teacher_observation": teacher_observation.tolist(),
                "queried_teacher_mean": queried_teacher_mean.tolist(),
                "frozen_teacher_mean": frozen_teacher_mean.tolist(),
                "teacher_std": teacher_std.tolist(),
                "frozen_raw_action": raw_action.tolist(),
                "applied_action": applied_action.tolist(),
                "digests": {
                    "v25_invariant_columns": observed_invariant_sha,
                    "baseline_invariant_columns": expected_invariant_sha,
                    "queried_teacher_mean": queried_mean_sha,
                    "frozen_teacher_mean": frozen_mean_sha,
                    "frozen_raw_action": raw_action_sha,
                    "applied_action": applied_action_sha,
                },
                "checks": checks,
                "reward": float(reward),
                "reward_terms": legacy._jsonable(reward_terms),
                "terminated": bool(terminated),
                "truncated": bool(truncated),
                "end_reason": info.get("end_reason"),
                "binary_phase_sensor_samples": legacy._jsonable(validated_samples),
                "binary_phase_fsm": legacy._jsonable(info.get("binary_phase_fsm")),
                "binary_phase_active_adapter": legacy._jsonable(
                    info.get("binary_phase_active_adapter")
                ),
                "phase_fsm": legacy._jsonable(phase),
                "so_recovery_counters": legacy._jsonable(counters),
            }
            writer.write_step(step, forensic_row)
            rows.append({"step": step, **forensic_row})
            observation = observation_after
            completed = step
            if completed == 1 or completed % 10 == 0:
                elapsed = time.monotonic() - started
                eta = elapsed / completed * (contract.EXPECTED_STEPS - completed)
                print(
                    f"[V6/V25 teacher replay/{case['case_id']}] "
                    f"{completed:3d}/500 elapsed={elapsed:7.1f}s eta={eta:7.1f}s",
                    flush=True,
                )
            if terminated or truncated:
                break
    finally:
        env.close()

    phase = info.get("phase_fsm", {}) if isinstance(info, Mapping) else {}
    binary_event_gate = legacy._finalize_binary_event_gate(
        binary_events,
        raw_sample_count,
    )
    summary = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.ROLLOUT_COLLECTED_STATUS,
        "protocol_id": contract.PROTOCOL_ID,
        "collector_id": contract.COLLECTOR_ID,
        "case_id": case["case_id"],
        "action_selection": case["action_selection"],
        "episode_start_offset_s": case["episode_start_offset_s"],
        "action_seed": case["action_seed"],
        "runtime_seed": case["runtime_seed"],
        "source_h0_id": contract.SOURCE_H0_ID,
        "source_observation_contract_id": (contract.SOURCE_OBSERVATION_CONTRACT_ID),
        "target_observation_contract_id": (contract.TARGET_OBSERVATION_CONTRACT_ID),
        "behavior": "FROZEN_V5_RAW_ACTION_REPLAY",
        "steps": len(rows),
        "control_window_count": control_window_count,
        "v25_raw_sensor_sample_count": raw_sample_count,
        "end_reason": info.get("end_reason") if isinstance(info, Mapping) else None,
        "terminated": bool(terminated),
        "truncated": bool(truncated),
        "phase_valid_cycle_count": int(float(phase.get("valid_cycle_count", 0))),
        "grf_penetration_max_m": max(penetrations, default=0.0),
        "invariant_mismatch_count": invariant_mismatch_count,
        "teacher_mean_mismatch_count": teacher_mean_mismatch_count,
        "time_mismatch_count": time_mismatch_count,
        "step_contract_failure_count": step_contract_failure_count,
        "action_clipped_values": action_clipped_values,
        "timeout_count": timeout_count,
        "safety_stop_count": int(bool(terminated)),
        "invalid_event_count": invalid_event_count,
        "hard_invalid_count": hard_invalid_count,
        "nonfinite_count": nonfinite_count,
        "so_solver_unaccepted_count": unaccepted_so_count,
        "sea_plugin_fallback_count": sea_fallback_count,
        "binary_phase_fsm_mode": "binary_active",
        "binary_phase_event_contract_id": (contract.V25_ACTIVE_EVENT_CONTRACT_ID),
        "binary_phase_event_gate": binary_event_gate,
        "morphology_weight": env_config["reward"]["morphology_weight"],
        "n_actor": len(actor_names),
        "n_observation": len(full_names),
        "observation_dtype": contract.EXPECTED_OBSERVATION_DTYPE,
        "actor_feature_names": list(actor_names),
        "observation_feature_names": list(full_names),
        "invariant_columns": list(contract.INVARIANT_COLUMNS),
        "baseline_trace": _record(preflight.resolve_relative(case["baseline_trace"])),
        "source_h0_module_state": _record(INPUT_PATHS["source_h0_module_state"]),
        "v25_profile": _record(INPUT_PATHS["v25_profile"]),
        "candidate_created": False,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
    }
    partial_summary = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": "H0_V6_V25_TEACHER_REPLAY_PERSISTED_BEFORE_GATE",
        "protocol_id": contract.PROTOCOL_ID,
        "case_id": case["case_id"],
        "steps": len(rows),
        "expected_steps": contract.EXPECTED_STEPS,
        "end_reason": summary["end_reason"],
        "terminated": summary["terminated"],
        "truncated": summary["truncated"],
        "invariant_mismatch_count": invariant_mismatch_count,
        "teacher_mean_mismatch_count": teacher_mean_mismatch_count,
        "time_mismatch_count": time_mismatch_count,
        "step_contract_failure_count": step_contract_failure_count,
        "gate_evaluated": False,
        "candidate_created": False,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
    }
    persisted = writer.finalize_before_gate(
        trace=rows,
        partial_summary=partial_summary,
        summary=summary,
    )

    def evaluate_gate(records: dict[str, Any]) -> dict[str, Any]:
        gate = contract.replay_gate(summary)
        gate["persisted_before_gate"] = records
        return gate

    gate_record = writer.run_gate(evaluate_gate)
    gate = _mapping(writer.gate_path)
    if gate.get("passed") is not True:
        writer.publish_failure(
            end_reason="development_gate_failed",
            error={
                "type": "V6TeacherReplayGateFailure",
                "message": f"teacher replay gate failed for {case['case_id']}",
            },
            status=contract.ROLLOUT_FAIL_STATUS,
            details={"gate": gate_record},
        )
        raise V6TeacherReplayExecutionError(
            f"teacher replay gate failed for {case['case_id']}"
        )
    receipt = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.ROLLOUT_PASS_STATUS,
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "case_id": case["case_id"],
        "artifacts": writer.artifact_records(),
        "persisted_before_gate": persisted,
        "execution_lock": _record(LOCK),
        "execution_claim": _record(EXECUTION_CLAIM),
        "worker_claim": _record(worker_claim_path(str(case["case_id"]))),
        "candidate_created": False,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
    }
    receipt_path = forensic.write_json_exclusive(
        writer.run_directory / "receipt.json",
        receipt,
    )
    return {**receipt, "receipt": _record(receipt_path)}


def run_worker(
    *,
    case_id: str,
    output_dir: str | Path,
    execution_token: str,
) -> dict[str, Any]:
    lock = verify_lock()
    verify_worker_claim(case_id, execution_token)
    case, baseline_rows = load_frozen_baseline(case_id)
    expected = canonical_destination(case_id)
    observed = Path(output_dir).expanduser().resolve()
    if observed != expected:
        raise V6TeacherReplayExecutionError(
            f"non-canonical worker destination: {observed} != {expected}"
        )
    writer = forensic.ForensicRolloutWriter(
        expected,
        artifact_root=REPO_ROOT,
    )
    try:
        writer.start(_worker_start_payload(case=case, lock=lock))
        return _execute_worker(
            case=case,
            baseline_rows=baseline_rows,
            writer=writer,
        )
    except Exception as exc:
        if writer.run_start_path.is_file() and not os.path.lexists(writer.failure_path):
            try:
                writer.publish_failure(
                    end_reason="worker_exception",
                    error=exc,
                    status=contract.ROLLOUT_FAIL_STATUS,
                    details={
                        "case_id": case_id,
                        "candidate_created": False,
                        "actor_updates": 0,
                        "critic_updates": 0,
                        "ppo_updates": 0,
                        "protected_trials_opened": [],
                    },
                )
            except Exception as receipt_error:
                raise V6TeacherReplayExecutionError(
                    f"worker failed and failure receipt failed: {receipt_error}"
                ) from exc
        raise


def _worker_command(case_id: str, execution_token: str) -> list[str]:
    return [
        sys.executable,
        str(Path(__file__).resolve()),
        "--worker",
        "--case",
        case_id,
        "--output-dir",
        str(canonical_destination(case_id)),
        "--execution-token",
        execution_token,
    ]


def execute() -> dict[str, Any]:
    verify_lock(require_run_root_absent=True)
    started = time.time()
    execution_token = secrets.token_urlsafe(32)
    token_sha256 = _token_sha256(execution_token)
    forensic.write_json_exclusive(
        EXECUTION_CLAIM,
        _execution_claim_payload(token_sha256),
    )
    status = contract.PROTOCOL_FAIL_STATUS
    passed = False
    error = None
    completed_cases: list[str] = []
    completed_receipts: list[dict[str, Any]] = []
    try:
        for case_id in contract.CASE_IDS:
            previous_receipts = [
                verify_case_receipt(previous_id) for previous_id in completed_cases
            ]
            forensic.write_json_exclusive(
                worker_claim_path(case_id),
                _worker_claim_payload(
                    case_id=case_id,
                    execution_token_sha256=token_sha256,
                    previous_receipts=previous_receipts,
                ),
            )
            completed = subprocess.run(
                _worker_command(case_id, execution_token),
                cwd=REPO_ROOT,
                timeout=WORKER_TIMEOUT_S,
                check=False,
            )
            if completed.returncode != 0:
                raise V6TeacherReplayExecutionError(
                    f"worker {case_id} exited {completed.returncode}"
                )
            receipt = verify_case_receipt(case_id)
            completed_cases.append(case_id)
            completed_receipts.append(receipt)
        status = contract.PROTOCOL_PASS_STATUS
        passed = True
    except Exception as exc:
        error = f"{type(exc).__name__}: {exc}"
    ledger = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": status,
        "passed": passed,
        "protocol_id": contract.PROTOCOL_ID,
        "started_unix_s": started,
        "completed_unix_s": time.time(),
        "completed_cases": completed_cases,
        "completed_receipts": completed_receipts,
        "expected_cases": list(contract.CASE_IDS),
        "error": error,
        "execution_lock": _record(LOCK),
        "execution_claim": _record(EXECUTION_CLAIM),
        "candidate_created": False,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "next_stage": (
            "BUILD_V6_RESIDUAL_DATASET" if passed else "STOP_WITHOUT_RETRY_OR_RETUNING"
        ),
    }
    forensic.write_json_exclusive(EXECUTION_LEDGER, ledger)
    print(json.dumps(ledger, indent=2, sort_keys=True, allow_nan=False), flush=True)
    if not passed:
        raise V6TeacherReplayExecutionError(error or status)
    return ledger


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--worker", action="store_true")
    mode.add_argument("--execute", action="store_true")
    parser.add_argument("--case", choices=contract.CASE_IDS)
    parser.add_argument("--output-dir")
    parser.add_argument("--execution-token")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        if args.execute:
            result = execute()
        else:
            if (
                args.case is None
                or args.output_dir is None
                or args.execution_token is None
            ):
                raise V6TeacherReplayExecutionError(
                    "--case, --output-dir and supervisor execution token are "
                    "required for a worker"
                )
            result = run_worker(
                case_id=args.case,
                output_dir=args.output_dir,
                execution_token=args.execution_token,
            )
    except Exception as exc:
        print(
            f"V6 teacher replay failed closed: {type(exc).__name__}: {exc}",
            file=sys.stderr,
        )
        return 2
    print(json.dumps(result, indent=2, sort_keys=True, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
