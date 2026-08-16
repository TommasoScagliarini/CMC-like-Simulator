"""Prepare and collect the V8/V26 H0 teacher-action replay corpus.

The numerical collector deliberately reuses the audited V6 persistence engine,
but this entry point replaces every protocol/path/runtime binding before either
the supervisor or a subprocess worker can run.  Workers are always spawned
through this V8 file, so the binding is reapplied in the child interpreter.

``--prepare`` publishes a strict no-clobber preflight receipt and execution
lock.  ``--execute`` then runs the six frozen V5 action tapes sequentially in
the exact V26-active runtime.  Protected and reserve trials are never opened.
"""

from __future__ import annotations

import argparse
import copy
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
for import_root in (REPO_ROOT, VALIDATION_ROOT, BASELINE_ROOT, TRAJECTORY_ROOT):
    if str(import_root) not in sys.path:
        sys.path.insert(0, str(import_root))

from validation import h0_forensic_rollout as forensic  # noqa: E402
from validation import h0_primary_grf_split_v8_teacher_replay_contract as contract  # noqa: E402
from validation import run_h0_primary_grf_split_v6_teacher_replay as engine  # noqa: E402
from validation import build_h0_primary_grf_split_v6_teacher_replay_preflight as v6_preflight  # noqa: E402


class V8TeacherReplayExecutionError(RuntimeError):
    """Raised on any V8 provenance, routing, persistence, or gate failure."""


def resolve_relative(relative: str | PurePosixPath) -> Path:
    value = PurePosixPath(relative)
    if value.is_absolute() or ".." in value.parts or not value.parts:
        raise V8TeacherReplayExecutionError(
            f"non-canonical repository path: {value}"
        )
    return REPO_ROOT.joinpath(*value.parts)


LOCK = resolve_relative(contract.LOCK_PATH)
PREFLIGHT = resolve_relative(contract.PREFLIGHT_RECEIPT_PATH)
RUN_ROOT = resolve_relative(contract.RUN_ROOT)
EXECUTION_LEDGER = resolve_relative(contract.EXECUTION_LEDGER_PATH)
EXECUTION_CLAIM = resolve_relative(contract.EXECUTION_CLAIM_PATH)
WORKER_CLAIMS_ROOT = resolve_relative(contract.WORKER_CLAIMS_ROOT)
WORKER_TIMEOUT_S = 2400.0


def _mapping(path: Path) -> dict[str, Any]:
    value = forensic.strict_json_load(path)
    if not isinstance(value, Mapping):
        raise V8TeacherReplayExecutionError(f"expected JSON object: {path}")
    return dict(value)


def _record(path: str | Path) -> dict[str, Any]:
    return forensic.artifact_record(path, artifact_root=REPO_ROOT)


def _record_matches(record: Any, path: Path) -> bool:
    return (
        isinstance(record, Mapping)
        and set(record) == {"path", "sha256", "size_bytes"}
        and dict(record) == _record(path)
    )


def source_paths() -> dict[str, Path]:
    return {
        name: resolve_relative(relative)
        for name, relative in contract.SOURCE_RELATIVE_PATHS.items()
    }


def input_paths() -> dict[str, Path]:
    result = {
        name: resolve_relative(relative)
        for name, relative in contract.INPUT_RELATIVE_PATHS.items()
    }
    for case in contract.CASES:
        case_id = str(case["case_id"])
        for artifact in ("trace", "summary", "receipt"):
            result[f"v5_baseline_{case_id}_{artifact}"] = resolve_relative(
                str(case[f"baseline_{artifact}"])
            )
        baseline_root = resolve_relative(str(case["baseline_receipt"])).parent
        result[f"v5_baseline_{case_id}_common_gate"] = (
            baseline_root / "common_gate.json"
        )
        result[f"v5_baseline_{case_id}_solver_audit_journal"] = (
            baseline_root / "solver_audit_journal.json"
        )
    return result


SOURCE_PATHS = source_paths()
INPUT_PATHS = input_paths()
SOURCE_H0_MODULE = INPUT_PATHS["source_h0_module_state"].parent
SOURCE_H0_CONFIG = INPUT_PATHS["source_h0_config"]


def _receipt_source_matches(receipt_record: Any, path: Path) -> bool:
    actual = _record(path)
    return bool(
        isinstance(receipt_record, Mapping)
        and receipt_record.get("path") == actual["path"]
        and receipt_record.get("sha256") == actual["sha256"]
    )


def validate_v26_lineage() -> dict[str, Any]:
    development_path = INPUT_PATHS["v26_development_receipt"]
    replay_path = INPUT_PATHS["v26_v7_replay_receipt"]
    profile_path = INPUT_PATHS["v25_profile"]
    freeze_path = INPUT_PATHS["v25_candidate_freeze"]
    development = _mapping(development_path)
    replay = _mapping(replay_path)
    freeze = _mapping(freeze_path)
    candidate = development.get("candidate")
    development_sources = development.get("sources")
    replay_sources = replay.get("sources")
    frozen_candidate = freeze.get("candidate")
    data_access = development.get("data_access")
    replay_access = replay.get("data_access")
    if (
        development.get("schema_version") != 26
        or development.get("status") != "V26_DEVELOPMENT_READY"
        or development.get("pass") is not True
        or not isinstance(candidate, Mapping)
        or candidate.get("event_contract_id")
        != contract.V26_ACTIVE_EVENT_CONTRACT_ID
        or candidate.get("source") != contract.V26_FSM_SOURCE
        or candidate.get("geometry") != "V25 frozen force-free binary points"
        or candidate.get("retuning_performed") is not False
        or not isinstance(development_sources, Mapping)
        or not _receipt_source_matches(
            development_sources.get("v26_fsm"),
            SOURCE_PATHS["v26_binary_phase_fsm"],
        )
        or not _receipt_source_matches(
            development_sources.get("profile"), profile_path
        )
        or not isinstance(data_access, Mapping)
        or data_access.get("protected_trials_opened") != []
        or data_access.get("reserve_trials_opened") != []
    ):
        raise V8TeacherReplayExecutionError(
            "V26 development receipt or source closure drifted"
        )
    if (
        replay.get("schema_version") != 26
        or replay.get("status") != "V26_V7_REPLAY_PASS"
        or replay.get("pass") is not True
        or replay.get("expected_events_exact") is not True
        or replay.get("forbidden_v20_toe_only_pair_absent") is not True
        or replay.get("complete_cycle_count") != 2
        or not isinstance(replay.get("minimum_functional_flight_s"), (int, float))
        or float(replay["minimum_functional_flight_s"]) < 0.25
        or replay.get("historical_v7_status_preserved")
        != "TERMINAL_FAIL_NO_REINTERPRETATION"
        or not isinstance(replay_sources, Mapping)
        or not _receipt_source_matches(
            replay_sources.get("v26_fsm"), SOURCE_PATHS["v26_binary_phase_fsm"]
        )
        or not isinstance(replay_access, Mapping)
        or replay_access.get("simulation_rerun") is not False
        or replay_access.get("protected_trials_opened") != []
        or replay_access.get("reserve_trials_opened") != []
        or replay_access.get("ppo_updates") != 0
    ):
        raise V8TeacherReplayExecutionError(
            "V26 replay of the terminal V7 journal drifted"
        )
    if (
        freeze.get("schema_version") != 25
        or freeze.get("status")
        != "V25_DEVELOPMENT_CANDIDATE_FROZEN_H0_PROTOCOL_REQUIRED"
        or freeze.get("pass") is not True
        or not isinstance(frozen_candidate, Mapping)
        or frozen_candidate.get("candidate_id") != "v25_4b351f67b5b86ab0"
        or not _receipt_source_matches(frozen_candidate.get("profile"), profile_path)
    ):
        raise V8TeacherReplayExecutionError("V25 geometry freeze drifted")
    return {
        "event_contract_id": contract.V26_ACTIVE_EVENT_CONTRACT_ID,
        "fsm_source": contract.V26_FSM_SOURCE,
        "actor_adapter_source": contract.V26_ACTOR_ADAPTER_SOURCE,
        "candidate_id": "v25_4b351f67b5b86ab0",
        "profile": _record(profile_path),
        "v25_freeze": _record(freeze_path),
        "v26_development_receipt": _record(development_path),
        "v26_v7_replay_receipt": _record(replay_path),
        "historical_v7_status": "TERMINAL_FAIL_NO_REINTERPRETATION",
    }


def build_preflight_payload(*, require_unoccupied: bool) -> dict[str, Any]:
    sources = {name: _record(path) for name, path in SOURCE_PATHS.items()}
    inputs = {name: _record(path) for name, path in INPUT_PATHS.items()}
    v5_terminal = v6_preflight.validate_v5_terminal()
    source_h0 = v6_preflight.validate_source_h0_layout()
    baseline_cases = [
        v6_preflight.validate_baseline_case(case_id)
        for case_id in contract.CASE_IDS
    ]
    for item in baseline_cases:
        item["reclassification"] = "V5_PASS_BASELINE_TO_V8_DEVELOPMENT_ONLY"
    v26 = validate_v26_lineage()
    destinations = [
        resolve_relative(str(case["destination"])) for case in contract.CASES
    ]
    checks = {
        "all_sources_present_and_hashed": len(sources)
        == len(contract.SOURCE_RELATIVE_PATHS),
        "all_inputs_present_and_hashed": len(inputs) == len(INPUT_PATHS),
        "v5_terminal_history_preserved": v5_terminal["status"]
        == "FAIL_H0_PRIMARY_SPLIT_V5_AUTONOMOUS_QUALIFICATION",
        "six_v5_pass_baselines_reclassified_development": len(baseline_cases) == 6
        and all(item["rows"] == contract.EXPECTED_STEPS for item in baseline_cases),
        "source_h0_original": source_h0["source_h0_id"]
        == contract.SOURCE_H0_ID,
        "v26_development_ready": v26["event_contract_id"]
        == contract.V26_ACTIVE_EVENT_CONTRACT_ID,
        "v7_terminal_history_preserved": v26["historical_v7_status"]
        == "TERMINAL_FAIL_NO_REINTERPRETATION",
        "target_contract_exact": contract.TARGET_OBSERVATION_CONTRACT_ID
        == "primary_grf_split_v1+binary_point_v25+heel_qualified_fsm_v2",
        "invariant_columns_exact": list(contract.INVARIANT_COLUMNS)
        == [*range(2, 10), *range(25, 35)],
        "v8_authorized": contract.AUTHORITY["v8_protocol_authorized"],
        "updates_forbidden": not any(
            contract.AUTHORITY[key]
            for key in (
                "actor_updates_authorized",
                "critic_updates_authorized",
                "ppo_updates_authorized",
            )
        ),
        "protected_closed": not contract.AUTHORITY[
            "protected_trial_access_authorized"
        ],
        "reserve_closed": not contract.AUTHORITY["reserve_trial_access_authorized"],
        "destinations_unoccupied": not any(
            os.path.lexists(path) for path in destinations
        ),
        "run_root_unoccupied": not os.path.lexists(RUN_ROOT),
        "execution_lock_unoccupied": not os.path.lexists(LOCK),
        "preflight_unoccupied": not os.path.lexists(PREFLIGHT),
    }
    if require_unoccupied and not all(checks.values()):
        failed = [name for name, passed in checks.items() if not passed]
        raise V8TeacherReplayExecutionError(f"V8 preflight failed: {failed}")
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.PREFLIGHT_STATUS,
        "passed": all(checks.values()),
        "protocol_id": contract.PROTOCOL_ID,
        "collector_id": contract.COLLECTOR_ID,
        "revision": contract.REVISION,
        "checks": checks,
        "v5_terminal": v5_terminal,
        "source_h0": source_h0,
        "v26": v26,
        "cases": baseline_cases,
        "sources": sources,
        "inputs": inputs,
        "destinations": [path.relative_to(REPO_ROOT).as_posix() for path in destinations],
        "authority": copy.deepcopy(contract.AUTHORITY),
        "simulations_executed": 0,
        "candidate_created": False,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
        "next_stage": "FREEZE_V8_V26_TEACHER_REPLAY_EXECUTION",
    }


def _lock_payload(preflight_payload: Mapping[str, Any]) -> dict[str, Any]:
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.LOCK_STATUS,
        "protocol_id": contract.PROTOCOL_ID,
        "collector_id": contract.COLLECTOR_ID,
        "revision": contract.REVISION,
        "run_root": contract.RUN_ROOT.as_posix(),
        "execution_ledger": contract.EXECUTION_LEDGER_PATH.as_posix(),
        "execution_claim": contract.EXECUTION_CLAIM_PATH.as_posix(),
        "worker_claims_root": contract.WORKER_CLAIMS_ROOT.as_posix(),
        "execution_order": list(contract.CASE_IDS),
        "cases": [dict(case) for case in contract.CASES],
        "matrix": {
            "rollout_count": len(contract.CASES),
            "destinations": [str(case["destination"]) for case in contract.CASES],
            "behavior": "FROZEN_V5_RAW_ACTION_REPLAY",
            "observation": contract.TARGET_OBSERVATION_CONTRACT_ID,
            "teacher": contract.SOURCE_H0_ID,
        },
        "invariant_columns": {
            "ranges_half_open": [
                list(bounds) for bounds in contract.INVARIANT_COLUMN_RANGES
            ],
            "indices": list(contract.INVARIANT_COLUMNS),
            "comparison": "FLOAT32_C_CONTIGUOUS_BYTES_EXACT",
        },
        "gate": {
            "expected_steps": contract.EXPECTED_STEPS,
            "expected_control_windows": contract.EXPECTED_CONTROL_WINDOWS,
            "expected_v25_raw_sensor_samples": contract.EXPECTED_RAW_SENSOR_SAMPLES,
            "minimum_valid_cycles": contract.MINIMUM_VALID_CYCLES,
            "penetration_limit_m": contract.PENETRATION_LIMIT_M,
            "penetration_comparison": "strict_less_than",
            "persist_before_gate": ["trace.json", "partial_summary.json", "summary.json"],
        },
        "preflight_receipt": _record(PREFLIGHT),
        "sources": dict(preflight_payload["sources"]),
        "inputs": dict(preflight_payload["inputs"]),
        "v5_terminal": dict(preflight_payload["v5_terminal"]),
        "v26": dict(preflight_payload["v26"]),
        "source_h0": dict(preflight_payload["source_h0"]),
        "development_reclassification": {
            "source": "V5_PASS_CONDITION_MATCHED_BASELINES",
            "case_ids": list(contract.CASE_IDS),
            "new_role": "V8_V26_DEVELOPMENT_TEACHER_ACTION_REPLAY",
            "eligible_as_future_holdout": False,
        },
        "authority": copy.deepcopy(contract.AUTHORITY),
        "simulations_executed_at_freeze": 0,
        "candidate_created": False,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
        "next_stage": "EXECUTE_SIX_V26_ACTIVE_DEVELOPMENT_REPLAYS_ONCE",
    }


def prepare() -> dict[str, Any]:
    payload = build_preflight_payload(require_unoccupied=True)
    forensic.write_json_exclusive(PREFLIGHT, payload)
    lock = _lock_payload(payload)
    forensic.write_json_exclusive(LOCK, lock)
    return {"preflight": payload, "lock": lock}


def verify_lock(*, require_run_root_absent: bool = False) -> dict[str, Any]:
    lock = _mapping(LOCK)
    receipt = _mapping(PREFLIGHT)
    if (
        lock.get("schema_version") != contract.SCHEMA_VERSION
        or lock.get("status") != contract.LOCK_STATUS
        or lock.get("protocol_id") != contract.PROTOCOL_ID
        or lock.get("collector_id") != contract.COLLECTOR_ID
        or lock.get("revision") != contract.REVISION
        or lock.get("run_root") != contract.RUN_ROOT.as_posix()
        or lock.get("execution_order") != list(contract.CASE_IDS)
        or lock.get("cases") != [dict(case) for case in contract.CASES]
        or lock.get("authority") != contract.AUTHORITY
        or lock.get("v26") != receipt.get("v26")
        or lock.get("source_h0") != receipt.get("source_h0")
        or lock.get("simulations_executed_at_freeze") != 0
        or lock.get("candidate_created") is not False
        or lock.get("actor_updates") != 0
        or lock.get("critic_updates") != 0
        or lock.get("ppo_updates") != 0
        or lock.get("protected_trials_opened") != []
        or lock.get("reserve_trials_opened") != []
        or lock.get("next_stage")
        != "EXECUTE_SIX_V26_ACTIVE_DEVELOPMENT_REPLAYS_ONCE"
        or receipt.get("status") != contract.PREFLIGHT_STATUS
        or receipt.get("passed") is not True
        or receipt.get("protected_trials_opened") != []
        or receipt.get("reserve_trials_opened") != []
        or not _record_matches(lock.get("preflight_receipt"), PREFLIGHT)
    ):
        raise V8TeacherReplayExecutionError("V8 execution lock identity drifted")
    for label, paths in (("sources", SOURCE_PATHS), ("inputs", INPUT_PATHS)):
        records = lock.get(label)
        if not isinstance(records, Mapping) or set(records) != set(paths):
            raise V8TeacherReplayExecutionError(f"lock {label} closure drifted")
        for name, path in paths.items():
            if not _record_matches(records[name], path):
                raise V8TeacherReplayExecutionError(
                    f"lock {label}.{name} drifted"
                )
    if require_run_root_absent and os.path.lexists(RUN_ROOT):
        raise V8TeacherReplayExecutionError(f"run root already claimed: {RUN_ROOT}")
    return lock


def canonical_destination(case_id: str) -> Path:
    return resolve_relative(contract.canonical_case(case_id)["destination"])


def worker_claim_path(case_id: str) -> Path:
    try:
        index = contract.CASE_IDS.index(case_id)
    except ValueError as exc:
        raise V8TeacherReplayExecutionError(f"unknown case: {case_id!r}") from exc
    return WORKER_CLAIMS_ROOT / f"{index + 1:02d}_{case_id}.json"


def _token_sha256(token: str) -> str:
    if not isinstance(token, str) or len(token) < 32:
        raise V8TeacherReplayExecutionError("execution token is malformed")
    import hashlib

    return hashlib.sha256(token.encode("utf-8")).hexdigest()


def _execution_claim_payload(token_sha256: str) -> dict[str, Any]:
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": "H0_V8_V26_TEACHER_REPLAY_EXECUTION_CLAIMED",
        "protocol_id": contract.PROTOCOL_ID,
        "collector_id": contract.COLLECTOR_ID,
        "execution_order": list(contract.CASE_IDS),
        "execution_token_sha256": token_sha256,
        "execution_lock": _record(LOCK),
        "authority": copy.deepcopy(contract.AUTHORITY),
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
        raise V8TeacherReplayExecutionError("execution claim/token drifted")
    return observed


def _worker_claim_payload(
    *,
    case_id: str,
    execution_token_sha256: str,
    previous_receipts: Sequence[Mapping[str, Any]],
) -> dict[str, Any]:
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": "H0_V8_V26_TEACHER_REPLAY_WORKER_CLAIMED",
        "protocol_id": contract.PROTOCOL_ID,
        "collector_id": contract.COLLECTOR_ID,
        "case_id": case_id,
        "case_index": contract.CASE_IDS.index(case_id),
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
    if os.path.lexists(EXECUTION_LEDGER):
        raise V8TeacherReplayExecutionError(
            "execution ledger already exists; no worker may run"
        )
    claim = verify_execution_claim(execution_token)
    index = contract.CASE_IDS.index(case_id)
    previous_receipts = [
        engine.verify_case_receipt(previous_id)
        for previous_id in contract.CASE_IDS[:index]
    ]
    expected = _worker_claim_payload(
        case_id=case_id,
        execution_token_sha256=claim["execution_token_sha256"],
        previous_receipts=previous_receipts,
    )
    observed = _mapping(worker_claim_path(case_id))
    if forensic.canonical_json_bytes(observed) != forensic.canonical_json_bytes(
        expected
    ):
        raise V8TeacherReplayExecutionError(f"worker claim drifted: {case_id}")
    for later_case in contract.CASE_IDS[index + 1 :]:
        if os.path.lexists(canonical_destination(later_case)):
            raise V8TeacherReplayExecutionError(
                f"later worker destination already occupied: {later_case}"
            )
    return observed


def build_env_config(case: Mapping[str, Any]) -> dict[str, Any]:
    translated = {
        "id": case["case_id"],
        "action_selection": case["action_selection"],
        "offset_s": case["episode_start_offset_s"],
        "seed": case["runtime_seed"],
    }
    legacy = engine.legacy
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
    result["binary_phase_fsm_mode"] = "binary_active"
    result["binary_phase_event_contract_id"] = (
        contract.V26_ACTIVE_EVENT_CONTRACT_ID
    )
    if (
        result.get("phase_fsm_input_mode") != "legacy_events"
        or result.get("online_grf_applied_sides") != ["left"]
        or result.get("reward", {}).get("morphology_weight")
        != contract.MORPHOLOGY_WEIGHT
        or result.get("detector_sample_dt_s") != contract.EXPECTED_SAMPLE_DT_S
        or result.get("segment_duration") != contract.EXPECTED_POLICY_DT_S
        or result.get("episode_duration") != contract.EXPECTED_EPISODE_DURATION_S
    ):
        raise V8TeacherReplayExecutionError(
            "V26-active environment routing drifted"
        )
    return result


def _accumulate_binary_events_v26(
    accumulator: dict[str, Any],
    *,
    info: Mapping[str, Any],
    boundary_s: float,
) -> None:
    payload = info.get("binary_phase_fsm")
    if not isinstance(payload, Mapping):
        raise V8TeacherReplayExecutionError("missing V26 binary FSM payload")
    events = payload.get("events_this_step")
    if not isinstance(events, Sequence) or isinstance(events, (str, bytes)):
        raise V8TeacherReplayExecutionError("V26 events_this_step is malformed")
    accumulator.setdefault("left_non_v26_source_count", 0)
    for raw in events:
        if not isinstance(raw, Mapping):
            raise V8TeacherReplayExecutionError("V26 event is malformed")
        name = str(raw.get("event", ""))
        event_time = float(raw.get("event_time_s"))
        confirmed = float(raw.get("confirmed_time_s"))
        delivered = float(raw.get("delivered_time_s"))
        identity = (name, event_time, confirmed, delivered)
        if identity in accumulator["identities"]:
            accumulator["duplicate_event_count"] += 1
        accumulator["identities"].add(identity)
        if name != accumulator["expected_event"]:
            accumulator["out_of_order_event_count"] += 1
        accumulator["expected_event"] = (
            "toe_off" if name == "heel_strike" else "heel_strike"
        )
        if (
            name not in {"heel_strike", "toe_off"}
            or raw.get("source") != contract.V26_FSM_SOURCE
            or raw.get("event_contract_id")
            != contract.V26_ACTIVE_EVENT_CONTRACT_ID
            or not all(
                math.isfinite(value) for value in (event_time, confirmed, delivered)
            )
            or abs((confirmed - event_time) - 0.005) > 1.0e-9
            or delivered < confirmed - 1.0e-9
            or delivered - confirmed > 0.010 + 1.0e-9
            or abs(delivered - boundary_s) > 1.0e-9
        ):
            accumulator["hard_invalid_count"] += 1
        if (
            len(accumulator["events"]) == 0
            and name == "toe_off"
            and raw.get("startup_partial_stance") is not True
        ):
            accumulator["hard_invalid_count"] += 1
        accumulator["events"].append(engine.legacy._jsonable(raw))
    active_events = info.get("online_events", [])
    if not isinstance(active_events, Sequence) or isinstance(
        active_events, (str, bytes)
    ):
        raise V8TeacherReplayExecutionError("active event stream is malformed")
    for event in active_events:
        if not isinstance(event, Mapping):
            raise V8TeacherReplayExecutionError("active event is malformed")
        if (
            str(event.get("side", "")).lower() == "left"
            and event.get("source") != contract.V26_ACTOR_ADAPTER_SOURCE
        ):
            accumulator["left_non_v26_source_count"] += 1


def _finalize_binary_event_gate_v26(
    accumulator: Mapping[str, Any], sample_count: int
) -> dict[str, Any]:
    result = {
        "sample_count": sample_count,
        "event_count": len(accumulator["events"]),
        "events": list(accumulator["events"]),
        "duplicate_event_count": int(accumulator["duplicate_event_count"]),
        "out_of_order_event_count": int(accumulator["out_of_order_event_count"]),
        "left_non_v26_source_count": int(
            accumulator.get("left_non_v26_source_count", 0)
        ),
        "fallback_count": int(accumulator["fallback_count"]),
        "hard_invalid_count": int(accumulator["hard_invalid_count"]),
    }
    result["passed"] = bool(
        sample_count == contract.EXPECTED_RAW_SENSOR_SAMPLES
        and all(
            result[field] == 0
            for field in (
                "duplicate_event_count",
                "out_of_order_event_count",
                "left_non_v26_source_count",
                "fallback_count",
                "hard_invalid_count",
            )
        )
    )
    return result


def _worker_start_payload(
    *, case: Mapping[str, Any], lock: Mapping[str, Any]
) -> dict[str, Any]:
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": "H0_V8_V26_TEACHER_REPLAY_STARTED",
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
        "baseline_trace": _record(resolve_relative(case["baseline_trace"])),
        "baseline_receipt": _record(resolve_relative(case["baseline_receipt"])),
        "source_h0_module_state": lock["inputs"]["source_h0_module_state"],
        "v25_profile": lock["inputs"]["v25_profile"],
        "v26_development_receipt": lock["inputs"]["v26_development_receipt"],
        "v26_v7_replay_receipt": lock["inputs"]["v26_v7_replay_receipt"],
        "authority": copy.deepcopy(contract.AUTHORITY),
        "candidate_created": False,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
    }


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


def _bind_v8_engine() -> None:
    """Replace every V6 module-global consumed by an inherited worker."""

    engine.contract = contract
    engine.LOCK = LOCK
    engine.RUN_ROOT = RUN_ROOT
    engine.EXECUTION_LEDGER = EXECUTION_LEDGER
    engine.EXECUTION_CLAIM = EXECUTION_CLAIM
    engine.WORKER_CLAIMS_ROOT = WORKER_CLAIMS_ROOT
    engine.INPUT_PATHS = INPUT_PATHS
    engine.SOURCE_PATHS = SOURCE_PATHS
    engine.SOURCE_H0_MODULE = SOURCE_H0_MODULE
    engine.SOURCE_H0_CONFIG = SOURCE_H0_CONFIG
    engine.V6TeacherReplayExecutionError = V8TeacherReplayExecutionError
    engine.verify_lock = verify_lock
    engine.canonical_destination = canonical_destination
    engine.worker_claim_path = worker_claim_path
    engine._token_sha256 = _token_sha256
    engine._execution_claim_payload = _execution_claim_payload
    engine.verify_execution_claim = verify_execution_claim
    engine._worker_claim_payload = _worker_claim_payload
    engine.verify_worker_claim = verify_worker_claim
    engine.build_env_config = build_env_config
    engine._worker_start_payload = _worker_start_payload
    engine._worker_command = _worker_command
    engine.legacy._accumulate_binary_events = _accumulate_binary_events_v26
    engine.legacy._finalize_binary_event_gate = _finalize_binary_event_gate_v26


_bind_v8_engine()

# Public pure helpers remain available under the fresh V8 runner module.
exact_float32_columns = engine.exact_float32_columns
exact_float32_vector = engine.exact_float32_vector
load_frozen_baseline = engine.load_frozen_baseline
verify_case_receipt = engine.verify_case_receipt
run_worker = engine.run_worker


def execute() -> dict[str, Any]:
    verify_lock(require_run_root_absent=True)
    started = time.time()
    execution_token = secrets.token_urlsafe(32)
    token_sha256 = _token_sha256(execution_token)
    forensic.write_json_exclusive(
        EXECUTION_CLAIM, _execution_claim_payload(token_sha256)
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
                raise V8TeacherReplayExecutionError(
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
        "reserve_trials_opened": [],
        "next_stage": (
            "BUILD_V8_RESIDUAL_DATASET"
            if passed
            else "STOP_WITHOUT_RETRY_OR_RETUNING"
        ),
    }
    forensic.write_json_exclusive(EXECUTION_LEDGER, ledger)
    print(json.dumps(ledger, indent=2, sort_keys=True, allow_nan=False), flush=True)
    if not passed:
        raise V8TeacherReplayExecutionError(error or status)
    return ledger


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--prepare", action="store_true")
    mode.add_argument("--execute", action="store_true")
    mode.add_argument("--worker", action="store_true")
    parser.add_argument("--case", choices=contract.CASE_IDS)
    parser.add_argument("--output-dir")
    parser.add_argument("--execution-token")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        if args.prepare:
            result = prepare()
        elif args.execute:
            result = execute()
        else:
            if (
                args.case is None
                or args.output_dir is None
                or args.execution_token is None
            ):
                raise V8TeacherReplayExecutionError(
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
            f"V8/V26 teacher replay failed closed: {type(exc).__name__}: {exc}",
            file=sys.stderr,
        )
        return 2
    print(json.dumps(result, indent=2, sort_keys=True, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
