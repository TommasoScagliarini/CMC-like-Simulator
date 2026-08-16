"""Build the non-simulation preflight for V6 V25 teacher-action replay."""

from __future__ import annotations

import json
import math
import os
import sys
from pathlib import Path, PurePosixPath
from typing import Any, Mapping, Sequence


REPO_ROOT = Path(__file__).resolve().parents[1]
VALIDATION_ROOT = REPO_ROOT / "validation"
if str(VALIDATION_ROOT) not in sys.path:
    sys.path.insert(0, str(VALIDATION_ROOT))

import h0_forensic_rollout as forensic  # noqa: E402
import h0_primary_grf_split_v6_teacher_replay_contract as contract  # noqa: E402


class V6TeacherReplayPreflightError(RuntimeError):
    """Raised when any frozen development prerequisite is invalid."""


def resolve_relative(relative: str | PurePosixPath) -> Path:
    value = PurePosixPath(relative)
    if value.is_absolute() or ".." in value.parts or not value.parts:
        raise V6TeacherReplayPreflightError(f"non-canonical repository path: {value}")
    return REPO_ROOT.joinpath(*value.parts)


PREFLIGHT_PATH = resolve_relative(contract.PREFLIGHT_RECEIPT_PATH)
LOCK_PATH = resolve_relative(contract.LOCK_PATH)


def source_record(path: str | Path) -> dict[str, Any]:
    unresolved = Path(path).expanduser()
    if unresolved.is_symlink():
        raise V6TeacherReplayPreflightError(
            f"required file must not be a symlink: {unresolved}"
        )
    source = unresolved.resolve()
    if not source.is_file():
        raise V6TeacherReplayPreflightError(
            f"required regular non-symlink file is missing: {source}"
        )
    return forensic.artifact_record(source, artifact_root=REPO_ROOT)


def _record_matches(record: Any, path: Path) -> bool:
    return (
        isinstance(record, Mapping)
        and set(record) == {"path", "sha256", "size_bytes"}
        and dict(record) == source_record(path)
    )


def _mapping(path: Path) -> dict[str, Any]:
    value = forensic.strict_json_load(path)
    if not isinstance(value, Mapping):
        raise V6TeacherReplayPreflightError(f"expected JSON object: {path}")
    return dict(value)


def _sequence(path: Path) -> list[Any]:
    value = forensic.strict_json_load(path)
    if isinstance(value, (str, bytes)) or not isinstance(value, Sequence):
        raise V6TeacherReplayPreflightError(f"expected JSON array: {path}")
    return list(value)


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
        for artifact in ("common_gate.json", "solver_audit_journal.json"):
            key = artifact.removesuffix(".json")
            result[f"v5_baseline_{case_id}_{key}"] = baseline_root / artifact
    return result


def _finite_vector(value: Any, length: int) -> bool:
    return (
        isinstance(value, list)
        and len(value) == length
        and all(
            not isinstance(item, bool)
            and isinstance(item, (int, float))
            and math.isfinite(float(item))
            for item in value
        )
    )


def validate_v5_terminal() -> dict[str, Any]:
    path = resolve_relative(
        contract.INPUT_RELATIVE_PATHS["v5_terminal_qualification_ledger"]
    )
    ledger = _mapping(path)
    qualification_lock_path = resolve_relative(
        contract.INPUT_RELATIVE_PATHS["v5_qualification_lock"]
    )
    qualification_lock = _mapping(qualification_lock_path)
    v5_execution_ledger_path = resolve_relative(
        contract.INPUT_RELATIVE_PATHS["v5_execution_ledger"]
    )
    v5_holdout_receipt_path = resolve_relative(
        contract.INPUT_RELATIVE_PATHS["v5_holdout_receipt"]
    )
    if (
        ledger.get("schema_version") != 5
        or ledger.get("status") != "FAIL_H0_PRIMARY_SPLIT_V5_AUTONOMOUS_QUALIFICATION"
        or ledger.get("passed") is not False
        or ledger.get("baseline_rollouts_completed") != 6
        or ledger.get("candidate_rollouts_completed") != 0
        or ledger.get("next_stage") != "STOP_WITHOUT_RETRY_OR_RETUNING"
        or ledger.get("actor_updates") != 0
        or ledger.get("critic_updates") != 0
        or ledger.get("ppo_updates") != 0
        or ledger.get("protected_trials_opened") != []
        or not _record_matches(
            ledger.get("qualification_lock"), qualification_lock_path
        )
        or not _record_matches(
            ledger.get("v5_execution_ledger"), v5_execution_ledger_path
        )
        or not _record_matches(
            ledger.get("v5_holdout_receipt"), v5_holdout_receipt_path
        )
    ):
        raise V6TeacherReplayPreflightError(
            "V5 terminal qualification lineage is not canonical"
        )
    if (
        qualification_lock.get("schema_version") != 5
        or qualification_lock.get("status")
        != "H0_PRIMARY_SPLIT_V5_QUALIFICATION_UNLOCKED"
        or qualification_lock.get("protocol_id")
        != "AB06_H0_PRIMARY_GRF_SPLIT_V5_AUTONOMOUS_QUALIFICATION"
        or qualification_lock.get("event_contract_id")
        != "primary_grf_split_v1+legacy_events_v1"
        or qualification_lock.get("source_protocol_id")
        != "AB06_H0_PRIMARY_GRF_SPLIT_V5_FULL_MEAN"
        or not isinstance(qualification_lock.get("inputs"), Mapping)
        or not _record_matches(
            qualification_lock["inputs"].get("source_h0_state"),
            resolve_relative(contract.INPUT_RELATIVE_PATHS["source_h0_module_state"]),
        )
        or not _record_matches(
            qualification_lock["inputs"].get("source_h0_config"),
            resolve_relative(contract.INPUT_RELATIVE_PATHS["source_h0_config"]),
        )
        or qualification_lock.get("actor_updates") != 0
        or qualification_lock.get("critic_updates") != 0
        or qualification_lock.get("ppo_updates") != 0
        or qualification_lock.get("protected_trials_opened") != []
    ):
        raise V6TeacherReplayPreflightError(
            "V5 qualification lock lineage is not canonical"
        )
    return {
        "status": ledger["status"],
        "baseline_rollouts_completed": 6,
        "candidate_rollouts_completed": 0,
        "artifact": source_record(path),
        "qualification_lock": source_record(qualification_lock_path),
        "v5_execution_ledger": source_record(v5_execution_ledger_path),
        "v5_holdout_receipt": source_record(v5_holdout_receipt_path),
    }


def validate_v25_active() -> dict[str, Any]:
    freeze_path = resolve_relative(
        contract.INPUT_RELATIVE_PATHS["v25_candidate_freeze"]
    )
    protocol_path = resolve_relative(
        contract.INPUT_RELATIVE_PATHS["v25_corrected_active_protocol"]
    )
    profile_path = resolve_relative(contract.INPUT_RELATIVE_PATHS["v25_profile"])
    freeze = _mapping(freeze_path)
    protocol = _mapping(protocol_path)
    candidate = freeze.get("candidate")
    contracts = protocol.get("contracts")
    if (
        freeze.get("status") != "V25_DEVELOPMENT_CANDIDATE_FROZEN_H0_PROTOCOL_REQUIRED"
        or freeze.get("pass") is not True
        or not isinstance(candidate, Mapping)
        or candidate.get("candidate_id") != "v25_4b351f67b5b86ab0"
        or not _record_matches(candidate.get("profile"), profile_path)
        or protocol.get("status")
        != "H0_V25_ABC_PROTOCOL_CORRECTED_FROZEN_EXECUTION_NOT_AUTHORIZED"
        or not isinstance(contracts, Mapping)
        or contracts.get("target_active_bundle_contract_id")
        != contract.TARGET_OBSERVATION_CONTRACT_ID
        or contracts.get("runtime_active_component_contract_id")
        != contract.V25_ACTIVE_EVENT_CONTRACT_ID
    ):
        raise V6TeacherReplayPreflightError(
            "V25 candidate/active bridge prerequisite drifted"
        )
    return {
        "candidate_id": candidate["candidate_id"],
        "profile": source_record(profile_path),
        "candidate_freeze": source_record(freeze_path),
        "active_protocol": source_record(protocol_path),
    }


def validate_source_h0_layout() -> dict[str, Any]:
    layout_path = resolve_relative(
        contract.INPUT_RELATIVE_PATHS["actor_layout_reference"]
    )
    layout = _mapping(layout_path)
    if (
        layout.get("n_actor") != contract.EXPECTED_ACTOR_FEATURES
        or layout.get("n_observation") != contract.EXPECTED_FULL_FEATURES
        or layout.get("observation_dtype") != contract.EXPECTED_OBSERVATION_DTYPE
        or layout.get("actor_feature_names")
        != list(contract.EXPECTED_ACTOR_FEATURE_NAMES)
        or layout.get("observation_feature_names")
        != list(contract.EXPECTED_OBSERVATION_FEATURE_NAMES)
        or layout.get("protected_trials_opened") != []
    ):
        raise V6TeacherReplayPreflightError("source H0 layout reference drifted")
    return {
        "source_h0_id": contract.SOURCE_H0_ID,
        "actor_feature_names": list(contract.EXPECTED_ACTOR_FEATURE_NAMES),
        "observation_feature_names": list(contract.EXPECTED_OBSERVATION_FEATURE_NAMES),
        "layout_reference": source_record(layout_path),
        "module_state": source_record(
            resolve_relative(contract.INPUT_RELATIVE_PATHS["source_h0_module_state"])
        ),
    }


def validate_baseline_case(case_id: str) -> dict[str, Any]:
    case = contract.canonical_case(case_id)
    trace_path = resolve_relative(case["baseline_trace"])
    summary_path = resolve_relative(case["baseline_summary"])
    receipt_path = resolve_relative(case["baseline_receipt"])
    trace = _sequence(trace_path)
    summary = _mapping(summary_path)
    receipt = _mapping(receipt_path)
    artifacts = receipt.get("artifacts")
    qualification_lock_path = resolve_relative(
        contract.INPUT_RELATIVE_PATHS["v5_qualification_lock"]
    )
    common_gate_path = receipt_path.parent / "common_gate.json"
    solver_audit_path = receipt_path.parent / "solver_audit_journal.json"
    common_gate = _mapping(common_gate_path)
    solver_audit = _sequence(solver_audit_path)
    if (
        set(receipt)
        != {
            "actor_updates",
            "artifacts",
            "case_id",
            "critic_updates",
            "passed",
            "ppo_updates",
            "protected_trials_opened",
            "qualification_lock",
            "role",
            "schema_version",
            "status",
        }
        or receipt.get("schema_version") != 5
        or receipt.get("status") != "PASS_H0_PRIMARY_SPLIT_V5_QUALIFICATION_ROLLOUT"
        or receipt.get("passed") is not True
        or receipt.get("role") != "baseline"
        or receipt.get("case_id") != case_id
        or receipt.get("actor_updates") != 0
        or receipt.get("critic_updates") != 0
        or receipt.get("ppo_updates") != 0
        or receipt.get("protected_trials_opened") != []
        or not _record_matches(
            receipt.get("qualification_lock"), qualification_lock_path
        )
        or not isinstance(artifacts, Mapping)
        or set(artifacts)
        != {
            "trace.json",
            "summary.json",
            "common_gate.json",
            "solver_audit_journal.json",
        }
        or not _record_matches(artifacts.get("trace.json"), trace_path)
        or not _record_matches(artifacts.get("summary.json"), summary_path)
        or not _record_matches(artifacts.get("common_gate.json"), common_gate_path)
        or not _record_matches(
            artifacts.get("solver_audit_journal.json"), solver_audit_path
        )
    ):
        raise V6TeacherReplayPreflightError(f"V5 baseline receipt drifted: {case_id}")
    common_checks = common_gate.get("checks")
    if (
        set(common_gate)
        != {"case_id", "checks", "passed", "role", "schema_version", "status"}
        or common_gate.get("schema_version") != 5
        or common_gate.get("status") != "PASS_H0_PRIMARY_SPLIT_V5_QUALIFICATION_ROLLOUT"
        or common_gate.get("passed") is not True
        or common_gate.get("role") != "baseline"
        or common_gate.get("case_id") != case_id
        or not isinstance(common_checks, list)
        or len(common_checks) != 49
        or not all(
            isinstance(check, Mapping) and check.get("status") == "PASS"
            for check in common_checks
        )
    ):
        raise V6TeacherReplayPreflightError(
            f"V5 baseline common gate drifted: {case_id}"
        )
    if (
        summary.get("schema_version") != 5
        or summary.get("role") != "baseline"
        or summary.get("case_id") != case_id
        or summary.get("actor_input_view") != "historical_analog"
        or summary.get("event_contract_id") != "primary_grf_split_v1+legacy_events_v1"
        or summary.get("binary_phase_fsm_mode") != "disabled"
        or summary.get("phase_fsm_input_mode") != "legacy_events"
        or summary.get("online_grf_applied_sides") != ["left"]
        or summary.get("morphology_weight") != 0.0
        or summary.get("so_policy_id") != contract.SO_POLICY_ID
        or summary.get("action_selection") != case["action_selection"]
        or summary.get("episode_start_offset_s") != case["episode_start_offset_s"]
        or summary.get("action_seed") != case["action_seed"]
        or summary.get("runtime_seed") != case["runtime_seed"]
        or summary.get("sigma") != case["sigma"]
        or summary.get("steps") != contract.EXPECTED_STEPS
        or summary.get("end_reason") != "episode_time_limit"
        or summary.get("terminated") is not False
        or summary.get("truncated") is not True
        or summary.get("n_actor") != contract.EXPECTED_ACTOR_FEATURES
        or summary.get("n_observation") != contract.EXPECTED_FULL_FEATURES
        or summary.get("observation_dtype") != contract.EXPECTED_OBSERVATION_DTYPE
        or summary.get("actor_updates") != 0
        or summary.get("critic_updates") != 0
        or summary.get("ppo_updates") != 0
        or summary.get("protected_trials_opened") != []
    ):
        raise V6TeacherReplayPreflightError(f"V5 baseline summary drifted: {case_id}")
    expected_keys = {
        "step",
        "time_s",
        "actor_input_view",
        "actor_observation",
        "mean_action",
        "standard_normal",
        "raw_action",
        "reward",
        "terminated",
        "truncated",
    }
    if len(trace) != contract.EXPECTED_STEPS:
        raise V6TeacherReplayPreflightError(
            f"V5 baseline trace length drifted: {case_id}"
        )
    previous_time = -math.inf
    for index, raw_row in enumerate(trace, start=1):
        if not isinstance(raw_row, Mapping):
            raise V6TeacherReplayPreflightError(
                f"V5 baseline row is not an object: {case_id}/{index}"
            )
        row = dict(raw_row)
        time_s = row.get("time_s")
        if (
            set(row) != expected_keys
            or row.get("step") != index
            or row.get("actor_input_view") != "historical_analog"
            or not _finite_vector(row.get("actor_observation"), 35)
            or not _finite_vector(row.get("mean_action"), 2)
            or not _finite_vector(row.get("standard_normal"), 2)
            or not _finite_vector(row.get("raw_action"), 2)
            or not isinstance(time_s, (int, float))
            or isinstance(time_s, bool)
            or not math.isfinite(float(time_s))
            or float(time_s) <= previous_time
            or (
                index > 1
                and abs(float(time_s) - previous_time - contract.EXPECTED_POLICY_DT_S)
                > 1.0e-12
            )
            or not isinstance(row.get("reward"), (int, float))
            or isinstance(row.get("reward"), bool)
            or not math.isfinite(float(row["reward"]))
            or type(row.get("terminated")) is not bool
            or type(row.get("truncated")) is not bool
            or (index < contract.EXPECTED_STEPS and row.get("terminated") is not False)
            or (index < contract.EXPECTED_STEPS and row.get("truncated") is not False)
        ):
            raise V6TeacherReplayPreflightError(
                f"V5 baseline row schema drifted: {case_id}/{index}"
            )
        previous_time = float(time_s)
    if trace[-1]["terminated"] is not False or trace[-1]["truncated"] is not True:
        raise V6TeacherReplayPreflightError(
            f"V5 baseline terminal row drifted: {case_id}"
        )
    if len(solver_audit) != contract.EXPECTED_STEPS:
        raise V6TeacherReplayPreflightError(
            f"V5 solver audit length drifted: {case_id}"
        )
    for index, raw_entry in enumerate(solver_audit, start=1):
        if not isinstance(raw_entry, Mapping):
            raise V6TeacherReplayPreflightError(
                f"V5 solver audit entry malformed: {case_id}/{index}"
            )
        entry = dict(raw_entry)
        windows = entry.get("control_windows")
        if (
            set(entry) != {"step", "time_s", "control_windows"}
            or entry.get("step") != index
            or entry.get("time_s") != trace[index - 1]["time_s"]
            or not isinstance(windows, list)
            or len(windows) != contract.EXPECTED_SAMPLES_PER_STEP
            or not all(isinstance(window, Mapping) for window in windows)
        ):
            raise V6TeacherReplayPreflightError(
                f"V5 solver audit entry drifted: {case_id}/{index}"
            )
    return {
        "case": case,
        "rows": len(trace),
        "trace": source_record(trace_path),
        "summary": source_record(summary_path),
        "receipt": source_record(receipt_path),
        "common_gate": source_record(common_gate_path),
        "solver_audit_journal": source_record(solver_audit_path),
        "first_time_s": float(trace[0]["time_s"]),
        "last_time_s": float(trace[-1]["time_s"]),
        "reclassification": "V5_PASS_BASELINE_TO_V6_DEVELOPMENT_ONLY",
    }


def build_payload(*, require_destinations_absent: bool = True) -> dict[str, Any]:
    sources = source_paths()
    inputs = input_paths()
    source_records = {name: source_record(path) for name, path in sources.items()}
    input_records = {name: source_record(path) for name, path in inputs.items()}
    v5_terminal = validate_v5_terminal()
    v25 = validate_v25_active()
    source_h0 = validate_source_h0_layout()
    cases = [validate_baseline_case(case_id) for case_id in contract.CASE_IDS]
    destinations = [resolve_relative(case["destination"]) for case in contract.CASES]
    destinations_unoccupied = not any(os.path.lexists(path) for path in destinations)
    run_root_unoccupied = not os.path.lexists(resolve_relative(contract.RUN_ROOT))
    checks = {
        "all_sources_present_and_hashed": len(source_records)
        == len(contract.SOURCE_RELATIVE_PATHS),
        "all_inputs_present_and_hashed": len(input_records) == len(inputs),
        "v5_terminal_fail_cited": v5_terminal["status"]
        == "FAIL_H0_PRIMARY_SPLIT_V5_AUTONOMOUS_QUALIFICATION",
        "six_v5_pass_baselines_reclassified_development": len(cases) == 6
        and all(item["rows"] == contract.EXPECTED_STEPS for item in cases),
        "source_h0_original": source_h0["source_h0_id"] == contract.SOURCE_H0_ID,
        "v25_active_candidate_frozen": v25["candidate_id"] == "v25_4b351f67b5b86ab0",
        "target_contract_exact": contract.TARGET_OBSERVATION_CONTRACT_ID
        == "primary_grf_split_v1+binary_point_v25+functional_contact_fsm_v1",
        "invariant_columns_exact": list(contract.INVARIANT_COLUMNS)
        == [*range(2, 10), *range(25, 35)],
        "development_replay_authorized": contract.AUTHORITY[
            "development_teacher_action_replay_authorized"
        ],
        "updates_forbidden": not any(
            contract.AUTHORITY[key]
            for key in (
                "actor_updates_authorized",
                "critic_updates_authorized",
                "ppo_updates_authorized",
            )
        ),
        "protected_closed": not contract.AUTHORITY["protected_trial_access_authorized"],
        "destinations_unoccupied": destinations_unoccupied,
        "run_root_unoccupied": run_root_unoccupied,
        "execution_lock_unoccupied": not os.path.lexists(LOCK_PATH),
    }
    if require_destinations_absent and not all(checks.values()):
        failed = [name for name, passed in checks.items() if not passed]
        raise V6TeacherReplayPreflightError(
            f"V6 teacher replay preflight failed: {failed}"
        )
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.PREFLIGHT_STATUS,
        "passed": all(checks.values()),
        "protocol_id": contract.PROTOCOL_ID,
        "collector_id": contract.COLLECTOR_ID,
        "revision": contract.REVISION,
        "checks": checks,
        "v5_terminal": v5_terminal,
        "v25": v25,
        "source_h0": source_h0,
        "cases": cases,
        "sources": source_records,
        "inputs": input_records,
        "destinations": [
            path.relative_to(REPO_ROOT).as_posix() for path in destinations
        ],
        "authority": dict(contract.AUTHORITY),
        "simulations_executed": 0,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "candidate_created": False,
        "next_stage": "FREEZE_V6_V25_TEACHER_REPLAY_EXECUTION",
    }


def publish() -> dict[str, Any]:
    payload = build_payload(require_destinations_absent=True)
    forensic.write_json_exclusive(PREFLIGHT_PATH, payload)
    return payload


def main() -> int:
    try:
        result = publish()
    except Exception as exc:
        print(
            f"V6 teacher replay preflight failed: {type(exc).__name__}: {exc}",
            file=sys.stderr,
        )
        return 2
    print(json.dumps(result, indent=2, sort_keys=True, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
