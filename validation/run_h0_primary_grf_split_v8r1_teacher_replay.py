"""Fresh V8R1 procedural correction for the V26 teacher replay.

V8R1 overlays the frozen V8 collector without changing its scientific gate.
The sole runtime correction is the explicit inherited-engine attribute
``V25_ACTIVE_EVENT_CONTRACT_ID == V26_ACTIVE_EVENT_CONTRACT_ID``.  All
receipts, claims, worker directories, and ledgers use fresh no-clobber paths.
"""

from __future__ import annotations

import argparse
import copy
import json
import os
import sys
from pathlib import Path
from typing import Any, Mapping, Sequence


REPO_ROOT = Path(__file__).resolve().parents[1]
VALIDATION_ROOT = REPO_ROOT / "validation"
for import_root in (REPO_ROOT, VALIDATION_ROOT):
    if str(import_root) not in sys.path:
        sys.path.insert(0, str(import_root))

from validation import h0_forensic_rollout as forensic  # noqa: E402
from validation import h0_primary_grf_split_v8r1_teacher_replay_contract as contract  # noqa: E402
from validation import run_h0_primary_grf_split_v8_teacher_replay as base  # noqa: E402


class V8R1TeacherReplayExecutionError(RuntimeError):
    """Raised on any V8R1 compatibility, provenance, or execution failure."""


# Every contract attribute dereferenced by the inherited V6 numerical engine.
# This list was produced by an AST/name audit and is itself frozen in the
# V8R1 source closure.
INHERITED_ENGINE_CONTRACT_ATTRIBUTES = (
    "AUTHORITY",
    "CASES",
    "CASE_IDS",
    "COLLECTOR_ID",
    "EXECUTION_CLAIM_PATH",
    "EXECUTION_LEDGER_PATH",
    "EXPECTED_ACTION_DIM",
    "EXPECTED_ACTOR_FEATURES",
    "EXPECTED_ACTOR_FEATURE_NAMES",
    "EXPECTED_CONTROL_WINDOWS",
    "EXPECTED_EPISODE_DURATION_S",
    "EXPECTED_FULL_FEATURES",
    "EXPECTED_OBSERVATION_DTYPE",
    "EXPECTED_OBSERVATION_FEATURE_NAMES",
    "EXPECTED_POLICY_DT_S",
    "EXPECTED_RAW_SENSOR_SAMPLES",
    "EXPECTED_SAMPLES_PER_STEP",
    "EXPECTED_SAMPLE_DT_S",
    "EXPECTED_STEPS",
    "INVARIANT_COLUMNS",
    "INVARIANT_COLUMN_RANGES",
    "LOCK_PATH",
    "LOCK_STATUS",
    "MINIMUM_VALID_CYCLES",
    "MORPHOLOGY_WEIGHT",
    "PENETRATION_LIMIT_M",
    "PREFLIGHT_RECEIPT_PATH",
    "PREFLIGHT_STATUS",
    "PROTOCOL_FAIL_STATUS",
    "PROTOCOL_ID",
    "PROTOCOL_PASS_STATUS",
    "REVISION",
    "ROLLOUT_COLLECTED_STATUS",
    "ROLLOUT_FAIL_STATUS",
    "ROLLOUT_PASS_STATUS",
    "RUN_ROOT",
    "SCHEMA_VERSION",
    "SOURCE_H0_ID",
    "SOURCE_OBSERVATION_CONTRACT_ID",
    "SO_POLICY_ID",
    "STOCHASTIC_SIGMA",
    "TARGET_OBSERVATION_CONTRACT_ID",
    "V25_ACTIVE_EVENT_CONTRACT_ID",
    "WORKER_CLAIMS_ROOT",
)


def _install_fresh_paths_and_contract() -> None:
    base.contract = contract
    base.LOCK = base.resolve_relative(contract.LOCK_PATH)
    base.PREFLIGHT = base.resolve_relative(contract.PREFLIGHT_RECEIPT_PATH)
    base.RUN_ROOT = base.resolve_relative(contract.RUN_ROOT)
    base.EXECUTION_LEDGER = base.resolve_relative(contract.EXECUTION_LEDGER_PATH)
    base.EXECUTION_CLAIM = base.resolve_relative(contract.EXECUTION_CLAIM_PATH)
    base.WORKER_CLAIMS_ROOT = base.resolve_relative(contract.WORKER_CLAIMS_ROOT)
    base.SOURCE_PATHS = base.source_paths()
    base.INPUT_PATHS = base.input_paths()
    base.SOURCE_H0_MODULE = base.INPUT_PATHS["source_h0_module_state"].parent
    base.SOURCE_H0_CONFIG = base.INPUT_PATHS["source_h0_config"]


_install_fresh_paths_and_contract()

LOCK = base.LOCK
PREFLIGHT = base.PREFLIGHT
RUN_ROOT = base.RUN_ROOT
EXECUTION_LEDGER = base.EXECUTION_LEDGER
EXECUTION_CLAIM = base.EXECUTION_CLAIM
WORKER_CLAIMS_ROOT = base.WORKER_CLAIMS_ROOT
SOURCE_PATHS = base.SOURCE_PATHS
INPUT_PATHS = base.INPUT_PATHS
SOURCE_H0_MODULE = base.SOURCE_H0_MODULE
SOURCE_H0_CONFIG = base.SOURCE_H0_CONFIG
COMPATIBILITY_PREFLIGHT = base.resolve_relative(
    contract.COMPATIBILITY_PREFLIGHT_PATH
)

_BASE_BUILD_PREFLIGHT_PAYLOAD = base.build_preflight_payload
_BASE_LOCK_PAYLOAD = base._lock_payload
_BASE_VERIFY_LOCK = base.verify_lock


def audit_inherited_engine_contract() -> dict[str, Any]:
    missing = [
        name
        for name in INHERITED_ENGINE_CONTRACT_ATTRIBUTES
        if not hasattr(contract, name)
    ]
    checks = {
        "all_inherited_attributes_present": missing == [],
        "compatibility_alias_is_v26_exact": (
            getattr(contract, "V25_ACTIVE_EVENT_CONTRACT_ID", None)
            == contract.V26_ACTIVE_EVENT_CONTRACT_ID
        ),
        "compatibility_alias_is_not_v20": (
            getattr(contract, "V25_ACTIVE_EVENT_CONTRACT_ID", None)
            != "binary_point_v25+functional_contact_fsm_v1"
        ),
        "target_contract_unchanged_from_v8": (
            contract.TARGET_OBSERVATION_CONTRACT_ID
            == base.v6_preflight.contract.TARGET_OBSERVATION_CONTRACT_ID.replace(
                "functional_contact_fsm_v1", "heel_qualified_fsm_v2"
            )
        ),
    }
    return {
        "passed": all(checks.values()),
        "checks": checks,
        "required_attributes": list(INHERITED_ENGINE_CONTRACT_ATTRIBUTES),
        "missing_attributes": missing,
        "compatibility_alias": {
            "name": "V25_ACTIVE_EVENT_CONTRACT_ID",
            "value": getattr(contract, "V25_ACTIVE_EVENT_CONTRACT_ID", None),
            "semantic_target": "V26_ACTIVE_EVENT_CONTRACT_ID",
        },
    }


def validate_terminal_v8_history() -> dict[str, Any]:
    ledger_path = INPUT_PATHS["v8_terminal_execution_ledger"]
    failure_path = INPUT_PATHS["v8_terminal_worker_failure"]
    ledger = base._mapping(ledger_path)
    failure = base._mapping(failure_path)
    error = failure.get("error")
    if (
        ledger.get("schema_version") != 8
        or ledger.get("status") != "FAIL_H0_V8_V26_TEACHER_REPLAY_DEVELOPMENT"
        or ledger.get("passed") is not False
        or ledger.get("protocol_id")
        != "AB06_H0_PRIMARY_SPLIT_V8_V26_RESIDUAL_DAGGER"
        or ledger.get("completed_cases") != []
        or ledger.get("completed_receipts") != []
        or ledger.get("actor_updates") != 0
        or ledger.get("critic_updates") != 0
        or ledger.get("ppo_updates") != 0
        or ledger.get("protected_trials_opened") != []
        or ledger.get("reserve_trials_opened") != []
        or ledger.get("next_stage") != "STOP_WITHOUT_RETRY_OR_RETUNING"
        or failure.get("status") != "FAIL_H0_V8_V26_TEACHER_REPLAY"
        or failure.get("last_completed_step") != 0
        or not isinstance(error, Mapping)
        or error.get("type") != "AttributeError"
        or error.get("message")
        != (
            "module 'validation.h0_primary_grf_split_v8_teacher_replay_contract' "
            "has no attribute 'V25_ACTIVE_EVENT_CONTRACT_ID'"
        )
    ):
        raise V8R1TeacherReplayExecutionError("terminal V8 history drifted")
    return {
        "status": ledger["status"],
        "completed_policy_steps": 0,
        "failure_class": "PROCEDURAL_COMPATIBILITY_BINDING",
        "failure_message": error["message"],
        "execution_ledger": base._record(ledger_path),
        "worker_failure": base._record(failure_path),
        "history_policy": "TERMINAL_NO_REINTERPRETATION",
    }


def build_env_config(case: Mapping[str, Any]) -> dict[str, Any]:
    return base.build_env_config(case)


def _compatibility_payload() -> dict[str, Any]:
    audit = audit_inherited_engine_contract()
    terminal = validate_terminal_v8_history()
    case = contract.canonical_case(contract.CASE_IDS[0])
    env_config = build_env_config(case)
    reset_checks: dict[str, bool] = {}
    reset_observation: dict[str, Any] = {}
    reset_info_summary: dict[str, Any] = {}
    env = None
    error: str | None = None
    try:
        _rollout_eval, np, torch, _rl_module, env_factory, _reward = (
            base.engine.legacy._load_inference_stack()
        )
        runtime_seed = int(case["runtime_seed"])
        np.random.seed(runtime_seed)
        torch.manual_seed(runtime_seed)
        env = env_factory.make_cmc_env(env_config)
        observation, info = env.reset(seed=runtime_seed)
        vector = np.asarray(observation)
        binary_payload = info.get("binary_phase_fsm")
        adapter_payload = info.get("binary_phase_active_adapter")
        baseline = info.get("binary_phase_sensor_baseline")
        reset_checks = {
            "observation_shape_84": tuple(vector.shape)
            == (contract.EXPECTED_FULL_FEATURES,),
            "observation_dtype_float32": str(vector.dtype)
            == contract.EXPECTED_OBSERVATION_DTYPE,
            "observation_finite": bool(np.all(np.isfinite(vector))),
            "binary_fsm_executed": info.get("binary_phase_fsm_executed") is True,
            "binary_mode_active": info.get("binary_phase_fsm_mode")
            == "binary_active",
            "event_contract_v26_exact": info.get(
                "binary_phase_event_contract_id"
            )
            == contract.V26_ACTIVE_EVENT_CONTRACT_ID,
            "binary_payload_v26_source": isinstance(binary_payload, Mapping)
            and binary_payload.get("source") == contract.V26_FSM_SOURCE,
            "binary_payload_v26_contract": isinstance(binary_payload, Mapping)
            and binary_payload.get("event_contract_id")
            == contract.V26_ACTIVE_EVENT_CONTRACT_ID,
            "no_reset_event": isinstance(binary_payload, Mapping)
            and binary_payload.get("events_this_step") == [],
            "adapter_v26_source": isinstance(adapter_payload, Mapping)
            and adapter_payload.get("adapter_source")
            == contract.V26_ACTOR_ADAPTER_SOURCE,
            "adapter_zero_events": isinstance(adapter_payload, Mapping)
            and adapter_payload.get("events_adapted_this_step") == 0,
            "baseline_present": isinstance(baseline, Mapping),
            "primary_grf_left_only": info.get("online_grf_applied_sides")
            == ["left"],
            "morphology_zero": env_config.get("reward", {}).get(
                "morphology_weight"
            )
            == 0.0,
        }
        reset_observation = {
            "shape": list(vector.shape),
            "dtype": str(vector.dtype),
        }
        reset_info_summary = {
            "binary_phase_fsm_executed": info.get("binary_phase_fsm_executed"),
            "binary_phase_fsm_mode": info.get("binary_phase_fsm_mode"),
            "binary_phase_event_contract_id": info.get(
                "binary_phase_event_contract_id"
            ),
            "binary_phase_fsm_source": (
                binary_payload.get("source")
                if isinstance(binary_payload, Mapping)
                else None
            ),
            "binary_phase_adapter_source": (
                adapter_payload.get("adapter_source")
                if isinstance(adapter_payload, Mapping)
                else None
            ),
            "reset_event_count": (
                len(binary_payload.get("events_this_step", []))
                if isinstance(binary_payload, Mapping)
                else None
            ),
        }
    except Exception as exc:  # persisted below as a terminal compatibility FAIL.
        error = f"{type(exc).__name__}: {exc}"
    finally:
        if env is not None:
            env.close()
    checks = {
        "inherited_contract_audit": audit["passed"] is True,
        "v8_terminal_history_preserved": terminal["completed_policy_steps"] == 0,
        "build_env_config_v26": env_config.get("binary_phase_event_contract_id")
        == contract.V26_ACTIVE_EVENT_CONTRACT_ID,
        "build_env_config_binary_active": env_config.get("binary_phase_fsm_mode")
        == "binary_active",
        "reset_completed_without_error": error is None,
        "reset_routing_exact": bool(reset_checks) and all(reset_checks.values()),
        "zero_policy_steps": True,
        "protected_closed": True,
        "reserve_closed": True,
    }
    passed = all(checks.values())
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": (
            contract.COMPATIBILITY_PREFLIGHT_STATUS
            if passed
            else contract.COMPATIBILITY_PREFLIGHT_FAIL_STATUS
        ),
        "passed": passed,
        "protocol_id": contract.PROTOCOL_ID,
        "checks": checks,
        "inherited_engine_contract_audit": audit,
        "terminal_v8": terminal,
        "environment_config": {
            "binary_phase_fsm_mode": env_config.get("binary_phase_fsm_mode"),
            "binary_phase_event_contract_id": env_config.get(
                "binary_phase_event_contract_id"
            ),
            "phase_fsm_input_mode": env_config.get("phase_fsm_input_mode"),
            "online_grf_applied_sides": env_config.get("online_grf_applied_sides"),
            "morphology_weight": env_config.get("reward", {}).get(
                "morphology_weight"
            ),
        },
        "reset_checks": reset_checks,
        "reset_observation": reset_observation,
        "reset_info": reset_info_summary,
        "error": error,
        "environment_reset_calls": 1 if error is None or reset_checks else 0,
        "environment_step_calls": 0,
        "simulations_executed": 0,
        "candidate_created": False,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
        "next_stage": (
            "PREPARE_V8R1_TEACHER_REPLAY"
            if passed
            else "STOP_WITHOUT_RETRY_OR_RETUNING"
        ),
    }


def publish_compatibility_preflight() -> dict[str, Any]:
    if os.path.lexists(COMPATIBILITY_PREFLIGHT):
        raise V8R1TeacherReplayExecutionError(
            f"compatibility receipt exists: {COMPATIBILITY_PREFLIGHT}"
        )
    if os.path.lexists(PREFLIGHT) or os.path.lexists(LOCK) or os.path.lexists(RUN_ROOT):
        raise V8R1TeacherReplayExecutionError(
            "V8R1 closure/output already occupied before compatibility preflight"
        )
    payload = _compatibility_payload()
    forensic.write_json_exclusive(COMPATIBILITY_PREFLIGHT, payload)
    if payload.get("passed") is not True:
        raise V8R1TeacherReplayExecutionError(
            f"V8R1 compatibility preflight failed: {payload.get('error')}"
        )
    return payload


def _validated_compatibility_receipt() -> dict[str, Any]:
    observed = base._mapping(COMPATIBILITY_PREFLIGHT)
    checks = observed.get("checks")
    audit = observed.get("inherited_engine_contract_audit")
    if (
        observed.get("schema_version") != contract.SCHEMA_VERSION
        or observed.get("status") != contract.COMPATIBILITY_PREFLIGHT_STATUS
        or observed.get("passed") is not True
        or not isinstance(checks, Mapping)
        or not all(value is True for value in checks.values())
        or not isinstance(audit, Mapping)
        or audit.get("passed") is not True
        or observed.get("environment_step_calls") != 0
        or observed.get("simulations_executed") != 0
        or observed.get("actor_updates") != 0
        or observed.get("critic_updates") != 0
        or observed.get("ppo_updates") != 0
        or observed.get("protected_trials_opened") != []
        or observed.get("reserve_trials_opened") != []
    ):
        raise V8R1TeacherReplayExecutionError(
            "V8R1 compatibility receipt is not a canonical PASS"
        )
    return observed


def build_preflight_payload(*, require_unoccupied: bool) -> dict[str, Any]:
    compatibility = _validated_compatibility_receipt()
    terminal = validate_terminal_v8_history()
    payload = _BASE_BUILD_PREFLIGHT_PAYLOAD(
        require_unoccupied=require_unoccupied
    )
    payload["checks"]["v8_terminal_failure_preserved"] = (
        terminal["completed_policy_steps"] == 0
    )
    payload["checks"]["v8r1_reset_compatibility_pass"] = (
        compatibility["passed"] is True
    )
    payload["passed"] = all(payload["checks"].values())
    payload["terminal_v8"] = terminal
    payload["compatibility_preflight"] = base._record(COMPATIBILITY_PREFLIGHT)
    payload["procedural_correction"] = {
        "changed": ["compatibility_attribute_name", "artifact_paths"],
        "unchanged": [
            "V26_contract_value",
            "V25_geometry",
            "timing",
            "action_tapes",
            "scientific_gate",
            "trial_allocation",
        ],
    }
    payload["next_stage"] = "FREEZE_V8R1_V26_TEACHER_REPLAY_EXECUTION"
    return payload


def _lock_payload(preflight_payload: Mapping[str, Any]) -> dict[str, Any]:
    payload = _BASE_LOCK_PAYLOAD(preflight_payload)
    payload["terminal_v8"] = copy.deepcopy(preflight_payload["terminal_v8"])
    payload["compatibility_preflight"] = copy.deepcopy(
        preflight_payload["compatibility_preflight"]
    )
    payload["procedural_correction"] = copy.deepcopy(
        preflight_payload["procedural_correction"]
    )
    payload["next_stage"] = "EXECUTE_SIX_V8R1_V26_DEVELOPMENT_REPLAYS_ONCE"
    return payload


def prepare() -> dict[str, Any]:
    payload = build_preflight_payload(require_unoccupied=True)
    forensic.write_json_exclusive(PREFLIGHT, payload)
    lock = _lock_payload(payload)
    forensic.write_json_exclusive(LOCK, lock)
    return {"preflight": payload, "lock": lock}


def verify_lock(*, require_run_root_absent: bool = False) -> dict[str, Any]:
    lock = _BASE_VERIFY_LOCK(require_run_root_absent=require_run_root_absent)
    receipt = base._mapping(PREFLIGHT)
    compatibility = _validated_compatibility_receipt()
    if (
        lock.get("next_stage")
        != "EXECUTE_SIX_V8R1_V26_DEVELOPMENT_REPLAYS_ONCE"
        or lock.get("terminal_v8") != receipt.get("terminal_v8")
        or lock.get("procedural_correction") != receipt.get("procedural_correction")
        or lock.get("compatibility_preflight")
        != receipt.get("compatibility_preflight")
        or lock.get("compatibility_preflight")
        != base._record(COMPATIBILITY_PREFLIGHT)
        or compatibility.get("passed") is not True
        or audit_inherited_engine_contract()["passed"] is not True
    ):
        raise V8R1TeacherReplayExecutionError("V8R1 correction lock drifted")
    return lock


def canonical_destination(case_id: str) -> Path:
    return base.resolve_relative(contract.canonical_case(case_id)["destination"])


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


# Rebind both overlay and inherited numerical engine only after every fresh
# verification function exists.  Subprocesses enter this file and repeat the
# same binding before a worker claim is accepted.
base.build_preflight_payload = build_preflight_payload
base._lock_payload = _lock_payload
base.verify_lock = verify_lock
base.canonical_destination = canonical_destination
base._worker_command = _worker_command
base._bind_v8_engine()
base.engine.verify_lock = verify_lock
base.engine.canonical_destination = canonical_destination
base.engine._worker_command = _worker_command

engine = base.engine
verify_case_receipt = base.verify_case_receipt
exact_float32_columns = base.exact_float32_columns
exact_float32_vector = base.exact_float32_vector
load_frozen_baseline = base.load_frozen_baseline
run_worker = base.run_worker


def execute() -> dict[str, Any]:
    return base.execute()


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--compatibility-preflight", action="store_true")
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
        if args.compatibility_preflight:
            result = publish_compatibility_preflight()
        elif args.prepare:
            result = prepare()
        elif args.execute:
            result = execute()
        else:
            if (
                args.case is None
                or args.output_dir is None
                or args.execution_token is None
            ):
                raise V8R1TeacherReplayExecutionError(
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
            f"V8R1/V26 teacher replay failed closed: {type(exc).__name__}: {exc}",
            file=sys.stderr,
        )
        return 2
    print(json.dumps(result, indent=2, sort_keys=True, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
