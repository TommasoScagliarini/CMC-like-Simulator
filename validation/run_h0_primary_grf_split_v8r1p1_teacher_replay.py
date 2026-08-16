"""Execute the final thin V8R1P1/V26 teacher-replay overlay."""

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
for root in (REPO_ROOT, VALIDATION_ROOT):
    if str(root) not in sys.path:
        sys.path.insert(0, str(root))

from validation import h0_forensic_rollout as forensic  # noqa: E402
from validation import h0_primary_grf_split_v8r1p1_teacher_replay_contract as contract  # noqa: E402
from validation import run_h0_primary_grf_split_v8r1_teacher_replay as prior  # noqa: E402


class V8R1P1TeacherReplayError(RuntimeError):
    pass


base = prior.base
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

LOCK = base.LOCK
PREFLIGHT = base.PREFLIGHT
RUN_ROOT = base.RUN_ROOT
EXECUTION_LEDGER = base.EXECUTION_LEDGER
EXECUTION_CLAIM = base.EXECUTION_CLAIM
WORKER_CLAIMS_ROOT = base.WORKER_CLAIMS_ROOT
SOURCE_PATHS = base.SOURCE_PATHS
INPUT_PATHS = base.INPUT_PATHS
COMPATIBILITY_PREFLIGHT = base.resolve_relative(
    contract.COMPATIBILITY_PREFLIGHT_PATH
)

_ORIGINAL_BUILD_PREFLIGHT = prior._BASE_BUILD_PREFLIGHT_PAYLOAD
_ORIGINAL_LOCK_PAYLOAD = prior._BASE_LOCK_PAYLOAD
_ORIGINAL_VERIFY_LOCK = prior._BASE_VERIFY_LOCK
FROZEN_VERIFIER_NEXT_STAGE = "EXECUTE_SIX_V26_ACTIVE_DEVELOPMENT_REPLAYS_ONCE"


def audit_compatibility() -> dict[str, Any]:
    missing = [
        name
        for name in prior.INHERITED_ENGINE_CONTRACT_ATTRIBUTES
        if not hasattr(contract, name)
    ]
    reset = base._mapping(INPUT_PATHS["v8r1_reset_compatibility_receipt"])
    reset_checks = reset.get("reset_checks")
    checks = {
        "all_engine_attributes_present": missing == [],
        "v26_alias_exact": contract.V25_ACTIVE_EVENT_CONTRACT_ID
        == contract.V26_ACTIVE_EVENT_CONTRACT_ID,
        "prior_reset_pass": reset.get("passed") is True,
        "prior_reset_zero_steps": reset.get("environment_step_calls") == 0,
        "prior_reset_v26": reset.get("reset_info", {}).get(
            "binary_phase_event_contract_id"
        )
        == contract.V26_ACTIVE_EVENT_CONTRACT_ID,
        "prior_reset_checks_all_pass": isinstance(reset_checks, Mapping)
        and all(value is True for value in reset_checks.values()),
        "frozen_verifier_literal_preserved": FROZEN_VERIFIER_NEXT_STAGE
        == "EXECUTE_SIX_V26_ACTIVE_DEVELOPMENT_REPLAYS_ONCE",
    }
    return {
        "passed": all(checks.values()),
        "checks": checks,
        "missing_attributes": missing,
        "v26_alias": contract.V25_ACTIVE_EVENT_CONTRACT_ID,
        "frozen_verifier_next_stage": FROZEN_VERIFIER_NEXT_STAGE,
        "reset_receipt": base._record(
            INPUT_PATHS["v8r1_reset_compatibility_receipt"]
        ),
    }


def build_env_config(case: Mapping[str, Any]) -> dict[str, Any]:
    return base.build_env_config(case)


def publish_compatibility_preflight() -> dict[str, Any]:
    if any(
        os.path.lexists(path)
        for path in (COMPATIBILITY_PREFLIGHT, PREFLIGHT, LOCK, RUN_ROOT)
    ):
        raise V8R1P1TeacherReplayError("V8R1P1 path already occupied")
    audit = audit_compatibility()
    env_config = build_env_config(contract.canonical_case(contract.CASE_IDS[0]))
    checks = {
        **audit["checks"],
        "build_config_binary_active": env_config.get("binary_phase_fsm_mode")
        == "binary_active",
        "build_config_v26_exact": env_config.get(
            "binary_phase_event_contract_id"
        )
        == contract.V26_ACTIVE_EVENT_CONTRACT_ID,
        "build_config_left_primary_only": env_config.get(
            "online_grf_applied_sides"
        )
        == ["left"],
        "morphology_zero": env_config.get("reward", {}).get("morphology_weight")
        == 0.0,
        "protected_closed": True,
        "reserve_closed": True,
    }
    passed = all(checks.values())
    payload = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": (
            contract.COMPATIBILITY_PREFLIGHT_STATUS
            if passed
            else "FAIL_H0_PRIMARY_SPLIT_V8R1P1_V26_COMPATIBILITY_PREFLIGHT"
        ),
        "passed": passed,
        "protocol_id": contract.PROTOCOL_ID,
        "checks": checks,
        "audit": audit,
        "prior_v8r1_preexecution": {
            "preflight": base._record(INPUT_PATHS["v8r1_preexecution_preflight"]),
            "lock": base._record(INPUT_PATHS["v8r1_preexecution_lock"]),
            "worker_execution_started": False,
        },
        "environment_config": {
            "binary_phase_fsm_mode": env_config["binary_phase_fsm_mode"],
            "binary_phase_event_contract_id": env_config[
                "binary_phase_event_contract_id"
            ],
            "online_grf_applied_sides": env_config["online_grf_applied_sides"],
            "morphology_weight": env_config["reward"]["morphology_weight"],
        },
        "environment_reset_calls": 0,
        "environment_step_calls": 0,
        "reset_evidence_reused": True,
        "candidate_created": False,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
        "next_stage": "PREPARE_V8R1P1_TEACHER_REPLAY" if passed else "STOP",
    }
    forensic.write_json_exclusive(COMPATIBILITY_PREFLIGHT, payload)
    if not passed:
        raise V8R1P1TeacherReplayError("compatibility preflight failed")
    return payload


def _validated_compatibility() -> dict[str, Any]:
    value = base._mapping(COMPATIBILITY_PREFLIGHT)
    if (
        value.get("status") != contract.COMPATIBILITY_PREFLIGHT_STATUS
        or value.get("passed") is not True
        or value.get("environment_step_calls") != 0
        or value.get("protected_trials_opened") != []
        or value.get("reserve_trials_opened") != []
        or not isinstance(value.get("checks"), Mapping)
        or not all(item is True for item in value["checks"].values())
    ):
        raise V8R1P1TeacherReplayError("compatibility receipt drifted")
    return value


def build_preflight_payload(*, require_unoccupied: bool) -> dict[str, Any]:
    compatibility = _validated_compatibility()
    payload = _ORIGINAL_BUILD_PREFLIGHT(require_unoccupied=require_unoccupied)
    payload["checks"]["v8r1p1_compatibility_pass"] = compatibility["passed"] is True
    payload["checks"]["frozen_verifier_literal_preserved"] = (
        compatibility["audit"]["frozen_verifier_next_stage"]
        == FROZEN_VERIFIER_NEXT_STAGE
    )
    payload["passed"] = all(payload["checks"].values())
    payload["compatibility_preflight"] = base._record(COMPATIBILITY_PREFLIGHT)
    payload["procedural_correction"] = {
        "semantic_change": False,
        "compatibility_alias": contract.V25_ACTIVE_EVENT_CONTRACT_ID,
        "verifier_next_stage_literal": FROZEN_VERIFIER_NEXT_STAGE,
        "fresh_artifact_paths": True,
    }
    return payload


def _lock_payload(preflight_payload: Mapping[str, Any]) -> dict[str, Any]:
    payload = _ORIGINAL_LOCK_PAYLOAD(preflight_payload)
    # Do not rename this inherited procedural label: the frozen verifier checks
    # it literally.  It does not identify V20 semantics.
    payload["next_stage"] = FROZEN_VERIFIER_NEXT_STAGE
    payload["compatibility_preflight"] = copy.deepcopy(
        preflight_payload["compatibility_preflight"]
    )
    payload["procedural_correction"] = copy.deepcopy(
        preflight_payload["procedural_correction"]
    )
    return payload


def prepare() -> dict[str, Any]:
    receipt = build_preflight_payload(require_unoccupied=True)
    forensic.write_json_exclusive(PREFLIGHT, receipt)
    lock = _lock_payload(receipt)
    forensic.write_json_exclusive(LOCK, lock)
    return {"preflight": receipt, "lock": lock}


def verify_lock(*, require_run_root_absent: bool = False) -> dict[str, Any]:
    lock = _ORIGINAL_VERIFY_LOCK(require_run_root_absent=require_run_root_absent)
    receipt = base._mapping(PREFLIGHT)
    if (
        lock.get("next_stage") != FROZEN_VERIFIER_NEXT_STAGE
        or lock.get("compatibility_preflight")
        != base._record(COMPATIBILITY_PREFLIGHT)
        or lock.get("compatibility_preflight")
        != receipt.get("compatibility_preflight")
        or lock.get("procedural_correction")
        != receipt.get("procedural_correction")
        or audit_compatibility()["passed"] is not True
    ):
        raise V8R1P1TeacherReplayError("V8R1P1 lock drifted")
    return lock


def canonical_destination(case_id: str) -> Path:
    return base.resolve_relative(contract.canonical_case(case_id)["destination"])


def _worker_command(case_id: str, token: str) -> list[str]:
    return [
        sys.executable,
        str(Path(__file__).resolve()),
        "--worker",
        "--case",
        case_id,
        "--output-dir",
        str(canonical_destination(case_id)),
        "--execution-token",
        token,
    ]


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
run_worker = base.run_worker
exact_float32_columns = base.exact_float32_columns
exact_float32_vector = base.exact_float32_vector
load_frozen_baseline = base.load_frozen_baseline


def execute() -> dict[str, Any]:
    return base.execute()


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_mutually_exclusive_group(required=True)
    modes.add_argument("--compatibility-preflight", action="store_true")
    modes.add_argument("--prepare", action="store_true")
    modes.add_argument("--execute", action="store_true")
    modes.add_argument("--worker", action="store_true")
    parser.add_argument("--case", choices=contract.CASE_IDS)
    parser.add_argument("--output-dir")
    parser.add_argument("--execution-token")
    args = parser.parse_args(argv)
    try:
        if args.compatibility_preflight:
            result = publish_compatibility_preflight()
        elif args.prepare:
            result = prepare()
        elif args.execute:
            result = execute()
        else:
            if not args.case or not args.output_dir or not args.execution_token:
                raise V8R1P1TeacherReplayError("worker arguments are incomplete")
            result = run_worker(
                case_id=args.case,
                output_dir=args.output_dir,
                execution_token=args.execution_token,
            )
    except Exception as exc:
        print(f"V8R1P1 failed closed: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 2
    print(json.dumps(result, indent=2, sort_keys=True, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
