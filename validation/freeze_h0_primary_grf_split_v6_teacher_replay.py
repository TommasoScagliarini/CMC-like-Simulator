"""Freeze the no-clobber V6 V25-active teacher-replay development closure."""

from __future__ import annotations

import json
import os
import sys
from pathlib import Path
from typing import Any, Mapping


REPO_ROOT = Path(__file__).resolve().parents[1]
VALIDATION_ROOT = REPO_ROOT / "validation"
if str(VALIDATION_ROOT) not in sys.path:
    sys.path.insert(0, str(VALIDATION_ROOT))

import build_h0_primary_grf_split_v6_teacher_replay_preflight as preflight  # noqa: E402
import h0_forensic_rollout as forensic  # noqa: E402
import h0_primary_grf_split_v6_teacher_replay_contract as contract  # noqa: E402


LOCK_PATH = preflight.resolve_relative(contract.LOCK_PATH)
PREFLIGHT_PATH = preflight.resolve_relative(contract.PREFLIGHT_RECEIPT_PATH)
RUN_ROOT = preflight.resolve_relative(contract.RUN_ROOT)


class V6TeacherReplayFreezeError(RuntimeError):
    """Raised when the V6 development closure cannot be frozen exactly."""


def _mapping(path: Path) -> dict[str, Any]:
    value = forensic.strict_json_load(path)
    if not isinstance(value, Mapping):
        raise V6TeacherReplayFreezeError(f"expected JSON object: {path}")
    return dict(value)


def verify_preflight() -> dict[str, Any]:
    observed = _mapping(PREFLIGHT_PATH)
    expected = preflight.build_payload(require_destinations_absent=True)
    if forensic.canonical_json_bytes(observed) != forensic.canonical_json_bytes(
        expected
    ):
        raise V6TeacherReplayFreezeError("V6 teacher replay preflight drifted")
    if (
        observed.get("status") != contract.PREFLIGHT_STATUS
        or observed.get("passed") is not True
        or not isinstance(observed.get("checks"), Mapping)
        or not all(observed["checks"].values())
        or observed.get("simulations_executed") != 0
        or observed.get("candidate_created") is not False
        or observed.get("actor_updates") != 0
        or observed.get("critic_updates") != 0
        or observed.get("ppo_updates") != 0
        or observed.get("protected_trials_opened") != []
    ):
        raise V6TeacherReplayFreezeError("V6 preflight is not a canonical PASS")
    return observed


def build_payload() -> dict[str, Any]:
    receipt = verify_preflight()
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
            "expected_v25_raw_sensor_samples": (contract.EXPECTED_RAW_SENSOR_SAMPLES),
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
        },
        "preflight_receipt": preflight.source_record(PREFLIGHT_PATH),
        "sources": dict(receipt["sources"]),
        "inputs": dict(receipt["inputs"]),
        "v5_terminal": dict(receipt["v5_terminal"]),
        "v25": dict(receipt["v25"]),
        "source_h0": dict(receipt["source_h0"]),
        "development_reclassification": {
            "source": "V5_PASS_CONDITION_MATCHED_BASELINES",
            "case_ids": list(contract.CASE_IDS),
            "new_role": "V6_DEVELOPMENT_TEACHER_ACTION_REPLAY",
            "eligible_as_future_holdout": False,
        },
        "authority": dict(contract.AUTHORITY),
        "simulations_executed_at_freeze": 0,
        "candidate_created": False,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "next_stage": "EXECUTE_SIX_V25_ACTIVE_DEVELOPMENT_REPLAYS_ONCE",
    }


def freeze() -> dict[str, Any]:
    if os.path.lexists(LOCK_PATH):
        raise V6TeacherReplayFreezeError(f"execution lock exists: {LOCK_PATH}")
    if os.path.lexists(RUN_ROOT):
        raise V6TeacherReplayFreezeError(f"run root already claimed: {RUN_ROOT}")
    destinations = [
        preflight.resolve_relative(case["destination"]) for case in contract.CASES
    ]
    if any(os.path.lexists(path) for path in destinations):
        raise V6TeacherReplayFreezeError("a frozen replay destination is occupied")
    payload = build_payload()
    forensic.write_json_exclusive(LOCK_PATH, payload)
    return payload


def main() -> int:
    try:
        result = freeze()
    except Exception as exc:
        print(
            f"V6 teacher replay freeze failed: {type(exc).__name__}: {exc}",
            file=sys.stderr,
        )
        return 2
    print(json.dumps(result, indent=2, sort_keys=True, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
