"""Freeze the no-clobber V7 early-contact shadow diagnostic closure."""

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

import build_h0_primary_split_v7_early_contact_preflight as preflight  # noqa: E402
import h0_forensic_rollout as forensic  # noqa: E402
import h0_primary_split_v7_early_contact_contract as contract  # noqa: E402


LOCK_PATH = preflight.resolve_relative(contract.LOCK_PATH)
PREFLIGHT_PATH = preflight.resolve_relative(contract.PREFLIGHT_RECEIPT_PATH)
RUN_ROOT = preflight.resolve_relative(contract.RUN_ROOT)
DESTINATION = RUN_ROOT / contract.CASE_ID


class V7EarlyContactFreezeError(RuntimeError):
    """Raised when the diagnostic closure cannot be frozen exactly."""


def _mapping(path: Path) -> dict[str, Any]:
    value = forensic.strict_json_load(path)
    if not isinstance(value, Mapping):
        raise V7EarlyContactFreezeError(f"expected JSON object: {path}")
    return dict(value)


def verify_preflight() -> dict[str, Any]:
    observed = _mapping(PREFLIGHT_PATH)
    expected = preflight.build_payload(require_destinations_absent=True)
    if forensic.canonical_json_bytes(observed) != forensic.canonical_json_bytes(
        expected
    ):
        raise V7EarlyContactFreezeError("V7 preflight receipt drifted")
    if (
        observed.get("status") != contract.PREFLIGHT_PASS_STATUS
        or observed.get("passed") is not True
        or not isinstance(observed.get("checks"), Mapping)
        or not all(value is True for value in observed["checks"].values())
        or observed.get("simulations_executed") != 0
        or observed.get("candidate_created") is not False
        or observed.get("candidate_implemented") is not False
        or observed.get("runtime_promoted") is not False
        or observed.get("actor_updates") != 0
        or observed.get("critic_updates") != 0
        or observed.get("ppo_updates") != 0
        or observed.get("protected_trials_opened") != []
        or observed.get("reserve_trials_opened") != []
    ):
        raise V7EarlyContactFreezeError("V7 preflight is not a canonical PASS")
    return observed


def build_payload_from_receipt(
    receipt: Mapping[str, Any],
    *,
    preflight_receipt_record: Mapping[str, Any],
) -> dict[str, Any]:
    """Build the complete canonical lock from already-verified inputs.

    Keeping this transformation free of filesystem reads lets the execution
    supervisor reconstruct the exact lock after the run root has been claimed.
    """

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
        "execution_order": [contract.CASE_ID],
        "case": dict(contract.CASE),
        "destination": DESTINATION.relative_to(REPO_ROOT).as_posix(),
        "matrix": {
            "rollout_count": 1,
            "condition": "DETERMINISTIC_NOMINAL_ALREADY_OPEN_DEVELOPMENT",
            "behavior": contract.CASE["behavior"],
            "actor_input_view": "historical_analog",
            "source_event_contract_id": (
                "primary_grf_split_v1+legacy_events_v1"
            ),
            "detector_mode": "binary_shadow",
            "shadow_event_contract_id": contract.SHADOW_EVENT_CONTRACT_ID,
            "steps": contract.EXPECTED_STEPS,
        },
        "diagnostic_contract": {
            "known_v6_preceding_to_time_s": contract.V6_PRECEDING_TO_TIME_S,
            "known_v6_early_hs_onset_time_s": contract.V6_EARLY_HS_ONSET_TIME_S,
            "known_v6_swing_duration_s": contract.V6_OBSERVED_SWING_S,
            "early_window_half_open_s": [
                contract.MIN_SWING_CANDIDATE_S,
                contract.LEGACY_MIN_SWING_DURATION_S,
            ],
            "primary_force_threshold_n": contract.PRIMARY_FORCE_THRESHOLD_N,
            "primary_crossing_deadline_s": (
                contract.PRIMARY_CROSSING_DEADLINE_S
            ),
            "minimum_accepted_stance_s": contract.MIN_ACCEPTED_STANCE_S,
            "maximum_transient_contact_s": contract.MAX_TRANSIENT_CONTACT_S,
            "primary_source_sample_dt_s": contract.EXPECTED_SAMPLE_DT_S,
            "primary_evidence_delivery_dt_s": contract.EXPECTED_POLICY_DT_S,
            "primary_load_alignment_id": contract.PRIMARY_LOAD_ALIGNMENT_ID,
            "primary_interpolation": False,
            "primary_load_evidence_role": contract.PRIMARY_LOAD_EVIDENCE_ROLE,
            "canonical_scientific_oracle": contract.CANONICAL_SCIENTIFIC_ORACLE,
            "primary_online_grf_used_as_event_source": False,
            "permitted_classifications": [
                contract.PERSISTENT_LANDING,
                contract.TRANSIENT_TOE_SCUFF,
            ],
            "ambiguous_result": contract.AMBIGUOUS_FAIL,
        },
        "gate": {
            "expected_steps": contract.EXPECTED_STEPS,
            "expected_control_windows": contract.EXPECTED_CONTROL_WINDOWS,
            "expected_v25_raw_sensor_samples": (
                contract.EXPECTED_RAW_SENSOR_SAMPLES
            ),
            "expected_primary_load_samples": (
                contract.EXPECTED_PRIMARY_LOAD_SAMPLES
            ),
            "v5_projected_trace_bit_exact": True,
            "penetration_limit_m": contract.PENETRATION_LIMIT_M,
            "penetration_comparison": "strict_less_than",
            "persist_before_gate": [
                "steps/000001.json...steps/000500.json",
                "v25_raw_journal.json",
                "trace.json",
                "partial_summary.json",
                "summary.json",
            ],
            "classification_must_be_non_ambiguous": True,
            "all_v25_flights_analyzed": True,
            "minimum_v25_flight_s": contract.MIN_SWING_CANDIDATE_S,
        },
        "preflight_receipt": dict(preflight_receipt_record),
        "sources": dict(receipt["sources"]),
        "inputs": dict(receipt["inputs"]),
        "v5_nominal": dict(receipt["v5_nominal"]),
        "v6_terminal": dict(receipt["v6_terminal"]),
        "frozen_runtime": dict(receipt["frozen_runtime"]),
        "runtime_closure": dict(receipt["runtime_closure"]),
        "platform": dict(receipt["platform"]),
        "platform_attestation": dict(receipt["platform_attestation"]),
        "primary_core": dict(receipt["primary_core"]),
        "authority": dict(contract.AUTHORITY),
        "simulations_executed_at_freeze": 0,
        "candidate_created": False,
        "candidate_implemented": False,
        "runtime_promoted": False,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
        "next_stage": "EXECUTE_V7_EARLY_CONTACT_SHADOW_DIAGNOSTIC_ONCE",
    }


def build_payload() -> dict[str, Any]:
    receipt = verify_preflight()
    return build_payload_from_receipt(
        receipt,
        preflight_receipt_record=preflight.source_record(PREFLIGHT_PATH),
    )


def freeze() -> dict[str, Any]:
    if os.path.lexists(LOCK_PATH):
        raise V7EarlyContactFreezeError(f"execution lock exists: {LOCK_PATH}")
    if os.path.lexists(RUN_ROOT):
        raise V7EarlyContactFreezeError(f"run root already claimed: {RUN_ROOT}")
    if os.path.lexists(DESTINATION):
        raise V7EarlyContactFreezeError(
            f"diagnostic destination is occupied: {DESTINATION}"
        )
    payload = build_payload()
    forensic.write_json_exclusive(LOCK_PATH, payload)
    return payload


def main() -> int:
    try:
        result = freeze()
    except Exception as exc:
        print(
            f"V7 early-contact freeze failed: {type(exc).__name__}: {exc}",
            file=sys.stderr,
        )
        return 2
    print(json.dumps(result, indent=2, sort_keys=True, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
