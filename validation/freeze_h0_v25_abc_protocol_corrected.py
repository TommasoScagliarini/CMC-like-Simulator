"""Supersede the rejected H0/V25 A/B/C protocol lock without overwriting it.

The first declarative lock gave case B a redundant scientific-bundle field
that case A did not have.  Although that lock authorized no execution and was
never used, this correction preserves it as immutable rejected evidence and
publishes a new no-clobber lock whose A/B case records differ only by label and
shadow-FSM execution mode.  This script cannot execute H0 or open any authority.
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
if str(VALIDATION_ROOT) not in sys.path:
    sys.path.insert(0, str(VALIDATION_ROOT))

import freeze_h0_v25_abc_protocol as rejected  # noqa: E402
import freeze_v25_shadow_integration_readiness as shadow  # noqa: E402


SCHEMA_VERSION = 2
PROTOCOL_ID = "AB06_H0_V25_ABC_PROTOCOL_ONLY_CORRECTED"
ORIGINAL_PROTOCOL_DATE = "2026-08-04"
CORRECTION_DATE = "2026-08-05"
DESTINATION = VALIDATION_ROOT / "h0_v25_abc_protocol_corrected_lock.json"
REJECTED_LOCK = rejected.DESTINATION
REJECTED_LOCK_SHA256 = (
    "4f24deb43537632f461b67ceb8d04ca4339a6b8e65e904ce7bc039334fb06b8e"
)
REJECTED_SCRIPT_SHA256 = (
    "0c6df5b231548de69cd7257d2f55be8ecc139dfdf41cbd084eccd293659d48e1"
)
CORRECTION_REASON = "B_ONLY_SCIENTIFIC_BUNDLE_SCHEMA_MISMATCH"


class H0V25CorrectedProtocolError(RuntimeError):
    """Raised when the rejected lock cannot be superseded exactly."""


def _rejected_lock() -> tuple[dict[str, Any], dict[str, Any]]:
    record = shadow.source_record(REJECTED_LOCK)
    if record["sha256"] != REJECTED_LOCK_SHA256:
        raise H0V25CorrectedProtocolError("rejected protocol lock drifted")
    payload = shadow.strict_json_load(REJECTED_LOCK, REJECTED_LOCK_SHA256)
    if payload.get("protocol_executed") is not False:
        raise H0V25CorrectedProtocolError("rejected lock claims execution")
    authority = payload.get("authority")
    if not isinstance(authority, Mapping) or not authority:
        raise H0V25CorrectedProtocolError("rejected lock authority is malformed")
    if not all(value is False for value in authority.values()):
        raise H0V25CorrectedProtocolError("rejected lock opened authority")
    freeze_script = payload.get("freeze_script", {})
    if freeze_script.get("sha256") != REJECTED_SCRIPT_SHA256:
        raise H0V25CorrectedProtocolError("rejected freeze-script record drifted")
    if shadow.sha256_file(Path(rejected.__file__)) != REJECTED_SCRIPT_SHA256:
        raise H0V25CorrectedProtocolError("rejected freeze script no longer exact")
    return record, payload


def _remove_redundant_case_bundle(payload: dict[str, Any]) -> None:
    cases = payload["matrix"]["cases"]
    case_a = cases["A"]
    case_b = cases["B"]
    global_bundle = payload["contracts"]["scientific_shadow_bundle_contract_id"]
    if "scientific_bundle_contract_id" in case_a:
        raise H0V25CorrectedProtocolError(
            "rejected schema unexpectedly contains the bundle in A"
        )
    if case_b.get("scientific_bundle_contract_id") != global_bundle:
        raise H0V25CorrectedProtocolError(
            "rejected schema defect is not the preregistered B-only bundle"
        )
    del case_b["scientific_bundle_contract_id"]


def _paired_ab_case_schema_exact(payload: Mapping[str, Any]) -> bool:
    cases = payload.get("matrix", {}).get("cases", {})
    if not isinstance(cases, Mapping):
        return False
    case_a = copy.deepcopy(cases.get("A"))
    case_b = copy.deepcopy(cases.get("B"))
    if not isinstance(case_a, dict) or not isinstance(case_b, dict):
        return False
    if case_a.pop("case_name", None) != "legacy_control":
        return False
    if case_b.pop("case_name", None) != "binary_shadow":
        return False
    if case_a.pop("binary_phase_fsm_mode", None) != "disabled":
        return False
    if case_b.pop("binary_phase_fsm_mode", None) != "binary_shadow":
        return False
    return case_a == case_b


def build_corrected_payload(
    *, require_destination_unoccupied: bool = True
) -> dict[str, Any]:
    if require_destination_unoccupied and os.path.lexists(DESTINATION):
        raise H0V25CorrectedProtocolError(f"refusing to clobber: {DESTINATION}")

    rejected_record, rejected_payload = _rejected_lock()
    reproducible = rejected.build_protocol_payload(
        require_destination_unoccupied=False
    )
    if reproducible != rejected_payload:
        raise H0V25CorrectedProtocolError(
            "rejected lock is not reproducible from its frozen script and inputs"
        )

    payload = copy.deepcopy(reproducible)
    if payload.get("date") != ORIGINAL_PROTOCOL_DATE:
        raise H0V25CorrectedProtocolError("rejected protocol date drifted")
    _remove_redundant_case_bundle(payload)
    if not _paired_ab_case_schema_exact(payload):
        raise H0V25CorrectedProtocolError(
            "corrected A/B case schema still differs beyond label and FSM mode"
        )

    payload["schema_version"] = SCHEMA_VERSION
    payload["protocol_id"] = PROTOCOL_ID
    payload["date"] = CORRECTION_DATE
    payload["status"] = (
        "H0_V25_ABC_PROTOCOL_CORRECTED_FROZEN_EXECUTION_NOT_AUTHORIZED"
    )
    payload["supersedes"] = {
        "rejected_lock": rejected_record,
        "rejected_freeze_script": shadow.source_record(Path(rejected.__file__)),
        "reason": CORRECTION_REASON,
        "rejected_lock_status": rejected_payload["status"],
        "original_protocol_date": ORIGINAL_PROTOCOL_DATE,
        "correction_date": CORRECTION_DATE,
        "rejected_lock_execution_authorized": False,
        "rejected_lock_h0_executed": False,
        "rejected_lock_used_for_execution": False,
        "rejected_lock_deleted_or_overwritten": False,
        "rejected_lock_must_not_be_used": True,
    }
    payload["provenance"]["superseded_rejected_protocol_lock"] = rejected_record
    payload["correction"] = {
        "schema_defect": CORRECTION_REASON,
        "original_protocol_date": ORIGINAL_PROTOCOL_DATE,
        "correction_date": CORRECTION_DATE,
        "removed_from_case_B": ["scientific_bundle_contract_id"],
        "removed_from_case_A": [],
        "scientific_shadow_bundle_retained_globally": payload["contracts"][
            "scientific_shadow_bundle_contract_id"
        ],
        "paired_case_allowed_differences": [
            "case_name",
            "binary_phase_fsm_mode",
        ],
        "scientific_semantics_changed": False,
        "execution_or_data_access_occurred_before_correction": False,
    }
    payload["assertions"]["superseded_lock_exact_never_authorized_or_used"] = (
        rejected_record["sha256"] == REJECTED_LOCK_SHA256
        and rejected_payload["protocol_executed"] is False
        and all(value is False for value in rejected_payload["authority"].values())
    )
    payload["assertions"]["paired_ab_case_schema_exact_after_correction"] = (
        _paired_ab_case_schema_exact(payload)
    )
    payload["assertions"]["corrected_authority_remains_closed"] = all(
        value is False for value in payload["authority"].values()
    )
    if not all(payload["assertions"].values()):
        raise H0V25CorrectedProtocolError(
            f"corrected protocol assertions failed: {payload['assertions']}"
        )
    payload["freeze_script"] = shadow.source_record(Path(__file__))
    payload["next_stage"] = "IMPLEMENT_AND_FREEZE_SEPARATE_EXECUTION_UNLOCK_OR_STOP"
    shadow.encode_json(payload)
    return payload


def preflight_unfrozen() -> dict[str, Any]:
    payload = build_corrected_payload(require_destination_unoccupied=True)
    return {
        "status": "H0_V25_ABC_CORRECTED_PROTOCOL_LOCK_READY_UNWRITTEN",
        "destination_unoccupied": True,
        "execution_authorized": False,
        "lock_record_if_frozen": shadow.payload_record(DESTINATION, payload),
        "protocol_payload": payload,
    }


def freeze_corrected_protocol() -> dict[str, Any]:
    payload = build_corrected_payload(require_destination_unoccupied=True)
    try:
        shadow.write_json_exclusive(DESTINATION, payload)
    except shadow.V25ShadowReadinessError as exc:
        raise H0V25CorrectedProtocolError(str(exc)) from exc
    return payload


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--check", action="store_true")
    mode.add_argument("--freeze", action="store_true")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        payload = preflight_unfrozen() if args.check else freeze_corrected_protocol()
    except Exception as exc:
        print(
            "Corrected H0/V25 protocol freeze failed closed: "
            f"{type(exc).__name__}: {exc}",
            file=sys.stderr,
        )
        return 2
    print(json.dumps(payload, indent=2, sort_keys=True, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
