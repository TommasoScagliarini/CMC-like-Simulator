"""Execute the one no-checkpoint V11 weighted-fit design audit.

The numerical design, source H0, base corpus, gates, and implementation files
are fixed before this command starts.  It performs exactly one in-memory P0
fit, writes only the canonical strict-JSON receipt, and refuses to clobber an
existing receipt.  It does not save a candidate module or authorize pipeline
execution by itself.
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path
from typing import Any, Mapping, Sequence


REPO_ROOT = Path(__file__).resolve().parents[1]
VALIDATION_ROOT = REPO_ROOT / "validation"
TRAJECTORY_ROOT = REPO_ROOT / "Trajectory Generator"
BASELINE_ROOT = TRAJECTORY_ROOT / "baseline_MLP"
for _root in (REPO_ROOT, VALIDATION_ROOT, TRAJECTORY_ROOT, BASELINE_ROOT):
    if str(_root) not in sys.path:
        sys.path.insert(0, str(_root))

import h0_forensic_rollout as forensic  # noqa: E402
import h0_primary_split_v11_weighted_fit as fit_engine  # noqa: E402
import h0_primary_split_v11_weighted_full_mean_contract as contract  # noqa: E402


class V11DesignAuditError(RuntimeError):
    """Raised when the one-shot audit cannot be published fail-closed."""


class V11DesignAuditConsumedFailure(V11DesignAuditError):
    """Raised after a canonical failure receipt consumes the audit stage."""


def _canonical_receipt_path() -> Path:
    return (REPO_ROOT / contract.DESIGN_AUDIT_RECEIPT_PATH).resolve()


def _artifact_record(path: str | os.PathLike[str]) -> dict[str, Any]:
    return forensic.artifact_record(Path(path).resolve(), artifact_root=REPO_ROOT)


def _failure_receipt(error: BaseException) -> dict[str, Any]:
    """Build an auditable terminal FAIL without claiming a completed fit."""

    source_bindings: dict[str, Any] = {}
    for name, relative in contract.DESIGN_AUDIT_SOURCE_RELATIVE_PATHS.items():
        try:
            source_bindings[name] = _artifact_record(REPO_ROOT / relative)
        except Exception:
            source_bindings[name] = None
    try:
        source_h0: Any = fit_engine._tree_record(contract.SOURCE_H0_MODULE_PATH)
    except Exception:
        source_h0 = None
    try:
        corpus_artifact: Any = _artifact_record(REPO_ROOT / contract.V10S_P0_CORPUS_PATH)
    except Exception:
        corpus_artifact = None
    payload: dict[str, Any] = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.DESIGN_AUDIT_FAIL_STATUS,
        "passed": False,
        "protocol_id": contract.PROTOCOL_ID,
        "contract_id": contract.FIT_CONTRACT_ID,
        "design_audit_id": contract.DESIGN_AUDIT_ID,
        "gate": {"passed": False},
        "audit_consumed": True,
        "dry_run": True,
        "no_candidate_checkpoint": True,
        "fit_design": contract.FIT,
        "gates": contract.OFFLINE_THRESHOLDS,
        "source_bindings": source_bindings,
        "source_h0": source_h0,
        "corpus": {"artifact": corpus_artifact},
        "error": {
            "type": type(error).__name__,
            "message": str(error),
        },
        "artifacts_written": [contract.DESIGN_AUDIT_RECEIPT_PATH.as_posix()],
        "candidate_checkpoint_paths": [],
        "candidate_checkpoints_persisted": 0,
        "actor_fit_execution_attempted": True,
        "actor_update_attempted_or_unknown": True,
        "confirmed_actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
        "retry_authorized": False,
        "sweep_authorized": False,
        "next_stage": "STOP_V11_DESIGN_AUDIT_CONSUMED_NO_RETRY",
    }
    payload["gate"] = dict(contract.design_audit_gate(payload))
    if payload["gate"].get("passed") is not False:
        raise V11DesignAuditError("failure receipt was incorrectly accepted")
    return payload


def execute_design_audit(
    *,
    output_path: str | os.PathLike[str] | None = None,
    enforce_canonical_destination: bool = True,
) -> dict[str, Any]:
    """Run P0 in memory and atomically publish its sole receipt."""

    destination = (
        Path(output_path).expanduser().resolve()
        if output_path is not None
        else _canonical_receipt_path()
    )
    if enforce_canonical_destination and destination != _canonical_receipt_path():
        raise V11DesignAuditError(f"non-canonical design-audit receipt: {destination}")
    if os.path.lexists(destination):
        raise V11DesignAuditError(
            f"design-audit receipt already exists/no-clobber: {destination}"
        )

    try:
        payload = fit_engine.run_design_audit_in_memory()
        if not isinstance(payload, Mapping):
            raise V11DesignAuditError("fit engine returned a malformed audit receipt")
        payload = dict(payload)
        gate = dict(contract.design_audit_gate(payload))
        if gate.get("passed") is not True or payload.get("gate") != gate:
            failed = [
                name
                for name, value in gate.get("checks", {}).items()
                if value is not True
            ]
            raise V11DesignAuditError(f"design-audit contract failed: {failed}")
        if payload.get("artifacts_written") != [
            contract.DESIGN_AUDIT_RECEIPT_PATH.as_posix()
        ]:
            raise V11DesignAuditError(
                "audit receipt does not declare receipt-only output"
            )
        if payload.get("candidate_checkpoint_paths") != []:
            raise V11DesignAuditError(
                "audit attempted to declare a candidate checkpoint"
            )
    except Exception as exc:
        failure = _failure_receipt(exc)
        try:
            forensic.write_json_exclusive(destination, failure)
        except Exception as publish_exc:
            raise V11DesignAuditError(
                "audit failed and its consuming failure receipt could not be published"
            ) from publish_exc
        raise V11DesignAuditConsumedFailure(
            f"design audit failed and is consumed: {type(exc).__name__}: {exc}"
        ) from exc

    try:
        receipt_path = forensic.write_json_exclusive(destination, payload)
        reloaded = forensic.strict_json_load(receipt_path)
    except Exception as exc:
        raise V11DesignAuditError("could not publish strict no-clobber receipt") from exc
    if not isinstance(reloaded, Mapping) or forensic.canonical_json_bytes(
        reloaded
    ) != forensic.canonical_json_bytes(payload):
        raise V11DesignAuditError("published audit receipt failed exact reload")
    if contract.design_audit_gate(reloaded).get("passed") is not True:
        raise V11DesignAuditError("published audit receipt failed gate after reload")
    return dict(reloaded)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--execute",
        action="store_true",
        help="perform the sole in-memory audit and write the canonical receipt",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    if not args.execute:
        raise V11DesignAuditError(
            "explicit --execute is required for the one-shot no-clobber audit"
        )
    try:
        receipt = execute_design_audit()
    except V11DesignAuditConsumedFailure as exc:
        print(str(exc), file=sys.stderr)
        return 1
    print(
        f"{receipt['status']}: {contract.DESIGN_AUDIT_RECEIPT_PATH.as_posix()}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = [
    "V11DesignAuditConsumedFailure",
    "V11DesignAuditError",
    "execute_design_audit",
    "main",
]
