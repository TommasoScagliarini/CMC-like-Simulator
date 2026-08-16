"""Freeze V12R8-Q3 only after five live semantic R8 verifiers pass.

Importing this module is inert.  Publication is explicit, exclusive, and
fail-closed; the canonical Q3 paths must all be absent when the live R8
snapshot is acquired.  Unit tests exercise the pure payload builders with
synthetic records and never publish canonical artifacts.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import importlib
import os
import sys
from collections.abc import Mapping, Sequence
from pathlib import Path, PurePosixPath
from typing import Any


RUNTIME_ROOT = Path(__file__).resolve().parent
Q3_ROOT = RUNTIME_ROOT.parent
LOCAL_VALIDATION = Q3_ROOT.parent
REPO_ROOT = Q3_ROOT.parents[3]
for _root in (
    Q3_ROOT,
    RUNTIME_ROOT,
    LOCAL_VALIDATION / "v12r8",
    REPO_ROOT / "validation",
):
    if str(_root) not in sys.path:
        sys.path.insert(0, str(_root))

import h0_forensic_rollout as forensic  # noqa: E402
import h0_v12r8_q3_artifacts as artifacts  # noqa: E402
import h0_v12r8_q3_prerequisites as prerequisites  # noqa: E402
import h0_v12r8_q3_qualification_contract as contract  # noqa: E402


class V12R8Q3FreezeError(RuntimeError):
    """Raised before any unsafe, premature, or drifting Q3 freeze."""


PROTOCOL_FREEZE_PASS_STATUS = "PASS_H0_V12R8_Q3_QUALIFICATION_PROTOCOL_FREEZE"
EXECUTION_LOCK_PASS_STATUS = "PASS_H0_V12R8_Q3_QUALIFICATION_EXECUTION_LOCK"


def resolve_relative(value: str | PurePosixPath) -> Path:
    raw = value.as_posix() if isinstance(value, PurePosixPath) else value
    if not artifacts.canonical_relative_path(raw):
        raise V12R8Q3FreezeError(f"non-canonical repository path: {raw!r}")
    return REPO_ROOT.joinpath(*PurePosixPath(raw).parts)


def _record(value: str | PurePosixPath | Path) -> dict[str, Any]:
    path = value if isinstance(value, Path) else resolve_relative(value)
    try:
        return forensic.artifact_record(path, artifact_root=REPO_ROOT)
    except BaseException as exc:
        raise V12R8Q3FreezeError(f"unsafe or missing artifact: {path}") from exc


def _mapping(path: str | PurePosixPath | Path) -> dict[str, Any]:
    source = path if isinstance(path, Path) else resolve_relative(path)
    try:
        value = forensic.strict_json_load(source)
    except BaseException as exc:
        raise V12R8Q3FreezeError(f"strict JSON could not be read: {source}") from exc
    if not isinstance(value, Mapping):
        raise V12R8Q3FreezeError(f"expected JSON object: {source}")
    result = dict(value)
    if source.read_bytes() != forensic.canonical_json_bytes(result):
        raise V12R8Q3FreezeError(f"JSON is not canonical: {source}")
    return result


def _verifier_payload_matches_record(
    payload: Any, record: Any, *, expected_path: str | PurePosixPath
) -> bool:
    """Bind a live verifier return value to the exact canonical JSON file."""

    if not isinstance(payload, Mapping):
        return False
    try:
        encoded = forensic.canonical_json_bytes(dict(payload))
    except BaseException:
        return False
    expected = (
        expected_path.as_posix()
        if isinstance(expected_path, PurePosixPath)
        else expected_path
    )
    return (
        artifacts.artifact_record_matches(record, expected)
        and record.get("sha256") == hashlib.sha256(encoded).hexdigest()
        and record.get("size_bytes") == len(encoded)
    )


def official_r8_hooks() -> prerequisites.R8SemanticVerifierHooks:
    """Load the five public R8 verifiers lazily in the RLlib environment."""

    try:
        freezer = importlib.import_module("freeze_h0_v12r8_recovery")
        runner = importlib.import_module("run_h0_v12r8_recovery")
    except BaseException as exc:
        raise V12R8Q3FreezeError("R8 verifier modules are unavailable") from exc
    return prerequisites.R8SemanticVerifierHooks(
        verify_protocol_freeze=freezer.verify_protocol_freeze,
        verify_execution_lock=lambda: freezer.verify_execution_lock(
            require_pristine=False
        ),
        verify_candidate_freeze_receipt=runner.verify_candidate_freeze_receipt,
        verify_final_development_receipt=runner.verify_final_development_receipt,
        verify_terminal_ledger=runner.verify_terminal_ledger,
    )


def _q3_occupied() -> list[str]:
    paths = (
        contract.PROTOCOL_FREEZE_PATH,
        contract.EXECUTION_LOCK_PATH,
        contract.NOISE_ROOT,
        contract.RUN_ROOT,
    )
    return [
        path.as_posix() for path in paths if os.path.lexists(resolve_relative(path))
    ]


def live_r8_prerequisite_snapshot(
    *,
    hooks: prerequisites.R8SemanticVerifierHooks | None = None,
    require_q3_unopened: bool = True,
) -> dict[str, Any]:
    """Call five live verifiers and cross-bind their exact files and actor manifest."""

    occupied = _q3_occupied() if require_q3_unopened else []
    if occupied:
        raise V12R8Q3FreezeError(f"Q3 namespace is already occupied: {occupied}")
    selected = official_r8_hooks() if hooks is None else hooks
    try:
        payloads = prerequisites.invoke_verifier_hooks(selected)
    except BaseException as exc:
        raise V12R8Q3FreezeError("five live R8 verifiers did not converge") from exc
    requirements = contract.prerequisite_requirements()
    records = {row["name"]: _record(row["path"]) for row in requirements}
    mismatched = [
        row["name"]
        for row in requirements
        if not _verifier_payload_matches_record(
            payloads.get(row["name"]),
            records.get(row["name"]),
            expected_path=row["path"],
        )
    ]
    if mismatched:
        raise V12R8Q3FreezeError(
            f"R8 verifier payload/file binding drifted: {mismatched}"
        )
    manifest_path = contract.CANDIDATE_MODULE_PATH / "actor_feature_manifest.json"
    manifest = _mapping(manifest_path)
    manifest_record = _record(manifest_path)
    gate = prerequisites.validate_verified_payloads(
        payloads,
        records,
        actor_manifest=manifest,
        actor_manifest_record=manifest_record,
    )
    if gate.get("passed") is not True:
        failed = sorted(
            name
            for name, passed in gate.get("checks", {}).items()
            if passed is not True
        )
        raise V12R8Q3FreezeError(f"semantic R8 prerequisite gate failed: {failed}")
    return {
        "payloads": payloads,
        "records": records,
        "actor_feature_manifest": manifest,
        "actor_feature_manifest_record": manifest_record,
        "gate": gate,
    }


def source_closure() -> dict[str, dict[str, Any]]:
    """Hash every preregistered Q3 source/runtime dependency and immutable input."""

    expected = {
        **contract.QUALIFICATION_SOURCE_PATHS,
        **contract.DEFERRED_RUNTIME_SOURCE_PATHS,
        **contract.QUALIFICATION_INPUT_PATHS,
    }
    records = {name: _record(path) for name, path in sorted(expected.items())}
    gate = prerequisites.validate_source_closure(records)
    if gate.get("passed") is not True:
        failed = sorted(
            name for name, passed in gate["checks"].items() if passed is not True
        )
        raise V12R8Q3FreezeError(f"Q3 source closure failed: {failed}")
    return records


def source_closure_readiness() -> dict[str, Any]:
    """Report missing preregistered sources without weakening ``source_closure``."""

    expected = {
        **contract.QUALIFICATION_SOURCE_PATHS,
        **contract.DEFERRED_RUNTIME_SOURCE_PATHS,
        **contract.QUALIFICATION_INPUT_PATHS,
    }
    missing = [
        name
        for name, path in sorted(expected.items())
        if not resolve_relative(path).is_file()
    ]
    if missing:
        return {
            "status": "DEFERRED_H0_V12R8_Q3_SOURCE_CLOSURE",
            "passed": False,
            "missing_sources": missing,
            "records": {},
        }
    try:
        records = source_closure()
    except V12R8Q3FreezeError as exc:
        return {
            "status": "FAIL_H0_V12R8_Q3_SOURCE_CLOSURE",
            "passed": False,
            "missing_sources": [],
            "error": str(exc),
            "records": {},
        }
    return {
        "status": "PASS_H0_V12R8_Q3_SOURCE_CLOSURE",
        "passed": True,
        "missing_sources": [],
        "records": records,
    }


def build_protocol_payload(
    *,
    prerequisite_snapshot: Mapping[str, Any],
    closure: Mapping[str, Mapping[str, Any]],
) -> dict[str, Any]:
    """Build the immutable candidate-bound protocol as a pure function."""

    gate = prerequisite_snapshot.get("gate")
    if not isinstance(gate, Mapping) or gate.get("passed") is not True:
        raise V12R8Q3FreezeError("protocol requires a semantic terminal-PASS R8 gate")
    binding = prerequisites.validate_candidate_tree(
        gate.get("candidate_id"), gate.get("candidate_module")
    )
    closure_gate = prerequisites.validate_source_closure(closure)
    if closure_gate.get("passed") is not True:
        raise V12R8Q3FreezeError("protocol source closure is incomplete")
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": PROTOCOL_FREEZE_PASS_STATUS,
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "revision": contract.REVISION,
        "authority": copy.deepcopy(contract.AUTHORITY),
        "candidate_binding_state": "BOUND_FROM_R8_TERMINAL_PASS",
        "candidate_selection_rule": contract.CANDIDATE_SELECTION_RULE,
        "candidate_id": binding["candidate_id"],
        "candidate_module": binding["candidate_module"],
        "candidate_required_files": list(contract.CANDIDATE_REQUIRED_FILES),
        "r8_prerequisite_gate": copy.deepcopy(dict(gate)),
        "r8_prerequisite_records": copy.deepcopy(
            dict(prerequisite_snapshot.get("records", {}))
        ),
        "actor_feature_manifest": copy.deepcopy(
            prerequisite_snapshot.get("actor_feature_manifest_record")
        ),
        "source_closure": copy.deepcopy(dict(closure)),
        "rollout_matrix": list(contract.ROLLOUT_MATRIX),
        "candidate_resolved_env_config": copy.deepcopy(
            contract.CANDIDATE_RESOLVED_ENV_CONFIG
        ),
        "baseline_resolved_env_config": copy.deepcopy(
            contract.BASELINE_RESOLVED_ENV_CONFIG
        ),
        "morphology_zero_ab_contract": copy.deepcopy(
            contract.MORPHOLOGY_ZERO_AB_CONTRACT
        ),
        "expected_tape_array_sha256": copy.deepcopy(
            contract.EXPECTED_TAPE_ARRAY_SHA256
        ),
        "noninferiority_tolerances": [
            list(row) for row in contract.NONINFERIORITY_TOLERANCES
        ],
        "one_shot": True,
        "baseline_first": True,
        "retry_authorized": False,
        "resume_authorized": False,
        "rescue_authorized": False,
        "sweep_authorized": False,
        "post_hoc_tuning_authorized": False,
        "checkpoint_zero_created": False,
        "positive_morphology_enabled": False,
        "runtime_promoted": False,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
    }


def protocol_payload_gate(payload: Any) -> dict[str, Any]:
    data = dict(payload) if isinstance(payload, Mapping) else {}
    candidate = data.get("candidate_module")
    try:
        binding = prerequisites.validate_candidate_tree(
            data.get("candidate_id"), candidate
        )
    except (TypeError, ValueError):
        binding = None
    closure = data.get("source_closure")
    closure_gate = prerequisites.validate_source_closure(closure)
    checks = {
        "schema": data.get("schema_version") == contract.SCHEMA_VERSION,
        "status": data.get("status") == PROTOCOL_FREEZE_PASS_STATUS
        and data.get("passed") is True,
        "identity": data.get("protocol_id") == contract.PROTOCOL_ID
        and data.get("pipeline_id") == contract.PIPELINE_ID,
        "candidate": binding is not None
        and data.get("candidate_binding_state") == "BOUND_FROM_R8_TERMINAL_PASS"
        and data.get("candidate_selection_rule") == contract.CANDIDATE_SELECTION_RULE,
        "prerequisite": isinstance(data.get("r8_prerequisite_gate"), Mapping)
        and data["r8_prerequisite_gate"].get("passed") is True,
        "source_closure": closure_gate.get("passed") is True,
        "baseline_first": data.get("rollout_matrix") == list(contract.ROLLOUT_MATRIX)
        and data.get("baseline_first") is True,
        "resolved_v26": data.get("candidate_resolved_env_config")
        == contract.CANDIDATE_RESOLVED_ENV_CONFIG,
        "morphology_zero": data.get("morphology_zero_ab_contract")
        == contract.MORPHOLOGY_ZERO_AB_CONTRACT,
        "zero_updates": type(data.get("actor_updates")) is int
        and data.get("actor_updates") == 0
        and type(data.get("critic_updates")) is int
        and data.get("critic_updates") == 0
        and type(data.get("ppo_updates")) is int
        and data.get("ppo_updates") == 0,
        "closed": data.get("one_shot") is True
        and data.get("retry_authorized") is False
        and data.get("resume_authorized") is False
        and data.get("runtime_promoted") is False,
    }
    return {"passed": all(checks.values()), "checks": checks}


def build_execution_lock_payload(
    *,
    protocol_payload: Mapping[str, Any],
    protocol_record: Mapping[str, Any],
    prerequisite_snapshot: Mapping[str, Any],
    closure: Mapping[str, Mapping[str, Any]],
    occupancy: Mapping[str, Any],
) -> dict[str, Any]:
    if protocol_payload_gate(protocol_payload).get("passed") is not True:
        raise V12R8Q3FreezeError("execution lock requires an exact protocol payload")
    expected_occupancy = {
        "noise_root_absent": True,
        "run_root_absent": True,
        "pipeline_claim_absent": True,
        "pipeline_ledger_absent": True,
    }
    if dict(occupancy) != expected_occupancy:
        raise V12R8Q3FreezeError("execution lock occupancy is not pristine")
    gate = prerequisite_snapshot["gate"]
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": EXECUTION_LOCK_PASS_STATUS,
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "candidate_id": gate["candidate_id"],
        "candidate_module": copy.deepcopy(gate["candidate_module"]),
        "protocol_freeze": copy.deepcopy(dict(protocol_record)),
        "r8_prerequisite_gate": copy.deepcopy(dict(gate)),
        "source_closure": copy.deepcopy(dict(closure)),
        "occupancy": copy.deepcopy(dict(occupancy)),
        "rollout_matrix": list(contract.ROLLOUT_MATRIX),
        "case_order": list(contract.CASE_IDS),
        "role_order": list(contract.ROLE_ORDER),
        "one_shot": True,
        "baseline_first": True,
        "retry_authorized": False,
        "resume_authorized": False,
        "rescue_authorized": False,
        "sweep_authorized": False,
        "post_hoc_tuning_authorized": False,
        "checkpoint_zero_created": False,
        "positive_morphology_enabled": False,
        "runtime_promoted": False,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
    }


def _occupancy() -> dict[str, bool]:
    return {
        "noise_root_absent": not os.path.lexists(resolve_relative(contract.NOISE_ROOT)),
        "run_root_absent": not os.path.lexists(resolve_relative(contract.RUN_ROOT)),
        "pipeline_claim_absent": not os.path.lexists(
            resolve_relative(contract.PIPELINE_CLAIM_PATH)
        ),
        "pipeline_ledger_absent": not os.path.lexists(
            resolve_relative(contract.PIPELINE_LEDGER_PATH)
        ),
    }


def freeze_protocol_and_lock() -> dict[str, Any]:
    """Publish both canonical Q3 locks exactly once after R8 terminal PASS."""

    if _q3_occupied():
        raise V12R8Q3FreezeError(f"Q3 freeze paths already occupied: {_q3_occupied()}")
    snapshot = live_r8_prerequisite_snapshot(require_q3_unopened=True)
    closure = source_closure()
    protocol = build_protocol_payload(prerequisite_snapshot=snapshot, closure=closure)
    protocol_gate = protocol_payload_gate(protocol)
    if protocol_gate.get("passed") is not True:
        raise V12R8Q3FreezeError("Q3 protocol pure gate failed")
    protocol_path = resolve_relative(contract.PROTOCOL_FREEZE_PATH)
    forensic.write_json_exclusive(protocol_path, protocol)
    protocol_record = _record(protocol_path)
    lock = build_execution_lock_payload(
        protocol_payload=protocol,
        protocol_record=protocol_record,
        prerequisite_snapshot=snapshot,
        closure=closure,
        occupancy=_occupancy(),
    )
    lock_path = resolve_relative(contract.EXECUTION_LOCK_PATH)
    forensic.write_json_exclusive(lock_path, lock)
    return {"protocol": verify_protocol_freeze(), "lock": verify_execution_lock()}


def verify_protocol_freeze() -> dict[str, Any]:
    payload = _mapping(contract.PROTOCOL_FREEZE_PATH)
    if protocol_payload_gate(payload).get("passed") is not True:
        raise V12R8Q3FreezeError("Q3 protocol freeze drifted")
    if payload.get("source_closure") != source_closure():
        raise V12R8Q3FreezeError("Q3 protocol source bytes drifted")
    return payload


def verify_execution_lock() -> dict[str, Any]:
    protocol = verify_protocol_freeze()
    lock = _mapping(contract.EXECUTION_LOCK_PATH)
    checks = {
        "status": lock.get("status") == EXECUTION_LOCK_PASS_STATUS
        and lock.get("passed") is True,
        "identity": lock.get("protocol_id") == contract.PROTOCOL_ID
        and lock.get("pipeline_id") == contract.PIPELINE_ID
        and lock.get("candidate_id") == protocol.get("candidate_id")
        and lock.get("candidate_module") == protocol.get("candidate_module"),
        "protocol": lock.get("protocol_freeze")
        == _record(contract.PROTOCOL_FREEZE_PATH),
        "sources": lock.get("source_closure") == source_closure(),
        "ordering": lock.get("rollout_matrix") == list(contract.ROLLOUT_MATRIX)
        and lock.get("baseline_first") is True,
        "stored_pristine": isinstance(lock.get("occupancy"), Mapping)
        and all(value is True for value in lock["occupancy"].values()),
        "zero_updates": all(
            type(lock.get(name)) is int and lock[name] == 0
            for name in ("actor_updates", "critic_updates", "ppo_updates")
        ),
        "closed": lock.get("retry_authorized") is False
        and lock.get("resume_authorized") is False
        and lock.get("runtime_promoted") is False,
    }
    if not all(checks.values()):
        failed = sorted(name for name, passed in checks.items() if passed is not True)
        raise V12R8Q3FreezeError(f"Q3 execution lock drifted: {failed}")
    prerequisites.bind_candidate(lock["candidate_id"], lock["candidate_module"])
    return lock


def deferred_preflight() -> dict[str, Any]:
    """Report eligibility without calling live verifiers or opening artifacts."""

    state = prerequisites.deferred_prerequisite_state()
    return {
        "status": "DEFERRED_H0_V12R8_Q3_PROTOCOL_FREEZE",
        "passed": False,
        "lineage_state": contract.LINEAGE_STATE,
        "r8_prerequisites": state,
        "source_closure": source_closure_readiness(),
        "q3_occupied": _q3_occupied(),
        "publication_performed": False,
    }


def live_preflight() -> dict[str, Any]:
    """Verify five live R8 receipts and the closed source set without writing."""

    occupied = _q3_occupied()
    if occupied:
        raise V12R8Q3FreezeError(f"Q3 namespace is already occupied: {occupied}")
    snapshot = live_r8_prerequisite_snapshot(require_q3_unopened=True)
    closure = source_closure()
    gate = snapshot["gate"]
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": "PASS_H0_V12R8_Q3_LIVE_FREEZE_PREFLIGHT",
        "passed": True,
        "candidate_id": gate["candidate_id"],
        "candidate_module": copy.deepcopy(gate["candidate_module"]),
        "official_verifier_count": gate["official_verifier_count"],
        "source_closure": closure,
        "q3_occupied": [],
        "publication_performed": False,
    }


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    action = parser.add_mutually_exclusive_group(required=True)
    action.add_argument("--preflight", action="store_true")
    action.add_argument("--live-preflight", action="store_true")
    action.add_argument("--freeze", action="store_true")
    action.add_argument("--verify", action="store_true")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    if args.preflight:
        result = deferred_preflight()
    elif args.live_preflight:
        result = live_preflight()
    elif args.freeze:
        result = freeze_protocol_and_lock()
    else:
        result = {"protocol": verify_protocol_freeze(), "lock": verify_execution_lock()}
    print(result.get("status", EXECUTION_LOCK_PASS_STATUS))
    if args.preflight:
        return 0
    if args.live_preflight:
        return 0 if result.get("passed") is True else 1
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())


__all__ = [
    "EXECUTION_LOCK_PASS_STATUS",
    "PROTOCOL_FREEZE_PASS_STATUS",
    "V12R8Q3FreezeError",
    "build_execution_lock_payload",
    "build_protocol_payload",
    "deferred_preflight",
    "freeze_protocol_and_lock",
    "live_r8_prerequisite_snapshot",
    "live_preflight",
    "official_r8_hooks",
    "protocol_payload_gate",
    "source_closure",
    "source_closure_readiness",
    "verify_execution_lock",
    "verify_protocol_freeze",
]
