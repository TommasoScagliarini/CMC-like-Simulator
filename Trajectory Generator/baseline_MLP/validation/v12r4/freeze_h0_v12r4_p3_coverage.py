"""Audit and exclusively publish the authorized V12R4 protocol freeze.

The freezer is intentionally independent from RLlib and OpenSim.  It binds the
immutable P2 collection-only source, both terminal predecessor lineages, the
complete R4 source closure, and the requirement that Q2 execution outputs stay
unopened.  Publication is no-clobber and performs no environment or fit work.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import os
import stat
import sys
from pathlib import Path, PurePosixPath
from typing import Any, Mapping, Sequence


def _discover_repo_root(source: Path) -> Path:
    for candidate in source.resolve().parents:
        if (
            (candidate / "AGENTS.md").is_file()
            and (candidate / "validation").is_dir()
            and (candidate / "Trajectory Generator").is_dir()
        ):
            return candidate
    raise RuntimeError("repository root could not be discovered")


REPO_ROOT = _discover_repo_root(Path(__file__))
REVISION_ROOT = Path(__file__).resolve().parent
for _root in (REVISION_ROOT,):
    if str(_root) not in sys.path:
        sys.path.insert(0, str(_root))

import h0_v12r4_p3_coverage_contract as contract  # noqa: E402


class V12R4ProtocolFreezeError(RuntimeError):
    """Raised when V12R4 source evidence is unsafe or inconsistent."""


def resolve_relative(path: str | os.PathLike[str] | PurePosixPath) -> Path:
    raw = path.as_posix() if isinstance(path, PurePosixPath) else os.fspath(path)
    pure = PurePosixPath(raw)
    if not raw or pure.is_absolute() or ".." in pure.parts or pure.as_posix() != raw:
        raise V12R4ProtocolFreezeError(
            f"non-canonical repository-relative path: {raw!r}"
        )
    return REPO_ROOT.joinpath(*pure.parts)


def _regular_file_no_links(path: Path) -> bool:
    try:
        info = path.lstat()
    except OSError:
        return False
    return stat.S_ISREG(info.st_mode) and not path.is_symlink()


def sha256_file(path: Path) -> str:
    if not _regular_file_no_links(path):
        raise V12R4ProtocolFreezeError(f"unsafe or missing regular file: {path}")
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def artifact_record(path: str | os.PathLike[str] | PurePosixPath) -> dict[str, Any]:
    resolved = resolve_relative(path)
    return {
        "path": resolved.relative_to(REPO_ROOT).as_posix(),
        "sha256": sha256_file(resolved),
        "size_bytes": resolved.stat().st_size,
    }


def tree_record(path: str | os.PathLike[str] | PurePosixPath) -> dict[str, Any]:
    root = resolve_relative(path)
    if not root.is_dir() or root.is_symlink():
        raise V12R4ProtocolFreezeError(f"unsafe or missing tree: {root}")
    files = sorted(item for item in root.rglob("*") if item.is_file())
    if not files or any(item.is_symlink() for item in files):
        raise V12R4ProtocolFreezeError(f"empty or linked artifact tree: {root}")
    digest = hashlib.sha256()
    rows: list[dict[str, Any]] = []
    for item in files:
        relative = item.relative_to(root).as_posix()
        sha256 = sha256_file(item)
        size_bytes = item.stat().st_size
        rows.append({"path": relative, "sha256": sha256, "size_bytes": size_bytes})
        digest.update(relative.encode("utf-8"))
        digest.update(b"\0")
        digest.update(sha256.encode("ascii"))
        digest.update(b"\0")
        digest.update(str(size_bytes).encode("ascii"))
        digest.update(b"\n")
    return {
        "path": root.relative_to(REPO_ROOT).as_posix(),
        "tree_sha256": digest.hexdigest(),
        "file_count": len(rows),
        "files": rows,
    }


def _load_mapping(path: str | os.PathLike[str] | PurePosixPath) -> dict[str, Any]:
    resolved = resolve_relative(path)
    try:
        value = json.loads(resolved.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise V12R4ProtocolFreezeError(f"cannot read strict JSON: {resolved}") from exc
    if not isinstance(value, dict):
        raise V12R4ProtocolFreezeError(f"JSON root is not an object: {resolved}")
    return value


def _records_exact() -> dict[str, bool]:
    return {
        "p2_corpus": artifact_record(contract.P2_CORPUS_ARTIFACT["path"])
        == contract.P2_CORPUS_ARTIFACT,
        "p2_report": artifact_record(contract.P2_ADAPTATION_REPORT_ARTIFACT["path"])
        == contract.P2_ADAPTATION_REPORT_ARTIFACT,
        "p2_history": artifact_record(contract.P2_ADAPTATION_HISTORY_ARTIFACT["path"])
        == contract.P2_ADAPTATION_HISTORY_ARTIFACT,
        "p2_module": tree_record(contract.P2_MODULE_TREE["path"])
        == contract.P2_MODULE_TREE,
        "v12r3_terminal": artifact_record(contract.V12R3_TERMINAL_LEDGER["path"])
        == contract.V12R3_TERMINAL_LEDGER,
        "p1s_terminal": artifact_record(contract.P1S_TERMINAL_LEDGER["path"])
        == contract.P1S_TERMINAL_LEDGER,
        "p1s_nominal_failure": artifact_record(contract.P1S_NOMINAL_FAILURE["path"])
        == contract.P1S_NOMINAL_FAILURE,
    }


def _terminal_semantics() -> dict[str, bool]:
    v12r3 = _load_mapping(contract.V12R3_TERMINAL_LEDGER["path"])
    p1s = _load_mapping(contract.P1S_TERMINAL_LEDGER["path"])
    failure = _load_mapping(contract.P1S_NOMINAL_FAILURE["path"])
    return {
        "v12r3_terminal_fit_p2": v12r3.get("status")
        == "FAIL_H0_PRIMARY_SPLIT_V12R3_PIPELINE_TERMINAL"
        and v12r3.get("passed") is False
        and v12r3.get("attempted_stage") == "fit_p2"
        and v12r3.get("retry_authorized") is False,
        "p1s_terminal_nominal": p1s.get("status")
        == "FAIL_H0_V12R3_P1_SALVAGE_DEVELOPMENT_TERMINAL"
        and p1s.get("terminal") is True
        and p1s.get("passed") is False
        and p1s.get("attempted_stage") == "development__deterministic_offset_nominal"
        and p1s.get("retry_authorized") is False
        and p1s.get("resume_authorized") is False,
        "failure_exact": failure.get("status") == "FAIL_H0_V12R3_P1_SALVAGE_STAGE"
        and failure.get("passed") is False
        and failure.get("last_completed_step") == 179
        and failure.get("end_reason")
        == "v12r3_p1_salvage_rollout_failed_terminal_no_retry"
        and "to_too_early_after_hs" in str(failure.get("error", {}).get("message", "")),
    }


def _p2_semantics() -> dict[str, bool]:
    report = _load_mapping(contract.P2_ADAPTATION_REPORT_ARTIFACT["path"])
    optimizer = report.get("optimizer_audit")
    preservation = report.get("preservation_audit")
    save_reload = report.get("save_reload")
    return {
        "p2_corpus_exact": report.get("corpus_exact") is True
        and report.get("training_samples") == contract.P2_CORPUS_ROWS
        and report.get("validation_samples") == 0,
        "p2_optimizer_audited": isinstance(optimizer, Mapping)
        and optimizer.get("sample_count") == contract.P2_CORPUS_ROWS
        and optimizer.get("adamw_epochs") == 3000
        and optimizer.get("lbfgs_max_iter") == 300
        and optimizer.get("lbfgs_max_eval") == 600
        and optimizer.get("fallback") is False
        and optimizer.get("sweep") is False,
        "p2_preserved": isinstance(preservation, Mapping)
        and preservation.get("passed") is True
        and preservation.get("all_candidate_tensors_finite") is True
        and preservation.get("disabled_clock_columns_bit_zero") is True
        and preservation.get("logstd_parameter_rows_bit_exact") is True
        and preservation.get("critic_present") is False,
        "p2_reload_exact": report.get("module_reload_exact") is True
        and isinstance(save_reload, Mapping)
        and save_reload.get("exact") is True,
        "p2_unreceipted_nonpromotable": not resolve_relative(
            contract.P2_ROOT / "receipt.json"
        ).exists()
        and not resolve_relative(contract.P2_ROOT / "summary.json").exists()
        and not resolve_relative(contract.P2_ROOT / "gate.json").exists(),
    }


def _q2_design_semantics() -> dict[str, bool]:
    path = resolve_relative(contract.Q2_DESIGN_FREEZE_PATH)
    if not _regular_file_no_links(path):
        return {"present_regular": False}
    payload = _load_mapping(contract.Q2_DESIGN_FREEZE_PATH)
    snapshot = payload.get("design_snapshot")
    access = payload.get("qualification_access")
    checks = payload.get("checks")
    holdout_cases = (
        snapshot.get("holdout_cases") if isinstance(snapshot, Mapping) else None
    )
    rollout_matrix = (
        snapshot.get("rollout_matrix") if isinstance(snapshot, Mapping) else None
    )
    return {
        "present_regular": True,
        "artifact_exact": artifact_record(contract.Q2_DESIGN_FREEZE_PATH)
        == contract.Q2_DESIGN_FREEZE_ARTIFACT,
        "identity": payload.get("schema_version") == 140
        and payload.get("status") == "PASS_H0_V12R4_Q2_QUALIFICATION_DESIGN_FREEZE"
        and payload.get("passed") is True
        and payload.get("freeze_kind")
        == "PRE_R4_DEFERRED_CANDIDATE_INDEPENDENT_Q2_DESIGN",
        "candidate_deferred": payload.get("candidate_binding_state") == "DEFERRED"
        and payload.get("candidate_id") is None
        and payload.get("candidate_module") is None
        and isinstance(snapshot, Mapping)
        and snapshot.get("candidate_binding_state") == "DEFERRED"
        and snapshot.get("candidate_id") is None
        and snapshot.get("candidate_module") is None,
        "matrix_exact_shape": isinstance(holdout_cases, list)
        and len(holdout_cases) == 6
        and isinstance(rollout_matrix, list)
        and len(rollout_matrix) == 12
        and snapshot.get("matrix_order") == "BASELINE_SIX_THEN_CANDIDATE_SIX",
        "design_gates": isinstance(payload.get("design_gate"), Mapping)
        and payload["design_gate"].get("passed") is True
        and isinstance(payload.get("source_gate"), Mapping)
        and payload["source_gate"].get("passed") is True
        and isinstance(payload.get("historical_exclusion_gate"), Mapping)
        and payload["historical_exclusion_gate"].get("passed") is True,
        "qualification_locked": isinstance(access, Mapping)
        and access.get("status")
        == "LOCKED_PENDING_R4_TERMINAL_PASS_AND_CANDIDATE_BINDING",
        "q2_outputs_unopened_at_design": isinstance(checks, Mapping)
        and checks.get("exact_deferred_design_preregistered") is True
        and all(
            checks.get(name) is True
            for name in (
                "q2_protocol_freeze_absent",
                "q2_execution_lock_absent",
                "q2_noise_tapes_absent",
                "q2_run_root_absent",
            )
        ),
    }


def _output_absence() -> dict[str, bool]:
    paths = {
        "protocol_freeze": contract.PROTOCOL_FREEZE_PATH,
        "design_audit": contract.DESIGN_AUDIT_PATH,
        "execution_lock": contract.EXECUTION_LOCK_PATH,
        "run_root": contract.RUN_ROOT,
        "candidate_freeze": contract.CANDIDATE_FREEZE_PATH,
        **{f"q2_{name}": path for name, path in contract.Q2_UNOPENED_PATHS.items()},
    }
    return {
        name: not os.path.lexists(resolve_relative(path))
        for name, path in paths.items()
    }


def source_precondition_gate() -> dict[str, Any]:
    """Recompute every immutable input and unopened-output prerequisite."""

    self_check = contract.contract_self_check()
    records = _records_exact()
    terminal = _terminal_semantics()
    p2 = _p2_semantics()
    q2_design = _q2_design_semantics()
    absence = _output_absence()
    checks = {
        "contract": self_check.get("passed") is True,
        "records": all(records.values()),
        "terminal_lineages": all(terminal.values()),
        "p2_collection_source": all(p2.values()),
        "q2_deferred_design": all(q2_design.values()),
        "outputs_unopened": all(absence.values()),
        "exact_one_shot_authority": all(
            contract.AUTHORITY[name]
            for name in (
                "source_implementation_authorized",
                "protocol_freeze_publication_authorized",
                "execution_lock_authorized",
                "collection_execution_authorized",
                "actor_fit_execution_authorized",
                "candidate_freeze_authorized",
                "development_execution_authorized",
            )
        )
        and not any(
            contract.AUTHORITY[name]
            for name in (
                "qualification_execution_authorized",
                "retry_authorized",
                "resume_authorized",
                "rescue_authorized",
                "sweep_authorized",
                "runtime_promotion_authorized",
                "checkpoint_zero_authorized",
                "positive_morphology_authorized",
            )
        ),
    }
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": (
            "PASS_H0_V12R4_SOURCE_PRECONDITIONS"
            if all(checks.values())
            else "FAIL_H0_V12R4_SOURCE_PRECONDITIONS"
        ),
        "passed": all(checks.values()),
        "protocol_id": contract.PROTOCOL_ID,
        "checks": checks,
        "record_checks": records,
        "terminal_checks": terminal,
        "p2_checks": p2,
        "q2_design_checks": q2_design,
        "q2_design_freeze": (
            artifact_record(contract.Q2_DESIGN_FREEZE_PATH)
            if all(q2_design.values())
            else None
        ),
        "absence_checks": absence,
        "q2_unopened": all(
            absence[f"q2_{name}"] for name in contract.Q2_UNOPENED_PATHS
        ),
    }


def _source_records() -> dict[str, dict[str, Any]]:
    expected = (
        ".gitattributes",
        "__init__.py",
        "conftest.py",
        "h0_v12r4_p3_coverage_contract.py",
        "freeze_h0_v12r4_p3_coverage.py",
        "h0_v12r4_p3_coverage_fitter.py",
        "run_h0_v12r4_p3_coverage.py",
        "test_h0_v12r4_p3_coverage_contract.py",
        "test_freeze_h0_v12r4_p3_coverage.py",
        "test_h0_v12r4_p3_coverage_fitter.py",
        "test_run_h0_v12r4_p3_coverage.py",
    )
    records: dict[str, dict[str, Any]] = {}
    for name in expected:
        relative = contract.VALIDATION_ROOT / name
        records[name] = artifact_record(relative)
    return records


def build_protocol_freeze_payload() -> dict[str, Any]:
    """Build the canonical pre-execution freeze payload in memory."""

    gate = source_precondition_gate()
    if gate.get("passed") is not True:
        failed = [name for name, value in gate["checks"].items() if not value]
        raise V12R4ProtocolFreezeError(f"V12R4 source preconditions failed: {failed}")
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.PROTOCOL_FREEZE_STATUS,
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "revision": contract.REVISION,
        "authority": dict(contract.AUTHORITY),
        "source_precondition_gate": gate,
        "source_records": _source_records(),
        "p2_collection_source": {
            "role": "COLLECTION_STUDENT_ONLY_NONPROMOTABLE",
            "corpus": dict(contract.P2_CORPUS_ARTIFACT),
            "module": dict(contract.P2_MODULE_TREE),
            "adaptation_report": dict(contract.P2_ADAPTATION_REPORT_ARTIFACT),
            "adaptation_history": dict(contract.P2_ADAPTATION_HISTORY_ARTIFACT),
        },
        "terminal_evidence": {
            "v12r3": dict(contract.V12R3_TERMINAL_LEDGER),
            "p1s": dict(contract.P1S_TERMINAL_LEDGER),
            "nominal_failure": dict(contract.P1S_NOMINAL_FAILURE),
        },
        "stage_order": list(contract.STAGE_IDS),
        "collection_cases": [
            contract.canonical_collection_case(case_id)
            for case_id in contract.COLLECTION_CASE_IDS
        ],
        "corpus_counts": contract.expected_corpus_counts(),
        "fit": dict(contract.FIT),
        "candidate_selection": {
            "rule": contract.CANDIDATE_SELECTION_RULE,
            "module_path": contract.P3_MODULE_PATH.as_posix(),
            "candidate_id": "DEFERRED_UNTIL_FIT_P3",
            "candidate_tree_sha256": "DEFERRED_UNTIL_FIT_P3",
        },
        "development_cases": [
            contract.canonical_development_case(case_id)
            for case_id in contract.DEVELOPMENT_CASE_IDS
        ],
        "q2_unopened_paths": {
            name: path.as_posix() for name, path in contract.Q2_UNOPENED_PATHS.items()
        },
        "q2_design_freeze_allowed_present": contract.Q2_DESIGN_FREEZE_PATH.as_posix(),
        "q2_design_freeze": gate["q2_design_freeze"],
        "q2_prerequisites_produced_by_r4": copy.deepcopy(contract.Q2_PREREQUISITES),
        "execution_lock_created": False,
        "pipeline_claim_created": False,
        "environment_reset_calls": 0,
        "environment_step_calls": 0,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
        "runtime_promoted": False,
        "checkpoint_zero_created": False,
        "positive_morphology_enabled": False,
    }


def _write_json_exclusive(path: Path, payload: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    encoded = (
        json.dumps(payload, indent=2, sort_keys=True, allow_nan=False) + "\n"
    ).encode("utf-8")
    descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o644)
    try:
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(encoded)
            stream.flush()
            os.fsync(stream.fileno())
    except BaseException:
        try:
            os.unlink(path)
        except OSError:
            pass
        raise


def verify_protocol_freeze() -> dict[str, Any]:
    payload = _load_mapping(contract.PROTOCOL_FREEZE_PATH)
    q2_semantics = _q2_design_semantics()
    q2_record = (
        artifact_record(contract.Q2_DESIGN_FREEZE_PATH)
        if all(q2_semantics.values())
        else None
    )
    checks = {
        "status": payload.get("status") == contract.PROTOCOL_FREEZE_STATUS,
        "passed": payload.get("passed") is True,
        "identity": payload.get("protocol_id") == contract.PROTOCOL_ID
        and payload.get("pipeline_id") == contract.PIPELINE_ID
        and payload.get("revision") == contract.REVISION,
        "authority": payload.get("authority") == contract.AUTHORITY,
        "stage_order": payload.get("stage_order") == list(contract.STAGE_IDS),
        "fit": payload.get("fit") == contract.FIT,
        "counts": payload.get("corpus_counts") == contract.expected_corpus_counts(),
        "source_closure": payload.get("source_records") == _source_records(),
        "p2_inputs": all(_records_exact().values()),
        "terminal_evidence": all(_terminal_semantics().values()),
        "q2_design_semantics": all(q2_semantics.values()),
        "q2_design_record": payload.get("q2_design_freeze") == q2_record,
        "q2_paths": payload.get("q2_unopened_paths")
        == {name: path.as_posix() for name, path in contract.Q2_UNOPENED_PATHS.items()},
    }
    if not all(checks.values()):
        failed = [name for name, passed in checks.items() if not passed]
        raise V12R4ProtocolFreezeError(f"protocol freeze drifted: {failed}")
    return payload


def publish_protocol_freeze() -> Path:
    """Publish the exact freeze and an additive source-design audit."""

    if not contract.AUTHORITY["protocol_freeze_publication_authorized"]:
        raise V12R4ProtocolFreezeError("V12R4 protocol publication is unauthorized")
    freeze_path = resolve_relative(contract.PROTOCOL_FREEZE_PATH)
    audit_path = resolve_relative(contract.DESIGN_AUDIT_PATH)
    if os.path.lexists(freeze_path) or os.path.lexists(audit_path):
        raise V12R4ProtocolFreezeError("protocol freeze/design audit exists")
    payload = build_protocol_freeze_payload()
    _write_json_exclusive(freeze_path, payload)
    audit = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": "PASS_H0_V12R4_P3_COVERAGE_DESIGN_AUDIT",
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "protocol_freeze": artifact_record(contract.PROTOCOL_FREEZE_PATH),
        "contract_self_check": contract.contract_self_check(),
        "source_precondition_gate": payload["source_precondition_gate"],
        "environment_reset_calls": 0,
        "environment_step_calls": 0,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
    }
    _write_json_exclusive(audit_path, audit)
    verify_protocol_freeze()
    return freeze_path


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="audit without writes")
    parser.add_argument(
        "--publish", action="store_true", help="publish once with no clobber"
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    if args.publish:
        path = publish_protocol_freeze()
        print(path.relative_to(REPO_ROOT).as_posix())
        return 0
    if resolve_relative(contract.PROTOCOL_FREEZE_PATH).exists():
        gate = {"passed": True, "freeze": verify_protocol_freeze()}
    else:
        gate = source_precondition_gate()
    print(json.dumps(gate, indent=2, sort_keys=True, allow_nan=False))
    return 0 if gate["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = [
    "V12R4ProtocolFreezeError",
    "artifact_record",
    "build_protocol_freeze_payload",
    "main",
    "publish_protocol_freeze",
    "resolve_relative",
    "source_precondition_gate",
    "tree_record",
    "verify_protocol_freeze",
]
