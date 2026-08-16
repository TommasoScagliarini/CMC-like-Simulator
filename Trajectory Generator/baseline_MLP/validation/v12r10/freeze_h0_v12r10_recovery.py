"""Freeze and verify the import-only V12R10 recovery protocol.

Importing this module is read-only.  ``prepare()`` is the only publication
surface.  The freezer never imports diagnostic Python modules: their sources
and strict-JSON results are immutable evidence read and attested as data.
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
from typing import Any, Callable, Mapping, Sequence


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
LOCAL_VALIDATION = REVISION_ROOT.parent
for _root in (
    REPO_ROOT,
    REPO_ROOT / "validation",
    REPO_ROOT / "Trajectory Generator" / "baseline_MLP",
    LOCAL_VALIDATION,
    LOCAL_VALIDATION / "v12r3",
    LOCAL_VALIDATION / "v12r6",
    LOCAL_VALIDATION / "v12r7",
    LOCAL_VALIDATION / "v12r8",
    LOCAL_VALIDATION / "v12r9",
    REVISION_ROOT,
):
    if str(_root) not in sys.path:
        sys.path.insert(0, str(_root))

import freeze_h0_v12r9_recovery as r9_freezer  # noqa: E402
import h0_v12r10_recovery_contract as contract  # noqa: E402


class V12R10FreezeError(RuntimeError):
    """Raised when a V12R10 freeze cannot remain exact and fail-closed."""


LOCAL_PRODUCTION_SOURCES = (
    "Trajectory Generator/baseline_MLP/validation/v12r10/__init__.py",
    "Trajectory Generator/baseline_MLP/validation/v12r10/"
    "h0_v12r10_recovery_contract.py",
    "Trajectory Generator/baseline_MLP/validation/v12r10/h0_v12r10_recovery_fitter.py",
    "Trajectory Generator/baseline_MLP/validation/v12r10/freeze_h0_v12r10_recovery.py",
    "Trajectory Generator/baseline_MLP/validation/v12r10/run_h0_v12r10_recovery.py",
)
EXPECTED_RLMODULE_FILES = {
    "actor_feature_manifest.json",
    "candidate_build_manifest.json",
    "class_and_ctor_args.pkl",
    "metadata.json",
    "module_state.pkl",
}

RuntimeAttestor = Callable[[], Mapping[str, Any]]


def resolve_relative(path: str | os.PathLike[str] | PurePosixPath) -> Path:
    """Resolve a canonical repository-relative POSIX path without following it."""

    raw = path.as_posix() if isinstance(path, PurePosixPath) else os.fspath(path)
    pure = PurePosixPath(raw)
    if (
        not raw
        or "\\" in raw
        or pure.is_absolute()
        or ".." in pure.parts
        or any(":" in part for part in pure.parts)
        or pure.as_posix() != raw
    ):
        raise V12R10FreezeError(f"non-canonical repository path: {raw!r}")
    return REPO_ROOT.joinpath(*pure.parts)


def _is_link_or_reparse(path: Path) -> bool:
    try:
        status = os.lstat(path)
    except OSError:
        return False
    attributes = int(getattr(status, "st_file_attributes", 0) or 0)
    reparse = int(getattr(stat, "FILE_ATTRIBUTE_REPARSE_POINT", 0x400))
    return stat.S_ISLNK(status.st_mode) or bool(attributes & reparse)


def _reject_link_ancestors(path: Path, *, include_leaf: bool) -> None:
    try:
        parts = path.relative_to(REPO_ROOT).parts
    except ValueError as exc:
        raise V12R10FreezeError(f"path escaped repository: {path}") from exc
    limit = len(parts) if include_leaf else max(0, len(parts) - 1)
    current = REPO_ROOT
    for part in parts[:limit]:
        current /= part
        if os.path.lexists(current) and _is_link_or_reparse(current):
            raise V12R10FreezeError(f"unsafe symlink/junction component: {current}")


def _regular_file(path: Path) -> bool:
    try:
        status = os.lstat(path)
    except OSError:
        return False
    attributes = int(getattr(status, "st_file_attributes", 0) or 0)
    reparse = int(getattr(stat, "FILE_ATTRIBUTE_REPARSE_POINT", 0x400))
    return stat.S_ISREG(status.st_mode) and not bool(attributes & reparse)


def safe_repository_path(
    path: str | os.PathLike[str] | PurePosixPath,
    *,
    include_leaf: bool = True,
) -> Path:
    if type(include_leaf) is not bool:
        raise V12R10FreezeError("include_leaf must be strict bool")
    target = resolve_relative(path)
    _reject_link_ancestors(target, include_leaf=include_leaf)
    return target


def artifact_record(path: str | os.PathLike[str] | PurePosixPath) -> dict[str, Any]:
    target = safe_repository_path(path)
    if not _regular_file(target) or _is_link_or_reparse(target):
        raise V12R10FreezeError(f"unsafe or missing artifact: {target}")
    data = target.read_bytes()
    return {
        "path": target.relative_to(REPO_ROOT).as_posix(),
        "sha256": hashlib.sha256(data).hexdigest(),
        "size_bytes": len(data),
    }


def tree_record(path: str | os.PathLike[str] | PurePosixPath) -> dict[str, Any]:
    root = safe_repository_path(path)
    try:
        root_status = os.lstat(root)
    except OSError as exc:
        raise V12R10FreezeError(f"unsafe or missing tree: {root}") from exc
    if not stat.S_ISDIR(root_status.st_mode) or _is_link_or_reparse(root):
        raise V12R10FreezeError(f"unsafe or missing tree: {root}")
    files: list[Path] = []
    for current_text, directory_names, file_names in os.walk(
        root, topdown=True, followlinks=False
    ):
        current = Path(current_text)
        directory_names.sort()
        file_names.sort()
        for name in directory_names:
            child = current / name
            try:
                child_status = os.lstat(child)
            except OSError as exc:
                raise V12R10FreezeError(f"unsafe tree entry: {child}") from exc
            if not stat.S_ISDIR(child_status.st_mode) or _is_link_or_reparse(child):
                raise V12R10FreezeError(f"unsafe tree directory: {child}")
        for name in file_names:
            child = current / name
            if not _regular_file(child) or _is_link_or_reparse(child):
                raise V12R10FreezeError(f"unsafe tree file: {child}")
            files.append(child)
    if not files:
        raise V12R10FreezeError(f"empty artifact tree: {root}")
    files.sort(key=lambda item: item.relative_to(root).as_posix())
    digest = hashlib.sha256()
    rows: list[dict[str, Any]] = []
    for item in files:
        relative = item.relative_to(root).as_posix()
        data = item.read_bytes()
        sha256 = hashlib.sha256(data).hexdigest()
        size_bytes = len(data)
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


def _reject_duplicate_keys(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"duplicate JSON key: {key!r}")
        result[key] = value
    return result


def _reject_constant(token: str) -> Any:
    raise ValueError(f"non-finite JSON constant: {token}")


def _mapping(path: str | os.PathLike[str] | PurePosixPath) -> dict[str, Any]:
    record = artifact_record(path)
    target = resolve_relative(record["path"])
    try:
        value = json.loads(
            target.read_text(encoding="utf-8"),
            object_pairs_hook=_reject_duplicate_keys,
            parse_constant=_reject_constant,
        )
    except Exception as exc:
        raise V12R10FreezeError(f"invalid strict JSON mapping: {target}") from exc
    if not isinstance(value, dict):
        raise V12R10FreezeError(f"expected JSON mapping: {target}")
    return value


def _strict_equal(left: Any, right: Any) -> bool:
    if type(left) is not type(right):
        return False
    if isinstance(left, Mapping):
        return set(left) == set(right) and all(
            _strict_equal(left[key], right[key]) for key in left
        )
    if isinstance(left, (list, tuple)):
        return len(left) == len(right) and all(
            _strict_equal(a, b) for a, b in zip(left, right, strict=True)
        )
    return bool(left == right)


def _canonical_digest(value: Any) -> str:
    try:
        data = json.dumps(
            value,
            sort_keys=True,
            separators=(",", ":"),
            ensure_ascii=False,
            allow_nan=False,
        ).encode("utf-8")
    except (TypeError, ValueError) as exc:
        raise V12R10FreezeError("value is not canonical-JSON encodable") from exc
    return hashlib.sha256(data).hexdigest()


def _expected_artifact(record: Mapping[str, Any]) -> dict[str, Any]:
    return {
        "path": record["path"],
        "sha256": record["sha256"],
        "size_bytes": record["size_bytes"],
    }


def _assert_artifact(name: str, record: Mapping[str, Any]) -> dict[str, Any]:
    expected = _expected_artifact(record)
    observed = artifact_record(str(expected["path"]))
    if not _strict_equal(observed, expected):
        raise V12R10FreezeError(f"locked artifact drifted: {name}")
    return observed


def production_source_paths() -> tuple[str, ...]:
    inherited = tuple(r9_freezer.production_source_paths())
    return tuple(dict.fromkeys((*inherited, *LOCAL_PRODUCTION_SOURCES)))


def production_source_closure() -> dict[str, dict[str, Any]]:
    return {path: artifact_record(path) for path in production_source_paths()}


def _attest_r9_corpus() -> dict[str, Any]:
    import numpy as np

    expected_keys = {
        "observations",
        "actions",
        "reset_mask",
        "actor_feature_names",
        "case_ids",
        "step_indices",
        "tranche_ids",
        "origins",
        "episode_ids",
        "raw_sample_weights",
        "normalized_sample_weights",
        "training_indices",
        "stratum_ids",
    }
    path = resolve_relative(contract.R9_CORPUS_PATH)
    try:
        with np.load(path, allow_pickle=False) as archive:
            if set(archive.files) != expected_keys:
                raise V12R10FreezeError("R9 corpus key set drifted")
            arrays = {
                name: np.ascontiguousarray(archive[name]) for name in archive.files
            }
    except V12R10FreezeError:
        raise
    except Exception as exc:
        raise V12R10FreezeError("R9 corpus is unreadable") from exc
    rows = contract.R9_TERMINAL_EVIDENCE["corpus"]["rows"]
    checks = {
        "observation_shape_dtype": arrays["observations"].shape
        == (rows, contract.EXPECTED_ACTOR_FEATURES)
        and arrays["observations"].dtype == np.dtype(np.float32),
        "action_shape_dtype": arrays["actions"].shape
        == (rows, contract.EXPECTED_ACTION_DIM)
        and arrays["actions"].dtype == np.dtype(np.float32),
        "row_vectors": all(
            arrays[name].shape == (rows,)
            for name in expected_keys
            - {"observations", "actions", "actor_feature_names"}
        ),
        "feature_names": arrays["actor_feature_names"].shape
        == (contract.EXPECTED_ACTOR_FEATURES,),
        "finite": bool(np.all(np.isfinite(arrays["observations"])))
        and bool(np.all(np.isfinite(arrays["actions"])))
        and bool(np.all(np.isfinite(arrays["raw_sample_weights"])))
        and bool(np.all(np.isfinite(arrays["normalized_sample_weights"]))),
        "training_indices_exact": arrays["training_indices"].dtype == np.dtype(np.int64)
        and bool(np.array_equal(arrays["training_indices"], np.arange(rows))),
        "reset_rows_exact": arrays["reset_mask"].dtype == np.dtype(np.bool_)
        and int(np.count_nonzero(arrays["reset_mask"])) == 26,
        "strata_exact": len(set(arrays["stratum_ids"].astype(str))) == 13,
    }
    if not all(checks.values()):
        failed = sorted(name for name, value in checks.items() if not value)
        raise V12R10FreezeError(f"R9 corpus semantic audit failed: {failed}")
    return {"passed": True, "rows": rows, "checks": checks}


def _attest_imported_labels() -> dict[str, Any]:
    import numpy as np

    expected_keys = {
        "observations",
        "actions",
        "reset_mask",
        "previous_penetration_m",
        "coverage_distance_rms_z",
        "coverage_nearest_reference_index",
        "coverage_ood_mask",
        "raw_sample_weights",
        "normalized_sample_weights",
        "actor_feature_names",
        "case_ids",
        "step_indices",
        "tranche_ids",
        "origins",
    }
    records: dict[str, Any] = {}
    row_counts: dict[str, int] = {}
    for case_id, expected in contract.IMPORTED_LABEL_RECORDS.items():
        records[case_id] = _assert_artifact(f"observer_label::{case_id}", expected)
        path = resolve_relative(expected["path"])
        try:
            with np.load(path, allow_pickle=False) as archive:
                if set(archive.files) != expected_keys:
                    raise V12R10FreezeError(
                        f"observer label key set drifted: {case_id}"
                    )
                arrays = {
                    name: np.ascontiguousarray(archive[name]) for name in archive.files
                }
        except V12R10FreezeError:
            raise
        except Exception as exc:
            raise V12R10FreezeError(f"observer labels unreadable: {case_id}") from exc
        rows = int(expected["rows"])
        vector_keys = expected_keys - {
            "observations",
            "actions",
            "actor_feature_names",
        }
        checks = {
            "observation_shape_dtype": arrays["observations"].shape
            == (rows, contract.EXPECTED_ACTOR_FEATURES)
            and arrays["observations"].dtype == np.dtype(np.float32),
            "action_shape_dtype": arrays["actions"].shape
            == (rows, contract.EXPECTED_ACTION_DIM)
            and arrays["actions"].dtype == np.dtype(np.float32),
            "row_vectors": all(arrays[name].shape == (rows,) for name in vector_keys),
            "feature_names": arrays["actor_feature_names"].shape
            == (contract.EXPECTED_ACTOR_FEATURES,),
            "case_exact": bool(np.all(arrays["case_ids"].astype(str) == case_id)),
            "reset_dtype": arrays["reset_mask"].dtype == np.dtype(np.bool_),
            "finite": all(
                bool(np.all(np.isfinite(array)))
                for array in arrays.values()
                if array.dtype.kind in "fiu"
            ),
        }
        if not all(checks.values()):
            raise V12R10FreezeError(f"observer label row closure drifted: {case_id}")
        row_counts[case_id] = rows
    if (
        set(records) != set(contract.COLLECTION_CASE_IDS)
        or sum(row_counts.values()) != 2_431
    ):
        raise V12R10FreezeError("six-case observer label closure drifted")
    return {
        "passed": True,
        "records": records,
        "row_counts": row_counts,
        "total_rows": sum(row_counts.values()),
        "environment_reset_calls": 0,
        "environment_step_calls": 0,
        "teacher_query_count": 0,
    }


def _attest_r9_terminal() -> dict[str, Any]:
    records = {
        name: _assert_artifact(f"r9::{name}", expected)
        for name, expected in contract.R9_TERMINAL_EVIDENCE.items()
    }
    ledger = _mapping(contract.R9_TERMINAL_LEDGER_PATH)
    gate = _mapping(contract.R9_FIT_GATE_PATH)
    summary = _mapping(contract.R9_FIT_SUMMARY_PATH)
    manifest = _mapping(contract.R9_CORPUS_MANIFEST_PATH)
    expected_tree = {
        key: contract.R9_TERMINAL_CANDIDATE_TREE[key]
        for key in ("path", "tree_sha256", "file_count", "files")
    }
    observed_tree = tree_record(contract.R9_CANDIDATE_MODULE_PATH)
    checks = {
        "terminal_fail": ledger.get("status") == contract.R9_TERMINAL_FAIL_STATUS
        and ledger.get("passed") is False
        and ledger.get("terminal") is True
        and ledger.get("attempted_stage") == contract.R9_TERMINAL_STAGE
        and ledger.get("next_stage") == "STOP_TERMINAL_NO_RETRY",
        "fit_failed": gate.get("passed") is False
        and gate.get("status") == "FAIL_H0_V12R9_RECOVERY_FIT",
        "candidate_tree_exact": _strict_equal(observed_tree, expected_tree)
        and set(row["path"] for row in observed_tree["files"])
        == EXPECTED_RLMODULE_FILES
        and _strict_equal(summary.get("candidate_module"), expected_tree)
        and _strict_equal(ledger.get("candidate_module"), expected_tree),
        "candidate_not_promoted": ledger.get("candidate_freeze") is None
        and ledger.get("runtime_promoted") is False
        and ledger.get("qualification_executed") is False
        and ledger.get("q3_paths_opened") == []
        and ledger.get("development_count") == 0
        and not os.path.lexists(resolve_relative(contract.v12r9.FIT_RECEIPT_PATH))
        and not os.path.lexists(resolve_relative(contract.v12r9.CANDIDATE_FREEZE_PATH))
        and not os.path.lexists(
            resolve_relative(contract.v12r9.FINAL_DEVELOPMENT_PATH)
        ),
        "candidate_initialization_only": contract.R9_TERMINAL_CANDIDATE_TREE["role"]
        == "INITIALIZATION_ONLY_NOT_PROMOTED"
        and contract.AUTHORITY["r9_candidate_promotion_authorized"] is False,
        "fit_identity": summary.get("hidden_dims") == [512, 512]
        and summary.get("actor_fit_count") == 1
        and summary.get("actor_updates") == 1
        and summary.get("critic_updates") == 0
        and summary.get("ppo_updates") == 0,
        "corpus_manifest": manifest.get("passed") is True
        and manifest.get("status") == "H0_V12R9_RECOVERY_CORPUS_PASS"
        and manifest.get("rows") == 11_875
        and _strict_equal(records["corpus"], manifest.get("corpus")),
        "terminal_no_retry": ledger.get("retry_authorized") is False
        and ledger.get("resume_authorized") is False,
    }
    if not all(checks.values()):
        failed = sorted(name for name, value in checks.items() if not value)
        raise V12R10FreezeError(f"terminal R9 audit failed: {failed}")
    return {
        "passed": True,
        "records": records,
        "candidate_tree": observed_tree,
        "checks": checks,
        "corpus": _attest_r9_corpus(),
    }


def _attest_diagnostics() -> dict[str, Any]:
    records = {
        name: _assert_artifact(f"diagnostic::{name}", expected)
        for name, expected in contract.LOCKED_DIAGNOSTIC_EVIDENCE.items()
    }
    uniform = _mapping(records["uniform_result"]["path"])
    gate = _mapping(records["gate_aligned_result"]["path"])
    observability = _mapping(records["observability_result"]["path"])

    uniform_primary = uniform.get("primary", {})
    uniform_checks = {
        "terminal_fail_preserved": uniform.get("passed") is False
        and uniform.get("status")
        == "COMPLETE_H0_V12R10_W1024_R6_RESIDUAL_RESET3_DRY_FIT",
        "state": uniform_primary.get("candidate_state_digest")
        == contract.EXPECTED_UNIFORM_STATE_DIGEST,
        "prediction": uniform_primary.get("candidate_predictions_sha256")
        == contract.EXPECTED_UNIFORM_PREDICTION_DIGEST,
        "closure": uniform_primary.get("optimizer", {}).get("lbfgs_closure_calls")
        == contract.EXPECTED_UNIFORM_LBFGS_CLOSURES,
        "loss": uniform_primary.get("history", [{}])[-1].get("loss")
        == contract.EXPECTED_UNIFORM_TERMINAL_LOSS,
        "no_publication": uniform.get("execution_accounting", {}).get(
            "production_candidate_created"
        )
        is False
        and uniform.get("execution_accounting", {}).get("production_candidate_promoted")
        is False,
    }

    primary = gate.get("primary", {})
    replica = gate.get("determinism_replica", {})
    compatibility = primary.get("runtime_save_reload_warm_start", {})
    reproduction = primary.get("uniform_terminal_reproduction", {})
    frozen_inputs = gate.get("frozen_inputs", {})
    gate_checks = {
        "result_pass": gate.get("passed") is True
        and gate.get("status") == contract.DIAGNOSTIC_ATTESTATION["status"]
        and gate.get("decision") == contract.DIAGNOSTIC_ATTESTATION["decision"],
        "source_bound": gate.get("diagnostic_source")
        == {
            "path": records["gate_aligned_source"]["path"],
            "sha256": records["gate_aligned_source"]["sha256"],
        },
        "uniform_result_bound": gate.get("frozen_uniform_result", {}).get("path")
        == records["uniform_result"]["path"]
        and gate.get("frozen_uniform_result", {}).get("sha256")
        == records["uniform_result"]["sha256"],
        "r9_inputs_bound": frozen_inputs.get("r9_corpus_sha256")
        == contract.R9_TERMINAL_EVIDENCE["corpus"]["sha256"],
        "r9_terminal_bound": frozen_inputs.get("r9_ledger_sha256")
        == contract.R9_TERMINAL_EVIDENCE["terminal_ledger"]["sha256"]
        and frozen_inputs.get("r9_candidate_tree_sha256")
        == contract.R9_TERMINAL_CANDIDATE_TREE["tree_sha256"],
        "primary_identity": primary.get("passed") is True
        and primary.get("candidate_state_digest")
        == contract.EXPECTED_FINAL_STATE_DIGEST
        and primary.get("candidate_predictions_sha256")
        == contract.EXPECTED_FINAL_PREDICTION_DIGEST,
        "uniform_reproduction": reproduction.get("passed") is True
        and reproduction.get("candidate_state_digest")
        == contract.EXPECTED_UNIFORM_STATE_DIGEST
        and reproduction.get("candidate_predictions_sha256")
        == contract.EXPECTED_UNIFORM_PREDICTION_DIGEST
        and reproduction.get("lbfgs_closure_calls")
        == contract.EXPECTED_UNIFORM_LBFGS_CLOSURES
        and reproduction.get("terminal_loss")
        == contract.EXPECTED_UNIFORM_TERMINAL_LOSS,
        "terminal_schedule": primary.get("optimizer", {}).get("lbfgs_closure_calls")
        == contract.EXPECTED_GATE_LBFGS_CLOSURES
        and primary.get("history", [{}])[-1].get("loss")
        == contract.EXPECTED_GATE_TERMINAL_LOSS,
        "metrics_digest": _canonical_digest(primary.get("metrics"))
        == contract.EXPECTED_METRICS_DIGEST,
        "history_digest": _canonical_digest(primary.get("history"))
        == contract.EXPECTED_HISTORY_DIGEST,
        "gate_digest": _canonical_digest(primary.get("w1024_gate"))
        == contract.EXPECTED_GATE_DIGEST,
        "compatibility_digest": _canonical_digest(compatibility)
        == contract.EXPECTED_COMPATIBILITY_DIGEST
        and compatibility.get("passed") is True
        and compatibility.get("runtime_logits_sha256")
        == contract.EXPECTED_RUNTIME_LOGITS_DIGEST,
        "replica_digest": _canonical_digest(replica) == contract.EXPECTED_REPLICA_DIGEST
        and replica.get("passed") is True
        and replica.get("state_byte_exact") is True
        and replica.get("predictions_byte_exact") is True
        and replica.get("metrics_exact") is True
        and replica.get("history_exact") is True
        and replica.get("candidate_state_digest")
        == contract.EXPECTED_FINAL_STATE_DIGEST
        and replica.get("candidate_predictions_sha256")
        == contract.EXPECTED_FINAL_PREDICTION_DIGEST,
        "no_environment_teacher_publication": all(
            gate.get("execution_accounting", {}).get(name) == 0
            for name in (
                "offline_h0_teacher_queries_total",
                "environment_reset_calls",
                "environment_step_calls",
                "persistent_model_state_writes",
                "persistent_checkpoint_writes",
                "publication_writes",
                "ppo_updates",
                "critic_updates",
            )
        )
        and gate.get("execution_accounting", {}).get("production_candidate_created")
        is False
        and gate.get("execution_accounting", {}).get("production_candidate_promoted")
        is False,
    }

    observability_checks = {
        "audit_pass": observability.get("passed") is True
        and observability.get("status")
        == "COMPLETE_H0_V12R10_TRANSITION_OBSERVABILITY_AUDIT",
        "risk_not_training_ready": observability.get("training_ready") is False
        and observability.get("current_label_contract_decision")
        == "BLOCK_LEGACY_SHADOW_PROJECTED_MARKOV35",
        "risk_frozen_as_limitation": contract.LIMITATIONS[
            "legacy_shadow_label_transition_alias"
        ]
        is True
        and contract.LIMITATIONS["offline_fit_proves_training_readiness"] is False,
        "no_runtime_shadow": contract.LIMITATIONS[
            "runtime_legacy_shadow_dependency_authorized"
        ]
        is False,
        "source_bound": observability.get("diagnostic_source")
        == records["observability_source"],
    }
    for label, checks in (
        ("uniform diagnostic", uniform_checks),
        ("gate-aligned diagnostic", gate_checks),
        ("observability diagnostic", observability_checks),
    ):
        if not all(checks.values()):
            failed = sorted(name for name, value in checks.items() if not value)
            raise V12R10FreezeError(f"{label} audit failed: {failed}")
    return {
        "passed": True,
        "records": records,
        "uniform_checks": uniform_checks,
        "gate_aligned_checks": gate_checks,
        "observability_checks": observability_checks,
        "attestation": copy.deepcopy(contract.DIAGNOSTIC_ATTESTATION),
    }


def attest_locked_inputs() -> dict[str, Any]:
    """Recompute all immutable import-only inputs without executing a fit."""

    r9 = _attest_r9_terminal()
    labels = _attest_imported_labels()
    diagnostics = _attest_diagnostics()
    r6 = tree_record(contract.R6_CANDIDATE_MODULE_PATH)
    if not _strict_equal(r6, contract.FULL_R6_CANDIDATE_TREE):
        raise V12R10FreezeError("locked R6 candidate tree drifted")
    source_h0 = tree_record(contract.SOURCE_H0_MODULE_PATH)
    if not _strict_equal(source_h0, contract.LOCKED_SOURCE_H0_TREE):
        raise V12R10FreezeError("locked source H0 tree drifted")
    coverage = _assert_artifact(
        "coverage_reference_corpus", contract.LOCKED_COVERAGE_REFERENCE
    )
    return {
        "passed": True,
        "r9_terminal": r9,
        "imported_labels": labels,
        "r6_candidate": r6,
        "source_h0_teacher_id": contract.SOURCE_H0_ID,
        "source_h0_teacher": source_h0,
        "coverage_reference_corpus": coverage,
        "diagnostics": diagnostics,
        "r9_candidate_role": "INITIALIZATION_ONLY_NOT_PROMOTED",
        "collection_round_count": 0,
        "label_generation_count": 0,
        "environment_reset_calls": 0,
        "environment_step_calls": 0,
        "offline_h0_teacher_queries": 0,
    }


def _runtime_attestation() -> dict[str, Any]:
    try:
        value = dict(r9_freezer._runtime_attestation())  # noqa: SLF001
        _canonical_digest(value)
    except Exception as exc:
        raise V12R10FreezeError("runtime/plugin attestation failed") from exc
    readiness = value.get("platform_plugin_readiness")
    if (
        value.get("inference_stack_ready") is not True
        or not isinstance(readiness, Mapping)
        or readiness.get("passed") is not True
    ):
        raise V12R10FreezeError("runtime/plugin stack is not ready")
    return value


def _strict_runtime(attestor: RuntimeAttestor | None) -> dict[str, Any]:
    source = _runtime_attestation if attestor is None else attestor
    try:
        value = dict(source())
        _canonical_digest(value)
    except Exception as exc:
        raise V12R10FreezeError("runtime attestor returned malformed evidence") from exc
    readiness = value.get("platform_plugin_readiness")
    if (
        value.get("inference_stack_ready") is not True
        or not isinstance(readiness, Mapping)
        or readiness.get("passed") is not True
    ):
        raise V12R10FreezeError("runtime/plugin readiness is not PASS")
    return value


def expected_protocol_payload() -> dict[str, Any]:
    sources = production_source_closure()
    locked = attest_locked_inputs()
    self_check = contract.contract_self_check()
    if self_check.get("passed") is not True:
        raise V12R10FreezeError("V12R10 contract self-check failed")
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.PROTOCOL_FREEZE_PASS_STATUS,
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "topology_id": contract.TOPOLOGY_ID,
        "revision": contract.REVISION,
        "authority": copy.deepcopy(contract.AUTHORITY),
        "contract_self_check": self_check,
        "candidate_selection_rule": contract.CANDIDATE_SELECTION_RULE,
        "candidate_module_path": contract.CANDIDATE_MODULE_PATH.as_posix(),
        "fit_contract": copy.deepcopy(contract.FIT),
        "offline_thresholds": copy.deepcopy(contract.OFFLINE_THRESHOLDS),
        "development_cases": [
            contract.canonical_development_case(case_id)
            for case_id in contract.DEVELOPMENT_CASE_IDS
        ],
        "stage_order": list(contract.STAGE_IDS),
        "run_root": contract.RUN_ROOT.as_posix(),
        "locked_inputs": locked,
        "limitations": copy.deepcopy(contract.LIMITATIONS),
        "production_source_count": len(sources),
        "production_source_closure": sources,
        "transitive_source_basis": (
            "V12R9_FULL_SOURCE_CLOSURE_PLUS_R9_TERMINAL_IMPORTS_PLUS_R10"
        ),
        "one_shot": True,
        "single_actor_fit": True,
        "collection_round_count": 0,
        "label_generation_count": 0,
        "retry_authorized": False,
        "resume_authorized": False,
        "sweep_authorized": False,
        "qualification_execution_authorized": False,
        "positive_morphology_authorized": False,
        "environment_reset_calls": 0,
        "environment_step_calls": 0,
        "offline_h0_teacher_queries": 0,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
    }


def verify_protocol_freeze() -> dict[str, Any]:
    observed = _mapping(contract.PROTOCOL_FREEZE_PATH)
    expected = expected_protocol_payload()
    if not _strict_equal(observed, expected):
        raise V12R10FreezeError("V12R10 protocol freeze or source closure drifted")
    return observed


def publish_protocol_freeze() -> dict[str, Any]:
    destination = safe_repository_path(contract.PROTOCOL_FREEZE_PATH)
    if os.path.lexists(destination):
        return verify_protocol_freeze()
    if os.path.lexists(resolve_relative(contract.RUN_ROOT)):
        raise V12R10FreezeError("run root exists before protocol freeze")
    r9_freezer.forensic.write_json_exclusive(destination, expected_protocol_payload())
    return verify_protocol_freeze()


def expected_execution_lock_payload(
    *, runtime_attestor: RuntimeAttestor | None = None
) -> dict[str, Any]:
    protocol = verify_protocol_freeze()
    runtime = _strict_runtime(runtime_attestor)
    locked = attest_locked_inputs()
    occupancy = _current_execution_occupancy()
    return _build_execution_lock_payload(
        protocol=protocol,
        runtime=runtime,
        locked=locked,
        occupancy=occupancy,
    )


def _current_execution_occupancy() -> dict[str, bool]:
    return {
        "run_root_absent": not os.path.lexists(resolve_relative(contract.RUN_ROOT)),
        "pipeline_claim_absent": not os.path.lexists(
            resolve_relative(contract.CLAIM_PATH)
        ),
        "pipeline_ledger_absent": not os.path.lexists(
            resolve_relative(contract.LEDGER_PATH)
        ),
        "r9_import_attestation_absent": not os.path.lexists(
            resolve_relative(contract.R9_IMPORT_ATTESTATION_PATH)
        ),
    }


def _build_execution_lock_payload(
    *,
    protocol: Mapping[str, Any],
    runtime: Mapping[str, Any],
    locked: Mapping[str, Any],
    occupancy: Mapping[str, Any],
) -> dict[str, Any]:
    """Build the complete lock schema from independently attested evidence.

    The same builder is used for publication and verification.  Verification
    supplies the historical pristine occupancy required at publication, so a
    completed run may be verified without rewriting that immutable history.
    """

    sources = production_source_closure()
    expected_occupancy_keys = {
        "run_root_absent",
        "pipeline_claim_absent",
        "pipeline_ledger_absent",
        "r9_import_attestation_absent",
    }
    checks = {
        "protocol_exact": _strict_equal(protocol, expected_protocol_payload()),
        "sources_exact": _strict_equal(
            protocol.get("production_source_closure"), sources
        ),
        "locked_inputs_exact": locked.get("passed") is True,
        "runtime_ready": runtime.get("inference_stack_ready") is True
        and runtime.get("platform_plugin_readiness", {}).get("passed") is True,
        "occupancy_pristine": set(occupancy) == expected_occupancy_keys
        and all(value is True for value in occupancy.values()),
        "authority_exact": contract.AUTHORITY.get("actor_fit_execution_authorized")
        is True
        and contract.AUTHORITY.get("development_execution_authorized") is True
        and contract.AUTHORITY.get("new_environment_collection_authorized") is False
        and contract.AUTHORITY.get("observer_only_teacher_queries_authorized") is False,
    }
    passed = all(value is True for value in checks.values())
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": (
            contract.EXECUTION_LOCK_PASS_STATUS
            if passed
            else "FAIL_H0_V12R10_RECOVERY_EXECUTION_LOCK"
        ),
        "passed": passed,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "topology_id": contract.TOPOLOGY_ID,
        "revision": contract.REVISION,
        "authority": copy.deepcopy(contract.AUTHORITY),
        "candidate_selection_rule": contract.CANDIDATE_SELECTION_RULE,
        "candidate_module_path": contract.CANDIDATE_MODULE_PATH.as_posix(),
        "checks": checks,
        "occupancy": occupancy,
        "runtime": runtime,
        "protocol_freeze": artifact_record(contract.PROTOCOL_FREEZE_PATH),
        "production_source_closure": sources,
        "locked_inputs": locked,
        "limitations": copy.deepcopy(contract.LIMITATIONS),
        "stage_order": list(contract.STAGE_IDS),
        "run_root": contract.RUN_ROOT.as_posix(),
        "one_shot": True,
        "single_actor_fit": True,
        "collection_round_count": 0,
        "label_generation_count": 0,
        "retry_authorized": False,
        "resume_authorized": False,
        "sweep_authorized": False,
        "qualification_execution_authorized": False,
        "positive_morphology_authorized": False,
        "environment_reset_calls": 0,
        "environment_step_calls": 0,
        "offline_h0_teacher_queries": 0,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
    }


def verify_execution_lock(
    *,
    require_pristine: bool = False,
    runtime_attestor: RuntimeAttestor | None = None,
) -> dict[str, Any]:
    if type(require_pristine) is not bool:
        raise V12R10FreezeError("require_pristine must be strict bool")
    protocol = verify_protocol_freeze()
    lock = _mapping(contract.EXECUTION_LOCK_PATH)
    current_runtime = _strict_runtime(runtime_attestor)
    locked = attest_locked_inputs()
    historical_occupancy = {key: True for key in _current_execution_occupancy()}
    expected = _build_execution_lock_payload(
        protocol=protocol,
        runtime=current_runtime,
        locked=locked,
        occupancy=historical_occupancy,
    )
    if not _strict_equal(lock, expected):
        missing = sorted(set(expected) - set(lock))
        extra = sorted(set(lock) - set(expected))
        drifted = sorted(
            key
            for key in set(lock) & set(expected)
            if not _strict_equal(lock[key], expected[key])
        )
        raise V12R10FreezeError(
            "V12R10 execution lock drifted: "
            f"missing={missing}, extra={extra}, fields={drifted}"
        )
    if require_pristine:
        current_occupancy = _current_execution_occupancy()
        if not all(value is True for value in current_occupancy.values()):
            occupied = sorted(
                name for name, value in current_occupancy.items() if value is not True
            )
            raise V12R10FreezeError(
                f"V12R10 run root is no longer pristine: {occupied}"
            )
    return lock


def publish_execution_lock(
    *, runtime_attestor: RuntimeAttestor | None = None
) -> dict[str, Any]:
    destination = safe_repository_path(contract.EXECUTION_LOCK_PATH)
    if os.path.lexists(destination):
        return verify_execution_lock(
            require_pristine=True, runtime_attestor=runtime_attestor
        )
    if os.path.lexists(resolve_relative(contract.RUN_ROOT)):
        raise V12R10FreezeError("run root exists before execution lock")
    payload = expected_execution_lock_payload(runtime_attestor=runtime_attestor)
    if payload.get("passed") is not True:
        failed = sorted(
            name for name, value in payload.get("checks", {}).items() if not value
        )
        raise V12R10FreezeError(f"execution lock preconditions failed: {failed}")
    r9_freezer.forensic.write_json_exclusive(destination, payload)
    return verify_execution_lock(
        require_pristine=True, runtime_attestor=runtime_attestor
    )


def prepare(*, runtime_attestor: RuntimeAttestor | None = None) -> dict[str, Any]:
    if os.path.lexists(resolve_relative(contract.RUN_ROOT)):
        raise V12R10FreezeError("run root must be pristine before prepare")
    protocol = publish_protocol_freeze()
    lock = publish_execution_lock(runtime_attestor=runtime_attestor)
    return {
        "passed": True,
        "protocol_freeze": artifact_record(contract.PROTOCOL_FREEZE_PATH),
        "execution_lock": artifact_record(contract.EXECUTION_LOCK_PATH),
        "protocol": protocol,
        "lock": lock,
    }


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--prepare", action="store_true")
    mode.add_argument("--verify", action="store_true")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    result = (
        prepare()
        if args.prepare
        else {
            "protocol": verify_protocol_freeze(),
            "lock": verify_execution_lock(require_pristine=False),
        }
    )
    print(json.dumps(result, indent=2, sort_keys=True, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = [
    "V12R10FreezeError",
    "artifact_record",
    "attest_locked_inputs",
    "expected_execution_lock_payload",
    "expected_protocol_payload",
    "prepare",
    "production_source_closure",
    "production_source_paths",
    "publish_execution_lock",
    "publish_protocol_freeze",
    "resolve_relative",
    "safe_repository_path",
    "tree_record",
    "verify_execution_lock",
    "verify_protocol_freeze",
]
