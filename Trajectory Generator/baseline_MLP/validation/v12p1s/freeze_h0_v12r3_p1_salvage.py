"""Freeze the additive V12R3-P1 salvage protocol and candidate selection.

The freezer is deliberately read-only except for one explicit, atomic,
no-clobber publication of the canonical protocol JSON.  It binds the eleven
selected V12R3 records, the complete terminal V12R3 run tree, and the unique
P1 candidate that passed both its fit and autonomous pure-probe gates.  P2 is
recorded only as excluded terminal history and is never loaded or selected.

Importing or building this freeze cannot fit a model, load a checkpoint,
label data, reset/step an environment, update a policy, or execute a rollout.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
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
ROOT_VALIDATION = REPO_ROOT / "validation"
LOCAL_VALIDATION = REPO_ROOT / "Trajectory Generator" / "baseline_MLP" / "validation"
V12P1S_ROOT = Path(__file__).resolve().parent
V12P1Q_ROOT = LOCAL_VALIDATION / "v12p1q"
for _root in (
    REPO_ROOT,
    ROOT_VALIDATION,
    LOCAL_VALIDATION,
    V12P1S_ROOT,
    V12P1Q_ROOT,
):
    if str(_root) not in sys.path:
        sys.path.insert(0, str(_root))

import h0_forensic_rollout as forensic  # noqa: E402
import h0_v12r3_p1_salvage_contract as contract  # noqa: E402
import freeze_h0_v12r3_p1_qualification_design as q_design_freezer  # noqa: E402
import h0_v12r3_p1_qualification_contract as q_contract  # noqa: E402


class V12R3P1SalvageFreezeError(RuntimeError):
    """Raised when the additive P1 selection cannot remain fail-closed."""


SCHEMA_VERSION = contract.SCHEMA_VERSION
PROTOCOL_ID = contract.PROTOCOL_ID
PROTOCOL_FREEZE_PATH = contract.PROTOCOL_FREEZE_PATH

R3_RUN_ROOT = PurePosixPath(contract.R3_TERMINAL_RUN_TREE["path"])
R3_P0_FIT_GATE_PATH = R3_RUN_ROOT / "fit/p0/gate.json"
R3_P0_PROBE_GATE_PATH = R3_RUN_ROOT / "probe/p0/gate.json"
R3_P2_FIT_ROOT = R3_RUN_ROOT / "fit/p2"
R3_P3_FIT_ROOT = R3_RUN_ROOT / "fit/p3"
R3_P2_CLAIM_PATH = R3_RUN_ROOT / "claims/11_fit_p2.json"

# These are the complete materialized P2 outputs.  Their bytes remain bound by
# the terminal tree digest, but their paths are explicitly barred from the
# candidate selection and every later execution input.
EXCLUDED_P2_HISTORY_PATHS = (
    R3_P2_CLAIM_PATH,
    R3_P2_FIT_ROOT / "adaptation_history.json",
    R3_P2_FIT_ROOT / "adaptation_report.json",
    R3_P2_FIT_ROOT / "corpus.npz",
    R3_P2_FIT_ROOT / "rl_module_target_adapted/class_and_ctor_args.pkl",
    R3_P2_FIT_ROOT / "rl_module_target_adapted/metadata.json",
    R3_P2_FIT_ROOT / "rl_module_target_adapted/module_state.pkl",
)

ZERO_FREEZE_ACTIVITY = {
    "actor_fit_executions": 0,
    "offline_teacher_label_calls": 0,
    "environment_reset_calls": 0,
    "environment_step_calls": 0,
    "actor_updates": 0,
    "critic_updates": 0,
    "ppo_updates": 0,
    "rollout_executions": 0,
}

P1_OFFLINE_LIMITS = {
    "rmse": 0.006,
    "max_abs_error": 0.06,
    "reset_max_abs_error": 0.003,
}


def resolve_relative(path: str | os.PathLike[str] | PurePosixPath) -> Path:
    """Return a lexical path without following symlinks or reparse points."""

    raw = path.as_posix() if isinstance(path, PurePosixPath) else os.fspath(path)
    value = Path(raw)
    if value.is_absolute():
        if ".." in value.parts:
            raise V12R3P1SalvageFreezeError(f"unsafe absolute path: {path}")
        return value
    pure = PurePosixPath(raw)
    if not raw or pure.is_absolute() or ".." in pure.parts or pure.as_posix() != raw:
        raise V12R3P1SalvageFreezeError(f"unsafe relative path: {path}")
    return REPO_ROOT.joinpath(*pure.parts)


def _is_link_or_reparse(path: Path) -> bool:
    """Reject POSIX symlinks and Windows junction/reparse points."""

    try:
        metadata = os.lstat(path)
    except FileNotFoundError:
        return False
    if stat.S_ISLNK(metadata.st_mode):
        return True
    attributes = getattr(metadata, "st_file_attributes", 0)
    reparse = getattr(stat, "FILE_ATTRIBUTE_REPARSE_POINT", 0)
    return bool(reparse and attributes & reparse)


def _assert_no_link_components(path: Path, *, allow_missing_leaf: bool) -> None:
    """Reject every existing link-like component without resolving it."""

    lexical = Path(os.path.abspath(os.fspath(path)))
    anchor = Path(lexical.anchor)
    current = anchor
    parts = lexical.parts[1:] if lexical.anchor else lexical.parts
    for part in parts:
        current = current / part
        if not os.path.lexists(current):
            if allow_missing_leaf:
                continue
            raise V12R3P1SalvageFreezeError(f"required path is missing: {current}")
        if _is_link_or_reparse(current):
            raise V12R3P1SalvageFreezeError(f"link/reparse path rejected: {current}")


def _relative(path: str | os.PathLike[str] | PurePosixPath) -> str:
    source = resolve_relative(path)
    try:
        return source.relative_to(REPO_ROOT).as_posix()
    except ValueError as exc:
        raise V12R3P1SalvageFreezeError(
            f"artifact is outside repository: {source}"
        ) from exc


def _mapping(path: str | os.PathLike[str] | PurePosixPath) -> dict[str, Any]:
    source = resolve_relative(path)
    _assert_no_link_components(source, allow_missing_leaf=False)
    try:
        value = forensic.strict_json_load(source)
    except Exception as exc:
        raise V12R3P1SalvageFreezeError(f"cannot read strict JSON: {path}") from exc
    if not isinstance(value, Mapping):
        raise V12R3P1SalvageFreezeError(f"expected strict JSON object: {path}")
    return dict(value)


def _record(path: str | os.PathLike[str] | PurePosixPath) -> dict[str, Any]:
    source = resolve_relative(path)
    _assert_no_link_components(source, allow_missing_leaf=False)
    metadata = os.lstat(source)
    if not stat.S_ISREG(metadata.st_mode) or _is_link_or_reparse(source):
        raise V12R3P1SalvageFreezeError(f"artifact is not a regular file: {path}")
    return forensic.artifact_record(source, artifact_root=REPO_ROOT)


def _qualification_design_freeze_binding() -> dict[str, Any]:
    """Verify and bind the canonical pre-salvage qualification design."""

    declared = resolve_relative(contract.QUALIFICATION_DESIGN_FREEZE_PATH)
    q_declared = q_design_freezer.resolve_relative(
        q_contract.QUALIFICATION_DESIGN_FREEZE_PATH
    )
    constants_exact = (
        q_contract.QUALIFICATION_DESIGN_FREEZE_PATH
        == contract.QUALIFICATION_DESIGN_FREEZE_PATH
        and q_contract.SCHEMA_VERSION == contract.QUALIFICATION_DESIGN_SCHEMA_VERSION
        and q_contract.QUALIFICATION_DESIGN_FREEZE_PASS_STATUS
        == contract.QUALIFICATION_DESIGN_FREEZE_PASS_STATUS
        and q_contract.PROTOCOL_ID == contract.QUALIFICATION_DESIGN_PROTOCOL_ID
        and declared == q_declared
    )
    if not constants_exact:
        raise V12R3P1SalvageFreezeError(
            "qualification design canonical identity drifted"
        )
    try:
        payload = q_design_freezer.verify_design_freeze()
    except Exception as exc:
        raise V12R3P1SalvageFreezeError(
            "canonical qualification design freeze verification failed"
        ) from exc
    if not isinstance(payload, Mapping):
        raise V12R3P1SalvageFreezeError(
            "qualification design verifier returned a malformed payload"
        )
    gate = dict(contract.qualification_design_freeze_gate(payload))
    if gate.get("passed") is not True:
        failed = [
            name for name, value in gate.get("checks", {}).items() if value is not True
        ]
        raise V12R3P1SalvageFreezeError(
            f"qualification design semantics are not PASS: {failed}"
        )
    return {
        "payload": dict(payload),
        "record": _record(declared),
        "gate": gate,
    }


def _tree_record(path: str | os.PathLike[str] | PurePosixPath) -> dict[str, Any]:
    """Hash every path/hash/size tuple in one symlink-free artifact tree."""

    root = resolve_relative(path)
    _assert_no_link_components(root, allow_missing_leaf=False)
    if not root.is_dir() or _is_link_or_reparse(root):
        raise V12R3P1SalvageFreezeError(f"artifact tree is missing: {path}")
    entries = sorted(root.rglob("*"))
    if any(_is_link_or_reparse(entry) for entry in entries):
        raise V12R3P1SalvageFreezeError(
            f"artifact tree contains link/reparse point: {path}"
        )
    files = [entry for entry in entries if entry.is_file()]
    if not files:
        raise V12R3P1SalvageFreezeError(f"artifact tree is empty: {path}")
    digest = hashlib.sha256()
    rows: list[dict[str, Any]] = []
    total_size = 0
    for item in files:
        relative = item.relative_to(root).as_posix()
        sha256 = forensic.sha256_file(item)
        size_bytes = item.stat().st_size
        total_size += size_bytes
        row = {
            "path": relative,
            "sha256": sha256,
            "size_bytes": size_bytes,
        }
        rows.append(row)
        digest.update(relative.encode("utf-8"))
        digest.update(b"\0")
        digest.update(sha256.encode("ascii"))
        digest.update(b"\0")
        digest.update(str(size_bytes).encode("ascii"))
        digest.update(b"\n")
    return {
        "path": _relative(root),
        "tree_sha256": digest.hexdigest(),
        "file_count": len(rows),
        "total_size_bytes": total_size,
        "files": rows,
    }


def _tree_summary(tree: Mapping[str, Any]) -> dict[str, Any]:
    return {
        "path": tree.get("path"),
        "tree_sha256": tree.get("tree_sha256"),
        "file_count": tree.get("file_count"),
        "total_size_bytes": tree.get("total_size_bytes"),
    }


def _source_records() -> dict[str, dict[str, Any]]:
    """Bind every Python source in this additive lineage at publication time."""

    sources = sorted(V12P1S_ROOT.glob("*.py"))
    if not sources:
        raise V12R3P1SalvageFreezeError("V12P1S has no source files")
    return {source.name: _record(source) for source in sources}


def _selected_artifact_gate() -> dict[str, Any]:
    expected = copy.deepcopy(contract.R3_SELECTED_ARTIFACTS)
    observed = {name: _record(artifact["path"]) for name, artifact in expected.items()}
    selected_paths = [str(record["path"]) for record in observed.values()]
    p2_prefix = f"{R3_P2_FIT_ROOT.as_posix()}/"
    label_p1_fragments = ("/label/p1/receipt.json", "/label/p1/gate.json")
    checks = {
        "exactly_eleven_records": len(expected) == len(observed) == 11,
        "records_byte_exact": observed == expected,
        "paths_unique": len(selected_paths) == len(set(selected_paths)),
        "no_selected_p2_output": all(
            not path.startswith(p2_prefix) and path != R3_P2_CLAIM_PATH.as_posix()
            for path in selected_paths
        ),
        "label_p1_receipt_and_gate_not_selected": all(
            not path.endswith(label_p1_fragments) for path in selected_paths
        ),
    }
    return {
        "passed": all(checks.values()),
        "checks": checks,
        "expected": expected,
        "observed": observed,
    }


def _terminal_lineage_gate(
    selected: Mapping[str, Any],
) -> dict[str, Any]:
    tree = _tree_record(contract.R3_TERMINAL_RUN_TREE["path"])
    run_tree = _tree_summary(tree)
    ledger = _mapping(contract.R3_SELECTED_ARTIFACTS["r3_pipeline_ledger"]["path"])
    semantics = {
        name: copy.deepcopy(ledger.get(name)) for name in contract.R3_TERMINAL_SEMANTICS
    }
    lineage = {
        "selected_artifacts": copy.deepcopy(selected.get("observed")),
        "run_tree": run_tree,
        "semantics": semantics,
    }
    checks = {
        "complete_tree_digest_exact": run_tree == contract.R3_TERMINAL_RUN_TREE,
        "all_tree_files_hashed": len(tree["files"]) == tree["file_count"],
        "terminal_semantics_exact": semantics == contract.R3_TERMINAL_SEMANTICS,
        "terminal_lineage_exact": lineage == contract.R3_TERMINAL_LINEAGE,
        "r3_terminal_fail_not_reopened": ledger.get("passed") is False
        and ledger.get("attempted_stage") == "fit_p2"
        and ledger.get("retry_authorized") is False
        and ledger.get("sweep_authorized") is False
        and ledger.get("rescue_authorized") is False
        and ledger.get("runtime_promoted") is False,
    }
    return {
        "passed": all(checks.values()),
        "checks": checks,
        "lineage": lineage,
        "tree": tree,
    }


def _p2_exclusion_gate(terminal: Mapping[str, Any]) -> dict[str, Any]:
    terminal_files = {
        str(row["path"])
        for row in terminal.get("tree", {}).get("files", [])
        if isinstance(row, Mapping)
    }
    expected_relative = {
        PurePosixPath(path).relative_to(R3_RUN_ROOT).as_posix()
        for path in EXCLUDED_P2_HISTORY_PATHS
    }
    materialized_relative = {
        path
        for path in terminal_files
        if path.startswith("fit/p2/") or path == "claims/11_fit_p2.json"
    }
    selected_paths = {
        str(record["path"]) for record in contract.R3_SELECTED_ARTIFACTS.values()
    }
    checks = {
        "all_seven_p2_history_paths_enumerated": len(expected_relative) == 7
        and materialized_relative == expected_relative,
        "p2_history_is_inside_terminal_tree": expected_relative <= terminal_files,
        "no_p2_history_path_selected": all(
            path.as_posix() not in selected_paths for path in EXCLUDED_P2_HISTORY_PATHS
        ),
        "p2_has_no_pass_summary_gate_or_receipt": all(
            not resolve_relative(R3_P2_FIT_ROOT / name).exists()
            for name in ("summary.json", "gate.json", "receipt.json")
        ),
        "p2_probe_not_materialized": not resolve_relative(
            R3_RUN_ROOT / "probe/p2"
        ).exists(),
        "p1_is_the_selected_module": contract.P1_CANDIDATE_MODULE["path"]
        != (R3_P2_FIT_ROOT / "rl_module_target_adapted").as_posix(),
    }
    return {
        "passed": all(checks.values()),
        "checks": checks,
        "excluded_paths": [path.as_posix() for path in EXCLUDED_P2_HISTORY_PATHS],
        "selection_allowed": False,
        "input_allowed": False,
        "module_loaded": False,
        "corpus_loaded": False,
    }


def _unique_eligibility_gate() -> dict[str, Any]:
    p0_fit = _mapping(R3_P0_FIT_GATE_PATH)
    p0_probe = _mapping(R3_P0_PROBE_GATE_PATH)
    fit_gate = _mapping(contract.R3_SELECTED_ARTIFACTS["r3_fit_p1_gate"]["path"])
    fit_summary = _mapping(contract.R3_SELECTED_ARTIFACTS["r3_fit_p1_summary"]["path"])
    fit_receipt = _mapping(contract.R3_SELECTED_ARTIFACTS["r3_fit_p1_receipt"]["path"])
    probe_gate = _mapping(contract.R3_SELECTED_ARTIFACTS["r3_probe_p1_gate"]["path"])
    probe_summary = _mapping(
        contract.R3_SELECTED_ARTIFACTS["r3_probe_p1_summary"]["path"]
    )
    probe_receipt = _mapping(
        contract.R3_SELECTED_ARTIFACTS["r3_probe_p1_receipt"]["path"]
    )
    module = _tree_record(contract.P1_CANDIDATE_MODULE["path"])
    module.pop("total_size_bytes")
    metrics = fit_summary.get("metrics")
    metric_checks = {
        name: isinstance(metrics, Mapping)
        and isinstance(metrics.get(name), (int, float))
        and not isinstance(metrics.get(name), bool)
        and float(metrics[name]) <= limit
        for name, limit in P1_OFFLINE_LIMITS.items()
    }
    p1_identity = all(
        payload.get("candidate_id") == contract.P1_CANDIDATE_ID
        for payload in (
            fit_summary,
            fit_receipt,
            probe_gate,
            probe_summary,
            probe_receipt,
        )
    )
    p1_tree_identity = (
        module == contract.P1_CANDIDATE_MODULE
        and fit_summary.get("candidate_module") == contract.P1_CANDIDATE_MODULE
        and probe_summary.get("candidate_module") == contract.P1_CANDIDATE_MODULE
        and probe_gate.get("candidate_module_tree_sha256")
        == contract.P1_CANDIDATE_MODULE["tree_sha256"]
        and probe_receipt.get("candidate_module_tree_sha256")
        == contract.P1_CANDIDATE_MODULE["tree_sha256"]
    )
    checks = {
        "p0_ineligible_after_autonomy_fail": p0_fit.get("passed") is True
        and p0_probe.get("passed") is False
        and p0_probe.get("integrity_passed") is True,
        "p1_fit_passed": fit_gate.get("passed") is True
        and fit_receipt.get("passed") is True
        and all(metric_checks.values()),
        "p1_probe_integrity_and_autonomy_passed": probe_gate.get("passed") is True
        and probe_gate.get("integrity_passed") is True
        and probe_receipt.get("passed") is True,
        "p1_identity_exact": p1_identity and p1_tree_identity,
        "p2_has_no_pass_fit_receipt": not resolve_relative(
            R3_P2_FIT_ROOT / "receipt.json"
        ).exists(),
        "p3_not_materialized": not resolve_relative(R3_P3_FIT_ROOT).exists(),
        "unique_candidate_is_p1": contract.CANDIDATE_SELECTION_POLICY.get(
            "eligible_candidates"
        )
        == ["p1"],
    }
    return {
        "passed": all(checks.values()),
        "checks": checks,
        "p1_offline_metric_checks": metric_checks,
        "p1_offline_metrics": copy.deepcopy(metrics),
        "candidate_module": module,
    }


def _candidate_selection_payload(
    *,
    selected: Mapping[str, Any],
    terminal: Mapping[str, Any],
    qualification_design: Mapping[str, Any],
) -> dict[str, Any]:
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.CANDIDATE_SELECTION_COMPLETE_STATUS,
        "protocol_id": contract.PROTOCOL_ID,
        "candidate_id": contract.P1_CANDIDATE_ID,
        "candidate_fit_stage": "p1",
        "candidate_module": copy.deepcopy(contract.P1_CANDIDATE_MODULE),
        "r3_selected_artifacts": copy.deepcopy(selected.get("observed")),
        "r3_terminal_run_tree": copy.deepcopy(
            terminal.get("lineage", {}).get("run_tree")
        ),
        "r3_terminal_semantics": copy.deepcopy(
            terminal.get("lineage", {}).get("semantics")
        ),
        "selection_policy": copy.deepcopy(contract.CANDIDATE_SELECTION_POLICY),
        "qualification_design_freeze": copy.deepcopy(qualification_design["record"]),
        "qualification_design_freeze_gate": copy.deepcopy(qualification_design["gate"]),
        "unique_eligible_candidate": True,
        "p1_fit_gate_passed": True,
        "p1_probe_integrity_passed": True,
        "p1_probe_autonomy_passed": True,
        "v12r3_reopened": False,
        "v12r3_completed": False,
        "runtime_promoted": False,
        "label_p1_used_for_candidate_selection": False,
        "p2_artifacts_used": [],
        "p2_module_loaded": False,
        "p2_corpus_loaded": False,
        "actor_fit_executions": 0,
        "offline_teacher_label_calls": 0,
        "environment_reset_calls": 0,
        "environment_step_calls": 0,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
        "retry_authorized": False,
        "sweep_authorized": False,
        "rescue_authorized": False,
        "post_hoc_retuning_authorized": False,
    }


def _authority_gate() -> dict[str, Any]:
    authority = contract.AUTHORITY
    checks = {
        "schema_exact": set(authority)
        == {
            "authority_date",
            "authority_text",
            "authority_scope",
            "one_shot",
            "protocol_freeze_authorized",
            "execution_lock_authorized",
            "development_execution_authorized",
            "actor_fit_authorized",
            "offline_teacher_labeling_authorized",
            "actor_updates_authorized",
            "critic_updates_authorized",
            "ppo_updates_authorized",
            "p2_reuse_authorized",
            "retry_authorized",
            "sweep_authorized",
            "rescue_authorized",
            "runtime_promotion_authorized",
            "qualification_execution_authorized",
            "checkpoint_zero_authorized",
            "positive_morphology_authorized",
        },
        "identity_exact": authority.get("authority_date") == "2026-08-09"
        and authority.get("authority_text") == "esegui i punti 1-6"
        and authority.get("authority_scope") == contract.AUTHORITY_SCOPE,
        "one_shot": authority.get("one_shot") is True,
        "freeze_authorized": authority.get("protocol_freeze_authorized") is True,
        "no_fit_update_or_p2": authority.get("actor_fit_authorized") is False
        and authority.get("offline_teacher_labeling_authorized") is False
        and authority.get("actor_updates_authorized") is False
        and authority.get("critic_updates_authorized") is False
        and authority.get("ppo_updates_authorized") is False
        and authority.get("p2_reuse_authorized") is False,
        "no_retry_sweep_rescue_promotion": authority.get("retry_authorized") is False
        and authority.get("sweep_authorized") is False
        and authority.get("rescue_authorized") is False
        and authority.get("runtime_promotion_authorized") is False,
    }
    return {"passed": all(checks.values()), "checks": checks}


def _path_isolation_gate() -> dict[str, Any]:
    declared = contract.declared_mutation_paths()
    lexical = {name: PurePosixPath(path) for name, path in declared.items()}
    values = [path.as_posix() for path in lexical.values()]
    new_root = contract.VALIDATION_ROOT.as_posix()
    old_root = R3_RUN_ROOT.as_posix()
    selected_paths = {
        str(record["path"]) for record in contract.R3_SELECTED_ARTIFACTS.values()
    }
    maximum = max(len(path) for path in values)
    checks = {
        "all_paths_relative_and_normalized": all(
            not path.is_absolute() and ".." not in path.parts
            for path in lexical.values()
        ),
        "all_mutations_inside_new_lineage": all(
            path == new_root or path.startswith(f"{new_root}/") for path in values
        ),
        "no_mutation_inside_terminal_r3": all(
            path != old_root and not path.startswith(f"{old_root}/") for path in values
        ),
        "no_selected_input_is_a_mutation": selected_paths.isdisjoint(values),
        "paths_unique": len(values) == len(set(values)),
        "protocol_freeze_declared_exact": declared.get("protocol_freeze")
        == contract.PROTOCOL_FREEZE_PATH,
        "portable_relative_path_budget": maximum < 220,
    }
    return {
        "passed": all(checks.values()),
        "checks": checks,
        "declared_mutation_paths": {
            name: path.as_posix() for name, path in lexical.items()
        },
        "maximum_relative_path_length": maximum,
    }


def _source_gate() -> dict[str, Any]:
    sources = _source_records()
    expected_prefix = f"{contract.VALIDATION_ROOT.as_posix()}/"
    checks = {
        "contract_freezer_and_tests_present": {
            "h0_v12r3_p1_salvage_contract.py",
            "freeze_h0_v12r3_p1_salvage.py",
            "test_freeze_h0_v12r3_p1_salvage.py",
        }
        <= set(sources),
        "all_sources_inside_new_lineage": all(
            str(record["path"]).startswith(expected_prefix)
            for record in sources.values()
        ),
        "all_hashes_well_formed": all(
            isinstance(record.get("sha256"), str)
            and len(record["sha256"]) == 64
            and type(record.get("size_bytes")) is int
            and record["size_bytes"] > 0
            for record in sources.values()
        ),
    }
    return {"passed": all(checks.values()), "checks": checks, "records": sources}


def _zero_activity_gate() -> dict[str, Any]:
    checks = {
        "exact_zero_schema": set(ZERO_FREEZE_ACTIVITY)
        == {
            "actor_fit_executions",
            "offline_teacher_label_calls",
            "environment_reset_calls",
            "environment_step_calls",
            "actor_updates",
            "critic_updates",
            "ppo_updates",
            "rollout_executions",
        },
        "all_zero_integers": all(
            type(value) is int and value == 0 for value in ZERO_FREEZE_ACTIVITY.values()
        ),
    }
    return {"passed": all(checks.values()), "checks": checks}


def _occupancy_snapshot() -> dict[str, bool]:
    return {
        "protocol_freeze_unoccupied": not os.path.lexists(
            resolve_relative(contract.PROTOCOL_FREEZE_PATH)
        ),
        "execution_lock_absent": not os.path.lexists(
            resolve_relative(contract.EXECUTION_LOCK_PATH)
        ),
        "new_run_root_absent": not os.path.lexists(resolve_relative(contract.RUN_ROOT)),
    }


def _assemble_protocol_freeze(occupancy: Mapping[str, bool]) -> dict[str, Any]:
    if set(occupancy) != {
        "protocol_freeze_unoccupied",
        "execution_lock_absent",
        "new_run_root_absent",
    } or not all(type(value) is bool for value in occupancy.values()):
        raise V12R3P1SalvageFreezeError("occupancy schema drifted")

    qualification_design = _qualification_design_freeze_binding()
    selected = _selected_artifact_gate()
    terminal = _terminal_lineage_gate(selected)
    p2 = _p2_exclusion_gate(terminal)
    eligibility = _unique_eligibility_gate()
    selection_payload = _candidate_selection_payload(
        selected=selected,
        terminal=terminal,
        qualification_design=qualification_design,
    )
    selection_gate = contract.candidate_selection_gate(selection_payload)
    authority = _authority_gate()
    isolation = _path_isolation_gate()
    source = _source_gate()
    zero = _zero_activity_gate()
    checks = {
        "schema_124": contract.SCHEMA_VERSION == 124,
        "authority_exact": authority["passed"] is True,
        "new_additive_non_retry_lineage": contract.PROTOCOL_ID
        == "AB06_H0_V12R3_P1_SALVAGE_V26_DEVELOPMENT"
        and contract.PROTOCOL_ID != "AB06_H0_PRIMARY_SPLIT_V12R3_V26_AUTONOMY_RECOVERY",
        "eleven_selected_r3_records": selected["passed"] is True,
        "complete_terminal_r3_tree": terminal["passed"] is True,
        "p2_explicitly_excluded": p2["passed"] is True,
        "p1_uniquely_gate_eligible": eligibility["passed"] is True,
        "pure_candidate_selection_gate": selection_gate.get("passed") is True,
        "qualification_design_freeze_bound": qualification_design["gate"].get("passed")
        is True,
        "source_hashes_bound": source["passed"] is True,
        "write_paths_isolated": isolation["passed"] is True,
        "freeze_activity_zero": zero["passed"] is True,
        **dict(occupancy),
    }
    passed = all(checks.values())
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": (
            contract.PROTOCOL_FREEZE_PASS_STATUS
            if passed
            else contract.PROTOCOL_FREEZE_FAIL_STATUS
        ),
        "passed": passed,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "revision": contract.REVISION,
        "freeze_kind": "PROTOCOL_AND_UNIQUE_CANDIDATE_SELECTION",
        "lineage_kind": "ADDITIVE_NEW_LINEAGE_NOT_V12R3_RETRY",
        "checks": checks,
        "authority": copy.deepcopy(contract.AUTHORITY),
        "authority_gate": authority,
        "selected_candidate": copy.deepcopy(contract.P1_CANDIDATE_MODULE),
        "selected_candidate_id": contract.P1_CANDIDATE_ID,
        "selected_candidate_fit_stage": "p1",
        "candidate_selection_payload": selection_payload,
        "candidate_selection_gate": selection_gate,
        "qualification_design_freeze": copy.deepcopy(qualification_design["record"]),
        "qualification_design_freeze_gate": copy.deepcopy(qualification_design["gate"]),
        "unique_eligibility_audit": eligibility,
        "r3_selected_evidence": copy.deepcopy(selected["observed"]),
        "r3_selected_evidence_gate": {
            "passed": selected["passed"],
            "checks": selected["checks"],
        },
        "r3_terminal_lineage": copy.deepcopy(terminal["lineage"]),
        "r3_terminal_lineage_gate": {
            "passed": terminal["passed"],
            "checks": terminal["checks"],
        },
        "excluded_p2_terminal_history": p2,
        "source_hashes": source["records"],
        "source_gate": {"passed": source["passed"], "checks": source["checks"]},
        "write_path_isolation": isolation,
        "zero_freeze_activity": copy.deepcopy(ZERO_FREEZE_ACTIVITY),
        "zero_freeze_activity_gate": zero,
        "execution_lock": None,
        "pipeline_claim": None,
        "v12r3_reopened": False,
        "v12r3_completed": False,
        "runtime_promoted": False,
        "qualification_claimed": False,
        "checkpoint_zero_created": False,
        "morphology_weight": 0.0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
        "next_stage": "REQUIRE_SEPARATE_NO_CLOBBER_EXECUTION_LOCK",
    }


def build_protocol_freeze() -> dict[str, Any]:
    """Build the prospective freeze in memory without publishing anything."""

    return _assemble_protocol_freeze(_occupancy_snapshot())


def publish_protocol_freeze(
    *,
    output_path: str | os.PathLike[str] | None = None,
    enforce_canonical_destination: bool = True,
) -> dict[str, Any]:
    """Atomically publish one freeze and refuse every overwrite."""

    destination = resolve_relative(
        contract.PROTOCOL_FREEZE_PATH if output_path is None else output_path
    )
    canonical = resolve_relative(contract.PROTOCOL_FREEZE_PATH)
    if enforce_canonical_destination and destination != canonical:
        raise V12R3P1SalvageFreezeError(
            f"non-canonical protocol freeze destination: {destination}"
        )
    _assert_no_link_components(destination, allow_missing_leaf=True)
    if os.path.lexists(destination):
        raise V12R3P1SalvageFreezeError(f"refusing to clobber: {destination}")
    payload = build_protocol_freeze()
    if payload.get("passed") is not True:
        failed = [
            name for name, value in payload["checks"].items() if value is not True
        ]
        raise V12R3P1SalvageFreezeError(f"protocol freeze checks failed: {failed}")
    try:
        forensic.write_json_exclusive(destination, payload)
    except Exception as exc:
        raise V12R3P1SalvageFreezeError(
            f"protocol freeze publication failed: {destination}"
        ) from exc
    return verify_protocol_freeze(
        input_path=destination,
        enforce_canonical_destination=enforce_canonical_destination,
    )


def verify_protocol_freeze(
    *,
    input_path: str | os.PathLike[str] | None = None,
    enforce_canonical_destination: bool = True,
) -> dict[str, Any]:
    """Rebuild every binding and require canonical byte-for-byte identity."""

    destination = resolve_relative(
        contract.PROTOCOL_FREEZE_PATH if input_path is None else input_path
    )
    canonical = resolve_relative(contract.PROTOCOL_FREEZE_PATH)
    if enforce_canonical_destination and destination != canonical:
        raise V12R3P1SalvageFreezeError(f"non-canonical protocol freeze: {destination}")
    _assert_no_link_components(destination, allow_missing_leaf=False)
    if not destination.is_file() or _is_link_or_reparse(destination):
        raise V12R3P1SalvageFreezeError(
            f"protocol freeze is not a safe regular file: {destination}"
        )
    try:
        payload = forensic.strict_json_load(destination)
    except Exception as exc:
        raise V12R3P1SalvageFreezeError("protocol freeze is not strict JSON") from exc
    expected = _assemble_protocol_freeze(
        {
            "protocol_freeze_unoccupied": True,
            "execution_lock_absent": True,
            "new_run_root_absent": True,
        }
    )
    if payload != expected or destination.read_bytes() != forensic.canonical_json_bytes(
        expected
    ):
        raise V12R3P1SalvageFreezeError("protocol freeze drifted")
    return dict(payload)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    action = parser.add_mutually_exclusive_group(required=True)
    action.add_argument("--publish", action="store_true")
    action.add_argument("--verify", action="store_true")
    action.add_argument("--build-only", action="store_true")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    if args.publish:
        payload = publish_protocol_freeze()
    elif args.verify:
        payload = verify_protocol_freeze()
    else:
        payload = build_protocol_freeze()
    print(f"{payload['status']}: {contract.PROTOCOL_FREEZE_PATH.as_posix()}")
    return 0 if payload.get("passed") is True else 1


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
