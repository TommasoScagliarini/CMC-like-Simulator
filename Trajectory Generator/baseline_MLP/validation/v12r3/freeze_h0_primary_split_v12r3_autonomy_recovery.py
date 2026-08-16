# ruff: noqa: F821
"""Build and verify the additive, non-executable V12R3 protocol freeze.

The command re-audits the immutable V12/V11 evidence and may publish exactly
one no-clobber V12R3 protocol JSON.  It cannot fit an actor, load H0 for probe
inference, label a probe, reset/step an environment, or mint an execution
claim.  A distinct execution lock must pass the pure V12R3 lock gate first.
"""

from __future__ import annotations

import copy
import hashlib
import os
import stat
import sys
import types
from pathlib import Path, PurePosixPath
from typing import Any, Mapping, Sequence

_PARENT_VALIDATION = Path(__file__).resolve().parent.parent
if str(_PARENT_VALIDATION) not in sys.path:
    sys.path.insert(0, str(_PARENT_VALIDATION))

try:
    from validation import freeze_h0_primary_split_v12_autonomy_recovery as _v12_freeze
except (ImportError, ModuleNotFoundError):
    import freeze_h0_primary_split_v12_autonomy_recovery as _v12_freeze

try:
    from validation import h0_primary_split_v12r3_autonomy_recovery_contract as contract
except (ImportError, ModuleNotFoundError):
    import h0_primary_split_v12r3_autonomy_recovery_contract as contract


for _name, _value in vars(_v12_freeze).items():
    if (
        not _name.startswith("__")
        and _name != "contract"
        and not isinstance(_value, types.FunctionType)
    ):
        globals()[_name] = _value


class V12R3ProtocolFreezeError(RuntimeError):
    """Raised when the corrected design-only protocol cannot remain exact."""


# Inherited code refers to the old class name; bind it to the new public type.
V12ProtocolFreezeError = V12R3ProtocolFreezeError
V12_VALIDATION_ROOT = Path(__file__).resolve().parent


for _name, _value in vars(_v12_freeze).items():
    if (
        isinstance(_value, types.FunctionType)
        and _value.__module__ == _v12_freeze.__name__
    ):
        _function = types.FunctionType(
            _value.__code__,
            globals(),
            name=_value.__name__,
            argdefs=_value.__defaults__,
            closure=_value.__closure__,
        )
        _function.__kwdefaults__ = copy.deepcopy(_value.__kwdefaults__)
        _function.__annotations__ = copy.deepcopy(_value.__annotations__)
        _function.__doc__ = _value.__doc__
        globals()[_name] = _function


_inherited_path_isolation_audit = _path_isolation_audit


def _path_isolation_audit(
    declared_paths: Mapping[str, str | Path] | None = None,
) -> dict[str, Any]:
    """Extend V12 path isolation with real exclusive-temp filename budgets."""

    audit = dict(_inherited_path_isolation_audit(declared_paths))
    declared = audit["declared_mutation_paths"]
    file_temp_lengths = [
        len(path) + 14 for path in declared.values() if Path(path).suffix
    ]
    step_temp_lengths = [
        len(path) + 1 + len("000001.json") + 14
        for name, path in declared.items()
        if "steps_root" in name
    ]
    maximum = max(file_temp_lengths + step_temp_lengths)
    checks = dict(audit["checks"])
    checks["exclusive_temp_relative_paths_short"] = maximum < 160
    audit["checks"] = checks
    audit["passed"] = all(checks.values())
    audit["exclusive_temp_filename_overhead"] = 14
    audit["maximum_temp_relative_path_length"] = maximum
    audit["maximum_checkout_root_length_for_legacy_windows_with_temp"] = 258 - maximum
    audit["maximum_checkout_root_length_for_preferred_windows_with_temp"] = (
        238 - maximum
    )
    return audit


def _authoritative_hash_gate(
    sources: Mapping[str, Mapping[str, Any]],
    inputs: Mapping[str, Mapping[str, Any]],
) -> dict[str, Any]:
    observed: dict[str, str | None] = {}
    checks: dict[str, bool] = {}
    for name, expected in contract.AUTHORITATIVE_SHA256.items():
        record = sources.get(name) if name in sources else inputs.get(name)
        value = record.get("sha256") if isinstance(record, Mapping) else None
        observed[name] = value if isinstance(value, str) else None
        checks[name] = value == expected
    return {
        "passed": bool(checks) and all(checks.values()),
        "checks": checks,
        "expected_sha256": copy.deepcopy(contract.AUTHORITATIVE_SHA256),
        "observed_sha256": observed,
    }


def _authority_gate() -> dict[str, Any]:
    false_flags = (
        "pipeline_execution_authorized",
        "actor_fit_execution_authorized",
        "environment_reset_authorized",
        "environment_step_authorized",
        "design_audit_fit_authorized",
        "offline_teacher_labeling_authorized",
        "critic_updates_authorized",
        "ppo_updates_authorized",
        "protected_trial_access_authorized",
        "reserve_trial_access_authorized",
        "runtime_promotion_authorized",
        "physical_gate_relaxation_authorized",
        "primary_grf_modification_authorized",
        "detector_or_fsm_modification_authorized",
        "sea_semantic_modification_authorized",
        "retry_authorized",
        "sweep_authorized",
        "rescue_authorized",
    )
    expected_keys = {
        "authority_date",
        "authority_text",
        "authority_scope",
        "design_and_freeze_authorized",
        *false_flags,
    }
    checks = {
        "exact_schema": set(contract.AUTHORITY) == expected_keys,
        "identity": contract.AUTHORITY.get("authority_date") == "2026-08-09"
        and contract.AUTHORITY.get("authority_text") == contract.AUTHORITY_TEXT
        and contract.AUTHORITY.get("authority_scope")
        == contract.TRAINING_READINESS_SCOPE,
        "design_freeze_only_true_flag": contract.AUTHORITY.get(
            "design_and_freeze_authorized"
        )
        is True,
        "all_execution_and_relaxation_flags_false": all(
            contract.AUTHORITY.get(name) is False for name in false_flags
        ),
        "boolean_types_exact": type(
            contract.AUTHORITY.get("design_and_freeze_authorized")
        )
        is bool
        and all(type(contract.AUTHORITY.get(name)) is bool for name in false_flags),
    }
    return {
        "passed": all(checks.values()),
        "checks": checks,
        "false_flags": list(false_flags),
    }


def _v12_protocol_freeze_gate() -> dict[str, Any]:
    """Verify the parent freeze both cryptographically and semantically."""

    path = resolve_relative(contract.V12_PROTOCOL_FREEZE_PATH)
    record = _record(contract.V12_PROTOCOL_FREEZE_PATH)
    try:
        payload = _v12_freeze.verify_protocol_freeze()
        verifier_passed = payload.get("passed") is True
    except Exception as exc:  # pragma: no cover - surfaced in result
        payload = None
        verifier_passed = False
        error = f"{type(exc).__name__}: {exc}"
    else:
        error = None
    checks = {
        "regular_file": path.is_file() and not path.is_symlink(),
        "artifact_exact": record
        == {
            "path": contract.V12_PROTOCOL_FREEZE_PATH.as_posix(),
            "sha256": contract.V12_PROTOCOL_FREEZE_SHA256,
            "size_bytes": contract.V12_PROTOCOL_FREEZE_SIZE_BYTES,
        },
        "parent_verifier": verifier_passed,
        "parent_schema": isinstance(payload, Mapping)
        and payload.get("schema_version") == 120,
        "parent_identity": isinstance(payload, Mapping)
        and payload.get("protocol_id")
        == "AB06_H0_PRIMARY_SPLIT_V12_V26_AUTONOMY_RECOVERY",
        "parent_design_only": isinstance(payload, Mapping)
        and payload.get("execution_lock") is None
        and payload.get("environment_reset_calls") == 0
        and payload.get("environment_step_calls") == 0,
    }
    return {
        "passed": all(checks.values()),
        "checks": checks,
        "artifact": record,
        "verifier_error": error,
    }


def _v12r1_terminal_failure_gate() -> dict[str, Any]:
    """Bind the complete R1 attempt as terminal, non-reusable evidence."""

    expected = copy.deepcopy(contract.V12R1_LINEAGE_ARTIFACTS)
    observed = {name: _record(artifact["path"]) for name, artifact in expected.items()}
    claim = _mapping(expected["v12r1_pipeline_claim"]["path"])
    ledger = _mapping(expected["v12r1_pipeline_ledger"]["path"])
    run_prefix = "Trajectory Generator/baseline_MLP/validation/h0_v12r1_run_20260809/"
    expected_run_files = {
        artifact["path"]
        for artifact in expected.values()
        if artifact["path"].startswith(run_prefix)
    }
    run_root = resolve_relative(run_prefix.rstrip("/"))
    actual_run_files = {
        item.relative_to(REPO_ROOT).as_posix()
        for item in run_root.rglob("*")
        if item.is_file()
    }
    symlink_free = not any(item.is_symlink() for item in run_root.rglob("*"))
    new_paths = tuple(contract.declared_mutation_paths().values())
    checks = {
        "all_twelve_artifacts_exact": observed == expected and len(observed) == 12,
        "failed_run_file_set_exact": actual_run_files == expected_run_files
        and len(actual_run_files) == 9,
        "failed_run_symlink_free": symlink_free,
        "pipeline_claim_terminal_authority": (
            claim.get("status") == "CLAIM_H0_PRIMARY_SPLIT_V12R1_PIPELINE"
            and claim.get("passed") is True
            and claim.get("protocol_id")
            == "AB06_H0_PRIMARY_SPLIT_V12R1_V26_AUTONOMY_RECOVERY"
            and claim.get("retry_authorized") is False
            and claim.get("sweep_authorized") is False
            and claim.get("rescue_authorized") is False
        ),
        "terminal_failure_exact": (
            ledger.get("status") == "FAIL_H0_PRIMARY_SPLIT_V12R1_PIPELINE_TERMINAL"
            and ledger.get("passed") is False
            and ledger.get("attempted_stage") == "fit_p0"
            and ledger.get("completed_stages") == []
            and ledger.get("completed_receipts") == []
            and ledger.get("error")
            == {
                "type": "KeyError",
                "message": "'completed_v12r1_collection_rounds'",
            }
        ),
        "one_fit_attempt_only": (
            ledger.get("actor_fit_stage_calls_attempted") == 1
            and ledger.get("actor_updates_attempted") == 1
            and ledger.get("actor_fit_executions_confirmed") == 0
            and ledger.get("fit_stage_receipts_confirmed") == 0
            and ledger.get("actor_updates_confirmed") == 0
        ),
        "zero_environment_or_labels": (
            ledger.get("environment_reset_calls") == 0
            and ledger.get("environment_step_calls") == 0
            and ledger.get("environment_rollout_stages_confirmed") == 0
            and ledger.get("offline_teacher_label_stage_calls") == 0
            and ledger.get("offline_teacher_label_calls_confirmed") == 0
        ),
        "no_retry_rescue_sweep_or_trials": (
            ledger.get("retry_authorized") is False
            and ledger.get("rescue_authorized") is False
            and ledger.get("sweep_authorized") is False
            and ledger.get("protected_trials_opened") == []
            and ledger.get("reserve_trials_opened") == []
            and ledger.get("runtime_promoted") is False
        ),
        "r2_run_root_distinct": not contract.RUN_ROOT.as_posix().startswith(
            run_prefix.rstrip("/")
        ),
        "r2_mutations_do_not_reuse_r1": all(
            not path.as_posix().startswith(run_prefix.rstrip("/")) for path in new_paths
        ),
    }
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": (
            "PASS_H0_PRIMARY_SPLIT_V12R3_R1_TERMINAL_LINEAGE"
            if all(checks.values())
            else "FAIL_H0_PRIMARY_SPLIT_V12R3_R1_TERMINAL_LINEAGE"
        ),
        "passed": all(checks.values()),
        "protocol_id": contract.PROTOCOL_ID,
        "checks": checks,
        "artifacts": observed,
        "pipeline_claim": claim,
        "pipeline_ledger": ledger,
    }


def _v12r2_terminal_run_tree_record() -> tuple[dict[str, Any], bool]:
    """Recompute the compact terminal R2 tree without following symlinks."""

    expected = contract.V12R2_TERMINAL_RUN_TREE
    root = _lexical_absolute(expected["path"])
    try:
        root_status = os.lstat(root)
    except FileNotFoundError as exc:
        raise V12R3ProtocolFreezeError("terminal V12R2 run root is missing") from exc
    if not stat.S_ISDIR(root_status.st_mode):
        raise V12R3ProtocolFreezeError(
            "terminal V12R2 run root is not a real directory"
        )

    entries = sorted(root.rglob("*"))
    statuses = {item: os.lstat(item) for item in entries}
    symlink_free = not stat.S_ISLNK(root_status.st_mode) and all(
        not stat.S_ISLNK(value.st_mode) for value in statuses.values()
    )
    regular_or_directory_only = all(
        stat.S_ISREG(value.st_mode) or stat.S_ISDIR(value.st_mode)
        for value in statuses.values()
    )
    files = [item for item in entries if stat.S_ISREG(statuses[item].st_mode)]
    if not files:
        raise V12R3ProtocolFreezeError("terminal V12R2 run tree is empty")

    digest = hashlib.sha256()
    total_size_bytes = 0
    for item in files:
        relative = item.relative_to(root).as_posix()
        sha256 = forensic.sha256_file(item)
        size_bytes = int(statuses[item].st_size)
        total_size_bytes += size_bytes
        digest.update(relative.encode("utf-8"))
        digest.update(b"\0")
        digest.update(sha256.encode("ascii"))
        digest.update(b"\0")
        digest.update(str(size_bytes).encode("ascii"))
        digest.update(b"\n")
    record = {
        "path": root.relative_to(REPO_ROOT).as_posix(),
        "tree_sha256": digest.hexdigest(),
        "file_count": len(files),
        "total_size_bytes": total_size_bytes,
    }
    return record, symlink_free and regular_or_directory_only


def _v12r2_terminal_failure_gate() -> dict[str, Any]:
    """Bind the sole R2 attempt as terminal evidence that cannot be reused."""

    expected_artifacts = copy.deepcopy(contract.V12R2_LINEAGE_ARTIFACTS)
    observed_artifacts = {
        name: _record(artifact["path"]) for name, artifact in expected_artifacts.items()
    }
    artifact_paths = {
        name: _lexical_absolute(artifact["path"])
        for name, artifact in expected_artifacts.items()
    }
    artifact_paths_safe = all(
        stat.S_ISREG(os.lstat(path).st_mode) and not _symlink_ancestors(path)
        for path in artifact_paths.values()
    )
    run_tree, run_tree_safe = _v12r2_terminal_run_tree_record()

    claim = _mapping(expected_artifacts["v12r2_pipeline_claim"]["path"])
    ledger = _mapping(expected_artifacts["v12r2_pipeline_ledger"]["path"])
    expected_lock = expected_artifacts["v12r2_execution_lock"]
    expected_claim = expected_artifacts["v12r2_pipeline_claim"]
    expected_fit_receipt = expected_artifacts["v12r2_fit_p0_receipt"]
    expected_probe_failure = expected_artifacts["v12r2_probe_p0_failure"]
    observed_semantics = {
        name: copy.deepcopy(ledger.get(name))
        for name in contract.V12R2_TERMINAL_SEMANTICS
    }

    r2_root = PurePosixPath(contract.V12R2_TERMINAL_RUN_TREE["path"])

    def _under_r2_root(value: str | Path | PurePosixPath) -> bool:
        path = PurePosixPath(Path(value).as_posix())
        return path == r2_root or r2_root in path.parents

    declared_mutations = tuple(contract.declared_mutation_paths().values())
    execution_sources = tuple(contract.FUTURE_EXECUTION_SOURCE_RELATIVE_PATHS.values())
    r2_lineage_input_paths = {
        artifact["path"]
        for artifact in contract.V12R2_LINEAGE_ARTIFACTS.values()
        if _under_r2_root(artifact["path"])
    }
    observed_r2_input_paths = {
        Path(path).as_posix()
        for path in contract.INPUT_RELATIVE_PATHS.values()
        if _under_r2_root(path)
    }

    checks = {
        "all_seven_artifacts_exact": observed_artifacts == expected_artifacts
        and len(observed_artifacts) == 7,
        "all_artifact_paths_safe_regular": artifact_paths_safe,
        "terminal_run_tree_exact": run_tree == contract.V12R2_TERMINAL_RUN_TREE,
        "terminal_run_tree_symlink_free": run_tree_safe,
        "pipeline_claim_terminal_authority_exact": (
            claim.get("schema_version") == 122
            and claim.get("status") == "CLAIM_H0_PRIMARY_SPLIT_V12R2_PIPELINE"
            and claim.get("passed") is True
            and claim.get("protocol_id")
            == "AB06_H0_PRIMARY_SPLIT_V12R2_V26_AUTONOMY_RECOVERY"
            and claim.get("pipeline_id")
            == "H0_V12R2_V26_RECOVERY_WEIGHTED_PURE_PROBE_DAGGER"
            and claim.get("authority")
            == {
                "authority_date": "2026-08-09",
                "authority_text": (
                    "Fai tutti i passaggi che ti servono per arrivare ad essere "
                    "training ready. Non hai bisogno di autorizzazioni mie, "
                    "considera già tutto autorizzato."
                ),
                "authority_scope": "V12R2_ONE_SHOT_EXECUTION_READY",
                "one_shot": True,
            }
            and claim.get("execution_lock") == expected_lock
            and claim.get("actor_updates_authorized") == 4
            and claim.get("critic_updates") == 0
            and claim.get("ppo_updates") == 0
            and claim.get("retry_authorized") is False
            and claim.get("sweep_authorized") is False
            and claim.get("rescue_authorized") is False
            and claim.get("protected_trials_opened") == []
            and claim.get("reserve_trials_opened") == []
            and claim.get("stage_order") == list(contract.STAGE_IDS)
        ),
        "terminal_ledger_semantics_exact": (
            ledger.get("schema_version") == 122
            and ledger.get("status") == "FAIL_H0_PRIMARY_SPLIT_V12R2_PIPELINE_TERMINAL"
            and ledger.get("passed") is False
            and ledger.get("protocol_id")
            == "AB06_H0_PRIMARY_SPLIT_V12R2_V26_AUTONOMY_RECOVERY"
            and ledger.get("pipeline_id")
            == "H0_V12R2_V26_RECOVERY_WEIGHTED_PURE_PROBE_DAGGER"
            and observed_semantics == contract.V12R2_TERMINAL_SEMANTICS
            and ledger.get("completed_receipts")
            == [{"stage_id": "fit_p0", "receipt": expected_fit_receipt}]
            and ledger.get("execution_lock") == expected_lock
            and ledger.get("pipeline_claim") == expected_claim
            and ledger.get("stage_order") == list(contract.STAGE_IDS)
        ),
        "one_fit_one_probe_prefix_only": (
            ledger.get("actor_fit_stage_calls_attempted") == 1
            and ledger.get("actor_fit_executions_confirmed") == 1
            and ledger.get("fit_stage_receipts_confirmed") == 1
            and ledger.get("actor_updates_attempted") == 1
            and ledger.get("actor_update_receipts_confirmed") == 1
            and ledger.get("actor_updates_confirmed") == 1
            and ledger.get("environment_rollout_stages_confirmed") == 0
            and ledger.get("environment_reset_calls") == 1
            and ledger.get("environment_step_calls") == 232
            and ledger.get("offline_teacher_label_stage_calls") == 0
            and ledger.get("offline_teacher_label_stage_receipts_confirmed") == 0
            and ledger.get("offline_teacher_label_calls_confirmed") == 0
            and ledger.get("critic_updates") == 0
            and ledger.get("ppo_updates") == 0
        ),
        "probe_failure_receipt_exact": (
            isinstance(ledger.get("stage_activity"), list)
            and len(ledger["stage_activity"]) == 2
            and ledger["stage_activity"][0].get("stage_id") == "fit_p0"
            and ledger["stage_activity"][0].get("stage_kind") == "fit"
            and ledger["stage_activity"][1].get("stage_id") == "probe_p0"
            and ledger["stage_activity"][1].get("stage_kind") == "probe"
            and ledger["stage_activity"][1].get("terminal_failure")
            == {
                "already_present": False,
                "applicable": True,
                "artifact": expected_probe_failure,
                "published": True,
                "verified": True,
            }
        ),
        "terminal_no_retry_rescue_sweep_or_trials": (
            ledger.get("retry_authorized") is False
            and ledger.get("sweep_authorized") is False
            and ledger.get("rescue_authorized") is False
            and ledger.get("protected_trials_opened") == []
            and ledger.get("reserve_trials_opened") == []
            and ledger.get("runtime_promoted") is False
        ),
        "r3_run_root_distinct": not _under_r2_root(contract.RUN_ROOT),
        "r3_mutations_do_not_reuse_r2": all(
            not _under_r2_root(path) for path in declared_mutations
        ),
        "r3_execution_sources_do_not_reuse_r2": all(
            not _under_r2_root(path) for path in execution_sources
        ),
        "r2_inputs_are_lineage_evidence_only": (
            observed_r2_input_paths == r2_lineage_input_paths
        ),
        "source_h0_and_seed_corpus_do_not_reuse_r2": (
            not _under_r2_root(contract.SOURCE_H0_MODULE_PATH)
            and not _under_r2_root(contract.V11_P3_CORPUS_PATH)
        ),
    }
    lineage = {
        "artifacts": observed_artifacts,
        "run_tree": run_tree,
        "semantics": observed_semantics,
    }
    passed = all(checks.values()) and lineage == contract.V12R2_TERMINAL_LINEAGE
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": (
            "PASS_H0_PRIMARY_SPLIT_V12R3_R2_TERMINAL_LINEAGE"
            if passed
            else "FAIL_H0_PRIMARY_SPLIT_V12R3_R2_TERMINAL_LINEAGE"
        ),
        "passed": passed,
        "protocol_id": contract.PROTOCOL_ID,
        "checks": checks,
        "lineage": lineage,
        "pipeline_claim": claim,
        "pipeline_ledger": ledger,
    }


def _assemble_protocol_freeze(occupancy: Mapping[str, bool]) -> dict[str, Any]:
    expected_occupancy_keys = {
        "protocol_freeze_unoccupied",
        "execution_lock_absent",
        "design_audit_absent",
        "run_root_absent",
    }
    if set(occupancy) != expected_occupancy_keys or not all(
        type(value) is bool for value in occupancy.values()
    ):
        raise V12R3ProtocolFreezeError("occupancy evidence schema drifted")

    v11_receipt_before = repository_artifact_snapshot(
        contract.prior.DESIGN_AUDIT_RECEIPT_PATH
    )
    sources = _source_records()
    inputs = _input_records()
    hashes = _authoritative_hash_gate(sources, inputs)
    authority = _authority_gate()
    topology = _stage_topology_gate()
    replay = contract.replay_schema_gate(contract.PROBE_REPLAY_SCHEMA)
    fit_contract = contract.fit_contract_self_check()
    parent_v12 = _v12_protocol_freeze_gate()
    parent_v12r1 = _v12r1_terminal_failure_gate()
    parent_v12r2 = _v12r2_terminal_failure_gate()
    v11_fit_provenance = _v11_fit_and_freeze_provenance_audit()
    terminal = _v11_terminal_gate()
    collection = _v11_collection_recovery_audit()
    corpus = _v11_corpus_audit()
    pure_failure = _v11_pure_failure_coverage_audit()
    isolation = _path_isolation_audit()
    v11_receipt_after = repository_artifact_snapshot(
        contract.prior.DESIGN_AUDIT_RECEIPT_PATH
    )
    design_gate_names = (
        "fit_gate",
        "probe_integrity_gate",
        "pure_probe_gate",
        "observer_label_gate",
        "collection_data_gate",
        "latch_dependence_gate",
        "candidate_freeze_gate",
        "final_rollout_gate",
        "final_development_gate",
        "replay_schema_gate",
        "replay_event_topology_gate",
        "p0_reproduction_gate",
        "design_audit_gate",
        "execution_lock_gate",
        "fit_contract_self_check",
    )
    checks = {
        "authority_design_freeze_only": authority["passed"] is True,
        "immutable_v12_protocol_freeze": parent_v12["passed"] is True,
        "stage_topology_exact": topology["passed"] is True,
        "corrected_teacher_columns_10_through_24": (
            contract.PROBE_REPLAY_SCHEMA.get("teacher_mutable_actor_columns")
            == list(range(10, 25))
        ),
        "self_contained_replay_schema": replay["passed"] is True,
        "fit_contract_wire_schema_total": fit_contract["passed"] is True,
        "v12r1_terminal_failure_bound": parent_v12r1["passed"] is True,
        "v12r2_terminal_failure_bound": parent_v12r2["passed"] is True,
        "p0_reproduction_tolerance_frozen": contract.P0_REPRODUCTION_TOLERANCE
        == {
            "absolute": 1.0e-9,
            "relative": 1.0e-9,
            "rule": "ABS_OBSERVED_MINUS_REFERENCE_LE_ABS_PLUS_REL_TIMES_ABS_REFERENCE",
            "applies_to": ["rmse", "max_abs_error", "reset_max_abs_error"],
            "reference": "SOLE_V12R3_DESIGN_AUDIT_P0_IN_MEMORY_FIT",
            "retry_or_retuning": False,
        },
        "pure_design_gates_present": all(
            callable(getattr(contract, name, None)) for name in design_gate_names
        ),
        "authoritative_hashes": hashes["passed"] is True,
        "v11_fit_and_freeze_provenance": v11_fit_provenance["passed"] is True,
        "v11_terminal": terminal["passed"] is True,
        "v11_collection_evidence": collection["passed"] is True,
        "v11_seed_corpus": corpus["passed"] is True,
        "v11_pure_failure_coverage": pure_failure["passed"] is True,
        "write_paths_isolated": isolation["passed"] is True,
        "v11_design_receipt_idempotent": v11_receipt_before == v11_receipt_after,
        "future_execution_sources_deferred_exact": (
            contract.FUTURE_EXECUTION_SOURCES_REQUIRED
            == tuple(contract.FUTURE_EXECUTION_SOURCE_RELATIVE_PATHS)
            and len(contract.FUTURE_EXECUTION_SOURCES_REQUIRED) == 7
        ),
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
        "checks": checks,
        "authority": copy.deepcopy(contract.AUTHORITY),
        "authority_gate": authority,
        "immutable_v12_protocol_freeze": parent_v12,
        "stage_order": list(contract.STAGE_IDS),
        "stage_topology_gate": topology,
        "fit_design": copy.deepcopy(contract.FIT),
        "p0_reproduction_tolerance": copy.deepcopy(contract.P0_REPRODUCTION_TOLERANCE),
        "recovery_weighting": copy.deepcopy(contract.RECOVERY_WEIGHTING),
        "coverage_weighting": copy.deepcopy(contract.COVERAGE_WEIGHTING),
        "latch_independence": copy.deepcopy(contract.LATCH_INDEPENDENCE),
        "probe_behavior": contract.PROBE_BEHAVIOR,
        "probe_replay_schema": copy.deepcopy(contract.PROBE_REPLAY_SCHEMA),
        "probe_replay_schema_gate": replay,
        "fit_contract_self_check": fit_contract,
        "v12r1_terminal_failure_lineage": parent_v12r1,
        "v12r2_terminal_failure_lineage": parent_v12r2,
        "authoritative_hashes": hashes,
        "v11_fit_and_freeze_provenance_audit": v11_fit_provenance,
        "v11_terminal_evidence": terminal,
        "v11_collection_recovery_audit": collection,
        "v11_seed_corpus_audit": corpus,
        "v11_pure_failure_coverage_audit": pure_failure,
        "write_path_isolation": isolation,
        "sources": sources,
        "inputs": inputs,
        "v11_design_receipt_snapshot_before": v11_receipt_before,
        "v11_design_receipt_snapshot_after": v11_receipt_after,
        "future_execution_sources_required": list(
            contract.FUTURE_EXECUTION_SOURCES_REQUIRED
        ),
        "future_execution_source_paths": {
            name: contract.FUTURE_EXECUTION_SOURCE_RELATIVE_PATHS[name]
            for name in contract.FUTURE_EXECUTION_SOURCES_REQUIRED
        },
        "execution_lock": None,
        "pipeline_claim": None,
        "environment_reset_calls": 0,
        "environment_step_calls": 0,
        "offline_teacher_label_calls": 0,
        "actor_fit_executions": 0,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
        "next_stage": "REQUIRE_SEPARATE_V12R3_NO_CLOBBER_EXECUTION_LOCK",
    }


def verify_protocol_freeze(
    *,
    input_path: str | os.PathLike[str] | None = None,
    enforce_canonical_destination: bool = True,
) -> dict[str, Any]:
    """Verify the original snapshot while allowing ordered later occupancy."""

    destination = _lexical_absolute(
        contract.PROTOCOL_FREEZE_PATH if input_path is None else input_path
    )
    canonical = _lexical_absolute(contract.PROTOCOL_FREEZE_PATH)
    if enforce_canonical_destination and destination != canonical:
        raise V12R3ProtocolFreezeError(f"non-canonical protocol freeze: {destination}")
    snapshot = artifact_snapshot(destination)
    if snapshot.get("kind") != "regular" or _symlink_ancestors(destination):
        raise V12R3ProtocolFreezeError(
            "V12R3 protocol freeze is not a regular safe path"
        )
    if enforce_canonical_destination:
        design = os.path.lexists(_lexical_absolute(contract.DESIGN_AUDIT_RECEIPT_PATH))
        lock = os.path.lexists(_lexical_absolute(contract.EXECUTION_LOCK_PATH))
        run_root = os.path.lexists(_lexical_absolute(contract.RUN_ROOT))
        claim = os.path.lexists(_lexical_absolute(contract.PIPELINE_CLAIM_PATH))
        invalid_order = (
            (lock and not design) or (run_root and not lock) or (claim and not run_root)
        )
        if invalid_order:
            raise V12R3ProtocolFreezeError(
                "V12R3 later occupancy is out of readiness order"
            )
    expected = _assemble_protocol_freeze(
        {
            "protocol_freeze_unoccupied": True,
            "execution_lock_absent": True,
            "design_audit_absent": True,
            "run_root_absent": True,
        }
    )
    try:
        payload = forensic.strict_json_load(destination)
    except Exception as exc:
        raise V12R3ProtocolFreezeError(
            "V12R3 protocol freeze is not strict JSON"
        ) from exc
    if payload != expected or destination.read_bytes() != forensic.canonical_json_bytes(
        expected
    ):
        raise V12R3ProtocolFreezeError("V12R3 protocol freeze drifted")
    return dict(payload)


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    payload = prepare_protocol_freeze() if args.prepare else verify_protocol_freeze()
    print(f"{payload['status']}: {contract.PROTOCOL_FREEZE_PATH.as_posix()}")
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
