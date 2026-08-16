"""One-shot V12R10 import-only recovery and pure-development runner.

V12R10 never reopens the terminal V12R9 lineage.  It semantically attests the
closed R9 terminal/corpus/labels, performs the sole frozen W1024 actor fit,
freezes that exact output, and runs six 500-step pure-policy developments in
hardest-first order.  The first failure publishes an immutable terminal ledger
and closes the lineage without retry or resume authority.
"""

from __future__ import annotations

import argparse
import copy
import json
import os
import sys
from dataclasses import dataclass
from pathlib import Path, PurePath
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
    LOCAL_VALIDATION / "v12r6",
    LOCAL_VALIDATION / "v12r9",
    REVISION_ROOT,
):
    if str(_root) not in sys.path:
        sys.path.insert(0, str(_root))

import freeze_h0_v12r10_recovery as freezer  # noqa: E402
import h0_forensic_rollout as forensic  # noqa: E402
import h0_v12r6_physical_development as physical  # noqa: E402
import h0_v12r9_prefix_adjudicator as r9_adjudicator  # noqa: E402
import h0_v12r10_recovery_contract as contract  # noqa: E402
import h0_v12r10_recovery_fitter as fitter  # noqa: E402


class V12R10ExecutionError(RuntimeError):
    """Raised after a V12R10 fail-closed orchestration violation."""


EXPECTED_CANDIDATE_FILES = {
    "actor_feature_manifest.json",
    "candidate_build_manifest.json",
    "class_and_ctor_args.pkl",
    "metadata.json",
    "module_state.pkl",
}
ACTIVITY_FIELDS = (
    "r9_import_attestation_calls",
    "actor_fit_stage_calls",
    "actor_updates",
    "uniform_adamw_epochs_completed",
    "uniform_lbfgs_closure_calls",
    "gate_adamw_epochs_completed",
    "gate_lbfgs_closure_calls",
    "development_rollouts_completed",
    "environment_reset_calls",
    "environment_step_calls",
    "development_environment_reset_calls",
    "development_environment_step_calls",
    "raw_sensor_sample_count",
    "teacher_query_count",
    "served_action_teacher_dependency_count",
    "mean_blend_count",
    "safety_intervention_count",
    "safety_latch_activation_count",
    "safety_latch_release_count",
    "critic_updates",
    "ppo_updates",
)

R9_ATTESTATION_FIELDS = {
    "schema_version",
    "status",
    "passed",
    "protocol_id",
    "pipeline_id",
    "stage_id",
    "r9_terminal_ledger",
    "r9_corpus",
    "r9_source_attestation",
    "pipeline_claim",
    "worker_claim",
    "import_only",
    "new_collection_rollout_count",
    "environment_reset_calls",
    "environment_step_calls",
    "actor_updates",
    "critic_updates",
    "ppo_updates",
}
CANDIDATE_FREEZE_FIELDS = {
    "schema_version",
    "status",
    "passed",
    "protocol_id",
    "pipeline_id",
    "stage_id",
    "candidate_selection_rule",
    "candidate_id",
    "candidate_module",
    "fit_receipt",
    "actor_feature_manifest",
    "actor_digest",
    "candidate_build_manifest",
    "semantic_audit",
    "pipeline_claim",
    "worker_claim",
    "candidate_frozen",
    "fit_gate_passed",
    "standard_actor",
    "warm_start_target_1024_compatible",
    "r6_functional_predecessor",
    "r9_hidden_initialization_only",
    "actor_fit_count",
    "runtime_promoted",
    "qualification_executed",
    "checkpoint_zero_created",
    "positive_morphology_enabled",
    "q3_paths_opened",
    "actor_updates",
    "critic_updates",
    "ppo_updates",
}
DEVELOPMENT_RECEIPT_FIELDS = {
    "schema_version",
    "status",
    "passed",
    "protocol_id",
    "pipeline_id",
    "stage_id",
    "case_id",
    "candidate_id",
    "candidate_module",
    "candidate_freeze",
    "raw_summary",
    "summary",
    "gate",
    "trace",
    "pure_policy_trace_audit",
    "runtime_integrity_checks",
    "environment_reset_calls",
    "environment_step_calls",
    "raw_sensor_sample_count",
    "teacher_query_count",
    "served_action_teacher_dependency_count",
    "mean_blend_count",
    "safety_intervention_count",
    "safety_latch_activation_count",
    "safety_latch_release_count",
    "actor_updates",
    "critic_updates",
    "ppo_updates",
}
TERMINAL_LEDGER_FIELDS = {
    "schema_version",
    "status",
    "passed",
    "terminal",
    "protocol_id",
    "pipeline_id",
    "candidate_selection_rule",
    "candidate_id",
    "candidate_module",
    "candidate_module_observation",
    "stage_order",
    "completed_stages",
    "completed_stage_count",
    "attempted_stage",
    "activity_totals",
    "import_attestation_count",
    "collection_round_count",
    "actor_fit_count",
    "development_count",
    "protocol_freeze",
    "execution_lock",
    "pipeline_claim",
    "r9_import_attestation",
    "candidate_freeze",
    "final_development_receipt",
    "qualification_executed",
    "runtime_promoted",
    "checkpoint_zero_created",
    "positive_morphology_enabled",
    "q3_paths_opened",
    "actor_updates",
    "critic_updates",
    "ppo_updates",
    "environment_reset_calls",
    "environment_step_calls",
    "development_environment_reset_calls",
    "development_environment_step_calls",
    "raw_sensor_sample_count",
    "teacher_query_count",
    "served_action_teacher_dependency_count",
    "mean_blend_count",
    "safety_intervention_count",
    "safety_latch_activation_count",
    "safety_latch_release_count",
    "retry_authorized",
    "resume_authorized",
    "fit_sweep_authorized",
    "error",
    "next_stage",
}


def _path(value: str | PurePath | Path) -> Path:
    if isinstance(value, Path) and value.is_absolute():
        target = Path(os.path.abspath(value))
        try:
            relative = target.relative_to(REPO_ROOT)
        except ValueError as exc:
            raise V12R10ExecutionError(f"path escaped repository: {target}") from exc
        raw = relative.as_posix()
    else:
        raw = value.as_posix() if isinstance(value, PurePath) else os.fspath(value)
    try:
        return freezer.safe_repository_path(raw)
    except Exception as exc:
        raise V12R10ExecutionError(f"unsafe repository path: {raw!r}") from exc


def _logical(path: str | PurePath | Path) -> str:
    return _path(path).relative_to(REPO_ROOT).as_posix()


def _record(path: str | PurePath | Path) -> dict[str, Any]:
    try:
        return freezer.artifact_record(_logical(path))
    except Exception as exc:
        raise V12R10ExecutionError(f"invalid artifact: {path}") from exc


def _tree(path: str | PurePath | Path) -> dict[str, Any]:
    try:
        return freezer.tree_record(_logical(path))
    except Exception as exc:
        raise V12R10ExecutionError(f"invalid artifact tree: {path}") from exc


def _mapping(path: str | PurePath | Path) -> dict[str, Any]:
    target = _path(path)
    try:
        _record(target)
        value = forensic.strict_json_load(target)
    except Exception as exc:
        raise V12R10ExecutionError(f"invalid strict JSON: {target}") from exc
    if not isinstance(value, Mapping):
        raise V12R10ExecutionError(f"expected JSON object: {target}")
    return dict(value)


def _sequence(path: str | PurePath | Path) -> list[Any]:
    target = _path(path)
    try:
        _record(target)
        value = forensic.strict_json_load(target)
    except Exception as exc:
        raise V12R10ExecutionError(f"invalid strict JSON: {target}") from exc
    if not isinstance(value, list):
        raise V12R10ExecutionError(f"expected JSON array: {target}")
    return value


def _write(
    path: str | PurePath | Path,
    payload: Mapping[str, Any] | Sequence[Any],
) -> dict[str, Any]:
    target = _path(path)
    mutation_root = _path(contract.ROOT)
    try:
        target.relative_to(mutation_root)
    except ValueError as exc:
        raise V12R10ExecutionError(
            "R10 runner mutation escaped the exclusive R10 namespace"
        ) from exc
    forensic.write_json_exclusive(target, payload)
    return _record(target)


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


def _strict_schema(value: Mapping[str, Any], fields: set[str]) -> bool:
    return bool(
        set(value) == fields
        and type(value.get("schema_version")) is int
        and value.get("schema_version") == contract.SCHEMA_VERSION
    )


def _zero_int(value: Any) -> bool:
    return type(value) is int and value == 0


def _zero_activity() -> dict[str, int]:
    return {name: 0 for name in ACTIVITY_FIELDS}


def _increment(activity: dict[str, int], name: str, amount: int = 1) -> None:
    if name not in activity:
        raise V12R10ExecutionError(f"unknown activity counter: {name}")
    if type(amount) is not int or amount < 0:
        raise V12R10ExecutionError("activity increment must be non-negative int")
    current = activity[name]
    if type(current) is not int or current < 0:
        raise V12R10ExecutionError("activity counter drifted")
    activity[name] = current + amount


def _increment_development(
    activity: dict[str, int], name: str, amount: int = 1
) -> None:
    _increment(activity, name, amount)
    if name in {"environment_reset_calls", "environment_step_calls"}:
        _increment(activity, f"development_{name}", amount)


def _verify_r9_inputs() -> dict[str, Any]:
    """Close byte-locked R9 imports without an H0 query or environment."""

    locked = fitter.attest_locked_inputs()
    terminal = _mapping(contract.R9_TERMINAL_LEDGER_PATH)
    if (
        terminal.get("status") != contract.R9_TERMINAL_FAIL_STATUS
        or terminal.get("passed") is not False
        or terminal.get("terminal") is not True
        or terminal.get("attempted_stage") != contract.R9_TERMINAL_STAGE
        or terminal.get("next_stage") != "STOP_TERMINAL_NO_RETRY"
        or terminal.get("retry_authorized") is not False
        or terminal.get("resume_authorized") is not False
    ):
        raise V12R10ExecutionError("R9 terminal FAIL attestation drifted")
    expected_candidate = {
        key: copy.deepcopy(value)
        for key, value in contract.R9_TERMINAL_CANDIDATE_TREE.items()
        if key != "role"
    }
    observed_candidate = _tree(contract.R9_CANDIDATE_MODULE_PATH)
    bundle = fitter.load_locked_r9_corpus()
    arrays = bundle.arrays
    rows = len(arrays["observations"])
    label_records: dict[str, Any] = {}
    label_rows: dict[str, int] = {}
    for case_id, expected in contract.IMPORTED_LABEL_RECORDS.items():
        record = _record(expected["path"])
        expected_record = {
            key: value for key, value in expected.items() if key != "rows"
        }
        if not _strict_equal(record, expected_record):
            raise V12R10ExecutionError(f"R9 label record drifted: {case_id}")
        count = int(len(bundle.observer_indices[case_id]))
        if count != expected["rows"]:
            raise V12R10ExecutionError(f"R9 label row closure drifted: {case_id}")
        label_records[case_id] = record
        label_rows[case_id] = count
    if (
        rows != contract.R9_TERMINAL_EVIDENCE["corpus"]["rows"]
        or len(set(arrays["stratum_ids"].astype(str))) != 13
        or not _strict_equal(observed_candidate, expected_candidate)
        or not _strict_equal(terminal.get("candidate_module"), observed_candidate)
        or sum(label_rows.values()) != 2_431
    ):
        raise V12R10ExecutionError("R9 corpus/label semantic closure drifted")
    return {
        "passed": True,
        "status": "PASS_H0_V12R10_R9_TERMINAL_IMPORT_SOURCE_ATTESTATION",
        "r9_terminal_status": terminal["status"],
        "r9_terminal_attempted_stage": terminal["attempted_stage"],
        "r9_terminal_completed_stage_count": terminal["completed_stage_count"],
        "r9_terminal_ledger": _record(contract.R9_TERMINAL_LEDGER_PATH),
        "r9_terminal_candidate": observed_candidate,
        "r9_corpus": _record(contract.R9_CORPUS_PATH),
        "r9_corpus_row_count": rows,
        "r9_corpus_stratum_count": 13,
        "r9_observer_label_records": label_records,
        "r9_observer_label_rows": label_rows,
        "locked_input_attestation": locked,
        "offline_h0_query_count": 0,
        "import_only": True,
        "new_collection_rollout_count": 0,
        "environment_reset_calls": 0,
        "environment_step_calls": 0,
    }


@dataclass(frozen=True)
class RunnerAdapters:
    """Injectable boundaries used by orchestration tests, never by evidence."""

    verify_r9_inputs: Callable[[], Mapping[str, Any]] = _verify_r9_inputs
    run_fit: Callable[..., Mapping[str, Any]] = fitter.run_fit_stage
    verify_fit: Callable[[], Mapping[str, Any]] = fitter.verify_fit_stage
    run_physical: Callable[..., Mapping[str, Any]] = physical.run_case


def _claims_root() -> Path:
    return _path(contract.RUN_ROOT) / "claims"


def _stage_claim_path(stage_id: str) -> Path:
    try:
        index = tuple(contract.STAGE_IDS).index(stage_id) + 1
    except ValueError as exc:
        raise V12R10ExecutionError(f"unknown stage id: {stage_id}") from exc
    if "/" in stage_id or "\\" in stage_id or ".." in stage_id:
        raise V12R10ExecutionError("unsafe stage identifier")
    return _claims_root() / f"{index:02d}_{stage_id}.json"


def _expected_pipeline_claim() -> dict[str, Any]:
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": "CLAIMED_H0_V12R10_RECOVERY_PIPELINE_ONCE",
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "authority": copy.deepcopy(contract.AUTHORITY),
        "stage_order": list(contract.STAGE_IDS),
        "protocol_freeze": _record(contract.PROTOCOL_FREEZE_PATH),
        "execution_lock": _record(contract.EXECUTION_LOCK_PATH),
        "one_shot": True,
        "import_only": True,
        "new_collection_rollout_count": 0,
        "single_actor_fit": True,
        "retry_authorized": False,
        "resume_authorized": False,
        "fit_sweep_authorized": False,
        "qualification_execution_authorized": False,
    }


def verify_pipeline_claim() -> dict[str, Any]:
    claim = _mapping(contract.CLAIM_PATH)
    if not _strict_equal(claim, _expected_pipeline_claim()):
        raise V12R10ExecutionError("pipeline claim semantic drifted")
    return claim


def _expected_worker_claim(stage_id: str) -> dict[str, Any]:
    if stage_id not in contract.STAGE_IDS:
        raise V12R10ExecutionError(f"unknown stage id: {stage_id}")
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": "CLAIMED_H0_V12R10_STAGE_ONCE",
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "stage_id": stage_id,
        "pipeline_claim": _record(contract.CLAIM_PATH),
        "protocol_freeze": _record(contract.PROTOCOL_FREEZE_PATH),
        "execution_lock": _record(contract.EXECUTION_LOCK_PATH),
        "one_shot": True,
        "retry_authorized": False,
        "resume_authorized": False,
        "fit_sweep_authorized": False,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
    }


def verify_worker_claim(stage_id: str) -> dict[str, Any]:
    verify_pipeline_claim()
    claim = _mapping(_stage_claim_path(stage_id))
    if not _strict_equal(claim, _expected_worker_claim(stage_id)):
        raise V12R10ExecutionError(f"worker claim semantic drifted: {stage_id}")
    return claim


def verify_claim_directory(stage_ids: Sequence[str]) -> dict[str, Any]:
    expected_ids = list(stage_ids)
    if len(expected_ids) != len(set(expected_ids)) or any(
        stage_id not in contract.STAGE_IDS for stage_id in expected_ids
    ):
        raise V12R10ExecutionError("claim directory stage set drifted")
    root = _claims_root()
    if not root.is_dir() or root.is_symlink():
        raise V12R10ExecutionError("claims root is absent or unsafe")
    expected_names = {_stage_claim_path(stage_id).name for stage_id in expected_ids}
    entries = list(root.iterdir())
    if {entry.name for entry in entries} != expected_names or any(
        not entry.is_file() or entry.is_symlink() for entry in entries
    ):
        raise V12R10ExecutionError("claims directory contains missing or extra entries")
    for stage_id in expected_ids:
        verify_worker_claim(stage_id)
    return {"passed": True, "stage_ids": expected_ids}


def _claim_stage(stage_id: str) -> dict[str, Any]:
    payload = _expected_worker_claim(stage_id)
    _write(_stage_claim_path(stage_id), payload)
    return verify_worker_claim(stage_id)


def _development_receipt_path(case_id: str) -> Path:
    contract.canonical_development_case(case_id)
    return _path(contract.DEVELOPMENT_ROOT) / case_id / "receipt.json"


def _run_r9_attestation(
    activity: dict[str, int], adapters: RunnerAdapters
) -> dict[str, Any]:
    stage_id = "attest_r9_terminal_imports"
    evidence = dict(adapters.verify_r9_inputs())
    if (
        evidence.get("passed") is not True
        or evidence.get("import_only") is not True
        or evidence.get("new_collection_rollout_count") != 0
        or evidence.get("environment_reset_calls") != 0
        or evidence.get("environment_step_calls") != 0
    ):
        raise V12R10ExecutionError("R9 import source attestation failed")
    _increment(activity, "r9_import_attestation_calls")
    receipt = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": "PASS_H0_V12R10_R9_TERMINAL_AND_IMPORTS_ATTESTATION",
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "stage_id": stage_id,
        "r9_terminal_ledger": _record(contract.R9_TERMINAL_LEDGER_PATH),
        "r9_corpus": _record(contract.R9_CORPUS_PATH),
        "r9_source_attestation": evidence,
        "pipeline_claim": _record(contract.CLAIM_PATH),
        "worker_claim": _record(_stage_claim_path(stage_id)),
        "import_only": True,
        "new_collection_rollout_count": 0,
        "environment_reset_calls": 0,
        "environment_step_calls": 0,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
    }
    _write(contract.R9_IMPORT_ATTESTATION_PATH, receipt)
    return verify_r9_attestation_receipt(source_verifier=adapters.verify_r9_inputs)


def verify_r9_attestation_receipt(
    *, source_verifier: Callable[[], Mapping[str, Any]] | None = None
) -> dict[str, Any]:
    verify_source = _verify_r9_inputs if source_verifier is None else source_verifier
    expected_evidence = dict(verify_source())
    stage_id = "attest_r9_terminal_imports"
    verify_worker_claim(stage_id)
    receipt = _mapping(contract.R9_IMPORT_ATTESTATION_PATH)
    checks = {
        "schema": _strict_schema(receipt, R9_ATTESTATION_FIELDS),
        "identity": receipt.get("status")
        == "PASS_H0_V12R10_R9_TERMINAL_AND_IMPORTS_ATTESTATION"
        and receipt.get("passed") is True
        and receipt.get("protocol_id") == contract.PROTOCOL_ID
        and receipt.get("pipeline_id") == contract.PIPELINE_ID
        and receipt.get("stage_id") == stage_id,
        "sources": receipt.get("r9_terminal_ledger")
        == _record(contract.R9_TERMINAL_LEDGER_PATH)
        and receipt.get("r9_corpus") == _record(contract.R9_CORPUS_PATH)
        and _strict_equal(receipt.get("r9_source_attestation"), expected_evidence),
        "claims": receipt.get("pipeline_claim") == _record(contract.CLAIM_PATH)
        and receipt.get("worker_claim") == _record(_stage_claim_path(stage_id)),
        "import_only": receipt.get("import_only") is True
        and receipt.get("new_collection_rollout_count") == 0,
        "zero_activity": all(
            _zero_int(receipt.get(name))
            for name in (
                "environment_reset_calls",
                "environment_step_calls",
                "actor_updates",
                "critic_updates",
                "ppo_updates",
            )
        ),
    }
    if not all(value is True for value in checks.values()):
        failed = sorted(name for name, value in checks.items() if not value)
        raise V12R10ExecutionError(f"R9 attestation receipt drifted: {failed}")
    return receipt


def _run_fit(activity: dict[str, int], adapters: RunnerAdapters) -> dict[str, Any]:
    stage_id = "fit_recovery_actor"
    _increment(activity, "actor_fit_stage_calls")
    receipt = dict(
        adapters.run_fit(
            pipeline_claim_path=_path(contract.CLAIM_PATH),
            worker_claim_path=_stage_claim_path(stage_id),
            protocol_freeze_path=_path(contract.PROTOCOL_FREEZE_PATH),
            execution_lock_path=_path(contract.EXECUTION_LOCK_PATH),
            activity_callback=lambda name, amount: _increment(activity, name, amount),
        )
    )
    if receipt.get("passed") is not True:
        raise V12R10ExecutionError("R10 actor fit receipt is not PASS")
    verified = dict(adapters.verify_fit())
    if not _strict_equal(receipt, verified):
        raise V12R10ExecutionError("R10 actor fit verifier disagrees")
    return verified


def _candidate_semantic_audit(
    module: Mapping[str, Any],
    actor_manifest: Mapping[str, Any],
    build_manifest: Mapping[str, Any],
) -> dict[str, Any]:
    file_names = {
        row.get("path") for row in module.get("files", []) if isinstance(row, Mapping)
    }
    provenance = build_manifest.get("source_provenance")
    if not isinstance(provenance, Mapping):
        provenance = build_manifest.get("provenance")
    provenance = provenance if isinstance(provenance, Mapping) else {}
    checks = {
        "five_file_tree": type(module.get("file_count")) is int
        and module.get("file_count") == 5
        and file_names == EXPECTED_CANDIDATE_FILES,
        "actor_manifest": actor_manifest.get("status")
        == contract.ACTOR_FEATURE_MANIFEST_STATUS
        and actor_manifest.get("topology_id") == contract.TOPOLOGY_ID
        and actor_manifest.get("fit_contract_id") == contract.FIT_CONTRACT_ID
        and actor_manifest.get("actor_feature_count")
        == contract.EXPECTED_ACTOR_FEATURES
        and actor_manifest.get("fcnet_hiddens") == [1024, 1024]
        and actor_manifest.get("disabled_clock_columns")
        == list(contract.DISABLED_CLOCK_COLUMNS)
        and actor_manifest.get("standard_rlmodule") is True
        and actor_manifest.get("legacy_shadow_runtime_dependency") is False,
        "build_manifest": build_manifest.get("passed") is True
        and build_manifest.get("protocol_id") == contract.PROTOCOL_ID
        and build_manifest.get("fit_contract_id") == contract.FIT_CONTRACT_ID
        and build_manifest.get("topology_id") == contract.TOPOLOGY_ID
        and build_manifest.get("architecture") == contract.FIT["architecture"],
        "single_update": type(build_manifest.get("actor_fit_count")) is int
        and build_manifest.get("actor_fit_count") == 1
        and type(build_manifest.get("actor_updates")) is int
        and build_manifest.get("actor_updates") == 1
        and _zero_int(build_manifest.get("critic_updates"))
        and _zero_int(build_manifest.get("ppo_updates")),
        "preservation": build_manifest.get("logstd_byte_exact") is True
        and build_manifest.get("disabled_clock_columns_bit_zero") is True
        and build_manifest.get("save_reload_exact") is True
        and build_manifest.get("tower_a_r6_byte_exact") is True
        and build_manifest.get("cross_blocks_positive_zero") is True,
        "digest_bound": actor_manifest.get("actor_digest")
        == build_manifest.get("actor_digest")
        and actor_manifest.get("module_state_sha256")
        == build_manifest.get("module_state_sha256"),
        "provenance": (
            build_manifest.get("r6_functional_predecessor") is True
            or provenance.get("r6_functional_predecessor") is True
        )
        and (
            build_manifest.get("r9_hidden_initialization_only") is True
            or provenance.get("r9_hidden_initialization_only") is True
            or provenance.get("r9_initialization_only") is True
        )
        and build_manifest.get("r9_terminal_candidate_promoted") is False
        and provenance.get("r9_promoted") is False,
        "no_runtime_or_query_dependency": build_manifest.get(
            "no_legacy_shadow_runtime_dependency"
        )
        is True
        and _zero_int(build_manifest.get("offline_h0_queries"))
        and _zero_int(build_manifest.get("environment_reset_calls"))
        and _zero_int(build_manifest.get("environment_step_calls")),
    }
    return {"passed": all(value is True for value in checks.values()), "checks": checks}


def _run_candidate_freeze(adapters: RunnerAdapters) -> dict[str, Any]:
    fit_receipt = dict(adapters.verify_fit())
    module = _tree(contract.CANDIDATE_MODULE_PATH)
    identity = contract.candidate_id(module["tree_sha256"])
    actor_manifest_path = (
        _path(contract.CANDIDATE_MODULE_PATH) / "actor_feature_manifest.json"
    )
    build_manifest_path = (
        _path(contract.CANDIDATE_MODULE_PATH) / "candidate_build_manifest.json"
    )
    actor_manifest = _mapping(actor_manifest_path)
    build_manifest = _mapping(build_manifest_path)
    semantic = _candidate_semantic_audit(module, actor_manifest, build_manifest)
    if semantic.get("passed") is not True:
        raise V12R10ExecutionError("R10 candidate semantic freeze failed")
    if fit_receipt.get("candidate_id") != identity:
        raise V12R10ExecutionError("fit/freeze candidate identity drifted")
    receipt = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.CANDIDATE_FREEZE_PASS_STATUS,
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "stage_id": "freeze_recovery_actor",
        "candidate_selection_rule": contract.CANDIDATE_SELECTION_RULE,
        "candidate_id": identity,
        "candidate_module": module,
        "fit_receipt": _record(contract.FIT_RECEIPT_PATH),
        "actor_feature_manifest": _record(actor_manifest_path),
        "actor_digest": actor_manifest["actor_digest"],
        "candidate_build_manifest": _record(build_manifest_path),
        "semantic_audit": semantic,
        "pipeline_claim": _record(contract.CLAIM_PATH),
        "worker_claim": _record(_stage_claim_path("freeze_recovery_actor")),
        "candidate_frozen": True,
        "fit_gate_passed": True,
        "standard_actor": True,
        "warm_start_target_1024_compatible": True,
        "r6_functional_predecessor": True,
        "r9_hidden_initialization_only": True,
        "actor_fit_count": 1,
        "runtime_promoted": False,
        "qualification_executed": False,
        "checkpoint_zero_created": False,
        "positive_morphology_enabled": False,
        "q3_paths_opened": [],
        "actor_updates": 1,
        "critic_updates": 0,
        "ppo_updates": 0,
    }
    _write(contract.CANDIDATE_FREEZE_PATH, receipt)
    return verify_candidate_freeze_receipt(fit_verifier=adapters.verify_fit)


def verify_candidate_freeze_receipt(
    *, fit_verifier: Callable[[], Mapping[str, Any]] | None = None
) -> dict[str, Any]:
    verify_fit = fitter.verify_fit_stage if fit_verifier is None else fit_verifier
    fit_receipt = dict(verify_fit())
    verify_worker_claim("fit_recovery_actor")
    verify_worker_claim("freeze_recovery_actor")
    receipt = _mapping(contract.CANDIDATE_FREEZE_PATH)
    module = _tree(contract.CANDIDATE_MODULE_PATH)
    identity = contract.candidate_id(module["tree_sha256"])
    actor_path = _path(contract.CANDIDATE_MODULE_PATH) / "actor_feature_manifest.json"
    build_path = _path(contract.CANDIDATE_MODULE_PATH) / "candidate_build_manifest.json"
    actor_manifest = _mapping(actor_path)
    build_manifest = _mapping(build_path)
    semantic = _candidate_semantic_audit(module, actor_manifest, build_manifest)
    checks = {
        "schema": _strict_schema(receipt, CANDIDATE_FREEZE_FIELDS),
        "identity": receipt.get("status") == contract.CANDIDATE_FREEZE_PASS_STATUS
        and receipt.get("passed") is True
        and receipt.get("protocol_id") == contract.PROTOCOL_ID
        and receipt.get("pipeline_id") == contract.PIPELINE_ID
        and receipt.get("stage_id") == "freeze_recovery_actor"
        and receipt.get("candidate_selection_rule")
        == contract.CANDIDATE_SELECTION_RULE,
        "candidate": receipt.get("candidate_id") == identity
        and _strict_equal(receipt.get("candidate_module"), module)
        and fit_receipt.get("candidate_id") == identity,
        "fit": receipt.get("fit_receipt") == _record(contract.FIT_RECEIPT_PATH),
        "manifests": receipt.get("actor_feature_manifest") == _record(actor_path)
        and receipt.get("candidate_build_manifest") == _record(build_path)
        and receipt.get("actor_digest") == actor_manifest.get("actor_digest"),
        "semantic": semantic.get("passed") is True
        and _strict_equal(receipt.get("semantic_audit"), semantic),
        "claims": receipt.get("pipeline_claim") == _record(contract.CLAIM_PATH)
        and receipt.get("worker_claim")
        == _record(_stage_claim_path("freeze_recovery_actor")),
        "closed": receipt.get("candidate_frozen") is True
        and receipt.get("fit_gate_passed") is True
        and receipt.get("standard_actor") is True
        and receipt.get("warm_start_target_1024_compatible") is True
        and receipt.get("r6_functional_predecessor") is True
        and receipt.get("r9_hidden_initialization_only") is True
        and receipt.get("actor_fit_count") == 1
        and type(receipt.get("actor_fit_count")) is int
        and receipt.get("runtime_promoted") is False
        and receipt.get("qualification_executed") is False
        and receipt.get("checkpoint_zero_created") is False
        and receipt.get("positive_morphology_enabled") is False
        and receipt.get("q3_paths_opened") == [],
        "counters": receipt.get("actor_updates") == 1
        and type(receipt.get("actor_updates")) is int
        and _zero_int(receipt.get("critic_updates"))
        and _zero_int(receipt.get("ppo_updates")),
    }
    if not all(value is True for value in checks.values()):
        failed = sorted(name for name, value in checks.items() if not value)
        raise V12R10ExecutionError(f"candidate freeze receipt drifted: {failed}")
    return receipt


def _physical_config() -> physical.PhysicalDevelopmentConfig:
    return physical.PhysicalDevelopmentConfig(
        protocol_id=contract.PROTOCOL_ID,
        start_status="STARTED_H0_V12R10_PURE_DEVELOPMENT",
        partial_status="PERSISTED_H0_V12R10_DEVELOPMENT_BEFORE_GATE",
        complete_status="COMPLETE_H0_V12R10_PURE_DEVELOPMENT",
        schema_version=contract.SCHEMA_VERSION,
        artifact_root=REPO_ROOT,
        progress_label="V12R10 pure development",
        progress_every=25,
    )


def _normalization_receipt(value: Mapping[str, Any]) -> dict[str, Any]:
    receipt = {
        key: copy.deepcopy(item) for key, item in value.items() if key != "summary"
    }
    upstream = receipt.get("status")
    receipt["upstream_v12r9_status"] = upstream
    receipt["status"] = (
        "PASS_H0_V12R10_V26_DEVELOPMENT_SUMMARY_NORMALIZATION"
        if receipt.get("passed") is True
        else "FAIL_H0_V12R10_V26_DEVELOPMENT_SUMMARY_NORMALIZATION"
    )
    return receipt


def _recompute_development_runtime_evidence(
    *, summary: Mapping[str, Any], trace: Sequence[Any], case_id: str
) -> tuple[dict[str, Any], dict[str, bool]]:
    audit = physical.pure_policy_trace_audit(
        trace, config=_physical_config(), case_id=case_id
    )
    pure_counter_names = tuple(physical.PURE_POLICY_COUNTER_FIELDS)
    checks = {
        "full_horizon": type(summary.get("steps")) is int
        and summary.get("steps") == contract.EXPECTED_STEPS
        and len(trace) == contract.EXPECTED_STEPS,
        "pure_trace": audit.get("passed") is True,
        "detector_active": summary.get("binary_phase_fsm_mode") == "binary_active"
        and summary.get("event_contract_id") == contract.EVENT_CONTRACT_ID
        and summary.get("target_contract_id") == contract.TARGET_CONTRACT_ID,
        "morphology_zero": isinstance(summary.get("morphology_weight"), (int, float))
        and not isinstance(summary.get("morphology_weight"), bool)
        and float(summary["morphology_weight"]) == 0.0,
        "zero_teacher_blend_latch": summary.get("teacher_enabled") is False
        and summary.get("blending_enabled") is False
        and summary.get("safety_latch_enabled") is False
        and all(_zero_int(summary.get(name)) for name in pure_counter_names),
        "zero_updates": all(
            _zero_int(summary.get(name))
            for name in ("actor_updates", "critic_updates", "ppo_updates")
        ),
        "sensor_count": sum(
            int(row.get("raw_sensor_sample_count", -1))
            for row in trace
            if isinstance(row, Mapping)
        )
        == contract.EXPECTED_STEPS * contract.RAW_SAMPLES_PER_STEP,
    }
    return audit, checks


def _run_development(
    case_id: str,
    activity: dict[str, int],
    adapters: RunnerAdapters,
) -> dict[str, Any]:
    verify_candidate_freeze_receipt(fit_verifier=adapters.verify_fit)
    module = _tree(contract.CANDIDATE_MODULE_PATH)
    identity = contract.candidate_id(module["tree_sha256"])
    case = contract.canonical_development_case(case_id)
    destination = _path(contract.DEVELOPMENT_ROOT) / case_id
    activity_before = {
        name: activity[name]
        for name in (
            "environment_reset_calls",
            "environment_step_calls",
            "development_environment_reset_calls",
            "development_environment_step_calls",
            "raw_sensor_sample_count",
        )
    }
    result = adapters.run_physical(
        config=_physical_config(),
        case=case,
        destination=destination,
        module_path=_path(contract.CANDIDATE_MODULE_PATH),
        activity_callback=lambda name, amount: _increment_development(
            activity, name, amount
        ),
        start_metadata={
            "pipeline_claim": _record(contract.CLAIM_PATH),
            "worker_claim": _record(_stage_claim_path(f"development__{case_id}")),
            "execution_lock": _record(contract.EXECUTION_LOCK_PATH),
            "candidate_id": identity,
            "candidate_module": module,
            "candidate_freeze": _record(contract.CANDIDATE_FREEZE_PATH),
            "binary_detector_required": True,
            "morphology_weight_required": 0.0,
        },
        summary_metadata={
            "candidate_id": identity,
            "candidate_module": module,
            "candidate_tree_sha256": module["tree_sha256"],
            "candidate_freeze": _record(contract.CANDIDATE_FREEZE_PATH),
            "target_contract_id": contract.TARGET_CONTRACT_ID,
            "event_contract_id": contract.EVENT_CONTRACT_ID,
        },
    )
    expected_delta = {
        "environment_reset_calls": 1,
        "environment_step_calls": contract.EXPECTED_STEPS,
        "development_environment_reset_calls": 1,
        "development_environment_step_calls": contract.EXPECTED_STEPS,
        "raw_sensor_sample_count": contract.EXPECTED_STEPS
        * contract.RAW_SAMPLES_PER_STEP,
    }
    if any(
        activity[name] - activity_before[name] != delta
        for name, delta in expected_delta.items()
    ):
        raise V12R10ExecutionError("development activity callback closure drifted")
    raw_summary = result.get("summary")
    if not isinstance(raw_summary, Mapping):
        raise V12R10ExecutionError("development summary is malformed")
    try:
        normalization = r9_adjudicator.normalize_v26_prefix_summary(
            raw_summary,
            expected_steps=int(raw_summary.get("steps", -1)),
        )
    except Exception as exc:
        raise V12R10ExecutionError("development V26 normalization failed") from exc
    summary = {
        **normalization["summary"],
        "v26_summary_normalization": _normalization_receipt(normalization),
    }
    normalized_summary_path = destination / "normalized_summary.json"
    _write(normalized_summary_path, summary)
    trace = result.get("trace")
    if not isinstance(trace, Sequence) or isinstance(trace, (str, bytes)):
        raise V12R10ExecutionError("development trace is malformed")
    expected_audit, runtime_checks = _recompute_development_runtime_evidence(
        summary=summary,
        trace=trace,
        case_id=case_id,
    )
    if not all(value is True for value in runtime_checks.values()):
        failed = sorted(name for name, value in runtime_checks.items() if not value)
        raise V12R10ExecutionError(
            f"R10 development runtime integrity failed: {failed}"
        )
    if not _strict_equal(summary.get("pure_policy_trace_audit"), expected_audit):
        raise V12R10ExecutionError("development trace audit binding drifted")
    gate = contract.development_gate(summary, case_id=case_id)
    _write(destination / "gate.json", gate)
    if gate.get("passed") is not True:
        raise V12R10ExecutionError(f"R10 development gate failed: {case_id}")
    zero_counters = {
        name: int(summary[name]) for name in physical.PURE_POLICY_COUNTER_FIELDS
    }
    receipt = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": "PASS_H0_V12R10_PURE_DEVELOPMENT_RECEIPT",
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "stage_id": f"development__{case_id}",
        "case_id": case_id,
        "candidate_id": identity,
        "candidate_module": module,
        "candidate_freeze": _record(contract.CANDIDATE_FREEZE_PATH),
        "raw_summary": _record(destination / "summary.json"),
        "summary": _record(normalized_summary_path),
        "gate": _record(destination / "gate.json"),
        "trace": _record(destination / "trace.json"),
        "pure_policy_trace_audit": expected_audit,
        "runtime_integrity_checks": runtime_checks,
        "environment_reset_calls": 1,
        "environment_step_calls": contract.EXPECTED_STEPS,
        "raw_sensor_sample_count": (
            contract.EXPECTED_STEPS * contract.RAW_SAMPLES_PER_STEP
        ),
        **zero_counters,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
    }
    _write(destination / "receipt.json", receipt)
    _increment(activity, "development_rollouts_completed")
    return verify_development_receipt(case_id, fit_verifier=adapters.verify_fit)


def verify_development_receipt(
    case_id: str,
    *,
    fit_verifier: Callable[[], Mapping[str, Any]] | None = None,
) -> dict[str, Any]:
    contract.canonical_development_case(case_id)
    verify_candidate_freeze_receipt(fit_verifier=fit_verifier)
    verify_worker_claim(f"development__{case_id}")
    root = _path(contract.DEVELOPMENT_ROOT) / case_id
    receipt = _mapping(root / "receipt.json")
    raw_summary = _mapping(root / "summary.json")
    summary = _mapping(root / "normalized_summary.json")
    gate = _mapping(root / "gate.json")
    trace = _sequence(root / "trace.json")
    module = _tree(contract.CANDIDATE_MODULE_PATH)
    identity = contract.candidate_id(module["tree_sha256"])
    try:
        normalization = r9_adjudicator.normalize_v26_prefix_summary(
            raw_summary,
            expected_steps=int(raw_summary.get("steps", -1)),
        )
    except Exception as exc:
        raise V12R10ExecutionError("development normalization closure failed") from exc
    expected_summary = {
        **normalization["summary"],
        "v26_summary_normalization": _normalization_receipt(normalization),
    }
    expected_audit, expected_runtime = _recompute_development_runtime_evidence(
        summary=summary,
        trace=trace,
        case_id=case_id,
    )
    expected_gate = contract.development_gate(summary, case_id=case_id)
    checks = {
        "schema": _strict_schema(receipt, DEVELOPMENT_RECEIPT_FIELDS),
        "identity": receipt.get("status") == "PASS_H0_V12R10_PURE_DEVELOPMENT_RECEIPT"
        and receipt.get("passed") is True
        and receipt.get("protocol_id") == contract.PROTOCOL_ID
        and receipt.get("pipeline_id") == contract.PIPELINE_ID
        and receipt.get("stage_id") == f"development__{case_id}"
        and receipt.get("case_id") == case_id,
        "candidate": receipt.get("candidate_id") == identity
        and _strict_equal(receipt.get("candidate_module"), module)
        and receipt.get("candidate_freeze") == _record(contract.CANDIDATE_FREEZE_PATH),
        "normalization": _strict_equal(summary, expected_summary)
        and summary.get("v26_summary_normalization", {}).get("passed") is True
        and summary.get("v26_summary_normalization", {}).get("status")
        == "PASS_H0_V12R10_V26_DEVELOPMENT_SUMMARY_NORMALIZATION",
        "gate": _strict_equal(gate, expected_gate) and gate.get("passed") is True,
        "trace": expected_audit.get("passed") is True
        and _strict_equal(summary.get("pure_policy_trace_audit"), expected_audit)
        and _strict_equal(receipt.get("pure_policy_trace_audit"), expected_audit),
        "runtime": all(value is True for value in expected_runtime.values())
        and _strict_equal(receipt.get("runtime_integrity_checks"), expected_runtime),
        "artifacts": receipt.get("raw_summary") == _record(root / "summary.json")
        and receipt.get("summary") == _record(root / "normalized_summary.json")
        and receipt.get("gate") == _record(root / "gate.json")
        and receipt.get("trace") == _record(root / "trace.json"),
        "activity": receipt.get("environment_reset_calls") == 1
        and type(receipt.get("environment_reset_calls")) is int
        and receipt.get("environment_step_calls") == contract.EXPECTED_STEPS
        and type(receipt.get("environment_step_calls")) is int
        and receipt.get("raw_sensor_sample_count")
        == contract.EXPECTED_STEPS * contract.RAW_SAMPLES_PER_STEP
        and type(receipt.get("raw_sensor_sample_count")) is int,
        "pure_zero": all(
            _zero_int(receipt.get(name))
            for name in (
                *physical.PURE_POLICY_COUNTER_FIELDS,
                "actor_updates",
                "critic_updates",
                "ppo_updates",
            )
        ),
    }
    if not all(value is True for value in checks.values()):
        failed = sorted(name for name, value in checks.items() if not value)
        raise V12R10ExecutionError(f"development receipt drifted: {failed}")
    return receipt


def _expected_final_activity() -> dict[str, int]:
    development_count = len(contract.DEVELOPMENT_CASE_IDS)
    return {
        "r9_import_attestation_calls": 1,
        "actor_fit_stage_calls": 1,
        "actor_updates": 1,
        "uniform_adamw_epochs_completed": fitter.UNIFORM_ADAMW_EPOCHS,
        "uniform_lbfgs_closure_calls": fitter.EXPECTED_UNIFORM_CLOSURES,
        "gate_adamw_epochs_completed": fitter.GATE_ADAMW_EPOCHS,
        "gate_lbfgs_closure_calls": fitter.EXPECTED_GATE_CLOSURES,
        "development_rollouts_completed": development_count,
        "environment_reset_calls": development_count,
        "environment_step_calls": development_count * contract.EXPECTED_STEPS,
        "development_environment_reset_calls": development_count,
        "development_environment_step_calls": development_count
        * contract.EXPECTED_STEPS,
        "raw_sensor_sample_count": development_count
        * contract.EXPECTED_STEPS
        * contract.RAW_SAMPLES_PER_STEP,
        "teacher_query_count": 0,
        "served_action_teacher_dependency_count": 0,
        "mean_blend_count": 0,
        "safety_intervention_count": 0,
        "safety_latch_activation_count": 0,
        "safety_latch_release_count": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
    }


def _run_final_development(
    activity: Mapping[str, int], adapters: RunnerAdapters
) -> dict[str, Any]:
    freeze = verify_candidate_freeze_receipt(fit_verifier=adapters.verify_fit)
    module = _tree(contract.CANDIDATE_MODULE_PATH)
    identity = contract.candidate_id(module["tree_sha256"])
    bindings = []
    for case_id in contract.DEVELOPMENT_CASE_IDS:
        development = verify_development_receipt(
            case_id, fit_verifier=adapters.verify_fit
        )
        root = _path(contract.DEVELOPMENT_ROOT) / case_id
        bindings.append(
            {
                "case_id": case_id,
                "passed": True,
                "receipt": _record(root / "receipt.json"),
                "summary": development["summary"],
                "gate": development["gate"],
            }
        )
    expected_activity = _expected_final_activity()
    if not _strict_equal(dict(activity), expected_activity):
        raise V12R10ExecutionError("final activity accounting drifted")
    receipt = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.DEVELOPMENT_PASS_STATUS,
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "stage_id": "finalize_development",
        "candidate_id": identity,
        "candidate_module": module,
        "candidate_freeze": _record(contract.CANDIDATE_FREEZE_PATH),
        "r9_import_attestation": _record(contract.R9_IMPORT_ATTESTATION_PATH),
        "rollout_bindings": bindings,
        "case_gates": [
            {"case_id": case_id, "passed": True}
            for case_id in contract.DEVELOPMENT_CASE_IDS
        ],
        "collection_round_count": 0,
        "actor_fit_count": 1,
        "actor_updates": 1,
        "development_rollout_count": len(contract.DEVELOPMENT_CASE_IDS),
        "development_count": len(contract.DEVELOPMENT_CASE_IDS),
        "passing_development_count": len(contract.DEVELOPMENT_CASE_IDS),
        "failed_development_count": 0,
        "environment_reset_calls": activity["environment_reset_calls"],
        "environment_step_calls": activity["environment_step_calls"],
        "raw_sensor_sample_count": activity["raw_sensor_sample_count"],
        "development_actor_updates": 0,
        "development_critic_updates": 0,
        "development_ppo_updates": 0,
        "offline_h0_query_count": 0,
        "teacher_query_count": 0,
        "served_action_teacher_dependency_count": 0,
        "mean_blend_count": 0,
        "safety_intervention_count": 0,
        "safety_latch_activation_count": 0,
        "safety_latch_release_count": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "binary_detector_active": True,
        "morphology_weight": 0.0,
        "r6_functional_predecessor": True,
        "r9_hidden_initialization_only": True,
        "qualification_executed": False,
        "runtime_promoted": False,
        "checkpoint_zero_created": False,
        "positive_morphology_enabled": False,
        "q3_paths_opened": [],
        "pipeline_claim": _record(contract.CLAIM_PATH),
        "worker_claim": _record(_stage_claim_path("finalize_development")),
        "fit_receipt": _record(contract.FIT_RECEIPT_PATH),
        "fit_candidate_id": freeze["candidate_id"],
        "retry_authorized": False,
        "resume_authorized": False,
        "fit_sweep_authorized": False,
    }
    _write(contract.FINAL_DEVELOPMENT_PATH, receipt)
    return verify_final_development_receipt(fit_verifier=adapters.verify_fit)


def _final_development_fields() -> set[str]:
    return {
        "schema_version",
        "status",
        "passed",
        "protocol_id",
        "pipeline_id",
        "stage_id",
        "candidate_id",
        "candidate_module",
        "candidate_freeze",
        "r9_import_attestation",
        "rollout_bindings",
        "case_gates",
        "collection_round_count",
        "actor_fit_count",
        "actor_updates",
        "development_rollout_count",
        "development_count",
        "passing_development_count",
        "failed_development_count",
        "environment_reset_calls",
        "environment_step_calls",
        "raw_sensor_sample_count",
        "development_actor_updates",
        "development_critic_updates",
        "development_ppo_updates",
        "offline_h0_query_count",
        "teacher_query_count",
        "served_action_teacher_dependency_count",
        "mean_blend_count",
        "safety_intervention_count",
        "safety_latch_activation_count",
        "safety_latch_release_count",
        "critic_updates",
        "ppo_updates",
        "binary_detector_active",
        "morphology_weight",
        "r6_functional_predecessor",
        "r9_hidden_initialization_only",
        "qualification_executed",
        "runtime_promoted",
        "checkpoint_zero_created",
        "positive_morphology_enabled",
        "q3_paths_opened",
        "pipeline_claim",
        "worker_claim",
        "fit_receipt",
        "fit_candidate_id",
        "retry_authorized",
        "resume_authorized",
        "fit_sweep_authorized",
    }


def verify_final_development_receipt(
    *, fit_verifier: Callable[[], Mapping[str, Any]] | None = None
) -> dict[str, Any]:
    freeze = verify_candidate_freeze_receipt(fit_verifier=fit_verifier)
    verify_worker_claim("finalize_development")
    receipt = _mapping(contract.FINAL_DEVELOPMENT_PATH)
    module = _tree(contract.CANDIDATE_MODULE_PATH)
    identity = contract.candidate_id(module["tree_sha256"])
    expected_bindings = []
    for case_id in contract.DEVELOPMENT_CASE_IDS:
        development = verify_development_receipt(case_id, fit_verifier=fit_verifier)
        root = _path(contract.DEVELOPMENT_ROOT) / case_id
        expected_bindings.append(
            {
                "case_id": case_id,
                "passed": True,
                "receipt": _record(root / "receipt.json"),
                "summary": development["summary"],
                "gate": development["gate"],
            }
        )
    count = len(contract.DEVELOPMENT_CASE_IDS)
    checks = {
        "schema": _strict_schema(receipt, _final_development_fields()),
        "identity": receipt.get("status") == contract.DEVELOPMENT_PASS_STATUS
        and receipt.get("passed") is True
        and receipt.get("protocol_id") == contract.PROTOCOL_ID
        and receipt.get("pipeline_id") == contract.PIPELINE_ID
        and receipt.get("stage_id") == "finalize_development",
        "candidate": receipt.get("candidate_id") == identity
        and _strict_equal(receipt.get("candidate_module"), module)
        and receipt.get("candidate_freeze") == _record(contract.CANDIDATE_FREEZE_PATH)
        and receipt.get("fit_candidate_id") == freeze.get("candidate_id"),
        "sources": receipt.get("r9_import_attestation")
        == _record(contract.R9_IMPORT_ATTESTATION_PATH)
        and receipt.get("fit_receipt") == _record(contract.FIT_RECEIPT_PATH),
        "rollouts": _strict_equal(receipt.get("rollout_bindings"), expected_bindings)
        and receipt.get("case_gates")
        == [
            {"case_id": case_id, "passed": True}
            for case_id in contract.DEVELOPMENT_CASE_IDS
        ],
        "counts": receipt.get("collection_round_count") == 0
        and receipt.get("actor_fit_count") == 1
        and receipt.get("actor_updates") == 1
        and receipt.get("development_rollout_count") == count
        and receipt.get("development_count") == count
        and receipt.get("passing_development_count") == count
        and receipt.get("failed_development_count") == 0
        and all(
            type(receipt.get(name)) is int
            for name in (
                "collection_round_count",
                "actor_fit_count",
                "actor_updates",
                "development_rollout_count",
                "development_count",
                "passing_development_count",
                "failed_development_count",
            )
        ),
        "physical_accounting": receipt.get("environment_reset_calls") == count
        and receipt.get("environment_step_calls") == count * contract.EXPECTED_STEPS
        and receipt.get("raw_sensor_sample_count")
        == count * contract.EXPECTED_STEPS * contract.RAW_SAMPLES_PER_STEP
        and all(
            type(receipt.get(name)) is int
            for name in (
                "environment_reset_calls",
                "environment_step_calls",
                "raw_sensor_sample_count",
            )
        ),
        "pure_zero": all(
            _zero_int(receipt.get(name))
            for name in (
                "development_actor_updates",
                "development_critic_updates",
                "development_ppo_updates",
                "offline_h0_query_count",
                *physical.PURE_POLICY_COUNTER_FIELDS,
                "critic_updates",
                "ppo_updates",
            )
        ),
        "runtime": receipt.get("binary_detector_active") is True
        and isinstance(receipt.get("morphology_weight"), (int, float))
        and not isinstance(receipt.get("morphology_weight"), bool)
        and float(receipt["morphology_weight"]) == 0.0,
        "provenance": receipt.get("r6_functional_predecessor") is True
        and receipt.get("r9_hidden_initialization_only") is True,
        "closed": receipt.get("qualification_executed") is False
        and receipt.get("runtime_promoted") is False
        and receipt.get("checkpoint_zero_created") is False
        and receipt.get("positive_morphology_enabled") is False
        and receipt.get("q3_paths_opened") == []
        and receipt.get("retry_authorized") is False
        and receipt.get("resume_authorized") is False
        and receipt.get("fit_sweep_authorized") is False,
        "claims": receipt.get("pipeline_claim") == _record(contract.CLAIM_PATH)
        and receipt.get("worker_claim")
        == _record(_stage_claim_path("finalize_development")),
    }
    if not all(value is True for value in checks.values()):
        failed = sorted(name for name, value in checks.items() if not value)
        raise V12R10ExecutionError(f"final development receipt drifted: {failed}")
    return receipt


def _candidate_module_snapshot() -> tuple[Any, Any, dict[str, Any]]:
    candidate: dict[str, Any] | None = None
    candidate_id: str | None = None
    observation: dict[str, Any] = {"state": "ABSENT", "tree": None}
    candidate_path = _path(contract.CANDIDATE_MODULE_PATH)
    if candidate_path.is_dir() and not candidate_path.is_symlink():
        try:
            candidate = _tree(candidate_path)
            candidate_id = contract.candidate_id(candidate["tree_sha256"])
            file_names = {
                row.get("path")
                for row in candidate.get("files", [])
                if isinstance(row, Mapping)
            }
            state = (
                "VALID_COMPLETE_TREE"
                if candidate.get("file_count") == 5
                and file_names == EXPECTED_CANDIDATE_FILES
                else "PARTIAL_OR_UNEXPECTED_TREE"
            )
            observation = {"state": state, "tree": candidate}
        except BaseException as exc:
            observation = {
                "state": "PARTIAL_OR_UNSAFE_TREE",
                "tree": candidate,
                "error": {"type": type(exc).__name__, "message": str(exc)},
            }
    elif os.path.lexists(candidate_path):
        observation = {"state": "UNSAFE_NONDIRECTORY", "tree": None}
    return candidate, candidate_id, observation


def _terminal_payload(
    *,
    passed: bool,
    completed: Sequence[Mapping[str, Any]],
    attempted_stage: str | None,
    activity: Mapping[str, int],
    error: BaseException | None,
) -> dict[str, Any]:
    candidate, candidate_id, observation = _candidate_module_snapshot()
    completed_ids = [
        item.get("stage_id") for item in completed if isinstance(item, Mapping)
    ]
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": (
            contract.PIPELINE_TERMINAL_PASS_STATUS
            if passed
            else "FAIL_H0_V12R10_RECOVERY_PIPELINE_TERMINAL"
        ),
        "passed": passed,
        "terminal": True,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "candidate_selection_rule": contract.CANDIDATE_SELECTION_RULE,
        "candidate_id": candidate_id,
        "candidate_module": candidate,
        "candidate_module_observation": observation,
        "stage_order": list(contract.STAGE_IDS),
        "completed_stages": list(completed),
        "completed_stage_count": len(completed),
        "attempted_stage": attempted_stage,
        "activity_totals": dict(activity),
        "import_attestation_count": sum(
            stage == "attest_r9_terminal_imports" for stage in completed_ids
        ),
        "collection_round_count": 0,
        "actor_fit_count": sum(
            stage == "fit_recovery_actor" for stage in completed_ids
        ),
        "development_count": sum(
            isinstance(stage, str) and stage.startswith("development__")
            for stage in completed_ids
        ),
        "protocol_freeze": _record(contract.PROTOCOL_FREEZE_PATH),
        "execution_lock": _record(contract.EXECUTION_LOCK_PATH),
        "pipeline_claim": _record(contract.CLAIM_PATH),
        "r9_import_attestation": (
            _record(contract.R9_IMPORT_ATTESTATION_PATH)
            if _path(contract.R9_IMPORT_ATTESTATION_PATH).is_file()
            else None
        ),
        "candidate_freeze": (
            _record(contract.CANDIDATE_FREEZE_PATH)
            if _path(contract.CANDIDATE_FREEZE_PATH).is_file()
            else None
        ),
        "final_development_receipt": (
            _record(contract.FINAL_DEVELOPMENT_PATH)
            if _path(contract.FINAL_DEVELOPMENT_PATH).is_file()
            else None
        ),
        "qualification_executed": False,
        "runtime_promoted": False,
        "checkpoint_zero_created": False,
        "positive_morphology_enabled": False,
        "q3_paths_opened": [],
        "actor_updates": activity["actor_updates"],
        "critic_updates": activity["critic_updates"],
        "ppo_updates": activity["ppo_updates"],
        "environment_reset_calls": activity["environment_reset_calls"],
        "environment_step_calls": activity["environment_step_calls"],
        "development_environment_reset_calls": activity[
            "development_environment_reset_calls"
        ],
        "development_environment_step_calls": activity[
            "development_environment_step_calls"
        ],
        "raw_sensor_sample_count": activity["raw_sensor_sample_count"],
        "teacher_query_count": activity["teacher_query_count"],
        "served_action_teacher_dependency_count": activity[
            "served_action_teacher_dependency_count"
        ],
        "mean_blend_count": activity["mean_blend_count"],
        "safety_intervention_count": activity["safety_intervention_count"],
        "safety_latch_activation_count": activity["safety_latch_activation_count"],
        "safety_latch_release_count": activity["safety_latch_release_count"],
        "retry_authorized": False,
        "resume_authorized": False,
        "fit_sweep_authorized": False,
        "error": (
            None
            if error is None
            else {"type": type(error).__name__, "message": str(error) or repr(error)}
        ),
        "next_stage": (
            "WAIT_SEPARATE_V12R10Q3_PROTOCOL" if passed else "STOP_TERMINAL_NO_RETRY"
        ),
    }


def _stage_receipt_path(stage_id: str) -> Path:
    if stage_id == "attest_r9_terminal_imports":
        return _path(contract.R9_IMPORT_ATTESTATION_PATH)
    if stage_id == "fit_recovery_actor":
        return _path(contract.FIT_RECEIPT_PATH)
    if stage_id == "freeze_recovery_actor":
        return _path(contract.CANDIDATE_FREEZE_PATH)
    if stage_id.startswith("development__"):
        return _development_receipt_path(stage_id.removeprefix("development__"))
    if stage_id == "finalize_development":
        return _path(contract.FINAL_DEVELOPMENT_PATH)
    raise V12R10ExecutionError(f"unknown stage: {stage_id}")


def _verify_completed_stage_semantics(
    stage_id: str,
    *,
    fit_verifier: Callable[[], Mapping[str, Any]],
    source_verifier: Callable[[], Mapping[str, Any]],
) -> dict[str, Any]:
    verify_worker_claim(stage_id)
    if stage_id == "attest_r9_terminal_imports":
        return verify_r9_attestation_receipt(source_verifier=source_verifier)
    if stage_id == "fit_recovery_actor":
        return dict(fit_verifier())
    if stage_id == "freeze_recovery_actor":
        return verify_candidate_freeze_receipt(fit_verifier=fit_verifier)
    if stage_id.startswith("development__"):
        return verify_development_receipt(
            stage_id.removeprefix("development__"), fit_verifier=fit_verifier
        )
    if stage_id == "finalize_development":
        return verify_final_development_receipt(fit_verifier=fit_verifier)
    raise V12R10ExecutionError(f"unknown completed stage: {stage_id}")


def _prefix_activity_coherent(
    ledger: Mapping[str, Any],
    *,
    completed_ids: Sequence[str],
    attempted_stage: str | None,
) -> bool:
    activity = ledger.get("activity_totals")
    if (
        not isinstance(activity, Mapping)
        or set(activity) != set(ACTIVITY_FIELDS)
        or any(
            type(activity.get(name)) is not int or activity[name] < 0
            for name in ACTIVITY_FIELDS
        )
    ):
        return False
    completed_import = int("attest_r9_terminal_imports" in completed_ids)
    completed_fit = int("fit_recovery_actor" in completed_ids)
    completed_development = sum(
        stage.startswith("development__") for stage in completed_ids
    )
    attempted_import = attempted_stage == "attest_r9_terminal_imports"
    attempted_fit = attempted_stage == "fit_recovery_actor"
    attempted_development = bool((attempted_stage or "").startswith("development__"))
    minimum_steps = completed_development * contract.EXPECTED_STEPS
    maximum_steps = minimum_steps + (
        contract.EXPECTED_STEPS if attempted_development else 0
    )
    minimum_samples = minimum_steps * contract.RAW_SAMPLES_PER_STEP
    maximum_samples = maximum_steps * contract.RAW_SAMPLES_PER_STEP

    fit_counters = {
        "uniform_adamw_epochs_completed": fitter.UNIFORM_ADAMW_EPOCHS,
        "uniform_lbfgs_closure_calls": fitter.EXPECTED_UNIFORM_CLOSURES,
        "gate_adamw_epochs_completed": fitter.GATE_ADAMW_EPOCHS,
        "gate_lbfgs_closure_calls": fitter.EXPECTED_GATE_CLOSURES,
    }
    if completed_fit:
        fit_activity_ok = (
            all(
                activity.get(name) == expected
                for name, expected in fit_counters.items()
            )
            and activity.get("actor_updates") == 1
        )
    elif attempted_fit:
        fit_activity_ok = (
            activity.get("actor_updates") in {0, 1}
            and 0
            <= activity["uniform_adamw_epochs_completed"]
            <= fitter.UNIFORM_ADAMW_EPOCHS
            and 0
            <= activity["uniform_lbfgs_closure_calls"]
            <= fitter.UNIFORM_LBFGS_MAX_EVAL
            and 0 <= activity["gate_adamw_epochs_completed"] <= fitter.GATE_ADAMW_EPOCHS
            and 0 <= activity["gate_lbfgs_closure_calls"] <= fitter.GATE_LBFGS_MAX_EVAL
        )
    else:
        fit_activity_ok = activity.get("actor_updates") == 0 and all(
            activity.get(name) == 0 for name in fit_counters
        )

    resets = activity.get("environment_reset_calls")
    steps = activity.get("environment_step_calls")
    samples = activity.get("raw_sensor_sample_count")
    return bool(
        ledger.get("collection_round_count") == 0
        and ledger.get("import_attestation_count") == completed_import
        and ledger.get("actor_fit_count") == completed_fit
        and ledger.get("development_count") == completed_development
        and activity.get("r9_import_attestation_calls")
        in {completed_import, completed_import + int(attempted_import)}
        and activity.get("actor_fit_stage_calls")
        in {completed_fit, completed_fit + int(attempted_fit)}
        and fit_activity_ok
        and activity.get("development_rollouts_completed") == completed_development
        and resets
        in {
            completed_development,
            completed_development + int(attempted_development),
        }
        and activity.get("development_environment_reset_calls") == resets
        and minimum_steps <= steps <= maximum_steps
        and activity.get("development_environment_step_calls") == steps
        and minimum_samples <= samples <= maximum_samples
        and samples <= steps * contract.RAW_SAMPLES_PER_STEP
        and all(
            activity.get(name) == 0
            for name in (
                "teacher_query_count",
                "served_action_teacher_dependency_count",
                "mean_blend_count",
                "safety_intervention_count",
                "safety_latch_activation_count",
                "safety_latch_release_count",
                "critic_updates",
                "ppo_updates",
            )
        )
        and ledger.get("actor_updates") == activity.get("actor_updates")
        and ledger.get("critic_updates") == activity.get("critic_updates")
        and ledger.get("ppo_updates") == activity.get("ppo_updates")
        and ledger.get("environment_reset_calls") == resets
        and ledger.get("environment_step_calls") == steps
        and ledger.get("development_environment_reset_calls") == resets
        and ledger.get("development_environment_step_calls") == steps
        and ledger.get("raw_sensor_sample_count") == samples
        and all(
            ledger.get(name) == activity.get(name)
            for name in (
                "teacher_query_count",
                "served_action_teacher_dependency_count",
                "mean_blend_count",
                "safety_intervention_count",
                "safety_latch_activation_count",
                "safety_latch_release_count",
            )
        )
    )


def verify_terminal_ledger(
    *,
    fit_verifier: Callable[[], Mapping[str, Any]] | None = None,
    source_verifier: Callable[[], Mapping[str, Any]] | None = None,
) -> dict[str, Any]:
    freezer.verify_protocol_freeze()
    freezer.verify_execution_lock(require_pristine=False)
    locked_before = freezer.attest_locked_inputs()
    verify_pipeline_claim()
    ledger = _mapping(contract.LEDGER_PATH)
    if set(ledger) != TERMINAL_LEDGER_FIELDS:
        raise V12R10ExecutionError("terminal ledger schema drifted")
    if type(ledger.get("passed")) is not bool:
        raise V12R10ExecutionError("terminal ledger passed flag drifted")
    passed = ledger["passed"]
    expected_status = (
        contract.PIPELINE_TERMINAL_PASS_STATUS
        if passed
        else "FAIL_H0_V12R10_RECOVERY_PIPELINE_TERMINAL"
    )
    candidate, candidate_id, observation = _candidate_module_snapshot()
    optional_records = {
        "r9_import_attestation": contract.R9_IMPORT_ATTESTATION_PATH,
        "candidate_freeze": contract.CANDIDATE_FREEZE_PATH,
        "final_development_receipt": contract.FINAL_DEVELOPMENT_PATH,
    }
    checks = {
        "identity": ledger.get("schema_version") == contract.SCHEMA_VERSION
        and type(ledger.get("schema_version")) is int
        and ledger.get("status") == expected_status
        and ledger.get("terminal") is True
        and ledger.get("protocol_id") == contract.PROTOCOL_ID
        and ledger.get("pipeline_id") == contract.PIPELINE_ID,
        "selection_and_order": ledger.get("candidate_selection_rule")
        == contract.CANDIDATE_SELECTION_RULE
        and ledger.get("stage_order") == list(contract.STAGE_IDS),
        "candidate_full_tree": _strict_equal(ledger.get("candidate_id"), candidate_id)
        and _strict_equal(ledger.get("candidate_module"), candidate)
        and _strict_equal(ledger.get("candidate_module_observation"), observation),
        "bindings": ledger.get("protocol_freeze")
        == _record(contract.PROTOCOL_FREEZE_PATH)
        and ledger.get("execution_lock") == _record(contract.EXECUTION_LOCK_PATH)
        and ledger.get("pipeline_claim") == _record(contract.CLAIM_PATH)
        and all(
            _strict_equal(
                ledger.get(name),
                _record(path) if _path(path).is_file() else None,
            )
            for name, path in optional_records.items()
        ),
        "closed": ledger.get("retry_authorized") is False
        and ledger.get("resume_authorized") is False
        and ledger.get("fit_sweep_authorized") is False
        and ledger.get("qualification_executed") is False
        and ledger.get("runtime_promoted") is False
        and ledger.get("checkpoint_zero_created") is False
        and ledger.get("positive_morphology_enabled") is False
        and ledger.get("q3_paths_opened") == [],
        "next_stage": ledger.get("next_stage")
        == ("WAIT_SEPARATE_V12R10Q3_PROTOCOL" if passed else "STOP_TERMINAL_NO_RETRY"),
        "activity": _prefix_activity_coherent(
            ledger,
            completed_ids=[
                str(item.get("stage_id"))
                for item in ledger.get("completed_stages", [])
                if isinstance(item, Mapping)
            ],
            attempted_stage=ledger.get("attempted_stage"),
        ),
    }
    completed = ledger.get("completed_stages")
    count = ledger.get("completed_stage_count")
    if (
        not isinstance(completed, list)
        or type(count) is not int
        or count != len(completed)
    ):
        raise V12R10ExecutionError("terminal completed-stage prefix drifted")
    selected_fit = fitter.verify_fit_stage if fit_verifier is None else fit_verifier
    selected_source = _verify_r9_inputs if source_verifier is None else source_verifier
    cached_fit: dict[str, Any] | None = None
    cached_source: dict[str, Any] | None = None

    def verify_fit_once() -> dict[str, Any]:
        nonlocal cached_fit
        if cached_fit is None:
            cached_fit = dict(selected_fit())
        return copy.deepcopy(cached_fit)

    def verify_source_once() -> dict[str, Any]:
        nonlocal cached_source
        if cached_source is None:
            cached_source = dict(selected_source())
        return copy.deepcopy(cached_source)

    completed_ids: list[str] = []
    for index, item in enumerate(completed):
        expected_stage = contract.STAGE_IDS[index]
        if (
            not isinstance(item, Mapping)
            or item.get("stage_id") != expected_stage
            or item.get("receipt") != _record(_stage_receipt_path(expected_stage))
        ):
            raise V12R10ExecutionError("terminal stage receipt binding drifted")
        _verify_completed_stage_semantics(
            expected_stage,
            fit_verifier=verify_fit_once,
            source_verifier=verify_source_once,
        )
        completed_ids.append(expected_stage)
    if not all(value is True for value in checks.values()):
        failed = sorted(name for name, value in checks.items() if not value)
        raise V12R10ExecutionError(f"terminal ledger drifted: {failed}")
    if passed:
        if (
            completed_ids != list(contract.STAGE_IDS)
            or ledger.get("attempted_stage") is not None
            or ledger.get("error") is not None
            or not _strict_equal(
                ledger.get("activity_totals"), _expected_final_activity()
            )
            or ledger.get("candidate_module_observation", {}).get("state")
            != "VALID_COMPLETE_TREE"
            or ledger.get("candidate_freeze") != _record(contract.CANDIDATE_FREEZE_PATH)
            or ledger.get("final_development_receipt")
            != _record(contract.FINAL_DEVELOPMENT_PATH)
        ):
            raise V12R10ExecutionError("terminal PASS closure drifted")
        verify_claim_directory(completed_ids)
        verify_r9_attestation_receipt(source_verifier=verify_source_once)
        verify_candidate_freeze_receipt(fit_verifier=verify_fit_once)
        verify_final_development_receipt(fit_verifier=verify_fit_once)
    else:
        attempted = ledger.get("attempted_stage")
        error = ledger.get("error")
        proper_prefix = len(completed_ids) < len(contract.STAGE_IDS)
        expected_attempted = (
            contract.STAGE_IDS[len(completed_ids)] if proper_prefix else None
        )
        strict_error = (
            isinstance(error, Mapping)
            and set(error) == {"type", "message"}
            and isinstance(error.get("type"), str)
            and bool(error["type"])
            and isinstance(error.get("message"), str)
            and bool(error["message"])
        )
        if not proper_prefix or attempted != expected_attempted or not strict_error:
            raise V12R10ExecutionError("terminal FAIL forensic prefix drifted")
        assert expected_attempted is not None
        verify_claim_directory([*completed_ids, expected_attempted])
    if not _strict_equal(freezer.attest_locked_inputs(), locked_before):
        raise V12R10ExecutionError("locked inputs changed during terminal verification")
    return ledger


def execute(*, adapters: RunnerAdapters | None = None) -> dict[str, Any]:
    selected = adapters or RunnerAdapters()
    ledger_path = _path(contract.LEDGER_PATH)
    if os.path.lexists(ledger_path):
        return verify_terminal_ledger(
            fit_verifier=selected.verify_fit,
            source_verifier=selected.verify_r9_inputs,
        )
    freezer.verify_protocol_freeze()
    freezer.verify_execution_lock(require_pristine=True)
    run_root = _path(contract.RUN_ROOT)
    if os.path.lexists(run_root):
        raise V12R10ExecutionError("R10 run root occupied; resume is forbidden")
    os.mkdir(run_root, 0o700)
    os.mkdir(_claims_root(), 0o700)
    _write(contract.CLAIM_PATH, _expected_pipeline_claim())
    activity = _zero_activity()
    completed: list[dict[str, Any]] = []
    attempted_stage: str | None = None
    error: BaseException | None = None
    try:
        for stage_id in contract.STAGE_IDS:
            attempted_stage = stage_id
            _claim_stage(stage_id)
            if stage_id == "attest_r9_terminal_imports":
                _run_r9_attestation(activity, selected)
            elif stage_id == "fit_recovery_actor":
                _run_fit(activity, selected)
            elif stage_id == "freeze_recovery_actor":
                _run_candidate_freeze(selected)
            elif stage_id.startswith("development__"):
                _run_development(
                    stage_id.removeprefix("development__"), activity, selected
                )
            elif stage_id == "finalize_development":
                _run_final_development(activity, selected)
            else:  # pragma: no cover - the frozen contract owns this branch.
                raise V12R10ExecutionError(f"unknown stage: {stage_id}")
            completed.append(
                {
                    "stage_id": stage_id,
                    "receipt": _record(_stage_receipt_path(stage_id)),
                }
            )
            attempted_stage = None
    except BaseException as exc:
        error = exc
    passed = error is None and len(completed) == len(contract.STAGE_IDS)
    terminal = _terminal_payload(
        passed=passed,
        completed=completed,
        attempted_stage=attempted_stage,
        activity=activity,
        error=error,
    )
    _write(contract.LEDGER_PATH, terminal)
    verified = verify_terminal_ledger(
        fit_verifier=selected.verify_fit,
        source_verifier=selected.verify_r9_inputs,
    )
    if not passed:
        assert error is not None
        raise V12R10ExecutionError(
            f"V12R10 stopped terminally at {attempted_stage}: {error}"
        ) from error
    return verified


def preflight(*, adapters: RunnerAdapters | None = None) -> dict[str, Any]:
    selected = adapters or RunnerAdapters()
    freezer.verify_protocol_freeze()
    lock = freezer.verify_execution_lock(require_pristine=True)
    locked = freezer.attest_locked_inputs()
    imports = dict(selected.verify_r9_inputs())
    if (
        imports.get("passed") is not True
        or imports.get("import_only") is not True
        or imports.get("new_collection_rollout_count") != 0
        or imports.get("environment_reset_calls") != 0
        or imports.get("environment_step_calls") != 0
    ):
        raise V12R10ExecutionError("preflight R9 import closure failed")
    if len(contract.STAGE_IDS) != 10 or tuple(contract.STAGE_IDS[3:9]) != tuple(
        f"development__{case_id}" for case_id in contract.DEVELOPMENT_CASE_IDS
    ):
        raise V12R10ExecutionError("preflight stage order drifted")
    return {
        "passed": True,
        "status": "READY_H0_V12R10_RECOVERY_ONE_SHOT",
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "stage_order": list(contract.STAGE_IDS),
        "locked_inputs": locked,
        "r9_import_source_attestation": imports,
        "source_count": len(freezer.production_source_closure()),
        "runtime": lock["runtime"],
        "run_root_absent": not os.path.lexists(_path(contract.RUN_ROOT)),
        "one_shot": True,
        "import_only": True,
        "new_collection_rollout_count": 0,
        "single_actor_fit": True,
        "candidate_hidden_dims": [1024, 1024],
        "development_case_order": list(contract.DEVELOPMENT_CASE_IDS),
        "development_rollout_count": len(contract.DEVELOPMENT_CASE_IDS),
        "expected_development_steps": len(contract.DEVELOPMENT_CASE_IDS)
        * contract.EXPECTED_STEPS,
        "expected_raw_sensor_samples": len(contract.DEVELOPMENT_CASE_IDS)
        * contract.EXPECTED_STEPS
        * contract.RAW_SAMPLES_PER_STEP,
        "binary_detector_required": True,
        "morphology_weight": 0.0,
        "qualification_execution_authorized": False,
        "checkpoint_zero_authorized": False,
        "positive_morphology_authorized": False,
        "q3_paths_opened": [],
    }


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--preflight", action="store_true")
    mode.add_argument("--execute", action="store_true")
    mode.add_argument("--verify", action="store_true")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    if args.preflight:
        result = preflight()
    elif args.execute:
        result = execute()
    else:
        result = verify_terminal_ledger()
    print(json.dumps(result, indent=2, sort_keys=True, allow_nan=False))
    return 0 if result.get("passed") is True else 1


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = [
    "RunnerAdapters",
    "V12R10ExecutionError",
    "execute",
    "preflight",
    "verify_candidate_freeze_receipt",
    "verify_development_receipt",
    "verify_final_development_receipt",
    "verify_r9_attestation_receipt",
    "verify_terminal_ledger",
]
