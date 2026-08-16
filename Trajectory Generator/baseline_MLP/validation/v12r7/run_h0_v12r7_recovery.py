"""One-shot V12R7 recovery collection, fit and pure development runner.

The stage order is fixed by the execution-free contract: six R6 pure-policy
observer collections with offline H0 labels, one actor-only fit, semantic
five-file candidate freeze, six pure candidate developments, and one final
development receipt.  Any failed stage publishes a terminal ledger and closes
the lineage without retry, resume, sweep, or post-hoc repair.
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
    LOCAL_VALIDATION / "v12r3",
    LOCAL_VALIDATION / "v12r6",
    REVISION_ROOT,
):
    if str(_root) not in sys.path:
        sys.path.insert(0, str(_root))

import freeze_h0_v12r7_recovery as freezer  # noqa: E402
import h0_forensic_rollout as forensic  # noqa: E402
import h0_v12r6_physical_development as physical  # noqa: E402
import h0_v12r7_recovery_contract as contract  # noqa: E402
import h0_v12r7_recovery_fitter as fitter  # noqa: E402
import h0_v12r7_recovery_probe as probe  # noqa: E402


class V12R7ExecutionError(RuntimeError):
    """Raised after the one-shot R7 lineage has failed terminally."""


ACTIVITY_FIELDS = (
    "collection_rounds_completed",
    "environment_reset_calls",
    "environment_step_calls",
    "raw_sensor_sample_count",
    "offline_teacher_label_calls",
    "actor_fit_stage_calls",
    "actor_updates",
    "adamw_epochs_completed",
    "lbfgs_closure_calls",
    "development_rollouts_completed",
    "critic_updates",
    "ppo_updates",
)
EXPECTED_CANDIDATE_FILES = {
    "actor_feature_manifest.json",
    "candidate_build_manifest.json",
    "class_and_ctor_args.pkl",
    "metadata.json",
    "module_state.pkl",
}


@dataclass(frozen=True)
class RunnerAdapters:
    """Small injectable boundary used only by source/unit orchestration tests."""

    run_probe: Callable[..., Mapping[str, Any]] = probe.run_recovery_probe
    run_labels: Callable[..., Mapping[str, Any]] = probe.label_recovery_probe
    run_fit: Callable[..., Mapping[str, Any]] = fitter.run_fit_stage
    verify_fit: Callable[[], Mapping[str, Any]] = fitter.verify_fit_stage
    run_physical: Callable[..., Mapping[str, Any]] = physical.run_case


def _path(value: str | PurePath | Path) -> Path:
    if isinstance(value, Path) and value.is_absolute():
        target = Path(os.path.abspath(value))
        try:
            relative = target.relative_to(REPO_ROOT)
        except ValueError as exc:
            raise V12R7ExecutionError(f"path escaped repository: {target}") from exc
        raw = relative.as_posix()
    else:
        raw = value.as_posix() if isinstance(value, PurePath) else os.fspath(value)
    try:
        return freezer.safe_repository_path(raw)
    except Exception as exc:
        raise V12R7ExecutionError(f"unsafe repository path: {raw!r}") from exc


def _logical(path: str | PurePath | Path) -> str:
    return _path(path).relative_to(REPO_ROOT).as_posix()


def _record(path: str | PurePath | Path) -> dict[str, Any]:
    try:
        return freezer.artifact_record(_logical(path))
    except Exception as exc:
        raise V12R7ExecutionError(f"invalid artifact: {path}") from exc


def _tree(path: str | PurePath | Path) -> dict[str, Any]:
    try:
        return freezer.tree_record(_logical(path))
    except Exception as exc:
        raise V12R7ExecutionError(f"invalid artifact tree: {path}") from exc


def _mapping(path: str | PurePath | Path) -> dict[str, Any]:
    target = _path(path)
    try:
        _record(target)
        value = forensic.strict_json_load(target)
    except Exception as exc:
        raise V12R7ExecutionError(f"invalid strict JSON: {target}") from exc
    if not isinstance(value, Mapping):
        raise V12R7ExecutionError(f"expected JSON object: {target}")
    return dict(value)


def _sequence(path: str | PurePath | Path) -> list[Any]:
    target = _path(path)
    try:
        _record(target)
        value = forensic.strict_json_load(target)
    except Exception as exc:
        raise V12R7ExecutionError(f"invalid strict JSON: {target}") from exc
    if not isinstance(value, list):
        raise V12R7ExecutionError(f"expected JSON array: {target}")
    return value


def _write(
    path: str | PurePath | Path,
    payload: Mapping[str, Any] | Sequence[Any],
) -> dict[str, Any]:
    target = _path(path)
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


def _artifact_exact(value: Any) -> bool:
    if not isinstance(value, Mapping):
        return False
    try:
        return _strict_equal(dict(value), _record(str(value["path"])))
    except (KeyError, TypeError, V12R7ExecutionError):
        return False


def _zero_activity() -> dict[str, int]:
    return {name: 0 for name in ACTIVITY_FIELDS}


def _increment(activity: dict[str, int], name: str, amount: int = 1) -> None:
    if name not in activity:
        raise V12R7ExecutionError(f"unknown activity counter: {name}")
    if type(amount) is not int or amount < 0:
        raise V12R7ExecutionError("activity increment must be non-negative int")
    current = activity[name]
    if type(current) is not int or current < 0:
        raise V12R7ExecutionError("activity counter drifted")
    activity[name] = current + amount


def _claims_root() -> Path:
    return _path(contract.RUN_ROOT) / "claims"


def _stage_claim_path(stage_id: str) -> Path:
    try:
        index = tuple(contract.STAGE_IDS).index(stage_id) + 1
    except ValueError as exc:
        raise V12R7ExecutionError(f"unknown stage id: {stage_id}") from exc
    if "/" in stage_id or "\\" in stage_id or ".." in stage_id:
        raise V12R7ExecutionError("unsafe stage identifier")
    return _claims_root() / f"{index:02d}_{stage_id}.json"


def _claim_stage(stage_id: str) -> dict[str, Any]:
    payload = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": "CLAIMED_H0_V12R7_STAGE_ONCE",
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
        "alpha_sweep_authorized": False,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
    }
    _write(_stage_claim_path(stage_id), payload)
    return payload


def _collection_stage_receipt_path(case_id: str) -> Path:
    contract.canonical_collection_case(case_id)
    return _path(contract.collection_case_root(case_id)) / "stage_receipt.json"


def _development_receipt_path(case_id: str) -> Path:
    contract.canonical_development_case(case_id)
    return _path(contract.DEVELOPMENT_ROOT) / case_id / "receipt.json"


def _run_collection_label(
    case_id: str,
    activity: dict[str, int],
    adapters: RunnerAdapters,
) -> dict[str, Any]:
    stage_id = f"collect_label__{case_id}"
    worker_claim = _record(_stage_claim_path(stage_id))
    case_root = _path(contract.collection_case_root(case_id))
    config = probe.RecoveryProbeConfig(case_id=case_id, artifact_root=REPO_ROOT)
    result = adapters.run_probe(
        config=config,
        destination=case_root,
        module_path=_path(contract.R6_CANDIDATE_MODULE_PATH),
        activity_callback=lambda name, amount: _increment(activity, name, amount),
    )
    if result.get("passed") is not True:
        raise V12R7ExecutionError(f"R7 collection probe failed: {case_id}")
    closed = probe.verify_probe_closure(case_root, config=config)
    label_root = case_root / "observer_labels"
    labelled = adapters.run_labels(
        config=config,
        probe_destination=case_root,
        label_destination=label_root,
        source_h0_path=_path(contract.SOURCE_H0_MODULE_PATH),
    )
    if labelled.get("passed") is not True:
        raise V12R7ExecutionError(f"R7 observer labels failed: {case_id}")
    label_summary = _mapping(label_root / "summary.json")
    label_gate = _mapping(label_root / "gate.json")
    if (
        contract.label_gate(label_summary) != label_gate
        or label_gate.get("passed") is not True
    ):
        raise V12R7ExecutionError(f"R7 observer label gate drifted: {case_id}")
    rows = label_summary.get("labelled_row_count")
    if type(rows) is not int or rows < contract.MINIMUM_RECOVERABLE_PREFIX_STEPS:
        raise V12R7ExecutionError("observer label row count drifted")
    _increment(activity, "offline_teacher_label_calls", rows)
    _increment(activity, "collection_rounds_completed")
    receipt = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": "PASS_H0_V12R7_COLLECTION_AND_LABEL_STAGE",
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "stage_id": stage_id,
        "case_id": case_id,
        "candidate_tree_sha256": contract.LOCKED_INPUTS["r6_candidate"]["tree_sha256"],
        "probe_step_count": closed["replay"].n_steps,
        "labelled_row_count": rows,
        "same_state_teacher_label_count": rows,
        "teacher_query_count": rows,
        "probe_receipt": _record(case_root / "receipt.json"),
        "probe_gate": _record(case_root / "gate.json"),
        "probe_replay": _record(case_root / "replay_boundaries.npz"),
        "labels": _record(label_root / "labels.npz"),
        "label_summary": _record(label_root / "summary.json"),
        "label_gate": _record(label_root / "gate.json"),
        "label_receipt": _record(label_root / "receipt.json"),
        "pipeline_claim": _record(contract.CLAIM_PATH),
        "worker_claim": worker_claim,
        "single_collection_round": True,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
    }
    _write(_collection_stage_receipt_path(case_id), receipt)
    return verify_collection_label_receipt(case_id)


def verify_collection_label_receipt(case_id: str) -> dict[str, Any]:
    stage_id = f"collect_label__{case_id}"
    root = _path(contract.collection_case_root(case_id))
    config = probe.RecoveryProbeConfig(case_id=case_id, artifact_root=REPO_ROOT)
    closed = probe.verify_probe_closure(root, config=config)
    label_root = root / "observer_labels"
    summary = _mapping(label_root / "summary.json")
    gate = _mapping(label_root / "gate.json")
    label_receipt = _mapping(label_root / "receipt.json")
    receipt = _mapping(_collection_stage_receipt_path(case_id))
    records = {
        "probe_receipt": root / "receipt.json",
        "probe_gate": root / "gate.json",
        "probe_replay": root / "replay_boundaries.npz",
        "labels": label_root / "labels.npz",
        "label_summary": label_root / "summary.json",
        "label_gate": label_root / "gate.json",
        "label_receipt": label_root / "receipt.json",
        "pipeline_claim": contract.CLAIM_PATH,
        "worker_claim": _stage_claim_path(stage_id),
    }
    checks = {
        "identity": receipt.get("status") == "PASS_H0_V12R7_COLLECTION_AND_LABEL_STAGE"
        and receipt.get("passed") is True
        and receipt.get("protocol_id") == contract.PROTOCOL_ID
        and receipt.get("pipeline_id") == contract.PIPELINE_ID
        and receipt.get("stage_id") == stage_id
        and receipt.get("case_id") == case_id,
        "probe": closed.get("passed") is True
        and receipt.get("probe_step_count") == closed["replay"].n_steps,
        "label_gate": gate == contract.label_gate(summary)
        and gate.get("passed") is True
        and label_receipt.get("passed") is True,
        "same_state": type(receipt.get("labelled_row_count")) is int
        and receipt.get("same_state_teacher_label_count")
        == receipt.get("labelled_row_count")
        and receipt.get("teacher_query_count") == receipt.get("labelled_row_count"),
        "records": all(
            receipt.get(name) == _record(path) for name, path in records.items()
        ),
        "one_round": receipt.get("single_collection_round") is True,
        "zero_updates": all(
            type(receipt.get(name)) is int and receipt[name] == 0
            for name in ("actor_updates", "critic_updates", "ppo_updates")
        ),
    }
    if not all(value is True for value in checks.values()):
        failed = sorted(name for name, value in checks.items() if not value)
        raise V12R7ExecutionError(f"collection/label receipt drifted: {failed}")
    return receipt


def _run_fit(activity: dict[str, int], adapters: RunnerAdapters) -> dict[str, Any]:
    stage_id = "fit_recovery_actor"
    _increment(activity, "actor_fit_stage_calls")
    receipt = adapters.run_fit(
        pipeline_claim_path=_path(contract.CLAIM_PATH),
        worker_claim_path=_stage_claim_path(stage_id),
        protocol_freeze_path=_path(contract.PROTOCOL_FREEZE_PATH),
        execution_lock_path=_path(contract.EXECUTION_LOCK_PATH),
        activity_callback=lambda name, amount: _increment(activity, name, amount),
    )
    if receipt.get("passed") is not True:
        raise V12R7ExecutionError("R7 actor fit receipt is not PASS")
    verified = dict(adapters.verify_fit())
    if not _strict_equal(receipt, verified):
        raise V12R7ExecutionError("R7 actor fit verifier disagrees")
    return verified


def _candidate_semantic_audit(
    module: Mapping[str, Any],
    actor_manifest: Mapping[str, Any],
    build_manifest: Mapping[str, Any],
) -> dict[str, Any]:
    file_names = {
        row.get("path") for row in module.get("files", []) if isinstance(row, Mapping)
    }
    checks = {
        "five_file_tree": module.get("file_count") == 5
        and file_names == EXPECTED_CANDIDATE_FILES,
        "actor_manifest": actor_manifest.get("status")
        == contract.ACTOR_FEATURE_MANIFEST_STATUS
        and actor_manifest.get("topology_id") == contract.TOPOLOGY_ID
        and actor_manifest.get("fit_contract_id") == contract.FIT_CONTRACT_ID
        and actor_manifest.get("actor_feature_count")
        == contract.EXPECTED_ACTOR_FEATURES
        and actor_manifest.get("fcnet_hiddens") == [512, 512]
        and actor_manifest.get("disabled_clock_columns") == [0, 1],
        "build_manifest": build_manifest.get("passed") is True
        and build_manifest.get("protocol_id") == contract.PROTOCOL_ID
        and build_manifest.get("fit_contract_id") == contract.FIT_CONTRACT_ID
        and build_manifest.get("topology_id") == contract.TOPOLOGY_ID
        and build_manifest.get("architecture") == contract.FIT["architecture"],
        "single_update": type(build_manifest.get("actor_fit_count")) is int
        and build_manifest.get("actor_fit_count") == 1
        and type(build_manifest.get("actor_updates")) is int
        and build_manifest.get("actor_updates") == 1
        and type(build_manifest.get("critic_updates")) is int
        and build_manifest.get("critic_updates") == 0
        and type(build_manifest.get("ppo_updates")) is int
        and build_manifest.get("ppo_updates") == 0,
        "preservation": build_manifest.get("logstd_byte_exact") is True
        and build_manifest.get("disabled_clock_columns_bit_zero") is True
        and build_manifest.get("save_reload_exact") is True,
        "digest_bound": actor_manifest.get("actor_digest")
        == build_manifest.get("actor_digest")
        and actor_manifest.get("module_state_sha256")
        == build_manifest.get("module_state_sha256"),
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
        raise V12R7ExecutionError("R7 candidate semantic freeze failed")
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
        "warm_start_target_512_compatible": True,
        "actor_fit_count": 1,
        "runtime_promoted": False,
        "qualification_executed": False,
        "q3_paths_opened": [],
        "actor_updates": 1,
        "critic_updates": 0,
        "ppo_updates": 0,
    }
    if fit_receipt.get("candidate_id") != identity:
        raise V12R7ExecutionError("fit/freeze candidate identity drifted")
    _write(contract.CANDIDATE_FREEZE_PATH, receipt)
    return verify_candidate_freeze_receipt(fit_verifier=adapters.verify_fit)


def verify_candidate_freeze_receipt(
    *, fit_verifier: Callable[[], Mapping[str, Any]] | None = None
) -> dict[str, Any]:
    verify_fit = fitter.verify_fit_stage if fit_verifier is None else fit_verifier
    fit_receipt = dict(verify_fit())
    receipt = _mapping(contract.CANDIDATE_FREEZE_PATH)
    module = _tree(contract.CANDIDATE_MODULE_PATH)
    identity = contract.candidate_id(module["tree_sha256"])
    actor_path = _path(contract.CANDIDATE_MODULE_PATH) / "actor_feature_manifest.json"
    build_path = _path(contract.CANDIDATE_MODULE_PATH) / "candidate_build_manifest.json"
    semantic = _candidate_semantic_audit(
        module, _mapping(actor_path), _mapping(build_path)
    )
    checks = {
        "identity": receipt.get("status") == contract.CANDIDATE_FREEZE_PASS_STATUS
        and receipt.get("passed") is True
        and receipt.get("protocol_id") == contract.PROTOCOL_ID
        and receipt.get("pipeline_id") == contract.PIPELINE_ID
        and receipt.get("stage_id") == "freeze_recovery_actor",
        "candidate": receipt.get("candidate_id") == identity
        and receipt.get("candidate_module") == module
        and fit_receipt.get("candidate_id") == identity,
        "fit": receipt.get("fit_receipt") == _record(contract.FIT_RECEIPT_PATH),
        "manifests": receipt.get("actor_feature_manifest") == _record(actor_path)
        and receipt.get("candidate_build_manifest") == _record(build_path)
        and receipt.get("actor_digest") == _mapping(actor_path).get("actor_digest"),
        "semantic": semantic.get("passed") is True
        and receipt.get("semantic_audit") == semantic,
        "claims": receipt.get("pipeline_claim") == _record(contract.CLAIM_PATH)
        and receipt.get("worker_claim")
        == _record(_stage_claim_path("freeze_recovery_actor")),
        "closed": receipt.get("candidate_frozen") is True
        and receipt.get("fit_gate_passed") is True
        and receipt.get("standard_actor") is True
        and receipt.get("warm_start_target_512_compatible") is True
        and type(receipt.get("actor_fit_count")) is int
        and receipt.get("actor_fit_count") == 1
        and receipt.get("q3_paths_opened") == []
        and receipt.get("runtime_promoted") is False
        and receipt.get("qualification_executed") is False,
        "counters": type(receipt.get("actor_updates")) is int
        and receipt.get("actor_updates") == 1
        and type(receipt.get("critic_updates")) is int
        and receipt.get("critic_updates") == 0
        and type(receipt.get("ppo_updates")) is int
        and receipt.get("ppo_updates") == 0,
    }
    if not all(value is True for value in checks.values()):
        failed = sorted(name for name, value in checks.items() if not value)
        raise V12R7ExecutionError(f"candidate freeze receipt drifted: {failed}")
    return receipt


def _physical_config() -> physical.PhysicalDevelopmentConfig:
    return physical.PhysicalDevelopmentConfig(
        protocol_id=contract.PROTOCOL_ID,
        start_status="STARTED_H0_V12R7_PURE_DEVELOPMENT",
        partial_status="PERSISTED_H0_V12R7_DEVELOPMENT_BEFORE_GATE",
        complete_status="COMPLETE_H0_V12R7_PURE_DEVELOPMENT",
        schema_version=contract.SCHEMA_VERSION,
        artifact_root=REPO_ROOT,
        progress_label="V12R7 pure development",
        progress_every=25,
    )


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
    result = adapters.run_physical(
        config=_physical_config(),
        case=case,
        destination=destination,
        module_path=_path(contract.CANDIDATE_MODULE_PATH),
        activity_callback=lambda name, amount: _increment(activity, name, amount),
        start_metadata={
            "pipeline_claim": _record(contract.CLAIM_PATH),
            "worker_claim": _record(_stage_claim_path(f"development__{case_id}")),
            "execution_lock": _record(contract.EXECUTION_LOCK_PATH),
            "candidate_id": identity,
            "candidate_module": module,
            "candidate_freeze": _record(contract.CANDIDATE_FREEZE_PATH),
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
    summary = result.get("summary")
    if not isinstance(summary, Mapping):
        raise V12R7ExecutionError("development summary is malformed")
    runtime_checks = {
        "pure_trace": isinstance(summary.get("pure_policy_trace_audit"), Mapping)
        and summary["pure_policy_trace_audit"].get("passed") is True,
        "detector_active": summary.get("binary_phase_fsm_mode") == "binary_active"
        and summary.get("event_contract_id") == contract.EVENT_CONTRACT_ID
        and summary.get("target_contract_id") == contract.TARGET_CONTRACT_ID,
        "morphology_zero": isinstance(summary.get("morphology_weight"), (int, float))
        and not isinstance(summary.get("morphology_weight"), bool)
        and float(summary["morphology_weight"]) == 0.0,
        "zero_updates": all(
            type(summary.get(name)) is int and summary[name] == 0
            for name in ("actor_updates", "critic_updates", "ppo_updates")
        ),
    }
    if not all(value is True for value in runtime_checks.values()):
        failed = sorted(name for name, value in runtime_checks.items() if not value)
        raise V12R7ExecutionError(f"R7 development runtime integrity failed: {failed}")
    gate = contract.development_gate(summary, case_id=case_id)
    _write(destination / "gate.json", gate)
    if gate.get("passed") is not True:
        raise V12R7ExecutionError(f"R7 development gate failed: {case_id}")
    receipt = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": "PASS_H0_V12R7_PURE_DEVELOPMENT_RECEIPT",
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "stage_id": f"development__{case_id}",
        "case_id": case_id,
        "candidate_id": identity,
        "candidate_module": module,
        "candidate_freeze": _record(contract.CANDIDATE_FREEZE_PATH),
        "summary": _record(destination / "summary.json"),
        "gate": _record(destination / "gate.json"),
        "trace": _record(destination / "trace.json"),
        "pure_policy_trace_audit": copy.deepcopy(
            summary.get("pure_policy_trace_audit")
        ),
        "runtime_integrity_checks": runtime_checks,
        "teacher_query_count": 0,
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
    verify_candidate_freeze_receipt(fit_verifier=fit_verifier)
    root = _path(contract.DEVELOPMENT_ROOT) / case_id
    receipt = _mapping(root / "receipt.json")
    summary = _mapping(root / "summary.json")
    gate = _mapping(root / "gate.json")
    module = _tree(contract.CANDIDATE_MODULE_PATH)
    identity = contract.candidate_id(module["tree_sha256"])
    expected_gate = contract.development_gate(summary, case_id=case_id)
    audit = summary.get("pure_policy_trace_audit")
    runtime_checks = receipt.get("runtime_integrity_checks")
    checks = {
        "identity": receipt.get("status") == "PASS_H0_V12R7_PURE_DEVELOPMENT_RECEIPT"
        and receipt.get("passed") is True
        and receipt.get("protocol_id") == contract.PROTOCOL_ID
        and receipt.get("pipeline_id") == contract.PIPELINE_ID
        and receipt.get("stage_id") == f"development__{case_id}"
        and receipt.get("case_id") == case_id,
        "candidate": receipt.get("candidate_id") == identity
        and receipt.get("candidate_module") == module
        and receipt.get("candidate_freeze") == _record(contract.CANDIDATE_FREEZE_PATH),
        "gate": gate == expected_gate and gate.get("passed") is True,
        "trace_audit": isinstance(audit, Mapping)
        and audit.get("passed") is True
        and receipt.get("pure_policy_trace_audit") == audit,
        "runtime_integrity": isinstance(runtime_checks, Mapping)
        and set(runtime_checks)
        == {"pure_trace", "detector_active", "morphology_zero", "zero_updates"}
        and all(value is True for value in runtime_checks.values())
        and summary.get("binary_phase_fsm_mode") == "binary_active"
        and summary.get("event_contract_id") == contract.EVENT_CONTRACT_ID
        and summary.get("target_contract_id") == contract.TARGET_CONTRACT_ID,
        "artifacts": receipt.get("summary") == _record(root / "summary.json")
        and receipt.get("gate") == _record(root / "gate.json")
        and receipt.get("trace") == _record(root / "trace.json"),
        "pure_zero": all(
            type(receipt.get(name)) is int and receipt[name] == 0
            for name in (
                "teacher_query_count",
                "actor_updates",
                "critic_updates",
                "ppo_updates",
            )
        ),
    }
    if not all(value is True for value in checks.values()):
        failed = sorted(name for name, value in checks.items() if not value)
        raise V12R7ExecutionError(f"development receipt drifted: {failed}")
    return receipt


def _run_final_development(
    activity: Mapping[str, int],
    adapters: RunnerAdapters,
) -> dict[str, Any]:
    freeze = verify_candidate_freeze_receipt(fit_verifier=adapters.verify_fit)
    module = _tree(contract.CANDIDATE_MODULE_PATH)
    identity = contract.candidate_id(module["tree_sha256"])
    bindings = []
    for case_id in contract.DEVELOPMENT_CASE_IDS:
        receipt = verify_development_receipt(case_id, fit_verifier=adapters.verify_fit)
        root = _path(contract.DEVELOPMENT_ROOT) / case_id
        bindings.append(
            {
                "case_id": case_id,
                "passed": True,
                "receipt": _record(root / "receipt.json"),
                "summary": receipt["summary"],
                "gate": receipt["gate"],
            }
        )
    if activity.get("collection_rounds_completed") != len(contract.COLLECTION_CASE_IDS):
        raise V12R7ExecutionError("collection round count drifted")
    if activity.get("actor_fit_stage_calls") != 1 or activity.get("actor_updates") != 1:
        raise V12R7ExecutionError("sole actor fit count drifted")
    if activity.get("development_rollouts_completed") != len(
        contract.DEVELOPMENT_CASE_IDS
    ):
        raise V12R7ExecutionError("development rollout count drifted")
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
        "rollout_bindings": bindings,
        "case_gates": [
            {"case_id": case_id, "passed": True}
            for case_id in contract.DEVELOPMENT_CASE_IDS
        ],
        "collection_round_count": len(contract.COLLECTION_CASE_IDS),
        "actor_fit_count": 1,
        "actor_updates": 1,
        "development_rollout_count": len(contract.DEVELOPMENT_CASE_IDS),
        "development_count": len(contract.DEVELOPMENT_CASE_IDS),
        "passing_development_count": len(contract.DEVELOPMENT_CASE_IDS),
        "failed_development_count": 0,
        "development_actor_updates": 0,
        "development_critic_updates": 0,
        "development_ppo_updates": 0,
        "teacher_query_count": 0,
        "mean_blend_count": 0,
        "safety_intervention_count": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
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
        "alpha_sweep_authorized": False,
    }
    _write(contract.FINAL_DEVELOPMENT_PATH, receipt)
    return verify_final_development_receipt(fit_verifier=adapters.verify_fit)


def verify_final_development_receipt(
    *, fit_verifier: Callable[[], Mapping[str, Any]] | None = None
) -> dict[str, Any]:
    freeze = verify_candidate_freeze_receipt(fit_verifier=fit_verifier)
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
    checks = {
        "identity": receipt.get("status") == contract.DEVELOPMENT_PASS_STATUS
        and receipt.get("passed") is True
        and receipt.get("protocol_id") == contract.PROTOCOL_ID
        and receipt.get("pipeline_id") == contract.PIPELINE_ID
        and receipt.get("stage_id") == "finalize_development",
        "candidate": receipt.get("candidate_id") == identity
        and receipt.get("candidate_module") == module
        and receipt.get("candidate_freeze") == _record(contract.CANDIDATE_FREEZE_PATH)
        and receipt.get("fit_candidate_id") == freeze.get("candidate_id"),
        "rollouts": receipt.get("rollout_bindings") == expected_bindings
        and receipt.get("development_rollout_count")
        == len(contract.DEVELOPMENT_CASE_IDS),
        "case_gates": receipt.get("case_gates")
        == [
            {"case_id": case_id, "passed": True}
            for case_id in contract.DEVELOPMENT_CASE_IDS
        ]
        and type(receipt.get("development_count")) is int
        and receipt.get("development_count") == len(contract.DEVELOPMENT_CASE_IDS)
        and type(receipt.get("passing_development_count")) is int
        and receipt.get("passing_development_count")
        == len(contract.DEVELOPMENT_CASE_IDS)
        and type(receipt.get("failed_development_count")) is int
        and receipt.get("failed_development_count") == 0,
        "single_collection_fit": receipt.get("collection_round_count")
        == len(contract.COLLECTION_CASE_IDS)
        and type(receipt.get("actor_fit_count")) is int
        and receipt.get("actor_fit_count") == 1
        and type(receipt.get("actor_updates")) is int
        and receipt.get("actor_updates") == 1,
        "zero_forbidden_updates": type(receipt.get("critic_updates")) is int
        and receipt.get("critic_updates") == 0
        and type(receipt.get("ppo_updates")) is int
        and receipt.get("ppo_updates") == 0,
        "pure_development_counters": all(
            type(receipt.get(name)) is int and receipt[name] == 0
            for name in (
                "development_actor_updates",
                "development_critic_updates",
                "development_ppo_updates",
                "teacher_query_count",
                "mean_blend_count",
                "safety_intervention_count",
            )
        ),
        "development_only": receipt.get("qualification_executed") is False
        and receipt.get("runtime_promoted") is False
        and receipt.get("checkpoint_zero_created") is False
        and receipt.get("positive_morphology_enabled") is False,
        "claims": receipt.get("pipeline_claim") == _record(contract.CLAIM_PATH)
        and receipt.get("worker_claim")
        == _record(_stage_claim_path("finalize_development"))
        and receipt.get("fit_receipt") == _record(contract.FIT_RECEIPT_PATH),
        "closed": receipt.get("retry_authorized") is False
        and receipt.get("resume_authorized") is False
        and receipt.get("alpha_sweep_authorized") is False
        and receipt.get("q3_paths_opened") == [],
    }
    if not all(value is True for value in checks.values()):
        failed = sorted(name for name, value in checks.items() if not value)
        raise V12R7ExecutionError(f"final development receipt drifted: {failed}")
    return receipt


def _terminal_payload(
    *,
    passed: bool,
    completed: Sequence[Mapping[str, Any]],
    attempted_stage: str | None,
    activity: Mapping[str, int],
    error: BaseException | None,
) -> dict[str, Any]:
    candidate = None
    candidate_id = None
    candidate_observation: dict[str, Any] = {"state": "ABSENT", "tree": None}
    candidate_path = _path(contract.CANDIDATE_MODULE_PATH)
    if candidate_path.is_dir() and not candidate_path.is_symlink():
        try:
            candidate = _tree(candidate_path)
            candidate_id = contract.candidate_id(candidate["tree_sha256"])
            candidate_observation = {"state": "VALID_NONEMPTY_TREE", "tree": candidate}
        except BaseException as exc:
            candidate_observation = {
                "state": "PARTIAL_OR_UNSAFE_TREE",
                "tree": None,
                "error": {"type": type(exc).__name__, "message": str(exc)},
            }
    elif os.path.lexists(candidate_path):
        candidate_observation = {"state": "UNSAFE_NONDIRECTORY", "tree": None}
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": (
            contract.PIPELINE_TERMINAL_PASS_STATUS
            if passed
            else "FAIL_H0_V12R7_RECOVERY_PIPELINE_TERMINAL"
        ),
        "passed": passed,
        "terminal": True,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "candidate_selection_rule": contract.CANDIDATE_SELECTION_RULE,
        "candidate_id": candidate_id,
        "candidate_module": candidate,
        "candidate_module_observation": candidate_observation,
        "stage_order": list(contract.STAGE_IDS),
        "completed_stages": list(completed),
        "completed_stage_count": len(completed),
        "attempted_stage": attempted_stage,
        "activity_totals": dict(activity),
        "collection_round_count": sum(
            item.get("stage_id", "").startswith("collect_label__") for item in completed
        ),
        "actor_fit_count": sum(
            item.get("stage_id") == "fit_recovery_actor" for item in completed
        ),
        "development_count": sum(
            item.get("stage_id", "").startswith("development__") for item in completed
        ),
        "protocol_freeze": _record(contract.PROTOCOL_FREEZE_PATH),
        "execution_lock": _record(contract.EXECUTION_LOCK_PATH),
        "pipeline_claim": _record(contract.CLAIM_PATH),
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
        "retry_authorized": False,
        "resume_authorized": False,
        "alpha_sweep_authorized": False,
        "error": (
            None
            if error is None
            else {"type": type(error).__name__, "message": str(error) or repr(error)}
        ),
        "next_stage": (
            "WAIT_SEPARATE_V12R7Q3_PROTOCOL" if passed else "STOP_TERMINAL_NO_RETRY"
        ),
    }


def verify_terminal_ledger(
    *, fit_verifier: Callable[[], Mapping[str, Any]] | None = None
) -> dict[str, Any]:
    ledger = _mapping(contract.LEDGER_PATH)
    if type(ledger.get("passed")) is not bool:
        raise V12R7ExecutionError("terminal ledger passed flag drifted")
    passed = ledger["passed"]
    expected_status = (
        contract.PIPELINE_TERMINAL_PASS_STATUS
        if passed
        else "FAIL_H0_V12R7_RECOVERY_PIPELINE_TERMINAL"
    )
    checks = {
        "identity": ledger.get("status") == expected_status
        and ledger.get("terminal") is True
        and ledger.get("protocol_id") == contract.PROTOCOL_ID
        and ledger.get("pipeline_id") == contract.PIPELINE_ID,
        "bindings": ledger.get("protocol_freeze")
        == _record(contract.PROTOCOL_FREEZE_PATH)
        and ledger.get("execution_lock") == _record(contract.EXECUTION_LOCK_PATH)
        and ledger.get("pipeline_claim") == _record(contract.CLAIM_PATH),
        "closed": ledger.get("retry_authorized") is False
        and ledger.get("resume_authorized") is False
        and ledger.get("alpha_sweep_authorized") is False
        and ledger.get("runtime_promoted") is False
        and ledger.get("qualification_executed") is False
        and ledger.get("q3_paths_opened") == [],
        "counter_types": all(
            type(ledger.get(name)) is int and ledger[name] >= 0
            for name in ("actor_updates", "critic_updates", "ppo_updates")
        ),
        "activity_schema": isinstance(ledger.get("activity_totals"), Mapping)
        and set(ledger["activity_totals"]) == set(ACTIVITY_FIELDS)
        and all(
            type(ledger["activity_totals"].get(name)) is int
            and ledger["activity_totals"][name] >= 0
            for name in ACTIVITY_FIELDS
        ),
    }
    completed = ledger.get("completed_stages")
    count = ledger.get("completed_stage_count")
    if (
        not isinstance(completed, list)
        or type(count) is not int
        or count != len(completed)
    ):
        raise V12R7ExecutionError("terminal completed-stage prefix drifted")
    for index, item in enumerate(completed):
        if (
            not isinstance(item, Mapping)
            or item.get("stage_id") != contract.STAGE_IDS[index]
            or not _artifact_exact(item.get("receipt"))
        ):
            raise V12R7ExecutionError("terminal stage receipt binding drifted")
    if not all(value is True for value in checks.values()):
        failed = sorted(name for name, value in checks.items() if not value)
        raise V12R7ExecutionError(f"terminal ledger drifted: {failed}")
    if passed:
        if (
            len(completed) != len(contract.STAGE_IDS)
            or ledger.get("attempted_stage") is not None
            or ledger.get("error") is not None
            or ledger.get("collection_round_count") != len(contract.COLLECTION_CASE_IDS)
            or ledger.get("actor_fit_count") != 1
            or ledger.get("development_count") != len(contract.DEVELOPMENT_CASE_IDS)
            or ledger.get("actor_updates") != 1
            or ledger.get("critic_updates") != 0
            or ledger.get("ppo_updates") != 0
            or ledger.get("candidate_freeze") != _record(contract.CANDIDATE_FREEZE_PATH)
            or ledger.get("final_development_receipt")
            != _record(contract.FINAL_DEVELOPMENT_PATH)
        ):
            raise V12R7ExecutionError("terminal PASS closure drifted")
        verify_candidate_freeze_receipt(fit_verifier=fit_verifier)
        verify_final_development_receipt(fit_verifier=fit_verifier)
    elif not isinstance(ledger.get("error"), Mapping):
        raise V12R7ExecutionError("terminal FAIL error evidence is missing")
    return ledger


def _stage_receipt_path(stage_id: str) -> Path:
    if stage_id.startswith("collect_label__"):
        return _collection_stage_receipt_path(stage_id.removeprefix("collect_label__"))
    if stage_id == "fit_recovery_actor":
        return _path(contract.FIT_RECEIPT_PATH)
    if stage_id == "freeze_recovery_actor":
        return _path(contract.CANDIDATE_FREEZE_PATH)
    if stage_id.startswith("development__"):
        return _development_receipt_path(stage_id.removeprefix("development__"))
    if stage_id == "finalize_development":
        return _path(contract.FINAL_DEVELOPMENT_PATH)
    raise V12R7ExecutionError(f"unknown stage: {stage_id}")


def execute(*, adapters: RunnerAdapters | None = None) -> dict[str, Any]:
    selected = adapters or RunnerAdapters()
    ledger_path = _path(contract.LEDGER_PATH)
    if os.path.lexists(ledger_path):
        return verify_terminal_ledger(fit_verifier=selected.verify_fit)
    freezer.verify_protocol_freeze()
    freezer.verify_execution_lock(require_pristine=True)
    run_root = _path(contract.RUN_ROOT)
    if os.path.lexists(run_root):
        raise V12R7ExecutionError("R7 run root occupied; resume is forbidden")
    os.mkdir(run_root, 0o700)
    os.mkdir(_claims_root(), 0o700)
    claim = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": "CLAIMED_H0_V12R7_RECOVERY_PIPELINE_ONCE",
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "authority": copy.deepcopy(contract.AUTHORITY),
        "stage_order": list(contract.STAGE_IDS),
        "protocol_freeze": _record(contract.PROTOCOL_FREEZE_PATH),
        "execution_lock": _record(contract.EXECUTION_LOCK_PATH),
        "one_shot": True,
        "single_collection_round": True,
        "single_actor_fit": True,
        "retry_authorized": False,
        "resume_authorized": False,
        "alpha_sweep_authorized": False,
        "qualification_execution_authorized": False,
    }
    _write(contract.CLAIM_PATH, claim)
    activity = _zero_activity()
    completed: list[dict[str, Any]] = []
    attempted_stage: str | None = None
    error: BaseException | None = None
    try:
        for stage_id in contract.STAGE_IDS:
            attempted_stage = stage_id
            _claim_stage(stage_id)
            if stage_id.startswith("collect_label__"):
                _run_collection_label(
                    stage_id.removeprefix("collect_label__"), activity, selected
                )
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
            else:  # pragma: no cover - contract self-check owns this branch.
                raise V12R7ExecutionError(f"unknown stage: {stage_id}")
            receipt_path = _stage_receipt_path(stage_id)
            completed.append({"stage_id": stage_id, "receipt": _record(receipt_path)})
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
    verified = verify_terminal_ledger(fit_verifier=selected.verify_fit)
    if not passed:
        assert error is not None
        raise V12R7ExecutionError(
            f"V12R7 stopped terminally at {attempted_stage}: {error}"
        ) from error
    return verified


def preflight() -> dict[str, Any]:
    freezer.verify_protocol_freeze()
    freezer.verify_execution_lock(require_pristine=True)
    locked = freezer.attest_locked_inputs()
    label_paths_match = all(
        fitter.observer_labels_path(case_id) == contract.observer_label_path(case_id)
        for case_id in contract.COLLECTION_CASE_IDS
    )
    if not label_paths_match:
        raise V12R7ExecutionError("fitter/probe observer label path contract drifted")
    return {
        "passed": True,
        "status": "READY_H0_V12R7_RECOVERY_ONE_SHOT",
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "stage_order": list(contract.STAGE_IDS),
        "locked_inputs": locked,
        "source_count": len(freezer.production_source_closure()),
        "runtime": freezer.verify_execution_lock(require_pristine=True)["runtime"],
        "run_root_absent": not os.path.lexists(_path(contract.RUN_ROOT)),
        "single_collection_round": True,
        "single_actor_fit": True,
        "qualification_execution_authorized": False,
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
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = [
    "RunnerAdapters",
    "V12R7ExecutionError",
    "execute",
    "preflight",
    "verify_candidate_freeze_receipt",
    "verify_collection_label_receipt",
    "verify_development_receipt",
    "verify_final_development_receipt",
    "verify_terminal_ledger",
]
