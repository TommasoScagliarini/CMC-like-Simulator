"""Execute the exact V12R6 composite pipeline once, fail closed.

The runner attests immutable P2/R5 forensic inputs, synthesizes one standard
512-wide actor, evaluates every locked offline slice, freezes the sole passing
candidate, and executes six pure-policy physical development cases in the
registered critical-first order.  It cannot fit, collect labels, qualify,
promote, retry, resume, or enable a positive morphology reward.
"""

from __future__ import annotations

import argparse
import copy
import json
import math
import os
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
for _root in (
    REPO_ROOT,
    REPO_ROOT / "validation",
    REPO_ROOT / "Trajectory Generator" / "baseline_MLP",
    REVISION_ROOT.parent / "v12r5",
    REVISION_ROOT,
):
    if str(_root) not in sys.path:
        sys.path.insert(0, str(_root))

import h0_forensic_rollout as forensic  # noqa: E402
import build_h0_v12r6_composite_actor as composite  # noqa: E402
import freeze_h0_v12r6_functional_composite as freezer  # noqa: E402
import h0_v12r6_functional_composite_contract as contract  # noqa: E402
import h0_v12r6_physical_development as physical  # noqa: E402


class V12R6ExecutionError(RuntimeError):
    """Raised after any one-shot V12R6 invariant fails closed."""


def resolve_relative(path: str | os.PathLike[str] | PurePosixPath) -> Path:
    raw = path.as_posix() if isinstance(path, PurePosixPath) else os.fspath(path)
    pure = PurePosixPath(raw)
    if (
        not raw
        or "\\" in raw
        or pure.is_absolute()
        or ".." in pure.parts
        or pure.as_posix() != raw
    ):
        raise V12R6ExecutionError(f"non-canonical repository path: {raw!r}")
    return REPO_ROOT.joinpath(*pure.parts)


RUN_ROOT = resolve_relative(contract.RUN_ROOT)
PIPELINE_CLAIM_PATH = resolve_relative(contract.PIPELINE_CLAIM_PATH)
PIPELINE_LEDGER_PATH = resolve_relative(contract.PIPELINE_LEDGER_PATH)
ATTESTATION_RECEIPT_PATH = resolve_relative(contract.ATTESTATION_RECEIPT_PATH)
CANDIDATE_ROOT = resolve_relative(contract.CANDIDATE_ROOT)
CANDIDATE_MODULE_PATH = resolve_relative(contract.CANDIDATE_MODULE_PATH)
SYNTHESIS_SUMMARY_PATH = resolve_relative(contract.SYNTHESIS_SUMMARY_PATH)
SYNTHESIS_GATE_PATH = resolve_relative(contract.SYNTHESIS_GATE_PATH)
SYNTHESIS_RECEIPT_PATH = resolve_relative(contract.SYNTHESIS_RECEIPT_PATH)
CANDIDATE_FREEZE_PATH = resolve_relative(contract.CANDIDATE_FREEZE_PATH)
FINAL_DEVELOPMENT_RECEIPT_PATH = resolve_relative(
    contract.FINAL_DEVELOPMENT_RECEIPT_PATH
)

_ACTIVITY_FIELDS = (
    "environment_reset_calls",
    "environment_step_calls",
    "raw_sensor_sample_count",
    "teacher_query_count",
    "actor_synthesis_stage_calls_attempted",
    "actor_synthesis_executions_confirmed",
    "actor_fit_stage_calls_attempted",
    "actor_fit_executions_confirmed",
    "actor_updates",
    "adamw_epochs_completed",
    "lbfgs_closure_calls",
    "critic_updates",
    "ppo_updates",
)


def _zero_activity() -> dict[str, int]:
    return {name: 0 for name in _ACTIVITY_FIELDS}


def _increment(activity: dict[str, int], name: str, amount: int = 1) -> None:
    if name not in activity or type(amount) is not int or amount < 0:
        raise V12R6ExecutionError(f"invalid activity update: {name}={amount!r}")
    activity[name] += amount


def _mapping(path: Path) -> dict[str, Any]:
    try:
        value = forensic.strict_json_load(path)
    except Exception as exc:
        raise V12R6ExecutionError(f"invalid strict JSON mapping: {path}") from exc
    if not isinstance(value, dict):
        raise V12R6ExecutionError(f"expected JSON mapping: {path}")
    return value


def _sequence(path: Path) -> list[Any]:
    try:
        value = forensic.strict_json_load(path)
    except Exception as exc:
        raise V12R6ExecutionError(f"invalid strict JSON array: {path}") from exc
    if not isinstance(value, list):
        raise V12R6ExecutionError(f"expected JSON array: {path}")
    return value


def _record(path: Path | str | PurePosixPath) -> dict[str, Any]:
    target = Path(path)
    if not target.is_absolute():
        target = resolve_relative(os.fspath(path))
    target = target.resolve()
    try:
        target.relative_to(REPO_ROOT.resolve())
    except ValueError as exc:
        raise V12R6ExecutionError(
            f"artifact escaped repository root: {target}"
        ) from exc
    return forensic.artifact_record(target, artifact_root=REPO_ROOT)


def _tree(path: Path | str | PurePosixPath) -> dict[str, Any]:
    target = Path(path)
    if not target.is_absolute():
        target = resolve_relative(os.fspath(path))
    target = target.resolve()
    try:
        relative = target.relative_to(REPO_ROOT.resolve())
    except ValueError as exc:
        raise V12R6ExecutionError(
            f"artifact tree escaped repository root: {target}"
        ) from exc
    return freezer.tree_record(PurePosixPath(relative.as_posix()))


def _write(path: Path, payload: Mapping[str, Any] | Sequence[Any]) -> dict[str, Any]:
    forensic.write_json_exclusive(path, payload)
    return _record(path)


def _closed_snapshot() -> dict[str, list[str]]:
    groups = {
        "historical_q2": contract.HISTORICAL_Q2_CLOSED_PATHS,
        "historical_q3": contract.HISTORICAL_Q3_CLOSED_PATHS,
        "future_v12r6q3": contract.Q3_CLOSED_PATHS,
    }
    return {
        name: [
            path.as_posix()
            for path in paths.values()
            if os.path.lexists(resolve_relative(path))
        ]
        for name, paths in groups.items()
    }


def _assert_qualification_closed() -> dict[str, list[str]]:
    snapshot = _closed_snapshot()
    if any(snapshot.values()):
        raise V12R6ExecutionError(f"qualification path opened during V12R6: {snapshot}")
    return snapshot


def _artifact_exact(record: Mapping[str, Any]) -> bool:
    try:
        return _record(str(record["path"])) == dict(record)
    except (KeyError, TypeError, OSError, V12R6ExecutionError):
        return False


def _attest_terminal_inputs() -> dict[str, Any]:
    if not all(
        _artifact_exact(record)
        for record in contract.SOURCE_RECORDS.values()
        if "sha256" in record
    ):
        raise V12R6ExecutionError("one or more locked input artifacts drifted")
    if _tree(contract.P2_MODULE_PATH) != contract.P2_MODULE_TREE:
        raise V12R6ExecutionError("P2 module tree drifted")
    if _tree(contract.R5_MODULE_PATH) != contract.R5_MODULE_TREE:
        raise V12R6ExecutionError("R5 forensic module tree drifted")

    r5_ledger = _mapping(resolve_relative(contract.v12r5.PIPELINE_LEDGER_PATH))
    r5_gate = _mapping(resolve_relative(contract.v12r5.FIT_ROOT / "gate.json"))
    expected_activity = {
        "actor_fit_executions_confirmed": 0,
        "actor_fit_stage_calls_attempted": 1,
        "actor_updates_attempted": 1,
        "actor_updates_confirmed": 0,
        "adamw_epochs_completed": 3000,
        "environment_reset_calls": 0,
        "environment_step_calls": 0,
        "lbfgs_closure_calls": 613,
        "raw_sensor_sample_count": 0,
        "teacher_query_count": 0,
    }
    r5_exact = (
        r5_ledger.get("status") == contract.v12r5.TERMINAL_FAIL_STATUS
        and r5_ledger.get("passed") is False
        and r5_ledger.get("terminal") is True
        and r5_ledger.get("attempted_stage") == "fit_case_balanced_candidate"
        and r5_ledger.get("completed_stage_count") == 2
        and [item.get("stage_id") for item in r5_ledger.get("completed_stages", [])]
        == ["attest_locked_inputs", "assemble_case_balanced_corpus"]
        and r5_ledger.get("activity_totals") == expected_activity
        and r5_ledger.get("development_count") == 0
        and r5_ledger.get("q2_paths_opened") == []
        and r5_ledger.get("q3_paths_opened") == []
        and r5_ledger.get("qualification_executed") is False
        and r5_ledger.get("runtime_promoted") is False
        and r5_ledger.get("checkpoint_zero_created") is False
        and r5_ledger.get("positive_morphology_enabled") is False
        and r5_gate.get("status") == contract.v12r5.TERMINAL_FAIL_STATUS
        and r5_gate.get("passed") is False
        and r5_gate.get("candidate_promoted") is False
        and r5_gate.get("next_stage") == "STOP_TERMINAL"
        and r5_gate.get("checks", {}).get("global_metrics") is False
        and r5_gate.get("checks", {}).get("per_case_metrics") is False
    )
    if not r5_exact:
        raise V12R6ExecutionError("R5 terminal failure semantics drifted")
    forbidden_r5 = (
        contract.v12r5.CANDIDATE_FREEZE_PATH,
        contract.v12r5.FINAL_DEVELOPMENT_RECEIPT_PATH,
        contract.v12r5.DEVELOPMENT_ROOT,
    )
    if any(os.path.lexists(resolve_relative(path)) for path in forbidden_r5):
        raise V12R6ExecutionError("R5 post-fit output unexpectedly exists")
    closed = _assert_qualification_closed()
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.ATTESTATION_PASS_STATUS,
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "stage_id": "attest_terminal_inputs",
        "source_records": copy.deepcopy(contract.SOURCE_RECORDS),
        "r5_terminal_failure_semantics_exact": True,
        "r5_terminal_activity": expected_activity,
        "p2_module": copy.deepcopy(contract.P2_MODULE_TREE),
        "r5_forensic_module": copy.deepcopy(contract.R5_MODULE_TREE),
        "closed_path_snapshot": closed,
        "new_collection_count": 0,
        "environment_reset_calls": 0,
        "environment_step_calls": 0,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
    }


def _metric_triplet(
    predictions: Any,
    targets: Any,
    reset_mask: Any,
    selection: Any,
    *,
    np: Any,
) -> dict[str, float]:
    selected_reset = selection & reset_mask
    if not bool(np.any(selection)) or not bool(np.any(selected_reset)):
        raise V12R6ExecutionError("offline metric slice or reset slice is empty")
    error = predictions[selection].astype(np.float64) - targets[selection].astype(
        np.float64
    )
    reset_error = predictions[selected_reset].astype(np.float64) - targets[
        selected_reset
    ].astype(np.float64)
    return {
        "rmse": float(np.sqrt(np.mean(np.square(error), dtype=np.float64))),
        "max_abs_error": float(np.max(np.abs(error))),
        "reset_max_abs_error": float(np.max(np.abs(reset_error))),
    }


def _metric_pair(
    predictions: Any, targets: Any, selection: Any, *, np: Any
) -> dict[str, float]:
    if not bool(np.any(selection)):
        raise V12R6ExecutionError("critical-window metric slice is empty")
    error = predictions[selection].astype(np.float64) - targets[selection].astype(
        np.float64
    )
    return {
        "rmse": float(np.sqrt(np.mean(np.square(error), dtype=np.float64))),
        "max_abs_error": float(np.max(np.abs(error))),
    }


def _triplet_passes(value: Mapping[str, Any]) -> bool:
    thresholds = contract.OFFLINE_THRESHOLDS
    return (
        all(
            not isinstance(value.get(name), bool)
            and isinstance(value.get(name), (int, float))
            and math.isfinite(float(value[name]))
            for name in ("rmse", "max_abs_error", "reset_max_abs_error")
        )
        and 0.0 <= float(value["rmse"]) <= thresholds["rmse_max"]
        and 0.0 <= float(value["max_abs_error"]) <= thresholds["max_abs_error_max"]
        and 0.0
        <= float(value["reset_max_abs_error"])
        <= thresholds["reset_max_abs_error_max"]
    )


def _offline_evaluation(candidate_module_path: Path) -> dict[str, Any]:
    import numpy as np
    from ray.rllib.core.rl_module.rl_module import RLModule

    corpus_path = resolve_relative(contract.R5_CORPUS_ARTIFACT["path"])
    with np.load(corpus_path, allow_pickle=False) as corpus:
        observations = np.ascontiguousarray(corpus["observations"], dtype=np.float32)
        targets = np.ascontiguousarray(corpus["actions"], dtype=np.float32)
        resets = np.ascontiguousarray(corpus["reset_mask"], dtype=np.bool_)
        cases = corpus["case_ids"].astype(str)
        tranches = corpus["tranche_ids"].astype(str)
        steps = np.ascontiguousarray(corpus["step_indices"], dtype=np.int64)
        feature_names = tuple(str(name) for name in corpus["actor_feature_names"])
    if len(observations) != 9232 or observations.shape != (9232, 35):
        raise V12R6ExecutionError("locked offline corpus layout drifted")
    candidate = RLModule.from_checkpoint(candidate_module_path)
    candidate.eval()
    p2 = RLModule.from_checkpoint(resolve_relative(contract.P2_MODULE_PATH))
    p2.eval()
    predictions = composite._logits(candidate, observations)[:, :2]
    p2_predictions = composite._logits(p2, observations)[:, :2]

    trace = _sequence(resolve_relative(contract.R4_NOMINAL_TRACE_ARTIFACT["path"]))
    if len(trace) != 500:
        raise V12R6ExecutionError("R4 nominal exposure trace length drifted")
    exposed_tail = np.asarray(
        [
            row.get("effective_alpha") == 0.5
            and row.get("safety_latch_active") is False
            for row in trace
        ],
        dtype=np.bool_,
    )
    exposed = np.zeros(len(observations), dtype=np.bool_)
    exposed[8732:] = exposed_tail
    selections = {
        "global": np.ones(len(observations), dtype=np.bool_),
        "p2_subset": np.arange(len(observations)) < 8732,
        "nominal_r4_pass": np.arange(len(observations)) >= 8732,
        "nominal_r4_student_exposed": exposed,
    }
    selections.update(
        {f"case::{case_id}": cases == case_id for case_id in contract.CASE_IDS}
    )
    critical = (
        (cases == contract.CRITICAL_WINDOW["case_id"])
        & (tranches == contract.CRITICAL_WINDOW["tranche_id"])
        & (steps >= contract.CRITICAL_WINDOW["step_start_inclusive"])
        & (steps <= contract.CRITICAL_WINDOW["step_end_inclusive"])
    )
    metrics = {
        name: _metric_triplet(predictions, targets, resets, selection, np=np)
        for name, selection in selections.items()
    }
    critical_metrics = _metric_pair(predictions, targets, critical, np=np)
    p2_critical = _metric_pair(p2_predictions, targets, critical, np=np)
    failed = [name for name, value in metrics.items() if not _triplet_passes(value)]
    if any(
        critical_metrics[name] > p2_critical[name] for name in ("rmse", "max_abs_error")
    ):
        failed.append("critical_window_non_regression")
    return {
        "passed": not failed,
        "failed_checks": failed,
        "thresholds": copy.deepcopy(contract.OFFLINE_THRESHOLDS),
        "rows": len(observations),
        "actor_feature_names": list(feature_names),
        "metrics": metrics,
        "critical_window": copy.deepcopy(contract.CRITICAL_WINDOW),
        "critical_window_metrics": critical_metrics,
        "p2_critical_window_metrics": p2_critical,
    }


def _functional_verification(manifest: Mapping[str, Any]) -> dict[str, Any]:
    phases = [
        manifest.get("before_save"),
        manifest.get("after_reload"),
        manifest.get("final_reload"),
    ]
    if not all(isinstance(phase, Mapping) for phase in phases):
        raise V12R6ExecutionError("composite verification phases are malformed")
    errors: list[float] = []
    logstd_exact = True
    clock_zero = True
    digests: list[str] = []
    for phase in phases:
        assert isinstance(phase, Mapping)
        for name in ("corpus_direct_equivalence", "deterministic_direct_equivalence"):
            direct = phase.get(name)
            if not isinstance(direct, Mapping):
                raise V12R6ExecutionError("direct equivalence evidence is malformed")
            errors.append(float(direct["mean_max_abs_error"]))
            logstd_exact = (
                logstd_exact and direct.get("composite_logstd_byte_exact") is True
            )
        topology = phase.get("topology")
        clock = phase.get("clock_invariance")
        if not isinstance(topology, Mapping) or not isinstance(clock, Mapping):
            raise V12R6ExecutionError("topology/clock evidence is malformed")
        digests.append(str(topology.get("actor_digest")))
        checks = topology.get("checks")
        clock_zero = (
            clock_zero
            and isinstance(checks, Mapping)
            and checks.get("disabled_clock_columns_positive_zero") is True
            and clock.get("output_byte_exact_under_clock_perturbation") is True
        )
    feature_manifest = _mapping(
        CANDIDATE_MODULE_PATH / composite.ACTOR_FEATURE_MANIFEST
    )
    transplant = manifest.get("warm_start_and_checkpoint_zero_512")
    warm_start_ok = (
        isinstance(transplant, Mapping)
        and transplant.get("passed") is True
        and transplant.get("required_target_fcnet_hiddens") == [512, 512]
        and transplant.get("warm_start_transplant_actor_exact") is True
        and transplant.get("checkpoint_zero_standard_actor_transplant_exact") is True
        and transplant.get("fresh_critic_preserved_exact") is True
        and transplant.get("forward_surface_byte_exact") is True
    )
    feature_ok = (
        feature_manifest.get("status") == "H0_V12R6_COMPOSITE_ACTOR_FEATURE_CONTRACT"
        and feature_manifest.get("actor_feature_count") == 35
        and feature_manifest.get("fcnet_hiddens") == [512, 512]
        and feature_manifest.get("actor_digest") == manifest.get("actor_digest")
        and feature_manifest.get("module_state_sha256")
        == manifest.get("module_state_sha256")
    )
    return {
        "passed": all(
            (
                all(phase.get("passed") is True for phase in phases),
                max(errors) <= composite.MEAN_ABS_TOLERANCE,
                logstd_exact,
                clock_zero,
                len(set(digests)) == 1,
                feature_ok,
                warm_start_ok,
            )
        ),
        "max_abs_error_before_save": max(errors[:2]),
        "max_abs_error_after_reload": max(errors[2:]),
        "logstd_exact": logstd_exact,
        "disabled_clock_columns_bit_zero": clock_zero,
        "save_reload_exact": len(set(digests)) == 1,
        "actor_feature_manifest_valid": feature_ok,
        "warm_start_target_512_compatible": warm_start_ok,
        "actor_digest": digests[0],
    }


def _run_synthesis(activity: dict[str, int]) -> dict[str, Any]:
    if os.path.lexists(CANDIDATE_ROOT):
        raise V12R6ExecutionError("candidate root already exists")
    os.mkdir(CANDIDATE_ROOT, 0o700)
    _increment(activity, "actor_synthesis_stage_calls_attempted")
    manifest = composite.build_verify_save_composite(
        output_path=CANDIDATE_MODULE_PATH,
        p2_checkpoint=resolve_relative(contract.P2_MODULE_PATH),
        r5_checkpoint=resolve_relative(contract.R5_MODULE_PATH),
        corpus_path=resolve_relative(contract.R5_CORPUS_ARTIFACT["path"]),
    )
    _increment(activity, "actor_synthesis_executions_confirmed")
    module = _tree(CANDIDATE_MODULE_PATH)
    identity = contract.candidate_id(module["tree_sha256"])
    offline = _offline_evaluation(CANDIDATE_MODULE_PATH)
    verification = _functional_verification(manifest)
    summary = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.SYNTHESIS_COMPLETE_STATUS,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "stage_id": "synthesize_functional_composite_candidate",
        "synthesis": copy.deepcopy(contract.SYNTHESIS),
        "source_records": copy.deepcopy(contract.SOURCE_RECORDS),
        "candidate_selection_rule": contract.CANDIDATE_SELECTION_RULE,
        "candidate_id": identity,
        "candidate_module": module,
        "builder_manifest": _record(CANDIDATE_MODULE_PATH / composite.BUILD_MANIFEST),
        "functional_verification": verification,
        "offline_evaluation": offline,
        "event_contract_id": contract.EVENT_CONTRACT_ID,
        "target_contract_id": contract.TARGET_CONTRACT_ID,
        "detector_or_fsm_modified": False,
        "q2_paths_opened": [],
        "q3_paths_opened": [],
        "new_collection_count": 0,
        "environment_reset_calls": 0,
        "environment_step_calls": 0,
        "actor_synthesis_stage_calls_attempted": activity[
            "actor_synthesis_stage_calls_attempted"
        ],
        "actor_synthesis_executions_confirmed": activity[
            "actor_synthesis_executions_confirmed"
        ],
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "adamw_epochs_completed": 0,
        "lbfgs_closure_calls": 0,
    }
    _write(SYNTHESIS_SUMMARY_PATH, summary)
    gate = contract.synthesis_gate(summary)
    _write(SYNTHESIS_GATE_PATH, gate)
    if gate.get("passed") is not True:
        raise V12R6ExecutionError("V12R6 synthesis gate failed")
    receipt = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.SYNTHESIS_PASS_STATUS,
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "stage_id": "synthesize_functional_composite_candidate",
        "candidate_id": identity,
        "candidate_module": module,
        "summary": _record(SYNTHESIS_SUMMARY_PATH),
        "gate": _record(SYNTHESIS_GATE_PATH),
        "builder_manifest": summary["builder_manifest"],
        "functional_verification": verification,
        "offline_evaluation": offline,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
    }
    _write(SYNTHESIS_RECEIPT_PATH, receipt)
    return receipt


def _run_candidate_freeze() -> dict[str, Any]:
    synthesis = _mapping(SYNTHESIS_RECEIPT_PATH)
    module = _tree(CANDIDATE_MODULE_PATH)
    identity = contract.candidate_id(module["tree_sha256"])
    if (
        synthesis.get("status") != contract.SYNTHESIS_PASS_STATUS
        or synthesis.get("passed") is not True
        or synthesis.get("candidate_id") != identity
        or synthesis.get("candidate_module") != module
    ):
        raise V12R6ExecutionError("synthesis identity drifted before candidate freeze")
    verification = synthesis.get("functional_verification")
    if not isinstance(verification, Mapping):
        raise V12R6ExecutionError("synthesis verification is malformed")
    summary = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.CANDIDATE_FREEZE_COMPLETE_STATUS,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "candidate_selection_rule": contract.CANDIDATE_SELECTION_RULE,
        "candidate_id": identity,
        "candidate_module": module,
        "synthesis_passed": True,
        "synthesis_receipt": _record(SYNTHESIS_RECEIPT_PATH),
        "candidate_frozen": True,
        "functional_equivalence_passed": verification.get("passed") is True,
        "logstd_exact": verification.get("logstd_exact") is True,
        "save_reload_exact": verification.get("save_reload_exact") is True,
        "warm_start_target_512_compatible": verification.get(
            "warm_start_target_512_compatible"
        )
        is True,
        "q2_paths_opened": [],
        "q3_paths_opened": [],
        "runtime_promoted": False,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
    }
    summary_path = RUN_ROOT / "candidate_freeze_summary.json"
    gate_path = RUN_ROOT / "candidate_freeze_gate.json"
    _write(summary_path, summary)
    gate = contract.candidate_freeze_gate(summary)
    _write(gate_path, gate)
    if gate.get("passed") is not True:
        raise V12R6ExecutionError("V12R6 candidate freeze gate failed")
    receipt = {
        **summary,
        "status": contract.CANDIDATE_FREEZE_PASS_STATUS,
        "passed": True,
        "stage_id": "freeze_functional_composite_candidate",
        "summary": _record(summary_path),
        "gate": _record(gate_path),
    }
    _write(CANDIDATE_FREEZE_PATH, receipt)
    return receipt


def _physical_config() -> physical.PhysicalDevelopmentConfig:
    return physical.PhysicalDevelopmentConfig(
        protocol_id=contract.PROTOCOL_ID,
        start_status="STARTED_H0_V12R6_FUNCTIONAL_COMPOSITE_DEVELOPMENT",
        partial_status="PERSISTED_H0_V12R6_DEVELOPMENT_BEFORE_GATE",
        complete_status=contract.DEVELOPMENT_COMPLETE_STATUS,
        schema_version=contract.SCHEMA_VERSION,
        artifact_root=REPO_ROOT,
        progress_label="V12R6 development",
        progress_every=25,
    )


def _run_development(case_id: str, activity: dict[str, int]) -> dict[str, Any]:
    _assert_qualification_closed()
    case = contract.canonical_development_case(case_id)
    destination = resolve_relative(contract.DEVELOPMENT_PATHS[case_id])
    freeze = _mapping(CANDIDATE_FREEZE_PATH)
    module = _tree(CANDIDATE_MODULE_PATH)
    identity = contract.candidate_id(module["tree_sha256"])
    if (
        freeze.get("status") != contract.CANDIDATE_FREEZE_PASS_STATUS
        or freeze.get("passed") is not True
        or freeze.get("candidate_id") != identity
        or freeze.get("candidate_module") != module
    ):
        raise V12R6ExecutionError("candidate freeze identity drifted")

    result = physical.run_case(
        config=_physical_config(),
        case=case,
        destination=destination,
        module_path=CANDIDATE_MODULE_PATH,
        activity_callback=lambda name, amount: _increment(activity, name, amount),
        start_metadata={
            "pipeline_claim": _record(PIPELINE_CLAIM_PATH),
            "execution_lock": _record(resolve_relative(contract.EXECUTION_LOCK_PATH)),
            "candidate_id": identity,
            "candidate_module": module,
            "candidate_freeze": _record(CANDIDATE_FREEZE_PATH),
        },
        summary_metadata={
            "candidate_id": identity,
            "candidate_module": module,
            "candidate_freeze": _record(CANDIDATE_FREEZE_PATH),
            "target_contract_id": contract.TARGET_CONTRACT_ID,
            "event_contract_id": contract.EVENT_CONTRACT_ID,
        },
    )
    _assert_qualification_closed()
    trace = result["trace"]
    summary = result["summary"]
    contract_audit = contract.pure_policy_trace_audit(trace, case_id=case_id)
    if summary.get("pure_policy_trace_audit") != contract_audit:
        raise V12R6ExecutionError("physical and contract trace audits disagree")
    gate = contract.development_gate(summary, case_id=case_id, trace=trace)
    gate_path = destination / "gate.json"
    _write(gate_path, gate)
    if gate.get("passed") is not True:
        raise V12R6ExecutionError(f"V12R6 development gate failed: {case_id}")
    receipt = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.DEVELOPMENT_PASS_STATUS,
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "stage_id": f"development__{case_id}",
        "case_id": case_id,
        "candidate_id": identity,
        "candidate_module": module,
        "candidate_freeze": _record(CANDIDATE_FREEZE_PATH),
        "summary": _record(destination / "summary.json"),
        "gate": _record(gate_path),
        "trace": _record(destination / "trace.json"),
        "pure_policy_trace_audit": contract_audit,
        "pure_policy_trace_row_count": len(trace),
        **{name: 0 for name in contract.PURE_POLICY_COUNTER_FIELDS},
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
    }
    _write(destination / "receipt.json", receipt)
    return receipt


def _run_finalize_development(activity: Mapping[str, int]) -> dict[str, Any]:
    module = _tree(CANDIDATE_MODULE_PATH)
    identity = contract.candidate_id(module["tree_sha256"])
    bindings: list[dict[str, Any]] = []
    for case_id in contract.DEVELOPMENT_CASE_IDS:
        root = resolve_relative(contract.DEVELOPMENT_PATHS[case_id])
        receipt = _mapping(root / "receipt.json")
        summary = _mapping(root / "summary.json")
        gate = _mapping(root / "gate.json")
        trace = _sequence(root / "trace.json")
        expected_gate = contract.development_gate(summary, case_id=case_id, trace=trace)
        if (
            receipt.get("status") != contract.DEVELOPMENT_PASS_STATUS
            or receipt.get("passed") is not True
            or receipt.get("candidate_id") != identity
            or receipt.get("candidate_module") != module
            or receipt.get("summary") != _record(root / "summary.json")
            or receipt.get("gate") != _record(root / "gate.json")
            or receipt.get("trace") != _record(root / "trace.json")
            or gate != expected_gate
            or gate.get("passed") is not True
        ):
            raise V12R6ExecutionError(f"development binding drifted: {case_id}")
        bindings.append(
            {
                "case_id": case_id,
                "passed": True,
                "receipt": _record(root / "receipt.json"),
                "gate": _record(root / "gate.json"),
                "summary": _record(root / "summary.json"),
                "trace": _record(root / "trace.json"),
            }
        )
    summary = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": "COMPLETE_H0_V12R6_FUNCTIONAL_COMPOSITE_DEVELOPMENT_AGGREGATE",
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "candidate_selection_rule": contract.CANDIDATE_SELECTION_RULE,
        "candidate_id": identity,
        "candidate_module": module,
        "candidate_freeze": _record(CANDIDATE_FREEZE_PATH),
        "case_gates": [
            {"case_id": item["case_id"], "passed": True} for item in bindings
        ],
        "rollout_bindings": bindings,
        "candidate_tree_unique_count": 1,
        "new_collection_count": 0,
        "development_count": 6,
        "environment_reset_calls": activity["environment_reset_calls"],
        "environment_step_calls": activity["environment_step_calls"],
        "raw_sensor_sample_count": activity["raw_sensor_sample_count"],
        "teacher_query_count": activity["teacher_query_count"],
        "pure_policy_trace_row_count": 3000,
        **{name: 0 for name in contract.PURE_POLICY_COUNTER_FIELDS},
        "actor_synthesis_stage_calls_attempted": activity[
            "actor_synthesis_stage_calls_attempted"
        ],
        "actor_synthesis_executions_confirmed": activity[
            "actor_synthesis_executions_confirmed"
        ],
        "actor_fit_stage_calls_attempted": activity["actor_fit_stage_calls_attempted"],
        "actor_fit_executions_confirmed": activity["actor_fit_executions_confirmed"],
        "actor_updates": activity["actor_updates"],
        "adamw_epochs_completed": activity["adamw_epochs_completed"],
        "lbfgs_closure_calls": activity["lbfgs_closure_calls"],
        "critic_updates": activity["critic_updates"],
        "ppo_updates": activity["ppo_updates"],
        "retry_authorized": False,
        "resume_authorized": False,
        "rescue_authorized": False,
        "q2_paths_opened": [],
        "q3_paths_opened": [],
        "runtime_promoted": False,
        "checkpoint_zero_created": False,
        "positive_morphology_enabled": False,
    }
    summary_path = RUN_ROOT / "final_development_summary.json"
    gate_path = RUN_ROOT / "final_development_gate.json"
    _write(summary_path, summary)
    gate = contract.aggregate_development_gate(summary)
    _write(gate_path, gate)
    if gate.get("passed") is not True:
        raise V12R6ExecutionError("V12R6 aggregate development gate failed")
    receipt = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.FINAL_DEVELOPMENT_PASS_STATUS,
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "stage_id": "finalize_development",
        "candidate_id": identity,
        "candidate_module": module,
        "candidate_freeze": _record(CANDIDATE_FREEZE_PATH),
        "summary": _record(summary_path),
        "gate": _record(gate_path),
        "rollout_bindings": bindings,
        "qualification_executed": False,
        "runtime_promoted": False,
        "checkpoint_zero_created": False,
        "positive_morphology_enabled": False,
        "retry_authorized": False,
        "resume_authorized": False,
        "rescue_authorized": False,
        "sweep_authorized": False,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
    }
    _write(FINAL_DEVELOPMENT_RECEIPT_PATH, receipt)
    return receipt


def _terminal_payload(
    *,
    passed: bool,
    completed: Sequence[Mapping[str, Any]],
    attempted_stage: str | None,
    activity: Mapping[str, int],
    error: BaseException | None,
) -> dict[str, Any]:
    candidate_module = None
    candidate_identity = None
    candidate_observation: dict[str, Any] = {"state": "ABSENT", "tree": None}
    if CANDIDATE_MODULE_PATH.is_dir() and not CANDIDATE_MODULE_PATH.is_symlink():
        try:
            candidate_module = _tree(CANDIDATE_MODULE_PATH)
            candidate_identity = contract.candidate_id(candidate_module["tree_sha256"])
            candidate_observation = {
                "state": "VALID_NONEMPTY_TREE",
                "tree": candidate_module,
            }
        except BaseException as exc:
            candidate_module = None
            candidate_identity = None
            candidate_observation = {
                "state": "PARTIAL_OR_UNSAFE_TREE",
                "tree": None,
                "error": {
                    "type": type(exc).__name__,
                    "message": str(exc) or repr(exc),
                },
            }
    elif os.path.lexists(CANDIDATE_MODULE_PATH):
        candidate_observation = {"state": "UNSAFE_NONDIRECTORY", "tree": None}
    snapshot = _closed_snapshot()
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.PIPELINE_PASS_STATUS
        if passed
        else contract.TERMINAL_FAIL_STATUS,
        "passed": passed,
        "terminal": True,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "candidate_selection_rule": contract.CANDIDATE_SELECTION_RULE,
        "candidate_id": candidate_identity,
        "candidate_module": candidate_module,
        "candidate_module_observation": candidate_observation,
        "stage_order": list(contract.STAGE_IDS),
        "completed_stages": list(completed),
        "completed_stage_count": len(completed),
        "attempted_stage": attempted_stage,
        "activity_totals": dict(activity),
        "development_count": sum(
            item.get("stage_id", "").startswith("development__") for item in completed
        ),
        "protocol_freeze": _record(resolve_relative(contract.PROTOCOL_FREEZE_PATH)),
        "execution_lock": _record(resolve_relative(contract.EXECUTION_LOCK_PATH)),
        "pipeline_claim": _record(PIPELINE_CLAIM_PATH),
        "candidate_freeze": _record(CANDIDATE_FREEZE_PATH)
        if CANDIDATE_FREEZE_PATH.is_file()
        else None,
        "final_development_receipt": _record(FINAL_DEVELOPMENT_RECEIPT_PATH)
        if FINAL_DEVELOPMENT_RECEIPT_PATH.is_file()
        else None,
        "qualification_snapshot": snapshot,
        "qualification_violation_detected": any(snapshot.values()),
        "qualification_executed": False,
        "new_collection_count": 0,
        "actor_updates": activity["actor_updates"],
        "critic_updates": activity["critic_updates"],
        "ppo_updates": activity["ppo_updates"],
        "retry_authorized": False,
        "resume_authorized": False,
        "rescue_authorized": False,
        "sweep_authorized": False,
        "runtime_promoted": False,
        "checkpoint_zero_created": False,
        "positive_morphology_enabled": False,
        "error": None
        if error is None
        else {
            "type": type(error).__name__,
            "message": str(error) or repr(error),
        },
        "next_stage": "WAIT_SEPARATE_V12R6Q3_PROTOCOL"
        if passed
        else "STOP_TERMINAL_NO_RETRY",
    }


def verify_terminal_ledger() -> dict[str, Any]:
    ledger = _mapping(PIPELINE_LEDGER_PATH)
    passed_field = ledger.get("passed")
    if type(passed_field) is not bool:
        raise V12R6ExecutionError("terminal ledger identity drifted")
    passed = passed_field
    status = contract.PIPELINE_PASS_STATUS if passed else contract.TERMINAL_FAIL_STATUS
    if (
        ledger.get("status") != status
        or ledger.get("terminal") is not True
        or ledger.get("protocol_id") != contract.PROTOCOL_ID
        or ledger.get("pipeline_id") != contract.PIPELINE_ID
        or ledger.get("pipeline_claim") != _record(PIPELINE_CLAIM_PATH)
        or ledger.get("execution_lock")
        != _record(resolve_relative(contract.EXECUTION_LOCK_PATH))
        or ledger.get("retry_authorized") is not False
        or ledger.get("resume_authorized") is not False
        or ledger.get("rescue_authorized") is not False
        or ledger.get("sweep_authorized") is not False
    ):
        raise V12R6ExecutionError("terminal ledger identity drifted")
    completed = ledger.get("completed_stages")
    completed_count = ledger.get("completed_stage_count")
    if (
        not isinstance(completed, list)
        or type(completed_count) is not int
        or completed_count != len(completed)
    ):
        raise V12R6ExecutionError("terminal completed-stage prefix drifted")
    for index, item in enumerate(completed):
        if (
            not isinstance(item, Mapping)
            or item.get("stage_id") != contract.STAGE_IDS[index]
            or not _artifact_exact(item.get("receipt", {}))
        ):
            raise V12R6ExecutionError("terminal stage receipt binding drifted")
    if passed:
        if (
            len(completed) != len(contract.STAGE_IDS)
            or ledger.get("attempted_stage") is not None
            or ledger.get("error") is not None
            or ledger.get("qualification_violation_detected") is not False
            or ledger.get("candidate_freeze") != _record(CANDIDATE_FREEZE_PATH)
            or ledger.get("final_development_receipt")
            != _record(FINAL_DEVELOPMENT_RECEIPT_PATH)
        ):
            raise V12R6ExecutionError("terminal PASS closure drifted")
    else:
        if not isinstance(ledger.get("error"), Mapping):
            raise V12R6ExecutionError("terminal FAIL error evidence is missing")
    return ledger


def execute() -> dict[str, Any]:
    if os.path.lexists(PIPELINE_LEDGER_PATH):
        return verify_terminal_ledger()
    freezer.verify_protocol_freeze()
    freezer.verify_execution_lock(
        require_pristine=True, require_qualification_closed=True
    )
    if os.path.lexists(RUN_ROOT):
        raise V12R6ExecutionError("V12R6 run root occupied; resume is forbidden")
    os.mkdir(RUN_ROOT, 0o700)
    claim = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": "CLAIMED_H0_V12R6_FUNCTIONAL_COMPOSITE_PIPELINE_ONCE",
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "authority": copy.deepcopy(contract.AUTHORITY),
        "stage_order": list(contract.STAGE_IDS),
        "protocol_freeze": _record(resolve_relative(contract.PROTOCOL_FREEZE_PATH)),
        "execution_lock": _record(resolve_relative(contract.EXECUTION_LOCK_PATH)),
        "one_shot": True,
        "retry_authorized": False,
        "resume_authorized": False,
        "rescue_authorized": False,
        "sweep_authorized": False,
        "qualification_execution_authorized": False,
    }
    _write(PIPELINE_CLAIM_PATH, claim)
    activity = _zero_activity()
    completed: list[dict[str, Any]] = []
    attempted_stage: str | None = None
    error: BaseException | None = None
    try:
        for stage_id in contract.STAGE_IDS:
            attempted_stage = stage_id
            _assert_qualification_closed()
            descriptor = contract.stage_descriptor(stage_id)
            if descriptor["kind"] == "attestation":
                receipt = _attest_terminal_inputs()
                _write(ATTESTATION_RECEIPT_PATH, receipt)
                receipt_path = ATTESTATION_RECEIPT_PATH
            elif descriptor["kind"] == "synthesis":
                _run_synthesis(activity)
                receipt_path = SYNTHESIS_RECEIPT_PATH
            elif descriptor["kind"] == "candidate_freeze":
                _run_candidate_freeze()
                receipt_path = CANDIDATE_FREEZE_PATH
            elif descriptor["kind"] == "development":
                case_id = str(descriptor["case"]["case_id"])
                _run_development(case_id, activity)
                receipt_path = (
                    resolve_relative(contract.DEVELOPMENT_PATHS[case_id])
                    / "receipt.json"
                )
            elif descriptor["kind"] == "finalize":
                _run_finalize_development(activity)
                receipt_path = FINAL_DEVELOPMENT_RECEIPT_PATH
            else:  # pragma: no cover - contract self-check owns this branch.
                raise V12R6ExecutionError(f"unknown stage kind: {descriptor['kind']}")
            completed.append({"stage_id": stage_id, "receipt": _record(receipt_path)})
            attempted_stage = None
        _assert_qualification_closed()
    except BaseException as exc:
        error = exc

    passed = error is None and len(completed) == len(contract.STAGE_IDS)
    ledger = _terminal_payload(
        passed=passed,
        completed=completed,
        attempted_stage=attempted_stage,
        activity=activity,
        error=error,
    )
    _write(PIPELINE_LEDGER_PATH, ledger)
    verified = verify_terminal_ledger()
    if not passed:
        assert error is not None
        raise V12R6ExecutionError(
            f"V12R6 stopped terminally at {attempted_stage}: {error}"
        ) from error
    return verified


def preflight() -> dict[str, Any]:
    freezer.verify_protocol_freeze()
    freezer.verify_execution_lock(
        require_pristine=True, require_qualification_closed=True
    )
    attestation = _attest_terminal_inputs()
    return {
        "passed": True,
        "status": "READY_H0_V12R6_FUNCTIONAL_COMPOSITE_ONE_SHOT",
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "attestation": attestation,
        "run_root_absent": not os.path.lexists(RUN_ROOT),
        "qualification_closed": True,
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
