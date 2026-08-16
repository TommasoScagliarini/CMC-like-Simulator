#!/usr/bin/env python3
"""Prepare or execute the V12R13 P1 tape-reference collection exactly once.

``--describe`` is the default and has no filesystem side effect.  ``--preflight``
reads and hashes the locked tape, P0 candidate, and runtime closure but does not
load a checkpoint or construct an environment.  Only explicit ``--execute``
loads P0 and runs the complete fixed 18-trajectory plan.  There is no single-
case execution surface, update hook, retry, resume, sweep, or teacher model.
"""

from __future__ import annotations

import argparse
import copy
import json
import math
import os
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Mapping, Sequence

import numpy as np


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
BASELINE_ROOT = REPO_ROOT / "Trajectory Generator" / "baseline_MLP"
LOCAL_VALIDATION = BASELINE_ROOT / "validation"
REVISION_ROOT = Path(__file__).resolve().parent
for _root in (
    REPO_ROOT,
    REPO_ROOT / "validation",
    BASELINE_ROOT,
    LOCAL_VALIDATION,
    LOCAL_VALIDATION / "v12r3",
    LOCAL_VALIDATION / "v12r6",
    REVISION_ROOT,
):
    if str(_root) not in sys.path:
        sys.path.insert(0, str(_root))

import h0_forensic_rollout as forensic  # noqa: E402
import h0_v12r13_tape_dagger as contract  # noqa: E402


class V12R13TapeDaggerExecutionError(RuntimeError):
    """Raised on any P1 provenance, ordering, rollout, or closure failure."""


@dataclass(frozen=True)
class PreparedP1:
    preflight: Mapping[str, Any]
    tape: contract.LockedTapeCorpus
    support: contract.SupportEnvelope
    source_snapshot: Mapping[str, Any]
    source_candidate_id: str
    source_candidate_tree: Mapping[str, Any]


@dataclass(frozen=True)
class ExecutionContext:
    module: Any
    rollout_eval: Any
    torch: Any
    env_factory: Any
    legacy: Any
    v26_collector: Any
    runtime: Any
    physical: Any
    env_config_builder: Callable[[Mapping[str, Any]], Mapping[str, Any]]


def _mapping(path: Path) -> dict[str, Any]:
    value = forensic.strict_json_load(path)
    if not isinstance(value, Mapping):
        raise V12R13TapeDaggerExecutionError(f"expected JSON object: {path}")
    return dict(value)


def _record(path: Path) -> dict[str, Any]:
    return forensic.artifact_record(path, artifact_root=REPO_ROOT)


def _record_matches(value: Any, path: Path) -> bool:
    return isinstance(value, Mapping) and dict(value) == _record(path)


def _tree_record(path: Path) -> dict[str, Any]:
    # The contract implementation is the single canonical tree algorithm.
    return contract._tree_record(path)  # noqa: SLF001


def _strict_equal(left: Any, right: Any) -> bool:
    return contract._strict_equal(left, right)  # noqa: SLF001


def _write_npz_exclusive(path: Path, arrays: Mapping[str, Any]) -> Path:
    destination = path.resolve()
    try:
        destination.relative_to(REPO_ROOT)
    except ValueError as exc:
        raise V12R13TapeDaggerExecutionError("NPZ path escaped repository") from exc
    destination.parent.mkdir(parents=True, exist_ok=True)
    flags = os.O_CREAT | os.O_EXCL | os.O_WRONLY | getattr(os, "O_BINARY", 0)
    try:
        descriptor = os.open(destination, flags, 0o600)
    except FileExistsError as exc:
        raise V12R13TapeDaggerExecutionError(
            f"artifact exists/no-clobber: {destination}"
        ) from exc
    try:
        with os.fdopen(descriptor, "wb") as stream:
            descriptor = -1
            np.savez_compressed(stream, **arrays)
            stream.flush()
            os.fsync(stream.fileno())
    finally:
        if descriptor >= 0:
            os.close(descriptor)
    return destination


def verify_p0_candidate(
    *,
    candidate_module: Path = contract.P0_MODULE,
    receipt_path: Path = contract.P0_RECEIPT,
    summary_path: Path = contract.P0_SUMMARY,
    gate_path: Path = contract.P0_GATE,
    enforce_canonical: bool = True,
) -> dict[str, Any]:
    """Close the collector-only P0 fit without loading its RLModule."""

    paths = tuple(
        path.resolve()
        for path in (candidate_module, receipt_path, summary_path, gate_path)
    )
    if enforce_canonical:
        expected = tuple(
            path.resolve()
            for path in (
                contract.P0_MODULE,
                contract.P0_RECEIPT,
                contract.P0_SUMMARY,
                contract.P0_GATE,
            )
        )
        if paths != expected:
            raise V12R13TapeDaggerExecutionError("non-canonical P0 input path")
    candidate, receipt_file, summary_file, gate_file = paths
    receipt = _mapping(receipt_file)
    summary = _mapping(summary_file)
    gate = _mapping(gate_file)
    artifacts = receipt.get("artifacts")
    actual_tree = _tree_record(candidate)
    checks = {
        "receipt_pass": receipt.get("status")
        == "PASS_H0_V12R13_MASKED_SAFE_TEACHER_FIT_RECEIPT"
        and receipt.get("passed") is True,
        "gate_pass": gate.get("status") == "PASS_H0_V12R13_MASKED_SAFE_TEACHER_FIT"
        and gate.get("passed") is True,
        "protocol": receipt.get("protocol_id") == contract.PROTOCOL_ID
        and summary.get("protocol_id") == contract.PROTOCOL_ID
        and gate.get("protocol_id") == contract.PROTOCOL_ID,
        "artifact_schema": isinstance(artifacts, Mapping)
        and set(artifacts) == {"corpus", "candidate_module", "summary", "gate"},
        "candidate_tree": isinstance(artifacts, Mapping)
        and artifacts.get("candidate_module") == actual_tree
        and summary.get("candidate_module") == actual_tree,
        "summary_binding": isinstance(artifacts, Mapping)
        and _record_matches(artifacts.get("summary"), summary_file),
        "gate_binding": isinstance(artifacts, Mapping)
        and _record_matches(artifacts.get("gate"), gate_file),
        "candidate_id": isinstance(receipt.get("candidate_id"), str)
        and receipt.get("candidate_id") == summary.get("candidate_id"),
        "collector_only": receipt.get("candidate_promoted") is False
        and summary.get("candidate_promoted") is False,
        "single_p0_fit": receipt.get("actor_fit_count") == 1
        and receipt.get("actor_updates") == 1,
        "no_runtime_updates": receipt.get("critic_updates") == 0
        and receipt.get("ppo_updates") == 0
        and receipt.get("environment_steps") == 0
        and receipt.get("policy_rollouts") == 0,
        "no_retry": receipt.get("retry_authorized") is False,
    }
    if not all(checks.values()):
        failed = sorted(name for name, value in checks.items() if not value)
        raise V12R13TapeDaggerExecutionError(f"P0 candidate closure failed: {failed}")
    return {
        "candidate_id": str(receipt["candidate_id"]),
        "candidate_tree": actual_tree,
        "receipt": _record(receipt_file),
        "summary": _record(summary_file),
        "gate": _record(gate_file),
        "checks": checks,
    }


def build_protocol_freeze() -> dict[str, Any]:
    """Build the P0-independent, content-addressed one-shot protocol freeze."""

    source_paths = {
        "contract": REVISION_ROOT / "h0_v12r13_tape_dagger.py",
        "runner": Path(__file__).resolve(),
        "tests": REVISION_ROOT / "test_h0_v12r13_tape_dagger.py",
    }
    sources = {name: _record(path) for name, path in source_paths.items()}
    runtime_closure = contract.verify_runtime_source_closure()
    tape_closure = contract.verify_tape_source_closure()
    description = contract.build_protocol_description()
    checks = {
        "three_sources": set(sources) == {"contract", "runner", "tests"},
        "runtime_closure_94": runtime_closure.get("file_count") == 94,
        "tape_steps_3000": tape_closure.get("transitive_step_files_verified")
        == contract.EXPECTED_BASE_ROWS,
        "plan_18": len(description["plan"]) == contract.EXPECTED_ROLLOUTS,
        "rows_9000": description["p1_rows"] == contract.EXPECTED_P1_ROWS,
        "support_constants": description["support"]["p99"]
        == contract.SUPPORT_P99_THRESHOLD
        and description["support"]["loo_max"] == contract.SUPPORT_LOO_MAX,
        "p0_unbound": True,
        "one_shot": True,
    }
    if not all(checks.values()):
        failed = sorted(name for name, value in checks.items() if not value)
        raise V12R13TapeDaggerExecutionError(
            f"protocol freeze construction failed: {failed}"
        )
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.PROTOCOL_FREEZE_STATUS,
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "stage_id": contract.STAGE_ID,
        "revision": contract.REVISION,
        "checks": checks,
        "sources": sources,
        "runtime_transitive_closure": runtime_closure,
        "tape_source_closure": tape_closure,
        "protocol": description,
        "support_scalar_contract": {
            "p99": contract.SUPPORT_P99_THRESHOLD,
            "p99_hex": contract.SUPPORT_P99_HEX,
            "loo_max": contract.SUPPORT_LOO_MAX,
            "loo_max_hex": contract.SUPPORT_LOO_MAX_HEX,
            "near_tie_candidate_atol": contract.NEAREST_CANDIDATE_ATOL,
            "decision": "fallback_iff_portable_distance_gt_serialized_p99",
        },
        "p0_binding": "DEFERRED_TO_EXECUTION_LOCK_AFTER_PASS_P0",
        "retry_authorized": False,
        "resume_authorized": False,
        "sweep_authorized": False,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
    }


def write_protocol_freeze() -> dict[str, Any]:
    if os.path.lexists(contract.P1_PROTOCOL_FREEZE):
        raise V12R13TapeDaggerExecutionError("protocol freeze exists/no-clobber")
    if os.path.lexists(contract.P1_EXECUTION_LOCK) or os.path.lexists(contract.P1_ROOT):
        raise V12R13TapeDaggerExecutionError(
            "cannot freeze after lock or P1 output exists"
        )
    payload = build_protocol_freeze()
    path = forensic.write_json_exclusive(contract.P1_PROTOCOL_FREEZE, payload)
    return {**payload, "protocol_freeze": _record(path)}


def verify_protocol_freeze() -> dict[str, Any]:
    observed = _mapping(contract.P1_PROTOCOL_FREEZE)
    expected = build_protocol_freeze()
    if forensic.canonical_json_bytes(observed) != forensic.canonical_json_bytes(
        expected
    ):
        raise V12R13TapeDaggerExecutionError(
            "protocol freeze is stale or content drifted"
        )
    return observed


def _execution_lock_payload(prepared: PreparedP1) -> dict[str, Any]:
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.EXECUTION_LOCK_STATUS,
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "stage_id": contract.STAGE_ID,
        "protocol_freeze": _record(contract.P1_PROTOCOL_FREEZE),
        "source_candidate_id": prepared.source_candidate_id,
        "source_p0_candidate": copy.deepcopy(prepared.source_candidate_tree),
        "source_snapshot": copy.deepcopy(dict(prepared.source_snapshot)),
        "tape_array_hashes": dict(prepared.tape.array_hashes),
        "support": contract.support_manifest(prepared.support),
        "plan": [dict(row) for row in contract.collection_plan()],
        "p1_destination": contract.P1_ROOT.relative_to(REPO_ROOT).as_posix(),
        # Historical attestation only: write_execution_lock() proves the
        # destination is unoccupied before publishing. The lock payload must
        # never depend on the CURRENT namespace occupancy (V12R10 lesson),
        # or the lock becomes unverifiable as soon as P1 runs.
        "p1_destination_unoccupied_at_lock_time": True,
        "one_shot": True,
        "retry_authorized": False,
        "resume_authorized": False,
        "sweep_authorized": False,
        "teacher_model_query_authorized": False,
        "candidate_update_between_rollouts_authorized": False,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
    }


def write_execution_lock() -> dict[str, Any]:
    verify_protocol_freeze()
    if os.path.lexists(contract.P1_EXECUTION_LOCK):
        raise V12R13TapeDaggerExecutionError("execution lock exists/no-clobber")
    if os.path.lexists(contract.P1_ROOT):
        raise V12R13TapeDaggerExecutionError("P1 output exists before lock")
    prepared = prepare_p1(
        require_unoccupied=True,
        enforce_canonical=True,
        require_execution_lock=False,
    )
    payload = _execution_lock_payload(prepared)
    path = forensic.write_json_exclusive(contract.P1_EXECUTION_LOCK, payload)
    return {**payload, "execution_lock": _record(path)}


def verify_execution_lock(prepared: PreparedP1) -> dict[str, Any]:
    verify_protocol_freeze()
    observed = _mapping(contract.P1_EXECUTION_LOCK)
    expected = _execution_lock_payload(prepared)
    if forensic.canonical_json_bytes(observed) != forensic.canonical_json_bytes(
        expected
    ):
        raise V12R13TapeDaggerExecutionError(
            "execution lock is stale or P0/source binding drifted"
        )
    return observed


def prepare_p1(
    *,
    candidate_module: Path = contract.P0_MODULE,
    receipt_path: Path = contract.P0_RECEIPT,
    summary_path: Path = contract.P0_SUMMARY,
    gate_path: Path = contract.P0_GATE,
    require_unoccupied: bool = True,
    enforce_canonical: bool = True,
    require_execution_lock: bool = True,
    tape_loader: Callable[[], contract.LockedTapeCorpus] = (
        contract.load_locked_tape_corpus
    ),
) -> PreparedP1:
    """Read-only preflight; no checkpoint, policy, environment, or write."""

    p0 = verify_p0_candidate(
        candidate_module=candidate_module,
        receipt_path=receipt_path,
        summary_path=summary_path,
        gate_path=gate_path,
        enforce_canonical=enforce_canonical,
    )
    tape = tape_loader()
    support = contract.build_support_envelope(tape)
    snapshot = contract.source_snapshot(candidate_module=candidate_module)
    occupied = os.path.lexists(contract.P1_ROOT)
    checks = {
        "p0_pass": all(p0["checks"].values()),
        "tape_rows": tape.observations.shape
        == (contract.EXPECTED_BASE_ROWS, contract.EXPECTED_ACTOR_FEATURES),
        "support_global_3000": support.raw.shape
        == (contract.EXPECTED_BASE_ROWS, len(contract.INVARIANT_COLUMNS)),
        "support_p99_exact": support.p99 == contract.SUPPORT_P99_THRESHOLD,
        "support_max_exact": support.loo_max == contract.SUPPORT_LOO_MAX,
        "plan_18": len(contract.collection_plan()) == contract.EXPECTED_ROLLOUTS,
        "alphas_exact": contract.ROUND_ALPHAS == (0.25, 0.50, 0.75),
        "case_order_exact": contract.CASE_IDS
        == tuple(row[0] for row in contract._CASE_ROWS),  # noqa: SLF001
        "no_updates_between": all(
            row["candidate_update_count_before"] == 0
            and row["candidate_update_count_after"] == 0
            for row in contract.collection_plan()
        ),
        "p1_unoccupied": not occupied,
        "no_teacher_model": True,
        "morphology_zero": contract.MORPHOLOGY_WEIGHT == 0.0,
    }
    passed = all(checks.values())
    if require_unoccupied and not passed:
        failed = sorted(name for name, value in checks.items() if not value)
        raise V12R13TapeDaggerExecutionError(f"P1 preflight failed: {failed}")
    payload = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": (
            "PASS_H0_V12R13_P1_CANDIDATE_EXPOSED_PREFLIGHT"
            if passed
            else "FAIL_H0_V12R13_P1_CANDIDATE_EXPOSED_PREFLIGHT"
        ),
        "passed": passed,
        "protocol_id": contract.PROTOCOL_ID,
        "stage_id": contract.STAGE_ID,
        "checks": checks,
        "source_p0": p0,
        "source_snapshot": snapshot,
        "tape_source_closure": copy.deepcopy(dict(tape.source_closure)),
        "tape_array_hashes": dict(tape.array_hashes),
        "support": contract.support_manifest(support),
        "plan": [dict(row) for row in contract.collection_plan()],
        "environment_reset_calls": 0,
        "environment_step_calls": 0,
        "checkpoint_load_calls": 0,
        "policy_query_calls": 0,
        "teacher_model_query_calls": 0,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "artifacts_written": 0,
    }
    prepared = PreparedP1(
        preflight=payload,
        tape=tape,
        support=support,
        source_snapshot=snapshot,
        source_candidate_id=str(p0["candidate_id"]),
        source_candidate_tree=copy.deepcopy(p0["candidate_tree"]),
    )
    if require_execution_lock:
        verify_execution_lock(prepared)
        payload = {
            **payload,
            "protocol_freeze": _record(contract.P1_PROTOCOL_FREEZE),
            "execution_lock": _record(contract.P1_EXECUTION_LOCK),
        }
        prepared = PreparedP1(
            preflight=payload,
            tape=tape,
            support=support,
            source_snapshot=snapshot,
            source_candidate_id=str(p0["candidate_id"]),
            source_candidate_tree=copy.deepcopy(p0["candidate_tree"]),
        )
    return prepared


def _load_production_context(candidate_module: Path) -> ExecutionContext:
    """Load the exact R6/V26 runtime only after explicit execution begins."""

    import h0_v12r6_physical_development as physical
    import run_h0_primary_split_v12r3_autonomy_recovery as runtime
    import run_h0_primary_split_v9_causal_teacher as env_source

    try:
        (
            rollout_eval,
            runtime_np,
            torch,
            RLModule,
            env_factory,
            legacy,
            v26_collector,
        ) = physical._rollout_stack()  # noqa: SLF001
    except Exception as exc:
        raise V12R13TapeDaggerExecutionError("R6/V26 stack is not ready") from exc
    if runtime_np is not np:
        raise V12R13TapeDaggerExecutionError("runtime NumPy identity drifted")
    module = RLModule.from_checkpoint(candidate_module)
    if not callable(getattr(module, "eval", None)):
        raise V12R13TapeDaggerExecutionError("P0 module lacks eval()")
    module.eval()
    return ExecutionContext(
        module=module,
        rollout_eval=rollout_eval,
        torch=torch,
        env_factory=env_factory,
        legacy=legacy,
        v26_collector=v26_collector,
        runtime=runtime,
        physical=physical,
        env_config_builder=env_source.build_env_config,
    )


def _finite_float(value: Any, *, label: str) -> float:
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(float(value))
    ):
        raise V12R13TapeDaggerExecutionError(f"{label} must be finite")
    return float(value)


def _phase_state(runtime: Any, info: Mapping[str, Any]) -> str:
    try:
        return str(runtime._phase_state(info))  # noqa: SLF001
    except Exception as exc:
        raise V12R13TapeDaggerExecutionError("active V26 phase is malformed") from exc


def _source_stable(before: Mapping[str, Any], after: Mapping[str, Any]) -> bool:
    return _strict_equal(dict(before), dict(after))


def _collection_start(
    *,
    plan_row: Mapping[str, Any],
    prepared: PreparedP1,
) -> dict[str, Any]:
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.COLLECTION_STARTED_STATUS,
        "protocol_id": contract.PROTOCOL_ID,
        "stage_id": contract.STAGE_ID,
        "trajectory_id": plan_row["trajectory_id"],
        "plan_row": dict(plan_row),
        "behavior_id": contract.BEHAVIOR_ID,
        "source_candidate_id": prepared.source_candidate_id,
        "source_p0_candidate": copy.deepcopy(prepared.source_candidate_tree),
        "source_snapshot_before": copy.deepcopy(dict(prepared.source_snapshot)),
        "tape_id": contract.TAPE_ID,
        "target_field": "frozen_teacher_mean",
        "target_binding": "same_case_and_one_based_step",
        "teacher_model_enabled": False,
        "legacy_gait_shadow_enabled": False,
        "support": contract.support_manifest(prepared.support),
        "alpha_semantics": "student_weight_zero_on_support_or_latch_fallback",
        "noise_semantics": "one_same_case_step_frozen_noise_after_mean_selection",
        "safety_signal_lag_steps": 1,
        "physical_gate_relaxed": False,
        "retry_authorized": False,
        "candidate_updates_before": 0,
        "candidate_updates_after": 0,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
    }


def _case_receipt_path(plan_row: Mapping[str, Any]) -> Path:
    return REPO_ROOT / str(plan_row["destination"]) / "receipt.json"


def _publish_case_failure(
    writer: forensic.ForensicRolloutWriter,
    exc: BaseException,
) -> None:
    if not writer.run_start_path.is_file() or os.path.lexists(writer.failure_path):
        return
    try:
        writer.publish_failure(
            end_reason="v12r13_p1_collection_failed",
            error=exc,
            status=contract.COLLECTION_FAIL_STATUS,
        )
    except BaseException:
        pass


def run_collection_case(
    *,
    plan_row: Mapping[str, Any],
    prepared: PreparedP1,
    context: ExecutionContext,
) -> dict[str, Any]:
    """Run one internally ordered P1 trajectory with forensic closure."""

    if dict(plan_row) not in [dict(row) for row in contract.collection_plan()]:
        raise V12R13TapeDaggerExecutionError("collection plan row is not canonical")
    destination = REPO_ROOT / str(plan_row["destination"])
    if os.path.lexists(destination):
        raise V12R13TapeDaggerExecutionError(
            f"collection destination exists/no-clobber: {destination}"
        )
    writer = forensic.ForensicRolloutWriter(destination, artifact_root=REPO_ROOT)
    env: Any | None = None
    rows: list[dict[str, Any]] = []
    actor_names: tuple[str, ...] = ()
    full_names: tuple[str, ...] = ()
    audit: dict[str, Any] | None = None
    info: Mapping[str, Any] = {}
    terminated = False
    truncated = False
    latch = contract.v10s_blend.SafetyLatchState()
    previous_penetration_m = 0.0
    counters = {
        "candidate_mean_query_count": 0,
        "tape_reference_lookup_count": 0,
        "tape_target_binding_count": 0,
        "teacher_model_query_count": 0,
        "legacy_shadow_query_count": 0,
        "support_query_count": 0,
        "support_fallback_count": 0,
        "safety_fallback_count": 0,
        "recovery_row_count": 0,
        "mean_blend_count": 0,
        "single_noise_application_count": 0,
        "frozen_noise_lookup_count": 0,
        "random_noise_draw_count": 0,
        "target_provenance_mismatch_count": 0,
        "tape_time_alignment_mismatch_count": 0,
        "candidate_query_mismatch_count": 0,
        "support_decision_mismatch_count": 0,
        "latch_rule_violation_count": 0,
        "blend_mismatch_count": 0,
        "noise_application_mismatch_count": 0,
        "source_or_candidate_drift_count": 0,
    }
    event_steps: list[int] = []
    started = time.monotonic()
    try:
        writer.start(_collection_start(plan_row=plan_row, prepared=prepared))
        built = context.env_config_builder(plan_row)
        if not isinstance(built, Mapping):
            raise V12R13TapeDaggerExecutionError("environment config is malformed")
        env = context.env_factory.make_cmc_env(dict(built))
        observation, reset_info = env.reset(seed=int(plan_row["runtime_seed"]))
        if not isinstance(reset_info, Mapping):
            raise V12R13TapeDaggerExecutionError("reset info is malformed")
        observation = np.ascontiguousarray(observation, dtype=np.float32)
        actor_names, full_names = context.runtime._validate_runtime_layout(  # noqa: SLF001
            module=context.module,
            env=env,
            observation=observation,
            rollout_eval=context.rollout_eval,
            np=np,
        )
        actor_names = tuple(actor_names)
        full_names = tuple(full_names)
        audit = context.runtime._new_physical_audit(  # noqa: SLF001
            reset_info=reset_info,
            legacy=context.legacy,
            np=np,
        )
        current_info: Mapping[str, Any] = dict(reset_info)
        for index in range(contract.EXPECTED_STEPS):
            step = index + 1
            observation_before = observation.copy()
            actor = np.ascontiguousarray(
                observation_before[: contract.EXPECTED_ACTOR_FEATURES],
                dtype=np.float32,
            )
            if actor.shape != (contract.EXPECTED_ACTOR_FEATURES,):
                raise V12R13TapeDaggerExecutionError("V26 actor view is malformed")
            reference = prepared.tape.reference(str(plan_row["case_id"]), step)
            counters["tape_reference_lookup_count"] += 1
            candidate_mean, candidate_std = context.runtime._query_mean_std(  # noqa: SLF001
                context.module,
                actor,
                np=np,
                torch=context.torch,
            )
            candidate_mean = np.ascontiguousarray(candidate_mean, dtype=np.float32)
            candidate_std = np.ascontiguousarray(candidate_std, dtype=np.float32)
            counters["candidate_mean_query_count"] += 1
            latch_before = latch
            phase_before = _phase_state(context.runtime, current_info)
            penetration_before = previous_penetration_m
            selected = contract.select_tape_dagger_action(
                candidate_mean=candidate_mean,
                candidate_std=candidate_std,
                observation=actor,
                reference=reference,
                requested_alpha=float(plan_row["requested_alpha"]),
                latch_state=latch_before,
                previous_penetration_m=penetration_before,
                active_v26_phase=phase_before,
                support_envelope=prepared.support,
            )
            latch = selected.latch_state
            counters["support_query_count"] += 1
            counters["support_fallback_count"] += int(selected.support_intervened)
            counters["safety_fallback_count"] += int(selected.safety_intervened)
            counters["recovery_row_count"] += int(selected.recovery)
            counters["mean_blend_count"] += 1
            counters["single_noise_application_count"] += 1
            counters["frozen_noise_lookup_count"] += 1
            counters["tape_target_binding_count"] += 1
            applied = np.ascontiguousarray(
                np.clip(
                    selected.raw_action,
                    env.action_space.low,
                    env.action_space.high,
                ),
                dtype=np.float32,
            )
            observation_after, reward, terminated, truncated, info = env.step(applied)
            observation_after = np.ascontiguousarray(
                observation_after, dtype=np.float32
            )
            if not isinstance(info, Mapping):
                raise V12R13TapeDaggerExecutionError("step info is malformed")
            physical = context.runtime._consume_physical_step(  # noqa: SLF001
                audit,
                step=step,
                info=info,
                observation_before=observation_before,
                observation_after=observation_after,
                reward=reward,
                action=selected.raw_action,
                applied_action=applied,
                extra_vectors=(
                    actor,
                    candidate_mean,
                    candidate_std,
                    reference.target_mean,
                    reference.frozen_noise,
                    selected.blended_mean,
                ),
                legacy=context.legacy,
                v26_collector=context.v26_collector,
            )
            if not isinstance(physical, Mapping):
                raise V12R13TapeDaggerExecutionError("physical payload is malformed")
            runtime_time = _finite_float(info.get("time"), label="runtime time")
            time_aligned = math.isclose(
                runtime_time,
                reference.runtime_time_s,
                rel_tol=0.0,
                abs_tol=1.0e-12,
            )
            counters["tape_time_alignment_mismatch_count"] += int(not time_aligned)
            phase = physical.get("phase")
            accepted = (
                phase.get("accepted_transitions_this_step")
                if isinstance(phase, Mapping)
                else None
            )
            if not isinstance(accepted, list):
                raise V12R13TapeDaggerExecutionError(
                    "accepted-transition journal is malformed"
                )
            if accepted:
                event_steps.append(step)
            expected_alpha = (
                0.0
                if selected.support_intervened or selected.safety_intervened
                else float(plan_row["requested_alpha"])
            )
            independent_support = prepared.support.query(actor)
            support_exact = (
                float(selected.support.distance_rms_z)
                == float(independent_support.distance_rms_z)
                and selected.support.within_p99 == independent_support.within_p99
                and selected.support.within_p99
                == (
                    float(independent_support.distance_rms_z)
                    <= contract.SUPPORT_P99_THRESHOLD
                )
            )
            expected_latch = contract.v10s_blend.advance_safety_latch(
                latch_before,
                previous_penetration_m=penetration_before,
                active_v26_phase=phase_before,
            )
            latch_exact = (
                selected.safety_intervened == expected_latch.state.active
                and selected.latch_state.active == expected_latch.state.active
                and selected.latch_entered == expected_latch.entered
                and selected.latch_released == expected_latch.released
            )
            independent_reference = prepared.tape.reference(
                str(plan_row["case_id"]), step
            )
            target_exact = (
                reference.case_id == str(plan_row["case_id"])
                and reference.step == step
                and np.array_equal(
                    reference.target_mean, independent_reference.target_mean
                )
            )
            expected_blend = np.add(
                np.multiply(
                    reference.target_mean,
                    np.float32(1.0 - expected_alpha),
                    dtype=np.float32,
                ),
                np.multiply(
                    candidate_mean,
                    np.float32(expected_alpha),
                    dtype=np.float32,
                ),
                dtype=np.float32,
            )
            blend_exact = np.array_equal(expected_blend, selected.blended_mean)
            expected_action = np.add(
                expected_blend, reference.frozen_noise, dtype=np.float32
            )
            noise_exact = np.array_equal(expected_action, selected.raw_action)
            alpha_exact = selected.effective_alpha == expected_alpha
            counters["target_provenance_mismatch_count"] += int(not target_exact)
            counters["support_decision_mismatch_count"] += int(not support_exact)
            counters["latch_rule_violation_count"] += int(not latch_exact)
            counters["blend_mismatch_count"] += int(not blend_exact or not alpha_exact)
            counters["noise_application_mismatch_count"] += int(not noise_exact)
            semantic_checks = {
                "same_case_step_tape_target": target_exact,
                "teacher_model_not_queried": True,
                "candidate_queried_once": True,
                "support_decision_exact": support_exact,
                "causal_latch_exact": latch_exact,
                "effective_alpha_exact": alpha_exact,
                "blend_exact": blend_exact,
                "single_frozen_noise_exact": noise_exact,
                "runtime_tape_time_aligned": time_aligned,
            }
            row = {
                "schema_version": contract.SCHEMA_VERSION,
                "protocol_id": contract.PROTOCOL_ID,
                "stage_id": contract.STAGE_ID,
                "trajectory_id": plan_row["trajectory_id"],
                "case_id": plan_row["case_id"],
                "requested_alpha": float(plan_row["requested_alpha"]),
                "effective_alpha": selected.effective_alpha,
                "v26_observation": actor.tolist(),
                "tape_target_mean": reference.target_mean.tolist(),
                "candidate_mean": candidate_mean.tolist(),
                "candidate_std": candidate_std.tolist(),
                "blended_mean": selected.blended_mean.tolist(),
                "single_frozen_noise": selected.frozen_noise.tolist(),
                "raw_action": selected.raw_action.tolist(),
                "applied_action": applied.tolist(),
                "target_provenance": {
                    "tape_id": contract.TAPE_ID,
                    "source_trace": copy.deepcopy(
                        prepared.tape.source_closure["traces"][str(plan_row["case_id"])]
                    ),
                    "source_case_id": reference.case_id,
                    "source_step": reference.step,
                    "source_field": "frozen_teacher_mean",
                    "teacher_model_query": False,
                    "legacy_gait_shadow_query": False,
                },
                "tape_runtime_time_s": reference.runtime_time_s,
                "runtime_time_s": runtime_time,
                "support_id": contract.SUPPORT_ID,
                "support_distance_rms_z": selected.support.distance_rms_z,
                "support_p99": contract.SUPPORT_P99_THRESHOLD,
                "support_within_p99": selected.support.within_p99,
                "support_nearest_case_id": selected.support.nearest_case_id,
                "support_nearest_step": selected.support.nearest_step,
                "support_intervened": selected.support_intervened,
                "safety_latch_active": latch.active,
                "safety_latch_entered": selected.latch_entered,
                "safety_latch_released": selected.latch_released,
                "safety_intervened": selected.safety_intervened,
                "previous_penetration_m": previous_penetration_m,
                "recovery": selected.recovery,
                "fallback_reasons": list(selected.fallback_reasons),
                "candidate_mean_query_count": 1,
                "tape_reference_lookup_count": 1,
                "teacher_model_query_count": 0,
                "legacy_shadow_query_count": 0,
                "reward": _finite_float(reward, label="reward"),
                "grf_penetration_m": _finite_float(
                    physical.get("penetration_m"), label="penetration"
                ),
                "reserve_norm_nm": _finite_float(
                    physical.get("reserve_norm_nm"), label="reserve norm"
                ),
                "residual_norm_nm": _finite_float(
                    physical.get("residual_norm_nm"), label="residual norm"
                ),
                "phase_fsm": context.legacy._jsonable(phase),
                "observer_raw_sensor_journal": context.legacy._jsonable(
                    context.physical.diagnostic_raw_journal(info, step=step)
                ),
                "checks": {
                    **context.legacy._jsonable(physical.get("checks")),
                    **semantic_checks,
                },
                "terminated": bool(terminated),
                "truncated": bool(truncated),
                "end_reason": info.get("end_reason"),
            }
            writer.write_step(step, row)
            rows.append({"step": step, **row})
            previous_penetration_m = float(physical["penetration_m"])
            observation = observation_after
            current_info = dict(info)
            if step == 1 or step % 25 == 0 or step == contract.EXPECTED_STEPS:
                elapsed = time.monotonic() - started
                eta = elapsed / step * (contract.EXPECTED_STEPS - step)
                print(
                    f"[V12R13 P1 {plan_row['trajectory_id']}] {step:3d}/500 "
                    f"elapsed={elapsed:7.1f}s eta={eta:7.1f}s",
                    flush=True,
                )
            if terminated or truncated:
                break
        if audit is None:
            raise V12R13TapeDaggerExecutionError("physical audit was not initialized")
        source_after = contract.source_snapshot(candidate_module=contract.P0_MODULE)
        stable = _source_stable(prepared.source_snapshot, source_after)
        counters["source_or_candidate_drift_count"] += int(not stable)
        physical_summary = context.runtime._physical_summary(  # noqa: SLF001
            audit,
            case=plan_row,
            rows=rows,
            info=info,
            terminated=terminated,
            truncated=truncated,
            actor_names=actor_names,
            full_names=full_names,
            legacy=context.legacy,
            v26_collector=context.v26_collector,
        )
        if not isinstance(physical_summary, Mapping):
            raise V12R13TapeDaggerExecutionError("physical summary is malformed")
        summary = {
            **dict(physical_summary),
            "schema_version": contract.SCHEMA_VERSION,
            "status": contract.COLLECTION_COMPLETE_STATUS,
            "protocol_id": contract.PROTOCOL_ID,
            "stage_id": contract.STAGE_ID,
            "trajectory_id": plan_row["trajectory_id"],
            "case_id": plan_row["case_id"],
            "requested_alpha": float(plan_row["requested_alpha"]),
            "behavior_id": contract.BEHAVIOR_ID,
            "source_candidate_id": prepared.source_candidate_id,
            "source_p0_candidate": copy.deepcopy(prepared.source_candidate_tree),
            "source_snapshot_before": copy.deepcopy(dict(prepared.source_snapshot)),
            "source_snapshot_after": source_after,
            "tape_id": contract.TAPE_ID,
            "support_id": contract.SUPPORT_ID,
            "support_p99": contract.SUPPORT_P99_THRESHOLD,
            "sample_count": len(rows),
            "persisted_label_count": len(rows),
            "accepted_transition_steps": event_steps,
            "safety_latch_activation_m": (
                contract.v10s_blend.SAFETY_LATCH_ACTIVATION_M
            ),
            "safety_latch_release_m": contract.v10s_blend.SAFETY_LATCH_RELEASE_M,
            "safety_latch_release_phase": (
                contract.v10s_blend.SAFETY_LATCH_RELEASE_PHASE
            ),
            "safety_signal_lag_steps": 1,
            "safety_latch_activation_count": latch.activation_count,
            "safety_latch_release_count": latch.release_count,
            "candidate_update_count_before": 0,
            "candidate_update_count_after": 0,
            "random_noise_draw_count": 0,
            "actor_updates": 0,
            "critic_updates": 0,
            "ppo_updates": 0,
            "protected_trials_opened": [],
            "reserve_trials_opened": [],
            **counters,
        }
        partial = {
            "schema_version": contract.SCHEMA_VERSION,
            "status": "H0_V12R13_P1_COLLECTION_PERSISTED_BEFORE_GATE",
            "protocol_id": contract.PROTOCOL_ID,
            "trajectory_id": plan_row["trajectory_id"],
            "steps": len(rows),
            "gate_evaluated": False,
            "actor_updates": 0,
            "critic_updates": 0,
            "ppo_updates": 0,
        }
        persisted = writer.finalize_before_gate(
            trace=rows,
            partial_summary=partial,
            summary=summary,
        )

        def evaluate(_records: dict[str, Any]) -> dict[str, Any]:
            gate = contract.collection_gate(summary, plan_row=plan_row)
            gate["persisted_before_gate"] = persisted
            return gate

        writer.run_gate(evaluate)
        gate = _mapping(writer.gate_path)
        if gate.get("passed") is not True:
            raise V12R13TapeDaggerExecutionError(
                f"collection gate failed: {plan_row['trajectory_id']}"
            )
        receipt = {
            "schema_version": contract.SCHEMA_VERSION,
            "status": contract.COLLECTION_PASS_STATUS,
            "passed": True,
            "protocol_id": contract.PROTOCOL_ID,
            "stage_id": contract.STAGE_ID,
            "trajectory_id": plan_row["trajectory_id"],
            "case_id": plan_row["case_id"],
            "requested_alpha": float(plan_row["requested_alpha"]),
            "source_candidate_id": prepared.source_candidate_id,
            "source_p0_candidate": copy.deepcopy(prepared.source_candidate_tree),
            "row_count": len(rows),
            "accepted_transition_steps": event_steps,
            "artifacts": writer.artifact_records(),
            "retry_authorized": False,
            "actor_updates": 0,
            "critic_updates": 0,
            "ppo_updates": 0,
            "protected_trials_opened": [],
            "reserve_trials_opened": [],
        }
        receipt_path = forensic.write_json_exclusive(
            destination / "receipt.json", receipt
        )
        return {**receipt, "receipt": _record(receipt_path)}
    except BaseException as exc:
        _publish_case_failure(writer, exc)
        if isinstance(exc, V12R13TapeDaggerExecutionError):
            raise
        raise V12R13TapeDaggerExecutionError(
            f"P1 collection failed: {plan_row['trajectory_id']}"
        ) from exc
    finally:
        if env is not None:
            active_error = sys.exc_info()[1]
            try:
                env.close()
            except BaseException as close_error:
                _publish_case_failure(writer, close_error)
                if active_error is None:
                    raise V12R13TapeDaggerExecutionError(
                        f"environment close failed: {plan_row['trajectory_id']}"
                    ) from close_error


def _transition_mask(
    rows: Sequence[Mapping[str, Any]],
) -> tuple[np.ndarray, list[int], list[int]]:
    if len(rows) != contract.EXPECTED_STEPS:
        raise V12R13TapeDaggerExecutionError("transition trace is incomplete")
    event_steps: list[int] = []
    for index, row in enumerate(rows, start=1):
        phase = row.get("phase_fsm")
        accepted = (
            phase.get("accepted_transitions_this_step")
            if isinstance(phase, Mapping)
            else None
        )
        if row.get("step") != index or not isinstance(accepted, list):
            raise V12R13TapeDaggerExecutionError("transition provenance drifted")
        if accepted:
            event_steps.append(index)
    mask = np.zeros(contract.EXPECTED_STEPS, dtype=np.bool_)
    for event_step in event_steps:
        start = max(0, event_step - 1 - contract.TRANSITION_RADIUS_STEPS)
        stop = min(
            contract.EXPECTED_STEPS,
            event_step + contract.TRANSITION_RADIUS_STEPS,
        )
        mask[start:stop] = True
    transition_steps = (np.flatnonzero(mask) + 1).astype(int).tolist()
    return mask, event_steps, transition_steps


def build_p1_corpus(
    case_receipts: Sequence[Mapping[str, Any]],
    *,
    prepared: PreparedP1,
) -> tuple[dict[str, np.ndarray], dict[str, Any]]:
    """Reconstruct the 9,000-row P1 NPZ from immutable passing traces."""

    plan = contract.collection_plan()
    if len(case_receipts) != len(plan):
        raise V12R13TapeDaggerExecutionError("P1 receipt count is not 18")
    observations: list[np.ndarray] = []
    targets: list[np.ndarray] = []
    case_ids: list[str] = []
    trajectory_ids: list[str] = []
    step_indices: list[int] = []
    requested_alphas: list[float] = []
    effective_alphas: list[float] = []
    recovery_mask: list[bool] = []
    support_distance: list[float] = []
    transition_masks: list[np.ndarray] = []
    transition_provenance: list[dict[str, Any]] = []
    for plan_row, receipt in zip(plan, case_receipts, strict=True):
        receipt_path = _case_receipt_path(plan_row)
        if (
            receipt.get("status") != contract.COLLECTION_PASS_STATUS
            or receipt.get("passed") is not True
            or receipt.get("trajectory_id") != plan_row["trajectory_id"]
            or not _record_matches(receipt.get("receipt"), receipt_path)
        ):
            raise V12R13TapeDaggerExecutionError("case receipt order/binding drifted")
        trace_path = receipt_path.parent / "trace.json"
        value = forensic.strict_json_load(trace_path)
        if not isinstance(value, list) or len(value) != contract.EXPECTED_STEPS:
            raise V12R13TapeDaggerExecutionError("case trace is incomplete")
        rows = [dict(row) for row in value if isinstance(row, Mapping)]
        if len(rows) != contract.EXPECTED_STEPS:
            raise V12R13TapeDaggerExecutionError("case trace row schema drifted")
        mask, event_steps, transition_steps = _transition_mask(rows)
        transition_masks.append(mask)
        transition_provenance.append(
            {
                "trajectory_id": plan_row["trajectory_id"],
                "case_id": plan_row["case_id"],
                "requested_alpha": plan_row["requested_alpha"],
                "accepted_event_steps": event_steps,
                "transition_radius_steps": contract.TRANSITION_RADIUS_STEPS,
                "transition_window_steps": transition_steps,
                "transition_window_row_count": int(np.count_nonzero(mask)),
                "source_trace": _record(trace_path),
            }
        )
        for step, row in enumerate(rows, start=1):
            observation = contract._float32_vector(  # noqa: SLF001
                row.get("v26_observation"),
                width=contract.EXPECTED_ACTOR_FEATURES,
                label="candidate-exposed observation",
            )
            target = contract._float32_vector(  # noqa: SLF001
                row.get("tape_target_mean"),
                width=contract.EXPECTED_ACTION_DIM,
                label="P1 tape target",
            )
            tape_target = prepared.tape.reference(
                str(plan_row["case_id"]), step
            ).target_mean
            requested = row.get("requested_alpha")
            effective = row.get("effective_alpha")
            distance = row.get("support_distance_rms_z")
            recovery = row.get("recovery")
            row_checks = {
                "identity": row.get("step") == step
                and row.get("trajectory_id") == plan_row["trajectory_id"]
                and row.get("case_id") == plan_row["case_id"],
                "target": np.array_equal(target, tape_target),
                "requested": requested == plan_row["requested_alpha"],
                "effective": effective in (0.0, plan_row["requested_alpha"]),
                "distance": isinstance(distance, (int, float))
                and not isinstance(distance, bool)
                and math.isfinite(float(distance))
                and float(distance) >= 0.0,
                "recovery": type(recovery) is bool and recovery == (effective == 0.0),
                "no_teacher": row.get("teacher_model_query_count") == 0
                and row.get("legacy_shadow_query_count") == 0,
            }
            if not all(row_checks.values()):
                failed = sorted(
                    name for name, passed in row_checks.items() if not passed
                )
                raise V12R13TapeDaggerExecutionError(
                    f"P1 corpus row failed {plan_row['trajectory_id']}/{step}: {failed}"
                )
            observations.append(observation)
            targets.append(target)
            case_ids.append(str(plan_row["case_id"]))
            trajectory_ids.append(str(plan_row["trajectory_id"]))
            step_indices.append(step)
            requested_alphas.append(float(requested))
            effective_alphas.append(float(effective))
            recovery_mask.append(bool(recovery))
            support_distance.append(float(distance))
    arrays = {
        "observations": np.ascontiguousarray(observations, dtype=np.float32),
        "targets": np.ascontiguousarray(targets, dtype=np.float32),
        "case_ids": np.ascontiguousarray(case_ids, dtype="U40"),
        "trajectory_ids": np.ascontiguousarray(trajectory_ids, dtype="U80"),
        "step_indices": np.ascontiguousarray(step_indices, dtype=np.int64),
        "requested_alpha": np.ascontiguousarray(requested_alphas, dtype=np.float32),
        "effective_alpha": np.ascontiguousarray(effective_alphas, dtype=np.float32),
        "recovery_mask": np.ascontiguousarray(recovery_mask, dtype=np.bool_),
        "support_distance": np.ascontiguousarray(support_distance, dtype=np.float64),
        "transition_mask": np.ascontiguousarray(
            np.concatenate(transition_masks), dtype=np.bool_
        ),
    }
    expected_shapes = {
        "observations": (
            contract.EXPECTED_P1_ROWS,
            contract.EXPECTED_ACTOR_FEATURES,
        ),
        "targets": (contract.EXPECTED_P1_ROWS, contract.EXPECTED_ACTION_DIM),
        **{
            name: (contract.EXPECTED_P1_ROWS,)
            for name in (
                "case_ids",
                "trajectory_ids",
                "step_indices",
                "requested_alpha",
                "effective_alpha",
                "recovery_mask",
                "support_distance",
                "transition_mask",
            )
        },
    }
    if any(arrays[name].shape != shape for name, shape in expected_shapes.items()):
        raise V12R13TapeDaggerExecutionError("P1 corpus shape drifted")
    expected_trajectories = [row["trajectory_id"] for row in plan]
    observed_trajectories = list(
        dict.fromkeys(str(value) for value in arrays["trajectory_ids"])
    )
    if observed_trajectories != expected_trajectories:
        raise V12R13TapeDaggerExecutionError("P1 trajectory order drifted")
    hashes = {name: contract.array_sha256(value) for name, value in arrays.items()}
    array_keys = sorted(arrays)
    manifest = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": "PASS_H0_V12R13_P1_CANDIDATE_EXPOSED_CORPUS_MANIFEST",
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "source_candidate_id": prepared.source_candidate_id,
        "source_p0_candidate": copy.deepcopy(prepared.source_candidate_tree),
        "row_count": contract.EXPECTED_P1_ROWS,
        "trajectory_count": contract.EXPECTED_ROLLOUTS,
        "ordering": "alpha_major_then_canonical_case_ids_then_one_based_step",
        "trajectory_ids": expected_trajectories,
        "array_keys": array_keys,
        "array_shapes": {name: list(arrays[name].shape) for name in array_keys},
        "array_dtypes": {name: arrays[name].dtype.str for name in array_keys},
        "array_hashes": hashes,
        "target_field": "same_case_step_frozen_teacher_mean",
        "observation_field": "candidate_exposed_pre_action_v26_observation",
        "transition_window_provenance": transition_provenance,
        "base_rows_for_p2": contract.EXPECTED_BASE_ROWS,
        "candidate_exposed_rows_for_p2": contract.EXPECTED_P1_ROWS,
        "expected_cumulative_p2_rows": contract.EXPECTED_P2_ROWS,
        "teacher_model_queries": 0,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
    }
    return arrays, manifest


def build_p1_ledger_payload(
    *,
    prepared: PreparedP1,
    source_after: Mapping[str, Any],
    trajectory_ids: Sequence[str],
    receipt_records: Sequence[Mapping[str, Any]],
    manifest: Mapping[str, Any],
    corpus_record: Mapping[str, Any],
    manifest_record: Mapping[str, Any],
) -> dict[str, Any]:
    """Build the exact P1 pipeline ledger payload consumed by the P2 loader."""

    receipts = [dict(record) for record in receipt_records]
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.P1_LEDGER_STATUS,
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "stage_id": contract.STAGE_ID,
        "protocol_freeze": _record(contract.P1_PROTOCOL_FREEZE),
        "execution_lock": _record(contract.P1_EXECUTION_LOCK),
        "source_candidate_id": prepared.source_candidate_id,
        "source_p0_candidate": copy.deepcopy(prepared.source_candidate_tree),
        "source_snapshot_before": copy.deepcopy(dict(prepared.source_snapshot)),
        "source_snapshot_after": dict(source_after),
        "completed_trajectory_ids": [str(value) for value in trajectory_ids],
        "completed_receipts": receipts,
        "row_count": contract.EXPECTED_P1_ROWS,
        "trajectory_count": contract.EXPECTED_ROLLOUTS,
        "array_hashes": dict(manifest["array_hashes"]),
        "artifacts": {
            "corpus": dict(corpus_record),
            "corpus_manifest": dict(manifest_record),
            "case_receipts": receipts,
        },
        "environment_resets": contract.EXPECTED_ROLLOUTS,
        "environment_steps": contract.EXPECTED_P1_ROWS,
        "policy_rollouts": contract.EXPECTED_ROLLOUTS,
        "candidate_mean_queries": contract.EXPECTED_P1_ROWS,
        "teacher_model_queries": 0,
        "legacy_shadow_queries": 0,
        "candidate_created": False,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
        "retry_authorized": False,
    }


def build_p1_receipt_payload(
    *,
    prepared: PreparedP1,
    receipt_records: Sequence[Mapping[str, Any]],
    manifest: Mapping[str, Any],
    corpus_record: Mapping[str, Any],
    manifest_record: Mapping[str, Any],
    ledger_record: Mapping[str, Any],
) -> dict[str, Any]:
    """Build the exact P1 stage receipt payload consumed by the P2 loader."""

    receipts = [dict(record) for record in receipt_records]
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.P1_RECEIPT_STATUS,
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "stage_id": contract.STAGE_ID,
        "protocol_freeze": _record(contract.P1_PROTOCOL_FREEZE),
        "execution_lock": _record(contract.P1_EXECUTION_LOCK),
        "source_candidate_id": prepared.source_candidate_id,
        "row_count": contract.EXPECTED_P1_ROWS,
        "trajectory_count": contract.EXPECTED_ROLLOUTS,
        "array_hashes": dict(manifest["array_hashes"]),
        "transition_window_provenance": copy.deepcopy(
            manifest["transition_window_provenance"]
        ),
        "artifacts": {
            "corpus": dict(corpus_record),
            "corpus_manifest": dict(manifest_record),
            "pipeline_ledger": dict(ledger_record),
            "source_p0_candidate": copy.deepcopy(prepared.source_candidate_tree),
            "case_receipts": receipts,
        },
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "environment_steps": contract.EXPECTED_P1_ROWS,
        "policy_rollouts": contract.EXPECTED_ROLLOUTS,
        "candidate_created": False,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
        "retry_authorized": False,
    }


def _publish_plan_failure(
    *,
    exc: BaseException,
    completed: Sequence[Mapping[str, Any]],
    source_candidate_id: str,
) -> None:
    if not contract.P1_ROOT.is_dir() or os.path.lexists(
        contract.P1_ROOT / "failure.json"
    ):
        return
    try:
        forensic.write_json_exclusive(
            contract.P1_ROOT / "failure.json",
            {
                "schema_version": contract.SCHEMA_VERSION,
                "status": contract.P1_FAILURE_STATUS,
                "passed": False,
                "protocol_id": contract.PROTOCOL_ID,
                "source_candidate_id": source_candidate_id,
                "completed_trajectory_ids": [
                    item.get("trajectory_id") for item in completed
                ],
                "completed_receipts": [item.get("receipt") for item in completed],
                "error": {"type": type(exc).__name__, "message": str(exc)},
                "retry_authorized": False,
                "actor_updates": 0,
                "critic_updates": 0,
                "ppo_updates": 0,
            },
        )
    except BaseException:
        pass


def execute_p1(
    *,
    context_loader: Callable[[Path], ExecutionContext] = _load_production_context,
) -> dict[str, Any]:
    """Execute all 18 rows in order and publish one terminal receipt."""

    prepared = prepare_p1(require_unoccupied=True, enforce_canonical=True)
    if os.path.lexists(contract.P1_ROOT):
        raise V12R13TapeDaggerExecutionError("P1 output root is occupied")
    completed: list[dict[str, Any]] = []
    forensic.write_json_exclusive(
        contract.P1_ROOT / "run_start.json",
        {
            "schema_version": contract.SCHEMA_VERSION,
            "status": "H0_V12R13_P1_CANDIDATE_EXPOSED_PIPELINE_STARTED",
            "protocol_id": contract.PROTOCOL_ID,
            "source_candidate_id": prepared.source_candidate_id,
            "source_p0_candidate": copy.deepcopy(prepared.source_candidate_tree),
            "preflight": copy.deepcopy(dict(prepared.preflight)),
            "plan": [dict(row) for row in contract.collection_plan()],
            "retry_authorized": False,
            "actor_updates": 0,
            "critic_updates": 0,
            "ppo_updates": 0,
        },
    )
    try:
        context = context_loader(contract.P0_MODULE)
        for plan_row in contract.collection_plan():
            completed.append(
                run_collection_case(
                    plan_row=plan_row,
                    prepared=prepared,
                    context=context,
                )
            )
        source_after = contract.source_snapshot(candidate_module=contract.P0_MODULE)
        if not _source_stable(prepared.source_snapshot, source_after):
            raise V12R13TapeDaggerExecutionError(
                "source or P0 candidate changed during P1"
            )
        arrays, manifest = build_p1_corpus(completed, prepared=prepared)
        corpus_path = _write_npz_exclusive(contract.P1_CORPUS, arrays)
        manifest = {
            **manifest,
            "corpus_artifact": _record(corpus_path),
        }
        manifest_path = forensic.write_json_exclusive(
            contract.P1_CORPUS_MANIFEST, manifest
        )
        receipt_records = [dict(item["receipt"]) for item in completed]
        trajectory_ids = [str(item["trajectory_id"]) for item in completed]
        ledger = build_p1_ledger_payload(
            prepared=prepared,
            source_after=source_after,
            trajectory_ids=trajectory_ids,
            receipt_records=receipt_records,
            manifest=manifest,
            corpus_record=_record(corpus_path),
            manifest_record=_record(manifest_path),
        )
        ledger_path = forensic.write_json_exclusive(contract.P1_LEDGER, ledger)
        receipt = build_p1_receipt_payload(
            prepared=prepared,
            receipt_records=receipt_records,
            manifest=manifest,
            corpus_record=_record(corpus_path),
            manifest_record=_record(manifest_path),
            ledger_record=_record(ledger_path),
        )
        receipt_path = forensic.write_json_exclusive(contract.P1_RECEIPT, receipt)
        return {**receipt, "receipt": _record(receipt_path)}
    except BaseException as exc:
        _publish_plan_failure(
            exc=exc,
            completed=completed,
            source_candidate_id=prepared.source_candidate_id,
        )
        if isinstance(exc, V12R13TapeDaggerExecutionError):
            raise
        raise V12R13TapeDaggerExecutionError("P1 pipeline failed") from exc


EXECUTION_ACKNOWLEDGEMENT = "V12R13_P1_ONE_SHOT_18_ROLLOUTS_9000_STEPS"
GOVERNANCE_ACKNOWLEDGEMENT = "V12R13_P1_FREEZE_REVIEWED_SOURCES"


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--describe", action="store_true")
    mode.add_argument("--write-protocol-freeze", action="store_true")
    mode.add_argument("--write-execution-lock", action="store_true")
    mode.add_argument("--preflight", action="store_true")
    mode.add_argument("--execute", action="store_true")
    parser.add_argument(
        "--acknowledge-one-shot",
        default=None,
        help=(f"required with --execute; exact value: {EXECUTION_ACKNOWLEDGEMENT}"),
    )
    parser.add_argument(
        "--acknowledge-governance",
        default=None,
        help=(
            "required for either governance write; exact value: "
            f"{GOVERNANCE_ACKNOWLEDGEMENT}"
        ),
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        if args.execute:
            if args.acknowledge_one_shot != EXECUTION_ACKNOWLEDGEMENT:
                raise V12R13TapeDaggerExecutionError(
                    "--execute requires the exact one-shot acknowledgement"
                )
            result = execute_p1()
        elif args.write_protocol_freeze or args.write_execution_lock:
            if args.acknowledge_governance != GOVERNANCE_ACKNOWLEDGEMENT:
                raise V12R13TapeDaggerExecutionError(
                    "governance write requires the exact reviewed-source "
                    "acknowledgement"
                )
            result = (
                write_protocol_freeze()
                if args.write_protocol_freeze
                else write_execution_lock()
            )
        elif args.preflight:
            result = dict(prepare_p1().preflight)
        else:
            result = contract.build_protocol_description()
        print(json.dumps(result, indent=2, sort_keys=True, allow_nan=False))
        return 0 if result.get("passed", True) is True else 2
    except (
        V12R13TapeDaggerExecutionError,
        contract.V12R13TapeDaggerError,
        forensic.ForensicRolloutError,
    ) as exc:
        print(
            json.dumps(
                {
                    "status": "FAIL_H0_V12R13_P1_TAPE_DAGGER_CLI",
                    "passed": False,
                    "error_type": type(exc).__name__,
                    "error": str(exc),
                },
                indent=2,
                sort_keys=True,
            ),
            file=sys.stderr,
        )
        return 2


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = [
    "ExecutionContext",
    "PreparedP1",
    "V12R13TapeDaggerExecutionError",
    "build_p1_corpus",
    "build_p1_ledger_payload",
    "build_p1_receipt_payload",
    "execute_p1",
    "main",
    "prepare_p1",
    "run_collection_case",
    "verify_execution_lock",
    "verify_p0_candidate",
    "verify_protocol_freeze",
    "write_execution_lock",
    "write_protocol_freeze",
]
