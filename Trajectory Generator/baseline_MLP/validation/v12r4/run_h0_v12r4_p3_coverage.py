"""Execute the authorized V12R4 P3 coverage pipeline exactly once.

The execution is additive, no-clobber, receipt-bound, and terminal on the
first failure.  It performs four P2-student shielded collection episodes, one
fresh-H0 P3 fit, a candidate freeze, and six pure-policy development episodes
in nominal-first order.  Q2 qualification remains unopened throughout.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import math
import os
import platform
import secrets
import sys
import time
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
LOCAL_VALIDATION = REPO_ROOT / "Trajectory Generator" / "baseline_MLP" / "validation"
REVISION_ROOT = Path(__file__).resolve().parent
for _root in (
    REPO_ROOT,
    REPO_ROOT / "validation",
    REPO_ROOT / "Trajectory Generator",
    REPO_ROOT / "Trajectory Generator" / "baseline_MLP",
    LOCAL_VALIDATION,
    LOCAL_VALIDATION / "v12r3",
    REVISION_ROOT,
):
    if str(_root) not in sys.path:
        sys.path.insert(0, str(_root))

import h0_forensic_rollout as forensic  # noqa: E402
import h0_primary_split_v10_coherent_teacher as coherent_teacher  # noqa: E402
import h0_primary_split_v10s_blend as safe_dagger  # noqa: E402
import h0_primary_split_v10s_fit as v10s_fit  # noqa: E402
import run_h0_primary_split_v9_causal_teacher as env_source  # noqa: E402
import run_h0_primary_split_v12r3_autonomy_recovery as runtime  # noqa: E402
import h0_v12r4_p3_coverage_contract as contract  # noqa: E402
import h0_v12r4_p3_coverage_fitter as fit_engine  # noqa: E402
import freeze_h0_v12r4_p3_coverage as freezer  # noqa: E402


class V12R4ExecutionError(RuntimeError):
    """Raised when continuing would violate the frozen one-shot protocol."""


def resolve_relative(path: str | os.PathLike[str] | PurePosixPath) -> Path:
    raw = path.as_posix() if isinstance(path, PurePosixPath) else os.fspath(path)
    pure = PurePosixPath(raw)
    if not raw or pure.is_absolute() or ".." in pure.parts or pure.as_posix() != raw:
        raise V12R4ExecutionError(f"non-canonical repository path: {raw!r}")
    return REPO_ROOT.joinpath(*pure.parts)


RUN_ROOT = resolve_relative(contract.RUN_ROOT)
LOCK_PATH = resolve_relative(contract.EXECUTION_LOCK_PATH)
PROTOCOL_FREEZE_PATH = resolve_relative(contract.PROTOCOL_FREEZE_PATH)
DESIGN_AUDIT_PATH = resolve_relative(contract.DESIGN_AUDIT_PATH)
PIPELINE_CLAIM_PATH = resolve_relative(contract.PIPELINE_CLAIM_PATH)
PIPELINE_LEDGER_PATH = resolve_relative(contract.PIPELINE_LEDGER_PATH)
WORKER_CLAIMS_ROOT = resolve_relative(contract.WORKER_CLAIMS_ROOT)
CANDIDATE_FREEZE_PATH = resolve_relative(contract.CANDIDATE_FREEZE_PATH)
SOURCE_H0_MODULE = resolve_relative(contract.SOURCE_H0_MODULE_PATH).resolve()
P2_MODULE = resolve_relative(contract.P2_MODULE_TREE["path"]).resolve()

_ACTIVITY_NAMES = (
    "environment_reset_calls",
    "environment_step_calls",
    "raw_sensor_sample_count",
    "teacher_query_count",
    "actor_fit_stage_calls_attempted",
    "actor_fit_executions_confirmed",
    "actor_updates_attempted",
    "actor_updates_confirmed",
    "adamw_epochs_completed",
    "lbfgs_closure_calls",
)
_ACTIVITY_TOTALS = {name: 0 for name in _ACTIVITY_NAMES}
_STAGE_ACTIVITY: dict[str, dict[str, Any]] = {}
_ACTIVE_STAGE_ID: str | None = None


def _reset_activity() -> None:
    global _ACTIVE_STAGE_ID
    _ACTIVE_STAGE_ID = None
    _STAGE_ACTIVITY.clear()
    for name in _ACTIVITY_NAMES:
        _ACTIVITY_TOTALS[name] = 0


def _begin_stage_activity(stage_id: str) -> None:
    global _ACTIVE_STAGE_ID
    if stage_id in _STAGE_ACTIVITY:
        raise V12R4ExecutionError(f"stage activity already opened: {stage_id}")
    _ACTIVE_STAGE_ID = stage_id
    _STAGE_ACTIVITY[stage_id] = {
        "stage_id": stage_id,
        "stage_kind": contract.stage_descriptor(stage_id)["kind"],
        **{name: 0 for name in _ACTIVITY_NAMES},
    }


def _activity_increment(name: str, amount: int = 1) -> None:
    if name not in _ACTIVITY_TOTALS:
        raise V12R4ExecutionError(f"unknown activity counter: {name}")
    if type(amount) is not int or amount < 0:
        raise V12R4ExecutionError("activity increment must be a nonnegative int")
    if _ACTIVE_STAGE_ID is None:
        raise V12R4ExecutionError("activity increment outside a claimed stage")
    _ACTIVITY_TOTALS[name] += amount
    _STAGE_ACTIVITY[_ACTIVE_STAGE_ID][name] += amount


def _mapping(path: str | Path) -> dict[str, Any]:
    try:
        value = forensic.strict_json_load(Path(path))
    except Exception as exc:
        raise V12R4ExecutionError(f"invalid strict JSON object: {path}") from exc
    if not isinstance(value, Mapping):
        raise V12R4ExecutionError(f"expected JSON object: {path}")
    return dict(value)


def _record(path: str | Path) -> dict[str, Any]:
    return forensic.artifact_record(Path(path), artifact_root=REPO_ROOT)


def _tree(path: str | Path) -> dict[str, Any]:
    target = Path(path)
    relative = target.relative_to(REPO_ROOT) if target.is_absolute() else target
    return freezer.tree_record(PurePosixPath(relative.as_posix()))


def _q2_unopened() -> bool:
    return all(
        not os.path.lexists(resolve_relative(path))
        for path in contract.Q2_UNOPENED_PATHS.values()
    )


def _load_stack() -> tuple[Any, Any, Any, Any, Any, Any, Any]:
    try:
        return runtime._load_rollout_stack()
    except Exception as exc:
        raise V12R4ExecutionError("V26 inference stack is not ready") from exc


def _module_preflight(
    path: Path, *, expected_tree: Mapping[str, Any]
) -> dict[str, Any]:
    import numpy as np
    import torch
    from ray.rllib.core.rl_module.rl_module import RLModule

    observed_tree = _tree(path)
    if observed_tree != dict(expected_tree):
        raise V12R4ExecutionError(f"checkpoint tree drifted: {path}")
    module = RLModule.from_checkpoint(path.resolve())
    module.eval()
    actor = np.zeros(contract.EXPECTED_ACTOR_FEATURES, dtype=np.float32)
    actor[1] = np.float32(1.0)
    mean, std = runtime._query_mean_std(module, actor, np=np, torch=torch)
    logits = np.concatenate((mean, np.log(std)), dtype=np.float32)
    if logits.shape != (4,) or not np.all(np.isfinite(logits)):
        raise V12R4ExecutionError(f"checkpoint inference is malformed: {path}")
    return {
        "module": observed_tree,
        "checkpoint_path_absolute": str(path.resolve()),
        "action_dist_inputs_shape": [1, 4],
        "action_dist_inputs_finite": True,
        "eval_mode": True,
    }


def build_execution_lock(*, require_unoccupied: bool = True) -> dict[str, Any]:
    protocol = freezer.verify_protocol_freeze()
    design = _mapping(DESIGN_AUDIT_PATH)
    occupancy = {
        "lock_absent": not os.path.lexists(LOCK_PATH),
        "run_root_absent": not os.path.lexists(RUN_ROOT),
        "pipeline_claim_absent": not os.path.lexists(PIPELINE_CLAIM_PATH),
        "pipeline_ledger_absent": not os.path.lexists(PIPELINE_LEDGER_PATH),
        "q2_unopened": _q2_unopened(),
    }
    p2_preflight = _module_preflight(P2_MODULE, expected_tree=contract.P2_MODULE_TREE)
    source_tree = _tree(SOURCE_H0_MODULE)
    checks = {
        "protocol": protocol.get("status") == contract.PROTOCOL_FREEZE_STATUS
        and protocol.get("passed") is True,
        "design_audit": design.get("status") == "PASS_H0_V12R4_P3_COVERAGE_DESIGN_AUDIT"
        and design.get("passed") is True,
        "contract": contract.contract_self_check().get("passed") is True,
        "source_h0": source_tree.get("tree_sha256") == contract.SOURCE_H0_TREE_SHA256,
        "p2_absolute_rlmodule_preflight": p2_preflight.get("action_dist_inputs_finite")
        is True,
        "occupancy": all(occupancy.values()),
        "authority": all(
            contract.AUTHORITY[name]
            for name in (
                "execution_lock_authorized",
                "collection_execution_authorized",
                "actor_fit_execution_authorized",
                "candidate_freeze_authorized",
                "development_execution_authorized",
            )
        ),
    }
    passed = all(checks.values())
    payload = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": (
            contract.EXECUTION_LOCK_STATUS if passed else contract.TERMINAL_FAIL_STATUS
        ),
        "passed": passed,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "revision": contract.REVISION,
        "authority": copy.deepcopy(contract.AUTHORITY),
        "checks": checks,
        "occupancy": occupancy,
        "protocol_freeze": _record(PROTOCOL_FREEZE_PATH),
        "design_audit": _record(DESIGN_AUDIT_PATH),
        "q2_design_freeze": _record(resolve_relative(contract.Q2_DESIGN_FREEZE_PATH)),
        "p2_collection_student_preflight": p2_preflight,
        "source_h0": source_tree,
        "candidate_selection": {
            "rule": contract.CANDIDATE_SELECTION_RULE,
            "module_path": contract.P3_MODULE_PATH.as_posix(),
            "candidate_id": "DEFERRED_UNTIL_FIT_P3",
            "candidate_tree_sha256": "DEFERRED_UNTIL_FIT_P3",
        },
        "stage_order": list(contract.STAGE_IDS),
        "run_root": contract.RUN_ROOT.as_posix(),
        "platform": {
            "system": platform.system(),
            "machine": platform.machine(),
            "python": platform.python_version(),
            "executable": str(Path(sys.executable).resolve()),
        },
        "one_shot": True,
        "retry_authorized": False,
        "resume_authorized": False,
        "rescue_authorized": False,
        "sweep_authorized": False,
        "qualification_execution_authorized": False,
        "environment_reset_calls": 0,
        "environment_step_calls": 0,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
    }
    if require_unoccupied and not passed:
        failed = [name for name, value in checks.items() if not value]
        raise V12R4ExecutionError(f"execution lock preflight failed: {failed}")
    return payload


def prepare_execution_lock() -> dict[str, Any]:
    if os.path.lexists(LOCK_PATH):
        raise V12R4ExecutionError("execution lock exists/no-clobber")
    payload = build_execution_lock(require_unoccupied=True)
    forensic.write_json_exclusive(LOCK_PATH, payload)
    return verify_execution_lock(require_run_root_absent=True)


def verify_execution_lock(*, require_run_root_absent: bool = False) -> dict[str, Any]:
    observed = _mapping(LOCK_PATH)
    protocol = freezer.verify_protocol_freeze()
    q2_design_record = _record(resolve_relative(contract.Q2_DESIGN_FREEZE_PATH))
    checks = observed.get("checks")
    if (
        observed.get("schema_version") != contract.SCHEMA_VERSION
        or observed.get("status") != contract.EXECUTION_LOCK_STATUS
        or observed.get("passed") is not True
        or observed.get("protocol_id") != contract.PROTOCOL_ID
        or observed.get("stage_order") != list(contract.STAGE_IDS)
        or not isinstance(checks, Mapping)
        or not all(checks.values())
        or observed.get("protocol_freeze") != _record(PROTOCOL_FREEZE_PATH)
        or observed.get("design_audit") != _record(DESIGN_AUDIT_PATH)
        or observed.get("q2_design_freeze") != q2_design_record
        or protocol.get("q2_design_freeze") != q2_design_record
        or observed.get("candidate_selection")
        != {
            "rule": contract.CANDIDATE_SELECTION_RULE,
            "module_path": contract.P3_MODULE_PATH.as_posix(),
            "candidate_id": "DEFERRED_UNTIL_FIT_P3",
            "candidate_tree_sha256": "DEFERRED_UNTIL_FIT_P3",
        }
    ):
        raise V12R4ExecutionError("execution lock content drifted")
    if require_run_root_absent and os.path.lexists(RUN_ROOT):
        raise V12R4ExecutionError("R4 run root already claimed")
    return observed


def _claim_path(stage_id: str) -> Path:
    return WORKER_CLAIMS_ROOT / f"{stage_id}.json"


def _stage_receipt_path(stage_id: str) -> Path:
    descriptor = contract.stage_descriptor(stage_id)
    kind = descriptor["kind"]
    if kind == "attestation":
        return RUN_ROOT / "p2_collection_source_attestation_receipt.json"
    if kind == "collection":
        return resolve_relative(descriptor["case"]["destination"]) / "receipt.json"
    if kind == "corpus":
        return RUN_ROOT / "p3_corpus_assembly_receipt.json"
    if kind == "fit":
        return resolve_relative(contract.FIT_ROOT) / "receipt.json"
    if kind == "candidate_freeze":
        return CANDIDATE_FREEZE_PATH
    if kind == "development":
        return resolve_relative(descriptor["case"]["destination"]) / "receipt.json"
    return resolve_relative(contract.FINAL_DEVELOPMENT_RECEIPT_PATH)


def _claim_run_root() -> tuple[str, dict[str, Any]]:
    if os.path.lexists(RUN_ROOT):
        raise V12R4ExecutionError("R4 run root exists/no retry")
    verify_execution_lock(require_run_root_absent=True)
    RUN_ROOT.mkdir(parents=True, exist_ok=False)
    WORKER_CLAIMS_ROOT.mkdir(parents=False, exist_ok=False)
    token = secrets.token_hex(32)
    token_hash = hashlib.sha256(token.encode("utf-8")).hexdigest()
    claim = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": "CLAIMED_H0_V12R4_P3_COVERAGE_PIPELINE_ONCE",
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "execution_token_sha256": token_hash,
        "execution_lock": _record(LOCK_PATH),
        "protocol_freeze": _record(PROTOCOL_FREEZE_PATH),
        "stage_order": list(contract.STAGE_IDS),
        "retry_authorized": False,
        "resume_authorized": False,
        "rescue_authorized": False,
        "sweep_authorized": False,
    }
    forensic.write_json_exclusive(PIPELINE_CLAIM_PATH, claim)
    return token_hash, claim


def _write_worker_claim(stage_id: str, token_hash: str) -> dict[str, Any]:
    index = contract.STAGE_IDS.index(stage_id)
    previous = []
    for prior in contract.STAGE_IDS[:index]:
        receipt = _stage_receipt_path(prior)
        if not receipt.is_file():
            raise V12R4ExecutionError(f"prior stage receipt missing: {prior}")
        previous.append({"stage_id": prior, "receipt": _record(receipt)})
    path = _claim_path(stage_id)
    if os.path.lexists(path) or os.path.lexists(_stage_receipt_path(stage_id)):
        raise V12R4ExecutionError(f"stage already consumed: {stage_id}")
    payload = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": "CLAIMED_H0_V12R4_STAGE_ONCE",
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "stage_id": stage_id,
        "stage_index": index,
        "stage_kind": contract.stage_descriptor(stage_id)["kind"],
        "execution_token_sha256": token_hash,
        "pipeline_claim": _record(PIPELINE_CLAIM_PATH),
        "previous_receipts": previous,
        "retry_authorized": False,
    }
    forensic.write_json_exclusive(path, payload)
    return payload


def _diagnostic_raw_journal(info: Mapping[str, Any], *, step: int) -> dict[str, Any]:
    """Capture only observer diagnostics; never consumed by actions or gates."""

    raw_samples = info.get("binary_phase_sensor_samples")
    phase_samples = info.get("phase_sensor_samples")
    samples: list[dict[str, Any]] = []
    if isinstance(raw_samples, Sequence) and not isinstance(raw_samples, (str, bytes)):
        phase_rows = (
            list(phase_samples)
            if isinstance(phase_samples, Sequence)
            and not isinstance(phase_samples, (str, bytes))
            else []
        )
        for index, raw in enumerate(raw_samples):
            raw_map = raw if isinstance(raw, Mapping) else {}
            phase_map = (
                phase_rows[index]
                if index < len(phase_rows) and isinstance(phase_rows[index], Mapping)
                else {}
            )
            samples.append(
                {
                    "sensor_index": index + 1,
                    "time_s": raw_map.get("time_s", phase_map.get("time_s")),
                    "left_heel_contact": raw_map.get("left_heel_contact"),
                    "left_toe_contact": raw_map.get("left_toe_contact"),
                    "left_heel_clearance_m": phase_map.get("left_heel_clearance_m"),
                    "left_toe_clearance_m": phase_map.get("left_toe_clearance_m"),
                    "left_heel_normal_n": phase_map.get("left_heel_normal_n"),
                    "left_toe_normal_n": phase_map.get("left_toe_normal_n"),
                }
            )
    binary = info.get("binary_phase_fsm")
    binary_map = binary if isinstance(binary, Mapping) else {}
    return {
        "observer_only": True,
        "control_dependency": False,
        "gate_dependency": False,
        "blocker_if_field_unavailable": False,
        "step": step,
        "samples": samples,
        "online_grf": copy.deepcopy(info.get("online_grf")),
        "online_grf_detector": copy.deepcopy(info.get("online_grf_detector")),
        "accepted_events": copy.deepcopy(binary_map.get("events_this_step", [])),
        "pending_event": copy.deepcopy(binary_map.get("pending_event")),
        "availability": {
            "binary_contact_samples": bool(samples),
            "clearance": any(
                row["left_heel_clearance_m"] is not None
                or row["left_toe_clearance_m"] is not None
                for row in samples
            ),
            "per_sensor_analog_grf": any(
                row["left_heel_normal_n"] is not None
                or row["left_toe_normal_n"] is not None
                for row in samples
            ),
            "step_online_grf": isinstance(info.get("online_grf"), Mapping),
        },
    }


def _rollout_stack() -> tuple[Any, Any, Any, Any, Any, Any, Any]:
    return _load_stack()


def _run_attestation() -> dict[str, Any]:
    stage_id = "attest_p2_collection_source"
    checks = {
        "p2_module": _tree(P2_MODULE) == contract.P2_MODULE_TREE,
        "p2_corpus": _record(contract.P2_CORPUS_ARTIFACT["path"])
        == contract.P2_CORPUS_ARTIFACT,
        "p2_report": _record(contract.P2_ADAPTATION_REPORT_ARTIFACT["path"])
        == contract.P2_ADAPTATION_REPORT_ARTIFACT,
        "p2_history": _record(contract.P2_ADAPTATION_HISTORY_ARTIFACT["path"])
        == contract.P2_ADAPTATION_HISTORY_ARTIFACT,
        "p2_nonpromotable": not resolve_relative(
            contract.P2_ROOT / "receipt.json"
        ).exists(),
        "q2_unopened": _q2_unopened(),
    }
    if not all(checks.values()):
        raise V12R4ExecutionError(f"P2 source attestation failed: {checks}")
    payload = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.SOURCE_ATTEST_PASS_STATUS,
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "stage_id": stage_id,
        "checks": checks,
        "candidate_role": "P2_COLLECTION_STUDENT_ONLY_NONPROMOTABLE",
        "p2_module": copy.deepcopy(contract.P2_MODULE_TREE),
        "p2_corpus": copy.deepcopy(contract.P2_CORPUS_ARTIFACT),
        "pipeline_claim": _record(PIPELINE_CLAIM_PATH),
        "worker_claim": _record(_claim_path(stage_id)),
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
    }
    forensic.write_json_exclusive(_stage_receipt_path(stage_id), payload)
    return payload


def _run_collection(case_id: str) -> dict[str, Any]:
    (
        rollout_eval,
        np,
        torch,
        RLModule,
        env_factory,
        legacy,
        v26_collector,
    ) = _rollout_stack()
    stage_id = f"collect_cov__{case_id}"
    case = contract.canonical_collection_case(case_id)
    if (
        safe_dagger.SAFETY_LATCH_ACTIVATION_M != contract.SAFETY_LATCH_ACTIVATION_M
        or safe_dagger.SAFETY_LATCH_RELEASE_M != contract.SAFETY_LATCH_RELEASE_M
        or safe_dagger.SAFETY_LATCH_RELEASE_PHASE != contract.SAFETY_LATCH_RELEASE_PHASE
    ):
        raise V12R4ExecutionError("shield latch constants drifted")
    destination = resolve_relative(case["destination"])
    if os.path.lexists(destination):
        raise V12R4ExecutionError(f"collection destination exists: {case_id}")
    candidate = RLModule.from_checkpoint(P2_MODULE.resolve())
    teacher = RLModule.from_checkpoint(SOURCE_H0_MODULE.resolve())
    candidate.eval()
    teacher.eval()
    innovations = runtime._frozen_innovations(
        case_id, action_selection=str(case["action_selection"]), np=np
    )
    env = env_factory.make_cmc_env(env_source.build_env_config(case))
    writer = forensic.ForensicRolloutWriter(destination, artifact_root=REPO_ROOT)
    writer.start(
        {
            "schema_version": contract.SCHEMA_VERSION,
            "status": "STARTED_H0_V12R4_SHIELDED_COLLECTION",
            "protocol_id": contract.PROTOCOL_ID,
            "stage_id": stage_id,
            "case": copy.deepcopy(case),
            "candidate_role": "P2_COLLECTION_STUDENT_ONLY_NONPROMOTABLE",
            "candidate_module": copy.deepcopy(contract.P2_MODULE_TREE),
            "teacher_id": contract.TEACHER_ID,
            "pipeline_claim": _record(PIPELINE_CLAIM_PATH),
            "worker_claim": _record(_claim_path(stage_id)),
            "retry_authorized": False,
        }
    )
    rows: list[dict[str, Any]] = []
    label_observations: list[Any] = []
    label_actions: list[Any] = []
    actor_names: tuple[str, ...] = ()
    full_names: tuple[str, ...] = ()
    audit: dict[str, Any] | None = None
    info: Mapping[str, Any] = {}
    terminated = False
    truncated = False
    latch = safe_dagger.SafetyLatchState()
    previous_penetration_m = 0.0
    max_takeover_streak = 0
    takeover_streak = 0
    started = time.monotonic()
    try:
        _activity_increment("environment_reset_calls")
        observation, reset_info = env.reset(seed=int(case["runtime_seed"]))
        observation = np.ascontiguousarray(observation, dtype=np.float32)
        actor_names, full_names = runtime._validate_runtime_layout(
            module=candidate,
            env=env,
            observation=observation,
            rollout_eval=rollout_eval,
            np=np,
        )
        rollout_eval._validate_module_observation_contract(
            teacher, actor_names, full_names
        )
        audit = runtime._new_physical_audit(reset_info=reset_info, legacy=legacy, np=np)
        body_weight_n = float(env.unwrapped._body_weight_n)
        if not math.isfinite(body_weight_n) or body_weight_n <= 0.0:
            raise V12R4ExecutionError("body weight is malformed")
        shadow = coherent_teacher.LegacyGaitShadow.from_runtime_phase_fsm(
            env.unwrapped._phase_fsm
        )
        student = np.ascontiguousarray(
            observation[: contract.EXPECTED_ACTOR_FEATURES], dtype=np.float32
        )
        teacher_view = coherent_teacher.build_teacher_view(
            student,
            actor_names,
            reset_info,
            body_weight_n=body_weight_n,
            shadow=shadow,
            reset_boundary=True,
        )
        current_info: Mapping[str, Any] = dict(reset_info)
        for index in range(contract.EXPECTED_STEPS):
            step = index + 1
            observation_before = observation.copy()
            student_before = student.copy()
            teacher_before = teacher_view.copy()
            coherent_teacher.assert_coherent_pair(student_before, teacher_before)
            candidate_mean, candidate_std = runtime._query_mean_std(
                candidate, student_before, np=np, torch=torch
            )
            teacher_mean, teacher_std = runtime._query_mean_std(
                teacher, teacher_before, np=np, torch=torch
            )
            _activity_increment("teacher_query_count")
            if candidate_std.tobytes() != teacher_std.tobytes():
                raise V12R4ExecutionError("candidate/teacher logstd mismatch")
            noise = np.ascontiguousarray(
                candidate_std * innovations[index], dtype=np.float32
            )
            selected = safe_dagger.select_safe_dagger_action(
                candidate_mean,
                teacher_mean,
                noise,
                requested_alpha=contract.P2_STUDENT_WEIGHT,
                latch_state=latch,
                previous_penetration_m=previous_penetration_m,
                active_v26_phase=runtime._phase_state(current_info),
            )
            latch = selected.latch_state
            takeover_streak = takeover_streak + 1 if latch.active else 0
            max_takeover_streak = max(max_takeover_streak, takeover_streak)
            expected_mean, expected_alpha = safe_dagger.blend_policy_means(
                candidate_mean,
                teacher_mean,
                requested_alpha=contract.P2_STUDENT_WEIGHT,
                latch_state=latch,
            )
            expected_action = safe_dagger.apply_single_noise(expected_mean, noise)
            if (
                expected_alpha != selected.effective_alpha
                or expected_mean.tobytes() != selected.blended_mean.tobytes()
                or expected_action.tobytes() != selected.action.tobytes()
            ):
                raise V12R4ExecutionError("blend/latch/noise semantics drifted")
            applied = np.ascontiguousarray(
                np.clip(selected.action, env.action_space.low, env.action_space.high),
                dtype=np.float32,
            )
            _activity_increment("environment_step_calls")
            observation_after, reward, terminated, truncated, info = env.step(applied)
            observation_after = np.ascontiguousarray(
                observation_after, dtype=np.float32
            )
            if not isinstance(info, Mapping):
                raise V12R4ExecutionError("collection info is malformed")
            next_student = np.ascontiguousarray(
                observation_after[: contract.EXPECTED_ACTOR_FEATURES],
                dtype=np.float32,
            )
            next_teacher = coherent_teacher.build_teacher_view(
                next_student,
                actor_names,
                info,
                body_weight_n=body_weight_n,
                shadow=shadow,
                reset_boundary=False,
            )
            physical = runtime._consume_physical_step(
                audit,
                step=step,
                info=info,
                observation_before=observation_before,
                observation_after=observation_after,
                reward=reward,
                action=selected.action,
                applied_action=applied,
                extra_vectors=(
                    student_before,
                    teacher_before,
                    candidate_mean,
                    candidate_std,
                    teacher_mean,
                    teacher_std,
                    selected.blended_mean,
                    noise,
                ),
                legacy=legacy,
                v26_collector=v26_collector,
            )
            raw_samples = info.get("binary_phase_sensor_samples")
            if isinstance(raw_samples, Sequence) and not isinstance(
                raw_samples, (str, bytes)
            ):
                _activity_increment("raw_sensor_sample_count", len(raw_samples))
            row = {
                "schema_version": contract.SCHEMA_VERSION,
                "protocol_id": contract.PROTOCOL_ID,
                "stage_id": stage_id,
                "case_id": case_id,
                "v26_observation": student_before.tolist(),
                "counterfactual_teacher_observation": teacher_before.tolist(),
                "counterfactual_teacher_mean": teacher_mean.tolist(),
                "candidate_mean": candidate_mean.tolist(),
                "candidate_std": candidate_std.tolist(),
                "requested_alpha": contract.P2_STUDENT_WEIGHT,
                "effective_alpha": float(selected.effective_alpha),
                "blended_mean": selected.blended_mean.tolist(),
                "standard_normal": innovations[index].tolist(),
                "single_noise": noise.tolist(),
                "raw_action": selected.action.tolist(),
                "applied_action": applied.tolist(),
                "safety_latch_active": bool(latch.active),
                "safety_latch_entered": bool(selected.latch_entered),
                "safety_latch_released": bool(selected.latch_released),
                "forced_teacher_takeover": bool(selected.safety_intervened),
                "previous_penetration_m": previous_penetration_m,
                "reward": float(reward),
                "time_s": float(info.get("time")),
                "grf_penetration_m": physical["penetration_m"],
                "reserve_norm_nm": physical["reserve_norm_nm"],
                "residual_norm_nm": physical["residual_norm_nm"],
                "phase_fsm": legacy._jsonable(physical["phase"]),
                "observer_raw_sensor_journal": legacy._jsonable(
                    _diagnostic_raw_journal(info, step=step)
                ),
                "checks": physical["checks"],
                "terminated": bool(terminated),
                "truncated": bool(truncated),
                "end_reason": info.get("end_reason"),
            }
            writer.write_step(step, row)
            rows.append({"step": step, **row})
            label_observations.append(student_before)
            label_actions.append(teacher_mean)
            previous_penetration_m = physical["penetration_m"]
            observation = observation_after
            student = next_student
            teacher_view = next_teacher
            current_info = dict(info)
            if step == 1 or step % 25 == 0:
                elapsed = time.monotonic() - started
                eta = elapsed / step * (contract.EXPECTED_STEPS - step)
                print(
                    f"[V12R4 collect/{case_id}] {step:3d}/{contract.EXPECTED_STEPS} "
                    f"elapsed={elapsed:7.1f}s eta={eta:7.1f}s",
                    flush=True,
                )
            if terminated or truncated:
                break
    except BaseException as exc:
        try:
            writer.publish_failure(
                end_reason="v12r4_shielded_collection_failed_terminal_no_retry",
                error=exc,
                status=contract.TERMINAL_FAIL_STATUS,
            )
        except Exception:
            pass
        raise
    finally:
        env.close()
    if audit is None:
        raise V12R4ExecutionError("collection audit was not initialized")
    sample_count = len(rows)
    labels_path = destination / "labels.npz"
    v10s_fit._write_npz_exclusive(
        labels_path,
        {
            "observations": np.ascontiguousarray(
                np.asarray(label_observations), dtype=np.float32
            ),
            "actions": np.ascontiguousarray(
                np.asarray(label_actions), dtype=np.float32
            ),
            "reset_mask": np.asarray(
                [index == 0 for index in range(sample_count)], dtype=np.bool_
            ),
            "actor_feature_names": np.asarray(actor_names, dtype="U64"),
            "case_ids": np.asarray([case_id] * sample_count, dtype="U96"),
            "step_indices": np.arange(1, sample_count + 1, dtype=np.int64),
            "tranche_ids": np.asarray(
                ["v12r4_p3_coverage"] * sample_count, dtype="U96"
            ),
            "origins": np.asarray(
                ["V12R4_SHIELDED_SAME_STATE_TEACHER_LABEL"] * sample_count,
                dtype="U160",
            ),
        },
    )
    partial = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": "PERSISTED_H0_V12R4_COLLECTION_BEFORE_GATE",
        "protocol_id": contract.PROTOCOL_ID,
        "stage_id": stage_id,
        "steps": sample_count,
        "gate_evaluated": False,
    }
    summary = {
        **runtime._physical_summary(
            audit,
            case=case,
            rows=rows,
            info=info,
            terminated=terminated,
            truncated=truncated,
            actor_names=actor_names,
            full_names=full_names,
            legacy=legacy,
            v26_collector=v26_collector,
        ),
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.COLLECTION_COMPLETE_STATUS,
        "protocol_id": contract.PROTOCOL_ID,
        "case": copy.deepcopy(case),
        "candidate_role": "P2_COLLECTION_STUDENT_ONLY_NONPROMOTABLE",
        "candidate_module": copy.deepcopy(contract.P2_MODULE_TREE),
        "candidate_promoted": False,
        "requested_alpha": contract.P2_STUDENT_WEIGHT,
        "sample_count": sample_count,
        "teacher_query_count": sample_count,
        "same_state_teacher_label_count": sample_count,
        "candidate_mean_query_count": sample_count,
        "mean_blend_count": sample_count,
        "alpha_mismatch_count": 0,
        "mean_blend_mismatch_count": 0,
        "noise_application_mismatch_count": 0,
        "blend_before_noise_count": sample_count,
        "noise_before_blend_count": 0,
        "safety_latch_rule_violation_count": 0,
        "safety_latch_activation_m": contract.SAFETY_LATCH_ACTIVATION_M,
        "safety_latch_release_m": contract.SAFETY_LATCH_RELEASE_M,
        "safety_latch_release_phase": contract.SAFETY_LATCH_RELEASE_PHASE,
        "safety_signal_lag_steps": 1,
        "safety_latch_activation_count": latch.activation_count,
        "safety_latch_release_count": latch.release_count,
        "safety_intervention_count": latch.intervention_action_count,
        "max_consecutive_takeover_steps": max_takeover_streak,
        "latch_active_at_episode_end": bool(latch.active),
        "event_contract_id": contract.EVENT_CONTRACT_ID,
        "target_contract_id": contract.TARGET_CONTRACT_ID,
        "detector_or_fsm_modified": False,
        "morphology_weight": contract.MORPHOLOGY_WEIGHT,
        "label_corpus": _record(labels_path),
        "q2_paths_opened": [],
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
    }
    writer.finalize_before_gate(trace=rows, partial_summary=partial, summary=summary)
    gate = contract.collection_gate(summary, case_id=case_id)
    writer.publish_gate(gate)
    if gate.get("passed") is not True:
        writer.publish_failure(
            end_reason="v12r4_collection_gate_failed_terminal_no_retry",
            error="V12R4 collection gate failed",
            status=contract.TERMINAL_FAIL_STATUS,
        )
        raise V12R4ExecutionError(f"collection gate failed: {case_id}")
    receipt = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.COLLECTION_PASS_STATUS,
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "stage_id": stage_id,
        "case_id": case_id,
        "sample_count": sample_count,
        "label_corpus": _record(labels_path),
        "summary": _record(destination / "summary.json"),
        "gate": _record(destination / "gate.json"),
        "trace": _record(destination / "trace.json"),
        "pipeline_claim": _record(PIPELINE_CLAIM_PATH),
        "worker_claim": _record(_claim_path(stage_id)),
    }
    forensic.write_json_exclusive(destination / "receipt.json", receipt)
    return receipt


def _run_corpus_assembly() -> dict[str, Any]:
    stage_id = "assemble_corpus_p3"
    corpus = fit_engine.load_p3_corpus(_record(PIPELINE_CLAIM_PATH))
    payload = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": "PASS_H0_V12R4_P3_CORPUS_ASSEMBLY",
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "stage_id": stage_id,
        "fit_counts": contract.expected_corpus_counts(),
        "corpus_audit": copy.deepcopy(corpus.audit),
        "source_records": copy.deepcopy(corpus.source_records),
        "pipeline_claim": _record(PIPELINE_CLAIM_PATH),
        "worker_claim": _record(_claim_path(stage_id)),
        "q2_paths_opened": [],
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
    }
    forensic.write_json_exclusive(_stage_receipt_path(stage_id), payload)
    return payload


def _run_fit() -> dict[str, Any]:
    _activity_increment("actor_fit_stage_calls_attempted")
    _activity_increment("actor_updates_attempted")
    receipt = fit_engine.run_fit_stage(
        pipeline_claim_path=PIPELINE_CLAIM_PATH,
        worker_claim_path=_claim_path("fit_p3"),
        protocol_freeze_path=PROTOCOL_FREEZE_PATH,
        execution_lock_path=LOCK_PATH,
        activity_callback=_activity_increment,
    )
    _activity_increment("actor_fit_executions_confirmed")
    _activity_increment("actor_updates_confirmed")
    return receipt


def _run_candidate_freeze() -> dict[str, Any]:
    stage_id = "freeze_candidate_p3"
    fit_receipt = fit_engine.verify_fit_stage()
    fit_summary = _mapping(resolve_relative(contract.FIT_ROOT) / "summary.json")
    module = _tree(resolve_relative(contract.P3_MODULE_PATH))
    identity = contract.candidate_id(module["tree_sha256"])
    if (
        fit_receipt.get("candidate_id") != identity
        or fit_receipt.get("candidate_module") != module
        or fit_summary.get("candidate_id") != identity
        or fit_summary.get("candidate_module") != module
    ):
        raise V12R4ExecutionError("fit candidate identity drifted before freeze")
    summary_path = RUN_ROOT / "candidate_freeze_summary.json"
    gate_path = RUN_ROOT / "candidate_freeze_gate.json"
    summary = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.CANDIDATE_FREEZE_COMPLETE_STATUS,
        "protocol_id": contract.PROTOCOL_ID,
        "candidate_selection_rule": contract.CANDIDATE_SELECTION_RULE,
        "candidate_id": identity,
        "candidate_module": module,
        "fit_receipt": _record(resolve_relative(contract.FIT_ROOT) / "receipt.json"),
        "fit_passed": True,
        "candidate_frozen": True,
        "source_h0_byte_exact": fit_summary.get("source_h0_byte_exact") is True,
        "logstd_byte_exact": fit_summary.get("logstd_byte_exact") is True,
        "critic_present": fit_summary.get("critic_present"),
        "save_reload_exact": fit_summary.get("save_reload_exact") is True,
        "pipeline_claim": _record(PIPELINE_CLAIM_PATH),
        "worker_claim": _record(_claim_path(stage_id)),
        "q2_paths_opened": [],
        "runtime_promoted": False,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
    }
    forensic.write_json_exclusive(summary_path, summary)
    gate = contract.candidate_freeze_gate(summary)
    forensic.write_json_exclusive(gate_path, gate)
    if gate.get("passed") is not True:
        raise V12R4ExecutionError("candidate freeze gate failed")
    receipt = {
        **summary,
        "status": contract.CANDIDATE_FREEZE_STATUS,
        "passed": True,
        "stage_id": stage_id,
        "summary": _record(summary_path),
        "gate": _record(gate_path),
    }
    forensic.write_json_exclusive(CANDIDATE_FREEZE_PATH, receipt)
    return receipt


def _run_development(case_id: str) -> dict[str, Any]:
    (
        rollout_eval,
        np,
        torch,
        RLModule,
        env_factory,
        legacy,
        v26_collector,
    ) = _rollout_stack()
    stage_id = f"development__{case_id}"
    case = contract.canonical_development_case(case_id)
    destination = resolve_relative(case["destination"])
    if os.path.lexists(destination):
        raise V12R4ExecutionError(f"development destination exists: {case_id}")
    freeze = _mapping(CANDIDATE_FREEZE_PATH)
    module_path = resolve_relative(contract.P3_MODULE_PATH).resolve()
    module_record = _tree(module_path)
    candidate_identity = contract.candidate_id(module_record["tree_sha256"])
    if (
        freeze.get("status") != contract.CANDIDATE_FREEZE_STATUS
        or freeze.get("passed") is not True
        or freeze.get("candidate_id") != candidate_identity
        or freeze.get("candidate_module") != module_record
    ):
        raise V12R4ExecutionError("candidate freeze identity drifted")
    candidate = RLModule.from_checkpoint(module_path.resolve())
    candidate.eval()
    innovations = runtime._frozen_innovations(
        case_id, action_selection=str(case["action_selection"]), np=np
    )
    env = env_factory.make_cmc_env(env_source.build_env_config(case))
    writer = forensic.ForensicRolloutWriter(destination, artifact_root=REPO_ROOT)
    writer.start(
        {
            "schema_version": contract.SCHEMA_VERSION,
            "status": "STARTED_H0_V12R4_P3_DEVELOPMENT",
            "protocol_id": contract.PROTOCOL_ID,
            "stage_id": stage_id,
            "case": copy.deepcopy(case),
            "candidate_id": candidate_identity,
            "candidate_module": module_record,
            "candidate_freeze": _record(CANDIDATE_FREEZE_PATH),
            "teacher_enabled": False,
            "blending_enabled": False,
            "safety_latch_enabled": False,
            "pipeline_claim": _record(PIPELINE_CLAIM_PATH),
            "worker_claim": _record(_claim_path(stage_id)),
        }
    )
    rows: list[dict[str, Any]] = []
    actor_names: tuple[str, ...] = ()
    full_names: tuple[str, ...] = ()
    audit: dict[str, Any] | None = None
    info: Mapping[str, Any] = {}
    terminated = False
    truncated = False
    started = time.monotonic()
    try:
        _activity_increment("environment_reset_calls")
        observation, reset_info = env.reset(seed=int(case["runtime_seed"]))
        observation = np.ascontiguousarray(observation, dtype=np.float32)
        actor_names, full_names = runtime._validate_runtime_layout(
            module=candidate,
            env=env,
            observation=observation,
            rollout_eval=rollout_eval,
            np=np,
        )
        audit = runtime._new_physical_audit(reset_info=reset_info, legacy=legacy, np=np)
        for index in range(contract.EXPECTED_STEPS):
            step = index + 1
            observation_before = observation.copy()
            actor = np.ascontiguousarray(
                observation_before[: contract.EXPECTED_ACTOR_FEATURES],
                dtype=np.float32,
            )
            mean, std = runtime._query_mean_std(candidate, actor, np=np, torch=torch)
            noise = np.ascontiguousarray(std * innovations[index], dtype=np.float32)
            raw_action = safe_dagger.apply_single_noise(mean, noise)
            applied = np.ascontiguousarray(
                np.clip(raw_action, env.action_space.low, env.action_space.high),
                dtype=np.float32,
            )
            _activity_increment("environment_step_calls")
            observation_after, reward, terminated, truncated, info = env.step(applied)
            observation_after = np.ascontiguousarray(
                observation_after, dtype=np.float32
            )
            if not isinstance(info, Mapping):
                raise V12R4ExecutionError("development info is malformed")
            physical = runtime._consume_physical_step(
                audit,
                step=step,
                info=info,
                observation_before=observation_before,
                observation_after=observation_after,
                reward=reward,
                action=raw_action,
                applied_action=applied,
                extra_vectors=(actor, mean, std, noise),
                legacy=legacy,
                v26_collector=v26_collector,
            )
            raw_samples = info.get("binary_phase_sensor_samples")
            if isinstance(raw_samples, Sequence) and not isinstance(
                raw_samples, (str, bytes)
            ):
                _activity_increment("raw_sensor_sample_count", len(raw_samples))
            row = {
                "schema_version": contract.SCHEMA_VERSION,
                "protocol_id": contract.PROTOCOL_ID,
                "stage_id": stage_id,
                "case_id": case_id,
                "v26_observation": actor.tolist(),
                "candidate_mean": mean.tolist(),
                "candidate_std": std.tolist(),
                "standard_normal": innovations[index].tolist(),
                "single_noise": noise.tolist(),
                "raw_action": raw_action.tolist(),
                "applied_action": applied.tolist(),
                "teacher_enabled": False,
                "blending_enabled": False,
                "safety_latch_enabled": False,
                "reward": float(reward),
                "time_s": float(info.get("time")),
                "grf_penetration_m": physical["penetration_m"],
                "reserve_norm_nm": physical["reserve_norm_nm"],
                "residual_norm_nm": physical["residual_norm_nm"],
                "phase_fsm": legacy._jsonable(physical["phase"]),
                "observer_raw_sensor_journal": legacy._jsonable(
                    _diagnostic_raw_journal(info, step=step)
                ),
                "checks": physical["checks"],
                "terminated": bool(terminated),
                "truncated": bool(truncated),
                "end_reason": info.get("end_reason"),
            }
            writer.write_step(step, row)
            rows.append({"step": step, **row})
            observation = observation_after
            if step == 1 or step % 25 == 0:
                elapsed = time.monotonic() - started
                eta = elapsed / step * (contract.EXPECTED_STEPS - step)
                print(
                    f"[V12R4 development/{case_id}] {step:3d}/"
                    f"{contract.EXPECTED_STEPS} elapsed={elapsed:7.1f}s "
                    f"eta={eta:7.1f}s",
                    flush=True,
                )
            if terminated or truncated:
                break
    except BaseException as exc:
        try:
            writer.publish_failure(
                end_reason="v12r4_development_failed_terminal_no_retry",
                error=exc,
                status=contract.TERMINAL_FAIL_STATUS,
            )
        except Exception:
            pass
        raise
    finally:
        env.close()
    if audit is None:
        raise V12R4ExecutionError("development audit was not initialized")
    partial = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": "PERSISTED_H0_V12R4_DEVELOPMENT_BEFORE_GATE",
        "protocol_id": contract.PROTOCOL_ID,
        "stage_id": stage_id,
        "steps": len(rows),
        "gate_evaluated": False,
    }
    summary = {
        **runtime._physical_summary(
            audit,
            case=case,
            rows=rows,
            info=info,
            terminated=terminated,
            truncated=truncated,
            actor_names=actor_names,
            full_names=full_names,
            legacy=legacy,
            v26_collector=v26_collector,
        ),
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.DEVELOPMENT_COMPLETE_STATUS,
        "protocol_id": contract.PROTOCOL_ID,
        "case": copy.deepcopy(case),
        "candidate_id": candidate_identity,
        "candidate_module": module_record,
        "candidate_freeze": _record(CANDIDATE_FREEZE_PATH),
        "teacher_enabled": False,
        "blending_enabled": False,
        "safety_latch_enabled": False,
        "sea_reserve_gate_passed": audit["sea_plugin_fallback_count"] == 0
        and audit["so_solver_unaccepted_count"] == 0
        and audit["nonfinite_count"] == 0,
        "target_contract_id": contract.TARGET_CONTRACT_ID,
        "detector_or_fsm_modified": False,
        "q2_paths_opened": [],
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
    }
    writer.finalize_before_gate(trace=rows, partial_summary=partial, summary=summary)
    gate = contract.development_gate(summary, case_id=case_id)
    writer.publish_gate(gate)
    if gate.get("passed") is not True:
        writer.publish_failure(
            end_reason="v12r4_development_gate_failed_terminal_no_retry",
            error="V12R4 development gate failed",
            status=contract.TERMINAL_FAIL_STATUS,
        )
        raise V12R4ExecutionError(f"development gate failed: {case_id}")
    receipt = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.DEVELOPMENT_PASS_STATUS,
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "stage_id": stage_id,
        "case_id": case_id,
        "candidate_id": candidate_identity,
        "candidate_module": module_record,
        "candidate_freeze": _record(CANDIDATE_FREEZE_PATH),
        "summary": _record(destination / "summary.json"),
        "gate": _record(destination / "gate.json"),
        "trace": _record(destination / "trace.json"),
        "pipeline_claim": _record(PIPELINE_CLAIM_PATH),
        "worker_claim": _record(_claim_path(stage_id)),
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
    }
    forensic.write_json_exclusive(destination / "receipt.json", receipt)
    return receipt


def _run_finalize_development() -> dict[str, Any]:
    stage_id = "finalize_development"
    freeze = _mapping(CANDIDATE_FREEZE_PATH)
    module = _tree(resolve_relative(contract.P3_MODULE_PATH))
    identity = contract.candidate_id(module["tree_sha256"])
    bindings: list[dict[str, Any]] = []
    for case_id in contract.DEVELOPMENT_CASE_IDS:
        root = resolve_relative(contract.DEVELOPMENT_PATHS[case_id])
        receipt = _mapping(root / "receipt.json")
        gate = _mapping(root / "gate.json")
        if (
            receipt.get("passed") is not True
            or receipt.get("candidate_id") != identity
            or receipt.get("candidate_module") != module
            or gate.get("passed") is not True
        ):
            raise V12R4ExecutionError(f"development binding drifted: {case_id}")
        bindings.append(
            {
                "case_id": case_id,
                "passed": True,
                "receipt": _record(root / "receipt.json"),
                "gate": _record(root / "gate.json"),
                "summary": _record(root / "summary.json"),
            }
        )
    if (
        freeze.get("candidate_id") != identity
        or freeze.get("candidate_module") != module
    ):
        raise V12R4ExecutionError("candidate identity drifted at finalization")
    expected_activity = {
        "environment_reset_calls": 10,
        "environment_step_calls": 5000,
        "raw_sensor_sample_count": 50_000,
        "teacher_query_count": 2000,
        "actor_fit_stage_calls_attempted": 1,
        "actor_fit_executions_confirmed": 1,
        "actor_updates_attempted": 1,
        "actor_updates_confirmed": 1,
        "adamw_epochs_completed": 3000,
    }
    observed_activity = {name: _ACTIVITY_TOTALS[name] for name in expected_activity}
    if (
        observed_activity != expected_activity
        or _ACTIVITY_TOTALS["lbfgs_closure_calls"] < 1
    ):
        raise V12R4ExecutionError(
            f"live pipeline activity drifted before finalization: {observed_activity}"
        )
    summary_path = RUN_ROOT / "final_development_summary.json"
    gate_path = RUN_ROOT / "final_development_gate.json"
    summary = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": "COMPLETE_H0_V12R4_P3_DEVELOPMENT_AGGREGATE",
        "protocol_id": contract.PROTOCOL_ID,
        "candidate_selection_rule": contract.CANDIDATE_SELECTION_RULE,
        "candidate_id": identity,
        "candidate_module": module,
        "candidate_freeze": _record(CANDIDATE_FREEZE_PATH),
        "case_gates": [
            {"case_id": item["case_id"], "passed": item["passed"]} for item in bindings
        ],
        "rollout_bindings": bindings,
        "candidate_tree_unique_count": 1,
        "collection_count": 4,
        "development_count": 6,
        "environment_reset_calls": _ACTIVITY_TOTALS["environment_reset_calls"],
        "environment_step_calls": _ACTIVITY_TOTALS["environment_step_calls"],
        "raw_sensor_sample_count": _ACTIVITY_TOTALS["raw_sensor_sample_count"],
        "teacher_query_count": _ACTIVITY_TOTALS["teacher_query_count"],
        "actor_updates": _ACTIVITY_TOTALS["actor_updates_confirmed"],
        "critic_updates": 0,
        "ppo_updates": 0,
        "retry_authorized": False,
        "resume_authorized": False,
        "rescue_authorized": False,
        "q2_paths_opened": [],
        "runtime_promoted": False,
        "checkpoint_zero_created": False,
        "positive_morphology_enabled": False,
        "pipeline_claim": _record(PIPELINE_CLAIM_PATH),
        "worker_claim": _record(_claim_path(stage_id)),
    }
    forensic.write_json_exclusive(summary_path, summary)
    gate = contract.aggregate_development_gate(summary)
    forensic.write_json_exclusive(gate_path, gate)
    if gate.get("passed") is not True:
        raise V12R4ExecutionError("final development aggregate gate failed")
    receipt = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.FINAL_DEVELOPMENT_PASS_STATUS,
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "stage_id": stage_id,
        "candidate_selection_rule": contract.CANDIDATE_SELECTION_RULE,
        "candidate_id": identity,
        "candidate_module": module,
        "candidate_freeze": _record(CANDIDATE_FREEZE_PATH),
        "summary": _record(summary_path),
        "gate": _record(gate_path),
        "rollout_bindings": bindings,
        "pipeline_claim": _record(PIPELINE_CLAIM_PATH),
        "worker_claim": _record(_claim_path(stage_id)),
        "development_only": True,
        "qualification_executed": False,
        "runtime_promoted": False,
        "checkpoint_zero_created": False,
        "positive_morphology_enabled": False,
        "retry_authorized": False,
        "resume_authorized": False,
        "rescue_authorized": False,
        "sweep_authorized": False,
        "actor_updates": 1,
        "critic_updates": 0,
        "ppo_updates": 0,
    }
    forensic.write_json_exclusive(
        resolve_relative(contract.FINAL_DEVELOPMENT_RECEIPT_PATH), receipt
    )
    return receipt


def _run_stage(stage_id: str) -> dict[str, Any]:
    descriptor = contract.stage_descriptor(stage_id)
    kind = descriptor["kind"]
    if kind == "attestation":
        return _run_attestation()
    if kind == "collection":
        return _run_collection(str(descriptor["case"]["case_id"]))
    if kind == "corpus":
        return _run_corpus_assembly()
    if kind == "fit":
        return _run_fit()
    if kind == "candidate_freeze":
        return _run_candidate_freeze()
    if kind == "development":
        return _run_development(str(descriptor["case"]["case_id"]))
    if kind == "finalize":
        return _run_finalize_development()
    raise V12R4ExecutionError(f"unknown stage kind: {kind!r}")


def _terminal_ledger(
    *,
    passed: bool,
    attempted_stage: str | None,
    completed: Sequence[Mapping[str, Any]],
    error: BaseException | None,
) -> dict[str, Any]:
    candidate_id: str | None = None
    candidate_module: Any = None
    candidate_freeze: Any = None
    final_receipt: Any = None
    if CANDIDATE_FREEZE_PATH.is_file():
        freeze = _mapping(CANDIDATE_FREEZE_PATH)
        candidate_id = freeze.get("candidate_id")
        candidate_module = freeze.get("candidate_module")
        candidate_freeze = _record(CANDIDATE_FREEZE_PATH)
    final_path = resolve_relative(contract.FINAL_DEVELOPMENT_RECEIPT_PATH)
    if final_path.is_file():
        final_receipt = _record(final_path)
    failure_artifact = None
    if attempted_stage is not None:
        descriptor = contract.stage_descriptor(attempted_stage)
        if descriptor["kind"] in {"collection", "development"}:
            failure_path = (
                resolve_relative(descriptor["case"]["destination"]) / "failure.json"
            )
            if failure_path.is_file():
                failure_artifact = _record(failure_path)
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": (
            contract.PIPELINE_PASS_STATUS if passed else contract.TERMINAL_FAIL_STATUS
        ),
        "passed": passed,
        "terminal": True,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "attempted_stage": attempted_stage,
        "completed_stages": [dict(row) for row in completed],
        "completed_stage_count": len(completed),
        "stage_order": list(contract.STAGE_IDS),
        "candidate_selection_rule": contract.CANDIDATE_SELECTION_RULE,
        "candidate_id": candidate_id,
        "candidate_module": candidate_module,
        "candidate_freeze": candidate_freeze,
        "final_development_receipt": final_receipt,
        "protocol_freeze": _record(PROTOCOL_FREEZE_PATH),
        "execution_lock": _record(LOCK_PATH),
        "pipeline_claim": (
            _record(PIPELINE_CLAIM_PATH) if PIPELINE_CLAIM_PATH.is_file() else None
        ),
        "q2_paths_opened": (
            []
            if _q2_unopened()
            else [
                name
                for name, path in contract.Q2_UNOPENED_PATHS.items()
                if os.path.lexists(resolve_relative(path))
            ]
        ),
        "error": (
            None
            if error is None
            else {"type": type(error).__name__, "message": str(error)}
        ),
        "activity_totals": copy.deepcopy(_ACTIVITY_TOTALS),
        "stage_activity": [
            copy.deepcopy(_STAGE_ACTIVITY[stage_id])
            for stage_id in contract.STAGE_IDS
            if stage_id in _STAGE_ACTIVITY
        ],
        "attempted_stage_worker_claim": (
            _record(_claim_path(attempted_stage))
            if attempted_stage is not None and _claim_path(attempted_stage).is_file()
            else None
        ),
        "attempted_stage_failure_artifact": failure_artifact,
        "collection_count": sum(
            contract.stage_descriptor(row["stage_id"])["kind"] == "collection"
            for row in completed
        ),
        "development_count": sum(
            contract.stage_descriptor(row["stage_id"])["kind"] == "development"
            for row in completed
        ),
        "environment_reset_calls": _ACTIVITY_TOTALS["environment_reset_calls"],
        "environment_step_calls": _ACTIVITY_TOTALS["environment_step_calls"],
        "raw_sensor_sample_count": _ACTIVITY_TOTALS["raw_sensor_sample_count"],
        "teacher_query_count": _ACTIVITY_TOTALS["teacher_query_count"],
        "actor_fit_stage_calls_attempted": _ACTIVITY_TOTALS[
            "actor_fit_stage_calls_attempted"
        ],
        "actor_fit_executions_confirmed": _ACTIVITY_TOTALS[
            "actor_fit_executions_confirmed"
        ],
        "actor_updates_attempted": _ACTIVITY_TOTALS["actor_updates_attempted"],
        "actor_updates": _ACTIVITY_TOTALS["actor_updates_confirmed"],
        "adamw_epochs_completed": _ACTIVITY_TOTALS["adamw_epochs_completed"],
        "lbfgs_closure_calls": _ACTIVITY_TOTALS["lbfgs_closure_calls"],
        "critic_updates": 0,
        "ppo_updates": 0,
        "retry_authorized": False,
        "resume_authorized": False,
        "rescue_authorized": False,
        "sweep_authorized": False,
        "qualification_executed": False,
        "runtime_promoted": False,
        "checkpoint_zero_created": False,
        "positive_morphology_enabled": False,
        "next_stage": "WAIT_SEPARATE_Q2_PROTOCOL" if passed else "STOP_TERMINAL",
    }


def execute_pipeline_once() -> dict[str, Any]:
    _reset_activity()
    verify_execution_lock(require_run_root_absent=True)
    token_hash, _claim = _claim_run_root()
    completed: list[dict[str, Any]] = []
    attempted: str | None = None
    try:
        for stage_id in contract.STAGE_IDS:
            attempted = stage_id
            _write_worker_claim(stage_id, token_hash)
            _begin_stage_activity(stage_id)
            receipt = _run_stage(stage_id)
            completed.append(
                {
                    "stage_id": stage_id,
                    "receipt": _record(_stage_receipt_path(stage_id)),
                }
            )
            if receipt.get("passed") is not True:
                raise V12R4ExecutionError(f"stage returned non-PASS: {stage_id}")
        if not _q2_unopened():
            raise V12R4ExecutionError("Q2 output opened during R4")
        ledger = _terminal_ledger(
            passed=True,
            attempted_stage=attempted,
            completed=completed,
            error=None,
        )
        forensic.write_json_exclusive(PIPELINE_LEDGER_PATH, ledger)
        return verify_terminal_ledger()
    except BaseException as exc:
        if not os.path.lexists(PIPELINE_LEDGER_PATH):
            ledger = _terminal_ledger(
                passed=False,
                attempted_stage=attempted,
                completed=completed,
                error=exc,
            )
            forensic.write_json_exclusive(PIPELINE_LEDGER_PATH, ledger)
            verify_terminal_ledger()
        raise


def verify_terminal_ledger() -> dict[str, Any]:
    """Recompute current artifact bindings and live-counter closure."""

    ledger = _mapping(PIPELINE_LEDGER_PATH)
    passed = ledger.get("passed") is True
    expected_status = (
        contract.PIPELINE_PASS_STATUS if passed else contract.TERMINAL_FAIL_STATUS
    )
    completed = ledger.get("completed_stages")
    stage_activity = ledger.get("stage_activity")
    totals = ledger.get("activity_totals")
    if (
        ledger.get("schema_version") != contract.SCHEMA_VERSION
        or ledger.get("status") != expected_status
        or ledger.get("terminal") is not True
        or ledger.get("protocol_id") != contract.PROTOCOL_ID
        or ledger.get("pipeline_id") != contract.PIPELINE_ID
        or ledger.get("stage_order") != list(contract.STAGE_IDS)
        or ledger.get("candidate_selection_rule") != contract.CANDIDATE_SELECTION_RULE
        or not isinstance(completed, list)
        or ledger.get("completed_stage_count") != len(completed)
        or not isinstance(stage_activity, list)
        or not isinstance(totals, Mapping)
        or set(totals) != set(_ACTIVITY_NAMES)
    ):
        raise V12R4ExecutionError("terminal ledger schema/identity drifted")
    expected_completed: list[dict[str, Any]] = []
    for stage_id in contract.STAGE_IDS[: len(completed)]:
        receipt_path = _stage_receipt_path(stage_id)
        if not receipt_path.is_file():
            raise V12R4ExecutionError(f"completed receipt missing: {stage_id}")
        expected_completed.append(
            {"stage_id": stage_id, "receipt": _record(receipt_path)}
        )
    if completed != expected_completed:
        raise V12R4ExecutionError("terminal ledger completed-stage closure drifted")
    activity_ids = [
        row.get("stage_id") for row in stage_activity if isinstance(row, Mapping)
    ]
    if (
        len(activity_ids) != len(stage_activity)
        or activity_ids != list(contract.STAGE_IDS[: len(activity_ids)])
        or any(
            set(row) != {"stage_id", "stage_kind", *_ACTIVITY_NAMES}
            for row in stage_activity
        )
        or any(
            row.get("stage_kind")
            != contract.stage_descriptor(str(row.get("stage_id")))["kind"]
            for row in stage_activity
        )
    ):
        raise V12R4ExecutionError("terminal ledger stage activity drifted")
    recomputed_totals = {
        name: sum(int(row[name]) for row in stage_activity) for name in _ACTIVITY_NAMES
    }
    top_level_counters = {
        "environment_reset_calls": ledger.get("environment_reset_calls"),
        "environment_step_calls": ledger.get("environment_step_calls"),
        "raw_sensor_sample_count": ledger.get("raw_sensor_sample_count"),
        "teacher_query_count": ledger.get("teacher_query_count"),
        "actor_fit_stage_calls_attempted": ledger.get(
            "actor_fit_stage_calls_attempted"
        ),
        "actor_fit_executions_confirmed": ledger.get("actor_fit_executions_confirmed"),
        "actor_updates_attempted": ledger.get("actor_updates_attempted"),
        "actor_updates_confirmed": ledger.get("actor_updates"),
        "adamw_epochs_completed": ledger.get("adamw_epochs_completed"),
        "lbfgs_closure_calls": ledger.get("lbfgs_closure_calls"),
    }
    if dict(totals) != recomputed_totals or top_level_counters != recomputed_totals:
        raise V12R4ExecutionError("terminal ledger live activity closure drifted")
    attempted = ledger.get("attempted_stage")
    expected_worker = (
        _record(_claim_path(attempted))
        if isinstance(attempted, str) and _claim_path(attempted).is_file()
        else None
    )
    if ledger.get("attempted_stage_worker_claim") != expected_worker:
        raise V12R4ExecutionError("terminal ledger attempted worker binding drifted")
    if passed:
        module = _tree(resolve_relative(contract.P3_MODULE_PATH))
        identity = contract.candidate_id(module["tree_sha256"])
        freeze = _mapping(CANDIDATE_FREEZE_PATH)
        final_path = resolve_relative(contract.FINAL_DEVELOPMENT_RECEIPT_PATH)
        final = _mapping(final_path)
        pass_checks = {
            "all_stages": len(completed) == len(contract.STAGE_IDS),
            "attempted_final": attempted == contract.STAGE_IDS[-1],
            "freeze_status": freeze.get("status") == contract.CANDIDATE_FREEZE_STATUS
            and freeze.get("passed") is True,
            "final_status": final.get("status")
            == contract.FINAL_DEVELOPMENT_PASS_STATUS
            and final.get("passed") is True,
            "candidate_identity": ledger.get("candidate_id") == identity
            and freeze.get("candidate_id") == identity
            and final.get("candidate_id") == identity,
            "candidate_tree": ledger.get("candidate_module") == module
            and freeze.get("candidate_module") == module
            and final.get("candidate_module") == module,
            "candidate_freeze_record": ledger.get("candidate_freeze")
            == _record(CANDIDATE_FREEZE_PATH),
            "final_record": ledger.get("final_development_receipt")
            == _record(final_path),
            "activity": recomputed_totals["environment_reset_calls"] == 10
            and recomputed_totals["environment_step_calls"] == 5000
            and recomputed_totals["raw_sensor_sample_count"] == 50_000
            and recomputed_totals["teacher_query_count"] == 2000
            and recomputed_totals["actor_fit_stage_calls_attempted"] == 1
            and recomputed_totals["actor_fit_executions_confirmed"] == 1
            and recomputed_totals["actor_updates_attempted"] == 1
            and recomputed_totals["actor_updates_confirmed"] == 1
            and recomputed_totals["adamw_epochs_completed"] == 3000
            and recomputed_totals["lbfgs_closure_calls"] >= 1,
            "q2_unopened_at_terminal": ledger.get("q2_paths_opened") == [],
        }
        if not all(pass_checks.values()):
            failed = [name for name, value in pass_checks.items() if not value]
            raise V12R4ExecutionError(f"terminal PASS ledger drifted: {failed}")
    return ledger


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--prepare-lock", action="store_true")
    group.add_argument("--verify-lock", action="store_true")
    group.add_argument("--execute", action="store_true")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    if args.prepare_lock:
        payload = prepare_execution_lock()
    elif args.verify_lock:
        payload = verify_execution_lock()
    else:
        payload = execute_pipeline_once()
    print(forensic.canonical_json_bytes(payload).decode("utf-8"))
    return 0 if payload.get("passed") is True else 1


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = [
    "V12R4ExecutionError",
    "build_execution_lock",
    "execute_pipeline_once",
    "main",
    "prepare_execution_lock",
    "resolve_relative",
    "verify_execution_lock",
    "verify_terminal_ledger",
]
