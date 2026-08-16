"""Freeze and execute the one-shot V9 hybrid-teacher closed-loop probe.

The environment always exposes the current V26 binary-active student view.
Only actor fields 10/11 are replaced from the same-state, force-free analog
shadow before querying the frozen H0 actor.  The deterministic H0 mean is then
served directly.  No actor, critic, PPO, morphology, protected, or reserve
operation is permitted by this development probe.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
import os
import secrets
import subprocess
import sys
import time
from pathlib import Path, PurePosixPath
from typing import Any, Mapping, Sequence


REPO_ROOT = Path(__file__).resolve().parents[1]
VALIDATION_ROOT = REPO_ROOT / "validation"
BASELINE_ROOT = REPO_ROOT / "Trajectory Generator" / "baseline_MLP"
for _root in (REPO_ROOT, VALIDATION_ROOT, BASELINE_ROOT):
    if str(_root) not in sys.path:
        sys.path.insert(0, str(_root))

import h0_primary_split_v9_hybrid_teacher_probe_contract as contract  # noqa: E402
import h0_v3_so_recovery_contract as so_recovery  # noqa: E402
import run_h0_primary_split_v9_causal_teacher as env_source  # noqa: E402
from h0_forensic_rollout import (  # noqa: E402
    ForensicRolloutWriter,
    artifact_record,
    canonical_json_bytes,
    sha256_file,
    strict_json_load,
    write_json_exclusive,
)
from h0_primary_split_v9_causal_teacher import (  # noqa: E402
    CAUSAL_INDICES,
    PRIVILEGED_INDICES,
    assert_causal_pair,
    from_current_shadow,
    validate_actor_feature_names,
)


class V9HybridTeacherProbeError(RuntimeError):
    """Raised on provenance, routing, persistence, or rollout failure."""


def resolve_relative(path: str | PurePosixPath) -> Path:
    raw = path.as_posix() if isinstance(path, PurePosixPath) else str(path)
    pure = PurePosixPath(raw)
    if not raw or pure.is_absolute() or ".." in pure.parts or pure.as_posix() != raw:
        raise V9HybridTeacherProbeError(f"non-canonical repository path: {raw!r}")
    return REPO_ROOT.joinpath(*pure.parts)


PREFLIGHT = resolve_relative(contract.PREFLIGHT_PATH)
LOCK = resolve_relative(contract.LOCK_PATH)
RUN_ROOT = resolve_relative(contract.RUN_ROOT)
ROLLOUT_ROOT = resolve_relative(contract.ROLLOUT_ROOT)
CLAIM = resolve_relative(contract.CLAIM_PATH)
WORKER_CLAIM = resolve_relative(contract.WORKER_CLAIM_PATH)
LEDGER = resolve_relative(contract.LEDGER_PATH)
SOURCE_H0 = resolve_relative(contract.SOURCE_H0_MODULE_PATH)
WORKER_TIMEOUT_S = 2400.0


def _mapping(path: str | Path) -> dict[str, Any]:
    value = strict_json_load(path)
    if not isinstance(value, Mapping):
        raise V9HybridTeacherProbeError(f"expected JSON object: {path}")
    return dict(value)


def _record(path: str | Path) -> dict[str, Any]:
    return artifact_record(path, artifact_root=REPO_ROOT)


def _tree_record(path: str | Path) -> dict[str, Any]:
    root = Path(path).expanduser().resolve()
    files = sorted(item for item in root.rglob("*") if item.is_file())
    if not files:
        raise V9HybridTeacherProbeError(f"empty artifact tree: {root}")
    digest = hashlib.sha256()
    rows: list[dict[str, Any]] = []
    for item in files:
        relative = item.relative_to(root).as_posix()
        sha = sha256_file(item)
        size = item.stat().st_size
        rows.append({"path": relative, "sha256": sha, "size_bytes": size})
        digest.update(relative.encode("utf-8"))
        digest.update(b"\0")
        digest.update(sha.encode("ascii"))
        digest.update(b"\0")
        digest.update(str(size).encode("ascii"))
        digest.update(b"\n")
    return {
        "path": root.relative_to(REPO_ROOT).as_posix(),
        "tree_sha256": digest.hexdigest(),
        "file_count": len(rows),
        "files": rows,
    }


def _source_records() -> dict[str, dict[str, Any]]:
    return {
        name: _record(resolve_relative(path))
        for name, path in contract.SOURCE_RELATIVE_PATHS.items()
    }


def _input_records() -> dict[str, dict[str, Any]]:
    return {
        name: _record(resolve_relative(path))
        for name, path in contract.INPUT_RELATIVE_PATHS.items()
    }


def _history_checks() -> dict[str, bool]:
    v9r1 = _mapping(resolve_relative(contract.INPUT_RELATIVE_PATHS["v9r1_terminal_ledger"]))
    causal = _mapping(
        resolve_relative(contract.INPUT_RELATIVE_PATHS["v9_causal_teacher_ledger"])
    )
    return {
        "v9r1_terminal_fail_preserved": v9r1.get("status")
        == "FAIL_H0_PRIMARY_SPLIT_V9R1_RESIDUAL_DAGGER_PIPELINE"
        and v9r1.get("passed") is False
        and v9r1.get("completed_stages") == ["fit_p0"]
        and v9r1.get("actor_updates") == 1
        and v9r1.get("critic_updates") == 0
        and v9r1.get("ppo_updates") == 0
        and v9r1.get("protected_trials_opened") == []
        and v9r1.get("retry_authorized") is False,
        "v9_causal_teacher_pass_preserved": causal.get("status")
        == "PASS_H0_PRIMARY_SPLIT_V9_CAUSAL_RELABEL"
        and causal.get("passed") is True
        and len(causal.get("completed_cases", [])) == 6
        and causal.get("actor_updates") == 0
        and causal.get("critic_updates") == 0
        and causal.get("ppo_updates") == 0
        and causal.get("protected_trials_opened") == [],
    }


def build_preflight(*, require_unoccupied: bool = True) -> dict[str, Any]:
    case = contract.canonical_case(contract.CASE_ID)
    config = env_source.build_env_config(case)
    occupancy = {
        "preflight_unoccupied": not os.path.lexists(PREFLIGHT),
        "lock_unoccupied": not os.path.lexists(LOCK),
        "run_root_unoccupied": not os.path.lexists(RUN_ROOT),
    }
    checks = {
        **_history_checks(),
        "single_case_exact": contract.CASE_IDS
        == ("deterministic_offset_minus_0p20",)
        and case["action_selection"] == "deterministic",
        "teacher_only_10_11": contract.PRIVILEGED_INDICES == (10, 11)
        and tuple(PRIVILEGED_INDICES) == (10, 11),
        "source_h0_exists": SOURCE_H0.is_dir(),
        "binary_active_v26": config.get("binary_phase_fsm_mode") == "binary_active"
        and config.get("binary_phase_event_contract_id")
        == contract.EVENT_CONTRACT_ID,
        "left_primary_only": config.get("online_grf_applied_sides") == ["left"],
        "morphology_zero": config.get("reward", {}).get("morphology_weight")
        == 0.0,
        "cadence_exact": config.get("detector_sample_dt_s")
        == contract.EXPECTED_SAMPLE_DT_S
        and config.get("segment_duration") == contract.EXPECTED_POLICY_DT_S,
        "zero_update_authority": not any(
            contract.AUTHORITY[name]
            for name in (
                "actor_updates_authorized",
                "critic_updates_authorized",
                "ppo_updates_authorized",
            )
        ),
        "protected_closed": not contract.AUTHORITY[
            "protected_trial_access_authorized"
        ],
        "reserve_closed": not contract.AUTHORITY["reserve_trial_access_authorized"],
        **occupancy,
    }
    passed = all(checks.values())
    if require_unoccupied and not passed:
        failed = [name for name, value in checks.items() if value is not True]
        raise V9HybridTeacherProbeError(f"hybrid-teacher preflight failed: {failed}")
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.PREFLIGHT_STATUS if passed else "FAIL_H0_V9_TEACHER_PROBE_PREFLIGHT",
        "passed": passed,
        "protocol_id": contract.PROTOCOL_ID,
        "probe_id": contract.PROBE_ID,
        "revision": contract.REVISION,
        "checks": checks,
        "case": case,
        "teacher": {
            "teacher_id": contract.TEACHER_ID,
            "construction": "from_current_shadow",
            "privileged_indices": list(contract.PRIVILEGED_INDICES),
            "served_action": "FROZEN_H0_DETERMINISTIC_MEAN",
        },
        "environment": {
            "binary_phase_fsm_mode": config["binary_phase_fsm_mode"],
            "binary_phase_event_contract_id": config[
                "binary_phase_event_contract_id"
            ],
            "online_grf_applied_sides": config["online_grf_applied_sides"],
            "morphology_weight": config["reward"]["morphology_weight"],
        },
        "source_h0": _tree_record(SOURCE_H0),
        "sources": _source_records(),
        "inputs": _input_records(),
        "authority": copy.deepcopy(contract.AUTHORITY),
        "environment_reset_calls": 0,
        "environment_step_calls": 0,
        "candidate_created": False,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
        "next_stage": "FREEZE_HYBRID_TEACHER_PROBE" if passed else "STOP",
    }


def _lock_payload(preflight: Mapping[str, Any]) -> dict[str, Any]:
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.LOCK_STATUS,
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "probe_id": contract.PROBE_ID,
        "revision": contract.REVISION,
        "preflight": _record(PREFLIGHT),
        "case": contract.canonical_case(contract.CASE_ID),
        "run_root": contract.RUN_ROOT.as_posix(),
        "rollout_root": contract.ROLLOUT_ROOT.as_posix(),
        "execution_claim": contract.CLAIM_PATH.as_posix(),
        "worker_claim": contract.WORKER_CLAIM_PATH.as_posix(),
        "execution_ledger": contract.LEDGER_PATH.as_posix(),
        "teacher_id": contract.TEACHER_ID,
        "teacher_builder": "from_current_shadow",
        "teacher_privileged_indices": list(contract.PRIVILEGED_INDICES),
        "served_action": "FROZEN_H0_DETERMINISTIC_MEAN",
        "event_contract_id": contract.EVENT_CONTRACT_ID,
        "source_h0": _tree_record(SOURCE_H0),
        "sources": _source_records(),
        "inputs": _input_records(),
        "gate": {
            "steps": contract.EXPECTED_STEPS,
            "control_windows": contract.EXPECTED_CONTROL_WINDOWS,
            "raw_sensor_samples": contract.EXPECTED_RAW_SENSOR_SAMPLES,
            "penetration_strictly_less_than_m": contract.PENETRATION_LIMIT_M,
            "minimum_valid_cycles": contract.MINIMUM_VALID_CYCLES,
        },
        "authority": copy.deepcopy(contract.AUTHORITY),
        "retry_authorized": False,
        "candidate_created": False,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
        "next_stage": "EXECUTE_HYBRID_TEACHER_PROBE_ONCE",
    }


def prepare() -> dict[str, Any]:
    preflight = build_preflight(require_unoccupied=True)
    write_json_exclusive(PREFLIGHT, preflight)
    lock = _lock_payload(preflight)
    write_json_exclusive(LOCK, lock)
    return {"preflight": preflight, "lock": lock}


def verify_lock(*, require_run_root_absent: bool = False) -> dict[str, Any]:
    preflight = _mapping(PREFLIGHT)
    if preflight.get("passed") is not True or preflight.get("status") != contract.PREFLIGHT_STATUS:
        raise V9HybridTeacherProbeError("preflight receipt is not PASS")
    expected = _lock_payload(preflight)
    observed = _mapping(LOCK)
    if canonical_json_bytes(observed) != canonical_json_bytes(expected):
        raise V9HybridTeacherProbeError("hybrid-teacher execution lock drifted")
    if require_run_root_absent and os.path.lexists(RUN_ROOT):
        raise V9HybridTeacherProbeError("hybrid-teacher run root already claimed")
    return observed


def _token_sha256(token: str) -> str:
    if not isinstance(token, str) or len(token) < 32:
        raise V9HybridTeacherProbeError("execution token is malformed")
    return hashlib.sha256(token.encode("utf-8")).hexdigest()


def _claim_payload(token_sha256: str) -> dict[str, Any]:
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.CLAIM_STATUS,
        "protocol_id": contract.PROTOCOL_ID,
        "probe_id": contract.PROBE_ID,
        "case_id": contract.CASE_ID,
        "execution_token_sha256": token_sha256,
        "execution_lock": _record(LOCK),
        "authority": copy.deepcopy(contract.AUTHORITY),
        "retry_authorized": False,
        "candidate_created": False,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
    }


def _worker_claim_payload(token_sha256: str) -> dict[str, Any]:
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.WORKER_CLAIM_STATUS,
        "protocol_id": contract.PROTOCOL_ID,
        "probe_id": contract.PROBE_ID,
        "case_id": contract.CASE_ID,
        "execution_token_sha256": token_sha256,
        "execution_claim": _record(CLAIM),
        "execution_lock": _record(LOCK),
        "retry_authorized": False,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
    }


def verify_worker_claim(token: str) -> dict[str, Any]:
    if os.path.lexists(LEDGER):
        raise V9HybridTeacherProbeError("terminal ledger exists; worker is closed")
    token_sha = _token_sha256(token)
    claim = _mapping(CLAIM)
    if canonical_json_bytes(claim) != canonical_json_bytes(_claim_payload(token_sha)):
        raise V9HybridTeacherProbeError("execution claim/token drifted")
    worker = _mapping(WORKER_CLAIM)
    if canonical_json_bytes(worker) != canonical_json_bytes(
        _worker_claim_payload(token_sha)
    ):
        raise V9HybridTeacherProbeError("worker claim/token drifted")
    return worker


def _query_h0_mean(module: Any, actor_view: Any, *, np: Any, torch: Any) -> tuple[Any, Any]:
    from ray.rllib.core.columns import Columns

    actor = np.ascontiguousarray(np.asarray(actor_view, dtype=np.float32))
    if actor.shape != (contract.EXPECTED_ACTOR_FEATURES,) or not np.all(
        np.isfinite(actor)
    ):
        raise V9HybridTeacherProbeError("teacher actor view is malformed")
    tensor = torch.as_tensor(actor[None, :], dtype=torch.float32)
    with torch.no_grad():
        logits = module._policy_logits({Columns.OBS: tensor}).detach().cpu().numpy()
    logits = np.ascontiguousarray(logits, dtype=np.float32)
    if logits.shape != (1, 4) or not np.all(np.isfinite(logits)):
        raise V9HybridTeacherProbeError("H0 teacher logits are malformed")
    mean = np.ascontiguousarray(logits[0, :2], dtype=np.float32)
    std = np.ascontiguousarray(np.exp(logits[0, 2:]), dtype=np.float32)
    if not np.all(np.isfinite(std)) or np.any(std <= 0.0):
        raise V9HybridTeacherProbeError("H0 teacher std is malformed")
    return mean, std


def _progress(completed: int, started: float) -> None:
    elapsed = max(0.0, time.monotonic() - started)
    eta = elapsed / completed * (contract.EXPECTED_STEPS - completed)
    width = 30
    filled = min(width, int(width * completed / contract.EXPECTED_STEPS))
    bar = "#" * filled + "-" * (width - filled)
    print(
        f"[V9 hybrid teacher probe] [{bar}] {completed:3d}/500 "
        f"elapsed={elapsed:7.1f}s eta={eta:7.1f}s",
        flush=True,
    )


def _worker_start_payload(lock: Mapping[str, Any]) -> dict[str, Any]:
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": "H0_PRIMARY_SPLIT_V9_HYBRID_TEACHER_PROBE_STARTED",
        "protocol_id": contract.PROTOCOL_ID,
        "probe_id": contract.PROBE_ID,
        "case": contract.canonical_case(contract.CASE_ID),
        "behavior": "H0_HYBRID_TEACHER_MEAN_CLOSED_LOOP_V26_BINARY_ACTIVE",
        "teacher_id": contract.TEACHER_ID,
        "teacher_builder": "from_current_shadow",
        "teacher_privileged_indices": list(contract.PRIVILEGED_INDICES),
        "served_action": "FROZEN_H0_DETERMINISTIC_MEAN",
        "source_h0": lock["source_h0"],
        "execution_lock": _record(LOCK),
        "execution_claim": _record(CLAIM),
        "worker_claim": _record(WORKER_CLAIM),
        "authority": copy.deepcopy(contract.AUTHORITY),
        "retry_authorized": False,
        "candidate_created": False,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
    }


def _execute_worker(*, writer: ForensicRolloutWriter) -> dict[str, Any]:
    rollout_eval, np, torch, RLModule, env_factory, _reward = (
        env_source.source_collector.engine.legacy._load_inference_stack()
    )
    legacy = env_source.source_collector.engine.legacy
    v26_collector = env_source.source_collector.base
    case = contract.canonical_case(contract.CASE_ID)
    runtime_seed = int(case["runtime_seed"])
    np.random.seed(runtime_seed)
    torch.manual_seed(runtime_seed)
    module = RLModule.from_checkpoint(SOURCE_H0)
    module.eval()
    env_config = env_source.build_env_config(case)
    env = env_factory.make_cmc_env(env_config)
    rows: list[dict[str, Any]] = []
    raw_sample_count = 0
    control_window_count = 0
    causal_mismatch_count = 0
    teacher_query_mismatch_count = 0
    clipped_values = 0
    nonfinite_count = 0
    timeout_count = 0
    invalid_event_count = 0
    hard_invalid_count = 0
    so_solver_unaccepted_count = 0
    sea_plugin_fallback_count = 0
    routing_failure_count = 0
    step_contract_failure_count = 0
    penetrations: list[float] = []
    phase_valid_cycles = 0
    terminated = False
    truncated = False
    info: Mapping[str, Any] = {}
    actor_names: tuple[str, ...] = ()
    full_names: tuple[str, ...] = ()
    binary_events: dict[str, Any] | None = None
    started = time.monotonic()
    try:
        observation, reset_info = env.reset(seed=runtime_seed)
        observation = np.ascontiguousarray(observation, dtype=np.float32)
        actor_names = tuple(str(name) for name in env.unwrapped.actor_feature_names)
        full_names = tuple(str(name) for name in env.unwrapped.observation_feature_names)
        validate_actor_feature_names(actor_names)
        rollout_eval._validate_module_observation_contract(
            module, actor_names, full_names
        )
        if (
            observation.shape != (contract.EXPECTED_FULL_FEATURES,)
            or observation.dtype != np.dtype(contract.EXPECTED_DTYPE)
            or actor_names != contract.EXPECTED_ACTOR_FEATURE_NAMES
            or full_names != contract.EXPECTED_OBSERVATION_FEATURE_NAMES
            or tuple(env.action_space.shape) != (contract.EXPECTED_ACTION_DIM,)
        ):
            raise V9HybridTeacherProbeError("runtime 35/84 float32 layout drifted")
        if (
            reset_info.get("binary_phase_fsm_executed") is not True
            or reset_info.get("binary_phase_fsm_mode") != "binary_active"
            or reset_info.get("binary_phase_event_contract_id")
            != contract.EVENT_CONTRACT_ID
            or reset_info.get("online_grf_applied_sides") != ["left"]
        ):
            raise V9HybridTeacherProbeError("V26 binary-active reset routing drifted")
        baseline_sensor = legacy._validate_raw_sample(
            reset_info.get("binary_phase_sensor_baseline"),
            float(reset_info.get("time")),
            "t0",
        )
        reset_fsm = reset_info.get("binary_phase_fsm")
        if not isinstance(reset_fsm, Mapping) or reset_fsm.get("events_this_step") != []:
            raise V9HybridTeacherProbeError("V26 attributed an event to t0")
        binary_events = legacy._binary_event_accumulator(baseline_sensor)
        body_weight_n = float(env.unwrapped._body_weight_n)
        if not math.isfinite(body_weight_n) or body_weight_n <= 0.0:
            raise V9HybridTeacherProbeError("body weight is malformed")
        current_info: Mapping[str, Any] = dict(reset_info)

        for index in range(contract.EXPECTED_STEPS):
            step = index + 1
            student = np.ascontiguousarray(
                observation[: contract.EXPECTED_ACTOR_FEATURES], dtype=np.float32
            )
            teacher_view = from_current_shadow(
                student,
                current_info,
                body_weight_n=body_weight_n,
                reset_boundary=index == 0,
            )
            assert_causal_pair(student, teacher_view)
            causal_exact = (
                student[list(CAUSAL_INDICES)].tobytes(order="C")
                == teacher_view[list(CAUSAL_INDICES)].tobytes(order="C")
            )
            causal_mismatch_count += int(not causal_exact)
            mean, std = _query_h0_mean(module, teacher_view, np=np, torch=torch)
            repeated_mean, repeated_std = _query_h0_mean(
                module, teacher_view, np=np, torch=torch
            )
            query_exact = mean.tobytes(order="C") == repeated_mean.tobytes(
                order="C"
            ) and std.tobytes(order="C") == repeated_std.tobytes(order="C")
            teacher_query_mismatch_count += int(not query_exact)
            applied = np.ascontiguousarray(
                np.clip(mean, env.action_space.low, env.action_space.high),
                dtype=np.float32,
            )
            clipped_values += int(np.count_nonzero(applied != mean))
            observation_after, reward, terminated, truncated, info = env.step(mean)
            observation_after = np.ascontiguousarray(
                observation_after, dtype=np.float32
            )
            if not isinstance(info, Mapping):
                raise V9HybridTeacherProbeError(f"step {step} info is malformed")
            samples = info.get("binary_phase_sensor_samples")
            if (
                not isinstance(samples, Sequence)
                or isinstance(samples, (str, bytes))
                or len(samples) != contract.EXPECTED_SAMPLES_PER_STEP
            ):
                raise V9HybridTeacherProbeError(
                    f"step {step} lacks ten V26 sensor samples"
                )
            previous = (
                float(baseline_sensor["time_s"])
                + index * contract.EXPECTED_POLICY_DT_S
            )
            validated_samples = [
                legacy._validate_raw_sample(
                    sample,
                    previous + sample_index * contract.EXPECTED_SAMPLE_DT_S,
                    f"step {step} sample {sample_index}",
                )
                for sample_index, sample in enumerate(samples, start=1)
            ]
            raw_sample_count += len(validated_samples)
            v26_collector._accumulate_binary_events_v26(
                binary_events,
                info=info,
                boundary_s=float(info.get("time")),
            )
            routing_exact = (
                info.get("binary_phase_fsm_executed") is True
                and info.get("binary_phase_fsm_mode") == "binary_active"
                and info.get("binary_phase_event_contract_id")
                == contract.EVENT_CONTRACT_ID
                and info.get("online_grf_applied_sides") == ["left"]
            )
            routing_failure_count += int(not routing_exact)
            reward_terms = info.get("reward_terms")
            phase = info.get("phase_fsm")
            if not isinstance(reward_terms, Mapping) or not isinstance(phase, Mapping):
                raise V9HybridTeacherProbeError(
                    f"step {step} diagnostics are incomplete"
                )
            penetration = float(reward_terms.get("grf_penetration_m"))
            if not math.isfinite(penetration) or penetration < 0.0:
                raise V9HybridTeacherProbeError("penetration is malformed")
            penetrations.append(penetration)
            phase_valid_cycles = int(float(phase.get("valid_cycle_count", 0)))
            timeout_count += int(float(phase.get("timeout_exceeded", 0.0)) > 0.0)
            invalid_event_count = max(
                invalid_event_count,
                int(float(phase.get("invalid_event_count", 0.0))),
            )
            classified = so_recovery.classify_policy_step(
                info.get("so_solver_audit_entries"),
                policy_id=contract.SO_POLICY_ID,
            )
            counters = classified["counters"]
            control_window_count += int(counters["control_window_count"])
            step_so_unaccepted = int(
                counters["unaccepted_hard_so_fallback_count"]
                + counters["unaccepted_bounded_ls_count"]
            )
            so_solver_unaccepted_count += step_so_unaccepted
            step_sea_fallback = env_source._sea_fallback_count(
                info.get("sea_segment_diagnostics")
            )
            sea_plugin_fallback_count += step_sea_fallback
            hard_invalid_count += int("failure" in info)
            finite = bool(
                observation.shape == (contract.EXPECTED_FULL_FEATURES,)
                and observation_after.shape == (contract.EXPECTED_FULL_FEATURES,)
                and np.all(np.isfinite(observation))
                and np.all(np.isfinite(observation_after))
                and np.all(np.isfinite(student))
                and np.all(np.isfinite(teacher_view))
                and np.all(np.isfinite(mean))
                and np.all(np.isfinite(std))
                and math.isfinite(float(reward))
            )
            nonfinite_count += int(not finite)
            checks = {
                "causal_columns_byte_exact": causal_exact,
                "teacher_query_byte_exact": query_exact,
                "teacher_mean_unclipped": bool(
                    mean.tobytes(order="C") == applied.tobytes(order="C")
                ),
                "binary_active_routing_exact": routing_exact,
                "ten_sensor_samples": len(validated_samples)
                == contract.EXPECTED_SAMPLES_PER_STEP,
                "ten_control_windows": counters["control_window_count"] == 10,
                "no_unaccepted_so": step_so_unaccepted == 0,
                "no_sea_fallback": step_sea_fallback == 0,
                "finite": finite,
            }
            step_contract_failure_count += int(not all(checks.values()))
            row = {
                "schema_version": contract.SCHEMA_VERSION,
                "protocol_id": contract.PROTOCOL_ID,
                "case_id": contract.CASE_ID,
                "student_v26_observation": student.tolist(),
                "hybrid_teacher_observation": teacher_view.tolist(),
                "teacher_privileged_values": teacher_view[
                    list(PRIVILEGED_INDICES)
                ].tolist(),
                "teacher_mean": mean.tolist(),
                "teacher_std": std.tolist(),
                "applied_action": applied.tolist(),
                "checks": checks,
                "reward": float(reward),
                "reward_terms": legacy._jsonable(reward_terms),
                "terminated": bool(terminated),
                "truncated": bool(truncated),
                "end_reason": info.get("end_reason"),
                "binary_phase_sensor_samples": legacy._jsonable(validated_samples),
                "binary_phase_fsm": legacy._jsonable(info.get("binary_phase_fsm")),
                "phase_fsm": legacy._jsonable(phase),
                "so_recovery_counters": legacy._jsonable(counters),
            }
            writer.write_step(step, row)
            rows.append({"step": step, **row})
            observation = observation_after
            current_info = dict(info)
            if step == 1 or step % 10 == 0 or terminated or truncated:
                _progress(step, started)
            if terminated or truncated:
                break
    finally:
        env.close()

    if binary_events is None:
        raise V9HybridTeacherProbeError("binary event accumulator was not initialized")
    binary_gate = v26_collector._finalize_binary_event_gate_v26(
        binary_events, raw_sample_count
    )
    summary = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.ROLLOUT_STATUS,
        "protocol_id": contract.PROTOCOL_ID,
        "probe_id": contract.PROBE_ID,
        "case_id": contract.CASE_ID,
        "action_selection": "deterministic",
        "episode_start_offset_s": case["episode_start_offset_s"],
        "runtime_seed": runtime_seed,
        "behavior": "H0_HYBRID_TEACHER_MEAN_CLOSED_LOOP_V26_BINARY_ACTIVE",
        "teacher_id": contract.TEACHER_ID,
        "teacher_builder": "from_current_shadow",
        "teacher_privileged_indices": list(contract.PRIVILEGED_INDICES),
        "steps": len(rows),
        "control_window_count": control_window_count,
        "raw_sensor_sample_count": raw_sample_count,
        "end_reason": info.get("end_reason") if isinstance(info, Mapping) else None,
        "terminated": bool(terminated),
        "truncated": bool(truncated),
        "phase_valid_cycle_count": phase_valid_cycles,
        "grf_penetration_max_m": max(penetrations, default=0.0),
        "causal_column_mismatch_count": causal_mismatch_count,
        "teacher_query_mismatch_count": teacher_query_mismatch_count,
        "action_clipped_values": clipped_values,
        "nonfinite_count": nonfinite_count,
        "timeout_count": timeout_count,
        "safety_stop_count": int(bool(terminated)),
        "invalid_event_count": invalid_event_count,
        "hard_invalid_count": hard_invalid_count,
        "so_solver_unaccepted_count": so_solver_unaccepted_count,
        "sea_plugin_fallback_count": sea_plugin_fallback_count,
        "routing_failure_count": routing_failure_count,
        "step_contract_failure_count": step_contract_failure_count,
        "binary_event_failure_count": int(binary_gate.get("passed") is not True),
        "binary_phase_event_gate": binary_gate,
        "binary_phase_fsm_mode": "binary_active",
        "event_contract_id": contract.EVENT_CONTRACT_ID,
        "morphology_weight": env_config["reward"]["morphology_weight"],
        "n_actor": len(actor_names),
        "n_observation": len(full_names),
        "observation_dtype": contract.EXPECTED_DTYPE,
        "actor_feature_names": list(actor_names),
        "observation_feature_names": list(full_names),
        "candidate_created": False,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
    }
    partial = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": "H0_PRIMARY_SPLIT_V9_HYBRID_TEACHER_PROBE_PERSISTED_BEFORE_GATE",
        "protocol_id": contract.PROTOCOL_ID,
        "case_id": contract.CASE_ID,
        "steps": len(rows),
        "gate_evaluated": False,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
    }
    persisted = writer.finalize_before_gate(
        trace=rows, partial_summary=partial, summary=summary
    )

    def evaluate(_records: dict[str, Any]) -> dict[str, Any]:
        gate = contract.rollout_gate(summary)
        gate["persisted_before_gate"] = persisted
        return gate

    writer.run_gate(evaluate)
    gate = _mapping(writer.gate_path)
    if gate.get("passed") is not True:
        writer.publish_failure(
            end_reason="hybrid_teacher_probe_gate_failed",
            error={
                "type": "V9HybridTeacherProbeGateFailure",
                "message": "hybrid-teacher closed-loop probe gate failed",
            },
            status=contract.ROLLOUT_FAIL_STATUS,
            details={"gate": _record(writer.gate_path)},
        )
        raise V9HybridTeacherProbeError("hybrid-teacher rollout gate failed")
    receipt = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.ROLLOUT_PASS_STATUS,
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "probe_id": contract.PROBE_ID,
        "case_id": contract.CASE_ID,
        "artifacts": writer.artifact_records(),
        "execution_lock": _record(LOCK),
        "execution_claim": _record(CLAIM),
        "worker_claim": _record(WORKER_CLAIM),
        "candidate_created": False,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
    }
    receipt_path = write_json_exclusive(writer.run_directory / "receipt.json", receipt)
    return {**receipt, "receipt": _record(receipt_path)}


def run_worker(*, output_dir: str | Path, execution_token: str) -> dict[str, Any]:
    lock = verify_lock()
    verify_worker_claim(execution_token)
    observed = Path(output_dir).expanduser().resolve()
    if observed != ROLLOUT_ROOT:
        raise V9HybridTeacherProbeError(
            f"non-canonical worker destination: {observed} != {ROLLOUT_ROOT}"
        )
    writer = ForensicRolloutWriter(ROLLOUT_ROOT, artifact_root=REPO_ROOT)
    try:
        writer.start(_worker_start_payload(lock))
        return _execute_worker(writer=writer)
    except Exception as exc:
        if writer.run_start_path.is_file() and not os.path.lexists(writer.failure_path):
            try:
                writer.publish_failure(
                    end_reason="worker_exception",
                    error=exc,
                    status=contract.ROLLOUT_FAIL_STATUS,
                    details={
                        "case_id": contract.CASE_ID,
                        "candidate_created": False,
                        "actor_updates": 0,
                        "critic_updates": 0,
                        "ppo_updates": 0,
                        "protected_trials_opened": [],
                        "reserve_trials_opened": [],
                    },
                )
            except Exception:
                pass
        raise


def verify_receipt() -> dict[str, Any]:
    receipt_path = ROLLOUT_ROOT / "receipt.json"
    receipt = _mapping(receipt_path)
    summary = _mapping(ROLLOUT_ROOT / "summary.json")
    gate = _mapping(ROLLOUT_ROOT / "gate.json")
    expected_gate = contract.rollout_gate(summary)
    expected_gate["persisted_before_gate"] = gate.get("persisted_before_gate")
    if (
        receipt.get("status") != contract.ROLLOUT_PASS_STATUS
        or receipt.get("passed") is not True
        or receipt.get("protocol_id") != contract.PROTOCOL_ID
        or receipt.get("case_id") != contract.CASE_ID
        or receipt.get("execution_lock") != _record(LOCK)
        or receipt.get("execution_claim") != _record(CLAIM)
        or receipt.get("worker_claim") != _record(WORKER_CLAIM)
        or receipt.get("actor_updates") != 0
        or receipt.get("critic_updates") != 0
        or receipt.get("ppo_updates") != 0
        or receipt.get("protected_trials_opened") != []
        or receipt.get("reserve_trials_opened") != []
        or canonical_json_bytes(gate) != canonical_json_bytes(expected_gate)
        or gate.get("passed") is not True
    ):
        raise V9HybridTeacherProbeError("hybrid-teacher receipt drifted")
    return receipt


def _worker_command(token: str) -> list[str]:
    return [
        sys.executable,
        str(Path(__file__).resolve()),
        "--worker",
        "--output-dir",
        str(ROLLOUT_ROOT),
        "--execution-token",
        token,
    ]


def execute() -> dict[str, Any]:
    verify_lock(require_run_root_absent=True)
    token = secrets.token_urlsafe(32)
    token_sha = _token_sha256(token)
    write_json_exclusive(CLAIM, _claim_payload(token_sha))
    write_json_exclusive(WORKER_CLAIM, _worker_claim_payload(token_sha))
    started = time.time()
    command = _worker_command(token)
    return_code: int | None = None
    error: str | None = None
    try:
        completed = subprocess.run(
            command,
            cwd=str(REPO_ROOT),
            check=False,
            timeout=WORKER_TIMEOUT_S,
        )
        return_code = completed.returncode
        if return_code != 0:
            error = f"worker exited with status {return_code}"
    except subprocess.TimeoutExpired:
        error = f"worker exceeded timeout {WORKER_TIMEOUT_S:.0f}s"

    passed = False
    receipt_record: dict[str, Any] | None = None
    failure_record: dict[str, Any] | None = None
    if error is None:
        try:
            verify_receipt()
            receipt_record = _record(ROLLOUT_ROOT / "receipt.json")
            passed = True
        except Exception as exc:
            error = f"{type(exc).__name__}: {exc}"
    if (ROLLOUT_ROOT / "failure.json").is_file():
        failure_record = _record(ROLLOUT_ROOT / "failure.json")
    ledger = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": (
            contract.PROTOCOL_PASS_STATUS if passed else contract.PROTOCOL_FAIL_STATUS
        ),
        "passed": passed,
        "protocol_id": contract.PROTOCOL_ID,
        "probe_id": contract.PROBE_ID,
        "case_id": contract.CASE_ID,
        "started_unix_s": started,
        "completed_unix_s": time.time(),
        "worker_return_code": return_code,
        "receipt": receipt_record,
        "failure": failure_record,
        "error": error,
        "retry_authorized": False,
        "candidate_created": False,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
        "next_stage": (
            "HYBRID_TEACHER_VIABLE_FOR_FRESH_ADAPTATION_DESIGN"
            if passed
            else "STOP_AND_REJECT_10_11_ONLY_HYBRID_TEACHER"
        ),
    }
    write_json_exclusive(LEDGER, ledger)
    print(json.dumps(ledger, indent=2, sort_keys=True, allow_nan=False))
    if not passed:
        raise V9HybridTeacherProbeError(error or contract.PROTOCOL_FAIL_STATUS)
    return ledger


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--prepare", action="store_true")
    mode.add_argument("--execute", action="store_true")
    mode.add_argument("--verify", action="store_true")
    mode.add_argument("--worker", action="store_true")
    parser.add_argument("--output-dir")
    parser.add_argument("--execution-token")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        if args.prepare:
            result = prepare()
        elif args.execute:
            result = execute()
        elif args.verify:
            result = {
                "lock": verify_lock(),
                "receipt": verify_receipt(),
                "ledger": _mapping(LEDGER),
            }
        else:
            if not args.output_dir or not args.execution_token:
                raise V9HybridTeacherProbeError("worker arguments are incomplete")
            result = run_worker(
                output_dir=args.output_dir,
                execution_token=args.execution_token,
            )
    except Exception as exc:
        print(
            f"V9 hybrid-teacher probe failed closed: {type(exc).__name__}: {exc}",
            file=sys.stderr,
        )
        return 2
    print(json.dumps(result, indent=2, sort_keys=True, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
