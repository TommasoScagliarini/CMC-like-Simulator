"""Execute the frozen post-holdout autonomous qualification of H0 V3.

The supervisor always completes all six condition-matched analog-H0 baseline
rollouts before publishing the baseline and tolerance decision receipts.  Only
then can a candidate worker open its preallocated destination.  This module
contains an executable entry point, but importing it never starts OpenSim and
the qualification freezer never invokes a rollout.
"""

from __future__ import annotations

import argparse
import copy
import json
import math
import subprocess
import sys
import time
import traceback
from pathlib import Path
from typing import Any, Mapping, Sequence


REPO_ROOT = Path(__file__).resolve().parents[1]
VALIDATION_ROOT = REPO_ROOT / "validation"
TRAJECTORY_ROOT = REPO_ROOT / "Trajectory Generator"
BASELINE_ROOT = TRAJECTORY_ROOT / "baseline_MLP"
for import_root in (VALIDATION_ROOT, BASELINE_ROOT, TRAJECTORY_ROOT, REPO_ROOT):
    if str(import_root) not in sys.path:
        sys.path.insert(0, str(import_root))

import compare_h0_primary_grf_split_v3_qualification as gates  # noqa: E402
import h0_primary_grf_split_v3_qualification_contract as contract  # noqa: E402
import h0_primary_grf_split_v3_qualification_scaffold as scaffold  # noqa: E402
import h0_v3_so_recovery_contract as so_recovery  # noqa: E402
import primary_grf_split_adaptation as split_contract  # noqa: E402
import run_h0_primary_grf_split_v1_adaptation as v1  # noqa: E402
import run_h0_primary_grf_split_v3_semantic_replay as v3  # noqa: E402
import run_h0_v25_abc_preflight as h0_runtime  # noqa: E402


LOCK = REPO_ROOT / contract.LOCK_PATH
RUN_ROOT = REPO_ROOT / contract.RUN_ROOT
SOURCE_H0_MODULE = REPO_ROOT / contract.SOURCE_H0_MODULE_PATH
CANDIDATE_MODULE = REPO_ROOT / contract.CANDIDATE_MODULE_PATH
WORKER_TIMEOUT_S = 2400.0


class QualificationExecutionError(RuntimeError):
    """Raised on every protocol, provenance, runtime, or gate failure."""


def _case(case_id: str) -> dict[str, Any]:
    matches = [item for item in contract.canonical_cases() if item["case_id"] == case_id]
    if len(matches) != 1:
        raise QualificationExecutionError(f"unknown qualification case {case_id!r}")
    return matches[0]


def _record(path: str | Path) -> dict[str, Any]:
    return v3.source_record(Path(path).resolve())


def _frozen_record_path(record: Any, label: str) -> Path:
    if not isinstance(record, Mapping) or set(record) != {"path", "sha256", "size_bytes"}:
        raise QualificationExecutionError(f"{label} artifact record is malformed")
    relative = record["path"]
    if not isinstance(relative, str) or not relative or Path(relative).is_absolute():
        raise QualificationExecutionError(f"{label} path is not repository-relative")
    resolved = (REPO_ROOT / relative).resolve()
    try:
        resolved.relative_to(REPO_ROOT.resolve())
    except ValueError as exc:
        raise QualificationExecutionError(f"{label} escaped the repository") from exc
    if (
        not resolved.is_file()
        or resolved.stat().st_size != v3._require_counter(record["size_bytes"], f"{label}.size")
        or v3.sha256_file(resolved) != record["sha256"]
    ):
        raise QualificationExecutionError(f"{label} integrity mismatch")
    return resolved


def verify_lock() -> dict[str, Any]:
    lock = v3._strict_mapping(LOCK)
    expected_destinations = [
        contract.rollout_destination(role, case_id).as_posix()
        for role in ("baseline", "candidate")
        for case_id in contract.CASE_IDS
    ]
    if (
        lock.get("schema_version") != contract.SCHEMA_VERSION
        or lock.get("status") != "H0_PRIMARY_SPLIT_V3_QUALIFICATION_UNLOCKED"
        or lock.get("protocol_id") != contract.PROTOCOL_ID
        or lock.get("so_policy_id") != contract.SO_POLICY_ID
        or lock.get("canonical_cases") != list(contract.canonical_cases())
        or lock.get("destinations") != expected_destinations
        or lock.get("authority") != contract.AUTHORITY
        or lock.get("protected_trials_opened") != []
    ):
        raise QualificationExecutionError("qualification lock contract drifted")
    for section in ("sources", "inputs"):
        records = lock.get(section)
        if not isinstance(records, Mapping) or not records:
            raise QualificationExecutionError(f"qualification lock {section} missing")
        for name, record in records.items():
            _frozen_record_path(record, f"lock.{section}.{name}")
    expected_candidate = {
        "state": _record(CANDIDATE_MODULE / "module_state.pkl"),
        "ctor": _record(CANDIDATE_MODULE / "class_and_ctor_args.pkl"),
        "metadata": _record(CANDIDATE_MODULE / "metadata.json"),
    }
    if lock.get("candidate_module") != expected_candidate:
        raise QualificationExecutionError("frozen candidate module drifted")
    return lock


def _load_noise_tape(case: Mapping[str, Any], np: Any) -> tuple[Any, str]:
    tape_path = (REPO_ROOT / str(case["noise_tape"])).resolve()
    lock = verify_lock()
    records = lock["inputs"]
    matching = [
        record
        for name, record in records.items()
        if name.startswith("noise_tape_") and record.get("path") == case["noise_tape"]
    ]
    if len(matching) != 1:
        raise QualificationExecutionError("noise tape is not uniquely frozen")
    _frozen_record_path(matching[0], "qualification noise tape")
    with np.load(tape_path, allow_pickle=False) as archive:
        if "standard_normal" not in archive.files:
            raise QualificationExecutionError("noise tape lacks standard_normal")
        tape = np.asarray(archive["standard_normal"]).copy()
    if tape.dtype != np.dtype("float32") or tape.shape != (contract.EXPECTED_STEPS, 2):
        raise QualificationExecutionError("noise tape dtype/shape drifted")
    if not np.all(np.isfinite(tape)):
        raise QualificationExecutionError("noise tape contains non-finite values")
    if case["action_selection"] == "deterministic" and np.count_nonzero(tape):
        raise QualificationExecutionError("deterministic tape is not all zero")
    return tape, split_contract.array_sha256(tape)


def _claim_destination(role: str, case_id: str, output_dir: str | Path) -> Path:
    expected = (REPO_ROOT / contract.rollout_destination(role, case_id)).resolve()
    destination = Path(output_dir).expanduser().resolve()
    if destination != expected:
        raise QualificationExecutionError("qualification destination drifted")
    if not destination.is_dir() or any(destination.iterdir()):
        raise QualificationExecutionError("destination is missing or already consumed")
    return destination


def _require_baseline_complete() -> None:
    for case_id in contract.CASE_IDS:
        receipt_path = REPO_ROOT / contract.rollout_destination("baseline", case_id) / "receipt.json"
        receipt = v3._strict_mapping(receipt_path)
        if (
            receipt.get("passed") is not True
            or receipt.get("status") != "PASS_H0_PRIMARY_SPLIT_V3_QUALIFICATION_ROLLOUT"
            or receipt.get("role") != "baseline"
            or receipt.get("case_id") != case_id
        ):
            raise QualificationExecutionError("all six baselines must pass before candidate access")


def _enforce_predecessors(role: str) -> None:
    if role == "baseline":
        if (REPO_ROOT / contract.DECISION_RECEIPT_PATH).exists():
            raise QualificationExecutionError("baseline stage cannot run after decision publication")
        return
    _require_baseline_complete()
    scaffold.validate_qualification_prerequisites(repo_root=REPO_ROOT)


def _solver_totals() -> dict[str, int]:
    return {key: 0 for key in v3.SO_RECOVERY_COUNTER_KEYS}


def _run_rollout(*, role: str, case_id: str, output_dir: str | Path) -> dict[str, Any]:
    if role not in {"baseline", "candidate"}:
        raise QualificationExecutionError(f"unknown qualification role {role!r}")
    verify_lock()
    _enforce_predecessors(role)
    destination = _claim_destination(role, case_id, output_dir)
    case = _case(case_id)
    rollout_eval, np, torch, RLModule, env_factory = v1._load_stack()
    runtime_seed = int(case["runtime_seed"])
    np.random.seed(runtime_seed)
    torch.manual_seed(runtime_seed)
    module_path = SOURCE_H0_MODULE if role == "baseline" else CANDIDATE_MODULE
    module = RLModule.from_checkpoint(module_path.resolve())
    env_config = v3.build_env_config(
        seed=runtime_seed, offset_s=float(case["episode_start_offset_s"])
    )
    env = env_factory.make_cmc_env(env_config)
    noise_tape, noise_tape_sha256 = _load_noise_tape(case, np)
    action_shape = tuple(int(value) for value in env.action_space.shape)
    if action_shape != contract.EXPECTED_ACTION_SHAPE:
        raise QualificationExecutionError("action shape drifted")

    reserve = h0_runtime._empty_accumulator()
    residual = h0_runtime._empty_accumulator()
    sea = h0_runtime._sea_accumulators()
    solver_totals = _solver_totals()
    solver_journal: list[dict[str, Any]] = []
    trace: list[dict[str, Any]] = []
    penetrations: list[float] = []
    clipping_values = 0
    timeout_count = 0
    invalid_event_count = 0
    hard_invalid_count = 0
    sea_plugin_fallback_count = 0
    policy_step_terminal_fallback_count = 0
    terminated = False
    truncated = False
    info: dict[str, Any] = {}
    actor_names: tuple[str, ...] = ()
    full_names: tuple[str, ...] = ()
    started = time.monotonic()
    try:
        observation, current_info = env.reset(seed=runtime_seed)
        observation = np.asarray(observation, dtype=np.float32)
        base = env.unwrapped
        actor_names = tuple(str(name) for name in base.actor_feature_names)
        full_names = tuple(str(name) for name in base.observation_feature_names)
        rollout_eval._validate_module_observation_contract(module, actor_names, full_names)
        if (
            observation.dtype != np.dtype("float32")
            or observation.shape != (contract.EXPECTED_FULL_FEATURES,)
            or len(actor_names) != contract.EXPECTED_ACTOR_FEATURES
            or len(full_names) != contract.EXPECTED_FULL_FEATURES
        ):
            raise QualificationExecutionError("35/84 float32 observation contract drifted")
        shadow_fsm = copy.deepcopy(base._phase_fsm)
        body_weight_n = float(base._body_weight_n)
        for step_index in range(contract.EXPECTED_STEPS):
            paired = split_contract.build_paired_views(
                observation,
                actor_names,
                current_info,
                body_weight_n=body_weight_n,
                reset_boundary=step_index == 0,
                teacher_phase_observation=shadow_fsm.observation(),
            )
            if role == "baseline":
                policy_input = v1._teacher_full_observation(observation, paired, np)
                actor_view = paired.teacher
            else:
                policy_input = observation.copy()
                actor_view = paired.student
                if policy_input[: contract.EXPECTED_ACTOR_FEATURES].tobytes() != actor_view.tobytes():
                    raise QualificationExecutionError("candidate action view is not primary-split")
            _raw, mean, std = v1._policy(module, policy_input, action_shape, rollout_eval)
            mean = np.asarray(mean, dtype=np.float32).reshape(action_shape)
            std = np.asarray(std, dtype=np.float32).reshape(action_shape)
            if not np.allclose(std, contract.STOCHASTIC_SIGMA, rtol=0.0, atol=1.0e-8):
                raise QualificationExecutionError("actor logstd drifted")
            if case["action_selection"] == "stochastic":
                raw_action = mean + std * noise_tape[step_index]
            else:
                raw_action = mean.copy()
            raw_action = np.asarray(raw_action, dtype=np.float32).reshape(action_shape)
            if not np.all(np.isfinite(raw_action)):
                raise QualificationExecutionError("actor action is non-finite")
            applied = np.clip(raw_action, env.action_space.low, env.action_space.high).astype(np.float32)
            clipping_values += int(np.count_nonzero(applied != raw_action))
            observation_after, reward, terminated, truncated, info = env.step(raw_action)
            observation_after = np.asarray(observation_after, dtype=np.float32)
            if not isinstance(info, Mapping):
                raise QualificationExecutionError("environment info is malformed")
            if (
                info.get("event_contract_id") != contract.EVENT_CONTRACT_ID
                or info.get("online_grf_applied_sides") != ["left"]
            ):
                raise QualificationExecutionError("primary/event routing drifted")
            reward_terms = info.get("reward_terms")
            if not isinstance(reward_terms, Mapping):
                raise QualificationExecutionError("reward terms are missing")
            h0_runtime._accumulate_scalar(reserve, reward_terms["reserve_norm_nm"])
            h0_runtime._accumulate_scalar(residual, reward_terms["residual_norm_nm"])
            penetration = float(reward_terms["grf_penetration_m"])
            if not math.isfinite(penetration) or penetration < 0.0:
                raise QualificationExecutionError("penetration is malformed")
            penetrations.append(penetration)
            sea_payload = info.get("sea_segment_diagnostics")
            h0_runtime._accumulate_sea(sea, sea_payload)
            if not isinstance(sea_payload, Mapping) or not isinstance(sea_payload.get("joints"), Mapping):
                raise QualificationExecutionError("SEA diagnostics are malformed")
            step_sea_fallback = 0
            for joint in contract.JOINTS:
                joint_payload = sea_payload["joints"].get(joint)
                if not isinstance(joint_payload, Mapping):
                    raise QualificationExecutionError(f"SEA diagnostics missing {joint}")
                step_sea_fallback += v3._require_counter(
                    joint_payload.get("tau_input_plugin_fallback_count"), f"{joint} tau fallback"
                ) + v3._require_counter(
                    joint_payload.get("motor_accel_plugin_fallback_count"), f"{joint} accel fallback"
                )
            sea_plugin_fallback_count += step_sea_fallback
            phase = info.get("phase_fsm")
            if not isinstance(phase, Mapping):
                raise QualificationExecutionError("phase FSM diagnostics are missing")
            timeout_count += int(float(phase.get("timeout_exceeded", 0.0)) > 0.0)
            invalid_event_count = max(invalid_event_count, int(float(phase.get("invalid_event_count", 0.0))))
            so = info.get("so_diagnostics")
            if not isinstance(so, Mapping) or type(so.get("solver_fallback_used")) is not bool:
                raise QualificationExecutionError("SO terminal diagnostic is missing")
            terminal_fallback = bool(so["solver_fallback_used"])
            policy_step_terminal_fallback_count += int(terminal_fallback)
            audit_entries, _attempts, _hard = v3._validate_so_solver_audit_entries(
                info.get("so_solver_audit_entries"),
                step_index=step_index + 1,
                selected_fallback=terminal_fallback,
            )
            recovery = so_recovery.classify_policy_step(
                info.get("so_solver_audit_entries"), policy_id=contract.SO_POLICY_ID
            )
            for key in v3.SO_RECOVERY_COUNTER_KEYS:
                solver_totals[key] += v3._require_counter(
                    recovery["counters"].get(key), f"SO recovery {key}"
                )
            solver_journal.append(
                {"step": step_index + 1, "time_s": float(info["time"]), "control_windows": audit_entries}
            )
            hard_invalid_count += int("failure" in info)
            v1._update_shadow_fsm(shadow_fsm, info=info, body_weight_n=body_weight_n)
            trace.append(
                {
                    "step": step_index + 1,
                    "time_s": float(info["time"]),
                    "actor_input_view": "historical_analog" if role == "baseline" else "primary_split",
                    "actor_observation": actor_view.tolist(),
                    "mean_action": mean.tolist(),
                    "standard_normal": noise_tape[step_index].tolist(),
                    "raw_action": raw_action.tolist(),
                    "reward": float(reward),
                    "terminated": bool(terminated),
                    "truncated": bool(truncated),
                }
            )
            observation = observation_after
            current_info = dict(info)
            completed = step_index + 1
            if completed == 1 or completed % 25 == 0 or completed == contract.EXPECTED_STEPS:
                elapsed = time.monotonic() - started
                eta = elapsed / completed * (contract.EXPECTED_STEPS - completed)
                print(f"[v3/qualification/{role}/{case_id}] {completed:3d}/500 elapsed={elapsed:7.1f}s eta={eta:7.1f}s", flush=True)
            if terminated or truncated:
                break
    finally:
        env.close()

    sea_metrics = h0_runtime._finalize_sea(sea)
    aggregate_sea_fallback = sum(int(sea[joint]["fallback_count"]) for joint in contract.JOINTS)
    if aggregate_sea_fallback != sea_plugin_fallback_count:
        raise QualificationExecutionError("SEA fallback aggregation mismatch")
    phase = info.get("phase_fsm", {}) if isinstance(info, Mapping) else {}
    unaccepted_fallbacks = (
        solver_totals["unaccepted_hard_so_fallback_count"]
        + solver_totals["unaccepted_bounded_ls_count"]
        + sea_plugin_fallback_count
    )
    summary = {
        "schema_version": contract.SCHEMA_VERSION,
        "role": role,
        "case_id": case_id,
        "action_selection": case["action_selection"],
        "episode_start_offset_s": float(case["episode_start_offset_s"]),
        "action_seed": case["action_seed"],
        "runtime_seed": runtime_seed,
        "sigma": float(case["sigma"]),
        "noise_tape_sha256": noise_tape_sha256,
        "steps": len(trace),
        "end_reason": info.get("end_reason") if isinstance(info, Mapping) else None,
        "terminated": bool(terminated),
        "truncated": bool(truncated),
        "phase_valid_cycle_count": int(float(phase.get("valid_cycle_count", 0))),
        "invalid_event_count": invalid_event_count,
        "grf_penetration_max_m": max(penetrations, default=0.0),
        "action_clipped_values": clipping_values,
        "fallback_count": unaccepted_fallbacks,
        "raw_so_fallback_count": solver_totals["bounded_ls_invocation_count"],
        "policy_step_terminal_so_fallback_count": policy_step_terminal_fallback_count,
        "timeout_count": timeout_count,
        "safety_stop_count": int(bool(terminated)),
        "sea_plugin_fallback_count": sea_plugin_fallback_count,
        "hard_invalid_count": hard_invalid_count,
        "nonfinite_count": 0,
        "so_policy_id": contract.SO_POLICY_ID,
        "so_solver_control_window_count": solver_totals["control_window_count"],
        "so_solver_bounded_ls_invocation_count": solver_totals["bounded_ls_invocation_count"],
        "so_solver_verified_bounded_ls_count": solver_totals["verified_bounded_ls_count"],
        "so_solver_verified_status0_max_iter_count": solver_totals["verified_status0_max_iter_count"],
        "so_solver_unaccepted_hard_fallback_count": solver_totals["unaccepted_hard_so_fallback_count"],
        "so_solver_unaccepted_bounded_ls_count": solver_totals["unaccepted_bounded_ls_count"],
        "so_solver_hard_fallback_count": solver_totals["hard_so_fallback_count"],
        "so_solver_reuse_previous_count": solver_totals["reuse_previous_count"],
        "so_solver_bounded_ls_unsuccessful_count": solver_totals["bounded_ls_unsuccessful_count"],
        "so_solver_bounds_violation_count": solver_totals["bounds_violation_count"],
        "so_solver_nonfinite_count": solver_totals["nonfinite_solver_count"],
        "so_solver_selected_infeasible_count": solver_totals["selected_infeasible_count"],
        "so_solver_selected_solution_mismatch_count": solver_totals["selected_solution_mismatch_count"],
        "so_solver_residual_contract_mismatch_count": solver_totals["residual_contract_mismatch_count"],
        "n_actor": len(actor_names),
        "n_observation": len(full_names),
        "observation_dtype": "float32",
        "action_shape": list(action_shape),
        "action_dtype": "float32",
        "actor_input_view": "historical_analog" if role == "baseline" else "primary_split",
        "event_contract_id": contract.EVENT_CONTRACT_ID,
        "phase_fsm_input_mode": contract.PHASE_FSM_INPUT_MODE,
        "binary_phase_fsm_mode": "disabled",
        "online_grf_applied_sides": list(info.get("online_grf_applied_sides", [])),
        "morphology_weight": float(env_config["reward"]["morphology_weight"]),
        "episode_metrics": {
            "reserve_norm_nm": h0_runtime._finalize_accumulator(reserve),
            "residual_norm_nm": h0_runtime._finalize_accumulator(residual),
        },
        "sea_episode_metrics": sea_metrics,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
    }
    gate = gates.common_rollout_gate(summary, role=role, case_id=case_id)
    artifacts = {}
    for filename, payload in (
        ("trace.json", trace),
        ("summary.json", summary),
        ("solver_audit_journal.json", solver_journal),
        ("common_gate.json", gate),
    ):
        path = v3._write_json_exclusive(destination / filename, payload)
        artifacts[filename] = _record(path)
    receipt = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": gate["status"],
        "passed": gate["passed"],
        "role": role,
        "case_id": case_id,
        "artifacts": artifacts,
        "qualification_lock": _record(LOCK),
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
    }
    v3._write_json_exclusive(destination / "receipt.json", receipt)
    if not gate["passed"]:
        raise QualificationExecutionError(f"{role}/{case_id} failed the common gate")
    return receipt


def _publish_baseline_and_decision() -> tuple[dict[str, Any], dict[str, Any]]:
    _require_baseline_complete()
    case_metrics: dict[str, Any] = {}
    artifacts: dict[str, Any] = {}
    for case_id in contract.CASE_IDS:
        root = REPO_ROOT / contract.rollout_destination("baseline", case_id)
        summary_path = root / "summary.json"
        case_metrics[case_id] = gates.baseline_case_metrics(v3._strict_mapping(summary_path))
        artifacts[case_id] = {
            "summary": _record(summary_path),
            "receipt": _record(root / "receipt.json"),
            "solver_audit_journal": _record(root / "solver_audit_journal.json"),
        }
    manifest = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": "H0_PRIMARY_SPLIT_V3_QUALIFICATION_BASELINE_EVIDENCE_FROZEN",
        "baseline_id": "analog_h0_primary_physics_condition_matched_v1",
        "cases": artifacts,
        "qualification_lock": _record(LOCK),
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
    }
    manifest_path = v3._write_json_exclusive(REPO_ROOT / contract.BASELINE_MANIFEST_PATH, manifest)
    baseline = {
        "schema_version": 1,
        "status": "H0_PRIMARY_SPLIT_V3_QUALIFICATION_BASELINE_FROZEN",
        "passed": True,
        "baseline_id": manifest["baseline_id"],
        "case_metrics": case_metrics,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
    }
    baseline_path = v3._write_json_exclusive(REPO_ROOT / contract.BASELINE_RECEIPT_PATH, baseline)
    tolerances = contract.tolerance_rows()
    decision = {
        "schema_version": 1,
        "status": "H0_PRIMARY_SPLIT_V3_QUALIFICATION_BASELINE_TOLERANCE_DECIDED",
        "passed": True,
        "protocol_id": scaffold.PROTOCOL_ID,
        "decision_authority": "EXPLICIT_USER_DECISION",
        "candidate_holdout_receipt": _record(REPO_ROOT / contract.HOLDOUT_RECEIPT_PATH),
        "baseline": {
            "baseline_id": baseline["baseline_id"],
            "comparison_scope": "condition_matched_six_cases",
            "receipt": _record(baseline_path),
        },
        "tolerances": {
            "comparison_formula": "candidate <= baseline + max(absolute_tolerance, relative_tolerance * abs(baseline))",
            **tolerances,
        },
        "fixed_gates": {
            "expected_steps": scaffold.EXPECTED_STEPS,
            "minimum_valid_cycles": scaffold.MINIMUM_VALID_CYCLES,
            "penetration_limit_m": scaffold.PENETRATION_LIMIT_M,
            "penetration_comparison": "strict_less_than",
            "zero_count_fields": list(scaffold.ZERO_COUNT_FIELDS),
        },
        "runtime_contract": {
            "event_contract_id": scaffold.EVENT_CONTRACT_ID,
            "phase_fsm_input_mode": scaffold.PHASE_FSM_INPUT_MODE,
            "morphology_weight": scaffold.MORPHOLOGY_WEIGHT,
        },
        "canonical_case_ids": list(scaffold.CASE_IDS),
        "authority": dict(scaffold._DECISION_AUTHORITY),
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
    }
    v3._write_json_exclusive(REPO_ROOT / contract.DECISION_RECEIPT_PATH, decision)
    # Prove that the published pair satisfies the pre-existing scaffold before
    # any candidate worker is allowed to start.
    scaffold.validate_qualification_prerequisites(repo_root=REPO_ROOT)
    if not manifest_path.is_file():
        raise QualificationExecutionError("baseline evidence manifest publication failed")
    return baseline, decision


def _worker_command(role: str, case_id: str) -> list[str]:
    return [
        sys.executable,
        str(Path(__file__).resolve()),
        "--worker",
        "--role",
        role,
        "--case",
        case_id,
        "--output-dir",
        str(REPO_ROOT / contract.rollout_destination(role, case_id)),
    ]


def _run_worker_process(role: str, case_id: str) -> None:
    completed = subprocess.run(
        _worker_command(role, case_id),
        cwd=REPO_ROOT,
        timeout=WORKER_TIMEOUT_S,
        check=False,
    )
    if completed.returncode != 0:
        raise QualificationExecutionError(f"worker {role}/{case_id} exited {completed.returncode}")


def execute() -> dict[str, Any]:
    verify_lock()
    started = time.time()
    status = "FAIL_H0_PRIMARY_SPLIT_V3_AUTONOMOUS_QUALIFICATION"
    error = None
    passed = False
    case_gates: dict[str, Any] = {}
    try:
        for case_id in contract.CASE_IDS:
            _run_worker_process("baseline", case_id)
        _publish_baseline_and_decision()
        for case_id in contract.CASE_IDS:
            _run_worker_process("candidate", case_id)
            baseline = v3._strict_mapping(REPO_ROOT / contract.rollout_destination("baseline", case_id) / "summary.json")
            candidate = v3._strict_mapping(REPO_ROOT / contract.rollout_destination("candidate", case_id) / "summary.json")
            gate = gates.condition_matched_gate(baseline, candidate, case_id=case_id)
            v3._write_json_exclusive(REPO_ROOT / contract.gate_destination(case_id), gate)
            case_gates[case_id] = gate
            if gate["passed"] is not True:
                raise QualificationExecutionError(f"candidate case failed: {case_id}")
        status = "PASS_H0_PRIMARY_SPLIT_V3_AUTONOMOUS_QUALIFICATION"
        passed = True
    except Exception as exc:
        error = f"{type(exc).__name__}: {exc}"
    ledger = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": status,
        "passed": passed,
        "error": error,
        "started_unix_s": started,
        "completed_unix_s": time.time(),
        "qualification_lock": _record(LOCK),
        "case_gates": case_gates,
        "baseline_rollouts_completed": sum(
            int((REPO_ROOT / contract.rollout_destination("baseline", case_id) / "receipt.json").is_file())
            for case_id in contract.CASE_IDS
        ),
        "candidate_rollouts_completed": sum(
            int((REPO_ROOT / contract.rollout_destination("candidate", case_id) / "receipt.json").is_file())
            for case_id in contract.CASE_IDS
        ),
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "next_stage": "TRAINER_ZERO_UPDATE_PORT" if passed else "STOP_WITHOUT_RETRY_OR_RETUNING",
    }
    v3._write_json_exclusive(REPO_ROOT / contract.EXECUTION_LEDGER_PATH, ledger)
    print(json.dumps(ledger, indent=2, sort_keys=True, allow_nan=False), flush=True)
    if not passed:
        raise QualificationExecutionError(error or status)
    return ledger


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--worker", action="store_true")
    mode.add_argument("--execute", action="store_true")
    parser.add_argument("--role", choices=("baseline", "candidate"))
    parser.add_argument("--case", choices=contract.CASE_IDS)
    parser.add_argument("--output-dir")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        if args.execute:
            result = execute()
        else:
            if args.role is None or args.case is None or args.output_dir is None:
                raise QualificationExecutionError("worker role/case/output-dir are required")
            result = _run_rollout(role=args.role, case_id=args.case, output_dir=args.output_dir)
    except Exception as exc:
        if args.worker and args.output_dir:
            failure = Path(args.output_dir).expanduser().resolve() / "failure.json"
            if not failure.exists():
                try:
                    v3._write_json_exclusive(
                        failure,
                        {
                            "status": "FAIL_CLOSED",
                            "error": f"{type(exc).__name__}: {exc}",
                            "traceback": traceback.format_exc(),
                            "actor_updates": 0,
                            "critic_updates": 0,
                            "ppo_updates": 0,
                            "protected_trials_opened": [],
                        },
                    )
                except Exception:
                    pass
        print(f"H0 V3 qualification failed closed: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 2
    print(json.dumps(result, indent=2, sort_keys=True, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
