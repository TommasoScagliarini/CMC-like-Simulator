"""One-shot prescribed-teacher adaptation of H0 to ``primary_grf_split_v1``.

The source H0 is never used as a behavior policy in this branch.  Collection
and qualification references are driven by the prescribed prosthetic IK; the
candidate sees only the deployable 35-feature target observation.  No PPO or
critic update is performed.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import subprocess
import sys
import time
from pathlib import Path
from typing import Any, Mapping, Sequence


REPO_ROOT = Path(__file__).resolve().parents[1]
VALIDATION_ROOT = REPO_ROOT / "validation"
TRAJECTORY_ROOT = REPO_ROOT / "Trajectory Generator"
BASELINE_ROOT = TRAJECTORY_ROOT / "baseline_MLP"
for import_root in (VALIDATION_ROOT, BASELINE_ROOT, TRAJECTORY_ROOT, REPO_ROOT):
    if str(import_root) not in sys.path:
        sys.path.insert(0, str(import_root))

import compare_h0_primary_grf_split_v2 as gates  # noqa: E402
import run_h0_primary_grf_split_v1_adaptation as v1  # noqa: E402
import target_domain_imitation as imitation  # noqa: E402
import warm_start  # noqa: E402


LOCK = VALIDATION_ROOT / "h0_primary_grf_split_v2_execution_lock.json"
RUN_ROOT = (
    VALIDATION_ROOT
    / "h0_primary_grf_split_adaptation_runs"
    / "2026-08-06_h0_primary_split_v2_prescribed_teacher"
)
H0_MODULE = v1.H0_MODULE
EXPECTED_STEPS = 500
EXPECTED_SIGMA = 0.005
EXPECTED_ACTOR_FEATURES = 35
EXPECTED_FULL_FEATURES = 84
EVENT_CONTRACT = "primary_grf_split_v1+legacy_events_v1"
WORKER_TIMEOUT_S = 2400.0
TRIAL_IDS = ("02", "04", "08")
SELECTIONS = ("deterministic", "stochastic")
FIT = {
    "seed": 123,
    "epochs": 400,
    "batch_size": 128,
    "learning_rate": 5.0e-5,
    "validation_fraction": 1.0 / 3.0,
    "patience": 60,
    "clip_weight": 1.0,
    "logstd_weight": 0.0,
    "anchor_weight": 1.0e-2,
}


class H0PrimarySplitV2Error(RuntimeError):
    """Raised on any fail-closed V2 protocol violation."""


def sha256_file(path: str | Path) -> str:
    digest = hashlib.sha256()
    with Path(path).expanduser().resolve().open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def source_record(path: str | Path) -> dict[str, Any]:
    resolved = Path(path).expanduser().resolve()
    return {
        "path": str(resolved.relative_to(REPO_ROOT)),
        "sha256": sha256_file(resolved),
        "size_bytes": resolved.stat().st_size,
    }


def _strict_mapping(path: str | Path) -> dict[str, Any]:
    payload = gates.strict_json_load(path)
    if not isinstance(payload, Mapping):
        raise H0PrimarySplitV2Error(f"expected JSON object: {path}")
    return dict(payload)


def _path_from_record(record: Any, label: str) -> Path:
    if not isinstance(record, Mapping):
        raise H0PrimarySplitV2Error(f"{label} record is missing")
    if not {"path", "sha256", "size_bytes"}.issubset(record):
        raise H0PrimarySplitV2Error(f"{label} record is malformed")
    path = (REPO_ROOT / str(record["path"])).resolve()
    try:
        path.relative_to(REPO_ROOT)
    except ValueError as exc:
        raise H0PrimarySplitV2Error(f"{label} path escapes the repository") from exc
    if (
        not path.is_file()
        or path.stat().st_size != int(record["size_bytes"])
        or sha256_file(path) != str(record["sha256"])
    ):
        raise H0PrimarySplitV2Error(f"{label} artifact drifted: {path}")
    return path


def _verify_record_tree(value: Any, label: str) -> int:
    if isinstance(value, Mapping):
        if {"path", "sha256", "size_bytes"}.issubset(value):
            _path_from_record(value, label)
            return 1
        return sum(
            _verify_record_tree(child, f"{label}.{key}") for key, child in value.items()
        )
    if isinstance(value, list):
        return sum(
            _verify_record_tree(child, f"{label}[{index}]")
            for index, child in enumerate(value)
        )
    return 0


def verify_lock() -> dict[str, Any]:
    lock = _strict_mapping(LOCK)
    if lock.get("status") != "H0_PRIMARY_GRF_SPLIT_V2_EXECUTION_FROZEN":
        raise H0PrimarySplitV2Error("V2 execution lock status is not authoritative")
    if lock.get("run_root") != str(RUN_ROOT.relative_to(REPO_ROOT)):
        raise H0PrimarySplitV2Error("V2 run root drifted")
    authority = lock.get("authority")
    if not isinstance(authority, Mapping):
        raise H0PrimarySplitV2Error("V2 authority is missing")
    required_true = {
        "prescribed_teacher_collection_authorized",
        "actor_only_adaptation_authorized",
        "development_qualification_authorized",
        "zero_update_port_authorized",
    }
    forbidden = {
        "ppo_updates_authorized",
        "critic_updates_authorized",
        "protected_trial_access_authorized",
        "primary_grf_modification_authorized",
        "sea_semantic_modification_authorized",
        "runtime_promotion_authorized",
    }
    if any(authority.get(name) is not True for name in required_true):
        raise H0PrimarySplitV2Error("V2 required authority is absent")
    if any(authority.get(name) is not False for name in forbidden):
        raise H0PrimarySplitV2Error("V2 contains forbidden authority")
    verified = 0
    for key in ("runtime_closure", "input_closure", "v2_sources", "v2_inputs"):
        verified += _verify_record_tree(lock.get(key), key)
    if verified < 50:
        raise H0PrimarySplitV2Error("V2 frozen closure is unexpectedly incomplete")
    if dict(lock.get("fit", {})) != FIT:
        raise H0PrimarySplitV2Error("V2 fit hyperparameters drifted")
    return lock


def _claim_empty_destination(path: str | Path) -> Path:
    destination = Path(path).expanduser().resolve()
    try:
        destination.relative_to(RUN_ROOT)
    except ValueError as exc:
        raise H0PrimarySplitV2Error("output destination escapes V2 run root") from exc
    destination.mkdir(parents=True, exist_ok=True)
    if any(destination.iterdir()):
        raise H0PrimarySplitV2Error(f"refusing to clobber non-empty {destination}")
    return destination


def _build_env_config(trial_id: str, *, qualification: bool) -> dict[str, Any]:
    trial = v1.TRIALS[trial_id]
    offset_key = "qualification_offset_s" if qualification else "collection_offset_s"
    seed_key = "qualification_seed" if qualification else "collection_seed"
    config = v1.build_env_config(
        trial_id,
        offset_s=float(trial[offset_key]),
        seed=int(trial[seed_key]),
    )
    if (
        config.get("event_contract_id") != EVENT_CONTRACT
        or config.get("binary_phase_fsm_mode") != "disabled"
        or float(config.get("reward", {}).get("morphology_weight", math.nan)) != 0.0
    ):
        raise H0PrimarySplitV2Error("target environment contract drifted")
    return config


def _noise_tape(trial_id: str, selection: str, *, qualification: bool):
    import numpy as np

    if selection == "deterministic":
        return np.zeros((EXPECTED_STEPS, 2), dtype=np.float32)
    if not qualification:
        raise H0PrimarySplitV2Error("collection must be deterministic")
    return v1._load_noise_tape(
        stage="qualification", trial_id=trial_id, selection=selection
    )


def _prescribed_action(base: Any):
    import numpy as np

    target_t = min(
        float(base.t) + float(base.env_cfg.segment_duration),
        float(base._episode_end),
    )
    action = imitation.prescribed_teacher_action(
        base, target_t, lookahead_s=0.0
    ).astype(np.float32)
    if action.shape != (2,) or not np.all(np.isfinite(action)):
        raise H0PrimarySplitV2Error("prescribed teacher action is malformed")
    return action


def _candidate_policy(module: Any, observation: Any, rollout_eval: Any):
    import numpy as np

    _raw, mean, std = v1._policy(module, observation, (2,), rollout_eval)
    mean = np.asarray(mean, dtype=np.float32).reshape(-1)
    std = np.asarray(std, dtype=np.float32).reshape(-1)
    if mean.shape != (2,) or std.shape != (2,):
        raise H0PrimarySplitV2Error("candidate action head is malformed")
    if not np.allclose(std, EXPECTED_SIGMA, rtol=0.0, atol=1.0e-8):
        raise H0PrimarySplitV2Error("candidate logstd differs from H0")
    return mean, std


def layout_preflight(output: str | Path) -> dict[str, Any]:
    """Validate the real 35/84 target env and prescribed action without stepping."""

    rollout_eval, np, torch, RLModule, env_factory = v1._load_stack()
    module = RLModule.from_checkpoint(H0_MODULE.resolve())
    cases: dict[str, Any] = {}
    for trial_id in TRIAL_IDS:
        trial = v1.TRIALS[trial_id]
        seed = int(trial["collection_seed"])
        np.random.seed(seed)
        torch.manual_seed(seed)
        config = _build_env_config(trial_id, qualification=False)
        env = env_factory.make_cmc_env(config)
        try:
            observation, info = env.reset(seed=seed)
            observation = np.asarray(observation, dtype=np.float32)
            base = env.unwrapped
            actor_names = tuple(str(name) for name in base.actor_feature_names)
            full_names = tuple(str(name) for name in base.observation_feature_names)
            rollout_eval._validate_module_observation_contract(
                module, actor_names, full_names
            )
            v1._validate_reset_contract(
                module=module,
                env=env,
                observation=observation,
                actor_names=actor_names,
                full_names=full_names,
            )
            prescribed = _prescribed_action(base)
            if list(base.cfg.online_grf_applied_sides) != ["left"]:
                raise H0PrimarySplitV2Error("primary routing is not left-only")
            if base.env_cfg.binary_phase_detector_profile_file is not None:
                raise H0PrimarySplitV2Error("V25 profile is loaded in V2")
            cases[trial_id] = {
                "absolute_start_s": float(info["time"]),
                "expected_absolute_start_s": float(trial["collection_absolute_s"]),
                "actor_feature_names": list(actor_names),
                "observation_feature_names": list(full_names),
                "observation_dtype": str(observation.dtype),
                "prescribed_action": prescribed.tolist(),
                "prescribed_action_dtype": str(prescribed.dtype),
                "online_grf_applied_sides": list(base.cfg.online_grf_applied_sides),
                "binary_phase_fsm_mode": str(base.env_cfg.binary_phase_fsm_mode),
                "morphology_weight": float(config["reward"]["morphology_weight"]),
            }
            if (
                abs(
                    cases[trial_id]["absolute_start_s"]
                    - cases[trial_id]["expected_absolute_start_s"]
                )
                > 1.0e-9
            ):
                raise H0PrimarySplitV2Error("layout preflight start time drifted")
        finally:
            env.close()
    payload = {
        "schema_version": 2,
        "status": "PASS_H0_PRIMARY_GRF_SPLIT_V2_LAYOUT_PREFLIGHT",
        "passed": True,
        "cases": cases,
        "h0_executed": False,
        "teacher_rollout_executed": False,
        "actor_updates": 0,
        "ppo_updates": 0,
        "critic_updates": 0,
        "protected_trials_opened": [],
    }
    gates.write_json_exclusive(output, payload)
    return payload


def _run_rollout(
    *,
    trial_id: str,
    selection: str,
    role: str,
    output_dir: Path,
    qualification: bool,
    candidate_module: Path | None = None,
) -> dict[str, Any]:
    rollout_eval, np, torch, RLModule, env_factory = v1._load_stack()
    trial = v1.TRIALS[trial_id]
    seed = int(trial["qualification_seed" if qualification else "collection_seed"])
    offset = float(
        trial["qualification_offset_s" if qualification else "collection_offset_s"]
    )
    expected_absolute = float(
        trial["qualification_absolute_s" if qualification else "collection_absolute_s"]
    )
    np.random.seed(seed)
    torch.manual_seed(seed)
    module = None
    if role == "candidate":
        if candidate_module is None:
            raise H0PrimarySplitV2Error("candidate module is missing")
        module = RLModule.from_checkpoint(candidate_module.resolve())
    elif role != "reference":
        raise H0PrimarySplitV2Error(f"unknown rollout role {role}")
    config = _build_env_config(trial_id, qualification=qualification)
    env = env_factory.make_cmc_env(config)
    tape = _noise_tape(trial_id, selection, qualification=qualification)
    observations: list[Any] = []
    labels: list[Any] = []
    served_means: list[Any] = []
    served_actions: list[Any] = []
    trace: list[dict[str, Any]] = []
    reserve = v1.h0_runtime._empty_accumulator()
    residual = v1.h0_runtime._empty_accumulator()
    sea = v1.h0_runtime._sea_accumulators()
    penetrations: list[float] = []
    clipping_count = 0
    fallback_count = 0
    timeout_count = 0
    invalid_event_count = 0
    hard_invalid_count = 0
    teacher_dependency_count = 0
    action_selected_first_count = 0
    terminated = False
    truncated = False
    final_info: dict[str, Any] = {}
    actor_names: tuple[str, ...] = ()
    full_names: tuple[str, ...] = ()
    started = time.monotonic()
    try:
        observation, current_info = env.reset(seed=seed)
        observation = np.asarray(observation, dtype=np.float32)
        if abs(float(current_info["time"]) - expected_absolute) > 1.0e-9:
            raise H0PrimarySplitV2Error("absolute start time drifted")
        base = env.unwrapped
        actor_names = tuple(str(name) for name in base.actor_feature_names)
        full_names = tuple(str(name) for name in base.observation_feature_names)
        contract_module = module or RLModule.from_checkpoint(H0_MODULE.resolve())
        rollout_eval._validate_module_observation_contract(
            contract_module, actor_names, full_names
        )
        v1._validate_reset_contract(
            module=contract_module,
            env=env,
            observation=observation,
            actor_names=actor_names,
            full_names=full_names,
        )
        for step_index in range(EXPECTED_STEPS):
            candidate_input = None
            if role == "candidate":
                candidate_input = observation.copy()
                served_mean, served_std = _candidate_policy(
                    module, candidate_input, rollout_eval
                )
                raw_action = (
                    served_mean.copy()
                    if selection == "deterministic"
                    else served_mean + served_std * tape[step_index]
                )
                action_selected_first_count += 1
            else:
                served_mean = None
                raw_action = None

            teacher_action = _prescribed_action(base)
            if role == "reference":
                served_mean = teacher_action.copy()
                raw_action = (
                    teacher_action.copy()
                    if selection == "deterministic"
                    else teacher_action + np.float32(EXPECTED_SIGMA) * tape[step_index]
                )
            if served_mean is None or raw_action is None:
                raise H0PrimarySplitV2Error("action selection is incomplete")
            raw_action = np.asarray(raw_action, dtype=np.float32).reshape(2)
            if not np.all(np.isfinite(raw_action)):
                raise H0PrimarySplitV2Error("served action is non-finite")
            applied = np.clip(
                raw_action, env.action_space.low, env.action_space.high
            ).astype(np.float32)
            clipping_count += int(np.count_nonzero(applied != raw_action))
            actor_observation = observation[:EXPECTED_ACTOR_FEATURES].copy()
            if candidate_input is not None and (
                candidate_input[:EXPECTED_ACTOR_FEATURES].tobytes()
                != actor_observation.tobytes()
            ):
                teacher_dependency_count += 1
            observations.append(actor_observation)
            labels.append(teacher_action.copy())
            served_means.append(served_mean.copy())
            served_actions.append(raw_action.copy())

            next_observation, reward, terminated, truncated, info = env.step(raw_action)
            next_observation = np.asarray(next_observation, dtype=np.float32)
            if next_observation.shape != (EXPECTED_FULL_FEATURES,) or not np.all(
                np.isfinite(next_observation)
            ):
                raise H0PrimarySplitV2Error("post-step observation is malformed")
            if info.get("event_contract_id") != EVENT_CONTRACT:
                raise H0PrimarySplitV2Error("runtime event contract drifted")
            if info.get("online_grf_applied_sides") != ["left"]:
                raise H0PrimarySplitV2Error("runtime primary-GRF routing drifted")
            terms = info.get("reward_terms")
            if not isinstance(terms, Mapping):
                raise H0PrimarySplitV2Error("reward terms are missing")
            v1.h0_runtime._accumulate_scalar(reserve, terms["reserve_norm_nm"])
            v1.h0_runtime._accumulate_scalar(residual, terms["residual_norm_nm"])
            penetration = float(terms["grf_penetration_m"])
            if not math.isfinite(penetration) or penetration < 0.0:
                raise H0PrimarySplitV2Error("GRF penetration is malformed")
            penetrations.append(penetration)
            v1.h0_runtime._accumulate_sea(sea, info.get("sea_segment_diagnostics"))
            phase = info.get("phase_fsm")
            if not isinstance(phase, Mapping):
                raise H0PrimarySplitV2Error("phase FSM diagnostics are missing")
            timeout_count += int(float(phase.get("timeout_exceeded", 0.0)) > 0.0)
            invalid_event_count = max(
                invalid_event_count,
                int(float(phase.get("invalid_event_count", 0.0))),
            )
            so = info.get("so_diagnostics")
            if not isinstance(so, Mapping) or "solver_fallback_used" not in so:
                raise H0PrimarySplitV2Error(
                    "static-optimization diagnostics are missing"
                )
            fallback_count += int(so["solver_fallback_used"] is True)
            hard_invalid_count += int("failure" in info)
            trace.append(
                {
                    "step": step_index + 1,
                    "time_s": float(info["time"]),
                    "served_mean": served_mean.tolist(),
                    "prescribed_action": teacher_action.tolist(),
                    "raw_action": raw_action.tolist(),
                    "standard_normal": (
                        tape[step_index].tolist() if selection == "stochastic" else None
                    ),
                    "reward": float(reward),
                    "penetration_m": penetration,
                    "terminated": bool(terminated),
                    "truncated": bool(truncated),
                }
            )
            observation = next_observation
            current_info = dict(info)
            final_info = dict(info)
            completed = step_index + 1
            if completed == 1 or completed % 25 == 0 or completed == EXPECTED_STEPS:
                elapsed = time.monotonic() - started
                eta = elapsed / completed * (EXPECTED_STEPS - completed)
                print(
                    f"[{role}/trial{trial_id}/{selection}] {completed:3d}/"
                    f"{EXPECTED_STEPS} elapsed={elapsed:7.1f}s eta={eta:7.1f}s",
                    flush=True,
                )
            if terminated or truncated:
                break
    finally:
        env.close()

    sea_metrics = v1.h0_runtime._finalize_sea(sea)
    fallback_count += sum(
        int(sea[joint]["fallback_count"]) for joint in v1.h0_runtime.comparator.JOINTS
    )
    phase = final_info.get("phase_fsm", {})
    observation_array = np.ascontiguousarray(observations, dtype=np.float32)
    label_array = np.ascontiguousarray(labels, dtype=np.float32)
    served_mean_array = np.ascontiguousarray(served_means, dtype=np.float32)
    counterfactual = v1.split_contract.prediction_metrics(
        served_mean_array, label_array
    )
    tape_hash = v1.split_contract.array_sha256(tape)
    summary = {
        "schema_version": 2,
        "trial_id": trial_id,
        "plateau_id": "04",
        "behavior_role": "prescribed_reference" if role == "reference" else "candidate",
        "action_selection": selection,
        "seed": seed,
        "episode_start_time_s": expected_absolute,
        "episode_start_offset_s": offset,
        "sigma": EXPECTED_SIGMA if selection == "stochastic" else 0.0,
        "noise_tape_sha256": tape_hash,
        "steps": len(trace),
        "end_reason": final_info.get("end_reason"),
        "terminated": bool(terminated),
        "truncated": bool(truncated),
        "phase_valid_cycle_count": int(float(phase.get("valid_cycle_count", 0.0))),
        "invalid_event_count": invalid_event_count,
        "grf_penetration_max_m": max(penetrations, default=0.0),
        "action_clipped_values": clipping_count,
        "timeout_count": timeout_count,
        "safety_stop_count": int(bool(terminated)),
        "fallback_count": fallback_count,
        "hard_invalid_count": hard_invalid_count,
        "nonfinite_count": 0,
        "n_actor": len(actor_names),
        "n_observation": len(full_names),
        "observation_dtype": "float32",
        "action_shape": [2],
        "action_dtype": "float32",
        "morphology_weight": 0.0,
        "episode_metrics": {
            "reserve_norm_nm": v1.h0_runtime._finalize_accumulator(reserve),
            "residual_norm_nm": v1.h0_runtime._finalize_accumulator(residual),
        },
        "sea_episode_metrics": sea_metrics,
        "prescribed_counterfactual_error": {
            "sample_count": int(counterfactual["samples"]),
            "rmse": float(counterfactual["rmse"]),
            "max_abs_error": float(counterfactual["max_abs_error"]),
            "served_action_teacher_dependency_count": teacher_dependency_count,
            "action_selected_before_teacher_diagnostic": (
                role == "reference" or action_selected_first_count == len(trace)
            ),
        },
        "actor_feature_names": list(actor_names),
        "observation_feature_names": list(full_names),
        "event_contract_id": EVENT_CONTRACT,
        "binary_phase_fsm_mode": "disabled",
        "grf_mode": str(final_info.get("grf_mode", "")),
        "online_grf_applied_sides": list(
            final_info.get("online_grf_applied_sides", [])
        ),
        "prescribed_teacher_used_for_dynamics": role == "reference",
        "prescribed_teacher_used_for_candidate_action": False,
        "ppo_updates": 0,
        "critic_updates": 0,
        "protected_trials_opened": [],
        "preregistered_provenance": {
            "status": "PASS_V2_ROLLOUT_PROVENANCE",
            "passed": True,
            "teacher_kind": "same_trial_prescribed_prosthetic_ik",
            "teacher_lookahead_s": 0.0,
            "candidate_action_selected_before_teacher_diagnostic": (
                role == "reference" or action_selected_first_count == len(trace)
            ),
        },
    }
    arrays = {
        "observations": observation_array,
        "prescribed_actions": label_array,
        "served_means": served_mean_array,
        "served_actions": np.ascontiguousarray(served_actions, dtype=np.float32),
        "times": np.asarray([row["time_s"] for row in trace], dtype=np.float64),
        "actor_feature_names": np.asarray(actor_names, dtype="U64"),
    }
    arrays_path = output_dir / "rollout_arrays.npz"
    v1._write_npz_exclusive(arrays_path, **arrays)
    trace_path = gates.write_json_exclusive(output_dir / "trace.json", trace)
    summary_path = gates.write_json_exclusive(output_dir / "summary.json", summary)
    gates.write_json_exclusive(
        output_dir / "receipt.json",
        {
            "schema_version": 2,
            "status": "ROLLOUT_COMPLETE_PENDING_GATE",
            "passed": None,
            "artifacts": {
                "arrays": source_record(arrays_path),
                "trace": source_record(trace_path),
                "summary": source_record(summary_path),
            },
            "ppo_updates": 0,
            "critic_updates": 0,
            "protected_trials_opened": [],
        },
    )
    return summary


def rollout_worker(
    *,
    trial_id: str,
    selection: str,
    role: str,
    output_dir: str | Path,
    qualification: bool,
) -> dict[str, Any]:
    verify_lock()
    destination = _claim_empty_destination(output_dir)
    candidate = None
    if role == "candidate":
        adaptation_receipt = _strict_mapping(RUN_ROOT / "adaptation" / "receipt.json")
        if adaptation_receipt.get("passed") is not True:
            raise H0PrimarySplitV2Error("adaptation receipt has not passed")
        candidate = _path_from_record(
            adaptation_receipt.get("candidate_module_state"),
            "candidate module state",
        ).parent
    return _run_rollout(
        trial_id=trial_id,
        selection=selection,
        role=role,
        output_dir=destination,
        qualification=qualification,
        candidate_module=candidate,
    )


def _worker_command(*arguments: str) -> list[str]:
    return [sys.executable, str(Path(__file__).resolve()), *arguments]


def _parallel(commands: Sequence[list[str]]) -> None:
    processes = [subprocess.Popen(command, cwd=REPO_ROOT) for command in commands]
    deadline = time.monotonic() + WORKER_TIMEOUT_S
    failure: str | None = None
    try:
        while True:
            returncodes = [process.poll() for process in processes]
            failed = [code for code in returncodes if code not in (None, 0)]
            if failed:
                failure = f"worker failures: {failed}"
                break
            if all(code == 0 for code in returncodes):
                return
            if time.monotonic() >= deadline:
                failure = "worker stage timeout"
                break
            time.sleep(0.2)
    finally:
        if failure is not None:
            for process in processes:
                if process.poll() is None:
                    process.terminate()
            for process in processes:
                if process.poll() is None:
                    try:
                        process.wait(timeout=5.0)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait(timeout=5.0)
    if failure is not None:
        raise H0PrimarySplitV2Error(failure)


def _gate_rollout(directory: Path) -> dict[str, Any]:
    summary = _strict_mapping(directory / "summary.json")
    gate = gates.common_rollout_gate(summary)
    gate_path = gates.write_json_exclusive(directory / "common_gate.json", gate)
    provisional = _strict_mapping(directory / "receipt.json")
    terminal = {
        **provisional,
        "status": gate["status"],
        "passed": gate["passed"],
        "common_gate": source_record(gate_path),
    }
    gates.write_json_exclusive(directory / "terminal_receipt.json", terminal)
    return gate


def finalize_corpus(output_dir: str | Path) -> dict[str, Any]:
    verify_lock()
    destination = _claim_empty_destination(output_dir)
    import numpy as np

    all_observations: list[Any] = []
    all_actions: list[Any] = []
    all_trials: list[str] = []
    names: Any = None
    sources: dict[str, Any] = {}
    for trial_id in TRIAL_IDS:
        directory = RUN_ROOT / "collection" / f"trial_{trial_id}"
        terminal = _strict_mapping(directory / "terminal_receipt.json")
        if terminal.get("passed") is not True:
            raise H0PrimarySplitV2Error(f"teacher trial {trial_id} did not pass")
        arrays_path = _path_from_record(
            terminal.get("artifacts", {}).get("arrays"), f"trial {trial_id} arrays"
        )
        with np.load(arrays_path, allow_pickle=False) as archive:
            observations = np.asarray(archive["observations"], dtype=np.float32)
            actions = np.asarray(archive["prescribed_actions"], dtype=np.float32)
            trial_names = np.asarray(archive["actor_feature_names"], dtype=str)
        if observations.shape != (EXPECTED_STEPS, EXPECTED_ACTOR_FEATURES):
            raise H0PrimarySplitV2Error(f"trial {trial_id} observation shape drifted")
        if actions.shape != (EXPECTED_STEPS, 2):
            raise H0PrimarySplitV2Error(f"trial {trial_id} action shape drifted")
        if names is None:
            names = trial_names
        elif not np.array_equal(names, trial_names):
            raise H0PrimarySplitV2Error("actor feature order differs across trials")
        all_observations.append(observations)
        all_actions.append(actions)
        all_trials.extend([trial_id] * EXPECTED_STEPS)
        sources[trial_id] = source_record(arrays_path)
    observations = np.concatenate(all_observations, axis=0)
    actions = np.concatenate(all_actions, axis=0)
    trial_ids = np.asarray(all_trials, dtype="U2")
    training_indices = np.flatnonzero(np.isin(trial_ids, ["02", "04"])).astype(np.int64)
    validation_indices = np.flatnonzero(trial_ids == "08").astype(np.int64)
    corpus_path = destination / "corpus.npz"
    v1._write_npz_exclusive(
        corpus_path,
        observations=np.ascontiguousarray(observations, dtype=np.float32),
        actions=np.ascontiguousarray(actions, dtype=np.float32),
        actor_feature_names=np.asarray(names, dtype="U64"),
        trial_ids=trial_ids,
        training_indices=training_indices,
        validation_indices=validation_indices,
    )
    manifest = {
        "schema_version": 2,
        "status": "H0_PRIMARY_SPLIT_V2_CORPUS_FROZEN",
        "samples": int(len(observations)),
        "training_samples": int(len(training_indices)),
        "validation_samples": int(len(validation_indices)),
        "training_trials": ["02", "04"],
        "validation_trials": ["08"],
        "actor_feature_names": np.asarray(names, dtype=str).tolist(),
        "source_arrays": sources,
        "corpus": source_record(corpus_path),
        "ppo_updates": 0,
        "critic_updates": 0,
        "protected_trials_opened": [],
    }
    manifest_path = gates.write_json_exclusive(destination / "manifest.json", manifest)
    receipt = {
        "schema_version": 2,
        "status": "PASS_H0_PRIMARY_SPLIT_V2_CORPUS_FROZEN",
        "passed": True,
        "corpus": source_record(corpus_path),
        "manifest": source_record(manifest_path),
        "ppo_updates": 0,
        "critic_updates": 0,
        "protected_trials_opened": [],
    }
    gates.write_json_exclusive(destination / "receipt.json", receipt)
    return receipt


def _module_predictions(module_path: Path, observations: Any):
    import numpy as np
    import torch
    from ray.rllib.core.rl_module.rl_module import RLModule

    module = RLModule.from_checkpoint(module_path.resolve())
    module.pi.eval()
    with torch.no_grad():
        logits = module.pi(torch.as_tensor(observations, dtype=torch.float32))
    values = logits.detach().cpu().numpy().astype(np.float32)
    return values[:, :2], values[:, 2:]


def _logstd_parameters_exact(
    source_state: Mapping[str, Any], candidate_state: Mapping[str, Any]
) -> bool:
    import numpy as np

    for key in ("pi.1.weight", "pi.1.bias"):
        source = warm_start._as_numpy(source_state[key])
        candidate = warm_start._as_numpy(candidate_state[key])
        if source.shape != candidate.shape or not np.array_equal(
            source[2:], candidate[2:]
        ):
            return False
    return True


def adapt_worker(output_dir: str | Path) -> dict[str, Any]:
    verify_lock()
    destination = _claim_empty_destination(output_dir)
    import numpy as np

    corpus_receipt = _strict_mapping(RUN_ROOT / "corpus" / "receipt.json")
    if corpus_receipt.get("passed") is not True:
        raise H0PrimarySplitV2Error("corpus receipt has not passed")
    corpus_path = _path_from_record(corpus_receipt.get("corpus"), "V2 corpus")
    with np.load(corpus_path, allow_pickle=False) as archive:
        observations = np.asarray(archive["observations"], dtype=np.float32)
        actions = np.asarray(archive["actions"], dtype=np.float32)
        names = np.asarray(archive["actor_feature_names"], dtype=str)
        training_indices = np.asarray(archive["training_indices"], dtype=np.int64)
        validation_indices = np.asarray(archive["validation_indices"], dtype=np.int64)
    dataset = {
        "observations": observations,
        "actions": actions,
        "actor_feature_names": names,
    }
    report = imitation.adapt_actor(
        H0_MODULE.resolve(),
        dataset,
        destination,
        seed=FIT["seed"],
        epochs=FIT["epochs"],
        batch_size=FIT["batch_size"],
        learning_rate=FIT["learning_rate"],
        validation_fraction=FIT["validation_fraction"],
        patience=FIT["patience"],
        clip_weight=FIT["clip_weight"],
        logstd_weight=FIT["logstd_weight"],
        anchor_weight=FIT["anchor_weight"],
        freeze_logstd_head=True,
        training_indices=training_indices,
        validation_indices=validation_indices,
    )
    candidate_module = destination / "rl_module_target_adapted"
    source_predictions, source_logstd = _module_predictions(H0_MODULE, observations)
    candidate_predictions, candidate_logstd = _module_predictions(
        candidate_module, observations
    )
    source_validation = v1.split_contract.prediction_metrics(
        source_predictions[validation_indices], actions[validation_indices]
    )
    candidate_validation = v1.split_contract.prediction_metrics(
        candidate_predictions[validation_indices], actions[validation_indices]
    )
    source_rmse = float(source_validation["rmse"])
    candidate_rmse = float(candidate_validation["rmse"])
    improvement = 1.0 - candidate_rmse / source_rmse if source_rmse > 0.0 else 0.0
    source_state = warm_start.load_module_state(H0_MODULE)
    candidate_state = warm_start.load_module_state(candidate_module)
    manifest = {
        "schema_version": 1,
        "actor_feature_count": int(len(names)),
        "actor_feature_names": names.tolist(),
        "actor_digest": warm_start.actor_state_digest(candidate_state),
        "module_state_sha256": sha256_file(candidate_module / "module_state.pkl"),
        "event_contract_id": EVENT_CONTRACT,
        "candidate_id": "H0_primary_split_v2",
    }
    manifest_path = gates.write_json_exclusive(
        candidate_module / warm_start.DEFAULT_ACTOR_FEATURE_MANIFEST_NAME,
        manifest,
    )
    checks = {
        "validation_rmse": candidate_rmse <= 0.03,
        "validation_max_abs": float(candidate_validation["max_abs_error"]) <= 0.15,
        "validation_rmse_improvement": improvement >= 0.50,
        "finite_predictions": bool(np.all(np.isfinite(candidate_predictions))),
        "predictions_within_action_bounds": bool(
            np.all(np.abs(candidate_predictions) <= 1.0)
        ),
        "logstd_output_bit_exact": bool(
            np.array_equal(source_logstd, candidate_logstd)
        ),
        "logstd_parameters_bit_exact": _logstd_parameters_exact(
            source_state, candidate_state
        ),
        "actor_digest_changed": (
            warm_start.actor_state_digest(source_state)
            != warm_start.actor_state_digest(candidate_state)
        ),
        "save_reload_actor_exact": bool(report["save_reload"]["exact"]),
        "non_actor_exact_or_absent": bool(
            report["non_actor_verification"]
            in {"exact", "not_available_in_inference_only_rl_module"}
        ),
        "actor_feature_manifest_35": len(names) == EXPECTED_ACTOR_FEATURES,
    }
    passed = all(checks.values())
    gate = {
        "schema_version": 2,
        "status": "PASS_H0_PRIMARY_SPLIT_V2_OFFLINE"
        if passed
        else "FAIL_H0_PRIMARY_SPLIT_V2_OFFLINE",
        "passed": passed,
        "checks": checks,
        "source_validation": source_validation,
        "candidate_validation": candidate_validation,
        "validation_rmse_improvement_fraction": improvement,
        "fit": FIT,
        "source_actor_digest": warm_start.actor_state_digest(source_state),
        "candidate_actor_digest": warm_start.actor_state_digest(candidate_state),
        "ppo_updates": 0,
        "critic_updates": 0,
        "protected_trials_opened": [],
    }
    gate_path = gates.write_json_exclusive(destination / "offline_gate.json", gate)
    receipt = {
        "schema_version": 2,
        "status": gate["status"],
        "passed": passed,
        "candidate_module_state": source_record(candidate_module / "module_state.pkl"),
        "candidate_module_ctor": source_record(
            candidate_module / "class_and_ctor_args.pkl"
        ),
        "candidate_module_metadata": source_record(candidate_module / "metadata.json"),
        "actor_feature_manifest": source_record(manifest_path),
        "adaptation_report": source_record(destination / "adaptation_report.json"),
        "offline_gate": source_record(gate_path),
        "corpus": source_record(corpus_path),
        "ppo_updates": 0,
        "critic_updates": 0,
        "protected_trials_opened": [],
    }
    gates.write_json_exclusive(destination / "receipt.json", receipt)
    if not passed:
        raise H0PrimarySplitV2Error("offline adaptation gate failed")
    return receipt


def execute() -> dict[str, Any]:
    verify_lock()
    started = time.time()
    stage = "collection"
    status = "ERROR_H0_PRIMARY_SPLIT_V2_TEACHER"
    error = None
    passed = False
    try:
        _parallel(
            [
                _worker_command(
                    "--rollout-worker",
                    "--trial",
                    trial_id,
                    "--selection",
                    "deterministic",
                    "--role",
                    "reference",
                    "--stage",
                    "collection",
                    "--output-dir",
                    str(RUN_ROOT / "collection" / f"trial_{trial_id}"),
                )
                for trial_id in TRIAL_IDS
            ]
        )
        for trial_id in TRIAL_IDS:
            gate = _gate_rollout(RUN_ROOT / "collection" / f"trial_{trial_id}")
            if gate.get("passed") is not True:
                raise H0PrimarySplitV2Error(
                    f"prescribed teacher trial {trial_id} failed"
                )
        finalize_corpus(RUN_ROOT / "corpus")

        stage = "adaptation"
        status = "FAIL_H0_PRIMARY_SPLIT_V2_OFFLINE"
        completed = subprocess.run(
            _worker_command(
                "--adapt-worker", "--output-dir", str(RUN_ROOT / "adaptation")
            ),
            cwd=REPO_ROOT,
            timeout=WORKER_TIMEOUT_S,
            check=False,
        )
        if completed.returncode != 0:
            raise H0PrimarySplitV2Error(
                f"adaptation worker exited {completed.returncode}"
            )

        stage = "qualification_reference"
        status = "ERROR_H0_PRIMARY_SPLIT_V2_REFERENCE"
        reference_commands: list[list[str]] = []
        for trial_id in TRIAL_IDS:
            for selection in SELECTIONS:
                reference_commands.append(
                    _worker_command(
                        "--rollout-worker",
                        "--trial",
                        trial_id,
                        "--selection",
                        selection,
                        "--role",
                        "reference",
                        "--stage",
                        "qualification",
                        "--output-dir",
                        str(
                            RUN_ROOT
                            / "qualification"
                            / "reference"
                            / f"trial_{trial_id}_{selection}"
                        ),
                    )
                )
        _parallel(reference_commands)
        for trial_id in TRIAL_IDS:
            for selection in SELECTIONS:
                directory = (
                    RUN_ROOT
                    / "qualification"
                    / "reference"
                    / f"trial_{trial_id}_{selection}"
                )
                if _gate_rollout(directory).get("passed") is not True:
                    raise H0PrimarySplitV2Error(
                        f"qualification reference {trial_id}/{selection} failed"
                    )

        stage = "qualification_candidate"
        status = "FAIL_H0_PRIMARY_SPLIT_V2_CLOSED_LOOP"
        candidate_commands: list[list[str]] = []
        for trial_id in TRIAL_IDS:
            for selection in SELECTIONS:
                candidate_commands.append(
                    _worker_command(
                        "--rollout-worker",
                        "--trial",
                        trial_id,
                        "--selection",
                        selection,
                        "--role",
                        "candidate",
                        "--stage",
                        "qualification",
                        "--output-dir",
                        str(
                            RUN_ROOT
                            / "qualification"
                            / "candidate"
                            / f"trial_{trial_id}_{selection}"
                        ),
                    )
                )
        _parallel(candidate_commands)
        pair_gates: dict[str, Any] = {}
        for trial_id in TRIAL_IDS:
            for selection in SELECTIONS:
                key = f"trial_{trial_id}_{selection}"
                reference_dir = RUN_ROOT / "qualification" / "reference" / key
                candidate_dir = RUN_ROOT / "qualification" / "candidate" / key
                _gate_rollout(candidate_dir)
                gate = gates.condition_matched_gate(
                    _strict_mapping(reference_dir / "summary.json"),
                    _strict_mapping(candidate_dir / "summary.json"),
                )
                gates.write_json_exclusive(
                    RUN_ROOT / "qualification" / "gates" / f"{key}.json",
                    gate,
                )
                pair_gates[key] = gate
                if gate.get("passed") is not True:
                    raise H0PrimarySplitV2Error(f"candidate qualification {key} failed")
        gates.write_json_exclusive(
            RUN_ROOT / "qualification" / "receipt.json",
            {
                "schema_version": 2,
                "status": "PASS_H0_PRIMARY_SPLIT_V2_CLOSED_LOOP",
                "passed": True,
                "condition_gate_sha256": {
                    key: source_record(
                        RUN_ROOT / "qualification" / "gates" / f"{key}.json"
                    )
                    for key in pair_gates
                },
                "ppo_updates": 0,
                "critic_updates": 0,
                "protected_trials_opened": [],
            },
        )
        stage = "closed_loop_complete"
        status = "PASS_H0_PRIMARY_SPLIT_V2_CLOSED_LOOP"
        passed = True
    except Exception as exc:
        error = f"{type(exc).__name__}: {exc}"
    ledger = {
        "schema_version": 2,
        "status": status,
        "passed": passed,
        "terminal_stage": stage,
        "error": error,
        "started_unix_s": started,
        "completed_unix_s": time.time(),
        "execution_lock": source_record(LOCK),
        "corpus_frozen": (RUN_ROOT / "corpus" / "receipt.json").is_file(),
        "candidate_created": (RUN_ROOT / "adaptation" / "receipt.json").is_file(),
        "qualification_completed": (
            RUN_ROOT / "qualification" / "receipt.json"
        ).is_file(),
        "actor_update_candidates": int(
            (RUN_ROOT / "adaptation" / "adaptation_report.json").is_file()
        ),
        "ppo_updates": 0,
        "critic_updates": 0,
        "protected_trials_opened": [],
        "next_stage": (
            "ZERO_ITER_FRESH_CRITIC_OPTIMIZER_PORT"
            if passed
            else "STOP_WITHOUT_RETRY_OR_RETUNING"
        ),
    }
    gates.write_json_exclusive(RUN_ROOT / "execution_ledger.json", ledger)
    print(json.dumps(ledger, indent=2, sort_keys=True, allow_nan=False), flush=True)
    if not passed:
        raise H0PrimarySplitV2Error(error or status)
    return ledger


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--rollout-worker", action="store_true")
    mode.add_argument("--layout-preflight", action="store_true")
    mode.add_argument("--finalize-corpus", action="store_true")
    mode.add_argument("--adapt-worker", action="store_true")
    mode.add_argument("--execute", action="store_true")
    parser.add_argument("--trial", choices=TRIAL_IDS)
    parser.add_argument("--selection", choices=SELECTIONS)
    parser.add_argument("--role", choices=("reference", "candidate"))
    parser.add_argument("--stage", choices=("collection", "qualification"))
    parser.add_argument("--output-dir")
    parser.add_argument("--output")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    if args.execute:
        execute()
    elif args.layout_preflight:
        if not args.output:
            raise H0PrimarySplitV2Error("--output is required")
        layout_preflight(args.output)
    elif args.finalize_corpus:
        if not args.output_dir:
            raise H0PrimarySplitV2Error("--output-dir is required")
        finalize_corpus(args.output_dir)
    elif args.adapt_worker:
        if not args.output_dir:
            raise H0PrimarySplitV2Error("--output-dir is required")
        adapt_worker(args.output_dir)
    else:
        if not all(
            (args.trial, args.selection, args.role, args.stage, args.output_dir)
        ):
            raise H0PrimarySplitV2Error("rollout worker arguments are incomplete")
        rollout_worker(
            trial_id=args.trial,
            selection=args.selection,
            role=args.role,
            output_dir=args.output_dir,
            qualification=args.stage == "qualification",
        )
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"{type(exc).__name__}: {exc}", file=sys.stderr, flush=True)
        raise SystemExit(2) from exc
