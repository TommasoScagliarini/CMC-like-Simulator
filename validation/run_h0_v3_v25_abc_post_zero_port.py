"""Execute the frozen V3 zero-port actor through V25 A/B/C qualification.

This file is intentionally a new branch.  It consumes but never edits the V3
and zero-update closures, performs no update or training, and uses the exact
six frozen qualification noise tapes.  A and B are independent closed-loop
runs; B is never action-injected.  C uses the same innovation tape and remains
closed-loop with V25 active.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import os
import subprocess
import sys
import time
import traceback
from pathlib import Path, PurePosixPath
from typing import Any, Mapping, Sequence


REPO_ROOT = Path(__file__).resolve().parents[1]
VALIDATION_ROOT = REPO_ROOT / "validation"
TRAJECTORY_ROOT = REPO_ROOT / "Trajectory Generator"
BASELINE_ROOT = TRAJECTORY_ROOT / "baseline_MLP"
for import_root in (VALIDATION_ROOT, BASELINE_ROOT, TRAJECTORY_ROOT, REPO_ROOT):
    if str(import_root) not in sys.path:
        sys.path.insert(0, str(import_root))

import compare_h0_v3_v25_abc_post_zero_port as comparator  # noqa: E402
import h0_v3_so_recovery_contract as so_recovery  # noqa: E402
import h0_v3_v25_abc_post_zero_port_contract as contract  # noqa: E402
import run_h0_v25_abc_preflight as legacy  # noqa: E402


def _resolve(relative: str) -> Path:
    pure = PurePosixPath(relative)
    if (
        not relative
        or pure.is_absolute()
        or ".." in pure.parts
        or pure.as_posix() != relative
    ):
        raise RuntimeError(f"non-canonical contract path: {relative!r}")
    return REPO_ROOT.joinpath(*pure.parts)


LOCK = _resolve(contract.LOCK_RELATIVE_PATH)
RUN_ROOT = _resolve(contract.RUN_ROOT_RELATIVE_PATH)
INPUT_PATHS = {
    key: _resolve(relative) for key, relative in contract.INPUT_RELATIVE_PATHS.items()
}
SOURCE_PATHS = {
    key: _resolve(relative) for key, relative in contract.SOURCE_RELATIVE_PATHS.items()
}
MODULE_DIR = INPUT_PATHS["zero_port_module_state"].parent
CONFIG_PATH = INPUT_PATHS["zero_port_config"]
WORKER_TIMEOUT_S = 2400.0
SO_COUNTER_KEYS = tuple(comparator.SUMMARY_COUNTERS)


class PostZeroPortExecutionError(RuntimeError):
    """Raised on any terminal post-zero-port protocol failure."""


def sha256_file(path: str | Path) -> str:
    digest = hashlib.sha256()
    with Path(path).expanduser().resolve().open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def repo_relative(path: str | Path) -> str:
    resolved = Path(path).expanduser().resolve()
    try:
        relative = resolved.relative_to(REPO_ROOT.resolve()).as_posix()
    except ValueError as exc:
        raise PostZeroPortExecutionError(f"path outside repository: {resolved}") from exc
    return relative


def source_record(path: str | Path) -> dict[str, Any]:
    resolved = Path(path).expanduser().resolve()
    if not resolved.is_file() or resolved.is_symlink():
        raise PostZeroPortExecutionError(f"required file is missing: {resolved}")
    return {
        "path": repo_relative(resolved),
        "sha256": sha256_file(resolved),
        "size_bytes": resolved.stat().st_size,
    }


def _verified_record(record: Any, expected: Path, label: str) -> None:
    if not isinstance(record, Mapping) or set(record) != {
        "path",
        "sha256",
        "size_bytes",
    }:
        raise PostZeroPortExecutionError(f"{label} record is malformed")
    if source_record(expected) != dict(record):
        raise PostZeroPortExecutionError(f"{label} record drifted")


def _strict_mapping(path: Path) -> dict[str, Any]:
    value = comparator.strict_json_load(path)
    if not isinstance(value, Mapping):
        raise PostZeroPortExecutionError(f"expected JSON object: {path}")
    return dict(value)


def condition(case_id: str) -> dict[str, Any]:
    matches = [item for item in contract.CASES if item["case_id"] == case_id]
    if len(matches) != 1:
        raise PostZeroPortExecutionError(f"unknown condition: {case_id}")
    return copy.deepcopy(matches[0])


def destination(mode: str, case_id: str) -> Path:
    if mode not in contract.MODES:
        raise PostZeroPortExecutionError(f"unknown mode: {mode}")
    condition(case_id)
    return RUN_ROOT / f"{mode}_{case_id}"


def verify_lock(*, require_run_root_absent: bool = False) -> dict[str, Any]:
    lock = _strict_mapping(LOCK)
    expected_keys = {
        "schema_version",
        "status",
        "protocol_id",
        "revision",
        "so_policy_id",
        "run_root",
        "cases",
        "modes",
        "matrix",
        "authority",
        "gates",
        "sources",
        "inputs",
        "actor_updates",
        "critic_updates",
        "ppo_updates",
        "protected_trials_opened",
    }
    if set(lock) != expected_keys:
        raise PostZeroPortExecutionError("execution lock schema drifted")
    if (
        lock.get("schema_version") != contract.SCHEMA_VERSION
        or lock.get("status") != contract.LOCK_STATUS
        or lock.get("protocol_id") != contract.PROTOCOL_ID
        or lock.get("revision") != contract.REVISION
        or lock.get("so_policy_id") != contract.SO_POLICY_ID
        or lock.get("run_root") != contract.RUN_ROOT_RELATIVE_PATH
        or lock.get("cases") != list(contract.CASES)
        or lock.get("modes") != contract.MODES
        or lock.get("authority") != contract.AUTHORITY
        or lock.get("actor_updates") != 0
        or lock.get("critic_updates") != 0
        or lock.get("ppo_updates") != 0
        or lock.get("protected_trials_opened") != []
    ):
        raise PostZeroPortExecutionError("execution lock identity/authority drifted")
    matrix = lock.get("matrix")
    expected_destinations = [
        repo_relative(destination(mode, item["case_id"]))
        for mode in contract.MODES
        for item in contract.CASES
    ]
    if (
        not isinstance(matrix, Mapping)
        or matrix.get("rollout_count") != 18
        or matrix.get("destinations") != expected_destinations
    ):
        raise PostZeroPortExecutionError("execution matrix drifted")
    for label, paths in (("sources", SOURCE_PATHS), ("inputs", INPUT_PATHS)):
        records = lock.get(label)
        if not isinstance(records, Mapping) or set(records) != set(paths):
            raise PostZeroPortExecutionError(f"lock {label} closure drifted")
        for key, path in paths.items():
            _verified_record(records[key], path, f"{label}.{key}")
    if require_run_root_absent and os.path.lexists(RUN_ROOT):
        raise PostZeroPortExecutionError(f"run root already claimed: {RUN_ROOT}")
    return lock


def build_env_config(*, mode: str, case: Mapping[str, Any]) -> dict[str, Any]:
    translated = {
        "id": case["case_id"],
        "action_selection": case["action_selection"],
        "offset_s": case["offset_s"],
        "seed": case["seed"],
    }
    previous_config = legacy.H0_CONFIG
    previous_profile = legacy.V25_PROFILE
    previous_analog = legacy.ANALOG_PROFILE
    try:
        legacy.H0_CONFIG = CONFIG_PATH
        legacy.V25_PROFILE = INPUT_PATHS["v25_profile"]
        legacy.ANALOG_PROFILE = INPUT_PATHS["analog_detector_profile"]
        result = legacy.build_env_config(case_id=mode, condition=translated)
    finally:
        legacy.H0_CONFIG = previous_config
        legacy.V25_PROFILE = previous_profile
        legacy.ANALOG_PROFILE = previous_analog
    if (
        result.get("reward", {}).get("morphology_weight") != contract.MORPHOLOGY_WEIGHT
        or result.get("phase_fsm_input_mode") != "legacy_events"
        or result.get("online_grf_applied_sides") != ["left"]
        or result.get("binary_phase_fsm_mode")
        != contract.MODES[mode]["binary_phase_fsm_mode"]
    ):
        raise PostZeroPortExecutionError("environment routing/reward contract drifted")
    return result


def _load_noise(case: Mapping[str, Any], np: Any) -> Any:
    path = INPUT_PATHS[str(case["noise_input"])]
    try:
        with np.load(path, allow_pickle=False) as archive:
            if "standard_normal" not in archive.files:
                raise PostZeroPortExecutionError("noise tape array is missing")
            values = np.asarray(archive["standard_normal"])
            seed_value = (
                np.asarray(archive["seed"]).reshape(-1).tolist()
                if "seed" in archive.files
                else []
            )
    except (OSError, ValueError) as exc:
        raise PostZeroPortExecutionError(f"invalid noise tape: {path}") from exc
    if (
        values.dtype != np.dtype("float32")
        or values.shape != (contract.EXPECTED_STEPS, 2)
        or not np.all(np.isfinite(values))
    ):
        raise PostZeroPortExecutionError("noise tape dtype/shape/finiteness drifted")
    if case["action_selection"] == "deterministic":
        if np.count_nonzero(values) != 0 or seed_value:
            raise PostZeroPortExecutionError("deterministic tape is not canonical zero")
    elif seed_value != [case["seed"]]:
        raise PostZeroPortExecutionError("stochastic noise tape seed drifted")
    return np.ascontiguousarray(values)


def _claim_destination(path: Path) -> Path:
    expected = path.resolve()
    if expected.parent != RUN_ROOT.resolve() or expected not in {
        destination(mode, item["case_id"]).resolve()
        for mode in contract.MODES
        for item in contract.CASES
    }:
        raise PostZeroPortExecutionError("worker destination is non-canonical")
    try:
        expected.mkdir()
    except FileExistsError as exc:
        raise PostZeroPortExecutionError(f"destination already claimed: {expected}") from exc
    return expected


def _require_predecessors(mode: str, case_id: str) -> None:
    case_ids = [item["case_id"] for item in contract.CASES]
    index = case_ids.index(case_id)
    if mode == "A" and index > 0:
        previous = RUN_ROOT / "gates" / f"AB_{case_ids[index - 1]}.json"
        if _strict_mapping(previous).get("passed") is not True:
            raise PostZeroPortExecutionError("previous A/B gate did not pass")
    elif mode == "B":
        if _strict_mapping(destination("A", case_id) / "receipt.json").get(
            "passed"
        ) is not True:
            raise PostZeroPortExecutionError("condition-matched A did not pass")
    elif mode == "C":
        for required in case_ids:
            if _strict_mapping(RUN_ROOT / "gates" / f"AB_{required}.json").get(
                "passed"
            ) is not True:
                raise PostZeroPortExecutionError("all A/B gates must pass before C")
        if index > 0 and _strict_mapping(
            RUN_ROOT / "gates" / f"C_{case_ids[index - 1]}.json"
        ).get("passed") is not True:
            raise PostZeroPortExecutionError("previous C gate did not pass")


def run_worker(*, mode: str, case_id: str, output_dir: str | Path) -> dict[str, Any]:
    verify_lock()
    if mode not in contract.MODES:
        raise PostZeroPortExecutionError(f"invalid mode: {mode}")
    case = condition(case_id)
    if Path(output_dir).expanduser().resolve() != destination(mode, case_id).resolve():
        raise PostZeroPortExecutionError("worker output is outside frozen matrix")
    _require_predecessors(mode, case_id)
    output = _claim_destination(destination(mode, case_id))
    comparator.write_json_exclusive(
        output / "run_start.json",
        {
            "status": "H0_V3_V25_ROLLOUT_STARTED",
            "mode": mode,
            "case": case,
            "noise_tape": source_record(INPUT_PATHS[case["noise_input"]]),
            "actor_updates": 0,
            "critic_updates": 0,
            "ppo_updates": 0,
        },
    )

    rollout_eval, np, torch, RLModule, env_factory, _reward = (
        legacy._load_inference_stack()
    )
    np.random.seed(int(case["seed"]))
    torch.manual_seed(int(case["seed"]))
    noise = _load_noise(case, np)
    module = RLModule.from_checkpoint(MODULE_DIR.resolve())
    env_config = build_env_config(mode=mode, case=case)
    env = env_factory.make_cmc_env(env_config)
    trace: list[dict[str, Any]] = []
    raw_samples: list[dict[str, Any]] = []
    action_steps: list[dict[str, Any]] = []
    solver_journal: list[dict[str, Any]] = []
    solver_totals = {key: 0 for key in SO_COUNTER_KEYS}
    reserve = legacy._empty_accumulator()
    residual = legacy._empty_accumulator()
    sea = legacy._sea_accumulators()
    rewards: list[float] = []
    clipping = 0
    timeout_count = 0
    so_fallback_count = 0
    hard_invalid_count = 0
    terminated = False
    truncated = False
    info: Mapping[str, Any] = {}
    binary_events: dict[str, Any] | None = None
    started = time.monotonic()
    try:
        obs, reset_info = env.reset(seed=int(case["seed"]))
        obs = np.asarray(obs, dtype=np.float32)
        actor_names = tuple(str(item) for item in env.unwrapped.actor_feature_names)
        full_names = tuple(
            str(item) for item in env.unwrapped.observation_feature_names
        )
        rollout_eval._validate_module_observation_contract(
            module, actor_names, full_names
        )
        if (
            obs.shape != (contract.EXPECTED_FULL_FEATURES,)
            or obs.dtype != np.dtype("float32")
            or len(actor_names) != contract.EXPECTED_ACTOR_FEATURES
            or len(full_names) != contract.EXPECTED_FULL_FEATURES
        ):
            raise PostZeroPortExecutionError("runtime layout is not 35/84 float32")
        baseline = legacy._validate_raw_sample(
            reset_info.get("binary_phase_sensor_baseline"),
            float(reset_info.get("time")),
            "t0",
        )
        if mode == "A" and reset_info.get("binary_phase_fsm_executed") is not False:
            raise PostZeroPortExecutionError("A executed the binary FSM")
        if mode == "C":
            binary_events = legacy._binary_event_accumulator(baseline)
            reset_fsm = reset_info.get("binary_phase_fsm")
            if not isinstance(reset_fsm, Mapping) or reset_fsm.get(
                "events_this_step"
            ) != []:
                raise PostZeroPortExecutionError("C attributed an event to t0")

        for step_index in range(contract.EXPECTED_STEPS):
            obs_before = np.asarray(obs, dtype=np.float32).copy()
            stochastic = case["action_selection"] == "stochastic"
            innovation = noise[step_index] if stochastic else None
            raw, mean, std, innovation = legacy._policy_values(
                module=module,
                obs=obs_before,
                action_shape=tuple(env.action_space.shape),
                standard_normal=innovation,
                stochastic=stochastic,
                rollout_eval=rollout_eval,
            )
            raw = np.asarray(raw, dtype=np.float32)
            mean = np.asarray(mean, dtype=np.float32)
            std = np.asarray(std, dtype=np.float32)
            if (
                raw.shape != (2,)
                or mean.shape != (2,)
                or std.shape != (2,)
                or not np.all(np.isfinite(raw))
                or not np.all(np.isfinite(mean))
                or not np.all(np.isfinite(std))
                or not np.allclose(
                    std, contract.STOCHASTIC_SIGMA, rtol=0.0, atol=1.0e-8
                )
            ):
                raise PostZeroPortExecutionError("policy output contract drifted")
            applied = np.clip(
                raw, env.action_space.low, env.action_space.high
            ).astype(np.float32)
            clipping += int(np.count_nonzero(applied != raw))
            action_steps.append(
                {
                    "step": step_index + 1,
                    "standard_normal": (
                        np.asarray(innovation, dtype=np.float32).tolist()
                        if innovation is not None
                        else None
                    ),
                    "policy_mean": mean.tolist(),
                    "policy_std": std.tolist(),
                    "raw_action": raw.tolist(),
                    "applied_action": applied.tolist(),
                }
            )
            obs, reward, terminated, truncated, info = env.step(raw)
            obs = np.asarray(obs, dtype=np.float32)
            if not isinstance(info, Mapping):
                raise PostZeroPortExecutionError("environment info is malformed")
            samples = info.get("binary_phase_sensor_samples")
            if (
                not isinstance(samples, Sequence)
                or isinstance(samples, (str, bytes))
                or len(samples) != legacy.EXPECTED_SAMPLES_PER_STEP
            ):
                raise PostZeroPortExecutionError("policy step lacks ten V25 samples")
            previous = float(baseline["time_s"]) + step_index * legacy.EXPECTED_POLICY_DT_S
            raw_samples.extend(
                legacy._validate_raw_sample(
                    sample,
                    previous + sample_index * legacy.EXPECTED_SAMPLE_DT_S,
                    f"step {step_index + 1} sample {sample_index}",
                )
                for sample_index, sample in enumerate(samples, start=1)
            )
            reward_terms = info.get("reward_terms")
            if not isinstance(reward_terms, Mapping):
                raise PostZeroPortExecutionError("reward terms are missing")
            legacy._accumulate_scalar(reserve, reward_terms["reserve_norm_nm"])
            legacy._accumulate_scalar(residual, reward_terms["residual_norm_nm"])
            legacy._accumulate_sea(sea, info.get("sea_segment_diagnostics"))
            phase = info.get("phase_fsm")
            if not isinstance(phase, Mapping):
                raise PostZeroPortExecutionError("phase FSM diagnostics are missing")
            timeout_count += int(float(phase.get("timeout_exceeded", 0.0)) > 0.0)
            so = info.get("so_diagnostics")
            if not isinstance(so, Mapping) or type(so.get("solver_fallback_used")) is not bool:
                raise PostZeroPortExecutionError("SO fallback diagnostic is missing")
            so_fallback_count += int(so["solver_fallback_used"])
            raw_audit = info.get("so_solver_audit_entries")
            try:
                classified = so_recovery.classify_policy_step(
                    raw_audit, policy_id=contract.SO_POLICY_ID
                )
            except so_recovery.SORecoveryContractError as exc:
                raise PostZeroPortExecutionError(str(exc)) from exc
            if classified["counters"]["control_window_count"] != 10:
                raise PostZeroPortExecutionError("policy step does not contain 10 SO windows")
            for key in SO_COUNTER_KEYS:
                solver_totals[key] += int(classified["counters"][key])
            solver_journal.append(
                {
                    "step": step_index + 1,
                    "time_s": float(info.get("time")),
                    "control_windows": legacy._jsonable(raw_audit),
                }
            )
            if "failure" in info:
                hard_invalid_count += 1
            if mode == "C":
                legacy._accumulate_binary_events(
                    binary_events, info=info, boundary_s=float(info["time"])
                )
            rewards.append(float(reward))
            trace.append(
                legacy._trace_row(
                    step=step_index + 1,
                    obs_before=obs_before,
                    obs_after=obs,
                    raw_action=raw,
                    mean=mean,
                    std=std,
                    standard_normal=innovation,
                    applied_action=applied,
                    reward=float(reward),
                    terminated=bool(terminated),
                    truncated=bool(truncated),
                    info=info,
                )
            )
            completed = step_index + 1
            if completed == 1 or completed % 10 == 0:
                elapsed = time.monotonic() - started
                eta = elapsed / completed * (contract.EXPECTED_STEPS - completed)
                print(
                    f"[{mode}/{case_id}] {completed:3d}/500 "
                    f"elapsed={elapsed:7.1f}s eta={eta:7.1f}s",
                    flush=True,
                )
            if terminated or truncated:
                break
    finally:
        env.close()

    sea_metrics = legacy._finalize_sea(sea)
    sea_fallback_count = sum(
        int(sea[joint]["fallback_count"]) for joint in legacy.comparator.JOINTS
    )
    phase = info.get("phase_fsm", {}) if isinstance(info, Mapping) else {}
    penetrations = [float(row["reward_terms"]["grf_penetration_m"]) for row in trace]
    binary_event_gate = (
        legacy._finalize_binary_event_gate(binary_events, len(raw_samples))
        if mode == "C"
        else None
    )
    summary = {
        "schema_version": 1,
        "checkpoint_module_state_sha256": sha256_file(
            INPUT_PATHS["zero_port_module_state"]
        ),
        "condition_id": case_id,
        "action_selection": case["action_selection"],
        "seed": case["seed"],
        "episode_start_offset_s": case["offset_s"],
        "steps": len(trace),
        "episode_return": float(sum(rewards)),
        "end_reason": info.get("end_reason") if isinstance(info, Mapping) else None,
        "terminated": bool(terminated),
        "truncated": bool(truncated),
        "phase_valid_cycle_count": int(float(phase.get("valid_cycle_count", 0))),
        "phase_valid_hs_count": int(float(phase.get("valid_hs_count", 0))),
        "phase_valid_to_count": int(float(phase.get("valid_to_count", 0))),
        "invalid_event_count": int(float(phase.get("invalid_event_count", 0))),
        "grf_penetration_max_m": max(penetrations, default=0.0),
        "action_clipped_values": clipping,
        "timeout_count": timeout_count,
        "safety_stop_count": int(bool(terminated)),
        "fallback_count": so_fallback_count + sea_fallback_count,
        "so_fallback_count": so_fallback_count,
        "sea_plugin_fallback_count": sea_fallback_count,
        "hard_invalid_count": hard_invalid_count,
        "nonfinite_count": 0,
        "n_actor": len(actor_names),
        "n_observation": len(full_names),
        "observation_dtype": "float32",
        "actor_feature_names": list(actor_names),
        "observation_feature_names": list(full_names),
        "morphology_weight": env_config["reward"]["morphology_weight"],
        "episode_metrics": {
            "reserve_norm_nm": legacy._finalize_accumulator(reserve),
            "residual_norm_nm": legacy._finalize_accumulator(residual),
        },
        "sea_episode_metrics": sea_metrics,
        "projected_trace_sha256": comparator.payload_sha256(
            [
                {key: value for key, value in row.items() if not key.startswith("binary_phase_")}
                for row in trace
            ]
        ),
        "v25_raw_journal_sha256": comparator.payload_sha256(
            {"baseline": baseline, "samples": raw_samples}
        ),
        "noise_tape_sha256": sha256_file(INPUT_PATHS[case["noise_input"]]),
        "binary_phase_fsm_mode": contract.MODES[mode]["binary_phase_fsm_mode"],
        "binary_phase_event_contract_id": contract.MODES[mode][
            "binary_phase_event_contract_id"
        ],
        "binary_phase_event_gate": binary_event_gate,
        "so_policy_id": contract.SO_POLICY_ID,
        **{
            comparator.SUMMARY_COUNTERS[key]: value
            for key, value in solver_totals.items()
        },
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
    }
    common_gate = comparator.common_rollout_gate(
        summary=summary, solver_journal=solver_journal
    )
    if mode == "C" and not binary_event_gate["passed"]:
        common_gate["passed"] = False
        common_gate["status"] = "FAIL_POST_ZERO_PORT_COMMON"

    payloads = {
        "action_tape.json": {
            "schema_version": 1,
            "condition_id": case_id,
            "action_selection": case["action_selection"],
            "noise_tape": source_record(INPUT_PATHS[case["noise_input"]]),
            "steps": action_steps,
        },
        "v25_raw_journal.json": {
            "schema_version": 1,
            "sample_dt_s": legacy.EXPECTED_SAMPLE_DT_S,
            "baseline": baseline,
            "samples": raw_samples,
        },
        "solver_audit_journal.json": solver_journal,
        "trace.json": trace,
        "summary.json": summary,
        "common_gate.json": common_gate,
    }
    records = {
        name: source_record(comparator.write_json_exclusive(output / name, payload))
        for name, payload in payloads.items()
    }
    receipt = {
        "schema_version": 1,
        "status": (
            "PASS_POST_ZERO_PORT_ROLLOUT"
            if common_gate["passed"]
            else "FAIL_POST_ZERO_PORT_ROLLOUT"
        ),
        "passed": common_gate["passed"],
        "mode": mode,
        "condition_id": case_id,
        "artifacts": records,
        "execution_lock": source_record(LOCK),
        "zero_port_receipt": source_record(INPUT_PATHS["zero_port_receipt"]),
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
    }
    comparator.write_json_exclusive(output / "receipt.json", receipt)
    if not receipt["passed"]:
        raise PostZeroPortExecutionError(f"{mode}/{case_id} common gate failed")
    return receipt


def _run_worker(mode: str, case_id: str) -> None:
    completed = subprocess.run(
        [
            sys.executable,
            str(Path(__file__).resolve()),
            "--worker",
            "--mode",
            mode,
            "--condition",
            case_id,
            "--output-dir",
            str(destination(mode, case_id)),
        ],
        cwd=REPO_ROOT,
        check=False,
        timeout=WORKER_TIMEOUT_S,
    )
    if completed.returncode != 0:
        raise PostZeroPortExecutionError(
            f"worker {mode}/{case_id} exited {completed.returncode}"
        )


def execute_protocol() -> dict[str, Any]:
    verify_lock(require_run_root_absent=True)
    RUN_ROOT.parent.mkdir(parents=True, exist_ok=True)
    RUN_ROOT.mkdir()
    (RUN_ROOT / "gates").mkdir()
    started = time.time()
    ab_results: dict[str, Any] = {}
    c_results: dict[str, Any] = {}
    terminal = contract.FAIL_STATUS
    try:
        for item in contract.CASES:
            case_id = item["case_id"]
            _run_worker("A", case_id)
            _run_worker("B", case_id)
            a_dir = destination("A", case_id)
            b_dir = destination("B", case_id)
            result = comparator.compare_ab(
                a_trace=comparator.strict_json_load(a_dir / "trace.json"),
                b_trace=comparator.strict_json_load(b_dir / "trace.json"),
                a_summary=_strict_mapping(a_dir / "summary.json"),
                b_summary=_strict_mapping(b_dir / "summary.json"),
                a_v25_journal=_strict_mapping(a_dir / "v25_raw_journal.json"),
                b_v25_journal=_strict_mapping(b_dir / "v25_raw_journal.json"),
                a_solver_journal=comparator.strict_json_load(
                    a_dir / "solver_audit_journal.json"
                ),
                b_solver_journal=comparator.strict_json_load(
                    b_dir / "solver_audit_journal.json"
                ),
            )
            comparator.write_json_exclusive(
                RUN_ROOT / "gates" / f"AB_{case_id}.json", result
            )
            ab_results[case_id] = result
            if not result["passed"]:
                raise PostZeroPortExecutionError(f"A/B failed for {case_id}")
        for item in contract.CASES:
            case_id = item["case_id"]
            _run_worker("C", case_id)
            a_dir = destination("A", case_id)
            c_dir = destination("C", case_id)
            result = comparator.gate_c(
                a_summary=_strict_mapping(a_dir / "summary.json"),
                c_summary=_strict_mapping(c_dir / "summary.json"),
                c_solver_journal=comparator.strict_json_load(
                    c_dir / "solver_audit_journal.json"
                ),
            )
            comparator.write_json_exclusive(
                RUN_ROOT / "gates" / f"C_{case_id}.json", result
            )
            c_results[case_id] = result
            if not result["passed"]:
                raise PostZeroPortExecutionError(f"C failed for {case_id}")
        passed = True
        terminal = contract.PASS_STATUS
        error = None
    except Exception as exc:
        passed = False
        error = f"{type(exc).__name__}: {exc}"
    ledger = {
        "schema_version": 1,
        "status": terminal,
        "passed": passed,
        "error": error,
        "started_unix_s": started,
        "completed_unix_s": time.time(),
        "execution_lock": source_record(LOCK),
        "ab_results": ab_results,
        "c_results": c_results,
        "rollout_count_completed": sum(
            int((destination(mode, item["case_id"]) / "receipt.json").is_file())
            for mode in contract.MODES
            for item in contract.CASES
        ),
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "runtime_promoted": False,
        "next_stage": contract.TERMINAL["next_on_pass" if passed else "next_on_fail"],
    }
    comparator.write_json_exclusive(RUN_ROOT / "execution_ledger.json", ledger)
    if not passed:
        raise PostZeroPortExecutionError(error or terminal)
    return ledger


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--worker", action="store_true")
    mode.add_argument("--execute", action="store_true")
    parser.add_argument("--mode", choices=tuple(contract.MODES))
    parser.add_argument(
        "--condition", choices=tuple(item["case_id"] for item in contract.CASES)
    )
    parser.add_argument("--output-dir")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        if args.worker:
            if not args.mode or not args.condition or not args.output_dir:
                raise PostZeroPortExecutionError(
                    "--worker requires mode, condition and output-dir"
                )
            result = run_worker(
                mode=args.mode,
                case_id=args.condition,
                output_dir=args.output_dir,
            )
        else:
            result = execute_protocol()
    except Exception as exc:
        if args.worker and args.output_dir:
            output = Path(args.output_dir).expanduser().resolve()
            if output.is_dir() and not (output / "failure.json").exists():
                try:
                    comparator.write_json_exclusive(
                        output / "failure.json",
                        {
                            "status": "FAIL_CLOSED",
                            "error": f"{type(exc).__name__}: {exc}",
                            "traceback": traceback.format_exc(),
                            "actor_updates": 0,
                            "critic_updates": 0,
                            "ppo_updates": 0,
                        },
                    )
                except Exception:
                    pass
        print(f"post-zero-port A/B/C failed: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 2
    print(json.dumps(result, indent=2, sort_keys=True, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
