"""Physical Q3 rollout implementation independent of the Q1 runner.

This module imports the current physical/runtime primitives directly.  It does
not import any Q1 orchestration or gate except indirectly through the separate
generic artifact predicate module used elsewhere in Q3.
"""

from __future__ import annotations

import copy
import math
import sys
import time
from collections.abc import Callable, Mapping
from typing import Any

import h0_v12r5_q3_artifacts as artifacts
import h0_v12r5_q3_runtime_contract as contract
import prepare_h0_v12r5_q3_qualification_noise_tapes as noise


ROOT_VALIDATION = artifacts.REPO_ROOT / "validation"
LOCAL_VALIDATION = (
    artifacts.REPO_ROOT / "Trajectory Generator" / "baseline_MLP" / "validation"
)
V12R3_ROOT = LOCAL_VALIDATION / "v12r3"
for _root in (artifacts.REPO_ROOT, ROOT_VALIDATION, LOCAL_VALIDATION, V12R3_ROOT):
    if str(_root) not in sys.path:
        sys.path.insert(0, str(_root))


class V12R5Q3PhysicalRolloutError(RuntimeError):
    """Raised when a physical Q3 rollout violates the frozen contract."""


def _load_noise_tape(case_id: str, *, np: Any) -> tuple[Any, dict[str, Any], str]:
    manifest = noise.verify_manifest()
    case = contract.canonical_case(case_id)
    path = artifacts.resolve_relative(case["noise_tape"])
    row = manifest.get("tapes", {}).get(path.name)
    if not isinstance(row, Mapping):
        raise V12R5Q3PhysicalRolloutError(f"missing Q3 noise row: {case_id}")
    try:
        with np.load(path, allow_pickle=False) as archive:
            expected_keys = (
                {"standard_normal"}
                if row.get("seed") is None
                else {"standard_normal", "seed"}
            )
            if set(archive.files) != expected_keys:
                raise V12R5Q3PhysicalRolloutError("noise tape keys drifted")
            if row.get("seed") is not None and archive["seed"].tolist() != [
                row["seed"]
            ]:
                raise V12R5Q3PhysicalRolloutError("noise tape seed drifted")
            array = np.ascontiguousarray(archive["standard_normal"], dtype=np.float32)
    except V12R5Q3PhysicalRolloutError:
        raise
    except BaseException as exc:
        raise V12R5Q3PhysicalRolloutError("noise tape could not be read") from exc
    expected_hash = contract.EXPECTED_TAPE_ARRAY_SHA256[path.name]
    if (
        array.shape != (contract.EXPECTED_STEPS, *contract.EXPECTED_ACTION_SHAPE)
        or array.dtype != np.float32
        or not array.flags.c_contiguous
        or noise.array_sha256(array) != expected_hash
        or row.get("array_sha256") != expected_hash
        or row.get("artifact") != artifacts.record(path)
    ):
        raise V12R5Q3PhysicalRolloutError("noise tape closure drifted")
    return array, artifacts.record(path), expected_hash


def _increment(activity: dict[str, int], name: str, amount: int = 1) -> None:
    if name not in activity or type(activity[name]) is not int or amount < 0:
        raise V12R5Q3PhysicalRolloutError(f"invalid activity counter: {name}")
    activity[name] += amount


def _append_step(
    rows: list[dict[str, Any]],
    row: Mapping[str, Any],
    *,
    persist_step: Callable[[int, Mapping[str, Any]], Any] | None,
) -> None:
    expected = len(rows) + 1
    if row.get("step") != expected:
        raise V12R5Q3PhysicalRolloutError("step sequence drifted")
    if persist_step is not None:
        persist_step(expected, row)
    rows.append(copy.deepcopy(dict(row)))


def _accumulate_baseline_so_recovery(
    solver_totals: dict[str, int],
    *,
    entries: Any,
    step: int,
    selected_fallback: bool,
    policy_id: str,
    v3: Any,
    so_recovery: Any,
) -> int:
    if not isinstance(entries, list):
        raise V12R5Q3PhysicalRolloutError("baseline SO audit entries are malformed")
    classified = so_recovery.classify_policy_step(entries, policy_id=policy_id)
    counters = classified.get("counters") if isinstance(classified, Mapping) else None
    if not isinstance(counters, Mapping):
        raise V12R5Q3PhysicalRolloutError("baseline SO classification is malformed")
    for key in v3.SO_RECOVERY_COUNTER_KEYS:
        value = counters.get(key)
        if type(value) is not int or value < 0 or key not in solver_totals:
            raise V12R5Q3PhysicalRolloutError(f"baseline SO counter drifted: {key}")
        solver_totals[key] += value
    if counters.get("control_window_count") != 10:
        raise V12R5Q3PhysicalRolloutError(f"baseline SO windows drifted at step {step}")
    fallback = int(
        counters["unaccepted_hard_so_fallback_count"]
        + counters["unaccepted_bounded_ls_count"]
    )
    if bool(fallback) != selected_fallback:
        raise V12R5Q3PhysicalRolloutError(
            f"baseline SO selected fallback drifted at step {step}"
        )
    return fallback


def collect_physical_rollout(
    *,
    role: str,
    case: Mapping[str, Any],
    runtime_inputs: Mapping[str, Mapping[str, Any]],
    activity: dict[str, int],
    persist_step: Callable[[int, Mapping[str, Any]], Any] | None = None,
) -> dict[str, Any]:
    """Run exactly one frozen role/condition and return replayable evidence."""

    import numpy as np

    import h0_v3_so_recovery_contract as so_recovery
    import primary_grf_split_adaptation as split_contract
    import run_h0_primary_grf_split_v1_adaptation as v1
    import run_h0_primary_grf_split_v3_semantic_replay as v3
    import run_h0_primary_split_v6_qualification as v6_runtime
    import run_h0_primary_split_v9_causal_teacher as env_source
    import run_h0_primary_split_v12r3_autonomy_recovery as r3_runtime

    (
        rollout_eval,
        runtime_np,
        torch,
        RLModule,
        env_factory,
        legacy,
        v26_collector,
    ) = r3_runtime._load_rollout_stack()
    if runtime_np is not np:
        raise V12R5Q3PhysicalRolloutError("runtime NumPy identity drifted")
    input_name = (
        "source_h0_module" if role == contract.BASELINE_ROLE else "candidate_module"
    )
    module_record = runtime_inputs.get(input_name)
    if not isinstance(module_record, Mapping):
        raise V12R5Q3PhysicalRolloutError(f"runtime input is missing: {input_name}")
    module = RLModule.from_checkpoint(artifacts.resolve_relative(module_record["path"]))
    if hasattr(module, "eval"):
        module.eval()
    runtime_seed = int(case["runtime_seed"])
    np.random.seed(runtime_seed)
    torch.manual_seed(runtime_seed)
    if role == contract.CANDIDATE_ROLE:
        env_config = env_source.build_env_config(case)
    else:
        env_config = v6_runtime._build_env_config(
            role=role,
            case=case,
            execution={
                "inputs": {
                    "source_h0_config": runtime_inputs["source_h0_config"],
                    "analog_profile": runtime_inputs["historical_analog_profile"],
                    "v25_profile": runtime_inputs["baseline_shadow_v25_profile"],
                }
            },
            legacy=legacy,
        )
    env = env_factory.make_cmc_env(env_config)
    tape, tape_record, tape_array_sha256 = _load_noise_tape(str(case["case_id"]), np=np)
    rows: list[dict[str, Any]] = []
    actor_names: tuple[str, ...] = ()
    full_names: tuple[str, ...] = ()
    audit: dict[str, Any] | None = None
    reserve = legacy._empty_accumulator()
    residual = legacy._empty_accumulator()
    sea = legacy._sea_accumulators()
    solver_totals = v6_runtime._solver_totals(v3)
    penetrations: list[float] = []
    binary_samples = 0
    valid_cycles = 0
    invalid_events = 0
    sea_fallback_count = 0
    timeout_count = 0
    clipping_count = 0
    nonfinite_count = 0
    hard_invalid_count = 0
    info: Mapping[str, Any] = {}
    terminated = False
    truncated = False
    shadow_fsm: Any | None = None
    started = time.monotonic()
    try:
        _increment(activity, "environment_reset_calls")
        observation, reset_info = env.reset(seed=runtime_seed)
        observation = np.ascontiguousarray(observation, dtype=np.float32)
        actor_names = tuple(str(name) for name in env.unwrapped.actor_feature_names)
        full_names = tuple(
            str(name) for name in env.unwrapped.observation_feature_names
        )
        rollout_eval._validate_module_observation_contract(
            module, actor_names, full_names
        )
        if (
            observation.shape != (contract.EXPECTED_FULL_FEATURES,)
            or observation.dtype != np.float32
            or len(actor_names) != contract.EXPECTED_ACTOR_FEATURES
            or len(full_names) != contract.EXPECTED_FULL_FEATURES
            or not np.all(np.isfinite(observation))
            or not isinstance(reset_info, Mapping)
        ):
            raise V12R5Q3PhysicalRolloutError("35/84 float32 reset contract drifted")
        if role == contract.CANDIDATE_ROLE:
            audit = r3_runtime._new_physical_audit(
                reset_info=reset_info, legacy=legacy, np=np
            )
        else:
            if not v6_runtime._runtime_routing_ok(
                reset_info, role=role, require_samples=False
            ):
                raise V12R5Q3PhysicalRolloutError("baseline routing drifted at reset")
            shadow_fsm = copy.deepcopy(env.unwrapped._phase_fsm)
        current_info = dict(reset_info)
        body_weight_n = float(env.unwrapped._body_weight_n)
        for index in range(contract.EXPECTED_STEPS):
            step = index + 1
            observation_before = observation.copy()
            if role == contract.BASELINE_ROLE:
                paired = split_contract.build_paired_views(
                    observation_before,
                    actor_names,
                    current_info,
                    body_weight_n=body_weight_n,
                    reset_boundary=index == 0,
                    teacher_phase_observation=shadow_fsm.observation(),
                )
                policy_input = v1._teacher_full_observation(
                    observation_before, paired, np
                )
                actor_observation = np.ascontiguousarray(
                    paired.teacher, dtype=np.float32
                )
                _raw, mean, std = v1._policy(
                    module,
                    policy_input,
                    tuple(env.action_space.shape),
                    rollout_eval,
                )
                _increment(activity, "baseline_actor_queries")
            else:
                actor_observation = np.ascontiguousarray(
                    observation_before[: contract.EXPECTED_ACTOR_FEATURES],
                    dtype=np.float32,
                )
                mean, std = r3_runtime._query_mean_std(
                    module, actor_observation, np=np, torch=torch
                )
                _increment(activity, "candidate_actor_queries")
            mean = np.ascontiguousarray(mean, dtype=np.float32).reshape(
                contract.EXPECTED_ACTION_SHAPE
            )
            std = np.ascontiguousarray(std, dtype=np.float32).reshape(
                contract.EXPECTED_ACTION_SHAPE
            )
            if (
                not np.all(np.isfinite(mean))
                or not np.all(np.isfinite(std))
                or not np.allclose(
                    std, contract.STOCHASTIC_SIGMA, rtol=0.0, atol=1.0e-8
                )
            ):
                raise V12R5Q3PhysicalRolloutError("actor mean/logstd contract drifted")
            single_noise = np.ascontiguousarray(std * tape[index], dtype=np.float32)
            raw_action = np.ascontiguousarray(mean + single_noise, dtype=np.float32)
            applied_action = np.ascontiguousarray(
                np.clip(raw_action, env.action_space.low, env.action_space.high),
                dtype=np.float32,
            )
            clipping_count += int(np.count_nonzero(raw_action != applied_action))
            _increment(activity, "environment_step_calls")
            observation_after, reward, terminated, truncated, info = env.step(
                applied_action
            )
            observation_after = np.ascontiguousarray(
                observation_after, dtype=np.float32
            )
            if not isinstance(info, Mapping):
                raise V12R5Q3PhysicalRolloutError("rollout info is malformed")
            if role == contract.CANDIDATE_ROLE:
                physical = r3_runtime._consume_physical_step(
                    audit,
                    step=step,
                    info=info,
                    observation_before=observation_before,
                    observation_after=observation_after,
                    reward=reward,
                    action=raw_action,
                    applied_action=applied_action,
                    extra_vectors=(actor_observation, mean, std, single_noise),
                    legacy=legacy,
                    v26_collector=v26_collector,
                )
                penetration = float(physical["penetration_m"])
                reserve_value = float(physical["reserve_norm_nm"])
                residual_value = float(physical["residual_norm_nm"])
            else:
                if not v6_runtime._runtime_routing_ok(
                    info, role=role, require_samples=True
                ):
                    raise V12R5Q3PhysicalRolloutError("baseline routing drifted")
                samples = info.get("binary_phase_sensor_samples")
                if not isinstance(samples, list):
                    raise V12R5Q3PhysicalRolloutError("baseline samples are malformed")
                binary_samples += len(samples)
                reward_terms = info.get("reward_terms")
                phase = info.get("phase_fsm")
                if not isinstance(reward_terms, Mapping) or not isinstance(
                    phase, Mapping
                ):
                    raise V12R5Q3PhysicalRolloutError(
                        "baseline diagnostics are incomplete"
                    )
                reserve_value = float(reward_terms.get("reserve_norm_nm"))
                residual_value = float(reward_terms.get("residual_norm_nm"))
                penetration = float(reward_terms.get("grf_penetration_m"))
                legacy._accumulate_scalar(reserve, reserve_value)
                legacy._accumulate_scalar(residual, residual_value)
                sea_payload = info.get("sea_segment_diagnostics")
                legacy._accumulate_sea(sea, sea_payload)
                sea_step_fallback = v6_runtime._sea_fallback_count(sea_payload)
                sea_fallback_count += sea_step_fallback
                so = info.get("so_diagnostics")
                if (
                    not isinstance(so, Mapping)
                    or type(so.get("solver_fallback_used")) is not bool
                ):
                    raise V12R5Q3PhysicalRolloutError(
                        "baseline SO diagnostic is missing"
                    )
                _accumulate_baseline_so_recovery(
                    solver_totals,
                    entries=info.get("so_solver_audit_entries"),
                    step=step,
                    selected_fallback=bool(so["solver_fallback_used"]),
                    policy_id=v6_runtime.contract.SO_POLICY_ID,
                    v3=v3,
                    so_recovery=so_recovery,
                )
                hard_invalid_count += int("failure" in info)
                valid_cycles = int(float(phase.get("valid_cycle_count", 0.0)))
                invalid_events = max(
                    invalid_events,
                    int(float(phase.get("invalid_event_count", 0.0))),
                )
                timeout_count += int(float(phase.get("timeout_exceeded", 0.0)) > 0)
                v1._update_shadow_fsm(
                    shadow_fsm, info=info, body_weight_n=body_weight_n
                )
            penetrations.append(penetration)
            finite = bool(
                np.all(np.isfinite(observation_after))
                and np.all(np.isfinite(raw_action))
                and math.isfinite(float(reward))
                and math.isfinite(penetration)
                and math.isfinite(reserve_value)
                and math.isfinite(residual_value)
            )
            nonfinite_count += int(not finite)
            if not finite:
                raise V12R5Q3PhysicalRolloutError("non-finite physical rollout value")
            row = {
                "schema_version": contract.SCHEMA_VERSION,
                "protocol_id": contract.PROTOCOL_ID,
                "role": role,
                "case_id": case["case_id"],
                "actor_input_view": (
                    "historical_analog"
                    if role == contract.BASELINE_ROLE
                    else "primary_split"
                ),
                "actor_observation": actor_observation.tolist(),
                "mean_action": mean.tolist(),
                "standard_normal": tape[index].tolist(),
                "single_noise": single_noise.tolist(),
                "raw_action": raw_action.tolist(),
                "applied_action": applied_action.tolist(),
                "reward": float(reward),
                "grf_penetration_m": penetration,
                "reserve_norm_nm": reserve_value,
                "residual_norm_nm": residual_value,
                "teacher_enabled": False,
                "teacher_query_count": 0,
                "blending_enabled": False,
                "safety_latch_enabled": False,
                "terminated": bool(terminated),
                "truncated": bool(truncated),
                "end_reason": info.get("end_reason"),
            }
            _append_step(
                rows,
                {"step": step, **row},
                persist_step=persist_step,
            )
            observation = observation_after
            current_info = dict(info)
            if step == 1 or step % 25 == 0:
                elapsed = time.monotonic() - started
                eta = elapsed / step * (contract.EXPECTED_STEPS - step)
                print(
                    f"[V12R5Q3 {role}/{case['case_id']}] {step:3d}/500 "
                    f"elapsed={elapsed:7.1f}s eta={eta:7.1f}s",
                    flush=True,
                )
            if terminated or truncated:
                break
    finally:
        env.close()
    if role == contract.CANDIDATE_ROLE:
        if audit is None:
            raise V12R5Q3PhysicalRolloutError(
                "candidate physical audit was not initialized"
            )
        physical_summary = r3_runtime._physical_summary(
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
        )
    else:
        phase = info.get("phase_fsm", {}) if isinstance(info, Mapping) else {}
        sea_metrics = legacy._finalize_sea(sea)
        aggregate_sea_fallback = sum(
            int(sea_metrics[joint]["fallback_count"]) for joint in contract.JOINTS
        )
        if aggregate_sea_fallback != sea_fallback_count:
            raise V12R5Q3PhysicalRolloutError(
                "baseline SEA fallback aggregation mismatch"
            )
        so_solver_unaccepted_count = (
            solver_totals["unaccepted_hard_so_fallback_count"]
            + solver_totals["unaccepted_bounded_ls_count"]
        )
        physical_summary = {
            "steps": len(rows),
            "end_reason": info.get("end_reason"),
            "terminated": bool(terminated),
            "truncated": bool(truncated),
            "phase_valid_cycle_count": max(
                valid_cycles, int(float(phase.get("valid_cycle_count", 0.0)))
            ),
            "grf_penetration_max_m": max(penetrations, default=0.0),
            "control_window_count": solver_totals["control_window_count"],
            "raw_sensor_sample_count": binary_samples,
            "binary_phase_sensor_sample_count": binary_samples,
            "action_clipped_values": clipping_count,
            "fallback_count": so_solver_unaccepted_count + sea_fallback_count,
            "timeout_count": timeout_count,
            "sea_plugin_fallback_count": sea_fallback_count,
            "hard_invalid_count": hard_invalid_count,
            "invalid_event_count": invalid_events,
            "nonfinite_count": nonfinite_count,
            "so_solver_unaccepted_count": so_solver_unaccepted_count,
            "routing_failure_count": 0,
            "step_contract_failure_count": 0,
            "safety_stop_count": int(bool(terminated)),
            "n_actor": len(actor_names),
            "n_observation": len(full_names),
            "observation_dtype": contract.EXPECTED_DTYPE,
            "action_shape": list(contract.EXPECTED_ACTION_SHAPE),
            "action_dtype": contract.EXPECTED_DTYPE,
            "episode_metrics": {
                "reserve_norm_nm": legacy._finalize_accumulator(reserve),
                "residual_norm_nm": legacy._finalize_accumulator(residual),
            },
            "sea_episode_metrics": sea_metrics,
            "legacy_event_integrity_passed": True,
        }
    return {
        "rows": rows,
        "physical_summary": physical_summary,
        "noise_tape": tape_record,
        "noise_tape_array_sha256": tape_array_sha256,
    }


__all__ = ["V12R5Q3PhysicalRolloutError", "collect_physical_rollout"]
