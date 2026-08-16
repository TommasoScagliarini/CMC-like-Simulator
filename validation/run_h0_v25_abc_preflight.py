"""Zero-update H0/V25 A/B/C full-environment preflight.

This driver never trains, restores an optimizer, opens protected trials, or
modifies the primary GRF.  It executes the frozen H0 RLModule for exactly 500
policy steps per rollout, records a complete typed trace, replays an explicit
standard-normal innovation tape, and delegates all scientific classification
to ``compare_h0_v25_abc.py``.

The supervisor order is terminal and one-shot: all six paired A/B units first;
only after every pair passes may the six condition-matched C rollouts run.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
import os
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

import compare_h0_v25_abc as comparator  # noqa: E402


PROTOCOL_LOCK = VALIDATION_ROOT / "h0_v25_abc_protocol_corrected_lock.json"
PROTOCOL_LOCK_SHA256 = (
    "04ae8e209ccae05075b625f89ac827b145d5149e4237fe2128b1c822d105fe8b"
)
EXECUTION_LOCK = VALIDATION_ROOT / "h0_v25_abc_execution_unlock.json"
H0_RUN_ROOT = (
    VALIDATION_ROOT
    / "h0_v25_abc_runs"
    / "2026-08-05_h0_v25_abc_full_environment_preflight"
)
H0_RUN = (
    VALIDATION_ROOT
    / "critic_warmup"
    / "2026-07-13_markov35_phase_aligned_sigma0005_iter1_retry"
)
H0_MODULE = H0_RUN / "rl_module_last"
H0_CONFIG = H0_RUN / "training_cfg.resolved.yaml"
ACTOR_LAYOUT_REFERENCE = H0_RUN / "actor_transplant_report.json"
FULL_LAYOUT_REFERENCE = (
    VALIDATION_ROOT
    / "controller_memory_ablation"
    / "2026-07-13_markov35_corrected_full_sigma0005_seed123"
    / "rollout_summary.json"
)
V25_PROFILE = (
    VALIDATION_ROOT
    / "binary_phase_detector_v25_geometry_runs"
    / "2026-08-04_local_reach_sweep_dev02_04_08"
    / "selected_candidate_profile.json"
)
ANALOG_PROFILE = (
    REPO_ROOT
    / "online_grf_profiles"
    / "AB06_SEASEA_stiff321_500_pi_grf_detector_HS-TO.json"
)
V25_SHADOW_CONTRACT = "binary_point_v25+functional_contact_fsm_v1_shadow"
V25_ACTIVE_CONTRACT = "binary_point_v25+functional_contact_fsm_v1"
EXPECTED_SIGMA = 0.005
EXPECTED_STEPS = 500
EXPECTED_SAMPLE_DT_S = 0.001
EXPECTED_POLICY_DT_S = 0.010
EXPECTED_SAMPLES_PER_STEP = 10
EXPECTED_ACTOR_FEATURES = 35
EXPECTED_FULL_FEATURES = 84
WORKER_TIMEOUT_S = 1800.0

CONDITIONS: tuple[dict[str, Any], ...] = (
    {
        "id": "det_minus020",
        "action_selection": "deterministic",
        "offset_s": 1.756870983805102,
        "seed": 123,
    },
    {
        "id": "det_nominal",
        "action_selection": "deterministic",
        "offset_s": 1.956870983805102,
        "seed": 123,
    },
    {
        "id": "det_plus020",
        "action_selection": "deterministic",
        "offset_s": 2.156870983805102,
        "seed": 123,
    },
    {
        "id": "stoch_nominal_seed123",
        "action_selection": "stochastic",
        "offset_s": 1.956870983805102,
        "seed": 123,
    },
    {
        "id": "stoch_nominal_seed124",
        "action_selection": "stochastic",
        "offset_s": 1.956870983805102,
        "seed": 124,
    },
    {
        "id": "stoch_nominal_seed125",
        "action_selection": "stochastic",
        "offset_s": 1.956870983805102,
        "seed": 125,
    },
)

CASE_CONFIG = {
    "A": {
        "binary_phase_fsm_mode": "disabled",
        "binary_phase_event_contract_id": V25_SHADOW_CONTRACT,
    },
    "B": {
        "binary_phase_fsm_mode": "binary_shadow",
        "binary_phase_event_contract_id": V25_SHADOW_CONTRACT,
    },
    "C": {
        "binary_phase_fsm_mode": "binary_active",
        "binary_phase_event_contract_id": V25_ACTIVE_CONTRACT,
    },
}


class H0V25ExecutionError(RuntimeError):
    """Raised on any terminal protocol/runtime failure."""


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


def _condition(condition_id: str) -> dict[str, Any]:
    matches = [item for item in CONDITIONS if item["id"] == condition_id]
    if len(matches) != 1:
        raise H0V25ExecutionError(f"unknown condition {condition_id!r}")
    return copy.deepcopy(matches[0])


def _strict_mapping(path: str | Path) -> dict[str, Any]:
    value = comparator.strict_json_load(path)
    if not isinstance(value, Mapping):
        raise H0V25ExecutionError(f"expected JSON object: {path}")
    return dict(value)


def _jsonable(value: Any) -> Any:
    """Convert NumPy/tuple diagnostics and reject all non-finite values."""

    if value is None or isinstance(value, (str, bool, int)):
        return value
    if isinstance(value, float):
        if not math.isfinite(value):
            raise H0V25ExecutionError("non-finite float in rollout trace")
        return value
    if isinstance(value, Mapping):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(item) for item in value]
    if hasattr(value, "tolist"):
        return _jsonable(value.tolist())
    if hasattr(value, "item"):
        return _jsonable(value.item())
    raise H0V25ExecutionError(
        f"unsupported diagnostic type {type(value).__name__}"
    )


def _load_inference_stack():
    import rollout_eval

    rollout_eval._load_inference_stack()
    return (
        rollout_eval,
        rollout_eval.np,
        rollout_eval.torch,
        rollout_eval.RLModule,
        rollout_eval.env_factory,
        rollout_eval.reward_function,
    )


def build_env_config(
    *,
    case_id: str,
    condition: Mapping[str, Any],
) -> dict[str, Any]:
    """Reconstruct the frozen H0 environment with only case/offset overrides."""

    if case_id not in CASE_CONFIG:
        raise H0V25ExecutionError(f"unknown case {case_id!r}")
    import training_config

    cfg = training_config.load(H0_CONFIG)
    simulation = dict(cfg.get("simulation", {}))
    grf = dict(cfg.get("grf", {}))
    reward = dict(cfg.get("reward", {}))
    if float(reward.get("morphology_weight", float("nan"))) != 0.0:
        raise H0V25ExecutionError("H0 morphology_weight must remain exactly zero")
    if bool(simulation.get("random_init", True)):
        raise H0V25ExecutionError("H0 preflight requires random_init=false")
    if float(simulation.get("segment_duration", 0.0)) != EXPECTED_POLICY_DT_S:
        raise H0V25ExecutionError("H0 policy step drifted from 10 ms")
    if float(simulation.get("episode_duration", 0.0)) != 5.0:
        raise H0V25ExecutionError("H0 episode duration drifted from 5 s")
    if int(simulation.get("policy_knots", 0)) != 1:
        raise H0V25ExecutionError("H0 policy_knots drifted from one")

    env_config = {
        "setup_xml_path": simulation["setup_xml"],
        "segment_duration": float(simulation["segment_duration"]),
        "episode_duration": float(simulation["episode_duration"]),
        "episode_start_offset_s": float(condition["offset_s"]),
        "policy_knots": int(simulation["policy_knots"]),
        "action_mode": simulation["action_mode"],
        "max_delta_rad": float(simulation["max_delta_rad"]),
        "target_slew_rate_limit_rad_s": {
            "pros_knee_angle": float(
                simulation["pros_knee_target_slew_rate_limit_rad_s"]
            ),
            "pros_ankle_angle": float(
                simulation["pros_ankle_target_slew_rate_limit_rad_s"]
            ),
        },
        "enable_pros_ref_governor": bool(simulation["pros_ref_governor"]),
        "pros_ref_model": simulation["pros_ref_model"],
        "pros_ref_lpf_cutoff_hz": float(simulation["pros_ref_cutoff_hz"]),
        "pros_ref_velocity_limit_rad_s": {
            "pros_knee_angle": float(
                simulation["pros_knee_ref_velocity_limit_rad_s"]
            ),
            "pros_ankle_angle": float(
                simulation["pros_ankle_ref_velocity_limit_rad_s"]
            ),
        },
        "pros_ref_acceleration_limit_rad_s2": {
            "pros_knee_angle": float(
                simulation["pros_knee_ref_acceleration_limit_rad_s2"]
            ),
            "pros_ankle_angle": float(
                simulation["pros_ankle_ref_acceleration_limit_rad_s2"]
            ),
        },
        "pros_ref_jerk_limit_rad_s3": {
            "pros_knee_angle": float(
                simulation["pros_knee_ref_jerk_limit_rad_s3"]
            ),
            "pros_ankle_angle": float(
                simulation["pros_ankle_ref_jerk_limit_rad_s3"]
            ),
        },
        "gait_clock_enable": bool(simulation["gait_clock_enable"]),
        "actor_cyclic_phase_only": bool(simulation["actor_cyclic_phase_only"]),
        "include_reference_state_observation": bool(
            simulation["include_reference_state_observation"]
        ),
        "include_controller_state_observation": bool(
            simulation["include_controller_state_observation"]
        ),
        "include_controller_diagnostic_observation": bool(
            simulation["include_controller_diagnostic_observation"]
        ),
        "deployable_minimal_observation": bool(
            simulation["deployable_minimal_observation"]
        ),
        "imitation_initialize_to_target": bool(
            simulation["imitation_initialize_to_target"]
        ),
        "reward_reference_range_floor": float(
            simulation["reward_reference_range_floor"]
        ),
        "reward_reference_velocity_range_floor": float(
            simulation["reward_reference_velocity_range_floor"]
        ),
        "random_init": False,
        "fail_fast": True,
        "record_outputs": False,
        "save_outputs_on_close": False,
        "grf_mode": grf["grf_mode"],
        "online_grf_profile_file": grf["online_grf_profile"],
        "online_grf_detector_profile_file": str(ANALOG_PROFILE),
        "binary_phase_detector_profile_file": str(V25_PROFILE),
        "phase_fsm_input_mode": "legacy_events",
        "phase_sensor_on_threshold_n": 5.0,
        "phase_sensor_off_threshold_n": 2.0,
        "phase_sensor_dwell_s": 0.03,
        "detector_sample_dt_s": EXPECTED_SAMPLE_DT_S,
        "event_contract_id": "legacy_events_v1",
        "binary_phase_debounce_s": 0.005,
        "include_online_grf_observation": bool(grf["online_grf_observation"]),
        "critic_privileged_observation": bool(
            cfg.get("model", {}).get("asymmetric_actor_critic", False)
        ),
        "prescribed_grf_disabled_sides": list(
            grf.get("disable_prescribed_grf_side", [])
        ),
        "online_grf_applied_sides": list(grf["online_grf_applied_side"]),
        "step_wall_timeout_s": float(simulation["step_wall_timeout_s"]),
        "grf_penetration_penalty_threshold_m": float(
            simulation["grf_penetration_penalty_threshold_m"]
        ),
        "grf_penetration_termination_m": float(
            simulation["grf_penetration_termination_m"]
        ),
        "reward": reward,
        **CASE_CONFIG[case_id],
    }
    return env_config


def _validate_layout(
    *,
    module: Any,
    env: Any,
    obs: Any,
    actor_names: Sequence[str],
    full_names: Sequence[str],
    expected_actor_names: Sequence[str],
    expected_full_names: Sequence[str],
) -> dict[str, Any]:
    import numpy as np

    vector = np.asarray(obs)
    checks = {
        "actor_count": len(actor_names) == EXPECTED_ACTOR_FEATURES,
        "full_count": len(full_names) == EXPECTED_FULL_FEATURES,
        "observation_shape": tuple(vector.shape) == (EXPECTED_FULL_FEATURES,),
        "observation_dtype": vector.dtype == np.dtype("float32"),
        "actor_names_order_exact": list(actor_names) == list(expected_actor_names),
        "full_names_order_exact": list(full_names) == list(expected_full_names),
        "checkpoint_actor_count": int(getattr(module, "_n_actor", -1))
        == EXPECTED_ACTOR_FEATURES,
        "checkpoint_full_count": int(getattr(module, "_n_full", -1))
        == EXPECTED_FULL_FEATURES,
        "action_shape": tuple(env.action_space.shape) == (2,),
        "action_dtype": getattr(env.action_space, "dtype", None)
        == np.dtype("float32"),
        "binary_features_absent_from_actor": not any(
            "binary_phase" in str(name) for name in actor_names
        ),
        "binary_features_absent_from_full": not any(
            "binary_phase" in str(name) for name in full_names
        ),
    }
    if not all(checks.values()):
        raise H0V25ExecutionError(f"layout contract failed: {checks}")
    return {
        "checks": checks,
        "actor_feature_names": list(actor_names),
        "observation_feature_names": list(full_names),
        "observation_shape": list(vector.shape),
        "observation_dtype": str(vector.dtype),
        "action_shape": list(env.action_space.shape),
        "action_dtype": str(env.action_space.dtype),
    }


def layout_preflight(output: str | Path) -> dict[str, Any]:
    rollout_eval, np, torch, RLModule, env_factory, _reward_function = (
        _load_inference_stack()
    )
    protocol = _strict_mapping(PROTOCOL_LOCK)
    if sha256_file(PROTOCOL_LOCK) != PROTOCOL_LOCK_SHA256:
        raise H0V25ExecutionError("corrected protocol lock hash mismatch")
    if protocol.get("protocol_executed") is not False:
        raise H0V25ExecutionError("protocol lock unexpectedly claims execution")
    module = RLModule.from_checkpoint(H0_MODULE.resolve())
    actor_reference = _strict_mapping(ACTOR_LAYOUT_REFERENCE)
    full_reference = _strict_mapping(FULL_LAYOUT_REFERENCE)
    expected_actor = list(actor_reference["target_actor_feature_names"])
    expected_full = list(full_reference["observation_feature_names"])
    condition = _condition("det_nominal")
    case_results: dict[str, Any] = {}
    common_reset_projection: dict[str, Any] | None = None
    for case_id in ("A", "B", "C"):
        np.random.seed(int(condition["seed"]))
        torch.manual_seed(int(condition["seed"]))
        env = env_factory.make_cmc_env(
            build_env_config(case_id=case_id, condition=condition)
        )
        try:
            obs, info = env.reset(seed=int(condition["seed"]))
            base_env = env.unwrapped
            actor_names = tuple(str(item) for item in base_env.actor_feature_names)
            full_names = tuple(
                str(item) for item in base_env.observation_feature_names
            )
            rollout_eval._validate_module_observation_contract(
                module, actor_names, full_names
            )
            layout = _validate_layout(
                module=module,
                env=env,
                obs=obs,
                actor_names=actor_names,
                full_names=full_names,
                expected_actor_names=expected_actor,
                expected_full_names=expected_full,
            )
            baseline = _jsonable(info.get("binary_phase_sensor_baseline"))
            if not isinstance(baseline, Mapping) or set(baseline) != {
                "time_s",
                "left_heel_contact",
                "left_toe_contact",
            }:
                raise H0V25ExecutionError(
                    f"case {case_id} has malformed V25 t0 baseline"
                )
            if case_id == "A" and info.get("binary_phase_fsm_executed") is not False:
                raise H0V25ExecutionError("case A executed V20 during reset")
            if case_id in {"B", "C"} and info.get("binary_phase_fsm_executed") is not True:
                raise H0V25ExecutionError(f"case {case_id} did not execute V20")
            reset_projection = {
                key: _jsonable(value)
                for key, value in dict(info).items()
                if not str(key).startswith("binary_phase_")
            }
            if case_id == "A":
                common_reset_projection = reset_projection
            elif case_id == "B" and reset_projection != common_reset_projection:
                raise H0V25ExecutionError(
                    "A/B reset differs outside top-level binary diagnostics"
                )
            case_results[case_id] = {
                **layout,
                "binary_phase_sensor_baseline": baseline,
                "binary_phase_fsm_executed": info.get(
                    "binary_phase_fsm_executed"
                ),
                "reset_projection_sha256": comparator.payload_sha256(
                    reset_projection
                ),
            }
        finally:
            env.close()
    payload = {
        "schema_version": 1,
        "status": "PASS_H0_V25_FULL_ENVIRONMENT_LAYOUT_PREFLIGHT",
        "h0_executed": False,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "protocol_lock": source_record(PROTOCOL_LOCK),
        "h0_module": {
            name: source_record(H0_MODULE / name)
            for name in (
                "module_state.pkl",
                "class_and_ctor_args.pkl",
                "metadata.json",
            )
        },
        "h0_config": source_record(H0_CONFIG),
        "actor_layout_reference": source_record(ACTOR_LAYOUT_REFERENCE),
        "full_layout_reference": source_record(FULL_LAYOUT_REFERENCE),
        "cases": case_results,
    }
    comparator.write_json_exclusive(output, payload)
    return payload


def _validate_raw_sample(sample: Any, expected_time_s: float, label: str) -> dict[str, Any]:
    if not isinstance(sample, Mapping) or set(sample) != {
        "time_s",
        "left_heel_contact",
        "left_toe_contact",
    }:
        raise H0V25ExecutionError(f"{label} raw V25 schema mismatch")
    time_s = float(sample["time_s"])
    if not math.isfinite(time_s) or abs(time_s - expected_time_s) > 1e-9:
        raise H0V25ExecutionError(f"{label} raw V25 timestamp mismatch")
    heel = sample["left_heel_contact"]
    toe = sample["left_toe_contact"]
    if type(heel) is not bool or type(toe) is not bool:
        raise H0V25ExecutionError(f"{label} raw V25 contacts must be booleans")
    return {
        "time_s": time_s,
        "left_heel_contact": heel,
        "left_toe_contact": toe,
    }


def _policy_values(
    *,
    module: Any,
    obs: Any,
    action_shape: tuple[int, ...],
    standard_normal: Any | None,
    stochastic: bool,
    rollout_eval: Any,
) -> tuple[Any, Any, Any, Any]:
    if stochastic:
        if standard_normal is None:
            raise H0V25ExecutionError("stochastic rollout is missing innovation")
        raw, mean, std, _applied_noise = rollout_eval._held_stochastic_action(
            module, obs, action_shape, standard_normal
        )
    else:
        raw, mean, std, _noise = rollout_eval._policy_action_with_diagnostics(
            module,
            obs,
            action_shape,
            action_selection="deterministic",
        )
    return raw, mean, std, standard_normal


def _array_exact(left: Any, right: Any, np: Any) -> bool:
    a = np.asarray(left, dtype=np.float32)
    b = np.asarray(right, dtype=np.float32)
    return a.shape == b.shape and a.tobytes(order="C") == b.tobytes(order="C")


def _empty_accumulator() -> dict[str, float | int]:
    return {"sum_squares": 0.0, "sample_count": 0, "abs_max": 0.0}


def _accumulate_scalar(accumulator: dict[str, float | int], value: Any) -> None:
    number = float(value)
    if not math.isfinite(number):
        raise H0V25ExecutionError("non-finite episode metric")
    accumulator["sum_squares"] = float(accumulator["sum_squares"]) + number * number
    accumulator["sample_count"] = int(accumulator["sample_count"]) + 1
    accumulator["abs_max"] = max(float(accumulator["abs_max"]), abs(number))


def _finalize_accumulator(accumulator: Mapping[str, Any]) -> dict[str, Any]:
    count = int(accumulator["sample_count"])
    if count <= 0:
        raise H0V25ExecutionError("episode metric has no samples")
    sum_squares = float(accumulator["sum_squares"])
    return {
        "sample_count": count,
        "rms": math.sqrt(sum_squares / count),
        "abs_max": float(accumulator["abs_max"]),
    }


_SEA_FIELD_MAP = {
    "torque_error_nm": (
        "sample_count",
        "torque_error_sum_squares_nm2",
        "torque_error_abs_max_nm",
    ),
    "tau_spring_nm": (
        "sample_count",
        "tau_spring_sum_squares_nm2",
        "tau_spring_abs_max_nm",
    ),
    "tau_spring_rate_nm_s": (
        "tau_spring_rate_sample_count",
        "tau_spring_rate_sum_squares_nm2_s2",
        "tau_spring_rate_abs_max_nm_s",
    ),
    "motor_speed_rad_s": (
        "sample_count",
        "motor_speed_sum_squares_rad2_s2",
        "motor_speed_abs_max_rad_s",
    ),
    "motor_accel_rad_s2": (
        "sample_count",
        "motor_accel_sum_squares_rad2_s4",
        "motor_accel_abs_max_rad_s2",
    ),
    "motor_power_w": (
        "sample_count",
        "motor_power_sum_squares_w2",
        "motor_power_abs_max_w",
    ),
}


def _sea_accumulators() -> dict[str, Any]:
    return {
        joint: {
            signal: _empty_accumulator() for signal in comparator.SEA_SIGNALS
        }
        | {
            "tau_input_saturated": {"sample_count": 0, "count": 0},
            "fallback_count": 0,
        }
        for joint in comparator.JOINTS
    }


def _accumulate_sea(accumulators: dict[str, Any], payload: Any) -> None:
    if not isinstance(payload, Mapping):
        raise H0V25ExecutionError("sea_segment_diagnostics must be an object")
    joints = payload.get("joints")
    if not isinstance(joints, Mapping):
        raise H0V25ExecutionError("SEA joint diagnostics are missing")
    for joint in comparator.JOINTS:
        values = joints.get(joint)
        if not isinstance(values, Mapping):
            raise H0V25ExecutionError(f"SEA diagnostics missing {joint}")
        target = accumulators[joint]
        for signal, (count_key, sum_key, max_key) in _SEA_FIELD_MAP.items():
            count = int(round(float(values[count_key])))
            sum_squares = float(values[sum_key])
            abs_max = float(values[max_key])
            if count <= 0 or not all(
                math.isfinite(item) and item >= 0.0
                for item in (sum_squares, abs_max)
            ):
                raise H0V25ExecutionError(f"invalid SEA sufficient statistics {joint}.{signal}")
            target[signal]["sample_count"] += count
            target[signal]["sum_squares"] += sum_squares
            target[signal]["abs_max"] = max(
                target[signal]["abs_max"], abs_max
            )
        segment_count = int(round(float(values["sample_count"])))
        saturation_count = int(values["tau_input_saturation_count"])
        target["tau_input_saturated"]["sample_count"] += segment_count
        target["tau_input_saturated"]["count"] += saturation_count
        target["fallback_count"] += int(
            values["tau_input_plugin_fallback_count"]
        ) + int(values["motor_accel_plugin_fallback_count"])


def _finalize_sea(accumulators: Mapping[str, Any]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for joint in comparator.JOINTS:
        source = accumulators[joint]
        result[joint] = {
            signal: _finalize_accumulator(source[signal])
            for signal in comparator.SEA_SIGNALS
        }
        saturation = source["tau_input_saturated"]
        denominator = int(saturation["sample_count"])
        count = int(saturation["count"])
        if denominator <= 0 or count < 0 or count > denominator:
            raise H0V25ExecutionError("invalid SEA saturation aggregate")
        result[joint]["tau_input_saturated"] = {
            "sample_count": denominator,
            "count": count,
            "fraction": count / denominator,
        }
    return result


def _binary_event_accumulator(baseline: Mapping[str, Any]) -> dict[str, Any]:
    initial_contact = bool(
        baseline["left_heel_contact"] or baseline["left_toe_contact"]
    )
    return {
        "initial_contact": initial_contact,
        "expected_event": "toe_off" if initial_contact else "heel_strike",
        "identities": set(),
        "events": [],
        "duplicate_event_count": 0,
        "out_of_order_event_count": 0,
        "left_non_v25_source_count": 0,
        "fallback_count": 0,
        "hard_invalid_count": 0,
    }


def _accumulate_binary_events(
    accumulator: dict[str, Any],
    *,
    info: Mapping[str, Any],
    boundary_s: float,
) -> None:
    payload = info.get("binary_phase_fsm")
    if not isinstance(payload, Mapping):
        raise H0V25ExecutionError("C is missing the V20 payload")
    events = payload.get("events_this_step")
    if not isinstance(events, Sequence) or isinstance(events, (str, bytes)):
        raise H0V25ExecutionError("V20 events_this_step is malformed")
    for raw in events:
        if not isinstance(raw, Mapping):
            raise H0V25ExecutionError("V20 event is malformed")
        name = str(raw.get("event", ""))
        event_time = float(raw.get("event_time_s"))
        confirmed = float(raw.get("confirmed_time_s"))
        delivered = float(raw.get("delivered_time_s"))
        identity = (name, event_time, confirmed, delivered)
        if identity in accumulator["identities"]:
            accumulator["duplicate_event_count"] += 1
        accumulator["identities"].add(identity)
        if name != accumulator["expected_event"]:
            accumulator["out_of_order_event_count"] += 1
        accumulator["expected_event"] = (
            "toe_off" if name == "heel_strike" else "heel_strike"
        )
        if (
            name not in {"heel_strike", "toe_off"}
            or raw.get("source") != "binary_phase_fsm_v20"
            or raw.get("event_contract_id") != V25_ACTIVE_CONTRACT
            or not all(math.isfinite(item) for item in (event_time, confirmed, delivered))
            or abs((confirmed - event_time) - 0.005) > 1e-9
            or delivered < confirmed - 1e-9
            or delivered - confirmed > 0.010 + 1e-9
            or abs(delivered - boundary_s) > 1e-9
        ):
            accumulator["hard_invalid_count"] += 1
        if (
            len(accumulator["events"]) == 0
            and name == "toe_off"
            and raw.get("startup_partial_stance") is not True
        ):
            accumulator["hard_invalid_count"] += 1
        accumulator["events"].append(_jsonable(raw))
    active_events = info.get("online_events", [])
    if not isinstance(active_events, Sequence) or isinstance(active_events, (str, bytes)):
        raise H0V25ExecutionError("active event stream is malformed")
    for event in active_events:
        if not isinstance(event, Mapping):
            raise H0V25ExecutionError("active event is malformed")
        if str(event.get("side", "")).lower() == "left" and event.get("source") != "v25_fsm_v20":
            accumulator["left_non_v25_source_count"] += 1


def _finalize_binary_event_gate(
    accumulator: Mapping[str, Any], sample_count: int
) -> dict[str, Any]:
    result = {
        "sample_count": sample_count,
        "event_count": len(accumulator["events"]),
        "events": list(accumulator["events"]),
        "duplicate_event_count": int(accumulator["duplicate_event_count"]),
        "out_of_order_event_count": int(accumulator["out_of_order_event_count"]),
        "left_non_v25_source_count": int(accumulator["left_non_v25_source_count"]),
        "fallback_count": int(accumulator["fallback_count"]),
        "hard_invalid_count": int(accumulator["hard_invalid_count"]),
    }
    result["passed"] = bool(
        sample_count == EXPECTED_STEPS * EXPECTED_SAMPLES_PER_STEP
        and all(
            result[field] == 0
            for field in (
                "duplicate_event_count",
                "out_of_order_event_count",
                "left_non_v25_source_count",
                "fallback_count",
                "hard_invalid_count",
            )
        )
    )
    return result


def _trace_row(
    *,
    step: int,
    obs_before: Any,
    obs_after: Any,
    raw_action: Any,
    mean: Any,
    std: Any,
    standard_normal: Any | None,
    applied_action: Any,
    reward: float,
    terminated: bool,
    truncated: bool,
    info: Mapping[str, Any],
) -> dict[str, Any]:
    row = {
        "step": step,
        "observation_before_dtype": str(obs_before.dtype),
        "observation_before_shape": list(obs_before.shape),
        "observation_before_vector": obs_before.tolist(),
        "observation_after_dtype": str(obs_after.dtype),
        "observation_after_shape": list(obs_after.shape),
        "observation_after_vector": obs_after.tolist(),
        "policy_action_mean_float32": mean.tolist(),
        "policy_action_std_float32": std.tolist(),
        "standard_normal_innovation_float32": (
            standard_normal.tolist() if standard_normal is not None else None
        ),
        "raw_policy_action_float32": raw_action.tolist(),
        "applied_policy_action_float32": applied_action.tolist(),
        "reward": float(reward),
        "terminated": bool(terminated),
        "truncated": bool(truncated),
        "time": float(info.get("time")),
        "target_time": float(info.get("target_time")),
        "end_reason": info.get("end_reason"),
        "observation": _jsonable(info.get("observation", {})),
        "observation_feature_names": _jsonable(
            info.get("observation_feature_names", [])
        ),
        "reward_terms": _jsonable(info.get("reward_terms", {})),
        "policy_segment_times": _jsonable(info.get("policy_segment_times", [])),
        "policy_segment_values": _jsonable(info.get("policy_segment_values", [])),
        "policy_segment_derivatives": _jsonable(
            info.get("policy_segment_derivatives", [])
        ),
        "reference_governor_diagnostics": _jsonable(
            info.get("reference_governor_diagnostics", {})
        ),
        "imitation_target": _jsonable(info.get("imitation_target", {})),
        "sea_segment_diagnostics": _jsonable(
            info.get("sea_segment_diagnostics", {})
        ),
        "so_diagnostics": _jsonable(info.get("so_diagnostics", {})),
        "phase_fsm": _jsonable(info.get("phase_fsm", {})),
        "legacy_online_events": _jsonable(info.get("legacy_online_events", [])),
        "online_events": _jsonable(info.get("online_events", [])),
        "online_grf": _jsonable(info.get("online_grf", {})),
        "online_grf_detector": _jsonable(info.get("online_grf_detector", {})),
        "online_gait": _jsonable(info.get("online_gait", {})),
        "phase_fsm_input_mode": info.get("phase_fsm_input_mode"),
        "detector_sample_dt_s": float(info.get("detector_sample_dt_s")),
        "event_contract_id": info.get("event_contract_id"),
        "grf_mode": info.get("grf_mode"),
        "prescribed_grf_disabled_sides": _jsonable(
            info.get("prescribed_grf_disabled_sides", [])
        ),
        "online_grf_applied_sides": _jsonable(
            info.get("online_grf_applied_sides", [])
        ),
    }
    for key, value in info.items():
        if str(key).startswith("binary_phase_"):
            row[str(key)] = _jsonable(value)
    comparator.canonical_json_bytes(row)
    return row


def _claim_empty_destination(path: str | Path) -> Path:
    destination = Path(path).expanduser().resolve()
    if not destination.is_dir():
        raise H0V25ExecutionError(
            f"preallocated rollout destination is missing: {destination}"
        )
    if any(destination.iterdir()):
        raise H0V25ExecutionError(
            f"rollout destination is not empty/no-clobber: {destination}"
        )
    return destination


def _reference_tape(path: str | Path, expected_condition: str) -> dict[str, Any]:
    tape = _strict_mapping(path)
    if tape.get("condition_id") != expected_condition:
        raise H0V25ExecutionError("reference tape condition mismatch")
    if tape.get("dtype") != "float32" or tape.get("action_shape") != [2]:
        raise H0V25ExecutionError("reference tape dtype/shape mismatch")
    steps = tape.get("steps")
    if not isinstance(steps, list) or len(steps) != EXPECTED_STEPS:
        raise H0V25ExecutionError("reference tape must contain 500 steps")
    comparator.canonical_json_bytes(tape)
    return tape


def _require_pass_receipt(path: Path, expected_status: str | None = None) -> dict[str, Any]:
    receipt = _strict_mapping(path)
    if receipt.get("passed") is not True:
        raise H0V25ExecutionError(f"required predecessor did not pass: {path}")
    if expected_status is not None and receipt.get("status") != expected_status:
        raise H0V25ExecutionError(
            f"required predecessor status mismatch: {path}"
        )
    return receipt


def _enforce_worker_predecessors(
    *,
    case_id: str,
    condition_id: str,
    reference_action_tape: str | Path | None,
) -> None:
    condition_ids = [str(item["id"]) for item in CONDITIONS]
    index = condition_ids.index(condition_id)
    expected_tape = H0_RUN_ROOT / f"A_{condition_id}" / "action_tape.json"
    if case_id == "A":
        if reference_action_tape is not None:
            raise H0V25ExecutionError("case A cannot consume a reference tape")
        if index > 0:
            _require_pass_receipt(
                H0_RUN_ROOT / "gates" / f"AB_{condition_ids[index - 1]}.json",
                "PASS_AB_SHADOW_NONINTERFERENCE",
            )
        return
    if reference_action_tape is None or Path(reference_action_tape).expanduser().resolve() != expected_tape.resolve():
        raise H0V25ExecutionError(
            f"case {case_id} must consume the exact condition-matched A tape"
        )
    _require_pass_receipt(
        H0_RUN_ROOT / f"A_{condition_id}" / "receipt.json",
        "PASS_ROLLOUT_COMMON_GATE",
    )
    if case_id == "B":
        return
    for predecessor in condition_ids:
        _require_pass_receipt(
            H0_RUN_ROOT / "gates" / f"AB_{predecessor}.json",
            "PASS_AB_SHADOW_NONINTERFERENCE",
        )
    if index > 0:
        _require_pass_receipt(
            H0_RUN_ROOT / "gates" / f"C_{condition_ids[index - 1]}.json",
            "PASS_H0_V25_COMPATIBLE_CONDITION",
        )


def run_worker(
    *,
    case_id: str,
    condition_id: str,
    output_dir: str | Path,
    reference_action_tape: str | Path | None,
) -> dict[str, Any]:
    execution_lock = _verify_execution_lock(require_all_destinations_empty=False)
    destination = _claim_empty_destination(output_dir)
    condition = _condition(condition_id)
    if case_id not in CASE_CONFIG:
        raise H0V25ExecutionError(f"invalid case {case_id!r}")
    declared_destinations = execution_lock["matrix"]["destinations"]
    relative_destination = str(destination.relative_to(REPO_ROOT))
    expected_destination = str(
        (H0_RUN_ROOT / f"{case_id}_{condition_id}").relative_to(REPO_ROOT)
    )
    if (
        relative_destination != expected_destination
        or relative_destination not in declared_destinations
    ):
        raise H0V25ExecutionError(
            "worker destination is outside the frozen case/condition matrix"
        )
    _enforce_worker_predecessors(
        case_id=case_id,
        condition_id=condition_id,
        reference_action_tape=reference_action_tape,
    )
    start_receipt = {
        "status": "H0_V25_ROLLOUT_STARTED",
        "case_id": case_id,
        "condition": condition,
        "started_unix_s": time.time(),
        "no_clobber": True,
        "ppo_updates": 0,
    }
    comparator.write_json_exclusive(destination / "run_start.json", start_receipt)
    reference_tape = None
    if case_id in {"B", "C"}:
        if reference_action_tape is None:
            raise H0V25ExecutionError(f"case {case_id} requires the A action tape")
        reference_tape = _reference_tape(reference_action_tape, condition_id)
    elif reference_action_tape is not None:
        raise H0V25ExecutionError("case A must create, not consume, its tape")

    rollout_eval, np, torch, RLModule, env_factory, _reward_function = (
        _load_inference_stack()
    )
    np.random.seed(int(condition["seed"]))
    torch.manual_seed(int(condition["seed"]))
    module = RLModule.from_checkpoint(H0_MODULE.resolve())
    env_config = build_env_config(case_id=case_id, condition=condition)
    env = env_factory.make_cmc_env(env_config)
    trace: list[dict[str, Any]] = []
    journal_samples: list[dict[str, Any]] = []
    tape_steps: list[dict[str, Any]] = []
    reserve = _empty_accumulator()
    residual = _empty_accumulator()
    sea = _sea_accumulators()
    rng = np.random.default_rng(int(condition["seed"]))
    stochastic = condition["action_selection"] == "stochastic"
    if stochastic:
        generated_innovations = rng.standard_normal(
            (EXPECTED_STEPS, 2)
        ).astype(np.float32)
    else:
        generated_innovations = None
    action_shape = tuple(int(item) for item in env.action_space.shape)
    rewards: list[float] = []
    clipping_values = 0
    timeout_count = 0
    safety_stop_count = 0
    fallback_count = 0
    hard_invalid_count = 0
    terminated = False
    truncated = False
    info: dict[str, Any] = {}
    binary_events: dict[str, Any] | None = None
    started = time.monotonic()
    try:
        obs, reset_info = env.reset(seed=int(condition["seed"]))
        obs = np.asarray(obs, dtype=np.float32)
        base_env = env.unwrapped
        actor_names = tuple(str(item) for item in base_env.actor_feature_names)
        full_names = tuple(str(item) for item in base_env.observation_feature_names)
        rollout_eval._validate_module_observation_contract(
            module, actor_names, full_names
        )
        if (
            obs.shape != (EXPECTED_FULL_FEATURES,)
            or obs.dtype != np.dtype("float32")
            or len(actor_names) != EXPECTED_ACTOR_FEATURES
            or len(full_names) != EXPECTED_FULL_FEATURES
        ):
            raise H0V25ExecutionError("runtime observation layout is not 35/84 float32")
        baseline = _validate_raw_sample(
            reset_info.get("binary_phase_sensor_baseline"),
            float(reset_info.get("time")),
            "t0",
        )
        if case_id == "A" and reset_info.get("binary_phase_fsm_executed") is not False:
            raise H0V25ExecutionError("case A executed V20")
        if case_id == "C":
            binary_events = _binary_event_accumulator(baseline)
            reset_fsm = reset_info.get("binary_phase_fsm")
            if not isinstance(reset_fsm, Mapping) or reset_fsm.get("events_this_step") != []:
                raise H0V25ExecutionError("C attributed an event to t0")

        for step_index in range(EXPECTED_STEPS):
            obs_before = np.asarray(obs, dtype=np.float32).copy()
            if stochastic:
                if reference_tape is None:
                    standard_normal = generated_innovations[step_index]
                else:
                    standard_normal = np.asarray(
                        reference_tape["steps"][step_index]["standard_normal"],
                        dtype=np.float32,
                    )
            else:
                standard_normal = None
            raw, mean, std, standard_normal = _policy_values(
                module=module,
                obs=obs_before,
                action_shape=action_shape,
                standard_normal=standard_normal,
                stochastic=stochastic,
                rollout_eval=rollout_eval,
            )
            raw = np.asarray(raw, dtype=np.float32)
            mean = np.asarray(mean, dtype=np.float32)
            std = np.asarray(std, dtype=np.float32)
            if raw.shape != action_shape or mean.shape != action_shape or std.shape != action_shape:
                raise H0V25ExecutionError("policy action shape drift")
            if not np.all(np.isfinite(raw)) or not np.all(np.isfinite(mean)) or not np.all(np.isfinite(std)):
                raise H0V25ExecutionError("policy emitted non-finite values")
            if not np.allclose(std, EXPECTED_SIGMA, rtol=0.0, atol=1e-8):
                raise H0V25ExecutionError(
                    f"policy sigma drifted from {EXPECTED_SIGMA}: {std.tolist()}"
                )
            applied = np.clip(raw, env.action_space.low, env.action_space.high).astype(np.float32)
            clipping_values += int(np.count_nonzero(applied != raw))
            tape_entry = {
                "step": step_index + 1,
                "standard_normal": (
                    np.asarray(standard_normal, dtype=np.float32).tolist()
                    if standard_normal is not None
                    else None
                ),
                "policy_mean": mean.tolist(),
                "policy_std": std.tolist(),
                "raw_action": raw.tolist(),
                "applied_action": applied.tolist(),
            }
            if case_id == "B":
                expected = reference_tape["steps"][step_index]
                for field, observed in (
                    ("standard_normal", tape_entry["standard_normal"]),
                    ("policy_mean", tape_entry["policy_mean"]),
                    ("policy_std", tape_entry["policy_std"]),
                    ("raw_action", tape_entry["raw_action"]),
                    ("applied_action", tape_entry["applied_action"]),
                ):
                    if observed is None:
                        if expected[field] is not None:
                            raise H0V25ExecutionError(
                                f"B {field} mismatch before step {step_index + 1}"
                            )
                    elif not _array_exact(observed, expected[field], np):
                        raise H0V25ExecutionError(
                            f"B {field} mismatch before step {step_index + 1}"
                        )
            tape_steps.append(tape_entry)
            obs, reward, terminated, truncated, info = env.step(raw)
            obs = np.asarray(obs, dtype=np.float32)
            rewards.append(float(reward))
            if not isinstance(info, Mapping):
                raise H0V25ExecutionError("environment info must be an object")
            samples = info.get("binary_phase_sensor_samples")
            if not isinstance(samples, Sequence) or isinstance(samples, (str, bytes)) or len(samples) != EXPECTED_SAMPLES_PER_STEP:
                raise H0V25ExecutionError(
                    f"step {step_index + 1} does not contain ten V25 samples"
                )
            previous = float(baseline["time_s"]) + step_index * EXPECTED_POLICY_DT_S
            validated_samples = [
                _validate_raw_sample(
                    sample,
                    previous + sample_index * EXPECTED_SAMPLE_DT_S,
                    f"step {step_index + 1} sample {sample_index}",
                )
                for sample_index, sample in enumerate(samples, start=1)
            ]
            journal_samples.extend(validated_samples)
            reward_terms = info.get("reward_terms")
            if not isinstance(reward_terms, Mapping):
                raise H0V25ExecutionError("reward_terms are missing")
            _accumulate_scalar(reserve, reward_terms["reserve_norm_nm"])
            _accumulate_scalar(residual, reward_terms["residual_norm_nm"])
            _accumulate_sea(sea, info.get("sea_segment_diagnostics"))
            phase = info.get("phase_fsm")
            if not isinstance(phase, Mapping):
                raise H0V25ExecutionError("phase_fsm payload is missing")
            timeout_count += int(float(phase.get("timeout_exceeded", 0.0)) > 0.0)
            so = info.get("so_diagnostics")
            if not isinstance(so, Mapping) or "solver_fallback_used" not in so:
                raise H0V25ExecutionError("SO fallback diagnostic is missing")
            fallback_count += int(so["solver_fallback_used"] is True)
            if "failure" in info:
                hard_invalid_count += 1
            if case_id == "C":
                _accumulate_binary_events(
                    binary_events,
                    info=info,
                    boundary_s=float(info["time"]),
                )
            trace.append(
                _trace_row(
                    step=step_index + 1,
                    obs_before=obs_before,
                    obs_after=obs,
                    raw_action=raw,
                    mean=mean,
                    std=std,
                    standard_normal=standard_normal,
                    applied_action=applied,
                    reward=float(reward),
                    terminated=bool(terminated),
                    truncated=bool(truncated),
                    info=info,
                )
            )
            elapsed = time.monotonic() - started
            completed = step_index + 1
            if completed == 1 or completed % 10 == 0 or completed == EXPECTED_STEPS:
                eta = elapsed / completed * (EXPECTED_STEPS - completed)
                print(
                    f"[{case_id}/{condition_id}] {completed:3d}/{EXPECTED_STEPS} "
                    f"elapsed={elapsed:7.1f}s eta={eta:7.1f}s",
                    flush=True,
                )
            if terminated or truncated:
                break
    finally:
        env.close()

    steps = len(trace)
    end_reason = info.get("end_reason") if isinstance(info, Mapping) else None
    if terminated or (end_reason not in {None, "episode_time_limit"}):
        safety_stop_count = int(bool(terminated))
    sea_metrics = _finalize_sea(sea)
    fallback_count += sum(
        int(sea[joint]["fallback_count"]) for joint in comparator.JOINTS
    )
    phase = info.get("phase_fsm", {}) if isinstance(info, Mapping) else {}
    penetration_values = [
        float(row["reward_terms"]["grf_penetration_m"]) for row in trace
    ]
    action_tape = {
        "schema_version": 1,
        "condition_id": condition_id,
        "action_selection": condition["action_selection"],
        "seed": int(condition["seed"]),
        "dtype": "float32",
        "action_shape": list(action_shape),
        "standard_normal_pre_scaling": True,
        "expected_sigma": EXPECTED_SIGMA,
        "steps": tape_steps,
    }
    journal = {
        "schema_version": 1,
        "sample_dt_s": EXPECTED_SAMPLE_DT_S,
        "baseline": baseline,
        "samples": journal_samples,
    }
    projected_trace_sha = comparator.payload_sha256(
        [
            {
                key: value
                for key, value in row.items()
                if not key.startswith("binary_phase_")
            }
            for row in trace
        ]
    )
    binary_event_gate = (
        _finalize_binary_event_gate(binary_events, len(journal_samples))
        if case_id == "C"
        else None
    )
    summary = {
        "schema_version": 1,
        "checkpoint_module_state_sha256": sha256_file(
            H0_MODULE / "module_state.pkl"
        ),
        "condition_id": condition_id,
        "action_selection": condition["action_selection"],
        "seed": int(condition["seed"]),
        "episode_start_offset_s": float(condition["offset_s"]),
        "steps": steps,
        "episode_return": float(sum(rewards)),
        "end_reason": end_reason,
        "terminated": bool(terminated),
        "truncated": bool(truncated),
        "phase_valid_cycle_count": int(float(phase.get("valid_cycle_count", 0))),
        "phase_valid_hs_count": int(float(phase.get("valid_hs_count", 0))),
        "phase_valid_to_count": int(float(phase.get("valid_to_count", 0))),
        "invalid_event_count": int(float(phase.get("invalid_event_count", 0))),
        "grf_penetration_max_m": max(penetration_values, default=0.0),
        "action_clipped_values": clipping_values,
        "timeout_count": timeout_count,
        "safety_stop_count": safety_stop_count,
        "fallback_count": fallback_count,
        "hard_invalid_count": hard_invalid_count,
        "nonfinite_count": 0,
        "n_actor": len(actor_names),
        "n_observation": len(full_names),
        "observation_dtype": "float32",
        "actor_feature_names": list(actor_names),
        "observation_feature_names": list(full_names),
        "morphology_weight": float(env_config["reward"]["morphology_weight"]),
        "episode_metrics": {
            "reserve_norm_nm": _finalize_accumulator(reserve),
            "residual_norm_nm": _finalize_accumulator(residual),
        },
        "sea_episode_metrics": sea_metrics,
        "projected_trace_sha256": projected_trace_sha,
        "v25_raw_journal_sha256": comparator.payload_sha256(journal),
        "action_tape_sha256": comparator.payload_sha256(action_tape),
        "binary_phase_fsm_mode": CASE_CONFIG[case_id]["binary_phase_fsm_mode"],
        "binary_phase_event_contract_id": CASE_CONFIG[case_id][
            "binary_phase_event_contract_id"
        ],
        "binary_phase_full_trace_sha256": comparator.payload_sha256(trace),
        "binary_phase_event_gate": binary_event_gate,
    }
    common_checks = comparator.common_rollout_checks(summary)
    common_pass = all(item["status"] == "PASS" for item in common_checks)
    if case_id == "C" and not bool(binary_event_gate["passed"]):
        common_pass = False

    artifact_payloads = {
        "action_tape.json": action_tape,
        "v25_raw_journal.json": journal,
        "trace.json": trace,
        "summary.json": summary,
    }
    artifact_records: dict[str, Any] = {}
    for name, payload in artifact_payloads.items():
        path = comparator.write_json_exclusive(destination / name, payload)
        artifact_records[name] = source_record(path)
    manifest = {
        "schema_version": 1,
        "case_id": case_id,
        "condition": condition,
        "status": "ROLLOUT_ARTIFACTS_COMPLETE",
        "protocol_lock": source_record(PROTOCOL_LOCK),
        "execution_lock": source_record(EXECUTION_LOCK),
        "driver": source_record(Path(__file__)),
        "comparator": source_record(Path(comparator.__file__)),
        "artifacts": artifact_records,
        "zero_update": True,
        "ppo_updates": 0,
    }
    manifest_path = comparator.write_json_exclusive(
        destination / "manifest.json", manifest
    )
    receipt = {
        "schema_version": 1,
        "status": "PASS_ROLLOUT_COMMON_GATE" if common_pass else "FAIL_ROLLOUT_COMMON_GATE",
        "passed": common_pass,
        "case_id": case_id,
        "condition_id": condition_id,
        "common_checks": common_checks,
        "manifest": source_record(manifest_path),
        "zero_update": True,
        "ppo_updates": 0,
        "protected_trials_opened": [],
    }
    comparator.write_json_exclusive(destination / "receipt.json", receipt)
    if not common_pass:
        raise H0V25ExecutionError(
            f"{case_id}/{condition_id} failed its common rollout gate"
        )
    return receipt


def _verify_execution_lock(
    *, require_all_destinations_empty: bool = True
) -> dict[str, Any]:
    lock = _strict_mapping(EXECUTION_LOCK)
    if lock.get("status") != "H0_V25_ABC_EXECUTION_UNLOCKED_FOR_PREFLIGHT":
        raise H0V25ExecutionError("execution lock is not authoritative")
    if lock.get("scope") != "ZERO_UPDATE_H0_FULL_ENVIRONMENT_PREFLIGHT_ONLY":
        raise H0V25ExecutionError("execution lock scope drifted")
    authority = lock.get("authority")
    if not isinstance(authority, Mapping):
        raise H0V25ExecutionError("execution lock authority is malformed")
    if authority.get("h0_preflight_execution_authorized") is not True:
        raise H0V25ExecutionError("H0 preflight execution is not authorized")
    for forbidden in (
        "ppo_updates_authorized",
        "training_authorized",
        "h0_sep_authorized",
        "protected_trial_access_authorized",
        "reserve_trial_access_authorized",
        "primary_grf_modification_authorized",
        "detector_retuning_authorized",
    ):
        if authority.get(forbidden) is not False:
            raise H0V25ExecutionError(f"forbidden authority opened: {forbidden}")
    frozen = lock.get("frozen_sources")
    if not isinstance(frozen, Mapping):
        raise H0V25ExecutionError("execution lock source map is missing")
    for key, record in frozen.items():
        if not isinstance(record, Mapping):
            raise H0V25ExecutionError(f"frozen source {key} record is malformed")
        relative = record.get("path")
        if not isinstance(relative, str) or not relative:
            raise H0V25ExecutionError(f"frozen source {key} path is malformed")
        path = (REPO_ROOT / relative).resolve()
        if record.get("sha256") != sha256_file(path):
            raise H0V25ExecutionError(f"frozen source {key} drifted")
    frozen_inputs = lock.get("frozen_inputs")
    if not isinstance(frozen_inputs, Mapping):
        raise H0V25ExecutionError("execution lock input map is missing")
    for key, record in frozen_inputs.items():
        if not isinstance(record, Mapping):
            raise H0V25ExecutionError(f"frozen input {key} record is malformed")
        relative = record.get("path")
        if not isinstance(relative, str) or not relative:
            raise H0V25ExecutionError(f"frozen input {key} path is malformed")
        path = (REPO_ROOT / relative).resolve()
        if record.get("sha256") != sha256_file(path):
            raise H0V25ExecutionError(f"frozen input {key} drifted")
    preflight_receipts = lock.get("preflight_receipts")
    if not isinstance(preflight_receipts, Mapping):
        raise H0V25ExecutionError("execution lock preflight receipts are missing")
    for key in ("layout", "tests"):
        record = preflight_receipts.get(key)
        if not isinstance(record, Mapping):
            raise H0V25ExecutionError(f"preflight receipt {key} is malformed")
        relative = record.get("path")
        if not isinstance(relative, str) or not relative:
            raise H0V25ExecutionError(f"preflight receipt {key} path is malformed")
        path = (REPO_ROOT / relative).resolve()
        if record.get("sha256") != sha256_file(path):
            raise H0V25ExecutionError(f"preflight receipt {key} drifted")
    replay = lock.get("replay_contract")
    if (
        not isinstance(replay, Mapping)
        or replay.get("selected_mode") != "standard_normal_innovation_replay"
        or replay.get("B_action_injection_allowed") is not False
        or replay.get("C_reuses_only_innovations_and_remains_closed_loop") is not True
    ):
        raise H0V25ExecutionError("frozen replay contract drifted")
    expected_destinations = [
        str((H0_RUN_ROOT / f"{case}_{condition['id']}").relative_to(REPO_ROOT))
        for case in ("A", "B", "C")
        for condition in CONDITIONS
    ]
    matrix = lock.get("matrix")
    if (
        not isinstance(matrix, Mapping)
        or matrix.get("destinations") != expected_destinations
        or matrix.get("rollout_count") != 18
    ):
        raise H0V25ExecutionError("execution destination matrix drifted")
    for relative in expected_destinations:
        destination = (REPO_ROOT / relative).resolve()
        if not destination.is_dir() or (
            require_all_destinations_empty and any(destination.iterdir())
        ):
            raise H0V25ExecutionError(
                f"one-shot destination is missing or non-empty: {destination}"
            )
    return lock


def _run_worker_subprocess(
    *,
    case_id: str,
    condition_id: str,
    destination: Path,
    reference_tape: Path | None,
) -> None:
    command = [
        sys.executable,
        str(Path(__file__).resolve()),
        "--worker",
        "--case",
        case_id,
        "--condition",
        condition_id,
        "--output-dir",
        str(destination),
    ]
    if reference_tape is not None:
        command.extend(["--reference-action-tape", str(reference_tape)])
    completed = subprocess.run(
        command,
        cwd=REPO_ROOT,
        check=False,
        timeout=WORKER_TIMEOUT_S,
    )
    if completed.returncode != 0:
        raise H0V25ExecutionError(
            f"worker {case_id}/{condition_id} exited {completed.returncode}"
        )


def execute_protocol() -> dict[str, Any]:
    lock = _verify_execution_lock(require_all_destinations_empty=True)
    started = time.time()
    pair_results: dict[str, Any] = {}
    c_results: dict[str, Any] = {}
    terminal_status = "ERROR_H0_REFERENCE"
    try:
        for condition in CONDITIONS:
            condition_id = str(condition["id"])
            a_dir = H0_RUN_ROOT / f"A_{condition_id}"
            b_dir = H0_RUN_ROOT / f"B_{condition_id}"
            _run_worker_subprocess(
                case_id="A",
                condition_id=condition_id,
                destination=a_dir,
                reference_tape=None,
            )
            terminal_status = "ERROR_SHADOW_NONINTERFERENCE"
            _run_worker_subprocess(
                case_id="B",
                condition_id=condition_id,
                destination=b_dir,
                reference_tape=a_dir / "action_tape.json",
            )
            pair = comparator.compare_ab(
                a_trace=comparator.strict_json_load(a_dir / "trace.json"),
                b_trace=comparator.strict_json_load(b_dir / "trace.json"),
                a_summary=comparator.strict_json_load(a_dir / "summary.json"),
                b_summary=comparator.strict_json_load(b_dir / "summary.json"),
                a_journal=comparator.strict_json_load(a_dir / "v25_raw_journal.json"),
                b_journal=comparator.strict_json_load(b_dir / "v25_raw_journal.json"),
            )
            comparator.write_json_exclusive(
                H0_RUN_ROOT / "gates" / f"AB_{condition_id}.json", pair
            )
            pair_results[condition_id] = pair
            if not pair["passed"]:
                raise H0V25ExecutionError(
                    f"A/B bit-exact gate failed for {condition_id}"
                )

        terminal_status = "FAIL_H0_V25_COMPATIBILITY"
        for condition in CONDITIONS:
            condition_id = str(condition["id"])
            a_dir = H0_RUN_ROOT / f"A_{condition_id}"
            c_dir = H0_RUN_ROOT / f"C_{condition_id}"
            _run_worker_subprocess(
                case_id="C",
                condition_id=condition_id,
                destination=c_dir,
                reference_tape=a_dir / "action_tape.json",
            )
            gate = comparator.gate_c(
                a_summary=comparator.strict_json_load(a_dir / "summary.json"),
                c_summary=comparator.strict_json_load(c_dir / "summary.json"),
            )
            comparator.write_json_exclusive(
                H0_RUN_ROOT / "gates" / f"C_{condition_id}.json", gate
            )
            c_results[condition_id] = gate
            if not gate["passed"]:
                raise H0V25ExecutionError(
                    f"C compatibility gate failed for {condition_id}"
                )
        terminal_status = "PASS_H0_V25_COMPATIBLE"
        passed = True
        error = None
    except Exception as exc:
        passed = False
        error = f"{type(exc).__name__}: {exc}"
    ledger = {
        "schema_version": 1,
        "status": terminal_status,
        "passed": passed,
        "error": error,
        "started_unix_s": started,
        "completed_unix_s": time.time(),
        "execution_lock": source_record(EXECUTION_LOCK),
        "protocol_lock": source_record(PROTOCOL_LOCK),
        "pair_results": pair_results,
        "c_results": c_results,
        "rollout_count_completed": sum(
            int((H0_RUN_ROOT / f"{case}_{condition['id']}" / "receipt.json").is_file())
            for case in ("A", "B", "C")
            for condition in CONDITIONS
        ),
        "protocol_unit_count_completed": len(pair_results) + len(c_results),
        "ppo_updates": 0,
        "training_performed": False,
        "protected_trials_opened": [],
        "runtime_promoted": False,
        "next_stage": (
            "H0_BINARY_V25_CANDIDATE_READY"
            if passed
            else "STOP_WITHOUT_RETRY_RETUNING_OR_FALLBACK"
        ),
    }
    comparator.write_json_exclusive(H0_RUN_ROOT / "execution_ledger.json", ledger)
    print(json.dumps(ledger, indent=2, sort_keys=True, allow_nan=False))
    if not passed:
        raise H0V25ExecutionError(error or terminal_status)
    return ledger


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--layout-preflight", action="store_true")
    mode.add_argument("--worker", action="store_true")
    mode.add_argument("--execute", action="store_true")
    parser.add_argument("--output")
    parser.add_argument("--case", choices=("A", "B", "C"))
    parser.add_argument("--condition", choices=tuple(item["id"] for item in CONDITIONS))
    parser.add_argument("--output-dir")
    parser.add_argument("--reference-action-tape")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        if args.layout_preflight:
            if not args.output:
                raise H0V25ExecutionError("--layout-preflight requires --output")
            result = layout_preflight(args.output)
        elif args.worker:
            if not args.case or not args.condition or not args.output_dir:
                raise H0V25ExecutionError(
                    "--worker requires --case, --condition and --output-dir"
                )
            result = run_worker(
                case_id=args.case,
                condition_id=args.condition,
                output_dir=args.output_dir,
                reference_action_tape=args.reference_action_tape,
            )
        else:
            result = execute_protocol()
    except Exception as exc:
        if args.worker and args.output_dir:
            failure_path = Path(args.output_dir).expanduser().resolve() / "failure.json"
            if not failure_path.exists():
                try:
                    comparator.write_json_exclusive(
                        failure_path,
                        {
                            "status": "FAIL_CLOSED",
                            "error": f"{type(exc).__name__}: {exc}",
                            "traceback": traceback.format_exc(),
                            "ppo_updates": 0,
                        },
                    )
                except Exception:
                    pass
        print(
            f"H0/V25 preflight failed closed: {type(exc).__name__}: {exc}",
            file=sys.stderr,
        )
        return 2
    print(json.dumps(result, indent=2, sort_keys=True, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
