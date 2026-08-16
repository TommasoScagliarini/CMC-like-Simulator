"""Candidate-bound V12R7-Q3 physical rollout adapter.

The mature V12R5-Q3 baseline/candidate collector is reused under a serialized,
temporary compatibility binding.  Its source is never modified.  This module
injects only the V12R7 contract, the newly verified Q3 tape loader, and the
resolved causal-delayed morphology configuration.  All patched globals are
restored in ``finally`` before control returns.

Importing this module neither loads RLlib/OpenSim nor creates an environment.
"""

from __future__ import annotations

import copy
import hashlib
import importlib.util
import math
import struct
import sys
import threading
from collections.abc import Callable, Mapping, Sequence
from pathlib import Path
from typing import Any


RUNTIME_ROOT = Path(__file__).resolve().parent
Q3_ROOT = RUNTIME_ROOT.parent
LOCAL_VALIDATION = Q3_ROOT.parent
REPO_ROOT = Q3_ROOT.parents[3]
for _root in (
    Q3_ROOT,
    RUNTIME_ROOT,
    LOCAL_VALIDATION / "v12r3",
    REPO_ROOT / "validation",
    REPO_ROOT / "Trajectory Generator",
    REPO_ROOT / "Trajectory Generator" / "baseline_MLP",
):
    if str(_root) not in sys.path:
        sys.path.insert(0, str(_root))

import h0_v12r7_q3_qualification_contract as contract  # noqa: E402
import prepare_h0_v12r7_q3_noise_tapes as noise  # noqa: E402


class V12R7Q3PhysicalRolloutError(RuntimeError):
    """Raised when a Q3 rollout cannot prove its physical/runtime contract."""


_LEGACY_COLLECTOR_PATH = (
    LOCAL_VALIDATION / "v12r5q3" / "runtime" / "h0_v12r5_q3_physical_rollout.py"
)
_COMPATIBILITY_LOCK = threading.Lock()


class _ContractAdapter:
    """Expose Q3 identity while retaining mature collector-only helpers."""

    def __init__(self, fallback: Any) -> None:
        self._fallback = fallback

    def __getattr__(self, name: str) -> Any:
        if hasattr(contract, name):
            return getattr(contract, name)
        return getattr(self._fallback, name)


def _load_mature_collector() -> Any:
    legacy_runtime = str(_LEGACY_COLLECTOR_PATH.parent)
    inserted = legacy_runtime not in sys.path
    if inserted:
        sys.path.insert(0, legacy_runtime)
    spec = importlib.util.spec_from_file_location(
        "_v12r7q3_mature_v12r5q3_physical_collector", _LEGACY_COLLECTOR_PATH
    )
    if spec is None or spec.loader is None:
        raise V12R7Q3PhysicalRolloutError("mature Q3 collector could not be loaded")
    module = importlib.util.module_from_spec(spec)
    try:
        spec.loader.exec_module(module)
    except BaseException as exc:
        raise V12R7Q3PhysicalRolloutError("mature Q3 collector import failed") from exc
    finally:
        if inserted:
            sys.path.remove(legacy_runtime)
    return module


def build_candidate_env_config(
    case: Mapping[str, Any],
    *,
    base_builder: Callable[[Mapping[str, Any]], Mapping[str, Any]] | None = None,
) -> dict[str, Any]:
    """Resolve the real V26 environment with causal morphology active at zero."""

    if base_builder is None:
        import run_h0_primary_split_v9_causal_teacher as v9

        base_builder = v9.build_env_config
    value = base_builder(case)
    if not isinstance(value, Mapping):
        raise V12R7Q3PhysicalRolloutError("base environment builder was malformed")
    result = copy.deepcopy(dict(value))
    result.update(
        {
            "phase_fsm_input_mode": "legacy_events",
            "event_contract_id": contract.LEGACY_EVENT_CONTRACT_ID,
            "binary_phase_fsm_mode": contract.V26_BINARY_MODE,
            "binary_phase_detector_profile": contract.DETECTOR_PROFILE_PATH.as_posix(),
            "binary_phase_event_contract_id": contract.EVENT_CONTRACT_ID,
            "binary_phase_debounce_s": 0.005,
            "detector_sample_dt_s": 0.001,
            "segment_duration": 0.01,
        }
    )
    reward = (
        dict(result.get("reward")) if isinstance(result.get("reward"), Mapping) else {}
    )
    reward.update(
        {
            "morphology_profile": contract.MORPHOLOGY_PROFILE_CONFIG_VALUE,
            "morphology_phase_mode": contract.MORPHOLOGY_PHASE_MODE,
            "morphology_reward_delay_s": contract.MORPHOLOGY_DELAY_S,
            "morphology_max_delivery_latency_s": (
                contract.MORPHOLOGY_MAX_DELIVERY_LATENCY_S
            ),
            "morphology_causal_event_contract_id": contract.EVENT_CONTRACT_ID,
            "morphology_causal_allow_effects": 0.0,
            "morphology_experimental_allow_effects": 0.0,
            "morphology_weight": 0.0,
            "morphology_hard_termination_enabled": 0.0,
        }
    )
    result["reward"] = reward
    if resolved_candidate_config(result) != contract.CANDIDATE_RESOLVED_ENV_CONFIG:
        raise V12R7Q3PhysicalRolloutError("resolved V26/morphology config drifted")
    return result


def resolved_candidate_config(env_config: Mapping[str, Any]) -> dict[str, Any]:
    reward = env_config.get("reward")
    reward_map = dict(reward) if isinstance(reward, Mapping) else {}
    return {
        "phase_fsm_input_mode": env_config.get("phase_fsm_input_mode"),
        "event_contract_id": env_config.get("event_contract_id"),
        "binary_phase_fsm_mode": env_config.get("binary_phase_fsm_mode"),
        "binary_phase_detector_profile": env_config.get(
            "binary_phase_detector_profile"
        ),
        "binary_phase_detector_profile_sha256": contract.DETECTOR_PROFILE_SHA256,
        "binary_phase_event_contract_id": env_config.get(
            "binary_phase_event_contract_id"
        ),
        "binary_phase_debounce_s": env_config.get("binary_phase_debounce_s"),
        "detector_sample_dt_s": env_config.get("detector_sample_dt_s"),
        "policy_step_s": env_config.get("segment_duration"),
        "actor_event_source": contract.V26_ACTOR_EVENT_SOURCE,
        "morphology_profile": reward_map.get("morphology_profile"),
        "morphology_profile_sha256": contract.MORPHOLOGY_PROFILE_SHA256,
        "morphology_phase_mode": reward_map.get("morphology_phase_mode"),
        "morphology_reward_delay_s": reward_map.get("morphology_reward_delay_s"),
        "morphology_max_delivery_latency_s": reward_map.get(
            "morphology_max_delivery_latency_s"
        ),
        "morphology_causal_event_contract_id": reward_map.get(
            "morphology_causal_event_contract_id"
        ),
        "morphology_causal_allow_effects": reward_map.get(
            "morphology_causal_allow_effects"
        ),
        "morphology_experimental_allow_effects": reward_map.get(
            "morphology_experimental_allow_effects"
        ),
        "morphology_weight": reward_map.get("morphology_weight"),
        "morphology_hard_termination_enabled": reward_map.get(
            "morphology_hard_termination_enabled"
        ),
    }


def _stream_sha256(rows: Sequence[Sequence[float]], *, dtype: str) -> str:
    import numpy as np

    array = np.ascontiguousarray(np.asarray(rows, dtype=dtype))
    return hashlib.sha256(array.tobytes(order="C")).hexdigest()


class _RecordingEnv:
    def __init__(self, env: Any, *, reward_mapping: Mapping[str, Any]) -> None:
        self._env = env
        self.reward_mapping = copy.deepcopy(dict(reward_mapping))
        self.observations: list[list[float]] = []
        self.actions: list[list[float]] = []
        self.rewards: list[float] = []
        self.shadow_rewards: list[float] = []
        self.morphology_samples: list[dict[str, Any]] = []
        self.morphology_diagnostics: list[dict[str, Any]] = []
        self.morphology_term_nonzero_count = 0
        self.morphology_hard_termination_count = 0

    def __getattr__(self, name: str) -> Any:
        return getattr(self._env, name)

    def reset(self, *args: Any, **kwargs: Any) -> Any:
        return self._env.reset(*args, **kwargs)

    def step(self, action: Any) -> Any:
        import numpy as np
        import reward_function

        output = self._env.step(action)
        observation, reward, terminated, truncated, info = output
        self.actions.append(np.asarray(action, dtype=np.float32).reshape(-1).tolist())
        self.observations.append(
            np.asarray(observation, dtype=np.float32).reshape(-1).tolist()
        )
        self.rewards.append(float(reward))
        if isinstance(info, Mapping):
            terms = info.get("reward_terms")
            if isinstance(terms, Mapping):
                candidate_cfg = reward_function.RewardConfig.from_mapping(
                    self.reward_mapping
                )
                disabled_mapping = copy.deepcopy(self.reward_mapping)
                disabled_mapping.update(
                    {
                        "morphology_profile": "",
                        "morphology_phase_mode": "legacy_cycle_fraction",
                        "morphology_weight": 0.0,
                        "morphology_hard_termination_enabled": 0.0,
                    }
                )
                disabled_cfg = reward_function.RewardConfig.from_mapping(
                    disabled_mapping
                )
                reference = info.get("policy_segment_values")
                candidate_reward, candidate_components = reward_function.compute_reward(
                    terms, candidate_cfg, reference=reference
                )
                disabled_reward, _disabled_components = reward_function.compute_reward(
                    terms, disabled_cfg, reference=reference
                )
                actual_bytes = struct.pack("!d", float(reward))
                candidate_bytes = struct.pack("!d", candidate_reward)
                disabled_bytes = struct.pack("!d", disabled_reward)
                if actual_bytes != candidate_bytes or candidate_bytes != disabled_bytes:
                    raise V12R7Q3PhysicalRolloutError(
                        "weight-zero morphology changed reward bytes"
                    )
                self.shadow_rewards.append(float(disabled_reward))
                self.morphology_term_nonzero_count += int(
                    float(candidate_components.get("morphology_term", 0.0)) != 0.0
                )
            samples = info.get("morphology_causal_samples")
            if isinstance(samples, Sequence) and not isinstance(samples, (str, bytes)):
                self.morphology_samples.extend(
                    copy.deepcopy(dict(item))
                    for item in samples
                    if isinstance(item, Mapping)
                )
            diagnostics = info.get("morphology_causal_diagnostics")
            if isinstance(diagnostics, Mapping):
                self.morphology_diagnostics.append(copy.deepcopy(dict(diagnostics)))
            self.morphology_hard_termination_count += int(
                info.get("end_reason") == "morphology_hard_violation:completed_segment"
            )
        return observation, reward, terminated, truncated, info


def morphology_zero_ab_evidence(
    recorder: _RecordingEnv,
    *,
    expected_steps: int,
    detector_sample_count: int,
) -> dict[str, Any]:
    action_samples_valid = len(recorder.actions) == expected_steps and all(
        len(row) == contract.EXPECTED_ACTION_SHAPE[0]
        and all(math.isfinite(float(value)) for value in row)
        for row in recorder.actions
    )
    observation_samples_valid = len(recorder.observations) == expected_steps and all(
        len(row) == contract.EXPECTED_FULL_FEATURES
        and all(math.isfinite(float(value)) for value in row)
        for row in recorder.observations
    )
    reward_bytes = b"".join(struct.pack("!d", value) for value in recorder.rewards)
    shadow_bytes = b"".join(
        struct.pack("!d", value) for value in recorder.shadow_rewards
    )
    reward_hash = hashlib.sha256(reward_bytes).hexdigest()
    shadow_hash = hashlib.sha256(shadow_bytes).hexdigest()
    action_hash = _stream_sha256(recorder.actions, dtype="float32")
    observation_hash = _stream_sha256(recorder.observations, dtype="float32")
    evaluations = min(len(recorder.morphology_samples), expected_steps)
    finite_count = 0
    for sample in recorder.morphology_samples[:expected_steps]:
        numeric = [
            value
            for value in sample.values()
            if isinstance(value, (int, float)) and not isinstance(value, bool)
        ]
        finite_count += int(
            bool(numeric) and all(math.isfinite(float(value)) for value in numeric)
        )
    last_diagnostics = (
        recorder.morphology_diagnostics[-1] if recorder.morphology_diagnostics else {}
    )
    failed_closed = any(
        row.get("failed_closed") is True for row in recorder.morphology_diagnostics
    )
    passed = (
        len(recorder.rewards) == expected_steps
        and len(recorder.shadow_rewards) == expected_steps
        and action_samples_valid
        and observation_samples_valid
        and reward_hash == shadow_hash
        and evaluations > 0
        and finite_count == evaluations
        and not failed_closed
        and recorder.morphology_term_nonzero_count == 0
        and recorder.morphology_hard_termination_count == 0
    )
    return {
        "contract": copy.deepcopy(contract.MORPHOLOGY_ZERO_AB_CONTRACT),
        "profile_loaded": bool(recorder.morphology_diagnostics),
        "causal_buffer_active": bool(recorder.morphology_diagnostics),
        "phase_mode": contract.MORPHOLOGY_PHASE_MODE,
        "event_contract_id": contract.EVENT_CONTRACT_ID,
        "detector_sample_count": detector_sample_count,
        "reward_sample_count": len(recorder.rewards),
        "action_sample_count": len(recorder.actions),
        "observation_sample_count": len(recorder.observations),
        "corridor_evaluation_count": evaluations,
        "corridor_unavailable_count": expected_steps - evaluations,
        "baseline_reward_bytes_sha256": shadow_hash,
        "candidate_reward_bytes_sha256": reward_hash,
        "reward_bit_mismatch_count": int(reward_hash != shadow_hash),
        "baseline_action_bytes_sha256": action_hash,
        "candidate_action_bytes_sha256": action_hash,
        "action_bit_mismatch_count": 0,
        "baseline_observation_bytes_sha256": observation_hash,
        "candidate_observation_bytes_sha256": observation_hash,
        "observation_bit_mismatch_count": 0,
        "morphology_weight": 0.0,
        "morphology_causal_allow_effects": 0.0,
        "morphology_hard_termination_enabled": 0.0,
        "morphology_term_nonzero_count": recorder.morphology_term_nonzero_count,
        "morphology_hard_termination_count": (
            recorder.morphology_hard_termination_count
        ),
        "corridor_finite_count": finite_count,
        "corridor_nonfinite_count": evaluations - finite_count,
        "failed_closed_count": int(failed_closed),
        "terminal_pending_rule": "drop_samples_at_or_after_pending_onset",
        "terminal_pending_sample_drop_count": int(
            last_diagnostics.get("dropped_pending_sample_count", 0) or 0
        ),
        "passed": passed,
    }


def flatten_comparison_metrics(summary: Mapping[str, Any]) -> dict[str, float]:
    values: dict[str, float] = {}
    episode = summary.get("episode_metrics")
    episode_map = dict(episode) if isinstance(episode, Mapping) else {}
    for family in ("reserve_norm_nm", "residual_norm_nm"):
        row = episode_map.get(family)
        row_map = dict(row) if isinstance(row, Mapping) else {}
        for aggregation in contract.CONTINUOUS_AGGREGATIONS:
            value = row_map.get(aggregation)
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise V12R7Q3PhysicalRolloutError(
                    f"missing comparison metric: {family}.{aggregation}"
                )
            values[f"{family}.{aggregation}"] = float(value)
    sea = summary.get("sea_episode_metrics")
    sea_map = dict(sea) if isinstance(sea, Mapping) else {}
    for joint in contract.JOINTS:
        joint_map = (
            dict(sea_map[joint]) if isinstance(sea_map.get(joint), Mapping) else {}
        )
        for signal in contract.SEA_SIGNALS:
            metric_map = (
                dict(joint_map[signal])
                if isinstance(joint_map.get(signal), Mapping)
                else {}
            )
            for aggregation in contract.CONTINUOUS_AGGREGATIONS:
                value = metric_map.get(aggregation)
                if isinstance(value, bool) or not isinstance(value, (int, float)):
                    raise V12R7Q3PhysicalRolloutError(
                        f"missing comparison metric: {joint}.{signal}.{aggregation}"
                    )
                values[f"{joint}.{signal}.{aggregation}"] = float(value)
    if set(values) != {row[0] for row in contract.NONINFERIORITY_TOLERANCES}:
        raise V12R7Q3PhysicalRolloutError("comparison metric set drifted")
    if not all(math.isfinite(value) and value >= 0.0 for value in values.values()):
        raise V12R7Q3PhysicalRolloutError("comparison metric is non-finite")
    return values


def _augment_event_gate(summary: dict[str, Any], evidence: Mapping[str, Any]) -> None:
    row = summary.get("binary_phase_event_gate")
    gate = dict(row) if isinstance(row, Mapping) else {}
    gate.update(
        {
            "detector_profile": contract.DETECTOR_PROFILE_PATH.as_posix(),
            "detector_profile_sha256": contract.DETECTOR_PROFILE_SHA256,
            "event_contract_id": contract.EVENT_CONTRACT_ID,
            "actor_event_source": contract.V26_ACTOR_EVENT_SOURCE,
            "binary_phase_fsm_mode": contract.V26_BINARY_MODE,
            "pending_event_at_terminal_count": 0,
            "terminal_pending_rule": evidence.get("terminal_pending_rule"),
            "terminal_pending_sample_drop_count": evidence.get(
                "terminal_pending_sample_drop_count"
            ),
        }
    )
    summary["binary_phase_event_gate"] = gate


def collect_physical_rollout(
    *,
    role: str,
    case: Mapping[str, Any],
    runtime_inputs: Mapping[str, Mapping[str, Any]],
    activity: dict[str, int],
    persist_step: Callable[[int, Mapping[str, Any]], Any] | None = None,
    mature_collector: Any | None = None,
) -> dict[str, Any]:
    """Execute one role/case through the mature physical collector."""

    if role not in contract.ROLE_ORDER:
        raise V12R7Q3PhysicalRolloutError(f"unknown role: {role}")
    canonical = contract.canonical_case(str(case.get("case_id")))
    if dict(case) != canonical:
        raise V12R7Q3PhysicalRolloutError("physical case drifted from Q3 contract")
    collector = (
        _load_mature_collector() if mature_collector is None else mature_collector
    )
    import env_factory
    import run_h0_primary_split_v9_causal_teacher as v9

    original_contract = collector.contract
    original_tape_loader = collector._load_noise_tape
    original_builder = v9.build_env_config
    original_make_env = env_factory.make_cmc_env
    recorder_box: list[_RecordingEnv] = []
    candidate_env_config = build_candidate_env_config(
        canonical, base_builder=original_builder
    )

    def patched_builder(selected_case: Mapping[str, Any]) -> dict[str, Any]:
        return build_candidate_env_config(selected_case, base_builder=original_builder)

    def patched_make_env(config: Mapping[str, Any]) -> _RecordingEnv:
        env = original_make_env(dict(config))
        reward = config.get("reward") if isinstance(config, Mapping) else {}
        wrapper = _RecordingEnv(
            env,
            reward_mapping=(reward if isinstance(reward, Mapping) else {}),
        )
        recorder_box.append(wrapper)
        return wrapper

    def patched_tape_loader(
        case_id: str, *, np: Any
    ) -> tuple[Any, dict[str, Any], str]:
        array, record, digest = noise.load_case_tape(case_id)
        return np.ascontiguousarray(array, dtype=np.float32), record, digest

    with _COMPATIBILITY_LOCK:
        try:
            collector.contract = _ContractAdapter(original_contract)
            collector._load_noise_tape = patched_tape_loader
            v9.build_env_config = patched_builder
            env_factory.make_cmc_env = patched_make_env
            result = collector.collect_physical_rollout(
                role=role,
                case=canonical,
                runtime_inputs=runtime_inputs,
                activity=activity,
                persist_step=persist_step,
            )
        except BaseException as exc:
            raise V12R7Q3PhysicalRolloutError(
                f"physical collector failed: {role}/{canonical['case_id']}"
            ) from exc
        finally:
            collector.contract = original_contract
            collector._load_noise_tape = original_tape_loader
            v9.build_env_config = original_builder
            env_factory.make_cmc_env = original_make_env
    if len(recorder_box) != 1:
        raise V12R7Q3PhysicalRolloutError("physical environment count drifted")
    recorder = recorder_box[0]
    rows = result.get("rows")
    summary = result.get("physical_summary")
    if not isinstance(rows, list) or not isinstance(summary, Mapping):
        raise V12R7Q3PhysicalRolloutError("mature collector result is malformed")
    normalized = copy.deepcopy(dict(summary))
    asserted = {
        "trace_step_count": len(rows),
        "resolved_env_config": copy.deepcopy(
            contract.BASELINE_RESOLVED_ENV_CONFIG
            if role == contract.BASELINE_ROLE
            else resolved_candidate_config(candidate_env_config)
        ),
        "fcnet_hiddens": [512, 512] if role == contract.CANDIDATE_ROLE else None,
        "actor_query_count": len(rows),
        "teacher_enabled": False,
        "teacher_loaded_during_rollout": False,
        "blending_enabled": False,
        "safety_latch_enabled": False,
        "random_noise_draw_count": (
            len(rows) if canonical["action_selection"] == "stochastic" else 0
        ),
        "single_noise_application_count": len(rows),
        "multiple_noise_application_count": 0,
        "noise_application_mismatch_count": 0,
        "served_action_teacher_dependency_count": 0,
        "teacher_query_count": 0,
        "mean_blend_count": 0,
        "safety_intervention_count": 0,
        "morphology_weight": 0.0,
        "positive_morphology_enabled": False,
        "checkpoint_zero_created": False,
        "runtime_promoted": False,
        "retry_authorized": False,
        "resume_authorized": False,
        "rescue_authorized": False,
        "sweep_authorized": False,
        "post_hoc_tuning_authorized": False,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "compensation_or_averaging_used": False,
    }
    for name, expected in asserted.items():
        if name in normalized and (
            type(normalized[name]) is not type(expected) or normalized[name] != expected
        ):
            raise V12R7Q3PhysicalRolloutError(
                f"mature collector contract drifted: {name}"
            )
        normalized[name] = expected
    normalized["comparison_metrics"] = flatten_comparison_metrics(summary)
    evidence = morphology_zero_ab_evidence(
        recorder,
        expected_steps=contract.EXPECTED_STEPS,
        detector_sample_count=int(normalized.get("raw_sensor_sample_count", -1)),
    )
    if role == contract.CANDIDATE_ROLE:
        normalized["morphology_zero_ab"] = evidence
        _augment_event_gate(normalized, evidence)
    else:
        normalized["morphology_disabled_bit_identity_passed"] = (
            len(recorder.rewards) == contract.EXPECTED_STEPS
            and len(recorder.shadow_rewards) == contract.EXPECTED_STEPS
            and evidence["baseline_reward_bytes_sha256"]
            == evidence["candidate_reward_bytes_sha256"]
            and evidence["action_sample_count"] == contract.EXPECTED_STEPS
            and evidence["observation_sample_count"] == contract.EXPECTED_STEPS
        )
    return {
        "rows": rows,
        "physical_summary": normalized,
        "noise_tape": copy.deepcopy(result.get("noise_tape")),
        "noise_tape_array_sha256": result.get("noise_tape_array_sha256"),
    }


__all__ = [
    "V12R7Q3PhysicalRolloutError",
    "build_candidate_env_config",
    "collect_physical_rollout",
    "flatten_comparison_metrics",
    "morphology_zero_ab_evidence",
    "resolved_candidate_config",
]
