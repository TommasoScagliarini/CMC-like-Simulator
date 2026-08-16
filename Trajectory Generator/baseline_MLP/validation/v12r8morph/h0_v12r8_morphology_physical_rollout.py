"""Physical control/positive collector for the V12R8 morphology A/B.

The mature Q3 physical loop is reused for action generation, OpenSim stepping,
V26 collection, SEA/solver diagnostics, and frozen noise tapes.  This adapter
changes only the causal morphology reward authority for the selected arm and
adds canonical, comparison-complete trace surfaces.  Both roles query the same
RLModule exported from the verified full checkpoint-zero.

Importing this module does not load RLlib, OpenSim, or construct an environment.
"""

from __future__ import annotations

import copy
import hashlib
import math
import struct
import sys
import time
from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import Any, Callable

import numpy as np

try:
    from . import h0_v12r8_morphology_contract as contract
    from . import h0_v12r8_morphology_gates as gates
except ImportError:
    import h0_v12r8_morphology_contract as contract
    import h0_v12r8_morphology_gates as gates


HERE = Path(__file__).resolve().parent
VALIDATION_ROOT = HERE.parent
Q3_ROOT = VALIDATION_ROOT / "v12r8q3"
Q3_RUNTIME = Q3_ROOT / "runtime"
BASELINE_ROOT = VALIDATION_ROOT.parent
REPO_ROOT = BASELINE_ROOT.parents[1]
for _root in (Q3_ROOT, Q3_RUNTIME, BASELINE_ROOT, REPO_ROOT / "validation"):
    if str(_root) not in sys.path:
        sys.path.insert(0, str(_root))


class MorphologyPhysicalRolloutError(RuntimeError):
    """Raised when a physical arm cannot prove the frozen A/B contract."""


def _json_value(value: Any) -> Any:
    if value is None or type(value) in {bool, str, int}:
        return value
    if isinstance(value, (float, np.floating)):
        result = float(value)
        if not math.isfinite(result):
            raise MorphologyPhysicalRolloutError("non-finite trace value")
        return result
    if isinstance(value, np.integer):
        return int(value)
    if isinstance(value, np.ndarray):
        return _json_value(value.tolist())
    if isinstance(value, Mapping):
        output: dict[str, Any] = {}
        for key, child in value.items():
            if not isinstance(key, str) or not key:
                raise MorphologyPhysicalRolloutError("non-string trace key")
            output[key] = _json_value(child)
        return output
    if isinstance(value, Sequence) and not isinstance(value, (str, bytes, bytearray)):
        return [_json_value(item) for item in value]
    raise MorphologyPhysicalRolloutError(
        f"unsupported trace value type: {type(value).__name__}"
    )


def _float_bits_equal(left: float, right: float) -> bool:
    return struct.pack("!d", float(left)) == struct.pack("!d", float(right))


def stream_record(rows: Sequence[Any]) -> dict[str, Any]:
    encoded = gates.canonical_json_bytes(list(rows))
    return {
        "sha256": hashlib.sha256(encoded).hexdigest(),
        "size_bytes": len(encoded),
        "sample_count": len(rows),
        "encoding": contract.STREAM_ENCODING,
    }


def build_env_config(
    case: Mapping[str, Any],
    *,
    role: str,
    base_builder: Callable[[Mapping[str, Any]], Mapping[str, Any]] | None = None,
) -> dict[str, Any]:
    """Build the Q3 V26 environment and change exactly the two A/B fields."""

    import h0_v12r8_q3_physical_rollout as q3_physical

    value = q3_physical.build_candidate_env_config(case, base_builder=base_builder)
    result = copy.deepcopy(dict(value))
    reward = dict(result.get("reward") or {})
    reward.update(contract.reward_config_for_role(role))
    result["reward"] = reward
    observed = {name: reward.get(name) for name in contract.CONTROL_REWARD_CONFIG}
    if observed != contract.reward_config_for_role(role):
        raise MorphologyPhysicalRolloutError("live morphology reward config drifted")
    fixed = {
        "phase_fsm_input_mode": result.get("phase_fsm_input_mode"),
        "event_contract_id": result.get("event_contract_id"),
        "binary_phase_fsm_mode": result.get("binary_phase_fsm_mode"),
        "binary_phase_detector_profile": result.get(
            "binary_phase_detector_profile"
        ),
        "detector_sample_dt_s": result.get("detector_sample_dt_s"),
        "binary_phase_debounce_s": result.get("binary_phase_debounce_s"),
        "binary_phase_event_contract_id": result.get(
            "binary_phase_event_contract_id"
        ),
        "policy_step_s": result.get("segment_duration"),
    }
    expected_fixed = {
        key: contract.V26_RUNTIME_CONFIG[key]
        for key in fixed
    }
    if fixed != expected_fixed:
        raise MorphologyPhysicalRolloutError("live V26 environment config drifted")
    return result


class _RecordingEnv:
    def __init__(self, env: Any, *, role: str, reward_mapping: Mapping[str, Any]) -> None:
        self._env = env
        self.role = role
        self.reward_mapping = copy.deepcopy(dict(reward_mapping))
        self._current_observation: np.ndarray | None = None
        self.steps: list[dict[str, Any]] = []

    def __getattr__(self, name: str) -> Any:
        return getattr(self._env, name)

    def reset(self, *args: Any, **kwargs: Any) -> Any:
        output = self._env.reset(*args, **kwargs)
        observation = output[0] if isinstance(output, tuple) else output
        self._current_observation = np.ascontiguousarray(
            observation, dtype=np.float32
        ).reshape(-1)
        return output

    def step(self, action: Any) -> Any:
        import reward_function

        if self._current_observation is None:
            raise MorphologyPhysicalRolloutError("step occurred before reset")
        observation_before = self._current_observation.copy()
        output = self._env.step(action)
        observation, reward, terminated, truncated, info = output
        if not isinstance(info, Mapping):
            raise MorphologyPhysicalRolloutError("physical info is not a mapping")
        observation_after = np.ascontiguousarray(
            observation, dtype=np.float32
        ).reshape(-1)
        if (
            observation_before.shape != (contract.EXPECTED_FULL_FEATURES,)
            or observation_after.shape != (contract.EXPECTED_FULL_FEATURES,)
            or not np.all(np.isfinite(observation_before))
            or not np.all(np.isfinite(observation_after))
        ):
            raise MorphologyPhysicalRolloutError("full observation contract drifted")
        terms = info.get("reward_terms")
        if not isinstance(terms, Mapping):
            raise MorphologyPhysicalRolloutError("reward terms are missing")
        terms_map = copy.deepcopy(dict(terms))
        reference = info.get("policy_segment_values")
        control_cfg = reward_function.RewardConfig.from_mapping(
            contract.CONTROL_REWARD_CONFIG
        )
        active_cfg = reward_function.RewardConfig.from_mapping(self.reward_mapping)
        base_reward, _base_components = reward_function.compute_reward(
            terms_map, control_cfg, reference=reference
        )
        recomputed, components = reward_function.compute_reward(
            terms_map, active_cfg, reference=reference
        )
        actual_reward = float(reward)
        morphology_loss = float(terms_map.get("morphology_loss", 0.0))
        morphology_term = float(components.get("morphology_term", 0.0))
        if not (
            math.isfinite(morphology_loss)
            and morphology_loss >= 0.0
            and _float_bits_equal(actual_reward, recomputed)
        ):
            raise MorphologyPhysicalRolloutError("live reward recomposition drifted")
        expected_term = 0.0 if self.role == contract.CONTROL_ROLE else 0.0025 * morphology_loss
        if not _float_bits_equal(morphology_term, expected_term):
            raise MorphologyPhysicalRolloutError("live morphology term drifted")
        samples = info.get("morphology_causal_samples")
        diagnostics = info.get("morphology_causal_diagnostics")
        if not (
            isinstance(samples, Sequence)
            and not isinstance(samples, (str, bytes, bytearray))
            and isinstance(diagnostics, Mapping)
        ):
            raise MorphologyPhysicalRolloutError("causal morphology evidence missing")
        self.steps.append(
            {
                "observation_before": observation_before.tolist(),
                "observation_after": observation_after.tolist(),
                "applied_action": np.ascontiguousarray(
                    action, dtype=np.float32
                ).reshape(-1).tolist(),
                "actual_reward": actual_reward,
                "recomputed_reward": float(recomputed),
                "reward_without_morphology": float(base_reward),
                "morphology_loss": morphology_loss,
                "morphology_term": morphology_term,
                "causal_samples": _json_value(samples),
                "causal_diagnostics": _json_value(diagnostics),
                "binary_phase_sensor_samples": _json_value(
                    info.get("binary_phase_sensor_samples", [])
                ),
                "phase_fsm": _json_value(info.get("phase_fsm", {})),
                "policy_segment_values": _json_value(
                    reference if reference is not None else {}
                ),
                "sea_segment_diagnostics": _json_value(
                    info.get("sea_segment_diagnostics", {})
                ),
                "terminated": bool(terminated),
                "truncated": bool(truncated),
                "end_reason": _json_value(info.get("end_reason")),
            }
        )
        self._current_observation = observation_after
        return output


def _detector_audit(physical_summary: Mapping[str, Any]) -> dict[str, Any]:
    gate = _json_value(physical_summary.get("binary_phase_event_gate", {}))
    if not isinstance(gate, Mapping):
        gate = {}
    return {
        "runtime_config": copy.deepcopy(contract.V26_RUNTIME_CONFIG),
        "sample_count": int(gate.get("sample_count", -1)),
        "fallback_count": int(gate.get("fallback_count", -1)),
        "non_v26_source_count": int(gate.get("left_non_v26_source_count", -1)),
        "duplicate_event_count": int(gate.get("duplicate_event_count", -1)),
        "out_of_order_event_count": int(gate.get("out_of_order_event_count", -1)),
        "hard_invalid_count": int(gate.get("hard_invalid_count", -1)),
        "passed": gate.get("passed") is True,
    }


def _canonical_surfaces(
    *,
    role: str,
    case_id: str,
    recorder: _RecordingEnv,
    mature_rows: Sequence[Mapping[str, Any]],
) -> dict[str, Any]:
    if (
        len(recorder.steps) != contract.EXPECTED_STEPS_PER_ROLLOUT
        or len(mature_rows) != contract.EXPECTED_STEPS_PER_ROLLOUT
    ):
        raise MorphologyPhysicalRolloutError("physical rollout step count drifted")
    trace_rows: list[dict[str, Any]] = []
    reward_rows: list[dict[str, Any]] = []
    causal_rows: list[dict[str, Any]] = []
    observations: list[Any] = []
    actions: list[Any] = []
    dynamics: list[Any] = []
    events: list[Any] = []
    base_rewards: list[float] = []
    morphology_losses: list[float] = []
    for index, (recorded, mature_raw) in enumerate(
        zip(recorder.steps, mature_rows, strict=True), start=1
    ):
        mature = _json_value(mature_raw)
        if not isinstance(mature, Mapping) or mature.get("step") != index:
            raise MorphologyPhysicalRolloutError("mature trace order drifted")
        action = {
            "raw_action": mature.get("raw_action"),
            "applied_action": mature.get("applied_action"),
        }
        if action["applied_action"] != recorded["applied_action"]:
            raise MorphologyPhysicalRolloutError("recorded action drifted")
        observation = {
            "before": recorded["observation_before"],
            "after": recorded["observation_after"],
        }
        dynamics_proxy = {
            "grf_penetration_m": mature.get("grf_penetration_m"),
            "reserve_norm_nm": mature.get("reserve_norm_nm"),
            "residual_norm_nm": mature.get("residual_norm_nm"),
            "sea_segment_diagnostics": recorded["sea_segment_diagnostics"],
            "policy_segment_values": recorded["policy_segment_values"],
            "terminated": recorded["terminated"],
            "truncated": recorded["truncated"],
            "end_reason": recorded["end_reason"],
        }
        event = {
            "binary_phase_sensor_samples": recorded[
                "binary_phase_sensor_samples"
            ],
            "phase_fsm": recorded["phase_fsm"],
        }
        reward_row = {
            "step": index,
            "actual_reward": recorded["actual_reward"],
            "recomputed_reward": recorded["recomputed_reward"],
            "reward_without_morphology": recorded["reward_without_morphology"],
            "morphology_loss": recorded["morphology_loss"],
            "morphology_term": recorded["morphology_term"],
        }
        causal_row = {
            "step": index,
            "samples": recorded["causal_samples"],
            "diagnostics": recorded["causal_diagnostics"],
        }
        trace_rows.append(
            {
                "schema_version": contract.SCHEMA_VERSION,
                "protocol_id": contract.PROTOCOL_ID,
                "case_id": case_id,
                "role": role,
                "step": index,
                "observation": observation,
                "action": action,
                "dynamics": dynamics_proxy,
                "events": event,
                "reward": reward_row,
                "causal": causal_row,
            }
        )
        reward_rows.append(reward_row)
        causal_rows.append(causal_row)
        observations.append(observation)
        actions.append(action)
        dynamics.append(dynamics_proxy)
        events.append(event)
        base_rewards.append(recorded["reward_without_morphology"])
        morphology_losses.append(recorded["morphology_loss"])
    streams = {
        "observations": stream_record(observations),
        "actions": stream_record(actions),
        "dynamics": stream_record(dynamics),
        "events": stream_record(events),
        "reward_without_morphology": stream_record(base_rewards),
        "morphology_loss": stream_record(morphology_losses),
    }
    return {
        "trace_rows": trace_rows,
        "reward_rows": reward_rows,
        "causal_rows": causal_rows,
        "streams": streams,
    }


def collect_physical_arm(
    *,
    role: str,
    case: Mapping[str, Any],
    module_export: Mapping[str, Any],
    activity: dict[str, int],
) -> dict[str, Any]:
    """Execute one real physical arm using the checkpoint-derived module."""

    if role not in contract.PAIR_ROLE_ORDER:
        raise MorphologyPhysicalRolloutError(f"unsupported role: {role}")
    canonical = contract.canonical_case(str(case.get("case_id")))
    if dict(case) != canonical:
        raise MorphologyPhysicalRolloutError("case drifted from frozen Q3 condition")
    import env_factory
    import h0_v12r8_morphology_causal_runtime as causal_runtime
    import h0_v12r8_q3_physical_rollout as q3_physical
    import prepare_h0_v12r8_q3_noise_tapes as q3_noise
    import run_h0_primary_split_v9_causal_teacher as v9

    causal_runtime.install()
    causal_attestation = causal_runtime.assert_installed()
    collector = q3_physical._load_mature_collector()  # noqa: SLF001
    original_contract = collector.contract
    original_tape_loader = collector._load_noise_tape
    original_builder = v9.build_env_config
    original_make_env = env_factory.make_cmc_env
    recorder_box: list[_RecordingEnv] = []

    def patched_builder(selected_case: Mapping[str, Any]) -> dict[str, Any]:
        return build_env_config(
            selected_case, role=role, base_builder=original_builder
        )

    def patched_make_env(config: Mapping[str, Any]) -> _RecordingEnv:
        env = original_make_env(dict(config))
        reward = config.get("reward") if isinstance(config, Mapping) else {}
        wrapper = _RecordingEnv(
            env,
            role=role,
            reward_mapping=(reward if isinstance(reward, Mapping) else {}),
        )
        recorder_box.append(wrapper)
        return wrapper

    def patched_tape_loader(
        case_id: str, *, np: Any
    ) -> tuple[Any, dict[str, Any], str]:
        array, record, digest = q3_noise.load_case_tape(case_id)
        return np.ascontiguousarray(array, dtype=np.float32), record, digest

    started = time.time()
    with q3_physical._COMPATIBILITY_LOCK:  # noqa: SLF001
        try:
            collector.contract = q3_physical._ContractAdapter(original_contract)  # noqa: SLF001
            collector._load_noise_tape = patched_tape_loader
            v9.build_env_config = patched_builder
            env_factory.make_cmc_env = patched_make_env
            result = collector.collect_physical_rollout(
                role=contract.q3.CANDIDATE_ROLE,
                case=canonical,
                runtime_inputs={"candidate_module": dict(module_export)},
                activity=activity,
                persist_step=None,
            )
        except BaseException as exc:
            raise MorphologyPhysicalRolloutError(
                f"physical arm failed: {role}/{canonical['case_id']}"
            ) from exc
        finally:
            collector.contract = original_contract
            collector._load_noise_tape = original_tape_loader
            v9.build_env_config = original_builder
            env_factory.make_cmc_env = original_make_env
    if len(recorder_box) != 1:
        raise MorphologyPhysicalRolloutError("physical environment count drifted")
    rows = result.get("rows")
    physical_summary = result.get("physical_summary")
    tape = result.get("noise_tape")
    tape_array_sha = result.get("noise_tape_array_sha256")
    if not (
        isinstance(rows, list)
        and isinstance(physical_summary, Mapping)
        and isinstance(tape, Mapping)
        and gates.artifact_record_valid(tape, expected_path=canonical["noise_tape"])
        and tape_array_sha
        == contract.EXPECTED_TAPE_ARRAY_SHA256[Path(canonical["noise_tape"]).name]
    ):
        raise MorphologyPhysicalRolloutError("physical result closure drifted")
    surfaces = _canonical_surfaces(
        role=role,
        case_id=canonical["case_id"],
        recorder=recorder_box[0],
        mature_rows=rows,
    )
    reward_gate = gates.reward_ledger_gate(surfaces["reward_rows"], role=role)
    causal_gate = gates.causal_ledger_gate(
        surfaces["causal_rows"], reward_rows=surfaces["reward_rows"]
    )
    detector = _detector_audit(physical_summary)
    if not (
        reward_gate["passed"] is True
        and causal_gate["passed"] is True
        and gates.detector_audit_gate(detector)["passed"] is True
    ):
        raise MorphologyPhysicalRolloutError(
            f"physical arm runtime gates failed: {role}/{canonical['case_id']}"
        )
    return {
        **surfaces,
        "detector_audit": detector,
        "causal_audit": causal_gate,
        "causal_runtime": {
            **copy.deepcopy(contract.CAUSAL_RUNTIME_CONFIG),
            "live_attestation": causal_attestation,
        },
        "noise_tape": copy.deepcopy(dict(tape)),
        "noise_tape_array_sha256": tape_array_sha,
        "started_unix_s": float(started),
        "completed_unix_s": float(time.time()),
        "physical_summary_proxy": {
            "trace_step_count": len(rows),
            "raw_sensor_sample_count": physical_summary.get(
                "raw_sensor_sample_count"
            ),
            "binary_phase_event_gate": _json_value(
                physical_summary.get("binary_phase_event_gate", {})
            ),
        },
    }


__all__ = [
    "MorphologyPhysicalRolloutError",
    "build_env_config",
    "collect_physical_arm",
    "stream_record",
]
