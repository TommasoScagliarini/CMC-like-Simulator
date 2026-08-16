"""Reusable pure-policy physical development runtime for H0 V12R6.

This module deliberately owns no protocol freeze, execution lock, candidate
identity, gate, receipt, or terminal ledger.  A lineage-specific runner passes
those bindings as metadata, evaluates its own gate after :func:`run_case`, and
publishes the gate/receipt itself.  The only durable output created here is the
crash-forensic rollout prefix used by V12R5::

    run_start.json -> steps/*.json
      -> trace.json + partial_summary.json + summary.json

The production inference/environment/physics primitives are the same ones used
by V12R5, reached through the V12R3 runtime adapter.  In particular this file
does not import or rebind the V12R5 contract or runner globals.
"""

from __future__ import annotations

import copy
import math
import os
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Mapping, Sequence


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
import run_h0_primary_split_v9_causal_teacher as env_source  # noqa: E402
import run_h0_primary_split_v12r3_autonomy_recovery as runtime  # noqa: E402


EXPECTED_STEPS = 500
EXPECTED_ACTOR_FEATURES = 35
EXPECTED_ACTION_DIM = 2
EXPECTED_RAW_SENSOR_SAMPLES_PER_STEP = 10
PURE_POLICY_COUNTER_FIELDS = (
    "teacher_query_count",
    "served_action_teacher_dependency_count",
    "mean_blend_count",
    "safety_intervention_count",
    "safety_latch_activation_count",
    "safety_latch_release_count",
)

ActivityCallback = Callable[[str, int], None]
StackLoader = Callable[[], tuple[Any, Any, Any, Any, Any, Any, Any]]
EnvConfigBuilder = Callable[[Mapping[str, Any]], Mapping[str, Any]]


class V12R6PhysicalDevelopmentError(RuntimeError):
    """Raised when a physical rollout violates the reusable V12R6 surface."""


@dataclass(frozen=True)
class PhysicalDevelopmentConfig:
    """Lineage labels and filesystem boundary for one 500-step rollout.

    Counts and tensor dimensions are intentionally not configurable: changing
    them would create a different evidence topology rather than another V12R6
    case.  ``artifact_root`` defaults to the repository and can be redirected
    to a temporary directory by mocked tests.
    """

    protocol_id: str
    start_status: str
    partial_status: str
    complete_status: str
    schema_version: int = 1
    artifact_root: Path = REPO_ROOT
    progress_label: str = "V12R6 physical development"
    progress_every: int = 25


def _validate_config(config: PhysicalDevelopmentConfig) -> None:
    if not isinstance(config, PhysicalDevelopmentConfig):
        raise V12R6PhysicalDevelopmentError(
            "config must be a PhysicalDevelopmentConfig"
        )
    labels = {
        "protocol_id": config.protocol_id,
        "start_status": config.start_status,
        "partial_status": config.partial_status,
        "complete_status": config.complete_status,
        "progress_label": config.progress_label,
    }
    malformed = [
        name
        for name, value in labels.items()
        if not isinstance(value, str) or not value.strip()
    ]
    if malformed:
        raise V12R6PhysicalDevelopmentError(
            f"config labels must be non-empty strings: {malformed}"
        )
    if type(config.schema_version) is not int or config.schema_version < 1:
        raise V12R6PhysicalDevelopmentError("schema_version must be a positive int")
    if type(config.progress_every) is not int or config.progress_every < 0:
        raise V12R6PhysicalDevelopmentError("progress_every must be a non-negative int")
    root = Path(config.artifact_root).expanduser().resolve()
    if not root.is_dir() or root.is_symlink():
        raise V12R6PhysicalDevelopmentError(
            f"artifact_root must be an existing non-symlink directory: {root}"
        )


def _strict_metadata(value: Mapping[str, Any] | None, *, label: str) -> dict[str, Any]:
    if value is None:
        return {}
    if not isinstance(value, Mapping):
        raise V12R6PhysicalDevelopmentError(f"{label} must be a mapping")
    result = copy.deepcopy(dict(value))
    try:
        forensic.canonical_json_bytes(result)
    except BaseException as exc:
        raise V12R6PhysicalDevelopmentError(
            f"{label} is not finite strict JSON"
        ) from exc
    return result


def _merge_metadata(
    base: Mapping[str, Any],
    metadata: Mapping[str, Any] | None,
    *,
    label: str,
) -> dict[str, Any]:
    extra = _strict_metadata(metadata, label=label)
    overlap = sorted(set(base).intersection(extra))
    if overlap:
        raise V12R6PhysicalDevelopmentError(
            f"{label} cannot override runtime-owned fields: {overlap}"
        )
    return {**dict(base), **extra}


def _finite_number(value: Any) -> bool:
    return (
        not isinstance(value, bool)
        and isinstance(value, (int, float))
        and math.isfinite(float(value))
    )


def _finite_float(value: Any, *, label: str) -> float:
    if not _finite_number(value):
        raise V12R6PhysicalDevelopmentError(f"{label} must be finite")
    return float(value)


def _activity(
    callback: ActivityCallback | None,
    name: str,
    amount: int = 1,
) -> None:
    if type(amount) is not int or amount < 0:
        raise V12R6PhysicalDevelopmentError("activity amount must be non-negative")
    if callback is not None:
        callback(name, amount)


def _inside(path: Path, root: Path, *, label: str) -> Path:
    resolved = path.expanduser().resolve()
    try:
        resolved.relative_to(root)
    except ValueError as exc:
        raise V12R6PhysicalDevelopmentError(
            f"{label} escaped artifact_root: {resolved}"
        ) from exc
    return resolved


def _module_display_path(path: Path, artifact_root: Path) -> str:
    try:
        return path.relative_to(artifact_root).as_posix()
    except ValueError:
        return path.as_posix()


def _rollout_stack() -> tuple[Any, Any, Any, Any, Any, Any, Any]:
    """Load the exact production stack reached by the V12R5 rollout helper."""

    try:
        return runtime._load_rollout_stack()
    except Exception as exc:
        raise V12R6PhysicalDevelopmentError(
            "V26 physical inference stack is not ready"
        ) from exc


def diagnostic_raw_journal(info: Mapping[str, Any], *, step: int) -> dict[str, Any]:
    """Capture observer diagnostics that cannot influence actions or gates."""

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


def pure_policy_trace_audit(
    trace: Any,
    *,
    config: PhysicalDevelopmentConfig,
    case_id: str,
) -> dict[str, Any]:
    """Recompute the V12R6 pure mean-plus-single-noise action evidence."""

    _validate_config(config)
    if not isinstance(case_id, str) or not case_id:
        raise V12R6PhysicalDevelopmentError("case_id must be a non-empty string")
    rows = (
        list(trace)
        if isinstance(trace, Sequence) and not isinstance(trace, (str, bytes))
        else []
    )
    counters = {name: 0 for name in PURE_POLICY_COUNTER_FIELDS}
    schema_exact = len(rows) == EXPECTED_STEPS
    identity_exact = schema_exact
    action_path_exact = schema_exact
    raw_sensor_exact = schema_exact
    forbidden_teacher_payload_absent = schema_exact
    per_row_zero_counters = schema_exact
    forbidden = {
        "teacher_mean",
        "teacher_action",
        "blended_mean",
        "requested_alpha",
        "effective_alpha",
        "safety_latch_active",
    }
    for expected_step, row in enumerate(rows, start=1):
        if not isinstance(row, Mapping):
            schema_exact = False
            identity_exact = False
            action_path_exact = False
            raw_sensor_exact = False
            forbidden_teacher_payload_absent = False
            continue
        identity_exact = identity_exact and (
            row.get("step") == expected_step
            and row.get("schema_version") == config.schema_version
            and row.get("protocol_id") == config.protocol_id
            and row.get("stage_id") == f"development__{case_id}"
            and row.get("case_id") == case_id
        )
        schema_exact = schema_exact and all(
            type(row.get(name)) is int for name in PURE_POLICY_COUNTER_FIELDS
        )
        for name in PURE_POLICY_COUNTER_FIELDS:
            value = row.get(name)
            if type(value) is int:
                counters[name] += value
            per_row_zero_counters = per_row_zero_counters and value == 0
        schema_exact = schema_exact and (
            row.get("teacher_enabled") is False
            and row.get("blending_enabled") is False
            and row.get("safety_latch_enabled") is False
        )
        forbidden_teacher_payload_absent = (
            forbidden_teacher_payload_absent and forbidden.isdisjoint(row)
        )
        mean = row.get("candidate_mean")
        noise = row.get("single_noise")
        action = row.get("raw_action")
        vectors = (mean, noise, action)
        action_path_exact = action_path_exact and all(
            isinstance(value, list)
            and len(value) == EXPECTED_ACTION_DIM
            and all(_finite_number(item) for item in value)
            for value in vectors
        )
        if action_path_exact:
            action_path_exact = all(
                math.isclose(
                    float(action[index]),
                    float(mean[index]) + float(noise[index]),
                    rel_tol=0.0,
                    abs_tol=1.0e-7,
                )
                for index in range(EXPECTED_ACTION_DIM)
            )
        journal = row.get("observer_raw_sensor_journal")
        samples = journal.get("samples") if isinstance(journal, Mapping) else None
        raw_sensor_exact = raw_sensor_exact and (
            row.get("raw_sensor_sample_count") == EXPECTED_RAW_SENSOR_SAMPLES_PER_STEP
            and isinstance(samples, list)
            and len(samples) == EXPECTED_RAW_SENSOR_SAMPLES_PER_STEP
        )
    zero_counters = all(value == 0 for value in counters.values())
    passed = all(
        (
            schema_exact,
            identity_exact,
            action_path_exact,
            raw_sensor_exact,
            forbidden_teacher_payload_absent,
            per_row_zero_counters,
            zero_counters,
        )
    )
    return {
        "passed": passed,
        "row_count": len(rows),
        "schema_exact": schema_exact,
        "identity_exact": identity_exact,
        "candidate_mean_plus_noise_exact": action_path_exact,
        "raw_sensor_samples_exact": raw_sensor_exact,
        "forbidden_teacher_payload_absent": forbidden_teacher_payload_absent,
        "per_row_zero_counters": per_row_zero_counters,
        "zero_counters": zero_counters,
        "counters": counters,
    }


def run_case(
    *,
    config: PhysicalDevelopmentConfig,
    case: Mapping[str, Any],
    destination: str | Path,
    module_path: str | Path,
    activity_callback: ActivityCallback | None = None,
    start_metadata: Mapping[str, Any] | None = None,
    summary_metadata: Mapping[str, Any] | None = None,
    stack_loader: StackLoader | None = None,
    env_config_builder: EnvConfigBuilder | None = None,
) -> dict[str, Any]:
    """Execute and durably journal one 500-step pure-policy physical case.

    The target module may use any hidden width, including the V12R6 512-wide
    composite, as long as the production runtime validates its 35-feature
    actor input and two-action output.  No teacher, blend, latch, learner
    update, gate, receipt, or candidate promotion API is reachable here.

    ``start_metadata`` and ``summary_metadata`` let the owning runner bind a
    frozen candidate and claims without allowing it to overwrite runtime-owned
    fields.  The returned aggregate is in memory and the same trace/summary is
    already durable.  The owning runner must evaluate and publish its gate.
    """

    _validate_config(config)
    if not isinstance(case, Mapping):
        raise V12R6PhysicalDevelopmentError("case must be a mapping")
    canonical_case = copy.deepcopy(dict(case))
    try:
        forensic.canonical_json_bytes(canonical_case)
    except BaseException as exc:
        raise V12R6PhysicalDevelopmentError("case is not finite strict JSON") from exc
    case_id = canonical_case.get("case_id")
    runtime_seed = canonical_case.get("runtime_seed")
    action_selection = canonical_case.get("action_selection")
    if not isinstance(case_id, str) or not case_id:
        raise V12R6PhysicalDevelopmentError("case_id must be a non-empty string")
    if type(runtime_seed) is not int:
        raise V12R6PhysicalDevelopmentError("runtime_seed must be an int")
    if action_selection not in {"deterministic", "stochastic"}:
        raise V12R6PhysicalDevelopmentError(
            "action_selection must be deterministic or stochastic"
        )

    artifact_root = Path(config.artifact_root).expanduser().resolve()
    destination_path = _inside(
        Path(destination), artifact_root, label="development destination"
    )
    if os.path.lexists(destination_path):
        raise V12R6PhysicalDevelopmentError(
            f"development destination already exists: {destination_path}"
        )
    checkpoint = Path(module_path).expanduser().resolve()
    if not checkpoint.is_dir() or checkpoint.is_symlink():
        raise V12R6PhysicalDevelopmentError(
            f"module checkpoint must be a non-symlink directory: {checkpoint}"
        )

    loader = _rollout_stack if stack_loader is None else stack_loader
    builder = (
        env_source.build_env_config
        if env_config_builder is None
        else env_config_builder
    )
    try:
        (
            rollout_eval,
            np,
            torch,
            RLModule,
            env_factory,
            legacy,
            v26_collector,
        ) = loader()
    except V12R6PhysicalDevelopmentError:
        raise
    except Exception as exc:
        raise V12R6PhysicalDevelopmentError("rollout stack loader failed") from exc

    module = RLModule.from_checkpoint(checkpoint)
    eval_mode = getattr(module, "eval", None)
    if not callable(eval_mode):
        raise V12R6PhysicalDevelopmentError("checkpoint module lacks eval()")
    eval_mode()
    innovations = runtime._frozen_innovations(
        case_id,
        action_selection=str(action_selection),
        np=np,
    )
    innovations = np.ascontiguousarray(innovations, dtype=np.float32)
    if innovations.shape != (EXPECTED_STEPS, EXPECTED_ACTION_DIM) or not np.all(
        np.isfinite(innovations)
    ):
        raise V12R6PhysicalDevelopmentError(
            "frozen innovations must be finite float32 [500, 2]"
        )
    built_config = builder(canonical_case)
    if not isinstance(built_config, Mapping):
        raise V12R6PhysicalDevelopmentError("environment config builder was malformed")
    env = env_factory.make_cmc_env(dict(built_config))

    stage_id = f"development__{case_id}"
    writer = forensic.ForensicRolloutWriter(
        destination_path,
        artifact_root=artifact_root,
    )
    run_start = _merge_metadata(
        {
            "schema_version": config.schema_version,
            "status": config.start_status,
            "protocol_id": config.protocol_id,
            "stage_id": stage_id,
            "case": canonical_case,
            "module_checkpoint_path": _module_display_path(checkpoint, artifact_root),
            "teacher_enabled": False,
            "blending_enabled": False,
            "safety_latch_enabled": False,
        },
        start_metadata,
        label="start_metadata",
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
        writer.start(run_start)
        _activity(activity_callback, "environment_reset_calls")
        observation, reset_info = env.reset(seed=runtime_seed)
        if not isinstance(reset_info, Mapping):
            raise V12R6PhysicalDevelopmentError("reset info is malformed")
        observation = np.ascontiguousarray(observation, dtype=np.float32)
        actor_names, full_names = runtime._validate_runtime_layout(
            module=module,
            env=env,
            observation=observation,
            rollout_eval=rollout_eval,
            np=np,
        )
        actor_names = tuple(actor_names)
        full_names = tuple(full_names)
        audit = runtime._new_physical_audit(
            reset_info=reset_info,
            legacy=legacy,
            np=np,
        )
        for index in range(EXPECTED_STEPS):
            step = index + 1
            observation_before = observation.copy()
            actor = np.ascontiguousarray(
                observation_before[:EXPECTED_ACTOR_FEATURES],
                dtype=np.float32,
            )
            if actor.shape != (EXPECTED_ACTOR_FEATURES,):
                raise V12R6PhysicalDevelopmentError(
                    "actor observation must contain exactly 35 features"
                )
            mean, std = runtime._query_mean_std(module, actor, np=np, torch=torch)
            mean = np.ascontiguousarray(mean, dtype=np.float32)
            std = np.ascontiguousarray(std, dtype=np.float32)
            if (
                mean.shape != (EXPECTED_ACTION_DIM,)
                or std.shape != (EXPECTED_ACTION_DIM,)
                or not np.all(np.isfinite(mean))
                or not np.all(np.isfinite(std))
                or np.any(std < 0.0)
            ):
                raise V12R6PhysicalDevelopmentError(
                    "policy output must be finite mean/std vectors of length two"
                )
            noise = np.ascontiguousarray(
                std * innovations[index],
                dtype=np.float32,
            )
            raw_action = np.ascontiguousarray(
                np.add(mean, noise, dtype=np.float32),
                dtype=np.float32,
            )
            if not np.all(np.isfinite(raw_action)):
                raise V12R6PhysicalDevelopmentError("development action is non-finite")
            applied = np.ascontiguousarray(
                np.clip(raw_action, env.action_space.low, env.action_space.high),
                dtype=np.float32,
            )
            _activity(activity_callback, "environment_step_calls")
            observation_after, reward, terminated, truncated, info = env.step(applied)
            observation_after = np.ascontiguousarray(
                observation_after,
                dtype=np.float32,
            )
            if not isinstance(info, Mapping):
                raise V12R6PhysicalDevelopmentError("development info is malformed")
            raw_samples = info.get("binary_phase_sensor_samples")
            if (
                not isinstance(raw_samples, Sequence)
                or isinstance(raw_samples, (str, bytes))
                or len(raw_samples) != EXPECTED_RAW_SENSOR_SAMPLES_PER_STEP
            ):
                raise V12R6PhysicalDevelopmentError(
                    "development step must expose exactly 10 raw sensor samples"
                )
            _activity(
                activity_callback,
                "raw_sensor_sample_count",
                EXPECTED_RAW_SENSOR_SAMPLES_PER_STEP,
            )
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
            if not isinstance(physical, Mapping):
                raise V12R6PhysicalDevelopmentError(
                    "physical step consumer returned a malformed payload"
                )
            row = {
                "schema_version": config.schema_version,
                "protocol_id": config.protocol_id,
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
                "teacher_query_count": 0,
                "served_action_teacher_dependency_count": 0,
                "mean_blend_count": 0,
                "safety_intervention_count": 0,
                "safety_latch_activation_count": 0,
                "safety_latch_release_count": 0,
                "raw_sensor_sample_count": len(raw_samples),
                "reward": _finite_float(reward, label="reward"),
                "time_s": _finite_float(info.get("time"), label="info.time"),
                "grf_penetration_m": _finite_float(
                    physical.get("penetration_m"),
                    label="penetration_m",
                ),
                "reserve_norm_nm": _finite_float(
                    physical.get("reserve_norm_nm"),
                    label="reserve_norm_nm",
                ),
                "residual_norm_nm": _finite_float(
                    physical.get("residual_norm_nm"),
                    label="residual_norm_nm",
                ),
                "phase_fsm": legacy._jsonable(physical.get("phase")),
                "observer_raw_sensor_journal": legacy._jsonable(
                    diagnostic_raw_journal(info, step=step)
                ),
                "checks": legacy._jsonable(physical.get("checks")),
                "terminated": bool(terminated),
                "truncated": bool(truncated),
                "end_reason": info.get("end_reason"),
            }
            writer.write_step(step, row)
            rows.append({"step": step, **row})
            observation = observation_after
            if config.progress_every and (
                step == 1 or step % config.progress_every == 0 or step == EXPECTED_STEPS
            ):
                elapsed = time.monotonic() - started
                eta = elapsed / step * (EXPECTED_STEPS - step)
                print(
                    f"[{config.progress_label}/{case_id}] {step:3d}/"
                    f"{EXPECTED_STEPS} elapsed={elapsed:7.1f}s eta={eta:7.1f}s",
                    flush=True,
                )
            if terminated or truncated:
                break
    finally:
        env.close()

    if audit is None:
        raise V12R6PhysicalDevelopmentError("physical audit was not initialized")
    trace_audit = pure_policy_trace_audit(rows, config=config, case_id=case_id)
    if trace_audit.get("passed") is not True:
        raise V12R6PhysicalDevelopmentError("pure-policy trace audit failed")
    physical_summary = runtime._physical_summary(
        audit,
        case=canonical_case,
        rows=rows,
        info=info,
        terminated=terminated,
        truncated=truncated,
        actor_names=actor_names,
        full_names=full_names,
        legacy=legacy,
        v26_collector=v26_collector,
    )
    if not isinstance(physical_summary, Mapping):
        raise V12R6PhysicalDevelopmentError("physical summary is malformed")
    partial_summary = {
        "schema_version": config.schema_version,
        "status": config.partial_status,
        "protocol_id": config.protocol_id,
        "stage_id": stage_id,
        "steps": len(rows),
        "gate_evaluated": False,
    }
    summary_base = {
        **dict(physical_summary),
        "schema_version": config.schema_version,
        "status": config.complete_status,
        "protocol_id": config.protocol_id,
        "stage_id": stage_id,
        "case": canonical_case,
        "module_checkpoint_path": _module_display_path(checkpoint, artifact_root),
        "teacher_enabled": False,
        "blending_enabled": False,
        "safety_latch_enabled": False,
        "pure_policy_trace_audit": trace_audit,
        "pure_policy_trace_row_count": len(rows),
        **{name: 0 for name in PURE_POLICY_COUNTER_FIELDS},
        "sea_reserve_gate_passed": audit.get("sea_plugin_fallback_count") == 0
        and audit.get("so_solver_unaccepted_count") == 0
        and audit.get("nonfinite_count") == 0,
        "detector_or_fsm_modified": False,
        "q2_paths_opened": [],
        "q3_paths_opened": [],
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
    }
    summary = _merge_metadata(
        summary_base,
        summary_metadata,
        label="summary_metadata",
    )
    published = writer.finalize_before_gate(
        trace=rows,
        partial_summary=partial_summary,
        summary=summary,
    )
    return {
        "case_id": case_id,
        "stage_id": stage_id,
        "destination": destination_path,
        "trace": rows,
        "partial_summary": partial_summary,
        "summary": summary,
        "pure_policy_trace_audit": trace_audit,
        "artifacts": published,
    }


__all__ = [
    "EXPECTED_ACTION_DIM",
    "EXPECTED_ACTOR_FEATURES",
    "EXPECTED_RAW_SENSOR_SAMPLES_PER_STEP",
    "EXPECTED_STEPS",
    "PURE_POLICY_COUNTER_FIELDS",
    "PhysicalDevelopmentConfig",
    "V12R6PhysicalDevelopmentError",
    "diagnostic_raw_journal",
    "pure_policy_trace_audit",
    "run_case",
]
