"""Pure R6 recovery probes and offline same-state H0 labels for V12R7.

The online path in this module loads only the frozen V12R6 candidate.  It
records the V26 actor state and the causal legacy event journal through the
already-audited V12R3 ``PureProbeReplayRecorder``.  A GRF-penetration stop is
therefore a closed, labelable data prefix, not autonomy evidence, but only
after the independent action-path, detector, runtime and teacher-free checks
all pass.

The offline path re-verifies the durable probe closure before the H0 loader is
reachable.  It then delegates coherent V10 shadow replay and one exact
same-state H0 query per row to V12R3's observer label engine.  No environment
object is accepted by the offline API.
"""

from __future__ import annotations

import copy
import math
import os
import sys
import time
from dataclasses import dataclass
from pathlib import Path, PurePosixPath
from typing import Any, Callable, Mapping, Sequence

import numpy as np


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
LOCAL_VALIDATION_ROOT = (
    REPO_ROOT / "Trajectory Generator" / "baseline_MLP" / "validation"
)
REVISION_ROOT = Path(__file__).resolve().parent
for _root in (
    REPO_ROOT,
    REPO_ROOT / "validation",
    REPO_ROOT / "Trajectory Generator",
    REPO_ROOT / "Trajectory Generator" / "baseline_MLP",
    LOCAL_VALIDATION_ROOT,
    LOCAL_VALIDATION_ROOT / "v12r3",
    LOCAL_VALIDATION_ROOT / "v12r6",
    REVISION_ROOT,
):
    if str(_root) not in sys.path:
        sys.path.insert(0, str(_root))

import freeze_h0_v12r6_functional_composite as r6_freezer  # noqa: E402
import h0_forensic_rollout as forensic  # noqa: E402
import h0_primary_split_v12r3_autonomy_recovery_contract as replay_contract  # noqa: E402
import h0_primary_split_v12r3_pure_probe_observer_labeler as observer  # noqa: E402
import h0_v12r6_physical_development as physical  # noqa: E402
import h0_v12r7_recovery_contract as contract  # noqa: E402
import run_h0_primary_split_v9_causal_teacher as env_source  # noqa: E402
import run_h0_primary_split_v12r3_autonomy_recovery as runtime  # noqa: E402


PROBE_BEHAVIOR = "FROZEN_R6_PURE_POLICY_OBSERVER_ONLY"
PROBE_COMPLETE_STATUS = "COMPLETE_H0_V12R7_R6_RECOVERY_PROBE"
LABEL_COMPLETE_STATUS = "COMPLETE_H0_V12R7_SAME_STATE_LABELS"
RECEIPT_STATUS = "PASS_H0_V12R7_RECOVERY_PROBE_CLOSURE"
EXPECTED_CANDIDATE_ID = (
    "AB06_H0_V12R6_FUNCTIONAL_COMPOSITE_A030_W512:"
    + contract.LOCKED_INPUTS["r6_candidate"]["tree_sha256"]
)
ZERO_ONLINE_COUNTERS = (
    "teacher_query_count",
    "served_action_teacher_dependency_count",
    "mean_blend_count",
    "safety_intervention_count",
    "safety_latch_activation_count",
    "safety_latch_release_count",
)
FORBIDDEN_ONLINE_FIELDS = frozenset(
    {
        "teacher_mean",
        "teacher_action",
        "teacher_observation",
        "blended_mean",
        "requested_alpha",
        "effective_alpha",
        "safety_latch_active",
    }
)

ActivityCallback = Callable[[str, int], None]
StackLoader = Callable[[], tuple[Any, Any, Any, Any, Any, Any, Any]]
EnvConfigBuilder = Callable[[Mapping[str, Any]], Mapping[str, Any]]
TreeRecorder = Callable[[str | os.PathLike[str] | PurePosixPath], Mapping[str, Any]]


class V12R7RecoveryProbeError(RuntimeError):
    """Raised when an R7 probe or offline label stage cannot fail closed."""


@dataclass(frozen=True)
class RecoveryProbeConfig:
    """Frozen bindings for one R6-behaviour observer probe.

    ``case_id`` defaults to the critical V12R6 failure.  The same component is
    usable for the remaining preregistered V12R7 collection cases.  The
    candidate digest and all tensor/count dimensions are deliberately fixed.
    """

    case_id: str = "deterministic_offset_plus_0p20"
    artifact_root: Path = REPO_ROOT
    protocol_id: str = contract.PROTOCOL_ID
    schema_version: int = contract.SCHEMA_VERSION
    expected_candidate_tree_sha256: str = contract.LOCKED_INPUTS["r6_candidate"][
        "tree_sha256"
    ]
    expected_candidate_id: str = EXPECTED_CANDIDATE_ID
    event_contract_id: str = contract.EVENT_CONTRACT_ID
    target_contract_id: str = contract.TARGET_CONTRACT_ID
    progress_every: int = 25
    progress_label: str = "V12R7 R6 recovery probe"


def _validate_config(config: RecoveryProbeConfig) -> None:
    if not isinstance(config, RecoveryProbeConfig):
        raise V12R7RecoveryProbeError("config must be RecoveryProbeConfig")
    if config.case_id not in contract.COLLECTION_CASE_IDS:
        raise V12R7RecoveryProbeError(f"unknown V12R7 case: {config.case_id!r}")
    if config.protocol_id != contract.PROTOCOL_ID:
        raise V12R7RecoveryProbeError("protocol identity drifted")
    if type(config.schema_version) is not int or config.schema_version < 1:
        raise V12R7RecoveryProbeError("schema_version must be a positive int")
    if type(config.progress_every) is not int or config.progress_every < 0:
        raise V12R7RecoveryProbeError("progress_every must be non-negative")
    if not isinstance(config.progress_label, str) or not config.progress_label:
        raise V12R7RecoveryProbeError("progress_label must be non-empty")
    expected_sha = contract.LOCKED_INPUTS["r6_candidate"]["tree_sha256"]
    if config.expected_candidate_tree_sha256 != expected_sha:
        raise V12R7RecoveryProbeError("frozen R6 candidate digest drifted")
    if config.expected_candidate_id != EXPECTED_CANDIDATE_ID:
        raise V12R7RecoveryProbeError("frozen R6 candidate identity drifted")
    if (
        config.event_contract_id != contract.EVENT_CONTRACT_ID
        or config.target_contract_id != contract.TARGET_CONTRACT_ID
    ):
        raise V12R7RecoveryProbeError("V26 detector contract drifted")
    root = Path(config.artifact_root).expanduser().resolve()
    if not root.is_dir() or root.is_symlink():
        raise V12R7RecoveryProbeError("artifact_root must be a safe directory")


def _inside(path: str | Path, root: Path, *, label: str) -> Path:
    target = Path(path).expanduser().resolve()
    try:
        target.relative_to(root)
    except ValueError as exc:
        raise V12R7RecoveryProbeError(f"{label} escaped artifact_root") from exc
    return target


def _relative(path: Path, root: Path) -> str:
    return path.resolve().relative_to(root.resolve()).as_posix()


def _artifact(path: Path, root: Path) -> dict[str, Any]:
    try:
        return forensic.artifact_record(path, artifact_root=root)
    except Exception as exc:
        raise V12R7RecoveryProbeError(f"invalid artifact: {path}") from exc


def _strict_mapping(path: Path) -> dict[str, Any]:
    try:
        value = forensic.strict_json_load(path)
    except Exception as exc:
        raise V12R7RecoveryProbeError(f"invalid strict JSON: {path}") from exc
    if not isinstance(value, Mapping):
        raise V12R7RecoveryProbeError(f"expected JSON object: {path}")
    return dict(value)


def _strict_sequence(path: Path) -> list[Any]:
    try:
        value = forensic.strict_json_load(path)
    except Exception as exc:
        raise V12R7RecoveryProbeError(f"invalid strict JSON: {path}") from exc
    if not isinstance(value, list):
        raise V12R7RecoveryProbeError(f"expected JSON array: {path}")
    return value


def _finite_vector(value: Any, *, length: int) -> bool:
    return (
        isinstance(value, list)
        and len(value) == length
        and all(
            not isinstance(item, bool)
            and isinstance(item, (int, float))
            and math.isfinite(float(item))
            for item in value
        )
    )


def _activity(callback: ActivityCallback | None, name: str, amount: int = 1) -> None:
    if type(amount) is not int or amount < 0:
        raise V12R7RecoveryProbeError("activity amount must be non-negative")
    if callback is not None:
        callback(name, amount)


def canonical_probe_case(config: RecoveryProbeConfig) -> dict[str, Any]:
    """Return the R7 collection case after enforcing deterministic plus first."""

    _validate_config(config)
    case = contract.canonical_collection_case(config.case_id)
    if config.case_id == "deterministic_offset_plus_0p20" and (
        case.get("action_selection") != "deterministic"
        or case.get("runtime_seed") != 123
        or case.get("action_seed") is not None
        or float(case.get("sigma", math.nan)) != 0.0
    ):
        raise V12R7RecoveryProbeError("critical plus case drifted")
    return case


def pure_prefix_trace_audit(
    trace: Any,
    *,
    config: RecoveryProbeConfig,
) -> dict[str, Any]:
    """Audit a closed 1..500-row pure-policy trace, including terminal shape."""

    _validate_config(config)
    rows = (
        list(trace)
        if isinstance(trace, Sequence) and not isinstance(trace, (str, bytes))
        else []
    )
    row_count_valid = 1 <= len(rows) <= contract.EXPECTED_STEPS
    identity_exact = row_count_valid
    action_path_exact = row_count_valid
    deterministic_noise_exact = row_count_valid
    sensor_exact = row_count_valid
    teacher_free = row_count_valid
    terminal_shape_exact = row_count_valid
    counters = {name: 0 for name in ZERO_ONLINE_COUNTERS}
    expected_case = canonical_probe_case(config)
    try:
        expected_innovations = np.ascontiguousarray(
            runtime._frozen_innovations(
                config.case_id,
                action_selection=str(expected_case["action_selection"]),
                np=np,
            ),
            dtype=np.float32,
        )
    except Exception:
        expected_innovations = np.empty((0, contract.EXPECTED_ACTION_DIM))
        deterministic_noise_exact = False
    for expected_step, row in enumerate(rows, start=1):
        if not isinstance(row, Mapping):
            identity_exact = False
            action_path_exact = False
            deterministic_noise_exact = False
            sensor_exact = False
            teacher_free = False
            terminal_shape_exact = False
            continue
        identity_exact = identity_exact and (
            row.get("step") == expected_step
            and row.get("schema_version") == config.schema_version
            and row.get("protocol_id") == config.protocol_id
            and row.get("stage_id") == f"collect_label__{config.case_id}"
            and row.get("case_id") == config.case_id
            and row.get("candidate_id") == config.expected_candidate_id
        )
        mean = row.get("candidate_mean")
        std = row.get("candidate_std")
        noise = row.get("single_noise")
        action = row.get("raw_action")
        applied = row.get("applied_action")
        vectors_ok = all(
            _finite_vector(vector, length=contract.EXPECTED_ACTION_DIM)
            for vector in (mean, std, noise, action, applied)
        )
        action_path_exact = action_path_exact and vectors_ok
        if vectors_ok:
            action_path_exact = action_path_exact and all(
                math.isclose(
                    float(action[index]),
                    float(mean[index]) + float(noise[index]),
                    rel_tol=0.0,
                    abs_tol=1.0e-7,
                )
                and math.isclose(
                    float(applied[index]),
                    float(action[index]),
                    rel_tol=0.0,
                    abs_tol=1.0e-7,
                )
                for index in range(contract.EXPECTED_ACTION_DIM)
            )
        standard_normal = row.get("standard_normal")
        innovation_ok = (
            expected_innovations.shape
            == (contract.EXPECTED_STEPS, contract.EXPECTED_ACTION_DIM)
            and _finite_vector(standard_normal, length=contract.EXPECTED_ACTION_DIM)
            and np.asarray(standard_normal, dtype=np.float32).tobytes()
            == expected_innovations[expected_step - 1].tobytes()
        )
        noise_ok = (
            vectors_ok
            and innovation_ok
            and all(
                math.isclose(
                    float(noise[index]),
                    float(std[index]) * float(standard_normal[index]),
                    rel_tol=0.0,
                    abs_tol=1.0e-7,
                )
                and math.isclose(
                    float(std[index]),
                    float(np.float32(contract.EXPECTED_SIGMA)),
                    rel_tol=0.0,
                    abs_tol=1.0e-9,
                )
                for index in range(contract.EXPECTED_ACTION_DIM)
            )
        )
        deterministic_noise_exact = deterministic_noise_exact and noise_ok
        journal = row.get("observer_raw_sensor_journal")
        samples = journal.get("samples") if isinstance(journal, Mapping) else None
        sensor_exact = sensor_exact and (
            row.get("raw_sensor_sample_count") == contract.RAW_SAMPLES_PER_STEP
            and isinstance(samples, list)
            and len(samples) == contract.RAW_SAMPLES_PER_STEP
        )
        teacher_free = teacher_free and FORBIDDEN_ONLINE_FIELDS.isdisjoint(row)
        for name in ZERO_ONLINE_COUNTERS:
            value = row.get(name)
            teacher_free = teacher_free and type(value) is int and value == 0
            if type(value) is int:
                counters[name] += value
        is_last = expected_step == len(rows)
        if is_last:
            physical_stop = (
                row.get("terminated") is True
                and row.get("truncated") is False
                and row.get("end_reason") == "grf_penetration"
            )
            complete_stop = (
                len(rows) == contract.EXPECTED_STEPS
                and row.get("terminated") is False
                and row.get("truncated") is True
                and row.get("end_reason") == "episode_time_limit"
            )
            terminal_shape_exact = terminal_shape_exact and (
                physical_stop or complete_stop
            )
        else:
            terminal_shape_exact = terminal_shape_exact and (
                row.get("terminated") is False and row.get("truncated") is False
            )
    zero_counters = all(value == 0 for value in counters.values())
    passed = all(
        (
            row_count_valid,
            identity_exact,
            action_path_exact,
            deterministic_noise_exact,
            sensor_exact,
            teacher_free,
            terminal_shape_exact,
            zero_counters,
        )
    )
    return {
        "passed": passed,
        "row_count": len(rows),
        "row_count_valid": row_count_valid,
        "identity_exact": identity_exact,
        "candidate_mean_plus_single_noise_exact": action_path_exact,
        "frozen_innovation_and_single_noise_exact": deterministic_noise_exact,
        "raw_sensor_samples_exact": sensor_exact,
        "teacher_free": teacher_free,
        "terminal_shape_exact": terminal_shape_exact,
        "zero_online_counters": zero_counters,
        "counters": counters,
    }


def r6_plus_reproduction_audit(
    trace: Any,
    *,
    historical_steps_root: str | Path | None = None,
) -> dict[str, Any]:
    """Compare a new critical-plus trace with the immutable 179-row R6 prefix.

    The hook is intentionally independent of the generic collection gate.  It
    is mandatory for the critical plus case in :func:`run_recovery_probe` and
    can also be called during preflight to prove that the added recorder has
    not changed the R6 behaviour-policy trajectory.
    """

    rows = (
        list(trace)
        if isinstance(trace, Sequence) and not isinstance(trace, (str, bytes))
        else []
    )
    default_root = REPO_ROOT.joinpath(
        *contract.R6_RUN_ROOT.parts,
        "development",
        "deterministic_offset_plus_0p20",
        "steps",
    )
    source = Path(historical_steps_root or default_root).expanduser().resolve()
    paths = sorted(source.glob("[0-9][0-9][0-9][0-9][0-9][0-9].json"))
    fields = (
        "v26_observation",
        "candidate_mean",
        "candidate_std",
        "standard_normal",
        "single_noise",
        "raw_action",
        "applied_action",
        "raw_sensor_sample_count",
        "observer_raw_sensor_journal",
        "reward",
        "time_s",
        "grf_penetration_m",
        "reserve_norm_nm",
        "residual_norm_nm",
        "phase_fsm",
        "checks",
        "terminated",
        "truncated",
        "end_reason",
    )
    mismatch_step: int | None = None
    mismatch_field: str | None = None
    source_valid = len(paths) == 179
    if source_valid:
        for index, path in enumerate(paths, start=1):
            try:
                historical = _strict_mapping(path)
            except V12R7RecoveryProbeError:
                source_valid = False
                break
            if historical.get("step") != index:
                source_valid = False
                break
            if index > len(rows) or not isinstance(rows[index - 1], Mapping):
                mismatch_step = index
                mismatch_field = "row_presence"
                break
            current = rows[index - 1]
            for field in fields:
                try:
                    same = forensic.canonical_json_bytes(current.get(field)) == (
                        forensic.canonical_json_bytes(historical.get(field))
                    )
                except Exception:
                    same = False
                if not same:
                    mismatch_step = index
                    mismatch_field = field
                    break
            if mismatch_step is not None:
                break
    row_count_exact = len(rows) == len(paths) == 179
    passed = source_valid and row_count_exact and mismatch_step is None
    return {
        "passed": passed,
        "historical_step_count": len(paths),
        "current_step_count": len(rows),
        "historical_source_valid": source_valid,
        "row_count_exact": row_count_exact,
        "shared_runtime_fields_exact": mismatch_step is None,
        "first_mismatch_step": mismatch_step,
        "first_mismatch_field": mismatch_field,
    }


def recovery_prefix_gate(
    summary: Mapping[str, Any],
    *,
    trace: Any,
    replay: observer.LoadedReplay,
    config: RecoveryProbeConfig,
) -> dict[str, Any]:
    """Accept a physical prefix for labels, never as candidate autonomy."""

    _validate_config(config)
    if not isinstance(summary, Mapping):
        raise V12R7RecoveryProbeError("summary must be a mapping")
    trace_audit = pure_prefix_trace_audit(trace, config=config)
    contract_gate = contract.collection_integrity_gate(summary)
    reproduction_audit = (
        r6_plus_reproduction_audit(trace)
        if config.case_id == "deterministic_offset_plus_0p20"
        else {
            "passed": True,
            "applicable": False,
            "reason": "not_the_historical_r6_critical_plus_case",
        }
    )
    steps = summary.get("steps")
    binary_prefix = summary.get("binary_event_prefix_integrity")
    replay_trace_exact = False
    penetration_exact = False
    rows = list(trace) if isinstance(trace, list) else []
    if type(steps) is int and steps == len(rows) == replay.n_steps:
        try:
            trace_actor = np.ascontiguousarray(
                [row["v26_observation"] for row in rows], dtype=np.float32
            )
            trace_penetration = np.asarray(
                [row["previous_penetration_m"] for row in rows], dtype=np.float64
            )
            replay_trace_exact = trace_actor.shape == (
                steps,
                contract.EXPECTED_ACTOR_FEATURES,
            ) and trace_actor.tobytes(order="C") == replay.arrays[
                "actor_observations"
            ].tobytes(order="C")
            penetration_exact = trace_penetration.shape == (
                steps,
            ) and trace_penetration.tobytes(order="C") == replay.arrays[
                "previous_penetration_m"
            ].tobytes(order="C")
        except (KeyError, TypeError, ValueError):
            replay_trace_exact = False
            penetration_exact = False
    detector_anomalies = (
        "fallback_count",
        "hard_invalid_count",
        "duplicate_event_count",
        "out_of_order_event_count",
        "left_non_v26_source_count",
        "invalid_event_count",
        "routing_failure_count",
        "step_contract_failure_count",
        "binary_event_failure_count",
    )
    runtime_anomalies = (
        "action_clipped_values",
        "nonfinite_count",
        "sea_plugin_fallback_count",
        "so_solver_unaccepted_count",
    )
    physical_prefix = (
        type(steps) is int
        and contract.MINIMUM_RECOVERABLE_PREFIX_STEPS <= steps < contract.EXPECTED_STEPS
        and summary.get("end_reason") == "grf_penetration"
        and summary.get("terminated") is True
        and summary.get("truncated") is False
        and summary.get("safety_stop_count") == 1
        and isinstance(summary.get("grf_penetration_max_m"), (int, float))
        and not isinstance(summary.get("grf_penetration_max_m"), bool)
        and math.isfinite(float(summary["grf_penetration_max_m"]))
        and float(summary["grf_penetration_max_m"]) >= contract.PENETRATION_LIMIT_M
    )
    complete_episode = (
        steps == contract.EXPECTED_STEPS
        and summary.get("end_reason") == "episode_time_limit"
        and summary.get("terminated") is False
        and summary.get("truncated") is True
        and summary.get("safety_stop_count") == 0
    )
    checks = {
        "contract_integrity": contract_gate.get("passed") is True,
        "contract_closed_trajectory": contract_gate.get("recoverable_prefix") is True
        or complete_episode,
        "trace_pure_and_closed": trace_audit.get("passed") is True,
        "r6_plus_reproduced_when_applicable": reproduction_audit.get("passed") is True
        and (
            config.case_id != "deterministic_offset_plus_0p20"
            or summary.get("r6_plus_reproduction_audit") == reproduction_audit
        ),
        "physical_prefix_or_complete_episode": physical_prefix or complete_episode,
        "candidate_exact": summary.get("candidate_tree_sha256")
        == config.expected_candidate_tree_sha256
        and summary.get("candidate_id") == config.expected_candidate_id,
        "detector_active": summary.get("binary_phase_fsm_mode") == "binary_active"
        and summary.get("event_contract_id") == config.event_contract_id
        and summary.get("target_contract_id") == config.target_contract_id,
        "detector_prefix_integrity": isinstance(binary_prefix, Mapping)
        and binary_prefix.get("passed") is True
        and binary_prefix.get("sample_count") == steps * contract.RAW_SAMPLES_PER_STEP,
        "zero_detector_anomalies": all(
            type(summary.get(name)) is int and summary[name] == 0
            for name in detector_anomalies
        ),
        "zero_runtime_anomalies": all(
            type(summary.get(name)) is int and summary[name] == 0
            for name in runtime_anomalies
        ),
        "teacher_free_summary": summary.get("teacher_enabled") is False
        and summary.get("teacher_loaded_during_rollout") is False
        and summary.get("blending_enabled") is False
        and summary.get("safety_latch_enabled") is False
        and all(
            type(summary.get(name)) is int and summary[name] == 0
            for name in ZERO_ONLINE_COUNTERS
        ),
        "replay_shape": type(steps) is int
        and replay.n_steps == steps
        and replay.boundary_count == steps + 1,
        "replay_event_contract": replay.event_contract_id == config.event_contract_id,
        "replay_trace_actor_byte_exact": replay_trace_exact,
        "replay_trace_penetration_byte_exact": penetration_exact,
        "zero_updates": summary.get("actor_updates") == 0
        and type(summary.get("actor_updates")) is int
        and type(summary.get("critic_updates")) is int
        and summary.get("critic_updates") == 0
        and type(summary.get("ppo_updates")) is int
        and summary.get("ppo_updates") == 0,
        "morphology_disabled": isinstance(
            summary.get("morphology_weight"), (int, float)
        )
        and not isinstance(summary.get("morphology_weight"), bool)
        and float(summary["morphology_weight"]) == 0.0,
    }
    passed = all(value is True for value in checks.values())
    return {
        "schema_version": config.schema_version,
        "status": (
            "PASS_H0_V12R7_RECOVERABLE_R6_PREFIX"
            if passed
            else "FAIL_H0_V12R7_R6_PREFIX_INTEGRITY"
        ),
        "passed": passed,
        "integrity_passed": passed,
        "recoverable_for_observer_label": passed,
        "recoverable_prefix": bool(passed and physical_prefix),
        "complete_episode": bool(passed and complete_episode),
        "autonomy_passed": False,
        "protocol_id": config.protocol_id,
        "case_id": config.case_id,
        "probe_step_count": steps if type(steps) is int else None,
        "candidate_id": config.expected_candidate_id,
        "candidate_tree_sha256": config.expected_candidate_tree_sha256,
        "checks": checks,
        "trace_audit": trace_audit,
        "r6_plus_reproduction_audit": reproduction_audit,
        "contract_gate": contract_gate,
        "next_stage": (
            "OFFLINE_COHERENT_H0_LABEL_REQUIRED" if passed else "STOP_INTEGRITY_FAILURE"
        ),
    }


def _default_tree_record(path: Path) -> Mapping[str, Any]:
    try:
        relative = path.resolve().relative_to(REPO_ROOT).as_posix()
    except ValueError as exc:
        raise V12R7RecoveryProbeError("candidate escaped repository root") from exc
    return r6_freezer.tree_record(relative)


def run_recovery_probe(
    *,
    config: RecoveryProbeConfig,
    destination: str | Path,
    module_path: str | Path | None = None,
    activity_callback: ActivityCallback | None = None,
    stack_loader: StackLoader | None = None,
    env_config_builder: EnvConfigBuilder | None = None,
    tree_recorder: Callable[[Path], Mapping[str, Any]] | None = None,
) -> dict[str, Any]:
    """Run one frozen-R6 pure probe and publish its recoverable-prefix gate.

    Production callers omit all dependency hooks.  Tests may inject the stack,
    environment config builder and read-only tree recorder; no hook can inject
    a teacher, blend, latch or action-serving callback.
    """

    _validate_config(config)
    artifact_root = Path(config.artifact_root).expanduser().resolve()
    root = _inside(destination, artifact_root, label="probe destination")
    if os.path.lexists(root):
        raise V12R7RecoveryProbeError(f"probe destination exists: {root}")
    candidate_path = (
        Path(
            module_path
            if module_path is not None
            else REPO_ROOT.joinpath(*contract.R6_CANDIDATE_MODULE_PATH.parts)
        )
        .expanduser()
        .resolve()
    )
    if not candidate_path.is_dir() or candidate_path.is_symlink():
        raise V12R7RecoveryProbeError("candidate checkpoint is missing or unsafe")
    record_tree = tree_recorder or _default_tree_record
    try:
        candidate_tree = dict(record_tree(candidate_path))
    except Exception as exc:
        raise V12R7RecoveryProbeError("candidate tree attestation failed") from exc
    if (
        candidate_tree.get("tree_sha256") != config.expected_candidate_tree_sha256
        or candidate_tree.get("file_count")
        != contract.LOCKED_INPUTS["r6_candidate"]["file_count"]
    ):
        raise V12R7RecoveryProbeError("candidate is not the frozen V12R6 actor")

    case = canonical_probe_case(config)
    loader = physical._rollout_stack if stack_loader is None else stack_loader
    builder = (
        env_source.build_env_config
        if env_config_builder is None
        else env_config_builder
    )
    try:
        (
            rollout_eval,
            np_runtime,
            torch,
            RLModule,
            env_factory,
            legacy,
            v26_collector,
        ) = loader()
    except Exception as exc:
        raise V12R7RecoveryProbeError("production rollout stack is not ready") from exc
    candidate = RLModule.from_checkpoint(candidate_path)
    eval_mode = getattr(candidate, "eval", None)
    if not callable(eval_mode):
        raise V12R7RecoveryProbeError("candidate lacks eval()")
    eval_mode()
    innovations = runtime._frozen_innovations(
        config.case_id,
        action_selection=str(case["action_selection"]),
        np=np_runtime,
    )
    innovations = np_runtime.ascontiguousarray(innovations, dtype=np_runtime.float32)
    if innovations.shape != (contract.EXPECTED_STEPS, contract.EXPECTED_ACTION_DIM):
        raise V12R7RecoveryProbeError("frozen innovations shape drifted")
    if case["action_selection"] == "deterministic" and np_runtime.any(innovations != 0):
        raise V12R7RecoveryProbeError("deterministic innovations are not exact zero")
    built_config = builder(case)
    if not isinstance(built_config, Mapping):
        raise V12R7RecoveryProbeError("environment config builder was malformed")
    env = env_factory.make_cmc_env(dict(built_config))
    writer = forensic.ForensicRolloutWriter(root, artifact_root=artifact_root)
    stage_id = f"collect_label__{config.case_id}"
    writer.start(
        {
            "schema_version": config.schema_version,
            "status": "STARTED_H0_V12R7_R6_RECOVERY_PROBE",
            "protocol_id": config.protocol_id,
            "stage_id": stage_id,
            "case": copy.deepcopy(case),
            "behavior": PROBE_BEHAVIOR,
            "candidate_id": config.expected_candidate_id,
            "candidate_module": candidate_tree,
            "teacher_enabled": False,
            "teacher_loaded_during_rollout": False,
            "blending_enabled": False,
            "safety_latch_enabled": False,
            "actor_updates": 0,
            "critic_updates": 0,
            "ppo_updates": 0,
        }
    )
    rows: list[dict[str, Any]] = []
    actor_names: tuple[str, ...] = ()
    full_names: tuple[str, ...] = ()
    audit: dict[str, Any] | None = None
    replay_recorder: observer.PureProbeReplayRecorder | None = None
    info: Mapping[str, Any] = {}
    terminated = False
    truncated = False
    started = time.monotonic()
    try:
        _activity(activity_callback, "environment_reset_calls")
        observation, reset_info = env.reset(seed=int(case["runtime_seed"]))
        if not isinstance(reset_info, Mapping):
            raise V12R7RecoveryProbeError("reset info is malformed")
        observation = np_runtime.ascontiguousarray(
            observation, dtype=np_runtime.float32
        )
        actor_names, full_names = runtime._validate_runtime_layout(
            module=candidate,
            env=env,
            observation=observation,
            rollout_eval=rollout_eval,
            np=np_runtime,
        )
        actor_names = tuple(actor_names)
        full_names = tuple(full_names)
        audit = runtime._new_physical_audit(
            reset_info=reset_info,
            legacy=legacy,
            np=np_runtime,
        )
        body_weight_n = float(env.unwrapped._body_weight_n)
        if not math.isfinite(body_weight_n) or body_weight_n <= 0.0:
            raise V12R7RecoveryProbeError("body weight is malformed")
        replay_recorder = observer.PureProbeReplayRecorder.from_runtime(
            env.unwrapped._phase_fsm,
            body_weight_n,
            actor_names,
            event_contract_id=config.event_contract_id,
        )
        actor = np_runtime.ascontiguousarray(
            observation[: contract.EXPECTED_ACTOR_FEATURES],
            dtype=np_runtime.float32,
        )
        replay_recorder.record_reset(actor, reset_info, previous_penetration_m=0.0)
        previous_penetration = 0.0
        for index in range(contract.EXPECTED_STEPS):
            step = index + 1
            observation_before = observation.copy()
            actor_before = actor.copy()
            mean, std = runtime._query_mean_std(
                candidate, actor_before, np=np_runtime, torch=torch
            )
            mean = np_runtime.ascontiguousarray(mean, dtype=np_runtime.float32)
            std = np_runtime.ascontiguousarray(std, dtype=np_runtime.float32)
            noise = np_runtime.ascontiguousarray(
                std * innovations[index], dtype=np_runtime.float32
            )
            raw_action = np_runtime.ascontiguousarray(
                np_runtime.add(mean, noise, dtype=np_runtime.float32),
                dtype=np_runtime.float32,
            )
            applied = np_runtime.ascontiguousarray(
                np_runtime.clip(
                    raw_action, env.action_space.low, env.action_space.high
                ),
                dtype=np_runtime.float32,
            )
            _activity(activity_callback, "environment_step_calls")
            observation_after, reward, terminated, truncated, info = env.step(applied)
            observation_after = np_runtime.ascontiguousarray(
                observation_after, dtype=np_runtime.float32
            )
            if not isinstance(info, Mapping):
                raise V12R7RecoveryProbeError("step info is malformed")
            raw_samples = info.get("binary_phase_sensor_samples")
            if (
                not isinstance(raw_samples, Sequence)
                or isinstance(raw_samples, (str, bytes))
                or len(raw_samples) != contract.RAW_SAMPLES_PER_STEP
            ):
                raise V12R7RecoveryProbeError(
                    "each step must expose exactly ten raw detector samples"
                )
            _activity(
                activity_callback,
                "raw_sensor_sample_count",
                contract.RAW_SAMPLES_PER_STEP,
            )
            physical_step = runtime._consume_physical_step(
                audit,
                step=step,
                info=info,
                observation_before=observation_before,
                observation_after=observation_after,
                reward=reward,
                action=raw_action,
                applied_action=applied,
                extra_vectors=(actor_before, mean, std, noise),
                legacy=legacy,
                v26_collector=v26_collector,
            )
            if not isinstance(physical_step, Mapping):
                raise V12R7RecoveryProbeError("physical step audit is malformed")
            next_actor = np_runtime.ascontiguousarray(
                observation_after[: contract.EXPECTED_ACTOR_FEATURES],
                dtype=np_runtime.float32,
            )
            terminal_boundary = bool(
                terminated or truncated or step == contract.EXPECTED_STEPS
            )
            replay_recorder.record_step_boundary(
                None if terminal_boundary else next_actor,
                info,
                previous_penetration_m=(
                    None if terminal_boundary else float(physical_step["penetration_m"])
                ),
            )
            row = {
                "schema_version": config.schema_version,
                "protocol_id": config.protocol_id,
                "stage_id": stage_id,
                "case_id": config.case_id,
                "candidate_id": config.expected_candidate_id,
                "v26_observation": actor_before.tolist(),
                "candidate_mean": mean.tolist(),
                "candidate_std": std.tolist(),
                "standard_normal": innovations[index].tolist(),
                "single_noise": noise.tolist(),
                "raw_action": raw_action.tolist(),
                "applied_action": applied.tolist(),
                "teacher_enabled": False,
                "teacher_query_count": 0,
                "served_action_teacher_dependency_count": 0,
                "blending_enabled": False,
                "mean_blend_count": 0,
                "safety_latch_enabled": False,
                "safety_intervention_count": 0,
                "safety_latch_activation_count": 0,
                "safety_latch_release_count": 0,
                "previous_penetration_m": previous_penetration,
                "legacy_online_events": legacy._jsonable(
                    info.get("legacy_online_events", [])
                ),
                "raw_sensor_sample_count": len(raw_samples),
                "observer_raw_sensor_journal": legacy._jsonable(
                    physical.diagnostic_raw_journal(info, step=step)
                ),
                "reward": float(reward),
                "time_s": float(info.get("time")),
                "grf_penetration_m": float(physical_step["penetration_m"]),
                "reserve_norm_nm": float(physical_step["reserve_norm_nm"]),
                "residual_norm_nm": float(physical_step["residual_norm_nm"]),
                "phase_fsm": legacy._jsonable(physical_step["phase"]),
                "checks": legacy._jsonable(physical_step["checks"]),
                "terminated": bool(terminated),
                "truncated": bool(truncated),
                "end_reason": info.get("end_reason"),
            }
            writer.write_step(step, row)
            rows.append({"step": step, **row})
            previous_penetration = float(physical_step["penetration_m"])
            observation = observation_after
            actor = next_actor
            if config.progress_every and (
                step == 1 or step % config.progress_every == 0 or terminal_boundary
            ):
                elapsed = time.monotonic() - started
                eta = elapsed / step * (contract.EXPECTED_STEPS - step)
                print(
                    f"[{config.progress_label}/{config.case_id}] "
                    f"{step:3d}/{contract.EXPECTED_STEPS} "
                    f"elapsed={elapsed:7.1f}s eta={eta:7.1f}s",
                    flush=True,
                )
            if terminal_boundary:
                break
    except BaseException as exc:
        try:
            writer.publish_failure(
                end_reason="v12r7_recovery_probe_failed",
                error=exc,
                status="FAIL_H0_V12R7_RECOVERY_PROBE_RUNTIME",
            )
        except Exception:
            pass
        raise
    finally:
        env.close()
    if audit is None or replay_recorder is None or not rows:
        raise V12R7RecoveryProbeError("probe audit/replay was not initialized")

    replay_path = root / "replay_boundaries.npz"
    replay_recorder.write_exclusive(replay_path)
    loaded_replay = observer.load_probe_replay_strict(
        replay_path, contract_module=replay_contract
    )
    physical_summary = runtime._physical_summary(
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
    if not isinstance(physical_summary, Mapping):
        raise V12R7RecoveryProbeError("physical summary is malformed")
    reproduction_audit = (
        r6_plus_reproduction_audit(rows)
        if config.case_id == "deterministic_offset_plus_0p20"
        else {
            "passed": True,
            "applicable": False,
            "reason": "not_the_historical_r6_critical_plus_case",
        }
    )
    summary = {
        **dict(physical_summary),
        "schema_version": config.schema_version,
        "status": PROBE_COMPLETE_STATUS,
        "protocol_id": config.protocol_id,
        "stage_id": stage_id,
        "case_id": config.case_id,
        "behavior": PROBE_BEHAVIOR,
        "candidate_id": config.expected_candidate_id,
        "candidate_module": candidate_tree,
        "candidate_tree_sha256": candidate_tree["tree_sha256"],
        "trace_step_count": len(rows),
        "replay_step_count": loaded_replay.n_steps,
        "replay_boundary_count": loaded_replay.boundary_count,
        "replay_event_count": loaded_replay.event_count,
        "replay_payload": _artifact(replay_path, artifact_root),
        "r6_plus_reproduction_audit": reproduction_audit,
        "teacher_enabled": False,
        "teacher_loaded_during_rollout": False,
        "blending_enabled": False,
        "safety_latch_enabled": False,
        **{name: 0 for name in ZERO_ONLINE_COUNTERS},
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
    }
    partial = {
        "schema_version": config.schema_version,
        "status": "PERSISTED_H0_V12R7_PROBE_BEFORE_GATE",
        "protocol_id": config.protocol_id,
        "stage_id": stage_id,
        "steps": len(rows),
        "gate_evaluated": False,
        "teacher_query_count": 0,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
    }
    artifacts = writer.finalize_before_gate(
        trace=rows,
        partial_summary=partial,
        summary=summary,
    )
    gate = recovery_prefix_gate(
        summary,
        trace=rows,
        replay=loaded_replay,
        config=config,
    )
    gate_record = writer.publish_gate(gate)
    receipt = {
        "schema_version": config.schema_version,
        "status": RECEIPT_STATUS if gate["passed"] else gate["status"],
        "passed": gate["passed"],
        "integrity_passed": gate["integrity_passed"],
        "recoverable_for_observer_label": gate["recoverable_for_observer_label"],
        "autonomy_passed": False,
        "protocol_id": config.protocol_id,
        "stage_id": stage_id,
        "case_id": config.case_id,
        "probe_step_count": len(rows),
        "candidate_id": config.expected_candidate_id,
        "candidate_tree_sha256": config.expected_candidate_tree_sha256,
        "run_start": _artifact(root / "run_start.json", artifact_root),
        "trace": artifacts["trace"],
        "partial_summary": artifacts["partial_summary"],
        "summary": artifacts["summary"],
        "gate": gate_record,
        "replay_payload": _artifact(replay_path, artifact_root),
        "r6_plus_reproduction_audit": reproduction_audit,
        "teacher_query_count": 0,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
    }
    forensic.write_json_exclusive(root / "receipt.json", receipt)
    verify_probe_closure(root, config=config)
    return {
        "passed": gate["passed"],
        "destination": root,
        "summary": summary,
        "gate": gate,
        "receipt": receipt,
        "replay": loaded_replay,
    }


def verify_probe_closure(
    probe_destination: str | Path,
    *,
    config: RecoveryProbeConfig,
    expected_replay: observer.LoadedReplay | None = None,
) -> dict[str, Any]:
    """Re-read, hash-bind and recompute one closed recoverable probe."""

    _validate_config(config)
    artifact_root = Path(config.artifact_root).expanduser().resolve()
    root = _inside(probe_destination, artifact_root, label="probe destination")
    summary = _strict_mapping(root / "summary.json")
    trace = _strict_sequence(root / "trace.json")
    persisted_gate = _strict_mapping(root / "gate.json")
    receipt = _strict_mapping(root / "receipt.json")
    replay = observer.load_probe_replay_strict(
        root / "replay_boundaries.npz", contract_module=replay_contract
    )
    recomputed_gate = recovery_prefix_gate(
        summary,
        trace=trace,
        replay=replay,
        config=config,
    )
    if persisted_gate != recomputed_gate or recomputed_gate.get("passed") is not True:
        raise V12R7RecoveryProbeError("recovery prefix gate is not closed PASS")
    expected_records = {
        "run_start": _artifact(root / "run_start.json", artifact_root),
        "trace": _artifact(root / "trace.json", artifact_root),
        "partial_summary": _artifact(root / "partial_summary.json", artifact_root),
        "summary": _artifact(root / "summary.json", artifact_root),
        "gate": _artifact(root / "gate.json", artifact_root),
        "replay_payload": _artifact(root / "replay_boundaries.npz", artifact_root),
    }
    if any(receipt.get(name) != record for name, record in expected_records.items()):
        raise V12R7RecoveryProbeError("probe receipt artifact binding drifted")
    scalar_checks = {
        "passed": True,
        "integrity_passed": True,
        "recoverable_for_observer_label": True,
        "autonomy_passed": False,
        "protocol_id": config.protocol_id,
        "case_id": config.case_id,
        "probe_step_count": replay.n_steps,
        "candidate_id": config.expected_candidate_id,
        "candidate_tree_sha256": config.expected_candidate_tree_sha256,
    }
    if any(receipt.get(name) != value for name, value in scalar_checks.items()):
        raise V12R7RecoveryProbeError("probe receipt semantic binding drifted")
    if any(
        type(receipt.get(name)) is not int or receipt[name] != 0
        for name in (
            "teacher_query_count",
            "actor_updates",
            "critic_updates",
            "ppo_updates",
        )
    ):
        raise V12R7RecoveryProbeError("probe receipt zero counters drifted")
    if receipt.get("r6_plus_reproduction_audit") != persisted_gate.get(
        "r6_plus_reproduction_audit"
    ):
        raise V12R7RecoveryProbeError("R6 plus reproduction receipt drifted")
    if expected_replay is not None:
        if expected_replay.n_steps != replay.n_steps or any(
            expected_replay.arrays[name].tobytes(order="C")
            != replay.arrays[name].tobytes(order="C")
            for name in observer.REPLAY_ARRAY_NAMES
        ):
            raise V12R7RecoveryProbeError("probe replay changed across closure check")
    return {
        "passed": True,
        "summary": summary,
        "trace": trace,
        "gate": persisted_gate,
        "receipt": receipt,
        "replay": replay,
    }


def label_recovery_probe(
    *,
    config: RecoveryProbeConfig,
    probe_destination: str | Path,
    label_destination: str | Path,
    source_h0_path: str | Path,
    module_loader: Callable[[Path], Any] | None = None,
    mean_query: Callable[[Any, np.ndarray], Any] | None = None,
    coverage_evaluator: Callable[[np.ndarray], Any] | None = None,
    phase_fsm_factory: Callable[[Mapping[str, Any], str, str], Any] | None = None,
) -> dict[str, Any]:
    """Offline replay and persist exact same-state H0 labels for a PASS prefix."""

    _validate_config(config)
    artifact_root = Path(config.artifact_root).expanduser().resolve()
    probe_root = _inside(probe_destination, artifact_root, label="probe destination")
    label_root = _inside(label_destination, artifact_root, label="label destination")
    expected_label_root = probe_root / "observer_labels"
    if label_root != expected_label_root:
        raise V12R7RecoveryProbeError(
            "label destination must be <probe_destination>/observer_labels"
        )
    if label_root.exists():
        raise V12R7RecoveryProbeError(f"label destination exists: {label_root}")
    label_artifact_names = (
        "labels.npz",
        "summary.json",
        "gate.json",
        "receipt.json",
    )
    if any(os.path.lexists(label_root / name) for name in label_artifact_names):
        raise V12R7RecoveryProbeError("label artifacts already exist")
    closure = verify_probe_closure(probe_root, config=config)

    def closure_validator(replay: observer.LoadedReplay) -> None:
        verify_probe_closure(
            probe_root,
            config=config,
            expected_replay=replay,
        )

    result = observer.label_closed_probe_in_memory(
        probe_root / "replay_boundaries.npz",
        probe_closure_validator=closure_validator,
        source_h0_path=source_h0_path,
        case_id=config.case_id,
        probe_stage="p0",
        module_loader=module_loader,
        mean_query=mean_query,
        coverage_evaluator=coverage_evaluator,
        phase_fsm_factory=phase_fsm_factory,
        contract_module=replay_contract,
    )
    if (
        result.environment_reset_calls != 0
        or result.environment_step_calls != 0
        or result.action_served_count != 0
        or result.teacher_query_count != result.replay.n_steps
        or result.arrays["observations"].tobytes(order="C")
        != result.replay.arrays["actor_observations"].tobytes(order="C")
    ):
        raise V12R7RecoveryProbeError("offline same-state label invariant failed")
    label_root.mkdir(parents=True, exist_ok=False)
    labels_path = label_root / "labels.npz"
    observer.write_npz_exclusive(labels_path, result.arrays)
    rows = result.replay.n_steps
    summary = {
        "schema_version": config.schema_version,
        "status": LABEL_COMPLETE_STATUS,
        "protocol_id": config.protocol_id,
        "stage_id": f"label__{config.case_id}",
        "case_id": config.case_id,
        "collection_integrity_passed": closure["gate"]["passed"],
        "labelled_row_count": rows,
        "same_state_teacher_label_count": rows,
        "teacher_query_count": result.teacher_query_count,
        "environment_reset_calls": result.environment_reset_calls,
        "environment_step_calls": result.environment_step_calls,
        "action_served_count": result.action_served_count,
        "observation_shape": list(result.arrays["observations"].shape),
        "action_shape": list(result.arrays["actions"].shape),
        "nonfinite_count": int(
            np.count_nonzero(~np.isfinite(result.arrays["observations"]))
            + np.count_nonzero(~np.isfinite(result.arrays["actions"]))
        ),
        "replayed_boundary_count": result.teacher_views.replayed_boundary_count,
        "changed_only_mutable_count": result.teacher_views.changed_only_mutable_count,
        "invariant_columns_byte_exact_count": (
            result.teacher_views.invariant_columns_byte_exact_count
        ),
        "column_24_changed_count": result.teacher_views.column_24_changed_count,
        "coverage_audit": copy.deepcopy(dict(result.coverage.audit)),
        "probe_receipt": _artifact(probe_root / "receipt.json", artifact_root),
        "probe_replay": _artifact(probe_root / "replay_boundaries.npz", artifact_root),
        "labels": _artifact(labels_path, artifact_root),
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
    }
    label_gate = contract.label_gate(summary)
    label_summary_path = label_root / "summary.json"
    label_gate_path = label_root / "gate.json"
    label_receipt_path = label_root / "receipt.json"
    forensic.write_json_exclusive(label_summary_path, summary)
    forensic.write_json_exclusive(label_gate_path, label_gate)
    receipt = {
        "schema_version": config.schema_version,
        "status": label_gate["status"],
        "passed": label_gate["passed"],
        "protocol_id": config.protocol_id,
        "stage_id": f"label__{config.case_id}",
        "case_id": config.case_id,
        "labelled_row_count": rows,
        "same_state_teacher_label_count": rows,
        "teacher_query_count": result.teacher_query_count,
        "environment_reset_calls": 0,
        "environment_step_calls": 0,
        "action_served_count": 0,
        "probe_receipt": _artifact(probe_root / "receipt.json", artifact_root),
        "probe_replay": _artifact(probe_root / "replay_boundaries.npz", artifact_root),
        "labels": _artifact(labels_path, artifact_root),
        "summary": _artifact(label_summary_path, artifact_root),
        "gate": _artifact(label_gate_path, artifact_root),
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
    }
    forensic.write_json_exclusive(label_receipt_path, receipt)
    if label_gate.get("passed") is not True:
        raise V12R7RecoveryProbeError("offline observer label gate failed")
    return {
        "passed": True,
        "destination": label_root,
        "summary": summary,
        "gate": label_gate,
        "receipt": receipt,
        "result": result,
    }


def run_probe_and_label(
    *,
    config: RecoveryProbeConfig,
    probe_destination: str | Path,
    label_destination: str | Path,
    source_h0_path: str | Path,
    module_path: str | Path | None = None,
    activity_callback: ActivityCallback | None = None,
) -> dict[str, Any]:
    """Production convenience API: close pure probe, then label it offline."""

    probe = run_recovery_probe(
        config=config,
        destination=probe_destination,
        module_path=module_path,
        activity_callback=activity_callback,
    )
    if probe.get("passed") is not True:
        raise V12R7RecoveryProbeError("probe prefix is not labelable")
    labels = label_recovery_probe(
        config=config,
        probe_destination=probe_destination,
        label_destination=label_destination,
        source_h0_path=source_h0_path,
    )
    return {"passed": True, "probe": probe, "labels": labels}


__all__ = [
    "EXPECTED_CANDIDATE_ID",
    "FORBIDDEN_ONLINE_FIELDS",
    "LABEL_COMPLETE_STATUS",
    "PROBE_BEHAVIOR",
    "PROBE_COMPLETE_STATUS",
    "RecoveryProbeConfig",
    "V12R7RecoveryProbeError",
    "canonical_probe_case",
    "label_recovery_probe",
    "pure_prefix_trace_audit",
    "recovery_prefix_gate",
    "run_probe_and_label",
    "run_recovery_probe",
    "verify_probe_closure",
]
