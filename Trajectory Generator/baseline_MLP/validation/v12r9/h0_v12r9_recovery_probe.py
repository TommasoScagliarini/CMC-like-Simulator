"""V12R9 probes plus imported-prefix same-state H0 labels.

The plus labels are referenced directly from immutable V12R8 artifacts.  The
valid V12R8 minus prefix is labelled offline in the R9 namespace.  Only the
four cases never started by V12R8 execute a physical R6-behaviour probe.
"""

from __future__ import annotations

import copy
import hashlib
import math
import os
import stat
import sys
import time
from dataclasses import dataclass
from pathlib import Path
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
    LOCAL_VALIDATION_ROOT / "v12r7",
    REVISION_ROOT,
):
    if str(_root) not in sys.path:
        sys.path.insert(0, str(_root))

import freeze_h0_v12r6_functional_composite as r6_freezer  # noqa: E402
import h0_forensic_rollout as forensic  # noqa: E402
import h0_primary_split_v12r3_autonomy_recovery_contract as replay_contract  # noqa: E402
import h0_primary_split_v12r3_pure_probe_observer_labeler as observer  # noqa: E402
import h0_v12r6_physical_development as physical  # noqa: E402
import h0_v12r9_prefix_adjudicator as adjudicator  # noqa: E402
import h0_v12r9_recovery_contract as contract  # noqa: E402
import run_h0_primary_split_v9_causal_teacher as env_source  # noqa: E402
import run_h0_primary_split_v12r3_autonomy_recovery as runtime  # noqa: E402


PROBE_BEHAVIOR = "FROZEN_R6_PURE_POLICY_R9_OBSERVER_ONLY"
PROBE_COMPLETE_STATUS = "COMPLETE_H0_V12R9_R6_RECOVERY_PROBE"
LABEL_COMPLETE_STATUS = "COMPLETE_H0_V12R9_SAME_STATE_LABELS"
RECEIPT_STATUS = "PASS_H0_V12R9_RECOVERY_PROBE_CLOSURE"
ADJUDICATION_RECEIPT_STATUS = "PASS_H0_V12R9_R8_MINUS_ADJUDICATION_RECEIPT"
PROBE_RECEIPT_FIELDS = {
    "schema_version",
    "status",
    "passed",
    "integrity_passed",
    "recoverable_for_observer_label",
    "autonomy_passed",
    "protocol_id",
    "stage_id",
    "case_id",
    "probe_step_count",
    "candidate_id",
    "candidate_tree_sha256",
    "run_start",
    "trace",
    "partial_summary",
    "summary",
    "gate",
    "replay_payload",
    "v26_summary_normalization",
    "historical_plus_rerun",
    "teacher_query_count",
    "actor_updates",
    "critic_updates",
    "ppo_updates",
}
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


class V12R9RecoveryProbeError(RuntimeError):
    """Raised when an R9 probe or label stage cannot fail closed."""


@dataclass(frozen=True)
class RecoveryProbeConfig:
    """Frozen bindings for one of the four new R9 R6-behaviour probes."""

    case_id: str
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
    progress_label: str = "V12R9 R6 normalized recovery probe"


def _validate_config(config: RecoveryProbeConfig) -> None:
    if not isinstance(config, RecoveryProbeConfig):
        raise V12R9RecoveryProbeError("config must be RecoveryProbeConfig")
    if config.case_id not in contract.NEW_COLLECTION_CASE_IDS:
        raise V12R9RecoveryProbeError(
            "R9 probe accepts only the four unstarted cases; R8 rerun is forbidden"
        )
    if config.protocol_id != contract.PROTOCOL_ID:
        raise V12R9RecoveryProbeError("protocol identity drifted")
    if config.schema_version != contract.SCHEMA_VERSION:
        raise V12R9RecoveryProbeError("schema identity drifted")
    if type(config.progress_every) is not int or config.progress_every < 0:
        raise V12R9RecoveryProbeError("progress_every must be non-negative")
    if not isinstance(config.progress_label, str) or not config.progress_label:
        raise V12R9RecoveryProbeError("progress_label must be non-empty")
    if (
        config.expected_candidate_tree_sha256
        != contract.LOCKED_INPUTS["r6_candidate"]["tree_sha256"]
        or config.expected_candidate_id != EXPECTED_CANDIDATE_ID
    ):
        raise V12R9RecoveryProbeError("frozen R6 candidate identity drifted")
    if (
        config.event_contract_id != contract.EVENT_CONTRACT_ID
        or config.target_contract_id != contract.TARGET_CONTRACT_ID
    ):
        raise V12R9RecoveryProbeError("V26 detector contract drifted")
    root = Path(config.artifact_root).expanduser().resolve()
    if not root.is_dir() or root.is_symlink():
        raise V12R9RecoveryProbeError("artifact_root must be a safe directory")


def _inside(path: str | Path, root: Path, *, label: str) -> Path:
    target = Path(path).expanduser().resolve()
    try:
        target.relative_to(root)
    except ValueError as exc:
        raise V12R9RecoveryProbeError(f"{label} escaped artifact_root") from exc
    return target


def _artifact(path: Path, root: Path) -> dict[str, Any]:
    try:
        return forensic.artifact_record(path, artifact_root=root)
    except Exception as exc:
        raise V12R9RecoveryProbeError(f"invalid artifact: {path}") from exc


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _tree_artifact(path: Path, root: Path) -> dict[str, Any]:
    """Return a no-follow tree record using the same canonical digest as freeze."""

    target = _inside(path, root, label="teacher H0 tree")
    try:
        root_status = os.lstat(target)
    except OSError as exc:
        raise V12R9RecoveryProbeError("teacher H0 tree is missing") from exc
    if not stat.S_ISDIR(root_status.st_mode) or stat.S_ISLNK(root_status.st_mode):
        raise V12R9RecoveryProbeError("teacher H0 tree is unsafe")
    files: list[Path] = []
    for current_text, directory_names, file_names in os.walk(
        target, topdown=True, followlinks=False
    ):
        current = Path(current_text)
        directory_names.sort()
        file_names.sort()
        for name in directory_names:
            child = current / name
            status = os.lstat(child)
            if not stat.S_ISDIR(status.st_mode) or stat.S_ISLNK(status.st_mode):
                raise V12R9RecoveryProbeError(
                    "teacher H0 tree contains unsafe directory"
                )
        for name in file_names:
            child = current / name
            status = os.lstat(child)
            if not stat.S_ISREG(status.st_mode) or stat.S_ISLNK(status.st_mode):
                raise V12R9RecoveryProbeError("teacher H0 tree contains unsafe file")
            files.append(child)
    if not files:
        raise V12R9RecoveryProbeError("teacher H0 tree is empty")
    files.sort(key=lambda item: item.relative_to(target).as_posix())
    digest = hashlib.sha256()
    rows: list[dict[str, Any]] = []
    for item in files:
        relative = item.relative_to(target).as_posix()
        sha256 = _sha256_file(item)
        size_bytes = int(item.stat().st_size)
        rows.append({"path": relative, "sha256": sha256, "size_bytes": size_bytes})
        digest.update(relative.encode("utf-8"))
        digest.update(b"\0")
        digest.update(sha256.encode("ascii"))
        digest.update(b"\0")
        digest.update(str(size_bytes).encode("ascii"))
        digest.update(b"\n")
    return {
        "path": target.relative_to(root).as_posix(),
        "tree_sha256": digest.hexdigest(),
        "file_count": len(rows),
        "files": rows,
    }


def attest_offline_label_inputs(
    source_h0_path: str | Path,
    *,
    artifact_root: str | Path = REPO_ROOT,
) -> dict[str, Any]:
    """Hash-bind the two filesystem inputs consumed by offline labeling."""

    root = Path(artifact_root).expanduser().resolve()
    teacher = _tree_artifact(Path(source_h0_path).expanduser().resolve(), root)
    canonical_root = Path(REPO_ROOT).resolve()
    if root == canonical_root:
        expected_teacher_path = canonical_root.joinpath(
            *contract.SOURCE_H0_MODULE_PATH.parts
        )
        if (
            Path(source_h0_path).expanduser().resolve() != expected_teacher_path
            or teacher != contract.LOCKED_SOURCE_H0_TREE
        ):
            raise V12R9RecoveryProbeError("locked source H0 teacher tree drifted")
    coverage_path = canonical_root.joinpath(
        *contract.COVERAGE_REFERENCE_CORPUS_PATH.parts
    )
    coverage = _artifact(coverage_path, canonical_root)
    expected_coverage = {
        key: contract.LOCKED_COVERAGE_REFERENCE[key]
        for key in ("path", "sha256", "size_bytes")
    }
    if coverage != expected_coverage:
        raise V12R9RecoveryProbeError("locked coverage reference corpus drifted")
    try:
        with np.load(coverage_path, allow_pickle=False) as archive:
            coverage_rows = int(len(archive["observations"]))
            coverage_shape = tuple(archive["observations"].shape)
            training_indices = np.ascontiguousarray(archive["training_indices"])
    except Exception as exc:
        raise V12R9RecoveryProbeError(
            "locked coverage reference corpus is unreadable"
        ) from exc
    if (
        coverage_rows != contract.LOCKED_COVERAGE_REFERENCE["rows"]
        or coverage_shape
        != (
            contract.LOCKED_COVERAGE_REFERENCE["rows"],
            contract.EXPECTED_ACTOR_FEATURES,
        )
        or training_indices.dtype != np.dtype(np.int64)
        or not np.array_equal(training_indices, np.arange(coverage_rows))
    ):
        raise V12R9RecoveryProbeError("locked coverage reference rows drifted")
    return {
        "teacher_h0_id": contract.SOURCE_H0_ID,
        "teacher_observation_contract_id": contract.TEACHER_OBSERVATION_CONTRACT_ID,
        "teacher_h0": teacher,
        "coverage_reference_corpus": coverage,
        "coverage_reference_row_count": coverage_rows,
    }


def _assert_array_mapping_byte_exact(
    expected: Mapping[str, np.ndarray],
    observed: Mapping[str, np.ndarray],
    *,
    label: str,
) -> None:
    if set(expected) != set(observed):
        raise V12R9RecoveryProbeError(f"{label} array schema drifted")
    for name in expected:
        left = np.ascontiguousarray(np.asarray(expected[name]))
        right = np.ascontiguousarray(np.asarray(observed[name]))
        if (
            left.dtype != right.dtype
            or left.shape != right.shape
            or left.tobytes(order="C") != right.tobytes(order="C")
        ):
            raise V12R9RecoveryProbeError(f"{label} array bytes drifted: {name}")


def _strict_mapping(path: Path) -> dict[str, Any]:
    try:
        value = forensic.strict_json_load(path)
    except Exception as exc:
        raise V12R9RecoveryProbeError(f"invalid strict JSON: {path}") from exc
    if not isinstance(value, Mapping):
        raise V12R9RecoveryProbeError(f"expected JSON object: {path}")
    return dict(value)


def _strict_sequence(path: Path) -> list[Any]:
    try:
        value = forensic.strict_json_load(path)
    except Exception as exc:
        raise V12R9RecoveryProbeError(f"invalid strict JSON: {path}") from exc
    if not isinstance(value, list):
        raise V12R9RecoveryProbeError(f"expected JSON array: {path}")
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
        raise V12R9RecoveryProbeError("activity amount must be non-negative")
    if callback is not None:
        callback(name, amount)


def canonical_probe_case(config: RecoveryProbeConfig) -> dict[str, Any]:
    _validate_config(config)
    return contract.canonical_collection_case(config.case_id)


def pure_prefix_trace_audit(
    trace: Any, *, config: RecoveryProbeConfig
) -> dict[str, Any]:
    """Audit one closed R9 pure-policy prefix or complete episode."""

    _validate_config(config)
    rows = (
        list(trace)
        if isinstance(trace, Sequence) and not isinstance(trace, (str, bytes))
        else []
    )
    row_count_valid = 1 <= len(rows) <= contract.EXPECTED_STEPS
    identity_exact = row_count_valid
    action_path_exact = row_count_valid
    noise_exact = row_count_valid
    sensor_exact = row_count_valid
    teacher_free = row_count_valid
    terminal_shape_exact = row_count_valid
    counters = {name: 0 for name in ZERO_ONLINE_COUNTERS}
    expected_case = canonical_probe_case(config)
    try:
        innovations = np.ascontiguousarray(
            runtime._frozen_innovations(
                config.case_id,
                action_selection=str(expected_case["action_selection"]),
                np=np,
            ),
            dtype=np.float32,
        )
    except Exception:
        innovations = np.empty((0, contract.EXPECTED_ACTION_DIM))
        noise_exact = False
    for expected_step, row in enumerate(rows, start=1):
        if not isinstance(row, Mapping):
            identity_exact = action_path_exact = noise_exact = False
            sensor_exact = teacher_free = terminal_shape_exact = False
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
            innovations.shape == (contract.EXPECTED_STEPS, contract.EXPECTED_ACTION_DIM)
            and _finite_vector(standard_normal, length=contract.EXPECTED_ACTION_DIM)
            and np.asarray(standard_normal, dtype=np.float32).tobytes()
            == innovations[expected_step - 1].tobytes()
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
        noise_exact = noise_exact and noise_ok
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
            teacher_free = teacher_free and _zero_int(value)
            if type(value) is int:
                counters[name] += value
        is_last = expected_step == len(rows)
        if is_last:
            terminal_shape_exact = terminal_shape_exact and (
                (
                    row.get("terminated") is True
                    and row.get("truncated") is False
                    and row.get("end_reason") == "grf_penetration"
                )
                or (
                    len(rows) == contract.EXPECTED_STEPS
                    and row.get("terminated") is False
                    and row.get("truncated") is True
                    and row.get("end_reason") == "episode_time_limit"
                )
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
            noise_exact,
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
        "frozen_innovation_and_single_noise_exact": noise_exact,
        "raw_sensor_samples_exact": sensor_exact,
        "teacher_free": teacher_free,
        "terminal_shape_exact": terminal_shape_exact,
        "zero_online_counters": zero_counters,
        "counters": counters,
    }


def _zero_int(value: Any) -> bool:
    return type(value) is int and value == 0


def recovery_prefix_gate(
    summary: Mapping[str, Any],
    *,
    trace: Any,
    replay: observer.LoadedReplay,
    config: RecoveryProbeConfig,
) -> dict[str, Any]:
    """Accept a normalized R9 physical prefix for labels, never autonomy."""

    _validate_config(config)
    if not isinstance(summary, Mapping):
        raise V12R9RecoveryProbeError("summary must be a mapping")
    trace_audit = pure_prefix_trace_audit(trace, config=config)
    contract_gate = contract.collection_integrity_gate(summary)
    steps = summary.get("steps")
    binary_prefix = summary.get("binary_event_prefix_integrity")
    rows = list(trace) if isinstance(trace, list) else []
    replay_trace_exact = False
    penetration_exact = False
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
            pass
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
        "historical_plus_rerun_forbidden": config.case_id
        in contract.NEW_COLLECTION_CASE_IDS,
        "physical_prefix_or_complete_episode": physical_prefix or complete_episode,
        "candidate_exact": summary.get("candidate_tree_sha256")
        == config.expected_candidate_tree_sha256
        and summary.get("candidate_id") == config.expected_candidate_id,
        "detector_active": summary.get("binary_phase_fsm_mode") == "binary_active"
        and summary.get("event_contract_id") == config.event_contract_id
        and summary.get("target_contract_id") == config.target_contract_id,
        "summary_normalized_before_gate": isinstance(
            summary.get("v26_summary_normalization"), Mapping
        )
        and summary["v26_summary_normalization"].get("passed") is True,
        "detector_prefix_integrity": isinstance(binary_prefix, Mapping)
        and binary_prefix.get("passed") is True
        and binary_prefix.get("sample_count") == steps * contract.RAW_SAMPLES_PER_STEP,
        "zero_detector_anomalies": all(
            _zero_int(summary.get(name)) for name in adjudicator.DETECTOR_ANOMALY_FIELDS
        ),
        "zero_runtime_anomalies": all(
            _zero_int(summary.get(name)) for name in adjudicator.RUNTIME_ANOMALY_FIELDS
        ),
        "teacher_free_summary": summary.get("teacher_enabled") is False
        and summary.get("teacher_loaded_during_rollout") is False
        and summary.get("blending_enabled") is False
        and summary.get("safety_latch_enabled") is False
        and all(_zero_int(summary.get(name)) for name in ZERO_ONLINE_COUNTERS),
        "replay_shape": type(steps) is int
        and replay.n_steps == steps
        and replay.boundary_count == steps + 1,
        "replay_event_contract": replay.event_contract_id == config.event_contract_id,
        "replay_trace_actor_byte_exact": replay_trace_exact,
        "replay_trace_penetration_byte_exact": penetration_exact,
        "zero_updates": all(
            _zero_int(summary.get(name))
            for name in ("actor_updates", "critic_updates", "ppo_updates")
        ),
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
            "PASS_H0_V12R9_RECOVERABLE_R6_PREFIX"
            if passed
            else "FAIL_H0_V12R9_R6_PREFIX_INTEGRITY"
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
        "contract_gate": contract_gate,
        "next_stage": (
            "OFFLINE_COHERENT_H0_LABEL_REQUIRED" if passed else "STOP_INTEGRITY_FAILURE"
        ),
    }


def _default_tree_record(path: Path) -> Mapping[str, Any]:
    try:
        relative = path.resolve().relative_to(REPO_ROOT).as_posix()
    except ValueError as exc:
        raise V12R9RecoveryProbeError("candidate escaped repository root") from exc
    return r6_freezer.tree_record(relative)


def _normalization_receipt(value: Mapping[str, Any]) -> dict[str, Any]:
    return {key: copy.deepcopy(item) for key, item in value.items() if key != "summary"}


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
    """Run one of exactly four new probes and normalize V26 before gating."""

    _validate_config(config)
    artifact_root = Path(config.artifact_root).expanduser().resolve()
    root = _inside(destination, artifact_root, label="probe destination")
    if artifact_root == REPO_ROOT:
        expected_root = REPO_ROOT.joinpath(
            *contract.collection_case_root(config.case_id).parts
        )
        if root != expected_root:
            raise V12R9RecoveryProbeError(
                "canonical R9 probe destination drifted outside its case root"
            )
    if os.path.lexists(root):
        raise V12R9RecoveryProbeError(f"probe destination exists: {root}")
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
        raise V12R9RecoveryProbeError("candidate checkpoint is missing or unsafe")
    record_tree = tree_recorder or _default_tree_record
    try:
        candidate_tree = dict(record_tree(candidate_path))
    except Exception as exc:
        raise V12R9RecoveryProbeError("candidate tree attestation failed") from exc
    if candidate_tree != contract.LOCKED_INPUTS["r6_candidate"]:
        raise V12R9RecoveryProbeError("candidate is not the frozen V12R6 actor")

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
        raise V12R9RecoveryProbeError("production rollout stack is not ready") from exc
    candidate = RLModule.from_checkpoint(candidate_path)
    eval_mode = getattr(candidate, "eval", None)
    if not callable(eval_mode):
        raise V12R9RecoveryProbeError("candidate lacks eval()")
    eval_mode()
    innovations = runtime._frozen_innovations(
        config.case_id,
        action_selection=str(case["action_selection"]),
        np=np_runtime,
    )
    innovations = np_runtime.ascontiguousarray(innovations, dtype=np_runtime.float32)
    if innovations.shape != (contract.EXPECTED_STEPS, contract.EXPECTED_ACTION_DIM):
        raise V12R9RecoveryProbeError("frozen innovations shape drifted")
    if case["action_selection"] == "deterministic" and np_runtime.any(innovations != 0):
        raise V12R9RecoveryProbeError("deterministic innovations are not exact zero")
    built_config = builder(case)
    if not isinstance(built_config, Mapping):
        raise V12R9RecoveryProbeError("environment config builder was malformed")
    env = env_factory.make_cmc_env(dict(built_config))
    writer = forensic.ForensicRolloutWriter(root, artifact_root=artifact_root)
    stage_id = f"collect_label__{config.case_id}"
    writer.start(
        {
            "schema_version": config.schema_version,
            "status": "STARTED_H0_V12R9_R6_RECOVERY_PROBE",
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
            "historical_plus_rerun": False,
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
            raise V12R9RecoveryProbeError("reset info is malformed")
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
            reset_info=reset_info, legacy=legacy, np=np_runtime
        )
        body_weight_n = float(env.unwrapped._body_weight_n)
        if not math.isfinite(body_weight_n) or body_weight_n <= 0.0:
            raise V12R9RecoveryProbeError("body weight is malformed")
        replay_recorder = observer.PureProbeReplayRecorder.from_runtime(
            env.unwrapped._phase_fsm,
            body_weight_n,
            actor_names,
            event_contract_id=config.event_contract_id,
        )
        actor = np_runtime.ascontiguousarray(
            observation[: contract.EXPECTED_ACTOR_FEATURES], dtype=np_runtime.float32
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
                raise V12R9RecoveryProbeError("step info is malformed")
            raw_samples = info.get("binary_phase_sensor_samples")
            if (
                not isinstance(raw_samples, Sequence)
                or isinstance(raw_samples, (str, bytes))
                or len(raw_samples) != contract.RAW_SAMPLES_PER_STEP
            ):
                raise V12R9RecoveryProbeError(
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
                raise V12R9RecoveryProbeError("physical step audit is malformed")
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
                end_reason="v12r9_recovery_probe_failed",
                error=exc,
                status="FAIL_H0_V12R9_RECOVERY_PROBE_RUNTIME",
            )
        except Exception:
            pass
        raise
    finally:
        env.close()
    if audit is None or replay_recorder is None or not rows:
        raise V12R9RecoveryProbeError("probe audit/replay was not initialized")

    replay_path = root / "replay_boundaries.npz"
    replay_recorder.write_exclusive(replay_path)
    loaded_replay = observer.load_probe_replay_strict(
        replay_path, contract_module=replay_contract
    )
    raw_physical_summary = runtime._physical_summary(
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
    if not isinstance(raw_physical_summary, Mapping):
        raise V12R9RecoveryProbeError("physical summary is malformed")
    try:
        normalization = adjudicator.normalize_v26_prefix_summary(
            raw_physical_summary,
            expected_steps=len(rows),
        )
    except adjudicator.V12R9AdjudicationError as exc:
        raise V12R9RecoveryProbeError("V26 summary normalization failed") from exc
    summary = {
        **normalization["summary"],
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
        "v26_summary_normalization": _normalization_receipt(normalization),
        "historical_plus_rerun": False,
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
        "status": "PERSISTED_H0_V12R9_PROBE_BEFORE_GATE",
        "protocol_id": config.protocol_id,
        "stage_id": stage_id,
        "steps": len(rows),
        "summary_normalized": True,
        "gate_evaluated": False,
        "teacher_query_count": 0,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
    }
    artifacts = writer.finalize_before_gate(
        trace=rows, partial_summary=partial, summary=summary
    )
    gate = recovery_prefix_gate(
        summary, trace=rows, replay=loaded_replay, config=config
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
        "v26_summary_normalization": _normalization_receipt(normalization),
        "historical_plus_rerun": False,
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
    """Re-read, hash-bind, and recompute one normalized R9 probe."""

    _validate_config(config)
    artifact_root = Path(config.artifact_root).expanduser().resolve()
    root = _inside(probe_destination, artifact_root, label="probe destination")
    candidate_path = _inside(
        artifact_root.joinpath(*contract.R6_CANDIDATE_MODULE_PATH.parts),
        artifact_root,
        label="locked R6 candidate",
    )
    candidate_tree = _tree_artifact(candidate_path, artifact_root)
    if candidate_tree != contract.LOCKED_INPUTS["r6_candidate"]:
        raise V12R9RecoveryProbeError("full locked R6 candidate tree drifted")
    summary = _strict_mapping(root / "summary.json")
    trace = _strict_sequence(root / "trace.json")
    persisted_gate = _strict_mapping(root / "gate.json")
    receipt = _strict_mapping(root / "receipt.json")
    replay = observer.load_probe_replay_strict(
        root / "replay_boundaries.npz", contract_module=replay_contract
    )
    recomputed_gate = recovery_prefix_gate(
        summary, trace=trace, replay=replay, config=config
    )
    if persisted_gate != recomputed_gate or recomputed_gate.get("passed") is not True:
        raise V12R9RecoveryProbeError("recovery prefix gate is not closed PASS")
    expected_records = {
        "run_start": _artifact(root / "run_start.json", artifact_root),
        "trace": _artifact(root / "trace.json", artifact_root),
        "partial_summary": _artifact(root / "partial_summary.json", artifact_root),
        "summary": _artifact(root / "summary.json", artifact_root),
        "gate": _artifact(root / "gate.json", artifact_root),
        "replay_payload": _artifact(root / "replay_boundaries.npz", artifact_root),
    }
    if any(receipt.get(name) != record for name, record in expected_records.items()):
        raise V12R9RecoveryProbeError("probe receipt artifact binding drifted")
    stage_id = f"collect_label__{config.case_id}"
    candidate_tree = copy.deepcopy(candidate_tree)
    summary_checks = {
        "schema_version": config.schema_version,
        "status": PROBE_COMPLETE_STATUS,
        "protocol_id": config.protocol_id,
        "stage_id": stage_id,
        "case_id": config.case_id,
        "behavior": PROBE_BEHAVIOR,
        "candidate_id": config.expected_candidate_id,
        "candidate_module": candidate_tree,
        "candidate_tree_sha256": config.expected_candidate_tree_sha256,
        "trace_step_count": len(trace),
        "replay_step_count": replay.n_steps,
        "replay_boundary_count": replay.boundary_count,
        "replay_event_count": replay.event_count,
        "historical_plus_rerun": False,
        "teacher_enabled": False,
        "teacher_loaded_during_rollout": False,
        "blending_enabled": False,
        "safety_latch_enabled": False,
    }
    if type(summary.get("schema_version")) is not int or any(
        summary.get(name) != value for name, value in summary_checks.items()
    ):
        raise V12R9RecoveryProbeError("probe summary identity drifted")
    scalar_checks = {
        "schema_version": config.schema_version,
        "status": RECEIPT_STATUS,
        "passed": True,
        "integrity_passed": True,
        "recoverable_for_observer_label": True,
        "autonomy_passed": False,
        "protocol_id": config.protocol_id,
        "stage_id": stage_id,
        "case_id": config.case_id,
        "probe_step_count": replay.n_steps,
        "candidate_id": config.expected_candidate_id,
        "candidate_tree_sha256": config.expected_candidate_tree_sha256,
        "historical_plus_rerun": False,
    }
    if (
        set(receipt) != PROBE_RECEIPT_FIELDS
        or type(receipt.get("schema_version")) is not int
        or any(receipt.get(name) != value for name, value in scalar_checks.items())
    ):
        raise V12R9RecoveryProbeError("probe receipt semantic binding drifted")
    if receipt.get("v26_summary_normalization") != summary.get(
        "v26_summary_normalization"
    ):
        raise V12R9RecoveryProbeError("normalization receipt binding drifted")
    if any(
        not _zero_int(receipt.get(name))
        for name in (
            "teacher_query_count",
            "actor_updates",
            "critic_updates",
            "ppo_updates",
        )
    ):
        raise V12R9RecoveryProbeError("probe receipt zero counters drifted")
    if expected_replay is not None:
        if expected_replay.n_steps != replay.n_steps or any(
            expected_replay.arrays[name].tobytes(order="C")
            != replay.arrays[name].tobytes(order="C")
            for name in observer.REPLAY_ARRAY_NAMES
        ):
            raise V12R9RecoveryProbeError("probe replay changed across closure check")
    return {
        "passed": True,
        "summary": summary,
        "trace": trace,
        "gate": persisted_gate,
        "receipt": receipt,
        "replay": replay,
    }


def publish_r8_minus_adjudication(
    *, destination: str | Path, artifact_root: str | Path = REPO_ROOT
) -> dict[str, Any]:
    """Publish the R9 receipt for read-only R8 terminal/prefix adjudication."""

    root = Path(artifact_root).expanduser().resolve()
    target = _inside(destination, root, label="adjudication destination")
    expected = root.joinpath(*contract.R8_MINUS_ADJUDICATION_PATH.parts)
    if target != expected:
        raise V12R9RecoveryProbeError("adjudication destination is not canonical R9")
    result = adjudicator.load_and_adjudicate_r8_terminal(artifact_root=root)
    if result.get("passed") is not True:
        raise V12R9RecoveryProbeError("R8 minus prefix adjudication failed")
    payload = copy.deepcopy(result)
    payload["adjudication_status"] = payload["status"]
    payload["status"] = ADJUDICATION_RECEIPT_STATUS
    forensic.write_json_exclusive(target, payload)
    return verify_r8_minus_adjudication(destination=target, artifact_root=root)


def verify_r8_minus_adjudication(
    *, destination: str | Path, artifact_root: str | Path = REPO_ROOT
) -> dict[str, Any]:
    """Recompute and strictly compare the R9 R8-minus receipt."""

    root = Path(artifact_root).expanduser().resolve()
    target = _inside(destination, root, label="adjudication destination")
    observed = _strict_mapping(target)
    result = adjudicator.load_and_adjudicate_r8_terminal(artifact_root=root)
    expected = copy.deepcopy(result)
    expected["adjudication_status"] = expected["status"]
    expected["status"] = ADJUDICATION_RECEIPT_STATUS
    if observed != expected or observed.get("passed") is not True:
        raise V12R9RecoveryProbeError("R8 minus adjudication receipt drifted")
    return observed


def _label_summary_payload(
    *,
    config_case_id: str,
    artifact_root: Path,
    labels_path: Path,
    result: observer.ObserverLabelResult,
    collection_integrity_passed: bool,
    source_receipt: Mapping[str, Any],
    source_replay: Mapping[str, Any],
    offline_inputs: Mapping[str, Any],
    stage_id: str,
) -> dict[str, Any]:
    rows = result.replay.n_steps
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": LABEL_COMPLETE_STATUS,
        "protocol_id": contract.PROTOCOL_ID,
        "stage_id": stage_id,
        "case_id": config_case_id,
        "collection_integrity_passed": collection_integrity_passed,
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
        "teacher_h0_id": offline_inputs["teacher_h0_id"],
        "teacher_observation_contract_id": offline_inputs[
            "teacher_observation_contract_id"
        ],
        "teacher_h0": copy.deepcopy(offline_inputs["teacher_h0"]),
        "coverage_reference_corpus": copy.deepcopy(
            offline_inputs["coverage_reference_corpus"]
        ),
        "coverage_reference_row_count": offline_inputs["coverage_reference_row_count"],
        "source_receipt": copy.deepcopy(dict(source_receipt)),
        "source_replay": copy.deepcopy(dict(source_replay)),
        "labels": _artifact(labels_path, artifact_root),
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
    }


def _label_receipt_payload(
    *,
    config_case_id: str,
    artifact_root: Path,
    label_root: Path,
    result: observer.ObserverLabelResult,
    source_receipt: Mapping[str, Any],
    source_replay: Mapping[str, Any],
    offline_inputs: Mapping[str, Any],
    stage_id: str,
    gate: Mapping[str, Any],
) -> dict[str, Any]:
    rows = result.replay.n_steps
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": gate["status"],
        "passed": gate["passed"],
        "protocol_id": contract.PROTOCOL_ID,
        "stage_id": stage_id,
        "case_id": config_case_id,
        "labelled_row_count": rows,
        "same_state_teacher_label_count": rows,
        "teacher_query_count": result.teacher_query_count,
        "environment_reset_calls": 0,
        "environment_step_calls": 0,
        "action_served_count": 0,
        "teacher_h0_id": offline_inputs["teacher_h0_id"],
        "teacher_observation_contract_id": offline_inputs[
            "teacher_observation_contract_id"
        ],
        "teacher_h0": copy.deepcopy(offline_inputs["teacher_h0"]),
        "coverage_reference_corpus": copy.deepcopy(
            offline_inputs["coverage_reference_corpus"]
        ),
        "coverage_reference_row_count": offline_inputs["coverage_reference_row_count"],
        "source_receipt": copy.deepcopy(dict(source_receipt)),
        "source_replay": copy.deepcopy(dict(source_replay)),
        "labels": _artifact(label_root / "labels.npz", artifact_root),
        "summary": _artifact(label_root / "summary.json", artifact_root),
        "gate": _artifact(label_root / "gate.json", artifact_root),
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
    }


def _persist_labels(
    *,
    config_case_id: str,
    artifact_root: Path,
    label_root: Path,
    result: observer.ObserverLabelResult,
    collection_integrity_passed: bool,
    source_receipt: Mapping[str, Any],
    source_replay: Mapping[str, Any],
    offline_inputs: Mapping[str, Any],
    stage_id: str,
) -> dict[str, Any]:
    if label_root.exists():
        raise V12R9RecoveryProbeError(f"label destination exists: {label_root}")
    label_root.mkdir(parents=True, exist_ok=False)
    labels_path = label_root / "labels.npz"
    observer.write_npz_exclusive(labels_path, result.arrays)
    summary = _label_summary_payload(
        config_case_id=config_case_id,
        artifact_root=artifact_root,
        labels_path=labels_path,
        result=result,
        collection_integrity_passed=collection_integrity_passed,
        source_receipt=source_receipt,
        source_replay=source_replay,
        offline_inputs=offline_inputs,
        stage_id=stage_id,
    )
    gate = contract.label_gate(summary)
    summary_path = label_root / "summary.json"
    gate_path = label_root / "gate.json"
    receipt_path = label_root / "receipt.json"
    forensic.write_json_exclusive(summary_path, summary)
    forensic.write_json_exclusive(gate_path, gate)
    receipt = _label_receipt_payload(
        config_case_id=config_case_id,
        artifact_root=artifact_root,
        label_root=label_root,
        result=result,
        source_receipt=source_receipt,
        source_replay=source_replay,
        offline_inputs=offline_inputs,
        stage_id=stage_id,
        gate=gate,
    )
    forensic.write_json_exclusive(receipt_path, receipt)
    if gate.get("passed") is not True:
        raise V12R9RecoveryProbeError("offline observer label gate failed")
    return {
        "passed": True,
        "destination": label_root,
        "summary": summary,
        "gate": gate,
        "receipt": receipt,
        "result": result,
    }


def _check_offline_result(result: observer.ObserverLabelResult) -> None:
    if (
        result.environment_reset_calls != 0
        or result.environment_step_calls != 0
        or result.action_served_count != 0
        or result.teacher_query_count != result.replay.n_steps
        or result.arrays["observations"].tobytes(order="C")
        != result.replay.arrays["actor_observations"].tobytes(order="C")
    ):
        raise V12R9RecoveryProbeError("offline same-state label invariant failed")


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
    """Offline-label one newly collected, normalized R9 probe."""

    _validate_config(config)
    artifact_root = Path(config.artifact_root).expanduser().resolve()
    probe_root = _inside(probe_destination, artifact_root, label="probe destination")
    label_root = _inside(label_destination, artifact_root, label="label destination")
    if label_root != probe_root / "observer_labels":
        raise V12R9RecoveryProbeError(
            "label destination must be <probe_destination>/observer_labels"
        )
    closure = verify_probe_closure(probe_root, config=config)
    offline_inputs_before = attest_offline_label_inputs(
        source_h0_path, artifact_root=artifact_root
    )

    def closure_validator(replay: observer.LoadedReplay) -> None:
        verify_probe_closure(probe_root, config=config, expected_replay=replay)

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
    _check_offline_result(result)
    offline_inputs_after = attest_offline_label_inputs(
        source_h0_path, artifact_root=artifact_root
    )
    if offline_inputs_before != offline_inputs_after:
        raise V12R9RecoveryProbeError("offline label inputs changed during labeling")
    return _persist_labels(
        config_case_id=config.case_id,
        artifact_root=artifact_root,
        label_root=label_root,
        result=result,
        collection_integrity_passed=closure["gate"]["passed"],
        source_receipt=_artifact(probe_root / "receipt.json", artifact_root),
        source_replay=_artifact(probe_root / "replay_boundaries.npz", artifact_root),
        offline_inputs=offline_inputs_before,
        stage_id=f"label__{config.case_id}",
    )


def label_adjudicated_r8_minus(
    *,
    adjudication_destination: str | Path,
    label_destination: str | Path,
    source_h0_path: str | Path,
    artifact_root: str | Path = REPO_ROOT,
    module_loader: Callable[[Path], Any] | None = None,
    mean_query: Callable[[Any, np.ndarray], Any] | None = None,
    coverage_evaluator: Callable[[np.ndarray], Any] | None = None,
    phase_fsm_factory: Callable[[Mapping[str, Any], str, str], Any] | None = None,
) -> dict[str, Any]:
    """Offline-label the immutable R8 minus replay after R9 adjudication."""

    root = Path(artifact_root).expanduser().resolve()
    adjudication_path = _inside(
        adjudication_destination, root, label="adjudication destination"
    )
    label_root = _inside(label_destination, root, label="label destination")
    expected_label_root = root.joinpath(
        *contract.collection_case_root(contract.IMPORTED_MINUS_CASE_ID).parts,
        "observer_labels",
    )
    if label_root != expected_label_root:
        raise V12R9RecoveryProbeError(
            "R8 minus labels must use the canonical R9 namespace"
        )
    adjudication_receipt = verify_r8_minus_adjudication(
        destination=adjudication_path, artifact_root=root
    )
    if adjudication_receipt.get("offline_label_authorized") is not True:
        raise V12R9RecoveryProbeError("R8 minus offline label is not authorized")
    replay_path = root.joinpath(*contract.R8_MINUS_REPLAY_PATH.parts)
    offline_inputs_before = attest_offline_label_inputs(
        source_h0_path, artifact_root=root
    )

    def closure_validator(replay: observer.LoadedReplay) -> None:
        observed = verify_r8_minus_adjudication(
            destination=adjudication_path, artifact_root=root
        )
        expected = observer.load_probe_replay_strict(
            replay_path, contract_module=replay_contract
        )
        if (
            observed.get("passed") is not True
            or expected.n_steps != replay.n_steps
            or any(
                expected.arrays[name].tobytes(order="C")
                != replay.arrays[name].tobytes(order="C")
                for name in observer.REPLAY_ARRAY_NAMES
            )
        ):
            raise V12R9RecoveryProbeError("R8 minus replay closure drifted")

    result = observer.label_closed_probe_in_memory(
        replay_path,
        probe_closure_validator=closure_validator,
        source_h0_path=source_h0_path,
        case_id=contract.IMPORTED_MINUS_CASE_ID,
        probe_stage="p0",
        module_loader=module_loader,
        mean_query=mean_query,
        coverage_evaluator=coverage_evaluator,
        phase_fsm_factory=phase_fsm_factory,
        contract_module=replay_contract,
    )
    _check_offline_result(result)
    offline_inputs_after = attest_offline_label_inputs(
        source_h0_path, artifact_root=root
    )
    if offline_inputs_before != offline_inputs_after:
        raise V12R9RecoveryProbeError("offline label inputs changed during labeling")
    return _persist_labels(
        config_case_id=contract.IMPORTED_MINUS_CASE_ID,
        artifact_root=root,
        label_root=label_root,
        result=result,
        collection_integrity_passed=True,
        source_receipt=_artifact(adjudication_path, root),
        source_replay=_artifact(replay_path, root),
        offline_inputs=offline_inputs_before,
        stage_id="label_r8_minus_prefix",
    )


def _read_label_arrays(path: Path) -> dict[str, np.ndarray]:
    try:
        with np.load(path, allow_pickle=False) as archive:
            if set(archive.files) != set(observer.LABEL_ARRAY_DTYPES):
                raise V12R9RecoveryProbeError("observer label NPZ schema drifted")
            arrays = {
                name: np.ascontiguousarray(archive[name].copy())
                for name in archive.files
            }
    except V12R9RecoveryProbeError:
        raise
    except Exception as exc:
        raise V12R9RecoveryProbeError("observer label NPZ is unreadable") from exc
    rows = len(arrays["observations"])
    try:
        observer.validate_observer_label_arrays(arrays, rows=rows)
    except Exception as exc:
        raise V12R9RecoveryProbeError("observer label NPZ validation failed") from exc
    return arrays


def _verify_label_stage_binding(
    *,
    case_id: str,
    artifact_root: Path,
    label_root: Path,
    offline_inputs: Mapping[str, Any],
) -> dict[str, Any]:
    if case_id == contract.HISTORICAL_CASE_ID:
        raise V12R9RecoveryProbeError(
            "imported plus labels use the dedicated import-stage verifier"
        )
    imported_minus = case_id == contract.IMPORTED_MINUS_CASE_ID
    stage_path = artifact_root.joinpath(
        *(
            contract.R8_MINUS_LABEL_STAGE_RECEIPT_PATH.parts
            if imported_minus
            else (contract.collection_case_root(case_id) / "stage_receipt.json").parts
        )
    )
    stage = _strict_mapping(stage_path)
    expected_stage_id = (
        "label_r8_minus_prefix" if imported_minus else f"collect_label__{case_id}"
    )
    stage_index = tuple(contract.STAGE_IDS).index(expected_stage_id) + 1
    worker_claim_path = artifact_root.joinpath(
        *contract.RUN_ROOT.parts,
        "claims",
        f"{stage_index:02d}_{expected_stage_id}.json",
    )
    expected_records = {
        "labels": label_root / "labels.npz",
        "label_summary": label_root / "summary.json",
        "label_gate": label_root / "gate.json",
        "label_receipt": label_root / "receipt.json",
        "pipeline_claim": artifact_root.joinpath(*contract.CLAIM_PATH.parts),
        "worker_claim": worker_claim_path,
    }
    if imported_minus:
        expected_records["adjudication_stage_receipt"] = artifact_root.joinpath(
            *contract.R8_MINUS_ADJUDICATION_STAGE_RECEIPT_PATH.parts
        )
        expected_records["source_replay"] = artifact_root.joinpath(
            *contract.R8_MINUS_REPLAY_PATH.parts
        )
    else:
        probe_root = artifact_root.joinpath(
            *contract.collection_case_root(case_id).parts
        )
        expected_records.update(
            {
                "probe_receipt": probe_root / "receipt.json",
                "probe_gate": probe_root / "gate.json",
                "probe_replay": probe_root / "replay_boundaries.npz",
            }
        )
    common = (
        stage.get("passed") is True
        and stage.get("protocol_id") == contract.PROTOCOL_ID
        and stage.get("pipeline_id") == contract.PIPELINE_ID
        and stage.get("stage_id") == expected_stage_id
        and stage.get("case_id") == case_id
        and stage.get("teacher_h0_id") == offline_inputs["teacher_h0_id"]
        and stage.get("teacher_h0") == offline_inputs["teacher_h0"]
        and stage.get("coverage_reference_corpus")
        == offline_inputs["coverage_reference_corpus"]
        and all(
            stage.get(name) == _artifact(path, artifact_root)
            for name, path in expected_records.items()
        )
        and type(stage.get("labelled_row_count")) is int
        and stage.get("labelled_row_count")
        == stage.get("same_state_teacher_label_count")
        == stage.get("teacher_query_count")
        and all(
            type(stage.get(name)) is int and stage[name] == 0
            for name in ("actor_updates", "critic_updates", "ppo_updates")
        )
        and (
            (
                imported_minus
                and stage.get("imported_r8_prefix") is True
                and "single_collection_round" not in stage
            )
            or (
                not imported_minus
                and "imported_r8_prefix" not in stage
                and stage.get("single_collection_round") is True
            )
        )
    )
    if not common:
        raise V12R9RecoveryProbeError("observer label stage binding drifted")
    return {"stage_receipt": _artifact(stage_path, artifact_root), "stage": stage}


def _verify_plus_import_stage_binding(*, artifact_root: Path) -> dict[str, Any]:
    """Bind the direct, no-copy R8 plus reference to the R9 stage claim."""

    stage_path = artifact_root.joinpath(
        *contract.R8_PLUS_IMPORT_STAGE_RECEIPT_PATH.parts
    )
    stage = _strict_mapping(stage_path)
    stage_id = "import_r8_plus_labels"
    stage_index = tuple(contract.STAGE_IDS).index(stage_id) + 1
    expected_records = {
        "labels": artifact_root.joinpath(*contract.R8_PLUS_LABELS_PATH.parts),
        "label_summary": artifact_root.joinpath(
            *contract.R8_PLUS_LABEL_SUMMARY_PATH.parts
        ),
        "label_gate": artifact_root.joinpath(*contract.R8_PLUS_LABEL_GATE_PATH.parts),
        "label_receipt": artifact_root.joinpath(
            *contract.R8_PLUS_LABEL_RECEIPT_PATH.parts
        ),
        "source_stage_receipt": artifact_root.joinpath(
            *contract.R8_PLUS_LABEL_STAGE_RECEIPT_PATH.parts
        ),
        "pipeline_claim": artifact_root.joinpath(*contract.CLAIM_PATH.parts),
        "worker_claim": artifact_root.joinpath(
            *contract.RUN_ROOT.parts,
            "claims",
            f"{stage_index:02d}_{stage_id}.json",
        ),
    }
    checks = {
        "identity": stage.get("passed") is True
        and stage.get("protocol_id") == contract.PROTOCOL_ID
        and stage.get("pipeline_id") == contract.PIPELINE_ID
        and stage.get("stage_id") == stage_id
        and stage.get("case_id") == contract.HISTORICAL_CASE_ID,
        "artifact_bindings": all(
            stage.get(name) == _artifact(path, artifact_root)
            for name, path in expected_records.items()
        ),
        "direct_reference": stage.get("direct_immutable_reference") is True
        and stage.get("labels_copied") is False
        and stage.get("r8_artifacts_modified") is False
        and stage.get("semantic_and_byte_exact_closed") is True,
        "activity": stage.get("labelled_row_count") == 179
        and stage.get("same_state_teacher_label_count") == 179
        and all(
            _zero_int(stage.get(name))
            for name in (
                "environment_reset_calls",
                "environment_step_calls",
                "teacher_query_count",
                "actor_updates",
                "critic_updates",
                "ppo_updates",
            )
        ),
    }
    if not all(checks.values()):
        failed = sorted(name for name, value in checks.items() if value is not True)
        raise V12R9RecoveryProbeError(f"plus import stage binding drifted: {failed}")
    return {"stage_receipt": _artifact(stage_path, artifact_root), "stage": stage}


def verify_observer_label_closure(
    case_id: str,
    *,
    artifact_root: str | Path = REPO_ROOT,
    source_h0_path: str | Path | None = None,
    module_loader: Callable[[Path], Any] | None = None,
    mean_query: Callable[[Any, np.ndarray], Any] | None = None,
    coverage_evaluator: Callable[[np.ndarray], Any] | None = None,
    phase_fsm_factory: Callable[[Mapping[str, Any], str, str], Any] | None = None,
    require_stage_receipt: bool = True,
) -> dict[str, Any]:
    """Recompute one same-state H0 label set and compare every NPZ byte.

    This verifier never constructs or steps an environment.  It first closes
    the source replay, re-attests the H0/coverage inputs, queries the locked H0
    once per replay row, and reconstructs summary, gate, receipt and stage
    bindings from the recomputed arrays.
    """

    if case_id not in contract.COLLECTION_CASE_IDS:
        raise V12R9RecoveryProbeError(f"unknown observer case: {case_id!r}")
    if type(require_stage_receipt) is not bool:
        raise V12R9RecoveryProbeError("require_stage_receipt must be strict bool")
    root = Path(artifact_root).expanduser().resolve()
    teacher_path = (
        root.joinpath(*contract.SOURCE_H0_MODULE_PATH.parts)
        if source_h0_path is None
        else Path(source_h0_path).expanduser().resolve()
    )
    if case_id == contract.HISTORICAL_CASE_ID:
        imported = adjudicator.verify_r8_plus_label_import(
            artifact_root=root, semantic_verify=True
        )
        stage_binding: dict[str, Any] = {}
        if require_stage_receipt:
            stage_binding = _verify_plus_import_stage_binding(artifact_root=root)
        return {
            "passed": True,
            "protocol_id": contract.PROTOCOL_ID,
            "case_id": case_id,
            "labelled_row_count": imported["labelled_row_count"],
            "same_state_teacher_label_count": imported[
                "same_state_teacher_label_count"
            ],
            "teacher_query_count": 0,
            "imported_source_teacher_query_count": imported[
                "same_state_teacher_label_count"
            ],
            "direct_immutable_reference": True,
            "labels_copied": False,
            "semantic_and_byte_exact_closed": True,
            "labels": copy.deepcopy(contract.LOCKED_R8_EVIDENCE["plus_labels"]),
            "label_summary": copy.deepcopy(
                contract.LOCKED_R8_EVIDENCE["plus_label_summary"]
            ),
            "label_gate": copy.deepcopy(contract.LOCKED_R8_EVIDENCE["plus_label_gate"]),
            "label_receipt": copy.deepcopy(
                contract.LOCKED_R8_EVIDENCE["plus_label_receipt"]
            ),
            **stage_binding,
            "environment_reset_calls": 0,
            "environment_step_calls": 0,
            "action_served_count": 0,
        }
    if case_id == contract.IMPORTED_MINUS_CASE_ID:
        source_receipt_path = root.joinpath(*contract.R8_MINUS_ADJUDICATION_PATH.parts)
        replay_path = root.joinpath(*contract.R8_MINUS_REPLAY_PATH.parts)
        label_root = root.joinpath(
            *contract.collection_case_root(case_id).parts, "observer_labels"
        )
        adjudication_receipt = verify_r8_minus_adjudication(
            destination=source_receipt_path, artifact_root=root
        )
        if adjudication_receipt.get("offline_label_authorized") is not True:
            raise V12R9RecoveryProbeError("R8 minus offline label is not authorized")

        def closure_validator(replay: observer.LoadedReplay) -> None:
            current = verify_r8_minus_adjudication(
                destination=source_receipt_path, artifact_root=root
            )
            expected = observer.load_probe_replay_strict(
                replay_path, contract_module=replay_contract
            )
            if current.get("passed") is not True or expected.n_steps != replay.n_steps:
                raise V12R9RecoveryProbeError("R8 minus replay closure drifted")
            _assert_array_mapping_byte_exact(
                {name: expected.arrays[name] for name in observer.REPLAY_ARRAY_NAMES},
                {name: replay.arrays[name] for name in observer.REPLAY_ARRAY_NAMES},
                label="R8 minus replay",
            )

        collection_integrity_passed = True
        stage_id = "label_r8_minus_prefix"
    else:
        probe_root = root.joinpath(*contract.collection_case_root(case_id).parts)
        config = RecoveryProbeConfig(case_id=case_id, artifact_root=root)
        closed = verify_probe_closure(probe_root, config=config)
        source_receipt_path = probe_root / "receipt.json"
        replay_path = probe_root / "replay_boundaries.npz"
        label_root = probe_root / "observer_labels"

        def closure_validator(replay: observer.LoadedReplay) -> None:
            verify_probe_closure(probe_root, config=config, expected_replay=replay)

        collection_integrity_passed = closed["gate"].get("passed") is True
        stage_id = f"label__{case_id}"

    offline_inputs_before = attest_offline_label_inputs(
        teacher_path, artifact_root=root
    )
    result = observer.label_closed_probe_in_memory(
        replay_path,
        probe_closure_validator=closure_validator,
        source_h0_path=teacher_path,
        case_id=case_id,
        probe_stage="p0",
        module_loader=module_loader,
        mean_query=mean_query,
        coverage_evaluator=coverage_evaluator,
        phase_fsm_factory=phase_fsm_factory,
        contract_module=replay_contract,
    )
    _check_offline_result(result)
    offline_inputs_after = attest_offline_label_inputs(teacher_path, artifact_root=root)
    if offline_inputs_before != offline_inputs_after:
        raise V12R9RecoveryProbeError(
            "offline label inputs changed during verification"
        )
    labels_path = label_root / "labels.npz"
    persisted_arrays = _read_label_arrays(labels_path)
    _assert_array_mapping_byte_exact(
        result.arrays, persisted_arrays, label=f"observer label {case_id}"
    )
    source_receipt = _artifact(source_receipt_path, root)
    source_replay = _artifact(replay_path, root)
    expected_summary = _label_summary_payload(
        config_case_id=case_id,
        artifact_root=root,
        labels_path=labels_path,
        result=result,
        collection_integrity_passed=collection_integrity_passed,
        source_receipt=source_receipt,
        source_replay=source_replay,
        offline_inputs=offline_inputs_before,
        stage_id=stage_id,
    )
    observed_summary = _strict_mapping(label_root / "summary.json")
    if observed_summary != expected_summary:
        raise V12R9RecoveryProbeError("observer label summary semantic drifted")
    expected_gate = contract.label_gate(expected_summary)
    observed_gate = _strict_mapping(label_root / "gate.json")
    if observed_gate != expected_gate or expected_gate.get("passed") is not True:
        raise V12R9RecoveryProbeError("observer label gate semantic drifted")
    expected_receipt = _label_receipt_payload(
        config_case_id=case_id,
        artifact_root=root,
        label_root=label_root,
        result=result,
        source_receipt=source_receipt,
        source_replay=source_replay,
        offline_inputs=offline_inputs_before,
        stage_id=stage_id,
        gate=expected_gate,
    )
    observed_receipt = _strict_mapping(label_root / "receipt.json")
    if observed_receipt != expected_receipt:
        raise V12R9RecoveryProbeError("observer label receipt semantic drifted")
    stage_binding: dict[str, Any] = {}
    if require_stage_receipt:
        stage_binding = _verify_label_stage_binding(
            case_id=case_id,
            artifact_root=root,
            label_root=label_root,
            offline_inputs=offline_inputs_before,
        )
    return {
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "case_id": case_id,
        "labelled_row_count": result.replay.n_steps,
        "same_state_teacher_label_count": result.teacher_query_count,
        "teacher_query_count": result.teacher_query_count,
        **copy.deepcopy(offline_inputs_before),
        "source_receipt": source_receipt,
        "source_replay": source_replay,
        "labels": _artifact(labels_path, root),
        "label_summary": _artifact(label_root / "summary.json", root),
        "label_gate": _artifact(label_root / "gate.json", root),
        "label_receipt": _artifact(label_root / "receipt.json", root),
        **stage_binding,
        "environment_reset_calls": 0,
        "environment_step_calls": 0,
        "action_served_count": 0,
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
    probe = run_recovery_probe(
        config=config,
        destination=probe_destination,
        module_path=module_path,
        activity_callback=activity_callback,
    )
    if probe.get("passed") is not True:
        raise V12R9RecoveryProbeError("probe prefix is not labelable")
    labels = label_recovery_probe(
        config=config,
        probe_destination=probe_destination,
        label_destination=label_destination,
        source_h0_path=source_h0_path,
    )
    return {"passed": True, "probe": probe, "labels": labels}


__all__ = [
    "ADJUDICATION_RECEIPT_STATUS",
    "EXPECTED_CANDIDATE_ID",
    "FORBIDDEN_ONLINE_FIELDS",
    "LABEL_COMPLETE_STATUS",
    "PROBE_BEHAVIOR",
    "PROBE_COMPLETE_STATUS",
    "RECEIPT_STATUS",
    "RecoveryProbeConfig",
    "V12R9RecoveryProbeError",
    "attest_offline_label_inputs",
    "canonical_probe_case",
    "label_adjudicated_r8_minus",
    "label_recovery_probe",
    "publish_r8_minus_adjudication",
    "pure_prefix_trace_audit",
    "recovery_prefix_gate",
    "run_probe_and_label",
    "run_recovery_probe",
    "verify_probe_closure",
    "verify_observer_label_closure",
    "verify_r8_minus_adjudication",
]
