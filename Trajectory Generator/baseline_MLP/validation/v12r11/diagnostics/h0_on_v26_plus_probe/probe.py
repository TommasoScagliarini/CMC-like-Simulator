"""One-shot source-H0-on-V26 physical diagnostic for the hardest +0.20 case.

The source H0 checkpoint is the action policy itself.  It receives the first
35 float32 features of the live V26 observation through the frozen production
physical runtime.  There is no teacher role, fit, blend, latch, or update path.

The production runtime intentionally raises when an episode ends before its
500-row pure-policy trace audit.  This wrapper preserves that behavior as a
failed gate while completing ``trace.json``, ``partial_summary.json``,
``summary.json``, and ``gate.json`` from the already-durable step journal.
"""

from __future__ import annotations

import ast
import copy
import hashlib
import importlib
import math
import os
import sys
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path, PurePosixPath
from typing import Any


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
PACKAGE_ROOT = Path(__file__).resolve().parent
LOCAL_VALIDATION = REPO_ROOT / "Trajectory Generator/baseline_MLP/validation"
for _root in (
    REPO_ROOT / "validation",
    LOCAL_VALIDATION,
    LOCAL_VALIDATION / "v12r6",
):
    if str(_root) not in sys.path:
        sys.path.insert(0, str(_root))

import h0_forensic_rollout as forensic  # noqa: E402


SCHEMA_VERSION = 1311
PROTOCOL_ID = "AB06_H0_V12R11_SOURCE_H0_DIRECT_V26_35_PLUS_PROBE"
DIAGNOSTIC_ID = "H0_V12R11_SOURCE_H0_ON_V26_35_PLUS_ZERO_FIT"
CASE_ID = "deterministic_offset_plus_0p20"
STAGE_ID = f"development__{CASE_ID}"
START_STATUS = "STARTED_H0_V12R11_SOURCE_H0_ON_V26_PLUS_PROBE"
PARTIAL_STATUS = "PERSISTED_H0_V12R11_SOURCE_H0_ON_V26_PLUS_BEFORE_GATE"
COMPLETE_STATUS = "COMPLETE_H0_V12R11_SOURCE_H0_ON_V26_PLUS_PROBE"
EARLY_COMPLETE_STATUS = "COMPLETE_H0_V12R11_SOURCE_H0_ON_V26_PLUS_EARLY_TERMINATION"
GATE_PASS_STATUS = "PASS_H0_V12R11_SOURCE_H0_ON_V26_PLUS_PROBE"
GATE_FAIL_STATUS = "FAIL_H0_V12R11_SOURCE_H0_ON_V26_PLUS_PROBE"
CLOSURE_FAIL_STATUS = "FAIL_H0_V12R11_SOURCE_H0_ON_V26_PLUS_CLOSURE"
CLOSURE_PASS_STATUS = "PASS_H0_V12R11_SOURCE_H0_ON_V26_PLUS_CLOSURE"
EXPECTED_STEPS = 500
EXPECTED_ACTOR_FEATURES = 35
EXPECTED_ACTION_DIM = 2
EXPECTED_RAW_SENSOR_SAMPLES_PER_STEP = 10
EXPECTED_H0_STD = 0.005
PENETRATION_LIMIT_M = 0.025
MINIMUM_VALID_CYCLES = 2
EVENT_CONTRACT_ID = "binary_point_v25+heel_qualified_fsm_v2"
TARGET_CONTRACT_ID = "primary_grf_split_v1+binary_point_v25+heel_qualified_fsm_v2"
EVENT_SOURCE = "binary_active_v26"
SOURCE_H0_ID = "H0_MARKOV35_PHASE_ALIGNED_SIGMA0005_ITER1_RETRY"
SOURCE_H0_RELATIVE_PATH = PurePosixPath(
    "validation/critic_warmup/"
    "2026-07-13_markov35_phase_aligned_sigma0005_iter1_retry/rl_module_last"
)
PHYSICAL_RUNTIME_RELATIVE_PATH = PurePosixPath(
    "Trajectory Generator/baseline_MLP/validation/v12r6/"
    "h0_v12r6_physical_development.py"
)
R10_EXECUTION_LOCK_RELATIVE_PATH = PurePosixPath(
    "Trajectory Generator/baseline_MLP/validation/v12r10/"
    "h0_v12r10_recovery_execution_lock.json"
)
DEFAULT_DESTINATION = (
    PACKAGE_ROOT / "artifacts" / ("20260815_deterministic_offset_plus_0p20")
)
PURE_COUNTER_FIELDS = (
    "teacher_query_count",
    "served_action_teacher_dependency_count",
    "mean_blend_count",
    "safety_intervention_count",
    "safety_latch_activation_count",
    "safety_latch_release_count",
)
REQUIRED_STEP_CHECKS = (
    "finite",
    "binary_active_routing_exact",
    "ten_control_windows",
    "ten_sensor_samples",
    "no_timeout",
    "no_unaccepted_so",
    "no_sea_fallback",
    "no_hard_invalid",
)
ALLOWED_ACTIVITY_FIELDS = (
    "environment_reset_calls",
    "environment_step_calls",
    "raw_sensor_sample_count",
)
FORBIDDEN_TEACHER_FIELDS = {
    "teacher_mean",
    "teacher_action",
    "blended_mean",
    "requested_alpha",
    "effective_alpha",
    "safety_latch_active",
}

LOCKED_SOURCE_H0_TREE = {
    "path": SOURCE_H0_RELATIVE_PATH.as_posix(),
    "tree_sha256": "f7f6c898975af109412af8c3f1a338b5076f9fefcec1e2723673fd821f1f13ee",
    "file_count": 3,
    "files": [
        {
            "path": "class_and_ctor_args.pkl",
            "sha256": "5c98f006d99a71a0f1ddcbb31d8d73fe0a6dade8401e679f6af5b1bc943b4228",
            "size_bytes": 2_262,
        },
        {
            "path": "metadata.json",
            "sha256": "3a032ba54abcee8c9bcbb39e72fa05566912e94461d01f3c6228dc60e088bf12",
            "size_bytes": 197,
        },
        {
            "path": "module_state.pkl",
            "sha256": "44457ca5df7fa0e0e1f1d361d940136917fe8f71e984a1b0afaefb8ca3ced33b",
            "size_bytes": 604_772,
        },
    ],
}
LOCKED_PHYSICAL_RUNTIME = {
    "path": PHYSICAL_RUNTIME_RELATIVE_PATH.as_posix(),
    "sha256": "453917785a5e6c8ebed13ac813f7c226a8e8a4529ade3992fba9433f52ad4554",
    "size_bytes": 29_263,
}
LOCKED_R10_EXECUTION_LOCK = {
    "path": R10_EXECUTION_LOCK_RELATIVE_PATH.as_posix(),
    "sha256": "e6601a67d251f19586f25f834df09957450a78fac18ace5d620902314888fb82",
    "size_bytes": 44_027,
}
EXPECTED_TRANSITIVE_SOURCE_FILE_COUNT = 94
REQUIRED_TRANSITIVE_RUNTIME_SOURCES = (
    PHYSICAL_RUNTIME_RELATIVE_PATH.as_posix(),
    "Trajectory Generator/baseline_MLP/validation/v12r3/"
    "run_h0_primary_split_v12r3_autonomy_recovery.py",
    "validation/run_h0_primary_split_v9_causal_teacher.py",
    "validation/h0_forensic_rollout.py",
    "Trajectory Generator/baseline_MLP/env_factory.py",
    "Trajectory Generator/baseline_MLP/rollout_eval.py",
    "Trajectory Generator/binary_phase_adapter_v26.py",
    "Trajectory Generator/binary_phase_fsm_v26.py",
)
LOCKED_DIRECT_QUERY_RUNTIME_EVIDENCE = {
    "kind": "LOCKED_PRODUCTION_RUNTIME_AST",
    "runtime_sha256": LOCKED_PHYSICAL_RUNTIME["sha256"],
    "function": "run_case",
    "loop": "range(EXPECTED_STEPS)",
    "actor_binding": (
        "np.ascontiguousarray(observation_before[:EXPECTED_ACTOR_FEATURES], "
        "dtype=np.float32)"
    ),
    "policy_query_call": ("runtime._query_mean_std(module, actor, np=np, torch=torch)"),
    "static_policy_query_calls_per_loop_iteration": 1,
    "journal_bindings": {
        "v26_observation": "actor.tolist()",
        "candidate_mean": "mean.tolist()",
        "durable_step": "writer.write_step(step, row)",
    },
}
TEST_DIRECT_QUERY_RUNTIME_EVIDENCE = {
    "kind": "TEST_ADAPTER_EXPLICIT_ROW_BINDING",
    "static_policy_query_calls_per_loop_iteration": 1,
    "journal_bindings": {
        "v26_observation": "actor.tolist()",
        "candidate_mean": "mean.tolist()",
        "durable_step": "writer.write_step(step, row)",
    },
}
TEST_TRANSITIVE_RUNTIME_SOURCE_CLOSURE = {
    "anchor_execution_lock": {
        "path": "TEST_ADAPTER_LOCK",
        "sha256": "0" * 64,
        "size_bytes": 0,
    },
    "source_file_count": 0,
    "source_closure_sha256": "0" * 64,
    "required_runtime_sources": [],
}


class ProbeError(RuntimeError):
    """Raised when the diagnostic cannot preserve its one-shot contract."""


@dataclass(frozen=True)
class ProbeProtocol:
    """Filesystem and identity bindings; production uses ``PRODUCTION_PROTOCOL``."""

    artifact_root: Path
    allowed_destination_root: Path
    source_h0_id: str
    source_h0_relative_path: PurePosixPath
    locked_source_h0_tree: Mapping[str, Any]
    enforce_physical_runtime_lock: bool = True


PRODUCTION_PROTOCOL = ProbeProtocol(
    artifact_root=REPO_ROOT,
    allowed_destination_root=PACKAGE_ROOT / "artifacts",
    source_h0_id=SOURCE_H0_ID,
    source_h0_relative_path=SOURCE_H0_RELATIVE_PATH,
    locked_source_h0_tree=LOCKED_SOURCE_H0_TREE,
)

PhysicalRunner = Callable[..., Mapping[str, Any]]


def _finite_number(value: Any) -> bool:
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(float(value))
    )


def _inside(path: Path, root: Path, *, label: str) -> Path:
    resolved = path.expanduser().resolve()
    boundary = root.expanduser().resolve()
    try:
        resolved.relative_to(boundary)
    except ValueError as exc:
        raise ProbeError(f"{label} escaped its boundary: {resolved}") from exc
    return resolved


def _canonical_relative(value: PurePosixPath, *, label: str) -> PurePosixPath:
    raw = value.as_posix()
    if value.is_absolute() or ".." in value.parts or not raw or raw != str(value):
        raise ProbeError(f"{label} is not a canonical relative path: {raw!r}")
    return value


def tree_record(path: Path, *, artifact_root: Path) -> dict[str, Any]:
    """Return the project-standard content-addressed checkpoint tree record."""

    root = path.expanduser().resolve()
    boundary = artifact_root.expanduser().resolve()
    _inside(root, boundary, label="source H0")
    if not root.is_dir() or root.is_symlink():
        raise ProbeError(f"source H0 is not a regular directory: {root}")
    entries = sorted(root.rglob("*"))
    if any(entry.is_symlink() for entry in entries):
        raise ProbeError("source H0 tree contains a symlink")
    files = [entry for entry in entries if entry.is_file()]
    if not files:
        raise ProbeError("source H0 tree is empty")
    digest = hashlib.sha256()
    rows = []
    for item in files:
        relative = item.relative_to(root).as_posix()
        sha256 = forensic.sha256_file(item)
        size_bytes = item.stat().st_size
        rows.append({"path": relative, "sha256": sha256, "size_bytes": size_bytes})
        digest.update(relative.encode("utf-8"))
        digest.update(b"\0")
        digest.update(sha256.encode("ascii"))
        digest.update(b"\0")
        digest.update(str(size_bytes).encode("ascii"))
        digest.update(b"\n")
    return {
        "path": root.relative_to(boundary).as_posix(),
        "tree_sha256": digest.hexdigest(),
        "file_count": len(rows),
        "files": rows,
    }


def _source_path(protocol: ProbeProtocol) -> Path:
    relative = _canonical_relative(
        protocol.source_h0_relative_path,
        label="source_h0_relative_path",
    )
    return protocol.artifact_root.joinpath(*relative.parts).resolve()


def verify_source_h0(protocol: ProbeProtocol = PRODUCTION_PROTOCOL) -> dict[str, Any]:
    expected = copy.deepcopy(dict(protocol.locked_source_h0_tree))
    forensic.canonical_json_bytes(expected)
    observed = tree_record(_source_path(protocol), artifact_root=protocol.artifact_root)
    if observed != expected:
        raise ProbeError("locked source H0 tree drifted")
    if not protocol.source_h0_id.strip():
        raise ProbeError("source H0 identity is empty")
    return observed


def verify_physical_runtime() -> dict[str, Any]:
    path = REPO_ROOT.joinpath(*PHYSICAL_RUNTIME_RELATIVE_PATH.parts)
    observed = forensic.artifact_record(path, artifact_root=REPO_ROOT)
    if observed != LOCKED_PHYSICAL_RUNTIME:
        raise ProbeError("locked production physical runtime drifted")
    return observed


def verify_transitive_runtime_source_closure() -> dict[str, Any]:
    """Bind every production source inherited by the last frozen V26 run."""

    lock_path = REPO_ROOT.joinpath(*R10_EXECUTION_LOCK_RELATIVE_PATH.parts)
    lock_record = forensic.artifact_record(lock_path, artifact_root=REPO_ROOT)
    if lock_record != LOCKED_R10_EXECUTION_LOCK:
        raise ProbeError("R10 execution-lock anchor drifted")
    value = forensic.strict_json_load(lock_path)
    closure = (
        value.get("production_source_closure") if isinstance(value, Mapping) else None
    )
    if (
        not isinstance(closure, Mapping)
        or len(closure) != EXPECTED_TRANSITIVE_SOURCE_FILE_COUNT
    ):
        raise ProbeError("R10 transitive production source closure is malformed")
    if not set(REQUIRED_TRANSITIVE_RUNTIME_SOURCES).issubset(closure):
        raise ProbeError("R10 source closure lacks a required physical dependency")
    for relative, expected_value in sorted(closure.items()):
        if not isinstance(relative, str) or not isinstance(expected_value, Mapping):
            raise ProbeError("R10 source closure contains a malformed record")
        relative_path = _canonical_relative(
            PurePosixPath(relative), label="source path"
        )
        observed = forensic.artifact_record(
            REPO_ROOT.joinpath(*relative_path.parts),
            artifact_root=REPO_ROOT,
        )
        if observed != dict(expected_value):
            raise ProbeError(f"transitive production source drifted: {relative}")
    closure_digest = hashlib.sha256(
        forensic.canonical_json_bytes(dict(closure))
    ).hexdigest()
    result = {
        "anchor_execution_lock": copy.deepcopy(lock_record),
        "source_file_count": len(closure),
        "source_closure_sha256": closure_digest,
        "required_runtime_sources": list(REQUIRED_TRANSITIVE_RUNTIME_SOURCES),
    }
    forensic.canonical_json_bytes(result)
    return result


def verify_direct_query_runtime_evidence() -> dict[str, Any]:
    """Verify the locked runner's one-query/one-journal-row AST contract."""

    path = REPO_ROOT.joinpath(*PHYSICAL_RUNTIME_RELATIVE_PATH.parts)
    runtime_record = verify_physical_runtime()
    try:
        tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    except (OSError, SyntaxError, UnicodeError) as exc:
        raise ProbeError("production physical runtime AST could not be read") from exc
    functions = [
        node
        for node in tree.body
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
        and node.name == "run_case"
    ]
    if len(functions) != 1:
        raise ProbeError("locked runtime must expose exactly one run_case function")
    run_case_node = functions[0]
    loops = [
        node
        for node in ast.walk(run_case_node)
        if isinstance(node, ast.For)
        and ast.unparse(node.iter) == "range(EXPECTED_STEPS)"
    ]
    if len(loops) != 1:
        raise ProbeError("locked run_case must have one EXPECTED_STEPS loop")
    loop = loops[0]
    actor_assignments = [
        node
        for node in loop.body
        if isinstance(node, ast.Assign)
        and any(ast.unparse(target) == "actor" for target in node.targets)
    ]
    if (
        len(actor_assignments) != 1
        or ast.unparse(actor_assignments[0].value)
        != LOCKED_DIRECT_QUERY_RUNTIME_EVIDENCE["actor_binding"]
    ):
        raise ProbeError("locked run_case V26-to-actor AST binding drifted")
    query_calls = [
        node
        for node in ast.walk(run_case_node)
        if isinstance(node, ast.Call)
        and ast.unparse(node.func) == "runtime._query_mean_std"
    ]
    loop_query_calls = [
        node
        for node in ast.walk(loop)
        if isinstance(node, ast.Call)
        and ast.unparse(node.func) == "runtime._query_mean_std"
    ]
    if (
        len(query_calls) != 1
        or len(loop_query_calls) != 1
        or ast.unparse(loop_query_calls[0])
        != LOCKED_DIRECT_QUERY_RUNTIME_EVIDENCE["policy_query_call"]
    ):
        raise ProbeError("locked run_case direct policy query AST drifted")
    row_assignments = [
        node
        for node in loop.body
        if isinstance(node, ast.Assign)
        and any(ast.unparse(target) == "row" for target in node.targets)
        and isinstance(node.value, ast.Dict)
    ]
    if len(row_assignments) != 1:
        raise ProbeError("locked run_case row journal AST drifted")
    row_dict = row_assignments[0].value
    bindings = {
        key.value: ast.unparse(value)
        for key, value in zip(row_dict.keys, row_dict.values, strict=True)
        if isinstance(key, ast.Constant) and isinstance(key.value, str)
    }
    expected_bindings = LOCKED_DIRECT_QUERY_RUNTIME_EVIDENCE["journal_bindings"]
    if (
        bindings.get("v26_observation") != expected_bindings["v26_observation"]
        or bindings.get("candidate_mean") != expected_bindings["candidate_mean"]
    ):
        raise ProbeError("locked run_case query-to-row bindings drifted")
    durable_calls = [
        node
        for node in ast.walk(loop)
        if isinstance(node, ast.Call)
        and ast.unparse(node) == expected_bindings["durable_step"]
    ]
    if len(durable_calls) != 1:
        raise ProbeError("locked run_case durable step binding drifted")
    evidence = copy.deepcopy(LOCKED_DIRECT_QUERY_RUNTIME_EVIDENCE)
    evidence["runtime_sha256"] = runtime_record["sha256"]
    forensic.canonical_json_bytes(evidence)
    return evidence


def canonical_case(destination: Path) -> dict[str, Any]:
    return {
        "case_id": CASE_ID,
        "action_selection": "deterministic",
        "episode_start_offset_s": 2.156870983805102,
        "action_seed": None,
        "runtime_seed": 123,
        "sigma": 0.0,
        "destination": destination.as_posix(),
        "behavior": "SOURCE_H0_DIRECT_ON_RUNTIME_V26_35_ZERO_FIT",
    }


def _probe_contract(
    *,
    source_h0: Mapping[str, Any],
    physical_runtime: Mapping[str, Any],
    direct_query_runtime_evidence: Mapping[str, Any],
    transitive_runtime_source_closure: Mapping[str, Any],
) -> dict[str, Any]:
    return {
        "diagnostic_id": DIAGNOSTIC_ID,
        "source_h0_id": SOURCE_H0_ID,
        "source_h0": copy.deepcopy(dict(source_h0)),
        "source_h0_role": "DIRECT_POLICY_NOT_TEACHER",
        "inference_route": "RUNTIME_V26_FIRST_35_FLOAT32_FEATURES_DIRECT",
        "expected_steps": EXPECTED_STEPS,
        "penetration_limit_m": PENETRATION_LIMIT_M,
        "minimum_valid_cycles": MINIMUM_VALID_CYCLES,
        "event_contract_id": EVENT_CONTRACT_ID,
        "target_contract_id": TARGET_CONTRACT_ID,
        "morphology_weight_required": 0.0,
        "fit_executed": False,
        "teacher_enabled": False,
        "blending_enabled": False,
        "safety_latch_enabled": False,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "production_physical_runtime": copy.deepcopy(dict(physical_runtime)),
        "direct_query_runtime_evidence": copy.deepcopy(
            dict(direct_query_runtime_evidence)
        ),
        "transitive_runtime_source_closure": copy.deepcopy(
            dict(transitive_runtime_source_closure)
        ),
    }


def _load_rows(destination: Path) -> list[dict[str, Any]]:
    steps = destination / "steps"
    paths = sorted(steps.glob("*.json")) if steps.is_dir() else []
    expected = [f"{index:06d}.json" for index in range(1, len(paths) + 1)]
    if [path.name for path in paths] != expected:
        raise ProbeError("durable step journal is not contiguous")
    rows = []
    for index, path in enumerate(paths, start=1):
        value = forensic.strict_json_load(path)
        if not isinstance(value, Mapping) or value.get("step") != index:
            raise ProbeError(f"malformed durable step row: {path.name}")
        rows.append(dict(value))
    return rows


def analyze_rows(rows: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    """Recompute every gate metric from the immutable step prefix."""

    counters = {name: 0 for name in PURE_COUNTER_FIELDS}
    identities_exact = True
    observations_exact = True
    action_path_exact = True
    policy_distribution_exact = True
    sensors_exact = True
    forbidden_teacher_payload_absent = True
    zero_row_counters = True
    clipping_count = 0
    false_check_counts = {name: 0 for name in REQUIRED_STEP_CHECKS}
    invalid_event_count = 0
    phase_timeout_exceeded_count = 0
    phase_timeout_payload_exact = True
    valid_cycle_count = 0
    accepted_event_count = 0
    event_sources: set[str] = set()
    penetrations: list[float] = []
    candidate_mean_journal_row_count = 0
    direct_mean_action_binding_row_count = 0
    for expected_step, row in enumerate(rows, start=1):
        identities_exact = identities_exact and (
            row.get("step") == expected_step
            and row.get("schema_version") == SCHEMA_VERSION
            and row.get("protocol_id") == PROTOCOL_ID
            and row.get("stage_id") == STAGE_ID
            and row.get("case_id") == CASE_ID
        )
        observation = row.get("v26_observation")
        observations_exact = observations_exact and (
            isinstance(observation, list)
            and len(observation) == EXPECTED_ACTOR_FEATURES
            and all(_finite_number(value) for value in observation)
        )
        mean = row.get("candidate_mean")
        std = row.get("candidate_std")
        standard_normal = row.get("standard_normal")
        noise = row.get("single_noise")
        raw = row.get("raw_action")
        applied = row.get("applied_action")
        vectors = (mean, noise, raw, applied)
        vectors_exact = all(
            isinstance(vector, list)
            and len(vector) == EXPECTED_ACTION_DIM
            and all(_finite_number(value) for value in vector)
            for vector in vectors
        )
        action_path_exact = action_path_exact and vectors_exact
        distribution_vectors_exact = all(
            isinstance(vector, list)
            and len(vector) == EXPECTED_ACTION_DIM
            and all(_finite_number(value) for value in vector)
            for vector in (std, standard_normal)
        )
        policy_distribution_exact = (
            policy_distribution_exact and distribution_vectors_exact
        )
        if distribution_vectors_exact:
            policy_distribution_exact = policy_distribution_exact and all(
                math.isclose(
                    float(std[index]),
                    EXPECTED_H0_STD,
                    rel_tol=0.0,
                    abs_tol=1.0e-9,
                )
                and float(standard_normal[index]) == 0.0
                for index in range(EXPECTED_ACTION_DIM)
            )
        if vectors_exact:
            candidate_mean_journal_row_count += 1
            direct_binding = all(
                float(noise[index]) == 0.0
                and math.isclose(
                    float(raw[index]),
                    float(mean[index]),
                    rel_tol=0.0,
                    abs_tol=1.0e-7,
                )
                for index in range(EXPECTED_ACTION_DIM)
            )
            action_path_exact = action_path_exact and direct_binding
            direct_mean_action_binding_row_count += int(direct_binding)
            clipping_count += sum(
                raw[index] != applied[index] for index in range(EXPECTED_ACTION_DIM)
            )
        journal = row.get("observer_raw_sensor_journal")
        samples = journal.get("samples") if isinstance(journal, Mapping) else None
        sensors_exact = sensors_exact and (
            row.get("raw_sensor_sample_count") == EXPECTED_RAW_SENSOR_SAMPLES_PER_STEP
            and isinstance(samples, list)
            and len(samples) == EXPECTED_RAW_SENSOR_SAMPLES_PER_STEP
        )
        forbidden_teacher_payload_absent = (
            forbidden_teacher_payload_absent
            and FORBIDDEN_TEACHER_FIELDS.isdisjoint(row)
            and row.get("teacher_enabled") is False
            and row.get("blending_enabled") is False
            and row.get("safety_latch_enabled") is False
        )
        for name in PURE_COUNTER_FIELDS:
            value = row.get(name)
            if type(value) is int:
                counters[name] += value
            else:
                zero_row_counters = False
            zero_row_counters = zero_row_counters and value == 0
        checks = row.get("checks")
        for name in REQUIRED_STEP_CHECKS:
            if not isinstance(checks, Mapping) or checks.get(name) is not True:
                false_check_counts[name] += 1
        phase = row.get("phase_fsm")
        if isinstance(phase, Mapping):
            invalid_event_count = max(
                invalid_event_count,
                int(float(phase.get("invalid_event_count", 0))),
            )
            valid_cycle_count = max(
                valid_cycle_count,
                int(float(phase.get("valid_cycle_count", 0))),
            )
            accepted = phase.get("accepted_transitions_this_step")
            if isinstance(accepted, list):
                accepted_event_count += len(accepted)
            source = phase.get("event_source")
            if isinstance(source, str):
                event_sources.add(source)
            timeout_exceeded = phase.get("timeout_exceeded")
            if _finite_number(timeout_exceeded):
                phase_timeout_exceeded_count += int(float(timeout_exceeded) != 0.0)
            else:
                phase_timeout_payload_exact = False
        else:
            phase_timeout_payload_exact = False
        penetration = row.get("grf_penetration_m")
        if _finite_number(penetration):
            penetrations.append(float(penetration))
        else:
            false_check_counts["finite"] += 1
    final = rows[-1] if rows else {}
    zero_counters = all(value == 0 for value in counters.values())
    timeout_check_failure_count = false_check_counts["no_timeout"]
    anomaly_count = (
        sum(false_check_counts.values())
        + invalid_event_count
        + phase_timeout_exceeded_count
    )
    return {
        "row_count": len(rows),
        "identities_exact": identities_exact,
        "v26_observation_35_finite": observations_exact,
        "direct_mean_zero_noise_action_exact": action_path_exact,
        "deterministic_h0_distribution_exact": policy_distribution_exact,
        "candidate_mean_journal_row_count": candidate_mean_journal_row_count,
        "direct_mean_action_binding_row_count": (direct_mean_action_binding_row_count),
        "raw_sensor_samples_exact": sensors_exact,
        "forbidden_teacher_payload_absent": forbidden_teacher_payload_absent,
        "per_row_zero_counters": zero_row_counters,
        "zero_counters": zero_counters,
        "counters": counters,
        "clipped_action_value_count": clipping_count,
        "false_check_counts": false_check_counts,
        "anomaly_count": anomaly_count,
        "invalid_event_count": invalid_event_count,
        "timeout_check_failure_count": timeout_check_failure_count,
        "phase_timeout_payload_exact": phase_timeout_payload_exact,
        "phase_timeout_exceeded_count": phase_timeout_exceeded_count,
        "valid_cycle_count": valid_cycle_count,
        "accepted_event_count": accepted_event_count,
        "event_sources": sorted(event_sources),
        "raw_sensor_sample_count": sum(
            int(row.get("raw_sensor_sample_count", 0)) for row in rows
        ),
        "grf_penetration_max_m": max(penetrations, default=0.0),
        "end_reason": final.get("end_reason"),
        "terminated": final.get("terminated"),
        "truncated": final.get("truncated"),
    }


def _runtime_error_payload(error: BaseException | None) -> dict[str, str] | None:
    if error is None:
        return None
    return {"type": type(error).__name__, "message": str(error)}


def build_early_summary(
    *,
    rows: Sequence[Mapping[str, Any]],
    run_start: Mapping[str, Any],
    activity: Mapping[str, int],
    probe_contract: Mapping[str, Any],
    source_after: Mapping[str, Any],
    runtime_error: BaseException,
) -> dict[str, Any]:
    audit = analyze_rows(rows)
    summary = {
        "schema_version": SCHEMA_VERSION,
        "status": EARLY_COMPLETE_STATUS,
        "protocol_id": PROTOCOL_ID,
        "stage_id": STAGE_ID,
        "case": copy.deepcopy(run_start.get("case")),
        "steps": len(rows),
        "expected_steps": EXPECTED_STEPS,
        "gate_evaluated": False,
        "source_h0_id": probe_contract["source_h0_id"],
        "source_h0": copy.deepcopy(probe_contract["source_h0"]),
        "source_h0_role": "DIRECT_POLICY_NOT_TEACHER",
        "inference_route": probe_contract["inference_route"],
        "probe_contract": copy.deepcopy(dict(probe_contract)),
        "source_h0_after_rollout": copy.deepcopy(dict(source_after)),
        "source_h0_immutable_during_rollout": (
            source_after == probe_contract["source_h0"]
        ),
        "production_runner_completed_without_exception": False,
        "production_runner_error": _runtime_error_payload(runtime_error),
        "teacher_enabled": False,
        "blending_enabled": False,
        "safety_latch_enabled": False,
        "fit_executed": False,
        "actor_fit_count": 0,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "morphology_weight": 0.0,
        "event_contract_id": EVENT_CONTRACT_ID,
        "target_contract_id": TARGET_CONTRACT_ID,
        "binary_phase_fsm_mode": "binary_active",
        "activity": copy.deepcopy(dict(activity)),
        **audit,
    }
    forensic.canonical_json_bytes(summary)
    return summary


def evaluate_gate(
    *,
    summary: Mapping[str, Any],
    run_start: Mapping[str, Any],
    rows: Sequence[Mapping[str, Any]],
    activity: Mapping[str, int],
    protocol: ProbeProtocol,
    source_after: Mapping[str, Any],
    runtime_error: BaseException | None,
    finalized_artifacts: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    audit = analyze_rows(rows)
    expected_source = dict(protocol.locked_source_h0_tree)
    expected_runtime = (
        LOCKED_PHYSICAL_RUNTIME
        if protocol.enforce_physical_runtime_lock
        else {"path": "TEST_ADAPTER", "sha256": "0" * 64, "size_bytes": 0}
    )
    expected_query_evidence = (
        LOCKED_DIRECT_QUERY_RUNTIME_EVIDENCE
        if protocol.enforce_physical_runtime_lock
        else TEST_DIRECT_QUERY_RUNTIME_EVIDENCE
    )
    expected_transitive_closure = (
        verify_transitive_runtime_source_closure()
        if protocol.enforce_physical_runtime_lock
        else TEST_TRANSITIVE_RUNTIME_SOURCE_CLOSURE
    )
    start_contract = run_start.get("probe_contract")
    summary_contract = summary.get("probe_contract")
    checks = {
        "identity": summary.get("schema_version") == SCHEMA_VERSION
        and summary.get("protocol_id") == PROTOCOL_ID
        and summary.get("stage_id") == STAGE_ID,
        "source_h0_locked": (
            isinstance(start_contract, Mapping)
            and isinstance(summary_contract, Mapping)
            and start_contract.get("source_h0_id") == protocol.source_h0_id
            and summary_contract.get("source_h0_id") == protocol.source_h0_id
            and start_contract.get("source_h0") == expected_source
            and summary_contract.get("source_h0") == expected_source
            and dict(source_after) == expected_source
            and start_contract.get("source_h0_role") == "DIRECT_POLICY_NOT_TEACHER"
            and summary_contract.get("source_h0_role") == "DIRECT_POLICY_NOT_TEACHER"
        ),
        "hardest_case_exact": (
            isinstance(run_start.get("case"), Mapping)
            and run_start["case"].get("case_id") == CASE_ID
            and run_start["case"].get("action_selection") == "deterministic"
            and run_start["case"].get("action_seed") is None
            and run_start["case"].get("runtime_seed") == 123
            and float(run_start["case"].get("sigma", -1.0)) == 0.0
            and math.isclose(
                float(run_start["case"].get("episode_start_offset_s", -1.0)),
                2.156870983805102,
                rel_tol=0.0,
                abs_tol=0.0,
            )
            and run_start["case"].get("behavior")
            == "SOURCE_H0_DIRECT_ON_RUNTIME_V26_35_ZERO_FIT"
        ),
        "direct_v26_35": audit["identities_exact"]
        and audit["v26_observation_35_finite"]
        and audit["direct_mean_zero_noise_action_exact"]
        and audit["deterministic_h0_distribution_exact"]
        and audit["raw_sensor_samples_exact"],
        "detector_target_contracts_exact": (
            isinstance(start_contract, Mapping)
            and isinstance(summary_contract, Mapping)
            and start_contract.get("event_contract_id") == EVENT_CONTRACT_ID
            and summary_contract.get("event_contract_id") == EVENT_CONTRACT_ID
            and start_contract.get("target_contract_id") == TARGET_CONTRACT_ID
            and summary_contract.get("target_contract_id") == TARGET_CONTRACT_ID
            and summary.get("event_contract_id") == EVENT_CONTRACT_ID
            and summary.get("target_contract_id") == TARGET_CONTRACT_ID
            and summary.get("binary_phase_fsm_mode") == "binary_active"
        ),
        "direct_source_h0_query_evidence_per_step": (
            isinstance(start_contract, Mapping)
            and isinstance(summary_contract, Mapping)
            and start_contract.get("production_physical_runtime") == expected_runtime
            and summary_contract.get("production_physical_runtime") == expected_runtime
            and start_contract.get("direct_query_runtime_evidence")
            == expected_query_evidence
            and summary_contract.get("direct_query_runtime_evidence")
            == expected_query_evidence
            and start_contract.get("transitive_runtime_source_closure")
            == expected_transitive_closure
            and summary_contract.get("transitive_runtime_source_closure")
            == expected_transitive_closure
            and audit["candidate_mean_journal_row_count"] == len(rows)
            and audit["direct_mean_action_binding_row_count"] == len(rows)
        ),
        "full_horizon": audit["row_count"] == EXPECTED_STEPS,
        "complete_episode_shape": audit["terminated"] is False
        and audit["truncated"] is True
        and audit["end_reason"] == "episode_time_limit",
        "penetration": (audit["grf_penetration_max_m"] < PENETRATION_LIMIT_M),
        "cycles": audit["valid_cycle_count"] >= MINIMUM_VALID_CYCLES,
        "zero_clipping": audit["clipped_action_value_count"] == 0,
        "zero_anomalies": audit["anomaly_count"] == 0,
        "zero_timeouts": (
            audit["timeout_check_failure_count"] == 0
            and audit["phase_timeout_payload_exact"]
            and audit["phase_timeout_exceeded_count"] == 0
        ),
        "binary_detector_active": audit["event_sources"] == [EVENT_SOURCE],
        "zero_teacher_blend_latch": (
            audit["forbidden_teacher_payload_absent"]
            and audit["per_row_zero_counters"]
            and audit["zero_counters"]
            and run_start.get("teacher_enabled") is False
            and run_start.get("blending_enabled") is False
            and run_start.get("safety_latch_enabled") is False
        ),
        "zero_fit_and_updates": (
            isinstance(start_contract, Mapping)
            and start_contract.get("fit_executed") is False
            and start_contract.get("actor_updates") == 0
            and start_contract.get("critic_updates") == 0
            and start_contract.get("ppo_updates") == 0
            and summary.get("actor_updates") == 0
            and summary.get("critic_updates") == 0
            and summary.get("ppo_updates") == 0
        ),
        "morphology_zero": (
            isinstance(start_contract, Mapping)
            and float(start_contract.get("morphology_weight_required", -1.0)) == 0.0
            and float(summary.get("morphology_weight", -1.0)) == 0.0
        ),
        "activity_exact": (
            activity.get("environment_reset_calls") == 1
            and activity.get("environment_step_calls") == len(rows)
            and activity.get("raw_sensor_sample_count")
            == len(rows) * EXPECTED_RAW_SENSOR_SAMPLES_PER_STEP
        ),
        "summary_trace_binding": summary.get("steps") == len(rows),
        "production_runner_clean": runtime_error is None,
    }
    integrity_names = (
        "identity",
        "source_h0_locked",
        "hardest_case_exact",
        "direct_v26_35",
        "detector_target_contracts_exact",
        "direct_source_h0_query_evidence_per_step",
        "zero_anomalies",
        "binary_detector_active",
        "zero_teacher_blend_latch",
        "zero_fit_and_updates",
        "morphology_zero",
        "activity_exact",
        "summary_trace_binding",
    )
    physical_names = (
        "full_horizon",
        "complete_episode_shape",
        "penetration",
        "cycles",
        "zero_clipping",
        "zero_timeouts",
        "production_runner_clean",
    )
    diagnostic_integrity_passed = all(checks[name] for name in integrity_names)
    physical_passed = all(checks[name] for name in physical_names)
    passed = diagnostic_integrity_passed and physical_passed
    gate = {
        "schema_version": SCHEMA_VERSION,
        "status": GATE_PASS_STATUS if passed else GATE_FAIL_STATUS,
        "passed": passed,
        "protocol_id": PROTOCOL_ID,
        "diagnostic_id": DIAGNOSTIC_ID,
        "case_id": CASE_ID,
        "source_h0_id": protocol.source_h0_id,
        "source_h0": copy.deepcopy(expected_source),
        "checks": checks,
        "diagnostic_integrity_passed": diagnostic_integrity_passed,
        "physical_passed": physical_passed,
        "autonomy_passed": physical_passed,
        "trace_audit": audit,
        "production_runner_error": _runtime_error_payload(runtime_error),
        "inputs": (
            None
            if finalized_artifacts is None
            else copy.deepcopy(dict(finalized_artifacts))
        ),
        "next_stage": "DIAGNOSTIC_PASS_ONLY" if passed else "STOP_FAIL_CLOSED",
        "training_or_qualification_authorized": False,
    }
    forensic.canonical_json_bytes(gate)
    return gate


def _load_production_physical() -> Any:
    return importlib.import_module("h0_v12r6_physical_development")


def _publish_closure_failure(
    *,
    destination: Path,
    protocol: ProbeProtocol,
    error: BaseException,
) -> dict[str, Any] | None:
    """Best-effort durable receipt for any post-start closure exception."""

    writer = forensic.ForensicRolloutWriter(
        destination,
        artifact_root=protocol.artifact_root,
    )
    if not writer.run_start_path.is_file() or writer.failure_path.is_file():
        return None
    return writer.publish_failure(
        end_reason="post_rollout_closure_failure",
        error=error,
        status=CLOSURE_FAIL_STATUS,
        details={
            "retry_authorized": False,
            "summary_or_gate_may_be_partial": True,
        },
    )


def _verify_closed_rollout(
    *,
    writer: Any,
    expected_run_start: Mapping[str, Any],
    expected_rows: Sequence[Mapping[str, Any]],
    expected_summary: Mapping[str, Any],
    expected_gate: Mapping[str, Any],
    expected_gate_record: Mapping[str, Any],
) -> dict[str, Any]:
    """Re-read and bind the complete immutable rollout after gate publication."""

    if os.path.lexists(writer.failure_path):
        raise ProbeError("closed rollout unexpectedly contains failure.json")
    run_start_value = forensic.strict_json_load(writer.run_start_path)
    trace_value = forensic.strict_json_load(writer.trace_path)
    summary_value = forensic.strict_json_load(writer.summary_path)
    gate_value = forensic.strict_json_load(writer.gate_path)
    rows_value = _load_rows(writer.run_directory)
    comparisons = {
        "run_start": (run_start_value, expected_run_start),
        "step_journal": (rows_value, list(expected_rows)),
        "trace_to_step_journal": (trace_value, rows_value),
        "summary": (summary_value, expected_summary),
        "gate": (gate_value, expected_gate),
    }
    for label, (observed, expected) in comparisons.items():
        if forensic.canonical_json_bytes(observed) != forensic.canonical_json_bytes(
            expected
        ):
            raise ProbeError(f"closed rollout {label} binding failed")
    finalized = writer.finalized_artifact_records()
    if not isinstance(gate_value, Mapping) or gate_value.get("inputs") != finalized:
        raise ProbeError("closed rollout gate input hashes are not exact")
    observed_gate_record = forensic.artifact_record(
        writer.gate_path,
        artifact_root=writer.artifact_root,
    )
    if observed_gate_record != dict(expected_gate_record):
        raise ProbeError("closed rollout gate record changed after publication")
    artifacts = writer.artifact_records()
    if artifacts.get("gate") != observed_gate_record:
        raise ProbeError("closed rollout artifact inventory is not gate-bound")
    verification = {
        "schema_version": SCHEMA_VERSION,
        "status": "VERIFIED_H0_V12R11_CLOSED_ROLLOUT",
        "passed": True,
        "failure_absent": True,
        "run_start_bound": True,
        "step_journal_bound": True,
        "trace_bound_to_step_journal": True,
        "summary_bound": True,
        "gate_inputs_bound": True,
        "gate_record": copy.deepcopy(observed_gate_record),
        "artifacts": copy.deepcopy(artifacts),
    }
    forensic.canonical_json_bytes(verification)
    closure_receipt = {
        **copy.deepcopy(verification),
        "status": CLOSURE_PASS_STATUS,
        "protocol_id": PROTOCOL_ID,
        "diagnostic_id": DIAGNOSTIC_ID,
        "case_id": CASE_ID,
        "retry_authorized": False,
        "resume_authorized": False,
    }
    closure_path = writer.run_directory / "closure_receipt.json"
    forensic.write_json_exclusive(closure_path, closure_receipt)
    persisted = forensic.strict_json_load(closure_path)
    if forensic.canonical_json_bytes(persisted) != forensic.canonical_json_bytes(
        closure_receipt
    ):
        raise ProbeError("durable closure receipt changed after publication")
    verification["closure_receipt"] = forensic.artifact_record(
        closure_path,
        artifact_root=writer.artifact_root,
    )
    return verification


def _publish_aggregate_and_gate(
    *,
    destination: Path,
    protocol: ProbeProtocol,
    activity: Mapping[str, int],
    probe_contract: Mapping[str, Any],
    source_after: Mapping[str, Any],
    runtime_error: BaseException | None,
) -> dict[str, Any]:
    writer = forensic.ForensicRolloutWriter(
        destination,
        artifact_root=protocol.artifact_root,
    )
    rows = _load_rows(destination)
    if not rows:
        if writer.run_start_path.is_file():
            writer.publish_failure(
                end_reason="production_runner_failed_before_first_step",
                error=runtime_error or "production runner returned zero steps",
                status=GATE_FAIL_STATUS,
            )
        raise ProbeError("production physical runtime completed no durable step")
    run_start_value = forensic.strict_json_load(writer.run_start_path)
    if not isinstance(run_start_value, Mapping):
        raise ProbeError("run_start.json is malformed")
    run_start = dict(run_start_value)
    if writer.summary_path.is_file():
        summary_value = forensic.strict_json_load(writer.summary_path)
        if not isinstance(summary_value, Mapping):
            raise ProbeError("summary.json is malformed")
        summary = dict(summary_value)
    else:
        partial = {
            "schema_version": SCHEMA_VERSION,
            "status": PARTIAL_STATUS,
            "protocol_id": PROTOCOL_ID,
            "stage_id": STAGE_ID,
            "steps": len(rows),
            "gate_evaluated": False,
            "source_h0_id": protocol.source_h0_id,
        }
        if runtime_error is None:
            runtime_error = ProbeError(
                "production runner returned without publishing its summary"
            )
        summary = build_early_summary(
            rows=rows,
            run_start=run_start,
            activity=activity,
            probe_contract=probe_contract,
            source_after=source_after,
            runtime_error=runtime_error,
        )
        writer.finalize_before_gate(
            trace=rows,
            partial_summary=partial,
            summary=summary,
        )

    evaluated_gate: dict[str, Any] | None = None

    def gate_callback(finalized: dict[str, Any]) -> dict[str, Any]:
        nonlocal evaluated_gate
        evaluated_gate = evaluate_gate(
            summary=summary,
            run_start=run_start,
            rows=rows,
            activity=activity,
            protocol=protocol,
            source_after=source_after,
            runtime_error=runtime_error,
            finalized_artifacts=finalized,
        )
        return evaluated_gate

    gate_record = writer.run_gate(gate_callback)
    if evaluated_gate is None:
        raise ProbeError("gate callback returned without an evaluated gate")
    gate_value = forensic.strict_json_load(writer.gate_path)
    if not isinstance(gate_value, Mapping):
        raise ProbeError("published gate.json is malformed")
    gate = dict(gate_value)
    closure_verification = _verify_closed_rollout(
        writer=writer,
        expected_run_start=run_start,
        expected_rows=rows,
        expected_summary=summary,
        expected_gate=evaluated_gate,
        expected_gate_record=gate_record,
    )
    return {
        "destination": destination,
        "summary": summary,
        "gate": gate,
        "gate_record": gate_record,
        "closure_verification": closure_verification,
        "artifacts": closure_verification["artifacts"],
    }


def run_probe(
    *,
    destination: Path = DEFAULT_DESTINATION,
    protocol: ProbeProtocol = PRODUCTION_PROTOCOL,
    physical_runner: PhysicalRunner | None = None,
    physical_config: Any = None,
) -> dict[str, Any]:
    """Run exactly one direct source-H0 physical probe; destination is one-shot."""

    artifact_root = protocol.artifact_root.expanduser().resolve()
    if not artifact_root.is_dir() or artifact_root.is_symlink():
        raise ProbeError("artifact_root must be an existing regular directory")
    allowed = _inside(
        protocol.allowed_destination_root,
        artifact_root,
        label="allowed destination root",
    )
    target = _inside(destination, allowed, label="probe destination")
    if os.path.lexists(target):
        raise ProbeError(f"probe destination already exists/no retry: {target}")
    source_before = verify_source_h0(protocol)
    if protocol.enforce_physical_runtime_lock:
        physical_runtime = verify_physical_runtime()
        direct_query_runtime_evidence = verify_direct_query_runtime_evidence()
        transitive_runtime_source_closure = verify_transitive_runtime_source_closure()
    else:
        physical_runtime = {
            "path": "TEST_ADAPTER",
            "sha256": "0" * 64,
            "size_bytes": 0,
        }
        direct_query_runtime_evidence = TEST_DIRECT_QUERY_RUNTIME_EVIDENCE
        transitive_runtime_source_closure = TEST_TRANSITIVE_RUNTIME_SOURCE_CLOSURE
    probe_contract = _probe_contract(
        source_h0=source_before,
        physical_runtime=physical_runtime,
        direct_query_runtime_evidence=direct_query_runtime_evidence,
        transitive_runtime_source_closure=transitive_runtime_source_closure,
    )
    # Test protocols retain their own locked identity without weakening the
    # production binding embedded by ``_probe_contract``.
    probe_contract["source_h0_id"] = protocol.source_h0_id
    activity = {name: 0 for name in ALLOWED_ACTIVITY_FIELDS}

    def activity_callback(name: str, amount: int = 1) -> None:
        if name not in activity or type(amount) is not int or amount < 0:
            raise ProbeError(f"unexpected production activity: {name}/{amount!r}")
        activity[name] += amount

    if physical_runner is None:
        physical = _load_production_physical()
        physical_runner = physical.run_case
        physical_config = physical.PhysicalDevelopmentConfig(
            protocol_id=PROTOCOL_ID,
            start_status=START_STATUS,
            partial_status=PARTIAL_STATUS,
            complete_status=COMPLETE_STATUS,
            schema_version=SCHEMA_VERSION,
            artifact_root=artifact_root,
            progress_label="V12R11 source H0 direct V26 +0.20 probe",
            progress_every=25,
        )
    elif physical_config is None:
        physical_config = {"protocol_id": PROTOCOL_ID, "schema_version": SCHEMA_VERSION}

    metadata = {
        "diagnostic_id": DIAGNOSTIC_ID,
        "source_h0_id": protocol.source_h0_id,
        "source_h0": copy.deepcopy(source_before),
        "source_h0_role": "DIRECT_POLICY_NOT_TEACHER",
        "inference_route": "RUNTIME_V26_FIRST_35_FLOAT32_FEATURES_DIRECT",
        "target_contract_id": TARGET_CONTRACT_ID,
        "probe_contract": copy.deepcopy(probe_contract),
    }
    runtime_error: BaseException | None = None
    try:
        physical_runner(
            config=physical_config,
            case=canonical_case(target),
            destination=target,
            module_path=_source_path(protocol),
            activity_callback=activity_callback,
            start_metadata=metadata,
            summary_metadata=metadata,
        )
    except Exception as exc:  # Preserve and gate the durable physical prefix.
        runtime_error = exc
    try:
        source_after = verify_source_h0(protocol)
        if protocol.enforce_physical_runtime_lock:
            source_closure_after = verify_transitive_runtime_source_closure()
            if source_closure_after != transitive_runtime_source_closure:
                raise ProbeError(
                    "transitive runtime source closure changed during rollout"
                )
        return _publish_aggregate_and_gate(
            destination=target,
            protocol=protocol,
            activity=activity,
            probe_contract=probe_contract,
            source_after=source_after,
            runtime_error=runtime_error,
        )
    except Exception as closure_error:
        try:
            failure_record = _publish_closure_failure(
                destination=target,
                protocol=protocol,
                error=closure_error,
            )
        except Exception as failure_error:
            raise ProbeError(
                "post-rollout closure failed and failure receipt could not be "
                f"published: {failure_error}"
            ) from closure_error
        suffix = (
            "no run_start was available"
            if failure_record is None
            else f"failure receipt sha256={failure_record['sha256']}"
        )
        raise ProbeError(f"post-rollout closure failed; {suffix}") from closure_error


def describe_protocol() -> dict[str, Any]:
    """Return an execution-free review surface; this performs no filesystem I/O."""

    return {
        "schema_version": SCHEMA_VERSION,
        "status": "READY_SOURCE_ONLY_NOT_EXECUTED",
        "protocol_id": PROTOCOL_ID,
        "diagnostic_id": DIAGNOSTIC_ID,
        "case_id": CASE_ID,
        "destination": DEFAULT_DESTINATION.relative_to(REPO_ROOT).as_posix(),
        "source_h0_id": SOURCE_H0_ID,
        "source_h0": copy.deepcopy(LOCKED_SOURCE_H0_TREE),
        "production_physical_runtime": copy.deepcopy(LOCKED_PHYSICAL_RUNTIME),
        "transitive_runtime_source_anchor": copy.deepcopy(LOCKED_R10_EXECUTION_LOCK),
        "transitive_runtime_source_file_count": (EXPECTED_TRANSITIVE_SOURCE_FILE_COUNT),
        "gate": {
            "steps": EXPECTED_STEPS,
            "penetration_strictly_below_m": PENETRATION_LIMIT_M,
            "minimum_valid_cycles": MINIMUM_VALID_CYCLES,
            "zero_clipping": True,
            "zero_anomalies": True,
            "zero_timeouts": True,
        },
        "forbidden": [
            "fit",
            "teacher_role_or_query",
            "blend",
            "safety_latch",
            "actor_or_critic_or_ppo_update",
            "retry_or_resume",
        ],
        "rollout_executed": False,
    }


__all__ = [
    "CASE_ID",
    "CLOSURE_FAIL_STATUS",
    "DEFAULT_DESTINATION",
    "DIAGNOSTIC_ID",
    "EXPECTED_STEPS",
    "GATE_FAIL_STATUS",
    "GATE_PASS_STATUS",
    "LOCKED_R10_EXECUTION_LOCK",
    "LOCKED_SOURCE_H0_TREE",
    "LOCKED_DIRECT_QUERY_RUNTIME_EVIDENCE",
    "PENETRATION_LIMIT_M",
    "PRODUCTION_PROTOCOL",
    "PROTOCOL_ID",
    "ProbeError",
    "ProbeProtocol",
    "SOURCE_H0_ID",
    "analyze_rows",
    "build_early_summary",
    "canonical_case",
    "describe_protocol",
    "evaluate_gate",
    "run_probe",
    "tree_record",
    "verify_physical_runtime",
    "verify_direct_query_runtime_evidence",
    "verify_source_h0",
    "verify_transitive_runtime_source_closure",
]
