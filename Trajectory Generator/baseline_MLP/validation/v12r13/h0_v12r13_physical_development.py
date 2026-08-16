"""Fail-closed V12R13 P3 pure-policy physical-development contract.

The module is deliberately import-inert: it imports neither Ray nor the
OpenSim environment.  The CLI runner loads the already-qualified V12R6
physical runtime only after every P2, freeze, lock, closure, and no-clobber
precondition has passed.

P3 consumes one and only one actor tree: the canonical W256 P2 output.  P0 is
not an accepted alias.  The first pure rollout is the +0.20 s discriminator;
only a PASS opens the remaining five cases in their preregistered risk-first
order.  Any failed case closes the lineage without retry or resume.
"""

from __future__ import annotations

import copy
import hashlib
import json
import math
import os
import stat
from dataclasses import dataclass
from pathlib import Path, PurePosixPath
from typing import Any, Mapping, Sequence


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
REVISION_ROOT = Path(__file__).resolve().parent

SCHEMA_VERSION = 1213
REVISION = "2026-08-16"
PROTOCOL_ID = "AB06_H0_V12R13_V26_INVARIANT_SAFE_TEACHER"
DEVELOPMENT_ID = "H0_V12R13_P3_PURE_V26_W256_DEVELOPMENT_V1"
TOPOLOGY_ID = "STANDARD_ACTOR_35_256_256_4_TANH"
EVENT_CONTRACT_ID = "binary_point_v25+heel_qualified_fsm_v2"
TARGET_CONTRACT_ID = "primary_grf_split_v1+binary_point_v25+heel_qualified_fsm_v2"
EVENT_SOURCE = "binary_active_v26"
P2_RECEIPT_STATUS = "PASS_H0_V12R13_P2_MASKED_CANDIDATE_EXPOSED_FIT_RECEIPT"
P2_GATE_STATUS = "PASS_H0_V12R13_P2_MASKED_CANDIDATE_EXPOSED_FIT"
P2_SUMMARY_STATUS = "COMPLETE_H0_V12R13_P2_MASKED_CANDIDATE_EXPOSED_FIT"
P2_NEXT_STAGE = "FREEZE_THEN_RUN_P3_PLUS_0P20_PURE"

EXPECTED_STEPS = 500
EXPECTED_CONTROL_WINDOWS = 5_000
EXPECTED_RAW_SENSOR_SAMPLES = 5_000
EXPECTED_RAW_SENSOR_SAMPLES_PER_STEP = 10
EXPECTED_POLICY_DT_S = 0.01
EXPECTED_ACTOR_FEATURES = 35
EXPECTED_ACTION_DIM = 2
EXPECTED_SIGMA = 0.005
MINIMUM_VALID_CYCLES = 2
PENETRATION_LIMIT_M = 0.025
MORPHOLOGY_WEIGHT = 0.0

START_STATUS = "STARTED_H0_V12R13_P3_PURE_PHYSICAL_CASE"
PARTIAL_STATUS = "PERSISTED_H0_V12R13_P3_PURE_CASE_BEFORE_GATE"
COMPLETE_STATUS = "COMPLETE_H0_V12R13_P3_PURE_PHYSICAL_CASE"
EARLY_COMPLETE_STATUS = "COMPLETE_H0_V12R13_P3_PURE_CASE_EARLY_TERMINATION"
CASE_PASS_STATUS = "PASS_H0_V12R13_P3_PURE_PHYSICAL_CASE"
CASE_FAIL_STATUS = "FAIL_H0_V12R13_P3_PURE_PHYSICAL_CASE_TERMINAL"
AGGREGATE_PASS_STATUS = "PASS_H0_V12R13_P3_PURE_PHYSICAL_DEVELOPMENT"
PIPELINE_PASS_STATUS = "PASS_H0_V12R13_P3_PURE_PIPELINE_TERMINAL"
PIPELINE_FAIL_STATUS = "FAIL_H0_V12R13_P3_PURE_PIPELINE_TERMINAL"
FREEZE_PASS_STATUS = "PASS_H0_V12R13_P3_PROTOCOL_FREEZE"
LOCK_PASS_STATUS = "PASS_H0_V12R13_P3_EXECUTION_LOCK"

BASE_START_S = 1.956870983805102
DEVELOPMENT_CASES = (
    {
        "case_id": "deterministic_offset_plus_0p20",
        "action_selection": "deterministic",
        "episode_start_offset_s": BASE_START_S + 0.20,
        "action_seed": None,
        "runtime_seed": 123,
        "sigma": 0.0,
    },
    {
        "case_id": "stochastic_nominal_seed_127",
        "action_selection": "stochastic",
        "episode_start_offset_s": BASE_START_S,
        "action_seed": 127,
        "runtime_seed": 127,
        "sigma": EXPECTED_SIGMA,
    },
    {
        "case_id": "deterministic_offset_minus_0p20",
        "action_selection": "deterministic",
        "episode_start_offset_s": BASE_START_S - 0.20,
        "action_seed": None,
        "runtime_seed": 123,
        "sigma": 0.0,
    },
    {
        "case_id": "deterministic_offset_nominal",
        "action_selection": "deterministic",
        "episode_start_offset_s": BASE_START_S,
        "action_seed": None,
        "runtime_seed": 123,
        "sigma": 0.0,
    },
    {
        "case_id": "stochastic_nominal_seed_126",
        "action_selection": "stochastic",
        "episode_start_offset_s": BASE_START_S,
        "action_seed": 126,
        "runtime_seed": 126,
        "sigma": EXPECTED_SIGMA,
    },
    {
        "case_id": "stochastic_nominal_seed_128",
        "action_selection": "stochastic",
        "episode_start_offset_s": BASE_START_S,
        "action_seed": 128,
        "runtime_seed": 128,
        "sigma": EXPECTED_SIGMA,
    },
)
DEVELOPMENT_CASE_IDS = tuple(case["case_id"] for case in DEVELOPMENT_CASES)
DISCRIMINATOR_CASE_ID = DEVELOPMENT_CASE_IDS[0]

PURE_POLICY_COUNTER_FIELDS = (
    "teacher_query_count",
    "served_action_teacher_dependency_count",
    "mean_blend_count",
    "safety_intervention_count",
    "safety_latch_activation_count",
    "safety_latch_release_count",
)
REQUIRED_STEP_CHECKS = (
    "finite",
    "action_unclipped",
    "binary_active_routing_exact",
    "ten_control_windows",
    "ten_sensor_samples",
    "no_timeout",
    "no_unaccepted_so",
    "no_sea_fallback",
    "no_hard_invalid",
)
FORBIDDEN_TEACHER_FIELDS = frozenset(
    {
        "teacher_mean",
        "teacher_action",
        "blended_mean",
        "requested_alpha",
        "effective_alpha",
        "safety_latch_active",
    }
)
ZERO_INVALID_SUMMARY_FIELDS = (
    "action_clipped_values",
    "fallback_count",
    "timeout_count",
    "safety_stop_count",
    "sea_plugin_fallback_count",
    "so_solver_unaccepted_count",
    "hard_invalid_count",
    "invalid_event_count",
    "nonfinite_count",
    "routing_failure_count",
    "step_contract_failure_count",
    "binary_event_failure_count",
)

R10_EXECUTION_LOCK = PurePosixPath(
    "Trajectory Generator/baseline_MLP/validation/v12r10/"
    "h0_v12r10_recovery_execution_lock.json"
)
R10_EXECUTION_LOCK_SHA256 = (
    "e6601a67d251f19586f25f834df09957450a78fac18ace5d620902314888fb82"
)
R10_SOURCE_CLOSURE_COUNT = 94

OWN_PRODUCTION_SOURCES = (
    PurePosixPath(
        "Trajectory Generator/baseline_MLP/validation/v12r13/"
        "h0_v12r13_physical_development.py"
    ),
    PurePosixPath(
        "Trajectory Generator/baseline_MLP/validation/v12r13/"
        "run_h0_v12r13_physical_development.py"
    ),
)
GOVERNANCE_SOURCE_PATHS = (
    PurePosixPath(
        "Trajectory Generator/baseline_MLP/validation/v12r13/"
        "test_h0_v12r13_physical_development.py"
    ),
)
LINEAGE_SOURCE_PATHS = (
    PurePosixPath(
        "Trajectory Generator/baseline_MLP/validation/v12r13/"
        "h0_v12r13_masked_teacher_fitter.py"
    ),
    PurePosixPath(
        "Trajectory Generator/baseline_MLP/validation/v12r13/"
        "run_h0_v12r13_masked_teacher_fit.py"
    ),
    PurePosixPath(
        "Trajectory Generator/baseline_MLP/validation/v12r13/h0_v12r13_tape_dagger.py"
    ),
    PurePosixPath(
        "Trajectory Generator/baseline_MLP/validation/v12r13/"
        "run_h0_v12r13_tape_dagger.py"
    ),
)
REQUIRED_TRANSITIVE_RUNTIME_SOURCES = (
    *OWN_PRODUCTION_SOURCES,
    PurePosixPath(
        "Trajectory Generator/baseline_MLP/validation/v12r6/"
        "h0_v12r6_physical_development.py"
    ),
    PurePosixPath(
        "Trajectory Generator/baseline_MLP/validation/v12r3/"
        "run_h0_primary_split_v12r3_autonomy_recovery.py"
    ),
    PurePosixPath("validation/run_h0_primary_split_v9_causal_teacher.py"),
    PurePosixPath("validation/h0_forensic_rollout.py"),
    PurePosixPath("Trajectory Generator/baseline_MLP/env_factory.py"),
    PurePosixPath("Trajectory Generator/baseline_MLP/rollout_eval.py"),
    PurePosixPath("Trajectory Generator/binary_phase_adapter_v26.py"),
    PurePosixPath("Trajectory Generator/binary_phase_fsm_v26.py"),
)


class V12R13PhysicalDevelopmentError(RuntimeError):
    """Raised when P3 cannot preserve its one-shot physical contract."""


@dataclass(frozen=True)
class DevelopmentPaths:
    """All mutable P3 paths, injectable so tests never touch production."""

    artifact_root: Path
    revision_root: Path
    run_root: Path
    p2_output: Path
    p2_candidate: Path
    protocol_freeze: Path
    execution_lock: Path
    p3_root: Path

    @classmethod
    def production(cls) -> "DevelopmentPaths":
        run_root = REVISION_ROOT / "h0_v12r13_run_20260816"
        p2_output = run_root / "p2_fit"
        p3_root = run_root / "p3_development"
        return cls(
            artifact_root=REPO_ROOT,
            revision_root=REVISION_ROOT,
            run_root=run_root,
            p2_output=p2_output,
            p2_candidate=(p2_output / "rl_module_p2_masked_candidate_exposed"),
            protocol_freeze=(REVISION_ROOT / "h0_v12r13_p3_protocol_freeze.json"),
            execution_lock=(REVISION_ROOT / "h0_v12r13_p3_execution_lock.json"),
            p3_root=p3_root,
        )

    @property
    def p2_receipt(self) -> Path:
        return self.p2_output / "receipt.json"

    @property
    def p2_gate(self) -> Path:
        return self.p2_output / "gate.json"

    @property
    def p2_summary(self) -> Path:
        return self.p2_output / "summary.json"

    @property
    def pipeline_claim(self) -> Path:
        return self.p3_root / "pipeline_claim.json"

    @property
    def candidate_freeze(self) -> Path:
        return self.p3_root / "candidate_freeze.json"

    @property
    def cases_root(self) -> Path:
        return self.p3_root / "cases"

    @property
    def aggregate_summary(self) -> Path:
        return self.p3_root / "aggregate_summary.json"

    @property
    def aggregate_gate(self) -> Path:
        return self.p3_root / "aggregate_gate.json"

    @property
    def final_receipt(self) -> Path:
        return self.p3_root / "final_receipt.json"

    @property
    def pipeline_ledger(self) -> Path:
        return self.p3_root / "pipeline_ledger.json"

    def case_root(self, case_id: str) -> Path:
        canonical_case(case_id)
        return self.cases_root / case_id


PRODUCTION_PATHS = DevelopmentPaths.production()


def _finite_number(value: Any) -> bool:
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(float(value))
    )


def _exact_int(value: Any, expected: int) -> bool:
    return type(value) is int and value == expected


def canonical_json_bytes(value: Any) -> bytes:
    """Serialize strict deterministic JSON and reject NaN/Infinity."""

    try:
        encoded = json.dumps(
            value,
            sort_keys=True,
            separators=(",", ":"),
            ensure_ascii=False,
            allow_nan=False,
        )
    except (TypeError, ValueError) as exc:
        raise V12R13PhysicalDevelopmentError("value is not strict JSON") from exc
    return encoded.encode("utf-8")


def strict_json_load(path: Path) -> Any:
    try:
        return json.loads(
            path.read_text(encoding="utf-8"),
            parse_constant=lambda token: (_ for _ in ()).throw(
                ValueError(f"non-finite JSON token: {token}")
            ),
        )
    except (OSError, UnicodeError, json.JSONDecodeError, ValueError) as exc:
        raise V12R13PhysicalDevelopmentError(f"invalid strict JSON: {path}") from exc


def write_json_exclusive(path: Path, value: Any) -> dict[str, Any]:
    """Publish one canonical JSON file with O_EXCL/no-clobber semantics."""

    payload = canonical_json_bytes(value) + b"\n"
    path.parent.mkdir(parents=True, exist_ok=True)
    flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL
    try:
        descriptor = os.open(path, flags, 0o600)
    except OSError as exc:
        raise V12R13PhysicalDevelopmentError(
            f"artifact exists or cannot be exclusively created: {path}"
        ) from exc
    try:
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
    except BaseException:
        raise
    persisted = strict_json_load(path)
    if canonical_json_bytes(persisted) != canonical_json_bytes(value):
        raise V12R13PhysicalDevelopmentError(
            f"exclusive JSON publication changed payload: {path}"
        )
    return artifact_record(path, artifact_root=_boundary_for(path))


def _boundary_for(path: Path) -> Path:
    resolved = path.resolve()
    production = REPO_ROOT.resolve()
    try:
        resolved.relative_to(production)
    except ValueError:
        # Test fixtures live below a temporary artifact root.  Artifact records
        # used by production always pass their boundary explicitly.
        return resolved.parent
    return production


def _inside(path: Path, root: Path, *, label: str) -> Path:
    resolved = path.expanduser().resolve()
    boundary = root.expanduser().resolve()
    try:
        resolved.relative_to(boundary)
    except ValueError as exc:
        raise V12R13PhysicalDevelopmentError(
            f"{label} escaped artifact root: {resolved}"
        ) from exc
    return resolved


def _is_link_or_reparse(path: Path) -> bool:
    try:
        status = path.lstat()
    except OSError:
        return False
    attributes = int(getattr(status, "st_file_attributes", 0) or 0)
    reparse = int(getattr(stat, "FILE_ATTRIBUTE_REPARSE_POINT", 0x400))
    return stat.S_ISLNK(status.st_mode) or bool(attributes & reparse)


def _safe_regular_file(path: Path) -> bool:
    try:
        mode = path.lstat().st_mode
    except OSError:
        return False
    return stat.S_ISREG(mode) and not _is_link_or_reparse(path)


def sha256_file(path: Path) -> str:
    if not _safe_regular_file(path):
        raise V12R13PhysicalDevelopmentError(f"unsafe or missing file: {path}")
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def artifact_record(path: Path, *, artifact_root: Path) -> dict[str, Any]:
    target = _inside(path, artifact_root, label="artifact")
    if not _safe_regular_file(target):
        raise V12R13PhysicalDevelopmentError(f"artifact is not regular: {target}")
    return {
        "path": target.relative_to(artifact_root.resolve()).as_posix(),
        "sha256": sha256_file(target),
        "size_bytes": target.stat().st_size,
    }


def tree_record(path: Path, *, artifact_root: Path) -> dict[str, Any]:
    root = _inside(path, artifact_root, label="tree")
    if not root.is_dir() or _is_link_or_reparse(root):
        raise V12R13PhysicalDevelopmentError(f"unsafe or missing tree: {root}")
    entries = list(root.rglob("*"))
    if any(_is_link_or_reparse(entry) for entry in entries):
        raise V12R13PhysicalDevelopmentError(
            f"tree contains a symlink/junction: {root}"
        )
    files = sorted(
        (entry for entry in entries if entry.is_file()),
        key=lambda item: item.relative_to(root).as_posix(),
    )
    if not files:
        raise V12R13PhysicalDevelopmentError(f"tree is empty: {root}")
    digest = hashlib.sha256()
    records: list[dict[str, Any]] = []
    for item in files:
        relative = item.relative_to(root).as_posix()
        sha256 = sha256_file(item)
        size_bytes = item.stat().st_size
        record = {"path": relative, "sha256": sha256, "size_bytes": size_bytes}
        records.append(record)
        digest.update(relative.encode("utf-8"))
        digest.update(b"\0")
        digest.update(sha256.encode("ascii"))
        digest.update(b"\0")
        digest.update(str(size_bytes).encode("ascii"))
        digest.update(b"\n")
    return {
        "path": root.relative_to(artifact_root.resolve()).as_posix(),
        "tree_sha256": digest.hexdigest(),
        "file_count": len(records),
        "files": records,
    }


def validate_artifact_record(record: Mapping[str, Any], *, artifact_root: Path) -> None:
    if set(record) != {"path", "sha256", "size_bytes"}:
        raise V12R13PhysicalDevelopmentError("artifact record schema drifted")
    raw = record.get("path")
    if not isinstance(raw, str):
        raise V12R13PhysicalDevelopmentError("artifact record path is malformed")
    pure = PurePosixPath(raw)
    if pure.is_absolute() or ".." in pure.parts or pure.as_posix() != raw:
        raise V12R13PhysicalDevelopmentError("artifact record path is non-canonical")
    observed = artifact_record(
        artifact_root.joinpath(*pure.parts), artifact_root=artifact_root
    )
    if observed != dict(record):
        raise V12R13PhysicalDevelopmentError(f"artifact record drifted: {raw}")


def validate_tree_record(record: Mapping[str, Any], *, artifact_root: Path) -> None:
    if set(record) != {"path", "tree_sha256", "file_count", "files"}:
        raise V12R13PhysicalDevelopmentError("tree record schema drifted")
    raw = record.get("path")
    if not isinstance(raw, str):
        raise V12R13PhysicalDevelopmentError("tree record path is malformed")
    pure = PurePosixPath(raw)
    if pure.is_absolute() or ".." in pure.parts or pure.as_posix() != raw:
        raise V12R13PhysicalDevelopmentError("tree record path is non-canonical")
    observed = tree_record(
        artifact_root.joinpath(*pure.parts), artifact_root=artifact_root
    )
    if observed != dict(record):
        raise V12R13PhysicalDevelopmentError(f"tree record drifted: {raw}")


def canonical_case(case_id: str) -> dict[str, Any]:
    matches = [case for case in DEVELOPMENT_CASES if case["case_id"] == case_id]
    if len(matches) != 1:
        raise V12R13PhysicalDevelopmentError(f"unknown P3 case: {case_id!r}")
    return copy.deepcopy(matches[0])


def protocol_design() -> dict[str, Any]:
    """Return the immutable scientific design; this function performs no I/O."""

    return {
        "schema_version": SCHEMA_VERSION,
        "revision": REVISION,
        "protocol_id": PROTOCOL_ID,
        "development_id": DEVELOPMENT_ID,
        "topology_id": TOPOLOGY_ID,
        "p2_candidate_relative_path": (
            "Trajectory Generator/baseline_MLP/validation/v12r13/"
            "h0_v12r13_run_20260816/p2_fit/"
            "rl_module_p2_masked_candidate_exposed"
        ),
        "p0_eligible": False,
        "case_order": list(DEVELOPMENT_CASE_IDS),
        "first_and_only_discriminator": DISCRIMINATOR_CASE_ID,
        "case_contracts": copy.deepcopy(list(DEVELOPMENT_CASES)),
        "gates": {
            "steps": EXPECTED_STEPS,
            "control_windows": EXPECTED_CONTROL_WINDOWS,
            "raw_sensor_samples": EXPECTED_RAW_SENSOR_SAMPLES,
            "policy_dt_s": EXPECTED_POLICY_DT_S,
            "minimum_valid_cycles": MINIMUM_VALID_CYCLES,
            "penetration_strictly_less_than_m": PENETRATION_LIMIT_M,
            "event_contract_id": EVENT_CONTRACT_ID,
            "target_contract_id": TARGET_CONTRACT_ID,
            "event_source": EVENT_SOURCE,
            "morphology_weight": MORPHOLOGY_WEIGHT,
            "zero_invalid_summary_fields": list(ZERO_INVALID_SUMMARY_FIELDS),
        },
        "runtime": {
            "physical_runtime": (
                "Trajectory Generator/baseline_MLP/validation/v12r6/"
                "h0_v12r6_physical_development.py"
            ),
            "actor_features": EXPECTED_ACTOR_FEATURES,
            "action_dim": EXPECTED_ACTION_DIM,
            "teacher": False,
            "blend": False,
            "latch": False,
            "actor_queries_per_case": EXPECTED_STEPS,
            "actor_updates": 0,
            "critic_updates": 0,
            "ppo_updates": 0,
        },
        "execution": {
            "one_shot": True,
            "retry_authorized": False,
            "resume_authorized": False,
            "rescue_authorized": False,
            "no_clobber": True,
            "q3_opened": False,
            "checkpoint_zero_created": False,
            "positive_morphology_enabled": False,
        },
    }


def protocol_design_sha256() -> str:
    return hashlib.sha256(canonical_json_bytes(protocol_design())).hexdigest()


def describe_protocol(paths: DevelopmentPaths = PRODUCTION_PATHS) -> dict[str, Any]:
    """Describe deferred prerequisites without probing the filesystem."""

    return {
        "passed": True,
        "status": "DESCRIBED_H0_V12R13_P3_PURE_PHYSICAL_DEVELOPMENT",
        "design": protocol_design(),
        "design_sha256": protocol_design_sha256(),
        "paths": {
            "p2_receipt": paths.p2_receipt.as_posix(),
            "p2_gate": paths.p2_gate.as_posix(),
            "p2_summary": paths.p2_summary.as_posix(),
            "p2_candidate": paths.p2_candidate.as_posix(),
            "protocol_freeze": paths.protocol_freeze.as_posix(),
            "execution_lock": paths.execution_lock.as_posix(),
            "p3_root": paths.p3_root.as_posix(),
        },
        "deferred_blocker": (
            "P2 receipt/gate/candidate and the post-P2 freeze/lock must exist "
            "before preflight can pass"
        ),
        "environment_calls": 0,
        "policy_queries": 0,
        "artifacts_written": 0,
    }


def _mapping(path: Path) -> dict[str, Any]:
    value = strict_json_load(path)
    if not isinstance(value, dict):
        raise V12R13PhysicalDevelopmentError(f"expected JSON mapping: {path}")
    return value


def verify_p2(paths: DevelopmentPaths = PRODUCTION_PATHS) -> dict[str, Any]:
    """Adapt only the public P2 receipt/gate/manifest fields, fail closed."""

    for path in (paths.p2_receipt, paths.p2_gate, paths.p2_summary):
        if not _safe_regular_file(path):
            raise V12R13PhysicalDevelopmentError(
                f"P2 prerequisite is unavailable (P0 cannot substitute): {path}"
            )
    receipt = _mapping(paths.p2_receipt)
    gate = _mapping(paths.p2_gate)
    summary = _mapping(paths.p2_summary)
    public_checks = {
        "passed": receipt.get("passed") is True
        and gate.get("passed") is True
        and summary.get("passed") is True,
        "protocol": receipt.get("protocol_id") == PROTOCOL_ID
        and gate.get("protocol_id") == PROTOCOL_ID
        and summary.get("protocol_id") == PROTOCOL_ID,
        "exact_status": receipt.get("status") == P2_RECEIPT_STATUS
        and gate.get("status") == P2_GATE_STATUS
        and summary.get("status") == P2_SUMMARY_STATUS,
        "explicit_p2_stage": receipt.get("fit_stage") == "p2"
        and gate.get("fit_stage") == "p2"
        and summary.get("fit_stage") == "p2",
        "eligible_for_p3": receipt.get("collector_only") is False
        and receipt.get("non_promotable") is False
        and receipt.get("candidate_promoted") is False
        and receipt.get("next_stage") == P2_NEXT_STAGE
        and summary.get("collector_only") is False
        and summary.get("non_promotable") is False
        and summary.get("candidate_promoted") is False
        and summary.get("next_stage") == P2_NEXT_STAGE,
        "offline_only": receipt.get("environment_steps") == 0
        and receipt.get("policy_rollouts") == 0
        and receipt.get("retry_authorized") is False
        and summary.get("environment_steps") == 0
        and summary.get("policy_rollouts") == 0,
    }
    if not all(public_checks.values()):
        raise V12R13PhysicalDevelopmentError(
            f"P2 public terminal contract failed: {public_checks}"
        )

    candidate = tree_record(paths.p2_candidate, artifact_root=paths.artifact_root)
    expected_candidate_path = (
        paths.p2_candidate.resolve()
        .relative_to(paths.artifact_root.resolve())
        .as_posix()
    )
    if (
        candidate.get("path") != expected_candidate_path
        or "p0" in expected_candidate_path
    ):
        raise V12R13PhysicalDevelopmentError("canonical P2 candidate path drifted")
    artifacts = receipt.get("artifacts")
    if not isinstance(artifacts, Mapping) or set(artifacts) != {
        "corpus",
        "candidate_module",
        "summary",
        "gate",
    }:
        raise V12R13PhysicalDevelopmentError("P2 receipt artifacts are malformed")
    for value, label in (
        (artifacts.get("candidate_module"), "receipt"),
        (summary.get("candidate_module"), "summary"),
    ):
        if not isinstance(value, Mapping) or dict(value) != candidate:
            raise V12R13PhysicalDevelopmentError(
                f"P2 candidate tree does not match {label}"
            )
    for key, path in (("gate", paths.p2_gate), ("summary", paths.p2_summary)):
        record = artifacts.get(key)
        expected = artifact_record(path, artifact_root=paths.artifact_root)
        if not isinstance(record, Mapping) or dict(record) != expected:
            raise V12R13PhysicalDevelopmentError(f"P2 {key} binding drifted")
    corpus_record = artifacts.get("corpus")
    if not isinstance(corpus_record, Mapping):
        raise V12R13PhysicalDevelopmentError("P2 corpus binding is malformed")
    validate_artifact_record(corpus_record, artifact_root=paths.artifact_root)

    actor_manifest_path = paths.p2_candidate / "actor_feature_manifest.json"
    build_manifest_path = paths.p2_candidate / "candidate_build_manifest.json"
    actor_manifest = _mapping(actor_manifest_path)
    build_manifest = _mapping(build_manifest_path)
    manifest_checks = {
        "p2_actor_manifest": actor_manifest.get("fit_stage") == "p2",
        "p2_build_manifest": build_manifest.get("fit_stage") == "p2",
        "standard_w256": actor_manifest.get("fcnet_hiddens") == [256, 256]
        and actor_manifest.get("actor_feature_count") == EXPECTED_ACTOR_FEATURES
        and actor_manifest.get("topology_id") == TOPOLOGY_ID
        and actor_manifest.get("standard_rlmodule") is True,
        "tanh": str(actor_manifest.get("fcnet_activation", "")).lower() == "tanh",
        "no_legacy_shadow": actor_manifest.get("legacy_shadow_runtime_dependency")
        is False,
        "build_protocol": build_manifest.get("protocol_id") == PROTOCOL_ID,
        "no_runtime_teacher": build_manifest.get("no_teacher_runtime_dependency")
        is True,
    }
    if not all(manifest_checks.values()):
        raise V12R13PhysicalDevelopmentError(
            f"P2 W256 runtime manifest failed: {manifest_checks}"
        )
    candidate_id = receipt.get("candidate_id")
    if not isinstance(candidate_id, str) or summary.get("candidate_id") != candidate_id:
        raise V12R13PhysicalDevelopmentError("P2 candidate identity drifted")
    return {
        "passed": True,
        "candidate_id": candidate_id,
        "candidate_module": candidate,
        "receipt": artifact_record(paths.p2_receipt, artifact_root=paths.artifact_root),
        "gate": artifact_record(paths.p2_gate, artifact_root=paths.artifact_root),
        "summary": artifact_record(paths.p2_summary, artifact_root=paths.artifact_root),
        "actor_feature_manifest": artifact_record(
            actor_manifest_path, artifact_root=paths.artifact_root
        ),
        "candidate_build_manifest": artifact_record(
            build_manifest_path, artifact_root=paths.artifact_root
        ),
        "manifest_checks": manifest_checks,
        "public_checks": public_checks,
        "p0_used": False,
    }


def _closure_digest(closure: Mapping[str, Any]) -> str:
    return hashlib.sha256(canonical_json_bytes(dict(closure))).hexdigest()


def verify_execution_lock(
    paths: DevelopmentPaths = PRODUCTION_PATHS,
    *,
    p2: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    """Verify every locked runtime/lineage source and the exact P2 tree."""

    bound_p2 = verify_p2(paths) if p2 is None else copy.deepcopy(dict(p2))
    freeze = _mapping(paths.protocol_freeze)
    lock = _mapping(paths.execution_lock)
    freeze_record = artifact_record(
        paths.protocol_freeze, artifact_root=paths.artifact_root
    )
    checks = {
        "freeze_status": freeze.get("status") == FREEZE_PASS_STATUS,
        "freeze_passed": freeze.get("passed") is True,
        "freeze_protocol": freeze.get("protocol_id") == PROTOCOL_ID,
        "freeze_design": freeze.get("design_sha256") == protocol_design_sha256(),
        "freeze_design_payload": freeze.get("design") == protocol_design(),
        "freeze_p2_receipt": freeze.get("p2_receipt") == bound_p2.get("receipt"),
        "freeze_p2_gate": freeze.get("p2_gate") == bound_p2.get("gate"),
        "freeze_p2_summary": freeze.get("p2_summary") == bound_p2.get("summary"),
        "freeze_p2_candidate": freeze.get("p2_candidate")
        == bound_p2.get("candidate_module"),
        "freeze_p0_forbidden": freeze.get("p0_eligible") is False,
        "lock_status": lock.get("status") == LOCK_PASS_STATUS,
        "lock_passed": lock.get("passed") is True,
        "lock_protocol": lock.get("protocol_id") == PROTOCOL_ID,
        "lock_design": lock.get("design_sha256") == protocol_design_sha256(),
        "freeze_bound": lock.get("protocol_freeze") == freeze_record,
        "p2_receipt_bound": lock.get("p2_receipt") == bound_p2.get("receipt"),
        "p2_gate_bound": lock.get("p2_gate") == bound_p2.get("gate"),
        "p2_summary_bound": lock.get("p2_summary") == bound_p2.get("summary"),
        "p2_candidate_bound": lock.get("p2_candidate")
        == bound_p2.get("candidate_module"),
        "p0_forbidden": lock.get("p0_eligible") is False,
        "one_shot": lock.get("retry_authorized") is False
        and lock.get("resume_authorized") is False,
    }
    if not all(checks.values()):
        raise V12R13PhysicalDevelopmentError(
            f"P3 freeze/execution-lock binding failed: {checks}"
        )
    closure = lock.get("production_source_closure")
    if not isinstance(closure, Mapping) or not closure:
        raise V12R13PhysicalDevelopmentError("execution source closure is malformed")
    required = {
        path.as_posix()
        for path in (
            *REQUIRED_TRANSITIVE_RUNTIME_SOURCES,
            *GOVERNANCE_SOURCE_PATHS,
            *LINEAGE_SOURCE_PATHS,
        )
    }
    if not required.issubset(closure):
        missing = sorted(required.difference(closure))
        raise V12R13PhysicalDevelopmentError(
            f"execution source closure lacks runtime dependencies: {missing}"
        )
    for relative, expected in sorted(closure.items()):
        if not isinstance(relative, str) or not isinstance(expected, Mapping):
            raise V12R13PhysicalDevelopmentError("source closure entry is malformed")
        pure = PurePosixPath(relative)
        if pure.is_absolute() or ".." in pure.parts or pure.as_posix() != relative:
            raise V12R13PhysicalDevelopmentError(
                f"source closure path is non-canonical: {relative!r}"
            )
        observed = artifact_record(
            paths.artifact_root.joinpath(*pure.parts),
            artifact_root=paths.artifact_root,
        )
        if observed != dict(expected):
            raise V12R13PhysicalDevelopmentError(f"locked source drifted: {relative}")
    expected_digest = _closure_digest(closure)
    if (
        lock.get("production_source_closure_sha256") != expected_digest
        or lock.get("production_source_file_count") != len(closure)
        or freeze.get("production_source_closure_sha256") != expected_digest
        or freeze.get("production_source_file_count") != len(closure)
    ):
        raise V12R13PhysicalDevelopmentError("source closure digest drifted")
    return {
        "passed": True,
        "freeze": freeze_record,
        "execution_lock": artifact_record(
            paths.execution_lock, artifact_root=paths.artifact_root
        ),
        "p2": bound_p2,
        "production_source_file_count": len(closure),
        "production_source_closure_sha256": expected_digest,
    }


def preflight(paths: DevelopmentPaths = PRODUCTION_PATHS) -> dict[str, Any]:
    """Read-only P3 preflight; raises with a concrete deferred blocker."""

    p2 = verify_p2(paths)
    locked = verify_execution_lock(paths, p2=p2)
    if os.path.lexists(paths.p3_root):
        raise V12R13PhysicalDevelopmentError(
            "P3 output is occupied; retry/resume/no-clobber contract forbids execution"
        )
    return {
        "passed": True,
        "status": "READY_H0_V12R13_P3_PURE_PHYSICAL_ONE_SHOT",
        "protocol_id": PROTOCOL_ID,
        "development_id": DEVELOPMENT_ID,
        "design_sha256": protocol_design_sha256(),
        "p2": p2,
        "locked_inputs": locked,
        "p3_root_absent": True,
        "first_case": DISCRIMINATOR_CASE_ID,
        "case_order": list(DEVELOPMENT_CASE_IDS),
        "environment_calls": 0,
        "policy_queries": 0,
        "artifacts_written": 0,
    }


def trace_audit(trace: Any, *, case_id: str) -> dict[str, Any]:
    """Recompute all action-path, detector, sensor, and purity evidence."""

    case = canonical_case(case_id)
    rows = (
        list(trace)
        if isinstance(trace, Sequence) and not isinstance(trace, (str, bytes))
        else []
    )
    counters = {name: 0 for name in PURE_POLICY_COUNTER_FIELDS}
    checks = {
        "row_count": len(rows) == EXPECTED_STEPS,
        "identity": len(rows) == EXPECTED_STEPS,
        "observation": len(rows) == EXPECTED_STEPS,
        "distribution": len(rows) == EXPECTED_STEPS,
        "action_path": len(rows) == EXPECTED_STEPS,
        "unclipped": len(rows) == EXPECTED_STEPS,
        "pure": len(rows) == EXPECTED_STEPS,
        "raw_sensors": len(rows) == EXPECTED_STEPS,
        "binary_v26": len(rows) == EXPECTED_STEPS,
        "step_checks": len(rows) == EXPECTED_STEPS,
        "zero_row_counters": len(rows) == EXPECTED_STEPS,
        "full_time_grid": len(rows) == EXPECTED_STEPS,
    }
    event_sources: set[str] = set()
    times: list[float] = []
    clipping_count = 0
    actor_query_count = 0
    for expected_step, row in enumerate(rows, start=1):
        if not isinstance(row, Mapping):
            for name in checks:
                if name != "row_count":
                    checks[name] = False
            continue
        checks["identity"] = checks["identity"] and (
            row.get("step") == expected_step
            and row.get("schema_version") == SCHEMA_VERSION
            and row.get("protocol_id") == PROTOCOL_ID
            and row.get("stage_id") == f"development__{case_id}"
            and row.get("case_id") == case_id
        )
        time_s = row.get("time_s")
        checks["full_time_grid"] = checks["full_time_grid"] and _finite_number(time_s)
        if _finite_number(time_s):
            times.append(float(time_s))
        observation = row.get("v26_observation")
        checks["observation"] = checks["observation"] and (
            isinstance(observation, list)
            and len(observation) == EXPECTED_ACTOR_FEATURES
            and all(_finite_number(value) for value in observation)
        )
        mean = row.get("candidate_mean")
        std = row.get("candidate_std")
        innovation = row.get("standard_normal")
        noise = row.get("single_noise")
        raw = row.get("raw_action")
        applied = row.get("applied_action")
        vectors = (mean, std, innovation, noise, raw, applied)
        vectors_ok = all(
            isinstance(vector, list)
            and len(vector) == EXPECTED_ACTION_DIM
            and all(_finite_number(value) for value in vector)
            for vector in vectors
        )
        checks["distribution"] = checks["distribution"] and vectors_ok
        checks["action_path"] = checks["action_path"] and vectors_ok
        checks["unclipped"] = checks["unclipped"] and vectors_ok
        if vectors_ok:
            actor_query_count += 1
            checks["distribution"] = checks["distribution"] and all(
                math.isclose(
                    float(std[index]), EXPECTED_SIGMA, rel_tol=0.0, abs_tol=1.0e-8
                )
                and math.isclose(
                    float(noise[index]),
                    float(std[index]) * float(innovation[index]),
                    rel_tol=0.0,
                    abs_tol=1.0e-7,
                )
                for index in range(EXPECTED_ACTION_DIM)
            )
            if case["action_selection"] == "deterministic":
                checks["distribution"] = checks["distribution"] and all(
                    float(innovation[index]) == 0.0 and float(noise[index]) == 0.0
                    for index in range(EXPECTED_ACTION_DIM)
                )
            checks["action_path"] = checks["action_path"] and all(
                math.isclose(
                    float(raw[index]),
                    float(mean[index]) + float(noise[index]),
                    rel_tol=0.0,
                    abs_tol=1.0e-7,
                )
                for index in range(EXPECTED_ACTION_DIM)
            )
            row_clipping = sum(
                float(raw[index]) != float(applied[index])
                for index in range(EXPECTED_ACTION_DIM)
            )
            clipping_count += row_clipping
            checks["unclipped"] = checks["unclipped"] and row_clipping == 0
        checks["pure"] = checks["pure"] and (
            row.get("teacher_enabled") is False
            and row.get("blending_enabled") is False
            and row.get("safety_latch_enabled") is False
            and FORBIDDEN_TEACHER_FIELDS.isdisjoint(row)
        )
        for name in PURE_POLICY_COUNTER_FIELDS:
            value = row.get(name)
            if type(value) is int:
                counters[name] += value
            checks["zero_row_counters"] = checks["zero_row_counters"] and value == 0
        journal = row.get("observer_raw_sensor_journal")
        samples = journal.get("samples") if isinstance(journal, Mapping) else None
        checks["raw_sensors"] = checks["raw_sensors"] and (
            row.get("raw_sensor_sample_count") == EXPECTED_RAW_SENSOR_SAMPLES_PER_STEP
            and isinstance(samples, list)
            and len(samples) == EXPECTED_RAW_SENSOR_SAMPLES_PER_STEP
        )
        phase = row.get("phase_fsm")
        source = phase.get("event_source") if isinstance(phase, Mapping) else None
        if isinstance(source, str):
            event_sources.add(source)
        checks["binary_v26"] = checks["binary_v26"] and source == EVENT_SOURCE
        row_checks = row.get("checks")
        checks["step_checks"] = checks["step_checks"] and (
            isinstance(row_checks, Mapping)
            and all(row_checks.get(name) is True for name in REQUIRED_STEP_CHECKS)
        )
    checks["zero_counters"] = all(value == 0 for value in counters.values())
    checks["actor_queries"] = actor_query_count == EXPECTED_STEPS
    checks["full_time_grid"] = checks["full_time_grid"] and len(times) == len(rows)
    if len(times) == EXPECTED_STEPS:
        checks["full_time_grid"] = checks["full_time_grid"] and all(
            math.isclose(
                times[index] - times[index - 1],
                EXPECTED_POLICY_DT_S,
                rel_tol=0.0,
                abs_tol=1.0e-6,
            )
            for index in range(1, len(times))
        )
    passed = all(checks.values())
    return {
        "passed": passed,
        "checks": checks,
        "row_count": len(rows),
        "actor_query_count": actor_query_count,
        "raw_sensor_sample_count": sum(
            int(row.get("raw_sensor_sample_count", 0))
            for row in rows
            if isinstance(row, Mapping)
            and type(row.get("raw_sensor_sample_count")) is int
        ),
        "action_clipped_values": clipping_count,
        "event_sources": sorted(event_sources),
        "counters": counters,
    }


def development_gate(
    summary: Mapping[str, Any], *, case_id: str, trace: Any
) -> dict[str, Any]:
    """Evaluate one case with strict physical and causal-integrity gates."""

    case = canonical_case(case_id)
    audit = trace_audit(trace, case_id=case_id)
    binary_gate = summary.get("binary_phase_event_gate")
    binary_gate_ok = (
        isinstance(binary_gate, Mapping)
        and binary_gate.get("passed") is True
        and _exact_int(binary_gate.get("sample_count"), EXPECTED_RAW_SENSOR_SAMPLES)
        and all(
            _exact_int(binary_gate.get(name), 0)
            for name in (
                "duplicate_event_count",
                "out_of_order_event_count",
                "left_non_v26_source_count",
                "fallback_count",
                "hard_invalid_count",
            )
        )
    )
    checks = {
        "identity": summary.get("schema_version") == SCHEMA_VERSION
        and summary.get("status") == COMPLETE_STATUS
        and summary.get("protocol_id") == PROTOCOL_ID
        and summary.get("case") == case,
        "p2_only": summary.get("p2_candidate_only") is True
        and summary.get("p0_used") is False,
        "pure_trace": audit.get("passed") is True
        and isinstance(summary.get("pure_policy_trace_audit"), Mapping)
        and summary["pure_policy_trace_audit"].get("passed") is True
        and summary["pure_policy_trace_audit"].get("row_count") == EXPECTED_STEPS,
        "full_duration": _exact_int(summary.get("steps"), EXPECTED_STEPS)
        and _exact_int(summary.get("control_window_count"), EXPECTED_CONTROL_WINDOWS)
        and _exact_int(
            summary.get("raw_sensor_sample_count"), EXPECTED_RAW_SENSOR_SAMPLES
        )
        and summary.get("end_reason") == "episode_time_limit"
        and summary.get("terminated") is False
        and summary.get("truncated") is True,
        "cycles": type(summary.get("phase_valid_cycle_count")) is int
        and summary["phase_valid_cycle_count"] >= MINIMUM_VALID_CYCLES,
        "penetration": _finite_number(summary.get("grf_penetration_max_m"))
        and float(summary["grf_penetration_max_m"]) < PENETRATION_LIMIT_M,
        "zero_invalids": all(
            _exact_int(summary.get(name), 0) for name in ZERO_INVALID_SUMMARY_FIELDS
        ),
        "sea_reserve": summary.get("sea_reserve_gate_passed") is True,
        "binary_v26": binary_gate_ok
        and summary.get("event_contract_id") == EVENT_CONTRACT_ID
        and summary.get("target_contract_id") == TARGET_CONTRACT_ID
        and summary.get("detector_or_fsm_modified") is False,
        "morphology_zero": _finite_number(summary.get("morphology_weight"))
        and float(summary["morphology_weight"]) == MORPHOLOGY_WEIGHT,
        "zero_runtime_dependencies": summary.get("teacher_enabled") is False
        and summary.get("blending_enabled") is False
        and summary.get("safety_latch_enabled") is False
        and all(
            _exact_int(summary.get(name), 0) for name in PURE_POLICY_COUNTER_FIELDS
        ),
        "zero_updates": _exact_int(summary.get("actor_updates"), 0)
        and _exact_int(summary.get("critic_updates"), 0)
        and _exact_int(summary.get("ppo_updates"), 0),
        "no_next_protocol": summary.get("q2_paths_opened") == []
        and summary.get("q3_paths_opened") == [],
    }
    passed = all(checks.values())
    return {
        "schema_version": SCHEMA_VERSION,
        "status": CASE_PASS_STATUS if passed else CASE_FAIL_STATUS,
        "passed": passed,
        "terminal": not passed,
        "protocol_id": PROTOCOL_ID,
        "development_id": DEVELOPMENT_ID,
        "case_id": case_id,
        "checks": checks,
        "trace_audit": audit,
        "limits": {
            "penetration_m_strict": PENETRATION_LIMIT_M,
            "minimum_valid_cycles": MINIMUM_VALID_CYCLES,
            "steps": EXPECTED_STEPS,
            "raw_sensor_samples": EXPECTED_RAW_SENSOR_SAMPLES,
        },
        "next_stage": (
            "NEXT_PREREGISTERED_P3_CASE"
            if passed and case_id != DEVELOPMENT_CASE_IDS[-1]
            else "FINALIZE_P3"
            if passed
            else "STOP_TERMINAL_NO_RETRY"
        ),
        "retry_authorized": False,
        "resume_authorized": False,
    }


def aggregate_gate(summary: Mapping[str, Any]) -> dict[str, Any]:
    bindings = summary.get("case_bindings")
    ordered = (
        isinstance(bindings, list)
        and len(bindings) == len(DEVELOPMENT_CASE_IDS)
        and all(
            isinstance(binding, Mapping)
            and binding.get("case_id") == case_id
            and binding.get("passed") is True
            for binding, case_id in zip(bindings, DEVELOPMENT_CASE_IDS, strict=True)
        )
    )
    checks = {
        "identity": summary.get("schema_version") == SCHEMA_VERSION
        and summary.get("protocol_id") == PROTOCOL_ID,
        "risk_first": isinstance(bindings, list)
        and len(bindings) > 0
        and isinstance(bindings[0], Mapping)
        and bindings[0].get("case_id") == DISCRIMINATOR_CASE_ID,
        "six_of_six": ordered,
        "fixed_p2_candidate": _exact_int(summary.get("candidate_tree_unique_count"), 1)
        and summary.get("p0_used") is False,
        "activity": _exact_int(summary.get("development_count"), 6)
        and _exact_int(summary.get("environment_reset_calls"), 6)
        and _exact_int(summary.get("environment_step_calls"), 3_000)
        and _exact_int(summary.get("raw_sensor_sample_count"), 30_000)
        and _exact_int(summary.get("actor_query_count"), 3_000)
        and all(
            _exact_int(summary.get(name), 0) for name in PURE_POLICY_COUNTER_FIELDS
        ),
        "no_updates": _exact_int(summary.get("actor_updates"), 0)
        and _exact_int(summary.get("critic_updates"), 0)
        and _exact_int(summary.get("ppo_updates"), 0),
        "no_retry": summary.get("retry_authorized") is False
        and summary.get("resume_authorized") is False
        and summary.get("rescue_authorized") is False,
        "next_protocol_closed": summary.get("q3_paths_opened") == []
        and summary.get("checkpoint_zero_created") is False
        and summary.get("positive_morphology_enabled") is False,
    }
    passed = all(checks.values())
    return {
        "schema_version": SCHEMA_VERSION,
        "status": AGGREGATE_PASS_STATUS if passed else PIPELINE_FAIL_STATUS,
        "passed": passed,
        "protocol_id": PROTOCOL_ID,
        "development_id": DEVELOPMENT_ID,
        "checks": checks,
        "next_stage": "RUN_SEPARATE_Q3_PROTOCOL" if passed else "STOP_TERMINAL",
    }


__all__ = [
    "AGGREGATE_PASS_STATUS",
    "CASE_FAIL_STATUS",
    "CASE_PASS_STATUS",
    "COMPLETE_STATUS",
    "DEVELOPMENT_CASE_IDS",
    "DEVELOPMENT_CASES",
    "DEVELOPMENT_ID",
    "DevelopmentPaths",
    "EARLY_COMPLETE_STATUS",
    "EVENT_CONTRACT_ID",
    "EVENT_SOURCE",
    "EXPECTED_RAW_SENSOR_SAMPLES",
    "EXPECTED_STEPS",
    "LOCK_PASS_STATUS",
    "MORPHOLOGY_WEIGHT",
    "PARTIAL_STATUS",
    "PENETRATION_LIMIT_M",
    "PIPELINE_FAIL_STATUS",
    "PIPELINE_PASS_STATUS",
    "PRODUCTION_PATHS",
    "PROTOCOL_ID",
    "REQUIRED_TRANSITIVE_RUNTIME_SOURCES",
    "SCHEMA_VERSION",
    "START_STATUS",
    "TARGET_CONTRACT_ID",
    "V12R13PhysicalDevelopmentError",
    "aggregate_gate",
    "artifact_record",
    "canonical_case",
    "canonical_json_bytes",
    "describe_protocol",
    "development_gate",
    "preflight",
    "protocol_design",
    "protocol_design_sha256",
    "sha256_file",
    "strict_json_load",
    "trace_audit",
    "tree_record",
    "verify_execution_lock",
    "verify_p2",
    "write_json_exclusive",
]
