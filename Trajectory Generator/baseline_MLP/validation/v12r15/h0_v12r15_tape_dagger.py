"""Fail-closed V12R15 P1 tape-reference candidate-exposed contract.

P1 never constructs a teacher model.  The only teacher signal is the frozen
``frozen_teacher_mean`` at the same case and one-based policy step in the six
passing V8R1P1/V26 traces.  A fixed P0 collector is queried once per step and
its mean is mixed with that tape mean at the three preregistered student
weights.  The exact frozen tape noise is applied once, after mean selection.

The student is removed from the served action whenever either of two causal
shields fires:

* its pre-action V26 observation is outside the global 3,000-row invariant-18
  nearest-neighbour RMS-z p99 support envelope; or
* the V10S penetration latch is active (15 mm enter, 10 mm swing-only exit),
  using only the preceding completed step.

The shield changes behavior only.  Every candidate-exposed V26 observation is
still paired with the same-step tape mean and persisted for the cumulative P2
refit.  This module is source-only: importing it cannot load a checkpoint,
construct an environment, write an artifact, or query a policy.
"""

from __future__ import annotations

import copy
import hashlib
import math
import os
import sys
from dataclasses import dataclass
from pathlib import Path, PurePosixPath
from typing import Any, Mapping

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
BASELINE_ROOT = REPO_ROOT / "Trajectory Generator" / "baseline_MLP"
REVISION_ROOT = Path(__file__).resolve().parent
for _root in (REPO_ROOT, REPO_ROOT / "validation", BASELINE_ROOT, REVISION_ROOT):
    if str(_root) not in sys.path:
        sys.path.insert(0, str(_root))

import h0_forensic_rollout as forensic  # noqa: E402
import h0_primary_split_v10s_blend as v10s_blend  # noqa: E402


class V12R15TapeDaggerError(RuntimeError):
    """Raised when P1 provenance, support, action, or gate closure drifts."""


SCHEMA_VERSION = 12151
REVISION = "2026-08-16"
PROTOCOL_ID = "AB06_H0_V12R15_V26_INVARIANT_SAFE_TEACHER"
STAGE_ID = "P1_CANDIDATE_EXPOSED_TAPE_REFERENCE"
BEHAVIOR_ID = "V26_P0_MEAN_TAPE_REFERENCE_SHIELDED_SINGLE_FROZEN_NOISE"
TAPE_ID = "V8R1P1_V26_SAFE_TAPE_6X500_FROZEN_MEAN"
SUPPORT_ID = "GLOBAL_3000_INVARIANT18_LOO_NN_RMS_Z_P99"
EVENT_CONTRACT_ID = "binary_point_v25+heel_qualified_fsm_v2"

EXPECTED_STEPS = 500
EXPECTED_CASES = 6
EXPECTED_ALPHAS = 3
EXPECTED_ROLLOUTS = EXPECTED_CASES * EXPECTED_ALPHAS
EXPECTED_P1_ROWS = EXPECTED_ROLLOUTS * EXPECTED_STEPS
EXPECTED_BASE_ROWS = EXPECTED_CASES * EXPECTED_STEPS
EXPECTED_P2_ROWS = EXPECTED_BASE_ROWS + EXPECTED_P1_ROWS
EXPECTED_ACTOR_FEATURES = 35
EXPECTED_FULL_FEATURES = 84
EXPECTED_ACTION_DIM = 2
EXPECTED_RAW_SENSOR_SAMPLES = 5_000
EXPECTED_CONTROL_WINDOWS = 5_000
EXPECTED_SIGMA = np.float32(0.005)
MORPHOLOGY_WEIGHT = 0.0
PENETRATION_LIMIT_M = 0.025
MINIMUM_VALID_CYCLES = 2
TRANSITION_RADIUS_STEPS = 2

MASKED_COLUMNS = (0, 1, *range(10, 25))
INVARIANT_COLUMNS = (*range(2, 10), *range(25, 35))
ROUND_ALPHAS = (0.25, 0.50, 0.75)
CASE_IDS = (
    "deterministic_offset_minus_0p20",
    "deterministic_offset_nominal",
    "deterministic_offset_plus_0p20",
    "stochastic_nominal_seed_126",
    "stochastic_nominal_seed_127",
    "stochastic_nominal_seed_128",
)
_CASE_ROWS = (
    (CASE_IDS[0], "deterministic", 1.756870983805102, None, 123),
    (CASE_IDS[1], "deterministic", 1.956870983805102, None, 123),
    (CASE_IDS[2], "deterministic", 2.156870983805102, None, 123),
    (CASE_IDS[3], "stochastic", 1.956870983805102, 126, 126),
    (CASE_IDS[4], "stochastic", 1.956870983805102, 127, 127),
    (CASE_IDS[5], "stochastic", 1.956870983805102, 128, 128),
)

RUN_ROOT = REVISION_ROOT / "h0_v12r15_run_20260816"
P0_FIT_ROOT = RUN_ROOT / "p0_fit"
P0_MODULE = P0_FIT_ROOT / "rl_module_p0_masked_safe_teacher"
P0_RECEIPT = P0_FIT_ROOT / "receipt.json"
P0_SUMMARY = P0_FIT_ROOT / "summary.json"
P0_GATE = P0_FIT_ROOT / "gate.json"
P1_ROOT = RUN_ROOT / "p1_candidate_exposed"
P1_PROTOCOL_FREEZE = REVISION_ROOT / "h0_v12r15_tape_dagger_protocol_freeze.json"
P1_EXECUTION_LOCK = REVISION_ROOT / "h0_v12r15_tape_dagger_execution_lock.json"
P1_CORPUS = P1_ROOT / "corpus_candidate_exposed.npz"
P1_CORPUS_MANIFEST = P1_ROOT / "corpus_manifest.json"
P1_RECEIPT = P1_ROOT / "receipt.json"
P1_LEDGER = P1_ROOT / "pipeline_ledger.json"

TAPE_ROOT = PurePosixPath(
    "validation/h0_primary_grf_split_adaptation_runs/"
    "2026-08-07_h0_primary_split_v8r1p1_v26_residual/teacher_replay"
)
TAPE_LEDGER = TAPE_ROOT / "execution_ledger.json"
TAPE_EXECUTION_LOCK = PurePosixPath(
    "validation/h0_primary_grf_split_v8r1p1_teacher_replay_execution_lock.json"
)
TAPE_PREFLIGHT = PurePosixPath(
    "validation/h0_primary_grf_split_v8r1p1_teacher_replay_preflight_receipt.json"
)
TAPE_COMPATIBILITY_PREFLIGHT = PurePosixPath(
    "validation/h0_primary_grf_split_v8r1p1_compatibility_preflight_receipt.json"
)
V12R10_EXECUTION_LOCK = PurePosixPath(
    "Trajectory Generator/baseline_MLP/validation/v12r10/"
    "h0_v12r10_recovery_execution_lock.json"
)

TAPE_LEDGER_RECORD = {
    "path": TAPE_LEDGER.as_posix(),
    "sha256": "f8a1807d750762227defd87e185b9fbc36487e5570e917398f87c54ba6b24433",
    "size_bytes": 7_488,
}
TAPE_EXECUTION_LOCK_RECORD = {
    "path": TAPE_EXECUTION_LOCK.as_posix(),
    "sha256": "2046f33c06d0aa052af8b7ef2089e33a0befb4bb53c75564853d6a29607df6c7",
    "size_bytes": 40_779,
}
TAPE_PREFLIGHT_RECORD = {
    "path": TAPE_PREFLIGHT.as_posix(),
    "sha256": "b98af127a6fd6f678c8b67e77207d636e3609c273ebd1777af1b63d452d4456e",
    "size_bytes": 49_927,
}
TAPE_COMPATIBILITY_PREFLIGHT_RECORD = {
    "path": TAPE_COMPATIBILITY_PREFLIGHT.as_posix(),
    "sha256": "99f0c84b7add23fa46d4d0182608f5334d235aefde87c2dadc40bd46d84c0d10",
    "size_bytes": 2_465,
}
V12R10_EXECUTION_LOCK_RECORD = {
    "path": V12R10_EXECUTION_LOCK.as_posix(),
    "sha256": "e6601a67d251f19586f25f834df09957450a78fac18ace5d620902314888fb82",
    "size_bytes": 44_027,
}
TAPE_TRACE_RECORDS = {
    CASE_IDS[0]: {
        "sha256": "a66b30b6656e188fb45f8142f50de5c66f04fc410645c58347b4b67db70b0ce7",
        "size_bytes": 13_086_935,
    },
    CASE_IDS[1]: {
        "sha256": "93bca26abd90ab7a930421c8b6d172d7998cd7e1aa9c62a126b78614f66820a8",
        "size_bytes": 13_036_427,
    },
    CASE_IDS[2]: {
        "sha256": "c3ca347011ce79f5c8d1d3235d1a1b2b595eb7fcfafdc1a74f60dc34e640887d",
        "size_bytes": 13_042_971,
    },
    CASE_IDS[3]: {
        "sha256": "f8f2a9f6a11d6448cca99c0c8e08c7c699c88da8c73febeff915ad81b225e348",
        "size_bytes": 13_036_138,
    },
    CASE_IDS[4]: {
        "sha256": "5fec1066a9d3427eff60464582e3c959a53b11fa8008facbe9978024b597338b",
        "size_bytes": 13_038_166,
    },
    CASE_IDS[5]: {
        "sha256": "26c89dff89af3c351ef32ebd81ba1c92ddfd7b88f362e3343b133ab57e8e51f3",
        "size_bytes": 13_036_350,
    },
}

EXPECTED_TAPE_ARRAY_HASHES = {
    "observations": "4d964dff170fb076a7768084563ca7cf47a6ae77294d475d862df525d59b3e60",
    "targets": "4cbfd46c0787c7f52f7d8f642de8998faced2dee86a2e6fd79d3b3a3d758b402",
    "raw_actions": "4c42b0cbd1d96c0531e4531e1199c82b308338aac48601a83ea48e659f8fcee6",
    "teacher_std": "e2e9cb8960dc26380f7ada42df7971f68ca2b09aaa090da7a3765ddf80118839",
    "case_ids": "bab5db356e03645dd1818329f18ff0e459f7921f6d2f5bdd991da761130113ee",
    "step_indices": "aeac92d327ded462799935323f78a9e4f245badb535dca1a5510ed89a98fe961",
    "runtime_time_s": "9b813fbed031e36b75f864828fcd2ff50246c287083e5d8c9af03da08ca844c9",
}

# Canonical support arithmetic uses float32 tape values promoted to float64,
# ``math.fsum`` in row order for population statistics and final RMS values,
# and a stable linear quantile.  Vectorized blocks only identify the nearest
# row; their distance is recomputed portably before it can affect calibration
# or runtime decisions.  Intermediate NumPy hashes are diagnostic because
# reduction kernels can differ across macOS ARM and Windows x86.  The locked
# raw reference hash and hexadecimal scalar constants are the portable gate.
SUPPORT_P99_THRESHOLD = 0.1937808123139821
SUPPORT_P99_HEX = "0x1.8cdcf45bd8d0ep-3"
SUPPORT_LOO_MAX = 0.9882838030060314
SUPPORT_LOO_MAX_HEX = "0x1.fa0055aa27a27p-1"
SUPPORT_CONSTANT_ATOL = 2.0e-15
SUPPORT_BLOCK_ROWS = 128
NEAREST_CANDIDATE_ATOL = 1.0e-12
EXPECTED_SUPPORT_RAW_HASH = (
    "d65365149c6e76134ed27bd9d8b6006d3b37b78fde22a4190b9a1530dd8d8511"
)

COLLECTION_STARTED_STATUS = "H0_V12R15_P1_CANDIDATE_EXPOSED_STARTED"
COLLECTION_COMPLETE_STATUS = "H0_V12R15_P1_CANDIDATE_EXPOSED_PERSISTED_UNGATED"
COLLECTION_PASS_STATUS = "PASS_H0_V12R15_P1_CANDIDATE_EXPOSED_COLLECTION"
COLLECTION_FAIL_STATUS = "FAIL_H0_V12R15_P1_CANDIDATE_EXPOSED_COLLECTION"
P1_RECEIPT_STATUS = "PASS_H0_V12R15_P1_CANDIDATE_EXPOSED_RECEIPT"
P1_LEDGER_STATUS = "PASS_H0_V12R15_P1_CANDIDATE_EXPOSED_PIPELINE"
P1_FAILURE_STATUS = "FAIL_H0_V12R15_P1_CANDIDATE_EXPOSED"
PROTOCOL_FREEZE_STATUS = "H0_V12R15_P1_CANDIDATE_EXPOSED_PROTOCOL_FROZEN"
EXECUTION_LOCK_STATUS = "H0_V12R15_P1_CANDIDATE_EXPOSED_EXECUTION_LOCKED"


@dataclass(frozen=True)
class TapeReferenceStep:
    case_id: str
    step: int
    observation: np.ndarray
    target_mean: np.ndarray
    raw_action: np.ndarray
    teacher_std: np.ndarray
    frozen_noise: np.ndarray
    runtime_time_s: float


@dataclass(frozen=True)
class LockedTapeCorpus:
    observations: np.ndarray
    targets: np.ndarray
    raw_actions: np.ndarray
    teacher_std: np.ndarray
    case_ids: np.ndarray
    step_indices: np.ndarray
    runtime_time_s: np.ndarray
    rows_by_case: Mapping[str, tuple[TapeReferenceStep, ...]]
    source_closure: Mapping[str, Any]
    array_hashes: Mapping[str, str]

    def reference(self, case_id: str, step: int) -> TapeReferenceStep:
        if case_id not in self.rows_by_case:
            raise V12R15TapeDaggerError(f"unknown tape case: {case_id!r}")
        if type(step) is not int or not 1 <= step <= EXPECTED_STEPS:
            raise V12R15TapeDaggerError(f"invalid one-based tape step: {step!r}")
        return self.rows_by_case[case_id][step - 1]


@dataclass(frozen=True)
class SupportQuery:
    distance_rms_z: float
    within_p99: bool
    nearest_index: int
    nearest_case_id: str
    nearest_step: int


@dataclass(frozen=True)
class SupportEnvelope:
    raw: np.ndarray
    mean: np.ndarray
    scale: np.ndarray
    standardized: np.ndarray
    loo_nearest: np.ndarray
    p99: float
    loo_max: float
    array_hashes: Mapping[str, str]
    case_ids: np.ndarray
    step_indices: np.ndarray

    def query(self, observation: Any) -> SupportQuery:
        actor = _float32_vector(
            observation, width=EXPECTED_ACTOR_FEATURES, label="V26 observation"
        )
        selected = np.ascontiguousarray(
            actor[np.asarray(INVARIANT_COLUMNS, dtype=np.int64)], dtype=np.float64
        )
        standardized = (selected - self.mean) / self.scale
        delta = self.standardized - standardized[None, :]
        distances = np.sqrt(
            np.mean(np.square(delta, dtype=np.float64), axis=1, dtype=np.float64)
        )
        nearest_index, distance = _canonical_nearest_rms(
            standardized,
            self.standardized,
            approximate_distances=distances,
        )
        if not math.isfinite(distance):
            raise V12R15TapeDaggerError("support distance is non-finite")
        return SupportQuery(
            distance_rms_z=distance,
            within_p99=support_within_p99(distance),
            nearest_index=nearest_index,
            nearest_case_id=str(self.case_ids[nearest_index]),
            nearest_step=int(self.step_indices[nearest_index]),
        )


@dataclass(frozen=True)
class TapeDaggerAction:
    raw_action: np.ndarray
    blended_mean: np.ndarray
    frozen_noise: np.ndarray
    requested_alpha: float
    effective_alpha: float
    support: SupportQuery
    latch_state: v10s_blend.SafetyLatchState
    latch_entered: bool
    latch_released: bool
    safety_intervened: bool
    support_intervened: bool
    recovery: bool
    fallback_reasons: tuple[str, ...]


def _resolve(relative: str | PurePosixPath | Path) -> Path:
    raw = Path(relative)
    path = raw if raw.is_absolute() else REPO_ROOT / raw
    absolute = Path(os.path.abspath(path))
    root = Path(os.path.abspath(REPO_ROOT))
    try:
        absolute.relative_to(root)
    except ValueError as exc:
        raise V12R15TapeDaggerError(f"path escapes repository: {relative}") from exc
    return absolute


def _record(path: str | PurePosixPath | Path) -> dict[str, Any]:
    return forensic.artifact_record(_resolve(path), artifact_root=REPO_ROOT)


def _strict_json(path: str | PurePosixPath | Path) -> Any:
    return forensic.strict_json_load(_resolve(path))


def _mapping_json(path: str | PurePosixPath | Path) -> dict[str, Any]:
    value = _strict_json(path)
    if not isinstance(value, Mapping):
        raise V12R15TapeDaggerError(f"expected JSON object: {path}")
    return dict(value)


def _strict_equal(left: Any, right: Any) -> bool:
    if type(left) is not type(right):
        return False
    if isinstance(left, Mapping):
        return set(left) == set(right) and all(
            _strict_equal(left[key], right[key]) for key in left
        )
    if isinstance(left, (list, tuple)):
        return len(left) == len(right) and all(
            _strict_equal(a, b) for a, b in zip(left, right, strict=True)
        )
    return bool(left == right)


def _assert_record(observed: Mapping[str, Any], expected: Mapping[str, Any]) -> None:
    if not _strict_equal(dict(observed), dict(expected)):
        raise V12R15TapeDaggerError(
            f"artifact record drifted: expected={dict(expected)}, observed={dict(observed)}"
        )


def _tree_record(path: str | PurePosixPath | Path) -> dict[str, Any]:
    root = _resolve(path)
    if not root.is_dir() or root.is_symlink():
        raise V12R15TapeDaggerError(f"unsafe or missing tree: {root}")
    entries = list(root.rglob("*"))
    if any(item.is_symlink() for item in entries):
        raise V12R15TapeDaggerError(f"empty or linked tree: {root}")
    files = sorted(
        (item for item in entries if item.is_file()),
        key=lambda item: item.relative_to(root).as_posix(),
    )
    if not files:
        raise V12R15TapeDaggerError(f"empty or linked tree: {root}")
    digest = hashlib.sha256()
    records: list[dict[str, Any]] = []
    for item in files:
        relative = item.relative_to(root).as_posix()
        sha256 = forensic.sha256_file(item)
        size_bytes = item.stat().st_size
        records.append({"path": relative, "sha256": sha256, "size_bytes": size_bytes})
        digest.update(relative.encode("utf-8"))
        digest.update(b"\0")
        digest.update(sha256.encode("ascii"))
        digest.update(b"\0")
        digest.update(str(size_bytes).encode("ascii"))
        digest.update(b"\n")
    return {
        "path": root.relative_to(REPO_ROOT).as_posix(),
        "tree_sha256": digest.hexdigest(),
        "file_count": len(records),
        "files": records,
    }


def array_sha256(value: Any) -> str:
    array = np.ascontiguousarray(np.asarray(value))
    digest = hashlib.sha256()
    digest.update(array.dtype.str.encode("ascii"))
    digest.update(repr(array.shape).encode("ascii"))
    digest.update(array.tobytes(order="C"))
    return digest.hexdigest()


def _portable_rms_distance(left: Any, right: Any) -> float:
    a = np.asarray(left, dtype=np.float64)
    b = np.asarray(right, dtype=np.float64)
    if a.shape != (len(INVARIANT_COLUMNS),) or b.shape != a.shape:
        raise V12R15TapeDaggerError("portable RMS vectors are malformed")
    return math.sqrt(
        math.fsum((float(x) - float(y)) ** 2 for x, y in zip(a, b, strict=True))
        / len(INVARIANT_COLUMNS)
    )


def support_within_p99(distance: Any) -> bool:
    if (
        isinstance(distance, bool)
        or not isinstance(distance, (int, float, np.integer, np.floating))
        or not math.isfinite(float(distance))
        or float(distance) < 0.0
    ):
        raise V12R15TapeDaggerError("support distance must be finite nonnegative")
    return float(distance) <= SUPPORT_P99_THRESHOLD


def _canonical_nearest_rms(
    query: Any,
    reference: Any,
    *,
    approximate_distances: Any,
    excluded_index: int | None = None,
) -> tuple[int, float]:
    """Resolve vectorized near-ties with portable RMS and index ordering."""

    query_array = np.asarray(query, dtype=np.float64)
    reference_array = np.asarray(reference, dtype=np.float64)
    approximate = np.asarray(approximate_distances, dtype=np.float64)
    if (
        query_array.shape != (len(INVARIANT_COLUMNS),)
        or reference_array.ndim != 2
        or reference_array.shape[1] != len(INVARIANT_COLUMNS)
        or approximate.shape != (reference_array.shape[0],)
        or not np.all(np.isfinite(query_array))
        or not np.all(np.isfinite(reference_array))
    ):
        raise V12R15TapeDaggerError("nearest-neighbour inputs are malformed")
    eligible = np.ones(reference_array.shape[0], dtype=np.bool_)
    if excluded_index is not None:
        if not 0 <= excluded_index < reference_array.shape[0]:
            raise V12R15TapeDaggerError("excluded nearest index is invalid")
        eligible[excluded_index] = False
    finite = np.isfinite(approximate) & eligible
    if not np.any(finite):
        raise V12R15TapeDaggerError("nearest-neighbour set is empty")
    approximate_minimum = float(np.min(approximate[finite]))
    candidate_indices = np.flatnonzero(
        finite & (approximate <= approximate_minimum + NEAREST_CANDIDATE_ATOL)
    )
    if candidate_indices.size == 0:  # pragma: no cover - guarded by finite min.
        raise V12R15TapeDaggerError("nearest-neighbour candidate set is empty")
    candidates = [
        (
            _portable_rms_distance(query_array, reference_array[int(index)]),
            int(index),
        )
        for index in candidate_indices
    ]
    distance, index = min(candidates, key=lambda pair: (pair[0], pair[1]))
    return index, distance


def _float32_vector(value: Any, *, width: int, label: str) -> np.ndarray:
    try:
        result = np.ascontiguousarray(np.asarray(value, dtype=np.float32))
    except (TypeError, ValueError, OverflowError) as exc:
        raise V12R15TapeDaggerError(f"{label} is not float32") from exc
    if result.shape != (width,) or not np.all(np.isfinite(result)):
        raise V12R15TapeDaggerError(f"{label} must be finite shape ({width},)")
    return result


def _bytes_equal(left: Any, right: Any) -> bool:
    a = np.ascontiguousarray(np.asarray(left))
    b = np.ascontiguousarray(np.asarray(right))
    return (
        a.dtype == b.dtype
        and a.shape == b.shape
        and a.tobytes(order="C") == b.tobytes(order="C")
    )


def canonical_case(case_id: str) -> dict[str, Any]:
    matches = [row for row in _CASE_ROWS if row[0] == case_id]
    if len(matches) != 1:
        raise V12R15TapeDaggerError(f"unknown canonical case: {case_id!r}")
    name, selection, offset, action_seed, runtime_seed = matches[0]
    return {
        "case_id": name,
        "action_selection": selection,
        "episode_start_offset_s": offset,
        "action_seed": action_seed,
        "runtime_seed": runtime_seed,
        "sigma": float(EXPECTED_SIGMA) if selection == "stochastic" else 0.0,
        "morphology_weight": MORPHOLOGY_WEIGHT,
    }


def alpha_tag(alpha: float) -> str:
    mapping = {0.25: "0p25", 0.50: "0p50", 0.75: "0p75"}
    if alpha not in mapping:
        raise V12R15TapeDaggerError(f"unregistered alpha: {alpha!r}")
    return mapping[alpha]


def trajectory_id(alpha: float, case_id: str) -> str:
    canonical_case(case_id)
    return f"alpha_{alpha_tag(alpha)}__{case_id}"


def collection_plan() -> tuple[dict[str, Any], ...]:
    plan: list[dict[str, Any]] = []
    for alpha_index, alpha in enumerate(ROUND_ALPHAS, start=1):
        for case_index, case_id in enumerate(CASE_IDS, start=1):
            ordinal = (alpha_index - 1) * len(CASE_IDS) + case_index
            plan.append(
                {
                    **canonical_case(case_id),
                    "ordinal": ordinal,
                    "alpha_index": alpha_index,
                    "requested_alpha": alpha,
                    "trajectory_id": trajectory_id(alpha, case_id),
                    "destination": (
                        P1_ROOT / "collections" / f"alpha_{alpha_tag(alpha)}" / case_id
                    )
                    .relative_to(REPO_ROOT)
                    .as_posix(),
                    "candidate_stage": "p0_fit",
                    "candidate_update_count_before": 0,
                    "candidate_update_count_after": 0,
                }
            )
    return tuple(plan)


def verify_runtime_source_closure() -> dict[str, Any]:
    """Verify the complete 94-file physical-runtime closure frozen by R10."""

    lock_record = _record(V12R10_EXECUTION_LOCK)
    _assert_record(lock_record, V12R10_EXECUTION_LOCK_RECORD)
    lock = _mapping_json(V12R10_EXECUTION_LOCK)
    closure = lock.get("production_source_closure")
    if not isinstance(closure, Mapping) or len(closure) != 94:
        raise V12R15TapeDaggerError("R10 transitive runtime closure drifted")
    required = {
        "Trajectory Generator/baseline_MLP/validation/v12r6/"
        "h0_v12r6_physical_development.py",
        "Trajectory Generator/baseline_MLP/validation/v12r3/"
        "run_h0_primary_split_v12r3_autonomy_recovery.py",
        "validation/h0_primary_split_v10s_blend.py",
        "validation/h0_forensic_rollout.py",
        "validation/run_h0_primary_split_v9_causal_teacher.py",
    }
    if not required.issubset(closure):
        raise V12R15TapeDaggerError("runtime closure lacks required P1 primitives")
    verified: dict[str, dict[str, Any]] = {}
    for relative, expected in sorted(closure.items()):
        if not isinstance(relative, str) or not isinstance(expected, Mapping):
            raise V12R15TapeDaggerError("runtime closure record is malformed")
        observed = _record(relative)
        _assert_record(observed, expected)
        verified[relative] = observed
    manifest_sha256 = hashlib.sha256(
        forensic.canonical_json_bytes(verified)
    ).hexdigest()
    return {
        "basis_execution_lock": lock_record,
        "file_count": len(verified),
        "manifest_sha256": manifest_sha256,
        "files": verified,
    }


def source_snapshot(*, candidate_module: Path = P0_MODULE) -> dict[str, Any]:
    """Record P1 code, frozen runtime closure, and the immutable P0 tree."""

    local_paths = {
        "contract": Path(__file__).resolve(),
        "runner": REVISION_ROOT / "run_h0_v12r15_tape_dagger.py",
    }
    local = {name: _record(path) for name, path in local_paths.items()}
    return {
        "local_sources": local,
        "runtime_transitive_closure": verify_runtime_source_closure(),
        "p0_candidate_tree": _tree_record(candidate_module),
    }


def verify_tape_source_closure() -> dict[str, Any]:
    """Verify content-addressed V8R1P1 ledger, receipts, and all 3,000 steps."""

    for expected in (
        TAPE_LEDGER_RECORD,
        TAPE_EXECUTION_LOCK_RECORD,
        TAPE_PREFLIGHT_RECORD,
        TAPE_COMPATIBILITY_PREFLIGHT_RECORD,
    ):
        _assert_record(_record(str(expected["path"])), expected)
    ledger = _mapping_json(TAPE_LEDGER)
    ledger_checks = {
        "passed": ledger.get("passed") is True,
        "status": ledger.get("status")
        == "PASS_H0_V8R1P1_V26_TEACHER_REPLAY_DEVELOPMENT",
        "protocol": ledger.get("protocol_id")
        == "AB06_H0_PRIMARY_SPLIT_V8R1P1_V26_RESIDUAL_DAGGER",
        "case_order": ledger.get("expected_cases") == list(CASE_IDS)
        and ledger.get("completed_cases") == list(CASE_IDS),
        "zero_updates": ledger.get("actor_updates") == 0
        and ledger.get("critic_updates") == 0
        and ledger.get("ppo_updates") == 0,
        "no_candidate": ledger.get("candidate_created") is False,
        "protected_closed": ledger.get("protected_trials_opened") == [],
        "reserve_closed": ledger.get("reserve_trials_opened") == [],
        "execution_lock": ledger.get("execution_lock") == TAPE_EXECUTION_LOCK_RECORD,
    }
    if not all(ledger_checks.values()):
        failed = sorted(name for name, value in ledger_checks.items() if not value)
        raise V12R15TapeDaggerError(f"V8R1P1 ledger closure failed: {failed}")
    completed = ledger.get("completed_receipts")
    if not isinstance(completed, list) or len(completed) != EXPECTED_CASES:
        raise V12R15TapeDaggerError("V8R1P1 receipt index drifted")
    receipts: dict[str, dict[str, Any]] = {}
    trace_records: dict[str, dict[str, Any]] = {}
    verified_steps = 0
    for case_id, binding in zip(CASE_IDS, completed, strict=True):
        if not isinstance(binding, Mapping) or binding.get("case_id") != case_id:
            raise V12R15TapeDaggerError("V8R1P1 receipt order drifted")
        receipt_record = binding.get("receipt")
        canonical_receipt = TAPE_ROOT / case_id / "receipt.json"
        if (
            not isinstance(receipt_record, Mapping)
            or receipt_record.get("path") != canonical_receipt.as_posix()
        ):
            raise V12R15TapeDaggerError(f"receipt binding drifted: {case_id}")
        _assert_record(_record(canonical_receipt), receipt_record)
        receipt = _mapping_json(canonical_receipt)
        artifacts = receipt.get("artifacts")
        if (
            receipt.get("passed") is not True
            or receipt.get("status") != "PASS_H0_V8R1P1_V26_TEACHER_REPLAY"
            or receipt.get("case_id") != case_id
            or not isinstance(artifacts, Mapping)
            or not isinstance(artifacts.get("steps"), list)
            or len(artifacts["steps"]) != EXPECTED_STEPS
        ):
            raise V12R15TapeDaggerError(f"receipt content drifted: {case_id}")
        trace_path = TAPE_ROOT / case_id / "trace.json"
        expected_trace = {
            "path": trace_path.as_posix(),
            **TAPE_TRACE_RECORDS[case_id],
        }
        _assert_record(_record(trace_path), expected_trace)
        _assert_record(artifacts.get("trace", {}), expected_trace)
        gate = _mapping_json(TAPE_ROOT / case_id / "gate.json")
        if gate.get("passed") is not True:
            raise V12R15TapeDaggerError(f"source tape gate is not PASS: {case_id}")
        for index, step_record in enumerate(artifacts["steps"], start=1):
            expected_path = TAPE_ROOT / case_id / "steps" / f"{index:06d}.json"
            if (
                not isinstance(step_record, Mapping)
                or step_record.get("path") != expected_path.as_posix()
            ):
                raise V12R15TapeDaggerError(
                    f"source step binding drifted: {case_id}/{index}"
                )
            _assert_record(_record(expected_path), step_record)
            verified_steps += 1
        receipts[case_id] = dict(receipt_record)
        trace_records[case_id] = expected_trace
    return {
        "ledger": copy.deepcopy(TAPE_LEDGER_RECORD),
        "execution_lock": copy.deepcopy(TAPE_EXECUTION_LOCK_RECORD),
        "preflight": copy.deepcopy(TAPE_PREFLIGHT_RECORD),
        "compatibility_preflight": copy.deepcopy(TAPE_COMPATIBILITY_PREFLIGHT_RECORD),
        "receipts": receipts,
        "traces": trace_records,
        "transitive_step_files_verified": verified_steps,
    }


def load_locked_tape_corpus() -> LockedTapeCorpus:
    """Load the exact six-case safe tape without constructing a teacher."""

    source_closure = verify_tape_source_closure()
    observations: list[np.ndarray] = []
    targets: list[np.ndarray] = []
    raw_actions: list[np.ndarray] = []
    teacher_stds: list[np.ndarray] = []
    case_values: list[str] = []
    steps: list[int] = []
    times: list[float] = []
    rows_by_case: dict[str, tuple[TapeReferenceStep, ...]] = {}
    for case_id in CASE_IDS:
        value = _strict_json(TAPE_ROOT / case_id / "trace.json")
        if not isinstance(value, list) or len(value) != EXPECTED_STEPS:
            raise V12R15TapeDaggerError(f"source trace is incomplete: {case_id}")
        case_rows: list[TapeReferenceStep] = []
        for index, row in enumerate(value, start=1):
            if not isinstance(row, Mapping):
                raise V12R15TapeDaggerError(f"source row malformed: {case_id}/{index}")
            observation = _float32_vector(
                row.get("v25_observation"),
                width=EXPECTED_ACTOR_FEATURES,
                label="source V26 observation",
            )
            target = _float32_vector(
                row.get("frozen_teacher_mean"),
                width=EXPECTED_ACTION_DIM,
                label="frozen tape mean",
            )
            queried = _float32_vector(
                row.get("queried_teacher_mean"),
                width=EXPECTED_ACTION_DIM,
                label="persisted queried mean",
            )
            raw = _float32_vector(
                row.get("frozen_raw_action"),
                width=EXPECTED_ACTION_DIM,
                label="frozen raw action",
            )
            std = _float32_vector(
                row.get("teacher_std"),
                width=EXPECTED_ACTION_DIM,
                label="frozen teacher std",
            )
            runtime_time = row.get("runtime_time_s")
            row_checks = row.get("checks")
            semantics = {
                "identity": row.get("case_id") == case_id and row.get("step") == index,
                "target_exact": _bytes_equal(target, queried),
                "sigma_exact": _bytes_equal(
                    std,
                    np.full(EXPECTED_ACTION_DIM, EXPECTED_SIGMA, dtype=np.float32),
                ),
                "time_finite": not isinstance(runtime_time, bool)
                and isinstance(runtime_time, (int, float))
                and math.isfinite(float(runtime_time)),
                "row_checks": isinstance(row_checks, Mapping)
                and bool(row_checks)
                and all(check is True for check in row_checks.values()),
                "binary_active": row.get("phase_fsm", {}).get("event_source")
                == "binary_active_v26",
                "event_contract": row.get("binary_phase_fsm", {}).get(
                    "event_contract_id"
                )
                == EVENT_CONTRACT_ID,
            }
            if not all(semantics.values()):
                failed = sorted(
                    name for name, passed in semantics.items() if not passed
                )
                raise V12R15TapeDaggerError(
                    f"source tape semantics failed {case_id}/{index}: {failed}"
                )
            frozen_noise = np.ascontiguousarray(
                np.subtract(raw, target, dtype=np.float32), dtype=np.float32
            )
            reconstructed = np.add(target, frozen_noise, dtype=np.float32)
            if not _bytes_equal(reconstructed, raw):
                raise V12R15TapeDaggerError(
                    f"frozen noise is not reversible: {case_id}/{index}"
                )
            step = TapeReferenceStep(
                case_id=case_id,
                step=index,
                observation=observation,
                target_mean=target,
                raw_action=raw,
                teacher_std=std,
                frozen_noise=frozen_noise,
                runtime_time_s=float(runtime_time),
            )
            case_rows.append(step)
            observations.append(observation)
            targets.append(target)
            raw_actions.append(raw)
            teacher_stds.append(std)
            case_values.append(case_id)
            steps.append(index)
            times.append(float(runtime_time))
        rows_by_case[case_id] = tuple(case_rows)
    arrays = {
        "observations": np.ascontiguousarray(observations, dtype=np.float32),
        "targets": np.ascontiguousarray(targets, dtype=np.float32),
        "raw_actions": np.ascontiguousarray(raw_actions, dtype=np.float32),
        "teacher_std": np.ascontiguousarray(teacher_stds, dtype=np.float32),
        "case_ids": np.ascontiguousarray(case_values, dtype="U40"),
        "step_indices": np.ascontiguousarray(steps, dtype=np.int64),
        "runtime_time_s": np.ascontiguousarray(times, dtype=np.float64),
    }
    expected_shapes = {
        "observations": (EXPECTED_BASE_ROWS, EXPECTED_ACTOR_FEATURES),
        "targets": (EXPECTED_BASE_ROWS, EXPECTED_ACTION_DIM),
        "raw_actions": (EXPECTED_BASE_ROWS, EXPECTED_ACTION_DIM),
        "teacher_std": (EXPECTED_BASE_ROWS, EXPECTED_ACTION_DIM),
        "case_ids": (EXPECTED_BASE_ROWS,),
        "step_indices": (EXPECTED_BASE_ROWS,),
        "runtime_time_s": (EXPECTED_BASE_ROWS,),
    }
    if any(arrays[name].shape != shape for name, shape in expected_shapes.items()):
        raise V12R15TapeDaggerError("source tape array shapes drifted")
    hashes = {name: array_sha256(value) for name, value in arrays.items()}
    if hashes != EXPECTED_TAPE_ARRAY_HASHES:
        raise V12R15TapeDaggerError(f"source tape arrays drifted: {hashes}")
    for value in arrays.values():
        value.flags.writeable = False
    return LockedTapeCorpus(
        observations=arrays["observations"],
        targets=arrays["targets"],
        raw_actions=arrays["raw_actions"],
        teacher_std=arrays["teacher_std"],
        case_ids=arrays["case_ids"],
        step_indices=arrays["step_indices"],
        runtime_time_s=arrays["runtime_time_s"],
        rows_by_case=rows_by_case,
        source_closure=source_closure,
        array_hashes=hashes,
    )


def build_support_envelope(corpus: LockedTapeCorpus) -> SupportEnvelope:
    """Recompute and lock the global 3,000-row invariant-18 LOO envelope."""

    if not isinstance(corpus, LockedTapeCorpus):
        raise V12R15TapeDaggerError("support calibration requires locked tape")
    columns = np.asarray(INVARIANT_COLUMNS, dtype=np.int64)
    raw = np.ascontiguousarray(corpus.observations[:, columns], dtype=np.float64)
    mean = np.asarray(
        [
            math.fsum(float(value) for value in raw[:, column]) / EXPECTED_BASE_ROWS
            for column in range(len(INVARIANT_COLUMNS))
        ],
        dtype=np.float64,
    )
    centered = np.subtract(raw, mean[None, :], dtype=np.float64)
    scale = np.asarray(
        [
            math.sqrt(
                math.fsum(float(value) ** 2 for value in centered[:, column])
                / EXPECTED_BASE_ROWS
            )
            for column in range(len(INVARIANT_COLUMNS))
        ],
        dtype=np.float64,
    )
    if (
        raw.shape != (EXPECTED_BASE_ROWS, len(INVARIANT_COLUMNS))
        or mean.shape != (len(INVARIANT_COLUMNS),)
        or scale.shape != (len(INVARIANT_COLUMNS),)
        or not np.all(np.isfinite(raw))
        or not np.all(np.isfinite(mean))
        or not np.all(np.isfinite(scale))
        or np.any(scale <= 0.0)
    ):
        raise V12R15TapeDaggerError("support normalization is malformed")
    standardized = np.ascontiguousarray(centered / scale[None, :], dtype=np.float64)
    nearest_indices = np.empty(EXPECTED_BASE_ROWS, dtype=np.int64)
    loo = np.empty(EXPECTED_BASE_ROWS, dtype=np.float64)
    for start in range(0, EXPECTED_BASE_ROWS, SUPPORT_BLOCK_ROWS):
        stop = min(start + SUPPORT_BLOCK_ROWS, EXPECTED_BASE_ROWS)
        delta = standardized[start:stop, None, :] - standardized[None, :, :]
        distance = np.sqrt(
            np.mean(np.square(delta, dtype=np.float64), axis=2, dtype=np.float64)
        )
        distance[np.arange(stop - start), np.arange(start, stop)] = np.inf
        for local, index in enumerate(range(start, stop)):
            nearest, portable_distance = _canonical_nearest_rms(
                standardized[index],
                standardized,
                approximate_distances=distance[local],
                excluded_index=index,
            )
            nearest_indices[index] = nearest
            loo[index] = portable_distance
    ordered = sorted(float(value) for value in loo)
    position = (len(ordered) - 1) * 0.99
    lower = math.floor(position)
    upper = math.ceil(position)
    fraction = position - lower
    p99 = ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction
    loo_max = float(np.max(loo))
    hashes = {
        "raw": array_sha256(raw),
        "mean": array_sha256(mean),
        "scale": array_sha256(scale),
        "standardized": array_sha256(standardized),
        "loo_nearest": array_sha256(loo),
    }
    if hashes["raw"] != EXPECTED_SUPPORT_RAW_HASH:
        raise V12R15TapeDaggerError(f"support source rows drifted: {hashes['raw']}")
    if float(SUPPORT_P99_THRESHOLD).hex() != SUPPORT_P99_HEX:
        raise V12R15TapeDaggerError("serialized support p99 hex contract drifted")
    if float(SUPPORT_LOO_MAX).hex() != SUPPORT_LOO_MAX_HEX:
        raise V12R15TapeDaggerError("serialized support max hex contract drifted")
    if not math.isclose(
        p99, SUPPORT_P99_THRESHOLD, rel_tol=0.0, abs_tol=SUPPORT_CONSTANT_ATOL
    ) or not math.isclose(
        loo_max, SUPPORT_LOO_MAX, rel_tol=0.0, abs_tol=SUPPORT_CONSTANT_ATOL
    ):
        raise V12R15TapeDaggerError(
            f"support constants drifted: p99={p99!r}, loo_max={loo_max!r}"
        )
    for value in (raw, mean, scale, standardized, loo):
        value.flags.writeable = False
    return SupportEnvelope(
        raw=raw,
        mean=mean,
        scale=scale,
        standardized=standardized,
        loo_nearest=loo,
        p99=SUPPORT_P99_THRESHOLD,
        loo_max=SUPPORT_LOO_MAX,
        array_hashes=hashes,
        case_ids=corpus.case_ids,
        step_indices=corpus.step_indices,
    )


def support_manifest(envelope: SupportEnvelope) -> dict[str, Any]:
    return {
        "support_id": SUPPORT_ID,
        "reference_scope": "all_six_v8r1p1_cases_3000_rows",
        "calibration_scope": "same_global_3000_rows_leave_one_out",
        "dimension": len(INVARIANT_COLUMNS),
        "columns": list(INVARIANT_COLUMNS),
        "normalization": "float32_tape_to_c_contiguous_float64_population_zscore",
        "distance": "sqrt(mean(square(query_z-reference_z)))",
        "nearest_neighbor": "global_3000_argmin_first_index_tie_break",
        "near_tie_resolution": (
            "all_vectorized_candidates_within_1e-12_recomputed_math_fsum_"
            "then_min_distance_index"
        ),
        "near_tie_candidate_atol": NEAREST_CANDIDATE_ATOL,
        "loo_algorithm": "128_row_float64_blocks_self_distance_inf",
        "quantile": "sorted_float64_linear_interpolation_p99",
        "threshold_rule": "fallback_iff_distance_strictly_greater_than_p99",
        "p99": envelope.p99,
        "p99_hex": SUPPORT_P99_HEX,
        "loo_max": envelope.loo_max,
        "loo_max_hex": SUPPORT_LOO_MAX_HEX,
        "constant_absolute_tolerance": SUPPORT_CONSTANT_ATOL,
        "portable_gate_hashes": {"raw": envelope.array_hashes["raw"]},
        "diagnostic_intermediate_hashes_not_cross_platform_gates": {
            name: value
            for name, value in envelope.array_hashes.items()
            if name != "raw"
        },
        "legacy_plus_0p20_only_p99_excluded": 0.20330878485396986,
    }


def select_tape_dagger_action(
    *,
    candidate_mean: Any,
    candidate_std: Any,
    observation: Any,
    reference: TapeReferenceStep,
    requested_alpha: float,
    latch_state: v10s_blend.SafetyLatchState,
    previous_penetration_m: float,
    active_v26_phase: str,
    support_envelope: SupportEnvelope,
) -> TapeDaggerAction:
    """Apply global support and causal latch shields, then one frozen noise."""

    if requested_alpha not in ROUND_ALPHAS:
        raise V12R15TapeDaggerError("requested alpha is not preregistered")
    candidate = _float32_vector(
        candidate_mean, width=EXPECTED_ACTION_DIM, label="candidate mean"
    )
    std = _float32_vector(
        candidate_std, width=EXPECTED_ACTION_DIM, label="candidate std"
    )
    if not _bytes_equal(std, reference.teacher_std):
        raise V12R15TapeDaggerError("P0 logstd drifted from frozen tape sigma")
    support = support_envelope.query(observation)
    try:
        transition = v10s_blend.advance_safety_latch(
            latch_state,
            previous_penetration_m=previous_penetration_m,
            active_v26_phase=active_v26_phase,
        )
    except v10s_blend.V10SBlendError as exc:
        raise V12R15TapeDaggerError("causal V10S latch input is malformed") from exc
    support_intervened = not support.within_p99
    safety_intervened = transition.state.active
    effective_alpha = (
        0.0 if support_intervened or safety_intervened else float(requested_alpha)
    )
    try:
        blended, observed_alpha = v10s_blend.blend_policy_means(
            candidate,
            reference.target_mean,
            requested_alpha=effective_alpha,
            latch_state=transition.state,
        )
        raw_action = v10s_blend.apply_single_noise(blended, reference.frozen_noise)
    except v10s_blend.V10SBlendError as exc:
        raise V12R15TapeDaggerError("V10S blend/noise semantics failed") from exc
    if observed_alpha != effective_alpha:
        raise V12R15TapeDaggerError("effective alpha drifted after latch selection")
    reasons: list[str] = []
    if support_intervened:
        reasons.append("support_p99_exit")
    if safety_intervened:
        reasons.append("causal_penetration_latch")
    recovery = bool(reasons)
    if recovery and effective_alpha != 0.0:
        raise V12R15TapeDaggerError("recovery row retained nonzero student weight")
    return TapeDaggerAction(
        raw_action=np.ascontiguousarray(raw_action, dtype=np.float32),
        blended_mean=np.ascontiguousarray(blended, dtype=np.float32),
        frozen_noise=np.ascontiguousarray(reference.frozen_noise, dtype=np.float32),
        requested_alpha=float(requested_alpha),
        effective_alpha=effective_alpha,
        support=support,
        latch_state=transition.state,
        latch_entered=transition.entered,
        latch_released=transition.released,
        safety_intervened=safety_intervened,
        support_intervened=support_intervened,
        recovery=recovery,
        fallback_reasons=tuple(reasons),
    )


def _is_physical_truncation_end_reason(end_reason: Any) -> bool:
    if end_reason == "grf_penetration":
        return True
    return isinstance(end_reason, str) and end_reason.startswith("joint_divergence_")


def collection_gate(
    summary: Mapping[str, Any], *, plan_row: Mapping[str, Any]
) -> dict[str, Any]:
    """Mandatory prefix integrity; the full horizon is recorded, not required.

    V12R13 lesson (57-micron terminal fail on alpha 0.50/seed 127): a shielded
    rollout that the physics terminates early stays VALID collection data —
    every persisted row still carries a genuine candidate-exposed observation
    with its frozen tape label — but it is never a physical PASS. Only prefix
    integrity, shield semantics, and zero anomalies can fail a collection.
    The V12R3 `recoverable_for_data_collection` pattern, made structural."""

    zero_fields = (
        "action_clipped_values",
        "fallback_count",
        "timeout_count",
        "sea_plugin_fallback_count",
        "so_solver_unaccepted_count",
        "hard_invalid_count",
        "invalid_event_count",
        "nonfinite_count",
        "routing_failure_count",
        "step_contract_failure_count",
        "binary_event_failure_count",
        "target_provenance_mismatch_count",
        "tape_time_alignment_mismatch_count",
        "candidate_query_mismatch_count",
        "support_decision_mismatch_count",
        "latch_rule_violation_count",
        "blend_mismatch_count",
        "noise_application_mismatch_count",
        "source_or_candidate_drift_count",
    )
    diagnostics = (
        "support_fallback_count",
        "safety_fallback_count",
        "recovery_row_count",
        "safety_latch_activation_count",
        "safety_latch_release_count",
    )
    steps = summary.get("steps")
    valid_steps = type(steps) is int and 1 <= steps <= EXPECTED_STEPS
    steps_int = steps if valid_steps else 0
    windows_per_step = EXPECTED_CONTROL_WINDOWS // EXPECTED_STEPS
    samples_per_step = EXPECTED_RAW_SENSOR_SAMPLES // EXPECTED_STEPS
    full_horizon = (
        valid_steps
        and steps_int == EXPECTED_STEPS
        and summary.get("end_reason") == "episode_time_limit"
        and summary.get("terminated") is False
        and summary.get("truncated") is True
    )
    physically_truncated = (
        valid_steps
        and steps_int < EXPECTED_STEPS
        and _is_physical_truncation_end_reason(summary.get("end_reason"))
        and summary.get("terminated") is True
        and summary.get("truncated") is False
    )
    penetration_value = summary.get("grf_penetration_max_m")
    penetration_finite = (
        isinstance(penetration_value, (int, float))
        and not isinstance(penetration_value, bool)
        and math.isfinite(float(penetration_value))
    )
    if full_horizon:
        penetration_semantics = (
            penetration_finite and float(penetration_value) < PENETRATION_LIMIT_M
        )
    elif physically_truncated and summary.get("end_reason") == "grf_penetration":
        # The terminal guard fired: the recorded maximum must attest it.
        penetration_semantics = (
            penetration_finite and float(penetration_value) >= PENETRATION_LIMIT_M
        )
    else:
        penetration_semantics = penetration_finite
    checks = {
        "schema": summary.get("schema_version") == SCHEMA_VERSION,
        "status": summary.get("status") == COLLECTION_COMPLETE_STATUS,
        "protocol": summary.get("protocol_id") == PROTOCOL_ID,
        "behavior": summary.get("behavior_id") == BEHAVIOR_ID,
        "identity": summary.get("trajectory_id") == plan_row.get("trajectory_id")
        and summary.get("case_id") == plan_row.get("case_id")
        and summary.get("requested_alpha") == plan_row.get("requested_alpha"),
        "horizon_semantics": (full_horizon or physically_truncated)
        and summary.get("sample_count") == steps_int
        and summary.get("persisted_label_count") == steps_int,
        "physical_cadence": valid_steps
        and summary.get("control_window_count") == steps_int * windows_per_step
        and summary.get("raw_sensor_sample_count") == steps_int * samples_per_step,
        "cycles_semantics": type(summary.get("phase_valid_cycle_count")) is int
        and summary["phase_valid_cycle_count"] >= 0
        and (
            summary["phase_valid_cycle_count"] >= MINIMUM_VALID_CYCLES
            if full_horizon
            else True
        ),
        "penetration_semantics": penetration_semantics,
        # Producer semantics (V12R14 lesson): safety_stop_count is literally
        # int(bool(terminated)) — 1 is the signature OF a physical truncation,
        # 0 the signature of a clean full horizon. Anything else is incoherent.
        "safety_stop_semantics": summary.get("safety_stop_count")
        == (1 if physically_truncated else 0),
        "zero_anomalies": all(
            type(summary.get(field)) is int and summary[field] == 0
            for field in zero_fields
        ),
        "diagnostics_well_formed": all(
            type(summary.get(field)) is int and summary[field] >= 0
            for field in diagnostics
        ),
        "v26_active": summary.get("binary_phase_fsm_mode") == "binary_active"
        and summary.get("event_contract_id") == EVENT_CONTRACT_ID,
        "morphology_zero": summary.get("morphology_weight") == MORPHOLOGY_WEIGHT,
        "candidate_once_per_step": summary.get("candidate_mean_query_count")
        == steps_int,
        "tape_lookup_once_per_step": summary.get("tape_reference_lookup_count")
        == steps_int
        and summary.get("tape_target_binding_count") == steps_int,
        "no_teacher_model_query": summary.get("teacher_model_query_count") == 0
        and summary.get("legacy_shadow_query_count") == 0,
        "support_every_step": summary.get("support_query_count") == steps_int
        and summary.get("support_id") == SUPPORT_ID
        and summary.get("support_p99") == SUPPORT_P99_THRESHOLD,
        "blend_then_one_frozen_noise": summary.get("mean_blend_count") == steps_int
        and summary.get("single_noise_application_count") == steps_int
        and summary.get("frozen_noise_lookup_count") == steps_int
        and summary.get("random_noise_draw_count") == 0,
        "causal_latch": summary.get("safety_latch_activation_m")
        == v10s_blend.SAFETY_LATCH_ACTIVATION_M
        and summary.get("safety_latch_release_m") == v10s_blend.SAFETY_LATCH_RELEASE_M
        and summary.get("safety_latch_release_phase")
        == v10s_blend.SAFETY_LATCH_RELEASE_PHASE
        and summary.get("safety_signal_lag_steps") == 1,
        "no_updates_between": summary.get("candidate_update_count_before") == 0
        and summary.get("candidate_update_count_after") == 0
        and summary.get("actor_updates") == 0
        and summary.get("critic_updates") == 0
        and summary.get("ppo_updates") == 0,
        "closed_trials": summary.get("protected_trials_opened") == []
        and summary.get("reserve_trials_opened") == [],
        "source_stable": summary.get("source_snapshot_before")
        == summary.get("source_snapshot_after"),
    }
    passed = all(checks.values())
    return {
        "schema_version": SCHEMA_VERSION,
        "status": COLLECTION_PASS_STATUS if passed else COLLECTION_FAIL_STATUS,
        "passed": passed,
        "protocol_id": PROTOCOL_ID,
        "trajectory_id": summary.get("trajectory_id"),
        "case_id": summary.get("case_id"),
        "requested_alpha": summary.get("requested_alpha"),
        "checks": checks,
        "steps": steps_int,
        "full_horizon": bool(full_horizon),
        "physically_truncated": bool(physically_truncated),
        "recoverable_for_data_collection": passed,
        "physical_full_horizon_pass": bool(passed and full_horizon),
        "physical_penetration_limit_m": PENETRATION_LIMIT_M,
        "physical_gate_relaxed": False,
        "shielded_collection_consumes_pure_trial": False,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
    }


def build_protocol_description() -> dict[str, Any]:
    plan = collection_plan()
    return {
        "schema_version": SCHEMA_VERSION,
        "protocol_id": PROTOCOL_ID,
        "stage_id": STAGE_ID,
        "revision": REVISION,
        "behavior_id": BEHAVIOR_ID,
        "tape_id": TAPE_ID,
        "support_id": SUPPORT_ID,
        "plan": [dict(row) for row in plan],
        "ordering": "alpha_major_then_canonical_case_ids_then_one_based_step",
        "rollout_count": len(plan),
        "rows_per_rollout": EXPECTED_STEPS,
        "p1_rows": EXPECTED_P1_ROWS,
        "base_rows": EXPECTED_BASE_ROWS,
        "expected_p2_rows": EXPECTED_P2_ROWS,
        "target_field": "frozen_teacher_mean",
        "teacher_model_queries": 0,
        "legacy_shadow_queries": 0,
        "candidate_updates_between_rollouts": 0,
        "noise_semantics": "same_case_step_frozen_noise_once_after_mean_selection",
        "support": {
            "reference_rows": EXPECTED_BASE_ROWS,
            "calibration_rows": EXPECTED_BASE_ROWS,
            "columns": list(INVARIANT_COLUMNS),
            "p99": SUPPORT_P99_THRESHOLD,
            "loo_max": SUPPORT_LOO_MAX,
            "fallback_rule": "distance_gt_p99",
        },
        "safety_latch": {
            "signal_lag_steps": 1,
            "activation_m": v10s_blend.SAFETY_LATCH_ACTIVATION_M,
            "release_m": v10s_blend.SAFETY_LATCH_RELEASE_M,
            "release_phase": v10s_blend.SAFETY_LATCH_RELEASE_PHASE,
        },
        "outputs": {
            "protocol_freeze": P1_PROTOCOL_FREEZE.relative_to(REPO_ROOT).as_posix(),
            "execution_lock": P1_EXECUTION_LOCK.relative_to(REPO_ROOT).as_posix(),
            "root": P1_ROOT.relative_to(REPO_ROOT).as_posix(),
            "corpus": P1_CORPUS.relative_to(REPO_ROOT).as_posix(),
            "manifest": P1_CORPUS_MANIFEST.relative_to(REPO_ROOT).as_posix(),
            "receipt": P1_RECEIPT.relative_to(REPO_ROOT).as_posix(),
            "ledger": P1_LEDGER.relative_to(REPO_ROOT).as_posix(),
        },
        "execution_mode": {
            "description_only": True,
            "checkpoint_loaded": False,
            "policy_queried": False,
            "environment_built": False,
            "environment_reset": False,
            "environment_step": False,
            "artifact_written": False,
        },
    }


__all__ = [
    "BEHAVIOR_ID",
    "CASE_IDS",
    "COLLECTION_COMPLETE_STATUS",
    "COLLECTION_FAIL_STATUS",
    "COLLECTION_PASS_STATUS",
    "COLLECTION_STARTED_STATUS",
    "EVENT_CONTRACT_ID",
    "EXECUTION_LOCK_STATUS",
    "EXPECTED_ACTION_DIM",
    "EXPECTED_ACTOR_FEATURES",
    "EXPECTED_ALPHAS",
    "EXPECTED_BASE_ROWS",
    "EXPECTED_CASES",
    "EXPECTED_CONTROL_WINDOWS",
    "EXPECTED_FULL_FEATURES",
    "EXPECTED_P1_ROWS",
    "EXPECTED_P2_ROWS",
    "EXPECTED_RAW_SENSOR_SAMPLES",
    "EXPECTED_ROLLOUTS",
    "EXPECTED_SIGMA",
    "EXPECTED_STEPS",
    "INVARIANT_COLUMNS",
    "LockedTapeCorpus",
    "MASKED_COLUMNS",
    "MINIMUM_VALID_CYCLES",
    "MORPHOLOGY_WEIGHT",
    "NEAREST_CANDIDATE_ATOL",
    "P0_FIT_ROOT",
    "P0_GATE",
    "P0_MODULE",
    "P0_RECEIPT",
    "P0_SUMMARY",
    "P1_CORPUS",
    "P1_CORPUS_MANIFEST",
    "P1_FAILURE_STATUS",
    "P1_EXECUTION_LOCK",
    "P1_LEDGER",
    "P1_LEDGER_STATUS",
    "P1_RECEIPT",
    "P1_RECEIPT_STATUS",
    "P1_ROOT",
    "P1_PROTOCOL_FREEZE",
    "PENETRATION_LIMIT_M",
    "PROTOCOL_ID",
    "PROTOCOL_FREEZE_STATUS",
    "REVISION",
    "ROUND_ALPHAS",
    "RUN_ROOT",
    "SCHEMA_VERSION",
    "STAGE_ID",
    "SUPPORT_ID",
    "SUPPORT_LOO_MAX",
    "SUPPORT_LOO_MAX_HEX",
    "SUPPORT_P99_THRESHOLD",
    "SUPPORT_P99_HEX",
    "SupportEnvelope",
    "SupportQuery",
    "TAPE_ID",
    "TRANSITION_RADIUS_STEPS",
    "TapeDaggerAction",
    "TapeReferenceStep",
    "V12R15TapeDaggerError",
    "alpha_tag",
    "array_sha256",
    "build_protocol_description",
    "build_support_envelope",
    "canonical_case",
    "collection_gate",
    "collection_plan",
    "load_locked_tape_corpus",
    "select_tape_dagger_action",
    "source_snapshot",
    "support_manifest",
    "support_within_p99",
    "trajectory_id",
    "verify_runtime_source_closure",
    "verify_tape_source_closure",
]
