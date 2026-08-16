"""One-shot V12R12 actor builder from the frozen V8R1P1/V26 safe tape.

The candidate is an ordinary actor-only RLModule with the production topology
``35 -> 256 -> 256 -> 4`` and tanh hidden activations.  Its mean network is
fit either to P0's 3,000 frozen V5 teacher means or, after candidate-exposed
collection, to the exact P0-then-P1 12,000-row P2 union.  Columns 0, 1, and
10:25 are unavailable to imitation: they are
zeroed before optimization and their first-layer weights remain positive-zero
in the serialized runtime actor.  Consequently the learned mean depends only
on the invariant mechanical/reference columns 2:10 and 25:35.

There is exactly one fixed full-batch fit per authorized stage (AdamW followed
by LBFGS), no sweep, retry, replica, early stopping, checkpoint selection,
teacher query, policy rollout, environment call, critic update, or PPO update.
Both P0 and P2 reinitialize from H0; P2 never starts from P0.  H0 is also the
byte-exact source of the frozen ``sigma=0.005`` log-standard-deviation head.
Import and preflight are read-only.  Execution requires a matching protocol
freeze and execution lock; publication is no-clobber and fail-closed.
"""

from __future__ import annotations

import copy
import hashlib
import json
import math
import os
import stat
import sys
import tempfile
import time
from dataclasses import dataclass
from pathlib import Path, PurePath
from typing import Any, Mapping, Sequence

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

import warm_start  # noqa: E402
from asymmetric_rl_module import (  # noqa: E402
    AsymmetricActorCriticTorchRLModule,
)


class V12R12MaskedTeacherFitError(RuntimeError):
    """Raised when any immutable input, fit gate, or artifact gate fails."""


SCHEMA_VERSION = 1212
REVISION = "2026-08-15"
PROTOCOL_ID = "AB06_H0_V12R12_V26_INVARIANT_SAFE_TEACHER"
FIT_CONTRACT_ID = "h0_v12r12_masked_v8r1p1_safe_teacher_w256_v1"
TOPOLOGY_ID = "STANDARD_ACTOR_35_256_256_4_TANH"
SOURCE_H0_ID = "H0_MARKOV35_PHASE_ALIGNED_SIGMA0005_ITER1_RETRY"
SOURCE_CORPUS_ID = "H0_V8R1P1_V26_ACTIVE_TEACHER_ACTION_REPLAY"
EVENT_CONTRACT_ID = "binary_point_v25+heel_qualified_fsm_v2"

INPUT_DIM = 35
HIDDEN_DIMS = (256, 256)
ACTION_DIM = 2
LOGITS_DIM = 4
FULL_OBSERVATION_DIM = 84
P0_ROW_COUNT = 3_000
P1_ROW_COUNT = 9_000
P2_ROW_COUNT = 12_000
ROWS_PER_CASE = 500
TEACHER_SIGMA = np.float32(0.005)
LOGSTD_VALUE = np.float32(np.log(TEACHER_SIGMA))

MASKED_COLUMNS = (0, 1, *range(10, 25))
ACTIVE_COLUMNS = (*range(2, 10), *range(25, 35))
MASKED_FEATURE_NAMES = (
    "gait_phase_sin",
    "gait_phase_cos",
    "online_left_normal_grf_bw",
    "online_left_in_contact",
    "online_left_heel_strike",
    "online_left_toe_off",
    "online_left_gait_phase_sin",
    "online_left_gait_phase_cos",
    "online_left_cycle_duration_s",
    "phase_fsm_wait_hs",
    "phase_fsm_stance_after_hs",
    "phase_fsm_swing_after_to",
    "phase_expected_hs",
    "phase_expected_to",
    "phase_stance_elapsed_norm",
    "phase_swing_elapsed_norm",
    "phase_cycle_progress_credit",
)
ACTOR_FEATURE_NAMES = (
    "gait_phase_sin",
    "gait_phase_cos",
    "pros_knee_angle",
    "pros_knee_angle_vel",
    "pros_ankle_angle",
    "pros_ankle_angle_vel",
    "SEA_Knee_motor_angle",
    "SEA_Knee_motor_speed",
    "SEA_Ankle_motor_angle",
    "SEA_Ankle_motor_speed",
    "online_left_normal_grf_bw",
    "online_left_in_contact",
    "online_left_heel_strike",
    "online_left_toe_off",
    "online_left_gait_phase_sin",
    "online_left_gait_phase_cos",
    "online_left_cycle_duration_s",
    "phase_fsm_wait_hs",
    "phase_fsm_stance_after_hs",
    "phase_fsm_swing_after_to",
    "phase_expected_hs",
    "phase_expected_to",
    "phase_stance_elapsed_norm",
    "phase_swing_elapsed_norm",
    "phase_cycle_progress_credit",
    "pros_knee_angle_previous_endpoint",
    "pros_knee_angle_served_ref",
    "pros_knee_angle_served_ref_vel",
    "pros_knee_angle_served_ref_accel",
    "pros_knee_angle_sea_u",
    "pros_ankle_angle_previous_endpoint",
    "pros_ankle_angle_served_ref",
    "pros_ankle_angle_served_ref_vel",
    "pros_ankle_angle_served_ref_accel",
    "pros_ankle_angle_sea_u",
)
CASE_IDS = (
    "deterministic_offset_minus_0p20",
    "deterministic_offset_nominal",
    "deterministic_offset_plus_0p20",
    "stochastic_nominal_seed_126",
    "stochastic_nominal_seed_127",
    "stochastic_nominal_seed_128",
)
P1_ALPHAS = (0.25, 0.50, 0.75)
P1_SUPPORT_P99_THRESHOLD = 0.1937808123139821

# One fixed fit per stage with the sole preregistered reset-weighted MSE.
SEED = 20260815
FRESH_CRITIC_SEED = 20260715
TORCH_THREADS = 5
NORMALIZATION_STD_FLOOR = np.float32(1.0e-4)
ADAMW_EPOCHS = 3_000
ADAMW_BOUNDARIES = (1_500, 2_500, 3_000)
ADAMW_RATES = (3.0e-4, 1.0e-4, 3.0e-5)
ADAMW_WEIGHT_DECAY = 1.0e-7
GRADIENT_CLIP_NORM = 10.0
LBFGS_LR = 0.7
LBFGS_MAX_ITER = 3_000
LBFGS_MAX_EVAL = 4_500
LBFGS_HISTORY_SIZE = 50
LBFGS_TOLERANCE_GRAD = 1.0e-10
LBFGS_TOLERANCE_CHANGE = 1.0e-12
RESET_SAMPLE_WEIGHT = np.float32(100.0)
NONRESET_SAMPLE_WEIGHT = np.float32(1.0)
TRANSITION_RADIUS_STEPS = 2

# V5 W256 failed physically despite .00304/.02477 and V8R1P1 reached only
# .00330/.0471 offline.  These gates require an order-of-magnitude stronger
# closure before any physical development is even eligible to start.
GLOBAL_RMSE_LIMIT = 2.5e-4
GLOBAL_MAX_ABS_LIMIT = 3.0e-3
PER_CASE_ACTION_RMSE_LIMIT = 5.0e-4
PER_TRAJECTORY_ACTION_RMSE_LIMIT = 5.0e-4
RESET_MAX_ABS_LIMIT = 1.0e-5
TRANSITION_MAX_ABS_LIMIT = 2.0e-3
TEMPORAL_FIRST_DIFFERENCE_MAX_ABS_LIMIT = 3.0e-3
PREDICTED_MEAN_ABS_LIMIT = 0.95
FOLD_MAX_ABS_LIMIT = 2.0e-6

ACTOR_FIT_COUNT = 1
ACTOR_UPDATE_COUNT = 1
OFFLINE_TEACHER_QUERY_COUNT = 0
ENVIRONMENT_RESET_COUNT = 0
ENVIRONMENT_STEP_COUNT = 0
POLICY_ROLLOUT_COUNT = 0
CRITIC_UPDATE_COUNT = 0
PPO_UPDATE_COUNT = 0

SOURCE_H0_PATH = PurePath(
    "validation/critic_warmup/"
    "2026-07-13_markov35_phase_aligned_sigma0005_iter1_retry/rl_module_last"
)
SOURCE_H0_CONFIG_RECORD = {
    "path": (
        "validation/critic_warmup/"
        "2026-07-13_markov35_phase_aligned_sigma0005_iter1_retry/"
        "training_cfg.resolved.yaml"
    ),
    "sha256": "6904f7a9000b63b5c1aab661ebcab4974dffdd1cfb8c731df6a953fc9234229e",
    "size_bytes": 6_097,
}
V8R1P1_ROOT = PurePath(
    "validation/h0_primary_grf_split_adaptation_runs/"
    "2026-08-07_h0_primary_split_v8r1p1_v26_residual/teacher_replay"
)
V8R1P1_LEDGER_PATH = V8R1P1_ROOT / "execution_ledger.json"
RUN_ROOT = REVISION_ROOT / "h0_v12r12_run_20260815"
P0_OUTPUT = RUN_ROOT / "p0_fit"
P1_ROOT = RUN_ROOT / "p1_candidate_exposed"
P1_RECEIPT_PATH = P1_ROOT / "receipt.json"
P1_LEDGER_PATH = P1_ROOT / "pipeline_ledger.json"
P1_PROTOCOL_FREEZE_PATH = REVISION_ROOT / "h0_v12r12_tape_dagger_protocol_freeze.json"
P1_EXECUTION_LOCK_PATH = REVISION_ROOT / "h0_v12r12_tape_dagger_execution_lock.json"
P1_CORPUS_PATH = P1_ROOT / "corpus_candidate_exposed.npz"
P1_CORPUS_MANIFEST_PATH = P1_ROOT / "corpus_manifest.json"
P2_OUTPUT = RUN_ROOT / "p2_fit"
DEFAULT_OUTPUT = P0_OUTPUT
PROTOCOL_FREEZE_PATH = REVISION_ROOT / "h0_v12r12_protocol_freeze.json"
EXECUTION_LOCK_PATH = REVISION_ROOT / "h0_v12r12_execution_lock.json"
CANDIDATE_DIRNAMES = {
    "p0": "rl_module_p0_masked_safe_teacher",
    "p2": "rl_module_p2_masked_candidate_exposed",
}
STAGE_OUTPUTS = {"p0": P0_OUTPUT, "p2": P2_OUTPUT}
STAGE_NEXT = {
    "p0": "RUN_P1_CANDIDATE_EXPOSED",
    "p2": "FREEZE_THEN_RUN_P3_PLUS_0P20_PURE",
}
ACTOR_FEATURE_MANIFEST_NAME = "actor_feature_manifest.json"
CANDIDATE_BUILD_MANIFEST_NAME = "candidate_build_manifest.json"

SOURCE_H0_TREE = {
    "path": SOURCE_H0_PATH.as_posix(),
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
V8R1P1_LEDGER_RECORD = {
    "path": V8R1P1_LEDGER_PATH.as_posix(),
    "sha256": "f8a1807d750762227defd87e185b9fbc36487e5570e917398f87c54ba6b24433",
    "size_bytes": 7_488,
}
V8R1P1_EXECUTION_LOCK_RECORD = {
    "path": "validation/h0_primary_grf_split_v8r1p1_teacher_replay_execution_lock.json",
    "sha256": "2046f33c06d0aa052af8b7ef2089e33a0befb4bb53c75564853d6a29607df6c7",
    "size_bytes": 40_779,
}
EXPECTED_ARRAY_HASHES = {
    "observations": "4d964dff170fb076a7768084563ca7cf47a6ae77294d475d862df525d59b3e60",
    "masked_observations": "02be67d5fb530208be2b4fc5f66aa93c384c96d551f5b3901861d06a4b9d1bb4",
    "active_observations": "174f8e8d35a3fa1668d7e89fe19b7a27ddb80d493948b934f474d3739e4578f0",
    "actions": "4cbfd46c0787c7f52f7d8f642de8998faced2dee86a2e6fd79d3b3a3d758b402",
    "case_ids": "bab5db356e03645dd1818329f18ff0e459f7921f6d2f5bdd991da761130113ee",
    "trajectory_ids": "a8bb8cd015bc5ca609e27487bd053735d95b53be5ec1f3cf4114accce294baf3",
    "step_indices": "aeac92d327ded462799935323f78a9e4f245badb535dca1a5510ed89a98fe961",
    "transition_mask": "748a4c1a1febb0a3af12451686e55ff9182aa00c1c07662b428ec8f9596a1953",
}
P1_CORPUS_KEYS = frozenset(
    {
        "observations",
        "targets",
        "case_ids",
        "trajectory_ids",
        "requested_alpha",
        "effective_alpha",
        "step_indices",
        "recovery_mask",
        "support_distance",
        "transition_mask",
    }
)

STATE_KEYS = frozenset(
    {
        "pi_encoder.0.weight",
        "pi_encoder.0.bias",
        "pi_encoder.2.weight",
        "pi_encoder.2.bias",
        "pi.0.0.weight",
        "pi.0.0.bias",
        "pi.0.2.weight",
        "pi.0.2.bias",
        "pi.1.weight",
        "pi.1.bias",
    }
)
ALIASES = (
    ("pi_encoder.0.weight", "pi.0.0.weight"),
    ("pi_encoder.0.bias", "pi.0.0.bias"),
    ("pi_encoder.2.weight", "pi.0.2.weight"),
    ("pi_encoder.2.bias", "pi.0.2.bias"),
)
EXPECTED_STATE_SHAPES = {
    "pi_encoder.0.weight": (256, 35),
    "pi_encoder.0.bias": (256,),
    "pi_encoder.2.weight": (256, 256),
    "pi_encoder.2.bias": (256,),
    "pi.0.0.weight": (256, 35),
    "pi.0.0.bias": (256,),
    "pi.0.2.weight": (256, 256),
    "pi.0.2.bias": (256,),
    "pi.1.weight": (4, 256),
    "pi.1.bias": (4,),
}


@dataclass(frozen=True)
class LockedCorpus:
    stage: str
    observations: np.ndarray
    actions: np.ndarray
    case_ids: np.ndarray
    trajectory_ids: np.ndarray
    step_indices: np.ndarray
    transition_mask: np.ndarray
    source_records: Mapping[str, Any]
    audit: Mapping[str, Any]

    def arrays(self) -> dict[str, np.ndarray]:
        return {
            "observations": self.observations,
            "targets": self.actions,
            "case_ids": self.case_ids,
            "trajectory_ids": self.trajectory_ids,
            "step_indices": self.step_indices,
            "transition_mask": self.transition_mask,
            "actor_feature_names": np.asarray(ACTOR_FEATURE_NAMES, dtype="U64"),
            "masked_columns": np.asarray(MASKED_COLUMNS, dtype=np.int64),
            "active_columns": np.asarray(ACTIVE_COLUMNS, dtype=np.int64),
        }


@dataclass(frozen=True)
class MaskedNormalization:
    mean: np.ndarray
    std: np.ndarray
    source_rows: int

    def record(self) -> dict[str, Any]:
        return {
            "scope": "complete_locked_stage_corpus_active_columns_only",
            "source_rows": self.source_rows,
            "estimator": "population_mean_std_float64_then_float32",
            "std_floor": float(NORMALIZATION_STD_FLOOR),
            "masked_columns": list(MASKED_COLUMNS),
            "active_columns": list(ACTIVE_COLUMNS),
            "masked_means_positive_zero": _columns_vector_positive_zero(
                self.mean, MASKED_COLUMNS
            ),
            "masked_stds_exact_one": bool(
                np.all(self.std[np.asarray(MASKED_COLUMNS)] == np.float32(1.0))
            ),
            "mean_sha256": array_sha256(self.mean),
            "std_sha256": array_sha256(self.std),
            "folded_into_first_layer": True,
            "runtime_normalization_wrapper": False,
        }


@dataclass(frozen=True)
class FitResult:
    candidate_state: Mapping[str, np.ndarray]
    predictions: np.ndarray
    metrics: Mapping[str, Any]
    state_audit: Mapping[str, Any]
    normalization: MaskedNormalization
    fold_audit: Mapping[str, Any]
    optimizer_audit: Mapping[str, Any]
    history: tuple[Mapping[str, Any], ...]


def _absolute_no_follow(path: str | PurePath | Path) -> Path:
    raw = Path(path)
    if not raw.is_absolute():
        raw = REPO_ROOT / raw
    return Path(os.path.abspath(raw))


def _resolve(path: str | PurePath | Path) -> Path:
    absolute = _absolute_no_follow(path)
    root = _absolute_no_follow(REPO_ROOT)
    try:
        absolute.relative_to(root)
    except ValueError as exc:
        raise V12R12MaskedTeacherFitError(
            f"path escapes repository root: {path}"
        ) from exc
    return absolute


def _is_link_or_reparse(path: Path) -> bool:
    try:
        status = os.lstat(path)
    except OSError:
        return False
    attributes = int(getattr(status, "st_file_attributes", 0) or 0)
    reparse = int(getattr(stat, "FILE_ATTRIBUTE_REPARSE_POINT", 0x400))
    return stat.S_ISLNK(status.st_mode) or bool(attributes & reparse)


def _reject_link_components(path: Path, *, include_leaf: bool) -> None:
    root = _absolute_no_follow(REPO_ROOT)
    try:
        parts = path.relative_to(root).parts
    except ValueError as exc:
        raise V12R12MaskedTeacherFitError(f"unsafe path: {path}") from exc
    limit = len(parts) if include_leaf else max(0, len(parts) - 1)
    current = root
    for part in parts[:limit]:
        current /= part
        if os.path.lexists(current) and _is_link_or_reparse(current):
            raise V12R12MaskedTeacherFitError(
                f"symlink/junction path component rejected: {current}"
            )


def _regular_file(path: Path) -> bool:
    try:
        status = os.lstat(path)
    except OSError:
        return False
    attributes = int(getattr(status, "st_file_attributes", 0) or 0)
    reparse = int(getattr(stat, "FILE_ATTRIBUTE_REPARSE_POINT", 0x400))
    return stat.S_ISREG(status.st_mode) and not bool(attributes & reparse)


def _logical_path(path: str | PurePath | Path) -> str:
    return _resolve(path).relative_to(_absolute_no_follow(REPO_ROOT)).as_posix()


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _record(path: str | PurePath | Path) -> dict[str, Any]:
    resolved = _resolve(path)
    _reject_link_components(resolved, include_leaf=True)
    if not _regular_file(resolved):
        raise V12R12MaskedTeacherFitError(f"unsafe or missing file: {resolved}")
    return {
        "path": _logical_path(resolved),
        "sha256": _sha256_file(resolved),
        "size_bytes": int(resolved.stat().st_size),
    }


def _tree_record(path: str | PurePath | Path) -> dict[str, Any]:
    root = _resolve(path)
    _reject_link_components(root, include_leaf=True)
    try:
        root_status = os.lstat(root)
    except OSError as exc:
        raise V12R12MaskedTeacherFitError(f"missing tree: {root}") from exc
    if not stat.S_ISDIR(root_status.st_mode) or _is_link_or_reparse(root):
        raise V12R12MaskedTeacherFitError(f"unsafe tree: {root}")
    files: list[Path] = []
    for current_text, directory_names, file_names in os.walk(
        root, topdown=True, followlinks=False
    ):
        current = Path(current_text)
        directory_names.sort()
        file_names.sort()
        for name in directory_names:
            child = current / name
            if _is_link_or_reparse(child) or not stat.S_ISDIR(os.lstat(child).st_mode):
                raise V12R12MaskedTeacherFitError(f"unsafe tree directory: {child}")
        for name in file_names:
            child = current / name
            if not _regular_file(child) or _is_link_or_reparse(child):
                raise V12R12MaskedTeacherFitError(f"unsafe tree file: {child}")
            files.append(child)
    files.sort(key=lambda item: item.relative_to(root).as_posix())
    digest = hashlib.sha256()
    rows: list[dict[str, Any]] = []
    for item in files:
        relative = item.relative_to(root).as_posix()
        sha256 = _sha256_file(item)
        size_bytes = int(item.stat().st_size)
        rows.append({"path": relative, "sha256": sha256, "size_bytes": size_bytes})
        digest.update(relative.encode("utf-8"))
        digest.update(b"\0")
        digest.update(sha256.encode("ascii"))
        digest.update(b"\0")
        digest.update(str(size_bytes).encode("ascii"))
        digest.update(b"\n")
    if not rows:
        raise V12R12MaskedTeacherFitError(f"empty tree: {root}")
    return {
        "path": _logical_path(root),
        "tree_sha256": digest.hexdigest(),
        "file_count": len(rows),
        "files": rows,
    }


def _strict_json_value(path: str | PurePath | Path) -> Any:
    resolved = _resolve(path)
    try:
        value = json.loads(
            resolved.read_text(encoding="utf-8"),
            parse_constant=lambda token: (_ for _ in ()).throw(
                ValueError(f"non-finite JSON constant: {token}")
            ),
        )
    except Exception as exc:
        raise V12R12MaskedTeacherFitError(f"invalid strict JSON: {resolved}") from exc
    return value


def _strict_json(path: str | PurePath | Path) -> dict[str, Any]:
    resolved = _resolve(path)
    value = _strict_json_value(path)
    if not isinstance(value, dict):
        raise V12R12MaskedTeacherFitError(f"expected JSON object: {resolved}")
    return value


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


def _json_digest(value: Any) -> str:
    encoded = json.dumps(
        value,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=True,
        allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _assert_record_exact(
    observed: Mapping[str, Any], expected: Mapping[str, Any]
) -> None:
    if not _strict_equal(dict(observed), dict(expected)):
        raise V12R12MaskedTeacherFitError(
            f"artifact record drifted: expected {dict(expected)}, observed {dict(observed)}"
        )


def _write_json_exclusive(path: str | PurePath | Path, value: Any) -> Path:
    destination = _resolve(path)
    _reject_link_components(destination, include_leaf=False)
    destination.parent.mkdir(parents=True, exist_ok=True)
    encoded = (
        json.dumps(value, indent=2, sort_keys=True, ensure_ascii=False, allow_nan=False)
        + "\n"
    )
    flags = os.O_CREAT | os.O_EXCL | os.O_WRONLY | getattr(os, "O_BINARY", 0)
    try:
        descriptor = os.open(destination, flags, 0o600)
    except FileExistsError as exc:
        raise V12R12MaskedTeacherFitError(
            f"artifact exists/no-clobber: {destination}"
        ) from exc
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            descriptor = -1
            stream.write(encoded)
            stream.flush()
            os.fsync(stream.fileno())
    finally:
        if descriptor >= 0:
            os.close(descriptor)
    return destination


def _write_npz_exclusive(
    path: str | PurePath | Path, arrays: Mapping[str, Any]
) -> Path:
    destination = _resolve(path)
    _reject_link_components(destination, include_leaf=False)
    flags = os.O_CREAT | os.O_EXCL | os.O_WRONLY | getattr(os, "O_BINARY", 0)
    try:
        descriptor = os.open(destination, flags, 0o600)
    except FileExistsError as exc:
        raise V12R12MaskedTeacherFitError(
            f"artifact exists/no-clobber: {destination}"
        ) from exc
    try:
        with os.fdopen(descriptor, "wb") as stream:
            descriptor = -1
            np.savez_compressed(stream, **arrays)
            stream.flush()
            os.fsync(stream.fileno())
    finally:
        if descriptor >= 0:
            os.close(descriptor)
    return destination


def array_sha256(value: Any) -> str:
    array = np.ascontiguousarray(np.asarray(value))
    digest = hashlib.sha256()
    digest.update(array.dtype.str.encode("ascii"))
    digest.update(repr(array.shape).encode("ascii"))
    digest.update(array.tobytes(order="C"))
    return digest.hexdigest()


def _array(value: Any) -> np.ndarray:
    if hasattr(value, "detach"):
        value = value.detach()
    if hasattr(value, "cpu"):
        value = value.cpu()
    return np.ascontiguousarray(np.asarray(value))


def _state_arrays(state: Mapping[str, Any]) -> dict[str, np.ndarray]:
    return {name: _array(value).copy() for name, value in state.items()}


def _bytes_equal(left: Any, right: Any) -> bool:
    a = _array(left)
    b = _array(right)
    return (
        a.dtype == b.dtype
        and a.shape == b.shape
        and a.tobytes(order="C") == b.tobytes(order="C")
    )


def _state_byte_exact(left: Mapping[str, Any], right: Mapping[str, Any]) -> bool:
    return set(left) == set(right) and all(
        _bytes_equal(left[name], right[name]) for name in left
    )


def _positive_zero(value: Any) -> bool:
    array = _array(value)
    if array.dtype == np.dtype(np.float32):
        return bool(np.all(array.view(np.uint32) == 0))
    if array.dtype == np.dtype(np.float64):
        return bool(np.all(array.view(np.uint64) == 0))
    return False


def _columns_positive_zero(value: Any, columns: Sequence[int]) -> bool:
    array = _array(value)
    return array.ndim == 2 and all(
        _positive_zero(array[:, int(column)]) for column in columns
    )


def _columns_vector_positive_zero(value: Any, columns: Sequence[int]) -> bool:
    array = _array(value)
    return array.ndim == 1 and _positive_zero(array[np.asarray(columns, dtype=int)])


def _state_digest(state: Mapping[str, Any]) -> str:
    digest = hashlib.sha256()
    for name in sorted(state):
        value = _array(state[name])
        digest.update(name.encode("utf-8"))
        digest.update(b"\0")
        digest.update(value.dtype.str.encode("ascii"))
        digest.update(b"\0")
        digest.update(repr(value.shape).encode("ascii"))
        digest.update(b"\0")
        digest.update(value.tobytes(order="C"))
    return digest.hexdigest()


def _validate_referenced_artifact(record: Mapping[str, Any]) -> None:
    if set(record) != {"path", "sha256", "size_bytes"}:
        raise V12R12MaskedTeacherFitError("referenced artifact record schema drifted")
    _assert_record_exact(_record(str(record["path"])), record)


def attest_locked_sources() -> dict[str, Any]:
    """Close H0 and the complete transitive V8R1P1 artifact graph by hash."""

    h0 = _tree_record(SOURCE_H0_PATH)
    h0_config = _record(SOURCE_H0_CONFIG_RECORD["path"])
    ledger_record = _record(V8R1P1_LEDGER_PATH)
    execution_lock = _record(V8R1P1_EXECUTION_LOCK_RECORD["path"])
    _assert_record_exact(h0, SOURCE_H0_TREE)
    _assert_record_exact(h0_config, SOURCE_H0_CONFIG_RECORD)
    _assert_record_exact(ledger_record, V8R1P1_LEDGER_RECORD)
    _assert_record_exact(execution_lock, V8R1P1_EXECUTION_LOCK_RECORD)
    ledger = _strict_json(V8R1P1_LEDGER_PATH)
    checks = {
        "ledger_pass": ledger.get("passed") is True,
        "ledger_status": ledger.get("status")
        == "PASS_H0_V8R1P1_V26_TEACHER_REPLAY_DEVELOPMENT",
        "protocol_exact": ledger.get("protocol_id")
        == "AB06_H0_PRIMARY_SPLIT_V8R1P1_V26_RESIDUAL_DAGGER",
        "case_order_exact": ledger.get("expected_cases") == list(CASE_IDS)
        and ledger.get("completed_cases") == list(CASE_IDS),
        "zero_upstream_updates": ledger.get("actor_updates") == 0
        and ledger.get("critic_updates") == 0
        and ledger.get("ppo_updates") == 0,
        "no_upstream_candidate": ledger.get("candidate_created") is False,
        "protected_closed": ledger.get("protected_trials_opened") == [],
        "reserve_closed": ledger.get("reserve_trials_opened") == [],
        "execution_lock_exact": ledger.get("execution_lock")
        == V8R1P1_EXECUTION_LOCK_RECORD,
    }
    completed = ledger.get("completed_receipts")
    if not isinstance(completed, list) or len(completed) != len(CASE_IDS):
        raise V12R12MaskedTeacherFitError("V8R1P1 receipt index drifted")
    receipt_records: dict[str, dict[str, Any]] = {}
    for expected_case, item in zip(CASE_IDS, completed, strict=True):
        if not isinstance(item, Mapping) or item.get("case_id") != expected_case:
            raise V12R12MaskedTeacherFitError("V8R1P1 receipt order drifted")
        receipt_record = item.get("receipt")
        if not isinstance(receipt_record, Mapping):
            raise V12R12MaskedTeacherFitError("missing V8R1P1 receipt record")
        canonical = V8R1P1_ROOT / expected_case / "receipt.json"
        if receipt_record.get("path") != canonical.as_posix():
            raise V12R12MaskedTeacherFitError("non-canonical V8R1P1 receipt path")
        _validate_referenced_artifact(receipt_record)
        receipt = _strict_json(canonical)
        receipt_checks = {
            "pass": receipt.get("passed") is True,
            "status": receipt.get("status") == "PASS_H0_V8R1P1_V26_TEACHER_REPLAY",
            "case": receipt.get("case_id") == expected_case,
            "protocol": receipt.get("protocol_id")
            == "AB06_H0_PRIMARY_SPLIT_V8R1P1_V26_RESIDUAL_DAGGER",
            "persisted_before_gate": isinstance(
                receipt.get("persisted_before_gate"), Mapping
            )
            and set(receipt["persisted_before_gate"])
            == {"partial_summary", "summary", "trace"},
            "zero_updates": receipt.get("actor_updates") == 0
            and receipt.get("critic_updates") == 0
            and receipt.get("ppo_updates") == 0,
            "no_candidate": receipt.get("candidate_created") is False,
            "protected_closed": receipt.get("protected_trials_opened") == [],
        }
        artifacts = receipt.get("artifacts")
        expected_artifact_keys = {
            "gate",
            "partial_summary",
            "run_start",
            "steps",
            "summary",
            "trace",
        }
        if (
            not all(receipt_checks.values())
            or not isinstance(artifacts, Mapping)
            or set(artifacts) != expected_artifact_keys
            or not isinstance(artifacts.get("steps"), list)
            or len(artifacts["steps"]) != ROWS_PER_CASE
        ):
            raise V12R12MaskedTeacherFitError(
                f"V8R1P1 terminal receipt drifted: {expected_case}"
            )
        for name in expected_artifact_keys - {"steps"}:
            record = artifacts[name]
            if not isinstance(record, Mapping):
                raise V12R12MaskedTeacherFitError("artifact record is not a mapping")
            _validate_referenced_artifact(record)
        for index, record in enumerate(artifacts["steps"], start=1):
            expected_path = V8R1P1_ROOT / expected_case / "steps" / f"{index:06d}.json"
            if (
                not isinstance(record, Mapping)
                or record.get("path") != expected_path.as_posix()
            ):
                raise V12R12MaskedTeacherFitError(
                    f"V8R1P1 step index drifted: {expected_case}/{index}"
                )
            _validate_referenced_artifact(record)
        receipt_records[expected_case] = dict(receipt_record)
    if not all(checks.values()):
        failed = sorted(name for name, passed in checks.items() if not passed)
        raise V12R12MaskedTeacherFitError(f"V8R1P1 source closure failed: {failed}")
    return {
        "source_h0": h0,
        "source_h0_config": h0_config,
        "v8r1p1_ledger": ledger_record,
        "v8r1p1_execution_lock": execution_lock,
        "v8r1p1_receipts": receipt_records,
        "transitive_step_files_verified": P0_ROW_COUNT,
        "upstream_actor_updates": 0,
        "upstream_critic_updates": 0,
        "upstream_ppo_updates": 0,
    }


def _float32_vector(value: Any, *, width: int, label: str) -> np.ndarray:
    result = np.ascontiguousarray(np.asarray(value, dtype=np.float32))
    if result.shape != (width,) or not np.all(np.isfinite(result)):
        raise V12R12MaskedTeacherFitError(f"malformed finite {label}")
    return result


def load_locked_corpus() -> LockedCorpus:
    """Materialize the byte-locked 3000x35 -> 3000x2 offline corpus."""

    source_records = attest_locked_sources()
    observations: list[np.ndarray] = []
    actions: list[np.ndarray] = []
    case_ids: list[str] = []
    trajectory_ids: list[str] = []
    step_indices: list[int] = []
    transition_mask_rows: list[bool] = []
    binary_active_rows = 0
    invariant_exact_rows = 0
    for case_id in CASE_IDS:
        receipt = _strict_json(V8R1P1_ROOT / case_id / "receipt.json")
        step_records = receipt["artifacts"]["steps"]
        case_event_steps: list[int] = []
        for index, step_record in enumerate(step_records, start=1):
            step = _strict_json(str(step_record["path"]))
            observation = _float32_vector(
                step.get("v25_observation"), width=INPUT_DIM, label="V26 observation"
            )
            baseline = _float32_vector(
                step.get("baseline_teacher_observation"),
                width=INPUT_DIM,
                label="baseline teacher observation",
            )
            target = _float32_vector(
                step.get("frozen_teacher_mean"), width=ACTION_DIM, label="teacher mean"
            )
            queried = _float32_vector(
                step.get("queried_teacher_mean"), width=ACTION_DIM, label="queried mean"
            )
            teacher_std = _float32_vector(
                step.get("teacher_std"), width=ACTION_DIM, label="teacher std"
            )
            row_checks = step.get("checks")
            semantic_checks = {
                "case": step.get("case_id") == case_id,
                "step": step.get("step") == index,
                "row_checks": isinstance(row_checks, Mapping)
                and bool(row_checks)
                and all(value is True for value in row_checks.values()),
                "teacher_mean_exact": _bytes_equal(target, queried),
                "teacher_std_exact": _bytes_equal(
                    teacher_std, np.full(ACTION_DIM, TEACHER_SIGMA, dtype=np.float32)
                ),
                "active_columns_exact": _bytes_equal(
                    observation[np.asarray(ACTIVE_COLUMNS)],
                    baseline[np.asarray(ACTIVE_COLUMNS)],
                ),
                "binary_active_v26": step.get("phase_fsm", {}).get("event_source")
                == "binary_active_v26",
                "adapter_v26": step.get("binary_phase_active_adapter", {}).get(
                    "actor_event_source"
                )
                == "binary_active_v26",
                "fsm_v26": step.get("binary_phase_fsm", {}).get("source")
                == "binary_phase_fsm_v26",
                "event_contract": step.get("binary_phase_fsm", {}).get(
                    "event_contract_id"
                )
                == EVENT_CONTRACT_ID,
            }
            if not all(semantic_checks.values()):
                failed = sorted(
                    name for name, passed in semantic_checks.items() if not passed
                )
                raise V12R12MaskedTeacherFitError(
                    f"V8R1P1 row semantics failed {case_id}/{index}: {failed}"
                )
            binary_active_rows += 1
            invariant_exact_rows += 1
            accepted = step.get("phase_fsm", {}).get("accepted_transitions_this_step")
            if not isinstance(accepted, list):
                raise V12R12MaskedTeacherFitError(
                    f"transition journal malformed: {case_id}/{index}"
                )
            if accepted:
                case_event_steps.append(index)
            observations.append(observation)
            actions.append(target)
            case_ids.append(case_id)
            trajectory_ids.append(case_id)
            step_indices.append(index)
        case_transition_mask = np.zeros(ROWS_PER_CASE, dtype=np.bool_)
        for event_step in case_event_steps:
            start = max(0, event_step - 1 - TRANSITION_RADIUS_STEPS)
            stop = min(ROWS_PER_CASE, event_step + TRANSITION_RADIUS_STEPS)
            case_transition_mask[start:stop] = True
        transition_mask_rows.extend(case_transition_mask.tolist())
    observation_array = np.ascontiguousarray(observations, dtype=np.float32)
    action_array = np.ascontiguousarray(actions, dtype=np.float32)
    case_array = np.ascontiguousarray(case_ids, dtype="U40")
    trajectory_array = np.ascontiguousarray(trajectory_ids, dtype="U128")
    step_array = np.ascontiguousarray(step_indices, dtype=np.int64)
    transition_array = np.ascontiguousarray(transition_mask_rows, dtype=np.bool_)
    if (
        observation_array.shape != (P0_ROW_COUNT, INPUT_DIM)
        or action_array.shape != (P0_ROW_COUNT, ACTION_DIM)
        or case_array.shape != (P0_ROW_COUNT,)
        or trajectory_array.shape != (P0_ROW_COUNT,)
        or step_array.shape != (P0_ROW_COUNT,)
        or transition_array.shape != (P0_ROW_COUNT,)
    ):
        raise V12R12MaskedTeacherFitError("materialized corpus shape drifted")
    masked = observation_array.copy()
    masked[:, np.asarray(MASKED_COLUMNS)] = np.float32(0.0)
    observed_hashes = {
        "observations": array_sha256(observation_array),
        "masked_observations": array_sha256(masked),
        "active_observations": array_sha256(
            observation_array[:, np.asarray(ACTIVE_COLUMNS)]
        ),
        "actions": array_sha256(action_array),
        "case_ids": array_sha256(case_array),
        "trajectory_ids": array_sha256(trajectory_array),
        "step_indices": array_sha256(step_array),
        "transition_mask": array_sha256(transition_array),
    }
    if observed_hashes != EXPECTED_ARRAY_HASHES:
        raise V12R12MaskedTeacherFitError(
            f"V8R1P1 corpus arrays drifted: {observed_hashes}"
        )
    duplicate_targets: dict[bytes, bytes] = {}
    unique_active_rows: set[bytes] = set()
    duplicate_rows = 0
    for row, target in zip(
        observation_array[:, np.asarray(ACTIVE_COLUMNS)],
        action_array,
        strict=True,
    ):
        key = row.tobytes(order="C")
        encoded_target = target.tobytes(order="C")
        if key in duplicate_targets:
            duplicate_rows += 1
            if duplicate_targets[key] != encoded_target:
                raise V12R12MaskedTeacherFitError(
                    "active-input duplicate has conflicting safe-teacher mean"
                )
        else:
            duplicate_targets[key] = encoded_target
        unique_active_rows.add(key)
    audit = {
        "rows": P0_ROW_COUNT,
        "rows_per_case": ROWS_PER_CASE,
        "case_count": len(CASE_IDS),
        "observation_dim": INPUT_DIM,
        "action_dim": ACTION_DIM,
        "binary_active_v26_rows": binary_active_rows,
        "invariant_columns_exact_rows": invariant_exact_rows,
        "active_column_count": len(ACTIVE_COLUMNS),
        "masked_column_count": len(MASKED_COLUMNS),
        "unique_active_rows": len(unique_active_rows),
        "duplicate_active_rows": duplicate_rows,
        "conflicting_duplicate_targets": 0,
        "transition_window_radius_steps": TRANSITION_RADIUS_STEPS,
        "transition_window_rows": int(np.count_nonzero(transition_array)),
        "array_hashes": observed_hashes,
        "teacher_mean_max_abs": float(np.max(np.abs(action_array))),
        "offline_teacher_queries": 0,
    }
    return LockedCorpus(
        stage="p0",
        observations=observation_array,
        actions=action_array,
        case_ids=case_array,
        trajectory_ids=trajectory_array,
        step_indices=step_array,
        transition_mask=transition_array,
        source_records=source_records,
        audit=audit,
    )


def _validate_referenced_tree(record: Mapping[str, Any]) -> None:
    required = {"path", "tree_sha256", "file_count", "files"}
    if set(record) != required:
        raise V12R12MaskedTeacherFitError("referenced tree record schema drifted")
    _assert_record_exact(_tree_record(str(record["path"])), record)


def _validated_p0_terminal() -> dict[str, Any]:
    receipt_path = P0_OUTPUT / "receipt.json"
    gate_path = P0_OUTPUT / "gate.json"
    receipt = _strict_json(receipt_path)
    gate = _strict_json(gate_path)
    artifacts = receipt.get("artifacts")
    checks = {
        "receipt_pass": receipt.get("passed") is True,
        "receipt_status": receipt.get("status")
        == "PASS_H0_V12R12_MASKED_SAFE_TEACHER_FIT_RECEIPT",
        "protocol": receipt.get("protocol_id") == PROTOCOL_ID,
        "gate_pass": gate.get("passed") is True,
        "gate_status": gate.get("status") == "PASS_H0_V12R12_MASKED_SAFE_TEACHER_FIT",
        "p0_collector_only": receipt.get("collector_only") is True
        and receipt.get("non_promotable") is True
        and receipt.get("candidate_promoted") is False
        and receipt.get("next_stage") == STAGE_NEXT["p0"],
        "artifact_schema": isinstance(artifacts, Mapping)
        and set(artifacts) == {"corpus", "candidate_module", "summary", "gate"},
    }
    if not all(checks.values()):
        failed = sorted(name for name, passed in checks.items() if not passed)
        raise V12R12MaskedTeacherFitError(f"P0 terminal drifted: {failed}")
    for name in ("corpus", "summary", "gate"):
        record = artifacts[name]
        if not isinstance(record, Mapping):
            raise V12R12MaskedTeacherFitError("P0 artifact record malformed")
        _validate_referenced_artifact(record)
    candidate = artifacts["candidate_module"]
    if not isinstance(candidate, Mapping):
        raise V12R12MaskedTeacherFitError("P0 candidate tree record malformed")
    _validate_referenced_tree(candidate)
    expected_candidate_path = P0_OUTPUT / CANDIDATE_DIRNAMES["p0"]
    if candidate.get("path") != _logical_path(expected_candidate_path):
        raise V12R12MaskedTeacherFitError("P0 candidate path is non-canonical")
    return {
        "receipt": _record(receipt_path),
        "gate": _record(gate_path),
        "candidate_id": receipt.get("candidate_id"),
        "candidate_module": copy.deepcopy(dict(candidate)),
    }


def _p1_alpha_tag(alpha: float) -> str:
    tags = {0.25: "0p25", 0.50: "0p50", 0.75: "0p75"}
    if alpha not in tags:
        raise V12R12MaskedTeacherFitError("non-canonical P1 alpha")
    return tags[alpha]


def _p1_trajectory_id(alpha: float, case_id: str) -> str:
    if case_id not in CASE_IDS:
        raise V12R12MaskedTeacherFitError("non-canonical P1 trajectory identity")
    return f"alpha_{_p1_alpha_tag(alpha)}__{case_id}"


def _reconstruct_p1_journals(
    *,
    case_receipt_records: Sequence[Mapping[str, Any]],
    p0_terminal: Mapping[str, Any],
    p0_corpus: LockedCorpus,
) -> tuple[dict[str, np.ndarray], list[dict[str, Any]]]:
    """Independently rebuild all supervised P1 arrays from hashed step journals."""

    if len(case_receipt_records) != len(P1_ALPHAS) * len(CASE_IDS):
        raise V12R12MaskedTeacherFitError("P1 journal receipt count drifted")
    observations: list[np.ndarray] = []
    targets: list[np.ndarray] = []
    case_ids: list[str] = []
    trajectory_ids: list[str] = []
    step_indices: list[int] = []
    requested_alphas: list[float] = []
    effective_alphas: list[float] = []
    recovery_rows: list[bool] = []
    support_distances: list[float] = []
    transition_masks: list[np.ndarray] = []
    transition_provenance: list[dict[str, Any]] = []
    p0_targets = {
        case_id: p0_corpus.actions[
            case_index * ROWS_PER_CASE : (case_index + 1) * ROWS_PER_CASE
        ]
        for case_index, case_id in enumerate(CASE_IDS)
    }
    ordinal = 0
    for alpha in P1_ALPHAS:
        for case_id in CASE_IDS:
            trajectory_id = _p1_trajectory_id(alpha, case_id)
            collection_root = (
                P1_ROOT / "collections" / f"alpha_{_p1_alpha_tag(alpha)}" / case_id
            )
            receipt_path = collection_root / "receipt.json"
            receipt_record = case_receipt_records[ordinal]
            ordinal += 1
            if receipt_record.get("path") != _logical_path(receipt_path):
                raise V12R12MaskedTeacherFitError(
                    f"P1 receipt path drifted: {trajectory_id}"
                )
            _validate_referenced_artifact(receipt_record)
            receipt = _strict_json(receipt_path)
            artifacts = receipt.get("artifacts")
            receipt_checks = {
                "pass": receipt.get("passed") is True,
                "status": receipt.get("status")
                == "PASS_H0_V12R12_P1_CANDIDATE_EXPOSED_COLLECTION",
                "protocol": receipt.get("protocol_id") == PROTOCOL_ID,
                "identity": receipt.get("trajectory_id") == trajectory_id
                and receipt.get("case_id") == case_id
                and receipt.get("requested_alpha") == alpha,
                "source_candidate": receipt.get("source_candidate_id")
                == p0_terminal["candidate_id"]
                and _strict_equal(
                    receipt.get("source_p0_candidate"),
                    p0_terminal["candidate_module"],
                ),
                "rows": receipt.get("row_count") == ROWS_PER_CASE,
                "zero_updates": receipt.get("actor_updates") == 0
                and receipt.get("critic_updates") == 0
                and receipt.get("ppo_updates") == 0,
                "closed_trials": receipt.get("protected_trials_opened") == []
                and receipt.get("reserve_trials_opened") == [],
                "no_retry": receipt.get("retry_authorized") is False,
                "artifact_schema": isinstance(artifacts, Mapping)
                and set(artifacts)
                == {
                    "run_start",
                    "steps",
                    "trace",
                    "partial_summary",
                    "summary",
                    "gate",
                },
            }
            if not all(receipt_checks.values()):
                failed = sorted(
                    name for name, passed in receipt_checks.items() if not passed
                )
                raise V12R12MaskedTeacherFitError(
                    f"P1 collection receipt drifted {trajectory_id}: {failed}"
                )
            assert isinstance(artifacts, Mapping)
            step_records = artifacts["steps"]
            if not isinstance(step_records, list) or len(step_records) != ROWS_PER_CASE:
                raise V12R12MaskedTeacherFitError(
                    f"P1 step receipt count drifted: {trajectory_id}"
                )
            canonical_paths = {
                "run_start": collection_root / "run_start.json",
                "trace": collection_root / "trace.json",
                "partial_summary": collection_root / "partial_summary.json",
                "summary": collection_root / "summary.json",
                "gate": collection_root / "gate.json",
            }
            for name, path in canonical_paths.items():
                record = artifacts[name]
                if not isinstance(record, Mapping) or record.get(
                    "path"
                ) != _logical_path(path):
                    raise V12R12MaskedTeacherFitError(
                        f"P1 {name} binding drifted: {trajectory_id}"
                    )
                _validate_referenced_artifact(record)
            run_start = _strict_json(canonical_paths["run_start"])
            summary = _strict_json(canonical_paths["summary"])
            gate = _strict_json(canonical_paths["gate"])
            aggregate_checks = {
                "start_source": run_start.get("source_candidate_id")
                == p0_terminal["candidate_id"]
                and _strict_equal(
                    run_start.get("source_p0_candidate"),
                    p0_terminal["candidate_module"],
                ),
                "summary_identity": summary.get("trajectory_id") == trajectory_id
                and summary.get("case_id") == case_id
                and summary.get("requested_alpha") == alpha,
                "summary_source": summary.get("source_candidate_id")
                == p0_terminal["candidate_id"]
                and _strict_equal(
                    summary.get("source_p0_candidate"),
                    p0_terminal["candidate_module"],
                )
                and _strict_equal(
                    summary.get("source_snapshot_before"),
                    summary.get("source_snapshot_after"),
                ),
                "summary_rows": summary.get("steps") == ROWS_PER_CASE
                and summary.get("sample_count") == ROWS_PER_CASE
                and summary.get("persisted_label_count") == ROWS_PER_CASE,
                "summary_zero_updates": summary.get("actor_updates") == 0
                and summary.get("critic_updates") == 0
                and summary.get("ppo_updates") == 0
                and summary.get("candidate_update_count_before") == 0
                and summary.get("candidate_update_count_after") == 0,
                "gate_pass": gate.get("passed") is True
                and gate.get("status")
                == "PASS_H0_V12R12_P1_CANDIDATE_EXPOSED_COLLECTION"
                and gate.get("trajectory_id") == trajectory_id,
                "gate_checks": isinstance(gate.get("checks"), Mapping)
                and bool(gate["checks"])
                and all(value is True for value in gate["checks"].values()),
            }
            if not all(aggregate_checks.values()):
                failed = sorted(
                    name for name, passed in aggregate_checks.items() if not passed
                )
                raise V12R12MaskedTeacherFitError(
                    f"P1 collection aggregate drifted {trajectory_id}: {failed}"
                )
            step_rows: list[dict[str, Any]] = []
            event_steps: list[int] = []
            recovery_count = 0
            support_fallback_count = 0
            safety_fallback_count = 0
            for step, record in enumerate(step_records, start=1):
                step_path = collection_root / "steps" / f"{step:06d}.json"
                if not isinstance(record, Mapping) or record.get(
                    "path"
                ) != _logical_path(step_path):
                    raise V12R12MaskedTeacherFitError(
                        f"P1 step path drifted: {trajectory_id}/{step}"
                    )
                _validate_referenced_artifact(record)
                row = _strict_json(step_path)
                observation = _float32_vector(
                    row.get("v26_observation"),
                    width=INPUT_DIM,
                    label="P1 V26 observation",
                )
                target = _float32_vector(
                    row.get("tape_target_mean"),
                    width=ACTION_DIM,
                    label="P1 frozen target",
                )
                requested = row.get("requested_alpha")
                effective = row.get("effective_alpha")
                recovery = row.get("recovery")
                distance = row.get("support_distance_rms_z")
                support_intervened = row.get("support_intervened")
                safety_intervened = row.get("safety_intervened")
                within = row.get("support_within_p99")
                distance_valid = (
                    isinstance(distance, (int, float))
                    and not isinstance(distance, bool)
                    and math.isfinite(float(distance))
                    and float(distance) >= 0.0
                )
                observed_distance = float(distance) if distance_valid else math.nan
                phase = row.get("phase_fsm")
                accepted = (
                    phase.get("accepted_transitions_this_step")
                    if isinstance(phase, Mapping)
                    else None
                )
                expected_reasons = []
                if support_intervened is True:
                    expected_reasons.append("support_p99_exit")
                if safety_intervened is True:
                    expected_reasons.append("causal_penetration_latch")
                provenance = row.get("target_provenance")
                row_checks = row.get("checks")
                semantic_checks = {
                    "identity": row.get("step") == step
                    and row.get("trajectory_id") == trajectory_id
                    and row.get("case_id") == case_id,
                    "requested_alpha": requested == alpha,
                    "frozen_target": _bytes_equal(
                        target, p0_targets[case_id][step - 1]
                    ),
                    "target_provenance": isinstance(provenance, Mapping)
                    and provenance.get("source_case_id") == case_id
                    and provenance.get("source_step") == step
                    and provenance.get("source_field") == "frozen_teacher_mean"
                    and provenance.get("teacher_model_query") is False
                    and provenance.get("legacy_gait_shadow_query") is False,
                    "effective_alpha": effective
                    == (0.0 if recovery is True else alpha),
                    "recovery": type(recovery) is bool
                    and type(support_intervened) is bool
                    and type(safety_intervened) is bool
                    and recovery == (support_intervened or safety_intervened),
                    "fallback_reasons": row.get("fallback_reasons") == expected_reasons,
                    "support_distance": distance_valid,
                    "support_decision": type(within) is bool
                    and within == (observed_distance <= P1_SUPPORT_P99_THRESHOLD)
                    and support_intervened == (not within),
                    "accepted_transitions": isinstance(accepted, list),
                    "row_checks": isinstance(row_checks, Mapping)
                    and bool(row_checks)
                    and all(value is True for value in row_checks.values()),
                    "no_teacher_query": row.get("teacher_model_query_count") == 0
                    and row.get("legacy_shadow_query_count") == 0,
                }
                if not all(semantic_checks.values()):
                    failed = sorted(
                        name for name, passed in semantic_checks.items() if not passed
                    )
                    raise V12R12MaskedTeacherFitError(
                        f"P1 journal row drifted {trajectory_id}/{step}: {failed}"
                    )
                if accepted:
                    event_steps.append(step)
                recovery_count += int(recovery)
                support_fallback_count += int(support_intervened)
                safety_fallback_count += int(safety_intervened)
                observations.append(observation)
                targets.append(target)
                case_ids.append(case_id)
                trajectory_ids.append(trajectory_id)
                step_indices.append(step)
                requested_alphas.append(float(requested))
                effective_alphas.append(float(effective))
                recovery_rows.append(bool(recovery))
                support_distances.append(observed_distance)
                step_rows.append(row)
            trace = _strict_json_value(canonical_paths["trace"])
            if not isinstance(trace, list) or not _strict_equal(trace, step_rows):
                raise V12R12MaskedTeacherFitError(
                    f"P1 trace does not equal journals: {trajectory_id}"
                )
            transition_mask = np.zeros(ROWS_PER_CASE, dtype=np.bool_)
            for event_step in event_steps:
                start = max(0, event_step - 1 - TRANSITION_RADIUS_STEPS)
                stop = min(ROWS_PER_CASE, event_step + TRANSITION_RADIUS_STEPS)
                transition_mask[start:stop] = True
            transition_steps = (
                (np.flatnonzero(transition_mask) + 1).astype(int).tolist()
            )
            aggregate_semantics = {
                "receipt_events": receipt.get("accepted_transition_steps")
                == event_steps,
                "summary_events": summary.get("accepted_transition_steps")
                == event_steps,
                "summary_recovery": summary.get("recovery_row_count") == recovery_count,
                "summary_support": summary.get("support_fallback_count")
                == support_fallback_count,
                "summary_safety": summary.get("safety_fallback_count")
                == safety_fallback_count,
            }
            if not all(aggregate_semantics.values()):
                failed = sorted(
                    name for name, passed in aggregate_semantics.items() if not passed
                )
                raise V12R12MaskedTeacherFitError(
                    f"P1 journal counters drifted {trajectory_id}: {failed}"
                )
            transition_masks.append(transition_mask)
            transition_provenance.append(
                {
                    "trajectory_id": trajectory_id,
                    "case_id": case_id,
                    "requested_alpha": alpha,
                    "accepted_event_steps": event_steps,
                    "transition_radius_steps": TRANSITION_RADIUS_STEPS,
                    "transition_window_steps": transition_steps,
                    "transition_window_row_count": int(
                        np.count_nonzero(transition_mask)
                    ),
                    "source_trace": dict(artifacts["trace"]),
                }
            )
    return (
        {
            "observations": np.ascontiguousarray(observations, dtype=np.float32),
            "targets": np.ascontiguousarray(targets, dtype=np.float32),
            "case_ids": np.ascontiguousarray(case_ids, dtype="U40"),
            "trajectory_ids": np.ascontiguousarray(trajectory_ids, dtype="U80"),
            "step_indices": np.ascontiguousarray(step_indices, dtype=np.int64),
            "requested_alpha": np.ascontiguousarray(requested_alphas, dtype=np.float32),
            "effective_alpha": np.ascontiguousarray(effective_alphas, dtype=np.float32),
            "recovery_mask": np.ascontiguousarray(recovery_rows, dtype=np.bool_),
            "support_distance": np.ascontiguousarray(
                support_distances, dtype=np.float64
            ),
            "transition_mask": np.ascontiguousarray(
                np.concatenate(transition_masks), dtype=np.bool_
            ),
        },
        transition_provenance,
    )


def _assert_p1_npz_matches_journals(
    arrays: Mapping[str, Any], journal_arrays: Mapping[str, Any]
) -> dict[str, bool]:
    checks = {
        name: name in arrays
        and name in journal_arrays
        and _bytes_equal(arrays[name], journal_arrays[name])
        for name in sorted(P1_CORPUS_KEYS)
    }
    if not all(checks.values()):
        failed = sorted(name for name, passed in checks.items() if not passed)
        raise V12R12MaskedTeacherFitError(
            f"P1 NPZ does not equal immutable step journals: {failed}"
        )
    return checks


def _load_p1_candidate_exposed(
    *, p0_corpus: LockedCorpus | None = None
) -> LockedCorpus:
    """Load the hash-closed P1 18-trajectory candidate-exposed corpus."""

    p0 = _validated_p0_terminal()
    if p0_corpus is None:
        p0_corpus = load_locked_corpus()
    if p0_corpus.stage != "p0" or p0_corpus.actions.shape != (
        P0_ROW_COUNT,
        ACTION_DIM,
    ):
        raise V12R12MaskedTeacherFitError("P1 target source is not canonical P0")
    expected_trajectory_ids = tuple(
        _p1_trajectory_id(alpha, case_id) for alpha in P1_ALPHAS for case_id in CASE_IDS
    )
    alpha_count = len(P1_ALPHAS)
    case_count = len(CASE_IDS)
    trajectory_count = alpha_count * case_count
    receipt = _strict_json(P1_RECEIPT_PATH)
    ledger = _strict_json(P1_LEDGER_PATH)
    artifacts = receipt.get("artifacts")
    receipt_checks = {
        "pass": receipt.get("passed") is True,
        "status": receipt.get("status")
        == "PASS_H0_V12R12_P1_CANDIDATE_EXPOSED_RECEIPT",
        "protocol": receipt.get("protocol_id") == PROTOCOL_ID,
        "source_candidate": receipt.get("source_candidate_id") == p0["candidate_id"],
        "rows": receipt.get("row_count") == P1_ROW_COUNT,
        "trajectories": receipt.get("trajectory_count") == trajectory_count,
        "no_updates": receipt.get("actor_updates") == 0
        and receipt.get("critic_updates") == 0
        and receipt.get("ppo_updates") == 0,
        "no_new_candidate": receipt.get("candidate_created") is False,
        "protected_closed": receipt.get("protected_trials_opened") == [],
        "reserve_closed": receipt.get("reserve_trials_opened") == [],
        "no_retry": receipt.get("retry_authorized") is False,
        "artifact_schema": isinstance(artifacts, Mapping)
        and set(artifacts)
        == {
            "corpus",
            "corpus_manifest",
            "pipeline_ledger",
            "source_p0_candidate",
            "case_receipts",
        },
    }
    if not all(receipt_checks.values()):
        failed = sorted(name for name, passed in receipt_checks.items() if not passed)
        raise V12R12MaskedTeacherFitError(f"P1 receipt drifted: {failed}")
    for name in ("corpus", "corpus_manifest", "pipeline_ledger"):
        record = artifacts[name]
        if not isinstance(record, Mapping):
            raise V12R12MaskedTeacherFitError("P1 artifact record malformed")
        _validate_referenced_artifact(record)
    if artifacts["corpus"].get("path") != _logical_path(P1_CORPUS_PATH):
        raise V12R12MaskedTeacherFitError("P1 corpus path is non-canonical")
    if artifacts["corpus_manifest"].get("path") != _logical_path(
        P1_CORPUS_MANIFEST_PATH
    ):
        raise V12R12MaskedTeacherFitError("P1 corpus manifest path is non-canonical")
    if artifacts["pipeline_ledger"].get("path") != _logical_path(P1_LEDGER_PATH):
        raise V12R12MaskedTeacherFitError("P1 ledger path is non-canonical")
    for record_name, governance_path in (
        ("protocol_freeze", P1_PROTOCOL_FREEZE_PATH),
        ("execution_lock", P1_EXECUTION_LOCK_PATH),
    ):
        for payload_name, payload in (("receipt", receipt), ("ledger", ledger)):
            record = payload.get(record_name)
            if not isinstance(record, Mapping):
                raise V12R12MaskedTeacherFitError(
                    f"P1 {payload_name} does not bind the P1 {record_name}"
                )
            _validate_referenced_artifact(record)
            if record.get("path") != _logical_path(governance_path):
                raise V12R12MaskedTeacherFitError(
                    f"P1 {payload_name} {record_name} path is non-canonical"
                )
    source_p0 = artifacts["source_p0_candidate"]
    if not isinstance(source_p0, Mapping):
        raise V12R12MaskedTeacherFitError("P1 source P0 tree malformed")
    _validate_referenced_tree(source_p0)
    if not _strict_equal(source_p0, p0["candidate_module"]):
        raise V12R12MaskedTeacherFitError("P1 source P0 candidate identity drifted")
    case_receipts = artifacts["case_receipts"]
    if not isinstance(case_receipts, list) or len(case_receipts) != trajectory_count:
        raise V12R12MaskedTeacherFitError("P1 case receipt index drifted")
    for record in case_receipts:
        if not isinstance(record, Mapping):
            raise V12R12MaskedTeacherFitError("P1 case receipt record malformed")
        _validate_referenced_artifact(record)
    ledger_checks = {
        "pass": ledger.get("passed") is True,
        "status": ledger.get("status")
        == "PASS_H0_V12R12_P1_CANDIDATE_EXPOSED_PIPELINE",
        "protocol": ledger.get("protocol_id") == PROTOCOL_ID,
        "source_candidate": ledger.get("source_candidate_id") == p0["candidate_id"],
        "source_candidate_tree": _strict_equal(
            ledger.get("source_p0_candidate"), p0["candidate_module"]
        ),
        "completed_exact": ledger.get("completed_trajectory_ids")
        == list(expected_trajectory_ids),
        "receipts_exact": _strict_equal(
            ledger.get("completed_receipts"), case_receipts
        ),
        "rows": ledger.get("row_count") == P1_ROW_COUNT
        and ledger.get("trajectory_count") == trajectory_count,
        "no_updates": ledger.get("actor_updates") == 0
        and ledger.get("critic_updates") == 0
        and ledger.get("ppo_updates") == 0,
        "no_candidate": ledger.get("candidate_created") is False,
        "closed_trials": ledger.get("protected_trials_opened") == []
        and ledger.get("reserve_trials_opened") == [],
        "no_retry": ledger.get("retry_authorized") is False,
    }
    if not all(ledger_checks.values()):
        failed = sorted(name for name, passed in ledger_checks.items() if not passed)
        raise V12R12MaskedTeacherFitError(f"P1 ledger drifted: {failed}")
    try:
        with np.load(_resolve(P1_CORPUS_PATH), allow_pickle=False) as archive:
            if set(archive.files) != P1_CORPUS_KEYS:
                raise V12R12MaskedTeacherFitError("P1 NPZ key set drifted")
            arrays = {
                name: np.ascontiguousarray(archive[name]) for name in archive.files
            }
    except V12R12MaskedTeacherFitError:
        raise
    except Exception as exc:
        raise V12R12MaskedTeacherFitError("cannot read strict P1 corpus") from exc
    expected_shapes = {
        "observations": (P1_ROW_COUNT, INPUT_DIM),
        "targets": (P1_ROW_COUNT, ACTION_DIM),
        "case_ids": (P1_ROW_COUNT,),
        "trajectory_ids": (P1_ROW_COUNT,),
        "requested_alpha": (P1_ROW_COUNT,),
        "effective_alpha": (P1_ROW_COUNT,),
        "step_indices": (P1_ROW_COUNT,),
        "recovery_mask": (P1_ROW_COUNT,),
        "support_distance": (P1_ROW_COUNT,),
        "transition_mask": (P1_ROW_COUNT,),
    }
    expected_dtypes = {
        "observations": np.dtype(np.float32),
        "targets": np.dtype(np.float32),
        "requested_alpha": np.dtype(np.float32),
        "effective_alpha": np.dtype(np.float32),
        "step_indices": np.dtype(np.int64),
        "recovery_mask": np.dtype(np.bool_),
        "support_distance": np.dtype(np.float64),
        "transition_mask": np.dtype(np.bool_),
    }
    if any(arrays[name].shape != shape for name, shape in expected_shapes.items()):
        raise V12R12MaskedTeacherFitError("P1 corpus shape drifted")
    if any(arrays[name].dtype != dtype for name, dtype in expected_dtypes.items()):
        raise V12R12MaskedTeacherFitError("P1 corpus dtype drifted")
    if (
        arrays["case_ids"].dtype.kind != "U"
        or arrays["trajectory_ids"].dtype.kind != "U"
    ):
        raise V12R12MaskedTeacherFitError("P1 identity arrays must be Unicode")
    if not all(
        np.all(np.isfinite(arrays[name]))
        for name in (
            "observations",
            "targets",
            "requested_alpha",
            "effective_alpha",
            "support_distance",
        )
    ):
        raise V12R12MaskedTeacherFitError("P1 corpus contains non-finite values")
    ordered_trajectories = tuple(dict.fromkeys(arrays["trajectory_ids"].astype(str)))
    expected_cases = np.tile(
        np.repeat(np.asarray(CASE_IDS, dtype="U40"), ROWS_PER_CASE), alpha_count
    )
    expected_steps = np.tile(
        np.arange(1, ROWS_PER_CASE + 1, dtype=np.int64), trajectory_count
    )
    expected_alphas = np.repeat(
        np.asarray(P1_ALPHAS, dtype=np.float32), case_count * ROWS_PER_CASE
    )
    expected_trajectory_rows = np.repeat(
        np.asarray(expected_trajectory_ids, dtype=arrays["trajectory_ids"].dtype),
        ROWS_PER_CASE,
    )
    recovery = arrays["recovery_mask"]
    expected_effective = np.where(
        recovery, np.float32(0.0), arrays["requested_alpha"]
    ).astype(np.float32)
    support_exits = arrays["support_distance"] > P1_SUPPORT_P99_THRESHOLD
    expected_targets = np.ascontiguousarray(
        np.tile(p0_corpus.actions, (len(P1_ALPHAS), 1)), dtype=np.float32
    )
    ordering_checks = {
        "case_order": _bytes_equal(arrays["case_ids"], expected_cases),
        "step_order": _bytes_equal(arrays["step_indices"], expected_steps),
        "alpha_order": _bytes_equal(arrays["requested_alpha"], expected_alphas),
        "frozen_tape_targets_exact": _bytes_equal(arrays["targets"], expected_targets),
        "trajectory_ids_exact": ordered_trajectories == expected_trajectory_ids
        and _bytes_equal(arrays["trajectory_ids"], expected_trajectory_rows),
        "effective_alpha_recovery_exact": _bytes_equal(
            arrays["effective_alpha"], expected_effective
        ),
        "support_distance_nonnegative": bool(np.all(arrays["support_distance"] >= 0.0)),
        "support_exit_implies_recovery": bool(np.all(recovery[support_exits])),
        "nonrecovery_within_support": bool(
            np.all(arrays["support_distance"][~recovery] <= P1_SUPPORT_P99_THRESHOLD)
        ),
        "transition_rows_present": bool(np.any(arrays["transition_mask"])),
    }
    if not all(ordering_checks.values()):
        failed = sorted(name for name, passed in ordering_checks.items() if not passed)
        raise V12R12MaskedTeacherFitError(f"P1 corpus ordering drifted: {failed}")
    journal_arrays, transition_provenance = _reconstruct_p1_journals(
        case_receipt_records=case_receipts,
        p0_terminal=p0,
        p0_corpus=p0_corpus,
    )
    journal_checks = _assert_p1_npz_matches_journals(arrays, journal_arrays)
    array_hashes = {name: array_sha256(value) for name, value in arrays.items()}
    manifest = _strict_json(P1_CORPUS_MANIFEST_PATH)
    manifest_checks = {
        "pass": manifest.get("passed") is True,
        "status": manifest.get("status")
        == "PASS_H0_V12R12_P1_CANDIDATE_EXPOSED_CORPUS_MANIFEST",
        "protocol": manifest.get("protocol_id") == PROTOCOL_ID,
        "source_candidate": manifest.get("source_candidate_id") == p0["candidate_id"]
        and _strict_equal(manifest.get("source_p0_candidate"), p0["candidate_module"]),
        "rows": manifest.get("row_count") == P1_ROW_COUNT,
        "trajectories": manifest.get("trajectory_count") == trajectory_count,
        "keys": manifest.get("array_keys") == sorted(P1_CORPUS_KEYS),
        "shapes": manifest.get("array_shapes")
        == {name: list(expected_shapes[name]) for name in sorted(expected_shapes)},
        "dtypes": manifest.get("array_dtypes")
        == {name: arrays[name].dtype.str for name in sorted(arrays)},
        "hashes": manifest.get("array_hashes") == array_hashes,
        "trajectory_order": manifest.get("trajectory_ids")
        == list(expected_trajectory_ids),
        "ordering": manifest.get("ordering")
        == "alpha_major_then_canonical_case_ids_then_one_based_step",
        "fields": manifest.get("target_field") == "same_case_step_frozen_teacher_mean"
        and manifest.get("observation_field")
        == "candidate_exposed_pre_action_v26_observation",
        "transition_provenance": _strict_equal(
            manifest.get("transition_window_provenance"), transition_provenance
        )
        and _strict_equal(
            receipt.get("transition_window_provenance"), transition_provenance
        ),
        "p2_counts": manifest.get("base_rows_for_p2") == P0_ROW_COUNT
        and manifest.get("candidate_exposed_rows_for_p2") == P1_ROW_COUNT
        and manifest.get("expected_cumulative_p2_rows") == P2_ROW_COUNT,
        "zero_updates": manifest.get("teacher_model_queries") == 0
        and manifest.get("actor_updates") == 0
        and manifest.get("critic_updates") == 0
        and manifest.get("ppo_updates") == 0,
        "corpus_binding": manifest.get("corpus_artifact") == artifacts["corpus"],
    }
    if not all(manifest_checks.values()):
        failed = sorted(name for name, passed in manifest_checks.items() if not passed)
        raise V12R12MaskedTeacherFitError(f"P1 corpus manifest drifted: {failed}")
    if receipt.get("array_hashes") != array_hashes:
        raise V12R12MaskedTeacherFitError("P1 receipt array hashes drifted")
    ledger_artifacts = ledger.get("artifacts")
    ledger_closure = {
        "array_hashes": ledger.get("array_hashes") == array_hashes,
        "artifact_schema": isinstance(ledger_artifacts, Mapping)
        and set(ledger_artifacts) == {"corpus", "corpus_manifest", "case_receipts"},
        "corpus": isinstance(ledger_artifacts, Mapping)
        and _strict_equal(ledger_artifacts.get("corpus"), artifacts["corpus"]),
        "manifest": isinstance(ledger_artifacts, Mapping)
        and _strict_equal(
            ledger_artifacts.get("corpus_manifest"), artifacts["corpus_manifest"]
        ),
        "receipts": isinstance(ledger_artifacts, Mapping)
        and _strict_equal(ledger_artifacts.get("case_receipts"), case_receipts),
    }
    if not all(ledger_closure.values()):
        failed = sorted(name for name, passed in ledger_closure.items() if not passed)
        raise V12R12MaskedTeacherFitError(f"P1 ledger artifacts drifted: {failed}")
    return LockedCorpus(
        stage="p1",
        observations=np.ascontiguousarray(arrays["observations"], dtype=np.float32),
        actions=np.ascontiguousarray(arrays["targets"], dtype=np.float32),
        case_ids=np.ascontiguousarray(arrays["case_ids"], dtype="U40"),
        trajectory_ids=np.ascontiguousarray(arrays["trajectory_ids"], dtype="U128"),
        step_indices=np.ascontiguousarray(arrays["step_indices"], dtype=np.int64),
        transition_mask=np.ascontiguousarray(arrays["transition_mask"], dtype=np.bool_),
        source_records={
            "p0_terminal": p0,
            "p1_receipt": _record(P1_RECEIPT_PATH),
            "p1_ledger": _record(P1_LEDGER_PATH),
            "p1_corpus": _record(P1_CORPUS_PATH),
            "p1_corpus_manifest": _record(P1_CORPUS_MANIFEST_PATH),
            "p1_case_receipts": copy.deepcopy(case_receipts),
        },
        audit={
            "rows": P1_ROW_COUNT,
            "trajectory_count": trajectory_count,
            "reset_rows": int(np.count_nonzero(arrays["step_indices"] == 1)),
            "transition_window_rows": int(np.count_nonzero(arrays["transition_mask"])),
            "array_hashes": array_hashes,
            "ordering_checks": ordering_checks,
            "journal_byte_exact_checks": journal_checks,
            "recovery_rows": int(np.count_nonzero(arrays["recovery_mask"])),
        },
    )


def load_fit_corpus(stage: str) -> LockedCorpus:
    """Return P0 or the canonical P0+P1 union for the sole stage fit."""

    if stage == "p0":
        return load_locked_corpus()
    if stage != "p2":
        raise V12R12MaskedTeacherFitError(f"unsupported fit stage: {stage!r}")
    p0 = load_locked_corpus()
    p1 = _load_p1_candidate_exposed(p0_corpus=p0)
    observations = np.ascontiguousarray(
        np.concatenate((p0.observations, p1.observations)), dtype=np.float32
    )
    actions = np.ascontiguousarray(
        np.concatenate((p0.actions, p1.actions)), dtype=np.float32
    )
    case_ids = np.ascontiguousarray(
        np.concatenate((p0.case_ids, p1.case_ids)), dtype="U40"
    )
    trajectory_ids = np.ascontiguousarray(
        np.concatenate((p0.trajectory_ids, p1.trajectory_ids)), dtype="U128"
    )
    step_indices = np.ascontiguousarray(
        np.concatenate((p0.step_indices, p1.step_indices)), dtype=np.int64
    )
    transition_mask = np.ascontiguousarray(
        np.concatenate((p0.transition_mask, p1.transition_mask)), dtype=np.bool_
    )
    if (
        observations.shape != (P2_ROW_COUNT, INPUT_DIM)
        or actions.shape != (P2_ROW_COUNT, ACTION_DIM)
        or len(set(trajectory_ids.astype(str))) != 24
        or int(np.count_nonzero(step_indices == 1)) != 24
    ):
        raise V12R12MaskedTeacherFitError("P2 canonical union closure failed")
    hashes = {
        "observations": array_sha256(observations),
        "actions": array_sha256(actions),
        "case_ids": array_sha256(case_ids),
        "trajectory_ids": array_sha256(trajectory_ids),
        "step_indices": array_sha256(step_indices),
        "transition_mask": array_sha256(transition_mask),
    }
    return LockedCorpus(
        stage="p2",
        observations=observations,
        actions=actions,
        case_ids=case_ids,
        trajectory_ids=trajectory_ids,
        step_indices=step_indices,
        transition_mask=transition_mask,
        source_records={
            "p0": copy.deepcopy(dict(p0.source_records)),
            "p1": copy.deepcopy(dict(p1.source_records)),
        },
        audit={
            "rows": P2_ROW_COUNT,
            "p0_rows": P0_ROW_COUNT,
            "p1_rows": P1_ROW_COUNT,
            "trajectory_count": 24,
            "reset_rows": 24,
            "transition_window_rows": int(np.count_nonzero(transition_mask)),
            "array_hashes": hashes,
            "union_order": "P0_3000_THEN_P1_ALPHA_MAJOR_9000",
        },
    )


def build_masked_normalization(observations: Any) -> MaskedNormalization:
    raw = np.ascontiguousarray(np.asarray(observations), dtype=np.float32)
    if (
        raw.ndim != 2
        or raw.shape[1] != INPUT_DIM
        or len(raw)
        not in {
            P0_ROW_COUNT,
            P2_ROW_COUNT,
        }
    ):
        raise V12R12MaskedTeacherFitError("normalization requires exact P0 or P2 rows")
    if not np.all(np.isfinite(raw)):
        raise V12R12MaskedTeacherFitError("normalization rows are non-finite")
    mean = np.zeros(INPUT_DIM, dtype=np.float32)
    std = np.ones(INPUT_DIM, dtype=np.float32)
    active = np.asarray(ACTIVE_COLUMNS)
    mean[active] = raw[:, active].mean(axis=0, dtype=np.float64).astype(np.float32)
    raw_std = raw[:, active].std(axis=0, dtype=np.float64).astype(np.float32)
    std[active] = np.maximum(raw_std, NORMALIZATION_STD_FLOOR)
    mean[np.asarray(MASKED_COLUMNS)] = np.float32(0.0)
    std[np.asarray(MASKED_COLUMNS)] = np.float32(1.0)
    if not _columns_vector_positive_zero(mean, MASKED_COLUMNS):
        raise V12R12MaskedTeacherFitError("masked normalization means are not +0")
    return MaskedNormalization(
        mean=np.ascontiguousarray(mean),
        std=np.ascontiguousarray(std),
        source_rows=len(raw),
    )


def normalized_masked_observations(
    observations: Any, normalization: MaskedNormalization
) -> np.ndarray:
    raw = np.ascontiguousarray(np.asarray(observations), dtype=np.float32)
    if raw.ndim != 2 or raw.shape[1] != INPUT_DIM or not np.all(np.isfinite(raw)):
        raise V12R12MaskedTeacherFitError("observations must be finite Nx35")
    result = np.ascontiguousarray(
        (raw - normalization.mean) / normalization.std, dtype=np.float32
    )
    result[:, np.asarray(MASKED_COLUMNS)] = np.float32(0.0)
    if not _columns_positive_zero(result, MASKED_COLUMNS):
        raise V12R12MaskedTeacherFitError("imitation input mask is not positive-zero")
    return result


def validate_standard_state(
    state: Mapping[str, Any], *, source_state: Mapping[str, Any] | None = None
) -> dict[str, Any]:
    arrays = _state_arrays(state)
    checks = {
        "actor_only_key_set_exact": set(arrays) == STATE_KEYS,
        "state_shapes_exact": set(arrays) == set(EXPECTED_STATE_SHAPES)
        and all(
            arrays[name].shape == shape for name, shape in EXPECTED_STATE_SHAPES.items()
        ),
        "float32_finite": all(
            value.dtype == np.float32 and np.all(np.isfinite(value))
            for value in arrays.values()
        ),
        "encoder_aliases_byte_exact": all(
            _bytes_equal(arrays[left], arrays[right]) for left, right in ALIASES
        ),
        "masked_first_layer_columns_positive_zero": _columns_positive_zero(
            arrays.get("pi_encoder.0.weight", np.empty((0, 0))), MASKED_COLUMNS
        )
        and _columns_positive_zero(
            arrays.get("pi.0.0.weight", np.empty((0, 0))), MASKED_COLUMNS
        ),
        "logstd_weight_positive_zero": _positive_zero(
            arrays.get("pi.1.weight", np.empty((0, 0)))[ACTION_DIM:]
        )
        if "pi.1.weight" in arrays
        else False,
        "logstd_bias_sigma_0p005": _bytes_equal(
            arrays.get("pi.1.bias", np.empty(0))[ACTION_DIM:],
            np.full(ACTION_DIM, LOGSTD_VALUE, dtype=np.float32),
        )
        if "pi.1.bias" in arrays
        else False,
    }
    if source_state is not None and set(arrays) == STATE_KEYS:
        source = _state_arrays(source_state)
        checks["logstd_source_byte_exact"] = _bytes_equal(
            arrays["pi.1.weight"][ACTION_DIM:],
            source["pi.1.weight"][ACTION_DIM:],
        ) and _bytes_equal(
            arrays["pi.1.bias"][ACTION_DIM:],
            source["pi.1.bias"][ACTION_DIM:],
        )
    passed = all(checks.values())
    return {
        "passed": passed,
        "checks": checks,
        "topology_id": TOPOLOGY_ID,
        "state_digest": _state_digest(arrays) if set(arrays) == STATE_KEYS else None,
    }


def _load_source_module_and_state() -> tuple[Any, dict[str, np.ndarray]]:
    from ray.rllib.core.rl_module.rl_module import RLModule

    _assert_record_exact(_tree_record(SOURCE_H0_PATH), SOURCE_H0_TREE)
    module = RLModule.from_checkpoint(_resolve(SOURCE_H0_PATH))
    if type(module) is not AsymmetricActorCriticTorchRLModule:
        raise V12R12MaskedTeacherFitError("source H0 RLModule class drifted")
    state = _state_arrays(module.get_state())
    source_audit = validate_standard_state(_masked_source_view(state))
    source_checks = {
        "source_actor_only": set(state) == STATE_KEYS,
        "source_topology": list(module.model_config.get("fcnet_hiddens", ()))
        == list(HIDDEN_DIMS)
        and str(module.model_config.get("fcnet_activation", "")).lower() == "tanh",
        "source_actor_width": module.model_config.get("n_actor") == INPUT_DIM,
        "source_full_width": module.model_config.get("n_full") == FULL_OBSERVATION_DIM,
        "source_aliases": all(_bytes_equal(state[a], state[b]) for a, b in ALIASES),
        "source_logstd_weight_positive_zero": _positive_zero(
            state["pi.1.weight"][ACTION_DIM:]
        ),
        "source_logstd_sigma_0p005": _bytes_equal(
            state["pi.1.bias"][ACTION_DIM:],
            np.full(ACTION_DIM, LOGSTD_VALUE, dtype=np.float32),
        ),
        "masked_source_view_valid": source_audit["passed"],
    }
    if not all(source_checks.values()):
        failed = sorted(name for name, passed in source_checks.items() if not passed)
        raise V12R12MaskedTeacherFitError(f"source H0 state drifted: {failed}")
    return module, state


def _masked_source_view(source_state: Mapping[str, Any]) -> dict[str, np.ndarray]:
    state = _state_arrays(source_state)
    for key in ("pi_encoder.0.weight", "pi.0.0.weight"):
        state[key][:, np.asarray(MASKED_COLUMNS)] = np.float32(0.0)
    return state


def _new_normalized_model(
    source_state: Mapping[str, Any], normalization: MaskedNormalization
) -> Any:
    import torch
    from torch import nn

    source = _state_arrays(source_state)
    if set(source) != STATE_KEYS:
        raise V12R12MaskedTeacherFitError("source H0 is not actor-only")
    model = nn.Sequential(
        nn.Linear(INPUT_DIM, HIDDEN_DIMS[0]),
        nn.Tanh(),
        nn.Linear(HIDDEN_DIMS[0], HIDDEN_DIMS[1]),
        nn.Tanh(),
        nn.Linear(HIDDEN_DIMS[1], ACTION_DIM),
    )
    active = np.asarray(ACTIVE_COLUMNS)
    normalized_w0 = np.zeros((HIDDEN_DIMS[0], INPUT_DIM), dtype=np.float32)
    normalized_w0[:, active] = (
        source["pi_encoder.0.weight"][:, active] * normalization.std[active]
    )
    normalized_b0 = source["pi_encoder.0.bias"].copy()
    normalized_b0 += (
        source["pi_encoder.0.weight"][:, active] @ normalization.mean[active]
    )
    with torch.no_grad():
        model[0].weight.copy_(torch.as_tensor(normalized_w0))
        model[0].bias.copy_(torch.as_tensor(normalized_b0))
        model[2].weight.copy_(torch.as_tensor(source["pi_encoder.2.weight"]))
        model[2].bias.copy_(torch.as_tensor(source["pi_encoder.2.bias"]))
        model[4].weight.copy_(torch.as_tensor(source["pi.1.weight"][:ACTION_DIM]))
        model[4].bias.copy_(torch.as_tensor(source["pi.1.bias"][:ACTION_DIM]))
    _canonicalize_model_mask(model)
    return model


def _canonicalize_model_mask(model: Any) -> None:
    import torch

    with torch.no_grad():
        model[0].weight.index_fill_(
            1, torch.as_tensor(MASKED_COLUMNS, dtype=torch.long), 0.0
        )
    if not _columns_positive_zero(
        model[0].weight.detach().cpu().numpy(), MASKED_COLUMNS
    ):
        raise V12R12MaskedTeacherFitError("trainable first-layer mask drifted")


def _state_logits(state: Mapping[str, Any], observations: Any) -> np.ndarray:
    import torch
    import torch.nn.functional as functional

    arrays = _state_arrays(state)
    x = torch.as_tensor(
        np.ascontiguousarray(observations, dtype=np.float32), dtype=torch.float32
    )
    with torch.no_grad():
        hidden = torch.tanh(
            functional.linear(
                x,
                torch.as_tensor(arrays["pi_encoder.0.weight"]),
                torch.as_tensor(arrays["pi_encoder.0.bias"]),
            )
        )
        hidden = torch.tanh(
            functional.linear(
                hidden,
                torch.as_tensor(arrays["pi_encoder.2.weight"]),
                torch.as_tensor(arrays["pi_encoder.2.bias"]),
            )
        )
        logits = functional.linear(
            hidden,
            torch.as_tensor(arrays["pi.1.weight"]),
            torch.as_tensor(arrays["pi.1.bias"]),
        )
    result = np.ascontiguousarray(logits.cpu().numpy(), dtype=np.float32)
    if result.shape != (len(x), LOGITS_DIM) or not np.all(np.isfinite(result)):
        raise V12R12MaskedTeacherFitError("candidate logits are malformed")
    return result


def _fold_model_into_state(
    model: Any,
    source_state: Mapping[str, Any],
    normalization: MaskedNormalization,
) -> tuple[dict[str, np.ndarray], dict[str, Any]]:
    _canonicalize_model_mask(model)
    normalized_w0 = np.ascontiguousarray(
        model[0].weight.detach().cpu().numpy(), dtype=np.float32
    )
    normalized_b0 = np.ascontiguousarray(
        model[0].bias.detach().cpu().numpy(), dtype=np.float32
    )
    raw_w0 = np.ascontiguousarray(
        normalized_w0 / normalization.std[None, :], dtype=np.float32
    )
    raw_w0[:, np.asarray(MASKED_COLUMNS)] = np.float32(0.0)
    raw_b0 = np.ascontiguousarray(
        normalized_b0 - raw_w0 @ normalization.mean, dtype=np.float32
    )
    second_w = np.ascontiguousarray(
        model[2].weight.detach().cpu().numpy(), dtype=np.float32
    )
    second_b = np.ascontiguousarray(
        model[2].bias.detach().cpu().numpy(), dtype=np.float32
    )
    mean_w = np.ascontiguousarray(
        model[4].weight.detach().cpu().numpy(), dtype=np.float32
    )
    mean_b = np.ascontiguousarray(
        model[4].bias.detach().cpu().numpy(), dtype=np.float32
    )
    candidate = _state_arrays(source_state)
    for prefix in ("pi_encoder", "pi.0"):
        candidate[f"{prefix}.0.weight"] = raw_w0.copy()
        candidate[f"{prefix}.0.bias"] = raw_b0.copy()
        candidate[f"{prefix}.2.weight"] = second_w.copy()
        candidate[f"{prefix}.2.bias"] = second_b.copy()
    output_w = candidate["pi.1.weight"].copy()
    output_b = candidate["pi.1.bias"].copy()
    output_w[:ACTION_DIM] = mean_w
    output_b[:ACTION_DIM] = mean_b
    candidate["pi.1.weight"] = output_w
    candidate["pi.1.bias"] = output_b
    return candidate, {
        "normalization_folded": True,
        "runtime_wrapper": False,
        "masked_columns_positive_zero": _columns_positive_zero(raw_w0, MASKED_COLUMNS),
    }


def _fit_objective(
    prediction: Any,
    target: Any,
    reset_mask: Any,
) -> tuple[Any, Any, Any]:
    import torch

    error = prediction - target
    mse = torch.mean(torch.square(error))
    row_mse = torch.mean(torch.square(error), dim=1)
    weights = torch.ones_like(row_mse)
    weights = torch.where(
        reset_mask,
        torch.full_like(weights, float(RESET_SAMPLE_WEIGHT)),
        weights,
    )
    weighted_mse = torch.sum(weights * row_mse) / torch.sum(weights)
    reset_mse = torch.mean(torch.square(error[reset_mask]))
    return weighted_mse, mse, reset_mse


def adamw_rate(epoch: int) -> float:
    if type(epoch) is not int or epoch < 1 or epoch > ADAMW_EPOCHS:
        raise V12R12MaskedTeacherFitError(f"AdamW epoch outside fixed range: {epoch!r}")
    if epoch <= ADAMW_BOUNDARIES[0]:
        return ADAMW_RATES[0]
    if epoch <= ADAMW_BOUNDARIES[1]:
        return ADAMW_RATES[1]
    return ADAMW_RATES[2]


def _milestone(
    history: list[dict[str, Any]],
    *,
    optimizer: str,
    index: int,
    objective: Any,
    mse: Any,
    reset_mse: Any,
    learning_rate: float,
) -> None:
    values = {
        "objective": float(objective.detach().cpu().item()),
        "mse": float(mse.detach().cpu().item()),
        "reset_mse": float(reset_mse.detach().cpu().item()),
    }
    if not all(math.isfinite(value) for value in values.values()):
        raise V12R12MaskedTeacherFitError("non-finite optimizer milestone")
    history.append(
        {
            "optimizer": optimizer,
            "index": int(index),
            "learning_rate": float(learning_rate),
            **values,
        }
    )


def prediction_metrics(
    predictions: Any,
    targets: Any,
    case_ids: Any,
    trajectory_ids: Any,
    step_indices: Any,
    transition_mask: Any,
) -> dict[str, Any]:
    pred = np.ascontiguousarray(predictions, dtype=np.float32)
    target = np.ascontiguousarray(targets, dtype=np.float32)
    cases = np.asarray(case_ids).astype(str)
    trajectories = np.asarray(trajectory_ids).astype(str)
    steps = np.asarray(step_indices, dtype=np.int64)
    transitions = np.asarray(transition_mask, dtype=np.bool_)
    rows = len(pred)
    if pred.shape != target.shape or pred.ndim != 2 or pred.shape[1] != ACTION_DIM:
        raise V12R12MaskedTeacherFitError("metric arrays have wrong shape")
    if (
        cases.shape != (rows,)
        or trajectories.shape != (rows,)
        or steps.shape != (rows,)
    ):
        raise V12R12MaskedTeacherFitError("metric identity arrays have wrong shape")
    error = pred.astype(np.float64) - target.astype(np.float64)

    if transitions.shape != (rows,) or not np.any(transitions):
        raise V12R12MaskedTeacherFitError("transition mask is malformed")

    def metric(indices: np.ndarray) -> dict[str, Any]:
        selected = error[indices]
        return {
            "rmse": float(np.sqrt(np.mean(np.square(selected)))),
            "max_abs_error": float(np.max(np.abs(selected))),
            "per_action_rmse": [
                float(np.sqrt(np.mean(np.square(selected[:, action]))))
                for action in range(ACTION_DIM)
            ],
        }

    ordered_cases = tuple(dict.fromkeys(cases.tolist()))
    per_case = {
        case_id: metric(np.flatnonzero(cases == case_id)) for case_id in ordered_cases
    }
    ordered_trajectories = tuple(dict.fromkeys(trajectories.tolist()))
    per_trajectory = {
        trajectory_id: metric(np.flatnonzero(trajectories == trajectory_id))
        for trajectory_id in ordered_trajectories
    }
    reset = np.flatnonzero(steps == 1)
    per_action_rmse = [
        float(np.sqrt(np.mean(np.square(error[:, action]))))
        for action in range(ACTION_DIM)
    ]
    case_action_values = [
        value for row in per_case.values() for value in row["per_action_rmse"]
    ]
    trajectory_action_values = [
        value for row in per_trajectory.values() for value in row["per_action_rmse"]
    ]
    consecutive = (
        np.flatnonzero(
            (trajectories[1:] == trajectories[:-1]) & (steps[1:] == steps[:-1] + 1)
        )
        + 1
    )
    if len(consecutive) != rows - len(set(trajectories.tolist())):
        raise V12R12MaskedTeacherFitError(
            "trajectory ordering is not contiguous step 1..N"
        )
    prediction_delta = pred[consecutive].astype(np.float64) - pred[
        consecutive - 1
    ].astype(np.float64)
    target_delta = target[consecutive].astype(np.float64) - target[
        consecutive - 1
    ].astype(np.float64)
    temporal_error = prediction_delta - target_delta
    return {
        "global": metric(np.arange(rows)),
        "per_case": per_case,
        "per_trajectory": per_trajectory,
        "per_action_rmse": per_action_rmse,
        "worst_per_case_or_action_rmse": max(
            max(row["rmse"] for row in per_case.values()),
            max(per_action_rmse),
            max(case_action_values),
        ),
        "worst_per_trajectory_or_action_rmse": max(
            max(row["rmse"] for row in per_trajectory.values()),
            max(trajectory_action_values),
        ),
        "reset_max_abs_error": float(np.max(np.abs(error[reset]))),
        "transition_window_rows": int(np.count_nonzero(transitions)),
        "transition_window_max_abs_error": float(np.max(np.abs(error[transitions]))),
        "temporal_first_difference_pair_count": len(consecutive),
        "temporal_first_difference_max_abs_error": float(
            np.max(np.abs(temporal_error))
        ),
        "predicted_mean_max_abs": float(np.max(np.abs(pred))),
        "prediction_sha256": array_sha256(pred),
    }


def fit_gate(
    *,
    metrics: Mapping[str, Any],
    state_audit: Mapping[str, Any],
    fold_audit: Mapping[str, Any],
) -> dict[str, Any]:
    global_metrics = metrics.get("global", {})
    checks = {
        "global_rmse_le_0p00025": float(global_metrics.get("rmse", math.inf))
        <= GLOBAL_RMSE_LIMIT,
        "global_max_abs_le_0p003": float(global_metrics.get("max_abs_error", math.inf))
        <= GLOBAL_MAX_ABS_LIMIT,
        "every_case_and_action_rmse_le_0p0005": float(
            metrics.get("worst_per_case_or_action_rmse", math.inf)
        )
        <= PER_CASE_ACTION_RMSE_LIMIT,
        "every_trajectory_and_action_rmse_le_0p0005": float(
            metrics.get("worst_per_trajectory_or_action_rmse", math.inf)
        )
        <= PER_TRAJECTORY_ACTION_RMSE_LIMIT,
        "reset_max_abs_le_0p00001": float(metrics.get("reset_max_abs_error", math.inf))
        <= RESET_MAX_ABS_LIMIT,
        "transition_window_max_abs_le_0p002": float(
            metrics.get("transition_window_max_abs_error", math.inf)
        )
        <= TRANSITION_MAX_ABS_LIMIT,
        "temporal_first_difference_max_abs_le_0p003": float(
            metrics.get("temporal_first_difference_max_abs_error", math.inf)
        )
        <= TEMPORAL_FIRST_DIFFERENCE_MAX_ABS_LIMIT,
        "predicted_mean_abs_strictly_lt_0p95": float(
            metrics.get("predicted_mean_max_abs", math.inf)
        )
        < PREDICTED_MEAN_ABS_LIMIT,
        "state_pass": state_audit.get("passed") is True,
        "normalization_fold_equivalent": fold_audit.get("passed") is True,
        "mask_positive_zero": state_audit.get("checks", {}).get(
            "masked_first_layer_columns_positive_zero"
        )
        is True,
        "logstd_exact_0p005": state_audit.get("checks", {}).get(
            "logstd_source_byte_exact"
        )
        is True
        and state_audit.get("checks", {}).get("logstd_bias_sigma_0p005") is True,
    }
    return {
        "passed": all(checks.values()),
        "checks": checks,
        "limits": {
            "global_rmse": GLOBAL_RMSE_LIMIT,
            "global_max_abs_error": GLOBAL_MAX_ABS_LIMIT,
            "per_case_and_action_rmse": PER_CASE_ACTION_RMSE_LIMIT,
            "per_trajectory_and_action_rmse": PER_TRAJECTORY_ACTION_RMSE_LIMIT,
            "reset_max_abs_error": RESET_MAX_ABS_LIMIT,
            "transition_window_max_abs_error": TRANSITION_MAX_ABS_LIMIT,
            "temporal_first_difference_max_abs_error": (
                TEMPORAL_FIRST_DIFFERENCE_MAX_ABS_LIMIT
            ),
            "predicted_mean_max_abs_strict": PREDICTED_MEAN_ABS_LIMIT,
            "fold_max_abs_difference": FOLD_MAX_ABS_LIMIT,
        },
    }


def fit_candidate_in_memory(
    *,
    source_state: Mapping[str, Any],
    corpus: LockedCorpus,
    normalization: MaskedNormalization,
) -> FitResult:
    """Execute the sole fixed V12R12 fit without publishing artifacts."""

    import torch

    normalized = normalized_masked_observations(corpus.observations, normalization)
    labels = np.ascontiguousarray(corpus.actions, dtype=np.float32)
    previous_threads = torch.get_num_threads()
    previous_deterministic = torch.are_deterministic_algorithms_enabled()
    started = time.monotonic()
    torch.set_num_threads(TORCH_THREADS)
    torch.use_deterministic_algorithms(True)
    try:
        torch.manual_seed(SEED)
        model = _new_normalized_model(source_state, normalization)
        x = torch.as_tensor(normalized, dtype=torch.float32)
        y = torch.as_tensor(labels, dtype=torch.float32)
        reset_mask = torch.as_tensor(corpus.step_indices == 1, dtype=torch.bool)
        mask = torch.as_tensor(MASKED_COLUMNS, dtype=torch.long)
        model[0].weight.register_hook(
            lambda gradient: gradient.index_fill(1, mask.to(gradient.device), 0.0)
        )
        history: list[dict[str, Any]] = []
        optimizer = torch.optim.AdamW(
            model.parameters(),
            lr=ADAMW_RATES[0],
            weight_decay=ADAMW_WEIGHT_DECAY,
            foreach=False,
        )
        adam_milestones = {1, 500, 1_000, 1_500, 2_000, 2_500, 3_000}
        for epoch in range(1, ADAMW_EPOCHS + 1):
            learning_rate = adamw_rate(epoch)
            for group in optimizer.param_groups:
                group["lr"] = learning_rate
            optimizer.zero_grad(set_to_none=True)
            objective, mse, reset_mse = _fit_objective(model(x), y, reset_mask)
            if not torch.isfinite(objective):
                raise V12R12MaskedTeacherFitError(
                    f"non-finite AdamW objective at epoch {epoch}"
                )
            objective.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), GRADIENT_CLIP_NORM)
            optimizer.step()
            _canonicalize_model_mask(model)
            if epoch in adam_milestones:
                _milestone(
                    history,
                    optimizer="adamw",
                    index=epoch,
                    objective=objective,
                    mse=mse,
                    reset_mse=reset_mse,
                    learning_rate=learning_rate,
                )
        lbfgs = torch.optim.LBFGS(
            model.parameters(),
            lr=LBFGS_LR,
            max_iter=LBFGS_MAX_ITER,
            max_eval=LBFGS_MAX_EVAL,
            tolerance_grad=LBFGS_TOLERANCE_GRAD,
            tolerance_change=LBFGS_TOLERANCE_CHANGE,
            history_size=LBFGS_HISTORY_SIZE,
            line_search_fn="strong_wolfe",
        )
        closure_calls = 0
        terminal: tuple[Any, Any, Any] | None = None

        def closure() -> Any:
            nonlocal closure_calls, terminal
            lbfgs.zero_grad(set_to_none=True)
            objective, mse, reset_mse = _fit_objective(model(x), y, reset_mask)
            if not torch.isfinite(objective):
                raise V12R12MaskedTeacherFitError(
                    f"non-finite LBFGS objective at closure {closure_calls + 1}"
                )
            objective.backward()
            closure_calls += 1
            terminal = (objective, mse, reset_mse)
            if closure_calls in {1, 100, 250, 500, 1_000, 2_000, 3_000}:
                _milestone(
                    history,
                    optimizer="lbfgs_closure",
                    index=closure_calls,
                    objective=objective,
                    mse=mse,
                    reset_mse=reset_mse,
                    learning_rate=LBFGS_LR,
                )
            return objective

        lbfgs.step(closure)
        _canonicalize_model_mask(model)
        if terminal is None:
            raise V12R12MaskedTeacherFitError("LBFGS never evaluated the objective")
        candidate, fold = _fold_model_into_state(model, source_state, normalization)
        with torch.no_grad():
            normalized_prediction = np.ascontiguousarray(
                model(x).cpu().numpy(), dtype=np.float32
            )
        runtime_logits = _state_logits(candidate, corpus.observations)
        runtime_prediction = runtime_logits[:, :ACTION_DIM]
        fold_difference = np.abs(
            normalized_prediction.astype(np.float64)
            - runtime_prediction.astype(np.float64)
        )
        fold_max = float(np.max(fold_difference))
        fold_audit = {
            **fold,
            "passed": bool(
                fold["masked_columns_positive_zero"]
                and math.isfinite(fold_max)
                and fold_max <= FOLD_MAX_ABS_LIMIT
            ),
            "max_abs_difference": fold_max,
            "limit": FOLD_MAX_ABS_LIMIT,
        }
        state_audit = validate_standard_state(candidate, source_state=source_state)
        mean_changed = any(
            not _bytes_equal(candidate[key], source_state[key])
            for key in (
                "pi_encoder.0.weight",
                "pi_encoder.0.bias",
                "pi_encoder.2.weight",
                "pi_encoder.2.bias",
                "pi.1.weight",
                "pi.1.bias",
            )
        )
        state_audit = copy.deepcopy(dict(state_audit))
        state_audit["checks"]["mean_network_changed"] = mean_changed
        state_audit["passed"] = bool(state_audit["passed"] and mean_changed)
        metrics = prediction_metrics(
            runtime_prediction,
            corpus.actions,
            corpus.case_ids,
            corpus.trajectory_ids,
            corpus.step_indices,
            corpus.transition_mask,
        )
        final_objective, final_mse, final_reset_mse = terminal
        optimizer_audit = {
            "fit_contract_id": FIT_CONTRACT_ID,
            "single_fixed_fit": True,
            "seed": SEED,
            "torch_threads": TORCH_THREADS,
            "deterministic_algorithms": True,
            "full_batch": True,
            "sample_count": len(corpus.observations),
            "fit_stage": corpus.stage,
            "adamw_epochs": ADAMW_EPOCHS,
            "adamw_boundaries": list(ADAMW_BOUNDARIES),
            "adamw_rates": list(ADAMW_RATES),
            "adamw_weight_decay": ADAMW_WEIGHT_DECAY,
            "gradient_clip_norm": GRADIENT_CLIP_NORM,
            "lbfgs_lr": LBFGS_LR,
            "lbfgs_max_iter": LBFGS_MAX_ITER,
            "lbfgs_max_eval": LBFGS_MAX_EVAL,
            "lbfgs_history_size": LBFGS_HISTORY_SIZE,
            "lbfgs_closure_calls": closure_calls,
            "reset_sample_weight": float(RESET_SAMPLE_WEIGHT),
            "nonreset_sample_weight": float(NONRESET_SAMPLE_WEIGHT),
            "reset_row_count": int(np.count_nonzero(corpus.step_indices == 1)),
            "terminal_objective": float(final_objective.detach().cpu().item()),
            "terminal_mse": float(final_mse.detach().cpu().item()),
            "terminal_reset_mse": float(final_reset_mse.detach().cpu().item()),
            "input_mask_gradient_hook": True,
            "input_mask_recanonicalized_after_each_update": True,
            "sweep": False,
            "retry": False,
            "replica": False,
            "early_stopping": False,
            "best_checkpoint_selection": False,
            "critic_updates": 0,
            "ppo_updates": 0,
            "elapsed_seconds": float(time.monotonic() - started),
        }
        return FitResult(
            candidate_state=candidate,
            predictions=runtime_prediction,
            metrics=metrics,
            state_audit=state_audit,
            normalization=normalization,
            fold_audit=fold_audit,
            optimizer_audit=optimizer_audit,
            history=tuple(history),
        )
    except V12R12MaskedTeacherFitError:
        raise
    except Exception as exc:
        raise V12R12MaskedTeacherFitError("fixed V12R12 fit failed") from exc
    finally:
        torch.use_deterministic_algorithms(previous_deterministic)
        torch.set_num_threads(previous_threads)


def _new_w256_module(
    *, source_module: Any, state: Mapping[str, Any] | None, inference_only: bool
) -> Any:
    config = dict(source_module.model_config)
    config.update(
        {
            "fcnet_hiddens": list(HIDDEN_DIMS),
            "fcnet_activation": "tanh",
            "n_actor": INPUT_DIM,
            "n_full": FULL_OBSERVATION_DIM,
            "freeze_actor": bool(inference_only),
            "freeze_logstd": True,
        }
    )
    module = AsymmetricActorCriticTorchRLModule(
        observation_space=source_module.observation_space,
        action_space=source_module.action_space,
        inference_only=inference_only,
        learner_only=False,
        model_config=config,
        catalog_class=None,
    )
    if state is not None:
        module.set_state(state)
    module.eval()
    return module


def _runtime_logits(module: Any, observations: np.ndarray) -> np.ndarray:
    import torch
    from ray.rllib.core.columns import Columns

    values = torch.as_tensor(observations, dtype=torch.float32)
    with torch.no_grad():
        output = module.forward_inference({Columns.OBS: values})
    result = np.ascontiguousarray(
        output[Columns.ACTION_DIST_INPUTS].detach().cpu().numpy(), dtype=np.float32
    )
    if result.shape != (len(observations), LOGITS_DIM):
        raise V12R12MaskedTeacherFitError("runtime logits shape drifted")
    return result


def _save_rlmodule_no_clobber(module: Any, destination: Path) -> Any:
    from ray.rllib.core.rl_module.rl_module import RLModule

    output = _resolve(destination)
    _reject_link_components(output, include_leaf=True)
    if not output.parent.is_dir() or _is_link_or_reparse(output.parent):
        raise V12R12MaskedTeacherFitError("candidate parent is unsafe")
    if os.path.lexists(output):
        raise V12R12MaskedTeacherFitError("candidate destination exists/no-clobber")
    try:
        os.mkdir(output, 0o700)
    except OSError as exc:
        raise V12R12MaskedTeacherFitError("cannot exclusively claim candidate") from exc
    module.save_to_path(output)
    stray_entries = sorted(path.name for path in output.iterdir() if not path.is_file())
    if stray_entries:
        raise V12R12MaskedTeacherFitError(
            f"candidate tree contains non-file entries: {stray_entries}"
        )
    core_files = {path.name for path in output.iterdir() if path.is_file()}
    if core_files != {"class_and_ctor_args.pkl", "metadata.json", "module_state.pkl"}:
        raise V12R12MaskedTeacherFitError(
            f"candidate core file set drifted: {sorted(core_files)}"
        )
    reloaded = RLModule.from_checkpoint(output)
    reloaded.eval()
    return reloaded


def _actor_manifest(
    *, state: Mapping[str, Any], module_state_path: Path, stage: str
) -> dict[str, Any]:
    return {
        "schema_version": 1,
        "status": "H0_V12R12_ACTOR_FEATURE_BUILD_EVIDENCE",
        "terminal_gate_pending_at_write": True,
        "protocol_id": PROTOCOL_ID,
        "fit_contract_id": FIT_CONTRACT_ID,
        "fit_stage": stage,
        "topology_id": TOPOLOGY_ID,
        "actor_feature_count": INPUT_DIM,
        "actor_feature_names": list(ACTOR_FEATURE_NAMES),
        "fcnet_hiddens": list(HIDDEN_DIMS),
        "fcnet_activation": "tanh",
        "masked_input_columns": list(MASKED_COLUMNS),
        "masked_input_features": list(MASKED_FEATURE_NAMES),
        "active_input_columns": list(ACTIVE_COLUMNS),
        "actor_digest": warm_start.actor_state_digest(state),
        "state_digest": _state_digest(state),
        "module_state_sha256": _sha256_file(module_state_path),
        "standard_rlmodule": True,
        "legacy_shadow_runtime_dependency": False,
    }


def _build_manifest(
    *,
    state: Mapping[str, Any],
    actor_manifest: Mapping[str, Any],
    corpus: LockedCorpus,
    normalization: MaskedNormalization,
) -> dict[str, Any]:
    return {
        "schema_version": SCHEMA_VERSION,
        "status": "H0_V12R12_CANDIDATE_BUILD_EVIDENCE",
        "terminal_gate_pending_at_write": True,
        "protocol_id": PROTOCOL_ID,
        "fit_contract_id": FIT_CONTRACT_ID,
        "fit_stage": corpus.stage,
        "topology_id": TOPOLOGY_ID,
        "architecture": {
            "input_dim": INPUT_DIM,
            "hidden_dims": list(HIDDEN_DIMS),
            "logits_dim": LOGITS_DIM,
            "activation": "tanh",
            "standard_rlmodule": True,
        },
        "input_contract": {
            "mask_implemented_as_first_layer_positive_zero_columns": True,
            "masked_columns": list(MASKED_COLUMNS),
            "active_columns": list(ACTIVE_COLUMNS),
            "only_invariant_columns_active": True,
        },
        "source_h0_initialization": copy.deepcopy(SOURCE_H0_TREE),
        "safe_teacher_corpus": copy.deepcopy(dict(corpus.audit)),
        "normalization": normalization.record(),
        "actor_fit_count": ACTOR_FIT_COUNT,
        "actor_updates": ACTOR_UPDATE_COUNT,
        "offline_teacher_queries": OFFLINE_TEACHER_QUERY_COUNT,
        "environment_resets": ENVIRONMENT_RESET_COUNT,
        "environment_steps": ENVIRONMENT_STEP_COUNT,
        "policy_rollouts": POLICY_ROLLOUT_COUNT,
        "critic_updates": CRITIC_UPDATE_COUNT,
        "ppo_updates": PPO_UPDATE_COUNT,
        "logstd_sigma": float(TEACHER_SIGMA),
        "logstd_source_byte_exact": True,
        "actor_digest": actor_manifest["actor_digest"],
        "candidate_state_digest": _state_digest(state),
        "no_teacher_runtime_dependency": True,
        "no_legacy_shadow_runtime_dependency": True,
        "collector_only": corpus.stage == "p0",
        "non_promotable": corpus.stage == "p0",
    }


def save_candidate_and_manifests(
    *,
    source_module: Any,
    candidate_state: Mapping[str, Any],
    destination: Path,
    corpus: LockedCorpus,
    normalization: MaskedNormalization,
) -> tuple[dict[str, Any], dict[str, Any], dict[str, Any]]:
    module = _new_w256_module(
        source_module=source_module, state=candidate_state, inference_only=True
    )
    reloaded = _save_rlmodule_no_clobber(module, destination)
    reloaded_state = _state_arrays(reloaded.get_state())
    if not _state_byte_exact(candidate_state, reloaded_state):
        raise V12R12MaskedTeacherFitError("candidate save/reload state drifted")
    actor_manifest = _actor_manifest(
        state=reloaded_state,
        module_state_path=destination / "module_state.pkl",
        stage=corpus.stage,
    )
    _write_json_exclusive(destination / ACTOR_FEATURE_MANIFEST_NAME, actor_manifest)
    build_manifest = _build_manifest(
        state=reloaded_state,
        actor_manifest=actor_manifest,
        corpus=corpus,
        normalization=normalization,
    )
    _write_json_exclusive(destination / CANDIDATE_BUILD_MANIFEST_NAME, build_manifest)
    expected_files = {
        "class_and_ctor_args.pkl",
        "metadata.json",
        "module_state.pkl",
        ACTOR_FEATURE_MANIFEST_NAME,
        CANDIDATE_BUILD_MANIFEST_NAME,
    }
    stray_entries = sorted(
        path.name for path in destination.iterdir() if not path.is_file()
    )
    if stray_entries:
        raise V12R12MaskedTeacherFitError(
            f"candidate tree contains non-file entries: {stray_entries}"
        )
    observed_files = {path.name for path in destination.iterdir() if path.is_file()}
    if observed_files != expected_files:
        raise V12R12MaskedTeacherFitError(
            f"candidate five-file schema drifted: {sorted(observed_files)}"
        )
    return _tree_record(destination), actor_manifest, build_manifest


def runtime_and_transplant_audit(
    *,
    candidate_path: Path,
    intended_state: Mapping[str, Any],
    source_module: Any,
    observations: np.ndarray,
) -> dict[str, Any]:
    """Prove real reload plus actor-only transplant into a fresh critic."""

    import torch
    from ray.rllib.core.rl_module.rl_module import RLModule

    candidate = RLModule.from_checkpoint(_resolve(candidate_path))
    candidate.eval()
    checkpoint_state = _state_arrays(candidate.get_state())
    expected = _state_logits(intended_state, observations)
    direct = _runtime_logits(candidate, observations)
    padded = np.zeros((len(observations), FULL_OBSERVATION_DIM), dtype=np.float32)
    padded[:, :INPUT_DIM] = observations
    full = _runtime_logits(candidate, padded)
    with tempfile.TemporaryDirectory(prefix="h0_v12r12_roundtrip_") as temporary:
        roundtrip_path = Path(temporary) / "rl_module"
        candidate.save_to_path(roundtrip_path)
        roundtrip = RLModule.from_checkpoint(roundtrip_path)
        roundtrip.eval()
        roundtrip_state = _state_arrays(roundtrip.get_state())
        roundtrip_logits = _runtime_logits(roundtrip, observations)
    torch.manual_seed(FRESH_CRITIC_SEED)
    fresh = _new_w256_module(
        source_module=source_module, state=None, inference_only=False
    )
    fresh_before = _state_arrays(fresh.get_state())
    transplanted, report = warm_start.transplant_actor_state(
        target_state=fresh_before,
        target_actor_feature_names=ACTOR_FEATURE_NAMES,
        source_checkpoint=candidate_path,
        source_actor_feature_manifest=candidate_path / ACTOR_FEATURE_MANIFEST_NAME,
        mode="drop",
        zero_target_features=MASKED_FEATURE_NAMES,
    )
    actor_compare = warm_start.compare_actor_states(intended_state, transplanted)
    critic_compare = warm_start.compare_non_actor_states(fresh_before, transplanted)
    fresh.set_state(transplanted)
    transplanted_logits = _runtime_logits(fresh, padded)
    checks = {
        "standard_w256_module": type(candidate) is AsymmetricActorCriticTorchRLModule
        and list(candidate.model_config.get("fcnet_hiddens", ())) == list(HIDDEN_DIMS)
        and str(candidate.model_config.get("fcnet_activation", "")).lower() == "tanh",
        "candidate_actor_only": set(checkpoint_state) == STATE_KEYS,
        "checkpoint_state_byte_exact": _state_byte_exact(
            intended_state, checkpoint_state
        ),
        "runtime_direct_exact": _bytes_equal(expected, direct),
        "runtime_full_prefix_exact": _bytes_equal(expected, full),
        "roundtrip_state_byte_exact": _state_byte_exact(
            checkpoint_state, roundtrip_state
        ),
        "roundtrip_logits_byte_exact": _bytes_equal(direct, roundtrip_logits),
        "warm_start_actor_exact": actor_compare["exact"],
        "warm_start_fresh_critic_preserved": critic_compare["exact"],
        "warm_start_forward_exact": _bytes_equal(expected, transplanted_logits),
        "fresh_target_freeze_actor_false": fresh.model_config.get("freeze_actor")
        is False,
        "fresh_target_freeze_logstd_true": fresh.model_config.get("freeze_logstd")
        is True,
        "source_actor_only": report["source_state_is_actor_only"] is True,
        "masked_features_rezeroed": report["shared_features_zeroed"]
        == list(MASKED_FEATURE_NAMES),
        "no_runtime_teacher": True,
        "no_optimizer_state_imported": True,
    }
    return {
        "passed": all(checks.values()),
        "checks": checks,
        "actor_compare": actor_compare,
        "critic_compare": critic_compare,
        "transplant_report": report,
        "runtime_logits_sha256": array_sha256(expected),
        "optimizer_invocations": 0,
        "temporary_roundtrip_removed": True,
    }


def _stage_spec(stage: str) -> dict[str, Any]:
    if stage not in STAGE_OUTPUTS:
        raise V12R12MaskedTeacherFitError(f"unsupported fit stage: {stage!r}")
    return {
        "stage": stage,
        "output": STAGE_OUTPUTS[stage],
        "candidate_dirname": CANDIDATE_DIRNAMES[stage],
        "row_count": P0_ROW_COUNT if stage == "p0" else P2_ROW_COUNT,
        "trajectory_count": 6 if stage == "p0" else 24,
        "collector_only": stage == "p0",
        "non_promotable": stage == "p0",
        "next_stage": STAGE_NEXT[stage],
    }


def _protocol_design() -> dict[str, Any]:
    return {
        "protocol_id": PROTOCOL_ID,
        "fit_contract_id": FIT_CONTRACT_ID,
        "topology_id": TOPOLOGY_ID,
        "source_h0_id": SOURCE_H0_ID,
        "source_corpus_id": SOURCE_CORPUS_ID,
        "architecture": {
            "input_dim": INPUT_DIM,
            "hidden_dims": list(HIDDEN_DIMS),
            "logits_dim": LOGITS_DIM,
            "activation": "tanh",
        },
        "input_mask": {
            "masked_columns": list(MASKED_COLUMNS),
            "masked_features": list(MASKED_FEATURE_NAMES),
            "active_columns": list(ACTIVE_COLUMNS),
            "first_layer_mask_persists_at_runtime": True,
        },
        "stages": {
            stage: {
                "output": _logical_path(spec["output"]),
                "candidate_dirname": spec["candidate_dirname"],
                "row_count": spec["row_count"],
                "trajectory_count": spec["trajectory_count"],
                "collector_only": spec["collector_only"],
                "non_promotable": spec["non_promotable"],
                "next_stage": spec["next_stage"],
                "initialization": "same_byte_locked_h0_not_prior_candidate",
                "normalization_scope": "entire_stage_corpus_active18",
            }
            for stage in ("p0", "p2")
            for spec in (_stage_spec(stage),)
        },
        "fit": {
            "single_fixed_fit_per_stage": True,
            "seed": SEED,
            "adamw_epochs": ADAMW_EPOCHS,
            "adamw_boundaries": list(ADAMW_BOUNDARIES),
            "adamw_rates": list(ADAMW_RATES),
            "adamw_weight_decay": ADAMW_WEIGHT_DECAY,
            "lbfgs_lr": LBFGS_LR,
            "lbfgs_max_iter": LBFGS_MAX_ITER,
            "lbfgs_max_eval": LBFGS_MAX_EVAL,
            "objective": "reset_weighted_full_batch_mean_squared_error_only",
            "reset_sample_weight": float(RESET_SAMPLE_WEIGHT),
            "nonreset_sample_weight": float(NONRESET_SAMPLE_WEIGHT),
            "sweep": False,
            "retry": False,
            "replica": False,
        },
        "gates": {
            "global_rmse": GLOBAL_RMSE_LIMIT,
            "global_max_abs_error": GLOBAL_MAX_ABS_LIMIT,
            "per_case_and_action_rmse": PER_CASE_ACTION_RMSE_LIMIT,
            "per_trajectory_and_action_rmse": PER_TRAJECTORY_ACTION_RMSE_LIMIT,
            "reset_max_abs_error": RESET_MAX_ABS_LIMIT,
            "transition_window_max_abs_error": TRANSITION_MAX_ABS_LIMIT,
            "temporal_first_difference_max_abs_error": (
                TEMPORAL_FIRST_DIFFERENCE_MAX_ABS_LIMIT
            ),
            "predicted_mean_max_abs_strict": PREDICTED_MEAN_ABS_LIMIT,
            "save_reload_exact": True,
            "transplant_actor_exact": True,
            "fresh_critic_preserved": True,
            "no_optimizer_state_imported": True,
        },
        "counts": {
            "actor_fits": 1,
            "actor_updates": 1,
            "offline_teacher_queries": 0,
            "environment_resets": 0,
            "environment_steps": 0,
            "policy_rollouts": 0,
            "critic_updates": 0,
            "ppo_updates": 0,
        },
    }


def _governance_source_records() -> dict[str, dict[str, Any]]:
    return {
        "builder": _record(Path(__file__)),
        "package": _record(REVISION_ROOT / "__init__.py"),
        "runner": _record(REVISION_ROOT / "run_h0_v12r12_masked_teacher_fit.py"),
        "tests": _record(REVISION_ROOT / "test_h0_v12r12_masked_teacher_fitter.py"),
    }


def protocol_freeze_payload() -> dict[str, Any]:
    design = _protocol_design()
    return {
        "schema_version": SCHEMA_VERSION,
        "status": "FROZEN_H0_V12R12_MASKED_TEACHER_FIT_PROTOCOL",
        "passed": True,
        "protocol_id": PROTOCOL_ID,
        "fit_contract_id": FIT_CONTRACT_ID,
        "authorized_stages": ["p0", "p2"],
        "design": design,
        "design_sha256": _json_digest(design),
        "production_source_records": _governance_source_records(),
        "production_fit_executed": False,
    }


def write_protocol_freeze() -> dict[str, Any]:
    payload = protocol_freeze_payload()
    path = _write_json_exclusive(PROTOCOL_FREEZE_PATH, payload)
    return {**payload, "artifact": _record(path)}


def _validate_protocol_freeze() -> dict[str, Any]:
    observed = _strict_json(PROTOCOL_FREEZE_PATH)
    expected = protocol_freeze_payload()
    if not _strict_equal(observed, expected):
        raise V12R12MaskedTeacherFitError(
            "protocol freeze does not match the current builder/runner/tests"
        )
    return _record(PROTOCOL_FREEZE_PATH)


def execution_lock_payload() -> dict[str, Any]:
    freeze_record = _validate_protocol_freeze()
    sources = _governance_source_records()
    return {
        "schema_version": SCHEMA_VERSION,
        "status": "LOCKED_H0_V12R12_MASKED_TEACHER_FIT_EXECUTION",
        "passed": True,
        "protocol_id": PROTOCOL_ID,
        "fit_contract_id": FIT_CONTRACT_ID,
        "authorized_stages": ["p0", "p2"],
        "protocol_freeze": freeze_record,
        "production_source_records": sources,
        "production_source_manifest_sha256": _json_digest(sources),
        "source_heads": {
            "h0_tree": copy.deepcopy(SOURCE_H0_TREE),
            "h0_training_config": copy.deepcopy(SOURCE_H0_CONFIG_RECORD),
            "v8r1p1_ledger": copy.deepcopy(V8R1P1_LEDGER_RECORD),
            "v8r1p1_execution_lock": copy.deepcopy(V8R1P1_EXECUTION_LOCK_RECORD),
        },
        "one_shot_per_stage": True,
        "production_fit_executed": False,
    }


def write_execution_lock() -> dict[str, Any]:
    payload = execution_lock_payload()
    path = _write_json_exclusive(EXECUTION_LOCK_PATH, payload)
    return {**payload, "artifact": _record(path)}


def verify_execution_governance(stage: str) -> dict[str, Any]:
    _stage_spec(stage)
    observed = _strict_json(EXECUTION_LOCK_PATH)
    expected = execution_lock_payload()
    if not _strict_equal(observed, expected):
        raise V12R12MaskedTeacherFitError(
            "execution lock does not match the frozen current sources"
        )
    if stage not in observed.get("authorized_stages", []):
        raise V12R12MaskedTeacherFitError(f"stage {stage} is not execution-locked")
    return {
        "protocol_freeze": _record(PROTOCOL_FREEZE_PATH),
        "execution_lock": _record(EXECUTION_LOCK_PATH),
        "production_source_records": _governance_source_records(),
    }


def describe_protocol(stage: str = "p0") -> dict[str, Any]:
    spec = _stage_spec(stage)
    return {
        "schema_version": SCHEMA_VERSION,
        "status": "H0_V12R12_MASKED_TEACHER_SOURCE_READY_UNEXECUTED",
        "passed": True,
        "fit_stage": stage,
        "candidate_output": _logical_path(spec["output"]),
        **_protocol_design(),
        "production_fit_executed": False,
    }


def preflight(stage: str = "p0") -> dict[str, Any]:
    spec = _stage_spec(stage)
    output = spec["output"]
    if os.path.lexists(output):
        raise V12R12MaskedTeacherFitError(
            f"canonical output already occupied/no-clobber: {output}"
        )
    corpus = load_fit_corpus(stage)
    normalization = build_masked_normalization(corpus.observations)
    _module, source_state = _load_source_module_and_state()
    source_after = _tree_record(SOURCE_H0_PATH)
    _assert_record_exact(source_after, SOURCE_H0_TREE)
    initial_model = _new_normalized_model(source_state, normalization)
    initial_state, initial_fold = _fold_model_into_state(
        initial_model, source_state, normalization
    )
    initial_audit = validate_standard_state(initial_state, source_state=source_state)
    initial_predictions = _state_logits(initial_state, corpus.observations)[
        :, :ACTION_DIM
    ]
    initial_metrics = prediction_metrics(
        initial_predictions,
        corpus.actions,
        corpus.case_ids,
        corpus.trajectory_ids,
        corpus.step_indices,
        corpus.transition_mask,
    )
    trainable_mean_parameter_count = (
        len(ACTIVE_COLUMNS) * HIDDEN_DIMS[0]
        + HIDDEN_DIMS[0]
        + HIDDEN_DIMS[0] * HIDDEN_DIMS[1]
        + HIDDEN_DIMS[1]
        + HIDDEN_DIMS[1] * ACTION_DIM
        + ACTION_DIM
    )
    checks = {
        "canonical_output_absent": not os.path.lexists(output),
        "locked_stage_rows_exact": corpus.audit["rows"] == spec["row_count"],
        "trajectory_count_exact": len(set(corpus.trajectory_ids.astype(str)))
        == spec["trajectory_count"],
        "only_18_invariant_columns_active": len(ACTIVE_COLUMNS) == 18
        and len(MASKED_COLUMNS) == 17,
        "transition_rows_present": corpus.audit["transition_window_rows"] > 0,
        "normalization_uses_all_stage_rows": normalization.source_rows
        == spec["row_count"],
        "normalization_mask_exact": normalization.record()["masked_means_positive_zero"]
        is True,
        "masked_h0_initialization_valid": initial_audit["passed"] is True,
        "initial_fold_mask_exact": initial_fold["masked_columns_positive_zero"] is True,
        "reinitialized_from_same_h0": True,
        "source_h0_unchanged": source_after == SOURCE_H0_TREE,
        "execution_governance_not_required_for_preflight": True,
        "no_fit_executed": True,
        "no_candidate_created": True,
    }
    return {
        "schema_version": SCHEMA_VERSION,
        "status": (
            f"PASS_H0_V12R12_{stage.upper()}_MASKED_TEACHER_PREFLIGHT"
            if all(checks.values())
            else f"FAIL_H0_V12R12_{stage.upper()}_MASKED_TEACHER_PREFLIGHT"
        ),
        "passed": all(checks.values()),
        "fit_stage": stage,
        "protocol": describe_protocol(stage),
        "checks": checks,
        "corpus_audit": copy.deepcopy(dict(corpus.audit)),
        "source_records": copy.deepcopy(dict(corpus.source_records)),
        "normalization": normalization.record(),
        "representability_evidence": {
            "trainable_mean_parameter_count": trainable_mean_parameter_count,
            "scalar_supervision_constraints": len(corpus.observations) * ACTION_DIM,
            "parameter_to_constraint_ratio": trainable_mean_parameter_count
            / (len(corpus.observations) * ACTION_DIM),
            "unique_active_inputs": corpus.audit.get("unique_active_rows"),
            "duplicate_active_inputs": corpus.audit.get("duplicate_active_rows"),
            "conflicting_duplicate_targets": corpus.audit.get(
                "conflicting_duplicate_targets"
            ),
            "masked_h0_initial_metrics": initial_metrics,
            "interpretation": (
                "This source-only preflight proves contract and initialization "
                "closure, not that the fixed fit will meet the frozen gates."
            ),
        },
        "production_fit_executed": False,
        "candidate_created": False,
    }


def _claim_output_dir(destination: Path) -> None:
    output = _resolve(destination)
    _reject_link_components(output, include_leaf=False)
    output.parent.mkdir(parents=True, exist_ok=True)
    if os.path.lexists(output):
        raise V12R12MaskedTeacherFitError(
            f"production destination occupied/no-clobber: {output}"
        )
    try:
        os.mkdir(output, 0o700)
    except OSError as exc:
        raise V12R12MaskedTeacherFitError(
            f"cannot exclusively claim production destination: {output}"
        ) from exc


def _write_failure_once(destination: Path, exc: BaseException, *, stage: str) -> None:
    failure = destination / "failure.json"
    if not destination.is_dir() or os.path.lexists(failure):
        return
    payload = {
        "schema_version": SCHEMA_VERSION,
        "status": "FAIL_H0_V12R12_MASKED_SAFE_TEACHER_FIT_TERMINAL",
        "passed": False,
        "protocol_id": PROTOCOL_ID,
        "fit_contract_id": FIT_CONTRACT_ID,
        "fit_stage": stage,
        "error_type": type(exc).__name__,
        "error": str(exc),
        "retry_authorized": False,
        "candidate_promoted": False,
        "critic_updates": 0,
        "ppo_updates": 0,
    }
    try:
        _write_json_exclusive(failure, payload)
    except Exception:
        pass


def run_production_fit(
    *,
    stage: str = "p0",
    destination: str | PurePath | Path | None = None,
    enforce_canonical: bool = True,
) -> dict[str, Any]:
    """Execute and publish the single irreversible fit; never overwrite/retry."""

    spec = _stage_spec(stage)
    output = _resolve(spec["output"] if destination is None else destination)
    if enforce_canonical and output != _resolve(spec["output"]):
        raise V12R12MaskedTeacherFitError("non-canonical production destination")
    if os.path.lexists(output):
        raise V12R12MaskedTeacherFitError(
            f"production destination occupied/no-clobber: {output}"
        )
    governance = verify_execution_governance(stage)
    corpus = load_fit_corpus(stage)
    if corpus.stage != stage or len(corpus.observations) != spec["row_count"]:
        raise V12R12MaskedTeacherFitError("fit-stage corpus binding drifted")
    normalization = build_masked_normalization(corpus.observations)
    source_module, source_state = _load_source_module_and_state()
    source_before = _tree_record(SOURCE_H0_PATH)
    _claim_output_dir(output)
    try:
        corpus_path = _write_npz_exclusive(output / "corpus.npz", corpus.arrays())
        result = fit_candidate_in_memory(
            source_state=source_state,
            corpus=corpus,
            normalization=normalization,
        )
        metric_gate = fit_gate(
            metrics=result.metrics,
            state_audit=result.state_audit,
            fold_audit=result.fold_audit,
        )
        if metric_gate["passed"] is not True:
            _write_json_exclusive(output / "gate.json", metric_gate)
            raise V12R12MaskedTeacherFitError(
                "single fixed fit failed strict offline metric/state gates"
            )
        candidate_path = output / spec["candidate_dirname"]
        module_record, actor_manifest, build_manifest = save_candidate_and_manifests(
            source_module=source_module,
            candidate_state=result.candidate_state,
            destination=candidate_path,
            corpus=corpus,
            normalization=normalization,
        )
        compatibility = runtime_and_transplant_audit(
            candidate_path=candidate_path,
            intended_state=result.candidate_state,
            source_module=source_module,
            observations=corpus.observations,
        )
        source_after = _tree_record(SOURCE_H0_PATH)
        governance_after = verify_execution_governance(stage)
        final_checks = {
            **dict(metric_gate["checks"]),
            "candidate_five_file_tree": module_record.get("file_count") == 5,
            "save_reload_transplant_ready": compatibility.get("passed") is True,
            "source_h0_unchanged": source_after == source_before == SOURCE_H0_TREE,
            "execution_governance_unchanged": _strict_equal(
                governance_after, governance
            ),
            "single_actor_fit": result.optimizer_audit.get("single_fixed_fit") is True,
            "no_sweep_retry_replica": result.optimizer_audit.get("sweep") is False
            and result.optimizer_audit.get("retry") is False
            and result.optimizer_audit.get("replica") is False,
            "no_critic_or_ppo_update": result.optimizer_audit.get("critic_updates") == 0
            and result.optimizer_audit.get("ppo_updates") == 0,
            "no_environment_or_policy_rollout": True,
        }
        final_gate = {
            "schema_version": SCHEMA_VERSION,
            "status": (
                (
                    "PASS_H0_V12R12_MASKED_SAFE_TEACHER_FIT"
                    if stage == "p0"
                    else "PASS_H0_V12R12_P2_MASKED_CANDIDATE_EXPOSED_FIT"
                )
                if all(final_checks.values())
                else (
                    "FAIL_H0_V12R12_MASKED_SAFE_TEACHER_FIT"
                    if stage == "p0"
                    else "FAIL_H0_V12R12_P2_MASKED_CANDIDATE_EXPOSED_FIT"
                )
            ),
            "passed": all(final_checks.values()),
            "protocol_id": PROTOCOL_ID,
            "fit_contract_id": FIT_CONTRACT_ID,
            "fit_stage": stage,
            "checks": final_checks,
            "limits": metric_gate["limits"],
        }
        if final_gate["passed"] is not True:
            _write_json_exclusive(output / "gate.json", final_gate)
            raise V12R12MaskedTeacherFitError(
                "candidate save/reload/transplant gate failed"
            )
        summary = {
            "schema_version": SCHEMA_VERSION,
            "status": (
                "COMPLETE_H0_V12R12_MASKED_SAFE_TEACHER_FIT"
                if stage == "p0"
                else "COMPLETE_H0_V12R12_P2_MASKED_CANDIDATE_EXPOSED_FIT"
            ),
            "passed": True,
            "protocol_id": PROTOCOL_ID,
            "fit_contract_id": FIT_CONTRACT_ID,
            "fit_stage": stage,
            "topology_id": TOPOLOGY_ID,
            "candidate_id": f"{PROTOCOL_ID}:{module_record['tree_sha256'][:16]}",
            "candidate_module": module_record,
            "candidate_state_digest": _state_digest(result.candidate_state),
            "candidate_predictions_sha256": array_sha256(result.predictions),
            "metrics": copy.deepcopy(dict(result.metrics)),
            "corpus": {
                "artifact": _record(corpus_path),
                "audit": copy.deepcopy(dict(corpus.audit)),
                "source_records": copy.deepcopy(dict(corpus.source_records)),
            },
            "normalization": normalization.record(),
            "state_audit": copy.deepcopy(dict(result.state_audit)),
            "fold_audit": copy.deepcopy(dict(result.fold_audit)),
            "optimizer_audit": copy.deepcopy(dict(result.optimizer_audit)),
            "history": copy.deepcopy(list(result.history)),
            "runtime_transplant_audit": compatibility,
            "actor_feature_manifest": actor_manifest,
            "candidate_build_manifest": build_manifest,
            "source_h0_before": source_before,
            "source_h0_after": source_after,
            "source_h0_initialization": "same_byte_locked_h0_not_prior_candidate",
            "execution_governance": copy.deepcopy(governance),
            "actor_fit_count": 1,
            "actor_updates": 1,
            "offline_teacher_queries": 0,
            "environment_resets": 0,
            "environment_steps": 0,
            "policy_rollouts": 0,
            "critic_updates": 0,
            "ppo_updates": 0,
            "candidate_promoted": False,
            "collector_only": spec["collector_only"],
            "non_promotable": spec["non_promotable"],
            "next_stage": spec["next_stage"],
        }
        summary_path = _write_json_exclusive(output / "summary.json", summary)
        gate_path = _write_json_exclusive(output / "gate.json", final_gate)
        receipt = {
            "schema_version": SCHEMA_VERSION,
            "status": (
                "PASS_H0_V12R12_MASKED_SAFE_TEACHER_FIT_RECEIPT"
                if stage == "p0"
                else "PASS_H0_V12R12_P2_MASKED_CANDIDATE_EXPOSED_FIT_RECEIPT"
            ),
            "passed": True,
            "protocol_id": PROTOCOL_ID,
            "fit_contract_id": FIT_CONTRACT_ID,
            "fit_stage": stage,
            "candidate_id": summary["candidate_id"],
            "artifacts": {
                "corpus": _record(corpus_path),
                "candidate_module": module_record,
                "summary": _record(summary_path),
                "gate": _record(gate_path),
            },
            "actor_fit_count": 1,
            "actor_updates": 1,
            "critic_updates": 0,
            "ppo_updates": 0,
            "environment_steps": 0,
            "policy_rollouts": 0,
            "retry_authorized": False,
            "candidate_promoted": False,
            "collector_only": spec["collector_only"],
            "non_promotable": spec["non_promotable"],
            "next_stage": spec["next_stage"],
            "execution_governance": copy.deepcopy(governance),
        }
        receipt_path = _write_json_exclusive(output / "receipt.json", receipt)
        return {
            **receipt,
            "receipt": _record(receipt_path),
        }
    except BaseException as exc:
        _write_failure_once(output, exc, stage=stage)
        if isinstance(exc, V12R12MaskedTeacherFitError):
            raise
        if not isinstance(exc, Exception):
            raise
        raise V12R12MaskedTeacherFitError("V12R12 production fit failed") from exc


__all__ = [
    "ACTIVE_COLUMNS",
    "ACTOR_FEATURE_NAMES",
    "ADAMW_EPOCHS",
    "ADAMW_BOUNDARIES",
    "ADAMW_RATES",
    "ADAMW_WEIGHT_DECAY",
    "CASE_IDS",
    "DEFAULT_OUTPUT",
    "EXPECTED_ARRAY_HASHES",
    "FIT_CONTRACT_ID",
    "FitResult",
    "GLOBAL_MAX_ABS_LIMIT",
    "GLOBAL_RMSE_LIMIT",
    "HIDDEN_DIMS",
    "INPUT_DIM",
    "LBFGS_MAX_EVAL",
    "LBFGS_MAX_ITER",
    "LockedCorpus",
    "MASKED_COLUMNS",
    "MASKED_FEATURE_NAMES",
    "MaskedNormalization",
    "PER_CASE_ACTION_RMSE_LIMIT",
    "PER_TRAJECTORY_ACTION_RMSE_LIMIT",
    "P1_ALPHAS",
    "P2_OUTPUT",
    "PREDICTED_MEAN_ABS_LIMIT",
    "PROTOCOL_ID",
    "PROTOCOL_FREEZE_PATH",
    "RESET_MAX_ABS_LIMIT",
    "SOURCE_H0_TREE",
    "TEMPORAL_FIRST_DIFFERENCE_MAX_ABS_LIMIT",
    "TOPOLOGY_ID",
    "TRANSITION_MAX_ABS_LIMIT",
    "V12R12MaskedTeacherFitError",
    "adamw_rate",
    "attest_locked_sources",
    "build_masked_normalization",
    "describe_protocol",
    "fit_candidate_in_memory",
    "fit_gate",
    "load_fit_corpus",
    "load_locked_corpus",
    "normalized_masked_observations",
    "prediction_metrics",
    "preflight",
    "protocol_freeze_payload",
    "run_production_fit",
    "runtime_and_transplant_audit",
    "save_candidate_and_manifests",
    "validate_standard_state",
    "verify_execution_governance",
    "write_execution_lock",
    "write_protocol_freeze",
]
