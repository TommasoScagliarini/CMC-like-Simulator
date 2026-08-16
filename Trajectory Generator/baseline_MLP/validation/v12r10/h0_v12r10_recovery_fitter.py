"""Production fitter for the single V12R10 W1024 recovery actor.

The fitter consumes the byte-locked terminal V12R9 corpus without querying a
teacher or opening an environment.  It creates one ordinary
``35 -> 1024 -> 1024 -> 2`` tanh actor composed of two isolated towers:

* tower A is the immutable V12R6 W512 actor;
* tower B starts from the terminal V12R9 hidden representation and a positive
  zero mean head, and is the only trainable tower;
* both cross-tower blocks, both disabled clock columns, and the complete
  log-standard-deviation head remain positive zero or byte-exact as required.

There is one production actor fit and one actor update.  The call pattern is
the frozen successful design: reset-x3 equal-stratum replay (AdamW 2500,
LBFGS 3000/4500), followed by the symmetric 15-group gate objective (AdamW
1500, LBFGS 3000/4500).  Both phase terminals are accepted only when their
closure counts, losses, state digests, and prediction digests exactly match
the preregistered values below.  There is no replica, retry, sweep, candidate
selection, critic update, PPO update, H0 query, rollout, or repair path.

Import is execution-free.  ``run_fit_stage`` is no-clobber.  The semantic
verifier rebuilds the corpus indices, weights, normalization, predictions,
metrics, state invariants, real RLModule reload, and actor warm-start while
proving that a freshly initialized critic is preserved; it never invokes an
optimizer.
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
from pathlib import Path, PurePath, PurePosixPath
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
LOCAL_VALIDATION = REPO_ROOT / "Trajectory Generator" / "baseline_MLP" / "validation"
REVISION_ROOT = Path(__file__).resolve().parent
for _root in (
    REPO_ROOT,
    REPO_ROOT / "validation",
    REPO_ROOT / "Trajectory Generator" / "baseline_MLP",
    LOCAL_VALIDATION,
    LOCAL_VALIDATION / "v12r6",
    LOCAL_VALIDATION / "v12r9",
    REVISION_ROOT,
):
    if str(_root) not in sys.path:
        sys.path.insert(0, str(_root))

import h0_v12r9_recovery_fitter as r9  # noqa: E402
import h0_v12r10_recovery_contract as contract  # noqa: E402
import warm_start  # noqa: E402
from asymmetric_rl_module import (  # noqa: E402
    AsymmetricActorCriticTorchRLModule,
)


class V12R10RecoveryFitError(RuntimeError):
    """Raised when a locked input, fit terminal, or publication drifts."""


FrozenNormalization = r9.v11.FrozenNormalization

TOPOLOGY_ID = contract.TOPOLOGY_ID
ACTOR_FEATURE_MANIFEST_NAME = "actor_feature_manifest.json"
CANDIDATE_BUILD_MANIFEST_NAME = "candidate_build_manifest.json"
FIT_COMPLETE_STATUS = "COMPLETE_H0_V12R10_RECOVERY_FIT"
FIT_PASS_STATUS = "PASS_H0_V12R10_RECOVERY_FIT_RECEIPT"
CANDIDATE_BUILD_STATUS = "H0_V12R10_RECOVERY_CANDIDATE_BUILD_PASS"

INPUT_WIDTH = 35
ACTION_DIM = 2
TOWER_WIDTH = 512
TARGET_WIDTH = 1024
SEED = 20260814
TORCH_THREADS = 5
RESET_MULTIPLIER = 3.0
STRATUM_MASS = 500.0
STRATUM_COUNT = 13

# Phase A: exact reset-x3 equal-stratum replay.
UNIFORM_ADAMW_EPOCHS = 2_500
UNIFORM_ADAMW_BOUNDARIES = (1_000, 2_000, 2_500)
UNIFORM_ADAMW_RATES = (3.0e-4, 1.0e-4, 3.0e-5)
UNIFORM_ADAMW_WEIGHT_DECAY = 1.0e-7
UNIFORM_GRADIENT_CLIP_NORM = 10.0
UNIFORM_LBFGS_LR = 0.7
UNIFORM_LBFGS_MAX_ITER = 3_000
UNIFORM_LBFGS_MAX_EVAL = 4_500
UNIFORM_LBFGS_HISTORY_SIZE = 50
UNIFORM_LBFGS_TOLERANCE_GRAD = 1.0e-10
UNIFORM_LBFGS_TOLERANCE_CHANGE = 1.0e-12
EXPECTED_UNIFORM_CLOSURES = 3_072
EXPECTED_UNIFORM_TERMINAL_LOSS = 3.0605827798686175e-05
EXPECTED_UNIFORM_STATE_DIGEST = (
    "52aee29da6db7535e8fcfe14f66fa1c7eaa0b95a0ac74fbf5c73b25a1c8fe167"
)
EXPECTED_UNIFORM_PREDICTION_DIGEST = (
    "2aa5c64704163b949525a28a4fd9d3e6688e6f2973fb4fa137fa7ff01f4da48d"
)

# Phase B: symmetric objective derived only from the unchanged gates.
RMSE_LIMIT = 0.006
MAX_ABS_LIMIT = 0.060
RESET_MAX_ABS_LIMIT = 0.003
SMOOTH_MAX_TEMPERATURE = 0.05
SAFETY_MARGIN_FRACTION = 0.90
LOSS_COEFFICIENTS = {
    "reset3_mse_preservation": 1.0,
    "worst_group_rmse": 1.0,
    "global_tail_max_abs": 1.0,
    "reset_max_abs": 1.0,
}
GATE_ADAMW_EPOCHS = 1_500
GATE_ADAMW_BOUNDARIES = (500, 1_000, 1_500)
GATE_ADAMW_RATES = (3.0e-5, 1.0e-5, 3.0e-6)
GATE_ADAMW_WEIGHT_DECAY = 1.0e-7
GATE_GRADIENT_CLIP_NORM = 10.0
GATE_LBFGS_LR = 0.5
GATE_LBFGS_MAX_ITER = 3_000
GATE_LBFGS_MAX_EVAL = 4_500
GATE_LBFGS_HISTORY_SIZE = 50
GATE_LBFGS_TOLERANCE_GRAD = 1.0e-10
GATE_LBFGS_TOLERANCE_CHANGE = 1.0e-12
EXPECTED_GATE_CLOSURES = 3_020
EXPECTED_GATE_TERMINAL_LOSS = 0.7536344049605196
EXPECTED_FINAL_STATE_DIGEST = (
    "a0ad3cd981162879069c1317f7973b74b4bbcafa9e244c3b6180da34b17b24c0"
)
EXPECTED_FINAL_PREDICTION_DIGEST = (
    "684ec03d2a9e5c3bc37ad98cad943754d394500d198c0efc6b0ff4eb10256cf0"
)

FOLD_NUMERICAL_TOLERANCE = 2.0e-6
OFFLINE_H0_QUERY_COUNT = 0
ENVIRONMENT_RESET_COUNT = 0
ENVIRONMENT_STEP_COUNT = 0
CRITIC_UPDATE_COUNT = 0
PPO_UPDATE_COUNT = 0

R9_CORPUS_RECORD = {
    "path": (
        "Trajectory Generator/baseline_MLP/validation/v12r9/"
        "h0_v12r9_run_20260814/fit/corpus.npz"
    ),
    "sha256": "1b35d0789d11a0f3bca3cae15c5877ceaf68845bf69ed7231d5a4ecc4d5b9dfe",
    "size_bytes": 32_561_707,
}
R9_TERMINAL_LEDGER_RECORD = {
    "path": (
        "Trajectory Generator/baseline_MLP/validation/v12r9/"
        "h0_v12r9_run_20260814/pipeline_ledger.json"
    ),
    "sha256": "591fdc6ebe6e2a553b24ff210232a0f16185c4bc7b5e33104e34b780a0a18785",
    "size_bytes": 8_787,
}
R9_FIT_GATE_RECORD = {
    "path": (
        "Trajectory Generator/baseline_MLP/validation/v12r9/"
        "h0_v12r9_run_20260814/fit/gate.json"
    ),
    "sha256": "fe69de6cc8f2ae83bb5838104dd658353a897c95c7ed326a2ec5b605e3af9f2b",
    "size_bytes": 398,
}
R9_FIT_SUMMARY_RECORD = {
    "path": (
        "Trajectory Generator/baseline_MLP/validation/v12r9/"
        "h0_v12r9_run_20260814/fit/summary.json"
    ),
    "sha256": "88d8318e98e359061d6bb6cbc9cb0f15fffe4af3237474bcbc61a9f2b3e7f44b",
    "size_bytes": 7_424,
}
R9_CANDIDATE_TREE = {
    "path": (
        "Trajectory Generator/baseline_MLP/validation/v12r9/"
        "h0_v12r9_run_20260814/fit/rl_module_recovery"
    ),
    "tree_sha256": "8bc8554c573f8224ea3fa9d8682d315f9ff30c4aaa1557c4974e4fa422b5d1ff",
    "file_count": 5,
    "files": [
        {
            "path": "actor_feature_manifest.json",
            "sha256": "14479f424b46300fa45f5b453f6ec95d182717327e7c5e456bf6492c2f672724",
            "size_bytes": 1_627,
        },
        {
            "path": "candidate_build_manifest.json",
            "sha256": "40bf0bbb017eb05aaab4c7a594031a2663ee281182343922de1cb227ad126dcd",
            "size_bytes": 2_336,
        },
        {
            "path": "class_and_ctor_args.pkl",
            "sha256": "a5cc979046fbb4dde936d4d3f2d913e16448e8cea5a8f03762f60145dbe674b0",
            "size_bytes": 2_262,
        },
        {
            "path": "metadata.json",
            "sha256": "3a032ba54abcee8c9bcbb39e72fa05566912e94461d01f3c6228dc60e088bf12",
            "size_bytes": 197,
        },
        {
            "path": "module_state.pkl",
            "sha256": "6d0d319b8c1de66adc45116319fba38abfe15890c207545bec05653760d0d0b3",
            "size_bytes": 2_257_526,
        },
    ],
}
R6_CANDIDATE_TREE = copy.deepcopy(r9.contract.FULL_R6_CANDIDATE_TREE)

R9_CORPUS_KEYS = {
    "observations",
    "actions",
    "reset_mask",
    "actor_feature_names",
    "case_ids",
    "step_indices",
    "tranche_ids",
    "origins",
    "episode_ids",
    "raw_sample_weights",
    "normalized_sample_weights",
    "training_indices",
    "stratum_ids",
}
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
EXPECTED_CANDIDATE_FILES = {
    ACTOR_FEATURE_MANIFEST_NAME,
    CANDIDATE_BUILD_MANIFEST_NAME,
    "class_and_ctor_args.pkl",
    "metadata.json",
    "module_state.pkl",
}

TRANSITION_ALIAS_LIMITATION = {
    "id": "LEGACY_TEACHER_TRANSITION_HISTORY_HIDDEN",
    "description": (
        "The frozen legacy teacher changes action at a timeout transition whose "
        "history is not fully observable in the deployed V26 actor features."
    ),
    "mitigation": (
        "The symmetric gate-aligned offline fit bounds the frozen corpus error; "
        "physical development and independent Q3 must still validate deployment."
    ),
    "resolved_by_offline_fit": False,
    "legacy_shadow_runtime_dependency": False,
}


@dataclass(frozen=True)
class RecoveryCorpusBundle:
    """Byte-locked R9 corpus with deterministic R10 derived views."""

    arrays: Mapping[str, np.ndarray]
    sample_weights: np.ndarray
    weight_audit: Mapping[str, Any]
    normalization: FrozenNormalization
    base_indices: Mapping[str, np.ndarray]
    r4_indices: np.ndarray
    observer_indices: Mapping[str, np.ndarray]
    observer_plus_late_indices: np.ndarray
    source_records: Mapping[str, Any]


@dataclass(frozen=True)
class RecoveryFitResult:
    """In-memory terminal of the sole production actor fit."""

    candidate_state: Mapping[str, np.ndarray]
    predictions: np.ndarray
    metrics: Mapping[str, Any]
    uniform_attestation: Mapping[str, Any]
    final_attestation: Mapping[str, Any]
    state_audit: Mapping[str, Any]
    tower_isolation: Mapping[str, Any]
    normalization_audit: Mapping[str, Any]
    optimizer_audit: Mapping[str, Any]
    history: tuple[Mapping[str, Any], ...]


def _absolute_no_follow(path: str | PurePath | Path) -> Path:
    raw = Path(path)
    if not raw.is_absolute():
        raw = REPO_ROOT / raw
    return Path(os.path.abspath(raw))


def _resolve(path: str | PurePath | Path) -> Path:
    resolved = _absolute_no_follow(path)
    root = _absolute_no_follow(REPO_ROOT)
    try:
        resolved.relative_to(root)
    except ValueError as exc:
        raise V12R10RecoveryFitError(f"path escapes repository root: {path}") from exc
    return resolved


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
        raise V12R10RecoveryFitError(f"path escapes repository root: {path}") from exc
    limit = len(parts) if include_leaf else max(0, len(parts) - 1)
    current = root
    for part in parts[:limit]:
        current /= part
        if os.path.lexists(current) and _is_link_or_reparse(current):
            raise V12R10RecoveryFitError(
                f"unsafe symlink/junction path component: {current}"
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
        raise V12R10RecoveryFitError(f"unsafe or missing artifact: {resolved}")
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
        raise V12R10RecoveryFitError(f"unsafe or missing tree: {root}") from exc
    if not stat.S_ISDIR(root_status.st_mode) or _is_link_or_reparse(root):
        raise V12R10RecoveryFitError(f"unsafe or missing tree: {root}")
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
                raise V12R10RecoveryFitError(f"unsafe tree directory: {child}")
        for name in file_names:
            child = current / name
            if not _regular_file(child) or _is_link_or_reparse(child):
                raise V12R10RecoveryFitError(f"unsafe tree file: {child}")
            files.append(child)
    if not files:
        raise V12R10RecoveryFitError(f"empty artifact tree: {root}")
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
    return {
        "path": _logical_path(root),
        "tree_sha256": digest.hexdigest(),
        "file_count": len(rows),
        "files": rows,
    }


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


def _strict_json(path: str | PurePath | Path) -> dict[str, Any]:
    resolved = _resolve(path)
    try:
        value = json.loads(
            resolved.read_text(encoding="utf-8"),
            parse_constant=lambda token: (_ for _ in ()).throw(
                ValueError(f"non-finite JSON constant: {token}")
            ),
        )
    except Exception as exc:
        raise V12R10RecoveryFitError(f"invalid strict JSON: {resolved}") from exc
    if not isinstance(value, dict):
        raise V12R10RecoveryFitError(f"expected JSON object: {resolved}")
    return value


def _write_json_exclusive(path: Path, value: Any) -> None:
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
        raise V12R10RecoveryFitError(
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


def _array(value: Any) -> np.ndarray:
    if hasattr(value, "detach"):
        value = value.detach()
    if hasattr(value, "cpu"):
        value = value.cpu()
    return np.ascontiguousarray(np.asarray(value))


def _state_arrays(state: Mapping[str, Any]) -> dict[str, np.ndarray]:
    return {name: _array(value) for name, value in state.items()}


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


def _prediction_digest(value: Any) -> str:
    array = _array(value)
    digest = hashlib.sha256()
    digest.update(array.dtype.str.encode("ascii"))
    digest.update(repr(array.shape).encode("ascii"))
    digest.update(array.tobytes(order="C"))
    return digest.hexdigest()


def _expected_shapes(width: int) -> dict[str, tuple[int, ...]]:
    return {
        "pi_encoder.0.weight": (width, INPUT_WIDTH),
        "pi_encoder.0.bias": (width,),
        "pi_encoder.2.weight": (width, width),
        "pi_encoder.2.bias": (width,),
        "pi.0.0.weight": (width, INPUT_WIDTH),
        "pi.0.0.bias": (width,),
        "pi.0.2.weight": (width, width),
        "pi.0.2.bias": (width,),
        "pi.1.weight": (2 * ACTION_DIM, width),
        "pi.1.bias": (2 * ACTION_DIM,),
    }


def validate_w1024_state(state: Mapping[str, Any]) -> dict[str, Any]:
    """Validate the deployable standard W1024 actor state."""

    arrays = _state_arrays(state)
    expected = _expected_shapes(TARGET_WIDTH)
    checks = {
        "key_set_exact": set(arrays) == STATE_KEYS,
        "shapes_exact": set(arrays) == set(expected)
        and all(arrays[name].shape == shape for name, shape in expected.items()),
        "float32_finite": all(
            value.dtype == np.float32 and np.all(np.isfinite(value))
            for value in arrays.values()
        ),
        "encoder_aliases_byte_exact": all(
            _bytes_equal(arrays[left], arrays[right]) for left, right in ALIASES
        ),
        "clock_columns_positive_zero": _positive_zero(
            arrays["pi_encoder.0.weight"][:, contract.DISABLED_CLOCK_COLUMNS]
        ),
        "logstd_weight_positive_zero": _positive_zero(
            arrays["pi.1.weight"][ACTION_DIM:]
        ),
    }
    if not all(checks.values()):
        failed = sorted(name for name, passed in checks.items() if not passed)
        raise V12R10RecoveryFitError(f"W1024 state invariant failed: {failed}")
    return {
        "passed": True,
        "checks": checks,
        "hidden_dims": [TARGET_WIDTH, TARGET_WIDTH],
        "actor_feature_count": INPUT_WIDTH,
        "actor_digest": warm_start.actor_state_digest(arrays),
        "state_digest": _state_digest(arrays),
    }


def _assert_record_exact(
    observed: Mapping[str, Any], expected: Mapping[str, Any]
) -> None:
    if not _strict_equal(dict(observed), dict(expected)):
        raise V12R10RecoveryFitError(f"locked input drifted: {expected.get('path')}")


def attest_locked_inputs() -> dict[str, Any]:
    """Attest the immutable R9 corpus/terminal and R6 source without H0 calls."""

    r9_corpus = _record(contract.R9_CORPUS_PATH)
    r9_terminal = _tree_record(contract.R9_CANDIDATE_MODULE_PATH)
    r6_source = _tree_record(contract.R6_CANDIDATE_MODULE_PATH)
    ledger_path = getattr(
        contract,
        "R9_TERMINAL_LEDGER_PATH",
        PurePosixPath(R9_TERMINAL_LEDGER_RECORD["path"]),
    )
    gate_path = getattr(
        contract, "R9_FIT_GATE_PATH", PurePosixPath(R9_FIT_GATE_RECORD["path"])
    )
    summary_path = getattr(
        contract,
        "R9_FIT_SUMMARY_PATH",
        PurePosixPath(R9_FIT_SUMMARY_RECORD["path"]),
    )
    ledger_record = _record(ledger_path)
    gate_record = _record(gate_path)
    summary_record = _record(summary_path)
    _assert_record_exact(r9_corpus, R9_CORPUS_RECORD)
    _assert_record_exact(r9_terminal, R9_CANDIDATE_TREE)
    _assert_record_exact(r6_source, R6_CANDIDATE_TREE)
    _assert_record_exact(ledger_record, R9_TERMINAL_LEDGER_RECORD)
    _assert_record_exact(gate_record, R9_FIT_GATE_RECORD)
    _assert_record_exact(summary_record, R9_FIT_SUMMARY_RECORD)
    ledger = _strict_json(ledger_path)
    gate = _strict_json(gate_path)
    summary = _strict_json(summary_path)
    terminal_checks = {
        "r9_terminal_failed": ledger.get("status")
        == "FAIL_H0_V12R9_RECOVERY_PIPELINE_TERMINAL"
        and ledger.get("passed") is False,
        "r9_failed_at_single_fit": ledger.get("attempted_stage") == "fit_recovery_actor"
        and gate.get("status") == "FAIL_H0_V12R9_RECOVERY_FIT"
        and gate.get("passed") is False,
        "r9_candidate_not_frozen": ledger.get("candidate_freeze") is None,
        "r9_candidate_not_promoted": ledger.get("runtime_promoted") is False,
        "r9_tree_identity": summary.get("candidate_module", {}).get("tree_sha256")
        == R9_CANDIDATE_TREE["tree_sha256"],
    }
    if not all(terminal_checks.values()):
        failed = sorted(name for name, passed in terminal_checks.items() if not passed)
        raise V12R10RecoveryFitError(f"R9 terminal provenance drifted: {failed}")
    return {
        "r9_corpus": r9_corpus,
        "r9_terminal_candidate": r9_terminal,
        "r9_terminal_ledger": ledger_record,
        "r9_fit_gate": gate_record,
        "r9_fit_summary": summary_record,
        "r6_source_candidate": r6_source,
        "r9_initialization_only": True,
        "r9_promoted": False,
        "offline_h0_queries": 0,
    }


def expected_stratum_ids() -> tuple[str, ...]:
    return tuple(r9.expected_stratum_ids())


def compute_reset3_equal_stratum_weights(
    stratum_ids: Any,
    reset_mask: Any,
    original_weights: Any | None = None,
) -> tuple[np.ndarray, dict[str, Any]]:
    """Apply reset x3, then restore each of the thirteen strata to mass 500."""

    strata = np.asarray(stratum_ids)
    reset = np.asarray(reset_mask)
    if (
        strata.ndim != 1
        or strata.dtype.kind != "U"
        or reset.shape != strata.shape
        or reset.dtype != np.dtype(np.bool_)
        or set(strata.astype(str)) != set(expected_stratum_ids())
    ):
        raise V12R10RecoveryFitError("reset3 stratum/reset schema drifted")
    if original_weights is None:
        original, _audit = r9.compute_equal_stratum_weights(strata)
    else:
        original = np.ascontiguousarray(original_weights, dtype=np.float64)
        expected, _audit = r9.compute_equal_stratum_weights(strata)
        if not _bytes_equal(original, expected):
            raise V12R10RecoveryFitError("R9 equal-stratum input weights drifted")
    weights = np.ascontiguousarray(original.copy(), dtype=np.float64)
    weights[reset] *= RESET_MULTIPLIER
    strata_audit: dict[str, Any] = {}
    for stratum_id in expected_stratum_ids():
        selected = np.flatnonzero(strata == stratum_id)
        weights[selected] *= STRATUM_MASS / math.fsum(weights[selected])
        mass = math.fsum(weights[selected])
        if not math.isclose(mass, STRATUM_MASS, rel_tol=0.0, abs_tol=1.0e-9):
            raise V12R10RecoveryFitError(f"reset3 stratum mass drifted: {stratum_id}")
        reset_selected = selected[reset[selected]]
        strata_audit[stratum_id] = {
            "rows": int(len(selected)),
            "reset_rows": int(len(reset_selected)),
            "mass": float(mass),
            "row_weight_min": float(np.min(weights[selected])),
            "row_weight_max": float(np.max(weights[selected])),
            "reset_weight": (
                float(weights[reset_selected[0]]) if len(reset_selected) else None
            ),
        }
    total = math.fsum(weights)
    if (
        not math.isclose(total, STRATUM_COUNT * STRATUM_MASS, abs_tol=1.0e-8)
        or not np.all(np.isfinite(weights))
        or np.any(weights <= 0.0)
    ):
        raise V12R10RecoveryFitError("reset3 total weight closure failed")
    return weights, {
        "policy": "R9_UNIFORM_EQUAL_STRATUM_WITH_RESET_ROWS_X3",
        "reset_multiplier": RESET_MULTIPLIER,
        "reset_row_count": int(np.count_nonzero(reset)),
        "stratum_count": STRATUM_COUNT,
        "stratum_mass": STRATUM_MASS,
        "total_mass": float(total),
        "pre_renormalization_reset_mass": float(
            math.fsum(original[reset] * RESET_MULTIPLIER)
        ),
        "post_renormalization_reset_mass": float(math.fsum(weights[reset])),
        "weights_sha256": r9.v10s_fit.array_sha256(weights),
        "strata": strata_audit,
    }


def _read_r9_corpus_exact() -> dict[str, np.ndarray]:
    path = _resolve(contract.R9_CORPUS_PATH)
    _reject_link_components(path, include_leaf=True)
    if not _regular_file(path):
        raise V12R10RecoveryFitError("locked R9 corpus is missing or unsafe")
    try:
        with np.load(path, allow_pickle=False) as archive:
            if set(archive.files) != R9_CORPUS_KEYS:
                raise V12R10RecoveryFitError("R9 corpus NPZ schema drifted")
            return {
                name: np.ascontiguousarray(archive[name].copy())
                for name in archive.files
            }
    except V12R10RecoveryFitError:
        raise
    except Exception as exc:
        raise V12R10RecoveryFitError("cannot read locked R9 corpus") from exc


def load_locked_r9_corpus() -> RecoveryCorpusBundle:
    """Load and semantically close the byte-exact R9 corpus without relabeling."""

    source_records = attest_locked_inputs()
    arrays = _read_r9_corpus_exact()
    rows = 11_875
    unicode_rows = ("case_ids", "tranche_ids", "origins", "episode_ids", "stratum_ids")
    checks = {
        "observations": arrays["observations"].shape == (rows, INPUT_WIDTH)
        and arrays["observations"].dtype == np.float32,
        "actions": arrays["actions"].shape == (rows, ACTION_DIM)
        and arrays["actions"].dtype == np.float32,
        "reset": arrays["reset_mask"].shape == (rows,)
        and arrays["reset_mask"].dtype == np.bool_
        and int(np.count_nonzero(arrays["reset_mask"])) == 26,
        "features": arrays["actor_feature_names"].shape == (INPUT_WIDTH,)
        and arrays["actor_feature_names"].dtype.kind == "U",
        "unicode_rows": all(
            arrays[name].shape == (rows,) and arrays[name].dtype.kind == "U"
            for name in unicode_rows
        ),
        "steps": arrays["step_indices"].shape == (rows,)
        and arrays["step_indices"].dtype == np.int64,
        "training_indices": arrays["training_indices"].shape == (rows,)
        and arrays["training_indices"].dtype == np.int64
        and np.array_equal(arrays["training_indices"], np.arange(rows)),
        "weights": all(
            arrays[name].shape == (rows,) and arrays[name].dtype == np.float64
            for name in ("raw_sample_weights", "normalized_sample_weights")
        ),
        "finite": all(
            np.all(np.isfinite(arrays[name]))
            for name in (
                "observations",
                "actions",
                "raw_sample_weights",
                "normalized_sample_weights",
            )
        ),
        "clock_literals": _positive_zero(arrays["observations"][:, 0])
        and np.array_equal(
            arrays["observations"][:, 1], np.ones(rows, dtype=np.float32)
        ),
        "strata": set(arrays["stratum_ids"].astype(str)) == set(expected_stratum_ids()),
    }
    if not all(checks.values()):
        failed = sorted(name for name, passed in checks.items() if not passed)
        raise V12R10RecoveryFitError(f"R9 corpus semantic closure failed: {failed}")
    weights, weight_audit = compute_reset3_equal_stratum_weights(
        arrays["stratum_ids"],
        arrays["reset_mask"],
        arrays["normalized_sample_weights"],
    )
    selected = np.flatnonzero(arrays["tranche_ids"].astype(str) == "v8r1p1_base")
    if len(selected) != 3_000:
        raise V12R10RecoveryFitError("v8r1p1_base normalization rows drifted")
    normalization = r9.v11.frozen_base_normalization(arrays["observations"][selected])
    strata = arrays["stratum_ids"].astype(str)
    base_indices = {
        case_id: np.flatnonzero(strata == f"base::{case_id}").astype(np.int64)
        for case_id in contract.COLLECTION_CASE_IDS
    }
    observer_indices = {
        case_id: np.flatnonzero(strata == f"observer::{case_id}").astype(np.int64)
        for case_id in contract.COLLECTION_CASE_IDS
    }
    r4_indices = np.flatnonzero(
        strata == "r4_failure::deterministic_offset_plus_0p20"
    ).astype(np.int64)
    plus = observer_indices["deterministic_offset_plus_0p20"]
    observer_plus_late = plus[arrays["step_indices"][plus] >= 140]
    if (
        any(len(index) == 0 for index in base_indices.values())
        or any(len(index) == 0 for index in observer_indices.values())
        or len(r4_indices) == 0
        or len(observer_plus_late) == 0
    ):
        raise V12R10RecoveryFitError("R9 corpus metric slices are incomplete")
    return RecoveryCorpusBundle(
        arrays=arrays,
        sample_weights=weights,
        weight_audit=weight_audit,
        normalization=normalization,
        base_indices=base_indices,
        r4_indices=r4_indices,
        observer_indices=observer_indices,
        observer_plus_late_indices=observer_plus_late,
        source_records=source_records,
    )


def _load_w512_source(
    path: str | PurePath | Path,
    *,
    expected_tree: Mapping[str, Any],
    label: str,
) -> tuple[Any, dict[str, np.ndarray], dict[str, Any]]:
    from ray.rllib.core.rl_module.rl_module import RLModule

    if not _strict_equal(_tree_record(path), expected_tree):
        raise V12R10RecoveryFitError(f"{label} checkpoint tree drifted")
    module = RLModule.from_checkpoint(_resolve(path))
    module.eval()
    state = _state_arrays(module.get_state())
    audit = r9.validate_source_r6_state(state)
    if (
        list(module.model_config.get("fcnet_hiddens", ())) != [512, 512]
        or str(module.model_config.get("fcnet_activation", "")).lower() != "tanh"
        or module.inference_only is not True
    ):
        raise V12R10RecoveryFitError(f"{label} W512 module topology drifted")
    manifest = _strict_json(_resolve(path) / ACTOR_FEATURE_MANIFEST_NAME)
    if (
        manifest.get("actor_digest") != audit["actor_digest"]
        or not isinstance(manifest.get("actor_feature_names"), list)
        or len(manifest["actor_feature_names"]) != INPUT_WIDTH
    ):
        raise V12R10RecoveryFitError(f"{label} actor manifest drifted")
    return module, state, manifest


def _load_source_modules() -> dict[str, Any]:
    r6_module, r6_state, r6_manifest = _load_w512_source(
        contract.R6_CANDIDATE_MODULE_PATH,
        expected_tree=R6_CANDIDATE_TREE,
        label="R6 immutable tower",
    )
    r9_module, r9_state, r9_manifest = _load_w512_source(
        contract.R9_CANDIDATE_MODULE_PATH,
        expected_tree=R9_CANDIDATE_TREE,
        label="R9 initialization-only tower",
    )
    if r6_manifest["actor_feature_names"] != r9_manifest["actor_feature_names"]:
        raise V12R10RecoveryFitError("R6/R9 actor feature order drifted")
    return {
        "r6_module": r6_module,
        "r6_state": r6_state,
        "r6_manifest": r6_manifest,
        "r9_module": r9_module,
        "r9_state": r9_state,
        "r9_manifest": r9_manifest,
    }


def _new_normalized_residual_model(
    r9_state: Mapping[str, Any], normalization: FrozenNormalization
) -> Any:
    import torch
    from torch import nn

    r9.validate_source_r6_state(r9_state)
    model = nn.Sequential(
        nn.Linear(INPUT_WIDTH, TOWER_WIDTH),
        nn.Tanh(),
        nn.Linear(TOWER_WIDTH, TOWER_WIDTH),
        nn.Tanh(),
        nn.Linear(TOWER_WIDTH, ACTION_DIM),
    )
    mean = torch.as_tensor(normalization.mean, dtype=torch.float32)
    std = torch.as_tensor(normalization.std, dtype=torch.float32)
    raw_w0 = torch.as_tensor(r9_state["pi_encoder.0.weight"], dtype=torch.float32)
    raw_b0 = torch.as_tensor(r9_state["pi_encoder.0.bias"], dtype=torch.float32)
    with torch.no_grad():
        model[0].weight.copy_(raw_w0 * std[None, :])
        model[0].bias.copy_(raw_b0 + raw_w0 @ mean)
        model[2].weight.copy_(
            torch.as_tensor(r9_state["pi_encoder.2.weight"], dtype=torch.float32)
        )
        model[2].bias.copy_(
            torch.as_tensor(r9_state["pi_encoder.2.bias"], dtype=torch.float32)
        )
        model[4].weight.zero_()
        model[4].bias.zero_()
    if not _positive_zero(
        model[0].weight.detach().cpu().numpy()[:, contract.DISABLED_CLOCK_COLUMNS]
    ) or not _positive_zero(model[4].weight.detach().cpu().numpy()):
        raise V12R10RecoveryFitError("residual zero-head initialization drifted")
    return model


def _pack_w1024_state(
    *,
    residual: Any,
    r6_state: Mapping[str, Any],
    normalization: FrozenNormalization,
) -> dict[str, np.ndarray]:
    import torch

    normalized_w0 = np.ascontiguousarray(
        residual[0].weight.detach().cpu().numpy(), dtype=np.float32
    )
    normalized_b0 = np.ascontiguousarray(
        residual[0].bias.detach().cpu().numpy(), dtype=np.float32
    )
    if not _positive_zero(normalized_w0[:, contract.DISABLED_CLOCK_COLUMNS]):
        raise V12R10RecoveryFitError("trained residual clock columns drifted")
    raw_w0 = np.ascontiguousarray(
        normalized_w0 / normalization.std[None, :], dtype=np.float32
    )
    raw_b0 = np.ascontiguousarray(
        normalized_b0
        - torch.as_tensor(raw_w0, dtype=torch.float32)
        .matmul(torch.as_tensor(normalization.mean, dtype=torch.float32))
        .detach()
        .cpu()
        .numpy(),
        dtype=np.float32,
    )
    if not _positive_zero(raw_w0[:, contract.DISABLED_CLOCK_COLUMNS]):
        raise V12R10RecoveryFitError("folded residual clock columns drifted")

    first_weight = np.zeros((TARGET_WIDTH, INPUT_WIDTH), dtype=np.float32)
    first_bias = np.zeros(TARGET_WIDTH, dtype=np.float32)
    first_weight[:TOWER_WIDTH] = r6_state["pi_encoder.0.weight"]
    first_weight[TOWER_WIDTH:] = raw_w0
    first_bias[:TOWER_WIDTH] = r6_state["pi_encoder.0.bias"]
    first_bias[TOWER_WIDTH:] = raw_b0

    second_weight = np.zeros((TARGET_WIDTH, TARGET_WIDTH), dtype=np.float32)
    second_bias = np.zeros(TARGET_WIDTH, dtype=np.float32)
    second_weight[:TOWER_WIDTH, :TOWER_WIDTH] = r6_state["pi_encoder.2.weight"]
    second_weight[TOWER_WIDTH:, TOWER_WIDTH:] = np.ascontiguousarray(
        residual[2].weight.detach().cpu().numpy(), dtype=np.float32
    )
    second_bias[:TOWER_WIDTH] = r6_state["pi_encoder.2.bias"]
    second_bias[TOWER_WIDTH:] = np.ascontiguousarray(
        residual[2].bias.detach().cpu().numpy(), dtype=np.float32
    )

    head_weight = np.zeros((2 * ACTION_DIM, TARGET_WIDTH), dtype=np.float32)
    head_bias = np.ascontiguousarray(r6_state["pi.1.bias"], dtype=np.float32).copy()
    head_weight[:ACTION_DIM, :TOWER_WIDTH] = r6_state["pi.1.weight"][:ACTION_DIM]
    head_weight[:ACTION_DIM, TOWER_WIDTH:] = np.ascontiguousarray(
        residual[4].weight.detach().cpu().numpy(), dtype=np.float32
    )
    head_bias[:ACTION_DIM] += np.ascontiguousarray(
        residual[4].bias.detach().cpu().numpy(), dtype=np.float32
    )
    state = {
        "pi_encoder.0.weight": first_weight,
        "pi_encoder.0.bias": first_bias,
        "pi_encoder.2.weight": second_weight,
        "pi_encoder.2.bias": second_bias,
        "pi.0.0.weight": first_weight.copy(),
        "pi.0.0.bias": first_bias.copy(),
        "pi.0.2.weight": second_weight.copy(),
        "pi.0.2.bias": second_bias.copy(),
        "pi.1.weight": head_weight,
        "pi.1.bias": head_bias,
    }
    validate_w1024_state(state)
    return state


def tower_isolation_audit(
    candidate: Mapping[str, Any], r6_state: Mapping[str, Any]
) -> dict[str, Any]:
    """Prove tower A, both cross blocks, and logstd stayed immutable."""

    validate_w1024_state(candidate)
    r9.validate_source_r6_state(r6_state)
    checks = {
        "r6_first_weight_byte_exact": _bytes_equal(
            candidate["pi_encoder.0.weight"][:TOWER_WIDTH],
            r6_state["pi_encoder.0.weight"],
        ),
        "r6_first_bias_byte_exact": _bytes_equal(
            candidate["pi_encoder.0.bias"][:TOWER_WIDTH],
            r6_state["pi_encoder.0.bias"],
        ),
        "r6_second_weight_byte_exact": _bytes_equal(
            candidate["pi_encoder.2.weight"][:TOWER_WIDTH, :TOWER_WIDTH],
            r6_state["pi_encoder.2.weight"],
        ),
        "r6_second_bias_byte_exact": _bytes_equal(
            candidate["pi_encoder.2.bias"][:TOWER_WIDTH],
            r6_state["pi_encoder.2.bias"],
        ),
        "r6_mean_head_weight_byte_exact": _bytes_equal(
            candidate["pi.1.weight"][:ACTION_DIM, :TOWER_WIDTH],
            r6_state["pi.1.weight"][:ACTION_DIM],
        ),
        "cross_blocks_positive_zero": _positive_zero(
            candidate["pi_encoder.2.weight"][:TOWER_WIDTH, TOWER_WIDTH:]
        )
        and _positive_zero(
            candidate["pi_encoder.2.weight"][TOWER_WIDTH:, :TOWER_WIDTH]
        ),
        "logstd_bias_byte_exact": _bytes_equal(
            candidate["pi.1.bias"][ACTION_DIM:],
            r6_state["pi.1.bias"][ACTION_DIM:],
        ),
        "logstd_full_weight_positive_zero": _positive_zero(
            candidate["pi.1.weight"][ACTION_DIM:]
        ),
    }
    return {"passed": all(checks.values()), "checks": checks}


def uniform_adamw_rate(epoch: int) -> float:
    if type(epoch) is not int or not 1 <= epoch <= UNIFORM_ADAMW_EPOCHS:
        raise V12R10RecoveryFitError(f"uniform AdamW epoch outside schedule: {epoch!r}")
    if epoch <= UNIFORM_ADAMW_BOUNDARIES[0]:
        return UNIFORM_ADAMW_RATES[0]
    if epoch <= UNIFORM_ADAMW_BOUNDARIES[1]:
        return UNIFORM_ADAMW_RATES[1]
    return UNIFORM_ADAMW_RATES[2]


def gate_adamw_rate(epoch: int) -> float:
    if type(epoch) is not int or not 1 <= epoch <= GATE_ADAMW_EPOCHS:
        raise V12R10RecoveryFitError(f"gate AdamW epoch outside schedule: {epoch!r}")
    if epoch <= GATE_ADAMW_BOUNDARIES[0]:
        return GATE_ADAMW_RATES[0]
    if epoch <= GATE_ADAMW_BOUNDARIES[1]:
        return GATE_ADAMW_RATES[1]
    return GATE_ADAMW_RATES[2]


def _notify(
    callback: Callable[[str, int], None] | None, name: str, amount: int = 1
) -> None:
    if callback is not None:
        callback(name, amount)


def _reproduce_uniform_terminal(
    *,
    arrays: Mapping[str, np.ndarray],
    sample_weights: np.ndarray,
    normalization: FrozenNormalization,
    r6_state: Mapping[str, Any],
    r9_state: Mapping[str, Any],
    activity_callback: Callable[[str, int], None] | None,
) -> dict[str, Any]:
    """Run phase A and reject anything but its byte-locked terminal."""

    import torch

    raw = arrays["observations"]
    normalized = r9.v11.normalized_observations(raw, normalization)
    fixed_predictions_np = np.ascontiguousarray(
        r9.v11._state_logits(r6_state, raw)[:, :ACTION_DIM],  # noqa: SLF001
        dtype=np.float32,
    )
    torch.manual_seed(SEED)
    residual = _new_normalized_residual_model(r9_state, normalization)
    x = torch.as_tensor(normalized, dtype=torch.float32)
    targets = torch.as_tensor(arrays["actions"], dtype=torch.float32)
    fixed = torch.as_tensor(fixed_predictions_np, dtype=torch.float32)
    weights = torch.as_tensor(sample_weights, dtype=torch.float64)
    weight_sum = torch.sum(weights)

    def objective() -> tuple[Any, Any]:
        combined = fixed + residual(x)
        row_mse = torch.mean(torch.square(combined - targets), dim=1).to(torch.float64)
        return torch.sum(weights * row_mse) / weight_sum, combined

    with torch.no_grad():
        initial_combined = fixed + residual(x)
        initial_loss, _ = objective()
    if not _bytes_equal(initial_combined.cpu().numpy(), fixed_predictions_np):
        raise V12R10RecoveryFitError("zero-head residual is not exactly R6")

    history: list[dict[str, Any]] = [
        {
            "stage": "uniform_initial",
            "index": 0,
            "loss": float(initial_loss.detach().cpu()),
        }
    ]
    adamw = torch.optim.AdamW(
        residual.parameters(),
        lr=UNIFORM_ADAMW_RATES[0],
        weight_decay=UNIFORM_ADAMW_WEIGHT_DECAY,
    )
    uniform_milestones = {1, 100, 250, 500, 1_000, 1_500, 2_000, 2_500}
    for epoch in range(1, UNIFORM_ADAMW_EPOCHS + 1):
        rate = uniform_adamw_rate(epoch)
        for group in adamw.param_groups:
            group["lr"] = rate
        adamw.zero_grad(set_to_none=True)
        loss, _ = objective()
        if not torch.isfinite(loss):
            raise V12R10RecoveryFitError(
                f"non-finite uniform loss at AdamW epoch {epoch}"
            )
        loss.backward()
        torch.nn.utils.clip_grad_norm_(
            residual.parameters(), UNIFORM_GRADIENT_CLIP_NORM
        )
        adamw.step()
        with torch.no_grad():
            residual[0].weight[:, contract.DISABLED_CLOCK_COLUMNS] = 0.0
        _notify(activity_callback, "uniform_adamw_epochs_completed")
        # Preserve the validated call pattern exactly.
        if epoch in uniform_milestones:
            with torch.no_grad():
                milestone_loss, _ = objective()
            history.append(
                {
                    "stage": "uniform_adamw",
                    "index": epoch,
                    "loss": float(milestone_loss.detach().cpu()),
                    "learning_rate": rate,
                }
            )

    lbfgs = torch.optim.LBFGS(
        residual.parameters(),
        lr=UNIFORM_LBFGS_LR,
        max_iter=UNIFORM_LBFGS_MAX_ITER,
        max_eval=UNIFORM_LBFGS_MAX_EVAL,
        tolerance_grad=UNIFORM_LBFGS_TOLERANCE_GRAD,
        tolerance_change=UNIFORM_LBFGS_TOLERANCE_CHANGE,
        history_size=UNIFORM_LBFGS_HISTORY_SIZE,
        line_search_fn="strong_wolfe",
    )
    closure_calls = 0
    closure_milestones = {
        1,
        100,
        250,
        500,
        750,
        1_000,
        1_250,
        1_500,
        1_750,
        2_000,
        2_250,
        2_500,
        2_750,
        3_000,
        3_500,
        4_000,
        4_500,
    }

    def closure() -> Any:
        nonlocal closure_calls
        lbfgs.zero_grad(set_to_none=True)
        value, _ = objective()
        if not torch.isfinite(value):
            raise V12R10RecoveryFitError(
                f"non-finite uniform loss at LBFGS closure {closure_calls + 1}"
            )
        value.backward()
        closure_calls += 1
        _notify(activity_callback, "uniform_lbfgs_closure_calls")
        if closure_calls in closure_milestones:
            history.append(
                {
                    "stage": "uniform_lbfgs_closure",
                    "index": closure_calls,
                    "loss": float(value.detach().cpu()),
                }
            )
        return value

    lbfgs.step(closure)
    with torch.no_grad():
        residual[0].weight[:, contract.DISABLED_CLOCK_COLUMNS] = 0.0
        terminal_loss, normalized_prediction_tensor = objective()
    candidate_state = _pack_w1024_state(
        residual=residual, r6_state=r6_state, normalization=normalization
    )
    predictions = np.ascontiguousarray(
        r9.v11._state_logits(candidate_state, raw)[:, :ACTION_DIM],  # noqa: SLF001
        dtype=np.float32,
    )
    normalized_predictions = np.ascontiguousarray(
        normalized_prediction_tensor.cpu().numpy(), dtype=np.float32
    )
    fold_max = float(
        np.max(
            np.abs(
                predictions.astype(np.float64)
                - normalized_predictions.astype(np.float64)
            )
        )
    )
    attestation = {
        "candidate_state_digest": _state_digest(candidate_state),
        "candidate_predictions_sha256": _prediction_digest(predictions),
        "lbfgs_closure_calls": closure_calls,
        "terminal_loss": float(terminal_loss.detach().cpu()),
        "normalization_fold_max_abs_difference": fold_max,
    }
    checks = {
        "state_digest_exact": attestation["candidate_state_digest"]
        == EXPECTED_UNIFORM_STATE_DIGEST,
        "prediction_digest_exact": attestation["candidate_predictions_sha256"]
        == EXPECTED_UNIFORM_PREDICTION_DIGEST,
        "closure_count_exact": closure_calls == EXPECTED_UNIFORM_CLOSURES,
        "terminal_loss_exact": attestation["terminal_loss"]
        == EXPECTED_UNIFORM_TERMINAL_LOSS,
        "normalization_fold_within_tolerance": fold_max <= FOLD_NUMERICAL_TOLERANCE,
    }
    if not all(checks.values()):
        failed = sorted(name for name, passed in checks.items() if not passed)
        raise V12R10RecoveryFitError(f"uniform terminal attestation failed: {failed}")
    history.append(
        {
            "stage": "uniform_terminal_exact",
            "index": closure_calls,
            "loss": attestation["terminal_loss"],
            "candidate_state_digest": attestation["candidate_state_digest"],
            "candidate_predictions_sha256": attestation["candidate_predictions_sha256"],
        }
    )
    return {
        "residual": residual,
        "x": x,
        "targets": targets,
        "fixed": fixed,
        "weights": weights,
        "weight_sum": weight_sum,
        "attestation": {"passed": True, "checks": checks, **attestation},
        "history": history,
    }


def _gate_group_indices(arrays: Mapping[str, np.ndarray]) -> dict[str, np.ndarray]:
    strata = arrays["stratum_ids"].astype(str)
    steps = arrays["step_indices"]
    groups: dict[str, np.ndarray] = {
        "global": np.arange(len(strata), dtype=np.int64),
    }
    for case_id in contract.COLLECTION_CASE_IDS:
        groups[f"base::{case_id}"] = np.flatnonzero(
            strata == f"base::{case_id}"
        ).astype(np.int64)
    for case_id in contract.COLLECTION_CASE_IDS:
        groups[f"observer::{case_id}"] = np.flatnonzero(
            strata == f"observer::{case_id}"
        ).astype(np.int64)
    groups["r4_failure::deterministic_offset_plus_0p20"] = np.flatnonzero(
        strata == "r4_failure::deterministic_offset_plus_0p20"
    ).astype(np.int64)
    groups["observer_plus_late"] = np.flatnonzero(
        (strata == "observer::deterministic_offset_plus_0p20") & (steps >= 140)
    ).astype(np.int64)
    if len(groups) != 15 or any(len(index) == 0 for index in groups.values()):
        raise V12R10RecoveryFitError("symmetric gate-group construction drifted")
    return groups


def _smooth_margin(value: Any) -> tuple[Any, Any]:
    import torch

    temperature = SMOOTH_MAX_TEMPERATURE
    smooth_max = temperature * torch.logsumexp(value / temperature, dim=0)
    excess = temperature * torch.nn.functional.softplus(
        (smooth_max - SAFETY_MARGIN_FRACTION) / temperature
    )
    return smooth_max, excess


def _metric_pair(
    predictions: np.ndarray, targets: np.ndarray, indices: np.ndarray
) -> dict[str, float]:
    selected = np.asarray(indices, dtype=np.int64)
    if selected.ndim != 1 or len(selected) == 0:
        raise V12R10RecoveryFitError("metric slice is empty or malformed")
    error = predictions[selected].astype(np.float64) - targets[selected].astype(
        np.float64
    )
    return {
        "rmse": float(np.sqrt(np.mean(np.square(error)))),
        "max_abs_error": float(np.max(np.abs(error))),
    }


def recompute_fit_metric_payload(
    bundle: RecoveryCorpusBundle,
    candidate_state: Mapping[str, Any],
) -> dict[str, Any]:
    arrays = bundle.arrays
    predictions = np.ascontiguousarray(
        r9.v11._state_logits(candidate_state, arrays["observations"])[  # noqa: SLF001
            :, :ACTION_DIM
        ],
        dtype=np.float32,
    )
    if predictions.shape != arrays["actions"].shape or not np.all(
        np.isfinite(predictions)
    ):
        raise V12R10RecoveryFitError("candidate predictions are malformed")
    absolute_error = np.abs(
        predictions.astype(np.float64) - arrays["actions"].astype(np.float64)
    )
    flat = int(np.argmax(absolute_error))
    worst_row, worst_action = np.unravel_index(flat, absolute_error.shape)
    reset_error = absolute_error[arrays["reset_mask"]]
    return {
        "predictions": predictions,
        "global_metrics": _metric_pair(
            predictions,
            arrays["actions"],
            np.arange(len(predictions), dtype=np.int64),
        ),
        "reset_max_abs_error": float(np.max(reset_error)),
        "per_case_metrics": {
            case_id: _metric_pair(
                predictions, arrays["actions"], bundle.base_indices[case_id]
            )
            for case_id in contract.COLLECTION_CASE_IDS
        },
        "r4_failed_plus_metrics": _metric_pair(
            predictions, arrays["actions"], bundle.r4_indices
        ),
        "observer_case_metrics": {
            case_id: _metric_pair(
                predictions, arrays["actions"], bundle.observer_indices[case_id]
            )
            for case_id in contract.COLLECTION_CASE_IDS
        },
        "observer_plus_late_metrics": _metric_pair(
            predictions, arrays["actions"], bundle.observer_plus_late_indices
        ),
        "worst_row": {
            "absolute_error": float(absolute_error[worst_row, worst_action]),
            "action_dimension": int(worst_action),
            "case_id": str(arrays["case_ids"][worst_row]),
            "row_index": int(worst_row),
            "step_index": int(arrays["step_indices"][worst_row]),
            "stratum_id": str(arrays["stratum_ids"][worst_row]),
            "tranche_id": str(arrays["tranche_ids"][worst_row]),
        },
    }


def _metric_payload_from_predictions(
    bundle: RecoveryCorpusBundle, predictions: np.ndarray
) -> dict[str, Any]:
    """Testing/fit helper equivalent to state-based metric reconstruction."""

    arrays = bundle.arrays
    absolute_error = np.abs(
        predictions.astype(np.float64) - arrays["actions"].astype(np.float64)
    )
    flat = int(np.argmax(absolute_error))
    worst_row, worst_action = np.unravel_index(flat, absolute_error.shape)
    return {
        "global_metrics": _metric_pair(
            predictions,
            arrays["actions"],
            np.arange(len(predictions), dtype=np.int64),
        ),
        "reset_max_abs_error": float(np.max(absolute_error[arrays["reset_mask"]])),
        "per_case_metrics": {
            case_id: _metric_pair(
                predictions, arrays["actions"], bundle.base_indices[case_id]
            )
            for case_id in contract.COLLECTION_CASE_IDS
        },
        "r4_failed_plus_metrics": _metric_pair(
            predictions, arrays["actions"], bundle.r4_indices
        ),
        "observer_case_metrics": {
            case_id: _metric_pair(
                predictions, arrays["actions"], bundle.observer_indices[case_id]
            )
            for case_id in contract.COLLECTION_CASE_IDS
        },
        "observer_plus_late_metrics": _metric_pair(
            predictions, arrays["actions"], bundle.observer_plus_late_indices
        ),
        "worst_row": {
            "absolute_error": float(absolute_error[worst_row, worst_action]),
            "action_dimension": int(worst_action),
            "case_id": str(arrays["case_ids"][worst_row]),
            "row_index": int(worst_row),
            "step_index": int(arrays["step_indices"][worst_row]),
            "stratum_id": str(arrays["stratum_ids"][worst_row]),
            "tranche_id": str(arrays["tranche_ids"][worst_row]),
        },
    }


FIT_METRIC_FIELDS = (
    "global_metrics",
    "reset_max_abs_error",
    "per_case_metrics",
    "r4_failed_plus_metrics",
    "observer_case_metrics",
    "observer_plus_late_metrics",
    "worst_row",
)
FIT_SUMMARY_FIELDS = frozenset(
    """
    schema_version status passed protocol_id fit_contract_id
    candidate_selection_rule hidden_dims actor_feature_count sample_count
    reset_row_count stratum_count stratum_target_mass weight_audit
    normalization_audit global_metrics reset_max_abs_error per_case_metrics
    r4_failed_plus_metrics observer_case_metrics observer_plus_late_metrics
    worst_row actor_fit_count actor_updates critic_updates ppo_updates
    offline_h0_queries environment_reset_calls environment_step_calls
    uniform_adamw_epochs uniform_lbfgs_max_iter uniform_lbfgs_max_eval
    uniform_state_digest uniform_predictions_sha256
    uniform_lbfgs_closure_calls uniform_terminal_loss gate_adamw_epochs
    gate_lbfgs_max_iter gate_lbfgs_max_eval candidate_state_digest
    candidate_predictions_sha256 gate_lbfgs_closure_calls gate_terminal_loss
    tower_a_r6_byte_exact cross_blocks_positive_zero logstd_byte_exact
    disabled_clock_columns_bit_zero save_reload_exact runtime_save_reload_exact
    warm_start_actor_exact warm_start_critic_preserved standard_rlmodule
    no_legacy_shadow_runtime_dependency critic_present_in_candidate
    fallback_used sweep_used retry_used replica_used optimizer_audit history
    state_audit tower_isolation runtime_warm_start_audit candidate_id
    candidate_module r9_corpus r9_terminal_candidate r6_source_candidate
    actor_feature_manifest candidate_build_manifest pipeline_claim worker_claim
    protocol_freeze execution_lock limitations
    """.split()
)
FIT_RECEIPT_FIELDS = frozenset(
    """
    schema_version status passed protocol_id fit_contract_id
    candidate_selection_rule candidate_id candidate_module summary gate
    r9_corpus r9_terminal_candidate r6_source_candidate actor_feature_manifest
    candidate_build_manifest pipeline_claim worker_claim protocol_freeze
    execution_lock actor_fit_count actor_updates critic_updates ppo_updates
    offline_h0_queries environment_reset_calls environment_step_calls
    runtime_save_reload_exact warm_start_actor_exact
    warm_start_critic_preserved no_legacy_shadow_runtime_dependency limitations
    """.split()
)


def _assert_expected_fit_attestation(payload: Mapping[str, Any]) -> None:
    expected = {
        "uniform_state_digest": EXPECTED_UNIFORM_STATE_DIGEST,
        "uniform_predictions_sha256": EXPECTED_UNIFORM_PREDICTION_DIGEST,
        "uniform_lbfgs_closure_calls": EXPECTED_UNIFORM_CLOSURES,
        "uniform_terminal_loss": EXPECTED_UNIFORM_TERMINAL_LOSS,
        "candidate_state_digest": EXPECTED_FINAL_STATE_DIGEST,
        "candidate_predictions_sha256": EXPECTED_FINAL_PREDICTION_DIGEST,
        "gate_lbfgs_closure_calls": EXPECTED_GATE_CLOSURES,
        "gate_terminal_loss": EXPECTED_GATE_TERMINAL_LOSS,
    }
    drifted = [
        name
        for name, value in expected.items()
        if not _strict_equal(payload.get(name), value)
    ]
    if drifted:
        raise V12R10RecoveryFitError(f"frozen fit attestation drifted: {drifted}")


def fit_recovery_actor(
    *,
    bundle: RecoveryCorpusBundle,
    r6_state: Mapping[str, Any],
    r9_initialization_state: Mapping[str, Any],
    activity_callback: Callable[[str, int], None] | None = None,
) -> RecoveryFitResult:
    """Execute the sole two-phase, actor-only production fit in memory."""

    import torch

    r9.validate_source_r6_state(r6_state)
    r9.validate_source_r6_state(r9_initialization_state)
    arrays = bundle.arrays
    previous_threads = torch.get_num_threads()
    previous_deterministic = torch.are_deterministic_algorithms_enabled()
    torch.set_num_threads(TORCH_THREADS)
    torch.use_deterministic_algorithms(True)
    started = time.monotonic()
    _notify(activity_callback, "actor_updates")
    try:
        replay = _reproduce_uniform_terminal(
            arrays=arrays,
            sample_weights=bundle.sample_weights,
            normalization=bundle.normalization,
            r6_state=r6_state,
            r9_state=r9_initialization_state,
            activity_callback=activity_callback,
        )
        residual = replay["residual"]
        x = replay["x"]
        targets = replay["targets"]
        fixed = replay["fixed"]
        weights = replay["weights"]
        weight_sum = replay["weight_sum"]
        group_indices_np = _gate_group_indices(arrays)
        group_indices = {
            name: torch.as_tensor(index, dtype=torch.int64)
            for name, index in group_indices_np.items()
        }
        reset_indices = torch.as_tensor(
            np.flatnonzero(arrays["reset_mask"]), dtype=torch.int64
        )

        def objective() -> tuple[Any, Any, dict[str, Any]]:
            combined = fixed + residual(x)
            error = (combined - targets).to(torch.float64)
            row_mse = torch.mean(torch.square(error), dim=1)
            preserve = (
                torch.sum(weights * row_mse) / weight_sum / (RMSE_LIMIT * RMSE_LIMIT)
            )
            normalized_group_rmse = torch.stack(
                [
                    torch.sqrt(torch.mean(torch.square(error[index])) + 1.0e-18)
                    / RMSE_LIMIT
                    for index in group_indices.values()
                ]
            )
            group_smooth_max, group_term = _smooth_margin(normalized_group_rmse)
            normalized_scalar_error = torch.abs(error).reshape(-1) / MAX_ABS_LIMIT
            tail_smooth_max, tail_term = _smooth_margin(normalized_scalar_error)
            normalized_reset_error = (
                torch.abs(error[reset_indices]).reshape(-1) / RESET_MAX_ABS_LIMIT
            )
            reset_smooth_max, reset_term = _smooth_margin(normalized_reset_error)
            # All four frozen coefficients are one; keep the validated graph
            # and floating-point operation order byte-for-byte.
            total = preserve + group_term + tail_term + reset_term
            terms = {
                "reset3_mse_preservation": preserve,
                "worst_group_rmse": group_term,
                "global_tail_max_abs": tail_term,
                "reset_max_abs": reset_term,
                "smooth_worst_group_rmse_fraction": group_smooth_max,
                "smooth_global_tail_fraction": tail_smooth_max,
                "smooth_reset_fraction": reset_smooth_max,
                "group_rmse_fractions": normalized_group_rmse,
            }
            return total, combined, terms

        history = list(replay["history"])

        def snapshot(stage: str, index: int) -> None:
            with torch.no_grad():
                loss, _prediction, terms = objective()
            history.append(
                {
                    "stage": stage,
                    "index": int(index),
                    "loss": float(loss.detach().cpu()),
                    "terms": {
                        name: float(value.detach().cpu())
                        for name, value in terms.items()
                        if name != "group_rmse_fractions"
                    },
                    "group_rmse_fractions": {
                        name: float(value)
                        for name, value in zip(
                            group_indices,
                            terms["group_rmse_fractions"].detach().cpu().tolist(),
                            strict=True,
                        )
                    },
                }
            )

        snapshot("gate_initial_uniform_terminal", 0)
        adamw = torch.optim.AdamW(
            residual.parameters(),
            lr=GATE_ADAMW_RATES[0],
            weight_decay=GATE_ADAMW_WEIGHT_DECAY,
        )
        gate_milestones = {1, 100, 250, 500, 750, 1_000, 1_250, 1_500}
        for epoch in range(1, GATE_ADAMW_EPOCHS + 1):
            rate = gate_adamw_rate(epoch)
            for group in adamw.param_groups:
                group["lr"] = rate
            adamw.zero_grad(set_to_none=True)
            loss, _, _ = objective()
            if not torch.isfinite(loss):
                raise V12R10RecoveryFitError(
                    f"non-finite gate loss at AdamW epoch {epoch}"
                )
            loss.backward()
            torch.nn.utils.clip_grad_norm_(
                residual.parameters(), GATE_GRADIENT_CLIP_NORM
            )
            adamw.step()
            with torch.no_grad():
                residual[0].weight[:, contract.DISABLED_CLOCK_COLUMNS] = 0.0
            _notify(activity_callback, "gate_adamw_epochs_completed")
            if epoch in gate_milestones:
                snapshot("gate_adamw", epoch)
                history[-1]["learning_rate"] = rate

        lbfgs = torch.optim.LBFGS(
            residual.parameters(),
            lr=GATE_LBFGS_LR,
            max_iter=GATE_LBFGS_MAX_ITER,
            max_eval=GATE_LBFGS_MAX_EVAL,
            tolerance_grad=GATE_LBFGS_TOLERANCE_GRAD,
            tolerance_change=GATE_LBFGS_TOLERANCE_CHANGE,
            history_size=GATE_LBFGS_HISTORY_SIZE,
            line_search_fn="strong_wolfe",
        )
        closure_calls = 0
        gate_closure_milestones = {
            1,
            100,
            250,
            500,
            750,
            1_000,
            1_250,
            1_500,
            1_750,
            2_000,
            2_250,
            2_500,
            2_750,
            3_000,
            3_500,
            4_000,
            4_500,
        }

        def closure() -> Any:
            nonlocal closure_calls
            lbfgs.zero_grad(set_to_none=True)
            value, _, terms = objective()
            if not torch.isfinite(value):
                raise V12R10RecoveryFitError(
                    f"non-finite gate loss at LBFGS closure {closure_calls + 1}"
                )
            value.backward()
            closure_calls += 1
            _notify(activity_callback, "gate_lbfgs_closure_calls")
            if closure_calls in gate_closure_milestones:
                history.append(
                    {
                        "stage": "gate_lbfgs_closure",
                        "index": closure_calls,
                        "loss": float(value.detach().cpu()),
                        "terms": {
                            name: float(term.detach().cpu())
                            for name, term in terms.items()
                            if name != "group_rmse_fractions"
                        },
                    }
                )
            return value

        lbfgs.step(closure)
        with torch.no_grad():
            residual[0].weight[:, contract.DISABLED_CLOCK_COLUMNS] = 0.0
            terminal_loss, normalized_prediction_tensor, terminal_terms = objective()
        candidate_state = _pack_w1024_state(
            residual=residual,
            r6_state=r6_state,
            normalization=bundle.normalization,
        )
        predictions = np.ascontiguousarray(
            r9.v11._state_logits(candidate_state, arrays["observations"])[  # noqa: SLF001
                :, :ACTION_DIM
            ],
            dtype=np.float32,
        )
        normalized_predictions = np.ascontiguousarray(
            normalized_prediction_tensor.cpu().numpy(), dtype=np.float32
        )
        fold_max = float(
            np.max(
                np.abs(
                    predictions.astype(np.float64)
                    - normalized_predictions.astype(np.float64)
                )
            )
        )
        final_attestation = {
            "candidate_state_digest": _state_digest(candidate_state),
            "candidate_predictions_sha256": _prediction_digest(predictions),
            "gate_lbfgs_closure_calls": closure_calls,
            "gate_terminal_loss": float(terminal_loss.detach().cpu()),
        }
        fit_attestation = {
            "uniform_state_digest": replay["attestation"]["candidate_state_digest"],
            "uniform_predictions_sha256": replay["attestation"][
                "candidate_predictions_sha256"
            ],
            "uniform_lbfgs_closure_calls": replay["attestation"]["lbfgs_closure_calls"],
            "uniform_terminal_loss": replay["attestation"]["terminal_loss"],
            **final_attestation,
        }
        _assert_expected_fit_attestation(fit_attestation)
        if fold_max > FOLD_NUMERICAL_TOLERANCE:
            raise V12R10RecoveryFitError(
                f"final normalization fold drifted by {fold_max}"
            )
        metrics = _metric_payload_from_predictions(bundle, predictions)
        state_audit = validate_w1024_state(candidate_state)
        isolation = tower_isolation_audit(candidate_state, r6_state)
        if not isolation["passed"]:
            raise V12R10RecoveryFitError("immutable R6 tower isolation failed")
        history.append(
            {
                "stage": "gate_terminal_final_state",
                "index": closure_calls,
                "loss": final_attestation["gate_terminal_loss"],
                "global_rmse": metrics["global_metrics"]["rmse"],
                "global_max_abs_error": metrics["global_metrics"]["max_abs_error"],
                "reset_max_abs_error": metrics["reset_max_abs_error"],
                "terms": {
                    name: float(term.detach().cpu())
                    for name, term in terminal_terms.items()
                    if name != "group_rmse_fractions"
                },
                "candidate_state_digest": final_attestation["candidate_state_digest"],
                "candidate_predictions_sha256": final_attestation[
                    "candidate_predictions_sha256"
                ],
            }
        )
        return RecoveryFitResult(
            candidate_state=candidate_state,
            predictions=predictions,
            metrics=metrics,
            uniform_attestation=replay["attestation"],
            final_attestation=final_attestation,
            state_audit=state_audit,
            tower_isolation=isolation,
            normalization_audit={
                "normalization": bundle.normalization.record(),
                "source_rows": "v8r1p1_base",
                "source_row_count": 3_000,
                "normalization_folded_into_residual_first_layer": True,
                "runtime_normalization_wrapper": False,
                "fold_equivalence_passed": True,
                "fold_max_abs_difference": fold_max,
                "fold_tolerance": FOLD_NUMERICAL_TOLERANCE,
            },
            optimizer_audit={
                "fit_contract_id": contract.FIT_CONTRACT_ID,
                "seed": SEED,
                "torch_threads": TORCH_THREADS,
                "deterministic_algorithms": True,
                "full_batch": True,
                "terminal_state_only": True,
                "single_actor_fit": True,
                "actor_updates": 1,
                "critic_updates": 0,
                "ppo_updates": 0,
                "offline_h0_queries": 0,
                "environment_reset_calls": 0,
                "environment_step_calls": 0,
                "uniform_adamw_epochs": UNIFORM_ADAMW_EPOCHS,
                "uniform_adamw_boundaries": list(UNIFORM_ADAMW_BOUNDARIES),
                "uniform_adamw_rates": list(UNIFORM_ADAMW_RATES),
                "uniform_lbfgs_max_iter": UNIFORM_LBFGS_MAX_ITER,
                "uniform_lbfgs_max_eval": UNIFORM_LBFGS_MAX_EVAL,
                "uniform_lbfgs_closure_calls": EXPECTED_UNIFORM_CLOSURES,
                "gate_adamw_epochs": GATE_ADAMW_EPOCHS,
                "gate_adamw_boundaries": list(GATE_ADAMW_BOUNDARIES),
                "gate_adamw_rates": list(GATE_ADAMW_RATES),
                "gate_lbfgs_max_iter": GATE_LBFGS_MAX_ITER,
                "gate_lbfgs_max_eval": GATE_LBFGS_MAX_EVAL,
                "gate_lbfgs_closure_calls": EXPECTED_GATE_CLOSURES,
                "sweep": False,
                "retry": False,
                "repair": False,
                "early_stopping": False,
                "best_state_selection": False,
                "determinism_replica": False,
                "elapsed_seconds": float(time.monotonic() - started),
            },
            history=tuple(history),
        )
    except V12R10RecoveryFitError:
        raise
    except Exception as exc:
        raise V12R10RecoveryFitError("fixed V12R10 recovery fit failed") from exc
    finally:
        torch.use_deterministic_algorithms(previous_deterministic)
        torch.set_num_threads(previous_threads)


def _new_w1024_module(
    *,
    source_module: Any,
    state: Mapping[str, Any] | None,
    inference_only: bool,
) -> Any:
    config = dict(source_module.model_config)
    config.update(
        {
            "fcnet_hiddens": [TARGET_WIDTH, TARGET_WIDTH],
            "fcnet_activation": "tanh",
            "n_actor": INPUT_WIDTH,
            "n_full": contract.EXPECTED_FULL_FEATURES,
            "freeze_actor": bool(inference_only),
            "freeze_logstd": bool(inference_only),
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
    return np.ascontiguousarray(
        output[Columns.ACTION_DIST_INPUTS].detach().cpu().numpy(), dtype=np.float32
    )


def _save_rlmodule_no_clobber(module: Any, destination: Path) -> Any:
    from ray.rllib.core.rl_module.rl_module import RLModule

    output = _resolve(destination)
    _reject_link_components(output, include_leaf=True)
    if not output.parent.is_dir() or _is_link_or_reparse(output.parent):
        raise V12R10RecoveryFitError(
            f"candidate parent must be a regular directory: {output.parent}"
        )
    if os.path.lexists(output):
        raise V12R10RecoveryFitError("candidate destination exists/no-clobber")
    try:
        os.mkdir(output, 0o700)
    except OSError as exc:
        raise V12R10RecoveryFitError("cannot exclusively claim candidate") from exc
    module.save_to_path(output)
    core_files = {path.name for path in output.iterdir() if path.is_file()}
    if core_files != {"class_and_ctor_args.pkl", "metadata.json", "module_state.pkl"}:
        raise V12R10RecoveryFitError(
            f"RLModule core file set drifted: {sorted(core_files)}"
        )
    reloaded = RLModule.from_checkpoint(output)
    reloaded.eval()
    return reloaded


def runtime_and_warm_start_audit(
    *,
    candidate_path: str | PurePath | Path,
    intended_state: Mapping[str, Any],
    source_module: Any,
    observations: np.ndarray,
    feature_names: Sequence[str],
) -> dict[str, Any]:
    """Reload the real candidate and transplant it into a fresh critic module."""

    from ray.rllib.core.rl_module.rl_module import RLModule

    checkpoint = _resolve(candidate_path)
    candidate = RLModule.from_checkpoint(checkpoint)
    candidate.eval()
    candidate_state = _state_arrays(candidate.get_state())
    expected_logits = np.ascontiguousarray(
        r9.v11._state_logits(intended_state, observations),  # noqa: SLF001
        dtype=np.float32,
    )
    direct = _runtime_logits(candidate, observations)
    padded = np.zeros(
        (len(observations), contract.EXPECTED_FULL_FEATURES), dtype=np.float32
    )
    padded[:, :INPUT_WIDTH] = observations
    full_input = _runtime_logits(candidate, padded)

    # The verifier does not merely trust the canonical load: exercise a
    # second real RLModule serialization roundtrip in an ephemeral directory.
    with tempfile.TemporaryDirectory(prefix="h0_v12r10_verify_") as temporary:
        roundtrip_path = Path(temporary) / "rl_module"
        candidate.save_to_path(roundtrip_path)
        roundtrip = RLModule.from_checkpoint(roundtrip_path)
        roundtrip.eval()
        roundtrip_state = _state_arrays(roundtrip.get_state())
        roundtrip_logits = _runtime_logits(roundtrip, observations)

    fresh = _new_w1024_module(
        source_module=source_module,
        state=None,
        inference_only=False,
    )
    fresh_before = _state_arrays(fresh.get_state())
    transplanted, transplant_report = warm_start.transplant_actor_state(
        target_state=fresh_before,
        target_actor_feature_names=[str(name) for name in feature_names],
        source_checkpoint=checkpoint,
        source_actor_feature_manifest=checkpoint / ACTOR_FEATURE_MANIFEST_NAME,
        mode="drop",
        zero_target_features=warm_start.DISABLED_GAIT_CLOCK_FEATURES,
    )
    actor_compare = warm_start.compare_actor_states(intended_state, transplanted)
    critic_compare = warm_start.compare_non_actor_states(fresh_before, transplanted)
    fresh.set_state(transplanted)
    transplant_logits = _runtime_logits(fresh, padded)
    checks = {
        "standard_module_class": type(candidate) is AsymmetricActorCriticTorchRLModule,
        "inference_only_actor_state": set(candidate_state) == STATE_KEYS,
        "model_config_w1024_tanh": list(candidate.model_config.get("fcnet_hiddens", ()))
        == [TARGET_WIDTH, TARGET_WIDTH]
        and str(candidate.model_config.get("fcnet_activation", "")).lower() == "tanh",
        "checkpoint_state_byte_exact": _state_byte_exact(
            intended_state, candidate_state
        ),
        "temporary_resave_state_byte_exact": _state_byte_exact(
            candidate_state, roundtrip_state
        ),
        "temporary_resave_logits_byte_exact": _bytes_equal(direct, roundtrip_logits),
        "actor_prefix_runtime_exact": _bytes_equal(direct, expected_logits),
        "full_input_runtime_exact": _bytes_equal(full_input, expected_logits),
        "warm_start_actor_exact": actor_compare["exact"],
        "warm_start_critic_preserved": critic_compare["exact"],
        "warm_start_forward_exact": _bytes_equal(transplant_logits, expected_logits),
        "no_legacy_shadow_runtime_dependency": True,
    }
    return {
        "passed": all(checks.values()),
        "checks": checks,
        "source_actor_only": transplant_report["source_state_is_actor_only"],
        "target_non_actor_keys_preserved": transplant_report[
            "target_non_actor_keys_preserved"
        ],
        "runtime_logits_sha256": _prediction_digest(expected_logits),
        "optimizer_invocations": 0,
        "temporary_roundtrip_removed": True,
        "legacy_shadow_runtime_dependency": False,
    }


def _candidate_actor_manifest(
    *,
    state: Mapping[str, Any],
    feature_names: Sequence[str],
    module_state: Path,
) -> dict[str, Any]:
    return {
        "schema_version": 1,
        "status": contract.ACTOR_FEATURE_MANIFEST_STATUS,
        "topology_id": contract.TOPOLOGY_ID,
        "fit_contract_id": contract.FIT_CONTRACT_ID,
        "actor_feature_count": INPUT_WIDTH,
        "actor_feature_names": [str(name) for name in feature_names],
        "fcnet_hiddens": [TARGET_WIDTH, TARGET_WIDTH],
        "disabled_clock_columns": list(contract.DISABLED_CLOCK_COLUMNS),
        "actor_digest": warm_start.actor_state_digest(state),
        "state_digest": _state_digest(state),
        "module_state_sha256": _sha256_file(module_state),
        "standard_rlmodule": True,
        "legacy_shadow_runtime_dependency": False,
    }


def _candidate_build_manifest(
    *,
    state: Mapping[str, Any],
    actor_manifest: Mapping[str, Any],
    bundle: RecoveryCorpusBundle,
    isolation: Mapping[str, Any],
) -> dict[str, Any]:
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": CANDIDATE_BUILD_STATUS,
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "fit_contract_id": contract.FIT_CONTRACT_ID,
        "topology_id": contract.TOPOLOGY_ID,
        "architecture": copy.deepcopy(contract.FIT["architecture"]),
        "source_provenance": {
            "r6_functional_predecessor": True,
            "r6_candidate": copy.deepcopy(bundle.source_records["r6_source_candidate"]),
            "r9_hidden_initialization_only": True,
            "r9_terminal_candidate": copy.deepcopy(
                bundle.source_records["r9_terminal_candidate"]
            ),
            "r9_promoted": False,
            "r9_corpus": copy.deepcopy(bundle.source_records["r9_corpus"]),
        },
        "r6_functional_predecessor": True,
        "r9_hidden_initialization_only": True,
        "r9_terminal_candidate_promoted": False,
        "stratum_count": STRATUM_COUNT,
        "stratum_target_mass": STRATUM_MASS,
        "reset_multiplier": RESET_MULTIPLIER,
        "weight_audit": copy.deepcopy(dict(bundle.weight_audit)),
        "normalization_source": "v8r1p1_base",
        "normalization_source_rows": 3_000,
        "actor_fit_count": 1,
        "actor_updates": 1,
        "critic_updates": 0,
        "ppo_updates": 0,
        "offline_h0_queries": 0,
        "environment_reset_calls": 0,
        "environment_step_calls": 0,
        "tower_a_r6_byte_exact": isolation["passed"],
        "cross_blocks_positive_zero": isolation["checks"]["cross_blocks_positive_zero"],
        "logstd_byte_exact": isolation["checks"]["logstd_bias_byte_exact"]
        and isolation["checks"]["logstd_full_weight_positive_zero"],
        "disabled_clock_columns_bit_zero": True,
        "save_reload_exact": True,
        "actor_digest": actor_manifest["actor_digest"],
        "candidate_state_digest": _state_digest(state),
        "module_state_sha256": actor_manifest["module_state_sha256"],
        "actor_feature_manifest": ACTOR_FEATURE_MANIFEST_NAME,
        "uniform_state_digest": EXPECTED_UNIFORM_STATE_DIGEST,
        "uniform_predictions_sha256": EXPECTED_UNIFORM_PREDICTION_DIGEST,
        "uniform_lbfgs_closure_calls": EXPECTED_UNIFORM_CLOSURES,
        "uniform_terminal_loss": EXPECTED_UNIFORM_TERMINAL_LOSS,
        "candidate_predictions_sha256": EXPECTED_FINAL_PREDICTION_DIGEST,
        "gate_lbfgs_closure_calls": EXPECTED_GATE_CLOSURES,
        "gate_terminal_loss": EXPECTED_GATE_TERMINAL_LOSS,
        "no_legacy_shadow_runtime_dependency": True,
        "limitations": [copy.deepcopy(TRANSITION_ALIAS_LIMITATION)],
    }


def _save_candidate_exact(
    *,
    candidate_state: Mapping[str, Any],
    feature_names: Sequence[str],
    source_module: Any,
    destination: Path,
    bundle: RecoveryCorpusBundle,
    isolation: Mapping[str, Any],
) -> tuple[dict[str, Any], dict[str, Any], dict[str, Any]]:
    module = _new_w1024_module(
        source_module=source_module,
        state=candidate_state,
        inference_only=True,
    )
    reloaded = _save_rlmodule_no_clobber(module, destination)
    reloaded_state = _state_arrays(reloaded.get_state())
    if not _state_byte_exact(candidate_state, reloaded_state):
        raise V12R10RecoveryFitError("candidate core save/reload drifted")
    actor_manifest = _candidate_actor_manifest(
        state=reloaded_state,
        feature_names=feature_names,
        module_state=destination / "module_state.pkl",
    )
    _write_json_exclusive(destination / ACTOR_FEATURE_MANIFEST_NAME, actor_manifest)
    build_manifest = _candidate_build_manifest(
        state=reloaded_state,
        actor_manifest=actor_manifest,
        bundle=bundle,
        isolation=isolation,
    )
    _write_json_exclusive(destination / CANDIDATE_BUILD_MANIFEST_NAME, build_manifest)
    observed = {item.name for item in destination.iterdir() if item.is_file()}
    if observed != EXPECTED_CANDIDATE_FILES:
        raise V12R10RecoveryFitError(
            f"candidate five-file schema drifted: {sorted(observed)}"
        )
    final_record = _tree_record(destination)
    return final_record, actor_manifest, build_manifest


def _fit_summary(
    *,
    result: RecoveryFitResult,
    bundle: RecoveryCorpusBundle,
    module_record: Mapping[str, Any],
    actor_manifest: Mapping[str, Any],
    build_manifest: Mapping[str, Any],
    compatibility: Mapping[str, Any],
    pipeline_claim: Mapping[str, Any],
    worker_claim: Mapping[str, Any],
    protocol_freeze: Mapping[str, Any],
    execution_lock: Mapping[str, Any],
) -> dict[str, Any]:
    metrics = result.metrics
    isolation = result.tower_isolation
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": FIT_COMPLETE_STATUS,
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "fit_contract_id": contract.FIT_CONTRACT_ID,
        "candidate_selection_rule": contract.CANDIDATE_SELECTION_RULE,
        "hidden_dims": [TARGET_WIDTH, TARGET_WIDTH],
        "actor_feature_count": INPUT_WIDTH,
        "sample_count": len(bundle.arrays["observations"]),
        "reset_row_count": int(np.count_nonzero(bundle.arrays["reset_mask"])),
        "stratum_count": STRATUM_COUNT,
        "stratum_target_mass": STRATUM_MASS,
        "weight_audit": copy.deepcopy(dict(bundle.weight_audit)),
        "normalization_audit": copy.deepcopy(dict(result.normalization_audit)),
        "global_metrics": copy.deepcopy(metrics["global_metrics"]),
        "reset_max_abs_error": metrics["reset_max_abs_error"],
        "per_case_metrics": copy.deepcopy(metrics["per_case_metrics"]),
        "r4_failed_plus_metrics": copy.deepcopy(metrics["r4_failed_plus_metrics"]),
        "observer_case_metrics": copy.deepcopy(metrics["observer_case_metrics"]),
        "observer_plus_late_metrics": copy.deepcopy(
            metrics["observer_plus_late_metrics"]
        ),
        "worst_row": copy.deepcopy(metrics["worst_row"]),
        "actor_fit_count": 1,
        "actor_updates": 1,
        "critic_updates": 0,
        "ppo_updates": 0,
        "offline_h0_queries": 0,
        "environment_reset_calls": 0,
        "environment_step_calls": 0,
        "uniform_adamw_epochs": UNIFORM_ADAMW_EPOCHS,
        "uniform_lbfgs_max_iter": UNIFORM_LBFGS_MAX_ITER,
        "uniform_lbfgs_max_eval": UNIFORM_LBFGS_MAX_EVAL,
        "uniform_state_digest": result.uniform_attestation["candidate_state_digest"],
        "uniform_predictions_sha256": result.uniform_attestation[
            "candidate_predictions_sha256"
        ],
        "uniform_lbfgs_closure_calls": result.uniform_attestation[
            "lbfgs_closure_calls"
        ],
        "uniform_terminal_loss": result.uniform_attestation["terminal_loss"],
        "gate_adamw_epochs": GATE_ADAMW_EPOCHS,
        "gate_lbfgs_max_iter": GATE_LBFGS_MAX_ITER,
        "gate_lbfgs_max_eval": GATE_LBFGS_MAX_EVAL,
        "candidate_state_digest": result.final_attestation["candidate_state_digest"],
        "candidate_predictions_sha256": result.final_attestation[
            "candidate_predictions_sha256"
        ],
        "gate_lbfgs_closure_calls": result.final_attestation[
            "gate_lbfgs_closure_calls"
        ],
        "gate_terminal_loss": result.final_attestation["gate_terminal_loss"],
        "tower_a_r6_byte_exact": isolation["passed"],
        "cross_blocks_positive_zero": isolation["checks"]["cross_blocks_positive_zero"],
        "logstd_byte_exact": isolation["checks"]["logstd_bias_byte_exact"]
        and isolation["checks"]["logstd_full_weight_positive_zero"],
        "disabled_clock_columns_bit_zero": result.state_audit["checks"][
            "clock_columns_positive_zero"
        ],
        "save_reload_exact": compatibility["checks"]["checkpoint_state_byte_exact"],
        "runtime_save_reload_exact": compatibility["checks"][
            "checkpoint_state_byte_exact"
        ]
        and compatibility["checks"]["actor_prefix_runtime_exact"],
        "warm_start_actor_exact": compatibility["checks"]["warm_start_actor_exact"],
        "warm_start_critic_preserved": compatibility["checks"][
            "warm_start_critic_preserved"
        ],
        "standard_rlmodule": compatibility["checks"]["standard_module_class"],
        "no_legacy_shadow_runtime_dependency": True,
        "critic_present_in_candidate": False,
        "fallback_used": False,
        "sweep_used": False,
        "retry_used": False,
        "replica_used": False,
        "optimizer_audit": copy.deepcopy(dict(result.optimizer_audit)),
        "history": copy.deepcopy(list(result.history)),
        "state_audit": copy.deepcopy(dict(result.state_audit)),
        "tower_isolation": copy.deepcopy(dict(result.tower_isolation)),
        "runtime_warm_start_audit": copy.deepcopy(dict(compatibility)),
        "candidate_id": contract.candidate_id(module_record["tree_sha256"]),
        "candidate_module": copy.deepcopy(dict(module_record)),
        "r9_corpus": copy.deepcopy(bundle.source_records["r9_corpus"]),
        "r9_terminal_candidate": copy.deepcopy(
            bundle.source_records["r9_terminal_candidate"]
        ),
        "r6_source_candidate": copy.deepcopy(
            bundle.source_records["r6_source_candidate"]
        ),
        "actor_feature_manifest": copy.deepcopy(dict(actor_manifest)),
        "candidate_build_manifest": copy.deepcopy(dict(build_manifest)),
        "pipeline_claim": copy.deepcopy(dict(pipeline_claim)),
        "worker_claim": copy.deepcopy(dict(worker_claim)),
        "protocol_freeze": copy.deepcopy(dict(protocol_freeze)),
        "execution_lock": copy.deepcopy(dict(execution_lock)),
        "limitations": [copy.deepcopy(TRANSITION_ALIAS_LIMITATION)],
    }


def run_fit_stage(
    *,
    pipeline_claim_path: str | PurePath | Path,
    worker_claim_path: str | PurePath | Path,
    protocol_freeze_path: str | PurePath | Path,
    execution_lock_path: str | PurePath | Path,
    activity_callback: Callable[[str, int], None] | None = None,
) -> dict[str, Any]:
    """Execute and no-clobber publish the sole V12R10 actor fit."""

    destination = _resolve(contract.FIT_ROOT)
    if os.path.lexists(destination):
        raise V12R10RecoveryFitError("R10 fit destination exists/no-clobber")
    pipeline_claim = _record(pipeline_claim_path)
    worker_claim = _record(worker_claim_path)
    protocol_freeze = _record(protocol_freeze_path)
    execution_lock = _record(execution_lock_path)
    locked_before = attest_locked_inputs()
    bundle = load_locked_r9_corpus()
    sources = _load_source_modules()
    feature_names = bundle.arrays["actor_feature_names"].astype(str).tolist()
    if feature_names != sources["r6_manifest"]["actor_feature_names"]:
        raise V12R10RecoveryFitError("source/corpus actor feature order drifted")
    r6_tree_before = _tree_record(contract.R6_CANDIDATE_MODULE_PATH)
    r9_tree_before = _tree_record(contract.R9_CANDIDATE_MODULE_PATH)
    result = fit_recovery_actor(
        bundle=bundle,
        r6_state=sources["r6_state"],
        r9_initialization_state=sources["r9_state"],
        activity_callback=activity_callback,
    )
    if not _strict_equal(
        r6_tree_before, _tree_record(contract.R6_CANDIDATE_MODULE_PATH)
    ) or not _strict_equal(
        r9_tree_before, _tree_record(contract.R9_CANDIDATE_MODULE_PATH)
    ):
        raise V12R10RecoveryFitError("immutable source changed during fit")
    destination.mkdir(parents=True, exist_ok=False)
    candidate_path = _resolve(contract.CANDIDATE_MODULE_PATH)
    module_record, actor_manifest, build_manifest = _save_candidate_exact(
        candidate_state=result.candidate_state,
        feature_names=feature_names,
        source_module=sources["r6_module"],
        destination=candidate_path,
        bundle=bundle,
        isolation=result.tower_isolation,
    )
    compatibility = runtime_and_warm_start_audit(
        candidate_path=candidate_path,
        intended_state=result.candidate_state,
        source_module=sources["r6_module"],
        observations=bundle.arrays["observations"],
        feature_names=feature_names,
    )
    if not compatibility["passed"]:
        raise V12R10RecoveryFitError("candidate runtime/warm-start audit failed")
    recomputed = recompute_fit_metric_payload(bundle, result.candidate_state)
    if not _bytes_equal(recomputed.pop("predictions"), result.predictions):
        raise V12R10RecoveryFitError("saved candidate prediction bytes drifted")
    if not _strict_equal(recomputed, result.metrics):
        raise V12R10RecoveryFitError("saved candidate metrics drifted")
    summary = _fit_summary(
        result=result,
        bundle=bundle,
        module_record=module_record,
        actor_manifest=actor_manifest,
        build_manifest=build_manifest,
        compatibility=compatibility,
        pipeline_claim=pipeline_claim,
        worker_claim=worker_claim,
        protocol_freeze=protocol_freeze,
        execution_lock=execution_lock,
    )
    _assert_expected_fit_attestation(summary)
    if set(summary) != FIT_SUMMARY_FIELDS:
        raise V12R10RecoveryFitError("fit summary schema drifted")
    summary_path = _resolve(contract.FIT_SUMMARY_PATH)
    gate_path = _resolve(contract.FIT_GATE_PATH)
    receipt_path = _resolve(contract.FIT_RECEIPT_PATH)
    _write_json_exclusive(summary_path, summary)
    gate = contract.fit_gate(summary)
    _write_json_exclusive(gate_path, gate)
    if gate.get("passed") is not True:
        raise V12R10RecoveryFitError("R10 frozen offline fit gate failed")
    receipt = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": FIT_PASS_STATUS,
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "fit_contract_id": contract.FIT_CONTRACT_ID,
        "candidate_selection_rule": contract.CANDIDATE_SELECTION_RULE,
        "candidate_id": summary["candidate_id"],
        "candidate_module": module_record,
        "summary": _record(summary_path),
        "gate": _record(gate_path),
        "r9_corpus": copy.deepcopy(bundle.source_records["r9_corpus"]),
        "r9_terminal_candidate": copy.deepcopy(
            bundle.source_records["r9_terminal_candidate"]
        ),
        "r6_source_candidate": copy.deepcopy(
            bundle.source_records["r6_source_candidate"]
        ),
        "actor_feature_manifest": _record(candidate_path / ACTOR_FEATURE_MANIFEST_NAME),
        "candidate_build_manifest": _record(
            candidate_path / CANDIDATE_BUILD_MANIFEST_NAME
        ),
        "pipeline_claim": pipeline_claim,
        "worker_claim": worker_claim,
        "protocol_freeze": protocol_freeze,
        "execution_lock": execution_lock,
        "actor_fit_count": 1,
        "actor_updates": 1,
        "critic_updates": 0,
        "ppo_updates": 0,
        "offline_h0_queries": 0,
        "environment_reset_calls": 0,
        "environment_step_calls": 0,
        "runtime_save_reload_exact": True,
        "warm_start_actor_exact": True,
        "warm_start_critic_preserved": True,
        "no_legacy_shadow_runtime_dependency": True,
        "limitations": [copy.deepcopy(TRANSITION_ALIAS_LIMITATION)],
    }
    if set(receipt) != FIT_RECEIPT_FIELDS:
        raise V12R10RecoveryFitError("fit receipt schema drifted")
    _write_json_exclusive(receipt_path, receipt)
    if not _strict_equal(attest_locked_inputs(), locked_before):
        raise V12R10RecoveryFitError("locked inputs changed during fit")
    return receipt


def _assert_metric_payload(
    observed: Mapping[str, Any], recomputed: Mapping[str, Any], *, label: str
) -> None:
    drifted = [
        name
        for name in FIT_METRIC_FIELDS
        if not _strict_equal(observed.get(name), recomputed.get(name))
    ]
    if drifted:
        raise V12R10RecoveryFitError(f"{label} metrics drifted: {drifted}")


def _verify_optimizer_and_history(summary: Mapping[str, Any]) -> bool:
    audit = summary.get("optimizer_audit")
    history = summary.get("history")
    if not isinstance(audit, Mapping) or not isinstance(history, list) or not history:
        return False
    expected = {
        "fit_contract_id": contract.FIT_CONTRACT_ID,
        "seed": SEED,
        "torch_threads": TORCH_THREADS,
        "deterministic_algorithms": True,
        "full_batch": True,
        "terminal_state_only": True,
        "single_actor_fit": True,
        "actor_updates": 1,
        "critic_updates": 0,
        "ppo_updates": 0,
        "offline_h0_queries": 0,
        "environment_reset_calls": 0,
        "environment_step_calls": 0,
        "uniform_adamw_epochs": UNIFORM_ADAMW_EPOCHS,
        "uniform_adamw_boundaries": list(UNIFORM_ADAMW_BOUNDARIES),
        "uniform_adamw_rates": list(UNIFORM_ADAMW_RATES),
        "uniform_lbfgs_max_iter": UNIFORM_LBFGS_MAX_ITER,
        "uniform_lbfgs_max_eval": UNIFORM_LBFGS_MAX_EVAL,
        "uniform_lbfgs_closure_calls": EXPECTED_UNIFORM_CLOSURES,
        "gate_adamw_epochs": GATE_ADAMW_EPOCHS,
        "gate_adamw_boundaries": list(GATE_ADAMW_BOUNDARIES),
        "gate_adamw_rates": list(GATE_ADAMW_RATES),
        "gate_lbfgs_max_iter": GATE_LBFGS_MAX_ITER,
        "gate_lbfgs_max_eval": GATE_LBFGS_MAX_EVAL,
        "gate_lbfgs_closure_calls": EXPECTED_GATE_CLOSURES,
        "sweep": False,
        "retry": False,
        "repair": False,
        "early_stopping": False,
        "best_state_selection": False,
        "determinism_replica": False,
    }
    elapsed = audit.get("elapsed_seconds")
    terminal = history[-1]
    uniform_terminal = [
        row for row in history if row.get("stage") == "uniform_terminal_exact"
    ]
    return bool(
        all(_strict_equal(audit.get(name), value) for name, value in expected.items())
        and type(elapsed) is float
        and math.isfinite(elapsed)
        and elapsed >= 0.0
        and isinstance(terminal, Mapping)
        and terminal.get("stage") == "gate_terminal_final_state"
        and terminal.get("index") == EXPECTED_GATE_CLOSURES
        and terminal.get("loss") == EXPECTED_GATE_TERMINAL_LOSS
        and terminal.get("candidate_state_digest") == EXPECTED_FINAL_STATE_DIGEST
        and terminal.get("candidate_predictions_sha256")
        == EXPECTED_FINAL_PREDICTION_DIGEST
        and len(uniform_terminal) == 1
        and uniform_terminal[0].get("index") == EXPECTED_UNIFORM_CLOSURES
        and uniform_terminal[0].get("loss") == EXPECTED_UNIFORM_TERMINAL_LOSS
        and uniform_terminal[0].get("candidate_state_digest")
        == EXPECTED_UNIFORM_STATE_DIGEST
        and uniform_terminal[0].get("candidate_predictions_sha256")
        == EXPECTED_UNIFORM_PREDICTION_DIGEST
    )


def verify_fit_receipt(
    path: str | PurePath | Path = contract.FIT_RECEIPT_PATH,
) -> dict[str, Any]:
    """Semantically verify the published fit without invoking an optimizer."""

    if _resolve(path) != _resolve(contract.FIT_RECEIPT_PATH):
        raise V12R10RecoveryFitError("fit receipt path is not canonical")
    locked_before = attest_locked_inputs()
    bundle = load_locked_r9_corpus()
    sources = _load_source_modules()
    feature_names = bundle.arrays["actor_feature_names"].astype(str).tolist()
    if feature_names != sources["r6_manifest"]["actor_feature_names"]:
        raise V12R10RecoveryFitError("source/corpus feature order drifted")

    summary = _strict_json(contract.FIT_SUMMARY_PATH)
    gate = _strict_json(contract.FIT_GATE_PATH)
    receipt = _strict_json(path)
    candidate_path = _resolve(contract.CANDIDATE_MODULE_PATH)
    module_record = _tree_record(candidate_path)
    file_names = {row["path"] for row in module_record["files"]}
    if module_record["file_count"] != 5 or file_names != EXPECTED_CANDIDATE_FILES:
        raise V12R10RecoveryFitError("candidate five-file tree drifted")
    actor_manifest_path = candidate_path / ACTOR_FEATURE_MANIFEST_NAME
    build_manifest_path = candidate_path / CANDIDATE_BUILD_MANIFEST_NAME
    actor_manifest = _strict_json(actor_manifest_path)
    build_manifest = _strict_json(build_manifest_path)

    from ray.rllib.core.rl_module.rl_module import RLModule

    candidate = RLModule.from_checkpoint(candidate_path)
    candidate.eval()
    candidate_state = _state_arrays(candidate.get_state())
    state_audit = validate_w1024_state(candidate_state)
    isolation = tower_isolation_audit(candidate_state, sources["r6_state"])
    if not isolation["passed"]:
        raise V12R10RecoveryFitError("candidate tower isolation drifted")
    metrics = recompute_fit_metric_payload(bundle, candidate_state)
    predictions = metrics.pop("predictions")
    compatibility = runtime_and_warm_start_audit(
        candidate_path=candidate_path,
        intended_state=candidate_state,
        source_module=sources["r6_module"],
        observations=bundle.arrays["observations"],
        feature_names=feature_names,
    )
    if not compatibility["passed"]:
        raise V12R10RecoveryFitError("runtime/warm-start semantic closure failed")
    expected_actor_manifest = _candidate_actor_manifest(
        state=candidate_state,
        feature_names=feature_names,
        module_state=candidate_path / "module_state.pkl",
    )
    expected_build_manifest = _candidate_build_manifest(
        state=candidate_state,
        actor_manifest=expected_actor_manifest,
        bundle=bundle,
        isolation=isolation,
    )
    _assert_expected_fit_attestation(summary)
    _assert_metric_payload(summary, metrics, label="fit summary")
    expected_gate = contract.fit_gate(summary)
    expected_id = contract.candidate_id(module_record["tree_sha256"])
    expected_normalization = {
        "normalization": bundle.normalization.record(),
        "source_rows": "v8r1p1_base",
        "source_row_count": 3_000,
        "normalization_folded_into_residual_first_layer": True,
        "runtime_normalization_wrapper": False,
        "fold_equivalence_passed": True,
        "fold_max_abs_difference": summary.get("normalization_audit", {}).get(
            "fold_max_abs_difference"
        ),
        "fold_tolerance": FOLD_NUMERICAL_TOLERANCE,
    }
    fold_max = expected_normalization["fold_max_abs_difference"]
    normalization_valid = bool(
        type(fold_max) is float
        and math.isfinite(fold_max)
        and fold_max <= FOLD_NUMERICAL_TOLERANCE
        and _strict_equal(summary.get("normalization_audit"), expected_normalization)
    )
    static_summary = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": FIT_COMPLETE_STATUS,
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "fit_contract_id": contract.FIT_CONTRACT_ID,
        "candidate_selection_rule": contract.CANDIDATE_SELECTION_RULE,
        "hidden_dims": [TARGET_WIDTH, TARGET_WIDTH],
        "actor_feature_count": INPUT_WIDTH,
        "sample_count": 11_875,
        "reset_row_count": 26,
        "stratum_count": STRATUM_COUNT,
        "stratum_target_mass": STRATUM_MASS,
        "weight_audit": dict(bundle.weight_audit),
        "actor_fit_count": 1,
        "actor_updates": 1,
        "critic_updates": 0,
        "ppo_updates": 0,
        "offline_h0_queries": 0,
        "environment_reset_calls": 0,
        "environment_step_calls": 0,
        "uniform_adamw_epochs": UNIFORM_ADAMW_EPOCHS,
        "uniform_lbfgs_max_iter": UNIFORM_LBFGS_MAX_ITER,
        "uniform_lbfgs_max_eval": UNIFORM_LBFGS_MAX_EVAL,
        "gate_adamw_epochs": GATE_ADAMW_EPOCHS,
        "gate_lbfgs_max_iter": GATE_LBFGS_MAX_ITER,
        "gate_lbfgs_max_eval": GATE_LBFGS_MAX_EVAL,
        "tower_a_r6_byte_exact": True,
        "cross_blocks_positive_zero": True,
        "logstd_byte_exact": True,
        "disabled_clock_columns_bit_zero": True,
        "save_reload_exact": True,
        "runtime_save_reload_exact": True,
        "warm_start_actor_exact": True,
        "warm_start_critic_preserved": True,
        "standard_rlmodule": True,
        "no_legacy_shadow_runtime_dependency": True,
        "critic_present_in_candidate": False,
        "fallback_used": False,
        "sweep_used": False,
        "retry_used": False,
        "replica_used": False,
        "candidate_id": expected_id,
        "candidate_module": module_record,
        "r9_corpus": bundle.source_records["r9_corpus"],
        "r9_terminal_candidate": bundle.source_records["r9_terminal_candidate"],
        "r6_source_candidate": bundle.source_records["r6_source_candidate"],
        "actor_feature_manifest": actor_manifest,
        "candidate_build_manifest": build_manifest,
        "pipeline_claim": _record(contract.CLAIM_PATH),
        "protocol_freeze": _record(contract.PROTOCOL_FREEZE_PATH),
        "execution_lock": _record(contract.EXECUTION_LOCK_PATH),
        "limitations": [TRANSITION_ALIAS_LIMITATION],
    }
    summary_static_valid = all(
        _strict_equal(summary.get(name), value)
        for name, value in static_summary.items()
    )
    worker_summary = summary.get("worker_claim")
    worker_summary_valid = isinstance(worker_summary, Mapping) and _strict_equal(
        dict(worker_summary), _record(worker_summary.get("path", ""))
    )
    state_and_prediction_exact = bool(
        state_audit["state_digest"] == EXPECTED_FINAL_STATE_DIGEST
        and _prediction_digest(predictions) == EXPECTED_FINAL_PREDICTION_DIGEST
    )
    expected_receipt_static = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": FIT_PASS_STATUS,
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "fit_contract_id": contract.FIT_CONTRACT_ID,
        "candidate_selection_rule": contract.CANDIDATE_SELECTION_RULE,
        "candidate_id": expected_id,
        "candidate_module": module_record,
        "summary": _record(contract.FIT_SUMMARY_PATH),
        "gate": _record(contract.FIT_GATE_PATH),
        "r9_corpus": bundle.source_records["r9_corpus"],
        "r9_terminal_candidate": bundle.source_records["r9_terminal_candidate"],
        "r6_source_candidate": bundle.source_records["r6_source_candidate"],
        "actor_feature_manifest": _record(actor_manifest_path),
        "candidate_build_manifest": _record(build_manifest_path),
        "pipeline_claim": _record(contract.CLAIM_PATH),
        "protocol_freeze": _record(contract.PROTOCOL_FREEZE_PATH),
        "execution_lock": _record(contract.EXECUTION_LOCK_PATH),
        "actor_fit_count": 1,
        "actor_updates": 1,
        "critic_updates": 0,
        "ppo_updates": 0,
        "offline_h0_queries": 0,
        "environment_reset_calls": 0,
        "environment_step_calls": 0,
        "runtime_save_reload_exact": True,
        "warm_start_actor_exact": True,
        "warm_start_critic_preserved": True,
        "no_legacy_shadow_runtime_dependency": True,
        "limitations": [TRANSITION_ALIAS_LIMITATION],
    }
    receipt_static_valid = all(
        _strict_equal(receipt.get(name), value)
        for name, value in expected_receipt_static.items()
    )
    worker_receipt = receipt.get("worker_claim")
    worker_receipt_valid = isinstance(worker_receipt, Mapping) and _strict_equal(
        dict(worker_receipt), _record(worker_receipt.get("path", ""))
    )
    checks = {
        "input_closure": _strict_equal(attest_locked_inputs(), locked_before),
        "candidate_state_and_predictions": state_and_prediction_exact,
        "candidate_actor_manifest": _strict_equal(
            actor_manifest, expected_actor_manifest
        ),
        "candidate_build_manifest": _strict_equal(
            build_manifest, expected_build_manifest
        ),
        "runtime_save_reload_warm_start": compatibility["passed"],
        "metrics": True,
        "normalization": normalization_valid,
        "summary_schema_exact": set(summary) == FIT_SUMMARY_FIELDS,
        "receipt_schema_exact": set(receipt) == FIT_RECEIPT_FIELDS,
        "state_and_tower_audits": _strict_equal(summary.get("state_audit"), state_audit)
        and _strict_equal(summary.get("tower_isolation"), isolation),
        "persisted_runtime_audit": _strict_equal(
            summary.get("runtime_warm_start_audit"), compatibility
        ),
        "optimizer_attestation_without_replay": _verify_optimizer_and_history(summary),
        "summary_static": summary_static_valid and worker_summary_valid,
        "contract_gate": _strict_equal(gate, expected_gate)
        and gate.get("passed") is True,
        "receipt_static": receipt_static_valid and worker_receipt_valid,
        "r9_initialization_only_not_promoted": build_manifest.get(
            "r9_hidden_initialization_only"
        )
        is True
        and build_manifest.get("r9_terminal_candidate_promoted") is False,
        "no_legacy_shadow_runtime_dependency": summary.get(
            "no_legacy_shadow_runtime_dependency"
        )
        is True,
        "zero_optimizer_invocations_in_verifier": compatibility.get(
            "optimizer_invocations"
        )
        == 0,
    }
    if not all(checks.values()):
        failed = sorted(name for name, passed in checks.items() if not passed)
        raise V12R10RecoveryFitError(f"R10 fit verification failed: {failed}")
    return receipt


def verify_fit_stage() -> dict[str, Any]:
    """Runner-facing alias for the canonical receipt verifier."""

    return verify_fit_receipt(contract.FIT_RECEIPT_PATH)


__all__ = [
    "V12R10RecoveryFitError",
    "RecoveryCorpusBundle",
    "RecoveryFitResult",
    "UNIFORM_ADAMW_EPOCHS",
    "EXPECTED_UNIFORM_CLOSURES",
    "GATE_ADAMW_EPOCHS",
    "EXPECTED_GATE_CLOSURES",
    "attest_locked_inputs",
    "compute_reset3_equal_stratum_weights",
    "expected_stratum_ids",
    "fit_recovery_actor",
    "gate_adamw_rate",
    "load_locked_r9_corpus",
    "recompute_fit_metric_payload",
    "run_fit_stage",
    "runtime_and_warm_start_audit",
    "tower_isolation_audit",
    "uniform_adamw_rate",
    "validate_w1024_state",
    "verify_fit_receipt",
    "verify_fit_stage",
]
