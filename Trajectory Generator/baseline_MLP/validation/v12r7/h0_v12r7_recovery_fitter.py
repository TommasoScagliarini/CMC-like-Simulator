"""Single actor-only recovery fitter for the V12R7 lineage.

The fitter is deliberately execution-free on import.  Its only training entry
point consumes the locked 9,232-row R5 corpus, the 212-row forensic R4
``+0.20`` prefix, and the six V12R7 observer-label NPZ files.  The rows are
partitioned into exactly thirteen uniform, equal-mass strata and used for one
full-mean update of the frozen ordinary R6 35->512->512->2 actor.

No environment, teacher runtime, critic, PPO update, retry, sweep, fallback,
or post-fit repair surface exists here.
"""

from __future__ import annotations

import copy
import hashlib
import math
import os
import stat
import sys
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
    LOCAL_VALIDATION / "v12r3",
    LOCAL_VALIDATION / "v12r6",
    REVISION_ROOT,
):
    if str(_root) not in sys.path:
        sys.path.insert(0, str(_root))

import build_h0_v12r6_composite_actor as r6_builder  # noqa: E402
import h0_forensic_rollout as forensic  # noqa: E402
import h0_primary_split_v10s_fit as v10s_fit  # noqa: E402
import h0_primary_split_v11_weighted_fit as v11  # noqa: E402
import h0_primary_split_v12r3_recovery_weighted_fitter as v12r3_fit  # noqa: E402
import h0_v12r7_recovery_contract as contract  # noqa: E402
import warm_start  # noqa: E402


class V12R7RecoveryFitError(RuntimeError):
    """Raised when an R7 corpus, fit, or publication invariant drifts."""


FrozenNormalization = v11.FrozenNormalization
InMemoryFitResult = v11.InMemoryFitResult
RecoveryFitCorpus = v12r3_fit.RecoveryFitCorpus
DETERMINISTIC_TORCH_THREADS = v11.DETERMINISTIC_TORCH_THREADS

TOPOLOGY_ID = "V12R7_STANDARD_RECOVERY_W512_V1"
ACTOR_FEATURE_MANIFEST_NAME = "actor_feature_manifest.json"
CANDIDATE_BUILD_MANIFEST_NAME = "candidate_build_manifest.json"
CORPUS_MANIFEST_NAME = "corpus_manifest.json"
ADAPTATION_HISTORY_NAME = "adaptation_history.json"
ADAPTATION_REPORT_NAME = "adaptation_report.json"
FIT_COMPLETE_STATUS = "COMPLETE_H0_V12R7_RECOVERY_FIT"
FIT_PASS_STATUS = "PASS_H0_V12R7_RECOVERY_FIT_RECEIPT"
CANDIDATE_BUILD_STATUS = "H0_V12R7_RECOVERY_CANDIDATE_BUILD_PASS"

BASE_KEYS = {
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
}
R4_KEYS = {
    "observations",
    "actions",
    "reset_mask",
    "actor_feature_names",
    "case_ids",
    "step_indices",
    "tranche_ids",
    "origins",
}
OBSERVER_KEYS = {
    "observations",
    "actions",
    "reset_mask",
    "previous_penetration_m",
    "coverage_distance_rms_z",
    "coverage_nearest_reference_index",
    "coverage_ood_mask",
    "raw_sample_weights",
    "normalized_sample_weights",
    "actor_feature_names",
    "case_ids",
    "step_indices",
    "tranche_ids",
    "origins",
}
ROW_KEYS = (
    "observations",
    "actions",
    "reset_mask",
    "case_ids",
    "step_indices",
    "tranche_ids",
    "origins",
    "episode_ids",
)
EXPECTED_SOURCE_FILES = {
    "actor_feature_manifest.json",
    "composite_build_manifest.json",
    "class_and_ctor_args.pkl",
    "metadata.json",
    "module_state.pkl",
}
EXPECTED_CANDIDATE_FILES = {
    ACTOR_FEATURE_MANIFEST_NAME,
    CANDIDATE_BUILD_MANIFEST_NAME,
    "class_and_ctor_args.pkl",
    "metadata.json",
    "module_state.pkl",
}
ACTOR_MANIFEST_FIELDS = {
    "schema_version",
    "status",
    "topology_id",
    "fit_contract_id",
    "actor_feature_count",
    "actor_feature_names",
    "fcnet_hiddens",
    "disabled_clock_columns",
    "actor_digest",
    "module_state_sha256",
}
BUILD_MANIFEST_FIELDS = {
    "schema_version",
    "status",
    "passed",
    "protocol_id",
    "fit_contract_id",
    "topology_id",
    "source_candidate",
    "architecture",
    "stratum_count",
    "stratum_target_mass",
    "actor_fit_count",
    "actor_updates",
    "critic_updates",
    "ppo_updates",
    "logstd_byte_exact",
    "disabled_clock_columns_bit_zero",
    "save_reload_exact",
    "actor_digest",
    "module_state_sha256",
    "actor_feature_manifest",
    "corpus",
}
FIT_RECEIPT_FIELDS = {
    "schema_version",
    "status",
    "passed",
    "protocol_id",
    "fit_contract_id",
    "candidate_selection_rule",
    "candidate_id",
    "candidate_module",
    "summary",
    "gate",
    "corpus",
    "corpus_manifest",
    "adaptation_report",
    "adaptation_history",
    "actor_feature_manifest",
    "candidate_build_manifest",
    "pipeline_claim",
    "worker_claim",
    "protocol_freeze",
    "execution_lock",
    "actor_updates",
    "critic_updates",
    "ppo_updates",
}


@dataclass(frozen=True)
class RecoveryCorpusBundle:
    """Assembled fit corpus plus immutable stratum selections."""

    corpus: RecoveryFitCorpus
    stratum_ids: np.ndarray
    base_indices: Mapping[str, np.ndarray]
    r4_indices: np.ndarray
    observer_indices: Mapping[str, np.ndarray]
    observer_plus_late_indices: np.ndarray


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
        raise V12R7RecoveryFitError(f"path escapes repository root: {path}") from exc
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
        raise V12R7RecoveryFitError(f"path escapes repository root: {path}") from exc
    limit = len(parts) if include_leaf else max(0, len(parts) - 1)
    current = root
    for part in parts[:limit]:
        current /= part
        if os.path.lexists(current) and _is_link_or_reparse(current):
            raise V12R7RecoveryFitError(
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
        raise V12R7RecoveryFitError(f"unsafe or missing artifact: {resolved}")
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
        raise V12R7RecoveryFitError(f"unsafe or missing tree: {root}") from exc
    if not stat.S_ISDIR(root_status.st_mode) or _is_link_or_reparse(root):
        raise V12R7RecoveryFitError(f"unsafe or missing tree: {root}")
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
                raise V12R7RecoveryFitError(f"unsafe tree directory: {child}")
        for name in file_names:
            child = current / name
            if not _regular_file(child) or _is_link_or_reparse(child):
                raise V12R7RecoveryFitError(f"unsafe tree file: {child}")
            files.append(child)
    if not files:
        raise V12R7RecoveryFitError(f"empty artifact tree: {root}")
    files.sort(key=lambda item: item.relative_to(root).as_posix())
    digest = hashlib.sha256()
    rows = []
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


def _mapping(path: str | PurePath | Path) -> dict[str, Any]:
    resolved = _resolve(path)
    try:
        payload = forensic.strict_json_load(resolved)
    except Exception as exc:
        raise V12R7RecoveryFitError(f"invalid strict JSON: {resolved}") from exc
    if not isinstance(payload, Mapping):
        raise V12R7RecoveryFitError(f"expected JSON object: {resolved}")
    return dict(payload)


def _bytes_equal(left: Any, right: Any) -> bool:
    a = np.ascontiguousarray(np.asarray(left))
    b = np.ascontiguousarray(np.asarray(right))
    return a.dtype == b.dtype and a.shape == b.shape and a.tobytes() == b.tobytes()


def _positive_zero(value: Any) -> bool:
    array = np.ascontiguousarray(np.asarray(value))
    if array.dtype == np.float32:
        return bool(np.all(array.view(np.uint32) == 0))
    if array.dtype == np.float64:
        return bool(np.all(array.view(np.uint64) == 0))
    return False


def _read_npz_exact(
    path: str | PurePath | Path, expected_keys: set[str]
) -> dict[str, np.ndarray]:
    resolved = _resolve(path)
    _reject_link_components(resolved, include_leaf=True)
    if not _regular_file(resolved):
        raise V12R7RecoveryFitError(f"unsafe or missing NPZ: {resolved}")
    try:
        with np.load(resolved, allow_pickle=False) as archive:
            if set(archive.files) != expected_keys:
                raise V12R7RecoveryFitError(
                    f"NPZ key set drifted for {resolved}: {sorted(archive.files)}"
                )
            return {
                name: np.ascontiguousarray(archive[name].copy())
                for name in archive.files
            }
    except V12R7RecoveryFitError:
        raise
    except Exception as exc:
        raise V12R7RecoveryFitError(f"cannot read exact NPZ: {resolved}") from exc


def _finite_rows(arrays: Mapping[str, np.ndarray], names: Sequence[str]) -> bool:
    return all(np.all(np.isfinite(arrays[name])) for name in names)


def _core_shapes_valid(arrays: Mapping[str, np.ndarray], rows: int) -> bool:
    return (
        arrays["observations"].shape == (rows, contract.EXPECTED_ACTOR_FEATURES)
        and arrays["observations"].dtype == np.dtype(np.float32)
        and arrays["actions"].shape == (rows, contract.EXPECTED_ACTION_DIM)
        and arrays["actions"].dtype == np.dtype(np.float32)
        and arrays["reset_mask"].shape == (rows,)
        and arrays["reset_mask"].dtype == np.dtype(np.bool_)
        and arrays["actor_feature_names"].shape == (contract.EXPECTED_ACTOR_FEATURES,)
        and arrays["actor_feature_names"].dtype.kind == "U"
        and arrays["case_ids"].shape == (rows,)
        and arrays["case_ids"].dtype.kind == "U"
        and arrays["step_indices"].shape == (rows,)
        and arrays["step_indices"].dtype == np.dtype(np.int64)
        and arrays["tranche_ids"].shape == (rows,)
        and arrays["tranche_ids"].dtype.kind == "U"
        and arrays["origins"].shape == (rows,)
        and arrays["origins"].dtype.kind == "U"
        and _finite_rows(arrays, ("observations", "actions"))
    )


def _locked_artifact_exact(name: str, path: str | PurePath | Path) -> None:
    expected = contract.LOCKED_INPUTS[name]
    if _record(path) != {
        "path": expected["path"],
        "sha256": expected["sha256"],
        "size_bytes": expected["size_bytes"],
    }:
        raise V12R7RecoveryFitError(f"locked input drifted: {name}")


def attest_locked_inputs() -> dict[str, Any]:
    """Recompute every immutable predecessor binding used by the fitter."""

    _locked_artifact_exact("r6_terminal_ledger", contract.R6_TERMINAL_LEDGER)
    _locked_artifact_exact("r6_candidate_freeze", contract.R6_CANDIDATE_FREEZE)
    _locked_artifact_exact("base_corpus", contract.BASE_CORPUS_PATH)
    _locked_artifact_exact("r4_failed_plus_labels", contract.R4_PLUS_LABELS_PATH)
    source_tree = _tree_record(contract.R6_CANDIDATE_MODULE_PATH)
    expected_tree = contract.LOCKED_INPUTS["r6_candidate"]
    if (
        source_tree["tree_sha256"] != expected_tree["tree_sha256"]
        or source_tree["file_count"] != expected_tree["file_count"]
        or {row["path"] for row in source_tree["files"]} != EXPECTED_SOURCE_FILES
    ):
        raise V12R7RecoveryFitError("locked R6 candidate tree drifted")
    return {
        "r6_terminal_ledger": _record(contract.R6_TERMINAL_LEDGER),
        "r6_candidate_freeze": _record(contract.R6_CANDIDATE_FREEZE),
        "r6_candidate": source_tree,
        "base_corpus": _record(contract.BASE_CORPUS_PATH),
        "r4_failed_plus_labels": _record(contract.R4_PLUS_LABELS_PATH),
    }


def _load_base_piece() -> dict[str, np.ndarray]:
    _locked_artifact_exact("base_corpus", contract.BASE_CORPUS_PATH)
    arrays = _read_npz_exact(contract.BASE_CORPUS_PATH, BASE_KEYS)
    rows = int(contract.LOCKED_INPUTS["base_corpus"]["rows"])
    cases = arrays["case_ids"].astype(str)
    if (
        not _core_shapes_valid(arrays, rows)
        or arrays["episode_ids"].shape != (rows,)
        or arrays["episode_ids"].dtype.kind != "U"
        or arrays["raw_sample_weights"].shape != (rows,)
        or arrays["raw_sample_weights"].dtype != np.dtype(np.float64)
        or arrays["normalized_sample_weights"].shape != (rows,)
        or arrays["normalized_sample_weights"].dtype != np.dtype(np.float64)
        or arrays["training_indices"].shape != (rows,)
        or arrays["training_indices"].dtype != np.dtype(np.int64)
        or not np.array_equal(arrays["training_indices"], np.arange(rows))
        or set(cases) != set(contract.COLLECTION_CASE_IDS)
        or int(np.count_nonzero(arrays["reset_mask"])) != 19
        or len(set(arrays["episode_ids"].astype(str))) != 19
        or not _finite_rows(arrays, ("raw_sample_weights", "normalized_sample_weights"))
        or np.any(arrays["raw_sample_weights"] <= 0.0)
        or np.any(arrays["normalized_sample_weights"] <= 0.0)
        or not _positive_zero(arrays["observations"][:, 0])
        or not np.array_equal(
            arrays["observations"][:, 1],
            np.ones(rows, dtype=np.float32),
        )
        or any(not np.any(cases == case_id) for case_id in contract.COLLECTION_CASE_IDS)
    ):
        raise V12R7RecoveryFitError("locked R5 base corpus semantic drifted")
    return arrays


def _load_r4_piece(expected_features: np.ndarray) -> dict[str, np.ndarray]:
    _locked_artifact_exact("r4_failed_plus_labels", contract.R4_PLUS_LABELS_PATH)
    arrays = _read_npz_exact(contract.R4_PLUS_LABELS_PATH, R4_KEYS)
    rows = int(contract.LOCKED_INPUTS["r4_failed_plus_labels"]["rows"])
    if (
        not _core_shapes_valid(arrays, rows)
        or not _bytes_equal(arrays["actor_feature_names"], expected_features)
        or not np.array_equal(arrays["step_indices"], np.arange(1, rows + 1))
        or int(np.count_nonzero(arrays["reset_mask"])) != 1
        or not bool(arrays["reset_mask"][0])
        or set(arrays["case_ids"].astype(str)) != {"deterministic_offset_plus_0p20"}
        or set(arrays["tranche_ids"].astype(str)) != {"v12r4_p3_coverage"}
        or set(arrays["origins"].astype(str))
        != {"V12R4_SHIELDED_SAME_STATE_TEACHER_LABEL"}
        or not _positive_zero(arrays["observations"][:, 0])
        or not np.array_equal(
            arrays["observations"][:, 1],
            np.ones(rows, dtype=np.float32),
        )
    ):
        raise V12R7RecoveryFitError("locked R4 failure labels drifted")
    arrays["episode_ids"] = np.repeat(
        np.asarray(["v12r7_source:r4_failed_plus"], dtype="U192"), rows
    )
    return arrays


def observer_labels_path(case_id: str) -> PurePosixPath:
    if case_id not in contract.COLLECTION_CASE_IDS:
        raise V12R7RecoveryFitError(f"unknown observer case: {case_id!r}")
    return contract.observer_label_path(case_id)


def _load_observer_piece(
    case_id: str,
    expected_features: np.ndarray,
    *,
    path: str | PurePath | Path | None = None,
) -> dict[str, np.ndarray]:
    source = observer_labels_path(case_id) if path is None else path
    arrays = _read_npz_exact(source, OBSERVER_KEYS)
    rows = len(arrays["observations"])
    numeric = (
        "previous_penetration_m",
        "coverage_distance_rms_z",
        "raw_sample_weights",
        "normalized_sample_weights",
    )
    if (
        not contract.MINIMUM_RECOVERABLE_PREFIX_STEPS <= rows <= contract.EXPECTED_STEPS
        or not _core_shapes_valid(arrays, rows)
        or not _bytes_equal(arrays["actor_feature_names"], expected_features)
        or not np.array_equal(arrays["step_indices"], np.arange(1, rows + 1))
        or int(np.count_nonzero(arrays["reset_mask"])) != 1
        or not bool(arrays["reset_mask"][0])
        or set(arrays["case_ids"].astype(str)) != {case_id}
        or set(arrays["tranche_ids"].astype(str)) != {"observer_probe_p0"}
        or not np.array_equal(
            arrays["origins"].astype(str),
            np.asarray(
                [f"pure_observer:p0:{case_id}:{step}" for step in range(1, rows + 1)]
            ),
        )
        or any(arrays[name].shape != (rows,) for name in numeric)
        or arrays["previous_penetration_m"].dtype != np.dtype(np.float64)
        or arrays["coverage_distance_rms_z"].dtype != np.dtype(np.float64)
        or arrays["coverage_nearest_reference_index"].shape != (rows,)
        or arrays["coverage_nearest_reference_index"].dtype != np.dtype(np.int64)
        or arrays["coverage_ood_mask"].shape != (rows,)
        or arrays["coverage_ood_mask"].dtype != np.dtype(np.bool_)
        or arrays["raw_sample_weights"].dtype != np.dtype(np.float64)
        or arrays["normalized_sample_weights"].dtype != np.dtype(np.float64)
        or not _finite_rows(arrays, numeric)
        or np.any(arrays["previous_penetration_m"] < 0.0)
        or np.any(arrays["coverage_distance_rms_z"] < 0.0)
        or np.any(arrays["coverage_nearest_reference_index"] < 0)
        or np.any(arrays["raw_sample_weights"] <= 0.0)
        or np.any(arrays["normalized_sample_weights"] <= 0.0)
        or not _positive_zero(arrays["observations"][:, 0])
        or not np.array_equal(
            arrays["observations"][:, 1],
            np.ones(rows, dtype=np.float32),
        )
    ):
        raise V12R7RecoveryFitError(f"observer label NPZ drifted: {case_id}")
    arrays["episode_ids"] = np.repeat(
        np.asarray([f"v12r7_observer:{case_id}"], dtype="U192"), rows
    )
    return arrays


def expected_stratum_ids() -> tuple[str, ...]:
    return (
        *(f"base::{case_id}" for case_id in contract.COLLECTION_CASE_IDS),
        *(f"observer::{case_id}" for case_id in contract.COLLECTION_CASE_IDS),
        "r4_failure::deterministic_offset_plus_0p20",
    )


def compute_equal_stratum_weights(
    stratum_ids: Any,
) -> tuple[np.ndarray, dict[str, Any]]:
    """Assign uniform row weight within thirteen equal 500-mass strata."""

    strata = np.asarray(stratum_ids)
    if strata.ndim != 1 or strata.dtype.kind != "U" or len(strata) == 0:
        raise V12R7RecoveryFitError("stratum ids must be a non-empty Unicode vector")
    expected = expected_stratum_ids()
    if set(strata.astype(str)) != set(expected):
        raise V12R7RecoveryFitError("stratum id set drifted")
    weights = np.empty(len(strata), dtype=np.float64)
    masses: dict[str, float] = {}
    row_counts: dict[str, int] = {}
    row_weights: dict[str, float] = {}
    target = float(contract.FIT["stratum_target_mass"])
    for stratum_id in expected:
        selected = np.flatnonzero(strata == stratum_id)
        if len(selected) == 0:
            raise V12R7RecoveryFitError(f"empty stratum: {stratum_id}")
        row_weight = np.float64(target / len(selected))
        weights[selected] = row_weight
        mass = math.fsum(float(weights[index]) for index in selected)
        if not math.isclose(mass, target, rel_tol=0.0, abs_tol=1.0e-9):
            raise V12R7RecoveryFitError(f"stratum mass failed closure: {stratum_id}")
        masses[stratum_id] = float(mass)
        row_counts[stratum_id] = int(len(selected))
        row_weights[stratum_id] = float(row_weight)
    total = math.fsum(float(value) for value in weights)
    expected_total = target * len(expected)
    if (
        not np.all(np.isfinite(weights))
        or np.any(weights <= 0.0)
        or not math.isclose(total, expected_total, rel_tol=0.0, abs_tol=1.0e-8)
    ):
        raise V12R7RecoveryFitError("equal-stratum weights failed closure")
    return np.ascontiguousarray(weights), {
        "policy": contract.FIT["stratum_policy"],
        "within_stratum_weighting": contract.FIT["within_stratum_weighting"],
        "stratum_order": list(expected),
        "stratum_count": len(expected),
        "stratum_target_mass": target,
        "stratum_row_counts": row_counts,
        "stratum_uniform_row_weights": row_weights,
        "stratum_mass": masses,
        "total_sample_mass": float(total),
        "stratum_ids_sha256": v10s_fit.array_sha256(strata),
        "sample_weights_sha256": v10s_fit.array_sha256(weights),
    }


def load_recovery_corpus() -> RecoveryCorpusBundle:
    """Load and bind the exact thirteen-source R7 recovery corpus."""

    locked = attest_locked_inputs()
    base = _load_base_piece()
    features = np.ascontiguousarray(base["actor_feature_names"], dtype="U64")
    r4 = _load_r4_piece(features)
    observers = {
        case_id: _load_observer_piece(case_id, features)
        for case_id in contract.COLLECTION_CASE_IDS
    }
    pieces = [
        base,
        r4,
        *(observers[case_id] for case_id in contract.COLLECTION_CASE_IDS),
    ]
    combined = {
        name: np.ascontiguousarray(np.concatenate([piece[name] for piece in pieces]))
        for name in ROW_KEYS
    }
    stratum_parts = [
        np.asarray(
            [f"base::{case_id}" for case_id in base["case_ids"].astype(str)],
            dtype="U96",
        ),
        np.repeat(
            np.asarray(["r4_failure::deterministic_offset_plus_0p20"], dtype="U96"),
            len(r4["observations"]),
        ),
        *(
            np.repeat(
                np.asarray([f"observer::{case_id}"], dtype="U96"),
                len(observers[case_id]["observations"]),
            )
            for case_id in contract.COLLECTION_CASE_IDS
        ),
    ]
    stratum_ids = np.ascontiguousarray(np.concatenate(stratum_parts), dtype="U96")
    weights, weight_audit = compute_equal_stratum_weights(stratum_ids)
    rows = len(combined["observations"])
    combined["raw_sample_weights"] = np.ones(rows, dtype=np.float64)
    combined["normalized_sample_weights"] = weights
    base_stop = len(base["observations"])
    r4_stop = base_stop + len(r4["observations"])
    base_indices = {
        case_id: np.flatnonzero(base["case_ids"].astype(str) == case_id)
        for case_id in contract.COLLECTION_CASE_IDS
    }
    r4_indices = np.arange(base_stop, r4_stop, dtype=np.int64)
    observer_indices: dict[str, np.ndarray] = {}
    cursor = r4_stop
    for case_id in contract.COLLECTION_CASE_IDS:
        count = len(observers[case_id]["observations"])
        observer_indices[case_id] = np.arange(cursor, cursor + count, dtype=np.int64)
        cursor += count
    plus = observer_indices["deterministic_offset_plus_0p20"]
    plus_steps = combined["step_indices"][plus]
    observer_plus_late = plus[plus_steps >= 140]
    if (
        cursor != rows
        or len(observer_plus_late) == 0
        or int(np.count_nonzero(combined["reset_mask"]))
        != 20 + len(contract.COLLECTION_CASE_IDS)
        or not np.all(np.isfinite(combined["observations"]))
        or not np.all(np.isfinite(combined["actions"]))
    ):
        raise V12R7RecoveryFitError("assembled recovery corpus failed closure")
    corpus = RecoveryFitCorpus(
        observations=combined["observations"],
        actions=combined["actions"],
        reset_mask=combined["reset_mask"],
        actor_feature_names=features,
        case_ids=combined["case_ids"],
        step_indices=combined["step_indices"],
        tranche_ids=combined["tranche_ids"],
        origins=combined["origins"],
        episode_ids=combined["episode_ids"],
        raw_sample_weights=combined["raw_sample_weights"],
        normalized_sample_weights=weights,
        source_records={
            "locked_inputs": locked,
            "observer_labels": {
                case_id: _record(observer_labels_path(case_id))
                for case_id in contract.COLLECTION_CASE_IDS
            },
        },
        probe_label_bindings=(),
        collection_bindings=tuple(
            _record(observer_labels_path(case_id))
            for case_id in contract.COLLECTION_CASE_IDS
        ),
        audit={
            "all_finite": True,
            "component_order": [
                "r5_base",
                "r4_failed_plus",
                *(f"observer::{case_id}" for case_id in contract.COLLECTION_CASE_IDS),
            ],
            "base_rows": base_stop,
            "r4_rows": len(r4_indices),
            "observer_rows": {
                case_id: len(observer_indices[case_id])
                for case_id in contract.COLLECTION_CASE_IDS
            },
            "observer_plus_late_rows": len(observer_plus_late),
            "weight_audit": weight_audit,
        },
    )
    return RecoveryCorpusBundle(
        corpus=corpus,
        stratum_ids=stratum_ids,
        base_indices=base_indices,
        r4_indices=r4_indices,
        observer_indices=observer_indices,
        observer_plus_late_indices=observer_plus_late,
    )


def adamw_learning_rate(epoch: int) -> float:
    optimizer = contract.FIT["optimizer"]
    epochs = int(optimizer["adamw_epochs"])
    if type(epoch) is not int or not 1 <= epoch <= epochs:
        raise V12R7RecoveryFitError(f"AdamW epoch outside 1..{epochs}: {epoch!r}")
    if epoch <= 1000:
        return 3.0e-4
    if epoch <= 1700:
        return 1.0e-4
    return 3.0e-5


def _expected_state_shapes() -> dict[str, tuple[int, ...]]:
    return {
        "pi_encoder.0.weight": (512, 35),
        "pi_encoder.0.bias": (512,),
        "pi_encoder.2.weight": (512, 512),
        "pi_encoder.2.bias": (512,),
        "pi.0.0.weight": (512, 35),
        "pi.0.0.bias": (512,),
        "pi.0.2.weight": (512, 512),
        "pi.0.2.bias": (512,),
        "pi.1.weight": (4, 512),
        "pi.1.bias": (4,),
    }


def validate_source_r6_state(state: Mapping[str, Any]) -> dict[str, Any]:
    shapes = _expected_state_shapes()
    if set(state) != set(shapes):
        raise V12R7RecoveryFitError("R6 actor state key set drifted")
    arrays: dict[str, np.ndarray] = {}
    for name in shapes:
        value = state[name]
        if hasattr(value, "detach"):
            value = value.detach()
        if hasattr(value, "cpu"):
            value = value.cpu()
        array = np.ascontiguousarray(np.asarray(value))
        if array.dtype != np.dtype(np.float32):
            raise V12R7RecoveryFitError(f"R6 actor tensor dtype drifted: {name}")
        arrays[name] = array
    if any(arrays[name].shape != shape for name, shape in shapes.items()):
        raise V12R7RecoveryFitError("R6 actor state shape drifted")
    aliases = (
        ("pi_encoder.0.weight", "pi.0.0.weight"),
        ("pi_encoder.0.bias", "pi.0.0.bias"),
        ("pi_encoder.2.weight", "pi.0.2.weight"),
        ("pi_encoder.2.bias", "pi.0.2.bias"),
    )
    if (
        not all(np.all(np.isfinite(value)) for value in arrays.values())
        or not all(_bytes_equal(arrays[a], arrays[b]) for a, b in aliases)
        or not _positive_zero(
            arrays["pi_encoder.0.weight"][:, contract.DISABLED_CLOCK_COLUMNS]
        )
        or not _positive_zero(arrays["pi.1.weight"][2:])
    ):
        raise V12R7RecoveryFitError("R6 actor state semantic drifted")
    sigma = np.exp(arrays["pi.1.bias"][2:].astype(np.float64))
    if not np.allclose(
        sigma,
        np.repeat(contract.EXPECTED_SIGMA, contract.EXPECTED_ACTION_DIM),
        rtol=0.0,
        atol=1.0e-9,
    ):
        raise V12R7RecoveryFitError("R6 actor logstd/sigma drifted")
    return {
        "hidden_dims": [512, 512],
        "actor_feature_count": 35,
        "action_dim": 2,
        "encoder_aliases_byte_exact": True,
        "disabled_clock_columns_bit_zero": True,
        "logstd_weight_bit_zero": True,
        "sigma": sigma.astype(float).tolist(),
        "actor_digest": warm_start.actor_state_digest(arrays),
    }


def _load_source_module_and_state() -> tuple[Any, dict[str, Any], dict[str, Any]]:
    from ray.rllib.core.rl_module.rl_module import RLModule

    source_path = _resolve(contract.R6_CANDIDATE_MODULE_PATH)
    module = RLModule.from_checkpoint(source_path)
    module.eval()
    state = v11._clone_state(module.get_state())
    state_audit = validate_source_r6_state(state)
    if (
        list(module.model_config.get("fcnet_hiddens", ())) != [512, 512]
        or str(module.model_config.get("fcnet_activation", "")).lower() != "tanh"
        or module.inference_only is not True
    ):
        raise V12R7RecoveryFitError("R6 source module topology drifted")
    source_manifest = _mapping(source_path / ACTOR_FEATURE_MANIFEST_NAME)
    feature_names = source_manifest.get("actor_feature_names")
    if (
        not isinstance(feature_names, list)
        or len(feature_names) != contract.EXPECTED_ACTOR_FEATURES
        or source_manifest.get("actor_digest") != state_audit["actor_digest"]
    ):
        raise V12R7RecoveryFitError("R6 source actor manifest drifted")
    return module, state, source_manifest


def _new_normalized_model(
    source_state: Mapping[str, Any], normalization: FrozenNormalization
) -> Any:
    import torch
    from torch import nn

    validate_source_r6_state(source_state)
    model = nn.Sequential(
        nn.Linear(35, 512),
        nn.Tanh(),
        nn.Linear(512, 512),
        nn.Tanh(),
        nn.Linear(512, 2),
    )
    mean = torch.as_tensor(normalization.mean, dtype=torch.float32)
    std = torch.as_tensor(normalization.std, dtype=torch.float32)
    source_w0 = torch.as_tensor(
        np.asarray(source_state["pi_encoder.0.weight"]), dtype=torch.float32
    )
    source_b0 = torch.as_tensor(
        np.asarray(source_state["pi_encoder.0.bias"]), dtype=torch.float32
    )
    with torch.no_grad():
        model[0].weight.copy_(source_w0 * std[None, :])
        model[0].bias.copy_(source_b0 + source_w0 @ mean)
        model[2].weight.copy_(
            torch.as_tensor(source_state["pi_encoder.2.weight"], dtype=torch.float32)
        )
        model[2].bias.copy_(
            torch.as_tensor(source_state["pi_encoder.2.bias"], dtype=torch.float32)
        )
        model[4].weight.copy_(
            torch.as_tensor(source_state["pi.1.weight"], dtype=torch.float32)[:2]
        )
        model[4].bias.copy_(
            torch.as_tensor(source_state["pi.1.bias"], dtype=torch.float32)[:2]
        )
    if not _positive_zero(
        model[0].weight.detach().cpu().numpy()[:, contract.DISABLED_CLOCK_COLUMNS]
    ):
        raise V12R7RecoveryFitError("normalized model re-enabled clock columns")
    return model


def _fold_normalization_into_state(
    model: Any,
    source_state: Mapping[str, Any],
    normalization: FrozenNormalization,
) -> tuple[dict[str, Any], dict[str, Any]]:
    import torch

    normalized_w0 = np.ascontiguousarray(
        model[0].weight.detach().cpu().numpy(), dtype=np.float32
    )
    normalized_b0 = np.ascontiguousarray(
        model[0].bias.detach().cpu().numpy(), dtype=np.float32
    )
    if not _positive_zero(normalized_w0[:, contract.DISABLED_CLOCK_COLUMNS]):
        raise V12R7RecoveryFitError("trained clock columns are not positive zero")
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
        raise V12R7RecoveryFitError("folded clock columns are not positive zero")
    candidate = v11._clone_state(source_state)
    second_weight = np.ascontiguousarray(
        model[2].weight.detach().cpu().numpy(), dtype=np.float32
    )
    second_bias = np.ascontiguousarray(
        model[2].bias.detach().cpu().numpy(), dtype=np.float32
    )
    for prefix in ("pi_encoder", "pi.0"):
        candidate[f"{prefix}.0.weight"] = raw_w0.copy()
        candidate[f"{prefix}.0.bias"] = raw_b0.copy()
        candidate[f"{prefix}.2.weight"] = second_weight.copy()
        candidate[f"{prefix}.2.bias"] = second_bias.copy()
    output_weight = np.ascontiguousarray(
        np.asarray(source_state["pi.1.weight"]), dtype=np.float32
    ).copy()
    output_bias = np.ascontiguousarray(
        np.asarray(source_state["pi.1.bias"]), dtype=np.float32
    ).copy()
    output_weight[:2] = np.ascontiguousarray(
        model[4].weight.detach().cpu().numpy(), dtype=np.float32
    )
    output_bias[:2] = np.ascontiguousarray(
        model[4].bias.detach().cpu().numpy(), dtype=np.float32
    )
    candidate["pi.1.weight"] = output_weight
    candidate["pi.1.bias"] = output_bias
    validate_source_r6_state(candidate)
    return candidate, {
        "normalization_folded_into_first_layer": True,
        "runtime_normalization_wrapper": False,
        "normalized_clock_columns_bit_zero": True,
        "folded_clock_columns_bit_zero": True,
    }


def full_mean_update_audit(
    source_state: Mapping[str, Any], candidate_state: Mapping[str, Any]
) -> dict[str, Any]:
    validate_source_r6_state(source_state)
    validate_source_r6_state(candidate_state)
    hidden_keys = (
        "pi_encoder.0.weight",
        "pi_encoder.0.bias",
        "pi_encoder.2.weight",
        "pi_encoder.2.bias",
    )
    hidden_changed = any(
        not _bytes_equal(source_state[name], candidate_state[name])
        for name in hidden_keys
    )
    mean_changed = not _bytes_equal(
        np.asarray(source_state["pi.1.weight"])[:2],
        np.asarray(candidate_state["pi.1.weight"])[:2],
    ) or not _bytes_equal(
        np.asarray(source_state["pi.1.bias"])[:2],
        np.asarray(candidate_state["pi.1.bias"])[:2],
    )
    logstd_exact = _bytes_equal(
        np.asarray(source_state["pi.1.weight"])[2:],
        np.asarray(candidate_state["pi.1.weight"])[2:],
    ) and _bytes_equal(
        np.asarray(source_state["pi.1.bias"])[2:],
        np.asarray(candidate_state["pi.1.bias"])[2:],
    )
    aliases_exact = all(
        _bytes_equal(candidate_state[left], candidate_state[right])
        for left, right in (
            ("pi_encoder.0.weight", "pi.0.0.weight"),
            ("pi_encoder.0.bias", "pi.0.0.bias"),
            ("pi_encoder.2.weight", "pi.0.2.weight"),
            ("pi_encoder.2.bias", "pi.0.2.bias"),
        )
    )
    clock_zero = _positive_zero(
        np.asarray(candidate_state["pi_encoder.0.weight"])[
            :, contract.DISABLED_CLOCK_COLUMNS
        ]
    )
    passed = bool(
        hidden_changed
        and mean_changed
        and logstd_exact
        and aliases_exact
        and clock_zero
    )
    return {
        "passed": passed,
        "state_key_set_byte_exact": set(source_state) == set(candidate_state),
        "changes_confined_to_full_mean_network": passed,
        "hidden_mean_network_changed": bool(hidden_changed),
        "mean_output_changed": bool(mean_changed),
        "logstd_parameter_rows_byte_exact": bool(logstd_exact),
        "encoder_aliases_byte_exact": bool(aliases_exact),
        "disabled_clock_columns_bit_zero": bool(clock_zero),
        "critic_present": False,
        "critic_updates": 0,
        "ppo_updates": 0,
    }


def fit_recovery_full_mean_in_memory(
    *,
    source_state: Mapping[str, Any],
    observations: Any,
    targets: Any,
    reset_mask: Any,
    sample_weights: Any,
    normalization: FrozenNormalization,
    activity_callback: Callable[[str, int], None] | None = None,
) -> InMemoryFitResult:
    """Run the sole fixed AdamW(2000)+LBFGS(300/600) R7 fit."""

    import torch

    validate_source_r6_state(source_state)
    raw = np.ascontiguousarray(observations, dtype=np.float32)
    labels = np.ascontiguousarray(targets, dtype=np.float32)
    reset = np.ascontiguousarray(reset_mask, dtype=np.bool_)
    weights_np = np.ascontiguousarray(sample_weights, dtype=np.float64)
    rows = len(raw)
    if (
        raw.shape != (rows, contract.EXPECTED_ACTOR_FEATURES)
        or labels.shape != (rows, contract.EXPECTED_ACTION_DIM)
        or reset.shape != (rows,)
        or not np.any(reset)
        or weights_np.shape != (rows,)
        or not np.all(np.isfinite(raw))
        or not np.all(np.isfinite(labels))
        or not np.all(np.isfinite(weights_np))
        or np.any(weights_np <= 0.0)
    ):
        raise V12R7RecoveryFitError("R7 weighted fit arrays are malformed")
    normalized = v11.normalized_observations(raw, normalization)
    optimizer_spec = contract.FIT["optimizer"]
    previous_threads = torch.get_num_threads()
    previous_deterministic = torch.are_deterministic_algorithms_enabled()
    torch.set_num_threads(DETERMINISTIC_TORCH_THREADS)
    torch.use_deterministic_algorithms(True)
    try:
        torch.manual_seed(int(optimizer_spec["seed"]))
        model = _new_normalized_model(source_state, normalization)
        if activity_callback is not None:
            activity_callback("actor_updates", 1)
        x = torch.as_tensor(normalized, dtype=torch.float32)
        y = torch.as_tensor(labels, dtype=torch.float32)
        weights = torch.as_tensor(weights_np, dtype=torch.float64)
        weight_sum = torch.sum(weights)
        history: list[dict[str, Any]] = []
        adamw = torch.optim.AdamW(
            model.parameters(),
            lr=3.0e-4,
            weight_decay=float(optimizer_spec["weight_decay"]),
        )
        adam_milestones = {1, 250, 500, 1000, 1400, 1700, 2000}
        for epoch in range(1, 2001):
            rate = adamw_learning_rate(epoch)
            for group in adamw.param_groups:
                group["lr"] = rate
            adamw.zero_grad(set_to_none=True)
            prediction = model(x)
            row_loss = torch.mean(torch.square(prediction - y), dim=1)
            loss = torch.sum(weights * row_loss) / weight_sum
            if not torch.isfinite(loss):
                raise V12R7RecoveryFitError(f"non-finite AdamW loss at epoch {epoch}")
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), 10.0)
            adamw.step()
            if activity_callback is not None:
                activity_callback("adamw_epochs_completed", 1)
            if epoch in adam_milestones:
                v11._milestone(history, stage="adamw", index=epoch, loss=loss, lr=rate)
        lbfgs = torch.optim.LBFGS(
            model.parameters(),
            lr=0.7,
            max_iter=300,
            max_eval=600,
            tolerance_grad=1.0e-10,
            tolerance_change=1.0e-12,
            history_size=50,
            line_search_fn="strong_wolfe",
        )
        closure_calls = 0
        last_lbfgs_loss: Any = None

        def closure() -> Any:
            nonlocal closure_calls, last_lbfgs_loss
            lbfgs.zero_grad(set_to_none=True)
            prediction = model(x)
            row_loss = torch.mean(torch.square(prediction - y), dim=1)
            value = torch.sum(weights * row_loss) / weight_sum
            if not torch.isfinite(value):
                raise V12R7RecoveryFitError(
                    f"non-finite LBFGS loss at closure {closure_calls + 1}"
                )
            value.backward()
            closure_calls += 1
            if activity_callback is not None:
                activity_callback("lbfgs_closure_calls", 1)
            last_lbfgs_loss = value
            if closure_calls in {1, 50, 100, 200, 300, 400, 600}:
                v11._milestone(
                    history,
                    stage="lbfgs_closure",
                    index=closure_calls,
                    loss=value,
                    lr=0.7,
                )
            return value

        lbfgs.step(closure)
        if last_lbfgs_loss is None:
            raise V12R7RecoveryFitError("LBFGS never evaluated the objective")
        v11._milestone(
            history,
            stage="lbfgs_final",
            index=closure_calls,
            loss=last_lbfgs_loss,
            lr=0.7,
        )
        candidate_state, fold_audit = _fold_normalization_into_state(
            model, source_state, normalization
        )
        with torch.no_grad():
            normalized_prediction = np.ascontiguousarray(
                model(x).cpu().numpy(), dtype=np.float32
            )
        runtime_logits = v11._state_logits(candidate_state, raw)
        runtime_prediction = np.ascontiguousarray(
            runtime_logits[:, : contract.EXPECTED_ACTION_DIM], dtype=np.float32
        )
        normalization_audit = {
            **fold_audit,
            **v11.fold_equivalence_audit(normalized_prediction, runtime_prediction),
            "normalization": normalization.record(),
        }
        preservation = full_mean_update_audit(source_state, candidate_state)
        source_logits = v11._state_logits(source_state, raw)
        preservation = {
            **preservation,
            "logstd_outputs_byte_exact": bool(
                source_logits[:, 2:].tobytes() == runtime_logits[:, 2:].tobytes()
            ),
        }
        preservation["passed"] = bool(
            preservation["passed"] and preservation["logstd_outputs_byte_exact"]
        )
        if not preservation["passed"]:
            raise V12R7RecoveryFitError("mean-only preservation audit failed")
        return InMemoryFitResult(
            candidate_state=candidate_state,
            predictions=runtime_prediction,
            metrics=v11.prediction_metrics(runtime_prediction, labels, reset),
            normalization=normalization,
            normalization_audit=normalization_audit,
            preservation_audit=preservation,
            history=tuple(history),
            optimizer_audit={
                "fit_contract_id": contract.FIT_CONTRACT_ID,
                "seed": int(optimizer_spec["seed"]),
                "full_batch": True,
                "sample_count": rows,
                "explicit_sample_weights": True,
                "sample_weight_dtype": "float64",
                "sample_weights_sha256": v10s_fit.array_sha256(weights_np),
                "total_sample_mass": float(
                    math.fsum(float(value) for value in weights_np)
                ),
                "adamw_epochs": 2000,
                "adamw_learning_rates": list(optimizer_spec["adamw_learning_rates"]),
                "adamw_boundaries": list(optimizer_spec["adamw_boundaries"]),
                "adamw_weight_decay": float(optimizer_spec["weight_decay"]),
                "gradient_clip_norm": 10.0,
                "lbfgs_lr": 0.7,
                "lbfgs_max_iter": 300,
                "lbfgs_max_eval": 600,
                "lbfgs_tolerance_grad": 1.0e-10,
                "lbfgs_tolerance_change": 1.0e-12,
                "lbfgs_history_size": 50,
                "lbfgs_line_search": "strong_wolfe",
                "lbfgs_closure_calls": closure_calls,
                "actor_fit_count": 1,
                "actor_updates": 1,
                "critic_updates": 0,
                "ppo_updates": 0,
                "fallback": False,
                "sweep": False,
                "torch_threads": DETERMINISTIC_TORCH_THREADS,
                "deterministic_algorithms_enabled": True,
            },
        )
    except V12R7RecoveryFitError:
        raise
    except Exception as exc:
        raise V12R7RecoveryFitError("fixed R7 recovery fit failed") from exc
    finally:
        torch.use_deterministic_algorithms(previous_deterministic)
        torch.set_num_threads(previous_threads)


def _metric_pair(
    predictions: np.ndarray, targets: np.ndarray, indices: np.ndarray
) -> dict[str, float]:
    selected = np.asarray(indices, dtype=np.int64)
    if selected.ndim != 1 or len(selected) == 0:
        raise V12R7RecoveryFitError("metric slice is empty or malformed")
    value = v11.prediction_metrics(predictions[selected], targets[selected])
    return {
        "rmse": float(value["rmse"]),
        "max_abs_error": float(value["max_abs_error"]),
    }


def _candidate_actor_manifest(
    *, state: Mapping[str, Any], feature_names: Sequence[str], module_state: Path
) -> dict[str, Any]:
    return {
        "schema_version": 1,
        "status": "H0_V12R7_RECOVERY_ACTOR_FEATURE_CONTRACT",
        "topology_id": TOPOLOGY_ID,
        "fit_contract_id": contract.FIT_CONTRACT_ID,
        "actor_feature_count": contract.EXPECTED_ACTOR_FEATURES,
        "actor_feature_names": [str(name) for name in feature_names],
        "fcnet_hiddens": [512, 512],
        "disabled_clock_columns": list(contract.DISABLED_CLOCK_COLUMNS),
        "actor_digest": warm_start.actor_state_digest(state),
        "module_state_sha256": _sha256_file(module_state),
    }


def _save_candidate_exact(
    *,
    source_state: Mapping[str, Any],
    candidate_state: Mapping[str, Any],
    feature_names: Sequence[str],
    destination: Path,
    corpus_record: Mapping[str, Any],
) -> tuple[dict[str, Any], dict[str, Any], dict[str, Any]]:
    from ray.rllib.core.rl_module.rl_module import RLModule

    source_module = RLModule.from_checkpoint(
        _resolve(contract.R6_CANDIDATE_MODULE_PATH)
    )
    source_module.eval()
    source_module.set_state(candidate_state)
    reloaded = r6_builder.save_composite_checkpoint_no_clobber(
        source_module, destination
    )
    reloaded_state = v11._clone_state(reloaded.get_state())
    if not all(
        _bytes_equal(candidate_state[name], reloaded_state[name])
        for name in candidate_state
    ):
        raise V12R7RecoveryFitError("candidate core save/reload drifted")
    actor_manifest = _candidate_actor_manifest(
        state=reloaded_state,
        feature_names=feature_names,
        module_state=destination / "module_state.pkl",
    )
    if set(actor_manifest) != ACTOR_MANIFEST_FIELDS:
        raise V12R7RecoveryFitError("actor manifest schema drifted")
    forensic.write_json_exclusive(
        destination / ACTOR_FEATURE_MANIFEST_NAME, actor_manifest
    )
    preservation = full_mean_update_audit(source_state, reloaded_state)
    build_manifest = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": CANDIDATE_BUILD_STATUS,
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "fit_contract_id": contract.FIT_CONTRACT_ID,
        "topology_id": TOPOLOGY_ID,
        "source_candidate": copy.deepcopy(contract.LOCKED_INPUTS["r6_candidate"]),
        "architecture": copy.deepcopy(contract.FIT["architecture"]),
        "stratum_count": int(contract.FIT["stratum_count"]),
        "stratum_target_mass": float(contract.FIT["stratum_target_mass"]),
        "actor_fit_count": 1,
        "actor_updates": 1,
        "critic_updates": 0,
        "ppo_updates": 0,
        "logstd_byte_exact": preservation["logstd_parameter_rows_byte_exact"],
        "disabled_clock_columns_bit_zero": preservation[
            "disabled_clock_columns_bit_zero"
        ],
        "save_reload_exact": True,
        "actor_digest": actor_manifest["actor_digest"],
        "module_state_sha256": actor_manifest["module_state_sha256"],
        "actor_feature_manifest": ACTOR_FEATURE_MANIFEST_NAME,
        "corpus": copy.deepcopy(dict(corpus_record)),
    }
    if set(build_manifest) != BUILD_MANIFEST_FIELDS:
        raise V12R7RecoveryFitError("candidate build manifest schema drifted")
    forensic.write_json_exclusive(
        destination / CANDIDATE_BUILD_MANIFEST_NAME, build_manifest
    )
    observed = {item.name for item in destination.iterdir() if item.is_file()}
    if observed != EXPECTED_CANDIDATE_FILES:
        raise V12R7RecoveryFitError(f"candidate file set drifted: {sorted(observed)}")
    final_module = RLModule.from_checkpoint(destination)
    final_module.eval()
    final_state = v11._clone_state(final_module.get_state())
    if not all(
        _bytes_equal(candidate_state[name], final_state[name])
        for name in candidate_state
    ):
        raise V12R7RecoveryFitError("candidate final save/reload drifted")
    return _tree_record(destination), actor_manifest, build_manifest


def run_fit_stage(
    *,
    pipeline_claim_path: str | PurePath | Path,
    worker_claim_path: str | PurePath | Path,
    protocol_freeze_path: str | PurePath | Path,
    execution_lock_path: str | PurePath | Path,
    activity_callback: Callable[[str, int], None] | None = None,
) -> dict[str, Any]:
    """Execute and exclusively publish the sole R7 recovery actor fit."""

    destination = _resolve(contract.FIT_ROOT)
    if os.path.lexists(destination):
        raise V12R7RecoveryFitError("R7 fit destination exists/no-clobber")
    pipeline_claim = _record(pipeline_claim_path)
    worker_claim = _record(worker_claim_path)
    protocol_freeze = _record(protocol_freeze_path)
    execution_lock = _record(execution_lock_path)
    bundle = load_recovery_corpus()
    corpus = bundle.corpus
    source_tree_before = _tree_record(contract.R6_CANDIDATE_MODULE_PATH)
    _source_module, source_state, source_manifest = _load_source_module_and_state()
    if tuple(source_manifest["actor_feature_names"]) != tuple(
        corpus.actor_feature_names.astype(str)
    ):
        raise V12R7RecoveryFitError("source/corpus actor feature order drifted")
    base_v8 = np.flatnonzero(
        corpus.tranche_ids[: int(contract.LOCKED_INPUTS["base_corpus"]["rows"])].astype(
            str
        )
        == "v8r1p1_base"
    )
    if len(base_v8) != 3000:
        raise V12R7RecoveryFitError("frozen normalization base row count drifted")
    normalization = v11.frozen_base_normalization(corpus.observations[base_v8])
    result = fit_recovery_full_mean_in_memory(
        source_state=source_state,
        observations=corpus.observations,
        targets=corpus.actions,
        reset_mask=corpus.reset_mask,
        sample_weights=corpus.normalized_sample_weights,
        normalization=normalization,
        activity_callback=activity_callback,
    )
    if source_tree_before != _tree_record(contract.R6_CANDIDATE_MODULE_PATH):
        raise V12R7RecoveryFitError("R6 source candidate changed during fit")
    destination.mkdir(parents=True, exist_ok=False)
    corpus_path = destination / "corpus.npz"
    v10s_fit._write_npz_exclusive(corpus_path, corpus.arrays())
    corpus_record = _record(corpus_path)
    corpus_manifest = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": "H0_V12R7_RECOVERY_CORPUS_PASS",
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "fit_contract_id": contract.FIT_CONTRACT_ID,
        "rows": int(len(corpus.observations)),
        "actor_feature_count": contract.EXPECTED_ACTOR_FEATURES,
        "action_dim": contract.EXPECTED_ACTION_DIM,
        "component_order": list(corpus.audit["component_order"]),
        "stratum_audit": copy.deepcopy(corpus.audit["weight_audit"]),
        "source_records": copy.deepcopy(corpus.source_records),
        "corpus": corpus_record,
    }
    corpus_manifest_path = destination / CORPUS_MANIFEST_NAME
    forensic.write_json_exclusive(corpus_manifest_path, corpus_manifest)
    module_record, actor_manifest, build_manifest = _save_candidate_exact(
        source_state=source_state,
        candidate_state=result.candidate_state,
        feature_names=corpus.actor_feature_names.astype(str).tolist(),
        destination=_resolve(contract.CANDIDATE_MODULE_PATH),
        corpus_record=corpus_record,
    )
    predictions = result.predictions
    base_case_metrics = {
        case_id: _metric_pair(predictions, corpus.actions, bundle.base_indices[case_id])
        for case_id in contract.COLLECTION_CASE_IDS
    }
    observer_metrics = {
        case_id: _metric_pair(
            predictions, corpus.actions, bundle.observer_indices[case_id]
        )
        for case_id in contract.COLLECTION_CASE_IDS
    }
    global_metrics = _metric_pair(
        predictions, corpus.actions, np.arange(len(predictions), dtype=np.int64)
    )
    reset_error = np.abs(
        predictions[corpus.reset_mask].astype(np.float64)
        - corpus.actions[corpus.reset_mask].astype(np.float64)
    )
    reset_max_abs_error = float(np.max(reset_error))
    r4_metrics = _metric_pair(predictions, corpus.actions, bundle.r4_indices)
    observer_plus_late = _metric_pair(
        predictions, corpus.actions, bundle.observer_plus_late_indices
    )
    absolute_error = np.abs(
        predictions.astype(np.float64) - corpus.actions.astype(np.float64)
    )
    flat = int(np.argmax(absolute_error))
    worst_row, worst_action = np.unravel_index(flat, absolute_error.shape)
    history_path = destination / ADAPTATION_HISTORY_NAME
    report_path = destination / ADAPTATION_REPORT_NAME
    summary_path = _resolve(contract.FIT_SUMMARY_PATH)
    gate_path = _resolve(contract.FIT_GATE_PATH)
    receipt_path = _resolve(contract.FIT_RECEIPT_PATH)
    forensic.write_json_exclusive(history_path, list(result.history))
    report = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": "COMPLETE_H0_V12R7_RECOVERY_ADAPTATION_REPORT",
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "fit_contract_id": contract.FIT_CONTRACT_ID,
        "training_samples": int(len(corpus.observations)),
        "stratum_audit": copy.deepcopy(corpus.audit["weight_audit"]),
        "global_metrics": global_metrics,
        "reset_max_abs_error": reset_max_abs_error,
        "per_case_metrics": base_case_metrics,
        "r4_failed_plus_metrics": r4_metrics,
        "observer_case_metrics": observer_metrics,
        "observer_plus_late_metrics": observer_plus_late,
        "normalization_audit": dict(result.normalization_audit),
        "preservation_audit": dict(result.preservation_audit),
        "optimizer_audit": dict(result.optimizer_audit),
        "candidate_module": module_record,
        "actor_feature_manifest": actor_manifest,
        "candidate_build_manifest": build_manifest,
        "source_records": copy.deepcopy(corpus.source_records),
    }
    forensic.write_json_exclusive(report_path, report)
    summary = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": FIT_COMPLETE_STATUS,
        "protocol_id": contract.PROTOCOL_ID,
        "fit_contract_id": contract.FIT_CONTRACT_ID,
        "candidate_selection_rule": contract.CANDIDATE_SELECTION_RULE,
        "hidden_dims": [512, 512],
        "actor_feature_count": contract.EXPECTED_ACTOR_FEATURES,
        "sample_count": int(len(corpus.observations)),
        "reset_row_count": int(np.count_nonzero(corpus.reset_mask)),
        "stratum_count": int(contract.FIT["stratum_count"]),
        "stratum_target_mass": float(contract.FIT["stratum_target_mass"]),
        "stratum_mass": copy.deepcopy(corpus.audit["weight_audit"]["stratum_mass"]),
        "global_metrics": global_metrics,
        "reset_max_abs_error": reset_max_abs_error,
        "per_case_metrics": base_case_metrics,
        "r4_failed_plus_metrics": r4_metrics,
        "observer_case_metrics": observer_metrics,
        "observer_plus_late_metrics": observer_plus_late,
        "actor_fit_count": 1,
        "actor_updates": 1,
        "critic_updates": 0,
        "ppo_updates": 0,
        "adamw_epochs_run": 2000,
        "lbfgs_max_iter": 300,
        "lbfgs_max_eval": 600,
        "lbfgs_closure_calls": int(result.optimizer_audit["lbfgs_closure_calls"]),
        "logstd_byte_exact": result.preservation_audit.get(
            "logstd_parameter_rows_byte_exact"
        )
        is True,
        "disabled_clock_columns_bit_zero": result.preservation_audit.get(
            "disabled_clock_columns_bit_zero"
        )
        is True,
        "save_reload_exact": True,
        "critic_present": False,
        "fallback_used": False,
        "sweep_used": False,
        "candidate_id": contract.candidate_id(module_record["tree_sha256"]),
        "candidate_module": module_record,
        "worst_row": {
            "absolute_error": float(absolute_error[worst_row, worst_action]),
            "action_dimension": int(worst_action),
            "case_id": str(corpus.case_ids[worst_row]),
            "step_index": int(corpus.step_indices[worst_row]),
            "tranche_id": str(corpus.tranche_ids[worst_row]),
        },
        "corpus": corpus_record,
        "corpus_manifest": _record(corpus_manifest_path),
        "adaptation_report": _record(report_path),
        "adaptation_history": _record(history_path),
        "actor_feature_manifest": _record(
            _resolve(contract.CANDIDATE_MODULE_PATH) / ACTOR_FEATURE_MANIFEST_NAME
        ),
        "candidate_build_manifest": _record(
            _resolve(contract.CANDIDATE_MODULE_PATH) / CANDIDATE_BUILD_MANIFEST_NAME
        ),
        "pipeline_claim": pipeline_claim,
        "worker_claim": worker_claim,
        "protocol_freeze": protocol_freeze,
        "execution_lock": execution_lock,
    }
    forensic.write_json_exclusive(summary_path, summary)
    gate = contract.fit_gate(summary)
    forensic.write_json_exclusive(gate_path, gate)
    if gate.get("passed") is not True:
        raise V12R7RecoveryFitError("R7 offline fit gate failed")
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
        "corpus": corpus_record,
        "corpus_manifest": _record(corpus_manifest_path),
        "adaptation_report": _record(report_path),
        "adaptation_history": _record(history_path),
        "actor_feature_manifest": summary["actor_feature_manifest"],
        "candidate_build_manifest": summary["candidate_build_manifest"],
        "pipeline_claim": pipeline_claim,
        "worker_claim": worker_claim,
        "protocol_freeze": protocol_freeze,
        "execution_lock": execution_lock,
        "actor_updates": 1,
        "critic_updates": 0,
        "ppo_updates": 0,
    }
    if set(receipt) != FIT_RECEIPT_FIELDS:
        raise V12R7RecoveryFitError("fit receipt schema drifted")
    forensic.write_json_exclusive(receipt_path, receipt)
    return receipt


def verify_fit_stage() -> dict[str, Any]:
    """Recompute all publication bindings without assuming Q3 is unopened."""

    root = _resolve(contract.FIT_ROOT)
    summary = _mapping(contract.FIT_SUMMARY_PATH)
    gate = _mapping(contract.FIT_GATE_PATH)
    receipt = _mapping(contract.FIT_RECEIPT_PATH)
    module = _tree_record(contract.CANDIDATE_MODULE_PATH)
    if {row["path"] for row in module["files"]} != EXPECTED_CANDIDATE_FILES or module[
        "file_count"
    ] != 5:
        raise V12R7RecoveryFitError("candidate five-file tree drifted")
    expected_id = contract.candidate_id(module["tree_sha256"])
    actor_manifest = _mapping(
        _resolve(contract.CANDIDATE_MODULE_PATH) / ACTOR_FEATURE_MANIFEST_NAME
    )
    build_manifest = _mapping(
        _resolve(contract.CANDIDATE_MODULE_PATH) / CANDIDATE_BUILD_MANIFEST_NAME
    )
    if (
        set(actor_manifest) != ACTOR_MANIFEST_FIELDS
        or set(build_manifest) != BUILD_MANIFEST_FIELDS
    ):
        raise V12R7RecoveryFitError("candidate manifest schema drifted")
    _source_module, source_state, source_manifest = _load_source_module_and_state()
    from ray.rllib.core.rl_module.rl_module import RLModule

    candidate = RLModule.from_checkpoint(_resolve(contract.CANDIDATE_MODULE_PATH))
    candidate.eval()
    candidate_state = v11._clone_state(candidate.get_state())
    state_audit = validate_source_r6_state(candidate_state)
    preservation = full_mean_update_audit(source_state, candidate_state)
    expected_gate = contract.fit_gate(summary)
    expected_records = {
        "summary": contract.FIT_SUMMARY_PATH,
        "gate": contract.FIT_GATE_PATH,
        "corpus": contract.CORPUS_PATH,
        "corpus_manifest": root / CORPUS_MANIFEST_NAME,
        "adaptation_report": root / ADAPTATION_REPORT_NAME,
        "adaptation_history": root / ADAPTATION_HISTORY_NAME,
        "actor_feature_manifest": _resolve(contract.CANDIDATE_MODULE_PATH)
        / ACTOR_FEATURE_MANIFEST_NAME,
        "candidate_build_manifest": _resolve(contract.CANDIDATE_MODULE_PATH)
        / CANDIDATE_BUILD_MANIFEST_NAME,
        "pipeline_claim": contract.CLAIM_PATH,
        "protocol_freeze": contract.PROTOCOL_FREEZE_PATH,
        "execution_lock": contract.EXECUTION_LOCK_PATH,
    }
    records_valid = all(
        _strict_equal(receipt.get(name), _record(path))
        for name, path in expected_records.items()
    )
    worker_record = receipt.get("worker_claim")
    worker_valid = isinstance(worker_record, Mapping) and _strict_equal(
        dict(worker_record), _record(worker_record.get("path", ""))
    )
    checks = {
        "receipt_schema": set(receipt) == FIT_RECEIPT_FIELDS,
        "receipt_pass": receipt.get("status") == FIT_PASS_STATUS
        and receipt.get("passed") is True,
        "protocol_exact": receipt.get("protocol_id") == contract.PROTOCOL_ID
        and receipt.get("fit_contract_id") == contract.FIT_CONTRACT_ID,
        "candidate_id": receipt.get("candidate_id") == expected_id
        and summary.get("candidate_id") == expected_id,
        "candidate_tree": _strict_equal(receipt.get("candidate_module"), module)
        and _strict_equal(summary.get("candidate_module"), module),
        "contract_gate": _strict_equal(gate, expected_gate)
        and gate.get("passed") is True,
        "artifact_records": records_valid and worker_valid,
        "actor_manifest": actor_manifest.get("schema_version") == 1
        and actor_manifest.get("status") == "H0_V12R7_RECOVERY_ACTOR_FEATURE_CONTRACT"
        and actor_manifest.get("topology_id") == TOPOLOGY_ID
        and actor_manifest.get("fit_contract_id") == contract.FIT_CONTRACT_ID
        and actor_manifest.get("actor_feature_count")
        == contract.EXPECTED_ACTOR_FEATURES
        and actor_manifest.get("actor_feature_names")
        == source_manifest.get("actor_feature_names")
        and actor_manifest.get("fcnet_hiddens") == [512, 512]
        and actor_manifest.get("disabled_clock_columns")
        == list(contract.DISABLED_CLOCK_COLUMNS)
        and actor_manifest.get("actor_digest") == state_audit["actor_digest"]
        and actor_manifest.get("module_state_sha256")
        == _sha256_file(_resolve(contract.CANDIDATE_MODULE_PATH) / "module_state.pkl"),
        "build_manifest": build_manifest.get("passed") is True
        and build_manifest.get("actor_digest") == state_audit["actor_digest"],
        "mean_only": preservation["passed"] is True,
        "counters": type(receipt.get("actor_updates")) is int
        and receipt["actor_updates"] == 1
        and type(receipt.get("critic_updates")) is int
        and receipt["critic_updates"] == 0
        and type(receipt.get("ppo_updates")) is int
        and receipt["ppo_updates"] == 0,
    }
    if not all(checks.values()):
        failed = sorted(name for name, passed in checks.items() if not passed)
        raise V12R7RecoveryFitError(f"R7 fit verification failed: {failed}")
    return receipt


__all__ = [
    "V12R7RecoveryFitError",
    "RecoveryCorpusBundle",
    "adamw_learning_rate",
    "attest_locked_inputs",
    "compute_equal_stratum_weights",
    "expected_stratum_ids",
    "fit_recovery_full_mean_in_memory",
    "full_mean_update_audit",
    "load_recovery_corpus",
    "observer_labels_path",
    "run_fit_stage",
    "validate_source_r6_state",
    "verify_fit_stage",
]
