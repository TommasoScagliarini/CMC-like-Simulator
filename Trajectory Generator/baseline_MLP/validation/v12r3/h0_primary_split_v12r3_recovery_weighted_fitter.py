"""Deterministic recovery-weighted full-mean fitter for V12R3.

This module is deliberately additive.  It does not modify the frozen V12
sources and it never opens an environment.  Each fit starts from the immutable
H0 actor, consumes the byte-bound V11 P3 seed, then appends each earlier pure
observer-label episode followed by the corresponding two shielded collection
episodes.  Every episode is normalized to an exact mass of 500 before the
fixed AdamW+LBFGS objective is evaluated.

The public coverage helpers are also used by the observer-only labeler.  They
recompute the frozen V11 leave-one-out reference and certify each new nearest
neighbour with the preregistered top-64 float64/fsum procedure.
"""

from __future__ import annotations

import copy
import math
import os
import sys
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
ROOT_VALIDATION = REPO_ROOT / "validation"
TRAJECTORY_ROOT = REPO_ROOT / "Trajectory Generator"
BASELINE_ROOT = TRAJECTORY_ROOT / "baseline_MLP"
LOCAL_VALIDATION = BASELINE_ROOT / "validation"
REVISION_ROOT = Path(__file__).resolve().parent
for _root in (
    REPO_ROOT,
    ROOT_VALIDATION,
    TRAJECTORY_ROOT,
    BASELINE_ROOT,
    LOCAL_VALIDATION,
    REVISION_ROOT,
):
    if str(_root) not in sys.path:
        sys.path.insert(0, str(_root))

import h0_forensic_rollout as forensic  # noqa: E402
import h0_primary_split_v11_weighted_fit as v11  # noqa: E402
import h0_primary_split_v12r3_autonomy_recovery_contract as contract  # noqa: E402


FIT_CONTRACT_ID = contract.FIT_CONTRACT_ID
CLOCK_COLUMNS = (0, 1)
BASE_NORMALIZATION_ROWS = 3000
EPISODE_TARGET_MASS = 500.0
DETERMINISTIC_TORCH_THREADS = v11.DETERMINISTIC_TORCH_THREADS

_SEED_KEYS = {
    "observations",
    "actions",
    "reset_mask",
    "actor_feature_names",
    "case_ids",
    "step_indices",
    "tranche_ids",
    "origins",
    "training_indices",
}
_LABEL_KEYS = {
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
_SHIELDED_CORE_KEYS = {
    "observations",
    "actions",
    "reset_mask",
    "actor_feature_names",
    "case_ids",
    "step_indices",
    "tranche_ids",
    "origins",
}


class V12R3RecoveryWeightedFitError(RuntimeError):
    """Raised when V12R3 fitting evidence is incomplete or inconsistent."""


# Compatibility spelling used by inherited orchestration code.
V10SFitError = V12R3RecoveryWeightedFitError
FrozenNormalization = v11.FrozenNormalization
InMemoryFitResult = v11.InMemoryFitResult


@dataclass(frozen=True)
class CoverageReference:
    observations: np.ndarray
    normalized_features: np.ndarray
    normalization: FrozenNormalization
    nearest_indices: np.ndarray
    loo_distances: np.ndarray
    loo_p95: float
    audit: Mapping[str, Any]


@dataclass(frozen=True)
class RecoveryFitCorpus:
    observations: np.ndarray
    actions: np.ndarray
    reset_mask: np.ndarray
    actor_feature_names: np.ndarray
    case_ids: np.ndarray
    step_indices: np.ndarray
    tranche_ids: np.ndarray
    origins: np.ndarray
    episode_ids: np.ndarray
    raw_sample_weights: np.ndarray
    normalized_sample_weights: np.ndarray
    source_records: Mapping[str, Any]
    probe_label_bindings: tuple[Mapping[str, Any], ...]
    collection_bindings: tuple[Mapping[str, Any], ...]
    audit: Mapping[str, Any]

    def arrays(self) -> dict[str, np.ndarray]:
        rows = len(self.observations)
        return {
            "observations": self.observations,
            "actions": self.actions,
            "reset_mask": self.reset_mask,
            "actor_feature_names": self.actor_feature_names,
            "case_ids": self.case_ids,
            "step_indices": self.step_indices,
            "tranche_ids": self.tranche_ids,
            "origins": self.origins,
            "episode_ids": self.episode_ids,
            "raw_sample_weights": self.raw_sample_weights,
            "normalized_sample_weights": self.normalized_sample_weights,
            "training_indices": np.arange(rows, dtype=np.int64),
        }


def _resolve(path: str | PurePath | Path) -> Path:
    raw = Path(path)
    return (raw if raw.is_absolute() else REPO_ROOT / raw).resolve()


def _mapping(path: str | PurePath | Path) -> dict[str, Any]:
    resolved = _resolve(path)
    try:
        value = forensic.strict_json_load(resolved)
    except Exception as exc:
        raise V12R3RecoveryWeightedFitError(
            f"invalid strict JSON object: {resolved}"
        ) from exc
    if not isinstance(value, Mapping):
        raise V12R3RecoveryWeightedFitError(f"expected strict JSON object: {resolved}")
    return dict(value)


def _record(path: str | PurePath | Path) -> dict[str, Any]:
    try:
        return forensic.artifact_record(_resolve(path), artifact_root=REPO_ROOT)
    except Exception as exc:
        raise V12R3RecoveryWeightedFitError(f"cannot record artifact: {path}") from exc


def _tree_record(path: str | PurePath | Path) -> dict[str, Any]:
    try:
        return v11._tree_record(_resolve(path))
    except Exception as exc:
        raise V12R3RecoveryWeightedFitError(
            f"cannot record artifact tree: {path}"
        ) from exc


def array_sha256(value: Any) -> str:
    return v11.array_sha256(value)


def _bytes_equal(left: Any, right: Any) -> bool:
    a = np.ascontiguousarray(np.asarray(left))
    b = np.ascontiguousarray(np.asarray(right))
    return a.dtype == b.dtype and a.shape == b.shape and a.tobytes() == b.tobytes()


def _read_npz_exact(
    path: str | PurePath | Path, expected_keys: set[str]
) -> dict[str, np.ndarray]:
    resolved = _resolve(path)
    try:
        with np.load(resolved, allow_pickle=False) as archive:
            if set(archive.files) != expected_keys:
                raise V12R3RecoveryWeightedFitError(
                    f"NPZ keys drifted at {resolved}: {sorted(archive.files)}"
                )
            return {name: archive[name] for name in archive.files}
    except V12R3RecoveryWeightedFitError:
        raise
    except Exception as exc:
        raise V12R3RecoveryWeightedFitError(
            f"cannot read safe NPZ: {resolved}"
        ) from exc


def _finite_array(value: Any) -> bool:
    try:
        return bool(np.all(np.isfinite(np.asarray(value))))
    except (TypeError, ValueError):
        return False


def recovery_ramp_weight(previous_penetration_m: float) -> float:
    """Return the pure-row recovery modifier without reset/coverage modifiers."""

    try:
        value = float(previous_penetration_m)
    except (TypeError, ValueError, OverflowError) as exc:
        raise V12R3RecoveryWeightedFitError("penetration is not numeric") from exc
    if not math.isfinite(value) or value < 0.0:
        raise V12R3RecoveryWeightedFitError(
            "penetration must be finite and nonnegative"
        )
    lower = float(contract.RECOVERY_WEIGHTING["nominal_upper_m"])
    upper = float(contract.RECOVERY_WEIGHTING["latch_activation_m"])
    if value <= lower:
        return 1.0
    if value >= upper:
        return 100.0
    return 1.0 + 99.0 * ((value - lower) / (upper - lower))


def _normalize_episode_exact(raw_weights: Sequence[float]) -> np.ndarray:
    """Normalize one episode and require ``math.fsum(result) == 500.0``."""

    raw = np.ascontiguousarray(raw_weights, dtype=np.float64)
    if raw.ndim != 1 or len(raw) < 1 or not _finite_array(raw) or np.any(raw <= 0.0):
        raise V12R3RecoveryWeightedFitError(
            "episode raw weights must be finite, positive, and nonempty"
        )
    total = math.fsum(float(value) for value in raw)
    scaled = np.asarray(
        [float(value) * EPISODE_TARGET_MASS / total for value in raw],
        dtype=np.float64,
    )
    # Computing the final term as a remainder gives an exact fsum on all
    # supported runtimes.  The guarded nextafter loop is a fail-closed remedy
    # for a one-ulp platform rounding difference, not a fit retry or sweep.
    scaled[-1] = EPISODE_TARGET_MASS - math.fsum(float(value) for value in scaled[:-1])
    for _ in range(4):
        observed = math.fsum(float(value) for value in scaled)
        if observed == EPISODE_TARGET_MASS:
            break
        direction = math.inf if observed < EPISODE_TARGET_MASS else -math.inf
        scaled[-1] = np.nextafter(scaled[-1], direction)
    if (
        math.fsum(float(value) for value in scaled) != EPISODE_TARGET_MASS
        or np.any(scaled <= 0.0)
        or not _finite_array(scaled)
    ):
        raise V12R3RecoveryWeightedFitError(
            "episode weights cannot be normalized to exact mass 500"
        )
    return np.ascontiguousarray(scaled, dtype=np.float64)


def normalized_episode_sample_weights(raw_weights: Sequence[float]) -> np.ndarray:
    """Public exact episode-normalization primitive."""

    return _normalize_episode_exact(raw_weights)


def frozen_base_normalization(base_observations: Any) -> FrozenNormalization:
    try:
        return v11.frozen_base_normalization(base_observations)
    except Exception as exc:
        raise V12R3RecoveryWeightedFitError("base normalization failed") from exc


def normalized_observations(
    observations: Any, normalization: FrozenNormalization
) -> np.ndarray:
    try:
        return v11.normalized_observations(observations, normalization)
    except Exception as exc:
        raise V12R3RecoveryWeightedFitError("observation normalization failed") from exc


def _load_seed_arrays() -> tuple[dict[str, np.ndarray], dict[str, Any]]:
    path = _resolve(contract.V11_P3_CORPUS_PATH)
    record = _record(path)
    expected_record = dict(contract.V11_P3_CORPUS_ARTIFACT)
    if record != expected_record:
        raise V12R3RecoveryWeightedFitError(
            f"V11 P3 seed artifact drifted: {record} != {expected_record}"
        )
    arrays = _read_npz_exact(path, _SEED_KEYS)
    observations = arrays["observations"]
    actions = arrays["actions"]
    reset = arrays["reset_mask"]
    if (
        observations.shape != (6000, 35)
        or observations.dtype != np.dtype("float32")
        or actions.shape != (6000, 2)
        or actions.dtype != np.dtype("float32")
        or reset.shape != (6000,)
        or reset.dtype != np.dtype("bool")
        or int(np.count_nonzero(reset)) != 12
        or not _finite_array(observations)
        or not _finite_array(actions)
        or np.any(np.abs(actions) > 1.0)
        or not np.array_equal(arrays["training_indices"], np.arange(6000))
        or array_sha256(observations)
        != contract.COVERAGE_WEIGHTING["reference_observations_sha256"]
    ):
        raise V12R3RecoveryWeightedFitError("V11 P3 seed arrays drifted")
    for offset in range(0, 6000, 500):
        slc = slice(offset, offset + 500)
        if (
            not bool(reset[offset])
            or int(np.count_nonzero(reset[slc])) != 1
            or not np.array_equal(
                arrays["step_indices"][slc], np.arange(1, 501, dtype=np.int64)
            )
            or len(set(arrays["case_ids"][slc].astype(str).tolist())) != 1
            or len(set(arrays["tranche_ids"][slc].astype(str).tolist())) != 1
        ):
            raise V12R3RecoveryWeightedFitError(
                f"V11 seed episode layout drifted at row {offset}"
            )
    return arrays, record


def load_coverage_reference() -> CoverageReference:
    """Recompute and validate the frozen V11 coverage reference exactly."""

    try:
        from scipy.spatial import cKDTree
    except Exception as exc:
        raise V12R3RecoveryWeightedFitError("scipy cKDTree is unavailable") from exc

    arrays, artifact = _load_seed_arrays()
    observations = np.ascontiguousarray(arrays["observations"], dtype=np.float32)
    base = np.ascontiguousarray(
        observations[:BASE_NORMALIZATION_ROWS], dtype=np.float32
    )
    normalization = frozen_base_normalization(base)
    normalized = normalized_observations(observations, normalization)
    included = np.asarray(
        contract.COVERAGE_WEIGHTING["included_feature_indices"], dtype=np.int64
    )
    features = np.ascontiguousarray(normalized[:, included], dtype=np.float32)
    expected = contract.COVERAGE_WEIGHTING
    static_hashes = {
        "observations": array_sha256(observations),
        "base_observations": array_sha256(base),
        "normalization_mean": array_sha256(normalization.mean),
        "normalization_std": array_sha256(normalization.std),
        "normalized_observations": array_sha256(normalized),
        "normalized_features": array_sha256(features),
    }
    if static_hashes != {
        "observations": expected["reference_observations_sha256"],
        "base_observations": expected["base_observations_sha256"],
        "normalization_mean": expected["normalization_mean_sha256"],
        "normalization_std": expected["normalization_std_sha256"],
        "normalized_observations": expected["normalized_observations_sha256"],
        "normalized_features": expected["normalized_feature_matrix_sha256"],
    }:
        raise V12R3RecoveryWeightedFitError(
            f"coverage reference static hash parity failed: {static_hashes}"
        )

    tree_spec = expected["loo_tree"]
    tree = cKDTree(
        features,
        leafsize=tree_spec["leafsize"],
        compact_nodes=tree_spec["compact_nodes"],
        balanced_tree=tree_spec["balanced_tree"],
    )
    _, query_indices = tree.query(
        features,
        k=tree_spec["query_k"],
        eps=tree_spec["eps"],
        p=tree_spec["p"],
        workers=tree_spec["workers"],
    )
    tie_spec = expected["tie_audit"]
    _, extended_indices = tree.query(
        features,
        k=tie_spec["extended_query_k"],
        eps=tree_spec["eps"],
        p=tree_spec["p"],
        workers=tree_spec["workers"],
    )

    def select(candidates_matrix: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        selected = np.empty(len(features), dtype=np.int64)
        tie_counts = np.empty(len(features), dtype=np.int64)
        for row_index, candidate_row in enumerate(candidates_matrix):
            alternatives = [
                int(value) for value in candidate_row if int(value) != row_index
            ]
            if not alternatives:
                raise V12R3RecoveryWeightedFitError(
                    "coverage LOO query did not find a neighbour"
                )
            candidate_features = features[np.asarray(alternatives, dtype=np.int64)]
            delta = candidate_features.astype(np.float64) - features[row_index].astype(
                np.float64
            )
            distances = np.sqrt(
                np.sum(np.square(delta), axis=1, dtype=np.float64) / features.shape[1]
            )
            minimum = float(np.min(distances))
            ties = [
                candidate
                for candidate, distance in zip(alternatives, distances, strict=True)
                if float(distance) == minimum
            ]
            selected[row_index] = min(ties)
            tie_counts[row_index] = len(ties)
        return selected, tie_counts

    nearest, query_ties = select(np.asarray(query_indices))
    extended_nearest, extended_ties = select(np.asarray(extended_indices))
    delta = features.astype(np.float64) - features[nearest].astype(np.float64)
    loo_distances = np.sqrt(
        np.sum(np.square(delta), axis=1, dtype=np.float64) / features.shape[1]
    )
    p95 = float(np.percentile(loo_distances, 95, method="linear"))
    dynamic_hashes = {
        "nearest_indices": array_sha256(nearest),
        "loo_distances": array_sha256(loo_distances),
    }
    unique_count = int(np.unique(features, axis=0).shape[0])
    maximum_ties = int(np.max(extended_ties))
    checks = {
        "nearest_hash": dynamic_hashes["nearest_indices"]
        == expected["loo_nearest_indices_sha256"],
        "distance_hash": dynamic_hashes["loo_distances"]
        == expected["loo_distances_sha256"],
        "p95": abs(p95 - float(expected["loo_p95"])) <= 1.0e-12,
        "unique_count": unique_count == tie_spec["unique_observation_count"],
        "maximum_ties": maximum_ties == tie_spec["maximum_minimum_distance_tie_count"],
        "query_matches_extended": bool(np.array_equal(nearest, extended_nearest))
        is tie_spec["query_k_matches_extended_query"],
        "tie_counts_match": bool(np.array_equal(query_ties, extended_ties)),
    }
    if not all(checks.values()):
        raise V12R3RecoveryWeightedFitError(
            f"coverage reference parity failed: {checks}"
        )
    audit = {
        "passed": True,
        "checks": checks,
        "artifact": artifact,
        "hashes": {**static_hashes, **dynamic_hashes},
        "loo_p95": p95,
        "reference_loo_p95": float(expected["loo_p95"]),
        "included_feature_indices": included.tolist(),
        "feature_count": int(features.shape[1]),
        "new_row_query": copy.deepcopy(expected["new_row_query"]),
    }
    return CoverageReference(
        observations=observations,
        normalized_features=features,
        normalization=normalization,
        nearest_indices=nearest,
        loo_distances=loo_distances,
        loo_p95=p95,
        audit=audit,
    )


def evaluate_observer_coverage(observations: Any) -> dict[str, Any]:
    """Evaluate new pure rows with an exact global-nearest certificate."""

    raw = np.ascontiguousarray(observations, dtype=np.float32)
    if raw.ndim != 2 or raw.shape[1] != 35 or not _finite_array(raw):
        raise V12R3RecoveryWeightedFitError(
            "coverage observations must be finite float32 Nx35"
        )
    reference = load_coverage_reference()
    query_features = np.ascontiguousarray(
        normalized_observations(raw, reference.normalization)[:, 2:35],
        dtype=np.float32,
    )
    reference64 = reference.normalized_features.astype(np.float64)
    nearest = np.empty(len(raw), dtype=np.int64)
    distances = np.empty(len(raw), dtype=np.float64)
    margins = np.empty(len(raw), dtype=np.float64)
    thresholds = np.empty(len(raw), dtype=np.float64)
    audit_k = int(
        contract.COVERAGE_WEIGHTING["new_row_query"]["global_minimum_candidate_audit_k"]
    )
    if audit_k != 64 or audit_k >= len(reference64):
        raise V12R3RecoveryWeightedFitError("coverage candidate audit k drifted")
    for row_index, query in enumerate(query_features):
        delta = reference64 - query.astype(np.float64)
        squared = np.square(delta)
        approximate_sums = np.sum(squared, axis=1, dtype=np.float64)
        partition = np.argpartition(approximate_sums, audit_k)
        candidates = partition[:audit_k]
        exact_sums = np.asarray(
            [math.fsum(squared[index].tolist()) for index in candidates],
            dtype=np.float64,
        )
        minimum = float(np.min(exact_sums))
        tied = candidates[exact_sums == minimum]
        nearest[row_index] = int(np.min(tied))
        distances[row_index] = math.sqrt(
            minimum / reference.normalized_features.shape[1]
        )
        excluded = float(np.min(approximate_sums[partition[audit_k:]]))
        margins[row_index] = excluded - minimum
        thresholds[row_index] = 1.0e-10 * max(1.0, excluded)
    if not bool(np.all(margins > thresholds)):
        raise V12R3RecoveryWeightedFitError(
            "new-row global-nearest certificate margin failed"
        )
    ood = np.ascontiguousarray(distances > reference.loo_p95, dtype=np.bool_)
    return {
        "distance_rms_z": np.ascontiguousarray(distances, dtype=np.float64),
        "nearest_reference_index": np.ascontiguousarray(nearest, dtype=np.int64),
        "ood_mask": ood,
        "normalized_features_sha256": array_sha256(query_features),
        "nearest_indices_sha256": array_sha256(nearest),
        "distances_sha256": array_sha256(distances),
        "reference_observations_sha256": contract.COVERAGE_WEIGHTING[
            "reference_observations_sha256"
        ],
        "normalization_mean_sha256": contract.COVERAGE_WEIGHTING[
            "normalization_mean_sha256"
        ],
        "normalization_std_sha256": contract.COVERAGE_WEIGHTING[
            "normalization_std_sha256"
        ],
        "reference_features_sha256": contract.COVERAGE_WEIGHTING[
            "normalized_feature_matrix_sha256"
        ],
        "loo_p95": reference.loo_p95,
        "new_row_query": copy.deepcopy(contract.COVERAGE_WEIGHTING["new_row_query"]),
        "minimum_certificate_margin_sum": float(np.min(margins, initial=math.inf)),
        "maximum_certificate_threshold_sum": float(np.max(thresholds, initial=0.0)),
        "reference_audit": copy.deepcopy(dict(reference.audit)),
        "audit": {
            "passed": True,
            "reference": copy.deepcopy(dict(reference.audit)),
            "normalized_features_sha256": array_sha256(query_features),
            "nearest_indices_sha256": array_sha256(nearest),
            "distances_sha256": array_sha256(distances),
            "ood_row_count": int(np.count_nonzero(ood)),
            "minimum_certificate_margin_sum": float(np.min(margins, initial=math.inf)),
            "new_row_query": copy.deepcopy(
                contract.COVERAGE_WEIGHTING["new_row_query"]
            ),
        },
    }


def coverage_reference_audit() -> dict[str, Any]:
    """Return the exact, JSON-safe reference-parity audit used by the lock."""

    return copy.deepcopy(dict(load_coverage_reference().audit))


def observer_episode_weights(
    previous_penetration_m: Any,
    reset_mask: Any,
    ood_mask: Any,
) -> tuple[np.ndarray, np.ndarray]:
    """Return independently recomputed raw and exact-normalized pure weights."""

    penetration = np.ascontiguousarray(previous_penetration_m, dtype=np.float64)
    reset = np.ascontiguousarray(reset_mask, dtype=np.bool_)
    ood = np.ascontiguousarray(ood_mask, dtype=np.bool_)
    if (
        penetration.ndim != 1
        or reset.shape != penetration.shape
        or ood.shape != penetration.shape
        or len(penetration) < 1
        or not _finite_array(penetration)
        or np.any(penetration < 0.0)
        or int(np.count_nonzero(reset)) != 1
        or not bool(reset[0])
    ):
        raise V12R3RecoveryWeightedFitError("pure episode weight metadata drifted")
    raw = np.asarray(
        [
            max(
                100.0 if bool(reset[index]) else 1.0,
                recovery_ramp_weight(float(penetration[index])),
                100.0 if bool(ood[index]) else 1.0,
            )
            for index in range(len(penetration))
        ],
        dtype=np.float64,
    )
    normalized = _normalize_episode_exact(raw)
    return np.ascontiguousarray(raw), normalized


def _shielded_episode_weights(reset_mask: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    reset = np.ascontiguousarray(reset_mask, dtype=np.bool_)
    if (
        reset.ndim != 1
        or len(reset) < 1
        or int(np.count_nonzero(reset)) != 1
        or not reset[0]
    ):
        raise V12R3RecoveryWeightedFitError("shielded episode reset layout drifted")
    raw = np.where(reset, 100.0, 1.0).astype(np.float64)
    return raw, _normalize_episode_exact(raw)


def _episode_id(family: str, stage_or_round: str, case_id: str) -> str:
    if not family or not stage_or_round or not case_id:
        raise V12R3RecoveryWeightedFitError("empty episode identity component")
    return f"{family}:{stage_or_round}:{case_id}"


def _seed_piece() -> dict[str, Any]:
    arrays, record = _load_seed_arrays()
    episode_ids = np.empty(6000, dtype="U192")
    raw_weights = np.empty(6000, dtype=np.float64)
    normalized_weights = np.empty(6000, dtype=np.float64)
    for offset in range(0, 6000, 500):
        slc = slice(offset, offset + 500)
        case_id = str(arrays["case_ids"][offset])
        tranche = str(arrays["tranche_ids"][offset])
        identity = _episode_id("v11_seed", tranche, case_id)
        raw, normalized = _shielded_episode_weights(arrays["reset_mask"][slc])
        episode_ids[slc] = identity
        raw_weights[slc] = raw
        normalized_weights[slc] = normalized
    return {
        **{name: arrays[name] for name in _SHIELDED_CORE_KEYS},
        "episode_ids": episode_ids,
        "raw_sample_weights": raw_weights,
        "normalized_sample_weights": normalized_weights,
        "source_record": record,
        "family": "v11_seed",
    }


def _artifact_matches(record: Any, path: str | PurePath | Path) -> bool:
    return isinstance(record, Mapping) and dict(record) == _record(path)


def _load_observer_piece(
    stage: str, pipeline_claim: Mapping[str, Any]
) -> tuple[dict[str, Any], dict[str, Any]]:
    receipt_path = _resolve(contract.LABEL_RECEIPT_PATHS[stage])
    receipt = _mapping(receipt_path)
    corpus_path = _resolve(contract.LABEL_CORPUS_PATHS[stage])
    corpus_record = _record(corpus_path)
    expected_receipt_keys = {
        "schema_version",
        "status",
        "passed",
        "protocol_id",
        "fit_stage",
        "labelled_row_count",
        "label_corpus",
        "pipeline_claim",
        "worker_claim",
    }
    if (
        set(receipt) != expected_receipt_keys
        or receipt.get("schema_version") != contract.SCHEMA_VERSION
        or receipt.get("status") != contract.OBSERVER_LABEL_PASS_STATUS
        or receipt.get("passed") is not True
        or receipt.get("protocol_id") != contract.PROTOCOL_ID
        or receipt.get("fit_stage") != stage
        or receipt.get("pipeline_claim") != pipeline_claim
        or receipt.get("label_corpus") != corpus_record
        or not _artifact_matches(
            receipt.get("worker_claim"), contract.worker_claim_path(f"label_{stage}")
        )
    ):
        raise V12R3RecoveryWeightedFitError(f"observer label receipt drifted: {stage}")
    arrays = _read_npz_exact(corpus_path, _LABEL_KEYS)
    rows = receipt["labelled_row_count"]
    if type(rows) is not int or not 1 <= rows <= contract.EXPECTED_STEPS:
        raise V12R3RecoveryWeightedFitError("observer label row count drifted")
    observations = arrays["observations"]
    actions = arrays["actions"]
    reset = arrays["reset_mask"]
    if (
        observations.shape != (rows, 35)
        or observations.dtype != np.dtype("float32")
        or actions.shape != (rows, 2)
        or actions.dtype != np.dtype("float32")
        or reset.shape != (rows,)
        or reset.dtype != np.dtype("bool")
        or not _finite_array(observations)
        or not _finite_array(actions)
        or np.any(np.abs(actions) > 1.0)
        or int(np.count_nonzero(reset)) != 1
        or not bool(reset[0])
        or not np.array_equal(arrays["step_indices"], np.arange(1, rows + 1))
        or len(set(arrays["case_ids"].astype(str).tolist())) != 1
        or set(arrays["tranche_ids"].astype(str).tolist())
        != {f"observer_probe_{stage}"}
    ):
        raise V12R3RecoveryWeightedFitError(f"observer label arrays drifted: {stage}")
    coverage = evaluate_observer_coverage(observations)
    expected_raw, expected_normalized = observer_episode_weights(
        arrays["previous_penetration_m"], reset, coverage["ood_mask"]
    )
    exact_arrays = {
        "coverage_distance_rms_z": coverage["distance_rms_z"],
        "coverage_nearest_reference_index": coverage["nearest_reference_index"],
        "coverage_ood_mask": coverage["ood_mask"],
        "raw_sample_weights": expected_raw,
        "normalized_sample_weights": expected_normalized,
    }
    mismatches = [
        name
        for name, expected in exact_arrays.items()
        if not _bytes_equal(arrays[name], expected)
    ]
    if mismatches:
        raise V12R3RecoveryWeightedFitError(
            f"observer label independently recomputed arrays differ: {stage}/{mismatches}"
        )
    case_id = str(arrays["case_ids"][0])
    expected_origins = np.asarray(
        [f"pure_observer:{stage}:{case_id}:{step}" for step in range(1, rows + 1)],
        dtype="U160",
    )
    if not _bytes_equal(arrays["origins"], expected_origins):
        raise V12R3RecoveryWeightedFitError(
            f"observer label origin order drifted: {stage}"
        )
    episode_ids = np.repeat(
        np.asarray([_episode_id("pure_observer", stage, case_id)], dtype="U192"),
        rows,
    )
    piece = {
        **{name: arrays[name] for name in _SHIELDED_CORE_KEYS},
        "episode_ids": episode_ids,
        "raw_sample_weights": expected_raw,
        "normalized_sample_weights": expected_normalized,
        "source_record": corpus_record,
        "family": "pure_observer",
    }
    binding = {
        "fit_stage": stage,
        "receipt": _record(receipt_path),
        "receipt_payload": receipt,
        "label_corpus": corpus_record,
        "labelled_row_count": rows,
        "worker_claim": dict(receipt["worker_claim"]),
        "passed": True,
    }
    return piece, binding


def _load_shielded_piece(
    round_index: int,
    case_id: str,
    pipeline_claim: Mapping[str, Any],
) -> tuple[dict[str, Any], dict[str, Any]]:
    case = contract.canonical_collection_case(case_id, round_index)
    root = _resolve(case["destination"])
    receipt_path = root / "receipt.json"
    receipt = _mapping(receipt_path)
    corpus_path = root / "labels.npz"
    corpus_record = _record(corpus_path)
    expected_receipt_keys = {
        "schema_version",
        "status",
        "passed",
        "protocol_id",
        "round_index",
        "case_id",
        "sample_count",
        "label_corpus",
        "pipeline_claim",
        "worker_claim",
    }
    if (
        set(receipt) != expected_receipt_keys
        or receipt.get("schema_version") != contract.SCHEMA_VERSION
        or receipt.get("status") != contract.COLLECTION_PASS_STATUS
        or receipt.get("passed") is not True
        or receipt.get("protocol_id") != contract.PROTOCOL_ID
        or receipt.get("round_index") != round_index
        or receipt.get("case_id") != case_id
        or receipt.get("sample_count") != contract.EXPECTED_STEPS
        or receipt.get("label_corpus") != corpus_record
        or receipt.get("pipeline_claim") != pipeline_claim
        or not _artifact_matches(
            receipt.get("worker_claim"),
            contract.worker_claim_path(f"collect_r{round_index}__{case_id}"),
        )
    ):
        raise V12R3RecoveryWeightedFitError(
            f"shielded collection receipt drifted: r{round_index}/{case_id}"
        )
    resolved = _resolve(corpus_path)
    try:
        with np.load(resolved, allow_pickle=False) as archive:
            keys = set(archive.files)
            allowed = (_SHIELDED_CORE_KEYS, _SHIELDED_CORE_KEYS | {"training_indices"})
            if keys not in allowed:
                raise V12R3RecoveryWeightedFitError(
                    f"shielded label NPZ keys drifted: {sorted(keys)}"
                )
            arrays = {name: archive[name] for name in archive.files}
    except V12R3RecoveryWeightedFitError:
        raise
    except Exception as exc:
        raise V12R3RecoveryWeightedFitError(
            f"cannot read shielded collection labels: {resolved}"
        ) from exc
    rows = contract.EXPECTED_STEPS
    reset = arrays["reset_mask"]
    if (
        arrays["observations"].shape != (rows, 35)
        or arrays["observations"].dtype != np.dtype("float32")
        or arrays["actions"].shape != (rows, 2)
        or arrays["actions"].dtype != np.dtype("float32")
        or reset.shape != (rows,)
        or reset.dtype != np.dtype("bool")
        or not _finite_array(arrays["observations"])
        or not _finite_array(arrays["actions"])
        or np.any(np.abs(arrays["actions"]) > 1.0)
        or int(np.count_nonzero(reset)) != 1
        or not bool(reset[0])
        or not np.array_equal(arrays["step_indices"], np.arange(1, rows + 1))
        or set(arrays["case_ids"].astype(str).tolist()) != {case_id}
    ):
        raise V12R3RecoveryWeightedFitError(
            f"shielded collection arrays drifted: r{round_index}/{case_id}"
        )
    raw, normalized = _shielded_episode_weights(reset)
    episode_ids = np.repeat(
        np.asarray(
            [_episode_id("v12r3_shielded", f"r{round_index}", case_id)],
            dtype="U192",
        ),
        rows,
    )
    piece = {
        **{name: arrays[name] for name in _SHIELDED_CORE_KEYS},
        "episode_ids": episode_ids,
        "raw_sample_weights": raw,
        "normalized_sample_weights": normalized,
        "source_record": corpus_record,
        "family": "v12r3_shielded",
    }
    binding = {
        "round_index": round_index,
        "case_id": case_id,
        "receipt": _record(receipt_path),
        "receipt_payload": receipt,
        "label_corpus": corpus_record,
        "worker_claim": dict(receipt["worker_claim"]),
        "data_passed": True,
    }
    return piece, binding


def _validate_piece(piece: Mapping[str, Any]) -> None:
    count = len(piece["observations"])
    if (
        piece["observations"].shape != (count, 35)
        or piece["actions"].shape != (count, 2)
        or piece["reset_mask"].shape != (count,)
        or piece["case_ids"].shape != (count,)
        or piece["step_indices"].shape != (count,)
        or piece["tranche_ids"].shape != (count,)
        or piece["origins"].shape != (count,)
        or piece["episode_ids"].shape != (count,)
        or piece["raw_sample_weights"].shape != (count,)
        or piece["normalized_sample_weights"].shape != (count,)
    ):
        raise V12R3RecoveryWeightedFitError("fit corpus piece shape drifted")
    identities = piece["episode_ids"].astype(str)
    for identity in dict.fromkeys(identities.tolist()):
        weights = piece["normalized_sample_weights"][identities == identity]
        if math.fsum(float(value) for value in weights) != EPISODE_TARGET_MASS:
            raise V12R3RecoveryWeightedFitError(
                f"episode normalized mass drifted: {identity}"
            )


def expected_corpus_component_order(stage: str) -> tuple[tuple[Any, ...], ...]:
    """Return the frozen, filesystem-independent component order for a fit."""

    if stage not in contract.FIT_STAGES:
        raise V12R3RecoveryWeightedFitError(f"unknown fit stage: {stage!r}")
    order: list[tuple[Any, ...]] = [("v11_seed",)]
    for prior_index in range(contract.FIT_STAGES.index(stage)):
        prior_stage = contract.FIT_STAGES[prior_index]
        order.append(("pure_observer", prior_stage))
        round_index = prior_index + 1
        order.extend(
            ("v12r3_shielded", round_index, case_id)
            for case_id in contract.COLLECTION_CASE_IDS
        )
    return tuple(order)


def build_fit_corpus_audit(
    *,
    v11_seed_rows: int,
    dagger_rows: int,
    pure_rows: int,
    sample_count: int,
    reset_row_count: int,
    duplicate_sample_count: int,
    all_finite: bool,
) -> dict[str, Any]:
    """Build the single canonical wire audit consumed by the real fit gate."""

    values = (
        v11_seed_rows,
        dagger_rows,
        pure_rows,
        sample_count,
        reset_row_count,
        duplicate_sample_count,
    )
    if any(type(value) is not int or value < 0 for value in values):
        raise V12R3RecoveryWeightedFitError("fit corpus audit counts are invalid")
    if type(all_finite) is not bool:
        raise V12R3RecoveryWeightedFitError("fit corpus finite flag is invalid")
    return {
        "v11_seed_sample_count": v11_seed_rows,
        "v12_dagger_sample_count": dagger_rows,
        "same_state_v12_dagger_sample_count": dagger_rows,
        "pure_probe_label_sample_count": pure_rows,
        "same_state_pure_probe_label_sample_count": pure_rows,
        "sample_count": sample_count,
        "reset_row_count": reset_row_count,
        "duplicate_sample_count": duplicate_sample_count,
        "all_finite": all_finite,
    }


def load_fit_corpus(
    *, stage: str, pipeline_claim_path: str | PurePath | Path
) -> RecoveryFitCorpus:
    """Build the exact cumulative corpus in seed/label/collection order."""

    if stage not in contract.FIT_STAGES:
        raise V12R3RecoveryWeightedFitError(f"unknown fit stage: {stage!r}")
    pipeline_claim_record = _record(pipeline_claim_path)
    if _resolve(pipeline_claim_path) != _resolve(contract.PIPELINE_CLAIM_PATH):
        raise V12R3RecoveryWeightedFitError("pipeline claim path is not canonical")
    pieces: list[dict[str, Any]] = []
    label_bindings: list[dict[str, Any]] = []
    collection_bindings: list[dict[str, Any]] = []
    stage_index = contract.FIT_STAGES.index(stage)
    # Ordering is frozen: seed, then observer P0, shielded R1, observer P1,
    # shielded R2, and so on.  It is never sorted from filesystem discovery.
    for descriptor in expected_corpus_component_order(stage):
        if descriptor[0] == "v11_seed":
            pieces.append(_seed_piece())
        elif descriptor[0] == "pure_observer":
            label_piece, label_binding = _load_observer_piece(
                str(descriptor[1]), pipeline_claim_record
            )
            pieces.append(label_piece)
            label_bindings.append(label_binding)
        else:
            round_index = int(descriptor[1])
            case_id = str(descriptor[2])
            collection_piece, collection_binding = _load_shielded_piece(
                round_index, case_id, pipeline_claim_record
            )
            pieces.append(collection_piece)
            collection_bindings.append(collection_binding)
    for piece in pieces:
        _validate_piece(piece)

    observations = np.ascontiguousarray(
        np.concatenate([piece["observations"] for piece in pieces]), dtype=np.float32
    )
    actions = np.ascontiguousarray(
        np.concatenate([piece["actions"] for piece in pieces]), dtype=np.float32
    )
    reset = np.ascontiguousarray(
        np.concatenate([piece["reset_mask"] for piece in pieces]), dtype=np.bool_
    )
    case_ids = np.asarray(
        np.concatenate([piece["case_ids"] for piece in pieces]), dtype="U64"
    )
    step_indices = np.asarray(
        np.concatenate([piece["step_indices"] for piece in pieces]), dtype=np.int64
    )
    tranche_ids = np.asarray(
        np.concatenate([piece["tranche_ids"] for piece in pieces]), dtype="U64"
    )
    origins = np.asarray(
        np.concatenate([piece["origins"] for piece in pieces]), dtype="U160"
    )
    episode_ids = np.asarray(
        np.concatenate([piece["episode_ids"] for piece in pieces]), dtype="U192"
    )
    raw_weights = np.ascontiguousarray(
        np.concatenate([piece["raw_sample_weights"] for piece in pieces]),
        dtype=np.float64,
    )
    normalized_weights = np.ascontiguousarray(
        np.concatenate([piece["normalized_sample_weights"] for piece in pieces]),
        dtype=np.float64,
    )
    labelled_rows = {
        str(binding["fit_stage"]): int(binding["labelled_row_count"])
        for binding in label_bindings
    }
    expected = contract.expected_fit_counts(stage, labelled_probe_rows=labelled_rows)
    episode_order = list(dict.fromkeys(episode_ids.astype(str).tolist()))
    identities = list(
        zip(
            tranche_ids.astype(str).tolist(),
            case_ids.astype(str).tolist(),
            step_indices.tolist(),
        )
    )
    pure_rows = sum(
        len(piece["observations"])
        for piece in pieces
        if piece["family"] == "pure_observer"
    )
    dagger_rows = sum(
        len(piece["observations"])
        for piece in pieces
        if piece["family"] == "v12r3_shielded"
    )
    if (
        len(observations) != expected["sample_count"]
        or int(np.count_nonzero(reset)) != expected["reset_row_count"]
        or len(episode_order) != 12 + 2 * stage_index + stage_index
        or len(identities) != len(set(identities))
        or math.fsum(float(value) for value in normalized_weights)
        != len(episode_order) * EPISODE_TARGET_MASS
        or not _finite_array(observations)
        or not _finite_array(actions)
        or not _finite_array(normalized_weights)
    ):
        raise V12R3RecoveryWeightedFitError(
            f"{stage} cumulative corpus contract drifted"
        )
    feature_names = np.asarray(pieces[0]["actor_feature_names"], dtype="U64")
    if any(
        not _bytes_equal(piece["actor_feature_names"], feature_names)
        for piece in pieces
    ):
        raise V12R3RecoveryWeightedFitError("actor feature names drifted")
    audit = build_fit_corpus_audit(
        v11_seed_rows=6000,
        dagger_rows=dagger_rows,
        pure_rows=pure_rows,
        sample_count=len(observations),
        reset_row_count=int(np.count_nonzero(reset)),
        duplicate_sample_count=len(identities) - len(set(identities)),
        all_finite=True,
    )
    return RecoveryFitCorpus(
        observations=observations,
        actions=actions,
        reset_mask=reset,
        actor_feature_names=feature_names,
        case_ids=case_ids,
        step_indices=step_indices,
        tranche_ids=tranche_ids,
        origins=origins,
        episode_ids=episode_ids,
        raw_sample_weights=raw_weights,
        normalized_sample_weights=normalized_weights,
        source_records={
            "v11_seed": pieces[0]["source_record"],
            "observer_labels": [
                piece["source_record"]
                for piece in pieces
                if piece["family"] == "pure_observer"
            ],
            "shielded_collections": [
                piece["source_record"]
                for piece in pieces
                if piece["family"] == "v12r3_shielded"
            ],
        },
        probe_label_bindings=tuple(label_bindings),
        collection_bindings=tuple(collection_bindings),
        audit=audit,
    )


def adamw_learning_rate(epoch: int) -> float:
    try:
        return v11.adamw_learning_rate(epoch)
    except Exception as exc:
        raise V12R3RecoveryWeightedFitError(
            f"AdamW epoch outside frozen schedule: {epoch!r}"
        ) from exc


def fit_recovery_weighted_full_mean_in_memory(
    *,
    source_state: Mapping[str, Any],
    observations: Any,
    targets: Any,
    reset_mask: Any,
    sample_weights: Any,
    normalization: FrozenNormalization,
) -> InMemoryFitResult:
    """Run the sole fixed AdamW+LBFGS design with explicit V12R3 weights."""

    import torch

    try:
        v11.validate_source_h0_state(source_state)
    except Exception as exc:
        raise V12R3RecoveryWeightedFitError(
            "source H0 state validation failed"
        ) from exc
    raw = np.ascontiguousarray(observations, dtype=np.float32)
    labels = np.ascontiguousarray(targets, dtype=np.float32)
    reset = np.ascontiguousarray(reset_mask, dtype=np.bool_)
    weights_np = np.ascontiguousarray(sample_weights, dtype=np.float64)
    if (
        raw.ndim != 2
        or raw.shape[1] != 35
        or labels.shape != (len(raw), 2)
        or reset.shape != (len(raw),)
        or weights_np.shape != (len(raw),)
        or not _finite_array(raw)
        or not _finite_array(labels)
        or not _finite_array(weights_np)
        or np.any(weights_np <= 0.0)
    ):
        raise V12R3RecoveryWeightedFitError("weighted fit corpus arrays are malformed")
    normalized = normalized_observations(raw, normalization)
    previous_threads = torch.get_num_threads()
    previous_deterministic = torch.are_deterministic_algorithms_enabled()
    torch.set_num_threads(DETERMINISTIC_TORCH_THREADS)
    torch.use_deterministic_algorithms(True)
    try:
        torch.manual_seed(20260807)
        model = v11._new_normalized_model(source_state, normalization)
        x = torch.as_tensor(normalized, dtype=torch.float32)
        y = torch.as_tensor(labels, dtype=torch.float32)
        # Preserve the persisted exact float64 episode weights in the actual
        # objective.  Gradients are accumulated into the float32 model.
        weights = torch.as_tensor(weights_np, dtype=torch.float64)
        weight_sum = torch.sum(weights)
        history: list[dict[str, Any]] = []

        optimizer = torch.optim.AdamW(
            model.parameters(), lr=3.0e-4, weight_decay=1.0e-7
        )
        milestones = {1, 250, 500, 1000, 1500, 2000, 2500, 3000}
        for epoch in range(1, 3001):
            rate = adamw_learning_rate(epoch)
            for group in optimizer.param_groups:
                group["lr"] = rate
            optimizer.zero_grad(set_to_none=True)
            prediction = model(x)
            row_loss = torch.mean(torch.square(prediction - y), dim=1)
            loss = torch.sum(weights * row_loss) / weight_sum
            if not torch.isfinite(loss):
                raise V12R3RecoveryWeightedFitError(
                    f"non-finite AdamW loss at epoch {epoch}"
                )
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), 10.0)
            optimizer.step()
            if epoch in milestones:
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
            closure_loss = torch.sum(weights * row_loss) / weight_sum
            if not torch.isfinite(closure_loss):
                raise V12R3RecoveryWeightedFitError(
                    f"non-finite LBFGS loss at closure {closure_calls + 1}"
                )
            closure_loss.backward()
            closure_calls += 1
            last_lbfgs_loss = closure_loss
            if closure_calls in {1, 50, 100, 200, 300, 400, 600}:
                v11._milestone(
                    history,
                    stage="lbfgs_closure",
                    index=closure_calls,
                    loss=closure_loss,
                    lr=0.7,
                )
            return closure_loss

        lbfgs.step(closure)
        if last_lbfgs_loss is None:
            raise V12R3RecoveryWeightedFitError(
                "LBFGS did not evaluate the weighted objective"
            )
        v11._milestone(
            history,
            stage="lbfgs_final",
            index=closure_calls,
            loss=last_lbfgs_loss,
            lr=0.7,
        )
        candidate_state, fold_audit = v11._fold_normalization_into_state(
            model, source_state, normalization
        )
        with torch.no_grad():
            normalized_prediction = np.ascontiguousarray(
                model(x).cpu().numpy(), dtype=np.float32
            )
        runtime_logits = v11._state_logits(candidate_state, raw)
        runtime_prediction = runtime_logits[:, :2]
        normalization_audit = {
            **fold_audit,
            **v11.fold_equivalence_audit(normalized_prediction, runtime_prediction),
            "normalization": normalization.record(),
        }
        preservation = v11.full_mean_update_audit(source_state, candidate_state)
        source_logits = v11._state_logits(source_state, raw)
        preservation = {
            **preservation,
            "logstd_outputs_bit_exact": bool(
                source_logits[:, 2:].tobytes() == runtime_logits[:, 2:].tobytes()
            ),
        }
        preservation["passed"] = bool(
            preservation["passed"] and preservation["logstd_outputs_bit_exact"]
        )
        if not preservation["passed"]:
            raise V12R3RecoveryWeightedFitError("mean-only preservation audit failed")
        metrics = v11.prediction_metrics(runtime_prediction, labels, reset)
        optimizer_audit = {
            "fit_contract_id": FIT_CONTRACT_ID,
            "seed": 20260807,
            "full_batch": True,
            "sample_count": len(raw),
            "explicit_sample_weights": True,
            "sample_weight_dtype": "float64",
            "sample_weights_sha256": array_sha256(weights_np),
            "normalized_total_sample_mass": float(
                math.fsum(float(value) for value in weights_np)
            ),
            "adamw_epochs": 3000,
            "adamw_schedule": copy.deepcopy(
                contract.FIT["adamw"]["learning_rate_schedule"]
            ),
            "adamw_weight_decay": 1.0e-7,
            "gradient_clip_norm": 10.0,
            "lbfgs_lr": 0.7,
            "lbfgs_max_iter": 300,
            "lbfgs_max_eval": 600,
            "lbfgs_tolerance_grad": 1.0e-10,
            "lbfgs_tolerance_change": 1.0e-12,
            "lbfgs_history_size": 50,
            "lbfgs_line_search": "strong_wolfe",
            "lbfgs_closure_calls": closure_calls,
            "hard_polish": False,
            "fallback": False,
            "sweep": False,
            "torch_threads": DETERMINISTIC_TORCH_THREADS,
            "deterministic_algorithms_enabled": True,
        }
        return InMemoryFitResult(
            candidate_state=candidate_state,
            predictions=runtime_prediction,
            metrics=metrics,
            normalization=normalization,
            normalization_audit=normalization_audit,
            preservation_audit=preservation,
            history=tuple(history),
            optimizer_audit=optimizer_audit,
        )
    except V12R3RecoveryWeightedFitError:
        raise
    except Exception as exc:
        raise V12R3RecoveryWeightedFitError("fixed weighted fit failed") from exc
    finally:
        torch.use_deterministic_algorithms(previous_deterministic)
        torch.set_num_threads(previous_threads)


def _source_h0_path() -> Path:
    return _resolve(contract.SOURCE_H0_MODULE_PATH)


def _load_source_module_and_state() -> tuple[Any, dict[str, Any]]:
    try:
        return v11._load_source_module_and_state()
    except Exception as exc:
        raise V12R3RecoveryWeightedFitError("cannot load immutable H0 module") from exc


def _save_candidate_exact(
    *, source_module: Any, candidate_state: Mapping[str, Any], destination: Path
) -> dict[str, Any]:
    try:
        return v11._save_candidate_exact(
            source_module=source_module,
            candidate_state=candidate_state,
            destination=destination,
        )
    except Exception as exc:
        raise V12R3RecoveryWeightedFitError("candidate save/reload failed") from exc


def _source_tree_checked() -> dict[str, Any]:
    record = _tree_record(_source_h0_path())
    if record.get("tree_sha256") != contract.SOURCE_H0_TREE_SHA256:
        raise V12R3RecoveryWeightedFitError("source H0 tree hash drifted")
    return record


def run_design_audit_in_memory() -> dict[str, Any]:
    """Execute the fixed P0 design once without persisting any checkpoint."""

    # A design audit has no pipeline claim yet, so compose the immutable seed
    # directly.  P0 contains no R1 label or collection input.
    piece = _seed_piece()
    _validate_piece(piece)
    base = np.ascontiguousarray(piece["observations"][:3000], dtype=np.float32)
    normalization = frozen_base_normalization(base)
    coverage = load_coverage_reference()
    source_before = _source_tree_checked()
    _module, source_state = _load_source_module_and_state()
    result = fit_recovery_weighted_full_mean_in_memory(
        source_state=source_state,
        observations=piece["observations"],
        targets=piece["actions"],
        reset_mask=piece["reset_mask"],
        sample_weights=piece["normalized_sample_weights"],
        normalization=normalization,
    )
    if source_before != _source_tree_checked():
        raise V12R3RecoveryWeightedFitError("source H0 changed during design audit")
    thresholds = contract.OFFLINE_THRESHOLDS
    checks = {
        "rmse": result.metrics["rmse"] <= thresholds["rmse_max"],
        "max_abs_error": result.metrics["max_abs_error"]
        <= thresholds["max_abs_error_max"],
        "reset_max_abs_error": result.metrics["reset_max_abs_error"]
        <= thresholds["reset_max_abs_error_max"],
    }
    if not all(checks.values()):
        raise V12R3RecoveryWeightedFitError(
            f"P0 design audit offline gates failed: {checks}"
        )
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.DESIGN_AUDIT_PASS_STATUS,
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "contract_id": FIT_CONTRACT_ID,
        "design_audit_id": contract.DESIGN_AUDIT_ID,
        "dry_run": True,
        "no_candidate_checkpoint": True,
        "fit_design": copy.deepcopy(contract.FIT),
        "observed_metrics": dict(result.metrics),
        "p0_reproduction_reference_metrics": dict(result.metrics),
        "p0_reproduction_tolerance": copy.deepcopy(contract.P0_REPRODUCTION_TOLERANCE),
        "metric_gate_checks": checks,
        "source_h0": source_before,
        "v11_seed_corpus": copy.deepcopy(contract.V11_P3_CORPUS_ARTIFACT),
        "normalization": normalization.record(),
        "coverage_reference_audit": copy.deepcopy(dict(coverage.audit)),
        "episode_count": 12,
        "episode_target_mass": EPISODE_TARGET_MASS,
        "normalized_total_sample_mass": 12 * EPISODE_TARGET_MASS,
        "sample_weights_sha256": array_sha256(piece["normalized_sample_weights"]),
        "optimizer_audit": dict(result.optimizer_audit),
        "normalization_audit": dict(result.normalization_audit),
        "preservation_audit": dict(result.preservation_audit),
        "history": [dict(row) for row in result.history],
        "actor_fit_executions": 1,
        "actor_updates": 1,
        "candidate_checkpoints_persisted": 0,
        "candidate_checkpoint_paths": [],
        "environment_reset_calls": 0,
        "environment_step_calls": 0,
        "offline_teacher_label_calls": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
        "retry_authorized": False,
        "sweep_authorized": False,
        "rescue_authorized": False,
    }


def _metric_match(
    observed: Mapping[str, Any], expected: Mapping[str, Any]
) -> dict[str, Any]:
    tolerance = contract.P0_REPRODUCTION_TOLERANCE
    absolute = float(tolerance.get("absolute", tolerance.get("abs", 1.0e-10)))
    relative = float(tolerance.get("relative", tolerance.get("rel", 1.0e-8)))
    differences: dict[str, float] = {}
    checks: dict[str, bool] = {}
    for name in ("rmse", "max_abs_error", "reset_max_abs_error"):
        left = float(observed[name])
        right = float(expected[name])
        differences[name] = abs(left - right)
        checks[name] = math.isclose(left, right, rel_tol=relative, abs_tol=absolute)
    return {
        "passed": all(checks.values()),
        "checks": checks,
        "absolute_differences": differences,
        "absolute_tolerance": absolute,
        "relative_tolerance": relative,
    }


def _canonical_destination(stage: str, output_dir: str | PurePath | Path) -> Path:
    destination = _resolve(output_dir)
    expected = _resolve(contract.FIT_ROOTS[stage])
    if destination != expected:
        raise V12R3RecoveryWeightedFitError(
            f"non-canonical fit destination: {destination}"
        )
    if os.path.lexists(destination):
        raise V12R3RecoveryWeightedFitError(
            f"fit destination exists/no-clobber: {destination}"
        )
    return destination


def run_fit_stage(
    *,
    stage: str,
    output_dir: str | PurePath | Path,
    pipeline_claim_path: str | PurePath | Path,
    worker_claim_path: str | PurePath | Path,
    design_audit_path: str | PurePath | Path | None = None,
    execution_lock_path: str | PurePath | Path | None = None,
) -> dict[str, Any]:
    """Run and publish one canonical, no-clobber V12R3 fit stage."""

    if stage not in contract.FIT_STAGES:
        raise V12R3RecoveryWeightedFitError(f"unknown fit stage: {stage!r}")
    design_path = _resolve(
        contract.DESIGN_AUDIT_RECEIPT_PATH
        if design_audit_path is None
        else design_audit_path
    )
    lock_path = _resolve(
        contract.EXECUTION_LOCK_PATH
        if execution_lock_path is None
        else execution_lock_path
    )
    if design_path != _resolve(contract.DESIGN_AUDIT_RECEIPT_PATH):
        raise V12R3RecoveryWeightedFitError("design audit path is not canonical")
    if lock_path != _resolve(contract.EXECUTION_LOCK_PATH):
        raise V12R3RecoveryWeightedFitError("execution lock path is not canonical")
    pipeline_record = _record(pipeline_claim_path)
    worker_record = _record(worker_claim_path)
    design_record = _record(design_path)
    lock_record = _record(lock_path)
    if _resolve(worker_claim_path) != _resolve(
        contract.worker_claim_path(f"fit_{stage}")
    ):
        raise V12R3RecoveryWeightedFitError("fit worker claim path is not canonical")
    design_payload = _mapping(design_path)
    lock_payload = _mapping(lock_path)
    if (
        design_payload.get("passed") is not True
        or lock_payload.get("passed") is not True
    ):
        raise V12R3RecoveryWeightedFitError("design audit/execution lock is not PASS")

    corpus = load_fit_corpus(stage=stage, pipeline_claim_path=pipeline_claim_path)
    destination = _canonical_destination(stage, output_dir)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.mkdir(exist_ok=False)
    corpus_path = v11.v10s_fit._write_npz_exclusive(
        destination / "corpus.npz", corpus.arrays()
    )
    normalization = frozen_base_normalization(corpus.observations[:3000])
    source_before = _source_tree_checked()
    source_module, source_state = _load_source_module_and_state()
    result = fit_recovery_weighted_full_mean_in_memory(
        source_state=source_state,
        observations=corpus.observations,
        targets=corpus.actions,
        reset_mask=corpus.reset_mask,
        sample_weights=corpus.normalized_sample_weights,
        normalization=normalization,
    )
    module_path = destination / "rl_module_target_adapted"
    save_reload = _save_candidate_exact(
        source_module=source_module,
        candidate_state=result.candidate_state,
        destination=module_path,
    )
    if source_before != _source_tree_checked():
        raise V12R3RecoveryWeightedFitError("source H0 changed during fit")
    module_record = _tree_record(module_path)
    if module_record["path"] != contract.MODULE_PATHS[stage].as_posix():
        raise V12R3RecoveryWeightedFitError("candidate module path drifted")

    report = {
        "fit_contract_id": FIT_CONTRACT_ID,
        "fit_stage": stage,
        "fit": copy.deepcopy(contract.FIT),
        "training_samples": len(corpus.observations),
        "validation_samples": 0,
        "normalization": normalization.record(),
        "optimizer_audit": dict(result.optimizer_audit),
        "preservation_audit": dict(result.preservation_audit),
        "save_reload": save_reload,
        "corpus_exact": True,
        "module_reload_exact": save_reload["exact"],
    }
    history_path = forensic.write_json_exclusive(
        destination / "adaptation_history.json",
        [dict(row) for row in result.history],
    )
    report_path = forensic.write_json_exclusive(
        destination / "adaptation_report.json", report
    )
    expected = contract.expected_fit_counts(
        stage,
        labelled_probe_rows={
            str(binding["fit_stage"]): int(binding["labelled_row_count"])
            for binding in corpus.probe_label_bindings
        },
    )
    reproduction = (
        _metric_match(
            result.metrics,
            design_payload.get(
                "p0_reproduction_reference_metrics",
                design_payload.get("observed_metrics", {}),
            ),
        )
        if stage == "p0"
        else {"passed": True, "checks": {}}
    )
    summary = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.FIT_COMPLETE_STATUS,
        "protocol_id": contract.PROTOCOL_ID,
        "fit_stage": stage,
        "fit_contract_id": FIT_CONTRACT_ID,
        "fit": copy.deepcopy(contract.FIT),
        "actor_architecture": copy.deepcopy(contract.ACTOR_ARCHITECTURE),
        "normalization": copy.deepcopy(contract.BASE_CORPUS_NORMALIZATION),
        "trainable_scope": contract.FIT["trainable_scope"],
        "source_h0_id": contract.SOURCE_H0_ID,
        "source_h0": source_before,
        "initial_checkpoint_id": contract.SOURCE_H0_ID,
        "continued_from_previous_candidate": False,
        "candidate_module": module_record,
        "candidate_id": contract.candidate_id(stage, module_record["tree_sha256"]),
        "component_receipts": {
            "design_audit": design_record,
            "execution_lock": lock_record,
            "pipeline_claim": pipeline_record,
            "worker_claim": worker_record,
        },
        "design_audit_passed": True,
        "execution_lock_passed": True,
        "pipeline_claimed": True,
        "teacher_evidence_id": contract.TEACHER_EVIDENCE_ID,
        "teacher_evidence_passed": True,
        "teacher_evidence_receipt": copy.deepcopy(contract.TEACHER_EVIDENCE_ARTIFACT),
        "v11_seed_corpus": copy.deepcopy(contract.V11_P3_CORPUS_ARTIFACT),
        "v11_seed_corpus_audit_passed": True,
        "collection_corpus_receipts": [
            dict(binding) for binding in corpus.collection_bindings
        ],
        "probe_label_receipts": [
            dict(binding) for binding in corpus.probe_label_bindings
        ],
        "fit_artifacts": {
            "corpus": _record(corpus_path),
            "adaptation_history": _record(history_path),
            "adaptation_report": _record(report_path),
        },
        "corpus_artifact": _record(corpus_path),
        "fit_counts": expected,
        "sample_count": len(corpus.observations),
        "reset_row_count": int(np.count_nonzero(corpus.reset_mask)),
        "labelled_probe_rows": {
            str(binding["fit_stage"]): int(binding["labelled_row_count"])
            for binding in corpus.probe_label_bindings
        },
        "corpus_audit": dict(corpus.audit),
        "report_checks": {
            "corpus_exact": True,
            "module_reload_exact": save_reload["exact"],
        },
        "adamw_epochs_run": 3000,
        "lbfgs_completed": True,
        "deterministic_algorithms_enabled": result.optimizer_audit[
            "deterministic_algorithms_enabled"
        ],
        "metrics": dict(result.metrics),
        "episode_count": len(set(corpus.episode_ids.astype(str).tolist())),
        "episode_target_mass": EPISODE_TARGET_MASS,
        "normalized_total_sample_mass": len(
            set(corpus.episode_ids.astype(str).tolist())
        )
        * EPISODE_TARGET_MASS,
        "recovery_weighting": copy.deepcopy(contract.RECOVERY_WEIGHTING),
        "coverage_weighting": copy.deepcopy(contract.COVERAGE_WEIGHTING),
        "shielded_nonreset_raw_weight": 1.0,
        "shielded_reset_raw_weight": 100.0,
        "row_loss": contract.RECOVERY_WEIGHTING["row_loss"],
        "corpus_loss_reduction": contract.RECOVERY_WEIGHTING["corpus_loss_reduction"],
        "disabled_clock_column_indices": [0, 1],
        "disabled_clock_columns_bit_zero": result.preservation_audit[
            "disabled_clock_columns_bit_zero"
        ],
        "disabled_clock_columns_bit_zero_after_save_reload": save_reload[
            "clock_columns_bit_zero"
        ],
        "normalization_stats_from_base_corpus_only": True,
        "normalization_stats_frozen_across_stages": True,
        "normalization_folded_into_first_layer": True,
        "runtime_normalization_wrapper_present": False,
        "prescribed_clock_present": False,
        "fold_equivalence_passed": result.normalization_audit[
            "fold_equivalence_passed"
        ],
        "anchor_used": False,
        "hard_polish_used": False,
        "design_audit_reproduction_within_tolerance": reproduction["passed"],
        "design_audit_reproduction": reproduction,
        "p0_reproduction_reference_metrics": (
            copy.deepcopy(
                design_payload.get(
                    "p0_reproduction_reference_metrics",
                    design_payload.get("observed_metrics", {}),
                )
            )
            if stage == "p0"
            else {}
        ),
        "p0_reproduction_tolerance": copy.deepcopy(contract.P0_REPRODUCTION_TOLERANCE),
        "source_checkpoint_scope": result.preservation_audit["source_checkpoint_scope"],
        "critic_present": result.preservation_audit["critic_present"],
        "critic_parameter_count": result.preservation_audit["critic_parameter_count"],
        "critic_byte_exact": result.preservation_audit["critic_byte_exact"],
        "logstd_byte_exact": bool(
            result.preservation_audit["logstd_parameter_rows_bit_exact"]
            and result.preservation_audit["logstd_outputs_bit_exact"]
        ),
        "duplicate_sample_count": corpus.audit["duplicate_sample_count"],
        "all_finite": corpus.audit["all_finite"],
        "source_h0_byte_exact": source_before == _source_tree_checked(),
        "target_contract_id": contract.TARGET_CONTRACT_ID,
        "event_contract_id": contract.EVENT_CONTRACT_ID,
        "actor_updates": 1,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
    }
    gate = dict(contract.fit_gate(summary, stage=stage))
    if gate.get("passed") is not True:
        failed = [name for name, value in gate["checks"].items() if value is not True]
        raise V12R3RecoveryWeightedFitError(
            f"{stage} fit summary failed contract gate: {failed}"
        )
    summary_path = forensic.write_json_exclusive(destination / "summary.json", summary)
    gate_path = forensic.write_json_exclusive(destination / "gate.json", gate)
    receipt = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.FIT_PASS_STATUS,
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "fit_stage": stage,
        "candidate_id": summary["candidate_id"],
        "candidate_module": module_record,
        "corpus": _record(corpus_path),
        "summary": _record(summary_path),
        "gate": _record(gate_path),
        "pipeline_claim": pipeline_record,
        "worker_claim": worker_record,
        "actor_updates": 1,
        "critic_updates": 0,
        "ppo_updates": 0,
        "retry_authorized": False,
        "sweep_authorized": False,
        "rescue_authorized": False,
    }
    receipt_path = forensic.write_json_exclusive(destination / "receipt.json", receipt)
    return {**receipt, "receipt": _record(receipt_path)}


def verify_fit_stage(stage: str) -> dict[str, Any]:
    """Recompute a completed fit gate and all direct artifact bindings."""

    if stage not in contract.FIT_STAGES:
        raise V12R3RecoveryWeightedFitError(f"unknown fit stage: {stage!r}")
    root = _resolve(contract.FIT_ROOTS[stage])
    summary_path = root / "summary.json"
    gate_path = root / "gate.json"
    receipt_path = root / "receipt.json"
    summary = _mapping(summary_path)
    observed_gate = _mapping(gate_path)
    expected_gate = dict(contract.fit_gate(summary, stage=stage))
    if observed_gate != expected_gate or expected_gate.get("passed") is not True:
        raise V12R3RecoveryWeightedFitError(f"persisted fit gate drifted: {stage}")
    receipt = _mapping(receipt_path)
    expected_keys = {
        "schema_version",
        "status",
        "passed",
        "protocol_id",
        "fit_stage",
        "candidate_id",
        "candidate_module",
        "corpus",
        "summary",
        "gate",
        "pipeline_claim",
        "worker_claim",
        "actor_updates",
        "critic_updates",
        "ppo_updates",
        "retry_authorized",
        "sweep_authorized",
        "rescue_authorized",
    }
    module = _tree_record(contract.MODULE_PATHS[stage])
    expected_components = {
        "design_audit": _record(contract.DESIGN_AUDIT_RECEIPT_PATH),
        "execution_lock": _record(contract.EXECUTION_LOCK_PATH),
        "pipeline_claim": _record(contract.PIPELINE_CLAIM_PATH),
        "worker_claim": _record(contract.worker_claim_path(f"fit_{stage}")),
    }
    expected_fit_artifacts = {
        "corpus": _record(root / "corpus.npz"),
        "adaptation_history": _record(root / "adaptation_history.json"),
        "adaptation_report": _record(root / "adaptation_report.json"),
    }
    if (
        set(receipt) != expected_keys
        or summary.get("component_receipts") != expected_components
        or summary.get("source_h0") != _source_tree_checked()
        or summary.get("candidate_module") != module
        or summary.get("fit_artifacts") != expected_fit_artifacts
        or summary.get("corpus_artifact") != expected_fit_artifacts["corpus"]
        or receipt.get("schema_version") != contract.SCHEMA_VERSION
        or receipt.get("status") != contract.FIT_PASS_STATUS
        or receipt.get("passed") is not True
        or receipt.get("protocol_id") != contract.PROTOCOL_ID
        or receipt.get("fit_stage") != stage
        or receipt.get("candidate_id")
        != contract.candidate_id(stage, module["tree_sha256"])
        or receipt.get("candidate_module") != module
        or not _artifact_matches(receipt.get("corpus"), root / "corpus.npz")
        or not _artifact_matches(receipt.get("summary"), summary_path)
        or not _artifact_matches(receipt.get("gate"), gate_path)
        or not _artifact_matches(
            receipt.get("pipeline_claim"), contract.PIPELINE_CLAIM_PATH
        )
        or not _artifact_matches(
            receipt.get("worker_claim"), contract.worker_claim_path(f"fit_{stage}")
        )
        or receipt.get("actor_updates") != 1
        or receipt.get("critic_updates") != 0
        or receipt.get("ppo_updates") != 0
        or receipt.get("retry_authorized") is not False
        or receipt.get("sweep_authorized") is not False
        or receipt.get("rescue_authorized") is not False
    ):
        raise V12R3RecoveryWeightedFitError(f"persisted fit receipt drifted: {stage}")
    return receipt


__all__ = [
    "CoverageReference",
    "EPISODE_TARGET_MASS",
    "FIT_CONTRACT_ID",
    "FrozenNormalization",
    "InMemoryFitResult",
    "RecoveryFitCorpus",
    "V10SFitError",
    "V12R3RecoveryWeightedFitError",
    "adamw_learning_rate",
    "array_sha256",
    "build_fit_corpus_audit",
    "coverage_reference_audit",
    "expected_corpus_component_order",
    "evaluate_observer_coverage",
    "fit_recovery_weighted_full_mean_in_memory",
    "frozen_base_normalization",
    "load_coverage_reference",
    "load_fit_corpus",
    "normalized_episode_sample_weights",
    "normalized_observations",
    "observer_episode_weights",
    "recovery_ramp_weight",
    "run_design_audit_in_memory",
    "run_fit_stage",
    "verify_fit_stage",
]
