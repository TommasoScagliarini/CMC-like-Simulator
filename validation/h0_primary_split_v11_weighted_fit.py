"""Deterministic reset-weighted full-mean fitter for the V11 lineage.

V11 changes only the numerical actor fit which failed in terminal V10S P0.
The immutable V10S corpus validators are reused under the V11 contract; every
fit reloads the original H0, uses normalization statistics frozen from the
3,000 base rows, and trains the complete 35->256->256->2 mean network.  The
normalization is folded into the first layer before a runtime checkpoint is
saved.  No runtime wrapper, hard-example polish, fallback, sweep, critic
update, or PPO update exists in this module.

The design-audit entry point executes P0 entirely in memory.  The normal fit
entry point is API-compatible with the content-pinned V10S orchestrator.
"""

from __future__ import annotations

import copy
import math
import os
import sys
from contextlib import contextmanager
from dataclasses import dataclass
from pathlib import Path, PurePath
from typing import Any, Iterator, Mapping, Sequence

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
VALIDATION_ROOT = REPO_ROOT / "validation"
TRAJECTORY_ROOT = REPO_ROOT / "Trajectory Generator"
BASELINE_ROOT = TRAJECTORY_ROOT / "baseline_MLP"
for _root in (REPO_ROOT, VALIDATION_ROOT, TRAJECTORY_ROOT, BASELINE_ROOT):
    if str(_root) not in sys.path:
        sys.path.insert(0, str(_root))

import h0_forensic_rollout as forensic  # noqa: E402
import h0_primary_split_v10s_fit as v10s_fit  # noqa: E402
import h0_primary_split_v11_weighted_full_mean_contract as contract  # noqa: E402


FIT_CONTRACT_ID = "h0_primary_split_v11_weighted_full_mean_v1"
DESIGN_AUDIT_ID = (
    "h0_primary_split_v11_weighted_full_mean_p0_design_audit_v1"
)
CLOCK_COLUMNS = (0, 1)
BASE_NORMALIZATION_ROWS = 3000
NORMALIZATION_STD_FLOOR = 1.0e-4
DETERMINISTIC_TORCH_THREADS = 5
FOLD_EQUIVALENCE_MAX_ABS = 1.0e-6

_MEAN_HIDDEN_KEYS = (
    "pi_encoder.0.weight",
    "pi_encoder.0.bias",
    "pi_encoder.2.weight",
    "pi_encoder.2.bias",
    "pi.0.0.weight",
    "pi.0.0.bias",
    "pi.0.2.weight",
    "pi.0.2.bias",
)
_ALIASES = (
    ("pi_encoder.0.weight", "pi.0.0.weight"),
    ("pi_encoder.0.bias", "pi.0.0.bias"),
    ("pi_encoder.2.weight", "pi.0.2.weight"),
    ("pi_encoder.2.bias", "pi.0.2.bias"),
)


class V11WeightedFitError(RuntimeError):
    """Raised when a V11 fit cannot proceed without changing its design."""


# Preserve the old public exception spelling expected by the inherited runner.
V10SFitError = V11WeightedFitError


FitCorpus = v10s_fit.FitCorpus


@dataclass(frozen=True)
class FrozenNormalization:
    """Float32 normalization fixed from the immutable 3,000-row base corpus."""

    mean: np.ndarray
    std: np.ndarray
    source_rows: int
    observations_sha256: str

    def record(self) -> dict[str, Any]:
        return {
            "source": "frozen_base_corpus_only",
            "source_rows": self.source_rows,
            "std_floor": NORMALIZATION_STD_FLOOR,
            "dtype": "float32",
            "mean_sha256": array_sha256(self.mean),
            "std_sha256": array_sha256(self.std),
            "base_observations_sha256": self.observations_sha256,
            "clock_columns": list(CLOCK_COLUMNS),
            "clock_means": [float(self.mean[index]) for index in CLOCK_COLUMNS],
            "clock_stds": [float(self.std[index]) for index in CLOCK_COLUMNS],
            "folded_into_first_layer_before_save": True,
            "runtime_normalization_wrapper": False,
        }


@dataclass(frozen=True)
class InMemoryFitResult:
    """Complete numerical evidence from one fit, before optional publication."""

    candidate_state: Mapping[str, Any]
    predictions: np.ndarray
    metrics: Mapping[str, float]
    normalization: FrozenNormalization
    normalization_audit: Mapping[str, Any]
    preservation_audit: Mapping[str, Any]
    history: tuple[Mapping[str, Any], ...]
    optimizer_audit: Mapping[str, Any]


def _resolve(relative_or_absolute: str | PurePath | Path) -> Path:
    path = Path(relative_or_absolute)
    return (path if path.is_absolute() else REPO_ROOT / path).resolve()


def _mapping(path: str | Path) -> dict[str, Any]:
    try:
        value = forensic.strict_json_load(_resolve(path))
    except Exception as exc:
        raise V11WeightedFitError(f"invalid strict JSON object: {_resolve(path)}") from exc
    if not isinstance(value, Mapping):
        raise V11WeightedFitError(f"expected strict JSON object: {_resolve(path)}")
    return dict(value)


def _record(path: str | Path) -> dict[str, Any]:
    try:
        return forensic.artifact_record(_resolve(path), artifact_root=REPO_ROOT)
    except Exception as exc:
        raise V11WeightedFitError(f"cannot record artifact: {_resolve(path)}") from exc


def _tree_record(path: str | Path) -> dict[str, Any]:
    try:
        return v10s_fit._tree_record(_resolve(path))
    except Exception as exc:
        raise V11WeightedFitError(f"cannot record artifact tree: {_resolve(path)}") from exc


def _audit_source_record(path: str | Path) -> dict[str, Any]:
    tree = _tree_record(path)
    return {
        "path": tree["path"],
        "tree_sha256": tree["tree_sha256"],
        "file_hashes": {
            str(row["path"]): str(row["sha256"]) for row in tree["files"]
        },
    }


def array_sha256(value: Any) -> str:
    return v10s_fit.array_sha256(value)


def prediction_metrics(
    predictions: Any,
    targets: Any,
    reset_mask: Any | None = None,
) -> dict[str, float]:
    pred = np.ascontiguousarray(np.asarray(predictions), dtype=np.float32)
    target = np.ascontiguousarray(np.asarray(targets), dtype=np.float32)
    if pred.shape != target.shape or pred.ndim != 2 or pred.shape[1] != 2:
        raise V11WeightedFitError("predictions/targets must both have shape (N, 2)")
    if not np.all(np.isfinite(pred)) or not np.all(np.isfinite(target)):
        raise V11WeightedFitError("predictions/targets contain non-finite values")
    error = np.abs(pred.astype(np.float64) - target.astype(np.float64))
    result = {
        "rmse": float(np.sqrt(np.mean(np.square(error)))),
        "max_abs_error": float(np.max(error, initial=0.0)),
    }
    if reset_mask is not None:
        reset = np.asarray(reset_mask, dtype=np.bool_)
        if reset.shape != (len(pred),) or not np.any(reset):
            raise V11WeightedFitError("reset mask must select at least one corpus row")
        result["reset_max_abs_error"] = float(np.max(error[reset], initial=0.0))
    return result


@contextmanager
def _v11_bound_v10s_validator() -> Iterator[None]:
    """Reuse immutable V10S validators while binding all paths to V11.

    The validator module is never edited.  Its sole contract global is swapped
    for the duration of a synchronous call and restored even on failure.
    """

    previous = v10s_fit.contract
    v10s_fit.contract = contract
    try:
        yield
    finally:
        v10s_fit.contract = previous


def load_frozen_v8_corpus() -> FitCorpus:
    try:
        with _v11_bound_v10s_validator():
            return v10s_fit.load_frozen_v8_corpus()
    except Exception as exc:
        if isinstance(exc, V11WeightedFitError):
            raise
        raise V11WeightedFitError("V11 base corpus validation failed") from exc


def load_dagger_tranche(receipt_path: str | Path) -> FitCorpus:
    try:
        with _v11_bound_v10s_validator():
            return v10s_fit.load_dagger_tranche(receipt_path)
    except Exception as exc:
        if isinstance(exc, V11WeightedFitError):
            raise
        raise V11WeightedFitError("V11 DAgger tranche validation failed") from exc


def load_fit_corpus(
    dagger_receipt_paths: Sequence[str | Path], *, stage: str
) -> FitCorpus:
    """Validate a cumulative V11 corpus using the content-pinned V10S logic."""

    try:
        with _v11_bound_v10s_validator():
            return v10s_fit.load_fit_corpus(dagger_receipt_paths, stage=stage)
    except Exception as exc:
        if isinstance(exc, V11WeightedFitError):
            raise
        raise V11WeightedFitError(f"V11 {stage} cumulative corpus failed") from exc


def frozen_base_normalization(
    base_observations: Any,
) -> FrozenNormalization:
    observations = np.ascontiguousarray(base_observations, dtype=np.float32)
    if observations.shape != (BASE_NORMALIZATION_ROWS, 35):
        raise V11WeightedFitError(
            "normalization source must be exactly the frozen 3000x35 base corpus"
        )
    if not np.all(np.isfinite(observations)):
        raise V11WeightedFitError("normalization source contains non-finite values")
    if not np.all(observations[:, 0] == np.float32(0.0)):
        raise V11WeightedFitError("base gait-clock column 0 is not exactly zero")
    if not np.all(observations[:, 1] == np.float32(1.0)):
        raise V11WeightedFitError("base gait-clock column 1 is not exactly one")

    mean = observations.mean(axis=0, dtype=np.float64).astype(np.float32)
    raw_std = observations.std(axis=0, dtype=np.float64).astype(np.float32)
    std = np.ascontiguousarray(
        np.maximum(raw_std, np.float32(NORMALIZATION_STD_FLOOR)),
        dtype=np.float32,
    )
    mean = np.ascontiguousarray(mean, dtype=np.float32)
    if (
        mean[0].tobytes() != np.float32(0.0).tobytes()
        or mean[1].tobytes() != np.float32(1.0).tobytes()
        or std[0].tobytes() != np.float32(NORMALIZATION_STD_FLOOR).tobytes()
        or std[1].tobytes() != np.float32(NORMALIZATION_STD_FLOOR).tobytes()
    ):
        raise V11WeightedFitError("frozen gait-clock normalization drifted")
    return FrozenNormalization(
        mean=mean,
        std=std,
        source_rows=BASE_NORMALIZATION_ROWS,
        observations_sha256=array_sha256(observations),
    )


def normalized_observations(
    observations: Any, normalization: FrozenNormalization
) -> np.ndarray:
    raw = np.ascontiguousarray(observations, dtype=np.float32)
    if raw.ndim != 2 or raw.shape[1] != 35 or not np.all(np.isfinite(raw)):
        raise V11WeightedFitError("fit observations must be finite float32 Nx35")
    result = np.ascontiguousarray(
        (raw - normalization.mean) / normalization.std,
        dtype=np.float32,
    )
    if not np.all(np.isfinite(result)):
        raise V11WeightedFitError("normalized observations are non-finite")
    if not _columns_bit_zero(result, CLOCK_COLUMNS):
        raise V11WeightedFitError("normalized gait-clock columns are not bit-zero")
    return result


def reset_sample_weights(reset_mask: Any, *, rows: int) -> np.ndarray:
    reset = np.asarray(reset_mask, dtype=np.bool_)
    if reset.shape != (rows,) or not np.any(reset):
        raise V11WeightedFitError("reset mask must select at least one fit row")
    weights = np.ones(rows, dtype=np.float32)
    weights[reset] = np.float32(100.0)
    if set(np.unique(weights).tolist()) != {1.0, 100.0}:
        raise V11WeightedFitError("reset weighting drifted from 1/100")
    return np.ascontiguousarray(weights)


def _bytes_equal(left: Any, right: Any) -> bool:
    a = np.ascontiguousarray(np.asarray(left))
    b = np.ascontiguousarray(np.asarray(right))
    return a.dtype == b.dtype and a.shape == b.shape and a.tobytes() == b.tobytes()


def _bit_zero(value: Any) -> bool:
    array = np.ascontiguousarray(np.asarray(value))
    return array.tobytes(order="C") == bytes(array.nbytes)


def _columns_bit_zero(value: Any, columns: Sequence[int]) -> bool:
    array = np.asarray(value)
    return array.ndim == 2 and all(_bit_zero(array[:, int(index)]) for index in columns)


def _clone_state(state: Mapping[str, Any]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in state.items():
        if hasattr(value, "detach"):
            result[key] = value.detach().cpu().numpy().copy()
        elif isinstance(value, np.ndarray):
            result[key] = value.copy()
        else:
            result[key] = copy.deepcopy(value)
    return result


def validate_source_h0_state(state: Mapping[str, Any]) -> dict[str, Any]:
    required_shapes = {
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
    missing = [key for key in required_shapes if key not in state]
    malformed = [
        key
        for key, shape in required_shapes.items()
        if key in state and tuple(np.asarray(state[key]).shape) != shape
    ]
    finite = all(
        np.all(np.isfinite(np.asarray(state[key], dtype=np.float32)))
        for key in required_shapes
        if key in state
    )
    aliases_exact = all(_bytes_equal(state[left], state[right]) for left, right in _ALIASES)
    clocks_zero = (
        _columns_bit_zero(state["pi_encoder.0.weight"], CLOCK_COLUMNS)
        and _columns_bit_zero(state["pi.0.0.weight"], CLOCK_COLUMNS)
        if not missing
        else False
    )
    standard = not missing and not malformed and finite and aliases_exact and clocks_zero
    if not standard:
        raise V11WeightedFitError(
            "source H0 is not the standard 35->256->256->2 actor with disabled clocks"
        )
    return {
        "standard_actor_architecture": True,
        "input_width": 35,
        "hidden_widths": [256, 256],
        "mean_output_width": 2,
        "source_aliases_bit_exact": aliases_exact,
        "source_clock_columns_bit_zero": clocks_zero,
        "all_source_tensors_finite": finite,
    }


def _new_normalized_model(
    source_state: Mapping[str, Any], normalization: FrozenNormalization
) -> Any:
    import torch
    from torch import nn

    validate_source_h0_state(source_state)
    model = nn.Sequential(
        nn.Linear(35, 256),
        nn.Tanh(),
        nn.Linear(256, 256),
        nn.Tanh(),
        nn.Linear(256, 2),
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
    if not _columns_bit_zero(
        model[0].weight.detach().cpu().numpy(), CLOCK_COLUMNS
    ):
        raise V11WeightedFitError("normalized initialization re-enabled gait clocks")
    return model


def _state_logits(state: Mapping[str, Any], observations: Any) -> np.ndarray:
    import torch
    import torch.nn.functional as functional

    x = torch.as_tensor(
        np.ascontiguousarray(observations, dtype=np.float32), dtype=torch.float32
    )
    with torch.no_grad():
        hidden = torch.tanh(
            functional.linear(
                x,
                torch.as_tensor(state["pi_encoder.0.weight"], dtype=torch.float32),
                torch.as_tensor(state["pi_encoder.0.bias"], dtype=torch.float32),
            )
        )
        hidden = torch.tanh(
            functional.linear(
                hidden,
                torch.as_tensor(state["pi_encoder.2.weight"], dtype=torch.float32),
                torch.as_tensor(state["pi_encoder.2.bias"], dtype=torch.float32),
            )
        )
        logits = functional.linear(
            hidden,
            torch.as_tensor(state["pi.1.weight"], dtype=torch.float32),
            torch.as_tensor(state["pi.1.bias"], dtype=torch.float32),
        )
    result = np.ascontiguousarray(logits.cpu().numpy(), dtype=np.float32)
    if result.shape != (len(x), 4) or not np.all(np.isfinite(result)):
        raise V11WeightedFitError("candidate logits are malformed")
    return result


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
    if not _columns_bit_zero(normalized_w0, CLOCK_COLUMNS):
        raise V11WeightedFitError("trained normalized clock weights are not bit-zero")
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
    if not _columns_bit_zero(raw_w0, CLOCK_COLUMNS):
        raise V11WeightedFitError("folded runtime clock weights are not bit-zero")

    candidate = _clone_state(source_state)
    first_bias = raw_b0
    second_weight = np.ascontiguousarray(
        model[2].weight.detach().cpu().numpy(), dtype=np.float32
    )
    second_bias = np.ascontiguousarray(
        model[2].bias.detach().cpu().numpy(), dtype=np.float32
    )
    output_weight = np.ascontiguousarray(
        model[4].weight.detach().cpu().numpy(), dtype=np.float32
    )
    output_bias = np.ascontiguousarray(
        model[4].bias.detach().cpu().numpy(), dtype=np.float32
    )
    for prefix in ("pi_encoder", "pi.0"):
        candidate[f"{prefix}.0.weight"] = raw_w0.copy()
        candidate[f"{prefix}.0.bias"] = first_bias.copy()
        candidate[f"{prefix}.2.weight"] = second_weight.copy()
        candidate[f"{prefix}.2.bias"] = second_bias.copy()
    candidate_output_weight = np.ascontiguousarray(
        np.asarray(source_state["pi.1.weight"]), dtype=np.float32
    ).copy()
    candidate_output_bias = np.ascontiguousarray(
        np.asarray(source_state["pi.1.bias"]), dtype=np.float32
    ).copy()
    candidate_output_weight[:2] = output_weight
    candidate_output_bias[:2] = output_bias
    candidate["pi.1.weight"] = candidate_output_weight
    candidate["pi.1.bias"] = candidate_output_bias
    return candidate, {
        "normalization_folded_into_first_layer": True,
        "runtime_normalization_wrapper": False,
        "normalized_clock_columns_bit_zero": True,
        "folded_clock_columns_bit_zero": True,
    }


def fold_equivalence_audit(
    normalized_predictions: Any, runtime_predictions: Any
) -> dict[str, Any]:
    normalized = np.ascontiguousarray(normalized_predictions, dtype=np.float32)
    runtime = np.ascontiguousarray(runtime_predictions, dtype=np.float32)
    if normalized.shape != runtime.shape or normalized.ndim != 2:
        raise V11WeightedFitError("fold-equivalence prediction shapes differ")
    error = np.abs(normalized.astype(np.float64) - runtime.astype(np.float64))
    maximum = float(np.max(error, initial=0.0))
    finite = bool(np.all(np.isfinite(error)) and math.isfinite(maximum))
    passed = bool(finite and maximum <= FOLD_EQUIVALENCE_MAX_ABS)
    result = {
        "fold_equivalence_passed": passed,
        "normalized_runtime_max_abs_difference": maximum,
        "max_abs_tolerance": FOLD_EQUIVALENCE_MAX_ABS,
        "normalized_runtime_all_finite": finite,
    }
    if not passed:
        raise V11WeightedFitError(f"normalization fold equivalence failed: {result}")
    return result


def full_mean_update_audit(
    source_state: Mapping[str, Any], candidate_state: Mapping[str, Any]
) -> dict[str, Any]:
    source_keys = set(source_state)
    candidate_keys = set(candidate_state)
    keys_exact = source_keys == candidate_keys
    aliases_exact = keys_exact and all(
        _bytes_equal(candidate_state[left], candidate_state[right])
        for left, right in _ALIASES
    )
    clocks_zero = keys_exact and all(
        _columns_bit_zero(candidate_state[key], CLOCK_COLUMNS)
        for key in ("pi_encoder.0.weight", "pi.0.0.weight")
    )
    hidden_changed = keys_exact and any(
        not _bytes_equal(source_state[key], candidate_state[key])
        for key in _MEAN_HIDDEN_KEYS
    )
    output_mean_changed = keys_exact and (
        not _bytes_equal(
            np.asarray(source_state["pi.1.weight"])[:2],
            np.asarray(candidate_state["pi.1.weight"])[:2],
        )
        or not _bytes_equal(
            np.asarray(source_state["pi.1.bias"])[:2],
            np.asarray(candidate_state["pi.1.bias"])[:2],
        )
    )
    logstd_exact = keys_exact and _bytes_equal(
        np.asarray(source_state["pi.1.weight"])[2:],
        np.asarray(candidate_state["pi.1.weight"])[2:],
    ) and _bytes_equal(
        np.asarray(source_state["pi.1.bias"])[2:],
        np.asarray(candidate_state["pi.1.bias"])[2:],
    )
    actor_keys = set(_MEAN_HIDDEN_KEYS) | {"pi.1.weight", "pi.1.bias"}
    nonactor_keys = sorted(source_keys - actor_keys) if keys_exact else []
    nonactor_exact = keys_exact and all(
        _bytes_equal(source_state[key], candidate_state[key]) for key in nonactor_keys
    )
    critic_parameter_count = sum(
        int(np.asarray(source_state[key]).size) for key in nonactor_keys
    )
    critic_present = critic_parameter_count > 0
    all_finite = keys_exact and all(
        np.all(np.isfinite(np.asarray(candidate_state[key])))
        for key in candidate_keys
        if hasattr(candidate_state[key], "shape")
    )
    passed = bool(
        keys_exact
        and aliases_exact
        and clocks_zero
        and hidden_changed
        and output_mean_changed
        and logstd_exact
        and nonactor_exact
        and all_finite
    )
    return {
        "passed": passed,
        "state_key_set_bit_exact": keys_exact,
        "changes_confined_to_full_mean_network": passed,
        "hidden_mean_network_changed": bool(hidden_changed),
        "mean_output_changed": bool(output_mean_changed),
        "logstd_parameter_rows_bit_exact": bool(logstd_exact),
        "critic_byte_exact": bool(nonactor_exact),
        "critic_present": critic_present,
        "critic_parameter_count": critic_parameter_count,
        "source_checkpoint_scope": (
            "actor_only_rl_module" if not critic_present else "actor_critic_rl_module"
        ),
        "nonactor_byte_exact": bool(nonactor_exact),
        "nonactor_keys": nonactor_keys,
        "encoder_aliases_bit_exact": bool(aliases_exact),
        "disabled_clock_columns_bit_zero": bool(clocks_zero),
        "all_candidate_tensors_finite": bool(all_finite),
    }


def _milestone(
    history: list[dict[str, Any]], *, stage: str, index: int, loss: Any, lr: float | None
) -> None:
    value = float(loss.detach().cpu().item() if hasattr(loss, "detach") else loss)
    if not math.isfinite(value):
        raise V11WeightedFitError(f"non-finite {stage} loss at {index}")
    row: dict[str, Any] = {"optimizer": stage, "index": int(index), "loss": value}
    if lr is not None:
        row["learning_rate"] = float(lr)
    history.append(row)


def adamw_learning_rate(epoch: int) -> float:
    """Return the fixed V11 rate without the exploratory off-by-one bug."""

    if type(epoch) is not int or epoch < 1 or epoch > 3000:
        raise V11WeightedFitError(f"AdamW epoch is outside 1..3000: {epoch!r}")
    if epoch <= 1500:
        return 3.0e-4
    if epoch <= 2500:
        return 1.0e-4
    return 3.0e-5


def fit_weighted_full_mean_in_memory(
    *,
    source_state: Mapping[str, Any],
    observations: Any,
    targets: Any,
    reset_mask: Any,
    normalization: FrozenNormalization,
) -> InMemoryFitResult:
    """Run the sole V11 AdamW+LBFGS design and return an unsaved actor state."""

    import torch

    validate_source_h0_state(source_state)
    raw = np.ascontiguousarray(observations, dtype=np.float32)
    labels = np.ascontiguousarray(targets, dtype=np.float32)
    if raw.ndim != 2 or raw.shape[1] != 35 or labels.shape != (len(raw), 2):
        raise V11WeightedFitError("fit corpus must contain Nx35 observations and Nx2 labels")
    if not np.all(np.isfinite(labels)):
        raise V11WeightedFitError("fit labels contain non-finite values")
    normalized = normalized_observations(raw, normalization)
    weights_np = reset_sample_weights(reset_mask, rows=len(raw))

    previous_threads = torch.get_num_threads()
    previous_deterministic = torch.are_deterministic_algorithms_enabled()
    torch.set_num_threads(DETERMINISTIC_TORCH_THREADS)
    torch.use_deterministic_algorithms(True)
    try:
        torch.manual_seed(20260807)
        model = _new_normalized_model(source_state, normalization)
        x = torch.as_tensor(normalized, dtype=torch.float32)
        y = torch.as_tensor(labels, dtype=torch.float32)
        weights = torch.as_tensor(weights_np, dtype=torch.float32)
        history: list[dict[str, Any]] = []

        optimizer = torch.optim.AdamW(
            model.parameters(), lr=3.0e-4, weight_decay=1.0e-7
        )
        schedule = ((1, 3.0e-4), (1501, 1.0e-4), (2501, 3.0e-5))
        adam_milestones = {1, 250, 500, 1000, 1500, 2000, 2500, 3000}
        for epoch in range(1, 3001):
            learning_rate = adamw_learning_rate(epoch)
            for group in optimizer.param_groups:
                group["lr"] = learning_rate
            optimizer.zero_grad(set_to_none=True)
            prediction = model(x)
            row_loss = torch.mean(torch.square(prediction - y), dim=1)
            loss = torch.sum(weights * row_loss) / torch.sum(weights)
            if not torch.isfinite(loss):
                raise V11WeightedFitError(f"non-finite AdamW loss at epoch {epoch}")
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), 10.0)
            optimizer.step()
            if epoch in adam_milestones:
                _milestone(
                    history,
                    stage="adamw",
                    index=epoch,
                    loss=loss,
                    lr=learning_rate,
                )

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
            closure_loss = torch.sum(weights * row_loss) / torch.sum(weights)
            if not torch.isfinite(closure_loss):
                raise V11WeightedFitError(
                    f"non-finite LBFGS loss at closure {closure_calls + 1}"
                )
            closure_loss.backward()
            closure_calls += 1
            last_lbfgs_loss = closure_loss
            if closure_calls in {1, 50, 100, 200, 300, 400, 600}:
                _milestone(
                    history,
                    stage="lbfgs_closure",
                    index=closure_calls,
                    loss=closure_loss,
                    lr=0.7,
                )
            return closure_loss

        lbfgs.step(closure)
        if last_lbfgs_loss is None:
            raise V11WeightedFitError("LBFGS did not evaluate the weighted objective")
        _milestone(
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
        runtime_logits = _state_logits(candidate_state, raw)
        runtime_prediction = runtime_logits[:, :2]
        normalization_audit = {
            **fold_audit,
            **fold_equivalence_audit(normalized_prediction, runtime_prediction),
            "normalization": normalization.record(),
        }
        preservation = full_mean_update_audit(source_state, candidate_state)
        source_logits = _state_logits(source_state, raw)
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
            raise V11WeightedFitError("mean-only preservation audit failed")
        metrics = prediction_metrics(runtime_prediction, labels, reset_mask)
        optimizer_audit = {
            "fit_contract_id": FIT_CONTRACT_ID,
            "seed": 20260807,
            "full_batch": True,
            "sample_count": len(raw),
            "reset_count": int(np.count_nonzero(reset_mask)),
            "reset_weight": 100.0,
            "nonreset_weight": 1.0,
            "adamw_epochs": 3000,
            "adamw_schedule": [
                {"start_epoch": start, "learning_rate": rate}
                for start, rate in schedule
            ],
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
    finally:
        torch.use_deterministic_algorithms(previous_deterministic)
        torch.set_num_threads(previous_threads)


def _source_h0_path() -> Path:
    return _resolve(contract.SOURCE_H0_MODULE_PATH)


def _fit_spec() -> dict[str, Any]:
    fit = copy.deepcopy(dict(contract.FIT))
    # This duplicate literal is intentional: changing only the pure contract
    # cannot silently change the numerical implementation.
    expected = {
        "fit_contract_id": FIT_CONTRACT_ID,
        "actor_architecture": {
            "kind": "standard_mean_mlp",
            "input_dim": 35,
            "hidden_dims": [256, 256],
            "output_dim": 2,
            "activation": "tanh",
            "residual_actor": False,
        },
        "normalization": {
            "scope": "base_corpus_3000_rows_only",
            "feature_count": 35,
            "estimator": "population_mean_and_std_float64",
            "std_floor": 1.0e-4,
            "frozen_across_fit_stages": True,
            "fold_into_first_layer_before_save": True,
            "runtime_normalization_wrapper": False,
            "prescribed_clock": False,
        },
        "sample_weighting": {
            "default_weight": 1.0,
            "reset_weight": 100.0,
            "reset_definition": "case_offset_eq_zero",
            "reduction": "sum_weighted_squared_error_div_sum_weights",
        },
        "adamw": {
            "optimizer": "AdamW",
            "seed": 20260807,
            "full_batch": True,
            "epochs": 3000,
            "learning_rate_schedule": [
                {
                    "start_epoch": 1,
                    "end_epoch": 1500,
                    "learning_rate": 3.0e-4,
                },
                {
                    "start_epoch": 1501,
                    "end_epoch": 2500,
                    "learning_rate": 1.0e-4,
                },
                {
                    "start_epoch": 2501,
                    "end_epoch": 3000,
                    "learning_rate": 3.0e-5,
                },
            ],
            "weight_decay": 1.0e-7,
            "grad_clip_norm": 10.0,
        },
        "lbfgs": {
            "optimizer": "LBFGS",
            "deterministic": True,
            "lr": 0.7,
            "max_iter": 300,
            "max_eval": 600,
            "tolerance_grad": 1.0e-10,
            "tolerance_change": 1.0e-12,
            "history_size": 50,
            "line_search_fn": "strong_wolfe",
        },
        "optimizer_phase_order": ["adamw", "lbfgs"],
        "trainable_scope": "full_mean_network",
        "freeze_logstd_head": True,
        "disabled_clock_columns": [0, 1],
        "disabled_clock_policy": "bit_zero_before_and_after_save_reload",
        "anchor_enabled": False,
        "hard_polish_enabled": False,
    }
    if fit != expected:
        raise V11WeightedFitError("V11 sole numerical design drifted")
    return fit


def _load_source_module_and_state() -> tuple[Any, dict[str, Any]]:
    from ray.rllib.core.rl_module.rl_module import RLModule

    source = _source_h0_path()
    module = RLModule.from_checkpoint(source)
    state = _clone_state(module.get_state())
    validate_source_h0_state(state)
    return module, state


def _save_candidate_exact(
    *, source_module: Any, candidate_state: Mapping[str, Any], destination: Path
) -> dict[str, Any]:
    from ray.rllib.core.rl_module.rl_module import RLModule

    if os.path.lexists(destination):
        raise V11WeightedFitError(f"candidate destination occupied/no-clobber: {destination}")
    source_module.set_state(candidate_state)
    source_module.save_to_path(destination)
    reloaded = RLModule.from_checkpoint(destination)
    reloaded_state = _clone_state(reloaded.get_state())
    exact = set(reloaded_state) == set(candidate_state) and all(
        _bytes_equal(candidate_state[key], reloaded_state[key]) for key in candidate_state
    )
    audit = full_mean_update_audit(
        _clone_state(RLModule.from_checkpoint(_source_h0_path()).get_state()),
        reloaded_state,
    )
    if not exact or not audit["passed"]:
        raise V11WeightedFitError("candidate save/reload preservation failed")
    return {
        "exact": True,
        "state_key_count": len(reloaded_state),
        "clock_columns_bit_zero": audit["disabled_clock_columns_bit_zero"],
        "logstd_parameter_rows_bit_exact": audit["logstd_parameter_rows_bit_exact"],
        "critic_byte_exact": audit["critic_byte_exact"],
        "nonactor_byte_exact": audit["nonactor_byte_exact"],
    }


def _design_audit_receipt_path() -> Path:
    return _resolve(contract.DESIGN_AUDIT_RECEIPT_PATH)


def current_design_audit_bindings() -> dict[str, Any]:
    """Recompute every hash whose drift invalidates audit reuse."""

    return {
        "source_bindings": {
            name: _record(relative)
            for name, relative in contract.DESIGN_AUDIT_SOURCE_RELATIVE_PATHS.items()
        },
        "source_h0": _tree_record(_source_h0_path()),
        "corpus": _record(contract.V10S_P0_CORPUS_PATH),
    }


def _metric_match(
    observed: Mapping[str, Any], expected: Mapping[str, Any]
) -> dict[str, Any]:
    tolerance = dict(contract.DESIGN_AUDIT_DETERMINISTIC_TOLERANCE)
    absolute = float(tolerance["absolute"])
    relative = float(tolerance["relative"])
    checks: dict[str, bool] = {}
    differences: dict[str, float] = {}
    for name in ("rmse", "max_abs_error", "reset_max_abs_error"):
        left = float(observed.get(name, math.nan))
        right = float(expected.get(name, math.nan))
        checks[name] = bool(
            math.isfinite(left)
            and math.isfinite(right)
            and math.isclose(left, right, rel_tol=relative, abs_tol=absolute)
        )
        differences[name] = abs(left - right) if math.isfinite(left - right) else math.inf
    return {
        "passed": all(checks.values()),
        "checks": checks,
        "absolute_differences": differences,
        "tolerance": tolerance,
    }


def _validate_design_audit_for_p0(
    *, corpus: FitCorpus, source_record: Mapping[str, Any], metrics: Mapping[str, Any]
) -> dict[str, Any]:
    receipt = _mapping(_design_audit_receipt_path())
    gate = dict(contract.design_audit_gate(receipt))
    current_binding_gate = dict(
        contract.design_audit_current_binding_gate(
            receipt, current_design_audit_bindings()
        )
    )
    corpus_matches = (
        receipt.get("corpus", {}).get("observations_sha256")
        == array_sha256(corpus.observations)
        and receipt.get("corpus", {}).get("actions_sha256")
        == array_sha256(corpus.actions)
        and receipt.get("corpus", {}).get("reset_mask_sha256")
        == array_sha256(corpus.reset_mask)
    )
    source_matches = receipt.get("source_h0") == _tree_record(_source_h0_path())
    metric_match = _metric_match(
        metrics, receipt.get("p0_reproduction_reference_metrics", {})
    )
    passed = bool(
        gate.get("passed") is True
        and current_binding_gate.get("passed") is True
        and corpus_matches
        and source_matches
        and metric_match["passed"]
    )
    return {
        "passed": passed,
        "receipt": _record(_design_audit_receipt_path()),
        "gate": gate,
        "current_binding_gate": current_binding_gate,
        "source_matches": source_matches,
        "corpus_matches": corpus_matches,
        "metric_match": metric_match,
    }


def run_design_audit_in_memory() -> dict[str, Any]:
    """Execute the formal P0 audit without creating a candidate checkpoint."""

    fit = _fit_spec()
    base = load_frozen_v8_corpus()
    if len(base.observations) != BASE_NORMALIZATION_ROWS:
        raise V11WeightedFitError("design audit P0 corpus is not exactly 3000 rows")
    normalization = frozen_base_normalization(base.observations)
    source_module, source_state = _load_source_module_and_state()
    del source_module  # Explicitly no save path exists in the audit operation.
    source_before = _tree_record(_source_h0_path())
    result = fit_weighted_full_mean_in_memory(
        source_state=source_state,
        observations=base.observations,
        targets=base.actions,
        reset_mask=base.reset_mask,
        normalization=normalization,
    )
    if source_before != _tree_record(_source_h0_path()):
        raise V11WeightedFitError("frozen source H0 changed during design audit")
    # This is the only authorized feasibility execution of the already-fixed
    # design.  Its observed triplet is accepted only against the unchanged
    # V10S gates and becomes the later formal-P0 deterministic reference.
    reproduction_reference = dict(result.metrics)
    reference_identity = _metric_match(result.metrics, reproduction_reference)
    thresholds = dict(contract.OFFLINE_THRESHOLDS)
    below_gates = {
        "rmse": result.metrics["rmse"] <= thresholds["rmse_max"],
        "max_abs_error": (
            result.metrics["max_abs_error"] <= thresholds["max_abs_error_max"]
        ),
        "reset_max_abs_error": (
            result.metrics["reset_max_abs_error"]
            <= thresholds["reset_max_abs_error_max"]
        ),
    }
    if not all(below_gates.values()):
        raise V11WeightedFitError(f"V11 design audit failed offline gates: {below_gates}")
    receipt_path = _design_audit_receipt_path()
    v10s_corpus_path = _resolve(contract.V10S_P0_CORPUS_PATH)
    if not v10s_corpus_path.is_file():
        raise V11WeightedFitError("frozen V10S P0 corpus artifact is missing")
    with np.load(v10s_corpus_path, allow_pickle=False) as frozen:
        frozen_arrays_match = (
            _bytes_equal(frozen["observations"], base.observations)
            and _bytes_equal(frozen["actions"], base.actions)
            and _bytes_equal(frozen["reset_mask"], base.reset_mask)
        )
    if not frozen_arrays_match:
        raise V11WeightedFitError("revalidated base corpus differs from frozen V10S P0")
    corpus_artifact = _record(v10s_corpus_path)
    current_bindings = current_design_audit_bindings()
    preservation = {
        "source_h0_byte_exact": source_before == _tree_record(_source_h0_path()),
        "critic_byte_exact": result.preservation_audit["critic_byte_exact"],
        "source_checkpoint_scope": result.preservation_audit[
            "source_checkpoint_scope"
        ],
        "critic_present": result.preservation_audit["critic_present"],
        "critic_parameter_count": result.preservation_audit[
            "critic_parameter_count"
        ],
        "logstd_byte_exact": bool(
            result.preservation_audit["logstd_parameter_rows_bit_exact"]
            and result.preservation_audit["logstd_outputs_bit_exact"]
        ),
        "disabled_clock_columns_0_1_bit_zero": result.preservation_audit[
            "disabled_clock_columns_bit_zero"
        ],
        "normalization_folded_into_first_layer": result.normalization_audit[
            "normalization_folded_into_first_layer"
        ],
        "fold_equivalence_passed": result.normalization_audit[
            "fold_equivalence_passed"
        ],
        "no_runtime_normalization_wrapper": True,
        "no_prescribed_clock": True,
    }
    payload = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.DESIGN_AUDIT_PASS_STATUS,
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "contract_id": FIT_CONTRACT_ID,
        "design_audit_id": DESIGN_AUDIT_ID,
        # The gate is recomputed below.  This seed flag lets the pure gate
        # validate the receipt topology without allowing the receipt to define
        # any metric target.
        "gate": {"passed": True},
        "dry_run": True,
        "no_candidate_checkpoint": True,
        "fit_design": fit,
        "gates": thresholds,
        "observed_metrics": dict(result.metrics),
        "p0_reproduction_reference_metrics": reproduction_reference,
        "deterministic_tolerance": dict(
            contract.DESIGN_AUDIT_DETERMINISTIC_TOLERANCE
        ),
        "metric_gate_checks": below_gates,
        "reproduction_reference_identity": reference_identity,
        "source_h0": current_bindings["source_h0"],
        "source_bindings": current_bindings["source_bindings"],
        "corpus": {
            "artifact": corpus_artifact,
            "rows": len(base.observations),
            "observation_dim": 35,
            "action_dim": 2,
            "reset_rows": int(np.count_nonzero(base.reset_mask)),
            "observations_sha256": array_sha256(base.observations),
            "actions_sha256": array_sha256(base.actions),
            "reset_mask_sha256": array_sha256(base.reset_mask),
            "source_records": dict(base.source_records),
            "validator": _record(v10s_fit.__file__),
        },
        "normalization": normalization.record(),
        "normalization_audit": dict(result.normalization_audit),
        "optimizer_audit": dict(result.optimizer_audit),
        "preservation_audit": preservation,
        "history": [dict(row) for row in result.history],
        "artifacts_written": [
            receipt_path.relative_to(REPO_ROOT).as_posix()
        ],
        "candidate_checkpoint_paths": [],
        "actor_fit_executions": 1,
        "actor_updates": 1,
        "candidate_checkpoints_persisted": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
        "retry_authorized": False,
        "sweep_authorized": False,
    }
    gate = dict(contract.design_audit_gate(payload))
    payload["gate"] = gate
    if gate.get("passed") is not True:
        failed = [name for name, value in gate.get("checks", {}).items() if value is not True]
        raise V11WeightedFitError(f"design-audit receipt contract failed: {failed}")
    if contract.design_audit_gate(payload).get("passed") is not True:
        raise V11WeightedFitError("design-audit receipt is not self-consistent")
    return payload


def _canonical_fit_destination(
    stage: str, output_dir: str | Path, *, enforce: bool
) -> Path:
    destination = _resolve(output_dir)
    expected = _resolve(contract.FIT_ROOTS[stage])
    if enforce and destination != expected:
        raise V11WeightedFitError(f"non-canonical {stage} destination: {destination}")
    if os.path.lexists(destination):
        raise V11WeightedFitError(f"fit stage already exists/no-clobber: {destination}")
    if enforce and (destination / "rl_module_target_adapted").resolve() != _resolve(
        contract.MODULE_PATHS[stage]
    ):
        raise V11WeightedFitError("contract module path drifted")
    return destination


def _validate_claim(
    path: str | Path, *, stage: str, worker: bool, enforce_canonical: bool
) -> dict[str, Any]:
    try:
        with _v11_bound_v10s_validator():
            return v10s_fit._validate_claim(
                path,
                stage=stage,
                worker=worker,
                enforce_canonical=enforce_canonical,
            )
    except Exception as exc:
        raise V11WeightedFitError("pipeline/worker claim validation failed") from exc


def run_fit_stage(
    *,
    stage: str,
    output_dir: str | Path,
    dagger_receipt_paths: Sequence[str | Path] = (),
    pipeline_claim_path: str | Path,
    worker_claim_path: str | Path,
    enforce_canonical_destination: bool = True,
) -> dict[str, Any]:
    """Fit one V11 stage from fresh H0 and publish a no-clobber receipt."""

    fit = _fit_spec()
    pipeline_claim = _validate_claim(
        pipeline_claim_path,
        stage=stage,
        worker=False,
        enforce_canonical=enforce_canonical_destination,
    )
    worker_claim = _validate_claim(
        worker_claim_path,
        stage=stage,
        worker=True,
        enforce_canonical=enforce_canonical_destination,
    )
    if enforce_canonical_destination and (
        worker_claim.get("execution_token_sha256")
        != pipeline_claim.get("execution_token_sha256")
    ):
        raise V11WeightedFitError("pipeline/worker execution token drifted")
    corpus = load_fit_corpus(dagger_receipt_paths, stage=stage)
    base = load_frozen_v8_corpus()
    if not _bytes_equal(
        corpus.observations[:BASE_NORMALIZATION_ROWS], base.observations
    ):
        raise V11WeightedFitError("cumulative corpus base rows drifted")
    normalization = frozen_base_normalization(base.observations)
    destination = _canonical_fit_destination(
        stage, output_dir, enforce=enforce_canonical_destination
    )
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.mkdir(exist_ok=False)
    corpus_path = v10s_fit._write_npz_exclusive(
        destination / "corpus.npz", corpus.arrays()
    )

    source_module, source_state = _load_source_module_and_state()
    source_before = _tree_record(_source_h0_path())
    result = fit_weighted_full_mean_in_memory(
        source_state=source_state,
        observations=corpus.observations,
        targets=corpus.actions,
        reset_mask=corpus.reset_mask,
        normalization=normalization,
    )
    module_path = destination / "rl_module_target_adapted"
    save_reload = _save_candidate_exact(
        source_module=source_module,
        candidate_state=result.candidate_state,
        destination=module_path,
    )
    if source_before != _tree_record(_source_h0_path()):
        raise V11WeightedFitError("frozen source H0 changed during fit")

    report = {
        "fit_contract_id": FIT_CONTRACT_ID,
        "source_checkpoint": _source_h0_path().relative_to(REPO_ROOT).as_posix(),
        "fit_stage": stage,
        "training_samples": len(corpus.observations),
        "validation_samples": 0,
        "fit_design": fit,
        "normalization": normalization.record(),
        "optimizer_audit": dict(result.optimizer_audit),
        "preservation_audit": dict(result.preservation_audit),
        "save_reload": save_reload,
        "hard_polish": False,
        "fallback": False,
        "sweep": False,
    }
    history_path = forensic.write_json_exclusive(
        destination / "adaptation_history.json",
        [dict(row) for row in result.history],
    )
    report_path = forensic.write_json_exclusive(
        destination / "adaptation_report.json", report
    )

    expected_counts = contract.expected_fit_counts(stage)
    identities = list(
        zip(
            corpus.tranche_ids.astype(str).tolist(),
            corpus.case_ids.astype(str).tolist(),
            corpus.step_indices.tolist(),
        )
    )
    design_binding = (
        _validate_design_audit_for_p0(
            corpus=corpus,
            source_record=source_before,
            metrics=result.metrics,
        )
        if stage == "p0"
        else {
            "passed": contract.design_audit_gate(
                _mapping(_design_audit_receipt_path())
            ).get("passed")
            is True,
            "receipt": _record(_design_audit_receipt_path()),
            "scope": "P0_REPRODUCTION_REFERENCE;_P1_P3_DESIGN_ONLY",
        }
    )
    terminal_gate = dict(
        contract.v10s_terminal_failure_gate(
            _mapping(contract.V10S_TERMINAL_LEDGER_PATH),
            _mapping(contract.V10S_P0_GATE_PATH),
            _mapping(contract.V10S_P0_SUMMARY_PATH),
        )
    )
    summary = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": "H0_PRIMARY_SPLIT_V11_WEIGHTED_FULL_MEAN_FIT_COMPLETE_UNGATED",
        "protocol_id": contract.PROTOCOL_ID,
        "fit_contract_id": FIT_CONTRACT_ID,
        "fit_stage": stage,
        "fit": fit,
        "actor_architecture": copy.deepcopy(contract.ACTOR_ARCHITECTURE),
        "normalization": copy.deepcopy(contract.BASE_CORPUS_NORMALIZATION),
        "sample_weighting": copy.deepcopy(contract.SAMPLE_WEIGHTING),
        "source_h0_id": contract.SOURCE_H0_ID,
        "initial_checkpoint_id": contract.SOURCE_H0_ID,
        "continued_from_previous_candidate": False,
        "trainable_scope": contract.TRAINABLE_SCOPE,
        "event_contract_id": contract.EVENT_CONTRACT_ID,
        "teacher_evidence_id": corpus.audit["teacher_evidence_id"],
        "teacher_evidence_passed": corpus.audit["teacher_evidence_passed"],
        "base_corpus_case_ids": list(contract.FINAL_CASE_IDS),
        "completed_collection_rounds": list(
            expected_counts["completed_collection_rounds"]
        ),
        "sample_count": len(corpus.observations),
        "reset_row_count": int(np.count_nonzero(corpus.reset_mask)),
        "base_sample_count": int(corpus.audit["base_sample_count"]),
        "dagger_sample_count": int(corpus.audit["dagger_sample_count"]),
        "dagger_receipt_count": len(dagger_receipt_paths),
        "duplicate_sample_count": len(identities) - len(set(identities)),
        "metrics": dict(result.metrics),
        "all_finite": bool(
            all(math.isfinite(float(value)) for value in result.metrics.values())
        ),
        "source_h0_byte_exact": source_before == _tree_record(_source_h0_path()),
        "critic_byte_exact": result.preservation_audit["critic_byte_exact"],
        "source_checkpoint_scope": result.preservation_audit[
            "source_checkpoint_scope"
        ],
        "critic_present": result.preservation_audit["critic_present"],
        "critic_parameter_count": result.preservation_audit[
            "critic_parameter_count"
        ],
        "logstd_byte_exact": bool(
            result.preservation_audit["logstd_parameter_rows_bit_exact"]
            and result.preservation_audit["logstd_outputs_bit_exact"]
        ),
        "v10s_terminal_failure_id": contract.V10S_TERMINAL_FAILURE_ID,
        "v10s_terminal_failure_passed": terminal_gate.get("passed") is True,
        "v10s_terminal_gate": terminal_gate,
        "v10s_terminal_records": {
            "ledger": _record(contract.V10S_TERMINAL_LEDGER_PATH),
            "p0_gate": _record(contract.V10S_P0_GATE_PATH),
            "p0_summary": _record(contract.V10S_P0_SUMMARY_PATH),
        },
        "design_audit_id": contract.DESIGN_AUDIT_ID,
        "design_audit_passed": design_binding["passed"],
        "design_audit_receipt": _record(_design_audit_receipt_path()),
        "design_audit_reproduction_within_tolerance": (
            design_binding.get("metric_match", {}).get("passed") is True
            if stage == "p0"
            else True
        ),
        "adamw_epochs_run": 3000,
        "lbfgs_completed": True,
        "deterministic_algorithms_enabled": result.optimizer_audit[
            "deterministic_algorithms_enabled"
        ],
        "disabled_clock_column_indices": list(CLOCK_COLUMNS),
        "disabled_clock_columns_bit_zero_after_save_reload": save_reload[
            "clock_columns_bit_zero"
        ],
        "normalization_stats_from_base_corpus_only": True,
        "normalization_stats_frozen_across_stages": True,
        "normalization_folded_into_first_layer": True,
        "fold_equivalence_passed": result.normalization_audit[
            "fold_equivalence_passed"
        ],
        "runtime_normalization_wrapper_present": False,
        "prescribed_clock_present": False,
        "anchor_used": False,
        "hard_polish_used": False,
        "disabled_clock_columns_bit_zero": result.preservation_audit[
            "disabled_clock_columns_bit_zero"
        ],
        "save_reload_exact": save_reload["exact"],
        "design_audit_binding": design_binding,
        "report_checks": {
            "fresh_source_h0": True,
            "fixed_design": True,
            "all_rows_used_full_batch": True,
            "mean_only_update": result.preservation_audit["passed"],
            "normalization_frozen_from_3000_base_rows": True,
            "normalization_folded_before_save": True,
            "fold_equivalence_passed": result.normalization_audit[
                "fold_equivalence_passed"
            ],
            "runtime_wrapper_absent": True,
            "clock_columns_bit_zero": result.preservation_audit[
                "disabled_clock_columns_bit_zero"
            ],
            "logstd_parameters_exact": result.preservation_audit[
                "logstd_parameter_rows_bit_exact"
            ],
            "logstd_outputs_exact": result.preservation_audit[
                "logstd_outputs_bit_exact"
            ],
            "critic_nonactor_exact": result.preservation_audit[
                "nonactor_byte_exact"
            ],
            "save_reload_exact": save_reload["exact"],
            "design_audit_bound": design_binding["passed"],
            "no_hard_polish_fallback_sweep": True,
            "failed_v9_rows_excluded": corpus.audit["failed_v9_rows_used"] == 0,
            "dagger_labels_same_state": corpus.audit[
                "same_state_dagger_sample_count"
            ]
            == corpus.audit["dagger_sample_count"],
        },
        "corpus_audit": dict(corpus.audit),
        "corpus_observations_sha256": array_sha256(corpus.observations),
        "corpus_actions_sha256": array_sha256(corpus.actions),
        "corpus_reset_mask_sha256": array_sha256(corpus.reset_mask),
        "normalization_statistics": normalization.record(),
        "normalization_audit": dict(result.normalization_audit),
        "optimizer_audit": dict(result.optimizer_audit),
        "full_mean_update_audit": dict(result.preservation_audit),
        "corpus": _record(corpus_path),
        "corpus_sources": dict(corpus.source_records),
        "source_h0": source_before,
        "module": _tree_record(module_path),
        "adaptation_report": _record(report_path),
        "adaptation_history": _record(history_path),
        "pipeline_claim": _record(pipeline_claim_path),
        "worker_claim": _record(worker_claim_path),
        "actor_updates": 1,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
    }
    summary_path = forensic.write_json_exclusive(destination / "summary.json", summary)
    gate = dict(contract.fit_gate(summary, stage=stage))
    gate_path = forensic.write_json_exclusive(destination / "gate.json", gate)
    receipt = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": gate["status"],
        "passed": gate["passed"],
        "protocol_id": contract.PROTOCOL_ID,
        "stage_id": f"fit_{stage}",
        "fit_stage": stage,
        "candidate_created": True,
        "candidate_scope": "FIT_ONLY_UNFROZEN",
        "retry_authorized": False,
        "fit": fit,
        "corpus": _record(corpus_path),
        "summary": _record(summary_path),
        "gate": _record(gate_path),
        "adaptation_report": _record(report_path),
        "adaptation_history": _record(history_path),
        "module": _tree_record(module_path),
        "source_h0": source_before,
        "design_audit": _record(_design_audit_receipt_path()),
        "pipeline_claim": _record(pipeline_claim_path),
        "worker_claim": _record(worker_claim_path),
        "dagger_receipts": [_record(item) for item in dagger_receipt_paths],
        "actor_updates": 1,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
    }
    receipt_path = forensic.write_json_exclusive(destination / "receipt.json", receipt)
    receipt["receipt"] = _record(receipt_path)
    return receipt


__all__ = [
    "DESIGN_AUDIT_ID",
    "FIT_CONTRACT_ID",
    "FitCorpus",
    "FrozenNormalization",
    "InMemoryFitResult",
    "V10SFitError",
    "V11WeightedFitError",
    "array_sha256",
    "adamw_learning_rate",
    "current_design_audit_bindings",
    "fit_weighted_full_mean_in_memory",
    "fold_equivalence_audit",
    "frozen_base_normalization",
    "full_mean_update_audit",
    "load_dagger_tranche",
    "load_fit_corpus",
    "load_frozen_v8_corpus",
    "normalized_observations",
    "prediction_metrics",
    "reset_sample_weights",
    "run_design_audit_in_memory",
    "run_fit_stage",
]
