"""Per-start-condition learner metrics for the baseline PPO pipeline.

RLlib computes Generalized Advantage Estimation (GAE) on the Learner, after
the environment callbacks have finished.  This module keeps the episode start
offset attached to every sampled timestep and logs additive advantage moments
*after* GAE, so downstream code can derive exact per-start means and standard
deviations without averaging already-averaged worker metrics.

All public connector builders and Learner classes are module-level objects.
That is intentional: RLlib/Ray must be able to pickle them by import path when
using spawned workers on Windows and macOS.
"""

from __future__ import annotations

import math
from collections.abc import Mapping, MutableMapping, Sequence
from dataclasses import dataclass
from typing import Any

import numpy as np
import torch
from ray.rllib.algorithms.ppo.ppo import LEARNER_RESULTS_KL_KEY
from ray.rllib.algorithms.ppo.torch.ppo_torch_learner import PPOTorchLearner
from ray.rllib.connectors.connector_v2 import ConnectorV2
from ray.rllib.core import DEFAULT_MODULE_ID
from ray.rllib.core.columns import Columns
from ray.rllib.evaluation.postprocessing import Postprocessing
from ray.rllib.utils.annotations import override


EPISODE_START_OFFSET_COLUMN = "episode_start_offset_s"
START_CONDITION_METRICS_PREFIX = "start_condition"
START_CONDITION_BATCH_METRICS_PREFIX = "start_condition_batch"
KL_UPDATE_METRICS_PREFIX = "kl_update"
KL_UPDATE_MAX_MINIBATCH_MEAN_KEY = "max_minibatch_mean"
KL_UPDATE_MIN_MINIBATCH_MEAN_KEY = "min_minibatch_mean"
KL_UPDATE_MINIBATCH_COUNT_KEY = "minibatch_count"
KL_UPDATE_NONFINITE_COUNT_KEY = "nonfinite_count"
_OFFSET_DECIMAL_PLACES = 6
_OFFSET_CONSISTENCY_ABS_TOL = 1e-9

_ADVANTAGE_MOMENT_KEYS = (
    "advantage_sum",
    "advantage_sumsq",
    "advantage_positive_count",
    "advantage_count",
)


@dataclass(frozen=True)
class BatchCompactionReport:
    """Audit information for one post-GAE module batch compaction."""

    input_rows: int
    real_rows: int
    artificial_rows: int
    advantage_mean_before_normalization: float
    advantage_std_before_normalization: float
    interleaved_rows: int
    interleaved_start_conditions: int
    interleaved_rows_per_start: int
    max_start_run_length: int


def _finite_float(value: Any, *, field: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{field} must be numeric, got {value!r}") from exc
    if not math.isfinite(result):
        raise ValueError(f"{field} must be finite, got {value!r}")
    return result


def format_start_offset_label(offset_s: float) -> str:
    """Return a stable, TensorBoard-safe label for one start offset."""
    value = _finite_float(offset_s, field=EPISODE_START_OFFSET_COLUMN)
    # Avoid a distinct ``m0p000000`` label for negative floating-point zero.
    if round(value, _OFFSET_DECIMAL_PLACES) == 0.0:
        value = 0.0
    text = f"{value:.{_OFFSET_DECIMAL_PLACES}f}"
    text = text.replace("-", "m").replace(".", "p")
    return f"offset_{text}s"


def parse_start_offset_label(label: str) -> float:
    """Recover the numeric offset from :func:`format_start_offset_label`."""
    text = str(label)
    if not text.startswith("offset_") or not text.endswith("s"):
        raise ValueError(f"invalid start-offset label: {label!r}")
    numeric = text[len("offset_") : -1]
    if numeric.startswith("m"):
        numeric = "-" + numeric[1:]
    numeric = numeric.replace("p", ".")
    return _finite_float(numeric, field="start-offset label")


def episode_start_offset_from_infos(
    infos: Mapping[str, Any] | Sequence[Mapping[str, Any]],
) -> float:
    """Extract and consistency-check an episode's start offset from info dicts."""
    if isinstance(infos, Mapping):
        candidates = [infos]
    else:
        candidates = list(infos)

    values: list[float] = []
    for info in candidates:
        if not isinstance(info, Mapping) or EPISODE_START_OFFSET_COLUMN not in info:
            continue
        values.append(
            _finite_float(
                info[EPISODE_START_OFFSET_COLUMN],
                field=EPISODE_START_OFFSET_COLUMN,
            )
        )
    if not values:
        raise ValueError(
            f"episode infos do not contain {EPISODE_START_OFFSET_COLUMN!r}"
        )

    reference = values[0]
    for value in values[1:]:
        if not math.isclose(
            value,
            reference,
            rel_tol=0.0,
            abs_tol=_OFFSET_CONSISTENCY_ABS_TOL,
        ):
            raise ValueError(
                "episode contains inconsistent start offsets: "
                f"{reference!r} and {value!r}"
            )
    return reference


def _flat_numeric_array(value: Any, *, field: str) -> np.ndarray:
    """Convert NumPy/Torch-like input to a flat float64 array."""
    if hasattr(value, "detach"):
        value = value.detach()
    if hasattr(value, "cpu"):
        value = value.cpu()
    try:
        result = np.asarray(value, dtype=np.float64).reshape(-1)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{field} must be a numeric array") from exc
    return result


def _validated_loss_mask(
    loss_mask: Any,
    *,
    expected_rows: int | None = None,
) -> np.ndarray:
    """Return a strict one-value-per-timestep boolean loss mask.

    RLlib's bootstrap connector emits exactly one boolean per batch row.  Do not
    silently accept arbitrary non-zero values or a multi-feature mask: either
    case could remove the wrong samples while still producing a plausible batch.
    """
    if hasattr(loss_mask, "detach"):
        loss_mask = loss_mask.detach()
    if hasattr(loss_mask, "cpu"):
        loss_mask = loss_mask.cpu()
    try:
        raw_mask = np.asarray(loss_mask)
    except (TypeError, ValueError) as exc:
        raise ValueError("loss mask must be a numeric or boolean array") from exc

    if raw_mask.ndim == 0:
        raise ValueError("loss mask must contain one value per timestep")
    if raw_mask.ndim != 1:
        raise ValueError(
            "loss mask must be one-dimensional with one scalar per timestep, "
            "got shape "
            f"{raw_mask.shape!r}"
        )
    raw_mask = raw_mask.reshape(-1)
    if expected_rows is not None and raw_mask.size != expected_rows:
        raise ValueError(
            "loss mask and timestep batch must contain the same number of rows: "
            f"{raw_mask.size} != {expected_rows}"
        )

    if np.issubdtype(raw_mask.dtype, np.bool_):
        return raw_mask.astype(bool, copy=False)
    if not np.issubdtype(raw_mask.dtype, np.number):
        raise ValueError("loss mask values must be boolean, zero, or one")
    try:
        numeric_mask = raw_mask.astype(np.float64, copy=False)
    except (TypeError, ValueError) as exc:
        raise ValueError("loss mask values must be boolean, zero, or one") from exc
    if not np.all(np.isfinite(numeric_mask)) or not np.all(
        (numeric_mask == 0.0) | (numeric_mask == 1.0)
    ):
        raise ValueError("loss mask values must be boolean, zero, or one")
    return numeric_mask.astype(bool, copy=False)


def _rebuild_mapping(original: Mapping[Any, Any], items: list[tuple[Any, Any]]):
    """Rebuild common nested mapping types without mutating the input batch."""
    if type(original) is dict:
        return dict(items)
    try:
        return type(original)(items)
    except (TypeError, ValueError):
        # RLlib consumes the mapping protocol; a plain dict is a safe fallback for
        # custom mapping implementations whose constructors require extra state.
        return dict(items)


def _filter_timestep_tree(
    value: Any,
    *,
    active_rows: np.ndarray,
    input_rows: int,
    field: str,
) -> tuple[Any, bool]:
    """Filter all batch-dimension leaves in a possibly nested column.

    The boolean return value says whether at least one timestep leaf was found.
    Array/tensor leaves are considered timestep data and therefore fail closed
    when their leading dimension does not match the loss mask.
    """
    if torch.is_tensor(value):
        if value.ndim == 0:
            return value, False
        if int(value.shape[0]) != input_rows:
            raise ValueError(
                f"{field} has {int(value.shape[0])} rows, expected {input_rows} "
                "from the loss mask"
            )
        torch_mask = torch.as_tensor(
            active_rows,
            dtype=torch.bool,
            device=value.device,
        )
        return value[torch_mask], True

    if isinstance(value, np.ndarray):
        if value.ndim == 0:
            return value, False
        if int(value.shape[0]) != input_rows:
            raise ValueError(
                f"{field} has {int(value.shape[0])} rows, expected {input_rows} "
                "from the loss mask"
            )
        return value[active_rows], True

    if isinstance(value, Mapping):
        filtered_items: list[tuple[Any, Any]] = []
        found_timestep_leaf = False
        for key, nested_value in value.items():
            filtered, found = _filter_timestep_tree(
                nested_value,
                active_rows=active_rows,
                input_rows=input_rows,
                field=f"{field}.{key}",
            )
            filtered_items.append((key, filtered))
            found_timestep_leaf = found_timestep_leaf or found
        return _rebuild_mapping(value, filtered_items), found_timestep_leaf

    if isinstance(value, list):
        # A plain list of N items is itself a timestep column (for example infos).
        # Short lists containing tensor leaves are nested structures instead.
        if len(value) == input_rows:
            return [item for item, keep in zip(value, active_rows) if keep], True
        filtered_items = []
        found_timestep_leaf = False
        for index, nested_value in enumerate(value):
            filtered, found = _filter_timestep_tree(
                nested_value,
                active_rows=active_rows,
                input_rows=input_rows,
                field=f"{field}[{index}]",
            )
            filtered_items.append(filtered)
            found_timestep_leaf = found_timestep_leaf or found
        return filtered_items, found_timestep_leaf

    if isinstance(value, tuple):
        if len(value) == input_rows:
            return tuple(
                item for item, keep in zip(value, active_rows) if keep
            ), True
        filtered_items = []
        found_timestep_leaf = False
        for index, nested_value in enumerate(value):
            filtered, found = _filter_timestep_tree(
                nested_value,
                active_rows=active_rows,
                input_rows=input_rows,
                field=f"{field}[{index}]",
            )
            filtered_items.append(filtered)
            found_timestep_leaf = found_timestep_leaf or found
        if hasattr(value, "_fields"):
            return type(value)(*filtered_items), found_timestep_leaf
        return tuple(filtered_items), found_timestep_leaf

    # Scalar/string metadata is not indexed by timestep and is left untouched.
    return value, False


def _reorder_timestep_tree(
    value: Any,
    *,
    permutation: np.ndarray,
    input_rows: int,
    field: str,
) -> tuple[Any, bool]:
    """Apply one row permutation to every timestep-aligned nested leaf.

    This deliberately mirrors :func:`_filter_timestep_tree`: array/tensor leaves
    are treated as timestep data and therefore fail closed on a leading-dimension
    mismatch.  Scalar metadata is preserved.  The caller builds an entirely new
    mapping and commits it only after all leaves and post-conditions pass.
    """
    if torch.is_tensor(value):
        if value.ndim == 0:
            return value, False
        if int(value.shape[0]) != input_rows:
            raise ValueError(
                f"{field} has {int(value.shape[0])} rows, expected {input_rows} "
                "for start-condition interleaving"
            )
        torch_indices = torch.as_tensor(
            permutation,
            dtype=torch.long,
            device=value.device,
        )
        return value[torch_indices], True

    if isinstance(value, np.ndarray):
        if value.ndim == 0:
            return value, False
        if int(value.shape[0]) != input_rows:
            raise ValueError(
                f"{field} has {int(value.shape[0])} rows, expected {input_rows} "
                "for start-condition interleaving"
            )
        return value[permutation], True

    if isinstance(value, Mapping):
        reordered_items: list[tuple[Any, Any]] = []
        found_timestep_leaf = False
        for key, nested_value in value.items():
            reordered, found = _reorder_timestep_tree(
                nested_value,
                permutation=permutation,
                input_rows=input_rows,
                field=f"{field}.{key}",
            )
            reordered_items.append((key, reordered))
            found_timestep_leaf = found_timestep_leaf or found
        return _rebuild_mapping(value, reordered_items), found_timestep_leaf

    if isinstance(value, list):
        if len(value) == input_rows:
            return [value[int(index)] for index in permutation], True
        reordered_items = []
        found_timestep_leaf = False
        for index, nested_value in enumerate(value):
            reordered, found = _reorder_timestep_tree(
                nested_value,
                permutation=permutation,
                input_rows=input_rows,
                field=f"{field}[{index}]",
            )
            reordered_items.append(reordered)
            found_timestep_leaf = found_timestep_leaf or found
        return reordered_items, found_timestep_leaf

    if isinstance(value, tuple):
        if len(value) == input_rows:
            return tuple(value[int(index)] for index in permutation), True
        reordered_items = []
        found_timestep_leaf = False
        for index, nested_value in enumerate(value):
            reordered, found = _reorder_timestep_tree(
                nested_value,
                permutation=permutation,
                input_rows=input_rows,
                field=f"{field}[{index}]",
            )
            reordered_items.append(reordered)
            found_timestep_leaf = found_timestep_leaf or found
        if hasattr(value, "_fields"):
            return type(value)(*reordered_items), found_timestep_leaf
        return tuple(reordered_items), found_timestep_leaf

    return value, False


def _maximum_run_length(values: np.ndarray) -> int:
    """Return the longest contiguous run in a non-empty one-dimensional array."""
    if values.ndim != 1 or values.size <= 0:
        raise ValueError("start-condition run-length input must be non-empty and flat")
    boundaries = np.flatnonzero(values[1:] != values[:-1]) + 1
    lengths = np.diff(np.concatenate(([0], boundaries, [values.size])))
    return int(np.max(lengths))


def interleave_equal_start_condition_rows(
    module_batch: MutableMapping[str, Any],
    *,
    expected_start_conditions: int = 3,
) -> tuple[int, int, int, int]:
    """Round-robin exactly equal start groups across all timestep columns.

    Within each start condition, original row order is preserved.  The three
    groups are ordered by their rounded numeric start offset, then interleaved one
    row at a time.  This makes every contiguous PPO minibatch as balanced as
    integer arithmetic permits while keeping GAE complete before rows move.

    Returns ``(rows, start_conditions, rows_per_start, max_run_length)``.  The
    mutation is transactional: no part of ``module_batch`` changes unless every
    nested timestep leaf and every post-condition validates successfully.
    """
    if isinstance(expected_start_conditions, bool) or expected_start_conditions != 3:
        raise ValueError("exact interleaving requires exactly three start conditions")
    if EPISODE_START_OFFSET_COLUMN not in module_batch:
        raise KeyError("post-GAE Learner batch is missing start-offset column")

    offsets = _flat_numeric_array(
        module_batch[EPISODE_START_OFFSET_COLUMN],
        field=EPISODE_START_OFFSET_COLUMN,
    )
    input_rows = int(offsets.size)
    if input_rows <= 0:
        raise ValueError("post-GAE Learner batch must not be empty")
    if not np.all(np.isfinite(offsets)):
        raise ValueError("real episode start offsets must all be finite")

    rounded = np.round(offsets, decimals=_OFFSET_DECIMAL_PLACES)
    unique_offsets = np.unique(rounded)
    if unique_offsets.size != expected_start_conditions:
        raise ValueError(
            "exact interleaving requires exactly three distinct start conditions; "
            f"got {int(unique_offsets.size)}"
        )
    row_groups = [np.flatnonzero(rounded == offset) for offset in unique_offsets]
    counts = np.asarray([indices.size for indices in row_groups], dtype=np.int64)
    if np.any(counts <= 0) or not np.all(counts == counts[0]):
        raise ValueError(
            "exact interleaving requires equal rows for all three start conditions; "
            f"got {counts.tolist()}"
        )
    rows_per_start = int(counts[0])
    permutation = np.stack(row_groups, axis=1).reshape(-1)
    if (
        permutation.size != input_rows
        or np.unique(permutation).size != input_rows
        or int(np.min(permutation)) != 0
        or int(np.max(permutation)) != input_rows - 1
    ):
        raise RuntimeError("start-condition interleaving is not a row permutation")

    reordered: dict[str, Any] = {}
    found_by_column: dict[str, bool] = {}
    for column, value in module_batch.items():
        reordered_value, found = _reorder_timestep_tree(
            value,
            permutation=permutation,
            input_rows=input_rows,
            field=str(column),
        )
        reordered[column] = reordered_value
        found_by_column[str(column)] = found

    for required_column in (
        Columns.LOSS_MASK,
        EPISODE_START_OFFSET_COLUMN,
        Postprocessing.ADVANTAGES,
    ):
        if not found_by_column.get(required_column, False):
            raise ValueError(
                f"{required_column} is not a timestep column with {input_rows} rows"
            )

    reordered_offsets = _flat_numeric_array(
        reordered[EPISODE_START_OFFSET_COLUMN],
        field=EPISODE_START_OFFSET_COLUMN,
    )
    reordered_rounded = np.round(
        reordered_offsets,
        decimals=_OFFSET_DECIMAL_PLACES,
    )
    expected_pattern = np.tile(unique_offsets, rows_per_start)
    if not np.array_equal(reordered_rounded, expected_pattern):
        raise RuntimeError("start-condition round-robin post-condition failed")
    max_run_length = _maximum_run_length(reordered_rounded)
    if max_run_length != 1:
        raise RuntimeError(
            "start-condition interleaving produced a contiguous run longer than one"
        )

    module_batch.update(reordered)
    return (
        input_rows,
        int(unique_offsets.size),
        rows_per_start,
        max_run_length,
    )


def standardize_real_advantages(
    advantages: Any,
    *,
    expected_rows: int,
) -> tuple[Any, float, float]:
    """Standardize scalar advantages over real rows only, preserving array type."""
    if not (torch.is_tensor(advantages) or isinstance(advantages, np.ndarray)):
        raise ValueError("advantages must be a torch tensor or NumPy array")
    if advantages.ndim != 1 or int(advantages.shape[0]) != expected_rows:
        raise ValueError(
            "advantages must be one-dimensional with one scalar per real "
            f"timestep, got shape {tuple(advantages.shape)!r}"
        )
    values = _flat_numeric_array(advantages, field="advantages")
    if values.size != expected_rows:
        raise ValueError(
            "advantages must contain exactly one scalar per real timestep: "
            f"{values.size} != {expected_rows}"
        )
    if not np.all(np.isfinite(values)):
        raise ValueError("real GAE advantages must all be finite")

    mean = float(np.mean(values, dtype=np.float64))
    std = float(np.std(values, dtype=np.float64))
    denominator = max(1e-4, std)

    if torch.is_tensor(advantages):
        if not torch.is_floating_point(advantages):
            raise ValueError("advantages tensor must have a floating-point dtype")
        normalized = (advantages - mean) / denominator
    else:  # NumPy branch; the supported types were checked above.
        if not np.issubdtype(advantages.dtype, np.floating):
            raise ValueError("advantages array must have a floating-point dtype")
        normalized = (advantages - mean) / denominator

    normalized_values = _flat_numeric_array(
        normalized,
        field="normalized advantages",
    )
    if not np.all(np.isfinite(normalized_values)):
        raise ValueError("normalized advantages must all be finite")
    return normalized, mean, std


def compact_and_normalize_module_batch(
    module_batch: MutableMapping[str, Any],
    *,
    interleave_starts: bool = False,
) -> BatchCompactionReport:
    """Remove artificial bootstrap rows and re-standardize real advantages.

    Every array/tensor leaf whose first axis is the timestep axis is filtered by
    ``Columns.LOSS_MASK``.  Validation and replacement are transactional: a
    malformed later column raises before any part of ``module_batch`` is changed.
    """
    if Columns.LOSS_MASK not in module_batch:
        raise KeyError("post-GAE Learner batch is missing loss-mask column")
    if EPISODE_START_OFFSET_COLUMN not in module_batch:
        raise KeyError(
            "post-GAE Learner batch is missing start-offset column; "
            "configure build_start_condition_learner_connector"
        )
    if Postprocessing.ADVANTAGES not in module_batch:
        raise KeyError("post-GAE Learner batch is missing advantages")
    if Columns.SEQ_LENS in module_batch:
        raise ValueError(
            "post-GAE compaction does not support sequence-padded/stateful batches"
        )

    active_rows = _validated_loss_mask(module_batch[Columns.LOSS_MASK])
    input_rows = int(active_rows.size)
    real_rows = int(np.count_nonzero(active_rows))
    if input_rows <= 0:
        raise ValueError("post-GAE Learner batch must not be empty")
    if real_rows <= 0:
        raise ValueError("loss mask removes every post-GAE Learner batch row")

    compacted: dict[str, Any] = {}
    found_by_column: dict[str, bool] = {}
    for column, value in module_batch.items():
        compacted_value, found = _filter_timestep_tree(
            value,
            active_rows=active_rows,
            input_rows=input_rows,
            field=str(column),
        )
        compacted[column] = compacted_value
        found_by_column[str(column)] = found

    for required_column in (
        Columns.LOSS_MASK,
        EPISODE_START_OFFSET_COLUMN,
        Postprocessing.ADVANTAGES,
    ):
        if not found_by_column.get(required_column, False):
            raise ValueError(
                f"{required_column} is not a timestep column with {input_rows} rows"
            )

    compacted_mask = _validated_loss_mask(
        compacted[Columns.LOSS_MASK],
        expected_rows=real_rows,
    )
    if not np.all(compacted_mask):
        raise ValueError("compacted loss mask still contains artificial rows")
    compacted_loss_mask = compacted[Columns.LOSS_MASK]
    if torch.is_tensor(compacted_loss_mask):
        compacted[Columns.LOSS_MASK] = torch.ones(
            real_rows,
            dtype=torch.bool,
            device=compacted_loss_mask.device,
        )
    else:
        compacted[Columns.LOSS_MASK] = np.ones(real_rows, dtype=bool)

    offsets = _flat_numeric_array(
        compacted[EPISODE_START_OFFSET_COLUMN],
        field=EPISODE_START_OFFSET_COLUMN,
    )
    if offsets.size != real_rows:
        raise ValueError(
            "start offsets must contain exactly one value per real timestep: "
            f"{offsets.size} != {real_rows}"
        )
    if not np.all(np.isfinite(offsets)):
        raise ValueError("real episode start offsets must all be finite")

    normalized_advantages, advantage_mean, advantage_std = (
        standardize_real_advantages(
            compacted[Postprocessing.ADVANTAGES],
            expected_rows=real_rows,
        )
    )
    compacted[Postprocessing.ADVANTAGES] = normalized_advantages

    interleaving = (0, 0, 0, 0)
    if interleave_starts:
        # ``compacted`` is still a private replacement mapping here.  If
        # interleaving rejects a group count, nested shape, or post-condition, the
        # caller's original module batch remains completely untouched.
        interleaving = interleave_equal_start_condition_rows(compacted)

    # Commit only after every relevant column and post-condition has passed.
    module_batch.update(compacted)
    return BatchCompactionReport(
        input_rows=input_rows,
        real_rows=real_rows,
        artificial_rows=input_rows - real_rows,
        advantage_mean_before_normalization=advantage_mean,
        advantage_std_before_normalization=advantage_std,
        interleaved_rows=interleaving[0],
        interleaved_start_conditions=interleaving[1],
        interleaved_rows_per_start=interleaving[2],
        max_start_run_length=interleaving[3],
    )


def aggregate_advantage_moments_by_start(
    start_offsets_s: Any,
    advantages: Any,
    loss_mask: Any | None = None,
) -> dict[str, dict[str, float | int]]:
    """Aggregate additive advantage moments for each active start condition.

    ``advantages`` must be the post-GAE values placed in the Learner batch by
    RLlib.  When supplied, ``loss_mask`` excludes artificial bootstrap and
    padded timesteps before any validation or aggregation.
    """
    offsets = _flat_numeric_array(
        start_offsets_s, field=EPISODE_START_OFFSET_COLUMN
    )
    advantage_values = _flat_numeric_array(advantages, field="advantages")
    if offsets.size != advantage_values.size:
        raise ValueError(
            "start offsets and advantages must contain the same number of values: "
            f"{offsets.size} != {advantage_values.size}"
        )

    if loss_mask is None:
        active = np.ones(offsets.size, dtype=bool)
    else:
        active = _validated_loss_mask(loss_mask, expected_rows=offsets.size)

    offsets = offsets[active]
    advantage_values = advantage_values[active]
    if offsets.size == 0:
        return {}
    if not np.all(np.isfinite(offsets)):
        raise ValueError("active episode start offsets must all be finite")
    if not np.all(np.isfinite(advantage_values)):
        raise ValueError("active GAE advantages must all be finite")

    # The label precision matches existing TensorBoard start-offset labels.  Group
    # by the same quantization so values represented with harmless float noise do
    # not split one configured start into multiple metrics.
    rounded_offsets = np.round(offsets, decimals=_OFFSET_DECIMAL_PLACES)
    result: dict[str, dict[str, float | int]] = {}
    for rounded_offset in np.unique(rounded_offsets):
        selected = rounded_offsets == rounded_offset
        selected_advantages = advantage_values[selected]
        label = format_start_offset_label(float(rounded_offset))
        result[label] = {
            EPISODE_START_OFFSET_COLUMN: float(rounded_offset),
            "advantage_sum": float(np.sum(selected_advantages, dtype=np.float64)),
            "advantage_sumsq": float(
                np.sum(np.square(selected_advantages), dtype=np.float64)
            ),
            "advantage_positive_count": int(
                np.count_nonzero(selected_advantages > 0.0)
            ),
            "advantage_count": int(selected_advantages.size),
        }
    return result


def derive_advantage_statistics_by_start(
    moments_by_start: Mapping[str, Mapping[str, Any]],
) -> dict[str, dict[str, float | int]]:
    """Derive population mean/std/positive fraction from additive moments."""
    result: dict[str, dict[str, float | int]] = {}
    for label, raw in moments_by_start.items():
        count = int(raw["advantage_count"])
        positive_count = int(raw["advantage_positive_count"])
        advantage_sum = _finite_float(raw["advantage_sum"], field="advantage_sum")
        advantage_sumsq = _finite_float(
            raw["advantage_sumsq"], field="advantage_sumsq"
        )
        offset = (
            _finite_float(
                raw[EPISODE_START_OFFSET_COLUMN],
                field=EPISODE_START_OFFSET_COLUMN,
            )
            if EPISODE_START_OFFSET_COLUMN in raw
            else parse_start_offset_label(str(label))
        )
        if count <= 0:
            raise ValueError(f"{label}: advantage_count must be positive")
        if positive_count < 0 or positive_count > count:
            raise ValueError(
                f"{label}: advantage_positive_count must be within [0, count]"
            )
        if advantage_sumsq < 0.0:
            raise ValueError(f"{label}: advantage_sumsq must be non-negative")

        mean = advantage_sum / count
        second_moment = advantage_sumsq / count
        variance = second_moment - mean * mean
        tolerance = 1e-12 * max(1.0, abs(second_moment), mean * mean)
        if variance < -tolerance:
            raise ValueError(f"{label}: moments imply a negative variance {variance}")
        variance = max(0.0, variance)
        result[str(label)] = {
            EPISODE_START_OFFSET_COLUMN: offset,
            "advantage_sum": advantage_sum,
            "advantage_sumsq": advantage_sumsq,
            "advantage_positive_count": positive_count,
            "advantage_count": count,
            "advantage_mean": float(mean),
            "advantage_std": float(math.sqrt(variance)),
            "advantage_positive_fraction": float(positive_count / count),
        }
    return result


class AddEpisodeStartOffsetToTrainBatch(ConnectorV2):
    """Attach each episode's numeric start offset to all of its timesteps."""

    @override(ConnectorV2)
    def __call__(
        self,
        *,
        rl_module,
        batch: dict[str, Any],
        episodes,
        **kwargs,
    ) -> dict[str, Any]:
        if EPISODE_START_OFFSET_COLUMN in batch:
            return batch

        for episode in self.single_agent_episode_iterator(
            episodes,
            agents_that_stepped_only=False,
        ):
            length = len(episode)
            if length <= 0:
                continue
            offset = episode_start_offset_from_infos(episode.get_infos())
            self.add_n_batch_items(
                batch,
                EPISODE_START_OFFSET_COLUMN,
                np.full(length, offset, dtype=np.float64),
                length,
                episode,
            )
        return batch


def build_start_condition_learner_connector(
    input_observation_space=None,
    input_action_space=None,
) -> AddEpisodeStartOffsetToTrainBatch:
    """Picklable top-level builder for ``AlgorithmConfig.learners``."""
    return AddEpisodeStartOffsetToTrainBatch(
        input_observation_space=input_observation_space,
        input_action_space=input_action_space,
    )


def _iter_module_batches(batch: Mapping[str, Any]):
    """Yield both plain single-module and module-keyed Learner batch layouts."""
    if Postprocessing.ADVANTAGES in batch:
        yield DEFAULT_MODULE_ID, batch
        return
    for module_id, module_batch in batch.items():
        if (
            isinstance(module_batch, Mapping)
            and Postprocessing.ADVANTAGES in module_batch
        ):
            yield str(module_id), module_batch


class LogStartConditionAdvantageMoments(ConnectorV2):
    """Compact, re-standardize, and log the post-GAE Learner batch."""

    @override(ConnectorV2)
    def __call__(
        self,
        *,
        rl_module,
        batch: dict[str, Any],
        episodes,
        metrics=None,
        **kwargs,
    ) -> dict[str, Any]:
        for module_id, module_batch in _iter_module_batches(batch):
            if not isinstance(module_batch, MutableMapping):
                raise TypeError(
                    f"post-GAE module batch {module_id!r} must be mutable"
                )
            compaction = compact_and_normalize_module_batch(
                module_batch,
                interleave_starts=True,
            )
            moments = aggregate_advantage_moments_by_start(
                module_batch[EPISODE_START_OFFSET_COLUMN],
                module_batch[Postprocessing.ADVANTAGES],
            )
            if metrics is None:
                continue
            for key, value in (
                ("pre_rows", compaction.input_rows),
                ("removed_rows", compaction.artificial_rows),
                ("compacted_rows", compaction.real_rows),
                ("interleaved_rows", compaction.interleaved_rows),
                (
                    "interleaved_start_conditions",
                    compaction.interleaved_start_conditions,
                ),
                ("interleaved_rows_per_start", compaction.interleaved_rows_per_start),
            ):
                metrics.log_value(
                    (
                        module_id,
                        START_CONDITION_BATCH_METRICS_PREFIX,
                        key,
                    ),
                    value,
                    reduce="sum",
                )
            metrics.log_value(
                (
                    module_id,
                    START_CONDITION_BATCH_METRICS_PREFIX,
                    "max_start_run_length",
                ),
                compaction.max_start_run_length,
                reduce="max",
            )
            for label, values in moments.items():
                for key in _ADVANTAGE_MOMENT_KEYS:
                    metrics.log_value(
                        (
                            module_id,
                            START_CONDITION_METRICS_PREFIX,
                            label,
                            key,
                        ),
                        values[key],
                        reduce="sum",
                    )
        return batch


class StartConditionMetricsPPOTorchLearner(PPOTorchLearner):
    """PPO Learner that compacts and audits each batch after RLlib's GAE."""

    @override(PPOTorchLearner)
    def _update(self, batch: dict[str, Any]):
        """Track the full-update range of PPO's per-minibatch mean KL.

        RLlib logs ``mean_kl_loss`` with ``window=1`` inside
        ``compute_loss_for_module``.  At this point that metric therefore holds
        the value from the minibatch that ``super()._update`` just consumed.
        The max/min/sum reducers below accumulate until ``Learner.update``
        reduces the MetricsLogger after the complete minibatch loop.

        Ray's max/min reducers intentionally ignore NaNs.  Replace every
        non-finite KL with opposite infinities for max and min so neither
        reduction can silently hide it; ``nonfinite_count`` records the reason
        explicitly and downstream finite-only extraction fails closed.
        """

        result = super()._update(batch)
        _, loss_per_module, _ = result
        for module_id in loss_per_module:
            current_kl = self.metrics.peek(
                (module_id, LEARNER_RESULTS_KL_KEY)
            )
            kl = torch.as_tensor(current_kl)
            if kl.numel() != 1:
                raise RuntimeError(
                    "PPO mean_kl_loss must be scalar, got shape "
                    f"{tuple(kl.shape)} for module {module_id!r}"
                )
            kl = kl.reshape(())
            if not torch.is_floating_point(kl):
                kl = kl.to(dtype=torch.float32)

            is_nonfinite = ~torch.isfinite(kl)
            max_safe_kl = torch.where(
                is_nonfinite,
                torch.full_like(kl, float("inf")),
                kl,
            )
            min_safe_kl = torch.where(
                is_nonfinite,
                torch.full_like(kl, float("-inf")),
                kl,
            )
            metric_prefix = (module_id, KL_UPDATE_METRICS_PREFIX)
            self.metrics.log_value(
                metric_prefix + (KL_UPDATE_MAX_MINIBATCH_MEAN_KEY,),
                max_safe_kl,
                reduce="max",
            )
            self.metrics.log_value(
                metric_prefix + (KL_UPDATE_MIN_MINIBATCH_MEAN_KEY,),
                min_safe_kl,
                reduce="min",
            )
            self.metrics.log_value(
                metric_prefix + (KL_UPDATE_MINIBATCH_COUNT_KEY,),
                1,
                reduce="sum",
            )
            self.metrics.log_value(
                metric_prefix + (KL_UPDATE_NONFINITE_COUNT_KEY,),
                is_nonfinite.to(dtype=torch.int64),
                reduce="sum",
            )
        return result

    @override(PPOTorchLearner)
    def build(self) -> None:
        # PPOTorchLearner/PPOLearner appends GeneralAdvantageEstimation here.
        # Appending our connector afterwards guarantees that the values being
        # compacted/re-standardized/logged are the exact real advantages consumed
        # by PPO's loss.
        super().build()
        if self._learner_connector is None:
            return
        if not getattr(self, "_start_condition_metrics_connector_added", False):
            self._learner_connector.append(LogStartConditionAdvantageMoments())
            self._start_condition_metrics_connector_added = True


__all__ = [
    "AddEpisodeStartOffsetToTrainBatch",
    "BatchCompactionReport",
    "EPISODE_START_OFFSET_COLUMN",
    "KL_UPDATE_MAX_MINIBATCH_MEAN_KEY",
    "KL_UPDATE_METRICS_PREFIX",
    "KL_UPDATE_MINIBATCH_COUNT_KEY",
    "KL_UPDATE_MIN_MINIBATCH_MEAN_KEY",
    "KL_UPDATE_NONFINITE_COUNT_KEY",
    "LogStartConditionAdvantageMoments",
    "START_CONDITION_BATCH_METRICS_PREFIX",
    "START_CONDITION_METRICS_PREFIX",
    "StartConditionMetricsPPOTorchLearner",
    "aggregate_advantage_moments_by_start",
    "build_start_condition_learner_connector",
    "compact_and_normalize_module_batch",
    "derive_advantage_statistics_by_start",
    "episode_start_offset_from_infos",
    "format_start_offset_label",
    "interleave_equal_start_condition_rows",
    "parse_start_offset_label",
    "standardize_real_advantages",
]
