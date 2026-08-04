"""Fail-closed robustness gate for one selected warm-start PPO iteration.

The tool has two modes:

* launch four ``rollout_eval.py`` processes serially (three deterministic
  absolute-start rollouts and one stochastic +0.20 s rollout); or
* classify four existing ``rollout_summary.json`` files.

It only writes evaluation artifacts below ``--output-dir``.  In particular it
never copies, replaces, or promotes the checkpoint under test.

Reserve non-regression is condition-matched by default in the scientific
workflow: four validated H0 summaries provide one immutable cap per rollout
condition.  A single scalar cap remains available only for legacy callers.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import shutil
import subprocess
import sys
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Mapping, Sequence


ROOT_DIR = Path(__file__).resolve().parents[1]
DEFAULT_ROLLOUT_SCRIPT = (
    ROOT_DIR / "Trajectory Generator" / "baseline_MLP" / "rollout_eval.py"
)
DEFAULT_START_OFFSETS_S = (
    1.756870983805102,
    1.956870983805102,
    2.156870983805102,
)
EXPECTED_STEPS = 500
EXPECTED_SEED = 123
EXPECTED_ACTOR_FEATURES = 35
EXPECTED_OBSERVATION_FEATURES = 84
EXPECTED_MIN_CYCLES = 2
PENETRATION_LIMIT_M = 0.025
KL_LIMIT = 0.01
KL_MIN_NUMERICAL_TOLERANCE = -1.0e-7
EXPECTED_KL_MINIBATCH_COUNT = 9
EXPECTED_START_STEPS = 1536
EXPECTED_TRAINING_ITERATION = 2
OFFSET_TOLERANCE_S = 1.0e-9
SIGMA_ABS_TOLERANCE = 1.0e-8
GATE_SCHEMA_VERSION = 2
# A 500-step stochastic rollout supplies many Gaussian draws, but its realized
# RMS is still a sample statistic.  A deliberately broad [0.5, 1.5] sigma band
# rejects disabled/zero or grossly mis-scaled exploration without requiring an
# implausibly exact match to the configured standard deviation.
REALIZED_NOISE_RMS_MIN_SIGMA_RATIO = 0.5
REALIZED_NOISE_RMS_MAX_SIGMA_RATIO = 1.5
REQUIRED_RL_MODULE_FILES = (
    "class_and_ctor_args.pkl",
    "metadata.json",
    "module_state.pkl",
)
RESERVE_NUMERICAL_TOLERANCE_ABS_NM = 1.0e-6
RESERVE_NUMERICAL_TOLERANCE_REL = 1.0e-9
RESERVE_MODE_LEGACY_SCALAR = "legacy_scalar"
RESERVE_MODE_CONDITION_MATCHED = "condition_matched_non_regression"


@dataclass(frozen=True)
class GateConfig:
    checkpoint: Path
    output_dir: Path
    max_reserve_norm_nm: float | None = None
    reserve_reference_checkpoint: Path | None = None
    reserve_reference_summaries: tuple[Path, Path, Path, Path] | None = None
    expected_sigma: float = 0.005
    start_offsets_s: tuple[float, float, float] = DEFAULT_START_OFFSETS_S
    train_iterations: Path | None = None
    expected_training_iteration: int = EXPECTED_TRAINING_ITERATION
    run_rollouts: bool = False
    deterministic_summaries: tuple[Path, Path, Path] | None = None
    stochastic_summary: Path | None = None
    python_executable: str = sys.executable
    rollout_script: Path = DEFAULT_ROLLOUT_SCRIPT
    rollout_timeout_s: float = 900.0


@dataclass(frozen=True)
class RolloutSpec:
    name: str
    offset_s: float
    action_selection: str
    seed: int = EXPECTED_SEED


def _reject_nonfinite_json(token: str) -> None:
    raise ValueError(f"non-finite JSON number {token!r}")


def _read_json_object(path: Path) -> dict[str, Any]:
    value = json.loads(
        path.read_text(encoding="utf-8"),
        parse_constant=_reject_nonfinite_json,
    )
    if not isinstance(value, Mapping):
        raise ValueError(f"expected a JSON object: {path}")
    return dict(value)


def _jsonl_objects(path: Path) -> list[dict[str, Any]]:
    records: list[dict[str, Any]] = []
    for line_number, raw_line in enumerate(
        path.read_text(encoding="utf-8").splitlines(), start=1
    ):
        if not raw_line.strip():
            continue
        try:
            value = json.loads(raw_line, parse_constant=_reject_nonfinite_json)
        except (json.JSONDecodeError, ValueError) as exc:
            raise ValueError(f"invalid JSON on {path}:{line_number}: {exc}") from exc
        if not isinstance(value, Mapping):
            raise ValueError(f"expected an object on {path}:{line_number}")
        records.append(dict(value))
    if not records:
        raise ValueError(f"no training iterations found: {path}")
    return records


def _finite_number(value: Any) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    converted = float(value)
    return converted if math.isfinite(converted) else None


def _reserve_mode(config: GateConfig) -> str:
    scalar = config.max_reserve_norm_nm is not None
    reference_checkpoint = config.reserve_reference_checkpoint is not None
    reference_summaries = config.reserve_reference_summaries is not None
    if scalar and not reference_checkpoint and not reference_summaries:
        return RESERVE_MODE_LEGACY_SCALAR
    if not scalar and reference_checkpoint and reference_summaries:
        return RESERVE_MODE_CONDITION_MATCHED
    return "invalid"


def _reserve_numerical_tolerance_nm(reference_cap_nm: float) -> float:
    """Return the declared numerical-only reserve comparison tolerance."""

    value = _finite_number(reference_cap_nm)
    if value is None or value < 0.0:
        raise ValueError("reserve reference cap must be finite and non-negative")
    return max(
        RESERVE_NUMERICAL_TOLERANCE_ABS_NM,
        RESERVE_NUMERICAL_TOLERANCE_REL * value,
    )


def _canonical_path(value: str | os.PathLike[str]) -> str:
    text = str(value).strip()
    if os.sep == "/":
        text = text.replace("\\", "/")
    path = Path(text).expanduser().resolve(strict=False)
    return os.path.normcase(os.path.normpath(str(path)))


def _same_path(left: Any, right: Path) -> bool:
    if not isinstance(left, str) or not left.strip():
        return False
    try:
        return _canonical_path(left) == _canonical_path(right)
    except (OSError, RuntimeError, ValueError):
        return False


def _portable_path(path: Path) -> str:
    return str(path.expanduser().resolve(strict=False))


def _check(name: str, ok: bool, actual: Any, expected: Any) -> dict[str, Any]:
    return {
        "name": name,
        "status": "PASS" if ok else "FAIL",
        "actual": actual,
        "expected": expected,
    }


def _numeric_equal(actual: Any, expected: float, *, tolerance: float = 0.0) -> bool:
    value = _finite_number(actual)
    if value is None:
        return False
    return math.isclose(value, expected, rel_tol=0.0, abs_tol=tolerance)


def _sigma_contract(value: Any, expected_sigma: float) -> tuple[bool, Any]:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)):
        return False, value
    converted: list[float] = []
    for item in value:
        number = _finite_number(item)
        if number is None:
            return False, value
        converted.append(number)
    if len(converted) != 2:
        return False, converted
    ok = all(
        math.isclose(
            item,
            expected_sigma,
            rel_tol=0.0,
            abs_tol=SIGMA_ABS_TOLERANCE,
        )
        for item in converted
    )
    return ok, converted


def _realized_noise_rms_contract(
    value: Any,
    expected_sigma: float,
) -> tuple[bool, Any]:
    """Require two positive RMS values within a robust finite-sample band."""
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)):
        return False, value
    converted: list[float] = []
    for item in value:
        number = _finite_number(item)
        if number is None:
            return False, value
        converted.append(number)
    if len(converted) != 2:
        return False, converted

    lower = expected_sigma * REALIZED_NOISE_RMS_MIN_SIGMA_RATIO
    upper = expected_sigma * REALIZED_NOISE_RMS_MAX_SIGMA_RATIO
    return (
        all(item > 0.0 and lower <= item <= upper for item in converted),
        converted,
    )


def _action_shape_contract(value: Any) -> bool:
    """Require the exact JSON action-shape contract emitted by this policy."""
    return (
        isinstance(value, list)
        and len(value) == 1
        and type(value[0]) is int
        and value[0] == 2
    )


def classify_rollout_summary(
    summary: Mapping[str, Any],
    *,
    expected_checkpoint: Path,
    spec: RolloutSpec,
    expected_sigma: float,
    max_reserve_norm_nm: float,
    reserve_numerical_tolerance_nm: float = 0.0,
) -> dict[str, Any]:
    """Classify one rollout summary without importing the simulator stack."""

    checks: list[dict[str, Any]] = []
    checks.append(_check("summary_ok", summary.get("ok") is True, summary.get("ok"), True))
    checks.append(
        _check(
            "checkpoint",
            _same_path(summary.get("checkpoint"), expected_checkpoint),
            summary.get("checkpoint"),
            _portable_path(expected_checkpoint),
        )
    )
    checks.append(
        _check(
            "episode_start_offset_s",
            _numeric_equal(
                summary.get("episode_start_offset_s"),
                spec.offset_s,
                tolerance=OFFSET_TOLERANCE_S,
            ),
            summary.get("episode_start_offset_s"),
            spec.offset_s,
        )
    )
    checks.append(
        _check(
            "action_mode",
            summary.get("action_mode") == "absolute",
            summary.get("action_mode"),
            "absolute",
        )
    )
    checks.append(
        _check(
            "action_selection",
            summary.get("action_selection") == spec.action_selection,
            summary.get("action_selection"),
            spec.action_selection,
        )
    )
    checks.append(
        _check(
            "action_seed",
            _numeric_equal(summary.get("action_seed"), float(spec.seed)),
            summary.get("action_seed"),
            spec.seed,
        )
    )

    if spec.action_selection == "stochastic":
        sigma_ok, sigma_actual = _sigma_contract(
            summary.get("exploration_std_mean"), expected_sigma
        )
        checks.append(
            _check(
                "exploration_sigma",
                sigma_ok,
                sigma_actual,
                f"two finite values, each == {expected_sigma}",
            )
        )
        noise_rms_ok, noise_rms_actual = _realized_noise_rms_contract(
            summary.get("exploration_noise_realized_rms"), expected_sigma
        )
        checks.append(
            _check(
                "exploration_noise_realized_rms",
                noise_rms_ok,
                noise_rms_actual,
                (
                    "two finite positive values in "
                    f"[{REALIZED_NOISE_RMS_MIN_SIGMA_RATIO}, "
                    f"{REALIZED_NOISE_RMS_MAX_SIGMA_RATIO}] * sigma"
                ),
            )
        )
    else:
        sigma_present = "exploration_std_mean" in summary
        sigma_actual = summary.get("exploration_std_mean")
        checks.append(
            _check(
                "deterministic_has_no_exploration_sigma",
                sigma_present and sigma_actual is None,
                sigma_actual if sigma_present else "<missing>",
                None,
            )
        )
        noise_rms_present = "exploration_noise_realized_rms" in summary
        noise_rms_actual = summary.get("exploration_noise_realized_rms")
        checks.append(
            _check(
                "deterministic_has_no_exploration_noise_rms",
                noise_rms_present and noise_rms_actual is None,
                noise_rms_actual if noise_rms_present else "<missing>",
                None,
            )
        )

    checks.extend(
        [
            _check(
                "steps",
                _numeric_equal(summary.get("steps"), float(EXPECTED_STEPS)),
                summary.get("steps"),
                EXPECTED_STEPS,
            ),
            _check(
                "end_reason",
                summary.get("end_reason") == "episode_time_limit",
                summary.get("end_reason"),
                "episode_time_limit",
            ),
            _check(
                "terminated",
                summary.get("terminated") is False,
                summary.get("terminated"),
                False,
            ),
            _check(
                "truncated",
                summary.get("truncated") is True,
                summary.get("truncated"),
                True,
            ),
        ]
    )

    cycles = _finite_number(summary.get("phase_valid_cycle_count"))
    checks.append(
        _check(
            "action_shape",
            _action_shape_contract(summary.get("action_shape")),
            summary.get("action_shape"),
            [2],
        )
    )
    checks.append(
        _check(
            "phase_valid_cycle_count",
            cycles is not None and cycles >= EXPECTED_MIN_CYCLES,
            summary.get("phase_valid_cycle_count"),
            f">= {EXPECTED_MIN_CYCLES}",
        )
    )
    penetration = _finite_number(summary.get("grf_penetration_max_m"))
    penetration_samples = _finite_number(summary.get("grf_penetration_samples"))
    checks.append(
        _check(
            "grf_penetration_samples",
            penetration_samples is not None
            and math.isclose(
                penetration_samples,
                float(EXPECTED_STEPS),
                rel_tol=0.0,
                abs_tol=0.0,
            ),
            summary.get("grf_penetration_samples"),
            EXPECTED_STEPS,
        )
    )
    checks.append(
        _check(
            "grf_penetration_max_m",
            penetration is not None
            and penetration >= 0.0
            and penetration < PENETRATION_LIMIT_M,
            summary.get("grf_penetration_max_m"),
            f"finite and in [0, {PENETRATION_LIMIT_M})",
        )
    )
    checks.append(
        _check(
            "action_clipped_steps",
            _numeric_equal(summary.get("action_clipped_steps"), 0.0),
            summary.get("action_clipped_steps"),
            0,
        )
    )
    checks.append(
        _check(
            "n_actor",
            _numeric_equal(summary.get("n_actor"), float(EXPECTED_ACTOR_FEATURES)),
            summary.get("n_actor"),
            EXPECTED_ACTOR_FEATURES,
        )
    )
    checks.append(
        _check(
            "n_observation",
            _numeric_equal(
                summary.get("n_observation"),
                float(EXPECTED_OBSERVATION_FEATURES),
            ),
            summary.get("n_observation"),
            EXPECTED_OBSERVATION_FEATURES,
        )
    )
    reserve = _finite_number(summary.get("reserve_norm_max_nm"))
    reserve_samples = _finite_number(summary.get("reserve_norm_samples"))
    reserve_limit = _finite_number(max_reserve_norm_nm)
    reserve_tolerance = _finite_number(reserve_numerical_tolerance_nm)
    reserve_upper_bound = _finite_number(
        reserve_limit + reserve_tolerance
        if reserve_limit is not None
        and reserve_limit >= 0.0
        and reserve_tolerance is not None
        and reserve_tolerance >= 0.0
        else None
    )
    reserve_expected: Any
    if reserve_tolerance == 0.0:
        reserve_expected = f"finite and in [0, {reserve_limit}]"
    else:
        reserve_expected = {
            "finite_non_negative": True,
            "reference_or_scalar_cap_nm": reserve_limit,
            "numerical_tolerance_nm": reserve_tolerance,
            "maximum_inclusive_nm": reserve_upper_bound,
        }
    checks.append(
        _check(
            "reserve_norm_samples",
            reserve_samples is not None
            and math.isclose(
                reserve_samples,
                float(EXPECTED_STEPS),
                rel_tol=0.0,
                abs_tol=0.0,
            ),
            summary.get("reserve_norm_samples"),
            EXPECTED_STEPS,
        )
    )
    checks.append(
        _check(
            "reserve_norm_max_nm",
            reserve is not None
            and reserve >= 0.0
            and reserve_upper_bound is not None
            and reserve <= reserve_upper_bound,
            summary.get("reserve_norm_max_nm"),
            reserve_expected,
        )
    )

    failed = [check["name"] for check in checks if check["status"] != "PASS"]
    return {
        "name": spec.name,
        "status": "PASS" if not failed else "FAIL",
        "expected": {
            "offset_s": spec.offset_s,
            "action_selection": spec.action_selection,
            "seed": spec.seed,
        },
        "checks": checks,
        "failed_checks": failed,
        "reserve_contract": {
            "reference_or_scalar_cap_nm": reserve_limit,
            "numerical_tolerance_nm": reserve_tolerance,
            "maximum_inclusive_nm": reserve_upper_bound,
        },
    }


def _offset_metric_label(offset_s: float) -> str:
    return f"offset_{offset_s:.6f}".replace("-", "m").replace(".", "p") + "s"


def _current_start_steps(coverage: Any) -> dict[str, Any] | None:
    """Select current-update counts while deliberately ignoring lifetime counts."""

    if not isinstance(coverage, Mapping):
        return None
    marker = "/episode_start_steps_current/"
    selected: dict[str, Any] = {}
    for raw_key, value in coverage.items():
        key = str(raw_key)
        if marker in key:
            label = key.split(marker, 1)[1]
        elif key.startswith("episode_start_steps_current/"):
            label = key.split("/", 1)[1]
        else:
            continue
        selected[label] = value
    return selected


def _exact_step_mapping(value: Any, expected: Mapping[str, int]) -> bool:
    return (
        isinstance(value, Mapping)
        and set(value) == set(expected)
        and all(
            _numeric_equal(value.get(label), float(step_count))
            for label, step_count in expected.items()
        )
    )


def classify_training_iterations(
    path: Path,
    *,
    start_offsets_s: Sequence[float],
    expected_training_iteration: int = EXPECTED_TRAINING_ITERATION,
) -> dict[str, Any]:
    """Require one selected PPO iteration to meet KL and balance gates."""

    if (
        type(expected_training_iteration) is not int
        or expected_training_iteration <= 0
    ):
        return {
            "status": "FAIL",
            "path": _portable_path(path),
            "expected_iteration": expected_training_iteration,
            "error": "expected training iteration must be a positive integer",
            "checks": [],
            "failed_checks": ["training_iteration_selection"],
        }

    try:
        records = _jsonl_objects(path)
    except (OSError, ValueError) as exc:
        return {
            "status": "FAIL",
            "path": _portable_path(path),
            "expected_iteration": expected_training_iteration,
            "error": str(exc),
            "checks": [],
            "failed_checks": ["train_iterations_read"],
        }

    available_iterations = [
        record.get("iteration")
        for record in records
        if type(record.get("iteration")) is int
    ]
    matching_records = [
        record
        for record in records
        if type(record.get("iteration")) is int
        and record.get("iteration") == expected_training_iteration
    ]
    if len(matching_records) != 1:
        return {
            "status": "FAIL",
            "path": _portable_path(path),
            "record_count": len(records),
            "expected_iteration": expected_training_iteration,
            "available_iterations": available_iterations,
            "matching_record_count": len(matching_records),
            "classified_iteration": None,
            "error": (
                "expected exactly one train_iterations record for iteration "
                f"{expected_training_iteration}, found {len(matching_records)}"
            ),
            "checks": [],
            "failed_checks": ["training_iteration_selection"],
        }

    record = matching_records[0]
    # ``mean_kl_loss`` is retained for compatibility, but RLlib exposes the
    # final minibatch value there.  The four update-scoped fields below are the
    # authoritative guard against an earlier minibatch exceeding the KL cap.
    kl = _finite_number(record.get("mean_kl_loss"))
    max_minibatch_kl = _finite_number(
        record.get("max_minibatch_mean_kl_loss")
    )
    min_minibatch_kl = _finite_number(
        record.get("min_minibatch_mean_kl_loss")
    )
    kl_minibatch_count = _finite_number(record.get("kl_minibatch_count"))
    kl_nonfinite_count = _finite_number(record.get("kl_nonfinite_count"))
    coverage = record.get("start_coverage_metrics")
    current_steps = _current_start_steps(coverage)
    balance = record.get("exact_start_balance")
    expected_steps = {
        _offset_metric_label(offset): EXPECTED_START_STEPS
        for offset in start_offsets_s
    }
    balance_is_mapping = isinstance(balance, Mapping)
    reported_expected = balance.get("expected_steps") if balance_is_mapping else None
    reported_actual = balance.get("actual_steps") if balance_is_mapping else None
    reported_advantage_counts = (
        balance.get("advantage_counts") if balance_is_mapping else None
    )
    reported_connector_out = _finite_number(
        balance.get("learner_connector_steps_out") if balance_is_mapping else None
    )
    reported_pre_rows = _finite_number(
        balance.get("pre_compaction_rows") if balance_is_mapping else None
    )
    reported_removed_rows = _finite_number(
        balance.get("removed_compaction_rows") if balance_is_mapping else None
    )
    reported_compacted_rows = _finite_number(
        balance.get("compacted_rows") if balance_is_mapping else None
    )
    reported_interleaved_rows = _finite_number(
        balance.get("interleaved_rows") if balance_is_mapping else None
    )
    reported_interleaved_start_conditions = _finite_number(
        balance.get("interleaved_start_conditions")
        if balance_is_mapping
        else None
    )
    reported_interleaved_rows_per_start = _finite_number(
        balance.get("interleaved_rows_per_start")
        if balance_is_mapping
        else None
    )
    reported_max_start_run_length = _finite_number(
        balance.get("max_start_run_length") if balance_is_mapping else None
    )
    reported_learner_checks = (
        balance.get("learner_checks") if balance_is_mapping else None
    )
    learner_checks_is_mapping = isinstance(reported_learner_checks, Mapping)
    expected_real_steps = EXPECTED_START_STEPS * len(expected_steps)

    checks = [
        _check(
            "iteration",
            _numeric_equal(
                record.get("iteration"), float(expected_training_iteration)
            ),
            record.get("iteration"),
            expected_training_iteration,
        ),
        _check(
            "mean_kl_loss",
            kl is not None and 0.0 <= kl <= KL_LIMIT,
            record.get("mean_kl_loss"),
            f"legacy last-minibatch value; finite and in [0, {KL_LIMIT}]",
        ),
        _check(
            "max_minibatch_mean_kl_loss",
            max_minibatch_kl is not None
            and 0.0 <= max_minibatch_kl <= KL_LIMIT,
            record.get("max_minibatch_mean_kl_loss"),
            f"update-scoped finite maximum in [0, {KL_LIMIT}]",
        ),
        _check(
            "min_minibatch_mean_kl_loss",
            min_minibatch_kl is not None
            and min_minibatch_kl >= KL_MIN_NUMERICAL_TOLERANCE,
            record.get("min_minibatch_mean_kl_loss"),
            (
                "update-scoped finite minimum >= "
                f"{KL_MIN_NUMERICAL_TOLERANCE} (numerical tolerance)"
            ),
        ),
        _check(
            "kl_minibatch_count",
            kl_minibatch_count is not None
            and math.isclose(
                kl_minibatch_count,
                float(EXPECTED_KL_MINIBATCH_COUNT),
                rel_tol=0.0,
                abs_tol=0.0,
            ),
            record.get("kl_minibatch_count"),
            EXPECTED_KL_MINIBATCH_COUNT,
        ),
        _check(
            "kl_nonfinite_count",
            kl_nonfinite_count is not None
            and math.isclose(
                kl_nonfinite_count,
                0.0,
                rel_tol=0.0,
                abs_tol=0.0,
            ),
            record.get("kl_nonfinite_count"),
            0,
        ),
        _check(
            "exact_start_balance.pass",
            balance_is_mapping and balance.get("pass") is True,
            balance.get("pass") if balance_is_mapping else balance,
            True,
        ),
        _check(
            "exact_start_balance.expected_steps",
            _exact_step_mapping(reported_expected, expected_steps),
            dict(reported_expected)
            if isinstance(reported_expected, Mapping)
            else reported_expected,
            expected_steps,
        ),
        _check(
            "exact_start_balance.actual_steps",
            _exact_step_mapping(reported_actual, expected_steps),
            dict(reported_actual)
            if isinstance(reported_actual, Mapping)
            else reported_actual,
            expected_steps,
        ),
        _check(
            "start_coverage_metrics.episode_start_steps_current",
            _exact_step_mapping(current_steps, expected_steps),
            current_steps,
            expected_steps,
        ),
        _check(
            "exact_start_balance.learner_batch_pass",
            balance_is_mapping and balance.get("learner_batch_pass") is True,
            balance.get("learner_batch_pass") if balance_is_mapping else balance,
            True,
        ),
        _check(
            "exact_start_balance.expected_real_steps",
            balance_is_mapping
            and _numeric_equal(
                balance.get("expected_real_steps"), float(expected_real_steps)
            ),
            balance.get("expected_real_steps") if balance_is_mapping else balance,
            expected_real_steps,
        ),
        _check(
            "exact_start_balance.learner_connector_steps_in",
            balance_is_mapping
            and _numeric_equal(
                balance.get("learner_connector_steps_in"),
                float(expected_real_steps),
            ),
            balance.get("learner_connector_steps_in")
            if balance_is_mapping
            else balance,
            expected_real_steps,
        ),
        _check(
            "exact_start_balance.post_gae_compaction",
            balance_is_mapping
            and reported_connector_out is not None
            and reported_pre_rows is not None
            and reported_removed_rows is not None
            and reported_compacted_rows is not None
            and reported_removed_rows > 0.0
            and math.isclose(
                reported_connector_out,
                reported_pre_rows,
                rel_tol=0.0,
                abs_tol=0.0,
            )
            and math.isclose(
                reported_compacted_rows,
                float(expected_real_steps),
                rel_tol=0.0,
                abs_tol=0.0,
            )
            and math.isclose(
                reported_pre_rows - reported_removed_rows,
                float(expected_real_steps),
                rel_tol=0.0,
                abs_tol=0.0,
            ),
            {
                "connector_out": balance.get("learner_connector_steps_out"),
                "pre_rows": balance.get("pre_compaction_rows"),
                "removed_rows": balance.get("removed_compaction_rows"),
                "compacted_rows": balance.get("compacted_rows"),
            }
            if balance_is_mapping
            else balance,
            {
                "connector_out": "pre_rows",
                "pre_rows_minus_removed_rows": expected_real_steps,
                "removed_rows": "> 0",
                "compacted_rows": expected_real_steps,
            },
        ),
        _check(
            "exact_start_balance.interleaved_rows",
            reported_interleaved_rows is not None
            and math.isclose(
                reported_interleaved_rows,
                float(expected_real_steps),
                rel_tol=0.0,
                abs_tol=0.0,
            ),
            balance.get("interleaved_rows") if balance_is_mapping else balance,
            expected_real_steps,
        ),
        _check(
            "exact_start_balance.interleaved_start_conditions",
            reported_interleaved_start_conditions is not None
            and math.isclose(
                reported_interleaved_start_conditions,
                float(len(expected_steps)),
                rel_tol=0.0,
                abs_tol=0.0,
            ),
            (
                balance.get("interleaved_start_conditions")
                if balance_is_mapping
                else balance
            ),
            len(expected_steps),
        ),
        _check(
            "exact_start_balance.interleaved_rows_per_start",
            reported_interleaved_rows_per_start is not None
            and math.isclose(
                reported_interleaved_rows_per_start,
                float(EXPECTED_START_STEPS),
                rel_tol=0.0,
                abs_tol=0.0,
            ),
            (
                balance.get("interleaved_rows_per_start")
                if balance_is_mapping
                else balance
            ),
            EXPECTED_START_STEPS,
        ),
        _check(
            "exact_start_balance.max_start_run_length",
            reported_max_start_run_length is not None
            and math.isclose(
                reported_max_start_run_length,
                1.0,
                rel_tol=0.0,
                abs_tol=0.0,
            ),
            (
                balance.get("max_start_run_length")
                if balance_is_mapping
                else balance
            ),
            1,
        ),
        _check(
            "exact_start_balance.learner_checks.start_interleaving",
            learner_checks_is_mapping
            and reported_learner_checks.get("start_interleaving") is True,
            (
                reported_learner_checks.get("start_interleaving")
                if learner_checks_is_mapping
                else reported_learner_checks
            ),
            True,
        ),
        _check(
            "exact_start_balance.learner_checks.single_epoch_contract",
            learner_checks_is_mapping
            and reported_learner_checks.get("single_epoch_contract") is True,
            (
                reported_learner_checks.get("single_epoch_contract")
                if learner_checks_is_mapping
                else reported_learner_checks
            ),
            True,
        ),
        _check(
            "exact_start_balance.module_steps_trained",
            balance_is_mapping
            and _numeric_equal(
                balance.get("expected_module_steps_trained"),
                float(expected_real_steps),
            )
            and _numeric_equal(
                balance.get("module_steps_trained"),
                float(expected_real_steps),
            ),
            {
                "expected": balance.get("expected_module_steps_trained"),
                "actual": balance.get("module_steps_trained"),
            }
            if balance_is_mapping
            else balance,
            {"expected": expected_real_steps, "actual": expected_real_steps},
        ),
        _check(
            "exact_start_balance.advantage_counts",
            _exact_step_mapping(reported_advantage_counts, expected_steps),
            dict(reported_advantage_counts)
            if isinstance(reported_advantage_counts, Mapping)
            else reported_advantage_counts,
            expected_steps,
        ),
    ]
    failed = [check["name"] for check in checks if check["status"] != "PASS"]
    return {
        "status": "PASS" if not failed else "FAIL",
        "path": _portable_path(path),
        "record_count": len(records),
        "expected_iteration": expected_training_iteration,
        "available_iterations": available_iterations,
        "matching_record_count": len(matching_records),
        "classified_iteration": record.get("iteration"),
        "checks": checks,
        "failed_checks": failed,
    }


def _rollout_specs(offsets: Sequence[float]) -> tuple[RolloutSpec, ...]:
    return (
        RolloutSpec("deterministic_minus020", float(offsets[0]), "deterministic"),
        RolloutSpec("deterministic_nominal", float(offsets[1]), "deterministic"),
        RolloutSpec("deterministic_plus020", float(offsets[2]), "deterministic"),
        RolloutSpec("stochastic_plus020_seed123", float(offsets[2]), "stochastic"),
    )


def _validate_rl_module_checkpoint(checkpoint: Path) -> list[str]:
    """Validate the direct, inference-only RLModule checkpoint contract."""

    path = checkpoint.expanduser().resolve(strict=False)
    failures: list[str] = []
    if path.name != "rl_module_last":
        failures.append(
            "checkpoint basename must be exactly 'rl_module_last': " f"{path}"
        )
    if not path.is_dir():
        failures.append(f"checkpoint is not a directory: {path}")
        return failures

    for filename in REQUIRED_RL_MODULE_FILES:
        required = path / filename
        if not required.is_file():
            failures.append(
                "RLModule checkpoint file is missing or not a regular file: "
                f"{required}"
            )
            continue
        try:
            size = required.stat().st_size
        except OSError as exc:
            failures.append(f"could not stat RLModule checkpoint file {required}: {exc}")
            continue
        if size <= 0:
            failures.append(f"RLModule checkpoint file is empty: {required}")
    return failures


def _validate_config(config: GateConfig) -> list[str]:
    failures = _validate_rl_module_checkpoint(config.checkpoint)
    reserve_mode = _reserve_mode(config)
    if reserve_mode == RESERVE_MODE_LEGACY_SCALAR:
        reserve_limit = _finite_number(config.max_reserve_norm_nm)
        if reserve_limit is None or reserve_limit < 0.0:
            failures.append("--max-reserve-norm-nm must be finite and non-negative")
    elif reserve_mode == RESERVE_MODE_CONDITION_MATCHED:
        assert config.reserve_reference_checkpoint is not None
        assert config.reserve_reference_summaries is not None
        reference_failures = _validate_rl_module_checkpoint(
            config.reserve_reference_checkpoint
        )
        failures.extend(
            f"reserve reference checkpoint: {failure}"
            for failure in reference_failures
        )
        if len(config.reserve_reference_summaries) != 4:
            failures.append(
                "--reserve-reference-summaries requires exactly four summaries"
            )
    else:
        scalar = config.max_reserve_norm_nm is not None
        checkpoint = config.reserve_reference_checkpoint is not None
        summaries = config.reserve_reference_summaries is not None
        if scalar and (checkpoint or summaries):
            failures.append(
                "legacy --max-reserve-norm-nm is mutually exclusive with the "
                "condition-matched reserve reference"
            )
        elif checkpoint != summaries:
            failures.append(
                "condition-matched reserve mode requires both "
                "--reserve-reference-checkpoint and "
                "--reserve-reference-summaries"
            )
        else:
            failures.append(
                "choose condition-matched reserve references (recommended) or "
                "legacy --max-reserve-norm-nm"
            )
    sigma = _finite_number(config.expected_sigma)
    if sigma is None or sigma <= 0.0:
        failures.append("--expected-sigma must be finite and positive")
    timeout = _finite_number(config.rollout_timeout_s)
    if timeout is None or timeout <= 0.0:
        failures.append("--rollout-timeout-s must be finite and positive")
    if (
        type(config.expected_training_iteration) is not int
        or config.expected_training_iteration <= 0
    ):
        failures.append(
            "--expected-training-iteration must be a positive integer"
        )

    offsets = tuple(_finite_number(value) for value in config.start_offsets_s)
    if len(offsets) != 3 or any(value is None for value in offsets):
        failures.append("exactly three finite --start-offsets-s values are required")
    else:
        low, nominal, high = (float(value) for value in offsets if value is not None)
        if not (low < nominal < high):
            failures.append("start offsets must be strictly increasing")
        if not (
            math.isclose(nominal - low, 0.2, rel_tol=0.0, abs_tol=OFFSET_TOLERANCE_S)
            and math.isclose(
                high - nominal, 0.2, rel_tol=0.0, abs_tol=OFFSET_TOLERANCE_S
            )
        ):
            failures.append("start offsets must be nominal -0.20, nominal, nominal +0.20")
        if len({_offset_metric_label(value) for value in (low, nominal, high)}) != 3:
            failures.append("start offsets collide after six-decimal metric labeling")

    if config.run_rollouts:
        if config.deterministic_summaries is not None or config.stochastic_summary is not None:
            failures.append("summary paths cannot be combined with --run-rollouts")
        if not config.rollout_script.is_file():
            failures.append(f"rollout script not found: {config.rollout_script}")
        executable = config.python_executable
        if not executable.strip():
            failures.append("--python-executable cannot be empty")
        elif any(separator in executable for separator in (os.sep, os.altsep) if separator):
            if not Path(executable).expanduser().is_file():
                failures.append(f"Python executable not found: {executable}")
        elif shutil.which(executable) is None:
            failures.append(f"Python executable not found on PATH: {executable}")
    else:
        if config.deterministic_summaries is None:
            failures.append("classification mode requires three --deterministic-summaries")
        if config.stochastic_summary is None:
            failures.append("classification mode requires --stochastic-summary")
    if config.train_iterations is None:
        failures.append("--train-iterations is required for the PPO update gate")
    else:
        training_path = config.train_iterations.expanduser().resolve(strict=False)
        checkpoint_run_dir = config.checkpoint.expanduser().resolve(strict=False).parent
        if training_path.parent != checkpoint_run_dir:
            failures.append(
                "--train-iterations must be a sibling of the checkpoint under test"
            )
    return failures


def _summary_path(value: Path) -> Path:
    path = value.expanduser().resolve(strict=False)
    return path / "rollout_summary.json" if path.is_dir() else path


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _checkpoint_artifact_report(checkpoint: Path) -> dict[str, Any]:
    path = checkpoint.expanduser().resolve(strict=False)
    failures = _validate_rl_module_checkpoint(path)
    files: dict[str, Any] = {}
    if not failures:
        for filename in REQUIRED_RL_MODULE_FILES:
            artifact = path / filename
            try:
                files[filename] = {
                    "path": _portable_path(artifact),
                    "sha256": _sha256(artifact),
                }
            except OSError as exc:
                failures.append(f"could not hash {artifact}: {exc}")
    return {
        "path": _portable_path(path),
        "status": "PASS" if not failures else "FAIL",
        "failures": failures,
        "files": files,
    }


def _classify_reserve_reference(
    config: GateConfig,
    specs: Sequence[RolloutSpec],
) -> tuple[dict[str, Any] | None, list[dict[str, Any] | None]]:
    """Validate H0 case-by-case and derive trusted non-regression caps."""

    caps: list[dict[str, Any] | None] = [None] * len(specs)
    if _reserve_mode(config) != RESERVE_MODE_CONDITION_MATCHED:
        return None, caps

    assert config.reserve_reference_checkpoint is not None
    assert config.reserve_reference_summaries is not None
    checkpoint_report = _checkpoint_artifact_report(
        config.reserve_reference_checkpoint
    )
    supplied = list(config.reserve_reference_summaries)
    results: list[dict[str, Any]] = []

    for index, spec in enumerate(specs):
        if index >= len(supplied):
            results.append(
                {
                    "name": spec.name,
                    "status": "FAIL",
                    "summary_path": None,
                    "checks": [],
                    "failed_checks": ["reserve_reference_summary_assignment"],
                    "error": "reserve reference summary was not supplied",
                }
            )
            continue

        path = _summary_path(supplied[index])
        try:
            summary = _read_json_object(path)
            reference_cap = _finite_number(summary.get("reserve_norm_max_nm"))
            valid_cap = reference_cap is not None and reference_cap >= 0.0
            result = classify_rollout_summary(
                summary,
                expected_checkpoint=config.reserve_reference_checkpoint,
                spec=spec,
                expected_sigma=config.expected_sigma,
                max_reserve_norm_nm=(reference_cap if valid_cap else 0.0),
                # H0 is classified against its own exact value.  The numerical
                # tolerance is reserved for the later candidate comparison.
                reserve_numerical_tolerance_nm=0.0,
            )
            result["summary_path"] = _portable_path(path)
            result["summary_sha256"] = _sha256(path)
            if not valid_cap:
                result["status"] = "FAIL"
                if "reserve_reference_cap" not in result["failed_checks"]:
                    result["failed_checks"].append("reserve_reference_cap")
            if valid_cap:
                assert reference_cap is not None
                tolerance = _reserve_numerical_tolerance_nm(reference_cap)
                effective_limit = _finite_number(reference_cap + tolerance)
                if effective_limit is None:
                    result["status"] = "FAIL"
                    result["failed_checks"].append(
                        "reserve_reference_effective_limit"
                    )
                result["condition_matched_cap"] = {
                    "summary_path": _portable_path(path),
                    "summary_sha256": result["summary_sha256"],
                    "reference_cap_nm": reference_cap,
                    "numerical_tolerance_nm": tolerance,
                    "maximum_candidate_value_inclusive_nm": effective_limit,
                    "tolerance_formula": "max(1e-6 Nm, 1e-9 * T_c)",
                }
                if result["status"] == "PASS":
                    caps[index] = dict(result["condition_matched_cap"])
        except (OSError, ValueError) as exc:
            result = {
                "name": spec.name,
                "status": "FAIL",
                "summary_path": _portable_path(path),
                "error": str(exc),
                "checks": [],
                "failed_checks": ["reserve_reference_summary_read"],
            }
        results.append(result)

    passed = (
        checkpoint_report["status"] == "PASS"
        and len(supplied) == len(specs)
        and all(result.get("status") == "PASS" for result in results)
    )
    return (
        {
            "mode": RESERVE_MODE_CONDITION_MATCHED,
            "status": "PASS" if passed else "FAIL",
            "checkpoint": checkpoint_report,
            "rollouts": results,
            "case_caps": [cap for cap in caps],
            "tolerance_formula": "max(1e-6 Nm, 1e-9 * T_c)",
            "percent_tolerance": None,
        },
        caps,
    )


def _rollout_command(
    config: GateConfig,
    spec: RolloutSpec,
    output_dir: Path,
) -> list[str]:
    return [
        config.python_executable,
        _portable_path(config.rollout_script),
        "--checkpoint",
        _portable_path(config.checkpoint),
        "--output-dir",
        _portable_path(output_dir),
        "--episode-duration",
        "5.0",
        "--max-steps",
        str(EXPECTED_STEPS),
        "--episode-start-offset-s",
        repr(spec.offset_s),
        "--action-mode",
        "absolute",
        "--action-selection",
        spec.action_selection,
        "--seed",
        str(spec.seed),
        # The rollout's own cross-platform watchdog owns the timeout and process-tree
        # cleanup.  An outer subprocess timeout would kill only this supervisor and
        # could leave its separately grouped OpenSim worker orphaned.
        "--run-timeout-s",
        str(config.rollout_timeout_s),
        "--no-record-outputs",
        "--no-record-policy-trace",
        "--no-progress",
    ]


def _write_report(path: Path, report: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.tmp")
    temporary.write_text(
        json.dumps(report, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    os.replace(temporary, path)


def evaluate_gate(
    config: GateConfig,
    *,
    command_runner: Callable[..., Any] = subprocess.run,
) -> dict[str, Any]:
    """Run or classify the matrix and always emit ``robust_gate.json``."""

    output_dir = config.output_dir.expanduser().resolve(strict=False)
    report_path = output_dir / "robust_gate.json"
    specs = _rollout_specs(config.start_offsets_s)
    reserve_mode = _reserve_mode(config)
    config_failures = _validate_config(config)
    reserve_reference, reserve_caps = _classify_reserve_reference(config, specs)
    reserve_reference_failed = (
        reserve_reference is not None
        and reserve_reference.get("status") != "PASS"
    )
    invocations: list[dict[str, Any]] = []
    summary_paths: list[Path | None] = [None] * len(specs)

    if config.run_rollouts and not config_failures and not reserve_reference_failed:
        rollout_root = output_dir / "rollouts"
        rollout_root.mkdir(parents=True, exist_ok=True)
        for index, spec in enumerate(specs):
            case_dir = rollout_root / spec.name
            command = _rollout_command(config, spec, case_dir)
            invocation: dict[str, Any] = {
                "name": spec.name,
                "command": command,
                "output_dir": _portable_path(case_dir),
                "status": "FAIL",
            }
            if case_dir.exists():
                invocation["error"] = (
                    "refusing to reuse an existing rollout directory; choose a new "
                    f"--output-dir: {case_dir}"
                )
                invocations.append(invocation)
                continue
            try:
                completed = command_runner(
                    command,
                    check=False,
                )
                returncode = int(completed.returncode)
                invocation["returncode"] = returncode
                invocation["status"] = "PASS" if returncode == 0 else "FAIL"
                if returncode != 0:
                    invocation["error"] = f"rollout process returned {returncode}"
            except (OSError, ValueError) as exc:
                invocation["error"] = f"could not launch rollout: {exc}"
            invocations.append(invocation)
            summary_paths[index] = case_dir / "rollout_summary.json"
    elif not config.run_rollouts and not config_failures:
        assert config.deterministic_summaries is not None
        assert config.stochastic_summary is not None
        supplied = (*config.deterministic_summaries, config.stochastic_summary)
        summary_paths = [_summary_path(path) for path in supplied]

    rollout_results: list[dict[str, Any]] = []
    for index, (spec, path) in enumerate(zip(specs, summary_paths)):
        if path is None:
            rollout_results.append(
                {
                    "name": spec.name,
                    "status": "FAIL",
                    "summary_path": None,
                    "error": "rollout summary was not assigned",
                    "checks": [],
                    "failed_checks": ["summary_assignment"],
                }
            )
            continue
        if (
            reserve_mode == RESERVE_MODE_CONDITION_MATCHED
            and reserve_caps[index] is None
        ):
            rollout_results.append(
                {
                    "name": spec.name,
                    "status": "FAIL",
                    "summary_path": _portable_path(path),
                    "error": (
                        "condition-matched reserve reference did not pass for "
                        f"{spec.name}"
                    ),
                    "checks": [],
                    "failed_checks": [
                        "condition_matched_reserve_reference"
                    ],
                    "reserve_contract": {
                        "mode": RESERVE_MODE_CONDITION_MATCHED,
                        "reference_available": False,
                    },
                }
            )
            continue
        try:
            summary = _read_json_object(path)
            if reserve_mode == RESERVE_MODE_LEGACY_SCALAR:
                assert config.max_reserve_norm_nm is not None
                reserve_cap = config.max_reserve_norm_nm
                reserve_tolerance = 0.0
                reserve_provenance: dict[str, Any] = {
                    "mode": RESERVE_MODE_LEGACY_SCALAR,
                    "scalar_cap_nm": reserve_cap,
                }
            elif reserve_mode == RESERVE_MODE_CONDITION_MATCHED:
                cap = reserve_caps[index]
                assert cap is not None
                reserve_cap = float(cap["reference_cap_nm"])
                reserve_tolerance = float(cap["numerical_tolerance_nm"])
                reserve_provenance = {
                    "mode": RESERVE_MODE_CONDITION_MATCHED,
                    "reference_summary_path": cap["summary_path"],
                    "reference_summary_sha256": cap["summary_sha256"],
                    "reference_cap_nm": reserve_cap,
                    "numerical_tolerance_nm": reserve_tolerance,
                    "maximum_candidate_value_inclusive_nm": cap[
                        "maximum_candidate_value_inclusive_nm"
                    ],
                    "tolerance_formula": cap["tolerance_formula"],
                }
            else:
                raise ValueError("reserve gate mode is invalid")
            result = classify_rollout_summary(
                summary,
                expected_checkpoint=config.checkpoint,
                spec=spec,
                expected_sigma=config.expected_sigma,
                max_reserve_norm_nm=reserve_cap,
                reserve_numerical_tolerance_nm=reserve_tolerance,
            )
            result["reserve_contract"].update(reserve_provenance)
            result["summary_path"] = _portable_path(path)
            result["summary_sha256"] = _sha256(path)
        except (OSError, ValueError) as exc:
            result = {
                "name": spec.name,
                "status": "FAIL",
                "summary_path": _portable_path(path),
                "error": str(exc),
                "checks": [],
                "failed_checks": ["summary_read"],
            }
        rollout_results.append(result)

    training_result: dict[str, Any] | None = None
    if config.train_iterations is not None:
        training_result = classify_training_iterations(
            config.train_iterations.expanduser().resolve(strict=False),
            start_offsets_s=config.start_offsets_s,
            expected_training_iteration=config.expected_training_iteration,
        )

    invocation_failures = [
        invocation["name"]
        for invocation in invocations
        if invocation.get("status") != "PASS"
    ]
    rollout_failures = [
        result["name"]
        for result in rollout_results
        if result.get("status") != "PASS"
    ]
    training_failed = (
        training_result is not None and training_result.get("status") != "PASS"
    )
    passed = not (
        config_failures
        or reserve_reference_failed
        or invocation_failures
        or rollout_failures
        or training_failed
    )

    if reserve_mode == RESERVE_MODE_LEGACY_SCALAR:
        reserve_gate_contract: dict[str, Any] = {
            "mode": RESERVE_MODE_LEGACY_SCALAR,
            "recommended": False,
            "maximum_reserve_norm_nm_inclusive": config.max_reserve_norm_nm,
            "numerical_tolerance_nm": 0.0,
        }
    elif reserve_mode == RESERVE_MODE_CONDITION_MATCHED:
        reserve_gate_contract = {
            "mode": RESERVE_MODE_CONDITION_MATCHED,
            "recommended": True,
            "comparison": "candidate_case <= H0_same_case + numerical_tolerance",
            "reference_checkpoint": (
                _portable_path(config.reserve_reference_checkpoint)
                if config.reserve_reference_checkpoint is not None
                else None
            ),
            "case_caps": (
                reserve_reference.get("case_caps")
                if reserve_reference is not None
                else None
            ),
            "tolerance_formula": "max(1e-6 Nm, 1e-9 * T_c)",
            "percent_tolerance": None,
        }
    else:
        reserve_gate_contract = {
            "mode": "invalid",
            "recommended": False,
        }

    report: dict[str, Any] = {
        "schema_version": GATE_SCHEMA_VERSION,
        "gate": "robust_ppo_gate",
        "status": "PASS" if passed else "FAIL",
        "ok": passed,
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "mode": "run_rollouts" if config.run_rollouts else "classify_summaries",
        "reserve_gate_mode": reserve_mode,
        "checkpoint": _portable_path(config.checkpoint),
        "output": _portable_path(report_path),
        "contract": {
            "action_mode": "absolute",
            "start_offsets_s": list(config.start_offsets_s),
            "worst_case": {
                "offset_s": config.start_offsets_s[2],
                "action_selection": "stochastic",
                "seed": EXPECTED_SEED,
                "sigma": config.expected_sigma,
            },
            "steps": EXPECTED_STEPS,
            "end_reason": "episode_time_limit",
            "terminated": False,
            "truncated": True,
            "minimum_valid_cycles": EXPECTED_MIN_CYCLES,
            "penetration_limit_m_strict": PENETRATION_LIMIT_M,
            "required_penetration_samples": EXPECTED_STEPS,
            "maximum_action_clipped_steps": 0,
            "actor_features": EXPECTED_ACTOR_FEATURES,
            "observation_features": EXPECTED_OBSERVATION_FEATURES,
            "maximum_reserve_norm_nm_inclusive": (
                config.max_reserve_norm_nm
                if reserve_mode == RESERVE_MODE_LEGACY_SCALAR
                else None
            ),
            "reserve_gate": reserve_gate_contract,
            "training": {
                "iteration": config.expected_training_iteration,
                "maximum_mean_kl_loss_inclusive": KL_LIMIT,
                "mean_kl_loss_scope": "legacy_last_minibatch",
                "max_minibatch_mean_kl_loss": {
                    "scope": "entire_update",
                    "minimum_inclusive": 0.0,
                    "maximum_inclusive": KL_LIMIT,
                },
                "min_minibatch_mean_kl_loss": {
                    "scope": "entire_update",
                    "minimum_inclusive": KL_MIN_NUMERICAL_TOLERANCE,
                    "minimum_rationale": "floating-point numerical tolerance",
                },
                "kl_minibatch_count": EXPECTED_KL_MINIBATCH_COUNT,
                "kl_nonfinite_count": 0,
                "exact_steps_per_start": EXPECTED_START_STEPS,
                "post_gae_compacted_real_steps": (
                    EXPECTED_START_STEPS * len(config.start_offsets_s)
                ),
                "single_epoch_contract": True,
                "start_interleaving": {
                    "interleaved_rows": (
                        EXPECTED_START_STEPS * len(config.start_offsets_s)
                    ),
                    "interleaved_start_conditions": len(
                        config.start_offsets_s
                    ),
                    "interleaved_rows_per_start": EXPECTED_START_STEPS,
                    "maximum_start_run_length": 1,
                },
                "module_steps_trained": (
                    EXPECTED_START_STEPS * len(config.start_offsets_s)
                ),
            },
        },
        "configuration_failures": config_failures,
        "reserve_reference": reserve_reference,
        "reserve_reference_failed": reserve_reference_failed,
        "invocations": invocations,
        "rollouts": rollout_results,
        "training": training_result,
        "failed_rollouts": rollout_failures,
        "failed_invocations": invocation_failures,
        "checkpoint_promoted": False,
        "checkpoint_copied": False,
    }
    _write_report(report_path, report)
    return report


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    reserve_mode = parser.add_mutually_exclusive_group(required=True)
    reserve_mode.add_argument(
        "--max-reserve-norm-nm",
        type=float,
        help=(
            "Legacy scalar inclusive upper bound for reserve_norm_max_nm. "
            "Prefer --reserve-reference-checkpoint."
        ),
    )
    reserve_mode.add_argument(
        "--reserve-reference-checkpoint",
        type=Path,
        help=(
            "Recommended H0 rl_module_last used for condition-matched reserve "
            "non-regression. Requires --reserve-reference-summaries."
        ),
    )
    parser.add_argument(
        "--reserve-reference-summaries",
        type=Path,
        nargs=4,
        metavar="SUMMARY",
        help=(
            "Four H0 summaries ordered deterministic -0.20, nominal, +0.20, "
            "then stochastic +0.20 seed 123. Mutually alternative to the "
            "legacy scalar reserve cap."
        ),
    )
    parser.add_argument("--expected-sigma", type=float, default=0.005)
    parser.add_argument(
        "--start-offsets-s",
        type=float,
        nargs=3,
        default=DEFAULT_START_OFFSETS_S,
        metavar="SECONDS",
        help="Absolute start offsets: nominal-0.20, nominal, nominal+0.20.",
    )
    parser.add_argument(
        "--train-iterations",
        type=Path,
        required=True,
        help=(
            "Candidate train_iterations.jsonl; must be beside the checkpoint and "
            "pass iteration/KL/raw-balance/post-GAE-compaction audits."
        ),
    )
    parser.add_argument(
        "--expected-training-iteration",
        type=int,
        default=EXPECTED_TRAINING_ITERATION,
        help=(
            "Logical iteration to select uniquely from --train-iterations "
            f"(default: {EXPECTED_TRAINING_ITERATION})."
        ),
    )
    parser.add_argument(
        "--run-rollouts",
        action="store_true",
        help="Launch the four rollout_eval processes serially.",
    )
    parser.add_argument(
        "--deterministic-summaries",
        type=Path,
        nargs=3,
        metavar="SUMMARY",
        help="Existing summaries ordered minus0.20, nominal, plus0.20.",
    )
    parser.add_argument(
        "--stochastic-summary",
        type=Path,
        help="Existing +0.20 stochastic seed-123 summary.",
    )
    parser.add_argument(
        "--python-executable",
        default=sys.executable,
        help="Python used to launch rollout_eval.py in --run-rollouts mode.",
    )
    parser.add_argument(
        "--rollout-script",
        type=Path,
        default=DEFAULT_ROLLOUT_SCRIPT,
    )
    parser.add_argument("--rollout-timeout-s", type=float, default=900.0)
    return parser


def _config_from_args(args: argparse.Namespace) -> GateConfig:
    deterministic = (
        tuple(args.deterministic_summaries)
        if args.deterministic_summaries is not None
        else None
    )
    return GateConfig(
        checkpoint=args.checkpoint,
        output_dir=args.output_dir,
        max_reserve_norm_nm=args.max_reserve_norm_nm,
        reserve_reference_checkpoint=args.reserve_reference_checkpoint,
        reserve_reference_summaries=(
            tuple(args.reserve_reference_summaries)
            if args.reserve_reference_summaries is not None
            else None
        ),
        expected_sigma=args.expected_sigma,
        start_offsets_s=tuple(args.start_offsets_s),
        train_iterations=args.train_iterations,
        expected_training_iteration=args.expected_training_iteration,
        run_rollouts=args.run_rollouts,
        deterministic_summaries=deterministic,
        stochastic_summary=args.stochastic_summary,
        python_executable=args.python_executable,
        rollout_script=args.rollout_script,
        rollout_timeout_s=args.rollout_timeout_s,
    )


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    report = evaluate_gate(_config_from_args(args))
    print(json.dumps(report, indent=2, sort_keys=True, allow_nan=False))
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
