"""One-shot gate-derived continuation of the frozen W1024 uniform dry fit.

The process first reproduces, in memory, the exact terminal state of
``dry_fit_w1024_r6_residual.py`` and refuses to continue unless the frozen
state digest, prediction digest, and LBFGS closure count all match.  It then
runs one finite continuation phase with a loss derived symmetrically from the
unchanged H0 gates:

``L = L_preserve + L_worst_group + L_tail + L_reset``.

``L_preserve`` is the reset-x3 weighted MSE divided by the squared global RMSE
limit. ``L_worst_group`` is a smooth worst-case over every RMSE-bearing gate
group (global, six base cases, six observer cases, the R4 recovery group, and
observer-plus-late), each divided by the 0.006 limit. ``L_tail`` is a smooth
maximum over every scalar absolute error divided by 0.060. ``L_reset`` is the
same construction over reset scalar errors divided by 0.003.  Every term has
coefficient one.  Both smooth maxima use temperature 0.05; a softplus safety
margin targets 0.90 of each frozen gate.  These constants and the optimizer
schedule are fixed in this source before execution.

Only the terminal optimizer state is evaluated.  There is no sweep, retry,
hard-coded row/case coefficient, best-state selection, early stopping, repair,
teacher query, environment access, checkpoint publication, or promotion.  A
single byte-determinism replica is run only if the primary state passes every
unchanged numerical/semantic gate and all W1024 compatibility checks.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import sys
import time
from pathlib import Path
from typing import Any, Mapping

import numpy as np


DIAGNOSTIC_ROOT = Path(__file__).resolve().parent
if str(DIAGNOSTIC_ROOT) not in sys.path:
    sys.path.insert(0, str(DIAGNOSTIC_ROOT))

import dry_fit_w1024_r6_residual as uniform  # noqa: E402


forensic = uniform.forensic
r9 = uniform.r9
REPO_ROOT = uniform.REPO_ROOT
R9_ROOT = uniform.R9_ROOT

STATUS = "COMPLETE_H0_V12R10_W1024_GATE_ALIGNED_DRY_FIT"
STRATEGY_ID = "V12R10_STANDARD_W1024_UNIFORM_TERMINAL_GATE_ALIGNED_V1"

FROZEN_UNIFORM_RESULT = (
    DIAGNOSTIC_ROOT / "results" / "w1024_r6_residual_reset3_dry_fit.json"
)
EXPECTED_UNIFORM_STATE_DIGEST = (
    "52aee29da6db7535e8fcfe14f66fa1c7eaa0b95a0ac74fbf5c73b25a1c8fe167"
)
EXPECTED_UNIFORM_PREDICTION_DIGEST = (
    "2aa5c64704163b949525a28a4fd9d3e6688e6f2973fb4fa137fa7ff01f4da48d"
)
EXPECTED_UNIFORM_CLOSURES = 3_072
EXPECTED_UNIFORM_TERMINAL_LOSS = 3.0605827798686175e-05

# Gate-derived loss constants.  These are dimensionless and apply symmetrically.
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

# Single preregistered continuation schedule.
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

OFFLINE_H0_QUERY_COUNT = 0


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _strict_json(path: Path) -> dict[str, Any]:
    value = json.loads(
        path.read_text(encoding="utf-8"),
        parse_constant=lambda token: (_ for _ in ()).throw(
            ValueError(f"non-finite JSON constant: {token}")
        ),
    )
    if not isinstance(value, dict):
        raise TypeError(f"expected JSON object: {path}")
    return value


def _gate_adamw_rate(epoch: int) -> float:
    if type(epoch) is not int or not 1 <= epoch <= GATE_ADAMW_EPOCHS:
        raise ValueError(f"epoch outside preregistered schedule: {epoch!r}")
    if epoch <= GATE_ADAMW_BOUNDARIES[0]:
        return GATE_ADAMW_RATES[0]
    if epoch <= GATE_ADAMW_BOUNDARIES[1]:
        return GATE_ADAMW_RATES[1]
    return GATE_ADAMW_RATES[2]


def _gate_group_indices(arrays: Mapping[str, np.ndarray]) -> dict[str, np.ndarray]:
    strata = arrays["stratum_ids"].astype(str)
    steps = arrays["step_indices"]
    groups: dict[str, np.ndarray] = {
        "global": np.arange(len(strata), dtype=np.int64),
    }
    for case_id in r9.contract.COLLECTION_CASE_IDS:
        groups[f"base::{case_id}"] = np.flatnonzero(
            strata == f"base::{case_id}"
        ).astype(np.int64)
    for case_id in r9.contract.COLLECTION_CASE_IDS:
        groups[f"observer::{case_id}"] = np.flatnonzero(
            strata == f"observer::{case_id}"
        ).astype(np.int64)
    groups["r4_failure::deterministic_offset_plus_0p20"] = np.flatnonzero(
        strata == "r4_failure::deterministic_offset_plus_0p20"
    ).astype(np.int64)
    groups["observer_plus_late"] = np.flatnonzero(
        (strata == "observer::deterministic_offset_plus_0p20") & (steps >= 140)
    ).astype(np.int64)
    expected = 2 + 2 * len(r9.contract.COLLECTION_CASE_IDS) + 1
    if len(groups) != expected or any(len(index) == 0 for index in groups.values()):
        raise RuntimeError("gate-derived group construction is incomplete")
    return groups


def _smooth_margin(value: Any) -> tuple[Any, Any]:
    """Return smooth max and smooth positive excess above the safety margin."""
    import torch

    temperature = SMOOTH_MAX_TEMPERATURE
    smooth_max = temperature * torch.logsumexp(value / temperature, dim=0)
    excess = temperature * torch.nn.functional.softplus(
        (smooth_max - SAFETY_MARGIN_FRACTION) / temperature
    )
    return smooth_max, excess


def _reproduce_uniform_terminal(
    *,
    arrays: Mapping[str, np.ndarray],
    sample_weights: np.ndarray,
    normalization: Any,
    r6_state: Mapping[str, Any],
    r9_state: Mapping[str, Any],
) -> dict[str, Any]:
    """Execute the frozen uniform fit exactly and return its in-memory residual."""
    import torch

    raw = arrays["observations"]
    normalized = r9.v11.normalized_observations(raw, normalization)
    fixed_predictions_np = np.ascontiguousarray(
        r9.v11._state_logits(r6_state, raw)[:, : uniform.ACTION_DIM],  # noqa: SLF001
        dtype=np.float32,
    )
    torch.manual_seed(uniform.SEED)
    residual = uniform._new_normalized_residual_model(  # noqa: SLF001
        r9_state, normalization
    )
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
        initial_residual = residual(x)
        initial_combined = fixed + initial_residual
        initial_loss, _ = objective()
    if not uniform._bytes_equal(initial_combined.cpu().numpy(), fixed_predictions_np):  # noqa: SLF001
        raise RuntimeError("uniform replay did not initialize exactly at R6")

    adamw = torch.optim.AdamW(
        residual.parameters(),
        lr=uniform.ADAMW_RATES[0],
        weight_decay=uniform.ADAMW_WEIGHT_DECAY,
    )
    for epoch in range(1, uniform.ADAMW_EPOCHS + 1):
        rate = uniform._adamw_rate(epoch)  # noqa: SLF001
        for group in adamw.param_groups:
            group["lr"] = rate
        adamw.zero_grad(set_to_none=True)
        loss, _ = objective()
        if not torch.isfinite(loss):
            raise RuntimeError(f"non-finite uniform replay loss at epoch {epoch}")
        loss.backward()
        torch.nn.utils.clip_grad_norm_(
            residual.parameters(), uniform.GRADIENT_CLIP_NORM
        )
        adamw.step()
        with torch.no_grad():
            residual[0].weight[:, r9.contract.DISABLED_CLOCK_COLUMNS] = 0.0
        # Preserve the original diagnostic's evaluation call pattern exactly.
        if epoch in {1, 100, 250, 500, 1_000, 1_500, 2_000, 2_500}:
            with torch.no_grad():
                objective()

    lbfgs = torch.optim.LBFGS(
        residual.parameters(),
        lr=uniform.LBFGS_LR,
        max_iter=uniform.LBFGS_MAX_ITER,
        max_eval=uniform.LBFGS_MAX_EVAL,
        tolerance_grad=uniform.LBFGS_TOLERANCE_GRAD,
        tolerance_change=uniform.LBFGS_TOLERANCE_CHANGE,
        history_size=uniform.LBFGS_HISTORY_SIZE,
        line_search_fn="strong_wolfe",
    )
    closure_calls = 0

    def closure() -> Any:
        nonlocal closure_calls
        lbfgs.zero_grad(set_to_none=True)
        value, _ = objective()
        if not torch.isfinite(value):
            raise RuntimeError(
                f"non-finite uniform replay loss at closure {closure_calls + 1}"
            )
        value.backward()
        closure_calls += 1
        return value

    lbfgs.step(closure)
    with torch.no_grad():
        residual[0].weight[:, r9.contract.DISABLED_CLOCK_COLUMNS] = 0.0
        terminal_loss, normalized_prediction_tensor = objective()

    candidate_state = uniform._pack_w1024_state(  # noqa: SLF001
        residual=residual,
        r6_state=r6_state,
        normalization=normalization,
    )
    predictions = np.ascontiguousarray(
        r9.v11._state_logits(candidate_state, raw)[:, : uniform.ACTION_DIM],  # noqa: SLF001
        dtype=np.float32,
    )
    normalized_predictions = np.ascontiguousarray(
        normalized_prediction_tensor.cpu().numpy(), dtype=np.float32
    )
    state_digest = uniform._state_digest(candidate_state)  # noqa: SLF001
    prediction_digest = uniform._prediction_digest(predictions)  # noqa: SLF001
    fold_max = float(
        np.max(
            np.abs(
                predictions.astype(np.float64)
                - normalized_predictions.astype(np.float64)
            )
        )
    )
    checks = {
        "state_digest_exact": state_digest == EXPECTED_UNIFORM_STATE_DIGEST,
        "prediction_digest_exact": prediction_digest
        == EXPECTED_UNIFORM_PREDICTION_DIGEST,
        "closure_count_exact": closure_calls == EXPECTED_UNIFORM_CLOSURES,
        "terminal_loss_exact": float(terminal_loss.detach().cpu())
        == EXPECTED_UNIFORM_TERMINAL_LOSS,
        "normalization_fold_within_tolerance": fold_max
        <= uniform.FOLD_NUMERICAL_TOLERANCE,
    }
    if not all(checks.values()):
        raise RuntimeError(
            "frozen W1024 uniform terminal reproduction failed: "
            + repr(sorted(name for name, passed in checks.items() if not passed))
        )
    return {
        "residual": residual,
        "x": x,
        "targets": targets,
        "fixed": fixed,
        "weights": weights,
        "weight_sum": weight_sum,
        "attestation": {
            "passed": True,
            "checks": checks,
            "initial_loss": float(initial_loss.detach().cpu()),
            "terminal_loss": float(terminal_loss.detach().cpu()),
            "lbfgs_closure_calls": closure_calls,
            "candidate_state_digest": state_digest,
            "candidate_predictions_sha256": prediction_digest,
            "normalization_fold_max_abs_difference": fold_max,
        },
    }


def _fit_once(
    *,
    label: str,
    arrays: Mapping[str, np.ndarray],
    sample_weights: np.ndarray,
    normalization: Any,
    r6_state: Mapping[str, Any],
    r9_state: Mapping[str, Any],
    source_module: Any,
) -> dict[str, Any]:
    import torch

    previous_threads = torch.get_num_threads()
    previous_deterministic = torch.are_deterministic_algorithms_enabled()
    torch.set_num_threads(uniform.TORCH_THREADS)
    torch.use_deterministic_algorithms(True)
    started = time.monotonic()
    try:
        replay = _reproduce_uniform_terminal(
            arrays=arrays,
            sample_weights=sample_weights,
            normalization=normalization,
            r6_state=r6_state,
            r9_state=r9_state,
        )
        residual = replay.pop("residual")
        x = replay.pop("x")
        targets = replay.pop("targets")
        fixed = replay.pop("fixed")
        weights = replay.pop("weights")
        weight_sum = replay.pop("weight_sum")
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

        history: list[dict[str, Any]] = []

        def snapshot(stage: str, index: int) -> dict[str, Any]:
            with torch.no_grad():
                loss, prediction, terms = objective()
            values = prediction.detach().cpu().numpy().astype(np.float64)
            row = {
                "stage": stage,
                "index": int(index),
                "loss": float(loss.detach().cpu()),
                "prediction_abs_max": float(np.max(np.abs(values))),
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
            history.append(row)
            print(json.dumps({"fit": label, **row}, sort_keys=True), flush=True)
            return row

        snapshot("gate_initial_uniform_terminal", 0)
        adamw = torch.optim.AdamW(
            residual.parameters(),
            lr=GATE_ADAMW_RATES[0],
            weight_decay=GATE_ADAMW_WEIGHT_DECAY,
        )
        milestones = {1, 100, 250, 500, 750, 1_000, 1_250, 1_500}
        for epoch in range(1, GATE_ADAMW_EPOCHS + 1):
            rate = _gate_adamw_rate(epoch)
            for group in adamw.param_groups:
                group["lr"] = rate
            adamw.zero_grad(set_to_none=True)
            loss, _, _ = objective()
            if not torch.isfinite(loss):
                raise RuntimeError(f"non-finite gate loss at AdamW epoch {epoch}")
            loss.backward()
            torch.nn.utils.clip_grad_norm_(
                residual.parameters(), GATE_GRADIENT_CLIP_NORM
            )
            adamw.step()
            with torch.no_grad():
                residual[0].weight[:, r9.contract.DISABLED_CLOCK_COLUMNS] = 0.0
            if epoch in milestones:
                row = snapshot("gate_adamw", epoch)
                row["learning_rate"] = rate

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
            value, _, terms = objective()
            if not torch.isfinite(value):
                raise RuntimeError(
                    f"non-finite gate loss at LBFGS closure {closure_calls + 1}"
                )
            value.backward()
            closure_calls += 1
            if closure_calls in closure_milestones:
                row = {
                    "stage": "gate_lbfgs_closure",
                    "index": closure_calls,
                    "loss": float(value.detach().cpu()),
                    "terms": {
                        name: float(term.detach().cpu())
                        for name, term in terms.items()
                        if name != "group_rmse_fractions"
                    },
                }
                history.append(row)
                print(json.dumps({"fit": label, **row}, sort_keys=True), flush=True)
            return value

        lbfgs.step(closure)
        with torch.no_grad():
            residual[0].weight[:, r9.contract.DISABLED_CLOCK_COLUMNS] = 0.0
            terminal_loss, normalized_prediction_tensor, terminal_terms = objective()

        candidate_state = uniform._pack_w1024_state(  # noqa: SLF001
            residual=residual,
            r6_state=r6_state,
            normalization=normalization,
        )
        raw = arrays["observations"]
        predictions = np.ascontiguousarray(
            r9.v11._state_logits(candidate_state, raw)[  # noqa: SLF001
                :, : uniform.ACTION_DIM
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
        if fold_max > uniform.FOLD_NUMERICAL_TOLERANCE:
            raise RuntimeError(f"W1024 normalization fold drifted by {fold_max}")

        metrics = forensic._metric_payload(predictions, arrays)
        state_audit = uniform._validate_w1024_state(candidate_state)  # noqa: SLF001
        isolation = uniform._tower_isolation_audit(  # noqa: SLF001
            candidate_state, r6_state
        )
        compatibility = uniform._runtime_and_save_reload_audit(  # noqa: SLF001
            source_module=source_module,
            candidate_state=candidate_state,
            observations=raw,
            feature_names=arrays["actor_feature_names"].astype(str).tolist(),
        )
        literal_gate, w1024_gate = uniform._w1024_gate(  # noqa: SLF001
            metrics=metrics, compatibility=compatibility
        )
        terminal_row = {
            "stage": "gate_terminal_final_state",
            "index": closure_calls,
            "loss": float(terminal_loss.detach().cpu()),
            "global_rmse": metrics["global_metrics"]["rmse"],
            "global_max_abs_error": metrics["global_metrics"]["max_abs_error"],
            "reset_max_abs_error": metrics["reset_max_abs_error"],
            "w1024_gate_passed": w1024_gate["passed"],
            "terms": {
                name: float(term.detach().cpu())
                for name, term in terminal_terms.items()
                if name != "group_rmse_fractions"
            },
        }
        history.append(terminal_row)
        print(json.dumps({"fit": label, **terminal_row}, sort_keys=True), flush=True)
        passed = bool(
            w1024_gate["passed"]
            and state_audit["passed"]
            and isolation["passed"]
            and compatibility["passed"]
        )
        return {
            "passed": passed,
            "candidate_state": candidate_state,
            "predictions": predictions,
            "uniform_terminal_reproduction": replay["attestation"],
            "metrics": metrics,
            "literal_frozen_r9_gate": literal_gate,
            "w1024_gate": w1024_gate,
            "state_audit": state_audit,
            "tower_isolation": isolation,
            "runtime_save_reload_warm_start": compatibility,
            "normalization_fold_equivalence": {
                "passed": True,
                "max_abs_difference": fold_max,
                "tolerance": uniform.FOLD_NUMERICAL_TOLERANCE,
            },
            "optimizer": {
                "seed": uniform.SEED,
                "torch_threads": uniform.TORCH_THREADS,
                "deterministic_algorithms": True,
                "full_batch": True,
                "terminal_state_only": True,
                "adamw_epochs": GATE_ADAMW_EPOCHS,
                "adamw_boundaries": list(GATE_ADAMW_BOUNDARIES),
                "adamw_rates": list(GATE_ADAMW_RATES),
                "adamw_weight_decay": GATE_ADAMW_WEIGHT_DECAY,
                "gradient_clip_norm": GATE_GRADIENT_CLIP_NORM,
                "lbfgs_lr": GATE_LBFGS_LR,
                "lbfgs_max_iter": GATE_LBFGS_MAX_ITER,
                "lbfgs_max_eval": GATE_LBFGS_MAX_EVAL,
                "lbfgs_history_size": GATE_LBFGS_HISTORY_SIZE,
                "lbfgs_closure_calls": closure_calls,
                "sweep": False,
                "retry": False,
                "repair": False,
                "early_stopping": False,
                "best_state_selection": False,
            },
            "history": history,
            "terminal_stratum_errors": forensic._stratum_error_audit(  # noqa: SLF001
                predictions, arrays, sample_weights
            ),
            "terminal_reset": forensic._reset_audit(  # noqa: SLF001
                predictions, arrays, sample_weights
            ),
            "terminal_top_error_rows": forensic._top_error_rows(  # noqa: SLF001
                predictions, arrays, sample_weights, limit=20
            ),
            "candidate_state_digest": uniform._state_digest(  # noqa: SLF001
                candidate_state
            ),
            "candidate_predictions_sha256": uniform._prediction_digest(  # noqa: SLF001
                predictions
            ),
            "elapsed_seconds": float(time.monotonic() - started),
        }
    finally:
        torch.use_deterministic_algorithms(previous_deterministic)
        torch.set_num_threads(previous_threads)


def run() -> dict[str, Any]:
    frozen_result = _strict_json(FROZEN_UNIFORM_RESULT)
    frozen_checks = {
        "terminal_fail_not_reclassified": frozen_result.get("passed") is False,
        "state_digest_expected": frozen_result.get("primary", {}).get(
            "candidate_state_digest"
        )
        == EXPECTED_UNIFORM_STATE_DIGEST,
        "prediction_digest_expected": frozen_result.get("primary", {}).get(
            "candidate_predictions_sha256"
        )
        == EXPECTED_UNIFORM_PREDICTION_DIGEST,
        "closure_count_expected": frozen_result.get("primary", {})
        .get("optimizer", {})
        .get("lbfgs_closure_calls")
        == EXPECTED_UNIFORM_CLOSURES,
    }
    if not all(frozen_checks.values()):
        raise RuntimeError("frozen uniform result lock failed")

    (
        arrays,
        weights,
        weight_audit,
        normalization,
        r6_state,
        r9_state,
        source,
    ) = uniform._load_locked_inputs()  # noqa: SLF001
    primary = _fit_once(
        label="primary",
        arrays=arrays,
        sample_weights=weights,
        normalization=normalization,
        r6_state=r6_state,
        r9_state=r9_state,
        source_module=source["module"],
    )
    primary_state = primary.pop("candidate_state")
    primary_predictions = primary.pop("predictions")

    total_fits = 1
    if primary["passed"]:
        repeated = _fit_once(
            label="determinism_replica",
            arrays=arrays,
            sample_weights=weights,
            normalization=normalization,
            r6_state=r6_state,
            r9_state=r9_state,
            source_module=source["module"],
        )
        repeated_state = repeated.pop("candidate_state")
        repeated_predictions = repeated.pop("predictions")
        state_exact = uniform._state_byte_exact(  # noqa: SLF001
            primary_state, repeated_state
        )
        predictions_exact = uniform._bytes_equal(  # noqa: SLF001
            primary_predictions, repeated_predictions
        )
        metrics_exact = primary["metrics"] == repeated["metrics"]
        history_exact = primary["history"] == repeated["history"]
        replica = {
            "executed": True,
            "passed": bool(
                repeated["passed"]
                and state_exact
                and predictions_exact
                and metrics_exact
                and history_exact
            ),
            "state_byte_exact": state_exact,
            "predictions_byte_exact": predictions_exact,
            "metrics_exact": metrics_exact,
            "history_exact": history_exact,
            "candidate_state_digest": repeated["candidate_state_digest"],
            "candidate_predictions_sha256": repeated["candidate_predictions_sha256"],
            "elapsed_seconds": repeated["elapsed_seconds"],
        }
        total_fits = 2
    else:
        replica = {
            "executed": False,
            "passed": None,
            "reason": "preregistered rule: stop after first failed terminal fit",
        }

    passed = bool(primary["passed"] and replica.get("passed") is True)
    explicitly_tracked = (
        "base::deterministic_offset_minus_0p20",
        "observer::deterministic_offset_nominal",
        "observer::stochastic_nominal_seed_127",
        "observer::stochastic_nominal_seed_128",
    )
    gate_groups = _gate_group_indices(arrays)
    return {
        "status": STATUS,
        "passed": passed,
        "decision": (
            "ACCEPT_DIAGNOSTIC_STRATEGY_FOR_CANONICAL_DESIGN"
            if passed
            else "REJECT_DIAGNOSTIC_STRATEGY_NO_RETRY"
        ),
        "scope": "DRY_FIT_ONLY_NO_ENVIRONMENT_NO_TEACHER_NO_PUBLICATION",
        "strategy": {
            "strategy_id": STRATEGY_ID,
            "standard_rlmodule_topology": [
                uniform.INPUT_WIDTH,
                uniform.TARGET_WIDTH,
                uniform.TARGET_WIDTH,
                uniform.ACTION_DIM,
            ],
            "tower_a": "IMMUTABLE_R6_W512_FUNCTION",
            "tower_b": "TRAINABLE_W512_RESIDUAL_CONTINUED_FROM_EXACT_UNIFORM_TERMINAL",
            "cross_blocks": "PERMANENT_POSITIVE_ZERO",
            "starting_state": "BYTE_EXACT_TERMINAL_W1024_RESET3_UNIFORM_DRY_FIT",
            "objective_formula": (
                "L=(reset3_weighted_MSE/0.006^2) + margin_smoothmax("
                "all_gate_group_RMSE/0.006) + margin_smoothmax("
                "all_scalar_abs_error/0.060) + margin_smoothmax("
                "reset_scalar_abs_error/0.003)"
            ),
            "margin_smoothmax_formula": (
                "tau*softplus((tau*logsumexp(z/tau)-margin)/tau)"
            ),
            "smooth_max_temperature": SMOOTH_MAX_TEMPERATURE,
            "safety_margin_fraction": SAFETY_MARGIN_FRACTION,
            "loss_coefficients": LOSS_COEFFICIENTS,
            "gate_group_count": len(gate_groups),
            "gate_groups": {
                name: int(len(index)) for name, index in gate_groups.items()
            },
            "explicitly_tracked_prior_failures": list(explicitly_tracked),
            "explicitly_tracked_groups_are_not_reweighted": all(
                name in gate_groups for name in explicitly_tracked
            ),
            "all_gate_groups_enter_symmetrically": True,
            "weight_audit": weight_audit,
            "unchanged_offline_thresholds": r9.contract.OFFLINE_THRESHOLDS,
            "single_preregistered_schedule": True,
            "sweep": False,
            "retry": False,
            "selection": False,
        },
        "frozen_uniform_result": {
            "path": FROZEN_UNIFORM_RESULT.relative_to(REPO_ROOT).as_posix(),
            "sha256": _sha256_file(FROZEN_UNIFORM_RESULT),
            "checks": frozen_checks,
            "expected_state_digest": EXPECTED_UNIFORM_STATE_DIGEST,
            "expected_prediction_digest": EXPECTED_UNIFORM_PREDICTION_DIGEST,
            "expected_lbfgs_closure_calls": EXPECTED_UNIFORM_CLOSURES,
        },
        "frozen_inputs": source["records"],
        "corpus": {
            "rows": int(len(arrays["observations"])),
            "observation_shape": list(arrays["observations"].shape),
            "action_shape": list(arrays["actions"].shape),
            "reset_rows": int(np.count_nonzero(arrays["reset_mask"])),
            "strata": sorted(set(arrays["stratum_ids"].astype(str))),
        },
        "primary": primary,
        "determinism_replica": replica,
        "execution_accounting": {
            "diagnostic_fit_processes": total_fits,
            "uniform_terminal_reproductions": total_fits,
            "gate_aligned_continuations": total_fits,
            "offline_h0_teacher_queries_per_fit": OFFLINE_H0_QUERY_COUNT,
            "offline_h0_teacher_queries_total": OFFLINE_H0_QUERY_COUNT * total_fits,
            "environment_reset_calls": 0,
            "environment_step_calls": 0,
            "ppo_updates": 0,
            "critic_updates": 0,
            "persistent_model_state_writes": 0,
            "persistent_checkpoint_writes": 0,
            "temporary_compatibility_checkpoint_roundtrips_per_fit": 1,
            "production_candidate_created": False,
            "production_candidate_promoted": False,
            "publication_writes": 0,
        },
        "limitations": {
            "literal_r9_architecture_gate": (
                "necessarily false because its frozen schema requires [512,512]"
            ),
            "architecture_replacement": (
                "standard [1024,1024] RLModule compatibility is tested; every "
                "numerical threshold and other semantic gate is unchanged"
            ),
            "rollout_evidence": False,
            "production_attestation": False,
            "known_transition_alias_risk": (
                "offline fit cannot prove Markov sufficiency at the legacy timeout "
                "boundary; no label or observation semantics were changed here"
            ),
        },
        "diagnostic_source": {
            "path": Path(__file__).resolve().relative_to(REPO_ROOT).as_posix(),
            "sha256": _sha256_file(Path(__file__).resolve()),
        },
        "working_directory": Path.cwd().resolve().as_posix(),
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    destination = args.output.expanduser().resolve()
    try:
        destination.relative_to(R9_ROOT.resolve())
    except ValueError:
        pass
    else:
        raise SystemExit("refusing to write below validation/v12r9")
    if destination.exists():
        raise SystemExit(f"refusing to overwrite existing result: {destination}")
    payload = run()
    destination.parent.mkdir(parents=True, exist_ok=True)
    encoded = (
        json.dumps(
            payload, indent=2, sort_keys=True, ensure_ascii=False, allow_nan=False
        )
        + "\n"
    )
    flags = os.O_CREAT | os.O_EXCL | os.O_WRONLY | getattr(os, "O_BINARY", 0)
    descriptor = os.open(destination, flags, 0o600)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            descriptor = -1
            stream.write(encoded)
            stream.flush()
            os.fsync(stream.fileno())
    finally:
        if descriptor >= 0:
            os.close(descriptor)
    print(destination, flush=True)


if __name__ == "__main__":
    main()
