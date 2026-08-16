"""Single preregistered W512 dry fit with a frozen-boundary IRLS tail phase.

Phase A reproduces byte-for-byte the already-recorded reset-x3 continuation in
``dry_fit_extended_uniform_reset3.py``.  Its terminal state is audited against
the immutable diagnostic receipt before Phase B starts.

Phase B is one fixed, finite optimization.  Element weights are computed once
from the absolute Phase-A boundary errors and never updated::

    multiplier[i, d] = 1 + 4 * min((abs(error_A[i, d]) / 0.045)**2, 9)

For each of the thirteen strata and each action dimension, the resulting
``reset3_row_weight * multiplier`` values are normalized to mass 250.  The
fixed Phase-B objective is::

    0.5 * reset3_equal_stratum_mse
      + 0.5 * frozen_boundary_irls_element_mse
      + 5.0 * reset_mse

The optimizer is fixed before execution: AdamW for 600 epochs with learning
rates 2e-5/7e-6/2e-6 over epochs 1:300/301:500/501:600, followed by one LBFGS
call with max_iter=1200 and max_eval=1800.  Only the terminal optimizer state
is evaluated.  There is no sweep, retry, best-checkpoint selection, model-state
persistence, teacher query, environment access, checkpoint publication, critic
update, or PPO update.  A second replica is allowed only if this run passes all
unchanged R9 offline gates.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path
from typing import Any, Mapping

import numpy as np


DIAGNOSTIC_ROOT = Path(__file__).resolve().parent
if str(DIAGNOSTIC_ROOT) not in sys.path:
    sys.path.insert(0, str(DIAGNOSTIC_ROOT))

import analyze_h0_v12r9_frozen_fit as forensic  # noqa: E402
import dry_fit_extended_uniform_reset3 as phase_a  # noqa: E402


r9 = forensic.r9
REPO_ROOT = forensic.REPO_ROOT
R9_ROOT = forensic.R9_ROOT

STATUS = "COMPLETE_H0_V12R10_TWO_STAGE_IRLS_TAIL_DRY_FIT"
EXPECTED_PHASE_A_SOURCE_SHA256 = (
    "0c7a70ac0bdf08e80409f7b5ccdba1a49400cacbc14981e26092d26a8068f6d2"
)
EXPECTED_PHASE_A_RESULT_SHA256 = (
    "1aa594a15b6d54a97bf374f1d78801ffa1fafa5050e2add33dd4637f7107c009"
)
EXPECTED_PHASE_A_STATE_DIGEST = (
    "d8ff64158dbd12f058cf172cdf6513fbea2376ee9b7d096bf62929cefe7b74f3"
)
EXPECTED_PHASE_A_CLOSURE_CALLS = 2083
EXPECTED_PHASE_A_WORST_ROW = 10259
EXPECTED_PHASE_A_WORST_ERROR = 0.12474837899208069

PHASE_A_SOURCE_PATH = DIAGNOSTIC_ROOT / "dry_fit_extended_uniform_reset3.py"
PHASE_A_RESULT_PATH = (
    DIAGNOSTIC_ROOT / "results" / "r9_extended_uniform_reset3_dry_fit.json"
)

TAIL_MARGIN = 0.045
TAIL_STRENGTH = 4.0
TAIL_RATIO_SQUARED_CAP = 9.0
PRESERVATION_COEFFICIENT = 0.5
IRLS_COEFFICIENT = 0.5
RESET_COEFFICIENT = 5.0
ACTION_DIMENSION_MASS_PER_STRATUM = phase_a.STRATUM_MASS / 2.0

PHASE_B_ADAMW_EPOCHS = 600
PHASE_B_ADAMW_BOUNDARIES = (300, 500, 600)
PHASE_B_ADAMW_RATES = (2.0e-5, 7.0e-6, 2.0e-6)
PHASE_B_ADAMW_WEIGHT_DECAY = 1.0e-8
PHASE_B_GRADIENT_CLIP_NORM = 10.0
PHASE_B_LBFGS_LR = 0.5
PHASE_B_LBFGS_MAX_ITER = 1200
PHASE_B_LBFGS_MAX_EVAL = 1800
PHASE_B_LBFGS_HISTORY_SIZE = 50
PHASE_B_LBFGS_TOLERANCE_GRAD = 1.0e-10
PHASE_B_LBFGS_TOLERANCE_CHANGE = 1.0e-12


def _strict_json(path: Path) -> Any:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def _phase_a_receipt_audit() -> dict[str, Any]:
    source_sha = phase_a._sha256(PHASE_A_SOURCE_PATH)  # noqa: SLF001
    result_sha = phase_a._sha256(PHASE_A_RESULT_PATH)  # noqa: SLF001
    receipt = _strict_json(PHASE_A_RESULT_PATH)
    top = receipt.get("terminal_top_error_rows", [])
    if (
        source_sha != EXPECTED_PHASE_A_SOURCE_SHA256
        or result_sha != EXPECTED_PHASE_A_RESULT_SHA256
        or receipt.get("status") != phase_a.STATUS
        or receipt.get("decision") != "REJECT_DRY_FIT_NO_RETRY"
        or receipt.get("terminal_gate", {}).get("passed") is not False
        or receipt.get("terminal_state", {}).get("state_persisted") is not False
        or receipt.get("terminal_state", {}).get("state_digest")
        != EXPECTED_PHASE_A_STATE_DIGEST
        or receipt.get("optimizer", {}).get("lbfgs_closure_calls")
        != EXPECTED_PHASE_A_CLOSURE_CALLS
        or not top
        or top[0].get("row_index") != EXPECTED_PHASE_A_WORST_ROW
        or not math.isclose(
            float(top[0].get("absolute_error", math.nan)),
            EXPECTED_PHASE_A_WORST_ERROR,
            rel_tol=0.0,
            abs_tol=0.0,
        )
    ):
        raise RuntimeError("recorded Phase-A result or top-error audit drifted")
    return {
        "source_path": PHASE_A_SOURCE_PATH.relative_to(REPO_ROOT).as_posix(),
        "source_sha256": source_sha,
        "result_path": PHASE_A_RESULT_PATH.relative_to(REPO_ROOT).as_posix(),
        "result_sha256": result_sha,
        "decision": receipt["decision"],
        "terminal_state_digest": receipt["terminal_state"]["state_digest"],
        "lbfgs_closure_calls": receipt["optimizer"]["lbfgs_closure_calls"],
        "global_metrics": receipt["terminal_metrics"]["global_metrics"],
        "reset_max_abs_error": receipt["terminal_metrics"]["reset_max_abs_error"],
        "worst_row": top[0],
        "top_error_rows_audited_before_design_execution": len(top),
    }


def _snapshot(
    *,
    stage: str,
    index: int,
    loss: Any,
    terms: Mapping[str, Any],
    model: Any,
    x: Any,
    arrays: Mapping[str, np.ndarray],
) -> dict[str, Any]:
    import torch

    with torch.no_grad():
        predictions = np.ascontiguousarray(model(x).cpu().numpy(), dtype=np.float32)
    metrics = forensic._metric_payload(predictions, arrays)
    gate = forensic._gate(metrics)
    return {
        "stage": stage,
        "index": int(index),
        "loss": float(loss.detach().cpu()),
        "terms": {name: float(value.detach().cpu()) for name, value in terms.items()},
        "global_rmse": metrics["global_metrics"]["rmse"],
        "global_max_abs_error": metrics["global_metrics"]["max_abs_error"],
        "reset_max_abs_error": metrics["reset_max_abs_error"],
        "gate_passed": gate["passed"],
    }


def _phase_a_fit(
    *,
    model: Any,
    x: Any,
    y: Any,
    reset3_weights: Any,
    arrays: Mapping[str, np.ndarray],
) -> tuple[list[dict[str, Any]], int]:
    """Reproduce exactly the frozen extended-reset3 optimizer sequence."""

    import torch

    weight_sum = torch.sum(reset3_weights)

    def objective() -> Any:
        prediction = model(x)
        row_mse = torch.mean(torch.square(prediction - y), dim=1)
        return torch.sum(reset3_weights * row_mse) / weight_sum

    history: list[dict[str, Any]] = []
    with torch.no_grad():
        initial_loss = objective()
    initial = phase_a._snapshot(  # noqa: SLF001
        stage="phase_a_initial",
        index=0,
        loss=initial_loss,
        model=model,
        x=x,
        arrays=arrays,
    )
    history.append(initial)
    print(json.dumps(initial, sort_keys=True), flush=True)

    adamw = torch.optim.AdamW(
        model.parameters(),
        lr=phase_a.ADAMW_LR,
        weight_decay=phase_a.ADAMW_WEIGHT_DECAY,
    )
    milestones = {1, 100, 250, 500, 750, 1000}
    for epoch in range(1, phase_a.ADAMW_EPOCHS + 1):
        adamw.zero_grad(set_to_none=True)
        loss = objective()
        if not torch.isfinite(loss):
            raise RuntimeError(f"non-finite Phase-A AdamW loss at epoch {epoch}")
        loss.backward()
        torch.nn.utils.clip_grad_norm_(model.parameters(), phase_a.GRADIENT_CLIP_NORM)
        adamw.step()
        with torch.no_grad():
            model[0].weight[:, r9.contract.DISABLED_CLOCK_COLUMNS] = 0.0
        if epoch in milestones:
            row = phase_a._snapshot(  # noqa: SLF001
                stage="phase_a_adamw",
                index=epoch,
                loss=loss,
                model=model,
                x=x,
                arrays=arrays,
            )
            history.append(row)
            print(json.dumps(row, sort_keys=True), flush=True)

    lbfgs = torch.optim.LBFGS(
        model.parameters(),
        lr=phase_a.LBFGS_LR,
        max_iter=phase_a.LBFGS_MAX_ITER,
        max_eval=phase_a.LBFGS_MAX_EVAL,
        tolerance_grad=phase_a.LBFGS_TOLERANCE_GRAD,
        tolerance_change=phase_a.LBFGS_TOLERANCE_CHANGE,
        history_size=phase_a.LBFGS_HISTORY_SIZE,
        line_search_fn="strong_wolfe",
    )
    closure_calls = 0

    def closure() -> Any:
        nonlocal closure_calls
        lbfgs.zero_grad(set_to_none=True)
        value = objective()
        if not torch.isfinite(value):
            raise RuntimeError(
                f"non-finite Phase-A LBFGS loss at closure {closure_calls + 1}"
            )
        value.backward()
        closure_calls += 1
        if closure_calls in {
            1,
            100,
            250,
            500,
            750,
            1000,
            1250,
            1500,
            1750,
            2000,
            2250,
            2500,
            2750,
            3000,
        }:
            row = {
                "stage": "phase_a_lbfgs_closure",
                "index": closure_calls,
                "loss": float(value.detach().cpu()),
            }
            history.append(row)
            print(json.dumps(row, sort_keys=True), flush=True)
        return value

    lbfgs.step(closure)
    with torch.no_grad():
        model[0].weight[:, r9.contract.DISABLED_CLOCK_COLUMNS] = 0.0
        terminal_loss = objective()
    row = phase_a._snapshot(  # noqa: SLF001
        stage="phase_a_terminal_boundary",
        index=closure_calls,
        loss=terminal_loss,
        model=model,
        x=x,
        arrays=arrays,
    )
    history.append(row)
    print(json.dumps(row, sort_keys=True), flush=True)
    return history, closure_calls


def _frozen_boundary_irls_weights(
    *,
    boundary_predictions: np.ndarray,
    arrays: Mapping[str, np.ndarray],
    reset3_weights: np.ndarray,
) -> tuple[np.ndarray, dict[str, Any]]:
    absolute_error = np.abs(
        boundary_predictions.astype(np.float64) - arrays["actions"].astype(np.float64)
    )
    ratio_squared = np.square(absolute_error / TAIL_MARGIN)
    multiplier = 1.0 + TAIL_STRENGTH * np.minimum(ratio_squared, TAIL_RATIO_SQUARED_CAP)
    element_weights = reset3_weights[:, None] * multiplier
    strata = arrays["stratum_ids"].astype(str)
    stratum_audit: dict[str, Any] = {}
    for stratum_id in r9.expected_stratum_ids():
        selected = np.flatnonzero(strata == stratum_id)
        dimension_audit = []
        for action_dimension in range(2):
            values = element_weights[selected, action_dimension]
            values *= ACTION_DIMENSION_MASS_PER_STRATUM / math.fsum(values)
            element_weights[selected, action_dimension] = values
            mass = math.fsum(values)
            if not math.isclose(
                mass,
                ACTION_DIMENSION_MASS_PER_STRATUM,
                rel_tol=0.0,
                abs_tol=1.0e-9,
            ):
                raise RuntimeError(
                    f"Phase-B IRLS mass drifted: {stratum_id}/{action_dimension}"
                )
            dimension_audit.append(
                {
                    "action_dimension": action_dimension,
                    "mass": float(mass),
                    "weight_min": float(np.min(values)),
                    "weight_max": float(np.max(values)),
                    "multiplier_min": float(
                        np.min(multiplier[selected, action_dimension])
                    ),
                    "multiplier_max": float(
                        np.max(multiplier[selected, action_dimension])
                    ),
                }
            )
        stratum_audit[stratum_id] = {
            "rows": int(len(selected)),
            "dimensions": dimension_audit,
        }
    total = math.fsum(element_weights.ravel())
    if (
        not math.isclose(total, 6500.0, rel_tol=0.0, abs_tol=1.0e-8)
        or not np.all(np.isfinite(element_weights))
        or np.any(element_weights <= 0.0)
    ):
        raise RuntimeError("Phase-B IRLS weight closure failed")
    hard = absolute_error >= TAIL_MARGIN
    return np.ascontiguousarray(element_weights), {
        "formula": (
            "reset3_row_weight*(1+4*min((abs_phase_a_error/0.045)^2,9));"
            " each stratum/action dimension renormalized to mass 250"
        ),
        "weights_frozen_for_all_phase_b_updates": True,
        "tail_margin": TAIL_MARGIN,
        "tail_strength": TAIL_STRENGTH,
        "ratio_squared_cap": TAIL_RATIO_SQUARED_CAP,
        "multiplier_min": float(np.min(multiplier)),
        "multiplier_max": float(np.max(multiplier)),
        "boundary_elements_at_or_above_margin": int(np.count_nonzero(hard)),
        "boundary_rows_at_or_above_margin": int(np.count_nonzero(np.any(hard, axis=1))),
        "boundary_absolute_error_sha256": r9.v10s_fit.array_sha256(
            np.ascontiguousarray(absolute_error)
        ),
        "element_weights_sha256": r9.v10s_fit.array_sha256(element_weights),
        "total_mass": float(total),
        "strata": stratum_audit,
    }


def _phase_b_rate(epoch: int) -> float:
    if epoch <= PHASE_B_ADAMW_BOUNDARIES[0]:
        return PHASE_B_ADAMW_RATES[0]
    if epoch <= PHASE_B_ADAMW_BOUNDARIES[1]:
        return PHASE_B_ADAMW_RATES[1]
    return PHASE_B_ADAMW_RATES[2]


def _phase_b_fit(
    *,
    model: Any,
    x: Any,
    y: Any,
    reset3_weights: Any,
    element_weights: Any,
    reset_indices: Any,
    arrays: Mapping[str, np.ndarray],
) -> tuple[list[dict[str, Any]], int, dict[str, float]]:
    import torch

    reset3_mass = torch.sum(reset3_weights)
    irls_mass = torch.sum(element_weights)

    def objective() -> tuple[Any, dict[str, Any]]:
        prediction = model(x)
        squared = torch.square(prediction - y).to(torch.float64)
        row_mse = torch.mean(squared, dim=1)
        preservation = torch.sum(reset3_weights * row_mse) / reset3_mass
        irls = torch.sum(element_weights * squared) / irls_mass
        reset = torch.mean(squared[reset_indices])
        total = (
            PRESERVATION_COEFFICIENT * preservation
            + IRLS_COEFFICIENT * irls
            + RESET_COEFFICIENT * reset
        )
        return total, {
            "reset3_equal_stratum_mse": preservation,
            "frozen_boundary_irls_element_mse": irls,
            "reset_mse": reset,
        }

    history: list[dict[str, Any]] = []
    with torch.no_grad():
        initial_loss, initial_terms = objective()
    initial = _snapshot(
        stage="phase_b_initial",
        index=0,
        loss=initial_loss,
        terms=initial_terms,
        model=model,
        x=x,
        arrays=arrays,
    )
    history.append(initial)
    print(json.dumps(initial, sort_keys=True), flush=True)

    adamw = torch.optim.AdamW(
        model.parameters(),
        lr=PHASE_B_ADAMW_RATES[0],
        weight_decay=PHASE_B_ADAMW_WEIGHT_DECAY,
    )
    milestones = {1, 100, 200, 300, 400, 500, 600}
    for epoch in range(1, PHASE_B_ADAMW_EPOCHS + 1):
        rate = _phase_b_rate(epoch)
        for group in adamw.param_groups:
            group["lr"] = rate
        adamw.zero_grad(set_to_none=True)
        loss, terms = objective()
        if not torch.isfinite(loss):
            raise RuntimeError(f"non-finite Phase-B AdamW loss at epoch {epoch}")
        loss.backward()
        torch.nn.utils.clip_grad_norm_(model.parameters(), PHASE_B_GRADIENT_CLIP_NORM)
        adamw.step()
        with torch.no_grad():
            model[0].weight[:, r9.contract.DISABLED_CLOCK_COLUMNS] = 0.0
        if epoch in milestones:
            row = _snapshot(
                stage="phase_b_adamw",
                index=epoch,
                loss=loss,
                terms=terms,
                model=model,
                x=x,
                arrays=arrays,
            )
            row["learning_rate"] = rate
            history.append(row)
            print(json.dumps(row, sort_keys=True), flush=True)

    lbfgs = torch.optim.LBFGS(
        model.parameters(),
        lr=PHASE_B_LBFGS_LR,
        max_iter=PHASE_B_LBFGS_MAX_ITER,
        max_eval=PHASE_B_LBFGS_MAX_EVAL,
        tolerance_grad=PHASE_B_LBFGS_TOLERANCE_GRAD,
        tolerance_change=PHASE_B_LBFGS_TOLERANCE_CHANGE,
        history_size=PHASE_B_LBFGS_HISTORY_SIZE,
        line_search_fn="strong_wolfe",
    )
    closure_calls = 0

    def closure() -> Any:
        nonlocal closure_calls
        lbfgs.zero_grad(set_to_none=True)
        value, components = objective()
        if not torch.isfinite(value):
            raise RuntimeError(
                f"non-finite Phase-B LBFGS loss at closure {closure_calls + 1}"
            )
        value.backward()
        closure_calls += 1
        if closure_calls in {
            1,
            100,
            250,
            500,
            750,
            1000,
            1250,
            1500,
            1750,
            1800,
        }:
            row = {
                "stage": "phase_b_lbfgs_closure",
                "index": closure_calls,
                "loss": float(value.detach().cpu()),
                "terms": {
                    name: float(term.detach().cpu())
                    for name, term in components.items()
                },
            }
            history.append(row)
            print(json.dumps(row, sort_keys=True), flush=True)
        return value

    lbfgs.step(closure)
    with torch.no_grad():
        model[0].weight[:, r9.contract.DISABLED_CLOCK_COLUMNS] = 0.0
        terminal_loss, terminal_terms = objective()
    terminal = _snapshot(
        stage="phase_b_terminal_final_state",
        index=closure_calls,
        loss=terminal_loss,
        terms=terminal_terms,
        model=model,
        x=x,
        arrays=arrays,
    )
    history.append(terminal)
    print(json.dumps(terminal, sort_keys=True), flush=True)
    return (
        history,
        closure_calls,
        {
            "loss": terminal["loss"],
            **terminal["terms"],
        },
    )


def run() -> dict[str, Any]:
    import torch

    phase_a_receipt = _phase_a_receipt_audit()
    arrays = phase_a._load_arrays()  # noqa: SLF001
    candidate_state, initial_predictions = forensic._candidate_state_and_predictions(
        arrays["observations"]
    )
    initial_metrics = forensic._metric_payload(initial_predictions, arrays)
    initial_gate = forensic._gate(initial_metrics)
    if initial_gate.get("passed") is not False:
        raise RuntimeError("R9 initialization no longer reproduces its failed gate")
    reset3_weights_np, reset3_audit = phase_a._reset3_weights(arrays)  # noqa: SLF001
    normalization = phase_a._normalization(arrays)  # noqa: SLF001
    normalized = r9.v11.normalized_observations(arrays["observations"], normalization)

    previous_threads = torch.get_num_threads()
    previous_deterministic = torch.are_deterministic_algorithms_enabled()
    torch.set_num_threads(phase_a.TORCH_THREADS)
    torch.use_deterministic_algorithms(True)
    started = time.monotonic()
    try:
        torch.manual_seed(phase_a.SEED)
        model = r9._new_normalized_model(candidate_state, normalization)
        x = torch.as_tensor(normalized, dtype=torch.float32)
        y = torch.as_tensor(arrays["actions"], dtype=torch.float32)
        reset3_weights = torch.as_tensor(reset3_weights_np, dtype=torch.float64)

        phase_a_history, phase_a_closure_calls = _phase_a_fit(
            model=model,
            x=x,
            y=y,
            reset3_weights=reset3_weights,
            arrays=arrays,
        )
        boundary_state, boundary_fold_audit = r9._fold_normalization_into_state(
            model, candidate_state, normalization
        )
        boundary_state_digest = phase_a._state_digest(boundary_state)  # noqa: SLF001
        if (
            phase_a_closure_calls != EXPECTED_PHASE_A_CLOSURE_CALLS
            or boundary_state_digest != EXPECTED_PHASE_A_STATE_DIGEST
        ):
            raise RuntimeError(
                "Phase-A in-process reproduction is not byte-identical to receipt"
            )
        with torch.no_grad():
            boundary_normalized_predictions = np.ascontiguousarray(
                model(x).cpu().numpy(), dtype=np.float32
            )
        boundary_predictions = np.ascontiguousarray(
            r9.v11._state_logits(boundary_state, arrays["observations"])[:, :2],
            dtype=np.float32,
        )
        boundary_fold_equivalence = r9.v11.fold_equivalence_audit(
            boundary_normalized_predictions, boundary_predictions
        )
        boundary_metrics = forensic._metric_payload(boundary_predictions, arrays)
        boundary_gate = forensic._gate(boundary_metrics)
        if boundary_gate.get("passed") is not False:
            raise RuntimeError("Phase-A boundary unexpectedly passed")

        element_weights_np, irls_audit = _frozen_boundary_irls_weights(
            boundary_predictions=boundary_predictions,
            arrays=arrays,
            reset3_weights=reset3_weights_np,
        )
        element_weights = torch.as_tensor(element_weights_np, dtype=torch.float64)
        reset_indices = torch.as_tensor(
            np.flatnonzero(arrays["reset_mask"]), dtype=torch.int64
        )
        phase_b_history, phase_b_closure_calls, terminal_objective = _phase_b_fit(
            model=model,
            x=x,
            y=y,
            reset3_weights=reset3_weights,
            element_weights=element_weights,
            reset_indices=reset_indices,
            arrays=arrays,
        )

        terminal_state, terminal_fold_audit = r9._fold_normalization_into_state(
            model, candidate_state, normalization
        )
        predictions = np.ascontiguousarray(
            r9.v11._state_logits(terminal_state, arrays["observations"])[:, :2],
            dtype=np.float32,
        )
        with torch.no_grad():
            normalized_predictions = np.ascontiguousarray(
                model(x).cpu().numpy(), dtype=np.float32
            )
        terminal_fold_equivalence = r9.v11.fold_equivalence_audit(
            normalized_predictions, predictions
        )
        terminal_metrics = forensic._metric_payload(predictions, arrays)
        terminal_gate = forensic._gate(terminal_metrics)
        terminal_state_digest = phase_a._state_digest(terminal_state)  # noqa: SLF001
        elapsed = time.monotonic() - started

        return {
            "status": STATUS,
            "passed": True,
            "scope": "DRY_FIT_ONLY_NO_ENVIRONMENT_NO_TEACHER_NO_PUBLICATION",
            "decision": (
                "ACCEPT_DRY_FIT_AUTHORIZE_BYTE_REPLICA"
                if terminal_gate["passed"] is True
                else "REJECT_DRY_FIT_NO_RETRY"
            ),
            "preregistration": {
                "single_phase_b_configuration": True,
                "sweep": False,
                "retry": False,
                "best_checkpoint_selection": False,
                "terminal_state_only": True,
                "phase_b_weights_frozen_before_phase_b_update_one": True,
                "final_metrics_used_for_checkpoint_selection": False,
                "second_replica_authorized_only_on_integral_pass": True,
            },
            "pre_fit_outlier_interpretation": {
                "observer_nominal_steps_385_388": (
                    "LEGACY_TEACHER_TIMEOUT_TRANSITION_WHILE_V26_REMAINS_STANCE"
                ),
                "observer_seed127_steps_385_386": (
                    "ANALOGOUS_LEGACY_TEACHER_TIMEOUT_WHILE_V26_REMAINS_STANCE"
                ),
                "base_minus_steps_415_421": (
                    "SMOOTH_TARGET_REGION_WITH_LOW_RESET3_ROW_WEIGHT_0P1334"
                ),
                "design_consequence": (
                    "FIXED_BOUNDARY_ERROR_REWEIGHTING_WITHOUT_CHANGING_LABELS_OR_FSM"
                ),
            },
            "phase_a_receipt_audit": phase_a_receipt,
            "phase_a_reproduction": {
                "byte_exact_terminal_state": True,
                "state_digest": boundary_state_digest,
                "lbfgs_closure_calls": phase_a_closure_calls,
                "metrics": boundary_metrics,
                "gate": boundary_gate,
                "normalization_fold": boundary_fold_audit,
                "fold_equivalence": boundary_fold_equivalence,
            },
            "objective": {
                "formula": (
                    "0.5*reset3_equal_stratum_mse + "
                    "0.5*frozen_boundary_irls_element_mse + 5.0*reset_mse"
                ),
                "preservation_coefficient": PRESERVATION_COEFFICIENT,
                "irls_coefficient": IRLS_COEFFICIENT,
                "reset_coefficient": RESET_COEFFICIENT,
                "reset3_weight_audit": reset3_audit,
                "frozen_boundary_irls_weight_audit": irls_audit,
            },
            "optimizer": {
                "initialization": "TERMINAL_R9_THEN_EXACT_PHASE_A_REPRODUCTION",
                "seed": phase_a.SEED,
                "torch_threads": phase_a.TORCH_THREADS,
                "deterministic_algorithms": True,
                "phase_a": {
                    "source": "dry_fit_extended_uniform_reset3.py",
                    "adamw_epochs": phase_a.ADAMW_EPOCHS,
                    "adamw_learning_rate": phase_a.ADAMW_LR,
                    "lbfgs_max_iter": phase_a.LBFGS_MAX_ITER,
                    "lbfgs_max_eval": phase_a.LBFGS_MAX_EVAL,
                    "lbfgs_closure_calls": phase_a_closure_calls,
                },
                "phase_b": {
                    "adamw_epochs": PHASE_B_ADAMW_EPOCHS,
                    "adamw_boundaries": list(PHASE_B_ADAMW_BOUNDARIES),
                    "adamw_learning_rates": list(PHASE_B_ADAMW_RATES),
                    "adamw_weight_decay": PHASE_B_ADAMW_WEIGHT_DECAY,
                    "gradient_clip_norm": PHASE_B_GRADIENT_CLIP_NORM,
                    "lbfgs_lr": PHASE_B_LBFGS_LR,
                    "lbfgs_max_iter": PHASE_B_LBFGS_MAX_ITER,
                    "lbfgs_max_eval": PHASE_B_LBFGS_MAX_EVAL,
                    "lbfgs_history_size": PHASE_B_LBFGS_HISTORY_SIZE,
                    "lbfgs_tolerance_grad": PHASE_B_LBFGS_TOLERANCE_GRAD,
                    "lbfgs_tolerance_change": PHASE_B_LBFGS_TOLERANCE_CHANGE,
                    "lbfgs_line_search": "strong_wolfe",
                    "lbfgs_closure_calls": phase_b_closure_calls,
                },
            },
            "initial_metrics": initial_metrics,
            "initial_gate": initial_gate,
            "terminal_objective": terminal_objective,
            "terminal_metrics": terminal_metrics,
            "terminal_gate": terminal_gate,
            "terminal_state": {
                "state_digest": terminal_state_digest,
                "candidate_published": False,
                "state_persisted": False,
                "critic_present": False,
                "critic_updates": 0,
                "ppo_updates": 0,
                "teacher_queries": 0,
                "offline_h0_target_queries": 0,
                "environment_reset_calls": 0,
                "environment_step_calls": 0,
                "clock_columns_positive_zero": r9._positive_zero(
                    np.asarray(terminal_state["pi_encoder.0.weight"])[
                        :, r9.contract.DISABLED_CLOCK_COLUMNS
                    ]
                ),
                "logstd_rows_byte_exact_to_r9": bool(
                    np.asarray(terminal_state["pi.1.weight"])[2:].tobytes()
                    == np.asarray(candidate_state["pi.1.weight"])[2:].tobytes()
                    and np.asarray(terminal_state["pi.1.bias"])[2:].tobytes()
                    == np.asarray(candidate_state["pi.1.bias"])[2:].tobytes()
                ),
                "encoder_aliases_byte_exact": all(
                    np.asarray(terminal_state[left]).tobytes()
                    == np.asarray(terminal_state[right]).tobytes()
                    for left, right in (
                        ("pi_encoder.0.weight", "pi.0.0.weight"),
                        ("pi_encoder.0.bias", "pi.0.0.bias"),
                        ("pi_encoder.2.weight", "pi.0.2.weight"),
                        ("pi_encoder.2.bias", "pi.0.2.bias"),
                    )
                ),
                "normalization_fold": terminal_fold_audit,
                "fold_equivalence": terminal_fold_equivalence,
            },
            "terminal_stratum_errors": forensic._stratum_error_audit(
                predictions, arrays, reset3_weights_np
            ),
            "terminal_reset": forensic._reset_audit(
                predictions, arrays, reset3_weights_np
            ),
            "terminal_top_error_rows": forensic._top_error_rows(
                predictions, arrays, reset3_weights_np, limit=20
            ),
            "history": [*phase_a_history, *phase_b_history],
            "elapsed_seconds": float(elapsed),
            "working_directory": Path.cwd().resolve().as_posix(),
        }
    finally:
        torch.use_deterministic_algorithms(previous_deterministic)
        torch.set_num_threads(previous_threads)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output",
        required=True,
        type=Path,
        help="Diagnostic JSON destination outside validation/v12r9.",
    )
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
    destination.write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(destination, flush=True)


if __name__ == "__main__":
    main()
