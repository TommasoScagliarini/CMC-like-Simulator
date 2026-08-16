"""Preregistered R10 dry fit: extended R9 continuation with reset x3.

This diagnostic consumes the immutable, terminal-failed R9 candidate and its
frozen corpus.  It changes exactly one aspect of the R9 equal-stratum
objective: each of the 26 reset rows is multiplied by three before every
stratum is renormalized to mass 500.  There is no source-hardness term, tail
term, sweep, retry, early candidate selection, post-fit repair, teacher query,
environment access, checkpoint publication, or production promotion.

The optimizer is fixed before execution: AdamW(1000, lr=3e-5) followed by
LBFGS(max_iter=2000, max_eval=3000).  Only the terminal optimizer state is
evaluated.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
from pathlib import Path
from typing import Any, Mapping

import numpy as np


DIAGNOSTIC_ROOT = Path(__file__).resolve().parent
if str(DIAGNOSTIC_ROOT) not in sys.path:
    sys.path.insert(0, str(DIAGNOSTIC_ROOT))

import analyze_h0_v12r9_frozen_fit as forensic  # noqa: E402


r9 = forensic.r9
REPO_ROOT = forensic.REPO_ROOT
R9_ROOT = forensic.R9_ROOT

STATUS = "COMPLETE_H0_V12R10_EXTENDED_UNIFORM_RESET3_DRY_FIT"
R9_TERMINAL_STATUS = "FAIL_H0_V12R9_RECOVERY_PIPELINE_TERMINAL"
R9_FIT_FAIL_STATUS = "FAIL_H0_V12R9_RECOVERY_FIT"
SEED = 20260814
TORCH_THREADS = 5
RESET_MULTIPLIER = 3.0
STRATUM_MASS = 500.0
ADAMW_EPOCHS = 1000
ADAMW_LR = 3.0e-5
ADAMW_WEIGHT_DECAY = 1.0e-7
GRADIENT_CLIP_NORM = 10.0
LBFGS_LR = 0.7
LBFGS_MAX_ITER = 2000
LBFGS_MAX_EVAL = 3000
LBFGS_HISTORY_SIZE = 50
LBFGS_TOLERANCE_GRAD = 1.0e-10
LBFGS_TOLERANCE_CHANGE = 1.0e-12


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _load_arrays() -> dict[str, np.ndarray]:
    summary = forensic._strict_json(forensic.SUMMARY_PATH)
    ledger = forensic._strict_json(forensic.LEDGER_PATH)
    gate = forensic._strict_json(forensic.GATE_PATH)
    if (
        ledger.get("status") != R9_TERMINAL_STATUS
        or ledger.get("passed") is not False
        or ledger.get("attempted_stage") != "fit_recovery_actor"
        or gate.get("status") != R9_FIT_FAIL_STATUS
        or gate.get("passed") is not False
        or summary.get("candidate_module", {}).get("tree_sha256")
        != "8bc8554c573f8224ea3fa9d8682d315f9ff30c4aaa1557c4974e4fa422b5d1ff"
        or _sha256(forensic.CORPUS_PATH) != summary.get("corpus", {}).get("sha256")
    ):
        raise RuntimeError("immutable terminal R9 initialization drifted")
    with np.load(forensic.CORPUS_PATH, allow_pickle=False) as loaded:
        arrays = {name: np.ascontiguousarray(loaded[name]) for name in loaded.files}
    if (
        arrays["observations"].shape != (11875, 35)
        or arrays["actions"].shape != (11875, 2)
        or int(np.count_nonzero(arrays["reset_mask"])) != 26
        or set(arrays["stratum_ids"].astype(str)) != set(r9.expected_stratum_ids())
    ):
        raise RuntimeError("persisted R9 corpus shape or stratum closure drifted")
    return arrays


def _reset3_weights(arrays: Mapping[str, np.ndarray]) -> tuple[np.ndarray, Any]:
    original = np.ascontiguousarray(
        arrays["normalized_sample_weights"], dtype=np.float64
    )
    weights = original.copy()
    weights[arrays["reset_mask"]] *= RESET_MULTIPLIER
    strata = arrays["stratum_ids"].astype(str)
    audit: dict[str, Any] = {}
    for stratum_id in r9.expected_stratum_ids():
        selected = np.flatnonzero(strata == stratum_id)
        weights[selected] *= STRATUM_MASS / math.fsum(weights[selected])
        mass = math.fsum(weights[selected])
        if not math.isclose(mass, STRATUM_MASS, rel_tol=0.0, abs_tol=1.0e-9):
            raise RuntimeError(f"reset3 stratum mass drifted: {stratum_id}")
        reset_selected = selected[arrays["reset_mask"][selected]]
        audit[stratum_id] = {
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
        not math.isclose(total, 6500.0, rel_tol=0.0, abs_tol=1.0e-8)
        or not np.all(np.isfinite(weights))
        or np.any(weights <= 0.0)
    ):
        raise RuntimeError("reset3 weight closure failed")
    return np.ascontiguousarray(weights), {
        "policy": "R9_UNIFORM_EQUAL_STRATUM_WITH_RESET_ROWS_X3",
        "only_objective_change_from_r9": "RESET_ROWS_MULTIPLIED_BY_3",
        "reset_multiplier": RESET_MULTIPLIER,
        "reset_row_count": 26,
        "stratum_count": 13,
        "stratum_mass": STRATUM_MASS,
        "total_mass": float(total),
        "pre_renormalization_reset_mass": float(
            math.fsum((original[arrays["reset_mask"]] * RESET_MULTIPLIER))
        ),
        "post_renormalization_reset_mass": float(
            math.fsum(weights[arrays["reset_mask"]])
        ),
        "weights_sha256": r9.v10s_fit.array_sha256(weights),
        "strata": audit,
    }


def _normalization(arrays: Mapping[str, np.ndarray]) -> Any:
    base_rows = int(r9.contract.LOCKED_INPUTS["base_corpus"]["rows"])
    selected = np.flatnonzero(
        arrays["tranche_ids"][:base_rows].astype(str) == "v8r1p1_base"
    )
    if len(selected) != 3000:
        raise RuntimeError("frozen R9 normalization selection drifted")
    return r9.v11.frozen_base_normalization(arrays["observations"][selected])


def _snapshot(
    *,
    stage: str,
    index: int,
    loss: Any,
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
        "global_rmse": metrics["global_metrics"]["rmse"],
        "global_max_abs_error": metrics["global_metrics"]["max_abs_error"],
        "reset_max_abs_error": metrics["reset_max_abs_error"],
        "gate_passed": gate["passed"],
    }


def _state_digest(state: Mapping[str, Any]) -> str:
    digest = hashlib.sha256()
    for name in sorted(state):
        value = np.ascontiguousarray(np.asarray(state[name]))
        digest.update(name.encode("utf-8"))
        digest.update(str(value.dtype).encode("ascii"))
        digest.update(repr(value.shape).encode("ascii"))
        digest.update(value.tobytes(order="C"))
    return digest.hexdigest()


def run() -> dict[str, Any]:
    import torch

    arrays = _load_arrays()
    candidate_state, initial_predictions = forensic._candidate_state_and_predictions(
        arrays["observations"]
    )
    initial_metrics = forensic._metric_payload(initial_predictions, arrays)
    initial_gate = forensic._gate(initial_metrics)
    if (
        initial_gate.get("status") != R9_FIT_FAIL_STATUS
        or initial_gate.get("passed") is not False
    ):
        raise RuntimeError("R9 initialization no longer reproduces its failed gate")
    weights_np, weight_audit = _reset3_weights(arrays)
    normalization = _normalization(arrays)
    normalized = r9.v11.normalized_observations(arrays["observations"], normalization)

    previous_threads = torch.get_num_threads()
    previous_deterministic = torch.are_deterministic_algorithms_enabled()
    torch.set_num_threads(TORCH_THREADS)
    torch.use_deterministic_algorithms(True)
    try:
        torch.manual_seed(SEED)
        model = r9._new_normalized_model(candidate_state, normalization)
        x = torch.as_tensor(normalized, dtype=torch.float32)
        y = torch.as_tensor(arrays["actions"], dtype=torch.float32)
        weights = torch.as_tensor(weights_np, dtype=torch.float64)
        weight_sum = torch.sum(weights)

        def objective() -> Any:
            prediction = model(x)
            row_mse = torch.mean(torch.square(prediction - y), dim=1)
            return torch.sum(weights * row_mse) / weight_sum

        history: list[dict[str, Any]] = []
        with torch.no_grad():
            initial_loss = objective()
        initial_row = _snapshot(
            stage="initial",
            index=0,
            loss=initial_loss,
            model=model,
            x=x,
            arrays=arrays,
        )
        history.append(initial_row)
        print(json.dumps(initial_row, sort_keys=True), flush=True)

        adamw = torch.optim.AdamW(
            model.parameters(), lr=ADAMW_LR, weight_decay=ADAMW_WEIGHT_DECAY
        )
        adam_milestones = {1, 100, 250, 500, 750, 1000}
        for epoch in range(1, ADAMW_EPOCHS + 1):
            adamw.zero_grad(set_to_none=True)
            loss = objective()
            if not torch.isfinite(loss):
                raise RuntimeError(f"non-finite AdamW loss at epoch {epoch}")
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), GRADIENT_CLIP_NORM)
            adamw.step()
            with torch.no_grad():
                model[0].weight[:, r9.contract.DISABLED_CLOCK_COLUMNS] = 0.0
            if epoch in adam_milestones:
                row = _snapshot(
                    stage="adamw",
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
            lr=LBFGS_LR,
            max_iter=LBFGS_MAX_ITER,
            max_eval=LBFGS_MAX_EVAL,
            tolerance_grad=LBFGS_TOLERANCE_GRAD,
            tolerance_change=LBFGS_TOLERANCE_CHANGE,
            history_size=LBFGS_HISTORY_SIZE,
            line_search_fn="strong_wolfe",
        )
        closure_calls = 0

        def closure() -> Any:
            nonlocal closure_calls
            lbfgs.zero_grad(set_to_none=True)
            value = objective()
            if not torch.isfinite(value):
                raise RuntimeError(
                    f"non-finite LBFGS loss at closure {closure_calls + 1}"
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
                    "stage": "lbfgs_closure",
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

        continued_state, fold_audit = r9._fold_normalization_into_state(
            model, candidate_state, normalization
        )
        predictions = np.ascontiguousarray(
            r9.v11._state_logits(continued_state, arrays["observations"])[:, :2],
            dtype=np.float32,
        )
        with torch.no_grad():
            normalized_predictions = np.ascontiguousarray(
                model(x).cpu().numpy(), dtype=np.float32
            )
        fold_equivalence = r9.v11.fold_equivalence_audit(
            normalized_predictions, predictions
        )
        terminal_metrics = forensic._metric_payload(predictions, arrays)
        terminal_gate = forensic._gate(terminal_metrics)
        terminal_row = {
            "stage": "terminal_final_state",
            "index": closure_calls,
            "loss": float(terminal_loss.detach().cpu()),
            "global_rmse": terminal_metrics["global_metrics"]["rmse"],
            "global_max_abs_error": terminal_metrics["global_metrics"]["max_abs_error"],
            "reset_max_abs_error": terminal_metrics["reset_max_abs_error"],
            "gate_passed": terminal_gate["passed"],
        }
        history.append(terminal_row)
        print(json.dumps(terminal_row, sort_keys=True), flush=True)

        return {
            "status": STATUS,
            "passed": True,
            "scope": "DRY_FIT_ONLY_NO_ENVIRONMENT_NO_TEACHER_NO_PUBLICATION",
            "decision": "ACCEPT_DRY_FIT"
            if terminal_gate["passed"] is True
            else "REJECT_DRY_FIT_NO_RETRY",
            "r9_initialization": {
                "artifact_is_terminal_fail": True,
                "pipeline_status": R9_TERMINAL_STATUS,
                "fit_gate_status": R9_FIT_FAIL_STATUS,
                "candidate_tree_sha256": forensic._strict_json(forensic.SUMMARY_PATH)[
                    "candidate_module"
                ]["tree_sha256"],
                "candidate_state_digest": _state_digest(candidate_state),
                "production_use_requires_explicit_adjudication": True,
                "production_adjudication_performed": False,
            },
            "frozen_inputs": {
                "r9_ledger_sha256": _sha256(forensic.LEDGER_PATH),
                "r9_gate_sha256": _sha256(forensic.GATE_PATH),
                "r9_summary_sha256": _sha256(forensic.SUMMARY_PATH),
                "r9_corpus_sha256": _sha256(forensic.CORPUS_PATH),
            },
            "objective": {
                "formula": (
                    "R9_UNIFORM_EQUAL_STRATUM_WEIGHTED_MSE_WITH_RESET_ROWS_X3_"
                    "THEN_EACH_STRATUM_RENORMALIZED_TO_MASS_500"
                ),
                "source_hardness_term": False,
                "tail_term": False,
                "reset_multiplier": RESET_MULTIPLIER,
                "weight_audit": weight_audit,
            },
            "optimizer": {
                "initialization": "TERMINAL_R9_CANDIDATE",
                "terminal_state_only": True,
                "early_candidate_selection": False,
                "sweep": False,
                "retry": False,
                "repair": False,
                "seed": SEED,
                "torch_threads": TORCH_THREADS,
                "deterministic_algorithms": True,
                "adamw_epochs": ADAMW_EPOCHS,
                "adamw_learning_rate": ADAMW_LR,
                "adamw_weight_decay": ADAMW_WEIGHT_DECAY,
                "gradient_clip_norm": GRADIENT_CLIP_NORM,
                "lbfgs_lr": LBFGS_LR,
                "lbfgs_max_iter": LBFGS_MAX_ITER,
                "lbfgs_max_eval": LBFGS_MAX_EVAL,
                "lbfgs_tolerance_grad": LBFGS_TOLERANCE_GRAD,
                "lbfgs_tolerance_change": LBFGS_TOLERANCE_CHANGE,
                "lbfgs_history_size": LBFGS_HISTORY_SIZE,
                "lbfgs_line_search": "strong_wolfe",
                "lbfgs_closure_calls": closure_calls,
            },
            "initial_metrics": initial_metrics,
            "initial_gate": initial_gate,
            "terminal_metrics": terminal_metrics,
            "terminal_gate": terminal_gate,
            "terminal_state": {
                "state_digest": _state_digest(continued_state),
                "candidate_published": False,
                "state_persisted": False,
                "critic_present": False,
                "critic_updates": 0,
                "ppo_updates": 0,
                "teacher_queries": 0,
                "environment_reset_calls": 0,
                "environment_step_calls": 0,
                "clock_columns_positive_zero": r9._positive_zero(
                    np.asarray(continued_state["pi_encoder.0.weight"])[
                        :, r9.contract.DISABLED_CLOCK_COLUMNS
                    ]
                ),
                "logstd_rows_byte_exact_to_r9": bool(
                    np.asarray(continued_state["pi.1.weight"])[2:].tobytes()
                    == np.asarray(candidate_state["pi.1.weight"])[2:].tobytes()
                    and np.asarray(continued_state["pi.1.bias"])[2:].tobytes()
                    == np.asarray(candidate_state["pi.1.bias"])[2:].tobytes()
                ),
                "encoder_aliases_byte_exact": all(
                    np.asarray(continued_state[left]).tobytes()
                    == np.asarray(continued_state[right]).tobytes()
                    for left, right in (
                        ("pi_encoder.0.weight", "pi.0.0.weight"),
                        ("pi_encoder.0.bias", "pi.0.0.bias"),
                        ("pi_encoder.2.weight", "pi.0.2.weight"),
                        ("pi_encoder.2.bias", "pi.0.2.bias"),
                    )
                ),
                "normalization_fold": fold_audit,
                "fold_equivalence": fold_equivalence,
            },
            "terminal_stratum_errors": forensic._stratum_error_audit(
                predictions, arrays, weights_np
            ),
            "terminal_reset": forensic._reset_audit(predictions, arrays, weights_np),
            "terminal_top_error_rows": forensic._top_error_rows(
                predictions, arrays, weights_np, limit=20
            ),
            "history": history,
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
