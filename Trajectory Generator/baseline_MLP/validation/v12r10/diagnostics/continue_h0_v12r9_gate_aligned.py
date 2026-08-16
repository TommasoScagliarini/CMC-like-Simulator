"""One preregistered, in-memory full-model continuation of the failed R9 fit.

The frozen R9 candidate is used only as initialization.  The objective is
aligned with the already-frozen offline gates while retaining the original
within-stratum training hardness as a secondary preservation term.  This file
does not publish an RLModule, touch R9, query a teacher, or open an environment.

Preregistered objective (no coefficient sweep)::

    uniform_equal_stratum_mse
      + 0.25 * source_normalized_equal_stratum_mse
      + 4.0  * mean_slice_fourth_error / 0.045**2
      + 10.0 * reset_mse

The fourteen tail slices are the thirteen training strata plus the locked
observer ``+0.20`` late subset used by its own gate.  The 0.045 margin is 75%
of the frozen 0.060 max-error ceiling.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import sys
import time
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

UNIFORM_COEFFICIENT = 1.0
SOURCE_COEFFICIENT = 0.25
TAIL_COEFFICIENT = 4.0
RESET_COEFFICIENT = 10.0
TAIL_MARGIN = 0.045
ADAMW_EPOCHS = 400
ADAMW_BOUNDARIES = (250, 350, 400)
ADAMW_RATES = (3.0e-5, 1.0e-5, 3.0e-6)
ADAMW_WEIGHT_DECAY = 1.0e-8
LBFGS_LR = 0.5
LBFGS_MAX_ITER = 500
LBFGS_MAX_EVAL = 750
LBFGS_HISTORY_SIZE = 50
TORCH_THREADS = 5
SEED = 20260814


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _load_frozen_arrays() -> dict[str, np.ndarray]:
    with np.load(forensic.CORPUS_PATH, allow_pickle=False) as loaded:
        arrays = {name: np.ascontiguousarray(loaded[name]) for name in loaded.files}
    persisted = forensic._strict_json(forensic.SUMMARY_PATH)
    if (
        _sha256(forensic.CORPUS_PATH) != persisted["corpus"]["sha256"]
        or persisted["candidate_module"]["tree_sha256"]
        != "8bc8554c573f8224ea3fa9d8682d315f9ff30c4aaa1557c4974e4fa422b5d1ff"
    ):
        raise RuntimeError("frozen R9 inputs drifted")
    return arrays


def _normalization(arrays: Mapping[str, np.ndarray]) -> Any:
    base_rows = int(r9.contract.LOCKED_INPUTS["base_corpus"]["rows"])
    selected = np.flatnonzero(
        arrays["tranche_ids"][:base_rows].astype(str) == "v8r1p1_base"
    )
    if len(selected) != 3000:
        raise RuntimeError("frozen normalization row selection drifted")
    return r9.v11.frozen_base_normalization(arrays["observations"][selected])


def _tail_slices(arrays: Mapping[str, np.ndarray]) -> dict[str, np.ndarray]:
    strata = forensic._indices_by_value(arrays["stratum_ids"])
    plus = strata["observer::deterministic_offset_plus_0p20"]
    plus_late = plus[arrays["step_indices"][plus] >= 140]
    if len(strata) != 13 or len(plus_late) == 0:
        raise RuntimeError("gate-aligned tail slices drifted")
    return {**strata, "gate::observer_plus_late": plus_late}


def _adamw_rate(epoch: int) -> float:
    if epoch <= ADAMW_BOUNDARIES[0]:
        return ADAMW_RATES[0]
    if epoch <= ADAMW_BOUNDARIES[1]:
        return ADAMW_RATES[1]
    return ADAMW_RATES[2]


def _metric_snapshot(
    model: Any,
    x: Any,
    arrays: Mapping[str, np.ndarray],
) -> tuple[dict[str, Any], dict[str, Any]]:
    import torch

    with torch.no_grad():
        predictions = np.ascontiguousarray(model(x).cpu().numpy(), dtype=np.float32)
    metrics = forensic._metric_payload(predictions, arrays)
    return metrics, forensic._gate(metrics)


def _loss_factory(
    *,
    targets: Any,
    uniform_weights: Any,
    source_weights: Any,
    reset_indices: Any,
    tail_indices: Mapping[str, Any],
) -> Any:
    import torch

    uniform_mass = torch.sum(uniform_weights)
    source_mass = torch.sum(source_weights)

    def compute(prediction: Any) -> tuple[Any, dict[str, Any]]:
        error = prediction - targets
        row_mse = torch.mean(torch.square(error), dim=1).to(torch.float64)
        uniform = torch.sum(uniform_weights * row_mse) / uniform_mass
        source = torch.sum(source_weights * row_mse) / source_mass
        fourth = []
        for index in tail_indices.values():
            selected = error[index].to(torch.float64)
            fourth.append(torch.mean(torch.pow(selected, 4)) / (TAIL_MARGIN**2))
        tail = torch.mean(torch.stack(fourth))
        reset = torch.mean(torch.square(error[reset_indices]).to(torch.float64))
        total = (
            UNIFORM_COEFFICIENT * uniform
            + SOURCE_COEFFICIENT * source
            + TAIL_COEFFICIENT * tail
            + RESET_COEFFICIENT * reset
        )
        return total, {
            "uniform_equal_stratum_mse": uniform,
            "source_normalized_equal_stratum_mse": source,
            "tail_fourth_scaled": tail,
            "reset_mse": reset,
        }

    return compute


def _float_terms(loss: Any, terms: Mapping[str, Any]) -> dict[str, float]:
    return {
        "loss": float(loss.detach().cpu()),
        **{name: float(value.detach().cpu()) for name, value in terms.items()},
    }


def run() -> dict[str, Any]:
    import torch

    arrays = _load_frozen_arrays()
    candidate_state, initial_predictions = forensic._candidate_state_and_predictions(
        arrays["observations"]
    )
    initial_metrics = forensic._metric_payload(initial_predictions, arrays)
    initial_gate = forensic._gate(initial_metrics)
    if initial_gate.get("passed") is not False:
        raise RuntimeError("R9 initialization is not the frozen failed fit")
    source_weights_np, source_weight_audit = forensic._source_piece_weights(
        arrays, source_key="normalized_sample_weights"
    )
    uniform_weights_np = np.ascontiguousarray(
        arrays["normalized_sample_weights"], dtype=np.float64
    )
    if not (
        math.isclose(math.fsum(source_weights_np), 6500.0, abs_tol=1.0e-8)
        and math.isclose(math.fsum(uniform_weights_np), 6500.0, abs_tol=1.0e-8)
    ):
        raise RuntimeError("thirteen-stratum mass closure failed")
    normalization = _normalization(arrays)
    normalized = r9.v11.normalized_observations(arrays["observations"], normalization)
    tail_slices_np = _tail_slices(arrays)
    reset_np = np.flatnonzero(arrays["reset_mask"])

    previous_threads = torch.get_num_threads()
    previous_deterministic = torch.are_deterministic_algorithms_enabled()
    torch.set_num_threads(TORCH_THREADS)
    torch.use_deterministic_algorithms(True)
    started = time.monotonic()
    try:
        torch.manual_seed(SEED)
        model = r9._new_normalized_model(candidate_state, normalization)
        x = torch.as_tensor(normalized, dtype=torch.float32)
        y = torch.as_tensor(arrays["actions"], dtype=torch.float32)
        uniform_weights = torch.as_tensor(uniform_weights_np, dtype=torch.float64)
        source_weights = torch.as_tensor(source_weights_np, dtype=torch.float64)
        reset_indices = torch.as_tensor(reset_np, dtype=torch.int64)
        tail_indices = {
            name: torch.as_tensor(index, dtype=torch.int64)
            for name, index in tail_slices_np.items()
        }
        objective = _loss_factory(
            targets=y,
            uniform_weights=uniform_weights,
            source_weights=source_weights,
            reset_indices=reset_indices,
            tail_indices=tail_indices,
        )
        history: list[dict[str, Any]] = []
        with torch.no_grad():
            initial_loss, initial_terms = objective(model(x))
        history.append(
            {
                "stage": "initial",
                "index": 0,
                **_float_terms(initial_loss, initial_terms),
            }
        )
        print(json.dumps(history[-1], sort_keys=True), flush=True)

        adamw = torch.optim.AdamW(
            model.parameters(), lr=ADAMW_RATES[0], weight_decay=ADAMW_WEIGHT_DECAY
        )
        adam_milestones = {1, 50, 100, 200, 300, 400}
        for epoch in range(1, ADAMW_EPOCHS + 1):
            rate = _adamw_rate(epoch)
            for group in adamw.param_groups:
                group["lr"] = rate
            adamw.zero_grad(set_to_none=True)
            loss, terms = objective(model(x))
            if not torch.isfinite(loss):
                raise RuntimeError(f"non-finite AdamW loss at epoch {epoch}")
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), 10.0)
            adamw.step()
            with torch.no_grad():
                model[0].weight[:, r9.contract.DISABLED_CLOCK_COLUMNS] = 0.0
            if epoch in adam_milestones:
                row = {
                    "stage": "adamw",
                    "index": epoch,
                    "learning_rate": rate,
                    **_float_terms(loss, terms),
                }
                history.append(row)
                print(json.dumps(row, sort_keys=True), flush=True)

        lbfgs = torch.optim.LBFGS(
            model.parameters(),
            lr=LBFGS_LR,
            max_iter=LBFGS_MAX_ITER,
            max_eval=LBFGS_MAX_EVAL,
            tolerance_grad=1.0e-10,
            tolerance_change=1.0e-12,
            history_size=LBFGS_HISTORY_SIZE,
            line_search_fn="strong_wolfe",
        )
        closure_calls = 0
        final_closure: tuple[Any, Mapping[str, Any]] | None = None

        def closure() -> Any:
            nonlocal closure_calls, final_closure
            lbfgs.zero_grad(set_to_none=True)
            value, components = objective(model(x))
            if not torch.isfinite(value):
                raise RuntimeError(
                    f"non-finite LBFGS loss at closure {closure_calls + 1}"
                )
            value.backward()
            closure_calls += 1
            final_closure = (value, components)
            if closure_calls in {1, 50, 100, 200, 300, 400, 500, 600, 750}:
                row = {
                    "stage": "lbfgs_closure",
                    "index": closure_calls,
                    **_float_terms(value, components),
                }
                history.append(row)
                print(json.dumps(row, sort_keys=True), flush=True)
            return value

        lbfgs.step(closure)
        if final_closure is None:
            raise RuntimeError("LBFGS never evaluated the objective")
        with torch.no_grad():
            model[0].weight[:, r9.contract.DISABLED_CLOCK_COLUMNS] = 0.0
            terminal_loss, terminal_terms = objective(model(x))
        terminal_row = {
            "stage": "terminal",
            "index": closure_calls,
            **_float_terms(terminal_loss, terminal_terms),
        }
        history.append(terminal_row)
        print(json.dumps(terminal_row, sort_keys=True), flush=True)

        continued_state, fold_audit = r9._fold_normalization_into_state(
            model, candidate_state, normalization
        )
        predictions = np.ascontiguousarray(
            r9.v11._state_logits(continued_state, arrays["observations"])[:, :2],
            dtype=np.float32,
        )
        normalized_predictions = np.ascontiguousarray(
            model(x).detach().cpu().numpy(), dtype=np.float32
        )
        fold_equivalence = r9.v11.fold_equivalence_audit(
            normalized_predictions, predictions
        )
        metrics = forensic._metric_payload(predictions, arrays)
        gate = forensic._gate(metrics)
        elapsed = time.monotonic() - started
        return {
            "status": "COMPLETE_H0_V12R10_GATE_ALIGNED_CONTINUATION_DIAGNOSTIC",
            "passed": True,
            "scope": "IN_MEMORY_NO_CANDIDATE_PUBLICATION_NO_ENVIRONMENT_NO_TEACHER",
            "diagnostic_source": {
                "path": Path(__file__).resolve().relative_to(REPO_ROOT).as_posix(),
                "sha256": _sha256(Path(__file__).resolve()),
            },
            "frozen_inputs": {
                "r9_terminal_ledger_sha256": _sha256(forensic.LEDGER_PATH),
                "r9_corpus_sha256": _sha256(forensic.CORPUS_PATH),
                "r9_candidate_tree_sha256": forensic._strict_json(
                    forensic.SUMMARY_PATH
                )["candidate_module"]["tree_sha256"],
            },
            "initialization": "FROZEN_TERMINAL_R9_CANDIDATE_FULL_MEAN",
            "objective": {
                "formula": (
                    "1.0*uniform_equal_stratum_mse + "
                    "0.25*source_normalized_equal_stratum_mse + "
                    "4.0*mean_14_slice_fourth_error/0.045^2 + "
                    "10.0*reset_mse"
                ),
                "uniform_coefficient": UNIFORM_COEFFICIENT,
                "source_coefficient": SOURCE_COEFFICIENT,
                "tail_coefficient": TAIL_COEFFICIENT,
                "reset_coefficient": RESET_COEFFICIENT,
                "tail_margin": TAIL_MARGIN,
                "tail_slices": list(tail_slices_np),
                "source_weight_provenance": (
                    "source normalized_sample_weights, renormalized to mass 500 "
                    "inside each stratum; R4 uniform"
                ),
                "source_weight_audit": source_weight_audit,
                "source_weight_sha256": r9.v10s_fit.array_sha256(source_weights_np),
                "uniform_weight_sha256": r9.v10s_fit.array_sha256(uniform_weights_np),
            },
            "optimizer": {
                "seed": SEED,
                "torch_threads": TORCH_THREADS,
                "deterministic_algorithms": True,
                "adamw_epochs": ADAMW_EPOCHS,
                "adamw_boundaries": list(ADAMW_BOUNDARIES),
                "adamw_learning_rates": list(ADAMW_RATES),
                "adamw_weight_decay": ADAMW_WEIGHT_DECAY,
                "lbfgs_lr": LBFGS_LR,
                "lbfgs_max_iter": LBFGS_MAX_ITER,
                "lbfgs_max_eval": LBFGS_MAX_EVAL,
                "lbfgs_history_size": LBFGS_HISTORY_SIZE,
                "lbfgs_line_search": "strong_wolfe",
                "lbfgs_closure_calls": closure_calls,
                "no_sweep": True,
            },
            "initial_metrics": initial_metrics,
            "initial_gate": initial_gate,
            "terminal_metrics": metrics,
            "terminal_gate": gate,
            "terminal_stratum_errors": forensic._stratum_error_audit(
                predictions, arrays, uniform_weights_np
            ),
            "terminal_reset": forensic._reset_audit(
                predictions, arrays, source_weights_np
            ),
            "terminal_top_error_rows": forensic._top_error_rows(
                predictions, arrays, source_weights_np, limit=20
            ),
            "preservation": {
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
                "critic_present": False,
                "critic_updates": 0,
                "ppo_updates": 0,
                "candidate_published": False,
            },
            "history": history,
            "elapsed_seconds": float(elapsed),
            "process": {
                "pid": os.getpid(),
                "working_directory": Path.cwd().resolve().as_posix(),
            },
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
    payload = run()
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(destination, flush=True)


if __name__ == "__main__":
    main()
