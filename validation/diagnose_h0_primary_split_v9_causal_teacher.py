"""Read-only diagnostic for the H0 primary-split V9 causal teacher.

The V8R1P1 teacher replay is a gated physical/action replay, but its labels
were queried on a complete legacy actor observation.  This diagnostic builds
the V9 counterfactual at the same state by changing *only* actor columns 10/11
(analog shadow load/contact).  Event, gait-phase, and FSM columns remain the
exact V26 values seen by the deployable student.

No artifact or checkpoint is written.
"""

from __future__ import annotations

import hashlib
import json
import argparse
import sys
from pathlib import Path
from typing import Any

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
BASELINE_ROOT = REPO_ROOT / "Trajectory Generator" / "baseline_MLP"
for root in (REPO_ROOT, BASELINE_ROOT):
    if str(root) not in sys.path:
        sys.path.insert(0, str(root))

from ray.rllib.core.columns import Columns  # noqa: E402
from ray.rllib.core.rl_module.rl_module import RLModule  # noqa: E402

from validation import (  # noqa: E402
    h0_primary_grf_split_v8r1p1_teacher_replay_contract as source_contract,
)


SOURCE_ROOT = REPO_ROOT / source_contract.RUN_ROOT
SOURCE_H0 = REPO_ROOT / (
    "validation/critic_warmup/"
    "2026-07-13_markov35_phase_aligned_sigma0005_iter1_retry/rl_module_last"
)
LOAD_CONTACT_INDICES = (10, 11)
CAUSAL_INDICES = tuple(index for index in range(35) if index not in {10, 11})


class V9CausalTeacherDiagnosticError(RuntimeError):
    pass


def _array(value: Any, *, shape: tuple[int, ...], label: str) -> np.ndarray:
    result = np.ascontiguousarray(np.asarray(value, dtype=np.float32))
    if result.shape != shape or not np.all(np.isfinite(result)):
        raise V9CausalTeacherDiagnosticError(
            f"{label} is not finite float32 {shape}"
        )
    return result


def _array_sha256(value: np.ndarray) -> str:
    array = np.ascontiguousarray(value)
    digest = hashlib.sha256()
    digest.update(str(array.dtype).encode("ascii"))
    digest.update(json.dumps(list(array.shape), separators=(",", ":")).encode())
    digest.update(array.tobytes(order="C"))
    return digest.hexdigest()


def load_views() -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    students: list[np.ndarray] = []
    legacy: list[np.ndarray] = []
    reset_mask: list[bool] = []
    for case_id in source_contract.CASE_IDS:
        trace_path = SOURCE_ROOT / case_id / "trace.json"
        rows = json.loads(trace_path.read_text(encoding="utf-8"))
        if len(rows) != source_contract.EXPECTED_STEPS:
            raise V9CausalTeacherDiagnosticError(
                f"source trace length drifted: {case_id}"
            )
        for index, row in enumerate(rows):
            if row.get("step") != index + 1:
                raise V9CausalTeacherDiagnosticError(
                    f"source trace order drifted: {case_id}/{index + 1}"
                )
            students.append(
                _array(
                    row.get("v25_observation"),
                    shape=(35,),
                    label=f"{case_id}/{index + 1} student",
                )
            )
            legacy.append(
                _array(
                    row.get("baseline_teacher_observation"),
                    shape=(35,),
                    label=f"{case_id}/{index + 1} legacy teacher",
                )
            )
            reset_mask.append(index == 0)
    student_array = np.ascontiguousarray(students, dtype=np.float32)
    legacy_array = np.ascontiguousarray(legacy, dtype=np.float32)
    reset_array = np.asarray(reset_mask, dtype=np.bool_)
    if (
        student_array.shape != (3000, 35)
        or legacy_array.shape != (3000, 35)
        or int(np.count_nonzero(reset_array)) != 6
    ):
        raise V9CausalTeacherDiagnosticError("source corpus aggregate drifted")
    return student_array, legacy_array, reset_array


def _fit_diagnostic(
    *,
    students: np.ndarray,
    teacher_means: np.ndarray,
    reset_mask: np.ndarray,
    epochs: int,
    reset_weight: float,
    tail_weight: float,
    tail_threshold: float,
) -> dict[str, Any]:
    import torch

    from primary_split_v25_residual import (
        build_v25_residual_module_from_checkpoint,
    )

    selected = students[:, 2:35].astype(np.float64)
    mean = np.ascontiguousarray(selected.mean(axis=0), dtype=np.float32)
    std = np.ascontiguousarray(
        np.maximum(selected.std(axis=0), 1.0e-4), dtype=np.float32
    )
    module = build_v25_residual_module_from_checkpoint(
        SOURCE_H0,
        input_mean=mean.tolist(),
        input_std=std.tolist(),
        residual_limits=(0.175, 0.15),
        init_seed=20260808,
    )
    parameters = module.prepare_residual_fit()
    optimizer = torch.optim.AdamW(parameters, lr=1.0e-3, weight_decay=1.0e-6)
    x = torch.as_tensor(students, dtype=torch.float32)
    y = torch.as_tensor(teacher_means, dtype=torch.float32)
    weights = np.ones(len(students), dtype=np.float32)
    weights[reset_mask] = np.float32(reset_weight)
    w = torch.as_tensor(weights, dtype=torch.float32)
    initial_loss: float | None = None
    final_loss: float | None = None
    for epoch in range(1, epochs + 1):
        lr = 1.0e-3 if epoch <= 2000 else 3.0e-4 if epoch <= 5000 else 1.0e-4
        for group in optimizer.param_groups:
            group["lr"] = lr
        optimizer.zero_grad(set_to_none=True)
        prediction = module._policy_logits({Columns.OBS: x})[:, :2]
        error = prediction - y
        component_weights = 1.0 + (tail_weight - 1.0) * (
            torch.abs(error.detach()) > tail_threshold
        ).to(dtype=error.dtype)
        weighted = w[:, None] * component_weights
        loss = torch.sum(weighted * torch.square(error)) / torch.sum(weighted)
        loss.backward()
        torch.nn.utils.clip_grad_norm_(parameters, 5.0)
        optimizer.step()
        scalar = float(loss.detach().cpu())
        if initial_loss is None:
            initial_loss = scalar
        final_loss = scalar
        if epoch == 1 or epoch % 1000 == 0 or epoch == epochs:
            print(
                f"[V9 causal diagnostic] {epoch}/{epochs} loss={scalar:.9g}",
                file=sys.stderr,
                flush=True,
            )
    module.eval()
    with torch.no_grad():
        predictions = (
            module._policy_logits({Columns.OBS: x})[:, :2]
            .detach()
            .cpu()
            .numpy()
            .astype(np.float32)
        )
    error = predictions.astype(np.float64) - teacher_means.astype(np.float64)
    return {
        "epochs": epochs,
        "residual_limits": [0.175, 0.15],
        "reset_weight": reset_weight,
        "tail_weight": tail_weight,
        "tail_threshold": tail_threshold,
        "initial_weighted_loss": initial_loss,
        "final_weighted_loss": final_loss,
        "rmse": float(np.sqrt(np.mean(np.square(error)))),
        "max_abs_error": float(np.max(np.abs(error))),
        "reset_max_abs_error": float(
            np.max(np.abs(error[reset_mask]), initial=0.0)
        ),
        "all_finite": bool(np.all(np.isfinite(predictions))),
    }


def main(argv: list[str] | None = None) -> int:
    import torch

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fit-epochs", type=int, default=0)
    parser.add_argument("--reset-weight", type=float, default=100.0)
    parser.add_argument("--tail-weight", type=float, default=1.0)
    parser.add_argument("--tail-threshold", type=float, default=0.0075)
    args = parser.parse_args(argv)
    if args.fit_epochs < 0:
        parser.error("--fit-epochs must be non-negative")
    if args.reset_weight < 1.0 or args.tail_weight < 1.0:
        parser.error("fit weights must be >= 1")
    if not 0.0 < args.tail_threshold < 1.0:
        parser.error("--tail-threshold must be in (0, 1)")

    students, legacy, reset_mask = load_views()
    teacher_views = students.copy()
    teacher_views[:, list(LOAD_CONTACT_INDICES)] = legacy[
        :, list(LOAD_CONTACT_INDICES)
    ]
    if (
        teacher_views[:, list(CAUSAL_INDICES)].tobytes(order="C")
        != students[:, list(CAUSAL_INDICES)].tobytes(order="C")
    ):
        raise V9CausalTeacherDiagnosticError("V26 causal columns drifted")

    module = RLModule.from_checkpoint(SOURCE_H0)
    module.eval()
    with torch.no_grad():
        student_logits = module._policy_logits(
            {Columns.OBS: torch.as_tensor(students, dtype=torch.float32)}
        )
        teacher_logits = module._policy_logits(
            {Columns.OBS: torch.as_tensor(teacher_views, dtype=torch.float32)}
        )
    student_means = np.ascontiguousarray(
        student_logits[:, :2].detach().cpu().numpy(), dtype=np.float32
    )
    teacher_means = np.ascontiguousarray(
        teacher_logits[:, :2].detach().cpu().numpy(), dtype=np.float32
    )
    delta = teacher_means.astype(np.float64) - student_means.astype(np.float64)
    reset_delta = np.abs(delta[reset_mask])
    report = {
        "schema_version": 1,
        "status": "DIAGNOSTIC_H0_PRIMARY_SPLIT_V9_CAUSAL_TEACHER",
        "sample_count": len(students),
        "reset_count": int(np.count_nonzero(reset_mask)),
        "target_semantics": (
            "H0(student_V26_with_only_10_11_replaced_by_analog_shadow)"
        ),
        "teacher_student_byte_exact_indices": list(CAUSAL_INDICES),
        "teacher_privileged_indices": list(LOAD_CONTACT_INDICES),
        "student_observations_sha256": _array_sha256(students),
        "teacher_views_sha256": _array_sha256(teacher_views),
        "teacher_means_sha256": _array_sha256(teacher_means),
        "target_equals_base": bool(
            teacher_means.tobytes(order="C") == student_means.tobytes(order="C")
        ),
        "base_to_teacher_rmse": float(np.sqrt(np.mean(np.square(delta)))),
        "base_to_teacher_max_abs": float(np.max(np.abs(delta))),
        "reset_max_abs": float(np.max(reset_delta, initial=0.0)),
        "required_delta_min": np.min(delta, axis=0).tolist(),
        "required_delta_max": np.max(delta, axis=0).tolist(),
        "finite": bool(
            np.all(np.isfinite(teacher_views))
            and np.all(np.isfinite(teacher_means))
        ),
    }
    if args.fit_epochs:
        report["fit_diagnostic"] = _fit_diagnostic(
            students=students,
            teacher_means=teacher_means,
            reset_mask=reset_mask,
            epochs=args.fit_epochs,
            reset_weight=args.reset_weight,
            tail_weight=args.tail_weight,
            tail_threshold=args.tail_threshold,
        )
    print(json.dumps(report, indent=2, sort_keys=True, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
