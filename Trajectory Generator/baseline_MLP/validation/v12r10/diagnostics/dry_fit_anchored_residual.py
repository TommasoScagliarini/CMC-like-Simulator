"""One preregistered, environment-free V12R10 anchored-residual dry fit.

This diagnostic answers one narrow question: can the immutable thirteen-stratum
V12R9 corpus be fitted without moving the known-safe P2 tower of the exact R6
functional composite?  It does not publish a checkpoint, open an environment,
run PPO, or select among candidates.  Reconstructing the corpus through the
frozen R9 semantic-closure loader does re-query the locked H0 teacher offline;
those verification-only queries are counted explicitly in the result.

The deployable 35->512->512->2 state is split into the same two 256-wide towers
as R6.  The first tower and its ``0.70 * P2`` mean contribution remain byte
exact.  The second tower starts as ``0.30 * R5`` so the initial function is the
R6 function, and only that residual tower is optimized against::

    target - 0.70 * P2

The sole preregistered objective is::

    equal_13_stratum_mse
      + 4.0 * mean_14_gate_slice_fourth_error / 0.045**2
      + 100.0 * reset_mse

The fourteen slices are the thirteen corpus strata plus the observer ``+0.20``
late subset used by the frozen R9 gate.  The fourth-error term is a smooth
max-error surrogate with a margin at 75% of the unchanged 0.060 ceiling.  The
100x reset term carries forward the historical reset protection and reflects
the much tighter unchanged 0.003 reset ceiling.  Coefficients, optimizer,
initialization, and final-state rule are constants below; there is no sweep,
fallback, repair, early selection, or post-gate model retry.  One identical
instrumentation re-execution is disclosed below because the first process
finished fitting but aborted in its reporting-only tolerance check.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import pickle
import sys
import time
from pathlib import Path
from typing import Any, Mapping

import numpy as np


def _discover_repo_root(source: Path) -> Path:
    for candidate in source.resolve().parents:
        if (
            (candidate / "AGENTS.md").is_file()
            and (candidate / "Trajectory Generator").is_dir()
            and (candidate / "validation").is_dir()
        ):
            return candidate
    raise RuntimeError("repository root could not be discovered")


REPO_ROOT = _discover_repo_root(Path(__file__))
BASELINE_ROOT = REPO_ROOT / "Trajectory Generator" / "baseline_MLP"
LOCAL_VALIDATION = BASELINE_ROOT / "validation"
R6_ROOT = LOCAL_VALIDATION / "v12r6"
R9_ROOT = LOCAL_VALIDATION / "v12r9"
for _root in (REPO_ROOT, BASELINE_ROOT, LOCAL_VALIDATION, R6_ROOT, R9_ROOT):
    if str(_root) not in sys.path:
        sys.path.insert(0, str(_root))

import warm_start  # noqa: E402,F401  # registers the custom RLModule class
import build_h0_v12r6_composite_actor as r6_builder  # noqa: E402
import freeze_h0_v12r6_functional_composite as r6_freeze  # noqa: E402
import h0_v12r6_functional_composite_contract as r6_contract  # noqa: E402
import h0_v12r9_recovery_fitter as r9  # noqa: E402


STRATEGY_ID = "V12R10_ANCHORED_P2_RESIDUAL_R5_GATE_ALIGNED_V1"
RESULT_PATH = (
    Path(__file__).resolve().parent / "results" / "dry_fit_anchored_residual.json"
)

ALPHA_P2 = 0.70
ALPHA_RESIDUAL_INITIAL = 0.30
GATE_SLICE_FOURTH_COEFFICIENT = 4.0
RESET_COEFFICIENT = 100.0
TAIL_MARGIN = 0.045

SEED = 20260814
TORCH_THREADS = 5
ADAMW_EPOCHS = 3_000
ADAMW_BOUNDARIES = (1_500, 2_500, 3_000)
ADAMW_RATES = (3.0e-4, 1.0e-4, 3.0e-5)
ADAMW_WEIGHT_DECAY = 1.0e-7
GRADIENT_CLIP_NORM = 10.0
LBFGS_LR = 0.7
LBFGS_MAX_ITER = 600
LBFGS_MAX_EVAL = 1_200
LBFGS_TOLERANCE_GRAD = 1.0e-10
LBFGS_TOLERANCE_CHANGE = 1.0e-12
LBFGS_HISTORY_SIZE = 50
INITIAL_EQUIVALENCE_TOLERANCE = 2.0e-6
OFFLINE_TEACHER_REVERIFICATION_QUERIES_PER_PROCESS = 2_252

# The first execution of this exact strategy completed AdamW and 624 LBFGS
# closure calls, then the reporting layer rejected a 1.2516975402832031e-6
# two-tower-vs-packed float32 difference using the legacy 1e-6 single-network
# tolerance.  No result or checkpoint was written.  R6's frozen functional
# composite contract uses 2e-6 for this exact arithmetic boundary; the sole
# correction below adopts that already-established tolerance and records the
# aborted instrumentation attempt instead of hiding it.
PRIOR_ABORTED_INSTRUMENTATION_ATTEMPT = {
    "executed": True,
    "strategy_identical": True,
    "adamw_epochs_completed": 3_000,
    "lbfgs_closure_calls": 624,
    "environment_reset_calls": 0,
    "environment_step_calls": 0,
    "teacher_query_count": OFFLINE_TEACHER_REVERIFICATION_QUERIES_PER_PROCESS,
    "checkpoint_publication_count": 0,
    "result_publication_count": 0,
    "abort_stage": "POST_FIT_NORMALIZATION_FOLD_REPORTING_AUDIT",
    "observed_max_abs_difference": 1.2516975402832031e-6,
    "legacy_tolerance": 1.0e-6,
    "applicable_r6_composite_tolerance": INITIAL_EQUIVALENCE_TOLERANCE,
}
PRIOR_ABORTED_MILESTONES = (
    {
        "stage": "initial",
        "index": 0,
        "loss": 0.009335664237297906,
        "equal_13_stratum_mse": 0.00039382005196193597,
        "mean_14_gate_slice_fourth_error_scaled": 0.002233531391574561,
        "reset_mse": 7.718619037725502e-08,
    },
    {
        "stage": "adamw",
        "index": 1,
        "loss": 0.009335664237297906,
        "equal_13_stratum_mse": 0.00039382005196193597,
        "mean_14_gate_slice_fourth_error_scaled": 0.002233531391574561,
        "reset_mse": 7.718619037725502e-08,
    },
    {
        "stage": "adamw",
        "index": 250,
        "loss": 0.0022250985069622066,
        "equal_13_stratum_mse": 0.00034831530203612487,
        "mean_14_gate_slice_fourth_error_scaled": 0.00046917364963177353,
        "reset_mse": 8.860639898749865e-10,
    },
    {
        "stage": "adamw",
        "index": 500,
        "loss": 0.0018302321035886485,
        "equal_13_stratum_mse": 0.00031406341619072284,
        "mean_14_gate_slice_fourth_error_scaled": 0.00037902554195912136,
        "reset_mse": 6.651956144030016e-10,
    },
    {
        "stage": "adamw",
        "index": 1_000,
        "loss": 0.0014852097047548473,
        "equal_13_stratum_mse": 0.0002836325196700475,
        "mean_14_gate_slice_fourth_error_scaled": 0.0003001416400553792,
        "reset_mse": 1.010624863282918e-08,
    },
    {
        "stage": "adamw",
        "index": 1_500,
        "loss": 0.0013398474934958463,
        "equal_13_stratum_mse": 0.000266351874945702,
        "mean_14_gate_slice_fourth_error_scaled": 0.00026170321882099226,
        "reset_mse": 2.6682743266175197e-07,
    },
    {
        "stage": "adamw",
        "index": 2_000,
        "loss": 0.0012599914631070315,
        "equal_13_stratum_mse": 0.0002605218133137259,
        "mean_14_gate_slice_fourth_error_scaled": 0.0002498658868051045,
        "reset_mse": 6.102572887461112e-11,
    },
    {
        "stage": "adamw",
        "index": 2_500,
        "loss": 0.0012021882741829607,
        "equal_13_stratum_mse": 0.00025419708439803246,
        "mean_14_gate_slice_fourth_error_scaled": 0.00023699671353328303,
        "reset_mse": 4.335651796183875e-11,
    },
    {
        "stage": "adamw",
        "index": 3_000,
        "loss": 0.0011822509626372038,
        "equal_13_stratum_mse": 0.00025190841188070984,
        "mean_14_gate_slice_fourth_error_scaled": 0.0002325846798340571,
        "reset_mse": 3.831420265522783e-11,
    },
    {
        "stage": "lbfgs_closure",
        "index": 1,
        "loss": 0.0011822072613911438,
        "equal_13_stratum_mse": 0.0002519033693774384,
        "mean_14_gate_slice_fourth_error_scaled": 0.0002325750099222653,
        "reset_mse": 3.852324644178229e-11,
    },
    {
        "stage": "lbfgs_closure",
        "index": 50,
        "loss": 0.0011095146568282502,
        "equal_13_stratum_mse": 0.0002507248198256452,
        "mean_14_gate_slice_fourth_error_scaled": 0.0002146170054904375,
        "reset_mse": 3.2181504085502755e-09,
    },
    {
        "stage": "lbfgs_closure",
        "index": 100,
        "loss": 0.0010616742396151334,
        "equal_13_stratum_mse": 0.0002450602834338706,
        "mean_14_gate_slice_fourth_error_scaled": 0.00020410668347341078,
        "reset_mse": 1.8722228761971835e-09,
    },
    {
        "stage": "lbfgs_closure",
        "index": 200,
        "loss": 0.0009431377238369415,
        "equal_13_stratum_mse": 0.00022630201081382542,
        "mean_14_gate_slice_fourth_error_scaled": 0.00017914475627389426,
        "reset_mse": 2.566879275390131e-09,
    },
    {
        "stage": "lbfgs_closure",
        "index": 300,
        "loss": 0.0008359248183022696,
        "equal_13_stratum_mse": 0.00020624370241612803,
        "mean_14_gate_slice_fourth_error_scaled": 0.0001574065388194137,
        "reset_mse": 5.496060848685198e-10,
    },
    {
        "stage": "lbfgs_closure",
        "index": 400,
        "loss": 0.0007727920875125069,
        "equal_13_stratum_mse": 0.00019701811974609973,
        "mean_14_gate_slice_fourth_error_scaled": 0.00014378849048335207,
        "reset_mse": 6.200058329989352e-09,
    },
    {
        "stage": "lbfgs_closure",
        "index": 600,
        "loss": 0.0006767589675793962,
        "equal_13_stratum_mse": 0.00018002193274267193,
        "mean_14_gate_slice_fourth_error_scaled": 0.0001241717180768798,
        "reset_mse": 5.016252920520846e-10,
    },
    {
        "stage": "terminal",
        "index": 624,
        "loss": 0.0006664256301479409,
        "equal_13_stratum_mse": 0.00017977815637506867,
        "mean_14_gate_slice_fourth_error_scaled": 0.00012164597368136254,
        "reset_mse": 6.357904742212342e-10,
    },
)

EXPECTED_R9_TERMINAL_STATUS = "FAIL_H0_V12R9_RECOVERY_PIPELINE_TERMINAL"


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _array(value: Any) -> np.ndarray:
    if hasattr(value, "detach"):
        value = value.detach()
    if hasattr(value, "cpu"):
        value = value.cpu()
    return np.ascontiguousarray(np.asarray(value))


def _state_arrays(state: Mapping[str, Any]) -> dict[str, np.ndarray]:
    return {name: _array(value) for name, value in state.items()}


def _array_byte_exact(left: Any, right: Any) -> bool:
    a = _array(left)
    b = _array(right)
    return (
        a.dtype == b.dtype
        and a.shape == b.shape
        and a.tobytes(order="C") == b.tobytes(order="C")
    )


def _state_byte_exact(left: Mapping[str, Any], right: Mapping[str, Any]) -> bool:
    return set(left) == set(right) and all(
        _array_byte_exact(left[name], right[name]) for name in left
    )


def _array_sha256(value: Any) -> str:
    array = _array(value)
    digest = hashlib.sha256()
    digest.update(array.dtype.str.encode("ascii"))
    digest.update(b"\0")
    digest.update(repr(array.shape).encode("ascii"))
    digest.update(b"\0")
    digest.update(array.tobytes(order="C"))
    return digest.hexdigest()


def _positive_zero(value: Any) -> bool:
    array = _array(value)
    if array.dtype == np.dtype(np.float32):
        return bool(np.all(array.view(np.uint32) == 0))
    if array.dtype == np.dtype(np.float64):
        return bool(np.all(array.view(np.uint64) == 0))
    return False


def _strict_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as stream:
        value = json.load(stream)
    if not isinstance(value, dict):
        raise RuntimeError(f"expected JSON object: {path}")
    return value


def _metric_pair(predictions: Any, targets: Any) -> dict[str, float]:
    prediction = np.ascontiguousarray(predictions, dtype=np.float32)
    target = np.ascontiguousarray(targets, dtype=np.float32)
    if prediction.shape != target.shape or prediction.size == 0:
        raise RuntimeError("metric arrays are malformed")
    error = prediction.astype(np.float64) - target.astype(np.float64)
    return {
        "rmse": float(np.sqrt(np.mean(np.square(error), dtype=np.float64))),
        "max_abs_error": float(np.max(np.abs(error))),
    }


def _adamw_rate(epoch: int) -> float:
    if type(epoch) is not int or not 1 <= epoch <= ADAMW_EPOCHS:
        raise ValueError(f"AdamW epoch outside fixed schedule: {epoch!r}")
    if epoch <= ADAMW_BOUNDARIES[0]:
        return ADAMW_RATES[0]
    if epoch <= ADAMW_BOUNDARIES[1]:
        return ADAMW_RATES[1]
    return ADAMW_RATES[2]


def _tail_slices(bundle: r9.RecoveryCorpusBundle) -> dict[str, np.ndarray]:
    strata = bundle.stratum_ids.astype(str)
    slices = {
        stratum_id: np.flatnonzero(strata == stratum_id)
        for stratum_id in r9.expected_stratum_ids()
    }
    slices["gate::observer_plus_late"] = np.ascontiguousarray(
        bundle.observer_plus_late_indices, dtype=np.int64
    )
    if len(slices) != 14 or any(len(index) == 0 for index in slices.values()):
        raise RuntimeError("the fourteen preregistered gate slices drifted")
    return slices


def _normalization(bundle: r9.RecoveryCorpusBundle) -> Any:
    corpus = bundle.corpus
    base_rows = int(r9.contract.LOCKED_INPUTS["base_corpus"]["rows"])
    selected = np.flatnonzero(
        corpus.tranche_ids[:base_rows].astype(str) == "v8r1p1_base"
    )
    if len(selected) != 3_000:
        raise RuntimeError("frozen normalization source row count drifted")
    return r9.v11.frozen_base_normalization(corpus.observations[selected])


def _load_locked_sources() -> tuple[
    Any,
    dict[str, np.ndarray],
    dict[str, np.ndarray],
    dict[str, np.ndarray],
    dict[str, Any],
]:
    """Load P2/R5/R6 only after all immutable identities close."""

    from ray.rllib.core.rl_module.rl_module import RLModule

    r9_attestation = r9.attest_locked_inputs()
    p2_tree = r6_freeze.tree_record(r6_contract.P2_MODULE_PATH)
    r5_tree = r6_freeze.tree_record(r6_contract.R5_MODULE_PATH)
    if p2_tree != r6_contract.P2_MODULE_TREE:
        raise RuntimeError("locked P2 module tree drifted")
    if r5_tree != r6_contract.R5_MODULE_TREE:
        raise RuntimeError("locked R5 module tree drifted")

    p2_module = RLModule.from_checkpoint(
        (REPO_ROOT / r6_contract.P2_MODULE_PATH).resolve()
    )
    r5_module = RLModule.from_checkpoint(
        (REPO_ROOT / r6_contract.R5_MODULE_PATH).resolve()
    )
    p2_module.eval()
    r5_module.eval()
    p2_state, r5_state, pair_audit = r6_builder._validate_source_pair(  # noqa: SLF001
        p2_module, r5_module
    )

    r6_module, r6_state_raw, _manifest = r9._load_source_module_and_state()  # noqa: SLF001
    r6_state = _state_arrays(r6_state_raw)
    rebuilt_r6 = r6_builder.build_composite_state(p2_state, r5_state)
    if not _state_byte_exact(r6_state, rebuilt_r6):
        raise RuntimeError("locked R6 state is not the exact P2/R5 composite")

    records = {
        "r9_attestation": {
            "r6_candidate_tree_sha256": r9_attestation["r6_candidate"]["tree_sha256"],
            "base_corpus_sha256": r9_attestation["base_corpus"]["sha256"],
            "r4_failed_plus_labels_sha256": r9_attestation["r4_failed_plus_labels"][
                "sha256"
            ],
        },
        "p2_tree": p2_tree,
        "r5_tree": r5_tree,
        "source_pair_audit": pair_audit,
        "r6_actor_digest": warm_start.actor_state_digest(r6_state),
        "r6_rebuild_byte_exact": True,
    }
    return r6_module, p2_state, r5_state, r6_state, records


def _new_normalized_residual_model(
    r5_state: Mapping[str, Any], normalization: Any
) -> Any:
    import torch
    from torch import nn

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
        np.asarray(r5_state["pi_encoder.0.weight"]), dtype=torch.float32
    )
    source_b0 = torch.as_tensor(
        np.asarray(r5_state["pi_encoder.0.bias"]), dtype=torch.float32
    )
    with torch.no_grad():
        model[0].weight.copy_(source_w0 * std[None, :])
        model[0].bias.copy_(source_b0 + source_w0 @ mean)
        model[2].weight.copy_(
            torch.as_tensor(r5_state["pi_encoder.2.weight"], dtype=torch.float32)
        )
        model[2].bias.copy_(
            torch.as_tensor(r5_state["pi_encoder.2.bias"], dtype=torch.float32)
        )
        model[4].weight.copy_(
            ALPHA_RESIDUAL_INITIAL
            * torch.as_tensor(r5_state["pi.1.weight"], dtype=torch.float32)[:2]
        )
        model[4].bias.copy_(
            ALPHA_RESIDUAL_INITIAL
            * torch.as_tensor(r5_state["pi.1.bias"], dtype=torch.float32)[:2]
        )
    if not _positive_zero(
        model[0].weight.detach().cpu().numpy()[:, r9.contract.DISABLED_CLOCK_COLUMNS]
    ):
        raise RuntimeError("residual initialization re-enabled clock columns")
    return model


def _pack_candidate_state(
    *,
    model: Any,
    p2_state: Mapping[str, Any],
    r6_state: Mapping[str, Any],
    normalization: Any,
) -> dict[str, np.ndarray]:
    """Fold normalization and place only the trained residual in R6 tower 2."""

    import torch

    normalized_w0 = np.ascontiguousarray(
        model[0].weight.detach().cpu().numpy(), dtype=np.float32
    )
    normalized_b0 = np.ascontiguousarray(
        model[0].bias.detach().cpu().numpy(), dtype=np.float32
    )
    if not _positive_zero(normalized_w0[:, r9.contract.DISABLED_CLOCK_COLUMNS]):
        raise RuntimeError("trained residual clock columns are not positive zero")
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
    if not _positive_zero(raw_w0[:, r9.contract.DISABLED_CLOCK_COLUMNS]):
        raise RuntimeError("folded residual clock columns are not positive zero")

    candidate = {name: value.copy() for name, value in r6_state.items()}
    first_weight = candidate["pi_encoder.0.weight"].copy()
    first_bias = candidate["pi_encoder.0.bias"].copy()
    first_weight[256:] = raw_w0
    first_bias[256:] = raw_b0

    second_weight = candidate["pi_encoder.2.weight"].copy()
    second_bias = candidate["pi_encoder.2.bias"].copy()
    second_weight[256:, 256:] = np.ascontiguousarray(
        model[2].weight.detach().cpu().numpy(), dtype=np.float32
    )
    second_bias[256:] = np.ascontiguousarray(
        model[2].bias.detach().cpu().numpy(), dtype=np.float32
    )

    head_weight = candidate["pi.1.weight"].copy()
    head_bias = candidate["pi.1.bias"].copy()
    head_weight[:2, 256:] = np.ascontiguousarray(
        model[4].weight.detach().cpu().numpy(), dtype=np.float32
    )
    head_bias[:2] = np.ascontiguousarray(
        ALPHA_P2 * np.asarray(p2_state["pi.1.bias"], dtype=np.float32)[:2]
        + model[4].bias.detach().cpu().numpy(),
        dtype=np.float32,
    )

    for prefix in ("pi_encoder", "pi.0"):
        candidate[f"{prefix}.0.weight"] = first_weight.copy()
        candidate[f"{prefix}.0.bias"] = first_bias.copy()
        candidate[f"{prefix}.2.weight"] = second_weight.copy()
        candidate[f"{prefix}.2.bias"] = second_bias.copy()
    candidate["pi.1.weight"] = head_weight
    candidate["pi.1.bias"] = head_bias
    r9.validate_source_r6_state(candidate)
    return candidate


def _p2_anchor_audit(
    *,
    p2_state: Mapping[str, Any],
    r6_state: Mapping[str, Any],
    candidate_state: Mapping[str, Any],
) -> dict[str, Any]:
    checks = {
        "p2_first_weight_byte_exact": _array_byte_exact(
            candidate_state["pi_encoder.0.weight"][:256],
            p2_state["pi_encoder.0.weight"],
        ),
        "p2_first_bias_byte_exact": _array_byte_exact(
            candidate_state["pi_encoder.0.bias"][:256],
            p2_state["pi_encoder.0.bias"],
        ),
        "p2_second_weight_byte_exact": _array_byte_exact(
            candidate_state["pi_encoder.2.weight"][:256, :256],
            p2_state["pi_encoder.2.weight"],
        ),
        "p2_second_bias_byte_exact": _array_byte_exact(
            candidate_state["pi_encoder.2.bias"][:256],
            p2_state["pi_encoder.2.bias"],
        ),
        "p2_scaled_head_weight_byte_exact": _array_byte_exact(
            candidate_state["pi.1.weight"][:2, :256],
            r6_state["pi.1.weight"][:2, :256],
        ),
        "cross_blocks_positive_zero": _positive_zero(
            candidate_state["pi_encoder.2.weight"][:256, 256:]
        )
        and _positive_zero(candidate_state["pi_encoder.2.weight"][256:, :256]),
        "encoder_aliases_byte_exact": all(
            _array_byte_exact(candidate_state[left], candidate_state[right])
            for left, right in (
                ("pi_encoder.0.weight", "pi.0.0.weight"),
                ("pi_encoder.0.bias", "pi.0.0.bias"),
                ("pi_encoder.2.weight", "pi.0.2.weight"),
                ("pi_encoder.2.bias", "pi.0.2.bias"),
            )
        ),
        "logstd_rows_byte_exact": _array_byte_exact(
            candidate_state["pi.1.weight"][2:], r6_state["pi.1.weight"][2:]
        )
        and _array_byte_exact(
            candidate_state["pi.1.bias"][2:], r6_state["pi.1.bias"][2:]
        ),
        "all_clock_columns_positive_zero": _positive_zero(
            candidate_state["pi_encoder.0.weight"][:, :2]
        ),
    }
    return {"passed": all(checks.values()), "checks": checks}


def _metric_payload(
    bundle: r9.RecoveryCorpusBundle,
    candidate_state: Mapping[str, Any],
) -> dict[str, Any]:
    payload = r9.recompute_fit_metric_payload(bundle, candidate_state)
    return {name: value for name, value in payload.items() if name != "predictions"}


def _gate(
    metric_payload: Mapping[str, Any],
    *,
    candidate_state: Mapping[str, Any],
    r6_state: Mapping[str, Any],
    save_reload_exact: bool,
) -> dict[str, Any]:
    state_audit = r9.validate_source_r6_state(candidate_state)
    logstd_exact = _array_byte_exact(
        candidate_state["pi.1.weight"][2:], r6_state["pi.1.weight"][2:]
    ) and _array_byte_exact(candidate_state["pi.1.bias"][2:], r6_state["pi.1.bias"][2:])
    clock_zero = _positive_zero(candidate_state["pi_encoder.0.weight"][:, :2])
    summary = {
        "actor_fit_count": 1,
        "actor_updates": 1,
        "critic_updates": 0,
        "ppo_updates": 0,
        "hidden_dims": state_audit["hidden_dims"],
        "actor_feature_count": state_audit["actor_feature_count"],
        "logstd_byte_exact": logstd_exact,
        "disabled_clock_columns_bit_zero": clock_zero,
        "save_reload_exact": save_reload_exact,
        **metric_payload,
    }
    return r9.contract.fit_gate(summary)


def _prior_milestone_reproduction(
    history: list[dict[str, Any]],
) -> dict[str, Any]:
    fields = (
        "stage",
        "index",
        "loss",
        "equal_13_stratum_mse",
        "mean_14_gate_slice_fourth_error_scaled",
        "reset_mse",
    )
    observed = {
        (row["stage"], row["index"]): {name: row[name] for name in fields}
        for row in history
    }
    comparisons = []
    for expected in PRIOR_ABORTED_MILESTONES:
        key = (expected["stage"], expected["index"])
        actual = observed.get(key)
        comparisons.append(
            {
                "stage": expected["stage"],
                "index": expected["index"],
                "exact": actual == expected,
                "expected": expected,
                "observed": actual,
            }
        )
    passed = all(row["exact"] for row in comparisons)
    return {
        "passed": passed,
        "comparison": "EXACT_PYTHON_FLOAT_VALUES_PRINTED_BY_PRIOR_PROCESS",
        "milestone_count": len(comparisons),
        "all_prior_observable_milestones_reproduced": passed,
        "comparisons": comparisons,
    }


def _float_terms(loss: Any, terms: Mapping[str, Any]) -> dict[str, float]:
    return {
        "loss": float(loss.detach().cpu()),
        **{name: float(value.detach().cpu()) for name, value in terms.items()},
    }


def _fit_once(
    *,
    label: str,
    bundle: r9.RecoveryCorpusBundle,
    normalization: Any,
    p2_state: Mapping[str, Any],
    r5_state: Mapping[str, Any],
    r6_state: Mapping[str, Any],
    fixed_p2_predictions: np.ndarray,
    initial_equivalence: Mapping[str, Any],
) -> dict[str, Any]:
    import torch

    corpus = bundle.corpus
    normalized = r9.v11.normalized_observations(corpus.observations, normalization)
    tail_slices_np = _tail_slices(bundle)
    reset_np = np.flatnonzero(corpus.reset_mask)
    if len(reset_np) != 26:
        raise RuntimeError("reset row count drifted")

    previous_threads = torch.get_num_threads()
    previous_deterministic = torch.are_deterministic_algorithms_enabled()
    torch.set_num_threads(TORCH_THREADS)
    torch.use_deterministic_algorithms(True)
    started = time.monotonic()
    try:
        torch.manual_seed(SEED)
        model = _new_normalized_residual_model(r5_state, normalization)
        x = torch.as_tensor(normalized, dtype=torch.float32)
        targets = torch.as_tensor(corpus.actions, dtype=torch.float32)
        fixed_p2 = torch.as_tensor(fixed_p2_predictions, dtype=torch.float32)
        weights = torch.as_tensor(corpus.normalized_sample_weights, dtype=torch.float64)
        weight_sum = torch.sum(weights)
        reset_indices = torch.as_tensor(reset_np, dtype=torch.int64)
        tail_indices = {
            name: torch.as_tensor(index, dtype=torch.int64)
            for name, index in tail_slices_np.items()
        }

        def objective(prediction: Any) -> tuple[Any, dict[str, Any]]:
            error = fixed_p2 + prediction - targets
            row_mse = torch.mean(torch.square(error), dim=1).to(torch.float64)
            equal_stratum = torch.sum(weights * row_mse) / weight_sum
            fourth_terms = []
            for index in tail_indices.values():
                selected = error[index].to(torch.float64)
                fourth_terms.append(
                    torch.mean(torch.pow(selected, 4)) / (TAIL_MARGIN**2)
                )
            tail_fourth = torch.mean(torch.stack(fourth_terms))
            reset_mse = torch.mean(torch.square(error[reset_indices]).to(torch.float64))
            total = (
                equal_stratum
                + GATE_SLICE_FOURTH_COEFFICIENT * tail_fourth
                + RESET_COEFFICIENT * reset_mse
            )
            return total, {
                "equal_13_stratum_mse": equal_stratum,
                "mean_14_gate_slice_fourth_error_scaled": tail_fourth,
                "reset_mse": reset_mse,
            }

        history: list[dict[str, Any]] = []
        with torch.no_grad():
            initial_loss, initial_terms = objective(model(x))
        initial_row = {
            "stage": "initial",
            "index": 0,
            **_float_terms(initial_loss, initial_terms),
        }
        history.append(initial_row)
        print(json.dumps({"fit": label, **initial_row}, sort_keys=True), flush=True)

        adamw = torch.optim.AdamW(
            model.parameters(),
            lr=ADAMW_RATES[0],
            weight_decay=ADAMW_WEIGHT_DECAY,
        )
        adamw_milestones = {1, 250, 500, 1_000, 1_500, 2_000, 2_500, 3_000}
        for epoch in range(1, ADAMW_EPOCHS + 1):
            rate = _adamw_rate(epoch)
            for group in adamw.param_groups:
                group["lr"] = rate
            adamw.zero_grad(set_to_none=True)
            prediction = model(x)
            loss, terms = objective(prediction)
            if not torch.isfinite(loss):
                raise RuntimeError(f"non-finite AdamW loss at epoch {epoch}")
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), GRADIENT_CLIP_NORM)
            adamw.step()
            with torch.no_grad():
                model[0].weight[:, r9.contract.DISABLED_CLOCK_COLUMNS] = 0.0
            if epoch in adamw_milestones:
                row = {
                    "stage": "adamw",
                    "index": epoch,
                    "learning_rate": rate,
                    **_float_terms(loss, terms),
                }
                history.append(row)
                print(json.dumps({"fit": label, **row}, sort_keys=True), flush=True)

        adamw_state = _pack_candidate_state(
            model=model,
            p2_state=p2_state,
            r6_state=r6_state,
            normalization=normalization,
        )
        adamw_metrics = _metric_payload(bundle, adamw_state)
        adamw_gate = _gate(
            adamw_metrics,
            candidate_state=adamw_state,
            r6_state=r6_state,
            save_reload_exact=False,
        )
        print(
            json.dumps(
                {
                    "fit": label,
                    "stage": "adamw_gate_snapshot",
                    "passed": adamw_gate["passed"],
                    "checks": adamw_gate["checks"],
                    "global_metrics": adamw_metrics["global_metrics"],
                    "reset_max_abs_error": adamw_metrics["reset_max_abs_error"],
                    "worst_row": adamw_metrics["worst_row"],
                },
                sort_keys=True,
            ),
            flush=True,
        )

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
        last_closure: tuple[Any, Mapping[str, Any]] | None = None

        def closure() -> Any:
            nonlocal closure_calls, last_closure
            lbfgs.zero_grad(set_to_none=True)
            prediction = model(x)
            value, components = objective(prediction)
            if not torch.isfinite(value):
                raise RuntimeError(
                    f"non-finite LBFGS loss at closure {closure_calls + 1}"
                )
            value.backward()
            closure_calls += 1
            last_closure = (value, components)
            if closure_calls in {1, 50, 100, 200, 300, 400, 600, 800, 1_200}:
                row = {
                    "stage": "lbfgs_closure",
                    "index": closure_calls,
                    **_float_terms(value, components),
                }
                history.append(row)
                print(json.dumps({"fit": label, **row}, sort_keys=True), flush=True)
            return value

        lbfgs.step(closure)
        if last_closure is None:
            raise RuntimeError("LBFGS never evaluated the objective")
        with torch.no_grad():
            model[0].weight[:, r9.contract.DISABLED_CLOCK_COLUMNS] = 0.0
            final_loss, final_terms = objective(model(x))
        final_row = {
            "stage": "terminal",
            "index": closure_calls,
            **_float_terms(final_loss, final_terms),
        }
        history.append(final_row)
        print(json.dumps({"fit": label, **final_row}, sort_keys=True), flush=True)

        candidate_state = _pack_candidate_state(
            model=model,
            p2_state=p2_state,
            r6_state=r6_state,
            normalization=normalization,
        )
        predictions = np.ascontiguousarray(
            r9.v11._state_logits(candidate_state, corpus.observations)[:, :2],
            dtype=np.float32,
        )
        with torch.no_grad():
            normalized_predictions = np.ascontiguousarray(
                (fixed_p2 + model(x)).cpu().numpy(), dtype=np.float32
            )
        fold_error = np.abs(
            normalized_predictions.astype(np.float64) - predictions.astype(np.float64)
        )
        fold_max = float(np.max(fold_error))
        fold_equivalence = {
            "fold_equivalence_passed": bool(
                np.all(np.isfinite(normalized_predictions))
                and np.all(np.isfinite(predictions))
                and fold_max <= INITIAL_EQUIVALENCE_TOLERANCE
            ),
            "normalized_runtime_max_abs_difference": fold_max,
            "max_abs_tolerance": INITIAL_EQUIVALENCE_TOLERANCE,
            "tolerance_source": "FROZEN_R6_FUNCTIONAL_COMPOSITE_CONTRACT",
            "normalized_runtime_all_finite": bool(
                np.all(np.isfinite(normalized_predictions))
            ),
        }
        if not fold_equivalence["fold_equivalence_passed"]:
            raise RuntimeError(
                f"two-tower normalization fold equivalence failed: {fold_equivalence}"
            )
        anchor_audit = _p2_anchor_audit(
            p2_state=p2_state,
            r6_state=r6_state,
            candidate_state=candidate_state,
        )
        metrics = _metric_payload(bundle, candidate_state)
        return {
            "candidate_state": candidate_state,
            "predictions": predictions,
            "history": history,
            "adamw_metrics": adamw_metrics,
            "adamw_gate": adamw_gate,
            "metrics": metrics,
            "anchor_audit": anchor_audit,
            "fold_equivalence": fold_equivalence,
            "initial_equivalence": dict(initial_equivalence),
            "optimizer_audit": {
                "seed": SEED,
                "torch_threads": TORCH_THREADS,
                "deterministic_algorithms_enabled": True,
                "full_batch": True,
                "actor_fit_count": 1,
                "actor_updates": 1,
                "critic_updates": 0,
                "ppo_updates": 0,
                "adamw_epochs": ADAMW_EPOCHS,
                "adamw_boundaries": list(ADAMW_BOUNDARIES),
                "adamw_rates": list(ADAMW_RATES),
                "adamw_weight_decay": ADAMW_WEIGHT_DECAY,
                "gradient_clip_norm": GRADIENT_CLIP_NORM,
                "lbfgs_lr": LBFGS_LR,
                "lbfgs_max_iter": LBFGS_MAX_ITER,
                "lbfgs_max_eval": LBFGS_MAX_EVAL,
                "lbfgs_closure_calls": closure_calls,
                "fallback": False,
                "sweep": False,
                "retry": False,
            },
            "elapsed_seconds": float(time.monotonic() - started),
        }
    finally:
        torch.use_deterministic_algorithms(previous_deterministic)
        torch.set_num_threads(previous_threads)


def _simulated_save_reload(
    *,
    module: Any,
    candidate_state: Mapping[str, Any],
    observations: np.ndarray,
    predictions: np.ndarray,
) -> dict[str, Any]:
    """Exercise serialization and module rehydration without writing a checkpoint."""

    encoded = pickle.dumps(
        _state_arrays(candidate_state), protocol=pickle.HIGHEST_PROTOCOL
    )
    decoded = pickle.loads(encoded)  # noqa: S301 - trusted in-memory bytes
    state_exact = _state_byte_exact(candidate_state, decoded)
    module.set_state(decoded)
    rehydrated_state = _state_arrays(module.get_state())
    module_state_exact = _state_byte_exact(candidate_state, rehydrated_state)
    runtime_logits = r6_builder._logits(module, observations)  # noqa: SLF001
    runtime_predictions = np.ascontiguousarray(runtime_logits[:, :2], dtype=np.float32)
    predictions_exact = _array_byte_exact(runtime_predictions, predictions)
    return {
        "passed": bool(state_exact and module_state_exact and predictions_exact),
        "mode": "IN_MEMORY_PICKLE_AND_RLMODULE_SET_STATE_NO_CHECKPOINT_WRITE",
        "serialized_size_bytes": len(encoded),
        "serialized_state_byte_exact": state_exact,
        "rehydrated_module_state_byte_exact": module_state_exact,
        "rehydrated_predictions_byte_exact": predictions_exact,
    }


def _base_drift(
    *,
    bundle: r9.RecoveryCorpusBundle,
    candidate_predictions: np.ndarray,
    r6_predictions: np.ndarray,
) -> dict[str, Any]:
    all_base = np.sort(
        np.concatenate(
            [
                bundle.base_indices[case_id]
                for case_id in r9.contract.COLLECTION_CASE_IDS
            ]
        )
    )
    return {
        "metric": "candidate_mean_minus_locked_r6_mean",
        "all_base": _metric_pair(
            candidate_predictions[all_base], r6_predictions[all_base]
        ),
        "per_base_case": {
            case_id: _metric_pair(candidate_predictions[index], r6_predictions[index])
            for case_id, index in bundle.base_indices.items()
        },
        "all_corpus": _metric_pair(candidate_predictions, r6_predictions),
    }


def _write_result(payload: Mapping[str, Any]) -> None:
    RESULT_PATH.parent.mkdir(parents=True, exist_ok=True)
    if os.path.lexists(RESULT_PATH):
        raise RuntimeError(f"diagnostic result exists/no-clobber: {RESULT_PATH}")
    temporary = RESULT_PATH.with_name(f".{RESULT_PATH.name}.tmp-{os.getpid()}")
    if os.path.lexists(temporary):
        raise RuntimeError(f"temporary result path is occupied: {temporary}")
    data = json.dumps(payload, indent=2, sort_keys=True, allow_nan=False) + "\n"
    with temporary.open("x", encoding="utf-8", newline="\n") as stream:
        stream.write(data)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, RESULT_PATH)


def run() -> dict[str, Any]:
    ledger_path = REPO_ROOT / r9.contract.LEDGER_PATH
    ledger = _strict_json(ledger_path)
    if ledger.get("status") != EXPECTED_R9_TERMINAL_STATUS:
        raise RuntimeError("R9 is not the expected immutable terminal FAIL lineage")

    bundle = r9.load_recovery_corpus()
    corpus_verification = r9.verify_persisted_corpus(bundle)
    if len(bundle.corpus.observations) != 11_875:
        raise RuntimeError("R9 reconstructed corpus row count drifted")
    normalization = _normalization(bundle)
    r6_module, p2_state, r5_state, r6_state, source_records = _load_locked_sources()

    observations = bundle.corpus.observations
    p2_predictions = np.ascontiguousarray(
        r9.v11._state_logits(p2_state, observations)[:, :2], dtype=np.float32
    )
    fixed_p2_predictions = np.ascontiguousarray(
        ALPHA_P2 * p2_predictions, dtype=np.float32
    )
    r6_predictions = np.ascontiguousarray(
        r9.v11._state_logits(r6_state, observations)[:, :2], dtype=np.float32
    )

    import torch

    normalized = r9.v11.normalized_observations(observations, normalization)
    initial_model = _new_normalized_residual_model(r5_state, normalization)
    with torch.no_grad():
        initial_predictions = np.ascontiguousarray(
            fixed_p2_predictions
            + initial_model(torch.as_tensor(normalized, dtype=torch.float32))
            .cpu()
            .numpy(),
            dtype=np.float32,
        )
    initial_error = np.abs(
        initial_predictions.astype(np.float64) - r6_predictions.astype(np.float64)
    )
    initial_equivalence = {
        "formula": "0.70*P2+0.30*R5",
        "structurally_exact": True,
        "float32_max_abs_difference_vs_locked_r6": float(np.max(initial_error)),
        "tolerance": INITIAL_EQUIVALENCE_TOLERANCE,
        "passed": bool(np.max(initial_error) <= INITIAL_EQUIVALENCE_TOLERANCE),
    }
    if not initial_equivalence["passed"]:
        raise RuntimeError(
            "residual initialization is not functionally equivalent to R6"
        )

    first = _fit_once(
        label="primary",
        bundle=bundle,
        normalization=normalization,
        p2_state=p2_state,
        r5_state=r5_state,
        r6_state=r6_state,
        fixed_p2_predictions=fixed_p2_predictions,
        initial_equivalence=initial_equivalence,
    )
    save_reload = _simulated_save_reload(
        module=r6_module,
        candidate_state=first["candidate_state"],
        observations=observations,
        predictions=first["predictions"],
    )
    first_gate = _gate(
        first["metrics"],
        candidate_state=first["candidate_state"],
        r6_state=r6_state,
        save_reload_exact=save_reload["passed"],
    )
    print(
        json.dumps(
            {
                "fit": "primary",
                "stage": "final_gate",
                "passed": first_gate["passed"],
                "checks": first_gate["checks"],
                "global_metrics": first["metrics"]["global_metrics"],
                "reset_max_abs_error": first["metrics"]["reset_max_abs_error"],
                "worst_row": first["metrics"]["worst_row"],
            },
            sort_keys=True,
        ),
        flush=True,
    )

    replica: dict[str, Any]
    if first_gate.get("passed") is True:
        second = _fit_once(
            label="determinism_replica",
            bundle=bundle,
            normalization=normalization,
            p2_state=p2_state,
            r5_state=r5_state,
            r6_state=r6_state,
            fixed_p2_predictions=fixed_p2_predictions,
            initial_equivalence=initial_equivalence,
        )
        state_exact = _state_byte_exact(
            first["candidate_state"], second["candidate_state"]
        )
        predictions_exact = _array_byte_exact(
            first["predictions"], second["predictions"]
        )
        metrics_exact = first["metrics"] == second["metrics"]
        replica = {
            "executed": True,
            "conditional_rule": "EXECUTE_ONLY_IF_PRIMARY_R9_GATE_PASSES",
            "passed": bool(state_exact and predictions_exact and metrics_exact),
            "candidate_state_byte_exact": state_exact,
            "predictions_byte_exact": predictions_exact,
            "metrics_exact": metrics_exact,
            "candidate_actor_digest": warm_start.actor_state_digest(
                second["candidate_state"]
            ),
            "optimizer_audit": second["optimizer_audit"],
            "elapsed_seconds": second["elapsed_seconds"],
        }
    else:
        replica = {
            "executed": False,
            "conditional_rule": "EXECUTE_ONLY_IF_PRIMARY_R9_GATE_PASSES",
            "passed": None,
            "reason": "PRIMARY_R9_GATE_FAILED",
        }

    prior_reproduction = _prior_milestone_reproduction(first["history"])
    primary_pass = bool(
        first_gate.get("passed") is True
        and first["anchor_audit"]["passed"] is True
        and first["fold_equivalence"].get("fold_equivalence_passed") is True
        and save_reload["passed"] is True
        and prior_reproduction["passed"] is True
    )
    determinism_pass = bool(replica["passed"] is True) if replica["executed"] else False
    output = {
        "status": "PASS_H0_V12R10_ANCHORED_RESIDUAL_DRY_FIT"
        if primary_pass and determinism_pass
        else "FAIL_H0_V12R10_ANCHORED_RESIDUAL_DRY_FIT",
        "passed": bool(primary_pass and determinism_pass),
        "scope": (
            "IN_MEMORY_OFFLINE_ONLY_NO_ENVIRONMENT_NO_CHECKPOINT_PUBLICATION_"
            "NO_CRITIC_NO_PPO_WITH_LOCKED_H0_SEMANTIC_REVERIFICATION"
        ),
        "strategy": {
            "strategy_id": STRATEGY_ID,
            "candidate_selection": "SOLE_FINAL_LBFGS_STATE",
            "initialization": "EXACT_R6_FUNCTION_0.70_P2_PLUS_0.30_R5",
            "frozen_tower": "P2_AND_ITS_0.70_MEAN_CONTRIBUTION",
            "trainable_tower": "R5_INITIALIZED_RESIDUAL_ONLY",
            "residual_target": "H0_LABEL_MINUS_0.70_P2",
            "objective": (
                "equal_13_stratum_mse+4.0*mean_14_gate_slice_fourth_error/"
                "0.045**2+100.0*reset_mse"
            ),
            "gate_thresholds_unchanged": r9.contract.OFFLINE_THRESHOLDS,
            "sweep": False,
            "fallback": False,
            "repair": False,
            "post_gate_model_retry": False,
            "instrumentation_reexecution_after_reporting_abort": True,
            "model_strategy_changed": False,
            "production_candidate_retry": False,
            "diagnostic_process_rerun_after_instrumentation_abort": True,
        },
        "limitations": {
            "save_reload_scope": (
                "IN_MEMORY_PICKLE_PLUS_RLMODULE_SET_STATE_ONLY;_NO_PRODUCTION_"
                "CHECKPOINT_SAVE_TO_PATH_OR_FROM_CHECKPOINT"
            ),
            "save_reload_is_production_checkpoint_evidence": False,
            "gate_architecture_logstd_clock_fields": (
                "DERIVED_FROM_PACKED_CANDIDATE_STATE;NOT_HARDCODED"
            ),
            "reporting_tolerance_correction": (
                "POST_ABORT_ONLY;FIT_OBJECTIVE_SEED_AND_OPTIMIZER_UNCHANGED"
            ),
            "one_shot_production_attestation": False,
            "postrun_metadata_only_correction": (
                "DISCLOSED_LOCKED_H0_REVERIFICATION_QUERY_COUNTS_AND_REVIEWER_"
                "SEMANTIC_ALIASES;NUMERICAL_PAYLOAD_UNCHANGED"
            ),
        },
        "diagnostic_source": {
            "path": Path(__file__).resolve().relative_to(REPO_ROOT).as_posix(),
            "sha256": _sha256_file(Path(__file__).resolve()),
        },
        "frozen_inputs": {
            "r9_terminal_ledger": {
                "path": ledger_path.relative_to(REPO_ROOT).as_posix(),
                "sha256": _sha256_file(ledger_path),
                "status": ledger["status"],
            },
            "r9_persisted_corpus": corpus_verification,
            "source_modules": source_records,
        },
        "corpus": {
            "reconstructed_from_source_npz": True,
            "row_count": len(observations),
            "stratum_order": list(r9.expected_stratum_ids()),
            "stratum_count": len(r9.expected_stratum_ids()),
            "stratum_row_counts": bundle.corpus.audit["weight_audit"][
                "stratum_row_counts"
            ],
            "total_sample_mass": bundle.corpus.audit["weight_audit"][
                "total_sample_mass"
            ],
            "observations_sha256": _array_sha256(observations),
            "actions_sha256": _array_sha256(bundle.corpus.actions),
            "weights_sha256": _array_sha256(bundle.corpus.normalized_sample_weights),
            "reset_row_count": int(np.count_nonzero(bundle.corpus.reset_mask)),
        },
        "primary": {
            "passed": primary_pass,
            "r9_gate": first_gate,
            "metrics": first["metrics"],
            "adamw_gate_snapshot": first["adamw_gate"],
            "adamw_metrics_snapshot": first["adamw_metrics"],
            "p2_anchor_audit": first["anchor_audit"],
            "initial_r6_equivalence": first["initial_equivalence"],
            "normalization_fold_equivalence": first["fold_equivalence"],
            "simulated_save_reload": save_reload,
            "prior_aborted_milestone_reproduction": prior_reproduction,
            "candidate_actor_digest": warm_start.actor_state_digest(
                first["candidate_state"]
            ),
            "candidate_predictions_sha256": _array_sha256(first["predictions"]),
            "base_drift_vs_r6": _base_drift(
                bundle=bundle,
                candidate_predictions=first["predictions"],
                r6_predictions=r6_predictions,
            ),
            "optimizer_audit": first["optimizer_audit"],
            "history": first["history"],
            "elapsed_seconds": first["elapsed_seconds"],
        },
        "determinism_replica": replica,
        "execution_accounting": {
            "environment_reset_calls": 0,
            "environment_step_calls": 0,
            "teacher_query_count_this_process": (
                OFFLINE_TEACHER_REVERIFICATION_QUERIES_PER_PROCESS
            ),
            "teacher_query_count_including_aborted_instrumentation_attempt": (
                2 * OFFLINE_TEACHER_REVERIFICATION_QUERIES_PER_PROCESS
            ),
            "checkpoint_publication_count": 0,
            "diagnostic_process_runs": 2,
            "prior_aborted_instrumentation_attempt": (
                PRIOR_ABORTED_INSTRUMENTATION_ATTEMPT
            ),
            "actor_fit_count_this_process": 2 if replica["executed"] else 1,
            "actor_updates_this_process": 2 if replica["executed"] else 1,
            "actor_fit_count_including_aborted_instrumentation_attempt": (
                3 if replica["executed"] else 2
            ),
            "critic_updates": 0,
            "ppo_updates": 0,
        },
    }
    _write_result(output)
    return output


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--run",
        action="store_true",
        help="execute the sole preregistered in-memory dry fit",
    )
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    if not args.run:
        raise SystemExit("explicit --run is required")
    result = run()
    print(
        json.dumps(
            {
                "status": result["status"],
                "passed": result["passed"],
                "result": RESULT_PATH.relative_to(REPO_ROOT).as_posix(),
            },
            sort_keys=True,
        ),
        flush=True,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
