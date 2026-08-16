"""Preregistered W1024 residual-capacity dry fit on the frozen V12R9 corpus.

This diagnostic changes topology, not the frozen H0 labels or numerical gates.
It builds an ordinary ``35 -> 1024 -> 1024 -> 2`` tanh actor as two isolated
512-wide towers.  Tower A is the immutable R6 actor byte-for-byte.  Tower B is
initialized with the terminal R9 hidden representation and a positive-zero
mean head, so the initial combined function is algebraically exactly R6.  Only
tower B is optimized; the R6 tower and all cross-block weights remain fixed.

The single objective is the V12R9 equal-stratum uniform weighted MSE after the
26 reset rows are multiplied by three and each of the thirteen strata is
renormalized to mass 500.  The fixed optimizer is AdamW(2500 epochs; learning
rates 3e-4, 1e-4, 3e-5) followed by LBFGS(max_iter=3000, max_eval=4500).  Only
the terminal optimizer state is evaluated.  There is no sweep, retry, early
candidate selection, repair, teacher query, environment access, checkpoint
publication, or production promotion.  A second byte-determinism replica is
run only when the first fit passes every unchanged numerical/semantic H0 gate.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import sys
import tempfile
import time
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np


DIAGNOSTIC_ROOT = Path(__file__).resolve().parent
for _root in (DIAGNOSTIC_ROOT,):
    if str(_root) not in sys.path:
        sys.path.insert(0, str(_root))

import analyze_h0_v12r9_frozen_fit as forensic  # noqa: E402
import dry_fit_extended_uniform_reset3 as extended  # noqa: E402


r9 = forensic.r9
REPO_ROOT = forensic.REPO_ROOT
R9_ROOT = forensic.R9_ROOT
BASELINE_ROOT = REPO_ROOT / "Trajectory Generator" / "baseline_MLP"
if str(BASELINE_ROOT) not in sys.path:
    sys.path.insert(0, str(BASELINE_ROOT))

import warm_start  # noqa: E402
from asymmetric_rl_module import (  # noqa: E402
    AsymmetricActorCriticTorchRLModule,
)


STATUS = "COMPLETE_H0_V12R10_W1024_R6_RESIDUAL_RESET3_DRY_FIT"
STRATEGY_ID = "V12R10_STANDARD_W1024_FROZEN_R6_PLUS_TRAINABLE_R9_HIDDEN_V1"
SEED = 20260814
TORCH_THREADS = 5
TOWER_WIDTH = 512
TARGET_WIDTH = 1024
INPUT_WIDTH = 35
ACTION_DIM = 2

ADAMW_EPOCHS = 2_500
ADAMW_BOUNDARIES = (1_000, 2_000, 2_500)
ADAMW_RATES = (3.0e-4, 1.0e-4, 3.0e-5)
ADAMW_WEIGHT_DECAY = 1.0e-7
GRADIENT_CLIP_NORM = 10.0
LBFGS_LR = 0.7
LBFGS_MAX_ITER = 3_000
LBFGS_MAX_EVAL = 4_500
LBFGS_HISTORY_SIZE = 50
LBFGS_TOLERANCE_GRAD = 1.0e-10
LBFGS_TOLERANCE_CHANGE = 1.0e-12

INITIAL_NUMERICAL_TOLERANCE = 2.0e-6
FOLD_NUMERICAL_TOLERANCE = 2.0e-6
OFFLINE_H0_QUERY_COUNT = 0

STATE_KEYS = frozenset(
    {
        "pi_encoder.0.weight",
        "pi_encoder.0.bias",
        "pi_encoder.2.weight",
        "pi_encoder.2.bias",
        "pi.0.0.weight",
        "pi.0.0.bias",
        "pi.0.2.weight",
        "pi.0.2.bias",
        "pi.1.weight",
        "pi.1.bias",
    }
)
ALIASES = (
    ("pi_encoder.0.weight", "pi.0.0.weight"),
    ("pi_encoder.0.bias", "pi.0.0.bias"),
    ("pi_encoder.2.weight", "pi.0.2.weight"),
    ("pi_encoder.2.bias", "pi.0.2.bias"),
)


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


def _bytes_equal(left: Any, right: Any) -> bool:
    a = _array(left)
    b = _array(right)
    return (
        a.dtype == b.dtype
        and a.shape == b.shape
        and a.tobytes(order="C") == b.tobytes(order="C")
    )


def _state_byte_exact(left: Mapping[str, Any], right: Mapping[str, Any]) -> bool:
    return set(left) == set(right) and all(
        _bytes_equal(left[name], right[name]) for name in left
    )


def _positive_zero(value: Any) -> bool:
    array = _array(value)
    if array.dtype == np.dtype(np.float32):
        return bool(np.all(array.view(np.uint32) == 0))
    if array.dtype == np.dtype(np.float64):
        return bool(np.all(array.view(np.uint64) == 0))
    return False


def _state_digest(state: Mapping[str, Any]) -> str:
    digest = hashlib.sha256()
    for name in sorted(state):
        value = _array(state[name])
        digest.update(name.encode("utf-8"))
        digest.update(b"\0")
        digest.update(value.dtype.str.encode("ascii"))
        digest.update(b"\0")
        digest.update(repr(value.shape).encode("ascii"))
        digest.update(b"\0")
        digest.update(value.tobytes(order="C"))
    return digest.hexdigest()


def _prediction_digest(value: Any) -> str:
    array = _array(value)
    digest = hashlib.sha256()
    digest.update(array.dtype.str.encode("ascii"))
    digest.update(repr(array.shape).encode("ascii"))
    digest.update(array.tobytes(order="C"))
    return digest.hexdigest()


def _adamw_rate(epoch: int) -> float:
    if type(epoch) is not int or not 1 <= epoch <= ADAMW_EPOCHS:
        raise ValueError(f"epoch outside preregistered schedule: {epoch!r}")
    if epoch <= ADAMW_BOUNDARIES[0]:
        return ADAMW_RATES[0]
    if epoch <= ADAMW_BOUNDARIES[1]:
        return ADAMW_RATES[1]
    return ADAMW_RATES[2]


def _expected_shapes(width: int) -> dict[str, tuple[int, ...]]:
    return {
        "pi_encoder.0.weight": (width, INPUT_WIDTH),
        "pi_encoder.0.bias": (width,),
        "pi_encoder.2.weight": (width, width),
        "pi_encoder.2.bias": (width,),
        "pi.0.0.weight": (width, INPUT_WIDTH),
        "pi.0.0.bias": (width,),
        "pi.0.2.weight": (width, width),
        "pi.0.2.bias": (width,),
        "pi.1.weight": (2 * ACTION_DIM, width),
        "pi.1.bias": (2 * ACTION_DIM,),
    }


def _validate_w1024_state(state: Mapping[str, Any]) -> dict[str, Any]:
    arrays = _state_arrays(state)
    expected = _expected_shapes(TARGET_WIDTH)
    checks = {
        "key_set_exact": set(arrays) == STATE_KEYS,
        "shapes_exact": set(arrays) == set(expected)
        and all(arrays[name].shape == shape for name, shape in expected.items()),
        "float32_finite": all(
            value.dtype == np.float32 and np.all(np.isfinite(value))
            for value in arrays.values()
        ),
        "encoder_aliases_byte_exact": all(
            _bytes_equal(arrays[left], arrays[right]) for left, right in ALIASES
        ),
        "clock_columns_positive_zero": _positive_zero(
            arrays["pi_encoder.0.weight"][:, r9.contract.DISABLED_CLOCK_COLUMNS]
        ),
        "logstd_weight_positive_zero": _positive_zero(
            arrays["pi.1.weight"][ACTION_DIM:]
        ),
    }
    if not all(checks.values()):
        raise RuntimeError(
            "W1024 state invariant failed: "
            + repr(sorted(name for name, passed in checks.items() if not passed))
        )
    return {
        "passed": True,
        "checks": checks,
        "hidden_dims": [TARGET_WIDTH, TARGET_WIDTH],
        "actor_feature_count": INPUT_WIDTH,
        "actor_digest": warm_start.actor_state_digest(arrays),
        "state_digest": _state_digest(arrays),
    }


def _load_locked_inputs() -> tuple[
    dict[str, np.ndarray],
    np.ndarray,
    dict[str, Any],
    Any,
    dict[str, np.ndarray],
    dict[str, np.ndarray],
    dict[str, Any],
]:
    arrays = extended._load_arrays()  # noqa: SLF001
    weights, weight_audit = extended._reset3_weights(arrays)  # noqa: SLF001
    normalization = extended._normalization(arrays)  # noqa: SLF001
    r6_module, r6_state_raw, r6_manifest = r9._load_source_module_and_state()  # noqa: SLF001
    r6_state = _state_arrays(r6_state_raw)
    r9_state_raw, _r9_predictions = forensic._candidate_state_and_predictions(
        arrays["observations"]
    )
    r9_state = _state_arrays(r9_state_raw)
    r9.validate_source_r6_state(r6_state)
    r9.validate_source_r6_state(r9_state)
    records = {
        "r6_actor_digest": warm_start.actor_state_digest(r6_state),
        "r9_terminal_actor_digest": warm_start.actor_state_digest(r9_state),
        "r6_manifest_actor_digest": r6_manifest["actor_digest"],
        "r9_ledger_sha256": _sha256_file(forensic.LEDGER_PATH),
        "r9_gate_sha256": _sha256_file(forensic.GATE_PATH),
        "r9_summary_sha256": _sha256_file(forensic.SUMMARY_PATH),
        "r9_corpus_sha256": _sha256_file(forensic.CORPUS_PATH),
        "r9_candidate_tree_sha256": forensic._strict_json(forensic.SUMMARY_PATH)[
            "candidate_module"
        ]["tree_sha256"],
    }
    return (
        arrays,
        weights,
        weight_audit,
        normalization,
        r6_state,
        r9_state,
        {"module": r6_module, "records": records},
    )


def _new_normalized_residual_model(
    r9_state: Mapping[str, Any], normalization: Any
) -> Any:
    import torch
    from torch import nn

    model = nn.Sequential(
        nn.Linear(INPUT_WIDTH, TOWER_WIDTH),
        nn.Tanh(),
        nn.Linear(TOWER_WIDTH, TOWER_WIDTH),
        nn.Tanh(),
        nn.Linear(TOWER_WIDTH, ACTION_DIM),
    )
    mean = torch.as_tensor(normalization.mean, dtype=torch.float32)
    std = torch.as_tensor(normalization.std, dtype=torch.float32)
    raw_w0 = torch.as_tensor(r9_state["pi_encoder.0.weight"], dtype=torch.float32)
    raw_b0 = torch.as_tensor(r9_state["pi_encoder.0.bias"], dtype=torch.float32)
    with torch.no_grad():
        model[0].weight.copy_(raw_w0 * std[None, :])
        model[0].bias.copy_(raw_b0 + raw_w0 @ mean)
        model[2].weight.copy_(
            torch.as_tensor(r9_state["pi_encoder.2.weight"], dtype=torch.float32)
        )
        model[2].bias.copy_(
            torch.as_tensor(r9_state["pi_encoder.2.bias"], dtype=torch.float32)
        )
        model[4].weight.zero_()
        model[4].bias.zero_()
    if not _positive_zero(
        model[0].weight.detach().cpu().numpy()[:, r9.contract.DISABLED_CLOCK_COLUMNS]
    ) or not _positive_zero(model[4].weight.detach().cpu().numpy()):
        raise RuntimeError("residual positive-zero initialization drifted")
    return model


def _pack_w1024_state(
    *,
    residual: Any,
    r6_state: Mapping[str, Any],
    normalization: Any,
) -> dict[str, np.ndarray]:
    import torch

    normalized_w0 = np.ascontiguousarray(
        residual[0].weight.detach().cpu().numpy(), dtype=np.float32
    )
    normalized_b0 = np.ascontiguousarray(
        residual[0].bias.detach().cpu().numpy(), dtype=np.float32
    )
    if not _positive_zero(normalized_w0[:, r9.contract.DISABLED_CLOCK_COLUMNS]):
        raise RuntimeError("trained residual clock columns drifted")
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
        raise RuntimeError("folded residual clock columns drifted")

    first_weight = np.zeros((TARGET_WIDTH, INPUT_WIDTH), dtype=np.float32)
    first_bias = np.zeros(TARGET_WIDTH, dtype=np.float32)
    first_weight[:TOWER_WIDTH] = r6_state["pi_encoder.0.weight"]
    first_weight[TOWER_WIDTH:] = raw_w0
    first_bias[:TOWER_WIDTH] = r6_state["pi_encoder.0.bias"]
    first_bias[TOWER_WIDTH:] = raw_b0

    second_weight = np.zeros((TARGET_WIDTH, TARGET_WIDTH), dtype=np.float32)
    second_bias = np.zeros(TARGET_WIDTH, dtype=np.float32)
    second_weight[:TOWER_WIDTH, :TOWER_WIDTH] = r6_state["pi_encoder.2.weight"]
    second_weight[TOWER_WIDTH:, TOWER_WIDTH:] = np.ascontiguousarray(
        residual[2].weight.detach().cpu().numpy(), dtype=np.float32
    )
    second_bias[:TOWER_WIDTH] = r6_state["pi_encoder.2.bias"]
    second_bias[TOWER_WIDTH:] = np.ascontiguousarray(
        residual[2].bias.detach().cpu().numpy(), dtype=np.float32
    )

    head_weight = np.zeros((2 * ACTION_DIM, TARGET_WIDTH), dtype=np.float32)
    head_bias = np.ascontiguousarray(r6_state["pi.1.bias"], dtype=np.float32).copy()
    head_weight[:ACTION_DIM, :TOWER_WIDTH] = r6_state["pi.1.weight"][:ACTION_DIM]
    head_weight[:ACTION_DIM, TOWER_WIDTH:] = np.ascontiguousarray(
        residual[4].weight.detach().cpu().numpy(), dtype=np.float32
    )
    head_bias[:ACTION_DIM] += np.ascontiguousarray(
        residual[4].bias.detach().cpu().numpy(), dtype=np.float32
    )

    state = {
        "pi_encoder.0.weight": first_weight,
        "pi_encoder.0.bias": first_bias,
        "pi_encoder.2.weight": second_weight,
        "pi_encoder.2.bias": second_bias,
        "pi.0.0.weight": first_weight.copy(),
        "pi.0.0.bias": first_bias.copy(),
        "pi.0.2.weight": second_weight.copy(),
        "pi.0.2.bias": second_bias.copy(),
        "pi.1.weight": head_weight,
        "pi.1.bias": head_bias,
    }
    _validate_w1024_state(state)
    return state


def _tower_isolation_audit(
    candidate: Mapping[str, Any], r6_state: Mapping[str, Any]
) -> dict[str, Any]:
    checks = {
        "r6_first_weight_byte_exact": _bytes_equal(
            candidate["pi_encoder.0.weight"][:TOWER_WIDTH],
            r6_state["pi_encoder.0.weight"],
        ),
        "r6_first_bias_byte_exact": _bytes_equal(
            candidate["pi_encoder.0.bias"][:TOWER_WIDTH],
            r6_state["pi_encoder.0.bias"],
        ),
        "r6_second_weight_byte_exact": _bytes_equal(
            candidate["pi_encoder.2.weight"][:TOWER_WIDTH, :TOWER_WIDTH],
            r6_state["pi_encoder.2.weight"],
        ),
        "r6_second_bias_byte_exact": _bytes_equal(
            candidate["pi_encoder.2.bias"][:TOWER_WIDTH],
            r6_state["pi_encoder.2.bias"],
        ),
        "r6_mean_head_weight_byte_exact": _bytes_equal(
            candidate["pi.1.weight"][:ACTION_DIM, :TOWER_WIDTH],
            r6_state["pi.1.weight"][:ACTION_DIM],
        ),
        "cross_blocks_positive_zero": _positive_zero(
            candidate["pi_encoder.2.weight"][:TOWER_WIDTH, TOWER_WIDTH:]
        )
        and _positive_zero(
            candidate["pi_encoder.2.weight"][TOWER_WIDTH:, :TOWER_WIDTH]
        ),
        "logstd_bias_byte_exact": _bytes_equal(
            candidate["pi.1.bias"][ACTION_DIM:],
            r6_state["pi.1.bias"][ACTION_DIM:],
        ),
        "logstd_full_weight_positive_zero": _positive_zero(
            candidate["pi.1.weight"][ACTION_DIM:]
        ),
    }
    return {"passed": all(checks.values()), "checks": checks}


def _module(
    *, source_module: Any, state: Mapping[str, Any], inference_only: bool
) -> Any:
    config = dict(source_module.model_config)
    config.update(
        {
            "fcnet_hiddens": [TARGET_WIDTH, TARGET_WIDTH],
            "fcnet_activation": "tanh",
            "n_actor": INPUT_WIDTH,
            "n_full": r9.contract.EXPECTED_FULL_FEATURES,
            "freeze_actor": bool(inference_only),
            "freeze_logstd": bool(inference_only),
        }
    )
    module = AsymmetricActorCriticTorchRLModule(
        observation_space=source_module.observation_space,
        action_space=source_module.action_space,
        inference_only=inference_only,
        learner_only=False,
        model_config=config,
        catalog_class=None,
    )
    module.set_state(state)
    module.eval()
    return module


def _runtime_logits(module: Any, observations: np.ndarray) -> np.ndarray:
    import torch
    from ray.rllib.core.columns import Columns

    values = torch.as_tensor(observations, dtype=torch.float32)
    with torch.no_grad():
        output = module.forward_inference({Columns.OBS: values})
    return np.ascontiguousarray(
        output[Columns.ACTION_DIST_INPUTS].detach().cpu().numpy(), dtype=np.float32
    )


def _runtime_and_save_reload_audit(
    *,
    source_module: Any,
    candidate_state: Mapping[str, Any],
    observations: np.ndarray,
    feature_names: Sequence[str],
) -> dict[str, Any]:
    from ray.rllib.core.rl_module.rl_module import RLModule

    expected = np.ascontiguousarray(
        r9.v11._state_logits(candidate_state, observations), dtype=np.float32
    )
    module = _module(
        source_module=source_module,
        state=candidate_state,
        inference_only=True,
    )
    direct = _runtime_logits(module, observations)
    actor_prefix_exact = _bytes_equal(direct, expected)
    padded = np.zeros(
        (len(observations), r9.contract.EXPECTED_FULL_FEATURES), dtype=np.float32
    )
    padded[:, :INPUT_WIDTH] = observations
    full_input_exact = _bytes_equal(_runtime_logits(module, padded), expected)

    with tempfile.TemporaryDirectory(prefix="h0_v12r10_w1024_") as temporary:
        checkpoint = Path(temporary) / "rl_module"
        module.save_to_path(checkpoint)
        manifest = checkpoint / warm_start.DEFAULT_ACTOR_FEATURE_MANIFEST_NAME
        manifest.write_text(
            json.dumps(
                {
                    "schema_version": 1,
                    "status": "TEMPORARY_V12R10_W1024_COMPATIBILITY_MANIFEST",
                    "actor_feature_count": len(feature_names),
                    "actor_feature_names": list(feature_names),
                    "actor_digest": warm_start.actor_state_digest(candidate_state),
                    "fcnet_hiddens": [TARGET_WIDTH, TARGET_WIDTH],
                    "disabled_clock_columns": list(r9.contract.DISABLED_CLOCK_COLUMNS),
                },
                indent=2,
                sort_keys=True,
            )
            + "\n",
            encoding="utf-8",
        )
        reloaded = RLModule.from_checkpoint(checkpoint)
        reloaded.eval()
        reloaded_state = _state_arrays(reloaded.get_state())
        save_reload_state_exact = _state_byte_exact(candidate_state, reloaded_state)
        save_reload_logits_exact = _bytes_equal(
            _runtime_logits(reloaded, observations), expected
        )

        fresh = _module(
            source_module=source_module,
            state=candidate_state,
            inference_only=False,
        )
        fresh_before = _state_arrays(fresh.get_state())
        transplanted, transplant_report = warm_start.transplant_actor_state(
            target_state=fresh_before,
            target_actor_feature_names=feature_names,
            source_checkpoint=checkpoint,
            source_actor_feature_manifest=manifest,
            mode="drop",
            zero_target_features=warm_start.DISABLED_GAIT_CLOCK_FEATURES,
        )
        actor_compare = warm_start.compare_actor_states(candidate_state, transplanted)
        critic_compare = warm_start.compare_non_actor_states(fresh_before, transplanted)
        fresh.set_state(transplanted)
        transplant_logits_exact = _bytes_equal(_runtime_logits(fresh, padded), expected)

    checks = {
        "standard_module_class": type(module) is AsymmetricActorCriticTorchRLModule,
        "inference_only_actor_state": set(module.get_state()) == STATE_KEYS,
        "model_config_w1024_tanh": list(module.model_config.get("fcnet_hiddens", ()))
        == [TARGET_WIDTH, TARGET_WIDTH]
        and str(module.model_config.get("fcnet_activation", "")).lower() == "tanh",
        "actor_prefix_runtime_exact": actor_prefix_exact,
        "full_input_runtime_exact": full_input_exact,
        "checkpoint_state_byte_exact": save_reload_state_exact,
        "checkpoint_logits_byte_exact": save_reload_logits_exact,
        "warm_start_actor_exact": actor_compare["exact"],
        "warm_start_critic_preserved": critic_compare["exact"],
        "warm_start_forward_exact": transplant_logits_exact,
    }
    return {
        "passed": all(checks.values()),
        "checks": checks,
        "mode": "TEMPORARY_REAL_RLMODULE_CHECKPOINT_RELOAD_AND_FULL_MODULE_TRANSPLANT",
        "temporary_checkpoint_removed": True,
        "source_actor_only": transplant_report["source_state_is_actor_only"],
        "target_non_actor_keys_preserved": transplant_report[
            "target_non_actor_keys_preserved"
        ],
        "runtime_logits_sha256": _prediction_digest(expected),
    }


def _w1024_gate(
    *, metrics: Mapping[str, Any], compatibility: Mapping[str, Any]
) -> tuple[dict[str, Any], dict[str, Any]]:
    summary = {
        "actor_fit_count": 1,
        "actor_updates": 1,
        "critic_updates": 0,
        "ppo_updates": 0,
        "hidden_dims": [TARGET_WIDTH, TARGET_WIDTH],
        "actor_feature_count": INPUT_WIDTH,
        "logstd_byte_exact": True,
        "disabled_clock_columns_bit_zero": True,
        "save_reload_exact": compatibility["passed"],
        **metrics,
    }
    literal_r9_gate = r9.contract.fit_gate(summary)
    checks = dict(literal_r9_gate["checks"])
    if checks.get("architecture") is not False:
        raise RuntimeError("literal frozen R9 architecture gate unexpectedly passed")
    checks["architecture"] = True
    checks["architecture_w1024_standard"] = True
    passed = all(checks.values())
    return literal_r9_gate, {
        "status": (
            "PASS_H0_V12R10_STANDARD_W1024_UNCHANGED_NUMERICAL_GATES"
            if passed
            else "FAIL_H0_V12R10_STANDARD_W1024_UNCHANGED_NUMERICAL_GATES"
        ),
        "passed": passed,
        "checks": checks,
        "only_gate_schema_change": (
            "architecture [512,512] replaced by explicitly tested standard "
            "[1024,1024]; every numerical threshold and other semantic check "
            "is the frozen R9 check"
        ),
        "offline_thresholds_unchanged": r9.contract.OFFLINE_THRESHOLDS,
    }


def _snapshot(
    *, stage: str, index: int, loss: Any, combined_prediction: Any
) -> dict[str, Any]:
    values = combined_prediction.detach().cpu().numpy().astype(np.float64)
    return {
        "stage": stage,
        "index": int(index),
        "loss": float(loss.detach().cpu()),
        "prediction_abs_max": float(np.max(np.abs(values))),
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

    raw = arrays["observations"]
    normalized = r9.v11.normalized_observations(raw, normalization)
    fixed_predictions_np = np.ascontiguousarray(
        r9.v11._state_logits(r6_state, raw)[:, :ACTION_DIM], dtype=np.float32
    )
    previous_threads = torch.get_num_threads()
    previous_deterministic = torch.are_deterministic_algorithms_enabled()
    torch.set_num_threads(TORCH_THREADS)
    torch.use_deterministic_algorithms(True)
    started = time.monotonic()
    try:
        torch.manual_seed(SEED)
        residual = _new_normalized_residual_model(r9_state, normalization)
        x = torch.as_tensor(normalized, dtype=torch.float32)
        targets = torch.as_tensor(arrays["actions"], dtype=torch.float32)
        fixed = torch.as_tensor(fixed_predictions_np, dtype=torch.float32)
        weights = torch.as_tensor(sample_weights, dtype=torch.float64)
        weight_sum = torch.sum(weights)

        with torch.no_grad():
            initial_residual = residual(x)
            initial_combined = fixed + initial_residual
        initial_error = np.abs(
            initial_combined.cpu().numpy().astype(np.float64)
            - fixed_predictions_np.astype(np.float64)
        )
        initial_max = float(np.max(initial_error))
        initial_exact = _bytes_equal(
            initial_combined.cpu().numpy(), fixed_predictions_np
        )
        if not initial_exact or initial_max != 0.0:
            raise RuntimeError("zero-head residual did not initialize exactly at R6")

        def objective() -> tuple[Any, Any]:
            combined = fixed + residual(x)
            row_mse = torch.mean(torch.square(combined - targets), dim=1).to(
                torch.float64
            )
            loss = torch.sum(weights * row_mse) / weight_sum
            return loss, combined

        history: list[dict[str, Any]] = []
        with torch.no_grad():
            initial_loss, initial_prediction = objective()
        row = _snapshot(
            stage="initial",
            index=0,
            loss=initial_loss,
            combined_prediction=initial_prediction,
        )
        history.append(row)
        print(json.dumps({"fit": label, **row}, sort_keys=True), flush=True)

        adamw = torch.optim.AdamW(
            residual.parameters(),
            lr=ADAMW_RATES[0],
            weight_decay=ADAMW_WEIGHT_DECAY,
        )
        milestones = {1, 100, 250, 500, 1_000, 1_500, 2_000, 2_500}
        for epoch in range(1, ADAMW_EPOCHS + 1):
            rate = _adamw_rate(epoch)
            for group in adamw.param_groups:
                group["lr"] = rate
            adamw.zero_grad(set_to_none=True)
            loss, prediction = objective()
            if not torch.isfinite(loss):
                raise RuntimeError(f"non-finite AdamW loss at epoch {epoch}")
            loss.backward()
            torch.nn.utils.clip_grad_norm_(residual.parameters(), GRADIENT_CLIP_NORM)
            adamw.step()
            with torch.no_grad():
                residual[0].weight[:, r9.contract.DISABLED_CLOCK_COLUMNS] = 0.0
            if epoch in milestones:
                with torch.no_grad():
                    snap_loss, snap_prediction = objective()
                row = {
                    **_snapshot(
                        stage="adamw",
                        index=epoch,
                        loss=snap_loss,
                        combined_prediction=snap_prediction,
                    ),
                    "learning_rate": rate,
                }
                history.append(row)
                print(json.dumps({"fit": label, **row}, sort_keys=True), flush=True)

        lbfgs = torch.optim.LBFGS(
            residual.parameters(),
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
            value, _prediction = objective()
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
            }:
                row = {
                    "stage": "lbfgs_closure",
                    "index": closure_calls,
                    "loss": float(value.detach().cpu()),
                }
                history.append(row)
                print(json.dumps({"fit": label, **row}, sort_keys=True), flush=True)
            return value

        lbfgs.step(closure)
        with torch.no_grad():
            residual[0].weight[:, r9.contract.DISABLED_CLOCK_COLUMNS] = 0.0
            terminal_loss, normalized_predictions_tensor = objective()

        candidate_state = _pack_w1024_state(
            residual=residual,
            r6_state=r6_state,
            normalization=normalization,
        )
        predictions = np.ascontiguousarray(
            r9.v11._state_logits(candidate_state, raw)[:, :ACTION_DIM],
            dtype=np.float32,
        )
        normalized_predictions = np.ascontiguousarray(
            normalized_predictions_tensor.cpu().numpy(), dtype=np.float32
        )
        fold_max = float(
            np.max(
                np.abs(
                    predictions.astype(np.float64)
                    - normalized_predictions.astype(np.float64)
                )
            )
        )
        fold_passed = bool(fold_max <= FOLD_NUMERICAL_TOLERANCE)
        if not fold_passed:
            raise RuntimeError(f"W1024 normalization fold drifted by {fold_max}")

        metrics = forensic._metric_payload(predictions, arrays)
        state_audit = _validate_w1024_state(candidate_state)
        isolation = _tower_isolation_audit(candidate_state, r6_state)
        compatibility = _runtime_and_save_reload_audit(
            source_module=source_module,
            candidate_state=candidate_state,
            observations=raw,
            feature_names=arrays["actor_feature_names"].astype(str).tolist(),
        )
        literal_gate, w1024_gate = _w1024_gate(
            metrics=metrics, compatibility=compatibility
        )
        terminal_row = {
            "stage": "terminal_final_state",
            "index": closure_calls,
            "loss": float(terminal_loss.detach().cpu()),
            "global_rmse": metrics["global_metrics"]["rmse"],
            "global_max_abs_error": metrics["global_metrics"]["max_abs_error"],
            "reset_max_abs_error": metrics["reset_max_abs_error"],
            "w1024_gate_passed": w1024_gate["passed"],
        }
        history.append(terminal_row)
        print(json.dumps({"fit": label, **terminal_row}, sort_keys=True), flush=True)
        return {
            "passed": bool(
                w1024_gate["passed"]
                and state_audit["passed"]
                and isolation["passed"]
                and compatibility["passed"]
            ),
            "candidate_state": candidate_state,
            "predictions": predictions,
            "metrics": metrics,
            "literal_frozen_r9_gate": literal_gate,
            "w1024_gate": w1024_gate,
            "state_audit": state_audit,
            "tower_isolation": isolation,
            "runtime_save_reload_warm_start": compatibility,
            "initial_equivalence": {
                "algebra": "R6 + ZERO_HEAD(R9_HIDDEN) = R6",
                "prediction_byte_exact": initial_exact,
                "max_abs_difference": initial_max,
                "tolerance": INITIAL_NUMERICAL_TOLERANCE,
                "passed": initial_max <= INITIAL_NUMERICAL_TOLERANCE,
            },
            "normalization_fold_equivalence": {
                "passed": fold_passed,
                "max_abs_difference": fold_max,
                "tolerance": FOLD_NUMERICAL_TOLERANCE,
            },
            "optimizer": {
                "seed": SEED,
                "torch_threads": TORCH_THREADS,
                "deterministic_algorithms": True,
                "full_batch": True,
                "terminal_state_only": True,
                "adamw_epochs": ADAMW_EPOCHS,
                "adamw_boundaries": list(ADAMW_BOUNDARIES),
                "adamw_rates": list(ADAMW_RATES),
                "adamw_weight_decay": ADAMW_WEIGHT_DECAY,
                "gradient_clip_norm": GRADIENT_CLIP_NORM,
                "lbfgs_lr": LBFGS_LR,
                "lbfgs_max_iter": LBFGS_MAX_ITER,
                "lbfgs_max_eval": LBFGS_MAX_EVAL,
                "lbfgs_history_size": LBFGS_HISTORY_SIZE,
                "lbfgs_closure_calls": closure_calls,
                "sweep": False,
                "retry": False,
                "repair": False,
                "early_candidate_selection": False,
            },
            "history": history,
            "terminal_stratum_errors": forensic._stratum_error_audit(
                predictions, arrays, sample_weights
            ),
            "terminal_reset": forensic._reset_audit(
                predictions, arrays, sample_weights
            ),
            "terminal_top_error_rows": forensic._top_error_rows(
                predictions, arrays, sample_weights, limit=20
            ),
            "candidate_state_digest": _state_digest(candidate_state),
            "candidate_predictions_sha256": _prediction_digest(predictions),
            "elapsed_seconds": float(time.monotonic() - started),
        }
    finally:
        torch.use_deterministic_algorithms(previous_deterministic)
        torch.set_num_threads(previous_threads)


def run() -> dict[str, Any]:
    (
        arrays,
        weights,
        weight_audit,
        normalization,
        r6_state,
        r9_state,
        source,
    ) = _load_locked_inputs()
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

    replica: dict[str, Any]
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
        state_exact = _state_byte_exact(primary_state, repeated_state)
        predictions_exact = _bytes_equal(primary_predictions, repeated_predictions)
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
            "reason": "preregistered rule: stop after first failed fit",
        }

    passed = bool(primary["passed"] and replica.get("passed") is True)
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
                INPUT_WIDTH,
                TARGET_WIDTH,
                TARGET_WIDTH,
                ACTION_DIM,
            ],
            "tower_a": "IMMUTABLE_R6_W512_FUNCTION",
            "tower_b": "TRAINABLE_W512_RESIDUAL_R9_HIDDEN_ZERO_HEAD_INITIALIZATION",
            "cross_blocks": "PERMANENT_POSITIVE_ZERO",
            "initial_output": "EXACT_R6",
            "objective": "R9_EQUAL_STRATUM_UNIFORM_MSE_RESET_ROWS_X3",
            "weight_audit": weight_audit,
            "unchanged_offline_thresholds": r9.contract.OFFLINE_THRESHOLDS,
            "single_preregistered_schedule": True,
            "sweep": False,
            "retry": False,
            "selection": False,
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
            "offline_h0_teacher_queries_per_fit": OFFLINE_H0_QUERY_COUNT,
            "offline_h0_teacher_queries_total": OFFLINE_H0_QUERY_COUNT * total_fits,
            "environment_reset_calls": 0,
            "environment_step_calls": 0,
            "ppo_updates": 0,
            "critic_updates": 0,
            "persistent_model_state_writes": 0,
            "temporary_checkpoint_roundtrips_per_fit": 1,
            "production_candidate_created": False,
            "production_candidate_promoted": False,
        },
        "limitations": {
            "literal_r9_architecture_gate": (
                "necessarily false because its frozen schema requires [512,512]"
            ),
            "interpretation": (
                "acceptance requires the replacement standard-W1024 architecture "
                "check plus every unchanged R9 numerical and semantic gate"
            ),
            "rollout_evidence": False,
            "production_attestation": False,
            "known_transition_alias_risk": {
                "observer_case": "deterministic_offset_nominal",
                "step_window": [385, 388],
                "forensic_interpretation": (
                    "legacy teacher timeout transition is not signalled by V26, "
                    "which remains in STANCE while target action 0 jumps by about 0.18"
                ),
                "design_changed_after_observation": False,
                "implication": (
                    "extra capacity may fit the frozen labels but cannot by itself "
                    "prove that the deployed observation is Markov at this boundary"
                ),
            },
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
