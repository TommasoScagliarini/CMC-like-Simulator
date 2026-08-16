"""Build the exact deployable 70% P2 + 30% R5 composite H0 actor.

The prediction-space blend used during R6 design is made deployable without
distillation error by widening the ordinary asymmetric actor from 256 to 512
units per hidden layer.  The first hidden layers are concatenated, the second
hidden layer is block diagonal, and the mean head combines the two independent
towers with fixed weights.  The result remains an ordinary
``AsymmetricActorCriticTorchRLModule`` checkpoint; there is no custom runtime
module or ensemble dependency.

This module is inert on import.  It has no canonical output path and the CLI
requires ``--output`` explicitly.  Publication claims the destination directory
exclusively and never replaces an existing path.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import stat
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np


def _repo_root() -> Path:
    for candidate in Path(__file__).resolve().parents:
        if (candidate / "AGENTS.md").is_file():
            return candidate
    raise RuntimeError("repository root not found")


REPO_ROOT = _repo_root()
BASELINE_ROOT = REPO_ROOT / "Trajectory Generator" / "baseline_MLP"
for import_root in (BASELINE_ROOT, REPO_ROOT / "validation", REPO_ROOT):
    text_root = os.fspath(import_root)
    if text_root not in sys.path:
        sys.path.insert(0, text_root)

import warm_start  # noqa: E402
from asymmetric_rl_module import (  # noqa: E402
    AsymmetricActorCriticTorchRLModule,
)
from ray.rllib.core.columns import Columns  # noqa: E402
from ray.rllib.core.rl_module.rl_module import RLModule  # noqa: E402
from ray.rllib.utils.framework import try_import_torch  # noqa: E402


torch, _nn = try_import_torch()


SCHEMA_VERSION = 1
TOPOLOGY_ID = "V12R6_STANDARD_FUNCTIONAL_BLEND_512_V1"
P2_WEIGHT = 0.70
R5_WEIGHT = 0.30
SOURCE_HIDDENS = (256, 256)
TARGET_HIDDENS = (512, 512)
EXPECTED_ACTOR_FEATURES = 35
EXPECTED_FULL_FEATURES = 84
EXPECTED_ACTION_DIM = 2
DISABLED_CLOCK_COLUMNS = (0, 1)
MEAN_ABS_TOLERANCE = 2.0e-6
MEAN_REL_TOLERANCE = 0.0

DEFAULT_P2_CHECKPOINT = (
    BASELINE_ROOT
    / "validation"
    / "v12r3"
    / "h0_v12r3_run_20260809"
    / "fit"
    / "p2"
    / "rl_module_target_adapted"
)
DEFAULT_R5_CHECKPOINT = (
    BASELINE_ROOT
    / "validation"
    / "v12r5"
    / "h0_v12r5_run_20260809"
    / "fit"
    / "rl_module_target_adapted"
)
DEFAULT_CORPUS_PATH = (
    BASELINE_ROOT
    / "validation"
    / "v12r5"
    / "h0_v12r5_run_20260809"
    / "fit"
    / "corpus.npz"
)

CANONICAL_ENCODER_KEYS = (
    "pi_encoder.0.weight",
    "pi_encoder.0.bias",
    "pi_encoder.2.weight",
    "pi_encoder.2.bias",
)
ENCODER_ALIAS_PAIRS = (
    ("pi_encoder.0.weight", "pi.0.0.weight"),
    ("pi_encoder.0.bias", "pi.0.0.bias"),
    ("pi_encoder.2.weight", "pi.0.2.weight"),
    ("pi_encoder.2.bias", "pi.0.2.bias"),
)
HEAD_KEYS = ("pi.1.weight", "pi.1.bias")
EXPECTED_ACTOR_STATE_KEYS = frozenset(
    {
        *CANONICAL_ENCODER_KEYS,
        *(alias for _canonical, alias in ENCODER_ALIAS_PAIRS),
        *HEAD_KEYS,
    }
)
RL_MODULE_FILES = frozenset(
    {"class_and_ctor_args.pkl", "metadata.json", "module_state.pkl"}
)
ACTOR_FEATURE_MANIFEST = warm_start.DEFAULT_ACTOR_FEATURE_MANIFEST_NAME
BUILD_MANIFEST = "composite_build_manifest.json"


class CompositeActorBuildError(RuntimeError):
    """Raised when synthesis or any exactness invariant fails closed."""


def _absolute_no_follow(path: str | Path) -> Path:
    """Return an absolute lexical path without resolving a symlink component."""

    expanded = Path(path).expanduser()
    return Path(os.path.abspath(expanded))


def _is_link_or_reparse(path: Path) -> bool:
    try:
        status = os.lstat(path)
    except OSError:
        return False
    attributes = int(getattr(status, "st_file_attributes", 0) or 0)
    reparse_flag = int(getattr(stat, "FILE_ATTRIBUTE_REPARSE_POINT", 0x400))
    return stat.S_ISLNK(status.st_mode) or bool(attributes & reparse_flag)


def _reject_link_or_reparse_components(path: Path, *, include_leaf: bool) -> None:
    """Reject every existing symlink/junction in one absolute lexical path."""

    absolute = _absolute_no_follow(path)
    anchor = Path(absolute.anchor)
    parts = absolute.parts[1:] if absolute.anchor else absolute.parts
    limit = len(parts) if include_leaf else max(0, len(parts) - 1)
    current = anchor
    for part in parts[:limit]:
        current /= part
        if os.path.lexists(current) and _is_link_or_reparse(current):
            raise CompositeActorBuildError(
                f"unsafe symlink/junction path component: {current}"
            )


@dataclass(frozen=True)
class _CompositeBundle:
    module: AsymmetricActorCriticTorchRLModule
    p2: AsymmetricActorCriticTorchRLModule
    r5: AsymmetricActorCriticTorchRLModule
    source_audit: dict[str, Any]


def _array(value: Any) -> np.ndarray:
    if hasattr(value, "detach"):
        value = value.detach()
    if hasattr(value, "cpu"):
        value = value.cpu()
    return np.asarray(value)


def _float32(value: Any, *, label: str) -> np.ndarray:
    array = np.ascontiguousarray(_array(value))
    if array.dtype != np.float32:
        raise CompositeActorBuildError(f"{label} must have dtype float32")
    if not np.all(np.isfinite(array)):
        raise CompositeActorBuildError(f"{label} contains a non-finite value")
    return array


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _array_digest(value: Any) -> str:
    array = np.ascontiguousarray(_array(value))
    digest = hashlib.sha256()
    digest.update(str(array.dtype).encode("ascii"))
    digest.update(repr(tuple(int(item) for item in array.shape)).encode("ascii"))
    digest.update(array.tobytes(order="C"))
    return digest.hexdigest()


def _checkpoint_record(path: Path) -> dict[str, Any]:
    checkpoint = _absolute_no_follow(path)
    _reject_link_or_reparse_components(checkpoint, include_leaf=True)
    if not checkpoint.is_dir():
        raise CompositeActorBuildError(f"checkpoint directory is missing: {checkpoint}")
    files: dict[str, dict[str, Any]] = {}
    digest = hashlib.sha256()
    for name in sorted(RL_MODULE_FILES):
        source = checkpoint / name
        if not source.is_file() or source.is_symlink():
            raise CompositeActorBuildError(
                f"checkpoint file is missing or unsafe: {source}"
            )
        sha256 = _sha256_file(source)
        size = source.stat().st_size
        files[name] = {"sha256": sha256, "size_bytes": int(size)}
        digest.update(name.encode("utf-8"))
        digest.update(b"\0")
        digest.update(sha256.encode("ascii"))
        digest.update(b"\n")
    return {
        "path": os.fspath(checkpoint),
        "tree_sha256": digest.hexdigest(),
        "files": files,
    }


def _box_contract(space: Any) -> tuple[tuple[int, ...], str, bytes, bytes]:
    return (
        tuple(int(item) for item in space.shape),
        str(space.dtype),
        np.ascontiguousarray(space.low).tobytes(),
        np.ascontiguousarray(space.high).tobytes(),
    )


def _positive_zero_bits(value: Any) -> bool:
    array = np.ascontiguousarray(_array(value))
    if array.dtype == np.float32:
        return bool(np.all(array.view(np.uint32) == 0))
    if array.dtype == np.float64:
        return bool(np.all(array.view(np.uint64) == 0))
    return False


def _require_encoder_alias_values(state: Mapping[str, Any], label: str) -> None:
    for canonical, alias in ENCODER_ALIAS_PAIRS:
        if not np.array_equal(_array(state[canonical]), _array(state[alias])):
            raise CompositeActorBuildError(
                f"{label} encoder alias drifted: {canonical} != {alias}"
            )


def _expected_source_shapes() -> dict[str, tuple[int, ...]]:
    return {
        "pi_encoder.0.weight": (256, EXPECTED_ACTOR_FEATURES),
        "pi_encoder.0.bias": (256,),
        "pi_encoder.2.weight": (256, 256),
        "pi_encoder.2.bias": (256,),
        "pi.0.0.weight": (256, EXPECTED_ACTOR_FEATURES),
        "pi.0.0.bias": (256,),
        "pi.0.2.weight": (256, 256),
        "pi.0.2.bias": (256,),
        "pi.1.weight": (2 * EXPECTED_ACTION_DIM, 256),
        "pi.1.bias": (2 * EXPECTED_ACTION_DIM,),
    }


def _expected_target_shapes() -> dict[str, tuple[int, ...]]:
    return {
        "pi_encoder.0.weight": (512, EXPECTED_ACTOR_FEATURES),
        "pi_encoder.0.bias": (512,),
        "pi_encoder.2.weight": (512, 512),
        "pi_encoder.2.bias": (512,),
        "pi.0.0.weight": (512, EXPECTED_ACTOR_FEATURES),
        "pi.0.0.bias": (512,),
        "pi.0.2.weight": (512, 512),
        "pi.0.2.bias": (512,),
        "pi.1.weight": (2 * EXPECTED_ACTION_DIM, 512),
        "pi.1.bias": (2 * EXPECTED_ACTION_DIM,),
    }


def _validate_source_module(
    module: Any, *, label: str
) -> tuple[dict[str, np.ndarray], dict[str, Any]]:
    if type(module) is not AsymmetricActorCriticTorchRLModule:
        raise CompositeActorBuildError(
            f"{label} must be the standard AsymmetricActorCriticTorchRLModule"
        )
    if module.inference_only is not True or module.learner_only is not False:
        raise CompositeActorBuildError(f"{label} must be actor-only inference form")
    config = dict(module.model_config)
    if list(config.get("fcnet_hiddens", ())) != list(SOURCE_HIDDENS):
        raise CompositeActorBuildError(f"{label} hidden topology drifted")
    if str(config.get("fcnet_activation", "")).lower() != "tanh":
        raise CompositeActorBuildError(f"{label} activation must be tanh")
    if int(config.get("n_actor", -1)) != EXPECTED_ACTOR_FEATURES:
        raise CompositeActorBuildError(f"{label} n_actor drifted")
    if int(config.get("n_full", -1)) != EXPECTED_FULL_FEATURES:
        raise CompositeActorBuildError(f"{label} n_full drifted")
    if tuple(module.observation_space.shape) != (EXPECTED_FULL_FEATURES,):
        raise CompositeActorBuildError(f"{label} observation space drifted")
    if tuple(module.action_space.shape) != (EXPECTED_ACTION_DIM,):
        raise CompositeActorBuildError(f"{label} action space drifted")
    state = module.get_state()
    if set(state) != EXPECTED_ACTOR_STATE_KEYS:
        raise CompositeActorBuildError(
            f"{label} actor-only state schema drifted: {sorted(state)}"
        )
    expected_shapes = _expected_source_shapes()
    arrays: dict[str, np.ndarray] = {}
    for name in expected_shapes:
        array = _float32(state[name], label=f"{label}.{name}")
        if array.shape != expected_shapes[name]:
            raise CompositeActorBuildError(
                f"{label}.{name} shape {array.shape} != {expected_shapes[name]}"
            )
        arrays[name] = array
    _require_encoder_alias_values(arrays, label)
    first = arrays["pi_encoder.0.weight"]
    clock_zero = _positive_zero_bits(first[:, DISABLED_CLOCK_COLUMNS])
    if not clock_zero:
        raise CompositeActorBuildError(f"{label} disabled clock columns are not +0")
    logstd_weight = arrays["pi.1.weight"][EXPECTED_ACTION_DIM:]
    if not _positive_zero_bits(logstd_weight):
        raise CompositeActorBuildError(f"{label} logstd head is not constant")
    return arrays, {
        "standard_module_class": True,
        "inference_only": True,
        "fcnet_hiddens": list(SOURCE_HIDDENS),
        "fcnet_activation": "tanh",
        "n_actor": EXPECTED_ACTOR_FEATURES,
        "n_full": EXPECTED_FULL_FEATURES,
        "action_dim": EXPECTED_ACTION_DIM,
        "encoder_alias_values_exact": True,
        "disabled_clock_columns_positive_zero": True,
        "logstd_weight_positive_zero": True,
        "actor_digest": warm_start.actor_state_digest(arrays),
    }


def _validate_source_pair(
    p2: Any, r5: Any
) -> tuple[dict[str, np.ndarray], dict[str, np.ndarray], dict[str, Any]]:
    p2_state, p2_audit = _validate_source_module(p2, label="P2")
    r5_state, r5_audit = _validate_source_module(r5, label="R5")
    if _box_contract(p2.observation_space) != _box_contract(r5.observation_space):
        raise CompositeActorBuildError("P2/R5 observation spaces differ")
    if _box_contract(p2.action_space) != _box_contract(r5.action_space):
        raise CompositeActorBuildError("P2/R5 action spaces differ")
    p2_logstd_bias = p2_state["pi.1.bias"][EXPECTED_ACTION_DIM:]
    r5_logstd_bias = r5_state["pi.1.bias"][EXPECTED_ACTION_DIM:]
    if not np.array_equal(p2_logstd_bias, r5_logstd_bias):
        raise CompositeActorBuildError("P2/R5 logstd biases differ")
    if not np.array_equal(
        p2_state["pi.1.weight"][EXPECTED_ACTION_DIM:],
        r5_state["pi.1.weight"][EXPECTED_ACTION_DIM:],
    ):
        raise CompositeActorBuildError("P2/R5 logstd weights differ")
    return (
        p2_state,
        r5_state,
        {
            "p2": p2_audit,
            "r5": r5_audit,
            "spaces_byte_exact": True,
            "logstd_sources_byte_exact": True,
            "logstd_bias": p2_logstd_bias.astype(float).tolist(),
            "sigma": np.exp(p2_logstd_bias.astype(np.float64)).tolist(),
        },
    )


def build_composite_state(
    p2_state: Mapping[str, Any], r5_state: Mapping[str, Any]
) -> dict[str, np.ndarray]:
    """Return the actor-only 512 state implementing ``0.70*P2 + 0.30*R5``.

    Both inputs must already satisfy the fixed standard 256 topology.  Module
    validation is performed by :func:`build_composite_module`; this lower-level
    state API is intentionally reusable by an execution runner after its own
    source-lineage checks.
    """

    expected = _expected_source_shapes()
    sources: dict[str, dict[str, np.ndarray]] = {}
    for label, source in (("P2", p2_state), ("R5", r5_state)):
        if set(source) != EXPECTED_ACTOR_STATE_KEYS:
            raise CompositeActorBuildError(f"{label} state schema drifted")
        arrays = {
            name: _float32(source[name], label=f"{label}.{name}") for name in expected
        }
        if any(arrays[name].shape != shape for name, shape in expected.items()):
            raise CompositeActorBuildError(f"{label} state shape drifted")
        _require_encoder_alias_values(arrays, label)
        sources[label] = arrays
    p2 = sources["P2"]
    r5 = sources["R5"]
    if not _positive_zero_bits(p2["pi_encoder.0.weight"][:, :2]):
        raise CompositeActorBuildError("P2 clock columns are not positive zero")
    if not _positive_zero_bits(r5["pi_encoder.0.weight"][:, :2]):
        raise CompositeActorBuildError("R5 clock columns are not positive zero")
    if not _positive_zero_bits(p2["pi.1.weight"][EXPECTED_ACTION_DIM:]):
        raise CompositeActorBuildError("P2 logstd weight is not positive zero")
    if not _positive_zero_bits(r5["pi.1.weight"][EXPECTED_ACTION_DIM:]):
        raise CompositeActorBuildError("R5 logstd weight is not positive zero")
    if not np.array_equal(
        p2["pi.1.bias"][EXPECTED_ACTION_DIM:],
        r5["pi.1.bias"][EXPECTED_ACTION_DIM:],
    ):
        raise CompositeActorBuildError("source logstd biases differ")

    first_weight = np.ascontiguousarray(
        np.concatenate((p2["pi_encoder.0.weight"], r5["pi_encoder.0.weight"]), axis=0),
        dtype=np.float32,
    )
    first_bias = np.ascontiguousarray(
        np.concatenate((p2["pi_encoder.0.bias"], r5["pi_encoder.0.bias"]), axis=0),
        dtype=np.float32,
    )
    second_weight = np.zeros(TARGET_HIDDENS, dtype=np.float32)
    second_weight[:256, :256] = p2["pi_encoder.2.weight"]
    second_weight[256:, 256:] = r5["pi_encoder.2.weight"]
    second_bias = np.ascontiguousarray(
        np.concatenate((p2["pi_encoder.2.bias"], r5["pi_encoder.2.bias"]), axis=0),
        dtype=np.float32,
    )
    head_weight = np.zeros((2 * EXPECTED_ACTION_DIM, 512), dtype=np.float32)
    head_weight[:EXPECTED_ACTION_DIM, :256] = (
        P2_WEIGHT * p2["pi.1.weight"][:EXPECTED_ACTION_DIM]
    )
    head_weight[:EXPECTED_ACTION_DIM, 256:] = (
        R5_WEIGHT * r5["pi.1.weight"][:EXPECTED_ACTION_DIM]
    )
    head_bias = np.empty(2 * EXPECTED_ACTION_DIM, dtype=np.float32)
    head_bias[:EXPECTED_ACTION_DIM] = (
        P2_WEIGHT * p2["pi.1.bias"][:EXPECTED_ACTION_DIM]
        + R5_WEIGHT * r5["pi.1.bias"][:EXPECTED_ACTION_DIM]
    )
    head_bias[EXPECTED_ACTION_DIM:] = p2["pi.1.bias"][EXPECTED_ACTION_DIM:]

    result = {
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
    for name, shape in _expected_target_shapes().items():
        if result[name].shape != shape or result[name].dtype != np.float32:
            raise CompositeActorBuildError(f"constructed state drifted at {name}")
    _require_encoder_alias_values(result, "composite")
    if not _positive_zero_bits(first_weight[:, DISABLED_CLOCK_COLUMNS]):
        raise CompositeActorBuildError("constructed clock columns are not +0")
    if not _positive_zero_bits(head_weight[EXPECTED_ACTION_DIM:]):
        raise CompositeActorBuildError("constructed logstd weight is not +0")
    return result


def _build_composite_bundle(
    p2_checkpoint: str | Path,
    r5_checkpoint: str | Path,
) -> _CompositeBundle:
    p2_path = _absolute_no_follow(p2_checkpoint)
    r5_path = _absolute_no_follow(r5_checkpoint)
    _reject_link_or_reparse_components(p2_path, include_leaf=True)
    _reject_link_or_reparse_components(r5_path, include_leaf=True)
    p2 = RLModule.from_checkpoint(p2_path)
    r5 = RLModule.from_checkpoint(r5_path)
    p2.eval()
    r5.eval()
    p2_state, r5_state, source_audit = _validate_source_pair(p2, r5)
    model_config = dict(p2.model_config)
    model_config.update(
        {
            "fcnet_hiddens": list(TARGET_HIDDENS),
            "fcnet_activation": "tanh",
            "n_actor": EXPECTED_ACTOR_FEATURES,
            "n_full": EXPECTED_FULL_FEATURES,
            "freeze_actor": True,
        }
    )
    module = AsymmetricActorCriticTorchRLModule(
        observation_space=p2.observation_space,
        action_space=p2.action_space,
        inference_only=True,
        learner_only=False,
        model_config=model_config,
        catalog_class=None,
    )
    module.set_state(build_composite_state(p2_state, r5_state))
    module.eval()
    return _CompositeBundle(
        module=module,
        p2=p2,
        r5=r5,
        source_audit=source_audit,
    )


def build_composite_module(
    p2_checkpoint: str | Path = DEFAULT_P2_CHECKPOINT,
    r5_checkpoint: str | Path = DEFAULT_R5_CHECKPOINT,
) -> AsymmetricActorCriticTorchRLModule:
    """Build an in-memory standard 512 composite without writing artifacts."""

    return _build_composite_bundle(p2_checkpoint, r5_checkpoint).module


def deterministic_actor_inputs() -> np.ndarray:
    """Return a fixed non-random input matrix independent of the qualification corpus."""

    columns = np.arange(EXPECTED_ACTOR_FEATURES, dtype=np.float64)
    rows = [
        np.zeros(EXPECTED_ACTOR_FEATURES, dtype=np.float64),
        np.linspace(-1.0, 1.0, EXPECTED_ACTOR_FEATURES, dtype=np.float64),
        np.linspace(1.0, -1.0, EXPECTED_ACTOR_FEATURES, dtype=np.float64),
    ]
    for index in range(1, 14):
        rows.append(
            np.sin((index + 1) * (columns + 0.5) / 11.0)
            + 0.25 * np.cos((index + 3) * (columns + 1.0) / 7.0)
        )
    return np.ascontiguousarray(np.stack(rows), dtype=np.float32)


def _logits(module: Any, observations: np.ndarray) -> np.ndarray:
    tensor = torch.as_tensor(observations, dtype=torch.float32)
    module.eval()
    with torch.no_grad():
        output = module.forward_inference({Columns.OBS: tensor})
    logits = output[Columns.ACTION_DIST_INPUTS]
    return np.ascontiguousarray(logits.detach().cpu().numpy(), dtype=np.float32)


def _direct_equivalence(
    *,
    module: Any,
    p2_module: Any,
    r5_module: Any,
    observations: np.ndarray,
    label: str,
) -> dict[str, Any]:
    values = np.ascontiguousarray(observations, dtype=np.float32)
    if values.ndim != 2 or values.shape[1] != EXPECTED_ACTOR_FEATURES:
        raise CompositeActorBuildError(f"{label} input shape drifted: {values.shape}")
    if len(values) == 0:
        raise CompositeActorBuildError(f"{label} must contain at least one row")
    if not np.all(np.isfinite(values)):
        raise CompositeActorBuildError(f"{label} contains non-finite inputs")
    p2_logits = _logits(p2_module, values)
    r5_logits = _logits(r5_module, values)
    composite_logits = _logits(module, values)
    if not np.array_equal(
        p2_logits[:, EXPECTED_ACTION_DIM:],
        r5_logits[:, EXPECTED_ACTION_DIM:],
    ):
        raise CompositeActorBuildError(f"{label} source logstd outputs drifted")
    expected_mean = np.ascontiguousarray(
        P2_WEIGHT * p2_logits[:, :EXPECTED_ACTION_DIM]
        + R5_WEIGHT * r5_logits[:, :EXPECTED_ACTION_DIM],
        dtype=np.float32,
    )
    mean_error = composite_logits[:, :EXPECTED_ACTION_DIM].astype(np.float64) - (
        expected_mean.astype(np.float64)
    )
    max_abs_error = float(np.max(np.abs(mean_error)))
    scale = float(np.max(np.abs(expected_mean)))
    limit = MEAN_ABS_TOLERANCE + MEAN_REL_TOLERANCE * scale
    mean_equivalent = bool(max_abs_error <= limit)
    logstd_byte_exact = bool(
        np.array_equal(
            composite_logits[:, EXPECTED_ACTION_DIM:],
            p2_logits[:, EXPECTED_ACTION_DIM:],
        )
    )
    if not mean_equivalent or not logstd_byte_exact:
        raise CompositeActorBuildError(
            f"{label} direct equivalence failed: mean error {max_abs_error}, "
            f"limit {limit}, logstd exact={logstd_byte_exact}"
        )
    return {
        "rows": int(len(values)),
        "input_sha256": _array_digest(values),
        "mean_formula": "0.70*P2+0.30*R5",
        "mean_max_abs_error": max_abs_error,
        "mean_error_limit": float(limit),
        "mean_equivalent": True,
        "source_logstd_byte_exact": True,
        "composite_logstd_byte_exact": True,
        "logstd": p2_logits[0, EXPECTED_ACTION_DIM:].astype(float).tolist(),
        "sigma": np.exp(p2_logits[0, EXPECTED_ACTION_DIM:].astype(np.float64)).tolist(),
    }


def _topology_audit(module: Any) -> dict[str, Any]:
    state = module.get_state()
    config = dict(module.model_config)
    checks = {
        "standard_module_class": type(module) is AsymmetricActorCriticTorchRLModule,
        "actor_only_inference": (
            module.inference_only is True and module.learner_only is False
        ),
        "hiddens_512_512": list(config.get("fcnet_hiddens", ()))
        == list(TARGET_HIDDENS),
        "activation_tanh": str(config.get("fcnet_activation", "")).lower() == "tanh",
        "actor_full_action_dims": (
            int(config.get("n_actor", -1)) == EXPECTED_ACTOR_FEATURES
            and int(config.get("n_full", -1)) == EXPECTED_FULL_FEATURES
            and tuple(module.observation_space.shape) == (EXPECTED_FULL_FEATURES,)
            and tuple(module.action_space.shape) == (EXPECTED_ACTION_DIM,)
        ),
        "actor_only_state_schema": set(state) == EXPECTED_ACTOR_STATE_KEYS,
        "encoder_object_alias": (
            module.pi[0] is module.pi_encoder
            and module.pi[0][0].weight is module.pi_encoder[0].weight
            and module.pi[0][2].weight is module.pi_encoder[2].weight
        ),
    }
    expected_shapes = _expected_target_shapes()
    checks["state_shapes"] = set(state) == set(expected_shapes) and all(
        tuple(_array(state[name]).shape) == shape
        for name, shape in expected_shapes.items()
    )
    try:
        _require_encoder_alias_values(state, "composite")
        checks["encoder_state_alias_values"] = True
    except CompositeActorBuildError:
        checks["encoder_state_alias_values"] = False
    first = _array(state.get("pi_encoder.0.weight", np.empty(0)))
    head = _array(state.get("pi.1.weight", np.empty(0)))
    checks["disabled_clock_columns_positive_zero"] = first.shape == (
        512,
        EXPECTED_ACTOR_FEATURES,
    ) and _positive_zero_bits(first[:, DISABLED_CLOCK_COLUMNS])
    checks["logstd_weight_positive_zero"] = head.shape == (
        2 * EXPECTED_ACTION_DIM,
        512,
    ) and _positive_zero_bits(head[EXPECTED_ACTION_DIM:])
    if not all(checks.values()):
        failed = sorted(name for name, passed in checks.items() if not passed)
        raise CompositeActorBuildError(f"composite topology failed: {failed}")
    return {
        "passed": True,
        "checks": checks,
        "topology_id": TOPOLOGY_ID,
        "module_class": (f"{type(module).__module__}.{type(module).__qualname__}"),
        "fcnet_hiddens": list(TARGET_HIDDENS),
        "state_shapes": {
            name: list(_array(state[name]).shape) for name in sorted(state)
        },
        "actor_digest": warm_start.actor_state_digest(state),
    }


def _clock_invariance(module: Any) -> dict[str, Any]:
    baseline = deterministic_actor_inputs()
    changed = baseline.copy()
    changed[:, 0] = np.linspace(-100.0, 100.0, len(changed), dtype=np.float32)
    changed[:, 1] = np.linspace(77.0, -33.0, len(changed), dtype=np.float32)
    before = _logits(module, baseline)
    after = _logits(module, changed)
    exact = bool(np.array_equal(before, after))
    if not exact:
        raise CompositeActorBuildError("disabled gait clock changes actor output")
    return {
        "disabled_clock_columns": list(DISABLED_CLOCK_COLUMNS),
        "output_byte_exact_under_clock_perturbation": True,
    }


def verify_composite_actor(
    module: AsymmetricActorCriticTorchRLModule,
    p2_module: AsymmetricActorCriticTorchRLModule,
    r5_module: AsymmetricActorCriticTorchRLModule,
    observations: np.ndarray,
) -> dict[str, Any]:
    """Fail closed unless topology and functional equivalence both hold."""

    _p2, _r5, source_audit = _validate_source_pair(p2_module, r5_module)
    topology = _topology_audit(module)
    corpus = _direct_equivalence(
        module=module,
        p2_module=p2_module,
        r5_module=r5_module,
        observations=observations,
        label="corpus",
    )
    deterministic = _direct_equivalence(
        module=module,
        p2_module=p2_module,
        r5_module=r5_module,
        observations=deterministic_actor_inputs(),
        label="deterministic_inputs",
    )
    clock = _clock_invariance(module)
    return {
        "passed": True,
        "source_audit": source_audit,
        "topology": topology,
        "corpus_direct_equivalence": corpus,
        "deterministic_direct_equivalence": deterministic,
        "clock_invariance": clock,
    }


def _load_corpus(path: str | Path) -> tuple[np.ndarray, tuple[str, ...]]:
    corpus_path = _absolute_no_follow(path)
    _reject_link_or_reparse_components(corpus_path, include_leaf=True)
    if not corpus_path.is_file() or corpus_path.is_symlink():
        raise CompositeActorBuildError(f"corpus is missing or unsafe: {corpus_path}")
    try:
        with np.load(corpus_path, allow_pickle=False) as corpus:
            observations = np.ascontiguousarray(corpus["observations"])
            feature_names = tuple(str(item) for item in corpus["actor_feature_names"])
    except (OSError, KeyError, ValueError) as exc:
        raise CompositeActorBuildError("invalid R6 verification corpus") from exc
    if observations.dtype != np.float32:
        raise CompositeActorBuildError("corpus observations must be float32")
    if observations.ndim != 2 or observations.shape[1] != EXPECTED_ACTOR_FEATURES:
        raise CompositeActorBuildError("corpus actor observation shape drifted")
    if len(feature_names) != EXPECTED_ACTOR_FEATURES:
        raise CompositeActorBuildError("corpus feature-name count drifted")
    if len(set(feature_names)) != len(feature_names):
        raise CompositeActorBuildError("corpus feature names are not unique")
    if feature_names[:2] != warm_start.DISABLED_GAIT_CLOCK_FEATURES:
        raise CompositeActorBuildError("disabled gait-clock feature order drifted")
    return observations, feature_names


def _write_json_exclusive(path: Path, payload: Mapping[str, Any]) -> None:
    encoded = (
        json.dumps(
            dict(payload),
            indent=2,
            sort_keys=True,
            ensure_ascii=False,
            allow_nan=False,
        ).encode("utf-8")
        + b"\n"
    )
    flags = os.O_CREAT | os.O_EXCL | os.O_WRONLY | getattr(os, "O_BINARY", 0)
    try:
        descriptor = os.open(path, flags, 0o600)
    except OSError as exc:
        raise CompositeActorBuildError(f"refusing to clobber {path}") from exc
    try:
        with os.fdopen(descriptor, "wb") as stream:
            descriptor = -1
            stream.write(encoded)
            stream.flush()
            os.fsync(stream.fileno())
    finally:
        if descriptor >= 0:
            os.close(descriptor)


def save_composite_checkpoint_no_clobber(
    module: AsymmetricActorCriticTorchRLModule,
    output_path: str | Path,
) -> AsymmetricActorCriticTorchRLModule:
    """Exclusively save and reconstruct one composite RLModule checkpoint."""

    output = _absolute_no_follow(output_path)
    _reject_link_or_reparse_components(output, include_leaf=True)
    parent = output.parent
    if not parent.is_dir() or parent.is_symlink():
        raise CompositeActorBuildError(
            f"output parent must be an existing regular directory: {parent}"
        )
    if os.path.lexists(output):
        raise CompositeActorBuildError(f"refusing to clobber output: {output}")
    try:
        os.mkdir(output, 0o700)
    except OSError as exc:
        raise CompositeActorBuildError(
            f"cannot exclusively claim output: {output}"
        ) from exc
    module.save_to_path(output)
    observed_files = {path.name for path in output.iterdir() if path.is_file()}
    if observed_files != RL_MODULE_FILES:
        raise CompositeActorBuildError(
            f"RLModule checkpoint file set drifted: {sorted(observed_files)}"
        )
    reloaded = RLModule.from_checkpoint(output)
    reloaded.eval()
    _topology_audit(reloaded)
    return reloaded


def _standard_512_transplant_audit(
    *,
    checkpoint: Path,
    source_module: AsymmetricActorCriticTorchRLModule,
    feature_names: Sequence[str],
) -> dict[str, Any]:
    """Exercise the actor transplant used by warm-start/checkpoint-zero at 512."""

    config = dict(source_module.model_config)
    config.update({"freeze_actor": False, "freeze_logstd": False})
    fresh_full = AsymmetricActorCriticTorchRLModule(
        observation_space=source_module.observation_space,
        action_space=source_module.action_space,
        inference_only=False,
        learner_only=False,
        model_config=config,
        catalog_class=None,
    )
    before = fresh_full.get_state()
    transplanted, report = warm_start.transplant_actor_state(
        target_state=before,
        target_actor_feature_names=feature_names,
        source_checkpoint=checkpoint,
        source_actor_feature_manifest=checkpoint / ACTOR_FEATURE_MANIFEST,
        mode="drop",
        zero_target_features=warm_start.DISABLED_GAIT_CLOCK_FEATURES,
    )
    actor = warm_start.compare_actor_states(source_module.get_state(), transplanted)
    critic = warm_start.compare_non_actor_states(before, transplanted)
    if not actor["exact"] or not critic["exact"]:
        raise CompositeActorBuildError(
            "512 warm-start/checkpoint-zero actor transplant is not exact"
        )
    fresh_full.set_state(transplanted)
    direct = _logits(source_module, deterministic_actor_inputs())
    transplanted_logits = _logits(fresh_full, deterministic_actor_inputs())
    if not np.array_equal(direct, transplanted_logits):
        raise CompositeActorBuildError(
            "512 warm-start/checkpoint-zero forward surface drifted"
        )
    return {
        "passed": True,
        "required_target_fcnet_hiddens": list(TARGET_HIDDENS),
        "warm_start_transplant_actor_exact": True,
        "checkpoint_zero_standard_actor_transplant_exact": True,
        "fresh_critic_preserved_exact": True,
        "forward_surface_byte_exact": True,
        "source_state_is_actor_only": report["source_state_is_actor_only"],
        "source_actor_digest": report["source_actor_digest"],
        "target_actor_digest_after": report["target_actor_digest_after"],
        "target_non_actor_keys_preserved": report["target_non_actor_keys_preserved"],
    }


def build_verify_save_composite(
    *,
    output_path: str | Path,
    p2_checkpoint: str | Path = DEFAULT_P2_CHECKPOINT,
    r5_checkpoint: str | Path = DEFAULT_R5_CHECKPOINT,
    corpus_path: str | Path = DEFAULT_CORPUS_PATH,
) -> dict[str, Any]:
    """Build, verify, exclusively save, reload, and reverify the R6 actor."""

    output = _absolute_no_follow(output_path)
    p2_path = _absolute_no_follow(p2_checkpoint)
    r5_path = _absolute_no_follow(r5_checkpoint)
    corpus = _absolute_no_follow(corpus_path)
    observations, feature_names = _load_corpus(corpus)
    bundle = _build_composite_bundle(p2_path, r5_path)
    before_save = verify_composite_actor(
        bundle.module, bundle.p2, bundle.r5, observations
    )
    reloaded = save_composite_checkpoint_no_clobber(bundle.module, output)
    after_reload = verify_composite_actor(reloaded, bundle.p2, bundle.r5, observations)
    if (
        before_save["topology"]["actor_digest"]
        != after_reload["topology"]["actor_digest"]
    ):
        raise CompositeActorBuildError("actor digest changed across save/reload")

    actor_manifest = {
        "schema_version": SCHEMA_VERSION,
        "status": "H0_V12R6_COMPOSITE_ACTOR_FEATURE_CONTRACT",
        "topology_id": TOPOLOGY_ID,
        "actor_feature_count": len(feature_names),
        "actor_feature_names": list(feature_names),
        "actor_digest": warm_start.actor_state_digest(reloaded.get_state()),
        "module_state_sha256": _sha256_file(output / "module_state.pkl"),
        "fcnet_hiddens": list(TARGET_HIDDENS),
        "disabled_clock_columns": list(DISABLED_CLOCK_COLUMNS),
        "mean_formula": "0.70*P2+0.30*R5",
    }
    _write_json_exclusive(output / ACTOR_FEATURE_MANIFEST, actor_manifest)
    transplant = _standard_512_transplant_audit(
        checkpoint=output,
        source_module=reloaded,
        feature_names=feature_names,
    )
    final_reload = RLModule.from_checkpoint(output)
    final_reload.eval()
    final_verification = verify_composite_actor(
        final_reload, bundle.p2, bundle.r5, observations
    )
    if final_verification["topology"]["actor_digest"] != actor_manifest["actor_digest"]:
        raise CompositeActorBuildError("final reload actor digest drifted")

    manifest = {
        "schema_version": SCHEMA_VERSION,
        "status": "H0_V12R6_COMPOSITE_BUILD_PASS",
        "passed": True,
        "topology_id": TOPOLOGY_ID,
        "output_path": os.fspath(output),
        "mean_formula": "0.70*P2+0.30*R5",
        "weights": {"p2": float(P2_WEIGHT), "r5": float(R5_WEIGHT)},
        "construction": {
            "source_hiddens": list(SOURCE_HIDDENS),
            "target_hiddens": list(TARGET_HIDDENS),
            "first_layer": "concatenated_P2_then_R5",
            "second_layer": "block_diagonal_P2_then_R5",
            "mean_head": "scaled_concatenation_and_blended_bias",
            "logstd_head": "positive_zero_weight_and_byte_exact_source_bias",
        },
        "sources": {
            "p2": _checkpoint_record(p2_path),
            "r5": _checkpoint_record(r5_path),
            "corpus": {
                "path": os.fspath(corpus),
                "sha256": _sha256_file(corpus),
                "rows": int(len(observations)),
                "actor_features": len(feature_names),
            },
        },
        "source_audit": bundle.source_audit,
        "before_save": before_save,
        "after_reload": after_reload,
        "final_reload": final_verification,
        "warm_start_and_checkpoint_zero_512": transplant,
        "actor_feature_manifest": ACTOR_FEATURE_MANIFEST,
        "actor_digest": actor_manifest["actor_digest"],
        "module_state_sha256": actor_manifest["module_state_sha256"],
    }
    _write_json_exclusive(output / BUILD_MANIFEST, manifest)
    return manifest


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output",
        required=True,
        help="Explicit, non-existing RLModule checkpoint directory.",
    )
    parser.add_argument("--p2-checkpoint", default=os.fspath(DEFAULT_P2_CHECKPOINT))
    parser.add_argument("--r5-checkpoint", default=os.fspath(DEFAULT_R5_CHECKPOINT))
    parser.add_argument("--corpus", default=os.fspath(DEFAULT_CORPUS_PATH))
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    result = build_verify_save_composite(
        output_path=args.output,
        p2_checkpoint=args.p2_checkpoint,
        r5_checkpoint=args.r5_checkpoint,
        corpus_path=args.corpus,
    )
    print(json.dumps(result, indent=2, sort_keys=True, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
