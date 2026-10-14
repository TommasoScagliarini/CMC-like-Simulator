"""Exact actor initialization for the two unified V1 training objectives.

No baseline checkpoint fallback and no feature conversion are permitted. These
helpers copy only policy tensors into an already fresh actor-critic; optimizer
and critic state are never loaded from the source. Phase flags are deliberately
not policy tensors and remain owned by the destination trainer.
"""

from __future__ import annotations

import copy
import hashlib
import json
import os
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np
from unified_MLP_39.target_availability import (
    TARGET_FEATURES, PRESCRIBED_CLOCK_FEATURES, DEFAULT_TARGET_VELOCITY_SCALE,
    target_contract,
)

SCHEMA_VERSION = "unified_mlp39_v3"
MODULE_CLASS = "unified_MLP_39.asymmetric_rl_module.AsymmetricActorCriticTorchRLModule"
CONTRACT_FILENAME = "unified_contract.json"
_ACTOR_PREFIXES = ("pi_encoder.", "pi.")
_CONTRACT_KEYS = (
    "schema_version", "module_class", "actor_feature_names",
    "observation_feature_names", "action_low", "action_high", "target_channels",
)


def _array(value: Any) -> np.ndarray:
    if hasattr(value, "detach"):
        value = value.detach().cpu()
    return np.asarray(value)


def _clone(value: Any) -> Any:
    if hasattr(value, "detach"):
        return value.detach().clone()
    return value.copy() if hasattr(value, "copy") else copy.deepcopy(value)


def build_contract(
    actor_feature_names: Sequence[str],
    observation_feature_names: Sequence[str],
    action_low: Sequence[float],
    action_high: Sequence[float],
    *, target_velocity_scale: float = DEFAULT_TARGET_VELOCITY_SCALE,
) -> dict[str, Any]:
    contract = {
        "schema_version": SCHEMA_VERSION,
        "target_channels": target_contract(target_velocity_scale),
        "module_class": MODULE_CLASS,
        "actor_feature_names": list(actor_feature_names),
        "observation_feature_names": list(observation_feature_names),
        "action_low": np.asarray(action_low, dtype=float).tolist(),
        "action_high": np.asarray(action_high, dtype=float).tolist(),
    }
    _validate_contract_shape(contract)
    return contract


def _validate_contract_shape(contract: Mapping[str, Any]) -> None:
    for key in _CONTRACT_KEYS:
        if key not in contract:
            raise ValueError(f"unified contract missing {key}")
    if contract["schema_version"] != SCHEMA_VERSION:
        raise ValueError("checkpoint is not a unified_mlp39_v3 prescribed-left phase contract")
    if contract["module_class"] != MODULE_CLASS:
        raise ValueError("checkpoint module_class is not the unified V1 module")
    actor = contract["actor_feature_names"]
    full = contract["observation_feature_names"]
    for key, names in (("actor_feature_names", actor), ("observation_feature_names", full)):
        if not isinstance(names, list) or not names or not all(isinstance(n, str) and n for n in names):
            raise ValueError(f"invalid ordered {key}")
        if len(set(names)) != len(names):
            raise ValueError(f"duplicate {key}")
    if len(actor) != 39 or len(full) != 88 or full[:39] != actor:
        raise ValueError("unified V1 requires 39 actor features and 88 total features as the observation prefix")
    channels = contract["target_channels"]
    if not isinstance(channels, Mapping) or "target_velocity_scale" not in channels:
        raise ValueError("variant39 target velocity scale is missing from the contract")
    expected_channels = target_contract(channels["target_velocity_scale"])
    if (actor[:2] != list(PRESCRIBED_CLOCK_FEATURES) or actor[35:39] != list(TARGET_FEATURES)
            or channels != expected_channels):
        raise ValueError("variant39 target order/normalization/mask contract mismatch")
    low, high = (np.asarray(contract[k], dtype=float) for k in ("action_low", "action_high"))
    if low.shape != (2,) or high.shape != (2,) or not np.all(np.isfinite([low, high])) or np.any(low >= high):
        raise ValueError("unified V1 requires two finite increasing action bounds")


def validate_contract(source: Mapping[str, Any], target: Mapping[str, Any]) -> None:
    """Reject any different ordered feature schema, class or action contract."""
    _validate_contract_shape(source)
    _validate_contract_shape(target)
    for key in _CONTRACT_KEYS:
        if source[key] != target[key]:
            raise ValueError(f"unified contract mismatch: {key}")


def _cli_path(value: str | Path) -> Path:
    raw = os.fspath(value)
    if os.name != "nt":
        raw = raw.replace("\\", "/")
    return Path(raw).expanduser().resolve()


def resolve_module_export(value: str | Path) -> Path:
    """Resolve an explicit RLModule export, module state, run or PPO checkpoint."""
    path = _cli_path(value)
    if path.is_file():
        if path.name != "module_state.pkl":
            raise ValueError(f"expected module_state.pkl: {path}")
        path = path.parent
    suffixes = (
        Path(), Path("learner_group/learner/rl_module/default_policy"),
        Path("rl_module/default_policy"), Path("default_policy"),
        Path("rl_module_final"), Path("rl_module_best"),
    )
    for suffix in suffixes:
        candidate = path / suffix
        if (candidate / "module_state.pkl").is_file():
            return candidate
    raise FileNotFoundError(f"no RLModule export found at {path}")


def read_contract(path: str | Path) -> dict[str, Any]:
    """Read an export/run contract, including RLlib's known PPO module layout."""
    location = _cli_path(path)
    if location.name == CONTRACT_FILENAME and location.is_file():
        candidates = [location]
    else:
        location = location.parent if location.is_file() else location
        candidates = [location / CONTRACT_FILENAME, location.parent / CONTRACT_FILENAME]
        if location.parts[-4:] == ("learner_group", "learner", "rl_module", "default_policy"):
            candidates.append(location.parents[3] / CONTRACT_FILENAME)
    for candidate in candidates:
        if candidate.is_file():
            contract = json.loads(candidate.read_text(encoding="utf-8"))
            if not isinstance(contract, dict):
                raise ValueError(f"unified contract must be an object: {candidate}")
            _validate_contract_shape(contract)
            return contract
    raise FileNotFoundError(f"missing {CONTRACT_FILENAME} beside unified export {location}")


def write_contract(path: str | Path, contract: Mapping[str, Any]) -> Path:
    _validate_contract_shape(contract)
    destination = Path(path)
    if destination.name != CONTRACT_FILENAME:
        destination = destination / CONTRACT_FILENAME
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(json.dumps(dict(contract), indent=2, allow_nan=False) + "\n", encoding="utf-8")
    return destination


def validate_module_contract(module: Any, contract: Mapping[str, Any]) -> None:
    """Validate the actual reconstructed module, including its Gaussian tower."""
    _validate_contract_shape(contract)
    cls = type(module)
    if f"{cls.__module__}.{cls.__name__}" != MODULE_CLASS:
        raise ValueError("loaded module is not the unified V1 class")
    if module._n_actor != len(contract["actor_feature_names"]) or module._n_full != len(contract["observation_feature_names"]):
        raise ValueError("module observation dimensions do not match unified contract")
    if [type(layer).__name__ for layer in module.pi_encoder] != ["Linear", "Tanh", "Linear", "Tanh"]:
        raise ValueError("unified V1 requires two tanh policy hidden layers")
    for attr, key in (("low", "action_low"), ("high", "action_high")):
        if not np.array_equal(np.asarray(getattr(module.action_space, attr), dtype=float), np.asarray(contract[key], dtype=float)):
            raise ValueError(f"module {key} does not match unified contract")
    _validate_actor_state(module.get_state(), contract)


def actor_state(state: Mapping[str, Any]) -> dict[str, Any]:
    return {key: _clone(value) for key, value in state.items() if key.startswith(_ACTOR_PREFIXES)}


def _validate_actor_state(state: Mapping[str, Any], contract: Mapping[str, Any]) -> None:
    expected = {
        "pi_encoder.0.weight": (256, 39), "pi_encoder.0.bias": (256,),
        "pi_encoder.2.weight": (256, 256), "pi_encoder.2.bias": (256,),
        "pi.0.0.weight": (256, 39), "pi.0.0.bias": (256,),
        "pi.0.2.weight": (256, 256), "pi.0.2.bias": (256,),
        "pi.1.weight": (4, 256), "pi.1.bias": (4,),
    }
    actual = actor_state(state)
    if set(actual) != set(expected):
        raise ValueError(f"actor tensor keys differ from unified architecture: {sorted(set(actual) ^ set(expected))}")
    for key, shape in expected.items():
        array = _array(actual[key])
        if array.shape != shape or not np.all(np.isfinite(array)):
            raise ValueError(f"invalid actor tensor shape or nonfinite value: {key}, expected {shape}")
    # pi_encoder is also registered under pi.0. RLlib serializes both names.
    # Reject contradictory aliases rather than letting load order choose weights.
    for suffix in ("0.weight", "0.bias", "2.weight", "2.bias"):
        if not np.array_equal(_array(actual[f"pi_encoder.{suffix}"]), _array(actual[f"pi.0.{suffix}"])):
            raise ValueError(f"inconsistent duplicate actor tensor: {suffix}")


def _digest(state: Mapping[str, Any]) -> str:
    digest = hashlib.sha256()
    for key, value in sorted(state.items()):
        array = np.ascontiguousarray(_array(value))
        digest.update(key.encode("utf-8"))
        digest.update(str(array.dtype).encode("ascii"))
        digest.update(str(array.shape).encode("ascii"))
        digest.update(array.tobytes())
    return digest.hexdigest()


def actor_state_digest(state: Mapping[str, Any]) -> str:
    return _digest(actor_state(state))


def actor_mean_digest(state: Mapping[str, Any], action_dim: int = 2) -> str:
    mean = actor_state(state)
    for key in ("pi.1.weight", "pi.1.bias"):
        mean[key] = mean[key][:action_dim]
    return _digest(mean)


def copy_actor_state(
    source_state: Mapping[str, Any], target_state: Mapping[str, Any],
    source_contract: Mapping[str, Any], target_contract: Mapping[str, Any],
) -> tuple[dict[str, Any], dict[str, Any]]:
    """Copy every policy tensor by identical key/shape; preserve all other state."""
    validate_contract(source_contract, target_contract)
    _validate_actor_state(source_state, source_contract)
    _validate_actor_state(target_state, target_contract)
    combined = {key: _clone(value) for key, value in target_state.items()}
    for key, value in actor_state(source_state).items():
        if _array(value).dtype != _array(target_state[key]).dtype:
            raise ValueError(f"actor dtype mismatch: {key}")
        combined[key] = _clone(value)
    non_actor = {k: v for k, v in target_state.items() if not k.startswith(_ACTOR_PREFIXES)}
    preserved = all(np.array_equal(_array(value), _array(combined[key])) for key, value in non_actor.items())
    report = {
        "actor_exact": actor_state_digest(combined) == actor_state_digest(source_state),
        "critic_and_phase_unchanged": preserved,
        "actor_digest": actor_state_digest(combined),
        "mean_digest": actor_mean_digest(combined),
        "copied_keys": sorted(actor_state(source_state)),
    }
    if not report["actor_exact"] or not preserved:
        raise AssertionError("actor initialization violated exact-copy invariants")
    return combined, report


def load_actor_source(
    path: str | Path, expected_contract: Mapping[str, Any],
) -> tuple[dict[str, Any], dict[str, Any]]:
    module_dir = resolve_module_export(path)
    contract = read_contract(module_dir)
    validate_contract(contract, expected_contract)
    # RLModule reconstruction also verifies the actual saved class rather than
    # trusting a manifest attached to an unrelated baseline checkpoint.
    from ray.rllib.core.rl_module.rl_module import RLModule

    module = RLModule.from_checkpoint(str(module_dir))
    validate_module_contract(module, contract)
    state = actor_state(module.get_state())
    return state, {
        "source_module": str(module_dir), "source_contract": contract,
        "actor_digest": actor_state_digest(state), "mean_digest": actor_mean_digest(state),
        "optimizer_state_loaded": False, "critic_state_loaded": False,
    }


def configure_constant_std(
    state: Mapping[str, Any], *, sigma: float | Sequence[float] = 0.010,
    action_dim: int = 2,
) -> tuple[dict[str, Any], dict[str, Any]]:
    """Set constant Gaussian sigma separately from exact actor initialization."""
    values = np.asarray(sigma, dtype=float)
    if values.ndim == 0 or values.shape == (1,):
        values = np.full(action_dim, float(values.reshape(-1)[0]))
    if action_dim <= 0 or values.shape != (action_dim,) or not np.all(np.isfinite(values)) or np.any(values <= 0):
        raise ValueError("sigma must contain one or action_dim finite positive values")
    weight, bias = state["pi.1.weight"], state["pi.1.bias"]
    if _array(weight).ndim != 2 or _array(weight).shape[0] != 2 * action_dim or _array(bias).shape != (2 * action_dim,):
        raise ValueError("Gaussian actor output shape mismatch")
    configured = {key: _clone(value) for key, value in state.items()}
    configured["pi.1.weight"][action_dim:] = 0.0
    target_bias = configured["pi.1.bias"]
    log_std = np.log(values)
    if hasattr(target_bias, "new_tensor"):
        target_bias[action_dim:] = target_bias.new_tensor(log_std)
    else:
        target_bias[action_dim:] = log_std
    unchanged = actor_mean_digest(configured, action_dim) == actor_mean_digest(state, action_dim)
    if not unchanged:
        raise AssertionError("sigma configuration changed the action-mean network")
    return configured, {
        "sigma": values.tolist(), "log_std": log_std.tolist(),
        "mean_parameters_exact": unchanged, "mean_digest": actor_mean_digest(configured, action_dim),
    }
