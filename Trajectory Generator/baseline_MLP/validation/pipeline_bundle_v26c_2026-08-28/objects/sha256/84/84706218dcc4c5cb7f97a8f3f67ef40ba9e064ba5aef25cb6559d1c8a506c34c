"""Actor-only imitation warm start for ex-novo MLP training.

This module intentionally does not import Ray, OpenSim, or the training stack.
It operates on RLModule state dictionaries so it can be unit-tested with small
synthetic tensors and used after ``PPOConfig.build_algo()`` has created the
target ex-novo actor-critic.
"""

from __future__ import annotations

import copy
import hashlib
import json
import os
import pickle
from pathlib import Path
from typing import Any, Mapping, Sequence

_THIS_DIR = Path(__file__).resolve().parent
_TRAJ_GEN_DIR = _THIS_DIR.parent
_TRAINING_RUNS_ROOT = _TRAJ_GEN_DIR / "runs" / "training"

DEFAULT_WARM_START_SOURCE = (
    _TRAINING_RUNS_ROOT
    / "MLP_imitation_training_06-23-2026_grfsoft_knee1_ankle2_100iter"
    / "rl_module_best"
)
DEFAULT_WARM_START_SOURCE_CONFIG = (
    DEFAULT_WARM_START_SOURCE.parent / "training_cfg.resolved.yaml"
)
DEFAULT_WARM_START_NAME = "asym100_GRFpenalty-lowered"
DEFAULT_WARM_START_MODE = "drop"
DEFAULT_ACTOR_FEATURE_MANIFEST_NAME = "actor_feature_manifest.json"

# The ex-novo config disables the prescribed sound-side gait clock. Keeping
# these source columns would turn a learned cyclic input into the constant
# target value (sin=0, cos=1), effectively injecting an undocumented bias.
DISABLED_GAIT_CLOCK_FEATURES: tuple[str, ...] = (
    "gait_phase_sin",
    "gait_phase_cos",
)

# Historical actor prefix for the official source checkpoint above. It matches
# the source first-layer width (31). Current ex-novo adds the phase FSM features;
# those are target-only and start with zero first-layer columns.
ASYM100_GRF_PENALTY_LOWERED_ACTOR_FEATURES: tuple[str, ...] = (
    "gait_phase_sin",
    "gait_phase_cos",
    "pros_knee_angle",
    "pros_knee_angle_vel",
    "pros_ankle_angle",
    "pros_ankle_angle_vel",
    "SEA_Knee_motor_angle",
    "SEA_Knee_motor_speed",
    "SEA_Ankle_motor_angle",
    "SEA_Ankle_motor_speed",
    "online_left_normal_grf_bw",
    "online_left_in_contact",
    "online_left_heel_strike",
    "online_left_toe_off",
    "online_left_gait_phase_sin",
    "online_left_gait_phase_cos",
    "online_left_cycle_duration_s",
    "pros_knee_angle_previous_endpoint",
    "pros_knee_angle_served_ref",
    "pros_knee_angle_served_ref_vel",
    "pros_knee_angle_served_ref_accel",
    "pros_knee_angle_sea_u",
    "pros_knee_angle_sea_u_abs",
    "pros_knee_angle_sea_u_saturated",
    "pros_ankle_angle_previous_endpoint",
    "pros_ankle_angle_served_ref",
    "pros_ankle_angle_served_ref_vel",
    "pros_ankle_angle_served_ref_accel",
    "pros_ankle_angle_sea_u",
    "pros_ankle_angle_sea_u_abs",
    "pros_ankle_angle_sea_u_saturated",
)

_FIRST_LAYER_WEIGHT_KEYS = (
    "pi_encoder.0.weight",
    "pi.0.0.weight",
)
_DIRECT_ACTOR_KEYS = (
    "pi_encoder.0.bias",
    "pi_encoder.2.weight",
    "pi_encoder.2.bias",
    "pi.0.0.bias",
    "pi.0.2.weight",
    "pi.0.2.bias",
    "pi.1.weight",
    "pi.1.bias",
)
_ACTOR_KEYS = (*_FIRST_LAYER_WEIGHT_KEYS, *_DIRECT_ACTOR_KEYS)


def _cli_path(value: str | Path) -> Path:
    text = os.fspath(value)
    if os.name != "nt":
        text = text.replace("\\", "/")
    return Path(text).expanduser()


def resolve_source_checkpoint(value: str | Path | None = None) -> Path:
    """Resolve a warm-start source.

    Accepted forms:
    - ``None`` -> official asym100 GRF-soft ``rl_module_best``.
    - module directory containing ``module_state.pkl``.
    - run directory containing ``rl_module_best/module_state.pkl``.
    - direct path to ``module_state.pkl``.
    """
    path = _cli_path(value) if value else DEFAULT_WARM_START_SOURCE
    if not path.is_absolute():
        cwd_candidate = (Path.cwd() / path).resolve()
        if cwd_candidate.exists():
            path = cwd_candidate
        else:
            path = (_TRAJ_GEN_DIR / path).resolve()
    else:
        path = path.resolve()

    if path.is_file():
        if path.name != "module_state.pkl":
            raise FileNotFoundError(f"warm-start source is not module_state.pkl: {path}")
        return path.parent
    if (path / "module_state.pkl").is_file():
        return path
    if (path / "rl_module_best" / "module_state.pkl").is_file():
        return path / "rl_module_best"
    raise FileNotFoundError(
        "warm-start source must be an rl_module directory, run directory, or "
        f"module_state.pkl file: {path}"
    )


def resolve_source_config(value: str | Path | None, source_checkpoint: Path) -> Path:
    if value:
        path = _cli_path(value)
        if not path.is_absolute():
            cwd_candidate = (Path.cwd() / path).resolve()
            path = cwd_candidate if cwd_candidate.exists() else (_TRAJ_GEN_DIR / path)
        return path.resolve()
    candidate = source_checkpoint.parent / "training_cfg.resolved.yaml"
    return candidate.resolve()


def _resolve_auxiliary_path(value: str | Path) -> Path:
    path = _cli_path(value)
    if path.is_absolute():
        return path.resolve()
    cwd_candidate = (Path.cwd() / path).resolve()
    return cwd_candidate if cwd_candidate.exists() else (_TRAJ_GEN_DIR / path).resolve()


def _load_actor_feature_manifest(path: Path) -> tuple[tuple[str, ...], dict[str, Any]]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise ValueError(f"invalid actor feature manifest JSON: {path}") from exc
    if not isinstance(payload, Mapping):
        raise ValueError(f"actor feature manifest must contain an object: {path}")
    raw_names = payload.get("actor_feature_names")
    if not isinstance(raw_names, list) or not raw_names:
        raise ValueError(
            "actor feature manifest must contain a non-empty "
            f"actor_feature_names list: {path}"
        )
    names = tuple(str(name) for name in raw_names)
    if len(set(names)) != len(names):
        raise ValueError(f"actor feature manifest contains duplicate names: {path}")
    declared_count = payload.get("actor_feature_count")
    if declared_count is not None and int(declared_count) != len(names):
        raise ValueError(
            f"actor feature manifest count {declared_count} != {len(names)} names: "
            f"{path}"
        )
    return names, dict(payload)


def resolve_source_actor_features(
    *,
    source_checkpoint: Path,
    source_state: Mapping[str, Any],
    source_actor_feature_manifest: str | Path | None = None,
) -> tuple[tuple[str, ...], dict[str, Any]]:
    """Resolve and validate the ordered observation contract of an actor.

    New checkpoints should carry ``actor_feature_manifest.json`` either inside
    the module directory or in its parent run directory. The built-in 31-column
    manifest remains only as a compatibility fallback for the historical
    official imitation checkpoint.
    """
    explicit = source_actor_feature_manifest is not None
    manifest_path = (
        _resolve_auxiliary_path(source_actor_feature_manifest)
        if explicit
        else None
    )
    if manifest_path is None:
        candidates = (
            source_checkpoint / DEFAULT_ACTOR_FEATURE_MANIFEST_NAME,
            source_checkpoint.parent / DEFAULT_ACTOR_FEATURE_MANIFEST_NAME,
        )
        manifest_path = next((path for path in candidates if path.is_file()), None)

    manifest_payload: dict[str, Any] | None = None
    if manifest_path is not None:
        if not manifest_path.is_file():
            raise FileNotFoundError(
                f"warm-start actor feature manifest does not exist: {manifest_path}"
            )
        source_features, manifest_payload = _load_actor_feature_manifest(manifest_path)
        resolution = "explicit_manifest" if explicit else "adjacent_manifest"
    else:
        source_features = ASYM100_GRF_PENALTY_LOWERED_ACTOR_FEATURES
        resolution = "legacy_builtin_manifest"

    first_shape = _shape(source_state.get("pi_encoder.0.weight"))
    if first_shape is None or len(first_shape) != 2:
        raise ValueError("source first actor layer is not a rank-2 tensor")
    if first_shape[1] != len(source_features):
        raise ValueError(
            "source actor feature manifest does not match checkpoint width: "
            f"{len(source_features)} names vs shape {first_shape}. Provide the "
            "checkpoint's actor_feature_manifest.json explicitly."
        )

    actual_digest = actor_state_digest(source_state)
    declared_digest = (
        str(manifest_payload.get("actor_digest"))
        if manifest_payload and manifest_payload.get("actor_digest")
        else None
    )
    if declared_digest is not None and declared_digest != actual_digest:
        raise ValueError(
            "actor feature manifest digest does not match source checkpoint: "
            f"declared {declared_digest}, actual {actual_digest}"
        )
    return source_features, {
        "path": str(manifest_path) if manifest_path is not None else None,
        "resolution": resolution,
        "declared_actor_digest": declared_digest,
        "validated_actor_digest": actual_digest,
    }


def load_module_state(module_dir: str | Path) -> dict[str, Any]:
    path = Path(module_dir) / "module_state.pkl"
    with path.open("rb") as handle:
        state = pickle.load(handle)
    if not isinstance(state, Mapping):
        raise TypeError(f"{path} did not contain a mapping state dict")
    return dict(state)


def _clone_value(value: Any) -> Any:
    if hasattr(value, "clone"):
        return value.clone()
    return copy.deepcopy(value)


def _shape(value: Any) -> tuple[int, ...] | None:
    shape = getattr(value, "shape", None)
    return tuple(int(dim) for dim in shape) if shape is not None else None


def _norm(value: Any) -> float | None:
    try:
        if hasattr(value, "detach"):
            value = value.detach()
        if hasattr(value, "norm"):
            return float(value.norm().item())
        if hasattr(value, "__array__"):
            import numpy as np

            return float(np.linalg.norm(np.asarray(value)))
    except Exception:
        return None
    return None


def _as_numpy(value: Any):
    import numpy as np

    if hasattr(value, "detach"):
        value = value.detach()
    if hasattr(value, "cpu"):
        value = value.cpu()
    return np.asarray(value)


def _values_equal(left: Any, right: Any) -> bool:
    try:
        import numpy as np

        return bool(np.array_equal(_as_numpy(left), _as_numpy(right)))
    except Exception:
        return left == right


def tensor_digest(value: Any) -> str:
    """Stable SHA-256 over tensor dtype, shape, and contiguous bytes."""
    import numpy as np

    array = np.ascontiguousarray(_as_numpy(value))
    digest = hashlib.sha256()
    digest.update(str(array.dtype).encode("ascii"))
    digest.update(repr(tuple(int(dim) for dim in array.shape)).encode("ascii"))
    digest.update(array.tobytes(order="C"))
    return digest.hexdigest()


def actor_state_digest(state: Mapping[str, Any]) -> str:
    """Stable aggregate digest for all actor tensors in a module state."""
    digest = hashlib.sha256()
    for key in sorted(_ACTOR_KEYS):
        if key not in state:
            raise KeyError(f"actor state is missing {key}")
        digest.update(key.encode("utf-8"))
        digest.update(tensor_digest(state[key]).encode("ascii"))
    return digest.hexdigest()


def compare_actor_states(
    expected: Mapping[str, Any],
    actual: Mapping[str, Any],
) -> dict[str, Any]:
    """Return exact and maximum-error checks for two actor state mappings."""
    import numpy as np

    missing = [key for key in _ACTOR_KEYS if key not in expected or key not in actual]
    per_key: dict[str, dict[str, Any]] = {}
    max_abs_diff = 0.0
    for key in _ACTOR_KEYS:
        if key in missing:
            continue
        left = _as_numpy(expected[key])
        right = _as_numpy(actual[key])
        shape_match = left.shape == right.shape
        diff = (
            float(np.max(np.abs(left.astype(float) - right.astype(float))))
            if shape_match and left.size
            else (0.0 if shape_match else float("inf"))
        )
        max_abs_diff = max(max_abs_diff, diff)
        per_key[key] = {
            "shape_match": bool(shape_match),
            "exact": bool(shape_match and np.array_equal(left, right)),
            "max_abs_diff": diff,
        }
    exact = not missing and all(item["exact"] for item in per_key.values())
    return {
        "exact": bool(exact),
        "max_abs_diff": float(max_abs_diff),
        "missing_keys": missing,
        "expected_digest": actor_state_digest(expected) if not missing else None,
        "actual_digest": actor_state_digest(actual) if not missing else None,
        "per_key": per_key,
    }


def compare_non_actor_states(
    expected: Mapping[str, Any],
    actual: Mapping[str, Any],
) -> dict[str, Any]:
    """Compare all tensor leaves outside the policy network."""
    import numpy as np

    expected_keys = sorted(set(expected) - set(_ACTOR_KEYS))
    actual_keys = sorted(set(actual) - set(_ACTOR_KEYS))
    missing = sorted(set(expected_keys) - set(actual_keys))
    unexpected = sorted(set(actual_keys) - set(expected_keys))
    per_key: dict[str, dict[str, Any]] = {}
    max_abs_diff = 0.0
    digest_expected = hashlib.sha256()
    digest_actual = hashlib.sha256()
    for key in sorted(set(expected_keys) & set(actual_keys)):
        left = _as_numpy(expected[key])
        right = _as_numpy(actual[key])
        shape_match = left.shape == right.shape
        diff = (
            float(np.max(np.abs(left.astype(float) - right.astype(float))))
            if shape_match and left.size
            else (0.0 if shape_match else float("inf"))
        )
        max_abs_diff = max(max_abs_diff, diff)
        exact = bool(shape_match and np.array_equal(left, right))
        per_key[key] = {
            "shape_match": bool(shape_match),
            "exact": exact,
            "max_abs_diff": diff,
        }
        digest_expected.update(key.encode("utf-8"))
        digest_expected.update(tensor_digest(expected[key]).encode("ascii"))
        digest_actual.update(key.encode("utf-8"))
        digest_actual.update(tensor_digest(actual[key]).encode("ascii"))
    exact = (
        bool(expected_keys)
        and not missing
        and not unexpected
        and all(item["exact"] for item in per_key.values())
    )
    return {
        "exact": bool(exact),
        "keys": expected_keys,
        "missing_keys": missing,
        "unexpected_keys": unexpected,
        "max_abs_diff": float(max_abs_diff),
        "expected_digest": digest_expected.hexdigest(),
        "actual_digest": digest_actual.hexdigest(),
        "per_key": per_key,
    }


def find_actor_state(value: Any) -> Mapping[str, Any] | None:
    """Find an actor state recursively inside an RLlib component state."""
    if not isinstance(value, Mapping):
        return None
    if all(key in value for key in _ACTOR_KEYS):
        return value
    for child in value.values():
        found = find_actor_state(child)
        if found is not None:
            return found
    return None


def _zero_column(matrix: Any, column_index: int) -> None:
    column = matrix[:, column_index]
    if hasattr(column, "zero_"):
        column.zero_()
    else:
        matrix[:, column_index] = 0.0


def transplant_actor_state(
    *,
    target_state: Mapping[str, Any],
    target_actor_feature_names: Sequence[str],
    source_checkpoint: str | Path | None = None,
    source_config: str | Path | None = None,
    source_actor_feature_manifest: str | Path | None = None,
    mode: str = DEFAULT_WARM_START_MODE,
    zero_target_features: Sequence[str] = (),
) -> tuple[dict[str, Any], dict[str, Any]]:
    """Return ``target_state`` with actor weights warm-started from imitation.

    Only ``mode='drop'`` is implemented: source-only features are ignored and
    target-only first-layer columns are zeroed. Critic/value keys are copied from
    the target untouched.
    """
    if mode != "drop":
        raise ValueError("Only warm-start mode 'drop' is implemented")

    source_dir = resolve_source_checkpoint(source_checkpoint)
    source_config_path = resolve_source_config(source_config, source_dir)
    source_state = load_module_state(source_dir)
    source_features, source_feature_manifest = resolve_source_actor_features(
        source_checkpoint=source_dir,
        source_state=source_state,
        source_actor_feature_manifest=source_actor_feature_manifest,
    )
    target_features = tuple(str(name) for name in target_actor_feature_names)
    if not target_features:
        raise ValueError("target_actor_feature_names is empty")

    source_index = {name: idx for idx, name in enumerate(source_features)}
    target_index = {name: idx for idx, name in enumerate(target_features)}
    forced_zero_features = tuple(dict.fromkeys(str(name) for name in zero_target_features))
    unknown_zero_features = [
        name for name in forced_zero_features if name not in target_index
    ]
    if unknown_zero_features:
        raise ValueError(
            "zero_target_features are absent from the target actor schema: "
            f"{unknown_zero_features}"
        )
    shared_features_zeroed = [
        name for name in forced_zero_features if name in source_index
    ]
    copied_features = [
        name
        for name in source_features
        if name in target_index and name not in shared_features_zeroed
    ]
    source_only_features = [
        name for name in source_features if name not in target_index
    ]
    target_only_features = [
        name for name in target_features if name not in source_index
    ]
    zeroed_target_features = list(
        dict.fromkeys([*target_only_features, *forced_zero_features])
    )

    new_state = {key: _clone_value(value) for key, value in target_state.items()}
    missing = [key for key in _ACTOR_KEYS if key not in source_state or key not in new_state]
    if missing:
        raise KeyError(f"missing actor key(s) for warm-start transplant: {missing}")

    source_first_shape = _shape(source_state["pi_encoder.0.weight"])
    if source_first_shape is None or len(source_first_shape) != 2:
        raise ValueError("source first actor layer is not a rank-2 tensor")
    if source_first_shape[1] != len(source_features):
        raise ValueError(
            "source actor feature manifest does not match checkpoint width: "
            f"{len(source_features)} names vs shape {source_first_shape}"
        )

    for key in _FIRST_LAYER_WEIGHT_KEYS:
        source_weight = source_state[key]
        target_weight = new_state[key]
        source_shape = _shape(source_weight)
        target_shape = _shape(target_weight)
        if source_shape is None or target_shape is None:
            raise ValueError(f"{key} does not expose a tensor shape")
        if source_shape[0] != target_shape[0]:
            raise ValueError(
                f"{key} hidden width mismatch: source {source_shape}, target {target_shape}"
            )
        if source_shape[1] != len(source_features):
            raise ValueError(
                f"{key} source input width {source_shape[1]} != "
                f"manifest length {len(source_features)}"
            )
        if target_shape[1] != len(target_features):
            raise ValueError(
                f"{key} target input width {target_shape[1]} != "
                f"target actor feature count {len(target_features)}"
            )
        for name in zeroed_target_features:
            _zero_column(target_weight, target_index[name])
        for name in copied_features:
            target_weight[:, target_index[name]] = source_weight[:, source_index[name]]

    for key in _DIRECT_ACTOR_KEYS:
        if _shape(source_state[key]) != _shape(new_state[key]):
            raise ValueError(
                f"{key} shape mismatch: source {_shape(source_state[key])}, "
                f"target {_shape(new_state[key])}"
            )
        new_state[key] = _clone_value(source_state[key])

    non_actor_keys = sorted(set(new_state) - set(_ACTOR_KEYS))
    non_actor_unchanged = all(
        _values_equal(target_state[key], new_state[key]) for key in non_actor_keys
    )
    if not non_actor_unchanged:
        raise AssertionError("warm-start transplant changed non-actor target state")

    report = {
        "warm_start_name": DEFAULT_WARM_START_NAME,
        "source_checkpoint": str(source_dir),
        "source_config": str(source_config_path),
        "source_config_exists": source_config_path.is_file(),
        "source_actor_feature_manifest": source_feature_manifest,
        "removed_feature_mode": mode,
        "critic_init_mode": "fresh_target_untouched",
        "source_actor_feature_names": list(source_features),
        "target_actor_feature_names": list(target_features),
        "copied_features": copied_features,
        "source_only_features_dropped": source_only_features,
        "target_only_features_zeroed": target_only_features,
        "shared_features_zeroed": shared_features_zeroed,
        "zeroed_target_features": zeroed_target_features,
        "zero_target_feature_reason": (
            "target signal intentionally unavailable; prevent constant-input bias"
            if shared_features_zeroed
            else None
        ),
        "source_state_is_actor_only": not bool(set(source_state) - set(_ACTOR_KEYS)),
        "source_non_actor_keys": sorted(set(source_state) - set(_ACTOR_KEYS)),
        "target_non_actor_keys_preserved": non_actor_keys,
        "target_non_actor_state_unchanged": bool(non_actor_unchanged),
        "source_actor_digest": actor_state_digest(source_state),
        "target_actor_digest_after": actor_state_digest(new_state),
        "weight_shapes": {
            key: {
                "source": list(_shape(source_state[key]) or ()),
                "target": list(_shape(target_state[key]) or ()),
            }
            for key in _ACTOR_KEYS
        },
        "weight_norms": {
            key: {
                "source": _norm(source_state[key]),
                "target_before": _norm(target_state[key]),
                "target_after": _norm(new_state[key]),
            }
            for key in _ACTOR_KEYS
        },
    }
    return new_state, report


def write_report(path: str | Path, report: Mapping[str, Any]) -> Path:
    report_path = Path(path)
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(json.dumps(dict(report), indent=2), encoding="utf-8")
    return report_path
