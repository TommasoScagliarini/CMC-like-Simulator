"""Create a warm-start actor with controlled Gaussian exploration variance."""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import pickle
import shutil
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np

import warm_start


def _clone(value: Any) -> Any:
    if hasattr(value, "clone"):
        return value.clone()
    if hasattr(value, "copy"):
        return value.copy()
    return copy.deepcopy(value)


def _array(value: Any) -> np.ndarray:
    if hasattr(value, "detach"):
        value = value.detach()
    if hasattr(value, "cpu"):
        value = value.cpu()
    return np.asarray(value)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _load_actor_feature_manifest(
    path: Path,
    *,
    expected_feature_count: int,
) -> dict[str, Any]:
    manifest = json.loads(path.read_text(encoding="utf-8"))
    names = manifest.get("actor_feature_names")
    count = manifest.get("actor_feature_count")
    if not isinstance(names, list) or not all(isinstance(name, str) for name in names):
        raise ValueError(f"invalid actor_feature_names in {path}")
    if count != len(names):
        raise ValueError(
            f"actor feature manifest count mismatch in {path}: {count} != {len(names)}"
        )
    if count != expected_feature_count:
        raise ValueError(
            "actor feature manifest does not match checkpoint input width: "
            f"{count} != {expected_feature_count}"
        )
    return manifest


def configure_constant_std(
    state: Mapping[str, Any],
    *,
    sigma: float | Sequence[float],
    action_dim: int,
) -> tuple[dict[str, Any], dict[str, Any]]:
    """Return a copy with constant log-std and an untouched action mean."""
    if action_dim <= 0:
        raise ValueError("action_dim must be > 0")
    sigma_array = np.asarray(sigma, dtype=float)
    if sigma_array.ndim == 0 or sigma_array.shape == (1,):
        sigma_array = np.full(action_dim, float(sigma_array.reshape(-1)[0]), dtype=float)
    if sigma_array.shape != (action_dim,):
        raise ValueError(f"sigma must be scalar or have {action_dim} values")
    if not np.all(np.isfinite(sigma_array)) or np.any(sigma_array <= 0.0):
        raise ValueError("all sigma values must be finite and > 0")
    output_weight = state.get("pi.1.weight")
    output_bias = state.get("pi.1.bias")
    if output_weight is None or output_bias is None:
        raise KeyError("actor state must contain pi.1.weight and pi.1.bias")
    weight_shape = tuple(int(value) for value in output_weight.shape)
    bias_shape = tuple(int(value) for value in output_bias.shape)
    if weight_shape[0] != 2 * action_dim or bias_shape != (2 * action_dim,):
        raise ValueError(
            "Gaussian output width mismatch: expected "
            f"{2 * action_dim}, got weight {weight_shape} and bias {bias_shape}"
        )

    configured = {key: _clone(value) for key, value in state.items()}
    weight = configured["pi.1.weight"]
    bias = configured["pi.1.bias"]
    log_std = np.log(sigma_array)
    weight[action_dim:, :] = 0.0
    bias[action_dim:] = np.asarray(log_std, dtype=_array(bias).dtype)

    mean_exact = bool(
        np.array_equal(
            _array(configured["pi.1.weight"])[:action_dim],
            _array(state["pi.1.weight"])[:action_dim],
        )
        and np.array_equal(
            _array(configured["pi.1.bias"])[:action_dim],
            _array(state["pi.1.bias"])[:action_dim],
        )
        and all(
            np.array_equal(_array(configured[key]), _array(value))
            for key, value in state.items()
            if key not in {"pi.1.weight", "pi.1.bias"}
        )
    )
    report = {
        "sigma": sigma_array.astype(float).tolist(),
        "log_std": log_std.astype(float).tolist(),
        "action_dim": int(action_dim),
        "mean_parameters_exact": mean_exact,
        "source_logstd_weight_l2": float(
            np.linalg.norm(_array(state["pi.1.weight"])[action_dim:])
        ),
        "configured_logstd_weight_l2": float(
            np.linalg.norm(_array(configured["pi.1.weight"])[action_dim:])
        ),
        "source_logstd_bias": _array(state["pi.1.bias"])[action_dim:]
        .astype(float)
        .tolist(),
        "configured_logstd_bias": _array(configured["pi.1.bias"])[action_dim:]
        .astype(float)
        .tolist(),
    }
    if not mean_exact:
        raise AssertionError("constant-std configuration changed action-mean parameters")
    return configured, report


def _forward(state: Mapping[str, Any], observations: np.ndarray) -> np.ndarray:
    hidden = np.tanh(
        observations @ _array(state["pi.0.0.weight"]).T
        + _array(state["pi.0.0.bias"])
    )
    hidden = np.tanh(
        hidden @ _array(state["pi.0.2.weight"]).T
        + _array(state["pi.0.2.bias"])
    )
    return (
        hidden @ _array(state["pi.1.weight"]).T
        + _array(state["pi.1.bias"])
    )


def _trace_validation(
    source_state: Mapping[str, Any],
    configured_state: Mapping[str, Any],
    trace_path: Path,
    action_dim: int,
) -> dict[str, Any]:
    rows = json.loads(trace_path.read_text(encoding="utf-8"))
    observations = np.asarray(
        [row["actor_observation_vector_before"] for row in rows],
        dtype=np.float32,
    )
    source_logits = _forward(source_state, observations)
    configured_logits = _forward(configured_state, observations)
    configured_logstd = configured_logits[:, action_dim:]
    return {
        "trace": str(trace_path),
        "samples": int(len(observations)),
        "mean_max_abs_diff": float(
            np.max(
                np.abs(
                    source_logits[:, :action_dim]
                    - configured_logits[:, :action_dim]
                )
            )
        ),
        "configured_logstd_min": np.min(configured_logstd, axis=0)
        .astype(float)
        .tolist(),
        "configured_logstd_max": np.max(configured_logstd, axis=0)
        .astype(float)
        .tolist(),
        "configured_std": np.exp(configured_logstd[0]).astype(float).tolist(),
    }


def run(args: argparse.Namespace) -> dict[str, Any]:
    source_module = warm_start.resolve_source_checkpoint(args.source_module)
    source_root = source_module.parent
    output_dir = Path(args.output_dir).expanduser().resolve()
    if output_dir.exists():
        raise FileExistsError(f"output already exists: {output_dir}")

    source_state = warm_start.load_module_state(source_module)
    configured_state, configuration = configure_constant_std(
        source_state,
        sigma=args.sigma,
        action_dim=args.action_dim,
    )
    output_dir.mkdir(parents=True)
    output_module = output_dir / "rl_module_warm_start"
    shutil.copytree(source_module, output_module, copy_function=shutil.copy2)
    with (output_module / "module_state.pkl").open("wb") as handle:
        pickle.dump(configured_state, handle, protocol=pickle.HIGHEST_PROTOCOL)

    source_config = source_root / "training_cfg.resolved.yaml"
    source_feature_manifest = (
        Path(args.source_feature_manifest).expanduser().resolve()
        if args.source_feature_manifest
        else source_root / warm_start.DEFAULT_ACTOR_FEATURE_MANIFEST_NAME
    )
    if not source_config.is_file() or not source_feature_manifest.is_file():
        raise FileNotFoundError(
            "source module must have adjacent training_cfg.resolved.yaml and an "
            "actor feature manifest must be adjacent or supplied explicitly"
        )
    output_config = output_dir / "training_cfg.resolved.yaml"
    shutil.copy2(source_config, output_config)
    actor_input_width = int(_array(source_state["pi.0.0.weight"]).shape[1])
    source_manifest = _load_actor_feature_manifest(
        source_feature_manifest,
        expected_feature_count=actor_input_width,
    )

    reloaded_state = warm_start.load_module_state(output_module)
    reload_comparison = warm_start.compare_actor_states(
        configured_state,
        reloaded_state,
    )
    if not reload_comparison["exact"]:
        raise RuntimeError("configured actor changed during save/reload")
    configured_digest = warm_start.actor_state_digest(reloaded_state)
    actor_manifest = {
        "schema_version": 1,
        "actor_feature_count": int(source_manifest["actor_feature_count"]),
        "actor_feature_names": list(source_manifest["actor_feature_names"]),
        "actor_digest": configured_digest,
        "module_state_sha256": _sha256(output_module / "module_state.pkl"),
        "exploration_sigma": configuration["sigma"],
        "exploration_log_std": configuration["log_std"],
    }
    warm_start.write_report(
        output_dir / warm_start.DEFAULT_ACTOR_FEATURE_MANIFEST_NAME,
        actor_manifest,
    )

    trace_validation = None
    if args.reference_trace:
        trace_validation = _trace_validation(
            source_state,
            reloaded_state,
            Path(args.reference_trace).expanduser().resolve(),
            args.action_dim,
        )
        if trace_validation["mean_max_abs_diff"] != 0.0:
            raise RuntimeError("configured actor changed deterministic trace means")

    report = {
        "ok": True,
        "source_module": str(source_module),
        "source_feature_manifest": str(source_feature_manifest),
        "output_module": str(output_module),
        "source_actor_digest": warm_start.actor_state_digest(source_state),
        "configured_actor_digest": configured_digest,
        "configuration": configuration,
        "trace_validation": trace_validation,
        "save_reload": reload_comparison,
        "module_state_sha256": actor_manifest["module_state_sha256"],
        "source_config_sha256": _sha256(source_config),
        "source_feature_manifest_sha256": _sha256(source_feature_manifest),
        "configured_config_sha256": _sha256(output_config),
        "ppo_updates": 0,
        "critic_trained": False,
    }
    warm_start.write_report(output_dir / "exploration_configuration_report.json", report)
    return report


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-module", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument(
        "--sigma",
        type=float,
        nargs="+",
        required=True,
        help="One shared sigma or one value per action component.",
    )
    parser.add_argument("--action-dim", type=int, default=2)
    parser.add_argument("--reference-trace", default=None)
    parser.add_argument(
        "--source-feature-manifest",
        default=None,
        help=(
            "Explicit actor_feature_manifest.json for checkpoints whose trainer "
            "export did not preserve the adjacent schema manifest."
        ),
    )
    return parser.parse_args()


if __name__ == "__main__":
    print(json.dumps(run(parse_args()), indent=2))
