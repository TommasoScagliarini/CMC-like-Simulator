"""Project an actor-only update onto a nominal action-mean trust region."""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import pickle
import shutil
from pathlib import Path
from typing import Any, Mapping

import numpy as np

import warm_start


def _array(value: Any) -> np.ndarray:
    if hasattr(value, "detach"):
        value = value.detach()
    if hasattr(value, "cpu"):
        value = value.cpu()
    return np.asarray(value)


def _clone(value: Any) -> Any:
    if hasattr(value, "clone"):
        return value.clone()
    return copy.deepcopy(value)


def _is_actor_key(key: str) -> bool:
    return key.startswith("pi.") or key.startswith("pi_encoder.")


def interpolate_actor_states(
    source: Mapping[str, Any], candidate: Mapping[str, Any], alpha: float
) -> dict[str, Any]:
    if not 0.0 <= alpha <= 1.0:
        raise ValueError("alpha must be in [0, 1]")
    if set(source) != set(candidate):
        raise ValueError("source and candidate state keys differ")
    projected = {key: _clone(value) for key, value in source.items()}
    for key, source_value in source.items():
        if not _is_actor_key(key):
            continue
        candidate_value = candidate[key]
        if tuple(source_value.shape) != tuple(candidate_value.shape):
            raise ValueError(f"actor tensor shape mismatch for {key}")
        projected[key] = source_value + (candidate_value - source_value) * alpha
    return projected


def _forward(state: Mapping[str, Any], observations: np.ndarray) -> np.ndarray:
    hidden = np.tanh(
        observations @ _array(state["pi.0.0.weight"]).T
        + _array(state["pi.0.0.bias"])
    )
    hidden = np.tanh(
        hidden @ _array(state["pi.0.2.weight"]).T
        + _array(state["pi.0.2.bias"])
    )
    return hidden @ _array(state["pi.1.weight"]).T + _array(state["pi.1.bias"])


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _nominal_shift(
    source: Mapping[str, Any],
    candidate: Mapping[str, Any],
    observations: np.ndarray,
    action_dim: int,
) -> dict[str, float]:
    difference = (
        _forward(candidate, observations)[:, :action_dim]
        - _forward(source, observations)[:, :action_dim]
    )
    return {
        "max_abs": float(np.max(np.abs(difference))),
        "rmse": float(np.sqrt(np.mean(np.square(difference)))),
    }


def run(args: argparse.Namespace) -> dict[str, Any]:
    source_module = warm_start.resolve_source_checkpoint(args.source_module)
    candidate_module = warm_start.resolve_source_checkpoint(args.candidate_module)
    output_dir = Path(args.output_dir).expanduser().resolve()
    if output_dir.exists():
        raise FileExistsError(output_dir)
    source = warm_start.load_module_state(source_module)
    candidate = warm_start.load_module_state(candidate_module)
    rows = json.loads(Path(args.reference_trace).read_text(encoding="utf-8"))
    observations = np.asarray(
        [row["actor_observation_vector_before"] for row in rows], dtype=np.float32
    )

    full_shift = _nominal_shift(
        source, candidate, observations, args.action_dim
    )
    if full_shift["max_abs"] <= args.max_nominal_mean_shift:
        alpha = 1.0
    else:
        low, high = 0.0, 1.0
        for _ in range(40):
            middle = 0.5 * (low + high)
            state = interpolate_actor_states(source, candidate, middle)
            shift = _nominal_shift(source, state, observations, args.action_dim)
            if shift["max_abs"] <= args.max_nominal_mean_shift:
                low = middle
            else:
                high = middle
        alpha = low
    projected = interpolate_actor_states(source, candidate, alpha)
    projected_shift = _nominal_shift(
        source, projected, observations, args.action_dim
    )
    if projected_shift["max_abs"] > args.max_nominal_mean_shift + 1e-9:
        raise RuntimeError("projected actor exceeds nominal trust-region budget")

    output_dir.mkdir(parents=True)
    output_module = output_dir / "rl_module_projected"
    shutil.copytree(source_module, output_module, copy_function=shutil.copy2)
    with (output_module / "module_state.pkl").open("wb") as handle:
        pickle.dump(projected, handle, protocol=pickle.HIGHEST_PROTOCOL)

    source_root = source_module.parent
    for name in ("training_cfg.resolved.yaml",):
        shutil.copy2(source_root / name, output_dir / name)
    manifest_path = source_root / warm_start.DEFAULT_ACTOR_FEATURE_MANIFEST_NAME
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest.update(
        {
            "actor_digest": warm_start.actor_state_digest(projected),
            "module_state_sha256": _sha256(output_module / "module_state.pkl"),
            "adaptation": "nominal_trust_region_projection",
            "source_actor_digest": warm_start.actor_state_digest(source),
            "candidate_actor_digest": warm_start.actor_state_digest(candidate),
            "projection_alpha": float(alpha),
        }
    )
    warm_start.write_report(
        output_dir / warm_start.DEFAULT_ACTOR_FEATURE_MANIFEST_NAME, manifest
    )

    reloaded = warm_start.load_module_state(output_module)
    reload_check = warm_start.compare_actor_states(projected, reloaded)
    non_actor_check = warm_start.compare_non_actor_states(source, reloaded)
    non_actor_preserved = bool(
        non_actor_check["exact"]
        or (
            not non_actor_check["keys"]
            and not non_actor_check["missing_keys"]
            and not non_actor_check["unexpected_keys"]
            and float(non_actor_check["max_abs_diff"]) == 0.0
        )
    )
    if not reload_check["exact"] or not non_actor_preserved:
        raise RuntimeError("projected checkpoint failed state preservation audit")
    report = {
        "ok": True,
        "source_module": str(source_module),
        "candidate_module": str(candidate_module),
        "output_module": str(output_module),
        "reference_trace": str(Path(args.reference_trace).resolve()),
        "reference_samples": len(observations),
        "action_dim": int(args.action_dim),
        "max_nominal_mean_shift": float(args.max_nominal_mean_shift),
        "candidate_nominal_shift": full_shift,
        "projection_alpha": float(alpha),
        "projected_nominal_shift": projected_shift,
        "source_actor_digest": warm_start.actor_state_digest(source),
        "projected_actor_digest": warm_start.actor_state_digest(projected),
        "save_reload": reload_check,
        "non_actor_unchanged": non_actor_check,
        "non_actor_preserved": non_actor_preserved,
        "ppo_updates": 0,
        "critic_trained": False,
    }
    warm_start.write_report(output_dir / "projection_report.json", report)
    return report


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-module", required=True)
    parser.add_argument("--candidate-module", required=True)
    parser.add_argument("--reference-trace", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--max-nominal-mean-shift", type=float, default=0.005)
    parser.add_argument("--action-dim", type=int, default=2)
    return parser.parse_args()


if __name__ == "__main__":
    print(json.dumps(run(parse_args()), indent=2))
