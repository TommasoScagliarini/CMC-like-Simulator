"""Compare two MLP actor checkpoints on fixed rollout observations."""

from __future__ import annotations

import argparse
import hashlib
import json
import pickle
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np


_CANONICAL_ACTOR_KEYS = (
    "pi.0.0.weight",
    "pi.0.0.bias",
    "pi.0.2.weight",
    "pi.0.2.bias",
    "pi.1.weight",
    "pi.1.bias",
)


def _array(value: Any) -> np.ndarray:
    if hasattr(value, "detach"):
        value = value.detach()
    if hasattr(value, "cpu"):
        value = value.cpu()
    return np.asarray(value)


def _load_state(path: str | Path) -> Mapping[str, Any]:
    resolved = Path(path).expanduser().resolve()
    state_path = resolved / "module_state.pkl" if resolved.is_dir() else resolved
    with state_path.open("rb") as handle:
        state = pickle.load(handle)
    if not isinstance(state, Mapping):
        raise ValueError(f"module state is not a mapping: {state_path}")
    missing = [key for key in _CANONICAL_ACTOR_KEYS if key not in state]
    if missing:
        raise ValueError(f"actor keys missing from {state_path}: {missing}")
    return state


def _load_trace(path: str | Path) -> list[Mapping[str, Any]]:
    resolved = Path(path).expanduser().resolve()
    rows = json.loads(resolved.read_text(encoding="utf-8"))
    if not isinstance(rows, list) or not rows:
        raise ValueError(f"trace must be a non-empty JSON list: {resolved}")
    return rows


def _actor_logits(state: Mapping[str, Any], observations: np.ndarray) -> np.ndarray:
    w1, b1, w2, b2, w3, b3 = (
        _array(state[key]).astype(np.float64, copy=False)
        for key in _CANONICAL_ACTOR_KEYS
    )
    h1 = np.tanh(observations @ w1.T + b1)
    h2 = np.tanh(h1 @ w2.T + b2)
    return h2 @ w3.T + b3


def _actor_digest(state: Mapping[str, Any]) -> str:
    digest = hashlib.sha256()
    for key in sorted(key for key in state if str(key).startswith("pi")):
        value = np.ascontiguousarray(_array(state[key]))
        digest.update(str(key).encode("utf-8"))
        digest.update(str(value.dtype).encode("ascii"))
        digest.update(np.asarray(value.shape, dtype=np.int64).tobytes())
        digest.update(value.tobytes())
    return digest.hexdigest()


def _parameter_comparison(
    reference: Mapping[str, Any], candidate: Mapping[str, Any]
) -> dict[str, Any]:
    rows: dict[str, Any] = {}
    squared_sum = 0.0
    element_count = 0
    max_abs = 0.0
    for key in _CANONICAL_ACTOR_KEYS:
        ref = _array(reference[key]).astype(np.float64, copy=False)
        cur = _array(candidate[key]).astype(np.float64, copy=False)
        if ref.shape != cur.shape:
            raise ValueError(f"shape mismatch for {key}: {ref.shape} != {cur.shape}")
        delta = cur - ref
        key_max = float(np.max(np.abs(delta)))
        key_rms = float(np.sqrt(np.mean(np.square(delta))))
        rows[key] = {"max_abs_diff": key_max, "rms_diff": key_rms}
        max_abs = max(max_abs, key_max)
        squared_sum += float(np.sum(np.square(delta)))
        element_count += int(delta.size)
    return {
        "exact": bool(max_abs == 0.0),
        "max_abs_diff": max_abs,
        "rms_diff": float(np.sqrt(squared_sum / max(element_count, 1))),
        "per_key": rows,
    }


def _fixed_observation_metrics(
    reference: Mapping[str, Any],
    candidate: Mapping[str, Any],
    observations: np.ndarray,
    action_dim: int,
) -> dict[str, Any]:
    ref_logits = _actor_logits(reference, observations)
    cur_logits = _actor_logits(candidate, observations)
    if ref_logits.shape[1] != 2 * action_dim:
        raise ValueError(
            f"expected {2 * action_dim} Gaussian logits, got {ref_logits.shape[1]}"
        )

    ref_mean = ref_logits[:, :action_dim]
    cur_mean = cur_logits[:, :action_dim]
    ref_logstd = ref_logits[:, action_dim:]
    cur_logstd = cur_logits[:, action_dim:]
    mean_delta = cur_mean - ref_mean
    logstd_delta = cur_logstd - ref_logstd
    ref_var = np.exp(2.0 * ref_logstd)
    cur_var = np.exp(2.0 * cur_logstd)
    kl_rows = np.sum(
        cur_logstd
        - ref_logstd
        + (ref_var + np.square(ref_mean - cur_mean)) / (2.0 * cur_var)
        - 0.5,
        axis=1,
    )
    return {
        "samples": int(observations.shape[0]),
        "mean_delta_rmse": float(np.sqrt(np.mean(np.square(mean_delta)))),
        "mean_delta_rmse_per_action": np.sqrt(
            np.mean(np.square(mean_delta), axis=0)
        ).tolist(),
        "mean_delta_abs_max": float(np.max(np.abs(mean_delta))),
        "mean_delta_signed_mean_per_action": np.mean(mean_delta, axis=0).tolist(),
        "logstd_delta_abs_max": float(np.max(np.abs(logstd_delta))),
        "reference_sigma_mean": np.mean(np.exp(ref_logstd), axis=0).tolist(),
        "candidate_sigma_mean": np.mean(np.exp(cur_logstd), axis=0).tolist(),
        "empirical_kl_reference_to_candidate_mean": float(np.mean(kl_rows)),
        "empirical_kl_reference_to_candidate_max": float(np.max(kl_rows)),
    }


def compare(
    reference_path: str | Path,
    candidate_path: str | Path,
    trace_paths: Sequence[str | Path],
) -> dict[str, Any]:
    reference = _load_state(reference_path)
    candidate = _load_state(candidate_path)
    trace_rows = [_load_trace(path) for path in trace_paths]
    action_dims = {
        len(row[0]["raw_policy_action"])
        for row in trace_rows
    }
    if len(action_dims) != 1:
        raise ValueError(f"inconsistent action dimensions across traces: {action_dims}")
    action_dim = action_dims.pop()

    per_trace: list[dict[str, Any]] = []
    all_observations: list[np.ndarray] = []
    for path, rows in zip(trace_paths, trace_rows):
        observations = np.asarray(
            [row["actor_observation_vector_before"] for row in rows],
            dtype=np.float64,
        )
        all_observations.append(observations)
        per_trace.append(
            {
                "trace": str(Path(path).expanduser().resolve()),
                **_fixed_observation_metrics(
                    reference, candidate, observations, action_dim
                ),
            }
        )

    aggregate = _fixed_observation_metrics(
        reference,
        candidate,
        np.concatenate(all_observations, axis=0),
        action_dim,
    )
    return {
        "reference_checkpoint": str(Path(reference_path).expanduser().resolve()),
        "candidate_checkpoint": str(Path(candidate_path).expanduser().resolve()),
        "reference_actor_digest": _actor_digest(reference),
        "candidate_actor_digest": _actor_digest(candidate),
        "parameter_comparison": _parameter_comparison(reference, candidate),
        "fixed_observation_aggregate": aggregate,
        "fixed_observation_per_trace": per_trace,
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--reference", required=True)
    parser.add_argument("--candidate", required=True)
    parser.add_argument("--trace", action="append", required=True)
    parser.add_argument("--output", required=True)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    result = compare(args.reference, args.candidate, args.trace)
    output = Path(args.output).expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
