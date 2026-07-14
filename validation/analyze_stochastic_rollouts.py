"""Recover and summarize Gaussian exploration noise from rollout traces."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np

try:
    from . import compare_policy_checkpoints
except ImportError:  # Direct script execution adds validation/ to sys.path.
    import compare_policy_checkpoints


def _noise_metrics(
    sampled_actions: np.ndarray,
    action_means: np.ndarray,
    action_sigmas: np.ndarray,
) -> dict[str, Any]:
    noise = sampled_actions - action_means
    standardized = noise / action_sigmas
    return {
        "samples": int(noise.shape[0]),
        "realized_noise_rms": float(np.sqrt(np.mean(np.square(noise)))),
        "realized_noise_rms_per_action": np.sqrt(
            np.mean(np.square(noise), axis=0)
        ).tolist(),
        "realized_noise_mean_per_action": np.mean(noise, axis=0).tolist(),
        "policy_sigma_mean_per_action": np.mean(action_sigmas, axis=0).tolist(),
        "standardized_noise_rms_per_action": np.sqrt(
            np.mean(np.square(standardized), axis=0)
        ).tolist(),
    }


def _analyze_run(
    state: Mapping[str, Any],
    run_dir: Path,
) -> tuple[dict[str, Any], np.ndarray, np.ndarray, np.ndarray]:
    trace_path = run_dir / "rollout_policy_trace.json"
    summary_path = run_dir / "rollout_summary.json"
    rows = json.loads(trace_path.read_text(encoding="utf-8"))
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    observations = np.asarray(
        [row["actor_observation_vector_before"] for row in rows],
        dtype=np.float64,
    )
    sampled_actions = np.asarray(
        [row["raw_policy_action"] for row in rows],
        dtype=np.float64,
    )
    action_dim = int(sampled_actions.shape[1])
    logits = compare_policy_checkpoints._actor_logits(state, observations)
    action_means = logits[:, :action_dim]
    action_sigmas = np.exp(logits[:, action_dim:])
    metrics = _noise_metrics(sampled_actions, action_means, action_sigmas)
    metrics.update(
        {
            "run_dir": str(run_dir),
            "seed": int(summary["action_seed"]),
            "steps": int(summary["steps"]),
            "episode_return": float(summary["episode_return"]),
            "valid_cycles": int(summary["phase_valid_cycle_count"]),
            "grf_penetration_max_m": float(summary["grf_penetration_max_m"]),
            "action_clipped_fraction": float(summary["action_clipped_fraction"]),
            "end_reason": summary["end_reason"],
            "valid_episode": bool(
                int(summary["steps"]) == 500
                and int(summary["phase_valid_cycle_count"]) >= 1
                and float(summary["grf_penetration_max_m"]) < 0.025
                and float(summary["action_clipped_fraction"]) == 0.0
                and summary["end_reason"] == "episode_time_limit"
            ),
        }
    )
    return metrics, sampled_actions, action_means, action_sigmas


def analyze(module_path: str | Path, run_dirs: Sequence[str | Path]) -> dict[str, Any]:
    state = compare_policy_checkpoints._load_state(module_path)
    per_run: list[dict[str, Any]] = []
    sampled: list[np.ndarray] = []
    means: list[np.ndarray] = []
    sigmas: list[np.ndarray] = []
    for value in run_dirs:
        run_dir = Path(value).expanduser().resolve()
        metrics, run_sampled, run_means, run_sigmas = _analyze_run(state, run_dir)
        per_run.append(metrics)
        sampled.append(run_sampled)
        means.append(run_means)
        sigmas.append(run_sigmas)

    returns = np.asarray([row["episode_return"] for row in per_run], dtype=float)
    aggregate_noise = _noise_metrics(
        np.concatenate(sampled),
        np.concatenate(means),
        np.concatenate(sigmas),
    )
    return {
        "module": str(Path(module_path).expanduser().resolve()),
        "runs": per_run,
        "aggregate": {
            "run_count": len(per_run),
            "valid_run_count": sum(bool(row["valid_episode"]) for row in per_run),
            "episode_return_mean": float(np.mean(returns)),
            "episode_return_std": float(np.std(returns)),
            "episode_return_min": float(np.min(returns)),
            "episode_return_max": float(np.max(returns)),
            **aggregate_noise,
        },
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--module", required=True)
    parser.add_argument("--run", action="append", required=True)
    parser.add_argument("--output", required=True)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    result = analyze(args.module, args.run)
    output = Path(args.output).expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
