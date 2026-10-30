"""Robustify a target-domain actor around a validated nominal rollout.

Disturbed on-policy observations are labelled with the deterministic actor
action at the same fixed-start step. This recovery-distillation objective
preserves the successful nominal policy while teaching it how to respond to
nearby states reached under small Gaussian exploration noise.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np

import target_domain_imitation as imitation
import warm_start


def _path(value: str | Path, *, must_exist: bool = True) -> Path:
    text = os.fspath(value)
    if os.name != "nt":
        text = text.replace("\\", "/")
    path = Path(text).expanduser()
    if not path.is_absolute():
        path = Path.cwd() / path
    path = path.resolve()
    if must_exist and not path.exists():
        raise FileNotFoundError(path)
    return path


def _load_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _prediction_metrics(prediction: np.ndarray, target: np.ndarray) -> dict[str, Any]:
    error = np.asarray(prediction, dtype=float) - np.asarray(target, dtype=float)
    return {
        "rmse": float(np.sqrt(np.mean(np.square(error)))),
        "max_abs_error": float(np.max(np.abs(error))),
        "per_action_rmse": np.sqrt(np.mean(np.square(error), axis=0))
        .astype(float)
        .tolist(),
    }


def _discrete_feature_indices(feature_names: Sequence[str]) -> np.ndarray:
    return np.asarray(
        [
            index
            for index, name in enumerate(feature_names)
            if name.endswith(
                ("_in_contact", "_heel_strike", "_toe_off", "_saturated")
            )
            or name.startswith(("phase_fsm_", "phase_expected_"))
        ],
        dtype=int,
    )


def truncate_before_discrete_mismatch(
    nominal_rows: Sequence[Mapping[str, Any]],
    disturbed_rows: Sequence[Mapping[str, Any]],
    feature_names: Sequence[str],
) -> tuple[list[Mapping[str, Any]], dict[str, Any]]:
    """Keep only time-aligned rows whose deployable event/FSM state still agrees."""
    discrete_indices = _discrete_feature_indices(feature_names)
    limit = min(len(nominal_rows), len(disturbed_rows))
    first_mismatch_step = None
    for index in range(limit):
        nominal = np.asarray(
            nominal_rows[index]["actor_observation_vector_before"], dtype=float
        )
        disturbed = np.asarray(
            disturbed_rows[index]["actor_observation_vector_before"], dtype=float
        )
        if discrete_indices.size and np.any(
            nominal[discrete_indices] != disturbed[discrete_indices]
        ):
            first_mismatch_step = index + 1
            limit = index
            break
    return list(disturbed_rows[:limit]), {
        "original_steps": len(disturbed_rows),
        "retained_steps": limit,
        "first_discrete_mismatch_step": first_mismatch_step,
    }


def _trace_arrays(
    rows: Sequence[Mapping[str, Any]],
    *,
    feature_count: int,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    observations: list[np.ndarray] = []
    actions: list[np.ndarray] = []
    times: list[float] = []
    for expected_step, row in enumerate(rows, start=1):
        step = int(row.get("step", expected_step))
        if step != expected_step:
            raise ValueError(f"trace step sequence is not contiguous at {step}")
        observation = np.asarray(
            row["actor_observation_vector_before"], dtype=np.float32
        ).reshape(-1)
        if observation.shape != (feature_count,):
            raise ValueError(
                f"trace step {step} observation width {observation.shape} "
                f"!= {(feature_count,)}"
            )
        action = np.asarray(row["raw_policy_action"], dtype=np.float32).reshape(-1)
        if not np.all(np.isfinite(observation)) or not np.all(np.isfinite(action)):
            raise ValueError(f"trace step {step} contains non-finite values")
        observations.append(observation)
        actions.append(action)
        times.append(float(row.get("time", step)))
    if not observations:
        raise ValueError("trace must not be empty")
    return (
        np.asarray(observations, dtype=np.float32),
        np.asarray(actions, dtype=np.float32),
        np.asarray(times, dtype=np.float64),
    )


def build_recovery_dataset(
    nominal_rows: Sequence[Mapping[str, Any]],
    disturbed_traces: Sequence[Sequence[Mapping[str, Any]]],
    feature_names: Sequence[str],
    *,
    nominal_repeat: int,
    trace_repeat: int,
    interpolation_steps: int,
) -> tuple[dict[str, np.ndarray], dict[str, Any]]:
    if nominal_repeat < 1:
        raise ValueError("nominal_repeat must be >= 1")
    if trace_repeat < 1:
        raise ValueError("trace_repeat must be >= 1")
    if interpolation_steps < 0:
        raise ValueError("interpolation_steps must be >= 0")
    feature_names = tuple(str(name) for name in feature_names)
    nominal_obs, nominal_actions, nominal_times = _trace_arrays(
        nominal_rows, feature_count=len(feature_names)
    )
    discrete_indices = _discrete_feature_indices(feature_names)

    recovery_observations: list[np.ndarray] = []
    recovery_actions: list[np.ndarray] = []
    recovery_times: list[float] = []
    trace_summaries: list[dict[str, Any]] = []
    for trace_index, rows in enumerate(disturbed_traces, start=1):
        observations, sampled_actions, _ = _trace_arrays(
            rows, feature_count=len(feature_names)
        )
        if len(observations) > len(nominal_obs):
            raise ValueError(
                f"disturbed trace {trace_index} exceeds nominal horizon "
                f"({len(observations)} > {len(nominal_obs)})"
            )
        labels = nominal_actions[: len(observations)]
        for index, observation in enumerate(observations):
            local_observations = [observation]
            for interpolation_index in range(1, interpolation_steps + 1):
                alpha = interpolation_index / float(interpolation_steps + 1)
                interpolated = nominal_obs[index] + alpha * (
                    observation - nominal_obs[index]
                )
                if discrete_indices.size:
                    interpolated[discrete_indices] = observation[discrete_indices]
                local_observations.append(interpolated.astype(np.float32))
            for local_observation in local_observations:
                recovery_observations.append(local_observation)
                recovery_actions.append(labels[index].copy())
                recovery_times.append(float(nominal_times[index]))
        trace_summaries.append(
            {
                "trace_index": trace_index,
                "steps": len(observations),
                "sampled_action_vs_nominal": _prediction_metrics(
                    sampled_actions, labels
                ),
                "observation_abs_diff_max": float(
                    np.max(np.abs(observations - nominal_obs[: len(observations)]))
                ),
            }
        )

    if not recovery_observations:
        raise ValueError("at least one disturbed trace is required")
    recovery_obs = np.asarray(recovery_observations, dtype=np.float32)
    recovery_actions_array = np.asarray(recovery_actions, dtype=np.float32)
    recovery_times_array = np.asarray(recovery_times, dtype=np.float64)
    dataset = {
        "observations": np.concatenate(
            [
                np.tile(nominal_obs, (nominal_repeat, 1)),
                np.tile(recovery_obs, (trace_repeat, 1)),
            ],
            axis=0,
        ),
        "actions": np.concatenate(
            [
                np.tile(nominal_actions, (nominal_repeat, 1)),
                np.tile(recovery_actions_array, (trace_repeat, 1)),
            ],
            axis=0,
        ),
        "times": np.concatenate(
            [
                np.tile(nominal_times, nominal_repeat),
                np.tile(recovery_times_array, trace_repeat),
            ],
            axis=0,
        ),
        "actor_feature_names": np.asarray(feature_names, dtype=str),
    }
    report = {
        "nominal_samples": len(nominal_obs),
        "nominal_repeat": nominal_repeat,
        "nominal_training_samples": len(nominal_obs) * nominal_repeat,
        "disturbed_trace_samples": sum(len(rows) for rows in disturbed_traces),
        "interpolation_steps": interpolation_steps,
        "unique_recovery_samples": len(recovery_obs),
        "trace_repeat": trace_repeat,
        "recovery_training_samples": len(recovery_obs) * trace_repeat,
        "aggregate_samples": len(dataset["observations"]),
        "traces": trace_summaries,
    }
    return dataset, report


def _forward(state: Mapping[str, Any], observations: np.ndarray) -> np.ndarray:
    def array(value: Any) -> np.ndarray:
        if hasattr(value, "detach"):
            value = value.detach()
        if hasattr(value, "cpu"):
            value = value.cpu()
        return np.asarray(value)

    hidden = np.tanh(
        observations @ array(state["pi.0.0.weight"]).T
        + array(state["pi.0.0.bias"])
    )
    hidden = np.tanh(
        hidden @ array(state["pi.0.2.weight"]).T
        + array(state["pi.0.2.bias"])
    )
    return hidden @ array(state["pi.1.weight"]).T + array(state["pi.1.bias"])


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument("--nominal-trace", required=True)
    parser.add_argument("--disturbed-trace", action="append", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--nominal-repeat", type=int, default=4)
    parser.add_argument("--trace-repeat", type=int, default=1)
    parser.add_argument("--interpolation-steps", type=int, default=0)
    parser.add_argument("--epochs", type=int, default=250)
    parser.add_argument("--batch-size", type=int, default=64)
    parser.add_argument("--learning-rate", type=float, default=5e-5)
    parser.add_argument("--validation-fraction", type=float, default=0.20)
    parser.add_argument("--patience", type=int, default=40)
    parser.add_argument("--clip-weight", type=float, default=1.0)
    parser.add_argument("--anchor-weight", type=float, default=1e-3)
    parser.add_argument(
        "--stop-before-discrete-mismatch",
        action=argparse.BooleanOptionalAction,
        default=True,
        help=(
            "Discard a disturbed trace from its first contact/event/FSM mismatch. "
            "After that point, fixed-step nominal labels are no longer phase aligned."
        ),
    )
    parser.add_argument(
        "--max-nominal-mean-shift",
        type=float,
        default=0.005,
        help="Reject an adapted actor whose nominal max action-mean shift exceeds this.",
    )
    parser.add_argument("--seed", type=int, default=123)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    checkpoint = _path(args.checkpoint)
    nominal_trace_path = _path(args.nominal_trace)
    disturbed_paths = [_path(value) for value in args.disturbed_trace]
    output_dir = _path(args.output_dir, must_exist=False)
    if output_dir.exists():
        raise FileExistsError(f"output already exists: {output_dir}")

    source_root = checkpoint.parent
    source_manifest_path = source_root / warm_start.DEFAULT_ACTOR_FEATURE_MANIFEST_NAME
    source_config = source_root / "training_cfg.resolved.yaml"
    if not source_manifest_path.is_file() or not source_config.is_file():
        raise FileNotFoundError(
            "checkpoint must have adjacent actor feature manifest and resolved config"
        )
    source_manifest = _load_json(source_manifest_path)
    feature_names = tuple(str(name) for name in source_manifest["actor_feature_names"])
    nominal_rows = _load_json(nominal_trace_path)
    disturbed_traces = [_load_json(path) for path in disturbed_paths]
    trace_filters = []
    if args.stop_before_discrete_mismatch:
        filtered_traces = []
        for rows in disturbed_traces:
            filtered, filter_report = truncate_before_discrete_mismatch(
                nominal_rows, rows, feature_names
            )
            if not filtered:
                raise ValueError(
                    "disturbed trace has no phase-aligned rows before its first "
                    "discrete mismatch"
                )
            filtered_traces.append(filtered)
            trace_filters.append(filter_report)
        disturbed_traces = filtered_traces
    dataset, dataset_report = build_recovery_dataset(
        nominal_rows,
        disturbed_traces,
        feature_names,
        nominal_repeat=args.nominal_repeat,
        trace_repeat=args.trace_repeat,
        interpolation_steps=args.interpolation_steps,
    )

    output_dir.mkdir(parents=True)
    np.savez_compressed(output_dir / "recovery_dataset.npz", **dataset)
    (output_dir / "recovery_dataset_report.json").write_text(
        json.dumps(
            {
                **dataset_report,
                "nominal_trace": str(nominal_trace_path),
                "disturbed_traces": [str(path) for path in disturbed_paths],
                "trace_filters": trace_filters,
            },
            indent=2,
        ),
        encoding="utf-8",
    )
    shutil.copy2(source_config, output_dir / source_config.name)

    source_state = warm_start.load_module_state(checkpoint)
    adaptation = imitation.adapt_actor(
        checkpoint,
        dataset,
        output_dir,
        seed=args.seed,
        epochs=args.epochs,
        batch_size=args.batch_size,
        learning_rate=args.learning_rate,
        validation_fraction=args.validation_fraction,
        patience=args.patience,
        clip_weight=args.clip_weight,
        logstd_weight=0.0,
        anchor_weight=args.anchor_weight,
        freeze_logstd_head=True,
    )
    output_module = Path(adaptation["output_module"])
    adapted_state = warm_start.load_module_state(output_module)
    nominal_obs, nominal_actions, _ = _trace_arrays(
        nominal_rows, feature_count=len(feature_names)
    )
    action_dim = nominal_actions.shape[1]
    source_logits = _forward(source_state, nominal_obs)
    adapted_logits = _forward(adapted_state, nominal_obs)
    nominal_validation = {
        "source_vs_recorded": _prediction_metrics(
            source_logits[:, :action_dim], nominal_actions
        ),
        "adapted_vs_recorded": _prediction_metrics(
            adapted_logits[:, :action_dim], nominal_actions
        ),
        "adapted_vs_source_mean": _prediction_metrics(
            adapted_logits[:, :action_dim], source_logits[:, :action_dim]
        ),
        "source_logstd_min": np.min(source_logits[:, action_dim:], axis=0)
        .astype(float)
        .tolist(),
        "source_logstd_max": np.max(source_logits[:, action_dim:], axis=0)
        .astype(float)
        .tolist(),
        "adapted_logstd_min": np.min(adapted_logits[:, action_dim:], axis=0)
        .astype(float)
        .tolist(),
        "adapted_logstd_max": np.max(adapted_logits[:, action_dim:], axis=0)
        .astype(float)
        .tolist(),
    }
    nominal_shift = float(
        nominal_validation["adapted_vs_source_mean"]["max_abs_error"]
    )
    nominal_gate_pass = nominal_shift <= float(args.max_nominal_mean_shift)

    manifest = dict(source_manifest)
    manifest.update(
        {
            "actor_digest": adaptation["adapted_actor_digest"],
            "source_actor_digest": adaptation["source_actor_digest"],
            "module_state_sha256": _sha256(output_module / "module_state.pkl"),
            "adaptation": "target_domain_noise_recovery",
        }
    )
    warm_start.write_report(
        output_dir / warm_start.DEFAULT_ACTOR_FEATURE_MANIFEST_NAME,
        manifest,
    )
    summary = {
        "ok": bool(nominal_gate_pass),
        "stage": "complete" if nominal_gate_pass else "nominal_preservation_gate",
        "checkpoint": str(checkpoint),
        "output_dir": str(output_dir),
        "dataset": dataset_report,
        "adaptation": adaptation,
        "nominal_validation": nominal_validation,
        "nominal_preservation_gate": {
            "max_allowed_shift": float(args.max_nominal_mean_shift),
            "observed_max_shift": nominal_shift,
            "pass": bool(nominal_gate_pass),
        },
        "ppo_updates": 0,
        "critic_trained": False,
    }
    (output_dir / "run_summary.json").write_text(
        json.dumps(summary, indent=2), encoding="utf-8"
    )
    print(json.dumps(summary, indent=2))
    return 0 if nominal_gate_pass else 2


if __name__ == "__main__":
    raise SystemExit(main())
