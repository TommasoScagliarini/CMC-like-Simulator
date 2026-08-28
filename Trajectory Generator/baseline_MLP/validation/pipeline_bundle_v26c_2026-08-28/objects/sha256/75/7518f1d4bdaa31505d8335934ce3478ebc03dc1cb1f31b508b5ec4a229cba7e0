"""Teach a warm-start actor to use deployable Markov controller state.

The source actor already passes the nominal full-episode gate.  Its shared
weights stay frozen while only the newly added first-layer columns for target
slew, reference-governor, and previous SEA-command state are trained.  Nominal
states are anchored to the source actor output; states visited under stochastic
exploration are labelled with the time-aligned prescribed warm-start teacher.
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np

import target_domain_imitation as imitation
import target_domain_noise_adaptation as noise_adaptation
import warm_start


MARKOV_CONTROLLER_FEATURE_SUFFIXES: tuple[str, ...] = (
    "_previous_endpoint",
    "_served_ref",
    "_served_ref_vel",
    "_served_ref_accel",
    "_sea_u",
)

MARKOV_CONTROLLER_FEATURE_SCALES: dict[str, float] = {
    "pros_knee_angle_previous_endpoint": 1.0,
    "pros_knee_angle_served_ref": 1.0,
    "pros_knee_angle_served_ref_vel": 4.0,
    "pros_knee_angle_served_ref_accel": 60.0,
    "pros_knee_angle_sea_u": 1.0,
    "pros_ankle_angle_previous_endpoint": 1.0,
    "pros_ankle_angle_served_ref": 1.0,
    "pros_ankle_angle_served_ref_vel": 3.5,
    "pros_ankle_angle_served_ref_accel": 55.0,
    "pros_ankle_angle_sea_u": 1.0,
}


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


def _trace_arrays(
    rows: Sequence[Mapping[str, Any]],
    feature_names: Sequence[str],
) -> tuple[np.ndarray, np.ndarray]:
    observations: list[np.ndarray] = []
    means: list[np.ndarray] = []
    expected_names = tuple(str(name) for name in feature_names)
    for expected_step, row in enumerate(rows, start=1):
        if int(row.get("step", expected_step)) != expected_step:
            raise ValueError(f"trace is not contiguous at step {expected_step}")
        vector = np.asarray(
            row["actor_observation_vector_before"], dtype=np.float32
        ).reshape(-1)
        if vector.shape != (len(expected_names),):
            raise ValueError(
                f"step {expected_step} actor width {vector.shape} != "
                f"{(len(expected_names),)}"
            )
        named = row.get("actor_observation_before")
        if isinstance(named, Mapping):
            names = tuple(str(name) for name in named)
            if names != expected_names:
                raise ValueError(
                    f"step {expected_step} actor feature order does not match nominal"
                )
            named_vector = np.asarray(list(named.values()), dtype=np.float32)
            if not np.allclose(named_vector, vector, rtol=0.0, atol=1e-7):
                raise ValueError(
                    f"step {expected_step} named and vector observations differ"
                )
        action_value = row.get("policy_action_mean")
        if action_value is None:
            action_value = row.get("raw_policy_action")
        if action_value is None:
            raise ValueError(
                f"step {expected_step} contains neither a policy mean nor a raw action"
            )
        mean = np.asarray(action_value, dtype=np.float32).reshape(-1)
        if not np.all(np.isfinite(vector)) or not np.all(np.isfinite(mean)):
            raise ValueError(f"step {expected_step} contains non-finite values")
        observations.append(vector)
        means.append(mean)
    if not observations:
        raise ValueError("trace must not be empty")
    return np.asarray(observations), np.asarray(means)


def markov_feature_names(feature_names: Sequence[str]) -> tuple[str, ...]:
    selected = tuple(
        str(name)
        for name in feature_names
        if str(name).endswith(MARKOV_CONTROLLER_FEATURE_SUFFIXES)
        and not str(name).endswith(("_sea_u_abs", "_sea_u_saturated"))
    )
    if len(selected) != 10:
        raise ValueError(
            "expected 10 deployable Markov controller features, found "
            f"{len(selected)}: {selected}"
        )
    return selected


def build_markov_recovery_dataset(
    nominal_rows: Sequence[Mapping[str, Any]],
    recovery_traces: Sequence[Sequence[Mapping[str, Any]]],
    teacher_actions: np.ndarray,
    *,
    nominal_repeat: int,
    recovery_repeat: int,
) -> tuple[dict[str, np.ndarray], dict[str, Any]]:
    if nominal_repeat < 1 or recovery_repeat < 1:
        raise ValueError("nominal_repeat and recovery_repeat must be >= 1")
    first_named = nominal_rows[0].get("actor_observation_before")
    if not isinstance(first_named, Mapping):
        raise ValueError("nominal trace must contain named actor observations")
    feature_names = tuple(str(name) for name in first_named)
    markov_names = markov_feature_names(feature_names)
    nominal_obs, nominal_means = _trace_arrays(nominal_rows, feature_names)
    teacher_actions = np.asarray(teacher_actions, dtype=np.float32)
    if teacher_actions.ndim != 2 or teacher_actions.shape[1] != nominal_means.shape[1]:
        raise ValueError("teacher action shape does not match actor action shape")
    if len(teacher_actions) < len(nominal_obs):
        raise ValueError("teacher action horizon is shorter than nominal trace")

    recovery_observations: list[np.ndarray] = []
    recovery_targets: list[np.ndarray] = []
    trace_reports: list[dict[str, Any]] = []
    for trace_index, rows in enumerate(recovery_traces, start=1):
        observations, policy_means = _trace_arrays(rows, feature_names)
        if len(observations) > len(teacher_actions):
            raise ValueError(f"recovery trace {trace_index} exceeds teacher horizon")
        labels = teacher_actions[: len(observations)]
        recovery_observations.append(observations)
        recovery_targets.append(labels)
        difference = policy_means - labels
        trace_reports.append(
            {
                "trace_index": trace_index,
                "steps": len(observations),
                "policy_mean_vs_teacher_rmse": float(
                    np.sqrt(np.mean(np.square(difference)))
                ),
                "policy_mean_vs_teacher_max_abs": float(np.max(np.abs(difference))),
            }
        )
    if not recovery_observations:
        raise ValueError("at least one recovery trace is required")
    recovery_obs = np.concatenate(recovery_observations, axis=0)
    recovery_labels = np.concatenate(recovery_targets, axis=0)
    observations = np.concatenate(
        [
            np.tile(nominal_obs, (nominal_repeat, 1)),
            np.tile(recovery_obs, (recovery_repeat, 1)),
        ],
        axis=0,
    )
    actions = np.concatenate(
        [
            np.tile(nominal_means, (nominal_repeat, 1)),
            np.tile(recovery_labels, (recovery_repeat, 1)),
        ],
        axis=0,
    )
    dataset = {
        "observations": observations.astype(np.float32),
        "actions": actions.astype(np.float32),
        "times": np.arange(len(observations), dtype=np.float64),
        "actor_feature_names": np.asarray(feature_names, dtype=str),
    }
    report = {
        "actor_feature_count": len(feature_names),
        "actor_feature_names": list(feature_names),
        "trainable_markov_features": list(markov_names),
        "nominal_steps": len(nominal_obs),
        "nominal_repeat": nominal_repeat,
        "nominal_training_samples": len(nominal_obs) * nominal_repeat,
        "recovery_steps": len(recovery_obs),
        "recovery_repeat": recovery_repeat,
        "recovery_training_samples": len(recovery_obs) * recovery_repeat,
        "aggregate_samples": len(observations),
        "traces": trace_reports,
    }
    return dataset, report


def _actor_means(state: Mapping[str, Any], observations: np.ndarray) -> np.ndarray:
    logits = noise_adaptation._forward(state, np.asarray(observations, np.float32))
    return np.asarray(logits[:, :2], dtype=float)


def selected_column_update_audit(
    source_state: Mapping[str, Any],
    adapted_state: Mapping[str, Any],
    feature_names: Sequence[str],
    trainable_features: Sequence[str],
) -> dict[str, Any]:
    selected = {feature_names.index(name) for name in trainable_features}
    shared = [index for index in range(len(feature_names)) if index not in selected]
    first_layer: dict[str, Any] = {}
    shared_max_abs = 0.0
    selected_max_abs = 0.0
    for key in ("pi_encoder.0.weight", "pi.0.0.weight"):
        source = warm_start._as_numpy(source_state[key]).astype(float)
        adapted = warm_start._as_numpy(adapted_state[key]).astype(float)
        shared_diff = np.abs(adapted[:, shared] - source[:, shared])
        selected_diff = np.abs(adapted[:, sorted(selected)] - source[:, sorted(selected)])
        key_shared_max = float(np.max(shared_diff)) if shared_diff.size else 0.0
        key_selected_max = float(np.max(selected_diff)) if selected_diff.size else 0.0
        shared_max_abs = max(shared_max_abs, key_shared_max)
        selected_max_abs = max(selected_max_abs, key_selected_max)
        first_layer[key] = {
            "shared_columns_max_abs_change": key_shared_max,
            "selected_columns_max_abs_change": key_selected_max,
        }
    direct_keys = (
        "pi_encoder.0.bias",
        "pi_encoder.2.weight",
        "pi_encoder.2.bias",
        "pi.0.0.bias",
        "pi.0.2.weight",
        "pi.0.2.bias",
        "pi.1.weight",
        "pi.1.bias",
    )
    direct_changes = {}
    direct_max_abs = 0.0
    for key in direct_keys:
        source = warm_start._as_numpy(source_state[key]).astype(float)
        adapted = warm_start._as_numpy(adapted_state[key]).astype(float)
        change = float(np.max(np.abs(adapted - source))) if source.size else 0.0
        direct_changes[key] = change
        direct_max_abs = max(direct_max_abs, change)
    return {
        "first_layer": first_layer,
        "shared_columns_max_abs_change": shared_max_abs,
        "selected_columns_max_abs_change": selected_max_abs,
        "direct_actor_keys_max_abs_change": direct_max_abs,
        "direct_actor_key_changes": direct_changes,
        "only_selected_columns_changed": (
            shared_max_abs == 0.0
            and direct_max_abs == 0.0
            and selected_max_abs > 0.0
        ),
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument("--nominal-trace", required=True)
    parser.add_argument("--recovery-trace", action="append", required=True)
    parser.add_argument(
        "--recovery-dataset",
        action="append",
        default=[],
        help="Additional coherent observation/action NPZ, e.g. another start phase.",
    )
    parser.add_argument("--teacher-dataset", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--nominal-repeat", type=int, default=8)
    parser.add_argument("--recovery-repeat", type=int, default=1)
    parser.add_argument("--recovery-dataset-repeat", type=int, default=1)
    parser.add_argument("--epochs", type=int, default=300)
    parser.add_argument("--batch-size", type=int, default=128)
    parser.add_argument("--learning-rate", type=float, default=1e-4)
    parser.add_argument("--validation-fraction", type=float, default=0.2)
    parser.add_argument("--patience", type=int, default=60)
    parser.add_argument("--anchor-weight", type=float, default=1e-3)
    parser.add_argument("--max-nominal-mean-shift", type=float, default=0.005)
    parser.add_argument(
        "--stop-before-discrete-mismatch",
        action=argparse.BooleanOptionalAction,
        default=True,
        help=(
            "Discard each stochastic recovery trace from its first contact/FSM "
            "mismatch, after which fixed-step teacher labels are phase-invalid."
        ),
    )
    parser.add_argument(
        "--scale-markov-inputs",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Condition optimization with fixed physical scales for new columns.",
    )
    parser.add_argument(
        "--train-full-actor",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Fine-tune the full mean actor instead of only the new input columns.",
    )
    parser.add_argument("--seed", type=int, default=123)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    checkpoint = _path(args.checkpoint)
    nominal_trace_path = _path(args.nominal_trace)
    recovery_paths = [_path(value) for value in args.recovery_trace]
    recovery_dataset_paths = [_path(value) for value in args.recovery_dataset]
    teacher_dataset_path = _path(args.teacher_dataset)
    output_dir = _path(args.output_dir, must_exist=False)
    if output_dir.exists():
        raise FileExistsError(output_dir)
    output_dir.mkdir(parents=True)

    nominal_rows = _load_json(nominal_trace_path)
    recovery_rows = [_load_json(path) for path in recovery_paths]
    trace_filters = []
    if args.stop_before_discrete_mismatch:
        filtered_rows = []
        first_named = nominal_rows[0].get("actor_observation_before")
        if not isinstance(first_named, Mapping):
            raise ValueError("nominal trace must contain named actor observations")
        feature_names = tuple(str(name) for name in first_named)
        for rows in recovery_rows:
            filtered, filter_report = noise_adaptation.truncate_before_discrete_mismatch(
                nominal_rows, rows, feature_names
            )
            if not filtered:
                raise ValueError(
                    "recovery trace has no phase-aligned rows before its first "
                    "contact/FSM mismatch"
                )
            filtered_rows.append(filtered)
            trace_filters.append(filter_report)
        recovery_rows = filtered_rows
    with np.load(teacher_dataset_path) as archive:
        teacher_actions = np.asarray(archive["actions"], dtype=np.float32)
    dataset, dataset_report = build_markov_recovery_dataset(
        nominal_rows,
        recovery_rows,
        teacher_actions,
        nominal_repeat=args.nominal_repeat,
        recovery_repeat=args.recovery_repeat,
    )
    extra_dataset_reports = []
    for path in recovery_dataset_paths:
        with np.load(path) as archive:
            extra_observations = np.asarray(archive["observations"], np.float32)
            extra_actions = np.asarray(archive["actions"], np.float32)
            extra_names = tuple(
                str(name) for name in np.asarray(archive["actor_feature_names"]).tolist()
            )
        expected_names = tuple(dataset_report["actor_feature_names"])
        if extra_names != expected_names:
            raise ValueError(f"recovery dataset actor schema mismatch: {path}")
        if (
            extra_observations.ndim != 2
            or extra_observations.shape[1] != len(expected_names)
            or extra_actions.ndim != 2
            or len(extra_observations) != len(extra_actions)
        ):
            raise ValueError(f"invalid recovery dataset shapes: {path}")
        repeated_observations = np.tile(
            extra_observations, (args.recovery_dataset_repeat, 1)
        )
        repeated_actions = np.tile(
            extra_actions, (args.recovery_dataset_repeat, 1)
        )
        dataset["observations"] = np.concatenate(
            [dataset["observations"], repeated_observations], axis=0
        )
        dataset["actions"] = np.concatenate(
            [dataset["actions"], repeated_actions], axis=0
        )
        dataset["times"] = np.arange(len(dataset["observations"]), dtype=np.float64)
        extra_dataset_reports.append(
            {
                "path": str(path),
                "samples": len(extra_observations),
                "training_samples": len(repeated_observations),
            }
        )
    dataset_report["additional_recovery_datasets"] = extra_dataset_reports
    dataset_report["recovery_trace_filters"] = trace_filters
    dataset_report["recovery_dataset_repeat"] = args.recovery_dataset_repeat
    dataset_report["aggregate_samples"] = len(dataset["observations"])
    np.savez_compressed(output_dir / "markov_recovery_dataset.npz", **dataset)
    (output_dir / "markov_dataset_report.json").write_text(
        json.dumps(
            {
                **dataset_report,
                "nominal_trace": str(nominal_trace_path),
                "recovery_traces": [str(path) for path in recovery_paths],
                "recovery_datasets": [
                    str(path) for path in recovery_dataset_paths
                ],
                "teacher_dataset": str(teacher_dataset_path),
            },
            indent=2,
        ),
        encoding="utf-8",
    )

    source_state = warm_start.load_module_state(checkpoint)
    nominal_observations, _ = _trace_arrays(
        nominal_rows, dataset_report["actor_feature_names"]
    )
    source_nominal_means = _actor_means(source_state, nominal_observations)
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
        clip_weight=1.0,
        logstd_weight=0.0,
        anchor_weight=args.anchor_weight,
        freeze_logstd_head=True,
        trainable_first_layer_features=(
            None
            if args.train_full_actor
            else dataset_report["trainable_markov_features"]
        ),
        first_layer_feature_scales=(
            MARKOV_CONTROLLER_FEATURE_SCALES if args.scale_markov_inputs else None
        ),
    )
    adapted_module = output_dir / "rl_module_target_adapted"
    adapted_state = warm_start.load_module_state(adapted_module)
    adapted_nominal_means = _actor_means(adapted_state, nominal_observations)
    nominal_shift = adapted_nominal_means - source_nominal_means
    nominal_shift_report = {
        "rms": float(np.sqrt(np.mean(np.square(nominal_shift)))),
        "max_abs": float(np.max(np.abs(nominal_shift))),
        "per_action_rms": np.sqrt(np.mean(np.square(nominal_shift), axis=0)).tolist(),
    }
    first_weight = warm_start._as_numpy(adapted_state["pi_encoder.0.weight"])
    feature_names = dataset_report["actor_feature_names"]
    learned_column_norms = {
        name: float(np.linalg.norm(first_weight[:, feature_names.index(name)]))
        for name in dataset_report["trainable_markov_features"]
    }
    update_audit = selected_column_update_audit(
        source_state,
        adapted_state,
        feature_names,
        dataset_report["trainable_markov_features"],
    )
    train_scope_matches = (
        adaptation["hyperparameters"]["trainable_first_layer_features"] is None
        if args.train_full_actor
        else adaptation["hyperparameters"]["trainable_first_layer_features"]
        == dataset_report["trainable_markov_features"]
    )
    update_scope_matches = (
        (
            update_audit["shared_columns_max_abs_change"] > 0.0
            and update_audit["selected_columns_max_abs_change"] > 0.0
            and update_audit["direct_actor_keys_max_abs_change"] > 0.0
        )
        if args.train_full_actor
        else update_audit["only_selected_columns_changed"]
    )
    gate = {
        "nominal_mean_shift_within_limit": (
            nominal_shift_report["max_abs"] <= args.max_nominal_mean_shift
        ),
        "new_columns_learned": any(value > 0.0 for value in learned_column_norms.values()),
        "logstd_head_unchanged": (
            adaptation["logstd_head_max_abs_parameter_change"] == 0.0
        ),
        "requested_actor_train_scope_applied": train_scope_matches,
        "requested_actor_update_scope_observed": update_scope_matches,
    }
    actor_digest = warm_start.actor_state_digest(adapted_state)
    manifest = {
        "schema_version": 1,
        "actor_feature_names": feature_names,
        "actor_feature_count": len(feature_names),
        "actor_digest": actor_digest,
        "contract": "deployable_markov_controller_state",
    }
    (output_dir / warm_start.DEFAULT_ACTOR_FEATURE_MANIFEST_NAME).write_text(
        json.dumps(manifest, indent=2), encoding="utf-8"
    )
    source_config = checkpoint.parent / "training_cfg.resolved.yaml"
    if source_config.is_file():
        shutil.copy2(source_config, output_dir / source_config.name)
    summary = {
        "ok": all(gate.values()),
        "stage": "complete" if all(gate.values()) else "adaptation_gate",
        "source_checkpoint": str(checkpoint),
        "output_module": str(adapted_module),
        "actor_digest": actor_digest,
        "dataset": dataset_report,
        "adaptation": adaptation,
        "nominal_mean_shift": nominal_shift_report,
        "max_nominal_mean_shift": args.max_nominal_mean_shift,
        "learned_column_norms": learned_column_norms,
        "selected_column_update_audit": update_audit,
        "train_full_actor": bool(args.train_full_actor),
        "scale_markov_inputs": bool(args.scale_markov_inputs),
        "gate": gate,
        "gate_pass": all(gate.values()),
    }
    (output_dir / "run_summary.json").write_text(
        json.dumps(summary, indent=2), encoding="utf-8"
    )
    print(json.dumps(summary, indent=2))
    return 0 if all(gate.values()) else 2


if __name__ == "__main__":
    raise SystemExit(main())
