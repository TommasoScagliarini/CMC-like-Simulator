"""Offline, episode-held-out diagnostics of the original 35 actor features.

Only the standard library is imported at module load; execution additionally
requires NumPy, never OpenSim, Ray or a training environment. The time oracle
is an offline diagnostic with a nominal 1.58 s period, NOT measured gait phase.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
from typing import Any


ACTOR_FEATURES = (
    "gait_phase_sin", "gait_phase_cos", "pros_knee_angle", "pros_knee_angle_vel",
    "pros_ankle_angle", "pros_ankle_angle_vel", "SEA_Knee_motor_angle",
    "SEA_Knee_motor_speed", "SEA_Ankle_motor_angle", "SEA_Ankle_motor_speed",
    "online_left_normal_grf_bw", "online_left_in_contact", "online_left_heel_strike",
    "online_left_toe_off", "online_left_gait_phase_sin", "online_left_gait_phase_cos",
    "online_left_cycle_duration_s", "phase_fsm_wait_hs", "phase_fsm_stance_after_hs",
    "phase_fsm_swing_after_to", "phase_expected_hs", "phase_expected_to",
    "phase_stance_elapsed_norm", "phase_swing_elapsed_norm", "phase_cycle_progress_credit",
    "pros_knee_angle_previous_endpoint", "pros_knee_angle_served_ref",
    "pros_knee_angle_served_ref_vel", "pros_knee_angle_served_ref_accel",
    "pros_knee_angle_sea_u", "pros_ankle_angle_previous_endpoint",
    "pros_ankle_angle_served_ref", "pros_ankle_angle_served_ref_vel",
    "pros_ankle_angle_served_ref_accel", "pros_ankle_angle_sea_u",
)
TARGET_NAMES = ("knee_q", "knee_qdot", "ankle_q", "ankle_qdot")
TARGET_SCALES = (1.0, 10.0, 1.0, 10.0)
HISTORY_SAMPLES = 5
RIDGE_ALPHA = 1.0
ORACLE_PERIOD_S = 1.58
TIME_TOL = 1e-7
MIN_TRAIN_SAMPLES = 10
MIN_TEST_SAMPLES = 5
MODELS = ("instant35", "history175", "instant35_time_oracle38")


def _finite(value: Any, field: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"Invalid numeric {field}: {value!r}") from exc
    if not math.isfinite(result):
        raise ValueError(f"Nonfinite {field}: {value!r}")
    return result


def _hash(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load_episode(path: str | Path) -> dict:
    """Read one rollout episode; malformed data fail, short episodes are allowed."""
    directory = Path(path).expanduser().resolve()
    if directory.is_file():
        if directory.name not in {"rollout_policy_trace.json", "rollout_summary.json"}:
            raise ValueError(f"Expected rollout directory, trace or summary: {directory}")
        directory = directory.parent
    summary_path = directory / "rollout_summary.json"
    trace_path = directory / "rollout_policy_trace.json"
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    rows = json.loads(trace_path.read_text(encoding="utf-8"))
    if summary.get("ok") is not True or not isinstance(rows, list):
        raise ValueError(f"Unsuccessful rollout or invalid trace: {directory}")
    if summary.get("steps") != len(rows):
        raise ValueError(f"Summary/trace step count mismatch: {directory}")
    schema = summary.get("actor_feature_names", [])
    if len(schema) not in (35, 39) or tuple(schema[:35]) != ACTOR_FEATURES:
        raise ValueError(f"Original ordered 35-feature actor prefix required: {directory}")
    if len(set(schema)) != len(schema):
        raise ValueError(f"Duplicate actor feature names: {directory}")
    offset = _finite(summary.get("episode_start_offset_s"), "episode start offset")
    prepared = []
    for index, row in enumerate(rows):
        if row.get("step") != index + 1:
            raise ValueError(f"Nonsequential step at row {index}: {directory}")
        segment = row.get("policy_segment_times", [])
        if not isinstance(segment, list) or len(segment) < 2:
            raise ValueError(f"Missing policy segment timestamps at row {index}")
        segment = [_finite(t, "policy segment timestamp") for t in segment]
        if any(b <= a for a, b in zip(segment, segment[1:])):
            raise ValueError(f"Nonpositive policy timestamp interval at row {index}")
        pre, post = segment[0], _finite(row.get("time"), "post-step timestamp")
        if abs(post - segment[-1]) > TIME_TOL:
            raise ValueError(f"Post-step/segment timestamp mismatch at row {index}")
        if prepared and abs(pre - prepared[-1]["post_time_s"]) > TIME_TOL:
            raise ValueError(f"Discontinuous timestamps/reset inside episode at row {index}")
        obs_map = row.get("actor_observation_before", {})
        try:
            observations = [_finite(obs_map[name], name) for name in ACTOR_FEATURES]
        except KeyError as exc:
            raise ValueError(f"Missing original actor feature at row {index}: {exc}") from exc
        vector = row.get("actor_observation_vector_before")
        if vector is not None:
            if len(vector) != len(schema):
                raise ValueError(f"Actor vector/schema dimension mismatch at row {index}")
            for name, first, second in zip(ACTOR_FEATURES, observations, vector[:35]):
                if not math.isclose(first, _finite(second, name), rel_tol=1e-6, abs_tol=1e-7):
                    raise ValueError(f"Actor vector/map disagreement at row {index}: {name}")
        if observations[16] < 0:
            raise ValueError(f"Negative online period at row {index}")
        target = []
        for joint in ("pros_knee_angle", "pros_ankle_angle"):
            for field in ("imitation_target_q", "imitation_target_qdot"):
                try:
                    target.append(_finite(row[field][joint], f"{field}.{joint}"))
                except KeyError as exc:
                    raise ValueError(f"Missing prescribed target at row {index}: {exc}") from exc
        prepared.append({
            "step": index + 1, "pre_time_s": pre, "post_time_s": post,
            "dt_s": post - pre, "observations": observations,
            "post_target": target, "regime": observations[16] > 0,
        })
    return {
        "episode_id": str(directory), "start_offset_s": offset, "rows": prepared,
        "actor_feature_names": schema, "summary": summary,
        "source_hashes": {str(p): _hash(p) for p in (summary_path, trace_path)},
    }


def build_dataset(episodes: list[dict]) -> dict:
    """Use identical rows for all models; five observations end at the current row."""
    import numpy as np

    samples, instant, history, oracle, targets, exclusions = [], [], [], [], [], []
    for episode in episodes:
        rows = episode["rows"]
        exclusions.append({
            "episode_id": episode["episode_id"], "raw_samples": len(rows),
            "missing_pre_target": min(1, len(rows)),
            "insufficient_past_history": min(max(len(rows) - 1, 0), HISTORY_SAMPLES - 2),
            "included_samples": max(0, len(rows) - HISTORY_SAMPLES + 1),
        })
        for index in range(HISTORY_SAMPLES - 1, len(rows)):
            current, previous = rows[index], rows[index - 1]
            if abs(current["pre_time_s"] - previous["post_time_s"]) > TIME_TOL:
                raise ValueError("Dataset target/observation timestamp mismatch")
            past = rows[index - HISTORY_SAMPLES + 1:index + 1]
            x = current["observations"]
            t = current["pre_time_s"]
            angle = 2 * math.pi * t / ORACLE_PERIOD_S
            instant.append(x)
            history.append([value for row in past for value in row["observations"]])
            oracle.append(x + [t, math.sin(angle), math.cos(angle)])
            targets.append(previous["post_target"])
            samples.append({
                "episode_id": episode["episode_id"], "start_offset_s": episode["start_offset_s"],
                "step": current["step"], "observation_time_s": t,
                "target_time_s": previous["post_time_s"], "target_source_step": previous["step"],
                "history_first_time_s": past[0]["pre_time_s"],
                "history_last_time_s": past[-1]["pre_time_s"],
                "dt_s": current["dt_s"], "regime": current["regime"],
            })
    return {
        "samples": samples, "exclusions": exclusions,
        "targets": np.asarray(targets, dtype=float).reshape(-1, 4),
        "inputs": {
            "instant35": np.asarray(instant, dtype=float).reshape(-1, 35),
            "history175": np.asarray(history, dtype=float).reshape(-1, 175),
            "instant35_time_oracle38": np.asarray(oracle, dtype=float).reshape(-1, 38),
        },
    }


def ridge_predict(train_x, train_y, test_x) -> tuple[Any, dict]:
    """Train-only standardization, unpenalized intercept, fixed output scales."""
    import numpy as np

    train_x, train_y, test_x = map(lambda x: np.asarray(x, dtype=float), (train_x, train_y, test_x))
    if train_x.ndim != 2 or train_y.shape != (len(train_x), 4) or test_x.ndim != 2:
        raise ValueError("Invalid ridge input dimensions")
    if not len(train_x) or test_x.shape[1] != train_x.shape[1]:
        raise ValueError("Empty training set or incompatible ridge features")
    if not all(np.isfinite(x).all() for x in (train_x, train_y, test_x)):
        raise ValueError("Nonfinite ridge data")
    mean, scale = train_x.mean(axis=0), train_x.std(axis=0)
    scale = np.where(scale > 1e-12, scale, 1.0)
    normalized_x = (train_x - mean) / scale
    target_scales = np.asarray(TARGET_SCALES)
    normalized_y = train_y / target_scales
    intercept = normalized_y.mean(axis=0)
    gram = normalized_x.T @ normalized_x + RIDGE_ALPHA * np.eye(train_x.shape[1])
    coefficients = np.linalg.solve(gram, normalized_x.T @ (normalized_y - intercept))
    predicted = (((test_x - mean) / scale) @ coefficients + intercept) * target_scales
    if not np.isfinite(predicted).all():
        raise ValueError("Nonfinite ridge predictions")
    return predicted, {
        "alpha": RIDGE_ALPHA, "train_samples": len(train_x), "feature_count": train_x.shape[1],
        "train_feature_mean": mean.tolist(), "train_feature_scale": scale.tolist(),
        "train_target_mean_physical": train_y.mean(axis=0).tolist(),
        "target_scales": list(TARGET_SCALES), "intercept_penalized": False,
    }


def _metrics(targets, predictions, samples: list[dict]) -> dict:
    import numpy as np

    result = {}
    for stratum in ("all", "startup", "regime"):
        mask = np.asarray([stratum == "all" or s["regime"] == (stratum == "regime") for s in samples])
        count = int(mask.sum())
        if not count:
            result[stratum] = {"status": "not_evaluable", "samples": 0, "metrics": None}
            continue
        errors = predictions[mask] - targets[mask]
        weights = np.asarray([s["dt_s"] for s in samples])[mask]
        rmse = np.sqrt(np.average(errors ** 2, axis=0, weights=weights))
        mae = np.average(np.abs(errors), axis=0, weights=weights)
        values = {}
        for index, name in enumerate(TARGET_NAMES):
            unit, degree_unit = ("rad_s", "deg_s") if "qdot" in name else ("rad", "deg")
            values[name] = {
                f"rmse_{unit}": float(rmse[index]), f"mae_{unit}": float(mae[index]),
                f"rmse_{degree_unit}": float(rmse[index] * 180 / math.pi),
                f"mae_{degree_unit}": float(mae[index] * 180 / math.pi),
            }
        result[stratum] = {
            "status": "evaluated", "samples": count, "duration_s": float(weights.sum()),
            "metrics": values,
        }
    return result


def _nearest_associations(train_x, train_y, test_x, test_y) -> dict:
    """One train neighbor per held-out observation; association, not impossibility."""
    import numpy as np

    mean, scale = train_x.mean(axis=0), train_x.std(axis=0)
    scale = np.where(scale > 1e-12, scale, 1.0)
    train, heldout = (train_x - mean) / scale, (test_x - mean) / scale
    nearest, distances = [], []
    for begin in range(0, len(heldout), 128):
        chunk = heldout[begin:begin + 128]
        squares = (chunk ** 2).sum(axis=1)[:, None] + (train ** 2).sum(axis=1)[None, :] - 2 * chunk @ train.T
        indices = squares.argmin(axis=1)
        nearest.extend(indices.tolist())
        distances.extend(np.sqrt(np.maximum(squares[np.arange(len(chunk)), indices], 0) / train.shape[1]).tolist())
    scaled_target_gap = (test_y - train_y[np.asarray(nearest)]) / np.asarray(TARGET_SCALES)
    gaps = np.sqrt(np.mean(scaled_target_gap ** 2, axis=1))
    return {
        "description": "Held-out to train-only nearest associations; no causal ambiguity verdict",
        "observation_distance": "RMS train-standardized feature difference across original 35 features",
        "target_gap": "RMS difference of four targets after fixed [1,10,1,10] scaling",
        "samples": len(test_x), "median_observation_distance": float(np.median(distances)),
        "p90_observation_distance": float(np.quantile(distances, 0.9)),
        "median_scaled_target_gap": float(np.median(gaps)),
        "p90_scaled_target_gap": float(np.quantile(gaps, 0.9)),
    }


def _write_artifacts(output_dir: Path, result: dict, dataset: dict, predictions: dict) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "observability.json").write_text(
        json.dumps(result, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    metadata = ("episode_id", "start_offset_s", "step", "observation_time_s", "target_time_s",
                "target_source_step", "history_first_time_s", "history_last_time_s", "dt_s", "regime")
    with (output_dir / "aligned_samples.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(list(metadata) + list(TARGET_NAMES) + list(ACTOR_FEATURES))
        for sample, target, x in zip(dataset["samples"], dataset["targets"], dataset["inputs"]["instant35"]):
            writer.writerow([sample[key] for key in metadata] + target.tolist() + x.tolist())
    with (output_dir / "heldout_predictions.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["model", "episode_id", "start_offset_s", "step", "observation_time_s"] + list(TARGET_NAMES))
        for name, values in predictions.items():
            for sample, prediction in zip(dataset["samples"], values):
                if all(math.isfinite(float(value)) for value in prediction):
                    writer.writerow([name] + [sample[key] for key in ("episode_id", "start_offset_s", "step", "observation_time_s")] + prediction.tolist())
    lines = ["# Osservabilità: diagnostica offline", "", f"Stato tecnico: **{result['status']}**.", "",
             "Ricostruzione associativa con episodi/start esclusi dal fit. Non dimostra impossibilità",
             "informativa né stabilità del controllo. Il time oracle usa tempo assoluto e periodo",
             "nominale 1.58 s: non è fase prescribed misurata e non è un ingresso della policy.", "",
             f"Campioni comuni: {len(dataset['samples'])}; start utilizzabili: {len(result['usable_start_offsets_s'])}.", "",
             "| Modello | Campioni | RMSE knee [°] | RMSE knee qdot [°/s] | RMSE ankle [°] | RMSE ankle qdot [°/s] |",
             "|---|---:|---:|---:|---:|---:|"]
    for name, model in result["models"].items():
        full = model["metrics"]["all"]
        if full["metrics"] is None:
            lines.append(f"| {name} | 0 | N/A | N/A | N/A | N/A |")
        else:
            values = full["metrics"]
            numbers = [values[key]["rmse_deg_s" if "qdot" in key else "rmse_deg"] for key in TARGET_NAMES]
            lines.append(f"| {name} | {full['samples']} | " + " | ".join(f"{number:.6g}" for number in numbers) + " |")
    lines.extend(["", "Avvio/regime, metriche rad/rad/s, MAE, fold, standardizzazione fit-only,",
                  "copertura, esclusioni e hash sono in `observability.json`."])
    if not any(s["regime"] for s in dataset["samples"]):
        lines.extend(["", "**Regime non valutabile:** nessun campione ha un periodo online valido > 0."])
    if result["reasons"]:
        lines.extend(["", "Motivi: " + "; ".join(result["reasons"]) + "."])
    (output_dir / "observability.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def analyze(paths, output_dir) -> dict:
    """Analyze supplied rollout directories, writing JSON, aligned CSV and Markdown."""
    import numpy as np

    episodes = [load_episode(path) for path in paths]
    if len({ep["episode_id"] for ep in episodes}) != len(episodes):
        raise ValueError("Duplicate rollout input would count the same episode twice")
    dataset = build_dataset(episodes)
    samples, targets = dataset["samples"], dataset["targets"]
    starts = sorted({s["start_offset_s"] for s in samples})
    predicted = {name: np.full_like(targets, np.nan) for name in (*MODELS, "train_constant")}
    result = {
        "schema_version": 1, "status": "completed", "reasons": [],
        "interpretation": "Associative offline reconstruction, not a causal impossibility or walking-quality test",
        "protocol": {
            "actor_features": list(ACTOR_FEATURES), "target_names": list(TARGET_NAMES),
            "target_scales": list(TARGET_SCALES), "ridge_alpha": RIDGE_ALPHA,
            "history_samples": HISTORY_SAMPLES, "history_order": "oldest to current, no future observations",
            "alignment": "current pre-step observation with previous row post-step target; first row excluded",
            "cohort": "identical history-eligible rows for every model and constant predictor",
            "split": "leave-one-start-out, all episodes of held-out start excluded from fit",
            "normalization": "feature mean/std estimated on train rows only; constant columns scale=1",
            "oracle": {"inputs": ["absolute_time_s", "nominal_period_sin", "nominal_period_cos"],
                       "period_s": ORACLE_PERIOD_S, "measured_prescribed_phase": False, "deployment_input": False},
            "min_train_samples": MIN_TRAIN_SAMPLES, "min_test_samples": MIN_TEST_SAMPLES,
            "metrics_weighting": "sample duration; fitting itself uses equal sample weights",
            "strata": "current pre-step online_left_cycle_duration_s > 0 is regime; zero is startup/unavailable",
        },
        "sources": [{key: ep[key] for key in ("episode_id", "start_offset_s", "source_hashes", "actor_feature_names")} for ep in episodes],
        "analyzer_sha256": _hash(Path(__file__)), "exclusions": dataset["exclusions"],
        "usable_start_offsets_s": starts, "samples": len(samples), "folds": [], "models": {},
    }
    groups = np.asarray([s["start_offset_s"] for s in samples])
    if len(starts) < 3:
        result["reasons"].append("Fewer than three usable independent start conditions")
    else:
        for heldout in starts:
            test, train = groups == heldout, groups != heldout
            fold = {
                "heldout_start_offset_s": heldout, "train_samples": int(train.sum()),
                "test_samples": int(test.sum()), "status": "completed", "models": {},
                "train_episodes": sorted({s["episode_id"] for s, yes in zip(samples, train) if yes}),
                "test_episodes": sorted({s["episode_id"] for s, yes in zip(samples, test) if yes}),
            }
            result["folds"].append(fold)
            if train.sum() < MIN_TRAIN_SAMPLES or test.sum() < MIN_TEST_SAMPLES:
                fold["status"] = "inconclusive"
                result["reasons"].append(f"Insufficient rows for held-out start {heldout}")
                continue
            selected = [s for s, yes in zip(samples, test) if yes]
            predicted["train_constant"][test] = targets[train].mean(axis=0)
            for name, inputs in dataset["inputs"].items():
                pred, audit = ridge_predict(inputs[train], targets[train], inputs[test])
                predicted[name][test] = pred
                fold["models"][name] = {"fit_audit": audit, "metrics": _metrics(targets[test], pred, selected)}
            fold["nearest_observation_associations"] = _nearest_associations(
                dataset["inputs"]["instant35"][train], targets[train],
                dataset["inputs"]["instant35"][test], targets[test])
    if result["reasons"]:
        result["status"] = "inconclusive"
    for name, values in predicted.items():
        evaluated = np.isfinite(values).all(axis=1)
        selected = [s for s, yes in zip(samples, evaluated) if yes]
        result["models"][name] = {
            "evaluated_samples": len(selected), "available_samples": len(samples),
            "metrics": _metrics(targets[evaluated], values[evaluated], selected),
        }
    _write_artifacts(Path(output_dir).expanduser().resolve(), result, dataset, predicted)
    return result


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rollouts", nargs="+", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    args = parser.parse_args(argv)
    result = analyze(args.rollouts, args.output_dir)
    print(json.dumps({"status": result["status"], "samples": result["samples"], "output_dir": str(args.output_dir)}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
