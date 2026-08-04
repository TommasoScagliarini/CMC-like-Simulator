"""Validate rollout served references against the morphology corridor by plot.

The rollout trace records the exact morphology phase, served knee/ankle
reference, and corridor limits used by ``RewardFunction`` at every policy
step.  This script overlays those samples on the full corridor reconstructed
from the saved morphology profile and the rollout's resolved reward config.
"""

from __future__ import annotations

import argparse
import json
import os
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping, Sequence

os.environ.setdefault(
    "MPLCONFIGDIR", str(Path(tempfile.gettempdir()) / "cmc_like_matplotlib")
)
os.environ.setdefault(
    "XDG_CACHE_HOME", str(Path(tempfile.gettempdir()) / "cmc_like_cache")
)

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent
BASELINE_DIR = REPO_ROOT / "Trajectory Generator" / "baseline_MLP"


@dataclass(frozen=True)
class JointSpec:
    coord: str
    short_name: str
    title: str
    multiplier_key: str
    margin_key: str
    value_key: str
    min_key: str
    max_key: str
    excursion_key: str
    display_sign: float


JOINTS = (
    JointSpec(
        coord="pros_knee_angle",
        short_name="knee",
        title="Prosthetic knee",
        multiplier_key="morphology_std_multiplier_knee",
        margin_key="morphology_margin_knee_deg",
        value_key="morphology_knee_value_rad",
        min_key="morphology_knee_min_rad",
        max_key="morphology_knee_max_rad",
        excursion_key="morphology_knee_excursion_rad",
        display_sign=-1.0,
    ),
    JointSpec(
        coord="pros_ankle_angle",
        short_name="ankle",
        title="Prosthetic ankle",
        multiplier_key="morphology_std_multiplier_ankle",
        margin_key="morphology_margin_ankle_deg",
        value_key="morphology_ankle_value_rad",
        min_key="morphology_ankle_min_rad",
        max_key="morphology_ankle_max_rad",
        excursion_key="morphology_ankle_excursion_rad",
        display_sign=1.0,
    ),
)


def _load_json(path: Path) -> Any:
    with path.open("r", encoding="utf-8") as stream:
        return json.load(stream)


def _finite_float(value: Any) -> float | None:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if np.isfinite(result) else None


def _resolve_profile_path(
    rollout_dir: Path,
    summary: Mapping[str, Any],
    override: Path | None,
) -> Path:
    if override is not None:
        path = override.expanduser()
        if not path.is_absolute():
            path = (Path.cwd() / path).resolve()
        if not path.is_file():
            raise FileNotFoundError(f"Morphology profile not found: {path}")
        return path

    reward_config = summary.get("reward_config")
    if not isinstance(reward_config, Mapping):
        raise ValueError("rollout_summary.json has no reward_config mapping")
    raw = reward_config.get("morphology_profile")
    if not isinstance(raw, str) or not raw.strip():
        raise ValueError("reward_config.morphology_profile is missing")

    stored = Path(raw).expanduser()
    candidates = (
        stored,
        BASELINE_DIR / stored,
        SCRIPT_DIR / stored,
        rollout_dir / stored,
        REPO_ROOT / stored,
    )
    for candidate in candidates:
        path = candidate if candidate.is_absolute() else candidate.resolve()
        if path.is_file():
            return path
    searched = ", ".join(str(path) for path in candidates)
    raise FileNotFoundError(
        f"Could not resolve morphology profile {raw!r}; searched: {searched}"
    )


def _reward_config(summary: Mapping[str, Any]) -> Mapping[str, Any]:
    config = summary.get("reward_config")
    if not isinstance(config, Mapping):
        raise ValueError("rollout_summary.json has no reward_config mapping")
    return config


def _extract_samples(trace: Sequence[Any]) -> dict[str, np.ndarray]:
    rows: list[dict[str, float]] = []
    for item in trace:
        if not isinstance(item, Mapping):
            continue
        terms = item.get("reward_terms")
        if not isinstance(terms, Mapping):
            continue
        available = _finite_float(terms.get("morphology_available"))
        phase = _finite_float(terms.get("morphology_phase"))
        step = _finite_float(item.get("step"))
        time_s = _finite_float(item.get("time"))
        source_id = _finite_float(terms.get("morphology_phase_source_id"))
        if available != 1.0 or phase is None or step is None or time_s is None:
            continue

        row = {
            "phase": float(np.clip(phase, 0.0, 1.0)),
            "step": step,
            "time_s": time_s,
            "source_id": source_id or 0.0,
            "fsm_state_id": _finite_float(terms.get("phase_fsm_state_id")) or 0.0,
            "valid_hs_count": _finite_float(terms.get("phase_valid_hs_count")) or 0.0,
            "valid_to_count": _finite_float(terms.get("phase_valid_to_count")) or 0.0,
        }
        valid = True
        for joint in JOINTS:
            for key in (
                joint.value_key,
                joint.min_key,
                joint.max_key,
                joint.excursion_key,
            ):
                value = _finite_float(terms.get(key))
                if value is None:
                    valid = False
                    break
                row[key] = value
            if not valid:
                break
        if valid:
            rows.append(row)

    if not rows:
        raise ValueError("No complete morphology samples found in rollout trace")
    return {
        key: np.asarray([row[key] for row in rows], dtype=float)
        for key in rows[0]
    }


def _continuous_segments(samples: Mapping[str, np.ndarray]) -> list[np.ndarray]:
    phase = samples["phase"]
    step = samples["step"]
    source_id = samples["source_id"]
    fsm_state_id = samples["fsm_state_id"]
    starts = [0]
    for index in range(1, phase.size):
        phase_reset = phase[index] < phase[index - 1] - 0.20
        step_gap = step[index] != step[index - 1] + 1.0
        source_change = source_id[index] != source_id[index - 1]
        fsm_state_change = fsm_state_id[index] != fsm_state_id[index - 1]
        if phase_reset or step_gap or source_change or fsm_state_change:
            starts.append(index)
    starts.append(phase.size)
    return [np.arange(starts[i], starts[i + 1]) for i in range(len(starts) - 1)]


def _corridor(
    profile: Mapping[str, Any],
    config: Mapping[str, Any],
    joint: JointSpec,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    phase = np.asarray(profile["phase_grid"], dtype=float)
    coordinates = profile.get("coordinates")
    if not isinstance(coordinates, Mapping) or joint.coord not in coordinates:
        raise ValueError(f"Profile is missing coordinates.{joint.coord}")
    coord = coordinates[joint.coord]
    if not isinstance(coord, Mapping):
        raise ValueError(f"coordinates.{joint.coord} must be a mapping")
    mean = np.asarray(coord["mean_rad"], dtype=float)
    std = np.asarray(coord["std_rad"], dtype=float)
    if phase.ndim != 1 or mean.shape != phase.shape or std.shape != phase.shape:
        raise ValueError(f"Invalid phase/mean/std shape for {joint.coord}")
    multiplier = float(config[joint.multiplier_key])
    margin = float(np.deg2rad(float(config[joint.margin_key])))
    low = mean - multiplier * std - margin
    high = mean + multiplier * std + margin
    return phase, mean, np.minimum(low, high), np.maximum(low, high)


def _joint_metrics(
    samples: Mapping[str, np.ndarray],
    joint: JointSpec,
) -> dict[str, float | int]:
    values = samples[joint.value_key]
    low = samples[joint.min_key]
    high = samples[joint.max_key]
    outside = (values < low) | (values > high)
    excursions = samples[joint.excursion_key]
    return {
        "sample_count": int(values.size),
        "inside_count": int(np.count_nonzero(~outside)),
        "outside_count": int(np.count_nonzero(outside)),
        "outside_fraction": float(np.mean(outside)),
        "max_excursion_rad": float(np.max(excursions)),
        "mean_excursion_rad": float(np.mean(excursions)),
    }


def _validate_logged_corridor(
    samples: Mapping[str, np.ndarray],
    profile: Mapping[str, Any],
    config: Mapping[str, Any],
) -> dict[str, float]:
    errors: dict[str, float] = {}
    for joint in JOINTS:
        phase, _, low, high = _corridor(profile, config, joint)
        expected_low = np.interp(samples["phase"], phase, low)
        expected_high = np.interp(samples["phase"], phase, high)
        errors[f"{joint.short_name}_min_max_abs_error_rad"] = float(
            np.max(np.abs(expected_low - samples[joint.min_key]))
        )
        errors[f"{joint.short_name}_max_max_abs_error_rad"] = float(
            np.max(np.abs(expected_high - samples[joint.max_key]))
        )
    return errors


def _plot(
    *,
    rollout_dir: Path,
    output_path: Path,
    profile: Mapping[str, Any],
    config: Mapping[str, Any],
    samples: Mapping[str, np.ndarray],
    segments: Sequence[np.ndarray],
    metrics: Mapping[str, Mapping[str, float | int]],
) -> None:
    plt.rcParams.update(
        {
            "font.size": 11,
            "axes.titlesize": 15,
            "axes.labelsize": 12,
            "legend.fontsize": 9,
            "figure.titlesize": 19,
        }
    )
    fig, axes = plt.subplots(1, 2, figsize=(16, 7.5), sharex=True)
    colors = plt.cm.viridis(np.linspace(0.12, 0.88, max(1, len(segments))))

    profile_metadata = profile.get("metadata")
    mean_to_phase = None
    if isinstance(profile_metadata, Mapping):
        mean_to_phase = _finite_float(profile_metadata.get("mean_to_phase"))
    fsm_stance_phase_end = _finite_float(config.get("prosthetic_stance_phase_end"))
    valid_to_event = np.diff(
        samples["valid_to_count"], prepend=samples["valid_to_count"][0]
    ) > 0.0

    for axis, joint in zip(axes, JOINTS):
        phase, mean, low, high = _corridor(profile, config, joint)
        phase_pct = phase * 100.0
        display_mean = joint.display_sign * mean
        display_low = np.minimum(joint.display_sign * low, joint.display_sign * high)
        display_high = np.maximum(joint.display_sign * low, joint.display_sign * high)
        axis.fill_between(
            phase_pct,
            display_low,
            display_high,
            color="#9ecae1",
            alpha=0.52,
            linewidth=0.0,
            label="morphology corridor",
            zorder=1,
        )
        axis.plot(
            phase_pct,
            display_mean,
            color="#37474f",
            linestyle="--",
            linewidth=1.8,
            label="profile mean",
            zorder=2,
        )

        served_label_used = False
        bootstrap_label_used = False
        for segment_number, indices in enumerate(segments):
            is_bootstrap = bool(np.all(samples["source_id"][indices] == 1.0))
            if is_bootstrap and not bootstrap_label_used:
                label = "served_ref (FSM bootstrap)"
                bootstrap_label_used = True
            elif not is_bootstrap and not served_label_used:
                label = "served_ref (measured FSM cycles)"
                served_label_used = True
            else:
                label = "_nolegend_"
            axis.plot(
                samples["phase"][indices] * 100.0,
                joint.display_sign * samples[joint.value_key][indices],
                color=colors[segment_number],
                linestyle="--" if is_bootstrap else "-",
                linewidth=1.65,
                alpha=0.92,
                label=label,
                zorder=3,
            )

        values = samples[joint.value_key]
        display_values = joint.display_sign * values
        low_logged = samples[joint.min_key]
        high_logged = samples[joint.max_key]
        outside = (values < low_logged) | (values > high_logged)
        if np.any(outside):
            axis.scatter(
                samples["phase"][outside] * 100.0,
                display_values[outside],
                marker="x",
                s=15,
                linewidths=0.75,
                color="#d62728",
                alpha=0.62,
                label="served_ref outside corridor",
                zorder=4,
            )
        if np.any(valid_to_event):
            axis.scatter(
                samples["phase"][valid_to_event] * 100.0,
                display_values[valid_to_event],
                marker="v",
                s=58,
                linewidths=0.7,
                edgecolors="#7f2704",
                facecolors="#fdae6b",
                label="valid runtime toe-off",
                zorder=5,
            )

        if mean_to_phase is not None:
            axis.axvline(
                mean_to_phase * 100.0,
                color="#7f7f7f",
                linestyle=":",
                linewidth=1.2,
                label="profile mean toe-off",
                zorder=0,
            )
        if fsm_stance_phase_end is not None:
            axis.axvline(
                fsm_stance_phase_end * 100.0,
                color="#e67e22",
                linestyle="-.",
                linewidth=1.2,
                label="FSM bootstrap stance/swing boundary",
                zorder=0,
            )

        joint_metrics = metrics[joint.short_name]
        outside_count = int(joint_metrics["outside_count"])
        sample_count = int(joint_metrics["sample_count"])
        max_excursion = float(joint_metrics["max_excursion_rad"])
        axis.text(
            0.015,
            0.02,
            (
                f"step-weighted inside: {sample_count - outside_count}/{sample_count}\n"
                f"step-weighted outside: {outside_count}/{sample_count}\n"
                f"max excursion: {max_excursion:.3f} rad"
            ),
            transform=axis.transAxes,
            ha="left",
            va="bottom",
            bbox={"boxstyle": "round,pad=0.35", "fc": "white", "alpha": 0.86},
            zorder=5,
        )
        title = joint.title
        if joint.display_sign < 0.0:
            title += " (flexion positive)"
        axis.set_title(title)
        axis.set_xlabel("morphology phase [% gait cycle]")
        axis.set_ylabel("served reference [rad]")
        axis.set_xlim(0.0, 100.0)
        axis.set_xticks(np.arange(0.0, 101.0, 10.0))
        axis.grid(True, alpha=0.28)

    weight = float(config.get("morphology_weight", 0.0))
    knee_k = float(config["morphology_std_multiplier_knee"])
    ankle_k = float(config["morphology_std_multiplier_ankle"])
    knee_margin = float(config["morphology_margin_knee_deg"])
    ankle_margin = float(config["morphology_margin_ankle_deg"])
    phase_mode = str(config.get("morphology_phase_mode", "legacy_cycle_fraction"))
    profile_name = str(profile.get("name", "unnamed_morphology_profile"))
    fig.suptitle(
        f"Checkpoint best: served references vs morphology corridor ({phase_mode})",
        y=0.975,
    )
    fig.text(
        0.5,
        0.925,
        (
            f"knee: mean ± {knee_k:g}σ ± {knee_margin:g}°   |   "
            f"ankle: mean ± {ankle_k:g}σ ± {ankle_margin:g}°   |   "
            f"morphology_weight={weight:g} (diagnostic when zero)"
        ),
        ha="center",
        va="center",
        color="#555555",
    )
    fig.text(
        0.5,
        0.892,
        (
            f"profile: {profile_name}   |   "
            f"{samples['phase'].size} morphology-available steps, "
            f"{len(segments)} continuous phase segments"
        ),
        ha="center",
        va="center",
        color="#666666",
        fontsize=9.5,
    )

    handles, labels = axes[1].get_legend_handles_labels()
    if not labels:
        handles, labels = axes[0].get_legend_handles_labels()
    unique: dict[str, Any] = {}
    for handle, label in zip(handles, labels):
        if label != "_nolegend_" and label not in unique:
            unique[label] = handle
    fig.legend(
        unique.values(),
        unique.keys(),
        loc="upper center",
        bbox_to_anchor=(0.5, 0.855),
        ncol=min(4, len(unique)),
        frameon=True,
    )
    fig.subplots_adjust(left=0.075, right=0.985, bottom=0.09, top=0.74, wspace=0.16)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=180, bbox_inches="tight")
    plt.close(fig)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--rollout-dir",
        required=True,
        type=Path,
        help="Rollout directory containing rollout_summary.json and policy trace.",
    )
    parser.add_argument(
        "--output",
        required=True,
        type=Path,
        help="Destination PNG path.",
    )
    parser.add_argument(
        "--profile",
        type=Path,
        default=None,
        help="Optional morphology profile override.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    rollout_dir = args.rollout_dir.expanduser().resolve()
    output_path = args.output.expanduser().resolve()
    summary_path = rollout_dir / "rollout_summary.json"
    trace_path = rollout_dir / "rollout_policy_trace.json"
    if not summary_path.is_file():
        raise FileNotFoundError(summary_path)
    if not trace_path.is_file():
        raise FileNotFoundError(trace_path)

    summary = _load_json(summary_path)
    trace = _load_json(trace_path)
    if not isinstance(summary, Mapping):
        raise ValueError("rollout_summary.json must contain a JSON object")
    if not isinstance(trace, list):
        raise ValueError("rollout_policy_trace.json must contain a JSON list")
    config = _reward_config(summary)
    profile_path = _resolve_profile_path(rollout_dir, summary, args.profile)
    profile = _load_json(profile_path)
    if not isinstance(profile, Mapping):
        raise ValueError("Morphology profile must contain a JSON object")

    samples = _extract_samples(trace)
    segments = _continuous_segments(samples)
    joint_metrics = {
        joint.short_name: _joint_metrics(samples, joint) for joint in JOINTS
    }
    corridor_errors = _validate_logged_corridor(samples, profile, config)
    max_error = max(corridor_errors.values(), default=0.0)
    if max_error > 1e-8:
        raise ValueError(
            "Reconstructed corridor does not match logged limits: "
            f"max error {max_error:.6g} rad"
        )

    _plot(
        rollout_dir=rollout_dir,
        output_path=output_path,
        profile=profile,
        config=config,
        samples=samples,
        segments=segments,
        metrics=joint_metrics,
    )
    result = {
        "ok": True,
        "rollout_dir": str(rollout_dir),
        "profile": str(profile_path),
        "output": str(output_path),
        "morphology_weight": float(config.get("morphology_weight", 0.0)),
        "evaluated_samples": int(samples["phase"].size),
        "continuous_phase_segments": len(segments),
        "phase_source_counts": {
            str(int(source_id)): int(np.count_nonzero(samples["source_id"] == source_id))
            for source_id in np.unique(samples["source_id"])
        },
        "corridor_reconstruction_errors": corridor_errors,
        "joints": joint_metrics,
    }
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
