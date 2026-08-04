#!/usr/bin/env python3
"""Validate the event-warped AB06 morphology profile with blocked folds.

The five folds are contiguous in acquisition order.  This intentionally keeps
speed/style regimes together and avoids the optimistic leakage produced by a
random cycle split.  For every fold, the canonical TO phase and corridor are
estimated only from the other four blocks.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import tempfile
from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import Any

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

from build_event_warped_morphology_profile import (
    DEFAULT_COORDINATES,
    DEFAULT_GRF,
    DEFAULT_IK,
    DEFAULT_OUTPUT,
    _phase_grid,
    build_profile,
    detect_cycles,
    event_warp_cycles,
    load_inputs,
)


SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent
DEFAULT_LEGACY_PROFILE = (
    REPO_ROOT
    / "Trajectory Generator"
    / "baseline_MLP"
    / "morphology_profiles"
    / "ab06_prosthetic_mean_std_corridor.json"
)
DEFAULT_PROFILE = REPO_ROOT / DEFAULT_OUTPUT
DEFAULT_OUTPUT_DIR = (
    SCRIPT_DIR
    / "event_anchored_morphology_runs"
    / "2026-07-20_profile_blocked_cv"
)
JOINTS = {
    "pros_knee_angle": {"short": "knee", "k": 1.6, "margin_deg": 7.5},
    "pros_ankle_angle": {"short": "ankle", "k": 0.6, "margin_deg": 7.5},
}
GATE_THRESHOLDS = {
    "aggregate_coverage_min": 0.95,
    "worst_fold_coverage_min": 0.85,
    "outside_excursion_p95_max_deg": 10.0,
    "mean_std_reduction_min": 0.10,
}


def _load_json(path: Path) -> Mapping[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, Mapping):
        raise ValueError(f"expected a JSON object: {path}")
    return value


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _cycle_duty(cycles: Sequence[tuple[float, float, float]]) -> np.ndarray:
    return np.asarray(
        [stance / (end - start) for start, end, stance in cycles],
        dtype=float,
    )


def _whole_cycle_values(
    ik_time: np.ndarray,
    values: np.ndarray,
    cycles: Sequence[tuple[float, float, float]],
    phase: np.ndarray,
) -> np.ndarray:
    rows = []
    for start, end, _stance in cycles:
        rows.append(np.interp(start + phase * (end - start), ik_time, values))
    return np.vstack(rows)


def _empty_accumulator() -> dict[str, Any]:
    return {
        "points": 0,
        "inside": 0,
        "stance_points": 0,
        "stance_inside": 0,
        "swing_points": 0,
        "swing_inside": 0,
        "excursions_rad": [],
        "outside_excursions_rad": [],
        "cycle_coverages": [],
    }


def _evaluate_corridor(
    values: np.ndarray,
    mean: np.ndarray,
    std: np.ndarray,
    phase: np.ndarray,
    alpha: float,
    *,
    multiplier: float,
    margin_deg: float,
) -> tuple[dict[str, Any], dict[str, Any]]:
    margin = float(np.deg2rad(margin_deg))
    low = mean - multiplier * std - margin
    high = mean + multiplier * std + margin
    excursion = np.maximum(low[None, :] - values, values - high[None, :])
    excursion = np.maximum(excursion, 0.0)
    inside = excursion <= 1e-15
    stance_mask = phase <= alpha
    swing_mask = ~stance_mask
    cycle_coverages = np.mean(inside, axis=1)
    outside_values = excursion[~inside]
    raw = {
        "points": int(inside.size),
        "inside": int(np.count_nonzero(inside)),
        "stance_points": int(values.shape[0] * np.count_nonzero(stance_mask)),
        "stance_inside": int(np.count_nonzero(inside[:, stance_mask])),
        "swing_points": int(values.shape[0] * np.count_nonzero(swing_mask)),
        "swing_inside": int(np.count_nonzero(inside[:, swing_mask])),
        "excursions_rad": excursion.ravel().tolist(),
        "outside_excursions_rad": outside_values.tolist(),
        "cycle_coverages": cycle_coverages.tolist(),
    }
    summary = {
        "point_coverage": float(np.mean(inside)),
        "stance_coverage": float(np.mean(inside[:, stance_mask])),
        "swing_coverage": float(np.mean(inside[:, swing_mask])),
        "cycle_coverage_mean": float(np.mean(cycle_coverages)),
        "cycle_coverage_min": float(np.min(cycle_coverages)),
        "outside_point_count": int(outside_values.size),
        "outside_excursion_p95_deg": float(
            np.rad2deg(np.percentile(outside_values, 95.0))
        )
        if outside_values.size
        else 0.0,
        "max_excursion_deg": float(np.rad2deg(np.max(excursion))),
    }
    return raw, summary


def _merge_accumulator(target: dict[str, Any], current: Mapping[str, Any]) -> None:
    for key in (
        "points",
        "inside",
        "stance_points",
        "stance_inside",
        "swing_points",
        "swing_inside",
    ):
        target[key] += int(current[key])
    for key in (
        "excursions_rad",
        "outside_excursions_rad",
        "cycle_coverages",
    ):
        target[key].extend(current[key])


def _summarize_accumulator(accumulator: Mapping[str, Any]) -> dict[str, Any]:
    outside = np.asarray(accumulator["outside_excursions_rad"], dtype=float)
    excursions = np.asarray(accumulator["excursions_rad"], dtype=float)
    cycle_coverage = np.asarray(accumulator["cycle_coverages"], dtype=float)
    return {
        "point_count": int(accumulator["points"]),
        "point_coverage": float(accumulator["inside"] / accumulator["points"]),
        "stance_coverage": float(
            accumulator["stance_inside"] / accumulator["stance_points"]
        ),
        "swing_coverage": float(
            accumulator["swing_inside"] / accumulator["swing_points"]
        ),
        "cycle_coverage_mean": float(np.mean(cycle_coverage)),
        "cycle_coverage_min": float(np.min(cycle_coverage)),
        "outside_point_count": int(outside.size),
        "outside_excursion_p95_deg": float(
            np.rad2deg(np.percentile(outside, 95.0))
        )
        if outside.size
        else 0.0,
        "max_excursion_deg": float(np.rad2deg(np.max(excursions))),
    }


def _blocked_cross_validation(
    inputs: Mapping[str, Any],
    cycles: Sequence[tuple[float, float, float]],
    folds: int,
) -> dict[str, Any]:
    if folds < 2 or folds > len(cycles):
        raise ValueError("fold count must be between 2 and the cycle count")
    blocks = [
        np.asarray(block, dtype=int)
        for block in np.array_split(np.arange(len(cycles), dtype=int), folds)
    ]
    aggregate = {coordinate: _empty_accumulator() for coordinate in JOINTS}
    fold_reports: list[dict[str, Any]] = []
    all_indices = np.arange(len(cycles), dtype=int)

    for fold_index, held_indices in enumerate(blocks, start=1):
        train_indices = np.setdiff1d(all_indices, held_indices, assume_unique=True)
        train_cycles = [cycles[index] for index in train_indices]
        held_cycles = [cycles[index] for index in held_indices]
        train_alpha = round(float(np.mean(_cycle_duty(train_cycles))), 10)
        phase, alpha_inserted = _phase_grid(201, train_alpha)
        fold_report: dict[str, Any] = {
            "fold": fold_index,
            "split": "contiguous_chronological_block",
            "train_cycle_count": len(train_cycles),
            "heldout_cycle_count": len(held_cycles),
            "heldout_cycle_indices_zero_based": held_indices.tolist(),
            "heldout_time_start_s": float(held_cycles[0][0]),
            "heldout_time_end_s": float(held_cycles[-1][1]),
            "train_canonical_to_phase": train_alpha,
            "alpha_inserted_in_grid": bool(alpha_inserted),
            "train_period_mean_s": float(
                np.mean([end - start for start, end, _ in train_cycles])
            ),
            "heldout_period_mean_s": float(
                np.mean([end - start for start, end, _ in held_cycles])
            ),
            "heldout_duty_factor_mean": float(np.mean(_cycle_duty(held_cycles))),
            "joints": {},
        }
        for coordinate, spec in JOINTS.items():
            source = inputs["coordinate_values_rad"][coordinate]
            train_values = event_warp_cycles(
                inputs["ik_time"], source, train_cycles, phase, train_alpha
            )
            held_values = event_warp_cycles(
                inputs["ik_time"], source, held_cycles, phase, train_alpha
            )
            raw, summary = _evaluate_corridor(
                held_values,
                np.mean(train_values, axis=0),
                np.std(train_values, axis=0, ddof=0),
                phase,
                train_alpha,
                multiplier=spec["k"],
                margin_deg=spec["margin_deg"],
            )
            _merge_accumulator(aggregate[coordinate], raw)
            fold_report["joints"][spec["short"]] = summary
        fold_reports.append(fold_report)

    return {
        "strategy": "five contiguous chronological blocks; alpha fit on train only",
        "fold_count": folds,
        "folds": fold_reports,
        "aggregate": {
            JOINTS[coordinate]["short"]: _summarize_accumulator(values)
            for coordinate, values in aggregate.items()
        },
    }


def _dispersion_comparison(
    inputs: Mapping[str, Any],
    cycles: Sequence[tuple[float, float, float]],
    event_profile: Mapping[str, Any],
    legacy_profile: Mapping[str, Any],
) -> dict[str, Any]:
    alpha = float(event_profile["metadata"]["canonical_to_phase"])
    event_phase = np.asarray(event_profile["phase_grid"], dtype=float)
    legacy_phase = np.asarray(legacy_profile["phase_grid"], dtype=float)
    result: dict[str, Any] = {}
    for coordinate, spec in JOINTS.items():
        event_std = np.asarray(
            event_profile["coordinates"][coordinate]["std_rad"], dtype=float
        )
        legacy_std = np.asarray(
            legacy_profile["coordinates"][coordinate]["std_rad"], dtype=float
        )
        event_mean_std = float(np.mean(event_std))
        legacy_mean_std = float(np.mean(legacy_std))
        event_to_std = float(np.interp(alpha, event_phase, event_std))
        legacy_to_std = float(np.interp(alpha, legacy_phase, legacy_std))
        result[spec["short"]] = {
            "legacy_mean_std_deg": float(np.rad2deg(legacy_mean_std)),
            "event_warped_mean_std_deg": float(np.rad2deg(event_mean_std)),
            "mean_std_reduction_fraction": float(
                1.0 - event_mean_std / legacy_mean_std
            ),
            "legacy_to_std_deg": float(np.rad2deg(legacy_to_std)),
            "event_warped_to_std_deg": float(np.rad2deg(event_to_std)),
            "to_std_reduction_fraction": float(1.0 - event_to_std / legacy_to_std),
        }
    return result


def _gates(
    *,
    profile_matches: bool,
    profile: Mapping[str, Any],
    cv: Mapping[str, Any],
    dispersion: Mapping[str, Any],
) -> dict[str, Any]:
    gates: dict[str, Any] = {
        "profile_rebuild_byte_exact": {"pass": profile_matches},
        "event_profile_contract": {
            "pass": (
                profile.get("version") == 2
                and profile.get("metadata", {}).get("phase_parameterization")
                == "event_warped_hs_to_to_to_hs_v1"
                and profile.get("metadata", {}).get("n_cycles") == 123
            )
        },
    }
    for joint in ("knee", "ankle"):
        aggregate = cv["aggregate"][joint]
        fold_coverages = [
            fold["joints"][joint]["point_coverage"] for fold in cv["folds"]
        ]
        gates[f"{joint}_aggregate_coverage"] = {
            "pass": aggregate["point_coverage"]
            >= GATE_THRESHOLDS["aggregate_coverage_min"],
            "value": aggregate["point_coverage"],
            "threshold": GATE_THRESHOLDS["aggregate_coverage_min"],
        }
        gates[f"{joint}_worst_fold_coverage"] = {
            "pass": min(fold_coverages)
            >= GATE_THRESHOLDS["worst_fold_coverage_min"],
            "value": min(fold_coverages),
            "threshold": GATE_THRESHOLDS["worst_fold_coverage_min"],
        }
        gates[f"{joint}_outside_excursion_p95"] = {
            "pass": aggregate["outside_excursion_p95_deg"]
            <= GATE_THRESHOLDS["outside_excursion_p95_max_deg"],
            "value_deg": aggregate["outside_excursion_p95_deg"],
            "threshold_deg": GATE_THRESHOLDS["outside_excursion_p95_max_deg"],
        }
        gates[f"{joint}_dispersion_reduced"] = {
            "pass": dispersion[joint]["mean_std_reduction_fraction"]
            >= GATE_THRESHOLDS["mean_std_reduction_min"],
            "value": dispersion[joint]["mean_std_reduction_fraction"],
            "threshold": GATE_THRESHOLDS["mean_std_reduction_min"],
        }
    gates["all"] = {
        "pass": all(item["pass"] for item in gates.values())
    }
    return gates


def _plot_dispersion(
    output: Path,
    legacy: Mapping[str, Any],
    event: Mapping[str, Any],
) -> None:
    alpha = float(event["metadata"]["canonical_to_phase"])
    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    for axis, (coordinate, spec) in zip(axes, JOINTS.items()):
        for profile, label, color in (
            (legacy, "legacy HS-HS normalization", "#4c78a8"),
            (event, "event-warped HS-TO-HS", "#f58518"),
        ):
            phase = np.asarray(profile["phase_grid"], dtype=float)
            std = np.asarray(profile["coordinates"][coordinate]["std_rad"])
            axis.plot(phase * 100.0, np.rad2deg(std), label=label, color=color)
        axis.axvline(alpha * 100.0, color="#7f3c8d", linestyle=":", label="TO")
        axis.set_ylabel(f"{spec['short'].title()} std [deg]")
        axis.grid(alpha=0.25)
        axis.legend(loc="best")
    axes[-1].set_xlabel("Canonical gait phase [%]")
    fig.suptitle("AB06 morphology dispersion before and after event warping")
    fig.tight_layout()
    fig.savefig(output, dpi=180, bbox_inches="tight")
    plt.close(fig)


def _plot_fold_coverage(output: Path, cv: Mapping[str, Any]) -> None:
    folds = cv["folds"]
    x = np.arange(len(folds))
    width = 0.36
    knee = [fold["joints"]["knee"]["point_coverage"] * 100.0 for fold in folds]
    ankle = [fold["joints"]["ankle"]["point_coverage"] * 100.0 for fold in folds]
    fig, axis = plt.subplots(figsize=(11, 5.5))
    axis.bar(x - width / 2, knee, width, label="knee", color="#4c78a8")
    axis.bar(x + width / 2, ankle, width, label="ankle", color="#f58518")
    axis.axhline(
        GATE_THRESHOLDS["worst_fold_coverage_min"] * 100.0,
        color="#d62728",
        linestyle="--",
        label="worst-fold gate",
    )
    axis.set_xticks(x, [f"Fold {fold['fold']}" for fold in folds])
    axis.set_ylim(75.0, 101.0)
    axis.set_ylabel("Held-out point coverage [%]")
    axis.set_title("Blocked chronological cross-validation")
    axis.grid(axis="y", alpha=0.25)
    axis.legend(loc="lower right")
    fig.tight_layout()
    fig.savefig(output, dpi=180, bbox_inches="tight")
    plt.close(fig)


def _markdown(report: Mapping[str, Any]) -> str:
    lines = [
        "# Event-warped morphology profile validation",
        "",
        f"Overall gate: **{'PASS' if report['gate_pass'] else 'FAIL'}**.",
        "",
        "The held-out split uses contiguous chronological blocks; each fold fits both the corridor and the canonical TO phase only on its training blocks.",
        "",
        "## Blocked held-out coverage",
        "",
        "| Fold | Held cycles | Period mean [s] | Duty mean | Knee | Ankle |",
        "|---:|---:|---:|---:|---:|---:|",
    ]
    for fold in report["blocked_cross_validation"]["folds"]:
        lines.append(
            f"| {fold['fold']} | {fold['heldout_cycle_count']} | "
            f"{fold['heldout_period_mean_s']:.3f} | "
            f"{fold['heldout_duty_factor_mean']:.3f} | "
            f"{fold['joints']['knee']['point_coverage']:.2%} | "
            f"{fold['joints']['ankle']['point_coverage']:.2%} |"
        )
    lines.extend(["", "## Aggregate", ""])
    for joint in ("knee", "ankle"):
        item = report["blocked_cross_validation"]["aggregate"][joint]
        dispersion = report["dispersion_comparison"][joint]
        lines.append(
            f"- {joint}: coverage {item['point_coverage']:.3%}; "
            f"stance {item['stance_coverage']:.3%}; swing {item['swing_coverage']:.3%}; "
            f"outside p95 {item['outside_excursion_p95_deg']:.3f} deg; "
            f"mean std reduction {dispersion['mean_std_reduction_fraction']:.2%}."
        )
    lines.extend(["", "## Gates", "", "| Gate | Result |", "|---|---:|"])
    for name, item in report["gates"].items():
        lines.append(f"| `{name}` | {'PASS' if item['pass'] else 'FAIL'} |")
    lines.extend(
        [
            "",
            "## Figures",
            "",
            "- `profile_dispersion_comparison.png`",
            "- `blocked_cross_validation_coverage.png`",
            "",
        ]
    )
    return "\n".join(lines)


def run(args: argparse.Namespace) -> dict[str, Any]:
    profile_path = args.profile.expanduser().resolve()
    legacy_path = args.legacy_profile.expanduser().resolve()
    if not profile_path.is_file() or not legacy_path.is_file():
        raise FileNotFoundError("event and legacy profiles must both exist")
    event_profile = _load_json(profile_path)
    legacy_profile = _load_json(legacy_path)
    rebuilt, rebuild_summary = build_profile(
        ik=args.ik,
        grf=args.grf,
        output=profile_path,
        check=True,
    )
    inputs = load_inputs(args.ik, args.grf, coordinates=DEFAULT_COORDINATES)
    detection = detect_cycles(
        inputs["grf_time"], inputs["vertical_grf_n"], expected_cycles=123
    )
    cycles = detection["production_cycles"]
    cv = _blocked_cross_validation(inputs, cycles, args.folds)
    dispersion = _dispersion_comparison(
        inputs, cycles, event_profile, legacy_profile
    )
    gates = _gates(
        profile_matches=bool(rebuild_summary["matches_existing"]),
        profile=event_profile,
        cv=cv,
        dispersion=dispersion,
    )
    report = {
        "schema_version": 1,
        "validation": "event_warped_morphology_profile_blocked_cv",
        "inputs": {
            "ik": {"path": str(inputs["ik_path"]), "sha256": _sha256(inputs["ik_path"])},
            "grf": {"path": str(inputs["grf_path"]), "sha256": _sha256(inputs["grf_path"])},
            "legacy_profile": {"path": str(legacy_path), "sha256": _sha256(legacy_path)},
            "event_profile": {"path": str(profile_path), "sha256": _sha256(profile_path)},
        },
        "profile": {
            "canonical_to_phase": float(event_profile["metadata"]["canonical_to_phase"]),
            "phase_grid_points": len(event_profile["phase_grid"]),
            "production_cycles": len(cycles),
            "full_span_cycles": len(detection["full_span_cycles"]),
            "excluded_cycles": detection["excluded_cycles"],
            "rebuild_matches_existing": bool(rebuild_summary["matches_existing"]),
            "rebuild_sha256": rebuild_summary["profile_sha256"],
            "rebuilt_object_matches_loaded": rebuilt == event_profile,
        },
        "corridor_parameters": JOINTS,
        "gate_thresholds": GATE_THRESHOLDS,
        "blocked_cross_validation": cv,
        "dispersion_comparison": dispersion,
        "gates": gates,
        "gate_pass": bool(gates["all"]["pass"]),
    }
    output = args.output_dir.expanduser().resolve()
    output.mkdir(parents=True, exist_ok=True)
    _plot_dispersion(
        output / "profile_dispersion_comparison.png", legacy_profile, event_profile
    )
    _plot_fold_coverage(output / "blocked_cross_validation_coverage.png", cv)
    (output / "profile_validation.json").write_text(
        json.dumps(report, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    (output / "profile_validation.md").write_text(
        _markdown(report), encoding="utf-8"
    )
    return report


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ik", default=DEFAULT_IK)
    parser.add_argument("--grf", default=DEFAULT_GRF)
    parser.add_argument("--profile", type=Path, default=DEFAULT_PROFILE)
    parser.add_argument(
        "--legacy-profile", type=Path, default=DEFAULT_LEGACY_PROFILE
    )
    parser.add_argument("--folds", type=int, default=5)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--fail-on-gate", action="store_true")
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    report = run(args)
    print(
        json.dumps(
            {
                "gate_pass": report["gate_pass"],
                "profile": report["inputs"]["event_profile"]["path"],
                "output_dir": str(args.output_dir.expanduser().resolve()),
                "aggregate": report["blocked_cross_validation"]["aggregate"],
            },
            indent=2,
        )
    )
    return 2 if args.fail_on_gate and not report["gate_pass"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
