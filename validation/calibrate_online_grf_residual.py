"""Fit a bounded state-only full-wrench residual for an onlineGRF profile."""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np
from scipy.optimize import lsq_linear

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from online_grf import (
    OnlineGRFProfile,
    load_online_grf_profile,
    write_online_grf_profile,
)
from path_resolver import resolve_repo_path
from setup_io import read_setup_xml
from validation.calibrate_online_grf_basis import (
    _sample_spheres_from_coordinate_states,
)
from validation.validate_online_grf import (
    _calculate_wrench,
    _external_wrench,
    _wrench_metrics,
)


def _with_side_residual(
    profile: OnlineGRFProfile,
    side: str,
    parameters: np.ndarray,
) -> OnlineGRFProfile:
    force_ratio = tuple(float(value) for value in parameters[:3])
    moment_ratio = tuple(float(value) for value in parameters[3:])
    return replace(
        profile,
        spheres=tuple(
            replace(
                sphere,
                residual_force_ratio=force_ratio,
                residual_moment_ratio_m=moment_ratio,
            )
            if sphere.side == side
            else sphere
            for sphere in profile.spheres
        ),
    )


def _fit_side(
    profile: OnlineGRFProfile,
    samples: dict,
    reference: dict,
    side: str,
    calibration_mask: np.ndarray,
    moment_weight: float,
    force_ratio_bound: float,
    moment_ratio_bound_m: float,
    lock_vertical_force: bool,
) -> tuple[np.ndarray, dict]:
    zero = np.zeros(6)
    baseline = _calculate_wrench(_with_side_residual(profile, side, zero), samples)
    target_force = reference[side]["force"] - baseline[side]["force"]
    target_moment = reference[side]["moment"] - baseline[side]["moment"]
    force_scale = np.maximum(
        np.max(np.abs(reference[side]["force"][calibration_mask]), axis=0),
        1.0,
    )
    moment_scale = np.maximum(
        np.max(np.abs(reference[side]["moment"][calibration_mask]), axis=0),
        1.0,
    )

    columns = []
    for index in range(6):
        unit = np.zeros(6)
        unit[index] = 1.0
        predicted = _calculate_wrench(
            _with_side_residual(profile, side, unit),
            samples,
        )
        columns.append(
            (
                predicted[side]["force"] - baseline[side]["force"],
                predicted[side]["moment"] - baseline[side]["moment"],
            )
        )

    def stack(force: np.ndarray, moment: np.ndarray) -> np.ndarray:
        return np.concatenate(
            [
                (force[calibration_mask] / force_scale).reshape(-1),
                (
                    moment_weight
                    * moment[calibration_mask]
                    / moment_scale
                ).reshape(-1),
            ]
        )

    active_parameters = [0, 2, 3, 4, 5] if lock_vertical_force else list(range(6))
    design = np.column_stack(
        [stack(*columns[index]) for index in active_parameters]
    )
    target = stack(target_force, target_moment)
    full_lower = np.array([-force_ratio_bound] * 3 + [-moment_ratio_bound_m] * 3)
    full_upper = np.array([force_ratio_bound] * 3 + [moment_ratio_bound_m] * 3)
    bounds = (full_lower[active_parameters], full_upper[active_parameters])
    fit = lsq_linear(design, target, bounds=bounds, lsmr_tol="auto")
    parameters = np.zeros(6)
    parameters[active_parameters] = np.asarray(fit.x, dtype=float)
    return parameters, {
        "success": bool(fit.success),
        "status": int(fit.status),
        "cost": float(fit.cost),
        "optimality": float(fit.optimality),
        "force_ratio": parameters[:3].tolist(),
        "moment_ratio_m": parameters[3:].tolist(),
        "vertical_force_locked": bool(lock_vertical_force),
    }


def _with_side_vertical_state_gains(
    profile: OnlineGRFProfile,
    samples: dict,
    side: str,
    penetration_gain_per_m: float,
    penetration_rate_gain_s_per_m: float,
) -> OnlineGRFProfile:
    normal = np.asarray(profile.ground.normal, dtype=float)
    normal /= np.linalg.norm(normal)
    origin = np.asarray(profile.ground.origin, dtype=float)
    spheres = []
    for sphere in profile.spheres:
        if sphere.side != side:
            spheres.append(sphere)
            continue
        center = samples["centers"][sphere.name][0]
        reference = max(0.0, sphere.radius - float((center - origin) @ normal))
        spheres.append(
            replace(
                sphere,
                residual_penetration_reference_m=reference,
                residual_force_penetration_gain_per_m=(
                    0.0,
                    float(penetration_gain_per_m),
                    0.0,
                ),
                residual_force_penetration_rate_gain_s_per_m=(
                    0.0,
                    float(penetration_rate_gain_s_per_m),
                    0.0,
                ),
            )
        )
    return replace(profile, spheres=tuple(spheres))


def _fit_vertical_state_gains(
    profile: OnlineGRFProfile,
    samples: dict,
    reference: dict,
    side: str,
    calibration_mask: np.ndarray,
    moment_weight: float,
    penetration_gain_bound_per_m: float,
    rate_gain_bound_s_per_m: float,
    gain_scale: float,
) -> tuple[OnlineGRFProfile, dict]:
    baseline_profile = _with_side_vertical_state_gains(profile, samples, side, 0.0, 0.0)
    baseline = _calculate_wrench(baseline_profile, samples)
    target_force = reference[side]["force"] - baseline[side]["force"]
    target_moment = reference[side]["moment"] - baseline[side]["moment"]
    force_scale = np.maximum(
        np.max(np.abs(reference[side]["force"][calibration_mask]), axis=0),
        1.0,
    )
    moment_scale = np.maximum(
        np.max(np.abs(reference[side]["moment"][calibration_mask]), axis=0),
        1.0,
    )

    columns = []
    for penetration_gain, rate_gain in ((1.0, 0.0), (0.0, 1.0)):
        candidate = _calculate_wrench(
            _with_side_vertical_state_gains(
                profile,
                samples,
                side,
                penetration_gain,
                rate_gain,
            ),
            samples,
        )
        columns.append(
            (
                candidate[side]["force"] - baseline[side]["force"],
                candidate[side]["moment"] - baseline[side]["moment"],
            )
        )

    def stack(force: np.ndarray, moment: np.ndarray) -> np.ndarray:
        return np.concatenate(
            [
                (force[calibration_mask] / force_scale).reshape(-1),
                (
                    moment_weight
                    * moment[calibration_mask]
                    / moment_scale
                ).reshape(-1),
            ]
        )

    design = np.column_stack([stack(*column) for column in columns])
    target = stack(target_force, target_moment)
    fit = lsq_linear(
        design,
        target,
        bounds=(
            [-penetration_gain_bound_per_m, -rate_gain_bound_s_per_m],
            [penetration_gain_bound_per_m, rate_gain_bound_s_per_m],
        ),
        lsmr_tol="auto",
    )
    unscaled_gains = np.asarray(fit.x, dtype=float)
    gains = gain_scale * unscaled_gains
    fitted_profile = _with_side_vertical_state_gains(
        profile,
        samples,
        side,
        gains[0],
        gains[1],
    )
    return fitted_profile, {
        "success": bool(fit.success),
        "status": int(fit.status),
        "cost": float(fit.cost),
        "optimality": float(fit.optimality),
        "penetration_gain_per_m": float(gains[0]),
        "penetration_rate_gain_s_per_m": float(gains[1]),
        "unscaled_penetration_gain_per_m": float(unscaled_gains[0]),
        "unscaled_penetration_rate_gain_s_per_m": float(unscaled_gains[1]),
        "gain_scale": float(gain_scale),
        "zero_at_initial_state": True,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--setup", required=True)
    parser.add_argument("--profile", required=True)
    parser.add_argument("--states-sto", required=True)
    parser.add_argument("--out-profile", required=True)
    parser.add_argument(
        "--report",
        default="results/online_grf_residual_calibration.json",
    )
    parser.add_argument("--calibration-fraction", type=float, default=0.8)
    parser.add_argument("--moment-weight", type=float, default=0.3)
    parser.add_argument("--force-ratio-bound", type=float, default=0.5)
    parser.add_argument("--moment-ratio-bound-m", type=float, default=0.3)
    parser.add_argument(
        "--lock-vertical-force",
        action="store_true",
        help="Fit only tangential force and free moment; keep residual Fy at zero.",
    )
    parser.add_argument("--fit-vertical-state-gains", action="store_true")
    parser.add_argument("--penetration-gain-bound-per-m", type=float, default=100.0)
    parser.add_argument("--rate-gain-bound-s-per-m", type=float, default=2.0)
    parser.add_argument("--vertical-state-gain-scale", type=float, default=1.0)
    parser.add_argument("--threshold", type=float, default=20.0)
    parser.add_argument(
        "--sea-plugin",
        default="plugins/SEA_Plugin_BlackBox_mCMC_impedence_ff",
    )
    args = parser.parse_args()

    if not 0.0 < args.calibration_fraction < 1.0:
        raise ValueError("--calibration-fraction must be between 0 and 1.")
    if args.moment_weight < 0.0:
        raise ValueError("--moment-weight must be non-negative.")
    if args.force_ratio_bound <= 0.0 or args.moment_ratio_bound_m <= 0.0:
        raise ValueError("Residual bounds must be positive.")
    if (
        args.penetration_gain_bound_per_m <= 0.0
        or args.rate_gain_bound_s_per_m <= 0.0
    ):
        raise ValueError("Vertical state-gain bounds must be positive.")
    if not 0.0 < args.vertical_state_gain_scale <= 1.0:
        raise ValueError("--vertical-state-gain-scale must be in (0, 1].")

    setup = read_setup_xml(args.setup)
    profile = load_online_grf_profile(args.profile)
    times, samples = _sample_spheres_from_coordinate_states(
        setup,
        profile,
        args.states_sto,
        args.sea_plugin,
    )
    split = min(
        max(int(round(len(times) * args.calibration_fraction)), 2),
        len(times) - 1,
    )
    calibration_mask = np.arange(len(times)) < split
    holdout_mask = ~calibration_mask
    reference = _external_wrench(setup, times)

    fits = {}
    calibrated = profile
    for side in ("left", "right"):
        parameters, fits[side] = _fit_side(
            profile,
            samples,
            reference,
            side,
            calibration_mask,
            args.moment_weight,
            args.force_ratio_bound,
            args.moment_ratio_bound_m,
            args.lock_vertical_force,
        )
        calibrated = _with_side_residual(calibrated, side, parameters)

    vertical_state_fits = {}
    if args.fit_vertical_state_gains:
        for side in ("left", "right"):
            calibrated, vertical_state_fits[side] = _fit_vertical_state_gains(
                calibrated,
                samples,
                reference,
                side,
                calibration_mask,
                args.moment_weight,
                args.penetration_gain_bound_per_m,
                args.rate_gain_bound_s_per_m,
                args.vertical_state_gain_scale,
            )

    calibrated = replace(
        calibrated,
        source="calibrated_state_only_full_wrench_residual",
        metadata={
            **dict(calibrated.metadata),
            "residual_model": {
                "type": "normal_force_scaled_full_wrench",
                "runtime_inputs": (
                    [
                        "instantaneous_normal_force",
                        "penetration",
                        "penetration_rate",
                    ]
                    if args.fit_vertical_state_gains
                    else ["instantaneous_normal_force"]
                ),
                "uses_time_or_prescribed_at_runtime": False,
                "calibration_states": str(resolve_repo_path(args.states_sto).resolve()),
                "fits": fits,
                "vertical_state_fits": vertical_state_fits,
            },
            "online_mode_status": "requires_forward_validation",
        },
    )
    baseline = _calculate_wrench(profile, samples)
    predicted = _calculate_wrench(calibrated, samples)
    report = {
        "setup": str(resolve_repo_path(args.setup).resolve()),
        "input_profile": str(resolve_repo_path(args.profile).resolve()),
        "states_sto": str(resolve_repo_path(args.states_sto).resolve()),
        "samples": int(len(times)),
        "calibration_time_range": [float(times[0]), float(times[split - 1])],
        "holdout_time_range": [float(times[split]), float(times[-1])],
        "fits": fits,
        "vertical_state_fits": vertical_state_fits,
        "baseline_calibration_wrench_metrics": _wrench_metrics(
            reference, baseline, times, calibration_mask, args.threshold
        ),
        "residual_calibration_wrench_metrics": _wrench_metrics(
            reference, predicted, times, calibration_mask, args.threshold
        ),
        "baseline_holdout_wrench_metrics": _wrench_metrics(
            reference, baseline, times, holdout_mask, args.threshold
        ),
        "residual_holdout_wrench_metrics": _wrench_metrics(
            reference, predicted, times, holdout_mask, args.threshold
        ),
        "calibrated_profile": calibrated.to_dict(),
    }
    destination = write_online_grf_profile(
        calibrated,
        resolve_repo_path(args.out_profile),
    )
    report_path = resolve_repo_path(args.report)
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(f"onlineGRF residual profile: {destination}")
    print(f"onlineGRF residual report: {report_path.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
