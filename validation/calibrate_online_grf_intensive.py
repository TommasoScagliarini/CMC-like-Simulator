"""
Two-stage onlineGRF calibration with per-contact material parameters.

Stage one fits vertical GRF and contact timing. Stage two keeps the vertical
fit fixed and fits treadmill surface velocity plus per-contact friction.
Calibration and holdout metrics are always reported separately.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np
from scipy.optimize import differential_evolution

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from online_grf import OnlineGRFProfile, OnlineGRFSphere, load_online_grf_profile
from online_grf import write_online_grf_profile
from path_resolver import resolve_repo_path
from setup_io import read_setup_xml
from validation.validate_online_grf import (
    _calculate_grf,
    _contact_f1,
    _external_grf,
    _metrics,
    _sample_spheres,
)


def _expand_contact_patch(
    profile: OnlineGRFProfile,
    spheres_per_side: int,
) -> OnlineGRFProfile:
    if spheres_per_side <= 2:
        return profile

    expanded: list[OnlineGRFSphere] = []
    for side in ("left", "right"):
        side_spheres = [sphere for sphere in profile.spheres if sphere.side == side]
        if len(side_spheres) != 2 or side_spheres[0].frame != side_spheres[1].frame:
            expanded.extend(side_spheres)
            continue

        start, end = side_spheres
        start_location = np.asarray(start.location, dtype=float)
        end_location = np.asarray(end.location, dtype=float)
        for index, fraction in enumerate(np.linspace(0.0, 1.0, spheres_per_side)):
            if index == 0:
                name = start.name
            elif index == spheres_per_side - 1:
                name = end.name
            else:
                name = f"{side}_patch_{index:02d}"
            expanded.append(
                replace(
                    start,
                    name=name,
                    location=tuple(
                        float(value)
                        for value in (
                            start_location + fraction * (end_location - start_location)
                        )
                    ),
                    radius=float(
                        start.radius + fraction * (end.radius - start.radius)
                    ),
                    material=None,
                )
            )

    if len(expanded) == len(profile.spheres):
        return profile
    return replace(
        profile,
        spheres=tuple(expanded),
        metadata={
            **dict(profile.metadata),
            "contact_patch_spheres_per_side": spheres_per_side,
        },
    )


def _profile_with_vertical_parameters(
    profile: OnlineGRFProfile,
    parameters: np.ndarray,
    side: str | None = None,
) -> OnlineGRFProfile:
    selected = [
        index
        for index, sphere in enumerate(profile.spheres)
        if side is None or sphere.side == side
    ]
    count = len(selected)
    radii = parameters[:count]
    log_stiffness = parameters[count : 2 * count]
    dissipation = parameters[2 * count : 3 * count]
    exponent = parameters[3 * count : 4 * count]
    spheres = list(profile.spheres)
    for parameter_index, sphere_index in enumerate(selected):
        sphere = spheres[sphere_index]
        material = sphere.material or profile.material
        spheres[sphere_index] = replace(
            sphere,
            radius=float(radii[parameter_index]),
            material=replace(
                material,
                stiffness=float(10.0 ** log_stiffness[parameter_index]),
                dissipation=float(dissipation[parameter_index]),
                exponent=float(exponent[parameter_index]),
            ),
        )
    return replace(profile, spheres=tuple(spheres))


def _profile_with_tangent_parameters(
    profile: OnlineGRFProfile,
    parameters: np.ndarray,
) -> OnlineGRFProfile:
    velocity = np.asarray(profile.ground.surface_velocity, dtype=float)
    velocity[0] = float(parameters[0])
    velocity[2] = float(parameters[1])
    friction = parameters[2:]
    spheres = []
    for index, sphere in enumerate(profile.spheres):
        material = sphere.material or profile.material
        spheres.append(
            replace(
                sphere,
                material=replace(
                    material,
                    static_friction=float(friction[index]),
                    dynamic_friction=float(friction[index]),
                ),
            )
        )
    return replace(
        profile,
        ground=replace(
            profile.ground,
            surface_velocity=tuple(float(value) for value in velocity),
        ),
        spheres=tuple(spheres),
    )


def _callback(label: str):
    iteration = 0

    def report(_candidate: np.ndarray, convergence: float) -> bool:
        nonlocal iteration
        iteration += 1
        if iteration == 1 or iteration % 5 == 0:
            print(f"[{label}] iteration={iteration} convergence={convergence:.6g}")
        return False

    return report


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--setup", required=True)
    parser.add_argument("--profile", required=True)
    parser.add_argument("--out-profile", required=True)
    parser.add_argument("--report", default="results/online_grf_calibration_intensive.json")
    parser.add_argument("--sample-dt", type=float, default=0.01)
    parser.add_argument("--calibration-fraction", type=float, default=0.6)
    parser.add_argument("--threshold", type=float, default=20.0)
    parser.add_argument("--spheres-per-side", type=int, default=4)
    parser.add_argument("--vertical-iterations", type=int, default=60)
    parser.add_argument("--tangent-iterations", type=int, default=40)
    parser.add_argument("--population-size", type=int, default=8)
    parser.add_argument(
        "--sea-plugin",
        default="plugins/SEA_Plugin_BlackBox_mCMC_impedence_ff",
    )
    args = parser.parse_args()

    setup = read_setup_xml(args.setup)
    if setup.external_loads_xml is None:
        raise ValueError("Calibration requires prescribed ExternalLoads.")
    source_profile = load_online_grf_profile(args.profile)
    profile = _expand_contact_patch(source_profile, args.spheres_per_side)
    times = np.arange(setup.t_start, setup.t_end + args.sample_dt * 0.25, args.sample_dt)
    split = int(round(len(times) * args.calibration_fraction))
    split = min(max(split, 2), len(times) - 1)
    calibration_mask = np.arange(len(times)) < split
    holdout_mask = ~calibration_mask
    samples = _sample_spheres(setup, profile, times, args.sea_plugin)
    reference = _external_grf(setup, times)
    count = len(profile.spheres)
    vertical_profile = profile
    vertical_results = {}
    for side in ("left", "right"):
        side_spheres = [sphere for sphere in vertical_profile.spheres if sphere.side == side]
        side_count = len(side_spheres)
        vertical_scale = max(
            1.0,
            float(np.max(reference[side][calibration_mask, 1])),
        )

        def vertical_objective(candidate: np.ndarray) -> float:
            candidate_profile = _profile_with_vertical_parameters(
                vertical_profile,
                candidate,
                side,
            )
            predicted = _calculate_grf(candidate_profile, samples)
            error = (
                predicted[side][calibration_mask, 1]
                - reference[side][calibration_mask, 1]
            ) / vertical_scale
            loss = float(np.mean(error * error))
            loss += 0.20 * (
                1.0
                - _contact_f1(
                    reference[side][calibration_mask],
                    predicted[side][calibration_mask],
                    args.threshold,
                )
            )
            return loss

        radius_bounds = []
        for sphere in side_spheres:
            radius_bounds.append(
                (
                    max(0.004, sphere.radius - 0.035),
                    min(0.12, sphere.radius + 0.070),
                )
            )
        vertical_seed = np.concatenate(
            [
                np.asarray([sphere.radius for sphere in side_spheres], dtype=float),
                np.asarray(
                    [
                        math.log10(
                            (sphere.material or vertical_profile.material).stiffness
                        )
                        for sphere in side_spheres
                    ],
                    dtype=float,
                ),
                np.asarray(
                    [
                        (sphere.material or vertical_profile.material).dissipation
                        for sphere in side_spheres
                    ],
                    dtype=float,
                ),
                np.asarray(
                    [
                        (sphere.material or vertical_profile.material).exponent
                        for sphere in side_spheres
                    ],
                    dtype=float,
                ),
            ]
        )
        result = differential_evolution(
            vertical_objective,
            bounds=(
                radius_bounds
                + [(1.0, 8.0)] * side_count
                + [(0.0, 5.0)] * side_count
                + [(0.5, 3.0)] * side_count
            ),
            maxiter=args.vertical_iterations,
            popsize=args.population_size,
            seed=20260607 if side == "left" else 20260608,
            polish=True,
            updating="immediate",
            callback=_callback(f"vertical-{side}"),
            x0=vertical_seed,
        )
        vertical_results[side] = result
        vertical_profile = _profile_with_vertical_parameters(
            vertical_profile,
            result.x,
            side,
        )

    tangent_scales = {}
    for side in ("left", "right"):
        peak_vertical = float(np.max(reference[side][calibration_mask, 1]))
        tangent_scales[side] = np.maximum(
            np.max(np.abs(reference[side][calibration_mask][:, [0, 2]]), axis=0),
            max(10.0, 0.05 * peak_vertical),
        )

    def tangent_objective(candidate: np.ndarray) -> float:
        candidate_profile = _profile_with_tangent_parameters(vertical_profile, candidate)
        predicted = _calculate_grf(candidate_profile, samples)
        loss = 0.0
        for side in ("left", "right"):
            error = (
                predicted[side][calibration_mask][:, [0, 2]]
                - reference[side][calibration_mask][:, [0, 2]]
            ) / tangent_scales[side]
            loss += float(np.mean(error * error))
        return loss

    tangent_seed = np.concatenate(
        [
            np.asarray(
                [
                    vertical_profile.ground.surface_velocity[0],
                    vertical_profile.ground.surface_velocity[2],
                ],
                dtype=float,
            ),
            np.asarray(
                [
                    (sphere.material or vertical_profile.material).static_friction
                    for sphere in vertical_profile.spheres
                ],
                dtype=float,
            ),
        ]
    )
    tangent_result = differential_evolution(
        tangent_objective,
        bounds=[(-3.0, 3.0), (-1.0, 1.0)] + [(0.0, 2.0)] * count,
        maxiter=args.tangent_iterations,
        popsize=args.population_size,
        seed=20260608,
        polish=True,
        updating="immediate",
        callback=_callback("tangent"),
        x0=tangent_seed,
    )
    calibrated = _profile_with_tangent_parameters(vertical_profile, tangent_result.x)
    calibrated = replace(
        calibrated,
        source="intensively_calibrated_against_prescribed_grf",
        metadata={
            **dict(calibrated.metadata),
            "status": "calibrated",
            "calibration_sample_dt": float(args.sample_dt),
            "calibration_fraction": float(args.calibration_fraction),
            "vertical_objective": float(
                sum(result.fun for result in vertical_results.values())
            ),
            "tangent_objective": float(tangent_result.fun),
        },
    )
    predicted = _calculate_grf(calibrated, samples)
    report = {
        "setup": str(Path(args.setup).resolve()),
        "input_profile": str(Path(args.profile).resolve()),
        "samples": int(len(times)),
        "contacts": int(count),
        "sample_dt": float(args.sample_dt),
        "calibration_fraction": float(args.calibration_fraction),
        "calibration_time_range": [float(times[0]), float(times[split - 1])],
        "holdout_time_range": [float(times[split]), float(times[-1])],
        "vertical_optimization": {
            side: {
                "success": bool(result.success),
                "message": str(result.message),
                "objective": float(result.fun),
                "iterations": int(result.nit),
            }
            for side, result in vertical_results.items()
        },
        "tangent_optimization": {
            "success": bool(tangent_result.success),
            "message": str(tangent_result.message),
            "objective": float(tangent_result.fun),
            "iterations": int(tangent_result.nit),
        },
        "calibration_metrics": _metrics(
            reference,
            predicted,
            times,
            calibration_mask,
            args.threshold,
        ),
        "holdout_metrics": _metrics(
            reference,
            predicted,
            times,
            holdout_mask,
            args.threshold,
        ),
        "calibrated_profile": calibrated.to_dict(),
    }
    report_path = resolve_repo_path(args.report)
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    destination = write_online_grf_profile(calibrated, resolve_repo_path(args.out_profile))
    print(f"onlineGRF calibrated profile: {destination}")
    print(f"onlineGRF calibration report: {report_path.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
