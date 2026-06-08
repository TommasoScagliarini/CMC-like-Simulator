"""Calibrate a sparse state-based onlineGRF contact patch."""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np
from scipy.optimize import differential_evolution, lsq_linear, nnls

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from online_grf import OnlineGRFProfile, OnlineGRFSphere, load_online_grf_profile
from online_grf import write_online_grf_profile
from path_resolver import resolve_repo_path
from setup_io import read_setup_xml
from validation.validate_online_grf import (
    _calculate_grf,
    _external_grf,
    _metrics,
    _sample_spheres,
    _sample_spheres_from_coordinate_states,
    _smooth_positive,
)


def _candidate_patch(
    profile: OnlineGRFProfile,
    samples: dict,
    side: str,
    location_count: int,
    radius_count: int,
) -> list[dict]:
    endpoints = [sphere for sphere in profile.spheres if sphere.side == side]
    if len(endpoints) < 2:
        raise ValueError(f"Basis calibration requires two endpoint spheres for {side}.")
    start, end = endpoints[0], endpoints[-1]
    if start.frame != end.frame:
        raise ValueError(f"Basis calibration requires one contact frame for {side}.")

    start_location = np.asarray(start.location, dtype=float)
    end_location = np.asarray(end.location, dtype=float)
    start_center = samples["centers"][start.name]
    end_center = samples["centers"][end.name]
    start_velocity = samples["velocities"][start.name]
    end_velocity = samples["velocities"][end.name]
    normal = np.asarray(profile.ground.normal, dtype=float)
    normal /= np.linalg.norm(normal)
    origin = np.asarray(profile.ground.origin, dtype=float)
    epsilon = profile.material.smoothing

    candidates = []
    for fraction in np.linspace(-0.15, 1.15, location_count):
        location = start_location + fraction * (end_location - start_location)
        center = start_center + fraction * (end_center - start_center)
        velocity = start_velocity + fraction * (end_velocity - start_velocity)
        distance = (center - origin) @ normal
        normal_velocity = velocity @ normal
        for radius in np.linspace(0.006, 0.080, radius_count):
            penetration = _smooth_positive(radius - distance, epsilon)
            for exponent in (0.8, 1.2, 1.6, 2.0):
                base = np.power(penetration, exponent)
                for dissipation in (0.0, 1.0):
                    damping = _smooth_positive(
                        1.0 - dissipation * normal_velocity,
                        epsilon,
                    )
                    candidates.append(
                        {
                            "location": tuple(float(value) for value in location),
                            "center": center,
                            "velocity": velocity,
                            "radius": float(radius),
                            "exponent": float(exponent),
                            "dissipation": float(dissipation),
                            "basis": base * damping,
                        }
                    )
    return candidates


def _fit_sparse_patch(
    candidates: list[dict],
    target: np.ndarray,
    calibration_mask: np.ndarray,
    max_contacts: int,
) -> list[dict]:
    matrix = np.column_stack([candidate["basis"] for candidate in candidates])
    calibration = matrix[calibration_mask]
    norms = np.linalg.norm(calibration, axis=0)
    usable = norms > 1.0e-14
    if not np.any(usable):
        raise RuntimeError("No candidate onlineGRF contact reaches the ground.")
    normalized = calibration[:, usable] / norms[usable]
    coefficients, _ = nnls(normalized, target[calibration_mask], maxiter=20000)
    usable_indices = np.flatnonzero(usable)
    selected = usable_indices[np.argsort(coefficients)[-max_contacts:]]
    selected_matrix = calibration[:, selected]
    selected_norms = np.linalg.norm(selected_matrix, axis=0)
    normalized_selected = selected_matrix / selected_norms
    selected_coefficients, _ = nnls(
        normalized_selected,
        target[calibration_mask],
        maxiter=20000,
    )
    stiffness = selected_coefficients / selected_norms
    result = []
    for candidate_index, value in zip(selected, stiffness):
        if value <= 1.0e-8:
            continue
        item = dict(candidates[int(candidate_index)])
        item["stiffness"] = float(value)
        result.append(item)
    return result


def _profile_and_samples(
    source: OnlineGRFProfile,
    selected: dict[str, list[dict]],
) -> tuple[OnlineGRFProfile, dict]:
    spheres = []
    samples = {"centers": {}, "velocities": {}}
    for side in ("left", "right"):
        frame = next(sphere.frame for sphere in source.spheres if sphere.side == side)
        for index, item in enumerate(selected[side]):
            name = f"{side}_basis_{index:02d}"
            spheres.append(
                OnlineGRFSphere(
                    name=name,
                    side=side,
                    frame=frame,
                    location=item["location"],
                    radius=item["radius"],
                    material=replace(
                        source.material,
                        stiffness=item["stiffness"],
                        exponent=item["exponent"],
                        dissipation=item["dissipation"],
                        static_friction=0.0,
                        dynamic_friction=0.0,
                    ),
                )
            )
            samples["centers"][name] = item["center"]
            samples["velocities"][name] = item["velocity"]
    profile = replace(
        source,
        source="sparse_basis_calibrated_against_prescribed_grf",
        spheres=tuple(spheres),
        metadata={
            **dict(source.metadata),
            "status": "calibrated",
            "calibration_strategy": "sparse_nonnegative_contact_basis",
        },
    )
    return profile, samples


def _fit_tangent(
    profile: OnlineGRFProfile,
    samples: dict,
    reference: dict,
    calibration_mask: np.ndarray,
) -> tuple[OnlineGRFProfile, dict]:
    normal = np.asarray(profile.ground.normal, dtype=float)
    normal /= np.linalg.norm(normal)
    normal_profile = replace(
        profile,
        ground=replace(profile.ground, surface_velocity=(0.0, 0.0, 0.0)),
    )
    predicted_normal = _calculate_grf(normal_profile, samples)

    def solve(surface_xz: np.ndarray) -> tuple[float, dict[str, np.ndarray]]:
        surface = np.asarray([surface_xz[0], 0.0, surface_xz[1]], dtype=float)
        coefficients = {}
        total = 0.0
        for side in ("left", "right"):
            columns = []
            side_spheres = [sphere for sphere in profile.spheres if sphere.side == side]
            for sphere in side_spheres:
                velocity = samples["velocities"][sphere.name] - surface
                normal_velocity = velocity @ normal
                tangent = velocity - normal_velocity[:, None] * normal
                slip = np.linalg.norm(tangent, axis=1)
                transition = (sphere.material or profile.material).transition_velocity
                normal_force = (
                    _calculate_grf(
                        replace(profile, spheres=(sphere,)),
                        {
                            "centers": {sphere.name: samples["centers"][sphere.name]},
                            "velocities": {sphere.name: samples["velocities"][sphere.name]},
                        },
                    )[side][:, 1]
                )
                unit_friction = (
                    -normal_force[:, None]
                    * tangent
                    / np.sqrt(slip * slip + transition * transition)[:, None]
                )
                columns.append(unit_friction[:, [0, 2]].reshape(-1))
            matrix = np.column_stack(columns)
            target = reference[side][:, [0, 2]].reshape(-1)
            row_mask = np.repeat(calibration_mask, 2)
            fit = lsq_linear(
                matrix[row_mask],
                target[row_mask],
                bounds=(0.0, 2.0),
                lsmr_tol="auto",
            )
            coefficients[side] = fit.x
            error = matrix[row_mask] @ fit.x - target[row_mask]
            scale = max(10.0, float(np.max(np.abs(target[row_mask]))))
            total += float(np.mean((error / scale) ** 2))
        return total, coefficients

    result = differential_evolution(
        lambda candidate: solve(candidate)[0],
        bounds=[(-3.0, 3.0), (-1.0, 1.0)],
        maxiter=40,
        popsize=10,
        seed=20260609,
        polish=True,
    )
    _, coefficients = solve(result.x)
    spheres = []
    side_indices = {"left": 0, "right": 0}
    for sphere in profile.spheres:
        index = side_indices[sphere.side]
        side_indices[sphere.side] += 1
        mu = float(coefficients[sphere.side][index])
        spheres.append(
            replace(
                sphere,
                material=replace(
                    sphere.material or profile.material,
                    static_friction=mu,
                    dynamic_friction=mu,
                ),
            )
        )
    calibrated = replace(
        profile,
        ground=replace(
            profile.ground,
            surface_velocity=(float(result.x[0]), 0.0, float(result.x[1])),
        ),
        spheres=tuple(spheres),
    )
    return calibrated, {
        "objective": float(result.fun),
        "surface_velocity_xz": result.x.tolist(),
        "normal_force_peak": {
            side: float(np.max(predicted_normal[side][:, 1]))
            for side in ("left", "right")
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--setup", required=True)
    parser.add_argument("--profile", required=True)
    parser.add_argument("--out-profile", required=True)
    parser.add_argument("--report", default="results/online_grf_calibration_basis.json")
    parser.add_argument("--sample-dt", type=float, default=0.005)
    parser.add_argument(
        "--states-sto",
        default="",
        help="Calibrate from saved simulator CoordinateStates instead of IK replay.",
    )
    parser.add_argument("--calibration-fraction", type=float, default=0.6)
    parser.add_argument("--threshold", type=float, default=20.0)
    parser.add_argument("--location-count", type=int, default=10)
    parser.add_argument("--radius-count", type=int, default=10)
    parser.add_argument("--max-contacts-per-side", type=int, default=12)
    parser.add_argument(
        "--sea-plugin",
        default="plugins/SEA_Plugin_BlackBox_mCMC_impedence_ff",
    )
    args = parser.parse_args()

    setup = read_setup_xml(args.setup)
    if setup.external_loads_xml is None:
        raise ValueError("Calibration requires prescribed ExternalLoads.")
    source = load_online_grf_profile(args.profile)
    if args.states_sto:
        times, endpoint_samples = _sample_spheres_from_coordinate_states(
            setup,
            source,
            args.states_sto,
            args.sea_plugin,
        )
    else:
        times = np.arange(
            setup.t_start,
            setup.t_end + args.sample_dt * 0.25,
            args.sample_dt,
        )
        endpoint_samples = _sample_spheres(setup, source, times, args.sea_plugin)
    split = int(round(len(times) * args.calibration_fraction))
    split = min(max(split, 2), len(times) - 1)
    calibration_mask = np.arange(len(times)) < split
    holdout_mask = ~calibration_mask
    reference = _external_grf(setup, times)

    selected = {}
    candidate_counts = {}
    for side in ("left", "right"):
        candidates = _candidate_patch(
            source,
            endpoint_samples,
            side,
            args.location_count,
            args.radius_count,
        )
        candidate_counts[side] = len(candidates)
        selected[side] = _fit_sparse_patch(
            candidates,
            reference[side][:, 1],
            calibration_mask,
            args.max_contacts_per_side,
        )
        print(
            f"[basis] {side}: selected {len(selected[side])}/{len(candidates)} contacts"
        )

    profile, samples = _profile_and_samples(source, selected)
    profile = replace(
        profile,
        metadata={
            **dict(profile.metadata),
            "calibration_input": (
                "saved_forward_coordinate_states" if args.states_sto else "ik_replay"
            ),
        },
    )
    profile, tangent_report = _fit_tangent(
        profile,
        samples,
        reference,
        calibration_mask,
    )
    predicted = _calculate_grf(profile, samples)
    report = {
        "setup": str(Path(args.setup).resolve()),
        "input_profile": str(Path(args.profile).resolve()),
        "samples": int(len(times)),
        "sample_dt": (
            float(np.median(np.diff(times))) if len(times) > 1 else float(args.sample_dt)
        ),
        "calibration_time_range": [float(times[0]), float(times[split - 1])],
        "holdout_time_range": [float(times[split]), float(times[-1])],
        "states_sto": (
            str(resolve_repo_path(args.states_sto).resolve())
            if args.states_sto
            else ""
        ),
        "candidate_counts": candidate_counts,
        "selected_counts": {
            side: len(selected[side]) for side in ("left", "right")
        },
        "tangent_optimization": tangent_report,
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
        "calibrated_profile": profile.to_dict(),
    }
    destination = write_online_grf_profile(profile, resolve_repo_path(args.out_profile))
    report_path = resolve_repo_path(args.report)
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(f"onlineGRF calibrated profile: {destination}")
    print(f"onlineGRF calibration report: {report_path.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
