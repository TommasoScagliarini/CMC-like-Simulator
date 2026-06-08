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
    _calculate_wrench,
    _external_grf,
    _external_wrench,
    _metrics,
    _physical_metrics,
    _sample_spheres,
    _sample_spheres_from_coordinate_states,
    _smooth_positive,
    _wrench_metrics,
)


def _candidate_sampling_profile(
    profile: OnlineGRFProfile,
    location_count: int,
    lateral_count: int,
    lateral_half_width: float,
) -> OnlineGRFProfile:
    spheres = []
    for side in ("left", "right"):
        endpoints = [sphere for sphere in profile.spheres if sphere.side == side]
        if len(endpoints) < 2:
            raise ValueError(f"Basis calibration requires two endpoint spheres for {side}.")
        start, end = endpoints[0], endpoints[-1]
        if start.frame != end.frame:
            raise ValueError(f"Basis calibration requires one contact frame for {side}.")
        start_location = np.asarray(start.location, dtype=float)
        end_location = np.asarray(end.location, dtype=float)
        for longitudinal_index, fraction in enumerate(
            np.linspace(-0.15, 1.15, location_count)
        ):
            centerline = start_location + fraction * (end_location - start_location)
            for lateral_index, lateral_offset in enumerate(
                np.linspace(-lateral_half_width, lateral_half_width, lateral_count)
            ):
                location = centerline.copy()
                location[2] += lateral_offset
                spheres.append(
                    OnlineGRFSphere(
                        name=(
                            f"{side}_candidate_location_"
                            f"{longitudinal_index:02d}_{lateral_index:02d}"
                        ),
                        side=side,
                        frame=start.frame,
                        location=tuple(float(value) for value in location),
                        radius=start.radius,
                        material=start.material,
                    )
                )
    return replace(profile, spheres=tuple(spheres))


def _candidate_patch(
    profile: OnlineGRFProfile,
    samples: dict,
    side: str,
    location_count: int,
    radius_count: int,
    maximum_penetration: float | None = None,
) -> list[dict]:
    normal = np.asarray(profile.ground.normal, dtype=float)
    normal /= np.linalg.norm(normal)
    origin = np.asarray(profile.ground.origin, dtype=float)
    epsilon = profile.material.smoothing

    candidates = []
    location_spheres = [sphere for sphere in profile.spheres if sphere.side == side]
    if len(location_spheres) != location_count:
        raise ValueError(
            f"Expected {location_count} sampled locations for {side}, "
            f"found {len(location_spheres)}."
        )
    for location_sphere in location_spheres:
        location = np.asarray(location_sphere.location, dtype=float)
        center = samples["centers"][location_sphere.name]
        velocity = samples["velocities"][location_sphere.name]
        distance = (center - origin) @ normal
        normal_velocity = velocity @ normal
        for radius in np.linspace(0.006, 0.080, radius_count):
            penetration_raw = radius - distance
            penetration_max = float(np.max(np.maximum(0.0, penetration_raw)))
            if (
                maximum_penetration is not None
                and penetration_max > maximum_penetration + 1.0e-12
            ):
                continue
            if penetration_max <= 0.0:
                continue
            penetration = _smooth_positive(penetration_raw, epsilon)
            exponents = (
                (1.0, 1.5, 2.0)
                if maximum_penetration is not None
                else (0.8, 1.2, 1.6, 2.0)
            )
            for exponent in exponents:
                base = np.power(penetration, exponent)
                for dissipation in (0.0, 1.0, 3.0):
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
                            "penetration_max_m": penetration_max,
                            "basis": base * damping,
                            "contact_point": center - radius * normal,
                        }
                    )
    return candidates


def _weighted_wrench_vector(
    force: np.ndarray,
    moment: np.ndarray,
    force_scale: float,
    moment_scale: float,
    moment_weight: float,
) -> np.ndarray:
    return np.column_stack(
        [
            force / force_scale,
            moment_weight * moment / moment_scale,
        ]
    ).reshape(-1)


def _fit_sparse_patch(
    candidates: list[dict],
    target: dict[str, np.ndarray],
    calibration_mask: np.ndarray,
    max_contacts: int,
    normal: np.ndarray,
    max_stiffness: float | None = None,
    moment_weight: float = 1.0,
) -> list[dict]:
    target_force = target["force"][calibration_mask]
    target_moment = target["moment"][calibration_mask]
    force_scale = max(1.0, float(np.max(np.abs(target_force))))
    moment_scale = max(1.0, float(np.max(np.abs(target_moment))))
    normal = np.asarray(normal, dtype=float)
    normal /= np.linalg.norm(normal)
    columns = []
    for candidate in candidates:
        force = candidate["basis"][:, None] * normal
        moment = np.cross(candidate["contact_point"], force)
        columns.append(
            _weighted_wrench_vector(
                force[calibration_mask],
                moment[calibration_mask],
                force_scale,
                moment_scale,
                moment_weight,
            )
        )
    calibration = np.column_stack(columns)
    target_vector = _weighted_wrench_vector(
        target_force,
        target_moment,
        force_scale,
        moment_scale,
        moment_weight,
    )
    norms = np.linalg.norm(calibration, axis=0)
    usable = norms > 1.0e-14
    if not np.any(usable):
        raise RuntimeError("No candidate onlineGRF contact reaches the ground.")
    normalized = calibration[:, usable] / norms[usable]
    coefficients, _ = nnls(normalized, target_vector, maxiter=20000)
    usable_indices = np.flatnonzero(usable)
    selected = usable_indices[np.argsort(coefficients)[-max_contacts:]]
    selected_matrix = calibration[:, selected]
    selected_fit = lsq_linear(
        selected_matrix,
        target_vector,
        bounds=(0.0, max_stiffness if max_stiffness is not None else np.inf),
        lsmr_tol="auto",
    )
    stiffness = selected_fit.x
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


def _bounded_ground(
    profile: OnlineGRFProfile,
    samples: dict,
    maximum_penetration: float,
) -> tuple[OnlineGRFProfile, float]:
    normal = np.asarray(profile.ground.normal, dtype=float)
    normal /= np.linalg.norm(normal)
    origin = np.asarray(profile.ground.origin, dtype=float)
    current_maximum = 0.0
    for sphere in profile.spheres:
        distance = (samples["centers"][sphere.name] - origin) @ normal
        current_maximum = max(
            current_maximum,
            float(np.max(np.maximum(0.0, sphere.radius - distance))),
        )
    offset = float(maximum_penetration - current_maximum)
    return (
        replace(
            profile,
            ground=replace(
                profile.ground,
                origin=tuple(float(value) for value in origin + offset * normal),
            ),
        ),
        offset,
    )


def _fit_tangent(
    profile: OnlineGRFProfile,
    samples: dict,
    reference: dict,
    calibration_mask: np.ndarray,
    moment_weight: float,
) -> tuple[OnlineGRFProfile, dict]:
    normal = np.asarray(profile.ground.normal, dtype=float)
    normal /= np.linalg.norm(normal)
    normal_profile = replace(
        profile,
        ground=replace(profile.ground, surface_velocity=(0.0, 0.0, 0.0)),
    )
    predicted_normal = _calculate_wrench(normal_profile, samples)

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
                contact_point = samples["centers"][sphere.name] - sphere.radius * normal
                unit_moment = np.cross(contact_point, unit_friction)
                force_scale = max(
                    1.0,
                    float(np.max(np.abs(reference[side]["force"][calibration_mask]))),
                )
                moment_scale = max(
                    1.0,
                    float(np.max(np.abs(reference[side]["moment"][calibration_mask]))),
                )
                columns.append(
                    _weighted_wrench_vector(
                        unit_friction[calibration_mask],
                        unit_moment[calibration_mask],
                        force_scale,
                        moment_scale,
                        moment_weight,
                    )
                )
            matrix = np.column_stack(columns)
            target = _weighted_wrench_vector(
                (
                    reference[side]["force"]
                    - predicted_normal[side]["force"]
                )[calibration_mask],
                (
                    reference[side]["moment"]
                    - predicted_normal[side]["moment"]
                )[calibration_mask],
                force_scale,
                moment_scale,
                moment_weight,
            )
            fit = lsq_linear(
                matrix,
                target,
                bounds=(0.0, 2.0),
                lsmr_tol="auto",
            )
            coefficients[side] = fit.x
            error = matrix @ fit.x - target
            total += float(np.mean(error * error))
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
            side: float(np.max(predicted_normal[side]["force"][:, 1]))
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
    parser.add_argument(
        "--additional-states-sto",
        action="append",
        default=[],
        help="Additional forward CoordinateStates trajectory; may be repeated.",
    )
    parser.add_argument(
        "--primary-state-repeat",
        type=int,
        default=1,
        help="Repeat the primary --states-sto samples to weight active trajectories.",
    )
    parser.add_argument("--calibration-fraction", type=float, default=0.6)
    parser.add_argument("--threshold", type=float, default=20.0)
    parser.add_argument("--location-count", type=int, default=10)
    parser.add_argument("--lateral-count", type=int, default=5)
    parser.add_argument("--lateral-half-width", type=float, default=0.08)
    parser.add_argument("--radius-count", type=int, default=10)
    parser.add_argument("--max-contacts-per-side", type=int, default=12)
    parser.add_argument(
        "--moment-weight",
        type=float,
        default=1.0,
        help="Relative weight of moment error in full-wrench contact fitting.",
    )
    parser.add_argument(
        "--maximum-penetration",
        type=float,
        default=0.0,
        help=(
            "Enable physical-basis mode: shift the ground and reject candidates "
            "whose maximum raw penetration exceeds this value [m]."
        ),
    )
    parser.add_argument(
        "--max-stiffness",
        type=float,
        default=1.0e8,
        help="Maximum per-contact stiffness in physical-basis mode.",
    )
    parser.add_argument(
        "--sea-plugin",
        default="plugins/SEA_Plugin_BlackBox_mCMC_impedence_ff",
    )
    args = parser.parse_args()

    setup = read_setup_xml(args.setup)
    if setup.external_loads_xml is None:
        raise ValueError("Calibration requires prescribed ExternalLoads.")
    source = load_online_grf_profile(args.profile)
    state_files = []
    if args.states_sto:
        if args.primary_state_repeat < 1:
            raise ValueError("--primary-state-repeat must be >= 1.")
        state_files.extend([args.states_sto] * args.primary_state_repeat)
        state_files.extend(args.additional_states_sto)

    def sample_state_files(profile: OnlineGRFProfile) -> tuple[np.ndarray, dict]:
        sampled = [
            _sample_spheres_from_coordinate_states(
                setup,
                profile,
                state_file,
                args.sea_plugin,
            )
            for state_file in state_files
        ]
        combined_times = np.concatenate([item[0] for item in sampled])
        names = [sphere.name for sphere in profile.spheres]
        combined_samples = {
            key: {
                name: np.concatenate([item[1][key][name] for item in sampled], axis=0)
                for name in names
            }
            for key in ("centers", "velocities")
        }
        return combined_times, combined_samples

    if state_files:
        times, endpoint_samples = sample_state_files(source)
    else:
        times = np.arange(
            setup.t_start,
            setup.t_end + args.sample_dt * 0.25,
            args.sample_dt,
        )
        endpoint_samples = _sample_spheres(setup, source, times, args.sea_plugin)
    ground_offset = 0.0
    physical_limit = None
    if args.maximum_penetration > 0.0:
        if args.maximum_penetration > 0.03:
            raise ValueError("--maximum-penetration must be <= 0.03 m.")
        physical_limit = float(args.maximum_penetration)
        source, ground_offset = _bounded_ground(
            source,
            endpoint_samples,
            physical_limit,
        )
    candidate_profile = _candidate_sampling_profile(
        source,
        args.location_count,
        args.lateral_count,
        args.lateral_half_width,
    )
    if state_files:
        candidate_times, candidate_samples = sample_state_files(candidate_profile)
    else:
        candidate_times = times
        candidate_samples = _sample_spheres(
            setup,
            candidate_profile,
            times,
            args.sea_plugin,
        )
    if not np.allclose(candidate_times, times, rtol=0.0, atol=1.0e-12):
        raise ValueError("Candidate contact sampling time does not match endpoint sampling.")
    split = int(round(len(times) * args.calibration_fraction))
    split = min(max(split, 2), len(times) - 1)
    calibration_mask = np.arange(len(times)) < split
    holdout_mask = ~calibration_mask
    reference = _external_grf(setup, times)
    reference_wrench = _external_wrench(setup, times)
    normal = np.asarray(source.ground.normal, dtype=float)

    selected = {}
    candidate_counts = {}
    for side in ("left", "right"):
        candidates = _candidate_patch(
            candidate_profile,
            candidate_samples,
            side,
            args.location_count * args.lateral_count,
            args.radius_count,
            physical_limit,
        )
        candidate_counts[side] = len(candidates)
        selected[side] = _fit_sparse_patch(
            candidates,
            reference_wrench[side],
            calibration_mask,
            args.max_contacts_per_side,
            normal,
            args.max_stiffness if physical_limit is not None else None,
            args.moment_weight,
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
                "saved_forward_coordinate_states" if state_files else "ik_replay"
            ),
            "recommended_mode": (
                "online_candidate" if physical_limit is not None else "online_sensor"
            ),
            "online_mode_status": (
                "requires_forward_validation"
                if physical_limit is not None
                else "sensor_calibrated"
            ),
            "maximum_penetration_constraint_m": physical_limit,
            "ground_offset_along_normal_m": ground_offset,
            "calibration_target": "full_wrench_force_and_moment",
            "candidate_patch": {
                "longitudinal_count": args.location_count,
                "lateral_count": args.lateral_count,
                "lateral_half_width_m": args.lateral_half_width,
                "moment_weight": args.moment_weight,
            },
        },
    )
    profile, tangent_report = _fit_tangent(
        profile,
        samples,
        reference_wrench,
        calibration_mask,
        args.moment_weight,
    )
    predicted_wrench = _calculate_wrench(profile, samples)
    predicted = {
        side: predicted_wrench[side]["force"]
        for side in ("left", "right")
    }
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
        "state_trajectories": [
            str(resolve_repo_path(path).resolve())
            for path in state_files
        ],
        "candidate_counts": candidate_counts,
        "candidate_patch": {
            "longitudinal_count": args.location_count,
            "lateral_count": args.lateral_count,
            "lateral_half_width_m": args.lateral_half_width,
            "moment_weight": args.moment_weight,
        },
        "selected_counts": {
            side: len(selected[side]) for side in ("left", "right")
        },
        "maximum_penetration_constraint_m": physical_limit,
        "ground_offset_along_normal_m": ground_offset,
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
        "calibration_wrench_metrics": _wrench_metrics(
            reference_wrench,
            predicted_wrench,
            times,
            calibration_mask,
            args.threshold,
        ),
        "holdout_wrench_metrics": _wrench_metrics(
            reference_wrench,
            predicted_wrench,
            times,
            holdout_mask,
            args.threshold,
        ),
        "calibration_physical_metrics": _physical_metrics(
            profile,
            samples,
            calibration_mask,
        ),
        "holdout_physical_metrics": _physical_metrics(
            profile,
            samples,
            holdout_mask,
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
