"""
Calibrate and validate an onlineGRF profile against prescribed ExternalLoads.

The script replays the setup kinematics without forward integration, samples
the profile sphere positions/velocities, and evaluates the same contact law as
the OnlineGRFContact plugin. Parameters are fit only on the calibration split;
the final JSON report contains separate calibration and holdout metrics.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import replace
from pathlib import Path
from typing import Dict

import numpy as np
from scipy.optimize import differential_evolution

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

import opensim

from config import SimulatorConfig
from kinematics_interpolator import KinematicsInterpolator
from model_loader import _infer_external_force_side, _load_plugin
from online_grf import (
    OnlineGRFProfile,
    load_online_grf_profile,
    write_online_grf_profile,
)
from output import _read_storage_table
from path_resolver import resolve_repo_path
from setup_io import read_setup_xml


def _external_grf(setup, times: np.ndarray) -> Dict[str, np.ndarray]:
    ext = opensim.ExternalLoads(str(setup.external_loads_xml), True)
    data_file = Path(ext.getDataFileName())
    if not data_file.is_absolute():
        data_file = setup.external_loads_xml.parent / data_file
    source_time, columns, data = _read_storage_table(str(data_file))
    col_idx = {name: i for i, name in enumerate(columns)}
    result: Dict[str, np.ndarray] = {}
    for i in range(ext.getSize()):
        force = ext.get(i)
        side = _infer_external_force_side(
            force.getName(),
            force.get_applied_to_body(),
        )
        if side not in {"left", "right"}:
            continue
        identifier = force.getForceIdentifier()
        values = np.column_stack(
            [
                data[:, col_idx[f"{identifier}{axis}"]]
                for axis in ("x", "y", "z")
            ]
        )
        result[side] = np.column_stack(
            [
                np.interp(times, source_time, values[:, axis])
                for axis in range(3)
            ]
        )
    missing = {"left", "right"} - set(result)
    if missing:
        raise ValueError(f"Missing prescribed GRF sides: {sorted(missing)}")
    return result


def _sample_spheres(
    setup,
    profile: OnlineGRFProfile,
    times: np.ndarray,
    sea_plugin: str,
) -> dict:
    _load_plugin(str(resolve_repo_path(sea_plugin)))
    model = opensim.Model(str(setup.model_file))
    state = model.initSystem()
    cfg = SimulatorConfig()
    cfg.model_bundle_dir = str(setup.model_file.parent)
    cfg.model_file = str(setup.model_file)
    cfg.kinematics_file = str(setup.kinematics_file)
    cfg.t_start = float(times[0])
    cfg.t_end = float(times[-1])
    kin = KinematicsInterpolator(cfg)
    coord_set = model.getCoordinateSet()
    frames = {
        sphere.name: opensim.PhysicalFrame.safeDownCast(
            model.getComponent(sphere.frame)
        )
        for sphere in profile.spheres
    }
    if any(frame is None for frame in frames.values()):
        raise ValueError("Every profile sphere frame must be a PhysicalFrame.")

    centers = {sphere.name: np.empty((len(times), 3)) for sphere in profile.spheres}
    velocities = {
        sphere.name: np.empty((len(times), 3)) for sphere in profile.spheres
    }
    for row, time in enumerate(times):
        q, qdot, _ = kin.get(float(time))
        state.setTime(float(time))
        for i in range(coord_set.getSize()):
            coord = coord_set.get(i)
            name = coord.getName()
            if name in q:
                coord.setValue(state, float(q[name]), False)
                coord.setSpeedValue(state, float(qdot[name]))
        model.realizeVelocity(state)
        for sphere in profile.spheres:
            location = opensim.Vec3(*sphere.location)
            center = frames[sphere.name].findStationLocationInGround(state, location)
            velocity = frames[sphere.name].findStationVelocityInGround(state, location)
            centers[sphere.name][row] = [float(center.get(i)) for i in range(3)]
            velocities[sphere.name][row] = [float(velocity.get(i)) for i in range(3)]
    return {"centers": centers, "velocities": velocities}


def _sample_spheres_from_coordinate_states(
    setup,
    profile: OnlineGRFProfile,
    states_file: str | Path,
    sea_plugin: str,
) -> tuple[np.ndarray, dict]:
    """Sample contact geometry from a saved simulator CoordinateStates file."""
    _load_plugin(str(resolve_repo_path(sea_plugin)))
    model = opensim.Model(str(setup.model_file))
    state = model.initSystem()
    times, columns, data = _read_storage_table(str(resolve_repo_path(states_file)))
    col_idx = {name: index for index, name in enumerate(columns)}
    coord_set = model.getCoordinateSet()
    frames = {
        sphere.name: opensim.PhysicalFrame.safeDownCast(
            model.getComponent(sphere.frame)
        )
        for sphere in profile.spheres
    }
    if any(frame is None for frame in frames.values()):
        raise ValueError("Every profile sphere frame must be a PhysicalFrame.")
    centers = {sphere.name: np.empty((len(times), 3)) for sphere in profile.spheres}
    velocities = {
        sphere.name: np.empty((len(times), 3)) for sphere in profile.spheres
    }
    for row, time in enumerate(times):
        state.setTime(float(time))
        for index in range(coord_set.getSize()):
            coord = coord_set.get(index)
            name = coord.getName()
            q_index = col_idx.get(f"{name}_q")
            qdot_index = col_idx.get(f"{name}_qdot")
            if q_index is None or qdot_index is None:
                raise ValueError(f"CoordinateStates file is missing {name}_q/qdot.")
            coord.setValue(state, float(data[row, q_index]), False)
            coord.setSpeedValue(state, float(data[row, qdot_index]))
        model.realizeVelocity(state)
        for sphere in profile.spheres:
            location = opensim.Vec3(*sphere.location)
            center = frames[sphere.name].findStationLocationInGround(state, location)
            velocity = frames[sphere.name].findStationVelocityInGround(state, location)
            centers[sphere.name][row] = [float(center.get(i)) for i in range(3)]
            velocities[sphere.name][row] = [float(velocity.get(i)) for i in range(3)]
    return times, {"centers": centers, "velocities": velocities}


def _smooth_positive(values: np.ndarray, epsilon: float) -> np.ndarray:
    return 0.5 * (values + np.sqrt(values * values + epsilon * epsilon))


def _calculate_grf(
    profile: OnlineGRFProfile,
    samples: dict,
    *,
    ground_offset: float = 0.0,
    radius_scale: float = 1.0,
    stiffness: float | None = None,
    dissipation: float | None = None,
    friction: float | None = None,
) -> Dict[str, np.ndarray]:
    normal = np.asarray(profile.ground.normal, dtype=float)
    normal /= np.linalg.norm(normal)
    origin = np.asarray(profile.ground.origin, dtype=float) + ground_offset * normal
    surface_velocity = np.asarray(profile.ground.surface_velocity, dtype=float)
    material = profile.material
    result = {
        "left": np.zeros_like(next(iter(samples["centers"].values()))),
        "right": np.zeros_like(next(iter(samples["centers"].values()))),
    }

    for sphere in profile.spheres:
        center = samples["centers"][sphere.name]
        velocity = samples["velocities"][sphere.name]
        sphere_material = sphere.material or material
        sphere_k = sphere_material.stiffness if stiffness is None else float(stiffness)
        sphere_d = (
            sphere_material.dissipation if dissipation is None else float(dissipation)
        )
        sphere_mu_static = (
            sphere_material.static_friction if friction is None else float(friction)
        )
        sphere_mu_dynamic = (
            sphere_material.dynamic_friction if friction is None else float(friction)
        )
        sphere_epsilon = sphere_material.smoothing
        sphere_transition = sphere_material.transition_velocity
        distance = (center - origin) @ normal
        penetration_raw = sphere.radius * radius_scale - distance
        penetration = _smooth_positive(penetration_raw, sphere_epsilon)
        relative_velocity = velocity - surface_velocity
        normal_velocity = relative_velocity @ normal
        damping = _smooth_positive(
            1.0 - sphere_d * normal_velocity,
            sphere_epsilon,
        )
        normal_force = (
            sphere_k
            * np.power(penetration, sphere_material.exponent)
            * damping
        )
        tangent = relative_velocity - normal_velocity[:, None] * normal
        slip = np.linalg.norm(tangent, axis=1)
        ratio = slip / sphere_transition
        mu = sphere_mu_dynamic + (
            sphere_mu_static - sphere_mu_dynamic
        ) * np.exp(-(ratio * ratio))
        friction_force = (
            -mu[:, None]
            * normal_force[:, None]
            * tangent
            / np.sqrt(slip * slip + sphere_transition * sphere_transition)[:, None]
            - sphere_material.viscous_friction * tangent
        )
        result[sphere.side] += normal_force[:, None] * normal + friction_force
    return result


def _contact_f1(reference: np.ndarray, predicted: np.ndarray, threshold: float) -> float:
    truth = reference[:, 1] > threshold
    test = predicted[:, 1] > threshold
    tp = int(np.sum(truth & test))
    fp = int(np.sum(~truth & test))
    fn = int(np.sum(truth & ~test))
    return 2.0 * tp / max(1, 2 * tp + fp + fn)


def _crossing_times(
    time: np.ndarray,
    vertical_force: np.ndarray,
    threshold: float,
    rising: bool,
) -> np.ndarray:
    before = vertical_force[:-1] <= threshold
    after = vertical_force[1:] > threshold
    mask = before & after if rising else ~before & ~after
    indices = np.flatnonzero(mask)
    values = []
    for index in indices:
        delta = vertical_force[index + 1] - vertical_force[index]
        fraction = 0.0 if abs(delta) < 1e-12 else (
            threshold - vertical_force[index]
        ) / delta
        values.append(time[index] + fraction * (time[index + 1] - time[index]))
    return np.asarray(values, dtype=float)


def _event_timing(
    time: np.ndarray,
    reference: np.ndarray,
    predicted: np.ndarray,
    threshold: float,
) -> dict:
    report = {}
    for label, rising in (("heel_strike", True), ("toe_off", False)):
        reference_times = _crossing_times(time, reference[:, 1], threshold, rising)
        predicted_times = _crossing_times(time, predicted[:, 1], threshold, rising)
        if len(reference_times) and len(predicted_times):
            errors = np.asarray(
                [
                    float(np.min(np.abs(predicted_times - event_time)))
                    for event_time in reference_times
                ]
            )
            mae = float(np.mean(errors))
            maximum = float(np.max(errors))
        else:
            mae = float("nan")
            maximum = float("nan")
        report[label] = {
            "reference_count": int(len(reference_times)),
            "predicted_count": int(len(predicted_times)),
            "nearest_event_mae_s": mae,
            "nearest_event_max_s": maximum,
        }
    return report


def _metrics(
    reference: Dict[str, np.ndarray],
    predicted: Dict[str, np.ndarray],
    time: np.ndarray,
    mask: np.ndarray,
    threshold: float,
) -> dict:
    report = {}
    for side in ("left", "right"):
        ref = reference[side][mask]
        pred = predicted[side][mask]
        selected_time = time[mask]
        error = pred - ref
        peak = np.maximum(np.max(np.abs(ref), axis=0), 1.0)
        correlations = []
        for axis in range(3):
            if np.std(ref[:, axis]) < 1e-12 or np.std(pred[:, axis]) < 1e-12:
                correlations.append(float("nan"))
            else:
                correlations.append(float(np.corrcoef(ref[:, axis], pred[:, axis])[0, 1]))
        report[side] = {
            "rmse_n": np.sqrt(np.mean(error * error, axis=0)).tolist(),
            "nrmse_peak": (
                np.sqrt(np.mean(error * error, axis=0)) / peak
            ).tolist(),
            "correlation": correlations,
            "vertical_contact_f1": _contact_f1(ref, pred, threshold),
            "reference_peak_vertical_n": float(np.max(ref[:, 1])),
            "predicted_peak_vertical_n": float(np.max(pred[:, 1])),
            "event_timing": _event_timing(
                selected_time, ref, pred, threshold
            ),
        }
    return report


def _profile_with_fit(
    profile: OnlineGRFProfile,
    parameters: np.ndarray,
) -> OnlineGRFProfile:
    ground_offset, radius_scale, log_stiffness, dissipation, friction = parameters
    normal = np.asarray(profile.ground.normal, dtype=float)
    origin = np.asarray(profile.ground.origin, dtype=float) + ground_offset * normal
    return replace(
        profile,
        source="calibrated_against_prescribed_grf",
        ground=replace(profile.ground, origin=tuple(float(v) for v in origin)),
        material=replace(
            profile.material,
            stiffness=float(10.0 ** log_stiffness),
            dissipation=float(dissipation),
            static_friction=float(friction),
            dynamic_friction=float(friction),
        ),
        spheres=tuple(
            replace(sphere, radius=float(sphere.radius * radius_scale))
            for sphere in profile.spheres
        ),
        metadata={
            **dict(profile.metadata),
            "calibration_parameters": {
                "ground_offset_along_normal_m": float(ground_offset),
                "radius_scale": float(radius_scale),
            },
        },
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--setup", required=True)
    parser.add_argument("--profile", required=True)
    parser.add_argument("--out-profile", default="")
    parser.add_argument("--report", default="results/online_grf_validation.json")
    parser.add_argument("--sample-dt", type=float, default=0.005)
    parser.add_argument("--calibration-fraction", type=float, default=0.6)
    parser.add_argument("--threshold", type=float, default=20.0)
    parser.add_argument("--max-iterations", type=int, default=30)
    parser.add_argument("--no-calibrate", action="store_true")
    parser.add_argument(
        "--sea-plugin",
        default="plugins/SEA_Plugin_BlackBox_mCMC_impedence_ff",
    )
    args = parser.parse_args()

    setup = read_setup_xml(args.setup)
    if setup.external_loads_xml is None:
        raise ValueError("Validation requires prescribed ExternalLoads.")
    profile = load_online_grf_profile(args.profile)
    times = np.arange(setup.t_start, setup.t_end + args.sample_dt * 0.25, args.sample_dt)
    split = int(round(len(times) * args.calibration_fraction))
    split = min(max(split, 2), len(times) - 1)
    calibration_mask = np.arange(len(times)) < split
    holdout_mask = ~calibration_mask

    samples = _sample_spheres(setup, profile, times, args.sea_plugin)
    reference = _external_grf(setup, times)
    parameters = np.array(
        [
            0.0,
            1.0,
            math.log10(profile.material.stiffness),
            profile.material.dissipation,
            0.5 * (
                profile.material.static_friction
                + profile.material.dynamic_friction
            ),
        ],
        dtype=float,
    )
    optimization = None
    if not args.no_calibrate:
        scales = {
            side: max(1.0, float(np.max(np.abs(reference[side][calibration_mask]))))
            for side in ("left", "right")
        }

        def objective(candidate: np.ndarray) -> float:
            predicted = _calculate_grf(
                profile,
                samples,
                ground_offset=candidate[0],
                radius_scale=candidate[1],
                stiffness=10.0 ** candidate[2],
                dissipation=candidate[3],
                friction=candidate[4],
            )
            loss = 0.0
            for side in ("left", "right"):
                error = (
                    predicted[side][calibration_mask]
                    - reference[side][calibration_mask]
                ) / scales[side]
                loss += float(np.mean(error * error))
                loss += 0.25 * (
                    1.0
                    - _contact_f1(
                        reference[side][calibration_mask],
                        predicted[side][calibration_mask],
                        args.threshold,
                    )
                )
            return loss

        optimization = differential_evolution(
            objective,
            bounds=[
                (-0.10, 0.10),
                (0.5, 1.5),
                (4.0, 8.0),
                (0.0, 5.0),
                (0.05, 1.5),
            ],
            maxiter=args.max_iterations,
            seed=20260607,
            polish=True,
            updating="immediate",
        )
        parameters = optimization.x

    calibrated = _profile_with_fit(profile, parameters)
    predicted = _calculate_grf(calibrated, samples)
    report = {
        "setup": str(Path(args.setup).resolve()),
        "input_profile": str(Path(args.profile).resolve()),
        "samples": len(times),
        "sample_dt": args.sample_dt,
        "calibration_fraction": args.calibration_fraction,
        "calibration_time_range": [float(times[0]), float(times[split - 1])],
        "holdout_time_range": [float(times[split]), float(times[-1])],
        "optimization": None
        if optimization is None
        else {
            "success": bool(optimization.success),
            "message": str(optimization.message),
            "objective": float(optimization.fun),
            "iterations": int(optimization.nit),
        },
        "calibration_metrics": _metrics(
            reference, predicted, times, calibration_mask, args.threshold
        ),
        "holdout_metrics": _metrics(
            reference, predicted, times, holdout_mask, args.threshold
        ),
        "calibrated_profile": calibrated.to_dict(),
    }
    report_path = resolve_repo_path(args.report)
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    if args.out_profile:
        write_online_grf_profile(calibrated, resolve_repo_path(args.out_profile))
    print(f"onlineGRF validation report: {report_path.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
