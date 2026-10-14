"""Compare saved AB07 IK/remap poses with measured foot markers, without integration.

The free rigid fits are diagnostic lower bounds, not replacement IK or a new
marker calibration. No model, profile, marker or controller file is modified.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np


REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "tools"))
sys.path.insert(0, str(REPO))
from calibrate_ab06_seasea_markers import (  # noqa: E402
    from_vec3, interpolate_marker, interpolate_motion, load_plugin,
    parse_motion, parse_trc, set_state_from_motion, vec3,
)
import opensim as osim  # noqa: E402
from binary_phase_detector import load_binary_phase_detector_profile  # noqa: E402


MARKERS = ("L_Ankle_Lat", "L_Heel", "L_Toe_Lat", "L_Toe_Med", "L_Toe_Tip")


def source(path: Path) -> dict:
    return {"path": str(path.resolve()), "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}


def storage_columns(path: Path) -> tuple[list[str], np.ndarray]:
    lines = path.read_text().splitlines()
    endheader = next(i for i, line in enumerate(lines) if line.strip().lower() == "endheader")
    body = [line for line in lines[endheader+1:] if line.strip()]
    labels = body[0].split()
    data = np.loadtxt(body[1:], dtype=float, ndmin=2)
    return labels, data


def stats(values: np.ndarray) -> dict:
    x = np.asarray(values, dtype=float)
    x = x[np.isfinite(x)]
    return {"count": int(x.size), "mean": float(np.mean(x)),
            "rms": float(np.sqrt(np.mean(x*x))), "min": float(np.min(x)),
            "p95": float(np.percentile(x, 95)), "max": float(np.max(x))}


def rigid_transform(local: np.ndarray, measured: np.ndarray, fit_indices: list[int]) -> tuple:
    """Least-squares proper rigid transform fitted on the selected marker subset."""
    p, q = local[fit_indices], measured[fit_indices]
    pc, qc = p.mean(axis=0), q.mean(axis=0)
    u, _, vt = np.linalg.svd((p-pc).T @ (q-qc))
    correction = np.eye(3)
    correction[2, 2] = np.linalg.det(vt.T @ u.T)
    rotation = vt.T @ correction @ u.T
    return rotation, pc, qc


def rigid_fit(local: np.ndarray, measured: np.ndarray, fit_indices: list[int]) -> np.ndarray:
    rotation, pc, qc = rigid_transform(local, measured, fit_indices)
    return (local-pc) @ rotation.T + qc


def sensor_geometry(ground_points: np.ndarray, times: np.ndarray, profile, load: np.ndarray | None) -> dict:
    origin, normal = np.asarray(profile.ground.origin), np.asarray(profile.ground.normal)
    clearances = (ground_points-origin) @ normal
    contacts = clearances <= 0.0
    loaded = None if load is None else load > 20.0
    roles = [point.name for point in profile.points]
    return {
        "pose_count": len(times), "time_range_s": [float(times[0]), float(times[-1])],
        "contact_semantics": "geometric signed clearance <=0; no debounce or FSM events",
        "points": {role: {
            "signed_clearance_m": stats(clearances[:, i]),
            "contact_fraction": float(np.mean(contacts[:, i])),
            "contact_fraction_when_measured_left_fz_gt20n": None if loaded is None or not np.any(loaded) else float(np.mean(contacts[loaded, i])),
        } for i, role in enumerate(roles)},
        "any_contact_fraction": float(np.mean(np.any(contacts, axis=1))),
        "any_contact_fraction_when_measured_left_fz_gt20n": None if loaded is None or not np.any(loaded) else float(np.mean(np.any(contacts[loaded], axis=1))),
        "measured_loaded_samples_gt20n": None if loaded is None else int(np.count_nonzero(loaded)),
        "series": {"time_s": times.tolist(),
                   "signed_clearance_m": {role: clearances[:, i].tolist() for i, role in enumerate(roles)},
                   "measured_left_fz_n": None if load is None else load.tolist()},
    }


def world_y_decomposition(predicted: np.ndarray, sensor_ground: np.ndarray, measured: np.ndarray, local: np.ndarray, profile) -> dict:
    heel_index = MARKERS.index("L_Heel")
    heel_y, measured_heel_y = predicted[:, heel_index, 1], measured[:, heel_index, 1]
    result = {}
    for i, point in enumerate(profile.points):
        sensor_y = sensor_ground[:, i, 1]
        gap_y = sensor_y-heel_y
        bias_y = heel_y-measured_heel_y
        result[point.name] = {
            "identity": "sensor_ground_Y = measured_L_Heel_ground_Y + marker_fit_bias_Y + [R_G_foot @ (sensor_local - L_Heel_local)]_Y",
            "local_sensor_minus_L_Heel_m": (np.asarray(point.location)-local[heel_index]).tolist(),
            "sensor_ground_y_m": stats(sensor_y), "modeled_L_Heel_ground_y_m": stats(heel_y),
            "measured_L_Heel_ground_y_m": stats(measured_heel_y), "marker_fit_bias_y_m": stats(bias_y),
            "projected_local_gap_y_m": stats(gap_y),
            "projection_method": "difference of two stations sharing one rigid foot transform, equivalent to R_G_foot times their local difference",
            "identity_max_abs_error_m": float(np.max(np.abs(sensor_y-(measured_heel_y+bias_y+gap_y)))),
        }
    return result


def static_reference(model_path: Path, trc_path: Path, motion_path: Path, profile) -> dict:
    model = osim.Model(str(model_path))
    state = model.initSystem()
    body = model.getBodySet().get("foot_l")
    markers = [model.getMarkerSet().get(name) for name in MARKERS]
    if {str(marker.getSocket("parent_frame").getConnecteePath()) for marker in markers} != {"/bodyset/foot_l"}:
        raise ValueError("AB06 static reference marker stations must belong to foot_l.")
    local = np.stack([from_vec3(marker.get_location()) for marker in markers])
    sensor_local = np.asarray([point.location for point in profile.points])
    trc, motion = parse_trc(trc_path), parse_motion(motion_path)
    times = trc.times[(trc.times >= motion.times[0]-1e-9) & (trc.times <= motion.times[-1]+1e-9)]
    cloud = np.stack([[interpolate_marker(trc, marker, float(t)) for marker in MARKERS] for t in times])
    if not np.all(np.isfinite(cloud)):
        raise ValueError("AB06 static measured marker cloud contains missing/nonfinite values.")
    predicted, native_sensor = [], []
    for t in times:
        set_state_from_motion(model, state, interpolate_motion(motion, float(t)), motion.in_degrees)
        transform = body.getTransformInGround(state)
        predicted.append([from_vec3(transform.shiftFrameStationToBase(vec3(point))) for point in local])
        native_sensor.append([from_vec3(transform.shiftFrameStationToBase(vec3(point))) for point in sensor_local])
    errors = np.stack(predicted)-cloud
    distance = np.linalg.norm(errors, axis=2)
    free = {}
    for name, indices in (("all_five", list(range(5))), ("four_calcaneus_markers", list(range(1, 5)))):
        fit_errors, free_sensor = [], []
        for row in cloud:
            rotation, pc, qc = rigid_transform(local, row, indices)
            fit_errors.append((local-pc) @ rotation.T + qc-row)
            free_sensor.append((sensor_local-pc) @ rotation.T + qc)
        e = np.stack(fit_errors)
        d = np.linalg.norm(e, axis=2)
        free[name] = {
            "fitted_marker_frame_rms_m": stats(np.sqrt(np.mean(d[:, indices]**2, axis=1))),
            "all_marker_frame_rms_m": stats(np.sqrt(np.mean(d**2, axis=1))),
            "held_out_ankle_distance_m": stats(d[:, 0]) if name == "four_calcaneus_markers" else None,
            "frozen_binary_sensor_geometry": sensor_geometry(np.stack(free_sensor), times, profile, None),
            "world_y_decomposition": world_y_decomposition(e+cloud, np.stack(free_sensor), cloud, local, profile),
        }
    return {
        "subject": "AB06", "scope": "Existing converted static baseline with its own operational model and MarkerSet; same frozen binary profile as AB07",
        "sources": {"model": source(model_path), "trc": source(trc_path), "healthy_remap": source(motion_path)},
        "pose_count": len(times), "time_range_s": [float(times[0]), float(times[-1])],
        "converted_trc_units": trc.units,
        "converted_trc_sample_dt_s": float(np.median(np.diff(trc.times))),
        "measured_marker_ground_y_m": {marker: stats(cloud[:, i, 1]) for i, marker in enumerate(MARKERS)},
        "local_markers_m": dict(zip(MARKERS, local.tolist())),
        "native_pose": {"frame_marker_rms_m": stats(np.sqrt(np.mean(distance**2, axis=1))),
                        "per_marker_distance_m": {marker: stats(distance[:, i]) for i, marker in enumerate(MARKERS)},
                        "frozen_binary_sensor_geometry": sensor_geometry(np.stack(native_sensor), times, profile, None),
                        "world_y_decomposition": world_y_decomposition(np.stack(predicted), np.stack(native_sensor), cloud, local, profile)},
        "unconstrained_rigid_cloud_fits": free,
        "limitations": ["AB06 uses its existing historical converted static clock, not a reconversion of source MAT timestamps.",
                        "No measured static GRF is present in this converted baseline; geometric contact does not establish static force, physical floor datum or stance qualification."],
    }


def run(args: argparse.Namespace) -> dict:
    paths = {"model": args.model, "donor": args.donor, "trc": args.trc,
             "new_ik": args.ik, "healthy_remap": args.remap, "plugin": args.plugin,
             "binary_profile": args.binary_profile, "measured_grf": args.grf,
             "static_trc": args.static_trc, "static_healthy_remap": args.static_remap,
             "reference_static_trc": args.reference_static_trc, "reference_static_healthy_remap": args.reference_static_remap,
             "script": Path(__file__).resolve()}
    before = {name: source(path) for name, path in paths.items()}
    load_plugin(args.plugin)
    model = osim.Model(str(args.model))
    state = model.initSystem()
    marker_set = model.getMarkerSet()
    markers = [marker_set.get(name) for name in MARKERS]
    parents = [str(marker.getSocket("parent_frame").getConnecteePath()) for marker in markers]
    if set(parents) != {"/bodyset/foot_l"}:
        raise ValueError(f"Expected the five marker stations on foot_l; observed {parents}")
    body = model.getBodySet().get("foot_l")
    local = np.stack([from_vec3(marker.get_location()) for marker in markers])
    profile = load_binary_phase_detector_profile(args.binary_profile)
    if {point.frame for point in profile.points} != {"/bodyset/foot_l"}:
        raise ValueError("Counterfactual sensor stations must belong to foot_l.")
    sensor_local = np.asarray([point.location for point in profile.points])
    trc = parse_trc(args.trc)
    static_trc = parse_trc(args.static_trc)
    static_motion = parse_motion(args.static_remap)
    grf_labels, grf_data = storage_columns(args.grf)
    grf_y = grf_data[:, grf_labels.index("ground_force1_vy")]
    motions = {"new_ik": parse_motion(args.ik), "healthy_remap": parse_motion(args.remap)}
    first = max(trc.times[0], *(motion.times[0] for motion in motions.values()))
    last = min(trc.times[-1], *(motion.times[-1] for motion in motions.values()))
    valid = trc.times[(trc.times >= first-1e-9) & (trc.times <= last+1e-9)]
    windows = {"selected_window": valid[(valid >= args.start-1e-9) & (valid <= args.end+1e-9)],
               "full_trial_sampled": valid[::args.full_stride]}
    if windows["full_trial_sampled"][-1] != valid[-1]:
        windows["full_trial_sampled"] = np.append(windows["full_trial_sampled"], valid[-1])
    windows["static_window"] = static_trc.times[(static_trc.times >= static_motion.times[0]-1e-9) & (static_trc.times <= static_motion.times[-1]+1e-9)]
    if any(len(times) == 0 for times in windows.values()):
        raise ValueError("Requested marker diagnostic window has no overlapping samples.")
    loads = {name: None if name == "static_window" else np.interp(times, grf_data[:, 0], grf_y) for name, times in windows.items()}
    measured = {}
    rigid = {}
    for name, times in windows.items():
        selected_trc = static_trc if name == "static_window" else trc
        cloud = np.stack([[interpolate_marker(selected_trc, marker, float(t)) for marker in MARKERS] for t in times])
        if not np.all(np.isfinite(cloud)):
            raise ValueError("Measured foot-marker cloud has missing or nonfinite values.")
        measured[name] = cloud
        fits = {"all_five": [], "four_calcaneus_markers": []}
        sensor_points = {key: [] for key in fits}
        for row in cloud:
            for fit_name, indices in (("all_five", list(range(5))), ("four_calcaneus_markers", list(range(1, 5)))):
                rotation, pc, qc = rigid_transform(local, row, indices)
                fits[fit_name].append((local-pc) @ rotation.T + qc-row)
                sensor_points[fit_name].append((sensor_local-pc) @ rotation.T + qc)
        rigid[name] = {}
        for fit_name, errors in fits.items():
            e = np.stack(errors)
            d = np.linalg.norm(e, axis=2)
            use = list(range(5)) if fit_name == "all_five" else list(range(1, 5))
            rigid[name][fit_name] = {
                "fitted_marker_frame_rms_m": stats(np.sqrt(np.mean(d[:, use]**2, axis=1))),
                "all_marker_frame_rms_m": stats(np.sqrt(np.mean(d**2, axis=1))),
                "held_out_ankle_distance_m": stats(d[:, 0]) if fit_name != "all_five" else None,
                "heel_vertical_residual_m": stats(e[:, 1, 1]),
                "frozen_binary_sensor_geometry": sensor_geometry(np.stack(sensor_points[fit_name]), times, profile, loads[name]),
            }
            if name == "static_window":
                rigid[name][fit_name]["world_y_decomposition"] = world_y_decomposition(e+cloud, np.stack(sensor_points[fit_name]), cloud, local, profile)
    streams = {}
    motions["static_healthy_remap"] = static_motion
    for stream, motion in motions.items():
        streams[stream] = {}
        applicable_windows = ["static_window"] if stream == "static_healthy_remap" else ["selected_window", "full_trial_sampled"]
        for window in applicable_windows:
            times = windows[window]
            predicted = []
            sensor_points = []
            for t in times:
                set_state_from_motion(model, state, interpolate_motion(motion, float(t)), motion.in_degrees)
                transform = body.getTransformInGround(state)
                predicted.append([from_vec3(transform.shiftFrameStationToBase(vec3(point))) for point in local])
                sensor_points.append([from_vec3(transform.shiftFrameStationToBase(vec3(point))) for point in sensor_local])
            pred = np.stack(predicted)
            errors = pred-measured[window]
            distance = np.linalg.norm(errors, axis=2)
            streams[stream][window] = {
                "pose_count": len(times), "time_range_s": [float(times[0]), float(times[-1])],
                "frame_marker_rms_m": stats(np.sqrt(np.mean(distance**2, axis=1))),
                "per_marker": {marker: {
                    "distance_m": stats(distance[:, i]),
                    "geometric_mean_distance_m": float(np.exp(np.mean(np.log(np.maximum(distance[:, i], np.finfo(float).tiny))))),
                    "vertical_residual_m": stats(errors[:, i, 1]),
                    "max_distance_time_s": float(times[np.argmax(distance[:, i])]),
                } for i, marker in enumerate(MARKERS)},
                "frozen_binary_sensor_geometry": sensor_geometry(np.stack(sensor_points), times, profile, loads[window]),
            }
            if window == "static_window":
                streams[stream][window]["world_y_decomposition"] = world_y_decomposition(pred, np.stack(sensor_points), measured[window], local, profile)
    donor = ET.parse(args.donor).getroot()
    donor_points = {name: [float(x) for x in donor.find(f".//Marker[@name='{name}']/location").text.split()] for name in MARKERS}
    marker_geometry = {name: {"donor_local_m": donor_points[name], "subject_local_m": local[i].tolist(),
                              "subject_minus_donor_m": (local[i]-donor_points[name]).tolist()} for i, name in enumerate(MARKERS)}
    cross_reference = static_reference(args.donor, args.reference_static_trc, args.reference_static_remap, profile)
    comparisons = {}
    native_ab07 = streams["static_healthy_remap"]["static_window"]["frozen_binary_sensor_geometry"]
    for case, geometry_ab06 in (("native_pose", cross_reference["native_pose"]["frozen_binary_sensor_geometry"]),
                               *((name, fit["frozen_binary_sensor_geometry"]) for name, fit in cross_reference["unconstrained_rigid_cloud_fits"].items())):
        geometry_ab07 = native_ab07 if case == "native_pose" else rigid["static_window"][case]["frozen_binary_sensor_geometry"]
        comparisons[case] = {point.name: {
            "AB06_mean_signed_clearance_m": geometry_ab06["points"][point.name]["signed_clearance_m"]["mean"],
            "AB07_mean_signed_clearance_m": geometry_ab07["points"][point.name]["signed_clearance_m"]["mean"],
            "AB07_minus_AB06_mean_signed_clearance_m": geometry_ab07["points"][point.name]["signed_clearance_m"]["mean"]-geometry_ab06["points"][point.name]["signed_clearance_m"]["mean"],
        } for point in profile.points}
    cross_reference["static_comparison"] = comparisons
    decomposition_deltas = {}
    for case in ("native_pose", "all_five", "four_calcaneus_markers"):
        ref = cross_reference["native_pose"] if case == "native_pose" else cross_reference["unconstrained_rigid_cloud_fits"][case]
        subject = streams["static_healthy_remap"]["static_window"] if case == "native_pose" else rigid["static_window"][case]
        decomposition_deltas[case] = {point.name: {
            f"AB07_minus_AB06_mean_{key}": subject["world_y_decomposition"][point.name][key]["mean"]-ref["world_y_decomposition"][point.name][key]["mean"]
            for key in ("sensor_ground_y_m", "measured_L_Heel_ground_y_m", "marker_fit_bias_y_m", "projected_local_gap_y_m")
        } for point in profile.points}
    cross_reference["world_y_decomposition_AB07_minus_AB06"] = decomposition_deltas
    after = {name: source(path) for name, path in paths.items()}
    if before != after:
        raise RuntimeError("Source file changed during the pose-only diagnostic.")
    return {
        "schema_version": 4, "subject": "AB07", "status": "diagnostic_completed",
        "scope": "Read saved motion/TRC; set coordinate poses and realizePosition; no integration, IK solve, model/profile/marker changes or retuning",
        "sources": before, "source_files_unchanged": True,
        "units": "m", "residual_sign": "predicted station minus measured TRC; vertical is ground Y",
        "marker_order": list(MARKERS), "marker_parent_frames": dict(zip(MARKERS, parents)),
        "frozen_binary_profile": {"ground_origin_m": list(profile.ground.origin),
                                  "ground_normal": list(profile.ground.normal),
                                  "points": {point.name: {"frame": point.frame, "location_m": list(point.location)} for point in profile.points}},
        "sampling": {"selected_requested_s": [args.start, args.end], "selected_stride": 1,
                     "full_trial_stride": args.full_stride, "full_trial_end_included": True},
        "calibrated_marker_geometry": marker_geometry, "pose_marker_residuals": streams,
        "unconstrained_rigid_cloud_fits": rigid,
        "static_measured_marker_ground_y_m": {marker: stats(measured["static_window"][:, i, 1]) for i, marker in enumerate(MARKERS)},
        "cross_subject_static_reference": cross_reference,
        "limitations": [
            "Static calibration inherits a remapped healthy pose; low static marker-fit RMS is not a foot/ground qualification.",
            "L_Ankle_Lat originated on healthy tibia_l but is assigned to prosthetic foot_l, both in AB06 and AB07.",
            "The four calcaneus markers exclude L_Ankle_Lat; a free rigid fit is a geometric diagnostic lower bound, not a physically reachable model pose or new IK.",
            "Sensor clearance under a free foot-cloud rigid fit is a counterfactual geometry diagnostic; neither sensor coordinates nor ground/reference parameters are fitted or changed.",
            "Loaded-condition statistics use measured left ground_force1_vy >20 N for comparison only; they do not alter binary contact thresholds or qualify FSM event timing.",
            "Differences in residuals do not establish a unique cause or justify changing weights, marker locations, mounting geometry or event thresholds.",
            "Healthy remap omits left subtalar/MTP coordinates and substitutes PinJoint coordinates; it is not the complete healthy model pose.",
        ],
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, default=REPO / "models/AB07_SEASEA_Threadmill/AB07_SEASEA_stiff321_500_pi.osim")
    parser.add_argument("--donor", type=Path, default=REPO / "models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi.osim")
    parser.add_argument("--trc", type=Path, default=REPO / "models/AB07_SEASEA-raw/data/converted/treadmill/treadmill_01_01/treadmill_01_01.trc")
    parser.add_argument("--ik", type=Path, default=REPO / "models/AB07_SEASEA_Threadmill/data/IK_results_AB07_SEASEA.mot")
    parser.add_argument("--remap", type=Path, default=REPO / "models/AB07_SEASEA-raw/data/converted/treadmill/treadmill_01_01/treadmill_01_01_ik_dataset_ab06_seasea.mot")
    parser.add_argument("--plugin", type=Path, default=REPO / "plugins/libSEA_Plugin_BlackBox_mCMC_impedence_ff.dylib")
    parser.add_argument("--binary-profile", type=Path, default=REPO / "validation/binary_phase_detector_v25_geometry_runs/2026-08-04_local_reach_sweep_dev02_04_08/selected_candidate_profile.json")
    parser.add_argument("--grf", type=Path, default=REPO / "models/AB07_SEASEA_Threadmill/data/AB07_SEASEA_GRF_FullSpan.mot")
    parser.add_argument("--static-trc", type=Path, default=REPO / "models/AB07_SEASEA-raw/data/converted/static/static01/static01.trc")
    parser.add_argument("--static-remap", type=Path, default=REPO / "models/AB07_SEASEA-raw/data/converted/static/static01/static01_ik_dataset_ab06_seasea.mot")
    parser.add_argument("--reference-static-trc", type=Path, default=REPO / "models/AB06_SEASEA-raw/data/converted/static/static_01/static_01.trc")
    parser.add_argument("--reference-static-remap", type=Path, default=REPO / "models/AB06_SEASEA-raw/data/converted/static/static_01/static_01_ik_dataset_ab06_seasea.mot")
    parser.add_argument("--start", type=float, default=10.36)
    parser.add_argument("--end", type=float, default=19.37)
    parser.add_argument("--full-stride", type=int, default=20)
    parser.add_argument("--output", type=Path, default=REPO / "validation/ab07_detector_investigation_2026_10_06/marker_geometry_audit.json")
    parser.add_argument("--force", action="store_true", help="Replace only the requested diagnostic receipt.")
    args = parser.parse_args()
    if args.output.exists() and not args.force:
        parser.error("Output already exists; use a separate diagnostic receipt path.")
    if args.full_stride < 1 or args.end <= args.start:
        parser.error("Require positive stride and a positive requested window.")
    receipt = run(args)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(receipt, indent=2, allow_nan=False)+"\n", encoding="utf-8")
    print(f"Pose-only marker geometry audit saved: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
