"""Offline stage-2 registration diagnostics; no integration or operational edits.

Healthy coordinates come from an independent raw-MAT export. Cross-subject
marker bindings and constant rigid registration changes are counterfactuals,
not fitted operational models, replacement IK or validated corrections.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

from audit_marker_geometry import (
    REPO, MARKERS, source, stats, sensor_geometry, rigid_transform, storage_columns,
    parse_trc, parse_motion, load_plugin, from_vec3, vec3,
)
from binary_phase_detector import load_binary_phase_detector_profile
import opensim as osim


TRANSLATIONS = {"pelvis_tx", "pelvis_ty", "pelvis_tz"}
SUBJECTS = {
    "AB06": {"healthy": REPO/"models/AB06-raw/osimxml/ab06.osim",
             "model": REPO/"models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi.osim",
             "static": REPO/"models/AB06_SEASEA-raw/data/converted/static/static_01/static_01.trc",
             "remap": REPO/"models/AB06_SEASEA-raw/data/converted/static/static_01/static_01_ik_dataset_ab06_seasea.mot",
             "dynamic": REPO/"models/AB06_SEASEA-raw/data/converted/treadmill/treadmill_01_01/treadmill_01_01.trc",
             "grf": REPO/"models/AB06_SEASEA-raw/data/converted/treadmill/treadmill_01_01/treadmill_01_01_grf.mot",
             "start": 11.99},
    "AB07": {"healthy": REPO/"models/AB07-raw/osimxml/ab07.osim",
             "model": REPO/"models/AB07_SEASEA_Threadmill/AB07_SEASEA_stiff321_500_pi.osim",
             "static": REPO/"models/AB07_SEASEA-raw/data/converted/static/static01/static01.trc",
             "remap": REPO/"models/AB07_SEASEA-raw/data/converted/static/static01/static01_ik_dataset_ab06_seasea.mot",
             "dynamic": REPO/"models/AB07_SEASEA-raw/data/converted/treadmill/treadmill_01_01/treadmill_01_01.trc",
             "grf": REPO/"models/AB07_SEASEA-raw/data/converted/treadmill/treadmill_01_01/treadmill_01_01_grf.mot",
             "start": 10.36},
}


def local_markers(path: Path) -> np.ndarray:
    root = ET.parse(path).getroot()
    markers = [root.find(f".//Marker[@name='{name}']") for name in MARKERS]
    if any(marker.findtext("socket_parent_frame") != "/bodyset/foot_l" for marker in markers):
        raise ValueError("All counterfactual binding stations must be in foot_l.")
    return np.asarray([[float(v) for v in marker.findtext("location").split()] for marker in markers])


def set_pose(model, state, row: dict, *, prosthetic=False, overrides=None) -> None:
    coords = model.updCoordinateSet()
    mapping = {"knee_angle_l": "pros_knee_angle", "ankle_angle_l": "pros_ankle_angle"} if prosthetic else {}
    for name, value in {**row, **(overrides or {})}.items():
        target = mapping.get(name, name)
        if coords.contains(target):
            coords.get(target).setValue(state, float(value) if target in TRANSLATIONS else np.deg2rad(float(value)), False)
    model.realizePosition(state)


def transform(body, state) -> tuple[np.ndarray, np.ndarray]:
    x = body.getTransformInGround(state)
    origin = from_vec3(x.shiftFrameStationToBase(vec3([0, 0, 0])))
    rotation = np.column_stack([from_vec3(x.shiftFrameStationToBase(vec3(np.eye(3)[i])))-origin for i in range(3)])
    return rotation, origin


def positions(rotation, origin, local) -> np.ndarray:
    return np.asarray(local) @ rotation.T + origin


def marker_errors(predicted, measured) -> dict:
    d = np.linalg.norm(np.asarray(predicted)-measured, axis=2)
    return {"frame_marker_rms_m": stats(np.sqrt(np.mean(d*d, axis=1))),
            "mean_of_per_marker_rms_m": float(np.mean(np.sqrt(np.mean(d*d, axis=0)))),
            "per_marker_distance_m": {name: stats(d[:, i]) for i, name in enumerate(MARKERS)}}


def angle_stats(matrices: np.ndarray) -> dict:
    return stats(np.rad2deg(Rotation.from_matrix(np.asarray(matrices)).magnitude()))


def static_native(subject: str, cfg: dict, raw: dict, profile) -> dict:
    trc = parse_trc(cfg["static"])
    values = raw["coordinates_deg_or_m"]
    times = np.asarray(raw["time_s"], dtype=float)
    if len(times) != len(trc.times) or any(len(v) != len(times) for v in values.values()):
        raise ValueError("Raw IK and converted static marker sample counts disagree.")
    measured = np.stack([trc.markers[name] for name in MARKERS], axis=1)
    if not np.all(np.isfinite(measured)):
        raise ValueError("Nonfinite measured static marker cloud.")
    healthy, prosthetic = osim.Model(str(cfg["healthy"])), osim.Model(str(cfg["model"]))
    hs, ps = healthy.initSystem(), prosthetic.initSystem()
    hfoot, pfoot = healthy.getBodySet().get("calcn_l"), prosthetic.getBodySet().get("foot_l")
    hmarkers = [healthy.getMarkerSet().get(name) for name in MARKERS]
    local_h = np.stack([from_vec3(marker.get_location()) for marker in hmarkers])
    parents_h = [str(marker.getSocket("parent_frame").getConnecteePath()).split("/")[-1] for marker in hmarkers]
    local_p = local_markers(cfg["model"])
    sensor_local = np.asarray([point.location for point in profile.points])
    relative_full, relative_without_subtalar, subtalar_effect, ankle_zero_relative = [], [], [], []
    rs, ts, measured_p_local, healthy_p_local, healthy_pred, registration_origin = [], [], [], [], [], []
    for i in range(len(times)):
        row = {name: v[i] for name, v in values.items()}
        set_pose(healthy, hs, row)
        rh, th = transform(hfoot, hs)
        hg = []
        for parent, loc in zip(parents_h, local_h):
            rb, tb = transform(healthy.getBodySet().get(parent), hs)
            hg.append(positions(rb, tb, loc))
        set_pose(prosthetic, ps, row, prosthetic=True)
        rp, tp = transform(pfoot, ps)
        relative_full.append(rp.T @ rh)
        registration_origin.append(rp.T @ (th-tp))
        healthy_pred.append(hg)
        rs.append(rp); ts.append(tp)
        measured_p_local.append((measured[i]-tp) @ rp)
        healthy_p_local.append((np.asarray(hg)-tp) @ rp)
        set_pose(healthy, hs, row, overrides={"subtalar_angle_l": 0.0})
        rh0, _ = transform(hfoot, hs)
        relative_without_subtalar.append(rp.T @ rh0)
        subtalar_effect.append(rh0.T @ rh)
        set_pose(healthy, hs, row, overrides={"subtalar_angle_l": 0.0, "ankle_angle_l": 0.0})
        set_pose(prosthetic, ps, row, prosthetic=True, overrides={"ankle_angle_l": 0.0})
        rha0, _ = transform(hfoot, hs); rpa0, _ = transform(pfoot, ps)
        ankle_zero_relative.append(rpa0.T @ rha0)
    rs, ts = np.stack(rs), np.stack(ts)
    measured_p_local, healthy_p_local = np.stack(measured_p_local), np.stack(healthy_p_local)
    fit_local = measured_p_local.mean(axis=0)
    fitted_prediction = np.einsum("nij,mj->nmi", rs, fit_local)+ts[:, None, :]
    neutral = {name: 0.0 for name in values}
    set_pose(healthy, hs, neutral); set_pose(prosthetic, ps, neutral, prosthetic=True)
    rn_h, tn_h = transform(hfoot, hs); rn_p, tn_p = transform(pfoot, ps)
    root_h = ET.parse(cfg["healthy"]).getroot()
    ankle_axis = np.asarray([float(x) for x in root_h.findtext(".//CustomJoint[@name='ankle_l']/SpatialTransform/TransformAxis[@name='rotation1']/axis").split()])
    remap = parse_motion(cfg["remap"])
    rename = {"pros_knee_angle": "knee_angle_l", "pros_ankle_angle": "ankle_angle_l"}
    copy_error = {name: float(np.max(np.abs(remap.values[:, j]-np.asarray(values[rename.get(name, name)])))) for j, name in enumerate(remap.labels[1:], start=1)}
    calibrated = {}
    for i, name in enumerate(MARKERS):
        registration = healthy_p_local[:, i].mean(axis=0)-local_h[i]
        measurement_bias = (measured_p_local[:, i]-healthy_p_local[:, i]).mean(axis=0)
        calibrated[name] = {
            "healthy_parent_body": parents_h[i], "healthy_original_local_m": local_h[i].tolist(),
            "operational_prosthetic_local_m": local_p[i].tolist(), "reconstructed_calibration_local_m": fit_local[i].tolist(),
            "healthy_frame_registration_contribution_m": registration.tolist(),
            "healthy_measurement_residual_contribution_m": measurement_bias.tolist(),
            "reconstructed_minus_operational_m": (fit_local[i]-local_p[i]).tolist(),
            "decomposition_identity_max_abs_m": float(np.max(np.abs(fit_local[i]-(local_h[i]+registration+measurement_bias)))),
        }
    gauge = {}
    for name, rotation, shift in (
        ("identity", np.eye(3), np.zeros(3)),
        ("foot_local_y_plus10mm", np.eye(3), np.asarray([0, .01, 0])),
        ("foot_local_y_minus10mm", np.eye(3), np.asarray([0, -.01, 0])),
        ("foot_local_pitch_plus5deg", Rotation.from_euler("z", 5, degrees=True).as_matrix(), np.zeros(3)),
    ):
        rg = rs @ rotation
        tg = ts + np.einsum("nij,j->ni", rs, shift)
        mg = (fit_local-shift) @ rotation
        prediction = np.einsum("nij,mj->nmi", rg, mg)+tg[:, None, :]
        sensors = np.einsum("nij,mj->nmi", rg, sensor_local)+tg[:, None, :]
        gauge[name] = {
            "body_local_translation_m": shift.tolist(), "body_local_rotation": rotation.tolist(),
            "counterfactual_marker_local_m": dict(zip(MARKERS, mg.tolist())),
            "marker_fit": marker_errors(prediction, measured),
            "max_marker_prediction_change_from_identity_m": float(np.max(np.abs(prediction-fitted_prediction))),
            "frozen_binary_sensor_geometry": sensor_geometry(sensors, times, profile, None),
        }
    return {
        "subject": subject, "sample_count": len(times), "raw_time_range_s": [float(times[0]), float(times[-1])],
        "converted_trc_time_range_s": [float(trc.times[0]), float(trc.times[-1])],
        "pairing": "raw IK and converted TRC matched by row; equal counts verified; historical AB06 converted time scale differs",
        "healthy_ankle_axis": ankle_axis.tolist(), "prosthetic_PinJoint_axis_local": [0, 0, 1],
        "axis_angle_to_prosthetic_z_deg": float(np.rad2deg(np.arccos(ankle_axis[2]/np.linalg.norm(ankle_axis)))),
        "copied_coordinate_max_abs_error_deg_or_m": copy_error,
        "rotation_mismatch_deg": {
            "full_healthy_calcn_vs_prosthetic_foot": angle_stats(relative_full),
            "healthy_subtalar_zero_vs_prosthetic_foot": angle_stats(relative_without_subtalar),
            "healthy_subtalar_contribution": angle_stats(subtalar_effect),
            "both_ankle_zero_and_healthy_subtalar_zero": angle_stats(ankle_zero_relative),
            "all_zero_coordinates": angle_stats([rn_p.T @ rn_h]),
        },
        "healthy_calcn_origin_in_prosthetic_foot_m": {axis: stats(np.asarray(registration_origin)[:, i]) for i, axis in enumerate("xyz")},
        "all_zero_healthy_calcn_origin_in_prosthetic_foot_m": (rn_p.T @ (tn_h-tn_p)).tolist(),
        "healthy_original_marker_fit": marker_errors(np.asarray(healthy_pred), measured),
        "reconstructed_prosthetic_marker_fit": marker_errors(fitted_prediction, measured),
        "calibration_local_decomposition": calibrated,
        "calibration_gauge_counterfactuals": gauge,
    }


def binding_transfer(subject: str, cfg: dict, local_bindings: dict, profile, stride: int) -> dict:
    static, dynamic = parse_trc(cfg["static"]), parse_trc(cfg["dynamic"])
    grf_labels, grf = storage_columns(cfg["grf"])
    windows = {
        "static": (static, np.arange(len(static.times))),
        "selected_9p01s": (dynamic, np.flatnonzero((dynamic.times >= cfg["start"]-1e-9) & (dynamic.times <= cfg["start"]+9.01+1e-9))),
        "full_trial_sampled": (dynamic, np.unique(np.r_[np.arange(0, len(dynamic.times), stride), len(dynamic.times)-1])),
    }
    sensor_local = np.asarray([point.location for point in profile.points])
    result = {}
    for window, (trc, indices) in windows.items():
        times = trc.times[indices]
        cloud = np.stack([trc.markers[name][indices] for name in MARKERS], axis=1)
        if not np.all(np.isfinite(cloud)):
            raise ValueError("Cross-binding cloud contains nonfinite values.")
        load = None if window == "static" else np.interp(times, grf[:, 0], grf[:, grf_labels.index("ground_force1_vy")])
        result[window] = {}
        for binding, local in local_bindings.items():
            result[window][binding] = {}
            for fit_name, use in (("all_five", list(range(5))), ("four_calcaneus_markers", list(range(1, 5)))):
                predictions, sensors = [], []
                for row in cloud:
                    rotation, pc, qc = rigid_transform(local, row, use)
                    predictions.append((local-pc) @ rotation.T+qc)
                    sensors.append((sensor_local-pc) @ rotation.T+qc)
                error = marker_errors(np.stack(predictions), cloud)
                d = np.linalg.norm(np.stack(predictions)-cloud, axis=2)
                error["fitted_subset_frame_marker_rms_m"] = stats(np.sqrt(np.mean(d[:, use]**2, axis=1)))
                result[window][binding][fit_name] = {"measured_data_subject": subject, "binding_subject": binding,
                    "counterfactual": subject != binding, "marker_fit": error,
                    "frozen_binary_sensor_geometry": sensor_geometry(np.stack(sensors), times, profile, load)}
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--raw-coordinates", type=Path, default=REPO/"validation/ab07_detector_investigation_2026_10_06/stage2/static_healthy_coordinates.json")
    parser.add_argument("--profile", type=Path, default=REPO/"validation/binary_phase_detector_v25_geometry_runs/2026-08-04_local_reach_sweep_dev02_04_08/selected_candidate_profile.json")
    parser.add_argument("--plugin", type=Path, default=REPO/"plugins/libSEA_Plugin_BlackBox_mCMC_impedence_ff.dylib")
    parser.add_argument("--output-dir", type=Path, default=REPO/"validation/ab07_detector_investigation_2026_10_06/stage2")
    parser.add_argument("--full-stride", type=int, default=20)
    args = parser.parse_args()
    outputs = [args.output_dir/"static_registration_identifiability.json", args.output_dir/"marker_binding_transfer.json"]
    if args.full_stride < 1 or any(path.exists() for path in outputs):
        parser.error("Positive stride and new receipt paths required; previous artifacts are never overwritten.")
    raw = json.loads(args.raw_coordinates.read_text())
    files = {"script": Path(__file__).resolve(), "raw_coordinate_export": args.raw_coordinates,
             "profile": args.profile, "plugin": args.plugin,
             "shared_diagnostic_helper": Path(__file__).with_name("audit_marker_geometry.py"),
             "calibration_helper": REPO/"tools/calibrate_ab06_seasea_markers.py",
             "fitting_helper": REPO/"tools/fit_seasea_subject.py", "builder": REPO/"tools/build_abx_seasea_pipeline.py"}
    for subject, cfg in SUBJECTS.items():
        for key, path in cfg.items():
            if isinstance(path, Path): files[f"{subject}_{key}"] = path
        files[f"{subject}_raw_static_ik"] = REPO/raw[subject]["source"]["path"]
    before = {name: source(path) for name, path in files.items()}
    for subject in SUBJECTS:
        if before[f"{subject}_raw_static_ik"]["sha256"] != raw[subject]["source"]["sha256"]:
            raise ValueError("Raw-coordinate export source hash mismatch.")
    profile = load_binary_phase_detector_profile(args.profile)
    if {point.frame for point in profile.points} != {"/bodyset/foot_l"}:
        raise ValueError("Expected frozen foot_l detector stations.")
    load_plugin(args.plugin)
    osim.Logger.setLevelString("error")
    native = {subject: static_native(subject, cfg, raw[subject], profile) for subject, cfg in SUBJECTS.items()}
    bindings = {subject: local_markers(cfg["model"]) for subject, cfg in SUBJECTS.items()}
    transfer = {subject: binding_transfer(subject, cfg, bindings, profile, args.full_stride) for subject, cfg in SUBJECTS.items()}
    if before != {name: source(path) for name, path in files.items()}:
        raise RuntimeError("An operational source changed during the diagnostic.")
    common = {"schema_version": 1, "instruction_check_token": "CMC_AGENT_OK_2026", "status": "diagnostic_completed",
              "scope": "Offline native pose setting and rigid cloud fits; zero integration or new IK; operational models, profiles and controllers unchanged",
              "source_files": before, "source_files_unchanged": True,
              "ground_origin_m": list(profile.ground.origin), "ground_normal": list(profile.ground.normal),
              "limitations": ["Free rigid poses and gauge changes need not be reachable with the existing joint chain.",
                              "Cross-subject anatomical marker bindings are deliberately counterfactual, not valid hardware registrations or proposed corrections.",
                              "Static marker RMS alone does not identify a prosthetic foot-to-ground registration when marker local coordinates can vary freely.",
                              "No contact/event, policy-transfer or biomechanical qualification is claimed."]}
    args.output_dir.mkdir(parents=True, exist_ok=True)
    outputs[0].write_text(json.dumps({**common, "subjects": native}, indent=2, allow_nan=False)+"\n")
    outputs[1].write_text(json.dumps({**common, "full_trial_stride": args.full_stride,
                                    "local_marker_bindings_m": {s: v.tolist() for s, v in bindings.items()}, "subjects": transfer}, indent=2, allow_nan=False)+"\n")
    for path in outputs: print(f"Saved diagnostic: {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
