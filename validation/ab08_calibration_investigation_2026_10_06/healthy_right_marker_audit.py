"""Evaluate saved healthy right-leg poses; no IK solver or integration."""
from pathlib import Path
import hashlib
import json
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from tools.calibrate_ab06_seasea_markers import (
    from_vec3, load_plugin, parse_motion, parse_trc, set_state_from_motion,
)
import opensim as osim


def stats(values):
    a = np.asarray(values)
    return {"mean": float(a.mean()), "rms": float(np.sqrt(np.mean(a * a))),
            "median": float(np.median(a)), "p95": float(np.percentile(a, 95)),
            "min": float(a.min()), "max": float(a.max())}


def main():
    base = ROOT / "models/AB08_SEASEA-raw/data/converted"
    trc_file = base / "treadmill/treadmill_01_01/treadmill_01_01.trc"
    motion_file = trc_file.with_name("treadmill_01_01_ik_dataset_ab06_seasea.mot")
    static_file = base / "static/static01/static01.trc"
    healthy_file = ROOT / "models/AB08-raw/osimxml/AB08.osim"
    plugin = ROOT / "plugins/libSEA_Plugin_BlackBox_mCMC_impedence_ff.dylib"
    sources = [trc_file, motion_file, static_file, healthy_file, plugin, Path(__file__)]
    hashes = {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}
    trc, static, motion = parse_trc(trc_file), parse_trc(static_file), parse_motion(motion_file)
    if not np.allclose(trc.times, motion.times, rtol=0, atol=1e-9):
        raise ValueError("Trial TRC and dataset IK clocks differ")
    names = ["R_Thigh_Upper", "R_Thigh_Front", "R_Thigh_Rear", "R_Knee_Lat"]
    load_plugin(plugin)
    osim.Logger.setLevelString("error")
    osim.Logger.removeFileSink()
    model = osim.Model(str(healthy_file))
    state = model.initSystem()
    markers = [model.getMarkerSet().get(n) for n in names]
    # Subject dataset was exported with prosthetic coordinate names. Invert
    # only these two aliases; right marker poses do not depend on omitted
    # left subtalar/MTP coordinates.
    rename = {"pros_knee_angle": "knee_angle_l", "pros_ankle_angle": "ankle_angle_l"}
    labels = [rename.get(n, n) for n in motion.labels[1:]]
    peak_index = int(np.argmin(np.abs(trc.times - 56.635)))
    indices = sorted(set(range(0, len(trc.times), 20)) | set(range(max(0, peak_index - 60), min(len(trc.times), peak_index + 61))))
    errors = np.zeros((len(indices), len(names)))
    measured_local = np.zeros((len(indices), len(names), 3))
    for k, idx in enumerate(indices):
        set_state_from_motion(model, state, dict(zip(labels, motion.values[idx, 1:])), motion.in_degrees)
        for j, marker in enumerate(markers):
            predicted = from_vec3(marker.getLocationInGround(state))
            measured = trc.markers[names[j]][idx]
            errors[k, j] = np.linalg.norm(predicted - measured)
            parent = marker.getParentFrame()
            measured_local[k, j] = from_vec3(parent.getTransformInGround(state).shiftBaseStationToFrame(osim.Vec3(*map(float, measured))))
    exact_k = indices.index(peak_index)
    pairs = {}
    for a, b in [(names[0], names[1]), (names[0], names[2]), (names[1], names[2]), (names[0], names[3])]:
        trial = np.linalg.norm(trc.markers[a] - trc.markers[b], axis=1)
        reference = np.linalg.norm(static.markers[a] - static.markers[b], axis=1)
        pairs[a + "--" + b] = {"static_m": stats(reference), "trial_m": stats(trial),
            "at_peak_m": float(trial[peak_index]), "at_peak_delta_from_static_median_m": float(trial[peak_index] - np.median(reference)),
            "range_peak_window_m": float(np.ptp(trial[max(0, peak_index - 10):peak_index + 11]))}
    time_series = []
    for idx in range(max(1, peak_index - 4), min(len(trc.times) - 1, peak_index + 5)):
        time_series.append({"time_s": float(trc.times[idx]),
            "upper_xyz_m": trc.markers[names[0]][idx].tolist(),
            "upper_displacement_from_previous_m": float(np.linalg.norm(trc.markers[names[0]][idx] - trc.markers[names[0]][idx - 1])),
            "front_displacement_from_previous_m": float(np.linalg.norm(trc.markers[names[1]][idx] - trc.markers[names[1]][idx - 1])),
            "rear_displacement_from_previous_m": float(np.linalg.norm(trc.markers[names[2]][idx] - trc.markers[names[2]][idx - 1]))})
    output = {"scope": "Read-only native healthy-model pose evaluation on exported dataset IK, sampled every20frames plus121frames around peak; no solver/dynamics/policy",
        "sources": hashes, "trial_rows": len(trc.times), "sampled_pose_count": len(indices),
        "sampled_errors_m": {n: stats(errors[:, j]) for j, n in enumerate(names)},
        "native_model_marker_locations_m": {n: from_vec3(m.get_location()).tolist() for n, m in zip(names, markers)},
        "peak_native_healthy_pose": {"time_s": float(trc.times[peak_index]),
            "errors_m": dict(zip(names, errors[exact_k].tolist())),
            "measured_local_positions_m": {n: measured_local[exact_k, j].tolist() for j, n in enumerate(names)}},
        "raw_pairwise_distances": pairs, "raw_marker_peak_window": time_series,
        "limitations": ["Sampled statistics are not full-trial IK error metrics.", "Unprovided left subtalar/MTP values retain healthy defaults; they do not affect right marker frames.", "Raw pair-distance changes establish nonrigid marker-cluster behavior, not its physical or acquisition cause."]}
    output["sources_unchanged"] = all(hashlib.sha256((ROOT / p).read_bytes()).hexdigest() == digest for p, digest in hashes.items())
    destination = Path(__file__).with_name("healthy_right_marker_audit.json")
    destination.write_text(json.dumps(output, indent=2, allow_nan=False) + "\n")
    print(json.dumps({"receipt": str(destination.relative_to(ROOT)), "sampled_pose_count": len(indices),
        "peak_healthy_errors_m": output["peak_native_healthy_pose"]["errors_m"],
        "pair_distance_deltas_at_peak_m": {n: v["at_peak_delta_from_static_median_m"] for n, v in pairs.items()},
        "sources_unchanged": output["sources_unchanged"]}, indent=2))


if __name__ == "__main__":
    main()
