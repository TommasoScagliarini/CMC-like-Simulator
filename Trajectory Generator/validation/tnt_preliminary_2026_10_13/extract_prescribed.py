"""Extract force-free, prescribed kinematics for an offline TNT replay.

Run with a Python environment containing OpenSim and NumPy. All default paths
are repository relative. Only the rollout receipt, its model, its prescribed
IK file, and the SEA plugin are used; no force, contact, policy, or reward data
are read. This is forward kinematics, not a dynamic rollout.
"""
from __future__ import annotations

import argparse
import hashlib
import io
import json
import platform
from pathlib import Path

import numpy as np
import opensim


HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]
RUNS = {
    "ab06": "MLP_V26C_best47_AB06_control_2026_10_06_5s",
    "ab07": "MLP_V26C_best47_AB07_pylon400_2026_10_07_6p11s",
}


def local_path(value: str) -> Path:
    """Rebase the historical absolute receipt paths onto this checkout."""
    normalized = value.replace("\\", "/")
    for anchor in ("/Trajectory Generator/", "/validation/", "/models/"):
        if anchor in normalized:
            return REPO / (anchor[1:] + normalized.split(anchor, 1)[1])
    path = Path(value)
    return path if path.is_absolute() else REPO / path


def fingerprint(path: Path) -> dict:
    return {
        "path": path.relative_to(REPO).as_posix() if path.is_relative_to(REPO) else str(path),
        "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
    }


def read_coordinates(path: Path):
    lines = path.read_text(encoding="utf-8-sig").splitlines()
    end = next(i for i, line in enumerate(lines) if line.strip().lower() == "endheader")
    names = lines[end + 1].split()
    values = np.loadtxt(io.StringIO("\n".join(lines[end + 2:])))
    degrees = any(line.lower().replace(" ", "") == "indegrees=yes" for line in lines[:end])
    if names[0] != "time" or values.shape[1] != len(names):
        raise ValueError(f"Invalid coordinate table: {path}")
    if not np.isfinite(values).all() or not np.all(np.diff(values[:, 0]) > 0):
        raise ValueError(f"Nonfinite or unordered coordinates: {path}")
    return names, values, degrees


def vector(value):
    return np.array([value.get(i) for i in range(3)])


def extract(subject: str, output_dir: Path, heading_seconds: float, rate_hz: float, plugin: Path):
    receipt_path = REPO / "Trajectory Generator" / "runs" / "rollout" / RUNS[subject] / "receipt.json"
    config = json.loads(receipt_path.read_text())["effective_simulator_config"]
    model_path = local_path(config["model_file"])
    ik_path = local_path(config["kinematics_file"])
    names, source, degrees = read_coordinates(ik_path)
    dt = float(np.median(np.diff(source[:, 0])))
    stride = int(round(1.0 / rate_hz / dt))
    if stride < 1 or not np.isclose(stride * dt, 1.0 / rate_hz, atol=1e-8):
        raise ValueError("Requested rate must be an integer decimation of source rate")
    if not np.allclose(np.diff(source[:, 0]), dt, atol=1e-8):
        raise ValueError("The source must have uniform sampling")
    samples = source[::stride]
    time = samples[:, 0]
    if heading_seconds > 0.30 * (time[-1] - time[0]):
        raise ValueError("Heading calibration must fit within the first 30% calibration split")
    model = opensim.Model(str(model_path))
    state = model.initSystem()
    coordinate_set = model.getCoordinateSet()
    coordinates = {coordinate_set.get(i).getName(): coordinate_set.get(i)
                   for i in range(coordinate_set.getSize())}
    missing = set(coordinates) - set(names)
    if missing:
        raise ValueError(f"Missing coordinates: {sorted(missing)}")
    columns = {name: names.index(name) for name in coordinates}
    translations = set(config["translation_coords"])
    thigh = model.getBodySet().get("transfemur")
    pelvis = model.getBodySet().get("pelvis")
    axes = np.empty((len(samples), 3))
    heading_samples = []
    for index, row in enumerate(samples):
        state.setTime(float(row[0]))
        for name, coordinate in coordinates.items():
            value = float(row[columns[name]])
            if degrees and name not in translations:
                value = np.deg2rad(value)
            coordinate.setValue(state, value, False)
        model.realizePosition(state)
        axes[index] = vector(thigh.expressVectorInGround(state, opensim.Vec3(0, -1, 0)))
        if row[0] < time[0] + heading_seconds:
            heading_samples.append(vector(pelvis.expressVectorInGround(state, opensim.Vec3(1, 0, 0))))
    forward = np.mean(heading_samples, axis=0)
    forward[1] = 0.0
    if np.linalg.norm(forward) < 1e-8:
        raise ValueError("Undefined horizontal heading")
    forward /= np.linalg.norm(forward)
    theta = np.rad2deg(np.unwrap(np.arctan2(axes @ forward, -axes[:, 1])))
    velocity = np.zeros_like(theta)
    velocity[1:] = np.diff(theta) / np.diff(time)
    knee = samples[:, names.index("pros_knee_angle")]
    ankle = samples[:, names.index("pros_ankle_angle")]
    if not degrees:
        knee, ankle = np.rad2deg(knee), np.rad2deg(ankle)
    output = output_dir / f"{subject}_prescribed.csv"
    np.savetxt(output, np.column_stack((time, theta, knee, ankle, velocity)),
               delimiter=",", header="time_s,thigh_deg,knee_deg,ankle_deg,thigh_velocity_deg_s",
               comments="", fmt="%.10f")
    duration = float(time[-1] - time[0])
    result = {
        "schema_version": 1,
        "subject": subject.upper(),
        "instruction_check_token": "CMC_AGENT_OK_2026",
        "scope": "Offline prescribed forward kinematics only; no GRF or contact data read; no integration or training",
        "inputs": {"rollout_receipt": fingerprint(receipt_path), "model": fingerprint(model_path),
                   "prescribed_ik": fingerprint(ik_path), "extractor": fingerprint(Path(__file__)),
                   "sea_plugin": fingerprint(plugin)},
        "output": fingerprint(output),
        "columns": {"time_s": "s", "thigh_deg": "degree", "knee_deg": "degree",
                    "ankle_deg": "degree", "thigh_velocity_deg_s": "degree/s"},
        "source_samples": len(source),
        "source_dt_s": dt,
        "source_time_range_s": [float(source[0, 0]), float(source[-1, 0])],
        "output_samples": len(time),
        "output_dt_s": float(1.0 / rate_hz),
        "output_time_range_s": [float(time[0]), float(time[-1])],
        "resampling": {"method": "Integer decimation without interpolation or filtering", "stride": stride},
        "angle_definition": {
            "body": "transfemur", "body_longitudinal_axis": [0, -1, 0],
            "vertical_ground": [0, 1, 0], "forward_ground": forward.tolist(),
            "equation": "theta = atan2(body_long_axis_ground dot forward_ground, -body_long_axis_ground_y)",
            "sign": "Distal thigh anterior to hip gives positive theta; knee/ankle preserve OpenSim coordinate signs",
            "heading_calibration": "Mean horizontal projection of pelvis +X from first heading_seconds only, frozen thereafter",
            "heading_seconds": heading_seconds,
            "heading_samples": len(heading_samples),
            "heading_calibration_interval_s": [float(time[0]), float(time[0] + heading_seconds)],
        },
        "velocity": "Backward first difference at output rate; first sample initialized to zero and should not drive a transition",
        "kinematic_processing": "Raw prescribed IK coordinates, no additional smoothing and no future samples for velocity",
        "proposed_temporal_split": {
            "calibration_fraction": 0.30,
            "calibration_end_s": float(time[0] + 0.30 * duration),
            "holdout_start_s": float(time[0] + 0.30 * duration),
            "constraint": "All heading samples belong to calibration; estimator and template calibration must not use held-out samples",
        },
        "ranges_deg": {"thigh": [float(theta.min()), float(theta.max())],
                       "knee": [float(knee.min()), float(knee.max())],
                       "ankle": [float(ankle.min()), float(ankle.max())]},
        "runtime": {"python": platform.python_version(), "platform": platform.platform(),
                    "opensim": opensim.GetVersion()},
        "limitations": [
            "This does not establish dynamic stability, force support, or response to actuator error.",
            "IK and model retain their original upstream experimental provenance; this extraction does not access force measurements.",
            "The first heading_seconds are an offline mounting/heading calibration and cannot be scored as held-out online estimates.",
            "Decimation is intentional for this preliminary raw-IK replay; real sensor bandwidth/noise is not reproduced.",
        ],
    }
    (output_dir / f"{subject}_extraction_receipt.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps({"subject": subject, "output": str(output), "samples": len(time),
                      "time_range_s": result["output_time_range_s"], "ranges_deg": result["ranges_deg"]}))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--subject", choices=["ab06", "ab07", "both"], default="both")
    parser.add_argument("--output-dir", type=Path, default=HERE)
    parser.add_argument("--heading-seconds", type=float, default=5.0)
    parser.add_argument("--rate-hz", type=float, default=100.0)
    parser.add_argument("--plugin", type=Path,
                        default=REPO / "plugins" / "SEA_Plugin_BlackBox_mCMC_impedence_ff")
    args = parser.parse_args()
    if args.heading_seconds <= 0 or args.rate_hz <= 0:
        parser.error("Heading duration and sample rate must be positive")
    args.output_dir.mkdir(parents=True, exist_ok=True)
    opensim.Logger.setLevelString("error")
    candidates = [args.plugin]
    platform_suffix = {"Darwin": ".dylib", "Windows": ".dll"}.get(platform.system(), ".so")
    for suffix in (platform_suffix,):
        candidates += [args.plugin.with_name(args.plugin.name + suffix),
                       args.plugin.with_name("lib" + args.plugin.name + suffix)]
    plugin = next((path for path in candidates if path.is_file()), None)
    if plugin is None:
        raise FileNotFoundError(f"No platform plugin found for {args.plugin}; supply --plugin")
    # The OpenSim Python wrapper returns None on a successful library load.
    load_stem = plugin.with_suffix("")
    if load_stem.name.startswith("lib"):
        load_stem = load_stem.with_name(load_stem.name[3:])
    opensim.LoadOpenSimLibrary(str(load_stem))
    for subject in RUNS if args.subject == "both" else [args.subject]:
        extract(subject, args.output_dir, args.heading_seconds, args.rate_hz, plugin)


if __name__ == "__main__":
    main()
