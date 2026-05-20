#!/usr/bin/env python3
"""Calibrate AB06_SEASEA marker locations from converted EPIC data.

The AB06_SEASEA model replaces the healthy left distal chain with SEA
prosthetic bodies. Reusing the healthy marker local coordinates on those new
frames gives poor IK fits, especially around the prosthetic ankle/foot. This
script estimates marker locations in their current parent body frames from a
converted TRC file and the dataset IK coordinates, then writes a calibrated
copy of the model.
"""

from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import numpy as np
import opensim as osim


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MODEL = REPO_ROOT / "models" / "AB06_SEASEA-raw" / "osimxml" / "AB06_SEASEA.osim"
DEFAULT_PLUGIN = REPO_ROOT / "plugins" / "libSEA_Plugin_BlackBox_mCMC_impedence_ff.dylib"
DEFAULT_TRC = (
    REPO_ROOT
    / "models"
    / "AB06_SEASEA-raw"
    / "data"
    / "converted"
    / "static"
    / "static_01"
    / "static_01.trc"
)
DEFAULT_IK = DEFAULT_TRC.with_name("static_01_ik_dataset_ab06_seasea.mot")
DEFAULT_OUTPUT_MODEL = (
    REPO_ROOT / "models" / "AB06_SEASEA-raw" / "osimxml" / "AB06_SEASEA_marker_calibrated.osim"
)
DEFAULT_REPORT = DEFAULT_OUTPUT_MODEL.with_suffix(".marker_calibration.csv")

DEFAULT_MARKERS = [
    "L_Thigh_Upper",
    "L_Thigh_Front",
    "L_Thigh_Rear",
    "L_Knee_Lat",
    "L_Shank_Upper",
    "L_Shank_Front",
    "L_Shank_Rear",
    "L_Ankle_Lat",
    "L_Heel",
    "L_Toe_Lat",
    "L_Toe_Med",
    "L_Toe_Tip",
]

ROTATION_TRANSLATION_NAMES = {"pelvis_tx", "pelvis_ty", "pelvis_tz"}


@dataclass(frozen=True)
class TrcData:
    times: np.ndarray
    markers: dict[str, np.ndarray]
    units: str


@dataclass(frozen=True)
class MotionData:
    times: np.ndarray
    labels: list[str]
    values: np.ndarray
    in_degrees: bool


def resolve(path: Path) -> Path:
    return path if path.is_absolute() else (REPO_ROOT / path).resolve()


def load_plugin(plugin: Path | None) -> None:
    if not plugin:
        return
    if not plugin.exists():
        raise FileNotFoundError(f"Plugin not found: {plugin}")
    # OpenSim's Python loader expects the library path without lib prefix/suffix.
    path = plugin
    stem = path.stem
    if stem.startswith("lib"):
        stem = stem[3:]
    osim.LoadOpenSimLibrary(str(path.with_name(stem)))


def parse_trc(path: Path) -> TrcData:
    lines = path.read_text().splitlines()
    if len(lines) < 6:
        raise ValueError(f"TRC file is too short: {path}")

    header_keys = lines[1].split("\t")
    header_vals = lines[2].split("\t")
    header = dict(zip(header_keys, header_vals))
    units = header.get("Units", "mm")
    scale = 0.001 if units.lower() == "mm" else 1.0

    marker_line = lines[3].split("\t")
    marker_names = [name for name in marker_line[2:] if name]
    if not marker_names:
        raise ValueError(f"No markers found in TRC header: {path}")

    times: list[float] = []
    marker_values: dict[str, list[list[float]]] = {name: [] for name in marker_names}
    for raw in lines[5:]:
        if not raw.strip():
            continue
        parts = raw.split("\t")
        if len(parts) < 2 + 3 * len(marker_names):
            continue
        times.append(float(parts[1]))
        values = [float(value) * scale if value else math.nan for value in parts[2 : 2 + 3 * len(marker_names)]]
        for index, marker in enumerate(marker_names):
            marker_values[marker].append(values[3 * index : 3 * index + 3])

    return TrcData(
        times=np.asarray(times, dtype=float),
        markers={name: np.asarray(values, dtype=float) for name, values in marker_values.items()},
        units=units,
    )


def parse_motion(path: Path) -> MotionData:
    lines = path.read_text().splitlines()
    try:
        endheader = next(i for i, line in enumerate(lines) if line.strip().lower() == "endheader")
    except StopIteration as exc:
        raise ValueError(f"Motion file has no endheader: {path}") from exc

    in_degrees = any("indegrees=yes" in line.replace(" ", "").lower() for line in lines[:endheader])
    labels = lines[endheader + 1].split()
    data = np.loadtxt(lines[endheader + 2 :], dtype=float)
    if data.ndim == 1:
        data = data.reshape(1, -1)
    return MotionData(times=data[:, 0], labels=labels, values=data, in_degrees=in_degrees)


def sample_times(trc: TrcData, motion: MotionData, time_range: tuple[float, float] | None, stride: int) -> np.ndarray:
    start = max(float(trc.times[0]), float(motion.times[0]))
    end = min(float(trc.times[-1]), float(motion.times[-1]))
    if time_range:
        start = max(start, time_range[0])
        end = min(end, time_range[1])
    mask = (trc.times >= start - 1e-9) & (trc.times <= end + 1e-9)
    times = trc.times[mask]
    if stride > 1:
        times = times[::stride]
    if times.size == 0:
        raise ValueError(f"No overlapping samples in requested range {start} - {end}")
    return times


def interpolate_motion(motion: MotionData, time: float) -> dict[str, float]:
    row: dict[str, float] = {}
    for col, label in enumerate(motion.labels[1:], start=1):
        row[label] = float(np.interp(time, motion.times, motion.values[:, col]))
    return row


def interpolate_marker(trc: TrcData, marker: str, time: float) -> np.ndarray:
    xyz = trc.markers[marker]
    return np.asarray([np.interp(time, trc.times, xyz[:, axis]) for axis in range(3)], dtype=float)


def vec3(values: Iterable[float]) -> osim.Vec3:
    a, b, c = values
    return osim.Vec3(float(a), float(b), float(c))


def from_vec3(value: osim.Vec3) -> np.ndarray:
    return np.asarray([value.get(0), value.get(1), value.get(2)], dtype=float)


def set_state_from_motion(model: osim.Model, state: osim.State, motion_row: dict[str, float], in_degrees: bool) -> None:
    coordinates = model.updCoordinateSet()
    for name, value in motion_row.items():
        if not coordinates.contains(name):
            continue
        coord = coordinates.get(name)
        if in_degrees and name not in ROTATION_TRANSLATION_NAMES:
            value = math.radians(value)
        coord.setValue(state, value, False)
    model.realizePosition(state)


def marker_parent_body_name(marker: osim.Marker) -> str:
    path = marker.getSocket("parent_frame").getConnecteePath()
    return str(path).rstrip("/").split("/")[-1]


def calibrate_markers(
    model_path: Path,
    trc_path: Path,
    ik_path: Path,
    output_model: Path,
    report_path: Path,
    plugin: Path | None,
    markers: list[str],
    time_range: tuple[float, float] | None,
    stride: int,
) -> None:
    load_plugin(plugin)
    trc = parse_trc(trc_path)
    motion = parse_motion(ik_path)
    times = sample_times(trc, motion, time_range, stride)

    model = osim.Model(str(model_path))
    state = model.initSystem()
    marker_set = model.updMarkerSet()
    body_set = model.getBodySet()

    rows: list[dict[str, object]] = []
    for marker_name in markers:
        if marker_name not in trc.markers:
            raise ValueError(f"Marker {marker_name} not found in TRC: {trc_path}")
        if not marker_set.contains(marker_name):
            raise ValueError(f"Marker {marker_name} not found in model: {model_path}")

        marker = marker_set.get(marker_name)
        body_name = marker_parent_body_name(marker)
        body = body_set.get(body_name)
        old_location = from_vec3(marker.get_location())

        local_samples: list[np.ndarray] = []
        old_errors: list[float] = []
        for time in times:
            set_state_from_motion(model, state, interpolate_motion(motion, float(time)), motion.in_degrees)
            measured_ground = interpolate_marker(trc, marker_name, float(time))
            if np.any(~np.isfinite(measured_ground)):
                continue
            transform = body.getTransformInGround(state)
            local = from_vec3(transform.shiftBaseStationToFrame(vec3(measured_ground)))
            old_ground = from_vec3(transform.shiftFrameStationToBase(vec3(old_location)))
            local_samples.append(local)
            old_errors.append(float(np.linalg.norm(old_ground - measured_ground)))

        if not local_samples:
            raise ValueError(f"No finite samples for marker {marker_name}")

        local_array = np.vstack(local_samples)
        new_location = np.mean(local_array, axis=0)
        fit_errors = np.linalg.norm(local_array - new_location, axis=1)
        marker.set_location(vec3(new_location))
        rows.append(
            {
                "marker": marker_name,
                "parent_body": body_name,
                "samples": len(local_samples),
                "old_location_x": old_location[0],
                "old_location_y": old_location[1],
                "old_location_z": old_location[2],
                "new_location_x": new_location[0],
                "new_location_y": new_location[1],
                "new_location_z": new_location[2],
                "old_mean_error_m": float(np.mean(old_errors)),
                "new_fit_rms_m": float(np.sqrt(np.mean(fit_errors**2))),
                "new_fit_max_m": float(np.max(fit_errors)),
                "local_std_x": float(np.std(local_array[:, 0])),
                "local_std_y": float(np.std(local_array[:, 1])),
                "local_std_z": float(np.std(local_array[:, 2])),
            }
        )

    output_model.parent.mkdir(parents=True, exist_ok=True)
    report_path.parent.mkdir(parents=True, exist_ok=True)
    model.printToXML(str(output_model))

    with report_path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    old_mean = float(np.mean([row["old_mean_error_m"] for row in rows]))
    new_mean = float(np.mean([row["new_fit_rms_m"] for row in rows]))
    print(f"Wrote calibrated model: {output_model}")
    print(f"Wrote marker report: {report_path}")
    print(f"Calibration samples: {len(times)} from {times[0]:.6f} to {times[-1]:.6f} s")
    print(f"Mean old marker error: {old_mean:.6f} m")
    print(f"Mean calibrated local-fit RMS: {new_mean:.6f} m")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--trc", type=Path, default=DEFAULT_TRC)
    parser.add_argument("--ik", type=Path, default=DEFAULT_IK)
    parser.add_argument("--output-model", type=Path, default=DEFAULT_OUTPUT_MODEL)
    parser.add_argument("--report", type=Path, default=DEFAULT_REPORT)
    parser.add_argument("--plugin", type=Path, default=DEFAULT_PLUGIN)
    parser.add_argument("--markers", nargs="+", default=DEFAULT_MARKERS)
    parser.add_argument("--time-range", type=float, nargs=2, metavar=("START", "END"))
    parser.add_argument("--stride", type=int, default=1)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    calibrate_markers(
        resolve(args.model),
        resolve(args.trc),
        resolve(args.ik),
        resolve(args.output_model),
        resolve(args.report),
        resolve(args.plugin) if args.plugin else None,
        list(args.markers),
        tuple(args.time_range) if args.time_range else None,
        max(1, args.stride),
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
