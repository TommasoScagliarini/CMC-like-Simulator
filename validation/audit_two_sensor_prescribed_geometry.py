"""Geometric V4 design audit for the prescribed heel/toe detector.

This is deliberately a diagnostic, not a detector candidate or a validation
gate.  It reuses only the already-open V3 validation block (50--100 s), keeps
the 100--155.045 s block sealed, and never runs a policy or changes the runtime
FSM/profile.  The audit answers two narrower questions:

1. are the current virtual heel/toe spheres attached plausibly to ``foot_l``;
2. at prescribed initial contact, where are the force-plate COP and the two
   virtual sensor contacts relative to the prosthetic foot?
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import re
import struct
import sys
import xml.etree.ElementTree as ET
from dataclasses import replace
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
TRAJECTORY_ROOT = REPO_ROOT / "Trajectory Generator"
for path in (REPO_ROOT, TRAJECTORY_ROOT):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from online_grf import OnlineGRFSphere, load_online_grf_profile  # noqa: E402
from path_resolver import resolve_repo_path  # noqa: E402
from setup_io import read_setup_xml  # noqa: E402
from validation.validate_online_grf import (  # noqa: E402
    _external_wrench,
    _sample_spheres,
)
from validation.validate_two_sensor_prescribed_replay import (  # noqa: E402
    DEFAULT_PROFILE,
    DEFAULT_SEA_PLUGIN,
    DEFAULT_SETUP,
    _left_sensor_spheres,
    _reference_events_from_prescribed_grf,
    _regional_loads_and_penetrations,
)


DEFAULT_V3_MANIFEST = (
    "validation/two_sensor_prescribed_threshold_sweep_runs/"
    "2026-07-21_fullspan_v3_confirmed/validation/manifest.json"
)
DEFAULT_OUTPUT_DIR = (
    "validation/two_sensor_geometry_audit_runs/"
    "2026-07-21_v4_design_audit"
)
DEFAULT_PLOT_DIR = "plot/07_21_2026_two_sensor_geometry_audit_v4"
V3_SELECTED_CANDIDATE = "on00p50_off00p25"
UNSEALED_START_S = 50.0
UNSEALED_END_S = 100.0
SEALED_START_S = 100.0


def _portable_path(path: Path) -> str:
    resolved = path.resolve()
    try:
        return resolved.relative_to(REPO_ROOT.resolve()).as_posix()
    except ValueError:
        return resolved.as_posix()


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _source_record(path: Path) -> dict[str, Any]:
    resolved = path.resolve()
    if not resolved.is_file():
        raise FileNotFoundError(resolved)
    return {
        "path": _portable_path(resolved),
        "sha256": _sha256(resolved),
        "bytes": int(resolved.stat().st_size),
    }


def _json_safe(value: Any) -> Any:
    if isinstance(value, Mapping):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    if isinstance(value, np.ndarray):
        return _json_safe(value.tolist())
    if isinstance(value, (np.floating, float)):
        number = float(value)
        return number if math.isfinite(number) else None
    if isinstance(value, (np.bool_, bool)):
        return bool(value)
    if isinstance(value, (np.integer, int)):
        return int(value)
    return value


def _validate_unsealed_window(start_s: float, end_s: float) -> None:
    if not (math.isfinite(start_s) and math.isfinite(end_s)):
        raise ValueError("audit time bounds must be finite")
    if start_s < UNSEALED_START_S - 1e-12 or end_s > UNSEALED_END_S + 1e-12:
        raise ValueError(
            "V4 design audit is restricted to the already-open 50--100 s "
            "block; the 100--155.045 s block remains sealed"
        )
    if end_s <= start_s:
        raise ValueError("audit end time must be greater than start time")


def _load_stl_triangles(path: Path) -> np.ndarray:
    """Read a binary or ASCII STL without adding a mesh dependency."""
    raw = path.read_bytes()
    triangles: np.ndarray | None = None
    if len(raw) >= 84:
        count = struct.unpack_from("<I", raw, 80)[0]
        expected = 84 + 50 * count
        if count > 0 and expected == len(raw):
            dtype = np.dtype(
                [
                    ("normal", "<f4", (3,)),
                    ("vertices", "<f4", (3, 3)),
                    ("attribute", "<u2"),
                ]
            )
            triangles = np.frombuffer(raw, dtype=dtype, offset=84, count=count)[
                "vertices"
            ].astype(float)
    if triangles is None:
        text = raw.decode("ascii", errors="strict")
        pattern = re.compile(
            r"\bvertex\s+"
            r"([-+0-9.eE]+)\s+([-+0-9.eE]+)\s+([-+0-9.eE]+)"
        )
        vertices = np.asarray(
            [[float(item) for item in match] for match in pattern.findall(text)],
            dtype=float,
        )
        if len(vertices) == 0 or len(vertices) % 3 != 0:
            raise ValueError(f"invalid STL triangle payload: {path}")
        triangles = vertices.reshape(-1, 3, 3)
    if triangles.ndim != 3 or triangles.shape[1:] != (3, 3):
        raise ValueError(f"invalid STL shape {triangles.shape}: {path}")
    if not np.all(np.isfinite(triangles)):
        raise ValueError(f"non-finite STL vertices: {path}")
    return triangles


def _closest_point_on_triangle(point: np.ndarray, triangle: np.ndarray) -> np.ndarray:
    """Closest point using the Voronoi-region construction from RTCD."""
    a, b, c = np.asarray(triangle, dtype=float)
    p = np.asarray(point, dtype=float)
    ab = b - a
    ac = c - a
    ap = p - a
    d1 = float(ab @ ap)
    d2 = float(ac @ ap)
    if d1 <= 0.0 and d2 <= 0.0:
        return a
    bp = p - b
    d3 = float(ab @ bp)
    d4 = float(ac @ bp)
    if d3 >= 0.0 and d4 <= d3:
        return b
    vc = d1 * d4 - d3 * d2
    if vc <= 0.0 and d1 >= 0.0 and d3 <= 0.0:
        return a + (d1 / (d1 - d3)) * ab
    cp = p - c
    d5 = float(ab @ cp)
    d6 = float(ac @ cp)
    if d6 >= 0.0 and d5 <= d6:
        return c
    vb = d5 * d2 - d1 * d6
    if vb <= 0.0 and d2 >= 0.0 and d6 <= 0.0:
        return a + (d2 / (d2 - d6)) * ac
    va = d3 * d6 - d5 * d4
    if va <= 0.0 and (d4 - d3) >= 0.0 and (d5 - d6) >= 0.0:
        edge = c - b
        return b + ((d4 - d3) / ((d4 - d3) + (d5 - d6))) * edge
    denominator = va + vb + vc
    if abs(denominator) <= 1e-18:
        # Degenerate triangle: its vertices are still a conservative fallback.
        return min((a, b, c), key=lambda item: float(np.linalg.norm(p - item)))
    v = vb / denominator
    w = vc / denominator
    return a + v * ab + w * ac


def _minimum_mesh_distance(point: Sequence[float], triangles: np.ndarray) -> float:
    p = np.asarray(point, dtype=float)
    return float(
        min(
            np.linalg.norm(p - _closest_point_on_triangle(p, triangle))
            for triangle in np.asarray(triangles, dtype=float)
        )
    )


def _resolve_case_insensitive(parent: Path, filename: str) -> Path:
    direct = parent / filename
    if direct.is_file():
        return direct.resolve()
    matches = [item for item in parent.iterdir() if item.name.lower() == filename.lower()]
    if len(matches) != 1:
        raise FileNotFoundError(
            f"could not resolve exactly one case-insensitive {filename!r} in {parent}"
        )
    return matches[0].resolve()


def _resolve_left_foot_mesh(model_path: Path) -> Path:
    root = ET.parse(model_path).getroot()
    bodies = [item for item in root.iter() if item.tag == "Body" and item.get("name") == "foot_l"]
    if len(bodies) != 1:
        raise ValueError("model must contain exactly one Body named foot_l")
    mesh_names = [
        (item.text or "").strip()
        for item in bodies[0].iter("mesh_file")
        if (item.text or "").strip()
    ]
    if len(mesh_names) != 1:
        raise ValueError(f"foot_l must declare exactly one mesh: {mesh_names}")
    search_dirs = (model_path.parent / "Geometry", REPO_ROOT / "Geometry")
    for directory in search_dirs:
        if directory.is_dir():
            try:
                return _resolve_case_insensitive(directory, mesh_names[0])
            except FileNotFoundError:
                continue
    raise FileNotFoundError(f"foot_l mesh not found: {mesh_names[0]}")


def _matching_model_markers(
    model_path: Path,
    sensors: Mapping[str, OnlineGRFSphere],
) -> dict[str, list[str]]:
    root = ET.parse(model_path).getroot()
    markers: list[tuple[str, str, np.ndarray]] = []
    for marker in root.iter("Marker"):
        frame = marker.findtext("socket_parent_frame", default="").strip()
        location_text = marker.findtext("location", default="").strip()
        if not location_text:
            continue
        location = np.fromstring(location_text, sep=" ", dtype=float)
        if location.shape == (3,):
            markers.append((str(marker.get("name", "")), frame, location))
    return {
        role: sorted(
            name
            for name, frame, location in markers
            if frame == sphere.frame
            and np.allclose(
                location,
                np.asarray(sphere.location, dtype=float),
                rtol=0.0,
                atol=1e-9,
            )
        )
        for role, sphere in sensors.items()
    }


def _static_sensor_geometry(
    triangles: np.ndarray,
    sensors: Mapping[str, OnlineGRFSphere],
) -> dict[str, Any]:
    vertices = np.asarray(triangles, dtype=float).reshape(-1, 3)
    bounds_min = np.min(vertices, axis=0)
    bounds_max = np.max(vertices, axis=0)
    result: dict[str, Any] = {
        "mesh_bounds_min_m": bounds_min,
        "mesh_bounds_max_m": bounds_max,
        "mesh_dimensions_m": bounds_max - bounds_min,
        "sensors": {},
    }
    for role, sphere in sensors.items():
        center = np.asarray(sphere.location, dtype=float)
        radius = float(sphere.radius)
        distance = _minimum_mesh_distance(center, triangles)
        result["sensors"][role] = {
            "name": sphere.name,
            "frame": sphere.frame,
            "location_m": center,
            "radius_m": radius,
            "center_to_mesh_surface_m": distance,
            "sphere_surface_gap_to_mesh_m": max(0.0, distance - radius),
            "sphere_mesh_overlap_depth_m": max(0.0, radius - distance),
            "local_bottom_y_m": float(center[1] - radius),
            "below_mesh_global_y_min_m": max(
                0.0, float(bounds_min[1] - (center[1] - radius))
            ),
            "center_inside_mesh_axis_aligned_bounds": bool(
                np.all(center >= bounds_min) and np.all(center <= bounds_max)
            ),
        }
    heel_bottom = float(result["sensors"]["left_heel"]["local_bottom_y_m"])
    toe_bottom = float(result["sensors"]["left_toe"]["local_bottom_y_m"])
    toe_gap = float(
        result["sensors"]["left_toe"]["sphere_surface_gap_to_mesh_m"]
    )
    result["heel_toe_bottom_offset_m"] = toe_bottom - heel_bottom
    result["diagnostic_checks"] = {
        "toe_sphere_within_5mm_of_mesh": bool(toe_gap <= 0.005),
        "heel_toe_bottoms_within_20mm": bool(abs(toe_bottom - heel_bottom) <= 0.020),
    }
    result["geometry_plausible_for_semantic_decision"] = bool(
        all(result["diagnostic_checks"].values())
    )
    return result


def _derive_critical_hs(
    manifest: Mapping[str, Any],
    *,
    candidate_id: str = V3_SELECTED_CANDIDATE,
    tolerance_s: float = 0.050,
) -> list[dict[str, Any]]:
    detail = manifest["primary_candidate_details"][candidate_id]
    reference = np.asarray(detail["events"]["reference"]["heel_strike"], dtype=float)
    predicted = np.asarray(detail["events"]["primary"]["heel_strike"], dtype=float)
    if reference.ndim != 1 or predicted.ndim != 1 or len(reference) == 0:
        raise ValueError("V3 manifest contains invalid heel-strike arrays")
    critical: list[dict[str, Any]] = []
    for index in range(min(len(reference), len(predicted))):
        error = float(predicted[index] - reference[index])
        if abs(error) > float(tolerance_s):
            critical.append(
                {
                    "reference_index": index,
                    "reference_time_s": float(reference[index]),
                    "kind": "late_or_early_primary_event",
                    "primary_time_s": float(predicted[index]),
                    "primary_error_s": error,
                }
            )
    if len(predicted) < len(reference):
        # Only the first missing event is an independent root cause; later
        # missing events occur after the latched V3 timeout.
        index = len(predicted)
        critical.append(
            {
                "reference_index": index,
                "reference_time_s": float(reference[index]),
                "kind": "first_missing_before_timeout_cascade",
                "primary_time_s": None,
                "primary_error_s": None,
            }
        )
    return critical


def _diagnostic_sampling_profile(profile: Any) -> tuple[Any, dict[str, str]]:
    left_sensor = _left_sensor_spheres(profile)["left_heel"]
    stations = {
        "origin": (0.0, 0.0, 0.0),
        "x": (0.1, 0.0, 0.0),
        "y": (0.0, 0.1, 0.0),
        "z": (0.0, 0.0, 0.1),
    }
    names = {key: f"__audit_foot_l_{key}" for key in stations}
    extra = tuple(
        replace(
            left_sensor,
            name=names[key],
            location=location,
            radius=0.001,
        )
        for key, location in stations.items()
    )
    return replace(profile, spheres=tuple(profile.spheres) + extra), names


def _foot_frame_kinematics(
    samples: Mapping[str, Any],
    station_names: Mapping[str, str],
) -> tuple[np.ndarray, np.ndarray]:
    origin = np.asarray(samples["centers"][station_names["origin"]], dtype=float)
    axes = []
    for key in ("x", "y", "z"):
        vector = (
            np.asarray(samples["centers"][station_names[key]], dtype=float) - origin
        )
        norm = np.linalg.norm(vector, axis=1)
        if np.any(norm <= 1e-12):
            raise ValueError(f"degenerate foot_l {key} axis")
        axes.append(vector / norm[:, None])
    rotation = np.stack(axes, axis=2)
    orthogonality = np.einsum("nji,njk->nik", rotation, rotation)
    identity = np.eye(3)[None, :, :]
    if float(np.max(np.abs(orthogonality - identity))) > 1e-8:
        raise ValueError("sampled foot_l frame is not orthonormal")
    return origin, rotation


def _points_in_local_frame(
    points_ground: np.ndarray,
    origins_ground: np.ndarray,
    rotations_local_to_ground: np.ndarray,
) -> np.ndarray:
    points = np.asarray(points_ground, dtype=float)
    origins = np.asarray(origins_ground, dtype=float)
    rotations = np.asarray(rotations_local_to_ground, dtype=float)
    return np.einsum("nji,nj->ni", rotations, points - origins)


def _cop_fraction(
    cop_ground: np.ndarray,
    heel_ground: np.ndarray,
    toe_ground: np.ndarray,
    normal: np.ndarray,
) -> np.ndarray:
    n = np.asarray(normal, dtype=float)
    n /= np.linalg.norm(n)
    cop = np.asarray(cop_ground, dtype=float)
    heel = np.asarray(heel_ground, dtype=float)
    toe = np.asarray(toe_ground, dtype=float)
    segment = toe - heel
    segment -= (segment @ n)[:, None] * n
    relative = cop - heel
    relative -= (relative @ n)[:, None] * n
    denominator = np.sum(segment * segment, axis=1)
    result = np.full(len(cop), np.nan, dtype=float)
    valid = denominator > 1e-12
    result[valid] = np.sum(relative[valid] * segment[valid], axis=1) / denominator[
        valid
    ]
    return result


def _first_above_time(
    times: np.ndarray,
    values: np.ndarray,
    *,
    reference_time_s: float,
    threshold: float,
    before_s: float = 0.100,
    after_s: float = 0.200,
) -> float | None:
    mask = (
        (times >= reference_time_s - before_s - 1e-12)
        & (times <= reference_time_s + after_s + 1e-12)
        & (values >= threshold)
    )
    indices = np.flatnonzero(mask)
    return float(times[indices[0]]) if indices.size else None


def _first_pulse_duration(
    times: np.ndarray,
    values: np.ndarray,
    *,
    start_time_s: float | None,
    threshold: float,
) -> float | None:
    if start_time_s is None:
        return None
    start = int(np.searchsorted(times, start_time_s - 1e-12, side="left"))
    if start >= len(times) or values[start] < threshold:
        return None
    below = np.flatnonzero(values[start:] < threshold)
    end = start + int(below[0]) if below.size else len(times) - 1
    return max(0.0, float(times[end] - times[start]))


def _threshold_run_near_reference(
    times: np.ndarray,
    values: np.ndarray,
    *,
    reference_time_s: float,
    threshold: float,
    after_s: float = 0.200,
    runtime_dt_s: float = 0.010,
    dwell_s: float = 0.030,
) -> dict[str, Any]:
    """Select the threshold run containing reference contact, else the next run."""
    time = np.asarray(times, dtype=float)
    signal = np.asarray(values, dtype=float)
    above = signal >= float(threshold)
    starts = np.flatnonzero(above & ~np.r_[False, above[:-1]])
    ends = np.flatnonzero(above & ~np.r_[above[1:], False]) + 1
    reference_index = int(np.argmin(np.abs(time - float(reference_time_s))))
    selected: tuple[int, int] | None = None
    for start, end in zip(starts, ends):
        if start <= reference_index < end:
            selected = (int(start), int(end))
            break
    if selected is None:
        for start, end in zip(starts, ends):
            if (
                time[start] >= reference_time_s - 1e-12
                and time[start] <= reference_time_s + after_s + 1e-12
            ):
                selected = (int(start), int(end))
                break
    if selected is None:
        return {
            "onset_time_s": None,
            "end_time_s": None,
            "duration_s": None,
            "ideal_dwell_confirmable": False,
            "runtime_10ms_confirmable": False,
        }
    start, end = selected
    dt = float(np.median(np.diff(time)))
    end_time = float(time[end]) if end < len(time) else float(time[-1] + dt)
    duration = end_time - float(time[start])
    stride_float = float(runtime_dt_s) / dt
    stride = int(round(stride_float))
    if stride <= 0 or abs(stride_float - stride) > 1e-8:
        raise ValueError("runtime_dt_s must be an integer multiple of audit dt")
    runtime_indices = np.arange(0, len(time), stride, dtype=int)
    within = runtime_indices[
        (runtime_indices >= start)
        & (runtime_indices < end)
        & above[runtime_indices]
    ]
    runtime_confirmable = bool(
        within.size >= 2
        and float(time[within[-1]] - time[within[0]]) >= dwell_s - 1e-12
    )
    return {
        "onset_time_s": float(time[start]),
        "end_time_s": end_time,
        "duration_s": duration,
        "ideal_dwell_confirmable": bool(duration >= dwell_s - 1e-12),
        "runtime_10ms_confirmable": runtime_confirmable,
    }


def _contact_order(heel_time: float | None, toe_time: float | None, dt: float) -> str:
    if heel_time is None and toe_time is None:
        return "none"
    if heel_time is None:
        return "toe_only"
    if toe_time is None:
        return "heel_only"
    if abs(heel_time - toe_time) <= dt + 1e-12:
        return "simultaneous"
    return "heel_first" if heel_time < toe_time else "toe_first"


def _event_rows(
    *,
    times: np.ndarray,
    reference_hs: np.ndarray,
    critical: Sequence[Mapping[str, Any]],
    prescribed_force: np.ndarray,
    prescribed_cop: np.ndarray,
    loads: Mapping[str, np.ndarray],
    penetrations: Mapping[str, np.ndarray],
    clearances: Mapping[str, np.ndarray],
    samples: Mapping[str, Any],
    sensors: Mapping[str, OnlineGRFSphere],
    foot_origins: np.ndarray,
    foot_rotations: np.ndarray,
    mesh_vertices_local: np.ndarray,
    ground_origin: np.ndarray,
    ground_normal: np.ndarray,
    threshold_n: float,
) -> list[dict[str, Any]]:
    dt = float(np.median(np.diff(times)))
    critical_by_index = {int(item["reference_index"]): dict(item) for item in critical}
    heel_centers = np.asarray(samples["centers"][sensors["left_heel"].name])
    toe_centers = np.asarray(samples["centers"][sensors["left_toe"].name])
    heel_velocities = np.asarray(
        samples["velocities"][sensors["left_heel"].name], dtype=float
    )
    toe_velocities = np.asarray(
        samples["velocities"][sensors["left_toe"].name], dtype=float
    )
    cop_fraction = _cop_fraction(
        prescribed_cop, heel_centers, toe_centers, ground_normal
    )
    local_cop = _points_in_local_frame(
        prescribed_cop, foot_origins, foot_rotations
    )
    normal = np.asarray(ground_normal, dtype=float)
    normal /= np.linalg.norm(normal)
    foot_x = foot_rotations[:, :, 0]
    foot_pitch = np.degrees(
        np.arctan2(
            foot_x @ normal,
            np.linalg.norm(foot_x - (foot_x @ normal)[:, None] * normal, axis=1),
        )
    )
    rows: list[dict[str, Any]] = []
    for event_index, reference_time in enumerate(reference_hs):
        index = int(np.argmin(np.abs(times - reference_time)))
        early = (
            (times >= reference_time + 0.010 - 0.5 * dt)
            & (times <= reference_time + 0.050 + 0.5 * dt)
            & (prescribed_force[:, 1] >= 20.0)
            & np.all(np.isfinite(prescribed_cop), axis=1)
        )
        if not np.any(early):
            early = (
                (times >= reference_time - 0.5 * dt)
                & (times <= reference_time + 0.050 + 0.5 * dt)
                & (prescribed_force[:, 1] >= 20.0)
                & np.all(np.isfinite(prescribed_cop), axis=1)
            )
        local_cop_median = (
            np.median(local_cop[early], axis=0)
            if np.any(early)
            else np.full(3, np.nan)
        )
        cop_fraction_median = (
            float(np.nanmedian(cop_fraction[early])) if np.any(early) else math.nan
        )
        heel_run = _threshold_run_near_reference(
            times,
            loads["left_heel"],
            reference_time_s=float(reference_time),
            threshold=threshold_n,
        )
        toe_run = _threshold_run_near_reference(
            times,
            loads["left_toe"],
            reference_time_s=float(reference_time),
            threshold=threshold_n,
        )
        heel_start = heel_run["onset_time_s"]
        toe_start = toe_run["onset_time_s"]
        heel_pulse = heel_run["duration_s"]
        toe_pulse = toe_run["duration_s"]
        heel_local = np.asarray(sensors["left_heel"].location, dtype=float)
        toe_local = np.asarray(sensors["left_toe"].location, dtype=float)
        reference_cop_local = local_cop[index]
        reference_heel_distance = float(
            np.linalg.norm(reference_cop_local[[0, 2]] - heel_local[[0, 2]])
        )
        reference_toe_distance = float(
            np.linalg.norm(reference_cop_local[[0, 2]] - toe_local[[0, 2]])
        )
        heel_cop_distance = float(
            np.linalg.norm(local_cop_median[[0, 2]] - heel_local[[0, 2]])
        )
        toe_cop_distance = float(
            np.linalg.norm(local_cop_median[[0, 2]] - toe_local[[0, 2]])
        )
        total_virtual = float(
            loads["left_heel"][index] + loads["left_toe"][index]
        )
        normal_in_foot = foot_rotations[index].T @ normal
        mesh_distances = (
            float((foot_origins[index] - ground_origin) @ normal)
            + np.asarray(mesh_vertices_local, dtype=float) @ normal_in_foot
        )
        lowest_mesh_index = int(np.argmin(mesh_distances))
        lowest_mesh_local = np.asarray(mesh_vertices_local)[lowest_mesh_index]
        critical_item = critical_by_index.get(event_index)
        heel_offset_ms = (
            1000.0 * (heel_start - reference_time)
            if heel_start is not None
            else None
        )
        if heel_offset_ms is None or heel_offset_ms > 50.0:
            cause = "GEOMETRIC_HEEL_DELAY"
        elif heel_pulse is None or heel_pulse < 0.030 - 0.5 * dt:
            cause = "SHORT_HEEL_PULSE"
        elif not bool(heel_run["runtime_10ms_confirmable"]):
            cause = "DEBOUNCE_SAMPLING_10MS"
        else:
            cause = "FIRST_HEEL_RUN_CONFIRMABLE"
        rows.append(
            {
                "reference_index": event_index,
                "reference_time_s": float(reference_time),
                "is_v3_root_critical": critical_item is not None,
                "v3_critical_kind": (
                    str(critical_item["kind"]) if critical_item else ""
                ),
                "v3_primary_error_ms": (
                    1000.0 * float(critical_item["primary_error_s"])
                    if critical_item and critical_item["primary_error_s"] is not None
                    else None
                ),
                "prescribed_fy_at_reference_n": float(prescribed_force[index, 1]),
                "virtual_heel_load_at_reference_n": float(
                    loads["left_heel"][index]
                ),
                "virtual_toe_load_at_reference_n": float(loads["left_toe"][index]),
                "virtual_toe_load_share_at_reference": (
                    float(loads["left_toe"][index] / total_virtual)
                    if total_virtual > 1e-12
                    else None
                ),
                "heel_penetration_at_reference_mm": 1000.0
                * float(penetrations["left_heel"][index]),
                "toe_penetration_at_reference_mm": 1000.0
                * float(penetrations["left_toe"][index]),
                "heel_bottom_clearance_at_reference_mm": 1000.0
                * float(clearances["left_heel"][index]),
                "toe_bottom_clearance_at_reference_mm": 1000.0
                * float(clearances["left_toe"][index]),
                "heel_normal_velocity_at_reference_m_s": float(
                    heel_velocities[index] @ normal
                ),
                "toe_normal_velocity_at_reference_m_s": float(
                    toe_velocities[index] @ normal
                ),
                "mesh_min_clearance_at_reference_mm": 1000.0
                * float(mesh_distances[lowest_mesh_index]),
                "mesh_lowest_local_x_m": float(lowest_mesh_local[0]),
                "mesh_lowest_local_y_m": float(lowest_mesh_local[1]),
                "mesh_lowest_local_z_m": float(lowest_mesh_local[2]),
                "heel_first_above_0p5_offset_ms": (
                    heel_offset_ms
                ),
                "toe_first_above_0p5_offset_ms": (
                    1000.0 * (toe_start - reference_time)
                    if toe_start is not None
                    else None
                ),
                "heel_first_0p5_pulse_duration_ms": (
                    1000.0 * heel_pulse if heel_pulse is not None else None
                ),
                "toe_first_0p5_pulse_duration_ms": (
                    1000.0 * toe_pulse if toe_pulse is not None else None
                ),
                "heel_first_run_ideal_30ms_confirmable": bool(
                    heel_run["ideal_dwell_confirmable"]
                ),
                "heel_first_run_runtime_10ms_confirmable": bool(
                    heel_run["runtime_10ms_confirmable"]
                ),
                "root_cause_classification": cause,
                "virtual_contact_order_0p5n": _contact_order(
                    heel_start, toe_start, dt
                ),
                "reference_cop_fraction_heel0_toe1": float(cop_fraction[index]),
                "reference_cop_local_x_m": float(reference_cop_local[0]),
                "reference_cop_local_y_m": float(reference_cop_local[1]),
                "reference_cop_local_z_m": float(reference_cop_local[2]),
                "reference_cop_distance_to_heel_xz_mm": 1000.0
                * reference_heel_distance,
                "reference_cop_distance_to_toe_xz_mm": 1000.0
                * reference_toe_distance,
                "reference_cop_closer_sensor": (
                    "heel"
                    if reference_heel_distance <= reference_toe_distance
                    else "toe"
                ),
                "early_cop_fraction_heel0_toe1": cop_fraction_median,
                "early_cop_local_x_m": float(local_cop_median[0]),
                "early_cop_local_y_m": float(local_cop_median[1]),
                "early_cop_local_z_m": float(local_cop_median[2]),
                "early_cop_distance_to_heel_xz_mm": 1000.0 * heel_cop_distance,
                "early_cop_distance_to_toe_xz_mm": 1000.0 * toe_cop_distance,
                "early_cop_closer_sensor": (
                    "heel" if heel_cop_distance <= toe_cop_distance else "toe"
                ),
                "foot_local_x_pitch_at_reference_deg": float(foot_pitch[index]),
            }
        )
    return rows


def _summarise_events(rows: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    critical = [row for row in rows if bool(row["is_v3_root_critical"])]

    def counts(items: Sequence[Mapping[str, Any]], key: str) -> dict[str, int]:
        result: dict[str, int] = {}
        for item in items:
            label = str(item[key])
            result[label] = result.get(label, 0) + 1
        return dict(sorted(result.items()))

    critical_reference_cop_heel = sum(
        row["reference_cop_closer_sensor"] == "heel" for row in critical
    )
    return {
        "reference_hs_count": len(rows),
        "v3_root_critical_hs_count": len(critical),
        "virtual_contact_order_counts_all_hs": counts(
            rows, "virtual_contact_order_0p5n"
        ),
        "virtual_contact_order_counts_critical_hs": counts(
            critical, "virtual_contact_order_0p5n"
        ),
        "root_cause_counts_critical_hs": counts(
            critical, "root_cause_classification"
        ),
        "reference_cop_closer_sensor_counts_all_hs": counts(
            rows, "reference_cop_closer_sensor"
        ),
        "reference_cop_closer_sensor_counts_critical_hs": counts(
            critical, "reference_cop_closer_sensor"
        ),
        "early_cop_closer_sensor_counts_all_hs": counts(
            rows, "early_cop_closer_sensor"
        ),
        "early_cop_closer_sensor_counts_critical_hs": counts(
            critical, "early_cop_closer_sensor"
        ),
        "critical_reference_cop_closer_to_heel_fraction": (
            float(critical_reference_cop_heel / len(critical)) if critical else None
        ),
        "critical_rows": critical,
    }


def _write_csv(path: Path, rows: Sequence[Mapping[str, Any]]) -> None:
    if not rows:
        raise ValueError("cannot write an empty event audit")
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = list(rows[0].keys())
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def _plot_static_geometry(
    path: Path,
    triangles: np.ndarray,
    sensors: Mapping[str, OnlineGRFSphere],
    static_geometry: Mapping[str, Any],
    rows: Sequence[Mapping[str, Any]],
    *,
    profile_label: str,
) -> None:
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle

    vertices = np.unique(np.asarray(triangles).reshape(-1, 3), axis=0)
    figure, axes = plt.subplots(1, 2, figsize=(14, 6))
    axes[0].scatter(
        1000.0 * vertices[:, 0],
        1000.0 * vertices[:, 1],
        s=1,
        alpha=0.25,
        color="0.35",
        label="foot_l mesh",
    )
    axes[1].scatter(
        1000.0 * vertices[:, 0],
        1000.0 * vertices[:, 2],
        s=1,
        alpha=0.25,
        color="0.35",
        label="foot_l mesh",
    )
    colors = {"left_heel": "tab:blue", "left_toe": "tab:orange"}
    for role, sphere in sensors.items():
        center = 1000.0 * np.asarray(sphere.location, dtype=float)
        radius = 1000.0 * float(sphere.radius)
        label = role.replace("left_", "")
        axes[0].add_patch(
            Circle(
                (center[0], center[1]),
                radius,
                fill=False,
                linewidth=2,
                color=colors[role],
                label=f"{label} sensor sphere",
            )
        )
        axes[0].plot(center[0], center[1], "o", color=colors[role])
        axes[1].add_patch(
            Circle(
                (center[0], center[2]),
                radius,
                fill=False,
                linewidth=2,
                color=colors[role],
                label=f"{label} sensor sphere",
            )
        )
        axes[1].plot(center[0], center[2], "o", color=colors[role])
    noncritical = [row for row in rows if not row["is_v3_root_critical"]]
    critical = [row for row in rows if row["is_v3_root_critical"]]
    for items, color, marker, label in (
        (noncritical, "tab:green", ".", "prescribed COP at reference contact"),
        (critical, "tab:red", "x", "critical-HS COP at reference contact"),
    ):
        axes[1].scatter(
            [1000.0 * float(row["reference_cop_local_x_m"]) for row in items],
            [1000.0 * float(row["reference_cop_local_z_m"]) for row in items],
            color=color,
            marker=marker,
            s=45 if marker == "x" else 20,
            label=label,
            zorder=5,
        )
    axes[0].set_title("Sagittal local geometry (x-y)")
    axes[0].set_xlabel("foot_l local x [mm]")
    axes[0].set_ylabel("foot_l local y [mm]")
    axes[1].set_title("Plantar projection and force-plate COP at contact (x-z)")
    axes[1].set_xlabel("foot_l local x [mm]")
    axes[1].set_ylabel("foot_l local z [mm]")
    for axis in axes:
        axis.set_aspect("equal", adjustable="box")
        axis.grid(True, alpha=0.25)
        axis.legend(loc="best", fontsize=8)
    toe = static_geometry["sensors"]["left_toe"]
    heel = static_geometry["sensors"]["left_heel"]
    toe_relation = (
        "mesh-adjacent"
        if toe["sphere_surface_gap_to_mesh_m"] <= 0.005
        else "not mesh-adjacent"
    )
    figure.suptitle(
        f"V4 design audit — {profile_label}; toe sensor is {toe_relation}\n"
        f"mesh-surface gap: heel {1000*heel['sphere_surface_gap_to_mesh_m']:.1f} mm, "
        f"toe {1000*toe['sphere_surface_gap_to_mesh_m']:.1f} mm; "
        f"toe-minus-heel sphere bottom "
        f"{1000*static_geometry['heel_toe_bottom_offset_m']:.1f} mm"
    )
    figure.tight_layout(rect=(0.0, 0.0, 1.0, 0.92))
    path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(path, dpi=180)
    plt.close(figure)


def _plot_all_hs(
    path: Path,
    rows: Sequence[Mapping[str, Any]],
    *,
    profile_label: str,
) -> None:
    import matplotlib.pyplot as plt

    times = np.asarray([row["reference_time_s"] for row in rows], dtype=float)
    critical = np.asarray([row["is_v3_root_critical"] for row in rows], dtype=bool)
    reference_cop_fraction = np.asarray(
        [row["reference_cop_fraction_heel0_toe1"] for row in rows], dtype=float
    )
    early_cop_fraction = np.asarray(
        [row["early_cop_fraction_heel0_toe1"] for row in rows], dtype=float
    )
    heel_clearance = np.asarray(
        [row["heel_bottom_clearance_at_reference_mm"] for row in rows], dtype=float
    )
    toe_clearance = np.asarray(
        [row["toe_bottom_clearance_at_reference_mm"] for row in rows], dtype=float
    )
    toe_share = np.asarray(
        [row["virtual_toe_load_share_at_reference"] for row in rows], dtype=float
    )
    figure, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
    axes[0].plot(
        times,
        reference_cop_fraction,
        "o-",
        linewidth=1,
        markersize=3,
        label="COP at prescribed contact",
    )
    axes[0].plot(
        times,
        early_cop_fraction,
        "--",
        linewidth=1,
        color="0.55",
        label="COP median +10…+50 ms",
    )
    axes[0].axhline(0.0, color="tab:blue", linestyle=":", label="heel sensor")
    axes[0].axhline(1.0, color="tab:orange", linestyle=":", label="toe sensor")
    axes[0].set_ylabel("COP fraction\nheel=0, toe=1")
    axes[0].legend(loc="best")
    axes[1].plot(times, heel_clearance, "o-", label="heel", markersize=3)
    axes[1].plot(times, toe_clearance, "o-", label="toe", markersize=3)
    axes[1].axhline(0.0, color="black", linewidth=0.8)
    axes[1].set_ylabel("sphere-bottom clearance\n[mm; negative=penetration]")
    axes[1].legend(loc="best")
    axes[2].plot(times, toe_share, "o-", color="tab:purple", markersize=3)
    axes[2].axhline(0.5, color="black", linestyle=":")
    axes[2].set_ylabel("virtual toe load share\nat prescribed HS")
    axes[2].set_xlabel("prescribed HS time [s]")
    for axis in axes:
        highlight_values = (
            reference_cop_fraction
            if axis is axes[0]
            else np.asarray(axis.lines[0].get_ydata())
        )
        axis.scatter(
            times[critical],
            highlight_values[critical],
            marker="x",
            s=70,
            linewidth=2,
            color="tab:red",
            label="V3 root-critical HS" if axis is axes[0] else None,
            zorder=6,
        )
        axis.grid(True, alpha=0.25)
    axes[0].legend(loc="best")
    figure.suptitle(
        "Already-open V3 block 50–100 s — prescribed COP vs virtual contacts\n"
        f"profile: {profile_label}"
    )
    figure.tight_layout(rect=(0.0, 0.0, 1.0, 0.96))
    path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(path, dpi=180)
    plt.close(figure)


def _plot_critical_windows(
    path: Path,
    *,
    rows: Sequence[Mapping[str, Any]],
    times: np.ndarray,
    prescribed_force: np.ndarray,
    loads: Mapping[str, np.ndarray],
    clearances: Mapping[str, np.ndarray],
    profile_label: str,
) -> None:
    import matplotlib.pyplot as plt

    critical = [row for row in rows if row["is_v3_root_critical"]]
    figure, axes = plt.subplots(len(critical), 2, figsize=(15, 3.2 * len(critical)))
    if len(critical) == 1:
        axes = np.asarray([axes])
    for row_index, row in enumerate(critical):
        reference_time = float(row["reference_time_s"])
        mask = (times >= reference_time - 0.100) & (times <= reference_time + 0.200)
        relative_ms = 1000.0 * (times[mask] - reference_time)
        force_axis = axes[row_index, 0]
        force_axis.plot(
            relative_ms,
            prescribed_force[mask, 1],
            color="0.25",
            linewidth=1.5,
            label="prescribed Fy",
        )
        sensor_axis = force_axis.twinx()
        sensor_axis.plot(
            relative_ms,
            loads["left_heel"][mask],
            color="tab:blue",
            label="heel load",
        )
        sensor_axis.plot(
            relative_ms,
            loads["left_toe"][mask],
            color="tab:orange",
            label="toe load",
        )
        sensor_axis.axhline(0.5, color="tab:red", linestyle=":", linewidth=1)
        sensor_axis.set_yscale("symlog", linthresh=0.5)
        force_axis.axvline(0.0, color="black", linestyle="--", linewidth=1)
        force_axis.set_ylabel("prescribed Fy [N]")
        sensor_axis.set_ylabel("virtual local load [N]")
        lines = [
            line
            for line in force_axis.lines + sensor_axis.lines[:2]
            if not line.get_label().startswith("_")
        ]
        force_axis.legend(lines, [line.get_label() for line in lines], fontsize=8)
        clearance_axis = axes[row_index, 1]
        clearance_axis.plot(
            relative_ms,
            1000.0 * clearances["left_heel"][mask],
            color="tab:blue",
            label="heel bottom clearance",
        )
        clearance_axis.plot(
            relative_ms,
            1000.0 * clearances["left_toe"][mask],
            color="tab:orange",
            label="toe bottom clearance",
        )
        clearance_axis.axhline(0.0, color="black", linewidth=0.8)
        clearance_axis.axvline(0.0, color="black", linestyle="--", linewidth=1)
        clearance_axis.set_ylabel("clearance [mm; negative=penetration]")
        clearance_axis.legend(fontsize=8)
        for axis in (force_axis, clearance_axis):
            axis.grid(True, alpha=0.25)
            axis.set_xlabel("time from prescribed initial contact [ms]")
        force_axis.set_title(
            f"HS {reference_time:.6f} s — {row['v3_critical_kind']}"
        )
        clearance_axis.set_title(
            f"order={row['virtual_contact_order_0p5n']}, "
            f"COP@contact→{row['reference_cop_closer_sensor']}, "
            f"cause={row['root_cause_classification']}"
        )
    figure.suptitle(
        "Five independent V3 root-critical contacts — diagnostic only\n"
        f"profile: {profile_label}"
    )
    figure.tight_layout(rect=(0.0, 0.0, 1.0, 0.98))
    path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(path, dpi=180)
    plt.close(figure)


def run_audit(args: argparse.Namespace) -> dict[str, Any]:
    start_s = float(args.t_start)
    end_s = float(args.t_end)
    sample_dt = float(args.sample_dt)
    _validate_unsealed_window(start_s, end_s)
    if not math.isfinite(sample_dt) or sample_dt <= 0.0 or sample_dt > 0.001:
        raise ValueError("geometric audit sample_dt must be in (0, 0.001] s")

    setup_path = resolve_repo_path(args.setup).resolve()
    profile_path = resolve_repo_path(args.profile).resolve()
    v3_manifest_path = resolve_repo_path(args.v3_manifest).resolve()
    output_dir = resolve_repo_path(args.output_dir).resolve()
    plot_dir = resolve_repo_path(args.plot_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    plot_dir.mkdir(parents=True, exist_ok=True)

    setup = replace(read_setup_xml(setup_path), t_start=start_s, t_end=end_s)
    profile = load_online_grf_profile(profile_path)
    sensors = _left_sensor_spheres(profile)
    v3_manifest = json.loads(v3_manifest_path.read_text(encoding="utf-8"))
    if [float(item) for item in v3_manifest["block_time_range_s"]] != [50.0, 100.0]:
        raise ValueError("V3 lineage must be the already-open 50--100 s manifest")
    critical = _derive_critical_hs(v3_manifest)
    if len(critical) != 5:
        raise ValueError(f"expected five independent V3 root-critical HS, got {critical}")

    reference_events, reference_provenance = _reference_events_from_prescribed_grf(
        setup,
        threshold_n=20.0,
        min_contact_duration_s=0.05,
        min_cycle_duration_s=0.3,
    )
    manifest_reference = np.asarray(
        v3_manifest["primary_candidate_details"][V3_SELECTED_CANDIDATE]["events"]
        ["reference"]["heel_strike"],
        dtype=float,
    )
    if not np.allclose(
        reference_events["heel_strike"], manifest_reference, rtol=0.0, atol=1e-10
    ):
        raise ValueError("reconstructed prescribed HS do not match V3 lineage")

    times = np.arange(start_s, end_s + 0.25 * sample_dt, sample_dt, dtype=float)
    if times[-1] > SEALED_START_S + 1e-12:
        raise ValueError("sample grid crossed into the sealed block")
    sampling_profile, station_names = _diagnostic_sampling_profile(profile)
    samples = _sample_spheres(setup, sampling_profile, times, args.sea_plugin)
    original_names = {sphere.name for sphere in profile.spheres}
    original_samples = {
        key: {name: values for name, values in bucket.items() if name in original_names}
        for key, bucket in samples.items()
    }
    loads, penetrations, _aggregate = _regional_loads_and_penetrations(
        profile, original_samples
    )
    ground_origin = np.asarray(profile.ground.origin, dtype=float)
    ground_normal = np.asarray(profile.ground.normal, dtype=float)
    ground_normal /= np.linalg.norm(ground_normal)
    clearances = {
        role: (
            np.asarray(original_samples["centers"][sphere.name], dtype=float)
            - ground_origin
        )
        @ ground_normal
        - float(sphere.radius)
        for role, sphere in sensors.items()
    }
    foot_origins, foot_rotations = _foot_frame_kinematics(samples, station_names)
    prescribed = _external_wrench(setup, times)["left"]
    prescribed_force = np.asarray(prescribed["force"], dtype=float)
    prescribed_cop = np.asarray(prescribed["cop"], dtype=float)

    mesh_path = _resolve_left_foot_mesh(setup.model_file)
    triangles = _load_stl_triangles(mesh_path)
    static_geometry = _static_sensor_geometry(triangles, sensors)
    static_geometry["exact_model_marker_matches"] = _matching_model_markers(
        setup.model_file, sensors
    )
    rows = _event_rows(
        times=times,
        reference_hs=np.asarray(reference_events["heel_strike"], dtype=float),
        critical=critical,
        prescribed_force=prescribed_force,
        prescribed_cop=prescribed_cop,
        loads=loads,
        penetrations=penetrations,
        clearances=clearances,
        samples=original_samples,
        sensors=sensors,
        foot_origins=foot_origins,
        foot_rotations=foot_rotations,
        mesh_vertices_local=np.unique(triangles.reshape(-1, 3), axis=0),
        ground_origin=ground_origin,
        ground_normal=ground_normal,
        threshold_n=0.5,
    )
    event_summary = _summarise_events(rows)

    csv_path = output_dir / "event_geometry_metrics.csv"
    plot_static = plot_dir / "01_static_sensor_mesh_geometry.png"
    plot_all = plot_dir / "02_all_hs_initial_contact_geometry.png"
    plot_critical = plot_dir / "03_critical_hs_contact_windows.png"
    _write_csv(csv_path, rows)
    profile_label = str(profile.source)
    _plot_static_geometry(
        plot_static,
        triangles,
        sensors,
        static_geometry,
        rows,
        profile_label=profile_label,
    )
    _plot_all_hs(plot_all, rows, profile_label=profile_label)
    _plot_critical_windows(
        plot_critical,
        rows=rows,
        times=times,
        prescribed_force=prescribed_force,
        loads=loads,
        clearances=clearances,
        profile_label=profile_label,
    )

    geometry_ok = bool(static_geometry["geometry_plausible_for_semantic_decision"])
    critical_reference_cop_heel_fraction = float(
        event_summary["critical_reference_cop_closer_to_heel_fraction"]
    )
    conclusion = {
        "reference_event_semantics": (
            "The V3 reference timestamp is total-foot initial contact: the "
            "accepted rising edge of prescribed left vertical GRF at 20 N. "
            "It is not an independent anatomical heel-sensor measurement."
        ),
        "geometry_result": (
            "CURRENT_GEOMETRY_NOT_PLAUSIBLE"
            if not geometry_ok
            else "CURRENT_GEOMETRY_PASSES_DIAGNOSTIC_CHECKS"
        ),
        "critical_contact_evidence": (
            f"At {critical_reference_cop_heel_fraction:.0%} of the five "
            "independent V3 root-critical contacts, the prescribed COP at "
            "the reference timestamp is closer to the selected-profile heel "
            "station than to its toe station."
        ),
        "decision": (
            "Correct and re-audit the two virtual sensor placements before "
            "changing the FSM event from heel_strike to either-sensor "
            "initial_contact. Current forefoot-first evidence is confounded "
            "by the detached/vertically offset toe sphere."
            if not geometry_ok
            else "Geometry alone does not block a subsequent event-semantics decision."
        ),
        "runtime_change_authorized_by_this_audit": False,
        "policy_training_needed_for_this_audit": False,
    }
    report = {
        "schema_version": 1,
        "audit_id": "AB06_TWO_SENSOR_GEOMETRY_V4_DESIGN_2026-07-21",
        "status": "DIAGNOSTIC_COMPLETE",
        "purpose": "V4 design evidence only; not validation or detector promotion",
        "time_block_s": [start_s, end_s],
        "sample_dt_s": sample_dt,
        "sealed_block_s": [100.0, 155.045],
        "sealed_block_opened": False,
        "policy_or_training_used": False,
        "runtime_fsm_modified": False,
        "runtime_detector_profile_modified": False,
        "sources": {
            "setup": _source_record(setup_path),
            "model": _source_record(setup.model_file),
            "foot_mesh": _source_record(mesh_path),
            "detector_profile": _source_record(profile_path),
            "v3_validation_manifest": _source_record(v3_manifest_path),
        },
        "v3_lineage": {
            "selected_candidate_id": V3_SELECTED_CANDIDATE,
            "independent_root_critical_hs": critical,
        },
        "reference_provenance": reference_provenance,
        "static_sensor_geometry": static_geometry,
        "event_summary": event_summary,
        "conclusion": conclusion,
        "artifacts": {
            "event_csv": _portable_path(csv_path),
            "static_geometry_plot": _portable_path(plot_static),
            "all_hs_plot": _portable_path(plot_all),
            "critical_windows_plot": _portable_path(plot_critical),
        },
    }
    manifest_path = output_dir / "manifest.json"
    manifest_path.write_text(
        json.dumps(_json_safe(report), indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return report


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--setup", default=DEFAULT_SETUP)
    parser.add_argument("--profile", default=DEFAULT_PROFILE)
    parser.add_argument("--sea-plugin", default=DEFAULT_SEA_PLUGIN)
    parser.add_argument("--v3-manifest", default=DEFAULT_V3_MANIFEST)
    parser.add_argument("--output-dir", default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--plot-dir", default=DEFAULT_PLOT_DIR)
    parser.add_argument("--t-start", type=float, default=UNSEALED_START_S)
    parser.add_argument("--t-end", type=float, default=UNSEALED_END_S)
    parser.add_argument("--sample-dt", type=float, default=0.001)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    try:
        report = run_audit(args)
    except Exception as exc:  # pragma: no cover - CLI fail-closed path
        print(f"FAIL: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 1
    print(json.dumps(_json_safe(report["conclusion"]), indent=2, sort_keys=True))
    print(f"manifest: {_portable_path(resolve_repo_path(args.output_dir) / 'manifest.json')}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
