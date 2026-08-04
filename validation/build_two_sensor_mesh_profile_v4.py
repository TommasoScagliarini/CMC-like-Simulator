"""Build the experimental mesh-based V4 heel/toe detector profile.

The builder is deliberately geometry-only: it never reads prescribed gait
events, force-plate signals, rollout traces, or policy outputs.  The current
heel sphere is retained as the trusted anchor and only ``left_toe.location``
is derived from the ``foot_l`` mesh.  All contact-law and runtime fields stay
identical to the frozen legacy detector profile.

The generated profile is experimental and opt-in.  This script does not edit
the training configuration, FSM, or legacy profile.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
from dataclasses import replace
from pathlib import Path
from typing import Any, Sequence

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
TRAJECTORY_ROOT = REPO_ROOT / "Trajectory Generator"
for path in (REPO_ROOT, TRAJECTORY_ROOT):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from online_grf import (  # noqa: E402
    OnlineGRFProfile,
    OnlineGRFSphere,
    load_online_grf_profile,
    write_online_grf_profile,
)
from path_resolver import resolve_repo_path  # noqa: E402
from setup_io import read_setup_xml  # noqa: E402
from validation.audit_two_sensor_prescribed_geometry import (  # noqa: E402
    _load_stl_triangles,
    _minimum_mesh_distance,
    _resolve_left_foot_mesh,
)
from validation.validate_two_sensor_prescribed_replay import (  # noqa: E402
    _left_sensor_spheres,
)


DEFAULT_SETUP = (
    "models/AB06_SEASEA_Threadmill/"
    "AB06_SEASEA_stiff321_500_pi_setup.xml"
)
DEFAULT_BASE_PROFILE = (
    "online_grf_profiles/"
    "AB06_SEASEA_stiff321_500_pi_grf_detector_HS-TO.json"
)
DEFAULT_OUTPUT = (
    "online_grf_profiles/"
    "AB06_SEASEA_stiff321_500_pi_grf_detector_"
    "HS-TO_v4_mesh_experimental.json"
)
DESIGN_ID = "heel_anchor_anterior_flush_equal_plantar_protrusion_v1"


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


def _deduplicate_scalars(values: Sequence[float], tolerance: float = 1e-10) -> list[float]:
    ordered = sorted(float(value) for value in values)
    unique: list[float] = []
    for value in ordered:
        if not unique or abs(value - unique[-1]) > tolerance:
            unique.append(value)
    return unique


def _section_z_bounds_at_x(
    triangles: np.ndarray,
    x_value: float,
    *,
    tolerance: float = 1e-12,
) -> tuple[float, float]:
    """Return the lateral extent of the mesh/constant-x intersection."""
    intersections: list[float] = []
    for triangle in np.asarray(triangles, dtype=float):
        for start_index, end_index in ((0, 1), (1, 2), (2, 0)):
            start = triangle[start_index]
            end = triangle[end_index]
            start_delta = float(start[0] - x_value)
            end_delta = float(end[0] - x_value)
            if abs(start_delta) <= tolerance and abs(end_delta) <= tolerance:
                intersections.extend((float(start[2]), float(end[2])))
                continue
            if start_delta * end_delta > 0.0:
                continue
            denominator = float(end[0] - start[0])
            if abs(denominator) <= tolerance:
                continue
            fraction = float((x_value - start[0]) / denominator)
            if -tolerance <= fraction <= 1.0 + tolerance:
                point = start + fraction * (end - start)
                intersections.append(float(point[2]))
    if len(intersections) < 2:
        raise ValueError(f"mesh does not have a valid section at x={x_value:.12g}")
    return min(intersections), max(intersections)


def _vertical_surface_intersections_y(
    triangles: np.ndarray,
    x_value: float,
    z_value: float,
    *,
    tolerance: float = 1e-10,
) -> list[float]:
    """Intersect a body-frame vertical line with a triangular surface."""
    target = np.asarray([x_value, z_value], dtype=float)
    intersections: list[float] = []
    for triangle in np.asarray(triangles, dtype=float):
        projected = triangle[:, (0, 2)]
        matrix = np.column_stack(
            (projected[1] - projected[0], projected[2] - projected[0])
        )
        determinant = float(np.linalg.det(matrix))
        if abs(determinant) <= 1e-15:
            continue
        weights_12 = np.linalg.solve(matrix, target - projected[0])
        weights = np.asarray(
            [1.0 - weights_12.sum(), weights_12[0], weights_12[1]],
            dtype=float,
        )
        if np.all(weights >= -tolerance) and np.all(weights <= 1.0 + tolerance):
            intersections.append(float(weights @ triangle[:, 1]))
    unique = _deduplicate_scalars(intersections)
    if len(unique) < 2:
        raise ValueError(
            "vertical mesh line must intersect plantar and dorsal surfaces: "
            f"x={x_value:.12g}, z={z_value:.12g}, intersections={unique}"
        )
    return unique


def derive_left_toe_location(
    triangles: np.ndarray,
    heel: OnlineGRFSphere,
    toe: OnlineGRFSphere,
) -> tuple[tuple[float, float, float], dict[str, Any]]:
    """Derive one anterior sphere from mesh geometry and the heel anchor."""
    if heel.frame != toe.frame or heel.side != "left" or toe.side != "left":
        raise ValueError("heel and toe must share the left foot frame")
    if not math.isclose(heel.radius, toe.radius, rel_tol=0.0, abs_tol=1e-12):
        raise ValueError("V4 requires the frozen heel and toe radii to match")

    vertices = np.asarray(triangles, dtype=float).reshape(-1, 3)
    bounds_min = np.min(vertices, axis=0)
    bounds_max = np.max(vertices, axis=0)
    radius = float(toe.radius)

    heel_hits = _vertical_surface_intersections_y(
        triangles,
        float(heel.location[0]),
        float(heel.location[2]),
    )
    heel_plantar_y = min(heel_hits)
    heel_bottom_y = float(heel.location[1]) - float(heel.radius)
    plantar_protrusion = heel_plantar_y - heel_bottom_y
    if not (0.0 < plantar_protrusion < 0.020):
        raise ValueError(
            "heel anchor protrusion must be positive and below 20 mm, got "
            f"{plantar_protrusion:.9g} m"
        )

    # The sphere's anterior edge is flush with the mesh tip.  This is a simple
    # physical/geometric rule; no gait-event timing enters the construction.
    toe_x = float(bounds_max[0] - radius)
    section_z_min, section_z_max = _section_z_bounds_at_x(triangles, toe_x)
    toe_z = 0.5 * (section_z_min + section_z_max)
    toe_hits = _vertical_surface_intersections_y(triangles, toe_x, toe_z)
    toe_plantar_y = min(toe_hits)
    toe_y = toe_plantar_y - plantar_protrusion + radius
    location = (toe_x, toe_y, toe_z)

    if not np.all(np.isfinite(location)):
        raise ValueError("derived toe location is non-finite")
    toe_mesh_distance = _minimum_mesh_distance(location, triangles)
    heel_mesh_distance = _minimum_mesh_distance(heel.location, triangles)
    if toe_mesh_distance > radius + 0.005:
        raise ValueError("derived toe sphere is more than 5 mm from the mesh")

    diagnostics = {
        "mesh_bounds_min_m": bounds_min.tolist(),
        "mesh_bounds_max_m": bounds_max.tolist(),
        "heel_plantar_surface_y_m": heel_plantar_y,
        "heel_bottom_y_m": heel_bottom_y,
        "plantar_protrusion_m": plantar_protrusion,
        "toe_section_z_min_m": section_z_min,
        "toe_section_z_max_m": section_z_max,
        "toe_plantar_surface_y_m": toe_plantar_y,
        "toe_dorsal_surface_y_m": max(toe_hits),
        "heel_center_to_mesh_m": heel_mesh_distance,
        "toe_center_to_mesh_m": toe_mesh_distance,
        "heel_sphere_surface_gap_to_mesh_m": max(
            0.0, heel_mesh_distance - float(heel.radius)
        ),
        "toe_sphere_surface_gap_to_mesh_m": max(0.0, toe_mesh_distance - radius),
        "heel_toe_bottom_offset_m": (toe_y - radius) - heel_bottom_y,
        "toe_anterior_edge_m": toe_x + radius,
        "mesh_anterior_limit_m": float(bounds_max[0]),
    }
    return location, diagnostics


def build_candidate_profile(
    base_profile_path: Path,
    setup_path: Path,
) -> tuple[OnlineGRFProfile, dict[str, Any]]:
    base_profile_path = base_profile_path.resolve()
    setup_path = setup_path.resolve()
    base_profile = load_online_grf_profile(base_profile_path)
    sensors = _left_sensor_spheres(base_profile)
    setup = read_setup_xml(setup_path)
    model_path = setup.model_file.resolve()
    mesh_path = _resolve_left_foot_mesh(model_path)
    triangles = _load_stl_triangles(mesh_path)

    location, geometry = derive_left_toe_location(
        triangles,
        sensors["left_heel"],
        sensors["left_toe"],
    )
    replacement = replace(sensors["left_toe"], location=location)
    spheres = tuple(
        replacement if sphere.name == sensors["left_toe"].name else sphere
        for sphere in base_profile.spheres
    )
    metadata = {
        "status": "experimental_not_promoted",
        "note": (
            "V4 geometry-only detector candidate; explicit opt-in only. "
            "The legacy detector profile remains the runtime default."
        ),
        "v4_mesh_design": {
            "design_id": DESIGN_ID,
            "design_uses_prescribed_timing_or_force_data": False,
            "changed_runtime_field": "spheres[left_toe].location",
            "heel_anchor_unchanged": True,
            "toe_rule": {
                "x": "mesh_anterior_limit_minus_frozen_radius",
                "z": "midpoint_of_mesh_section_at_toe_x",
                "y": "equal_plantar_protrusion_to_frozen_heel_anchor",
            },
            "base_profile": {
                "path": _portable_path(base_profile_path),
                "sha256": _sha256(base_profile_path),
            },
            "setup": {
                "path": _portable_path(setup_path),
                "sha256": _sha256(setup_path),
            },
            "model": {
                "path": _portable_path(model_path),
                "sha256": _sha256(model_path),
            },
            "foot_mesh": {
                "path": _portable_path(mesh_path),
                "sha256": _sha256(mesh_path),
            },
            "frozen_fields": [
                "version",
                "ground",
                "material",
                "heel_strike_confirmation_threshold_n",
                "spheres[left_heel]",
                "spheres[right_heel]",
                "spheres[right_toe]",
                "spheres[*].radius",
                "spheres[*].frame",
                "spheres[*].side",
                "spheres[*].name",
            ],
            "geometry_diagnostics": geometry,
        },
    }
    candidate = replace(
        base_profile,
        source="mesh_based_detector_experimental_v4",
        spheres=spheres,
        metadata=metadata,
    )
    return candidate, geometry


def _canonical(payload: dict[str, Any]) -> str:
    return json.dumps(payload, sort_keys=True, separators=(",", ":"))


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--setup", default=DEFAULT_SETUP)
    parser.add_argument("--base-profile", default=DEFAULT_BASE_PROFILE)
    parser.add_argument("--output", default=DEFAULT_OUTPUT)
    action = parser.add_mutually_exclusive_group()
    action.add_argument(
        "--check",
        action="store_true",
        help="fail unless the existing output exactly matches this builder",
    )
    action.add_argument(
        "--print-json",
        action="store_true",
        help="print the generated JSON without editing the workspace",
    )
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    setup_path = resolve_repo_path(args.setup)
    base_profile_path = resolve_repo_path(args.base_profile)
    output_path = resolve_repo_path(args.output)
    candidate, geometry = build_candidate_profile(base_profile_path, setup_path)
    payload = candidate.to_dict()

    if args.print_json:
        print(json.dumps(payload, indent=2) + "\n", end="")
        return 0
    if args.check:
        if not output_path.is_file():
            raise FileNotFoundError(output_path)
        existing = json.loads(output_path.read_text(encoding="utf-8"))
        if _canonical(existing) != _canonical(payload):
            raise ValueError(f"V4 profile does not match deterministic builder: {output_path}")
        action = "verified"
    else:
        write_online_grf_profile(candidate, output_path)
        action = "written"

    summary = {
        "action": action,
        "profile": _portable_path(output_path),
        "design_id": DESIGN_ID,
        "left_toe_location_m": list(
            _left_sensor_spheres(candidate)["left_toe"].location
        ),
        "geometry": geometry,
    }
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
