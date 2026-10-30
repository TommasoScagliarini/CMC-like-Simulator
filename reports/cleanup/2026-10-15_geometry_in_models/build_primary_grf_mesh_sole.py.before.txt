#!/usr/bin/env python3
"""Materialize one preregistered, mesh-constrained primary-GRF profile.

This builder is deliberately geometry-only.  It reads a frozen left-foot mesh,
model identity, and historical profile identity, then creates a *left-only*
contact sole for the hybrid contract:

    left physical support  = online primary GRF
    right physical support = prescribed ExternalLoads

At every frozen longitudinal section, lateral centers are the linear 0.25 and
0.75 quantiles of the raw deduplicated mesh-intersection z coordinates.  Each
center must lie in the section and its sphere footprint must overlap the
section positively; full lateral footprint containment is not required by the
preregistered grid.

The treadmill velocity is supplied as a plateau speed and mapped through the
frozen AB06 direction ``[0, 0, +1]``.  It is never fitted.  Historical sphere
locations, per-sphere material overrides, residuals, and absolute-path metadata
are not copied.

Only one candidate is written per invocation, under
``validation/experimental_primary_grf_profiles``.  The CLI validates that
boundary before creating any directory.  Output creation uses O_EXCL, so an
existing path is a hard no-clobber failure.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import re
import struct
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]

DEFAULT_CANDIDATE_GRID_PATH = Path(
    "validation/primary_grf_candidate_grid_v1.json"
)
DEFAULT_CANDIDATE_GRID_SHA256 = (
    "cf368f340ee17ae4e4adcd953c962979fee517d2420508b6781d5fee67010383"
)
EXPERIMENTAL_OUTPUT_DIRECTORY = Path(
    "validation/experimental_primary_grf_profiles"
)
DEFAULT_MESH_PATH = Path("Geometry/AM_foot_l.STL")
DEFAULT_MESH_SHA256 = (
    "fcfc4d7a90c4ccd3bedb501ec3e50d4337aa9ca6e8438b58cc6be00f47a689e9"
)
DEFAULT_MODEL_PATH = Path(
    "models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi.osim"
)
DEFAULT_MODEL_SHA256 = (
    "33e67d84bf11740eac509f620a143ad3c57d98c6f765d857e69c1892513de0c1"
)
DEFAULT_BASE_PROFILE_PATH = Path(
    "online_grf_profiles/"
    "AB06_SEASEA_stiff321_500_pi_grf_correct_magnitude.json"
)
DEFAULT_BASE_PROFILE_SHA256 = (
    "09e04ab94954703d74acc3a80b24ecefcc07d3fc918c03b9e9df8116a6c1a2b0"
)

GROUND_NORMAL = (0.0, 1.0, 0.0)
SURFACE_DIRECTION = (0.0, 0.0, 1.0)
BASE_GROUND_ORIGIN_M = (0.0, 0.0, 0.0)
GROUND_ORIGIN_DELTA_GRID_M = (-0.004, -0.002, 0.0, 0.002, 0.004)
LEFT_FRAME = "/bodyset/foot_l"
LONGITUDINAL_FRACTION_MIN = 0.08
LONGITUDINAL_FRACTION_MAX = 0.92
LONGITUDINAL_COUNT_DEFAULT = 8
LONGITUDINAL_COUNT_GRID = (8, 10)
LATERAL_SECTION_QUANTILES = (0.25, 0.75)
RADIUS_DEFAULT_M = 0.020
RADIUS_GRID_M = (0.018, 0.020, 0.0229053623)
FOREFOOT_OFFSET_GRID_M = (-0.002, 0.0, 0.002)
MESH_BOUNDS_MIN_M = (
    -0.1109737753868103,
    -0.04522102326154709,
    -0.040552083402872086,
)
MESH_BOUNDS_MAX_M = (
    0.17601655423641205,
    0.01600000075995922,
    0.04795738682150841,
)
FOOTPRINT_TOLERANCE_M = 1.0e-10
INTERSECTION_TOLERANCE = 1.0e-10

FROZEN_MATERIAL: Mapping[str, float] = {
    "stiffness": 46291.9265,
    "exponent": 1.5,
    "dissipation": 0.1920624923,
    "static_friction": 0.2981956025,
    "dynamic_friction": 0.2981956025,
    "viscous_friction": 0.0,
    "transition_velocity": 0.2,
    "smoothing": 1.0e-5,
}

RESIDUAL_ZERO_FIELDS: dict[str, object] = {
    "residual_force_ratio": [0.0, 0.0, 0.0],
    "residual_moment_ratio_m": [0.0, 0.0, 0.0],
    "residual_penetration_reference_m": 0.0,
    "residual_force_penetration_gain_per_m": [0.0, 0.0, 0.0],
    "residual_force_penetration_rate_gain_s_per_m": [0.0, 0.0, 0.0],
}


class MeshSoleError(RuntimeError):
    """Base class for fail-closed mesh-sole generation errors."""


class SourceIdentityError(MeshSoleError):
    """A frozen source is missing or has an unexpected SHA-256."""


class CandidateGridError(MeshSoleError):
    """The signed candidate grid disagrees with the executable contract."""


class MeshFormatError(MeshSoleError):
    """An STL cannot be parsed as a finite nondegenerate triangle mesh."""


class GeometryError(MeshSoleError):
    """A generated sphere violates the frozen mesh geometry contract."""


class NoClobberError(MeshSoleError):
    """The requested output already exists."""


class OutputPathError(MeshSoleError):
    """A CLI destination is outside the experimental profile directory."""


@dataclass(frozen=True)
class CandidateSpec:
    candidate_id: str
    axis: str
    ground_origin_delta_m: float = 0.0
    radius_m: float = RADIUS_DEFAULT_M
    longitudinal_count: int = LONGITUDINAL_COUNT_DEFAULT
    forefoot_offset_m: float = 0.0


_CANDIDATES = (
    CandidateSpec("G00_mesh_r20_n8_go0_ff0", "baseline"),
    CandidateSpec(
        "G01_ground_m2mm",
        "ground_origin_delta_m",
        ground_origin_delta_m=-0.002,
    ),
    CandidateSpec(
        "G02_ground_p2mm",
        "ground_origin_delta_m",
        ground_origin_delta_m=0.002,
    ),
    CandidateSpec(
        "G03_ground_m4mm",
        "ground_origin_delta_m",
        ground_origin_delta_m=-0.004,
    ),
    CandidateSpec(
        "G04_ground_p4mm",
        "ground_origin_delta_m",
        ground_origin_delta_m=0.004,
    ),
    CandidateSpec("G05_radius_18mm", "sphere_radius_m", radius_m=0.018),
    CandidateSpec(
        "G06_radius_22p9053623mm",
        "sphere_radius_m",
        radius_m=0.0229053623,
    ),
    CandidateSpec(
        "G07_longitudinal_count_10",
        "longitudinal_count",
        longitudinal_count=10,
    ),
    CandidateSpec(
        "G08_forefoot_m2mm",
        "forefoot_offset_m",
        forefoot_offset_m=-0.002,
    ),
    CandidateSpec(
        "G09_forefoot_p2mm",
        "forefoot_offset_m",
        forefoot_offset_m=0.002,
    ),
)
CANDIDATE_SPECS: Mapping[str, CandidateSpec] = {
    item.candidate_id: item for item in _CANDIDATES
}


def _finite_float(value: object, label: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise MeshSoleError(f"{label} must be numeric") from exc
    if not math.isfinite(result):
        raise MeshSoleError(f"{label} must be finite")
    return result


def _sha256_bytes(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def verify_file_identity(path: Path, expected_sha256: str, label: str) -> str:
    expected = str(expected_sha256).strip().lower()
    if not re.fullmatch(r"[0-9a-f]{64}", expected):
        raise SourceIdentityError(f"{label} expected SHA-256 is malformed")
    if not path.is_file():
        raise SourceIdentityError(f"{label} not found: {path}")
    actual = _sha256_file(path)
    if actual != expected:
        raise SourceIdentityError(
            f"{label} SHA-256 mismatch: expected {expected}, got {actual}"
        )
    return actual


def _strict_json_load(path: Path, label: str) -> dict[str, Any]:
    def reject_constant(token: str) -> None:
        raise ValueError(f"non-finite JSON constant {token}")

    def unique_object(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
        result: dict[str, Any] = {}
        for key, value in pairs:
            if key in result:
                raise ValueError(f"duplicate JSON key {key!r}")
            result[key] = value
        return result

    try:
        payload = json.loads(
            path.read_text(encoding="utf-8"),
            parse_constant=reject_constant,
            object_pairs_hook=unique_object,
        )
    except (OSError, UnicodeError, ValueError, json.JSONDecodeError) as exc:
        raise SourceIdentityError(f"invalid {label} JSON: {path}") from exc
    if not isinstance(payload, dict):
        raise SourceIdentityError(f"{label} JSON root must be an object")
    return payload


def _canonical_json(value: object, label: str) -> str:
    try:
        return json.dumps(
            value,
            sort_keys=True,
            separators=(",", ":"),
            allow_nan=False,
        )
    except (TypeError, ValueError) as exc:
        raise CandidateGridError(f"{label} is not finite JSON data") from exc


def _require_exact(actual: object, expected: object, label: str) -> None:
    if _canonical_json(actual, label) != _canonical_json(expected, label):
        raise CandidateGridError(f"candidate grid mismatch: {label}")


def _require_mapping(value: object, label: str) -> Mapping[str, object]:
    if not isinstance(value, dict):
        raise CandidateGridError(f"candidate grid field must be an object: {label}")
    return value


def _candidate_grid_value(spec: CandidateSpec) -> float | int:
    if spec.axis == "baseline":
        return 0.0
    if spec.axis == "ground_origin_delta_m":
        return spec.ground_origin_delta_m
    if spec.axis == "sphere_radius_m":
        return spec.radius_m
    if spec.axis == "longitudinal_count":
        return spec.longitudinal_count
    if spec.axis == "forefoot_offset_m":
        return spec.forefoot_offset_m
    raise CandidateGridError(f"unsupported code candidate axis: {spec.axis!r}")


def _validate_code_candidate_specs() -> None:
    expected_mapping = {spec.candidate_id: spec for spec in _CANDIDATES}
    if dict(CANDIDATE_SPECS) != expected_mapping:
        raise CandidateGridError("CANDIDATE_SPECS mapping drifted from _CANDIDATES")
    if len(expected_mapping) != len(_CANDIDATES):
        raise CandidateGridError("code candidate IDs are not unique")

    baseline = CandidateSpec("baseline_contract", "baseline")
    for spec in _CANDIDATES:
        _finite_float(spec.ground_origin_delta_m, f"{spec.candidate_id}.ground")
        radius = _finite_float(spec.radius_m, f"{spec.candidate_id}.radius")
        offset = _finite_float(spec.forefoot_offset_m, f"{spec.candidate_id}.offset")
        if radius <= 0.0:
            raise CandidateGridError(f"{spec.candidate_id} radius is not positive")
        if spec.longitudinal_count not in LONGITUDINAL_COUNT_GRID:
            raise CandidateGridError(
                f"{spec.candidate_id} longitudinal count is outside frozen grid"
            )

        changed_fields = {
            name
            for name in (
                "ground_origin_delta_m",
                "radius_m",
                "longitudinal_count",
                "forefoot_offset_m",
            )
            if getattr(spec, name) != getattr(baseline, name)
        }
        expected_changes_by_axis = {
            "ground_origin_delta_m": {"ground_origin_delta_m"},
            "sphere_radius_m": {"radius_m"},
            "longitudinal_count": {"longitudinal_count"},
            "forefoot_offset_m": {"forefoot_offset_m"},
            "baseline": set(),
        }
        try:
            expected_changed = expected_changes_by_axis[spec.axis]
        except KeyError as exc:
            raise CandidateGridError(
                f"{spec.candidate_id} has unsupported axis {spec.axis!r}"
            ) from exc
        if changed_fields != expected_changed:
            raise CandidateGridError(
                f"{spec.candidate_id} is not the frozen single-axis candidate"
            )
        if spec.ground_origin_delta_m not in GROUND_ORIGIN_DELTA_GRID_M:
            raise CandidateGridError(f"{spec.candidate_id} ground value is off-grid")
        if radius not in RADIUS_GRID_M:
            raise CandidateGridError(f"{spec.candidate_id} radius is off-grid")
        if offset not in FOREFOOT_OFFSET_GRID_M:
            raise CandidateGridError(f"{spec.candidate_id} offset is off-grid")


def validate_candidate_grid_contract(grid: Mapping[str, object]) -> None:
    """Validate the signed grid against every executable geometry constant."""

    _validate_code_candidate_specs()
    _require_exact(grid.get("schema_version"), 1, "schema_version")
    _require_exact(
        grid.get("grid_id"),
        "AB06_PRIMARY_GRF_CANDIDATE_GRID_V1",
        "grid_id",
    )
    _require_exact(grid.get("status"), "FROZEN", "status")
    _require_exact(grid.get("step_id"), "0", "step_id")
    _require_exact(
        grid.get("baseline_profile"),
        {
            "path": DEFAULT_BASE_PROFILE_PATH.as_posix(),
            "sha256": DEFAULT_BASE_PROFILE_SHA256,
            "role": "BASELINE_FORENSICS_DIAGNOSTIC_NONSELECTABLE",
        },
        "baseline_profile",
    )
    _require_exact(
        grid.get("physical_contract"),
        {
            "primary_applied_side": "left",
            "right_physical_support": "prescribed",
            "detector_role": "disabled",
            "detector_may_affect_primary_metrics": False,
            "runtime_forbidden_inputs": [
                "time",
                "gait_phase",
                "prescribed_grf",
                "target_kinematics",
                "trial_identifier",
            ],
        },
        "physical_contract",
    )
    _require_exact(
        grid.get("upstream_locks"),
        {
            "surface_velocity_audit": {
                "path": (
                    "validation/"
                    "primary_grf_surface_velocity_audit_2026-07-24.json"
                ),
                "sha256": (
                    "d173af6b1ad025874d45c37d37d41e558f3458761e87a720b3e5b1f2074d02d4"
                ),
            },
            "model_compatibility_audit": {
                "path": (
                    "validation/"
                    "primary_grf_model_compatibility_audit_2026-07-24.json"
                ),
                "sha256": (
                    "7bda56ec2a11f4ec78fd9a379ab6f3e24f2fda31714f2fb94e414363057be848"
                ),
            },
        },
        "upstream_locks",
    )

    geometry = _require_mapping(
        grid.get("geometry_candidate_grid"),
        "geometry_candidate_grid",
    )
    _require_exact(
        geometry.get("strategy"),
        "MESH_CONSTRAINED_SEQUENTIAL_OFAT",
        "geometry_candidate_grid.strategy",
    )
    ground = _require_mapping(
        geometry.get("ground"),
        "geometry_candidate_grid.ground",
    )
    _require_exact(
        ground.get("base_origin_m"),
        list(BASE_GROUND_ORIGIN_M),
        "ground.base_origin_m",
    )
    _require_exact(
        ground.get("normal"),
        list(GROUND_NORMAL),
        "ground.normal",
    )
    _require_exact(
        ground.get("origin_delta_along_normal_m"),
        list(GROUND_ORIGIN_DELTA_GRID_M),
        "ground.origin_delta_along_normal_m",
    )
    _require_exact(ground.get("normal_is_frozen"), True, "ground.normal_is_frozen")

    velocity = _require_mapping(
        geometry.get("surface_velocity"),
        "geometry_candidate_grid.surface_velocity",
    )
    _require_exact(
        velocity,
        {
            "rule": "METADATA_SPEED_TIMES_FROZEN_DIRECTION",
            "direction": list(SURFACE_DIRECTION),
            "formula": "surface_velocity_mps = [0.0, 0.0, plateau_speed_mps]",
            "candidate_axis": False,
            "fitting_allowed": False,
        },
        "surface_velocity",
    )

    template = _require_mapping(
        geometry.get("mesh_sole_template"),
        "geometry_candidate_grid.mesh_sole_template",
    )
    _require_exact(
        template.get("design_id"),
        "AB06_LEFT_MESH_SOLE_V1",
        "mesh_sole_template.design_id",
    )
    _require_exact(
        template.get("mesh"),
        {
            "path": DEFAULT_MESH_PATH.as_posix(),
            "sha256": DEFAULT_MESH_SHA256,
            "bounds_min_m": list(MESH_BOUNDS_MIN_M),
            "bounds_max_m": list(MESH_BOUNDS_MAX_M),
        },
        "mesh_sole_template.mesh",
    )
    _require_exact(template.get("frame"), LEFT_FRAME, "mesh_sole_template.frame")
    _require_exact(
        template.get("longitudinal_fraction_interval"),
        [LONGITUDINAL_FRACTION_MIN, LONGITUDINAL_FRACTION_MAX],
        "mesh_sole_template.longitudinal_fraction_interval",
    )
    _require_exact(
        template.get("longitudinal_count_default"),
        LONGITUDINAL_COUNT_DEFAULT,
        "mesh_sole_template.longitudinal_count_default",
    )
    _require_exact(
        template.get("longitudinal_count_grid"),
        list(LONGITUDINAL_COUNT_GRID),
        "mesh_sole_template.longitudinal_count_grid",
    )
    _require_exact(
        template.get("lateral_section_quantiles"),
        list(LATERAL_SECTION_QUANTILES),
        "mesh_sole_template.lateral_section_quantiles",
    )
    _require_exact(
        template.get("radius_default_m"),
        RADIUS_DEFAULT_M,
        "mesh_sole_template.radius_default_m",
    )
    _require_exact(
        template.get("radius_grid_m"),
        list(RADIUS_GRID_M),
        "mesh_sole_template.radius_grid_m",
    )
    _require_exact(
        template.get("forefoot_offset_grid_m"),
        list(FOREFOOT_OFFSET_GRID_M),
        "mesh_sole_template.forefoot_offset_grid_m",
    )
    _require_exact(
        template.get("sphere_footprint_must_intersect_section"),
        True,
        "mesh_sole_template.sphere_footprint_must_intersect_section",
    )
    _require_exact(
        template.get("residuals"),
        "ALL_ZERO",
        "mesh_sole_template.residuals",
    )
    _require_exact(
        template.get("left_sphere_material"),
        "INHERIT_FROZEN_PROFILE_MATERIAL",
        "mesh_sole_template.left_sphere_material",
    )
    _require_exact(
        template.get("right_profile_spheres"),
        "OMITTED_FROM_PRIMARY_CANDIDATE",
        "mesh_sole_template.right_profile_spheres",
    )
    _require_exact(
        geometry.get("frozen_geometry_stage_material"),
        dict(FROZEN_MATERIAL),
        "frozen_geometry_stage_material",
    )
    expected_candidates = [
        {
            "id": spec.candidate_id,
            "axis": spec.axis,
            "value": _candidate_grid_value(spec),
        }
        for spec in _CANDIDATES
    ]
    _require_exact(
        geometry.get("candidates"),
        expected_candidates,
        "geometry_candidate_grid.candidates",
    )


def load_frozen_candidate_grid(
    repo_root: Path = REPO_ROOT,
) -> tuple[dict[str, Any], Path, str]:
    grid_path = (repo_root / DEFAULT_CANDIDATE_GRID_PATH).resolve()
    verified_hash = verify_file_identity(
        grid_path,
        DEFAULT_CANDIDATE_GRID_SHA256,
        "candidate grid",
    )
    grid = _strict_json_load(grid_path, "candidate grid")
    validate_candidate_grid_contract(grid)
    return grid, grid_path, verified_hash


def _validate_triangles(triangles: np.ndarray) -> np.ndarray:
    array = np.asarray(triangles, dtype=float)
    if array.ndim != 3 or array.shape[1:] != (3, 3) or array.shape[0] < 1:
        raise MeshFormatError("STL must contain at least one 3-vertex triangle")
    if not np.all(np.isfinite(array)):
        raise MeshFormatError("STL contains non-finite vertices")
    cross = np.cross(array[:, 1] - array[:, 0], array[:, 2] - array[:, 0])
    if np.any(np.linalg.norm(cross, axis=1) <= 1.0e-15):
        raise MeshFormatError("STL contains a degenerate triangle")
    return array


def _parse_binary_stl(payload: bytes) -> np.ndarray | None:
    if len(payload) < 84:
        return None
    count = struct.unpack_from("<I", payload, 80)[0]
    if 84 + 50 * count != len(payload):
        return None
    triangles = np.empty((count, 3, 3), dtype=float)
    offset = 84
    for index in range(count):
        vertices = struct.unpack_from("<9f", payload, offset + 12)
        triangles[index] = np.asarray(vertices, dtype=float).reshape(3, 3)
        offset += 50
    return _validate_triangles(triangles)


def _parse_ascii_stl(payload: bytes) -> np.ndarray:
    try:
        text = payload.decode("utf-8-sig")
    except UnicodeDecodeError as exc:
        raise MeshFormatError("STL is neither valid binary nor UTF-8 ASCII") from exc
    if not re.search(r"(?im)^\s*solid(?:\s|$)", text):
        raise MeshFormatError("ASCII STL must start with 'solid'")
    if not re.search(r"(?im)^\s*endsolid(?:\s|$)", text):
        raise MeshFormatError("ASCII STL must end with 'endsolid'")

    vertices: list[tuple[float, float, float]] = []
    for match in re.finditer(
        r"(?im)^\s*vertex\s+"
        r"([^\s]+)\s+([^\s]+)\s+([^\s]+)\s*$",
        text,
    ):
        try:
            vertex = tuple(float(match.group(i)) for i in (1, 2, 3))
        except ValueError as exc:
            raise MeshFormatError("ASCII STL has a non-numeric vertex") from exc
        vertices.append(vertex)  # type: ignore[arg-type]
    if not vertices or len(vertices) % 3:
        raise MeshFormatError("ASCII STL vertex count must be a positive multiple of 3")
    return _validate_triangles(np.asarray(vertices, dtype=float).reshape(-1, 3, 3))


def load_stl_triangles(path: Path) -> np.ndarray:
    try:
        payload = path.read_bytes()
    except OSError as exc:
        raise MeshFormatError(f"cannot read STL: {path}") from exc
    binary = _parse_binary_stl(payload)
    return binary if binary is not None else _parse_ascii_stl(payload)


def _deduplicate_scalars(
    values: Iterable[float],
    tolerance: float = INTERSECTION_TOLERANCE,
) -> list[float]:
    ordered = sorted(float(value) for value in values)
    unique: list[float] = []
    for value in ordered:
        if not unique or abs(value - unique[-1]) > tolerance:
            unique.append(value)
    return unique


def section_z_intersections(
    triangles: np.ndarray,
    x_value: float,
    *,
    tolerance: float = 1.0e-12,
) -> list[float]:
    intersections: list[float] = []
    for triangle in triangles:
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
    unique = _deduplicate_scalars(intersections)
    if len(unique) < 2:
        raise GeometryError(f"mesh has no closed section at x={x_value:.12g}")
    return unique


def section_z_bounds(
    triangles: np.ndarray,
    x_value: float,
    *,
    tolerance: float = 1.0e-12,
) -> tuple[float, float]:
    intersections = section_z_intersections(
        triangles,
        x_value,
        tolerance=tolerance,
    )
    return intersections[0], intersections[-1]


def vertical_surface_intersections_y(
    triangles: np.ndarray,
    x_value: float,
    z_value: float,
    *,
    tolerance: float = INTERSECTION_TOLERANCE,
) -> list[float]:
    target = np.asarray([x_value, z_value], dtype=float)
    intersections: list[float] = []
    for triangle in triangles:
        projected = triangle[:, (0, 2)]
        matrix = np.column_stack(
            (projected[1] - projected[0], projected[2] - projected[0])
        )
        determinant = float(np.linalg.det(matrix))
        if abs(determinant) <= 1.0e-15:
            continue
        weights_12 = np.linalg.solve(matrix, target - projected[0])
        weights = np.asarray(
            [1.0 - weights_12.sum(), weights_12[0], weights_12[1]],
            dtype=float,
        )
        if np.all(weights >= -tolerance) and np.all(weights <= 1.0 + tolerance):
            intersections.append(float(weights @ triangle[:, 1]))
    unique = _deduplicate_scalars(intersections, tolerance)
    if len(unique) < 2:
        raise GeometryError(
            "vertical mesh line does not cross plantar and dorsal surfaces: "
            f"x={x_value:.12g}, z={z_value:.12g}"
        )
    return unique


def _forefoot_ramp(longitudinal_fraction: float) -> float:
    return min(
        1.0,
        max(0.0, (float(longitudinal_fraction) - 0.50) / 0.42),
    )


def _validate_spheres(
    spheres: Sequence[Mapping[str, object]],
    section_contracts: Sequence[Mapping[str, float]],
    mesh_bounds_min: np.ndarray,
    mesh_bounds_max: np.ndarray,
) -> None:
    if len(spheres) != len(section_contracts):
        raise GeometryError("sphere/section contract count mismatch")
    names: set[str] = set()
    locations: set[tuple[float, float, float]] = set()
    for sphere, contract in zip(spheres, section_contracts):
        name = str(sphere.get("name", "")).strip()
        if not name or name in names:
            raise GeometryError(f"duplicate or empty sphere name: {name!r}")
        names.add(name)
        radius = _finite_float(sphere.get("radius"), f"{name}.radius")
        if radius <= 0.0:
            raise GeometryError(f"{name}.radius must be positive")
        raw_location = sphere.get("location")
        if not isinstance(raw_location, list) or len(raw_location) != 3:
            raise GeometryError(f"{name}.location must be a three-vector")
        location = tuple(
            _finite_float(value, f"{name}.location") for value in raw_location
        )
        if location in locations:
            raise GeometryError(f"duplicate sphere location: {location}")
        locations.add(location)

        x_value, _, z_value = location
        if x_value - radius < mesh_bounds_min[0] - FOOTPRINT_TOLERANCE_M:
            raise GeometryError(f"{name} posterior footprint is outside mesh")
        if x_value + radius > mesh_bounds_max[0] + FOOTPRINT_TOLERANCE_M:
            raise GeometryError(f"{name} anterior footprint is outside mesh")
        z_min = float(contract["section_z_min_m"])
        z_max = float(contract["section_z_max_m"])
        if (
            z_value < z_min - FOOTPRINT_TOLERANCE_M
            or z_value > z_max + FOOTPRINT_TOLERANCE_M
        ):
            raise GeometryError(f"{name} center is outside mesh section")
        footprint_overlap = min(z_value + radius, z_max) - max(
            z_value - radius,
            z_min,
        )
        if footprint_overlap <= FOOTPRINT_TOLERANCE_M:
            raise GeometryError(
                f"{name} footprint does not positively intersect mesh section"
            )


def generate_mesh_spheres(
    triangles: np.ndarray,
    *,
    radius_m: float,
    longitudinal_count: int,
    forefoot_offset_m: float,
) -> tuple[list[dict[str, object]], list[dict[str, float]], dict[str, list[float]]]:
    triangles = _validate_triangles(triangles)
    radius = _finite_float(radius_m, "radius_m")
    offset = _finite_float(forefoot_offset_m, "forefoot_offset_m")
    if radius <= 0.0:
        raise GeometryError("radius_m must be positive")
    if isinstance(longitudinal_count, bool) or int(longitudinal_count) != longitudinal_count:
        raise GeometryError("longitudinal_count must be an integer")
    count = int(longitudinal_count)
    if count < 2:
        raise GeometryError("longitudinal_count must be at least 2")

    vertices = triangles.reshape(-1, 3)
    bounds_min = np.min(vertices, axis=0)
    bounds_max = np.max(vertices, axis=0)
    fractions = np.linspace(
        LONGITUDINAL_FRACTION_MIN,
        LONGITUDINAL_FRACTION_MAX,
        count,
    )
    x_values = bounds_min[0] + fractions * (bounds_max[0] - bounds_min[0])

    spheres: list[dict[str, object]] = []
    contracts: list[dict[str, float]] = []
    for longitudinal_index, (fraction, x_value) in enumerate(
        zip(fractions, x_values)
    ):
        if x_value - radius < bounds_min[0] - FOOTPRINT_TOLERANCE_M:
            raise GeometryError("posterior sphere footprint is outside mesh bounds")
        if x_value + radius > bounds_max[0] + FOOTPRINT_TOLERANCE_M:
            raise GeometryError("anterior sphere footprint is outside mesh bounds")
        section_z_values = section_z_intersections(triangles, float(x_value))
        z_min = section_z_values[0]
        z_max = section_z_values[-1]
        lateral: list[tuple[float, list[float]]] = []
        quantile_values = np.quantile(
            np.asarray(section_z_values, dtype=float),
            LATERAL_SECTION_QUANTILES,
            method="linear",
        )
        for quantile, raw_z_value in zip(
            LATERAL_SECTION_QUANTILES,
            quantile_values,
        ):
            z_value = float(raw_z_value)
            if (
                z_value < z_min - FOOTPRINT_TOLERANCE_M
                or z_value > z_max + FOOTPRINT_TOLERANCE_M
            ):
                raise GeometryError(
                    f"lateral quantile {quantile:.2f} center is outside "
                    f"mesh section at x={x_value:.12g}"
                )
            footprint_overlap = min(z_value + radius, z_max) - max(
                z_value - radius,
                z_min,
            )
            if footprint_overlap <= FOOTPRINT_TOLERANCE_M:
                raise GeometryError(
                    f"lateral quantile {quantile:.2f} footprint does not "
                    f"positively intersect mesh section at x={x_value:.12g}"
                )
            y_hits = vertical_surface_intersections_y(
                triangles,
                float(x_value),
                z_value,
            )
            lateral.append((z_value, y_hits))
        if abs(lateral[1][0] - lateral[0][0]) <= FOOTPRINT_TOLERANCE_M:
            raise GeometryError(
                f"lateral sphere locations collapse at x={x_value:.12g}"
            )
        ramp = _forefoot_ramp(float(fraction))
        for lateral_index, (z_value, y_hits) in enumerate(lateral):
            plantar_y = min(y_hits)
            center_y = plantar_y + radius + offset * ramp
            sphere = {
                "name": (
                    f"left_primary_mesh_x{longitudinal_index:02d}_"
                    f"z{lateral_index:02d}"
                ),
                "side": "left",
                "frame": LEFT_FRAME,
                "location": [
                    float(x_value),
                    float(center_y),
                    float(z_value),
                ],
                "radius": radius,
                **RESIDUAL_ZERO_FIELDS,
            }
            spheres.append(sphere)
            contracts.append(
                {
                    "section_z_min_m": float(z_min),
                    "section_z_max_m": float(z_max),
                    "plantar_surface_y_m": float(plantar_y),
                    "forefoot_ramp": float(ramp),
                    "lateral_section_quantile": float(
                        LATERAL_SECTION_QUANTILES[lateral_index]
                    ),
                }
            )

    _validate_spheres(spheres, contracts, bounds_min, bounds_max)
    return spheres, contracts, {
        "min_m": [float(value) for value in bounds_min],
        "max_m": [float(value) for value in bounds_max],
    }


def _repo_relative(path: Path, repo_root: Path) -> str:
    resolved = path.resolve()
    try:
        relative = resolved.relative_to(repo_root.resolve())
    except ValueError as exc:
        raise SourceIdentityError(
            f"source path must be inside repository root: {resolved}"
        ) from exc
    text = relative.as_posix()
    if not text or text.startswith("/") or re.match(r"^[A-Za-z]:", text):
        raise SourceIdentityError(f"source path is not repository-relative: {text}")
    return text


def _validate_frozen_source_request(
    *,
    path: Path,
    requested_sha256: str,
    expected_path: Path,
    expected_sha256: str,
    repo_root: Path,
    label: str,
) -> tuple[Path, str]:
    resolved = path.resolve()
    expected_resolved = (repo_root / expected_path).resolve()
    if resolved != expected_resolved:
        raise SourceIdentityError(
            f"{label} path is not frozen grid source: {resolved}"
        )
    requested_hash = str(requested_sha256).strip().lower()
    if requested_hash != expected_sha256:
        raise SourceIdentityError(
            f"{label} requested SHA-256 differs from frozen grid"
        )
    verified_hash = verify_file_identity(resolved, expected_sha256, label)
    return resolved, verified_hash


def _validated_material(base_profile: Mapping[str, object]) -> dict[str, float]:
    raw = base_profile.get("material")
    if not isinstance(raw, dict):
        raise SourceIdentityError("base profile material must be an object")
    required = (
        "stiffness",
        "exponent",
        "dissipation",
        "static_friction",
        "dynamic_friction",
        "viscous_friction",
        "transition_velocity",
        "smoothing",
    )
    if set(raw) != set(required):
        raise SourceIdentityError("base profile material schema drifted")
    material = {
        name: _finite_float(raw[name], f"material.{name}") for name in required
    }
    if material["stiffness"] <= 0.0:
        raise SourceIdentityError("material.stiffness must be positive")
    if material["exponent"] <= 0.0:
        raise SourceIdentityError("material.exponent must be positive")
    if material["transition_velocity"] <= 0.0:
        raise SourceIdentityError("material.transition_velocity must be positive")
    if material["smoothing"] <= 0.0:
        raise SourceIdentityError("material.smoothing must be positive")
    if any(value < 0.0 for value in material.values()):
        raise SourceIdentityError("material fields must be non-negative")
    _require_exact(material, dict(FROZEN_MATERIAL), "base profile frozen material")
    return material


def validate_built_profile_against_grid(
    profile: Mapping[str, object],
    *,
    candidate_grid: Mapping[str, object],
    spec: CandidateSpec,
    plateau_speed_mps: float,
    candidate_grid_sha256: str,
) -> None:
    """Close the loop between the signed grid, code constants, and output."""

    validate_candidate_grid_contract(candidate_grid)
    ground_origin = [
        float(origin + spec.ground_origin_delta_m * normal)
        for origin, normal in zip(BASE_GROUND_ORIGIN_M, GROUND_NORMAL)
    ]
    surface_velocity = [
        float(plateau_speed_mps * direction)
        for direction in SURFACE_DIRECTION
    ]
    _require_exact(profile.get("version"), 1, "output.version")
    _require_exact(
        profile.get("source"),
        "ab06_primary_grf_mesh_sole_v1",
        "output.source",
    )
    _require_exact(
        profile.get("ground"),
        {
            "origin": ground_origin,
            "normal": list(GROUND_NORMAL),
            "surface_velocity": surface_velocity,
        },
        "output.ground",
    )
    _require_exact(
        profile.get("material"),
        dict(FROZEN_MATERIAL),
        "output.material",
    )

    metadata = _require_mapping(profile.get("metadata"), "output.metadata")
    _require_exact(
        metadata.get("candidate_id"),
        spec.candidate_id,
        "output.metadata.candidate_id",
    )
    _require_exact(
        metadata.get("candidate_axis"),
        spec.axis,
        "output.metadata.candidate_axis",
    )
    _require_exact(
        metadata.get("candidate_value"),
        _candidate_grid_value(spec),
        "output.metadata.candidate_value",
    )
    _require_exact(
        metadata.get("application_contract"),
        {
            "primary_applied_sides": ["left"],
            "profile_required_sides": ["left"],
            "right_physical_support": "prescribed",
            "right_online_spheres_present": False,
            "detector_affects_primary": False,
        },
        "output.metadata.application_contract",
    )
    _require_exact(
        metadata.get("surface_velocity_contract"),
        {
            "source": "plateau_speed_metadata",
            "direction": list(SURFACE_DIRECTION),
            "plateau_speed_mps": plateau_speed_mps,
            "fitted": False,
            "candidate_axis": False,
        },
        "output.metadata.surface_velocity_contract",
    )

    design = _require_mapping(
        metadata.get("mesh_sole_design"),
        "output.metadata.mesh_sole_design",
    )
    expected_design_fields = {
        "design_id": "AB06_LEFT_MESH_SOLE_V1",
        "longitudinal_fraction_interval": [
            LONGITUDINAL_FRACTION_MIN,
            LONGITUDINAL_FRACTION_MAX,
        ],
        "longitudinal_count": spec.longitudinal_count,
        "lateral_section_quantiles": list(LATERAL_SECTION_QUANTILES),
        "sphere_footprint_must_intersect_section": True,
        "sphere_footprint_full_containment_required": False,
        "radius_m": spec.radius_m,
        "forefoot_offset_m": spec.forefoot_offset_m,
        "ground_origin_delta_m": spec.ground_origin_delta_m,
        "all_residuals_zero": True,
        "sphere_material_mode": "inherit_profile_material",
        "mesh_bounds": {
            "min_m": list(MESH_BOUNDS_MIN_M),
            "max_m": list(MESH_BOUNDS_MAX_M),
        },
    }
    for field, expected in expected_design_fields.items():
        _require_exact(
            design.get(field),
            expected,
            f"output.metadata.mesh_sole_design.{field}",
        )

    material_digest = _sha256_bytes(
        _canonical_json(dict(FROZEN_MATERIAL), "frozen material").encode("utf-8")
    )
    _require_exact(
        metadata.get("frozen_material"),
        {
            "sha256": material_digest,
            "inherited_by_every_left_sphere": True,
        },
        "output.metadata.frozen_material",
    )
    sources = _require_mapping(metadata.get("sources"), "output.metadata.sources")
    _require_exact(
        sources.get("candidate_grid"),
        {
            "path": DEFAULT_CANDIDATE_GRID_PATH.as_posix(),
            "sha256": candidate_grid_sha256,
        },
        "output.metadata.sources.candidate_grid",
    )
    _require_exact(
        sources.get("mesh"),
        {
            "path": DEFAULT_MESH_PATH.as_posix(),
            "sha256": DEFAULT_MESH_SHA256,
        },
        "output.metadata.sources.mesh",
    )
    _require_exact(
        sources.get("model"),
        {
            "path": DEFAULT_MODEL_PATH.as_posix(),
            "sha256": DEFAULT_MODEL_SHA256,
        },
        "output.metadata.sources.model",
    )
    _require_exact(
        sources.get("historical_profile_identity_only"),
        {
            "path": DEFAULT_BASE_PROFILE_PATH.as_posix(),
            "sha256": DEFAULT_BASE_PROFILE_SHA256,
            "historical_spheres_copied": False,
            "historical_metadata_copied": False,
        },
        "output.metadata.sources.historical_profile_identity_only",
    )

    spheres = profile.get("spheres")
    if not isinstance(spheres, list):
        raise CandidateGridError("output.spheres must be an array")
    if len(spheres) != 2 * spec.longitudinal_count:
        raise CandidateGridError("output sphere count disagrees with frozen candidate")
    for index, sphere_value in enumerate(spheres):
        sphere = _require_mapping(sphere_value, f"output.spheres[{index}]")
        _require_exact(sphere.get("side"), "left", f"sphere[{index}].side")
        _require_exact(sphere.get("frame"), LEFT_FRAME, f"sphere[{index}].frame")
        _require_exact(
            sphere.get("radius"),
            spec.radius_m,
            f"sphere[{index}].radius",
        )
        if "material" in sphere:
            raise CandidateGridError(
                f"output sphere[{index}] overrides frozen profile material"
            )
        for field, expected in RESIDUAL_ZERO_FIELDS.items():
            _require_exact(
                sphere.get(field),
                expected,
                f"sphere[{index}].{field}",
            )


def build_candidate_profile(
    *,
    candidate_id: str,
    plateau_speed_mps: float,
    mesh_path: Path,
    mesh_sha256: str,
    model_path: Path,
    model_sha256: str,
    base_profile_path: Path,
    base_profile_sha256: str,
    repo_root: Path = REPO_ROOT,
) -> dict[str, object]:
    try:
        spec = CANDIDATE_SPECS[candidate_id]
    except KeyError as exc:
        raise MeshSoleError(f"unknown preregistered candidate: {candidate_id}") from exc
    speed = _finite_float(plateau_speed_mps, "plateau_speed_mps")
    if speed < 0.0:
        raise MeshSoleError("plateau_speed_mps must be non-negative")

    candidate_grid, candidate_grid_path, verified_grid = (
        load_frozen_candidate_grid(repo_root)
    )
    mesh_path, verified_mesh = _validate_frozen_source_request(
        path=mesh_path,
        requested_sha256=mesh_sha256,
        expected_path=DEFAULT_MESH_PATH,
        expected_sha256=DEFAULT_MESH_SHA256,
        repo_root=repo_root,
        label="foot mesh",
    )
    model_path, verified_model = _validate_frozen_source_request(
        path=model_path,
        requested_sha256=model_sha256,
        expected_path=DEFAULT_MODEL_PATH,
        expected_sha256=DEFAULT_MODEL_SHA256,
        repo_root=repo_root,
        label="model",
    )
    base_profile_path, verified_profile = _validate_frozen_source_request(
        path=base_profile_path,
        requested_sha256=base_profile_sha256,
        expected_path=DEFAULT_BASE_PROFILE_PATH,
        expected_sha256=DEFAULT_BASE_PROFILE_SHA256,
        repo_root=repo_root,
        label="base profile",
    )
    triangles = load_stl_triangles(mesh_path)
    base_profile = _strict_json_load(base_profile_path, "base profile")
    material = _validated_material(base_profile)
    spheres, contracts, bounds = generate_mesh_spheres(
        triangles,
        radius_m=spec.radius_m,
        longitudinal_count=spec.longitudinal_count,
        forefoot_offset_m=spec.forefoot_offset_m,
    )
    ground_delta = _finite_float(
        spec.ground_origin_delta_m,
        "ground_origin_delta_m",
    )
    ground_origin = [
        float(origin + ground_delta * normal)
        for origin, normal in zip(BASE_GROUND_ORIGIN_M, GROUND_NORMAL)
    ]
    surface_velocity = [
        float(speed * direction) for direction in SURFACE_DIRECTION
    ]
    material_digest = _sha256_bytes(
        _canonical_json(material, "frozen material").encode("utf-8")
    )
    profile: dict[str, object] = {
        "version": 1,
        "source": "ab06_primary_grf_mesh_sole_v1",
        "ground": {
            "origin": ground_origin,
            "normal": list(GROUND_NORMAL),
            "surface_velocity": surface_velocity,
        },
        "material": material,
        "spheres": spheres,
        "metadata": {
            "status": "experimental_not_promoted",
            "candidate_id": spec.candidate_id,
            "candidate_axis": spec.axis,
            "candidate_value": _candidate_grid_value(spec),
            "application_contract": {
                "primary_applied_sides": ["left"],
                "profile_required_sides": ["left"],
                "right_physical_support": "prescribed",
                "right_online_spheres_present": False,
                "detector_affects_primary": False,
            },
            "surface_velocity_contract": {
                "source": "plateau_speed_metadata",
                "direction": list(SURFACE_DIRECTION),
                "plateau_speed_mps": speed,
                "fitted": False,
                "candidate_axis": False,
            },
            "mesh_sole_design": {
                "design_id": "AB06_LEFT_MESH_SOLE_V1",
                "longitudinal_fraction_interval": [
                    LONGITUDINAL_FRACTION_MIN,
                    LONGITUDINAL_FRACTION_MAX,
                ],
                "longitudinal_count": spec.longitudinal_count,
                "lateral_section_quantiles": list(LATERAL_SECTION_QUANTILES),
                "lateral_rule": (
                    "At each longitudinal section, use the 0.25 and 0.75 "
                    "quantiles of mesh-intersection z coordinates."
                ),
                "lateral_quantile_source": (
                    "raw deduplicated mesh-intersection z coordinates"
                ),
                "sphere_center_must_be_in_section": True,
                "sphere_footprint_must_intersect_section": True,
                "sphere_footprint_full_containment_required": False,
                "radius_m": spec.radius_m,
                "forefoot_offset_m": spec.forefoot_offset_m,
                "ground_origin_delta_m": spec.ground_origin_delta_m,
                "all_residuals_zero": True,
                "sphere_material_mode": "inherit_profile_material",
                "mesh_bounds": bounds,
                "sphere_section_contracts": contracts,
            },
            "frozen_material": {
                "sha256": material_digest,
                "inherited_by_every_left_sphere": True,
            },
            "sources": {
                "candidate_grid": {
                    "path": _repo_relative(candidate_grid_path, repo_root),
                    "sha256": verified_grid,
                },
                "mesh": {
                    "path": _repo_relative(mesh_path, repo_root),
                    "sha256": verified_mesh,
                },
                "model": {
                    "path": _repo_relative(model_path, repo_root),
                    "sha256": verified_model,
                },
                "historical_profile_identity_only": {
                    "path": _repo_relative(base_profile_path, repo_root),
                    "sha256": verified_profile,
                    "historical_spheres_copied": False,
                    "historical_metadata_copied": False,
                },
            },
        },
    }
    validate_built_profile_against_grid(
        profile,
        candidate_grid=candidate_grid,
        spec=spec,
        plateau_speed_mps=speed,
        candidate_grid_sha256=verified_grid,
    )
    # Serialization is also the final recursive finite-number check.
    json.dumps(profile, allow_nan=False)
    return profile


def write_json_no_clobber(path: Path, payload: Mapping[str, object]) -> Path:
    encoded = (
        json.dumps(payload, indent=2, sort_keys=True, allow_nan=False) + "\n"
    ).encode("utf-8")
    destination = path.resolve()
    destination.parent.mkdir(parents=True, exist_ok=True)
    flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL
    try:
        descriptor = os.open(destination, flags, 0o644)
    except FileExistsError as exc:
        raise NoClobberError(f"output already exists: {destination}") from exc
    try:
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(encoded)
            stream.flush()
            os.fsync(stream.fileno())
    except BaseException:
        # The exclusive path may contain a partial write.  Preserve it for
        # forensics; never unlink or retry over it automatically.
        raise
    return destination


def _resolve_repo_input(raw: str, repo_root: Path) -> Path:
    path = Path(raw)
    return path if path.is_absolute() else repo_root / path


def validate_cli_output_path(path: Path, repo_root: Path = REPO_ROOT) -> Path:
    """Return a canonical CLI destination without creating any directories."""

    allowed_directory = (repo_root / EXPERIMENTAL_OUTPUT_DIRECTORY).resolve()
    destination = path.resolve()
    try:
        relative = destination.relative_to(allowed_directory)
    except ValueError as exc:
        raise OutputPathError(
            "CLI output must be inside "
            f"{EXPERIMENTAL_OUTPUT_DIRECTORY.as_posix()}"
        ) from exc
    if not relative.parts:
        raise OutputPathError("CLI output must name a file inside the allowed directory")
    if destination.exists() and destination.is_dir():
        raise OutputPathError("CLI output destination is an existing directory")
    return destination


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--candidate-id", required=True, choices=sorted(CANDIDATE_SPECS))
    parser.add_argument("--plateau-speed-mps", required=True, type=float)
    parser.add_argument("--output", required=True)
    parser.add_argument("--mesh", default=DEFAULT_MESH_PATH.as_posix())
    parser.add_argument("--mesh-sha256", default=DEFAULT_MESH_SHA256)
    parser.add_argument("--model", default=DEFAULT_MODEL_PATH.as_posix())
    parser.add_argument("--model-sha256", default=DEFAULT_MODEL_SHA256)
    parser.add_argument(
        "--base-profile",
        default=DEFAULT_BASE_PROFILE_PATH.as_posix(),
    )
    parser.add_argument(
        "--base-profile-sha256",
        default=DEFAULT_BASE_PROFILE_SHA256,
    )
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = _parse_args(argv)
    output_path = validate_cli_output_path(
        _resolve_repo_input(args.output, REPO_ROOT),
        REPO_ROOT,
    )
    mesh_path = _resolve_repo_input(args.mesh, REPO_ROOT)
    model_path = _resolve_repo_input(args.model, REPO_ROOT)
    profile_path = _resolve_repo_input(args.base_profile, REPO_ROOT)
    profile = build_candidate_profile(
        candidate_id=args.candidate_id,
        plateau_speed_mps=args.plateau_speed_mps,
        mesh_path=mesh_path,
        mesh_sha256=args.mesh_sha256,
        model_path=model_path,
        model_sha256=args.model_sha256,
        base_profile_path=profile_path,
        base_profile_sha256=args.base_profile_sha256,
        repo_root=REPO_ROOT,
    )

    # Close the source-identity TOCTOU window immediately before publication.
    verify_file_identity(
        REPO_ROOT / DEFAULT_CANDIDATE_GRID_PATH,
        DEFAULT_CANDIDATE_GRID_SHA256,
        "candidate grid",
    )
    verify_file_identity(mesh_path, DEFAULT_MESH_SHA256, "foot mesh")
    verify_file_identity(model_path, DEFAULT_MODEL_SHA256, "model")
    verify_file_identity(
        profile_path,
        DEFAULT_BASE_PROFILE_SHA256,
        "base profile",
    )
    destination = write_json_no_clobber(output_path, profile)
    summary = {
        "status": "PROFILE_MATERIALIZED",
        "candidate_id": args.candidate_id,
        "output": _repo_relative(destination, REPO_ROOT),
        "output_sha256": _sha256_file(destination),
        "sphere_count": len(profile["spheres"]),  # type: ignore[arg-type]
        "applied_side": "left",
        "right_support": "prescribed",
    }
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
