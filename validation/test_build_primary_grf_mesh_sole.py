from __future__ import annotations

import hashlib
import json
import math
import struct
from pathlib import Path

import numpy as np
import pytest

from validation import build_primary_grf_mesh_sole as builder


def _box_triangles(
    *,
    x_min: float = -1.0,
    x_max: float = 1.0,
    y_min: float = -0.1,
    y_max: float = 0.1,
    z_min: float = -0.5,
    z_max: float = 0.5,
) -> np.ndarray:
    vertices = np.asarray(
        [
            [x_min, y_min, z_min],
            [x_max, y_min, z_min],
            [x_max, y_max, z_min],
            [x_min, y_max, z_min],
            [x_min, y_min, z_max],
            [x_max, y_min, z_max],
            [x_max, y_max, z_max],
            [x_min, y_max, z_max],
        ],
        dtype=float,
    )
    faces = (
        (0, 2, 1),
        (0, 3, 2),
        (4, 5, 6),
        (4, 6, 7),
        (0, 1, 5),
        (0, 5, 4),
        (3, 7, 6),
        (3, 6, 2),
        (0, 4, 7),
        (0, 7, 3),
        (1, 2, 6),
        (1, 6, 5),
    )
    return np.asarray([[vertices[i] for i in face] for face in faces])


def _write_binary_stl(path: Path, triangles: np.ndarray) -> None:
    payload = bytearray(b"synthetic binary STL".ljust(80, b"\0"))
    payload.extend(struct.pack("<I", len(triangles)))
    for triangle in triangles:
        payload.extend(struct.pack("<3f", 0.0, 1.0, 0.0))
        payload.extend(struct.pack("<9f", *triangle.reshape(-1)))
        payload.extend(struct.pack("<H", 0))
    path.write_bytes(bytes(payload))


def _write_ascii_stl(path: Path, triangles: np.ndarray) -> None:
    lines = ["solid synthetic"]
    for triangle in triangles:
        lines.extend(["  facet normal 0 1 0", "    outer loop"])
        lines.extend(
            f"      vertex {point[0]} {point[1]} {point[2]}"
            for point in triangle
        )
        lines.extend(["    endloop", "  endfacet"])
    lines.append("endsolid synthetic")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _canonical_sources() -> dict:
    return {
        "mesh_path": builder.REPO_ROOT / builder.DEFAULT_MESH_PATH,
        "mesh_sha256": builder.DEFAULT_MESH_SHA256,
        "model_path": builder.REPO_ROOT / builder.DEFAULT_MODEL_PATH,
        "model_sha256": builder.DEFAULT_MODEL_SHA256,
        "base_profile_path": (
            builder.REPO_ROOT / builder.DEFAULT_BASE_PROFILE_PATH
        ),
        "base_profile_sha256": builder.DEFAULT_BASE_PROFILE_SHA256,
        "repo_root": builder.REPO_ROOT,
    }


def _cli_args(output: Path) -> list[str]:
    return [
        "--candidate-id",
        "G00_mesh_r20_n8_go0_ff0",
        "--plateau-speed-mps",
        "0.55",
        "--output",
        str(output),
    ]


@pytest.mark.parametrize("ascii_mesh", [False, True])
def test_binary_and_ascii_stl_parse_to_same_synthetic_contract(
    tmp_path: Path,
    ascii_mesh: bool,
) -> None:
    mesh = tmp_path / "AM_foot_l.STL"
    triangles = _box_triangles()
    if ascii_mesh:
        _write_ascii_stl(mesh, triangles)
    else:
        _write_binary_stl(mesh, triangles)
    loaded = builder.load_stl_triangles(mesh)
    spheres, contracts, bounds = builder.generate_mesh_spheres(
        loaded,
        radius_m=0.020,
        longitudinal_count=8,
        forefoot_offset_m=0.0,
    )

    assert len(spheres) == 16
    assert len(contracts) == 16
    assert bounds == {
        "min_m": [-1.0, -0.10000000149011612, -0.5]
        if not ascii_mesh
        else [-1.0, -0.1, -0.5],
        "max_m": [1.0, 0.10000000149011612, 0.5]
        if not ascii_mesh
        else [1.0, 0.1, 0.5],
    }
    assert {sphere["side"] for sphere in spheres} == {"left"}
    assert all("material" not in sphere for sphere in spheres)


def test_canonical_g00_build_is_bound_to_signed_grid() -> None:
    profile = builder.build_candidate_profile(
        candidate_id="G00_mesh_r20_n8_go0_ff0",
        plateau_speed_mps=1.25,
        **_canonical_sources(),
    )

    assert profile["ground"] == {
        "origin": [0.0, 0.0, 0.0],
        "normal": [0.0, 1.0, 0.0],
        "surface_velocity": [0.0, 0.0, 1.25],
    }
    spheres = profile["spheres"]
    assert isinstance(spheres, list)
    assert len(spheres) == 16
    assert {sphere["side"] for sphere in spheres} == {"left"}
    assert all("material" not in sphere for sphere in spheres)
    assert all(
        all(sphere[key] == value for key, value in builder.RESIDUAL_ZERO_FIELDS.items())
        for sphere in spheres
    )
    metadata = profile["metadata"]
    assert metadata["application_contract"]["right_physical_support"] == "prescribed"
    assert metadata["surface_velocity_contract"]["fitted"] is False
    assert metadata["sources"]["mesh"]["path"] == "Geometry/AM_foot_l.STL"
    assert metadata["sources"]["candidate_grid"] == {
        "path": "validation/primary_grf_candidate_grid_v1.json",
        "sha256": builder.DEFAULT_CANDIDATE_GRID_SHA256,
    }
    assert metadata["mesh_sole_design"]["lateral_section_quantiles"] == [0.25, 0.75]
    assert metadata["candidate_value"] == 0.0
    serialized = json.dumps(profile)
    assert "C:\\\\Users\\\\tomma" not in serialized
    assert "support_tuning" not in serialized
    assert "residual_model" not in serialized
    assert "left_basis_00" not in serialized
    assert "right_basis_11" not in serialized


def test_lateral_centers_are_section_quantiles_not_admissible_edges() -> None:
    triangles = _box_triangles()
    radius = 0.020
    spheres, contracts, _ = builder.generate_mesh_spheres(
        triangles,
        radius_m=radius,
        longitudinal_count=8,
        forefoot_offset_m=0.0,
    )

    for longitudinal_index in range(8):
        section_spheres = spheres[
            2 * longitudinal_index : 2 * longitudinal_index + 2
        ]
        x_value = section_spheres[0]["location"][0]
        section_z_values = builder.section_z_intersections(triangles, x_value)
        expected_quantiles = np.quantile(
            section_z_values,
            builder.LATERAL_SECTION_QUANTILES,
            method="linear",
        )
        actual_z = [sphere["location"][2] for sphere in section_spheres]
        admissible_edges = [
            section_z_values[0] + radius,
            section_z_values[-1] - radius,
        ]

        assert np.allclose(actual_z, expected_quantiles, atol=0.0, rtol=0.0)
        assert not np.allclose(actual_z, admissible_edges)
        assert [
            contract["lateral_section_quantile"]
            for contract in contracts[
                2 * longitudinal_index : 2 * longitudinal_index + 2
            ]
        ] == [0.25, 0.75]


def test_hash_mismatch_fails_closed() -> None:
    sources = _canonical_sources()
    sources["mesh_sha256"] = "0" * 64
    with pytest.raises(builder.SourceIdentityError, match="frozen grid"):
        builder.build_candidate_profile(
            candidate_id="G00_mesh_r20_n8_go0_ff0",
            plateau_speed_mps=0.55,
            **sources,
        )


def test_frozen_grid_hash_is_machine_bound() -> None:
    grid_path = builder.REPO_ROOT / builder.DEFAULT_CANDIDATE_GRID_PATH
    assert _sha256(grid_path) == builder.DEFAULT_CANDIDATE_GRID_SHA256
    grid, loaded_path, loaded_hash = builder.load_frozen_candidate_grid()
    assert loaded_path == grid_path.resolve()
    assert loaded_hash == builder.DEFAULT_CANDIDATE_GRID_SHA256
    assert grid["status"] == "FROZEN"


def test_grid_file_drift_fails_before_semantic_use(tmp_path: Path) -> None:
    drifted = tmp_path / builder.DEFAULT_CANDIDATE_GRID_PATH
    drifted.parent.mkdir(parents=True)
    canonical = builder.REPO_ROOT / builder.DEFAULT_CANDIDATE_GRID_PATH
    drifted.write_bytes(canonical.read_bytes() + b" ")

    with pytest.raises(builder.SourceIdentityError, match="SHA-256 mismatch"):
        builder.load_frozen_candidate_grid(tmp_path)


def test_grid_candidate_spec_mismatch_is_rejected() -> None:
    grid_path = builder.REPO_ROOT / builder.DEFAULT_CANDIDATE_GRID_PATH
    grid = json.loads(grid_path.read_text(encoding="utf-8"))
    grid["geometry_candidate_grid"]["candidates"][5]["value"] = 0.019

    with pytest.raises(builder.CandidateGridError, match="candidates"):
        builder.validate_candidate_grid_contract(grid)


def test_code_candidate_mapping_drift_is_rejected(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    grid_path = builder.REPO_ROOT / builder.DEFAULT_CANDIDATE_GRID_PATH
    grid = json.loads(grid_path.read_text(encoding="utf-8"))
    monkeypatch.setattr(
        builder,
        "CANDIDATE_SPECS",
        dict(list(builder.CANDIDATE_SPECS.items())[:-1]),
    )

    with pytest.raises(builder.CandidateGridError, match="mapping drifted"):
        builder.validate_candidate_grid_contract(grid)


def test_cli_grid_hash_drift_creates_no_output(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    output = (
        builder.REPO_ROOT
        / builder.EXPERIMENTAL_OUTPUT_DIRECTORY
        / f"grid_drift_{tmp_path.name}"
        / "candidate.json"
    )
    assert not output.exists()
    monkeypatch.setattr(builder, "DEFAULT_CANDIDATE_GRID_SHA256", "0" * 64)

    with pytest.raises(builder.SourceIdentityError, match="SHA-256 mismatch"):
        builder.main(_cli_args(output))
    assert not output.exists()


@pytest.mark.parametrize("source_kind", ["mesh", "profile"])
def test_cli_rejects_alternate_frozen_sources_without_output(
    tmp_path: Path,
    source_kind: str,
) -> None:
    output = (
        builder.REPO_ROOT
        / builder.EXPERIMENTAL_OUTPUT_DIRECTORY
        / f"alternate_{source_kind}_{tmp_path.name}"
        / "candidate.json"
    )
    assert not output.exists()
    args = _cli_args(output)
    if source_kind == "mesh":
        alternate = tmp_path / "alternate_foot.STL"
        alternate.write_bytes(
            (builder.REPO_ROOT / builder.DEFAULT_MESH_PATH).read_bytes()
        )
        assert _sha256(alternate) == builder.DEFAULT_MESH_SHA256
        args.extend(
            [
                "--mesh",
                str(alternate),
                "--mesh-sha256",
                builder.DEFAULT_MESH_SHA256,
            ]
        )
    else:
        alternate = tmp_path / "alternate_profile.json"
        alternate.write_bytes(
            (builder.REPO_ROOT / builder.DEFAULT_BASE_PROFILE_PATH).read_bytes()
        )
        assert _sha256(alternate) == builder.DEFAULT_BASE_PROFILE_SHA256
        args.extend(
            [
                "--base-profile",
                str(alternate),
                "--base-profile-sha256",
                builder.DEFAULT_BASE_PROFILE_SHA256,
            ]
        )

    with pytest.raises(builder.SourceIdentityError, match="path is not frozen"):
        builder.main(args)
    assert not output.exists()


def test_cli_rejects_external_output_before_creating_parent(tmp_path: Path) -> None:
    parent = tmp_path / "must_not_be_created"
    output = parent / "candidate.json"
    assert not parent.exists()

    with pytest.raises(builder.OutputPathError, match="must be inside"):
        builder.main(_cli_args(output))
    assert not parent.exists()


def test_cli_rejects_repo_output_outside_allowed_directory(
    tmp_path: Path,
) -> None:
    parent = (
        builder.REPO_ROOT
        / "validation"
        / f"outside_primary_profiles_{tmp_path.name}"
    )
    output = parent / "candidate.json"
    assert not parent.exists()

    with pytest.raises(builder.OutputPathError, match="must be inside"):
        builder.main(_cli_args(output))
    assert not parent.exists()


def test_cli_output_validation_is_noncreating(tmp_path: Path) -> None:
    parent = (
        builder.REPO_ROOT
        / builder.EXPERIMENTAL_OUTPUT_DIRECTORY
        / f"valid_noncreating_{tmp_path.name}"
    )
    output = parent / "candidate.json"
    assert not parent.exists()
    assert builder.validate_cli_output_path(output) == output.resolve()
    assert not parent.exists()


@pytest.mark.parametrize("speed", [-0.1, math.nan, math.inf])
def test_surface_speed_must_be_finite_and_nonnegative(
    speed: float,
) -> None:
    with pytest.raises(builder.MeshSoleError):
        builder.build_candidate_profile(
            candidate_id="G00_mesh_r20_n8_go0_ff0",
            plateau_speed_mps=speed,
            **_canonical_sources(),
        )


@pytest.mark.parametrize("radius", [0.0, -0.01, math.nan, math.inf])
def test_radius_must_be_finite_and_positive(radius: float) -> None:
    with pytest.raises(builder.MeshSoleError):
        builder.generate_mesh_spheres(
            _box_triangles(),
            radius_m=radius,
            longitudinal_count=8,
            forefoot_offset_m=0.0,
        )


def test_narrow_section_requires_positive_overlap_not_full_containment() -> None:
    narrow = _box_triangles(z_min=-0.015, z_max=0.015)
    spheres, contracts, _ = builder.generate_mesh_spheres(
        narrow,
        radius_m=0.020,
        longitudinal_count=8,
        forefoot_offset_m=0.0,
    )

    assert len(spheres) == 16
    assert any(
        sphere["location"][2] - sphere["radius"]
        < contract["section_z_min_m"]
        or sphere["location"][2] + sphere["radius"]
        > contract["section_z_max_m"]
        for sphere, contract in zip(spheres, contracts)
    )
    for sphere, contract in zip(spheres, contracts):
        z_value = sphere["location"][2]
        radius = sphere["radius"]
        assert contract["section_z_min_m"] <= z_value <= contract["section_z_max_m"]
        assert min(
            z_value + radius,
            contract["section_z_max_m"],
        ) - max(
            z_value - radius,
            contract["section_z_min_m"],
        ) > 0.0


def test_rejects_center_outside_section_even_if_footprint_overlaps() -> None:
    sphere = {
        "name": "outside_center",
        "location": [0.0, 0.0, 0.51],
        "radius": 0.02,
    }
    contract = {"section_z_min_m": -0.5, "section_z_max_m": 0.5}
    with pytest.raises(builder.GeometryError, match="center is outside"):
        builder._validate_spheres(
            [sphere],
            [contract],
            np.asarray([-1.0, -0.1, -0.5]),
            np.asarray([1.0, 0.1, 0.5]),
        )


def test_duplicate_names_and_locations_are_rejected() -> None:
    sphere = {
        "name": "duplicate",
        "location": [0.0, 0.0, 0.0],
        "radius": 0.1,
    }
    contract = {"section_z_min_m": -0.5, "section_z_max_m": 0.5}
    with pytest.raises(builder.GeometryError, match="duplicate"):
        builder._validate_spheres(
            [sphere, dict(sphere)],
            [contract, contract],
            np.asarray([-1.0, -0.1, -0.5]),
            np.asarray([1.0, 0.1, 0.5]),
        )


def test_nonfinite_ascii_vertex_is_rejected(tmp_path: Path) -> None:
    mesh = tmp_path / "bad.STL"
    mesh.write_text(
        """solid bad
facet normal 0 1 0
outer loop
vertex 0 0 0
vertex 1 0 0
vertex nan 1 0
endloop
endfacet
endsolid bad
""",
        encoding="utf-8",
    )
    with pytest.raises(builder.MeshFormatError, match="non-finite"):
        builder.load_stl_triangles(mesh)


def test_no_clobber_is_race_safe(tmp_path: Path) -> None:
    output = tmp_path / "profiles" / "candidate.json"
    first = builder.write_json_no_clobber(output, {"status": "first"})
    before = first.read_bytes()
    with pytest.raises(builder.NoClobberError, match="already exists"):
        builder.write_json_no_clobber(output, {"status": "second"})
    assert first.read_bytes() == before
    assert json.loads(first.read_text(encoding="utf-8")) == {"status": "first"}


def test_default_mesh_uses_git_canonical_case() -> None:
    assert builder.DEFAULT_MESH_PATH.as_posix() == "Geometry/AM_foot_l.STL"


def test_every_preregistered_candidate_is_finite_and_single_axis() -> None:
    assert len(builder.CANDIDATE_SPECS) == 10
    assert len(set(builder.CANDIDATE_SPECS)) == 10
    for candidate_id, spec in builder.CANDIDATE_SPECS.items():
        assert candidate_id == spec.candidate_id
        assert math.isfinite(spec.ground_origin_delta_m)
        assert math.isfinite(spec.radius_m) and spec.radius_m > 0.0
        assert spec.longitudinal_count in {8, 10}
        assert math.isfinite(spec.forefoot_offset_m)
