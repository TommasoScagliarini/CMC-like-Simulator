"""Contract and geometry tests for the opt-in V4 detector profile."""

from __future__ import annotations

import hashlib
import json
import unittest
from copy import deepcopy
from pathlib import Path

import numpy as np

from online_grf import load_online_grf_profile, online_grf_sensor_role
from path_resolver import resolve_repo_path
from setup_io import read_setup_xml
from validation.audit_two_sensor_prescribed_geometry import (
    _load_stl_triangles,
    _resolve_left_foot_mesh,
    _static_sensor_geometry,
)
from validation.build_two_sensor_mesh_profile_v4 import (
    DEFAULT_BASE_PROFILE,
    DEFAULT_OUTPUT,
    DEFAULT_SETUP,
    DESIGN_ID,
    _section_z_bounds_at_x,
    _vertical_surface_intersections_y,
    build_candidate_profile,
)
from validation.validate_two_sensor_prescribed_replay import _left_sensor_spheres


REPO_ROOT = Path(__file__).resolve().parents[1]
TRAINING_CONFIG = REPO_ROOT / "Trajectory Generator/baseline_MLP/training_exnovo_cfg.yaml"
FSM_SOURCE = REPO_ROOT / "Trajectory Generator/prosthetic_phase_fsm.py"
FROZEN_HASHES = {
    resolve_repo_path(DEFAULT_BASE_PROFILE): (
        "61ea948a3c0613e5c0e684a3197de118c7116e36188fca6993da79ce713fd99e"
    ),
    TRAINING_CONFIG: (
        "847da69896fec1ae2a8da34b56369fdd7edf2b6b6bbdb1a463b30cd2d830ee13"
    ),
    FSM_SOURCE: (
        "ec726fdf392f099ac1135de9e736e984ada695c9704102621d0cd8af2fdc5b45"
    ),
}


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


class TwoSensorMeshProfileV4Test(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.base_path = resolve_repo_path(DEFAULT_BASE_PROFILE)
        cls.output_path = resolve_repo_path(DEFAULT_OUTPUT)
        cls.setup_path = resolve_repo_path(DEFAULT_SETUP)
        cls.base = load_online_grf_profile(cls.base_path)
        cls.checked = load_online_grf_profile(cls.output_path)
        cls.built, cls.geometry = build_candidate_profile(
            cls.base_path,
            cls.setup_path,
        )
        setup = read_setup_xml(cls.setup_path)
        cls.mesh_path = _resolve_left_foot_mesh(setup.model_file)
        cls.triangles = _load_stl_triangles(cls.mesh_path)

    def test_checked_profile_exactly_matches_deterministic_builder(self) -> None:
        self.assertEqual(self.checked.to_dict(), self.built.to_dict())
        design = self.checked.metadata["v4_mesh_design"]
        self.assertEqual(design["design_id"], DESIGN_ID)
        self.assertFalse(design["design_uses_prescribed_timing_or_force_data"])

    def test_only_left_toe_location_changes_runtime_semantics(self) -> None:
        base_payload = self.base.to_dict()
        candidate_payload = self.checked.to_dict()
        self.assertNotEqual(base_payload["source"], candidate_payload["source"])
        self.assertNotEqual(base_payload["metadata"], candidate_payload["metadata"])
        base_payload.pop("source")
        base_payload.pop("metadata")
        candidate_payload.pop("source")
        candidate_payload.pop("metadata")

        base_spheres = {item["name"]: item for item in base_payload["spheres"]}
        candidate_spheres = {
            item["name"]: item for item in candidate_payload["spheres"]
        }
        self.assertEqual(set(base_spheres), set(candidate_spheres))
        self.assertNotEqual(
            base_spheres["left_toe"]["location"],
            candidate_spheres["left_toe"]["location"],
        )
        candidate_spheres["left_toe"]["location"] = deepcopy(
            base_spheres["left_toe"]["location"]
        )
        candidate_payload["spheres"] = [
            candidate_spheres[item["name"]] for item in base_payload["spheres"]
        ]
        self.assertEqual(base_payload, candidate_payload)

    def test_geometry_rule_is_reproduced(self) -> None:
        sensors = _left_sensor_spheres(self.checked)
        heel = sensors["left_heel"]
        toe = sensors["left_toe"]
        vertices = self.triangles.reshape(-1, 3)
        mesh_max_x = float(np.max(vertices[:, 0]))
        self.assertAlmostEqual(toe.location[0] + toe.radius, mesh_max_x, places=12)

        z_min, z_max = _section_z_bounds_at_x(self.triangles, toe.location[0])
        self.assertAlmostEqual(toe.location[2], 0.5 * (z_min + z_max), places=12)
        heel_plantar = min(
            _vertical_surface_intersections_y(
                self.triangles, heel.location[0], heel.location[2]
            )
        )
        toe_plantar = min(
            _vertical_surface_intersections_y(
                self.triangles, toe.location[0], toe.location[2]
            )
        )
        heel_protrusion = heel_plantar - (heel.location[1] - heel.radius)
        toe_protrusion = toe_plantar - (toe.location[1] - toe.radius)
        self.assertAlmostEqual(heel_protrusion, toe_protrusion, places=12)

    def test_v4_passes_frozen_static_geometry_gate(self) -> None:
        result = _static_sensor_geometry(
            self.triangles,
            _left_sensor_spheres(self.checked),
        )
        self.assertTrue(result["geometry_plausible_for_semantic_decision"])
        self.assertLessEqual(
            result["sensors"]["left_toe"]["sphere_surface_gap_to_mesh_m"],
            0.005,
        )
        self.assertLessEqual(abs(result["heel_toe_bottom_offset_m"]), 0.020)

    def test_four_sensor_role_contract_remains_unambiguous(self) -> None:
        self.assertEqual(len(self.checked.spheres), 4)
        roles = {
            online_grf_sensor_role(sphere.name, sphere.side)
            for sphere in self.checked.spheres
        }
        self.assertIn("left_heel", roles)
        self.assertIn("left_toe", roles)
        sensors = _left_sensor_spheres(self.checked)
        self.assertEqual(set(sensors), {"left_heel", "left_toe"})

    def test_legacy_profile_config_and_fsm_are_bitwise_unchanged(self) -> None:
        for path, expected in FROZEN_HASHES.items():
            with self.subTest(path=path):
                self.assertEqual(_sha256(path), expected)
        config = TRAINING_CONFIG.read_text(encoding="utf-8")
        self.assertIn(
            "online_grf_detector_profile: "
            "online_grf_profiles/AB06_SEASEA_stiff321_500_pi_grf_detector_HS-TO.json",
            config,
        )
        self.assertIn("phase_fsm_input_mode: legacy_events", config)
        self.assertNotIn(self.output_path.name, config)

    def test_embedded_source_hashes_match_current_sources(self) -> None:
        design = self.checked.metadata["v4_mesh_design"]
        records = (
            design["base_profile"],
            design["setup"],
            design["model"],
            design["foot_mesh"],
        )
        for record in records:
            path = resolve_repo_path(record["path"])
            with self.subTest(path=path):
                self.assertEqual(_sha256(path), record["sha256"])

    def test_checked_json_marks_candidate_as_opt_in(self) -> None:
        payload = json.loads(self.output_path.read_text(encoding="utf-8"))
        self.assertEqual(payload["metadata"]["status"], "experimental_not_promoted")
        self.assertIn("explicit opt-in only", payload["metadata"]["note"])


if __name__ == "__main__":
    unittest.main()
