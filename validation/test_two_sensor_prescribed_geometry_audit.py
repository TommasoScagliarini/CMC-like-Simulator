"""Pure and source-level tests for the V4 two-sensor geometry audit."""

from __future__ import annotations

import math
import struct
import tempfile
import unittest
from pathlib import Path

import numpy as np

from online_grf import OnlineGRFSphere, load_online_grf_profile
from validation.audit_two_sensor_prescribed_geometry import (
    _contact_order,
    _cop_fraction,
    _derive_critical_hs,
    _first_above_time,
    _first_pulse_duration,
    _left_sensor_spheres,
    _load_stl_triangles,
    _minimum_mesh_distance,
    _resolve_left_foot_mesh,
    _static_sensor_geometry,
    _threshold_run_near_reference,
    _validate_unsealed_window,
)
from validation.validate_two_sensor_prescribed_replay import (
    DEFAULT_PROFILE,
    DEFAULT_SETUP,
)
from path_resolver import resolve_repo_path
from setup_io import read_setup_xml


class TwoSensorPrescribedGeometryAuditTest(unittest.TestCase):
    def test_unsealed_window_is_hard_limited_to_50_100(self) -> None:
        _validate_unsealed_window(50.0, 100.0)
        with self.assertRaisesRegex(ValueError, "remains sealed"):
            _validate_unsealed_window(50.0, 100.001)
        with self.assertRaises(ValueError):
            _validate_unsealed_window(49.999, 99.0)
        with self.assertRaises(ValueError):
            _validate_unsealed_window(60.0, 60.0)

    def test_binary_stl_and_point_triangle_distance(self) -> None:
        normal = (0.0, 1.0, 0.0)
        vertices = ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, 1.0))
        payload = (
            b"synthetic".ljust(80, b"\0")
            + struct.pack("<I", 1)
            + struct.pack("<12fH", *(normal + sum(vertices, ())), 0)
        )
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "triangle.stl"
            path.write_bytes(payload)
            triangles = _load_stl_triangles(path)
        self.assertEqual(triangles.shape, (1, 3, 3))
        self.assertAlmostEqual(
            _minimum_mesh_distance((0.25, 0.30, 0.25), triangles),
            0.30,
            places=12,
        )

    def test_static_geometry_flags_a_detached_toe_sphere(self) -> None:
        triangles = np.asarray(
            [[[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]],
            dtype=float,
        )
        sensors = {
            "left_heel": OnlineGRFSphere(
                name="left_heel",
                side="left",
                frame="/bodyset/foot_l",
                location=(0.1, 0.005, 0.1),
                radius=0.010,
            ),
            "left_toe": OnlineGRFSphere(
                name="left_toe",
                side="left",
                frame="/bodyset/foot_l",
                location=(0.2, -0.050, 0.2),
                radius=0.010,
            ),
        }
        result = _static_sensor_geometry(triangles, sensors)
        self.assertFalse(result["geometry_plausible_for_semantic_decision"])
        self.assertAlmostEqual(
            result["sensors"]["left_toe"]["sphere_surface_gap_to_mesh_m"],
            0.040,
        )

    def test_current_source_geometry_reproduces_detached_toe_finding(self) -> None:
        setup = read_setup_xml(resolve_repo_path(DEFAULT_SETUP))
        mesh_path = _resolve_left_foot_mesh(setup.model_file)
        triangles = _load_stl_triangles(mesh_path)
        profile = load_online_grf_profile(resolve_repo_path(DEFAULT_PROFILE))
        result = _static_sensor_geometry(
            triangles,
            _left_sensor_spheres(profile),
        )
        toe = result["sensors"]["left_toe"]
        self.assertGreater(toe["sphere_surface_gap_to_mesh_m"], 0.005)
        self.assertGreater(abs(result["heel_toe_bottom_offset_m"]), 0.020)
        self.assertFalse(result["geometry_plausible_for_semantic_decision"])

    def test_cop_fraction_is_translation_and_yaw_invariant(self) -> None:
        heel = np.asarray([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]])
        toe = np.asarray([[1.0, 0.0, 0.0], [1.0, 0.0, 0.0]])
        cop = np.asarray([[0.25, 0.0, 0.0], [0.75, 0.0, 0.0]])
        normal = np.asarray([0.0, 1.0, 0.0])
        expected = np.asarray([0.25, 0.75])
        np.testing.assert_allclose(_cop_fraction(cop, heel, toe, normal), expected)

        angle = math.radians(37.0)
        rotation = np.asarray(
            [
                [math.cos(angle), 0.0, math.sin(angle)],
                [0.0, 1.0, 0.0],
                [-math.sin(angle), 0.0, math.cos(angle)],
            ]
        )
        translation = np.asarray([3.0, 2.0, -4.0])
        transformed = [values @ rotation.T + translation for values in (cop, heel, toe)]
        np.testing.assert_allclose(
            _cop_fraction(*transformed, normal),
            expected,
            atol=1e-12,
        )

    def test_threshold_onset_duration_and_contact_order(self) -> None:
        times = np.arange(0.0, 0.101, 0.001)
        values = np.zeros_like(times)
        values[(times >= 0.020) & (times < 0.050)] = 1.0
        onset = _first_above_time(
            times,
            values,
            reference_time_s=0.030,
            threshold=0.5,
        )
        self.assertAlmostEqual(onset, 0.020)
        self.assertAlmostEqual(
            _first_pulse_duration(
                times,
                values,
                start_time_s=onset,
                threshold=0.5,
            ),
            0.030,
        )
        self.assertEqual(_contact_order(0.020, 0.021, 0.001), "simultaneous")
        self.assertEqual(_contact_order(0.020, 0.030, 0.001), "heel_first")
        self.assertEqual(_contact_order(0.040, 0.030, 0.001), "toe_first")
        self.assertEqual(_contact_order(None, 0.030, 0.001), "toe_only")

    def test_ideal_dwell_and_runtime_grid_are_distinct(self) -> None:
        times = np.arange(0.0, 0.101, 0.001)
        cases = (
            (0.029, False, False),
            (0.030, True, False),
            (0.031, True, True),
        )
        for duration, ideal, runtime in cases:
            with self.subTest(duration=duration):
                values = np.zeros_like(times)
                values[(times >= 0.0) & (times < duration)] = 1.0
                result = _threshold_run_near_reference(
                    times,
                    values,
                    reference_time_s=0.0,
                    threshold=0.5,
                )
                self.assertEqual(result["ideal_dwell_confirmable"], ideal)
                self.assertEqual(result["runtime_10ms_confirmable"], runtime)

    def test_selects_four_late_hs_and_first_missing_root(self) -> None:
        reference = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0]
        predicted = [1.0, 2.1, 3.0, 4.2, 5.1, 6.2]
        manifest = {
            "primary_candidate_details": {
                "on00p50_off00p25": {
                    "events": {
                        "reference": {"heel_strike": reference},
                        "primary": {"heel_strike": predicted},
                    }
                }
            }
        }
        critical = _derive_critical_hs(manifest)
        self.assertEqual([item["reference_index"] for item in critical], [1, 3, 4, 5, 6])
        self.assertEqual(critical[-1]["kind"], "first_missing_before_timeout_cascade")


if __name__ == "__main__":
    unittest.main()
