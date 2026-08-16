"""Contract tests for the V19 force-free binary point detector."""

from __future__ import annotations

import hashlib
import inspect
import json
import math
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

import binary_phase_detector as binary  # noqa: E402
from config import SimulatorConfig  # noqa: E402
from path_resolver import resolve_simulator_paths  # noqa: E402
from simulation_runner import (  # noqa: E402
    BinaryPhaseSensorTransportError,
    SimulationRunner,
)
from validation.audit_two_sensor_prescribed_geometry import (  # noqa: E402
    _load_stl_triangles,
    _minimum_mesh_distance,
)


PROFILE_PATH = (
    REPO_ROOT
    / "validation/experimental_detector_profiles/"
    "two_point_binary_v19_outsole_25mm.json"
)
MESH_PATH = REPO_ROOT / "Geometry/AM_foot_l.STL"
EXPECTED_MESH_SHA256 = (
    "fcfc4d7a90c4ccd3bedb501ec3e50d4337aa9ca6e8438b58cc6be00f47a689e9"
)


def _profile_payload() -> dict:
    return json.loads(PROFILE_PATH.read_text(encoding="utf-8"))


def _write_temporary_profile(payload: object) -> Path:
    temporary = tempfile.NamedTemporaryFile(
        mode="w",
        suffix=".json",
        encoding="utf-8",
        delete=False,
    )
    with temporary:
        json.dump(payload, temporary, allow_nan=True)
    return Path(temporary.name)


class BinaryPhaseDetectorProfileTests(unittest.TestCase):
    def test_canonical_profile_is_minimal_and_exact(self) -> None:
        profile = binary.load_binary_phase_detector_profile(PROFILE_PATH)
        self.assertEqual(profile.schema_version, 1)
        self.assertEqual(
            profile.detector_type,
            binary.BINARY_PHASE_DETECTOR_TYPE,
        )
        self.assertEqual(
            tuple(point.name for point in profile.points),
            binary.BINARY_PHASE_ROLES,
        )
        self.assertEqual(profile.ground.origin, (0.0, 0.0148208231, 0.0))
        self.assertEqual(profile.ground.normal, (0.0, 1.0, 0.0))
        forbidden = {
            "sphere",
            "spheres",
            "radius",
            "material",
            "stiffness",
            "force_threshold",
            "dwell",
        }

        def keys(value: object) -> set[str]:
            if isinstance(value, dict):
                return set(value).union(*(keys(item) for item in value.values()))
            if isinstance(value, list):
                return set().union(*(keys(item) for item in value))
            return set()

        self.assertTrue(forbidden.isdisjoint(keys(_profile_payload())))

    def test_points_are_common_25mm_outsole_reach_from_plantar_mesh(self) -> None:
        self.assertEqual(
            hashlib.sha256(MESH_PATH.read_bytes()).hexdigest(),
            EXPECTED_MESH_SHA256,
        )
        profile = binary.load_binary_phase_detector_profile(PROFILE_PATH)
        triangles = _load_stl_triangles(MESH_PATH)
        expected_surface_locations = {
            "left_heel": (
                -0.0946600475,
                -0.04490531781384512,
                0.01399567,
            ),
            "left_toe": (
                0.11574858501553537,
                -0.03903460020354058,
                0.0030479021621026177,
            ),
        }
        for point in profile.points:
            surface = expected_surface_locations[point.name]
            self.assertLessEqual(
                _minimum_mesh_distance(surface, triangles),
                1e-12,
                point.name,
            )
            self.assertAlmostEqual(point.location[0], surface[0], places=14)
            self.assertAlmostEqual(
                point.location[1],
                surface[1] - 0.025,
                places=14,
            )
            self.assertAlmostEqual(point.location[2], surface[2], places=14)
        by_role = {point.name: point for point in profile.points}
        self.assertEqual(by_role["left_heel"].location[0], -0.0946600475)
        self.assertEqual(by_role["left_heel"].location[2], 0.01399567)
        self.assertEqual(
            by_role["left_toe"].location[0],
            0.11574858501553537,
        )
        self.assertEqual(
            by_role["left_toe"].location[2],
            0.0030479021621026177,
        )

    def test_profile_parser_rejects_non_exact_or_nonfinite_schema(self) -> None:
        mutations = []
        extra = _profile_payload()
        extra["points"][0]["radius"] = 0.01
        mutations.append(extra)
        wrong_order = _profile_payload()
        wrong_order["points"].reverse()
        mutations.append(wrong_order)
        duplicate = _profile_payload()
        duplicate["points"][1]["name"] = "left_heel"
        mutations.append(duplicate)
        nonunit = _profile_payload()
        nonunit["ground"]["normal"] = [0.0, 2.0, 0.0]
        mutations.append(nonunit)
        nonfinite = _profile_payload()
        nonfinite["points"][0]["location"][0] = float("nan")
        mutations.append(nonfinite)
        float_schema = _profile_payload()
        float_schema["schema_version"] = 1.0
        mutations.append(float_schema)

        for payload in mutations:
            path = _write_temporary_profile(payload)
            try:
                with self.assertRaises(binary.BinaryPhaseDetectorProfileError):
                    binary.load_binary_phase_detector_profile(path)
            finally:
                path.unlink(missing_ok=True)

    def test_signed_clearance_uses_the_full_plane_normal(self) -> None:
        root_two = math.sqrt(2.0)
        plane = binary.BinaryGroundPlane(
            origin=(1.0, 2.0, 3.0),
            normal=(1.0 / root_two, 1.0 / root_two, 0.0),
        )
        self.assertAlmostEqual(
            binary.signed_clearance_m((2.0, 3.0, 99.0), plane),
            root_two,
        )
        self.assertAlmostEqual(
            binary.signed_clearance_m((2.0, 1.0, -99.0), plane),
            0.0,
        )

    def test_hard_switch_is_boolean_above_on_and_below_plane(self) -> None:
        self.assertIs(binary.contact_bit(1e-12), False)
        self.assertIs(binary.contact_bit(0.0), True)
        self.assertIs(binary.contact_bit(-1e-12), True)
        for bad in (float("nan"), float("inf"), True, "bad"):
            with self.assertRaises(binary.BinaryPhaseDetectorSamplingError):
                binary.contact_bit(bad)

    def test_public_reading_contains_bits_but_no_clearance_or_event(self) -> None:
        reading = binary.BinaryPhaseDetectorReading(
            time_s=1.25,
            contacts={"left_heel": True, "left_toe": False},
            signed_clearance_m={"left_heel": -0.001, "left_toe": 0.002},
        )
        sample = reading.public_sample()
        self.assertEqual(
            set(sample),
            {"time_s", "left_heel_contact", "left_toe_contact"},
        )
        self.assertIs(type(sample["left_heel_contact"]), bool)
        self.assertIs(type(sample["left_toe_contact"]), bool)
        self.assertFalse(any("event" in key for key in sample))
        self.assertFalse(any("clearance" in key for key in sample))
        json.dumps(sample, allow_nan=False)

    def test_module_has_no_force_grf_or_fsm_dependency(self) -> None:
        source = inspect.getsource(binary)
        for forbidden in (
            "addForce(",
            "read_online_grf(",
            "StreamingGaitEventDetector",
            "ProstheticPhaseFSM",
        ):
            self.assertNotIn(forbidden, source)

    def test_path_resolver_accepts_absolute_binary_profile(self) -> None:
        cfg = SimulatorConfig()
        cfg.binary_phase_detector_profile_file = str(PROFILE_PATH)
        paths = resolve_simulator_paths(cfg)
        self.assertEqual(
            paths.binary_phase_detector_profile_path,
            PROFILE_PATH,
        )


class _FakeBinarySampler:
    def sample(self, _state: object, time_s: float):
        return binary.BinaryPhaseDetectorReading(
            time_s=float(time_s),
            contacts={"left_heel": True, "left_toe": False},
            signed_clearance_m={"left_heel": -0.001, "left_toe": 0.002},
        )


def _fake_runner() -> SimulationRunner:
    runner = SimulationRunner.__new__(SimulationRunner)
    runner._binary_phase_detector_sampler = _FakeBinarySampler()
    runner._binary_phase_sensor_sampling_enabled = True
    runner._binary_phase_sensor_sample_dt_s = 0.001
    runner._binary_phase_sensor_last_sample_time_s = 4.125
    runner._last_step_info = {}
    return runner


class BinaryPhaseDetectorTransportTests(unittest.TestCase):
    def test_open_left_closed_right_transport_emits_ten_boolean_samples(self) -> None:
        runner = _fake_runner()
        samples: list[dict[str, float | bool]] = []
        runner._append_binary_phase_sensor_sample(None, 4.1255, samples)
        self.assertEqual(samples, [])
        for index in range(1, 11):
            runner._append_binary_phase_sensor_sample(
                None,
                4.125 + index * 0.001,
                samples,
            )
        runner._finalize_binary_phase_sensor_segment(
            segment_start_time_s=4.125,
            t_stop=4.135,
            samples=samples,
        )
        self.assertEqual(len(samples), 10)
        self.assertAlmostEqual(float(samples[0]["time_s"]), 4.126)
        self.assertAlmostEqual(float(samples[-1]["time_s"]), 4.135)
        for sample in samples:
            self.assertIs(type(sample["left_heel_contact"]), bool)
            self.assertIs(type(sample["left_toe_contact"]), bool)
            self.assertEqual(len(sample), 3)
            json.dumps(sample, allow_nan=False)

    def test_missing_duplicate_nonmonotonic_and_nonbool_fail_closed(self) -> None:
        duplicate = _fake_runner()
        with self.assertRaisesRegex(
            BinaryPhaseSensorTransportError,
            "Duplicate",
        ):
            duplicate._append_binary_phase_sensor_sample(None, 4.125, [])

        backwards = _fake_runner()
        with self.assertRaisesRegex(
            BinaryPhaseSensorTransportError,
            "non-monotonic",
        ):
            backwards._append_binary_phase_sensor_sample(None, 4.0, [])

        missing = _fake_runner()
        with self.assertRaisesRegex(
            BinaryPhaseSensorTransportError,
            "Missing",
        ):
            missing._append_binary_phase_sensor_sample(None, 4.127, [])

        class BadReading:
            @staticmethod
            def public_sample():
                return {
                    "time_s": 4.126,
                    "left_heel_contact": 1,
                    "left_toe_contact": 0,
                }

            @staticmethod
            def diagnostics():
                return {}

        malformed = _fake_runner()
        malformed._binary_phase_detector_sampler = SimpleNamespace(
            sample=lambda _state, _time: BadReading()
        )
        with self.assertRaisesRegex(
            BinaryPhaseSensorTransportError,
            "Python bool",
        ):
            malformed._append_binary_phase_sensor_sample(None, 4.126, [])

    def test_incomplete_or_off_grid_segment_fails_closed(self) -> None:
        runner = _fake_runner()
        with self.assertRaisesRegex(
            BinaryPhaseSensorTransportError,
            "not aligned",
        ):
            runner._finalize_binary_phase_sensor_segment(
                segment_start_time_s=4.125,
                t_stop=4.1265,
                samples=[],
            )
        with self.assertRaisesRegex(
            BinaryPhaseSensorTransportError,
            "incomplete",
        ):
            runner._finalize_binary_phase_sensor_segment(
                segment_start_time_s=4.125,
                t_stop=4.126,
                samples=[],
            )


if __name__ == "__main__":
    unittest.main()
