"""Native, pose-only regressions for benchmark-relative SEA foot registration.

Run with an OpenSim Python environment, e.g.:
    python -B -m unittest validation.test.test_ground_registration -v

Only temporary test outputs are written. No IK solver, integration, pipeline,
policy, controller or operational model is run or changed.
"""
from __future__ import annotations

from copy import deepcopy
from contextlib import redirect_stderr
import hashlib
import io
import json
import math
from pathlib import Path
import struct
import sys
import tempfile
import unittest
from unittest.mock import patch
from xml.etree import ElementTree as ET

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

import workspace_bootstrap

workspace_bootstrap.ensure_simulator_paths()

from tools.model_preparation_pipeline.run_opensim_sea_pipeline import default_plugin
from tools.model_preparation_pipeline import build_abx_seasea_pipeline as builder
from tools.model_preparation_pipeline import register_seasea_ground as registration
from path_resolver import BINARY_DETECTOR_PROFILE_PATH

DONOR = REPO_ROOT / "models/AB06/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi.osim"
SUBJECT = REPO_ROOT / "models/AB07/AB07_SEASEA_Threadmill/AB07_SEASEA_stiff321_500_pi.osim"
DONOR_STATIC = REPO_ROOT / "models/AB06/AB06_SEASEA-raw/data/converted/static/static_01/static_01_ik_dataset_ab06_seasea.mot"
SUBJECT_STATIC = REPO_ROOT / "models/AB07/AB07_SEASEA-raw/data/converted/static/static01/static01_ik_dataset_ab06_seasea.mot"
MESH = REPO_ROOT / "models/Geometry/AM_foot_l.STL"
PLANE = BINARY_DETECTOR_PROFILE_PATH
TRANSLATIONS = {"pelvis_tx", "pelvis_ty", "pelvis_tz"}


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_motion(path: Path, motion) -> None:
    """Independent motion serialization, including explicit angular units."""
    with path.open("w", encoding="utf-8") as stream:
        stream.write(f"test_static\ninDegrees={'yes' if motion.in_degrees else 'no'}\nendheader\n")
        stream.write("\t".join(motion.labels) + "\n")
        np.savetxt(stream, motion.values, fmt="%.17g", delimiter="\t")


def independent_vertices(path: Path) -> np.ndarray:
    """Read the triangle vertices with struct, independently of the helper."""
    raw = path.read_bytes()
    count = struct.unpack_from("<I", raw, 80)[0]
    if len(raw) != 84 + 50 * count:
        raise AssertionError("Unexpected binary STL length")
    vertices = []
    for index in range(count):
        vertices.extend(np.asarray(struct.unpack_from("<9f", raw, 84 + 50 * index + 12)).reshape(3, 3))
    return np.unique(np.asarray(vertices, dtype=float), axis=0)


def independent_geometry(model_path: Path, motion, vertices, origin, normal) -> tuple[float, float]:
    """Evaluate a newly loaded model through body stations, never helper FK.

    This detects an algebraic receipt that does not describe the serialized
    mount, wrong joint parent/frame propagation, or angular-unit mistakes.
    """
    osim = registration.osim
    model = osim.Model(str(model_path))
    state = model.initSystem()
    coordinates = model.getCoordinateSet()
    body = model.getBodySet().get("foot_l")
    stations = [osim.Vec3(0, 0, 0), osim.Vec3(1, 0, 0), osim.Vec3(0, 1, 0), osim.Vec3(0, 0, 1)]
    heights, pitches = [], []
    for row in motion.values:
        state.setTime(float(row[0]))
        for name, value in zip(motion.labels[1:], row[1:]):
            angle = math.radians(float(value)) if motion.in_degrees and name not in TRANSLATIONS else float(value)
            coordinates.get(name).setValue(state, angle, False)
        model.realizePosition(state)
        points = [np.array([p.get(i) for i in range(3)]) for p in
                  (body.findStationLocationInGround(state, station) for station in stations)]
        rotation = np.column_stack([point - points[0] for point in points[1:]])
        world_vertices = vertices @ rotation.T + points[0]
        heights.append(float(np.min((world_vertices - origin) @ normal)))
        pitches.append(math.atan2(float(normal @ rotation[:, 0]), float(normal @ rotation[:, 1])))
    return float(np.median(heights)), float(np.median(np.unwrap(pitches)))


def mount_element(tree: ET.ElementTree) -> ET.Element:
    joint = tree.find(".//JointSet/objects/PinJoint[@name='pros_ankle']")
    frame = joint.find(f"frames/PhysicalOffsetFrame[@name='{joint.findtext('socket_parent_frame')}']")
    return frame.find("translation")


class HeightSolveTests(unittest.TestCase):
    def test_identity_and_solver_do_not_mutate_inputs(self):
        heights, slopes = np.array([0.01, 0.02, 0.03]), np.array([0.95, 1.0, 0.98])
        old_h, old_s = heights.copy(), slopes.copy()
        self.assertEqual(registration._solve_height_shift(heights, slopes, 0.02, -0.3), 0.0)
        np.testing.assert_array_equal(heights, old_h)
        np.testing.assert_array_equal(slopes, old_s)

    def test_median_crossing_has_known_root_in_both_directions(self):
        # The row giving the median changes when the mount is shifted. Solving
        # median(h) + delta * median(slope) would fail this case.
        heights = np.array([0.0, 0.0001, 0.00011])
        slopes = np.array([1.0, 0.97, 0.99])
        for expected in (0.02, -0.02):
            with self.subTest(delta=expected):
                target = float(np.median(heights + slopes * expected))
                actual = registration._solve_height_shift(heights, slopes, target, -0.3)
                self.assertAlmostEqual(actual, expected, places=12)
                self.assertLess(abs(float(np.median(heights + slopes * actual)) - target), 1e-12)

    def test_invalid_sensitivity_and_input_fail_without_mutation(self):
        cases = [([], [], 0, -0.3), ([0], [0], 1, -0.3), ([0, 1], [1, -1], 0, -0.3),
                 ([0], [1e-11], 1, -0.3), ([0, 1], [1], 0, -0.3),
                 ([float("nan")], [1], 0, -0.3), ([0], [1], float("inf"), -0.3),
                 ([0], [1], 0, 0), ([0], [1], 0, 0.3)]
        for h, s, target, initial in cases:
            with self.subTest(h=h, s=s, target=target, initial=initial):
                heights, slopes = np.array(h), np.array(s)
                original_h, original_s = heights.copy(), slopes.copy()
                with self.assertRaises(ValueError):
                    registration._solve_height_shift(heights, slopes, target, initial)
                np.testing.assert_array_equal(heights, original_h)
                np.testing.assert_array_equal(slopes, original_s)

    def test_target_cannot_collapse_or_invert_mount(self):
        for target in (0.3, 0.31):
            with self.subTest(target=target), self.assertRaisesRegex(ValueError, "invert or collapse"):
                registration._solve_height_shift(np.array([0.0]), np.array([1.0]), target, -0.3)


class GroundRegistrationCliTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="ground_cli_test_")
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)

    def builder_argv(self) -> list[str]:
        return ["build_abx_seasea_pipeline.py", "--subject", "AB07",
                "--fit-reference-model", str(self.root / "healthy.osim"),
                "--ground-reference-motion", str(self.root / "static.mot"),
                "--ground-plane-profile", str(self.root / "plane.json"),
                "--foot-mesh", str(self.root / "foot.STL")]

    def test_builder_rejects_ground_registration_with_skipped_marker_ik(self):
        errors = io.StringIO()
        with patch.object(sys, "argv", self.builder_argv() + ["--skip-opensim-ik"]), redirect_stderr(errors):
            with self.assertRaises(SystemExit) as failure:
                builder.parse_args()
        self.assertEqual(failure.exception.code, 2)
        self.assertIn("requires a new marker IK", errors.getvalue())

    def test_validate_only_rejects_missing_registration_inputs_before_execution(self):
        inputs = {"healthy.osim": "Healthy fitting reference model", "static.mot": "Donor static registration motion",
                  "plane.json": "Shared ground registration profile", "foot.STL": "Fixed foot hardware STL"}
        for missing, label in inputs.items():
            with self.subTest(missing=missing):
                for name in inputs:
                    path = self.root / name
                    if name == missing:
                        if path.exists():
                            path.unlink()
                    else:
                        path.write_bytes(b"preflight existence fixture; never parsed")
                before = {p: digest(p) for p in self.root.iterdir()}
                with patch.object(sys, "argv", self.builder_argv() + ["--validate-only"]), \
                     patch.object(builder, "discover_subject", return_value=object()), \
                     patch.object(builder, "validate_discovery", side_effect=AssertionError("invalid preflight reached discovery validation")), \
                     patch.object(builder, "require_python_opensim", side_effect=AssertionError("unexpected Python execution")), \
                     patch.object(builder, "run_logged", side_effect=AssertionError("unexpected subprocess execution")), \
                     patch.object(builder, "build_subject", side_effect=AssertionError("unexpected pipeline execution")):
                    with self.assertRaisesRegex(FileNotFoundError, label):
                        builder.main()
                self.assertEqual({p: digest(p) for p in self.root.iterdir()}, before)

    def test_helper_output_plugin_collision_fails_without_changing_bytes(self):
        plugin = self.root / "plugin.library"
        plugin.write_bytes(b"frozen plugin bytes; must never be overwritten")
        original = plugin.read_bytes()
        for collision in ("output-model", "output-static-motion", "report"):
            with self.subTest(output=collision):
                argv = ["register_seasea_ground.py", "--force", "--plugin", str(plugin)]
                for flag in ("model", "static-motion", "reference-model", "reference-motion", "foot-mesh", "plane-profile",
                             "output-model", "output-static-motion", "report"):
                    argv.extend(["--" + flag, str(plugin if flag == collision else self.root / flag)])
                with patch.object(sys, "argv", argv), \
                     patch.object(registration, "compute_registration", side_effect=AssertionError("collision reached registration")):
                    with self.assertRaisesRegex(ValueError, "distinct from each other and all inputs"):
                        registration.main()
                self.assertEqual(plugin.read_bytes(), original)
                self.assertEqual(list(self.root.iterdir()), [plugin])


class NativeGroundRegistrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.plugin = default_plugin()
        for source in (DONOR, SUBJECT, DONOR_STATIC, SUBJECT_STATIC, MESH, PLANE, cls.plugin):
            if not source.is_file():
                raise unittest.SkipTest(f"Native regression fixture missing: {source}")
        registration.osim.Logger.setLevelString("error")
        cls.inputs = (DONOR, SUBJECT, DONOR_STATIC, SUBJECT_STATIC, MESH, PLANE, cls.plugin)
        cls.input_hashes = {path: digest(path) for path in cls.inputs}
        cls.temporary = tempfile.TemporaryDirectory(prefix="ground_registration_test_")
        cls.root = Path(cls.temporary.name)
        cls.addClassCleanup(cls.temporary.cleanup)
        cls.subject_original = registration.parse_motion(SUBJECT_STATIC)
        cls.donor_original = registration.parse_motion(DONOR_STATIC)
        cls.tree, cls.motion, cls.receipt = cls.compute()
        cls.registered_model = cls.root / "registered.osim"
        cls.registered_motion = cls.root / "registered_static.mot"
        cls.tree.write(cls.registered_model, encoding="utf-8", xml_declaration=True)
        write_motion(cls.registered_motion, cls.motion)
        profile = json.loads(PLANE.read_text())
        cls.origin = np.asarray(profile["ground"]["origin"], dtype=float)
        cls.normal = np.asarray(profile["ground"]["normal"], dtype=float)
        cls.vertices = independent_vertices(MESH)

    @classmethod
    def compute(cls, **overrides):
        args = dict(model_path=SUBJECT, static_motion=SUBJECT_STATIC, reference_model=DONOR,
                    reference_motion=DONOR_STATIC, foot_mesh=MESH, plane_profile=PLANE, plugin=cls.plugin)
        args.update(overrides)
        return registration.compute_registration(**args)

    def test_ab06_donor_identity_exact_xml_and_motion(self):
        tree, motion, receipt = self.compute(model_path=DONOR, static_motion=DONOR_STATIC)
        self.assertEqual(ET.tostring(tree.getroot()), ET.tostring(ET.parse(DONOR).getroot()))
        self.assertTrue(all(value == 0 for value in receipt["adjustments"].values()))
        np.testing.assert_array_equal(motion.values, self.donor_original.values)
        np.testing.assert_array_equal(motion.times, self.donor_original.times)
        self.assertEqual(motion.labels, self.donor_original.labels)
        self.assertEqual(motion.in_degrees, self.donor_original.in_degrees)

    def test_serialized_ab07_native_geometry_matches_donor_target(self):
        ref_h, ref_pitch = independent_geometry(DONOR, self.donor_original, self.vertices, self.origin, self.normal)
        out_h, out_pitch = independent_geometry(self.registered_model, self.motion, self.vertices, self.origin, self.normal)
        self.assertLess(abs(out_h - ref_h), 1e-10)
        self.assertLess(abs(out_pitch - ref_pitch), 1e-10)
        self.assertLess(abs(ref_h - self.receipt["reference"]["median_min_stl_clearance_m"]), 1e-10)
        self.assertLess(abs(ref_pitch - self.receipt["reference"]["median_sagittal_pitch_rad"]), 1e-10)
        self.assertLess(abs(out_h - self.receipt["subject_after"]["median_min_stl_clearance_m"]), 1e-10)
        self.assertLess(abs(out_pitch - self.receipt["subject_after"]["median_sagittal_pitch_rad"]), 1e-10)

    def test_entire_model_xml_preserved_except_distal_mount_y(self):
        original = ET.parse(SUBJECT)
        restored = deepcopy(self.tree)
        before = mount_element(original).text.split()
        after = mount_element(restored).text.split()
        self.assertEqual(after[::2], before[::2])
        mount_element(restored).text = mount_element(original).text
        self.assertEqual(ET.tostring(restored.getroot()), ET.tostring(original.getroot()))
        self.assertLess(float(after[1]), -1e-12)
        self.assertEqual(ET.tostring(self.tree.find(".//MarkerSet")), ET.tostring(original.find(".//MarkerSet")))

    def test_motion_changes_only_constant_static_ankle_offset(self):
        column = self.motion.labels.index("pros_ankle_angle")
        other_columns = [i for i in range(len(self.motion.labels)) if i != column]
        np.testing.assert_array_equal(self.motion.values[:, other_columns], self.subject_original.values[:, other_columns])
        np.testing.assert_array_equal(self.motion.times, self.subject_original.times)
        expected = self.receipt["adjustments"]["static_ankle_pose_offset_deg"]
        np.testing.assert_allclose(self.motion.values[:, column] - self.subject_original.values[:, column], expected, rtol=0, atol=1e-13)
        self.assertEqual(self.motion.labels, self.subject_original.labels)
        self.assertEqual(self.motion.in_degrees, self.subject_original.in_degrees)

    def test_registration_is_idempotent_after_serialization(self):
        tree, motion, receipt = self.compute(model_path=self.registered_model, static_motion=self.registered_motion)
        self.assertTrue(all(value == 0 for value in receipt["adjustments"].values()), receipt["adjustments"])
        self.assertEqual(ET.tostring(tree.getroot()), ET.tostring(ET.parse(self.registered_model).getroot()))
        np.testing.assert_array_equal(motion.values, self.motion.values)

    def test_radians_and_degrees_produce_same_hardware_registration(self):
        values = self.subject_original.values.copy()
        columns = [i for i, name in enumerate(self.subject_original.labels) if i and name not in TRANSLATIONS]
        values[:, columns] = np.deg2rad(values[:, columns])
        radians = registration.MotionData(values[:, 0].copy(), list(self.subject_original.labels), values, False)
        path = self.root / "radians_static.mot"
        write_motion(path, radians)
        tree, motion, receipt = self.compute(static_motion=path)
        self.assertLess(abs(receipt["adjustments"]["distal_mount_delta_y_m"] - self.receipt["adjustments"]["distal_mount_delta_y_m"]), 1e-12)
        self.assertLess(abs(receipt["adjustments"]["static_ankle_pose_offset_rad"] - self.receipt["adjustments"]["static_ankle_pose_offset_rad"]), 1e-12)
        np.testing.assert_allclose(motion.values[:, columns], np.deg2rad(self.motion.values[:, columns]), rtol=0, atol=1e-14)
        self.assertFalse(motion.in_degrees)

    def test_invalid_normal_fails_before_native_model_creation(self):
        profile = json.loads(PLANE.read_text())
        profile["ground"]["normal"] = [0, 2, 0]
        path = self.root / "invalid_normal.json"
        path.write_text(json.dumps(profile), encoding="utf-8")
        before = digest(path)
        with patch.object(registration.osim, "Model", side_effect=AssertionError("unexpected native model creation")):
            with self.assertRaisesRegex(ValueError, "unit length"):
                self.compute(plane_profile=path)
        self.assertEqual(digest(path), before)

    def test_unsupported_mount_fails_before_native_model_creation(self):
        tree = ET.parse(SUBJECT)
        mount_element(tree).text = "0 0.1 0"
        path = self.root / "invalid_mount.osim"
        tree.write(path, encoding="utf-8", xml_declaration=True)
        before = digest(path)
        with patch.object(registration.osim, "Model", side_effect=AssertionError("unexpected native model creation")):
            with self.assertRaisesRegex(ValueError, "distal pylon mount"):
                self.compute(model_path=path)
        self.assertEqual(digest(path), before)

    def test_unchanged_ankle_range_rejects_invalid_static_pose(self):
        values = self.subject_original.values.copy()
        values[:, self.subject_original.labels.index("pros_ankle_angle")] = 360.0
        path = self.root / "out_of_range_static.mot"
        write_motion(path, registration.MotionData(values[:, 0].copy(), list(self.subject_original.labels), values, True))
        before = digest(path)
        with self.assertRaisesRegex(ValueError, "unchanged joint range"):
            self.compute(static_motion=path)
        self.assertEqual(digest(path), before)

    def test_source_models_static_data_mesh_profiles_and_plugin_unchanged(self):
        self.assertEqual({path: digest(path) for path in self.inputs}, self.input_hashes)
        self.assertTrue(self.receipt["checks"]["source_files_unchanged"])
        self.assertTrue(self.receipt["checks"]["protected_model_xml_unchanged"])


if __name__ == "__main__":
    unittest.main()
