"""OpenSim-free checks of subject mounting geometry and donor preservation."""

from __future__ import annotations

from copy import deepcopy
import json
import math
from pathlib import Path
import sys
import unittest
from xml.etree import ElementTree as ET


REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "tools"))

import build_abx_seasea_pipeline as builder  # noqa: E402
from fit_seasea_subject import fit_seasea_subject  # noqa: E402


def model(path: str) -> ET.Element:
    return ET.parse(REPO / path).getroot().find("Model")


def translation(root: ET.Element, joint: str) -> list[float]:
    entry = next(j for j in root.findall("JointSet/objects/*")
                 if j.get("name") == joint)
    frame = entry.findtext("socket_parent_frame")
    return [float(v) for v in entry.findtext(
        f"frames/PhysicalOffsetFrame[@name='{frame}']/translation").split()]


class SubjectMountFittingTests(unittest.TestCase):
    def setUp(self) -> None:
        self.reference = model("models/AB06-raw/osimxml/ab06.osim")
        self.subject = model("models/AB07-raw/osimxml/ab07.osim")
        self.donor = model(
            "models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi.osim")

    def graft(self, healthy: ET.Element) -> ET.Element:
        graft = deepcopy(healthy)
        builder.graft_bodyset(graft, self.donor)
        builder.graft_jointset(graft, self.donor)
        builder.graft_forceset(graft, self.donor)
        builder.graft_markers(graft)
        return graft

    def test_ab06_fitting_is_exact_xml_identity(self) -> None:
        graft = self.graft(self.reference)
        before = ET.tostring(graft)
        manifest = fit_seasea_subject(
            graft, self.reference, self.reference, self.donor)
        self.assertEqual(ET.tostring(graft), before)
        self.assertEqual(manifest["changed_geometry"], [])
        self.assertEqual(manifest["length_ratios"], {"upper": 1.0, "shank": 1.0})
        for name in ("osseo", "pros_knee", "pros_ankle"):
            self.assertEqual(translation(graft, name), translation(self.donor, name))

    def test_ab07_fits_anatomical_ratios_and_preserves_hardware(self) -> None:
        graft = self.graft(self.subject)
        protected = {section: ET.tostring(graft.find(section))
                     for section in ("BodySet", "ForceSet", "MarkerSet")}
        ranges = [ET.tostring(c) for c in graft.findall(".//Coordinate")]
        sources = [ET.tostring(m) for m in (self.subject, self.reference, self.donor)]
        manifest = fit_seasea_subject(graft, self.subject, self.reference, self.donor)
        json.dumps(manifest, allow_nan=False)
        for section, before in protected.items():
            self.assertEqual(ET.tostring(graft.find(section)), before)
        self.assertEqual([ET.tostring(c) for c in graft.findall(".//Coordinate")], ranges)
        self.assertEqual([ET.tostring(m) for m in (self.subject, self.reference, self.donor)], sources)
        self.assertEqual({change["joint"] for change in manifest["changed_geometry"]},
                         {"osseo", "pros_ankle"})
        self.assertEqual(translation(graft, "pros_knee"), translation(self.donor, "pros_knee"))
        self.assertEqual(translation(graft, "osseo")[::2], translation(self.donor, "osseo")[::2])
        for segment in ("upper", "shank"):
            length_key = f"{segment}_center_distance"
            expected = (manifest["reference_lengths_m"][length_key]
                        * manifest["length_ratios"][segment])
            self.assertAlmostEqual(manifest["fitted_lengths_m"][length_key], expected, places=14)
        self.assertAlmostEqual(manifest["length_ratios"]["upper"],
                               0.98020719185224969 / 1.0270058634983084, places=14)
        self.assertAlmostEqual(manifest["length_ratios"]["shank"],
                               0.41774101582741596 / 0.46548059877209219, places=14)
        mass = sum(float(b.findtext("mass")) for b in graft.findall("BodySet/objects/Body"))
        healthy_mass = sum(float(b.findtext("mass")) for b in self.subject.findall("BodySet/objects/Body"))
        self.assertAlmostEqual(mass, healthy_mass, places=12)
        self.assertEqual(len(graft.findall(".//Coordinate")), 21)
        for c in graft.findall(".//Coordinate"):
            bounds = [float(v) for v in c.findtext("range").split()]
            self.assertEqual(len(bounds), 2)
            self.assertTrue(all(math.isfinite(v) for v in bounds))
            self.assertLess(bounds[0], bounds[1])

    def test_fitting_is_idempotent(self) -> None:
        graft = self.graft(self.subject)
        fit_seasea_subject(graft, self.subject, self.reference, self.donor)
        before = ET.tostring(graft)
        manifest = fit_seasea_subject(graft, self.subject, self.reference, self.donor)
        self.assertEqual(ET.tostring(graft), before)
        self.assertEqual(manifest["changed_geometry"], [])

    def test_mismatched_sea_rejected_before_mutation(self) -> None:
        graft = self.graft(self.subject)
        graft.find("ForceSet/objects/SeriesElasticActuator/Ki").text = "0"
        before = ET.tostring(graft)
        with self.assertRaisesRegex(ValueError, "SEA properties"):
            fit_seasea_subject(graft, self.subject, self.reference, self.donor)
        self.assertEqual(ET.tostring(graft), before)

    def test_unsupported_anatomical_shape_rejected_before_mutation(self) -> None:
        graft = self.graft(self.subject)
        knee = next(j for j in self.subject.findall("JointSet/objects/*")
                    if j.get("name") == "knee_l")
        knee.find("SpatialTransform/TransformAxis[@name='translation2']/MultiplierFunction/function/SimmSpline/y").text = "-0.4 -0.3"
        before = ET.tostring(graft)
        with self.assertRaisesRegex(ValueError, "Healthy knee shapes"):
            fit_seasea_subject(graft, self.subject, self.reference, self.donor)
        self.assertEqual(ET.tostring(graft), before)


if __name__ == "__main__":
    unittest.main()
