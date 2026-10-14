"""Native pose-only ankle geometry regression; never integrate or build an env."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import opensim


REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(REPO / "Trajectory Generator"))
from osim_trj_cmc_like import CMCLikeProsthesisTrajectoryEnv


def array(vec):
    return np.array([vec.get(i) for i in range(3)], dtype=float)


class AnkleGeometryPoseTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        # OpenSim adds the platform prefix/extension itself (as the loader does).
        plugin = REPO / "plugins/SEA_Plugin_BlackBox_mCMC_impedence_ff"
        if opensim.LoadOpenSimLibrary(str(plugin)) is False:
            raise RuntimeError(f"Unable to load SEA plugin: {plugin}")

    def evaluate(self, subject: str, values: dict[str, float]):
        model = opensim.Model(str(REPO / f"models/{subject}_SEASEA_Threadmill/{subject}_SEASEA_stiff321_500_pi.osim"))
        state = model.initSystem()
        for name, value in values.items():
            model.getCoordinateSet().get(name).setValue(state, value, False)
        model.realizePosition(state)
        proxy = SimpleNamespace(ctx=SimpleNamespace(model=model), runner=SimpleNamespace(state=state))
        geometry = CMCLikeProsthesisTrajectoryEnv._prosthetic_ankle_center_axis_ground(proxy)
        self.assertIsNotNone(geometry)
        tibia = model.getBodySet().get("tibia_pylon")
        return model, state, tibia, geometry

    def test_ab06_identity_center_axis_and_projected_wrench_in_three_poses(self) -> None:
        poses = ({}, {"pelvis_tilt": 0.12, "hip_flexion_l": 0.25, "pros_knee_angle": -0.4},
                 {"pelvis_list": -0.08, "pelvis_rotation": 0.2, "hip_adduction_l": 0.05, "pros_ankle_angle": 0.16})
        for pose in poses:
            _, state, tibia, (center, axis) = self.evaluate("AB06", pose)
            old_center = array(tibia.findStationLocationInGround(state, opensim.Vec3(0, -0.43, 0)))
            old_tip = array(tibia.findStationLocationInGround(state, opensim.Vec3(0, -0.43, 1)))
            old_axis = old_tip - old_center
            old_axis /= np.linalg.norm(old_axis)
            np.testing.assert_allclose(center, old_center, atol=1e-12, rtol=0)
            np.testing.assert_allclose(axis, old_axis, atol=1e-12, rtol=0)
            force, moment = np.array([40., 600., 5.]), np.array([3., 10., 5.])
            old_tau = old_axis @ (moment - np.cross(old_center, force))
            actual_tau = axis @ (moment - np.cross(center, force))
            self.assertLessEqual(abs(actual_tau - old_tau), 1e-12)

    def test_ab07_center_uses_fitted_mount_instead_of_donor_station(self) -> None:
        model, state, tibia, (center, axis) = self.evaluate("AB07", {"hip_flexion_l": 0.2, "pros_knee_angle": -0.3})
        # The unchanged ankle child offset is zero, so foot_l's native origin
        # independently locates the pivot after either anatomical or ground
        # fitting. A fixed numerical mount from an older bundle is stale.
        foot = model.getBodySet().get("foot_l")
        correct = array(foot.findStationLocationInGround(state, opensim.Vec3(0, 0, 0)))
        old = array(tibia.findStationLocationInGround(state, opensim.Vec3(0, -0.43, 0)))
        np.testing.assert_allclose(center, correct, atol=1e-12, rtol=0)
        self.assertGreater(float(np.linalg.norm(center - old)), 0.02)
        self.assertAlmostEqual(float(np.linalg.norm(axis)), 1.0, places=12)

    def test_axis_respects_parent_joint_frame_orientation(self) -> None:
        model = opensim.Model(str(REPO / "models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi.osim"))
        frame = opensim.PhysicalOffsetFrame.safeDownCast(model.getJointSet().get("pros_ankle").getParentFrame())
        frame.set_orientation(opensim.Vec3(0.2, -0.1, 0.15))
        state = model.initSystem()
        model.realizePosition(state)
        proxy = SimpleNamespace(ctx=SimpleNamespace(model=model), runner=SimpleNamespace(state=state))
        center, axis = CMCLikeProsthesisTrajectoryEnv._prosthetic_ankle_center_axis_ground(proxy)
        correct_center = array(frame.findStationLocationInGround(state, opensim.Vec3(0, 0, 0)))
        correct_axis = array(frame.findStationLocationInGround(state, opensim.Vec3(0, 0, 1))) - correct_center
        correct_axis /= np.linalg.norm(correct_axis)
        np.testing.assert_allclose(center, correct_center, atol=1e-12, rtol=0)
        np.testing.assert_allclose(axis, correct_axis, atol=1e-12, rtol=0)


if __name__ == "__main__":
    unittest.main()
