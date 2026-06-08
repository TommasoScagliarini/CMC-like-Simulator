from __future__ import annotations

import sys
import unittest
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from online_grf import (
    OnlineGRFMaterial,
    OnlineGRFProfile,
    StreamingGaitEventDetector,
    flatten_online_grf,
)


class OnlineGRFCoreTests(unittest.TestCase):
    def test_profile_requires_both_sides(self) -> None:
        with self.assertRaises(ValueError):
            OnlineGRFProfile.from_mapping(
                {
                    "spheres": [
                        {
                            "name": "left_heel",
                            "side": "left",
                            "frame": "/bodyset/foot_l",
                            "location": [0, 0, 0],
                            "radius": 0.03,
                        }
                    ]
                }
            )

    def test_streaming_events_require_sustained_contact(self) -> None:
        detector = StreamingGaitEventDetector(20.0, 0.02, 0.3)
        events = []
        for time, force in [
            (0.00, 0),
            (0.01, 30),
            (0.02, 30),
            (0.031, 30),
            (0.04, 0),
        ]:
            events.extend(detector.update(time, {"left": force, "right": 0}))
        self.assertEqual([event["event"] for event in events], ["heel_strike", "toe_off"])
        self.assertAlmostEqual(events[0]["time"], 0.01)

    def test_streaming_events_ignore_unconfirmed_contact_bump(self) -> None:
        detector = StreamingGaitEventDetector(20.0, 0.02, 0.3, 100.0)
        events = []
        for time, force in [
            (0.00, 0),
            (0.01, 30),
            (0.04, 30),
            (0.05, 0),
            (0.10, 30),
            (0.11, 120),
            (0.13, 120),
            (0.14, 0),
        ]:
            events.extend(detector.update(time, {"left": force, "right": 0}))
        self.assertEqual([event["event"] for event in events], ["heel_strike", "toe_off"])
        self.assertAlmostEqual(events[0]["time"], 0.10)
        self.assertAlmostEqual(events[0]["confirmed_time"], 0.13)

    def test_flatten_has_two_side_blocks(self) -> None:
        side = {
            "force": np.zeros(3),
            "moment": np.zeros(3),
            "cop": np.zeros(3),
            "normal_force": 0.0,
            "penetration": 0.0,
            "slip_speed": 0.0,
            "in_contact": False,
        }
        values = flatten_online_grf({"sides": {"left": side, "right": side}})
        self.assertEqual(values.shape, (26,))

    def test_contact_flag_is_separate_from_smoothed_force(self) -> None:
        side = {
            "force": np.zeros(3),
            "moment": np.zeros(3),
            "cop": np.zeros(3),
            "normal_force": 1e-3,
            "penetration": 0.0,
            "slip_speed": 0.0,
            "in_contact": False,
        }
        values = flatten_online_grf({"sides": {"left": side, "right": side}})
        self.assertEqual(values[12], 0.0)

    def test_profile_supports_per_sphere_material_and_exponent(self) -> None:
        profile = OnlineGRFProfile.from_mapping(
            {
                "spheres": [
                    {
                        "name": "left_heel",
                        "side": "left",
                        "frame": "/bodyset/foot_l",
                        "location": [0, 0, 0],
                        "radius": 0.03,
                        "material": {"stiffness": 1234, "exponent": 0.8},
                    },
                    {
                        "name": "right_heel",
                        "side": "right",
                        "frame": "/bodyset/foot_r",
                        "location": [0, 0, 0],
                        "radius": 0.03,
                    },
                ]
            }
        )
        self.assertIsInstance(profile.spheres[0].material, OnlineGRFMaterial)
        self.assertAlmostEqual(profile.spheres[0].material.exponent, 0.8)
        self.assertIsNone(profile.spheres[1].material)
        self.assertEqual(
            profile.to_dict()["spheres"][0]["material"]["stiffness"],
            1234.0,
        )

    def test_profile_round_trips_state_based_residual(self) -> None:
        profile = OnlineGRFProfile.from_mapping(
            {
                "spheres": [
                    {
                        "name": "left",
                        "side": "left",
                        "frame": "/bodyset/foot_l",
                        "location": [0, 0, 0],
                        "radius": 0.03,
                        "residual_force_ratio": [0.1, -0.2, 0.3],
                        "residual_moment_ratio_m": [0.01, 0.02, -0.03],
                        "residual_penetration_reference_m": 0.01,
                        "residual_force_penetration_gain_per_m": [0, 12, 0],
                        "residual_force_penetration_rate_gain_s_per_m": [0, 0.5, 0],
                    },
                    {
                        "name": "right",
                        "side": "right",
                        "frame": "/bodyset/foot_r",
                        "location": [0, 0, 0],
                        "radius": 0.03,
                    },
                ]
            }
        )
        sphere = profile.spheres[0]
        self.assertEqual(sphere.residual_force_ratio, (0.1, -0.2, 0.3))
        self.assertEqual(sphere.residual_moment_ratio_m, (0.01, 0.02, -0.03))
        self.assertEqual(sphere.residual_penetration_reference_m, 0.01)
        self.assertEqual(
            sphere.residual_force_penetration_gain_per_m,
            (0.0, 12.0, 0.0),
        )
        encoded = profile.to_dict()["spheres"]
        self.assertEqual(encoded[0]["residual_force_ratio"], [0.1, -0.2, 0.3])
        self.assertNotIn("residual_force_ratio", encoded[1])


if __name__ == "__main__":
    unittest.main()
