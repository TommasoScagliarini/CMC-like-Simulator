"""Pure contract tests for the post-sealed V13 development baseline."""

from __future__ import annotations

import hashlib
import json
import unittest
from pathlib import Path

from online_grf import load_online_grf_profile


REPO_ROOT = Path(__file__).resolve().parents[1]
PROFILE_DIR = REPO_ROOT / "validation" / "experimental_detector_profiles"
REGISTRY_PATH = PROFILE_DIR / "two_sensor_development_baseline_registry.json"
V13_PROFILE_PATH = (
    PROFILE_DIR
    / "two_sensor_v13_development_toe_down_p0p75mm_heel_x_p3p5mm.json"
)
V9_PROFILE_PATH = PROFILE_DIR / "two_sensor_v9_H2p50_X3p25_F79p0_P35p00.json"
LOCK_PATH = REPO_ROOT / "validation" / "two_sensor_sealed_candidate_lock_v13.json"
MANIFEST_PATH = (
    REPO_ROOT
    / "validation"
    / "two_sensor_sealed_runs"
    / "2026-07-22_ab06_100_155p045_v13"
    / "manifest.json"
)
RUNTIME_PROFILE_PATH = (
    REPO_ROOT
    / "online_grf_profiles"
    / "AB06_SEASEA_stiff321_500_pi_grf_detector_HS-TO.json"
)
TRAINING_CONFIG_PATH = (
    REPO_ROOT / "Trajectory Generator" / "baseline_MLP" / "training_exnovo_cfg.yaml"
)


def _json(path: Path) -> dict[str, object]:
    return json.loads(path.read_text(encoding="utf-8"))


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


class TwoSensorDevelopmentBaselineV13Test(unittest.TestCase):
    def setUp(self) -> None:
        self.registry = _json(REGISTRY_PATH)
        self.raw_profile = _json(V13_PROFILE_PATH)
        self.lock = _json(LOCK_PATH)
        self.manifest = _json(MANIFEST_PATH)

    def test_registry_selects_v13_and_keeps_v9_historical(self) -> None:
        active = self.registry["active_development_baseline"]
        historical = self.registry["historical_baseline"]
        self.assertEqual(active["baseline_id"], "two_sensor_v13_development_baseline")
        self.assertEqual(active["status"], "ACTIVE_DEVELOPMENT_BASELINE")
        self.assertEqual(
            active["profile"]["path"],
            V13_PROFILE_PATH.relative_to(REPO_ROOT).as_posix(),
        )
        self.assertEqual(active["profile"]["sha256"], _sha256(V13_PROFILE_PATH))
        self.assertEqual(
            historical["baseline_id"],
            "two_sensor_v9_H2p50_X3p25_F79p0_P35p00",
        )

    def test_profile_is_exactly_the_locked_primary_geometry(self) -> None:
        primary = self.lock["primary"]
        spheres = {item["name"]: item for item in self.raw_profile["spheres"]}
        self.assertEqual(set(spheres), {"left_heel", "left_toe"})
        self.assertEqual(spheres["left_heel"]["location"], primary["heel_location_m"])
        self.assertEqual(spheres["left_toe"]["location"], primary["toe_location_m"])
        self.assertEqual(spheres["left_heel"]["radius"], primary["heel_radius_m"])
        self.assertEqual(spheres["left_toe"]["radius"], primary["toe_radius_m"])
        self.assertEqual(
            self.raw_profile["metadata"]["candidate_id"], primary["candidate_id"]
        )

    def test_profile_is_a_loadable_two_sphere_detector(self) -> None:
        profile = load_online_grf_profile(V13_PROFILE_PATH, required_sides=("left",))
        self.assertEqual(len(profile.spheres), 2)
        self.assertEqual({sphere.name for sphere in profile.spheres}, {"left_heel", "left_toe"})
        self.assertTrue(all(sphere.side == "left" for sphere in profile.spheres))
        self.assertTrue(all(sphere.frame == "/bodyset/foot_l" for sphere in profile.spheres))

    def test_sealed_failure_is_preserved_and_runtime_use_is_forbidden(self) -> None:
        metadata = self.raw_profile["metadata"]
        validation_state = self.registry["validation_state"]
        self.assertEqual(self.manifest["conclusion"], "PRIMARY_SEALED_FAIL")
        self.assertEqual(metadata["sealed_validation"]["conclusion"], "PRIMARY_SEALED_FAIL")
        self.assertFalse(metadata["sealed_validation"]["passed"])
        self.assertFalse(validation_state["sealed_validated"])
        self.assertFalse(validation_state["runtime_promoted"])
        self.assertFalse(validation_state["training_promoted"])
        self.assertIn("runtime activation", metadata["forbidden_uses"])
        self.assertIn(
            "retuning against the consumed V13 sealed interval",
            metadata["forbidden_uses"],
        )

    def test_lineage_and_unchanged_inputs_are_hash_bound(self) -> None:
        expected = {
            LOCK_PATH: "9c062463592b896e5ca6a08b946b3c47b8ea4fa1815943b8b039b846117546cd",
            MANIFEST_PATH: "d55d31188fe3fe853af6618377c6fa5e8cba9f865db9194078820322ed8877a9",
            V9_PROFILE_PATH: "e08a7582776b41e883ae50cfce9ce53b5f6508c00b73ca54957afc98d01a4489",
            RUNTIME_PROFILE_PATH: "61ea948a3c0613e5c0e684a3197de118c7116e36188fca6993da79ce713fd99e",
            TRAINING_CONFIG_PATH: "847da69896fec1ae2a8da34b56369fdd7edf2b6b6bbdb1a463b30cd2d830ee13",
        }
        for path, digest in expected.items():
            with self.subTest(path=path):
                self.assertEqual(_sha256(path), digest)

        lineage = self.raw_profile["metadata"]["lineage"]
        self.assertEqual(lineage["candidate_lock"]["sha256"], expected[LOCK_PATH])
        self.assertEqual(lineage["sealed_manifest"]["sha256"], expected[MANIFEST_PATH])
        self.assertEqual(lineage["historical_v9_profile"]["sha256"], expected[V9_PROFILE_PATH])

    def test_registry_is_explicitly_development_only(self) -> None:
        designation = self.registry["designation"]
        self.assertTrue(designation["does_not_claim_metric_dominance"])
        self.assertTrue(designation["does_not_reinterpret_sealed_result"])
        self.assertIn("consumed 100-155.045 s sealed interval", self.registry["next_candidate_rule"])


if __name__ == "__main__":
    unittest.main()
