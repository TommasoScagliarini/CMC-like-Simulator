"""Preparation checks only; this module never starts an environment."""

from __future__ import annotations

import copy
import importlib.util
import unittest
from pathlib import Path

import yaml


SPEC = importlib.util.spec_from_file_location(
    "ab07_qualification", Path(__file__).with_name("qualify_prescribed_v26.py")
)
assert SPEC is not None and SPEC.loader is not None
PROBE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(PROBE)


class QualificationPreparationTests(unittest.TestCase):
    def setUp(self) -> None:
        self.source = yaml.safe_load(PROBE.SOURCE_CONFIG.read_text(encoding="utf-8"))

    def kwargs(self, source=None, stage="left_applied"):
        return PROBE.build_env_kwargs(
            self.source if source is None else source, setup=PROBE.DEFAULT_SETUP,
            output_dir="results/ab07_contact_probe_test_only", stage=stage,
        )

    def test_governor_and_fsm_settings_are_lineage_values(self) -> None:
        before = copy.deepcopy(self.source)
        cfg = self.kwargs()
        self.assertEqual(cfg["pros_ref_model"], "butterworth3_jerk_limited")
        self.assertEqual(cfg["pros_ref_lpf_cutoff_hz"], 4.0)
        self.assertEqual(cfg["pros_ref_velocity_limit_rad_s"], {"pros_knee_angle": 6.0, "pros_ankle_angle": 3.5})
        self.assertEqual(cfg["grf_penetration_penalty_threshold_m"], 0.02)
        self.assertEqual(cfg["grf_penetration_termination_m"], 0.028)
        self.assertEqual(cfg["phase_min_stance_duration_s"], 0.05)
        self.assertEqual(cfg["phase_min_stance_load_bw_s"], 0.0)
        self.assertEqual(cfg["phase_min_cycle_knee_excursion_rad"], 0.0)
        self.assertEqual(cfg["phase_swing_hard_timeout_s"], 2.6)
        self.assertEqual(cfg["binary_phase_actor_fsm_version"], "v3")
        self.assertEqual(cfg["binary_phase_debounce_s"], 0.005)
        self.assertEqual(cfg["detector_sample_dt_s"], 0.001)
        self.assertEqual(cfg["binary_phase_invalid_event_policy"], "reject_continue")
        for key, value in self.source["reward"].items():
            if key.startswith("phase_") and key in cfg:
                self.assertEqual(cfg[key], value, key)
        self.assertEqual(cfg["phase_landing_force_full_credit_bw"], self.source["reward"]["contact_load_target_bw"])
        self.assertEqual(self.source, before)

    def test_shared_profile_paths_and_contact_modes(self) -> None:
        applied, sensed = self.kwargs(), self.kwargs(stage="sensor_only")
        for key, (path, digest) in PROBE.FROZEN_PROFILES.items():
            self.assertEqual(applied[key], path)
            self.assertEqual(sensed[key], path)
            self.assertEqual(PROBE._fingerprint(path)["sha256"], digest)
        self.assertEqual(applied["online_grf_applied_sides"], ["left"])
        self.assertEqual(sensed["online_grf_applied_sides"], [])
        self.assertEqual(applied["prescribed_grf_disabled_sides"], [])

    def test_prescribed_reset_raw_action_and_duration_are_explicit(self) -> None:
        cfg = self.kwargs()
        self.assertFalse(cfg["imitation_initialize_to_target"])
        self.assertFalse(cfg["random_init"])
        self.assertEqual(cfg["episode_start_offset_s"], 0.0)
        self.assertEqual(cfg["episode_duration"], 9.01)
        self.assertEqual(cfg["action_mode"], "raw")
        self.assertEqual(cfg["segment_duration"], 0.01)
        self.assertEqual(cfg["policy_knots"], 1)
        self.assertNotIn("checkpoint", cfg)

    def test_profile_rename_and_incompatible_source_are_rejected(self) -> None:
        altered = copy.deepcopy(self.source)
        altered["grf"]["online_grf_detector_profile"] = "online_grf_profiles/AB07_copy.json"
        with self.assertRaisesRegex(ValueError, "profile path differs"):
            self.kwargs(altered)
        altered = copy.deepcopy(self.source)
        altered["grf"]["binary_phase_fsm_mode"] = "disabled"
        with self.assertRaisesRegex(ValueError, "frozen V26"):
            self.kwargs(altered)


if __name__ == "__main__":
    unittest.main()
