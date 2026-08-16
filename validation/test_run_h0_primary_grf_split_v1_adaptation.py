"""Static/preflight tests for the primary-split one-shot driver."""

from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

import numpy as np

from validation import run_h0_primary_grf_split_v1_adaptation as driver


class _FakeFSM:
    def __init__(self) -> None:
        self.kwargs = None

    def update(self, **kwargs):
        self.kwargs = kwargs
        return {"updated": True}


class PrimarySplitDriverTests(unittest.TestCase):
    def test_environment_overlay_is_primary_split_legacy_only(self) -> None:
        for trial_id, trial in driver.TRIALS.items():
            config = driver.build_env_config(
                trial_id,
                offset_s=float(trial["collection_offset_s"]),
                seed=int(trial["collection_seed"]),
            )
            self.assertEqual(config["phase_fsm_input_mode"], "legacy_events")
            self.assertEqual(config["binary_phase_fsm_mode"], "disabled")
            self.assertIsNone(config["binary_phase_detector_profile_file"])
            self.assertEqual(config["event_contract_id"], driver.EVENT_CONTRACT)
            self.assertEqual(config["online_grf_applied_sides"], ["left"])
            self.assertEqual(config["reward"]["morphology_weight"], 0.0)
            self.assertIn(
                f"trial_{trial_id}_primary_surface_velocity.json",
                config["online_grf_profile_file"],
            )
            self.assertIn(
                f"trial_{trial_id}_analog_surface_velocity.json",
                config["online_grf_detector_profile_file"],
            )

    def test_preregistered_absolute_times_match_setup_offsets(self) -> None:
        starts = {"02": 11.698, "04": 14.639, "08": 13.512}
        for trial_id, trial in driver.TRIALS.items():
            self.assertAlmostEqual(
                starts[trial_id] + float(trial["collection_offset_s"]),
                float(trial["collection_absolute_s"]),
            )
            self.assertAlmostEqual(
                starts[trial_id] + float(trial["qualification_offset_s"]),
                float(trial["qualification_absolute_s"]),
            )

    def test_shadow_fsm_consumes_analog_continuous_evidence(self) -> None:
        fsm = _FakeFSM()
        result = driver._update_shadow_fsm(
            fsm,
            info={
                "time": 10.01,
                "observation": {
                    "pros_knee_angle": -0.4,
                    "pros_ankle_angle": 0.1,
                },
                "legacy_online_events": [],
                "online_grf_detector": {
                    "left": {"normal_force": 80.0, "in_contact": True}
                },
            },
            body_weight_n=800.0,
        )
        self.assertEqual(result, {"updated": True})
        self.assertAlmostEqual(fsm.kwargs["normal_force_bw"], 0.1)
        self.assertTrue(fsm.kwargs["in_contact"])
        self.assertAlmostEqual(fsm.kwargs["prosthetic_knee_angle_rad"], -0.4)

    def test_preregistered_rollout_provenance_rejects_window_drift(self) -> None:
        noise = driver._load_noise_tape(
            stage="collection", trial_id="02", selection="stochastic"
        )
        noise_sha = driver.split_contract.array_sha256(noise)
        summary = {
            "trial_id": "02",
            "plateau_id": "04",
            "behavior_role": "teacher",
            "action_selection": "stochastic",
            "seed": 123,
            "episode_start_offset_s": 107.880,
            "episode_start_time_s": 119.578,
            "sigma": driver.COLLECTION_SIGMA,
            "noise_tape_sha256": noise_sha,
            "event_contract_id": driver.EVENT_CONTRACT,
            "binary_phase_fsm_mode": "disabled",
            "grf_mode": "online_sensor",
            "online_grf_applied_sides": ["left"],
            "teacher_action_drove_environment": True,
            "ppo_updates": 0,
            "protected_trials_opened": [],
        }
        gate = driver.preregistered_rollout_provenance(
            summary,
            trial_id="02",
            collection=True,
            behavior_role="teacher",
            selection="stochastic",
            noise_tape_sha256=noise_sha,
        )
        self.assertTrue(gate["passed"])
        summary["episode_start_time_s"] += 0.001
        self.assertFalse(
            driver.preregistered_rollout_provenance(
                summary,
                trial_id="02",
                collection=True,
                behavior_role="teacher",
                selection="stochastic",
                noise_tape_sha256=noise_sha,
            )["passed"]
        )

    def test_npz_writer_is_no_clobber(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "data.npz"
            driver._write_npz_exclusive(
                path, values=np.asarray([1.0, 2.0], dtype=np.float32)
            )
            with np.load(path, allow_pickle=False) as archive:
                np.testing.assert_array_equal(archive["values"], [1.0, 2.0])
            with self.assertRaisesRegex(driver.PrimarySplitExecutionError, "clobber"):
                driver._write_npz_exclusive(
                    path, values=np.asarray([3.0], dtype=np.float32)
                )

    def test_preregistered_noise_tapes_are_loaded_by_stage(self) -> None:
        collection = driver._load_noise_tape(
            stage="collection",
            trial_id="02",
            selection="stochastic",
        )
        qualification = driver._load_noise_tape(
            stage="qualification",
            trial_id="02",
            selection="stochastic",
        )
        deterministic = driver._load_noise_tape(
            stage="qualification",
            trial_id="02",
            selection="deterministic",
        )
        self.assertEqual(collection.shape, (500, 2))
        self.assertEqual(collection.dtype, np.float32)
        self.assertFalse(np.array_equal(collection, qualification))
        self.assertEqual(np.count_nonzero(deterministic), 0)


if __name__ == "__main__":
    unittest.main()
