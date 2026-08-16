from __future__ import annotations

import unittest
from pathlib import PurePosixPath

from validation import h0_v3_v25_abc_post_zero_port_contract as contract


class PostZeroPortContractTests(unittest.TestCase):
    def test_six_cases_and_three_modes_are_exact(self) -> None:
        self.assertEqual(len(contract.CASES), 6)
        self.assertEqual(set(contract.MODES), {"A", "B", "C"})
        self.assertEqual(
            [item["seed"] for item in contract.CASES[3:]], [126, 127, 128]
        )
        self.assertEqual(
            [item["offset_s"] for item in contract.CASES[:3]],
            [
                contract.CANONICAL_OFFSET_S - 0.2,
                contract.CANONICAL_OFFSET_S,
                contract.CANONICAL_OFFSET_S + 0.2,
            ],
        )

    def test_modes_preserve_legacy_shadow_active_semantics(self) -> None:
        self.assertEqual(contract.MODES["A"]["event_source"], "legacy_events")
        self.assertEqual(contract.MODES["B"]["event_source"], "legacy_events")
        self.assertEqual(contract.MODES["C"]["event_source"], "v25_fsm_v20")
        self.assertEqual(contract.MODES["C"]["binary_phase_fsm_mode"], "binary_active")
        self.assertEqual(contract.MORPHOLOGY_WEIGHT, 0.0)

    def test_all_paths_are_portable_repository_relative(self) -> None:
        values = [
            contract.LOCK_RELATIVE_PATH,
            contract.RUN_ROOT_RELATIVE_PATH,
            *contract.INPUT_RELATIVE_PATHS.values(),
            *contract.SOURCE_RELATIVE_PATHS.values(),
        ]
        for value in values:
            with self.subTest(value=value):
                pure = PurePosixPath(value)
                self.assertFalse(pure.is_absolute())
                self.assertNotIn("..", pure.parts)
                self.assertNotIn("\\", value)
                self.assertEqual(pure.as_posix(), value)

    def test_authority_forbids_all_updates_and_protected_access(self) -> None:
        self.assertTrue(contract.AUTHORITY["full_environment_rollout_authorized"])
        for key in (
            "actor_updates_authorized",
            "critic_updates_authorized",
            "ppo_updates_authorized",
            "training_authorized",
            "protected_trial_access_authorized",
            "primary_grf_modification_authorized",
            "detector_retuning_authorized",
            "sea_semantic_modification_authorized",
        ):
            self.assertFalse(contract.AUTHORITY[key])


if __name__ == "__main__":
    unittest.main()
