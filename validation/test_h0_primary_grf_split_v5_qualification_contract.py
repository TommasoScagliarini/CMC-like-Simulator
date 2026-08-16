from __future__ import annotations

import unittest

from validation import h0_primary_grf_split_v5_freeze_contract as v5
from validation import h0_primary_grf_split_v5_qualification_contract as contract


class QualificationContractTests(unittest.TestCase):
    def test_lineage_and_paths_are_v5_only(self) -> None:
        self.assertEqual(contract.SCHEMA_VERSION, v5.SCHEMA_VERSION)
        self.assertEqual(contract.SOURCE_PROTOCOL_ID, v5.PROTOCOL_ID)
        self.assertEqual(contract.CANDIDATE_ID, v5.CANDIDATE_ID)
        self.assertEqual(contract.V5_RUN_ROOT.as_posix(), v5.RUN_ROOT_RELATIVE)
        self.assertEqual(
            contract.V5_EXECUTION_LOCK_PATH.as_posix(),
            v5.LOCK_RELATIVE,
        )
        for value in (
            contract.LOCK_PATH,
            contract.RUN_ROOT,
            contract.CANDIDATE_MODULE_PATH,
            contract.CANDIDATE_FREEZE_PATH,
            contract.HOLDOUT_RECEIPT_PATH,
        ):
            self.assertIn("v5", value.as_posix())
            self.assertNotIn("v3_semantic_replay/qualification", value.as_posix())

    def test_exact_six_cases_and_noise_tapes_are_unchanged(self) -> None:
        cases = contract.canonical_cases()
        self.assertEqual(tuple(item["case_id"] for item in cases), contract.CASE_IDS)
        self.assertEqual(len(cases), 6)
        self.assertEqual(
            [item["episode_start_offset_s"] for item in cases[:3]],
            [
                contract.CANONICAL_OFFSET_S - 0.2,
                contract.CANONICAL_OFFSET_S,
                contract.CANONICAL_OFFSET_S + 0.2,
            ],
        )
        self.assertEqual([item["action_seed"] for item in cases[3:]], [126, 127, 128])
        self.assertTrue(
            all(item["runtime_seed"] == 123 for item in cases[:3])
        )

    def test_status0_policy_tracks_raw_recovery_without_rejecting_it(self) -> None:
        self.assertEqual(contract.SO_POLICY_ID, "verified_status0_max_iter_v1")
        self.assertNotIn("raw_so_fallback_count", contract.ZERO_REQUIRED_COUNTS)
        self.assertIn("fallback_count", contract.ZERO_REQUIRED_COUNTS)
        self.assertIn(
            "so_solver_unaccepted_bounded_ls_count",
            contract.ZERO_REQUIRED_COUNTS,
        )
        self.assertIn("sea_plugin_fallback_count", contract.ZERO_REQUIRED_COUNTS)

    def test_tolerances_are_exactly_the_accepted_conservative_values(self) -> None:
        rows = contract.tolerance_rows()
        reserve = {item["metric"]: item for item in rows["reserve"]}
        self.assertEqual(
            reserve["reserve_norm_nm.rms"]["absolute_tolerance"],
            5.0,
        )
        self.assertEqual(
            reserve["reserve_norm_nm.rms"]["relative_tolerance"],
            0.05,
        )
        self.assertEqual(
            reserve["residual_norm_nm.rms"]["absolute_tolerance"],
            1.0e-6,
        )
        self.assertEqual(len(rows["sea"]), 2 * 6 * 2)
        self.assertTrue(
            all(item["absolute_tolerance"] == 1.0e-6 for item in rows["sea"])
        )
        self.assertTrue(
            all(item["relative_tolerance"] == 0.05 for item in rows["sea"])
        )

    def test_authority_keeps_later_and_protected_stages_closed(self) -> None:
        self.assertTrue(
            contract.AUTHORITY["autonomous_qualification_execution_authorized"]
        )
        for key in (
            "actor_updates_authorized",
            "critic_updates_authorized",
            "ppo_updates_authorized",
            "zero_update_port_authorized",
            "v25_abc_execution_authorized",
            "protected_trial_access_authorized",
            "reserve_trial_access_authorized",
            "runtime_promotion_authorized",
        ):
            self.assertFalse(contract.AUTHORITY[key])


if __name__ == "__main__":
    unittest.main()
