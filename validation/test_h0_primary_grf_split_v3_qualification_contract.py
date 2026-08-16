from __future__ import annotations

import unittest

from validation import h0_primary_grf_split_v3_qualification_contract as contract
from validation import h0_primary_grf_split_v3_qualification_scaffold as scaffold


class QualificationContractTests(unittest.TestCase):
    def test_six_cases_match_existing_scaffold(self) -> None:
        cases = contract.canonical_cases()
        scaffold_cases = scaffold.canonical_cases()
        self.assertEqual(tuple(item["case_id"] for item in cases), scaffold.CASE_IDS)
        self.assertEqual(len(cases), 6)
        for observed, frozen in zip(cases, scaffold_cases, strict=True):
            self.assertEqual(observed["action_selection"], frozen["action_selection"])
            self.assertEqual(observed["episode_start_offset_s"], frozen["historical_offset_s"])
            self.assertEqual(observed["action_seed"], frozen["seed"])
            self.assertEqual(observed["sigma"], frozen["sigma"])
            self.assertEqual(observed["noise_tape"], frozen["noise_tape"])
            self.assertEqual(observed["runtime_seed"], observed["action_seed"] or 123)

    def test_status0_policy_tracks_raw_recovery_without_zeroing_it(self) -> None:
        self.assertEqual(contract.SO_POLICY_ID, "verified_status0_max_iter_v1")
        self.assertNotIn("raw_so_fallback_count", contract.ZERO_REQUIRED_COUNTS)
        self.assertIn("fallback_count", contract.ZERO_REQUIRED_COUNTS)
        self.assertIn("so_solver_unaccepted_bounded_ls_count", contract.ZERO_REQUIRED_COUNTS)
        self.assertIn("sea_plugin_fallback_count", contract.ZERO_REQUIRED_COUNTS)

    def test_tolerances_are_conservative_and_complete(self) -> None:
        rows = contract.tolerance_rows()
        reserve = {item["metric"]: item for item in rows["reserve"]}
        self.assertEqual(reserve["reserve_norm_nm.rms"]["absolute_tolerance"], 5.0)
        self.assertEqual(reserve["reserve_norm_nm.rms"]["relative_tolerance"], 0.05)
        self.assertEqual(reserve["residual_norm_nm.rms"]["absolute_tolerance"], 1.0e-6)
        self.assertEqual(len(rows["sea"]), 2 * 6 * 2)
        self.assertTrue(all(item["absolute_tolerance"] == 1.0e-6 for item in rows["sea"]))
        self.assertTrue(all(item["relative_tolerance"] == 0.05 for item in rows["sea"]))

    def test_authority_keeps_later_and_protected_stages_closed(self) -> None:
        self.assertTrue(contract.AUTHORITY["autonomous_qualification_execution_authorized"])
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
