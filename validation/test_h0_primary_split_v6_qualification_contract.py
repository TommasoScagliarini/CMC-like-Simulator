from __future__ import annotations

import unittest

from validation import h0_primary_grf_split_v5_qualification_contract as v5
from validation import h0_primary_split_v6_qualification_contract as contract


class H0PrimarySplitV6QualificationContractTests(unittest.TestCase):
    def test_exact_six_fresh_unopened_conditions(self) -> None:
        cases = contract.canonical_cases()
        self.assertEqual(tuple(row["case_id"] for row in cases), contract.CASE_IDS)
        self.assertEqual(len(cases), 6)
        self.assertEqual(
            [row["episode_start_offset_s"] for row in cases[:2]],
            [
                contract.CANONICAL_OFFSET_S - 0.30,
                contract.CANONICAL_OFFSET_S + 0.30,
            ],
        )
        self.assertEqual([row["action_seed"] for row in cases[:2]], [None, None])
        self.assertEqual([row["runtime_seed"] for row in cases[:2]], [129, 129])
        self.assertEqual(
            [row["action_seed"] for row in cases[2:]],
            [130, 131, 132, 133],
        )
        self.assertEqual(
            [row["runtime_seed"] for row in cases[2:]],
            [130, 131, 132, 133],
        )
        self.assertEqual([row["sigma"] for row in cases[:2]], [0.0, 0.0])
        self.assertTrue(
            all(row["sigma"] == contract.STOCHASTIC_SIGMA for row in cases[2:])
        )

        # Neither the old offsets nor V5's 126--128 tapes are recycled.
        self.assertTrue({126, 127, 128}.isdisjoint(contract.STOCHASTIC_SEEDS))
        self.assertNotEqual(
            {row["noise_tape"] for row in cases},
            {row["noise_tape"] for row in v5.canonical_cases()},
        )

    def test_zero_tape_is_shared_and_stochastic_tapes_are_unique(self) -> None:
        cases = contract.canonical_cases()
        self.assertEqual(cases[0]["noise_tape"], cases[1]["noise_tape"])
        stochastic = [row["noise_tape"] for row in cases[2:]]
        self.assertEqual(len(set(stochastic)), 4)
        self.assertNotIn(cases[0]["noise_tape"], stochastic)

    def test_baseline_and_candidate_semantics_are_explicit(self) -> None:
        baseline = contract.role_contract("baseline")
        self.assertEqual(baseline["actor_id"], "original_h0")
        self.assertEqual(baseline["actor_input_view"], "historical_analog")
        self.assertEqual(baseline["observation_semantics"], "counterfactual_analog")
        self.assertEqual(baseline["phase_fsm_input_mode"], "legacy_events")
        self.assertEqual(baseline["event_contract_id"], "legacy_events")
        self.assertEqual(baseline["binary_phase_fsm_mode"], "disabled")

        candidate = contract.role_contract("candidate")
        self.assertEqual(
            candidate["event_contract_id"],
            "binary_point_v25+functional_contact_fsm_v1",
        )
        self.assertEqual(candidate["phase_fsm_input_mode"], "legacy_events")
        self.assertEqual(candidate["binary_phase_fsm_mode"], "binary_active")
        self.assertEqual(candidate["actor_input_view"], "primary_split")
        self.assertEqual(
            baseline["primary_load_contract_id"],
            candidate["primary_load_contract_id"],
        )
        with self.assertRaisesRegex(ValueError, "unknown qualification role"):
            contract.role_contract("shadow")

    def test_physical_gates_and_v5_tolerances_are_frozen(self) -> None:
        self.assertEqual(contract.EXPECTED_STEPS, 500)
        self.assertEqual(contract.EXPECTED_CONTROL_WINDOWS, 5000)
        self.assertEqual(contract.MINIMUM_VALID_CYCLES, 2)
        self.assertEqual(contract.PENETRATION_LIMIT_M, 0.025)
        self.assertEqual(contract.RESERVE_TOLERANCES, v5.RESERVE_TOLERANCES)
        self.assertEqual(contract.SEA_TOLERANCES, v5.SEA_TOLERANCES)
        self.assertEqual(contract.SO_POLICY_ID, v5.SO_POLICY_ID)
        self.assertNotIn("raw_so_fallback_count", contract.ZERO_REQUIRED_COUNTS)
        for counter in (
            "action_clipped_values",
            "fallback_count",
            "timeout_count",
            "sea_plugin_fallback_count",
            "so_solver_unaccepted_hard_fallback_count",
            "so_solver_unaccepted_bounded_ls_count",
            "invalid_event_count",
            "nonfinite_count",
        ):
            self.assertIn(counter, contract.ZERO_REQUIRED_COUNTS)
        self.assertNotIn("invalid_event_count", contract.NONINCREASING_COUNTS)

    def test_qualification_is_locked_without_both_prerequisites(self) -> None:
        requirements = contract.prerequisite_requirements()
        self.assertEqual(
            [row["name"] for row in requirements],
            ["candidate_freeze", "development_gate"],
        )
        self.assertEqual(
            requirements[0]["required_status"],
            contract.CANDIDATE_FREEZE_REQUIRED_STATUS,
        )
        self.assertEqual(
            requirements[1]["required_status"],
            contract.DEVELOPMENT_PASS_REQUIRED_STATUS,
        )
        self.assertIs(requirements[1]["required_passed"], True)
        self.assertIn("LOCKED", contract.ACCESS_STATUS)
        self.assertFalse(contract.AUTHORITY["qualification_execution_authorized"])
        self.assertTrue(contract.AUTHORITY["candidate_freeze_required"])
        self.assertTrue(contract.AUTHORITY["development_pass_required"])
        self.assertFalse(contract.AUTHORITY["retry_authorized"])

    def test_no_update_protected_or_primary_authority_is_open(self) -> None:
        for key in (
            "actor_updates_authorized",
            "critic_updates_authorized",
            "ppo_updates_authorized",
            "protected_trial_access_authorized",
            "reserve_trial_access_authorized",
            "runtime_promotion_authorized",
            "primary_grf_modification_authorized",
            "sea_semantic_modification_authorized",
        ):
            self.assertFalse(contract.AUTHORITY[key])


if __name__ == "__main__":
    unittest.main()
