from __future__ import annotations

import unittest
from unittest import mock

from validation import freeze_h0_v3_v25_abc_post_zero_port as freezer


class PostZeroPortFreezerTests(unittest.TestCase):
    def test_payload_freezes_full_matrix_policy_and_zero_authority(self) -> None:
        with mock.patch.object(
            freezer.driver,
            "source_record",
            side_effect=lambda path: {
                "path": str(path),
                "sha256": "a" * 64,
                "size_bytes": 1,
            },
        ):
            payload = freezer.build_payload(validate_inputs=False)
        self.assertEqual(payload["status"], freezer.contract.LOCK_STATUS)
        self.assertEqual(payload["so_policy_id"], freezer.contract.SO_POLICY_ID)
        self.assertEqual(payload["matrix"]["rollout_count"], 18)
        self.assertTrue(payload["gates"]["raw_fallback_counters_preserved"])
        self.assertEqual(payload["actor_updates"], 0)
        self.assertEqual(payload["critic_updates"], 0)
        self.assertEqual(payload["ppo_updates"], 0)
        self.assertEqual(payload["protected_trials_opened"], [])

    def test_missing_v3_prerequisite_fails_closed(self) -> None:
        with mock.patch.object(
            freezer,
            "_strict",
            side_effect=freezer.PostZeroPortFreezeError("missing"),
        ):
            with self.assertRaisesRegex(freezer.PostZeroPortFreezeError, "missing"):
                freezer.validate_prerequisites()

    def test_freezer_contract_does_not_create_rollout_directories(self) -> None:
        self.assertFalse(
            freezer.contract.AUTHORITY["runtime_promotion_authorized"]
        )
        self.assertFalse(
            freezer.contract.AUTHORITY["protected_trial_access_authorized"]
        )


if __name__ == "__main__":
    unittest.main()
