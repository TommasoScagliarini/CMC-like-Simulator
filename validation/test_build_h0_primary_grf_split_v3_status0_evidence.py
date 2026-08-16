from __future__ import annotations

import hashlib
import json
import unittest

from validation import build_h0_primary_grf_split_v3_status0_evidence as builder


class Status0EvidenceBuilderTests(unittest.TestCase):
    def test_decision_payload_is_the_exact_closed_authorized_policy(self) -> None:
        payload = builder._decision_payload()
        self.assertEqual(payload["selected_policy_id"], builder.POLICY_ID)
        self.assertEqual(
            payload["policy"], builder.freeze_contract.SO_POLICIES[builder.POLICY_ID]
        )
        self.assertEqual(payload["authority_basis"], "explicit_user_authorization")
        self.assertEqual(payload["actor_updates"], 0)
        self.assertEqual(payload["critic_updates"], 0)
        self.assertEqual(payload["ppo_updates"], 0)
        self.assertEqual(payload["protected_trials_opened"], [])

    def test_virtual_record_matches_canonical_rendering(self) -> None:
        payload = builder._decision_payload()
        rendered = builder._json_bytes(payload)
        record = builder._virtual_record(builder.freezer.DECISION_RECEIPT, payload)
        self.assertEqual(record["sha256"], hashlib.sha256(rendered).hexdigest())
        self.assertEqual(record["size_bytes"], len(rendered))
        self.assertNotIn("\\", record["path"])
        self.assertEqual(json.loads(rendered), payload)

    def test_policy_identifiers_and_limits_match_classifier(self) -> None:
        policy = builder.freeze_contract.SO_POLICIES[builder.POLICY_ID]
        self.assertEqual(
            builder.POLICY_ID,
            builder.so_recovery.VERIFIED_STATUS0_MAX_ITER_POLICY,
        )
        self.assertEqual(
            policy["required_bounded_lsq_iterations"],
            builder.so_recovery.BOUNDED_LSQ_MAX_ITER,
        )
        self.assertEqual(
            policy["bounded_lsq_optimality_max"],
            builder.so_recovery.BOUNDED_LSQ_STATUS0_OPTIMALITY_MAX,
        )
        self.assertEqual(
            policy["residual_telemetry_consistency_atol"],
            builder.so_recovery.RESIDUAL_CONSISTENCY_ATOL,
        )


if __name__ == "__main__":
    unittest.main()
