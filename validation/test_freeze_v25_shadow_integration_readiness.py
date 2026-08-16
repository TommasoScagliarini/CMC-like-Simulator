"""Read-only tests for the dormant V25 shadow structural-readiness gate."""

from __future__ import annotations

import json
import os
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch


REPO_ROOT = Path(__file__).resolve().parents[1]
VALIDATION_ROOT = REPO_ROOT / "validation"
if str(VALIDATION_ROOT) not in os.sys.path:
    os.sys.path.insert(0, str(VALIDATION_ROOT))

import freeze_v25_shadow_integration_readiness as subject  # noqa: E402


class V25ShadowStructuralReadinessTests(unittest.TestCase):
    def temporary_dir(self) -> Path:
        temporary = tempfile.TemporaryDirectory(dir=VALIDATION_ROOT)
        self.addCleanup(temporary.cleanup)
        return Path(temporary.name)

    def test_preflight_is_read_only_and_every_hash_is_exact(self) -> None:
        temporary = self.temporary_dir()
        destination = temporary / "shadow_receipt.json"
        before = {
            label: (path.read_bytes(), subject.sha256_file(path))
            for label, (path, _expected) in subject.PINNED.items()
        }
        with patch.object(subject, "DESTINATION", destination):
            preflight = subject.preflight_unfrozen()

        self.assertFalse(destination.exists())
        self.assertEqual(
            preflight["status"],
            "V25_SHADOW_STRUCTURAL_RECEIPT_READY_UNWRITTEN",
        )
        receipt = preflight["receipt_payload"]
        self.assertEqual(
            receipt["status"],
            "V25_SHADOW_STRUCTURALLY_READY_NUMERICAL_AB_UNRUN",
        )
        self.assertTrue(receipt["structural_readiness"])
        self.assertFalse(receipt["numerical_ab_pass_claimed"])
        self.assertFalse(receipt["h0_compatibility_claimed"])
        self.assertTrue(all(receipt["assertions"].values()))
        expected_record = subject.payload_record(destination, receipt)
        self.assertEqual(preflight["receipt_record_if_frozen"], expected_record)

        for label, (path, expected_sha256) in subject.PINNED.items():
            with self.subTest(label=label):
                self.assertEqual(subject.sha256_file(path), expected_sha256)
                self.assertEqual(before[label], (path.read_bytes(), expected_sha256))

    def test_contract_bridge_and_closed_authority_are_unambiguous(self) -> None:
        temporary = self.temporary_dir()
        with patch.object(subject, "DESTINATION", temporary / "receipt.json"):
            receipt = subject.build_receipt_payload()

        contracts = receipt["contracts"]
        self.assertEqual(
            contracts["historical_freeze_fsm_contract_id"],
            subject.HISTORICAL_FSM_CONTRACT_ID,
        )
        self.assertEqual(
            contracts["runtime_binary_phase_event_contract_id"],
            subject.RUNTIME_V25_CONTRACT_ID,
        )
        self.assertEqual(
            contracts["scientific_shadow_bundle_contract_id"],
            subject.SCIENTIFIC_SHADOW_BUNDLE_ID,
        )
        self.assertNotEqual(
            contracts["runtime_binary_phase_event_contract_id"],
            contracts["scientific_shadow_bundle_contract_id"],
        )
        self.assertTrue(contracts["fsm_geometry_agnostic"])
        self.assertFalse(contracts["historical_freeze_reinterpreted_or_rewritten"])

        topology = receipt["dormant_topology"]
        self.assertTrue(topology["legacy_analog_loaded"])
        self.assertTrue(topology["legacy_analog_sensor_only"])
        self.assertFalse(topology["legacy_analog_applies_force"])
        self.assertTrue(topology["legacy_analog_distinct_from_primary_grf"])
        self.assertEqual(topology["actor_observation_count"], 35)
        self.assertEqual(topology["full_observation_count"], 84)

        authority = receipt["authority"]
        self.assertEqual(authority["h0_executed"], False)
        self.assertEqual(authority["binary_active_available"], False)
        for key, value in authority.items():
            if key == "h0_executed" or key == "binary_active_available":
                continue
            self.assertIs(value, False, key)

    def test_strict_json_rejects_duplicates_nonfinite_and_nonobject(self) -> None:
        temporary = self.temporary_dir()
        bad_documents = {
            "duplicate.json": '{"value": 1, "value": 2}\n',
            "nan.json": '{"value": NaN}\n',
            "infinity.json": '{"value": Infinity}\n',
            "overflow.json": '{"value": 1e999}\n',
            "array.json": "[]\n",
        }
        for name, document in bad_documents.items():
            with self.subTest(name=name):
                path = temporary / name
                path.write_text(document, encoding="utf-8")
                with self.assertRaises(subject.V25ShadowReadinessError):
                    subject.strict_json_load(path, subject.sha256_file(path))

    def test_exclusive_writer_is_finite_atomic_and_no_clobber(self) -> None:
        temporary = self.temporary_dir()
        destination = temporary / "receipt.json"
        payload = {"finite": 1.25, "ready": True}

        self.assertEqual(subject.write_json_exclusive(destination, payload), destination)
        original = destination.read_bytes()
        self.assertEqual(json.loads(original), payload)
        self.assertEqual(list(temporary.glob(f".{destination.name}.*.tmp")), [])

        with self.assertRaisesRegex(subject.V25ShadowReadinessError, "clobber"):
            subject.write_json_exclusive(destination, {"replacement": True})
        self.assertEqual(destination.read_bytes(), original)
        with self.assertRaises(subject.V25ShadowReadinessError):
            subject.write_json_exclusive(temporary / "nan.json", {"value": float("nan")})

    def test_exclusive_writer_rejects_dangling_symlink_and_link_race(self) -> None:
        temporary = self.temporary_dir()
        dangling = temporary / "dangling.json"
        try:
            dangling.symlink_to(temporary / "missing.json")
        except (OSError, NotImplementedError):
            self.skipTest("symlinks are unavailable on this platform")
        with self.assertRaisesRegex(subject.V25ShadowReadinessError, "clobber"):
            subject.write_json_exclusive(dangling, {"ready": True})
        self.assertTrue(dangling.is_symlink())

        raced = temporary / "raced.json"
        with patch.object(subject.os, "link", side_effect=FileExistsError):
            with self.assertRaisesRegex(subject.V25ShadowReadinessError, "clobber"):
                subject.write_json_exclusive(raced, {"ready": True})
        self.assertFalse(os.path.lexists(raced))
        self.assertEqual(list(temporary.glob(f".{raced.name}.*.tmp")), [])

    def test_freeze_path_is_exercised_only_against_temporary_destination(self) -> None:
        temporary = self.temporary_dir()
        destination = temporary / "shadow_receipt.json"
        with patch.object(subject, "DESTINATION", destination):
            receipt = subject.freeze_readiness()
        self.assertTrue(destination.is_file())
        self.assertEqual(
            subject.strict_json_load(destination),
            receipt,
        )
        self.assertFalse(subject.DESTINATION == destination)


if __name__ == "__main__":
    unittest.main()
