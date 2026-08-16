"""Tests for the no-clobber correction of the rejected H0/V25 lock."""

from __future__ import annotations

import copy
import os
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch


REPO_ROOT = Path(__file__).resolve().parents[1]
VALIDATION_ROOT = REPO_ROOT / "validation"
if str(VALIDATION_ROOT) not in os.sys.path:
    os.sys.path.insert(0, str(VALIDATION_ROOT))

import freeze_h0_v25_abc_protocol_corrected as subject  # noqa: E402


class CorrectedH0V25ProtocolTests(unittest.TestCase):
    def temporary_dir(self) -> Path:
        temporary = tempfile.TemporaryDirectory(dir=VALIDATION_ROOT)
        self.addCleanup(temporary.cleanup)
        return Path(temporary.name)

    def payload(self) -> dict:
        destination = self.temporary_dir() / "corrected.json"
        with patch.object(subject, "DESTINATION", destination):
            return subject.build_corrected_payload()

    def test_rejected_lock_is_exact_preserved_closed_and_reproducible(self) -> None:
        old_bytes = subject.REJECTED_LOCK.read_bytes()
        payload = self.payload()
        supersedes = payload["supersedes"]
        self.assertEqual(
            supersedes["rejected_lock"]["sha256"],
            subject.REJECTED_LOCK_SHA256,
        )
        self.assertFalse(supersedes["rejected_lock_execution_authorized"])
        self.assertFalse(supersedes["rejected_lock_h0_executed"])
        self.assertFalse(supersedes["rejected_lock_used_for_execution"])
        self.assertTrue(supersedes["rejected_lock_must_not_be_used"])
        self.assertEqual(subject.REJECTED_LOCK.read_bytes(), old_bytes)

    def test_a_b_cases_differ_only_by_label_and_shadow_execution_mode(self) -> None:
        payload = self.payload()
        case_a = copy.deepcopy(payload["matrix"]["cases"]["A"])
        case_b = copy.deepcopy(payload["matrix"]["cases"]["B"])
        self.assertNotIn("scientific_bundle_contract_id", case_a)
        self.assertNotIn("scientific_bundle_contract_id", case_b)
        self.assertEqual(case_a.pop("case_name"), "legacy_control")
        self.assertEqual(case_b.pop("case_name"), "binary_shadow")
        self.assertEqual(case_a.pop("binary_phase_fsm_mode"), "disabled")
        self.assertEqual(case_b.pop("binary_phase_fsm_mode"), "binary_shadow")
        self.assertEqual(case_a, case_b)
        self.assertEqual(
            payload["contracts"]["scientific_shadow_bundle_contract_id"],
            subject.rejected.shadow.SCIENTIFIC_SHADOW_BUNDLE_ID,
        )

    def test_corrected_protocol_keeps_matrix_authority_and_scope_closed(self) -> None:
        payload = self.payload()
        self.assertEqual(payload["schema_version"], 2)
        self.assertEqual(payload["protocol_id"], subject.PROTOCOL_ID)
        self.assertEqual(payload["date"], subject.CORRECTION_DATE)
        self.assertEqual(
            payload["supersedes"]["original_protocol_date"],
            subject.ORIGINAL_PROTOCOL_DATE,
        )
        self.assertEqual(
            payload["correction"]["correction_date"], subject.CORRECTION_DATE
        )
        self.assertFalse(payload["protocol_executed"])
        self.assertEqual(payload["matrix"]["protocol_unit_count"], 12)
        self.assertEqual(payload["matrix"]["underlying_rollout_count"], 18)
        self.assertTrue(all(payload["assertions"].values()))
        self.assertTrue(all(value is False for value in payload["authority"].values()))
        self.assertTrue(
            all(
                value is False
                for value in payload["unmet_execution_prerequisites"].values()
            )
        )

    def test_preflight_is_read_only_and_writer_is_no_clobber(self) -> None:
        temporary = self.temporary_dir()
        destination = temporary / "corrected.json"
        rejected_before = subject.REJECTED_LOCK.read_bytes()
        with patch.object(subject, "DESTINATION", destination):
            preflight = subject.preflight_unfrozen()
            self.assertFalse(destination.exists())
            frozen = subject.freeze_corrected_protocol()
            original = destination.read_bytes()
            with self.assertRaisesRegex(
                subject.H0V25CorrectedProtocolError, "clobber"
            ):
                subject.freeze_corrected_protocol()
        self.assertEqual(
            preflight["lock_record_if_frozen"],
            subject.shadow.payload_record(destination, frozen),
        )
        self.assertEqual(destination.read_bytes(), original)
        self.assertEqual(subject.REJECTED_LOCK.read_bytes(), rejected_before)


if __name__ == "__main__":
    unittest.main()
