"""Read-only pre-freeze tests for the V25 development-candidate receipt.

The suite does not import OpenSim and never invokes the ``--freeze`` CLI path.
It validates the pinned V25 evidence, the closed post-freeze authority, and the
exclusive writer only against temporary destinations.
"""

from __future__ import annotations

import builtins
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

import freeze_binary_phase_detector_v25_development_candidate as subject  # noqa: E402


class V25DevelopmentCandidateFreezeTests(unittest.TestCase):
    def _temporary_dir(self) -> Path:
        temporary = tempfile.TemporaryDirectory(dir=VALIDATION_ROOT)
        self.addCleanup(temporary.cleanup)
        return Path(temporary.name)

    def test_preflight_is_read_only_and_every_pin_is_exact(self) -> None:
        temporary = self._temporary_dir()
        fake_destination = temporary / "candidate_freeze.json"
        before = {
            label: (path.read_bytes(), subject.sha256_file(path))
            for label, (path, _) in subject.PINNED.items()
        }
        real_import = builtins.__import__

        def guarded_import(name, globals=None, locals=None, fromlist=(), level=0):
            if name == "opensim" or name.startswith("opensim."):
                raise AssertionError("pre-freeze check attempted to import OpenSim")
            return real_import(name, globals, locals, fromlist, level)

        with patch("builtins.__import__", side_effect=guarded_import), patch.object(
            subject, "DESTINATION", fake_destination
        ):
            payload = subject.preflight_unfrozen()

        self.assertEqual(
            payload["status"], "V25_DEVELOPMENT_CANDIDATE_FREEZE_READY_UNFROZEN"
        )
        self.assertTrue(all(payload["assertions"].values()))
        self.assertEqual(payload["candidate"]["candidate_id"], subject.CANDIDATE_ID)
        self.assertEqual(payload["candidate"]["heel_reach_m"], 0.025)
        self.assertEqual(payload["candidate"]["toe_reach_m"], 0.0275)
        self.assertEqual(len(payload["manifest_output_files"]), 7)

        for label, (path, expected_sha) in subject.PINNED.items():
            with self.subTest(label=label):
                self.assertEqual(payload["sources"][label]["sha256"], expected_sha)
                self.assertEqual(subject.sha256_file(path), expected_sha)
                self.assertEqual(before[label], (path.read_bytes(), expected_sha))

    def test_receipt_keeps_h0_protected_data_and_training_unauthorized(self) -> None:
        temporary = self._temporary_dir()
        fake_destination = temporary / "candidate_freeze.json"
        captured: dict[str, object] = {}

        def capture(path: Path, payload) -> Path:
            captured["path"] = path
            captured["payload"] = dict(payload)
            return path

        with patch.object(subject, "DESTINATION", fake_destination), patch.object(
            subject, "_write_json_exclusive", side_effect=capture
        ):
            receipt = subject.freeze_candidate()

        self.assertEqual(captured["path"], fake_destination)
        self.assertEqual(captured["payload"], receipt)
        self.assertFalse(fake_destination.exists())
        self.assertEqual(
            receipt["status"],
            "V25_DEVELOPMENT_CANDIDATE_FROZEN_H0_PROTOCOL_REQUIRED",
        )
        self.assertEqual(
            receipt["next_stage"],
            "FREEZE_H0_A_B_C_PROTOCOL_BEFORE_ANY_H0_EXECUTION",
        )
        self.assertEqual(receipt["data_governance"]["protected_trials_opened"], [])
        self.assertEqual(
            receipt["data_governance"]["protected_trials_remaining_closed"],
            ["05", "06"],
        )
        self.assertEqual(receipt["data_governance"]["reserve_trials_opened"], [])
        self.assertEqual(
            receipt["data_governance"]["reserve_trials_remaining_closed"],
            ["03", "07"],
        )
        scope = receipt["post_pass_scope"]
        self.assertTrue(scope["dormant_shadow_integration_implementation_allowed"])
        self.assertTrue(scope["h0_protocol_freeze_allowed"])
        for key in (
            "h0_execution_allowed",
            "protected_trial_access_allowed",
            "runtime_promotion_allowed",
            "training_allowed",
            "corridor_activation_allowed",
            "positive_morphology_reward_ppo_allowed",
        ):
            self.assertIs(scope[key], False, key)

    def test_exclusive_writer_succeeds_once_then_refuses_clobber(self) -> None:
        temporary = self._temporary_dir()
        destination = temporary / "receipt.json"
        payload = {"finite": 1.25, "pass": True}

        self.assertEqual(
            subject._write_json_exclusive(destination, payload), destination
        )
        original = destination.read_bytes()
        self.assertEqual(json.loads(original), payload)
        self.assertEqual(list(temporary.glob(f".{destination.name}.*.tmp")), [])

        with self.assertRaisesRegex(subject.V25CandidateFreezeError, "clobber"):
            subject._write_json_exclusive(destination, {"replacement": True})
        self.assertEqual(destination.read_bytes(), original)

    def test_exclusive_writer_rejects_dangling_symlink_and_link_race(self) -> None:
        temporary = self._temporary_dir()
        symlink = temporary / "dangling.json"
        try:
            symlink.symlink_to(temporary / "missing-target.json")
        except (OSError, NotImplementedError):
            self.skipTest("symlinks are unavailable on this platform")
        self.assertTrue(os.path.lexists(symlink))
        with self.assertRaisesRegex(subject.V25CandidateFreezeError, "clobber"):
            subject._write_json_exclusive(symlink, {"pass": True})
        self.assertTrue(symlink.is_symlink())

        raced = temporary / "raced.json"
        with patch.object(subject.os, "link", side_effect=FileExistsError):
            with self.assertRaisesRegex(subject.V25CandidateFreezeError, "clobber"):
                subject._write_json_exclusive(raced, {"pass": True})
        self.assertFalse(os.path.lexists(raced))
        self.assertEqual(list(temporary.glob(f".{raced.name}.*.tmp")), [])

    def test_strict_json_rejects_nonfinite_constants_and_non_object_roots(self) -> None:
        temporary = self._temporary_dir()
        for name, text in (
            ("nan.json", '{"value": NaN}\n'),
            ("infinity.json", '{"value": Infinity}\n'),
            ("array.json", "[]\n"),
        ):
            with self.subTest(name=name):
                path = temporary / name
                path.write_text(text, encoding="utf-8")
                with self.assertRaises(subject.V25CandidateFreezeError):
                    subject._strict_json(path, subject.sha256_file(path))


if __name__ == "__main__":
    unittest.main()
