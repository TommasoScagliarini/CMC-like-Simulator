from __future__ import annotations

import hashlib
import json
import tempfile
import unittest
from pathlib import Path

import numpy as np

from validation import h0_primary_split_v6_qualification_contract as contract
from validation import (
    prepare_h0_primary_split_v6_qualification_noise_tapes as preparer,
)


class H0PrimarySplitV6NoiseTapeTests(unittest.TestCase):
    def _write_prerequisites(
        self,
        root: Path,
        *,
        candidate_id: str = "candidate-v6-test",
        development_candidate_id: str | None = None,
        freeze_status: str = contract.CANDIDATE_FREEZE_REQUIRED_STATUS,
        development_status: str = contract.DEVELOPMENT_PASS_REQUIRED_STATUS,
        development_passed: bool = True,
    ) -> tuple[Path, Path]:
        freeze = root / "candidate_freeze.json"
        development = root / "development_gate.json"
        freeze.write_text(
            json.dumps(
                {
                    "status": freeze_status,
                    "candidate_id": candidate_id,
                },
                allow_nan=False,
            ),
            encoding="utf-8",
        )
        development.write_text(
            json.dumps(
                {
                    "status": development_status,
                    "passed": development_passed,
                    "candidate_id": development_candidate_id or candidate_id,
                },
                allow_nan=False,
            ),
            encoding="utf-8",
        )
        return freeze, development

    def test_materializes_exact_five_float32_tapes_and_manifest(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            freeze, development = self._write_prerequisites(root)
            output = root / "tapes"
            manifest = preparer.prepare(
                output_root=output,
                candidate_freeze_receipt=freeze,
                development_gate_receipt=development,
            )

            self.assertEqual(manifest["status"], contract.NOISE_TAPES_STATUS)
            self.assertEqual(manifest["candidate_id"], "candidate-v6-test")
            self.assertEqual(len(manifest["cases"]), 6)
            self.assertEqual(len(manifest["tapes"]), 5)
            self.assertEqual(
                {path.name for path in output.iterdir()},
                {"manifest.json", *manifest["tapes"]},
            )
            self.assertFalse(manifest["qualification_execution_authorized_by_manifest"])
            self.assertEqual(manifest["protected_trials_opened"], [])
            self.assertEqual(manifest["actor_updates"], 0)
            self.assertEqual(manifest["critic_updates"], 0)
            self.assertEqual(manifest["ppo_updates"], 0)

            for filename, record in manifest["tapes"].items():
                with np.load(output / filename, allow_pickle=False) as archive:
                    tape = archive["standard_normal"]
                    self.assertEqual(tape.shape, (500, 2))
                    self.assertEqual(tape.dtype, np.float32)
                    self.assertTrue(np.all(np.isfinite(tape)))
                    self.assertEqual(
                        preparer.array_sha256(tape), record["array_sha256"]
                    )
                    if record["seed"] is None:
                        self.assertEqual(np.count_nonzero(tape), 0)
                        self.assertNotIn("seed", archive.files)
                    else:
                        self.assertEqual(archive["seed"].tolist(), [record["seed"]])

            raw_manifest = (output / "manifest.json").read_text(encoding="utf-8")
            self.assertNotIn("NaN", raw_manifest)
            self.assertNotIn("Infinity", raw_manifest)
            self.assertEqual(json.loads(raw_manifest), manifest)

    def test_stochastic_values_are_exactly_seeded_and_cases_share_as_frozen(
        self,
    ) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            freeze, development = self._write_prerequisites(root)
            output = root / "tapes"
            manifest = preparer.prepare(
                output_root=output,
                candidate_freeze_receipt=freeze,
                development_gate_receipt=development,
            )

            for seed in (130, 131, 132, 133):
                filename = f"stochastic_seed_{seed}_standard_normal.npz"
                expected = (
                    np.random.default_rng(seed)
                    .standard_normal((500, 2))
                    .astype(np.float32)
                )
                with np.load(output / filename, allow_pickle=False) as archive:
                    np.testing.assert_array_equal(archive["standard_normal"], expected)

            cases = manifest["cases"]
            self.assertEqual(cases[0]["noise_tape"], cases[1]["noise_tape"])
            self.assertEqual(
                [row["action_seed"] for row in cases[2:]],
                [130, 131, 132, 133],
            )

    def test_second_materialization_refuses_to_clobber_any_byte(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            freeze, development = self._write_prerequisites(root)
            output = root / "tapes"
            preparer.prepare(
                output_root=output,
                candidate_freeze_receipt=freeze,
                development_gate_receipt=development,
            )
            before = {
                path.name: hashlib.sha256(path.read_bytes()).hexdigest()
                for path in output.iterdir()
            }

            with self.assertRaisesRegex(
                preparer.QualificationNoiseTapeError, "clobber"
            ):
                preparer.prepare(
                    output_root=output,
                    candidate_freeze_receipt=freeze,
                    development_gate_receipt=development,
                )
            after = {
                path.name: hashlib.sha256(path.read_bytes()).hexdigest()
                for path in output.iterdir()
            }
            self.assertEqual(before, after)

    def test_prerequisite_failure_creates_no_destination(self) -> None:
        cases = (
            {
                "freeze_status": "NOT_FROZEN",
            },
            {
                "development_status": "FAIL_H0_PRIMARY_SPLIT_V6_DEVELOPMENT",
                "development_passed": False,
            },
            {
                "development_candidate_id": "different-candidate",
            },
        )
        for index, overrides in enumerate(cases):
            with self.subTest(index=index), tempfile.TemporaryDirectory() as temporary:
                root = Path(temporary)
                freeze, development = self._write_prerequisites(root, **overrides)
                output = root / "tapes"
                with self.assertRaises(preparer.QualificationNoiseTapeError):
                    preparer.prepare(
                        output_root=output,
                        candidate_freeze_receipt=freeze,
                        development_gate_receipt=development,
                    )
                self.assertFalse(output.exists())

    def test_strict_json_rejects_nonfinite_and_duplicate_keys_without_writes(
        self,
    ) -> None:
        invalid_freezes = (
            '{"status":"H0_PRIMARY_SPLIT_V6_CANDIDATE_FROZEN",'
            '"candidate_id":"x","value":NaN}',
            '{"status":"H0_PRIMARY_SPLIT_V6_CANDIDATE_FROZEN",'
            '"candidate_id":"x","candidate_id":"y"}',
        )
        for index, content in enumerate(invalid_freezes):
            with self.subTest(index=index), tempfile.TemporaryDirectory() as temporary:
                root = Path(temporary)
                freeze, development = self._write_prerequisites(root)
                freeze.write_text(content, encoding="utf-8")
                output = root / "tapes"
                with self.assertRaises(preparer.QualificationNoiseTapeError):
                    preparer.prepare(
                        output_root=output,
                        candidate_freeze_receipt=freeze,
                        development_gate_receipt=development,
                    )
                self.assertFalse(output.exists())


if __name__ == "__main__":
    unittest.main()
