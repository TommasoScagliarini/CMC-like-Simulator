from __future__ import annotations

import ast
import hashlib
import inspect
import tempfile
import unittest
from pathlib import Path
from unittest import mock

from validation import h0_forensic_rollout as forensic
from validation import h0_primary_split_v6_qualification_contract as contract
from validation import h0_primary_split_v6_residual_dagger_contract as p1_contract
from validation import run_h0_primary_split_v6_qualification as runner


def _module_tree(root: Path, marker: bytes) -> None:
    root.mkdir(parents=True)
    for filename in (
        "module_state.pkl",
        "class_and_ctor_args.pkl",
        "metadata.json",
    ):
        (root / filename).write_bytes(marker + filename.encode("ascii"))


class QualificationRunnerTests(unittest.TestCase):
    def test_matrix_is_baseline_first_and_exact(self) -> None:
        expected = tuple(
            (role, case_id)
            for role in (contract.BASELINE_ROLE, contract.CANDIDATE_ROLE)
            for case_id in contract.CASE_IDS
        )
        self.assertEqual(runner.MATRIX, expected)
        self.assertEqual(len(runner.MATRIX), 12)
        self.assertTrue(
            all(role == contract.BASELINE_ROLE for role, _case in runner.MATRIX[:6])
        )

    def test_candidate_freeze_tree_and_development_are_same_p1(self) -> None:
        validation_root = runner.REPO_ROOT / "validation"
        with tempfile.TemporaryDirectory(
            prefix="v6_qualification_prereq_test_", dir=validation_root
        ) as temporary:
            root = Path(temporary)
            source = root / "source_h0"
            candidate = root / "candidate_p1"
            _module_tree(source, b"source-")
            _module_tree(candidate, b"candidate-")
            config = root / "training.yaml"
            analog = root / "analog.json"
            v25 = root / "v25.json"
            for path in (config, analog, v25):
                path.write_text("{}\n", encoding="utf-8")
            freeze_path = root / "candidate_freeze.json"
            development_path = root / "development.json"
            candidate_tree = runner._tree_record(candidate)  # noqa: SLF001
            candidate_id = (
                "H0_PRIMARY_SPLIT_V6_P1_"
                f"{candidate_tree['tree_sha256'][:16]}"
            )
            freeze = {
                "schema_version": contract.SCHEMA_VERSION,
                "status": contract.CANDIDATE_FREEZE_REQUIRED_STATUS,
                "passed": True,
                "protocol_id": p1_contract.PROTOCOL_ID,
                "stage_id": "freeze_p1",
                "candidate_id": candidate_id,
                "candidate_module": candidate_tree,
                "source_h0": runner._tree_record(source),  # noqa: SLF001
                "target_contract_id": p1_contract.TARGET_CONTRACT_ID,
                "fit": {
                    "p0": dict(p1_contract.P0_FIT),
                    "p1": dict(p1_contract.P1_FIT),
                },
                "dagger_rounds": 1,
                "dagger_case_ids": list(p1_contract.DAGGER_CASE_IDS),
                "p0_promotable": False,
                "p1_unique_final_candidate": True,
                "base_h0_byte_exact": True,
                "critic_byte_exact": True,
                "logstd_byte_exact": True,
                "retry_authorized": False,
                "actor_updates": 2,
                "critic_updates": 0,
                "ppo_updates": 0,
                "protected_trials_opened": [],
            }
            forensic.write_json_exclusive(freeze_path, freeze)
            development = {
                "schema_version": contract.SCHEMA_VERSION,
                "status": contract.DEVELOPMENT_PASS_REQUIRED_STATUS,
                "passed": True,
                "protocol_id": p1_contract.PROTOCOL_ID,
                "stage_id": "finalize_development",
                "candidate_id": candidate_id,
                "candidate_freeze": runner._record(freeze_path),  # noqa: SLF001
                "case_ids": list(p1_contract.DEVELOPMENT_CASE_IDS),
                "same_six_cases_as_teacher_replay": True,
                "development_only": True,
                "qualification_eligible": True,
                "retry_authorized": False,
                "dagger_rounds": 1,
                "actor_updates": 0,
                "critic_updates": 0,
                "ppo_updates": 0,
                "protected_trials_opened": [],
            }
            forensic.write_json_exclusive(development_path, development)
            patches = (
                mock.patch.object(runner, "CANDIDATE_FREEZE", freeze_path),
                mock.patch.object(runner, "DEVELOPMENT_GATE", development_path),
                mock.patch.object(runner, "SOURCE_H0_CONFIG", config),
                mock.patch.object(runner, "ANALOG_PROFILE", analog),
                mock.patch.object(runner, "V25_PROFILE", v25),
            )
            with patches[0], patches[1], patches[2], patches[3], patches[4]:
                result = runner._validate_prerequisites()  # noqa: SLF001
                self.assertEqual(result["candidate_id"], candidate_id)
                self.assertEqual(result["candidate_module_path"], candidate.resolve())
                self.assertEqual(result["source_h0_path"], source.resolve())
                (candidate / "module_state.pkl").write_bytes(b"tampered")
                with self.assertRaises(runner.QualificationExecutionError):
                    runner._validate_prerequisites()  # noqa: SLF001

    def test_execution_claim_contains_token_hash_not_plaintext(self) -> None:
        token = "supervisor-secret-" + "x" * 40
        token_hash = hashlib.sha256(token.encode("utf-8")).hexdigest()
        fake_prerequisites = {
            "candidate_id": "P1_ID",
            "candidate_module_path": Path("candidate"),
            "source_h0_path": Path("source"),
        }
        fake_noise = {
            "case_tapes": {
                case_id: {
                    "path": Path(f"{case_id}.npz"),
                    "array_sha256": "a" * 64,
                }
                for case_id in contract.CASE_IDS
            }
        }

        def fake_record(path: object) -> dict[str, object]:
            return {"path": str(path), "sha256": "b" * 64, "size_bytes": 1}

        with (
            mock.patch.object(
                runner, "_validate_prerequisites", return_value=fake_prerequisites
            ),
            mock.patch.object(
                runner, "_validate_noise_manifest", return_value=fake_noise
            ),
            mock.patch.object(
                runner,
                "_tree_record",
                side_effect=lambda path: {"path": str(path), "tree_sha256": "c" * 64},
            ),
            mock.patch.object(runner, "_record", side_effect=fake_record),
            mock.patch.object(runner, "_source_records", return_value={}),
        ):
            claim = runner._execution_claim_payload(token_hash)  # noqa: SLF001
        encoded = forensic.canonical_json_bytes(claim)
        self.assertEqual(claim["execution_token_sha256"], token_hash)
        self.assertNotIn(token.encode("utf-8"), encoded)
        candidate_role = claim["role_contracts"][1]
        self.assertEqual(candidate_role["actor_id"], "P1_ID")

    def test_forensic_aggregates_precede_gate_and_receipt_in_source(self) -> None:
        source = inspect.getsource(runner._execute_rollout)  # noqa: SLF001
        self.assertLess(source.index("finalize_before_gate"), source.index("writer.run_gate"))
        self.assertLess(source.index("writer.run_gate"), source.index('"receipt.json"'))

    def test_import_surface_has_no_heavy_runtime_module(self) -> None:
        tree = ast.parse(Path(runner.__file__).read_text(encoding="utf-8"))
        top_level_imports: set[str] = set()
        for node in tree.body:
            if isinstance(node, ast.Import):
                top_level_imports.update(alias.name for alias in node.names)
            elif isinstance(node, ast.ImportFrom) and node.module:
                top_level_imports.add(node.module)
        forbidden = {
            "opensim",
            "numpy",
            "torch",
            "ray",
            "run_h0_v25_abc_preflight",
            "run_h0_primary_grf_split_v1_adaptation",
        }
        self.assertTrue(top_level_imports.isdisjoint(forbidden))

    def test_claimed_failure_publishes_terminal_ledger_and_blocks_retry(self) -> None:
        validation_root = runner.REPO_ROOT / "validation"
        with tempfile.TemporaryDirectory(
            prefix="v6_qualification_terminal_test_", dir=validation_root
        ) as temporary:
            parent = Path(temporary)
            run_root = parent / "run"
            claim_path = run_root / "execution_claim.json"
            ledger_path = run_root / "execution_ledger.json"
            claim = {
                "candidate_id": "P1_ID",
                "execution_token_sha256": "a" * 64,
            }
            with (
                mock.patch.object(runner, "RUN_ROOT", run_root),
                mock.patch.object(runner, "EXECUTION_CLAIM", claim_path),
                mock.patch.object(runner, "EXECUTION_LEDGER", ledger_path),
                mock.patch.object(
                    runner, "_execution_claim_payload", return_value=claim
                ),
                mock.patch.object(runner, "_publish_worker_claim"),
                mock.patch.object(
                    runner,
                    "_run_worker_process",
                    side_effect=runner.QualificationExecutionError("synthetic worker stop"),
                ),
            ):
                with self.assertRaises(runner.QualificationExecutionError):
                    runner.execute()
                ledger = forensic.strict_json_load(ledger_path)
                self.assertEqual(ledger["status"], contract.FAIL_STATUS)
                self.assertIs(ledger["passed"], False)
                self.assertIs(ledger["retry_authorized"], False)
                with self.assertRaises(runner.QualificationExecutionError):
                    runner.execute()


if __name__ == "__main__":
    unittest.main()
