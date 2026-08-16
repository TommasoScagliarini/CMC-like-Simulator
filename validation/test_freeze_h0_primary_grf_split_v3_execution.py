from __future__ import annotations

import hashlib
import json
import tempfile
import unittest
from contextlib import ExitStack
from pathlib import Path
from unittest import mock

from validation import freeze_h0_primary_grf_split_v3_execution as freezer
from validation import h0_primary_grf_split_v3_freeze_contract as contract


class PureFreezeContractTests(unittest.TestCase):
    def test_all_declared_paths_are_canonical_repository_relative_posix(self) -> None:
        maps = (
            contract.LOCK_SOURCE_RELATIVE_PATHS,
            contract.PROTOCOL_SOURCE_RELATIVE_PATHS,
            contract.RUNTIME_SOURCE_RELATIVE_PATHS,
            contract.LOCK_INPUT_RELATIVE_PATHS,
            contract.RUNTIME_INPUT_RELATIVE_PATHS,
            contract.EVIDENCE_RELATIVE_PATHS,
            contract.LINEAGE_RELATIVE_PATHS,
        )
        values = [value for mapping in maps for value in mapping.values()]
        values.extend(
            value
            for run in contract.diagnostic_artifact_relative_paths().values()
            for value in run.values()
        )
        for value in values:
            with self.subTest(value=value):
                pure = Path(*value.split("/"))
                self.assertTrue(value)
                self.assertFalse(value.startswith("/"))
                self.assertNotIn("\\", value)
                self.assertNotIn("..", pure.parts)
                self.assertEqual(pure.as_posix(), value)

    def test_closed_status0_policy_is_exposed_with_exact_limits(self) -> None:
        self.assertEqual(
            set(contract.SO_POLICIES),
            {
                contract.STRICT_ZERO_POLICY,
                contract.VERIFIED_SUCCESS_POLICY,
                contract.VERIFIED_STATUS0_MAX_ITER_POLICY,
            },
        )
        policy = contract.SO_POLICIES[contract.VERIFIED_STATUS0_MAX_ITER_POLICY]
        self.assertFalse(policy["bounded_lsq_success_required"])
        self.assertFalse(policy["solver_status_override_allowed"])
        self.assertTrue(policy["status_zero_without_success_allowed"])
        self.assertTrue(policy["max_iter_without_success_allowed"])
        self.assertEqual(policy["required_bounded_lsq_status"], 0)
        self.assertEqual(policy["required_bounded_lsq_iterations"], 1000)
        self.assertEqual(policy["bounded_lsq_optimality_max"], 1.0e-8)
        self.assertEqual(policy["bound_violation_max"], 1.0e-9)
        self.assertEqual(policy["feasibility_abs_or_max_tolerance"], 1.0e-6)
        self.assertEqual(policy["feasibility_relative_tolerance"], 1.0e-3)
        self.assertEqual(policy["residual_telemetry_consistency_atol"], 1.0e-12)
        self.assertTrue(policy["determinism_bit_exact_required"])
        self.assertFalse(policy["residual_contract_mismatch_allowed"])

    def test_diagnostics_are_exactly_two_replicates_of_train_seeds(self) -> None:
        paths = contract.diagnostic_artifact_relative_paths()
        self.assertEqual(
            set(paths),
            {
                f"seed_{seed}_rep{replicate}"
                for seed in contract.TRAIN_SEEDS
                for replicate in contract.DIAGNOSTIC_REPLICATES
            },
        )
        self.assertTrue(
            all(
                set(run) == set(contract.DIAGNOSTIC_FILENAMES) for run in paths.values()
            )
        )
        self.assertFalse(any("seed_125" in path for path in paths))

    def test_diagnostic_comparator_and_residual_audit_are_frozen(self) -> None:
        self.assertIn("diagnostic_comparator", contract.PROTOCOL_SOURCE_RELATIVE_PATHS)
        self.assertIn(
            "diagnostic_comparator_tests",
            contract.PROTOCOL_SOURCE_RELATIVE_PATHS,
        )
        self.assertIn(
            "validation.test_compare_h0_primary_grf_split_v3_diagnostics",
            contract.REQUIRED_TEST_MODULES,
        )
        self.assertIn("zero_residual_contract_mismatch", contract.DIAGNOSTIC_CHECK_KEYS)
        self.assertIn("zero_residual_contract_mismatch", contract.PREFLIGHT_CHECK_KEYS)


class _FreezeFixture:
    def __init__(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name).resolve()
        self.validation = self.root / "validation"
        self.validation.mkdir(parents=True)
        self.lock = self.validation / "execution_lock.json"
        self.run_root = self.validation / "run_root"
        self.stack = ExitStack()

        def artifact(relative: str, payload: bytes = b"fixture\n") -> Path:
            path = self.root / relative
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(payload)
            return path

        self.artifact = artifact
        self.lock_sources = {
            "protocol_plan": artifact("reports/plan.md"),
            "runner": artifact("validation/runner.py"),
            "tests": artifact("validation/test_runner.py"),
        }
        self.protocol_sources = {
            "execution_freezer": artifact("validation/freezer.py"),
            "execution_freezer_tests": artifact("validation/test_freezer.py"),
            "diagnostic_comparator": artifact("validation/comparator.py"),
            "so_recovery_contract": artifact("validation/so_contract.py"),
        }
        self.runtime_sources = {
            "simulation_runner": artifact("simulation_runner.py"),
            "static_optimization": artifact("static_optimization.py"),
            "environment": artifact("Trajectory Generator/environment.py"),
        }
        self.runtime_inputs = {
            "online_grf_macos_dylib": artifact("plugins/grf.dylib"),
            "sea_macos_dylib": artifact("plugins/sea.dylib"),
            "runtime_setup": artifact("models/setup.xml"),
        }
        self.lineage = {
            "v1_terminal_ledger": artifact("validation/v1_ledger.json"),
            "v2_terminal_ledger": artifact("validation/v2_ledger.json"),
            "initial_v3_decision_required_receipt": artifact(
                "validation/v3_initial.json"
            ),
        }
        self.h0_config = artifact("validation/h0/config.yaml")
        self.h0_module = self.root / "validation/h0/module"
        self.lock_inputs = {
            "h0_config": self.h0_config,
            "h0_module_state": artifact("validation/h0/module/module_state.pkl"),
            "h0_module_ctor": artifact("validation/h0/module/class_and_ctor_args.pkl"),
            "h0_module_metadata": artifact("validation/h0/module/metadata.json"),
        }
        self.histories: dict[int, Path] = {}
        for seed in freezer.runner.CANONICAL_SEEDS:
            directory = self.root / "validation" / f"history_{seed}"
            # Seed 125 is intentionally not valid JSON.  A successful freeze
            # therefore proves that the holdout was hashed but never parsed.
            content = b"not-json-holdout\n" if seed == 125 else b"[]\n"
            artifact(f"validation/history_{seed}/rollout_policy_trace.json", content)
            artifact(f"validation/history_{seed}/rollout_summary.json", content)
            self.histories[seed] = directory

        self.decision = self.validation / "decision.json"
        self.diagnostic = self.validation / "diagnostic.json"
        self.preflight = self.validation / "preflight.json"
        self.tests = self.validation / "tests.json"
        self.platform = self.validation / "platform.json"
        self.diagnostic_root = self.validation / "diagnostics"
        self.diagnostic_artifacts = {
            f"seed_{seed}_rep{replicate}": {
                filename: self.diagnostic_root
                / f"seed_{seed}_rep{replicate}"
                / filename
                for filename in contract.DIAGNOSTIC_FILENAMES
            }
            for seed in contract.TRAIN_SEEDS
            for replicate in contract.DIAGNOSTIC_REPLICATES
        }
        self.historical_inputs = {
            str(seed): {
                "trace": directory / "rollout_policy_trace.json",
                "summary": directory / "rollout_summary.json",
            }
            for seed, directory in self.histories.items()
        }
        self.evidence_paths = {
            "so_policy_decision": self.decision,
            "diagnostic_determinism": self.diagnostic,
            "instrumented_preflight": self.preflight,
            "preflight_tests": self.tests,
            "platform": self.platform,
        }
        self.identity = {
            "system": "Darwin",
            "machine": "arm64",
            "python_version": "3.test",
            "python_implementation": "CPython",
            "python_executable": "/test/python",
            "distributions": {
                name: "test"
                for name in (
                    "numpy",
                    "scipy",
                    "torch",
                    "ray",
                    "gymnasium",
                    "opensim",
                )
            },
        }

        patches = (
            mock.patch.object(freezer, "REPO_ROOT", self.root),
            mock.patch.object(freezer, "VALIDATION_ROOT", self.validation),
            mock.patch.object(freezer, "LOCK", self.lock),
            mock.patch.object(freezer, "RUN_ROOT", self.run_root),
            mock.patch.object(freezer, "DECISION_RECEIPT", self.decision),
            mock.patch.object(freezer, "DIAGNOSTIC_RECEIPT", self.diagnostic),
            mock.patch.object(freezer, "PREFLIGHT_RECEIPT", self.preflight),
            mock.patch.object(freezer, "TEST_RECEIPT", self.tests),
            mock.patch.object(freezer, "PLATFORM_RECEIPT", self.platform),
            mock.patch.object(freezer, "DIAGNOSTIC_ROOT", self.diagnostic_root),
            mock.patch.object(
                freezer, "DIAGNOSTIC_ARTIFACT_PATHS", self.diagnostic_artifacts
            ),
            mock.patch.object(freezer, "LOCK_SOURCE_PATHS", self.lock_sources),
            mock.patch.object(freezer, "PROTOCOL_SOURCE_PATHS", self.protocol_sources),
            mock.patch.object(freezer, "RUNTIME_SOURCE_PATHS", self.runtime_sources),
            mock.patch.object(freezer, "LOCK_INPUT_PATHS", self.lock_inputs),
            mock.patch.object(freezer, "RUNTIME_INPUT_PATHS", self.runtime_inputs),
            mock.patch.object(
                freezer, "HISTORICAL_INPUT_PATHS", self.historical_inputs
            ),
            mock.patch.object(freezer, "EVIDENCE_PATHS", self.evidence_paths),
            mock.patch.object(freezer, "LINEAGE_PATHS", self.lineage),
            mock.patch.object(
                freezer, "_live_platform_identity", return_value=self.identity
            ),
            mock.patch.object(freezer.runner, "LOCK", self.lock),
            mock.patch.object(freezer.runner, "RUN_ROOT", self.run_root),
            mock.patch.object(
                freezer.runner, "PLAN", self.lock_sources["protocol_plan"]
            ),
            mock.patch.object(freezer.runner, "RUNNER", self.lock_sources["runner"]),
            mock.patch.object(freezer.runner, "TESTS", self.lock_sources["tests"]),
            mock.patch.object(freezer.runner, "H0_CONFIG", self.h0_config),
            mock.patch.object(freezer.runner, "H0_MODULE", self.h0_module),
            mock.patch.object(
                freezer.runner,
                "_historical_directory",
                side_effect=lambda seed: self.histories[seed],
            ),
            mock.patch.object(freezer.runner, "verify_lock", return_value={}),
            mock.patch.object(
                freezer.runner,
                "historical_inputs",
                side_effect=AssertionError("holdout semantic parser was called"),
            ),
        )
        for patcher in patches:
            self.stack.enter_context(patcher)

        self._write_lineage()
        self._write_evidence()

    @staticmethod
    def _write_json(path: Path, payload: dict) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(
            json.dumps(payload, indent=2, sort_keys=True, allow_nan=False) + "\n",
            encoding="utf-8",
        )

    @staticmethod
    def _footer() -> dict:
        return {
            "actor_updates": 0,
            "critic_updates": 0,
            "ppo_updates": 0,
            "protected_trials_opened": [],
        }

    def _write_lineage(self) -> None:
        self._write_json(
            self.lineage["v1_terminal_ledger"],
            {
                "status": "ERROR_PRIMARY_SPLIT_TEACHER",
                "passed": False,
                "next_stage": "STOP_WITHOUT_RETRY_OR_RETUNING",
                "protected_trials_opened": [],
            },
        )
        self._write_json(
            self.lineage["v2_terminal_ledger"],
            {
                "status": "ERROR_H0_PRIMARY_SPLIT_V2_TEACHER",
                "passed": False,
                "next_stage": "STOP_WITHOUT_RETRY_OR_RETUNING",
                "protected_trials_opened": [],
            },
        )
        self._write_json(
            self.lineage["initial_v3_decision_required_receipt"],
            {
                "status": "H0_PRIMARY_SPLIT_V3_PREFLIGHT_GATE_DECISION_REQUIRED",
                "passed": False,
                "actor_updates": 0,
                "ppo_updates": 0,
                "protected_trials_opened": [],
            },
        )

    def _write_evidence(self) -> None:
        policy_id = freezer.VERIFIED_RECOVERY_POLICY
        self._write_json(
            self.decision,
            {
                "schema_version": 3,
                "status": "H0_PRIMARY_SPLIT_V3_SO_POLICY_DECIDED",
                "passed": True,
                "protocol_id": freezer.runner.PROTOCOL_ID,
                "revision": freezer.runner.REVISION,
                "selected_policy_id": policy_id,
                "policy": freezer.SO_POLICIES[policy_id],
                "authority_basis": "explicit_user_authorization",
                "policy_applies_to_future_v3_execution": True,
                "actor_update_candidate_count": 1,
                **self._footer(),
            },
        )
        artifacts: dict[str, dict] = {}
        for run_id, paths in freezer._diagnostic_artifact_paths().items():
            artifacts[run_id] = {}
            for filename, path in paths.items():
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_bytes(f"{run_id}:{filename}\n".encode())
                artifacts[run_id][filename] = freezer._record(path)
        diagnostic_sources = {
            "runner": freezer.runner.RUNNER,
            "diagnostic_comparator": self.protocol_sources[
                "diagnostic_comparator"
            ],
            "so_recovery_contract": self.protocol_sources["so_recovery_contract"],
            "simulation_runner": self.runtime_sources["simulation_runner"],
            "static_optimization": self.runtime_sources["static_optimization"],
            "environment": self.runtime_sources["environment"],
        }
        self._write_json(
            self.diagnostic,
            {
                "schema_version": 3,
                "status": "PASS_H0_PRIMARY_SPLIT_V3_DIAGNOSTIC_DETERMINISM",
                "passed": True,
                "protocol_id": freezer.runner.PROTOCOL_ID,
                "revision": freezer.runner.REVISION,
                "so_policy_id": policy_id,
                "train_seeds": list(freezer.runner.TRAIN_SEEDS),
                "replicates_per_seed": 2,
                "checks": {key: True for key in freezer.DIAGNOSTIC_CHECK_KEYS},
                "sources": {
                    key: freezer._record(path)
                    for key, path in diagnostic_sources.items()
                },
                "artifacts": artifacts,
                **self._footer(),
            },
        )
        self._write_json(
            self.preflight,
            {
                "schema_version": 3,
                "status": "PASS_H0_PRIMARY_SPLIT_V3_PREFLIGHT",
                "passed": True,
                "protocol_id": freezer.runner.PROTOCOL_ID,
                "revision": freezer.runner.REVISION,
                "so_policy_id": policy_id,
                "checks": {key: True for key in freezer.PREFLIGHT_CHECK_KEYS},
                "decision_receipt": freezer._record(self.decision),
                "diagnostic_receipt": freezer._record(self.diagnostic),
                **self._footer(),
            },
        )
        tested_sources = {**self.lock_sources, **self.protocol_sources}
        self._write_json(
            self.tests,
            {
                "schema_version": 3,
                "status": "PASS_H0_PRIMARY_SPLIT_V3_PREFLIGHT_TESTS",
                "passed": True,
                "protocol_id": freezer.runner.PROTOCOL_ID,
                "revision": freezer.runner.REVISION,
                "test_modules": list(freezer.REQUIRED_TEST_MODULES),
                "checks": {key: True for key in freezer.TEST_CHECK_KEYS},
                "tested_sources": {
                    key: freezer._record(path) for key, path in tested_sources.items()
                },
                **self._footer(),
            },
        )
        self._write_json(
            self.platform,
            {
                "schema_version": 3,
                "status": "PASS_H0_PRIMARY_SPLIT_V3_PLATFORM",
                "passed": True,
                "protocol_id": freezer.runner.PROTOCOL_ID,
                "revision": freezer.runner.REVISION,
                "numerical_claim": "macOS-arm64-only",
                "windows_claim": "schema-and-path-compatibility-only",
                "identity": self.identity,
                "binary_artifacts": {
                    key: freezer._record(self.runtime_inputs[key])
                    for key in (
                        "online_grf_macos_dylib",
                        "sea_macos_dylib",
                    )
                },
                **self._footer(),
            },
        )

    def close(self) -> None:
        self.stack.close()
        self.temporary.cleanup()


class FreezeH0PrimarySplitV3Tests(unittest.TestCase):
    def setUp(self) -> None:
        self.fixture = _FreezeFixture()

    def tearDown(self) -> None:
        self.fixture.close()

    def test_freezer_creates_only_lock_and_hashes_holdout_without_parsing(self) -> None:
        before = {
            path.relative_to(self.fixture.root)
            for path in self.fixture.root.rglob("*")
            if path.is_file()
        }
        payload = freezer.freeze()
        after = {
            path.relative_to(self.fixture.root)
            for path in self.fixture.root.rglob("*")
            if path.is_file()
        }
        self.assertEqual(
            after - before, {self.fixture.lock.relative_to(self.fixture.root)}
        )
        self.assertFalse(self.fixture.run_root.exists())
        self.assertEqual(
            payload["so_policy_id"], contract.VERIFIED_STATUS0_MAX_ITER_POLICY
        )
        self.assertEqual(
            payload["so_policy"],
            contract.SO_POLICIES[contract.VERIFIED_STATUS0_MAX_ITER_POLICY],
        )
        holdout = payload["inputs"]["historical_inputs"]["125"]["trace"]
        holdout_path = self.fixture.histories[125] / "rollout_policy_trace.json"
        self.assertEqual(
            holdout["sha256"], hashlib.sha256(holdout_path.read_bytes()).hexdigest()
        )
        freezer.runner.verify_lock.assert_called_once_with()

    def test_missing_decision_fails_without_creating_lock_or_run_root(self) -> None:
        self.fixture.decision.unlink()
        with self.assertRaises(freezer.V3FreezeError):
            freezer.freeze()
        self.assertFalse(self.fixture.lock.exists())
        self.assertFalse(self.fixture.run_root.exists())

    def test_policy_mismatch_fails_closed_before_publication(self) -> None:
        receipt = json.loads(self.fixture.diagnostic.read_text(encoding="utf-8"))
        receipt["so_policy_id"] = freezer.STRICT_ZERO_POLICY
        self.fixture._write_json(self.fixture.diagnostic, receipt)
        with self.assertRaises(freezer.V3FreezeError):
            freezer.freeze()
        self.assertFalse(self.fixture.lock.exists())
        self.assertFalse(self.fixture.run_root.exists())

    def test_existing_lock_is_never_clobbered(self) -> None:
        sentinel = b"do-not-clobber\n"
        self.fixture.lock.write_bytes(sentinel)
        with self.assertRaises(freezer.V3FreezeError):
            freezer.freeze()
        self.assertEqual(self.fixture.lock.read_bytes(), sentinel)
        self.assertFalse(self.fixture.run_root.exists())

    def test_source_mutation_after_test_receipt_fails_closed(self) -> None:
        self.fixture.protocol_sources["execution_freezer"].write_bytes(b"drift\n")
        with self.assertRaises(freezer.V3FreezeError):
            freezer.freeze()
        self.assertFalse(self.fixture.lock.exists())
        self.assertFalse(self.fixture.run_root.exists())


if __name__ == "__main__":
    unittest.main()
