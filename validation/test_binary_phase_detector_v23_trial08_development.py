"""Regression tests for the V23 assertions/facts development correction.

No test executes detector sampling or the development executor.  The suite
binds the already-open canonical oracle, proves that descriptive false facts
cannot poison the positive assertion gate, and audits the fail-closed V23
protocol before its immutable freeze is published.
"""

from __future__ import annotations

import ast
import copy
import importlib
import json
import os
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch


REPO_ROOT = Path(__file__).resolve().parents[1]
VALIDATION_ROOT = REPO_ROOT / "validation"
if str(VALIDATION_ROOT) not in sys.path:
    sys.path.insert(0, str(VALIDATION_ROOT))

import freeze_binary_phase_detector_v23_trial08_development as freeze  # noqa: E402


RUNNER_MODULE = "validate_binary_phase_detector_v23_trial08_development"
RUNNER_PATH = VALIDATION_ROOT / f"{RUNNER_MODULE}.py"


class V23DevelopmentRegressionTests(unittest.TestCase):
    def _temporary_dir(self) -> Path:
        temporary = tempfile.TemporaryDirectory(dir=VALIDATION_ROOT)
        self.addCleanup(temporary.cleanup)
        return Path(temporary.name)

    def _oracle(self) -> tuple[dict, dict]:
        self.assertEqual(freeze.sha256_file(freeze.ORACLE_PATH), freeze.ORACLE_SHA256)
        ledger = json.loads(freeze.ORACLE_PATH.read_text(encoding="utf-8"))
        self.assertEqual(
            ledger["scientific_core_sha256"], freeze.ORACLE_CORE_SHA256
        )
        return ledger, ledger["scientific_core"]

    def _runner(self):
        self.assertTrue(RUNNER_PATH.is_file(), "V23 runner source is missing")
        importlib.invalidate_caches()
        return importlib.import_module(RUNNER_MODULE)

    @staticmethod
    def _cli_options(path: Path) -> set[str]:
        tree = ast.parse(path.read_text(encoding="utf-8"))
        options: set[str] = set()
        for node in ast.walk(tree):
            if not isinstance(node, ast.Call):
                continue
            if not isinstance(node.func, ast.Attribute):
                continue
            if node.func.attr != "add_argument":
                continue
            for argument in node.args:
                if (
                    isinstance(argument, ast.Constant)
                    and isinstance(argument.value, str)
                    and argument.value.startswith("-")
                ):
                    options.add(argument.value)
        return options

    def test_v21_and_v22_terminals_are_hash_exact_and_immutable(self) -> None:
        generations = (("V21", freeze.V21_TERMINAL), ("V22", freeze.V22_TERMINAL))
        for generation, declarations in generations:
            with self.subTest(generation=generation):
                before: dict[str, bytes] = {}
                for label, expected in declarations.items():
                    path = REPO_ROOT / expected["path"]
                    self.assertTrue(path.is_file(), f"{generation}.{label}")
                    self.assertEqual(
                        freeze.sha256_file(path),
                        expected["sha256"],
                        f"{generation}.{label}",
                    )
                    before[label] = path.read_bytes()

                verification = freeze._verify_terminal(
                    declarations, generation=generation
                )
                self.assertTrue(all(verification["checks"].values()))
                self.assertTrue(verification["checks"]["detector_not_sampled"])
                self.assertTrue(verification["checks"]["no_old_success_artifacts"])

                for label, expected in declarations.items():
                    path = REPO_ROOT / expected["path"]
                    self.assertEqual(path.read_bytes(), before[label])
                    self.assertEqual(freeze.sha256_file(path), expected["sha256"])

    def test_authorization_scope_and_destinations_are_exact_and_disjoint(self) -> None:
        payload = freeze.build_freeze_payload(check_destinations=False)
        authorization = payload["authorization"]
        self.assertEqual(
            authorization["kind"],
            "explicit_user_authorized_v23_after_terminal_v22_error",
        )
        self.assertFalse(authorization["previous_v21_error_reclassified"])
        self.assertFalse(authorization["previous_v22_error_reclassified"])
        self.assertFalse(authorization["previous_terminal_artifacts_mutable"])
        self.assertTrue(payload["correction"]["facts_are_never_aggregated"])
        self.assertFalse(payload["correction"]["global_grid_equality_required"])

        scope = payload["post_pass_scope"]
        self.assertTrue(scope["development_candidate_freeze_allowed"])
        self.assertTrue(scope["h0_integration_implementation_allowed"])
        for key in (
            "h0_execution_allowed",
            "development_candidate_h0_ready_allowed",
            "independent_validation_claim_allowed",
            "protected_trial_access_allowed",
            "runtime_promotion_allowed",
            "training_promotion_allowed",
            "corridor_activation_allowed",
            "positive_morphology_reward_ppo_allowed",
        ):
            self.assertIs(scope[key], False, key)
        self.assertEqual(
            payload["data_governance"]["protected_trials_remaining_closed"],
            ["05", "06"],
        )

        old_paths = {
            freeze.v21_freeze.FREEZE_PATH.resolve(),
            freeze.v21_freeze.EXECUTION_LEDGER_PATH.resolve(),
            freeze.v21_freeze.OUTPUT_DIR.resolve(),
            freeze.v22_freeze.FREEZE_PATH.resolve(),
            freeze.v22_freeze.EXECUTION_LEDGER_PATH.resolve(),
            freeze.v22_freeze.OUTPUT_DIR.resolve(),
        }
        new_paths = {
            freeze.FREEZE_PATH.resolve(),
            freeze.EXECUTION_LEDGER_PATH.resolve(),
            freeze.OUTPUT_DIR.resolve(),
        }
        self.assertTrue(old_paths.isdisjoint(new_paths))
        for new_path in new_paths:
            for old_path in old_paths:
                self.assertFalse(new_path.is_relative_to(old_path))
                self.assertFalse(old_path.is_relative_to(new_path))

    def test_real_oracle_binding_passes_with_false_descriptive_facts(self) -> None:
        ledger, core = self._oracle()
        ledger_before = copy.deepcopy(ledger)
        core_before = copy.deepcopy(core)
        binding = freeze.validate_oracle_binding(core, ledger)

        assertions = binding["assertions"]
        facts = binding["facts"]
        self.assertTrue(assertions)
        self.assertTrue(all(isinstance(value, bool) for value in assertions.values()))
        self.assertTrue(all(assertions.values()))
        self.assertIs(binding["assertions_pass"], True)
        self.assertIs(facts["global_grid_equality_required"], False)
        self.assertIs(facts["global_grids_equal"], False)
        self.assertIs(binding["coverage"]["pass"], True)
        self.assertEqual(
            binding["coverage"], freeze.FROZEN_V22_COVERAGE
        )
        self.assertEqual(core, core_before)
        self.assertEqual(ledger, ledger_before)

    def test_false_facts_are_demonstrably_not_part_of_assertion_aggregate(self) -> None:
        ledger, core = self._oracle()
        binding = freeze.validate_oracle_binding(core, ledger)
        mixed = {**binding["assertions"], **binding["facts"]}

        self.assertTrue(binding["assertions_pass"])
        self.assertFalse(all(mixed.values()))
        self.assertNotEqual(binding["assertions_pass"], all(mixed.values()))
        self.assertTrue(any(value is False for value in binding["facts"].values()))

    def test_mutated_positive_assertion_fails_the_gate(self) -> None:
        ledger, core = self._oracle()
        with patch.object(freeze, "ORACLE_CORE_SHA256", "0" * 64):
            binding = freeze.validate_oracle_binding(core, ledger)
        self.assertIs(binding["assertions"]["scientific_core_sha_exact"], False)
        self.assertIs(binding["assertions_pass"], False)
        self.assertFalse(all(binding["assertions"].values()))

    def test_preflight_binds_real_oracle_without_detector_acquisition(self) -> None:
        subject = self._runner()
        temporary = self._temporary_dir()
        lock_path = temporary / "freeze.json"
        ledger_path = temporary / "execution_ledger.json"
        output_dir = temporary / "run"
        replay_inputs = {
            "paths": {},
            "plugin_loader_absolute": REPO_ROOT / "unused",
            "assertions": {"synthetic_environment_bypass": True},
            "source_records": {},
        }
        common_environment = {
            "assertions": {"synthetic_environment_bypass": True},
            "plugin_loaded_in_current_process": False,
            "detector_replay_started": False,
        }
        with (
            patch.object(freeze, "FREEZE_PATH", lock_path),
            patch.object(freeze, "EXECUTION_LEDGER_PATH", ledger_path),
            patch.object(freeze, "OUTPUT_DIR", output_dir),
            patch.object(
                subject,
                "_preflight_replay_inputs",
                return_value=replay_inputs,
            ),
            patch.object(
                subject,
                "_preflight_common_environment",
                return_value=common_environment,
            ),
            patch.object(subject, "_acquire_trace") as acquire,
        ):
            payload = freeze.build_freeze_payload(check_destinations=False)
            lock_path.write_text(
                json.dumps(payload, sort_keys=True, allow_nan=False) + "\n",
                encoding="utf-8",
            )
            result = subject.preflight_unopened()

        acquire.assert_not_called()
        self.assertFalse(ledger_path.exists())
        self.assertFalse(output_dir.exists())
        self.assertFalse(result["detector_replay_started"])
        self.assertTrue(result["oracle_bound_preopen"])
        self.assertTrue(all(result["oracle"]["assertions"].values()))
        self.assertTrue(result["oracle"]["assertions_pass"])
        self.assertFalse(
            result["oracle"]["facts"]["global_grid_equality_required"]
        )

    def test_half_millisecond_trace_offset_fails_closed(self) -> None:
        _ledger, core = self._oracle()
        with self.assertRaisesRegex(freeze.v22_freeze.V22FreezeError, "subgrid"):
            freeze.v22_freeze.validate_oracle_trace_coverage(
                core,
                trace_start_s=freeze.TRACE_START_S + 0.0005,
                trace_end_s=freeze.TRACE_END_S + 0.0005,
            )

    def test_trace_count_mismatch_fails_closed(self) -> None:
        _ledger, core = self._oracle()
        with self.assertRaisesRegex(freeze.v22_freeze.V22FreezeError, "subgrid"):
            freeze.v22_freeze.validate_oracle_trace_coverage(
                core,
                trace_sample_count=freeze.TRACE_SAMPLE_COUNT - 1,
            )

    def test_oracle_grid_count_mismatch_fails_closed(self) -> None:
        _ledger, original = self._oracle()
        core = copy.deepcopy(original)
        core["time_grid"]["sample_count"] -= 1
        with self.assertRaisesRegex(freeze.v22_freeze.V22FreezeError, "subgrid"):
            freeze.v22_freeze.validate_oracle_trace_coverage(core)

    def test_view_coverage_mutation_fails_closed(self) -> None:
        _ledger, original = self._oracle()
        core = copy.deepcopy(original)
        core["views"][0]["interval_s"][0] = freeze.TRACE_START_S + 0.010
        expected = [copy.deepcopy(item) for item in freeze.EXPECTED_VIEWS]
        expected[0]["interval_s"][0] = freeze.TRACE_START_S + 0.010
        with patch.object(freeze.v22_freeze, "EXPECTED_VIEWS", tuple(expected)):
            with self.assertRaisesRegex(
                freeze.v22_freeze.V22FreezeError,
                "does not cover|scoreable event",
            ):
                freeze.v22_freeze.validate_oracle_trace_coverage(core)

    def test_scoreable_event_half_sample_mutation_fails_closed(self) -> None:
        _ledger, original = self._oracle()
        core = copy.deepcopy(original)
        event = core["views"][0]["scoreable_events"][0]
        event["event_time_s"] = float(event["event_time_s"]) + 0.0005
        event["persistence_confirmed_time_s"] = (
            float(event["persistence_confirmed_time_s"]) + 0.0005
        )
        with self.assertRaisesRegex(
            freeze.v22_freeze.V22FreezeError, "scoreable event"
        ):
            freeze.v22_freeze.validate_oracle_trace_coverage(core)

    def test_runner_cli_exposes_no_candidate_geometry_or_tuning_controls(self) -> None:
        options = self._cli_options(RUNNER_PATH)
        self.assertIn("--check", options)
        self.assertIn("--execute-development", options)
        forbidden = (
            "candidate",
            "geometry",
            "profile",
            "threshold",
            "debounce",
            "sweep",
            "tune",
            "retune",
            "h0",
            "protected",
        )
        for option in options:
            self.assertFalse(
                any(fragment in option.lower() for fragment in forbidden), option
            )

    def test_runner_writer_is_no_clobber_for_files_and_dangling_symlinks(self) -> None:
        subject = self._runner()
        temporary = self._temporary_dir()
        path = temporary / "artifact.json"
        subject._write_json_exclusive(path, {"value": 1.0})
        original = path.read_bytes()
        with self.assertRaisesRegex(Exception, "clobber|existing"):
            subject._write_json_exclusive(path, {"value": 2.0})
        self.assertEqual(path.read_bytes(), original)

        occupied = temporary / "occupied.json"
        occupied.symlink_to("missing-target.json")
        self.assertTrue(os.path.lexists(occupied))
        self.assertFalse(occupied.exists())
        with self.assertRaisesRegex(Exception, "clobber|existing"):
            subject._write_json_exclusive(occupied, {"value": 3.0})
        self.assertTrue(occupied.is_symlink())
        self.assertEqual(os.readlink(occupied), "missing-target.json")
        self.assertEqual(list(temporary.glob(".*.tmp")), [])

    def test_failure_writer_rejects_foreign_pid_and_terminal_decision(self) -> None:
        subject = self._runner()
        temporary = self._temporary_dir()
        output_dir = temporary / "run"
        output_dir.mkdir()
        ledger_path = temporary / "ledger.json"
        foreign = {
            "process_id": os.getpid() + 1000,
            "run_id": subject.RUN_ID,
            "candidate_id": freeze.CANDIDATE_ID,
            "trial_id": freeze.TRIAL_ID,
        }
        ledger_path.write_text(json.dumps(foreign) + "\n", encoding="utf-8")
        receipt = output_dir / subject.ACCESS_RECEIPT_NAME
        receipt.write_text(json.dumps(foreign) + "\n", encoding="utf-8")
        with (
            patch.object(freeze, "EXECUTION_LEDGER_PATH", ledger_path),
            patch.object(freeze, "OUTPUT_DIR", output_dir),
        ):
            subject._write_failure_after_open(RuntimeError("foreign process"))
        self.assertFalse((output_dir / subject.FAILURE_NAME).exists())

        decision = output_dir / subject.DECISION_NAME
        decision.write_text('{"pass":true}\n', encoding="utf-8")
        before = {path.name: path.read_bytes() for path in output_dir.iterdir()}
        with (
            patch.object(freeze, "EXECUTION_LEDGER_PATH", ledger_path),
            patch.object(freeze, "OUTPUT_DIR", output_dir),
        ):
            subject._write_failure_after_open(RuntimeError("after decision"))
        after = {path.name: path.read_bytes() for path in output_dir.iterdir()}
        self.assertEqual(after, before)
        self.assertFalse((output_dir / subject.FAILURE_NAME).exists())

    def test_execute_ast_preflights_then_opens_before_detector_acquisition(self) -> None:
        subject = self._runner()
        tree = ast.parse(RUNNER_PATH.read_text(encoding="utf-8"))
        function = next(
            node
            for node in tree.body
            if isinstance(node, ast.FunctionDef)
            and node.name == subject.execute_development.__name__
        )
        calls: list[tuple[str, int]] = []
        for node in ast.walk(function):
            if not isinstance(node, ast.Call):
                continue
            if isinstance(node.func, ast.Name):
                calls.append((node.func.id, node.lineno))
            elif isinstance(node.func, ast.Attribute):
                calls.append((node.func.attr, node.lineno))
        preflight = [line for name, line in calls if name == "preflight_unopened"]
        opening = [line for name, line in calls if name == "_open_stage"]
        acquisition = [line for name, line in calls if name == "_acquire_trace"]
        self.assertEqual(len(preflight), 1)
        self.assertEqual(len(opening), 1)
        self.assertEqual(len(acquisition), 1)
        self.assertLess(preflight[0], opening[0])
        self.assertLess(opening[0], acquisition[0])

    def test_oracle_ast_never_aggregates_facts_or_a_mixed_binding(self) -> None:
        paths = [Path(freeze.__file__).resolve(), RUNNER_PATH]
        inspected_functions = 0
        for path in paths:
            tree = ast.parse(path.read_text(encoding="utf-8"))
            for function in (
                node
                for node in ast.walk(tree)
                if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
                and ("oracle" in node.name or "binding" in node.name)
            ):
                inspected_functions += 1
                for call in (
                    node for node in ast.walk(function) if isinstance(node, ast.Call)
                ):
                    if not (
                        isinstance(call.func, ast.Name)
                        and call.func.id == "all"
                        and call.args
                    ):
                        continue
                    aggregate = ast.unparse(call.args[0])
                    self.assertNotIn("facts", aggregate)
                    self.assertNotIn("binding.values()", aggregate)
                    self.assertNotIn("checks.values()", aggregate)
                    if ".values()" in aggregate:
                        self.assertIn("assertions", aggregate)
        self.assertGreaterEqual(inspected_functions, 3)


if __name__ == "__main__":
    unittest.main()
