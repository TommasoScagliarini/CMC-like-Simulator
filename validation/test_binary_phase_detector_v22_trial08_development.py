"""Pre-execution tests for the V22 open-development coverage correction.

The suite never launches OpenSim and never executes the V22 development run.
It proves that the terminal V21 result remains immutable, exercises the sole
allowed oracle/trace coverage correction against both the frozen trial-08
oracle and synthetic failures, and audits the fail-closed runner contract when
the runner source is present.
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

import freeze_binary_phase_detector_v21_trial08 as v21_freeze  # noqa: E402
import validate_binary_phase_detector_v21_trial08_one_shot as v21_gate  # noqa: E402
import freeze_binary_phase_detector_v22_trial08_development as freeze  # noqa: E402


RUNNER_MODULE = "validate_binary_phase_detector_v22_trial08_development"
RUNNER_PATH = VALIDATION_ROOT / f"{RUNNER_MODULE}.py"


class V22DevelopmentCoverageTests(unittest.TestCase):
    def _temporary_dir(self) -> Path:
        temporary = tempfile.TemporaryDirectory(dir=VALIDATION_ROOT)
        self.addCleanup(temporary.cleanup)
        return Path(temporary.name)

    def _oracle_core(self) -> dict:
        self.assertEqual(freeze.sha256_file(freeze.ORACLE_PATH), freeze.ORACLE_SHA256)
        ledger = json.loads(freeze.ORACLE_PATH.read_text(encoding="utf-8"))
        self.assertEqual(
            ledger["scientific_core_sha256"], freeze.ORACLE_CORE_SHA256
        )
        return copy.deepcopy(ledger["scientific_core"])

    def _runner_or_skip(self):
        if not RUNNER_PATH.is_file():
            self.skipTest("V22 runner source is not present yet")
        importlib.invalidate_caches()
        return importlib.import_module(RUNNER_MODULE)

    def test_previous_v21_terminal_artifacts_are_exact_and_immutable(self) -> None:
        before: dict[str, bytes] = {}
        for label, expected in freeze.PREVIOUS_TERMINAL.items():
            path = REPO_ROOT / expected["path"]
            self.assertTrue(path.is_file(), label)
            self.assertEqual(freeze.sha256_file(path), expected["sha256"], label)
            before[label] = path.read_bytes()

        verification = freeze._verify_previous_terminal()
        self.assertTrue(all(verification["checks"].values()))
        self.assertTrue(verification["checks"]["ledger_consumed"])
        self.assertTrue(verification["checks"]["ledger_no_retry"])
        self.assertTrue(verification["checks"]["no_detector_trace_sampled"])
        self.assertTrue(verification["checks"]["no_old_success_artifacts"])

        for label, expected in freeze.PREVIOUS_TERMINAL.items():
            path = REPO_ROOT / expected["path"]
            self.assertEqual(path.read_bytes(), before[label], label)
            self.assertEqual(freeze.sha256_file(path), expected["sha256"], label)

    def test_real_oracle_is_an_exact_12_10_subgrid_coverage_pass(self) -> None:
        core = self._oracle_core()
        result = freeze.validate_oracle_trace_coverage(core)

        oracle_grid = result["oracle_native_grid"]
        trace_grid = result["detector_trace_grid"]
        self.assertNotEqual(oracle_grid["start_s"], trace_grid["start_s"])
        self.assertNotEqual(oracle_grid["end_s"], trace_grid["end_s"])
        self.assertNotEqual(oracle_grid["sample_count"], trace_grid["sample_count"])
        self.assertFalse(result["global_grid_equality_required"])
        self.assertFalse(result["consumer_rethresholds_grf"])
        self.assertTrue(result["pass"])
        subgrid = result["exact_contiguous_subgrid"]
        self.assertEqual(subgrid["left_margin_samples"], 12)
        self.assertEqual(subgrid["right_margin_samples"], 10)
        self.assertTrue(all(subgrid["checks"].values()))
        self.assertEqual(len(result["views"]), 4)
        self.assertTrue(all(view["pass"] for view in result["views"]))

    def test_half_millisecond_trace_offset_fails_closed(self) -> None:
        core = self._oracle_core()
        with self.assertRaisesRegex(freeze.V22FreezeError, "subgrid contract"):
            freeze.validate_oracle_trace_coverage(
                core,
                trace_start_s=freeze.TRACE_START_S + 0.0005,
                trace_end_s=freeze.TRACE_END_S + 0.0005,
            )

    def test_oracle_grid_count_mismatch_fails_closed(self) -> None:
        core = self._oracle_core()
        core["time_grid"]["sample_count"] -= 1
        with self.assertRaisesRegex(freeze.V22FreezeError, "subgrid contract"):
            freeze.validate_oracle_trace_coverage(core)

    def test_declared_trace_count_mismatch_fails_closed(self) -> None:
        core = self._oracle_core()
        with self.assertRaisesRegex(freeze.V22FreezeError, "subgrid contract"):
            freeze.validate_oracle_trace_coverage(
                core,
                trace_sample_count=freeze.TRACE_SAMPLE_COUNT - 1,
            )

    def test_view_without_required_left_margin_fails_closed(self) -> None:
        core = self._oracle_core()
        core["views"][0]["interval_s"][0] = freeze.TRACE_START_S + 0.010
        expected = [copy.deepcopy(item) for item in freeze.EXPECTED_VIEWS]
        expected[0]["interval_s"][0] = freeze.TRACE_START_S + 0.010
        with patch.object(freeze, "EXPECTED_VIEWS", tuple(expected)):
            with self.assertRaisesRegex(
                freeze.V22FreezeError, "does not cover|scoreable event"
            ):
                freeze.validate_oracle_trace_coverage(core)

    def test_scoreable_event_outside_trace_fails_closed(self) -> None:
        core = self._oracle_core()
        event = core["views"][0]["scoreable_events"][0]
        event["event_time_s"] = freeze.TRACE_START_S - freeze.SAMPLE_DT_S
        event["sample_index"] = 11
        event["persistence_confirmed_time_s"] = freeze.TRACE_START_S + 0.049
        with self.assertRaisesRegex(freeze.V22FreezeError, "scoreable event"):
            freeze.validate_oracle_trace_coverage(core)

    def test_scoreable_event_half_sample_offset_fails_closed(self) -> None:
        core = self._oracle_core()
        event = core["views"][0]["scoreable_events"][0]
        event["event_time_s"] = float(event["event_time_s"]) + 0.0005
        event["persistence_confirmed_time_s"] = (
            float(event["persistence_confirmed_time_s"]) + 0.0005
        )
        with self.assertRaisesRegex(freeze.V22FreezeError, "scoreable event"):
            freeze.validate_oracle_trace_coverage(core)

    def test_non_scoreable_core_events_in_oracle_prefix_suffix_are_allowed(self) -> None:
        core = self._oracle_core()
        prefix = {
            "event": "heel_strike",
            "event_time_s": freeze.ORACLE_START_S,
            "sample_index": 0,
            "persistence_confirmed_time_s": freeze.ORACLE_START_S + 0.050,
        }
        suffix = {
            "event": "toe_off",
            "event_time_s": freeze.ORACLE_END_S,
            "sample_index": freeze.ORACLE_SAMPLE_COUNT - 1,
        }
        core["events"] = [prefix, *core["events"], suffix]

        # Only the frozen scoreable_events in each view form the gate.  Events
        # that exist solely in the native-ledger prefix/suffix are permissible.
        result = freeze.validate_oracle_trace_coverage(core)
        self.assertTrue(result["pass"])
        self.assertEqual(result["exact_contiguous_subgrid"]["left_margin_samples"], 12)
        self.assertEqual(result["exact_contiguous_subgrid"]["right_margin_samples"], 10)

    def test_v22_destinations_are_disjoint_from_terminal_v21(self) -> None:
        old_paths = {
            v21_freeze.FREEZE_PATH.resolve(),
            v21_freeze.EXECUTION_LEDGER_PATH.resolve(),
            v21_freeze.OUTPUT_DIR.resolve(),
        }
        new_paths = {
            freeze.FREEZE_PATH.resolve(),
            freeze.EXECUTION_LEDGER_PATH.resolve(),
            freeze.OUTPUT_DIR.resolve(),
        }
        self.assertTrue(old_paths.isdisjoint(new_paths))
        for new_path in new_paths:
            for old_path in old_paths:
                self.assertNotEqual(new_path, old_path)
                self.assertFalse(new_path.is_relative_to(old_path))
                self.assertFalse(old_path.is_relative_to(new_path))

        old_artifact_paths = {
            (REPO_ROOT / item["path"]).resolve()
            for item in freeze.PREVIOUS_TERMINAL.values()
        }
        self.assertTrue(old_artifact_paths.isdisjoint(new_paths))

    def test_post_pass_scope_forbids_h0_protected_runtime_and_training(self) -> None:
        scope = freeze.POST_PASS_SCOPE
        self.assertTrue(scope["development_candidate_freeze_allowed"])
        self.assertTrue(scope["h0_integration_implementation_allowed"])
        for key in (
            "h0_execution_allowed",
            "development_candidate_h0_ready_allowed",
            "independent_validation_claim_allowed",
            "protected_trial_access_allowed",
            "runtime_promotion_allowed",
            "training_promotion_allowed",
            "positive_morphology_reward_ppo_allowed",
        ):
            self.assertIs(scope[key], False, key)

        if RUNNER_PATH.is_file():
            payload = freeze.build_freeze_payload(check_destinations=False)
            self.assertEqual(payload["post_pass_scope"], freeze.POST_PASS_SCOPE)
            self.assertEqual(
                payload["data_governance"]["protected_trials_opened"], []
            )
            self.assertEqual(
                payload["data_governance"]["protected_trials_remaining_closed"],
                ["05", "06"],
            )
            for key in (
                "h0_execution_allowed",
                "training_allowed",
                "protected_or_reserve_access_allowed",
            ):
                self.assertIs(payload["non_actions"][key], False, key)

    def test_runner_cli_has_no_candidate_geometry_or_tuning_controls(self) -> None:
        if not RUNNER_PATH.is_file():
            self.skipTest("V22 runner source is not present yet")
        tree = ast.parse(RUNNER_PATH.read_text(encoding="utf-8"))
        options: set[str] = set()
        for node in ast.walk(tree):
            if not isinstance(node, ast.Call):
                continue
            if not isinstance(node.func, ast.Attribute):
                continue
            if node.func.attr != "add_argument":
                continue
            for argument in node.args:
                if isinstance(argument, ast.Constant) and isinstance(
                    argument.value, str
                ):
                    if argument.value.startswith("-"):
                        options.add(argument.value)

        self.assertIn("--check", options)
        self.assertTrue(
            {"--execute-development", "--execute-development-once"} & options
        )
        forbidden_fragments = (
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
                any(fragment in option.lower() for fragment in forbidden_fragments),
                option,
            )

    def test_runner_writer_is_strict_no_clobber_including_dangling_link(self) -> None:
        subject = self._runner_or_skip()
        writer = getattr(subject, "_write_json_exclusive", None)
        if writer is None:
            self.skipTest("V22 runner does not expose its exclusive JSON writer")
        temporary = self._temporary_dir()
        path = temporary / "artifact.json"
        writer(path, {"value": 1.0})
        original = path.read_bytes()
        with self.assertRaisesRegex(Exception, "clobber|existing"):
            writer(path, {"value": 2.0})
        self.assertEqual(path.read_bytes(), original)

        occupied = temporary / "occupied.json"
        occupied.symlink_to("missing-target.json")
        self.assertTrue(os.path.lexists(occupied))
        self.assertFalse(occupied.exists())
        with self.assertRaisesRegex(Exception, "clobber|existing"):
            writer(occupied, {"value": 3.0})
        self.assertTrue(occupied.is_symlink())
        self.assertEqual(os.readlink(occupied), "missing-target.json")
        self.assertEqual(list(temporary.glob(".*.tmp")), [])

    def test_failure_writer_rejects_foreign_pid_and_terminal_decision(self) -> None:
        subject = self._runner_or_skip()
        failure_writer = getattr(subject, "_write_failure_after_open", None)
        if failure_writer is None:
            self.skipTest("V22 runner does not expose its terminal failure writer")
        access_name = getattr(subject, "ACCESS_RECEIPT_NAME", "trial08_access_receipt.json")
        failure_name = getattr(subject, "FAILURE_NAME", "failure.json")
        decision_name = getattr(subject, "DECISION_NAME", "trial08_decision_lock.json")
        temporary = self._temporary_dir()
        output_dir = temporary / "run"
        output_dir.mkdir()
        ledger_path = temporary / "ledger.json"
        foreign = {
            "process_id": os.getpid() + 1000,
            "candidate_id": freeze.CANDIDATE_ID,
            "trial_id": freeze.TRIAL_ID,
        }
        ledger_path.write_text(json.dumps(foreign) + "\n", encoding="utf-8")
        (output_dir / access_name).write_text(
            json.dumps(foreign) + "\n", encoding="utf-8"
        )
        with (
            patch.object(freeze, "EXECUTION_LEDGER_PATH", ledger_path),
            patch.object(freeze, "OUTPUT_DIR", output_dir),
        ):
            failure_writer(RuntimeError("foreign process"))
        self.assertFalse((output_dir / failure_name).exists())

        decision_path = output_dir / decision_name
        decision_path.write_text('{"pass":true}\n', encoding="utf-8")
        before = {path.name: path.read_bytes() for path in output_dir.iterdir()}
        with (
            patch.object(freeze, "EXECUTION_LEDGER_PATH", ledger_path),
            patch.object(freeze, "OUTPUT_DIR", output_dir),
        ):
            failure_writer(RuntimeError("after terminal decision"))
        after = {path.name: path.read_bytes() for path in output_dir.iterdir()}
        self.assertEqual(after, before)
        self.assertFalse((output_dir / failure_name).exists())

    def test_runner_publishes_opening_before_oracle_trace_or_evaluation(self) -> None:
        subject = self._runner_or_skip()
        execute = getattr(subject, "execute_development", None)
        if execute is None:
            execute = getattr(subject, "execute_development_once", None)
        if execute is None:
            self.skipTest("V22 runner does not expose its development executor")

        source = ast.parse(RUNNER_PATH.read_text(encoding="utf-8"))
        function = next(
            (
                node
                for node in source.body
                if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
                and node.name == execute.__name__
            ),
            None,
        )
        self.assertIsNotNone(function)
        calls: list[tuple[str, int]] = []
        for node in ast.walk(function):
            if not isinstance(node, ast.Call):
                continue
            if isinstance(node.func, ast.Name):
                calls.append((node.func.id, node.lineno))
            elif isinstance(node.func, ast.Attribute):
                calls.append((node.func.attr, node.lineno))

        opening_lines = [
            line
            for name, line in calls
            if name in {"_open_stage", "_publish_opening_receipts"}
        ]
        semantic_lines = [
            line
            for name, line in calls
            if name
            in {
                "_validate_trial08_replay_inputs",
                "_load_oracle",
                "_acquire_trace",
                "_evaluate",
            }
        ]
        self.assertEqual(len(opening_lines), 1)
        self.assertTrue(semantic_lines)
        self.assertLess(opening_lines[0], min(semantic_lines))


if __name__ == "__main__":
    unittest.main()
