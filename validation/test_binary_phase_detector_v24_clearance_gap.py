"""Pre-execution tests for the V24 trial-08 clearance-gap diagnostic.

The suite never invokes the OpenSim sampling path or the diagnostic executor.
It validates immutable V23 lineage, the frozen 1 ms window, signed-clearance
helpers, debounce sample semantics, compatibility declarations, and exclusive
output behavior using read-only artifacts and synthetic clearances.
"""

from __future__ import annotations

import ast
import copy
import json
import math
import os
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
VALIDATION_ROOT = REPO_ROOT / "validation"
if str(VALIDATION_ROOT) not in sys.path:
    sys.path.insert(0, str(VALIDATION_ROOT))

import diagnose_binary_phase_detector_v24_clearance_gap as subject  # noqa: E402


V23_RUN_DIR = (
    VALIDATION_ROOT
    / "binary_phase_detector_v23_development_runs"
    / "2026-08-04_trial08_assertions_facts_fix"
)
V23_LINEAGE = {
    "v23_freeze": (
        VALIDATION_ROOT
        / "binary_phase_detector_v23_trial08_development_freeze_lock.json",
        "6a958e90dcc7370adfae64c8bd7970bce92d9af227c12bf6019a93ae422072fd",
    ),
    "v23_execution_ledger": (
        VALIDATION_ROOT
        / "binary_phase_detector_v23_trial08_development_execution_ledger.json",
        "c77dd856503d377570cf095882156e33057cd9071a04f255561d91e3a8e4645c",
    ),
    "v23_access_receipt": (
        V23_RUN_DIR / "trial08_development_access_receipt.json",
        "c77dd856503d377570cf095882156e33057cd9071a04f255561d91e3a8e4645c",
    ),
    "v23_manifest": (
        V23_RUN_DIR / "manifest.json",
        "333dc729a88e458a81038a9a90155e4cf08b68daed8aa1ee780be974a8b51f1f",
    ),
    "v23_decision": (
        V23_RUN_DIR / "trial08_development_decision_lock.json",
        "de55bebbd9b1a21bf3aaadd7132cebed34554e14f5034797b159b14969a4510c",
    ),
    "v23_units": (
        V23_RUN_DIR / "unit_metrics.json",
        "29398345fac488f623b4f053cf3b1af69033c54891a579539f4e7a3f3e55e75b",
    ),
    "v23_packed_trace": (
        V23_RUN_DIR / "packed_binary_trace.json",
        "ed52d2044d5ffb6f74fcb03adad2829834ab41c6eb03e4774f058f1b96795181",
    ),
    "v23_event_journal": (
        V23_RUN_DIR / "event_journal.json",
        "00a135b9e4d735ca2969f2fe1a27e0d7ad6e8434893437ea0ab45770ad008694",
    ),
}


class V24ClearanceGapTests(unittest.TestCase):
    def _temporary_dir(self) -> Path:
        temporary = tempfile.TemporaryDirectory(dir=VALIDATION_ROOT)
        self.addCleanup(temporary.cleanup)
        return Path(temporary.name)

    @staticmethod
    def _index(times: np.ndarray, value: float) -> int:
        matches = np.flatnonzero(np.isclose(times, value, atol=1e-12, rtol=0.0))
        if matches.size != 1:
            raise AssertionError(f"timestamp is not unique on grid: {value}")
        return int(matches[0])

    def _synthetic_clearances(
        self, *, split_levels: bool = False
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        times = subject._time_grid()
        heel = np.full(times.shape, -0.001, dtype=float)
        toe = np.full(times.shape, -0.001, dtype=float)
        start = self._index(times, subject.V23_FALSE_AIR_ONSET_S)
        end = self._index(times, subject.V23_FALSE_AIR_LAST_SAMPLE_S)
        heel[start : end + 1] = 0.004
        toe[start : end + 1] = 0.003
        if split_levels:
            # At 1 mm the remaining OFF runs contain 5 and 6 samples: six
            # samples span the required 5 ms and still confirm TO.  At 2 mm,
            # the runs become 5 and 5 samples and cannot confirm TO.
            toe[start + 5] = 0.001
            toe[start + 6] = 0.002
        return times, heel, toe

    def _assert_finite_tree(self, value) -> None:
        if isinstance(value, bool) or value is None or isinstance(value, str):
            return
        if isinstance(value, (int, float, np.integer, np.floating)):
            self.assertTrue(math.isfinite(float(value)), value)
            return
        if isinstance(value, dict):
            for child in value.values():
                self._assert_finite_tree(child)
            return
        if isinstance(value, (list, tuple)):
            for child in value:
                self._assert_finite_tree(child)
            return
        self.fail(f"unsupported diagnostic value: {type(value).__name__}")

    def test_v23_lineage_hashes_and_terminal_failure_are_exact(self) -> None:
        before: dict[str, bytes] = {}
        for label, (path, expected_sha) in V23_LINEAGE.items():
            self.assertTrue(path.is_file(), label)
            self.assertEqual(subject.sha256_file(path), expected_sha, label)
            before[label] = path.read_bytes()

        self.assertEqual(
            V23_LINEAGE["v23_execution_ledger"][0].read_bytes(),
            V23_LINEAGE["v23_access_receipt"][0].read_bytes(),
        )
        decision = json.loads(V23_LINEAGE["v23_decision"][0].read_text())
        self.assertEqual(
            decision["status"],
            "FAIL_V23_TRIAL08_OPEN_DEVELOPMENT_REPLAY_TERMINAL",
        )
        self.assertFalse(decision["pass"])
        self.assertTrue(decision["cycle_consumed"])
        self.assertFalse(decision["rerun_allowed"])
        self.assertEqual(decision["unit_count"], 8)
        self.assertEqual(decision["unit_pass_count"], 6)
        self.assertEqual(decision["protected_trials_opened"], [])

        for label, (path, expected_sha) in V23_LINEAGE.items():
            self.assertEqual(path.read_bytes(), before[label])
            self.assertEqual(subject.sha256_file(path), expected_sha)

    def test_pinned_v23_lineage_declarations_match_real_artifacts(self) -> None:
        required = {
            "v23_freeze",
            "v23_execution_ledger",
            "v23_access_receipt",
            "v23_decision",
            "v23_manifest",
            "v23_units",
            "v23_packed_trace",
            "v23_event_journal",
        }
        self.assertTrue(required.issubset(subject.PINNED_INPUTS))
        for label in required:
            path, expected_sha = V23_LINEAGE[label]
            declaration = subject.PINNED_INPUTS[label]
            self.assertEqual(
                (REPO_ROOT / declaration["path"]).resolve(), path.resolve(), label
            )
            self.assertEqual(declaration["sha256"], expected_sha, label)

    def test_frozen_context_grid_is_exact_1ms_and_contains_dropout(self) -> None:
        times = subject._time_grid()
        expected_count = int(
            round(
                (subject.CONTEXT_END_S - subject.CONTEXT_START_S)
                / subject.SAMPLE_DT_S
            )
        ) + 1
        self.assertEqual(expected_count, 451)
        self.assertEqual(times.size, expected_count)
        self.assertAlmostEqual(float(times[0]), 99.750, places=12)
        self.assertAlmostEqual(float(times[-1]), 100.200, places=12)
        self.assertTrue(
            np.allclose(
                np.diff(times), subject.SAMPLE_DT_S, atol=1e-12, rtol=0.0
            )
        )
        for timestamp in (
            subject.CORE_WINDOW_START_S,
            subject.V23_FALSE_AIR_ONSET_S,
            subject.V23_FALSE_AIR_LAST_SAMPLE_S,
            subject.V23_RECONTACT_ONSET_S,
            subject.CORE_WINDOW_END_S,
            subject.ORACLE_TRUE_TO_S,
        ):
            self._index(times, timestamp)
        self.assertLessEqual(
            subject.CORE_WINDOW_START_S, subject.V23_FALSE_AIR_ONSET_S
        )
        self.assertGreaterEqual(
            subject.CORE_WINDOW_END_S, subject.V23_RECONTACT_ONSET_S
        )
        self.assertLessEqual(subject.CONTEXT_START_S, subject.CORE_WINDOW_START_S)
        self.assertGreaterEqual(subject.CONTEXT_END_S, subject.ORACLE_TRUE_TO_S)

    def test_time_grid_rejects_half_sample_and_nonfinite_contracts(self) -> None:
        with self.assertRaises(subject.V24DiagnosticError):
            subject._time_grid(99.7505, 100.200, 0.001)
        with self.assertRaises(subject.V24DiagnosticError):
            subject._time_grid(99.750, 100.2005, 0.001)
        with self.assertRaises(subject.V24DiagnosticError):
            subject._time_grid(math.nan, 100.200, 0.001)

    def test_synthetic_gap_metrics_are_finite_and_exact(self) -> None:
        times, heel, toe = self._synthetic_clearances()
        metrics = subject._clearance_gap_metrics(times, heel, toe)
        critical = metrics["critical_gap"]
        self.assertEqual(metrics["sample_count"], times.size)
        self.assertEqual(metrics["sample_dt_s"], 0.001)
        self.assertEqual(metrics["contact_rule"], "signed_clearance_le_zero")
        self.assertAlmostEqual(critical["start_time_s"], 99.882, places=12)
        self.assertAlmostEqual(critical["end_time_s"], 99.893, places=12)
        self.assertAlmostEqual(critical["recontact_onset_s"], 99.894, places=12)
        self.assertEqual(critical["sample_count"], 12)
        self.assertAlmostEqual(
            metrics["full_uniform_normal_shift_to_close_gap_mm"], 3.0, places=12
        )
        self.assertIn(
            metrics["full_closure_limiting_sensor"], {"left_heel", "left_toe"}
        )
        self._assert_finite_tree(metrics)

    def test_five_off_samples_do_not_confirm_but_six_do(self) -> None:
        times, heel, toe = self._synthetic_clearances(split_levels=True)
        metrics = subject._clearance_gap_metrics(times, heel, toe)

        # At a 1 mm shift, sample 5 is contact: remaining OFF runs have 5 and
        # 6 samples.  The 6-sample run spans 5 ms and therefore still confirms.
        start = self._index(times, subject.V23_FALSE_AIR_ONSET_S)
        end = self._index(times, subject.V23_FALSE_AIR_LAST_SAMPLE_S)
        envelope = np.minimum(heel[start : end + 1], toe[start : end + 1])
        runs_at_1mm = subject._false_runs(envelope <= 0.001)
        lengths_at_1mm = [finish - begin + 1 for begin, finish in runs_at_1mm]
        self.assertEqual(sorted(lengths_at_1mm), [5, 6])
        self.assertEqual((6 - 1) * subject.SAMPLE_DT_S, 0.005)
        self.assertLess((5 - 1) * subject.SAMPLE_DT_S, 0.005)

        # At 2 mm, samples 5 and 6 are contact and only two 5-sample OFF runs
        # remain.  This is the minimum shift that breaks 5 ms confirmation.
        self.assertAlmostEqual(
            metrics["minimum_uniform_shift_to_break_debounce_mm"], 2.0, places=12
        )
        self.assertEqual(metrics["longest_air_run_after_debounce_shift_samples"], 5)

    def test_full_closure_shift_leaves_zero_false_air_samples(self) -> None:
        times, heel, toe = self._synthetic_clearances(split_levels=True)
        metrics = subject._clearance_gap_metrics(times, heel, toe)
        start = self._index(times, subject.V23_FALSE_AIR_ONSET_S)
        end = self._index(times, subject.V23_FALSE_AIR_LAST_SAMPLE_S)
        envelope = np.minimum(heel[start : end + 1], toe[start : end + 1])
        shift = metrics["full_uniform_normal_shift_to_close_gap_m"]
        self.assertEqual(subject._false_runs(envelope <= shift), [])
        self.assertTrue(np.all(envelope <= shift))

    def test_clearance_metrics_reject_nonfinite_shape_and_non_1ms_data(self) -> None:
        times, heel, toe = self._synthetic_clearances()
        bad_heel = heel.copy()
        bad_heel[0] = math.nan
        with self.assertRaisesRegex(subject.V24DiagnosticError, "NaN/Inf"):
            subject._clearance_gap_metrics(times, bad_heel, toe)
        bad_toe = toe.copy()
        bad_toe[-1] = math.inf
        with self.assertRaisesRegex(subject.V24DiagnosticError, "NaN/Inf"):
            subject._clearance_gap_metrics(times, heel, bad_toe)
        with self.assertRaises(subject.V24DiagnosticError):
            subject._clearance_gap_metrics(times, heel[:-1], toe)
        bad_times = times.copy()
        bad_times[10] += 0.0005
        with self.assertRaisesRegex(subject.V24DiagnosticError, "1 ms"):
            subject._clearance_gap_metrics(bad_times, heel, toe)

    def test_real_v23_bits_reproduce_the_frozen_12_sample_dropout(self) -> None:
        declaration = subject.PINNED_INPUTS["v23_packed_trace"]
        payload = subject._strict_json(
            REPO_ROOT / declaration["path"],
            expected_sha256=declaration["sha256"],
        )
        decoded = subject._decode_v23_bits(payload)
        times = decoded["time_s"]
        start = self._index(times, subject.V23_FALSE_AIR_ONSET_S)
        end = self._index(times, subject.V23_FALSE_AIR_LAST_SAMPLE_S)
        recontact = self._index(times, subject.V23_RECONTACT_ONSET_S)
        self.assertEqual(end - start + 1, subject.V23_FALSE_AIR_SAMPLE_COUNT)
        self.assertFalse(np.any(decoded["heel"][start : end + 1]))
        self.assertFalse(np.any(decoded["toe"][start : end + 1]))
        self.assertTrue(bool(decoded["heel"][start - 1] | decoded["toe"][start - 1]))
        self.assertTrue(bool(decoded["heel"][recontact] | decoded["toe"][recontact]))

    def test_scope_forbids_protected_h0_tuning_and_promotion(self) -> None:
        self.assertTrue(subject.SCOPE["development_only"])
        self.assertEqual(subject.SCOPE["opened_development_trial"], ["08"])
        self.assertEqual(subject.SCOPE["protected_trials_opened"], [])
        self.assertEqual(subject.SCOPE["reserve_trials_opened"], [])
        for key in (
            "protected_trials_allowed",
            "reserve_trials_allowed",
            "h0_execution_allowed",
            "runtime_promotion_allowed",
            "training_allowed",
            "corridor_activation_allowed",
            "ppo_allowed",
            "geometry_or_fsm_tuning_allowed",
        ):
            self.assertIs(subject.SCOPE[key], False, key)
        for key in (
            "primary_grf_modified",
            "binary_detector_profile_modified",
            "fsm_modified",
            "cpp_or_contact_modified",
            "sea_semantics_modified",
            "h0_executed",
            "protected_trials_accessed",
            "runtime_or_training_promoted",
        ):
            self.assertIs(subject.NON_ACTIONS[key], False, key)

    def test_primary_grf_fsm_and_detector_sources_remain_hash_exact(self) -> None:
        sources = {
            REPO_ROOT / "online_grf.py": (
                "52e39bf9a3b20dd65242f3f9076d76ed788239fe7c3e5b825bc37a9657c4fefa"
            ),
            REPO_ROOT / "Trajectory Generator/binary_phase_fsm.py": (
                "0f7669b60a72c1b27ee3c4f1a43161eeb9f2d091dff5558cc4fa43f1fce8d9c1"
            ),
            REPO_ROOT / "binary_phase_detector.py": (
                "57a313133e1ce5a675b2699e940226325dfa5b2b895c7eb6b17c0892a94263b6"
            ),
        }
        before = {path: path.read_bytes() for path in sources}
        times, heel, toe = self._synthetic_clearances()
        subject._clearance_gap_metrics(times, heel, toe)
        for path, expected_sha in sources.items():
            self.assertEqual(path.read_bytes(), before[path])
            self.assertEqual(subject.sha256_file(path), expected_sha)

    def test_cli_has_only_check_and_explicit_diagnostic_execution(self) -> None:
        tree = ast.parse(Path(subject.__file__).read_text(encoding="utf-8"))
        options: set[str] = set()
        for node in ast.walk(tree):
            if (
                isinstance(node, ast.Call)
                and isinstance(node.func, ast.Attribute)
                and node.func.attr == "add_argument"
            ):
                for argument in node.args:
                    if (
                        isinstance(argument, ast.Constant)
                        and isinstance(argument.value, str)
                        and argument.value.startswith("-")
                    ):
                        options.add(argument.value)
        self.assertEqual(options, {"--check", "--execute-development-diagnostic"})

    def test_json_writer_is_strict_no_clobber_for_file_and_dangling_link(self) -> None:
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

    def test_preflight_is_read_only_and_declares_model_compatibility(self) -> None:
        temporary = self._temporary_dir()
        with (
            patch.object(subject, "OUTPUT_DIR", temporary / "unused_output"),
            patch.object(subject, "DIAGNOSTIC_PATH", temporary / "diagnostic.json"),
            patch.object(subject, "RECEIPT_PATH", temporary / "receipt.json"),
        ):
            result = subject.preflight_unopened()

        self.assertFalse((temporary / "unused_output").exists())
        self.assertFalse((temporary / "diagnostic.json").exists())
        self.assertFalse((temporary / "receipt.json").exists())
        self.assertEqual(result["scope"], subject.SCOPE)
        self.assertEqual(result["non_actions"], subject.NON_ACTIONS)
        compatibility = result["compatibility_declarations"]
        self.assertTrue(compatibility)
        self.assertTrue(all(type(value) is bool for value in compatibility.values()))
        self.assertTrue(all(compatibility.values()))
        self.assertEqual(result["paths"]["protected_trials"], [])
        self.assertFalse(result.get("opensim_sampling_started", False))

    def test_published_diagnostic_and_receipt_are_strict_and_hash_bound(self) -> None:
        diagnostic_path = V23_RUN_DIR.parent.parent / (
            "binary_phase_detector_v24_diagnostic_runs/"
            "2026-08-04_trial08_clearance_gap/clearance_gap_diagnostic.json"
        )
        receipt_path = diagnostic_path.with_name("diagnostic_receipt.json")
        diagnostic_sha = (
            "72112d308810685263d5f7af0c4371c6bcf0336e92192da9550feed6253a3546"
        )
        receipt_sha = (
            "3c4c0caa78745bafeaae812f332158f44152c4148a99d6aa4fb3fd0ec36c59c2"
        )
        diagnostic = subject._strict_json(
            diagnostic_path, expected_sha256=diagnostic_sha
        )
        receipt = subject._strict_json(receipt_path, expected_sha256=receipt_sha)

        self.assertEqual(receipt["diagnostic"]["sha256"], diagnostic_sha)
        self.assertEqual(
            receipt["diagnostic"]["size_bytes"], diagnostic_path.stat().st_size
        )
        self.assertEqual(
            subject.sha256_file(REPO_ROOT / receipt["script"]["path"]),
            receipt["script"]["sha256"],
        )
        self.assertTrue(diagnostic["pass"])
        self.assertTrue(receipt["pass"])
        self.assertEqual(
            diagnostic["status"],
            "PASS_V24_TRIAL08_GEOMETRIC_CLEARANCE_GAP_CONFIRMED",
        )
        self.assertEqual(receipt["scope"], subject.SCOPE)
        self.assertEqual(receipt["non_actions"], subject.NON_ACTIONS)
        for section in (
            "lineage_assertions",
            "compatibility_declarations",
            "numerical_assertions",
        ):
            self.assertTrue(diagnostic[section])
            self.assertTrue(all(diagnostic[section].values()), section)

        samples = diagnostic["samples"]
        self.assertEqual(len(samples), 451)
        self.assertAlmostEqual(samples[0]["time_s"], 99.750, places=12)
        self.assertAlmostEqual(samples[-1]["time_s"], 100.200, places=12)
        self.assertTrue(
            np.allclose(
                np.diff([row["time_s"] for row in samples]),
                subject.SAMPLE_DT_S,
                atol=1e-12,
                rtol=0.0,
            )
        )
        for row in samples:
            self.assertTrue(math.isfinite(row["reach_clearance_reduction_per_m"]))
            for model in ("marker_model", "runtime_model"):
                self.assertTrue(
                    math.isfinite(row[model]["left_heel_clearance_m"])
                )
                self.assertTrue(
                    math.isfinite(row[model]["left_toe_clearance_m"])
                )

        gap = diagnostic["gap_metrics"]
        self.assertEqual(gap["critical_gap"]["sample_count"], 12)
        self.assertEqual(gap["v20_confirmation_run_samples_inclusive"], 6)
        self.assertEqual(gap["longest_air_run_after_debounce_shift_samples"], 5)
        self.assertGreater(gap["full_uniform_normal_shift_to_close_gap_mm"], 0.0)
        self.assertGreater(
            gap["minimum_uniform_shift_to_break_debounce_mm"], 0.0
        )
        self.assertTrue(
            diagnostic["marker_runtime_comparison"][
                "marker_runtime_clearance_equivalent"
            ]
        )


if __name__ == "__main__":
    unittest.main()
