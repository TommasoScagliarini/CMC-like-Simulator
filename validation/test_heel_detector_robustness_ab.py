from __future__ import annotations

import json
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
VALIDATION_DIR = ROOT_DIR / "validation"
if str(VALIDATION_DIR) not in sys.path:
    sys.path.insert(0, str(VALIDATION_DIR))

import heel_detector_robustness_ab as subject  # noqa: E402


class HeelDetectorRobustnessABTest(unittest.TestCase):
    def test_default_manifest_builds_only_the_72_development_cases(self) -> None:
        manifest = subject.load_and_validate_manifest()
        cases = subject.build_cases(manifest)

        self.assertEqual(len(cases), 72)
        self.assertSetEqual(
            {case.mode_name for case in cases},
            {"legacy", "shadow", "two_sensor"},
        )
        self.assertSetEqual(
            {case.checkpoint_name for case in cases},
            {"h0", "pilot50_best"},
        )
        self.assertSetEqual(
            {case.start_name for case in cases},
            {"minus020", "nominal", "plus020"},
        )
        self.assertSetEqual({case.seed for case in cases}, {123, 124, 125})
        self.assertTrue(
            {case.seed for case in cases}.isdisjoint({126, 127, 128})
        )

    def test_manifest_refuses_a_heldout_seed(self) -> None:
        raw = json.loads(subject.DEFAULT_MANIFEST.read_text(encoding="utf-8"))
        raw["development_seeds"] = [123, 124, 126]
        raw["action_matrix"]["stochastic_seeds"] = [123, 124, 126]

        with tempfile.TemporaryDirectory() as temporary_directory:
            path = Path(temporary_directory) / "bad_protocol.json"
            path.write_text(json.dumps(raw), encoding="utf-8")

            with self.assertRaisesRegex(subject.ProtocolError, "development seeds"):
                subject.load_and_validate_manifest(path)

    def test_commands_keep_shadow_on_legacy_and_activate_two_sensor(self) -> None:
        manifest = subject.load_and_validate_manifest()
        cases = subject.build_cases(manifest)
        shadow = next(case for case in cases if case.mode_name == "shadow")
        active = next(case for case in cases if case.mode_name == "two_sensor")

        with tempfile.TemporaryDirectory() as temporary_directory:
            temporary_path = Path(temporary_directory)
            shadow_cmd = subject.build_rollout_command(
                manifest, shadow, temporary_path / "shadow"
            )
            active_cmd = subject.build_rollout_command(
                manifest, active, temporary_path / "two_sensor"
            )

        shadow_flag = shadow_cmd.index("--phase-fsm-input-mode")
        active_flag = active_cmd.index("--phase-fsm-input-mode")
        segment_flag = shadow_cmd.index("--segment-duration")
        self.assertEqual(shadow_cmd[segment_flag + 1], "0.01")
        self.assertEqual(shadow_cmd[shadow_flag + 1], "shadow")
        self.assertIn("--record-outputs", shadow_cmd)
        self.assertIn("--record-policy-trace", shadow_cmd)
        self.assertEqual(active_cmd[active_flag + 1], "two_sensor")
        self.assertIn("--no-record-outputs", active_cmd)
        self.assertIn("--no-record-policy-trace", active_cmd)

    def test_dry_run_is_non_mutating_and_reports_runtime_blocker_if_needed(
        self,
    ) -> None:
        manifest = subject.load_and_validate_manifest()
        cases = subject.build_cases(manifest)

        with tempfile.TemporaryDirectory() as temporary_directory:
            output_root = Path(temporary_directory) / "matrix"
            report = subject.dry_run_report(manifest, cases, output_root)
            output_root_exists = output_root.exists()

        self.assertTrue(report["ok"])
        self.assertEqual(report["case_count"], 72)
        self.assertEqual(
            report["heldout_contract"]["heldout_cases_generated"], 0
        )
        self.assertFalse(output_root_exists)
        self.assertEqual(
            report["execution_ready"], report["blocking_reasons"] == []
        )


if __name__ == "__main__":
    unittest.main()
