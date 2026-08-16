"""Pure tests for the V6 V25-active teacher-action replay collector."""

from __future__ import annotations

import inspect
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import numpy as np

from validation import h0_primary_grf_split_v6_teacher_replay_contract as contract
from validation import run_h0_primary_grf_split_v6_teacher_replay as collector


class ExactFloat32Tests(unittest.TestCase):
    def test_only_frozen_invariant_columns_are_compared(self) -> None:
        expected = np.linspace(-1.0, 1.0, 35, dtype=np.float32)
        observed = expected.copy()
        observed[0] += np.float32(3.0)
        observed[1] -= np.float32(4.0)
        observed[10:25] += np.float32(9.0)
        exact, observed_sha, expected_sha = collector.exact_float32_columns(
            observed,
            expected,
            columns=contract.INVARIANT_COLUMNS,
            np=np,
        )
        self.assertTrue(exact)
        self.assertEqual(observed_sha, expected_sha)

    def test_one_bit_change_inside_invariant_columns_is_detected(self) -> None:
        expected = np.zeros(35, dtype=np.float32)
        observed = expected.copy()
        observed[25] = np.nextafter(np.float32(0.0), np.float32(1.0), dtype=np.float32)
        exact, observed_sha, expected_sha = collector.exact_float32_columns(
            observed,
            expected,
            columns=contract.INVARIANT_COLUMNS,
            np=np,
        )
        self.assertFalse(exact)
        self.assertNotEqual(observed_sha, expected_sha)

    def test_vector_comparison_is_float32_byte_exact(self) -> None:
        expected = np.asarray([0.1, -0.2], dtype=np.float32)
        equal, left_sha, right_sha = collector.exact_float32_vector(
            expected.astype(np.float64),
            expected,
            length=2,
            np=np,
        )
        self.assertTrue(equal)
        self.assertEqual(left_sha, right_sha)
        changed = expected.copy()
        changed[1] = np.nextafter(changed[1], np.float32(0.0), dtype=np.float32)
        self.assertFalse(
            collector.exact_float32_vector(
                changed,
                expected,
                length=2,
                np=np,
            )[0]
        )

    def test_malformed_or_nonfinite_vectors_fail_closed(self) -> None:
        for value in ([0.0] * 34, [0.0] * 34 + [float("nan")]):
            with self.subTest(length=len(value)):
                with self.assertRaises(collector.V6TeacherReplayExecutionError):
                    collector.exact_float32_columns(
                        value,
                        [0.0] * 35,
                        columns=contract.INVARIANT_COLUMNS,
                        np=np,
                    )


class CollectorRoutingTests(unittest.TestCase):
    def test_build_env_config_selects_c_without_leaking_global_overrides(self) -> None:
        case = contract.canonical_case(contract.CASE_IDS[0])
        original = (
            collector.legacy.H0_CONFIG,
            collector.legacy.V25_PROFILE,
            collector.legacy.ANALOG_PROFILE,
        )

        def fake_build_env_config(*, case_id, condition):
            self.assertEqual(case_id, "C")
            self.assertEqual(condition["id"], case["case_id"])
            self.assertEqual(condition["offset_s"], case["episode_start_offset_s"])
            self.assertEqual(condition["seed"], case["runtime_seed"])
            self.assertEqual(collector.legacy.H0_CONFIG, collector.SOURCE_H0_CONFIG)
            self.assertEqual(
                collector.legacy.V25_PROFILE,
                collector.INPUT_PATHS["v25_profile"],
            )
            return {
                "binary_phase_fsm_mode": "binary_active",
                "binary_phase_event_contract_id": (
                    contract.V25_ACTIVE_EVENT_CONTRACT_ID
                ),
                "phase_fsm_input_mode": "legacy_events",
                "online_grf_applied_sides": ["left"],
                "detector_sample_dt_s": contract.EXPECTED_SAMPLE_DT_S,
                "segment_duration": contract.EXPECTED_POLICY_DT_S,
                "episode_duration": contract.EXPECTED_EPISODE_DURATION_S,
                "reward": {"morphology_weight": 0.0},
            }

        with mock.patch.object(
            collector.legacy,
            "build_env_config",
            side_effect=fake_build_env_config,
        ):
            result = collector.build_env_config(case)
        self.assertEqual(result["binary_phase_fsm_mode"], "binary_active")
        self.assertEqual(
            (
                collector.legacy.H0_CONFIG,
                collector.legacy.V25_PROFILE,
                collector.legacy.ANALOG_PROFILE,
            ),
            original,
        )

    def test_all_six_inputs_are_v5_baselines_and_have_500_actions(self) -> None:
        for case_id in contract.CASE_IDS:
            with self.subTest(case_id=case_id):
                case, rows = collector.load_frozen_baseline(case_id)
                self.assertEqual(case["case_id"], case_id)
                self.assertEqual(len(rows), contract.EXPECTED_STEPS)
                self.assertEqual(rows[0]["step"], 1)
                self.assertEqual(rows[-1]["step"], contract.EXPECTED_STEPS)
                self.assertEqual(len(rows[0]["raw_action"]), 2)

    def test_collector_source_freezes_action_replay_and_persist_before_gate(
        self,
    ) -> None:
        worker_source = inspect.getsource(collector._execute_worker)
        write_position = worker_source.index("writer.write_step")
        finalize_position = worker_source.index("writer.finalize_before_gate")
        gate_position = worker_source.index("writer.run_gate")
        self.assertLess(write_position, finalize_position)
        self.assertLess(finalize_position, gate_position)
        self.assertIn(
            "env.step(\n                raw_action\n            )", worker_source
        )
        self.assertIn('"v25_observation": obs_before.tolist()', worker_source)
        self.assertIn(
            '"queried_teacher_mean": queried_teacher_mean.tolist()',
            worker_source,
        )
        self.assertNotIn("optimizer", worker_source.lower())
        self.assertNotIn("train", worker_source.lower())
        run_worker_source = inspect.getsource(collector.run_worker)
        self.assertLess(
            run_worker_source.index("verify_worker_claim"),
            run_worker_source.index("load_frozen_baseline"),
        )

    def test_worker_requires_supervisor_token_and_terminal_ledger_blocks_it(
        self,
    ) -> None:
        destination = collector.canonical_destination(contract.CASE_IDS[0])
        self.assertEqual(
            collector.main(
                [
                    "--worker",
                    "--case",
                    contract.CASE_IDS[0],
                    "--output-dir",
                    str(destination),
                ]
            ),
            2,
        )
        with tempfile.TemporaryDirectory() as temporary:
            ledger = Path(temporary) / "execution_ledger.json"
            ledger.write_text('{"passed": false}\n', encoding="utf-8")
            with (
                mock.patch.object(collector, "EXECUTION_LEDGER", ledger),
                self.assertRaises(collector.V6TeacherReplayExecutionError),
            ):
                collector.verify_worker_claim(
                    contract.CASE_IDS[0],
                    "x" * 32,
                )

    def test_worker_command_carries_ephemeral_token_without_persisting_it(self) -> None:
        token = "secret-token-which-is-long-enough-123"
        command = collector._worker_command(contract.CASE_IDS[0], token)
        self.assertEqual(command[-2:], ["--execution-token", token])
        with mock.patch.object(collector, "_record", return_value={"record": 1}):
            claim = collector._execution_claim_payload(collector._token_sha256(token))
        self.assertNotIn(token, str(claim))
        self.assertEqual(
            claim["execution_token_sha256"],
            collector._token_sha256(token),
        )

    def test_sea_fallback_counter_rejects_missing_or_negative_fields(self) -> None:
        payload = {
            "joints": {
                joint: {
                    "tau_input_plugin_fallback_count": 0,
                    "motor_accel_plugin_fallback_count": 0,
                }
                for joint in ("pros_knee_angle", "pros_ankle_angle")
            }
        }
        self.assertEqual(collector._sea_fallback_count(payload), 0)
        payload["joints"]["pros_knee_angle"]["tau_input_plugin_fallback_count"] = -1
        with self.assertRaises(collector.V6TeacherReplayExecutionError):
            collector._sea_fallback_count(payload)


if __name__ == "__main__":
    unittest.main()
