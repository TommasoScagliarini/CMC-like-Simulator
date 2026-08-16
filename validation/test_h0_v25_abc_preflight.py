"""Contract tests for the one-shot H0/V25 A/B/C preflight tools."""

from __future__ import annotations

import copy
import math
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np


from validation import compare_h0_v25_abc as comparator
from validation import run_h0_v25_abc_preflight as driver


def metric(value: float = 1.0, count: int = 500) -> dict:
    return {"sample_count": count, "rms": value, "abs_max": value}


def summary(value: float = 1.0) -> dict:
    sea = {}
    for joint in comparator.JOINTS:
        sea[joint] = {
            signal: metric(value, 5000)
            for signal in comparator.SEA_SIGNALS
        }
        sea[joint]["tau_input_saturated"] = {
            "sample_count": 5000,
            "count": 0,
            "fraction": 0.0,
        }
    return {
        "steps": 500,
        "end_reason": "episode_time_limit",
        "terminated": False,
        "truncated": True,
        "phase_valid_cycle_count": 2,
        "grf_penetration_max_m": 0.01,
        "action_clipped_values": 0,
        "timeout_count": 0,
        "safety_stop_count": 0,
        "fallback_count": 0,
        "hard_invalid_count": 0,
        "nonfinite_count": 0,
        "n_actor": 35,
        "n_observation": 84,
        "observation_dtype": "float32",
        "morphology_weight": 0.0,
        "episode_metrics": {
            "reserve_norm_nm": metric(value),
            "residual_norm_nm": metric(value),
        },
        "sea_episode_metrics": sea,
        "binary_phase_event_gate": {
            "passed": True,
            "sample_count": 5000,
            "hard_invalid_count": 0,
            "fallback_count": 0,
            "duplicate_event_count": 0,
            "out_of_order_event_count": 0,
            "left_non_v25_source_count": 0,
        },
    }


def journal() -> dict:
    samples = [
        {
            "time_s": index / 1000.0,
            "left_heel_contact": bool(index % 2),
            "left_toe_contact": False,
        }
        for index in range(1, 5001)
    ]
    return {
        "schema_version": 1,
        "sample_dt_s": 0.001,
        "baseline": {
            "time_s": 0.0,
            "left_heel_contact": False,
            "left_toe_contact": False,
        },
        "samples": samples,
    }


class H0V25ComparatorTests(unittest.TestCase):
    def test_ab_excludes_only_top_level_binary_namespace(self) -> None:
        a_trace = [
            {"step": index + 1, "value": 1.0, "binary_phase_fsm": None}
            for index in range(500)
        ]
        b_trace = copy.deepcopy(a_trace)
        for row in b_trace:
            row["binary_phase_fsm"] = {"shadow": True}
        a_summary = summary()
        b_summary = copy.deepcopy(a_summary)
        a_summary["binary_phase_fsm_mode"] = "disabled"
        b_summary["binary_phase_fsm_mode"] = "binary_shadow"
        result = comparator.compare_ab(
            a_trace=a_trace,
            b_trace=b_trace,
            a_summary=a_summary,
            b_summary=b_summary,
            a_journal=journal(),
            b_journal=journal(),
        )
        self.assertTrue(result["passed"])

        b_trace[127]["nested"] = {"binary_phase_hidden": 1}
        result = comparator.compare_ab(
            a_trace=a_trace,
            b_trace=b_trace,
            a_summary=a_summary,
            b_summary=b_summary,
            a_journal=journal(),
            b_journal=journal(),
        )
        self.assertFalse(result["passed"])

    def test_ab_never_truncates_or_tolerates_raw_journal_drift(self) -> None:
        rows = [{"step": index + 1} for index in range(500)]
        short = rows[:-1]
        result = comparator.compare_ab(
            a_trace=rows,
            b_trace=short,
            a_summary=summary(),
            b_summary=summary(),
            a_journal=journal(),
            b_journal=journal(),
        )
        self.assertFalse(result["passed"])
        changed = journal()
        changed["samples"][2500]["left_toe_contact"] = True
        result = comparator.compare_ab(
            a_trace=rows,
            b_trace=rows,
            a_summary=summary(),
            b_summary=summary(),
            a_journal=journal(),
            b_journal=changed,
        )
        self.assertFalse(result["passed"])

    def test_c_gate_uses_condition_matched_caps_and_no_saturation_tolerance(self) -> None:
        reference = summary(1.0)
        candidate = summary(1.0 + 1e-7)
        passed = comparator.gate_c(a_summary=reference, c_summary=candidate)
        self.assertTrue(passed["passed"])
        candidate["sea_episode_metrics"]["pros_knee_angle"][
            "tau_input_saturated"
        ]["count"] = 1
        candidate["sea_episode_metrics"]["pros_knee_angle"][
            "tau_input_saturated"
        ]["fraction"] = 1 / 5000
        failed = comparator.gate_c(a_summary=reference, c_summary=candidate)
        self.assertFalse(failed["passed"])

    def test_strict_atomic_writer_is_no_clobber(self) -> None:
        with tempfile.TemporaryDirectory(dir=driver.VALIDATION_ROOT) as raw:
            path = Path(raw) / "value.json"
            comparator.write_json_exclusive(path, {"value": 1.0})
            with self.assertRaisesRegex(comparator.H0V25GateError, "clobber"):
                comparator.write_json_exclusive(path, {"value": 2.0})
            with self.assertRaises(comparator.H0V25GateError):
                comparator.write_json_exclusive(Path(raw) / "nan.json", {"x": math.nan})


class H0V25DriverPureTests(unittest.TestCase):
    def test_a_b_configs_differ_only_by_fsm_mode(self) -> None:
        condition = driver._condition("det_nominal")
        a = driver.build_env_config(case_id="A", condition=condition)
        b = driver.build_env_config(case_id="B", condition=condition)
        self.assertEqual(a.pop("binary_phase_fsm_mode"), "disabled")
        self.assertEqual(b.pop("binary_phase_fsm_mode"), "binary_shadow")
        self.assertEqual(a, b)
        self.assertEqual(a["binary_phase_event_contract_id"], driver.V25_SHADOW_CONTRACT)
        self.assertEqual(a["reward"]["morphology_weight"], 0.0)

    def test_noise_tape_is_float32_and_replay_exact(self) -> None:
        rng = np.random.default_rng(123)
        tape = rng.standard_normal((500, 2)).astype(np.float32)
        replay = np.asarray(tape.tolist(), dtype=np.float32)
        self.assertTrue(driver._array_exact(tape, replay, np))
        replay[17, 1] = np.nextafter(replay[17, 1], np.float32(np.inf))
        self.assertFalse(driver._array_exact(tape, replay, np))

    def test_sea_sufficient_statistics_finalize_exactly(self) -> None:
        rows = [
            {
                "pros_knee_angle": {
                    "time_s": 0.0,
                    "tau_spring_nm": 1.0,
                    "tau_input_raw_nm": 2.0,
                    "tau_input_nm": 2.0,
                    "tau_input_saturated": 0.0,
                    "torque_error_nm": 3.0,
                    "motor_speed_rad_s": 4.0,
                    "motor_accel_rad_s2": 5.0,
                    "joint_power_w": 6.0,
                    "motor_power_w": 7.0,
                    "tau_input_plugin_fallback": 0.0,
                    "motor_accel_plugin_fallback": 0.0,
                },
                "pros_ankle_angle": {
                    "time_s": 0.0,
                    "tau_spring_nm": 1.0,
                    "tau_input_raw_nm": 2.0,
                    "tau_input_nm": 2.0,
                    "tau_input_saturated": 0.0,
                    "torque_error_nm": 3.0,
                    "motor_speed_rad_s": 4.0,
                    "motor_accel_rad_s2": 5.0,
                    "joint_power_w": 6.0,
                    "motor_power_w": 7.0,
                    "tau_input_plugin_fallback": 0.0,
                    "motor_accel_plugin_fallback": 0.0,
                },
            },
            {
                "pros_knee_angle": {
                    "time_s": 0.001,
                    "tau_spring_nm": 3.0,
                    "tau_input_raw_nm": 2.0,
                    "tau_input_nm": 2.0,
                    "tau_input_saturated": 0.0,
                    "torque_error_nm": 5.0,
                    "motor_speed_rad_s": 6.0,
                    "motor_accel_rad_s2": 7.0,
                    "joint_power_w": 8.0,
                    "motor_power_w": 9.0,
                    "tau_input_plugin_fallback": 0.0,
                    "motor_accel_plugin_fallback": 0.0,
                },
                "pros_ankle_angle": {
                    "time_s": 0.001,
                    "tau_spring_nm": 3.0,
                    "tau_input_raw_nm": 2.0,
                    "tau_input_nm": 2.0,
                    "tau_input_saturated": 0.0,
                    "torque_error_nm": 5.0,
                    "motor_speed_rad_s": 6.0,
                    "motor_accel_rad_s2": 7.0,
                    "joint_power_w": 8.0,
                    "motor_power_w": 9.0,
                    "tau_input_plugin_fallback": 0.0,
                    "motor_accel_plugin_fallback": 0.0,
                },
            },
        ]
        from simulation_runner import SimulationRunner

        payload = SimulationRunner._summarize_sea_segment_diagnostics(rows)
        self.assertEqual(
            payload["joints"]["pros_knee_angle"][
                "torque_error_sum_squares_nm2"
            ],
            34.0,
        )
        self.assertEqual(
            payload["joints"]["pros_knee_angle"][
                "torque_error_abs_max_nm"
            ],
            5.0,
        )
        self.assertEqual(
            payload["joints"]["pros_knee_angle"][
                "tau_input_saturation_count"
            ],
            0,
        )


if __name__ == "__main__":
    unittest.main()
