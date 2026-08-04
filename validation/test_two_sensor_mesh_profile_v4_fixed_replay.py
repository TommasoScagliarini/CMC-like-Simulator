"""Unit tests for the frozen V4/current prescribed geometry comparison."""

from __future__ import annotations

import copy
import json
import sys
import tempfile
import unittest
from pathlib import Path
from types import ModuleType
from typing import Any
from unittest.mock import patch

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
VALIDATION_DIR = REPO_ROOT / "validation"
PROTOCOL_PATH = (
    VALIDATION_DIR / "two_sensor_mesh_profile_v4_fixed_replay_protocol_v4.json"
)
if str(VALIDATION_DIR) not in sys.path:
    sys.path.insert(0, str(VALIDATION_DIR))


class _ForbiddenOpenSim(ModuleType):
    def __getattr__(self, name: str) -> Any:
        raise AssertionError(f"pure V4 fixed-replay test touched opensim.{name}")


with patch.dict(sys.modules, {"opensim": _ForbiddenOpenSim("opensim")}):
    import compare_two_sensor_mesh_profiles_prescribed as subject  # noqa: E402


class TwoSensorMeshProfileV4FixedReplayTest(unittest.TestCase):
    def setUp(self) -> None:
        self.protocol = subject.load_and_validate_protocol(PROTOCOL_PATH)

    def _temporary_protocol(self, payload: dict[str, Any]) -> Path:
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        path = Path(temporary.name) / "protocol.json"
        path.write_text(json.dumps(payload), encoding="utf-8")
        return path

    @staticmethod
    def _passing_row(profile_id: str) -> dict[str, Any]:
        return {
            "candidate_id": profile_id,
            "primary_event_time_field": "confirmed_time_s",
            "exact_reference_and_detector_event_counts": True,
            "precision": 1.0,
            "recall": 1.0,
            "max_abs_hs_error_s": 0.04,
            "max_abs_toe_off_error_s": 0.07,
            "invalid_or_timeout_transition_count": 0,
            "unaccepted_sensor_gait_event_count": 0,
            "exact_hs_to_toe_off_to_hs_order_and_cycle_count": True,
            "confirmation_latency_in_range": True,
            "forbidden_phase_mismatch_count": 0,
            "unknown_fsm_phase_samples": 0,
            "confirmed_fsm_stance_f1": 0.97,
            "confirmed_fsm_stance_iou": 0.95,
            "worst_event_normalized_max_abs_error": 0.875,
        }

    def test_protocol_freezes_exclusive_block_detector_and_cadences(self) -> None:
        self.assertTrue(self.protocol["frozen_before_execution"])
        access = self.protocol["data_access"]
        self.assertEqual(access["already_open_block_s"], [50.0, 100.0])
        self.assertTrue(access["upper_bound_is_exclusive"])
        self.assertFalse(access["allow_samples_at_or_after_100_s"])
        self.assertEqual(access["sealed_block_s"], [100.0, 155.045])
        self.assertEqual(access["expected_right_boundary_cycles_excluded"], 1)
        replay = self.protocol["replay"]
        self.assertEqual(replay["sensor_on_threshold_n"], 0.5)
        self.assertEqual(replay["sensor_off_threshold_n"], 0.25)
        self.assertEqual(replay["sensor_dwell_s"], 0.030)
        self.assertEqual(replay["primary_sample_dt_s"], 0.010)
        self.assertEqual(replay["sensitivity_sample_dt_s"], 0.001)
        self.assertEqual(replay["primary_event_time_field"], "confirmed_time_s")

    def test_protocol_rejects_boundary_threshold_and_timestamp_drift(self) -> None:
        mutations = (
            lambda raw: raw["data_access"].update(
                {"allow_samples_at_or_after_100_s": True}
            ),
            lambda raw: raw["data_access"].update(
                {"already_open_block_s": [50.0, 100.001]}
            ),
            lambda raw: raw["replay"].update({"sensor_on_threshold_n": 0.75}),
            lambda raw: raw["replay"].update({"sensor_dwell_s": 0.020}),
            lambda raw: raw["replay"].update({"primary_sample_dt_s": 0.005}),
            lambda raw: raw["replay"].update(
                {"primary_event_time_field": "event_time_s"}
            ),
        )
        frozen = json.loads(PROTOCOL_PATH.read_text(encoding="utf-8"))
        for mutate in mutations:
            payload = copy.deepcopy(frozen)
            mutate(payload)
            with self.subTest(payload=payload), self.assertRaises(subject.ProtocolError):
                subject.load_and_validate_protocol(
                    self._temporary_protocol(payload)
                )

    def test_protocol_rejects_hash_drift(self) -> None:
        payload = json.loads(PROTOCOL_PATH.read_text(encoding="utf-8"))
        first_source = next(iter(payload["sources"].values()))
        first_source["sha256"] = "0" * 64
        with self.assertRaises(subject.ProtocolError):
            subject.load_and_validate_protocol(self._temporary_protocol(payload))

    def test_time_grids_are_strictly_below_sealed_boundary(self) -> None:
        heel_strikes = np.linspace(51.0, 98.0, 12)
        toe_offs = 0.5 * (heel_strikes[:-1] + heel_strikes[1:])
        events = {"heel_strike": heel_strikes, "toe_off": toe_offs}
        for dt in (0.010, 0.001):
            times, effective_end = subject._build_time_grid(
                events, sample_dt_s=dt
            )
            self.assertLess(effective_end, 100.0)
            self.assertTrue(np.all(times < 100.0))
            self.assertEqual(np.count_nonzero(times >= 100.0), 0)

    def test_integer_grid_never_overshoots_misaligned_effective_end(self) -> None:
        heel_strikes = np.linspace(51.0, 99.08974385950206, 12)
        toe_offs = 0.5 * (heel_strikes[:-1] + heel_strikes[1:])
        events = {"heel_strike": heel_strikes, "toe_off": toe_offs}
        for dt in (0.010, 0.001):
            times, effective_end = subject._build_time_grid(
                events, sample_dt_s=dt
            )
            self.assertLessEqual(times[-1], effective_end)
            self.assertLess(effective_end - times[-1], dt + 1e-12)
            self.assertTrue(np.all(times < 100.0))

    def test_time_grid_rejects_confirmation_that_would_reach_100_s(self) -> None:
        heel_strikes = np.linspace(51.0, 99.98, 12)
        toe_offs = 0.5 * (heel_strikes[:-1] + heel_strikes[1:])
        with self.assertRaises(subject.ProtocolError):
            subject._build_time_grid(
                {"heel_strike": heel_strikes, "toe_off": toe_offs},
                sample_dt_s=0.010,
            )

    def test_v4_uses_same_single_boundary_exclusion_at_both_cadences(self) -> None:
        heel_strikes = np.asarray(
            [51.0, 55.0, 59.0, 63.0, 67.0, 71.0, 75.0, 79.0, 83.0, 87.0, 91.0, 95.0, 99.98]
        )
        toe_offs = 0.5 * (heel_strikes[:-1] + heel_strikes[1:])
        censored_10ms, audit_10ms = subject._exclude_unconfirmable_right_boundary_cycles(
            {"heel_strike": heel_strikes, "toe_off": toe_offs},
            sample_dt_s=0.010,
        )
        censored_1ms, audit_1ms = subject._exclude_unconfirmable_right_boundary_cycles(
            {"heel_strike": heel_strikes, "toe_off": toe_offs},
            sample_dt_s=0.001,
        )
        self.assertEqual(audit_10ms["right_boundary_cycles_excluded"], 1)
        self.assertEqual(audit_1ms["right_boundary_cycles_excluded"], 1)
        np.testing.assert_array_equal(
            censored_10ms["heel_strike"], censored_1ms["heel_strike"]
        )
        np.testing.assert_array_equal(
            censored_10ms["toe_off"], censored_1ms["toe_off"]
        )
        self.assertEqual(censored_10ms["toe_off"].size, toe_offs.size - 1)
        self.assertLess(
            censored_10ms["heel_strike"][-1] + 0.030 + 0.010,
            100.0,
        )

    def test_profile_contract_allows_only_v4_left_toe_location(self) -> None:
        paths = subject._profile_paths(self.protocol)
        current = subject.load_online_grf_profile(
            paths[subject.CURRENT_PROFILE_ID]
        )
        candidate = subject.load_online_grf_profile(paths[subject.V4_PROFILE_ID])
        result = subject._profile_geometry_invariants(current, candidate)
        self.assertTrue(result["ok"])
        self.assertEqual(
            result["changed_runtime_fields"], ["spheres[left_toe].location"]
        )
        self.assertTrue(
            result["candidate_is_explicitly_experimental_not_promoted"]
        )

    def test_comparison_gate_requires_both_cadences(self) -> None:
        current = self._passing_row(subject.CURRENT_PROFILE_ID)
        candidate = self._passing_row(subject.V4_PROFILE_ID)
        rows = {
            "primary_10ms": {
                subject.CURRENT_PROFILE_ID: current,
                subject.V4_PROFILE_ID: candidate,
            },
            "sensitivity_1ms": {
                subject.CURRENT_PROFILE_ID: current,
                subject.V4_PROFILE_ID: candidate,
            },
        }
        result = subject.gate_profile_comparison(self.protocol, rows)
        self.assertTrue(result["ok"])
        with self.assertRaises(subject.ProtocolError):
            subject.gate_profile_comparison(
                self.protocol, {"primary_10ms": rows["primary_10ms"]}
            )

    def test_comparison_gate_rejects_v4_regression(self) -> None:
        current = self._passing_row(subject.CURRENT_PROFILE_ID)
        candidate = self._passing_row(subject.V4_PROFILE_ID)
        candidate["worst_event_normalized_max_abs_error"] = 0.876
        rows = {
            cadence: {
                subject.CURRENT_PROFILE_ID: current,
                subject.V4_PROFILE_ID: candidate,
            }
            for cadence in ("primary_10ms", "sensitivity_1ms")
        }
        result = subject.gate_profile_comparison(self.protocol, rows)
        self.assertFalse(result["ok"])
        self.assertFalse(
            result["cadences"]["primary_10ms"]["checks"]["timing_vs_baseline"]
        )

    def test_regional_continuity_reports_overlap_gap_and_to_error(self) -> None:
        times = np.arange(50.0, 51.01, 0.01)
        heel = np.zeros(times.size, dtype=bool)
        toe = np.zeros(times.size, dtype=bool)
        heel[10:50] = True
        toe[40:80] = True

        replay = {
            "heel_contact": heel.astype(float),
            "toe_contact": toe.astype(float),
            "accepted": [
                {
                    "event": "toe_off",
                    "confirmed_time_s": 50.82,
                    "segment_valid": 1.0,
                }
            ],
        }
        inputs = {
            "times": times,
            "loads": {
                "left_heel": 10.0 * heel.astype(float),
                "left_toe": 7.0 * toe.astype(float),
            },
            "penetrations": {
                "left_heel": np.zeros_like(times),
                "left_toe": np.zeros_like(times),
            },
            "aggregate": 10.0 * heel.astype(float) + 7.0 * toe.astype(float),
            "kinematics": {
                "knee_rad": np.zeros_like(times),
                "ankle_rad": np.zeros_like(times),
            },
            "body_weight_n": 700.0,
            "reference_events": {
                "heel_strike": np.asarray([times[10], times[90]]),
                "toe_off": np.asarray([times[80]]),
            },
        }
        config = subject._current_runtime_fsm_config()
        with (
            patch.object(subject, "_current_runtime_fsm_config", return_value=config),
            patch.object(subject, "_run_production_fsm", return_value=replay),
        ):
            result = subject._regional_continuity_diagnostics(
                inputs, sample_dt_s=0.01
            )
        cycle = result["cycles"][0]
        self.assertEqual(
            result["role"], "diagnostic_only_not_part_of_frozen_acceptance_gate"
        )
        self.assertAlmostEqual(cycle["toe_on_after_hs_s"], 0.3, places=9)
        self.assertAlmostEqual(cycle["heel_toe_overlap_s"], 0.1, places=9)
        self.assertAlmostEqual(cycle["maximum_interior_both_off_gap_s"], 0.0)
        self.assertAlmostEqual(cycle["toe_peak_load_n"], 7.0)
        self.assertAlmostEqual(cycle["confirmed_to_error_s"], 0.02, places=9)

    def test_cli_has_separate_validation_and_plot_destinations(self) -> None:
        args = subject.build_arg_parser().parse_args([])
        self.assertIn("validation/", args.output_dir.replace("\\", "/"))
        self.assertIn("plot/", args.plot_dir.replace("\\", "/"))


if __name__ == "__main__":
    unittest.main()
