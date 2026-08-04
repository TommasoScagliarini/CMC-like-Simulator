"""Pure regression tests for the staged V11 heel-x experiment.

The suite intentionally avoids OpenSim execution.  It freezes the protocol,
candidate geometry, shared-station accounting, conditional Stage-B trigger,
confirmed-event timing, no-clobber behaviour, and sealed-data boundary before
the development sweep is allowed to run.
"""

from __future__ import annotations

import copy
import json
import math
import sys
import tempfile
import unittest
from dataclasses import dataclass
from pathlib import Path
from types import ModuleType
from typing import Any
from unittest.mock import patch

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
VALIDATION_ROOT = REPO_ROOT / "validation"
PROTOCOL_PATH = VALIDATION_ROOT / "two_sensor_heel_x_sweep_protocol_v11.json"
EXPECTED_STAGE_A_X_SHIFTS_MM = (
    -0.500,
    -0.250,
    -0.150,
    0.150,
    0.250,
    0.500,
    0.750,
    1.000,
    1.250,
    1.500,
    1.750,
    2.000,
    2.250,
    2.500,
    2.750,
    3.000,
    3.250,
    3.500,
    4.000,
)
EXPECTED_STAGE_B_TOE_DOWN_MM = (
    0.050,
    0.100,
    0.150,
    0.200,
    0.250,
    0.300,
    0.400,
    0.500,
)
if str(VALIDATION_ROOT) not in sys.path:
    sys.path.insert(0, str(VALIDATION_ROOT))


class _ForbiddenOpenSim(ModuleType):
    def __getattr__(self, name: str) -> Any:
        raise AssertionError(f"pure V11 test touched opensim.{name}")


with patch.dict(sys.modules, {"opensim": _ForbiddenOpenSim("opensim")}):
    import sweep_two_sensor_heel_x_prescribed_v11 as subject  # noqa: E402


@dataclass(frozen=True)
class _SetupStub:
    """Minimum dataclass accepted by ``dataclasses.replace`` in the time guard."""

    t_start: float = 0.0
    t_end: float = 0.0


class TwoSensorHeelXSweepV11Test(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.protocol = subject.load_and_validate_protocol(PROTOCOL_PATH)
        (
            cls.stage_a_base,
            cls.stage_a_candidates,
            cls.stage_a_summary,
        ) = subject.build_stage_a_candidates(cls.protocol)
        cls.stage_a_sampler, cls.stage_a_pairs, cls.stage_a_profiles = (
            subject._sampling_bundle(
                cls.stage_a_base,
                cls.stage_a_candidates,
                stage_label="stage_a_test",
                expected_detector_stations=subject.STAGE_A_DETECTOR_STATIONS,
            )
        )
        cls.stage_b_anchor = next(
            candidate
            for candidate in cls.stage_a_candidates
            if candidate.selectable
            and math.isclose(
                float(candidate.geometry["heel_x_shift_mm"]),
                0.150,
                rel_tol=0.0,
                abs_tol=1.0e-12,
            )
        )
        (
            cls.stage_b_base,
            cls.stage_b_candidates,
            cls.stage_b_summary,
        ) = subject.build_stage_b_candidates(cls.protocol, cls.stage_b_anchor)
        cls.stage_b_sampler, cls.stage_b_pairs, cls.stage_b_profiles = (
            subject._sampling_bundle(
                cls.stage_b_base,
                cls.stage_b_candidates,
                stage_label="stage_b_test",
                expected_detector_stations=subject.STAGE_B_DETECTOR_STATIONS,
            )
        )

    def _temporary_protocol(self, payload: dict[str, Any]) -> Path:
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        path = Path(temporary.name) / "protocol.json"
        path.write_text(json.dumps(payload), encoding="utf-8")
        return path

    @staticmethod
    def _row(
        candidate: Any,
        *,
        hs_error_s: float = 0.060,
        transfer_gap_samples: int = 0,
        toe_down_mm: float | None = None,
    ) -> dict[str, Any]:
        """Build a complete synthetic row consumed by the real strict gate."""

        return {
            "candidate_id": candidate.candidate_id,
            "selectable": bool(candidate.selectable),
            "heel_x_shift_mm": float(candidate.geometry["heel_x_shift_mm"]),
            "toe_center_down_mm": (
                float(candidate.geometry["toe_center_down_mm"])
                if toe_down_mm is None
                else float(toe_down_mm)
            ),
            "reference_hs_count": subject.EXPECTED_REFERENCE_HS,
            "reference_to_count": subject.EXPECTED_REFERENCE_TO,
            "predicted_hs_count": subject.EXPECTED_REFERENCE_HS,
            "predicted_to_count": subject.EXPECTED_REFERENCE_TO,
            "observed_valid_cycle_count": subject.EXPECTED_CYCLES,
            "precision": 1.0,
            "recall": 1.0,
            "max_abs_hs_error_s": float(hs_error_s),
            "max_abs_toe_off_error_s": 0.030,
            "confirmed_fsm_stance_f1": 0.99,
            "confirmed_fsm_stance_iou": 0.95,
            "transfer_both_latches_off_sample_count": int(
                transfer_gap_samples
            ),
            "incomplete_heel_to_forefoot_transfer_count": 0,
            "to_candidates_before_min_stance_count": 0,
            "invalid_or_timeout_transition_count": 0,
            "unaccepted_sensor_gait_event_count": 0,
            "forbidden_phase_mismatch_count": 0,
            "unknown_fsm_phase_samples": 0,
            "minimum_causal_toe_clear_before_next_hs_onset_s": 0.040,
            "causal_swing_interval_count": subject.EXPECTED_CYCLES,
            "confirmation_latency_in_range": True,
            "exact_hs_to_toe_off_to_hs_order_and_cycle_count": True,
            "mesh_geometry_pre_gate_ok": True,
            "event_count_deficit": 0,
            "invalid_timeout_plus_unaccepted_count": 0,
            "early_max_abs_hs_error_s": float(hs_error_s),
            "long_max_abs_hs_error_s": float(hs_error_s),
            "global_mean_abs_hs_error_s": float(hs_error_s) / 2.0,
        }

    @staticmethod
    def _source_payload() -> dict[str, Any]:
        return json.loads(PROTOCOL_PATH.read_text(encoding="utf-8"))

    def test_protocol_and_all_declared_sources_are_hash_pinned(self) -> None:
        self.assertEqual(subject.SCHEMA_VERSION, 11)
        self.assertTrue(self.protocol["frozen_before_execution"])
        self.assertEqual(
            self.protocol["_protocol_sha256"],
            subject.v1._sha256(PROTOCOL_PATH),
        )
        self.assertEqual(
            self.protocol["_protocol_path"], PROTOCOL_PATH.resolve().as_posix()
        )
        sources = self.protocol["sources"]
        self.assertTrue(sources)
        self.assertEqual(set(sources), set(subject.REQUIRED_SOURCE_PATHS))
        for label, record in sources.items():
            self.assertEqual(record["path"], subject.REQUIRED_SOURCE_PATHS[label])
            source_path = subject.v1.resolve_repo_path(record["path"]).resolve()
            self.assertTrue(source_path.is_file(), label)
            self.assertEqual(record["sha256"], subject.v1._sha256(source_path), label)

        self.assertEqual(self.protocol["objective"], subject.OBJECTIVE)
        self.assertEqual(
            self.protocol["interpretation_limits"], subject.INTERPRETATION_LIMITS
        )

        replay = self.protocol["replay"]
        self.assertEqual(replay["runtime_sample_dt_s"], 0.010)
        self.assertEqual(replay["fine_sample_dt_s"], 0.001)
        self.assertTrue(replay["evaluate_all_candidates_at_both_resolutions"])
        self.assertEqual(replay["primary_event_time_field"], "confirmed_time_s")
        self.assertEqual(replay["diagnostic_event_time_field"], "event_time_s")
        self.assertEqual(
            self.protocol["stage_b_conditional_toe_compensation"]["trigger"],
            subject.STAGE_B_TRIGGER,
        )
        selection = self.protocol["selection"]
        self.assertEqual(selection["stage_a_ranking"], subject.STAGE_A_RANKING)
        self.assertEqual(selection["stage_b_ranking"], subject.STAGE_B_RANKING)
        self.assertTrue(
            selection[
                "stage_a_anchor_allows_only_toe_compensable_structural_failures"
            ]
        )
        self.assertEqual(
            selection["stage_a_anchor_required_root_gate_failures"],
            sorted(subject.TOE_COMPENSABLE_ROOT_GATE_FAILURES),
        )
        self.assertEqual(
            selection["stage_a_anchor_allowed_derived_gate_failures"],
            sorted(subject.TOE_COMPENSABLE_DERIVED_GATE_FAILURES),
        )

    def test_protocol_rejects_grid_timestamp_and_source_hash_drift(self) -> None:
        frozen = self._source_payload()

        grid_drift = copy.deepcopy(frozen)
        grid_drift["stage_a_heel_x"]["heel_local_x_shift_from_v9_mm"][0] = -0.4
        with self.assertRaises(subject.ProtocolError):
            subject.load_and_validate_protocol(self._temporary_protocol(grid_drift))

        timestamp_drift = copy.deepcopy(frozen)
        timestamp_drift["replay"]["primary_event_time_field"] = "event_time_s"
        with self.assertRaises(subject.ProtocolError):
            subject.load_and_validate_protocol(
                self._temporary_protocol(timestamp_drift)
            )

        source_drift = copy.deepcopy(frozen)
        first_source = next(iter(source_drift["sources"].values()))
        first_source["sha256"] = "0" * 64
        with self.assertRaises(subject.ProtocolError):
            subject.load_and_validate_protocol(self._temporary_protocol(source_drift))

    def test_stage_a_count_and_only_heel_local_x_changes(self) -> None:
        self.assertEqual(
            tuple(subject.STAGE_A_X_SHIFTS_MM), EXPECTED_STAGE_A_X_SHIFTS_MM
        )
        self.assertEqual(subject.STAGE_A_SELECTABLE_COUNT, 19)
        self.assertEqual(subject.STAGE_A_PAIR_COUNT, 20)
        self.assertEqual(len(self.stage_a_candidates), 20)
        self.assertEqual(
            sum(candidate.selectable for candidate in self.stage_a_candidates),
            19,
        )
        self.assertEqual(
            len({candidate.candidate_id for candidate in self.stage_a_candidates}),
            20,
        )

        baseline = subject.v1._left_sensor_spheres(self.stage_a_base)
        heel_0 = baseline["left_heel"]
        toe_0 = baseline["left_toe"]
        observed_shifts: list[float] = []
        for candidate in self.stage_a_candidates:
            sensors = subject.v1._left_sensor_spheres(
                self.stage_a_profiles[candidate.candidate_id]
            )
            heel = sensors["left_heel"]
            toe = sensors["left_toe"]
            shift_mm = float(candidate.geometry["heel_x_shift_mm"])

            self.assertAlmostEqual(
                heel.location[0], heel_0.location[0] + shift_mm / 1000.0, 14
            )
            self.assertEqual(tuple(heel.location[1:]), tuple(heel_0.location[1:]))
            self.assertEqual(tuple(toe.location), tuple(toe_0.location))
            self.assertEqual(heel.radius, heel_0.radius)
            self.assertEqual(toe.radius, toe_0.radius)
            self.assertEqual(float(candidate.geometry["toe_center_down_mm"]), 0.0)
            if candidate.selectable:
                observed_shifts.append(shift_mm)
            else:
                self.assertEqual(candidate.candidate_id, subject.BASELINE_ID)
                self.assertEqual(shift_mm, 0.0)

        np.testing.assert_allclose(
            sorted(observed_shifts),
            sorted(EXPECTED_STAGE_A_X_SHIFTS_MM),
            rtol=0.0,
            atol=1.0e-12,
        )
        self.assertTrue(self.stage_a_summary["only_heel_local_x_changes"])

    def test_stage_b_count_and_only_toe_local_y_changes(self) -> None:
        self.assertEqual(
            tuple(subject.STAGE_B_TOE_DOWN_MM), EXPECTED_STAGE_B_TOE_DOWN_MM
        )
        self.assertEqual(subject.STAGE_B_SELECTABLE_COUNT, 8)
        self.assertEqual(subject.STAGE_B_PAIR_COUNT, 9)
        self.assertEqual(len(self.stage_b_candidates), 9)
        self.assertEqual(
            sum(candidate.selectable for candidate in self.stage_b_candidates),
            8,
        )

        baseline = subject.v1._left_sensor_spheres(self.stage_b_base)
        heel_0 = baseline["left_heel"]
        toe_0 = baseline["left_toe"]
        anchor_heel = tuple(self.stage_b_anchor.heel_location)
        observed_down: list[float] = []
        for candidate in self.stage_b_candidates:
            sensors = subject.v1._left_sensor_spheres(
                self.stage_b_profiles[candidate.candidate_id]
            )
            heel = sensors["left_heel"]
            toe = sensors["left_toe"]
            down_mm = float(candidate.geometry["toe_center_down_mm"])

            self.assertEqual(tuple(heel.location), anchor_heel)
            self.assertEqual(tuple(toe.location)[::2], tuple(toe_0.location)[::2])
            self.assertAlmostEqual(
                toe.location[1], toe_0.location[1] - down_mm / 1000.0, 14
            )
            self.assertEqual(heel.radius, heel_0.radius)
            self.assertEqual(toe.radius, toe_0.radius)
            self.assertEqual(
                float(candidate.geometry["heel_x_shift_mm"]),
                float(self.stage_b_anchor.geometry["heel_x_shift_mm"]),
            )
            self.assertEqual(
                candidate.geometry["stage_a_anchor_id"],
                self.stage_b_anchor.candidate_id,
            )
            if candidate.selectable:
                observed_down.append(down_mm)
            else:
                self.assertEqual(down_mm, 0.0)

        np.testing.assert_allclose(
            observed_down,
            EXPECTED_STAGE_B_TOE_DOWN_MM,
            rtol=0.0,
            atol=1.0e-12,
        )
        self.assertTrue(self.stage_b_summary["only_toe_local_y_changes"])
        self.assertTrue(self.stage_b_summary["all_radii_fixed_to_v9"])

    def test_shared_station_counts_match_each_stage_contract(self) -> None:
        self.assertEqual(subject.STAGE_A_DETECTOR_STATIONS, 21)
        self.assertEqual(subject.STAGE_A_TOTAL_STATIONS, 29)
        self.assertEqual(len(self.stage_a_sampler.spheres), 21)
        self.assertEqual(len(self.stage_a_profiles), 20)
        self.assertEqual(len(set(self.stage_a_pairs)), 20)
        self.assertEqual(
            len({sphere.name for sphere in self.stage_a_sampler.spheres}), 21
        )

        self.assertEqual(subject.STAGE_B_DETECTOR_STATIONS, 10)
        self.assertEqual(subject.STAGE_B_TOTAL_STATIONS, 18)
        self.assertEqual(len(self.stage_b_sampler.spheres), 10)
        self.assertEqual(len(self.stage_b_profiles), 9)
        self.assertEqual(len(set(self.stage_b_pairs)), 9)
        self.assertEqual(
            len({sphere.name for sphere in self.stage_b_sampler.spheres}), 10
        )
        for profiles in (self.stage_a_profiles, self.stage_b_profiles):
            self.assertTrue(
                all(len(profile.spheres) == 2 for profile in profiles.values())
            )

        sampling = self.protocol["sampling"]
        for key, detector, total, pairs in (
            ("stage_a", 21, 29, 20),
            ("stage_b_if_triggered", 10, 18, 9),
        ):
            for cadence in ("runtime_10ms", "fine_1ms"):
                contract = sampling[key][cadence]
                self.assertEqual(contract["expected_unique_detector_stations"], detector)
                self.assertEqual(contract["expected_total_sampled_stations"], total)
                self.assertEqual(contract["evaluated_pair_count"], pairs)

    def test_stage_b_runs_only_when_no_strict_a_winner_has_timing_anchor(self) -> None:
        rows = [self._row(candidate) for candidate in self.stage_a_candidates]
        strict_id = self.stage_b_anchor.candidate_id
        for row in rows:
            if row["candidate_id"] == strict_id:
                row["max_abs_hs_error_s"] = 0.020
                row["early_max_abs_hs_error_s"] = 0.020
                row["long_max_abs_hs_error_s"] = 0.020
        strict = subject.select_stage_a(
            copy.deepcopy(rows), copy.deepcopy(rows), self.protocol
        )
        self.assertEqual(strict["strict_winner_id"], strict_id)
        self.assertIsNone(strict["stage_b_anchor_id"])
        self.assertEqual(
            strict["stage_b_trigger_decision"],
            "SKIPPED_STAGE_A_STRICT_WINNER",
        )

        for row in rows:
            if row["candidate_id"] == strict_id:
                row["transfer_both_latches_off_sample_count"] = 1
        compensated = subject.select_stage_a(
            copy.deepcopy(rows), copy.deepcopy(rows), self.protocol
        )
        self.assertIsNone(compensated["strict_winner_id"])
        self.assertEqual(compensated["stage_b_anchor_id"], strict_id)
        self.assertEqual(
            compensated["stage_b_trigger_decision"],
            "RUN_HS_QUALIFIED_X_WITH_TOE_COMPENSABLE_STRUCTURAL_FAILURES",
        )

        for row in rows:
            if row["candidate_id"] == strict_id:
                row["minimum_causal_toe_clear_before_next_hs_onset_s"] = 0.0
        noncompensable = subject.select_stage_a(
            copy.deepcopy(rows), copy.deepcopy(rows), self.protocol
        )
        self.assertIsNone(noncompensable["strict_winner_id"])
        self.assertIsNone(noncompensable["stage_b_anchor_id"])
        self.assertEqual(
            noncompensable["stage_b_trigger_decision"],
            "SKIPPED_NO_TOE_COMPENSABLE_STAGE_A_ANCHOR",
        )

        for row in rows:
            if row["candidate_id"] == strict_id:
                row["minimum_causal_toe_clear_before_next_hs_onset_s"] = 0.040
                row["max_abs_hs_error_s"] = 0.060
                row["early_max_abs_hs_error_s"] = 0.060
                row["long_max_abs_hs_error_s"] = 0.060
        no_anchor = subject.select_stage_a(
            copy.deepcopy(rows), copy.deepcopy(rows), self.protocol
        )
        self.assertIsNone(no_anchor["strict_winner_id"])
        self.assertIsNone(no_anchor["stage_b_anchor_id"])
        self.assertEqual(
            no_anchor["stage_b_trigger_decision"],
            "SKIPPED_NO_TOE_COMPENSABLE_STAGE_A_ANCHOR",
        )

    def test_confirmed_time_is_primary_and_onset_remains_diagnostic(self) -> None:
        detail = {
            "events": {
                "reference": {"heel_strike": np.asarray([20.0])},
                "confirmed": {"heel_strike": np.asarray([20.060])},
                "onset": {"heel_strike": np.asarray([20.000])},
            }
        }
        diagnostics = subject._critical_hs_diagnostics(detail)
        self.assertEqual(diagnostics["out_of_50ms_count"], 1)
        event = diagnostics["out_of_50ms"][0]
        self.assertAlmostEqual(event["confirmed_time_s"], 20.060)
        self.assertAlmostEqual(event["onset_time_s"], 20.000)
        self.assertAlmostEqual(event["absolute_error_s"], 0.060)
        self.assertEqual(self.protocol["_primary_event_time_field"], "confirmed_time_s")
        self.assertEqual(self.protocol["_diagnostic_event_time_field"], "event_time_s")

        unequal = subject._critical_hs_diagnostics(
            {
                "events": {
                    "reference": {"heel_strike": np.asarray([20.0, 21.0])},
                    "confirmed": {"heel_strike": np.asarray([20.020])},
                    "onset": {"heel_strike": np.asarray([20.000])},
                }
            }
        )
        self.assertFalse(unequal["equal_counts"])
        self.assertEqual(unequal["reference_count"], 2)
        self.assertEqual(unequal["confirmed_count"], 1)
        self.assertEqual(unequal["onset_count"], 1)
        self.assertIsNone(unequal["out_of_50ms_count"])
        self.assertEqual(unequal["out_of_50ms"], [])

    def test_no_clobber_rejects_existing_empty_directories(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            for occupied_name, free_name in (("output", "plot"), ("plot", "output")):
                with self.subTest(occupied=occupied_name):
                    occupied = root / f"{occupied_name}_occupied"
                    free = root / f"{free_name}_free"
                    occupied.mkdir()
                    output = occupied if occupied_name == "output" else free
                    plot_dir = occupied if occupied_name == "plot" else free
                    with self.assertRaises(subject.NoClobberError):
                        subject._preflight_no_clobber(output, plot_dir)
                    self.assertTrue(occupied.is_dir())
                    self.assertEqual(list(occupied.iterdir()), [])
                    self.assertFalse(free.exists())

    def test_open_time_grids_never_touch_the_sealed_100s_boundary(self) -> None:
        access = self.protocol["data_access"]
        self.assertEqual(access["already_open_interval_s"], [subject.OPEN_START_S, 100.0])
        self.assertTrue(access["upper_bound_is_exclusive"])
        self.assertFalse(access["allow_samples_at_or_after_100_s"])
        self.assertEqual(access["sealed_block_status"], "CLOSED_UNEVALUATED")
        self.assertFalse(self.protocol["decision_contract"]["sealed_validation_allowed"])

        events = {
            "heel_strike": np.linspace(14.0, 99.0, subject.EXPECTED_REFERENCE_HS),
            "toe_off": np.linspace(14.5, 98.5, subject.EXPECTED_REFERENCE_TO),
        }
        boundary = {"excluded_closing_hs_s": [99.96878691565038]}
        with (
            patch.object(
                subject.v10.v1,
                "read_setup_xml",
                return_value=_SetupStub(),
            ),
            patch.object(
                subject.v10.v1,
                "_reference_events_from_prescribed_grf",
                return_value=(events, {"synthetic": True}),
            ),
            patch.object(
                subject.v10.v1,
                "_exclude_unconfirmable_right_boundary_cycles",
                return_value=(events, boundary),
            ),
        ):
            for cadence in (subject.PRIMARY_DT_S, subject.FINE_DT_S):
                _setup, _events, audit, times = subject.v10._reference_bundle(
                    self.protocol, sample_dt_s=cadence
                )
                self.assertGreater(times.size, 1)
                self.assertTrue(np.all(times < subject.OPEN_END_S))
                self.assertLess(audit["last_sample_s"], subject.OPEN_END_S)
                self.assertEqual(audit["samples_at_or_after_100_s"], 0)
                self.assertFalse(audit["sealed_block_opened"])

        bad_events = dict(events)
        bad_events["heel_strike"] = np.linspace(
            14.0, 99.980, subject.EXPECTED_REFERENCE_HS
        )
        with (
            patch.object(
                subject.v10.v1,
                "read_setup_xml",
                return_value=_SetupStub(),
            ),
            patch.object(
                subject.v10.v1,
                "_reference_events_from_prescribed_grf",
                return_value=(bad_events, {"synthetic": True}),
            ),
            patch.object(
                subject.v10.v1,
                "_exclude_unconfirmable_right_boundary_cycles",
                return_value=(bad_events, boundary),
            ),
        ):
            for cadence in (subject.PRIMARY_DT_S, subject.FINE_DT_S):
                with self.assertRaises(subject.v10.ProtocolError):
                    subject.v10._reference_bundle(
                        self.protocol, sample_dt_s=cadence
                    )


if __name__ == "__main__":
    unittest.main()
