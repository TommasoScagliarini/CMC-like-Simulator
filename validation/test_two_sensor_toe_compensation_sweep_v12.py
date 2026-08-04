"""Pure regression tests for the isolated V12 toe-center extension.

The suite does not execute OpenSim.  It freezes the V11 boundary comparator,
the only permitted geometry axis, station accounting, multiresolution strict
selection, source identity, no-clobber behaviour, and the open-data boundary.
"""

from __future__ import annotations

import copy
import csv
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
PROTOCOL_PATH = (
    VALIDATION_ROOT / "two_sensor_toe_compensation_sweep_protocol_v12.json"
)
EXPECTED_TOE_DOWN_GRID_MM = (
    0.55,
    0.60,
    0.65,
    0.70,
    0.75,
    0.80,
    0.90,
    1.00,
)
if str(VALIDATION_ROOT) not in sys.path:
    sys.path.insert(0, str(VALIDATION_ROOT))


class _ForbiddenOpenSim(ModuleType):
    def __getattr__(self, name: str) -> Any:
        raise AssertionError(f"pure V12 test touched opensim.{name}")


with patch.dict(sys.modules, {"opensim": _ForbiddenOpenSim("opensim")}):
    import sweep_two_sensor_toe_compensation_prescribed_v12 as subject  # noqa: E402


@dataclass(frozen=True)
class _SetupStub:
    """Minimum dataclass accepted by ``dataclasses.replace`` in the time guard."""

    t_start: float = 0.0
    t_end: float = 0.0


class TwoSensorToeCompensationSweepV12Test(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.protocol = subject.load_and_validate_protocol(PROTOCOL_PATH)
        (
            cls.base,
            cls.candidates,
            cls.v11_boundary,
            cls.geometry,
        ) = subject.build_candidates(cls.protocol)
        cls.comparator = next(
            candidate
            for candidate in cls.candidates
            if candidate.candidate_id == subject.COMPARATOR_ID
        )
        cls.sampler, cls.pairs, cls.profiles = subject.v11._sampling_bundle(
            cls.base,
            cls.candidates,
            stage_label="v12_test",
            expected_detector_stations=subject.DETECTOR_STATIONS,
        )

    def _temporary_protocol(self, payload: dict[str, Any]) -> Path:
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        path = Path(temporary.name) / "protocol.json"
        path.write_text(json.dumps(payload), encoding="utf-8")
        return path

    @staticmethod
    def _source_payload() -> dict[str, Any]:
        return json.loads(PROTOCOL_PATH.read_text(encoding="utf-8"))

    @staticmethod
    def _row(candidate: Any, *, hs_error_s: float) -> dict[str, Any]:
        """Build a complete synthetic row consumed by the real strict gate."""

        return {
            "candidate_id": candidate.candidate_id,
            "selectable": bool(candidate.selectable),
            "heel_x_shift_mm": float(candidate.geometry["heel_x_shift_mm"]),
            "toe_center_down_mm": float(
                candidate.geometry["toe_center_down_mm"]
            ),
            "reference_hs_count": subject.v11.EXPECTED_REFERENCE_HS,
            "reference_to_count": subject.v11.EXPECTED_REFERENCE_TO,
            "predicted_hs_count": subject.v11.EXPECTED_REFERENCE_HS,
            "predicted_to_count": subject.v11.EXPECTED_REFERENCE_TO,
            "observed_valid_cycle_count": subject.v11.EXPECTED_CYCLES,
            "precision": 1.0,
            "recall": 1.0,
            "max_abs_hs_error_s": float(hs_error_s),
            "max_abs_toe_off_error_s": 0.030,
            "confirmed_fsm_stance_f1": 0.99,
            "confirmed_fsm_stance_iou": 0.95,
            "transfer_both_latches_off_sample_count": 0,
            "incomplete_heel_to_forefoot_transfer_count": 0,
            "to_candidates_before_min_stance_count": 0,
            "invalid_or_timeout_transition_count": 0,
            "unaccepted_sensor_gait_event_count": 0,
            "forbidden_phase_mismatch_count": 0,
            "unknown_fsm_phase_samples": 0,
            "minimum_causal_toe_clear_before_next_hs_onset_s": 0.040,
            "causal_swing_interval_count": subject.v11.EXPECTED_CYCLES,
            "confirmation_latency_in_range": True,
            "exact_hs_to_toe_off_to_hs_order_and_cycle_count": True,
            "mesh_geometry_pre_gate_ok": True,
            "event_count_deficit": 0,
            "invalid_timeout_plus_unaccepted_count": 0,
            "early_max_abs_hs_error_s": float(hs_error_s),
            "long_max_abs_hs_error_s": float(hs_error_s),
            "global_mean_abs_hs_error_s": float(hs_error_s) / 2.0,
        }

    def test_protocol_and_sources_are_frozen_and_hash_pinned(self) -> None:
        self.assertEqual(subject.SCHEMA_VERSION, 12)
        self.assertTrue(self.protocol["frozen_before_execution"])
        self.assertEqual(
            self.protocol["_protocol_sha256"], subject.v1._sha256(PROTOCOL_PATH)
        )
        self.assertEqual(
            self.protocol["_protocol_path"], PROTOCOL_PATH.resolve().as_posix()
        )
        self.assertEqual(self.protocol["objective"], subject.OBJECTIVE)
        self.assertEqual(
            self.protocol["interpretation_limits"], subject.INTERPRETATION_LIMITS
        )

        sources = self.protocol["sources"]
        self.assertEqual(set(sources), set(subject.REQUIRED_SOURCE_PATHS))
        for label, record in sources.items():
            self.assertEqual(record["path"], subject.REQUIRED_SOURCE_PATHS[label])
            source_path = subject.v1.resolve_repo_path(record["path"]).resolve()
            self.assertTrue(source_path.is_file(), label)
            self.assertEqual(record["sha256"], subject.v1._sha256(source_path), label)

        replay = self.protocol["replay"]
        self.assertEqual(replay["runtime_sample_dt_s"], 0.010)
        self.assertEqual(replay["fine_sample_dt_s"], 0.001)
        self.assertTrue(replay["evaluate_all_candidates_at_both_resolutions"])
        self.assertEqual(replay["primary_event_time_field"], "confirmed_time_s")
        self.assertEqual(replay["diagnostic_event_time_field"], "event_time_s")
        selection = self.protocol["selection"]
        self.assertEqual(selection["ranking"], subject.v11.STAGE_B_RANKING)
        self.assertEqual(
            selection["selector_implementation"],
            "v11_select_stage_b_with_equal_8_plus_1_cardinality",
        )
        self.assertTrue(selection["minimum_displacement_is_final_tie_break"])

        decision = self.protocol["decision_contract"]
        self.assertFalse(decision["profile_creation_allowed"])
        self.assertFalse(decision["profile_promotion_allowed"])
        self.assertFalse(decision["sealed_validation_allowed"])
        self.assertFalse(decision["training_allowed"])
        self.assertTrue(decision["v9_v10_v11_files_must_remain_immutable"])

    def test_protocol_rejects_grid_timestamp_and_source_hash_drift(self) -> None:
        frozen = self._source_payload()

        grid_drift = copy.deepcopy(frozen)
        grid_drift["toe_center_extension"][
            "selectable_toe_center_local_y_down_from_v9_mm"
        ][0] = 0.56
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

    def test_nine_candidates_change_only_toe_local_y_from_fixed_anchor(self) -> None:
        self.assertEqual(tuple(subject.TOE_DOWN_GRID_MM), EXPECTED_TOE_DOWN_GRID_MM)
        self.assertEqual(subject.SELECTABLE_COUNT, 8)
        self.assertEqual(subject.PAIR_COUNT, 9)
        self.assertEqual(len(self.candidates), 9)
        self.assertEqual(sum(item.selectable for item in self.candidates), 8)
        self.assertEqual(len({item.candidate_id for item in self.candidates}), 9)
        self.assertFalse(self.comparator.selectable)
        self.assertEqual(
            float(self.comparator.geometry["toe_center_down_mm"]), 0.5
        )

        baseline = subject.v1._left_sensor_spheres(self.base)
        heel_0 = baseline["left_heel"]
        toe_0 = baseline["left_toe"]
        expected_heel = (
            heel_0.location[0] + subject.HEEL_X_SHIFT_MM / 1000.0,
            heel_0.location[1],
            heel_0.location[2],
        )
        observed_grid: list[float] = []
        for candidate in self.candidates:
            sensors = subject.v1._left_sensor_spheres(
                self.profiles[candidate.candidate_id]
            )
            heel = sensors["left_heel"]
            toe = sensors["left_toe"]
            down_mm = float(candidate.geometry["toe_center_down_mm"])

            np.testing.assert_allclose(
                heel.location, expected_heel, rtol=0.0, atol=1.0e-15
            )
            self.assertEqual(tuple(toe.location)[::2], tuple(toe_0.location)[::2])
            self.assertAlmostEqual(
                toe.location[1], toe_0.location[1] - down_mm / 1000.0, 14
            )
            self.assertEqual(heel.radius, heel_0.radius)
            self.assertEqual(toe.radius, toe_0.radius)
            self.assertEqual(
                float(candidate.geometry["heel_x_shift_mm"]),
                subject.HEEL_X_SHIFT_MM,
            )
            self.assertEqual(
                candidate.geometry["stage_a_anchor_id"], subject.HEEL_ANCHOR_ID
            )
            if candidate.selectable:
                observed_grid.append(down_mm)

        np.testing.assert_allclose(
            observed_grid,
            EXPECTED_TOE_DOWN_GRID_MM,
            rtol=0.0,
            atol=1.0e-12,
        )
        self.assertTrue(self.geometry["only_toe_local_y_changes"])
        self.assertTrue(self.geometry["all_radii_fixed"])
        self.assertEqual(self.geometry["heel_x_shift_from_v9_mm"], 3.5)
        self.assertAlmostEqual(
            self.geometry["maximum_abs_heel_to_toe_bottom_offset_mm"],
            16.809700203540583,
            places=12,
        )
        self.assertAlmostEqual(
            self.geometry["minimum_bottom_offset_margin_to_20mm"],
            3.190299796459417,
            places=12,
        )
        self.assertGreater(
            self.geometry["minimum_bottom_offset_margin_to_20mm"], 0.0
        )

    def test_sampling_has_ten_detector_plus_eight_primary_stations(self) -> None:
        self.assertEqual(subject.DETECTOR_STATIONS, 10)
        self.assertEqual(subject.v10.EXPECTED_PRIMARY_SPHERES, 8)
        self.assertEqual(subject.TOTAL_STATIONS, 18)
        self.assertEqual(len(self.sampler.spheres), 10)
        self.assertEqual(len({sphere.name for sphere in self.sampler.spheres}), 10)
        self.assertEqual(len(self.profiles), 9)
        self.assertEqual(len(self.pairs), 9)
        self.assertTrue(
            all(len(profile.spheres) == 2 for profile in self.profiles.values())
        )

        sampling = self.protocol["sampling"]
        for cadence in ("runtime_10ms", "fine_1ms"):
            contract = sampling[cadence]
            self.assertEqual(contract["expected_unique_detector_stations"], 10)
            self.assertEqual(contract["expected_primary_load_spheres"], 8)
            self.assertEqual(contract["expected_total_sampled_stations"], 18)
            self.assertEqual(contract["evaluated_pair_count"], 9)

    def test_strict_selection_excludes_comparator_and_ranks_smallest_delta(self) -> None:
        selectable = [item for item in self.candidates if item.selectable]
        strict_candidates = {
            next(
                item.candidate_id
                for item in selectable
                if math.isclose(
                    float(item.geometry["toe_center_down_mm"]), 0.60
                )
            ),
            next(
                item.candidate_id
                for item in selectable
                if math.isclose(
                    float(item.geometry["toe_center_down_mm"]), 0.80
                )
            ),
        }
        smaller_delta_id = next(
            item.candidate_id
            for item in selectable
            if math.isclose(float(item.geometry["toe_center_down_mm"]), 0.60)
        )
        better_timing_id = next(
            item.candidate_id
            for item in selectable
            if math.isclose(float(item.geometry["toe_center_down_mm"]), 0.80)
        )
        timing_ranked_rows = [
            self._row(
                item,
                hs_error_s=(
                    0.010
                    if item.candidate_id == subject.COMPARATOR_ID
                    else 0.030
                    if item.candidate_id == smaller_delta_id
                    else 0.020
                    if item.candidate_id == better_timing_id
                    else 0.060
                ),
            )
            for item in self.candidates
        ]
        timing_ranked = subject.v11.select_stage_b(
            copy.deepcopy(timing_ranked_rows),
            copy.deepcopy(timing_ranked_rows),
            self.protocol,
        )
        self.assertEqual(timing_ranked["strict_winner_id"], better_timing_id)

        tied_rows = [
            self._row(
                item,
                hs_error_s=(
                    0.010
                    if item.candidate_id == subject.COMPARATOR_ID
                    else 0.020
                    if item.candidate_id in strict_candidates
                    else 0.060
                ),
            )
            for item in self.candidates
        ]
        tied = subject.v11.select_stage_b(
            copy.deepcopy(tied_rows), copy.deepcopy(tied_rows), self.protocol
        )
        self.assertEqual(tied["strict_winner_id"], smaller_delta_id)
        self.assertEqual(
            set(tied["eligible_ids_ranked"]), strict_candidates
        )
        self.assertNotIn(subject.COMPARATOR_ID, tied["eligible_ids_ranked"])
        self.assertNotIn(subject.COMPARATOR_ID, tied["candidate_gates"])
        self.assertIn(
            subject.COMPARATOR_ID, tied["all_candidates_diagnostic_ranked"]
        )
        self.assertFalse(tied["automatic_promotion_allowed"])

        comparator_only = [
            self._row(
                item,
                hs_error_s=(
                    0.010 if item.candidate_id == subject.COMPARATOR_ID else 0.060
                ),
            )
            for item in self.candidates
        ]
        no_winner = subject.v11.select_stage_b(
            copy.deepcopy(comparator_only),
            copy.deepcopy(comparator_only),
            self.protocol,
        )
        self.assertIsNone(no_winner["strict_winner_id"])
        self.assertEqual(no_winner["eligible_ids_ranked"], [])

    def test_v11_boundary_reference_inputs_replay_identically(self) -> None:
        reference = subject._load_v11_reference(self.protocol)
        self.assertEqual(reference["schema_version"], 11)
        self.assertEqual(reference["status"], "FAIL")
        self.assertTrue(reference["stage_b"]["executed"])
        self.assertFalse(reference["data_access"]["sealed_block_opened"])

        manifest_geometry = subject._v11_manifest_geometry_identity(
            reference, self.comparator
        )
        self.assertTrue(manifest_geometry["ok"])
        self.assertTrue(all(manifest_geometry["comparisons"].values()))

        tampered_reference = copy.deepcopy(reference)
        tampered_boundary = next(
            item
            for item in tampered_reference["stage_b"]["candidates"]
            if item["candidate_id"] == subject.V11_BOUNDARY_ID
        )
        tampered_boundary["forefoot_radius_m"] += 0.001
        tampered_geometry = subject._v11_manifest_geometry_identity(
            tampered_reference, self.comparator
        )
        self.assertFalse(tampered_geometry["ok"])
        self.assertFalse(
            tampered_geometry["comparisons"]["forefoot_radius_m"]
        )

        self.assertEqual(
            tuple(self.v11_boundary.heel_location),
            tuple(self.comparator.heel_location),
        )
        self.assertEqual(
            tuple(self.v11_boundary.forefoot_location),
            tuple(self.comparator.forefoot_location),
        )
        self.assertEqual(
            float(self.v11_boundary.geometry["heel_radius_m"]),
            float(self.comparator.geometry["heel_radius_m"]),
        )
        self.assertEqual(
            float(self.v11_boundary.geometry["toe_radius_m"]),
            float(self.comparator.geometry["toe_radius_m"]),
        )

        rows_b: dict[str, list[dict[str, Any]]] = {}
        details_b: dict[str, dict[str, Any]] = {}
        for cadence in ("runtime_10ms", "fine_1ms"):
            rows_a = reference["stage_b"][cadence]["rows"]
            boundary_row = next(
                row
                for row in rows_a
                if row["candidate_id"] == subject.V11_BOUNDARY_ID
            )
            comparator_row = copy.deepcopy(boundary_row)
            comparator_row["candidate_id"] = subject.COMPARATOR_ID
            rows_b[cadence] = [comparator_row]
            boundary_detail = reference["stage_b"][cadence]["details"][
                subject.V11_BOUNDARY_ID
            ]
            details_b[cadence] = {
                subject.COMPARATOR_ID: copy.deepcopy(boundary_detail)
            }

            csv_key = (
                "v11_stage_b_runtime_csv"
                if cadence == "runtime_10ms"
                else "v11_stage_b_fine_csv"
            )
            csv_path = subject.v1.resolve_repo_path(
                self.protocol["sources"][csv_key]["path"]
            )
            with csv_path.open(newline="", encoding="utf-8") as stream:
                csv_ids = {row["candidate_id"] for row in csv.DictReader(stream)}
            self.assertIn(subject.V11_BOUNDARY_ID, csv_ids)

        identity = subject.v11._stage_b_anchor_identity_audit(
            self.v11_boundary,
            self.comparator,
            self.protocol,
            {
                cadence: reference["stage_b"][cadence]["rows"]
                for cadence in ("runtime_10ms", "fine_1ms")
            },
            {
                cadence: reference["stage_b"][cadence]["details"]
                for cadence in ("runtime_10ms", "fine_1ms")
            },
            rows_b,
            details_b,
        )
        self.assertTrue(identity["geometry_equal"])
        self.assertTrue(identity["ok"])
        for audit in identity["resolutions"].values():
            self.assertTrue(audit["strict_gate_equal"])
            self.assertTrue(audit["heel_to_forefoot_transfer_equal"])

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
                        subject.v11._preflight_no_clobber(output, plot_dir)
                    self.assertTrue(occupied.is_dir())
                    self.assertEqual(list(occupied.iterdir()), [])
                    self.assertFalse(free.exists())

    def test_time_grids_never_touch_the_sealed_100s_boundary(self) -> None:
        self.assertEqual(self.protocol["data_access"], subject._expected_access())
        self.assertFalse(
            self.protocol["data_access"]["allow_samples_at_or_after_100_s"]
        )
        self.assertEqual(
            self.protocol["data_access"]["sealed_block_status"],
            "CLOSED_UNEVALUATED",
        )

        events = {
            "heel_strike": np.linspace(
                14.0, 99.0, subject.v11.EXPECTED_REFERENCE_HS
            ),
            "toe_off": np.linspace(
                14.5, 98.5, subject.v11.EXPECTED_REFERENCE_TO
            ),
        }
        boundary = {"excluded_closing_hs_s": [99.96878691565038]}
        with (
            patch.object(
                subject.v10.v1, "read_setup_xml", return_value=_SetupStub()
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
                self.assertEqual(audit["samples_at_or_after_100_s"], 0)
                self.assertFalse(audit["sealed_block_opened"])

        bad_events = dict(events)
        bad_events["heel_strike"] = np.linspace(
            14.0, 99.980, subject.v11.EXPECTED_REFERENCE_HS
        )
        with (
            patch.object(
                subject.v10.v1, "read_setup_xml", return_value=_SetupStub()
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
