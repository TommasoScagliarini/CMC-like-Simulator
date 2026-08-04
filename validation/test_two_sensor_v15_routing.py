from __future__ import annotations

import math
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
VALIDATION_ROOT = REPO_ROOT / "validation"
if str(VALIDATION_ROOT) not in sys.path:
    sys.path.insert(0, str(VALIDATION_ROOT))

import diagnose_two_sensor_v15_routing as subject  # noqa: E402


class V15ProtocolTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.protocol = subject.load_and_validate_protocol()

    def test_protocol_is_development_only_and_non_promoting(self) -> None:
        self.assertEqual(
            self.protocol["split"]["DEVELOPMENT"], ["02", "04", "08"]
        )
        self.assertEqual(
            self.protocol["split"]["FORBIDDEN"],
            ["01", "03", "05", "06", "07"],
        )
        self.assertFalse(
            self.protocol["decision_contract"][
                "candidate_selection_or_promotion_allowed"
            ]
        )
        self.assertFalse(
            self.protocol["geometry"]["alternative_geometry_sampling_allowed"]
        )

    def test_parent_v14_2_failed_without_opening_holdouts(self) -> None:
        audit = subject.validate_parent_provenance(self.protocol)
        self.assertTrue(all(audit["checks"].values()))
        self.assertEqual(
            audit["status"], "PASS_V14_2_DEVELOPMENT_ONLY_PARENT_VERIFIED"
        )

    def test_only_frozen_v14_2_preprocessing_is_routed(self) -> None:
        for trial_id in subject.ALLOWED_TRIALS:
            artifacts = subject._validate_preprocessing_trial(
                self.protocol, trial_id
            )
            self.assertEqual(artifacts.trial_id, trial_id)
            self.assertIn(
                f"preprocessed/trial_{trial_id}",
                artifacts.setup.kinematics_file.as_posix(),
            )
        with self.assertRaises(subject.ProtocolError):
            subject._validate_preprocessing_trial(self.protocol, "05")

    def test_frozen_references_are_identical_across_cadences(self) -> None:
        for trial_id in subject.ALLOWED_TRIALS:
            references = subject.load_frozen_references(self.protocol, trial_id)
            self.assertEqual(len(references), 4)
            for reference in references:
                hs = reference["events"]["heel_strike"]
                toe_off = reference["events"]["toe_off"]
                self.assertEqual(len(hs), len(toe_off) + 1)
                self.assertGreaterEqual(len(toe_off), 10)

    def test_profile_is_exactly_frozen_v13_pair(self) -> None:
        geometry = self.protocol["geometry"]
        self.assertEqual(geometry["candidate_id"], subject.BASELINE_ID)
        self.assertEqual(
            {item["name"] for item in geometry["spheres"]},
            {"left_heel", "left_toe"},
        )
        self.assertTrue(
            all(math.isclose(item["radius"], 0.0229053623) for item in geometry["spheres"])
        )

    def test_parent_incumbent_full_hs_sequences_are_cross_checked(self) -> None:
        sequences = subject.load_v14_2_incumbent_sequences(self.protocol)
        self.assertEqual(sequences["unit_count"], subject.EXPECTED_UNIT_COUNT)
        self.assertEqual(
            sequences["event_count"], subject.EXPECTED_INCUMBENT_HS_EVENTS
        )
        self.assertEqual(len(sequences["aggregate_sha256"]), 64)


class V15SignalLogicTests(unittest.TestCase):
    def test_raw_rising_crossings_report_only_off_to_on_edges(self) -> None:
        times = np.arange(0.0, 0.08, 0.01)
        force = np.asarray([0.0, 0.6, 0.7, 0.1, 0.2, 0.8, 0.9, 0.0])
        self.assertEqual(
            subject.raw_rising_crossings(times, force, 0.5), [0.01, 0.05]
        )

    def test_debounce_preserves_onset_and_delays_confirmation(self) -> None:
        times = np.arange(0.0, 0.11, 0.01)
        force = np.asarray([0.0, 0.6, 0.7, 0.8, 0.9, 0.9, 0.1, 0.1, 0.1, 0.1, 0.1])
        edges = subject.debounce_sensor_edges(
            times,
            force,
            sensor="heel",
            on_threshold_n=0.5,
            off_threshold_n=0.25,
            dwell_s=0.03,
        )
        self.assertEqual([item.edge for item in edges], ["contact_on", "contact_off"])
        self.assertAlmostEqual(edges[0].onset_s, 0.01)
        self.assertAlmostEqual(edges[0].confirmed_s, 0.04)
        self.assertAlmostEqual(edges[1].onset_s, 0.06)
        self.assertAlmostEqual(edges[1].confirmed_s, 0.09)

    def test_debounce_rejects_short_threshold_glitch(self) -> None:
        times = np.arange(0.0, 0.08, 0.01)
        force = np.asarray([0.0, 0.6, 0.7, 0.1, 0.6, 0.7, 0.8, 0.9])
        edges = subject.debounce_sensor_edges(
            times,
            force,
            sensor="heel",
            on_threshold_n=0.5,
            off_threshold_n=0.25,
            dwell_s=0.03,
        )
        self.assertEqual(len(edges), 1)
        self.assertAlmostEqual(edges[0].onset_s, 0.04)
        self.assertAlmostEqual(edges[0].confirmed_s, 0.07)

    def test_first_regional_routes_forefoot_but_incumbent_waits_for_heel(self) -> None:
        forefoot = [
            subject.SensorEdge("forefoot", "contact_on", 1.00, 1.03),
            subject.SensorEdge("forefoot", "contact_off", 1.40, 1.43),
        ]
        heel = [
            subject.SensorEdge("heel", "contact_on", 1.06, 1.09),
            subject.SensorEdge("heel", "contact_off", 1.30, 1.33),
        ]
        incumbent = subject.route_hs_events(
            heel, forefoot, routing_mode="heel_only"
        )
        regional = subject.route_hs_events(
            heel, forefoot, routing_mode="first_stable_regional"
        )
        self.assertEqual(incumbent[0].source_sensor, "heel")
        self.assertAlmostEqual(incumbent[0].confirmed_s, 1.09)
        self.assertEqual(regional[0].source_sensor, "forefoot")
        self.assertAlmostEqual(regional[0].confirmed_s, 1.03)

    def test_router_rearms_only_after_both_regions_are_off(self) -> None:
        heel = [
            subject.SensorEdge("heel", "contact_on", 1.0, 1.03),
            subject.SensorEdge("heel", "contact_off", 1.3, 1.33),
            subject.SensorEdge("heel", "contact_on", 1.5, 1.53),
            subject.SensorEdge("heel", "contact_off", 1.8, 1.83),
        ]
        forefoot = [
            subject.SensorEdge("forefoot", "contact_on", 1.2, 1.23),
            subject.SensorEdge("forefoot", "contact_off", 1.6, 1.63),
        ]
        routed = subject.route_hs_events(
            heel, forefoot, routing_mode="heel_only"
        )
        self.assertEqual(len(routed), 1)

    def test_samplewise_router_rearms_on_last_stable_off(self) -> None:
        times = np.arange(0.0, 0.201, 0.01)
        heel_force = np.zeros_like(times)
        forefoot_force = np.zeros_like(times)
        heel_force[(times >= 0.04) & (times < 0.09)] = 1.0
        forefoot_force[(times >= 0.10) & (times < 0.16)] = 1.0
        heel_edges = [
            subject.SensorEdge("heel", "contact_on", 0.04, 0.07),
            subject.SensorEdge("heel", "contact_off", 0.09, 0.12),
        ]
        forefoot_edges = [
            subject.SensorEdge("forefoot", "contact_on", 0.10, 0.13),
            subject.SensorEdge("forefoot", "contact_off", 0.16, 0.19),
        ]
        routed = subject.route_hs_trace(
            times,
            heel_force,
            forefoot_force,
            heel_edges,
            forefoot_edges,
            routing_mode="first_stable_regional",
            on_threshold_n=0.5,
            off_threshold_n=0.25,
            dwell_s=0.03,
        )
        self.assertEqual(
            [round(item.confirmed_s, 2) for item in routed], [0.07, 0.13]
        )

    def test_samplewise_router_rearms_with_other_raw_in_hysteresis(self) -> None:
        times = np.arange(0.0, 0.221, 0.01)
        heel_force = np.zeros_like(times)
        forefoot_force = np.zeros_like(times)
        heel_force[(times >= 0.04) & (times < 0.09)] = 1.0
        heel_force[(times >= 0.14) & (times < 0.20)] = 1.0
        forefoot_force[times >= 0.08] = 0.30
        heel_edges = subject.debounce_sensor_edges(
            times,
            heel_force,
            sensor="heel",
            on_threshold_n=0.5,
            off_threshold_n=0.25,
            dwell_s=0.03,
        )
        routed = subject.route_hs_trace(
            times,
            heel_force,
            forefoot_force,
            heel_edges,
            [],
            routing_mode="heel_only",
            on_threshold_n=0.5,
            off_threshold_n=0.25,
            dwell_s=0.03,
        )
        self.assertEqual(
            [round(item.confirmed_s, 2) for item in routed], [0.07, 0.17]
        )

    def test_samplewise_router_accepts_production_heel_only_startup(self) -> None:
        times = np.arange(0.0, 0.101, 0.01)
        heel_force = np.ones_like(times)
        forefoot_force = np.zeros_like(times)
        heel_edges = subject.debounce_sensor_edges(
            times,
            heel_force,
            sensor="heel",
            on_threshold_n=0.5,
            off_threshold_n=0.25,
            dwell_s=0.03,
        )
        routed = subject.route_hs_trace(
            times,
            heel_force,
            forefoot_force,
            heel_edges,
            [],
            routing_mode="heel_only",
            on_threshold_n=0.5,
            off_threshold_n=0.25,
            dwell_s=0.03,
        )
        self.assertEqual(len(routed), 1)
        self.assertAlmostEqual(routed[0].onset_s, 0.0)
        self.assertAlmostEqual(routed[0].confirmed_s, 0.03)

    def test_ordered_association_avoids_greedy_crossing_loss(self) -> None:
        mapping = subject._nearest_unique(
            [0.0, 0.2], [0.11, 0.31], maximum_distance_s=0.15
        )
        self.assertEqual(mapping, {0: 0, 1: 1})

    def test_boundary_context_preserves_early_out_of_tolerance_event(self) -> None:
        reference = {
            "plateau_index": 1,
            "speed_mps": 1.0,
            "plateau_interval_s": [0.0, 3.0],
            "events": {
                "heel_strike": np.asarray([1.0, 2.0]),
                "toe_off": np.asarray([1.5]),
            },
        }
        routed = [
            subject.RoutedHS("heel_only", "heel", 0.91, 0.94),
            subject.RoutedHS("heel_only", "heel", 1.97, 2.0),
        ]
        row = subject.feasibility_row(
            reference,
            routed,
            trial_id="02",
            cadence="fine_1ms",
            sample_dt_s=0.001,
            trial_start_s=0.0,
            threshold_n=0.5,
            dwell_s=0.03,
            routing_mode="heel_only",
            tolerance_s=0.05,
        )
        self.assertAlmostEqual(row["diagnostic_confirmed_error_min_s"], -0.06)
        self.assertEqual(row["diagnostic_failed_reference_count"], 1)

    def test_feasibility_row_requires_exact_count_and_50ms_match(self) -> None:
        reference = {
            "plateau_index": 1,
            "speed_mps": 1.0,
            "plateau_interval_s": [0.0, 3.0],
            "events": {
                "heel_strike": np.asarray([1.0, 2.0]),
                "toe_off": np.asarray([1.5]),
            },
        }
        routed = [
            subject.RoutedHS("heel_only", "heel", 0.98, 1.01),
            subject.RoutedHS("heel_only", "heel", 1.98, 2.01),
        ]
        row = subject.feasibility_row(
            reference,
            routed,
            trial_id="02",
            cadence="fine_1ms",
            sample_dt_s=0.001,
            trial_start_s=0.0,
            threshold_n=0.5,
            dwell_s=0.03,
            routing_mode="heel_only",
            tolerance_s=0.05,
        )
        self.assertTrue(row["exact_count_and_50ms_timing_feasible"])
        late = [*routed[:-1], subject.RoutedHS("heel_only", "heel", 2.05, 2.08)]
        row = subject.feasibility_row(
            reference,
            late,
            trial_id="02",
            cadence="fine_1ms",
            sample_dt_s=0.001,
            trial_start_s=0.0,
            threshold_n=0.5,
            dwell_s=0.03,
            routing_mode="heel_only",
            tolerance_s=0.05,
        )
        self.assertFalse(row["exact_count_and_50ms_timing_feasible"])

    def test_incumbent_parity_rejects_internal_sequence_mutation(self) -> None:
        protocol = subject.load_and_validate_protocol()
        parent = subject.load_v14_2_incumbent_sequences(protocol)
        rows = []
        for key in sorted(parent["by_unit"]):
            payload = parent["by_unit"][key]["payload"]
            sample_dt_s = dict(subject.CADENCES)[key[2]]
            trial_start_s = float(
                protocol["trials"][key[0]]["trial_interval_s"][0]
            )
            onset_s = [
                trial_start_s + int(index) * sample_dt_s
                for index in payload["onset_hs_sample_indices"]
            ]
            confirmed_s = [
                trial_start_s + int(index) * sample_dt_s
                for index in payload["confirmed_hs_sample_indices"]
            ]
            rows.append(
                {
                    "trial_id": key[0],
                    "plateau_index": key[1],
                    "cadence": key[2],
                    "routing_mode": "heel_only",
                    "sensor_on_threshold_n": 0.5,
                    "sensor_dwell_s": 0.03,
                    "reference_event_sha256": payload[
                        "reference_event_sha256"
                    ],
                    "ordered_routed_onset_s_json": onset_s,
                    "ordered_routed_confirmed_s_json": confirmed_s,
                    "ordered_routed_source_sensor_json": payload[
                        "source_sensors"
                    ],
                    "routed_hs_sequence_sha256": parent["by_unit"][key][
                        "sha256"
                    ],
                }
            )
        self.assertTrue(subject.incumbent_v14_2_parity(protocol, rows)["ok"])

        mutated = [dict(row) for row in rows]
        target = mutated[len(mutated) // 2]
        onset_s = list(target["ordered_routed_onset_s_json"])
        onset_s[len(onset_s) // 2] += dict(subject.CADENCES)[target["cadence"]]
        target["ordered_routed_onset_s_json"] = onset_s
        key = (
            target["trial_id"],
            int(target["plateau_index"]),
            target["cadence"],
        )
        mutated_payload = subject._sequence_payload(
            key=key,
            reference_event_sha256=target["reference_event_sha256"],
            onset_indices=subject._strict_grid_indices(
                onset_s,
                trial_start_s=float(
                    protocol["trials"][key[0]]["trial_interval_s"][0]
                ),
                sample_dt_s=dict(subject.CADENCES)[key[2]],
                label="mutated test sequence",
            ),
            confirmed_indices=parent["by_unit"][key]["payload"][
                "confirmed_hs_sample_indices"
            ],
            source_sensors=target["ordered_routed_source_sensor_json"],
        )
        target["routed_hs_sequence_sha256"] = subject._canonical_sha256(
            mutated_payload
        )
        self.assertFalse(
            subject.incumbent_v14_2_parity(protocol, mutated)["ok"]
        )


class V15HyEstimateTests(unittest.TestCase):
    def test_minimum_hy_uses_complete_causal_dwell_window(self) -> None:
        times = np.arange(0.0, 0.101, 0.01)
        required = np.asarray([2.0, 2.0, 2.0, 1.0, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2]) / 1000.0
        result = subject.minimum_hy_offset_for_reference(
            times,
            required,
            reference_hs_s=0.05,
            dwell_s=0.03,
            maximum_confirmed_error_s=0.05,
            search_lookback_s=0.05,
        )
        self.assertTrue(result["feasible_window_found"])
        self.assertAlmostEqual(
            result["equivalent_plantar_ground_normal_offset_mm"], 0.5
        )
        self.assertLessEqual(result["confirmed_s"] if "confirmed_s" in result else result["window_confirmed_s"], 0.10)

    def test_hy_checkpoint_columns_are_lower_bound_only(self) -> None:
        columns = subject._hy_checkpoint_columns(0.75, [0.0, 0.5, 0.75, 1.0])
        self.assertFalse(columns["window_bound_at_or_below_hy_0p0mm"])
        self.assertFalse(columns["window_bound_at_or_below_hy_0p5mm"])
        self.assertTrue(columns["window_bound_at_or_below_hy_0p75mm"])
        self.assertTrue(columns["window_bound_at_or_below_hy_1p0mm"])

    def test_hy_bound_cannot_use_previous_contact_window(self) -> None:
        times = np.arange(0.0, 0.501, 0.01)
        required = np.full_like(times, 2.0e-3)
        required[(times >= 0.05) & (times <= 0.10)] = 0.0
        required[(times >= 0.28) & (times <= 0.36)] = 0.8e-3
        result = subject.minimum_hy_offset_for_reference(
            times,
            required,
            reference_hs_s=0.30,
            dwell_s=0.03,
            maximum_confirmed_error_s=0.05,
            search_lookback_s=0.25,
        )
        self.assertAlmostEqual(
            result["equivalent_plantar_ground_normal_offset_mm"], 0.8
        )
        self.assertGreaterEqual(result["window_confirmed_s"], 0.25)

    def test_hunt_crossley_inverse_returns_finite_nonnegative_offsets(self) -> None:
        material = SimpleNamespace(
            smoothing=1e-5,
            dissipation=0.2,
            stiffness=46000.0,
            exponent=1.5,
        )
        profile = SimpleNamespace(
            ground=SimpleNamespace(
                normal=(0.0, 1.0, 0.0),
                origin=(0.0, 0.0, 0.0),
                surface_velocity=(0.0, 0.0, 0.0),
            ),
            material=material,
        )
        sphere = SimpleNamespace(name="heel", radius=0.02, material=None)
        samples = {
            "centers": {"heel": np.asarray([[0.0, 0.021, 0.0], [0.0, 0.019, 0.0]])},
            "velocities": {"heel": np.zeros((2, 3))},
        }
        offsets = subject.required_ground_normal_offsets_m(
            profile, sphere, samples, threshold_n=0.5
        )
        self.assertEqual(offsets.shape, (2,))
        self.assertTrue(np.all(np.isfinite(offsets)))
        self.assertTrue(np.all(offsets >= 0.0))
        self.assertGreater(offsets[0], offsets[1])


class V15NoClobberTests(unittest.TestCase):
    def test_noncanonical_destination_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            with self.assertRaises(subject.NoClobberError):
                subject._preflight_no_clobber(Path(temp_dir) / "v15")

    def test_existing_canonical_destination_is_rejected(self) -> None:
        with mock.patch.object(
            subject, "DEFAULT_OUTPUT_DIR", Path(tempfile.gettempdir()) / "v15_existing"
        ):
            subject.DEFAULT_OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
            try:
                with self.assertRaises(subject.NoClobberError):
                    subject._preflight_no_clobber(subject.DEFAULT_OUTPUT_DIR)
            finally:
                subject.DEFAULT_OUTPUT_DIR.rmdir()


if __name__ == "__main__":
    unittest.main()
