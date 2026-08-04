"""Pure pre-access tests for the AB06 cross-speed V14 harness.

These tests do not decode or replay trials 02--08 and never execute OpenSim.
They freeze the search geometry, continuous-trial plateau slicing, coherent
four-plateau aggregation, and fail-closed access primitives.
"""

from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path
from types import ModuleType, SimpleNamespace
from typing import Any
from unittest.mock import patch

import numpy as np


VALIDATION_ROOT = Path(__file__).resolve().parent
if str(VALIDATION_ROOT) not in sys.path:
    sys.path.insert(0, str(VALIDATION_ROOT))


class _ForbiddenOpenSim(ModuleType):
    def __getattr__(self, name: str) -> Any:
        raise AssertionError(f"pure V14 test touched opensim.{name}")


with patch.dict(sys.modules, {"opensim": _ForbiddenOpenSim("opensim")}):
    import sweep_two_sensor_cross_speed_v14 as subject  # noqa: E402


def _passing_row(plateau_index: int) -> dict[str, Any]:
    cycles = 10
    return {
        "candidate_id": "candidate",
        "trial_id": "02",
        "cadence": "runtime_10ms",
        "plateau_index": plateau_index,
        "reference_hs_count": cycles + 1,
        "reference_to_count": cycles,
        "predicted_hs_count": cycles + 1,
        "predicted_to_count": cycles,
        "observed_valid_cycle_count": cycles,
        "causal_swing_interval_count": cycles,
        "transfer_both_latches_off_sample_count": 0,
        "incomplete_heel_to_forefoot_transfer_count": 0,
        "to_candidates_before_min_stance_count": 0,
        "invalid_or_timeout_transition_count": 0,
        "unaccepted_sensor_gait_event_count": 0,
        "forbidden_phase_mismatch_count": 0,
        "unknown_fsm_phase_samples": 0,
        "event_matched_hs_count": cycles + 1,
        "event_matched_to_count": cycles,
        "fsm_true_positive_samples": 100,
        "fsm_false_positive_samples": 0,
        "fsm_false_negative_samples": 0,
        "fsm_true_negative_samples": 100,
        "max_abs_hs_error_s": 0.020,
        "max_abs_toe_off_error_s": 0.030,
        "minimum_causal_toe_clear_before_next_hs_onset_s": 0.040,
        "confirmation_latency_in_range": True,
        "exact_hs_to_toe_off_to_hs_order_and_cycle_count": True,
        "mesh_geometry_pre_gate_ok": True,
    }


class FrozenGridAndTemplateTests(unittest.TestCase):
    def test_exact_one_sided_grids_and_stage2_bound(self) -> None:
        self.assertEqual(
            subject.HEEL_X_ABSOLUTE_FROM_V9_MM,
            (3.50, 3.75, 4.00, 4.25, 4.50, 5.00, 5.50, 6.00),
        )
        self.assertEqual(
            subject.HEEL_RADIUS_REDUCTION_FROM_V13_MM,
            (0.00, 0.05, 0.10, 0.15, 0.20, 0.25),
        )
        self.assertEqual(
            subject.TOE_DOWN_ABSOLUTE_FROM_V9_MM,
            (0.75, 0.85, 0.95, 1.05, 1.15, 1.25, 1.35),
        )
        self.assertEqual(
            subject.TOE_RADIUS_REDUCTION_FROM_V13_MM,
            (0.00, 0.05, 0.10, 0.15, 0.20, 0.25),
        )
        self.assertEqual(subject.ISOLATED_SELECTABLE_COUNT, 23)
        self.assertEqual(subject.MAXIMUM_STAGE2_CARTESIAN_TUPLES, 81)

    def test_template_declares_continuous_fsm_and_exact_reference_source(self) -> None:
        payload = subject.expected_protocol_payload(
            require_metadata_audit=False,
            require_all_sources=False,
        )
        boundary = payload["unit_boundary_handling"]
        self.assertEqual(
            boundary["fsm_replay_scope"],
            "continuous_over_the_entire_trial_without_plateau_reset",
        )
        self.assertEqual(boundary["right_observation_margin_s"], 0.060)
        self.assertTrue(boundary["common_cycle_set_at_10ms_and_1ms"])
        replay = payload["replay"]
        self.assertEqual(replay["prescribed_contact_threshold_n"], 20.0)
        self.assertEqual(replay["reference_min_contact_duration_s"], 0.05)
        self.assertEqual(replay["reference_min_cycle_duration_s"], 0.3)
        self.assertFalse(
            payload["gate_contract"]["reference_source"][
                "gcLeft_or_gcRight_allowed"
            ]
        )
        self.assertTrue(
            all(
                "gcLeft" not in trial["raw_sources"]
                and "gcRight" not in trial["raw_sources"]
                for trial in payload["trials"].values()
            )
        )
        self.assertIn("ik", payload["trials"]["02"]["raw_sources"])
        self.assertEqual(
            payload["preprocessing_golden"]["status"],
            "PASS_VERIFIED_FULL_SPAN_BYTE_PARITY",
        )
        self.assertTrue(all(payload["preprocessing_golden"]["checks"].values()))
        self.assertEqual(
            payload["preprocessing"]["execution_platform_scope"],
            "macos_arm64_only_for_frozen_v14",
        )


class ContinuousCycleSetTests(unittest.TestCase):
    def test_prefix_censor_and_common_mask_are_cadence_independent(self) -> None:
        heel = np.arange(10.0, 22.0, 1.0)
        toe = heel[:-1] + 0.6
        # Closing HS at 21.0 lacks a 60 ms margin inside an end at 21.05,
        # so the last raw cycle is prefix-censored.
        events, access = subject.retain_scoreable_plateau_cycles(
            {"heel_strike": heel, "toe_off": toe},
            plateau_start_s=9.5,
            plateau_end_s=21.05,
            minimum_cycles=10,
        )
        self.assertEqual(access["raw_complete_cycle_count"], 11)
        self.assertEqual(access["retained_scoreable_cycle_count"], 10)
        self.assertGreaterEqual(access["left_context_guard_observed_s"], 0.09)
        self.assertEqual(events["heel_strike"].size, 11)
        self.assertEqual(events["toe_off"].size, 10)

        counts = []
        for dt in (0.010, 0.001):
            times = np.arange(9.5, 21.051, dt)
            mask = subject.continuous_local_score_mask(
                times,
                events,
                plateau_start_s=9.5,
                plateau_end_s=21.05,
            )
            counts.append(int(np.count_nonzero(mask)))
            self.assertLessEqual(float(times[mask][-1]), 20.060 + 1e-12)
        self.assertGreater(counts[1], counts[0])
        self.assertEqual(events["heel_strike"].tolist(), list(np.arange(10.0, 21.0)))

    def test_nonprefix_or_insufficient_cycles_fail_closed(self) -> None:
        with self.assertRaises(subject.ProtocolError):
            subject.retain_scoreable_plateau_cycles(
                {
                    "heel_strike": np.arange(0.0, 6.0),
                    "toe_off": np.arange(0.5, 5.5),
                },
                plateau_start_s=0.0,
                plateau_end_s=6.0,
            )

        with self.assertRaises(subject.ProtocolError):
            subject.retain_scoreable_plateau_cycles(
                {
                    "heel_strike": np.arange(0.05, 12.05, 1.0),
                    "toe_off": np.arange(0.55, 11.55, 1.0),
                },
                plateau_start_s=0.0,
                plateau_end_s=12.5,
                minimum_cycles=10,
            )

    def test_real_continuous_slice_excludes_preboundary_and_unowned_events(self) -> None:
        times = np.arange(-0.10, 2.001, 0.01)
        accepted = [
            # Same valid primary confirmation as the following HS, but its
            # onset predates the plateau and must never be owned locally.
            {"event": "heel_strike", "event_time_s": -0.01, "confirmed_time_s": 0.05},
            # Onset is 70 ms early, confirmed is only 40 ms early: ownership
            # follows the primary confirmed timestamp and must retain it.
            {"event": "heel_strike", "event_time_s": 0.02, "confirmed_time_s": 0.05},
            {"event": "toe_off", "event_time_s": 0.30, "confirmed_time_s": 0.33},
            # TO onset is 100 ms early, confirmed is 70 ms early and remains
            # inside the frozen primary TO tolerance.
            {"event": "toe_off", "event_time_s": 0.70, "confirmed_time_s": 0.73},
            {"event": "heel_strike", "event_time_s": 1.50, "confirmed_time_s": 1.53},
            {"event": "heel_strike", "event_time_s": 1.54, "confirmed_time_s": 1.61},
        ]
        candidates = [
            {"event": "heel_strike", "time": -0.01, "observed_at_s": 0.05},
            {"event": "heel_strike", "time": 0.02, "observed_at_s": 0.05},
            {"event": "toe_off", "time": 0.30, "observed_at_s": 0.33},
            {"event": "toe_off", "time": 0.69, "observed_at_s": 0.72},
            {"event": "toe_off", "time": 0.70, "observed_at_s": 0.73},
        ]
        replay = {
            "heel_contact": np.zeros(times.size),
            "toe_contact": np.zeros(times.size),
            "state_id": np.zeros(times.size),
            "accepted": accepted,
            "candidates": candidates,
            "sensor_edges": [],
            "invalid_steps": [
                {"observed_at_s": -0.01, "type": "preboundary"},
                {"observed_at_s": 0.40, "type": "owned"},
            ],
        }
        reference = {
            "plateau_interval_s": [0.0, 2.0],
            "events": {
                "heel_strike": np.asarray([0.09, 1.50]),
                "toe_off": np.asarray([0.80]),
            },
        }
        mask, local = subject._slice_continuous_replay(
            replay,
            times,
            reference,
            sample_dt_s=0.01,
            sensor_dwell_s=0.03,
        )
        self.assertGreaterEqual(
            float(times[mask][0]),
            -subject.NUMERIC_TOLERANCE,
        )
        self.assertLessEqual(float(times[mask][-1]), 1.56 + 1e-12)
        self.assertEqual(
            [(item["event"], item["event_time_s"]) for item in local["accepted"]],
            [("heel_strike", 0.02), ("toe_off", 0.70), ("heel_strike", 1.50)],
        )
        self.assertEqual(
            [(item["event"], item["time"]) for item in local["candidates"]],
            [
                ("heel_strike", 0.02),
                ("toe_off", 0.69),
                ("toe_off", 0.70),
            ],
        )
        self.assertEqual([item["type"] for item in local["invalid_steps"]], ["owned"])


class TrialAggregateTests(unittest.TestCase):
    def test_four_plateau_union_sums_counts_and_recomputes_gate(self) -> None:
        rows = [_passing_row(index) for index in range(1, 5)]
        aggregates = subject.attach_trial_aggregate_gates(rows)
        self.assertEqual(len(aggregates), 1)
        aggregate = aggregates[0]
        self.assertEqual(aggregate["reference_hs_count"], 44)
        self.assertEqual(aggregate["reference_to_count"], 40)
        self.assertEqual(aggregate["observed_valid_cycle_count"], 40)
        self.assertTrue(aggregate["reference_cycle_sum_consistent"])
        self.assertTrue(aggregate["trial_aggregate_gate_ok"])
        self.assertTrue(all(row["trial_aggregate_gate_ok"] for row in rows))

    def test_aggregate_is_additional_gate_and_detects_cross_plateau_count_loss(self) -> None:
        rows = [_passing_row(index) for index in range(1, 5)]
        rows[2]["predicted_to_count"] -= 1
        rows[2]["event_matched_to_count"] -= 1
        aggregate = subject.aggregate_trial_counts(rows)[0]
        self.assertFalse(aggregate["predicted_cycle_sum_consistent"])
        self.assertFalse(aggregate["trial_aggregate_gate_ok"])
        self.assertFalse(aggregate["ranking_weight"])


class ContinuousReplayAdapterTests(unittest.TestCase):
    def test_one_fsm_call_yields_four_local_rows_without_reset(self) -> None:
        times = np.asarray([0.0, 0.5, 1.0])
        accepted = [
            {
                "event": "heel_strike",
                "event_time_s": 0.0,
                "confirmed_time_s": 0.03,
                "segment_valid": 1.0,
                "matched_sensor_candidate": True,
            },
            {
                "event": "toe_off",
                "event_time_s": 0.5,
                "confirmed_time_s": 0.53,
                "segment_valid": 1.0,
                "matched_sensor_candidate": True,
            },
            {
                "event": "heel_strike",
                "event_time_s": 1.0,
                "confirmed_time_s": 1.03,
                "segment_valid": 1.0,
                "matched_sensor_candidate": True,
            },
        ]
        replay = {
            "heel_contact": np.asarray([1.0, 0.0, 1.0]),
            "toe_contact": np.asarray([0.0, 1.0, 0.0]),
            "state_id": np.asarray([1.0, 2.0, 1.0]),
            "accepted": accepted,
            "candidates": [],
            "sensor_edges": [],
            "invalid_steps": [],
        }
        gate = subject._dynamic_gate(2, 1)
        references = tuple(
            {
                "plateau_index": index,
                "speed_mps": float(index),
                "events": {
                    "heel_strike": np.asarray([0.03, 1.03]),
                    "toe_off": np.asarray([0.53]),
                },
                "dynamic_gate": gate,
                "plateau_interval_s": [-1.0, 2.0],
            }
            for index in range(1, 5)
        )
        bundle = subject.TrialCadenceBundle(
            trial_id="02",
            cadence_label="runtime_10ms",
            sample_dt_s=0.010,
            plan=SimpleNamespace(),
            samples={},
            shared={
                "times": times,
                "primary_penetration": np.zeros(3),
                "primary_aggregate": np.ones(3),
                "kinematics": {"knee_rad": np.zeros(3), "ankle_rad": np.zeros(3)},
                "body_weight_n": 700.0,
                "prescribed_vertical_n": np.asarray([100.0, 0.0, 100.0]),
            },
            plateau_references=references,
            access={},
        )
        candidate = SimpleNamespace(
            candidate_id="candidate",
            selectable=True,
            geometry={
                "parameter_arm": "combination",
                "heel_x_shift_from_v9_mm": 3.75,
                "heel_radius_reduction_from_v13_mm": 0.05,
                "toe_down_from_v9_mm": 0.85,
                "toe_radius_reduction_from_v13_mm": 0.05,
                "heel_radius_m": 0.02,
                "toe_radius_m": 0.02,
                "geometry_displacement_from_v13_m": 0.0004,
                "pre_gate_ok": True,
            },
        )
        runtime_cfg = subject.v1._current_runtime_fsm_config()
        phase = {
            "ok": True,
            "forbidden_mismatch_count": 0,
            "settled_outside_transition_windows_agreement": {
                "unknown_fsm_samples": 0
            },
            "_arrays": {
                "strict_mask": np.ones(3, dtype=bool),
                "reference_phase": np.asarray([1, 0, 1]),
            },
        }
        semantic = {
            "ok": True,
            "observed_valid_cycles": 1,
            "timeout_transitions": [],
            "unaccepted_sensor_gait_events": [],
        }
        transfer = {
            "both_latches_off_sample_count": 0,
            "incomplete_transfer_cycle_count": 0,
        }
        early = {"early_toe_off_candidate_count": 0}
        with (
            patch.object(subject, "_pair_for_candidate", return_value=(object(), {})),
            patch.object(
                subject.v1,
                "_contact_inputs",
                return_value=(
                    {"left_heel": np.ones(3), "left_toe": np.ones(3)},
                    {"left_heel": np.zeros(3), "left_toe": np.zeros(3)},
                    np.ones(3),
                ),
            ),
            patch.object(subject.v1, "_current_runtime_fsm_config", return_value=runtime_cfg),
            patch.object(subject.v1, "_run_production_fsm", return_value=replay) as run_fsm,
            patch.object(
                subject,
                "_slice_continuous_replay",
                return_value=(np.ones(3, dtype=bool), replay),
            ),
            patch.object(subject, "_local_semantic_gate", return_value=semantic),
            patch.object(subject.thresholds, "_phase_classification_gate", return_value=phase),
            patch.object(
                subject.thresholds,
                "_fsm_phase_from_state_id",
                return_value=np.asarray([1, 0, 1]),
            ),
            patch.object(
                subject.v6.v4.v3,
                "heel_to_forefoot_transfer_diagnostics",
                return_value=transfer,
            ),
            patch.object(
                subject.v6.v4.v3,
                "early_to_candidate_diagnostics",
                return_value=early,
            ),
            patch.object(subject.v10, "strict_gate", return_value={"ok": True, "checks": {}}),
        ):
            rows, details = subject.evaluate_continuous_candidate(
                {"replay": subject.v11._expected_replay()},
                object(),
                candidate,
                bundle,
            )
        self.assertEqual(run_fsm.call_count, 1)
        self.assertEqual(len(rows), 4)
        self.assertEqual(len(details), 4)
        self.assertTrue(all(row["continuous_parent_fsm_replay"] for row in rows))
        self.assertTrue(all(not row["fsm_reset_for_plateau"] for row in rows))
        self.assertTrue(
            all("heel_strike_onset_error_s_mean" in row for row in rows)
        )

    def test_decomposition_is_empty_when_cardinality_or_order_is_not_aligned(self) -> None:
        reference = {
            "heel_strike": np.asarray([0.0, 1.0]),
            "toe_off": np.asarray([0.5]),
        }
        accepted = [
            {"event": "heel_strike", "event_time_s": 0.0, "confirmed_time_s": 0.03},
            {"event": "heel_strike", "event_time_s": 0.4, "confirmed_time_s": 0.43},
            {"event": "toe_off", "event_time_s": 0.5, "confirmed_time_s": 0.53},
        ]
        result = subject.causal_event_decomposition(
            reference,
            accepted,
            sample_dt_s=0.01,
            dwell_s=0.03,
            hs_tolerance_s=0.05,
            toe_off_tolerance_s=0.08,
        )
        for event in ("heel_strike", "toe_off"):
            self.assertFalse(result[event]["alignment_complete"])
            for component in (
                "onset_error_s",
                "confirmation_latency_s",
                "total_error_s",
                "residual_quantization_or_routing_s",
            ):
                self.assertEqual(result[event][component]["count"], 0)
                self.assertIsNone(result[event][component]["mean"])
            self.assertEqual(result[event]["events"], [])

        same_count_but_unmatched = [
            {"event": "heel_strike", "event_time_s": 0.0, "confirmed_time_s": 0.20},
            {"event": "toe_off", "event_time_s": 0.5, "confirmed_time_s": 0.53},
            {"event": "heel_strike", "event_time_s": 1.0, "confirmed_time_s": 1.03},
        ]
        unmatched = subject.causal_event_decomposition(
            reference,
            same_count_but_unmatched,
            sample_dt_s=0.01,
            dwell_s=0.03,
            hs_tolerance_s=0.05,
            toe_off_tolerance_s=0.08,
        )
        self.assertFalse(unmatched["heel_strike"]["alignment_complete"])
        self.assertEqual(unmatched["heel_strike"]["events"], [])
        self.assertEqual(unmatched["heel_strike"]["total_error_s"]["count"], 0)


class Stage2AndBoundaryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        protocol = subject.expected_protocol_payload(
            require_metadata_audit=False, require_all_sources=False
        )
        cls.protocol = protocol
        cls.base, cls.isolated, cls.geometry = subject.build_isolated_candidates(
            protocol
        )

    def test_outer_winners_build_exact_81_tuple_cartesian_without_sampling(self) -> None:
        parameter = {
            "Hx": "heel_x_shift_from_v9_mm",
            "Hr": "heel_radius_reduction_from_v13_mm",
            "Ty": "toe_down_from_v9_mm",
            "Tr": "toe_radius_reduction_from_v13_mm",
        }
        winners = {}
        for arm in subject.ARM_ORDER:
            winners[arm] = next(
                item.candidate_id
                for item in self.isolated
                if item.geometry["parameter_arm"] == arm
                and np.isclose(
                    item.geometry[parameter[arm]],
                    subject.OUTER_BOUNDARY_VALUES[arm],
                )
            )
        rows = [
            {"candidate_id": item.candidate_id, "v14_stage_unit": "unit"}
            for item in self.isolated
        ]
        pool, audit = subject.build_combinations(
            self.protocol, winners, self.isolated, self.geometry, rows
        )
        self.assertEqual(audit["cartesian_tuple_count_including_v13"], 81)
        self.assertEqual(len(pool), 80)
        self.assertEqual(audit["new_opensim_station_sampling_passes"], 0)
        self.assertTrue(audit["reused_stage1_candidate_ids"])
        self.assertTrue(audit["new_candidate_ids"])

    def test_improving_outer_isolated_rank_stops_before_validation(self) -> None:
        baseline = next(
            item for item in self.isolated if item.candidate_id == subject.BASELINE_ID
        )
        hx = [
            item
            for item in self.isolated
            if item.geometry["parameter_arm"] == "Hx"
        ]
        outer = next(
            item
            for item in hx
            if np.isclose(
                item.geometry["heel_x_shift_from_v9_mm"],
                subject.OUTER_BOUNDARY_VALUES["Hx"],
            )
        )
        inner = next(
            item
            for item in hx
            if np.isclose(item.geometry["heel_x_shift_from_v9_mm"], 5.5)
        )

        def rank_row(candidate: Any, hs: float) -> dict[str, Any]:
            row = _passing_row(1)
            row.update(
                {
                    "candidate_id": candidate.candidate_id,
                    "v14_stage_unit": "unit",
                    "expected_reference_hs_count": 11,
                    "expected_reference_to_count": 10,
                    "expected_valid_cycle_count": 10,
                    "max_abs_hs_error_s": hs,
                    "max_abs_toe_off_error_s": 0.03,
                    "confirmed_fsm_stance_f1": 0.98,
                    "confirmed_fsm_stance_iou": 0.95,
                    "precision": 1.0,
                    "recall": 1.0,
                    "cell_mean_normalized_error": hs / 0.05,
                    "mesh_geometry_pre_gate_ok": True,
                    "heel_radius_m": candidate.geometry["heel_radius_m"],
                    "toe_radius_m": candidate.geometry["toe_radius_m"],
                }
            )
            return row

        rows = [
            rank_row(baseline, 0.04),
            rank_row(inner, 0.03),
            rank_row(outer, 0.02),
        ]
        result = subject.boundary_saturation_stop(
            outer, rows, (baseline, inner, outer)
        )
        self.assertTrue(result["stop_before_validation"])
        self.assertEqual(result["stop_arms"], ["Hx"])


class SelectorRegressionTests(unittest.TestCase):
    def test_holdout_rejects_lexicographically_better_but_pareto_inferior_primary(self) -> None:
        rows: list[dict[str, Any]] = []
        for candidate_id, hs_error, to_error in (
            (subject.BASELINE_ID, 0.045, 0.040),
            ("primary", 0.035, 0.048),
        ):
            for cadence in ("runtime_10ms", "fine_1ms"):
                for plateau in range(1, 5):
                    row = _passing_row(plateau)
                    row.update(
                        {
                            "candidate_id": candidate_id,
                            "trial_id": "05",
                            "cadence": cadence,
                            "v14_stage_unit": f"05::{plateau}::{cadence}",
                            "precision": 1.0,
                            "recall": 1.0,
                            "max_abs_hs_error_s": hs_error,
                            "max_abs_toe_off_error_s": to_error,
                            "cell_mean_normalized_error": 0.5
                            * (hs_error / 0.05 + to_error / 0.08),
                            "confirmed_fsm_stance_f1": 0.98,
                            "confirmed_fsm_stance_iou": 0.95,
                            "unit_full_gate_ok": True,
                            "trial_aggregate_gate_ok": True,
                        }
                    )
                    rows.append(row)
        decision = subject.paired_holdout_decision(
            rows, "primary", stage="validation"
        )
        self.assertTrue(decision["primary_ahead_of_v13_beyond_epsilon"])
        self.assertFalse(decision["pareto_vs_v13"]["pareto_noninferior"])
        self.assertFalse(decision["ok"])


class AccessAndGoldenPreprocessingTests(unittest.TestCase):
    @staticmethod
    def _candidate() -> Any:
        return SimpleNamespace(
            candidate_id="primary",
            selectable=True,
            role="local_cartesian_full_gate_finalist",
            heel_location=(1.0, 2.0, 3.0),
            forefoot_location=(4.0, 5.0, 6.0),
            geometry={"heel_radius_m": 0.02, "toe_radius_m": 0.02},
        )

    @staticmethod
    def _protocol(protocol_path: Path) -> dict[str, Any]:
        return {
            "protocol_id": "test-protocol",
            "_protocol_sha256": "abc123",
            "_protocol_path": protocol_path.as_posix(),
            "split": {
                "DEVELOPMENT": ["02"],
                "VALIDATION": ["05"],
                "SEALED": ["06"],
                "RESERVE": ["03", "07"],
            },
        }

    def test_receipt_requires_canonical_authorizing_lock_and_rejects_tamper(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            protocol_path = root / "protocol.json"
            protocol_path.write_text("{}", encoding="utf-8")
            protocol = self._protocol(protocol_path)
            candidate = self._candidate()
            record = subject._candidate_record(candidate)
            dev_lock = subject._write_json_exclusive(
                root / "development_candidate_lock.json",
                {
                    "status": "DEVELOPMENT_DECISION_FROZEN_BEFORE_HOLDOUT_ACCESS",
                    "protocol_sha256": protocol["_protocol_sha256"],
                    "finalist_id": candidate.candidate_id,
                    "finalist_record": record,
                    "validation_semantic_access_allowed": True,
                    "boundary_saturation_stop": {"stop_before_validation": False},
                },
            )
            receipt = subject._write_stage_access_receipt(
                root,
                protocol,
                stage="validation",
                trial_id="05",
                primary=candidate,
                development_candidate_lock=dev_lock,
            )
            subject._assert_semantic_access(
                protocol,
                trial_id="05",
                stage="validation",
                access_receipt=receipt,
            )
            wrong_protocol = {**protocol, "_protocol_sha256": "wrong"}
            with self.assertRaises(subject.ProtocolError):
                subject._assert_semantic_access(
                    wrong_protocol,
                    trial_id="05",
                    stage="validation",
                    access_receipt=receipt,
                )
            dev_lock.write_text("{}", encoding="utf-8")
            with self.assertRaises(subject.ProtocolError):
                subject._assert_semantic_access(
                    protocol,
                    trial_id="05",
                    stage="validation",
                    access_receipt=receipt,
                )

    def test_sealed_receipt_rejects_nonpassing_validation_lock(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            protocol_path = root / "protocol.json"
            protocol_path.write_text("{}", encoding="utf-8")
            protocol = self._protocol(protocol_path)
            candidate = self._candidate()
            dev_lock = subject._write_json_exclusive(
                root / "development_candidate_lock.json",
                {
                    "status": "DEVELOPMENT_DECISION_FROZEN_BEFORE_HOLDOUT_ACCESS",
                    "protocol_sha256": protocol["_protocol_sha256"],
                    "finalist_id": candidate.candidate_id,
                    "finalist_record": subject._candidate_record(candidate),
                    "validation_semantic_access_allowed": True,
                    "boundary_saturation_stop": {"stop_before_validation": False},
                },
            )
            validation_lock = subject._write_json_exclusive(
                root / "validation_decision_lock.json",
                {
                    "status": "VALIDATION_DECISION_FROZEN_BEFORE_SEALED_ACCESS",
                    "protocol_sha256": protocol["_protocol_sha256"],
                    "primary_candidate_id": candidate.candidate_id,
                    "development_candidate_lock": subject._source_record(dev_lock),
                    "validation_decision": {"ok": False},
                    "sealed_semantic_access_allowed": False,
                },
            )
            with self.assertRaises(subject.ProtocolError):
                subject._write_stage_access_receipt(
                    root,
                    protocol,
                    stage="sealed",
                    trial_id="06",
                    primary=candidate,
                    development_candidate_lock=dev_lock,
                    validation_decision_lock=validation_lock,
                )

    def test_canonical_destination_and_global_ledger_fail_closed(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            canonical = root / "canonical"
            ledger = root / "ledger.json"
            with (
                patch.object(subject, "DEFAULT_OUTPUT_DIR", canonical),
                patch.object(subject, "DEFAULT_EXECUTION_LEDGER", ledger),
            ):
                subject._preflight_no_clobber(canonical)
                with self.assertRaises(subject.ProtocolError):
                    subject._preflight_no_clobber(root / "alternate")
                ledger.write_text("{}", encoding="utf-8")
                with self.assertRaises(subject.NoClobberError):
                    subject._preflight_no_clobber(canonical)

    def test_jsonl_details_are_no_clobber_and_hash_pinnable(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "details.jsonl"
            subject._write_jsonl_exclusive(
                path,
                [
                    {"candidate_id": "a", "value": 1.0},
                    {"candidate_id": "b", "value": float("nan")},
                ],
            )
            lines = path.read_text(encoding="utf-8").splitlines()
            self.assertEqual(len(lines), 2)
            self.assertIn('"value":null', lines[1])
            source = subject._source_record(path)
            self.assertEqual(source["sha256"], subject._sha256(path))
            with self.assertRaises(subject.NoClobberError):
                subject._write_jsonl_exclusive(path, [{"candidate_id": "c"}])

    def test_post_start_no_clobber_persists_failure_without_overwrite(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary) / "run"
            output.mkdir()
            (output / "run_start_receipt.json").write_text("{}", encoding="utf-8")
            try:
                raise subject.NoClobberError("synthetic post-start collision")
            except subject.NoClobberError as exc:
                subject._write_failure(output, exc)
            failure = output / "failure.json"
            self.assertTrue(failure.is_file())
            before = failure.read_bytes()
            subject._write_failure(output, subject.NoClobberError("second"))
            self.assertEqual(failure.read_bytes(), before)

    def test_main_does_not_mutate_existing_alternate_output_directory(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            alternate = Path(temporary) / "alternate"
            alternate.mkdir()
            sentinel = alternate / "sentinel.bin"
            sentinel.write_bytes(b"unchanged")
            before = {path.name: path.read_bytes() for path in alternate.iterdir()}
            with patch("builtins.print"):
                status = subject.main(
                    ["--execute", "--output-dir", alternate.as_posix()]
                )
            after = {path.name: path.read_bytes() for path in alternate.iterdir()}
            self.assertEqual(status, 2)
            self.assertEqual(after, before)

    def test_canonical_retry_with_prior_run_start_is_strictly_read_only(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            canonical = root / "canonical"
            canonical.mkdir()
            (canonical / "run_start_receipt.json").write_text("{}", encoding="utf-8")
            (canonical / "sentinel.bin").write_bytes(b"prior-run")
            ledger = root / "ledger.json"
            ledger.write_text("{}", encoding="utf-8")
            before = {
                path.name: path.read_bytes() for path in canonical.iterdir()
            }
            with (
                patch.object(subject, "DEFAULT_OUTPUT_DIR", canonical),
                patch.object(subject, "DEFAULT_EXECUTION_LEDGER", ledger),
                patch("builtins.print"),
            ):
                status = subject.main(
                    ["--execute", "--output-dir", canonical.as_posix()]
                )
            after = {path.name: path.read_bytes() for path in canonical.iterdir()}
            self.assertEqual(status, 2)
            self.assertEqual(after, before)

    def test_prepare_trial_uses_golden_chain_and_marker_ik_only(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            sources = {}
            for name in ("conditions", "ik", "fp", "markers"):
                path = root / f"{name}.mat"
                path.write_bytes(name.encode("ascii"))
                sources[name] = {"path": path.as_posix(), "size_bytes": path.stat().st_size, "sha256": "stub"}
            model = root / "model.osim"
            model.write_text("model", encoding="utf-8")
            plugin_binary = root / "plugin.dylib"
            plugin_binary.write_bytes(b"plugin")
            plugin_loader = root / "plugin_loader"
            work = root / "converted"
            stem = "treadmill_02_01"

            def convert_side_effect(**kwargs: Any) -> dict[str, Any]:
                work.mkdir()
                paths = {
                    "ik_mot": work / f"{stem}_ik_dataset_ab06_seasea.mot",
                    "grf_mot": work / f"{stem}_grf.mot",
                    "trc": work / f"{stem}.trc",
                    "external_loads_xml": work / f"{stem}_ExternalLoads.xml",
                    "ik_setup_xml": work / f"{stem}_iksetup.xml",
                    "conversion_manifest": work / f"{stem}_conversion_manifest.json",
                }
                for path in paths.values():
                    path.write_text("stub", encoding="utf-8")
                return {
                    "status": "ok",
                    "trial": stem,
                    "output_dir": work.as_posix(),
                    **{key: value.as_posix() for key, value in paths.items()},
                    "time_range_s": [0.0, 2.0],
                }

            ik_motion = work / f"{stem}_ik.mot"
            ik_receipt = work / f"{stem}_ik_receipt.json"

            def run_side_effect(**_kwargs: Any) -> dict[str, Any]:
                ik_motion.write_text("marker IK", encoding="utf-8")
                return {"status": "ok"}

            def finalize_side_effect(**_kwargs: Any) -> dict[str, Any]:
                ik_receipt.write_text("{}", encoding="utf-8")
                return {
                    "status": "IK_OUTPUT_VERIFIED",
                    "receipt": ik_receipt.as_posix(),
                }

            protocol = {
                **self._protocol(root / "protocol.json"),
                "trials": {
                    "02": {
                        "trial_interval_s": [0.1, 1.9],
                        "raw_sources": sources,
                    }
                },
                "model_file": model.as_posix(),
                "replay": {"sea_plugin": plugin_loader.as_posix()},
                "reserve_actuators_xml": (root / "reserve.xml").as_posix(),
            }
            setup_stub = SimpleNamespace(
                model_file=model,
                kinematics_file=ik_motion,
                external_loads_xml=work / f"{stem}_ExternalLoads.xml",
            )

            def hash_side_effect(path: Path) -> str:
                if Path(path).resolve() == model.resolve():
                    return subject.metadata.EXPECTED_MARKER_CALIBRATED_MODEL_SHA256
                if Path(path).resolve() == plugin_binary.resolve():
                    return subject.EXPECTED_PLUGIN_BINARY_SHA256_BY_SUFFIX[".dylib"]
                return "stub"

            with (
                patch.object(subject, "_verify_raw_identity", side_effect=lambda record: Path(record["path"])),
                patch.object(subject, "_sha256", side_effect=hash_side_effect),
                patch.object(
                    subject.converter,
                    "resolve_plugin_spec",
                    return_value=SimpleNamespace(binary=plugin_binary, loader_basename=plugin_loader),
                ),
                patch.object(subject.converter, "convert_trial", side_effect=convert_side_effect) as convert,
                patch.object(subject.converter, "run_ik_from_setup", side_effect=run_side_effect) as run_ik,
                patch.object(subject.converter, "finalize_ik_receipt", side_effect=finalize_side_effect) as finalize,
                patch.object(subject.setup_io, "build_simulation_setup", return_value=setup_stub) as build_setup,
            ):
                artifacts = subject.prepare_trial(
                    protocol,
                    trial_id="02",
                    stage="development",
                    work_dir=work,
                    access_receipt=None,
                )
            self.assertEqual(convert.call_count, 1)
            self.assertEqual(run_ik.call_count, 1)
            self.assertEqual(finalize.call_count, 1)
            self.assertEqual(artifacts.ik_motion, ik_motion)
            self.assertEqual(
                Path(build_setup.call_args.kwargs["kinematics_file"]), ik_motion
            )
            self.assertNotEqual(
                Path(build_setup.call_args.kwargs["kinematics_file"]),
                work / f"{stem}_ik_dataset_ab06_seasea.mot",
            )


if __name__ == "__main__":
    unittest.main()
