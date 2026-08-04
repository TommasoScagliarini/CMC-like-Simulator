"""Pure-fixture tests for the prescribed two-sensor threshold sweep.

These tests intentionally exercise only protocol parsing, deterministic
ranking, metric arithmetic, and stage-lineage checks.  They do not load an
OpenSim model or evaluate detector contact loads.
"""

from __future__ import annotations

import copy
import json
import math
import sys
import tempfile
import unittest
from pathlib import Path
from types import ModuleType
from typing import Any
from unittest.mock import patch

import numpy as np


ROOT_DIR = Path(__file__).resolve().parents[1]
VALIDATION_DIR = ROOT_DIR / "validation"
V3_PROTOCOL_PATH = (
    VALIDATION_DIR / "two_sensor_prescribed_threshold_sweep_protocol_v3.json"
)
if str(VALIDATION_DIR) not in sys.path:
    sys.path.insert(0, str(VALIDATION_DIR))


class _ForbiddenOpenSim(ModuleType):
    """Import sentinel proving that these unit tests use no OpenSim API."""

    def __getattr__(self, name: str) -> Any:
        raise AssertionError(f"pure threshold-sweep test touched opensim.{name}")


with patch.dict(sys.modules, {"opensim": _ForbiddenOpenSim("opensim")}):
    import sweep_two_sensor_prescribed_thresholds as subject  # noqa: E402


class TwoSensorPrescribedThresholdSweepTest(unittest.TestCase):
    def setUp(self) -> None:
        self.protocol = subject.load_and_validate_protocol()

    def _write_protocol(self, payload: dict[str, Any]) -> Path:
        temporary_directory = tempfile.TemporaryDirectory()
        self.addCleanup(temporary_directory.cleanup)
        path = Path(temporary_directory.name) / "protocol.json"
        path.write_text(json.dumps(payload), encoding="utf-8")
        return path

    def _development_row(
        self,
        candidate_id: str,
        *,
        eligible: bool = True,
        worst: float = 0.50,
        mean: float = 0.25,
        sensor_iou: float = 0.96,
        fsm_iou: float = 0.95,
    ) -> dict[str, Any]:
        return {
            "candidate_id": candidate_id,
            "eligible": eligible,
            "worst_event_normalized_max_abs_error": worst,
            "mean_event_normalized_mean_abs_error": mean,
            "sensor_contact_stance_iou": sensor_iou,
            "confirmed_fsm_stance_iou": fsm_iou,
        }

    def _holdout_metrics(self, candidate_id: str) -> dict[str, Any]:
        return {
            "candidate_id": candidate_id,
            "primary_event_time_field": "confirmed_time_s",
            "exact_reference_and_detector_event_counts": True,
            "precision": 1.0,
            "recall": 1.0,
            "max_abs_hs_error_s": 0.050,
            "max_abs_toe_off_error_s": 0.080,
            "invalid_or_timeout_transition_count": 0,
            "unaccepted_sensor_gait_event_count": 0,
            "exact_hs_to_toe_off_to_hs_order_and_cycle_count": True,
            "confirmation_latency_in_range": True,
            "forbidden_phase_mismatch_count": 0,
            "unknown_fsm_phase_samples": 0,
            "sensor_contact_stance_f1": 0.94,
            "sensor_contact_stance_iou": 0.89,
            "confirmed_fsm_stance_f1": 0.95,
            "confirmed_fsm_stance_iou": 0.90,
            "worst_event_normalized_max_abs_error": 1.0,
        }

    def _v3_protocol(self) -> dict[str, Any]:
        return subject.load_and_validate_protocol(V3_PROTOCOL_PATH)

    def _evaluate_v3_fixture(
        self,
        accepted: list[dict[str, Any]],
    ) -> tuple[dict[str, Any], dict[str, Any]]:
        """Evaluate timing semantics without OpenSim or detector contact loads."""

        protocol = self._v3_protocol()
        candidate = next(
            item
            for item in subject.build_candidate_grid(protocol)
            if item.candidate_id == "on00p50_off00p01"
        )
        times = np.asarray([0.0, 0.01, 0.02], dtype=float)
        reference_phase = np.asarray([1, 0, 1], dtype=int)

        class _FakeFSM:
            @staticmethod
            def payload() -> dict[str, Any]:
                return {"valid_cycle_count": 1}

        replay = {
            "accepted": accepted,
            "invalid_steps": [],
            "heel_contact": np.asarray([1.0, 0.0, 1.0]),
            "toe_contact": np.zeros(3, dtype=float),
            "state_id": np.zeros(3, dtype=float),
            "fsm": _FakeFSM(),
        }
        inputs = {
            "times": times,
            "loads": {
                "left_heel": np.asarray([1.0, 0.0, 1.0]),
                "left_toe": np.zeros(3, dtype=float),
            },
            "penetrations": {
                "left_heel": np.zeros(3, dtype=float),
                "left_toe": np.zeros(3, dtype=float),
            },
            "aggregate": np.asarray([20.0, 0.0, 20.0]),
            "kinematics": {
                "knee_rad": np.zeros(3, dtype=float),
                "ankle_rad": np.zeros(3, dtype=float),
            },
            "body_weight_n": 700.0,
            "prescribed_vertical_n": np.asarray([20.0, 0.0, 20.0]),
            "reference_events": {
                "heel_strike": np.asarray([1.0, 2.0]),
                "toe_off": np.asarray([1.5]),
            },
        }
        semantic = {
            "ok": True,
            "timeout_transitions": [],
            "unaccepted_sensor_gait_events": [],
        }
        phase = {
            "ok": True,
            "forbidden_mismatch_count": 0,
            "settled_outside_transition_windows_agreement": {
                "unknown_fsm_samples": 0,
            },
            "_arrays": {
                "strict_mask": np.ones(3, dtype=bool),
                "reference_phase": reference_phase,
            },
        }
        runtime_config = subject._current_runtime_fsm_config()
        with (
            patch.object(subject, "_current_runtime_fsm_config", return_value=runtime_config),
            patch.object(subject, "_run_production_fsm", return_value=replay),
            patch.object(subject, "_semantic_gate", return_value=semantic),
            patch.object(subject, "_phase_classification_gate", return_value=phase),
            patch.object(
                subject,
                "_fsm_phase_from_state_id",
                return_value=reference_phase,
            ),
        ):
            return subject._evaluate_candidate(
                candidate,
                inputs,
                protocol,
                sample_dt_s=0.01,
            )

    def test_default_protocol_preserves_sealed_chronological_contract(self) -> None:
        split = self.protocol["chronological_split"]
        self.assertEqual(split["development_time_block_s"], [11.99, 50.0])
        self.assertEqual(split["validation_time_block_s"], [50.0, 100.0])
        self.assertEqual(split["sealed_time_block_s"], [100.0, 155.045])
        self.assertIs(split["selection_may_read_validation_metrics"], False)
        self.assertIs(
            split["selection_or_validation_may_read_sealed_metrics"], False
        )
        self.assertEqual(
            split["validation_candidates"],
            "selected development winner and baseline only",
        )
        self.assertTrue(self.protocol["frozen_before_execution"])

    def test_v3_protocol_freezes_confirmed_primary_and_onset_diagnostic(self) -> None:
        protocol = self._v3_protocol()

        self.assertEqual(protocol["schema_version"], 2)
        self.assertEqual(
            protocol["replay"]["primary_event_time_field"],
            "confirmed_time_s",
        )
        self.assertEqual(
            protocol["replay"]["diagnostic_event_time_field"],
            "event_time_s",
        )
        self.assertEqual(
            protocol["replay"]["phase_reference_mode"],
            "validated_event_intervals",
        )
        contract = protocol["event_time_contract"]
        self.assertEqual(contract["detector_onset_role"], "diagnostic_only")
        self.assertIs(
            contract[
                "onset_error_vs_reference_may_affect_development_eligibility"
            ],
            False,
        )
        self.assertIs(
            contract["onset_error_vs_reference_may_affect_ranking"],
            False,
        )
        self.assertIs(
            contract[
                "onset_error_vs_reference_may_affect_validation_or_sealed_gate"
            ],
            False,
        )
        split = protocol["chronological_split"]
        self.assertEqual(
            split["validation_status_at_protocol_freeze"],
            "CLOSED_UNEVALUATED",
        )
        self.assertEqual(
            split["sealed_status_at_protocol_freeze"],
            "CLOSED_UNEVALUATED",
        )
        self.assertIs(split["selection_may_read_validation_metrics"], False)
        self.assertIs(
            split["selection_or_validation_may_read_sealed_metrics"],
            False,
        )

    def test_v3_protocol_rejects_implicit_or_reversed_time_basis(self) -> None:
        frozen = self._v3_protocol()
        mutations: list[tuple[str, Any]] = [
            (
                "missing primary",
                lambda raw: raw["replay"].pop("primary_event_time_field"),
            ),
            (
                "missing diagnostic",
                lambda raw: raw["replay"].pop("diagnostic_event_time_field"),
            ),
            (
                "onset made primary",
                lambda raw: raw["replay"].__setitem__(
                    "primary_event_time_field", "event_time_s"
                ),
            ),
            (
                "confirmation made diagnostic",
                lambda raw: raw["replay"].__setitem__(
                    "diagnostic_event_time_field", "confirmed_time_s"
                ),
            ),
            (
                "missing phase reference mode",
                lambda raw: raw["replay"].pop("phase_reference_mode"),
            ),
            (
                "instantaneous noisy phase reference",
                lambda raw: raw["replay"].__setitem__(
                    "phase_reference_mode", "instantaneous_grf"
                ),
            ),
        ]
        for label, mutate in mutations:
            with self.subTest(label=label):
                raw = copy.deepcopy(frozen)
                raw.pop("_protocol_path", None)
                raw.pop("_protocol_sha256", None)
                mutate(raw)
                with self.assertRaises((subject.ProtocolError, ValueError)):
                    subject.load_and_validate_protocol(self._write_protocol(raw))

    def test_v3_time_extraction_fails_closed_on_bad_confirmed_timestamp(self) -> None:
        valid = [
            {
                "event": "heel_strike",
                "event_time_s": 0.97,
                "confirmed_time_s": 1.0,
                "segment_valid": 1.0,
            }
        ]
        extracted = subject._accepted_events_for_time_field(
            valid,
            "confirmed_time_s",
        )
        np.testing.assert_allclose(extracted["heel_strike"], [1.0])

        for label, confirmed in (
            ("missing", None),
            ("nan", math.nan),
            ("positive infinity", math.inf),
            ("not numeric", "bad"),
        ):
            with self.subTest(label=label):
                corrupt = copy.deepcopy(valid)
                if confirmed is None:
                    corrupt[0].pop("confirmed_time_s")
                else:
                    corrupt[0]["confirmed_time_s"] = confirmed
                with self.assertRaises((subject.ProtocolError, ValueError)):
                    subject._accepted_events_for_time_field(
                        corrupt,
                        "confirmed_time_s",
                    )

    def test_v3_event_order_uses_confirmed_time_field_explicitly(self) -> None:
        accepted = [
            {
                "event": "heel_strike",
                "event_time_s": 0.10,
                "confirmed_time_s": 1.0,
                "segment_valid": 1.0,
            },
            {
                "event": "toe_off",
                "event_time_s": 0.30,
                "confirmed_time_s": 2.0,
                "segment_valid": 1.0,
            },
            {
                "event": "heel_strike",
                "event_time_s": 0.20,
                "confirmed_time_s": 3.0,
                "segment_valid": 1.0,
            },
        ]

        self.assertTrue(
            subject._exact_event_order(
                accepted,
                1,
                time_field="confirmed_time_s",
            )
        )
        self.assertFalse(
            subject._exact_event_order(
                accepted,
                1,
                time_field="event_time_s",
            )
        )

    def test_v3_evaluation_gates_confirmed_time_and_reports_onset_only(self) -> None:
        # Primary confirmed timestamps are just inside the frozen gates.  Raw
        # onsets are 40 ms earlier (the maximum causal debounce at 10 ms), so
        # their HS/TO errors deliberately fail the old onset-based V2 gates.
        accepted = [
            {
                "event": "heel_strike",
                "event_time_s": 0.911,
                "confirmed_time_s": 0.951,
                "segment_valid": 1.0,
            },
            {
                "event": "toe_off",
                "event_time_s": 1.381,
                "confirmed_time_s": 1.421,
                "segment_valid": 1.0,
            },
            {
                "event": "heel_strike",
                "event_time_s": 1.911,
                "confirmed_time_s": 1.951,
                "segment_valid": 1.0,
            },
        ]

        row, detail = self._evaluate_v3_fixture(accepted)

        self.assertTrue(row["eligible"], row)
        self.assertAlmostEqual(row["max_abs_hs_error_s"], 0.049)
        self.assertAlmostEqual(row["max_abs_toe_off_error_s"], 0.079)
        self.assertEqual(detail["primary_event_time_field"], "confirmed_time_s")
        self.assertEqual(detail["diagnostic_event_time_field"], "event_time_s")
        self.assertAlmostEqual(
            detail["primary_event_diagnostics"]["heel_strike"][
                "timing_max_abs_s"
            ],
            0.049,
        )
        self.assertAlmostEqual(
            detail["diagnostic_event_diagnostics_not_gate"]["heel_strike"][
                "timing_max_abs_s"
            ],
            0.089,
        )
        self.assertAlmostEqual(
            detail["diagnostic_event_diagnostics_not_gate"]["toe_off"][
                "timing_max_abs_s"
            ],
            0.119,
        )

    def test_v3_holdout_gate_ignores_onset_diagnostics(self) -> None:
        protocol = self._v3_protocol()
        candidate = self._holdout_metrics("on00p50_off00p01")
        baseline = self._holdout_metrics("on005_off002")
        candidate.update(
            {
                "diagnostic_onset_max_abs_hs_error_s": 999.0,
                "diagnostic_onset_max_abs_toe_off_error_s": 999.0,
                "diagnostic_onset_worst_event_normalized_max_abs_error": 999.0,
            }
        )

        result = subject.evaluate_holdout_gate(candidate, baseline, protocol)

        self.assertTrue(result["ok"], result)

    def test_v3_ranking_uses_primary_confirmed_metrics_not_onset(self) -> None:
        protocol = self._v3_protocol()
        confirmed_winner = self._development_row(
            "on00p50_off00p01",
            worst=0.40,
            mean=0.20,
        )
        onset_winner_only = self._development_row(
            "on00p75_off00p01",
            worst=0.60,
            mean=0.30,
        )
        confirmed_winner["diagnostic_onset_worst_event_normalized_max_abs_error"] = 999.0
        confirmed_winner["diagnostic_onset_mean_event_normalized_mean_abs_error"] = 999.0
        onset_winner_only["diagnostic_onset_worst_event_normalized_max_abs_error"] = 0.0
        onset_winner_only["diagnostic_onset_mean_event_normalized_mean_abs_error"] = 0.0

        selected = subject.select_development_candidate(
            [onset_winner_only, confirmed_winner],
            protocol,
        )

        self.assertEqual(selected["candidate_id"], "on00p50_off00p01")

    def test_v3_holdout_gate_fails_closed_without_primary_confirmed_metrics(self) -> None:
        protocol = self._v3_protocol()
        baseline = self._holdout_metrics("on005_off002")
        for field in (
            "primary_event_time_field",
            "exact_reference_and_detector_event_counts",
            "precision",
            "recall",
            "max_abs_hs_error_s",
            "max_abs_toe_off_error_s",
            "invalid_or_timeout_transition_count",
            "unaccepted_sensor_gait_event_count",
            "exact_hs_to_toe_off_to_hs_order_and_cycle_count",
            "confirmation_latency_in_range",
            "forbidden_phase_mismatch_count",
            "unknown_fsm_phase_samples",
            "confirmed_fsm_stance_f1",
            "confirmed_fsm_stance_iou",
            "worst_event_normalized_max_abs_error",
        ):
            with self.subTest(field=field):
                candidate = self._holdout_metrics("on00p50_off00p01")
                candidate.pop(field)
                # Excellent onset diagnostics must not rescue absent primary
                # confirmed-time evidence.
                candidate["diagnostic_onset_max_abs_hs_error_s"] = 0.0
                candidate["diagnostic_onset_max_abs_toe_off_error_s"] = 0.0
                candidate[
                    "diagnostic_onset_worst_event_normalized_max_abs_error"
                ] = 0.0

                result = subject.evaluate_holdout_gate(
                    candidate,
                    baseline,
                    protocol,
                )

                self.assertFalse(result["ok"], result)

    def test_protocol_validation_rejects_leakage_and_baseline_drift(self) -> None:
        mutations: list[tuple[str, Any]] = [
            (
                "validation leakage",
                lambda raw: raw["chronological_split"].__setitem__(
                    "selection_may_read_validation_metrics", True
                ),
            ),
            (
                "sealed leakage",
                lambda raw: raw["chronological_split"].__setitem__(
                    "selection_or_validation_may_read_sealed_metrics", True
                ),
            ),
            (
                "baseline value drift",
                lambda raw: raw["candidate_grid"]["baseline"].__setitem__(
                    "off_threshold_n", 1.0
                ),
            ),
            (
                "invalid threshold relation",
                lambda raw: raw["candidate_grid"].__setitem__(
                    "off_thresholds_n", [0.5, 1.0, 2.0, 5.0]
                ),
            ),
        ]
        for label, mutate in mutations:
            with self.subTest(label=label):
                raw = copy.deepcopy(self.protocol)
                mutate(raw)
                with self.assertRaises((subject.ProtocolError, ValueError)):
                    subject.load_and_validate_protocol(self._write_protocol(raw))

    def test_candidate_grid_is_cartesian_ordered_and_contains_baseline_once(
        self,
    ) -> None:
        candidates = subject.build_candidate_grid(self.protocol)
        expected_values = [
            (on, off)
            for on in (5.0, 8.0, 12.0, 16.0, 20.0)
            for off in (0.5, 1.0, 2.0, 3.0)
        ]
        actual_values = [
            (candidate.on_threshold_n, candidate.off_threshold_n)
            for candidate in candidates
        ]
        self.assertEqual(actual_values, expected_values)
        self.assertEqual(len({candidate.candidate_id for candidate in candidates}), 20)
        baseline = [
            candidate
            for candidate in candidates
            if candidate.candidate_id == "on005_off002"
        ]
        self.assertEqual(len(baseline), 1)
        self.assertEqual(
            (baseline[0].on_threshold_n, baseline[0].off_threshold_n),
            (5.0, 2.0),
        )

    def test_development_plateau_selects_baseline_without_reading_holdout(self) -> None:
        candidates = subject.build_candidate_grid(self.protocol)
        rows = [
            self._development_row(candidate.candidate_id)
            for candidate in reversed(candidates)
        ]
        # These poison fields must not participate in development selection.
        for index, row in enumerate(rows):
            row["validation_metric_poison"] = float(index)
            row["sealed_metric_poison"] = float(-index)

        selected = subject.select_development_candidate(rows, self.protocol)

        self.assertEqual(selected["candidate_id"], "on005_off002")

    def test_final_ranking_tie_is_lexicographic_and_input_order_independent(
        self,
    ) -> None:
        candidates = subject.build_candidate_grid(self.protocol)
        symmetric = [
            candidate
            for candidate in candidates
            if candidate.on_threshold_n == 5.0
            and candidate.off_threshold_n in {1.0, 3.0}
        ]
        self.assertEqual(len(symmetric), 2)
        expected_id = min(candidate.candidate_id for candidate in symmetric)
        rows = [
            self._development_row(candidate.candidate_id)
            for candidate in reversed(symmetric)
        ]

        selected = subject.select_development_candidate(rows, self.protocol)

        self.assertEqual(selected["candidate_id"], expected_id)

    def test_development_ranking_fails_closed_on_missing_or_nonfinite_metric(
        self,
    ) -> None:
        baseline = self._development_row("on005_off002")
        missing = dict(baseline)
        missing.pop("confirmed_fsm_stance_iou")
        nonfinite = dict(baseline)
        nonfinite["worst_event_normalized_max_abs_error"] = math.nan

        for label, row in (("missing", missing), ("nonfinite", nonfinite)):
            with self.subTest(label=label):
                with self.assertRaises((subject.ProtocolError, ValueError)):
                    subject.select_development_candidate([row], self.protocol)

        with self.assertRaises((subject.ProtocolError, ValueError)):
            subject.select_development_candidate(
                [self._development_row("on005_off002", eligible=False)],
                self.protocol,
            )

    def test_binary_metrics_report_exact_confusion_arithmetic(self) -> None:
        metrics = subject.binary_metrics(
            [True, True, False, False],
            [True, False, True, False],
        )

        self.assertEqual(metrics["samples"], 4)
        self.assertEqual(metrics["true_positive"], 1)
        self.assertEqual(metrics["false_positive"], 1)
        self.assertEqual(metrics["false_negative"], 1)
        self.assertEqual(metrics["true_negative"], 1)
        self.assertEqual(metrics["precision"], 0.5)
        self.assertEqual(metrics["recall"], 0.5)
        self.assertEqual(metrics["f1"], 0.5)
        self.assertAlmostEqual(metrics["iou"], 1.0 / 3.0)
        self.assertEqual(metrics["accuracy"], 0.5)

    def test_binary_metrics_zero_positive_convention_and_shape_guard(self) -> None:
        metrics = subject.binary_metrics([False, False], [False, False])
        self.assertEqual(metrics["precision"], 0.0)
        self.assertEqual(metrics["recall"], 0.0)
        self.assertEqual(metrics["f1"], 0.0)
        self.assertEqual(metrics["iou"], 0.0)
        self.assertEqual(metrics["accuracy"], 1.0)

        with self.assertRaises(ValueError):
            subject.binary_metrics([True, False], [True])

    def test_holdout_gate_accepts_every_inclusive_boundary(self) -> None:
        candidate = self._holdout_metrics("on008_off002")
        baseline = self._holdout_metrics("on005_off002")

        result = subject.evaluate_holdout_gate(candidate, baseline, self.protocol)

        self.assertTrue(result["ok"], result)

    def test_sensor_union_iou_is_diagnostic_not_a_holdout_gate(self) -> None:
        candidate = self._holdout_metrics("on008_off002")
        baseline = self._holdout_metrics("on005_off002")
        candidate["sensor_contact_stance_f1"] = 0.0
        candidate["sensor_contact_stance_iou"] = 0.0

        result = subject.evaluate_holdout_gate(candidate, baseline, self.protocol)

        self.assertTrue(result["ok"], result)

    def test_holdout_gate_rejects_absolute_and_baseline_relative_failures(
        self,
    ) -> None:
        baseline = self._holdout_metrics("on005_off002")
        cases: list[tuple[str, dict[str, Any]]] = []

        precision_failure = self._holdout_metrics("on008_off002")
        precision_failure["precision"] = 1.0 - 1.0e-12
        cases.append(("precision", precision_failure))

        iou_regression = self._holdout_metrics("on008_off002")
        baseline_for_iou = copy.deepcopy(baseline)
        baseline_for_iou["confirmed_fsm_stance_iou"] = 0.95
        iou_regression["confirmed_fsm_stance_iou"] = 0.95 - 0.010000001
        cases.append(("iou regression", iou_regression))

        timing_regression = self._holdout_metrics("on008_off002")
        timing_regression["worst_event_normalized_max_abs_error"] = (
            baseline["worst_event_normalized_max_abs_error"] + 1.0e-6
        )
        cases.append(("timing regression", timing_regression))

        for label, candidate in cases:
            with self.subTest(label=label):
                comparison_baseline = (
                    baseline_for_iou if label == "iou regression" else baseline
                )
                result = subject.evaluate_holdout_gate(
                    candidate,
                    comparison_baseline,
                    self.protocol,
                )
                self.assertFalse(result["ok"], result)

    def test_stage_manifest_requires_exact_lineage_and_candidate_scope(self) -> None:
        protocol_sha256 = "a" * 64
        prerequisite_sha256 = "b" * 64
        selected_id = "on008_off002"
        manifest = {
            "schema_version": 1,
            "stage": "validation",
            "protocol_id": self.protocol["protocol_id"],
            "protocol_sha256": protocol_sha256,
            "selected_candidate_id": selected_id,
            "candidate_ids": [selected_id, "on005_off002"],
            "prerequisite_stage": "development",
            "prerequisite_status": "PASS",
            "prerequisite_manifest_sha256": prerequisite_sha256,
        }

        subject.validate_stage_manifest(
            manifest,
            expected_stage="validation",
            protocol_sha256=protocol_sha256,
            selected_candidate_id=selected_id,
            prerequisite_sha256=prerequisite_sha256,
        )

        corruptions: list[tuple[str, Any]] = [
            (
                "protocol hash",
                lambda raw: raw.__setitem__("protocol_sha256", "c" * 64),
            ),
            (
                "selection",
                lambda raw: raw.__setitem__(
                    "selected_candidate_id", "on012_off002"
                ),
            ),
            (
                "extra candidate",
                lambda raw: raw["candidate_ids"].append("on012_off002"),
            ),
            (
                "prerequisite hash",
                lambda raw: raw.__setitem__(
                    "prerequisite_manifest_sha256", "d" * 64
                ),
            ),
            (
                "prerequisite status",
                lambda raw: raw.__setitem__("prerequisite_status", "FAIL"),
            ),
        ]
        for label, corrupt in corruptions:
            with self.subTest(label=label):
                altered = copy.deepcopy(manifest)
                corrupt(altered)
                with self.assertRaises((subject.ProtocolError, ValueError)):
                    subject.validate_stage_manifest(
                        altered,
                        expected_stage="validation",
                        protocol_sha256=protocol_sha256,
                        selected_candidate_id=selected_id,
                        prerequisite_sha256=prerequisite_sha256,
                    )

    def test_sealed_manifest_cannot_open_after_failed_validation(self) -> None:
        manifest = {
            "schema_version": 1,
            "stage": "sealed",
            "protocol_id": self.protocol["protocol_id"],
            "protocol_sha256": "a" * 64,
            "selected_candidate_id": "on008_off002",
            "candidate_ids": ["on008_off002", "on005_off002"],
            "prerequisite_stage": "validation",
            "prerequisite_status": "FAIL",
            "prerequisite_manifest_sha256": "b" * 64,
        }

        with self.assertRaises((subject.ProtocolError, ValueError)):
            subject.validate_stage_manifest(
                manifest,
                expected_stage="sealed",
                protocol_sha256="a" * 64,
                selected_candidate_id="on008_off002",
                prerequisite_sha256="b" * 64,
            )


if __name__ == "__main__":
    unittest.main()
