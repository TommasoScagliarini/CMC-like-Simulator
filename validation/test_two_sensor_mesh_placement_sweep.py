"""Pure/unit tests for the preregistered forefoot-placement harness."""

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


REPO_ROOT = Path(__file__).resolve().parents[1]
VALIDATION_DIR = REPO_ROOT / "validation"
PROTOCOL_PATH = VALIDATION_DIR / "two_sensor_mesh_placement_sweep_protocol.json"
if str(VALIDATION_DIR) not in sys.path:
    sys.path.insert(0, str(VALIDATION_DIR))


class _ForbiddenOpenSim(ModuleType):
    def __getattr__(self, name: str) -> Any:
        raise AssertionError(f"pure placement-sweep test touched opensim.{name}")


with patch.dict(sys.modules, {"opensim": _ForbiddenOpenSim("opensim")}):
    import sweep_two_sensor_mesh_placements_prescribed as subject  # noqa: E402


class TwoSensorMeshPlacementSweepTest(unittest.TestCase):
    def setUp(self) -> None:
        self.protocol = subject.load_and_validate_protocol(PROTOCOL_PATH)

    def _temporary_protocol(self, payload: dict[str, Any]) -> Path:
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        path = Path(temporary.name) / "protocol.json"
        path.write_text(json.dumps(payload), encoding="utf-8")
        return path

    @staticmethod
    def _row(candidate_id: str, *, selectable: bool = True) -> dict[str, Any]:
        return {
            "candidate_id": candidate_id,
            "selectable": selectable,
            "primary_event_time_field": "confirmed_time_s",
            "exact_reference_and_detector_event_counts": True,
            "reference_hs_count": 51,
            "reference_to_count": 50,
            "predicted_hs_count": 51,
            "predicted_to_count": 50,
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
            "worst_event_normalized_max_abs_error": 0.8,
            "mean_event_normalized_mean_abs_error": 0.4,
            "observed_valid_cycle_count": 0,
            "event_count_deficit": 0,
            "invalid_timeout_plus_unaccepted_count": 0,
            "maximum_heel_off_to_forefoot_gap_s": 0.0,
            "maximum_interior_both_off_gap_s": 0.0,
            "forefoot_missing_stance_count": 0,
            "swings_without_forefoot_off_count": 0,
            "forefoot_contact_crossing_next_hs_count": 0,
            "geometry_displacement_from_current_m": 0.05,
        }

    @staticmethod
    def _selectable_ids() -> list[str]:
        return [
            f"H{heel:02d}_F{fraction:02d}_P{depth:02d}"
            for heel in (0, 2)
            for fraction in (60, 70, 80)
            for depth in (25, 30)
        ]

    def test_protocol_freezes_final_cartesian_grid_and_two_cadences(self) -> None:
        grid = self.protocol["placement_grid"]
        self.assertEqual(grid["heel_vertical_offsets_below_current_mm"], [0, 2])
        self.assertEqual(
            grid["forefoot_longitudinal_fractions_mesh_x"], [0.6, 0.7, 0.8]
        )
        self.assertEqual(
            grid["forefoot_absolute_local_plantar_protrusion_mm"], [25, 30]
        )
        self.assertEqual(grid["cartesian_candidate_count"], 12)
        self.assertEqual(
            grid["development_prescreen"]["role"],
            "non_gating_grid_design_only",
        )
        replay = self.protocol["replay"]
        self.assertEqual(replay["primary_sample_dt_s"], 0.01)
        self.assertEqual(replay["winner_sensitivity_sample_dt_s"], 0.001)
        self.assertTrue(replay["primary_evaluate_all_candidates"])
        self.assertTrue(replay["sensitivity_evaluate_winner_only"])

    def test_protocol_rejects_old_height_rules_and_grid_drift(self) -> None:
        frozen = json.loads(PROTOCOL_PATH.read_text(encoding="utf-8"))
        mutations = (
            lambda raw: raw["placement_grid"].update(
                {"forefoot_height_rule": "same_plantar_protrusion_as_selected_heel"}
            ),
            lambda raw: raw["placement_grid"].update(
                {"forefoot_absolute_local_plantar_protrusion_mm": [20, 25]}
            ),
            lambda raw: raw["placement_grid"].update(
                {"forefoot_longitudinal_fractions_mesh_x": [0.5, 0.7, 0.8]}
            ),
            lambda raw: raw["placement_grid"].update(
                {"heel_vertical_offsets_below_current_mm": [0, 3]}
            ),
            lambda raw: raw["data_access"].update(
                {"allow_samples_at_or_after_100_s": True}
            ),
        )
        for mutate in mutations:
            payload = copy.deepcopy(frozen)
            mutate(payload)
            with self.subTest(payload=payload), self.assertRaises(subject.ProtocolError):
                subject.load_and_validate_protocol(
                    self._temporary_protocol(payload)
                )

    def test_mesh_derivation_builds_12_pairs_and_two_comparators(self) -> None:
        _base, candidates, summary = subject.build_placement_candidates(
            self.protocol
        )
        selectable = [item for item in candidates if item.selectable]
        self.assertEqual(len(selectable), 12)
        self.assertEqual(
            [item.candidate_id for item in selectable], self._selectable_ids()
        )
        self.assertEqual(len(candidates), 14)
        self.assertEqual(summary["selectable_candidate_count"], 12)
        self.assertTrue(summary["direct_unique_sphere_sampling"])
        self.assertFalse(summary["affine_reconstruction_used"])

        by_id = {item.candidate_id: item for item in selectable}
        expected = {
            "H00_F60_P25": (0.0612204224, -0.0363389490, 0.0002837110),
            "H00_F70_P30": (0.0899194553, -0.0456373060, 0.0010976430),
            "H02_F80_P30": (0.1186184883, -0.0460941840, 0.0033689850),
        }
        for candidate_id, location in expected.items():
            for observed, wanted in zip(
                by_id[candidate_id].forefoot_location, location
            ):
                self.assertAlmostEqual(observed, wanted, places=8)
        self.assertAlmostEqual(
            by_id["H02_F80_P30"].heel_location[1], -0.0348195377, places=10
        )

    def test_single_pass_bundle_deduplicates_geometry_and_keeps_two_sensors(self) -> None:
        base, candidates, _summary = subject.build_placement_candidates(
            self.protocol
        )
        sampler, pairs, profiles = subject._sampling_bundle(base, candidates)
        # 2 heel placements + 6 grid forefeet + current toe + rejected-V4 toe.
        self.assertEqual(len(sampler.spheres), 10)
        self.assertEqual(set(pairs), {item.candidate_id for item in candidates})
        for candidate in candidates:
            self.assertEqual(len(profiles[candidate.candidate_id].spheres), 2)
            self.assertEqual(set(pairs[candidate.candidate_id]), {"heel", "toe"})

    def test_strict_winner_is_selected_only_from_v3_eligible_rows(self) -> None:
        rows = [self._row(candidate_id) for candidate_id in self._selectable_ids()]
        rows.append(self._row(subject.CURRENT_PROFILE_ID, selectable=False))
        rows.append(self._row(subject.V4_PROFILE_ID, selectable=False))
        for row in rows:
            if row["candidate_id"] == "H02_F80_P30":
                row["observed_valid_cycle_count"] = 50

        def fake_gate(row, _current, _protocol):
            return {"ok": row["candidate_id"] == "H02_F80_P30", "checks": {}}

        with patch.object(subject, "evaluate_holdout_gate", side_effect=fake_gate):
            selected_id, mode, selection = subject.select_development_outcome(
                rows, self.protocol
            )
        self.assertEqual(selected_id, "H02_F80_P30")
        self.assertEqual(mode, "strict_winner")
        self.assertEqual(selection["strict_winner_id"], selected_id)
        self.assertIsNone(selection["diagnostic_best_id"])

    def test_no_strict_candidate_locks_diagnostic_best_without_promotion(self) -> None:
        rows = [self._row(candidate_id) for candidate_id in self._selectable_ids()]
        rows.append(self._row(subject.CURRENT_PROFILE_ID, selectable=False))
        rows.append(self._row(subject.V4_PROFILE_ID, selectable=False))
        for row in rows:
            if row["candidate_id"] == "H02_F80_P30":
                row["observed_valid_cycle_count"] = 1
        with patch.object(
            subject, "evaluate_holdout_gate", return_value={"ok": False, "checks": {}}
        ):
            selected_id, mode, selection = subject.select_development_outcome(
                rows, self.protocol
            )
        self.assertEqual(selected_id, "H02_F80_P30")
        self.assertEqual(mode, "diagnostic_best")
        self.assertIsNone(selection["strict_winner_id"])
        self.assertFalse(selection["diagnostic_best_is_promotable"])

    def test_diagnostic_1ms_cannot_convert_primary_failure_to_pass(self) -> None:
        selected = self._row("H02_F80_P30")
        selected["observed_valid_cycle_count"] = 50
        current = self._row(subject.CURRENT_PROFILE_ID, selectable=False)
        with patch.object(
            subject, "evaluate_holdout_gate", return_value={"ok": True, "checks": {}}
        ):
            diagnostic = subject.build_sensitivity_assessment(
                "diagnostic_best", selected, current, self.protocol
            )
            strict = subject.build_sensitivity_assessment(
                "strict_winner", selected, current, self.protocol
            )
        self.assertFalse(diagnostic["ok"])
        self.assertFalse(diagnostic["can_convert_10ms_fail_to_pass"])
        self.assertTrue(strict["ok"])

    def test_physical_pre_gate_exposes_causal_continuity_checks(self) -> None:
        row = self._row("H02_F80_P30")
        row["observed_valid_cycle_count"] = 50
        passed = subject.physical_continuity_pre_gate(row, self.protocol)
        self.assertTrue(passed["ok"])
        self.assertEqual(
            set(passed["checks"]),
            {
                "heel_off_to_forefoot_gap",
                "interior_both_off_gap",
                "forefoot_present_every_stance",
                "forefoot_off_observed_every_swing",
                "forefoot_off_before_next_hs",
                "exact_event_counts_order_and_cycles",
                "no_invalid_or_timeout",
                "no_unaccepted_sensor_events",
            },
        )
        row["maximum_heel_off_to_forefoot_gap_s"] = 0.011
        failed = subject.physical_continuity_pre_gate(row, self.protocol)
        self.assertFalse(failed["ok"])
        self.assertFalse(failed["checks"]["heel_off_to_forefoot_gap"])

    def test_current_and_rejected_v4_are_nonselectable_comparators(self) -> None:
        self.assertEqual(
            self.protocol["comparators"],
            {subject.CURRENT_PROFILE_ID: True, subject.V4_PROFILE_ID: True},
        )
        self.assertEqual(
            self.protocol["selection"]["diagnostic_best_ranking"],
            subject.DIAGNOSTIC_BEST_RANKING,
        )


if __name__ == "__main__":
    unittest.main()
