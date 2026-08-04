"""Pure tests for the V2 two-sensor depth micro-sweep."""

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
PROTOCOL_PATH = VALIDATION_DIR / "two_sensor_mesh_placement_sweep_protocol_v2.json"
if str(VALIDATION_DIR) not in sys.path:
    sys.path.insert(0, str(VALIDATION_DIR))


class _ForbiddenOpenSim(ModuleType):
    def __getattr__(self, name: str) -> Any:
        raise AssertionError(f"pure V2 test touched opensim.{name}")


with patch.dict(sys.modules, {"opensim": _ForbiddenOpenSim("opensim")}):
    import sweep_two_sensor_mesh_placements_prescribed_v2 as subject  # noqa: E402


class TwoSensorMeshPlacementSweepV2Test(unittest.TestCase):
    def setUp(self) -> None:
        self.protocol = subject.load_and_validate_protocol(PROTOCOL_PATH)

    def _temporary_protocol(self, payload: dict[str, Any]) -> Path:
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        path = Path(temporary.name) / "protocol.json"
        path.write_text(json.dumps(payload), encoding="utf-8")
        return path

    @staticmethod
    def _row(depth: float, *, gap_s: float = 0.0) -> dict[str, Any]:
        candidate_id = subject._candidate_id(depth)
        return {
            "candidate_id": candidate_id,
            "selectable": True,
            "forefoot_protrusion_mm": float(depth),
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
            "worst_event_normalized_max_abs_error": 0.8,
            "mean_event_normalized_mean_abs_error": 0.4,
            "observed_valid_cycle_count": 50,
            "reference_to_count": 50,
            "maximum_heel_off_to_forefoot_gap_s": float(gap_s),
            "maximum_interior_both_off_gap_s": float(gap_s),
            "forefoot_missing_stance_count": 0,
            "swings_without_forefoot_off_count": 0,
            "forefoot_contact_crossing_next_hs_count": 0,
        }

    @staticmethod
    def _current_row() -> dict[str, Any]:
        return {
            "candidate_id": subject.v1.CURRENT_PROFILE_ID,
            "selectable": False,
            "primary_event_time_field": "confirmed_time_s",
            "confirmed_fsm_stance_iou": 0.90,
            "worst_event_normalized_max_abs_error": 1.0,
        }

    @staticmethod
    def _v4_row() -> dict[str, Any]:
        return {
            "candidate_id": subject.v1.V4_PROFILE_ID,
            "selectable": False,
        }

    def _selection_rows(self, gaps: list[float]) -> list[dict[str, Any]]:
        rows = [
            self._row(depth, gap_s=gap)
            for depth, gap in zip(subject.PROTRUSIONS_MM, gaps)
        ]
        rows.extend([self._current_row(), self._v4_row()])
        return rows

    def test_protocol_freezes_one_dimensional_five_pair_grid(self) -> None:
        grid = self.protocol["placement_grid"]
        self.assertEqual(grid["heel_vertical_offsets_below_current_mm"], [2.0])
        self.assertEqual(grid["forefoot_longitudinal_fractions_mesh_x"], [0.7])
        self.assertEqual(
            grid["forefoot_absolute_local_plantar_protrusion_mm"],
            [30.0, 30.5, 31.0, 31.5, 32.0],
        )
        self.assertEqual(grid["candidate_count"], 5)
        self.assertEqual(grid["sensors_per_candidate"], 2)
        self.assertEqual(
            self.protocol["selection"]["winner_rule"],
            "shallowest_full_pass_at_10ms",
        )

    def test_protocol_rejects_profile_path_and_grid_drift(self) -> None:
        frozen = json.loads(PROTOCOL_PATH.read_text(encoding="utf-8"))
        mutations = (
            lambda raw: raw["profile_paths"].update(
                {subject.v1.CURRENT_PROFILE_ID: raw["profile_paths"][subject.v1.V4_PROFILE_ID]}
            ),
            lambda raw: raw["placement_grid"].update(
                {"forefoot_absolute_local_plantar_protrusion_mm": [30, 31, 32]}
            ),
            lambda raw: raw["placement_grid"].update(
                {"heel_vertical_offsets_below_current_mm": [3.0]}
            ),
            lambda raw: raw["data_access"].update(
                {"allow_samples_at_or_after_100_s": True}
            ),
        )
        for mutate in mutations:
            payload = copy.deepcopy(frozen)
            mutate(payload)
            with self.subTest(payload=payload), self.assertRaises(subject.ProtocolError):
                subject.load_and_validate_protocol(self._temporary_protocol(payload))

    def test_builder_derives_five_pairs_plus_two_comparators(self) -> None:
        base, candidates, summary = subject.build_placement_candidates(self.protocol)
        selectable = [candidate for candidate in candidates if candidate.selectable]
        self.assertEqual(len(selectable), 5)
        self.assertEqual(len(candidates), 7)
        self.assertEqual(
            [candidate.forefoot_protrusion_mm for candidate in selectable],
            list(subject.PROTRUSIONS_MM),
        )
        self.assertEqual(
            [candidate.candidate_id for candidate in selectable],
            [subject._candidate_id(depth) for depth in subject.PROTRUSIONS_MM],
        )
        self.assertTrue(summary["all_absolute_bottom_offsets_within_20mm"])
        self.assertLess(
            max(abs(value) for value in summary["signed_forefoot_minus_heel_bottom_offsets_mm"]),
            20.0,
        )
        sampler, pairs, profiles = subject.v1._sampling_bundle(base, candidates)
        self.assertEqual(len(sampler.spheres), 9)
        self.assertEqual(set(pairs), {candidate.candidate_id for candidate in candidates})
        for candidate in candidates:
            self.assertEqual(len(profiles[candidate.candidate_id].spheres), 2)

    def test_shallowest_full_pass_wins(self) -> None:
        rows = self._selection_rows([0.02, 0.01, 0.0, 0.0, 0.0])
        with patch.object(
            subject.v1,
            "evaluate_holdout_gate",
            return_value={"ok": True, "checks": {}},
        ):
            selected_id, selection = subject.select_development_outcome(
                rows, self.protocol
            )
        self.assertEqual(selected_id, subject._candidate_id(30.5))
        self.assertEqual(selection["strict_winner_id"], selected_id)
        self.assertIsNone(selection["diagnostic_winner_id"])

    def test_no_full_pass_produces_no_diagnostic_winner_or_1ms_candidate(self) -> None:
        rows = self._selection_rows([0.02] * 5)
        with patch.object(
            subject.v1,
            "evaluate_holdout_gate",
            return_value={"ok": True, "checks": {}},
        ):
            selected_id, selection = subject.select_development_outcome(
                rows, self.protocol
            )
        self.assertIsNone(selected_id)
        self.assertIsNone(selection["diagnostic_winner_id"])
        self.assertEqual(
            selection["status"],
            "NO_STRICT_WINNER_DEPTH_MICRO_ADJUSTMENT_FAILED",
        )

    def test_release_failure_censors_same_and_deeper_depths(self) -> None:
        rows = self._selection_rows([0.02, 0.0, 0.0, 0.0, 0.0])
        rows[1]["forefoot_contact_crossing_next_hs_count"] = 1
        with patch.object(
            subject.v1,
            "evaluate_holdout_gate",
            return_value={"ok": True, "checks": {}},
        ):
            selected_id, selection = subject.select_development_outcome(
                rows, self.protocol
            )
        self.assertIsNone(selected_id)
        self.assertEqual(selection["release_cutoff_depth_mm"], 30.5)
        gates = selection["candidate_gates"]
        self.assertTrue(gates[0]["allowed_by_release_cutoff"])
        self.assertTrue(all(not gate["allowed_by_release_cutoff"] for gate in gates[1:]))

    def test_gap_monotonicity_is_diagnostic_only(self) -> None:
        rows = self._selection_rows([0.02, 0.01, 0.01, 0.0, 0.0])
        result = subject._gap_monotonicity(rows[:5])
        self.assertTrue(result["monotonic_nonincreasing"])
        self.assertTrue(result["any_candidate_at_or_below_10ms"])
        self.assertEqual(result["role"], "diagnostic_not_acceptance_gate")

    def test_one_sample_gap_accepts_only_float_roundoff(self) -> None:
        roundoff = self._row(30.5, gap_s=0.010000000000005)
        too_large = self._row(30.5, gap_s=0.0100001)
        self.assertTrue(
            subject.physical_continuity_pre_gate(roundoff, self.protocol)["ok"]
        )
        self.assertFalse(
            subject.physical_continuity_pre_gate(too_large, self.protocol)["ok"]
        )

    def test_no_clobber_preflight_does_not_mutate_occupied_output(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            output_dir = root / "occupied"
            output_dir.mkdir()
            sentinel = output_dir / "prior.txt"
            sentinel.write_text("preserve", encoding="utf-8")
            plot_dir = root / "new_plot"
            before = sorted(path.name for path in output_dir.iterdir())
            exit_code = subject.main(
                [
                    "--protocol",
                    str(PROTOCOL_PATH),
                    "--output-dir",
                    str(output_dir),
                    "--plot-dir",
                    str(plot_dir),
                ]
            )
            after = sorted(path.name for path in output_dir.iterdir())
            self.assertEqual(exit_code, 2)
            self.assertEqual(before, after)
            self.assertEqual(sentinel.read_text(encoding="utf-8"), "preserve")
            self.assertFalse(plot_dir.exists())

    def test_v1_artifacts_remain_byte_identical(self) -> None:
        expected = {
            "sweep_two_sensor_mesh_placements_prescribed.py": (
                "929e0bbdebaf1f9a9cbe2d021ff6610a22fb2be02f3a6438bb184c3a1ad0a39a"
            ),
            "two_sensor_mesh_placement_sweep_protocol.json": (
                "e33b6d59e9c6d87937847bf56fe9751893d0f1dd2a18152d1e295fa3a3ba4921"
            ),
            "test_two_sensor_mesh_placement_sweep.py": (
                "0d13846b0baebf1942b1f22c2bbfa5c3ddc3f0f3bcfe650119531eab74bcd76f"
            ),
        }
        for filename, digest in expected.items():
            with self.subTest(filename=filename):
                self.assertEqual(subject.v1._sha256(VALIDATION_DIR / filename), digest)


if __name__ == "__main__":
    unittest.main()
