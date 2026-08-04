"""Pure tests for the preregistered two-sensor V6 sweep."""

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
VALIDATION_ROOT = REPO_ROOT / "validation"
PROTOCOL_PATH = VALIDATION_ROOT / "two_sensor_timing_placement_sweep_protocol_v6.json"
if str(VALIDATION_ROOT) not in sys.path:
    sys.path.insert(0, str(VALIDATION_ROOT))


class _ForbiddenOpenSim(ModuleType):
    def __getattr__(self, name: str) -> Any:
        raise AssertionError(f"pure V6 test touched opensim.{name}")


with patch.dict(sys.modules, {"opensim": _ForbiddenOpenSim("opensim")}):
    import sweep_two_sensor_timing_placements_prescribed_v6 as subject  # noqa: E402


class TwoSensorTimingPlacementV6Test(unittest.TestCase):
    def setUp(self) -> None:
        self.protocol = subject.load_and_validate_protocol(PROTOCOL_PATH)

    def _temporary_protocol(self, payload: dict[str, Any]) -> Path:
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        path = Path(temporary.name) / "protocol.json"
        path.write_text(json.dumps(payload), encoding="utf-8")
        return path

    def test_protocol_freezes_simple_two_sensor_multiresolution_contract(self) -> None:
        grid = self.protocol["placement_grid"]
        self.assertEqual(grid["selectable_candidate_count"], 8)
        self.assertEqual(grid["total_pair_count"], 10)
        self.assertEqual(grid["sensors_per_pair"], 2)
        self.assertEqual(grid["heel_anterior_x_deltas_mm"], [2.5, 3.0])
        self.assertEqual(grid["forefoot_longitudinal_fractions_mesh_x"], [0.76, 0.78])
        self.assertTrue(
            self.protocol["replay"]["evaluate_all_pairs_at_both_resolutions"]
        )
        self.assertFalse(
            self.protocol["decision_contract"]["profile_promotion_allowed"]
        )
        self.assertEqual(
            self.protocol["recontact_diagnostic"]["role"],
            "diagnostic_only_not_detector_gate",
        )

    def test_protocol_rejects_grid_gate_and_hash_drift(self) -> None:
        frozen = json.loads(PROTOCOL_PATH.read_text(encoding="utf-8"))
        grid_drift = copy.deepcopy(frozen)
        grid_drift["placement_grid"]["heel_anterior_x_deltas_mm"][0] = 2.25
        with self.assertRaises(subject.ProtocolError):
            subject.load_and_validate_protocol(self._temporary_protocol(grid_drift))
        gate_drift = copy.deepcopy(frozen)
        gate_drift["fine_gate_1ms"][
            "minimum_causal_toe_clear_before_next_hs_onset_s"
        ] = 0.02
        with self.assertRaises(subject.ProtocolError):
            subject.load_and_validate_protocol(self._temporary_protocol(gate_drift))
        hash_drift = copy.deepcopy(frozen)
        hash_drift["sources"]["production_fsm"]["sha256"] = "0" * 64
        with self.assertRaises(subject.ProtocolError):
            subject.load_and_validate_protocol(self._temporary_protocol(hash_drift))

    def test_builder_has_eight_candidates_two_comparators_and_two_spheres(self) -> None:
        base, candidates, summary = subject.build_placement_candidates(self.protocol)
        selectable = [item for item in candidates if item.selectable]
        self.assertEqual(len(selectable), 8)
        self.assertEqual(len(candidates), 10)
        self.assertEqual(summary["sensors_per_pair"], 2)
        self.assertTrue(
            summary["shared_spheres_are_offline_sampling_optimization_only"]
        )
        sampler, _pairs, profiles = subject.v1._sampling_bundle(base, candidates)
        self.assertEqual(len(sampler.spheres), 10)
        self.assertTrue(all(len(profile.spheres) == 2 for profile in profiles.values()))
        self.assertEqual(
            {item.candidate_id for item in candidates if not item.selectable},
            {subject.V5_COMPARATOR_ID, subject.CURRENT_COMPARATOR_ID},
        )

    def test_causal_clearance_uses_next_accepted_hs_onset(self) -> None:
        times = np.arange(1.0, 2.01, 0.01)
        toe_contact = np.zeros(len(times), dtype=float)
        toe_contact[50:55] = 1.0
        replay = {
            "toe_contact": toe_contact,
            "accepted": [
                {
                    "event": "toe_off",
                    "confirmed_time_s": 1.20,
                    "event_time_s": 1.17,
                    "segment_valid": 1.0,
                },
                {
                    "event": "heel_strike",
                    "confirmed_time_s": 1.93,
                    "event_time_s": 1.90,
                    "segment_valid": 1.0,
                },
            ],
        }
        with patch.object(subject, "_replay_for_common", return_value=(times, replay)):
            result = subject.causal_swing_clearance_diagnostics(
                {}, sample_dt_s=0.01
            )
        self.assertEqual(result["interval_count"], 1)
        self.assertAlmostEqual(result["minimum_causal_clear_s"], 0.35)
        self.assertEqual(result["toe_latch_recontact_episode_count"], 1)
        self.assertEqual(result["recontact_role"], "diagnostic_only_not_detector_gate")

    def test_causal_gate_requires_thirty_ms_but_does_not_gate_recontact_count(self) -> None:
        row = {
            "causal_swing_interval_count": 50,
            "minimum_causal_toe_clear_before_next_hs_onset_s": 0.031,
            "toe_latch_recontact_episode_count": 7,
        }
        with patch.object(
            subject.v4,
            "strict_gate",
            return_value={"ok": True, "checks": {"inherited": True}},
        ):
            passed = subject.strict_gate(
                row,
                {},
                self.protocol,
                gate_key="fine_gate_1ms",
            )
            self.assertTrue(passed["ok"])
            self.assertTrue(passed["recontact_is_diagnostic_only"])
            failed_row = dict(row)
            failed_row["minimum_causal_toe_clear_before_next_hs_onset_s"] = 0.029
            failed = subject.strict_gate(
                failed_row,
                {},
                self.protocol,
                gate_key="fine_gate_1ms",
            )
            self.assertFalse(failed["ok"])

    def test_selection_requires_both_resolutions(self) -> None:
        ids = [
            subject._candidate_id(x, fraction, depth)
            for x in subject.HEEL_X_DELTAS_MM
            for fraction in subject.FOREFOOT_FRACTIONS
            for depth in subject.PROTRUSIONS_MM
        ]
        all_ids = ids + [subject.V5_COMPARATOR_ID, subject.CURRENT_COMPARATOR_ID]

        def rows() -> list[dict[str, Any]]:
            return [
                {
                    "candidate_id": candidate_id,
                    "selectable": candidate_id in ids,
                    "worst_event_normalized_max_abs_error": 0.8,
                    "mean_event_normalized_mean_abs_error": 0.4,
                    "confirmed_fsm_stance_iou": 0.93,
                    "geometry_displacement_from_current_m": float(index),
                }
                for index, candidate_id in enumerate(all_ids)
            ]

        calls = 0

        def gate(
            row: dict[str, Any],
            _current: dict[str, Any],
            _protocol: dict[str, Any],
            *,
            gate_key: str,
        ) -> dict[str, Any]:
            nonlocal calls
            calls += 1
            ok = not (
                row["candidate_id"] == ids[0] and gate_key == "fine_gate_1ms"
            )
            return {"ok": ok, "checks": {}}

        with patch.object(subject, "strict_gate", side_effect=gate):
            winner, decision = subject.select_multiresolution_winner(
                rows(), rows(), self.protocol
            )
        self.assertNotEqual(winner, ids[0])
        self.assertEqual(calls, 16)
        self.assertFalse(decision["candidate_gates"][ids[0]]["passes_both"])

    def test_no_clobber_keeps_destinations_untouched(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            output = root / "output"
            plot_dir = root / "plot"
            output.mkdir()
            (output / "owned.txt").write_text("owned", encoding="utf-8")
            with self.assertRaises(subject.NoClobberError):
                subject._preflight_no_clobber(output, plot_dir)
            self.assertFalse(plot_dir.exists())


if __name__ == "__main__":
    unittest.main()
