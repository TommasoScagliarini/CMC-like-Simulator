"""Pure contract tests for the two-sensor timing-placement V3 harness."""

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
PROTOCOL_PATH = VALIDATION_ROOT / "two_sensor_timing_placement_sweep_protocol_v3.json"
if str(VALIDATION_ROOT) not in sys.path:
    sys.path.insert(0, str(VALIDATION_ROOT))


class _ForbiddenOpenSim(ModuleType):
    def __getattr__(self, name: str) -> Any:
        raise AssertionError(f"pure V3 test touched opensim.{name}")


with patch.dict(sys.modules, {"opensim": _ForbiddenOpenSim("opensim")}):
    import sweep_two_sensor_timing_placements_prescribed_v3 as subject  # noqa: E402


class TwoSensorTimingPlacementV3Test(unittest.TestCase):
    def setUp(self) -> None:
        self.protocol = subject.load_and_validate_protocol(PROTOCOL_PATH)

    def _temporary_protocol(self, payload: dict[str, Any]) -> Path:
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        path = Path(temporary.name) / "protocol.json"
        path.write_text(json.dumps(payload), encoding="utf-8")
        return path

    def test_protocol_freezes_simple_detector_and_primary_load_routing(self) -> None:
        grid = self.protocol["placement_grid"]
        self.assertEqual(grid["sensors_per_pair"], 2)
        self.assertEqual(grid["selectable_candidate_count"], 10)
        self.assertEqual(
            self.protocol["evidence_routing"]["heel_toe_event_guards"],
            "candidate_two_sensor_forces_only",
        )
        self.assertFalse(
            self.protocol["evidence_routing"]["detector_spheres_generate_grf"]
        )
        decision = self.protocol["decision_contract"]
        self.assertFalse(decision["profile_promotion_allowed"])
        self.assertFalse(decision["sealed_validation_allowed"])
        self.assertFalse(decision["training_allowed"])

    def test_protocol_rejects_grid_and_source_drift(self) -> None:
        frozen = json.loads(PROTOCOL_PATH.read_text(encoding="utf-8"))
        grid_drift = copy.deepcopy(frozen)
        grid_drift["placement_grid"][
            "forefoot_absolute_local_plantar_protrusion_mm"
        ][-1] = 33.0
        with self.assertRaises(subject.ProtocolError):
            subject.load_and_validate_protocol(
                self._temporary_protocol(grid_drift)
            )
        hash_drift = copy.deepcopy(frozen)
        hash_drift["sources"]["production_fsm"]["sha256"] = "0" * 64
        with self.assertRaises(subject.ProtocolError):
            subject.load_and_validate_protocol(
                self._temporary_protocol(hash_drift)
            )

    def test_builder_has_ten_selectable_pairs_plus_current(self) -> None:
        base, candidates, summary = subject.build_placement_candidates(
            self.protocol
        )
        selectable = [candidate for candidate in candidates if candidate.selectable]
        self.assertEqual(len(selectable), 10)
        self.assertEqual(len(candidates), 11)
        self.assertEqual(
            [candidate.heel_offset_below_current_mm for candidate in selectable],
            [1.0] * 5 + [2.0] * 5,
        )
        self.assertEqual(
            sorted({candidate.forefoot_protrusion_mm for candidate in selectable}),
            list(subject.PROTRUSIONS_MM),
        )
        self.assertTrue(all(candidate.geometry["pre_gate_ok"] for candidate in selectable))
        self.assertEqual(summary["sensors_per_pair"], 2)
        sampler, pairs, profiles = subject.v1._sampling_bundle(base, candidates)
        self.assertEqual(len(sampler.spheres), 9)
        self.assertEqual(len(pairs), 11)
        self.assertTrue(all(len(profile.spheres) == 2 for profile in profiles.values()))

    def test_primary_branch_changes_only_fsm_load_contact_evidence(self) -> None:
        common = {
            "setup": object(),
            "times": np.arange(3, dtype=float),
            "kinematics": {"knee_rad": np.zeros(3), "ankle_rad": np.zeros(3)},
            "prescribed_vertical_n": np.zeros(3),
            "reference_events": {
                "heel_strike": np.asarray([0.0, 2.0]),
                "toe_off": np.asarray([1.0]),
            },
            "body_weight_n": 500.0,
            "detector_loads": {
                "left_heel": np.asarray([1.0, 0.0, 0.0]),
                "left_toe": np.asarray([0.0, 1.0, 0.0]),
            },
            "detector_penetrations": {
                "left_heel": np.asarray([1.0, 0.0, 0.0]),
                "left_toe": np.asarray([0.0, 1.0, 0.0]),
            },
            "detector_aggregate": np.asarray([1.0, 1.0, 0.0]),
            "primary_aggregate": np.asarray([100.0, 200.0, 0.0]),
            "primary_penetration": np.asarray([1.0, 1.0, 0.0]),
        }
        inputs, sources = subject.dual.compose_branch_inputs(
            common, "B_primary_load"
        )
        self.assertIs(inputs["loads"]["left_heel"], common["detector_loads"]["left_heel"])
        np.testing.assert_array_equal(inputs["aggregate"], common["primary_aggregate"])
        self.assertEqual(
            sources["normal_force_bw"], "primary_online_grf_left_aggregate"
        )

    def test_transfer_gate_detects_one_both_off_sample(self) -> None:
        times = np.arange(0.0, 0.7, 0.1)
        events = {
            "heel_strike": np.asarray([0.0, 0.7]),
            "toe_off": np.asarray([0.6]),
        }
        heel = np.asarray([1, 1, 1, 0, 0, 0, 0], dtype=float)
        toe = np.asarray([0, 0, 0, 0, 1, 1, 0], dtype=float)
        result = subject.heel_to_forefoot_transfer_diagnostics(
            times, heel, toe, events
        )
        self.assertEqual(result["incomplete_transfer_cycle_count"], 0)
        self.assertEqual(result["both_latches_off_sample_count"], 1)
        toe[3] = 1.0
        result = subject.heel_to_forefoot_transfer_diagnostics(
            times, heel, toe, events
        )
        self.assertEqual(result["both_latches_off_sample_count"], 0)

    def test_early_to_gate_and_no_clobber(self) -> None:
        result = subject.early_to_candidate_diagnostics(
            [
                {"event": "toe_off", "time": 1.0, "contact_duration_s": 0.29},
                {"event": "toe_off", "time": 2.0, "contact_duration_s": 0.30},
                {"event": "heel_strike", "time": 3.0},
            ],
            minimum_stance_duration_s=0.30,
        )
        self.assertEqual(result["early_toe_off_candidate_count"], 1)
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            output = root / "output"
            plot = root / "plot"
            output.mkdir()
            (output / "occupied.txt").write_text("owned", encoding="utf-8")
            with self.assertRaises(subject.NoClobberError):
                subject._preflight_no_clobber(output, plot)
            self.assertFalse(plot.exists())


if __name__ == "__main__":
    unittest.main()
