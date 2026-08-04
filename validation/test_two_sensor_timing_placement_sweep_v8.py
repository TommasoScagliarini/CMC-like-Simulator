"""Pure tests for the V8 heel-focused two-sensor sweep."""

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
VALIDATION_ROOT = REPO_ROOT / "validation"
PROTOCOL_PATH = VALIDATION_ROOT / "two_sensor_timing_placement_sweep_protocol_v8.json"
if str(VALIDATION_ROOT) not in sys.path:
    sys.path.insert(0, str(VALIDATION_ROOT))


class _ForbiddenOpenSim(ModuleType):
    def __getattr__(self, name: str) -> Any:
        raise AssertionError(f"pure V8 test touched opensim.{name}")


with patch.dict(sys.modules, {"opensim": _ForbiddenOpenSim("opensim")}):
    import sweep_two_sensor_timing_placements_prescribed_v8 as subject  # noqa: E402


class TwoSensorTimingPlacementV8Test(unittest.TestCase):
    def setUp(self) -> None:
        self.protocol = subject.load_and_validate_protocol(PROTOCOL_PATH)

    def _temporary_protocol(self, payload: dict[str, Any]) -> Path:
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        path = Path(temporary.name) / "protocol.json"
        path.write_text(json.dumps(payload), encoding="utf-8")
        return path

    def test_protocol_freezes_heel_micro_two_sensor_grid(self) -> None:
        grid = self.protocol["placement_grid"]
        self.assertEqual(grid["heel_vertical_offsets_below_current_mm"], [2.0, 2.25])
        self.assertEqual(grid["heel_anterior_x_deltas_mm"], [2.5, 2.75])
        self.assertEqual(grid["forefoot_longitudinal_fractions_mesh_x"], [0.795, 0.8])
        self.assertEqual(grid["forefoot_absolute_local_plantar_protrusion_mm"], [35.0])
        self.assertEqual(grid["selectable_candidate_count"], 8)
        self.assertEqual(grid["sensors_per_pair"], 2)
        self.assertEqual(
            self.protocol["runtime_gate_10ms"], self.protocol["fine_gate_1ms"]
        )

    def test_protocol_rejects_grid_and_source_drift(self) -> None:
        frozen = json.loads(PROTOCOL_PATH.read_text(encoding="utf-8"))
        changed = copy.deepcopy(frozen)
        changed["placement_grid"]["heel_vertical_offsets_below_current_mm"][1] = 2.5
        with self.assertRaises(subject.ProtocolError):
            subject.load_and_validate_protocol(self._temporary_protocol(changed))
        drifted = copy.deepcopy(frozen)
        drifted["sources"]["v7_manifest"]["sha256"] = "0" * 64
        with self.assertRaises(subject.ProtocolError):
            subject.load_and_validate_protocol(self._temporary_protocol(drifted))

    def test_builder_has_eight_pairs_two_comparators_and_nine_unique_spheres(self) -> None:
        base, candidates, summary = subject.build_placement_candidates(self.protocol)
        selectable = [item for item in candidates if item.selectable]
        self.assertEqual(len(selectable), 8)
        self.assertEqual(len(candidates), 10)
        self.assertEqual(summary["sensors_per_pair"], 2)
        sampler, _pairs, profiles = subject.v6.v1._sampling_bundle(base, candidates)
        self.assertEqual(len(sampler.spheres), 9)
        self.assertTrue(all(len(profile.spheres) == 2 for profile in profiles.values()))
        self.assertEqual(
            {item.candidate_id for item in candidates if not item.selectable},
            {subject.v6.V5_COMPARATOR_ID, subject.v6.CURRENT_COMPARATOR_ID},
        )

    def test_selection_requires_same_candidate_to_pass_both_resolutions(self) -> None:
        ids = [
            subject._candidate_id(offset, x_mm, fraction)
            for offset in subject.HEEL_OFFSETS_MM
            for x_mm in subject.HEEL_X_DELTAS_MM
            for fraction in subject.FOREFOOT_FRACTIONS
        ]
        all_ids = ids + [subject.v6.V5_COMPARATOR_ID, subject.v6.CURRENT_COMPARATOR_ID]

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
            ok = not (row["candidate_id"] == ids[0] and gate_key == "fine_gate_1ms")
            return {"ok": ok, "checks": {}}

        with patch.object(subject.v6, "strict_gate", side_effect=gate):
            winner, decision = subject.select_multiresolution_winner(
                rows(), rows(), self.protocol
            )
        self.assertNotEqual(winner, ids[0])
        self.assertEqual(calls, 16)
        self.assertFalse(decision["candidate_gates"][ids[0]]["passes_both"])

    def test_no_clobber_is_inherited(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            output = root / "output"
            plot_dir = root / "plot"
            output.mkdir()
            (output / "owned.txt").write_text("owned", encoding="utf-8")
            with self.assertRaises(subject.v6.NoClobberError):
                subject.v6._preflight_no_clobber(output, plot_dir)
            self.assertFalse(plot_dir.exists())


if __name__ == "__main__":
    unittest.main()
