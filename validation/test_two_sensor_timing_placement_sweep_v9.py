"""Pure tests for the final local V9 two-sensor placement sweep."""

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
PROTOCOL_PATH = VALIDATION_ROOT / "two_sensor_timing_placement_sweep_protocol_v9.json"
if str(VALIDATION_ROOT) not in sys.path:
    sys.path.insert(0, str(VALIDATION_ROOT))


class _ForbiddenOpenSim(ModuleType):
    def __getattr__(self, name: str) -> Any:
        raise AssertionError(f"pure V9 test touched opensim.{name}")


with patch.dict(sys.modules, {"opensim": _ForbiddenOpenSim("opensim")}):
    import sweep_two_sensor_timing_placements_prescribed_v9 as subject  # noqa: E402


class TwoSensorTimingPlacementV9Test(unittest.TestCase):
    def setUp(self) -> None:
        self.protocol = subject.load_and_validate_protocol(PROTOCOL_PATH)

    def _temporary_protocol(self, payload: dict[str, Any]) -> Path:
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        path = Path(temporary.name) / "protocol.json"
        path.write_text(json.dumps(payload), encoding="utf-8")
        return path

    def test_protocol_freezes_final_local_two_sensor_grid(self) -> None:
        grid = self.protocol["placement_grid"]
        self.assertEqual(grid["heel_vertical_offsets_below_current_mm"], [2.5, 2.75])
        self.assertEqual(grid["heel_anterior_x_deltas_mm"], [3.0, 3.25])
        self.assertEqual(grid["forefoot_longitudinal_fractions_mesh_x"], [0.79, 0.795])
        self.assertEqual(grid["sensors_per_pair"], 2)
        self.assertEqual(grid["selectable_candidate_count"], 8)
        frozen = json.loads(PROTOCOL_PATH.read_text(encoding="utf-8"))
        self.assertTrue(frozen["declared_final_local_iteration"])
        self.assertEqual(
            self.protocol["runtime_gate_10ms"], self.protocol["fine_gate_1ms"]
        )

    def test_protocol_rejects_grid_and_source_drift(self) -> None:
        frozen = json.loads(PROTOCOL_PATH.read_text(encoding="utf-8"))
        changed = copy.deepcopy(frozen)
        changed["placement_grid"]["heel_anterior_x_deltas_mm"][0] = 2.9
        with self.assertRaises(subject.ProtocolError):
            subject.load_and_validate_protocol(self._temporary_protocol(changed))
        drifted = copy.deepcopy(frozen)
        drifted["sources"]["v8_manifest"]["sha256"] = "0" * 64
        with self.assertRaises(subject.ProtocolError):
            subject.load_and_validate_protocol(self._temporary_protocol(drifted))

    def test_builder_keeps_exactly_two_spheres_per_pair(self) -> None:
        base, candidates, summary = subject.build_placement_candidates(self.protocol)
        self.assertEqual(len([item for item in candidates if item.selectable]), 8)
        self.assertEqual(len(candidates), 10)
        self.assertEqual(summary["sensors_per_pair"], 2)
        sampler, _pairs, profiles = subject.v8.v6.v1._sampling_bundle(
            base, candidates
        )
        self.assertEqual(len(sampler.spheres), 9)
        self.assertTrue(all(len(profile.spheres) == 2 for profile in profiles.values()))
        self.assertEqual(
            {item.candidate_id for item in candidates if not item.selectable},
            {subject.V8_COMPARATOR_ID, subject.v8.v6.CURRENT_COMPARATOR_ID},
        )

    def test_no_clobber_is_inherited(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            output = root / "output"
            plot_dir = root / "plot"
            output.mkdir()
            (output / "owned.txt").write_text("owned", encoding="utf-8")
            with self.assertRaises(subject.v8.v6.NoClobberError):
                subject.v8.v6._preflight_no_clobber(output, plot_dir)
            self.assertFalse(plot_dir.exists())


if __name__ == "__main__":
    unittest.main()
