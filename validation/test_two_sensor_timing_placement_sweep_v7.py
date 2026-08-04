"""Pure regression tests for the V7 correction of the frozen V6 harness."""

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
PROTOCOL_PATH = VALIDATION_ROOT / "two_sensor_timing_placement_sweep_protocol_v7.json"
V5_MANIFEST = (
    VALIDATION_ROOT
    / "two_sensor_timing_placement_sweep_runs/"
    "2026-07-22_ab06_50_100_x2_depth_micro_v5/manifest.json"
)
if str(VALIDATION_ROOT) not in sys.path:
    sys.path.insert(0, str(VALIDATION_ROOT))


class _ForbiddenOpenSim(ModuleType):
    def __getattr__(self, name: str) -> Any:
        raise AssertionError(f"pure V7 test touched opensim.{name}")


with patch.dict(sys.modules, {"opensim": _ForbiddenOpenSim("opensim")}):
    import sweep_two_sensor_timing_placements_prescribed_v7 as subject  # noqa: E402


class TwoSensorTimingPlacementV7Test(unittest.TestCase):
    def setUp(self) -> None:
        self.protocol = subject.load_and_validate_protocol(PROTOCOL_PATH)

    def _temporary_protocol(self, payload: dict[str, Any]) -> Path:
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        path = Path(temporary.name) / "protocol.json"
        path.write_text(json.dumps(payload), encoding="utf-8")
        return path

    def test_correction_adds_only_missing_inherited_gate(self) -> None:
        self.assertEqual(self.protocol["sealed_validation_gate"], subject.SEALED_GATE)
        self.assertFalse(
            self.protocol["_v7_harness_correction"][
                "scientific_contract_changed_from_v6"
            ]
        )
        self.assertEqual(self.protocol["placement_grid"]["sensors_per_pair"], 2)
        self.assertEqual(self.protocol["placement_grid"]["total_pair_count"], 10)
        self.assertTrue(
            self.protocol["replay"]["evaluate_all_pairs_at_both_resolutions"]
        )

    def test_inherited_strict_gate_no_longer_raises_missing_key(self) -> None:
        manifest = json.loads(V5_MANIFEST.read_text(encoding="utf-8"))
        candidate = next(
            row
            for row in manifest["primary_10ms"]["rows"]
            if row["candidate_id"] == "H02_X2_F80_P35p00"
        )
        current = next(
            row
            for row in manifest["primary_10ms"]["rows"]
            if row["candidate_id"] == subject.v6.CURRENT_COMPARATOR_ID
        )
        candidate = {
            **candidate,
            "causal_swing_interval_count": 50,
            "minimum_causal_toe_clear_before_next_hs_onset_s": 0.30,
        }
        result = subject.v6.strict_gate(
            candidate,
            current,
            self.protocol,
            gate_key="runtime_gate_10ms",
        )
        self.assertIn("ok", result)
        self.assertTrue(result["checks"]["exact_causal_swing_intervals"])

    def test_protocol_rejects_scientific_change_and_source_drift(self) -> None:
        frozen = json.loads(PROTOCOL_PATH.read_text(encoding="utf-8"))
        changed = copy.deepcopy(frozen)
        changed["scientific_contract_changed_from_v6"] = True
        with self.assertRaises(subject.ProtocolError):
            subject.load_and_validate_protocol(self._temporary_protocol(changed))
        drifted = copy.deepcopy(frozen)
        drifted["sources"]["v6_validator"]["sha256"] = "0" * 64
        with self.assertRaises(subject.ProtocolError):
            subject.load_and_validate_protocol(self._temporary_protocol(drifted))

    def test_invalid_v6_is_hash_pinned_and_sealed_remains_closed(self) -> None:
        frozen = json.loads(PROTOCOL_PATH.read_text(encoding="utf-8"))
        record = frozen["sources"]["invalid_v6_manifest"]
        path = subject.v6.v1.resolve_repo_path(record["path"])
        self.assertEqual(subject.v6.v1._sha256(path), record["sha256"])
        invalid = json.loads(path.read_text(encoding="utf-8"))
        self.assertFalse(invalid["scientific_result_available"])
        self.assertFalse(invalid["data_access"]["sealed_block_opened"])


if __name__ == "__main__":
    unittest.main()
