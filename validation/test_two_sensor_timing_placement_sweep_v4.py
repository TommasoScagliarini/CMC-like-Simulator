"""Pure tests for the preregistered two-sensor V4 micro-sweep."""

from __future__ import annotations

import copy
import json
import sys
import tempfile
import unittest
from pathlib import Path
from types import ModuleType, SimpleNamespace
from typing import Any
from unittest.mock import patch


REPO_ROOT = Path(__file__).resolve().parents[1]
VALIDATION_ROOT = REPO_ROOT / "validation"
PROTOCOL_PATH = VALIDATION_ROOT / "two_sensor_timing_placement_sweep_protocol_v4.json"
if str(VALIDATION_ROOT) not in sys.path:
    sys.path.insert(0, str(VALIDATION_ROOT))


class _ForbiddenOpenSim(ModuleType):
    def __getattr__(self, name: str) -> Any:
        raise AssertionError(f"pure V4 test touched opensim.{name}")


with patch.dict(sys.modules, {"opensim": _ForbiddenOpenSim("opensim")}):
    import sweep_two_sensor_timing_placements_prescribed_v4 as subject  # noqa: E402


class TwoSensorTimingPlacementV4Test(unittest.TestCase):
    def setUp(self) -> None:
        self.protocol = subject.load_and_validate_protocol(PROTOCOL_PATH)

    def _temporary_protocol(self, payload: dict[str, Any]) -> Path:
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        path = Path(temporary.name) / "protocol.json"
        path.write_text(json.dumps(payload), encoding="utf-8")
        return path

    def test_protocol_freezes_grid_routing_and_conditional_1ms(self) -> None:
        grid = self.protocol["placement_grid"]
        self.assertEqual(grid["heel_anterior_x_deltas_mm"], [0.0, 2.0, 4.0])
        self.assertEqual(grid["selectable_candidate_count"], 12)
        self.assertEqual(grid["sensors_per_pair"], 2)
        self.assertEqual(self.protocol["sampling"]["primary_10ms"]["expected_total_unique_spheres"], 17)
        replay = self.protocol["replay"]
        self.assertTrue(replay["sensitivity_requires_strict_10ms_winner"])
        self.assertTrue(replay["sensitivity_evaluates_selected_and_current_only"])
        self.assertTrue(replay["one_ms_cannot_convert_10ms_fail_to_pass"])
        self.assertFalse(self.protocol["decision_contract"]["profile_promotion_allowed"])

    def test_protocol_rejects_depth_gate_and_source_drift(self) -> None:
        frozen = json.loads(PROTOCOL_PATH.read_text(encoding="utf-8"))
        depth_drift = copy.deepcopy(frozen)
        depth_drift["placement_grid"][
            "forefoot_absolute_local_plantar_protrusion_mm"
        ][-1] = 34.25
        with self.assertRaises(subject.ProtocolError):
            subject.load_and_validate_protocol(self._temporary_protocol(depth_drift))
        diagnostic_drift = copy.deepcopy(frozen)
        diagnostic_drift["operational_reference_noise_diagnostic"][
            "can_replace_strict_gate"
        ] = True
        with self.assertRaises(subject.ProtocolError):
            subject.load_and_validate_protocol(
                self._temporary_protocol(diagnostic_drift)
            )
        source_drift = copy.deepcopy(frozen)
        source_drift["sources"]["production_fsm"]["sha256"] = "0" * 64
        with self.assertRaises(subject.ProtocolError):
            subject.load_and_validate_protocol(self._temporary_protocol(source_drift))

    def test_builder_has_twelve_two_sphere_pairs_plus_current(self) -> None:
        base, candidates, summary = subject.build_placement_candidates(self.protocol)
        selectable = [candidate for candidate in candidates if candidate.selectable]
        self.assertEqual(len(selectable), 12)
        self.assertEqual(len(candidates), 13)
        self.assertEqual(
            sorted({candidate.geometry["heel_anterior_x_delta_mm"] for candidate in selectable}),
            [0.0, 2.0, 4.0],
        )
        self.assertEqual(
            sorted({candidate.forefoot_protrusion_mm for candidate in selectable}),
            list(subject.PROTRUSIONS_MM),
        )
        self.assertTrue(all(candidate.geometry["pre_gate_ok"] for candidate in selectable))
        self.assertLessEqual(summary["maximum_absolute_bottom_offset_mm"], 20.0)
        sampler, _pairs, profiles = subject.v1._sampling_bundle(base, candidates)
        self.assertEqual(len(sampler.spheres), 9)
        self.assertTrue(all(len(profile.spheres) == 2 for profile in profiles.values()))

    def test_reference_noise_diagnostic_never_changes_strict_selection(self) -> None:
        row = {
            "reference_hs_count": 51,
            "predicted_hs_count": 51,
            "hs_comparable_event_count": 51,
            "hs_within_50ms_count": 50,
            "max_abs_hs_error_s": 0.055,
        }
        diagnostic = subject.operational_reference_noise_diagnostic(
            row, self.protocol
        )
        self.assertTrue(diagnostic["ok"])
        self.assertFalse(diagnostic["can_replace_strict_gate"])
        self.assertFalse(diagnostic["can_make_candidate_selectable"])
        self.assertFalse(diagnostic["can_make_candidate_promotable"])
        row["max_abs_hs_error_s"] = 0.061
        self.assertFalse(
            subject.operational_reference_noise_diagnostic(row, self.protocol)["ok"]
        )

    def test_sensitivity_pair_is_exactly_locked_winner_and_current(self) -> None:
        candidates = [
            SimpleNamespace(candidate_id="winner"),
            SimpleNamespace(candidate_id=subject.CURRENT_COMPARATOR_ID),
            SimpleNamespace(candidate_id="other"),
        ]
        winner, current = subject.sensitivity_pair(candidates, "winner")
        self.assertEqual(winner.candidate_id, "winner")
        self.assertEqual(current.candidate_id, subject.CURRENT_COMPARATOR_ID)
        with self.assertRaises(subject.ProtocolError):
            subject.sensitivity_pair(candidates, "missing")

    def test_no_clobber_does_not_create_other_destination(self) -> None:
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
