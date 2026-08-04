"""Pure tests for the preregistered two-sensor V5 micro-sweep."""

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
PROTOCOL_PATH = VALIDATION_ROOT / "two_sensor_timing_placement_sweep_protocol_v5.json"
if str(VALIDATION_ROOT) not in sys.path:
    sys.path.insert(0, str(VALIDATION_ROOT))


class _ForbiddenOpenSim(ModuleType):
    def __getattr__(self, name: str) -> Any:
        raise AssertionError(f"pure V5 test touched opensim.{name}")


with patch.dict(sys.modules, {"opensim": _ForbiddenOpenSim("opensim")}):
    import sweep_two_sensor_timing_placements_prescribed_v5 as subject  # noqa: E402


class TwoSensorTimingPlacementV5Test(unittest.TestCase):
    def setUp(self) -> None:
        self.protocol = subject.load_and_validate_protocol(PROTOCOL_PATH)

    def _temporary_protocol(self, payload: dict[str, Any]) -> Path:
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        path = Path(temporary.name) / "protocol.json"
        path.write_text(json.dumps(payload), encoding="utf-8")
        return path

    def test_protocol_freezes_depth_only_two_sensor_contract(self) -> None:
        grid = self.protocol["placement_grid"]
        self.assertEqual(grid["heel_anterior_x_deltas_mm"], [2.0])
        self.assertEqual(grid["selectable_candidate_count"], 8)
        self.assertEqual(grid["maximum_forefoot_protrusion_mm"], 36.0)
        self.assertEqual(grid["sensors_per_pair"], 2)
        self.assertEqual(
            self.protocol["development_gate_10ms"][
                "minimum_forefoot_release_margin_s"
            ],
            0.25,
        )
        self.assertTrue(
            self.protocol["replay"]["sensitivity_requires_strict_10ms_winner"]
        )
        self.assertFalse(
            self.protocol["decision_contract"]["profile_promotion_allowed"]
        )

    def test_protocol_rejects_depth_selection_and_source_drift(self) -> None:
        frozen = json.loads(PROTOCOL_PATH.read_text(encoding="utf-8"))
        depth_drift = copy.deepcopy(frozen)
        depth_drift["placement_grid"][
            "forefoot_absolute_local_plantar_protrusion_mm"
        ][-1] = 36.25
        with self.assertRaises(subject.ProtocolError):
            subject.load_and_validate_protocol(self._temporary_protocol(depth_drift))
        selection_drift = copy.deepcopy(frozen)
        selection_drift["selection"]["primary_order"] = "maximum_depth"
        with self.assertRaises(subject.ProtocolError):
            subject.load_and_validate_protocol(
                self._temporary_protocol(selection_drift)
            )
        source_drift = copy.deepcopy(frozen)
        source_drift["sources"]["production_fsm"]["sha256"] = "0" * 64
        with self.assertRaises(subject.ProtocolError):
            subject.load_and_validate_protocol(self._temporary_protocol(source_drift))

    def test_builder_has_eight_pairs_plus_current_and_twosphere_profiles(self) -> None:
        base, candidates, summary = subject.build_placement_candidates(self.protocol)
        selectable = [candidate for candidate in candidates if candidate.selectable]
        self.assertEqual(len(selectable), 8)
        self.assertEqual(len(candidates), 9)
        self.assertEqual(
            [candidate.forefoot_protrusion_mm for candidate in selectable],
            list(subject.PROTRUSIONS_MM),
        )
        self.assertTrue(all(candidate.geometry["pre_gate_ok"] for candidate in selectable))
        self.assertLessEqual(summary["maximum_absolute_bottom_offset_mm"], 20.0)
        sampler, _pairs, profiles = subject.v1._sampling_bundle(base, candidates)
        self.assertEqual(len(sampler.spheres), 11)
        self.assertTrue(all(len(profile.spheres) == 2 for profile in profiles.values()))

    def test_release_margin_is_computed_and_gated(self) -> None:
        base_row = {"candidate_id": "candidate"}
        base_detail = {
            "forefoot_swing_release": {
                "swings": [
                    {
                        "reference_to_s": 1.0,
                        "next_reference_hs_s": 1.4,
                        "forefoot_active_duration_s": 0.1,
                    },
                    {
                        "reference_to_s": 2.0,
                        "next_reference_hs_s": 2.35,
                        "forefoot_active_duration_s": 0.08,
                    },
                ]
            }
        }
        release = {
            "definition": "exact final active sample",
            "minimum_s": 0.27,
            "per_cycle_s": [0.30, 0.27],
            "cycles": [],
        }
        with (
            patch.object(
                subject.v4,
                "evaluate_placement",
                return_value=(base_row, base_detail),
            ),
            patch.object(
                subject,
                "forefoot_release_margin_diagnostics",
                return_value=release,
            ),
        ):
            row, detail = subject.evaluate_placement(
                self.protocol, object(), {}, sample_dt_s=0.01
            )
        self.assertAlmostEqual(row["minimum_forefoot_release_margin_s"], 0.27)
        self.assertEqual(len(detail["forefoot_release_margin"]["per_cycle_s"]), 2)
        with patch.object(
            subject.v4,
            "strict_gate",
            return_value={"ok": True, "checks": {}},
        ):
            self.assertTrue(
                subject.strict_gate(
                    row, {}, self.protocol, gate_key="development_gate_10ms"
                )["ok"]
            )
            row["minimum_forefoot_release_margin_s"] = 0.249
            self.assertFalse(
                subject.strict_gate(
                    row, {}, self.protocol, gate_key="development_gate_10ms"
                )["ok"]
            )

    def test_release_margin_uses_last_active_sample_not_total_duration(self) -> None:
        times = np.arange(1.0, 2.0, 0.1)
        inputs = {
            "times": times,
            "loads": {"left_heel": np.zeros(10), "left_toe": np.zeros(10)},
            "penetrations": {
                "left_heel": np.zeros(10),
                "left_toe": np.zeros(10),
            },
            "aggregate": np.zeros(10),
            "kinematics": {},
            "body_weight_n": 700.0,
            "reference_events": {
                "heel_strike": np.asarray([0.5, 2.0]),
                "toe_off": np.asarray([1.0]),
            },
        }
        toe_contact = np.asarray(
            [1, 1, 0, 0, 0, 0, 0, 1, 0, 0], dtype=float
        )
        with (
            patch.object(
                subject.dual,
                "compose_branch_inputs",
                return_value=(inputs, {}),
            ),
            patch.object(
                subject.v1,
                "_run_production_fsm",
                return_value={"toe_contact": toe_contact},
            ),
        ):
            result = subject.forefoot_release_margin_diagnostics(
                {}, sample_dt_s=0.1
            )
        self.assertAlmostEqual(result["minimum_s"], 0.2)
        self.assertAlmostEqual(
            result["cycles"][0]["last_active_toe_latch_sample_s"], 1.7
        )

    def test_selection_chooses_shallowest_full_strict_pass(self) -> None:
        rows: list[dict[str, Any]] = []
        for index, depth in enumerate(subject.PROTRUSIONS_MM):
            rows.append(
                {
                    "candidate_id": subject._candidate_id(depth),
                    "selectable": True,
                    "forefoot_protrusion_mm": depth,
                    "worst_event_normalized_max_abs_error": 1.0 - index / 100.0,
                    "mean_event_normalized_mean_abs_error": 1.0,
                    "confirmed_fsm_stance_iou": 0.96,
                    "geometry_displacement_from_current_m": depth,
                }
            )
        rows.append(
            {"candidate_id": subject.CURRENT_COMPARATOR_ID, "selectable": False}
        )
        with (
            patch.object(
                subject,
                "strict_gate",
                return_value={"ok": True, "checks": {}},
            ),
            patch.object(
                subject.v4,
                "operational_reference_noise_diagnostic",
                return_value={"ok": True, "role": "secondary_diagnostic_only"},
            ),
        ):
            winner, decision = subject.select_10ms_winner(rows, self.protocol)
        self.assertEqual(winner, subject._candidate_id(34.25))
        self.assertEqual(
            decision["winner_rule"],
            "shallowest_full_strict_pass_then_timing_tiebreakers",
        )

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
