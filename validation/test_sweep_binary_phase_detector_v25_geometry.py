"""Independent, non-numerical tests for the governed V25 reach sweep.

The suite never imports OpenSim and never calls ``execute``.  It exercises the
fixed candidate domain, oracle-stance continuity, exact 24-unit verification,
selection, provenance, no-clobber output, and progress cosmetics with synthetic
affine traces and small test doubles.
"""

from __future__ import annotations

import ast
import builtins
import io
import itertools
import json
import math
import os
import sys
import tempfile
import unittest
from dataclasses import dataclass
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
VALIDATION_ROOT = REPO_ROOT / "validation"
if str(VALIDATION_ROOT) not in sys.path:
    sys.path.insert(0, str(VALIDATION_ROOT))

import sweep_binary_phase_detector_v21_geometry as v21_progress  # noqa: E402
import sweep_binary_phase_detector_v25_geometry as subject  # noqa: E402


V24_SCRIPT_SHA256 = (
    "5a037622848fbd47ee302b158fe7ec17f0fbdde4f5cd51f3681ea5653282a0c0"
)


@dataclass(frozen=True)
class FakePoint:
    role: str
    x_m: float
    reach_m: float

    @property
    def surface_location_m(self) -> tuple[float, float, float]:
        return (self.x_m, 0.0, 0.01 if self.role == "left_heel" else -0.01)

    @property
    def location_m(self) -> tuple[float, float, float]:
        x, y, z = self.surface_location_m
        return (x, y - self.reach_m, z)

    def payload(self) -> dict:
        return {
            "role": self.role,
            "x_m": self.x_m,
            "reach_m": self.reach_m,
            "surface_location_m": list(self.surface_location_m),
            "location_m": list(self.location_m),
        }


class FakeFactory:
    def __init__(self) -> None:
        self.calls: list[tuple[str, float, float]] = []

    def build(self, role: str, x_m: float, reach_m: float) -> FakePoint:
        self.calls.append((str(role), float(x_m), float(reach_m)))
        return FakePoint(str(role), float(x_m), float(reach_m))


class FakeClock:
    def __init__(self) -> None:
        self.now = 0.0

    def __call__(self) -> float:
        return self.now


class V25GeometrySweepTests(unittest.TestCase):
    def _temporary_dir(self) -> Path:
        temporary = tempfile.TemporaryDirectory(dir=VALIDATION_ROOT)
        self.addCleanup(temporary.cleanup)
        return Path(temporary.name)

    @staticmethod
    def _candidate() -> subject.V25Candidate:
        return subject.V25Candidate(
            heel=FakePoint(
                "left_heel",
                subject.PARENT_HEEL_X_M,
                subject.PARENT_HEEL_REACH_MM / 1000.0,
            ),
            toe=FakePoint(
                "left_toe",
                subject.PARENT_TOE_X_M,
                subject.PARENT_TOE_REACH_MM / 1000.0,
            ),
            stage="synthetic",
            arm_membership=("heel_only", "toe_only"),
        )

    @staticmethod
    def _trace(times: np.ndarray) -> SimpleNamespace:
        coefficients = np.zeros((times.size, 3), dtype=float)
        coefficients[:, 1] = 1.0
        return SimpleNamespace(
            time_s=np.asarray(times, dtype=float),
            normal_coefficients=coefficients,
            normal_offset=np.full(times.size, 0.1, dtype=float),
        )

    @staticmethod
    def _cache_for(
        candidate: subject.V25Candidate,
        heel: np.ndarray,
        toe: np.ndarray,
    ) -> dict:
        cache: dict = {}
        for trial_id in subject.TRIALS:
            for point, values in ((candidate.heel, heel), (candidate.toe, toe)):
                key = (
                    trial_id,
                    point.role,
                    tuple(round(float(item), 12) for item in point.location_m),
                )
                cache[key] = np.asarray(values, dtype=bool)
        return cache

    def test_v24_receipt_detail_and_script_are_all_hash_pinned(self) -> None:
        prerequisite = subject._verify_v24_prerequisite()
        self.assertTrue(prerequisite["pass"])
        self.assertEqual(
            subject.sha256_file(subject.V24_DIAGNOSTIC_RECEIPT),
            subject.V24_DIAGNOSTIC_RECEIPT_SHA256,
        )
        self.assertEqual(
            subject.sha256_file(subject.V24_DIAGNOSTIC_DETAIL),
            subject.V24_DIAGNOSTIC_DETAIL_SHA256,
        )

        self.assertTrue(hasattr(subject, "V24_DIAGNOSTIC_SCRIPT"))
        self.assertTrue(hasattr(subject, "V24_DIAGNOSTIC_SCRIPT_SHA256"))
        self.assertEqual(
            subject.V24_DIAGNOSTIC_SCRIPT_SHA256, V24_SCRIPT_SHA256
        )
        self.assertEqual(
            subject.sha256_file(subject.V24_DIAGNOSTIC_SCRIPT),
            V24_SCRIPT_SHA256,
        )
        self.assertEqual(
            prerequisite["diagnostic_script"]["sha256"], V24_SCRIPT_SHA256
        )

    def test_trial_allowlist_is_exact_and_forbidden_trials_are_absent(self) -> None:
        self.assertEqual(subject.TRIALS, ("02", "04", "08"))
        self.assertEqual(subject.PROTECTED_TRIALS, ("05", "06"))
        self.assertEqual(subject.RESERVE_TRIALS, ("03", "07"))
        self.assertEqual(subject.HISTORICAL_TRIALS, ("01",))
        self.assertEqual(set(subject.TRIAL_SPECS), {"02", "04", "08"})
        self.assertEqual(subject.EXPECTED_VIEW_COUNT, 4)
        self.assertEqual(subject.EXPECTED_VERIFICATION_UNITS, 24)
        serialized = json.dumps(subject.TRIAL_SPECS, sort_keys=True)
        for forbidden in ("01", "03", "05", "06", "07"):
            self.assertNotIn(f'trial_{forbidden}', serialized)
            self.assertNotIn(f'trial {forbidden}', serialized)
        for forbidden in ("03", "05", "06", "07"):
            with self.assertRaises((KeyError, subject.V25SweepError)):
                subject._time_grid(forbidden)

    def test_all_development_time_grids_are_unique_monotonic_exact_1ms(self) -> None:
        for trial_id in subject.TRIALS:
            with self.subTest(trial_id=trial_id):
                times = subject._time_grid(trial_id)
                spec = subject.TRIAL_SPECS[trial_id]
                self.assertEqual(times.size, spec["sample_count"])
                self.assertAlmostEqual(
                    float(times[0]), float(spec["interval_s"][0]), places=12
                )
                self.assertAlmostEqual(
                    float(times[-1]), float(spec["interval_s"][1]), places=10
                )
                self.assertTrue(np.all(times[1:] > times[:-1]))
                self.assertTrue(
                    np.allclose(
                        np.diff(times),
                        subject.SAMPLE_DT_S,
                        atol=1e-12,
                        rtol=0.0,
                    )
                )

    def test_preflight_is_platform_neutral_read_only_and_never_imports_opensim(self) -> None:
        real_import = builtins.__import__

        def guarded_import(name, globals=None, locals=None, fromlist=(), level=0):
            if name == "opensim" or name.startswith("opensim."):
                raise AssertionError("OpenSim imported by read-only preflight")
            return real_import(name, globals, locals, fromlist, level)

        with patch("builtins.__import__", side_effect=guarded_import):
            result = subject.preflight(numerical=False)
        self.assertEqual(result["status"], "V25_PREFLIGHT_PASS")
        self.assertEqual(result["frozen_domain"]["trials"], ["02", "04", "08"])
        self.assertEqual(result["frozen_domain"]["sample_dt_s"], 0.001)
        self.assertTrue(result["frozen_domain"]["x_and_lateral_geometry_frozen"])
        self.assertTrue(result["frozen_domain"]["fsm_v20_frozen"])
        self.assertTrue(result["platform"]["path_code_platform_neutral"])
        scope = result["data_scope"]
        self.assertTrue(scope["pass"])
        self.assertTrue(all(scope["assertions"].values()))
        self.assertEqual(scope["development_allowlist"], ["02", "04", "08"])
        self.assertEqual(scope["protected_denylist"], ["05", "06"])
        self.assertEqual(scope["reserve_denylist"], ["03", "07"])
        self.assertEqual(scope["historical_denylist"], ["01"])
        self.assertEqual(scope["opened_protected_trials"], [])
        self.assertEqual(scope["opened_reserve_trials"], [])
        self.assertEqual(scope["opened_historical_trials"], [])

    def test_stage_one_contains_parent_once_and_changes_only_one_reach(self) -> None:
        factory = FakeFactory()
        candidates = subject._generate_candidates(factory, "single_sensor_arms")
        self.assertEqual(len(candidates), 9)
        self.assertEqual(len({candidate.key for candidate in candidates}), 9)
        parent = [
            candidate
            for candidate in candidates
            if abs(candidate.heel.reach_m * 1000.0 - 25.0) <= 1e-12
            and abs(candidate.toe.reach_m * 1000.0 - 27.0) <= 1e-12
        ]
        self.assertEqual(len(parent), 1)
        self.assertEqual(parent[0].arm_membership, ("heel_only", "toe_only"))
        self.assertEqual(subject.PARENT_CANDIDATE_ID, "v21_678b0b5162b706dd")
        self.assertEqual(
            parent[0].payload()["parent_candidate_id"],
            subject.PARENT_CANDIDATE_ID,
        )
        self.assertEqual(
            subject.sha256_file(subject.PARENT_PROFILE_PATH),
            subject.PARENT_PROFILE_SHA256,
        )

        expected_pairs = {
            (float(heel), float(toe))
            for axes in subject.SINGLE_SENSOR_ARMS_MM.values()
            for heel, toe in itertools.product(axes["heel"], axes["toe"])
        }
        observed_pairs = {
            (
                round(candidate.heel.reach_m * 1000.0, 10),
                round(candidate.toe.reach_m * 1000.0, 10),
            )
            for candidate in candidates
        }
        self.assertEqual(observed_pairs, expected_pairs)
        for candidate in candidates:
            heel_changed = abs(candidate.heel.reach_m * 1000.0 - 25.0) > 1e-12
            toe_changed = abs(candidate.toe.reach_m * 1000.0 - 27.0) > 1e-12
            self.assertFalse(heel_changed and toe_changed)
            self.assertEqual(candidate.heel.x_m, subject.PARENT_HEEL_X_M)
            self.assertEqual(candidate.toe.x_m, subject.PARENT_TOE_X_M)

    def test_conditional_stage_two_is_fixed_reach_only_cross(self) -> None:
        factory = FakeFactory()
        candidates = subject._generate_candidates(
            factory, "conditional_two_dimensional_cross"
        )
        self.assertEqual(len(candidates), 16)
        self.assertEqual(len({candidate.key for candidate in candidates}), 16)
        expected_pairs = set(
            itertools.product(
                subject.CONDITIONAL_STAGE2_MM["heel"],
                subject.CONDITIONAL_STAGE2_MM["toe"],
            )
        )
        observed_pairs = {
            (
                round(candidate.heel.reach_m * 1000.0, 10),
                round(candidate.toe.reach_m * 1000.0, 10),
            )
            for candidate in candidates
        }
        self.assertEqual(observed_pairs, expected_pairs)
        self.assertTrue(
            all(candidate.arm_membership == ("conditional_cross",) for candidate in candidates)
        )
        self.assertTrue(
            all(candidate.heel.x_m == subject.PARENT_HEEL_X_M for candidate in candidates)
        )
        self.assertTrue(
            all(candidate.toe.x_m == subject.PARENT_TOE_X_M for candidate in candidates)
        )

    def test_ground_mesh_contact_rule_x_and_fsm_are_frozen(self) -> None:
        self.assertEqual(subject.GROUND_ORIGIN, (0.0, 0.0148208231, 0.0))
        self.assertEqual(subject.GROUND_NORMAL, (0.0, 1.0, 0.0))
        self.assertEqual(subject.FOOT_FRAME, "/bodyset/foot_l")
        self.assertEqual(
            subject.PINNED_SOURCES["Geometry/AM_foot_l.STL"],
            "fcfc4d7a90c4ccd3bedb501ec3e50d4337aa9ca6e8438b58cc6be00f47a689e9",
        )
        fsm_path = REPO_ROOT / "Trajectory Generator/binary_phase_fsm.py"
        self.assertEqual(
            subject.PINNED_SOURCES[
                "Trajectory Generator/binary_phase_fsm.py"
            ],
            "0f7669b60a72c1b27ee3c4f1a43161eeb9f2d091dff5558cc4fa43f1fce8d9c1",
        )
        self.assertEqual(
            subject.sha256_file(fsm_path),
            subject.PINNED_SOURCES["Trajectory Generator/binary_phase_fsm.py"],
        )
        profile = subject._candidate_profile(self._candidate())
        self.assertEqual(profile["ground"]["origin"], list(subject.GROUND_ORIGIN))
        self.assertEqual(profile["ground"]["normal"], list(subject.GROUND_NORMAL))
        self.assertEqual(
            profile["contact_rule"], {"contact_when": "signed_clearance_le_zero"}
        )
        self.assertEqual(
            [point["location"][0] for point in profile["points"]],
            [subject.PARENT_HEEL_X_M, subject.PARENT_TOE_X_M],
        )

    def test_continuity_gate_accepts_continuous_contact_and_excludes_swing_edges(self) -> None:
        candidate = self._candidate()
        times = np.arange(11, dtype=float) * subject.SAMPLE_DT_S
        traces = {trial: self._trace(times) for trial in subject.TRIALS}
        ledgers = {
            trial: {
                "scientific_core": {
                    "cycles": [
                        {
                            "cycle_id": f"{trial}_cycle",
                            "heel_strike_time_s": float(times[0]),
                            "toe_off_time_s": float(times[-1]),
                        }
                    ]
                }
            }
            for trial in subject.TRIALS
        }
        fake_v21 = SimpleNamespace(MIN_STABLE_RUN_SAMPLES=6)
        heel = np.ones(times.size, dtype=bool)
        toe = np.zeros(times.size, dtype=bool)
        continuous = subject.oracle_stance_continuity_gate(
            candidate,
            traces=traces,
            ledgers=ledgers,
            v21=fake_v21,
            bit_cache=self._cache_for(candidate, heel, toe),
        )
        self.assertTrue(continuous["pass"])
        self.assertEqual(continuous["raw_internal_gap_count"], 0)

        edge_heel = np.zeros(times.size, dtype=bool)
        edge_heel[2:9] = True
        edge_only = subject.oracle_stance_continuity_gate(
            candidate,
            traces=traces,
            ledgers=ledgers,
            v21=fake_v21,
            bit_cache=self._cache_for(candidate, edge_heel, toe),
        )
        self.assertTrue(edge_only["pass"])
        self.assertTrue(edge_only["ordinary_swing_excluded"])
        self.assertEqual(edge_only["raw_internal_gap_count"], 0)

    def test_continuity_gate_rejects_even_one_ms_internal_gap(self) -> None:
        candidate = self._candidate()
        times = np.arange(11, dtype=float) * subject.SAMPLE_DT_S
        traces = {trial: self._trace(times) for trial in subject.TRIALS}
        ledgers = {
            trial: {
                "scientific_core": {
                    "cycles": [
                        {
                            "cycle_id": f"{trial}_cycle",
                            "heel_strike_time_s": float(times[0]),
                            "toe_off_time_s": float(times[-1]),
                        }
                    ]
                }
            }
            for trial in subject.TRIALS
        }
        heel = np.ones(times.size, dtype=bool)
        heel[5] = False
        toe = np.zeros(times.size, dtype=bool)
        result = subject.oracle_stance_continuity_gate(
            candidate,
            traces=traces,
            ledgers=ledgers,
            v21=SimpleNamespace(MIN_STABLE_RUN_SAMPLES=6),
            bit_cache=self._cache_for(candidate, heel, toe),
        )
        self.assertFalse(result["pass"])
        self.assertEqual(result["trial_count"], 3)
        self.assertEqual(result["raw_internal_gap_count"], 3)
        self.assertEqual(result["debounce_stable_internal_gap_count"], 0)
        for trial in subject.TRIALS:
            gap = result["trials"][trial]["gaps"][0]
            self.assertEqual(gap["sample_count"], 1)
            self.assertFalse(gap["debounce_stable"])
            self.assertTrue(gap["shorter_than_30ms"])

    def test_exact_verification_builds_24_units_and_scalar_batch_parity(self) -> None:
        candidate = self._candidate()
        times = np.arange(21, dtype=float) * subject.SAMPLE_DT_S
        traces = {trial: self._trace(times) for trial in subject.TRIALS}
        views = [{"view_id": f"view_{index}"} for index in range(4)]
        ledgers = {
            trial: {"scientific_core": {"views": copy_views(views)}}
            for trial in subject.TRIALS
        }
        heel = np.ones(times.size, dtype=bool)
        toe = np.zeros(times.size, dtype=bool)
        events = [
            {"event": "heel_strike", "event_time_s": 0.001},
            {"event": "toe_off", "event_time_s": 0.010},
        ]

        class FakeV20:
            @staticmethod
            def _run_mode(_trace, _mode):
                return {
                    "events": list(events),
                    "contact_state_transitions": [{"state": "HEEL"}],
                    "candidate_cancellations": [],
                    "boundary_snapshots_sha256": "a" * 64,
                    "final_payload": {"state": "HEEL"},
                }

            @staticmethod
            def _score_view(**_kwargs):
                return {"pass": True, "accepted_flight_pass": True}

        class FakeV21:
            @staticmethod
            def two_sensor_view_gate(_times, _heel, _toe, _views):
                return {"pass": True}

            @staticmethod
            def fast_fsm_events(_times, _heel, _toe):
                return list(events)

            @staticmethod
            def event_signature(items):
                return tuple((item["event"], item["event_time_s"]) for item in items)

            @staticmethod
            def _compact_unit(unit):
                return dict(unit)

        with patch.object(
            subject, "_compact_unit", side_effect=lambda _v21, unit: dict(unit)
        ):
            result = subject._verify_candidate(
                candidate,
                {
                    "oracle_stance_continuity": {"pass": True},
                    "geometry_gate": {"pass": True},
                },
                traces=traces,
                ledgers=ledgers,
                v21=FakeV21,
                v20=FakeV20,
                bit_cache=self._cache_for(candidate, heel, toe),
            )
        self.assertTrue(result["eligible"])
        self.assertEqual(result["expected_unit_count"], 24)
        self.assertEqual(result["unit_count"], 24)
        self.assertEqual(result["unit_pass_count"], 24)
        self.assertEqual(len(result["parity"]), 3)
        self.assertTrue(all(item["pass"] for item in result["parity"]))
        self.assertTrue(result["toe_clearance_at_least_30ms_pass"])

    def test_selection_key_is_deterministic_order_independent_and_fail_closed(self) -> None:
        rows = [
            {
                "candidate_id": "v25_b",
                "eligible": True,
                "reach_delta_l1_mm": 0.50,
                "selection_metrics": {"fixed_arm_center_distance_mm": 0.25},
            },
            {
                "candidate_id": "v25_a",
                "eligible": True,
                "reach_delta_l1_mm": 0.50,
                "selection_metrics": {"fixed_arm_center_distance_mm": 0.25},
            },
            {
                "candidate_id": "v25_ineligible",
                "eligible": False,
                "reach_delta_l1_mm": 0.0,
                "selection_metrics": {"fixed_arm_center_distance_mm": 0.0},
            },
        ]
        for permutation in itertools.permutations(rows):
            selected = min(permutation, key=subject._selection_key)
            self.assertEqual(selected["candidate_id"], "v25_a")
            self.assertTrue(selected["eligible"])
        ineligible = [dict(row, eligible=False) for row in rows[:2]]
        self.assertFalse(any(row["eligible"] for row in ineligible))

    def test_writers_are_strict_atomic_and_no_clobber_including_symlink(self) -> None:
        temporary = self._temporary_dir()
        path = temporary / "result.json"
        subject._write_json_exclusive(path, {"value": 1.0})
        original = path.read_bytes()
        with self.assertRaises((FileExistsError, subject.V25SweepError)):
            subject._write_json_exclusive(path, {"value": 2.0})
        self.assertEqual(path.read_bytes(), original)

        occupied = temporary / "occupied.json"
        occupied.symlink_to("missing-target.json")
        self.assertTrue(os.path.lexists(occupied))
        with self.assertRaises((FileExistsError, subject.V25SweepError)):
            subject._write_json_exclusive(occupied, {"value": 3.0})
        self.assertTrue(occupied.is_symlink())
        self.assertEqual(os.readlink(occupied), "missing-target.json")

        nonfinite = temporary / "nonfinite.json"
        with self.assertRaises(ValueError):
            subject._write_json_exclusive(nonfinite, {"value": math.nan})
        self.assertFalse(nonfinite.exists())
        self.assertEqual(list(temporary.glob(".*.tmp")), [])

    def test_progress_renders_bar_eta_and_elapsed_time(self) -> None:
        clock = FakeClock()
        stream = io.StringIO()
        progress = v21_progress.SweepProgress(
            total=4,
            label="V25 synthetic",
            stream=stream,
            width=8,
            min_redraw_interval_s=0.0,
            non_tty_interval_s=0.0,
            clock=clock,
        )
        clock.now = 5.0
        progress.update(1)
        clock.now = 10.0
        progress.update(2)
        clock.now = 20.0
        progress.update(4)
        progress.finish("DONE")
        output = stream.getvalue()
        self.assertIn("[", output)
        self.assertIn("#", output)
        self.assertIn("time elapsed", output)
        self.assertIn("ETA", output)
        self.assertIn("00:00:20", output)
        self.assertIn("4/4", output)
        self.assertIn("DONE", output)

    def test_cli_exposes_no_trial_geometry_fsm_h0_or_protected_overrides(self) -> None:
        tree = ast.parse(Path(subject.__file__).read_text(encoding="utf-8"))
        options: set[str] = set()
        for node in ast.walk(tree):
            if (
                isinstance(node, ast.Call)
                and isinstance(node.func, ast.Attribute)
                and node.func.attr == "add_argument"
            ):
                for argument in node.args:
                    if (
                        isinstance(argument, ast.Constant)
                        and isinstance(argument.value, str)
                        and argument.value.startswith("-")
                    ):
                        options.add(argument.value)
        self.assertEqual(
            options,
            {
                "--check",
                "--execute",
                "--output-dir",
                "--progress-width",
                "--progress-interval-s",
                "--non-tty-progress-interval-s",
                "--no-progress",
            },
        )
        forbidden = (
            "trial",
            "heel",
            "toe",
            "reach",
            "geometry",
            "mesh",
            "ground",
            "fsm",
            "h0",
            "protected",
            "reserve",
        )
        for option in options:
            self.assertFalse(any(word in option.lower() for word in forbidden), option)


def copy_views(views: list[dict]) -> list[dict]:
    return [dict(view) for view in views]


if __name__ == "__main__":
    unittest.main()
