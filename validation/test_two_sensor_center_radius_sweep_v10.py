"""Pure regression tests for the isolated four-arm V10 geometry sweep."""

from __future__ import annotations

import copy
import json
import math
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
PROTOCOL_PATH = VALIDATION_ROOT / "two_sensor_center_radius_sweep_protocol_v10.json"
EXPECTED_DELTAS_MM = (
    0.150,
    0.210,
    0.215,
    0.220,
    0.225,
    0.250,
    0.275,
    0.325,
    0.340,
    0.341,
    0.500,
    1.000,
    2.000,
    4.000,
)
EXPECTED_ARMS = (
    "heel_center_only",
    "heel_radius_only",
    "toe_center_only",
    "toe_radius_only",
)
if str(VALIDATION_ROOT) not in sys.path:
    sys.path.insert(0, str(VALIDATION_ROOT))


class _ForbiddenOpenSim(ModuleType):
    def __getattr__(self, name: str) -> Any:
        raise AssertionError(f"pure V10 test touched opensim.{name}")


with patch.dict(sys.modules, {"opensim": _ForbiddenOpenSim("opensim")}):
    import sweep_two_sensor_center_radius_prescribed_v10 as subject  # noqa: E402


class TwoSensorCenterRadiusSweepV10Test(unittest.TestCase):
    def setUp(self) -> None:
        self.protocol = subject.load_and_validate_protocol(PROTOCOL_PATH)

    def _temporary_protocol(self, payload: dict[str, Any]) -> Path:
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        path = Path(temporary.name) / "protocol.json"
        path.write_text(json.dumps(payload), encoding="utf-8")
        return path

    @staticmethod
    def _arm(candidate: Any) -> str:
        return str(candidate.geometry.get("parameter_arm", candidate.role))

    def _candidate_fixture(
        self,
    ) -> tuple[Any, list[Any], dict[str, Any], Any, dict[str, Any]]:
        base, candidates, summary = subject.build_candidates(self.protocol)
        sampler, _pairs, profiles = subject._sampling_bundle(base, candidates)
        return base, candidates, summary, sampler, profiles

    def test_protocol_freezes_four_isolated_arms_and_confirmed_timing(self) -> None:
        self.assertEqual(tuple(subject.DELTAS_MM), EXPECTED_DELTAS_MM)
        self.assertEqual(tuple(subject.PARAMETER_ARMS), EXPECTED_ARMS)
        self.assertEqual(subject.SELECTABLE_COUNT, 56)
        self.assertEqual(subject.PAIR_COUNT, 57)
        self.assertEqual(subject.EXPECTED_UNIQUE_DETECTOR_SPHERES, 30)

        grid = self.protocol["isolated_parameter_grid"]
        self.assertEqual(
            tuple(float(value) for value in grid["delta_mm"]), EXPECTED_DELTAS_MM
        )
        self.assertEqual(tuple(grid["parameter_arms"]), EXPECTED_ARMS)
        self.assertEqual(grid["selectable_candidate_count"], 56)
        self.assertEqual(grid["total_pair_count"], 57)
        self.assertEqual(grid["sensors_per_pair"], 2)

        replay = self.protocol["replay"]
        self.assertEqual(replay["primary_event_time_field"], "confirmed_time_s")
        self.assertEqual(replay["diagnostic_event_time_field"], "event_time_s")
        self.assertTrue(replay["evaluate_all_pairs_at_both_resolutions"])
        for cadence in ("runtime_10ms", "fine_1ms"):
            sampling = self.protocol["sampling"][cadence]
            self.assertEqual(sampling["expected_unique_detector_stations"], 30)
            self.assertEqual(sampling["evaluated_pair_count"], 57)

        decision = self.protocol["decision_contract"]
        self.assertTrue(decision["v9_files_must_remain_immutable"])
        self.assertFalse(decision["profile_creation_allowed"])
        self.assertFalse(decision["profile_promotion_allowed"])
        self.assertFalse(decision["sealed_validation_allowed"])

    def test_builder_has_56_selectable_candidates_plus_exact_v9_baseline(self) -> None:
        base, candidates, summary, _sampler, profiles = self._candidate_fixture()
        selectable = [candidate for candidate in candidates if candidate.selectable]
        baseline = [
            candidate
            for candidate in candidates
            if candidate.candidate_id == subject.BASELINE_ID
        ]
        self.assertEqual(len(selectable), 56)
        self.assertEqual(len(candidates), 57)
        self.assertEqual(len(baseline), 1)
        self.assertFalse(baseline[0].selectable)
        self.assertEqual(
            {self._arm(candidate) for candidate in selectable}, set(EXPECTED_ARMS)
        )
        self.assertTrue(all(len(profile.spheres) == 2 for profile in profiles.values()))

        base_sensors = subject.v1._left_sensor_spheres(base)
        baseline_sensors = subject.v1._left_sensor_spheres(
            profiles[subject.BASELINE_ID]
        )
        for role in ("left_heel", "left_toe"):
            self.assertEqual(
                tuple(baseline_sensors[role].location),
                tuple(base_sensors[role].location),
            )
            self.assertEqual(baseline_sensors[role].radius, base_sensors[role].radius)
            self.assertEqual(
                baseline_sensors[role].material,
                base_sensors[role].material,
            )
        self.assertEqual(summary["selectable_candidate_count"], 56)
        self.assertEqual(summary["total_pair_count"], 57)
        self.assertEqual(set(summary["parameter_arms"]), set(EXPECTED_ARMS))

    def test_each_candidate_changes_exactly_one_declared_parameter(self) -> None:
        base, candidates, _summary, _sampler, profiles = self._candidate_fixture()
        baseline = subject.v1._left_sensor_spheres(base)
        heel_location_0 = np.asarray(baseline["left_heel"].location, dtype=float)
        toe_location_0 = np.asarray(baseline["left_toe"].location, dtype=float)
        heel_radius_0 = float(baseline["left_heel"].radius)
        toe_radius_0 = float(baseline["left_toe"].radius)
        observed_deltas: dict[str, list[float]] = {arm: [] for arm in EXPECTED_ARMS}

        for candidate in candidates:
            if not candidate.selectable:
                continue
            arm = self._arm(candidate)
            sensors = subject.v1._left_sensor_spheres(profiles[candidate.candidate_id])
            heel_location = np.asarray(sensors["left_heel"].location, dtype=float)
            toe_location = np.asarray(sensors["left_toe"].location, dtype=float)
            heel_radius = float(sensors["left_heel"].radius)
            toe_radius = float(sensors["left_toe"].radius)

            heel_center_changed = not np.array_equal(heel_location, heel_location_0)
            heel_radius_changed = not math.isclose(
                heel_radius, heel_radius_0, rel_tol=0.0, abs_tol=1.0e-15
            )
            toe_center_changed = not np.array_equal(toe_location, toe_location_0)
            toe_radius_changed = not math.isclose(
                toe_radius, toe_radius_0, rel_tol=0.0, abs_tol=1.0e-15
            )
            changed = {
                "heel_center_only": heel_center_changed,
                "heel_radius_only": heel_radius_changed,
                "toe_center_only": toe_center_changed,
                "toe_radius_only": toe_radius_changed,
            }
            self.assertEqual(
                [name for name, value in changed.items() if value],
                [arm],
                candidate.candidate_id,
            )

            if arm == "heel_center_only":
                np.testing.assert_array_equal(
                    heel_location[[0, 2]], heel_location_0[[0, 2]]
                )
                delta_mm = 1000.0 * float(heel_location[1] - heel_location_0[1])
            elif arm == "heel_radius_only":
                np.testing.assert_array_equal(heel_location, heel_location_0)
                delta_mm = 1000.0 * (heel_radius_0 - heel_radius)
            elif arm == "toe_center_only":
                np.testing.assert_array_equal(
                    toe_location[[0, 2]], toe_location_0[[0, 2]]
                )
                delta_mm = 1000.0 * float(toe_location[1] - toe_location_0[1])
            else:
                np.testing.assert_array_equal(toe_location, toe_location_0)
                delta_mm = 1000.0 * (toe_radius_0 - toe_radius)
            observed_deltas[arm].append(delta_mm)

        for arm in EXPECTED_ARMS:
            np.testing.assert_allclose(
                sorted(observed_deltas[arm]),
                EXPECTED_DELTAS_MM,
                rtol=0.0,
                atol=1.0e-11,
            )

    def test_equal_center_and_radius_deltas_have_equivalent_bottom_height(self) -> None:
        base, candidates, _summary, _sampler, profiles = self._candidate_fixture()
        baseline = subject.v1._left_sensor_spheres(base)
        by_arm_delta: dict[tuple[str, float], tuple[float, float]] = {}
        for candidate in candidates:
            if not candidate.selectable:
                continue
            arm = self._arm(candidate)
            sensors = subject.v1._left_sensor_spheres(profiles[candidate.candidate_id])
            role = "left_heel" if arm.startswith("heel_") else "left_toe"
            base_sphere = baseline[role]
            sphere = sensors[role]
            bottom_0 = float(base_sphere.location[1] - base_sphere.radius)
            bottom = float(sphere.location[1] - sphere.radius)
            delta_mm = round(1000.0 * (bottom - bottom_0), 10)
            by_arm_delta[(arm, delta_mm)] = (bottom_0, bottom)

        for delta in EXPECTED_DELTAS_MM:
            for center_arm, radius_arm in (
                ("heel_center_only", "heel_radius_only"),
                ("toe_center_only", "toe_radius_only"),
            ):
                center_bottom = by_arm_delta[(center_arm, delta)][1]
                radius_bottom = by_arm_delta[(radius_arm, delta)][1]
                self.assertAlmostEqual(center_bottom, radius_bottom, places=14)

    def test_sampling_bundle_has_30_stations_and_preserves_variant_radii(self) -> None:
        base, candidates, _summary, sampler, profiles = self._candidate_fixture()
        self.assertEqual(len(sampler.spheres), 30)
        self.assertEqual(len(profiles), 57)
        self.assertEqual(len({sphere.name for sphere in sampler.spheres}), 30)

        baseline = subject.v1._left_sensor_spheres(base)
        for candidate in candidates:
            sensors = subject.v1._left_sensor_spheres(profiles[candidate.candidate_id])
            self.assertEqual(len(profiles[candidate.candidate_id].spheres), 2)
            if self._arm(candidate) == "heel_radius_only":
                self.assertEqual(
                    tuple(sensors["left_heel"].location),
                    tuple(baseline["left_heel"].location),
                )
                self.assertLess(
                    sensors["left_heel"].radius, baseline["left_heel"].radius
                )
            if self._arm(candidate) == "toe_radius_only":
                self.assertEqual(
                    tuple(sensors["left_toe"].location),
                    tuple(baseline["left_toe"].location),
                )
                self.assertLess(sensors["left_toe"].radius, baseline["left_toe"].radius)

    def test_window_metrics_gate_confirmed_time_and_keep_onset_diagnostic(self) -> None:
        detail = {
            "events": {
                "reference": {
                    "heel_strike": np.asarray([1.0, 2.0]),
                    "toe_off": np.asarray([1.5]),
                },
                "confirmed": {
                    "heel_strike": np.asarray([1.02, 1.98]),
                    "toe_off": np.asarray([1.51]),
                },
                "onset": {
                    "heel_strike": np.asarray([0.92, 1.90]),
                    "toe_off": np.asarray([1.42]),
                },
            }
        }
        result = subject._window_event_metrics(detail, start_s=0.0, end_s=3.0)
        self.assertAlmostEqual(result["heel_strike_confirmed_max_abs_error_s"], 0.02)
        self.assertAlmostEqual(result["heel_strike_onset_max_abs_error_s"], 0.10)
        self.assertAlmostEqual(result["toe_off_confirmed_max_abs_error_s"], 0.01)
        self.assertAlmostEqual(result["toe_off_onset_max_abs_error_s"], 0.08)

    def test_component_ranking_places_none_window_metrics_after_finite(self) -> None:
        def row(
            candidate_id: str,
            *,
            target_sensor: str,
            selectable: bool,
            metric: float | None,
        ) -> dict[str, Any]:
            return {
                "candidate_id": candidate_id,
                "target_sensor": target_sensor,
                "selectable": selectable,
                "event_count_deficit": 0,
                "invalid_timeout_plus_unaccepted_count": 0,
                "transfer_both_latches_off_sample_count": 0,
                "early_max_abs_hs_error_s": metric,
                "long_max_abs_hs_error_s": metric,
                "early_max_abs_toe_off_error_s": metric,
                "long_max_abs_toe_off_error_s": metric,
                "max_abs_hs_error_s": 0.01,
                "global_mean_abs_hs_error_s": 0.01,
                "max_abs_toe_off_error_s": 0.01,
                "global_mean_abs_toe_off_error_s": 0.01,
                "effective_bottom_raise_mm": 0.1,
            }

        rows = [
            row(
                subject.BASELINE_ID,
                target_sensor="baseline",
                selectable=False,
                metric=0.30,
            )
        ]
        for target_sensor in ("heel", "toe"):
            for index in range(2 * len(subject.DELTAS_MM)):
                candidate_id = f"synthetic_{target_sensor}_{index:02d}"
                metric: float | None = 0.20
                if target_sensor == "heel" and index == 0:
                    candidate_id = "finite_window_candidate"
                    metric = 0.02
                elif target_sensor == "heel" and index == 1:
                    candidate_id = "none_window_candidate"
                    # Event-count mismatch diagnostics deliberately expose
                    # unavailable per-window timing as None.
                    metric = None
                rows.append(
                    row(
                        candidate_id,
                        target_sensor=target_sensor,
                        selectable=True,
                        metric=metric,
                    )
                )

        self.assertEqual(len(rows), 57)
        with patch.object(
            subject,
            "strict_gate",
            return_value={"ok": False, "checks": {}},
        ):
            _heel_id, _toe_id, selection = subject.select_components(
                copy.deepcopy(rows),
                copy.deepcopy(rows),
                self.protocol,
            )

        heel_ranked = selection["components"]["heel"][
            "all_candidates_diagnostic_ranked"
        ]
        self.assertLess(
            heel_ranked.index("finite_window_candidate"),
            heel_ranked.index("none_window_candidate"),
        )

    def test_protocol_rejects_grid_timestamp_and_source_hash_drift(self) -> None:
        frozen = json.loads(PROTOCOL_PATH.read_text(encoding="utf-8"))

        grid_drift = copy.deepcopy(frozen)
        grid_drift["isolated_parameter_grid"]["delta_mm"][0] = 0.20
        with self.assertRaises(subject.ProtocolError):
            subject.load_and_validate_protocol(self._temporary_protocol(grid_drift))

        arm_drift = copy.deepcopy(frozen)
        arm_drift["isolated_parameter_grid"]["parameter_arms"][0] = "combined"
        with self.assertRaises(subject.ProtocolError):
            subject.load_and_validate_protocol(self._temporary_protocol(arm_drift))

        timestamp_drift = copy.deepcopy(frozen)
        timestamp_drift["replay"]["primary_event_time_field"] = "event_time_s"
        with self.assertRaises(subject.ProtocolError):
            subject.load_and_validate_protocol(
                self._temporary_protocol(timestamp_drift)
            )

        source_drift = copy.deepcopy(frozen)
        first_source = next(iter(source_drift["sources"].values()))
        first_source["sha256"] = "0" * 64
        with self.assertRaises(subject.ProtocolError):
            subject.load_and_validate_protocol(self._temporary_protocol(source_drift))

    def test_no_clobber_keeps_both_destinations_untouched(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            output = root / "output"
            plot_dir = root / "plot"
            output.mkdir()
            marker = output / "owned.txt"
            marker.write_text("owned", encoding="utf-8")
            with self.assertRaises(subject.NoClobberError):
                subject._preflight_no_clobber(output, plot_dir)
            self.assertEqual(marker.read_text(encoding="utf-8"), "owned")
            self.assertFalse(plot_dir.exists())


if __name__ == "__main__":
    unittest.main()
