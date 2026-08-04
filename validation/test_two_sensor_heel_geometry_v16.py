from __future__ import annotations

import copy
import math
import tempfile
import unittest
from pathlib import Path
from unittest import mock


import sys


REPO_ROOT = Path(__file__).resolve().parents[1]
VALIDATION_ROOT = REPO_ROOT / "validation"
if str(VALIDATION_ROOT) not in sys.path:
    sys.path.insert(0, str(VALIDATION_ROOT))

import sweep_two_sensor_heel_geometry_v16 as subject  # noqa: E402


class V16ProtocolTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.protocol = subject.load_and_validate_protocol()

    def test_scope_is_development_only_and_non_promoting(self) -> None:
        self.assertEqual(
            self.protocol["split"]["DEVELOPMENT"], list(subject.ALLOWED_TRIALS)
        )
        self.assertEqual(
            self.protocol["split"]["FORBIDDEN"], list(subject.FORBIDDEN_TRIALS)
        )
        self.assertFalse(
            self.protocol["decision_contract"][
                "holdout_or_runtime_promotion_allowed"
            ]
        )

    def test_parent_failures_and_unopened_splits_are_locked(self) -> None:
        result = subject.validate_parent_state(self.protocol)
        self.assertEqual(
            result["status"],
            "PASS_V16_PARENT_FAILURES_AND_PROTECTED_SPLITS_VERIFIED",
        )
        self.assertTrue(all(result["checks"].values()))

    def test_only_locked_dev_preprocessing_is_declared(self) -> None:
        trials = self.protocol["preprocessing"]["trials"]
        self.assertEqual(set(trials), set(subject.ALLOWED_TRIALS))
        for trial_id, records in trials.items():
            self.assertEqual(set(records), subject.EXPECTED_PREPROCESSING_KEYS)
            self.assertIn(f"preprocessed/trial_{trial_id}", records["ik_motion"]["path"])

    def test_replay_is_full_fsm_and_not_signal_router(self) -> None:
        replay = self.protocol["replay"]
        self.assertTrue(replay["full_production_fsm"])
        self.assertFalse(replay["signal_only_router_used"])
        self.assertTrue(replay["continuous_trial_no_plateau_reset"])
        self.assertEqual(replay["sensor_on_threshold_n"], 0.5)
        self.assertEqual(replay["sensor_off_threshold_n"], 0.25)
        self.assertEqual(replay["sensor_dwell_s"], 0.03)


class V16GeometryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.protocol = subject.load_and_validate_protocol()
        cls.base, cls.candidates, cls.summary = subject.build_candidates(cls.protocol)
        cls.baseline = next(
            item for item in cls.candidates if item.candidate_id == subject.BASELINE_ID
        )

    def test_grid_and_candidate_cardinality_are_exact(self) -> None:
        self.assertEqual(
            subject.DELTA_GRID_MM,
            (0.15, 0.25, 0.35, 0.45, 0.55, 0.65, 0.75, 0.85, 0.95, 1.05, 1.15, 1.25),
        )
        self.assertEqual(len(self.candidates), subject.EXPECTED_CANDIDATE_COUNT)
        self.assertEqual(len({item.candidate_id for item in self.candidates}), 25)
        self.assertEqual(sum(item.candidate_id == subject.BASELINE_ID for item in self.candidates), 1)

    def test_hy_and_hr_are_strictly_separate_one_factor_arms(self) -> None:
        baseline_y = float(self.baseline.heel_location[1])
        baseline_radius = float(self.baseline.geometry["heel_radius_m"])
        baseline_toe = tuple(self.baseline.forefoot_location)
        for candidate in self.candidates[1:]:
            arm = candidate.geometry["parameter_arm"]
            delta_mm = 1000.0 * float(
                candidate.geometry["geometry_displacement_from_v13_m"]
            )
            self.assertIn(arm, subject.ARM_ORDER)
            self.assertTrue(
                any(
                    math.isclose(delta_mm, expected, abs_tol=1.0e-12)
                    for expected in subject.DELTA_GRID_MM
                )
            )
            self.assertEqual(tuple(candidate.forefoot_location), baseline_toe)
            if arm == "Hy":
                self.assertAlmostEqual(
                    candidate.heel_location[1], baseline_y - delta_mm / 1000.0
                )
                self.assertAlmostEqual(
                    candidate.geometry["heel_radius_m"], baseline_radius
                )
            else:
                self.assertAlmostEqual(candidate.heel_location[1], baseline_y)
                self.assertAlmostEqual(
                    candidate.geometry["heel_radius_m"],
                    baseline_radius + delta_mm / 1000.0,
                )

    def test_all_geometry_pre_gates_pass_and_station_count_is_frozen(self) -> None:
        self.assertTrue(
            all(candidate.geometry["pre_gate_ok"] for candidate in self.candidates)
        )
        plan = subject.v14._build_sampling_plan(
            self.base,
            self.candidates,
            stage_label="v16_unit_test",
            expected_detector_stations=subject.EXPECTED_STATION_COUNT,
        )
        self.assertEqual(len(plan.sampler.spheres), subject.EXPECTED_STATION_COUNT)

    def test_baseline_matches_frozen_v13_geometry(self) -> None:
        profile = subject.v1.load_online_grf_profile(
            subject._resolve(self.protocol["geometry"]["profile"]["path"]),
            required_sides=("left",),
        )
        sensors = subject.v1._left_sensor_spheres(profile)
        heel = sensors["left_heel"]
        toe = sensors["left_toe"]
        self.assertEqual(tuple(heel.location), tuple(self.baseline.heel_location))
        self.assertEqual(tuple(toe.location), tuple(self.baseline.forefoot_location))
        self.assertTrue(
            math.isclose(heel.radius, self.baseline.geometry["heel_radius_m"])
        )


class V16ParityAndSelectionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.protocol = subject.load_and_validate_protocol()

    def test_parent_baseline_has_exact_24_full_detail_units(self) -> None:
        details = subject._load_parent_baseline_details(self.protocol)
        self.assertEqual(len(details), subject.EXPECTED_UNIT_COUNT)
        parity = subject.baseline_anchor_parity(
            self.protocol, list(details.values())
        )
        self.assertTrue(parity["ok"])
        self.assertEqual(
            parity["status"],
            "PASS_EXACT_V14_2_BASELINE_FULL_FSM_DETAIL_PARITY",
        )

    def test_baseline_parity_rejects_internal_detail_mutation(self) -> None:
        details = subject._load_parent_baseline_details(self.protocol)
        observed = [copy.deepcopy(details[key]) for key in sorted(details)]
        target = observed[len(observed) // 2]
        target["row"]["predicted_hs_count"] += 1
        parity = subject.baseline_anchor_parity(self.protocol, observed)
        self.assertFalse(parity["ok"])

    def test_v13_wins_an_exact_tie_and_no_candidate_is_selected(self) -> None:
        base, candidates, _summary = subject.build_candidates(self.protocol)
        del base
        details = subject._load_parent_baseline_details(self.protocol)
        baseline_rows = [copy.deepcopy(details[key]["row"]) for key in sorted(details)]
        rows = []
        for candidate in candidates:
            for baseline_row in baseline_rows:
                row = copy.deepcopy(baseline_row)
                row["candidate_id"] = candidate.candidate_id
                rows.append(row)
        decision = subject.select_development_result(rows, candidates)
        self.assertIsNone(decision["finalist_id"])
        self.assertEqual(
            decision["status"], "V13_RETAINED_NO_V16_DEVELOPMENT_FINALIST"
        )


class V16NoClobberTests(unittest.TestCase):
    def test_noncanonical_and_existing_destinations_are_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            with self.assertRaises(subject.NoClobberError):
                subject._preflight_no_clobber(Path(temp_dir) / "alternate")
        with tempfile.TemporaryDirectory() as temp_dir:
            canonical = Path(temp_dir) / "canonical"
            with mock.patch.object(subject, "DEFAULT_OUTPUT_DIR", canonical):
                subject._preflight_no_clobber(canonical)
                canonical.mkdir()
                with self.assertRaises(subject.NoClobberError):
                    subject._preflight_no_clobber(canonical)


if __name__ == "__main__":
    unittest.main()
