"""Pure tests for the two-sensor/primary-load causal A/B diagnostic."""

from __future__ import annotations

import copy
import io
import json
import sys
import tempfile
import unittest
from contextlib import redirect_stdout
from pathlib import Path
from types import ModuleType, SimpleNamespace
from typing import Any
from unittest.mock import patch

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
VALIDATION_DIR = REPO_ROOT / "validation"
PROTOCOL_PATH = VALIDATION_DIR / "two_sensor_dual_stream_prescribed_protocol.json"
if str(VALIDATION_DIR) not in sys.path:
    sys.path.insert(0, str(VALIDATION_DIR))


class _ForbiddenOpenSim(ModuleType):
    def __getattr__(self, name: str) -> Any:
        raise AssertionError(f"pure dual-stream test touched opensim.{name}")


with patch.dict(sys.modules, {"opensim": _ForbiddenOpenSim("opensim")}):
    import diagnose_two_sensor_dual_stream_prescribed as subject  # noqa: E402


class TwoSensorDualStreamPrescribedTest(unittest.TestCase):
    def setUp(self) -> None:
        self.protocol = subject.load_and_validate_protocol(PROTOCOL_PATH)

    def _temporary_protocol(self, payload: dict[str, Any]) -> Path:
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        path = Path(temporary.name) / "protocol.json"
        path.write_text(json.dumps(payload), encoding="utf-8")
        return path

    @staticmethod
    def _common_streams() -> dict[str, Any]:
        times = np.asarray([50.0, 50.01, 50.02], dtype=float)
        detector_loads = {
            "left_heel": np.asarray([0.0, 0.6, 0.7]),
            "left_toe": np.asarray([0.0, 0.0, 0.8]),
        }
        detector_penetrations = {
            "left_heel": np.asarray([0.0, 0.001, 0.001]),
            "left_toe": np.asarray([0.0, 0.0, 0.001]),
        }
        return {
            "setup": object(),
            "times": times,
            "kinematics": {
                "knee_rad": np.zeros(3),
                "ankle_rad": np.zeros(3),
            },
            "prescribed_vertical_n": np.zeros(3),
            "reference_events": {
                "heel_strike": np.asarray([50.0, 50.02]),
                "toe_off": np.asarray([50.01]),
            },
            "body_weight_n": 700.0,
            "detector_loads": detector_loads,
            "detector_penetrations": detector_penetrations,
            "detector_aggregate": np.asarray([0.0, 0.6, 1.5]),
            "primary_aggregate": np.asarray([0.0, 350.0, 500.0]),
            "primary_penetration": np.asarray([0.0, 0.002, 0.003]),
        }

    def test_protocol_freezes_causal_diagnostic_without_promotion(self) -> None:
        self.assertEqual(self.protocol["candidate_ids"], list(subject.CANDIDATE_IDS))
        self.assertEqual(set(self.protocol["branches"]), set(subject.BRANCH_IDS))
        decision = self.protocol["decision_contract"]
        self.assertEqual(decision["role"], "causal_diagnostic_only")
        self.assertFalse(decision["geometry_acceptance_applied"])
        self.assertFalse(decision["candidate_selection_allowed"])
        self.assertFalse(decision["profile_promotion_allowed"])
        self.assertFalse(decision["sealed_validation_allowed"])
        self.assertFalse(decision["training_allowed"])

    def test_protocol_has_every_evaluator_replay_input(self) -> None:
        required = {
            "sensor_dwell_s",
            "prescribed_contact_threshold_n",
            "hs_tolerance_s",
            "toe_off_tolerance_s",
            "primary_event_time_field",
            "diagnostic_event_time_field",
            "phase_reference_mode",
            "fsm_min_stance_contact_fraction",
            "fsm_min_stance_load_bw_s",
        }
        self.assertTrue(required.issubset(self.protocol["replay"]))
        self.assertEqual(self.protocol["replay"]["fsm_min_stance_load_bw_s"], 0.04)

    def test_protocol_rejects_branch_or_source_hash_drift(self) -> None:
        frozen = json.loads(PROTOCOL_PATH.read_text(encoding="utf-8"))
        branch_drift = copy.deepcopy(frozen)
        branch_drift["branches"]["B_primary_load"]["heel_toe_event_guards"] = (
            "primary_online_grf"
        )
        with self.assertRaises(subject.ProtocolError):
            subject.load_and_validate_protocol(
                self._temporary_protocol(branch_drift)
            )
        hash_drift = copy.deepcopy(frozen)
        hash_drift["sources"]["setup"]["sha256"] = "0" * 64
        with self.assertRaises(subject.ProtocolError):
            subject.load_and_validate_protocol(self._temporary_protocol(hash_drift))

    def test_builder_returns_only_the_two_frozen_two_sensor_pairs(self) -> None:
        _base, candidates, summary = subject.build_candidates(self.protocol)
        self.assertEqual(
            [candidate.candidate_id for candidate in candidates],
            list(subject.CANDIDATE_IDS),
        )
        self.assertEqual(
            [candidate.forefoot_protrusion_mm for candidate in candidates],
            [31.5, 32.0],
        )
        self.assertEqual(summary["candidate_count"], 2)
        self.assertEqual(summary["sensors_per_candidate"], 2)
        self.assertFalse(summary["affine_reconstruction_used"])

    def test_branch_adapter_changes_only_load_evidence(self) -> None:
        common = self._common_streams()
        branch_a, source_a = subject.compose_branch_inputs(
            common, "A_detector_load"
        )
        branch_b, source_b = subject.compose_branch_inputs(
            common, "B_primary_load"
        )
        self.assertIs(branch_a["loads"]["left_heel"], branch_b["loads"]["left_heel"])
        self.assertIs(branch_a["loads"]["left_toe"], branch_b["loads"]["left_toe"])
        np.testing.assert_array_equal(branch_a["aggregate"], common["detector_aggregate"])
        np.testing.assert_array_equal(branch_b["aggregate"], common["primary_aggregate"])
        primary_contact = (
            (branch_b["penetrations"]["left_heel"] > 0.0)
            | (branch_b["penetrations"]["left_toe"] > 0.0)
        )
        np.testing.assert_array_equal(
            primary_contact, common["primary_penetration"] > 0.0
        )
        self.assertEqual(
            source_a["heel_toe_event_guards"],
            source_b["heel_toe_event_guards"],
        )
        self.assertNotEqual(source_a["normal_force_bw"], source_b["normal_force_bw"])

    def test_reference_cycle_load_integrals_are_per_cycle(self) -> None:
        times = np.arange(0.0, 2.0, 0.1)
        force_n = np.full(times.shape, 100.0)
        events = {
            "heel_strike": np.asarray([0.0, 1.0, 2.0]),
            "toe_off": np.asarray([0.5, 1.5]),
        }
        values = subject._reference_cycle_load_integrals_bw_s(
            times,
            force_n,
            500.0,
            events,
            sample_dt_s=0.1,
        )
        np.testing.assert_allclose(values, [0.1, 0.1], atol=1e-12)

    def test_reason_counts_exposes_stance_load_failure(self) -> None:
        replay = {
            "invalid_steps": [
                {"type": "stance_load_too_low"},
                {"type": "stance_load_too_low"},
                {"type": "to_too_early_after_hs"},
            ]
        }
        detail = {
            "semantic_gate": {
                "timeout_transitions": [
                    {"cycle_reject_reason": "phase_timeout:stance"}
                ]
            }
        }
        counts = subject._reason_counts(replay, detail)
        self.assertEqual(counts["stance_load_too_low"], 2)
        self.assertEqual(counts["total_invalid_steps"], 3)
        self.assertEqual(counts["phase_timeout"], 1)

    def test_branch_comparison_requires_identical_raw_guard_trace(self) -> None:
        rows: list[dict[str, Any]] = []
        for candidate_id in subject.CANDIDATE_IDS:
            for branch_id, load_failures, cycles in (
                ("A_detector_load", 3, 0),
                ("B_primary_load", 0, 2),
            ):
                rows.append(
                    {
                        "candidate_id": candidate_id,
                        "branch_id": branch_id,
                        "detector_guard_trace_sha256": f"trace-{candidate_id}",
                        "stance_load_too_low_count": load_failures,
                        "predicted_hs_count": 3,
                        "predicted_to_count": 2,
                        "observed_valid_cycle_count": cycles,
                        "invalid_reason_counts_json": json.dumps(
                            {"stance_load_too_low": load_failures}
                        ),
                    }
                )
        comparisons = subject.compare_branches(rows)
        self.assertTrue(
            all(item["detector_guard_trace_identical"] for item in comparisons)
        )
        self.assertTrue(
            all(item["stance_load_too_low_disappears"] for item in comparisons)
        )
        self.assertEqual(
            subject._diagnostic_conclusion(comparisons),
            "PRIMARY_LOAD_REMOVES_STANCE_LOAD_TOO_LOW_FOR_ALL_CANDIDATES",
        )
        rows[1]["detector_guard_trace_sha256"] = "drift"
        self.assertEqual(
            subject._diagnostic_conclusion(subject.compare_branches(rows)),
            "INVALID_AB_DETECTOR_GUARD_STREAM_DRIFT",
        )

    def test_sampling_contract_uses_one_direct_eleven_sphere_pass(self) -> None:
        base, candidates, _summary = subject.build_candidates(self.protocol)
        detector_sampler, _pairs, _profiles = subject.v1._sampling_bundle(
            base, candidates
        )
        self.assertEqual(len(detector_sampler.spheres), 3)
        times = np.asarray([50.0, 50.01], dtype=float)
        events = {
            "heel_strike": np.asarray([50.0, 50.01]),
            "toe_off": np.asarray([50.005]),
        }
        setup = SimpleNamespace(model_file=REPO_ROOT / "dummy.osim")

        def fake_sample(_setup: Any, sampler: Any, sample_times: Any, _plugin: str) -> dict[str, Any]:
            self.assertEqual(len(sampler.spheres), 11)
            self.assertEqual(len(sample_times), 2)
            centers = {
                sphere.name: np.zeros((2, 3), dtype=float)
                for sphere in sampler.spheres
            }
            velocities = {
                sphere.name: np.zeros((2, 3), dtype=float)
                for sphere in sampler.spheres
            }
            return {"centers": centers, "velocities": velocities}

        with (
            patch.object(
                subject.v1,
                "_reference_bundle",
                return_value=(setup, events, {"samples_at_or_after_100_s": 0}, times),
            ),
            patch.object(subject.v1, "_sample_spheres", side_effect=fake_sample) as sampler,
            patch.object(
                subject.v1,
                "_prescribed_prosthetic_kinematics",
                return_value={"knee_rad": np.zeros(2), "ankle_rad": np.zeros(2)},
            ),
            patch.object(
                subject.v1,
                "_external_grf",
                return_value={"left": np.zeros((2, 3)), "right": np.zeros((2, 3))},
            ),
            patch.object(subject.v1, "_model_body_weight_n", return_value=700.0),
        ):
            streams, access = subject.sample_all_streams_once(
                self.protocol, base, candidates
            )
        self.assertEqual(sampler.call_count, 1)
        self.assertEqual(access["sampled_total_unique_sphere_count"], 11)
        self.assertTrue(access["single_opensim_sphere_sampling_pass"])
        self.assertFalse(
            not access["direct_sphere_sampling_without_affine_reconstruction"]
        )
        self.assertEqual(set(streams), set(subject.CANDIDATE_IDS))

    def test_no_clobber_preserves_occupied_output_and_plot(self) -> None:
        for occupied_kind in ("output", "plot"):
            with self.subTest(occupied_kind=occupied_kind), tempfile.TemporaryDirectory() as temporary:
                root = Path(temporary)
                output_dir = root / "output"
                plot_dir = root / "plot"
                occupied = output_dir if occupied_kind == "output" else plot_dir
                occupied.mkdir()
                sentinel = occupied / "prior.txt"
                sentinel.write_text("preserve", encoding="utf-8")
                with redirect_stdout(io.StringIO()):
                    exit_code = subject.main(
                        [
                            "--protocol",
                            str(PROTOCOL_PATH),
                            "--output-dir",
                            str(output_dir),
                            "--plot-dir",
                            str(plot_dir),
                        ]
                    )
                self.assertEqual(exit_code, 2)
                self.assertEqual(sentinel.read_text(encoding="utf-8"), "preserve")
                self.assertEqual(
                    sorted(path.name for path in occupied.iterdir()), ["prior.txt"]
                )
                other = plot_dir if occupied_kind == "output" else output_dir
                self.assertFalse(other.exists())


if __name__ == "__main__":
    unittest.main()
