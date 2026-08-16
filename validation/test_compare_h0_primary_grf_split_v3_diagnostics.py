from __future__ import annotations

import json
import math
import tempfile
import unittest
from pathlib import Path

import numpy as np

from validation import compare_h0_primary_grf_split_v3_diagnostics as subject


class DiagnosticFixture:
    def __init__(self, root: Path, *, seed: int = 123) -> None:
        self.root = root
        self.seed = seed
        self.history = root / f"historical_seed{seed}"
        self.history.mkdir(parents=True)
        self._write_json(self.history / "trace.json", [{"step": 1}])
        self._write_json(self.history / "rollout_summary.json", {"steps": 500})

    @staticmethod
    def _write_json(path: Path, payload) -> None:
        path.write_text(
            json.dumps(payload, indent=2, sort_keys=True, allow_nan=False) + "\n",
            encoding="utf-8",
        )

    def summary(self) -> dict:
        return {
            "schema_version": 3,
            "seed": self.seed,
            "steps": 500,
            "end_reason": "episode_time_limit",
            "terminated": False,
            "truncated": True,
            "phase_valid_cycle_count": 3,
            "invalid_event_count": 1,
            "historical_invalid_event_count": 1,
            "grf_penetration_max_m": 0.023,
            "action_clipped_values": 0,
            "timeout_count": 0,
            "safety_stop_count": 0,
            "fallback_count": 1,
            "so_fallback_count": 1,
            "sea_plugin_fallback_count": 0,
            "so_solver_control_window_count": 500,
            "so_solver_attempt_count": 500,
            "so_solver_hard_fallback_count": 1,
            "so_solver_primary_nonconvergence_count": 1,
            "so_solver_bounded_ls_invocation_count": 1,
            "so_solver_selected_bounded_ls_count": 1,
            "so_solver_verified_bounded_ls_count": 0,
            "so_solver_reuse_previous_count": 0,
            "so_solver_bounded_ls_unsuccessful_count": 1,
            "so_solver_bounds_violation_count": 0,
            "so_solver_nonfinite_count": 0,
            "so_solver_selected_infeasible_count": 0,
            "so_solver_selected_solution_mismatch_count": 0,
            "so_solver_residual_contract_mismatch_count": 0,
            "so_solver_audit_schema": "static_optimization_solver_audit_v1",
            "hard_invalid_count": 0,
            "nonfinite_count": 0,
            "fixed_feature_mismatch_count": 0,
            "teacher_view_mismatch_count": 0,
            "mutable_feature_difference_count": 499,
            "time_mismatch_count": 0,
            "n_actor": 35,
            "n_observation": 84,
            "observation_dtype": "float32",
            "actor_feature_names": [f"feature_{i}" for i in range(35)],
            "observation_feature_names": [f"observation_{i}" for i in range(84)],
            "event_contract_id": subject.EVENT_CONTRACT,
            "binary_phase_fsm_mode": "disabled",
            "online_grf_applied_sides": ["left"],
            "morphology_weight": 0.0,
            "episode_return": -1.0,
            "episode_metrics": {},
            "sea_episode_metrics": {},
            "h0_used_for_behavior": False,
            "historical_action_replay_used_for_behavior": True,
            "actor_updates": 0,
            "critic_updates": 0,
            "ppo_updates": 0,
            "protected_trials_opened": [],
        }

    def arrays(self) -> dict[str, np.ndarray]:
        return {
            key: (
                np.asarray([f"feature_{i}" for i in range(35)], dtype=dtype)
                if key == "actor_feature_names"
                else np.zeros(shape, dtype=dtype)
            )
            for key, (dtype, shape) in subject.EXPECTED_ARCHIVE.items()
        }

    @staticmethod
    def _solution() -> dict:
        return {
            "output_shape_matches": True,
            "output_finite": True,
            "output_sha256": "a" * 64,
            "bound_violation_max": 0.0,
            "equality_residual_finite": True,
            "equality_residual_norm": 1.0e-8,
            "equality_residual_max_abs": 8.0e-9,
            "equality_residual_relative_norm": 2.0e-10,
        }

    def _solver_path(self, *, anomaly: bool, solver_marker: int) -> dict:
        solution = self._solution()
        path = {
            "schema": subject.so_recovery.SCHEMA,
            "primary_solver": "slsqp",
            "input_matrix_shape": [3, 4],
            "input_target_shape": [3],
            "input_matrix_finite": True,
            "input_target_finite": True,
            "weights_finite": True,
            "bounds_finite": True,
            "warm_start_finite": True,
            "input_matrix_sha256": "b" * 64,
            "input_target_sha256": "c" * 64,
            "weights_sha256": "d" * 64,
            "bounds_sha256": "e" * 64,
            "warm_start_sha256": "f" * 64,
            "slsqp_invoked": True,
            "slsqp_success": not anomaly,
            "slsqp_status": 8 if anomaly else 0,
            "slsqp_message": "positive directional derivative" if anomaly else "ok",
            "slsqp_iterations": 17 + solver_marker,
            "slsqp_solution": solution,
            "bounded_lsq_used": anomaly,
            "reuse_previous_solution": False,
            "selected_solution": solution,
            "hard_fallback": anomaly,
        }
        if anomaly:
            path.update(
                {
                    "bounded_lsq_invoked": True,
                    "bounded_lsq_success": False,
                    "bounded_lsq_status": 0,
                    "bounded_lsq_message": (
                        "The maximum number of iterations is exceeded."
                    ),
                    "bounded_lsq_iterations": subject.BOUNDED_LSQ_MAX_ITER,
                    "bounded_lsq_cost": 1.0e-12,
                    "bounded_lsq_optimality": 1.0e-9,
                    "bounded_lsq_solution": solution,
                }
            )
        return path

    def _solver_window(self, *, step: int, solver_marker: int) -> dict:
        anomaly = step == 10
        return {
            "control_window_index": 1,
            "control_window_time_s": step / 100.0,
            "selected_feasibility_attempt_index": 1,
            "served_solution_sha256": "a" * 64,
            "selected_solver_solution_matches_served": True,
            "attempts": [
                {
                    "attempt_index": 1,
                    "feasibility_scale": 1.0,
                    "feasibility_accepted": True,
                    "residual_norm": 1.0e-8,
                    "residual_relative_norm": 2.0e-10,
                    "residual_max_abs": 8.0e-9,
                    "solver_fallback_used": anomaly,
                    "selected": True,
                    "solver_path": self._solver_path(
                        anomaly=anomaly, solver_marker=solver_marker
                    ),
                }
            ],
        }

    def make_replica(
        self,
        name: str,
        *,
        summary: dict | None = None,
        gate: dict | None = None,
        arrays: dict[str, np.ndarray] | None = None,
        solver_marker: int = 0,
    ) -> Path:
        directory = self.root / name
        directory.mkdir()
        summary = self.summary() if summary is None else summary
        gate = subject._expected_gate(summary) if gate is None else gate
        fallback = [
            {
                "step": 10,
                "time_s": 0.1,
                "so_fallback": 1,
                "sea_plugin_fallback_count": 0,
            }
        ]
        solver = [
            {
                "step": step,
                "time_s": step / 100.0,
                "control_windows": [
                    self._solver_window(step=step, solver_marker=solver_marker)
                ],
            }
            for step in range(1, 501)
        ]
        self._write_json(directory / "summary.json", summary)
        self._write_json(directory / "gate.json", gate)
        self._write_json(directory / "fallback_journal.json", fallback)
        self._write_json(directory / "solver_audit_journal.json", solver)
        np.savez(directory / "paired_replay.npz", **(arrays or self.arrays()))
        artifacts = {
            name: subject.source_record(directory / filename)
            for name, filename in subject.RECEIPT_ARTIFACT_NAMES.items()
        }
        artifacts["historical_trace"] = subject.source_record(
            self.history / "trace.json"
        )
        artifacts["historical_summary"] = subject.source_record(
            self.history / "rollout_summary.json"
        )
        receipt = {
            "schema_version": 3,
            "status": "FAIL_H0_PRIMARY_SPLIT_V3_REPLAY",
            "passed": False,
            "seed": self.seed,
            "diagnostic": True,
            "execution_lock": None,
            "attempt_claim": None,
            "artifacts": artifacts,
            "actor_updates": 0,
            "critic_updates": 0,
            "ppo_updates": 0,
            "protected_trials_opened": [],
        }
        self._write_json(directory / "receipt.json", receipt)
        return directory


class V3DiagnosticComparatorTests(unittest.TestCase):
    def test_exact_diagnostics_pass_without_upgrading_protocol_gate(self) -> None:
        for seed in subject.ALLOWED_DIAGNOSTIC_SEEDS:
            with self.subTest(seed=seed), tempfile.TemporaryDirectory() as raw:
                fixture = DiagnosticFixture(Path(raw), seed=seed)
                a = fixture.make_replica("a")
                b = fixture.make_replica("b")
                receipt = subject.compare_diagnostics(
                    seed=seed, replica_a_dir=a, replica_b_dir=b
                )
                self.assertFalse(receipt["passed"])
                self.assertTrue(receipt["comparison_passed"])
                self.assertTrue(receipt["diagnostic"])
                self.assertFalse(receipt["source_protocol_gate_passed"])
                self.assertFalse(receipt["eligible_for_protocol_promotion"])
                self.assertEqual(
                    set(receipt["measured_source_gate_failures"]),
                    subject.MEASURED_FAILED_CHECKS,
                )
                self.assertTrue(receipt["zero_residual_contract_mismatch"])
                self.assertEqual(receipt["actor_updates"], 0)
                self.assertEqual(receipt["protected_trials_opened"], [])
                self.assertEqual(
                    set(receipt["npz_arrays"]), set(subject.EXPECTED_ARCHIVE)
                )
                classifier = receipt["independent_solver_classification"]
                self.assertEqual(
                    classifier["bounded_lsq_status0_max_iter_case_count"], 1
                )
                self.assertEqual(
                    classifier["scientific_disposition"],
                    "UNACCEPTED_DIAGNOSTIC_REQUIRES_PROTOCOL_DECISION",
                )
                for key in (
                    "v3_runner",
                    "solver_classifier",
                    "static_optimization",
                    "simulation_runner",
                    "environment",
                ):
                    self.assertIn(key, receipt["sources"])

    def test_repository_source_record_matches_v3_schema(self) -> None:
        record = subject.source_record(subject.COMPARATOR)
        self.assertNotIn("path_scope", record)
        self.assertEqual(subject._record_path(record, "comparator"), subject.COMPARATOR)

    def test_seed_125_is_rejected_before_any_input_access(self) -> None:
        with self.assertRaisesRegex(
            subject.V3DiagnosticComparisonError, "seeds 123/124"
        ):
            subject.compare_diagnostics(
                seed=125,
                replica_a_dir="/does/not/exist/a",
                replica_b_dir="/does/not/exist/b",
            )

    def test_source_must_reproduce_exact_measured_failure_set(self) -> None:
        with tempfile.TemporaryDirectory() as raw:
            fixture = DiagnosticFixture(Path(raw))
            summary = fixture.summary()
            summary["timeout_count"] = 1
            a = fixture.make_replica("a", summary=summary)
            b = fixture.make_replica("b", summary=summary)
            with self.assertRaisesRegex(
                subject.V3DiagnosticComparisonError, "four-check"
            ):
                subject.compare_diagnostics(seed=123, replica_a_dir=a, replica_b_dir=b)

    def test_json_and_solver_drift_fail_closed(self) -> None:
        with tempfile.TemporaryDirectory() as raw:
            fixture = DiagnosticFixture(Path(raw))
            a = fixture.make_replica("a")
            changed = fixture.summary()
            changed["episode_return"] = -2.0
            b = fixture.make_replica("b", summary=changed)
            with self.assertRaisesRegex(subject.V3DiagnosticComparisonError, "summary"):
                subject.compare_diagnostics(seed=123, replica_a_dir=a, replica_b_dir=b)

    def test_raw_hard_case_must_be_status_zero_at_max_iterations(self) -> None:
        with tempfile.TemporaryDirectory() as raw:
            fixture = DiagnosticFixture(Path(raw))
            a = fixture.make_replica("a")
            b = fixture.make_replica("b")
            journal_path = b / "solver_audit_journal.json"
            journal = json.loads(journal_path.read_text(encoding="utf-8"))
            bounded_path = journal[9]["control_windows"][0]["attempts"][0][
                "solver_path"
            ]
            bounded_path["bounded_lsq_status"] = 1
            fixture._write_json(journal_path, journal)
            with self.assertRaisesRegex(
                subject.V3DiagnosticComparisonError, "status=0/max-iter"
            ):
                subject.compare_diagnostics(seed=123, replica_a_dir=a, replica_b_dir=b)
        with tempfile.TemporaryDirectory() as raw:
            fixture = DiagnosticFixture(Path(raw))
            a = fixture.make_replica("a")
            b = fixture.make_replica("b", solver_marker=1)
            with self.assertRaisesRegex(subject.V3DiagnosticComparisonError, "solver"):
                subject.compare_diagnostics(seed=123, replica_a_dir=a, replica_b_dir=b)

    def test_npz_requires_exact_key_dtype_shape_and_finiteness(self) -> None:
        base = DiagnosticFixture.arrays
        for mutation in ("key", "dtype", "shape", "nonfinite"):
            with self.subTest(mutation=mutation), tempfile.TemporaryDirectory() as raw:
                fixture = DiagnosticFixture(Path(raw))
                a = fixture.make_replica("a")
                arrays = base(fixture)
                if mutation == "key":
                    arrays.pop("rewards")
                elif mutation == "dtype":
                    arrays["times"] = arrays["times"].astype(np.float32)
                elif mutation == "shape":
                    arrays["rewards"] = arrays["rewards"][:-1]
                else:
                    arrays["rewards"][0] = math.inf
                b = fixture.make_replica("b", arrays=arrays)
                with self.assertRaises(subject.V3DiagnosticComparisonError):
                    subject.compare_diagnostics(
                        seed=123, replica_a_dir=a, replica_b_dir=b
                    )

    def test_npz_float_payload_is_compared_bit_for_bit(self) -> None:
        with tempfile.TemporaryDirectory() as raw:
            fixture = DiagnosticFixture(Path(raw))
            arrays_a = fixture.arrays()
            arrays_b = fixture.arrays()
            arrays_a["rewards"][0] = 0.0
            arrays_b["rewards"][0] = -0.0
            a = fixture.make_replica("a", arrays=arrays_a)
            b = fixture.make_replica("b", arrays=arrays_b)
            with self.assertRaisesRegex(
                subject.V3DiagnosticComparisonError, "bit-exact"
            ):
                subject.compare_diagnostics(seed=123, replica_a_dir=a, replica_b_dir=b)

    def test_output_receipt_is_strict_finite_and_no_clobber(self) -> None:
        with tempfile.TemporaryDirectory() as raw:
            path = Path(raw) / "receipt.json"
            payload = {"passed": True, "value": 1.0}
            subject.write_json_exclusive(path, payload)
            self.assertEqual(subject.strict_json_load(path), payload)
            with self.assertRaises(subject.V3DiagnosticComparisonError):
                subject.write_json_exclusive(path, payload)
            with self.assertRaises(subject.V3DiagnosticComparisonError):
                subject.write_json_exclusive(Path(raw) / "nan.json", {"x": math.nan})

    def test_strict_json_rejects_duplicate_keys_and_nonfinite_tokens(self) -> None:
        with tempfile.TemporaryDirectory() as raw:
            duplicate = Path(raw) / "duplicate.json"
            duplicate.write_text('{"x": 1, "x": 2}', encoding="utf-8")
            nonfinite = Path(raw) / "nonfinite.json"
            nonfinite.write_text('{"x": NaN}', encoding="utf-8")
            for path in (duplicate, nonfinite):
                with (
                    self.subTest(path=path.name),
                    self.assertRaises(subject.V3DiagnosticComparisonError),
                ):
                    subject.strict_json_load(path)


if __name__ == "__main__":
    unittest.main()
