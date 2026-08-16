from __future__ import annotations

import json
import math
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import numpy as np

from validation import run_h0_primary_grf_split_v3_semantic_replay as v3


class HistoricalInputTests(unittest.TestCase):
    def test_training_tapes_have_exact_contract(self) -> None:
        for seed in v3.TRAIN_SEEDS:
            trace, summary, trace_path, summary_path = v3.historical_inputs(seed)
            self.assertEqual(len(trace), v3.EXPECTED_STEPS)
            self.assertEqual(summary["steps"], v3.EXPECTED_STEPS)
            self.assertEqual(trace[0]["step"], 1)
            self.assertEqual(trace[-1]["step"], v3.EXPECTED_STEPS)
            self.assertTrue(trace_path.is_file())
            self.assertTrue(summary_path.is_file())

    def test_unregistered_seed_fails_closed(self) -> None:
        with self.assertRaises(v3.H0PrimarySplitV3Error):
            v3.historical_inputs(126)

    def test_holdout_diagnostic_is_inaccessible_before_freeze(self) -> None:
        with tempfile.TemporaryDirectory() as raw:
            with (
                mock.patch.object(
                    v3, "verify_lock", side_effect=AssertionError("lock opened")
                ),
                mock.patch.object(
                    v3,
                    "historical_inputs",
                    side_effect=AssertionError("holdout opened"),
                ),
            ):
                with self.assertRaisesRegex(
                    v3.H0PrimarySplitV3Error,
                    "diagnostic replay is restricted",
                ):
                    v3.run_replay(
                        seed=v3.FINAL_HOLDOUT_SEED,
                        output_dir=Path(raw) / "holdout-diagnostic",
                        diagnostic=True,
                    )


class EnvironmentContractTests(unittest.TestCase):
    def test_canonical_primary_split_overrides(self) -> None:
        config = v3.build_env_config(seed=123)
        self.assertEqual(config["event_contract_id"], v3.EVENT_CONTRACT)
        self.assertEqual(config["online_grf_applied_sides"], ["left"])
        self.assertEqual(config["binary_phase_fsm_mode"], "disabled")
        self.assertIsNone(config["binary_phase_detector_profile_file"])
        self.assertEqual(config["phase_fsm_input_mode"], "legacy_events")
        self.assertEqual(config["reward"]["morphology_weight"], 0.0)
        self.assertFalse(config["record_outputs"])
        self.assertFalse(config["save_outputs_on_close"])


class ReplayGateTests(unittest.TestCase):
    def passing_summary(self) -> dict:
        return {
            "steps": 500,
            "end_reason": "episode_time_limit",
            "terminated": False,
            "truncated": True,
            "phase_valid_cycle_count": 3,
            "grf_penetration_max_m": 0.024,
            "action_clipped_values": 0,
            "timeout_count": 0,
            "safety_stop_count": 0,
            "fallback_count": 0,
            "so_solver_hard_fallback_count": 0,
            "so_solver_bounded_ls_invocation_count": 0,
            "so_solver_verified_bounded_ls_count": 0,
            "so_solver_verified_bounded_ls_success_count": 0,
            "so_solver_verified_status0_max_iter_count": 0,
            "so_solver_unaccepted_hard_fallback_count": 0,
            "so_solver_unaccepted_bounded_ls_count": 0,
            "so_solver_reuse_previous_count": 0,
            "so_solver_bounded_ls_unsuccessful_count": 0,
            "so_solver_bounds_violation_count": 0,
            "so_solver_nonfinite_count": 0,
            "so_solver_selected_infeasible_count": 0,
            "so_solver_selected_solution_mismatch_count": 0,
            "so_solver_residual_contract_mismatch_count": 0,
            "hard_invalid_count": 0,
            "nonfinite_count": 0,
            "invalid_event_count": 1,
            "historical_invalid_event_count": 1,
            "fixed_feature_mismatch_count": 0,
            "teacher_view_mismatch_count": 0,
            "time_mismatch_count": 0,
            "mutable_feature_difference_count": 499,
            "n_actor": 35,
            "n_observation": 84,
            "observation_dtype": "float32",
            "online_grf_applied_sides": ["left"],
            "binary_phase_fsm_mode": "disabled",
            "event_contract_id": v3.EVENT_CONTRACT,
            "morphology_weight": 0.0,
            "h0_used_for_behavior": False,
            "ppo_updates": 0,
        }

    def test_complete_replay_passes(self) -> None:
        gate = v3._replay_gate(self.passing_summary())
        self.assertTrue(gate["passed"])
        self.assertEqual(gate["status"], "PASS_H0_PRIMARY_SPLIT_V3_REPLAY")

    def test_any_fallback_fails_closed(self) -> None:
        summary = self.passing_summary()
        summary["fallback_count"] = 1
        gate = v3._replay_gate(summary)
        self.assertFalse(gate["passed"])
        self.assertFalse(gate["checks"]["zero_fallbacks"])

    def test_closed_status0_policy_accepts_only_fully_verified_raw_counts(self) -> None:
        summary = self.passing_summary()
        summary.update(
            {
                "so_policy_id": v3.freeze_contract.VERIFIED_STATUS0_MAX_ITER_POLICY,
                "fallback_count": 61,
                "so_fallback_count": 61,
                "sea_plugin_fallback_count": 0,
                "so_solver_hard_fallback_count": 352,
                "so_solver_bounded_ls_invocation_count": 596,
                "so_solver_verified_bounded_ls_count": 596,
                "so_solver_verified_bounded_ls_success_count": 244,
                "so_solver_verified_status0_max_iter_count": 352,
                "so_solver_unaccepted_hard_fallback_count": 0,
                "so_solver_unaccepted_bounded_ls_count": 0,
                "so_solver_bounded_ls_unsuccessful_count": 352,
            }
        )
        gate = v3._replay_gate(summary)
        self.assertTrue(gate["passed"])
        self.assertEqual(
            gate["so_policy_id"],
            v3.freeze_contract.VERIFIED_STATUS0_MAX_ITER_POLICY,
        )

        summary["so_solver_unaccepted_bounded_ls_count"] = 1
        self.assertFalse(v3._replay_gate(summary)["passed"])

    def test_phase_or_teacher_scope_widening_fails_closed(self) -> None:
        for field in ("fixed_feature_mismatch_count", "teacher_view_mismatch_count"):
            with self.subTest(field=field):
                summary = self.passing_summary()
                summary[field] = 1
                self.assertFalse(v3._replay_gate(summary)["passed"])

    def test_penetration_limit_is_strict(self) -> None:
        summary = self.passing_summary()
        summary["grf_penetration_max_m"] = 0.025
        self.assertFalse(v3._replay_gate(summary)["passed"])

    def test_gate_rejects_coercible_types_and_nonfinite_values(self) -> None:
        mutations = {
            "steps": True,
            "fallback_count": 0.0,
            "terminated": 0,
            "truncated": 1,
            "morphology_weight": "0",
            "grf_penetration_max_m": math.nan,
        }
        for field, value in mutations.items():
            with self.subTest(field=field, value=value):
                summary = self.passing_summary()
                summary[field] = value
                self.assertFalse(v3._replay_gate(summary)["passed"])


class StrictRuntimeContractTests(unittest.TestCase):
    @staticmethod
    def solver_audit() -> list[dict]:
        digest = "a" * 64
        solver_path = {
            "schema": "static_optimization_solver_audit_v1",
            "input_matrix_finite": True,
            "input_target_finite": True,
            "weights_finite": True,
            "bounds_finite": True,
            "warm_start_finite": True,
            "bounded_lsq_used": False,
            "reuse_previous_solution": False,
            "hard_fallback": False,
            "selected_solution": {
                "output_shape_matches": True,
                "output_finite": True,
                "output_sha256": digest,
                "equality_residual_finite": True,
                "equality_residual_norm": 0.0,
            },
        }
        return [
            {
                "control_window_index": 1,
                "control_window_time_s": 1.0,
                "selected_feasibility_attempt_index": 1,
                "served_solution_sha256": digest,
                "selected_solver_solution_matches_served": True,
                "attempts": [
                    {
                        "attempt_index": 1,
                        "feasibility_scale": 1.0,
                        "feasibility_accepted": True,
                        "residual_norm": 0.0,
                        "residual_relative_norm": 0.0,
                        "residual_max_abs": 0.0,
                        "solver_fallback_used": False,
                        "solver_path": solver_path,
                        "selected": True,
                    }
                ],
            }
        ]

    def test_bool_and_counter_parsing_is_exact(self) -> None:
        for value in (False, "0", 0.0, -1, math.nan, math.inf):
            with self.subTest(value=value):
                with self.assertRaises(v3.H0PrimarySplitV3Error):
                    v3._require_counter(value, "counter")
        for value in (0, 1, "false", np.bool_(False)):
            with self.subTest(value=value):
                with self.assertRaises(v3.H0PrimarySplitV3Error):
                    v3._require_bool(value, "flag")

    def test_solver_audit_rejects_coercible_fallback_bool(self) -> None:
        valid = self.solver_audit()
        normalized, attempts, hard = v3._validate_so_solver_audit_entries(
            valid,
            step_index=1,
            selected_fallback=False,
        )
        self.assertEqual(len(normalized), 1)
        self.assertEqual(attempts, 1)
        self.assertEqual(hard, 0)
        malformed = self.solver_audit()
        malformed[0]["attempts"][0]["solver_fallback_used"] = 0
        with self.assertRaises(v3.H0PrimarySplitV3Error):
            v3._validate_so_solver_audit_entries(
                malformed,
                step_index=1,
                selected_fallback=False,
            )

    def test_array_dtype_is_checked_before_conversion(self) -> None:
        with self.assertRaises(v3.H0PrimarySplitV3Error):
            v3._require_array_contract(
                np.zeros((2,), dtype=np.float64),
                np=np,
                label="observation",
                dtype=np.float32,
                shape=(2,),
            )
        with self.assertRaises(v3.H0PrimarySplitV3Error):
            v3._require_array_contract(
                np.asarray([0.0, math.inf], dtype=np.float32),
                np=np,
                label="observation",
                dtype=np.float32,
                shape=(2,),
            )

    def test_json_vector_rejects_bool_and_nonfinite(self) -> None:
        for value in ([0.0, True], [0.0, math.inf], [0.0, "1"]):
            with self.subTest(value=value):
                with self.assertRaises(v3.H0PrimarySplitV3Error):
                    v3._require_finite_vector(value, 2, "vector")


class DestinationContractTests(unittest.TestCase):
    def test_diagnostics_are_disjoint_and_claims_are_one_shot(self) -> None:
        with tempfile.TemporaryDirectory() as raw:
            root = Path(raw)
            run_root = root / "protocol"
            outside = root / "diagnostic"
            with mock.patch.object(v3, "RUN_ROOT", run_root):
                with self.assertRaises(v3.H0PrimarySplitV3Error):
                    v3._claim_destination(
                        run_root / "diagnostic",
                        diagnostic=True,
                    )
                claimed = v3._claim_destination(outside, diagnostic=True)
                self.assertEqual(claimed, outside.resolve())
                with self.assertRaises(v3.H0PrimarySplitV3Error):
                    v3._claim_destination(outside, diagnostic=True)

    def test_protocol_destination_must_be_exact(self) -> None:
        with tempfile.TemporaryDirectory() as raw:
            root = Path(raw)
            canonical = root / "run" / "replay" / "seed_123"
            with mock.patch.object(v3, "RUN_ROOT", root / "run"):
                with self.assertRaises(v3.H0PrimarySplitV3Error):
                    v3._claim_destination(
                        root / "run" / "replay" / "typo",
                        diagnostic=False,
                        canonical_destination=canonical,
                    )


class ArtifactTests(unittest.TestCase):
    def test_npz_writer_is_no_clobber(self) -> None:
        with tempfile.TemporaryDirectory() as raw:
            destination = Path(raw) / "data.npz"
            with mock.patch.object(
                v3.os,
                "link",
                side_effect=AssertionError("hard links are forbidden"),
            ):
                v3._write_npz_exclusive(destination, values=np.arange(3))
            with np.load(destination, allow_pickle=False) as archive:
                np.testing.assert_array_equal(archive["values"], np.arange(3))
            with self.assertRaises(v3.H0PrimarySplitV3Error):
                v3._write_npz_exclusive(destination, values=np.arange(3))

    def test_json_writer_is_portable_and_no_clobber(self) -> None:
        with tempfile.TemporaryDirectory() as raw:
            destination = Path(raw) / "receipt.json"
            with mock.patch.object(
                v3.os,
                "link",
                side_effect=AssertionError("hard links are forbidden"),
            ):
                v3._write_json_exclusive(destination, {"passed": False})
            self.assertEqual(json.loads(destination.read_text()), {"passed": False})
            with self.assertRaises(v3.H0PrimarySplitV3Error):
                v3._write_json_exclusive(destination, {"passed": True})

    def test_source_record_uses_portable_repo_relative_path(self) -> None:
        record = v3.source_record(v3.TESTS)
        self.assertEqual(record["path"], "validation/test_h0_primary_grf_split_v3.py")
        self.assertNotIn("\\", record["path"])

    def test_frozen_record_rejects_noncanonical_path_and_types(self) -> None:
        canonical = v3.source_record(v3.TESTS)
        mutations = (
            {**canonical, "path": canonical["path"].replace("/", "\\")},
            {**canonical, "path": "validation/../AGENTS.md"},
            {**canonical, "size_bytes": True},
            {**canonical, "sha256": canonical["sha256"].upper()},
            {**canonical, "extra": 1},
        )
        for record in mutations:
            with self.subTest(record=record):
                with self.assertRaises(v3.H0PrimarySplitV3Error):
                    v3._verified_record_path(record, "record")

    def test_lock_rejects_noncanonical_schema_before_record_count(self) -> None:
        malformed = {key: None for key in v3.LOCK_TOP_LEVEL_KEYS}
        malformed["unexpected"] = None
        with mock.patch.object(v3, "_strict_mapping", return_value=malformed):
            with self.assertRaises(v3.H0PrimarySplitV3Error):
                v3.verify_lock()

    def test_lock_rejects_arbitrary_runtime_closure(self) -> None:
        histories = {}
        for seed in v3.CANONICAL_SEEDS:
            directory = v3._historical_directory(seed)
            histories[str(seed)] = {
                "trace": v3.source_record(directory / "rollout_policy_trace.json"),
                "summary": v3.source_record(directory / "rollout_summary.json"),
            }
        payload = {
            "schema_version": 3,
            "status": "H0_PRIMARY_GRF_SPLIT_V3_EXECUTION_FROZEN",
            "protocol_id": v3.PROTOCOL_ID,
            "revision": v3.REVISION,
            "candidate_id": v3.CANDIDATE_ID,
            "run_root": v3._repo_relative_posix(v3.RUN_ROOT),
            "expected_steps": v3.EXPECTED_STEPS,
            "canonical_seeds": list(v3.CANONICAL_SEEDS),
            "canonical_offset_s": v3.CANONICAL_OFFSET_S,
            "event_contract_id": v3.EVENT_CONTRACT,
            "so_policy_id": v3.freeze_contract.VERIFIED_STATUS0_MAX_ITER_POLICY,
            "so_policy": v3.freeze_contract.SO_POLICIES[
                v3.freeze_contract.VERIFIED_STATUS0_MAX_ITER_POLICY
            ],
            "fit": v3.FIT,
            "mutable_feature_names": list(v3.MUTABLE_FEATURE_NAMES),
            "destinations": [
                v3._repo_relative_posix(path)
                for path in v3._canonical_stage_destinations()
            ],
            "authority": v3.LOCK_AUTHORITY,
            "sources": {
                "protocol_plan": v3.source_record(v3.PLAN),
                "runner": v3.source_record(v3.RUNNER),
                "tests": v3.source_record(v3.TESTS),
            },
            "inputs": {
                "h0_config": v3.source_record(v3.H0_CONFIG),
                "h0_module_state": v3.source_record(v3.H0_MODULE / "module_state.pkl"),
                "h0_module_ctor": v3.source_record(
                    v3.H0_MODULE / "class_and_ctor_args.pkl"
                ),
                "h0_module_metadata": v3.source_record(v3.H0_MODULE / "metadata.json"),
                "historical_inputs": histories,
            },
            "runtime_closure": {
                "arbitrary_files": {
                    f"file_{index}": v3.source_record(v3.TESTS) for index in range(35)
                }
            },
            "actor_update_candidate_count": 1,
            "retry_or_retuning_allowed": False,
            "critic_updates": 0,
            "ppo_updates": 0,
            "protected_trials_opened": [],
        }
        with mock.patch.object(v3, "_strict_mapping", return_value=payload):
            with self.assertRaisesRegex(
                v3.H0PrimarySplitV3Error, "runtime closure schema"
            ):
                v3.verify_lock()

    def test_replay_receipt_rejects_diagnostic_provenance(self) -> None:
        receipt = {
            "schema_version": 3,
            "status": "PASS_H0_PRIMARY_SPLIT_V3_REPLAY",
            "passed": True,
            "seed": 123,
            "diagnostic": True,
            "execution_lock": None,
            "attempt_claim": None,
            "artifacts": {},
            "actor_updates": 0,
            "critic_updates": 0,
            "ppo_updates": 0,
            "protected_trials_opened": [],
        }
        with mock.patch.object(v3, "_strict_mapping", return_value=receipt):
            with self.assertRaises(v3.H0PrimarySplitV3Error):
                v3._validate_replay_receipt(123)

    def test_development_receipt_is_strict_finite_json(self) -> None:
        path = (
            v3.VALIDATION_ROOT
            / "h0_primary_grf_split_v3_development_preflight_receipt.json"
        )
        payload = json.loads(
            path.read_text(encoding="utf-8"),
            parse_constant=lambda token: (_ for _ in ()).throw(ValueError(token)),
        )

        def walk(value):
            if isinstance(value, float):
                self.assertTrue(math.isfinite(value))
            elif isinstance(value, dict):
                for child in value.values():
                    walk(child)
            elif isinstance(value, list):
                for child in value:
                    walk(child)

        walk(payload)
        self.assertFalse(payload["passed"])
        self.assertEqual(payload["actor_updates"], 0)
        self.assertEqual(payload["ppo_updates"], 0)
        self.assertEqual(payload["protected_trials_opened"], [])


if __name__ == "__main__":
    unittest.main()
