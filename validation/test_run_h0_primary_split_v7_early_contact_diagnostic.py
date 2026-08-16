from __future__ import annotations

import ast
import copy
import sys
import unittest
from pathlib import Path
from unittest import mock

import numpy as np


VALIDATION_ROOT = Path(__file__).resolve().parent
REPO_ROOT = VALIDATION_ROOT.parent
if str(VALIDATION_ROOT) not in sys.path:
    sys.path.insert(0, str(VALIDATION_ROOT))

import build_h0_primary_split_v7_early_contact_preflight as preflight  # noqa: E402
import freeze_h0_primary_split_v7_early_contact as freezer  # noqa: E402
import h0_forensic_rollout as forensic  # noqa: E402
import h0_primary_split_v7_early_contact_contract as contract  # noqa: E402
import run_h0_primary_split_v7_early_contact_diagnostic as runner  # noqa: E402


class _FakePrimaryRunner:
    def __init__(self, *, fail: bool = False) -> None:
        self.fail = fail
        self.calls: list[tuple[object, float]] = []
        self.grf = {
            "sides": {
                "left": {
                    "normal_force": 42.0,
                    "in_contact": True,
                }
            }
        }
        self.events: list[dict] = []
        self.result = (self.grf, self.events)

    def _sample_online_grf(self, state: object, time_s: float):
        self.calls.append((state, time_s))
        if self.fail:
            raise RuntimeError("synthetic primary failure")
        return self.result


class V7EarlyContactPreflightTests(unittest.TestCase):
    def test_resolve_relative_rejects_absolute_and_parent_paths(self) -> None:
        for value in ("/tmp/escape", "../escape", "validation/../../escape"):
            with self.subTest(value=value), self.assertRaises(
                preflight.V7EarlyContactPreflightError
            ):
                preflight.resolve_relative(value)

    def test_v5_nominal_is_exact_frozen_development_input(self) -> None:
        result = preflight.validate_v5_nominal()
        self.assertEqual(result["rows"], contract.EXPECTED_STEPS)
        self.assertEqual(result["actor_input_view"], "historical_analog")
        self.assertEqual(
            result["source_event_contract_id"],
            "primary_grf_split_v1+legacy_events_v1",
        )

    def test_v6_terminal_evidence_is_bound_without_retry(self) -> None:
        result = preflight.validate_v6_terminal()
        self.assertEqual(
            result["status"],
            "FAIL_H0_V6_V25_TEACHER_REPLAY_DEVELOPMENT",
        )
        self.assertTrue(result["terminal_failure_preserved"])
        self.assertFalse(result["retry_authorized"])
        self.assertEqual(result["early_onset_contact_state"], "TOE")
        self.assertAlmostEqual(
            result["observed_swing_s"],
            contract.V6_OBSERVED_SWING_S,
            places=12,
        )

    def test_v6_live_source_closure_fails_on_any_byte_record_drift(self) -> None:
        lock = preflight._mapping(
            preflight.input_paths()["v6_execution_lock"]
        )
        fake_record = {
            "path": "validation/h0_forensic_rollout.py",
            "sha256": "f" * 64,
            "size_bytes": 1,
        }
        with mock.patch.object(
            preflight,
            "source_record",
            return_value=fake_record,
        ):
            with self.assertRaises(preflight.V7EarlyContactPreflightError):
                preflight.validate_v6_live_source_closure(lock)

    def test_runtime_inputs_freeze_shadow_and_primary_role(self) -> None:
        result = preflight.validate_frozen_runtime_inputs()
        self.assertEqual(result["binary_phase_fsm_mode"], "binary_shadow")
        self.assertEqual(
            result["primary_load_evidence_role"],
            contract.PRIMARY_LOAD_EVIDENCE_ROLE,
        )
        self.assertFalse(result["primary_online_grf_used_as_event_source"])
        self.assertEqual(
            result["canonical_scientific_oracle"],
            contract.CANONICAL_SCIENTIFIC_ORACLE,
        )

    def test_transitive_runtime_platform_and_primary_closures_are_live(self) -> None:
        closure = preflight.validate_v3_runtime_closure()
        self.assertEqual(closure["runtime_source_count"], 36)
        self.assertEqual(closure["runtime_input_count"], 21)
        self.assertEqual(
            set(closure["v3_to_v7_source_drift"]),
            {"actor_fit", "binary_phase_adapter", "phase_fsm", "training_config"},
        )
        self.assertTrue(closure["runtime_inputs_v3_byte_exact"])
        platform = preflight.validate_platform_identity()
        self.assertEqual(platform["identity"], contract.EXPECTED_PLATFORM_IDENTITY)
        self.assertEqual(len(platform["binary_artifacts"]), 2)
        primary = preflight.validate_primary_core()
        self.assertEqual(len(primary["scientific_core"]), 6)
        self.assertEqual(
            primary["macos_arm64_dylib"]["sha256"],
            "5597a59a5368825fd207753b59291e72240c1ece3aa5280804f1e9b9c7d6a2b3",
        )

    def test_wrong_live_package_version_fails_platform_attestation(self) -> None:
        real_version = preflight.importlib.metadata.version

        def mutated(name: str) -> str:
            return "0.0" if name == "scipy" else real_version(name)

        with mock.patch.object(
            preflight.importlib.metadata,
            "version",
            side_effect=mutated,
        ), self.assertRaises(preflight.V7EarlyContactPreflightError):
            preflight.validate_platform_identity()

    def test_dry_preflight_is_pass_and_materializes_nothing(self) -> None:
        before = {
            path: path.exists()
            for path in (
                preflight.PREFLIGHT_PATH,
                preflight.LOCK_PATH,
                preflight.RUN_ROOT,
            )
        }
        result = preflight.build_payload(require_destinations_absent=True)
        self.assertTrue(result["passed"])
        self.assertTrue(all(result["checks"].values()))
        self.assertEqual(result["simulations_executed"], 0)
        self.assertEqual(result["actor_updates"], 0)
        self.assertEqual(result["critic_updates"], 0)
        self.assertEqual(result["ppo_updates"], 0)
        self.assertEqual(result["protected_trials_opened"], [])
        self.assertEqual(result["reserve_trials_opened"], [])
        after = {path: path.exists() for path in before}
        self.assertEqual(after, before)


class V7EarlyContactFreezeTests(unittest.TestCase):
    def test_lock_payload_preserves_evidence_roles_and_zero_updates(self) -> None:
        fake_receipt = {
            "sources": {"source": {"path": "x", "sha256": "0" * 64, "size_bytes": 1}},
            "inputs": {"input": {"path": "y", "sha256": "1" * 64, "size_bytes": 1}},
            "v5_nominal": {"rows": 500},
            "v6_terminal": {"status": "terminal"},
            "frozen_runtime": {"binary_phase_fsm_mode": "binary_shadow"},
            "runtime_closure": {"runtime_source_count": 36},
            "platform": copy.deepcopy(contract.EXPECTED_PLATFORM_IDENTITY),
            "platform_attestation": {"numerical_claim": "macOS-arm64-only"},
            "primary_core": {"scientific_core": {}},
        }
        preflight_record = {
            "path": contract.PREFLIGHT_RECEIPT_PATH.as_posix(),
            "sha256": "2" * 64,
            "size_bytes": 1,
        }
        with mock.patch.object(
            freezer, "verify_preflight", return_value=fake_receipt
        ), mock.patch.object(
            freezer.preflight,
            "source_record",
            return_value=preflight_record,
        ):
            payload = freezer.build_payload()
        self.assertEqual(payload["simulations_executed_at_freeze"], 0)
        self.assertFalse(payload["candidate_implemented"])
        self.assertFalse(payload["runtime_promoted"])
        self.assertEqual(payload["actor_updates"], 0)
        self.assertEqual(payload["critic_updates"], 0)
        self.assertEqual(payload["ppo_updates"], 0)
        self.assertEqual(payload["protected_trials_opened"], [])
        self.assertEqual(payload["reserve_trials_opened"], [])
        self.assertEqual(payload["runtime_closure"], fake_receipt["runtime_closure"])
        diagnostic = payload["diagnostic_contract"]
        self.assertEqual(
            diagnostic["primary_load_evidence_role"],
            contract.PRIMARY_LOAD_EVIDENCE_ROLE,
        )
        self.assertFalse(diagnostic["primary_online_grf_used_as_event_source"])
        self.assertEqual(
            diagnostic["canonical_scientific_oracle"],
            contract.CANONICAL_SCIENTIFIC_ORACLE,
        )

    def test_freeze_refuses_existing_lock_without_writing(self) -> None:
        with mock.patch.object(freezer.os.path, "lexists", return_value=True):
            with self.assertRaises(freezer.V7EarlyContactFreezeError):
                freezer.freeze()


class V7EarlyContactRunnerTests(unittest.TestCase):
    def test_canonical_lock_rejects_each_mutable_governance_block(self) -> None:
        receipt = {
            "sources": {},
            "inputs": {},
            "v5_nominal": {"rows": contract.EXPECTED_STEPS},
            "v6_terminal": {"status": "terminal"},
            "frozen_runtime": {"binary_phase_fsm_mode": "binary_shadow"},
            "runtime_closure": {"runtime_source_count": 36},
            "platform": copy.deepcopy(contract.EXPECTED_PLATFORM_IDENTITY),
            "platform_attestation": {"numerical_claim": "macOS-arm64-only"},
            "primary_core": {"scientific_core": {}},
        }
        preflight_record = {
            "path": contract.PREFLIGHT_RECEIPT_PATH.as_posix(),
            "sha256": "2" * 64,
            "size_bytes": 1,
        }
        canonical = freezer.build_payload_from_receipt(
            receipt,
            preflight_receipt_record=preflight_record,
        )
        runner._verify_canonical_lock_payload(
            canonical,
            receipt,
            preflight_receipt_record=preflight_record,
        )
        mutations = {
            "destination": lambda value: value.__setitem__(
                "destination", "validation/redirected"
            ),
            "matrix": lambda value: value["matrix"].__setitem__("steps", 499),
            "diagnostic_contract": lambda value: value[
                "diagnostic_contract"
            ].__setitem__("primary_force_threshold_n", 19.0),
            "gate": lambda value: value["gate"].__setitem__(
                "expected_primary_load_samples", 500
            ),
            "next_stage": lambda value: value.__setitem__(
                "next_stage", "UNAUTHORIZED_STAGE"
            ),
        }
        for name, mutate in mutations.items():
            with self.subTest(name=name):
                changed = copy.deepcopy(canonical)
                mutate(changed)
                with self.assertRaises(runner.V7EarlyContactExecutionError):
                    runner._verify_canonical_lock_payload(
                        changed,
                        receipt,
                        preflight_receipt_record=preflight_record,
                    )

    def test_transitive_runtime_closure_mutation_is_rejected(self) -> None:
        closure = preflight.validate_v3_runtime_closure()
        lock = {"runtime_closure": copy.deepcopy(closure)}
        receipt = {"runtime_closure": copy.deepcopy(closure)}
        mutated = copy.deepcopy(closure)
        first = next(iter(mutated["runtime_sources"]))
        mutated["runtime_sources"][first]["sha256"] = "0" * 64
        with mock.patch.object(
            runner.preflight,
            "validate_v3_runtime_closure",
            return_value=mutated,
        ), self.assertRaises(runner.V7EarlyContactExecutionError):
            runner._verify_transitive_runtime_closure(lock, receipt)

    def test_environment_is_shadow_only_with_legacy_actor(self) -> None:
        result = runner.build_env_config()
        self.assertEqual(result["binary_phase_fsm_mode"], "binary_shadow")
        self.assertEqual(
            result["binary_phase_event_contract_id"],
            contract.SHADOW_EVENT_CONTRACT_ID,
        )
        self.assertEqual(result["phase_fsm_input_mode"], "legacy_events")
        self.assertEqual(result["event_contract_id"], "legacy_events_v1")
        self.assertEqual(result["online_grf_applied_sides"], ["left"])
        self.assertEqual(result["reward"]["morphology_weight"], 0.0)

    def test_primary_tap_forwards_original_result_and_delivers_ten_samples(self) -> None:
        fake = _FakePrimaryRunner()
        body_weight_n = 686.7
        tap = runner._PrimaryGRFObservationTap(
            fake,
            body_weight_n=body_weight_n,
        )
        original_func = fake._sample_online_grf.__func__
        tap.install()
        state = object()
        for sample_index in range(contract.EXPECTED_PRIMARY_SAMPLES_PER_STEP):
            time_s = (
                contract.EPISODE_START_TIME_S
                + sample_index * contract.EXPECTED_SAMPLE_DT_S
            )
            result = fake._sample_online_grf(state, time_s)
            self.assertIs(result, fake.result)
            self.assertIs(result[0], fake.grf)
            self.assertIs(result[1], fake.events)
        delivered_time_s = (
            contract.EPISODE_START_TIME_S + contract.EXPECTED_POLICY_DT_S
        )
        delivered = tap.deliver_step(
            start_index=0,
            delivered_time_s=delivered_time_s,
        )
        self.assertEqual(
            len(delivered), contract.EXPECTED_PRIMARY_SAMPLES_PER_STEP
        )
        self.assertTrue(
            all(row["delivered_time_s"] == delivered_time_s for row in delivered)
        )
        self.assertEqual(delivered[0]["sampled_time_s"], contract.EPISODE_START_TIME_S)
        self.assertAlmostEqual(delivered[0]["left_normal_force_n"], 42.0)
        self.assertAlmostEqual(
            delivered[0]["left_normal_grf_bw"], 42.0 / body_weight_n
        )
        audit = tap.restore()
        self.assertEqual(audit["wrapper_call_count"], 10)
        self.assertEqual(audit["original_call_count"], 10)
        self.assertTrue(audit["one_original_call_per_wrapper_call"])
        self.assertTrue(audit["return_forwarded_unmodified"])
        self.assertTrue(audit["descriptor_identity_restored"])
        self.assertIs(fake._sample_online_grf.__func__, original_func)
        self.assertEqual(len(fake.calls), 10)

    def test_primary_tap_rejects_incomplete_step_and_restores_after_exception(self) -> None:
        incomplete = _FakePrimaryRunner()
        tap = runner._PrimaryGRFObservationTap(incomplete, body_weight_n=700.0)
        tap.install()
        for sample_index in range(9):
            incomplete._sample_online_grf(
                object(),
                contract.EPISODE_START_TIME_S
                + sample_index * contract.EXPECTED_SAMPLE_DT_S,
            )
        with self.assertRaises(runner.V7EarlyContactExecutionError):
            tap.deliver_step(
                start_index=0,
                delivered_time_s=(
                    contract.EPISODE_START_TIME_S
                    + contract.EXPECTED_POLICY_DT_S
                ),
            )
        tap.restore()

        failing = _FakePrimaryRunner(fail=True)
        failing_tap = runner._PrimaryGRFObservationTap(
            failing,
            body_weight_n=700.0,
        )
        original_func = failing._sample_online_grf.__func__
        failing_tap.install()
        try:
            with self.assertRaisesRegex(RuntimeError, "synthetic primary failure"):
                failing._sample_online_grf(object(), contract.EPISODE_START_TIME_S)
        finally:
            audit = failing_tap.restore()
        self.assertEqual(audit["wrapper_call_count"], 1)
        self.assertEqual(audit["original_call_count"], 0)
        self.assertFalse(audit["one_original_call_per_wrapper_call"])
        self.assertIs(failing._sample_online_grf.__func__, original_func)

    def test_primary_tap_receipt_is_audited_exactly(self) -> None:
        receipt = {
            "instrumentation_id": "primary_grf_existing_call_observer_v1",
            "installed_after_reset": True,
            "reset_call_count": 0,
            "wrapper_call_count": contract.EXPECTED_PRIMARY_LOAD_SAMPLES,
            "original_call_count": contract.EXPECTED_PRIMARY_LOAD_SAMPLES,
            "one_original_call_per_wrapper_call": True,
            "second_primary_evaluations": 0,
            "return_forwarded_unmodified": True,
            "restored_in_finally": True,
            "descriptor_identity_restored": True,
        }
        self.assertEqual(runner._validated_primary_tap_audit(receipt), receipt)
        for key in receipt:
            with self.subTest(key=key):
                mutated = copy.deepcopy(receipt)
                mutated[key] = None
                with self.assertRaises(runner.V7EarlyContactExecutionError):
                    runner._validated_primary_tap_audit(mutated)

    def test_projected_row_matches_exact_v5_schema(self) -> None:
        baseline = forensic.strict_json_load(runner.BASELINE_TRACE)[0]
        row = runner._projected_v5_row(
            step=baseline["step"],
            time_s=baseline["time_s"],
            actor_observation=np.asarray(
                baseline["actor_observation"], dtype=np.float32
            ),
            mean_action=np.asarray(baseline["mean_action"], dtype=np.float32),
            raw_action=np.asarray(baseline["raw_action"], dtype=np.float32),
            reward=baseline["reward"],
            terminated=baseline["terminated"],
            truncated=baseline["truncated"],
        )
        self.assertEqual(
            forensic.canonical_json_bytes(row),
            forensic.canonical_json_bytes(baseline),
        )

    def test_float32_exact_comparison_detects_one_ulp(self) -> None:
        left = np.asarray([0.1, 0.2], dtype=np.float32)
        right = left.copy()
        exact, _left_sha, _right_sha = runner.exact_float32_vector(
            left,
            right,
            length=2,
            np=np,
        )
        self.assertTrue(exact)
        right[1] = np.nextafter(right[1], np.float32(np.inf))
        exact, _left_sha, _right_sha = runner.exact_float32_vector(
            left,
            right,
            length=2,
            np=np,
        )
        self.assertFalse(exact)

    def test_teacher_std_digest_is_bound_to_h0_sigma(self) -> None:
        expected = np.asarray(
            [contract.EXPECTED_H0_POLICY_STD] * contract.EXPECTED_ACTION_DIM,
            dtype=np.float32,
        )
        self.assertEqual(
            runner.array_sha256(expected),
            contract.EXPECTED_H0_POLICY_STD_ARRAY_SHA256,
        )
        expected[0] = np.nextafter(expected[0], np.float32(np.inf))
        self.assertNotEqual(
            runner.array_sha256(expected),
            contract.EXPECTED_H0_POLICY_STD_ARRAY_SHA256,
        )

    def test_worker_requires_supervisor_claim_and_canonical_destination(self) -> None:
        with mock.patch.object(runner, "verify_lock", return_value={}), mock.patch.object(
            runner,
            "verify_worker_claim",
            side_effect=runner.V7EarlyContactExecutionError("missing claim"),
        ):
            with self.assertRaises(runner.V7EarlyContactExecutionError):
                runner.run_worker(
                    output_dir=runner.DESTINATION,
                    execution_token="x" * 40,
                )

    def test_source_contains_no_training_or_binary_active_execution(self) -> None:
        source = Path(runner.__file__).read_text(encoding="utf-8")
        tree = ast.parse(source)
        calls = [
            node
            for node in ast.walk(tree)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and node.func.attr == "train"
        ]
        self.assertEqual(calls, [])
        self.assertNotIn('case_id="C"', source)
        self.assertIn('case_id="B"', source)


if __name__ == "__main__":
    unittest.main()
