from __future__ import annotations

import ast
import json
import sys
import tempfile
import types
import unittest
from contextlib import ExitStack
from pathlib import Path
from unittest import mock

from validation import freeze_h0_primary_grf_split_v5_zero_update as freezer
from validation import h0_primary_grf_split_v5_zero_update_contract as contract
from validation import run_h0_primary_grf_split_v5_zero_update_port as driver


def _write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(payload, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )


class _Optimizer:
    def __init__(self, state: dict[object, object]) -> None:
        self._state = state

    def state_dict(self) -> dict[str, object]:
        return {"state": self._state, "param_groups": [{"params": []}]}


class _Learner:
    def __init__(self, optimizer: _Optimizer) -> None:
        self._optimizer = optimizer

    def get_optimizers_for_module(self, module_id: str):
        if module_id != contract.DEFAULT_POLICY_ID:
            raise AssertionError(module_id)
        return [("actor_critic", self._optimizer)]


class _Metrics:
    def __init__(self, values: dict[object, object] | None = None) -> None:
        self._values = values or {}

    def peek(self, key: object, *, default: object) -> object:
        return self._values.get(key, default)


class _Algorithm:
    def __init__(
        self,
        *,
        iteration: object = 0,
        metrics: _Metrics | None = None,
    ) -> None:
        self.iteration = iteration
        self.metrics = metrics or _Metrics()


class ZeroUpdatePortTests(unittest.TestCase):
    def test_contract_is_closed_and_uses_canonical_qualification_evidence(self) -> None:
        self.assertEqual(
            contract.PASS_STATUS,
            "PASS_H0_PRIMARY_SPLIT_V5_ZERO_UPDATE_PORT",
        )
        self.assertEqual(contract.TARGET_CONFIG_OVERRIDES["iterations"], 0)
        self.assertEqual(contract.TARGET_CONFIG_OVERRIDES["num_env_runners"], 0)
        self.assertFalse(contract.AUTHORITY["actor_gradient_updates_authorized"])
        self.assertFalse(contract.AUTHORITY["critic_updates_authorized"])
        self.assertFalse(contract.AUTHORITY["ppo_updates_authorized"])
        self.assertFalse(contract.AUTHORITY["environment_sampling_authorized"])
        self.assertFalse(contract.AUTHORITY["protected_trial_access_authorized"])
        self.assertIn(
            "qualification_execution_ledger", contract.INPUT_RELATIVE_PATHS
        )
        self.assertIn("v4_preexecution_failure", contract.INPUT_RELATIVE_PATHS)
        self.assertNotIn("qualification_receipt", contract.INPUT_RELATIVE_PATHS)
        for case_id in contract.QUALIFICATION_CASE_IDS:
            self.assertIn(
                f"qualification_gate_{case_id}", contract.INPUT_RELATIVE_PATHS
            )
        serialized = json.dumps(contract.INPUT_RELATIVE_PATHS, sort_keys=True)
        self.assertNotIn("h0_primary_split_v3_semantic_replay", serialized)
        for protected in ("trial_03", "trial_05", "trial_06", "trial_07"):
            self.assertNotIn(protected, serialized)

    def test_runtime_port_has_no_train_call(self) -> None:
        source_path = Path(driver.__file__).resolve()
        syntax = ast.parse(source_path.read_text(encoding="utf-8"))
        calls = [
            node
            for node in ast.walk(syntax)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and node.func.attr == "train"
        ]
        self.assertEqual(calls, [])
        self.assertNotIn("protected_trial", ast.get_source_segment(
            source_path.read_text(encoding="utf-8"),
            next(
                node
                for node in ast.walk(syntax)
                if isinstance(node, ast.FunctionDef) and node.name == "_runtime_port"
            ),
        ) or "")

    def test_json_publication_is_strict_and_no_clobber(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            destination = Path(temporary) / "receipt.json"
            driver.write_json_exclusive(destination, {"finite": 1.0})
            self.assertEqual(json.loads(destination.read_text()), {"finite": 1.0})
            with self.assertRaises(driver.ZeroUpdatePortError):
                driver.write_json_exclusive(destination, {"finite": 2.0})
            with self.assertRaises(driver.ZeroUpdatePortError):
                driver.write_json_exclusive(
                    Path(temporary) / "nonfinite.json", {"bad": float("nan")}
                )
            duplicate = Path(temporary) / "duplicate.json"
            duplicate.write_text('{"same": 1, "same": 2}\n', encoding="utf-8")
            with self.assertRaises(driver.ZeroUpdatePortError):
                driver._strict_json(duplicate)  # noqa: SLF001

    def test_optimizer_must_exist_and_have_empty_state(self) -> None:
        report = driver._optimizer_empty_on_learner(  # noqa: SLF001
            _Learner(_Optimizer({}))
        )
        self.assertEqual(report[0]["state_entry_count"], 0)
        self.assertTrue(report[0]["empty"])
        with self.assertRaises(driver.ZeroUpdatePortError):
            driver._optimizer_empty_on_learner(  # noqa: SLF001
                _Learner(_Optimizer({1: {"step": 1}}))
            )

    def test_fresh_critic_requires_actor_only_source_and_target_critic(self) -> None:
        class WarmStart:
            @staticmethod
            def compare_non_actor_states(
                expected: dict[str, object], actual: dict[str, object]
            ) -> dict[str, object]:
                actor_keys = {"actor"}
                expected_keys = sorted(set(expected) - actor_keys)
                actual_keys = sorted(set(actual) - actor_keys)
                missing = sorted(set(expected_keys) - set(actual_keys))
                unexpected = sorted(set(actual_keys) - set(expected_keys))
                exact = bool(expected_keys) and not missing and not unexpected
                return {
                    "keys": expected_keys,
                    "missing_keys": missing,
                    "unexpected_keys": unexpected,
                    "exact": exact,
                }

        report = driver.fresh_critic_audit(
            candidate_state={"actor": 1},
            learner_state={"actor": 2, "critic": 3},
            warm_start=WarmStart,
        )
        self.assertEqual(report["fresh_learner"]["keys"], ["critic"])
        with self.assertRaises(driver.ZeroUpdatePortError):
            driver.fresh_critic_audit(
                candidate_state={"actor": 1, "source_critic": 4},
                learner_state={"actor": 2, "critic": 3},
                warm_start=WarmStart,
            )
        with self.assertRaises(driver.ZeroUpdatePortError):
            driver.fresh_critic_audit(
                candidate_state={"actor": 1},
                learner_state={"actor": 2},
                warm_start=WarmStart,
            )

    def test_progress_counters_must_all_be_zero(self) -> None:
        metric_names = {
            "NUM_ENV_STEPS_SAMPLED_LIFETIME": "sampled_env",
            "NUM_AGENT_STEPS_SAMPLED_LIFETIME": "sampled_agent",
            "NUM_ENV_STEPS_TRAINED_LIFETIME": "trained_env",
            "NUM_AGENT_STEPS_TRAINED_LIFETIME": "trained_agent",
            "NUM_GRAD_UPDATES_LIFETIME": "grad_updates",
        }
        metrics_module = types.ModuleType("ray.rllib.utils.metrics")
        for name, value in metric_names.items():
            setattr(metrics_module, name, value)
        ray_module = types.ModuleType("ray")
        rllib_module = types.ModuleType("ray.rllib")
        utils_module = types.ModuleType("ray.rllib.utils")
        utils_module.metrics = metrics_module
        modules = {
            "ray": ray_module,
            "ray.rllib": rllib_module,
            "ray.rllib.utils": utils_module,
            "ray.rllib.utils.metrics": metrics_module,
        }
        with mock.patch.dict(sys.modules, modules):
            self.assertEqual(
                driver.zero_progress_audit(_Algorithm()),
                {name: 0 for name in contract.ZERO_COUNTER_NAMES},
            )
            with self.assertRaises(driver.ZeroUpdatePortError):
                driver.zero_progress_audit(
                    _Algorithm(metrics=_Metrics({"sampled_env": 1}))
                )
            with self.assertRaises(driver.ZeroUpdatePortError):
                driver.zero_progress_audit(_Algorithm(iteration=1))

    def _frozen_world(self) -> dict[str, object]:
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        root = Path(temporary.name).resolve()
        lock = root / "validation" / "zero_update_lock.json"
        output = root / "runs" / "zero_update_port"
        sources = {
            key: root / "sources" / f"{key}.py"
            for key in contract.SOURCE_RELATIVE_PATHS
        }
        inputs: dict[str, Path] = {}
        for key in contract.INPUT_RELATIVE_PATHS:
            suffix = ".pkl" if key in {
                "candidate_module_state",
                "candidate_module_ctor",
            } else ".json"
            inputs[key] = root / "inputs" / f"{key}{suffix}"

        for key, path in sources.items():
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(f"# frozen source {key}\n", encoding="utf-8")
        inputs["candidate_module_state"].parent.mkdir(parents=True, exist_ok=True)
        inputs["candidate_module_state"].write_bytes(b"qualified actor and critic")
        inputs["candidate_module_ctor"].write_bytes(b"constructor")
        _write_json(inputs["candidate_module_metadata"], {"metadata": 1})
        inputs["v5_training_config"].parent.mkdir(parents=True, exist_ok=True)
        inputs["v5_training_config"].write_text("iterations: 1\n", encoding="utf-8")

        historical_stub = {
            "path": "validation/historical_v4_artifact.json",
            "sha256": "b" * 64,
            "size_bytes": 1,
        }
        _write_json(
            inputs["v4_preexecution_failure"],
            {
                "schema_version": 1,
                "status": "FAIL_H0_PRIMARY_SPLIT_V4_PREEXECUTION_INTEGRITY",
                "passed": False,
                "protocol_id": "AB06_H0_PRIMARY_GRF_SPLIT_V4_FULL_MEAN",
                "failure": (
                    "post_lock_runner_drift_and_stale_preflight_removal"
                ),
                "failure_stage": "verify_lock_before_attempt_claim",
                "execution_error": (
                    "V4FreezeError: V4 preflight source drifted: runner"
                ),
                "execution_lock": historical_stub,
                "locked_preflight_receipt": historical_stub,
                "locked_runner": historical_stub,
                "observed_preflight_receipt_exists": False,
                "observed_runner": historical_stub,
                "v4_execution_started": False,
                "attempt_claim_created": False,
                "run_root_exists": False,
                "seed125_semantic_accessed": False,
                "actor_update_candidates": 0,
                "critic_updates": 0,
                "ppo_updates": 0,
                "protected_trials_opened": [],
                "v4_retry_allowed": False,
                "next_stage": "NEW_PROTOCOL_REQUIRED_NO_V4_RETRY",
            },
        )
        stack = ExitStack()
        self.addCleanup(stack.close)
        # The standalone freezer intentionally imports the standalone driver
        # name when executed as a script; unittest imports the package name.
        # Patch both module identities so the sandbox remains self-contained.
        for module in (driver, freezer.driver, freezer):
            stack.enter_context(mock.patch.object(module, "REPO_ROOT", root))
            stack.enter_context(mock.patch.object(module, "LOCK", lock))
            stack.enter_context(mock.patch.object(module, "OUTPUT_ROOT", output))
            stack.enter_context(mock.patch.object(module, "INPUT_PATHS", inputs))
            stack.enter_context(mock.patch.object(module, "SOURCE_PATHS", sources))
        stack.enter_context(
            mock.patch.object(
                driver, "CANDIDATE_DIR", inputs["candidate_module_state"].parent
            )
        )
        stack.enter_context(
            mock.patch.object(
                freezer.driver,
                "CANDIDATE_DIR",
                inputs["candidate_module_state"].parent,
            )
        )

        source_stub = driver.source_record(sources["driver"])
        v4_failure_record = driver.source_record(
            inputs["v4_preexecution_failure"]
        )

        actor_digest = "a" * 64
        _write_json(
            inputs["candidate_actor_manifest"],
            {
                "schema_version": 5,
                "candidate_id": contract.CANDIDATE_ID,
                "observation_contract_id": "primary_grf_split_v1",
                "event_contract_id": "legacy_events_v1",
                "trainable_scope": "full_mean_network",
                "logstd_policy": "frozen_bit_exact",
                "actor_feature_count": contract.EXPECTED_ACTOR_FEATURES,
                "actor_feature_names": [
                    f"actor_feature_{index:02d}"
                    for index in range(contract.EXPECTED_ACTOR_FEATURES)
                ],
                "actor_digest": actor_digest,
                "module_state_sha256": driver._sha256_file(  # noqa: SLF001
                    inputs["candidate_module_state"]
                ),
                "execution_lock": source_stub,
                "attempt_claim": source_stub,
                "v3_corpus": source_stub,
                "v3_terminal_ledger": source_stub,
                "v4_preexecution_failure": v4_failure_record,
                "source_h0": {
                    "state": source_stub,
                    "ctor": source_stub,
                    "metadata": source_stub,
                    "config": source_stub,
                },
            },
        )
        _write_json(
            inputs["candidate_receipt"],
            {
                "schema_version": 5,
                "status": "PASS_H0_PRIMARY_SPLIT_V5_OFFLINE",
                "passed": True,
                "candidate_module_state": driver.source_record(
                    inputs["candidate_module_state"]
                ),
                "candidate_module_ctor": driver.source_record(
                    inputs["candidate_module_ctor"]
                ),
                "candidate_module_metadata": driver.source_record(
                    inputs["candidate_module_metadata"]
                ),
                "actor_feature_manifest": driver.source_record(
                    inputs["candidate_actor_manifest"]
                ),
                "adaptation_report": source_stub,
                "offline_gate": source_stub,
                "execution_lock": source_stub,
                "attempt_claim": source_stub,
                "v3_corpus": source_stub,
                "v3_corpus_receipt": source_stub,
                "v3_corpus_manifest": source_stub,
                "v3_terminal_ledger": source_stub,
                "v4_preexecution_failure": v4_failure_record,
                "source_h0": {
                    "state": source_stub,
                    "ctor": source_stub,
                    "metadata": source_stub,
                    "config": source_stub,
                },
                "actor_updates": 1,
                "critic_updates": 0,
                "ppo_updates": 0,
                "protected_trials_opened": [],
            },
        )
        _write_json(
            inputs["candidate_offline_gate"],
            {
                "schema_version": 5,
                "status": "PASS_H0_PRIMARY_SPLIT_V5_OFFLINE",
                "passed": True,
                "candidate_actor_digest": actor_digest,
                "v4_preexecution_failure": v4_failure_record,
                "actor_updates": 1,
                "critic_updates": 0,
                "ppo_updates": 0,
                "protected_trials_opened": [],
            },
        )
        candidate_freeze = {
            "schema_version": 5,
            "status": "H0_PRIMARY_SPLIT_V5_CANDIDATE_FROZEN_BEFORE_HOLDOUT",
            "protocol_id": contract.SOURCE_PROTOCOL_ID,
            "candidate_id": contract.CANDIDATE_ID,
            "train_seeds": [123, 124],
            "final_holdout_seed": 125,
            "fit": {},
            "trainable_scope": "full_mean_network",
            "logstd_policy": "frozen_bit_exact",
            "candidate_receipt": driver.source_record(inputs["candidate_receipt"]),
            "candidate_offline_gate": driver.source_record(
                inputs["candidate_offline_gate"]
            ),
            "candidate_module_state": driver.source_record(
                inputs["candidate_module_state"]
            ),
            "candidate_module_ctor": driver.source_record(
                inputs["candidate_module_ctor"]
            ),
            "candidate_module_metadata": driver.source_record(
                inputs["candidate_module_metadata"]
            ),
            "actor_feature_manifest": driver.source_record(
                inputs["candidate_actor_manifest"]
            ),
            "candidate_actor_digest": actor_digest,
            "v3_terminal_ledger": driver.source_record(sources["driver"]),
            "v4_preexecution_failure": v4_failure_record,
            "v3_corpus": driver.source_record(inputs["candidate_module_state"]),
            "holdout_accessed_before_freeze": False,
            "actor_updates": 1,
            "critic_updates": 0,
            "ppo_updates": 0,
            "protected_trials_opened": [],
        }
        _write_json(inputs["candidate_freeze"], candidate_freeze)
        _write_json(
            inputs["holdout_replay_receipt"],
            {
                "schema_version": 5,
                "status": "PASS_H0_PRIMARY_SPLIT_V5_REPLAY",
                "passed": True,
                "seed": 125,
                "additional_actor_updates": 0,
                "v4_preexecution_failure": v4_failure_record,
                "actor_updates": 1,
                "critic_updates": 0,
                "ppo_updates": 0,
                "protected_trials_opened": [],
            },
        )
        _write_json(
            inputs["holdout_gate"],
            {
                "schema_version": 5,
                "status": "PASS_H0_PRIMARY_SPLIT_V5_FINAL_HOLDOUT",
                "passed": True,
                "additional_actor_updates": 0,
                "v4_preexecution_failure": v4_failure_record,
                "actor_updates": 1,
                "critic_updates": 0,
                "ppo_updates": 0,
                "protected_trials_opened": [],
            },
        )
        _write_json(
            inputs["holdout_receipt"],
            {
                "schema_version": 5,
                "status": "PASS_H0_PRIMARY_SPLIT_V5_FINAL_HOLDOUT",
                "passed": True,
                "gate": driver.source_record(inputs["holdout_gate"]),
                "candidate_freeze": driver.source_record(inputs["candidate_freeze"]),
                "holdout_access_claim": {},
                "holdout_replay_receipt": driver.source_record(
                    inputs["holdout_replay_receipt"]
                ),
                "v4_preexecution_failure": v4_failure_record,
                "actor_updates": 1,
                "additional_actor_updates": 0,
                "critic_updates": 0,
                "ppo_updates": 0,
                "protected_trials_opened": [],
            },
        )
        _write_json(
            inputs["v5_execution_lock"],
            {
                "schema_version": 5,
                "status": "H0_PRIMARY_GRF_SPLIT_V5_EXECUTION_FROZEN",
                "protocol_id": contract.SOURCE_PROTOCOL_ID,
                "candidate_id": contract.CANDIDATE_ID,
                "so_policy_id": contract.SO_POLICY_ID,
                "event_contract_id": contract.EVENT_CONTRACT_ID,
                "trainable_scope": "full_mean_network",
                "logstd_policy": "frozen_bit_exact",
                "authority": {
                    "full_mean_network_update": True,
                    "logstd_update": False,
                    "critic_updates": False,
                    "ppo_updates": False,
                    "protected_trial_access": False,
                },
                "sources": {
                    "v5": {"runner": driver.source_record(sources["driver"])}
                },
                "inputs": {
                    "h0": {
                        "state": driver.source_record(
                            inputs["candidate_module_state"]
                        )
                    },
                    "v4_preexecution_failure": v4_failure_record,
                },
                "protected_trials_opened": [],
            },
        )
        _write_json(
            inputs["v5_execution_ledger"],
            {
                "schema_version": 5,
                "status": "PASS_H0_PRIMARY_SPLIT_V5_FINAL_HOLDOUT",
                "passed": True,
                "terminal_stage": "final_holdout_complete",
                "error": None,
                "started_unix_s": 1.0,
                "completed_unix_s": 2.0,
                "execution_lock": driver.source_record(
                    inputs["v5_execution_lock"]
                ),
                "attempt_claim": source_stub,
                "v3_terminal_ledger": source_stub,
                "v4_preexecution_failure": v4_failure_record,
                "v3_corpus_reused": True,
                "v3_failed_candidate_reused": False,
                "v4_candidate_reused": False,
                "candidate_created": True,
                "candidate_frozen_before_holdout": True,
                "holdout_access_claimed": True,
                "holdout_replay_completed": True,
                "final_holdout_completed": True,
                "actor_update_candidates": 1,
                "critic_updates": 0,
                "ppo_updates": 0,
                "protected_trials_opened": [],
                "retry_or_retuning_allowed": False,
                "next_stage": "CANONICAL_CLOSED_LOOP_QUALIFICATION",
            },
        )
        qualification_lock = {
            "schema_version": 5,
            "status": "H0_PRIMARY_SPLIT_V5_QUALIFICATION_UNLOCKED",
            "protocol_id": (
                "AB06_H0_PRIMARY_GRF_SPLIT_V5_AUTONOMOUS_QUALIFICATION"
            ),
            "source_protocol_id": contract.SOURCE_PROTOCOL_ID,
            "candidate_id": contract.CANDIDATE_ID,
            "so_policy_id": contract.SO_POLICY_ID,
            "event_contract_id": contract.EVENT_CONTRACT_ID,
            "retry_or_retuning_allowed": False,
            "authority": {
                "actor_updates_authorized": False,
                "critic_updates_authorized": False,
                "ppo_updates_authorized": False,
                "protected_trial_access_authorized": False,
            },
            "candidate_module": {
                "state": driver.source_record(inputs["candidate_module_state"]),
                "actor_feature_manifest": driver.source_record(
                    inputs["candidate_actor_manifest"]
                ),
            },
            "post_holdout_prerequisite": {
                "status": (
                    "PASS_H0_PRIMARY_SPLIT_V5_POST_HOLDOUT_PREREQUISITE"
                ),
                "execution_ledger": driver.source_record(
                    inputs["v5_execution_ledger"]
                ),
                "holdout_receipt": driver.source_record(
                    inputs["holdout_receipt"]
                ),
                "candidate_freeze": driver.source_record(
                    inputs["candidate_freeze"]
                ),
                "v4_preexecution_failure": v4_failure_record,
                "candidate_actor_digest": actor_digest,
            },
            "sources": {
                "qualification_runner": driver.source_record(sources["driver"])
            },
            "inputs": {
                "candidate_state": driver.source_record(
                    inputs["candidate_module_state"]
                ),
                "v4_preexecution_failure": v4_failure_record,
            },
            "protected_trials_opened": [],
        }
        _write_json(inputs["qualification_lock"], qualification_lock)
        _write_json(
            inputs["qualification_decision_receipt"],
            {
                "schema_version": 5,
                "status": (
                    "H0_PRIMARY_SPLIT_V5_QUALIFICATION_"
                    "BASELINE_TOLERANCE_DECIDED"
                ),
                "passed": True,
                "protocol_id": contract.SOURCE_PROTOCOL_ID,
                "canonical_case_ids": list(contract.QUALIFICATION_CASE_IDS),
                "candidate_holdout_receipt": driver.source_record(
                    inputs["holdout_receipt"]
                ),
                "runtime_contract": {
                    "event_contract_id": contract.EVENT_CONTRACT_ID,
                    "phase_fsm_input_mode": "legacy_events",
                    "morphology_weight": 0.0,
                },
                "actor_updates": 0,
                "critic_updates": 0,
                "ppo_updates": 0,
                "protected_trials_opened": [],
            },
        )
        case_gates = {}
        for case_id in contract.QUALIFICATION_CASE_IDS:
            gate = {
                "schema_version": 5,
                "status": "PASS_H0_PRIMARY_SPLIT_V5_QUALIFICATION_CASE",
                "passed": True,
                "case_id": case_id,
            }
            case_gates[case_id] = gate
            _write_json(inputs[f"qualification_gate_{case_id}"], gate)
        _write_json(
            inputs["qualification_execution_ledger"],
            {
                "schema_version": 5,
                "status": contract.QUALIFICATION_PASS_STATUS,
                "passed": True,
                "error": None,
                "qualification_lock": driver.source_record(
                    inputs["qualification_lock"]
                ),
                "v5_execution_ledger": driver.source_record(
                    inputs["v5_execution_ledger"]
                ),
                "v5_holdout_receipt": driver.source_record(
                    inputs["holdout_receipt"]
                ),
                "v4_preexecution_failure": v4_failure_record,
                "case_gates": case_gates,
                "baseline_rollouts_completed": 6,
                "candidate_rollouts_completed": 6,
                "actor_updates": 0,
                "critic_updates": 0,
                "ppo_updates": 0,
                "protected_trials_opened": [],
                "next_stage": "TRAINER_ZERO_UPDATE_PORT",
            },
        )
        return {
            "root": root,
            "lock": lock,
            "output": output,
            "sources": sources,
            "inputs": inputs,
            "actor_digest": actor_digest,
        }

    def test_freezer_requires_complete_qualification_and_only_writes_lock(self) -> None:
        world = self._frozen_world()
        inputs = world["inputs"]
        assert isinstance(inputs, dict)
        missing_gate = inputs[
            f"qualification_gate_{contract.QUALIFICATION_CASE_IDS[-1]}"
        ]
        missing_gate.unlink()
        with self.assertRaises(freezer.ZeroUpdateFreezeError):
            freezer.freeze()
        self.assertFalse(Path(world["lock"]).exists())
        self.assertFalse(Path(world["output"]).exists())

    def test_freezer_rejects_v4_preexecution_lineage_drift(self) -> None:
        world = self._frozen_world()
        inputs = world["inputs"]
        assert isinstance(inputs, dict)
        failure = json.loads(
            inputs["v4_preexecution_failure"].read_text(encoding="utf-8")
        )
        failure["v4_retry_allowed"] = True
        _write_json(inputs["v4_preexecution_failure"], failure)
        with self.assertRaises(freezer.ZeroUpdateFreezeError):
            freezer.freeze()
        self.assertFalse(Path(world["lock"]).exists())
        self.assertFalse(Path(world["output"]).exists())

    def test_freezer_lock_is_no_clobber_and_detects_source_drift(self) -> None:
        world = self._frozen_world()
        payload = freezer.freeze()
        self.assertEqual(payload["status"], contract.LOCK_STATUS)
        self.assertTrue(Path(world["lock"]).is_file())
        self.assertFalse(Path(world["output"]).exists())
        with self.assertRaises(freezer.ZeroUpdateFreezeError):
            freezer.freeze()
        sources = world["sources"]
        assert isinstance(sources, dict)
        sources["driver"].write_text("# source drift\n", encoding="utf-8")
        with self.assertRaises(driver.ZeroUpdatePortError):
            driver.verify_lock()

    def test_execute_publishes_pass_only_after_zero_update_artifacts(self) -> None:
        world = self._frozen_world()
        freezer.freeze()
        output = Path(world["output"])

        def fake_runtime() -> dict[str, object]:
            (output / contract.OUTPUT_NAMES["resolved_config"]).write_text(
                "iterations: 0\n", encoding="utf-8"
            )
            checkpoint = output / contract.OUTPUT_NAMES["checkpoint"]
            for relative in driver._CHECKPOINT_REQUIRED_SUFFIXES:  # noqa: SLF001
                path = checkpoint / relative
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_bytes(relative.encode("utf-8"))
            for name in ("initial_export", "restored_export"):
                export_root = output / contract.OUTPUT_NAMES[name]
                destination = export_root / "module_state.pkl"
                export_root.mkdir(parents=True, exist_ok=True)
                destination.write_bytes(b"same qualified actor")
                _write_json(
                    export_root / "actor_feature_manifest.json",
                    {"actor_digest": world["actor_digest"]},
                )
            return {
                "candidate_actor_digest": world["actor_digest"],
                "checks": {name: True for name in driver.REQUIRED_CHECKS},
            }

        with mock.patch.object(driver, "_runtime_port", side_effect=fake_runtime):
            receipt = driver.execute()
        self.assertEqual(receipt["status"], contract.PASS_STATUS)
        self.assertEqual(receipt["actor_transplants"], 1)
        self.assertEqual(receipt["actor_updates"], 0)
        self.assertEqual(receipt["critic_updates"], 0)
        self.assertEqual(receipt["ppo_updates"], 0)
        self.assertEqual(receipt["environment_samples"], 0)
        ledger = json.loads(
            (output / contract.OUTPUT_NAMES["ledger"]).read_text(encoding="utf-8")
        )
        self.assertEqual(ledger["status"], contract.PASS_STATUS)
        self.assertTrue(ledger["passed"])
        with self.assertRaises(driver.ZeroUpdatePortError):
            driver.execute()

    def test_export_writes_adjacent_manifest_for_the_exact_actor(self) -> None:
        world = self._frozen_world()
        freezer.freeze()
        destination = Path(world["root"]) / "exported_module"
        expected_state = {"actor": b"qualified"}

        class Module:
            def save_to_path(self, path: Path) -> None:
                path.mkdir()
                (path / "module_state.pkl").write_bytes(b"exported state")
                (path / "class_and_ctor_args.pkl").write_bytes(b"ctor")
                _write_json(path / "metadata.json", {"module": "fake"})

        class Algorithm:
            def get_module(self, module_id: str) -> Module:
                if module_id != contract.DEFAULT_POLICY_ID:
                    raise AssertionError(module_id)
                return Module()

        class WarmStart:
            @staticmethod
            def load_module_state(path: Path) -> dict[str, bytes]:
                if path != destination:
                    raise AssertionError(path)
                return expected_state

            @staticmethod
            def compare_actor_states(left: object, right: object) -> dict[str, object]:
                return {"exact": left == right, "max_abs_diff": 0.0}

            @staticmethod
            def actor_state_digest(state: object) -> str:
                if state != expected_state:
                    raise AssertionError(state)
                return str(world["actor_digest"])

        report = driver._export_actor(  # noqa: SLF001
            algo=Algorithm(),
            destination=destination,
            expected_state=expected_state,
            warm_start=WarmStart,
        )
        manifest_path = destination / "actor_feature_manifest.json"
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        self.assertEqual(manifest["actor_digest"], world["actor_digest"])
        self.assertEqual(manifest["trainable_scope"], "full_mean_network")
        self.assertEqual(manifest["logstd_policy"], "frozen_bit_exact")
        self.assertEqual(
            manifest["module_state_sha256"],
            driver._sha256_file(destination / "module_state.pkl"),  # noqa: SLF001
        )
        self.assertIn("actor_feature_manifest.json", report["tree"]["files"])


if __name__ == "__main__":
    unittest.main()
