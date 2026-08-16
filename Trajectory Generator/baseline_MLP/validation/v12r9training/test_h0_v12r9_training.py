"""Source-only tests for the V12R9 training-readiness validator."""

from __future__ import annotations

import ast
import copy
import os
import sys
from pathlib import Path

import numpy as np
import pytest


HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import h0_v12r9_training_contract as contract  # noqa: E402
import validate_h0_v12r9_training as validator  # noqa: E402


def _surface() -> dict[str, object]:
    return {
        "actor_digest": "a" * 64,
        "actor_state_sha256": "b" * 64,
        "actor_key_count": 10,
        "actor_byte_count": 1,
    }


def _valid_preflight() -> dict[str, object]:
    progress = {name: 0 for name in contract.zero.ZERO_COUNTER_NAMES}
    actor = _surface()
    critic = {
        "critic_state_sha256": "c" * 64,
        "critic_key_count": 6,
        "critic_byte_count": 1,
    }
    optimizer = {"optimizer_state_empty": True, "optimizers": []}
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.PREFLIGHT_PASS_STATUS,
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "checks": {name: True for name in contract.REQUIRED_PREFLIGHT_CHECKS},
        "activity": {
            "algorithm_builds": 1,
            "checkpoint_restores": 1,
            "algorithm_update_calls": 0,
            "environment_sampling_calls": 0,
        },
        "morph_terminal_status": contract.morph.PIPELINE_TERMINAL_PASS_STATUS,
        "zero_terminal_status": contract.zero.PASS_STATUS,
        "training_argv": list(contract.final_training_argv("macos_arm64")),
        "source_closure_before": {"validator": {"sha256": "d" * 64}},
        "source_closure_after": {"validator": {"sha256": "d" * 64}},
        "actor_surfaces": {
            "expected_checkpoint_zero": actor,
            "local": copy.deepcopy(actor),
            "learner": copy.deepcopy(actor),
        },
        "critic_surface": critic,
        "expected_checkpoint_zero_critic": copy.deepcopy(critic),
        "optimizer": optimizer,
        "expected_checkpoint_zero_optimizer": copy.deepcopy(optimizer),
        "env_runners": [
            {
                "worker_index": index,
                "actor": copy.deepcopy(actor),
                "causal_runtime": {
                    "passed": True,
                    "runtime_id": contract.EXPECTED_CAUSAL_RUNTIME_ID,
                },
            }
            for index in contract.EXPECTED_RUNNER_INDICES
        ],
        "progress": {
            "before_restore": progress,
            "after_restore": copy.deepcopy(progress),
            "after_introspection": copy.deepcopy(progress),
        },
        "driver_causal_runtime": {
            "runtime_id": contract.EXPECTED_CAUSAL_RUNTIME_ID,
            "corridor_installed": True,
            "reward_installed": True,
        },
        "ray_shutdown": True,
        "training_executed": False,
        "final_output_absent": True,
    }


def _valid_history_row(iteration: int) -> dict[str, object]:
    steps = {
        label: float(contract.EXPECTED_STEPS_PER_START)
        for label in contract.EXPECTED_START_LABELS
    }
    balance = {
        "pass": True,
        "expected_steps": steps,
        "actual_steps": copy.deepcopy(steps),
        "missing": [],
        "unexpected": [],
        "mismatched": {},
        "learner_batch_pass": True,
        "learner_checks": {
            "single_epoch_contract": True,
            "three_start_contract": True,
            "connector_steps_in": True,
            "post_gae_compaction": True,
            "module_steps_trained": True,
            "start_interleaving": True,
            "advantage_counts": True,
            "kl_minibatch_count": True,
            "kl_values_finite": True,
        },
        "expected_real_steps": 4608.0,
        "learner_connector_steps_in": 4608.0,
        "learner_connector_steps_out": 4620.0,
        "pre_compaction_rows": 4620.0,
        "removed_compaction_rows": 12.0,
        "compacted_rows": 4608.0,
        "interleaved_rows": 4608.0,
        "interleaved_start_conditions": 3.0,
        "interleaved_rows_per_start": 1536.0,
        "max_start_run_length": 1.0,
        "expected_module_steps_trained": 4608.0,
        "module_steps_trained": 4608.0,
        "expected_kl_minibatches": 9.0,
        "kl_minibatch_count": 9.0,
        "kl_nonfinite_count": 0.0,
        "advantage_counts": copy.deepcopy(steps),
        "rollout_fragment_length": 384,
        "runners_per_start": 4,
    }
    kl = {
        "enabled": True,
        "pass": True,
        "logical_iteration": iteration,
        "max_minibatch_mean_kl_loss_limit": 0.01,
        "min_minibatch_mean_kl_loss_floor": -1.0e-7,
        "required_kl_nonfinite_count": 0.0,
        "metrics": {
            "max_minibatch_mean_kl_loss": 0.001,
            "min_minibatch_mean_kl_loss": 0.0,
            "kl_minibatch_count": 9.0,
            "kl_nonfinite_count": 0.0,
        },
        "checks": {
            "max_present_and_finite": True,
            "max_nonnegative": True,
            "max_within_limit": True,
            "min_present_and_finite": True,
            "min_above_floor": True,
            "nonfinite_count_present_and_finite": True,
            "nonfinite_count_zero": True,
        },
        "failed_checks": [],
    }
    return {
        "iteration": iteration,
        "episode_return_mean": 1.0,
        "episode_len_mean": 500.0,
        "num_env_steps_sampled_lifetime": float(iteration * 4608),
        "policy_loss": -0.1,
        "vf_loss": 0.2,
        "entropy": -7.0,
        "mean_kl_loss": 0.0005,
        "max_minibatch_mean_kl_loss": 0.001,
        "min_minibatch_mean_kl_loss": 0.0,
        "kl_minibatch_count": 9.0,
        "kl_nonfinite_count": 0.0,
        "current_kl_coeff": 0.5,
        "optimizer_learning_rates": [
            [
                {
                    "optimizer_name": "default_optimizer",
                    "optimizer_type": "Adam",
                    "learning_rate": 5.0e-7,
                }
            ]
        ],
        "exact_start_balance": balance,
        "kl_update_guard": kl,
        "iteration_milestone": f"/tmp/run/milestone_iteration_{iteration:06d}",
    }


def _valid_config() -> dict[str, object]:
    return {
        "model": {
            "num_hidden_layers": 2,
            "dim_hidden_layers": 512,
            "rl_module_kind": "standard",
            "asymmetric_actor_critic": True,
            "freeze_logstd": True,
            "freeze_actor": False,
        },
        "ppo": {
            "train_batch_size": 4608,
            "minibatch_size": 512,
            "num_epochs": 1,
            "lr": 5.0e-7,
            "gamma": 0.99,
            "lam": 0.9,
            "clip_param": 0.05,
            "kl_coeff": 1.0,
            "kl_target": 0.01,
        },
        "parallelism": {
            "num_env_runners": 12,
            "ray_num_cpus": 13,
            "exact_start_sampling": True,
        },
        "simulation": {
            "iterations": 50,
            "episode_start_offset_choices_s": list(
                contract.EXPECTED_START_OFFSETS_S
            ),
            "episode_start_offset_s": contract.EXPECTED_START_OFFSETS_S[1],
            "random_init": False,
        },
        "grf": {
            "phase_fsm_input_mode": "legacy_events",
            "event_contract_id": contract.morph.q3.LEGACY_EVENT_CONTRACT_ID,
            "binary_phase_fsm_mode": contract.morph.q3.V26_BINARY_MODE,
            "binary_phase_detector_profile": (
                contract.morph.q3.DETECTOR_PROFILE_PATH.as_posix()
            ),
            "detector_sample_dt_s": 0.001,
            "binary_phase_debounce_s": 0.005,
            "binary_phase_event_contract_id": contract.morph.q3.EVENT_CONTRACT_ID,
        },
        "supervision": {
            "max_consecutive_skips": 1,
            "max_consecutive_crash_restarts": 1,
            "checkpoint_every": 1,
            "retain_iteration_checkpoints": True,
            "max_minibatch_mean_kl_loss": 0.01,
        },
        "reward": copy.deepcopy(contract.EXPECTED_REWARD_OVERRIDES),
    }


def test_instruction_context_token_is_present() -> None:
    agents = validator.REPO_ROOT / "AGENTS.md"
    assert "CMC_AGENT_OK_2026" in agents.read_text(encoding="utf-8")


def test_contract_and_morph_endpoint_are_exact() -> None:
    assert contract.contract_self_check()["passed"] is True
    endpoint = contract.morph.TRAINING_VALIDATION_ENDPOINT
    assert endpoint["protocol_id"] == contract.PROTOCOL_ID
    assert endpoint["preflight_receipt_path"] == contract.PREFLIGHT_RECEIPT_PATH.as_posix()
    assert endpoint["postrun_audit_path"] == contract.POSTRUN_AUDIT_PATH.as_posix()


def test_final_command_parses_to_exact_12_runner_target() -> None:
    _, args = validator.parse_final_training_args()
    target = validator.target_args_snapshot(args)
    assert target["num_env_runners"] == 12
    assert target["ray_num_cpus"] == 13
    assert target["iterations"] == 50
    assert target["rollout_fragment_length"] == 384
    assert target["reward_overrides"]["morphology_weight"] == 0.0025


def test_plan_is_source_only_and_forbids_update_and_sampling() -> None:
    plan = validator.build_preflight_plan()
    assert plan["source_only"] is True
    assert plan["preflight"]["algorithm_update_calls"] == 0
    assert plan["preflight"]["environment_sampling_calls"] == 0
    assert plan["preflight"]["env_runner_surfaces"] == 13


def test_validator_ast_has_no_update_or_sampling_method_call() -> None:
    tree = ast.parse((HERE / "validate_h0_v12r9_training.py").read_text(encoding="utf-8"))
    forbidden = [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Attribute) and node.attr in {"train", "sample"}
    ]
    assert forbidden == []


def test_source_check_is_pass_but_does_not_claim_live_readiness() -> None:
    result = validator.source_check()
    assert result["passed"] is True
    assert result["source_only"] is True
    assert result["training_ready"] is False


def test_preflight_receipt_gate_accepts_exact_13_surfaces() -> None:
    assert validator.preflight_receipt_gate(_valid_preflight())["passed"] is True


@pytest.mark.parametrize(
    "mutation",
    (
        lambda payload: payload["env_runners"].pop(),
        lambda payload: payload["env_runners"][5]["causal_runtime"].update(
            {"runtime_id": "wrong"}
        ),
        lambda payload: payload["progress"]["after_introspection"].update(
            {"training_iteration": 1}
        ),
        lambda payload: payload["actor_surfaces"]["learner"].update(
            {"actor_digest": "e" * 64}
        ),
        lambda payload: payload.update({"source_closure_after": {}}),
        lambda payload: payload["activity"].update({"algorithm_update_calls": 1}),
    ),
)
def test_preflight_receipt_gate_rejects_drift(mutation) -> None:
    payload = _valid_preflight()
    mutation(payload)
    assert validator.preflight_receipt_gate(payload)["passed"] is False


def test_history_gate_accepts_exact_50_updates() -> None:
    rows = [_valid_history_row(index) for index in range(1, 51)]
    result = validator.history_integrity_gate(rows)
    assert result["passed"] is True
    assert result["failed_iterations"] == []


@pytest.mark.parametrize(
    "mutation",
    (
        lambda rows: rows.pop(),
        lambda rows: rows[9].update({"num_env_steps_sampled_lifetime": 1.0}),
        lambda rows: rows[4]["exact_start_balance"].update(
            {"max_start_run_length": 2.0}
        ),
        lambda rows: rows[7]["kl_update_guard"].update({"pass": False}),
        lambda rows: rows[2].update({"optimizer_learning_rates": []}),
    ),
)
def test_history_gate_rejects_any_update_drift(mutation) -> None:
    rows = [_valid_history_row(index) for index in range(1, 51)]
    mutation(rows)
    assert validator.history_integrity_gate(rows)["passed"] is False


def test_resolved_config_gate_accepts_exact_runtime_and_rejects_detector_drift() -> None:
    valid = validator._resolved_config_gate(_valid_config())  # noqa: SLF001
    assert valid["passed"] is True
    drifted = _valid_config()
    drifted["grf"]["binary_phase_fsm_mode"] = "binary_shadow"
    assert validator._resolved_config_gate(drifted)["passed"] is False  # noqa: SLF001


def test_complete_expected_resolved_config_passes_its_semantic_gate() -> None:
    expected = validator._expected_resolved_config()  # noqa: SLF001
    assert expected["reward"]["morphology_causal_max_samples"] == 4096.0
    assert validator._resolved_config_gate(expected)["passed"] is True  # noqa: SLF001


def test_logstd_gate_checks_all_50_milestones_bit_exact(tmp_path: Path) -> None:
    reference = {
        "pi.1.weight": np.zeros((4, 512), dtype=np.float32),
        "pi.1.bias": np.array([0.0, 0.0, -5.3, -5.3], dtype=np.float32),
    }

    def loader(_path: Path):
        return copy.deepcopy(reference)

    result = validator._logstd_gate(tmp_path, state_loader=loader)  # noqa: SLF001
    assert result["passed"] is True
    assert result["checked_iterations"] == 50


def test_logstd_gate_rejects_one_changed_milestone(tmp_path: Path) -> None:
    calls = 0
    reference = {
        "pi.1.weight": np.zeros((4, 512), dtype=np.float32),
        "pi.1.bias": np.array([0.0, 0.0, -5.3, -5.3], dtype=np.float32),
    }

    def loader(_path: Path):
        nonlocal calls
        calls += 1
        state = copy.deepcopy(reference)
        # First call is checkpoint-zero; call 11 is milestone 10.
        if calls == 11:
            state["pi.1.bias"][2] += np.float32(0.01)
        return state

    result = validator._logstd_gate(tmp_path, state_loader=loader)  # noqa: SLF001
    assert result["passed"] is False
    assert result["invalid_iterations"] == [10]


def test_postrun_gate_accepts_only_all_required_checks() -> None:
    payload = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.POSTRUN_PASS_STATUS,
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "checks": {name: True for name in contract.REQUIRED_POSTRUN_CHECKS},
        "failed_checks": [],
        "training_updates": 50,
        "environment_steps": 230400,
    }
    assert validator.postrun_integrity_gate(payload)["passed"] is True
    payload["checks"]["no_hidden_env_runner_restart"] = False
    assert validator.postrun_integrity_gate(payload)["passed"] is False


def test_strict_readers_reject_duplicate_nonfinite_and_blank_rows(tmp_path: Path) -> None:
    duplicate = tmp_path / "duplicate.json"
    duplicate.write_text('{"a":1,"a":2}', encoding="utf-8")
    with pytest.raises(validator.TrainingReadinessError):
        validator.strict_json_any(duplicate)
    nonfinite = tmp_path / "nonfinite.json"
    nonfinite.write_text('{"a":NaN}', encoding="utf-8")
    with pytest.raises(validator.TrainingReadinessError):
        validator.strict_json_any(nonfinite)
    blank = tmp_path / "rows.jsonl"
    blank.write_text('{}\n\n{}\n', encoding="utf-8")
    with pytest.raises(validator.TrainingReadinessError):
        validator.strict_jsonl(blank)


def test_repository_root_cwd_is_mandatory(tmp_path: Path) -> None:
    assert validator.require_repository_root_cwd(validator.REPO_ROOT)["passed"] is True
    with pytest.raises(validator.TrainingReadinessError):
        validator.require_repository_root_cwd(tmp_path)


def test_literal_abi_values_are_contiguous_in_validator_source() -> None:
    source = (HERE / "validate_h0_v12r9_training.py").read_text(encoding="utf-8")
    for value in validator.VALIDATION_ABI_LITERALS.values():
        assert value in source


def test_import_and_source_actions_cannot_publish_runtime_receipts() -> None:
    tree = ast.parse((HERE / "validate_h0_v12r9_training.py").read_text(encoding="utf-8"))
    top_level_calls = [
        node
        for node in tree.body
        if isinstance(node, ast.Expr) and isinstance(node.value, ast.Call)
    ]
    assert top_level_calls == []
    before = {
        path: (path.exists(), path.stat().st_mtime_ns if path.exists() else None)
        for path in (validator.PREFLIGHT_RECEIPT, validator.POSTRUN_AUDIT)
    }
    validator.build_preflight_plan()
    validator.source_check()
    after = {
        path: (path.exists(), path.stat().st_mtime_ns if path.exists() else None)
        for path in (validator.PREFLIGHT_RECEIPT, validator.POSTRUN_AUDIT)
    }
    assert after == before


def test_required_interpreter_is_not_baked_into_cross_platform_argv() -> None:
    mac = contract.final_training_argv("macos_arm64")
    windows = contract.final_training_argv("windows_x86_64")
    assert mac[0] == "python"
    assert windows[0] == "python.exe"
    assert mac[1].endswith("run_h0_v12r9_morphology_training.py")
    assert windows[1].endswith("run_h0_v12r9_morphology_training.py")
    assert os.path.sep not in "python"
