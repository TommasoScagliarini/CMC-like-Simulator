"""Immutable contract for V12R9 checkpoint-zero production training readiness.

Importing this module is inert.  The protocol has two distinct live gates:

* a one-shot, no-update/no-sampling restore preflight that builds the exact
  production topology (12 remote EnvRunners plus the local EnvRunner), restores
  the full checkpoint-zero, attests every state/runtime surface, and shuts Ray
  down; and
* a post-run integrity audit over exactly 50 successful logical updates.

The actual training command remains owned by the morphology terminal handoff.
This namespace validates that command; it does not create a competing launcher.
"""

from __future__ import annotations

import copy
import sys
from pathlib import Path, PurePosixPath
from typing import Any


_VALIDATION_ROOT = Path(__file__).resolve().parent.parent
for _root in (
    _VALIDATION_ROOT / "v12r9zero",
    _VALIDATION_ROOT / "v12r9morph",
):
    if str(_root) not in sys.path:
        sys.path.insert(0, str(_root))

import h0_v12r9_morphology_contract as morph  # noqa: E402
import h0_v12r9_zero_checkpoint_contract as zero  # noqa: E402


SCHEMA_VERSION = 1296
REVISION = "2026-08-14"
PROTOCOL_ID = "AB06_H0_V12R9_MORPH_TRAINING_50_UPDATE_READINESS"
PIPELINE_ID = "H0_V12R9_ZERO_TO_12_RUNNER_PREFLIGHT_AND_50_UPDATE_AUDIT"
SOURCE_STATE = "SOURCE_READY_LIVE_PREFLIGHT_AND_TRAINING_NOT_EXECUTED"
REQUIRED_WORKING_DIRECTORY = "repository_root"

ROOT = PurePosixPath("Trajectory Generator/baseline_MLP/validation/v12r9training")
PREFLIGHT_ROOT = ROOT / "h0_v12r9_training_preflight_20260814"
PREFLIGHT_RECEIPT_PATH = PREFLIGHT_ROOT / "receipt.json"
VALIDATOR_PATH = ROOT / "validate_h0_v12r9_training.py"
TEST_PATH = ROOT / "test_h0_v12r9_training.py"

PREFLIGHT_PASS_STATUS = "PASS_H0_V12R9_TRAINING_PREFLIGHT_12_RUNNER_RESTORE"
POSTRUN_PASS_STATUS = "PASS_H0_V12R9_TRAINING_50_UPDATE_INTEGRITY"
PREFLIGHT_FAILURE_MODE = "STOP_NO_TRAINING"
POSTRUN_FAILURE_MODE = "STOP_NO_DOWNSTREAM_QUALIFICATION"

MORPH_TERMINAL_ENDPOINT = {
    "schema_version": morph.SCHEMA_VERSION,
    "protocol_id": morph.PROTOCOL_ID,
    "pipeline_id": morph.PIPELINE_ID,
    "path": morph.TERMINAL_LEDGER_PATH.as_posix(),
    "handoff_path": morph.TRAINING_HANDOFF_PATH.as_posix(),
    "required_status": morph.PIPELINE_TERMINAL_PASS_STATUS,
    "handoff_required_status": morph.HANDOFF_PASS_STATUS,
    "verifier_module": (
        morph.ROOT / "run_h0_v12r9_morphology.py"
    ).as_posix(),
    "verifier": "verify_terminal_ledger",
}
ZERO_TERMINAL_ENDPOINT = copy.deepcopy(morph.ZERO_TERMINAL_ENDPOINT)

TRAINING_LAUNCHER = morph.TRAINING_ENTRYPOINT
CHECKPOINT_ZERO_PATH = morph.CHECKPOINT_ZERO_PATH
ZERO_AUDIT_PATH = zero.AUDIT_PATH
FINAL_OUTPUT_DIR = morph.FINAL_OUTPUT_DIR
POSTRUN_AUDIT_PATH = FINAL_OUTPUT_DIR / "v12r9_training_integrity_audit.json"
FINAL_TRAINING_ITERATIONS = 50

EXPECTED_REMOTE_ENV_RUNNERS = 12
EXPECTED_LOCAL_ENV_RUNNERS = 1
EXPECTED_ENV_RUNNER_SURFACES = 13
EXPECTED_RAY_NUM_CPUS = 13
EXPECTED_RUNNER_INDICES = tuple(range(EXPECTED_ENV_RUNNER_SURFACES))
EXPECTED_RUNNERS_PER_START = 4
EXPECTED_START_OFFSETS_S = (
    1.756870983805102,
    1.956870983805102,
    2.156870983805102,
)
EXPECTED_START_LABELS = (
    "offset_1p756871s",
    "offset_1p956871s",
    "offset_2p156871s",
)
EXPECTED_ROLLOUT_FRAGMENT_LENGTH = 384
EXPECTED_STEPS_PER_START = 1536
EXPECTED_TRAIN_BATCH_SIZE = 4608
EXPECTED_MINIBATCH_SIZE = 512
EXPECTED_NUM_EPOCHS = 1
EXPECTED_KL_MINIBATCH_COUNT = 9
EXPECTED_LIFETIME_ENV_STEPS = 230400
EXPECTED_LR = 5.0e-7
EXPECTED_GAMMA = 0.99
EXPECTED_LAMBDA = 0.9
EXPECTED_CLIP_PARAM = 0.05
EXPECTED_KL_COEFF = 1.0
EXPECTED_KL_TARGET = 0.01
EXPECTED_MAX_MINIBATCH_MEAN_KL_LOSS = 0.01
EXPECTED_MAX_START_RUN_LENGTH = 1

EXPECTED_MODEL = {
    "rl_module_kind": "standard",
    "num_hidden_layers": 2,
    "dim_hidden_layers": 512,
    "asymmetric_actor_critic": True,
    "freeze_logstd": True,
    "actor_feature_count": morph.EXPECTED_ACTOR_FEATURES,
    "full_feature_count": morph.EXPECTED_FULL_FEATURES,
    "action_dim": morph.EXPECTED_ACTION_DIM,
}
EXPECTED_DETECTOR_RUNTIME = copy.deepcopy(morph.V26_RUNTIME_CONFIG)
EXPECTED_REWARD_OVERRIDES = copy.deepcopy(morph.POSITIVE_REWARD_CONFIG)
EXPECTED_CAUSAL_RUNTIME = copy.deepcopy(morph.CAUSAL_RUNTIME_CONFIG)
EXPECTED_CAUSAL_RUNTIME_ID = morph.CAUSAL_RUNTIME_ID

SOURCE_RELATIVE_PATHS = {
    "package": (ROOT / "__init__.py").as_posix(),
    "contract": (ROOT / "h0_v12r9_training_contract.py").as_posix(),
    "validator": VALIDATOR_PATH.as_posix(),
    "tests": TEST_PATH.as_posix(),
    "morph_contract": (
        morph.ROOT / "h0_v12r9_morphology_contract.py"
    ).as_posix(),
    "morph_runner": MORPH_TERMINAL_ENDPOINT["verifier_module"],
    "morph_training_launcher": TRAINING_LAUNCHER.as_posix(),
    "morph_causal_runtime": morph.CAUSAL_RUNTIME_CONFIG["implementation"],
    "morph_site_hook": morph.CAUSAL_RUNTIME_CONFIG["child_process_site_hook"],
    "zero_contract": (
        zero.ROOT / "h0_v12r9_zero_checkpoint_contract.py"
    ).as_posix(),
    "zero_runner": ZERO_TERMINAL_ENDPOINT["verifier_module"],
    "training_entrypoint": zero.TRAINING_ENTRYPOINT.as_posix(),
    "training_config_input": zero.TRAINING_CONFIG_PATH.as_posix(),
    "training_config_module": "Trajectory Generator/baseline_MLP/training_config.py",
    "warm_start": "Trajectory Generator/baseline_MLP/warm_start.py",
    "asymmetric_module": "Trajectory Generator/baseline_MLP/asymmetric_rl_module.py",
    "reward_wrapper": "Trajectory Generator/baseline_MLP/reward_function.py",
    "causal_corridor": (
        "Trajectory Generator/baseline_MLP/experimental_morphology_corridor.py"
    ),
    "restart_auditor": "validation/audit_training_restarts.py",
}

REQUIRED_PREFLIGHT_CHECKS = (
    "repository_root_cwd",
    "morph_terminal_pass_semantic",
    "zero_terminal_pass_semantic",
    "handoff_exact",
    "source_closure_exact_before_after",
    "final_output_absent",
    "exact_final_argv",
    "exact_target_args",
    "production_topology_built",
    "checkpoint_zero_restored",
    "positive_live_config_preserved",
    "actor_exact_local_learner_and_13_env_runners",
    "critic_exact",
    "optimizer_exact_after_lr_reapply",
    "zero_progress_before_and_after_introspection",
    "causal_runtime_driver_and_13_env_runners",
    "no_algorithm_update_call",
    "no_environment_sampling_call",
    "ray_shutdown",
)

REQUIRED_POSTRUN_CHECKS = (
    "preflight_receipt_pass",
    "canonical_run_directory",
    "summary_completed_exact_50",
    "resume_from_checkpoint_zero",
    "no_warm_start_interface",
    "model_and_runtime_exact",
    "positive_reward_exact",
    "resolved_config_exact",
    "history_exact_50_unique_updates",
    "lifetime_steps_exact",
    "exact_start_balance_every_update",
    "kl_guard_every_update",
    "optimizer_lr_every_update",
    "critic_audit_complete",
    "logstd_frozen_across_all_milestones",
    "all_50_milestones_complete",
    "final_checkpoint_complete",
    "no_supervisor_restart_or_skip",
    "no_hidden_env_runner_restart",
)

AUTHORITY = {
    "source_validation_authorized": True,
    "restore_preflight_authorized_after_morph_terminal_pass": True,
    "algorithm_builds_authorized_in_preflight": 1,
    "checkpoint_restores_authorized_in_preflight": 1,
    "algorithm_updates_authorized_in_preflight": 0,
    "environment_samples_authorized_in_preflight": 0,
    "training_requires_preflight_pass": True,
    "postrun_audit_required": True,
    "postrun_failure_authorizes_downstream": False,
    "preflight_receipt_overwrite_authorized": False,
    "postrun_audit_overwrite_authorized": False,
}


def final_training_argv(platform_id: str) -> tuple[str, ...]:
    """Delegate the sole training command to the terminal morphology contract."""

    return morph.final_training_argv(platform_id)


def render_training_command(platform_id: str) -> str:
    argv = final_training_argv(platform_id)
    return morph.render_command(argv, platform_id)


def contract_self_check() -> dict[str, Any]:
    endpoint = morph.TRAINING_VALIDATION_ENDPOINT
    commands = {
        platform: final_training_argv(platform)
        for platform in ("macos_arm64", "windows_x86_64")
    }
    checks = {
        "morph_contract_passes": morph.contract_self_check()["passed"] is True,
        "endpoint_exact": endpoint["namespace_root"] == ROOT.as_posix()
        and endpoint["protocol_id"] == PROTOCOL_ID
        and endpoint["verifier_module"] == VALIDATOR_PATH.as_posix()
        and endpoint["preflight_receipt_path"] == PREFLIGHT_RECEIPT_PATH.as_posix()
        and endpoint["preflight_required_status"] == PREFLIGHT_PASS_STATUS
        and endpoint["postrun_audit_path"] == POSTRUN_AUDIT_PATH.as_posix()
        and endpoint["postrun_required_status"] == POSTRUN_PASS_STATUS
        and endpoint["expected_new_updates"] == FINAL_TRAINING_ITERATIONS,
        "production_topology": EXPECTED_REMOTE_ENV_RUNNERS == 12
        and EXPECTED_ENV_RUNNER_SURFACES == 13
        and EXPECTED_RAY_NUM_CPUS == 13,
        "exact_sampling": EXPECTED_RUNNERS_PER_START == 4
        and EXPECTED_ROLLOUT_FRAGMENT_LENGTH == 384
        and EXPECTED_STEPS_PER_START == 1536
        and EXPECTED_TRAIN_BATCH_SIZE == 4608
        and EXPECTED_KL_MINIBATCH_COUNT == 9,
        "exact_updates": FINAL_TRAINING_ITERATIONS == 50
        and EXPECTED_LIFETIME_ENV_STEPS
        == FINAL_TRAINING_ITERATIONS * EXPECTED_TRAIN_BATCH_SIZE,
        "resume_only_launcher": all(
            argv[1].replace("\\", "/") == str(TRAINING_LAUNCHER)
            and "--resume-from" in argv
            and "--warm-start" not in argv
            and "--warm-start-raw" not in argv
            and argv[argv.index("--iterations") + 1] == "50"
            for argv in commands.values()
        ),
        "positive_runtime": EXPECTED_REWARD_OVERRIDES["morphology_weight"]
        == 0.0025
        and EXPECTED_REWARD_OVERRIDES["morphology_causal_allow_effects"] == 1.0
        and EXPECTED_REWARD_OVERRIDES["morphology_hard_termination_enabled"]
        == 0.0
        and EXPECTED_CAUSAL_RUNTIME_ID
        == "h0_v12r9_morph_strict_terminal_delay_v1",
        "fail_closed_authority": AUTHORITY[
            "algorithm_updates_authorized_in_preflight"
        ]
        == 0
        and AUTHORITY["environment_samples_authorized_in_preflight"] == 0
        and AUTHORITY["training_requires_preflight_pass"] is True
        and AUTHORITY["postrun_audit_required"] is True,
    }
    return {"passed": all(checks.values()), "checks": checks}


__all__ = [name for name in globals() if name.isupper()] + [
    "contract_self_check",
    "final_training_argv",
    "render_training_command",
]
