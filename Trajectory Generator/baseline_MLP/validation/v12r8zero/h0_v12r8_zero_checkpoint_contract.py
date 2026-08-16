"""Source-only contract for the V12R8/Q3 full checkpoint-zero port.

The namespace is additive and binds only public, stable V12R8 and V12R8-Q3
interfaces.  Importing it performs no filesystem reads, model construction,
checkpoint publication, rollout, or training.  Candidate identity remains
deferred until both upstream pipelines have terminal PASS receipts for the
same exact five-file candidate tree.

The future port builds a fresh standard W512 RLlib Algorithm, transplants only
the qualified actor, and saves/reloads a full checkpoint at progress zero.
The downstream interface is resume-only: ``--resume-from checkpoint_zero``.
Neither warm-start mode belongs to this lineage.
"""

from __future__ import annotations

import json
import shlex
import subprocess
import sys
from pathlib import Path, PurePosixPath, PureWindowsPath
from typing import Any


_LOCAL_VALIDATION = Path(__file__).resolve().parent.parent
for _root in (_LOCAL_VALIDATION / "v12r8", _LOCAL_VALIDATION / "v12r8q3"):
    if str(_root) not in sys.path:
        sys.path.insert(0, str(_root))

import h0_v12r8_q3_qualification_contract as q3  # noqa: E402
import h0_v12r8_recovery_contract as r8  # noqa: E402


SCHEMA_VERSION = 1285
REVISION = "2026-08-14"
PROTOCOL_ID = "AB06_H0_V12R8_Q3_ZERO_CHECKPOINT_W512_V26"
PIPELINE_ID = "H0_V12R8_Q3_FRESH_CRITIC_ZERO_PROGRESS_PORT"
SOURCE_STATE = "SOURCE_CLOSED_RUNTIME_DEFERRED_UNTIL_R8_AND_Q3_TERMINAL_PASS"

ROOT = PurePosixPath("Trajectory Generator/baseline_MLP/validation/v12r8zero")
LOCK_PATH = ROOT / "h0_v12r8_zero_checkpoint_execution_lock.json"
OUTPUT_ROOT = ROOT / "h0_v12r8_zero_checkpoint_run_20260814"
CHECKPOINT_PATH = OUTPUT_ROOT / "checkpoint_zero"
AUDIT_PATH = OUTPUT_ROOT / "zero_checkpoint_audit.json"
RECEIPT_PATH = OUTPUT_ROOT / "receipt.json"
TERMINAL_LEDGER_PATH = OUTPUT_ROOT / "execution_ledger.json"
HANDOFF_PATH = OUTPUT_ROOT / "training_handoff.json"
RESOLVED_CONFIG_PATH = OUTPUT_ROOT / "training_cfg.resolved.yaml"

LOCK_STATUS = "PASS_H0_V12R8_ZERO_CHECKPOINT_EXECUTION_LOCK"
PASS_STATUS = "PASS_H0_V12R8_Q3_ZERO_CHECKPOINT"
FAIL_STATUS = "FAIL_H0_V12R8_Q3_ZERO_CHECKPOINT"
HANDOFF_STATUS = "DEFERRED_H0_V12R8_ZERO_CHECKPOINT_RESUME_INTERFACE"
NEXT_STAGE_AFTER_ZERO_PASS = "WAIT_SEPARATE_V12R8MORPH_POSITIVE_AB"
ATTEMPT_STATUS = "H0_V12R8_ZERO_CHECKPOINT_ATTEMPT_CLAIMED"
OUTPUT_NAMES = {
    "attempt_claim": "attempt_claim.json",
    "resolved_config": "training_cfg.resolved.yaml",
    "checkpoint": "checkpoint_zero",
    "initial_export": "rl_module_zero",
    "restored_export": "rl_module_reloaded",
    "audit": "zero_checkpoint_audit.json",
    "receipt": "receipt.json",
    "handoff": "training_handoff.json",
    "ledger": "execution_ledger.json",
}

R8_TERMINAL_ENDPOINT = {
    "schema_version": r8.SCHEMA_VERSION,
    "protocol_id": r8.PROTOCOL_ID,
    "pipeline_id": r8.PIPELINE_ID,
    "path": r8.LEDGER_PATH.as_posix(),
    "required_status": r8.PIPELINE_TERMINAL_PASS_STATUS,
    "verifier_module": (r8.ROOT / "run_h0_v12r8_recovery.py").as_posix(),
    "verifier": "verify_terminal_ledger",
    "expected_actor_updates": 1,
    "expected_critic_updates": 0,
    "expected_ppo_updates": 0,
}
R8_CANDIDATE_FREEZE_ENDPOINT = {
    "path": r8.CANDIDATE_FREEZE_PATH.as_posix(),
    "required_status": r8.CANDIDATE_FREEZE_PASS_STATUS,
    "verifier": "verify_candidate_freeze_receipt",
}
R8_FINAL_DEVELOPMENT_ENDPOINT = {
    "path": r8.FINAL_DEVELOPMENT_PATH.as_posix(),
    "required_status": r8.DEVELOPMENT_PASS_STATUS,
    "verifier": "verify_final_development_receipt",
}
Q3_TERMINAL_ENDPOINT = {
    "schema_version": q3.SCHEMA_VERSION,
    "protocol_id": q3.PROTOCOL_ID,
    "pipeline_id": q3.PIPELINE_ID,
    "path": q3.PIPELINE_LEDGER_PATH.as_posix(),
    "required_status": q3.PIPELINE_TERMINAL_PASS_STATUS,
    "verifier_module": (
        q3.ROOT / "runtime/run_h0_v12r8_q3_qualification.py"
    ).as_posix(),
    "verifier": "verify_terminal_ledger",
    "expected_actor_updates": 0,
    "expected_critic_updates": 0,
    "expected_ppo_updates": 0,
}
Q3_FINAL_ENDPOINT = {
    "path": q3.FINAL_RECEIPT_PATH.as_posix(),
    "required_status": q3.AGGREGATE_PASS_STATUS,
    "verifier": "verify_final_receipt",
}

UPSTREAM_ENDPOINTS = {
    "v12r8_terminal": R8_TERMINAL_ENDPOINT,
    "v12r8_q3_terminal": Q3_TERMINAL_ENDPOINT,
}
CANDIDATE_BINDING_STATE = "DEFERRED_UNTIL_R8_AND_Q3_TERMINAL_PASS"
CANDIDATE_ID = None
CANDIDATE_MODULE = None
CANDIDATE_MODULE_PATH = r8.CANDIDATE_MODULE_PATH
CANDIDATE_SELECTION_RULE = r8.CANDIDATE_SELECTION_RULE
CANDIDATE_REQUIRED_FILES = q3.CANDIDATE_REQUIRED_FILES
SOURCE_TOPOLOGY_ID = r8.TOPOLOGY_ID

TRAINING_CONFIG_PATH = PurePosixPath(
    "Trajectory Generator/runs/training/validation/warm_start_h1_runs/"
    "2026-07-15_h0_exact_interleaved_lr5e-7_iter2-51_pilot50/"
    "training_cfg.resolved.yaml"
)
TRAINING_ENTRYPOINT = PurePosixPath(
    "Trajectory Generator/baseline_MLP/train_ppo_mlp.py"
)
FINAL_TRAINING_OUTPUT = PurePosixPath(
    "Trajectory Generator/runs/training/v12r8_morphology_0025_50update"
)

INPUT_RELATIVE_PATHS = {
    "r8_candidate_freeze": r8.CANDIDATE_FREEZE_PATH.as_posix(),
    "r8_final_development": r8.FINAL_DEVELOPMENT_PATH.as_posix(),
    "r8_terminal_ledger": r8.LEDGER_PATH.as_posix(),
    "q3_protocol_freeze": q3.PROTOCOL_FREEZE_PATH.as_posix(),
    "q3_execution_lock": q3.EXECUTION_LOCK_PATH.as_posix(),
    "q3_final_receipt": q3.FINAL_RECEIPT_PATH.as_posix(),
    "q3_terminal_ledger": q3.PIPELINE_LEDGER_PATH.as_posix(),
    "training_config": TRAINING_CONFIG_PATH.as_posix(),
    "morphology_config": q3.MORPHOLOGY_CONFIG_PATH.as_posix(),
    "detector_profile": q3.DETECTOR_PROFILE_PATH.as_posix(),
    "morphology_profile": q3.MORPHOLOGY_PROFILE_PATH.as_posix(),
}

SOURCE_RELATIVE_PATHS = {
    "package": (ROOT / "__init__.py").as_posix(),
    "contract": (ROOT / "h0_v12r8_zero_checkpoint_contract.py").as_posix(),
    "gates": (ROOT / "h0_v12r8_zero_checkpoint_gates.py").as_posix(),
    "freezer": (ROOT / "freeze_h0_v12r8_zero_checkpoint.py").as_posix(),
    "runner": (ROOT / "run_h0_v12r8_zero_checkpoint.py").as_posix(),
    "tests": (ROOT / "test_h0_v12r8_zero_checkpoint.py").as_posix(),
    "r8_contract": (r8.ROOT / "h0_v12r8_recovery_contract.py").as_posix(),
    "r8_runner": R8_TERMINAL_ENDPOINT["verifier_module"],
    "q3_contract": (q3.ROOT / "h0_v12r8_q3_qualification_contract.py").as_posix(),
    "q3_gates": (q3.ROOT / "h0_v12r8_q3_qualification_gates.py").as_posix(),
    "q3_runtime_runner": Q3_TERMINAL_ENDPOINT["verifier_module"],
    "training_entrypoint": TRAINING_ENTRYPOINT.as_posix(),
    "training_config_module": "Trajectory Generator/baseline_MLP/training_config.py",
    "warm_start": "Trajectory Generator/baseline_MLP/warm_start.py",
    "asymmetric_module": "Trajectory Generator/baseline_MLP/asymmetric_rl_module.py",
    "reward_wrapper": "Trajectory Generator/baseline_MLP/reward_function.py",
    "morphology_corridor": (
        "Trajectory Generator/baseline_MLP/experimental_morphology_corridor.py"
    ),
    "environment_factory": "Trajectory Generator/baseline_MLP/env_factory.py",
    "environment": "Trajectory Generator/osim_trj_cmc_like.py",
}

STANDARD_RL_MODULE_KIND = "standard"
DEFAULT_POLICY_ID = "default_policy"
EXPECTED_ACTOR_FEATURES = q3.EXPECTED_ACTOR_FEATURES
EXPECTED_FULL_FEATURES = q3.EXPECTED_FULL_FEATURES
EXPECTED_ACTION_DIM = q3.EXPECTED_ACTION_SHAPE[0]
EXPECTED_HIDDENS = q3.EXPECTED_HIDDENS
EXPECTED_SIGMA = 0.005
DISABLED_CLOCK_COLUMNS = q3.DISABLED_CLOCK_COLUMNS
PRIMARY_LOAD_CONTRACT_ID = q3.PRIMARY_LOAD_CONTRACT_ID
LEGACY_EVENT_CONTRACT_ID = q3.LEGACY_EVENT_CONTRACT_ID
BINARY_EVENT_CONTRACT_ID = q3.EVENT_CONTRACT_ID
TARGET_CONTRACT_ID = q3.TARGET_CONTRACT_ID
POSITIVE_MORPHOLOGY_WEIGHT = 0.0025
POSITIVE_MORPHOLOGY_WEIGHTS = (POSITIVE_MORPHOLOGY_WEIGHT,)
FINAL_TRAINING_ITERATIONS = 50

TARGET_FIXED_CONFIG = {
    "iterations": 0,
    "num_env_runners": 0,
    "ray_num_cpus": 1,
    "tensorboard": False,
    "progress": False,
    "update_history": False,
    "exact_start_sampling": False,
    "asymmetric_actor_critic": True,
    "rl_module_kind": STANDARD_RL_MODULE_KIND,
    "num_hidden_layers": 2,
    "dim_hidden_layers": 512,
    "freeze_logstd": True,
    "phase_fsm_input_mode": "legacy_events",
    "event_contract_id": LEGACY_EVENT_CONTRACT_ID,
    "binary_phase_fsm_mode": q3.V26_BINARY_MODE,
    "binary_phase_detector_profile": q3.DETECTOR_PROFILE_PATH.as_posix(),
    "binary_phase_detector_profile_sha256": q3.DETECTOR_PROFILE_SHA256,
    "detector_sample_dt_s": 0.001,
    "binary_phase_debounce_s": 0.005,
    "binary_phase_event_contract_id": BINARY_EVENT_CONTRACT_ID,
}
TARGET_RUNTIME_ATTESTATIONS = {
    "binary_phase_detector_profile_sha256": q3.DETECTOR_PROFILE_SHA256,
    "actor_event_source": q3.V26_ACTOR_EVENT_SOURCE,
    "target_contract_id": TARGET_CONTRACT_ID,
}

ZERO_REWARD_CONFIG = {
    "morphology_profile": q3.MORPHOLOGY_PROFILE_CONFIG_VALUE,
    "morphology_phase_mode": q3.MORPHOLOGY_PHASE_MODE,
    "morphology_reward_delay_s": q3.MORPHOLOGY_DELAY_S,
    "morphology_max_delivery_latency_s": q3.MORPHOLOGY_MAX_DELIVERY_LATENCY_S,
    "morphology_causal_max_samples": 4096,
    "morphology_causal_event_contract_id": BINARY_EVENT_CONTRACT_ID,
    "morphology_causal_allow_effects": 0.0,
    "morphology_experimental_allow_effects": 0.0,
    "morphology_weight": 0.0,
    "morphology_hard_termination_enabled": 0.0,
}
POSITIVE_RESTORE_REWARD_CONFIG = {
    **ZERO_REWARD_CONFIG,
    "morphology_causal_allow_effects": 1.0,
    "morphology_weight": POSITIVE_MORPHOLOGY_WEIGHT,
}
PROFILE_ATTESTATIONS = {
    "detector_profile": {
        "path": q3.DETECTOR_PROFILE_PATH.as_posix(),
        "sha256": q3.DETECTOR_PROFILE_SHA256,
    },
    "morphology_config": {
        "path": q3.MORPHOLOGY_CONFIG_PATH.as_posix(),
        "sha256": q3.MORPHOLOGY_CONFIG_SHA256,
    },
    "morphology_profile": {
        "path": q3.MORPHOLOGY_PROFILE_PATH.as_posix(),
        "sha256": q3.MORPHOLOGY_PROFILE_SHA256,
    },
}

ZERO_COUNTER_NAMES = (
    "training_iteration",
    "num_env_steps_sampled_lifetime",
    "num_agent_steps_sampled_lifetime",
    "num_env_steps_trained_lifetime",
    "num_agent_steps_trained_lifetime",
    "num_grad_updates_lifetime",
)
ACTOR_SURFACE_NAMES = (
    "local",
    "learner",
    "env_runner",
    "initial_export",
    "restored_local",
    "restored_learner",
    "restored_env_runner",
    "restored_export",
    "positive_restore_local",
    "positive_restore_learner",
    "positive_restore_env_runner",
)
CRITIC_SURFACE_NAMES = (
    "fresh_before_transplant",
    "after_transplant",
    "after_save",
    "after_restore",
    "after_positive_restore",
)
OPTIMIZER_SURFACE_NAMES = (
    "before_transplant",
    "after_transplant",
    "after_save",
    "after_restore",
    "after_positive_restore",
)
PROGRESS_SURFACE_NAMES = (
    "before_transplant",
    "after_transplant",
    "after_save",
    "after_restore",
    "after_positive_restore",
)

CHECKPOINT_REQUIRED_SUFFIXES = frozenset(
    {
        "algorithm_state.pkl",
        "class_and_ctor_args.pkl",
        "rllib_checkpoint.json",
        "learner_group/learner/state.pkl",
        "learner_group/learner/rl_module/module_state.pkl",
        "env_runner/state.pkl",
    }
)

REQUIRED_AUDIT_CHECKS = (
    "r8_terminal_pass_semantic",
    "q3_terminal_pass_semantic",
    "same_exact_five_file_candidate",
    "standard_w512_actor_source_exact",
    "v26_binary_active_live_config_exact",
    "causal_morphology_zero_live_config_exact",
    "source_closure_rehashed_before_build",
    "fresh_target_algorithm",
    "source_critic_not_restored",
    "source_optimizer_not_restored",
    "fresh_critic_byte_exact_across_transplant_save_restore",
    "optimizer_empty_and_exact_across_transplant_save_restore",
    "zero_progress_across_transplant_save_restore",
    "actor_exact_on_all_surfaces",
    "full_rllib_checkpoint_saved_at_zero",
    "positive_restore_smoke_only_0025_allow_effects_one",
    "positive_restore_keeps_actor_critic_optimizer_and_zero_progress_exact",
    "source_closure_rehashed_after_restore",
    "no_algorithm_train_call",
    "no_environment_sample",
    "no_actor_update",
    "no_critic_update",
    "no_ppo_update",
)

AUTHORITY = {
    "source_scaffold_authorized": True,
    "candidate_binding_deferred": True,
    "freeze_requires_r8_terminal_pass": True,
    "freeze_requires_q3_terminal_pass": True,
    "checkpoint_build_requires_execution_lock": True,
    "fresh_target_algorithm_build_after_lock": True,
    "candidate_actor_only_transplant_after_lock": True,
    "full_checkpoint_save_reload_after_lock": True,
    "positive_restore_smoke_after_lock": True,
    "source_critic_restore_authorized": False,
    "source_optimizer_restore_authorized": False,
    "actor_updates_authorized": False,
    "critic_updates_authorized": False,
    "ppo_updates_authorized": False,
    "environment_sampling_authorized": False,
    "warm_start_flag_authorized": False,
    "warm_start_raw_flag_authorized": False,
    "training_execution_authorized_by_zero_protocol": False,
    "training_command_publication_authorized_by_zero_protocol": False,
    "separate_positive_ab_required": True,
}


def candidate_id_for_tree(tree_sha256: str) -> str:
    """Use the exact stable V12R8 candidate identity function."""

    return r8.candidate_id(tree_sha256)


def _platform_path(value: str | PurePosixPath, platform_id: str) -> str:
    pure = PurePosixPath(value)
    if platform_id == "macos_arm64":
        return pure.as_posix()
    if platform_id == "windows_x86_64":
        return str(PureWindowsPath(*pure.parts))
    raise ValueError(f"unsupported command platform: {platform_id!r}")


def resume_training_argv(platform_id: str) -> tuple[str, ...]:
    """Return the sole future training template: 50-update positive resume."""

    python = "python" if platform_id == "macos_arm64" else "python.exe"
    reward_json = json.dumps(
        POSITIVE_RESTORE_REWARD_CONFIG,
        sort_keys=True,
        separators=(",", ":"),
        allow_nan=False,
    )
    argv = (
        python,
        _platform_path(TRAINING_ENTRYPOINT, platform_id),
        "--config",
        _platform_path(TRAINING_CONFIG_PATH, platform_id),
        "--resume-from",
        _platform_path(CHECKPOINT_PATH, platform_id),
        "--output-dir",
        _platform_path(FINAL_TRAINING_OUTPUT, platform_id),
        "--iterations",
        str(FINAL_TRAINING_ITERATIONS),
        "--num-hidden-layers",
        "2",
        "--dim-hidden-layers",
        "512",
        "--freeze-logstd",
        "--asymmetric-actor-critic",
        "--rl-module-kind",
        STANDARD_RL_MODULE_KIND,
        "--phase-fsm-input-mode",
        "legacy_events",
        "--event-contract-id",
        LEGACY_EVENT_CONTRACT_ID,
        "--binary-phase-fsm-mode",
        q3.V26_BINARY_MODE,
        "--binary-phase-detector-profile",
        _platform_path(q3.DETECTOR_PROFILE_PATH, platform_id),
        "--detector-sample-dt-s",
        "0.001",
        "--binary-phase-debounce-s",
        "0.005",
        "--binary-phase-event-contract-id",
        BINARY_EVENT_CONTRACT_ID,
        "--reward-json",
        reward_json,
    )
    if (
        "--resume-from" not in argv
        or "--warm-start" in argv
        or "--warm-start-raw" in argv
        or argv[argv.index("--iterations") + 1] != "50"
    ):
        raise RuntimeError("checkpoint-zero training interface is not resume-only")
    return argv


def render_command(argv: tuple[str, ...], platform_id: str) -> str:
    if platform_id == "macos_arm64":
        return shlex.join(argv)
    if platform_id == "windows_x86_64":
        return subprocess.list2cmdline(argv)
    raise ValueError(f"unsupported command platform: {platform_id!r}")


def contract_self_check() -> dict[str, Any]:
    commands = {
        platform: resume_training_argv(platform)
        for platform in ("macos_arm64", "windows_x86_64")
    }
    checks = {
        "exact_upstream_abi": R8_TERMINAL_ENDPOINT["protocol_id"] == r8.PROTOCOL_ID
        and R8_TERMINAL_ENDPOINT["path"] == r8.LEDGER_PATH.as_posix()
        and Q3_TERMINAL_ENDPOINT["protocol_id"] == q3.PROTOCOL_ID
        and Q3_TERMINAL_ENDPOINT["path"] == q3.PIPELINE_LEDGER_PATH.as_posix(),
        "candidate_deferred": CANDIDATE_ID is None
        and CANDIDATE_MODULE is None
        and CANDIDATE_BINDING_STATE == "DEFERRED_UNTIL_R8_AND_Q3_TERMINAL_PASS",
        "standard_w512": EXPECTED_ACTOR_FEATURES == 35
        and EXPECTED_FULL_FEATURES == 84
        and EXPECTED_HIDDENS == (512, 512)
        and EXPECTED_ACTION_DIM == 2,
        "v26_binary_active": TARGET_FIXED_CONFIG["binary_phase_fsm_mode"]
        == "binary_active"
        and TARGET_FIXED_CONFIG["binary_phase_event_contract_id"]
        == q3.EVENT_CONTRACT_ID
        and TARGET_RUNTIME_ATTESTATIONS["binary_phase_detector_profile_sha256"]
        == q3.DETECTOR_PROFILE_SHA256
        and TARGET_RUNTIME_ATTESTATIONS["actor_event_source"]
        == q3.V26_ACTOR_EVENT_SOURCE,
        "zero_and_single_positive": ZERO_REWARD_CONFIG["morphology_weight"] == 0.0
        and ZERO_REWARD_CONFIG["morphology_causal_allow_effects"] == 0.0
        and POSITIVE_MORPHOLOGY_WEIGHTS == (0.0025,)
        and POSITIVE_RESTORE_REWARD_CONFIG["morphology_weight"] == 0.0025
        and POSITIVE_RESTORE_REWARD_CONFIG["morphology_causal_allow_effects"] == 1.0,
        "zero_progress": TARGET_FIXED_CONFIG["iterations"] == 0
        and TARGET_FIXED_CONFIG["num_env_runners"] == 0,
        "resume_only_50": all("--resume-from" in item for item in commands.values())
        and all("--warm-start" not in item for item in commands.values())
        and all("--warm-start-raw" not in item for item in commands.values())
        and all(
            item[item.index("--iterations") + 1] == "50" for item in commands.values()
        ),
        "zero_protocol_cannot_train": AUTHORITY[
            "training_execution_authorized_by_zero_protocol"
        ]
        is False
        and AUTHORITY["training_command_publication_authorized_by_zero_protocol"]
        is False,
    }
    return {"passed": all(checks.values()), "checks": checks}


__all__ = [name for name in globals() if name.isupper()] + [
    "candidate_id_for_tree",
    "contract_self_check",
    "render_command",
    "resume_training_argv",
]
