"""Historical source scaffold for the post-R7/Q3 W512 checkpoint-zero port.

Importing this module performs no filesystem access and grants no authority to
build an Algorithm, publish a checkpoint, or start training.  V12R7 has since
closed with a terminal FAIL, so this namespace can never satisfy its original
PASS prerequisites.  Its public freeze, execute, and terminal-verifier
entrypoints are permanently blocked; a successor must be rebound in the new
``v12r8zero`` namespace rather than mutating this historical lineage.

The eventual handoff is a full RLlib checkpoint at progress zero.  Downstream
training must therefore use ``--resume-from checkpoint_zero``; neither public
``--warm-start`` mode is part of this lineage.
"""

from __future__ import annotations

import copy
import json
import shlex
import subprocess
import sys
from pathlib import Path, PurePosixPath, PureWindowsPath
from typing import Any


_LOCAL_VALIDATION = Path(__file__).resolve().parent.parent
for _root in (_LOCAL_VALIDATION / "v12r7", _LOCAL_VALIDATION / "v12r7q3"):
    if str(_root) not in sys.path:
        sys.path.insert(0, str(_root))

import h0_v12r7_q3_qualification_contract as q3  # noqa: E402
import h0_v12r7_recovery_contract as r7  # noqa: E402


SCHEMA_VERSION = 1274
REVISION = "2026-08-14"
HISTORICAL_SCAFFOLD_ONLY = True
CANONICAL_ENTRYPOINTS_BLOCKED = True
SUCCESSOR_ZERO_NAMESPACE = "v12r8zero"
RETIREMENT_REASON = "V12R7_TERMINAL_FAIL_REQUIRES_NEW_V12R8_LINEAGE"
PROTOCOL_ID = "AB06_H0_V12R7_Q3_ZERO_CHECKPOINT_W512_V26"
PIPELINE_ID = "H0_V12R7_Q3_FRESH_CRITIC_ZERO_PROGRESS_PORT"
SOURCE_TOPOLOGY_ID = r7.TOPOLOGY_ID
QUALIFICATION_PROTOCOL_ID = q3.PROTOCOL_ID

ROOT = PurePosixPath("Trajectory Generator/baseline_MLP/validation/v12r7zero")
LOCK_PATH = ROOT / "h0_v12r7_zero_checkpoint_execution_lock.json"
OUTPUT_ROOT = ROOT / "h0_v12r7_zero_checkpoint_run_20260814"

LOCK_STATUS = "PASS_H0_V12R7_ZERO_CHECKPOINT_EXECUTION_LOCK"
PASS_STATUS = "PASS_H0_V12R7_Q3_ZERO_CHECKPOINT"
FAIL_STATUS = "FAIL_H0_V12R7_Q3_ZERO_CHECKPOINT"
ATTEMPT_STATUS = "H0_V12R7_ZERO_CHECKPOINT_ATTEMPT_CLAIMED"
NEXT_STAGE_AFTER_ZERO_PASS = "WAIT_SEPARATE_V12R7MORPH_POSITIVE_AB"

CANDIDATE_BINDING_STATE = "DEFERRED_UNTIL_R7_AND_Q3_TERMINAL_PASS"
CANDIDATE_ID = None
CANDIDATE_MODULE = None
CANDIDATE_MODULE_PATH = r7.CANDIDATE_MODULE_PATH
CANDIDATE_SELECTION_RULE = r7.CANDIDATE_SELECTION_RULE
CANDIDATE_REQUIRED_FILES = q3.CANDIDATE_REQUIRED_FILES

R7_CANDIDATE_FREEZE_PATH = r7.CANDIDATE_FREEZE_PATH
R7_FINAL_DEVELOPMENT_PATH = r7.FINAL_DEVELOPMENT_PATH
R7_TERMINAL_LEDGER_PATH = r7.LEDGER_PATH
Q3_PROTOCOL_FREEZE_PATH = q3.PROTOCOL_FREEZE_PATH
Q3_EXECUTION_LOCK_PATH = q3.EXECUTION_LOCK_PATH
Q3_FINAL_RECEIPT_PATH = q3.FINAL_RECEIPT_PATH
Q3_TERMINAL_LEDGER_PATH = q3.PIPELINE_LEDGER_PATH

# This is the resolved configuration of the successful 2026-07-15 warm-start
# pilot.  Architecture and V26/corridor fields are nevertheless restated as
# explicit CLI overrides, so a drift in the snapshot cannot silently change
# the checkpoint-zero ABI.
TRAINING_CONFIG_PATH = PurePosixPath(
    "Trajectory Generator/runs/training/validation/warm_start_h1_runs/"
    "2026-07-15_h0_exact_interleaved_lr5e-7_iter2-51_pilot50/"
    "training_cfg.resolved.yaml"
)
TRAINING_ENTRYPOINT = PurePosixPath(
    "Trajectory Generator/baseline_MLP/train_ppo_mlp.py"
)
MORPHOLOGY_CONFIG_PATH = q3.MORPHOLOGY_CONFIG_PATH
DETECTOR_PROFILE_PATH = q3.DETECTOR_PROFILE_PATH
MORPHOLOGY_PROFILE_PATH = q3.MORPHOLOGY_PROFILE_PATH

INPUT_RELATIVE_PATHS = {
    "r7_candidate_freeze": R7_CANDIDATE_FREEZE_PATH.as_posix(),
    "r7_final_development": R7_FINAL_DEVELOPMENT_PATH.as_posix(),
    "r7_terminal_ledger": R7_TERMINAL_LEDGER_PATH.as_posix(),
    "q3_protocol_freeze": Q3_PROTOCOL_FREEZE_PATH.as_posix(),
    "q3_execution_lock": Q3_EXECUTION_LOCK_PATH.as_posix(),
    "q3_final_receipt": Q3_FINAL_RECEIPT_PATH.as_posix(),
    "q3_terminal_ledger": Q3_TERMINAL_LEDGER_PATH.as_posix(),
    "training_config": TRAINING_CONFIG_PATH.as_posix(),
    "morphology_config": MORPHOLOGY_CONFIG_PATH.as_posix(),
    "detector_profile": DETECTOR_PROFILE_PATH.as_posix(),
    "morphology_profile": MORPHOLOGY_PROFILE_PATH.as_posix(),
}

SOURCE_RELATIVE_PATHS = {
    "package": (ROOT / "__init__.py").as_posix(),
    "contract": (ROOT / "h0_v12r7_zero_checkpoint_contract.py").as_posix(),
    "freezer": (ROOT / "freeze_h0_v12r7_zero_checkpoint.py").as_posix(),
    "runner": (ROOT / "run_h0_v12r7_zero_checkpoint.py").as_posix(),
    "tests": (ROOT / "test_h0_v12r7_zero_checkpoint.py").as_posix(),
    "r7_contract": (r7.ROOT / "h0_v12r7_recovery_contract.py").as_posix(),
    "r7_runner": (r7.ROOT / "run_h0_v12r7_recovery.py").as_posix(),
    "q3_contract": (q3.ROOT / "h0_v12r7_q3_qualification_contract.py").as_posix(),
    "q3_gates": (q3.ROOT / "h0_v12r7_q3_qualification_gates.py").as_posix(),
    "training_entrypoint": TRAINING_ENTRYPOINT.as_posix(),
    "training_config_module": "Trajectory Generator/baseline_MLP/training_config.py",
    "warm_start": "Trajectory Generator/baseline_MLP/warm_start.py",
    "asymmetric_module": ("Trajectory Generator/baseline_MLP/asymmetric_rl_module.py"),
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
EXPECTED_ACTION_DIM = 2
EXPECTED_HIDDENS = (512, 512)
EXPECTED_SIGMA = 0.005
DISABLED_CLOCK_COLUMNS = q3.DISABLED_CLOCK_COLUMNS
PRIMARY_LOAD_CONTRACT_ID = q3.PRIMARY_LOAD_CONTRACT_ID
BINARY_EVENT_CONTRACT_ID = q3.EVENT_CONTRACT_ID
TARGET_CONTRACT_ID = q3.TARGET_CONTRACT_ID
LEGACY_EVENT_CONTRACT_ID = q3.LEGACY_EVENT_CONTRACT_ID
MORPHOLOGY_WEIGHT = 0.0
POSITIVE_MORPHOLOGY_WEIGHTS = (0.0025, 0.005)

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
    "binary_phase_fsm_mode": "binary_active",
    "binary_phase_detector_profile": DETECTOR_PROFILE_PATH.as_posix(),
    "detector_sample_dt_s": 0.001,
    "binary_phase_debounce_s": 0.005,
    "binary_phase_event_contract_id": BINARY_EVENT_CONTRACT_ID,
}

TARGET_REWARD_CONFIG = {
    "morphology_profile": q3.MORPHOLOGY_PROFILE_CONFIG_VALUE,
    "morphology_profile_sha256": q3.MORPHOLOGY_PROFILE_SHA256,
    "morphology_phase_mode": q3.MORPHOLOGY_PHASE_MODE,
    "morphology_canonical_to_phase": 0.6223299989,
    "morphology_reward_delay_s": q3.MORPHOLOGY_DELAY_S,
    "morphology_max_delivery_latency_s": q3.MORPHOLOGY_MAX_DELIVERY_LATENCY_S,
    "morphology_causal_max_samples": 4096,
    "morphology_causal_event_contract_id": BINARY_EVENT_CONTRACT_ID,
    "morphology_causal_allow_effects": 0.0,
    "morphology_experimental_allow_effects": 0.0,
    "morphology_weight": MORPHOLOGY_WEIGHT,
    "morphology_hard_termination_enabled": 0.0,
}

AUTHORITY = {
    "source_scaffold_authorized": True,
    "historical_scaffold_only": True,
    "canonical_entrypoints_blocked": True,
    "r7_terminal_fail_acknowledged": True,
    "successor_requires_new_v12r8zero_namespace": True,
    "candidate_binding_deferred": True,
    "future_lock_requires_r7_terminal_pass": True,
    "future_lock_requires_q3_terminal_pass": True,
    "freeze_publication_authorized_now": False,
    "algorithm_build_authorized_now": False,
    "checkpoint_publication_authorized_now": False,
    "fresh_target_algorithm_build_authorized_after_lock": True,
    "candidate_actor_transplant_authorized_after_lock": True,
    "full_checkpoint_save_reload_authorized_after_lock": True,
    "positive_config_restore_smoke_authorized_after_lock": True,
    "source_critic_restore_authorized": False,
    "source_optimizer_restore_authorized": False,
    "actor_updates_authorized": False,
    "critic_updates_authorized": False,
    "ppo_updates_authorized": False,
    "environment_sampling_authorized": False,
    "positive_morphology_authorized_now": False,
    "training_execution_authorized_by_this_protocol": False,
}

ZERO_COUNTER_NAMES = (
    "training_iteration",
    "num_env_steps_sampled_lifetime",
    "num_agent_steps_sampled_lifetime",
    "num_env_steps_trained_lifetime",
    "num_agent_steps_trained_lifetime",
    "num_grad_updates_lifetime",
)

REQUIRED_CHECKS = (
    "r7_terminal_pass_semantic",
    "q3_terminal_pass_semantic",
    "same_exact_five_file_candidate",
    "standard_w512_actor_only_source",
    "actor_feature_manifest_exact",
    "v26_binary_active_target_exact",
    "morphology_corridor_weight_zero_exact",
    "morphology_positive_structure_deferred_compatible",
    "source_closure_rehashed_before_build",
    "fresh_target_algorithm",
    "fresh_critic_created_without_source_restore",
    "critic_unchanged_by_actor_transplant",
    "optimizer_state_empty_before_save",
    "optimizer_param_groups_exact_before_save",
    "optimizer_param_groups_unchanged_by_transplant",
    "zero_progress_before_save",
    "local_actor_exact_before_save",
    "learner_actor_exact_before_save",
    "env_runner_actor_exact_before_save",
    "export_actor_exact_before_save",
    "full_rllib_checkpoint_saved_at_zero",
    "source_closure_rehashed_before_restore",
    "restored_local_actor_exact",
    "restored_learner_actor_exact",
    "restored_env_runner_actor_exact",
    "restored_export_actor_exact",
    "restored_fresh_critic_exact",
    "restored_optimizer_state_empty",
    "restored_optimizer_param_groups_exact",
    "zero_progress_after_restore",
    "source_closure_rehashed_after_restore",
    "training_build_before_restore_source_order",
    "positive_resume_config_survives_restore",
    "deferred_resume_interface_uses_checkpoint_zero",
    "no_algorithm_train_call",
    "no_actor_update",
    "no_critic_update",
    "no_ppo_update",
    "no_environment_sample",
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

OUTPUT_NAMES = {
    "attempt_claim": "attempt_claim.json",
    "resolved_config": "training_cfg.resolved.yaml",
    "checkpoint": "checkpoint_zero",
    "initial_export": "rl_module_zero",
    "restored_export": "rl_module_reloaded",
    "handoff": "training_handoff.json",
    "audit": "zero_checkpoint_audit.json",
    "receipt": "receipt.json",
    "ledger": "execution_ledger.json",
}
CHECKPOINT_PATH = OUTPUT_ROOT / OUTPUT_NAMES["checkpoint"]
RESOLVED_CONFIG_PATH = OUTPUT_ROOT / OUTPUT_NAMES["resolved_config"]
HANDOFF_PATH = OUTPUT_ROOT / OUTPUT_NAMES["handoff"]
AUDIT_PATH = OUTPUT_ROOT / OUTPUT_NAMES["audit"]
RECEIPT_PATH = OUTPUT_ROOT / OUTPUT_NAMES["receipt"]
TERMINAL_LEDGER_PATH = OUTPUT_ROOT / OUTPUT_NAMES["ledger"]


def candidate_id_for_tree(tree_sha256: str) -> str:
    """Delegate the exact R7 candidate identity rule."""

    return r7.candidate_id(tree_sha256)


def _platform_path(value: str | PurePosixPath, platform_id: str) -> str:
    pure = PurePosixPath(value)
    if platform_id == "macos_arm64":
        return pure.as_posix()
    if platform_id == "windows_x86_64":
        return str(PureWindowsPath(*pure.parts))
    raise ValueError(f"unsupported command platform: {platform_id!r}")


def checkpoint_creation_argv(platform_id: str) -> tuple[tuple[str, ...], ...]:
    """Return template freezer/runner argv; R7 public entrypoints stay blocked."""

    python = "python" if platform_id == "macos_arm64" else "python.exe"
    freezer = _platform_path(SOURCE_RELATIVE_PATHS["freezer"], platform_id)
    runner = _platform_path(SOURCE_RELATIVE_PATHS["runner"], platform_id)
    return ((python, freezer), (python, runner))


def resume_training_argv(
    *,
    platform_id: str,
    output_dir: str | PurePosixPath,
    iterations: int,
    morphology_weight: float = 0.0,
    positive_morphology_authorized: bool = False,
) -> tuple[str, ...]:
    """Build one portable full-checkpoint resume argv.

    Positive corridor weights remain inaccessible unless a later caller holds
    and explicitly supplies separate positive-morphology authority.
    """

    output = PurePosixPath(output_dir)
    if (
        output.is_absolute()
        or ".." in output.parts
        or output.as_posix() != str(output_dir)
        or str(output_dir) in {"", "."}
    ):
        raise ValueError(
            "training output_dir must be canonical repository-relative POSIX"
        )
    if type(iterations) is not int or iterations <= 0:
        raise ValueError("training iterations must be a positive integer")
    if morphology_weight == 0.0:
        allow_effects = 0.0
    elif (
        positive_morphology_authorized is True
        and morphology_weight in POSITIVE_MORPHOLOGY_WEIGHTS
    ):
        allow_effects = 1.0
    else:
        raise ValueError("positive morphology requires separate frozen authority")

    reward = copy.deepcopy(TARGET_REWARD_CONFIG)
    reward["morphology_weight"] = float(morphology_weight)
    reward["morphology_causal_allow_effects"] = allow_effects
    python = "python" if platform_id == "macos_arm64" else "python.exe"
    return (
        python,
        _platform_path(TRAINING_ENTRYPOINT, platform_id),
        "--config",
        _platform_path(TRAINING_CONFIG_PATH, platform_id),
        "--resume-from",
        _platform_path(OUTPUT_ROOT / OUTPUT_NAMES["checkpoint"], platform_id),
        "--output-dir",
        _platform_path(output, platform_id),
        "--iterations",
        str(iterations),
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
        "binary_active",
        "--binary-phase-detector-profile",
        _platform_path(DETECTOR_PROFILE_PATH, platform_id),
        "--detector-sample-dt-s",
        "0.001",
        "--binary-phase-debounce-s",
        "0.005",
        "--binary-phase-event-contract-id",
        BINARY_EVENT_CONTRACT_ID,
        "--reward-json",
        json.dumps(reward, sort_keys=True, separators=(",", ":"), allow_nan=False),
    )


def render_command(argv: tuple[str, ...], platform_id: str) -> str:
    if platform_id == "macos_arm64":
        return shlex.join(argv)
    if platform_id == "windows_x86_64":
        return subprocess.list2cmdline(argv)
    raise ValueError(f"unsupported command platform: {platform_id!r}")


def contract_self_check() -> dict[str, Any]:
    """Check the source-only surface without reading future artifacts."""

    zero_argv = {
        platform_id: resume_training_argv(
            platform_id=platform_id,
            output_dir="Trajectory Generator/runs/training/v12r7_resume_example",
            iterations=50,
        )
        for platform_id in ("macos_arm64", "windows_x86_64")
    }
    checks = {
        "historical_r7_fail_closed": HISTORICAL_SCAFFOLD_ONLY is True
        and CANONICAL_ENTRYPOINTS_BLOCKED is True
        and SUCCESSOR_ZERO_NAMESPACE == "v12r8zero"
        and AUTHORITY["r7_terminal_fail_acknowledged"] is True
        and AUTHORITY["successor_requires_new_v12r8zero_namespace"] is True,
        "candidate_deferred": CANDIDATE_ID is None
        and CANDIDATE_MODULE is None
        and CANDIDATE_BINDING_STATE == "DEFERRED_UNTIL_R7_AND_Q3_TERMINAL_PASS",
        "terminal_paths_are_aliases": R7_TERMINAL_LEDGER_PATH == r7.LEDGER_PATH
        and Q3_FINAL_RECEIPT_PATH == q3.FINAL_RECEIPT_PATH
        and Q3_TERMINAL_LEDGER_PATH == q3.PIPELINE_LEDGER_PATH,
        "standard_w512": STANDARD_RL_MODULE_KIND == "standard"
        and EXPECTED_HIDDENS == (512, 512)
        and EXPECTED_ACTOR_FEATURES == 35
        and EXPECTED_FULL_FEATURES == 84,
        "v26_exact": TARGET_FIXED_CONFIG["binary_phase_fsm_mode"] == "binary_active"
        and TARGET_FIXED_CONFIG["binary_phase_event_contract_id"]
        == q3.EVENT_CONTRACT_ID
        and TARGET_CONTRACT_ID == q3.TARGET_CONTRACT_ID,
        "morphology_zero_compatible": TARGET_REWARD_CONFIG["morphology_weight"] == 0.0
        and TARGET_REWARD_CONFIG["morphology_causal_allow_effects"] == 0.0
        and TARGET_REWARD_CONFIG["morphology_phase_mode"] == q3.MORPHOLOGY_PHASE_MODE
        and POSITIVE_MORPHOLOGY_WEIGHTS == (0.0025, 0.005),
        "zero_updates": TARGET_FIXED_CONFIG["iterations"] == 0
        and TARGET_FIXED_CONFIG["num_env_runners"] == 0
        and AUTHORITY["actor_updates_authorized"] is False
        and AUTHORITY["critic_updates_authorized"] is False
        and AUTHORITY["ppo_updates_authorized"] is False,
        "publication_blocked_now": AUTHORITY["freeze_publication_authorized_now"]
        is False
        and AUTHORITY["algorithm_build_authorized_now"] is False
        and AUTHORITY["checkpoint_publication_authorized_now"] is False,
        "resume_only_handoff": all(
            "--resume-from" in argv for argv in zero_argv.values()
        )
        and all("--warm-start" not in argv for argv in zero_argv.values())
        and all("--warm-start-raw" not in argv for argv in zero_argv.values()),
    }
    return {"passed": all(checks.values()), "checks": checks}


__all__ = [name for name in globals() if name.isupper()] + [
    "candidate_id_for_tree",
    "checkpoint_creation_argv",
    "contract_self_check",
    "render_command",
    "resume_training_argv",
]
