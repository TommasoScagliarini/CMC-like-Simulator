"""Immutable contract for the latent-live V12R8 positive morphology A/B.

Importing this module is inert.  The protocol can be frozen only after the
native R8, R8-Q3, and R8-zero semantic verifiers all return terminal PASS for
one exact candidate/checkpoint lineage.  The current Q3 traces do not expose
the complete dynamics/event/reward/causal surfaces required by this A/B, so
the only admissible control strategy is six physical control-then-positive
pairs (twelve local rollouts).
"""

from __future__ import annotations

import copy
import json
import shlex
import subprocess
import sys
from pathlib import Path, PurePosixPath, PureWindowsPath
from typing import Any


_VALIDATION_ROOT = Path(__file__).resolve().parent.parent
for _root in (
    _VALIDATION_ROOT / "v12r8",
    _VALIDATION_ROOT / "v12r8q3",
    _VALIDATION_ROOT / "v12r8zero",
):
    if str(_root) not in sys.path:
        sys.path.insert(0, str(_root))

import h0_v12r8_q3_qualification_contract as q3  # noqa: E402
import h0_v12r8_recovery_contract as r8  # noqa: E402
import h0_v12r8_zero_checkpoint_contract as zero  # noqa: E402


SCHEMA_VERSION = 1287
REVISION = "2026-08-14"
PROTOCOL_ID = "AB06_H0_V12R8_MORPH_POSITIVE_0025_QUALIFICATION"
PIPELINE_ID = "H0_V12R8_MORPH_SIX_CASE_CAUSAL_PHYSICAL_AB"
SOURCE_STATE = "LATENT_LIVE_DEFERRED_UNTIL_R8_Q3_ZERO_TERMINAL_PASS"

ROOT = PurePosixPath("Trajectory Generator/baseline_MLP/validation/v12r8morph")
RUN_ROOT = ROOT / "h0_v12r8_morph_run_20260814"
PAIR_ROOT = RUN_ROOT / "pairs"
FINAL_ROOT = RUN_ROOT / "finalize"
PROTOCOL_FREEZE_PATH = ROOT / "h0_v12r8_morph_protocol_freeze.json"
EXECUTION_LOCK_PATH = ROOT / "h0_v12r8_morph_execution_lock.json"
PIPELINE_CLAIM_PATH = RUN_ROOT / "pipeline_claim.json"
# The zero-stage runtime already exported this exact standard RLModule *after*
# restoring the full checkpoint.  Reusing that audited export avoids creating
# a second Algorithm or introducing another state transformation before the
# physical A/B.
MODULE_EXPORT_PATH = zero.OUTPUT_ROOT / zero.OUTPUT_NAMES["restored_export"]
FINAL_RECEIPT_PATH = FINAL_ROOT / "receipt.json"
TERMINAL_LEDGER_PATH = RUN_ROOT / "pipeline_ledger.json"
TRAINING_HANDOFF_PATH = FINAL_ROOT / "training_handoff.json"

FREEZE_STATUS = "PASS_H0_V12R8_MORPH_PROTOCOL_FREEZE"
LOCK_STATUS = "PASS_H0_V12R8_MORPH_EXECUTION_LOCK"
CLAIM_STATUS = "CLAIM_H0_V12R8_MORPH_PIPELINE"
ARM_COMPLETE_STATUS = "COMPLETE_H0_V12R8_MORPH_ARM"
ARM_PASS_STATUS = "PASS_H0_V12R8_MORPH_ARM"
PAIR_COMPLETE_STATUS = "COMPLETE_H0_V12R8_MORPH_PAIR"
PAIR_PASS_STATUS = "PASS_H0_V12R8_MORPH_PAIR"
PAIR_FAIL_STATUS = "FAIL_H0_V12R8_MORPH_PAIR"
AGGREGATE_PASS_STATUS = "PASS_H0_V12R8_MORPH_POSITIVE_QUALIFICATION"
AGGREGATE_FAIL_STATUS = "FAIL_H0_V12R8_MORPH_POSITIVE_QUALIFICATION"
PIPELINE_TERMINAL_PASS_STATUS = "PASS_H0_V12R8_MORPH_PIPELINE_TERMINAL"
PIPELINE_TERMINAL_FAIL_STATUS = "FAIL_H0_V12R8_MORPH_PIPELINE_TERMINAL"
HANDOFF_PASS_STATUS = "PASS_H0_V12R8_MORPH_TRAINING_HANDOFF"
NEXT_STAGE_AFTER_PASS = "AUTHORIZED_50_UPDATE_RESUME_FROM_CHECKPOINT_ZERO"
NEXT_STAGE_AFTER_FAIL = "STOP_TERMINAL_NO_RETRY_SWEEP_OR_POSITIVE_TRAINING"

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
ZERO_TERMINAL_ENDPOINT = {
    "schema_version": zero.SCHEMA_VERSION,
    "protocol_id": zero.PROTOCOL_ID,
    "pipeline_id": zero.PIPELINE_ID,
    "path": zero.TERMINAL_LEDGER_PATH.as_posix(),
    "receipt_path": zero.RECEIPT_PATH.as_posix(),
    "required_status": zero.PASS_STATUS,
    "verifier_module": (
        zero.ROOT / "run_h0_v12r8_zero_checkpoint.py"
    ).as_posix(),
    "verifier": "verify_terminal_pass",
    "expected_actor_updates": 0,
    "expected_critic_updates": 0,
    "expected_ppo_updates": 0,
}
UPSTREAM_ENDPOINTS = {
    "v12r8_terminal": R8_TERMINAL_ENDPOINT,
    "v12r8_q3_terminal": Q3_TERMINAL_ENDPOINT,
    "v12r8_zero_terminal": ZERO_TERMINAL_ENDPOINT,
}

CANDIDATE_MODULE_PATH = r8.CANDIDATE_MODULE_PATH
CANDIDATE_REQUIRED_FILES = q3.CANDIDATE_REQUIRED_FILES
CHECKPOINT_ZERO_PATH = zero.CHECKPOINT_PATH
TRAINING_CONFIG_PATH = zero.RESOLVED_CONFIG_PATH
DEFAULT_POLICY_ID = zero.DEFAULT_POLICY_ID

SOURCE_RELATIVE_PATHS = {
    "package": (ROOT / "__init__.py").as_posix(),
    "contract": (ROOT / "h0_v12r8_morphology_contract.py").as_posix(),
    "gates": (ROOT / "h0_v12r8_morphology_gates.py").as_posix(),
    "freezer": (ROOT / "freeze_h0_v12r8_morphology.py").as_posix(),
    "physical_runtime": (
        ROOT / "h0_v12r8_morphology_physical_rollout.py"
    ).as_posix(),
    "causal_runtime": (
        ROOT / "h0_v12r8_morphology_causal_runtime.py"
    ).as_posix(),
    "training_launcher": (
        ROOT / "run_h0_v12r8_morphology_training.py"
    ).as_posix(),
    "training_site_hook": (ROOT / "sitecustomize.py").as_posix(),
    "runner": (ROOT / "run_h0_v12r8_morphology.py").as_posix(),
    "tests": (ROOT / "test_h0_v12r8_morphology_scaffold.py").as_posix(),
    "r8_contract": (r8.ROOT / "h0_v12r8_recovery_contract.py").as_posix(),
    "r8_runner": R8_TERMINAL_ENDPOINT["verifier_module"],
    "q3_contract": (q3.ROOT / "h0_v12r8_q3_qualification_contract.py").as_posix(),
    "q3_runner": Q3_TERMINAL_ENDPOINT["verifier_module"],
    "q3_physical": (
        q3.ROOT / "runtime/h0_v12r8_q3_physical_rollout.py"
    ).as_posix(),
    "q3_noise": (
        q3.ROOT / "runtime/prepare_h0_v12r8_q3_noise_tapes.py"
    ).as_posix(),
    "mature_physical": (
        "Trajectory Generator/baseline_MLP/validation/v12r5q3/runtime/"
        "h0_v12r5_q3_physical_rollout.py"
    ),
    "zero_contract": (zero.ROOT / "h0_v12r8_zero_checkpoint_contract.py").as_posix(),
    "zero_gates": (zero.ROOT / "h0_v12r8_zero_checkpoint_gates.py").as_posix(),
    "zero_runner": ZERO_TERMINAL_ENDPOINT["verifier_module"],
    "training_entrypoint": zero.TRAINING_ENTRYPOINT.as_posix(),
    "training_config_module": "Trajectory Generator/baseline_MLP/training_config.py",
    "warm_start": "Trajectory Generator/baseline_MLP/warm_start.py",
    "asymmetric_module": "Trajectory Generator/baseline_MLP/asymmetric_rl_module.py",
    "reward_wrapper": "Trajectory Generator/baseline_MLP/reward_function.py",
    "causal_corridor": (
        "Trajectory Generator/baseline_MLP/experimental_morphology_corridor.py"
    ),
    "environment_factory": "Trajectory Generator/baseline_MLP/env_factory.py",
    "environment": "Trajectory Generator/osim_trj_cmc_like.py",
}

INPUT_RELATIVE_PATHS = {
    "r8_terminal_ledger": r8.LEDGER_PATH.as_posix(),
    "q3_terminal_ledger": q3.PIPELINE_LEDGER_PATH.as_posix(),
    "q3_final_receipt": q3.FINAL_RECEIPT_PATH.as_posix(),
    "zero_terminal_ledger": zero.TERMINAL_LEDGER_PATH.as_posix(),
    "zero_receipt": zero.RECEIPT_PATH.as_posix(),
    "zero_audit": zero.AUDIT_PATH.as_posix(),
    "training_config": TRAINING_CONFIG_PATH.as_posix(),
    "detector_profile": q3.DETECTOR_PROFILE_PATH.as_posix(),
    "morphology_config": q3.MORPHOLOGY_CONFIG_PATH.as_posix(),
    "morphology_profile": q3.MORPHOLOGY_PROFILE_PATH.as_posix(),
    **{
        f"noise_tape_{case_id}": q3.canonical_case(case_id)["noise_tape"]
        for case_id in q3.CASE_IDS
    },
}

CONTROL_SOURCE_MODE = "paired_rerun"
Q3_CONTROL_REUSE_STATE = "REJECTED_TRACE_SURFACES_INSUFFICIENT"
Q3_REQUIRED_REUSE_STREAMS = (
    "observations",
    "actions",
    "dynamics",
    "events",
    "reward_without_morphology",
    "morphology_loss",
    "causal_samples",
)
Q3_TRACE_NATIVE_FIELDS = (
    "actor_observation",
    "raw_action",
    "applied_action",
    "reward",
    "grf_penetration_m",
    "reserve_norm_nm",
    "residual_norm_nm",
    "terminated",
    "truncated",
    "end_reason",
)

CASE_IDS = tuple(q3.CASE_IDS)
HOLDOUT_CASES = tuple(copy.deepcopy(q3.HOLDOUT_CASES))
EXPECTED_TAPE_ARRAY_SHA256 = copy.deepcopy(q3.EXPECTED_TAPE_ARRAY_SHA256)
EXPECTED_PAIR_COUNT = 6
CONTROL_ROLE = "control"
POSITIVE_ROLE = "positive"
PAIR_ROLE_ORDER = (CONTROL_ROLE, POSITIVE_ROLE)
EXPECTED_LOCAL_ROLLOUT_COUNT = EXPECTED_PAIR_COUNT * len(PAIR_ROLE_ORDER)
EXPECTED_STEPS_PER_ROLLOUT = q3.EXPECTED_STEPS
EXPECTED_RAW_SAMPLES_PER_ROLLOUT = q3.EXPECTED_RAW_SENSOR_SAMPLES
EXPECTED_TOTAL_POLICY_STEPS = EXPECTED_LOCAL_ROLLOUT_COUNT * EXPECTED_STEPS_PER_ROLLOUT

MORPHOLOGY_PHASE_MODE = q3.MORPHOLOGY_PHASE_MODE
MORPHOLOGY_CONTROL_WEIGHT = 0.0
MORPHOLOGY_POSITIVE_WEIGHT = 0.0025
MORPHOLOGY_POSITIVE_WEIGHTS = (MORPHOLOGY_POSITIVE_WEIGHT,)
MORPHOLOGY_REWARD_DELAY_S = q3.MORPHOLOGY_DELAY_S
MORPHOLOGY_MAX_DELIVERY_LATENCY_S = q3.MORPHOLOGY_MAX_DELIVERY_LATENCY_S
MORPHOLOGY_HARD_TERMINATION_ENABLED = 0.0
CONTROL_REWARD_CONFIG = copy.deepcopy(zero.ZERO_REWARD_CONFIG)
POSITIVE_REWARD_CONFIG = copy.deepcopy(zero.POSITIVE_RESTORE_REWARD_CONFIG)
POSITIVE_CONFIG_DELTA_FIELDS = frozenset(
    {"morphology_weight", "morphology_causal_allow_effects"}
)

V26_EVENT_CONTRACT_ID = q3.EVENT_CONTRACT_ID
V26_BINARY_MODE = q3.V26_BINARY_MODE
V26_ACTOR_EVENT_SOURCE = q3.V26_ACTOR_EVENT_SOURCE
V26_RUNTIME_CONFIG = {
    "phase_fsm_input_mode": "legacy_events",
    "event_contract_id": q3.LEGACY_EVENT_CONTRACT_ID,
    "binary_phase_fsm_mode": q3.V26_BINARY_MODE,
    "binary_phase_detector_profile": q3.DETECTOR_PROFILE_PATH.as_posix(),
    "binary_phase_detector_profile_sha256": q3.DETECTOR_PROFILE_SHA256,
    "detector_sample_dt_s": 0.001,
    "binary_phase_debounce_s": 0.005,
    "binary_phase_event_contract_id": q3.EVENT_CONTRACT_ID,
    "actor_event_source": q3.V26_ACTOR_EVENT_SOURCE,
    "target_contract_id": q3.TARGET_CONTRACT_ID,
    "policy_step_s": 0.01,
}
CAUSAL_RUNTIME_ID = "h0_v12r8_morph_strict_terminal_delay_v1"
CAUSAL_RUNTIME_CONFIG = {
    "runtime_id": CAUSAL_RUNTIME_ID,
    "implementation": (ROOT / "h0_v12r8_morphology_causal_runtime.py").as_posix(),
    "training_launcher": (ROOT / "run_h0_v12r8_morphology_training.py").as_posix(),
    "child_process_site_hook": (ROOT / "sitecustomize.py").as_posix(),
    "terminal_samples_younger_than_delay": "drop_fail_safe",
}
PROFILE_ATTESTATIONS = copy.deepcopy(zero.PROFILE_ATTESTATIONS)

EXPECTED_ACTOR_FEATURES = q3.EXPECTED_ACTOR_FEATURES
EXPECTED_FULL_FEATURES = q3.EXPECTED_FULL_FEATURES
EXPECTED_HIDDENS = q3.EXPECTED_HIDDENS
EXPECTED_ACTION_DIM = q3.EXPECTED_ACTION_SHAPE[0]
EXPECTED_SIGMA = 0.005
STANDARD_RL_MODULE_KIND = "standard"

STREAM_NAMES = (
    "observations",
    "actions",
    "dynamics",
    "events",
    "reward_without_morphology",
    "morphology_loss",
)
STREAM_ENCODING = "canonical_json_utf8_v1"
CAUSAL_SAMPLE_FIELDS = frozenset(
    {
        "time_s",
        "emitted_time_s",
        "delay_s",
        "terminal_flush",
        "segment_type",
        "segment_start_time_s",
        "anchor_confirmed_time_s",
        "anchor_delivered_time_s",
        "duration_basis_s",
        "phase",
        "knee_served_ref_rad",
        "ankle_served_ref_rad",
        "knee_min_rad",
        "knee_max_rad",
        "ankle_min_rad",
        "ankle_max_rad",
        "knee_loss",
        "ankle_loss",
        "knee_excursion_rad",
        "ankle_excursion_rad",
        "inside_score",
        "knee_hard_excursion_rad",
        "ankle_hard_excursion_rad",
    }
)
CAUSAL_DIAGNOSTIC_FIELDS = frozenset(
    {
        "event_contract_id",
        "delay_s",
        "failed_closed",
        "failure_reason",
        "drop_reason",
        "dropped_sample_count",
        "dropped_pending_sample_count",
        "dropped_wait_hs_sample_count",
        "pending_sample_count",
        "resolved_sample_count",
        "total_resolved_sample_count",
        "total_dropped_sample_count",
        "cancelled_transition_count",
        "total_cancelled_transition_count",
        "timeout_transition_count",
        "terminal_flushed",
        "actor_state_name",
        "partial_stance_active",
    }
)
EXPECTED_CAUSAL_DROP_REASONS = frozenset(
    {
        "before_initial_partial_to",
        "before_first_physical_hs",
        "partial_stance_before_anchor",
        "wait_hs_before_anchor",
        "episode_end_pending_transition",
        "episode_end_without_physical_anchor",
        "episode_end_before_delay",
    }
)
MIN_AGGREGATE_NONZERO_EFFECT_SAMPLES = 1

REQUIRED_PAIR_CHECKS = (
    "identity",
    "control_before_positive",
    "same_binding_condition",
    "exact_config_delta",
    "artifacts_hash_bound",
    "v26_integrity",
    "observations_identical",
    "actions_identical",
    "dynamics_identical",
    "events_identical",
    "base_reward_and_loss_identical",
    "reward_recomposition_exact",
    "causal_per_sample_valid",
    "causal_diagnostics_identical",
    "causal_runtime_identical",
    "zero_updates",
)

FINAL_TRAINING_ITERATIONS = zero.FINAL_TRAINING_ITERATIONS
TRAINING_ENTRYPOINT = ROOT / "run_h0_v12r8_morphology_training.py"
FINAL_OUTPUT_DIR = zero.FINAL_TRAINING_OUTPUT

AUTHORITY = {
    "latent_live_source_authorized": True,
    "freeze_requires_three_terminal_semantic_passes": True,
    "control_source_mode": CONTROL_SOURCE_MODE,
    "q3_control_reuse_authorized": False,
    "local_control_rollouts": 6,
    "local_positive_rollouts": 6,
    "sole_positive_weight": MORPHOLOGY_POSITIVE_WEIGHT,
    "weight_sweep_authorized": False,
    "positive_weight_005_authorized": False,
    "actor_updates_authorized": False,
    "critic_updates_authorized": False,
    "ppo_updates_authorized": False,
    "checkpoint_mutation_authorized": False,
    "hard_termination_authorized": False,
    "retry_authorized": False,
    "resume_authorized": False,
    "training_authorized_before_terminal_pass": False,
}


def candidate_id_for_tree(tree_sha256: str) -> str:
    return r8.candidate_id(tree_sha256)


def canonical_case(case_id: str) -> dict[str, Any]:
    return copy.deepcopy(q3.canonical_case(case_id))


def reward_config_for_role(role: str) -> dict[str, Any]:
    if role == CONTROL_ROLE:
        return copy.deepcopy(CONTROL_REWARD_CONFIG)
    if role == POSITIVE_ROLE:
        return copy.deepcopy(POSITIVE_REWARD_CONFIG)
    raise ValueError(f"unsupported morphology role: {role!r}")


def arm_root(case_id: str, role: str) -> PurePosixPath:
    canonical_case(case_id)
    if role not in PAIR_ROLE_ORDER:
        raise ValueError(f"unsupported morphology role: {role!r}")
    return PAIR_ROOT / case_id / role


def pair_gate_path(case_id: str) -> PurePosixPath:
    canonical_case(case_id)
    return PAIR_ROOT / case_id / "pair_gate.json"


def pair_receipt_path(case_id: str) -> PurePosixPath:
    canonical_case(case_id)
    return PAIR_ROOT / case_id / "pair_receipt.json"


def _platform_path(value: str | PurePosixPath, platform_id: str) -> str:
    pure = PurePosixPath(value)
    if platform_id == "macos_arm64":
        return pure.as_posix()
    if platform_id == "windows_x86_64":
        return str(PureWindowsPath(*pure.parts))
    raise ValueError(f"unsupported platform: {platform_id!r}")


def final_training_argv(platform_id: str) -> tuple[str, ...]:
    inherited = list(zero.resume_training_argv(platform_id))
    inherited[1] = _platform_path(TRAINING_ENTRYPOINT, platform_id)
    argv = tuple(inherited)
    if (
        "--resume-from" not in argv
        or "--warm-start" in argv
        or "--warm-start-raw" in argv
        or argv[argv.index("--iterations") + 1] != "50"
        or json.loads(argv[argv.index("--reward-json") + 1])
        != POSITIVE_REWARD_CONFIG
    ):
        raise RuntimeError("final morphology command is not the exact resume interface")
    return argv


def render_command(argv: tuple[str, ...], platform_id: str) -> str:
    if platform_id == "macos_arm64":
        return shlex.join(argv)
    if platform_id == "windows_x86_64":
        return subprocess.list2cmdline(argv)
    raise ValueError(f"unsupported platform: {platform_id!r}")


def contract_self_check() -> dict[str, Any]:
    changed = {
        name
        for name, value in CONTROL_REWARD_CONFIG.items()
        if POSITIVE_REWARD_CONFIG.get(name) != value
    }
    checks = {
        "native_endpoints": R8_TERMINAL_ENDPOINT["protocol_id"] == r8.PROTOCOL_ID
        and Q3_TERMINAL_ENDPOINT["protocol_id"] == q3.PROTOCOL_ID
        and ZERO_TERMINAL_ENDPOINT["protocol_id"] == zero.PROTOCOL_ID,
        "forced_physical_fallback": CONTROL_SOURCE_MODE == "paired_rerun"
        and EXPECTED_LOCAL_ROLLOUT_COUNT == 12,
        "six_cases": len(CASE_IDS) == EXPECTED_PAIR_COUNT == 6
        and CASE_IDS == tuple(q3.CASE_IDS),
        "exact_config_delta": changed == POSITIVE_CONFIG_DELTA_FIELDS
        and CONTROL_REWARD_CONFIG["morphology_weight"] == 0.0
        and POSITIVE_REWARD_CONFIG["morphology_weight"] == 0.0025
        and CONTROL_REWARD_CONFIG["morphology_causal_allow_effects"] == 0.0
        and POSITIVE_REWARD_CONFIG["morphology_causal_allow_effects"] == 1.0,
        "causal": MORPHOLOGY_REWARD_DELAY_S == 0.04
        and MORPHOLOGY_MAX_DELIVERY_LATENCY_S == 0.01
        and POSITIVE_REWARD_CONFIG["morphology_hard_termination_enabled"] == 0.0,
        "v26": V26_RUNTIME_CONFIG["binary_phase_fsm_mode"] == "binary_active"
        and V26_RUNTIME_CONFIG["binary_phase_event_contract_id"]
        == q3.EVENT_CONTRACT_ID,
        "resume_only": all(
            "--resume-from" in final_training_argv(platform)
            for platform in ("macos_arm64", "windows_x86_64")
        )
        and all(
            final_training_argv(platform)[1]
            == _platform_path(TRAINING_ENTRYPOINT, platform)
            for platform in ("macos_arm64", "windows_x86_64")
        ),
    }
    return {"passed": all(checks.values()), "checks": checks}


__all__ = [name for name in globals() if name.isupper()] + [
    "arm_root",
    "candidate_id_for_tree",
    "canonical_case",
    "contract_self_check",
    "final_training_argv",
    "pair_gate_path",
    "pair_receipt_path",
    "render_command",
    "reward_config_for_role",
]
