"""Pure contract for the one-shot V9 hybrid-teacher closed-loop probe.

This is a development-only, zero-update diagnostic.  It answers one narrow
question left open by the terminal V9R1 DAgger rollout: whether the privileged
H0 teacher used for the labels is itself viable when its deterministic mean is
served in the exact V26 binary-active runtime.
"""

from __future__ import annotations

import copy
import math
from pathlib import PurePosixPath
from typing import Any, Mapping

from validation import h0_primary_split_v9r1_residual_dagger_contract as v9r1
from validation import h0_primary_split_v9r1_teacher_compat_contract as layout


SCHEMA_VERSION = 92
REVISION = "2026-08-07"
PROTOCOL_ID = "AB06_H0_PRIMARY_SPLIT_V9_HYBRID_TEACHER_CLOSED_LOOP_PROBE"
PROBE_ID = "H0_V9_HYBRID_TEACHER_V26_ONE_SHOT_ZERO_UPDATE"
SOURCE_H0_ID = v9r1.SOURCE_H0_ID
TEACHER_ID = "H0_ANALOG_LOAD_CONTACT_WITH_V26_EVENTS_FSM_V1"
EVENT_CONTRACT_ID = v9r1.EVENT_CONTRACT_ID
SO_POLICY_ID = v9r1.SO_POLICY_ID

PREFLIGHT_STATUS = "PASS_H0_PRIMARY_SPLIT_V9_HYBRID_TEACHER_PROBE_PREFLIGHT"
LOCK_STATUS = "H0_PRIMARY_SPLIT_V9_HYBRID_TEACHER_PROBE_FROZEN"
CLAIM_STATUS = "H0_PRIMARY_SPLIT_V9_HYBRID_TEACHER_PROBE_EXECUTION_CLAIMED"
WORKER_CLAIM_STATUS = "H0_PRIMARY_SPLIT_V9_HYBRID_TEACHER_PROBE_WORKER_CLAIMED"
ROLLOUT_STATUS = "H0_PRIMARY_SPLIT_V9_HYBRID_TEACHER_PROBE_PERSISTED_UNGATED"
ROLLOUT_PASS_STATUS = "PASS_H0_PRIMARY_SPLIT_V9_HYBRID_TEACHER_PROBE"
ROLLOUT_FAIL_STATUS = "FAIL_H0_PRIMARY_SPLIT_V9_HYBRID_TEACHER_PROBE"
PROTOCOL_PASS_STATUS = "PASS_H0_PRIMARY_SPLIT_V9_HYBRID_TEACHER_PROBE_PROTOCOL"
PROTOCOL_FAIL_STATUS = "FAIL_H0_PRIMARY_SPLIT_V9_HYBRID_TEACHER_PROBE_PROTOCOL"

EXPECTED_STEPS = 500
EXPECTED_SAMPLES_PER_STEP = 10
EXPECTED_CONTROL_WINDOWS = 5000
EXPECTED_RAW_SENSOR_SAMPLES = 5000
EXPECTED_ACTOR_FEATURES = 35
EXPECTED_FULL_FEATURES = 84
EXPECTED_ACTION_DIM = 2
EXPECTED_DTYPE = "float32"
EXPECTED_SAMPLE_DT_S = 0.001
EXPECTED_POLICY_DT_S = 0.01
MORPHOLOGY_WEIGHT = 0.0
PENETRATION_LIMIT_M = 0.025
MINIMUM_VALID_CYCLES = 2
PRIVILEGED_INDICES = (10, 11)
EXPECTED_ACTOR_FEATURE_NAMES = layout.EXPECTED_ACTOR_FEATURE_NAMES
EXPECTED_OBSERVATION_FEATURE_NAMES = layout.EXPECTED_OBSERVATION_FEATURE_NAMES

CASE_ID = "deterministic_offset_minus_0p20"
_SOURCE_CASE = v9r1.canonical_case(CASE_ID, dagger=True)
CASE = {
    "case_id": CASE_ID,
    "action_selection": "deterministic",
    "episode_start_offset_s": _SOURCE_CASE["episode_start_offset_s"],
    "action_seed": None,
    "runtime_seed": _SOURCE_CASE["runtime_seed"],
    "sigma": 0.0,
}
CASE_IDS = (CASE_ID,)

RUN_ROOT = PurePosixPath(
    "validation/h0_primary_grf_split_adaptation_runs/"
    "2026-08-07_h0_primary_split_v9_hybrid_teacher_probe"
)
ROLLOUT_ROOT = RUN_ROOT / CASE_ID
PREFLIGHT_PATH = PurePosixPath(
    "validation/h0_primary_split_v9_hybrid_teacher_probe_preflight_receipt.json"
)
LOCK_PATH = PurePosixPath(
    "validation/h0_primary_split_v9_hybrid_teacher_probe_execution_lock.json"
)
CLAIM_PATH = RUN_ROOT / "execution_claim.json"
WORKER_CLAIM_PATH = RUN_ROOT / "worker_claim.json"
LEDGER_PATH = RUN_ROOT / "execution_ledger.json"

SOURCE_H0_MODULE_PATH = PurePosixPath(
    "validation/critic_warmup/"
    "2026-07-13_markov35_phase_aligned_sigma0005_iter1_retry/rl_module_last"
)

SOURCE_RELATIVE_PATHS = {
    "contract": "validation/h0_primary_split_v9_hybrid_teacher_probe_contract.py",
    "runner": "validation/run_h0_primary_split_v9_hybrid_teacher_probe.py",
    "contract_tests": (
        "validation/test_h0_primary_split_v9_hybrid_teacher_probe_contract.py"
    ),
    "runner_tests": (
        "validation/test_run_h0_primary_split_v9_hybrid_teacher_probe.py"
    ),
    "causal_teacher_helper": "validation/h0_primary_split_v9_causal_teacher.py",
    "causal_teacher_contract": (
        "validation/h0_primary_split_v9_causal_teacher_contract.py"
    ),
    "environment_config_source": (
        "validation/run_h0_primary_split_v9_causal_teacher.py"
    ),
    "v9r1_layout_compat": (
        "validation/h0_primary_split_v9r1_teacher_compat_contract.py"
    ),
    "forensic_writer": "validation/h0_forensic_rollout.py",
    "so_classifier": "validation/h0_v3_so_recovery_contract.py",
    "environment": "Trajectory Generator/osim_trj_cmc_like.py",
    "binary_phase_adapter": "Trajectory Generator/binary_phase_adapter.py",
    "binary_phase_fsm": "Trajectory Generator/binary_phase_fsm.py",
    "binary_phase_detector": "binary_phase_detector.py",
}

INPUT_RELATIVE_PATHS = {
    "v9r1_terminal_ledger": v9r1.PIPELINE_LEDGER_PATH.as_posix(),
    "v9_causal_teacher_ledger": (
        "validation/h0_primary_grf_split_adaptation_runs/"
        "2026-08-07_h0_primary_split_v9_v26_causal_residual/"
        "causal_teacher/execution_ledger.json"
    ),
    "v26_development_receipt": "validation/binary_phase_fsm_v26_development_receipt.json",
    "v26_v7_replay_receipt": "validation/binary_phase_fsm_v26_v7_replay_receipt.json",
    "source_h0_config": (
        "validation/critic_warmup/"
        "2026-07-13_markov35_phase_aligned_sigma0005_iter1_retry/"
        "training_cfg.resolved.yaml"
    ),
    "source_h0_module_state": (SOURCE_H0_MODULE_PATH / "module_state.pkl").as_posix(),
    "source_h0_module_ctor": (
        SOURCE_H0_MODULE_PATH / "class_and_ctor_args.pkl"
    ).as_posix(),
    "source_h0_module_metadata": (
        SOURCE_H0_MODULE_PATH / "metadata.json"
    ).as_posix(),
    "v25_profile": (
        "validation/binary_phase_detector_v25_geometry_runs/"
        "2026-08-04_local_reach_sweep_dev02_04_08/"
        "selected_candidate_profile.json"
    ),
}

AUTHORITY = {
    "authority_date": "2026-08-07",
    "authority_text": "ok allora procedi con la risoluzione",
    "development_hybrid_teacher_probe_authorized": True,
    "one_shot_no_retry": True,
    "zero_update_probe": True,
    "actor_updates_authorized": False,
    "critic_updates_authorized": False,
    "ppo_updates_authorized": False,
    "protected_trial_access_authorized": False,
    "reserve_trial_access_authorized": False,
}


def canonical_case(case_id: str) -> dict[str, Any]:
    if case_id != CASE_ID:
        raise ValueError(f"unknown V9 hybrid-teacher probe case: {case_id!r}")
    return copy.deepcopy(CASE)


def _nonnegative_int(value: Any) -> bool:
    return type(value) is int and value >= 0


def _finite(value: Any) -> bool:
    return (
        not isinstance(value, bool)
        and isinstance(value, (int, float))
        and math.isfinite(float(value))
    )


def rollout_gate(summary: Mapping[str, Any]) -> dict[str, Any]:
    """Classify the persisted one-shot rollout without tolerating partial data."""

    zero_fields = (
        "causal_column_mismatch_count",
        "teacher_query_mismatch_count",
        "action_clipped_values",
        "nonfinite_count",
        "timeout_count",
        "invalid_event_count",
        "hard_invalid_count",
        "so_solver_unaccepted_count",
        "sea_plugin_fallback_count",
        "routing_failure_count",
        "step_contract_failure_count",
        "binary_event_failure_count",
        "safety_stop_count",
    )
    checks = {
        "schema": summary.get("schema_version") == SCHEMA_VERSION,
        "ungated_status": summary.get("status") == ROLLOUT_STATUS,
        "protocol": summary.get("protocol_id") == PROTOCOL_ID,
        "probe": summary.get("probe_id") == PROBE_ID,
        "case": summary.get("case_id") == CASE_ID,
        "condition": summary.get("action_selection") == "deterministic"
        and summary.get("episode_start_offset_s") == CASE["episode_start_offset_s"]
        and summary.get("runtime_seed") == CASE["runtime_seed"],
        "behavior": summary.get("behavior")
        == "H0_HYBRID_TEACHER_MEAN_CLOSED_LOOP_V26_BINARY_ACTIVE",
        "teacher": summary.get("teacher_id") == TEACHER_ID,
        "only_10_11_privileged": summary.get("teacher_privileged_indices")
        == list(PRIVILEGED_INDICES),
        "binary_active": summary.get("binary_phase_fsm_mode") == "binary_active",
        "event_contract": summary.get("event_contract_id") == EVENT_CONTRACT_ID,
        "morphology_zero": _finite(summary.get("morphology_weight"))
        and float(summary["morphology_weight"]) == MORPHOLOGY_WEIGHT,
        "steps": summary.get("steps") == EXPECTED_STEPS,
        "samples": summary.get("raw_sensor_sample_count")
        == EXPECTED_RAW_SENSOR_SAMPLES,
        "windows": summary.get("control_window_count")
        == EXPECTED_CONTROL_WINDOWS,
        "time_limit": summary.get("end_reason") == "episode_time_limit"
        and summary.get("terminated") is False
        and summary.get("truncated") is True,
        "cycles": _nonnegative_int(summary.get("phase_valid_cycle_count"))
        and summary["phase_valid_cycle_count"] >= MINIMUM_VALID_CYCLES,
        "penetration": _finite(summary.get("grf_penetration_max_m"))
        and float(summary["grf_penetration_max_m"]) < PENETRATION_LIMIT_M,
        "zero_failures": all(
            _nonnegative_int(summary.get(name)) and summary[name] == 0
            for name in zero_fields
        ),
        "layout": summary.get("n_actor") == EXPECTED_ACTOR_FEATURES
        and summary.get("n_observation") == EXPECTED_FULL_FEATURES
        and summary.get("observation_dtype") == EXPECTED_DTYPE
        and summary.get("actor_feature_names")
        == list(EXPECTED_ACTOR_FEATURE_NAMES)
        and summary.get("observation_feature_names")
        == list(EXPECTED_OBSERVATION_FEATURE_NAMES),
        "no_updates": summary.get("actor_updates") == 0
        and summary.get("critic_updates") == 0
        and summary.get("ppo_updates") == 0,
        "no_candidate": summary.get("candidate_created") is False,
        "protected_closed": summary.get("protected_trials_opened") == [],
        "reserve_closed": summary.get("reserve_trials_opened") == [],
    }
    passed = all(checks.values())
    return {
        "schema_version": SCHEMA_VERSION,
        "status": ROLLOUT_PASS_STATUS if passed else ROLLOUT_FAIL_STATUS,
        "passed": passed,
        "protocol_id": PROTOCOL_ID,
        "case_id": summary.get("case_id"),
        "checks": checks,
        "authority": copy.deepcopy(AUTHORITY),
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
    }


__all__ = [name for name in globals() if name.isupper()] + [
    "canonical_case",
    "rollout_gate",
]
