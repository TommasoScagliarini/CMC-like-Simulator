"""Immutable contract for post-holdout H0 V4 autonomous qualification.

This module is pure and performs no filesystem or simulation work.  It fixes
the six condition-matched cases, numerical gates, lineage paths, and closed
authority before either a baseline or candidate rollout can be claimed.
"""

from __future__ import annotations

from pathlib import PurePosixPath
from typing import Any


SCHEMA_VERSION = 4
REVISION = "2026-08-06"
SOURCE_PROTOCOL_ID = "AB06_H0_PRIMARY_GRF_SPLIT_V4_FULL_MEAN"
PROTOCOL_ID = "AB06_H0_PRIMARY_GRF_SPLIT_V4_AUTONOMOUS_QUALIFICATION"
CANDIDATE_ID = "H0_primary_split_v4_full_mean"
LOCK_STATUS = "H0_PRIMARY_SPLIT_V4_QUALIFICATION_UNLOCKED"
ROLLOUT_PASS_STATUS = "PASS_H0_PRIMARY_SPLIT_V4_QUALIFICATION_ROLLOUT"
CASE_PASS_STATUS = "PASS_H0_PRIMARY_SPLIT_V4_QUALIFICATION_CASE"
PASS_STATUS = "PASS_H0_PRIMARY_SPLIT_V4_AUTONOMOUS_QUALIFICATION"
FAIL_STATUS = "FAIL_H0_PRIMARY_SPLIT_V4_AUTONOMOUS_QUALIFICATION"
SO_POLICY_ID = "verified_status0_max_iter_v1"
EVENT_CONTRACT_ID = "primary_grf_split_v1+legacy_events_v1"
PHASE_FSM_INPUT_MODE = "legacy_events"
MORPHOLOGY_WEIGHT = 0.0
EXPECTED_STEPS = 500
EXPECTED_CONTROL_WINDOWS = 5000
MINIMUM_VALID_CYCLES = 2
PENETRATION_LIMIT_M = 0.025
EXPECTED_ACTOR_FEATURES = 35
EXPECTED_FULL_FEATURES = 84
EXPECTED_DTYPE = "float32"
EXPECTED_ACTION_SHAPE = (2,)
STOCHASTIC_SIGMA = 0.005
CANONICAL_OFFSET_S = 1.956870983805102
DETERMINISTIC_RUNTIME_SEED = 123

V4_RUN_ROOT = PurePosixPath(
    "validation/h0_primary_grf_split_adaptation_runs/"
    "2026-08-06_h0_primary_split_v4_full_mean"
)
RUN_ROOT = V4_RUN_ROOT / "qualification"
LOCK_PATH = PurePosixPath(
    "validation/h0_primary_grf_split_v4_qualification_lock.json"
)
BASELINE_RECEIPT_PATH = RUN_ROOT / "baseline_receipt.json"
BASELINE_MANIFEST_PATH = RUN_ROOT / "baseline_manifest.json"
DECISION_RECEIPT_PATH = RUN_ROOT / "baseline_tolerance_decision_receipt.json"
EXECUTION_LEDGER_PATH = RUN_ROOT / "qualification_execution_ledger.json"

V4_EXECUTION_LOCK_PATH = PurePosixPath(
    "validation/h0_primary_grf_split_v4_execution_lock.json"
)
V4_EXECUTION_LEDGER_PATH = V4_RUN_ROOT / "execution_ledger.json"
HOLDOUT_RECEIPT_PATH = V4_RUN_ROOT / "holdout" / "receipt.json"
HOLDOUT_GATE_PATH = V4_RUN_ROOT / "holdout" / "gate.json"
CANDIDATE_FREEZE_PATH = V4_RUN_ROOT / "adaptation" / "candidate_freeze.json"
CANDIDATE_MODULE_PATH = (
    V4_RUN_ROOT / "adaptation" / "rl_module_target_adapted"
)
SOURCE_H0_MODULE_PATH = PurePosixPath(
    "validation/critic_warmup/"
    "2026-07-13_markov35_phase_aligned_sigma0005_iter1_retry/rl_module_last"
)
NOISE_MANIFEST_PATH = PurePosixPath(
    "validation/h0_primary_grf_split_noise_tapes/manifest.json"
)

CASE_IDS = (
    "deterministic_offset_minus_0p20",
    "deterministic_offset_nominal",
    "deterministic_offset_plus_0p20",
    "stochastic_nominal_seed_126",
    "stochastic_nominal_seed_127",
    "stochastic_nominal_seed_128",
)

_NOISE_ROOT = PurePosixPath("validation/h0_primary_grf_split_noise_tapes")
_CASE_ROWS = (
    (
        CASE_IDS[0],
        "deterministic",
        CANONICAL_OFFSET_S - 0.2,
        None,
        _NOISE_ROOT / "qualification_deterministic_all_zero.npz",
    ),
    (
        CASE_IDS[1],
        "deterministic",
        CANONICAL_OFFSET_S,
        None,
        _NOISE_ROOT / "qualification_deterministic_all_zero.npz",
    ),
    (
        CASE_IDS[2],
        "deterministic",
        CANONICAL_OFFSET_S + 0.2,
        None,
        _NOISE_ROOT / "qualification_deterministic_all_zero.npz",
    ),
    (
        CASE_IDS[3],
        "stochastic",
        CANONICAL_OFFSET_S,
        126,
        _NOISE_ROOT / "qualification_trial_02_standard_normal.npz",
    ),
    (
        CASE_IDS[4],
        "stochastic",
        CANONICAL_OFFSET_S,
        127,
        _NOISE_ROOT / "qualification_trial_04_standard_normal.npz",
    ),
    (
        CASE_IDS[5],
        "stochastic",
        CANONICAL_OFFSET_S,
        128,
        _NOISE_ROOT / "qualification_trial_08_standard_normal.npz",
    ),
)

JOINTS = ("pros_knee_angle", "pros_ankle_angle")
SEA_SIGNALS = (
    "torque_error_nm",
    "tau_spring_nm",
    "tau_spring_rate_nm_s",
    "motor_speed_rad_s",
    "motor_accel_rad_s2",
    "motor_power_w",
)
CONTINUOUS_AGGREGATIONS = ("rms", "abs_max")

# Explicitly accepted before any qualification execution.  V4 changes the
# candidate actor lineage, not these physical comparison tolerances.
RESERVE_TOLERANCES = (
    ("reserve_norm_nm.rms", 5.0, 0.05),
    ("reserve_norm_nm.abs_max", 5.0, 0.05),
    ("residual_norm_nm.rms", 1.0e-6, 0.05),
    ("residual_norm_nm.abs_max", 1.0e-6, 0.05),
)
SEA_TOLERANCES = tuple(
    (f"{joint}.{signal}.{aggregation}", 1.0e-6, 0.05)
    for joint in JOINTS
    for signal in SEA_SIGNALS
    for aggregation in CONTINUOUS_AGGREGATIONS
)

# Raw, independently verified bounded-LS recovery is diagnostic.  Only
# unaccepted solver recovery and SEA fallback close the gate.
ZERO_REQUIRED_COUNTS = (
    "action_clipped_values",
    "fallback_count",
    "timeout_count",
    "safety_stop_count",
    "sea_plugin_fallback_count",
    "so_solver_unaccepted_hard_fallback_count",
    "so_solver_unaccepted_bounded_ls_count",
    "so_solver_reuse_previous_count",
    "so_solver_bounds_violation_count",
    "so_solver_nonfinite_count",
    "so_solver_selected_infeasible_count",
    "so_solver_selected_solution_mismatch_count",
    "so_solver_residual_contract_mismatch_count",
    "hard_invalid_count",
    "nonfinite_count",
)

NONINCREASING_COUNTS = (
    "invalid_event_count",
    "pros_knee_angle.tau_input_saturated.count",
    "pros_ankle_angle.tau_input_saturated.count",
)

AUTHORITY = {
    "autonomous_qualification_execution_authorized": True,
    "baseline_and_tolerances_explicitly_accepted": True,
    "actor_updates_authorized": False,
    "critic_updates_authorized": False,
    "ppo_updates_authorized": False,
    "zero_update_port_authorized": False,
    "v25_abc_execution_authorized": False,
    "protected_trial_access_authorized": False,
    "reserve_trial_access_authorized": False,
    "runtime_promotion_authorized": False,
    "primary_grf_modification_authorized": False,
    "sea_semantic_modification_authorized": False,
}


def canonical_cases() -> tuple[dict[str, Any], ...]:
    """Return fresh descriptions of the exact six qualification cases."""

    result: list[dict[str, Any]] = []
    for case_id, selection, offset, action_seed, tape in _CASE_ROWS:
        runtime_seed = (
            DETERMINISTIC_RUNTIME_SEED
            if action_seed is None
            else int(action_seed)
        )
        result.append(
            {
                "case_id": case_id,
                "action_selection": selection,
                "episode_start_offset_s": float(offset),
                "action_seed": action_seed,
                "runtime_seed": runtime_seed,
                "sigma": STOCHASTIC_SIGMA if selection == "stochastic" else 0.0,
                "noise_tape": tape.as_posix(),
                "expected_steps": EXPECTED_STEPS,
            }
        )
    return tuple(result)


def rollout_destination(role: str, case_id: str) -> PurePosixPath:
    if role not in {"baseline", "candidate"}:
        raise ValueError(f"unknown qualification role: {role}")
    if case_id not in CASE_IDS:
        raise ValueError(f"unknown qualification case: {case_id}")
    return RUN_ROOT / role / case_id


def gate_destination(case_id: str) -> PurePosixPath:
    if case_id not in CASE_IDS:
        raise ValueError(f"unknown qualification case: {case_id}")
    return RUN_ROOT / "gates" / f"{case_id}.json"


def tolerance_rows() -> dict[str, list[dict[str, float | str]]]:
    def convert(
        rows: tuple[tuple[str, float, float], ...],
    ) -> list[dict[str, float | str]]:
        return [
            {
                "metric": metric,
                "absolute_tolerance": absolute,
                "relative_tolerance": relative,
            }
            for metric, absolute, relative in rows
        ]

    return {
        "sea": convert(SEA_TOLERANCES),
        "reserve": convert(RESERVE_TOLERANCES),
    }


__all__ = [
    "AUTHORITY",
    "BASELINE_MANIFEST_PATH",
    "BASELINE_RECEIPT_PATH",
    "CANDIDATE_FREEZE_PATH",
    "CANDIDATE_ID",
    "CANDIDATE_MODULE_PATH",
    "CASE_IDS",
    "DECISION_RECEIPT_PATH",
    "EXECUTION_LEDGER_PATH",
    "FAIL_STATUS",
    "LOCK_PATH",
    "LOCK_STATUS",
    "NONINCREASING_COUNTS",
    "PASS_STATUS",
    "PROTOCOL_ID",
    "RUN_ROOT",
    "SEA_TOLERANCES",
    "SO_POLICY_ID",
    "SOURCE_H0_MODULE_PATH",
    "SOURCE_PROTOCOL_ID",
    "V4_EXECUTION_LEDGER_PATH",
    "V4_EXECUTION_LOCK_PATH",
    "ZERO_REQUIRED_COUNTS",
    "canonical_cases",
    "gate_destination",
    "rollout_destination",
    "tolerance_rows",
]
