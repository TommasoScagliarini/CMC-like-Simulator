"""Pure, fail-closed contract for H0 primary-split V6 qualification.

V6 is intentionally source-only at this stage.  It preregisters fresh,
previously unopened qualification conditions and the physical gates, but it
does not identify or authorize a candidate.  Qualification can be unlocked
only by a future immutable candidate-freeze receipt and a condition-matched
development PASS for that same candidate.

Importing this module performs no filesystem, simulation, or random-number
work.
"""

from __future__ import annotations

from pathlib import PurePosixPath
from typing import Any


SCHEMA_VERSION = 6
REVISION = "2026-08-06"
PROTOCOL_ID = "AB06_H0_PRIMARY_SPLIT_V6_BINARY_EVENT_QUALIFICATION"
ACCESS_STATUS = "LOCKED_PENDING_CANDIDATE_FREEZE_AND_DEVELOPMENT_PASS"
UNLOCK_STATUS = "H0_PRIMARY_SPLIT_V6_QUALIFICATION_UNLOCKED"
ROLLOUT_PASS_STATUS = "PASS_H0_PRIMARY_SPLIT_V6_QUALIFICATION_ROLLOUT"
CASE_PASS_STATUS = "PASS_H0_PRIMARY_SPLIT_V6_QUALIFICATION_CASE"
PASS_STATUS = "PASS_H0_PRIMARY_SPLIT_V6_QUALIFICATION"
FAIL_STATUS = "FAIL_H0_PRIMARY_SPLIT_V6_QUALIFICATION"
NOISE_TAPES_STATUS = "H0_PRIMARY_SPLIT_V6_QUALIFICATION_NOISE_TAPES_FROZEN"

CANDIDATE_FREEZE_REQUIRED_STATUS = "H0_PRIMARY_SPLIT_V6_CANDIDATE_FROZEN"
DEVELOPMENT_PASS_REQUIRED_STATUS = "PASS_H0_PRIMARY_SPLIT_V6_DEVELOPMENT"
CANDIDATE_ID_POLICY = "read_from_future_candidate_freeze_receipt"

BASELINE_ROLE = "baseline"
CANDIDATE_ROLE = "candidate"
BASELINE_ACTOR_ID = "original_h0"
BASELINE_CONTRACT_ID = "original_h0+counterfactual_analog+legacy_events"
CANDIDATE_EVENT_TARGET_ID = "binary_point_v25+functional_contact_fsm_v1"
PRIMARY_LOAD_CONTRACT_ID = "primary_grf_split_v1"
SO_POLICY_ID = "verified_status0_max_iter_v1"
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
DETERMINISTIC_RUNTIME_SEED = 129
STOCHASTIC_SEEDS = (130, 131, 132, 133)

RUN_ROOT = PurePosixPath("validation/h0_primary_split_v6_qualification_runs")
NOISE_ROOT = PurePosixPath("validation/h0_primary_split_v6_qualification_noise_tapes")
NOISE_MANIFEST_PATH = NOISE_ROOT / "manifest.json"

CASE_IDS = (
    "deterministic_offset_minus_0p30",
    "deterministic_offset_plus_0p30",
    "stochastic_nominal_seed_130",
    "stochastic_nominal_seed_131",
    "stochastic_nominal_seed_132",
    "stochastic_nominal_seed_133",
)

_ZERO_TAPE = NOISE_ROOT / "deterministic_all_zero.npz"
_CASE_ROWS = (
    (
        CASE_IDS[0],
        "deterministic",
        CANONICAL_OFFSET_S - 0.30,
        None,
        DETERMINISTIC_RUNTIME_SEED,
        _ZERO_TAPE,
    ),
    (
        CASE_IDS[1],
        "deterministic",
        CANONICAL_OFFSET_S + 0.30,
        None,
        DETERMINISTIC_RUNTIME_SEED,
        _ZERO_TAPE,
    ),
    *tuple(
        (
            f"stochastic_nominal_seed_{seed}",
            "stochastic",
            CANONICAL_OFFSET_S,
            seed,
            seed,
            NOISE_ROOT / f"stochastic_seed_{seed}_standard_normal.npz",
        )
        for seed in STOCHASTIC_SEEDS
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

# V6 inherits V5's condition-matched reserve and SEA tolerances unchanged.
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

# Raw, independently verified bounded-LS recovery remains diagnostic, exactly
# as in V5.  Invalid events are stricter in V6: they must be zero rather than
# merely no worse than the baseline.
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
    "invalid_event_count",
    "nonfinite_count",
)

NONINCREASING_COUNTS = (
    "pros_knee_angle.tau_input_saturated.count",
    "pros_ankle_angle.tau_input_saturated.count",
)

# Source construction and pure testing are authorized.  Materialization and
# execution are not authorized by this source contract alone.
AUTHORITY = {
    "qualification_source_preparation_authorized": True,
    "noise_tape_materialization_requires_prerequisites": True,
    "qualification_execution_authorized": False,
    "candidate_freeze_required": True,
    "development_pass_required": True,
    "retry_authorized": False,
    "actor_updates_authorized": False,
    "critic_updates_authorized": False,
    "ppo_updates_authorized": False,
    "protected_trial_access_authorized": False,
    "reserve_trial_access_authorized": False,
    "runtime_promotion_authorized": False,
    "primary_grf_modification_authorized": False,
    "sea_semantic_modification_authorized": False,
}


def role_contract(role: str) -> dict[str, Any]:
    """Return a fresh description of one condition-matched rollout role."""

    common: dict[str, Any] = {
        "role": role,
        "primary_load_contract_id": PRIMARY_LOAD_CONTRACT_ID,
        "morphology_weight": MORPHOLOGY_WEIGHT,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
    }
    if role == BASELINE_ROLE:
        common.update(
            {
                "actor_id": BASELINE_ACTOR_ID,
                "contract_id": BASELINE_CONTRACT_ID,
                "actor_input_view": "historical_analog",
                "observation_semantics": "counterfactual_analog",
                "phase_fsm_input_mode": "legacy_events",
                "event_contract_id": "legacy_events",
                "binary_phase_fsm_mode": "disabled",
            }
        )
        return common
    if role == CANDIDATE_ROLE:
        common.update(
            {
                "actor_id": CANDIDATE_ID_POLICY,
                "contract_id": CANDIDATE_EVENT_TARGET_ID,
                "actor_input_view": "primary_split",
                "observation_semantics": PRIMARY_LOAD_CONTRACT_ID,
                # The environment keeps the historical actor-FSM input slot
                # named ``legacy_events``.  V25 replaces only the left event
                # source through the separately frozen binary-active bridge.
                "phase_fsm_input_mode": "legacy_events",
                "event_contract_id": CANDIDATE_EVENT_TARGET_ID,
                "binary_phase_fsm_mode": "binary_active",
            }
        )
        return common
    raise ValueError(f"unknown qualification role: {role}")


def prerequisite_requirements() -> tuple[dict[str, Any], ...]:
    """Return the two records required before qualification materialization."""

    return (
        {
            "name": "candidate_freeze",
            "required_status": CANDIDATE_FREEZE_REQUIRED_STATUS,
            "required_passed": None,
            "candidate_id_required": True,
        },
        {
            "name": "development_gate",
            "required_status": DEVELOPMENT_PASS_REQUIRED_STATUS,
            "required_passed": True,
            "candidate_id_required": True,
        },
    )


def canonical_cases() -> tuple[dict[str, Any], ...]:
    """Return fresh descriptions of the exact six unopened V6 cases."""

    result: list[dict[str, Any]] = []
    for case_id, selection, offset, action_seed, runtime_seed, tape in _CASE_ROWS:
        result.append(
            {
                "case_id": case_id,
                "action_selection": selection,
                "episode_start_offset_s": float(offset),
                "action_seed": action_seed,
                "runtime_seed": int(runtime_seed),
                "sigma": STOCHASTIC_SIGMA if selection == "stochastic" else 0.0,
                "noise_tape": tape.as_posix(),
                "expected_steps": EXPECTED_STEPS,
            }
        )
    return tuple(result)


def rollout_destination(role: str, case_id: str) -> PurePosixPath:
    if role not in {BASELINE_ROLE, CANDIDATE_ROLE}:
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
    "ACCESS_STATUS",
    "AUTHORITY",
    "BASELINE_CONTRACT_ID",
    "BASELINE_ROLE",
    "CANDIDATE_EVENT_TARGET_ID",
    "CANDIDATE_FREEZE_REQUIRED_STATUS",
    "CANDIDATE_ROLE",
    "CASE_IDS",
    "DEVELOPMENT_PASS_REQUIRED_STATUS",
    "FAIL_STATUS",
    "NOISE_MANIFEST_PATH",
    "NOISE_ROOT",
    "NONINCREASING_COUNTS",
    "PASS_STATUS",
    "PROTOCOL_ID",
    "RUN_ROOT",
    "SEA_TOLERANCES",
    "SO_POLICY_ID",
    "ZERO_REQUIRED_COUNTS",
    "canonical_cases",
    "gate_destination",
    "prerequisite_requirements",
    "role_contract",
    "rollout_destination",
    "tolerance_rows",
]
