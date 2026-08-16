"""Pure fail-closed contract for the V7 V25 early-contact diagnostic.

V6 is a terminal failure and is never retried or reinterpreted here.  V7 is
one development-only replay of the already-open deterministic nominal
condition.  Frozen V5 actions drive the historical actor while V25/V20 runs in
``binary_shadow``.  The replay exists only to decide whether the V6 toe-led
contact was a sustained physical landing or a transient toe scuff.

This module performs no filesystem, OpenSim, Torch, or RLlib work.  Its gates
are pure and its two non-ambiguous outcomes select a *subsequent* protocol;
they do not promote a runtime configuration or authorize an update.
"""

from __future__ import annotations

import copy
import hashlib
import json
import math
from pathlib import PurePosixPath
from typing import Any, Mapping, Sequence


SCHEMA_VERSION = 7
REVISION = "2026-08-06"
PROTOCOL_ID = "AB06_H0_PRIMARY_SPLIT_V7_V25_EARLY_CONTACT_DIAGNOSTIC"
COLLECTOR_ID = "H0_V7_V25_BINARY_SHADOW_NOMINAL_REPLAY"

PREFLIGHT_PASS_STATUS = "PASS_H0_V7_EARLY_CONTACT_DIAGNOSTIC_PREFLIGHT"
LOCK_STATUS = "H0_V7_EARLY_CONTACT_DIAGNOSTIC_FROZEN"
DIAGNOSTIC_COLLECTED_STATUS = "H0_V7_EARLY_CONTACT_DIAGNOSTIC_COLLECTED_UNGATED"
DIAGNOSTIC_PASS_STATUS = "PASS_H0_V7_EARLY_CONTACT_DIAGNOSTIC"
DIAGNOSTIC_FAIL_STATUS = "FAIL_H0_V7_EARLY_CONTACT_DIAGNOSTIC"
CLASSIFICATION_PASS_STATUS = "PASS_H0_V7_EARLY_CONTACT_CLASSIFICATION"
CLASSIFICATION_FAIL_STATUS = "FAIL_H0_V7_EARLY_CONTACT_CLASSIFICATION_AMBIGUOUS"

PERSISTENT_LANDING = "persistent_landing"
TRANSIENT_TOE_SCUFF = "transient_toe_scuff"
AMBIGUOUS_FAIL = "ambiguous_fail"

MIN_SWING_CANDIDATE_DECISION = (
    "NEW_CONTRACT_BINARY_ACTIVE_MIN_SWING_0P20_CANDIDATE"
)
ACTOR_CORRECTION_DECISION = "ACTOR_CORRECTION_REQUIRED_KEEP_MIN_SWING_0P25"
NO_DECISION = "NO_DECISION_FAIL_CLOSED"

SOURCE_H0_ID = "H0_MARKOV35_PHASE_ALIGNED_SIGMA0005_ITER1_RETRY"
SOURCE_OBSERVATION_CONTRACT_ID = "historical_analog"
SOURCE_EVENT_CONTRACT_ID = "primary_grf_split_v1+legacy_events_v1"
PRIMARY_LOAD_CONTRACT_ID = "primary_grf_split_v1"
PRIMARY_LOAD_EVIDENCE_ROLE = "offline_physical_corroboration_only"
CANONICAL_SCIENTIFIC_ORACLE = "prescribed_grf_event_ledger"
SO_POLICY_ID = "verified_status0_max_iter_v1"
SHADOW_EVENT_CONTRACT_ID = (
    "binary_point_v25+functional_contact_fsm_v1_shadow"
)
V20_EVENT_SOURCE = "binary_phase_fsm_v20"
MIN_SWING_CANDIDATE_CONTRACT_ID = (
    "primary_grf_split_v1+binary_point_v25+functional_contact_fsm_v1"
    "+actor_min_swing_0p20_v1"
)

EXPECTED_STEPS = 500
EXPECTED_CONTROL_WINDOWS = 5000
EXPECTED_RAW_SENSOR_SAMPLES = 5000
EXPECTED_PRIMARY_LOAD_SAMPLES = 5000
EXPECTED_PRIMARY_SAMPLES_PER_STEP = 10
EXPECTED_SAMPLES_PER_STEP = 10
EXPECTED_ACTOR_FEATURES = 35
EXPECTED_FULL_FEATURES = 84
EXPECTED_ACTION_DIM = 2
EXPECTED_OBSERVATION_DTYPE = "float32"
EXPECTED_LEGACY_PHASE_INVALID_EVENT_COUNT = 2
EXPECTED_LEGACY_PHASE_VALID_CYCLE_COUNT = 2
EXPECTED_SAMPLE_DT_S = 0.001
EXPECTED_POLICY_DT_S = 0.010
EXPECTED_EPISODE_DURATION_S = 5.0
EXPECTED_V20_DEBOUNCE_S = 0.005
MORPHOLOGY_WEIGHT = 0.0
PENETRATION_LIMIT_M = 0.025
TIME_TOLERANCE_S = 1.0e-9
EXPECTED_H0_POLICY_STD = 0.005
EXPECTED_H0_POLICY_STD_ARRAY_SHA256 = (
    "6a232a5f7f8e081b31257805849bd9a7941dfc786a6eebe127dfedb46c33d410"
)
PRIMARY_LOAD_ALIGNMENT_ID = (
    "control_window_preintegration_1ms_ledger_delivered_at_policy_boundary_"
    "no_interpolation_v1"
)
MINIMUM_VALID_CYCLES = 2
EXPECTED_PLATFORM_SYSTEM = "Darwin"
EXPECTED_PLATFORM_MACHINE = "arm64"
EXPECTED_PLATFORM_IDENTITY = {
    "system": "Darwin",
    "machine": "arm64",
    "python_version": "3.10.20",
    "python_implementation": "CPython",
    "python_executable": "/opt/anaconda3/envs/envCMC-rllib/bin/python3.10",
    "distributions": {
        "gymnasium": "1.2.2",
        "numpy": "2.2.6",
        "opensim": "4.5.2",
        "ray": "2.55.1",
        "scipy": "1.15.2",
        "torch": "2.10.0",
    },
}
V20_FINAL_AUDIT_PASS_STATUS = "PASS_H0_V7_V20_FINAL_STATE_AUDIT"
V20_FINAL_AUDIT_FAIL_STATUS = "FAIL_H0_V7_V20_FINAL_STATE_AUDIT"

# This is the exact V5/V6 nominal episode.  Absolute times are simulation
# times; ``episode_start_offset_s`` remains the public EnvConfig offset.
CASE_ID = "deterministic_offset_nominal_shadow_diagnostic"
EPISODE_START_OFFSET_S = 1.956870983805102
EPISODE_START_TIME_S = 13.946870983805102
RUNTIME_SEED = 123
CASE = {
    "case_id": CASE_ID,
    "action_selection": "deterministic",
    "behavior": "FROZEN_V5_RAW_ACTION_REPLAY",
    "episode_start_offset_s": EPISODE_START_OFFSET_S,
    "episode_start_time_s": EPISODE_START_TIME_S,
    "action_seed": None,
    "runtime_seed": RUNTIME_SEED,
    "sigma": 0.0,
    "binary_phase_fsm_mode": "binary_shadow",
    "binary_phase_event_contract_id": SHADOW_EVENT_CONTRACT_ID,
}
CASES = (CASE,)
CASE_IDS = (CASE_ID,)

# Immutable predecessor facts that opened V7.  The known pair must be observed
# again in shadow; a nearby but different pair does not inherit its meaning.
V6_PRECEDING_TO_TIME_S = 18.282870983806752
V6_EARLY_HS_ONSET_TIME_S = 18.515870983807037
V6_OBSERVED_SWING_S = V6_EARLY_HS_ONSET_TIME_S - V6_PRECEDING_TO_TIME_S
LEGACY_MIN_SWING_DURATION_S = 0.250
MIN_SWING_CANDIDATE_S = 0.200
EARLY_WINDOW_LOWER_S = MIN_SWING_CANDIDATE_S
EARLY_WINDOW_UPPER_S = LEGACY_MIN_SWING_DURATION_S

# Canonical-oracle-grounded physical classification.  V7 observes the existing
# pre-integration primary-GRF evaluation at each 1 ms control window.
PRIMARY_FORCE_THRESHOLD_N = 20.0
PRIMARY_CROSSING_DEADLINE_S = 0.060
PRIMARY_SUPPORT_MIN_FROM_PRECEDING_TO_S = LEGACY_MIN_SWING_DURATION_S
MIN_ACCEPTED_STANCE_S = 0.050
MAX_TRANSIENT_CONTACT_S = 0.050
EARLY_GEOMETRIC_PROXY_REASON = "EARLY_GEOMETRIC_PROXY"
ACTOR_OR_GEOMETRY_FOLLOWUP_DECISION = (
    "ACTOR_OR_GEOMETRY_FOLLOWUP_REQUIRED_KEEP_MIN_SWING_0P25"
)

RUN_ROOT = PurePosixPath(
    "validation/h0_primary_grf_split_adaptation_runs/"
    "2026-08-06_h0_primary_split_v7_v25_early_contact_diagnostic"
)
PREFLIGHT_RECEIPT_PATH = PurePosixPath(
    "validation/h0_primary_split_v7_early_contact_preflight_receipt.json"
)
LOCK_PATH = PurePosixPath(
    "validation/h0_primary_split_v7_early_contact_execution_lock.json"
)
EXECUTION_LEDGER_PATH = RUN_ROOT / "execution_ledger.json"
EXECUTION_CLAIM_PATH = RUN_ROOT / "execution_claim.json"
WORKER_CLAIMS_ROOT = RUN_ROOT / "worker_claims"

V5_NOMINAL_ROOT = PurePosixPath(
    "validation/h0_primary_grf_split_adaptation_runs/"
    "2026-08-06_h0_primary_split_v5_full_mean/qualification/baseline/"
    "deterministic_offset_nominal"
)
V6_ROOT = PurePosixPath(
    "validation/h0_primary_grf_split_adaptation_runs/"
    "2026-08-06_h0_primary_split_v6_v25_residual/teacher_replay"
)
V6_NOMINAL_ROOT = V6_ROOT / "deterministic_offset_nominal"

SOURCE_RELATIVE_PATHS = {
    "contract": "validation/h0_primary_split_v7_early_contact_contract.py",
    "collector": (
        "validation/run_h0_primary_split_v7_early_contact_diagnostic.py"
    ),
    "preflight_builder": (
        "validation/build_h0_primary_split_v7_early_contact_preflight.py"
    ),
    "freezer": "validation/freeze_h0_primary_split_v7_early_contact.py",
    "environment": "Trajectory Generator/osim_trj_cmc_like.py",
    "binary_phase_fsm": "Trajectory Generator/binary_phase_fsm.py",
    "binary_phase_detector": "binary_phase_detector.py",
    "so_classifier": "validation/h0_v3_so_recovery_contract.py",
    "legacy_v25_runner": "validation/run_h0_v25_abc_preflight.py",
    "teacher_replay_runner": (
        "validation/run_h0_primary_grf_split_v6_teacher_replay.py"
    ),
    "contract_tests": (
        "validation/test_h0_primary_split_v7_early_contact_contract.py"
    ),
    "collector_tests": (
        "validation/test_run_h0_primary_split_v7_early_contact_diagnostic.py"
    ),
}

INPUT_RELATIVE_PATHS = {
    "v5_nominal_trace": (V5_NOMINAL_ROOT / "trace.json").as_posix(),
    "v5_nominal_summary": (V5_NOMINAL_ROOT / "summary.json").as_posix(),
    "v5_nominal_receipt": (V5_NOMINAL_ROOT / "receipt.json").as_posix(),
    "v6_execution_lock": (
        "validation/h0_primary_grf_split_v6_teacher_replay_execution_lock.json"
    ),
    "v3_runtime_execution_lock": (
        "validation/h0_primary_grf_split_v3_execution_lock.json"
    ),
    "v3_platform_receipt": (
        "validation/h0_primary_grf_split_v3_platform_receipt.json"
    ),
    "primary_core_lock": "validation/primary_grf_core_lock_2026-08-03.json",
    "v6_terminal_ledger": (V6_ROOT / "execution_ledger.json").as_posix(),
    "v6_nominal_failure": (V6_NOMINAL_ROOT / "failure.json").as_posix(),
    "v6_nominal_last_step": (V6_NOMINAL_ROOT / "steps/000457.json").as_posix(),
    "source_h0_config": (
        "validation/critic_warmup/"
        "2026-07-13_markov35_phase_aligned_sigma0005_iter1_retry/"
        "training_cfg.resolved.yaml"
    ),
    "source_h0_module_state": (
        "validation/critic_warmup/"
        "2026-07-13_markov35_phase_aligned_sigma0005_iter1_retry/"
        "rl_module_last/module_state.pkl"
    ),
    "source_h0_module_ctor": (
        "validation/critic_warmup/"
        "2026-07-13_markov35_phase_aligned_sigma0005_iter1_retry/"
        "rl_module_last/class_and_ctor_args.pkl"
    ),
    "source_h0_module_metadata": (
        "validation/critic_warmup/"
        "2026-07-13_markov35_phase_aligned_sigma0005_iter1_retry/"
        "rl_module_last/metadata.json"
    ),
    "v25_candidate_freeze": (
        "validation/binary_phase_detector_v25_development_candidate_freeze_lock.json"
    ),
    "v25_profile": (
        "validation/binary_phase_detector_v25_geometry_runs/"
        "2026-08-04_local_reach_sweep_dev02_04_08/"
        "selected_candidate_profile.json"
    ),
    "primary_grf_profile": (
        "online_grf_profiles/"
        "AB06_SEASEA_stiff321_500_pi_grf_correct_magnitude.json"
    ),
    "analog_teacher_profile": (
        "online_grf_profiles/"
        "AB06_SEASEA_stiff321_500_pi_grf_detector_HS-TO.json"
    ),
}

EXPECTED_INPUT_SHA256 = {
    "v5_nominal_trace": (
        "1c28c751827c535286581f6f13f54da9beca44cda75564023e827fa7bb7e86ba"
    ),
    "v5_nominal_summary": (
        "4dcc04443e6e611c345ef5024d813807e07d485faacf1ffa47114cc3edaec11d"
    ),
    "v5_nominal_receipt": (
        "5b54f4aa3ce4140917a04575f5740ae268d9216703aa359f07d9666574c8be88"
    ),
    "v6_execution_lock": (
        "bfd39462bdd789635d0697cd05151c11ecfbb2c6aee8fdc01d5b3bd1ab074198"
    ),
    "v3_runtime_execution_lock": (
        "b373cd76cf64c1ccdc2c5ff67f2818b46c389fd9a8087d5407a46c03b5de204d"
    ),
    "v3_platform_receipt": (
        "bfd33eea8f29ef603d0efd94cd5884a4bbd1b39d7083d4bd3b9646834475f077"
    ),
    "primary_core_lock": (
        "e9347bb5e5d04e84ce96f5c3ed354e154d26d0bc5abb750dc68bb8b79d0c06ac"
    ),
    "v6_terminal_ledger": (
        "d5523d781c72190ba61df8fe778fe2f0b2fb75e7e7057825c4f721bfd54a17bd"
    ),
    "v6_nominal_failure": (
        "c7eb1af64a5ed963ac0346781c06cba5f3937c2f9bffd6d11f99bc47bc09e41a"
    ),
    "v6_nominal_last_step": (
        "8371402b56e6b2b2216dbe0ef9c0209a7ec0a97419fe34c82307428a8912a843"
    ),
    "source_h0_config": (
        "6904f7a9000b63b5c1aab661ebcab4974dffdd1cfb8c731df6a953fc9234229e"
    ),
    "source_h0_module_state": (
        "44457ca5df7fa0e0e1f1d361d940136917fe8f71e984a1b0afaefb8ca3ced33b"
    ),
    "source_h0_module_ctor": (
        "5c98f006d99a71a0f1ddcbb31d8d73fe0a6dade8401e679f6af5b1bc943b4228"
    ),
    "source_h0_module_metadata": (
        "3a032ba54abcee8c9bcbb39e72fa05566912e94461d01f3c6228dc60e088bf12"
    ),
    "v25_candidate_freeze": (
        "04ecfe68937bc0d4baa3be9ab9b62060b20eb92c2f218f8540db1cebe423d346"
    ),
    "v25_profile": (
        "db704e502b99e49bea6d89493812bafdac748f8ce8d3ce28214ff624078539a2"
    ),
    "primary_grf_profile": (
        "09e04ab94954703d74acc3a80b24ecefcc07d3fc918c03b9e9df8116a6c1a2b0"
    ),
    "analog_teacher_profile": (
        "61ea948a3c0613e5c0e684a3197de118c7116e36188fca6993da79ce713fd99e"
    ),
}

AUTHORITY = {
    "development_diagnostic_authorized": True,
    "single_nominal_replay_authorized": True,
    "binary_shadow_execution_authorized": True,
    "v5_frozen_action_replay_authorized": True,
    "read_v6_terminal_evidence_authorized": True,
    "deterministic_branch_decision_authorized": True,
    "supervisor_only_execution_required": True,
    "v6_retry_authorized": False,
    "v6_terminal_failure_reinterpretation_authorized": False,
    "binary_active_execution_authorized": False,
    "min_swing_runtime_change_authorized": False,
    "actor_correction_execution_authorized": False,
    "actor_updates_authorized": False,
    "critic_updates_authorized": False,
    "ppo_updates_authorized": False,
    "training_authorized": False,
    "protected_trial_access_authorized": False,
    "reserve_trial_access_authorized": False,
    "runtime_promotion_authorized": False,
    "primary_grf_modification_authorized": False,
    "primary_online_grf_as_event_source_authorized": False,
    "primary_online_grf_as_scientific_oracle_authorized": False,
    "detector_geometry_modification_authorized": False,
    "binary_fsm_v20_modification_authorized": False,
    "sea_semantic_modification_authorized": False,
}

_CONTACT_STATE_BY_BITS = {
    (False, False): "AIR",
    (True, False): "HEEL",
    (True, True): "BOTH",
    (False, True): "TOE",
}
_CONTACT_STATE_ID = {"AIR": 0, "TOE": 1, "HEEL": 2, "BOTH": 3}


class _ContactContractError(ValueError):
    """Internal malformed-evidence signal converted to an ambiguous result."""


def canonical_case(case_id: str = CASE_ID) -> dict[str, Any]:
    """Return a fresh copy of the only authorized V7 diagnostic case."""

    if case_id != CASE_ID:
        raise ValueError(f"unknown V7 early-contact case: {case_id!r}")
    return copy.deepcopy(CASE)


def _finite_number(value: Any) -> bool:
    return (
        not isinstance(value, bool)
        and isinstance(value, (int, float))
        and math.isfinite(float(value))
    )


def _nonnegative_int(value: Any) -> bool:
    return type(value) is int and value >= 0


def _time_close(observed: Any, expected: Any) -> bool:
    return bool(
        _finite_number(observed)
        and _finite_number(expected)
        and abs(float(observed) - float(expected)) <= TIME_TOLERANCE_S
    )


def _strict_sha256(payload: Any) -> str:
    try:
        encoded = json.dumps(
            payload,
            sort_keys=True,
            separators=(",", ":"),
            allow_nan=False,
        ).encode("utf-8")
    except (TypeError, ValueError) as exc:
        raise _ContactContractError(
            "classification evidence is not strict JSON"
        ) from exc
    return hashlib.sha256(encoded).hexdigest()


def _validate_raw_samples(
    raw_samples: Sequence[Mapping[str, Any]],
) -> list[dict[str, Any]]:
    if isinstance(raw_samples, (str, bytes)) or not isinstance(
        raw_samples, Sequence
    ):
        raise _ContactContractError("raw_samples must be a sequence")
    if len(raw_samples) != EXPECTED_RAW_SENSOR_SAMPLES:
        raise _ContactContractError(
            f"expected {EXPECTED_RAW_SENSOR_SAMPLES} raw samples"
        )
    expected_fields = {
        "time_s",
        "left_heel_contact",
        "left_toe_contact",
    }
    result: list[dict[str, Any]] = []
    for index, row in enumerate(raw_samples, start=1):
        if not isinstance(row, Mapping) or set(row) != expected_fields:
            raise _ContactContractError(f"raw sample {index} schema mismatch")
        time_s = row.get("time_s")
        heel = row.get("left_heel_contact")
        toe = row.get("left_toe_contact")
        expected_time = EPISODE_START_TIME_S + index * EXPECTED_SAMPLE_DT_S
        if (
            not _finite_number(time_s)
            or not _time_close(float(time_s), expected_time)
            or type(heel) is not bool
            or type(toe) is not bool
        ):
            raise _ContactContractError(f"raw sample {index} is invalid")
        result.append(
            {
                "time_s": float(time_s),
                "left_heel_contact": heel,
                "left_toe_contact": toe,
            }
        )
    return result


def _validate_baseline_sample(
    baseline_sample: Mapping[str, Any],
) -> dict[str, Any]:
    expected_fields = {
        "time_s",
        "left_heel_contact",
        "left_toe_contact",
    }
    if not isinstance(baseline_sample, Mapping) or set(
        baseline_sample
    ) != expected_fields:
        raise _ContactContractError("baseline raw sample schema mismatch")
    time_s = baseline_sample.get("time_s")
    heel = baseline_sample.get("left_heel_contact")
    toe = baseline_sample.get("left_toe_contact")
    if (
        not _time_close(time_s, EPISODE_START_TIME_S)
        or type(heel) is not bool
        or type(toe) is not bool
    ):
        raise _ContactContractError("baseline raw sample is invalid")
    return {
        "time_s": float(time_s),
        "left_heel_contact": heel,
        "left_toe_contact": toe,
    }


def _replay_v20(
    *,
    baseline: Mapping[str, Any],
    raw: Sequence[Mapping[str, Any]],
    policy_boundary_times: Sequence[float],
) -> dict[str, Any]:
    """Independently replay V20 from the complete baseline + raw journal."""

    baseline_bits = (
        bool(baseline["left_heel_contact"]),
        bool(baseline["left_toe_contact"]),
    )
    raw_state = _CONTACT_STATE_BY_BITS[baseline_bits]
    stable_contact_state = raw_state
    pending_contact_state: str | None = None
    pending_contact_since: float | None = None
    in_contact = any(baseline_bits)
    pending_phase_target: bool | None = None
    pending_phase_since: float | None = None
    pending_phase_onset_state: str | None = None
    cancellation_count = 0
    event_count = {"heel_strike": 0, "toe_off": 0}
    cycle_count = 0
    last_event: str | None = None
    last_hs: float | None = None
    last_to: float | None = None
    events: list[dict[str, Any]] = []
    events_this_step: list[dict[str, Any]] = []
    contact_transitions_this_step: list[dict[str, Any]] = []
    cancellations_this_step: list[dict[str, Any]] = []

    for zero_index, sample in enumerate(raw):
        if zero_index % EXPECTED_SAMPLES_PER_STEP == 0:
            events_this_step = []
            contact_transitions_this_step = []
            cancellations_this_step = []
        time_s = float(sample["time_s"])
        bits = (
            bool(sample["left_heel_contact"]),
            bool(sample["left_toe_contact"]),
        )
        current_state = _CONTACT_STATE_BY_BITS[bits]
        delivered_time_s = float(
            policy_boundary_times[zero_index // EXPECTED_SAMPLES_PER_STEP]
        )

        if current_state == stable_contact_state:
            pending_contact_state = None
            pending_contact_since = None
        elif current_state != pending_contact_state:
            pending_contact_state = current_state
            pending_contact_since = time_s
        elif (
            time_s - float(pending_contact_since) + TIME_TOLERANCE_S
            >= EXPECTED_V20_DEBOUNCE_S
        ):
            contact_onset = float(pending_contact_since)
            previous_stable = stable_contact_state
            stable_contact_state = current_state
            pending_contact_state = None
            pending_contact_since = None
            contact_transitions_this_step.append(
                {
                    "from": previous_stable,
                    "to": current_state,
                    "event_time_s": contact_onset,
                    "confirmed_time_s": time_s,
                    "delivered_time_s": delivered_time_s,
                }
            )

        raw_in_contact = any(bits)
        if raw_in_contact == in_contact:
            if pending_phase_target is not None:
                cancellations_this_step.append(
                    {
                        "event": (
                            "heel_strike" if pending_phase_target else "toe_off"
                        ),
                        "event_time_s": pending_phase_since,
                        "cancelled_time_s": time_s,
                        "reason": "raw_phase_returned_to_stable_latch",
                    }
                )
                cancellation_count += 1
            pending_phase_target = None
            pending_phase_since = None
            pending_phase_onset_state = None
        elif raw_in_contact != pending_phase_target:
            pending_phase_target = raw_in_contact
            pending_phase_since = time_s
            pending_phase_onset_state = current_state
        elif (
            time_s - float(pending_phase_since) + TIME_TOLERANCE_S
            >= EXPECTED_V20_DEBOUNCE_S
        ):
            target = bool(pending_phase_target)
            onset = float(pending_phase_since)
            onset_state = str(pending_phase_onset_state)
            in_contact = target
            pending_phase_target = None
            pending_phase_since = None
            pending_phase_onset_state = None
            event_name = "heel_strike" if target else "toe_off"
            event: dict[str, Any] = {
                "side": "left",
                "event": event_name,
                "time": onset,
                "event_time_s": onset,
                "confirmed_time_s": time_s,
                "confirmed_time": time_s,
                "delivered_time_s": delivered_time_s,
                "source": V20_EVENT_SOURCE,
                "event_contract_id": SHADOW_EVENT_CONTRACT_ID,
                "semantic": (
                    "functional_initial_contact"
                    if target
                    else "functional_final_contact"
                ),
                "onset_contact_state": onset_state,
                "confirmed_contact_state": current_state,
            }
            if target:
                contact_leader = {
                    "HEEL": "heel",
                    "TOE": "toe",
                    "BOTH": "both",
                }.get(onset_state, "unknown")
                event["landing_sensor"] = contact_leader
                event["contact_leader"] = contact_leader
                event["hs_semantics"] = "first_stable_any_contact"
                if (
                    last_hs is not None
                    and last_to is not None
                    and last_hs < last_to < onset
                ):
                    cycle_count += 1
                last_hs = onset
            else:
                event["startup_partial_stance"] = last_hs is None
                last_to = onset
            event_count[event_name] += 1
            last_event = event_name
            events.append(event)
            events_this_step.append(event)
        raw_state = current_state

    pending_event = None
    if pending_phase_target is not None:
        pending_event = {
            "event": "heel_strike" if pending_phase_target else "toe_off",
            "event_time_s": pending_phase_since,
            "onset_contact_state": pending_phase_onset_state,
        }
    return {
        "events": events,
        "final": {
            "last_sample_time_s": float(raw[-1]["time_s"]),
            "samples_processed": len(raw),
            "raw_contact_state": raw_state,
            "raw_contact_state_id": _CONTACT_STATE_ID[raw_state],
            "stable_contact_state": stable_contact_state,
            "stable_contact_state_id": _CONTACT_STATE_ID[stable_contact_state],
            "in_contact": bool(in_contact),
            "gait_phase": "STANCE" if in_contact else "SWING",
            "pending_event": pending_event,
            "pending_contact_state": pending_contact_state,
            "events_this_step": events_this_step,
            "contact_state_transitions_this_step": (
                contact_transitions_this_step
            ),
            "candidate_cancellations_this_step": cancellations_this_step,
            "candidate_cancellation_count": cancellation_count,
            "event_count": event_count,
            "cycle_count": cycle_count,
            "last_event": last_event,
            "last_hs_event_time_s": last_hs,
            "last_to_event_time_s": last_to,
        },
    }


def replay_v20_event_stream(
    *,
    baseline_sample: Mapping[str, Any],
    raw_samples: Sequence[Mapping[str, Any]],
    policy_boundary_times: Sequence[float],
) -> dict[str, Any]:
    """Public pure replay used by tests and the persisted supervisor audit."""

    baseline = _validate_baseline_sample(baseline_sample)
    raw = _validate_raw_samples(raw_samples)
    if (
        isinstance(policy_boundary_times, (str, bytes))
        or not isinstance(policy_boundary_times, Sequence)
        or len(policy_boundary_times) != EXPECTED_STEPS
        or any(
            not _time_close(
                value,
                EPISODE_START_TIME_S + index * EXPECTED_POLICY_DT_S,
            )
            for index, value in enumerate(policy_boundary_times, start=1)
        )
    ):
        raise _ContactContractError("policy boundary time ledger is invalid")
    return _replay_v20(
        baseline=baseline,
        raw=raw,
        policy_boundary_times=[float(value) for value in policy_boundary_times],
    )


def _validate_primary_load_samples(
    primary_load_samples: Sequence[Mapping[str, Any]],
    *,
    body_weight_n: float,
) -> list[dict[str, float]]:
    if not _finite_number(body_weight_n) or float(body_weight_n) <= 0.0:
        raise _ContactContractError("body_weight_n must be finite and positive")
    if isinstance(primary_load_samples, (str, bytes)) or not isinstance(
        primary_load_samples, Sequence
    ):
        raise _ContactContractError("primary_load_samples must be a sequence")
    if len(primary_load_samples) != EXPECTED_PRIMARY_LOAD_SAMPLES:
        raise _ContactContractError(
            f"expected {EXPECTED_PRIMARY_LOAD_SAMPLES} primary load samples"
        )
    expected_fields = {
        "sampled_time_s",
        "delivered_time_s",
        "left_normal_grf_bw",
        "left_normal_force_n",
        "left_in_contact",
    }
    result: list[dict[str, Any]] = []
    for index, row in enumerate(primary_load_samples, start=1):
        if not isinstance(row, Mapping) or set(row) != expected_fields:
            raise _ContactContractError(
                f"primary load sample {index} schema mismatch"
            )
        sampled_time_s = row.get("sampled_time_s")
        delivered_time_s = row.get("delivered_time_s")
        force_bw = row.get("left_normal_grf_bw")
        force_n_observed = row.get("left_normal_force_n")
        in_contact = row.get("left_in_contact")
        expected_sample = (
            EPISODE_START_TIME_S + (index - 1) * EXPECTED_SAMPLE_DT_S
        )
        expected_delivery = EPISODE_START_TIME_S + (
            (index - 1) // EXPECTED_PRIMARY_SAMPLES_PER_STEP + 1
        ) * EXPECTED_POLICY_DT_S
        if (
            not _finite_number(sampled_time_s)
            or not _finite_number(delivered_time_s)
            or not _time_close(float(sampled_time_s), expected_sample)
            or not _time_close(float(delivered_time_s), expected_delivery)
            or float(delivered_time_s) <= float(sampled_time_s)
            or float(delivered_time_s) - float(sampled_time_s)
            + TIME_TOLERANCE_S
            < EXPECTED_SAMPLE_DT_S
            or float(delivered_time_s) - float(sampled_time_s)
            > EXPECTED_POLICY_DT_S + TIME_TOLERANCE_S
            or not _finite_number(force_bw)
            or not _finite_number(force_n_observed)
            or float(force_bw) < 0.0
            or float(force_n_observed) < 0.0
            or type(in_contact) is not bool
        ):
            raise _ContactContractError(
                f"primary load sample {index} is invalid"
            )
        force_n = float(force_bw) * float(body_weight_n)
        if not math.isfinite(force_n):
            raise _ContactContractError(
                f"primary load sample {index} overflows force in newtons"
            )
        if not math.isclose(
            force_n,
            float(force_n_observed),
            rel_tol=1.0e-10,
            abs_tol=1.0e-9,
        ):
            raise _ContactContractError(
                f"primary load sample {index} has inconsistent N/BW values"
            )
        if force_n > PRIMARY_FORCE_THRESHOLD_N and in_contact is not True:
            raise _ContactContractError(
                f"primary load sample {index} has Fy>20 N without contact"
            )
        result.append(
            {
                "sampled_time_s": float(sampled_time_s),
                "delivered_time_s": float(delivered_time_s),
                "left_normal_grf_bw": float(force_bw),
                "left_normal_force_n": force_n,
                "left_in_contact": in_contact,
            }
        )
    return result


def _validate_shadow_events(
    shadow_events: Sequence[Mapping[str, Any]],
) -> list[dict[str, Any]]:
    if isinstance(shadow_events, (str, bytes)) or not isinstance(
        shadow_events, Sequence
    ):
        raise _ContactContractError("shadow_events must be a sequence")
    if not shadow_events:
        raise _ContactContractError("shadow_events cannot be empty")
    result: list[dict[str, Any]] = []
    previous_time = -math.inf
    previous_name: str | None = None
    for index, row in enumerate(shadow_events, start=1):
        if not isinstance(row, Mapping):
            raise _ContactContractError(f"shadow event {index} is malformed")
        name = row.get("event")
        event_time = row.get("event_time_s")
        confirmed_time = row.get("confirmed_time_s")
        delivered_time = row.get("delivered_time_s")
        expected_semantic = (
            "functional_initial_contact"
            if name == "heel_strike"
            else "functional_final_contact"
        )
        event_specific_valid = bool(
            (
                name == "heel_strike"
                and row.get("landing_sensor") in {"heel", "toe", "both"}
                and row.get("contact_leader") == row.get("landing_sensor")
                and row.get("hs_semantics") == "first_stable_any_contact"
            )
            or (
                name == "toe_off"
                and type(row.get("startup_partial_stance")) is bool
            )
        )
        if (
            name not in {"heel_strike", "toe_off"}
            or row.get("side") != "left"
            or row.get("source") != V20_EVENT_SOURCE
            or row.get("event_contract_id") != SHADOW_EVENT_CONTRACT_ID
            or row.get("semantic") != expected_semantic
            or not event_specific_valid
            or not all(
                _finite_number(value)
                for value in (
                    row.get("time"),
                    event_time,
                    row.get("confirmed_time"),
                    confirmed_time,
                    delivered_time,
                )
            )
            or not _time_close(row.get("time"), event_time)
            or not _time_close(row.get("confirmed_time"), confirmed_time)
            or float(event_time) <= previous_time
            or not _time_close(float(confirmed_time) - float(event_time), 0.005)
            or float(delivered_time) + TIME_TOLERANCE_S < float(confirmed_time)
            or float(delivered_time) - float(confirmed_time)
            > EXPECTED_POLICY_DT_S + TIME_TOLERANCE_S
            or previous_name == name
        ):
            raise _ContactContractError(f"shadow event {index} is invalid")
        result.append(copy.deepcopy(dict(row)))
        previous_time = float(event_time)
        previous_name = str(name)
    return result


def _ambiguous(reason: str, *, evidence_sha256: str | None = None) -> dict[str, Any]:
    return {
        "schema_version": SCHEMA_VERSION,
        "status": CLASSIFICATION_FAIL_STATUS,
        "passed": False,
        "classification": AMBIGUOUS_FAIL,
        "decision": NO_DECISION,
        "reason": str(reason),
        "evidence_sha256": evidence_sha256,
        "checks": {
            "evidence_valid": False,
            "known_early_pair_exact": False,
            "all_flights_analyzed": False,
            "no_flight_below_candidate_minimum": False,
            "classification_non_ambiguous": False,
        },
        "metrics": {},
        "candidate_contract_id": None,
        "candidate_min_swing_duration_s": None,
        "legacy_min_swing_duration_s": LEGACY_MIN_SWING_DURATION_S,
        "requires_followup_protocol": True,
    }


def classify_contact(
    *,
    baseline_sample: Mapping[str, Any],
    raw_samples: Sequence[Mapping[str, Any]],
    primary_load_samples: Sequence[Mapping[str, Any]],
    shadow_events: Sequence[Mapping[str, Any]],
    body_weight_n: float,
) -> dict[str, Any]:
    """Classify the V6 early toe contact from a complete V7 shadow replay.

    The primary-load stream contains the existing 1 ms pre-integration
    evaluations; no force is recomputed.  A persistent landing requires both
    uninterrupted V25 contact and an accepted primary stance.  A scuff must
    return to AIR within 50 ms before any accepted primary stance.  Every
    discordant or incomplete combination is deliberately ambiguous.
    """

    raw_evidence = {
        "baseline_sample": baseline_sample,
        "raw_samples": raw_samples,
        "primary_load_samples": primary_load_samples,
        "shadow_events": shadow_events,
        "body_weight_n": body_weight_n,
    }
    try:
        evidence_sha256 = _strict_sha256(raw_evidence)
        baseline = _validate_baseline_sample(baseline_sample)
        raw = _validate_raw_samples(raw_samples)
        primary = _validate_primary_load_samples(
            primary_load_samples,
            body_weight_n=float(body_weight_n),
        )
        events = _validate_shadow_events(shadow_events)
        replayed = _replay_v20(
            baseline=baseline,
            raw=raw,
            policy_boundary_times=[
                primary[index]["delivered_time_s"]
                for index in range(
                    EXPECTED_PRIMARY_SAMPLES_PER_STEP - 1,
                    len(primary),
                    EXPECTED_PRIMARY_SAMPLES_PER_STEP,
                )
            ],
        )
    except _ContactContractError as exc:
        return _ambiguous(str(exc))

    if _strict_sha256(events) != _strict_sha256(replayed["events"]):
        return _ambiguous(
            "shadow event journal is not the complete exact V20 replay of the "
            "baseline and 5000 raw samples",
            evidence_sha256=evidence_sha256,
        )

    for event in events:
        event_time = float(event["event_time_s"])
        confirmed_time = float(event["confirmed_time_s"])
        event_indices = [
            index
            for index, row in enumerate(raw)
            if _time_close(row["time_s"], event_time)
        ]
        if len(event_indices) != 1:
            return _ambiguous(
                "a shadow event onset is absent or duplicated in the raw journal",
                evidence_sha256=evidence_sha256,
            )
        event_index = event_indices[0]
        confirmation_indices = [
            index
            for index, row in enumerate(raw)
            if _time_close(row["time_s"], confirmed_time)
        ]
        if len(confirmation_indices) != 1:
            return _ambiguous(
                "a shadow event confirmation is absent or duplicated in the raw journal",
                evidence_sha256=evidence_sha256,
            )
        confirmation_index = confirmation_indices[0]
        stable_rows = raw[event_index : confirmation_index + 1]
        if event["event"] == "heel_strike":
            consistent = bool(
                stable_rows
                and all(
                    row["left_heel_contact"] or row["left_toe_contact"]
                    for row in stable_rows
                )
            )
        else:
            consistent = bool(
                stable_rows
                and all(
                    not row["left_heel_contact"]
                    and not row["left_toe_contact"]
                    for row in stable_rows
                )
            )
        if not consistent:
            return _ambiguous(
                "a shadow event lacks its complete raw debounce evidence",
                evidence_sha256=evidence_sha256,
            )

    flights: list[dict[str, float]] = []
    for previous, current in zip(events, events[1:]):
        if previous["event"] == "toe_off" and current["event"] == "heel_strike":
            duration = float(current["event_time_s"]) - float(
                previous["event_time_s"]
            )
            flights.append(
                {
                    "toe_off_time_s": float(previous["event_time_s"]),
                    "heel_strike_time_s": float(current["event_time_s"]),
                    "swing_duration_s": duration,
                }
            )
    if not flights:
        return _ambiguous("no completed V25 flight was observed", evidence_sha256=evidence_sha256)
    if any(
        not math.isfinite(row["swing_duration_s"])
        or row["swing_duration_s"] <= 0.0
        for row in flights
    ):
        return _ambiguous("a V25 flight duration is invalid", evidence_sha256=evidence_sha256)

    early = [
        row
        for row in flights
        if row["swing_duration_s"] + TIME_TOLERANCE_S
        >= EARLY_WINDOW_LOWER_S
        and row["swing_duration_s"] < EARLY_WINDOW_UPPER_S - TIME_TOLERANCE_S
    ]
    known = [
        row
        for row in early
        if _time_close(row["toe_off_time_s"], V6_PRECEDING_TO_TIME_S)
        and _time_close(row["heel_strike_time_s"], V6_EARLY_HS_ONSET_TIME_S)
    ]
    if len(early) != 1 or len(known) != 1:
        return _ambiguous(
            "the unique preregistered V6 early TO/HS pair was not reproduced",
            evidence_sha256=evidence_sha256,
        )
    candidate = known[0]
    minimum_flight = min(row["swing_duration_s"] for row in flights)
    if minimum_flight < MIN_SWING_CANDIDATE_S - TIME_TOLERANCE_S:
        return _ambiguous(
            "a V25 flight is shorter than the 0.20 s candidate guard",
            evidence_sha256=evidence_sha256,
        )

    onset = candidate["heel_strike_time_s"]
    onset_indices = [
        index for index, row in enumerate(raw) if _time_close(row["time_s"], onset)
    ]
    if len(onset_indices) != 1:
        return _ambiguous("early onset is absent or duplicated", evidence_sha256=evidence_sha256)
    onset_index = onset_indices[0]
    onset_row = raw[onset_index]
    if onset_row["left_heel_contact"] is not False or onset_row[
        "left_toe_contact"
    ] is not True:
        return _ambiguous("early onset is not toe-only", evidence_sha256=evidence_sha256)

    matching_hs = [
        event
        for event in events
        if event["event"] == "heel_strike"
        and _time_close(float(event["event_time_s"]), onset)
    ]
    if len(matching_hs) != 1 or matching_hs[0].get("landing_sensor") != "toe":
        return _ambiguous(
            "V20 did not report the early onset as toe-led",
            evidence_sha256=evidence_sha256,
        )

    raw_off_index: int | None = None
    for index in range(onset_index + 1, len(raw)):
        row = raw[index]
        if not row["left_heel_contact"] and not row["left_toe_contact"]:
            raw_off_index = index
            break
    raw_right_censored = raw_off_index is None
    if raw_right_censored:
        raw_off_time = None
        raw_last_active_time = float(raw[-1]["time_s"])
        raw_return_delay = None
    else:
        raw_off_time = float(raw[raw_off_index]["time_s"])
        raw_last_active_time = float(raw[raw_off_index - 1]["time_s"])
        raw_return_delay = raw_off_time - onset
    raw_observation_end_time = raw_last_active_time
    raw_duration = raw_last_active_time - onset

    later_events = [
        event for event in events if float(event["event_time_s"]) > onset
    ]
    if raw_right_censored:
        raw_event_consistent = not later_events
    else:
        raw_event_consistent = bool(
            later_events
            and later_events[0]["event"] == "toe_off"
            and _time_close(
                float(later_events[0]["event_time_s"]), raw_off_time
            )
        )
    if not raw_event_consistent:
        return _ambiguous(
            "raw contact end/censoring and subsequent V20 events disagree",
            evidence_sha256=evidence_sha256,
        )

    # Segment the *complete* 1 ms primary stream.  Searching only at/after
    # the V25 onset would incorrectly relabel a pre-existing physical support
    # run as though it began on the first post-HS endpoint.
    support_runs: list[dict[str, Any]] = []
    support_start_index: int | None = None
    for index, row in enumerate(primary):
        active = bool(
            row["left_normal_force_n"] > PRIMARY_FORCE_THRESHOLD_N
            and row["left_in_contact"] is True
        )
        if active and support_start_index is None:
            support_start_index = index
        if not active and support_start_index is not None:
            start = primary[support_start_index]
            last_active = primary[index - 1]
            previous_inactive = (
                primary[support_start_index - 1]
                if support_start_index > 0
                else None
            )
            support_runs.append(
                {
                    "start_index": support_start_index,
                    "onset_time_s": float(start["sampled_time_s"]),
                    "onset_lower_bound_s": (
                        None
                        if previous_inactive is None
                        else float(previous_inactive["sampled_time_s"])
                    ),
                    "onset_upper_bound_s": float(start["sampled_time_s"]),
                    "end_time_s": float(last_active["sampled_time_s"]),
                    "last_active_time_s": float(
                        last_active["sampled_time_s"]
                    ),
                    "first_inactive_time_s": float(row["sampled_time_s"]),
                    "duration_s": float(last_active["sampled_time_s"])
                    - float(start["sampled_time_s"]),
                    "right_censored": False,
                    "onset_identifiable": support_start_index > 0,
                }
            )
            support_start_index = None
    if support_start_index is not None:
        start = primary[support_start_index]
        previous_inactive = (
            primary[support_start_index - 1]
            if support_start_index > 0
            else None
        )
        support_runs.append(
            {
                "start_index": support_start_index,
                "onset_time_s": float(start["sampled_time_s"]),
                "onset_lower_bound_s": (
                    None
                    if previous_inactive is None
                    else float(previous_inactive["sampled_time_s"])
                ),
                "onset_upper_bound_s": float(start["sampled_time_s"]),
                "end_time_s": float(primary[-1]["sampled_time_s"]),
                "last_active_time_s": float(primary[-1]["sampled_time_s"]),
                "first_inactive_time_s": None,
                "duration_s": float(primary[-1]["sampled_time_s"])
                - float(start["sampled_time_s"]),
                "right_censored": True,
                "onset_identifiable": support_start_index > 0,
            }
        )

    accepted_flight_intersections = [
        run
        for run in support_runs
        if run["duration_s"] + TIME_TOLERANCE_S >= MIN_ACCEPTED_STANCE_S
        and run["end_time_s"] > candidate["toe_off_time_s"] + TIME_TOLERANCE_S
        and run["onset_time_s"] < onset - TIME_TOLERANCE_S
    ]
    unassociated_flight_support = [
        run
        for run in accepted_flight_intersections
        if run["end_time_s"] < onset - TIME_TOLERANCE_S
    ]
    if unassociated_flight_support:
        return _ambiguous(
            "accepted primary support intersects the V25 flight but ends before "
            "the early contact and cannot be causally associated",
            evidence_sha256=evidence_sha256,
        )
    overlapping_unidentifiable = [
        run
        for run in support_runs
        if run["onset_identifiable"] is False
        and run["end_time_s"] + TIME_TOLERANCE_S >= onset
    ]
    if overlapping_unidentifiable:
        return _ambiguous(
            "primary support is already active at the first endpoint; its "
            "physical onset is not identifiable",
            evidence_sha256=evidence_sha256,
        )
    accepted_overlapping_raw = [
        run
        for run in support_runs
        if run["duration_s"] + TIME_TOLERANCE_S >= MIN_ACCEPTED_STANCE_S
        and run["end_time_s"] + TIME_TOLERANCE_S >= onset
        and run["onset_time_s"] <= raw_observation_end_time + TIME_TOLERANCE_S
    ]
    pre_to_overlaps = [
        run
        for run in accepted_overlapping_raw
        if run["onset_time_s"]
        <= candidate["toe_off_time_s"] + TIME_TOLERANCE_S
    ]
    if pre_to_overlaps:
        return _ambiguous(
            "accepted primary support starts before TO and overlaps the early "
            "V25 episode",
            evidence_sha256=evidence_sha256,
        )
    accepted_post_to_runs = [
        run
        for run in support_runs
        if run["onset_identifiable"] is True
        and run["onset_time_s"] > candidate["toe_off_time_s"] + TIME_TOLERANCE_S
        and run["onset_time_s"] <= raw_observation_end_time + TIME_TOLERANCE_S
        and run["duration_s"] + TIME_TOLERANCE_S >= MIN_ACCEPTED_STANCE_S
    ]
    if len(accepted_post_to_runs) > 1:
        return _ambiguous(
            "multiple accepted primary support runs occur in the V25 flight",
            evidence_sha256=evidence_sha256,
        )
    associated_runs = [
        run
        for run in accepted_post_to_runs
        if run["end_time_s"] + TIME_TOLERANCE_S >= onset
    ]
    if accepted_post_to_runs and not associated_runs:
        return _ambiguous(
            "accepted primary support ends before the early V25 contact and "
            "cannot be uniquely associated",
            evidence_sha256=evidence_sha256,
        )
    selected_support = associated_runs[0] if associated_runs else None
    crossing_lower_bound: float | None = (
        float(selected_support["onset_lower_bound_s"])
        if selected_support is not None
        and selected_support["onset_lower_bound_s"] is not None
        else None
    )
    crossing_upper_bound: float | None = (
        float(selected_support["onset_upper_bound_s"])
        if selected_support is not None
        else None
    )
    primary_off_lower_bound: float | None = (
        float(selected_support["last_active_time_s"])
        if selected_support is not None
        else None
    )
    primary_off_upper_bound: float | None = (
        None
        if selected_support is None or selected_support["right_censored"]
        else float(selected_support["first_inactive_time_s"])
    )
    primary_duration = (
        0.0
        if selected_support is None
        else float(selected_support["duration_s"])
    )
    primary_right_censored = bool(
        selected_support is not None and selected_support["right_censored"]
    )
    primary_accepted = selected_support is not None
    raw_persistent = raw_duration + TIME_TOLERANCE_S >= MIN_ACCEPTED_STANCE_S
    overlap_duration = (
        0.0
        if selected_support is None
        else min(raw_observation_end_time, float(selected_support["end_time_s"]))
        - max(onset, float(selected_support["onset_time_s"]))
    )
    streams_overlap_for_dwell = bool(
        overlap_duration + TIME_TOLERANCE_S >= MIN_ACCEPTED_STANCE_S
    )
    transient_raw = bool(
        not raw_right_censored
        and raw_return_delay is not None
        and raw_return_delay <= MAX_TRANSIENT_CONTACT_S + TIME_TOLERANCE_S
    )

    common_checks = {
        "evidence_valid": True,
        "known_early_pair_exact": True,
        "all_flights_analyzed": True,
        "no_flight_below_candidate_minimum": True,
        "unique_early_pair": len(early) == 1,
        "toe_led_onset": True,
        "raw_contact_episode_observed_or_boundary_censored": True,
        "v20_to_matches_raw_air_return_or_boundary_censor": True,
        "all_shadow_events_have_raw_debounce_evidence": True,
    }
    metrics = {
        "completed_flight_count": len(flights),
        "minimum_swing_duration_s": minimum_flight,
        "early_flight_count": len(early),
        "preceding_to_time_s": candidate["toe_off_time_s"],
        "early_hs_onset_time_s": onset,
        "early_swing_duration_s": candidate["swing_duration_s"],
        "raw_contact_end_time_s": raw_off_time,
        "raw_last_active_time_s": raw_last_active_time,
        "raw_observation_end_time_s": raw_observation_end_time,
        "raw_guaranteed_contact_duration_s": raw_duration,
        "raw_return_to_air_delay_s": raw_return_delay,
        "raw_right_censored": raw_right_censored,
        "primary_force_threshold_n": PRIMARY_FORCE_THRESHOLD_N,
        "primary_onset_lower_bound_s": crossing_lower_bound,
        "primary_onset_upper_bound_s": crossing_upper_bound,
        "primary_onset_interval_open_closed": "(lower, upper]",
        "primary_crossing_delay_bounds_s": (
            [crossing_lower_bound - onset, crossing_upper_bound - onset]
            if crossing_lower_bound is not None
            and crossing_upper_bound is not None
            else None
        ),
        "primary_off_lower_bound_s": primary_off_lower_bound,
        "primary_off_upper_bound_s": primary_off_upper_bound,
        "primary_guaranteed_stance_duration_s": primary_duration,
        "primary_right_censored": primary_right_censored,
        "primary_support_run_count": len(support_runs),
        "primary_associated_run_count": len(associated_runs),
        "primary_raw_overlap_duration_s": overlap_duration,
        "primary_source_sample_dt_s": EXPECTED_SAMPLE_DT_S,
        "body_weight_n": float(body_weight_n),
    }

    persistent = raw_persistent and primary_accepted and streams_overlap_for_dwell
    guard_time = (
        candidate["toe_off_time_s"] + PRIMARY_SUPPORT_MIN_FROM_PRECEDING_TO_S
    )
    deadline_time = onset + PRIMARY_CROSSING_DEADLINE_S
    primary_support_definitely_within_deadline = bool(
        crossing_upper_bound is not None
        and crossing_upper_bound <= deadline_time + TIME_TOLERANCE_S
    )
    primary_support_definitely_after_guard = bool(
        crossing_lower_bound is not None
        and crossing_lower_bound >= guard_time - TIME_TOLERANCE_S
    )
    primary_support_definitely_before_guard = bool(
        crossing_upper_bound is not None
        and crossing_upper_bound <= guard_time + TIME_TOLERANCE_S
    )
    guard_relation = (
        "definitely_after"
        if primary_support_definitely_after_guard
        else (
            "definitely_before_or_at"
            if primary_support_definitely_before_guard
            else "straddles_guard"
        )
    )
    metrics["primary_support_from_preceding_to_bounds_s"] = (
        [
            crossing_lower_bound - candidate["toe_off_time_s"],
            crossing_upper_bound - candidate["toe_off_time_s"],
        ]
        if crossing_lower_bound is not None
        and crossing_upper_bound is not None
        else None
    )
    metrics["legacy_guard_time_s"] = guard_time
    metrics["primary_guard_relation"] = guard_relation

    if persistent and not primary_support_definitely_within_deadline:
        result = _ambiguous(
            "the interval-censored primary onset is not definitely within the "
            "60 ms causal association window",
            evidence_sha256=evidence_sha256,
        )
        result["reason_code"] = EARLY_GEOMETRIC_PROXY_REASON
        result["decision"] = ACTOR_OR_GEOMETRY_FOLLOWUP_DECISION
        result["checks"].update(common_checks)
        result["metrics"] = metrics
        return result

    if persistent and guard_relation == "straddles_guard":
        result = _ambiguous(
            "the interval-censored primary onset straddles the 0.25 s legacy "
            "guard and cannot select detector timing versus actor correction",
            evidence_sha256=evidence_sha256,
        )
        result["reason_code"] = "PRIMARY_SUPPORT_GUARD_INTERVAL_STRADDLE"
        result["decision"] = ACTOR_OR_GEOMETRY_FOLLOWUP_DECISION
        result["checks"].update(common_checks)
        result["metrics"] = metrics
        return result

    if persistent and primary_support_definitely_after_guard:
        classification = PERSISTENT_LANDING
        decision = MIN_SWING_CANDIDATE_DECISION
        candidate_contract_id: str | None = MIN_SWING_CANDIDATE_CONTRACT_ID
        candidate_min_swing: float | None = MIN_SWING_CANDIDATE_S
        reason = (
            "V25 contact and primary Fy jointly satisfy the frozen 50 ms "
            "landing criterion"
        )
        decision_basis = "detector_leads_support_beyond_legacy_guard"
        branch_checks = {
            "raw_contact_at_least_50ms": True,
            "primary_crossed_20n_within_60ms": True,
            "primary_stance_at_least_50ms": True,
            "raw_contact_covers_primary_dwell": True,
            "primary_support_within_60ms": True,
            "primary_support_definitely_after_legacy_guard": True,
            "primary_onset_interval_resolved_against_guard": True,
        }
    elif persistent and primary_support_definitely_before_guard:
        classification = PERSISTENT_LANDING
        decision = ACTOR_CORRECTION_DECISION
        candidate_contract_id = None
        candidate_min_swing = None
        reason = (
            "physical support begins before the legacy 0.25 s guard; lowering "
            "the guard would mask actor timing rather than detector lead"
        )
        decision_basis = "physical_support_precedes_legacy_guard"
        branch_checks = {
            "raw_contact_at_least_50ms": True,
            "primary_crossed_20n_within_60ms": True,
            "primary_stance_at_least_50ms": True,
            "raw_contact_covers_primary_dwell": True,
            "primary_support_definitely_before_legacy_guard": True,
            "primary_onset_interval_resolved_against_guard": True,
        }
    elif transient_raw and not primary_accepted:
        classification = TRANSIENT_TOE_SCUFF
        decision = ACTOR_CORRECTION_DECISION
        candidate_contract_id = None
        candidate_min_swing = None
        reason = (
            "V25 returned to AIR within 50 ms without an accepted primary stance"
        )
        decision_basis = "transient_contact_without_accepted_primary_stance"
        branch_checks = {
            "raw_returned_to_air_within_50ms": True,
            "no_accepted_primary_stance": True,
        }
    else:
        result = _ambiguous(
            "raw contact and primary-load evidence are intermediate or discordant",
            evidence_sha256=evidence_sha256,
        )
        result["checks"].update(common_checks)
        result["metrics"] = metrics
        return result

    checks = {
        **common_checks,
        **branch_checks,
        "classification_non_ambiguous": True,
    }
    return {
        "schema_version": SCHEMA_VERSION,
        "status": CLASSIFICATION_PASS_STATUS,
        "passed": True,
        "classification": classification,
        "decision": decision,
        "decision_basis": decision_basis,
        "reason": reason,
        "evidence_sha256": evidence_sha256,
        "checks": checks,
        "metrics": metrics,
        "primary_load_evidence_role": PRIMARY_LOAD_EVIDENCE_ROLE,
        "canonical_scientific_oracle": CANONICAL_SCIENTIFIC_ORACLE,
        "candidate_contract_id": candidate_contract_id,
        "candidate_min_swing_duration_s": candidate_min_swing,
        "legacy_min_swing_duration_s": LEGACY_MIN_SWING_DURATION_S,
        "requires_followup_protocol": True,
    }


def _classification_gate_passes(value: Any) -> bool:
    if not isinstance(value, Mapping):
        return False
    classification = value.get("classification")
    decision = value.get("decision")
    decision_valid = bool(
        (
            classification == PERSISTENT_LANDING
            and decision
            in {MIN_SWING_CANDIDATE_DECISION, ACTOR_CORRECTION_DECISION}
        )
        or (
            classification == TRANSIENT_TOE_SCUFF
            and decision == ACTOR_CORRECTION_DECISION
        )
    )
    checks = value.get("checks")
    metrics = value.get("metrics")
    common_check_names = (
        "evidence_valid",
        "known_early_pair_exact",
        "all_flights_analyzed",
        "no_flight_below_candidate_minimum",
        "unique_early_pair",
        "toe_led_onset",
        "raw_contact_episode_observed_or_boundary_censored",
        "v20_to_matches_raw_air_return_or_boundary_censor",
        "all_shadow_events_have_raw_debounce_evidence",
        "classification_non_ambiguous",
    )
    digest = value.get("evidence_sha256")
    common = bool(
        value.get("schema_version") == SCHEMA_VERSION
        and value.get("status") == CLASSIFICATION_PASS_STATUS
        and value.get("passed") is True
        and decision_valid
        and isinstance(checks, Mapping)
        and all(checks.get(name) is True for name in common_check_names)
        and isinstance(metrics, Mapping)
        and _nonnegative_int(metrics.get("completed_flight_count"))
        and metrics["completed_flight_count"] >= 1
        and metrics.get("early_flight_count") == 1
        and _finite_number(metrics.get("minimum_swing_duration_s"))
        and float(metrics["minimum_swing_duration_s"])
        >= MIN_SWING_CANDIDATE_S - TIME_TOLERANCE_S
        and _time_close(metrics.get("preceding_to_time_s"), V6_PRECEDING_TO_TIME_S)
        and _time_close(metrics.get("early_hs_onset_time_s"), V6_EARLY_HS_ONSET_TIME_S)
        and _time_close(metrics.get("early_swing_duration_s"), V6_OBSERVED_SWING_S)
        and isinstance(digest, str)
        and len(digest) == 64
        and all(character in "0123456789abcdef" for character in digest)
        and value.get("legacy_min_swing_duration_s")
        == LEGACY_MIN_SWING_DURATION_S
        and value.get("primary_load_evidence_role")
        == PRIMARY_LOAD_EVIDENCE_ROLE
        and value.get("canonical_scientific_oracle")
        == CANONICAL_SCIENTIFIC_ORACLE
        and value.get("requires_followup_protocol") is True
    )
    if not common:
        return False
    if (
        classification == PERSISTENT_LANDING
        and decision == MIN_SWING_CANDIDATE_DECISION
    ):
        return bool(
            checks.get("raw_contact_at_least_50ms") is True
            and checks.get("primary_crossed_20n_within_60ms") is True
            and checks.get("primary_stance_at_least_50ms") is True
            and checks.get("raw_contact_covers_primary_dwell") is True
            and checks.get("primary_support_within_60ms") is True
            and checks.get(
                "primary_support_definitely_after_legacy_guard"
            )
            is True
            and checks.get(
                "primary_onset_interval_resolved_against_guard"
            )
            is True
            and value.get("candidate_contract_id")
            == MIN_SWING_CANDIDATE_CONTRACT_ID
            and value.get("candidate_min_swing_duration_s")
            == MIN_SWING_CANDIDATE_S
            and value.get("decision_basis")
            == "detector_leads_support_beyond_legacy_guard"
        )
    if classification == PERSISTENT_LANDING:
        return bool(
            checks.get("raw_contact_at_least_50ms") is True
            and checks.get("primary_crossed_20n_within_60ms") is True
            and checks.get("primary_stance_at_least_50ms") is True
            and checks.get("raw_contact_covers_primary_dwell") is True
            and checks.get(
                "primary_support_definitely_before_legacy_guard"
            )
            is True
            and checks.get(
                "primary_onset_interval_resolved_against_guard"
            )
            is True
            and value.get("candidate_contract_id") is None
            and value.get("candidate_min_swing_duration_s") is None
            and value.get("decision_basis")
            == "physical_support_precedes_legacy_guard"
        )
    return bool(
        checks.get("raw_returned_to_air_within_50ms") is True
        and checks.get("no_accepted_primary_stance") is True
        and value.get("candidate_contract_id") is None
        and value.get("candidate_min_swing_duration_s") is None
        and value.get("decision_basis")
        == "transient_contact_without_accepted_primary_stance"
    )


def audit_v20_final_state(
    *,
    baseline_sample: Mapping[str, Any],
    policy_boundary_times: Sequence[float],
    final_payload: Mapping[str, Any],
    raw_samples: Sequence[Mapping[str, Any]],
    shadow_events: Sequence[Mapping[str, Any]],
) -> dict[str, Any]:
    """Audit cumulative V20 state against the persisted raw/event journal.

    A completed diagnostic must account for every 1 ms sample and every event;
    a pending debounce or cancelled candidate at the episode boundary is not
    silently accepted.
    """

    try:
        baseline = _validate_baseline_sample(baseline_sample)
        raw = _validate_raw_samples(raw_samples)
        events = _validate_shadow_events(shadow_events)
        replayed = replay_v20_event_stream(
            baseline_sample=baseline,
            raw_samples=raw,
            policy_boundary_times=policy_boundary_times,
        )
        if not isinstance(final_payload, Mapping):
            raise _ContactContractError("final V20 payload must be a mapping")
        event_count = final_payload.get("event_count")
        if (
            not isinstance(event_count, Mapping)
            or set(event_count) != {"heel_strike", "toe_off"}
            or not all(_nonnegative_int(value) for value in event_count.values())
        ):
            raise _ContactContractError("final V20 event_count is malformed")
        cycle_count = final_payload.get("cycle_count")
        cancellations = final_payload.get("candidate_cancellation_count")
        samples_processed = final_payload.get("samples_processed")
        if not all(
            _nonnegative_int(value)
            for value in (cycle_count, cancellations, samples_processed)
        ):
            raise _ContactContractError("final V20 counters are malformed")
        counts = {"heel_strike": 0, "toe_off": 0}
        reconstructed_cycles = 0
        last_hs: float | None = None
        last_to: float | None = None
        for event in events:
            name = str(event["event"])
            event_time = float(event["event_time_s"])
            counts[name] += 1
            if name == "heel_strike":
                if (
                    last_hs is not None
                    and last_to is not None
                    and last_hs < last_to < event_time
                ):
                    reconstructed_cycles += 1
                last_hs = event_time
            else:
                last_to = event_time
        last_raw = raw[-1]
        last_bits = (
            last_raw["left_heel_contact"],
            last_raw["left_toe_contact"],
        )
        expected_raw_state = {
            (False, False): "AIR",
            (True, False): "HEEL",
            (True, True): "BOTH",
            (False, True): "TOE",
        }[last_bits]
        expected_in_contact = bool(any(last_bits))
        expected_last_event = str(events[-1]["event"])
        replayed_final = replayed["final"]
        expected_end = EPISODE_START_TIME_S + EXPECTED_EPISODE_DURATION_S
        checks = {
            "source_exact": final_payload.get("source") == V20_EVENT_SOURCE,
            "event_contract_exact": final_payload.get("event_contract_id")
            == SHADOW_EVENT_CONTRACT_ID,
            "sample_dt_exact": _time_close(
                final_payload.get("sample_dt_s"), EXPECTED_SAMPLE_DT_S
            ),
            "debounce_exact": _time_close(
                final_payload.get("debounce_s"), EXPECTED_V20_DEBOUNCE_S
            ),
            "delivery_bound_exact": _time_close(
                final_payload.get("max_delivery_delay_s"), EXPECTED_POLICY_DT_S
            ),
            "all_samples_processed": samples_processed
            == EXPECTED_RAW_SENSOR_SAMPLES
            == len(raw),
            "last_sample_is_episode_end": _time_close(
                final_payload.get("last_sample_time_s"), expected_end
            )
            and _time_close(final_payload.get("last_sample_time_s"), raw[-1]["time_s"]),
            "event_counts_match_journal": dict(event_count) == counts
            and sum(counts.values()) == len(events)
            and dict(event_count) == replayed_final["event_count"],
            "complete_event_stream_exact": _strict_sha256(events)
            == _strict_sha256(replayed["events"]),
            "cycle_count_matches_journal": cycle_count == reconstructed_cycles,
            "minimum_cycle_count": cycle_count >= MINIMUM_VALID_CYCLES,
            "no_pending_event": final_payload.get("pending_event") is None
            and final_payload.get("pending_event")
            == replayed_final["pending_event"],
            "no_pending_contact_transition": final_payload.get(
                "pending_contact_state"
            )
            is None
            and final_payload.get("pending_contact_state")
            == replayed_final["pending_contact_state"],
            "zero_candidate_cancellations": cancellations == 0
            and cancellations == replayed_final["candidate_cancellation_count"],
            "final_raw_state_matches_journal": final_payload.get(
                "raw_contact_state"
            )
            == expected_raw_state
            == replayed_final["raw_contact_state"]
            and final_payload.get("raw_contact_state_id")
            == replayed_final["raw_contact_state_id"]
            and final_payload.get("stable_contact_state")
            == replayed_final["stable_contact_state"]
            and final_payload.get("stable_contact_state_id")
            == replayed_final["stable_contact_state_id"]
            and final_payload.get("in_contact") is expected_in_contact
            and final_payload.get("in_contact")
            is replayed_final["in_contact"]
            and final_payload.get("gait_phase")
            == ("STANCE" if expected_in_contact else "SWING")
            == replayed_final["gait_phase"],
            "last_event_matches_journal": final_payload.get("last_event")
            == expected_last_event
            and _time_close(final_payload.get("last_hs_event_time_s"), last_hs)
            and _time_close(final_payload.get("last_to_event_time_s"), last_to)
            and final_payload.get("last_event") == replayed_final["last_event"]
            and _time_close(
                final_payload.get("last_hs_event_time_s"),
                replayed_final["last_hs_event_time_s"],
            )
            and _time_close(
                final_payload.get("last_to_event_time_s"),
                replayed_final["last_to_event_time_s"],
            ),
            "final_batch_transients_exact": all(
                final_payload.get(name) == replayed_final[name]
                for name in (
                    "events_this_step",
                    "contact_state_transitions_this_step",
                    "candidate_cancellations_this_step",
                )
            ),
        }
        passed = all(checks.values())
        return {
            "schema_version": SCHEMA_VERSION,
            "status": (
                V20_FINAL_AUDIT_PASS_STATUS
                if passed
                else V20_FINAL_AUDIT_FAIL_STATUS
            ),
            "passed": passed,
            "checks": checks,
            "metrics": {
                "samples_processed": int(samples_processed),
                "last_sample_time_s": final_payload.get("last_sample_time_s"),
                "event_count": dict(event_count),
                "shadow_event_count": len(events),
                "cycle_count": int(cycle_count),
                "reconstructed_cycle_count": reconstructed_cycles,
                "candidate_cancellation_count": int(cancellations),
                "pending_event": copy.deepcopy(final_payload.get("pending_event")),
            },
        }
    except _ContactContractError as exc:
        return {
            "schema_version": SCHEMA_VERSION,
            "status": V20_FINAL_AUDIT_FAIL_STATUS,
            "passed": False,
            "reason": str(exc),
            "checks": {},
            "metrics": {},
        }


def _v20_final_gate_passes(value: Any) -> bool:
    if not isinstance(value, Mapping):
        return False
    checks = value.get("checks")
    metrics = value.get("metrics")
    event_count = metrics.get("event_count") if isinstance(metrics, Mapping) else None
    return bool(
        value.get("schema_version") == SCHEMA_VERSION
        and value.get("status") == V20_FINAL_AUDIT_PASS_STATUS
        and value.get("passed") is True
        and isinstance(checks, Mapping)
        and set(checks)
        == {
            "source_exact",
            "event_contract_exact",
            "sample_dt_exact",
            "debounce_exact",
            "delivery_bound_exact",
            "all_samples_processed",
            "last_sample_is_episode_end",
            "event_counts_match_journal",
            "complete_event_stream_exact",
            "cycle_count_matches_journal",
            "minimum_cycle_count",
            "no_pending_event",
            "no_pending_contact_transition",
            "zero_candidate_cancellations",
            "final_raw_state_matches_journal",
            "last_event_matches_journal",
            "final_batch_transients_exact",
        }
        and all(value is True for value in checks.values())
        and isinstance(metrics, Mapping)
        and metrics.get("samples_processed") == EXPECTED_RAW_SENSOR_SAMPLES
        and _time_close(
            metrics.get("last_sample_time_s"),
            EPISODE_START_TIME_S + EXPECTED_EPISODE_DURATION_S,
        )
        and isinstance(event_count, Mapping)
        and set(event_count) == {"heel_strike", "toe_off"}
        and all(_nonnegative_int(count) for count in event_count.values())
        and metrics.get("shadow_event_count") == sum(event_count.values())
        and _nonnegative_int(metrics.get("cycle_count"))
        and metrics.get("cycle_count") == metrics.get("reconstructed_cycle_count")
        and metrics.get("candidate_cancellation_count") == 0
        and metrics.get("pending_event") is None
    )


def _primary_observation_tap_gate_passes(value: Any) -> bool:
    return bool(
        isinstance(value, Mapping)
        and set(value)
        == {
            "instrumentation_id",
            "installed_after_reset",
            "reset_call_count",
            "wrapper_call_count",
            "original_call_count",
            "one_original_call_per_wrapper_call",
            "second_primary_evaluations",
            "return_forwarded_unmodified",
            "restored_in_finally",
            "descriptor_identity_restored",
        }
        and value.get("instrumentation_id")
        == "primary_grf_existing_call_observer_v1"
        and value.get("installed_after_reset") is True
        and value.get("reset_call_count") == 0
        and value.get("wrapper_call_count") == EXPECTED_PRIMARY_LOAD_SAMPLES
        and value.get("original_call_count") == EXPECTED_PRIMARY_LOAD_SAMPLES
        and value.get("one_original_call_per_wrapper_call") is True
        and value.get("second_primary_evaluations") == 0
        and value.get("return_forwarded_unmodified") is True
        and value.get("restored_in_finally") is True
        and value.get("descriptor_identity_restored") is True
    )


def diagnostic_gate(summary: Mapping[str, Any]) -> dict[str, Any]:
    """Classify one persisted V7 nominal shadow diagnostic fail-closed."""

    if not isinstance(summary, Mapping):
        summary = {}
    zero_fields = (
        "action_mismatch_count",
        "actor_observation_mismatch_count",
        "actor_mean_mismatch_count",
        "teacher_std_mismatch_count",
        "projected_trace_mismatch_count",
        "time_mismatch_count",
        "step_contract_failure_count",
        "action_clipped_values",
        "timeout_count",
        "safety_stop_count",
        "invalid_event_count",
        "hard_invalid_count",
        "nonfinite_count",
        "so_solver_unaccepted_count",
        "sea_plugin_fallback_count",
        "routing_failure_count",
    )
    classification = summary.get("contact_classification")
    checks = {
        "schema_version": summary.get("schema_version") == SCHEMA_VERSION,
        "ungated_status": summary.get("status") == DIAGNOSTIC_COLLECTED_STATUS,
        "protocol_id": summary.get("protocol_id") == PROTOCOL_ID,
        "collector_id": summary.get("collector_id") == COLLECTOR_ID,
        "case_exact": summary.get("case_id") == CASE_ID
        and summary.get("action_selection") == CASE["action_selection"]
        and summary.get("behavior") == CASE["behavior"]
        and summary.get("episode_start_offset_s") == EPISODE_START_OFFSET_S
        and summary.get("action_seed") is None
        and summary.get("runtime_seed") == RUNTIME_SEED,
        "source_h0_exact": summary.get("source_h0_id") == SOURCE_H0_ID,
        "source_contract_exact": summary.get("source_observation_contract_id")
        == SOURCE_OBSERVATION_CONTRACT_ID
        and summary.get("source_event_contract_id") == SOURCE_EVENT_CONTRACT_ID,
        "primary_load_contract_exact": summary.get("primary_load_contract_id")
        == PRIMARY_LOAD_CONTRACT_ID,
        "primary_load_role_exact": summary.get("primary_load_evidence_role")
        == PRIMARY_LOAD_EVIDENCE_ROLE
        and summary.get("canonical_scientific_oracle")
        == CANONICAL_SCIENTIFIC_ORACLE
        and summary.get("primary_online_grf_used_as_event_source") is False,
        "binary_shadow": summary.get("binary_phase_fsm_mode")
        == "binary_shadow"
        and summary.get("actor_event_source") == "legacy_events",
        "shadow_contract_exact": summary.get("binary_phase_event_contract_id")
        == SHADOW_EVENT_CONTRACT_ID,
        "morphology_zero": _finite_number(summary.get("morphology_weight"))
        and float(summary["morphology_weight"]) == MORPHOLOGY_WEIGHT,
        "steps_exact": summary.get("steps") == EXPECTED_STEPS,
        "control_windows_exact": summary.get("control_window_count")
        == EXPECTED_CONTROL_WINDOWS,
        "raw_samples_exact": summary.get("v25_raw_sensor_sample_count")
        == EXPECTED_RAW_SENSOR_SAMPLES,
        "primary_samples_exact": summary.get("primary_load_sample_count")
        == EXPECTED_PRIMARY_LOAD_SAMPLES,
        "primary_alignment_exact": summary.get("primary_load_alignment_id")
        == PRIMARY_LOAD_ALIGNMENT_ID,
        "episode_time_limit": summary.get("end_reason") == "episode_time_limit",
        "not_terminated": summary.get("terminated") is False,
        "truncated": summary.get("truncated") is True,
        "penetration": _finite_number(summary.get("grf_penetration_max_m"))
        and float(summary["grf_penetration_max_m"]) < PENETRATION_LIMIT_M,
        "layout": summary.get("n_actor") == EXPECTED_ACTOR_FEATURES
        and summary.get("n_observation") == EXPECTED_FULL_FEATURES
        and summary.get("observation_dtype") == EXPECTED_OBSERVATION_DTYPE,
        "macos_arm64_numeric_scope": isinstance(summary.get("platform"), Mapping)
        and summary["platform"].get("system") == EXPECTED_PLATFORM_SYSTEM
        and summary["platform"].get("machine") == EXPECTED_PLATFORM_MACHINE,
        "platform_identity_exact": summary.get("platform")
        == EXPECTED_PLATFORM_IDENTITY,
        "v5_projected_trace_bit_exact": summary.get(
            "v5_projected_trace_bit_exact"
        )
        is True,
        "legacy_phase_invalid_events_exact": summary.get(
            "legacy_phase_invalid_event_count"
        )
        == EXPECTED_LEGACY_PHASE_INVALID_EVENT_COUNT,
        "legacy_phase_cycles_exact": summary.get("phase_valid_cycle_count")
        == EXPECTED_LEGACY_PHASE_VALID_CYCLE_COUNT,
        "all_zero_counters": all(
            _nonnegative_int(summary.get(field)) and summary.get(field) == 0
            for field in zero_fields
        ),
        "classification": _classification_gate_passes(classification),
        "v20_final_state": _v20_final_gate_passes(
            summary.get("v20_final_state_audit")
        ),
        "primary_observation_tap": _primary_observation_tap_gate_passes(
            summary.get("primary_observation_tap")
        ),
        "v6_terminal_preserved": summary.get("v6_terminal_status")
        == "FAIL_H0_V6_V25_TEACHER_REPLAY_DEVELOPMENT"
        and summary.get("v6_terminal_ledger_sha256")
        == EXPECTED_INPUT_SHA256["v6_terminal_ledger"]
        and summary.get("v6_nominal_failure_sha256")
        == EXPECTED_INPUT_SHA256["v6_nominal_failure"],
        "zero_actor_updates": summary.get("actor_updates") == 0,
        "zero_critic_updates": summary.get("critic_updates") == 0,
        "zero_ppo_updates": summary.get("ppo_updates") == 0,
        "protected_closed": summary.get("protected_trials_opened") == [],
        "reserve_closed": summary.get("reserve_trials_opened") == [],
        "no_candidate_implemented": summary.get("candidate_implemented") is False,
        "no_candidate_created": summary.get("candidate_created") is False,
        "no_runtime_promotion": summary.get("runtime_promoted") is False,
    }
    passed = all(checks.values())
    return {
        "schema_version": SCHEMA_VERSION,
        "status": DIAGNOSTIC_PASS_STATUS if passed else DIAGNOSTIC_FAIL_STATUS,
        "passed": passed,
        "protocol_id": PROTOCOL_ID,
        "case_id": summary.get("case_id"),
        "classification": (
            classification.get("classification")
            if isinstance(classification, Mapping)
            else AMBIGUOUS_FAIL
        ),
        "decision": (
            classification.get("decision")
            if isinstance(classification, Mapping)
            else NO_DECISION
        ),
        "checks": checks,
        "authority": copy.deepcopy(AUTHORITY),
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
        "requires_followup_protocol": True,
    }


__all__ = [
    "ACTOR_CORRECTION_DECISION",
    "ACTOR_OR_GEOMETRY_FOLLOWUP_DECISION",
    "AMBIGUOUS_FAIL",
    "AUTHORITY",
    "CASE",
    "CASES",
    "CASE_ID",
    "CASE_IDS",
    "CANONICAL_SCIENTIFIC_ORACLE",
    "CLASSIFICATION_FAIL_STATUS",
    "CLASSIFICATION_PASS_STATUS",
    "COLLECTOR_ID",
    "DIAGNOSTIC_COLLECTED_STATUS",
    "DIAGNOSTIC_FAIL_STATUS",
    "DIAGNOSTIC_PASS_STATUS",
    "EARLY_GEOMETRIC_PROXY_REASON",
    "EPISODE_START_OFFSET_S",
    "EPISODE_START_TIME_S",
    "EXECUTION_CLAIM_PATH",
    "EXECUTION_LEDGER_PATH",
    "EXPECTED_ACTION_DIM",
    "EXPECTED_ACTOR_FEATURES",
    "EXPECTED_CONTROL_WINDOWS",
    "EXPECTED_FULL_FEATURES",
    "EXPECTED_H0_POLICY_STD",
    "EXPECTED_H0_POLICY_STD_ARRAY_SHA256",
    "EXPECTED_EPISODE_DURATION_S",
    "EXPECTED_INPUT_SHA256",
    "EXPECTED_LEGACY_PHASE_INVALID_EVENT_COUNT",
    "EXPECTED_LEGACY_PHASE_VALID_CYCLE_COUNT",
    "EXPECTED_OBSERVATION_DTYPE",
    "EXPECTED_PLATFORM_IDENTITY",
    "EXPECTED_PLATFORM_MACHINE",
    "EXPECTED_PLATFORM_SYSTEM",
    "EXPECTED_POLICY_DT_S",
    "EXPECTED_PRIMARY_LOAD_SAMPLES",
    "EXPECTED_PRIMARY_SAMPLES_PER_STEP",
    "EXPECTED_RAW_SENSOR_SAMPLES",
    "EXPECTED_SAMPLE_DT_S",
    "EXPECTED_SAMPLES_PER_STEP",
    "EXPECTED_STEPS",
    "EXPECTED_V20_DEBOUNCE_S",
    "INPUT_RELATIVE_PATHS",
    "LEGACY_MIN_SWING_DURATION_S",
    "LOCK_PATH",
    "LOCK_STATUS",
    "MAX_TRANSIENT_CONTACT_S",
    "MIN_ACCEPTED_STANCE_S",
    "MINIMUM_VALID_CYCLES",
    "MIN_SWING_CANDIDATE_CONTRACT_ID",
    "MIN_SWING_CANDIDATE_DECISION",
    "MIN_SWING_CANDIDATE_S",
    "NO_DECISION",
    "MORPHOLOGY_WEIGHT",
    "PENETRATION_LIMIT_M",
    "PERSISTENT_LANDING",
    "PREFLIGHT_PASS_STATUS",
    "PREFLIGHT_RECEIPT_PATH",
    "PRIMARY_CROSSING_DEADLINE_S",
    "PRIMARY_FORCE_THRESHOLD_N",
    "PRIMARY_LOAD_CONTRACT_ID",
    "PRIMARY_LOAD_ALIGNMENT_ID",
    "PRIMARY_LOAD_EVIDENCE_ROLE",
    "PRIMARY_SUPPORT_MIN_FROM_PRECEDING_TO_S",
    "PROTOCOL_ID",
    "REVISION",
    "RUNTIME_SEED",
    "RUN_ROOT",
    "SCHEMA_VERSION",
    "SHADOW_EVENT_CONTRACT_ID",
    "SOURCE_H0_ID",
    "SOURCE_EVENT_CONTRACT_ID",
    "SOURCE_OBSERVATION_CONTRACT_ID",
    "SOURCE_RELATIVE_PATHS",
    "SO_POLICY_ID",
    "TIME_TOLERANCE_S",
    "TRANSIENT_TOE_SCUFF",
    "V20_EVENT_SOURCE",
    "V20_FINAL_AUDIT_FAIL_STATUS",
    "V20_FINAL_AUDIT_PASS_STATUS",
    "V6_EARLY_HS_ONSET_TIME_S",
    "V6_OBSERVED_SWING_S",
    "V6_PRECEDING_TO_TIME_S",
    "WORKER_CLAIMS_ROOT",
    "canonical_case",
    "audit_v20_final_state",
    "classify_contact",
    "diagnostic_gate",
    "replay_v20_event_stream",
]
