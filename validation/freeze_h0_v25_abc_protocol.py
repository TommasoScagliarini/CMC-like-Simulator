"""Freeze the declarative, non-executable H0/V25 A/B/C protocol.

The lock produced by this script defines provenance, the 12 protocol units
(18 real rollouts), comparison schema, gates, stop rules, and closed authority.
It cannot execute H0 and cannot authorize a driver, ``binary_active``, H0_sep,
protected data, training, promotion, or the corridor.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import sys
from pathlib import Path
from typing import Any, Mapping, Sequence


REPO_ROOT = Path(__file__).resolve().parents[1]
VALIDATION_ROOT = REPO_ROOT / "validation"
if str(VALIDATION_ROOT) not in sys.path:
    sys.path.insert(0, str(VALIDATION_ROOT))

import freeze_v25_shadow_integration_readiness as shadow  # noqa: E402


SCHEMA_VERSION = 1
PROTOCOL_ID = "AB06_H0_V25_ABC_PROTOCOL_ONLY"
DESTINATION = VALIDATION_ROOT / "h0_v25_abc_protocol_lock.json"
READINESS_RECEIPT = shadow.DESTINATION
READINESS_RECEIPT_SHA256 = (
    "8c7316d19f4fe08bd90cd3476434ed111566c4ba7669c6c763dc163b6df073c2"
)
ADDENDUM_SHA256 = (
    "a8cea18b338b08c32225c3912561cefe953add3b9e4bb693009aba6b189e9835"
)
HISTORICAL_PLAN_SHA256 = (
    "2dd4e0a06e13dca87ce74a1fe2bb601a1822ea4decc78cd60987ec250f47b520"
)
PRIMARY_LOCK_SHA256 = (
    "e9347bb5e5d04e84ce96f5c3ed354e154d26d0bc5abb750dc68bb8b79d0c06ac"
)
V25_LOCK_SHA256 = shadow.V25_LOCK_SHA256
ACTIVE_RUNTIME_CONTRACT_ID = "binary_point_v25+functional_contact_fsm_v1"
CLOSED_AUTHORITY = {
    "execution_authorized": False,
    "h0_executed": False,
    "training_authorized": False,
    "ppo_updates_authorized": False,
    "h0_sep_authorized": False,
    "protected_trial_access_authorized": False,
    "reserve_trial_access_authorized": False,
    "corridor_authorized": False,
    "runtime_promotion_authorized": False,
    "primary_grf_modification_authorized": False,
    "detector_retuning_authorized": False,
}

H0_ROOT = (
    VALIDATION_ROOT
    / "critic_warmup"
    / "2026-07-13_markov35_phase_aligned_sigma0005_iter1_retry"
)
H0_CHECKPOINT = H0_ROOT / "checkpoint_last"
H0_RL_MODULE = H0_ROOT / "rl_module_last"

EXPECTED_CHECKPOINT_TREE = {
    "sha256": "828b133c7511c018be54e1ea77c9e45a6dc33762d6f3e36b2fc1698191eac243",
    "file_count": 24,
    "size_bytes": 1898007,
}
EXPECTED_RL_MODULE_TREE = {
    "sha256": "f4ed0a17e677d89ce42f2f276352075f3302b7141fda27aa15b2624a1f2c0bb5",
    "file_count": 3,
    "size_bytes": 607231,
}

PINNED: dict[str, tuple[Path, str]] = {
    "normative_addendum": (
        REPO_ROOT / "reports/plans/2026-08-04_addendum_v25_protocollo_h0_abc.md",
        ADDENDUM_SHA256,
    ),
    "historical_plan": (
        REPO_ROOT
        / "reports/plans/2026-07-23_piano_grf_primaria_h0_sep_detector_training_ready.md",
        HISTORICAL_PLAN_SHA256,
    ),
    "shadow_readiness_receipt": (READINESS_RECEIPT, READINESS_RECEIPT_SHA256),
    "shadow_readiness_script": (
        VALIDATION_ROOT / "freeze_v25_shadow_integration_readiness.py",
        "3a1d04c6db6254ecd2e53fb58e5a42b61220c0d6cd486014e8c5fe2b0d15df50",
    ),
    "shadow_readiness_test": (
        VALIDATION_ROOT / "test_freeze_v25_shadow_integration_readiness.py",
        "6dc0360dced657ad429187a0002d2f9f383db475cf345fddfd29e3cb40e3dd75",
    ),
    "primary_lock": (
        VALIDATION_ROOT / "primary_grf_core_lock_2026-08-03.json",
        PRIMARY_LOCK_SHA256,
    ),
    "v25_candidate_lock": (
        VALIDATION_ROOT
        / "binary_phase_detector_v25_development_candidate_freeze_lock.json",
        V25_LOCK_SHA256,
    ),
    "v25_profile": (
        shadow.V25_RUN / "selected_candidate_profile.json",
        shadow.V25_PROFILE_SHA256,
    ),
    "legacy_analog_profile": (
        REPO_ROOT
        / "online_grf_profiles"
        / "AB06_SEASEA_stiff321_500_pi_grf_detector_HS-TO.json",
        shadow.LEGACY_ANALOG_PROFILE_SHA256,
    ),
    "h0_module_state": (
        H0_RL_MODULE / "module_state.pkl",
        "44457ca5df7fa0e0e1f1d361d940136917fe8f71e984a1b0afaefb8ca3ced33b",
    ),
    "h0_module_constructor": (
        H0_RL_MODULE / "class_and_ctor_args.pkl",
        "5c98f006d99a71a0f1ddcbb31d8d73fe0a6dade8401e679f6af5b1bc943b4228",
    ),
    "h0_module_metadata": (
        H0_RL_MODULE / "metadata.json",
        "3a032ba54abcee8c9bcbb39e72fa05566912e94461d01f3c6228dc60e088bf12",
    ),
    "h0_resolved_config": (
        H0_ROOT / "training_cfg.resolved.yaml",
        "6904f7a9000b63b5c1aab661ebcab4974dffdd1cfb8c731df6a953fc9234229e",
    ),
    "h0_checkpoint_metadata": (
        H0_ROOT / "checkpoint_last_meta.json",
        "adc5e26c84f3a0fb2de88eacea945426d38ea4e80c7f2f0cd1979ea663c9dfff",
    ),
    "h0_summary": (
        H0_ROOT / "summary.json",
        "f9840662947b0e4e301951f28500dcd51b3daee7aa98828f0f635ab750c738fc",
    ),
    "h0_actor_transplant": (
        H0_ROOT / "actor_transplant_report.json",
        "6ffc0a4cee96438a529ab398f4730d95cde7a6247e786914ac36ef8ccdd2e4e7",
    ),
    "h0_selection_gate": (
        VALIDATION_ROOT
        / "controller_memory_ablation/2026-07-13_markov35_final_gate.json",
        "1f5125617a8d7705f02e28b4d6ac7e09922551a7695ce7f4a1665e5c2641d4b8",
    ),
    "h0_compare_evidence": (
        VALIDATION_ROOT
        / "checkpoint_comparisons/2026-07-14_h0_vs_target_domain_source.json",
        "01cd95237ad5b4c44c2802edeebeba0f32a594b3ccd098872169fbf3756efb0b",
    ),
    "h0_layout_evidence": (
        VALIDATION_ROOT
        / "controller_memory_ablation"
        / "2026-07-13_markov35_corrected_full_sigma0005_seed123"
        / "rollout_summary.json",
        "70676e7d2b4e8c1bca185fe23d1d29f96fa83ac34167ba916250eff6e96f601e",
    ),
    "warm_start_digest_source": (
        REPO_ROOT / "Trajectory Generator/baseline_MLP/warm_start.py",
        "84706218dcc4c5cb7f97a8f3f67ef40ba9e064ba5aef25cb6559d1c8a506c34c",
    ),
    "compare_digest_source": (
        VALIDATION_ROOT / "compare_policy_checkpoints.py",
        "e21a8db205eb2aa4b325601598e0c4b5b92f68d9a9655a09a1b27ce5d2dea5d7",
    ),
    "setup_xml": (
        REPO_ROOT
        / "models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi_setup.xml",
        "5fa4748e537ae1a0fbda091a5ed2b5774ac445a4904b3ad0bb02f259bf0a4931",
    ),
    "runtime_model": (
        REPO_ROOT
        / "models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi.osim",
        "33e67d84bf11740eac509f620a143ad3c57d98c6f765d857e69c1892513de0c1",
    ),
    "ik_input": (
        REPO_ROOT
        / "models/AB06_SEASEA_Threadmill/data/IK_results_AB06_SEASEA.mot",
        "ce4a948fd8f01f34ff32e4680b8a082f2e44de155aed89b7e4a55d37016c3596",
    ),
    "external_forces": (
        REPO_ROOT / "models/AB06_SEASEA_Threadmill/data/ExternalForces.xml",
        "a2fb6460fc0f8ae9294b4504eb9bb18f0dcc4b112ec71403448c28f4b5dc3f27",
    ),
    "prescribed_grf": (
        REPO_ROOT
        / "models/AB06_SEASEA_Threadmill/data/AB06_SEASEA_GRF_FullSpan.mot",
        "6b84cce4abd6b313d102c6fa6123eb871e12154976fbf1e3c380f34928dc4059",
    ),
    "actuator_set": (
        REPO_ROOT / "models/AB06_SEASEA_Threadmill/data/CMC_Actuators.xml",
        "238792c0024c405241e4a45b9be88e153660561a02e667aedbd1dd6116df73dd",
    ),
    "primary_profile": (
        REPO_ROOT
        / "online_grf_profiles/AB06_SEASEA_stiff321_500_pi_grf_correct_magnitude.json",
        "09e04ab94954703d74acc3a80b24ecefcc07d3fc918c03b9e9df8116a6c1a2b0",
    ),
    "sea_plugin_macos": (
        REPO_ROOT / "plugins/libSEA_Plugin_BlackBox_mCMC_impedence_ff.dylib",
        "77390d0f74055fb3419e88637baac1d215b1dd402ee1effe3e8cb14a66caf54b",
    ),
    "online_grf_plugin_macos": (
        REPO_ROOT / "plugins/libOnlineGRFContact.dylib",
        "5597a59a5368825fd207753b59291e72240c1ece3aa5280804f1e9b9c7d6a2b3",
    ),
    "morphology_profile_weight_zero_only": (
        REPO_ROOT
        / "Trajectory Generator/baseline_MLP/morphology_profiles/"
        "ab06_prosthetic_event_warped_mean_std_corridor.json",
        "33b1dd7cb0db40110a4f9c1b8c0dd49a498662211e6e132f0f3cefe8edc02a55",
    ),
    "historical_v24_preflight_test": (
        VALIDATION_ROOT / "test_binary_phase_detector_v24_clearance_gap.py",
        "0ca8d85476e8f9a9b690af9dcc48ce54158f3b29c912cb22f0f8ae6848d6bb24",
    ),
    "historical_v25_preflight_test": (
        VALIDATION_ROOT / "test_sweep_binary_phase_detector_v25_geometry.py",
        "df16ba3e09de5bfb53b6744a01886bfec8e5937b55d6fc3fc1e87a0e6f770d98",
    ),
}


class H0V25ProtocolFreezeError(RuntimeError):
    """Raised when the declarative protocol cannot be frozen exactly."""


def _verify_pins() -> tuple[dict[str, dict[str, Any]], dict[str, dict[str, Any]]]:
    records: dict[str, dict[str, Any]] = {}
    payloads: dict[str, dict[str, Any]] = {}
    for label, (path, expected_sha256) in PINNED.items():
        record = shadow.source_record(path)
        if record["sha256"] != expected_sha256:
            raise H0V25ProtocolFreezeError(f"pinned artifact drifted: {label}")
        records[label] = record
        if path.suffix == ".json":
            try:
                payloads[label] = shadow.strict_json_load(path, expected_sha256)
            except shadow.V25ShadowReadinessError as exc:
                raise H0V25ProtocolFreezeError(str(exc)) from exc
    return records, payloads


def tree_record(root: Path) -> dict[str, Any]:
    resolved = root.resolve()
    if not resolved.is_dir():
        raise H0V25ProtocolFreezeError(f"tree root is missing: {root}")
    files = sorted(path for path in resolved.rglob("*") if path.is_file())
    entries = [
        {
            "path": path.relative_to(resolved).as_posix(),
            "sha256": shadow.sha256_file(path),
            "size_bytes": int(path.stat().st_size),
        }
        for path in files
    ]
    canonical = json.dumps(
        entries,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=True,
        allow_nan=False,
    ).encode("utf-8")
    return {
        "path": resolved.relative_to(REPO_ROOT.resolve()).as_posix(),
        "tree_hash_algorithm": (
            "sha256(canonical_sorted_file_records_json;sort_keys;compact;ascii)"
        ),
        "sha256": hashlib.sha256(canonical).hexdigest(),
        "file_count": len(entries),
        "size_bytes": sum(item["size_bytes"] for item in entries),
        "files": entries,
    }


def _verify_readiness_receipt() -> dict[str, Any]:
    expected_payload = shadow.build_receipt_payload(
        require_destination_unoccupied=False
    )
    expected_record = shadow.payload_record(READINESS_RECEIPT, expected_payload)
    if expected_record["sha256"] != READINESS_RECEIPT_SHA256:
        raise H0V25ProtocolFreezeError("predicted readiness receipt hash drifted")
    if not READINESS_RECEIPT.is_file():
        raise H0V25ProtocolFreezeError("readiness receipt is not published")
    observed = shadow.source_record(READINESS_RECEIPT)
    if observed != expected_record:
        raise H0V25ProtocolFreezeError("published readiness receipt is not exact")
    payload = shadow.strict_json_load(READINESS_RECEIPT, READINESS_RECEIPT_SHA256)
    if payload != expected_payload:
        raise H0V25ProtocolFreezeError("readiness receipt payload drifted")
    if payload.get("status") != "V25_SHADOW_STRUCTURALLY_READY_NUMERICAL_AB_UNRUN":
        raise H0V25ProtocolFreezeError("readiness receipt status is not structural")
    if payload.get("authority", {}).get("execution_authorized") is not False:
        raise H0V25ProtocolFreezeError("readiness receipt opened execution")
    return observed


def conditions() -> list[dict[str, Any]]:
    return [
        {
            "condition_id": "det_minus020",
            "action_selection": "deterministic",
            "start_offset_s": 1.756870983805102,
            "seed": 123,
            "expected_sigma": None,
            "standard_normal_innovation_tape": False,
        },
        {
            "condition_id": "det_nominal",
            "action_selection": "deterministic",
            "start_offset_s": 1.956870983805102,
            "seed": 123,
            "expected_sigma": None,
            "standard_normal_innovation_tape": False,
        },
        {
            "condition_id": "det_plus020",
            "action_selection": "deterministic",
            "start_offset_s": 2.156870983805102,
            "seed": 123,
            "expected_sigma": None,
            "standard_normal_innovation_tape": False,
        },
        *[
            {
                "condition_id": f"stoch_nominal_seed{seed}",
                "action_selection": "stochastic",
                "start_offset_s": 1.956870983805102,
                "seed": seed,
                "expected_sigma": 0.005,
                "standard_normal_innovation_tape": True,
            }
            for seed in (123, 124, 125)
        ],
    ]


def cases() -> dict[str, dict[str, Any]]:
    common = {
        "legacy_analog_loaded": True,
        "legacy_analog_sensor_only": True,
        "legacy_analog_roles": [
            "left_heel",
            "left_toe",
            "right_heel",
            "right_toe",
        ],
        "v25_loaded_and_sampled": True,
        "detector_sample_dt_s": 0.001,
        "binary_phase_debounce_s": 0.005,
        "binary_phase_event_contract_id": shadow.RUNTIME_V25_CONTRACT_ID,
        "policy_step_s": 0.01,
        "morphology_weight": 0.0,
        "actor_observation_count": 35,
        "full_observation_count": 84,
    }
    return {
        "A": {
            **common,
            "case_name": "legacy_control",
            "phase_fsm_input_mode": "legacy_events",
            "event_contract_id": "legacy_events_v1",
            "binary_phase_fsm_mode": "disabled",
            "authoritative_left_event_source": "legacy_analog",
            "raw_v25_journal_required": True,
        },
        "B": {
            **common,
            "case_name": "binary_shadow",
            "phase_fsm_input_mode": "legacy_events",
            "event_contract_id": "legacy_events_v1",
            "binary_phase_fsm_mode": "binary_shadow",
            "scientific_bundle_contract_id": shadow.SCIENTIFIC_SHADOW_BUNDLE_ID,
            "authoritative_left_event_source": "legacy_analog",
            "raw_v25_journal_required": True,
        },
        "C": {
            **common,
            "case_name": "binary_active",
            "binary_phase_fsm_mode": "binary_active_FUTURE_UNAVAILABLE",
            "binary_phase_event_contract_id": ACTIVE_RUNTIME_CONTRACT_ID,
            "target_bundle_contract_id": shadow.TARGET_ACTIVE_BUNDLE_ID,
            "authoritative_left_event_source": "v25_fsm_v20_only",
            "legacy_analog_left_diagnostic_only": True,
            "left_fallback_allowed": False,
            "v20_to_prosthetic_phase_fsm_adapter_required": True,
            "direct_phase_fsm_payload_replacement_allowed": False,
            "v20_events_adapted_before_same_policy_step_observation": True,
            "phase_state_aggregator": "ProstheticPhaseFSM",
            "phase_event_order_after_optional_startup_edge": "HS_TO_HS",
        },
    }


def protocol_matrix() -> dict[str, Any]:
    condition_rows = conditions()
    rollout_rows: list[dict[str, Any]] = []
    units: list[dict[str, Any]] = []
    for condition in condition_rows:
        condition_id = condition["condition_id"]
        a_id = f"A__{condition_id}"
        b_id = f"B__{condition_id}"
        rollout_rows.extend(
            {
                "rollout_id": rollout_id,
                "case_id": case_id,
                "condition_id": condition_id,
                "destination": "UNALLOCATED_EXECUTION_NOT_AUTHORIZED",
            }
            for rollout_id, case_id in ((a_id, "A"), (b_id, "B"))
        )
        units.append(
            {
                "unit_id": f"paired_ab__{condition_id}",
                "unit_type": "indivisible_paired_ab",
                "condition_id": condition_id,
                "rollout_ids": [a_id, b_id],
                "pass_requirement": "A_COMMON_GATE_AND_B_COMMON_GATE_AND_BIT_EXACT",
            }
        )
    for condition in condition_rows:
        condition_id = condition["condition_id"]
        c_id = f"C__{condition_id}"
        rollout_rows.append(
            {
                "rollout_id": c_id,
                "case_id": "C",
                "condition_id": condition_id,
                "destination": "UNALLOCATED_EXECUTION_NOT_AUTHORIZED",
            }
        )
        units.append(
            {
                "unit_id": f"active_c__{condition_id}",
                "unit_type": "single_active_c",
                "condition_id": condition_id,
                "rollout_ids": [c_id],
                "prerequisite": "ALL_SIX_PAIRED_AB_UNITS_PASS",
                "reference_rollout_id": f"A__{condition_id}",
            }
        )
    return {
        "conditions": condition_rows,
        "cases": cases(),
        "units": units,
        "rollouts": rollout_rows,
        "condition_count": len(condition_rows),
        "paired_ab_unit_count": 6,
        "c_unit_count": 6,
        "protocol_unit_count": len(units),
        "underlying_rollout_count": len(rollout_rows),
        "execution_order": [
            "ALL_SIX_PAIRED_AB_UNITS",
            "VERIFY_ALL_SIX_PAIRED_AB_PASS",
            "ONLY_THEN_SIX_C_UNITS",
        ],
    }


def sea_metric_schema() -> list[dict[str, Any]]:
    families = (
        ("torque_error_nm", "N m"),
        ("tau_spring_nm", "N m"),
        ("tau_spring_rate_nm_s", "N m/s"),
        ("motor_speed_rad_s", "rad/s"),
        ("motor_accel_rad_s2", "rad/s^2"),
        ("motor_power_w", "W"),
    )
    rows = [
        {
            "joint": joint,
            "signal": signal,
            "unit": unit,
            "aggregations": ["episode_rms", "absolute_maximum"],
            "cap": "C <= A + max(1e-6, 1e-9 * abs(A))",
        }
        for joint in ("knee", "ankle")
        for signal, unit in families
    ]
    rows.extend(
        {
            "joint": joint,
            "signal": "tau_input_saturated",
            "unit": "count,1",
            "aggregations": ["count", "fraction_same_finite_denominator"],
            "cap": "C <= A_WITHOUT_TOLERANCE",
        }
        for joint in ("knee", "ankle")
    )
    return rows


def gate_schema() -> dict[str, Any]:
    return {
        "common_per_rollout": {
            "steps_exact": 500,
            "duration_s": 5.0,
            "end_reason_exact": "episode_time_limit",
            "minimum_complete_valid_cycles": 2,
            "max_penetration_m_strictly_less_than": 0.025,
            "per_step_metric_sample_count_exact": 500,
            "actor_full_layout_exact": [35, 84],
            "zero_required": [
                "action_clipping",
                "timeout",
                "safety_stop",
                "fallback",
                "nonfinite",
                "hard_invalid_runtime_or_contract",
            ],
            "legacy_invalid_event_count": (
                "A_AND_B_PRESENT_FINITE_BIT_EXACT_NOT_REQUIRED_ZERO"
            ),
            "artifacts": "strict_json_atomic_no_clobber_no_nan_or_inf",
            "missing_required_field": "FAIL",
        },
        "paired_ab": {
            "aligned_rows_exact": 500,
            "comparison": "BIT_EXACT_ZERO_TOLERANCE_NO_TRUNCATION",
            "top_level_diff_allowlist": {
                "rule": "key_name_startswith_exact",
                "prefix": "binary_phase_",
                "other_exclusions": [],
            },
            "raw_v25_input_journal": {
                "t0_baseline_count": 1,
                "one_ms_sample_count": 5000,
                "fields": [
                    "time_s",
                    "left_heel_contact",
                    "left_toe_contact",
                ],
                "comparison": "BIT_EXACT_INCLUDING_ORDER_TIMESTAMPS_AND_TRACE_SHA",
                "fsm_state_transitions_events_excluded": True,
            },
            "compared_active_payload": [
                "actor_and_critic_observations",
                "policy_mean_raw_and_applied_actions",
                "standard_normal_innovations",
                "served_references",
                "states_and_dynamics",
                "legacy_events_pulses_gait_state",
                "reward_terms_and_total",
                "sea_reserve_residual_diagnostics",
                "termination_and_common_summary_fields",
            ],
        },
        "action_tape": {
            "A_writes_no_clobber": True,
            "simple_reseed_for_B_forbidden": True,
            "stochastic_tape": "STANDARD_NORMAL_PRE_SCALING_PER_STEP_PER_ACTION",
            "expected_sigma": 0.005,
            "B_must_verify_mean_and_raw_before_any_A_action_injection": True,
            "B_replay_mode": "MUST_BE_CHOSEN_IN_FUTURE_EXECUTION_LOCK",
            "C_reuses_condition_matched_innovations": True,
            "C_is_closed_loop_and_never_replays_A_actions": True,
        },
        "c_events": {
            "samples_per_policy_step": 10,
            "t0_event_count": 0,
            "debounce_confirmation_s": 0.005,
            "delivery_after_confirmation_s": [0.0, 0.01],
            "valid_words": {
                "AIR": [0, 0],
                "HEEL": [1, 0],
                "BOTH": [1, 1],
                "TOE": [0, 1],
            },
            "candidate_cancellation_before_debounce_is_invalid": False,
            "startup_partial_stance": {
                "t0_contact_emits_hs": False,
                "leading_to_allowed": True,
                "leading_to_completes_cycle": False,
                "prosthetic_phase_fsm_bootstrap_required": True,
                "air_and_contact_reset_tests_required": True,
            },
            "event_order_after_optional_startup_edge": "HS_TO_HS",
            "left_active_source_exact": "v25_fsm_v20",
            "left_fallback_allowed": False,
            "binary_hard_invalid_unknown_unaccepted_count": 0,
        },
        "c_condition_matched_nonregression": {
            "reference": "A_SAME_CONDITION_ONLY",
            "reserve_and_residual_aggregations": ["absolute_maximum", "episode_rms"],
            "reserve_and_residual_cap": "C <= A + max(1e-6 N m, 1e-9 * abs(A))",
            "fallback_saturation_violation_counts": "C <= A_NO_TOLERANCE",
            "sea_metrics": sea_metric_schema(),
        },
    }


def build_protocol_payload(*, require_destination_unoccupied: bool = True) -> dict[str, Any]:
    if require_destination_unoccupied and os.path.lexists(DESTINATION):
        raise H0V25ProtocolFreezeError(f"refusing to clobber: {DESTINATION}")
    records, payloads = _verify_pins()
    readiness_record = _verify_readiness_receipt()
    checkpoint_tree = tree_record(H0_CHECKPOINT)
    module_tree = tree_record(H0_RL_MODULE)
    matrix = protocol_matrix()

    final_gate = payloads["h0_selection_gate"]
    transplant = payloads["h0_actor_transplant"]
    comparison = payloads["h0_compare_evidence"]
    layout = payloads["h0_layout_evidence"]
    summary = payloads["h0_summary"]
    checkpoint_meta = payloads["h0_checkpoint_metadata"]
    primary_lock = payloads["primary_lock"]
    v25_lock = payloads["v25_candidate_lock"]
    readiness = payloads["shadow_readiness_receipt"]
    h0_config_text = PINNED["h0_resolved_config"][0].read_text(encoding="utf-8")

    assertions = {
        "all_pins_exact": len(records) == len(PINNED),
        "readiness_receipt_exact_and_structural_only": readiness_record
        == records["shadow_readiness_receipt"]
        and readiness.get("numerical_ab_pass_claimed") is False
        and readiness.get("h0_compatibility_claimed") is False,
        "normative_addendum_protocol_only": (
            "PROTOCOL_ONLY_EXECUTION_NOT_AUTHORIZED"
            in PINNED["normative_addendum"][0].read_text(encoding="utf-8")
        ),
        "primary_contract_frozen": primary_lock.get("status")
        == "PRIMARY_CONTRACT_FROZEN_LIMITED_HYBRID_CLAIM"
        and primary_lock.get("scientific_core", {}).get("runtime_model", {}).get(
            "sha256"
        )
        == records["runtime_model"]["sha256"],
        "v25_candidate_frozen_not_promoted": v25_lock.get("candidate", {}).get(
            "candidate_id"
        )
        == "v25_4b351f67b5b86ab0"
        and v25_lock.get("lifecycle", {}).get("runtime_promoted") is False,
        "checkpoint_tree_exact": all(
            checkpoint_tree[key] == expected
            for key, expected in EXPECTED_CHECKPOINT_TREE.items()
        ),
        "rl_module_tree_exact": all(
            module_tree[key] == expected
            for key, expected in EXPECTED_RL_MODULE_TREE.items()
        ),
        "rl_module_bundle_files_exact": set(
            item["path"] for item in module_tree["files"]
        )
        == {"module_state.pkl", "class_and_ctor_args.pkl", "metadata.json"},
        "h0_evidence_exact": final_gate.get("decision")
        == "SELECT_MARKOV35_FOR_H0_H1_WITH_WARM_CRITIC"
        and final_gate.get("selected_actor", {}).get("actor_feature_count") == 35
        and final_gate.get("selected_actor", {}).get("prescribed_actor_features")
        == 0
        and final_gate.get("critic_warmup", {}).get("critic_digest_after_restore")
        == "4584399f63781c48002f8169240ae686b0317aaa22bab9799d553d7fbc2a9f6e",
        "h0_layout_exact_35_84": layout.get("n_actor") == 35
        and layout.get("n_observation") == 84
        and len(transplant.get("target_actor_feature_names", [])) == 35,
        "historical_digest_algorithms_distinct": transplant.get(
            "target_actor_digest_after"
        )
        == "a0801a9e635db4f2973da7d8f6461cbbf7b1643efef1dedc2baafd9c9f95ca21"
        and comparison.get("reference_actor_digest")
        == "585e33257a0b5fe5b419f8e8f3a5f793c9d2ea71ecd07b8668b77d599bcb0287"
        and comparison.get("parameter_comparison", {}).get("exact") is True,
        "h0_config_frozen_actor_sigma_source": all(
            token in h0_config_text
            for token in (
                "asymmetric_actor_critic: true",
                "freeze_actor: true",
                "seed: 123",
                "episode_start_offset_s: 1.956870983805102",
                "morphology_weight: 0.0",
            )
        )
        and len(summary.get("actor_freeze_audit", [])) == 2
        and checkpoint_meta.get("logical_iteration") == 1,
        "matrix_exact_12_units_18_rollouts": matrix["condition_count"] == 6
        and matrix["paired_ab_unit_count"] == 6
        and matrix["c_unit_count"] == 6
        and matrix["protocol_unit_count"] == 12
        and matrix["underlying_rollout_count"] == 18,
        "active_and_driver_unavailable": "binary_active_FUTURE_UNAVAILABLE"
        == matrix["cases"]["C"]["binary_phase_fsm_mode"]
        and matrix["cases"]["A"]["binary_phase_event_contract_id"]
        == shadow.RUNTIME_V25_CONTRACT_ID
        and matrix["cases"]["B"]["binary_phase_event_contract_id"]
        == shadow.RUNTIME_V25_CONTRACT_ID
        and matrix["cases"]["C"]["binary_phase_event_contract_id"]
        == ACTIVE_RUNTIME_CONTRACT_ID,
        "all_authority_remains_closed": set(CLOSED_AUTHORITY)
        == {
            "execution_authorized",
            "h0_executed",
            "training_authorized",
            "ppo_updates_authorized",
            "h0_sep_authorized",
            "protected_trial_access_authorized",
            "reserve_trial_access_authorized",
            "corridor_authorized",
            "runtime_promotion_authorized",
            "primary_grf_modification_authorized",
            "detector_retuning_authorized",
        }
        and all(value is False for value in CLOSED_AUTHORITY.values()),
    }
    if not all(assertions.values()):
        raise H0V25ProtocolFreezeError(f"protocol preflight failed: {assertions}")

    payload = {
        "schema_version": SCHEMA_VERSION,
        "protocol_id": PROTOCOL_ID,
        "date": "2026-08-04",
        "status": "H0_V25_ABC_PROTOCOL_FROZEN_EXECUTION_NOT_AUTHORIZED",
        "protocol_defined": True,
        "protocol_executed": False,
        "scientific_result": "UNAVAILABLE_PROTOCOL_NOT_EXECUTED",
        "provenance": {
            "normative_addendum": records["normative_addendum"],
            "historical_plan": records["historical_plan"],
            "shadow_readiness_receipt": records["shadow_readiness_receipt"],
            "primary_lock": records["primary_lock"],
            "v25_candidate_lock": records["v25_candidate_lock"],
        },
        "contracts": {
            "raw_detector_contract_id": "binary_point_clearance_v1",
            "runtime_binary_phase_event_contract_id": shadow.RUNTIME_V25_CONTRACT_ID,
            "scientific_shadow_bundle_contract_id": shadow.SCIENTIFIC_SHADOW_BUNDLE_ID,
            "target_active_bundle_contract_id": shadow.TARGET_ACTIVE_BUNDLE_ID,
            "runtime_active_component_contract_id": ACTIVE_RUNTIME_CONTRACT_ID,
            "historical_fsm_contract_id": shadow.HISTORICAL_FSM_CONTRACT_ID,
            "fsm_geometry_agnostic_bridge": True,
            "historical_lock_reinterpreted_or_rewritten": False,
        },
        "h0_identity": {
            "checkpoint_tree": checkpoint_tree,
            "rl_module_tree": module_tree,
            "module_files": {
                key: records[key]
                for key in (
                    "h0_module_state",
                    "h0_module_constructor",
                    "h0_module_metadata",
                    "h0_resolved_config",
                    "h0_checkpoint_metadata",
                )
            },
            "digests": {
                "raw_module_state_file_sha256": records["h0_module_state"]["sha256"],
                "warm_start_actor_state_digest": {
                    "value": "a0801a9e635db4f2973da7d8f6461cbbf7b1643efef1dedc2baafd9c9f95ca21",
                    "algorithm_source": records["warm_start_digest_source"],
                },
                "compare_policy_actor_digest": {
                    "value": "585e33257a0b5fe5b419f8e8f3a5f793c9d2ea71ecd07b8668b77d599bcb0287",
                    "algorithm_source": records["compare_digest_source"],
                },
                "critic_state_digest": (
                    "4584399f63781c48002f8169240ae686b0317aaa22bab9799d553d7fbc2a9f6e"
                ),
                "historical_actor_digest_values_expected_to_differ_by_algorithm": True,
            },
            "evidence": {
                key: records[key]
                for key in (
                    "h0_summary",
                    "h0_actor_transplant",
                    "h0_selection_gate",
                    "h0_compare_evidence",
                    "h0_layout_evidence",
                )
            },
        },
        "runtime_inputs": {
            key: records[key]
            for key in (
                "setup_xml",
                "runtime_model",
                "ik_input",
                "external_forces",
                "prescribed_grf",
                "actuator_set",
                "primary_profile",
                "legacy_analog_profile",
                "v25_profile",
                "sea_plugin_macos",
                "online_grf_plugin_macos",
                "morphology_profile_weight_zero_only",
            )
        },
        "platform_scope": {
            "numerical_claim": "macOS_arm64_only_after_future_execution",
            "windows_x86_64": "PENDING_DLL_HASH_AND_PARITY",
        },
        "matrix": matrix,
        "gates": gate_schema(),
        "stop_and_terminal_semantics": {
            "first_failure_stops_without_retry_rescue_or_retuning": True,
            "A_failure": "ERROR_H0_REFERENCE",
            "AB_mismatch": "ERROR_SHADOW_NONINTERFERENCE",
            "C_scientific_failure_after_all_AB_pass": "FAIL_H0_V25_COMPATIBILITY",
            "all_C_pass_after_all_AB_pass": "PASS_H0_V25_COMPATIBLE",
            "complete_pass_claim": "H0_BINARY_V25_CANDIDATE_READY",
            "C_failure_only_allows_future_h0_sep_freeze_proposal": True,
            "C_pass_forbids_h0_sep_as_unnecessary": True,
        },
        "unmet_execution_prerequisites": {
            "binary_active_implemented_and_tested": False,
            "v20_to_prosthetic_phase_fsm_adapter_implemented_and_tested": False,
            "partial_stance_bootstrap_tests_present": False,
            "A_raw_v25_journal_capture_implemented_and_tested": False,
            "abc_driver_and_bit_exact_comparator_frozen": False,
            "B_replay_mode_selected_and_frozen": False,
            "sea_trace_field_mapping_and_derivation_frozen": False,
            "eighteen_empty_output_destinations_frozen": False,
            "separate_execution_unlock_receipt_present": False,
        },
        "data_governance": {
            "development_trials_consumed": ["02", "04", "08"],
            "protected_trials_opened": [],
            "protected_trials_closed": ["05", "06"],
            "reserve_trials_opened": [],
            "reserve_trials_closed": ["03", "07"],
            "historical_trial_01_reopened": False,
        },
        "historical_preflight_refusals_expected": {
            "v24_or_v25_live_preflight_rerunnable": False,
            "reason": (
                "SANCTIONED_RUNTIME_INTEGRATION_CHANGED_PINNED_MODEL_LOADER_AFTER_"
                "HISTORICAL_DEVELOPMENT_FREEZE"
            ),
            "historical_evidence_and_locks_immutable": True,
            "old_preflight_refusal_is_new_readiness_failure": False,
            "historical_script_or_receipt_rewrite_allowed": False,
            "historical_model_loader_sha256": (
                "fba3f025a83082bb07276770b21f644e3c84750402d97c6305c7ea0eef8ccd76"
            ),
            "current_model_loader_sha256": (
                "401beddc52e2dd8ce4a88208cf5b38b036232cb1bc3ea37704e467852f2ace12"
            ),
            "expected_refusal_tests": [
                {
                    "source": records["historical_v24_preflight_test"],
                    "test_id": (
                        "validation.test_binary_phase_detector_v24_clearance_gap."
                        "V24ClearanceGapTests."
                        "test_preflight_is_read_only_and_declares_model_compatibility"
                    ),
                },
                {
                    "source": records["historical_v25_preflight_test"],
                    "test_id": (
                        "validation.test_sweep_binary_phase_detector_v25_geometry."
                        "V25GeometrySweepTests."
                        "test_preflight_is_platform_neutral_read_only_and_never_imports_opensim"
                    ),
                },
            ],
            "broad_suite_observed": {
                "tests_run": 105,
                "passes": 103,
                "expected_historical_preflight_refusals": 2,
                "unexpected_failures": 0,
            },
        },
        "authority": dict(CLOSED_AUTHORITY),
        "assertions": assertions,
        "freeze_script": shadow.source_record(Path(__file__)),
        "next_stage": "IMPLEMENT_AND_FREEZE_SEPARATE_EXECUTION_UNLOCK_OR_STOP",
    }
    shadow.encode_json(payload)
    return payload


def preflight_unfrozen() -> dict[str, Any]:
    payload = build_protocol_payload(require_destination_unoccupied=True)
    return {
        "status": "H0_V25_ABC_PROTOCOL_LOCK_READY_UNWRITTEN",
        "destination_unoccupied": True,
        "execution_authorized": False,
        "lock_record_if_frozen": shadow.payload_record(DESTINATION, payload),
        "protocol_payload": payload,
    }


def freeze_protocol() -> dict[str, Any]:
    payload = build_protocol_payload(require_destination_unoccupied=True)
    try:
        shadow.write_json_exclusive(DESTINATION, payload)
    except shadow.V25ShadowReadinessError as exc:
        raise H0V25ProtocolFreezeError(str(exc)) from exc
    return payload


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--check", action="store_true")
    mode.add_argument("--freeze", action="store_true")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        payload = preflight_unfrozen() if args.check else freeze_protocol()
    except Exception as exc:
        print(
            f"H0/V25 protocol freeze failed closed: {type(exc).__name__}: {exc}",
            file=sys.stderr,
        )
        return 2
    print(json.dumps(payload, indent=2, sort_keys=True, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
