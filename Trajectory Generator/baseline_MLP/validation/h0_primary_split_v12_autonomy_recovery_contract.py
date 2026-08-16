"""Pure design contract for the V12 autonomy/recovery lineage.

V12 preserves the terminal V11 failure byte-for-byte and changes the
scientific protocol in three ways:

* the initial fit corpus includes the six frozen V11 safe-DAgger traces;
* states reached by each pure probe are labelled only after the rollout by an
  observer-only coherent teacher replay, then receive preregistered causal
  risk weights derived from ``previous_penetration_m``;
* every P0--P3 candidate receives a full pure probe before the next stage.

This module performs no I/O, fitting, rollout, or checkpoint mutation.  The
current authority covers design and protocol freeze only.  A separate,
no-clobber execution lock is required before any actor fit or environment
reset/step is allowed.
"""

from __future__ import annotations

import copy
import hashlib
import json
import math
from pathlib import PurePosixPath
from typing import Any, Mapping, Sequence

try:
    from validation import h0_primary_split_v11_weighted_full_mean_contract as prior
except ModuleNotFoundError:  # Flat import when validation/ is on sys.path.
    import h0_primary_split_v11_weighted_full_mean_contract as prior


SCHEMA_VERSION = 120
REVISION = "2026-08-09"
PROTOCOL_ID = "AB06_H0_PRIMARY_SPLIT_V12_V26_AUTONOMY_RECOVERY"
PIPELINE_ID = "H0_V12_V26_RECOVERY_WEIGHTED_PURE_PROBE_DAGGER"
FIT_CONTRACT_ID = "h0_primary_split_v12_recovery_weighted_full_mean_v1"

SOURCE_H0_ID = prior.SOURCE_H0_ID
SOURCE_H0_MODULE_PATH = prior.SOURCE_H0_MODULE_PATH
TARGET_CONTRACT_ID = prior.TARGET_CONTRACT_ID
EVENT_CONTRACT_ID = prior.EVENT_CONTRACT_ID
TEACHER_EVIDENCE_ID = prior.TEACHER_EVIDENCE_ID
TEACHER_EVIDENCE_RECEIPT_PATH = prior.TEACHER_EVIDENCE_RECEIPT_PATH
TEACHER_ID = prior.TEACHER_ID

EXPECTED_STEPS = prior.EXPECTED_STEPS
EXPECTED_ACTOR_FEATURES = prior.EXPECTED_ACTOR_FEATURES
EXPECTED_FULL_FEATURES = prior.EXPECTED_FULL_FEATURES
EXPECTED_ACTION_DIM = prior.EXPECTED_ACTION_DIM
EXPECTED_DTYPE = prior.EXPECTED_DTYPE
EXPECTED_CONTROL_WINDOWS = prior.EXPECTED_CONTROL_WINDOWS
EXPECTED_RAW_SENSOR_SAMPLES = prior.EXPECTED_RAW_SENSOR_SAMPLES
EXPECTED_POLICY_DT_S = prior.EXPECTED_POLICY_DT_S
EXPECTED_SAMPLE_DT_S = prior.EXPECTED_SAMPLE_DT_S
MORPHOLOGY_WEIGHT = prior.MORPHOLOGY_WEIGHT
PENETRATION_LIMIT_M = prior.PENETRATION_LIMIT_M
MINIMUM_VALID_CYCLES = prior.MINIMUM_VALID_CYCLES

FIT_STAGES = ("p0", "p1", "p2", "p3")
ROUND_ALPHAS = copy.deepcopy(prior.ROUND_ALPHAS)
FIT_COMPLETED_ROUNDS = copy.deepcopy(prior.FIT_COMPLETED_ROUNDS)
COLLECTION_CASE_IDS = prior.COLLECTION_CASE_IDS
SAFETY_LATCH_ACTIVATION_M = prior.prior.SAFETY_LATCH_ACTIVATION_M
SAFETY_LATCH_RELEASE_M = prior.prior.SAFETY_LATCH_RELEASE_M
SAFETY_LATCH_RELEASE_PHASE = prior.prior.SAFETY_LATCH_RELEASE_PHASE

# V12 P0 starts with the immutable V11 P3 corpus: 3,000 coherent-teacher base
# rows plus 3,000 same-state safe-DAgger rows.  Starting at P1, every fit also
# consumes the observer-only labels from all earlier pure probes.  New V12
# collection tranches are cumulative, but every fit still restarts from H0.
BASE_CORPUS_SAMPLE_COUNT = prior.BASE_CORPUS_SAMPLE_COUNT
V11_SEED_DAGGER_SAMPLE_COUNT = 3 * prior.COLLECTION_SAMPLES_PER_ROUND
V12_INITIAL_SAMPLE_COUNT = BASE_CORPUS_SAMPLE_COUNT + V11_SEED_DAGGER_SAMPLE_COUNT
COLLECTION_SAMPLES_PER_CASE = prior.COLLECTION_SAMPLES_PER_CASE
COLLECTION_CASE_COUNT_PER_ROUND = prior.COLLECTION_CASE_COUNT_PER_ROUND
COLLECTION_SAMPLES_PER_ROUND = prior.COLLECTION_SAMPLES_PER_ROUND
V11_SEED_RESET_ROWS = (
    prior.BASE_CORPUS_CASE_COUNT + 3 * prior.COLLECTION_CASE_COUNT_PER_ROUND
)

ACTOR_ARCHITECTURE = copy.deepcopy(prior.ACTOR_ARCHITECTURE)
BASE_CORPUS_NORMALIZATION = copy.deepcopy(prior.BASE_CORPUS_NORMALIZATION)
OFFLINE_THRESHOLDS = copy.deepcopy(prior.OFFLINE_THRESHOLDS)

# Weighting is causal: the action selector already knows the penetration from
# the previous policy boundary.  No current- or future-step outcome is used.
# The risk modifier applies only to observer-labelled *pure-branch* rows.  The
# V11 shielded rows remain weight 1 (apart from reset rows), because the V11
# forensic audit showed that P3 already imitates the teacher accurately on
# those states.  The endpoints reuse the existing 10/15 mm thresholds; 100 is
# the already-authorized V11 reset weight.
RECOVERY_WEIGHTING = {
    "signal": "previous_penetration_m",
    "causal_at_action_selection": True,
    "applies_to": "OBSERVER_LABELLED_PURE_PROBE_ROWS_ONLY",
    "shielded_dagger_risk_modifier": 1.0,
    "nominal_upper_m": 0.010,
    "latch_activation_m": 0.015,
    "nominal_weight": 1.0,
    "maximum_weight": 100.0,
    "near_latch_rule": "linear_1_to_100_between_10_and_15_mm",
    "reset_weight": 100.0,
    "weight_combination": "maximum_of_reset_recovery_and_coverage_weight",
    "episode_identity": "origin_family+round_or_probe_stage+case_id",
    "episode_mass_policy": "SCALE_EACH_EPISODE_RAW_WEIGHTS_TO_SUM_500",
    "episode_target_mass": 500.0,
    "row_loss": "MEAN_SQUARED_ERROR_OVER_TWO_ACTIONS",
    "corpus_loss_reduction": "SUM_WEIGHTED_ROW_LOSS_DIVIDED_BY_SUM_WEIGHTS",
    "failed_probe_prefix_kept": True,
    "terminal_row_kept": True,
    "sweep_authorized": False,
    "post_hoc_retuning_authorized": False,
}

# The coverage metric is available from the actor observation itself and is
# therefore causal.  Its threshold is the frozen leave-one-out p95 of the V11
# P3 corpus under V11's exact float32 base normalization, excluding the two
# disabled gait-clock columns.  It is used only to weight observer labels from
# pure probes; it never changes the served action or the physical gate.
COVERAGE_WEIGHTING = {
    "reference": "V11_P3_CORPUS_6000_ROWS",
    "reference_derivation_runtime": {
        "scope": "REFERENCE_RECEIPT_ONLY_NOT_TARGET_RUNTIME_REQUIREMENT",
        "system": "Darwin",
        "machine": "arm64",
        "python": "3.13.9",
        "numpy": "2.3.5",
        "scipy": "1.16.3",
    },
    "reference_observations_sha256": (
        "e5e8ce0691b907ff21ffdb59a3323b9b9a3e494954919a5f89712af9800b6072"
    ),
    "base_observations_sha256": (
        "d367a4697606f7c5d721823c973aabbbc86fb314e9b74e1529afc22e45a4d9ad"
    ),
    "normalization_mean_sha256": (
        "2b91c5d2f56ca4188e1e24830fb059ce4e36660337d3670a8ff618a1f5cc6fbe"
    ),
    "normalization_std_sha256": (
        "c17a91bb6746a0292c53c9eb08818c05cea24a5b15d32ccb1a4af958ad20b2c5"
    ),
    "normalized_observations_sha256": (
        "b6819a920953632751a3a25120d9fe387a21d86e890b5645e225b72e0640beb3"
    ),
    "included_feature_indices": list(range(2, 35)),
    "normalized_feature_matrix_sha256": (
        "9e51a72ff44a90509d0ba908fb86210352efd2160d8b13fb2ed98113904a3708"
    ),
    "metric": "nearest_neighbor_rms_z_float64_accumulation",
    "loo_tree": {
        "implementation": "scipy.spatial.cKDTree",
        "leafsize": 16,
        "compact_nodes": True,
        "balanced_tree": True,
        "query_k": 8,
        "eps": 0.0,
        "p": 2,
        "workers": 1,
        "self_excluded": True,
        "tie_break": "minimum_index",
    },
    "tie_audit": {
        "extended_query_k": 64,
        "unique_observation_count": 5990,
        "maximum_minimum_distance_tie_count": 6,
        "query_k_matches_extended_query": True,
    },
    "loo_p95": 0.07945888479650812,
    "loo_nearest_indices_sha256": (
        "aa79fa7e9ab5c4145caa332f9984612d009af2fee7771550ab68db908ce42639"
    ),
    "loo_distances_sha256": (
        "17f87f8cdca0b9429dd760b8bf94483bf7a317fb1a85ffa6f6b474eb7ad19673"
    ),
    "out_of_distribution_rule": "distance_strictly_greater_than_loo_p95",
    "new_row_query": {
        "input_cast": "contiguous_float32",
        "normalization": "subtract_float32_mean_divide_float32_std_to_float32",
        "included_feature_order": list(range(2, 35)),
        "reference_candidate_count": 6000,
        "global_minimum_candidate_audit_k": 64,
        "delta_cast": "float64_before_subtraction",
        "distance": "sqrt(fsum(delta_i_squared_in_feature_order)/33)",
        "selection": "global_minimum_distance_then_minimum_reference_index",
        "global_minimum_certificate": (
            "TOP64_BY_FLOAT64_VECTOR_SUM_THEN_FSUM;_REQUIRE_EXCLUDED_MARGIN_"
            "GREATER_THAN_1E-10_TIMES_MAX_1_OR_EXCLUDED_MIN_SUM"
        ),
        "execution_time": "OBSERVER_LABEL_STAGE_ONLY_AFTER_PROBE_CLOSED",
    },
    "out_of_distribution_weight": 100.0,
    "applies_to": "OBSERVER_LABELLED_PURE_PROBE_ROWS_ONLY",
    "runtime_actor_feature_change": False,
    "target_platform_policy": (
        "FAIL_CLOSED_UNLESS_EXACT_REFERENCE_HASH_AND_P95_PARITY_AUDIT_PASSES_"
        "BEFORE_EXECUTION_LOCK"
    ),
}

FIT = {
    **copy.deepcopy(prior.FIT),
    "fit_contract_id": FIT_CONTRACT_ID,
    "sample_weighting": copy.deepcopy(RECOVERY_WEIGHTING),
    "coverage_weighting": copy.deepcopy(COVERAGE_WEIGHTING),
    "seed_corpus": "V11_P3_6000_ROWS_BYTE_BOUND_WITHOUT_RISK_REWEIGHTING",
    "pure_probe_label_corpus": "CUMULATIVE_OBSERVER_ONLY_SAME_STATE_LABELS",
    "every_fit_restarts_from_h0": True,
}
FIT_REPORT_CHECK_NAMES = ("corpus_exact", "module_reload_exact")

# A collection remains useful as labelled corrective data even when its latch
# dependence audit fails.  It must never be called autonomous evidence.
LATCH_INDEPENDENCE = {
    "forced_teacher_takeover_count_max": 0,
    "forced_teacher_takeover_fraction_max": 0.0,
    "max_consecutive_takeover_steps_max": 0,
    "latch_active_at_episode_end_required": False,
    "scientific_meaning": "NO_FORCED_TAKEOVER;_NOT_AUTONOMY_EVIDENCE",
}

V12_VALIDATION_ROOT = PurePosixPath("Trajectory Generator/baseline_MLP/validation")
RUN_ROOT = V12_VALIDATION_ROOT / "h0_v12_runs/2026-08-09_v12_v26_recovery"
FIT_ROOT = RUN_ROOT / "fit"
FIT_ROOTS = {stage: FIT_ROOT / stage for stage in FIT_STAGES}
MODULE_PATHS = {
    stage: FIT_ROOTS[stage] / "rl_module_target_adapted" for stage in FIT_STAGES
}
FIT_CORPUS_PATHS = {stage: FIT_ROOTS[stage] / "corpus.npz" for stage in FIT_STAGES}
FIT_RECEIPT_PATHS = {stage: FIT_ROOTS[stage] / "receipt.json" for stage in FIT_STAGES}
PROBE_ROOT = RUN_ROOT / "probe"
PROBE_RECEIPT_PATHS = {
    stage: PROBE_ROOT / stage / "receipt.json" for stage in FIT_STAGES
}
LABEL_ROOT = RUN_ROOT / "label"
LABEL_RECEIPT_PATHS = {
    stage: LABEL_ROOT / stage / "receipt.json" for stage in FIT_STAGES
}
LABEL_CORPUS_PATHS = {
    stage: LABEL_ROOT / stage / "observer_labels.npz" for stage in FIT_STAGES
}
COLLECTION_ROOT = RUN_ROOT / "collect"
FINAL_ROOT = RUN_ROOT / "final"
CANDIDATE_FREEZE_PATH = FIT_ROOT / "candidate_freeze.json"
FINAL_DEVELOPMENT_RECEIPT_PATH = FINAL_ROOT / "receipt.json"
PIPELINE_CLAIM_PATH = RUN_ROOT / "pipeline_claim.json"
PIPELINE_LEDGER_PATH = RUN_ROOT / "pipeline_ledger.json"
WORKER_CLAIMS_ROOT = RUN_ROOT / "claims"

PROTOCOL_FREEZE_PATH = V12_VALIDATION_ROOT / (
    "h0_primary_split_v12_autonomy_recovery_protocol_freeze.json"
)
EXECUTION_LOCK_PATH = V12_VALIDATION_ROOT / (
    "h0_primary_split_v12_autonomy_recovery_execution_lock.json"
)
DESIGN_AUDIT_RECEIPT_PATH = V12_VALIDATION_ROOT / (
    "h0_primary_split_v12_autonomy_recovery_design_audit.json"
)

V11_RUN_ROOT = prior.RUN_ROOT
V11_TERMINAL_LEDGER_PATH = prior.PIPELINE_LEDGER_PATH
V11_CANDIDATE_FREEZE_PATH = prior.CANDIDATE_FREEZE_PATH
V11_P3_CORPUS_PATH = prior.FIT_ROOTS["p3"] / "corpus.npz"
V11_FINAL_FAILURE_CASE_ID = "deterministic_offset_minus_0p20"
V11_FINAL_FAILURE_ROOT = prior.FINAL_ROOT / V11_FINAL_FAILURE_CASE_ID
V11_FINAL_FAILURE_SUMMARY_PATH = V11_FINAL_FAILURE_ROOT / "summary.json"
V11_FINAL_FAILURE_GATE_PATH = V11_FINAL_FAILURE_ROOT / "gate.json"
V11_FINAL_FAILURE_PATH = V11_FINAL_FAILURE_ROOT / "failure.json"
V11_FINAL_FAILURE_TRACE_PATH = V11_FINAL_FAILURE_ROOT / "trace.json"
V11_PURE_FAILURE_COVERAGE = {
    "trace_observations_sha256": (
        "da402e03efb63a38271837d583b5ae17c0432235347633787c74f1d2322d3e34"
    ),
    "normalized_features_sha256": (
        "30aaf23934fb743e17962aa4c430941496ef19310fecb22b8d64d832625e2043"
    ),
    "nearest_indices_sha256": (
        "2d6c34c4bc3d292f37656caeab03ba34c037229fbe48c20011159aa18cd5eb35"
    ),
    "nearest_distances_sha256": (
        "2be58bb58dbb70af4754874be96847a44b579ac4ea4eb5dafdedc8282a5da75b"
    ),
    "row_count": 259,
    "ood_row_count": 164,
    "all_steps_201_through_259_ood": True,
    "step_201_distance": 0.22454978476138582,
    "step_259_distance": 0.4934425626710174,
}
V11_AUTHORITATIVE_SHA256 = {
    "v10s_blend": "9e11a94eea1c32c187b7a0cf875d889e530252a3fae64cd017c87a2853b1f57d",
    "v10s_contract": "5c6ea0993cb7585e7a588542382350bfdd1fd27b425373ff696d6ccb9cf77e0a",
    "forensic_writer": "e305729eb896335ebb9f492ce449cc5e7656f4fce5968e3fa4359daf38a02ebd",
    "v11_contract": "1ee981c087973dd417f88214894171be21f75fde54fd821d8a0eb2958db34729",
    "v11_fitter": "877bb79d9bf4bd8d87b9d07dbdee127ca259cc8cc82fac1e592ea37032029fdc",
    "v11_runner": "ada9cc7bdd4c874caee39ce645cba5bc4d2fcf166b8975fd55acf78bba96d559",
    "v11_design_audit": "96f848d72e2aa72bec4bb108416a84c38730a7b9ccc2d1f36bd172f5a897bacf",
    "v11_preflight": "e5cc499eac563516054fe10497bd54ae60790750cc4a8cc92c14f876cd5864a5",
    "v11_execution_lock": "deba2d6e69e97f3b6814e8a7dcbd79cbef6f1ec7f938c94b6d6604c87e4404f1",
    "v11_terminal_ledger": "130b64f3e653e0d0afa8602c6ee65e0c20204fb3cace53365cdb9e0240bb4efc",
    "v11_candidate_freeze": "f2b496d18e26eb06e73672cb32b197ab54fe56e1cf1edbee4bd0a58fa6c72735",
    "v11_p3_corpus": "232e0776f67e7a1425288c4f3979409df998ef34a7e60c618ec6c5d7cd9c4933",
    "v11_final_failure_summary": "5efdfebc88d368476cecf4a8e42c3772faff93fc95dc4bbdc23d199a2526d143",
    "v11_final_failure_gate": "a73e898d5e618defe67b5fb668bf8bf281fe8d8f4d19649807eaf176f828cfcb",
    "v11_final_failure": "9b25a6870ccc537b823cd5d9f388939b75c1cd4929b6376670bb8a239bc8727c",
    "v11_final_failure_trace": "88934178abaec23ed302113f7fec3fe6a5082af53f71441f01a5a80fe85883e0",
    "teacher_evidence": "ba0d037f6229f9172d23ef5f349c64a536d3fe6253ecaeb13794399183007b0b",
}
SOURCE_H0_TREE_SHA256 = (
    "f7f6c898975af109412af8c3f1a338b5076f9fefcec1e2723673fd821f1f13ee"
)
V11_P3_CORPUS_ARTIFACT = {
    "path": V11_P3_CORPUS_PATH.as_posix(),
    "sha256": V11_AUTHORITATIVE_SHA256["v11_p3_corpus"],
    "size_bytes": 7_913_274,
}
TEACHER_EVIDENCE_ARTIFACT = {
    "path": TEACHER_EVIDENCE_RECEIPT_PATH.as_posix(),
    "sha256": V11_AUTHORITATIVE_SHA256["teacher_evidence"],
    "size_bytes": 4_717,
}

V11_COLLECTION_INPUTS = tuple(
    {
        "round_index": round_index,
        "case_id": case_id,
        "root": PurePosixPath(
            prior.canonical_collection_case(case_id, round_index)["destination"]
        ),
    }
    for round_index in (1, 2, 3)
    for case_id in COLLECTION_CASE_IDS
)


def _copy_case(case_id: str) -> dict[str, Any]:
    matches = [case for case in prior.FINAL_CASES if case["case_id"] == case_id]
    if len(matches) != 1:
        raise RuntimeError(f"V11 case binding drifted: {case_id}")
    return copy.deepcopy(matches[0])


# The same case that exposed V11 is used after every candidate.  P0/P1/P2
# failures are terminal for that candidate's autonomy claim but may proceed to
# the one preregistered shielded collection; P3 must pass before freeze.
PROBE_CASE = _copy_case(V11_FINAL_FAILURE_CASE_ID)
PROBE_CASE.pop("destination", None)
PROBE_BEHAVIOR = "PURE_UNBLENDED_NO_TEACHER_NO_LATCH"
PROBE_REPLAY_SCHEMA = {
    "schema_id": "H0_V12_OBSERVER_REPLAY_BOUNDARIES_V1",
    "container": "NPZ_ALLOW_PICKLE_FALSE",
    "boundary_arrays": {
        "boundary_time_s": "float64[N+1]",
        "pros_knee_angle_rad": "float32[N+1]",
        "pros_knee_speed_rad_s": "float32[N+1]",
        "pros_ankle_angle_rad": "float32[N+1]",
        "pros_ankle_speed_rad_s": "float32[N+1]",
        "legacy_left_normal_grf_bw": "float32[N+1]",
        "legacy_left_in_contact": "bool[N+1]",
        "legacy_heel_strike_event": "bool[N+1]",
        "legacy_toe_off_event": "bool[N+1]",
        "legacy_last_hs_time_s": "float64[N+1]",
        "legacy_last_to_time_s": "float64[N+1]",
        "legacy_phase_state": "int8[N+1]",
    },
    "step_arrays": {
        "actor_observations": "float32[N,35]",
        "previous_penetration_m": "float64[N]",
    },
    "scalar_arrays": {
        "body_weight_n": "float64[1]",
        "legacy_fsm_config_sha256": "ascii_lower_hex_u1[64]",
        "event_contract_id": "utf8_u1[len]",
    },
    "teacher_mutable_actor_columns": list(range(10, 24)),
    "all_other_actor_columns_byte_exact": True,
}

COLLECTION_CASES = tuple(
    {
        **{
            key: value
            for key, value in _copy_case(case_id).items()
            if key != "destination"
        },
        "destination": (COLLECTION_ROOT / f"r{round_index}" / case_id).as_posix(),
        "round_index": round_index,
        "requested_alpha": ROUND_ALPHAS[round_index],
        "candidate_fit_stage": f"p{round_index - 1}",
    }
    for round_index in (1, 2, 3)
    for case_id in COLLECTION_CASE_IDS
)

FINAL_CASES = tuple(
    {
        **{key: value for key, value in case.items() if key != "destination"},
        "destination": (FINAL_ROOT / case["case_id"]).as_posix(),
    }
    for case in prior.FINAL_CASES
)
FINAL_CASE_IDS = tuple(case["case_id"] for case in FINAL_CASES)

STAGE_IDS = (
    "fit_p0",
    "probe_p0",
    "label_p0",
    *(f"collect_r1__{case_id}" for case_id in COLLECTION_CASE_IDS),
    "fit_p1",
    "probe_p1",
    "label_p1",
    *(f"collect_r2__{case_id}" for case_id in COLLECTION_CASE_IDS),
    "fit_p2",
    "probe_p2",
    "label_p2",
    *(f"collect_r3__{case_id}" for case_id in COLLECTION_CASE_IDS),
    "fit_p3",
    "probe_p3",
    "label_p3",
    "freeze_p3",
    *(f"final__{case_id}" for case_id in FINAL_CASE_IDS),
    "finalize_development",
)

AUTHORITY = {
    "authority_date": REVISION,
    "authority_text": "procedi",
    "authority_scope": "V12_DESIGN_AND_PROTOCOL_FREEZE_ONLY",
    "design_and_freeze_authorized": True,
    "pipeline_execution_authorized": False,
    "actor_fit_execution_authorized": False,
    "environment_reset_authorized": False,
    "environment_step_authorized": False,
    "design_audit_fit_authorized": False,
    "offline_teacher_labeling_authorized": False,
    "critic_updates_authorized": False,
    "ppo_updates_authorized": False,
    "protected_trial_access_authorized": False,
    "reserve_trial_access_authorized": False,
    "runtime_promotion_authorized": False,
    "physical_gate_relaxation_authorized": False,
    "primary_grf_modification_authorized": False,
    "detector_or_fsm_modification_authorized": False,
    "sea_semantic_modification_authorized": False,
    "retry_authorized": False,
    "sweep_authorized": False,
    "rescue_authorized": False,
}

SOURCE_RELATIVE_PATHS = {
    "contract": (
        "Trajectory Generator/baseline_MLP/validation/"
        "h0_primary_split_v12_autonomy_recovery_contract.py"
    ),
    "freeze": (
        "Trajectory Generator/baseline_MLP/validation/"
        "freeze_h0_primary_split_v12_autonomy_recovery.py"
    ),
    "contract_tests": (
        "Trajectory Generator/baseline_MLP/validation/"
        "test_h0_primary_split_v12_autonomy_recovery_contract.py"
    ),
    "freeze_tests": (
        "Trajectory Generator/baseline_MLP/validation/"
        "test_freeze_h0_primary_split_v12_autonomy_recovery.py"
    ),
    "protocol_plan": (
        "reports/plans/2026-08-09_protocollo_h0_v12_autonomy_recovery.md"
    ),
    "line_endings": ".gitattributes",
    "forensic_writer": "validation/h0_forensic_rollout.py",
    "v10s_blend": "validation/h0_primary_split_v10s_blend.py",
    "v10s_contract": "validation/h0_primary_split_v10s_safe_dagger_contract.py",
    "v11_contract": ("validation/h0_primary_split_v11_weighted_full_mean_contract.py"),
    "v11_fitter": "validation/h0_primary_split_v11_weighted_fit.py",
    "v11_runner": "validation/run_h0_primary_split_v11_weighted_full_mean.py",
}

INPUT_RELATIVE_PATHS = {
    "v11_design_audit": prior.DESIGN_AUDIT_RECEIPT_PATH.as_posix(),
    "v11_preflight": prior.PREFLIGHT_PATH.as_posix(),
    "v11_execution_lock": prior.LOCK_PATH.as_posix(),
    "v11_terminal_ledger": V11_TERMINAL_LEDGER_PATH.as_posix(),
    "v11_candidate_freeze": V11_CANDIDATE_FREEZE_PATH.as_posix(),
    "v11_p3_corpus": V11_P3_CORPUS_PATH.as_posix(),
    "v11_final_failure_summary": V11_FINAL_FAILURE_SUMMARY_PATH.as_posix(),
    "v11_final_failure_gate": V11_FINAL_FAILURE_GATE_PATH.as_posix(),
    "v11_final_failure": V11_FINAL_FAILURE_PATH.as_posix(),
    "v11_final_failure_trace": V11_FINAL_FAILURE_TRACE_PATH.as_posix(),
    "teacher_evidence": TEACHER_EVIDENCE_RECEIPT_PATH.as_posix(),
}
for _row in V11_COLLECTION_INPUTS:
    _prefix = f"v11_r{_row['round_index']}_{_row['case_id']}"
    INPUT_RELATIVE_PATHS[f"{_prefix}_trace"] = (_row["root"] / "trace.json").as_posix()
    INPUT_RELATIVE_PATHS[f"{_prefix}_summary"] = (
        _row["root"] / "summary.json"
    ).as_posix()
    INPUT_RELATIVE_PATHS[f"{_prefix}_gate"] = (_row["root"] / "gate.json").as_posix()
    INPUT_RELATIVE_PATHS[f"{_prefix}_receipt"] = (
        _row["root"] / "receipt.json"
    ).as_posix()

FUTURE_EXECUTION_SOURCES_REQUIRED = (
    "v12_recovery_weighted_fitter",
    "v12_pure_probe_observer_labeler",
    "v12_design_audit_runner",
    "v12_pipeline_runner",
    "v12_execution_tests",
)

PROTOCOL_FREEZE_PASS_STATUS = "PASS_H0_PRIMARY_SPLIT_V12_PROTOCOL_FREEZE"
PROTOCOL_FREEZE_FAIL_STATUS = "FAIL_H0_PRIMARY_SPLIT_V12_PROTOCOL_FREEZE"
FIT_COMPLETE_STATUS = "COMPLETE_H0_PRIMARY_SPLIT_V12_RECOVERY_WEIGHTED_FIT"
FIT_PASS_STATUS = "PASS_H0_PRIMARY_SPLIT_V12_RECOVERY_WEIGHTED_FIT"
FIT_FAIL_STATUS = "FAIL_H0_PRIMARY_SPLIT_V12_RECOVERY_WEIGHTED_FIT"
PROBE_INTEGRITY_PASS_STATUS = "PASS_H0_PRIMARY_SPLIT_V12_PROBE_INTEGRITY"
PROBE_INTEGRITY_FAIL_STATUS = "FAIL_H0_PRIMARY_SPLIT_V12_PROBE_INTEGRITY"
PURE_PROBE_COMPLETE_STATUS = "COMPLETE_H0_PRIMARY_SPLIT_V12_PURE_PROBE"
PURE_PROBE_PASS_STATUS = "PASS_H0_PRIMARY_SPLIT_V12_PURE_PROBE"
PURE_PROBE_FAIL_STATUS = "FAIL_H0_PRIMARY_SPLIT_V12_CANDIDATE_AUTONOMY"
OBSERVER_LABEL_PASS_STATUS = "PASS_H0_PRIMARY_SPLIT_V12_PROBE_OBSERVER_LABELS"
OBSERVER_LABEL_FAIL_STATUS = "FAIL_H0_PRIMARY_SPLIT_V12_PROBE_OBSERVER_LABELS"
COLLECTION_COMPLETE_STATUS = "COMPLETE_H0_PRIMARY_SPLIT_V12_SHIELDED_COLLECTION"
COLLECTION_PASS_STATUS = "PASS_H0_PRIMARY_SPLIT_V12_COLLECTION_DATA"
COLLECTION_FAIL_STATUS = "FAIL_H0_PRIMARY_SPLIT_V12_COLLECTION_DATA"
LATCH_INDEPENDENCE_PASS_STATUS = "PASS_H0_PRIMARY_SPLIT_V12_NO_FORCED_TAKEOVER"
LATCH_INDEPENDENCE_FAIL_STATUS = "FAIL_H0_PRIMARY_SPLIT_V12_LATCH_DEPENDENT"
CANDIDATE_FREEZE_PASS_STATUS = "PASS_H0_PRIMARY_SPLIT_V12_CANDIDATE_FREEZE"
CANDIDATE_FREEZE_FAIL_STATUS = "FAIL_H0_PRIMARY_SPLIT_V12_CANDIDATE_FREEZE"
FINAL_ROLLOUT_COMPLETE_STATUS = "COMPLETE_H0_PRIMARY_SPLIT_V12_FINAL_ROLLOUT"
FINAL_ROLLOUT_PASS_STATUS = "PASS_H0_PRIMARY_SPLIT_V12_FINAL_ROLLOUT"
FINAL_ROLLOUT_FAIL_STATUS = "FAIL_H0_PRIMARY_SPLIT_V12_FINAL_ROLLOUT"
FINAL_DEVELOPMENT_COMPLETE_STATUS = "COMPLETE_H0_PRIMARY_SPLIT_V12_FINAL_DEVELOPMENT"
FINAL_DEVELOPMENT_PASS_STATUS = "PASS_H0_PRIMARY_SPLIT_V12_FINAL_DEVELOPMENT"
FINAL_DEVELOPMENT_FAIL_STATUS = "FAIL_H0_PRIMARY_SPLIT_V12_FINAL_DEVELOPMENT"

artifact_record_matches = prior.artifact_record_matches
tree_record_matches = prior.tree_record_matches


def _finite_number(value: Any) -> bool:
    return (
        not isinstance(value, bool)
        and isinstance(value, (int, float))
        and math.isfinite(float(value))
    )


def _nonnegative_int(value: Any) -> bool:
    return type(value) is int and value >= 0


def _metric_triplet(value: Any) -> bool:
    return isinstance(value, Mapping) and all(
        _finite_number(value.get(name)) and float(value[name]) >= 0.0
        for name in ("rmse", "max_abs_error", "reset_max_abs_error")
    )


def _metrics_within_thresholds(value: Mapping[str, Any]) -> bool:
    return (
        float(value["rmse"]) <= OFFLINE_THRESHOLDS["rmse_max"]
        and float(value["max_abs_error"]) <= OFFLINE_THRESHOLDS["max_abs_error_max"]
        and float(value["reset_max_abs_error"])
        <= OFFLINE_THRESHOLDS["reset_max_abs_error_max"]
    )


def _sha256(value: Any) -> bool:
    return (
        isinstance(value, str)
        and len(value) == 64
        and all(character in "0123456789abcdef" for character in value)
    )


def _canonical_json_artifact_matches(payload: Any, record: Any) -> bool:
    if not isinstance(payload, Mapping) or not isinstance(record, Mapping):
        return False
    try:
        encoded = (
            json.dumps(
                payload,
                indent=2,
                sort_keys=True,
                ensure_ascii=False,
                allow_nan=False,
            )
            + "\n"
        ).encode("utf-8")
    except (TypeError, ValueError):
        return False
    return record.get("sha256") == hashlib.sha256(encoded).hexdigest() and record.get(
        "size_bytes"
    ) == len(encoded)


def candidate_id(stage: str, module_tree_sha256: str) -> str:
    if stage not in FIT_STAGES or not _sha256(module_tree_sha256):
        raise ValueError("candidate id requires a known stage and lowercase SHA-256")
    return f"{PROTOCOL_ID}:{stage}:{module_tree_sha256[:16]}"


def recovery_row_class(
    previous_penetration_m: float,
    *,
    reset_row: bool = False,
) -> str:
    """Classify one row using only information causal at action selection."""

    if not _finite_number(previous_penetration_m) or previous_penetration_m < 0:
        raise ValueError("previous penetration must be finite and nonnegative")
    if type(reset_row) is not bool:
        raise TypeError("reset_row must be bool")
    if reset_row:
        return "reset"
    if previous_penetration_m <= RECOVERY_WEIGHTING["nominal_upper_m"]:
        return "nominal"
    if previous_penetration_m < RECOVERY_WEIGHTING["latch_activation_m"]:
        return "near_latch"
    return "activation_boundary_without_latch"


def recovery_sample_weight(
    previous_penetration_m: float,
    *,
    reset_row: bool = False,
    coverage_distance_rms_z: float | None = None,
) -> float:
    """Return the fixed causal V12 sample weight for one labelled row."""

    row_class = recovery_row_class(
        previous_penetration_m,
        reset_row=reset_row,
    )
    if row_class in {"reset", "activation_boundary_without_latch"}:
        recovery_weight = float(RECOVERY_WEIGHTING["maximum_weight"])
    elif row_class == "nominal":
        recovery_weight = float(RECOVERY_WEIGHTING["nominal_weight"])
    else:
        lower = float(RECOVERY_WEIGHTING["nominal_upper_m"])
        upper = float(RECOVERY_WEIGHTING["latch_activation_m"])
        ratio = (float(previous_penetration_m) - lower) / (upper - lower)
        recovery_weight = 1.0 + 99.0 * ratio
    if coverage_distance_rms_z is None:
        coverage_weight = 1.0
    else:
        if not _finite_number(coverage_distance_rms_z) or coverage_distance_rms_z < 0:
            raise ValueError("coverage distance must be finite and nonnegative")
        coverage_weight = (
            float(COVERAGE_WEIGHTING["out_of_distribution_weight"])
            if float(coverage_distance_rms_z) > COVERAGE_WEIGHTING["loo_p95"]
            else 1.0
        )
    return max(recovery_weight, coverage_weight)


def shielded_sample_weight(*, reset_row: bool = False) -> float:
    """Return the preregistered raw weight for base or shielded DAgger rows."""

    if type(reset_row) is not bool:
        raise TypeError("reset_row must be bool")
    return 100.0 if reset_row else 1.0


def normalize_episode_sample_weights(raw_weights: Sequence[float]) -> tuple[float, ...]:
    """Scale one episode to exactly the fixed 500-unit target mass."""

    raw = tuple(raw_weights)
    if not raw or any(not _finite_number(value) or value <= 0.0 for value in raw):
        raise ValueError("episode raw weights must be finite, positive, and nonempty")
    values = tuple(float(value) for value in raw)
    total = math.fsum(values)
    scale = float(RECOVERY_WEIGHTING["episode_target_mass"]) / total
    return tuple(value * scale for value in values)


def weighted_mean_row_loss(
    row_losses: Sequence[float], sample_weights: Sequence[float]
) -> float:
    """Reduce per-row two-action MSE values using one exact weighted mean."""

    raw_losses = tuple(row_losses)
    raw_weights = tuple(sample_weights)
    if not raw_losses or len(raw_losses) != len(raw_weights):
        raise ValueError("row losses and weights must have equal nonzero length")
    if any(not _finite_number(value) or value < 0.0 for value in raw_losses):
        raise ValueError("row losses must be finite and nonnegative")
    if any(not _finite_number(value) or value <= 0.0 for value in raw_weights):
        raise ValueError("sample weights must be finite and positive")
    losses = tuple(float(value) for value in raw_losses)
    weights = tuple(float(value) for value in raw_weights)
    return math.fsum(
        loss * weight for loss, weight in zip(losses, weights, strict=True)
    ) / math.fsum(weights)


def expected_fit_counts(
    stage: str,
    *,
    labelled_probe_rows: Mapping[str, int] | None = None,
) -> dict[str, Any]:
    if stage not in FIT_STAGES:
        raise ValueError(f"unknown V12 fit stage: {stage!r}")
    rounds = FIT_COMPLETED_ROUNDS[stage]
    required_probe_stages = FIT_STAGES[: FIT_STAGES.index(stage)]
    observed = {} if labelled_probe_rows is None else dict(labelled_probe_rows)
    if set(observed) != set(required_probe_stages):
        raise ValueError(
            f"{stage} labelled probe stages {sorted(observed)} != "
            f"{list(required_probe_stages)}"
        )
    if any(
        type(count) is not int or count < 1 or count > EXPECTED_STEPS
        for count in observed.values()
    ):
        raise ValueError("labelled pure-probe row counts must be within 1..500")
    pure_rows = sum(observed.values())
    new_samples = len(rounds) * COLLECTION_SAMPLES_PER_ROUND
    return {
        "v11_seed_sample_count": V12_INITIAL_SAMPLE_COUNT,
        "v12_dagger_sample_count": new_samples,
        "pure_probe_label_sample_count": pure_rows,
        "pure_probe_label_stages": list(required_probe_stages),
        "sample_count": V12_INITIAL_SAMPLE_COUNT + new_samples + pure_rows,
        "reset_row_count": V11_SEED_RESET_ROWS
        + len(rounds) * COLLECTION_CASE_COUNT_PER_ROUND
        + len(required_probe_stages),
        "completed_v12_collection_rounds": list(rounds),
    }


def _probe_label_bindings(
    value: Any, *, stage: str, pipeline_claim: Any
) -> tuple[bool, dict[str, int]]:
    required_stages = FIT_STAGES[: FIT_STAGES.index(stage)]
    if not isinstance(value, list) or len(value) != len(required_stages):
        return False, {}
    counts: dict[str, int] = {}
    for binding, probe_stage in zip(value, required_stages, strict=True):
        if not isinstance(binding, Mapping):
            return False, {}
        count = binding.get("labelled_row_count")
        receipt = binding.get("receipt")
        receipt_payload = binding.get("receipt_payload")
        label_corpus = binding.get("label_corpus")
        if (
            set(binding)
            != {
                "fit_stage",
                "receipt",
                "receipt_payload",
                "label_corpus",
                "labelled_row_count",
                "worker_claim",
                "passed",
            }
            or binding.get("fit_stage") != probe_stage
            or binding.get("passed") is not True
            or type(count) is not int
            or count < 1
            or count > EXPECTED_STEPS
            or not artifact_record_matches(receipt, LABEL_RECEIPT_PATHS[probe_stage])
            or not _canonical_json_artifact_matches(receipt_payload, receipt)
            or not artifact_record_matches(
                label_corpus, LABEL_CORPUS_PATHS[probe_stage]
            )
            or not isinstance(receipt_payload, Mapping)
            or set(receipt_payload)
            != {
                "schema_version",
                "status",
                "passed",
                "protocol_id",
                "fit_stage",
                "labelled_row_count",
                "label_corpus",
                "pipeline_claim",
                "worker_claim",
            }
            or receipt_payload.get("schema_version") != SCHEMA_VERSION
            or receipt_payload.get("status") != OBSERVER_LABEL_PASS_STATUS
            or receipt_payload.get("passed") is not True
            or receipt_payload.get("protocol_id") != PROTOCOL_ID
            or receipt_payload.get("fit_stage") != probe_stage
            or receipt_payload.get("labelled_row_count") != count
            or receipt_payload.get("label_corpus") != label_corpus
            or receipt_payload.get("pipeline_claim") != pipeline_claim
            or binding.get("worker_claim") != receipt_payload.get("worker_claim")
            or not artifact_record_matches(
                binding.get("worker_claim"),
                worker_claim_path(f"label_{probe_stage}"),
            )
        ):
            return False, {}
        counts[probe_stage] = count
    return True, counts


def _fit_collection_corpus_binding_matches(
    binding: Any, *, round_index: int, case_id: str, pipeline_claim: Any
) -> bool:
    if not isinstance(binding, Mapping):
        return False
    root = PurePosixPath(canonical_collection_case(case_id, round_index)["destination"])
    receipt = binding.get("receipt")
    receipt_payload = binding.get("receipt_payload")
    label_corpus = binding.get("label_corpus")
    return (
        set(binding)
        == {
            "round_index",
            "case_id",
            "receipt",
            "receipt_payload",
            "label_corpus",
            "worker_claim",
            "data_passed",
        }
        and binding.get("round_index") == round_index
        and binding.get("case_id") == case_id
        and binding.get("data_passed") is True
        and artifact_record_matches(
            receipt, stage_receipt_path(f"collect_r{round_index}__{case_id}")
        )
        and _canonical_json_artifact_matches(receipt_payload, receipt)
        and artifact_record_matches(label_corpus, root / "labels.npz")
        and isinstance(receipt_payload, Mapping)
        and set(receipt_payload)
        == {
            "schema_version",
            "status",
            "passed",
            "protocol_id",
            "round_index",
            "case_id",
            "sample_count",
            "label_corpus",
            "pipeline_claim",
            "worker_claim",
        }
        and receipt_payload.get("schema_version") == SCHEMA_VERSION
        and receipt_payload.get("status") == COLLECTION_PASS_STATUS
        and receipt_payload.get("passed") is True
        and receipt_payload.get("protocol_id") == PROTOCOL_ID
        and receipt_payload.get("round_index") == round_index
        and receipt_payload.get("case_id") == case_id
        and receipt_payload.get("sample_count") == EXPECTED_STEPS
        and receipt_payload.get("label_corpus") == label_corpus
        and receipt_payload.get("pipeline_claim") == pipeline_claim
        and binding.get("worker_claim") == receipt_payload.get("worker_claim")
        and artifact_record_matches(
            binding.get("worker_claim"),
            worker_claim_path(f"collect_r{round_index}__{case_id}"),
        )
    )


def _fit_collection_corpus_bindings_match(
    value: Any, *, stage: str, pipeline_claim: Any
) -> bool:
    expected = [
        (round_index, case_id)
        for round_index in FIT_COMPLETED_ROUNDS[stage]
        for case_id in COLLECTION_CASE_IDS
    ]
    return (
        isinstance(value, list)
        and len(value) == len(expected)
        and all(
            _fit_collection_corpus_binding_matches(
                binding,
                round_index=round_index,
                case_id=case_id,
                pipeline_claim=pipeline_claim,
            )
            for binding, (round_index, case_id) in zip(value, expected, strict=True)
        )
    )


def _fit_component_receipts_match(value: Any, *, stage: str) -> bool:
    return (
        isinstance(value, Mapping)
        and set(value)
        == {"design_audit", "execution_lock", "pipeline_claim", "worker_claim"}
        and artifact_record_matches(
            value.get("design_audit"), DESIGN_AUDIT_RECEIPT_PATH
        )
        and artifact_record_matches(value.get("execution_lock"), EXECUTION_LOCK_PATH)
        and artifact_record_matches(value.get("pipeline_claim"), PIPELINE_CLAIM_PATH)
        and artifact_record_matches(
            value.get("worker_claim"), worker_claim_path(f"fit_{stage}")
        )
    )


def _fit_artifacts_match(value: Any, *, stage: str) -> bool:
    root = FIT_ROOTS[stage]
    return (
        isinstance(value, Mapping)
        and set(value) == {"corpus", "adaptation_history", "adaptation_report"}
        and artifact_record_matches(value.get("corpus"), FIT_CORPUS_PATHS[stage])
        and artifact_record_matches(
            value.get("adaptation_history"), root / "adaptation_history.json"
        )
        and artifact_record_matches(
            value.get("adaptation_report"), root / "adaptation_report.json"
        )
    )


def _fit_corpus_audit_matches(value: Any, expected: Mapping[str, Any]) -> bool:
    expected_keys = {
        "v11_seed_sample_count",
        "v12_dagger_sample_count",
        "same_state_v12_dagger_sample_count",
        "pure_probe_label_sample_count",
        "same_state_pure_probe_label_sample_count",
        "sample_count",
        "reset_row_count",
        "duplicate_sample_count",
        "all_finite",
    }
    return (
        isinstance(value, Mapping)
        and set(value) == expected_keys
        and value.get("v11_seed_sample_count") == expected["v11_seed_sample_count"]
        and value.get("v12_dagger_sample_count") == expected["v12_dagger_sample_count"]
        and value.get("same_state_v12_dagger_sample_count")
        == expected["v12_dagger_sample_count"]
        and value.get("pure_probe_label_sample_count")
        == expected["pure_probe_label_sample_count"]
        and value.get("same_state_pure_probe_label_sample_count")
        == expected["pure_probe_label_sample_count"]
        and value.get("sample_count") == expected["sample_count"]
        and value.get("reset_row_count") == expected["reset_row_count"]
        and value.get("duplicate_sample_count") == 0
        and value.get("all_finite") is True
    )


def fit_gate(
    summary: Mapping[str, Any],
    *,
    stage: str,
) -> dict[str, Any]:
    """Validate one independent V12 actor fit against the frozen corpus design."""

    if stage not in FIT_STAGES:
        raise ValueError(f"unknown V12 fit stage: {stage!r}")
    component_receipts = summary.get("component_receipts")
    pipeline_claim = (
        component_receipts.get("pipeline_claim")
        if isinstance(component_receipts, Mapping)
        else None
    )
    label_bindings_passed, labelled_probe_rows = _probe_label_bindings(
        summary.get("probe_label_receipts"),
        stage=stage,
        pipeline_claim=pipeline_claim,
    )
    if not label_bindings_passed:
        labelled_probe_rows = {
            prior_stage: 1 for prior_stage in FIT_STAGES[: FIT_STAGES.index(stage)]
        }
    expected = expected_fit_counts(stage, labelled_probe_rows=labelled_probe_rows)
    module = summary.get("candidate_module")
    metrics = summary.get("metrics")
    report_checks = summary.get("report_checks")
    fit_artifacts = summary.get("fit_artifacts")
    expected_episode_count = (
        12
        + 2 * len(expected["completed_v12_collection_rounds"])
        + len(expected["pure_probe_label_stages"])
    )
    checks = {
        "schema": summary.get("schema_version") == SCHEMA_VERSION,
        "ungated_status": summary.get("status") == FIT_COMPLETE_STATUS,
        "protocol": summary.get("protocol_id") == PROTOCOL_ID,
        "stage": summary.get("fit_stage") == stage,
        "fit_contract": summary.get("fit_contract_id") == FIT_CONTRACT_ID,
        "fit_exact": summary.get("fit") == FIT,
        "architecture": summary.get("actor_architecture") == ACTOR_ARCHITECTURE,
        "normalization": summary.get("normalization") == BASE_CORPUS_NORMALIZATION,
        "full_mean": summary.get("trainable_scope") == FIT["trainable_scope"],
        "source_h0": summary.get("source_h0_id") == SOURCE_H0_ID
        and tree_record_matches(summary.get("source_h0"), SOURCE_H0_MODULE_PATH)
        and summary["source_h0"].get("tree_sha256") == SOURCE_H0_TREE_SHA256,
        "restart_from_h0": summary.get("initial_checkpoint_id") == SOURCE_H0_ID
        and summary.get("continued_from_previous_candidate") is False,
        "candidate_module": tree_record_matches(module, MODULE_PATHS[stage]),
        "candidate_identity": isinstance(module, Mapping)
        and summary.get("candidate_id")
        == candidate_id(stage, str(module.get("tree_sha256", "")))
        if isinstance(module, Mapping) and _sha256(module.get("tree_sha256"))
        else False,
        "execution_provenance": _fit_component_receipts_match(
            component_receipts, stage=stage
        )
        and summary.get("design_audit_passed") is True
        and summary.get("execution_lock_passed") is True
        and summary.get("pipeline_claimed") is True,
        "teacher_evidence": summary.get("teacher_evidence_id") == TEACHER_EVIDENCE_ID
        and summary.get("teacher_evidence_passed") is True
        and summary.get("teacher_evidence_receipt") == TEACHER_EVIDENCE_ARTIFACT,
        "v11_seed": summary.get("v11_seed_corpus") == V11_P3_CORPUS_ARTIFACT
        and summary.get("v11_seed_corpus_audit_passed") is True,
        "collection_corpora": _fit_collection_corpus_bindings_match(
            summary.get("collection_corpus_receipts"),
            stage=stage,
            pipeline_claim=pipeline_claim,
        ),
        "probe_label_corpora": label_bindings_passed,
        "fit_artifacts": _fit_artifacts_match(fit_artifacts, stage=stage)
        and isinstance(fit_artifacts, Mapping)
        and summary.get("corpus_artifact") == fit_artifacts.get("corpus"),
        "counts": summary.get("fit_counts") == expected
        and summary.get("sample_count") == expected["sample_count"]
        and summary.get("reset_row_count") == expected["reset_row_count"],
        "probe_rows_bound": summary.get("labelled_probe_rows") == labelled_probe_rows,
        "corpus_audit": _fit_corpus_audit_matches(
            summary.get("corpus_audit"), expected
        ),
        "report_checks": isinstance(report_checks, Mapping)
        and set(report_checks) == set(FIT_REPORT_CHECK_NAMES)
        and all(value is True for value in report_checks.values()),
        "optimizer_completion": summary.get("adamw_epochs_run")
        == FIT["adamw"]["epochs"]
        and summary.get("lbfgs_completed") is True
        and summary.get("deterministic_algorithms_enabled") is True,
        "offline_metrics": _metric_triplet(metrics)
        and _metrics_within_thresholds(metrics),
        "episode_mass": summary.get("episode_count") == expected_episode_count
        and summary.get("episode_target_mass")
        == RECOVERY_WEIGHTING["episode_target_mass"]
        and summary.get("normalized_total_sample_mass")
        == expected_episode_count * RECOVERY_WEIGHTING["episode_target_mass"],
        "weight_rules": summary.get("recovery_weighting") == RECOVERY_WEIGHTING
        and summary.get("coverage_weighting") == COVERAGE_WEIGHTING
        and summary.get("shielded_nonreset_raw_weight") == 1.0
        and summary.get("shielded_reset_raw_weight") == 100.0,
        "loss_reduction": summary.get("row_loss") == RECOVERY_WEIGHTING["row_loss"]
        and summary.get("corpus_loss_reduction")
        == RECOVERY_WEIGHTING["corpus_loss_reduction"],
        "clock_and_normalization": summary.get("disabled_clock_column_indices")
        == [0, 1]
        and summary.get("disabled_clock_columns_bit_zero") is True
        and summary.get("disabled_clock_columns_bit_zero_after_save_reload") is True
        and summary.get("normalization_stats_from_base_corpus_only") is True
        and summary.get("normalization_stats_frozen_across_stages") is True
        and summary.get("normalization_folded_into_first_layer") is True
        and summary.get("runtime_normalization_wrapper_present") is False
        and summary.get("prescribed_clock_present") is False,
        "fold_equivalence": summary.get("fold_equivalence_passed") is True,
        "no_anchor_polish": summary.get("anchor_used") is False
        and summary.get("hard_polish_used") is False,
        "p0_reproduces_design_audit": stage != "p0"
        or summary.get("design_audit_reproduction_within_tolerance") is True,
        "actor_only": summary.get("source_checkpoint_scope") == "actor_only_rl_module"
        and summary.get("critic_present") is False
        and summary.get("critic_parameter_count") == 0
        and summary.get("critic_byte_exact") is True
        and summary.get("logstd_byte_exact") is True,
        "integrity": summary.get("duplicate_sample_count") == 0
        and summary.get("all_finite") is True
        and summary.get("source_h0_byte_exact") is True
        and summary.get("target_contract_id") == TARGET_CONTRACT_ID
        and summary.get("event_contract_id") == EVENT_CONTRACT_ID,
        "one_actor_fit_only": summary.get("actor_updates") == 1
        and summary.get("critic_updates") == 0
        and summary.get("ppo_updates") == 0,
        "protected_closed": summary.get("protected_trials_opened") == []
        and summary.get("reserve_trials_opened") == [],
    }
    passed = all(checks.values())
    return {
        "schema_version": SCHEMA_VERSION,
        "status": FIT_PASS_STATUS if passed else FIT_FAIL_STATUS,
        "passed": passed,
        "protocol_id": PROTOCOL_ID,
        "fit_stage": stage,
        "checks": checks,
        "expected_fit_counts": expected,
        "candidate_promoted": False,
        "next_stage": f"PURE_PROBE_{stage.upper()}"
        if passed
        else "STOP_V12_FIT_FAILURE",
    }


def canonical_probe_case(stage: str) -> dict[str, Any]:
    if stage not in FIT_STAGES:
        raise ValueError(f"unknown V12 probe stage: {stage!r}")
    return {
        **copy.deepcopy(PROBE_CASE),
        "candidate_fit_stage": stage,
        "behavior": PROBE_BEHAVIOR,
        "destination": (PROBE_ROOT / stage).as_posix(),
    }


def canonical_collection_case(case_id: str, round_index: int) -> dict[str, Any]:
    matches = [
        case
        for case in COLLECTION_CASES
        if case["case_id"] == case_id and case["round_index"] == round_index
    ]
    if len(matches) != 1:
        raise ValueError(f"unknown V12 collection case/round: {round_index}/{case_id}")
    return copy.deepcopy(matches[0])


def canonical_final_case(case_id: str) -> dict[str, Any]:
    matches = [case for case in FINAL_CASES if case["case_id"] == case_id]
    if len(matches) != 1:
        raise ValueError(f"unknown V12 final case: {case_id!r}")
    return copy.deepcopy(matches[0])


def stage_descriptor(stage_id: str) -> dict[str, Any]:
    if stage_id not in STAGE_IDS:
        raise ValueError(f"unknown V12 pipeline stage: {stage_id!r}")
    if stage_id.startswith("fit_"):
        return {"kind": "fit", "fit_stage": stage_id.removeprefix("fit_")}
    if stage_id.startswith("probe_"):
        stage = stage_id.removeprefix("probe_")
        return {
            "kind": "probe",
            "fit_stage": stage,
            "case": canonical_probe_case(stage),
        }
    if stage_id.startswith("label_"):
        stage = stage_id.removeprefix("label_")
        return {
            "kind": "label",
            "fit_stage": stage,
            "probe_receipt": PROBE_RECEIPT_PATHS[stage],
        }
    if stage_id.startswith("collect_r"):
        prefix, case_id = stage_id.split("__", 1)
        round_index = int(prefix.removeprefix("collect_r"))
        return {
            "kind": "collection",
            "round_index": round_index,
            "case": canonical_collection_case(case_id, round_index),
        }
    if stage_id == "freeze_p3":
        return {"kind": "freeze", "fit_stage": "p3"}
    if stage_id.startswith("final__"):
        case_id = stage_id.removeprefix("final__")
        return {"kind": "final", "case": canonical_final_case(case_id)}
    return {"kind": "finalize"}


def worker_claim_path(stage_id: str) -> PurePosixPath:
    if stage_id not in STAGE_IDS:
        raise ValueError(f"unknown V12 pipeline stage: {stage_id!r}")
    return WORKER_CLAIMS_ROOT / f"{STAGE_IDS.index(stage_id) + 1:02d}_{stage_id}.json"


def stage_receipt_path(stage_id: str) -> PurePosixPath:
    descriptor = stage_descriptor(stage_id)
    kind = descriptor["kind"]
    if kind == "fit":
        return FIT_RECEIPT_PATHS[descriptor["fit_stage"]]
    if kind == "probe":
        return PROBE_RECEIPT_PATHS[descriptor["fit_stage"]]
    if kind == "label":
        return LABEL_RECEIPT_PATHS[descriptor["fit_stage"]]
    if kind == "collection":
        return PurePosixPath(descriptor["case"]["destination"]) / "receipt.json"
    if kind == "freeze":
        return CANDIDATE_FREEZE_PATH
    if kind == "final":
        return PurePosixPath(descriptor["case"]["destination"]) / "receipt.json"
    return FINAL_DEVELOPMENT_RECEIPT_PATH


def _fit_receipt_bindings_match(value: Any) -> bool:
    return (
        isinstance(value, list)
        and len(value) == len(FIT_STAGES)
        and all(
            isinstance(binding, Mapping)
            and set(binding) == {"fit_stage", "receipt", "passed"}
            and binding.get("fit_stage") == stage
            and binding.get("passed") is True
            and artifact_record_matches(
                binding.get("receipt"), FIT_RECEIPT_PATHS[stage]
            )
            for binding, stage in zip(value, FIT_STAGES, strict=True)
        )
    )


def _probe_receipt_bindings_match(value: Any) -> bool:
    return (
        isinstance(value, list)
        and len(value) == len(FIT_STAGES)
        and all(
            isinstance(binding, Mapping)
            and set(binding)
            == {
                "fit_stage",
                "receipt",
                "integrity_passed",
                "autonomy_passed",
                "recoverable_for_data_collection",
            }
            and binding.get("fit_stage") == stage
            and binding.get("integrity_passed") is True
            and type(binding.get("autonomy_passed")) is bool
            and type(binding.get("recoverable_for_data_collection")) is bool
            and (
                (
                    stage == "p3"
                    and binding.get("autonomy_passed") is True
                    and binding.get("recoverable_for_data_collection") is False
                )
                or (
                    stage != "p3"
                    and binding.get("recoverable_for_data_collection")
                    is (not binding.get("autonomy_passed"))
                )
            )
            and artifact_record_matches(
                binding.get("receipt"), PROBE_RECEIPT_PATHS[stage]
            )
            for binding, stage in zip(value, FIT_STAGES, strict=True)
        )
    )


def _label_receipt_bindings_match(value: Any) -> bool:
    return (
        isinstance(value, list)
        and len(value) == len(FIT_STAGES)
        and all(
            isinstance(binding, Mapping)
            and set(binding) == {"fit_stage", "receipt", "passed"}
            and binding.get("fit_stage") == stage
            and binding.get("passed") is True
            and artifact_record_matches(
                binding.get("receipt"), LABEL_RECEIPT_PATHS[stage]
            )
            for binding, stage in zip(value, FIT_STAGES, strict=True)
        )
    )


def _collection_receipt_bindings_match(value: Any) -> bool:
    expected = [
        (round_index, case_id)
        for round_index in (1, 2, 3)
        for case_id in COLLECTION_CASE_IDS
    ]
    return (
        isinstance(value, list)
        and len(value) == len(expected)
        and all(
            isinstance(binding, Mapping)
            and set(binding)
            == {
                "round_index",
                "case_id",
                "receipt",
                "data_passed",
                "latch_independence_passed",
            }
            and binding.get("round_index") == round_index
            and binding.get("case_id") == case_id
            and binding.get("data_passed") is True
            and type(binding.get("latch_independence_passed")) is bool
            and artifact_record_matches(
                binding.get("receipt"),
                stage_receipt_path(f"collect_r{round_index}__{case_id}"),
            )
            for binding, (round_index, case_id) in zip(value, expected, strict=True)
        )
    )


def probe_integrity_gate(summary: Mapping[str, Any], *, stage: str) -> dict[str, Any]:
    """Validate provenance and trace integrity independently of autonomy."""

    if stage not in FIT_STAGES:
        raise ValueError(f"unknown V12 probe stage: {stage!r}")
    expected = canonical_probe_case(stage)
    steps = summary.get("steps")
    step_shape = type(steps) is int and 1 <= steps <= EXPECTED_STEPS
    module = summary.get("candidate_module")
    module_hash = module.get("tree_sha256") if isinstance(module, Mapping) else None
    normal_stop = (
        step_shape
        and steps == EXPECTED_STEPS
        and summary.get("end_reason") == "episode_time_limit"
        and summary.get("terminated") is False
        and summary.get("truncated") is True
        and summary.get("safety_stop_count") == 0
        and _finite_number(summary.get("grf_penetration_max_m"))
        and float(summary["grf_penetration_max_m"]) < PENETRATION_LIMIT_M
    )
    physical_stop = (
        step_shape
        and summary.get("end_reason") == "grf_penetration"
        and summary.get("terminated") is True
        and summary.get("truncated") is False
        and summary.get("safety_stop_count") == 1
        and _finite_number(summary.get("grf_penetration_max_m"))
        and float(summary["grf_penetration_max_m"]) >= PENETRATION_LIMIT_M
    )
    zero_fields = (
        "action_clipped_values",
        "fallback_count",
        "timeout_count",
        "sea_plugin_fallback_count",
        "so_solver_unaccepted_count",
        "hard_invalid_count",
        "invalid_event_count",
        "nonfinite_count",
        "routing_failure_count",
        "step_contract_failure_count",
        "binary_event_failure_count",
        "physical_gate_bypass_count",
    )
    root = PROBE_ROOT / stage
    checks = {
        "schema": summary.get("schema_version") == SCHEMA_VERSION,
        "ungated_status": summary.get("status") == PURE_PROBE_COMPLETE_STATUS,
        "protocol": summary.get("protocol_id") == PROTOCOL_ID,
        "candidate_stage": summary.get("candidate_fit_stage") == stage,
        "case_exact": summary.get("case_id") == expected["case_id"]
        and summary.get("action_selection") == expected["action_selection"]
        and summary.get("episode_start_offset_s") == expected["episode_start_offset_s"]
        and summary.get("action_seed") == expected["action_seed"]
        and summary.get("runtime_seed") == expected["runtime_seed"]
        and summary.get("sigma") == expected["sigma"],
        "behavior": summary.get("behavior") == PROBE_BEHAVIOR,
        "candidate_module": tree_record_matches(module, MODULE_PATHS[stage]),
        "candidate_identity": _sha256(module_hash)
        and summary.get("candidate_id") == candidate_id(stage, module_hash),
        "fit_receipt": artifact_record_matches(
            summary.get("fit_receipt"), FIT_RECEIPT_PATHS[stage]
        )
        and summary.get("fit_receipt_passed") is True
        and summary.get("fit_gate_passed") is True,
        "worker_claim": artifact_record_matches(
            summary.get("worker_claim"), worker_claim_path(f"probe_{stage}")
        ),
        "persisted_artifacts": artifact_record_matches(
            summary.get("run_start"), root / "run_start.json"
        )
        and artifact_record_matches(summary.get("trace"), root / "trace.json")
        and artifact_record_matches(
            summary.get("partial_summary"), root / "partial_summary.json"
        )
        and artifact_record_matches(
            summary.get("replay_payload"), root / "replay_boundaries.npz"
        ),
        "step_shape": step_shape
        and summary.get("trace_step_count") == steps
        and summary.get("control_window_count") == steps * 10
        and summary.get("raw_sensor_sample_count") == steps * 10,
        "closed_stop_semantics": normal_stop or physical_stop,
        "candidate_queries": step_shape
        and summary.get("candidate_mean_query_count") == steps,
        "no_teacher": summary.get("teacher_enabled") is False
        and summary.get("teacher_loaded_during_rollout") is False
        and summary.get("teacher_query_count") == 0
        and summary.get("served_action_teacher_dependency_count") == 0,
        "unblended": summary.get("blending_enabled") is False
        and summary.get("mean_blend_count") == 0,
        "no_latch": summary.get("safety_latch_enabled") is False
        and summary.get("safety_intervention_count") == 0
        and summary.get("safety_latch_activation_count") == 0
        and summary.get("safety_latch_release_count") == 0
        and summary.get("latch_active_at_episode_end") is False,
        "deterministic_noise": summary.get("random_noise_draw_count") == 0
        and step_shape
        and summary.get("single_noise_application_count") == steps
        and summary.get("multiple_noise_application_count") == 0
        and summary.get("noise_application_mismatch_count") == 0,
        "observer_replay_payload": step_shape
        and summary.get("offline_teacher_replay_boundary_count") == steps + 1
        and summary.get("previous_penetration_metadata_count") == steps
        and summary.get("replay_schema") == PROBE_REPLAY_SCHEMA,
        "zero_invalids": all(
            _nonnegative_int(summary.get(field)) and summary[field] == 0
            for field in zero_fields
        ),
        "layout": summary.get("n_actor") == EXPECTED_ACTOR_FEATURES
        and summary.get("n_observation") == EXPECTED_FULL_FEATURES
        and summary.get("observation_dtype") == EXPECTED_DTYPE,
        "v26_active": summary.get("binary_phase_fsm_mode") == "binary_active"
        and summary.get("event_contract_id") == EVENT_CONTRACT_ID
        and summary.get("target_contract_id") == TARGET_CONTRACT_ID,
        "actor_invariants": summary.get("logstd_byte_exact") is True
        and summary.get("disabled_clock_column_indices") == [0, 1]
        and summary.get("disabled_clock_columns_bit_zero") is True
        and summary.get("normalization_folded_into_first_layer") is True
        and summary.get("runtime_normalization_wrapper_present") is False,
        "morphology_zero": _finite_number(summary.get("morphology_weight"))
        and float(summary["morphology_weight"]) == MORPHOLOGY_WEIGHT,
        "zero_updates": summary.get("actor_updates") == 0
        and summary.get("critic_updates") == 0
        and summary.get("ppo_updates") == 0,
        "protected_closed": summary.get("protected_trials_opened") == []
        and summary.get("reserve_trials_opened") == [],
    }
    passed = all(checks.values())
    return {
        "schema_version": SCHEMA_VERSION,
        "status": (
            PROBE_INTEGRITY_PASS_STATUS if passed else PROBE_INTEGRITY_FAIL_STATUS
        ),
        "passed": passed,
        "protocol_id": PROTOCOL_ID,
        "fit_stage": stage,
        "checks": checks,
        "probe_step_count": steps if step_shape else None,
        "candidate_id": summary.get("candidate_id"),
        "candidate_module_tree_sha256": module_hash,
        "trace": copy.deepcopy(summary.get("trace")),
        "replay_payload": copy.deepcopy(summary.get("replay_payload")),
        "replay_schema": copy.deepcopy(summary.get("replay_schema")),
        "next_stage": "EVALUATE_AUTONOMY_GATE"
        if passed
        else "STOP_V12_PROBE_INTEGRITY_FAILURE",
    }


def pure_probe_gate(summary: Mapping[str, Any], *, stage: str) -> dict[str, Any]:
    """Evaluate autonomy only after the independent integrity gate passes."""

    integrity = probe_integrity_gate(summary, stage=stage)
    checks = {
        "integrity": integrity["passed"] is True,
        "full_duration": summary.get("steps") == EXPECTED_STEPS
        and summary.get("control_window_count") == EXPECTED_CONTROL_WINDOWS
        and summary.get("raw_sensor_sample_count") == EXPECTED_RAW_SENSOR_SAMPLES,
        "normal_terminal": summary.get("end_reason") == "episode_time_limit"
        and summary.get("terminated") is False
        and summary.get("truncated") is True
        and summary.get("safety_stop_count") == 0,
        "cycles": _nonnegative_int(summary.get("phase_valid_cycle_count"))
        and summary["phase_valid_cycle_count"] >= MINIMUM_VALID_CYCLES,
        "penetration": _finite_number(summary.get("grf_penetration_max_m"))
        and float(summary["grf_penetration_max_m"]) < PENETRATION_LIMIT_M,
    }
    passed = all(checks.values())
    recoverable = integrity["passed"] is True and not passed and stage != "p3"
    if passed:
        next_stage = f"OBSERVER_LABEL_{stage.upper()}_REQUIRED"
    elif recoverable:
        next_stage = (
            f"OBSERVER_LABEL_{stage.upper()}_REQUIRED_BEFORE_"
            f"SHIELDED_COLLECTION_R{FIT_STAGES.index(stage) + 1}_DATA_ONLY"
        )
    elif integrity["passed"] is not True:
        next_stage = "STOP_V12_PROBE_INTEGRITY_FAILURE"
    else:
        next_stage = "STOP_V12_TERMINAL"
    return {
        "schema_version": SCHEMA_VERSION,
        "status": PURE_PROBE_PASS_STATUS if passed else PURE_PROBE_FAIL_STATUS,
        "passed": passed,
        "integrity_passed": integrity["passed"],
        "recoverable_for_data_collection": recoverable,
        "protocol_id": PROTOCOL_ID,
        "fit_stage": stage,
        "checks": checks,
        "integrity_gate": integrity,
        "probe_step_count": integrity["probe_step_count"],
        "candidate_id": integrity["candidate_id"],
        "candidate_module_tree_sha256": integrity["candidate_module_tree_sha256"],
        "trace": integrity["trace"],
        "replay_payload": integrity["replay_payload"],
        "replay_schema": integrity["replay_schema"],
        "candidate_promoted": False,
        "next_stage": next_stage,
    }


def collection_data_gate(
    summary: Mapping[str, Any], *, round_index: int
) -> dict[str, Any]:
    """Validate one shielded tranche as data, never as autonomy evidence."""

    if type(round_index) is not int or round_index not in (1, 2, 3):
        raise ValueError(f"unknown V12 collection round: {round_index!r}")
    case_id = summary.get("case_id")
    try:
        expected = canonical_collection_case(str(case_id), round_index)
    except ValueError:
        expected = None
    candidate_stage = f"p{round_index - 1}"
    selection = expected.get("action_selection") if expected else None
    expected_draws = EXPECTED_STEPS if selection == "stochastic" else 0
    root = (
        PurePosixPath(expected["destination"])
        if expected is not None
        else COLLECTION_ROOT / "invalid"
    )
    zero_fields = (
        "action_clipped_values",
        "fallback_count",
        "timeout_count",
        "safety_stop_count",
        "sea_plugin_fallback_count",
        "so_solver_unaccepted_count",
        "hard_invalid_count",
        "invalid_event_count",
        "nonfinite_count",
        "routing_failure_count",
        "step_contract_failure_count",
        "binary_event_failure_count",
        "physical_gate_bypass_count",
        "safety_latch_rule_violation_count",
        "alpha_mismatch_count",
        "mean_blend_mismatch_count",
        "noise_application_mismatch_count",
        "multiple_noise_application_count",
    )
    interventions = summary.get("forced_teacher_takeover_count")
    fraction = summary.get("forced_teacher_takeover_fraction")
    streak = summary.get("max_consecutive_takeover_steps")
    latch_metrics = (
        _nonnegative_int(interventions)
        and interventions <= EXPECTED_STEPS
        and _finite_number(fraction)
        and abs(float(fraction) - interventions / EXPECTED_STEPS) <= 1.0e-12
        and _nonnegative_int(streak)
        and streak <= EXPECTED_STEPS
        and type(summary.get("latch_active_at_episode_end")) is bool
    )
    checks = {
        "schema": summary.get("schema_version") == SCHEMA_VERSION,
        "ungated_status": summary.get("status") == COLLECTION_COMPLETE_STATUS,
        "protocol": summary.get("protocol_id") == PROTOCOL_ID,
        "known_case": expected is not None,
        "round_and_case": expected is not None
        and summary.get("round_index") == round_index
        and summary.get("candidate_fit_stage") == candidate_stage
        and summary.get("requested_alpha") == expected["requested_alpha"],
        "condition_exact": expected is not None
        and summary.get("action_selection") == expected["action_selection"]
        and summary.get("episode_start_offset_s") == expected["episode_start_offset_s"]
        and summary.get("action_seed") == expected["action_seed"]
        and summary.get("runtime_seed") == expected["runtime_seed"]
        and summary.get("sigma") == expected["sigma"],
        "candidate_bound": tree_record_matches(
            summary.get("candidate_module"), MODULE_PATHS[candidate_stage]
        )
        and artifact_record_matches(
            summary.get("candidate_fit_receipt"),
            FIT_RECEIPT_PATHS[candidate_stage],
        )
        and summary.get("candidate_fit_gate_passed") is True,
        "probe_label_precondition": artifact_record_matches(
            summary.get("prior_label_receipt"), LABEL_RECEIPT_PATHS[candidate_stage]
        )
        and summary.get("prior_label_gate_passed") is True,
        "labels_complete": summary.get("sample_count") == EXPECTED_STEPS
        and summary.get("teacher_query_count") == EXPECTED_STEPS
        and summary.get("persisted_label_count") == EXPECTED_STEPS
        and summary.get("candidate_mean_query_count") == EXPECTED_STEPS
        and summary.get("same_state_teacher_label_count") == EXPECTED_STEPS
        and summary.get("candidate_selected_before_teacher_count") == EXPECTED_STEPS
        and summary.get("served_action_teacher_dependency_count") == EXPECTED_STEPS,
        "teacher_bound": summary.get("teacher_id") == TEACHER_ID
        and summary.get("teacher_evidence_id") == TEACHER_EVIDENCE_ID
        and summary.get("teacher_evidence_receipt") == TEACHER_EVIDENCE_ARTIFACT
        and tree_record_matches(summary.get("source_h0"), SOURCE_H0_MODULE_PATH)
        and summary["source_h0"].get("tree_sha256") == SOURCE_H0_TREE_SHA256,
        "blend_then_noise": summary.get("mean_blend_count") == EXPECTED_STEPS
        and summary.get("blend_before_noise_count") == EXPECTED_STEPS
        and summary.get("noise_before_blend_count") == 0
        and summary.get("random_noise_draw_count") == expected_draws
        and summary.get("single_noise_application_count") == EXPECTED_STEPS,
        "latch_contract": summary.get("safety_latch_activation_m")
        == SAFETY_LATCH_ACTIVATION_M
        and summary.get("safety_latch_release_m") == SAFETY_LATCH_RELEASE_M
        and summary.get("safety_latch_release_phase") == SAFETY_LATCH_RELEASE_PHASE
        and summary.get("safety_signal_lag_steps") == 1
        and summary.get("safety_intervention_diagnostic_only") is True,
        "latch_metrics_well_formed": latch_metrics,
        "data_only_scope": summary.get("collection_is_data_only") is True
        and summary.get("autonomy_claimed") is False,
        "physical": summary.get("steps") == EXPECTED_STEPS
        and summary.get("control_window_count") == EXPECTED_CONTROL_WINDOWS
        and summary.get("raw_sensor_sample_count") == EXPECTED_RAW_SENSOR_SAMPLES
        and summary.get("end_reason") == "episode_time_limit"
        and summary.get("terminated") is False
        and summary.get("truncated") is True
        and _nonnegative_int(summary.get("phase_valid_cycle_count"))
        and summary["phase_valid_cycle_count"] >= MINIMUM_VALID_CYCLES
        and _finite_number(summary.get("grf_penetration_max_m"))
        and float(summary["grf_penetration_max_m"]) < PENETRATION_LIMIT_M,
        "zero_invalids": all(
            _nonnegative_int(summary.get(field)) and summary[field] == 0
            for field in zero_fields
        ),
        "layout_and_v26": summary.get("n_actor") == EXPECTED_ACTOR_FEATURES
        and summary.get("n_observation") == EXPECTED_FULL_FEATURES
        and summary.get("observation_dtype") == EXPECTED_DTYPE
        and summary.get("binary_phase_fsm_mode") == "binary_active"
        and summary.get("event_contract_id") == EVENT_CONTRACT_ID
        and summary.get("target_contract_id") == TARGET_CONTRACT_ID
        and summary.get("morphology_weight") == MORPHOLOGY_WEIGHT,
        "persisted_artifacts": artifact_record_matches(
            summary.get("run_start"), root / "run_start.json"
        )
        and artifact_record_matches(summary.get("trace"), root / "trace.json")
        and artifact_record_matches(
            summary.get("partial_summary"), root / "partial_summary.json"
        )
        and artifact_record_matches(summary.get("label_corpus"), root / "labels.npz")
        and artifact_record_matches(
            summary.get("worker_claim"),
            worker_claim_path(f"collect_r{round_index}__{case_id}"),
        ),
        "zero_updates": summary.get("actor_updates") == 0
        and summary.get("critic_updates") == 0
        and summary.get("ppo_updates") == 0,
        "protected_closed": summary.get("protected_trials_opened") == []
        and summary.get("reserve_trials_opened") == [],
    }
    passed = all(checks.values())
    return {
        "schema_version": SCHEMA_VERSION,
        "status": COLLECTION_PASS_STATUS if passed else COLLECTION_FAIL_STATUS,
        "passed": passed,
        "protocol_id": PROTOCOL_ID,
        "round_index": round_index,
        "case_id": case_id,
        "checks": checks,
        "collection_is_data_only": True,
        "autonomy_demonstrated": False,
        "collection_data_reusable": passed,
        "next_stage": "LATCH_DEPENDENCE_AUDIT"
        if passed
        else "STOP_V12_COLLECTION_INTEGRITY_FAILURE",
    }


def latch_dependence_gate(
    summary: Mapping[str, Any], *, collection_data_passed: bool
) -> dict[str, Any]:
    """Classify collection dependence without rejecting valid labelled data."""

    if type(collection_data_passed) is not bool:
        raise TypeError("collection_data_passed must be bool")

    steps = summary.get("steps")
    interventions = summary.get("forced_teacher_takeover_count")
    streak = summary.get("max_consecutive_takeover_steps")
    fraction = summary.get("forced_teacher_takeover_fraction")
    shape_ok = (
        steps == EXPECTED_STEPS
        and _nonnegative_int(interventions)
        and interventions <= EXPECTED_STEPS
        and _nonnegative_int(streak)
        and streak <= EXPECTED_STEPS
        and _finite_number(fraction)
        and abs(float(fraction) - interventions / EXPECTED_STEPS) <= 1.0e-12
    )
    checks = {
        "collection_data_integrity": collection_data_passed,
        "metric_shape": shape_ok,
        "zero_forced_takeover": shape_ok and interventions == 0,
        "zero_takeover_fraction": shape_ok and float(fraction) == 0.0,
        "zero_takeover_streak": shape_ok and streak == 0,
        "latch_inactive_at_end": summary.get("latch_active_at_episode_end") is False,
        "data_only_scope": summary.get("collection_is_data_only") is True
        and summary.get("autonomy_claimed") is False,
    }
    passed = all(checks.values())
    data_reusable = collection_data_passed and shape_ok and checks["data_only_scope"]
    return {
        "schema_version": SCHEMA_VERSION,
        "status": (
            LATCH_INDEPENDENCE_PASS_STATUS if passed else LATCH_INDEPENDENCE_FAIL_STATUS
        ),
        "passed": passed,
        "protocol_id": PROTOCOL_ID,
        "checks": checks,
        "collection_data_rejected": not data_reusable,
        "collection_data_reusable": data_reusable,
        "autonomy_demonstrated": False,
        "next_stage": (
            "NEXT_PREREGISTERED_STAGE"
            if passed
            else (
                "SHIELDED_DATA_MAY_BE_USED;_NO_AUTONOMY_CLAIM"
                if data_reusable
                else "STOP_V12_COLLECTION_INTEGRITY_FAILURE"
            )
        ),
    }


def observer_label_gate(
    summary: Mapping[str, Any],
    *,
    stage: str,
    probe_evidence: Mapping[str, Any],
) -> dict[str, Any]:
    """Validate labels bound to one already-closed, integrity-passing probe."""

    if stage not in FIT_STAGES:
        raise ValueError(f"unknown V12 observer-label stage: {stage!r}")
    if not isinstance(probe_evidence, Mapping):
        raise TypeError("probe_evidence must be a mapping")
    rows = summary.get("labelled_row_count")
    probe_steps = probe_evidence.get("probe_step_count")
    probe_passed = probe_evidence.get("passed")
    probe_trace = probe_evidence.get("trace")
    probe_replay_payload = probe_evidence.get("replay_payload")
    probe_receipt = probe_evidence.get("receipt")
    probe_gate_artifact = probe_evidence.get("gate_artifact")
    evidence_shape = (
        probe_evidence.get("protocol_id") == PROTOCOL_ID
        and probe_evidence.get("fit_stage") == stage
        and probe_evidence.get("integrity_passed") is True
        and type(probe_passed) is bool
        and type(probe_steps) is int
        and 1 <= probe_steps <= EXPECTED_STEPS
        and (probe_passed is False or probe_steps == EXPECTED_STEPS)
        and (
            probe_passed is True
            or (
                stage != "p3"
                and probe_evidence.get("recoverable_for_data_collection") is True
            )
        )
        and isinstance(probe_evidence.get("candidate_id"), str)
        and _sha256(probe_evidence.get("candidate_module_tree_sha256"))
        and probe_evidence.get("replay_schema") == PROBE_REPLAY_SCHEMA
        and artifact_record_matches(probe_trace, PROBE_ROOT / stage / "trace.json")
        and artifact_record_matches(
            probe_replay_payload, PROBE_ROOT / stage / "replay_boundaries.npz"
        )
        and artifact_record_matches(probe_receipt, PROBE_RECEIPT_PATHS[stage])
        and artifact_record_matches(
            probe_gate_artifact, PROBE_ROOT / stage / "gate.json"
        )
    )
    checks = {
        "schema": summary.get("schema_version") == SCHEMA_VERSION,
        "protocol": summary.get("protocol_id") == PROTOCOL_ID,
        "candidate_stage": summary.get("candidate_fit_stage") == stage,
        "probe_evidence_shape": evidence_shape,
        "probe_outcome_bound": evidence_shape
        and summary.get("probe_passed") is probe_passed
        and summary.get("probe_integrity_passed") is True
        and summary.get("probe_step_count") == probe_steps
        and summary.get("probe_candidate_id") == probe_evidence.get("candidate_id")
        and summary.get("probe_candidate_module_tree_sha256")
        == probe_evidence.get("candidate_module_tree_sha256")
        and summary.get("probe_trace") == probe_trace
        and summary.get("probe_replay_payload") == probe_replay_payload
        and summary.get("probe_receipt") == probe_receipt
        and summary.get("probe_gate_artifact") == probe_gate_artifact,
        "probe_replay_schema_bound": evidence_shape
        and summary.get("probe_replay_schema") == PROBE_REPLAY_SCHEMA,
        "row_count": evidence_shape and type(rows) is int and rows == probe_steps,
        "same_state_labels": type(rows) is int
        and summary.get("teacher_query_count") == rows
        and summary.get("same_state_teacher_label_count") == rows
        and summary.get("persisted_label_count") == rows,
        "complete_replay": type(rows) is int
        and summary.get("replayed_boundary_count") == rows + 1
        and summary.get("replay_payload_missing_count") == 0,
        "coherent_teacher_view": type(rows) is int
        and summary.get("teacher_view_changes_only_10_24_count") == rows
        and summary.get("invariant_columns_byte_exact_count") == rows,
        "coverage_metadata": type(rows) is int
        and summary.get("coverage_distance_count") == rows
        and summary.get("coverage_reference_observations_sha256")
        == COVERAGE_WEIGHTING["reference_observations_sha256"]
        and summary.get("coverage_normalization_mean_sha256")
        == COVERAGE_WEIGHTING["normalization_mean_sha256"]
        and summary.get("coverage_normalization_std_sha256")
        == COVERAGE_WEIGHTING["normalization_std_sha256"]
        and summary.get("coverage_reference_features_sha256")
        == COVERAGE_WEIGHTING["normalized_feature_matrix_sha256"]
        and summary.get("coverage_loo_p95") == COVERAGE_WEIGHTING["loo_p95"]
        and summary.get("coverage_new_row_query") == COVERAGE_WEIGHTING["new_row_query"]
        and _nonnegative_int(summary.get("coverage_ood_row_count"))
        and summary["coverage_ood_row_count"] <= rows,
        "weights_complete": type(rows) is int
        and summary.get("previous_penetration_metadata_count") == rows
        and summary.get("raw_sample_weight_count") == rows
        and summary.get("normalized_sample_weight_count") == rows
        and summary.get("reset_row_count") == 1
        and summary.get("recovery_weighting") == RECOVERY_WEIGHTING
        and summary.get("normalized_episode_mass")
        == RECOVERY_WEIGHTING["episode_target_mass"],
        "published_corpus": artifact_record_matches(
            summary.get("label_corpus"), LABEL_CORPUS_PATHS[stage]
        )
        and artifact_record_matches(
            summary.get("worker_claim"), worker_claim_path(f"label_{stage}")
        ),
        "teacher_bound": summary.get("teacher_id") == TEACHER_ID
        and summary.get("teacher_evidence_id") == TEACHER_EVIDENCE_ID
        and summary.get("teacher_evidence_receipt") == TEACHER_EVIDENCE_ARTIFACT
        and tree_record_matches(summary.get("source_h0"), SOURCE_H0_MODULE_PATH)
        and summary["source_h0"].get("tree_sha256") == SOURCE_H0_TREE_SHA256,
        "observer_only": summary.get("environment_reset_calls") == 0
        and summary.get("environment_step_calls") == 0
        and summary.get("action_served_count") == 0
        and summary.get("teacher_loaded_after_probe_closed") is True,
        "zero_updates": summary.get("actor_updates") == 0
        and summary.get("critic_updates") == 0
        and summary.get("ppo_updates") == 0,
        "protected_closed": summary.get("protected_trials_opened") == [],
        "reserve_closed": summary.get("reserve_trials_opened") == [],
    }
    passed = all(checks.values())
    if stage == "p3":
        next_stage = "FREEZE_P3" if passed else "STOP_V12_TERMINAL"
    else:
        next_stage = (
            f"SHIELDED_COLLECTION_R{FIT_STAGES.index(stage) + 1}_DATA_ONLY"
            if passed
            else "STOP_V12_LABEL_INTEGRITY_FAILURE"
        )
    return {
        "schema_version": SCHEMA_VERSION,
        "status": (
            OBSERVER_LABEL_PASS_STATUS if passed else OBSERVER_LABEL_FAIL_STATUS
        ),
        "passed": passed,
        "protocol_id": PROTOCOL_ID,
        "fit_stage": stage,
        "checks": checks,
        "observer_only": checks["observer_only"],
        "candidate_promoted": False,
        "next_stage": next_stage,
    }


def candidate_freeze_gate(summary: Mapping[str, Any]) -> dict[str, Any]:
    """Freeze P3 only after fit, probe, label, and collection closure."""

    module = summary.get("candidate_module")
    module_hash = module.get("tree_sha256") if isinstance(module, Mapping) else None
    checks = {
        "schema": summary.get("schema_version") == SCHEMA_VERSION,
        "protocol": summary.get("protocol_id") == PROTOCOL_ID,
        "fit_contract": summary.get("fit_contract_id") == FIT_CONTRACT_ID,
        "stage": summary.get("candidate_fit_stage") == "p3",
        "candidate_module": tree_record_matches(module, MODULE_PATHS["p3"]),
        "candidate_identity": _sha256(module_hash)
        and summary.get("candidate_id") == candidate_id("p3", module_hash),
        "fit_receipts": _fit_receipt_bindings_match(summary.get("fit_receipts")),
        "probe_receipts": _probe_receipt_bindings_match(summary.get("probe_receipts")),
        "label_receipts": _label_receipt_bindings_match(summary.get("label_receipts")),
        "collection_receipts": _collection_receipt_bindings_match(
            summary.get("collection_receipts")
        ),
        "exclusive_claims": artifact_record_matches(
            summary.get("pipeline_claim"), PIPELINE_CLAIM_PATH
        )
        and artifact_record_matches(
            summary.get("worker_claim"), worker_claim_path("freeze_p3")
        ),
        "p3_gates": summary.get("p3_fit_passed") is True
        and summary.get("p3_probe_integrity_passed") is True
        and summary.get("p3_probe_autonomy_passed") is True
        and summary.get("p3_label_passed") is True,
        "source_h0": tree_record_matches(
            summary.get("source_h0"), SOURCE_H0_MODULE_PATH
        )
        and summary["source_h0"].get("tree_sha256") == SOURCE_H0_TREE_SHA256,
        "design_and_execution_locks": artifact_record_matches(
            summary.get("design_audit_receipt"), DESIGN_AUDIT_RECEIPT_PATH
        )
        and summary.get("design_audit_passed") is True
        and artifact_record_matches(summary.get("execution_lock"), EXECUTION_LOCK_PATH),
        "four_independent_fits": summary.get("fit_actor_update_count") == 4
        and summary.get("every_fit_restarted_from_h0") is True,
        "actor_only": summary.get("source_checkpoint_scope") == "actor_only_rl_module"
        and summary.get("critic_present") is False
        and summary.get("critic_parameter_count") == 0
        and summary.get("logstd_byte_exact") is True,
        "normalization": summary.get("normalization_folded_into_first_layer") is True
        and summary.get("runtime_normalization_wrapper_present") is False
        and summary.get("disabled_clock_columns_0_1_bit_zero") is True,
        "frozen_not_promoted": summary.get("candidate_frozen") is True
        and summary.get("runtime_promoted") is False,
        "zero_non_actor_updates": summary.get("actor_updates") == 0
        and summary.get("critic_updates") == 0
        and summary.get("ppo_updates") == 0,
        "protected_closed": summary.get("protected_trials_opened") == []
        and summary.get("reserve_trials_opened") == [],
        "no_retry_sweep_rescue": summary.get("retry_authorized") is False
        and summary.get("sweep_authorized") is False
        and summary.get("rescue_authorized") is False,
    }
    passed = all(checks.values())
    return {
        "schema_version": SCHEMA_VERSION,
        "status": (
            CANDIDATE_FREEZE_PASS_STATUS if passed else CANDIDATE_FREEZE_FAIL_STATUS
        ),
        "passed": passed,
        "protocol_id": PROTOCOL_ID,
        "checks": checks,
        "candidate_frozen": passed,
        "runtime_promoted": False,
        "next_stage": (
            "FINAL_DEVELOPMENT_ROLLOUTS" if passed else "STOP_V12_FREEZE_FAILURE"
        ),
    }


def final_rollout_gate(summary: Mapping[str, Any]) -> dict[str, Any]:
    """Validate one of six preregistered pure P3 development rollouts."""

    case_id = summary.get("case_id")
    try:
        expected = canonical_final_case(str(case_id))
    except ValueError:
        expected = None
    root = (
        PurePosixPath(expected["destination"])
        if expected is not None
        else FINAL_ROOT / "invalid"
    )
    module = summary.get("candidate_module")
    module_hash = module.get("tree_sha256") if isinstance(module, Mapping) else None
    selection = expected.get("action_selection") if expected else None
    expected_draws = EXPECTED_STEPS if selection == "stochastic" else 0
    zero_fields = (
        "action_clipped_values",
        "fallback_count",
        "timeout_count",
        "safety_stop_count",
        "sea_plugin_fallback_count",
        "so_solver_unaccepted_count",
        "hard_invalid_count",
        "invalid_event_count",
        "nonfinite_count",
        "routing_failure_count",
        "step_contract_failure_count",
        "binary_event_failure_count",
        "physical_gate_bypass_count",
    )
    checks = {
        "schema": summary.get("schema_version") == SCHEMA_VERSION,
        "ungated_status": summary.get("status") == FINAL_ROLLOUT_COMPLETE_STATUS,
        "protocol": summary.get("protocol_id") == PROTOCOL_ID,
        "known_case": expected is not None,
        "condition_exact": expected is not None
        and summary.get("action_selection") == expected["action_selection"]
        and summary.get("episode_start_offset_s") == expected["episode_start_offset_s"]
        and summary.get("action_seed") == expected["action_seed"]
        and summary.get("runtime_seed") == expected["runtime_seed"]
        and summary.get("sigma") == expected["sigma"],
        "candidate": tree_record_matches(module, MODULE_PATHS["p3"])
        and _sha256(module_hash)
        and summary.get("candidate_id") == candidate_id("p3", module_hash),
        "candidate_freeze": artifact_record_matches(
            summary.get("candidate_freeze"), CANDIDATE_FREEZE_PATH
        )
        and summary.get("candidate_freeze_passed") is True,
        "pure_control": summary.get("teacher_enabled") is False
        and summary.get("teacher_loaded_during_rollout") is False
        and summary.get("teacher_query_count") == 0
        and summary.get("served_action_teacher_dependency_count") == 0
        and summary.get("blending_enabled") is False
        and summary.get("mean_blend_count") == 0
        and summary.get("safety_latch_enabled") is False
        and summary.get("safety_intervention_count") == 0,
        "noise": summary.get("random_noise_draw_count") == expected_draws
        and summary.get("single_noise_application_count") == EXPECTED_STEPS
        and summary.get("multiple_noise_application_count") == 0
        and summary.get("noise_application_mismatch_count") == 0,
        "physical": summary.get("steps") == EXPECTED_STEPS
        and summary.get("control_window_count") == EXPECTED_CONTROL_WINDOWS
        and summary.get("raw_sensor_sample_count") == EXPECTED_RAW_SENSOR_SAMPLES
        and summary.get("end_reason") == "episode_time_limit"
        and summary.get("terminated") is False
        and summary.get("truncated") is True
        and _nonnegative_int(summary.get("phase_valid_cycle_count"))
        and summary["phase_valid_cycle_count"] >= MINIMUM_VALID_CYCLES
        and _finite_number(summary.get("grf_penetration_max_m"))
        and float(summary["grf_penetration_max_m"]) < PENETRATION_LIMIT_M,
        "zero_invalids": all(
            _nonnegative_int(summary.get(field)) and summary[field] == 0
            for field in zero_fields
        ),
        "layout_and_v26": summary.get("n_actor") == EXPECTED_ACTOR_FEATURES
        and summary.get("n_observation") == EXPECTED_FULL_FEATURES
        and summary.get("observation_dtype") == EXPECTED_DTYPE
        and summary.get("binary_phase_fsm_mode") == "binary_active"
        and summary.get("event_contract_id") == EVENT_CONTRACT_ID
        and summary.get("target_contract_id") == TARGET_CONTRACT_ID
        and summary.get("morphology_weight") == MORPHOLOGY_WEIGHT,
        "persisted_artifacts": artifact_record_matches(
            summary.get("run_start"), root / "run_start.json"
        )
        and artifact_record_matches(summary.get("trace"), root / "trace.json")
        and artifact_record_matches(
            summary.get("partial_summary"), root / "partial_summary.json"
        )
        and expected is not None
        and artifact_record_matches(
            summary.get("worker_claim"), worker_claim_path(f"final__{case_id}")
        ),
        "zero_updates": summary.get("actor_updates") == 0
        and summary.get("critic_updates") == 0
        and summary.get("ppo_updates") == 0,
        "protected_closed": summary.get("protected_trials_opened") == []
        and summary.get("reserve_trials_opened") == [],
    }
    passed = all(checks.values())
    return {
        "schema_version": SCHEMA_VERSION,
        "status": FINAL_ROLLOUT_PASS_STATUS if passed else FINAL_ROLLOUT_FAIL_STATUS,
        "passed": passed,
        "protocol_id": PROTOCOL_ID,
        "case_id": case_id,
        "checks": checks,
        "development_only": True,
        "runtime_promoted": False,
        "next_stage": "NEXT_FINAL_CASE" if passed else "STOP_V12_TERMINAL",
    }


def final_development_gate(summary: Mapping[str, Any]) -> dict[str, Any]:
    """Aggregate exactly six passing development rollouts without promotion."""

    bindings = summary.get("rollout_bindings")
    bindings_pass = isinstance(bindings, list) and len(bindings) == len(FINAL_CASES)
    if bindings_pass:
        for binding, case in zip(bindings, FINAL_CASES, strict=True):
            destination = PurePosixPath(case["destination"])
            if (
                not isinstance(binding, Mapping)
                or set(binding) != {"case_id", "passed", "receipt", "gate", "summary"}
                or binding.get("case_id") != case["case_id"]
                or binding.get("passed") is not True
                or not artifact_record_matches(
                    binding.get("receipt"), destination / "receipt.json"
                )
                or not artifact_record_matches(
                    binding.get("gate"), destination / "gate.json"
                )
                or not artifact_record_matches(
                    binding.get("summary"), destination / "summary.json"
                )
            ):
                bindings_pass = False
                break
    module = summary.get("candidate_module")
    module_hash = module.get("tree_sha256") if isinstance(module, Mapping) else None
    checks = {
        "schema": summary.get("schema_version") == SCHEMA_VERSION,
        "ungated_status": summary.get("status") == FINAL_DEVELOPMENT_COMPLETE_STATUS,
        "protocol": summary.get("protocol_id") == PROTOCOL_ID,
        "candidate": tree_record_matches(module, MODULE_PATHS["p3"])
        and _sha256(module_hash)
        and summary.get("candidate_id") == candidate_id("p3", module_hash),
        "candidate_freeze": artifact_record_matches(
            summary.get("candidate_freeze"), CANDIDATE_FREEZE_PATH
        )
        and summary.get("candidate_freeze_passed") is True,
        "exclusive_claims": artifact_record_matches(
            summary.get("pipeline_claim"), PIPELINE_CLAIM_PATH
        )
        and artifact_record_matches(
            summary.get("worker_claim"), worker_claim_path("finalize_development")
        ),
        "six_exact_passes": bindings_pass
        and summary.get("rollout_count") == len(FINAL_CASES)
        and summary.get("passing_rollout_count") == len(FINAL_CASES),
        "zero_updates": summary.get("actor_updates") == 0
        and summary.get("critic_updates") == 0
        and summary.get("ppo_updates") == 0,
        "protected_closed": summary.get("protected_trials_opened") == []
        and summary.get("reserve_trials_opened") == [],
        "development_only": summary.get("development_only") is True
        and summary.get("runtime_promoted") is False
        and summary.get("qualification_required") is True,
        "no_retry_sweep_rescue": summary.get("retry_authorized") is False
        and summary.get("sweep_authorized") is False
        and summary.get("rescue_authorized") is False,
    }
    passed = all(checks.values())
    return {
        "schema_version": SCHEMA_VERSION,
        "status": (
            FINAL_DEVELOPMENT_PASS_STATUS if passed else FINAL_DEVELOPMENT_FAIL_STATUS
        ),
        "passed": passed,
        "protocol_id": PROTOCOL_ID,
        "checks": checks,
        "runtime_promoted": False,
        "next_stage": (
            "WAIT_INDEPENDENT_QUALIFICATION_PROTOCOL" if passed else "STOP_V12_TERMINAL"
        ),
    }


def declared_mutation_paths() -> dict[str, PurePosixPath]:
    """Enumerate every frozen root plus every exact claim/receipt destination."""

    paths: dict[str, PurePosixPath] = {
        "protocol_freeze": PROTOCOL_FREEZE_PATH,
        "execution_lock": EXECUTION_LOCK_PATH,
        "design_audit": DESIGN_AUDIT_RECEIPT_PATH,
        "run_root": RUN_ROOT,
        "fit_root": FIT_ROOT,
        "probe_root": PROBE_ROOT,
        "label_root": LABEL_ROOT,
        "collection_root": COLLECTION_ROOT,
        "final_root": FINAL_ROOT,
        "pipeline_claim": PIPELINE_CLAIM_PATH,
        "pipeline_ledger": PIPELINE_LEDGER_PATH,
        "worker_claims_root": WORKER_CLAIMS_ROOT,
    }
    for stage in FIT_STAGES:
        paths[f"fit_root_{stage}"] = FIT_ROOTS[stage]
        paths[f"module_{stage}"] = MODULE_PATHS[stage]
        paths[f"fit_corpus_{stage}"] = FIT_CORPUS_PATHS[stage]
        paths[f"fit_history_{stage}"] = FIT_ROOTS[stage] / "adaptation_history.json"
        paths[f"fit_report_{stage}"] = FIT_ROOTS[stage] / "adaptation_report.json"
        paths[f"fit_summary_{stage}"] = FIT_ROOTS[stage] / "summary.json"
        paths[f"fit_gate_{stage}"] = FIT_ROOTS[stage] / "gate.json"
        paths[f"probe_root_{stage}"] = PROBE_ROOT / stage
        paths[f"probe_steps_root_{stage}"] = PROBE_ROOT / stage / "steps"
        paths[f"probe_run_start_{stage}"] = PROBE_ROOT / stage / "run_start.json"
        paths[f"probe_trace_{stage}"] = PROBE_ROOT / stage / "trace.json"
        paths[f"probe_partial_summary_{stage}"] = (
            PROBE_ROOT / stage / "partial_summary.json"
        )
        paths[f"probe_summary_{stage}"] = PROBE_ROOT / stage / "summary.json"
        paths[f"probe_gate_{stage}"] = PROBE_ROOT / stage / "gate.json"
        paths[f"probe_replay_{stage}"] = PROBE_ROOT / stage / "replay_boundaries.npz"
        paths[f"label_root_{stage}"] = LABEL_ROOT / stage
        paths[f"label_corpus_{stage}"] = LABEL_CORPUS_PATHS[stage]
        paths[f"label_summary_{stage}"] = LABEL_ROOT / stage / "summary.json"
        paths[f"label_gate_{stage}"] = LABEL_ROOT / stage / "gate.json"
    for case in COLLECTION_CASES:
        prefix = f"collect_r{case['round_index']}_{case['case_id']}"
        root = PurePosixPath(case["destination"])
        paths[f"{prefix}_root"] = root
        paths[f"{prefix}_steps_root"] = root / "steps"
        paths[f"{prefix}_run_start"] = root / "run_start.json"
        paths[f"{prefix}_trace"] = root / "trace.json"
        paths[f"{prefix}_partial_summary"] = root / "partial_summary.json"
        paths[f"{prefix}_summary"] = root / "summary.json"
        paths[f"{prefix}_gate"] = root / "gate.json"
        paths[f"{prefix}_labels"] = root / "labels.npz"
    for case in FINAL_CASES:
        prefix = f"final_{case['case_id']}"
        root = PurePosixPath(case["destination"])
        paths[f"{prefix}_root"] = root
        paths[f"{prefix}_steps_root"] = root / "steps"
        paths[f"{prefix}_run_start"] = root / "run_start.json"
        paths[f"{prefix}_trace"] = root / "trace.json"
        paths[f"{prefix}_partial_summary"] = root / "partial_summary.json"
        paths[f"{prefix}_summary"] = root / "summary.json"
        paths[f"{prefix}_gate"] = root / "gate.json"
    paths["final_development_summary"] = FINAL_ROOT / "summary.json"
    paths["final_development_gate"] = FINAL_ROOT / "gate.json"
    for index, stage_id in enumerate(STAGE_IDS, start=1):
        paths[f"worker_claim_{index:02d}_{stage_id}"] = worker_claim_path(stage_id)
        paths[f"stage_receipt_{index:02d}_{stage_id}"] = stage_receipt_path(stage_id)
    return paths


__all__ = [
    "AUTHORITY",
    "COLLECTION_CASE_IDS",
    "COVERAGE_WEIGHTING",
    "DESIGN_AUDIT_RECEIPT_PATH",
    "EXECUTION_LOCK_PATH",
    "FINAL_CASE_IDS",
    "FIT",
    "FIT_STAGES",
    "INPUT_RELATIVE_PATHS",
    "LATCH_INDEPENDENCE",
    "PIPELINE_ID",
    "PROBE_BEHAVIOR",
    "PROBE_REPLAY_SCHEMA",
    "PROTOCOL_FREEZE_PATH",
    "PROTOCOL_ID",
    "RECOVERY_WEIGHTING",
    "RUN_ROOT",
    "SCHEMA_VERSION",
    "SOURCE_RELATIVE_PATHS",
    "STAGE_IDS",
    "V11_COLLECTION_INPUTS",
    "canonical_collection_case",
    "canonical_final_case",
    "canonical_probe_case",
    "candidate_freeze_gate",
    "candidate_id",
    "collection_data_gate",
    "declared_mutation_paths",
    "expected_fit_counts",
    "final_development_gate",
    "final_rollout_gate",
    "fit_gate",
    "latch_dependence_gate",
    "normalize_episode_sample_weights",
    "observer_label_gate",
    "probe_integrity_gate",
    "pure_probe_gate",
    "recovery_row_class",
    "recovery_sample_weight",
    "shielded_sample_weight",
    "stage_descriptor",
    "stage_receipt_path",
    "weighted_mean_row_loss",
    "worker_claim_path",
]
