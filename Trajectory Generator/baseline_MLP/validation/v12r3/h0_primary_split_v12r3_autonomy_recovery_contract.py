# ruff: noqa: F821
"""Additive V12R3 correction of the frozen V12 autonomy/recovery contract.

V12R3 preserves the complete V12 scientific design and immutable V12 evidence,
keeps the terminal V12R1 and V12R2 attempts as immutable failure evidence, and
closes the two execution gaps exposed by the sole V12R2 run:

* the forensic publication compares ``run_start`` before and after finalizing
  a rollout even though the shared writer does not return that record;
* a physically stopped pure-probe prefix is judged on detector integrity at
  the observed prefix length, independently from the full-horizon event gate.

The implementation deliberately derives its unchanged gates from the immutable
V12 module.  Functions are rebound to this module's globals, so every inherited
gate consumes the V12R3 identifiers and paths without mutating the V12 module.
This module performs no I/O, fitting, policy inference, or environment access.
"""

from __future__ import annotations

import copy
import sys
import types
from pathlib import Path, PurePosixPath
from typing import Any, Mapping, Sequence

_PARENT_VALIDATION = Path(__file__).resolve().parent.parent
if str(_PARENT_VALIDATION) not in sys.path:
    sys.path.insert(0, str(_PARENT_VALIDATION))

try:
    from validation import h0_primary_split_v12_autonomy_recovery_contract as _v12
except (ImportError, ModuleNotFoundError):  # Flat import when this dir is on sys.path.
    import h0_primary_split_v12_autonomy_recovery_contract as _v12


def _rebind_function(value: types.FunctionType) -> types.FunctionType:
    """Rebind a V12 function to R3 globals without rewriting its bytecode.

    Lower-case ``v12_*`` count fields are a stable wire schema shared by the
    corpus producer and inherited gates.  Rewriting code constants broke that
    invariant in V12R1 because strings nested in tuples were transformed
    differently from direct string constants.
    """

    function = types.FunctionType(
        value.__code__,
        globals(),
        name=value.__name__,
        argdefs=value.__defaults__,
        closure=value.__closure__,
    )
    function.__kwdefaults__ = copy.deepcopy(value.__kwdefaults__)
    function.__annotations__ = copy.deepcopy(value.__annotations__)
    function.__doc__ = value.__doc__
    return function


# Import the frozen design namespace without modifying it.  Mutable values that
# are changed below are always replaced by fresh copies.
for _name, _value in vars(_v12).items():
    if not _name.startswith("__") and (
        not isinstance(_value, types.FunctionType) or _value.__module__ != _v12.__name__
    ):
        globals()[_name] = _value


SCHEMA_VERSION = 123
REVISION = "2026-08-09"
PROTOCOL_ID = "AB06_H0_PRIMARY_SPLIT_V12R3_V26_AUTONOMY_RECOVERY"
PIPELINE_ID = "H0_V12R3_V26_RECOVERY_WEIGHTED_PURE_PROBE_DAGGER"
FIT_CONTRACT_ID = "h0_primary_split_v12r3_recovery_weighted_full_mean_v1"
DESIGN_AUDIT_ID = "H0_V12R3_P0_RECOVERY_WEIGHTED_FEASIBILITY_V1"
AUTHORITY_TEXT = "esegui i punti 1-6"
TRAINING_READINESS_SCOPE = "V12R3_CORRECTIVE_AUTONOMY_RECOVERY_READINESS"
EXECUTION_AUTHORITY_SCOPE = "V12R3_CORRECTIVE_ONE_SHOT_EXECUTION"

V12R3_VALIDATION_ROOT = PurePosixPath(
    "Trajectory Generator/baseline_MLP/validation/v12r3"
)
V12_VALIDATION_ROOT = V12R3_VALIDATION_ROOT
RUN_ROOT = V12R3_VALIDATION_ROOT / ("h0_v12r3_run_20260809")
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

PROTOCOL_FREEZE_PATH = V12R3_VALIDATION_ROOT / (
    "h0_primary_split_v12r3_autonomy_recovery_protocol_freeze.json"
)
EXECUTION_LOCK_PATH = V12R3_VALIDATION_ROOT / (
    "h0_primary_split_v12r3_autonomy_recovery_execution_lock.json"
)
DESIGN_AUDIT_RECEIPT_PATH = V12R3_VALIDATION_ROOT / (
    "h0_primary_split_v12r3_autonomy_recovery_design_audit.json"
)
V12_PROTOCOL_FREEZE_PATH = _v12.PROTOCOL_FREEZE_PATH
V12_PROTOCOL_FREEZE_SHA256 = (
    "c9d60bd386adc605da3f2187f80960f00f213d3d88bf819f5af12f6106367b42"
)
V12_PROTOCOL_FREEZE_SIZE_BYTES = 222_439
V11_EXECUTION_LOCK_ARTIFACT = {
    "path": "validation/h0_primary_split_v11_weighted_full_mean_execution_lock.json",
    "sha256": "deba2d6e69e97f3b6814e8a7dcbd79cbef6f1ec7f938c94b6d6604c87e4404f1",
    "size_bytes": 47_249,
}

# Exact terminal V12R1 lineage.  These records are failure evidence only: no
# checkpoint, corpus, or partial fit below may be promoted into V12R3.
V12R1_LINEAGE_ARTIFACTS = {
    "v12r1_protocol_freeze": {
        "path": (
            "Trajectory Generator/baseline_MLP/validation/"
            "h0_primary_split_v12r1_autonomy_recovery_protocol_freeze.json"
        ),
        "sha256": "05137af3d2dc53485272079e915f3e79439c8ea07e00870b231376f3354ae35b",
        "size_bytes": 227_376,
    },
    "v12r1_design_audit": {
        "path": (
            "Trajectory Generator/baseline_MLP/validation/"
            "h0_primary_split_v12r1_autonomy_recovery_design_audit.json"
        ),
        "sha256": "1eed1158a6eabc20bec81dce6632fe668d5e12f4fa9eeb02af5c8b949e5bba37",
        "size_bytes": 24_528,
    },
    "v12r1_execution_lock": {
        "path": (
            "Trajectory Generator/baseline_MLP/validation/"
            "h0_primary_split_v12r1_autonomy_recovery_execution_lock.json"
        ),
        "sha256": "bf92499802c8d9e9f92e33ec53e62ab6da47a1a638304bedc03fa86470bbe8f0",
        "size_bytes": 78_926,
    },
    "v12r1_worker_claim_fit_p0": {
        "path": (
            "Trajectory Generator/baseline_MLP/validation/"
            "h0_v12r1_run_20260809/claims/01_fit_p0.json"
        ),
        "sha256": "cbc2dfb5af7374bccd1294ab6a2990dbef702e19256700bd499c537584af7d1e",
        "size_bytes": 829,
    },
    "v12r1_adaptation_history": {
        "path": (
            "Trajectory Generator/baseline_MLP/validation/"
            "h0_v12r1_run_20260809/fit/p0/adaptation_history.json"
        ),
        "sha256": "614aa1589b219c2acfea4de7dec8fcb24234408f00ee5838826f2c43419389b4",
        "size_bytes": 1_659,
    },
    "v12r1_adaptation_report": {
        "path": (
            "Trajectory Generator/baseline_MLP/validation/"
            "h0_v12r1_run_20260809/fit/p0/adaptation_report.json"
        ),
        "sha256": "77ce97424a1c3b64905bb9bfe51193993607bae2e45339652fc19ef8475f4f95",
        "size_bytes": 10_097,
    },
    "v12r1_partial_corpus": {
        "path": (
            "Trajectory Generator/baseline_MLP/validation/"
            "h0_v12r1_run_20260809/fit/p0/corpus.npz"
        ),
        "sha256": "7cd2bbe7c0db4221d161214962fdab9324447d09661afcb362848aa1b70ecfdb",
        "size_bytes": 12_618_078,
    },
    "v12r1_partial_module_ctor": {
        "path": (
            "Trajectory Generator/baseline_MLP/validation/h0_v12r1_run_20260809/"
            "fit/p0/rl_module_target_adapted/class_and_ctor_args.pkl"
        ),
        "sha256": "5c98f006d99a71a0f1ddcbb31d8d73fe0a6dade8401e679f6af5b1bc943b4228",
        "size_bytes": 2_262,
    },
    "v12r1_partial_module_metadata": {
        "path": (
            "Trajectory Generator/baseline_MLP/validation/h0_v12r1_run_20260809/"
            "fit/p0/rl_module_target_adapted/metadata.json"
        ),
        "sha256": "3a032ba54abcee8c9bcbb39e72fa05566912e94461d01f3c6228dc60e088bf12",
        "size_bytes": 197,
    },
    "v12r1_partial_module_state": {
        "path": (
            "Trajectory Generator/baseline_MLP/validation/h0_v12r1_run_20260809/"
            "fit/p0/rl_module_target_adapted/module_state.pkl"
        ),
        "sha256": "a4e92b7efb239c5ad60b2805f35c3a5d32d6ea49993fc401d3cc8a4c6203249b",
        "size_bytes": 604_772,
    },
    "v12r1_pipeline_claim": {
        "path": (
            "Trajectory Generator/baseline_MLP/validation/"
            "h0_v12r1_run_20260809/pipeline_claim.json"
        ),
        "sha256": "fe5e4edeebcd61de40f27c2a9db9b4ea4708f898b577836b059b9cd23a7a58b5",
        "size_bytes": 1_784,
    },
    "v12r1_pipeline_ledger": {
        "path": (
            "Trajectory Generator/baseline_MLP/validation/"
            "h0_v12r1_run_20260809/pipeline_ledger.json"
        ),
        "sha256": "225e39897640231e7cbd58cc8e61ccd09bb90034accad6d3007fc58bddb42a8c",
        "size_bytes": 3_155,
    },
}

# Exact terminal V12R2 lineage.  The compact tree digest binds all 251 files;
# the selected records make the causal chain and terminal semantics directly
# inspectable without copying or promoting any R2 output into R3.
V12R2_LINEAGE_ARTIFACTS = {
    "v12r2_protocol_freeze": {
        "path": (
            "Trajectory Generator/baseline_MLP/validation/v12r2/"
            "h0_primary_split_v12r2_autonomy_recovery_protocol_freeze.json"
        ),
        "sha256": "014b72083b9f7ca469d659c1ef9f0bc10c0a6ba196f662b490fed1ea724fe34c",
        "size_bytes": 248_719,
    },
    "v12r2_design_audit": {
        "path": (
            "Trajectory Generator/baseline_MLP/validation/v12r2/"
            "h0_primary_split_v12r2_autonomy_recovery_design_audit.json"
        ),
        "sha256": "af9c356ef34ebfa9523ea76d6e55537ff404b8221aeaf279c4287d27c6853bd0",
        "size_bytes": 31_792,
    },
    "v12r2_execution_lock": {
        "path": (
            "Trajectory Generator/baseline_MLP/validation/v12r2/"
            "h0_primary_split_v12r2_autonomy_recovery_execution_lock.json"
        ),
        "sha256": "c4497c73ce0d0f0a67758929ab4fd0654d10db71c485e6c60fa35bb3dc2bf101",
        "size_bytes": 89_368,
    },
    "v12r2_pipeline_claim": {
        "path": (
            "Trajectory Generator/baseline_MLP/validation/v12r2/"
            "h0_v12r2_run_20260809/pipeline_claim.json"
        ),
        "sha256": "c361016851a13e1e3d0cd5d20cc4f30bc5b604195017e7c9a2a2007a0548c11e",
        "size_bytes": 1_937,
    },
    "v12r2_pipeline_ledger": {
        "path": (
            "Trajectory Generator/baseline_MLP/validation/v12r2/"
            "h0_v12r2_run_20260809/pipeline_ledger.json"
        ),
        "sha256": "c83a8b47446056940afce0006029b5f0d73b331ee80d713e7cb339c9998ad710",
        "size_bytes": 4_212,
    },
    "v12r2_fit_p0_receipt": {
        "path": (
            "Trajectory Generator/baseline_MLP/validation/v12r2/"
            "h0_v12r2_run_20260809/fit/p0/receipt.json"
        ),
        "sha256": "618a0a526ee6befcc8c265d3e1235428417dd6fc26225ff700adc72dd169c2cd",
        "size_bytes": 2_400,
    },
    "v12r2_probe_p0_failure": {
        "path": (
            "Trajectory Generator/baseline_MLP/validation/v12r2/"
            "h0_v12r2_run_20260809/probe/p0/failure.json"
        ),
        "sha256": "ea4ba494622dc18adbfd221ce3ebc4076d110c6daec974d7a781d4985fa86e5e",
        "size_bytes": 59_181,
    },
}
V12R2_TERMINAL_RUN_TREE = {
    "path": (
        "Trajectory Generator/baseline_MLP/validation/v12r2/h0_v12r2_run_20260809"
    ),
    "tree_sha256": "18a89b0fca12f3644164bee211a413a0dafcf8880a51513f53dc0d04bbe020a4",
    "file_count": 251,
    "total_size_bytes": 15_279_831,
}
V12R2_TERMINAL_SEMANTICS = {
    "status": "FAIL_H0_PRIMARY_SPLIT_V12R2_PIPELINE_TERMINAL",
    "attempted_stage": "probe_p0",
    "completed_stages": ["fit_p0"],
    "error": {
        "type": "V12R2ExecutionError",
        "message": "prospective rollout record drifted: run_start",
    },
    "actor_fit_executions_confirmed": 1,
    "actor_updates_confirmed": 1,
    "environment_reset_calls": 1,
    "environment_step_calls": 232,
    "offline_teacher_label_calls_confirmed": 0,
}
V12R2_TERMINAL_LINEAGE = {
    "artifacts": copy.deepcopy(V12R2_LINEAGE_ARTIFACTS),
    "run_tree": copy.deepcopy(V12R2_TERMINAL_RUN_TREE),
    "semantics": copy.deepcopy(V12R2_TERMINAL_SEMANTICS),
}

COLLECTION_CASES = tuple(
    {
        **{
            key: value
            for key, value in copy.deepcopy(case).items()
            if key != "destination"
        },
        "destination": (
            COLLECTION_ROOT / f"r{int(case['round_index'])}" / str(case["case_id"])
        ).as_posix(),
    }
    for case in _v12.COLLECTION_CASES
)
FINAL_CASES = tuple(
    {
        **{
            key: value
            for key, value in copy.deepcopy(case).items()
            if key != "destination"
        },
        "destination": (FINAL_ROOT / str(case["case_id"])).as_posix(),
    }
    for case in _v12.FINAL_CASES
)
FINAL_CASE_IDS = tuple(str(case["case_id"]) for case in FINAL_CASES)


# The journal uses CSR-style offsets: boundary b owns event rows
# offsets[b]:offsets[b+1].  There are B=N+1 boundaries and therefore N+2
# offsets.  Fixed-width Unicode and uint8 byte arrays keep allow_pickle=False.
PROBE_REPLAY_SCHEMA = {
    "schema_id": "H0_V12R3_OBSERVER_REPLAY_BOUNDARIES_V2",
    "container": "NPZ_ALLOW_PICKLE_FALSE",
    "probe_teacher_or_h0_load_allowed": False,
    "boundary_count_rule": "B=N+1",
    "boundary_arrays": {
        "boundary_time_s": "float64[B]",
        "pros_knee_angle_rad": "float32[B]",
        "pros_knee_speed_rad_s": "float32[B]",
        "pros_ankle_angle_rad": "float32[B]",
        "pros_ankle_speed_rad_s": "float32[B]",
        "legacy_left_normal_grf_bw": "float32[B]",
        "legacy_left_in_contact": "bool[B]",
    },
    "step_arrays": {
        "actor_observations": "float32[N,35]",
        "previous_penetration_m": "float64[N]",
    },
    "legacy_fsm_config_arrays": {
        "legacy_fsm_config_json_utf8": "uint8[C]",
        "legacy_fsm_config_sha256_ascii": "uint8[64]",
        "legacy_fsm_module_utf8": "uint8[M]",
        "legacy_fsm_class_utf8": "uint8[K]",
    },
    "legacy_fsm_config_canonicalization": {
        "input": "DATACLASS_ASDICT_WITH_EVENT_SOURCE_LEGACY_EVENTS",
        "encoding": "UTF-8",
        "json": "SORT_KEYS_TRUE_SEPARATORS_COMMA_COLON_ENSURE_ASCII_FALSE_ALLOW_NAN_FALSE",
        "trailing_newline": False,
        "hash": "SHA256_OF_EXACT_legacy_fsm_config_json_utf8_BYTES",
    },
    "scalar_arrays": {
        "body_weight_n": "float64[1]",
        "event_contract_id_utf8": "uint8[J]",
    },
    "left_event_journal_arrays": {
        "legacy_left_event_boundary_offsets": "int64[B+1]",
        "legacy_left_event_side": "str_<U5[E]",
        "legacy_left_event_type": "str_<U11[E]",
        "legacy_left_event_onset_time_s": "float64[E]",
        "legacy_left_event_confirmed_time_s": "float64[E]",
        "legacy_left_event_delivered_time_s": "float64[E]",
        "legacy_left_event_cycle_duration_present": "bool[E]",
        "legacy_left_event_cycle_duration_s": "float64[E]",
        "legacy_left_event_contact_duration_present": "bool[E]",
        "legacy_left_event_contact_duration_s": "float64[E]",
        "legacy_left_event_startup_contact_present": "bool[E]",
        "legacy_left_event_startup_contact": "bool[E]",
    },
    "left_event_journal_rules": {
        "side_value": "left",
        "event_values": ["heel_strike", "toe_off"],
        "boundary_offsets": "NONDECREASING_START_0_END_E_LENGTH_B_PLUS_1",
        "timestamp_order": "onset<=confirmed<=delivered==owning_boundary_time",
        "optional_float_absent_value": 0.0,
        "optional_bool_absent_value": False,
        "event_order": "SOURCE_ORDER_WITHIN_BOUNDARY;_STRICTLY_INCREASING_LEFT_ONSETS_GLOBAL",
        "reset_boundary_event_count": 0,
    },
    "teacher_mutable_actor_columns": list(range(10, 25)),
    "all_other_actor_columns_byte_exact": True,
}

TEACHER_MUTABLE_ACTOR_COLUMNS = tuple(range(10, 25))
TEACHER_IMMUTABLE_ACTOR_COLUMNS = tuple(
    index
    for index in range(EXPECTED_ACTOR_FEATURES)
    if index not in TEACHER_MUTABLE_ACTOR_COLUMNS
)

P0_REPRODUCTION_TOLERANCE = {
    "absolute": 1.0e-9,
    "relative": 1.0e-9,
    "rule": "ABS_OBSERVED_MINUS_REFERENCE_LE_ABS_PLUS_REL_TIMES_ABS_REFERENCE",
    "applies_to": ["rmse", "max_abs_error", "reset_max_abs_error"],
    "reference": "SOLE_V12R3_DESIGN_AUDIT_P0_IN_MEMORY_FIT",
    "retry_or_retuning": False,
}

FIT = {
    **copy.deepcopy(_v12.FIT),
    "fit_contract_id": FIT_CONTRACT_ID,
    "p0_reproduction_tolerance": copy.deepcopy(P0_REPRODUCTION_TOLERANCE),
}

AUTHORITY = {
    **copy.deepcopy(_v12.AUTHORITY),
    "authority_text": AUTHORITY_TEXT,
    "authority_scope": TRAINING_READINESS_SCOPE,
}

SOURCE_RELATIVE_PATHS = {
    "contract": (
        "Trajectory Generator/baseline_MLP/validation/v12r3/"
        "h0_primary_split_v12r3_autonomy_recovery_contract.py"
    ),
    "freeze": (
        "Trajectory Generator/baseline_MLP/validation/v12r3/"
        "freeze_h0_primary_split_v12r3_autonomy_recovery.py"
    ),
    "contract_tests": (
        "Trajectory Generator/baseline_MLP/validation/v12r3/"
        "test_h0_primary_split_v12r3_autonomy_recovery_contract.py"
    ),
    "freeze_tests": (
        "Trajectory Generator/baseline_MLP/validation/v12r3/"
        "test_freeze_h0_primary_split_v12r3_autonomy_recovery.py"
    ),
    "line_endings": ".gitattributes",
    "line_endings_r1": ("Trajectory Generator/baseline_MLP/validation/.gitattributes"),
    "line_endings_r2": (
        "Trajectory Generator/baseline_MLP/validation/v12r2/.gitattributes"
    ),
    "line_endings_r3": (
        "Trajectory Generator/baseline_MLP/validation/v12r3/.gitattributes"
    ),
    "forensic_writer": "validation/h0_forensic_rollout.py",
    "v10s_blend": "validation/h0_primary_split_v10s_blend.py",
    "v10s_contract": "validation/h0_primary_split_v10s_safe_dagger_contract.py",
    "v11_contract": "validation/h0_primary_split_v11_weighted_full_mean_contract.py",
    "v11_fitter": "validation/h0_primary_split_v11_weighted_fit.py",
    "v11_runner": "validation/run_h0_primary_split_v11_weighted_full_mean.py",
}
INPUT_RELATIVE_PATHS = copy.deepcopy(_v12.INPUT_RELATIVE_PATHS)
INPUT_RELATIVE_PATHS["v12_protocol_freeze"] = V12_PROTOCOL_FREEZE_PATH.as_posix()
INPUT_RELATIVE_PATHS.update(
    {name: artifact["path"] for name, artifact in V12R1_LINEAGE_ARTIFACTS.items()}
)
INPUT_RELATIVE_PATHS.update(
    {name: artifact["path"] for name, artifact in V12R2_LINEAGE_ARTIFACTS.items()}
)
AUTHORITATIVE_SHA256 = {
    **copy.deepcopy(_v12.V11_AUTHORITATIVE_SHA256),
    "v12_protocol_freeze": V12_PROTOCOL_FREEZE_SHA256,
    **{name: artifact["sha256"] for name, artifact in V12R1_LINEAGE_ARTIFACTS.items()},
    **{name: artifact["sha256"] for name, artifact in V12R2_LINEAGE_ARTIFACTS.items()},
}

FUTURE_EXECUTION_SOURCES_REQUIRED = (
    "v12r3_recovery_weighted_fitter",
    "v12r3_pure_probe_observer_labeler",
    "v12r3_design_audit_runner",
    "v12r3_pipeline_runner",
    "v12r3_execution_tests",
    "v12r3_fitter_tests",
    "v12r3_labeler_tests",
)
FUTURE_EXECUTION_SOURCE_RELATIVE_PATHS = {
    "v12r3_recovery_weighted_fitter": (
        "Trajectory Generator/baseline_MLP/validation/v12r3/"
        "h0_primary_split_v12r3_recovery_weighted_fitter.py"
    ),
    "v12r3_pure_probe_observer_labeler": (
        "Trajectory Generator/baseline_MLP/validation/v12r3/"
        "h0_primary_split_v12r3_pure_probe_observer_labeler.py"
    ),
    "v12r3_design_audit_runner": (
        "Trajectory Generator/baseline_MLP/validation/v12r3/"
        "run_h0_primary_split_v12r3_design_audit.py"
    ),
    "v12r3_pipeline_runner": (
        "Trajectory Generator/baseline_MLP/validation/v12r3/"
        "run_h0_primary_split_v12r3_autonomy_recovery.py"
    ),
    "v12r3_execution_tests": (
        "Trajectory Generator/baseline_MLP/validation/v12r3/"
        "test_h0_primary_split_v12r3_execution.py"
    ),
    "v12r3_fitter_tests": (
        "Trajectory Generator/baseline_MLP/validation/v12r3/"
        "test_h0_primary_split_v12r3_recovery_weighted_fit.py"
    ),
    "v12r3_labeler_tests": (
        "Trajectory Generator/baseline_MLP/validation/v12r3/"
        "test_h0_primary_split_v12r3_pure_probe_observer_labeler.py"
    ),
}


def _r2_status(value: str) -> str:
    return value.replace("V12", "V12R3")


for _status_name in (
    "PROTOCOL_FREEZE_PASS_STATUS",
    "PROTOCOL_FREEZE_FAIL_STATUS",
    "FIT_COMPLETE_STATUS",
    "FIT_PASS_STATUS",
    "FIT_FAIL_STATUS",
    "PROBE_INTEGRITY_PASS_STATUS",
    "PROBE_INTEGRITY_FAIL_STATUS",
    "PURE_PROBE_COMPLETE_STATUS",
    "PURE_PROBE_PASS_STATUS",
    "PURE_PROBE_FAIL_STATUS",
    "OBSERVER_LABEL_PASS_STATUS",
    "OBSERVER_LABEL_FAIL_STATUS",
    "COLLECTION_COMPLETE_STATUS",
    "COLLECTION_PASS_STATUS",
    "COLLECTION_FAIL_STATUS",
    "LATCH_INDEPENDENCE_PASS_STATUS",
    "LATCH_INDEPENDENCE_FAIL_STATUS",
    "CANDIDATE_FREEZE_PASS_STATUS",
    "CANDIDATE_FREEZE_FAIL_STATUS",
    "FINAL_ROLLOUT_COMPLETE_STATUS",
    "FINAL_ROLLOUT_PASS_STATUS",
    "FINAL_ROLLOUT_FAIL_STATUS",
    "FINAL_DEVELOPMENT_COMPLETE_STATUS",
    "FINAL_DEVELOPMENT_PASS_STATUS",
    "FINAL_DEVELOPMENT_FAIL_STATUS",
):
    globals()[_status_name] = _r2_status(getattr(_v12, _status_name))

DESIGN_AUDIT_PASS_STATUS = "PASS_H0_PRIMARY_SPLIT_V12R3_DESIGN_AUDIT"
DESIGN_AUDIT_FAIL_STATUS = "FAIL_H0_PRIMARY_SPLIT_V12R3_DESIGN_AUDIT"
EXECUTION_LOCK_PASS_STATUS = "PASS_H0_PRIMARY_SPLIT_V12R3_EXECUTION_LOCK"
EXECUTION_LOCK_FAIL_STATUS = "FAIL_H0_PRIMARY_SPLIT_V12R3_EXECUTION_LOCK"


# Rebind every inherited pure function after all V12R3 globals are defined.
for _name, _value in vars(_v12).items():
    if isinstance(_value, types.FunctionType) and _value.__module__ == _v12.__name__:
        globals()[_name] = _rebind_function(_value)


_NEXT_STAGE_RENAMES = {
    "STOP_V12_FIT_FAILURE": "STOP_V12R3_FIT_FAILURE",
    "STOP_V12_PROBE_INTEGRITY_FAILURE": "STOP_V12R3_PROBE_INTEGRITY_FAILURE",
    "STOP_V12_TERMINAL": "STOP_V12R3_TERMINAL",
    "STOP_V12_COLLECTION_INTEGRITY_FAILURE": (
        "STOP_V12R3_COLLECTION_INTEGRITY_FAILURE"
    ),
    "STOP_V12_LABEL_INTEGRITY_FAILURE": "STOP_V12R3_LABEL_INTEGRITY_FAILURE",
    "STOP_V12_FREEZE_FAILURE": "STOP_V12R3_FREEZE_FAILURE",
}


def _versioned_next_stage(result: Mapping[str, Any]) -> dict[str, Any]:
    payload = dict(result)
    next_stage = payload.get("next_stage")
    if isinstance(next_stage, str):
        payload["next_stage"] = _NEXT_STAGE_RENAMES.get(next_stage, next_stage)
    return payload


def _next_stage_wrapper(function: types.FunctionType) -> Any:
    def wrapped(*args: Any, **kwargs: Any) -> dict[str, Any]:
        return _versioned_next_stage(function(*args, **kwargs))

    wrapped.__name__ = function.__name__
    wrapped.__doc__ = function.__doc__
    return wrapped


# Only presentation-level next-stage labels are translated.  The underlying
# V12 code objects and every lower-case wire key remain bytecode-identical.
for _gate_name in (
    "probe_integrity_gate",
    "pure_probe_gate",
    "collection_data_gate",
    "latch_dependence_gate",
    "observer_label_gate",
    "candidate_freeze_gate",
    "final_rollout_gate",
    "final_development_gate",
):
    globals()[f"_v12_{_gate_name}"] = globals()[_gate_name]
    globals()[_gate_name] = _next_stage_wrapper(globals()[_gate_name])


def replay_schema_gate(value: Any) -> dict[str, Any]:
    """Validate the exact self-contained V12R3 replay declaration."""

    schema = value if isinstance(value, Mapping) else {}
    journal = schema.get("left_event_journal_arrays")
    config = schema.get("legacy_fsm_config_arrays")
    checks = {
        "exact_schema": schema == PROBE_REPLAY_SCHEMA,
        "teacher_absent_during_probe": schema.get("probe_teacher_or_h0_load_allowed")
        is False,
        "inclusive_teacher_columns": schema.get("teacher_mutable_actor_columns")
        == list(range(10, 25)),
        "canonical_config_bytes_and_hash": isinstance(config, Mapping)
        and set(config)
        == {
            "legacy_fsm_config_json_utf8",
            "legacy_fsm_config_sha256_ascii",
            "legacy_fsm_module_utf8",
            "legacy_fsm_class_utf8",
        },
        "event_journal_complete": isinstance(journal, Mapping)
        and set(journal)
        == {
            "legacy_left_event_boundary_offsets",
            "legacy_left_event_side",
            "legacy_left_event_type",
            "legacy_left_event_onset_time_s",
            "legacy_left_event_confirmed_time_s",
            "legacy_left_event_delivered_time_s",
            "legacy_left_event_cycle_duration_present",
            "legacy_left_event_cycle_duration_s",
            "legacy_left_event_contact_duration_present",
            "legacy_left_event_contact_duration_s",
            "legacy_left_event_startup_contact_present",
            "legacy_left_event_startup_contact",
        },
    }
    passed = all(checks.values())
    return {
        "schema_version": SCHEMA_VERSION,
        "passed": passed,
        "protocol_id": PROTOCOL_ID,
        "checks": checks,
    }


def replay_event_topology_gate(
    payload: Mapping[str, Sequence[Any]], *, n_steps: int
) -> dict[str, Any]:
    """Pure shape/order check for a decoded replay NPZ mapping.

    Dtype and cryptographic byte checks remain execution-source duties.  This
    helper validates the lossless CSR event topology and optional-field
    presence semantics without filesystem or environment access.
    """

    if type(n_steps) is not int or n_steps < 1:
        raise ValueError("n_steps must be a positive integer")
    boundaries = n_steps + 1
    boundary_names = tuple(PROBE_REPLAY_SCHEMA["boundary_arrays"])
    step_names = tuple(PROBE_REPLAY_SCHEMA["step_arrays"])
    event_names = tuple(PROBE_REPLAY_SCHEMA["left_event_journal_arrays"])
    offsets_raw = payload.get("legacy_left_event_boundary_offsets", ())
    try:
        offsets = tuple(int(item) for item in offsets_raw)
    except (TypeError, ValueError):
        offsets = ()
    offsets_shape = len(offsets) == boundaries + 1
    monotonic = (
        offsets_shape
        and offsets[0] == 0
        and all(left <= right for left, right in zip(offsets, offsets[1:]))
    )
    event_count = offsets[-1] if monotonic else -1

    def _length(name: str) -> int:
        try:
            return len(payload[name])
        except (KeyError, TypeError):
            return -1

    event_lengths = {
        name: _length(name)
        for name in event_names
        if name != "legacy_left_event_boundary_offsets"
    }
    sides = payload.get("legacy_left_event_side", ())
    event_types = payload.get("legacy_left_event_type", ())
    onset = payload.get("legacy_left_event_onset_time_s", ())
    confirmed = payload.get("legacy_left_event_confirmed_time_s", ())
    delivered = payload.get("legacy_left_event_delivered_time_s", ())
    times_causal = event_count >= 0 and all(
        float(onset[index]) <= float(confirmed[index]) <= float(delivered[index])
        for index in range(event_count)
    )
    owning_boundaries = event_count >= 0 and all(
        abs(float(delivered[index]) - float(payload["boundary_time_s"][boundary]))
        <= 1.0e-9
        for boundary in range(boundaries)
        for index in range(offsets[boundary], offsets[boundary + 1])
    )
    optional_pairs = (
        (
            "legacy_left_event_cycle_duration_present",
            "legacy_left_event_cycle_duration_s",
            0.0,
        ),
        (
            "legacy_left_event_contact_duration_present",
            "legacy_left_event_contact_duration_s",
            0.0,
        ),
        (
            "legacy_left_event_startup_contact_present",
            "legacy_left_event_startup_contact",
            False,
        ),
    )
    try:
        optional_absent_canonical = event_count >= 0 and all(
            bool(payload[present][index]) or payload[value][index] == absent
            for present, value, absent in optional_pairs
            for index in range(event_count)
        )
    except (KeyError, IndexError, TypeError):
        optional_absent_canonical = False
    checks = {
        "boundary_shapes": all(_length(name) == boundaries for name in boundary_names),
        "step_shapes": all(_length(name) == n_steps for name in step_names),
        "offset_shape_and_order": monotonic,
        "reset_boundary_empty": monotonic and offsets[1] == 0,
        "event_array_lengths": event_count >= 0
        and all(length == event_count for length in event_lengths.values()),
        "left_only": event_count >= 0 and all(str(value) == "left" for value in sides),
        "event_types": event_count >= 0
        and all(str(value) in {"heel_strike", "toe_off"} for value in event_types),
        "causal_timestamps": times_causal,
        "delivered_at_owning_boundary": owning_boundaries,
        "strictly_increasing_onsets": event_count >= 0
        and all(
            float(onset[index]) < float(onset[index + 1])
            for index in range(max(0, event_count - 1))
        ),
        "optional_absent_values_canonical": optional_absent_canonical,
    }
    return {
        "schema_version": SCHEMA_VERSION,
        "passed": all(checks.values()),
        "protocol_id": PROTOCOL_ID,
        "n_steps": n_steps,
        "boundary_count": boundaries,
        "event_count": event_count if event_count >= 0 else None,
        "checks": checks,
    }


def _metric_triplet_close(
    observed: Mapping[str, Any], reference: Mapping[str, Any]
) -> bool:
    if not (_metric_triplet(observed) and _metric_triplet(reference)):
        return False
    absolute = float(P0_REPRODUCTION_TOLERANCE["absolute"])
    relative = float(P0_REPRODUCTION_TOLERANCE["relative"])
    return all(
        abs(float(observed[name]) - float(reference[name]))
        <= absolute + relative * abs(float(reference[name]))
        for name in P0_REPRODUCTION_TOLERANCE["applies_to"]
    )


def p0_reproduction_gate(
    observed: Mapping[str, Any],
    reference: Mapping[str, Any],
    tolerance: Any,
) -> dict[str, Any]:
    """Apply the frozen componentwise P0 design-audit tolerance."""

    checks = {
        "observed_metrics": _metric_triplet(observed),
        "reference_metrics": _metric_triplet(reference),
        "tolerance_exact": tolerance == P0_REPRODUCTION_TOLERANCE,
        "componentwise_close": _metric_triplet_close(observed, reference),
    }
    return {
        "schema_version": SCHEMA_VERSION,
        "passed": all(checks.values()),
        "protocol_id": PROTOCOL_ID,
        "checks": checks,
    }


_v12_fit_gate = fit_gate


def fit_gate(summary: Mapping[str, Any], *, stage: str) -> dict[str, Any]:
    """Extend the V12 gate with a cryptographically bindable P0 comparison."""

    result = _v12_fit_gate(summary, stage=stage)
    reproduction = (
        p0_reproduction_gate(
            summary.get("metrics", {}),
            summary.get("p0_reproduction_reference_metrics", {}),
            summary.get("p0_reproduction_tolerance"),
        )
        if stage == "p0"
        else {
            "schema_version": SCHEMA_VERSION,
            "passed": True,
            "protocol_id": PROTOCOL_ID,
            "checks": {"not_p0": True},
        }
    )
    result["checks"]["p0_reproduction_metrics_and_tolerance_exact"] = (
        reproduction["passed"] is True
    )
    result["p0_reproduction_gate"] = reproduction
    passed = all(result["checks"].values())
    result["passed"] = passed
    result["status"] = FIT_PASS_STATUS if passed else FIT_FAIL_STATUS
    result["next_stage"] = (
        f"PURE_PROBE_{stage.upper()}" if passed else "STOP_V12R3_FIT_FAILURE"
    )
    return result


FIT_COUNT_WIRE_KEYS = frozenset(
    {
        "v11_seed_sample_count",
        "v12_dagger_sample_count",
        "pure_probe_label_sample_count",
        "pure_probe_label_stages",
        "sample_count",
        "reset_row_count",
        "completed_v12_collection_rounds",
    }
)


def fit_contract_self_check() -> dict[str, Any]:
    """Exercise the producer/gate wire schema without fitting or I/O."""

    stage_results: dict[str, Any] = {}
    for index, stage in enumerate(FIT_STAGES):
        labelled_rows = {prior: 1 for prior in FIT_STAGES[:index]}
        stage_checks: dict[str, bool] = {}
        try:
            expected = expected_fit_counts(stage, labelled_probe_rows=labelled_rows)
            audit = {
                "v11_seed_sample_count": expected["v11_seed_sample_count"],
                "v12_dagger_sample_count": expected["v12_dagger_sample_count"],
                "same_state_v12_dagger_sample_count": expected[
                    "v12_dagger_sample_count"
                ],
                "pure_probe_label_sample_count": expected[
                    "pure_probe_label_sample_count"
                ],
                "same_state_pure_probe_label_sample_count": expected[
                    "pure_probe_label_sample_count"
                ],
                "sample_count": expected["sample_count"],
                "reset_row_count": expected["reset_row_count"],
                "duplicate_sample_count": 0,
                "all_finite": True,
            }
            inherited_failure = _v12_fit_gate({}, stage=stage)
            revised_failure = fit_gate({}, stage=stage)
        except Exception as exc:
            stage_results[stage] = {
                "passed": False,
                "error": f"{type(exc).__name__}: {exc}",
                "checks": stage_checks,
            }
            continue
        stage_checks = {
            "wire_keys_exact": set(expected) == FIT_COUNT_WIRE_KEYS,
            "legacy_v12_keys_preserved": (
                "completed_v12_collection_rounds" in expected
                and "v12_dagger_sample_count" in expected
                and not any("v12r3" in key for key in expected)
            ),
            "fitter_shaped_audit_accepted": _fit_corpus_audit_matches(audit, expected),
            "inherited_gate_total_on_malformed_input": (
                isinstance(inherited_failure, Mapping)
                and inherited_failure.get("passed") is False
            ),
            "revised_gate_total_on_malformed_input": (
                isinstance(revised_failure, Mapping)
                and revised_failure.get("passed") is False
            ),
        }
        stage_results[stage] = {
            "passed": all(stage_checks.values()),
            "checks": stage_checks,
            "expected_fit_counts": expected,
        }
    passed = set(stage_results) == set(FIT_STAGES) and all(
        result.get("passed") is True for result in stage_results.values()
    )
    return {
        "schema_version": SCHEMA_VERSION,
        "status": (
            "PASS_H0_PRIMARY_SPLIT_V12R3_FIT_CONTRACT_SELF_CHECK"
            if passed
            else "FAIL_H0_PRIMARY_SPLIT_V12R3_FIT_CONTRACT_SELF_CHECK"
        ),
        "passed": passed,
        "protocol_id": PROTOCOL_ID,
        "wire_keys": sorted(FIT_COUNT_WIRE_KEYS),
        "stages": stage_results,
    }


def _execution_source_records_match(value: Any) -> bool:
    if not isinstance(value, Mapping) or set(value) != set(
        FUTURE_EXECUTION_SOURCES_REQUIRED
    ):
        return False
    return all(
        artifact_record_matches(
            value[name], FUTURE_EXECUTION_SOURCE_RELATIVE_PATHS[name]
        )
        for name in FUTURE_EXECUTION_SOURCES_REQUIRED
    )


def design_audit_gate(payload: Mapping[str, Any]) -> dict[str, Any]:
    """Validate the sole in-memory P0 feasibility audit, with no checkpoint."""

    observed = payload.get("observed_metrics")
    reference = payload.get("p0_reproduction_reference_metrics")
    checks = {
        "schema": payload.get("schema_version") == SCHEMA_VERSION,
        "identity": payload.get("protocol_id") == PROTOCOL_ID
        and payload.get("contract_id") == FIT_CONTRACT_ID
        and payload.get("design_audit_id") == DESIGN_AUDIT_ID,
        "status": payload.get("status") == DESIGN_AUDIT_PASS_STATUS
        and payload.get("passed") is True,
        "protocol_freeze_bound": artifact_record_matches(
            payload.get("protocol_freeze"), PROTOCOL_FREEZE_PATH
        ),
        "immutable_v12_freeze_bound": payload.get("v12_protocol_freeze")
        == {
            "path": V12_PROTOCOL_FREEZE_PATH.as_posix(),
            "sha256": V12_PROTOCOL_FREEZE_SHA256,
            "size_bytes": V12_PROTOCOL_FREEZE_SIZE_BYTES,
        },
        "execution_sources_exact": _execution_source_records_match(
            payload.get("execution_sources")
        ),
        "v12r1_terminal_failure_lineage_exact": (
            payload.get("v12r1_terminal_failure_lineage") == V12R1_LINEAGE_ARTIFACTS
        ),
        "v12r2_terminal_failure_lineage_exact": (
            payload.get("v12r2_terminal_failure_lineage") == V12R2_TERMINAL_LINEAGE
        ),
        "fit_contract_self_check": (
            payload.get("fit_contract_self_check") == fit_contract_self_check()
            and payload.get("fit_contract_self_check", {}).get("passed") is True
        ),
        "fit_design": payload.get("fit_design") == FIT,
        "replay_schema": replay_schema_gate(payload.get("probe_replay_schema"))[
            "passed"
        ]
        is True,
        "metrics": _metric_triplet(observed)
        and _metrics_within_thresholds(observed)
        and _metric_triplet(reference)
        and _metrics_within_thresholds(reference),
        "reference_is_observed": observed == reference,
        "tolerance_frozen": payload.get("p0_reproduction_tolerance")
        == P0_REPRODUCTION_TOLERANCE,
        "single_in_memory_fit": payload.get("dry_run") is True
        and payload.get("actor_fit_executions") == 1
        and payload.get("actor_updates") == 1,
        "no_checkpoint": payload.get("no_candidate_checkpoint") is True
        and payload.get("candidate_checkpoints_persisted") == 0
        and payload.get("candidate_checkpoint_paths") == [],
        "zero_environment_or_labels": payload.get("environment_reset_calls") == 0
        and payload.get("environment_step_calls") == 0
        and payload.get("offline_teacher_label_calls") == 0,
        "zero_forbidden_updates": payload.get("critic_updates") == 0
        and payload.get("ppo_updates") == 0,
        "no_retry_sweep_rescue": payload.get("retry_authorized") is False
        and payload.get("sweep_authorized") is False
        and payload.get("rescue_authorized") is False,
        "closed_data": payload.get("protected_trials_opened") == []
        and payload.get("reserve_trials_opened") == [],
        "receipt_only": payload.get("artifacts_written")
        == [DESIGN_AUDIT_RECEIPT_PATH.as_posix()],
    }
    passed = all(checks.values())
    return {
        "schema_version": SCHEMA_VERSION,
        "status": DESIGN_AUDIT_PASS_STATUS if passed else DESIGN_AUDIT_FAIL_STATUS,
        "passed": passed,
        "protocol_id": PROTOCOL_ID,
        "checks": checks,
    }


def execution_lock_gate(payload: Mapping[str, Any]) -> dict[str, Any]:
    """Validate a future no-clobber one-shot lock before any execution call."""

    authority = payload.get("execution_authority")
    manifest = payload.get("declared_mutation_paths")
    expected_manifest = {
        name: path.as_posix() for name, path in declared_mutation_paths().items()
    }
    inherited = payload.get("inherited_runtime_evidence")
    runtime = payload.get("runtime")
    checks = {
        "schema": payload.get("schema_version") == SCHEMA_VERSION,
        "identity": payload.get("protocol_id") == PROTOCOL_ID
        and payload.get("pipeline_id") == PIPELINE_ID,
        "status": payload.get("status") == EXECUTION_LOCK_PASS_STATUS
        and payload.get("passed") is True,
        "protocol_freeze_bound": artifact_record_matches(
            payload.get("protocol_freeze"), PROTOCOL_FREEZE_PATH
        ),
        "immutable_v12_freeze_bound": payload.get("v12_protocol_freeze")
        == {
            "path": V12_PROTOCOL_FREEZE_PATH.as_posix(),
            "sha256": V12_PROTOCOL_FREEZE_SHA256,
            "size_bytes": V12_PROTOCOL_FREEZE_SIZE_BYTES,
        },
        "design_audit_bound": artifact_record_matches(
            payload.get("design_audit"), DESIGN_AUDIT_RECEIPT_PATH
        )
        and payload.get("design_audit_gate_passed") is True,
        "execution_sources_exact": _execution_source_records_match(
            payload.get("execution_sources")
        ),
        "v12r1_terminal_failure_lineage_exact": (
            payload.get("v12r1_terminal_failure_lineage") == V12R1_LINEAGE_ARTIFACTS
        ),
        "v12r2_terminal_failure_lineage_exact": (
            payload.get("v12r2_terminal_failure_lineage") == V12R2_TERMINAL_LINEAGE
        ),
        "fit_contract_self_check": (
            payload.get("fit_contract_self_check") == fit_contract_self_check()
            and payload.get("fit_contract_self_check", {}).get("passed") is True
        ),
        "inherited_runtime_evidence_exact": isinstance(inherited, Mapping)
        and inherited.get("passed") is True
        and inherited.get("v11_execution_lock") == V11_EXECUTION_LOCK_ARTIFACT
        and inherited.get("source_count") == 66
        and inherited.get("input_count") == 54
        and inherited.get("sources_exact") is True
        and inherited.get("inputs_exact") is True,
        "current_runtime_training_ready": isinstance(runtime, Mapping)
        and runtime.get("inference_stack_ready") is True
        and isinstance(runtime.get("platform_plugin_readiness"), Mapping)
        and runtime["platform_plugin_readiness"].get("passed") is True,
        "training_ready_receipt": payload.get("training_ready") is True
        and payload.get("training_ready_scope")
        == {
            "system": runtime.get("system") if isinstance(runtime, Mapping) else None,
            "machine": runtime.get("machine") if isinstance(runtime, Mapping) else None,
            "executable": (
                runtime.get("executable") if isinstance(runtime, Mapping) else None
            ),
        },
        "execution_authority": isinstance(authority, Mapping)
        and set(authority)
        == {"authority_date", "authority_text", "authority_scope", "one_shot"}
        and authority.get("authority_date") == REVISION
        and authority.get("authority_text") == AUTHORITY_TEXT
        and authority.get("authority_scope") == EXECUTION_AUTHORITY_SCOPE
        and authority.get("one_shot") is True,
        "one_shot_fail_closed": payload.get("retry_authorized") is False
        and payload.get("sweep_authorized") is False
        and payload.get("rescue_authorized") is False
        and payload.get("post_hoc_retuning_authorized") is False,
        "manifest_exact": manifest == expected_manifest,
        "pipeline_unclaimed": payload.get("pipeline_claim_preexisting") is False,
        "pre_execution_zero_counts": payload.get("actor_fit_executions") == 0
        and payload.get("environment_reset_calls") == 0
        and payload.get("environment_step_calls") == 0
        and payload.get("offline_teacher_label_calls") == 0
        and payload.get("actor_updates") == 0
        and payload.get("critic_updates") == 0
        and payload.get("ppo_updates") == 0,
        "closed_data": payload.get("protected_trials_opened") == []
        and payload.get("reserve_trials_opened") == [],
        "p0_tolerance_frozen": payload.get("p0_reproduction_tolerance")
        == P0_REPRODUCTION_TOLERANCE,
    }
    passed = all(checks.values())
    return {
        "schema_version": SCHEMA_VERSION,
        "status": EXECUTION_LOCK_PASS_STATUS if passed else EXECUTION_LOCK_FAIL_STATUS,
        "passed": passed,
        "protocol_id": PROTOCOL_ID,
        "checks": checks,
    }


_inherited_declared_mutation_paths = declared_mutation_paths


def declared_mutation_paths() -> dict[str, PurePosixPath]:
    """Return the exhaustive V12R3 write manifest, including failure paths."""

    paths = dict(_inherited_declared_mutation_paths())
    paths["candidate_freeze_summary"] = FIT_ROOT / "candidate_freeze_summary.json"
    paths["candidate_freeze_gate"] = FIT_ROOT / "candidate_freeze_gate.json"
    for stage in FIT_STAGES:
        paths[f"probe_failure_{stage}"] = PROBE_ROOT / stage / "failure.json"
    for case in COLLECTION_CASES:
        prefix = f"collect_r{case['round_index']}_{case['case_id']}"
        paths[f"{prefix}_failure"] = PurePosixPath(case["destination"]) / "failure.json"
    for case in FINAL_CASES:
        paths[f"final_{case['case_id']}_failure"] = (
            PurePosixPath(case["destination"]) / "failure.json"
        )
    return paths


__all__ = sorted(
    set(getattr(_v12, "__all__", ()))
    | {
        "AUTHORITATIVE_SHA256",
        "AUTHORITY_TEXT",
        "DESIGN_AUDIT_ID",
        "DESIGN_AUDIT_PASS_STATUS",
        "DESIGN_AUDIT_FAIL_STATUS",
        "EXECUTION_LOCK_PASS_STATUS",
        "EXECUTION_LOCK_FAIL_STATUS",
        "EXECUTION_AUTHORITY_SCOPE",
        "FIT_COUNT_WIRE_KEYS",
        "FUTURE_EXECUTION_SOURCE_RELATIVE_PATHS",
        "P0_REPRODUCTION_TOLERANCE",
        "TEACHER_MUTABLE_ACTOR_COLUMNS",
        "TEACHER_IMMUTABLE_ACTOR_COLUMNS",
        "TRAINING_READINESS_SCOPE",
        "V12_PROTOCOL_FREEZE_PATH",
        "V12_PROTOCOL_FREEZE_SHA256",
        "V11_EXECUTION_LOCK_ARTIFACT",
        "V12R1_LINEAGE_ARTIFACTS",
        "V12R2_LINEAGE_ARTIFACTS",
        "V12R2_TERMINAL_LINEAGE",
        "V12R2_TERMINAL_RUN_TREE",
        "V12R2_TERMINAL_SEMANTICS",
        "design_audit_gate",
        "execution_lock_gate",
        "fit_contract_self_check",
        "p0_reproduction_gate",
        "replay_event_topology_gate",
        "replay_schema_gate",
    }
)
