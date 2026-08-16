"""Additive V12R1 correction of the frozen V12 autonomy/recovery contract.

V12R1 preserves the complete V12 scientific design and immutable V12 evidence,
but closes two fail-closed specification gaps discovered before execution:

* the coherent H0 teacher may replace actor columns 10 through 24 inclusive;
* a pure probe persists a self-contained, pickle-free replay journal, including
  the canonical legacy-FSM configuration and every raw left event needed by the
  later observer-only label stage.

The implementation deliberately derives its unchanged gates from the immutable
V12 module.  Functions are rebound to this module's globals, so every inherited
gate consumes the V12R1 identifiers and paths without mutating the V12 module.
This module performs no I/O, fitting, policy inference, or environment access.
"""

from __future__ import annotations

import copy
import types
from pathlib import PurePosixPath
from typing import Any, Mapping, Sequence

try:
    from validation import h0_primary_split_v12_autonomy_recovery_contract as _v12
except (ImportError, ModuleNotFoundError):  # Flat import when this dir is on sys.path.
    import h0_primary_split_v12_autonomy_recovery_contract as _v12


def _rewrite_code_strings(code: types.CodeType) -> types.CodeType:
    """Return a code object whose user-facing V12 literals say V12R1."""

    constants: list[Any] = []
    for value in code.co_consts:
        if isinstance(value, types.CodeType):
            constants.append(_rewrite_code_strings(value))
        elif isinstance(value, str):
            constants.append(value.replace("V12", "V12R1").replace("v12", "v12r1"))
        else:
            constants.append(value)
    return code.replace(co_consts=tuple(constants))


# Import the frozen design namespace without modifying it.  Mutable values that
# are changed below are always replaced by fresh copies.
for _name, _value in vars(_v12).items():
    if not _name.startswith("__") and (
        not isinstance(_value, types.FunctionType)
        or _value.__module__ != _v12.__name__
    ):
        globals()[_name] = _value


SCHEMA_VERSION = 121
REVISION = "2026-08-09"
PROTOCOL_ID = "AB06_H0_PRIMARY_SPLIT_V12R1_V26_AUTONOMY_RECOVERY"
PIPELINE_ID = "H0_V12R1_V26_RECOVERY_WEIGHTED_PURE_PROBE_DAGGER"
FIT_CONTRACT_ID = "h0_primary_split_v12r1_recovery_weighted_full_mean_v1"
DESIGN_AUDIT_ID = "H0_V12R1_P0_RECOVERY_WEIGHTED_FEASIBILITY_V1"

V12R1_VALIDATION_ROOT = PurePosixPath(
    "Trajectory Generator/baseline_MLP/validation"
)
V12_VALIDATION_ROOT = V12R1_VALIDATION_ROOT
RUN_ROOT = V12R1_VALIDATION_ROOT / (
    "h0_v12r1_run_20260809"
)
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

PROTOCOL_FREEZE_PATH = V12R1_VALIDATION_ROOT / (
    "h0_primary_split_v12r1_autonomy_recovery_protocol_freeze.json"
)
EXECUTION_LOCK_PATH = V12R1_VALIDATION_ROOT / (
    "h0_primary_split_v12r1_autonomy_recovery_execution_lock.json"
)
DESIGN_AUDIT_RECEIPT_PATH = V12R1_VALIDATION_ROOT / (
    "h0_primary_split_v12r1_autonomy_recovery_design_audit.json"
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

COLLECTION_CASES = tuple(
    {
        **{key: value for key, value in copy.deepcopy(case).items() if key != "destination"},
        "destination": (
            COLLECTION_ROOT / f"r{int(case['round_index'])}" / str(case["case_id"])
        ).as_posix(),
    }
    for case in _v12.COLLECTION_CASES
)
FINAL_CASES = tuple(
    {
        **{key: value for key, value in copy.deepcopy(case).items() if key != "destination"},
        "destination": (FINAL_ROOT / str(case["case_id"])).as_posix(),
    }
    for case in _v12.FINAL_CASES
)
FINAL_CASE_IDS = tuple(str(case["case_id"]) for case in FINAL_CASES)


# The journal uses CSR-style offsets: boundary b owns event rows
# offsets[b]:offsets[b+1].  There are B=N+1 boundaries and therefore N+2
# offsets.  Fixed-width Unicode and uint8 byte arrays keep allow_pickle=False.
PROBE_REPLAY_SCHEMA = {
    "schema_id": "H0_V12R1_OBSERVER_REPLAY_BOUNDARIES_V2",
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
    index for index in range(EXPECTED_ACTOR_FEATURES)
    if index not in TEACHER_MUTABLE_ACTOR_COLUMNS
)

P0_REPRODUCTION_TOLERANCE = {
    "absolute": 1.0e-9,
    "relative": 1.0e-9,
    "rule": "ABS_OBSERVED_MINUS_REFERENCE_LE_ABS_PLUS_REL_TIMES_ABS_REFERENCE",
    "applies_to": ["rmse", "max_abs_error", "reset_max_abs_error"],
    "reference": "SOLE_V12R1_DESIGN_AUDIT_P0_IN_MEMORY_FIT",
    "retry_or_retuning": False,
}

FIT = {
    **copy.deepcopy(_v12.FIT),
    "fit_contract_id": FIT_CONTRACT_ID,
    "p0_reproduction_tolerance": copy.deepcopy(P0_REPRODUCTION_TOLERANCE),
}

AUTHORITY = {
    **copy.deepcopy(_v12.AUTHORITY),
    "authority_text": "esegui",
    "authority_scope": "V12R1_CORRECTIVE_DESIGN_AND_PROTOCOL_FREEZE_ONLY",
}

SOURCE_RELATIVE_PATHS = {
    "contract": (
        "Trajectory Generator/baseline_MLP/validation/"
        "h0_primary_split_v12r1_autonomy_recovery_contract.py"
    ),
    "freeze": (
        "Trajectory Generator/baseline_MLP/validation/"
        "freeze_h0_primary_split_v12r1_autonomy_recovery.py"
    ),
    "contract_tests": (
        "Trajectory Generator/baseline_MLP/validation/"
        "test_h0_primary_split_v12r1_autonomy_recovery_contract.py"
    ),
    "freeze_tests": (
        "Trajectory Generator/baseline_MLP/validation/"
        "test_freeze_h0_primary_split_v12r1_autonomy_recovery.py"
    ),
    "line_endings": ".gitattributes",
    "line_endings_r1": (
        "Trajectory Generator/baseline_MLP/validation/.gitattributes"
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
AUTHORITATIVE_SHA256 = {
    **copy.deepcopy(_v12.V11_AUTHORITATIVE_SHA256),
    "v12_protocol_freeze": V12_PROTOCOL_FREEZE_SHA256,
}

FUTURE_EXECUTION_SOURCES_REQUIRED = (
    "v12r1_recovery_weighted_fitter",
    "v12r1_pure_probe_observer_labeler",
    "v12r1_design_audit_runner",
    "v12r1_pipeline_runner",
    "v12r1_execution_tests",
)
FUTURE_EXECUTION_SOURCE_RELATIVE_PATHS = {
    "v12r1_recovery_weighted_fitter": (
        "Trajectory Generator/baseline_MLP/validation/"
        "h0_primary_split_v12r1_recovery_weighted_fitter.py"
    ),
    "v12r1_pure_probe_observer_labeler": (
        "Trajectory Generator/baseline_MLP/validation/"
        "h0_primary_split_v12r1_pure_probe_observer_labeler.py"
    ),
    "v12r1_design_audit_runner": (
        "Trajectory Generator/baseline_MLP/validation/"
        "run_h0_primary_split_v12r1_design_audit.py"
    ),
    "v12r1_pipeline_runner": (
        "Trajectory Generator/baseline_MLP/validation/"
        "run_h0_primary_split_v12r1_autonomy_recovery.py"
    ),
    "v12r1_execution_tests": (
        "Trajectory Generator/baseline_MLP/validation/"
        "test_h0_primary_split_v12r1_execution.py"
    ),
}


def _r1_status(value: str) -> str:
    return value.replace("V12", "V12R1")


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
    globals()[_status_name] = _r1_status(getattr(_v12, _status_name))

DESIGN_AUDIT_PASS_STATUS = "PASS_H0_PRIMARY_SPLIT_V12R1_DESIGN_AUDIT"
DESIGN_AUDIT_FAIL_STATUS = "FAIL_H0_PRIMARY_SPLIT_V12R1_DESIGN_AUDIT"
EXECUTION_LOCK_PASS_STATUS = "PASS_H0_PRIMARY_SPLIT_V12R1_EXECUTION_LOCK"
EXECUTION_LOCK_FAIL_STATUS = "FAIL_H0_PRIMARY_SPLIT_V12R1_EXECUTION_LOCK"


# Rebind every inherited pure function after all V12R1 globals are defined.
for _name, _value in vars(_v12).items():
    if isinstance(_value, types.FunctionType) and _value.__module__ == _v12.__name__:
        _function = types.FunctionType(
            _rewrite_code_strings(_value.__code__),
            globals(),
            name=_value.__name__,
            argdefs=_value.__defaults__,
            closure=_value.__closure__,
        )
        _function.__kwdefaults__ = copy.deepcopy(_value.__kwdefaults__)
        _function.__annotations__ = copy.deepcopy(_value.__annotations__)
        _function.__doc__ = _value.__doc__
        globals()[_name] = _function


def replay_schema_gate(value: Any) -> dict[str, Any]:
    """Validate the exact self-contained V12R1 replay declaration."""

    schema = value if isinstance(value, Mapping) else {}
    journal = schema.get("left_event_journal_arrays")
    config = schema.get("legacy_fsm_config_arrays")
    checks = {
        "exact_schema": schema == PROBE_REPLAY_SCHEMA,
        "teacher_absent_during_probe": schema.get(
            "probe_teacher_or_h0_load_allowed"
        ) is False,
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
    monotonic = offsets_shape and offsets[0] == 0 and all(
        left <= right for left, right in zip(offsets, offsets[1:])
    )
    event_count = offsets[-1] if monotonic else -1

    def _length(name: str) -> int:
        try:
            return len(payload[name])
        except (KeyError, TypeError):
            return -1

    event_lengths = {name: _length(name) for name in event_names if name != "legacy_left_event_boundary_offsets"}
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
        ("legacy_left_event_cycle_duration_present", "legacy_left_event_cycle_duration_s", 0.0),
        ("legacy_left_event_contact_duration_present", "legacy_left_event_contact_duration_s", 0.0),
        ("legacy_left_event_startup_contact_present", "legacy_left_event_startup_contact", False),
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
        "left_only": event_count >= 0
        and all(str(value) == "left" for value in sides),
        "event_types": event_count >= 0
        and all(str(value) in {"heel_strike", "toe_off"} for value in event_types),
        "causal_timestamps": times_causal,
        "delivered_at_owning_boundary": owning_boundaries,
        "strictly_increasing_onsets": event_count >= 0
        and all(float(onset[index]) < float(onset[index + 1]) for index in range(max(0, event_count - 1))),
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


_inherited_fit_gate = fit_gate


def fit_gate(summary: Mapping[str, Any], *, stage: str) -> dict[str, Any]:
    """Extend the V12 gate with a cryptographically bindable P0 comparison."""

    result = _inherited_fit_gate(summary, stage=stage)
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
        f"PURE_PROBE_{stage.upper()}" if passed else "STOP_V12R1_FIT_FAILURE"
    )
    return result


def _execution_source_records_match(value: Any) -> bool:
    if not isinstance(value, Mapping) or set(value) != set(
        FUTURE_EXECUTION_SOURCES_REQUIRED
    ):
        return False
    return all(
        artifact_record_matches(value[name], FUTURE_EXECUTION_SOURCE_RELATIVE_PATHS[name])
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
        "fit_design": payload.get("fit_design") == FIT,
        "replay_schema": replay_schema_gate(payload.get("probe_replay_schema"))[
            "passed"
        ] is True,
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
        ) and payload.get("design_audit_gate_passed") is True,
        "execution_sources_exact": _execution_source_records_match(
            payload.get("execution_sources")
        ),
        "inherited_runtime_evidence_exact": isinstance(inherited, Mapping)
        and inherited.get("passed") is True
        and inherited.get("v11_execution_lock") == V11_EXECUTION_LOCK_ARTIFACT
        and inherited.get("source_count") == 66
        and inherited.get("input_count") == 54
        and inherited.get("sources_exact") is True
        and inherited.get("inputs_exact") is True,
        "execution_authority": isinstance(authority, Mapping)
        and set(authority)
        == {"authority_date", "authority_text", "authority_scope", "one_shot"}
        and authority.get("authority_date") == REVISION
        and authority.get("authority_text") == "esegui"
        and authority.get("authority_scope") == "V12R1_ONE_SHOT_EXECUTION"
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
    """Return the exhaustive V12R1 write manifest, including failure paths."""

    paths = dict(_inherited_declared_mutation_paths())
    paths["candidate_freeze_summary"] = FIT_ROOT / "candidate_freeze_summary.json"
    paths["candidate_freeze_gate"] = FIT_ROOT / "candidate_freeze_gate.json"
    for stage in FIT_STAGES:
        paths[f"probe_failure_{stage}"] = PROBE_ROOT / stage / "failure.json"
    for case in COLLECTION_CASES:
        prefix = f"collect_r{case['round_index']}_{case['case_id']}"
        paths[f"{prefix}_failure"] = (
            PurePosixPath(case["destination"]) / "failure.json"
        )
    for case in FINAL_CASES:
        paths[f"final_{case['case_id']}_failure"] = (
            PurePosixPath(case["destination"]) / "failure.json"
        )
    return paths


__all__ = sorted(
    set(getattr(_v12, "__all__", ()))
    | {
        "AUTHORITATIVE_SHA256",
        "DESIGN_AUDIT_ID",
        "DESIGN_AUDIT_PASS_STATUS",
        "DESIGN_AUDIT_FAIL_STATUS",
        "EXECUTION_LOCK_PASS_STATUS",
        "EXECUTION_LOCK_FAIL_STATUS",
        "FUTURE_EXECUTION_SOURCE_RELATIVE_PATHS",
        "P0_REPRODUCTION_TOLERANCE",
        "TEACHER_MUTABLE_ACTOR_COLUMNS",
        "TEACHER_IMMUTABLE_ACTOR_COLUMNS",
        "V12_PROTOCOL_FREEZE_PATH",
        "V12_PROTOCOL_FREEZE_SHA256",
        "V11_EXECUTION_LOCK_ARTIFACT",
        "design_audit_gate",
        "execution_lock_gate",
        "p0_reproduction_gate",
        "replay_event_topology_gate",
        "replay_schema_gate",
    }
)
