"""Build and verify the additive, non-executable V12R1 protocol freeze.

The command re-audits the immutable V12/V11 evidence and may publish exactly
one no-clobber V12R1 protocol JSON.  It cannot fit an actor, load H0 for probe
inference, label a probe, reset/step an environment, or mint an execution
claim.  A distinct execution lock must pass the pure V12R1 lock gate first.
"""

from __future__ import annotations

import copy
import types
from pathlib import Path
from typing import Any, Mapping, Sequence

try:
    from validation import freeze_h0_primary_split_v12_autonomy_recovery as _v12_freeze
except (ImportError, ModuleNotFoundError):
    import freeze_h0_primary_split_v12_autonomy_recovery as _v12_freeze

try:
    from validation import h0_primary_split_v12r1_autonomy_recovery_contract as contract
except (ImportError, ModuleNotFoundError):
    import h0_primary_split_v12r1_autonomy_recovery_contract as contract


def _rewrite_code_strings(code: types.CodeType) -> types.CodeType:
    constants: list[Any] = []
    for value in code.co_consts:
        if isinstance(value, types.CodeType):
            constants.append(_rewrite_code_strings(value))
        elif isinstance(value, str):
            constants.append(value.replace("V12", "V12R1").replace("v12", "v12r1"))
        else:
            constants.append(value)
    return code.replace(co_consts=tuple(constants))


for _name, _value in vars(_v12_freeze).items():
    if (
        not _name.startswith("__")
        and _name != "contract"
        and not isinstance(_value, types.FunctionType)
    ):
        globals()[_name] = _value


class V12R1ProtocolFreezeError(RuntimeError):
    """Raised when the corrected design-only protocol cannot remain exact."""


# Inherited code refers to the old class name; bind it to the new public type.
V12ProtocolFreezeError = V12R1ProtocolFreezeError
V12_VALIDATION_ROOT = Path(__file__).resolve().parent


for _name, _value in vars(_v12_freeze).items():
    if isinstance(_value, types.FunctionType) and _value.__module__ == _v12_freeze.__name__:
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


_inherited_path_isolation_audit = _path_isolation_audit


def _path_isolation_audit(
    declared_paths: Mapping[str, str | Path] | None = None,
) -> dict[str, Any]:
    """Extend V12 path isolation with real exclusive-temp filename budgets."""

    audit = dict(_inherited_path_isolation_audit(declared_paths))
    declared = audit["declared_mutation_paths"]
    file_temp_lengths = [
        len(path) + 14
        for path in declared.values()
        if Path(path).suffix
    ]
    step_temp_lengths = [
        len(path) + 1 + len("000001.json") + 14
        for name, path in declared.items()
        if "steps_root" in name
    ]
    maximum = max(file_temp_lengths + step_temp_lengths)
    checks = dict(audit["checks"])
    checks["exclusive_temp_relative_paths_short"] = maximum < 160
    audit["checks"] = checks
    audit["passed"] = all(checks.values())
    audit["exclusive_temp_filename_overhead"] = 14
    audit["maximum_temp_relative_path_length"] = maximum
    audit["maximum_checkout_root_length_for_legacy_windows_with_temp"] = 258 - maximum
    audit["maximum_checkout_root_length_for_preferred_windows_with_temp"] = 238 - maximum
    return audit


def _authoritative_hash_gate(
    sources: Mapping[str, Mapping[str, Any]],
    inputs: Mapping[str, Mapping[str, Any]],
) -> dict[str, Any]:
    observed: dict[str, str | None] = {}
    checks: dict[str, bool] = {}
    for name, expected in contract.AUTHORITATIVE_SHA256.items():
        record = sources.get(name) if name in sources else inputs.get(name)
        value = record.get("sha256") if isinstance(record, Mapping) else None
        observed[name] = value if isinstance(value, str) else None
        checks[name] = value == expected
    return {
        "passed": bool(checks) and all(checks.values()),
        "checks": checks,
        "expected_sha256": copy.deepcopy(contract.AUTHORITATIVE_SHA256),
        "observed_sha256": observed,
    }


def _authority_gate() -> dict[str, Any]:
    false_flags = (
        "pipeline_execution_authorized",
        "actor_fit_execution_authorized",
        "environment_reset_authorized",
        "environment_step_authorized",
        "design_audit_fit_authorized",
        "offline_teacher_labeling_authorized",
        "critic_updates_authorized",
        "ppo_updates_authorized",
        "protected_trial_access_authorized",
        "reserve_trial_access_authorized",
        "runtime_promotion_authorized",
        "physical_gate_relaxation_authorized",
        "primary_grf_modification_authorized",
        "detector_or_fsm_modification_authorized",
        "sea_semantic_modification_authorized",
        "retry_authorized",
        "sweep_authorized",
        "rescue_authorized",
    )
    expected_keys = {
        "authority_date",
        "authority_text",
        "authority_scope",
        "design_and_freeze_authorized",
        *false_flags,
    }
    checks = {
        "exact_schema": set(contract.AUTHORITY) == expected_keys,
        "identity": contract.AUTHORITY.get("authority_date") == "2026-08-09"
        and contract.AUTHORITY.get("authority_text") == "esegui"
        and contract.AUTHORITY.get("authority_scope")
        == "V12R1_CORRECTIVE_DESIGN_AND_PROTOCOL_FREEZE_ONLY",
        "design_freeze_only_true_flag": contract.AUTHORITY.get(
            "design_and_freeze_authorized"
        ) is True,
        "all_execution_and_relaxation_flags_false": all(
            contract.AUTHORITY.get(name) is False for name in false_flags
        ),
        "boolean_types_exact": type(
            contract.AUTHORITY.get("design_and_freeze_authorized")
        ) is bool
        and all(type(contract.AUTHORITY.get(name)) is bool for name in false_flags),
    }
    return {"passed": all(checks.values()), "checks": checks, "false_flags": list(false_flags)}


def _v12_protocol_freeze_gate() -> dict[str, Any]:
    """Verify the parent freeze both cryptographically and semantically."""

    path = resolve_relative(contract.V12_PROTOCOL_FREEZE_PATH)
    record = _record(contract.V12_PROTOCOL_FREEZE_PATH)
    try:
        payload = _v12_freeze.verify_protocol_freeze()
        verifier_passed = payload.get("passed") is True
    except Exception as exc:  # pragma: no cover - surfaced in result
        payload = None
        verifier_passed = False
        error = f"{type(exc).__name__}: {exc}"
    else:
        error = None
    checks = {
        "regular_file": path.is_file() and not path.is_symlink(),
        "artifact_exact": record
        == {
            "path": contract.V12_PROTOCOL_FREEZE_PATH.as_posix(),
            "sha256": contract.V12_PROTOCOL_FREEZE_SHA256,
            "size_bytes": contract.V12_PROTOCOL_FREEZE_SIZE_BYTES,
        },
        "parent_verifier": verifier_passed,
        "parent_schema": isinstance(payload, Mapping)
        and payload.get("schema_version") == 120,
        "parent_identity": isinstance(payload, Mapping)
        and payload.get("protocol_id")
        == "AB06_H0_PRIMARY_SPLIT_V12_V26_AUTONOMY_RECOVERY",
        "parent_design_only": isinstance(payload, Mapping)
        and payload.get("execution_lock") is None
        and payload.get("environment_reset_calls") == 0
        and payload.get("environment_step_calls") == 0,
    }
    return {
        "passed": all(checks.values()),
        "checks": checks,
        "artifact": record,
        "verifier_error": error,
    }


def _assemble_protocol_freeze(occupancy: Mapping[str, bool]) -> dict[str, Any]:
    expected_occupancy_keys = {
        "protocol_freeze_unoccupied",
        "execution_lock_absent",
        "design_audit_absent",
        "run_root_absent",
    }
    if set(occupancy) != expected_occupancy_keys or not all(
        type(value) is bool for value in occupancy.values()
    ):
        raise V12R1ProtocolFreezeError("occupancy evidence schema drifted")

    v11_receipt_before = repository_artifact_snapshot(
        contract.prior.DESIGN_AUDIT_RECEIPT_PATH
    )
    sources = _source_records()
    inputs = _input_records()
    hashes = _authoritative_hash_gate(sources, inputs)
    authority = _authority_gate()
    topology = _stage_topology_gate()
    replay = contract.replay_schema_gate(contract.PROBE_REPLAY_SCHEMA)
    parent_v12 = _v12_protocol_freeze_gate()
    v11_fit_provenance = _v11_fit_and_freeze_provenance_audit()
    terminal = _v11_terminal_gate()
    collection = _v11_collection_recovery_audit()
    corpus = _v11_corpus_audit()
    pure_failure = _v11_pure_failure_coverage_audit()
    isolation = _path_isolation_audit()
    v11_receipt_after = repository_artifact_snapshot(
        contract.prior.DESIGN_AUDIT_RECEIPT_PATH
    )
    design_gate_names = (
        "fit_gate",
        "probe_integrity_gate",
        "pure_probe_gate",
        "observer_label_gate",
        "collection_data_gate",
        "latch_dependence_gate",
        "candidate_freeze_gate",
        "final_rollout_gate",
        "final_development_gate",
        "replay_schema_gate",
        "replay_event_topology_gate",
        "p0_reproduction_gate",
        "design_audit_gate",
        "execution_lock_gate",
    )
    checks = {
        "authority_design_freeze_only": authority["passed"] is True,
        "immutable_v12_protocol_freeze": parent_v12["passed"] is True,
        "stage_topology_exact": topology["passed"] is True,
        "corrected_teacher_columns_10_through_24": (
            contract.PROBE_REPLAY_SCHEMA.get("teacher_mutable_actor_columns")
            == list(range(10, 25))
        ),
        "self_contained_replay_schema": replay["passed"] is True,
        "p0_reproduction_tolerance_frozen": contract.P0_REPRODUCTION_TOLERANCE
        == {
            "absolute": 1.0e-9,
            "relative": 1.0e-9,
            "rule": "ABS_OBSERVED_MINUS_REFERENCE_LE_ABS_PLUS_REL_TIMES_ABS_REFERENCE",
            "applies_to": ["rmse", "max_abs_error", "reset_max_abs_error"],
            "reference": "SOLE_V12R1_DESIGN_AUDIT_P0_IN_MEMORY_FIT",
            "retry_or_retuning": False,
        },
        "pure_design_gates_present": all(
            callable(getattr(contract, name, None)) for name in design_gate_names
        ),
        "authoritative_hashes": hashes["passed"] is True,
        "v11_fit_and_freeze_provenance": v11_fit_provenance["passed"] is True,
        "v11_terminal": terminal["passed"] is True,
        "v11_collection_evidence": collection["passed"] is True,
        "v11_seed_corpus": corpus["passed"] is True,
        "v11_pure_failure_coverage": pure_failure["passed"] is True,
        "write_paths_isolated": isolation["passed"] is True,
        "v11_design_receipt_idempotent": v11_receipt_before == v11_receipt_after,
        "future_execution_sources_deferred_exact": (
            contract.FUTURE_EXECUTION_SOURCES_REQUIRED
            == tuple(contract.FUTURE_EXECUTION_SOURCE_RELATIVE_PATHS)
            and len(contract.FUTURE_EXECUTION_SOURCES_REQUIRED) == 5
        ),
        **dict(occupancy),
    }
    passed = all(checks.values())
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": (
            contract.PROTOCOL_FREEZE_PASS_STATUS
            if passed
            else contract.PROTOCOL_FREEZE_FAIL_STATUS
        ),
        "passed": passed,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "revision": contract.REVISION,
        "checks": checks,
        "authority": copy.deepcopy(contract.AUTHORITY),
        "authority_gate": authority,
        "immutable_v12_protocol_freeze": parent_v12,
        "stage_order": list(contract.STAGE_IDS),
        "stage_topology_gate": topology,
        "fit_design": copy.deepcopy(contract.FIT),
        "p0_reproduction_tolerance": copy.deepcopy(
            contract.P0_REPRODUCTION_TOLERANCE
        ),
        "recovery_weighting": copy.deepcopy(contract.RECOVERY_WEIGHTING),
        "coverage_weighting": copy.deepcopy(contract.COVERAGE_WEIGHTING),
        "latch_independence": copy.deepcopy(contract.LATCH_INDEPENDENCE),
        "probe_behavior": contract.PROBE_BEHAVIOR,
        "probe_replay_schema": copy.deepcopy(contract.PROBE_REPLAY_SCHEMA),
        "probe_replay_schema_gate": replay,
        "authoritative_hashes": hashes,
        "v11_fit_and_freeze_provenance_audit": v11_fit_provenance,
        "v11_terminal_evidence": terminal,
        "v11_collection_recovery_audit": collection,
        "v11_seed_corpus_audit": corpus,
        "v11_pure_failure_coverage_audit": pure_failure,
        "write_path_isolation": isolation,
        "sources": sources,
        "inputs": inputs,
        "v11_design_receipt_snapshot_before": v11_receipt_before,
        "v11_design_receipt_snapshot_after": v11_receipt_after,
        "future_execution_sources_required": list(
            contract.FUTURE_EXECUTION_SOURCES_REQUIRED
        ),
        "future_execution_source_paths": {
            name: contract.FUTURE_EXECUTION_SOURCE_RELATIVE_PATHS[name]
            for name in contract.FUTURE_EXECUTION_SOURCES_REQUIRED
        },
        "execution_lock": None,
        "pipeline_claim": None,
        "environment_reset_calls": 0,
        "environment_step_calls": 0,
        "offline_teacher_label_calls": 0,
        "actor_fit_executions": 0,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
        "next_stage": "REQUIRE_SEPARATE_V12R1_NO_CLOBBER_EXECUTION_LOCK",
    }


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    payload = prepare_protocol_freeze() if args.prepare else verify_protocol_freeze()
    print(f"{payload['status']}: {contract.PROTOCOL_FREEZE_PATH.as_posix()}")
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
