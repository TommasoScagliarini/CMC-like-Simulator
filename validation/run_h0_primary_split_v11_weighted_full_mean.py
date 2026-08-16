"""Run the V11 reset-weighted full-mean safe-DAgger pipeline.

The rollout/orchestration implementation is reused from the immutable V10S
runner only after its exact bytes have been content-pinned.  Two imports are
then redirected explicitly to the V11 contract and fitter, and protocol-facing
``V10S`` labels are promoted to ``V11``.  The lowercase V10S blend helper is
intentionally retained: V11 changes only the preregistered fit objective, not
the safe blend/latch or final pure-V26 rollout semantics.

The wrapper also binds the terminal V10S failure and the offline V11 design
audit into both preflight and execution lock.  It never creates the design
audit, never writes into the V10S run, and preserves the inherited one-shot,
no-retry stage ordering.
"""

# ruff: noqa: F821 -- inherited names are defined by the content-pinned exec.

from __future__ import annotations

import hashlib as _bootstrap_hashlib
from pathlib import Path as _BootstrapPath


_FROZEN_RUNNER_RELATIVE_PATH = (
    "validation/run_h0_primary_split_v10s_safe_dagger.py"
)
_FROZEN_RUNNER_SHA256 = (
    "4240afdb37b99734f8dbe80b67c9dbef491a43ff65d6bbc78cab2f15a3300faf"
)
_WRAPPER_PATH = _BootstrapPath(__file__).resolve()
_BOOTSTRAP_REPO_ROOT = _WRAPPER_PATH.parents[1]
_FROZEN_RUNNER_PATH = (
    _BOOTSTRAP_REPO_ROOT / _FROZEN_RUNNER_RELATIVE_PATH
).resolve()
_frozen_runner_bytes = _FROZEN_RUNNER_PATH.read_bytes()
_observed_frozen_runner_sha256 = _bootstrap_hashlib.sha256(
    _frozen_runner_bytes
).hexdigest()
if _observed_frozen_runner_sha256 != _FROZEN_RUNNER_SHA256:
    raise RuntimeError(
        "refusing V11 reuse: frozen runner SHA-256 drifted "
        f"({_observed_frozen_runner_sha256} != {_FROZEN_RUNNER_SHA256})"
    )

try:
    _frozen_runner_source = _frozen_runner_bytes.decode("utf-8")
except UnicodeDecodeError as _decode_error:
    raise RuntimeError("frozen runner is not canonical UTF-8") from _decode_error

_EXPLICIT_SOURCE_SUBSTITUTIONS = (
    (
        "import h0_primary_split_v10s_safe_dagger_contract as contract  # noqa: E402",
        "import h0_primary_split_v11_weighted_full_mean_contract as contract  # noqa: E402",
    ),
    (
        "import h0_primary_split_v10s_fit as fit_engine  # noqa: E402",
        "import h0_primary_split_v11_weighted_fit as fit_engine  # noqa: E402",
    ),
    (
        '        "status": "H0_PRIMARY_SPLIT_V10S_P3_CANDIDATE_FREEZE_UNGATED",\n'
        '        "protocol_id": contract.PROTOCOL_ID,\n'
        '        "candidate_fit_stage": "p3",\n',
        '        "status": "H0_PRIMARY_SPLIT_V10S_P3_CANDIDATE_FREEZE_UNGATED",\n'
        '        "protocol_id": contract.PROTOCOL_ID,\n'
        '        "fit_contract_id": p3_summary.get("fit_contract_id"),\n'
        '        "design_audit_id": p3_summary.get("design_audit_id"),\n'
        '        "design_audit_passed": p3_summary.get(\n'
        '            "design_audit_passed"\n'
        '        ),\n'
        '        "design_audit_receipt": copy.deepcopy(\n'
        '            p3_summary.get("design_audit_receipt")\n'
        '        ),\n'
        '        "v10s_terminal_failure_id": p3_summary.get(\n'
        '            "v10s_terminal_failure_id"\n'
        '        ),\n'
        '        "v10s_terminal_failure_passed": p3_summary.get(\n'
        '            "v10s_terminal_failure_passed"\n'
        '        ),\n'
        '        "source_checkpoint_scope": p3_summary.get(\n'
        '            "source_checkpoint_scope"\n'
        '        ),\n'
        '        "critic_present": p3_summary.get("critic_present"),\n'
        '        "critic_parameter_count": p3_summary.get(\n'
        '            "critic_parameter_count"\n'
        '        ),\n'
        '        "normalization_folded_into_first_layer": p3_summary.get(\n'
        '            "normalization_folded_into_first_layer"\n'
        '        ) is True,\n'
        '        "runtime_normalization_wrapper_present": p3_summary.get(\n'
        '            "runtime_normalization_wrapper_present"\n'
        '        ),\n'
        '        "prescribed_clock_present": p3_summary.get(\n'
        '            "prescribed_clock_present"\n'
        '        ),\n'
        '        "disabled_clock_columns_0_1_bit_zero": p3_summary.get(\n'
        '            "disabled_clock_columns_bit_zero_after_save_reload"\n'
        '        ) is True,\n'
        '        "candidate_fit_stage": "p3",\n',
    ),
    (
        '        "safety_latch_activation_count": 0,\n'
        '    }\n'
        '    partial = {\n',
        '        "safety_latch_activation_count": 0,\n'
        '        "fit_contract_id": freeze.get("fit_contract_id"),\n'
        '        "design_audit_id": freeze.get("design_audit_id"),\n'
        '        "design_audit_passed": freeze.get("design_audit_passed"),\n'
        '        "design_audit_receipt": copy.deepcopy(\n'
        '            freeze.get("design_audit_receipt")\n'
        '        ),\n'
        '        "source_checkpoint_scope": freeze.get(\n'
        '            "source_checkpoint_scope"\n'
        '        ),\n'
        '        "critic_present": freeze.get("critic_present"),\n'
        '        "critic_parameter_count": freeze.get(\n'
        '            "critic_parameter_count"\n'
        '        ),\n'
        '        "normalization_folded_into_first_layer": freeze.get(\n'
        '            "normalization_folded_into_first_layer"\n'
        '        ),\n'
        '        "runtime_normalization_wrapper_present": freeze.get(\n'
        '            "runtime_normalization_wrapper_present"\n'
        '        ),\n'
        '        "prescribed_clock_present": freeze.get(\n'
        '            "prescribed_clock_present"\n'
        '        ),\n'
        '        "disabled_clock_columns_0_1_bit_zero": freeze.get(\n'
        '            "disabled_clock_columns_0_1_bit_zero"\n'
        '        ),\n'
        '    }\n'
        '    partial = {\n',
    ),
    (
        '        "status": "H0_PRIMARY_SPLIT_V10S_FINAL_DEVELOPMENT_UNGATED",\n'
        '        "protocol_id": contract.PROTOCOL_ID,\n'
        '        "candidate_id": freeze["candidate_id"],\n',
        '        "status": "H0_PRIMARY_SPLIT_V10S_FINAL_DEVELOPMENT_UNGATED",\n'
        '        "protocol_id": contract.PROTOCOL_ID,\n'
        '        "fit_contract_id": freeze.get("fit_contract_id"),\n'
        '        "design_audit_id": freeze.get("design_audit_id"),\n'
        '        "design_audit_passed": freeze.get("design_audit_passed"),\n'
        '        "design_audit_receipt": copy.deepcopy(\n'
        '            freeze.get("design_audit_receipt")\n'
        '        ),\n'
        '        "v10s_terminal_failure_id": freeze.get(\n'
        '            "v10s_terminal_failure_id"\n'
        '        ),\n'
        '        "v10s_terminal_failure_passed": freeze.get(\n'
        '            "v10s_terminal_failure_passed"\n'
        '        ),\n'
        '        "source_checkpoint_scope": freeze.get(\n'
        '            "source_checkpoint_scope"\n'
        '        ),\n'
        '        "critic_present": freeze.get("critic_present"),\n'
        '        "critic_parameter_count": freeze.get(\n'
        '            "critic_parameter_count"\n'
        '        ),\n'
        '        "normalization_folded_into_first_layer": freeze.get(\n'
        '            "normalization_folded_into_first_layer"\n'
        '        ),\n'
        '        "runtime_normalization_wrapper_present": freeze.get(\n'
        '            "runtime_normalization_wrapper_present"\n'
        '        ),\n'
        '        "prescribed_clock_present": freeze.get(\n'
        '            "prescribed_clock_present"\n'
        '        ),\n'
        '        "disabled_clock_columns_0_1_bit_zero": freeze.get(\n'
        '            "disabled_clock_columns_0_1_bit_zero"\n'
        '        ),\n'
        '        "candidate_id": freeze["candidate_id"],\n',
    ),
    (
        '        "status": contract.FINAL_ROLLOUT_PASS_STATUS,\n'
        '        "passed": True,\n'
        '        "protocol_id": contract.PROTOCOL_ID,\n'
        '        "stage_id": stage_id,\n'
        '        "case_id": case_id,\n',
        '        "status": contract.FINAL_ROLLOUT_PASS_STATUS,\n'
        '        "passed": True,\n'
        '        "protocol_id": contract.PROTOCOL_ID,\n'
        '        "fit_contract_id": freeze.get("fit_contract_id"),\n'
        '        "design_audit_id": freeze.get("design_audit_id"),\n'
        '        "design_audit_receipt": copy.deepcopy(\n'
        '            freeze.get("design_audit_receipt")\n'
        '        ),\n'
        '        "source_checkpoint_scope": freeze.get(\n'
        '            "source_checkpoint_scope"\n'
        '        ),\n'
        '        "critic_present": freeze.get("critic_present"),\n'
        '        "critic_parameter_count": freeze.get(\n'
        '            "critic_parameter_count"\n'
        '        ),\n'
        '        "normalization_folded_into_first_layer": freeze.get(\n'
        '            "normalization_folded_into_first_layer"\n'
        '        ),\n'
        '        "runtime_normalization_wrapper_present": freeze.get(\n'
        '            "runtime_normalization_wrapper_present"\n'
        '        ),\n'
        '        "prescribed_clock_present": freeze.get(\n'
        '            "prescribed_clock_present"\n'
        '        ),\n'
        '        "disabled_clock_columns_0_1_bit_zero": freeze.get(\n'
        '            "disabled_clock_columns_0_1_bit_zero"\n'
        '        ),\n'
        '        "stage_id": stage_id,\n'
        '        "case_id": case_id,\n',
    ),
    (
        '        "status": contract.FINAL_DEVELOPMENT_PASS_STATUS,\n'
        '        "passed": True,\n'
        '        "protocol_id": contract.PROTOCOL_ID,\n'
        '        "stage_id": stage_id,\n',
        '        "status": contract.FINAL_DEVELOPMENT_PASS_STATUS,\n'
        '        "passed": True,\n'
        '        "protocol_id": contract.PROTOCOL_ID,\n'
        '        "fit_contract_id": freeze.get("fit_contract_id"),\n'
        '        "design_audit_id": freeze.get("design_audit_id"),\n'
        '        "design_audit_receipt": copy.deepcopy(\n'
        '            freeze.get("design_audit_receipt")\n'
        '        ),\n'
        '        "source_checkpoint_scope": freeze.get(\n'
        '            "source_checkpoint_scope"\n'
        '        ),\n'
        '        "critic_present": freeze.get("critic_present"),\n'
        '        "critic_parameter_count": freeze.get(\n'
        '            "critic_parameter_count"\n'
        '        ),\n'
        '        "normalization_folded_into_first_layer": freeze.get(\n'
        '            "normalization_folded_into_first_layer"\n'
        '        ),\n'
        '        "runtime_normalization_wrapper_present": freeze.get(\n'
        '            "runtime_normalization_wrapper_present"\n'
        '        ),\n'
        '        "prescribed_clock_present": freeze.get(\n'
        '            "prescribed_clock_present"\n'
        '        ),\n'
        '        "disabled_clock_columns_0_1_bit_zero": freeze.get(\n'
        '            "disabled_clock_columns_0_1_bit_zero"\n'
        '        ),\n'
        '        "stage_id": stage_id,\n',
    ),
    (
        'if __name__ == "__main__":\n    raise SystemExit(main())\n',
        "# V11 wrapper owns the sole main-entry dispatch.\n",
    ),
)
_transformed_runner_source = _frozen_runner_source
for _old_source, _new_source in _EXPLICIT_SOURCE_SUBSTITUTIONS:
    _occurrences = _transformed_runner_source.count(_old_source)
    if _occurrences != 1:
        raise RuntimeError(
            "refusing V11 reuse: explicit source substitution cardinality "
            f"is {_occurrences}, expected 1: {_old_source!r}"
        )
    _transformed_runner_source = _transformed_runner_source.replace(
        _old_source, _new_source, 1
    )

_uppercase_label_count = _transformed_runner_source.count("V10S")
if _uppercase_label_count < 1:
    raise RuntimeError("refusing V11 reuse: frozen runner has no V10S labels")
_transformed_runner_source = _transformed_runner_source.replace("V10S", "V11")
if "V10S" in _transformed_runner_source:
    raise RuntimeError("refusing V11 reuse: an uppercase V10S label survived")
if "import h0_primary_split_v10s_blend as safe_dagger" not in (
    _transformed_runner_source
):
    raise RuntimeError("refusing V11 reuse: safe blend helper was not preserved")

# Compile against this wrapper path.  Consequently the inherited worker
# command re-enters this V11 file rather than the read-only V10S source.
_TRANSFORMED_RUNNER_SOURCE = _transformed_runner_source
exec(
    compile(_TRANSFORMED_RUNNER_SOURCE, str(_WRAPPER_PATH), "exec"),
    globals(),
)


_INHERITED_BUILD_PREFLIGHT = build_preflight
_INHERITED_LOCK_PAYLOAD = _lock_payload
_INHERITED_PREEXECUTION_ABSENCE = _preexecution_absence

_READ_ONLY_V10S_RUN_ROOT = resolve_relative(
    "validation/h0_primary_grf_split_adaptation_runs/"
    "2026-08-07_h0_primary_split_v10s_v26_full_mean_safe_dagger"
)
_READ_ONLY_V10S_PREFLIGHT = resolve_relative(
    "validation/h0_primary_split_v10s_safe_dagger_preflight_receipt.json"
)
_READ_ONLY_V10S_LOCK = resolve_relative(
    "validation/h0_primary_split_v10s_safe_dagger_execution_lock.json"
)


def _lineage_path(name: str) -> Path:
    """Resolve one contract-declared V11 lineage input fail-closed."""

    if not hasattr(contract, name):
        raise V11SafeDaggerError(f"V11 contract lacks lineage path: {name}")
    return resolve_relative(getattr(contract, name))


def _v11_lineage_audit() -> dict[str, Any]:
    """Verify and bind the terminal V10S failure and V11 design audit."""

    ledger_path = _lineage_path("V10S_TERMINAL_LEDGER_PATH")
    p0_gate_path = _lineage_path("V10S_P0_GATE_PATH")
    p0_summary_path = _lineage_path("V10S_P0_SUMMARY_PATH")
    design_receipt_path = _lineage_path("DESIGN_AUDIT_RECEIPT_PATH")

    ledger = _mapping(ledger_path)
    p0_gate = _mapping(p0_gate_path)
    p0_summary = _mapping(p0_summary_path)
    design_receipt = _mapping(design_receipt_path)
    current_bindings = _design_audit_current_bindings()
    terminal_gate = dict(
        contract.v10s_terminal_failure_gate(ledger, p0_gate, p0_summary)
    )
    design_gate = dict(contract.design_audit_gate(design_receipt))
    current_binding_gate = dict(
        contract.design_audit_current_binding_gate(
            design_receipt, current_bindings
        )
    )
    if terminal_gate.get("passed") is not True:
        raise V11SafeDaggerError("V10S terminal failure evidence is not intact")
    if design_gate.get("passed") is not True:
        raise V11SafeDaggerError("V11 weighted-fit design audit is not PASS")
    if current_binding_gate.get("passed") is not True:
        raise V11SafeDaggerError("V11 design-audit source bindings drifted")
    return {
        "v10s_terminal_failure": {
            "ledger": _record(ledger_path),
            "p0_gate": _record(p0_gate_path),
            "p0_summary": _record(p0_summary_path),
            "gate": terminal_gate,
        },
        "v11_design_audit": {
            "receipt": _record(design_receipt_path),
            "gate": design_gate,
            "current_bindings": current_bindings,
            "current_binding_gate": current_binding_gate,
        },
    }


def _design_audit_current_bindings() -> dict[str, Any]:
    """Re-hash every source, H0 tree, and corpus consumed by the dry run."""

    source_paths = getattr(
        contract, "DESIGN_AUDIT_SOURCE_RELATIVE_PATHS", None
    )
    if not isinstance(source_paths, Mapping) or set(source_paths) != {
        "contract",
        "fitter",
        "audit_cli",
        "validator",
    }:
        raise V11SafeDaggerError("V11 design-audit source manifest drifted")
    return {
        "source_bindings": {
            name: _record(resolve_relative(path))
            for name, path in sorted(source_paths.items())
        },
        "source_h0": _tree_record(
            resolve_relative(contract.SOURCE_H0_MODULE_PATH)
        ),
        "corpus": _record(_lineage_path("V10S_P0_CORPUS_PATH")),
    }


def _mutation_paths() -> dict[str, Path]:
    """Enumerate every contract-controlled destination used by workers."""

    paths: dict[str, Path] = {
        "preflight": PREFLIGHT,
        "lock": LOCK,
        "run_root": RUN_ROOT,
        "pipeline_claim": PIPELINE_CLAIM,
        "pipeline_ledger": PIPELINE_LEDGER,
        "worker_claims_root": WORKER_CLAIMS_ROOT,
        "candidate_freeze": CANDIDATE_FREEZE,
        "final_receipt": FINAL_RECEIPT,
    }
    for _stage in contract.FIT_STAGES:
        paths[f"fit_root_{_stage}"] = resolve_relative(
            contract.FIT_ROOTS[_stage]
        )
        paths[f"module_{_stage}"] = resolve_relative(
            contract.MODULE_PATHS[_stage]
        )
        paths[f"fit_receipt_{_stage}"] = resolve_relative(
            contract.FIT_RECEIPT_PATHS[_stage]
        )
    for _round_index in (1, 2, 3):
        for _case_id in contract.COLLECTION_CASE_IDS:
            _case = contract.canonical_collection_case(_case_id, _round_index)
            paths[f"collection_r{_round_index}_{_case_id}"] = resolve_relative(
                _case["destination"]
            )
    for _case_id in contract.FINAL_CASE_IDS:
        _case = contract.canonical_final_case(_case_id)
        paths[f"final_{_case_id}"] = resolve_relative(_case["destination"])
    return paths


def _v11_path_isolation_audit() -> dict[str, Any]:
    """Prove that no V11 mutation can target frozen V10S artifacts."""

    declared = _mutation_paths()
    forbidden_exact = {_READ_ONLY_V10S_PREFLIGHT, _READ_ONLY_V10S_LOCK}
    violations: list[dict[str, str]] = []
    for name, path in declared.items():
        resolved = path.resolve()
        inside_v10s = resolved == _READ_ONLY_V10S_RUN_ROOT or (
            _READ_ONLY_V10S_RUN_ROOT in resolved.parents
        )
        if inside_v10s or resolved in forbidden_exact:
            violations.append({"name": name, "path": str(resolved)})
    globals_match_contract = {
        "run_root": RUN_ROOT == resolve_relative(contract.RUN_ROOT),
        "preflight": PREFLIGHT == resolve_relative(contract.PREFLIGHT_PATH),
        "lock": LOCK == resolve_relative(contract.LOCK_PATH),
        "pipeline_claim": PIPELINE_CLAIM
        == resolve_relative(contract.PIPELINE_CLAIM_PATH),
        "pipeline_ledger": PIPELINE_LEDGER
        == resolve_relative(contract.PIPELINE_LEDGER_PATH),
        "worker_claims_root": WORKER_CLAIMS_ROOT
        == resolve_relative(contract.WORKER_CLAIMS_ROOT),
        "candidate_freeze": CANDIDATE_FREEZE
        == resolve_relative(contract.CANDIDATE_FREEZE_PATH),
        "final_receipt": FINAL_RECEIPT
        == resolve_relative(contract.FINAL_DEVELOPMENT_RECEIPT_PATH),
    }
    return {
        "passed": not violations and all(globals_match_contract.values()),
        "frozen_v10s_run_root": _READ_ONLY_V10S_RUN_ROOT.relative_to(
            REPO_ROOT
        ).as_posix(),
        "declared_mutation_paths": {
            name: path.relative_to(REPO_ROOT).as_posix()
            for name, path in sorted(declared.items())
        },
        "globals_match_contract": globals_match_contract,
        "violations": violations,
    }


def _frozen_runner_binding() -> dict[str, Any]:
    return {
        "reuse_mode": "CONTENT_PINNED_EXPLICIT_IMPORT_SUBSTITUTION",
        "source": _record(_FROZEN_RUNNER_PATH),
        "expected_sha256": _FROZEN_RUNNER_SHA256,
        "observed_sha256": _observed_frozen_runner_sha256,
        "uppercase_v10s_labels_replaced": _uppercase_label_count,
        "v11_worker_entrypoint": _WRAPPER_PATH.relative_to(REPO_ROOT).as_posix(),
        "lowercase_v10s_blend_helper_preserved": True,
    }


def build_preflight(*, require_unoccupied: bool = True) -> dict[str, Any]:
    """Build V11 preflight with immutable V10S/design lineage evidence."""

    base = dict(_INHERITED_BUILD_PREFLIGHT(require_unoccupied=False))
    lineage = _v11_lineage_audit()
    isolation = _v11_path_isolation_audit()
    checks = dict(base.get("checks", {}))
    checks.update(
        {
            "frozen_v10s_runner_content_pin": (
                _observed_frozen_runner_sha256 == _FROZEN_RUNNER_SHA256
            ),
            "v10s_terminal_failure_bound": lineage[
                "v10s_terminal_failure"
            ]["gate"].get("passed")
            is True,
            "v11_design_audit_pass": lineage["v11_design_audit"]["gate"].get(
                "passed"
            )
            is True,
            "v11_design_audit_current_bindings": lineage[
                "v11_design_audit"
            ]["current_binding_gate"].get("passed")
            is True,
            "v11_write_paths_isolated": isolation.get("passed") is True,
        }
    )
    passed = all(value is True for value in checks.values())
    base.update(
        {
            "status": (
                contract.PREFLIGHT_PASS_STATUS
                if passed
                else "FAIL_H0_PRIMARY_SPLIT_V11_WEIGHTED_FULL_MEAN_PREFLIGHT"
            ),
            "passed": passed,
            "checks": checks,
            "v11_lineage": lineage,
            "frozen_runner_reuse": _frozen_runner_binding(),
            "write_path_isolation": isolation,
            "next_stage": (
                "FREEZE_V11_WEIGHTED_FULL_MEAN_SAFE_DAGGER_PIPELINE"
                if passed
                else "STOP"
            ),
        }
    )
    if require_unoccupied and not passed:
        failed = [name for name, value in checks.items() if value is not True]
        raise V11SafeDaggerError(f"V11 preflight failed: {failed}")
    return base


def _lock_payload(preflight: Mapping[str, Any]) -> dict[str, Any]:
    """Extend the inherited lock with recomputed V11 lineage bindings."""

    lineage = _v11_lineage_audit()
    isolation = _v11_path_isolation_audit()
    if isolation.get("passed") is not True:
        raise V11SafeDaggerError(
            "V11 write-path isolation drifted before execution lock"
        )
    payload = dict(_INHERITED_LOCK_PAYLOAD(preflight))
    payload.update(
        {
            "v11_lineage": lineage,
            "frozen_runner_reuse": _frozen_runner_binding(),
            "write_path_isolation": isolation,
            "next_stage": "EXECUTE_V11_WEIGHTED_FULL_MEAN_SAFE_DAGGER_ONCE",
        }
    )
    return payload


def _preexecution_absence() -> None:
    """Revalidate immutable lineage and path isolation before minting a token."""

    _INHERITED_PREEXECUTION_ABSENCE()
    _v11_lineage_audit()
    isolation = _v11_path_isolation_audit()
    if isolation.get("passed") is not True:
        raise V11SafeDaggerError("V11 write-path isolation drifted")


if __name__ == "__main__":
    raise SystemExit(main())
