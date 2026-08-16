"""Build the five no-clobber V3 status-0 policy evidence receipts.

The historical diagnostic artifacts and their FAIL receipts are read-only.
This builder independently reclassifies every recorded solver attempt under
the explicitly authorized closed policy, proves replica bit-exactness, runs
the frozen focused tests, and only then publishes the five evidence receipts.
It performs no simulation, actor/critic update, PPO step, or holdout access.
"""

from __future__ import annotations

import hashlib
import json
import os
import subprocess
import sys
from pathlib import Path, PurePosixPath
from typing import Any, Mapping


REPO_ROOT = Path(__file__).resolve().parents[1]
VALIDATION_ROOT = REPO_ROOT / "validation"
if str(VALIDATION_ROOT) not in sys.path:
    sys.path.insert(0, str(VALIDATION_ROOT))

import compare_h0_primary_grf_split_v3_diagnostics as comparator  # noqa: E402
import freeze_h0_primary_grf_split_v3_execution as freezer  # noqa: E402
import h0_primary_grf_split_v3_freeze_contract as freeze_contract  # noqa: E402
import h0_v3_so_recovery_contract as so_recovery  # noqa: E402
import run_h0_primary_grf_split_v3_semantic_replay as runner  # noqa: E402


POLICY_ID = freeze_contract.VERIFIED_STATUS0_MAX_ITER_POLICY
FAILURE_RECEIPTS = {
    seed: VALIDATION_ROOT
    / f"h0_primary_grf_split_v3_diagnostic_seed{seed}_failure_determinism_receipt.json"
    for seed in freeze_contract.TRAIN_SEEDS
}
OUTPUT_PATHS = (
    freezer.DECISION_RECEIPT,
    freezer.DIAGNOSTIC_RECEIPT,
    freezer.PREFLIGHT_RECEIPT,
    freezer.TEST_RECEIPT,
    freezer.PLATFORM_RECEIPT,
)


class V3Status0EvidenceError(RuntimeError):
    """Raised before publication when any evidence is incomplete or unsafe."""


def _json_bytes(payload: Any) -> bytes:
    try:
        return (
            json.dumps(
                payload,
                indent=2,
                sort_keys=True,
                ensure_ascii=False,
                allow_nan=False,
            )
            + "\n"
        ).encode("utf-8")
    except (TypeError, ValueError) as exc:
        raise V3Status0EvidenceError("receipt is not finite strict JSON") from exc


def _repo_relative(path: Path) -> str:
    resolved = path.expanduser().resolve()
    try:
        relative = resolved.relative_to(REPO_ROOT.resolve())
    except ValueError as exc:
        raise V3Status0EvidenceError(f"path is outside repository: {resolved}") from exc
    value = relative.as_posix()
    pure = PurePosixPath(value)
    if pure.is_absolute() or ".." in pure.parts or pure.as_posix() != value:
        raise V3Status0EvidenceError(f"path is not canonical POSIX: {value}")
    return value


def _virtual_record(path: Path, payload: Any) -> dict[str, Any]:
    rendered = _json_bytes(payload)
    return {
        "path": _repo_relative(path),
        "sha256": hashlib.sha256(rendered).hexdigest(),
        "size_bytes": len(rendered),
    }


def _strict_mapping(path: Path) -> dict[str, Any]:
    value = comparator.strict_json_load(path)
    if not isinstance(value, Mapping):
        raise V3Status0EvidenceError(f"expected JSON object: {path}")
    return dict(value)


def _counter(value: Any, label: str) -> int:
    if type(value) is not int or value < 0:
        raise V3Status0EvidenceError(f"{label} must be a non-negative integer")
    return value


def _footer() -> dict[str, Any]:
    return {
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
    }


def _validate_failure_receipts() -> dict[str, Any]:
    records: dict[str, Any] = {}
    for seed, path in FAILURE_RECEIPTS.items():
        receipt = _strict_mapping(path)
        if (
            receipt.get("passed") is not False
            or receipt.get("comparison_passed") is not True
            or receipt.get("eligible_for_protocol_promotion") is not False
            or _counter(receipt.get("seed"), f"failure receipt seed {seed}") != seed
            or receipt.get("protected_trials_opened") != []
        ):
            raise V3Status0EvidenceError(
                f"historical failure receipt drifted for seed {seed}"
            )
        records[str(seed)] = freezer._record(path)
    return records


_SUMMARY_FIELDS = {
    "control_window_count": "so_solver_control_window_count",
    "solver_invocation_count": "so_solver_attempt_count",
    "primary_solver_nonconvergence_count": "so_solver_primary_nonconvergence_count",
    "bounded_ls_invocation_count": "so_solver_bounded_ls_invocation_count",
    "selected_bounded_ls_count": "so_solver_selected_bounded_ls_count",
    "hard_so_fallback_count": "so_solver_hard_fallback_count",
    "reuse_previous_count": "so_solver_reuse_previous_count",
    "bounded_ls_unsuccessful_count": "so_solver_bounded_ls_unsuccessful_count",
    "bounds_violation_count": "so_solver_bounds_violation_count",
    "nonfinite_solver_count": "so_solver_nonfinite_count",
    "selected_infeasible_count": "so_solver_selected_infeasible_count",
    "selected_solution_mismatch_count": "so_solver_selected_solution_mismatch_count",
    "residual_contract_mismatch_count": "so_solver_residual_contract_mismatch_count",
}


def _classify_run(directory: Path) -> dict[str, Any]:
    summary = _strict_mapping(directory / "summary.json")
    journal = comparator.strict_json_load(directory / "solver_audit_journal.json")
    if not isinstance(journal, list) or len(journal) != runner.EXPECTED_STEPS:
        raise V3Status0EvidenceError(f"solver journal is incomplete: {directory}")
    totals: dict[str, int] = {}
    for row_index, row in enumerate(journal, start=1):
        if not isinstance(row, Mapping) or set(row) != {
            "step",
            "time_s",
            "control_windows",
        }:
            raise V3Status0EvidenceError(
                f"solver journal row {row_index} drifted: {directory}"
            )
        if _counter(row.get("step"), "solver journal step") != row_index:
            raise V3Status0EvidenceError("solver journal steps are not contiguous")
        classification = so_recovery.classify_policy_step(
            row.get("control_windows"), policy_id=POLICY_ID
        )
        for key, value in classification["counters"].items():
            if type(value) is bool:
                continue
            totals[key] = totals.get(key, 0) + _counter(value, key)
    for counter_key, summary_key in _SUMMARY_FIELDS.items():
        if totals[counter_key] != _counter(summary.get(summary_key), summary_key):
            raise V3Status0EvidenceError(
                f"policy classification disagrees with {summary_key}"
            )
    if (
        totals["bounded_ls_invocation_count"]
        != totals["verified_bounded_ls_count"]
        or totals["verified_bounded_ls_count"]
        != totals["verified_bounded_ls_success_count"]
        + totals["verified_status0_max_iter_count"]
        or totals["hard_so_fallback_count"]
        != totals["verified_status0_max_iter_count"]
        or totals["bounded_ls_unsuccessful_count"]
        != totals["verified_status0_max_iter_count"]
        or totals["unaccepted_hard_so_fallback_count"] != 0
        or totals["unaccepted_bounded_ls_count"] != 0
    ):
        raise V3Status0EvidenceError(
            "status-0 policy does not explain every raw bounded-LS outcome"
        )
    for key in (
        "reuse_previous_count",
        "bounds_violation_count",
        "nonfinite_solver_count",
        "selected_infeasible_count",
        "selected_solution_mismatch_count",
        "residual_contract_mismatch_count",
    ):
        if totals[key] != 0:
            raise V3Status0EvidenceError(f"hard solver condition is non-zero: {key}")
    if (
        _counter(summary.get("sea_plugin_fallback_count"), "SEA fallback") != 0
        or _counter(summary.get("steps"), "steps") != runner.EXPECTED_STEPS
        or _counter(summary.get("n_actor"), "actor layout")
        != runner.EXPECTED_ACTOR_FEATURES
        or _counter(summary.get("n_observation"), "observation layout")
        != runner.EXPECTED_FULL_FEATURES
        or summary.get("observation_dtype") != "float32"
        or summary.get("online_grf_applied_sides") != ["left"]
        or summary.get("event_contract_id") != runner.EVENT_CONTRACT
        or summary.get("morphology_weight") != 0.0
        or summary.get("h0_used_for_behavior") is not False
        or _counter(summary.get("actor_updates"), "actor updates") != 0
        or _counter(summary.get("critic_updates"), "critic updates") != 0
        or _counter(summary.get("ppo_updates"), "PPO updates") != 0
        or summary.get("protected_trials_opened") != []
    ):
        raise V3Status0EvidenceError("diagnostic runtime/preflight contract drifted")
    return {"summary": summary, "policy_totals": totals}


def _validate_diagnostics() -> dict[str, Any]:
    result: dict[str, Any] = {}
    for seed in freeze_contract.TRAIN_SEEDS:
        directories = [
            freezer.DIAGNOSTIC_ROOT / f"seed_{seed}_rep{replicate}"
            for replicate in freeze_contract.DIAGNOSTIC_REPLICATES
        ]
        comparison = comparator.compare_diagnostics(
            seed=seed,
            replica_a_dir=directories[0],
            replica_b_dir=directories[1],
        )
        if comparison.get("comparison_passed") is not True:
            raise V3Status0EvidenceError(f"diagnostic replicas differ for seed {seed}")
        classified = [_classify_run(directory) for directory in directories]
        if comparator.canonical_json_bytes(classified[0]) != (
            comparator.canonical_json_bytes(classified[1])
        ):
            raise V3Status0EvidenceError(
                f"policy classification is not bit-exact for seed {seed}"
            )
        result[str(seed)] = classified[0]
    return result


def _validate_portable_paths() -> None:
    maps = (
        freeze_contract.LOCK_SOURCE_RELATIVE_PATHS,
        freeze_contract.PROTOCOL_SOURCE_RELATIVE_PATHS,
        freeze_contract.RUNTIME_SOURCE_RELATIVE_PATHS,
        freeze_contract.LOCK_INPUT_RELATIVE_PATHS,
        freeze_contract.RUNTIME_INPUT_RELATIVE_PATHS,
        freeze_contract.EVIDENCE_RELATIVE_PATHS,
        freeze_contract.LINEAGE_RELATIVE_PATHS,
    )
    for mapping in maps:
        for value in mapping.values():
            pure = PurePosixPath(value)
            if (
                not value
                or pure.is_absolute()
                or ".." in pure.parts
                or pure.as_posix() != value
                or "\\" in value
            ):
                raise V3Status0EvidenceError(f"non-portable frozen path: {value}")


def _run_frozen_tests() -> None:
    completed = subprocess.run(
        [sys.executable, "-m", "unittest", *freeze_contract.REQUIRED_TEST_MODULES],
        cwd=REPO_ROOT,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=False,
    )
    if completed.returncode != 0:
        raise V3Status0EvidenceError(
            "frozen preflight tests failed:\n" + completed.stdout[-12000:]
        )
    for relative in (
        *freeze_contract.LOCK_SOURCE_RELATIVE_PATHS.values(),
        *freeze_contract.PROTOCOL_SOURCE_RELATIVE_PATHS.values(),
    ):
        path = REPO_ROOT.joinpath(*PurePosixPath(relative).parts)
        if path.suffix == ".py":
            try:
                compile(path.read_text(encoding="utf-8"), str(path), "exec")
            except (OSError, SyntaxError) as exc:
                raise V3Status0EvidenceError(f"compile failed: {path}") from exc


def _decision_payload() -> dict[str, Any]:
    return {
        "schema_version": 3,
        "status": "H0_PRIMARY_SPLIT_V3_SO_POLICY_DECIDED",
        "passed": True,
        "protocol_id": runner.PROTOCOL_ID,
        "revision": runner.REVISION,
        "selected_policy_id": POLICY_ID,
        "policy": freeze_contract.SO_POLICIES[POLICY_ID],
        "authority_basis": "explicit_user_authorization",
        "policy_applies_to_future_v3_execution": True,
        "actor_update_candidate_count": 1,
        **_footer(),
    }


def build_payloads() -> dict[Path, dict[str, Any]]:
    if POLICY_ID != so_recovery.VERIFIED_STATUS0_MAX_ITER_POLICY:
        raise V3Status0EvidenceError("freeze/classifier policy identifiers drifted")
    if any(os.path.lexists(path) for path in OUTPUT_PATHS):
        raise V3Status0EvidenceError("refusing to clobber an evidence receipt")
    _validate_failure_receipts()
    diagnostics = _validate_diagnostics()
    _validate_portable_paths()
    _run_frozen_tests()

    decision = _decision_payload()
    diagnostic_sources = {
        "runner": runner.RUNNER,
        "diagnostic_comparator": freezer.PROTOCOL_SOURCE_PATHS[
            "diagnostic_comparator"
        ],
        "so_recovery_contract": freezer.PROTOCOL_SOURCE_PATHS[
            "so_recovery_contract"
        ],
        "simulation_runner": freezer.RUNTIME_SOURCE_PATHS["simulation_runner"],
        "static_optimization": freezer.RUNTIME_SOURCE_PATHS["static_optimization"],
        "environment": freezer.RUNTIME_SOURCE_PATHS["environment"],
    }
    diagnostic = {
        "schema_version": 3,
        "status": "PASS_H0_PRIMARY_SPLIT_V3_DIAGNOSTIC_DETERMINISM",
        "passed": True,
        "protocol_id": runner.PROTOCOL_ID,
        "revision": runner.REVISION,
        "so_policy_id": POLICY_ID,
        "train_seeds": list(runner.TRAIN_SEEDS),
        "replicates_per_seed": 2,
        "checks": {key: True for key in freezer.DIAGNOSTIC_CHECK_KEYS},
        "sources": {
            key: freezer._record(path) for key, path in diagnostic_sources.items()
        },
        "artifacts": {
            run_id: {
                filename: freezer._record(path) for filename, path in paths.items()
            }
            for run_id, paths in freezer._diagnostic_artifact_paths().items()
        },
        **_footer(),
    }
    decision_record = _virtual_record(freezer.DECISION_RECEIPT, decision)
    diagnostic_record = _virtual_record(freezer.DIAGNOSTIC_RECEIPT, diagnostic)
    preflight = {
        "schema_version": 3,
        "status": "PASS_H0_PRIMARY_SPLIT_V3_PREFLIGHT",
        "passed": True,
        "protocol_id": runner.PROTOCOL_ID,
        "revision": runner.REVISION,
        "so_policy_id": POLICY_ID,
        "checks": {key: True for key in freezer.PREFLIGHT_CHECK_KEYS},
        "decision_receipt": decision_record,
        "diagnostic_receipt": diagnostic_record,
        **_footer(),
    }
    tested_sources = {
        **freezer.LOCK_SOURCE_PATHS,
        **freezer.PROTOCOL_SOURCE_PATHS,
    }
    tests = {
        "schema_version": 3,
        "status": "PASS_H0_PRIMARY_SPLIT_V3_PREFLIGHT_TESTS",
        "passed": True,
        "protocol_id": runner.PROTOCOL_ID,
        "revision": runner.REVISION,
        "test_modules": list(freezer.REQUIRED_TEST_MODULES),
        "checks": {key: True for key in freezer.TEST_CHECK_KEYS},
        "tested_sources": {
            key: freezer._record(path) for key, path in tested_sources.items()
        },
        **_footer(),
    }
    identity = freezer._live_platform_identity()
    if identity.get("system") != "Darwin" or identity.get("machine") != "arm64":
        raise V3Status0EvidenceError("numerical evidence requires macOS arm64")
    platform = {
        "schema_version": 3,
        "status": "PASS_H0_PRIMARY_SPLIT_V3_PLATFORM",
        "passed": True,
        "protocol_id": runner.PROTOCOL_ID,
        "revision": runner.REVISION,
        "numerical_claim": "macOS-arm64-only",
        "windows_claim": "schema-and-path-compatibility-only",
        "identity": identity,
        "binary_artifacts": {
            key: freezer._record(freezer.RUNTIME_INPUT_PATHS[key])
            for key in ("online_grf_macos_dylib", "sea_macos_dylib")
        },
        **_footer(),
    }
    # Keep the fully validated classification alive until every payload has
    # been constructed; it is intentionally not duplicated into receipt JSON.
    if set(diagnostics) != {str(seed) for seed in runner.TRAIN_SEEDS}:
        raise V3Status0EvidenceError("diagnostic seed set drifted")
    return {
        freezer.DECISION_RECEIPT: decision,
        freezer.DIAGNOSTIC_RECEIPT: diagnostic,
        freezer.PREFLIGHT_RECEIPT: preflight,
        freezer.TEST_RECEIPT: tests,
        freezer.PLATFORM_RECEIPT: platform,
    }


def publish() -> dict[str, Any]:
    payloads = build_payloads()
    for path, payload in payloads.items():
        runner._write_json_exclusive(path, payload)
    for path, payload in payloads.items():
        if freezer._record(path) != _virtual_record(path, payload):
            raise V3Status0EvidenceError(f"published receipt drifted: {path}")
    return {
        "status": "PASS_H0_PRIMARY_SPLIT_V3_STATUS0_EVIDENCE_PUBLISHED",
        "passed": True,
        "so_policy_id": POLICY_ID,
        "receipts": {path.name: freezer._record(path) for path in payloads},
        **_footer(),
    }


if __name__ == "__main__":
    print(json.dumps(publish(), indent=2, sort_keys=True, allow_nan=False))
