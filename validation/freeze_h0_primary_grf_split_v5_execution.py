"""Publish only the immutable H0 primary-split V5 execution lock.

The freezer hashes the seed-125 historical files as opaque bytes.  It must not
load or interpret either file, create the V5 run root, fit an actor, or invoke
the replay engine.
"""

from __future__ import annotations

import json
import os
import sys
from pathlib import Path, PurePosixPath
from typing import Any, Mapping


REPO_ROOT = Path(__file__).resolve().parents[1]
VALIDATION_ROOT = REPO_ROOT / "validation"
TRAJECTORY_ROOT = REPO_ROOT / "Trajectory Generator"
BASELINE_ROOT = TRAJECTORY_ROOT / "baseline_MLP"
for import_root in (VALIDATION_ROOT, BASELINE_ROOT, TRAJECTORY_ROOT, REPO_ROOT):
    if str(import_root) not in sys.path:
        sys.path.insert(0, str(import_root))

import h0_primary_grf_split_v5_freeze_contract as contract  # noqa: E402
import run_h0_primary_grf_split_v3_semantic_replay as v3  # noqa: E402
import run_h0_primary_grf_split_v5_full_mean as runner  # noqa: E402


LOCK = runner.LOCK
RUN_ROOT = runner.RUN_ROOT


class V5FreezeError(RuntimeError):
    """Raised when the V5 boundary cannot be frozen safely."""


def _resolve_relative(relative: str) -> Path:
    pure = PurePosixPath(relative)
    if (
        not relative
        or pure.is_absolute()
        or ".." in pure.parts
        or pure.as_posix() != relative
    ):
        raise V5FreezeError(f"non-canonical contract path: {relative!r}")
    return REPO_ROOT.joinpath(*pure.parts)


def _resolve_map(values: Mapping[str, str]) -> dict[str, Path]:
    return {key: _resolve_relative(value) for key, value in values.items()}


V5_SOURCE_PATHS = _resolve_map(contract.V5_SOURCE_RELATIVE_PATHS)
INHERITED_SOURCE_PATHS = _resolve_map(contract.INHERITED_SOURCE_RELATIVE_PATHS)
H0_INPUT_PATHS = _resolve_map(contract.H0_INPUT_RELATIVE_PATHS)
V3_INPUT_PATHS = _resolve_map(contract.V3_INPUT_RELATIVE_PATHS)
HOLDOUT_BYTE_INPUT_PATHS = _resolve_map(contract.HOLDOUT_BYTE_INPUT_RELATIVE_PATHS)
PREFLIGHT_RECEIPT = _resolve_relative(contract.PREFLIGHT_RECEIPT_RELATIVE)
V4_PREEXECUTION_FAILURE = _resolve_relative(
    contract.V4_PREEXECUTION_FAILURE_RELATIVE
)


def _record(path: str | Path) -> dict[str, Any]:
    resolved = Path(path).expanduser().resolve()
    if not resolved.is_file():
        raise V5FreezeError(f"required frozen file is missing: {resolved}")
    return runner.source_record(resolved)


def _record_map(paths: Mapping[str, Path]) -> dict[str, Any]:
    return {key: _record(path) for key, path in paths.items()}


def _validate_alignment() -> None:
    exact = {
        "lock": (runner.LOCK, _resolve_relative(contract.LOCK_RELATIVE)),
        "run_root": (runner.RUN_ROOT, _resolve_relative(contract.RUN_ROOT_RELATIVE)),
        "runner": (runner.RUNNER, V5_SOURCE_PATHS["runner"]),
        "H0 config": (runner.H0_CONFIG, H0_INPUT_PATHS["config"]),
        "H0 state": (
            runner.H0_MODULE / "module_state.pkl",
            H0_INPUT_PATHS["state"],
        ),
        "H0 ctor": (
            runner.H0_MODULE / "class_and_ctor_args.pkl",
            H0_INPUT_PATHS["ctor"],
        ),
        "H0 metadata": (
            runner.H0_MODULE / "metadata.json",
            H0_INPUT_PATHS["metadata"],
        ),
        "V3 runner": (v3.RUNNER, INHERITED_SOURCE_PATHS["v3_runner_replay_engine"]),
        "V4 hardened template": (
            runner.V4_HARDENED_TEMPLATE,
            INHERITED_SOURCE_PATHS["v4_hardened_runner_template"],
        ),
        "V4 pre-execution failure": (
            runner.V4_PREEXECUTION_FAILURE,
            V4_PREEXECUTION_FAILURE,
        ),
    }
    for label, (observed, expected) in exact.items():
        if Path(observed).expanduser().resolve() != Path(expected).resolve():
            raise V5FreezeError(f"V5 contract alignment drifted at {label}")
    if (
        tuple(runner.TRAIN_SEEDS) != contract.TRAIN_SEEDS
        or runner.FINAL_HOLDOUT_SEED != contract.FINAL_HOLDOUT_SEED
        or runner.FIT != contract.FIT
        or runner.PROTOCOL_ID != contract.PROTOCOL_ID
        or runner.CANDIDATE_ID != contract.CANDIDATE_ID
    ):
        raise V5FreezeError("V5 runner constants drifted from the pure contract")


def _validate_train_only_design() -> None:
    evidence = contract.TRAIN_ONLY_PREFLIGHT_EVIDENCE
    thresholds = contract.OFFLINE_THRESHOLDS
    if (
        evidence["candidate_selection_count"] != 1
        or evidence["holdout_seed_accessed"] is not False
        or evidence["student_rmse"] > thresholds["student_rmse_max"]
        or evidence["student_max_abs_error"]
        > thresholds["student_max_abs_error_max"]
        or evidence["teacher_rmse"] > thresholds["teacher_rmse_max"]
        or evidence["teacher_max_abs_error"]
        > thresholds["teacher_max_abs_error_max"]
    ):
        raise V5FreezeError("authorized V5 train-only design evidence is invalid")


def _validate_v3_lineage_and_corpus() -> dict[str, Any]:
    v3_lock = v3.verify_lock()
    if v3_lock.get("so_policy_id") != contract.SO_POLICY_ID:
        raise V5FreezeError("V3 status0 policy identity drifted")
    ledger, gate = runner._validate_v3_terminal_lineage()
    receipt, manifest, corpus_path = v3._validate_corpus_receipt()
    if (
        receipt.get("passed") is not True
        or receipt.get("status") != "PASS_H0_PRIMARY_SPLIT_V3_CORPUS_FROZEN"
        or manifest.get("training_trials") != ["123", "124"]
        or manifest.get("validation_trials") != []
        or manifest.get("records") != 2000
        or corpus_path.resolve() != V3_INPUT_PATHS["corpus"].resolve()
    ):
        raise V5FreezeError("V3 inherited train corpus drifted")
    failed_receipt = runner._strict_mapping(V3_INPUT_PATHS["failed_candidate_receipt"])
    if (
        failed_receipt.get("status") != "FAIL_H0_PRIMARY_SPLIT_V3_OFFLINE"
        or failed_receipt.get("passed") is not False
        or failed_receipt.get("actor_updates") != 1
        or failed_receipt.get("critic_updates") != 0
        or failed_receipt.get("ppo_updates") != 0
        or failed_receipt.get("protected_trials_opened") != []
    ):
        raise V5FreezeError("V3 failed candidate receipt drifted")
    # Keep variables live to make the validated lineage explicit to audit tools.
    if ledger.get("status") != gate.get("status"):
        raise V5FreezeError("V3 terminal ledger/gate disagree")
    return v3_lock


def _validate_v4_preexecution_lineage() -> dict[str, Any]:
    try:
        return runner._validate_v4_preexecution_failure()
    except runner.H0PrimarySplitV5Error as exc:
        raise V5FreezeError(str(exc)) from exc


def _validate_preflight_receipt() -> dict[str, Any]:
    try:
        receipt = runner._strict_mapping(PREFLIGHT_RECEIPT)
    except runner.H0PrimarySplitV5Error as exc:
        raise V5FreezeError("V5 preflight receipt is absent or invalid") from exc
    expected_keys = {
        "schema_version",
        "status",
        "passed",
        "protocol_id",
        "revision",
        "checks",
        "commands",
        "pytest_passed",
        "recursive_test_exclusion",
        "tested_sources",
        "lineage",
        "actor_updates",
        "critic_updates",
        "ppo_updates",
        "protected_trials_opened",
    }
    checks = receipt.get("checks")
    exclusion = receipt.get("recursive_test_exclusion")
    if (
        set(receipt) != expected_keys
        or receipt.get("schema_version") != 5
        or receipt.get("status") != "PASS_H0_PRIMARY_SPLIT_V5_PREFLIGHT_TESTS"
        or receipt.get("passed") is not True
        or receipt.get("protocol_id") != contract.PROTOCOL_ID
        or receipt.get("revision") != contract.REVISION
        or not isinstance(checks, Mapping)
        or not checks
        or any(value is not True for value in checks.values())
        or receipt.get("pytest_passed") != 11
        or not isinstance(exclusion, Mapping)
        or exclusion.get("test")
        != "test_build_payload_is_complete_and_does_not_publish_lock_or_run"
        or exclusion.get("must_pass_after_receipt_publication") is not True
        or receipt.get("actor_updates") != 0
        or receipt.get("critic_updates") != 0
        or receipt.get("ppo_updates") != 0
        or receipt.get("protected_trials_opened") != []
    ):
        raise V5FreezeError("V5 preflight receipt is non-canonical")
    tested_sources = receipt.get("tested_sources")
    if not isinstance(tested_sources, Mapping) or set(tested_sources) != set(
        V5_SOURCE_PATHS
    ):
        raise V5FreezeError("V5 preflight tested-source closure drifted")
    for key, path in V5_SOURCE_PATHS.items():
        if runner._canonical_bytes(tested_sources[key]) != runner._canonical_bytes(
            _record(path)
        ):
            raise V5FreezeError(f"V5 preflight source drifted: {key}")
    expected_lineage = {
        "v3_execution_ledger": V3_INPUT_PATHS["execution_ledger"],
        "v3_corpus_receipt": V3_INPUT_PATHS["corpus_receipt"],
        "v3_failed_offline_gate": V3_INPUT_PATHS["failed_offline_gate"],
        "v4_preexecution_failure": V4_PREEXECUTION_FAILURE,
    }
    lineage = receipt.get("lineage")
    if not isinstance(lineage, Mapping) or set(lineage) != set(expected_lineage):
        raise V5FreezeError("V5 preflight lineage schema drifted")
    for key, path in expected_lineage.items():
        if runner._canonical_bytes(lineage[key]) != runner._canonical_bytes(
            _record(path)
        ):
            raise V5FreezeError(f"V5 preflight lineage drifted: {key}")
    return receipt


def _opaque_holdout_byte_records() -> dict[str, Any]:
    # Deliberately no json.load/np.load/historical_inputs call here.
    return _record_map(HOLDOUT_BYTE_INPUT_PATHS)


def _build_lock_payload() -> dict[str, Any]:
    _validate_alignment()
    _validate_train_only_design()
    v3_lock = _validate_v3_lineage_and_corpus()
    _validate_v4_preexecution_lineage()
    _validate_preflight_receipt()
    inputs = {
        "h0": _record_map(H0_INPUT_PATHS),
        **_record_map(V3_INPUT_PATHS),
        "holdout_seed125_opaque_bytes": _opaque_holdout_byte_records(),
        "preflight_receipt": _record(PREFLIGHT_RECEIPT),
        "v4_preexecution_failure": _record(V4_PREEXECUTION_FAILURE),
    }
    destinations = [
        f"{contract.RUN_ROOT_RELATIVE}/{suffix}"
        for suffix in contract.DESTINATION_SUFFIXES
    ]
    payload = {
        "schema_version": 5,
        "status": "H0_PRIMARY_GRF_SPLIT_V5_EXECUTION_FROZEN",
        "protocol_id": contract.PROTOCOL_ID,
        "revision": contract.REVISION,
        "candidate_id": contract.CANDIDATE_ID,
        "run_root": contract.RUN_ROOT_RELATIVE,
        "train_seeds": list(contract.TRAIN_SEEDS),
        "final_holdout_seed": contract.FINAL_HOLDOUT_SEED,
        "expected_steps": contract.EXPECTED_STEPS,
        "expected_actor_features": contract.EXPECTED_ACTOR_FEATURES,
        "event_contract_id": contract.EVENT_CONTRACT_ID,
        "so_policy_id": contract.SO_POLICY_ID,
        "so_policy": v3_lock["so_policy"],
        "fit": dict(contract.FIT),
        "trainable_scope": contract.TRAINABLE_SCOPE,
        "logstd_policy": contract.LOGSTD_POLICY,
        "offline_thresholds": dict(contract.OFFLINE_THRESHOLDS),
        "train_only_preflight_evidence": dict(
            contract.TRAIN_ONLY_PREFLIGHT_EVIDENCE
        ),
        "destinations": destinations,
        "authority": dict(contract.AUTHORITY),
        "sources": {
            "v5": _record_map(V5_SOURCE_PATHS),
            "inherited": _record_map(INHERITED_SOURCE_PATHS),
        },
        "inputs": inputs,
        "v3_terminal_expectation": dict(contract.V3_TERMINAL_EXPECTATION),
        "v4_preexecution_expectation": dict(
            contract.V4_PREEXECUTION_EXPECTATION
        ),
        "actor_update_candidate_count": 1,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
    }
    if set(payload) != runner.LOCK_TOP_LEVEL_KEYS:
        raise V5FreezeError("V5 runner/freezer lock schemas disagree")
    runner._require_finite_json(payload, "V5 lock")
    return payload


def freeze() -> dict[str, Any]:
    """Create only the no-clobber V5 lock after every prerequisite passes."""
    if os.path.lexists(LOCK):
        raise V5FreezeError(f"refusing to clobber existing V5 lock: {LOCK}")
    if os.path.lexists(RUN_ROOT):
        raise V5FreezeError(
            f"V5 run root must not exist before execution freeze: {RUN_ROOT}"
        )
    payload = _build_lock_payload()
    if os.path.lexists(LOCK) or os.path.lexists(RUN_ROOT):
        raise V5FreezeError("V5 lock/run root appeared during freeze")
    runner._write_json_exclusive(LOCK, payload)
    if os.path.lexists(RUN_ROOT):
        raise V5FreezeError("V5 freezer unexpectedly created the run root")
    runner.verify_lock()
    return payload


if __name__ == "__main__":
    print(json.dumps(freeze(), indent=2, sort_keys=True, allow_nan=False))
