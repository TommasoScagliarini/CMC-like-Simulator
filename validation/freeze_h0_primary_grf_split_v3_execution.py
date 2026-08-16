"""Freeze the one-shot H0 primary-split V3 execution contract.

This freezer deliberately performs no replay, corpus construction, actor
adaptation, holdout evaluation, qualification, or PPO work.  Its only allowed
mutation is publication of the immutable V3 execution lock.  In particular it
must never create ``runner.RUN_ROOT``.

The seed-125 history is a procedural holdout.  The freezer records its bytes by
SHA-256 as already-known provenance, but never parses or semantically validates
the trace or summary.  Semantic access remains guarded by the candidate freeze
and holdout-access claim in the execution runner.
"""

from __future__ import annotations

import importlib.metadata
import json
import math
import os
import platform
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

import h0_primary_grf_split_v3_freeze_contract as freeze_contract  # noqa: E402
import run_h0_primary_grf_split_v3_semantic_replay as runner  # noqa: E402


LOCK = runner.LOCK
RUN_ROOT = runner.RUN_ROOT


def _resolve_relative_path(relative: str) -> Path:
    pure = PurePosixPath(relative)
    if (
        not relative
        or pure.is_absolute()
        or ".." in pure.parts
        or relative != pure.as_posix()
    ):
        raise RuntimeError(f"non-canonical freeze-contract path: {relative!r}")
    return REPO_ROOT.joinpath(*pure.parts)


def _resolve_relative_map(values: Mapping[str, str]) -> dict[str, Path]:
    return {key: _resolve_relative_path(relative) for key, relative in values.items()}


LOCK_SOURCE_PATHS = _resolve_relative_map(freeze_contract.LOCK_SOURCE_RELATIVE_PATHS)
PROTOCOL_SOURCE_PATHS = _resolve_relative_map(
    freeze_contract.PROTOCOL_SOURCE_RELATIVE_PATHS
)
RUNTIME_SOURCE_PATHS = _resolve_relative_map(
    freeze_contract.RUNTIME_SOURCE_RELATIVE_PATHS
)
LOCK_INPUT_PATHS = _resolve_relative_map(freeze_contract.LOCK_INPUT_RELATIVE_PATHS)
RUNTIME_INPUT_PATHS = _resolve_relative_map(
    freeze_contract.RUNTIME_INPUT_RELATIVE_PATHS
)
EVIDENCE_PATHS = _resolve_relative_map(freeze_contract.EVIDENCE_RELATIVE_PATHS)
LINEAGE_PATHS = _resolve_relative_map(freeze_contract.LINEAGE_RELATIVE_PATHS)
DECISION_RECEIPT = EVIDENCE_PATHS["so_policy_decision"]
DIAGNOSTIC_RECEIPT = EVIDENCE_PATHS["diagnostic_determinism"]
PREFLIGHT_RECEIPT = EVIDENCE_PATHS["instrumented_preflight"]
TEST_RECEIPT = EVIDENCE_PATHS["preflight_tests"]
PLATFORM_RECEIPT = EVIDENCE_PATHS["platform"]
DIAGNOSTIC_ROOT = _resolve_relative_path(freeze_contract.DIAGNOSTIC_ROOT_RELATIVE_PATH)
HISTORICAL_INPUT_PATHS = {
    str(seed): {
        role: _resolve_relative_path(template.format(seed=seed))
        for role, template in (
            freeze_contract.HISTORICAL_INPUT_RELATIVE_PATH_TEMPLATES.items()
        )
    }
    for seed in (*freeze_contract.TRAIN_SEEDS, freeze_contract.FINAL_HOLDOUT_SEED)
}
DIAGNOSTIC_ARTIFACT_PATHS = {
    run_id: _resolve_relative_map(paths)
    for run_id, paths in (freeze_contract.diagnostic_artifact_relative_paths().items())
}

STRICT_ZERO_POLICY = freeze_contract.STRICT_ZERO_POLICY
VERIFIED_SUCCESS_POLICY = freeze_contract.VERIFIED_SUCCESS_POLICY
VERIFIED_STATUS0_MAX_ITER_POLICY = freeze_contract.VERIFIED_STATUS0_MAX_ITER_POLICY
# Backward-compatible internal name used by the focused freezer fixtures.
VERIFIED_RECOVERY_POLICY = VERIFIED_STATUS0_MAX_ITER_POLICY
SO_POLICIES = freeze_contract.SO_POLICIES
DIAGNOSTIC_FILENAMES = freeze_contract.DIAGNOSTIC_FILENAMES
DIAGNOSTIC_CHECK_KEYS = freeze_contract.DIAGNOSTIC_CHECK_KEYS
PREFLIGHT_CHECK_KEYS = freeze_contract.PREFLIGHT_CHECK_KEYS
TEST_CHECK_KEYS = freeze_contract.TEST_CHECK_KEYS
REQUIRED_TEST_MODULES = freeze_contract.REQUIRED_TEST_MODULES


class V3FreezeError(RuntimeError):
    """Raised when the V3 execution boundary cannot be frozen safely."""


def _sha256_file(path: str | Path) -> str:
    import hashlib

    digest = hashlib.sha256()
    with Path(path).expanduser().resolve().open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _repo_relative_posix(path: str | Path) -> str:
    resolved = Path(path).expanduser().resolve()
    try:
        relative = resolved.relative_to(REPO_ROOT.resolve())
    except ValueError as exc:
        raise V3FreezeError(f"path is outside the repository: {resolved}") from exc
    value = relative.as_posix()
    pure = PurePosixPath(value)
    if pure.is_absolute() or ".." in pure.parts or value != pure.as_posix():
        raise V3FreezeError(f"path is not canonical POSIX: {value}")
    return value


def _record(path: str | Path) -> dict[str, Any]:
    resolved = Path(path).expanduser().resolve()
    if not resolved.is_file():
        raise V3FreezeError(f"required frozen file is missing: {resolved}")
    return {
        "path": _repo_relative_posix(resolved),
        "sha256": _sha256_file(resolved),
        "size_bytes": resolved.stat().st_size,
    }


def _strict_mapping(path: str | Path) -> dict[str, Any]:
    resolved = Path(path).expanduser().resolve()
    try:
        with resolved.open("r", encoding="utf-8") as stream:
            value = json.load(
                stream,
                parse_constant=lambda token: (_ for _ in ()).throw(
                    ValueError(f"non-finite JSON token {token}")
                ),
            )
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        raise V3FreezeError(f"invalid strict JSON object: {resolved}") from exc
    if not isinstance(value, Mapping):
        raise V3FreezeError(f"expected JSON object: {resolved}")
    _require_finite_json(value, str(resolved))
    return dict(value)


def _require_finite_json(value: Any, label: str) -> None:
    if value is None or type(value) in {bool, str, int}:
        return
    if isinstance(value, float):
        if not math.isfinite(value):
            raise V3FreezeError(f"{label} contains a non-finite number")
        return
    if isinstance(value, Mapping):
        for key, child in value.items():
            if not isinstance(key, str) or not key:
                raise V3FreezeError(f"{label} contains an invalid key")
            _require_finite_json(child, f"{label}.{key}")
        return
    if isinstance(value, list):
        for index, child in enumerate(value):
            _require_finite_json(child, f"{label}[{index}]")
        return
    raise V3FreezeError(f"{label} contains a non-JSON value")


def _require_exact_record(record: Any, expected_path: str | Path, label: str) -> None:
    if not isinstance(record, Mapping) or set(record) != {
        "path",
        "sha256",
        "size_bytes",
    }:
        raise V3FreezeError(f"{label} is not an exact source record")
    expected = _record(expected_path)
    if runner.common_gates.canonical_json_bytes(record) != (
        runner.common_gates.canonical_json_bytes(expected)
    ):
        raise V3FreezeError(f"{label} identity drifted")


def _require_zero_update_footer(receipt: Mapping[str, Any], label: str) -> None:
    if (
        type(receipt.get("actor_updates")) is not int
        or receipt.get("actor_updates") != 0
        or type(receipt.get("critic_updates")) is not int
        or receipt.get("critic_updates") != 0
        or type(receipt.get("ppo_updates")) is not int
        or receipt.get("ppo_updates") != 0
        or receipt.get("protected_trials_opened") != []
    ):
        raise V3FreezeError(f"{label} widened execution authority")


def _validate_decision_receipt() -> tuple[dict[str, Any], str]:
    receipt = _strict_mapping(DECISION_RECEIPT)
    expected_keys = {
        "schema_version",
        "status",
        "passed",
        "protocol_id",
        "revision",
        "selected_policy_id",
        "policy",
        "authority_basis",
        "policy_applies_to_future_v3_execution",
        "actor_update_candidate_count",
        "actor_updates",
        "critic_updates",
        "ppo_updates",
        "protected_trials_opened",
    }
    if set(receipt) != expected_keys:
        raise V3FreezeError("SO-policy decision receipt schema drifted")
    policy_id = receipt.get("selected_policy_id")
    if policy_id not in SO_POLICIES:
        raise V3FreezeError("SO-policy decision is not one of the closed choices")
    if (
        receipt.get("schema_version") != 3
        or receipt.get("status") != "H0_PRIMARY_SPLIT_V3_SO_POLICY_DECIDED"
        or receipt.get("passed") is not True
        or receipt.get("protocol_id") != runner.PROTOCOL_ID
        or receipt.get("revision") != runner.REVISION
        or receipt.get("authority_basis") != "explicit_user_authorization"
        or receipt.get("policy_applies_to_future_v3_execution") is not True
        or receipt.get("actor_update_candidate_count") != 1
        or receipt.get("policy") != SO_POLICIES[policy_id]
    ):
        raise V3FreezeError("SO-policy decision receipt is non-canonical")
    _require_zero_update_footer(receipt, "SO-policy decision")
    return receipt, str(policy_id)


def _diagnostic_artifact_paths() -> dict[str, dict[str, Path]]:
    return {run_id: dict(paths) for run_id, paths in DIAGNOSTIC_ARTIFACT_PATHS.items()}


def _validate_diagnostic_receipt(policy_id: str) -> dict[str, Any]:
    receipt = _strict_mapping(DIAGNOSTIC_RECEIPT)
    expected_keys = {
        "schema_version",
        "status",
        "passed",
        "protocol_id",
        "revision",
        "so_policy_id",
        "train_seeds",
        "replicates_per_seed",
        "checks",
        "sources",
        "artifacts",
        "actor_updates",
        "critic_updates",
        "ppo_updates",
        "protected_trials_opened",
    }
    if set(receipt) != expected_keys:
        raise V3FreezeError("diagnostic determinism receipt schema drifted")
    if (
        receipt.get("schema_version") != 3
        or receipt.get("status") != "PASS_H0_PRIMARY_SPLIT_V3_DIAGNOSTIC_DETERMINISM"
        or receipt.get("passed") is not True
        or receipt.get("protocol_id") != runner.PROTOCOL_ID
        or receipt.get("revision") != runner.REVISION
        or receipt.get("so_policy_id") != policy_id
        or receipt.get("train_seeds") != list(runner.TRAIN_SEEDS)
        or receipt.get("replicates_per_seed") != 2
    ):
        raise V3FreezeError("diagnostic determinism receipt is non-canonical")
    checks = receipt.get("checks")
    if (
        not isinstance(checks, Mapping)
        or set(checks) != DIAGNOSTIC_CHECK_KEYS
        or any(value is not True for value in checks.values())
    ):
        raise V3FreezeError("diagnostic determinism checks did not all pass")
    expected_sources = {
        "runner": runner.RUNNER,
        "diagnostic_comparator": PROTOCOL_SOURCE_PATHS["diagnostic_comparator"],
        "so_recovery_contract": PROTOCOL_SOURCE_PATHS["so_recovery_contract"],
        "simulation_runner": RUNTIME_SOURCE_PATHS["simulation_runner"],
        "static_optimization": RUNTIME_SOURCE_PATHS["static_optimization"],
        "environment": RUNTIME_SOURCE_PATHS["environment"],
    }
    sources = receipt.get("sources")
    if not isinstance(sources, Mapping) or set(sources) != set(expected_sources):
        raise V3FreezeError("diagnostic source closure drifted")
    for key, path in expected_sources.items():
        _require_exact_record(sources[key], path, f"diagnostic.sources.{key}")
    expected_artifacts = _diagnostic_artifact_paths()
    artifacts = receipt.get("artifacts")
    if not isinstance(artifacts, Mapping) or set(artifacts) != set(expected_artifacts):
        raise V3FreezeError("diagnostic artifact run set drifted")
    for run_id, paths in expected_artifacts.items():
        run = artifacts[run_id]
        if not isinstance(run, Mapping) or set(run) != set(paths):
            raise V3FreezeError(f"diagnostic artifact schema drifted for {run_id}")
        for filename, path in paths.items():
            _require_exact_record(
                run[filename], path, f"diagnostic.artifacts.{run_id}.{filename}"
            )
    _require_zero_update_footer(receipt, "diagnostic determinism")
    return receipt


def _validate_preflight_receipt(policy_id: str) -> dict[str, Any]:
    receipt = _strict_mapping(PREFLIGHT_RECEIPT)
    expected_keys = {
        "schema_version",
        "status",
        "passed",
        "protocol_id",
        "revision",
        "so_policy_id",
        "checks",
        "decision_receipt",
        "diagnostic_receipt",
        "actor_updates",
        "critic_updates",
        "ppo_updates",
        "protected_trials_opened",
    }
    if set(receipt) != expected_keys:
        raise V3FreezeError("instrumented preflight receipt schema drifted")
    if (
        receipt.get("schema_version") != 3
        or receipt.get("status") != "PASS_H0_PRIMARY_SPLIT_V3_PREFLIGHT"
        or receipt.get("passed") is not True
        or receipt.get("protocol_id") != runner.PROTOCOL_ID
        or receipt.get("revision") != runner.REVISION
        or receipt.get("so_policy_id") != policy_id
    ):
        raise V3FreezeError("instrumented preflight receipt is non-canonical")
    checks = receipt.get("checks")
    if (
        not isinstance(checks, Mapping)
        or set(checks) != PREFLIGHT_CHECK_KEYS
        or any(value is not True for value in checks.values())
    ):
        raise V3FreezeError("instrumented preflight checks did not all pass")
    _require_exact_record(
        receipt.get("decision_receipt"), DECISION_RECEIPT, "preflight.decision"
    )
    _require_exact_record(
        receipt.get("diagnostic_receipt"),
        DIAGNOSTIC_RECEIPT,
        "preflight.diagnostic",
    )
    _require_zero_update_footer(receipt, "instrumented preflight")
    return receipt


def _validate_test_receipt() -> dict[str, Any]:
    receipt = _strict_mapping(TEST_RECEIPT)
    expected_keys = {
        "schema_version",
        "status",
        "passed",
        "protocol_id",
        "revision",
        "test_modules",
        "checks",
        "tested_sources",
        "actor_updates",
        "critic_updates",
        "ppo_updates",
        "protected_trials_opened",
    }
    if set(receipt) != expected_keys:
        raise V3FreezeError("preflight test receipt schema drifted")
    if (
        receipt.get("schema_version") != 3
        or receipt.get("status") != "PASS_H0_PRIMARY_SPLIT_V3_PREFLIGHT_TESTS"
        or receipt.get("passed") is not True
        or receipt.get("protocol_id") != runner.PROTOCOL_ID
        or receipt.get("revision") != runner.REVISION
        or receipt.get("test_modules") != list(REQUIRED_TEST_MODULES)
    ):
        raise V3FreezeError("preflight test receipt is non-canonical")
    checks = receipt.get("checks")
    if (
        not isinstance(checks, Mapping)
        or set(checks) != TEST_CHECK_KEYS
        or any(value is not True for value in checks.values())
    ):
        raise V3FreezeError("preflight test checks did not all pass")
    expected_sources = {
        **LOCK_SOURCE_PATHS,
        **PROTOCOL_SOURCE_PATHS,
    }
    tested_sources = receipt.get("tested_sources")
    if not isinstance(tested_sources, Mapping) or set(tested_sources) != set(
        expected_sources
    ):
        raise V3FreezeError("tested source closure drifted")
    for key, path in expected_sources.items():
        _require_exact_record(
            tested_sources[key], path, f"test_receipt.tested_sources.{key}"
        )
    _require_zero_update_footer(receipt, "preflight tests")
    return receipt


def _distribution_version(name: str) -> str:
    try:
        value = importlib.metadata.version(name)
    except importlib.metadata.PackageNotFoundError as exc:
        raise V3FreezeError(f"required distribution is missing: {name}") from exc
    if not value:
        raise V3FreezeError(f"distribution has an empty version: {name}")
    return value


def _live_platform_identity() -> dict[str, Any]:
    return {
        "system": platform.system(),
        "machine": platform.machine(),
        "python_version": platform.python_version(),
        "python_implementation": platform.python_implementation(),
        "python_executable": Path(sys.executable).resolve().as_posix(),
        "distributions": {
            name: _distribution_version(name)
            for name in ("numpy", "scipy", "torch", "ray", "gymnasium", "opensim")
        },
    }


def _validate_platform_receipt() -> dict[str, Any]:
    receipt = _strict_mapping(PLATFORM_RECEIPT)
    expected_keys = {
        "schema_version",
        "status",
        "passed",
        "protocol_id",
        "revision",
        "numerical_claim",
        "windows_claim",
        "identity",
        "binary_artifacts",
        "actor_updates",
        "critic_updates",
        "ppo_updates",
        "protected_trials_opened",
    }
    if set(receipt) != expected_keys:
        raise V3FreezeError("platform receipt schema drifted")
    identity = _live_platform_identity()
    if (
        receipt.get("schema_version") != 3
        or receipt.get("status") != "PASS_H0_PRIMARY_SPLIT_V3_PLATFORM"
        or receipt.get("passed") is not True
        or receipt.get("protocol_id") != runner.PROTOCOL_ID
        or receipt.get("revision") != runner.REVISION
        or receipt.get("numerical_claim") != "macOS-arm64-only"
        or receipt.get("windows_claim") != "schema-and-path-compatibility-only"
        or identity.get("system") != "Darwin"
        or identity.get("machine") != "arm64"
        or runner.common_gates.canonical_json_bytes(receipt.get("identity"))
        != runner.common_gates.canonical_json_bytes(identity)
    ):
        raise V3FreezeError("platform receipt is non-canonical or stale")
    expected_binaries = {
        "online_grf_macos_dylib": RUNTIME_INPUT_PATHS["online_grf_macos_dylib"],
        "sea_macos_dylib": RUNTIME_INPUT_PATHS["sea_macos_dylib"],
    }
    binaries = receipt.get("binary_artifacts")
    if not isinstance(binaries, Mapping) or set(binaries) != set(expected_binaries):
        raise V3FreezeError("platform binary closure drifted")
    for key, path in expected_binaries.items():
        _require_exact_record(binaries[key], path, f"platform.binary.{key}")
    _require_zero_update_footer(receipt, "platform")
    return receipt


def _validate_lineage() -> None:
    expected = {
        "v1_terminal_ledger": (
            "ERROR_PRIMARY_SPLIT_TEACHER",
            "STOP_WITHOUT_RETRY_OR_RETUNING",
        ),
        "v2_terminal_ledger": (
            "ERROR_H0_PRIMARY_SPLIT_V2_TEACHER",
            "STOP_WITHOUT_RETRY_OR_RETUNING",
        ),
    }
    for key, (status, next_stage) in expected.items():
        value = _strict_mapping(LINEAGE_PATHS[key])
        if (
            value.get("status") != status
            or value.get("passed") is not False
            or value.get("next_stage") != next_stage
            or value.get("protected_trials_opened") != []
        ):
            raise V3FreezeError(f"historical lineage drifted: {key}")
    initial = _strict_mapping(LINEAGE_PATHS["initial_v3_decision_required_receipt"])
    if (
        initial.get("status") != "H0_PRIMARY_SPLIT_V3_PREFLIGHT_GATE_DECISION_REQUIRED"
        or initial.get("passed") is not False
        or initial.get("actor_updates") != 0
        or initial.get("ppo_updates") != 0
        or initial.get("protected_trials_opened") != []
    ):
        raise V3FreezeError("initial V3 decision-required receipt drifted")


def _validate_contract_alignment() -> None:
    expected_seed_order = (
        *freeze_contract.TRAIN_SEEDS,
        freeze_contract.FINAL_HOLDOUT_SEED,
    )
    exact_paths = {
        "runner.PLAN": (runner.PLAN, LOCK_SOURCE_PATHS["protocol_plan"]),
        "runner.RUNNER": (runner.RUNNER, LOCK_SOURCE_PATHS["runner"]),
        "runner.TESTS": (runner.TESTS, LOCK_SOURCE_PATHS["tests"]),
        "runner.H0_CONFIG": (runner.H0_CONFIG, LOCK_INPUT_PATHS["h0_config"]),
        "runner.H0 state": (
            runner.H0_MODULE / "module_state.pkl",
            LOCK_INPUT_PATHS["h0_module_state"],
        ),
        "runner.H0 ctor": (
            runner.H0_MODULE / "class_and_ctor_args.pkl",
            LOCK_INPUT_PATHS["h0_module_ctor"],
        ),
        "runner.H0 metadata": (
            runner.H0_MODULE / "metadata.json",
            LOCK_INPUT_PATHS["h0_module_metadata"],
        ),
    }
    for label, (observed, expected) in exact_paths.items():
        if Path(observed).expanduser().resolve() != Path(expected).resolve():
            raise V3FreezeError(f"pure freeze contract drifted at {label}")
    if (
        tuple(runner.TRAIN_SEEDS) != freeze_contract.TRAIN_SEEDS
        or runner.FINAL_HOLDOUT_SEED != freeze_contract.FINAL_HOLDOUT_SEED
        or tuple(runner.CANONICAL_SEEDS) != expected_seed_order
        or set(HISTORICAL_INPUT_PATHS) != {str(seed) for seed in expected_seed_order}
    ):
        raise V3FreezeError("runner seed governance drifted from pure freeze contract")
    for seed in expected_seed_order:
        directory = runner._historical_directory(seed)
        for role, filename in (
            ("trace", "rollout_policy_trace.json"),
            ("summary", "rollout_summary.json"),
        ):
            if (directory / filename).resolve() != HISTORICAL_INPUT_PATHS[str(seed)][
                role
            ].resolve():
                raise V3FreezeError(
                    f"historical path drifted from pure contract: {seed}.{role}"
                )


def _history_records_without_semantic_access() -> dict[str, Any]:
    # Deliberately hash bytes only.  Do not call runner.historical_inputs,
    # json.load, np.load, or any semantic validator here.
    return {seed: _record_map(paths) for seed, paths in HISTORICAL_INPUT_PATHS.items()}


def _record_map(paths: Mapping[str, Path]) -> dict[str, Any]:
    return {key: _record(path) for key, path in paths.items()}


def _build_lock_payload(policy_id: str) -> dict[str, Any]:
    diagnostic_artifacts = {
        run_id: _record_map(paths)
        for run_id, paths in _diagnostic_artifact_paths().items()
    }
    runtime_closure = {
        "protocol_sources": _record_map(PROTOCOL_SOURCE_PATHS),
        "runtime_sources": _record_map(RUNTIME_SOURCE_PATHS),
        "runtime_inputs": _record_map(RUNTIME_INPUT_PATHS),
        "evidence_receipts": _record_map(EVIDENCE_PATHS),
        "diagnostic_artifacts": diagnostic_artifacts,
        "lineage": _record_map(LINEAGE_PATHS),
    }
    payload = {
        "schema_version": 3,
        "status": "H0_PRIMARY_GRF_SPLIT_V3_EXECUTION_FROZEN",
        "protocol_id": runner.PROTOCOL_ID,
        "revision": runner.REVISION,
        "candidate_id": runner.CANDIDATE_ID,
        "run_root": _repo_relative_posix(RUN_ROOT),
        "expected_steps": runner.EXPECTED_STEPS,
        "canonical_seeds": list(runner.CANONICAL_SEEDS),
        "canonical_offset_s": runner.CANONICAL_OFFSET_S,
        "event_contract_id": runner.EVENT_CONTRACT,
        "so_policy_id": policy_id,
        "so_policy": dict(SO_POLICIES[policy_id]),
        "fit": dict(runner.FIT),
        "mutable_feature_names": list(runner.MUTABLE_FEATURE_NAMES),
        "destinations": [
            _repo_relative_posix(path)
            for path in runner._canonical_stage_destinations()
        ],
        "authority": dict(runner.LOCK_AUTHORITY),
        "sources": _record_map(LOCK_SOURCE_PATHS),
        "inputs": {
            **_record_map(LOCK_INPUT_PATHS),
            "historical_inputs": _history_records_without_semantic_access(),
        },
        "runtime_closure": runtime_closure,
        "actor_update_candidate_count": 1,
        "retry_or_retuning_allowed": False,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
    }
    if set(payload) != runner.LOCK_TOP_LEVEL_KEYS:
        raise V3FreezeError("runner lock schema and freezer schema disagree")
    # The selected policy is committed through all PASS evidence receipts,
    # whose immutable records are part of runtime_closure.
    if policy_id not in SO_POLICIES:
        raise V3FreezeError("attempted to freeze an unknown SO policy")
    return payload


def _write_lock(payload: Mapping[str, Any]) -> None:
    runner.common_gates.canonical_json_bytes(payload)
    runner._write_json_exclusive(LOCK, dict(payload))


def freeze() -> dict[str, Any]:
    """Publish only the V3 lock after every prerequisite passes."""
    if os.path.lexists(LOCK):
        raise V3FreezeError(f"refusing to clobber existing lock: {LOCK}")
    if os.path.lexists(RUN_ROOT):
        raise V3FreezeError(
            f"V3 run root must not exist before execution freeze: {RUN_ROOT}"
        )

    _validate_contract_alignment()
    _decision, policy_id = _validate_decision_receipt()
    _validate_diagnostic_receipt(policy_id)
    _validate_preflight_receipt(policy_id)
    _validate_test_receipt()
    _validate_platform_receipt()
    _validate_lineage()
    payload = _build_lock_payload(policy_id)

    # Recheck immediately before the only authorized mutation.
    if os.path.lexists(LOCK) or os.path.lexists(RUN_ROOT):
        raise V3FreezeError("lock or run root appeared during V3 freeze")
    _write_lock(payload)
    if os.path.lexists(RUN_ROOT):
        raise V3FreezeError("freezer unexpectedly created the V3 run root")
    runner.verify_lock()
    return payload


if __name__ == "__main__":
    print(json.dumps(freeze(), indent=2, sort_keys=True, allow_nan=False))
