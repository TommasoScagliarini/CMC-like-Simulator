"""Execute the frozen V12R2 autonomy-recovery pipeline exactly once.

This module is the execution overlay for the additive V12R2 protocol.  The
design freeze remains read-only.  A separate in-memory design-audit receipt
and this file's no-clobber execution lock must both pass before the run root
can be claimed.  Every stage is ordered, receipt-bound, and terminal on any
integrity failure; retry, sweep, rescue, protected trials, and promotion are
outside this executable surface.
"""

from __future__ import annotations

import argparse
import contextlib
import copy
import hashlib
import math
import os
import platform
import secrets
import sys
import time
from pathlib import Path, PurePosixPath
from typing import Any, Iterator, Mapping, Sequence


def _discover_repo_root(source: Path) -> Path:
    for candidate in source.resolve().parents:
        if (
            (candidate / "AGENTS.md").is_file()
            and (candidate / "validation").is_dir()
            and (candidate / "Trajectory Generator").is_dir()
        ):
            return candidate
    raise RuntimeError("repository root could not be discovered")


REPO_ROOT = _discover_repo_root(Path(__file__))
VALIDATION_ROOT = REPO_ROOT / "validation"
TRAJECTORY_ROOT = REPO_ROOT / "Trajectory Generator"
BASELINE_ROOT = TRAJECTORY_ROOT / "baseline_MLP"
LOCAL_VALIDATION_ROOT = BASELINE_ROOT / "validation"
V12R2_ROOT = Path(__file__).resolve().parent
for _root in (
    REPO_ROOT,
    VALIDATION_ROOT,
    TRAJECTORY_ROOT,
    BASELINE_ROOT,
    LOCAL_VALIDATION_ROOT,
    V12R2_ROOT,
):
    if str(_root) not in sys.path:
        sys.path.insert(0, str(_root))

import h0_forensic_rollout as forensic  # noqa: E402
import h0_primary_split_v10_coherent_teacher as coherent_teacher  # noqa: E402
import h0_primary_split_v10s_blend as safe_dagger  # noqa: E402
import h0_primary_split_v10s_fit as v10s_fit  # noqa: E402
import run_h0_primary_split_v10s_safe_dagger as legacy_runner  # noqa: E402
import run_h0_primary_split_v9_causal_teacher as env_source  # noqa: E402
import h0_primary_split_v12r2_autonomy_recovery_contract as contract  # noqa: E402
import h0_primary_split_v12r2_pure_probe_observer_labeler as label_engine  # noqa: E402
import h0_primary_split_v12r2_recovery_weighted_fitter as fit_engine  # noqa: E402
import run_h0_primary_split_v12r2_design_audit as audit_engine  # noqa: E402


class V12R2ExecutionError(RuntimeError):
    """Raised when V12R2 cannot continue without violating the freeze."""


EXECUTION_SOURCE_RELATIVE_PATHS = {
    "v12r2_recovery_weighted_fitter": (
        "Trajectory Generator/baseline_MLP/validation/v12r2/"
        "h0_primary_split_v12r2_recovery_weighted_fitter.py"
    ),
    "v12r2_pure_probe_observer_labeler": (
        "Trajectory Generator/baseline_MLP/validation/v12r2/"
        "h0_primary_split_v12r2_pure_probe_observer_labeler.py"
    ),
    "v12r2_design_audit_runner": (
        "Trajectory Generator/baseline_MLP/validation/v12r2/"
        "run_h0_primary_split_v12r2_design_audit.py"
    ),
    "v12r2_pipeline_runner": (
        "Trajectory Generator/baseline_MLP/validation/v12r2/"
        "run_h0_primary_split_v12r2_autonomy_recovery.py"
    ),
    "v12r2_execution_tests": (
        "Trajectory Generator/baseline_MLP/validation/v12r2/"
        "test_h0_primary_split_v12r2_execution.py"
    ),
    "v12r2_fitter_tests": (
        "Trajectory Generator/baseline_MLP/validation/v12r2/"
        "test_h0_primary_split_v12r2_recovery_weighted_fit.py"
    ),
    "v12r2_labeler_tests": (
        "Trajectory Generator/baseline_MLP/validation/v12r2/"
        "test_h0_primary_split_v12r2_pure_probe_observer_labeler.py"
    ),
}

EXECUTION_AUTHORITY = {
    "authority_date": "2026-08-09",
    "authority_text": contract.AUTHORITY_TEXT,
    "authority_scope": contract.EXECUTION_AUTHORITY_SCOPE,
    "one_shot": True,
}

_ACTIVE_STAGE_ACTIVITY: dict[str, Any] | None = None


def _activity_increment(name: str, amount: int = 1) -> None:
    if _ACTIVE_STAGE_ACTIVITY is None:
        return
    if type(amount) is not int or amount < 0:
        raise V12R2ExecutionError("stage activity increment is malformed")
    _ACTIVE_STAGE_ACTIVITY[name] = int(_ACTIVE_STAGE_ACTIVITY.get(name, 0)) + amount


def resolve_relative(path: str | os.PathLike[str] | PurePosixPath) -> Path:
    raw = path.as_posix() if isinstance(path, PurePosixPath) else os.fspath(path)
    pure = PurePosixPath(raw)
    if not raw or pure.is_absolute() or ".." in pure.parts or pure.as_posix() != raw:
        raise V12R2ExecutionError(f"non-canonical repository path: {raw!r}")
    return REPO_ROOT.joinpath(*pure.parts)


RUN_ROOT = resolve_relative(contract.RUN_ROOT)
LOCK_PATH = resolve_relative(contract.EXECUTION_LOCK_PATH)
DESIGN_AUDIT_PATH = resolve_relative(contract.DESIGN_AUDIT_RECEIPT_PATH)
PROTOCOL_FREEZE_PATH = resolve_relative(contract.PROTOCOL_FREEZE_PATH)
PIPELINE_CLAIM_PATH = resolve_relative(contract.PIPELINE_CLAIM_PATH)
PIPELINE_LEDGER_PATH = resolve_relative(contract.PIPELINE_LEDGER_PATH)
SOURCE_H0_MODULE = resolve_relative(contract.SOURCE_H0_MODULE_PATH)
CANDIDATE_FREEZE_PATH = resolve_relative(contract.CANDIDATE_FREEZE_PATH)


def _mapping(path: str | Path) -> dict[str, Any]:
    value = forensic.strict_json_load(Path(path))
    if not isinstance(value, Mapping):
        raise V12R2ExecutionError(f"expected strict JSON object: {path}")
    return dict(value)


def _sequence(path: str | Path) -> list[Any]:
    value = forensic.strict_json_load(Path(path))
    if not isinstance(value, list):
        raise V12R2ExecutionError(f"expected strict JSON array: {path}")
    return value


def _record(path: str | Path) -> dict[str, Any]:
    return forensic.artifact_record(Path(path), artifact_root=REPO_ROOT)


def _record_matches(value: Any, path: str | Path) -> bool:
    return isinstance(value, Mapping) and dict(value) == _record(path)


def _tree_record(path: str | Path) -> dict[str, Any]:
    root = Path(path).resolve()
    if not root.is_dir():
        raise V12R2ExecutionError(f"artifact tree is missing: {root}")
    files = sorted(item for item in root.rglob("*") if item.is_file())
    if not files or any(item.is_symlink() for item in files):
        raise V12R2ExecutionError(f"artifact tree is empty or symlinked: {root}")
    digest = hashlib.sha256()
    rows: list[dict[str, Any]] = []
    for item in files:
        relative = item.relative_to(root).as_posix()
        sha256 = forensic.sha256_file(item)
        size_bytes = item.stat().st_size
        rows.append(
            {
                "path": relative,
                "sha256": sha256,
                "size_bytes": size_bytes,
            }
        )
        digest.update(relative.encode("utf-8"))
        digest.update(b"\0")
        digest.update(sha256.encode("ascii"))
        digest.update(b"\0")
        digest.update(str(size_bytes).encode("ascii"))
        digest.update(b"\n")
    return {
        "path": root.relative_to(REPO_ROOT).as_posix(),
        "tree_sha256": digest.hexdigest(),
        "file_count": len(rows),
        "files": rows,
    }


def _prospective_json_record(path: Path, payload: Any) -> dict[str, Any]:
    encoded = forensic.canonical_json_bytes(payload)
    return {
        "path": path.resolve().relative_to(REPO_ROOT).as_posix(),
        "sha256": hashlib.sha256(encoded).hexdigest(),
        "size_bytes": len(encoded),
    }


def _execution_source_records() -> dict[str, dict[str, Any]]:
    expected = set(getattr(contract, "FUTURE_EXECUTION_SOURCES_REQUIRED", ()))
    observed = set(EXECUTION_SOURCE_RELATIVE_PATHS)
    if expected and observed != expected:
        raise V12R2ExecutionError(
            f"execution source names drifted: {sorted(observed)} != {sorted(expected)}"
        )
    return {
        name: _record(resolve_relative(relative))
        for name, relative in sorted(EXECUTION_SOURCE_RELATIVE_PATHS.items())
    }


def _v12r1_lineage_records() -> dict[str, dict[str, Any]]:
    records = {
        name: _record(resolve_relative(artifact["path"]))
        for name, artifact in contract.V12R1_LINEAGE_ARTIFACTS.items()
    }
    if records != contract.V12R1_LINEAGE_ARTIFACTS:
        raise V12R2ExecutionError("terminal V12R1 lineage drifted")
    return records


def _inherited_runtime_evidence_audit() -> dict[str, Any]:
    """Rebind every V11 runtime source and input to its current exact bytes."""

    lock_path = resolve_relative(contract.V11_EXECUTION_LOCK_ARTIFACT["path"])
    lock_record = _record(lock_path)
    lock = _mapping(lock_path)
    frozen_sources = lock.get("sources")
    frozen_inputs = lock.get("inputs")
    if not isinstance(frozen_sources, Mapping) or not isinstance(
        frozen_inputs, Mapping
    ):
        raise V12R2ExecutionError("V11 execution lock evidence is malformed")

    def current_records(
        frozen: Mapping[str, Any], *, label: str
    ) -> dict[str, dict[str, Any]]:
        current: dict[str, dict[str, Any]] = {}
        for name, expected in sorted(frozen.items()):
            if (
                not isinstance(expected, Mapping)
                or set(expected) != {"path", "sha256", "size_bytes"}
                or not isinstance(expected.get("path"), str)
            ):
                raise V12R2ExecutionError(
                    f"V11 {label} artifact record is malformed: {name}"
                )
            current[name] = _record(resolve_relative(str(expected["path"])))
        return current

    current_sources = current_records(frozen_sources, label="source")
    current_inputs = current_records(frozen_inputs, label="input")
    sources_exact = current_sources == dict(frozen_sources)
    inputs_exact = current_inputs == dict(frozen_inputs)
    checks = {
        "v11_lock_artifact_exact": lock_record == contract.V11_EXECUTION_LOCK_ARTIFACT,
        "v11_lock_pass": lock.get("passed") is True,
        "source_count_exact": len(current_sources) == 66,
        "input_count_exact": len(current_inputs) == 54,
        "all_sources_current": sources_exact,
        "all_inputs_current": inputs_exact,
    }
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "passed": all(checks.values()),
        "protocol_id": contract.PROTOCOL_ID,
        "v11_execution_lock": lock_record,
        "source_count": len(current_sources),
        "input_count": len(current_inputs),
        "sources_exact": sources_exact,
        "inputs_exact": inputs_exact,
        "checks": checks,
        "sources": current_sources,
        "inputs": current_inputs,
    }


def _runtime_record() -> dict[str, Any]:
    import numpy as np
    import ray
    import scipy
    import torch
    from ray.rllib.core.rl_module.rl_module import RLModule

    (
        rollout_eval,
        runtime_np,
        runtime_torch,
        runtime_rlmodule,
        env_factory,
        legacy,
        v26_collector,
    ) = _load_rollout_stack()
    if (
        runtime_np is not np
        or runtime_torch is not torch
        or runtime_rlmodule is not RLModule
    ):
        raise V12R2ExecutionError("execution inference stack identity drifted")
    plugin_readiness = _platform_plugin_readiness()
    if plugin_readiness.get("passed") is not True:
        raise V12R2ExecutionError("platform plugin binaries are not training-ready")
    return {
        "system": platform.system(),
        "machine": platform.machine(),
        "python": platform.python_version(),
        "implementation": platform.python_implementation(),
        "numpy": str(np.__version__),
        "scipy": str(scipy.__version__),
        "torch": str(torch.__version__),
        "ray": str(ray.__version__),
        "rllib_rlmodule": f"{RLModule.__module__}.{RLModule.__qualname__}",
        "rollout_eval_module": str(rollout_eval.__name__),
        "env_factory_module": str(env_factory.__name__),
        "legacy_collector_module": str(legacy.__name__),
        "v26_collector_module": str(v26_collector.__name__),
        "inference_stack_ready": True,
        "platform_plugin_readiness": plugin_readiness,
        "executable": str(Path(sys.executable).resolve()),
    }


def _platform_plugin_readiness() -> dict[str, Any]:
    """Bind the exact plugin binaries required by this host platform."""

    system = platform.system()
    if system == "Darwin":
        expected = {
            "sea": "plugins/libSEA_Plugin_BlackBox_mCMC_impedence_ff.dylib",
            "online_grf": "plugins/libOnlineGRFContact.dylib",
        }
        supported_machine = platform.machine() in {"arm64", "x86_64"}
    elif system == "Windows":
        expected = {
            "sea": "plugins/SEA_Plugin_BlackBox_mCMC_impedence_ff.dll",
            "online_grf": "plugins/OnlineGRFContact.dll",
        }
        supported_machine = platform.machine().lower() in {
            "amd64",
            "x86_64",
            "x64",
        }
    elif system == "Linux":
        expected = {
            "sea": "plugins/libSEA_Plugin_BlackBox_mCMC_impedence_ff.so",
            "online_grf": "plugins/libOnlineGRFContact.so",
        }
        supported_machine = platform.machine().lower() in {"x86_64", "amd64"}
    else:
        expected = {}
        supported_machine = False
    records: dict[str, Any] = {}
    checks: dict[str, bool] = {"supported_machine": supported_machine}
    for name, relative in expected.items():
        path = resolve_relative(relative)
        safe = path.is_file() and not path.is_symlink()
        checks[f"{name}_binary_safe"] = safe
        if safe:
            records[name] = _record(path)
    checks["all_required_binaries_recorded"] = set(records) == set(expected)
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": (
            "PASS_H0_PRIMARY_SPLIT_V12R2_PLATFORM_PLUGINS"
            if all(checks.values())
            else "FAIL_H0_PRIMARY_SPLIT_V12R2_PLATFORM_PLUGINS"
        ),
        "passed": bool(expected) and all(checks.values()),
        "protocol_id": contract.PROTOCOL_ID,
        "system": system,
        "machine": platform.machine(),
        "expected_paths": expected,
        "records": records,
        "checks": checks,
    }


class _LegacyContractAdapter:
    """Expose R1 first and historical rollout-only constants as fallback."""

    @staticmethod
    def _lineage() -> Iterator[Any]:
        value: Any = contract
        seen: set[int] = set()
        while value is not None and id(value) not in seen:
            seen.add(id(value))
            yield value
            value = getattr(value, "prior", None)

    def __getattr__(self, name: str) -> Any:
        for value in self._lineage():
            if hasattr(value, name):
                return getattr(value, name)
        raise AttributeError(name)


_LEGACY_CONTRACT = _LegacyContractAdapter()


@contextlib.contextmanager
def _legacy_contract_bound() -> Iterator[None]:
    previous = legacy_runner.contract
    legacy_runner.contract = _LEGACY_CONTRACT
    try:
        yield
    finally:
        legacy_runner.contract = previous


def _query_mean_std(module: Any, actor: Any, *, np: Any, torch: Any) -> tuple[Any, Any]:
    with _legacy_contract_bound():
        return legacy_runner._query_mean_std(module, actor, np=np, torch=torch)


def _validate_runtime_layout(
    *, module: Any, env: Any, observation: Any, rollout_eval: Any, np: Any
) -> tuple[tuple[str, ...], tuple[str, ...]]:
    with _legacy_contract_bound():
        return legacy_runner._validate_runtime_layout(
            module=module,
            env=env,
            observation=observation,
            rollout_eval=rollout_eval,
            np=np,
        )


def _frozen_innovations(case_id: str, *, action_selection: str, np: Any) -> Any:
    with _legacy_contract_bound():
        return legacy_runner._frozen_innovations(
            case_id, action_selection=action_selection, np=np
        )


def _new_physical_audit(
    *, reset_info: Mapping[str, Any], legacy: Any, np: Any
) -> dict[str, Any]:
    with _legacy_contract_bound():
        return legacy_runner._new_physical_audit(
            reset_info=reset_info, legacy=legacy, np=np
        )


def _consume_physical_step(audit: dict[str, Any], **kwargs: Any) -> dict[str, Any]:
    with _legacy_contract_bound():
        return legacy_runner._consume_physical_step(audit, **kwargs)


def _physical_summary(audit: Mapping[str, Any], **kwargs: Any) -> dict[str, Any]:
    with _legacy_contract_bound():
        return legacy_runner._physical_summary(audit, **kwargs)


def _phase_state(info: Mapping[str, Any]) -> str:
    with _legacy_contract_bound():
        return legacy_runner._phase_state(info)


def _teacher_evidence_path() -> Path:
    return resolve_relative(contract.TEACHER_EVIDENCE_RECEIPT_PATH)


def _assert_execution_authority() -> None:
    if EXECUTION_AUTHORITY != {
        "authority_date": contract.REVISION,
        "authority_text": contract.AUTHORITY_TEXT,
        "authority_scope": contract.EXECUTION_AUTHORITY_SCOPE,
        "one_shot": True,
    }:
        raise V12R2ExecutionError("execution authority schema/identity drifted")


def _design_audit_gate(payload: Mapping[str, Any]) -> dict[str, Any]:
    verified = audit_engine.verify_design_audit()
    if dict(payload) != verified:
        raise V12R2ExecutionError("design audit receipt/current bindings drifted")
    gate_function = getattr(contract, "design_audit_gate", None)
    if not callable(gate_function):
        raise V12R2ExecutionError("R2 contract lacks design_audit_gate")
    gate = dict(gate_function(payload))
    if gate.get("passed") is not True:
        failed = [
            name for name, value in gate.get("checks", {}).items() if value is not True
        ]
        raise V12R2ExecutionError(f"design audit is not PASS: {failed}")
    return gate


def build_execution_lock(*, require_unoccupied: bool = True) -> dict[str, Any]:
    """Build the exact lock payload without opening the run root."""

    _assert_execution_authority()
    if not PROTOCOL_FREEZE_PATH.is_file():
        raise V12R2ExecutionError("canonical R2 protocol freeze is missing")
    protocol = _mapping(PROTOCOL_FREEZE_PATH)
    if protocol.get("passed") is not True:
        raise V12R2ExecutionError("R2 protocol freeze is not PASS")
    audit = _mapping(DESIGN_AUDIT_PATH)
    audit_gate = _design_audit_gate(audit)
    occupancy = {
        "execution_lock_absent": not os.path.lexists(LOCK_PATH),
        "run_root_absent": not os.path.lexists(RUN_ROOT),
        "pipeline_claim_absent": not os.path.lexists(PIPELINE_CLAIM_PATH),
        "pipeline_ledger_absent": not os.path.lexists(PIPELINE_LEDGER_PATH),
    }
    coverage = fit_engine.coverage_reference_audit()
    fit_contract = contract.fit_contract_self_check()
    r1_lineage = _v12r1_lineage_records()
    runtime = _runtime_record()
    inherited = _inherited_runtime_evidence_audit()
    checks = {
        "protocol_freeze_pass": protocol.get("passed") is True,
        "protocol_identity": protocol.get("protocol_id") == contract.PROTOCOL_ID,
        "design_audit_pass": audit_gate.get("passed") is True,
        "coverage_reference_parity": coverage.get("passed") is True,
        "fit_contract_wire_schema_total": fit_contract.get("passed") is True,
        "v12r1_terminal_lineage_exact": (
            r1_lineage == contract.V12R1_LINEAGE_ARTIFACTS
        ),
        "source_h0_exact": (
            _tree_record(SOURCE_H0_MODULE).get("tree_sha256")
            == contract.SOURCE_H0_TREE_SHA256
        ),
        "teacher_evidence_exact": _record(_teacher_evidence_path())
        == contract.TEACHER_EVIDENCE_ARTIFACT,
        "stage_order_exact": tuple(contract.STAGE_IDS)
        == tuple(
            contract.stage_descriptor(stage) and stage for stage in contract.STAGE_IDS
        ),
        "execution_sources_complete": set(_execution_source_records())
        == set(EXECUTION_SOURCE_RELATIVE_PATHS),
        "inference_stack_ready": runtime.get("inference_stack_ready") is True,
        "inherited_runtime_evidence_exact": inherited.get("passed") is True,
        **occupancy,
    }
    passed = all(value is True for value in checks.values())
    payload = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": (
            getattr(
                contract,
                "EXECUTION_LOCK_PASS_STATUS",
                "PASS_H0_PRIMARY_SPLIT_V12R2_EXECUTION_LOCK",
            )
            if passed
            else getattr(
                contract,
                "EXECUTION_LOCK_FAIL_STATUS",
                "FAIL_H0_PRIMARY_SPLIT_V12R2_EXECUTION_LOCK",
            )
        ),
        "passed": passed,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "revision": contract.REVISION,
        "checks": checks,
        "execution_authority": copy.deepcopy(EXECUTION_AUTHORITY),
        "protocol_freeze": _record(PROTOCOL_FREEZE_PATH),
        "v12_protocol_freeze": {
            "path": contract.V12_PROTOCOL_FREEZE_PATH.as_posix(),
            "sha256": contract.V12_PROTOCOL_FREEZE_SHA256,
            "size_bytes": contract.V12_PROTOCOL_FREEZE_SIZE_BYTES,
        },
        "design_audit": _record(DESIGN_AUDIT_PATH),
        "design_audit_gate_passed": audit_gate.get("passed") is True,
        "design_audit_gate": audit_gate,
        "coverage_reference_audit": coverage,
        "fit_contract_self_check": fit_contract,
        "v12r1_terminal_failure_lineage": r1_lineage,
        "source_h0": _tree_record(SOURCE_H0_MODULE),
        "teacher_evidence": _record(_teacher_evidence_path()),
        "execution_sources": _execution_source_records(),
        "inherited_runtime_evidence": inherited,
        "runtime": runtime,
        "training_ready": passed,
        "training_ready_scope": {
            "system": runtime["system"],
            "machine": runtime["machine"],
            "executable": runtime["executable"],
        },
        "stage_order": list(contract.STAGE_IDS),
        "run_root": contract.RUN_ROOT.as_posix(),
        "declared_mutation_paths": {
            name: path.as_posix()
            for name, path in contract.declared_mutation_paths().items()
        },
        "pipeline_claim_preexisting": False,
        "p0_reproduction_tolerance": copy.deepcopy(contract.P0_REPRODUCTION_TOLERANCE),
        "retry_authorized": False,
        "sweep_authorized": False,
        "rescue_authorized": False,
        "post_hoc_retuning_authorized": False,
        "actor_fit_executions": 0,
        "environment_reset_calls": 0,
        "environment_step_calls": 0,
        "offline_teacher_label_calls": 0,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
        "next_stage": "EXECUTE_V12R2_ONCE" if passed else "STOP",
    }
    gate_function = getattr(contract, "execution_lock_gate", None)
    if callable(gate_function):
        gate = dict(gate_function(payload))
        payload["contract_gate"] = gate
        if gate.get("passed") is not True:
            passed = False
    if require_unoccupied and (not passed or payload.get("passed") is not True):
        failed = [name for name, value in checks.items() if value is not True]
        raise V12R2ExecutionError(f"execution lock preflight failed: {failed}")
    return payload


def prepare_execution_lock() -> dict[str, Any]:
    if os.path.lexists(LOCK_PATH):
        raise V12R2ExecutionError("execution lock exists/no-clobber")
    payload = build_execution_lock(require_unoccupied=True)
    forensic.write_json_exclusive(LOCK_PATH, payload)
    return verify_execution_lock(require_run_root_absent=True)


def verify_execution_lock(*, require_run_root_absent: bool = False) -> dict[str, Any]:
    observed = _mapping(LOCK_PATH)
    protocol = _mapping(PROTOCOL_FREEZE_PATH)
    if (
        protocol.get("schema_version") != contract.SCHEMA_VERSION
        or protocol.get("protocol_id") != contract.PROTOCOL_ID
        or protocol.get("passed") is not True
        or protocol.get("execution_lock") is not None
        or protocol.get("actor_fit_executions") != 0
        or protocol.get("environment_reset_calls") != 0
        or protocol.get("environment_step_calls") != 0
    ):
        raise V12R2ExecutionError("protocol freeze content drifted")
    audit = _mapping(DESIGN_AUDIT_PATH)
    audit_gate = _design_audit_gate(audit)
    coverage = fit_engine.coverage_reference_audit()
    fit_contract = contract.fit_contract_self_check()
    r1_lineage = _v12r1_lineage_records()
    source_h0 = _tree_record(SOURCE_H0_MODULE)
    teacher = _record(_teacher_evidence_path())
    sources = _execution_source_records()
    runtime = _runtime_record()
    inherited = _inherited_runtime_evidence_audit()
    checks = {
        "protocol_freeze_pass": True,
        "protocol_identity": True,
        "design_audit_pass": audit_gate.get("passed") is True,
        "coverage_reference_parity": coverage.get("passed") is True,
        "fit_contract_wire_schema_total": fit_contract.get("passed") is True,
        "v12r1_terminal_lineage_exact": (
            r1_lineage == contract.V12R1_LINEAGE_ARTIFACTS
        ),
        "source_h0_exact": source_h0.get("tree_sha256")
        == contract.SOURCE_H0_TREE_SHA256,
        "teacher_evidence_exact": teacher == contract.TEACHER_EVIDENCE_ARTIFACT,
        "stage_order_exact": True,
        "execution_sources_complete": set(sources)
        == set(EXECUTION_SOURCE_RELATIVE_PATHS),
        "inference_stack_ready": runtime.get("inference_stack_ready") is True,
        "inherited_runtime_evidence_exact": inherited.get("passed") is True,
        # These are the immutable pre-publication occupancy facts captured by
        # the lock, not assertions about the later execution state.
        "execution_lock_absent": True,
        "run_root_absent": True,
        "pipeline_claim_absent": True,
        "pipeline_ledger_absent": True,
    }
    if not all(value is True for value in checks.values()):
        failed = [name for name, value in checks.items() if value is not True]
        raise V12R2ExecutionError(f"execution lock bindings failed: {failed}")
    expected = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.EXECUTION_LOCK_PASS_STATUS,
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "revision": contract.REVISION,
        "checks": checks,
        "execution_authority": copy.deepcopy(EXECUTION_AUTHORITY),
        "protocol_freeze": _record(PROTOCOL_FREEZE_PATH),
        "v12_protocol_freeze": {
            "path": contract.V12_PROTOCOL_FREEZE_PATH.as_posix(),
            "sha256": contract.V12_PROTOCOL_FREEZE_SHA256,
            "size_bytes": contract.V12_PROTOCOL_FREEZE_SIZE_BYTES,
        },
        "design_audit": _record(DESIGN_AUDIT_PATH),
        "design_audit_gate_passed": True,
        "design_audit_gate": audit_gate,
        "coverage_reference_audit": coverage,
        "fit_contract_self_check": fit_contract,
        "v12r1_terminal_failure_lineage": r1_lineage,
        "source_h0": source_h0,
        "teacher_evidence": teacher,
        "execution_sources": sources,
        "inherited_runtime_evidence": inherited,
        "runtime": runtime,
        "training_ready": True,
        "training_ready_scope": {
            "system": runtime["system"],
            "machine": runtime["machine"],
            "executable": runtime["executable"],
        },
        "stage_order": list(contract.STAGE_IDS),
        "run_root": contract.RUN_ROOT.as_posix(),
        "declared_mutation_paths": {
            name: path.as_posix()
            for name, path in contract.declared_mutation_paths().items()
        },
        "pipeline_claim_preexisting": False,
        "p0_reproduction_tolerance": copy.deepcopy(contract.P0_REPRODUCTION_TOLERANCE),
        "retry_authorized": False,
        "sweep_authorized": False,
        "rescue_authorized": False,
        "post_hoc_retuning_authorized": False,
        "actor_fit_executions": 0,
        "environment_reset_calls": 0,
        "environment_step_calls": 0,
        "offline_teacher_label_calls": 0,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
        "next_stage": "EXECUTE_V12R2_ONCE",
    }
    expected["contract_gate"] = dict(contract.execution_lock_gate(expected))
    if expected["contract_gate"].get("passed") is not True or observed != expected:
        raise V12R2ExecutionError("execution lock exact payload drifted")
    if require_run_root_absent and os.path.lexists(RUN_ROOT):
        raise V12R2ExecutionError("V12R2 run root already claimed")
    return observed


def _token_sha256(token: str) -> str:
    if not isinstance(token, str) or len(token) < 32:
        raise V12R2ExecutionError("execution token is malformed")
    return hashlib.sha256(token.encode("utf-8")).hexdigest()


def _claim_path(stage_id: str) -> Path:
    return resolve_relative(contract.worker_claim_path(stage_id))


def _stage_receipt_path(stage_id: str) -> Path:
    return resolve_relative(contract.stage_receipt_path(stage_id))


def _verify_pipeline_claim() -> dict[str, Any]:
    claim = _mapping(PIPELINE_CLAIM_PATH)
    token_sha256 = claim.get("execution_token_sha256")
    if not isinstance(token_sha256, str) or len(token_sha256) != 64:
        raise V12R2ExecutionError("pipeline claim token digest is malformed")
    expected = _pipeline_claim_payload(token_sha256)
    if claim != expected:
        raise V12R2ExecutionError("pipeline claim/current lock binding drifted")
    return claim


def _verify_worker_claim(stage_id: str) -> dict[str, Any]:
    pipeline = _verify_pipeline_claim()
    index = contract.STAGE_IDS.index(stage_id)
    previous = [
        {
            "stage_id": prior_stage,
            "receipt": _record(_stage_receipt_path(prior_stage)),
        }
        for prior_stage in contract.STAGE_IDS[:index]
    ]
    expected = _worker_claim_payload(
        stage_id=stage_id,
        token_sha256=str(pipeline["execution_token_sha256"]),
        previous_receipts=previous,
    )
    claim = _mapping(_claim_path(stage_id))
    if claim != expected:
        raise V12R2ExecutionError(f"worker claim chain drifted: {stage_id}")
    return claim


def _pipeline_claim_payload(token_sha256: str) -> dict[str, Any]:
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": "CLAIM_H0_PRIMARY_SPLIT_V12R2_PIPELINE",
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "stage_order": list(contract.STAGE_IDS),
        "execution_token_sha256": token_sha256,
        "execution_lock": _record(LOCK_PATH),
        "authority": copy.deepcopy(EXECUTION_AUTHORITY),
        "retry_authorized": False,
        "sweep_authorized": False,
        "rescue_authorized": False,
        "actor_updates_authorized": 4,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
    }


def _worker_claim_payload(
    *, stage_id: str, token_sha256: str, previous_receipts: Sequence[Mapping[str, Any]]
) -> dict[str, Any]:
    descriptor = contract.stage_descriptor(stage_id)
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": "CLAIM_H0_PRIMARY_SPLIT_V12R2_WORKER",
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "stage_id": stage_id,
        "stage_index": contract.STAGE_IDS.index(stage_id),
        "stage_kind": descriptor["kind"],
        "execution_token_sha256": token_sha256,
        "pipeline_claim": _record(PIPELINE_CLAIM_PATH),
        "previous_receipts": [dict(row) for row in previous_receipts],
        "retry_authorized": False,
        "actor_updates_authorized": int(descriptor["kind"] == "fit"),
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
    }


def _write_worker_claim(stage_id: str, token_sha256: str) -> Path:
    index = contract.STAGE_IDS.index(stage_id)
    previous: list[dict[str, Any]] = []
    for prior_stage in contract.STAGE_IDS[:index]:
        _verify_stage_receipt(prior_stage)
        previous.append(
            {
                "stage_id": prior_stage,
                "receipt": _record(_stage_receipt_path(prior_stage)),
            }
        )
    for later_stage in contract.STAGE_IDS[index + 1 :]:
        if os.path.lexists(_stage_receipt_path(later_stage)) or os.path.lexists(
            _claim_path(later_stage)
        ):
            raise V12R2ExecutionError(
                f"later claim/receipt exists before {stage_id}: {later_stage}"
            )
    claim_path = _claim_path(stage_id)
    if os.path.lexists(claim_path) or os.path.lexists(_stage_receipt_path(stage_id)):
        raise V12R2ExecutionError(f"stage already consumed: {stage_id}")
    return forensic.write_json_exclusive(
        claim_path,
        _worker_claim_payload(
            stage_id=stage_id,
            token_sha256=token_sha256,
            previous_receipts=previous,
        ),
    )


def _load_rollout_stack() -> tuple[Any, Any, Any, Any, Any, Any, Any]:
    import numpy as np
    import torch
    from ray.rllib.core.rl_module.rl_module import RLModule

    rollout_eval, runtime_np, runtime_torch, runtime_rlmodule, env_factory, _reward = (
        env_source.source_collector.engine.legacy._load_inference_stack()
    )
    if (
        runtime_np is not np
        or runtime_torch is not torch
        or runtime_rlmodule is not RLModule
    ):
        raise V12R2ExecutionError("inference stack identity drifted")
    legacy = env_source.source_collector.engine.legacy
    v26_collector = env_source.source_collector.base
    return rollout_eval, np, torch, RLModule, env_factory, legacy, v26_collector


def _verify_stage_receipt(stage_id: str) -> dict[str, Any]:
    """Recompute the gate and provenance for one already-completed stage."""

    descriptor = contract.stage_descriptor(stage_id)
    kind = descriptor["kind"]
    receipt_path = _stage_receipt_path(stage_id)
    receipt = _mapping(receipt_path)
    _verify_worker_claim(stage_id)
    if kind == "fit":
        return fit_engine.verify_fit_stage(descriptor["fit_stage"])
    if kind == "probe":
        stage = descriptor["fit_stage"]
        root = resolve_relative(contract.PROBE_ROOT / stage)
        summary = _mapping(root / "summary.json")
        gate = _mapping(root / "gate.json")
        expected = dict(contract.pure_probe_gate(summary, stage=stage))
        if gate != expected or gate.get("integrity_passed") is not True:
            raise V12R2ExecutionError(f"probe closure drifted: {stage}")
        module = _tree_record(resolve_relative(contract.MODULE_PATHS[stage]))
        candidate_id = contract.candidate_id(stage, module["tree_sha256"])
        expected_receipt = {
            "schema_version": contract.SCHEMA_VERSION,
            "status": gate["status"],
            "passed": bool(gate.get("passed")),
            "integrity_passed": bool(gate.get("integrity_passed")),
            "recoverable_for_data_collection": bool(
                gate.get("recoverable_for_data_collection")
            ),
            "protocol_id": contract.PROTOCOL_ID,
            "stage_id": stage_id,
            "fit_stage": stage,
            "probe_step_count": summary.get("steps"),
            "candidate_id": candidate_id,
            "candidate_module_tree_sha256": module["tree_sha256"],
            "summary": _record(root / "summary.json"),
            "gate": _record(root / "gate.json"),
            "trace": _record(root / "trace.json"),
            "replay_payload": _record(root / "replay_boundaries.npz"),
            "replay_schema": copy.deepcopy(contract.PROBE_REPLAY_SCHEMA),
            "pipeline_claim": _record(PIPELINE_CLAIM_PATH),
            "worker_claim": _record(_claim_path(stage_id)),
            "retry_authorized": False,
            "actor_updates": 0,
            "critic_updates": 0,
            "ppo_updates": 0,
            "protected_trials_opened": [],
            "reserve_trials_opened": [],
        }
        current_bindings = (
            summary.get("candidate_module") == module
            and summary.get("candidate_id") == candidate_id
            and summary.get("fit_receipt")
            == _record(resolve_relative(contract.FIT_RECEIPT_PATHS[stage]))
            and summary.get("run_start") == _record(root / "run_start.json")
            and summary.get("trace") == _record(root / "trace.json")
            and summary.get("partial_summary") == _record(root / "partial_summary.json")
            and summary.get("replay_payload") == _record(root / "replay_boundaries.npz")
            and summary.get("worker_claim") == _record(_claim_path(stage_id))
        )
        if receipt != expected_receipt or not current_bindings:
            raise V12R2ExecutionError(f"probe receipt/binding drifted: {stage}")
        return receipt
    if kind == "label":
        return label_engine.verify_observer_label_stage(descriptor["fit_stage"])
    if kind == "collection":
        case = descriptor["case"]
        root = resolve_relative(case["destination"])
        summary = _mapping(root / "summary.json")
        gates = _mapping(root / "gate.json")
        data_gate = dict(
            contract.collection_data_gate(
                summary, round_index=descriptor["round_index"]
            )
        )
        latch_gate = dict(
            contract.latch_dependence_gate(
                summary, collection_data_passed=data_gate.get("passed") is True
            )
        )
        if gates != {"data_gate": data_gate, "latch_dependence_gate": latch_gate}:
            raise V12R2ExecutionError(f"collection gate closure drifted: {stage_id}")
        if data_gate.get("passed") is not True:
            raise V12R2ExecutionError(f"collection data gate failed: {stage_id}")
        import numpy as np

        trace = _sequence(root / "trace.json")
        try:
            with np.load(root / "labels.npz", allow_pickle=False) as archive:
                expected_names = {
                    "observations",
                    "actions",
                    "reset_mask",
                    "actor_feature_names",
                    "case_ids",
                    "step_indices",
                    "tranche_ids",
                    "origins",
                }
                if set(archive.files) != expected_names:
                    raise V12R2ExecutionError(
                        f"collection label schema drifted: {stage_id}"
                    )
                label_arrays = {
                    name: np.ascontiguousarray(archive[name].copy())
                    for name in archive.files
                }
        except V12R2ExecutionError:
            raise
        except Exception as exc:
            raise V12R2ExecutionError(
                f"collection labels are unreadable: {stage_id}"
            ) from exc
        rows = len(trace)
        expected_observations = np.ascontiguousarray(
            np.asarray([row["v26_observation"] for row in trace]),
            dtype=np.float32,
        )
        expected_actions = np.ascontiguousarray(
            np.asarray([row["counterfactual_teacher_mean"] for row in trace]),
            dtype=np.float32,
        )
        expected_steps = np.arange(1, rows + 1, dtype=np.int64)
        trace_corpus_exact = (
            rows == summary.get("sample_count")
            and label_arrays["observations"].dtype == np.dtype(np.float32)
            and label_arrays["observations"].tobytes(order="C")
            == expected_observations.tobytes(order="C")
            and label_arrays["actions"].dtype == np.dtype(np.float32)
            and label_arrays["actions"].tobytes(order="C")
            == expected_actions.tobytes(order="C")
            and np.array_equal(
                label_arrays["reset_mask"],
                np.asarray([index == 0 for index in range(rows)], dtype=np.bool_),
            )
            and tuple(label_arrays["actor_feature_names"].astype(str).tolist())
            == tuple(coherent_teacher.EXPECTED_ACTOR_FEATURE_NAMES)
            and set(label_arrays["case_ids"].astype(str).tolist())
            == {str(case["case_id"])}
            and np.array_equal(label_arrays["step_indices"], expected_steps)
            and set(label_arrays["tranche_ids"].astype(str).tolist())
            == {f"v12r2_collect_r{descriptor['round_index']}"}
            and set(label_arrays["origins"].astype(str).tolist())
            == {"V12R2_SHIELDED_SAME_STATE"}
        )
        if not trace_corpus_exact:
            raise V12R2ExecutionError(
                f"collection labels no longer match trace: {stage_id}"
            )
        fit_stage = str(case["candidate_fit_stage"])
        module = _tree_record(resolve_relative(contract.MODULE_PATHS[fit_stage]))
        candidate_id = contract.candidate_id(fit_stage, module["tree_sha256"])
        expected_receipt = {
            "schema_version": contract.SCHEMA_VERSION,
            "status": contract.COLLECTION_PASS_STATUS,
            "passed": True,
            "protocol_id": contract.PROTOCOL_ID,
            "round_index": descriptor["round_index"],
            "case_id": case["case_id"],
            "sample_count": summary.get("sample_count"),
            "label_corpus": _record(root / "labels.npz"),
            "pipeline_claim": _record(PIPELINE_CLAIM_PATH),
            "worker_claim": _record(_claim_path(stage_id)),
        }
        current_bindings = (
            summary.get("candidate_id") == candidate_id
            and summary.get("candidate_module") == module
            and summary.get("candidate_fit_receipt")
            == _record(resolve_relative(contract.FIT_RECEIPT_PATHS[fit_stage]))
            and summary.get("prior_label_receipt")
            == _record(resolve_relative(contract.LABEL_RECEIPT_PATHS[fit_stage]))
            and summary.get("source_h0") == _tree_record(SOURCE_H0_MODULE)
            and summary.get("label_corpus") == _record(root / "labels.npz")
            and summary.get("run_start") == _record(root / "run_start.json")
            and summary.get("trace") == _record(root / "trace.json")
            and summary.get("partial_summary") == _record(root / "partial_summary.json")
            and summary.get("worker_claim") == _record(_claim_path(stage_id))
        )
        if receipt != expected_receipt or not current_bindings:
            raise V12R2ExecutionError(f"collection receipt/binding drifted: {stage_id}")
        return receipt
    if kind == "freeze":
        summary = _mapping(
            CANDIDATE_FREEZE_PATH.with_name("candidate_freeze_summary.json")
        )
        gate = _mapping(CANDIDATE_FREEZE_PATH.with_name("candidate_freeze_gate.json"))
        if (
            gate != contract.candidate_freeze_gate(summary)
            or gate.get("passed") is not True
        ):
            raise V12R2ExecutionError("candidate freeze closure drifted")
        module = _tree_record(resolve_relative(contract.MODULE_PATHS["p3"]))
        expected_receipt = {
            **summary,
            "status": contract.CANDIDATE_FREEZE_PASS_STATUS,
            "passed": True,
            "stage_id": stage_id,
            "summary": _record(
                CANDIDATE_FREEZE_PATH.with_name("candidate_freeze_summary.json")
            ),
            "gate": _record(
                CANDIDATE_FREEZE_PATH.with_name("candidate_freeze_gate.json")
            ),
        }
        expected_fit_receipts = [
            {
                "fit_stage": fit_stage,
                "receipt": _record(
                    resolve_relative(contract.FIT_RECEIPT_PATHS[fit_stage])
                ),
                "passed": True,
            }
            for fit_stage in contract.FIT_STAGES
        ]
        expected_probe_receipts: list[dict[str, Any]] = []
        expected_label_receipts: list[dict[str, Any]] = []
        for fit_stage in contract.FIT_STAGES:
            probe_gate = _mapping(
                resolve_relative(contract.PROBE_ROOT / fit_stage / "gate.json")
            )
            expected_probe_receipts.append(
                {
                    "fit_stage": fit_stage,
                    "receipt": _record(
                        resolve_relative(contract.PROBE_RECEIPT_PATHS[fit_stage])
                    ),
                    "integrity_passed": probe_gate.get("integrity_passed") is True,
                    "autonomy_passed": probe_gate.get("passed") is True,
                    "recoverable_for_data_collection": bool(
                        probe_gate.get("recoverable_for_data_collection")
                    ),
                }
            )
            expected_label_receipts.append(
                {
                    "fit_stage": fit_stage,
                    "receipt": _record(
                        resolve_relative(contract.LABEL_RECEIPT_PATHS[fit_stage])
                    ),
                    "passed": True,
                }
            )
        expected_collection_receipts: list[dict[str, Any]] = []
        for round_index in (1, 2, 3):
            for case_id in contract.COLLECTION_CASE_IDS:
                root = resolve_relative(
                    contract.canonical_collection_case(case_id, round_index)[
                        "destination"
                    ]
                )
                gates = _mapping(root / "gate.json")
                expected_collection_receipts.append(
                    {
                        "round_index": round_index,
                        "case_id": case_id,
                        "receipt": _record(root / "receipt.json"),
                        "data_passed": gates["data_gate"].get("passed") is True,
                        "latch_independence_passed": gates["latch_dependence_gate"].get(
                            "passed"
                        )
                        is True,
                    }
                )
        current_bindings = (
            summary.get("candidate_module") == module
            and summary.get("candidate_id")
            == contract.candidate_id("p3", module["tree_sha256"])
            and summary.get("source_h0") == _tree_record(SOURCE_H0_MODULE)
            and summary.get("design_audit_receipt") == _record(DESIGN_AUDIT_PATH)
            and summary.get("execution_lock") == _record(LOCK_PATH)
            and summary.get("pipeline_claim") == _record(PIPELINE_CLAIM_PATH)
            and summary.get("worker_claim") == _record(_claim_path(stage_id))
            and summary.get("fit_receipts") == expected_fit_receipts
            and summary.get("probe_receipts") == expected_probe_receipts
            and summary.get("label_receipts") == expected_label_receipts
            and summary.get("collection_receipts") == expected_collection_receipts
        )
        if receipt != expected_receipt or not current_bindings:
            raise V12R2ExecutionError("candidate freeze receipt/binding drifted")
        return receipt
    if kind == "final":
        root = resolve_relative(descriptor["case"]["destination"])
        summary = _mapping(root / "summary.json")
        gate = _mapping(root / "gate.json")
        if (
            gate != contract.final_rollout_gate(summary)
            or gate.get("passed") is not True
        ):
            raise V12R2ExecutionError(f"final closure drifted: {stage_id}")
        module = _tree_record(resolve_relative(contract.MODULE_PATHS["p3"]))
        candidate_id = contract.candidate_id("p3", module["tree_sha256"])
        expected_receipt = {
            "schema_version": contract.SCHEMA_VERSION,
            "status": contract.FINAL_ROLLOUT_PASS_STATUS,
            "passed": True,
            "protocol_id": contract.PROTOCOL_ID,
            "stage_id": stage_id,
            "case_id": descriptor["case"]["case_id"],
            "candidate_id": candidate_id,
            "candidate_module": module,
            "summary": _record(root / "summary.json"),
            "gate": _record(root / "gate.json"),
            "trace": _record(root / "trace.json"),
            "pipeline_claim": _record(PIPELINE_CLAIM_PATH),
            "worker_claim": _record(_claim_path(stage_id)),
            "retry_authorized": False,
            "actor_updates": 0,
            "critic_updates": 0,
            "ppo_updates": 0,
            "protected_trials_opened": [],
            "reserve_trials_opened": [],
        }
        current_bindings = (
            summary.get("candidate_module") == module
            and summary.get("candidate_id") == candidate_id
            and summary.get("candidate_freeze") == _record(CANDIDATE_FREEZE_PATH)
            and summary.get("run_start") == _record(root / "run_start.json")
            and summary.get("trace") == _record(root / "trace.json")
            and summary.get("partial_summary") == _record(root / "partial_summary.json")
            and summary.get("worker_claim") == _record(_claim_path(stage_id))
        )
        if receipt != expected_receipt or not current_bindings:
            raise V12R2ExecutionError(f"final receipt/binding drifted: {stage_id}")
        return receipt
    summary = _mapping(resolve_relative(contract.FINAL_ROOT) / "summary.json")
    gate = _mapping(resolve_relative(contract.FINAL_ROOT) / "gate.json")
    if (
        gate != contract.final_development_gate(summary)
        or gate.get("passed") is not True
    ):
        raise V12R2ExecutionError("final aggregate closure drifted")
    final_root = resolve_relative(contract.FINAL_ROOT)
    module = _tree_record(resolve_relative(contract.MODULE_PATHS["p3"]))
    expected_receipt = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.FINAL_DEVELOPMENT_PASS_STATUS,
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "stage_id": stage_id,
        "candidate_id": contract.candidate_id("p3", module["tree_sha256"]),
        "candidate_module": module,
        "candidate_freeze": _record(CANDIDATE_FREEZE_PATH),
        "summary": _record(final_root / "summary.json"),
        "gate": _record(final_root / "gate.json"),
        "rollout_bindings": summary.get("rollout_bindings"),
        "pipeline_claim": _record(PIPELINE_CLAIM_PATH),
        "worker_claim": _record(_claim_path(stage_id)),
        "development_only": True,
        "runtime_promoted": False,
        "qualification_required": True,
        "retry_authorized": False,
        "sweep_authorized": False,
        "rescue_authorized": False,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
    }
    current_bindings = (
        summary.get("candidate_module") == module
        and summary.get("candidate_freeze") == _record(CANDIDATE_FREEZE_PATH)
        and summary.get("pipeline_claim") == _record(PIPELINE_CLAIM_PATH)
        and summary.get("worker_claim") == _record(_claim_path(stage_id))
        and summary.get("rollout_bindings")
        == [
            {
                "case_id": case_id,
                "passed": True,
                "receipt": _record(
                    resolve_relative(
                        contract.canonical_final_case(case_id)["destination"]
                    )
                    / "receipt.json"
                ),
                "gate": _record(
                    resolve_relative(
                        contract.canonical_final_case(case_id)["destination"]
                    )
                    / "gate.json"
                ),
                "summary": _record(
                    resolve_relative(
                        contract.canonical_final_case(case_id)["destination"]
                    )
                    / "summary.json"
                ),
            }
            for case_id in contract.FINAL_CASE_IDS
        ]
    )
    if receipt != expected_receipt or not current_bindings:
        raise V12R2ExecutionError("final aggregate receipt/binding drifted")
    return receipt


# Rollout and publication routines are defined below.  They deliberately live
# in this content-bound source rather than in an unpinned runtime callback.


def _rollout_records(
    writer: Any,
    *,
    rows: Sequence[Mapping[str, Any]],
    partial: Mapping[str, Any],
) -> dict[str, dict[str, Any]]:
    return {
        "run_start": _record(writer.run_start_path),
        "trace": _prospective_json_record(writer.trace_path, list(rows)),
        "partial_summary": _prospective_json_record(
            writer.partial_summary_path, partial
        ),
    }


def _publish_rollout_before_gate(
    writer: Any,
    *,
    rows: Sequence[Mapping[str, Any]],
    partial: Mapping[str, Any],
    summary: Mapping[str, Any],
) -> None:
    expected = _rollout_records(writer, rows=rows, partial=partial)
    persisted = writer.finalize_before_gate(
        trace=rows,
        partial_summary=partial,
        summary=summary,
    )
    for name, record in expected.items():
        if persisted.get(name) != record:
            raise V12R2ExecutionError(f"prospective rollout record drifted: {name}")


def _run_probe(stage: str) -> dict[str, Any]:
    """Run one unblended candidate probe and persist self-contained replay."""

    (
        rollout_eval,
        np,
        torch,
        RLModule,
        env_factory,
        legacy,
        v26_collector,
    ) = _load_rollout_stack()
    stage_id = f"probe_{stage}"
    case = contract.canonical_probe_case(stage)
    destination = resolve_relative(case["destination"])
    if os.path.lexists(destination):
        raise V12R2ExecutionError(f"probe destination exists: {stage}")
    fit_engine.verify_fit_stage(stage)
    candidate_module_path = resolve_relative(contract.MODULE_PATHS[stage])
    candidate_module = _tree_record(candidate_module_path)
    candidate_id = contract.candidate_id(stage, candidate_module["tree_sha256"])
    candidate = RLModule.from_checkpoint(candidate_module_path)
    candidate.eval()
    innovations = _frozen_innovations(
        str(case["case_id"]),
        action_selection=str(case["action_selection"]),
        np=np,
    )
    env_config = env_source.build_env_config(case)
    env = env_factory.make_cmc_env(env_config)
    writer = forensic.ForensicRolloutWriter(destination, artifact_root=REPO_ROOT)
    writer.start(
        {
            "schema_version": contract.SCHEMA_VERSION,
            "status": "STARTED_H0_PRIMARY_SPLIT_V12R2_PURE_PROBE",
            "protocol_id": contract.PROTOCOL_ID,
            "pipeline_id": contract.PIPELINE_ID,
            "stage_id": stage_id,
            "fit_stage": stage,
            "case": copy.deepcopy(case),
            "behavior": contract.PROBE_BEHAVIOR,
            "candidate_id": candidate_id,
            "candidate_module": candidate_module,
            "fit_receipt": _record(resolve_relative(contract.FIT_RECEIPT_PATHS[stage])),
            "teacher_enabled": False,
            "teacher_loaded_during_rollout": False,
            "blending_enabled": False,
            "safety_latch_enabled": False,
            "probe_replay_schema": copy.deepcopy(contract.PROBE_REPLAY_SCHEMA),
            "pipeline_claim": _record(PIPELINE_CLAIM_PATH),
            "worker_claim": _record(_claim_path(stage_id)),
            "retry_authorized": False,
            "actor_updates": 0,
            "critic_updates": 0,
            "ppo_updates": 0,
            "protected_trials_opened": [],
            "reserve_trials_opened": [],
        }
    )
    rows: list[dict[str, Any]] = []
    actor_names: tuple[str, ...] = ()
    full_names: tuple[str, ...] = ()
    audit: dict[str, Any] | None = None
    info: Mapping[str, Any] = {}
    terminated = False
    truncated = False
    replay_recorder: Any | None = None
    started = time.monotonic()
    try:
        _activity_increment("environment_reset_calls")
        observation, reset_info = env.reset(seed=int(case["runtime_seed"]))
        observation = np.ascontiguousarray(observation, dtype=np.float32)
        actor_names, full_names = _validate_runtime_layout(
            module=candidate,
            env=env,
            observation=observation,
            rollout_eval=rollout_eval,
            np=np,
        )
        audit = _new_physical_audit(
            reset_info=reset_info,
            legacy=legacy,
            np=np,
        )
        body_weight_n = float(env.unwrapped._body_weight_n)
        if not math.isfinite(body_weight_n) or body_weight_n <= 0.0:
            raise V12R2ExecutionError("body weight is malformed")
        replay_recorder = label_engine.PureProbeReplayRecorder.from_runtime(
            env.unwrapped._phase_fsm,
            body_weight_n,
            actor_names,
            event_contract_id=contract.EVENT_CONTRACT_ID,
        )
        actor = np.ascontiguousarray(
            observation[: contract.EXPECTED_ACTOR_FEATURES], dtype=np.float32
        )
        replay_recorder.record_reset(
            actor,
            reset_info,
            previous_penetration_m=0.0,
        )
        for index in range(contract.EXPECTED_STEPS):
            step = index + 1
            observation_before = observation.copy()
            actor_before = actor.copy()
            mean, std = _query_mean_std(candidate, actor_before, np=np, torch=torch)
            noise = np.ascontiguousarray(std * innovations[index], dtype=np.float32)
            raw_action = safe_dagger.apply_single_noise(mean, noise)
            applied = np.ascontiguousarray(
                np.clip(raw_action, env.action_space.low, env.action_space.high),
                dtype=np.float32,
            )
            _activity_increment("environment_step_calls")
            observation_after, reward, terminated, truncated, info = env.step(applied)
            observation_after = np.ascontiguousarray(
                observation_after, dtype=np.float32
            )
            if not isinstance(info, Mapping):
                raise V12R2ExecutionError("probe info is malformed")
            physical = _consume_physical_step(
                audit,
                step=step,
                info=info,
                observation_before=observation_before,
                observation_after=observation_after,
                reward=reward,
                action=raw_action,
                applied_action=applied,
                extra_vectors=(actor_before, mean, std, noise),
                legacy=legacy,
                v26_collector=v26_collector,
            )
            next_actor = np.ascontiguousarray(
                observation_after[: contract.EXPECTED_ACTOR_FEATURES],
                dtype=np.float32,
            )
            terminal_boundary = bool(
                terminated or truncated or step == contract.EXPECTED_STEPS
            )
            replay_recorder.record_step_boundary(
                None if terminal_boundary else next_actor,
                info,
                previous_penetration_m=(
                    None if terminal_boundary else physical["penetration_m"]
                ),
            )
            row = {
                "schema_version": contract.SCHEMA_VERSION,
                "protocol_id": contract.PROTOCOL_ID,
                "stage_id": stage_id,
                "fit_stage": stage,
                "case_id": case["case_id"],
                "v26_observation": actor_before.tolist(),
                "candidate_mean": mean.tolist(),
                "candidate_std": std.tolist(),
                "standard_normal": innovations[index].tolist(),
                "single_noise": noise.tolist(),
                "raw_action": raw_action.tolist(),
                "applied_action": applied.tolist(),
                "teacher_enabled": False,
                "teacher_query_count": 0,
                "served_action_teacher_dependency_count": 0,
                "blending_enabled": False,
                "safety_latch_enabled": False,
                "previous_penetration_m": (
                    0.0 if index == 0 else rows[-1]["grf_penetration_m"]
                ),
                "legacy_online_events": legacy._jsonable(
                    info.get("legacy_online_events", [])
                ),
                "reward": float(reward),
                "time_s": float(info.get("time")),
                "grf_penetration_m": physical["penetration_m"],
                "reserve_norm_nm": physical["reserve_norm_nm"],
                "residual_norm_nm": physical["residual_norm_nm"],
                "phase_fsm": legacy._jsonable(physical["phase"]),
                "checks": physical["checks"],
                "terminated": bool(terminated),
                "truncated": bool(truncated),
                "end_reason": info.get("end_reason"),
            }
            writer.write_step(step, row)
            rows.append({"step": step, **row})
            observation = observation_after
            actor = next_actor
            if step == 1 or step % 25 == 0:
                elapsed = time.monotonic() - started
                eta = elapsed / step * (contract.EXPECTED_STEPS - step)
                print(
                    f"[V12R2 probe/{stage}] {step:3d}/"
                    f"{contract.EXPECTED_STEPS} elapsed={elapsed:7.1f}s "
                    f"eta={eta:7.1f}s",
                    flush=True,
                )
            if terminated or truncated:
                break
    except BaseException as exc:
        try:
            writer.publish_failure(
                end_reason="v12r2_pure_probe_failed",
                error=exc,
                status=contract.PROBE_INTEGRITY_FAIL_STATUS,
            )
        except Exception:
            pass
        raise
    finally:
        env.close()
    if audit is None or replay_recorder is None:
        raise V12R2ExecutionError("probe audit/replay was not initialized")
    replay_path = destination / "replay_boundaries.npz"
    replay_recorder.write_exclusive(replay_path)
    replay = label_engine.load_probe_replay_strict(
        replay_path,
        contract_module=contract,
    )
    if replay.n_steps != len(rows) or replay.boundary_count != len(rows) + 1:
        raise V12R2ExecutionError("probe replay row/boundary count drifted")
    partial = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": "PERSISTED_H0_PRIMARY_SPLIT_V12R2_PROBE_BEFORE_GATE",
        "protocol_id": contract.PROTOCOL_ID,
        "stage_id": stage_id,
        "fit_stage": stage,
        "steps": len(rows),
        "gate_evaluated": False,
        "teacher_query_count": 0,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
    }
    artifacts = _rollout_records(writer, rows=rows, partial=partial)
    summary = {
        **_physical_summary(
            audit,
            case=case,
            rows=rows,
            info=info,
            terminated=terminated,
            truncated=truncated,
            actor_names=actor_names,
            full_names=full_names,
            legacy=legacy,
            v26_collector=v26_collector,
        ),
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.PURE_PROBE_COMPLETE_STATUS,
        "protocol_id": contract.PROTOCOL_ID,
        "candidate_fit_stage": stage,
        "behavior": contract.PROBE_BEHAVIOR,
        "candidate_id": candidate_id,
        "candidate_module": candidate_module,
        "fit_receipt": _record(resolve_relative(contract.FIT_RECEIPT_PATHS[stage])),
        "fit_receipt_passed": True,
        "fit_gate_passed": True,
        "trace_step_count": len(rows),
        "candidate_mean_query_count": len(rows),
        "teacher_enabled": False,
        "teacher_loaded_during_rollout": False,
        "teacher_query_count": 0,
        "served_action_teacher_dependency_count": 0,
        "blending_enabled": False,
        "mean_blend_count": 0,
        "safety_latch_enabled": False,
        "safety_intervention_count": 0,
        "safety_latch_activation_count": 0,
        "safety_latch_release_count": 0,
        "latch_active_at_episode_end": False,
        "multiple_noise_application_count": 0,
        "noise_application_mismatch_count": 0,
        "offline_teacher_replay_boundary_count": replay.boundary_count,
        "previous_penetration_metadata_count": replay.n_steps,
        "replay_schema": copy.deepcopy(contract.PROBE_REPLAY_SCHEMA),
        "replay_payload": _record(replay_path),
        "physical_gate_bypass_count": 0,
        "target_contract_id": contract.TARGET_CONTRACT_ID,
        "logstd_byte_exact": True,
        "disabled_clock_column_indices": [0, 1],
        "disabled_clock_columns_bit_zero": True,
        "normalization_folded_into_first_layer": True,
        "runtime_normalization_wrapper_present": False,
        "worker_claim": _record(_claim_path(stage_id)),
        **artifacts,
    }
    _publish_rollout_before_gate(
        writer,
        rows=rows,
        partial=partial,
        summary=summary,
    )
    gate = dict(contract.pure_probe_gate(summary, stage=stage))
    writer.publish_gate(gate)
    receipt = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": gate["status"],
        "passed": bool(gate.get("passed")),
        "integrity_passed": bool(gate.get("integrity_passed")),
        "recoverable_for_data_collection": bool(
            gate.get("recoverable_for_data_collection")
        ),
        "protocol_id": contract.PROTOCOL_ID,
        "stage_id": stage_id,
        "fit_stage": stage,
        "probe_step_count": len(rows),
        "candidate_id": candidate_id,
        "candidate_module_tree_sha256": candidate_module["tree_sha256"],
        "summary": _record(writer.summary_path),
        "gate": _record(writer.gate_path),
        "trace": _record(writer.trace_path),
        "replay_payload": _record(replay_path),
        "replay_schema": copy.deepcopy(contract.PROBE_REPLAY_SCHEMA),
        "pipeline_claim": _record(PIPELINE_CLAIM_PATH),
        "worker_claim": _record(_claim_path(stage_id)),
        "retry_authorized": False,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
    }
    forensic.write_json_exclusive(destination / "receipt.json", receipt)
    _verify_stage_receipt(stage_id)
    if gate.get("integrity_passed") is not True:
        raise V12R2ExecutionError(f"probe integrity failed: {stage}")
    if stage == "p3" and gate.get("passed") is not True:
        raise V12R2ExecutionError("P3 autonomy probe failed terminally")
    return receipt


def _probe_evidence(stage: str) -> dict[str, Any]:
    _verify_stage_receipt(f"probe_{stage}")
    root = resolve_relative(contract.PROBE_ROOT / stage)
    receipt = _mapping(root / "receipt.json")
    gate = _mapping(root / "gate.json")
    return {
        "protocol_id": contract.PROTOCOL_ID,
        "fit_stage": stage,
        "integrity_passed": gate.get("integrity_passed"),
        "passed": gate.get("passed"),
        "recoverable_for_data_collection": gate.get("recoverable_for_data_collection"),
        "probe_step_count": receipt.get("probe_step_count"),
        "candidate_id": receipt.get("candidate_id"),
        "candidate_module_tree_sha256": receipt.get("candidate_module_tree_sha256"),
        "trace": _record(root / "trace.json"),
        "replay_payload": _record(root / "replay_boundaries.npz"),
        "replay_schema": copy.deepcopy(contract.PROBE_REPLAY_SCHEMA),
        "receipt": _record(root / "receipt.json"),
        "gate_artifact": _record(root / "gate.json"),
    }


def _run_label(stage: str) -> dict[str, Any]:
    _activity_increment("offline_teacher_label_stage_calls")
    result = label_engine.run_observer_label_stage(
        stage=stage,
        probe_evidence=_probe_evidence(stage),
        pipeline_claim_path=PIPELINE_CLAIM_PATH,
        worker_claim_path=_claim_path(f"label_{stage}"),
    )
    if result.get("passed") is not True:
        raise V12R2ExecutionError(f"observer label gate failed: {stage}")
    _activity_increment(
        "offline_teacher_label_calls_confirmed",
        int(result.get("labelled_row_count", 0)),
    )
    return _verify_stage_receipt(f"label_{stage}")


def _run_collection(round_index: int, case_id: str) -> dict[str, Any]:
    """Collect one preregistered shielded same-state tranche."""

    (
        rollout_eval,
        np,
        torch,
        RLModule,
        env_factory,
        legacy,
        v26_collector,
    ) = _load_rollout_stack()
    stage_id = f"collect_r{round_index}__{case_id}"
    case = contract.canonical_collection_case(case_id, round_index)
    destination = resolve_relative(case["destination"])
    if os.path.lexists(destination):
        raise V12R2ExecutionError(f"collection destination exists: {stage_id}")
    fit_stage = str(case["candidate_fit_stage"])
    fit_engine.verify_fit_stage(fit_stage)
    label_engine.verify_observer_label_stage(fit_stage)
    candidate_module_path = resolve_relative(contract.MODULE_PATHS[fit_stage])
    candidate_module_record = _tree_record(candidate_module_path)
    candidate = RLModule.from_checkpoint(candidate_module_path)
    teacher = RLModule.from_checkpoint(SOURCE_H0_MODULE)
    candidate.eval()
    teacher.eval()
    innovations = _frozen_innovations(
        case_id,
        action_selection=str(case["action_selection"]),
        np=np,
    )
    env_config = env_source.build_env_config(case)
    env = env_factory.make_cmc_env(env_config)
    writer = forensic.ForensicRolloutWriter(destination, artifact_root=REPO_ROOT)
    writer.start(
        {
            "schema_version": contract.SCHEMA_VERSION,
            "status": "STARTED_H0_PRIMARY_SPLIT_V12R2_SHIELDED_COLLECTION",
            "protocol_id": contract.PROTOCOL_ID,
            "pipeline_id": contract.PIPELINE_ID,
            "stage_id": stage_id,
            "round_index": round_index,
            "case": copy.deepcopy(case),
            "candidate_module": candidate_module_record,
            "candidate_fit_receipt": _record(
                resolve_relative(contract.FIT_RECEIPT_PATHS[fit_stage])
            ),
            "prior_label_receipt": _record(
                resolve_relative(contract.LABEL_RECEIPT_PATHS[fit_stage])
            ),
            "teacher_id": contract.TEACHER_ID,
            "teacher_evidence": _record(_teacher_evidence_path()),
            "pipeline_claim": _record(PIPELINE_CLAIM_PATH),
            "worker_claim": _record(_claim_path(stage_id)),
            "retry_authorized": False,
            "actor_updates": 0,
            "critic_updates": 0,
            "ppo_updates": 0,
            "protected_trials_opened": [],
            "reserve_trials_opened": [],
        }
    )

    rows: list[dict[str, Any]] = []
    label_observations: list[Any] = []
    label_actions: list[Any] = []
    actor_names: tuple[str, ...] = ()
    full_names: tuple[str, ...] = ()
    audit: dict[str, Any] | None = None
    info: Mapping[str, Any] = {}
    terminated = False
    truncated = False
    latch = safe_dagger.SafetyLatchState()
    previous_penetration_m = 0.0
    max_takeover_streak = 0
    takeover_streak = 0
    started = time.monotonic()
    try:
        _activity_increment("environment_reset_calls")
        observation, reset_info = env.reset(seed=int(case["runtime_seed"]))
        observation = np.ascontiguousarray(observation, dtype=np.float32)
        actor_names, full_names = _validate_runtime_layout(
            module=candidate,
            env=env,
            observation=observation,
            rollout_eval=rollout_eval,
            np=np,
        )
        rollout_eval._validate_module_observation_contract(
            teacher, actor_names, full_names
        )
        audit = _new_physical_audit(
            reset_info=reset_info,
            legacy=legacy,
            np=np,
        )
        body_weight_n = float(env.unwrapped._body_weight_n)
        if not math.isfinite(body_weight_n) or body_weight_n <= 0.0:
            raise V12R2ExecutionError("body weight is malformed")
        shadow = coherent_teacher.LegacyGaitShadow.from_runtime_phase_fsm(
            env.unwrapped._phase_fsm
        )
        student = np.ascontiguousarray(
            observation[: contract.EXPECTED_ACTOR_FEATURES], dtype=np.float32
        )
        teacher_view = coherent_teacher.build_teacher_view(
            student,
            actor_names,
            reset_info,
            body_weight_n=body_weight_n,
            shadow=shadow,
            reset_boundary=True,
        )
        current_info: Mapping[str, Any] = dict(reset_info)
        for index in range(contract.EXPECTED_STEPS):
            step = index + 1
            observation_before = observation.copy()
            student_before = student.copy()
            teacher_before = teacher_view.copy()
            coherent_teacher.assert_coherent_pair(student_before, teacher_before)
            candidate_mean, candidate_std = _query_mean_std(
                candidate, student_before, np=np, torch=torch
            )
            teacher_mean, teacher_std = _query_mean_std(
                teacher, teacher_before, np=np, torch=torch
            )
            if candidate_std.tobytes() != teacher_std.tobytes():
                raise V12R2ExecutionError("candidate/teacher logstd mismatch")
            noise = np.ascontiguousarray(
                candidate_std * innovations[index], dtype=np.float32
            )
            selected = safe_dagger.select_safe_dagger_action(
                candidate_mean,
                teacher_mean,
                noise,
                requested_alpha=case["requested_alpha"],
                latch_state=latch,
                previous_penetration_m=previous_penetration_m,
                active_v26_phase=_phase_state(current_info),
            )
            latch = selected.latch_state
            takeover_streak = takeover_streak + 1 if latch.active else 0
            max_takeover_streak = max(max_takeover_streak, takeover_streak)
            expected_mean, expected_alpha = safe_dagger.blend_policy_means(
                candidate_mean,
                teacher_mean,
                requested_alpha=case["requested_alpha"],
                latch_state=latch,
            )
            expected_action = safe_dagger.apply_single_noise(expected_mean, noise)
            if (
                expected_alpha != selected.effective_alpha
                or expected_mean.tobytes() != selected.blended_mean.tobytes()
                or expected_action.tobytes() != selected.action.tobytes()
            ):
                raise V12R2ExecutionError("blend/latch/noise semantics drifted")
            applied = np.ascontiguousarray(
                np.clip(selected.action, env.action_space.low, env.action_space.high),
                dtype=np.float32,
            )
            _activity_increment("environment_step_calls")
            observation_after, reward, terminated, truncated, info = env.step(applied)
            observation_after = np.ascontiguousarray(
                observation_after, dtype=np.float32
            )
            if not isinstance(info, Mapping):
                raise V12R2ExecutionError("collection info is malformed")
            next_student = np.ascontiguousarray(
                observation_after[: contract.EXPECTED_ACTOR_FEATURES],
                dtype=np.float32,
            )
            next_teacher_view = coherent_teacher.build_teacher_view(
                next_student,
                actor_names,
                info,
                body_weight_n=body_weight_n,
                shadow=shadow,
                reset_boundary=False,
            )
            physical = _consume_physical_step(
                audit,
                step=step,
                info=info,
                observation_before=observation_before,
                observation_after=observation_after,
                reward=reward,
                action=selected.action,
                applied_action=applied,
                extra_vectors=(
                    student_before,
                    teacher_before,
                    candidate_mean,
                    candidate_std,
                    teacher_mean,
                    teacher_std,
                    selected.blended_mean,
                    noise,
                ),
                legacy=legacy,
                v26_collector=v26_collector,
            )
            row = {
                "schema_version": contract.SCHEMA_VERSION,
                "protocol_id": contract.PROTOCOL_ID,
                "stage_id": stage_id,
                "round_index": round_index,
                "case_id": case_id,
                "v26_observation": student_before.tolist(),
                "counterfactual_teacher_observation": teacher_before.tolist(),
                "counterfactual_teacher_mean": teacher_mean.tolist(),
                "counterfactual_teacher_std": teacher_std.tolist(),
                "candidate_mean": candidate_mean.tolist(),
                "candidate_std": candidate_std.tolist(),
                "requested_alpha": float(case["requested_alpha"]),
                "effective_alpha": float(selected.effective_alpha),
                "blended_mean": selected.blended_mean.tolist(),
                "standard_normal": innovations[index].tolist(),
                "single_noise": noise.tolist(),
                "raw_action": selected.action.tolist(),
                "applied_action": applied.tolist(),
                "safety_latch_active": bool(latch.active),
                "safety_latch_entered": bool(selected.latch_entered),
                "safety_latch_released": bool(selected.latch_released),
                "forced_teacher_takeover": bool(selected.safety_intervened),
                "previous_penetration_m": previous_penetration_m,
                "reward": float(reward),
                "time_s": float(info.get("time")),
                "grf_penetration_m": physical["penetration_m"],
                "reserve_norm_nm": physical["reserve_norm_nm"],
                "residual_norm_nm": physical["residual_norm_nm"],
                "phase_fsm": legacy._jsonable(physical["phase"]),
                "checks": physical["checks"],
                "terminated": bool(terminated),
                "truncated": bool(truncated),
                "end_reason": info.get("end_reason"),
            }
            writer.write_step(step, row)
            rows.append({"step": step, **row})
            label_observations.append(student_before)
            label_actions.append(teacher_mean)
            previous_penetration_m = physical["penetration_m"]
            observation = observation_after
            student = next_student
            teacher_view = next_teacher_view
            current_info = dict(info)
            if step == 1 or step % 25 == 0:
                elapsed = time.monotonic() - started
                eta = elapsed / step * (contract.EXPECTED_STEPS - step)
                print(
                    f"[V12R2 collect r{round_index}/{case_id}] "
                    f"{step:3d}/{contract.EXPECTED_STEPS} "
                    f"elapsed={elapsed:7.1f}s eta={eta:7.1f}s",
                    flush=True,
                )
            if terminated or truncated:
                break
    except BaseException as exc:
        try:
            writer.publish_failure(
                end_reason="v12r2_shielded_collection_failed",
                error=exc,
                status=contract.COLLECTION_FAIL_STATUS,
            )
        except Exception:
            pass
        raise
    finally:
        env.close()
    if audit is None:
        raise V12R2ExecutionError("collection audit was not initialized")

    observations_array = np.ascontiguousarray(
        np.asarray(label_observations), dtype=np.float32
    )
    actions_array = np.ascontiguousarray(np.asarray(label_actions), dtype=np.float32)
    sample_count = len(rows)
    label_path = destination / "labels.npz"
    v10s_fit._write_npz_exclusive(
        label_path,
        {
            "observations": observations_array,
            "actions": actions_array,
            "reset_mask": np.asarray(
                [index == 0 for index in range(sample_count)], dtype=np.bool_
            ),
            "actor_feature_names": np.asarray(actor_names, dtype="U64"),
            "case_ids": np.asarray([case_id] * sample_count, dtype="U96"),
            "step_indices": np.arange(1, sample_count + 1, dtype=np.int64),
            "tranche_ids": np.asarray(
                [f"v12r2_collect_r{round_index}"] * sample_count,
                dtype="U96",
            ),
            "origins": np.asarray(
                ["V12R2_SHIELDED_SAME_STATE"] * sample_count, dtype="U160"
            ),
        },
    )
    partial = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": "PERSISTED_H0_PRIMARY_SPLIT_V12R2_COLLECTION_BEFORE_GATE",
        "protocol_id": contract.PROTOCOL_ID,
        "stage_id": stage_id,
        "round_index": round_index,
        "case_id": case_id,
        "steps": sample_count,
        "gate_evaluated": False,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
    }
    artifacts = _rollout_records(writer, rows=rows, partial=partial)
    summary = {
        **_physical_summary(
            audit,
            case=case,
            rows=rows,
            info=info,
            terminated=terminated,
            truncated=truncated,
            actor_names=actor_names,
            full_names=full_names,
            legacy=legacy,
            v26_collector=v26_collector,
        ),
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.COLLECTION_COMPLETE_STATUS,
        "protocol_id": contract.PROTOCOL_ID,
        "round_index": round_index,
        "requested_alpha": float(case["requested_alpha"]),
        "candidate_fit_stage": fit_stage,
        "candidate_id": contract.candidate_id(
            fit_stage, candidate_module_record["tree_sha256"]
        ),
        "candidate_module": candidate_module_record,
        "candidate_fit_receipt": _record(
            resolve_relative(contract.FIT_RECEIPT_PATHS[fit_stage])
        ),
        "candidate_fit_gate_passed": True,
        "prior_label_receipt": _record(
            resolve_relative(contract.LABEL_RECEIPT_PATHS[fit_stage])
        ),
        "prior_label_gate_passed": True,
        "teacher_id": contract.TEACHER_ID,
        "teacher_evidence_id": contract.TEACHER_EVIDENCE_ID,
        "teacher_evidence_receipt": contract.TEACHER_EVIDENCE_ARTIFACT,
        "source_h0": _tree_record(SOURCE_H0_MODULE),
        "sample_count": sample_count,
        "teacher_query_count": sample_count,
        "persisted_label_count": sample_count,
        "same_state_teacher_label_count": sample_count,
        "candidate_mean_query_count": sample_count,
        "candidate_selected_before_teacher_count": sample_count,
        "served_action_teacher_dependency_count": sample_count,
        "mean_blend_count": sample_count,
        "blend_before_noise_count": sample_count,
        "noise_before_blend_count": 0,
        "multiple_noise_application_count": 0,
        "noise_application_mismatch_count": 0,
        "alpha_mismatch_count": 0,
        "mean_blend_mismatch_count": 0,
        "safety_latch_rule_violation_count": 0,
        "physical_gate_bypass_count": 0,
        "safety_latch_activation_m": safe_dagger.SAFETY_LATCH_ACTIVATION_M,
        "safety_latch_release_m": safe_dagger.SAFETY_LATCH_RELEASE_M,
        "safety_latch_release_phase": safe_dagger.SAFETY_LATCH_RELEASE_PHASE,
        "safety_signal_lag_steps": 1,
        "safety_intervention_diagnostic_only": True,
        "safety_latch_activation_count": latch.activation_count,
        "safety_latch_release_count": latch.release_count,
        "safety_intervention_count": latch.intervention_action_count,
        "forced_teacher_takeover_count": latch.intervention_action_count,
        "forced_teacher_takeover_fraction": (
            latch.intervention_action_count / sample_count if sample_count else 0.0
        ),
        "max_consecutive_takeover_steps": max_takeover_streak,
        "latch_active_at_episode_end": bool(latch.active),
        "collection_is_data_only": True,
        "autonomy_claimed": False,
        "target_contract_id": contract.TARGET_CONTRACT_ID,
        "label_corpus": _record(label_path),
        "worker_claim": _record(_claim_path(stage_id)),
        **artifacts,
    }
    _publish_rollout_before_gate(
        writer,
        rows=rows,
        partial=partial,
        summary=summary,
    )
    data_gate = dict(contract.collection_data_gate(summary, round_index=round_index))
    latch_gate = dict(
        contract.latch_dependence_gate(
            summary, collection_data_passed=data_gate.get("passed") is True
        )
    )
    gate_payload = {
        "data_gate": data_gate,
        "latch_dependence_gate": latch_gate,
    }
    writer.publish_gate(gate_payload)
    if data_gate.get("passed") is not True:
        writer.publish_failure(
            end_reason="v12r2_collection_data_gate_failed",
            error="V12R2 collection integrity/physical gate failed",
            status=contract.COLLECTION_FAIL_STATUS,
            details={"gate": _record(writer.gate_path)},
        )
        raise V12R2ExecutionError(f"collection data gate failed: {stage_id}")
    receipt = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.COLLECTION_PASS_STATUS,
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "round_index": round_index,
        "case_id": case_id,
        "sample_count": sample_count,
        "label_corpus": _record(label_path),
        "pipeline_claim": _record(PIPELINE_CLAIM_PATH),
        "worker_claim": _record(_claim_path(stage_id)),
    }
    forensic.write_json_exclusive(destination / "receipt.json", receipt)
    return _verify_stage_receipt(stage_id)


def _candidate_freeze_receipt() -> dict[str, Any]:
    return _mapping(CANDIDATE_FREEZE_PATH)


def _run_candidate_freeze() -> dict[str, Any]:
    """Freeze P3 only after every fit/probe/label/collection is closed."""

    stage_id = "freeze_p3"
    summary_path = CANDIDATE_FREEZE_PATH.with_name("candidate_freeze_summary.json")
    gate_path = CANDIDATE_FREEZE_PATH.with_name("candidate_freeze_gate.json")
    for path in (CANDIDATE_FREEZE_PATH, summary_path, gate_path):
        if os.path.lexists(path):
            raise V12R2ExecutionError(f"candidate freeze output exists: {path}")
    fit_receipts: list[dict[str, Any]] = []
    probe_receipts: list[dict[str, Any]] = []
    label_receipts: list[dict[str, Any]] = []
    for fit_stage in contract.FIT_STAGES:
        fit_receipt = fit_engine.verify_fit_stage(fit_stage)
        _verify_stage_receipt(f"probe_{fit_stage}")
        label_receipt = label_engine.verify_observer_label_stage(fit_stage)
        probe_gate = _mapping(
            resolve_relative(contract.PROBE_ROOT / fit_stage / "gate.json")
        )
        fit_receipts.append(
            {
                "fit_stage": fit_stage,
                "receipt": _record(
                    resolve_relative(contract.FIT_RECEIPT_PATHS[fit_stage])
                ),
                "passed": fit_receipt.get("passed") is True,
            }
        )
        probe_receipts.append(
            {
                "fit_stage": fit_stage,
                "receipt": _record(
                    resolve_relative(contract.PROBE_RECEIPT_PATHS[fit_stage])
                ),
                "integrity_passed": probe_gate.get("integrity_passed") is True,
                "autonomy_passed": probe_gate.get("passed") is True,
                "recoverable_for_data_collection": bool(
                    probe_gate.get("recoverable_for_data_collection")
                ),
            }
        )
        label_receipts.append(
            {
                "fit_stage": fit_stage,
                "receipt": _record(
                    resolve_relative(contract.LABEL_RECEIPT_PATHS[fit_stage])
                ),
                "passed": label_receipt.get("passed") is True,
            }
        )
    collection_receipts: list[dict[str, Any]] = []
    for round_index in (1, 2, 3):
        for case_id in contract.COLLECTION_CASE_IDS:
            stage = f"collect_r{round_index}__{case_id}"
            _verify_stage_receipt(stage)
            root = resolve_relative(
                contract.canonical_collection_case(case_id, round_index)["destination"]
            )
            gates = _mapping(root / "gate.json")
            collection_receipts.append(
                {
                    "round_index": round_index,
                    "case_id": case_id,
                    "receipt": _record(root / "receipt.json"),
                    "data_passed": gates["data_gate"].get("passed") is True,
                    "latch_independence_passed": gates["latch_dependence_gate"].get(
                        "passed"
                    )
                    is True,
                }
            )
    p3_root = resolve_relative(contract.FIT_ROOTS["p3"])
    p3_summary = _mapping(p3_root / "summary.json")
    p3_module = _tree_record(resolve_relative(contract.MODULE_PATHS["p3"]))
    candidate_id = contract.candidate_id("p3", p3_module["tree_sha256"])
    p3_probe_gate = _mapping(resolve_relative(contract.PROBE_ROOT / "p3/gate.json"))
    p3_label_gate = _mapping(resolve_relative(contract.LABEL_ROOT / "p3/gate.json"))
    summary = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": "COMPLETE_H0_PRIMARY_SPLIT_V12R2_CANDIDATE_FREEZE_UNGATED",
        "protocol_id": contract.PROTOCOL_ID,
        "fit_contract_id": contract.FIT_CONTRACT_ID,
        "candidate_fit_stage": "p3",
        "candidate_id": candidate_id,
        "candidate_module": p3_module,
        "fit_receipts": fit_receipts,
        "probe_receipts": probe_receipts,
        "label_receipts": label_receipts,
        "collection_receipts": collection_receipts,
        "pipeline_claim": _record(PIPELINE_CLAIM_PATH),
        "worker_claim": _record(_claim_path(stage_id)),
        "p3_fit_passed": fit_receipts[-1]["passed"],
        "p3_probe_integrity_passed": p3_probe_gate.get("integrity_passed") is True,
        "p3_probe_autonomy_passed": p3_probe_gate.get("passed") is True,
        "p3_label_passed": p3_label_gate.get("passed") is True,
        "source_h0": _tree_record(SOURCE_H0_MODULE),
        "design_audit_receipt": _record(DESIGN_AUDIT_PATH),
        "design_audit_passed": _design_audit_gate(_mapping(DESIGN_AUDIT_PATH)).get(
            "passed"
        )
        is True,
        "execution_lock": _record(LOCK_PATH),
        "fit_actor_update_count": len(contract.FIT_STAGES),
        "every_fit_restarted_from_h0": all(
            _mapping(resolve_relative(contract.FIT_ROOTS[item]) / "summary.json").get(
                "continued_from_previous_candidate"
            )
            is False
            for item in contract.FIT_STAGES
        ),
        "source_checkpoint_scope": p3_summary.get("source_checkpoint_scope"),
        "critic_present": p3_summary.get("critic_present"),
        "critic_parameter_count": p3_summary.get("critic_parameter_count"),
        "logstd_byte_exact": p3_summary.get("logstd_byte_exact") is True,
        "normalization_folded_into_first_layer": p3_summary.get(
            "normalization_folded_into_first_layer"
        )
        is True,
        "runtime_normalization_wrapper_present": p3_summary.get(
            "runtime_normalization_wrapper_present"
        ),
        "disabled_clock_columns_0_1_bit_zero": p3_summary.get(
            "disabled_clock_columns_bit_zero_after_save_reload"
        )
        is True,
        "candidate_frozen": True,
        "runtime_promoted": False,
        "retry_authorized": False,
        "sweep_authorized": False,
        "rescue_authorized": False,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
    }
    forensic.write_json_exclusive(summary_path, summary)
    gate = dict(contract.candidate_freeze_gate(summary))
    forensic.write_json_exclusive(gate_path, gate)
    if gate.get("passed") is not True:
        raise V12R2ExecutionError("candidate freeze gate failed")
    receipt = {
        **summary,
        "status": contract.CANDIDATE_FREEZE_PASS_STATUS,
        "passed": True,
        "stage_id": stage_id,
        "summary": _record(summary_path),
        "gate": _record(gate_path),
    }
    forensic.write_json_exclusive(CANDIDATE_FREEZE_PATH, receipt)
    return _verify_stage_receipt(stage_id)


def _run_final(case_id: str) -> dict[str, Any]:
    """Run one pure P3 development case with no teacher/blend/latch."""

    (
        rollout_eval,
        np,
        torch,
        RLModule,
        env_factory,
        legacy,
        v26_collector,
    ) = _load_rollout_stack()
    stage_id = f"final__{case_id}"
    case = contract.canonical_final_case(case_id)
    destination = resolve_relative(case["destination"])
    if os.path.lexists(destination):
        raise V12R2ExecutionError(f"final destination exists: {case_id}")
    freeze = _candidate_freeze_receipt()
    candidate_module_path = resolve_relative(contract.MODULE_PATHS["p3"])
    candidate_module = _tree_record(candidate_module_path)
    candidate_id = contract.candidate_id("p3", candidate_module["tree_sha256"])
    if freeze.get("candidate_id") != candidate_id:
        raise V12R2ExecutionError("candidate freeze identity drifted")
    candidate = RLModule.from_checkpoint(candidate_module_path)
    candidate.eval()
    innovations = _frozen_innovations(
        case_id,
        action_selection=str(case["action_selection"]),
        np=np,
    )
    env_config = env_source.build_env_config(case)
    env = env_factory.make_cmc_env(env_config)
    writer = forensic.ForensicRolloutWriter(destination, artifact_root=REPO_ROOT)
    writer.start(
        {
            "schema_version": contract.SCHEMA_VERSION,
            "status": "STARTED_H0_PRIMARY_SPLIT_V12R2_FINAL_ROLLOUT",
            "protocol_id": contract.PROTOCOL_ID,
            "pipeline_id": contract.PIPELINE_ID,
            "stage_id": stage_id,
            "case": copy.deepcopy(case),
            "candidate_id": candidate_id,
            "candidate_module": candidate_module,
            "candidate_freeze": _record(CANDIDATE_FREEZE_PATH),
            "teacher_enabled": False,
            "blending_enabled": False,
            "safety_latch_enabled": False,
            "pipeline_claim": _record(PIPELINE_CLAIM_PATH),
            "worker_claim": _record(_claim_path(stage_id)),
            "retry_authorized": False,
            "actor_updates": 0,
            "critic_updates": 0,
            "ppo_updates": 0,
            "protected_trials_opened": [],
            "reserve_trials_opened": [],
        }
    )
    rows: list[dict[str, Any]] = []
    actor_names: tuple[str, ...] = ()
    full_names: tuple[str, ...] = ()
    audit: dict[str, Any] | None = None
    info: Mapping[str, Any] = {}
    terminated = False
    truncated = False
    started = time.monotonic()
    try:
        _activity_increment("environment_reset_calls")
        observation, reset_info = env.reset(seed=int(case["runtime_seed"]))
        observation = np.ascontiguousarray(observation, dtype=np.float32)
        actor_names, full_names = _validate_runtime_layout(
            module=candidate,
            env=env,
            observation=observation,
            rollout_eval=rollout_eval,
            np=np,
        )
        audit = _new_physical_audit(
            reset_info=reset_info,
            legacy=legacy,
            np=np,
        )
        for index in range(contract.EXPECTED_STEPS):
            step = index + 1
            observation_before = observation.copy()
            actor = np.ascontiguousarray(
                observation_before[: contract.EXPECTED_ACTOR_FEATURES],
                dtype=np.float32,
            )
            mean, std = _query_mean_std(candidate, actor, np=np, torch=torch)
            noise = np.ascontiguousarray(std * innovations[index], dtype=np.float32)
            raw_action = safe_dagger.apply_single_noise(mean, noise)
            applied = np.ascontiguousarray(
                np.clip(raw_action, env.action_space.low, env.action_space.high),
                dtype=np.float32,
            )
            _activity_increment("environment_step_calls")
            observation_after, reward, terminated, truncated, info = env.step(applied)
            observation_after = np.ascontiguousarray(
                observation_after, dtype=np.float32
            )
            if not isinstance(info, Mapping):
                raise V12R2ExecutionError("final rollout info is malformed")
            physical = _consume_physical_step(
                audit,
                step=step,
                info=info,
                observation_before=observation_before,
                observation_after=observation_after,
                reward=reward,
                action=raw_action,
                applied_action=applied,
                extra_vectors=(actor, mean, std, noise),
                legacy=legacy,
                v26_collector=v26_collector,
            )
            row = {
                "schema_version": contract.SCHEMA_VERSION,
                "protocol_id": contract.PROTOCOL_ID,
                "stage_id": stage_id,
                "case_id": case_id,
                "v26_observation": actor.tolist(),
                "candidate_mean": mean.tolist(),
                "candidate_std": std.tolist(),
                "standard_normal": innovations[index].tolist(),
                "single_noise": noise.tolist(),
                "raw_action": raw_action.tolist(),
                "applied_action": applied.tolist(),
                "teacher_enabled": False,
                "teacher_query_count": 0,
                "served_action_teacher_dependency_count": 0,
                "blending_enabled": False,
                "safety_latch_enabled": False,
                "reward": float(reward),
                "time_s": float(info.get("time")),
                "grf_penetration_m": physical["penetration_m"],
                "reserve_norm_nm": physical["reserve_norm_nm"],
                "residual_norm_nm": physical["residual_norm_nm"],
                "phase_fsm": legacy._jsonable(physical["phase"]),
                "checks": physical["checks"],
                "terminated": bool(terminated),
                "truncated": bool(truncated),
                "end_reason": info.get("end_reason"),
            }
            writer.write_step(step, row)
            rows.append({"step": step, **row})
            observation = observation_after
            if step == 1 or step % 25 == 0:
                elapsed = time.monotonic() - started
                eta = elapsed / step * (contract.EXPECTED_STEPS - step)
                print(
                    f"[V12R2 final/{case_id}] {step:3d}/"
                    f"{contract.EXPECTED_STEPS} elapsed={elapsed:7.1f}s "
                    f"eta={eta:7.1f}s",
                    flush=True,
                )
            if terminated or truncated:
                break
    except BaseException as exc:
        try:
            writer.publish_failure(
                end_reason="v12r2_final_rollout_failed",
                error=exc,
                status=contract.FINAL_ROLLOUT_FAIL_STATUS,
            )
        except Exception:
            pass
        raise
    finally:
        env.close()
    if audit is None:
        raise V12R2ExecutionError("final audit was not initialized")
    partial = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": "PERSISTED_H0_PRIMARY_SPLIT_V12R2_FINAL_BEFORE_GATE",
        "protocol_id": contract.PROTOCOL_ID,
        "stage_id": stage_id,
        "case_id": case_id,
        "steps": len(rows),
        "gate_evaluated": False,
        "teacher_query_count": 0,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
    }
    artifacts = _rollout_records(writer, rows=rows, partial=partial)
    summary = {
        **_physical_summary(
            audit,
            case=case,
            rows=rows,
            info=info,
            terminated=terminated,
            truncated=truncated,
            actor_names=actor_names,
            full_names=full_names,
            legacy=legacy,
            v26_collector=v26_collector,
        ),
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.FINAL_ROLLOUT_COMPLETE_STATUS,
        "protocol_id": contract.PROTOCOL_ID,
        "candidate_fit_stage": "p3",
        "candidate_id": candidate_id,
        "candidate_module": candidate_module,
        "candidate_freeze": _record(CANDIDATE_FREEZE_PATH),
        "candidate_freeze_passed": True,
        "candidate_mean_query_count": len(rows),
        "teacher_enabled": False,
        "teacher_loaded_during_rollout": False,
        "teacher_query_count": 0,
        "served_action_teacher_dependency_count": 0,
        "blending_enabled": False,
        "mean_blend_count": 0,
        "safety_latch_enabled": False,
        "safety_intervention_count": 0,
        "multiple_noise_application_count": 0,
        "noise_application_mismatch_count": 0,
        "physical_gate_bypass_count": 0,
        "target_contract_id": contract.TARGET_CONTRACT_ID,
        "worker_claim": _record(_claim_path(stage_id)),
        **artifacts,
    }
    _publish_rollout_before_gate(
        writer,
        rows=rows,
        partial=partial,
        summary=summary,
    )
    gate = dict(contract.final_rollout_gate(summary))
    writer.publish_gate(gate)
    if gate.get("passed") is not True:
        writer.publish_failure(
            end_reason="v12r2_final_gate_failed",
            error="V12R2 final rollout physical/dependency gate failed",
            status=contract.FINAL_ROLLOUT_FAIL_STATUS,
            details={"gate": _record(writer.gate_path)},
        )
        raise V12R2ExecutionError(f"final rollout gate failed: {case_id}")
    receipt = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.FINAL_ROLLOUT_PASS_STATUS,
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "stage_id": stage_id,
        "case_id": case_id,
        "candidate_id": candidate_id,
        "candidate_module": candidate_module,
        "summary": _record(writer.summary_path),
        "gate": _record(writer.gate_path),
        "trace": _record(writer.trace_path),
        "pipeline_claim": _record(PIPELINE_CLAIM_PATH),
        "worker_claim": _record(_claim_path(stage_id)),
        "retry_authorized": False,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
    }
    forensic.write_json_exclusive(destination / "receipt.json", receipt)
    return _verify_stage_receipt(stage_id)


def _run_finalize_development() -> dict[str, Any]:
    """Aggregate six exact final passes without promoting the candidate."""

    stage_id = "finalize_development"
    final_root = resolve_relative(contract.FINAL_ROOT)
    receipt_path = resolve_relative(contract.FINAL_DEVELOPMENT_RECEIPT_PATH)
    summary_path = final_root / "summary.json"
    gate_path = final_root / "gate.json"
    for path in (receipt_path, summary_path, gate_path):
        if os.path.lexists(path):
            raise V12R2ExecutionError(f"final aggregate output exists: {path}")
    freeze = _candidate_freeze_receipt()
    module = _tree_record(resolve_relative(contract.MODULE_PATHS["p3"]))
    candidate_id = contract.candidate_id("p3", module["tree_sha256"])
    bindings: list[dict[str, Any]] = []
    for case_id in contract.FINAL_CASE_IDS:
        _verify_stage_receipt(f"final__{case_id}")
        root = resolve_relative(contract.canonical_final_case(case_id)["destination"])
        bindings.append(
            {
                "case_id": case_id,
                "passed": True,
                "receipt": _record(root / "receipt.json"),
                "gate": _record(root / "gate.json"),
                "summary": _record(root / "summary.json"),
            }
        )
    summary = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.FINAL_DEVELOPMENT_COMPLETE_STATUS,
        "protocol_id": contract.PROTOCOL_ID,
        "candidate_id": candidate_id,
        "candidate_module": module,
        "candidate_freeze": _record(CANDIDATE_FREEZE_PATH),
        "candidate_freeze_passed": freeze.get("passed") is True,
        "pipeline_claim": _record(PIPELINE_CLAIM_PATH),
        "worker_claim": _record(_claim_path(stage_id)),
        "rollout_bindings": bindings,
        "rollout_count": len(bindings),
        "passing_rollout_count": len(bindings),
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
        "development_only": True,
        "runtime_promoted": False,
        "qualification_required": True,
        "retry_authorized": False,
        "sweep_authorized": False,
        "rescue_authorized": False,
    }
    forensic.write_json_exclusive(summary_path, summary)
    gate = dict(contract.final_development_gate(summary))
    forensic.write_json_exclusive(gate_path, gate)
    if gate.get("passed") is not True:
        raise V12R2ExecutionError("final development aggregate gate failed")
    receipt = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.FINAL_DEVELOPMENT_PASS_STATUS,
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "stage_id": stage_id,
        "candidate_id": candidate_id,
        "candidate_module": module,
        "candidate_freeze": _record(CANDIDATE_FREEZE_PATH),
        "summary": _record(summary_path),
        "gate": _record(gate_path),
        "rollout_bindings": bindings,
        "pipeline_claim": _record(PIPELINE_CLAIM_PATH),
        "worker_claim": _record(_claim_path(stage_id)),
        "development_only": True,
        "runtime_promoted": False,
        "qualification_required": True,
        "retry_authorized": False,
        "sweep_authorized": False,
        "rescue_authorized": False,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
    }
    forensic.write_json_exclusive(receipt_path, receipt)
    return _verify_stage_receipt(stage_id)


def _run_fit(stage: str) -> dict[str, Any]:
    _activity_increment("actor_fit_stage_calls_attempted")
    _activity_increment("actor_updates_attempted")
    result = fit_engine.run_fit_stage(
        stage=stage,
        output_dir=contract.FIT_ROOTS[stage],
        pipeline_claim_path=PIPELINE_CLAIM_PATH,
        worker_claim_path=_claim_path(f"fit_{stage}"),
        design_audit_path=DESIGN_AUDIT_PATH,
        execution_lock_path=LOCK_PATH,
    )
    if result.get("passed") is not True:
        raise V12R2ExecutionError(f"fit gate failed: {stage}")
    _activity_increment("actor_fit_executions_confirmed")
    _activity_increment("actor_updates_confirmed")
    return fit_engine.verify_fit_stage(stage)


def _run_stage(stage_id: str) -> dict[str, Any]:
    descriptor = contract.stage_descriptor(stage_id)
    kind = descriptor["kind"]
    if kind == "fit":
        return _run_fit(descriptor["fit_stage"])
    if kind == "probe":
        return _run_probe(descriptor["fit_stage"])
    if kind == "label":
        return _run_label(descriptor["fit_stage"])
    if kind == "collection":
        return _run_collection(
            int(descriptor["round_index"]),
            str(descriptor["case"]["case_id"]),
        )
    if kind == "freeze":
        return _run_candidate_freeze()
    if kind == "final":
        return _run_final(str(descriptor["case"]["case_id"]))
    if kind == "finalize":
        return _run_finalize_development()
    raise V12R2ExecutionError(f"unknown stage kind: {kind!r}")


def _pipeline_activity_gate(
    stage_activity: Mapping[str, Mapping[str, Any]],
) -> dict[str, Any]:
    expected_rows: list[dict[str, Any]] = []
    for stage_id in contract.STAGE_IDS:
        descriptor = contract.stage_descriptor(stage_id)
        kind = descriptor["kind"]
        row: dict[str, Any] = {
            "stage_id": stage_id,
            "stage_kind": kind,
            "actor_fit_stage_calls_attempted": 0,
            "actor_updates_attempted": 0,
            "actor_fit_executions_confirmed": 0,
            "actor_updates_confirmed": 0,
            "environment_reset_calls": 0,
            "environment_step_calls": 0,
            "offline_teacher_label_stage_calls": 0,
            "offline_teacher_label_calls_confirmed": 0,
        }
        if kind == "fit":
            row.update(
                {
                    "actor_fit_stage_calls_attempted": 1,
                    "actor_updates_attempted": 1,
                    "actor_fit_executions_confirmed": 1,
                    "actor_updates_confirmed": 1,
                }
            )
        elif kind == "probe":
            receipt = _mapping(_stage_receipt_path(stage_id))
            row["environment_reset_calls"] = 1
            row["environment_step_calls"] = int(receipt["probe_step_count"])
        elif kind == "label":
            receipt = _mapping(_stage_receipt_path(stage_id))
            row["offline_teacher_label_stage_calls"] = 1
            row["offline_teacher_label_calls_confirmed"] = int(
                receipt["labelled_row_count"]
            )
        elif kind == "collection":
            receipt = _mapping(_stage_receipt_path(stage_id))
            row["environment_reset_calls"] = 1
            row["environment_step_calls"] = int(receipt["sample_count"])
        elif kind == "final":
            root = resolve_relative(descriptor["case"]["destination"])
            summary = _mapping(root / "summary.json")
            row["environment_reset_calls"] = 1
            row["environment_step_calls"] = int(summary["steps"])
        expected_rows.append(row)
    observed_rows = [dict(stage_activity[stage]) for stage in contract.STAGE_IDS]
    totals = {
        name: sum(int(row[name]) for row in expected_rows)
        for name in (
            "actor_fit_stage_calls_attempted",
            "actor_updates_attempted",
            "actor_fit_executions_confirmed",
            "actor_updates_confirmed",
            "environment_reset_calls",
            "environment_step_calls",
            "offline_teacher_label_stage_calls",
            "offline_teacher_label_calls_confirmed",
        )
    }
    checks = {
        "all_26_stage_rows_present": set(stage_activity) == set(contract.STAGE_IDS),
        "per_stage_activity_exact": observed_rows == expected_rows,
        "four_fit_calls_and_updates": totals["actor_fit_stage_calls_attempted"] == 4
        and totals["actor_updates_attempted"] == 4
        and totals["actor_fit_executions_confirmed"] == 4
        and totals["actor_updates_confirmed"] == 4,
        "sixteen_rollout_resets": totals["environment_reset_calls"] == 16,
        "four_observer_label_stages": totals["offline_teacher_label_stage_calls"] == 4,
        "positive_exact_rollout_steps": totals["environment_step_calls"] > 0,
        "positive_exact_offline_labels": totals["offline_teacher_label_calls_confirmed"]
        > 0,
    }
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": (
            "PASS_H0_PRIMARY_SPLIT_V12R2_PIPELINE_ACTIVITY"
            if all(checks.values())
            else "FAIL_H0_PRIMARY_SPLIT_V12R2_PIPELINE_ACTIVITY"
        ),
        "passed": all(checks.values()),
        "protocol_id": contract.PROTOCOL_ID,
        "checks": checks,
        "expected_totals": totals,
    }


def _ledger_payload(
    *,
    status: str,
    passed: bool,
    completed_stages: Sequence[str],
    attempted_stage: str | None,
    stage_activity: Mapping[str, Mapping[str, Any]],
    error: BaseException | None,
) -> dict[str, Any]:
    completed = [
        {
            "stage_id": stage,
            "receipt": _record(_stage_receipt_path(stage)),
        }
        for stage in completed_stages
    ]
    completed_fit_count = sum(
        int(contract.stage_descriptor(stage)["kind"] == "fit")
        for stage in completed_stages
    )
    completed_rollout_count = sum(
        int(
            contract.stage_descriptor(stage)["kind"] in {"probe", "collection", "final"}
        )
        for stage in completed_stages
    )
    completed_label_count = sum(
        int(contract.stage_descriptor(stage)["kind"] == "label")
        for stage in completed_stages
    )
    activity_rows = [
        copy.deepcopy(dict(stage_activity[stage]))
        for stage in contract.STAGE_IDS
        if stage in stage_activity
    ]
    activity_totals = {
        name: sum(int(row.get(name, 0)) for row in activity_rows)
        for name in (
            "actor_fit_stage_calls_attempted",
            "actor_updates_attempted",
            "actor_fit_executions_confirmed",
            "actor_updates_confirmed",
            "environment_reset_calls",
            "environment_step_calls",
            "offline_teacher_label_stage_calls",
            "offline_teacher_label_calls_confirmed",
        )
    }
    activity_gate = (
        _pipeline_activity_gate(stage_activity)
        if passed
        else {
            "schema_version": contract.SCHEMA_VERSION,
            "status": "NOT_APPLICABLE_TERMINAL_FAILURE",
            "passed": False,
            "protocol_id": contract.PROTOCOL_ID,
            "checks": {},
        }
    )
    if passed and activity_gate.get("passed") is not True:
        raise V12R2ExecutionError("pipeline PASS activity accounting drifted")
    payload: dict[str, Any] = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": status,
        "passed": passed,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "execution_lock": _record(LOCK_PATH),
        "pipeline_claim": _record(PIPELINE_CLAIM_PATH),
        "stage_order": list(contract.STAGE_IDS),
        "completed_stages": list(completed_stages),
        "completed_receipts": completed,
        "attempted_stage": attempted_stage,
        "stage_activity": activity_rows,
        "activity_gate": activity_gate,
        **activity_totals,
        "fit_stage_receipts_confirmed": completed_fit_count,
        "actor_update_receipts_confirmed": completed_fit_count,
        "environment_rollout_stages_confirmed": completed_rollout_count,
        "offline_teacher_label_stage_receipts_confirmed": completed_label_count,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
        "runtime_promoted": False,
        "retry_authorized": False,
        "sweep_authorized": False,
        "rescue_authorized": False,
    }
    if error is not None:
        payload["error"] = {
            "type": type(error).__name__,
            "message": str(error),
        }
    else:
        payload["error"] = None
    return payload


def _claim_run_root() -> None:
    """Atomically claim the exact V12R2 run directory before its first file."""

    if RUN_ROOT.resolve().parent != resolve_relative(contract.RUN_ROOT.parent):
        raise V12R2ExecutionError("run-root parent drifted")
    for ancestor in (RUN_ROOT.parent, *RUN_ROOT.parent.parents):
        if ancestor == REPO_ROOT.parent:
            break
        if os.path.lexists(ancestor) and ancestor.is_symlink():
            raise V12R2ExecutionError(f"run-root ancestor is a symlink: {ancestor}")
    RUN_ROOT.parent.mkdir(parents=True, exist_ok=True)
    try:
        RUN_ROOT.mkdir(exist_ok=False)
    except FileExistsError as exc:
        raise V12R2ExecutionError("V12R2 run root already claimed") from exc


def _publish_terminal_stage_failure(
    stage_id: str | None, error: BaseException
) -> dict[str, Any]:
    """Never mask the original error while closing any started rollout."""

    try:
        if stage_id is None:
            return {"applicable": False, "reason": "no_stage_attempted"}
        descriptor = contract.stage_descriptor(stage_id)
        kind = descriptor["kind"]
        if kind == "probe":
            root = resolve_relative(contract.PROBE_ROOT / descriptor["fit_stage"])
            status = contract.PROBE_INTEGRITY_FAIL_STATUS
        elif kind == "collection":
            root = resolve_relative(descriptor["case"]["destination"])
            status = contract.COLLECTION_FAIL_STATUS
        elif kind == "final":
            root = resolve_relative(descriptor["case"]["destination"])
            status = contract.FINAL_ROLLOUT_FAIL_STATUS
        else:
            return {"applicable": False, "reason": f"non_rollout_stage:{kind}"}
        writer = forensic.ForensicRolloutWriter(root, artifact_root=REPO_ROOT)
        if not writer.run_start_path.is_file():
            return {"applicable": True, "published": False, "reason": "not_started"}
        if os.path.lexists(writer.failure_path):
            existing = _mapping(writer.failure_path)
            if (
                existing.get("passed") is not False
                or not isinstance(existing.get("status"), str)
                or type(existing.get("last_completed_step")) is not int
                or existing["last_completed_step"] < 0
                or not isinstance(existing.get("error"), Mapping)
                or not isinstance(existing.get("artifacts"), Mapping)
            ):
                raise V12R2ExecutionError(
                    "preexisting rollout failure receipt is malformed"
                )
            return {
                "applicable": True,
                "published": True,
                "already_present": True,
                "verified": True,
                "artifact": _record(writer.failure_path),
            }
        artifact = writer.publish_failure(
            end_reason="v12r2_stage_failed_terminally",
            error=error,
            status=status,
            details={"stage_id": stage_id, "stage_kind": kind},
        )
        return {
            "applicable": True,
            "published": True,
            "already_present": False,
            "verified": True,
            "artifact": artifact,
        }
    except BaseException as publication_error:
        return {
            "applicable": stage_id is not None,
            "published": False,
            "reason": "failure_receipt_publication_failed",
            "error": {
                "type": type(publication_error).__name__,
                "message": str(publication_error),
            },
        }


def _pipeline_claim_snapshot() -> dict[str, Any]:
    relative = PIPELINE_CLAIM_PATH.relative_to(REPO_ROOT).as_posix()
    if not os.path.lexists(PIPELINE_CLAIM_PATH):
        return {"path": relative, "lexists": False}
    status = os.lstat(PIPELINE_CLAIM_PATH)
    snapshot: dict[str, Any] = {
        "path": relative,
        "lexists": True,
        "size_bytes": int(status.st_size),
        "regular_file": PIPELINE_CLAIM_PATH.is_file(),
        "symlink": PIPELINE_CLAIM_PATH.is_symlink(),
    }
    if snapshot["regular_file"] and not snapshot["symlink"]:
        snapshot["sha256"] = forensic.sha256_file(PIPELINE_CLAIM_PATH)
    return snapshot


def _preclaim_failure_ledger(error: BaseException) -> dict[str, Any]:
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": "FAIL_H0_PRIMARY_SPLIT_V12R2_PIPELINE_CLAIM_TERMINAL",
        "passed": False,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "execution_lock": _record(LOCK_PATH),
        "pipeline_claim": None,
        "pipeline_claim_snapshot": _pipeline_claim_snapshot(),
        "run_root_claimed": RUN_ROOT.is_dir() and not RUN_ROOT.is_symlink(),
        "completed_stages": [],
        "attempted_stage": None,
        "actor_fit_stage_calls_attempted": 0,
        "actor_updates_attempted": 0,
        "environment_reset_calls": 0,
        "environment_step_calls": 0,
        "offline_teacher_label_stage_calls": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
        "runtime_promoted": False,
        "retry_authorized": False,
        "sweep_authorized": False,
        "rescue_authorized": False,
        "error": {"type": type(error).__name__, "message": str(error)},
    }


def execute_pipeline_once() -> dict[str, Any]:
    """Claim and execute the exact 26-stage V12R2 order once."""

    verify_execution_lock(require_run_root_absent=True)
    if os.path.lexists(PIPELINE_CLAIM_PATH) or os.path.lexists(PIPELINE_LEDGER_PATH):
        raise V12R2ExecutionError("pipeline is already claimed or terminal")
    token = secrets.token_urlsafe(48)
    token_sha = _token_sha256(token)
    del token
    claim_payload = _pipeline_claim_payload(token_sha)
    forensic.canonical_json_bytes(claim_payload)
    _claim_run_root()
    try:
        forensic.write_json_exclusive(PIPELINE_CLAIM_PATH, claim_payload)
        _verify_pipeline_claim()
    except BaseException as exc:
        if not os.path.lexists(PIPELINE_LEDGER_PATH):
            forensic.write_json_exclusive(
                PIPELINE_LEDGER_PATH,
                _preclaim_failure_ledger(exc),
            )
        raise
    global _ACTIVE_STAGE_ACTIVITY
    completed: list[str] = []
    stage_activity: dict[str, dict[str, Any]] = {}
    attempted: str | None = None
    try:
        for stage_id in contract.STAGE_IDS:
            attempted = stage_id
            activity = {
                "stage_id": stage_id,
                "stage_kind": contract.stage_descriptor(stage_id)["kind"],
                "actor_fit_stage_calls_attempted": 0,
                "actor_updates_attempted": 0,
                "actor_fit_executions_confirmed": 0,
                "actor_updates_confirmed": 0,
                "environment_reset_calls": 0,
                "environment_step_calls": 0,
                "offline_teacher_label_stage_calls": 0,
                "offline_teacher_label_calls_confirmed": 0,
            }
            stage_activity[stage_id] = activity
            _ACTIVE_STAGE_ACTIVITY = activity
            _write_worker_claim(stage_id, token_sha)
            _verify_worker_claim(stage_id)
            print(
                f"[V12R2 pipeline] start {len(completed) + 1:02d}/"
                f"{len(contract.STAGE_IDS):02d} {stage_id}",
                flush=True,
            )
            try:
                _run_stage(stage_id)
            finally:
                _ACTIVE_STAGE_ACTIVITY = None
            _verify_stage_receipt(stage_id)
            completed.append(stage_id)
            attempted = None
            print(
                f"[V12R2 pipeline] pass  {len(completed):02d}/"
                f"{len(contract.STAGE_IDS):02d} {stage_id}",
                flush=True,
            )
        if tuple(completed) != tuple(contract.STAGE_IDS):
            raise V12R2ExecutionError("pipeline completed-stage order drifted")
        ledger = _ledger_payload(
            status="PASS_H0_PRIMARY_SPLIT_V12R2_PIPELINE",
            passed=True,
            completed_stages=completed,
            attempted_stage=None,
            stage_activity=stage_activity,
            error=None,
        )
        forensic.write_json_exclusive(PIPELINE_LEDGER_PATH, ledger)
        return _mapping(PIPELINE_LEDGER_PATH)
    except BaseException as exc:
        _ACTIVE_STAGE_ACTIVITY = None
        terminal_failure = _publish_terminal_stage_failure(attempted, exc)
        if attempted is not None and attempted in stage_activity:
            stage_activity[attempted]["terminal_failure"] = terminal_failure
        failure = _ledger_payload(
            status="FAIL_H0_PRIMARY_SPLIT_V12R2_PIPELINE_TERMINAL",
            passed=False,
            completed_stages=completed,
            attempted_stage=attempted,
            stage_activity=stage_activity,
            error=exc,
        )
        if not os.path.lexists(PIPELINE_LEDGER_PATH):
            forensic.write_json_exclusive(PIPELINE_LEDGER_PATH, failure)
        raise


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    action = parser.add_mutually_exclusive_group(required=True)
    action.add_argument(
        "--prepare-execution-lock",
        action="store_true",
        help="publish the one no-clobber execution lock after design audit",
    )
    action.add_argument(
        "--verify-execution-lock",
        action="store_true",
        help="recompute every execution-lock source and evidence binding",
    )
    action.add_argument(
        "--execute-once",
        action="store_true",
        help="claim and execute the frozen V12R2 pipeline once",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    if args.prepare_execution_lock:
        payload = prepare_execution_lock()
        print(f"{payload['status']}: {contract.EXECUTION_LOCK_PATH.as_posix()}")
        return 0
    if args.verify_execution_lock:
        payload = verify_execution_lock()
        print(f"{payload['status']}: {contract.EXECUTION_LOCK_PATH.as_posix()}")
        return 0
    ledger = execute_pipeline_once()
    print(f"{ledger['status']}: {contract.PIPELINE_LEDGER_PATH.as_posix()}")
    return 0 if ledger.get("passed") is True else 1


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = [
    "EXECUTION_AUTHORITY",
    "EXECUTION_SOURCE_RELATIVE_PATHS",
    "V12R2ExecutionError",
    "build_execution_lock",
    "execute_pipeline_once",
    "main",
    "prepare_execution_lock",
    "verify_execution_lock",
]
