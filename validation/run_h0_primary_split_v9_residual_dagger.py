"""Run the isolated V9/V26 event-causal residual-DAgger pipeline.

The reviewed V6 numerical engine is loaded into a private module namespace and
bound explicitly to fresh V9 contracts and paths.  Unlike V8's textual source
substitution, this wrapper leaves the source bytes untouched, replaces only
audited runtime bindings, and records the binding in the execution claim.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import os
import subprocess
import sys
import time
from pathlib import Path
from types import CodeType, FunctionType, SimpleNamespace
from typing import Any, Mapping, Sequence


REPO_ROOT = Path(__file__).resolve().parents[1]
VALIDATION_ROOT = REPO_ROOT / "validation"
TRAJECTORY_ROOT = REPO_ROOT / "Trajectory Generator"
BASELINE_ROOT = TRAJECTORY_ROOT / "baseline_MLP"
for root in (REPO_ROOT, VALIDATION_ROOT, TRAJECTORY_ROOT, BASELINE_ROOT):
    if str(root) not in sys.path:
        sys.path.insert(0, str(root))

import h0_forensic_rollout as forensic  # noqa: E402
import h0_primary_split_v9_causal_teacher_contract as teacher_contract  # noqa: E402
import h0_primary_split_v9_residual_dagger_contract as contract  # noqa: E402
import run_h0_primary_split_v9_causal_teacher as teacher_collector  # noqa: E402
from h0_primary_split_v9_causal_teacher import (  # noqa: E402
    assert_causal_pair,
    validate_actor_feature_names,
)
from primary_grf_split_adaptation import (  # noqa: E402
    PHASE_FEATURES,
    PHASE_FEATURE_INDICES,
    build_paired_views as _base_build_paired_views,
)


_ENGINE_PATH = VALIDATION_ROOT / "run_h0_primary_split_v6_residual_dagger.py"
_ENGINE_SHA256 = "386a43042815c1e14ce1ba3afe09f8e6b5b607694112c780ee3bd4a772ea1ec2"


class V9ResidualDaggerBindingError(RuntimeError):
    pass


def _rewrite_code(code: CodeType) -> CodeType:
    constants: list[Any] = []
    for value in code.co_consts:
        if isinstance(value, CodeType):
            constants.append(_rewrite_code(value))
        elif isinstance(value, str):
            constants.append(
                value.replace(
                    "H0_ONLINE_GRF_DETECTOR_LEGACY_EVENTS_SHADOW_FSM",
                    contract.TEACHER_ID,
                )
                .replace(
                    "P0_CLOSED_LOOP_V25_BINARY_ACTIVE",
                    "P0_CLOSED_LOOP_V26_BINARY_ACTIVE",
                )
                .replace("V6", "V9")
                .replace("v6", "v9")
            )
        else:
            constants.append(value)
    return code.replace(co_consts=tuple(constants))


def _rewrite_function(function: FunctionType) -> FunctionType:
    rewritten = FunctionType(
        _rewrite_code(function.__code__),
        function.__globals__,
        function.__name__,
        function.__defaults__,
        function.__closure__,
    )
    rewritten.__kwdefaults__ = function.__kwdefaults__
    rewritten.__annotations__ = dict(function.__annotations__)
    rewritten.__dict__.update(function.__dict__)
    rewritten.__doc__ = function.__doc__
    rewritten.__module__ = function.__module__
    return rewritten


def _load_isolated_engine() -> Any:
    raw = _ENGINE_PATH.read_bytes()
    observed = hashlib.sha256(raw).hexdigest()
    if observed != _ENGINE_SHA256:
        raise V9ResidualDaggerBindingError(
            f"V6 numerical engine drifted: {_ENGINE_SHA256} != {observed}"
        )
    name = "_h0_primary_split_v9_isolated_numerical_engine"
    spec = importlib.util.spec_from_file_location(name, _ENGINE_PATH)
    if spec is None or spec.loader is None:
        raise V9ResidualDaggerBindingError("cannot load isolated numerical engine")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)

    module.contract = contract
    module.teacher_contract = teacher_contract
    module.teacher_collector = teacher_collector
    module.__file__ = str(Path(__file__).resolve())
    module.RUN_ROOT = module.resolve_relative(contract.RUN_ROOT)
    module.PIPELINE_CLAIM = module.resolve_relative(contract.PIPELINE_CLAIM_PATH)
    module.PIPELINE_LEDGER = module.resolve_relative(contract.PIPELINE_LEDGER_PATH)
    module.WORKER_CLAIMS_ROOT = module.resolve_relative(contract.WORKER_CLAIMS_ROOT)
    module.SOURCE_H0_MODULE = module.resolve_relative(contract.SOURCE_H0_MODULE_PATH)
    module.P0_MODULE = module.resolve_relative(contract.P0_MODULE_PATH)
    module.P1_MODULE = module.resolve_relative(contract.P1_MODULE_PATH)
    module.CANDIDATE_FREEZE = module.resolve_relative(contract.CANDIDATE_FREEZE_PATH)
    module.DEVELOPMENT_RECEIPT = module.resolve_relative(
        contract.DEVELOPMENT_RECEIPT_PATH
    )

    for key, value in list(vars(module).items()):
        if isinstance(value, FunctionType) and value.__module__ == name:
            setattr(module, key, _rewrite_function(value))
    return module


engine = _load_isolated_engine()


def _causal_paired_views(
    observation: Any,
    actor_feature_names: Sequence[str],
    info: Mapping[str, Any],
    *,
    body_weight_n: float,
    reset_boundary: bool = False,
    teacher_phase_observation: Mapping[str, Any] | None = None,
) -> Any:
    """Adapt the legacy helper signature while ignoring its shadow-FSM input."""

    del teacher_phase_observation
    validate_actor_feature_names(actor_feature_names)
    actor = engine._float32_array(
        observation[:35], shape=(35,), label="V9 causal actor view", np=__import__("numpy")
    )
    causal_phase = {
        name: float(actor[index])
        for index, name in zip(PHASE_FEATURE_INDICES, PHASE_FEATURES)
    }
    paired = _base_build_paired_views(
        observation,
        actor_feature_names,
        info,
        body_weight_n=body_weight_n,
        reset_boundary=reset_boundary,
        teacher_phase_observation=causal_phase,
    )
    assert_causal_pair(paired.student, paired.teacher)
    return paired


class _UnusedLegacyShadow:
    def observation(self) -> dict[str, float]:
        return {}


def _unused_legacy_shadow_fsm(_base: Any) -> _UnusedLegacyShadow:
    return _UnusedLegacyShadow()


engine.build_paired_views = _causal_paired_views
engine._legacy_shadow_fsm = _unused_legacy_shadow_fsm
engine.v1 = SimpleNamespace(_update_shadow_fsm=lambda *_args, **_kwargs: None)
engine._is_fsm_event_rejection = lambda _error: False

PREFLIGHT = engine.resolve_relative(contract.PREFLIGHT_PATH)
LOCK = engine.resolve_relative(contract.LOCK_PATH)

IMPLEMENTATION_BINDING = {
    "binding_id": "H0_PRIMARY_SPLIT_V9_ISOLATED_V6_NUMERICAL_ENGINE_V1",
    "source_path": _ENGINE_PATH.relative_to(REPO_ROOT).as_posix(),
    "source_sha256": _ENGINE_SHA256,
    "isolated_module_namespace": engine.__name__,
    "code_constant_namespace_rewrite": {
        "V6": "V9",
        "v6": "v9",
        "teacher": contract.TEACHER_ID,
        "behavior": "P0_CLOSED_LOOP_V26_BINARY_ACTIVE",
    },
    "paired_view_builder": "ONLY_10_11_PRIVILEGED_ALL_OTHER_COLUMNS_V26_EXACT",
    "legacy_shadow_fsm_influence": False,
    "partial_fsm_rejection_acceptance": False,
}

_original_claim_payload = engine._claim_payload


def _claim_payload(token_sha256: str) -> dict[str, Any]:
    payload = dict(_original_claim_payload(token_sha256))
    payload["implementation_binding"] = dict(IMPLEMENTATION_BINDING)
    payload["teacher_id"] = contract.TEACHER_ID
    payload["offline_thresholds"] = dict(contract.OFFLINE_THRESHOLDS)
    payload["execution_lock"] = engine._record(LOCK) if LOCK.is_file() else None
    return payload


engine._claim_payload = _claim_payload


def _source_records() -> dict[str, dict[str, Any]]:
    return {
        name: engine._record(engine.resolve_relative(path))
        for name, path in contract.SOURCE_RELATIVE_PATHS.items()
    }


def _input_records() -> dict[str, dict[str, Any]]:
    return {
        name: engine._record(engine.resolve_relative(path))
        for name, path in contract.INPUT_RELATIVE_PATHS.items()
    }


def build_preflight() -> dict[str, Any]:
    if PREFLIGHT.exists() or LOCK.exists():
        raise V9ResidualDaggerBindingError("V9 residual preflight path occupied")
    engine._preexecution_absence()
    teacher = engine.verify_teacher_replay()
    import numpy as np

    corpus = engine.load_teacher_corpus(np=np)
    target_sha = engine.array_sha256(corpus["targets"])
    v8_fail = engine._mapping(
        engine.resolve_relative(contract.INPUT_RELATIVE_PATHS["v8_terminal_fail_ledger"])
    )
    checks = {
        "causal_teacher_pass": teacher.get("passed") is True,
        "causal_teacher_protocol": teacher.get("protocol_id")
        == teacher_contract.PROTOCOL_ID,
        "teacher_samples": corpus["observations"].shape == (3000, 35)
        and corpus["targets"].shape == (3000, 2),
        "teacher_target_digest": target_sha
        == "e55b61161ff4b671e9a609c36dd7381665dda534b300fdc1efd92b8db198a6e6",
        "teacher_finite": bool(
            np.all(np.isfinite(corpus["observations"]))
            and np.all(np.isfinite(corpus["targets"]))
        ),
        "v8_terminal_fail_preserved": v8_fail.get("passed") is False
        and v8_fail.get("retry_authorized") is False,
        "v26_contract": contract.EVENT_CONTRACT_ID
        == "binary_point_v25+heel_qualified_fsm_v2",
        "causal_teacher": contract.TEACHER_ID
        == "H0_ANALOG_LOAD_CONTACT_WITH_V26_EVENTS_FSM_V1",
        "data_driven_ankle_limit": contract.RESIDUAL_LIMITS == (0.175, 0.15),
        "protected_closed": True,
        "reserve_closed": True,
    }
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": (
            "PASS_H0_PRIMARY_SPLIT_V9_RESIDUAL_DAGGER_PREFLIGHT"
            if all(checks.values())
            else "FAIL_H0_PRIMARY_SPLIT_V9_RESIDUAL_DAGGER_PREFLIGHT"
        ),
        "passed": all(checks.values()),
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "checks": checks,
        "teacher_target_sha256": target_sha,
        "implementation_binding": dict(IMPLEMENTATION_BINDING),
        "sources": _source_records(),
        "inputs": _input_records(),
        "source_h0": engine._tree_record(engine.SOURCE_H0_MODULE),
        "offline_thresholds": dict(contract.OFFLINE_THRESHOLDS),
        "residual_limits": list(contract.RESIDUAL_LIMITS),
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
        "next_stage": "FREEZE_V9_RESIDUAL_DAGGER" if all(checks.values()) else "STOP",
    }


def prepare() -> dict[str, Any]:
    preflight = build_preflight()
    if not preflight["passed"]:
        raise V9ResidualDaggerBindingError("V9 residual preflight failed")
    forensic.write_json_exclusive(PREFLIGHT, preflight)
    lock = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": "H0_PRIMARY_SPLIT_V9_RESIDUAL_DAGGER_FROZEN",
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "preflight": engine._record(PREFLIGHT),
        "implementation_binding": dict(IMPLEMENTATION_BINDING),
        "sources": _source_records(),
        "inputs": _input_records(),
        "source_h0": engine._tree_record(engine.SOURCE_H0_MODULE),
        "stage_order": list(contract.STAGE_IDS),
        "teacher_id": contract.TEACHER_ID,
        "offline_thresholds": dict(contract.OFFLINE_THRESHOLDS),
        "residual_limits": list(contract.RESIDUAL_LIMITS),
        "authority": dict(contract.AUTHORITY),
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
        "next_stage": "EXECUTE_V9_RESIDUAL_DAGGER_ONCE",
    }
    forensic.write_json_exclusive(LOCK, lock)
    return {"preflight": preflight, "lock": lock}


def verify_execution_lock() -> dict[str, Any]:
    observed = engine._mapping(LOCK)
    expected = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": "H0_PRIMARY_SPLIT_V9_RESIDUAL_DAGGER_FROZEN",
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "preflight": engine._record(PREFLIGHT),
        "implementation_binding": dict(IMPLEMENTATION_BINDING),
        "sources": _source_records(),
        "inputs": _input_records(),
        "source_h0": engine._tree_record(engine.SOURCE_H0_MODULE),
        "stage_order": list(contract.STAGE_IDS),
        "teacher_id": contract.TEACHER_ID,
        "offline_thresholds": dict(contract.OFFLINE_THRESHOLDS),
        "residual_limits": list(contract.RESIDUAL_LIMITS),
        "authority": dict(contract.AUTHORITY),
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
        "next_stage": "EXECUTE_V9_RESIDUAL_DAGGER_ONCE",
    }
    if forensic.canonical_json_bytes(observed) != forensic.canonical_json_bytes(
        expected
    ):
        raise V9ResidualDaggerBindingError("V9 residual execution lock drifted")
    return observed


def _fixed_execute() -> dict[str, Any]:
    """Execute once and account for a published terminal FAIL receipt."""

    verify_execution_lock()
    engine._preexecution_absence()
    token = engine.secrets.token_urlsafe(32)
    token_sha256 = engine._token_sha256(token)
    forensic.write_json_exclusive(
        engine.PIPELINE_CLAIM, _claim_payload(token_sha256)
    )
    started = time.time()
    attempted_stages: list[str] = []
    passed_stages: list[str] = []
    receipt_published_stages: list[str] = []
    published_receipts: list[dict[str, Any]] = []
    actor_updates = 0
    accounting_complete = True
    error: str | None = None
    passed = False
    try:
        for stage_id in contract.STAGE_IDS:
            attempted_stages.append(stage_id)
            previous = [
                {
                    "stage_id": prior,
                    "receipt": engine._record(engine._stage_receipt_path(prior)),
                }
                for prior in passed_stages
            ]
            forensic.write_json_exclusive(
                engine._claim_path(stage_id),
                engine._worker_claim_payload(
                    stage_id=stage_id,
                    token_sha256=token_sha256,
                    previous_receipts=previous,
                ),
            )
            completed = subprocess.run(
                engine._worker_command(stage_id, token),
                cwd=REPO_ROOT,
                timeout=engine.WORKER_TIMEOUT_S,
                check=False,
            )
            receipt_path = engine._stage_receipt_path(stage_id)
            if os.path.lexists(receipt_path):
                receipt = engine._mapping(receipt_path)
                receipt_published_stages.append(stage_id)
                published_receipts.append(
                    {"stage_id": stage_id, "receipt": engine._record(receipt_path)}
                )
                if stage_id in {"fit_p0", "fit_p1"}:
                    updates = receipt.get("actor_updates")
                    if type(updates) is not int or updates != 1:
                        accounting_complete = False
                    else:
                        actor_updates += updates
            else:
                accounting_complete = False
            if completed.returncode != 0:
                raise engine.V6ResidualDaggerError(
                    f"worker {stage_id} exited {completed.returncode}"
                )
            engine.verify_stage_receipt(stage_id)
            passed_stages.append(stage_id)
        development = engine._mapping(engine.DEVELOPMENT_RECEIPT)
        freeze = engine._candidate_freeze()
        if (
            development.get("status") != contract.DEVELOPMENT_PASS_STATUS
            or development.get("passed") is not True
            or development.get("candidate_id") != freeze["candidate_id"]
        ):
            raise engine.V6ResidualDaggerError("terminal development closure drifted")
        passed = True
    except Exception as exc:
        error = f"{type(exc).__name__}: {exc}"

    ledger = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.PIPELINE_PASS_STATUS if passed else contract.PIPELINE_FAIL_STATUS,
        "passed": passed,
        "protocol_id": contract.PROTOCOL_ID,
        "pipeline_id": contract.PIPELINE_ID,
        "started_unix_s": started,
        "completed_unix_s": time.time(),
        "expected_stages": list(contract.STAGE_IDS),
        "attempted_stages": attempted_stages,
        "receipt_published_stages": receipt_published_stages,
        "passed_stages": passed_stages,
        "completed_stages": passed_stages,
        "completed_receipts": published_receipts,
        "published_receipts": published_receipts,
        "error": error,
        "pipeline_claim": engine._record(engine.PIPELINE_CLAIM),
        "candidate_freeze": engine._record(engine.CANDIDATE_FREEZE) if passed else None,
        "development_receipt": (
            engine._record(engine.DEVELOPMENT_RECEIPT) if passed else None
        ),
        "dagger_rounds": 1 if "fit_p1" in passed_stages else 0,
        "actor_updates": actor_updates,
        "update_accounting_complete": accounting_complete,
        "retry_authorized": False,
        "second_dagger_round_authorized": False,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "next_stage": (
            "PREPARE_FRESH_V9_QUALIFICATION"
            if passed
            else "STOP_WITHOUT_RETRY_RETUNING_OR_SECOND_DAGGER_ROUND"
        ),
    }
    forensic.write_json_exclusive(engine.PIPELINE_LEDGER, ledger)
    print(json.dumps(ledger, indent=2, sort_keys=True, allow_nan=False), flush=True)
    if not passed:
        raise engine.V6ResidualDaggerError(error or contract.PIPELINE_FAIL_STATUS)
    return ledger


engine.execute = _fixed_execute


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--prepare", action="store_true")
    mode.add_argument("--execute", action="store_true")
    mode.add_argument("--worker", action="store_true")
    parser.add_argument("--stage", choices=contract.STAGE_IDS)
    parser.add_argument("--supervisor-token")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        if args.prepare:
            if args.stage is not None or args.supervisor_token is not None:
                raise V9ResidualDaggerBindingError(
                    "preparation does not accept worker arguments"
                )
            print(json.dumps(prepare(), indent=2, sort_keys=True, allow_nan=False))
        elif args.execute:
            if args.stage is not None or args.supervisor_token is not None:
                raise V9ResidualDaggerBindingError(
                    "supervisor execution does not accept worker arguments"
                )
            _fixed_execute()
        else:
            if args.stage is None or args.supervisor_token is None:
                raise V9ResidualDaggerBindingError(
                    "worker requires --stage and --supervisor-token"
                )
            engine.run_worker(
                stage_id=args.stage, supervisor_token=args.supervisor_token
            )
    except Exception as exc:
        print(f"V9 failed closed: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 2
    return 0


# Focused tests import these reviewed engine surfaces from the fresh wrapper.
for _name in (
    "RUN_ROOT", "P0_MODULE", "P1_MODULE", "CANDIDATE_FREEZE",
    "DEVELOPMENT_RECEIPT", "PIPELINE_CLAIM", "PIPELINE_LEDGER",
    "load_teacher_corpus", "verify_teacher_replay", "_collect_dagger",
    "_develop", "_fit_stage", "_freeze_p1", "_worker_command",
    "_is_fsm_event_rejection", "_dagger_start_payload",
):
    globals()[_name] = getattr(engine, _name)


if __name__ == "__main__":
    raise SystemExit(main())
