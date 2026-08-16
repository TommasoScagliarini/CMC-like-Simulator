"""Run the procedural V9R1 correction with no scientific changes from V9."""

from __future__ import annotations

import argparse
import importlib.util
import json
import sys
from pathlib import Path
from types import CodeType, FunctionType
from typing import Any, Sequence


REPO_ROOT = Path(__file__).resolve().parents[1]
VALIDATION_ROOT = REPO_ROOT / "validation"
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))
if str(VALIDATION_ROOT) not in sys.path:
    sys.path.insert(0, str(VALIDATION_ROOT))

import h0_primary_split_v9r1_residual_dagger_contract as contract  # noqa: E402
import h0_primary_split_v9r1_teacher_compat_contract as teacher_contract  # noqa: E402


class V9R1BindingError(RuntimeError):
    pass


def _load_isolated_v9_wrapper() -> Any:
    path = VALIDATION_ROOT / "run_h0_primary_split_v9_residual_dagger.py"
    name = "_h0_primary_split_v9r1_isolated_base_wrapper"
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise V9R1BindingError("cannot load isolated V9 wrapper")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def _rewrite_code(code: CodeType) -> CodeType:
    constants: list[Any] = []
    for value in code.co_consts:
        if isinstance(value, CodeType):
            constants.append(_rewrite_code(value))
        elif isinstance(value, str):
            constants.append(value.replace("V9", "V9R1").replace("v9", "v9r1"))
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
    return rewritten


base = _load_isolated_v9_wrapper()
engine = base.engine
base.contract = contract
base.teacher_contract = teacher_contract
engine.contract = contract
engine.teacher_contract = teacher_contract
engine.__file__ = str(Path(__file__).resolve())

for key, value in list(vars(engine).items()):
    if isinstance(value, FunctionType) and value.__module__ == engine.__name__:
        setattr(engine, key, _rewrite_function(value))

engine.RUN_ROOT = engine.resolve_relative(contract.RUN_ROOT)
engine.PIPELINE_CLAIM = engine.resolve_relative(contract.PIPELINE_CLAIM_PATH)
engine.PIPELINE_LEDGER = engine.resolve_relative(contract.PIPELINE_LEDGER_PATH)
engine.WORKER_CLAIMS_ROOT = engine.resolve_relative(contract.WORKER_CLAIMS_ROOT)
engine.SOURCE_H0_MODULE = engine.resolve_relative(contract.SOURCE_H0_MODULE_PATH)
engine.P0_MODULE = engine.resolve_relative(contract.P0_MODULE_PATH)
engine.P1_MODULE = engine.resolve_relative(contract.P1_MODULE_PATH)
engine.CANDIDATE_FREEZE = engine.resolve_relative(contract.CANDIDATE_FREEZE_PATH)
engine.DEVELOPMENT_RECEIPT = engine.resolve_relative(
    contract.DEVELOPMENT_RECEIPT_PATH
)
engine.build_paired_views = base._causal_paired_views
engine._legacy_shadow_fsm = base._unused_legacy_shadow_fsm
engine.v1 = base.SimpleNamespace(_update_shadow_fsm=lambda *_args, **_kwargs: None)
engine._is_fsm_event_rejection = lambda _error: False

base.RUN_ROOT = engine.RUN_ROOT
base.PIPELINE_CLAIM = engine.PIPELINE_CLAIM
base.PIPELINE_LEDGER = engine.PIPELINE_LEDGER
base.P0_MODULE = engine.P0_MODULE
base.P1_MODULE = engine.P1_MODULE
base.CANDIDATE_FREEZE = engine.CANDIDATE_FREEZE
base.DEVELOPMENT_RECEIPT = engine.DEVELOPMENT_RECEIPT
base.PREFLIGHT = engine.resolve_relative(contract.PREFLIGHT_PATH)
base.LOCK = engine.resolve_relative(contract.LOCK_PATH)

IMPLEMENTATION_BINDING = {
    **base.IMPLEMENTATION_BINDING,
    "binding_id": "H0_PRIMARY_SPLIT_V9R1_PROCEDURAL_ALIAS_OVERLAY_V1",
    "v9_terminal_fail_preserved": True,
    "scientific_change_from_v9": False,
    "procedural_correction": "ADD_CANONICAL_35_84_FEATURE_NAME_ALIASES",
    "wrapper_source": Path(__file__).relative_to(REPO_ROOT).as_posix(),
}
base.IMPLEMENTATION_BINDING = IMPLEMENTATION_BINDING

# Rebind the V9 wrapper functions to the fresh globals, then rewrite only their
# user-facing lineage literals.  Numerical code and fit specs stay unchanged.
base.build_preflight = _rewrite_function(base.build_preflight)
base.prepare = _rewrite_function(base.prepare)
base.verify_execution_lock = _rewrite_function(base.verify_execution_lock)
base._fixed_execute = _rewrite_function(base._fixed_execute)

_base_build_preflight = base.build_preflight


def build_preflight() -> dict[str, Any]:
    payload = _base_build_preflight()
    v9_ledger = engine._mapping(
        engine.resolve_relative(contract.INPUT_RELATIVE_PATHS["v9_terminal_fail_ledger"])
    )
    v9_p0 = engine._mapping(
        engine.resolve_relative(contract.INPUT_RELATIVE_PATHS["v9_p0_receipt"])
    )
    payload["checks"]["v9_terminal_fail_preserved"] = (
        v9_ledger.get("passed") is False
        and v9_ledger.get("retry_authorized") is False
        and v9_ledger.get("actor_updates") == 1
    )
    payload["checks"]["v9_p0_scientific_pass_preserved"] = (
        v9_p0.get("passed") is True
        and v9_p0.get("actor_updates") == 1
        and v9_p0.get("critic_updates") == 0
        and v9_p0.get("ppo_updates") == 0
    )
    payload["checks"]["feature_name_aliases_exact"] = (
        len(teacher_contract.EXPECTED_ACTOR_FEATURE_NAMES) == 35
        and len(teacher_contract.EXPECTED_OBSERVATION_FEATURE_NAMES) == 84
    )
    payload["passed"] = all(payload["checks"].values())
    payload["status"] = (
        "PASS_H0_PRIMARY_SPLIT_V9R1_RESIDUAL_DAGGER_PREFLIGHT"
        if payload["passed"]
        else "FAIL_H0_PRIMARY_SPLIT_V9R1_RESIDUAL_DAGGER_PREFLIGHT"
    )
    payload["procedural_correction"] = {
        "feature_name_aliases_added": True,
        "target_changed": False,
        "fit_changed": False,
        "gate_changed": False,
    }
    return payload


base.build_preflight = build_preflight
engine._claim_payload = base._claim_payload
engine.execute = base._fixed_execute


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
            result = base.prepare()
            print(json.dumps(result, indent=2, sort_keys=True, allow_nan=False))
        elif args.execute:
            base._fixed_execute()
        else:
            if args.stage is None or args.supervisor_token is None:
                raise V9R1BindingError("worker arguments are incomplete")
            engine.run_worker(
                stage_id=args.stage,
                supervisor_token=args.supervisor_token,
            )
    except Exception as exc:
        print(f"V9R1 failed closed: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 2
    return 0


for _name in (
    "load_teacher_corpus", "verify_teacher_replay", "_collect_dagger", "_develop",
    "_fit_stage", "_freeze_p1", "_worker_command", "_dagger_start_payload",
):
    globals()[_name] = getattr(engine, _name)


if __name__ == "__main__":
    raise SystemExit(main())
