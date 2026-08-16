"""Execute Q2 once through an isolated, byte-bound Q1 execution engine.

Only the generic, already tested orchestration/forensic engine is reused.  Its
private module instance is rebound to the Q2 runtime contract, Q2 gates, Q2
noise closure, Q2 protocol freezer, paths, and statuses.  No global Q1 module
is mutated.  Import performs no environment load, write, lock publication, or
rollout.
"""

from __future__ import annotations

import argparse
import copy
import importlib.util
import sys
from collections.abc import Sequence
from pathlib import Path
from typing import Any


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
ROOT_VALIDATION = REPO_ROOT / "validation"
LOCAL_VALIDATION = REPO_ROOT / "Trajectory Generator" / "baseline_MLP" / "validation"
Q1_ROOT = LOCAL_VALIDATION / "v12p1q"
Q2_ROOT = Path(__file__).resolve().parent
for _root in (REPO_ROOT, ROOT_VALIDATION, LOCAL_VALIDATION, Q1_ROOT, Q2_ROOT):
    if str(_root) not in sys.path:
        sys.path.insert(0, str(_root))

import freeze_h0_v12r4_q2_qualification_protocol as protocol_freezer  # noqa: E402
import h0_v12r4_q2_qualification_gates as gates  # noqa: E402
import h0_v12r4_q2_runtime_contract as contract  # noqa: E402
import prepare_h0_v12r4_q2_qualification_noise_tapes as noise  # noqa: E402


class V12R4Q2QualificationExecutionError(RuntimeError):
    """Raised when the one-shot Q2 runtime closure cannot continue."""


Q1_ENGINE_PATH = Q1_ROOT / "run_h0_v12r3_p1_qualification.py"
LOCK_PASS_STATUS = contract.EXECUTION_LOCK_PASS_STATUS
PIPELINE_CLAIM_STATUS = "CLAIM_H0_V12R4_Q2_QUALIFICATION_PIPELINE"
WORKER_CLAIM_STATUS = "CLAIM_H0_V12R4_Q2_QUALIFICATION_WORKER"
ROLLOUT_STARTED_STATUS = "STARTED_H0_V12R4_Q2_QUALIFICATION_ROLLOUT"
ROLLOUT_PERSISTED_STATUS = "PERSISTED_H0_V12R4_Q2_QUALIFICATION_BEFORE_GATE"
STAGE_FAILURE_STATUS = "FAIL_H0_V12R4_Q2_QUALIFICATION_STAGE"
LEDGER_PASS_STATUS = "PASS_H0_V12R4_Q2_QUALIFICATION_PIPELINE_TERMINAL"
LEDGER_FAIL_STATUS = "FAIL_H0_V12R4_Q2_QUALIFICATION_PIPELINE_TERMINAL"
NEXT_STAGE_AFTER_PASS = "WAIT_SEPARATE_ZERO_UPDATE_PROTOCOL"


def _load_isolated_q1_engine() -> Any:
    spec = importlib.util.spec_from_file_location(
        "_v12r4q2_frozen_q1_execution_engine", Q1_ENGINE_PATH
    )
    if spec is None or spec.loader is None:
        raise V12R4Q2QualificationExecutionError(
            "frozen Q1 execution engine could not be loaded"
        )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


_ENGINE = _load_isolated_q1_engine()
_ENGINE.contract = contract
_ENGINE.gates = gates
_ENGINE.noise = noise
_ENGINE.protocol_freezer = protocol_freezer
_ENGINE.RUN_ROOT = REPO_ROOT.joinpath(*contract.RUN_ROOT.parts)
_ENGINE.LOCK_PATH = REPO_ROOT.joinpath(*contract.EXECUTION_LOCK_PATH.parts)
_ENGINE.PROTOCOL_FREEZE_PATH = REPO_ROOT.joinpath(*contract.PROTOCOL_FREEZE_PATH.parts)
_ENGINE.PIPELINE_CLAIM_PATH = REPO_ROOT.joinpath(*contract.PIPELINE_CLAIM_PATH.parts)
_ENGINE.PIPELINE_LEDGER_PATH = REPO_ROOT.joinpath(*contract.PIPELINE_LEDGER_PATH.parts)
_ENGINE.NOISE_ROOT = REPO_ROOT.joinpath(*contract.NOISE_ROOT.parts)
_ENGINE.LOCK_PASS_STATUS = LOCK_PASS_STATUS
_ENGINE.PIPELINE_CLAIM_STATUS = PIPELINE_CLAIM_STATUS
_ENGINE.WORKER_CLAIM_STATUS = WORKER_CLAIM_STATUS
_ENGINE.ROLLOUT_STARTED_STATUS = ROLLOUT_STARTED_STATUS
_ENGINE.ROLLOUT_PERSISTED_STATUS = ROLLOUT_PERSISTED_STATUS
_ENGINE.STAGE_FAILURE_STATUS = STAGE_FAILURE_STATUS
_ENGINE.LEDGER_PASS_STATUS = LEDGER_PASS_STATUS
_ENGINE.LEDGER_FAIL_STATUS = LEDGER_FAIL_STATUS

EXECUTION_AUTHORITY = copy.deepcopy(_ENGINE.EXECUTION_AUTHORITY)
EXPECTED_TERMINAL_ACTIVITY = copy.deepcopy(_ENGINE.EXPECTED_TERMINAL_ACTIVITY)

_original_ledger_payload = _ENGINE._ledger_payload


def _q2_ledger_payload(**kwargs: Any) -> dict[str, Any]:
    payload = dict(_original_ledger_payload(**kwargs))
    payload["next_stage"] = (
        NEXT_STAGE_AFTER_PASS if payload.get("passed") is True else "STOP_TERMINAL"
    )
    payload["checkpoint_zero_created"] = False
    return payload


_ENGINE._ledger_payload = _q2_ledger_payload


def _sync_candidate_from_protocol() -> dict[str, Any]:
    try:
        freeze = protocol_freezer.verify_protocol_freeze()
        candidate_id = freeze["selected_candidate_id"]
        candidate_module = freeze["selected_candidate"]
        return contract.bind_candidate(candidate_id, candidate_module)
    except Exception as exc:
        raise V12R4Q2QualificationExecutionError(
            "Q2 protocol/candidate binding is not ready"
        ) from exc


def build_execution_lock(*, require_unoccupied: bool = True) -> dict[str, Any]:
    _sync_candidate_from_protocol()
    try:
        payload = dict(
            _ENGINE.build_execution_lock(require_unoccupied=require_unoccupied)
        )
    except Exception as exc:
        raise V12R4Q2QualificationExecutionError(
            "Q2 execution-lock build failed"
        ) from exc
    if (
        payload.get("candidate_id") != contract.P1_CANDIDATE_ID
        or payload.get("candidate_module") != contract.P1_CANDIDATE_MODULE
        or payload.get("rollout_matrix") != list(contract.ROLLOUT_MATRIX)
        or payload.get("stage_order") != list(contract.STAGE_IDS)
    ):
        raise V12R4Q2QualificationExecutionError(
            "Q2 execution-lock candidate/matrix drifted"
        )
    return payload


def prepare_execution_lock() -> dict[str, Any]:
    _sync_candidate_from_protocol()
    try:
        return dict(_ENGINE.prepare_execution_lock())
    except Exception as exc:
        raise V12R4Q2QualificationExecutionError(
            "Q2 execution-lock publication failed"
        ) from exc


def verify_execution_lock(*, require_run_root_absent: bool = False) -> dict[str, Any]:
    _sync_candidate_from_protocol()
    try:
        return dict(
            _ENGINE.verify_execution_lock(
                require_run_root_absent=require_run_root_absent
            )
        )
    except Exception as exc:
        raise V12R4Q2QualificationExecutionError(
            "Q2 execution lock/current closure verification failed"
        ) from exc


def execute_qualification_once() -> dict[str, Any]:
    """Execute baseline six, candidate six, and aggregate exactly once."""

    _sync_candidate_from_protocol()
    try:
        payload = dict(_ENGINE.execute_qualification_once())
    except Exception as exc:
        raise V12R4Q2QualificationExecutionError(
            "Q2 one-shot qualification failed terminally"
        ) from exc
    if (
        payload.get("status") != LEDGER_PASS_STATUS
        or payload.get("passed") is not True
        or payload.get("next_stage") != NEXT_STAGE_AFTER_PASS
        or payload.get("checkpoint_zero_created") is not False
    ):
        raise V12R4Q2QualificationExecutionError(
            "Q2 terminal ledger did not stop before zero-update"
        )
    return payload


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    action = parser.add_mutually_exclusive_group(required=True)
    action.add_argument("--prepare-execution-lock", action="store_true")
    action.add_argument("--verify-execution-lock", action="store_true")
    action.add_argument("--execute-once", action="store_true")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    if args.prepare_execution_lock:
        payload = prepare_execution_lock()
    elif args.verify_execution_lock:
        payload = verify_execution_lock()
    else:
        payload = execute_qualification_once()
    print(payload["status"])
    return 0 if payload.get("passed") is True else 1


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())


__all__ = [
    "EXECUTION_AUTHORITY",
    "LEDGER_FAIL_STATUS",
    "LEDGER_PASS_STATUS",
    "NEXT_STAGE_AFTER_PASS",
    "V12R4Q2QualificationExecutionError",
    "build_execution_lock",
    "execute_qualification_once",
    "prepare_execution_lock",
    "verify_execution_lock",
]
