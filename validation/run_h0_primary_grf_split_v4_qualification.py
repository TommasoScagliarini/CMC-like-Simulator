"""Execute the frozen post-holdout autonomous qualification of H0 V4.

The supervisor completes all six condition-matched source-H0 baselines before
publishing the baseline and tolerance decision.  Candidate access is then
one-shot and sequential.  Importing this module never starts OpenSim.
"""

from __future__ import annotations

import argparse
import contextlib
import json
import subprocess
import sys
import time
import traceback
from pathlib import Path
from typing import Any, Iterator, Mapping, Sequence


REPO_ROOT = Path(__file__).resolve().parents[1]
VALIDATION_ROOT = REPO_ROOT / "validation"
TRAJECTORY_ROOT = REPO_ROOT / "Trajectory Generator"
BASELINE_ROOT = TRAJECTORY_ROOT / "baseline_MLP"
for import_root in (VALIDATION_ROOT, BASELINE_ROOT, TRAJECTORY_ROOT, REPO_ROOT):
    if str(import_root) not in sys.path:
        sys.path.insert(0, str(import_root))

import compare_h0_primary_grf_split_v4_qualification as gates  # noqa: E402
import h0_primary_grf_split_v4_qualification_contract as contract  # noqa: E402
import run_h0_primary_grf_split_v3_qualification as engine  # noqa: E402
import run_h0_primary_grf_split_v4_full_mean as v4  # noqa: E402


LOCK = REPO_ROOT / contract.LOCK_PATH
RUN_ROOT = REPO_ROOT / contract.RUN_ROOT
SOURCE_H0_MODULE = REPO_ROOT / contract.SOURCE_H0_MODULE_PATH
CANDIDATE_MODULE = REPO_ROOT / contract.CANDIDATE_MODULE_PATH
WORKER_TIMEOUT_S = 2400.0


class QualificationExecutionError(RuntimeError):
    """Raised on every provenance, runtime, or gate failure."""


def _record(path: str | Path) -> dict[str, Any]:
    return v4.source_record(Path(path).resolve())


def _record_matches(record: Any, path: str | Path) -> bool:
    return isinstance(record, Mapping) and v4._canonical_bytes(record) == (
        v4._canonical_bytes(_record(path))
    )


def _frozen_record_path(record: Any, label: str) -> Path:
    if not isinstance(record, Mapping) or set(record) != {
        "path",
        "sha256",
        "size_bytes",
    }:
        raise QualificationExecutionError(f"{label} artifact record is malformed")
    relative = record["path"]
    if not isinstance(relative, str) or not relative or Path(relative).is_absolute():
        raise QualificationExecutionError(f"{label} path is not repository-relative")
    resolved = (REPO_ROOT / relative).resolve()
    try:
        resolved.relative_to(REPO_ROOT.resolve())
    except ValueError as exc:
        raise QualificationExecutionError(f"{label} escaped the repository") from exc
    if (
        not resolved.is_file()
        or resolved.stat().st_size
        != v4._require_int(record["size_bytes"], f"{label}.size")
        or v4.v3.sha256_file(resolved) != record["sha256"]
    ):
        raise QualificationExecutionError(f"{label} integrity mismatch")
    return resolved


def validate_v4_post_holdout() -> dict[str, Any]:
    """Require exact V4 candidate/holdout continuity without opening new data."""

    v4.verify_lock()
    candidate_receipt, candidate = v4._validate_candidate_receipt()
    freeze = v4.verify_candidate_freeze()
    ledger = v4._strict_mapping(REPO_ROOT / contract.V4_EXECUTION_LEDGER_PATH)
    receipt_path = REPO_ROOT / contract.HOLDOUT_RECEIPT_PATH
    gate_path = REPO_ROOT / contract.HOLDOUT_GATE_PATH
    receipt = v4._strict_mapping(receipt_path)
    gate = v4._strict_mapping(gate_path)

    expected_receipt_keys = {
        "schema_version",
        "status",
        "passed",
        "gate",
        "candidate_freeze",
        "holdout_access_claim",
        "holdout_replay_receipt",
        "actor_updates",
        "additional_actor_updates",
        "critic_updates",
        "ppo_updates",
        "protected_trials_opened",
    }
    if (
        set(receipt) != expected_receipt_keys
        or receipt.get("schema_version") != 4
        or receipt.get("status") != "PASS_H0_PRIMARY_SPLIT_V4_FINAL_HOLDOUT"
        or receipt.get("passed") is not True
        or receipt.get("actor_updates") != 1
        or receipt.get("additional_actor_updates") != 0
        or receipt.get("critic_updates") != 0
        or receipt.get("ppo_updates") != 0
        or receipt.get("protected_trials_opened") != []
        or not _record_matches(receipt.get("gate"), gate_path)
        or not _record_matches(
            receipt.get("candidate_freeze"),
            REPO_ROOT / contract.CANDIDATE_FREEZE_PATH,
        )
    ):
        raise QualificationExecutionError("V4 final holdout receipt is not canonical")

    expected_gate_keys = {
        "schema_version",
        "status",
        "passed",
        "checks",
        "metrics",
        "offline_thresholds",
        "candidate_freeze",
        "holdout_access_claim",
        "holdout_replay_receipt",
        "actor_updates",
        "additional_actor_updates",
        "critic_updates",
        "ppo_updates",
        "protected_trials_opened",
    }
    checks = gate.get("checks")
    if (
        set(gate) != expected_gate_keys
        or gate.get("schema_version") != 4
        or gate.get("status") != receipt["status"]
        or gate.get("passed") is not True
        or not isinstance(checks, Mapping)
        or not checks
        or not all(value is True for value in checks.values())
        or gate.get("actor_updates") != 1
        or gate.get("additional_actor_updates") != 0
        or gate.get("critic_updates") != 0
        or gate.get("ppo_updates") != 0
        or gate.get("protected_trials_opened") != []
        or gate.get("candidate_freeze") != receipt["candidate_freeze"]
        or gate.get("holdout_access_claim") != receipt["holdout_access_claim"]
        or gate.get("holdout_replay_receipt")
        != receipt["holdout_replay_receipt"]
    ):
        raise QualificationExecutionError("V4 final holdout gate is not canonical")

    if (
        ledger.get("schema_version") != 4
        or ledger.get("status") != receipt["status"]
        or ledger.get("passed") is not True
        or ledger.get("terminal_stage") != "final_holdout_complete"
        or ledger.get("error") is not None
        or ledger.get("v3_corpus_reused") is not True
        or ledger.get("v3_failed_candidate_reused") is not False
        or ledger.get("candidate_created") is not True
        or ledger.get("candidate_frozen_before_holdout") is not True
        or ledger.get("holdout_access_claimed") is not True
        or ledger.get("holdout_replay_completed") is not True
        or ledger.get("final_holdout_completed") is not True
        or ledger.get("actor_update_candidates") != 1
        or ledger.get("critic_updates") != 0
        or ledger.get("ppo_updates") != 0
        or ledger.get("protected_trials_opened") != []
        or ledger.get("retry_or_retuning_allowed") is not False
        or ledger.get("next_stage") != "CANONICAL_CLOSED_LOOP_QUALIFICATION"
        or not _record_matches(
            ledger.get("execution_lock"),
            REPO_ROOT / contract.V4_EXECUTION_LOCK_PATH,
        )
    ):
        raise QualificationExecutionError("V4 terminal execution ledger is not a PASS")

    if (
        freeze.get("schema_version") != 4
        or freeze.get("status")
        != "H0_PRIMARY_SPLIT_V4_CANDIDATE_FROZEN_BEFORE_HOLDOUT"
        or freeze.get("protocol_id") != contract.SOURCE_PROTOCOL_ID
        or freeze.get("candidate_id") != contract.CANDIDATE_ID
        or freeze.get("holdout_accessed_before_freeze") is not False
        or freeze.get("actor_updates") != 1
        or freeze.get("critic_updates") != 0
        or freeze.get("ppo_updates") != 0
        or freeze.get("protected_trials_opened") != []
        or not _record_matches(
            freeze.get("candidate_receipt"),
            v4.ADAPTATION_DIR / "receipt.json",
        )
        or not _record_matches(freeze.get("candidate_module_state"), candidate / "module_state.pkl")
        or not _record_matches(
            freeze.get("candidate_module_ctor"),
            candidate / "class_and_ctor_args.pkl",
        )
        or not _record_matches(
            freeze.get("candidate_module_metadata"),
            candidate / "metadata.json",
        )
        or freeze.get("actor_feature_manifest")
        != candidate_receipt.get("actor_feature_manifest")
    ):
        raise QualificationExecutionError("V4 candidate freeze continuity failed")
    manifest_path = candidate / "actor_feature_manifest.json"
    manifest = v4._strict_mapping(manifest_path)
    if (
        manifest.get("candidate_id") != contract.CANDIDATE_ID
        or manifest.get("observation_contract_id") != "primary_grf_split_v1"
        or manifest.get("event_contract_id") != "legacy_events_v1"
        or manifest.get("actor_feature_count") != contract.EXPECTED_ACTOR_FEATURES
        or manifest.get("trainable_scope") != "full_mean_network"
        or manifest.get("logstd_policy") != "frozen_bit_exact"
        or manifest.get("actor_digest") != freeze.get("candidate_actor_digest")
    ):
        raise QualificationExecutionError("V4 candidate actor manifest drifted")
    return {
        "status": "PASS_H0_PRIMARY_SPLIT_V4_POST_HOLDOUT_PREREQUISITE",
        "execution_ledger": _record(REPO_ROOT / contract.V4_EXECUTION_LEDGER_PATH),
        "holdout_receipt": _record(receipt_path),
        "candidate_freeze": _record(REPO_ROOT / contract.CANDIDATE_FREEZE_PATH),
        "candidate_actor_digest": freeze["candidate_actor_digest"],
    }


def verify_lock() -> dict[str, Any]:
    observed = v4._strict_mapping(LOCK)
    from freeze_h0_primary_grf_split_v4_qualification import build_payload

    expected = build_payload(require_unoccupied=False)
    if v4._canonical_bytes(observed) != v4._canonical_bytes(expected):
        raise QualificationExecutionError("V4 qualification lock drifted")
    return observed


def _require_baseline_complete() -> None:
    for case_id in contract.CASE_IDS:
        receipt_path = (
            REPO_ROOT / contract.rollout_destination("baseline", case_id) / "receipt.json"
        )
        receipt = v4._strict_mapping(receipt_path)
        if (
            receipt.get("passed") is not True
            or receipt.get("status") != contract.ROLLOUT_PASS_STATUS
            or receipt.get("role") != "baseline"
            or receipt.get("case_id") != case_id
        ):
            raise QualificationExecutionError(
                "all six baselines must pass before candidate access"
            )


def _validate_baseline_decision() -> dict[str, Any]:
    decision = v4._strict_mapping(REPO_ROOT / contract.DECISION_RECEIPT_PATH)
    expected_keys = {
        "schema_version",
        "status",
        "passed",
        "protocol_id",
        "decision_authority",
        "candidate_holdout_receipt",
        "baseline",
        "tolerances",
        "fixed_gates",
        "runtime_contract",
        "canonical_case_ids",
        "authority",
        "actor_updates",
        "critic_updates",
        "ppo_updates",
        "protected_trials_opened",
    }
    expected_tolerances = {
        "comparison_formula": (
            "candidate <= baseline + max(absolute_tolerance, "
            "relative_tolerance * abs(baseline))"
        ),
        **contract.tolerance_rows(),
    }
    expected_fixed = {
        "expected_steps": contract.EXPECTED_STEPS,
        "minimum_valid_cycles": contract.MINIMUM_VALID_CYCLES,
        "penetration_limit_m": contract.PENETRATION_LIMIT_M,
        "penetration_comparison": "strict_less_than",
        "zero_count_fields": list(contract.ZERO_REQUIRED_COUNTS),
        "nonincreasing_count_fields": list(contract.NONINCREASING_COUNTS),
    }
    if (
        set(decision) != expected_keys
        or decision.get("schema_version") != 4
        or decision.get("status")
        != "H0_PRIMARY_SPLIT_V4_QUALIFICATION_BASELINE_TOLERANCE_DECIDED"
        or decision.get("passed") is not True
        or decision.get("protocol_id") != contract.SOURCE_PROTOCOL_ID
        or decision.get("decision_authority") != "EXPLICIT_USER_DECISION"
        or decision.get("canonical_case_ids") != list(contract.CASE_IDS)
        or decision.get("authority") != contract.AUTHORITY
        or decision.get("tolerances") != expected_tolerances
        or decision.get("fixed_gates") != expected_fixed
        or decision.get("runtime_contract")
        != {
            "event_contract_id": contract.EVENT_CONTRACT_ID,
            "phase_fsm_input_mode": contract.PHASE_FSM_INPUT_MODE,
            "morphology_weight": contract.MORPHOLOGY_WEIGHT,
        }
        or decision.get("actor_updates") != 0
        or decision.get("critic_updates") != 0
        or decision.get("ppo_updates") != 0
        or decision.get("protected_trials_opened") != []
        or not _record_matches(
            decision.get("candidate_holdout_receipt"),
            REPO_ROOT / contract.HOLDOUT_RECEIPT_PATH,
        )
    ):
        raise QualificationExecutionError("V4 baseline/tolerance decision drifted")
    baseline = decision.get("baseline")
    baseline_path = REPO_ROOT / contract.BASELINE_RECEIPT_PATH
    if (
        not isinstance(baseline, Mapping)
        or set(baseline) != {"baseline_id", "comparison_scope", "receipt"}
        or baseline.get("baseline_id")
        != "analog_h0_primary_physics_condition_matched_v1"
        or baseline.get("comparison_scope") != "condition_matched_six_cases"
        or not _record_matches(baseline.get("receipt"), baseline_path)
    ):
        raise QualificationExecutionError("V4 baseline decision record drifted")
    frozen = v4._strict_mapping(baseline_path)
    if (
        frozen.get("schema_version") != 4
        or frozen.get("status")
        != "H0_PRIMARY_SPLIT_V4_QUALIFICATION_BASELINE_FROZEN"
        or frozen.get("passed") is not True
        or tuple(frozen.get("case_metrics", {})) != contract.CASE_IDS
        or frozen.get("actor_updates") != 0
        or frozen.get("critic_updates") != 0
        or frozen.get("ppo_updates") != 0
        or frozen.get("protected_trials_opened") != []
    ):
        raise QualificationExecutionError("V4 baseline receipt drifted")
    return decision


def _enforce_predecessors(role: str) -> None:
    if role == "baseline":
        if (REPO_ROOT / contract.DECISION_RECEIPT_PATH).exists():
            raise QualificationExecutionError(
                "baseline stage cannot run after decision publication"
            )
        return
    _require_baseline_complete()
    _validate_baseline_decision()


@contextlib.contextmanager
def _bind_v3_qualification_engine() -> Iterator[None]:
    """Bind the tested V3 rollout engine to V4 paths in this process only."""

    replacements = {
        "contract": contract,
        "gates": gates,
        "LOCK": LOCK,
        "RUN_ROOT": RUN_ROOT,
        "SOURCE_H0_MODULE": SOURCE_H0_MODULE,
        "CANDIDATE_MODULE": CANDIDATE_MODULE,
        "verify_lock": verify_lock,
        "_enforce_predecessors": _enforce_predecessors,
    }
    original = {name: getattr(engine, name) for name in replacements}
    try:
        for name, value in replacements.items():
            setattr(engine, name, value)
        yield
    finally:
        for name, value in original.items():
            setattr(engine, name, value)


def _run_rollout(*, role: str, case_id: str, output_dir: str | Path) -> dict[str, Any]:
    with _bind_v3_qualification_engine():
        return engine._run_rollout(
            role=role,
            case_id=case_id,
            output_dir=output_dir,
        )


def _publish_baseline_and_decision() -> tuple[dict[str, Any], dict[str, Any]]:
    _require_baseline_complete()
    case_metrics: dict[str, Any] = {}
    artifacts: dict[str, Any] = {}
    for case_id in contract.CASE_IDS:
        root = REPO_ROOT / contract.rollout_destination("baseline", case_id)
        summary_path = root / "summary.json"
        case_metrics[case_id] = gates.baseline_case_metrics(
            v4._strict_mapping(summary_path)
        )
        artifacts[case_id] = {
            "summary": _record(summary_path),
            "receipt": _record(root / "receipt.json"),
            "solver_audit_journal": _record(root / "solver_audit_journal.json"),
        }
    manifest = {
        "schema_version": 4,
        "status": "H0_PRIMARY_SPLIT_V4_QUALIFICATION_BASELINE_EVIDENCE_FROZEN",
        "baseline_id": "analog_h0_primary_physics_condition_matched_v1",
        "cases": artifacts,
        "qualification_lock": _record(LOCK),
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
    }
    v4._write_json_exclusive(REPO_ROOT / contract.BASELINE_MANIFEST_PATH, manifest)
    baseline = {
        "schema_version": 4,
        "status": "H0_PRIMARY_SPLIT_V4_QUALIFICATION_BASELINE_FROZEN",
        "passed": True,
        "baseline_id": manifest["baseline_id"],
        "case_metrics": case_metrics,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
    }
    baseline_path = v4._write_json_exclusive(
        REPO_ROOT / contract.BASELINE_RECEIPT_PATH,
        baseline,
    )
    decision = {
        "schema_version": 4,
        "status": "H0_PRIMARY_SPLIT_V4_QUALIFICATION_BASELINE_TOLERANCE_DECIDED",
        "passed": True,
        "protocol_id": contract.SOURCE_PROTOCOL_ID,
        "decision_authority": "EXPLICIT_USER_DECISION",
        "candidate_holdout_receipt": _record(
            REPO_ROOT / contract.HOLDOUT_RECEIPT_PATH
        ),
        "baseline": {
            "baseline_id": baseline["baseline_id"],
            "comparison_scope": "condition_matched_six_cases",
            "receipt": _record(baseline_path),
        },
        "tolerances": {
            "comparison_formula": (
                "candidate <= baseline + max(absolute_tolerance, "
                "relative_tolerance * abs(baseline))"
            ),
            **contract.tolerance_rows(),
        },
        "fixed_gates": {
            "expected_steps": contract.EXPECTED_STEPS,
            "minimum_valid_cycles": contract.MINIMUM_VALID_CYCLES,
            "penetration_limit_m": contract.PENETRATION_LIMIT_M,
            "penetration_comparison": "strict_less_than",
            "zero_count_fields": list(contract.ZERO_REQUIRED_COUNTS),
            "nonincreasing_count_fields": list(contract.NONINCREASING_COUNTS),
        },
        "runtime_contract": {
            "event_contract_id": contract.EVENT_CONTRACT_ID,
            "phase_fsm_input_mode": contract.PHASE_FSM_INPUT_MODE,
            "morphology_weight": contract.MORPHOLOGY_WEIGHT,
        },
        "canonical_case_ids": list(contract.CASE_IDS),
        "authority": dict(contract.AUTHORITY),
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
    }
    v4._write_json_exclusive(REPO_ROOT / contract.DECISION_RECEIPT_PATH, decision)
    _validate_baseline_decision()
    return baseline, decision


def _worker_command(role: str, case_id: str) -> list[str]:
    return [
        sys.executable,
        str(Path(__file__).resolve()),
        "--worker",
        "--role",
        role,
        "--case",
        case_id,
        "--output-dir",
        str(REPO_ROOT / contract.rollout_destination(role, case_id)),
    ]


def _run_worker_process(role: str, case_id: str) -> None:
    completed = subprocess.run(
        _worker_command(role, case_id),
        cwd=REPO_ROOT,
        timeout=WORKER_TIMEOUT_S,
        check=False,
    )
    if completed.returncode != 0:
        raise QualificationExecutionError(
            f"worker {role}/{case_id} exited {completed.returncode}"
        )


def execute() -> dict[str, Any]:
    verify_lock()
    started = time.time()
    status = contract.FAIL_STATUS
    error = None
    passed = False
    case_gates: dict[str, Any] = {}
    try:
        for case_id in contract.CASE_IDS:
            _run_worker_process("baseline", case_id)
        _publish_baseline_and_decision()
        for case_id in contract.CASE_IDS:
            _run_worker_process("candidate", case_id)
            baseline = v4._strict_mapping(
                REPO_ROOT
                / contract.rollout_destination("baseline", case_id)
                / "summary.json"
            )
            candidate = v4._strict_mapping(
                REPO_ROOT
                / contract.rollout_destination("candidate", case_id)
                / "summary.json"
            )
            gate = gates.condition_matched_gate(
                baseline,
                candidate,
                case_id=case_id,
            )
            v4._write_json_exclusive(
                REPO_ROOT / contract.gate_destination(case_id),
                gate,
            )
            case_gates[case_id] = gate
            if gate["passed"] is not True:
                raise QualificationExecutionError(
                    f"candidate case failed: {case_id}"
                )
        status = contract.PASS_STATUS
        passed = True
    except Exception as exc:
        error = f"{type(exc).__name__}: {exc}"
    ledger = {
        "schema_version": 4,
        "status": status,
        "passed": passed,
        "error": error,
        "started_unix_s": started,
        "completed_unix_s": time.time(),
        "qualification_lock": _record(LOCK),
        "v4_execution_ledger": _record(
            REPO_ROOT / contract.V4_EXECUTION_LEDGER_PATH
        ),
        "v4_holdout_receipt": _record(REPO_ROOT / contract.HOLDOUT_RECEIPT_PATH),
        "case_gates": case_gates,
        "baseline_rollouts_completed": sum(
            int(
                (
                    REPO_ROOT
                    / contract.rollout_destination("baseline", case_id)
                    / "receipt.json"
                ).is_file()
            )
            for case_id in contract.CASE_IDS
        ),
        "candidate_rollouts_completed": sum(
            int(
                (
                    REPO_ROOT
                    / contract.rollout_destination("candidate", case_id)
                    / "receipt.json"
                ).is_file()
            )
            for case_id in contract.CASE_IDS
        ),
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "next_stage": (
            "TRAINER_ZERO_UPDATE_PORT"
            if passed
            else "STOP_WITHOUT_RETRY_OR_RETUNING"
        ),
    }
    v4._write_json_exclusive(REPO_ROOT / contract.EXECUTION_LEDGER_PATH, ledger)
    print(json.dumps(ledger, indent=2, sort_keys=True, allow_nan=False), flush=True)
    if not passed:
        raise QualificationExecutionError(error or status)
    return ledger


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--worker", action="store_true")
    mode.add_argument("--execute", action="store_true")
    parser.add_argument("--role", choices=("baseline", "candidate"))
    parser.add_argument("--case", choices=contract.CASE_IDS)
    parser.add_argument("--output-dir")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        if args.execute:
            result = execute()
        else:
            if args.role is None or args.case is None or args.output_dir is None:
                raise QualificationExecutionError(
                    "worker role/case/output-dir are required"
                )
            result = _run_rollout(
                role=args.role,
                case_id=args.case,
                output_dir=args.output_dir,
            )
    except Exception as exc:
        if args.worker and args.output_dir:
            failure = Path(args.output_dir).expanduser().resolve() / "failure.json"
            if not failure.exists():
                try:
                    v4._write_json_exclusive(
                        failure,
                        {
                            "schema_version": 4,
                            "status": "FAIL_CLOSED",
                            "error": f"{type(exc).__name__}: {exc}",
                            "traceback": traceback.format_exc(),
                            "actor_updates": 0,
                            "critic_updates": 0,
                            "ppo_updates": 0,
                            "protected_trials_opened": [],
                        },
                    )
                except Exception:
                    pass
        print(
            f"H0 V4 qualification failed closed: {type(exc).__name__}: {exc}",
            file=sys.stderr,
        )
        return 2
    print(json.dumps(result, indent=2, sort_keys=True, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
