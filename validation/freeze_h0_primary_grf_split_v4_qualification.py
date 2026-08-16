"""Freeze and preallocate one-shot post-holdout H0 V4 qualification.

The freezer cannot start a rollout.  It validates V4 holdout continuity,
candidate identity, the accepted SO policy, noise tapes, and unit-test evidence;
then it creates twelve empty rollout destinations and one no-clobber lock.
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from pathlib import Path
from typing import Any, Sequence


REPO_ROOT = Path(__file__).resolve().parents[1]
VALIDATION_ROOT = REPO_ROOT / "validation"
TRAJECTORY_ROOT = REPO_ROOT / "Trajectory Generator"
BASELINE_ROOT = TRAJECTORY_ROOT / "baseline_MLP"
for import_root in (VALIDATION_ROOT, BASELINE_ROOT, TRAJECTORY_ROOT, REPO_ROOT):
    if str(import_root) not in sys.path:
        sys.path.insert(0, str(import_root))

import h0_primary_grf_split_v4_freeze_contract as v4_contract  # noqa: E402
import h0_primary_grf_split_v4_qualification_contract as contract  # noqa: E402
import h0_primary_grf_split_v3_qualification_scaffold as noise_contract  # noqa: E402
import h0_primary_grf_split_v3_freeze_contract as so_contract  # noqa: E402
import run_h0_primary_grf_split_v4_full_mean as v4  # noqa: E402
import run_h0_primary_grf_split_v4_qualification as runner  # noqa: E402


DESTINATION = REPO_ROOT / contract.LOCK_PATH
TEST_RECEIPT = VALIDATION_ROOT / "h0_primary_grf_split_v4_qualification_test_receipt.json"
TEST_MODULES = (
    "validation.test_h0_primary_grf_split_v4_qualification_contract",
    "validation.test_compare_h0_primary_grf_split_v4_qualification",
    "validation.test_freeze_h0_primary_grf_split_v4_qualification",
    "validation.test_run_h0_primary_grf_split_v4_qualification",
)


class QualificationFreezeError(RuntimeError):
    """Raised before autonomous qualification authority can be frozen."""


def _record(path: str | Path) -> dict[str, Any]:
    return v4.source_record(Path(path).resolve())


def _source_paths() -> dict[str, Path]:
    return {
        "qualification_contract": VALIDATION_ROOT
        / "h0_primary_grf_split_v4_qualification_contract.py",
        "qualification_comparator": VALIDATION_ROOT
        / "compare_h0_primary_grf_split_v4_qualification.py",
        "qualification_runner": VALIDATION_ROOT
        / "run_h0_primary_grf_split_v4_qualification.py",
        "qualification_freezer": Path(__file__).resolve(),
        "qualification_contract_tests": VALIDATION_ROOT
        / "test_h0_primary_grf_split_v4_qualification_contract.py",
        "qualification_comparator_tests": VALIDATION_ROOT
        / "test_compare_h0_primary_grf_split_v4_qualification.py",
        "qualification_freezer_tests": VALIDATION_ROOT
        / "test_freeze_h0_primary_grf_split_v4_qualification.py",
        "qualification_runner_tests": VALIDATION_ROOT
        / "test_run_h0_primary_grf_split_v4_qualification.py",
        "v4_runner": VALIDATION_ROOT / "run_h0_primary_grf_split_v4_full_mean.py",
        "v4_freeze_contract": VALIDATION_ROOT
        / "h0_primary_grf_split_v4_freeze_contract.py",
        "v3_qualification_engine": VALIDATION_ROOT
        / "run_h0_primary_grf_split_v3_qualification.py",
        "v3_noise_contract": VALIDATION_ROOT
        / "h0_primary_grf_split_v3_qualification_scaffold.py",
        "v3_replay_engine": VALIDATION_ROOT
        / "run_h0_primary_grf_split_v3_semantic_replay.py",
        "v3_so_policy_contract": VALIDATION_ROOT
        / "h0_primary_grf_split_v3_freeze_contract.py",
        "so_classifier": VALIDATION_ROOT / "h0_v3_so_recovery_contract.py",
        "primary_split_contract": BASELINE_ROOT
        / "primary_grf_split_adaptation.py",
        "rollout_eval": BASELINE_ROOT / "rollout_eval.py",
        "environment_factory": BASELINE_ROOT / "env_factory.py",
        "environment": TRAJECTORY_ROOT / "osim_trj_cmc_like.py",
        "phase_fsm": TRAJECTORY_ROOT / "prosthetic_phase_fsm.py",
        "simulation_runner": REPO_ROOT / "simulation_runner.py",
        "static_optimization": REPO_ROOT / "static_optimization.py",
        "model_loader": REPO_ROOT / "model_loader.py",
        "online_grf": REPO_ROOT / "online_grf.py",
    }


def _input_paths() -> dict[str, Path]:
    paths = {
        "v4_execution_lock": REPO_ROOT / contract.V4_EXECUTION_LOCK_PATH,
        "v4_execution_ledger": REPO_ROOT / contract.V4_EXECUTION_LEDGER_PATH,
        "v4_holdout_receipt": REPO_ROOT / contract.HOLDOUT_RECEIPT_PATH,
        "v4_holdout_gate": REPO_ROOT / contract.HOLDOUT_GATE_PATH,
        "v4_candidate_freeze": REPO_ROOT / contract.CANDIDATE_FREEZE_PATH,
        "v4_candidate_receipt": v4.ADAPTATION_DIR / "receipt.json",
        "candidate_state": REPO_ROOT
        / contract.CANDIDATE_MODULE_PATH
        / "module_state.pkl",
        "candidate_ctor": REPO_ROOT
        / contract.CANDIDATE_MODULE_PATH
        / "class_and_ctor_args.pkl",
        "candidate_metadata": REPO_ROOT
        / contract.CANDIDATE_MODULE_PATH
        / "metadata.json",
        "candidate_actor_manifest": REPO_ROOT
        / contract.CANDIDATE_MODULE_PATH
        / "actor_feature_manifest.json",
        "source_h0_state": REPO_ROOT
        / contract.SOURCE_H0_MODULE_PATH
        / "module_state.pkl",
        "source_h0_ctor": REPO_ROOT
        / contract.SOURCE_H0_MODULE_PATH
        / "class_and_ctor_args.pkl",
        "source_h0_metadata": REPO_ROOT
        / contract.SOURCE_H0_MODULE_PATH
        / "metadata.json",
        "source_h0_config": v4.H0_CONFIG,
        "noise_manifest": REPO_ROOT / contract.NOISE_MANIFEST_PATH,
        "qualification_tests": TEST_RECEIPT,
    }
    unique_tapes = sorted({str(case["noise_tape"]) for case in contract.canonical_cases()})
    for index, relative in enumerate(unique_tapes, start=1):
        paths[f"noise_tape_{index}"] = REPO_ROOT / relative
    return paths


def run_and_record_tests() -> dict[str, Any]:
    if os.path.lexists(TEST_RECEIPT):
        raise QualificationFreezeError(f"refusing to clobber: {TEST_RECEIPT}")
    completed = subprocess.run(
        [sys.executable, "-m", "unittest", *TEST_MODULES],
        cwd=REPO_ROOT,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        timeout=300.0,
        check=False,
    )
    payload = {
        "schema_version": 4,
        "status": (
            "PASS_H0_PRIMARY_SPLIT_V4_QUALIFICATION_TESTS"
            if completed.returncode == 0
            else "FAIL_H0_PRIMARY_SPLIT_V4_QUALIFICATION_TESTS"
        ),
        "passed": completed.returncode == 0,
        "command": [sys.executable, "-m", "unittest", *TEST_MODULES],
        "returncode": completed.returncode,
        "output": completed.stdout,
        "test_modules": list(TEST_MODULES),
        "rollouts_executed": 0,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
    }
    v4._write_json_exclusive(TEST_RECEIPT, payload)
    if not payload["passed"]:
        raise QualificationFreezeError("qualification tests failed")
    return payload


def _validate_prerequisites() -> dict[str, Any]:
    post_holdout = runner.validate_v4_post_holdout()
    noise = noise_contract.validate_noise_tapes(REPO_ROOT)
    tests = v4._strict_mapping(TEST_RECEIPT)
    if (
        tests.get("schema_version") != 4
        or tests.get("status") != "PASS_H0_PRIMARY_SPLIT_V4_QUALIFICATION_TESTS"
        or tests.get("passed") is not True
        or tests.get("rollouts_executed") != 0
        or tests.get("actor_updates") != 0
        or tests.get("critic_updates") != 0
        or tests.get("ppo_updates") != 0
        or tests.get("protected_trials_opened") != []
    ):
        raise QualificationFreezeError("qualification test receipt is not authoritative")
    for name, path in {**_source_paths(), **_input_paths()}.items():
        if not path.is_file():
            raise QualificationFreezeError(
                f"required frozen artifact missing: {name}: {path}"
            )
    return {"post_holdout": post_holdout, "noise": noise}


def build_payload(*, require_unoccupied: bool = True) -> dict[str, Any]:
    if require_unoccupied:
        if os.path.lexists(DESTINATION):
            raise QualificationFreezeError(f"refusing to clobber: {DESTINATION}")
        if os.path.lexists(REPO_ROOT / contract.RUN_ROOT):
            raise QualificationFreezeError("qualification run root is already claimed")
    prerequisites = _validate_prerequisites()
    destinations = [
        contract.rollout_destination(role, case_id).as_posix()
        for role in ("baseline", "candidate")
        for case_id in contract.CASE_IDS
    ]
    candidate_root = REPO_ROOT / contract.CANDIDATE_MODULE_PATH
    payload = {
        "schema_version": 4,
        "status": contract.LOCK_STATUS,
        "protocol_id": contract.PROTOCOL_ID,
        "source_protocol_id": contract.SOURCE_PROTOCOL_ID,
        "candidate_id": contract.CANDIDATE_ID,
        "revision": contract.REVISION,
        "scope": "ZERO_UPDATE_POST_HOLDOUT_AUTONOMOUS_QUALIFICATION_ONLY",
        "so_policy_id": contract.SO_POLICY_ID,
        "so_policy": so_contract.SO_POLICIES[
            so_contract.VERIFIED_STATUS0_MAX_ITER_POLICY
        ],
        "event_contract_id": contract.EVENT_CONTRACT_ID,
        "canonical_cases": list(contract.canonical_cases()),
        "destinations": destinations,
        "execution_order": [
            "all_six_condition_matched_analog_h0_baselines",
            "baseline_receipt",
            "baseline_tolerance_decision_receipt",
            "all_six_primary_split_v4_candidate_rollouts",
        ],
        "baseline_contract": {
            "actor": "frozen_source_H0",
            "actor_input_view": "historical_analog",
            "physics": "primary_grf_split_v1",
            "event_contract_id": contract.EVENT_CONTRACT_ID,
            "condition_matching": (
                "same_offset_runtime_seed_action_seed_and_noise_tape"
            ),
        },
        "candidate_contract": {
            "actor": contract.CANDIDATE_ID,
            "actor_input_view": "primary_split",
            "trainable_scope": v4_contract.TRAINABLE_SCOPE,
            "logstd_policy": v4_contract.LOGSTD_POLICY,
            "physics": "primary_grf_split_v1",
            "event_contract_id": contract.EVENT_CONTRACT_ID,
        },
        "tolerances": {
            "formula": (
                "candidate <= baseline + max(absolute_tolerance, "
                "relative_tolerance * abs(baseline))"
            ),
            **contract.tolerance_rows(),
            "error_counts": "candidate <= exact condition-matched baseline",
        },
        "fixed_gates": {
            "steps": contract.EXPECTED_STEPS,
            "control_windows": contract.EXPECTED_CONTROL_WINDOWS,
            "minimum_valid_cycles": contract.MINIMUM_VALID_CYCLES,
            "penetration_strictly_below_m": contract.PENETRATION_LIMIT_M,
            "zero_required_counts": list(contract.ZERO_REQUIRED_COUNTS),
            "nonincreasing_counts": list(contract.NONINCREASING_COUNTS),
            "raw_solver_fallback_is_traced_not_rejected": True,
            "all_bounded_lsq_must_be_verified": True,
        },
        "post_holdout_prerequisite": prerequisites["post_holdout"],
        "authority": dict(contract.AUTHORITY),
        "sources": {name: _record(path) for name, path in _source_paths().items()},
        "inputs": {name: _record(path) for name, path in _input_paths().items()},
        "candidate_module": {
            "state": _record(candidate_root / "module_state.pkl"),
            "ctor": _record(candidate_root / "class_and_ctor_args.pkl"),
            "metadata": _record(candidate_root / "metadata.json"),
            "actor_feature_manifest": _record(
                candidate_root / "actor_feature_manifest.json"
            ),
        },
        "retry_or_retuning_allowed": False,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
    }
    v4._require_finite_json(payload, "qualification lock")
    return payload


def freeze() -> dict[str, Any]:
    payload = build_payload(require_unoccupied=True)
    run_root = REPO_ROOT / contract.RUN_ROOT
    try:
        (run_root / "baseline").mkdir(parents=True, exist_ok=False)
        (run_root / "candidate").mkdir(exist_ok=False)
        (run_root / "gates").mkdir(exist_ok=False)
        for role in ("baseline", "candidate"):
            for case_id in contract.CASE_IDS:
                (REPO_ROOT / contract.rollout_destination(role, case_id)).mkdir(
                    exist_ok=False
                )
    except Exception as exc:
        raise QualificationFreezeError(
            f"failed to preallocate qualification: {exc}"
        ) from exc
    for relative in payload["destinations"]:
        path = REPO_ROOT / relative
        if not path.is_dir() or any(path.iterdir()):
            raise QualificationFreezeError(
                f"preallocated destination is not empty: {path}"
            )
    v4._write_json_exclusive(DESTINATION, payload)
    return payload


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--record-tests", action="store_true")
    mode.add_argument("--check", action="store_true")
    mode.add_argument("--freeze", action="store_true")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        if args.record_tests:
            result = run_and_record_tests()
        elif args.check:
            result = build_payload(require_unoccupied=True)
        else:
            result = freeze()
    except Exception as exc:
        print(
            f"H0 V4 qualification freeze failed closed: {type(exc).__name__}: {exc}",
            file=sys.stderr,
        )
        return 2
    print(json.dumps(result, indent=2, sort_keys=True, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
