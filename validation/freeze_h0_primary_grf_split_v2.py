"""Freeze the no-clobber H0 primary-split V2 prescribed-teacher execution."""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path
from typing import Any, Mapping


REPO_ROOT = Path(__file__).resolve().parents[1]
VALIDATION_ROOT = REPO_ROOT / "validation"
for path in (
    VALIDATION_ROOT,
    REPO_ROOT / "Trajectory Generator" / "baseline_MLP",
    REPO_ROOT / "Trajectory Generator",
    REPO_ROOT,
):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

import compare_h0_primary_grf_split_v2 as gates  # noqa: E402
import run_h0_primary_grf_split_v2_adaptation as runner  # noqa: E402


OLD_LOCK = VALIDATION_ROOT / "h0_primary_grf_split_v1_collection_execution_unlock.json"
OLD_LEDGER = (
    VALIDATION_ROOT
    / "h0_primary_grf_split_adaptation_runs"
    / "2026-08-05_h0_primary_grf_split_v1_one_shot"
    / "collection_execution_ledger.json"
)
PREFLIGHT_RECEIPT = VALIDATION_ROOT / "h0_primary_grf_split_v2_preflight_receipt.json"
LAYOUT_RECEIPT = (
    VALIDATION_ROOT / "h0_primary_grf_split_v2_layout_preflight_receipt.json"
)
PLAN = (
    REPO_ROOT
    / "reports"
    / "plans"
    / "2026-08-06_protocollo_h0_primary_split_v2_prescribed_teacher.md"
)


def _strict_mapping(path: Path) -> dict[str, Any]:
    value = gates.strict_json_load(path)
    if not isinstance(value, Mapping):
        raise RuntimeError(f"expected JSON object: {path}")
    return dict(value)


def _record(path: Path) -> dict[str, Any]:
    return runner.source_record(path)


def _verify_old_terminal_branch() -> tuple[dict[str, Any], dict[str, Any]]:
    old_lock = _strict_mapping(OLD_LOCK)
    old_ledger = _strict_mapping(OLD_LEDGER)
    if (
        old_lock.get("status")
        != "H0_PRIMARY_GRF_SPLIT_V1_COLLECTION_EXECUTION_UNLOCKED"
    ):
        raise RuntimeError("old V1 collection lock is not authoritative")
    if (
        old_ledger.get("status") != "ERROR_PRIMARY_SPLIT_TEACHER"
        or old_ledger.get("passed") is not False
        or old_ledger.get("next_stage") != "STOP_WITHOUT_RETRY_OR_RETUNING"
        or old_ledger.get("actor_updates") != 0
        or old_ledger.get("ppo_updates") != 0
        or old_ledger.get("protected_trials_opened") != []
    ):
        raise RuntimeError("old V1 branch is not the expected terminal failure")
    verified = runner._verify_record_tree(old_lock.get("frozen_sources"), "old.sources")
    verified += runner._verify_record_tree(old_lock.get("frozen_inputs"), "old.inputs")
    if verified < 80:
        raise RuntimeError(f"old frozen closure is incomplete: {verified}")
    return old_lock, old_ledger


def _run_preflight_tests() -> dict[str, Any]:
    commands = [
        [
            sys.executable,
            "-m",
            "unittest",
            "validation.test_h0_primary_grf_split_v2",
            "validation.test_primary_grf_split_adaptation",
            "validation.test_target_domain_imitation_split",
        ],
        [
            sys.executable,
            "-m",
            "py_compile",
            str(runner.__file__),
            str(Path(gates.__file__).resolve()),
            str(Path(__file__).resolve()),
        ],
    ]
    results = []
    for command in commands:
        completed = subprocess.run(
            command,
            cwd=REPO_ROOT,
            text=True,
            capture_output=True,
            timeout=300.0,
            check=False,
        )
        results.append(
            {
                "command": command,
                "returncode": completed.returncode,
                "stdout_tail": completed.stdout[-2000:],
                "stderr_tail": completed.stderr[-2000:],
            }
        )
        if completed.returncode != 0:
            raise RuntimeError(
                f"preflight command failed ({completed.returncode}): {command}"
            )
    payload = {
        "schema_version": 2,
        "status": "PASS_H0_PRIMARY_GRF_SPLIT_V2_PREFLIGHT",
        "passed": True,
        "results": results,
        "old_v1_reopened": False,
        "h0_executed": False,
        "actor_updates": 0,
        "ppo_updates": 0,
        "critic_updates": 0,
        "protected_trials_opened": [],
    }
    gates.write_json_exclusive(PREFLIGHT_RECEIPT, payload)
    return payload


def _destinations() -> list[Path]:
    values = [
        runner.RUN_ROOT / "collection" / f"trial_{trial_id}"
        for trial_id in runner.TRIAL_IDS
    ]
    values.extend(
        [
            runner.RUN_ROOT / "corpus",
            runner.RUN_ROOT / "adaptation",
            runner.RUN_ROOT / "qualification" / "gates",
        ]
    )
    for role in ("reference", "candidate"):
        for trial_id in runner.TRIAL_IDS:
            for selection in runner.SELECTIONS:
                values.append(
                    runner.RUN_ROOT
                    / "qualification"
                    / role
                    / f"trial_{trial_id}_{selection}"
                )
    return values


def freeze() -> dict[str, Any]:
    if (
        runner.LOCK.exists()
        or PREFLIGHT_RECEIPT.exists()
        or LAYOUT_RECEIPT.exists()
        or runner.RUN_ROOT.exists()
    ):
        raise FileExistsError("refusing to clobber an existing V2 lock/run/receipt")
    old_lock, _old_ledger = _verify_old_terminal_branch()
    runner.layout_preflight(LAYOUT_RECEIPT)
    _run_preflight_tests()
    destinations = _destinations()
    for destination in destinations:
        destination.mkdir(parents=True, exist_ok=False)
    v2_sources = {
        "runner": _record(Path(runner.__file__).resolve()),
        "comparator": _record(Path(gates.__file__).resolve()),
        "freezer": _record(Path(__file__).resolve()),
        "test": _record(VALIDATION_ROOT / "test_h0_primary_grf_split_v2.py"),
        "plan": _record(PLAN),
        "prescribed_teacher_and_fit": _record(
            REPO_ROOT
            / "Trajectory Generator"
            / "baseline_MLP"
            / "target_domain_imitation.py"
        ),
    }
    v2_inputs = {
        "old_terminal_ledger": _record(OLD_LEDGER),
        "old_v1_lock": _record(OLD_LOCK),
        "preflight_receipt": _record(PREFLIGHT_RECEIPT),
        "layout_preflight_receipt": _record(LAYOUT_RECEIPT),
    }
    lock = {
        "schema_version": 2,
        "status": "H0_PRIMARY_GRF_SPLIT_V2_EXECUTION_FROZEN",
        "revision": "2026-08-06",
        "run_root": str(runner.RUN_ROOT.relative_to(REPO_ROOT)),
        "target_contract": runner.EVENT_CONTRACT,
        "candidate_id": "H0_primary_split_v2",
        "teacher": {
            "kind": "same_trial_prescribed_prosthetic_ik",
            "lookahead_s": 0.0,
            "runtime_available_to_candidate": False,
            "h0_used_for_behavior": False,
            "h0_used_for_initialization_and_anchor_only": True,
        },
        "collection": {
            trial_id: {
                "trial_id": trial_id,
                "offset_s": float(v1_value["collection_offset_s"]),
                "absolute_start_s": float(v1_value["collection_absolute_s"]),
                "seed": int(v1_value["collection_seed"]),
                "steps": runner.EXPECTED_STEPS,
                "selection": "deterministic",
            }
            for trial_id, v1_value in runner.v1.TRIALS.items()
        },
        "qualification": {
            trial_id: {
                "trial_id": trial_id,
                "offset_s": float(v1_value["qualification_offset_s"]),
                "absolute_start_s": float(v1_value["qualification_absolute_s"]),
                "seed": int(v1_value["qualification_seed"]),
                "steps": runner.EXPECTED_STEPS,
                "selections": list(runner.SELECTIONS),
            }
            for trial_id, v1_value in runner.v1.TRIALS.items()
        },
        "fit": dict(runner.FIT),
        "authority": {
            "prescribed_teacher_collection_authorized": True,
            "actor_only_adaptation_authorized": True,
            "development_qualification_authorized": True,
            "zero_update_port_authorized": True,
            "ppo_updates_authorized": False,
            "critic_updates_authorized": False,
            "protected_trial_access_authorized": False,
            "primary_grf_modification_authorized": False,
            "sea_semantic_modification_authorized": False,
            "runtime_promotion_authorized": False,
        },
        "runtime_closure": old_lock["frozen_sources"],
        "input_closure": old_lock["frozen_inputs"],
        "v2_sources": v2_sources,
        "v2_inputs": v2_inputs,
        "destinations": [str(path.relative_to(REPO_ROOT)) for path in destinations],
        "old_v1_reopened": False,
        "retry_or_retuning_allowed": False,
        "actor_update_candidate_count": 1,
        "ppo_updates": 0,
        "critic_updates": 0,
        "protected_trials_opened": [],
    }
    gates.write_json_exclusive(runner.LOCK, lock)
    runner.verify_lock()
    print(json.dumps(lock, indent=2, sort_keys=True, allow_nan=False))
    return lock


if __name__ == "__main__":
    freeze()
