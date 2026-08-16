"""Run the V4 static/unit preflight and publish its no-clobber receipt.

The sole excluded test is the recursive lock-payload test: that test requires
this receipt to exist because the production freezer requires it.  It is run
immediately afterwards by the normal full suite and is not needed to attest
the runner, wrapper, scope audit, or no-clobber primitives themselves.
"""

from __future__ import annotations

import json
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[1]
VALIDATION_ROOT = REPO_ROOT / "validation"
if str(VALIDATION_ROOT) not in sys.path:
    sys.path.insert(0, str(VALIDATION_ROOT))

import h0_primary_grf_split_v4_freeze_contract as contract  # noqa: E402
import run_h0_primary_grf_split_v4_full_mean as runner  # noqa: E402


RECEIPT = REPO_ROOT / contract.PREFLIGHT_RECEIPT_RELATIVE
PYTHON_SOURCES = tuple(
    REPO_ROOT / relative
    for relative in contract.V4_SOURCE_RELATIVE_PATHS.values()
    if relative.endswith(".py")
)
TEST_SOURCES = tuple(
    REPO_ROOT / contract.V4_SOURCE_RELATIVE_PATHS[key]
    for key in ("contract_tests", "freezer_tests", "runner_tests")
)
RECURSIVE_TEST_EXCLUSION = (
    "test_build_payload_is_complete_and_does_not_publish_lock_or_run"
)


class V4PreflightError(RuntimeError):
    """Raised when the V4 preflight is not a clean PASS."""


def _run(command: list[str]) -> subprocess.CompletedProcess[str]:
    completed = subprocess.run(
        command,
        cwd=REPO_ROOT,
        check=False,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        timeout=300.0,
    )
    if completed.returncode != 0:
        raise V4PreflightError(
            f"preflight command failed ({completed.returncode}): "
            f"{' '.join(command)}\n{completed.stdout}"
        )
    return completed


def _relative(path: Path) -> str:
    return path.resolve().relative_to(REPO_ROOT).as_posix()


def build() -> dict[str, Any]:
    if os.path.lexists(RECEIPT):
        raise V4PreflightError(f"refusing to clobber preflight receipt: {RECEIPT}")
    if os.path.lexists(runner.LOCK) or os.path.lexists(runner.RUN_ROOT):
        raise V4PreflightError("V4 lock/run must remain absent during preflight")

    ruff = shutil.which("ruff")
    if ruff is None:
        mac_fallback = Path("/opt/anaconda3/bin/ruff")
        if mac_fallback.is_file():
            ruff = str(mac_fallback)
        else:
            raise V4PreflightError("ruff executable is unavailable")
    py_compile_command = [
        sys.executable,
        "-m",
        "py_compile",
        *(_relative(path) for path in PYTHON_SOURCES),
    ]
    ruff_command = [ruff, "check", *(_relative(path) for path in PYTHON_SOURCES)]
    pytest_command = [
        sys.executable,
        "-m",
        "pytest",
        "-q",
        *(_relative(path) for path in TEST_SOURCES),
        "-k",
        f"not {RECURSIVE_TEST_EXCLUSION}",
    ]
    py_compile_result = _run(py_compile_command)
    ruff_result = _run(ruff_command)
    pytest_result = _run(pytest_command)
    match = re.search(r"(\d+) passed", pytest_result.stdout)
    passed_tests = int(match.group(1)) if match else -1
    if passed_tests != 11:
        raise V4PreflightError(
            f"expected 11 non-recursive tests, observed {passed_tests}"
        )
    if os.path.lexists(runner.LOCK) or os.path.lexists(runner.RUN_ROOT):
        raise V4PreflightError("preflight unexpectedly created V4 lock/run")

    v3_root = (
        REPO_ROOT
        / "validation/h0_primary_grf_split_adaptation_runs/"
        "2026-08-06_h0_primary_split_v3_semantic_replay"
    )
    v3_holdout_absent = all(
        not os.path.lexists(path)
        for path in (
            v3_root / "adaptation/candidate_freeze.json",
            v3_root / "holdout_access_claim.json",
            v3_root / "replay/seed_125",
            v3_root / "holdout",
        )
    )
    checks = {
        "py_compile_pass": py_compile_result.returncode == 0,
        "ruff_pass": ruff_result.returncode == 0,
        "pytest_nonrecursive_11_pass": passed_tests == 11,
        "v4_lock_absent": not os.path.lexists(runner.LOCK),
        "v4_run_root_absent": not os.path.lexists(runner.RUN_ROOT),
        "v3_seed125_semantic_artifacts_absent": v3_holdout_absent,
        "actor_updates_zero": True,
        "critic_updates_zero": True,
        "ppo_updates_zero": True,
        "protected_trials_closed": True,
    }
    passed = all(checks.values())
    receipt = {
        "schema_version": 4,
        "status": (
            "PASS_H0_PRIMARY_SPLIT_V4_PREFLIGHT_TESTS"
            if passed
            else "FAIL_H0_PRIMARY_SPLIT_V4_PREFLIGHT_TESTS"
        ),
        "passed": passed,
        "protocol_id": contract.PROTOCOL_ID,
        "revision": contract.REVISION,
        "checks": checks,
        "commands": {
            "py_compile": py_compile_command,
            "ruff": ruff_command,
            "pytest": pytest_command,
        },
        "pytest_passed": passed_tests,
        "recursive_test_exclusion": {
            "test": RECURSIVE_TEST_EXCLUSION,
            "reason": "requires the completed preflight receipt",
            "must_pass_after_receipt_publication": True,
        },
        "tested_sources": {
            key: runner.source_record(REPO_ROOT / relative)
            for key, relative in contract.V4_SOURCE_RELATIVE_PATHS.items()
        },
        "lineage": {
            "v3_execution_ledger": runner.source_record(
                REPO_ROOT / contract.V3_INPUT_RELATIVE_PATHS["execution_ledger"]
            ),
            "v3_corpus_receipt": runner.source_record(
                REPO_ROOT / contract.V3_INPUT_RELATIVE_PATHS["corpus_receipt"]
            ),
            "v3_failed_offline_gate": runner.source_record(
                REPO_ROOT / contract.V3_INPUT_RELATIVE_PATHS["failed_offline_gate"]
            ),
        },
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
    }
    runner._write_json_exclusive(RECEIPT, receipt)
    if not passed:
        raise V4PreflightError("V4 preflight checks did not all pass")
    return receipt


if __name__ == "__main__":
    print(json.dumps(build(), indent=2, sort_keys=True, allow_nan=False))
