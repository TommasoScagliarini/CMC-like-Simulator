"""Run and receipt the frozen primary-split implementation test suite."""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
VALIDATION_ROOT = REPO_ROOT / "validation"
BASELINE_ROOT = REPO_ROOT / "Trajectory Generator" / "baseline_MLP"
for root in (VALIDATION_ROOT, BASELINE_ROOT):
    if str(root) not in sys.path:
        sys.path.insert(0, str(root))

import compare_h0_v25_abc as strict_io  # noqa: E402
import freeze_h0_primary_grf_split_v1_execution as collection_freezer  # noqa: E402
from run_h0_primary_grf_split_v1_adaptation import source_record  # noqa: E402


OUTPUT = VALIDATION_ROOT / "h0_primary_grf_split_v1_preflight_test_receipt_v2.json"
TEST_MODULES = (
    "validation.test_primary_grf_split_adaptation",
    "validation.test_target_domain_imitation_split",
    "validation.test_target_domain_imitation",
    "validation.test_run_h0_primary_grf_split_v1_adaptation",
    "validation.test_compare_h0_primary_grf_split",
    "validation.test_h0_v25_abc_preflight",
)
SCRIPT_TESTS = (
    VALIDATION_ROOT / "test_phase_fsm_prescribed_env.py",
    VALIDATION_ROOT / "test_prosthetic_phase_fsm_two_sensor.py",
)
SOURCE_PATHS = {
    "preflight_runner": Path(__file__).resolve(),
    "pure_contract": BASELINE_ROOT / "primary_grf_split_adaptation.py",
    "adapt_actor": BASELINE_ROOT / "target_domain_imitation.py",
    "driver": VALIDATION_ROOT / "run_h0_primary_grf_split_v1_adaptation.py",
    "h0_runtime_adapter": VALIDATION_ROOT / "run_h0_v25_abc_preflight.py",
    "comparator": VALIDATION_ROOT / "compare_h0_primary_grf_split.py",
    "input_preparer": VALIDATION_ROOT / "prepare_h0_primary_grf_split_inputs.py",
    "noise_preparer": VALIDATION_ROOT / "prepare_h0_primary_grf_split_noise_tapes.py",
    "collection_lock_freezer": VALIDATION_ROOT / "freeze_h0_primary_grf_split_v1_execution.py",
    "actor_lock_freezer": VALIDATION_ROOT / "freeze_h0_primary_grf_split_v1_actor_update.py",
    "qualification_lock_freezer": VALIDATION_ROOT / "freeze_h0_primary_grf_split_v1_qualification.py",
    "environment": REPO_ROOT / "Trajectory Generator" / "osim_trj_cmc_like.py",
    "phase_fsm": REPO_ROOT / "Trajectory Generator" / "prosthetic_phase_fsm.py",
    "simulation_runner": REPO_ROOT / "simulation_runner.py",
    "model_loader": REPO_ROOT / "model_loader.py",
    "online_grf": REPO_ROOT / "online_grf.py",
}
for module_name in TEST_MODULES:
    SOURCE_PATHS[f"test_{module_name.rsplit('.', 1)[-1]}"] = (
        REPO_ROOT / Path(*module_name.split(".")).with_suffix(".py")
    )
for script_path in SCRIPT_TESTS:
    SOURCE_PATHS[f"script_{script_path.stem}"] = script_path
for source_name, source_path in collection_freezer.SOURCE_PATHS.items():
    SOURCE_PATHS[f"collection_runtime_{source_name}"] = source_path


def run() -> dict:
    commands = [
        [sys.executable, "-m", "unittest", "-v", *TEST_MODULES],
        *[[sys.executable, str(path)] for path in SCRIPT_TESTS],
    ]
    executions = []
    for command in commands:
        completed = subprocess.run(
            command,
            cwd=REPO_ROOT,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            check=False,
            timeout=300.0,
        )
        executions.append(
            {
                "command": command,
                "returncode": completed.returncode,
                "output": completed.stdout,
            }
        )
    passed = all(item["returncode"] == 0 for item in executions)
    payload = {
        "schema_version": 1,
        "status": (
            "PASS_H0_PRIMARY_GRF_SPLIT_PREFLIGHT_TESTS"
            if passed
            else "FAIL_H0_PRIMARY_GRF_SPLIT_PREFLIGHT_TESTS"
        ),
        "passed": passed,
        "commands": commands,
        "executions": executions,
        "returncode": 0 if passed else 1,
        "test_modules": list(TEST_MODULES),
        "script_tests": [str(path.relative_to(REPO_ROOT)) for path in SCRIPT_TESTS],
        "frozen_sources_at_test": {
            name: source_record(path) for name, path in SOURCE_PATHS.items()
        },
        "h0_executed": False,
        "actor_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
    }
    strict_io.write_json_exclusive(OUTPUT, payload)
    if not passed:
        raise RuntimeError("primary-split preflight tests failed")
    return payload


if __name__ == "__main__":
    print(json.dumps(run(), indent=2, sort_keys=True, allow_nan=False))
