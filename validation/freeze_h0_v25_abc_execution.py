"""Create the narrow, no-clobber execution unlock for H0/V25 A/B/C.

The historical corrected protocol remains immutable and closed.  This receipt
opens only the zero-update full-environment H0 preflight explicitly requested
by the user on 2026-08-05.  Training, PPO, H0_sep, protected/reserve trials,
detector retuning, runtime promotion, corridor work, primary-GRF changes, and
SEA-semantic changes remain forbidden.
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from pathlib import Path
from typing import Any, Mapping, Sequence


REPO_ROOT = Path(__file__).resolve().parents[1]
VALIDATION_ROOT = REPO_ROOT / "validation"
if str(VALIDATION_ROOT) not in sys.path:
    sys.path.insert(0, str(VALIDATION_ROOT))

import compare_h0_v25_abc as comparator  # noqa: E402
import run_h0_v25_abc_preflight as driver  # noqa: E402


DESTINATION = driver.EXECUTION_LOCK
LAYOUT_RECEIPT = VALIDATION_ROOT / "h0_v25_abc_layout_preflight_receipt_v3.json"
TEST_RECEIPT = VALIDATION_ROOT / "h0_v25_abc_preflight_test_receipt_v3.json"
EXPECTED_PROTOCOL_SHA256 = driver.PROTOCOL_LOCK_SHA256
EXPECTED_LAYOUT_STATUS = "PASS_H0_V25_FULL_ENVIRONMENT_LAYOUT_PREFLIGHT"
TEST_MODULES = (
    "validation.test_binary_phase_detector_v19",
    "validation.test_binary_phase_fsm_v20",
    "validation.test_binary_phase_fsm_env_v20",
    "validation.test_binary_phase_active_adapter",
    "validation.test_binary_phase_active_env",
    "validation.test_h0_v25_abc_preflight",
)
FROZEN_SOURCE_PATHS = {
    "driver": Path(driver.__file__),
    "comparator": Path(comparator.__file__),
    "environment": REPO_ROOT / "Trajectory Generator" / "osim_trj_cmc_like.py",
    "binary_active_adapter": REPO_ROOT / "Trajectory Generator" / "binary_phase_adapter.py",
    "binary_fsm_v20": REPO_ROOT / "Trajectory Generator" / "binary_phase_fsm.py",
    "actor_phase_fsm": REPO_ROOT / "Trajectory Generator" / "prosthetic_phase_fsm.py",
    "raw_v25_sampler": REPO_ROOT / "binary_phase_detector.py",
    "simulation_runner": REPO_ROOT / "simulation_runner.py",
    "static_optimization": REPO_ROOT / "static_optimization.py",
    "rollout_eval": REPO_ROOT / "Trajectory Generator" / "baseline_MLP" / "rollout_eval.py",
    "env_factory": REPO_ROOT / "Trajectory Generator" / "baseline_MLP" / "env_factory.py",
    "reward_function": REPO_ROOT / "Trajectory Generator" / "baseline_MLP" / "reward_function.py",
    "model_loader": REPO_ROOT / "model_loader.py",
    "primary_grf_source": REPO_ROOT / "online_grf.py",
}
FROZEN_INPUT_PATHS = {
    "corrected_protocol": driver.PROTOCOL_LOCK,
    "h0_module_state": driver.H0_MODULE / "module_state.pkl",
    "h0_module_constructor": driver.H0_MODULE / "class_and_ctor_args.pkl",
    "h0_module_metadata": driver.H0_MODULE / "metadata.json",
    "h0_config": driver.H0_CONFIG,
    "v25_profile": driver.V25_PROFILE,
    "analog_legacy_detector_profile": driver.ANALOG_PROFILE,
    "actor_layout_reference": driver.ACTOR_LAYOUT_REFERENCE,
    "full_layout_reference": driver.FULL_LAYOUT_REFERENCE,
}
EXPECTED_INPUT_HASHES = {
    "corrected_protocol": EXPECTED_PROTOCOL_SHA256,
    "h0_module_state": "44457ca5df7fa0e0e1f1d361d940136917fe8f71e984a1b0afaefb8ca3ced33b",
    "h0_module_constructor": "5c98f006d99a71a0f1ddcbb31d8d73fe0a6dade8401e679f6af5b1bc943b4228",
    "h0_module_metadata": "3a032ba54abcee8c9bcbb39e72fa05566912e94461d01f3c6228dc60e088bf12",
    "h0_config": "6904f7a9000b63b5c1aab661ebcab4974dffdd1cfb8c731df6a953fc9234229e",
    "v25_profile": "db704e502b99e49bea6d89493812bafdac748f8ce8d3ce28214ff624078539a2",
    "analog_legacy_detector_profile": "61ea948a3c0613e5c0e684a3197de118c7116e36188fca6993da79ce713fd99e",
    "actor_layout_reference": "6ffc0a4cee96438a529ab398f4730d95cde7a6247e786914ac36ef8ccdd2e4e7",
    "full_layout_reference": "70676e7d2b4e8c1bca185fe23d1d29f96fa83ac34167ba916250eff6e96f601e",
}


class H0V25FreezeError(RuntimeError):
    """Raised when the execution receipt cannot be frozen exactly."""


def _strict_mapping(path: Path) -> dict[str, Any]:
    value = comparator.strict_json_load(path)
    if not isinstance(value, Mapping):
        raise H0V25FreezeError(f"expected JSON object: {path}")
    return dict(value)


def _source_records(paths: Mapping[str, Path]) -> dict[str, Any]:
    return {name: driver.source_record(path) for name, path in paths.items()}


def run_and_record_tests() -> dict[str, Any]:
    if os.path.lexists(TEST_RECEIPT):
        raise H0V25FreezeError(f"refusing to clobber: {TEST_RECEIPT}")
    command = [sys.executable, "-m", "unittest", *TEST_MODULES]
    completed = subprocess.run(
        command,
        cwd=REPO_ROOT,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=False,
        timeout=300.0,
    )
    payload = {
        "schema_version": 1,
        "status": "PASS_H0_V25_PREFLIGHT_TESTS" if completed.returncode == 0 else "FAIL_H0_V25_PREFLIGHT_TESTS",
        "passed": completed.returncode == 0,
        "command": command,
        "returncode": completed.returncode,
        "output": completed.stdout,
        "test_modules": list(TEST_MODULES),
        "frozen_sources_at_test": _source_records(FROZEN_SOURCE_PATHS),
        "h0_executed": False,
        "ppo_updates": 0,
        "protected_trials_opened": [],
    }
    comparator.write_json_exclusive(TEST_RECEIPT, payload)
    if not payload["passed"]:
        raise H0V25FreezeError("preflight tests failed")
    return payload


def _validate_prerequisites() -> tuple[dict[str, Any], dict[str, Any]]:
    if driver.sha256_file(driver.PROTOCOL_LOCK) != EXPECTED_PROTOCOL_SHA256:
        raise H0V25FreezeError("corrected protocol lock drifted")
    protocol = _strict_mapping(driver.PROTOCOL_LOCK)
    if protocol.get("protocol_executed") is not False:
        raise H0V25FreezeError("corrected protocol unexpectedly claims execution")
    layout = _strict_mapping(LAYOUT_RECEIPT)
    if layout.get("status") != EXPECTED_LAYOUT_STATUS:
        raise H0V25FreezeError("full-environment layout preflight did not pass")
    if layout.get("h0_executed") is not False or layout.get("ppo_updates") != 0:
        raise H0V25FreezeError("layout receipt exceeded its authority")
    tests = _strict_mapping(TEST_RECEIPT)
    if tests.get("status") != "PASS_H0_V25_PREFLIGHT_TESTS" or tests.get("passed") is not True:
        raise H0V25FreezeError("preflight test receipt did not pass")
    current_sources = _source_records(FROZEN_SOURCE_PATHS)
    if tests.get("frozen_sources_at_test") != current_sources:
        raise H0V25FreezeError("runtime sources changed after the test receipt")
    for name, path in FROZEN_INPUT_PATHS.items():
        observed = driver.sha256_file(path)
        if observed != EXPECTED_INPUT_HASHES[name]:
            raise H0V25FreezeError(
                f"frozen input {name} drifted: {observed}"
            )
    return layout, tests


def _destinations() -> list[Path]:
    return [
        driver.H0_RUN_ROOT / f"{case_id}_{condition['id']}"
        for case_id in ("A", "B", "C")
        for condition in driver.CONDITIONS
    ]


def build_payload(*, require_unoccupied: bool = True) -> dict[str, Any]:
    if require_unoccupied:
        if os.path.lexists(DESTINATION):
            raise H0V25FreezeError(f"refusing to clobber: {DESTINATION}")
        if os.path.lexists(driver.H0_RUN_ROOT):
            raise H0V25FreezeError(
                f"run root must not exist before freeze: {driver.H0_RUN_ROOT}"
            )
    layout, tests = _validate_prerequisites()
    destinations = _destinations()
    if len(destinations) != 18 or len(set(destinations)) != 18:
        raise H0V25FreezeError("the rollout matrix does not resolve to 18 destinations")
    payload = {
        "schema_version": 1,
        "date": "2026-08-05",
        "status": "H0_V25_ABC_EXECUTION_UNLOCKED_FOR_PREFLIGHT",
        "scope": "ZERO_UPDATE_H0_FULL_ENVIRONMENT_PREFLIGHT_ONLY",
        "authorization_basis": {
            "user_instruction": "fallo",
            "request_context": "execute full-environment H0 layout 35/84 and A/B/C preflight",
            "date": "2026-08-05",
        },
        "supersedes_only_execution_readiness_of": driver.source_record(
            driver.PROTOCOL_LOCK
        ),
        "historical_protocol_rewritten": False,
        "authority": {
            "h0_preflight_execution_authorized": True,
            "ppo_updates_authorized": False,
            "training_authorized": False,
            "h0_sep_authorized": False,
            "protected_trial_access_authorized": False,
            "reserve_trial_access_authorized": False,
            "corridor_authorized": False,
            "runtime_promotion_authorized": False,
            "primary_grf_modification_authorized": False,
            "sea_semantic_modification_authorized": False,
            "detector_retuning_authorized": False,
        },
        "frozen_sources": _source_records(FROZEN_SOURCE_PATHS),
        "frozen_inputs": _source_records(FROZEN_INPUT_PATHS),
        "preflight_receipts": {
            "layout": driver.source_record(LAYOUT_RECEIPT),
            "tests": driver.source_record(TEST_RECEIPT),
            "superseded_drafts": [
                "validation/h0_v25_abc_layout_preflight_receipt.json",
                "validation/h0_v25_abc_preflight_test_receipt.json",
                "validation/h0_v25_abc_layout_preflight_receipt_v2.json",
                "validation/h0_v25_abc_preflight_test_receipt_v2.json",
            ],
            "supersession_reason": (
                "worker entry point now independently verifies the execution "
                "lock, every frozen source/input, and its exact declared "
                "destination before any H0 step"
            ),
        },
        "matrix": {
            "conditions": list(driver.CONDITIONS),
            "case_order": ["A", "B", "C"],
            "execution_order": "six indivisible A/B pairs, then and only then six C",
            "protocol_unit_count": 12,
            "rollout_count": 18,
            "steps_per_rollout": 500,
            "episode_duration_s": 5.0,
            "destinations": [
                str(path.relative_to(REPO_ROOT)) for path in destinations
            ],
        },
        "replay_contract": {
            "selected_mode": "standard_normal_innovation_replay",
            "dtype": "float32",
            "shape_per_step": [2],
            "steps": 500,
            "sigma_expected": 0.005,
            "B_recomputes_mean_std_raw_and_compares_before_step": True,
            "B_action_injection_allowed": False,
            "C_reuses_only_innovations_and_remains_closed_loop": True,
            "simple_reseed_is_sufficient": False,
        },
        "trace_and_comparator_contract": {
            "rows_per_rollout": 500,
            "allow_truncation_to_shorter_trace": False,
            "only_ab_exclusion": "top-level keys with exact prefix binary_phase_",
            "v25_journal": "one raw t0 baseline plus exactly 5000 ordered 1 ms boolean samples",
            "strict_json": True,
            "finite_only": True,
            "atomic_no_clobber": True,
        },
        "sea_metric_mapping": {
            "signals": list(comparator.SEA_SIGNALS),
            "joints": list(comparator.JOINTS),
            "episode_rms": "sqrt(sum(segment_sum_squares)/sum(segment_sample_count))",
            "absolute_maximum": "max(segment_absolute_maximum)",
            "tau_spring_rate": "finite differences within each 10 ms segment; explicit rate sample_count excludes cross-segment gap",
            "saturation_count": "sum exact substep Boolean counts",
            "saturation_fraction": "episode count / identical finite episode denominator",
            "reserve_residual": "RMS and absolute maximum over exactly 500 policy-step norms",
            "numerical_cap": "C <= A + max(1e-6, 1e-9 * abs(A))",
            "saturation_cap": "C <= A without tolerance",
            "plugin_and_SO_fallbacks_instrumented": True,
        },
        "timeouts": {
            "environment_step_wall_timeout_s": 60.0,
            "worker_process_timeout_s": driver.WORKER_TIMEOUT_S,
            "retry_allowed": False,
        },
        "terminal_statuses": {
            "A_failure": "ERROR_H0_REFERENCE",
            "AB_mismatch": "ERROR_SHADOW_NONINTERFERENCE",
            "C_failure": "FAIL_H0_V25_COMPATIBILITY",
            "complete_pass": "PASS_H0_V25_COMPATIBLE",
        },
        "claims": {
            "primary_grf_calibration_reopened": False,
            "sea_semantics_changed": False,
            "h0_adapted_to_v25": False,
            "ppo_or_training_performed": False,
            "protected_data_opened": False,
            "runtime_promoted": False,
        },
        "freeze_script": driver.source_record(Path(__file__)),
    }
    comparator.canonical_json_bytes(payload)
    return payload


def freeze() -> dict[str, Any]:
    payload = build_payload(require_unoccupied=True)
    driver.H0_RUN_ROOT.parent.mkdir(parents=True, exist_ok=True)
    try:
        driver.H0_RUN_ROOT.mkdir()
        for destination in _destinations():
            destination.mkdir()
        (driver.H0_RUN_ROOT / "gates").mkdir()
    except Exception as exc:
        raise H0V25FreezeError(
            f"failed while allocating the one-shot destinations: {exc}"
        ) from exc
    for destination in _destinations():
        if not destination.is_dir() or any(destination.iterdir()):
            raise H0V25FreezeError(f"destination is not empty: {destination}")
    comparator.write_json_exclusive(DESTINATION, payload)
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
            f"H0/V25 execution freeze failed closed: {type(exc).__name__}: {exc}",
            file=sys.stderr,
        )
        return 2
    print(json.dumps(result, indent=2, sort_keys=True, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
