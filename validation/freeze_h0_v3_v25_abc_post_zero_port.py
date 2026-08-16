"""Freeze the separate post-zero-port V3/V25 A/B/C execution branch."""

from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Any, Mapping

from validation import h0_v3_v25_abc_post_zero_port_contract as contract
from validation import run_h0_v3_v25_abc_post_zero_port as driver


class PostZeroPortFreezeError(RuntimeError):
    """Raised before publication when a prerequisite is missing or stale."""


def _strict(path: Path) -> dict[str, Any]:
    try:
        return driver._strict_mapping(path)
    except Exception as exc:
        raise PostZeroPortFreezeError(str(exc)) from exc


def _zero_footer(value: Mapping[str, Any], label: str) -> None:
    if (
        value.get("actor_updates") != 0
        or value.get("critic_updates") != 0
        or value.get("ppo_updates") != 0
        or value.get("protected_trials_opened") != []
    ):
        raise PostZeroPortFreezeError(f"{label} update/protected footer drifted")


def _validate_v3_and_qualification() -> None:
    lock = _strict(driver.INPUT_PATHS["v3_execution_lock"])
    if (
        lock.get("status") != "H0_PRIMARY_GRF_SPLIT_V3_EXECUTION_FROZEN"
        or lock.get("so_policy_id") != contract.SO_POLICY_ID
        or lock.get("protected_trials_opened") != []
    ):
        raise PostZeroPortFreezeError("V3 execution lock is ineligible")
    qualification = _strict(driver.INPUT_PATHS["v3_qualification_ledger"])
    if (
        qualification.get("status")
        != "PASS_H0_PRIMARY_SPLIT_V3_AUTONOMOUS_QUALIFICATION"
        or qualification.get("passed") is not True
        or qualification.get("error") is not None
        or not isinstance(qualification.get("case_gates"), Mapping)
        or set(qualification["case_gates"])
        != {item["case_id"] for item in contract.CASES}
        or qualification.get("next_stage") != "TRAINER_ZERO_UPDATE_PORT"
    ):
        raise PostZeroPortFreezeError("six-case V3 qualification is not a PASS")
    _zero_footer(qualification, "V3 qualification")


def _validate_zero_port() -> None:
    lock = _strict(driver.INPUT_PATHS["zero_port_lock"])
    if (
        lock.get("status") != "H0_PRIMARY_SPLIT_V3_ZERO_UPDATE_PORT_FROZEN"
        or lock.get("so_policy_id") != contract.SO_POLICY_ID
        or lock.get("protected_trials_opened") != []
    ):
        raise PostZeroPortFreezeError("zero-port lock is ineligible")
    receipt = _strict(driver.INPUT_PATHS["zero_port_receipt"])
    if (
        receipt.get("status") != "PASS_H0_PRIMARY_SPLIT_V3_ZERO_UPDATE_PORT"
        or receipt.get("passed") is not True
        or receipt.get("environment_samples") != 0
    ):
        raise PostZeroPortFreezeError("zero-port receipt is not a PASS")
    _zero_footer(receipt, "zero-port receipt")
    ledger = _strict(driver.INPUT_PATHS["zero_port_ledger"])
    if (
        ledger.get("status") != receipt.get("status")
        or ledger.get("passed") is not True
        or ledger.get("error") is not None
        or ledger.get("next_stage") != "V25_ABC_PREFLIGHT"
        or ledger.get("environment_samples") != 0
    ):
        raise PostZeroPortFreezeError("zero-port ledger is not terminal PASS")
    _zero_footer(ledger, "zero-port ledger")
    restored = receipt.get("restored_export")
    if not isinstance(restored, Mapping) or restored.get("root") != driver.repo_relative(
        driver.MODULE_DIR
    ):
        raise PostZeroPortFreezeError("zero-port restored export identity drifted")
    files = restored.get("files")
    if not isinstance(files, Mapping):
        raise PostZeroPortFreezeError("zero-port restored export tree is missing")
    for relative, key in (
        ("module_state.pkl", "zero_port_module_state"),
        ("class_and_ctor_args.pkl", "zero_port_module_ctor"),
        ("metadata.json", "zero_port_module_metadata"),
        ("actor_feature_manifest.json", "zero_port_actor_manifest"),
    ):
        try:
            driver._verified_record(
                files[relative], driver.INPUT_PATHS[key], f"restored_export.{relative}"
            )
        except (KeyError, driver.PostZeroPortExecutionError) as exc:
            raise PostZeroPortFreezeError(
                f"zero-port restored export file drifted: {relative}"
            ) from exc


def _validate_v25() -> None:
    candidate = _strict(driver.INPUT_PATHS["v25_candidate_freeze"])
    profile_record = candidate.get("candidate", {}).get("profile")
    if (
        candidate.get("status")
        != "V25_DEVELOPMENT_CANDIDATE_FROZEN_H0_PROTOCOL_REQUIRED"
        or candidate.get("pass") is not True
        or candidate.get("lifecycle", {}).get("global_development_candidate_frozen")
        is not True
        or candidate.get("lifecycle", {}).get("candidate_reselection_or_retuning_allowed")
        is not False
    ):
        raise PostZeroPortFreezeError("V25 candidate freeze is ineligible")
    try:
        driver._verified_record(
            profile_record, driver.INPUT_PATHS["v25_profile"], "V25 profile"
        )
    except driver.PostZeroPortExecutionError as exc:
        raise PostZeroPortFreezeError(str(exc)) from exc
    readiness = _strict(driver.INPUT_PATHS["v25_shadow_readiness"])
    if (
        readiness.get("status")
        != "V25_SHADOW_STRUCTURALLY_READY_NUMERICAL_AB_UNRUN"
        or readiness.get("structural_readiness") is not True
        or readiness.get("candidate", {}).get("candidate_id")
        != candidate.get("candidate", {}).get("candidate_id")
        or readiness.get("numerical_ab_pass_claimed") is not False
    ):
        raise PostZeroPortFreezeError("V25 structural readiness is ineligible")


def _validate_noise() -> None:
    manifest = _strict(driver.INPUT_PATHS["noise_manifest"])
    if (
        manifest.get("status") != "H0_PRIMARY_GRF_SPLIT_NOISE_TAPES_FROZEN"
        or manifest.get("standard_normal_pre_scaling") is not True
        or manifest.get("protected_trials_opened") != []
    ):
        raise PostZeroPortFreezeError("noise manifest is ineligible")
    expected = {
        "noise_deterministic": manifest.get("deterministic", {}).get("artifact"),
        "noise_seed126": manifest.get("tapes", {})
        .get("qualification", {})
        .get("02", {})
        .get("artifact"),
        "noise_seed127": manifest.get("tapes", {})
        .get("qualification", {})
        .get("04", {})
        .get("artifact"),
        "noise_seed128": manifest.get("tapes", {})
        .get("qualification", {})
        .get("08", {})
        .get("artifact"),
    }
    for key, record in expected.items():
        try:
            driver._verified_record(record, driver.INPUT_PATHS[key], key)
        except driver.PostZeroPortExecutionError as exc:
            raise PostZeroPortFreezeError(str(exc)) from exc


def validate_prerequisites() -> None:
    _validate_v3_and_qualification()
    _validate_zero_port()
    _validate_v25()
    _validate_noise()


def build_payload(*, validate_inputs: bool = True) -> dict[str, Any]:
    if validate_inputs:
        validate_prerequisites()
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.LOCK_STATUS,
        "protocol_id": contract.PROTOCOL_ID,
        "revision": contract.REVISION,
        "so_policy_id": contract.SO_POLICY_ID,
        "run_root": contract.RUN_ROOT_RELATIVE_PATH,
        "cases": list(contract.CASES),
        "modes": contract.MODES,
        "matrix": {
            "rollout_count": 18,
            "destinations": [
                driver.repo_relative(driver.destination(mode, item["case_id"]))
                for mode in contract.MODES
                for item in contract.CASES
            ],
        },
        "authority": contract.AUTHORITY,
        "gates": {
            "ab_shadow_bit_exact": True,
            "v25_active_events": True,
            "sea_reserve_condition_matched": True,
            "so_policy_all_5000_windows": contract.SO_POLICY_ID,
            "zero_unaccepted_or_sea_fallback": True,
            "raw_fallback_counters_preserved": True,
            "morphology_weight": contract.MORPHOLOGY_WEIGHT,
            "terminal_status": contract.PASS_STATUS,
        },
        "sources": {
            key: driver.source_record(path) for key, path in driver.SOURCE_PATHS.items()
        },
        "inputs": {
            key: driver.source_record(path) for key, path in driver.INPUT_PATHS.items()
        },
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
    }


def freeze() -> dict[str, Any]:
    if os.path.lexists(driver.LOCK):
        raise PostZeroPortFreezeError(f"refusing to clobber lock: {driver.LOCK}")
    if os.path.lexists(driver.RUN_ROOT):
        raise PostZeroPortFreezeError(
            f"run root exists before freeze: {driver.RUN_ROOT}"
        )
    payload = build_payload(validate_inputs=True)
    driver.RUN_ROOT.parent.mkdir(parents=True, exist_ok=True)
    try:
        driver.comparator.write_json_exclusive(driver.LOCK, payload)
        driver.verify_lock(require_run_root_absent=True)
    except Exception as exc:
        raise PostZeroPortFreezeError(str(exc)) from exc
    return payload


if __name__ == "__main__":
    try:
        print(json.dumps(freeze(), indent=2, sort_keys=True, allow_nan=False))
    except Exception as exc:
        print(f"{type(exc).__name__}: {exc}")
        raise SystemExit(2) from exc
