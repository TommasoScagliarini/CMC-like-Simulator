"""Freeze (but never execute) the V6/P1 zero-update trainer port.

The freezer is intentionally unusable until the same immutable P1 has a
development PASS and qualification PASS.  It writes one strict, no-clobber
execution lock containing hashes of every prerequisite and source file.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Mapping

try:
    from . import h0_primary_split_v6_zero_update_contract as contract
    from . import run_h0_primary_split_v6_zero_update_port as driver
except ImportError:  # Direct ``python validation/freeze_*.py`` execution.
    import h0_primary_split_v6_zero_update_contract as contract
    import run_h0_primary_split_v6_zero_update_port as driver


def _same_candidate(payloads: Mapping[str, Mapping[str, Any]]) -> str:
    ids = {name: value.get("candidate_id") for name, value in payloads.items()}
    if any(not isinstance(value, str) or not value for value in ids.values()):
        raise driver.ZeroUpdatePortError(f"prerequisite candidate_id missing: {ids}")
    if len(set(ids.values())) != 1:
        raise driver.ZeroUpdatePortError(f"prerequisites do not bind one P1: {ids}")
    return next(iter(ids.values()))


def _residual_config(candidate_freeze: Mapping[str, Any]) -> dict[str, Any]:
    architecture = candidate_freeze.get("architecture")
    model_config = candidate_freeze.get("model_config")
    if not isinstance(architecture, Mapping) or set(architecture) != {
        "residual_input_indices",
        "residual_input_count",
        "hidden_dims",
        "action_dim",
        "limits",
        "init_seed",
    }:
        raise driver.ZeroUpdatePortError("candidate architecture schema drifted")
    if not isinstance(model_config, Mapping) or set(model_config) != {
        "primary_split_v25_residual_input_mean",
        "primary_split_v25_residual_input_std",
        "primary_split_v25_residual_limits",
        "primary_split_v25_residual_init_seed",
        "primary_split_v25_residual_reset_bypass",
    }:
        raise driver.ZeroUpdatePortError("candidate model_config schema drifted")
    exact_architecture = {
        "residual_input_indices": list(contract.RESIDUAL_INPUT_INDICES),
        "residual_input_count": contract.RESIDUAL_INPUT_COUNT,
        "hidden_dims": list(contract.RESIDUAL_ARCHITECTURE[1:-1]),
        "action_dim": contract.RESIDUAL_LIMIT_COUNT,
        "limits": list(model_config["primary_split_v25_residual_limits"]),
        "init_seed": model_config["primary_split_v25_residual_init_seed"],
    }
    if driver.canonical_json(architecture) != driver.canonical_json(exact_architecture):
        raise driver.ZeroUpdatePortError("candidate architecture/model_config disagree")
    if model_config["primary_split_v25_residual_reset_bypass"] is not False:
        raise driver.ZeroUpdatePortError(
            "residual reset bypass must remain unavailable"
        )
    return driver.validate_residual_config(
        {
            "input_mean": model_config["primary_split_v25_residual_input_mean"],
            "input_std": model_config["primary_split_v25_residual_input_std"],
            "limits": model_config["primary_split_v25_residual_limits"],
            "init_seed": model_config["primary_split_v25_residual_init_seed"],
            "input_indices": architecture["residual_input_indices"],
            "architecture": [
                architecture["residual_input_count"],
                *architecture["hidden_dims"],
                architecture["action_dim"],
            ],
        }
    )


def verify_prerequisites() -> tuple[str, dict[str, Any]]:
    payloads = {
        "candidate_freeze": driver.strict_json(driver.INPUT_PATHS["candidate_freeze"]),
        "development": driver.strict_json(driver.INPUT_PATHS["development_receipt"]),
        "qualification": driver.strict_json(
            driver.INPUT_PATHS["qualification_receipt"]
        ),
        "qualification_ledger": driver.strict_json(
            driver.INPUT_PATHS["qualification_ledger"]
        ),
    }
    expected = {
        "candidate_freeze": (contract.CANDIDATE_FREEZE_STATUS, True),
        "development": (contract.DEVELOPMENT_PASS_STATUS, True),
        "qualification": (contract.QUALIFICATION_PASS_STATUS, True),
        "qualification_ledger": (contract.QUALIFICATION_PASS_STATUS, True),
    }
    for name, (status, passed) in expected.items():
        payload = payloads[name]
        if payload.get("status") != status or payload.get("passed") is not passed:
            raise driver.ZeroUpdatePortError(f"{name} is not the required PASS")
    candidate_id = _same_candidate(payloads)
    candidate_freeze = payloads["candidate_freeze"]
    if candidate_freeze.get("target_contract_id") != contract.TARGET_BUNDLE_CONTRACT_ID:
        raise driver.ZeroUpdatePortError("candidate target contract drifted")
    if candidate_freeze.get("dagger_rounds") != 1:
        raise driver.ZeroUpdatePortError("candidate is not the preregistered P1")
    if (
        payloads["qualification"].get("protocol_id")
        != contract.QUALIFICATION_PROTOCOL_ID
    ):
        raise driver.ZeroUpdatePortError("qualification protocol drifted")
    if payloads["qualification_ledger"].get("protocol_id") != (
        contract.QUALIFICATION_PROTOCOL_ID
    ):
        raise driver.ZeroUpdatePortError("qualification ledger protocol drifted")
    morphology_weight = payloads["qualification"].get("morphology_weight")
    if (
        isinstance(morphology_weight, bool)
        or not isinstance(morphology_weight, (int, float))
        or float(morphology_weight) != contract.MORPHOLOGY_WEIGHT
    ):
        raise driver.ZeroUpdatePortError("qualification morphology_weight is not zero")
    declared_tree = candidate_freeze.get("candidate_module")
    observed_tree = driver.tree_record(driver.CANDIDATE_DIR)
    if driver.canonical_json(declared_tree) != driver.canonical_json(observed_tree):
        raise driver.ZeroUpdatePortError("candidate module tree drifted after freeze")
    residual = _residual_config(candidate_freeze)
    return candidate_id, residual


def build_lock() -> dict[str, Any]:
    candidate_id, residual = verify_prerequisites()
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.LOCK_STATUS,
        "protocol_id": contract.PROTOCOL_ID,
        "source_protocol_id": contract.SOURCE_PROTOCOL_ID,
        "qualification_protocol_id": contract.QUALIFICATION_PROTOCOL_ID,
        "revision": contract.REVISION,
        "candidate_id": candidate_id,
        "so_policy_id": contract.SO_POLICY_ID,
        "target_bundle_contract_id": contract.TARGET_BUNDLE_CONTRACT_ID,
        "output_root": driver.repo_relative(driver.OUTPUT_ROOT),
        "target_fixed_config": contract.TARGET_FIXED_CONFIG,
        "residual_model_config": residual,
        "authority": contract.AUTHORITY,
        "required_checks": list(contract.REQUIRED_CHECKS),
        "sources": {
            name: driver.source_record(path)
            for name, path in driver.SOURCE_PATHS.items()
        },
        "inputs": {
            name: driver.source_record(path)
            for name, path in driver.INPUT_PATHS.items()
        },
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "environment_samples": 0,
        "protected_trials_opened": [],
    }


def freeze() -> dict[str, Any]:
    if Path(driver.LOCK).exists():
        raise driver.ZeroUpdatePortError(f"refusing to clobber: {driver.LOCK}")
    payload = build_lock()
    driver.write_json_exclusive(driver.LOCK, payload)
    return payload


if __name__ == "__main__":
    try:
        print(json.dumps(freeze(), indent=2, sort_keys=True, allow_nan=False))
    except Exception as exc:
        print(f"{type(exc).__name__}: {exc}")
        raise SystemExit(2) from exc
