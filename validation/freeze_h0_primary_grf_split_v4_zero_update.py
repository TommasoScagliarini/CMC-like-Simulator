"""Freeze the post-qualification V4 zero-update port execution.

The freezer performs no Ray/OpenSim work and creates only the immutable
zero-update lock.  It requires the V4 holdout, candidate freeze, and autonomous
six-case qualification to have passed before granting one actor transplant and
a full save/reload at zero updates.
"""

from __future__ import annotations

import json
import os
import sys
from pathlib import Path
from typing import Any, Mapping


REPO_ROOT = Path(__file__).resolve().parents[1]
VALIDATION_ROOT = REPO_ROOT / "validation"
if str(VALIDATION_ROOT) not in sys.path:
    sys.path.insert(0, str(VALIDATION_ROOT))

import h0_primary_grf_split_v4_zero_update_contract as contract  # noqa: E402
import run_h0_primary_grf_split_v4_zero_update_port as driver  # noqa: E402


LOCK = driver.LOCK
OUTPUT_ROOT = driver.OUTPUT_ROOT
INPUT_PATHS = driver.INPUT_PATHS
SOURCE_PATHS = driver.SOURCE_PATHS


class ZeroUpdateFreezeError(RuntimeError):
    """Raised when zero-update execution authority cannot be frozen safely."""


def _strict(path: Path) -> dict[str, Any]:
    try:
        return driver._strict_json(path)  # noqa: SLF001
    except driver.ZeroUpdatePortError as exc:
        raise ZeroUpdateFreezeError(str(exc)) from exc


def _exact_record(record: Any, path: Path, label: str) -> None:
    try:
        driver._verified_record(record, path, label)  # noqa: SLF001
    except driver.ZeroUpdatePortError as exc:
        raise ZeroUpdateFreezeError(str(exc)) from exc


def _zero_footer(value: Mapping[str, Any], label: str, *, actor_updates: int) -> None:
    if (
        value.get("actor_updates") != actor_updates
        or value.get("critic_updates") != 0
        or value.get("ppo_updates") != 0
        or value.get("protected_trials_opened") != []
    ):
        raise ZeroUpdateFreezeError(f"{label} update/protected footer drifted")


def _verify_record_tree(value: Any, label: str) -> None:
    if isinstance(value, Mapping) and set(value) == {
        "path",
        "sha256",
        "size_bytes",
    }:
        try:
            expected = driver._resolve_relative(str(value.get("path", "")))  # noqa: SLF001
        except RuntimeError as exc:
            raise ZeroUpdateFreezeError(str(exc)) from exc
        _exact_record(value, expected, label)
        return
    if not isinstance(value, Mapping) or not value:
        raise ZeroUpdateFreezeError(f"{label} record tree is malformed")
    for name, child in value.items():
        _verify_record_tree(child, f"{label}.{name}")


def _validate_v4_execution() -> None:
    lock = _strict(INPUT_PATHS["v4_execution_lock"])
    authority = lock.get("authority")
    if (
        lock.get("schema_version") != 4
        or lock.get("status") != "H0_PRIMARY_GRF_SPLIT_V4_EXECUTION_FROZEN"
        or lock.get("protocol_id") != contract.SOURCE_PROTOCOL_ID
        or lock.get("candidate_id") != contract.CANDIDATE_ID
        or lock.get("so_policy_id") != contract.SO_POLICY_ID
        or lock.get("event_contract_id") != contract.EVENT_CONTRACT_ID
        or lock.get("trainable_scope") != "full_mean_network"
        or lock.get("logstd_policy") != "frozen_bit_exact"
        or not isinstance(authority, Mapping)
        or authority.get("full_mean_network_update") is not True
        or authority.get("logstd_update") is not False
        or authority.get("critic_updates") is not False
        or authority.get("ppo_updates") is not False
        or authority.get("protected_trial_access") is not False
        or lock.get("protected_trials_opened") != []
    ):
        raise ZeroUpdateFreezeError("V4 execution lock is not eligible")
    _verify_record_tree(lock.get("sources"), "V4 execution sources")
    _verify_record_tree(lock.get("inputs"), "V4 execution inputs")
    ledger = _strict(INPUT_PATHS["v4_execution_ledger"])
    if (
        ledger.get("status") != "PASS_H0_PRIMARY_SPLIT_V4_FINAL_HOLDOUT"
        or ledger.get("passed") is not True
        or ledger.get("schema_version") != 4
        or ledger.get("terminal_stage") != "final_holdout_complete"
        or ledger.get("error") is not None
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
    ):
        raise ZeroUpdateFreezeError("V4 execution ledger is not a final holdout PASS")


def _validate_candidate() -> str:
    freeze = _strict(INPUT_PATHS["candidate_freeze"])
    expected_freeze_keys = {
        "schema_version",
        "status",
        "protocol_id",
        "candidate_id",
        "train_seeds",
        "final_holdout_seed",
        "fit",
        "trainable_scope",
        "logstd_policy",
        "candidate_receipt",
        "candidate_offline_gate",
        "candidate_module_state",
        "candidate_module_ctor",
        "candidate_module_metadata",
        "actor_feature_manifest",
        "candidate_actor_digest",
        "v3_terminal_ledger",
        "v3_corpus",
        "holdout_accessed_before_freeze",
        "actor_updates",
        "critic_updates",
        "ppo_updates",
        "protected_trials_opened",
    }
    if (
        set(freeze) != expected_freeze_keys
        or freeze.get("schema_version") != 4
        or freeze.get("status")
        != "H0_PRIMARY_SPLIT_V4_CANDIDATE_FROZEN_BEFORE_HOLDOUT"
        or freeze.get("protocol_id") != contract.SOURCE_PROTOCOL_ID
        or freeze.get("candidate_id") != contract.CANDIDATE_ID
        or freeze.get("train_seeds") != [123, 124]
        or freeze.get("final_holdout_seed") != 125
        or freeze.get("trainable_scope") != "full_mean_network"
        or freeze.get("logstd_policy") != "frozen_bit_exact"
        or freeze.get("holdout_accessed_before_freeze") is not False
    ):
        raise ZeroUpdateFreezeError("V4 candidate freeze is non-canonical")
    _zero_footer(freeze, "candidate freeze", actor_updates=1)
    for key, path in {
        "candidate_receipt": INPUT_PATHS["candidate_receipt"],
        "candidate_offline_gate": INPUT_PATHS["candidate_offline_gate"],
        "candidate_module_state": INPUT_PATHS["candidate_module_state"],
        "candidate_module_ctor": INPUT_PATHS["candidate_module_ctor"],
        "candidate_module_metadata": INPUT_PATHS["candidate_module_metadata"],
        "actor_feature_manifest": INPUT_PATHS["candidate_actor_manifest"],
    }.items():
        _exact_record(freeze.get(key), path, f"candidate_freeze.{key}")

    receipt = _strict(INPUT_PATHS["candidate_receipt"])
    if (
        receipt.get("schema_version") != 4
        or receipt.get("status") != "PASS_H0_PRIMARY_SPLIT_V4_OFFLINE"
        or receipt.get("passed") is not True
    ):
        raise ZeroUpdateFreezeError("candidate adaptation receipt is not a PASS")
    _zero_footer(receipt, "candidate receipt", actor_updates=1)
    offline_gate = _strict(INPUT_PATHS["candidate_offline_gate"])
    if (
        offline_gate.get("schema_version") != 4
        or offline_gate.get("status") != "PASS_H0_PRIMARY_SPLIT_V4_OFFLINE"
        or offline_gate.get("passed") is not True
        or offline_gate.get("candidate_actor_digest")
        != freeze.get("candidate_actor_digest")
    ):
        raise ZeroUpdateFreezeError("candidate offline gate is not a PASS")
    _zero_footer(offline_gate, "candidate offline gate", actor_updates=1)
    for key in ("v3_terminal_ledger", "v3_corpus"):
        record = freeze.get(key)
        if not isinstance(record, Mapping):
            raise ZeroUpdateFreezeError(f"candidate freeze {key} is malformed")
        try:
            expected = driver._resolve_relative(str(record.get("path", "")))  # noqa: SLF001
        except RuntimeError as exc:
            raise ZeroUpdateFreezeError(str(exc)) from exc
        _exact_record(record, expected, f"candidate_freeze.{key}")
    manifest = _strict(INPUT_PATHS["candidate_actor_manifest"])
    actor_digest = freeze.get("candidate_actor_digest")
    feature_names = manifest.get("actor_feature_names")
    if (
        not isinstance(actor_digest, str)
        or len(actor_digest) != 64
        or not isinstance(feature_names, list)
        or len(feature_names) != contract.EXPECTED_ACTOR_FEATURES
        or any(not isinstance(name, str) or not name for name in feature_names)
        or len(set(feature_names)) != len(feature_names)
        or manifest.get("schema_version") != 4
        or manifest.get("candidate_id") != contract.CANDIDATE_ID
        or manifest.get("observation_contract_id") != "primary_grf_split_v1"
        or manifest.get("event_contract_id") != "legacy_events_v1"
        or manifest.get("trainable_scope") != "full_mean_network"
        or manifest.get("logstd_policy") != "frozen_bit_exact"
        or manifest.get("actor_feature_count") != contract.EXPECTED_ACTOR_FEATURES
        or manifest.get("actor_digest") != actor_digest
        or manifest.get("module_state_sha256")
        != driver._sha256_file(INPUT_PATHS["candidate_module_state"])  # noqa: SLF001
    ):
        raise ZeroUpdateFreezeError("candidate actor manifest/digest drifted")
    return actor_digest


def _validate_holdout() -> None:
    receipt = _strict(INPUT_PATHS["holdout_receipt"])
    expected_keys = {
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
        set(receipt) != expected_keys
        or receipt.get("schema_version") != 4
        or receipt.get("status") != "PASS_H0_PRIMARY_SPLIT_V4_FINAL_HOLDOUT"
        or receipt.get("passed") is not True
    ):
        raise ZeroUpdateFreezeError("V4 final holdout receipt is not a PASS")
    _zero_footer(receipt, "holdout receipt", actor_updates=1)
    if receipt.get("additional_actor_updates") != 0:
        raise ZeroUpdateFreezeError("holdout performed an additional actor update")
    _exact_record(
        receipt.get("candidate_freeze"),
        INPUT_PATHS["candidate_freeze"],
        "holdout.candidate_freeze",
    )
    _exact_record(
        receipt.get("gate"), INPUT_PATHS["holdout_gate"], "holdout.gate"
    )
    _exact_record(
        receipt.get("holdout_replay_receipt"),
        INPUT_PATHS["holdout_replay_receipt"],
        "holdout.holdout_replay_receipt",
    )
    gate = _strict(INPUT_PATHS["holdout_gate"])
    replay = _strict(INPUT_PATHS["holdout_replay_receipt"])
    if (
        gate.get("schema_version") != 4
        or gate.get("status") != "PASS_H0_PRIMARY_SPLIT_V4_FINAL_HOLDOUT"
        or gate.get("passed") is not True
        or gate.get("additional_actor_updates") != 0
        or replay.get("schema_version") != 4
        or replay.get("status") != "PASS_H0_PRIMARY_SPLIT_V4_REPLAY"
        or replay.get("passed") is not True
        or replay.get("seed") != 125
        or replay.get("additional_actor_updates") != 0
    ):
        raise ZeroUpdateFreezeError("V4 holdout gate/replay is not a PASS")
    _zero_footer(gate, "holdout gate", actor_updates=1)
    _zero_footer(replay, "holdout replay", actor_updates=1)


def _validate_qualification(actor_digest: str) -> None:
    lock = _strict(INPUT_PATHS["qualification_lock"])
    authority = lock.get("authority")
    if (
        lock.get("schema_version") != 4
        or lock.get("status") != contract.QUALIFICATION_LOCK_STATUS
        or lock.get("protocol_id") != contract.QUALIFICATION_PROTOCOL_ID
        or lock.get("source_protocol_id") != contract.SOURCE_PROTOCOL_ID
        or lock.get("candidate_id") != contract.CANDIDATE_ID
        or lock.get("so_policy_id") != contract.SO_POLICY_ID
        or lock.get("event_contract_id") != contract.EVENT_CONTRACT_ID
        or lock.get("retry_or_retuning_allowed") is not False
        or not isinstance(authority, Mapping)
        or authority.get("actor_updates_authorized") is not False
        or authority.get("critic_updates_authorized") is not False
        or authority.get("ppo_updates_authorized") is not False
        or authority.get("protected_trial_access_authorized") is not False
        or lock.get("protected_trials_opened") != []
    ):
        raise ZeroUpdateFreezeError("autonomous qualification lock is ineligible")
    for section in ("sources", "inputs"):
        records = lock.get(section)
        if not isinstance(records, Mapping) or not records:
            raise ZeroUpdateFreezeError(
                f"qualification lock {section} closure is missing"
            )
        for name, record in records.items():
            if not isinstance(record, Mapping):
                raise ZeroUpdateFreezeError(
                    f"qualification lock {section}.{name} is malformed"
                )
            try:
                expected = driver._resolve_relative(str(record.get("path", "")))  # noqa: SLF001
            except RuntimeError as exc:
                raise ZeroUpdateFreezeError(str(exc)) from exc
            _exact_record(record, expected, f"qualification.{section}.{name}")
    candidate_module = lock.get("candidate_module")
    if (
        not isinstance(candidate_module, Mapping)
        or candidate_module.get("state")
        != driver.source_record(INPUT_PATHS["candidate_module_state"])
        or candidate_module.get("actor_feature_manifest")
        != driver.source_record(INPUT_PATHS["candidate_actor_manifest"])
    ):
        raise ZeroUpdateFreezeError("qualification did not freeze this candidate")
    prerequisite = lock.get("post_holdout_prerequisite")
    if (
        not isinstance(prerequisite, Mapping)
        or prerequisite.get("status")
        != "PASS_H0_PRIMARY_SPLIT_V4_POST_HOLDOUT_PREREQUISITE"
        or prerequisite.get("candidate_actor_digest") != actor_digest
    ):
        raise ZeroUpdateFreezeError("qualification V4 prerequisite drifted")
    for key, path in {
        "execution_ledger": INPUT_PATHS["v4_execution_ledger"],
        "holdout_receipt": INPUT_PATHS["holdout_receipt"],
        "candidate_freeze": INPUT_PATHS["candidate_freeze"],
    }.items():
        _exact_record(
            prerequisite.get(key), path, f"qualification.prerequisite.{key}"
        )

    decision = _strict(INPUT_PATHS["qualification_decision_receipt"])
    runtime_contract = decision.get("runtime_contract")
    if (
        decision.get("schema_version") != 4
        or decision.get("status")
        != "H0_PRIMARY_SPLIT_V4_QUALIFICATION_BASELINE_TOLERANCE_DECIDED"
        or decision.get("passed") is not True
        or decision.get("protocol_id") != contract.SOURCE_PROTOCOL_ID
        or decision.get("canonical_case_ids")
        != list(contract.QUALIFICATION_CASE_IDS)
        or not isinstance(runtime_contract, Mapping)
        or runtime_contract.get("event_contract_id")
        != contract.EVENT_CONTRACT_ID
        or runtime_contract.get("phase_fsm_input_mode") != "legacy_events"
        or runtime_contract.get("morphology_weight") != 0.0
        or decision.get("protected_trials_opened") != []
    ):
        raise ZeroUpdateFreezeError("qualification decision receipt is ineligible")
    _zero_footer(decision, "qualification decision", actor_updates=0)
    _exact_record(
        decision.get("candidate_holdout_receipt"),
        INPUT_PATHS["holdout_receipt"],
        "qualification.candidate_holdout_receipt",
    )

    ledger = _strict(INPUT_PATHS["qualification_execution_ledger"])
    case_gates = ledger.get("case_gates")
    if (
        ledger.get("schema_version") != 4
        or ledger.get("status") != contract.QUALIFICATION_PASS_STATUS
        or ledger.get("passed") is not True
        or ledger.get("error") is not None
        or ledger.get("baseline_rollouts_completed")
        != len(contract.QUALIFICATION_CASE_IDS)
        or ledger.get("candidate_rollouts_completed")
        != len(contract.QUALIFICATION_CASE_IDS)
        or ledger.get("next_stage") != "TRAINER_ZERO_UPDATE_PORT"
        or not isinstance(case_gates, Mapping)
        or set(case_gates) != set(contract.QUALIFICATION_CASE_IDS)
    ):
        raise ZeroUpdateFreezeError("autonomous qualification ledger is not a PASS")
    _zero_footer(ledger, "qualification ledger", actor_updates=0)
    _exact_record(
        ledger.get("qualification_lock"),
        INPUT_PATHS["qualification_lock"],
        "qualification.qualification_lock",
    )
    _exact_record(
        ledger.get("v4_execution_ledger"),
        INPUT_PATHS["v4_execution_ledger"],
        "qualification.v4_execution_ledger",
    )
    _exact_record(
        ledger.get("v4_holdout_receipt"),
        INPUT_PATHS["holdout_receipt"],
        "qualification.v4_holdout_receipt",
    )
    for case_id in contract.QUALIFICATION_CASE_IDS:
        gate = case_gates[case_id]
        if (
            not isinstance(gate, Mapping)
            or gate.get("schema_version") != 4
            or gate.get("status")
            != "PASS_H0_PRIMARY_SPLIT_V4_QUALIFICATION_CASE"
            or gate.get("passed") is not True
            or gate.get("case_id") != case_id
        ):
            raise ZeroUpdateFreezeError(f"qualification gate failed: {case_id}")
        gate_path = INPUT_PATHS[f"qualification_gate_{case_id}"]
        if driver._canonical_json_bytes(gate) != driver._canonical_json_bytes(  # noqa: SLF001
            _strict(gate_path)
        ):
            raise ZeroUpdateFreezeError(
                f"qualification ledger/gate mismatch: {case_id}"
            )
    if not isinstance(actor_digest, str) or len(actor_digest) != 64:
        raise ZeroUpdateFreezeError("qualified candidate actor digest is malformed")


def _payload() -> dict[str, Any]:
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.LOCK_STATUS,
        "protocol_id": contract.PROTOCOL_ID,
        "source_protocol_id": contract.SOURCE_PROTOCOL_ID,
        "revision": contract.REVISION,
        "candidate_id": contract.CANDIDATE_ID,
        "so_policy_id": contract.SO_POLICY_ID,
        "event_contract_id": contract.EVENT_CONTRACT_ID,
        "output_root": driver._repo_relative(OUTPUT_ROOT),  # noqa: SLF001
        "target_config_overrides": dict(contract.TARGET_CONFIG_OVERRIDES),
        "authority": dict(contract.AUTHORITY),
        "sources": {key: driver.source_record(path) for key, path in SOURCE_PATHS.items()},
        "inputs": {key: driver.source_record(path) for key, path in INPUT_PATHS.items()},
        "required_checks": list(driver.REQUIRED_CHECKS),
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "environment_samples": 0,
        "protected_trials_opened": [],
    }


def freeze() -> dict[str, Any]:
    if os.path.lexists(LOCK):
        raise ZeroUpdateFreezeError(f"refusing to clobber lock: {LOCK}")
    if os.path.lexists(OUTPUT_ROOT):
        raise ZeroUpdateFreezeError(
            f"zero-update output exists before freeze: {OUTPUT_ROOT}"
        )
    _validate_v4_execution()
    actor_digest = _validate_candidate()
    _validate_holdout()
    _validate_qualification(actor_digest)
    payload = _payload()
    if set(payload) != driver.LOCK_KEYS:
        raise ZeroUpdateFreezeError("freezer/driver lock schema disagree")
    if os.path.lexists(LOCK) or os.path.lexists(OUTPUT_ROOT):
        raise ZeroUpdateFreezeError("lock/output appeared during freeze")
    try:
        driver.write_json_exclusive(LOCK, payload)
        driver.verify_lock()
    except driver.ZeroUpdatePortError as exc:
        raise ZeroUpdateFreezeError(str(exc)) from exc
    if os.path.lexists(OUTPUT_ROOT):
        raise ZeroUpdateFreezeError("freezer unexpectedly created output root")
    return payload


if __name__ == "__main__":
    try:
        print(json.dumps(freeze(), indent=2, sort_keys=True, allow_nan=False))
    except Exception as exc:
        print(f"{type(exc).__name__}: {exc}", file=sys.stderr)
        raise SystemExit(2) from exc
