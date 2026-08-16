"""Freeze the protocol-only H0 adaptation branch for ``primary_grf_split_v1``.

This script records the new user-authorized protocol after the terminal
``ERROR_H0_REFERENCE`` result.  It deliberately authorizes no rollout,
training, actor/critic update, PPO step, protected/reserve-trial access, or
runtime promotion.  A later, separate no-clobber execution unlock must pin the
implementation, tests, preregistered noise tapes, and empty destinations
against the protocol frozen here before collection.  The generated corpus and
teacher PASS receipt must then be frozen atomically before any actor update.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
import os
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any, Mapping, Sequence


REPO_ROOT = Path(__file__).resolve().parents[1]
VALIDATION_ROOT = REPO_ROOT / "validation"
DESTINATION = VALIDATION_ROOT / "h0_primary_grf_split_v1_adaptation_protocol_lock.json"

PROTOCOL_ID = "AB06_H0_PRIMARY_GRF_SPLIT_V1_ADAPTATION_PROTOCOL_ONLY"
PROTOCOL_DATE = "2026-08-05"
PROTOCOL_STATUS = (
    "H0_PRIMARY_GRF_SPLIT_V1_ADAPTATION_PROTOCOL_FROZEN_"
    "EXECUTION_AND_UPDATES_NOT_AUTHORIZED"
)

H0_ROOT = (
    VALIDATION_ROOT
    / "critic_warmup"
    / "2026-07-13_markov35_phase_aligned_sigma0005_iter1_retry"
)
PRIMARY_READINESS_PROTOCOL = VALIDATION_ROOT / "primary_grf_readiness_protocol_v5.json"

FIXED_PATHS: dict[str, Path] = {
    "historical_plan": (
        REPO_ROOT
        / "reports"
        / "plans"
        / "2026-07-23_piano_grf_primaria_h0_sep_detector_training_ready.md"
    ),
    "v25_addendum": (
        REPO_ROOT
        / "reports"
        / "plans"
        / "2026-08-04_addendum_v25_protocollo_h0_abc.md"
    ),
    "primary_split_addendum": (
        REPO_ROOT
        / "reports"
        / "plans"
        / "2026-08-05_addendum_h0_primary_grf_split_v1_adaptation.md"
    ),
    "corrected_v25_protocol": VALIDATION_ROOT / "h0_v25_abc_protocol_corrected_lock.json",
    "consumed_v25_execution_unlock": VALIDATION_ROOT / "h0_v25_abc_execution_unlock.json",
    "terminal_v25_execution_ledger": (
        VALIDATION_ROOT
        / "h0_v25_abc_runs"
        / "2026-08-05_h0_v25_abc_full_environment_preflight"
        / "execution_ledger.json"
    ),
    "terminal_report": (
        REPO_ROOT
        / "reports"
        / "user"
        / "2026-08-05_preflight_h0_v25_abc_error_h0_reference.md"
    ),
    "primary_core_lock": VALIDATION_ROOT / "primary_grf_core_lock_2026-08-03.json",
    "primary_readiness_protocol": PRIMARY_READINESS_PROTOCOL,
    "h0_module_state": H0_ROOT / "rl_module_last" / "module_state.pkl",
    "h0_module_constructor": H0_ROOT / "rl_module_last" / "class_and_ctor_args.pkl",
    "h0_module_metadata": H0_ROOT / "rl_module_last" / "metadata.json",
    "h0_config": H0_ROOT / "training_cfg.resolved.yaml",
    "primary_profile": (
        REPO_ROOT
        / "online_grf_profiles"
        / "AB06_SEASEA_stiff321_500_pi_grf_correct_magnitude.json"
    ),
    "legacy_detector_profile": (
        REPO_ROOT
        / "online_grf_profiles"
        / "AB06_SEASEA_stiff321_500_pi_grf_detector_HS-TO.json"
    ),
    "runtime_setup": (
        REPO_ROOT
        / "models"
        / "AB06_SEASEA_Threadmill"
        / "AB06_SEASEA_stiff321_500_pi_setup.xml"
    ),
    "input_prepare_script": (
        VALIDATION_ROOT / "prepare_h0_primary_grf_split_inputs.py"
    ),
    "input_manifest": (
        VALIDATION_ROOT / "h0_primary_grf_split_inputs" / "manifest.json"
    ),
    "overlay_02_primary": (
        VALIDATION_ROOT
        / "h0_primary_grf_split_inputs"
        / "trial_02_primary_surface_velocity.json"
    ),
    "overlay_02_analog": (
        VALIDATION_ROOT
        / "h0_primary_grf_split_inputs"
        / "trial_02_analog_surface_velocity.json"
    ),
    "overlay_04_primary": (
        VALIDATION_ROOT
        / "h0_primary_grf_split_inputs"
        / "trial_04_primary_surface_velocity.json"
    ),
    "overlay_04_analog": (
        VALIDATION_ROOT
        / "h0_primary_grf_split_inputs"
        / "trial_04_analog_surface_velocity.json"
    ),
    "overlay_08_primary": (
        VALIDATION_ROOT
        / "h0_primary_grf_split_inputs"
        / "trial_08_primary_surface_velocity.json"
    ),
    "overlay_08_analog": (
        VALIDATION_ROOT
        / "h0_primary_grf_split_inputs"
        / "trial_08_analog_surface_velocity.json"
    ),
}

EXPECTED_FIXED_HASHES = {
    "historical_plan": "2dd4e0a06e13dca87ce74a1fe2bb601a1822ea4decc78cd60987ec250f47b520",
    "v25_addendum": "a8cea18b338b08c32225c3912561cefe953add3b9e4bb693009aba6b189e9835",
    "primary_split_addendum": "dc8fe7e3b3c3f3135bba3828ab497f1bc7dd8fec6e06dc97ba781b2bb9bb671d",
    "corrected_v25_protocol": "04ae8e209ccae05075b625f89ac827b145d5149e4237fe2128b1c822d105fe8b",
    "consumed_v25_execution_unlock": "7497624066a10799697c17bb63406969656ab67707afbb3d04d7cb0bbc22171d",
    "terminal_v25_execution_ledger": "068f2c604b523867f80f7b0d711bf256a2380a3b0351a124b4eab533896c8c51",
    "terminal_report": "88a4963b570dedfb9ce2738e43a3b1e9e216acee5f287a58223744b115e63845",
    "primary_core_lock": "e9347bb5e5d04e84ce96f5c3ed354e154d26d0bc5abb750dc68bb8b79d0c06ac",
    "primary_readiness_protocol": "ddd8cff5dced20dc10ff9c61693273d3683e68c770ae77d4ed72690101bbedf0",
    "h0_module_state": "44457ca5df7fa0e0e1f1d361d940136917fe8f71e984a1b0afaefb8ca3ced33b",
    "h0_module_constructor": "5c98f006d99a71a0f1ddcbb31d8d73fe0a6dade8401e679f6af5b1bc943b4228",
    "h0_module_metadata": "3a032ba54abcee8c9bcbb39e72fa05566912e94461d01f3c6228dc60e088bf12",
    "h0_config": "6904f7a9000b63b5c1aab661ebcab4974dffdd1cfb8c731df6a953fc9234229e",
    "primary_profile": "09e04ab94954703d74acc3a80b24ecefcc07d3fc918c03b9e9df8116a6c1a2b0",
    "legacy_detector_profile": "61ea948a3c0613e5c0e684a3197de118c7116e36188fca6993da79ce713fd99e",
    "runtime_setup": "5fa4748e537ae1a0fbda091a5ed2b5774ac445a4904b3ad0bb02f259bf0a4931",
    "input_prepare_script": "a3f72c3ee7bdfcbd03a9f2e309b28554acefec17a72329e41eaa951efa8237fc",
    "input_manifest": "cb35c7a9b8d07f23bd0c8d5043713fd30b345e37ef3fa95b2d09922a1fdc5798",
    "overlay_02_primary": "2c24dd774ddbad14279447e8a4ecd0e2eae7856e4683fbf6c602c852444b01c8",
    "overlay_02_analog": "35fb7872594a517d65e8f0d0e42f8a5a5520549547ef5c76b0560553cb08426f",
    "overlay_04_primary": "59d3ee250d116e3f65aff2de6696a45ec2de78a3a34206179c51db6634d801a7",
    "overlay_04_analog": "661bdefb9cdfc1df15b9aea5750f5a3cec25cb6b4943eb7b41fd8483202da9a5",
    "overlay_08_primary": "7a361839ef94e349dc41fdb312bb4b0bd4c4eb53feedf1f812e586e11da35e84",
    "overlay_08_analog": "748cdea67e82b2c750ef4e715d6c7e0bfc3ee0e03b42571005db47ad2545f526",
}

DEV_TRIALS = ("02", "04", "08")
DEV_PLATEAUS = ("01", "02", "03", "04")
DEV_CADENCES = ("01ms", "10ms")
DEV_SETUP_SOURCE_NAMES = {
    "02": "development_02_setup",
    "04": "development_04_setup",
    "08": "development_08_setup",
}

TRIAL_SPEEDS_MPS = {"02": 0.95, "04": 1.05, "08": 1.25}

PHASE_FSM_FEATURES = [
    "phase_fsm_wait_hs",
    "phase_fsm_stance_after_hs",
    "phase_fsm_swing_after_to",
    "phase_expected_hs",
    "phase_expected_to",
    "phase_stance_elapsed_norm",
    "phase_swing_elapsed_norm",
    "phase_cycle_progress_credit",
]

AUTHORITY = {
    "h0_primary_split_supervised_adaptation_protocol_authorized": True,
    "h0_primary_split_supervised_adaptation_execution_authorized": False,
    "actor_updates_authorized": False,
    "critic_updates_authorized": False,
    "rollout_execution_authorized": False,
    "h0_primary_split_actor_updates_authorized": False,
    "h0_primary_split_critic_updates_authorized": False,
    "h0_primary_split_baseline_rollout_authorized": False,
    "h0_primary_split_teacher_collection_authorized": False,
    "h0_primary_split_candidate_qualification_authorized": False,
    "h0_preflight_execution_authorized": False,
    "h0_v25_abc_execution_authorized": False,
    "v25_ab_c_execution_authorized": False,
    "h0_sep_authorized": False,
    "general_training_authorized": False,
    "training_authorized": False,
    "ppo_updates_authorized": False,
    "protected_trial_access_authorized": False,
    "reserve_trial_access_authorized": False,
    "corridor_authorized": False,
    "runtime_promotion_authorized": False,
    "primary_grf_modification_authorized": False,
    "sea_semantic_modification_authorized": False,
    "detector_retuning_authorized": False,
}


class H0PrimarySplitProtocolError(RuntimeError):
    """Raised when the protocol-only lock cannot be reproduced exactly."""


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _reject_constant(token: str) -> None:
    raise H0PrimarySplitProtocolError(f"non-finite JSON constant: {token}")


def _reject_duplicate_pairs(pairs: Sequence[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise H0PrimarySplitProtocolError(f"duplicate JSON key: {key!r}")
        result[key] = value
    return result


def _finite_tree(value: object) -> bool:
    if value is None or isinstance(value, (bool, int, str)):
        return True
    if isinstance(value, float):
        return math.isfinite(value)
    if isinstance(value, Mapping):
        return all(
            isinstance(key, str) and _finite_tree(item)
            for key, item in value.items()
        )
    if isinstance(value, (list, tuple)):
        return all(_finite_tree(item) for item in value)
    return False


def strict_json_load(path: Path) -> dict[str, Any]:
    if not path.is_file():
        raise H0PrimarySplitProtocolError(f"missing JSON source: {path}")
    try:
        payload = json.loads(
            path.read_text(encoding="utf-8"),
            object_pairs_hook=_reject_duplicate_pairs,
            parse_constant=_reject_constant,
        )
    except H0PrimarySplitProtocolError:
        raise
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise H0PrimarySplitProtocolError(f"cannot parse strict JSON: {path}") from exc
    if not isinstance(payload, dict):
        raise H0PrimarySplitProtocolError(f"JSON root is not an object: {path}")
    if not _finite_tree(payload):
        raise H0PrimarySplitProtocolError(f"non-finite JSON tree: {path}")
    return payload


def encode_json(payload: Mapping[str, Any]) -> bytes:
    if not _finite_tree(payload):
        raise H0PrimarySplitProtocolError("refusing to encode non-finite payload")
    return (
        json.dumps(dict(payload), indent=2, sort_keys=True, allow_nan=False) + "\n"
    ).encode("utf-8")


def source_record(path: Path) -> dict[str, Any]:
    resolved = path.resolve()
    if not resolved.is_file():
        raise H0PrimarySplitProtocolError(f"missing source: {resolved}")
    try:
        relative = resolved.relative_to(REPO_ROOT.resolve()).as_posix()
    except ValueError as exc:
        raise H0PrimarySplitProtocolError(f"source escapes repository: {resolved}") from exc
    return {
        "path": relative,
        "sha256": sha256_file(resolved),
        "size_bytes": int(resolved.stat().st_size),
    }


def payload_record(path: Path, payload: Mapping[str, Any]) -> dict[str, Any]:
    encoded = encode_json(payload)
    return {
        "path": path.resolve().relative_to(REPO_ROOT.resolve()).as_posix(),
        "sha256": hashlib.sha256(encoded).hexdigest(),
        "size_bytes": len(encoded),
    }


def _mapping(value: Any, label: str) -> dict[str, Any]:
    if not isinstance(value, Mapping):
        raise H0PrimarySplitProtocolError(f"{label} is not an object")
    return dict(value)


def _verify_fixed_sources() -> tuple[dict[str, Any], dict[str, dict[str, Any]]]:
    records = {name: source_record(path) for name, path in FIXED_PATHS.items()}
    for name, expected in EXPECTED_FIXED_HASHES.items():
        if records[name]["sha256"] != expected:
            raise H0PrimarySplitProtocolError(
                f"fixed source {name} drifted: {records[name]['sha256']}"
            )

    corrected = strict_json_load(FIXED_PATHS["corrected_v25_protocol"])
    if corrected.get("protocol_executed") is not False:
        raise H0PrimarySplitProtocolError("corrected V25 protocol claims execution")

    consumed_unlock = strict_json_load(FIXED_PATHS["consumed_v25_execution_unlock"])
    if consumed_unlock.get("scope") != "ZERO_UPDATE_H0_FULL_ENVIRONMENT_PREFLIGHT_ONLY":
        raise H0PrimarySplitProtocolError("consumed unlock scope drifted")
    if consumed_unlock.get("timeouts", {}).get("retry_allowed") is not False:
        raise H0PrimarySplitProtocolError("consumed unlock permits retry")

    terminal = strict_json_load(FIXED_PATHS["terminal_v25_execution_ledger"])
    if terminal.get("status") != "ERROR_H0_REFERENCE":
        raise H0PrimarySplitProtocolError("terminal H0 result drifted")
    if terminal.get("next_stage") != "STOP_WITHOUT_RETRY_RETUNING_OR_FALLBACK":
        raise H0PrimarySplitProtocolError("terminal H0 stop semantics drifted")
    if terminal.get("ppo_updates") != 0 or terminal.get("training_performed") is not False:
        raise H0PrimarySplitProtocolError("terminal H0 run performed updates")
    if terminal.get("protected_trials_opened") != []:
        raise H0PrimarySplitProtocolError("terminal H0 run opened protected trials")

    primary_core = strict_json_load(FIXED_PATHS["primary_core_lock"])
    if primary_core.get("status") != "PRIMARY_CONTRACT_FROZEN_LIMITED_HYBRID_CLAIM":
        raise H0PrimarySplitProtocolError("primary core lock is not frozen")

    readiness = strict_json_load(PRIMARY_READINESS_PROTOCOL)
    if readiness.get("status") != "FROZEN":
        raise H0PrimarySplitProtocolError("primary readiness protocol is not frozen")
    if readiness.get("protected_trials_opened") != []:
        raise H0PrimarySplitProtocolError("primary readiness opened protected trials")
    expected_closed = {
        "03": "CLOSED_PRIMARY_VALIDATION_ONE_SHOT",
        "05": "CLOSED_DETECTOR_VALIDATION_ONE_SHOT",
        "06": "CLOSED_DETECTOR_SEALED_ONE_SHOT",
        "07": "CLOSED_PRIMARY_SEALED_ONE_SHOT",
    }
    if readiness.get("protected_trials") != expected_closed:
        raise H0PrimarySplitProtocolError("primary readiness trial governance drifted")
    return readiness, records


def _development_records(readiness: Mapping[str, Any]) -> dict[str, Any]:
    sources = _mapping(readiness.get("sources"), "primary readiness sources")
    setup_records: dict[str, Any] = {}
    receipt_records: dict[str, Any] = {}

    for trial, source_name in DEV_SETUP_SOURCE_NAMES.items():
        expected = _mapping(sources.get(source_name), source_name)
        setup_path = REPO_ROOT / str(expected.get("path"))
        record = source_record(setup_path)
        if record["sha256"] != expected.get("sha256"):
            raise H0PrimarySplitProtocolError(f"development setup {trial} drifted")
        try:
            grf_mode = ET.parse(setup_path).getroot().findtext(".//grf_mode")
        except (OSError, ET.ParseError) as exc:
            raise H0PrimarySplitProtocolError(
                f"development setup {trial} is not valid XML"
            ) from exc
        if grf_mode is None or grf_mode.strip() != "prescribed":
            raise H0PrimarySplitProtocolError(
                f"development setup {trial} is not prescribed-GRF"
            )
        setup_records[trial] = record

    for trial in DEV_TRIALS:
        for plateau in DEV_PLATEAUS:
            for cadence in DEV_CADENCES:
                source_name = f"unit_receipt_dev{trial}_p{plateau}_{cadence}"
                expected = _mapping(sources.get(source_name), source_name)
                path = REPO_ROOT / str(expected.get("path"))
                record = source_record(path)
                if record["sha256"] != expected.get("sha256"):
                    raise H0PrimarySplitProtocolError(
                        f"development receipt {source_name} drifted"
                    )
                receipt = strict_json_load(path)
                expected_unit = f"dev{trial}_p{plateau}_{cadence}"
                if receipt.get("status") != "PASS" or receipt.get("unit_id") != expected_unit:
                    raise H0PrimarySplitProtocolError(
                        f"development receipt {source_name} is not the expected PASS unit"
                    )
                if receipt.get("source_trial_id") != trial:
                    raise H0PrimarySplitProtocolError(
                        f"development receipt {source_name} has wrong trial"
                    )
                if receipt.get("access_role") != "DEVELOPMENT_OPEN":
                    raise H0PrimarySplitProtocolError(
                        f"development receipt {source_name} is not development-open"
                    )
                if receipt.get("protected_component_outcome") is not False:
                    raise H0PrimarySplitProtocolError(
                        f"development receipt {source_name} exposes a protected outcome"
                    )
                if plateau == "04":
                    expected_speed = TRIAL_SPEEDS_MPS[trial]
                    if receipt.get("plateau_speed_mps") != expected_speed:
                        raise H0PrimarySplitProtocolError(
                            f"development receipt {source_name} speed drifted"
                        )
                    if receipt.get("surface_velocity_mps") != [
                        0.0,
                        0.0,
                        expected_speed,
                    ]:
                        raise H0PrimarySplitProtocolError(
                            f"development receipt {source_name} velocity drifted"
                        )
                semantic_sources = _mapping(
                    receipt.get("semantic_sources"), f"{source_name}.semantic_sources"
                )
                receipt_setup = _mapping(
                    semantic_sources.get("setup"), f"{source_name}.semantic_sources.setup"
                )
                if (
                    receipt_setup.get("path") != setup_records[trial]["path"]
                    or receipt_setup.get("sha256") != setup_records[trial]["sha256"]
                ):
                    raise H0PrimarySplitProtocolError(
                        f"development receipt {source_name} does not pin its trial setup"
                    )
                receipt_records[expected_unit] = record

    if len(setup_records) != 3 or len(receipt_records) != 24:
        raise H0PrimarySplitProtocolError("development setup/receipt cardinality drifted")
    return {
        "trials": list(DEV_TRIALS),
        "trial_setups": setup_records,
        "unit_source_receipts": receipt_records,
        "unit_receipt_count": len(receipt_records),
        "raw_development_data_opened_by_this_freeze": False,
        "metadata_receipts_verified_read_only": True,
        "source_setup_grf_mode_exact": "prescribed",
        "plateau_04_speed_and_velocity_receipts_verified": True,
    }


def _without_surface_velocity(profile: Mapping[str, Any]) -> dict[str, Any]:
    result = copy.deepcopy(dict(profile))
    ground = _mapping(result.get("ground"), "profile.ground")
    ground.pop("surface_velocity", None)
    result["ground"] = ground
    return result


def _prepared_overlay_records(records: Mapping[str, Mapping[str, Any]]) -> dict[str, Any]:
    manifest = strict_json_load(FIXED_PATHS["input_manifest"])
    if manifest.get("schema_version") != 1:
        raise H0PrimarySplitProtocolError("input manifest schema drifted")
    if manifest.get("status") != "H0_PRIMARY_GRF_SPLIT_INPUTS_PREPARED":
        raise H0PrimarySplitProtocolError("input manifest status drifted")
    if manifest.get("mutation_whitelist") != ["ground.surface_velocity"]:
        raise H0PrimarySplitProtocolError("input overlay mutation whitelist drifted")
    if manifest.get("protected_trials_opened") != []:
        raise H0PrimarySplitProtocolError("input preparation opened protected trials")
    if manifest.get("primary_profile_overwritten") is not False:
        raise H0PrimarySplitProtocolError("primary profile overwrite flag drifted")
    if manifest.get("analog_profile_overwritten") is not False:
        raise H0PrimarySplitProtocolError("analog profile overwrite flag drifted")

    source_profiles = _mapping(
        manifest.get("source_profiles"), "input manifest source_profiles"
    )
    if set(source_profiles) != {"primary", "analog"}:
        raise H0PrimarySplitProtocolError("input manifest source roles drifted")
    source_by_role = {
        "primary": records["primary_profile"],
        "analog": records["legacy_detector_profile"],
    }
    for role, expected_record in source_by_role.items():
        if _mapping(source_profiles.get(role), f"source_profiles.{role}") != expected_record:
            raise H0PrimarySplitProtocolError(
                f"input manifest {role} source record drifted"
            )

    manifest_trials = _mapping(manifest.get("trials"), "input manifest trials")
    if set(manifest_trials) != set(TRIAL_SPEEDS_MPS):
        raise H0PrimarySplitProtocolError("input manifest trial set drifted")

    source_payloads = {
        "primary": strict_json_load(FIXED_PATHS["primary_profile"]),
        "analog": strict_json_load(FIXED_PATHS["legacy_detector_profile"]),
    }
    frozen_trials: dict[str, Any] = {}
    for trial, speed_mps in TRIAL_SPEEDS_MPS.items():
        trial_manifest = _mapping(manifest_trials.get(trial), f"trials.{trial}")
        expected_velocity = [0.0, 0.0, speed_mps]
        if trial_manifest.get("speed_mps") != speed_mps:
            raise H0PrimarySplitProtocolError(f"trial {trial} speed drifted")
        if trial_manifest.get("surface_velocity_mps") != expected_velocity:
            raise H0PrimarySplitProtocolError(
                f"trial {trial} surface velocity drifted"
            )
        manifest_profiles = _mapping(
            trial_manifest.get("profiles"), f"trials.{trial}.profiles"
        )
        if set(manifest_profiles) != {"primary", "analog"}:
            raise H0PrimarySplitProtocolError(
                f"trial {trial} overlay roles drifted"
            )
        trial_records: dict[str, Any] = {}
        for role in ("primary", "analog"):
            fixed_name = f"overlay_{trial}_{role}"
            actual_record = records[fixed_name]
            if (
                _mapping(
                    manifest_profiles.get(role),
                    f"trials.{trial}.profiles.{role}",
                )
                != actual_record
            ):
                raise H0PrimarySplitProtocolError(
                    f"trial {trial} {role} manifest record drifted"
                )
            overlay = strict_json_load(FIXED_PATHS[fixed_name])
            overlay_ground = _mapping(
                overlay.get("ground"), f"trial {trial} {role} ground"
            )
            if overlay_ground.get("surface_velocity") != expected_velocity:
                raise H0PrimarySplitProtocolError(
                    f"trial {trial} {role} velocity mismatch"
                )
            if _without_surface_velocity(overlay) != _without_surface_velocity(
                source_payloads[role]
            ):
                raise H0PrimarySplitProtocolError(
                    f"trial {trial} {role} changed more than surface velocity"
                )
            trial_records[role] = actual_record
        frozen_trials[trial] = {
            "speed_mps": speed_mps,
            "surface_velocity_mps": expected_velocity,
            "profiles": trial_records,
        }

    return {
        "prepare_script": records["input_prepare_script"],
        "manifest": records["input_manifest"],
        "status": "PREPARED_AND_HASH_FROZEN_NOT_EXECUTION_AUTHORITY",
        "source_profiles": source_by_role,
        "mutation_whitelist": ["ground.surface_velocity"],
        "source_profiles_overwritten": False,
        "trials": frozen_trials,
    }


def build_payload(
    *, require_destination_unoccupied: bool = True
) -> dict[str, Any]:
    if require_destination_unoccupied and os.path.lexists(DESTINATION):
        raise H0PrimarySplitProtocolError(f"refusing to clobber: {DESTINATION}")

    readiness, records = _verify_fixed_sources()
    development = _development_records(readiness)
    prepared_overlays = _prepared_overlay_records(records)
    protocol_authority_key = (
        "h0_primary_split_supervised_adaptation_protocol_authorized"
    )
    assertions = {
        "only_protocol_definition_authorized": (
            AUTHORITY[protocol_authority_key] is True
            and all(
                value is False
                for key, value in AUTHORITY.items()
                if key != protocol_authority_key
            )
        ),
        "h0_and_config_exact": (
            records["h0_module_state"]["sha256"]
            == EXPECTED_FIXED_HASHES["h0_module_state"]
            and records["h0_config"]["sha256"]
            == EXPECTED_FIXED_HASHES["h0_config"]
        ),
        "primary_and_legacy_profiles_exact": (
            records["primary_profile"]["sha256"]
            == EXPECTED_FIXED_HASHES["primary_profile"]
            and records["legacy_detector_profile"]["sha256"]
            == EXPECTED_FIXED_HASHES["legacy_detector_profile"]
        ),
        "development_02_04_08_receipts_exact": (
            development["unit_receipt_count"] == 24
            and development["raw_development_data_opened_by_this_freeze"] is False
            and development["source_setup_grf_mode_exact"] == "prescribed"
            and development[
                "plateau_04_speed_and_velocity_receipts_verified"
            ]
            is True
        ),
        "primary_split_addendum_exact": (
            records["primary_split_addendum"]["sha256"]
            == EXPECTED_FIXED_HASHES["primary_split_addendum"]
        ),
        "prepared_cross_speed_overlays_exact": (
            prepared_overlays["status"]
            == "PREPARED_AND_HASH_FROZEN_NOT_EXECUTION_AUTHORITY"
            and set(prepared_overlays["trials"]) == set(DEV_TRIALS)
        ),
        "teacher_replaces_load_contact_and_phase_fsm": True,
        "corpus_fit_and_qualification_protocol_preregistered": True,
        "terminal_error_h0_reference_preserved": True,
        "primary_contract_not_reopened": True,
        "protected_and_reserve_trials_closed": True,
        "protocol_only_no_execution_or_update": True,
    }
    if not all(assertions.values()):
        raise H0PrimarySplitProtocolError(
            f"protocol-only assertions failed: {assertions}"
        )

    payload: dict[str, Any] = {
        "schema_version": 1,
        "protocol_id": PROTOCOL_ID,
        "date": PROTOCOL_DATE,
        "status": PROTOCOL_STATUS,
        "protocol_defined": True,
        "protocol_executed": False,
        "adaptation_executed": False,
        "actor_updates_performed": 0,
        "critic_updates_performed": 0,
        "ppo_updates_performed": 0,
        "scientific_result": "UNAVAILABLE_PROTOCOL_ONLY",
        "authorization_basis": {
            "user_instruction": "fallo",
            "preceding_proposal": (
                "Il protocollo impone ora lo stop senza retry. Per proseguire "
                "servirà una nuova autorizzazione per adattare H0 alla semantica "
                "primary_grf_split_v1, senza ripristinare il detector come "
                "sorgente del carico continuo."
            ),
            "addendum_authority_shorthand": (
                "h0_primary_split_supervised_adaptation_authorized=true"
            ),
            "lock_normalization": (
                "THE_SHORTHAND_AUTHORIZES_PROTOCOL_DEFINITION_ONLY;_"
                "EXECUTION_AND_ACTOR_UPDATES_REMAIN_FALSE"
            ),
            "scope": "DEFINE_AND_FREEZE_NEW_PRIMARY_SPLIT_PROTOCOL_ONLY",
            "distinct_from_consumed_v25_preflight_authorization": True,
            "separate_execution_unlock_required": True,
        },
        "authority": dict(AUTHORITY),
        "lineage": {
            name: records[name]
            for name in (
                "historical_plan",
                "v25_addendum",
                "primary_split_addendum",
                "corrected_v25_protocol",
                "consumed_v25_execution_unlock",
                "terminal_v25_execution_ledger",
                "terminal_report",
            )
        },
        "frozen_inputs": {
            name: records[name]
            for name in (
                "primary_core_lock",
                "primary_readiness_protocol",
                "h0_module_state",
                "h0_module_constructor",
                "h0_module_metadata",
                "h0_config",
                "primary_profile",
                "legacy_detector_profile",
                "runtime_setup",
            )
        },
        "development_inputs": development,
        "prepared_cross_speed_inputs": prepared_overlays,
        "data_governance": {
            "historical_trial_01": "DIAGNOSTIC_ONLY_NOT_ADAPTATION_OR_HOLDOUT",
            "development_trials_declared": list(DEV_TRIALS),
            "development_data_execution_authorized_by_this_lock": False,
            "protected_trials": {
                "05": "CLOSED_VALIDATION_ONE_SHOT",
                "06": "CLOSED_SEALED_ONE_SHOT",
            },
            "reserve_trials": {
                "03": "UNALLOCATED_RESERVE_CLOSED",
                "07": "UNALLOCATED_RESERVE_CLOSED",
            },
            "protected_trials_opened": [],
            "reserve_trials_opened": [],
            "consumed_v25_abc_matrix_reopened": False,
            "old_v25_run_destinations_reused": False,
        },
        "adaptation_contract": {
            "candidate_label": "H0_primary_split_v1_FUTURE_UNAVAILABLE",
            "source_actor": "FROZEN_H0",
            "source_actor_immutable": True,
            "target_semantic_contract_id": (
                "primary_grf_split_v1+legacy_events_v1"
            ),
            "event_contract_during_adaptation": "legacy_events_v1",
            "student_runtime_semantics": {
                "actor_index_10_online_left_normal_grf_bw": (
                    "PRIMARY_ONLINE_GRF_ONLY"
                ),
                "actor_index_11_online_left_in_contact": (
                    "PRIMARY_ONLINE_GRF_ONLY"
                ),
                "phase_fsm_continuous_load_contact": "PRIMARY_ONLINE_GRF_ONLY",
                "phase_fsm_events": "legacy_events_v1",
            },
            "legacy_detector_continuous_actor_source_allowed": False,
            "legacy_detector_teacher_only": True,
            "fallback_away_from_primary_grf_allowed": False,
            "binary_v20_authoritative_during_adaptation": False,
            "v25_active_events_authorized": False,
            "v25_actor_input": False,
            "v20_enabled": False,
            "actor_layout_count": 35,
            "full_observation_count": 84,
            "actor_dtype": "float32",
            "action_shape": [2],
            "action_dtype": "float32",
            "observation_shape_order_dtype_change_allowed": False,
            "morphology_weight": 0.0,
            "method_family": "ACTOR_ONLY_SUPERVISED_ADAPTATION",
            "mean_network_trainable_scope": (
                "ENTIRE_MEAN_NETWORK_PROTOCOL_FROZEN_EXECUTION_LOCK_REQUIRED"
            ),
            "anchor_actor": "FROZEN_H0",
            "logstd": "FROZEN_BIT_EXACT",
            "critic_and_other_inference_tensors_during_fit": "UNCHANGED_BIT_EXACT",
            "critic_restore_or_update_authorized": False,
            "optimizer_restore_authorized": False,
            "ppo_or_reward_training_authorized": False,
            "original_h0_overwrite_allowed": False,
            "runtime_actor_prescription_allowed": False,
        },
        "teacher_view_contract": {
            "same_physical_state_as_student_view": True,
            "actor_index_base": 0,
            "direct_feature_replacements": [
                {
                    "index": 10,
                    "feature": "online_left_normal_grf_bw",
                    "teacher_value": (
                        "online_grf_detector.left.normal_force/body_weight_n"
                    ),
                },
                {
                    "index": 11,
                    "feature": "online_left_in_contact",
                    "teacher_value": "online_grf_detector.left.in_contact",
                },
            ],
            "phase_feature_slice": {
                "start_inclusive": 17,
                "end_exclusive": 25,
                "features_in_order": list(PHASE_FSM_FEATURES),
                "source": "INDEPENDENT_ANALOG_FED_PROSTHETIC_PHASE_FSM",
                "same_legacy_event_stream_as_student": True,
                "continuous_load_contact_source": "ANALOG_DETECTOR",
                "same_reset_semantics_as_h0_pre_split": True,
                "causal_inputs_at_or_before_current_time_only": True,
                "must_reproduce_h0_pre_split_features_exactly": True,
            },
            "unchanged_actor_feature_indices": (
                list(range(0, 10))
                + list(range(12, 17))
                + list(range(25, 35))
            ),
            "unchanged_actor_feature_count": 25,
            "unchanged_actor_features_required_bit_exact": True,
            "student_view_shape_dtype": "finite_float32[35]",
            "teacher_view_shape_dtype": "finite_float32[35]",
            "noise_vector_shape_dtype": "finite_float32[2]",
            "served_action_shape_dtype": "finite_unclipped_float32[2]",
            "student_view_matches_published_primary_grf_each_step": True,
            "teacher_view_matches_published_analog_detector_each_step": True,
            "analog_detector_force_free_required": True,
            "teacher_mean": "FROZEN_H0_DETERMINISTIC_MEAN_ON_TEACHER_VIEW",
            "paired_collection_served_action": "teacher_mean+sigma*z",
            "paired_collection_student_action_served": False,
            "candidate_qualification_teacher_timing": (
                "COUNTERFACTUAL_DIAGNOSTIC_ONLY_AFTER_CANDIDATE_ACTION_CHOSEN"
            ),
            "candidate_served_action_dependency_on_teacher_allowed": False,
        },
        "cross_speed_overlay_contract": {
            "source_setup_mode": "prescribed",
            "source_setup_trials": list(DEV_TRIALS),
            "source_setup_plateau": "04",
            "source_setups": development["trial_setups"],
            "supporting_plateau_receipts": {
                trial: {
                    cadence: development["unit_source_receipts"][
                        f"dev{trial}_p04_{cadence}"
                    ]
                    for cadence in DEV_CADENCES
                }
                for trial in DEV_TRIALS
            },
            "profile_roles": ["primary", "analog"],
            "only_allowed_profile_mutation": "ground.surface_velocity",
            "surface_velocity_by_trial_mps": {
                trial: [0.0, 0.0, speed]
                for trial, speed in TRIAL_SPEEDS_MPS.items()
            },
            "paired_profiles_same_speed_required": True,
            "geometry_material_contact_routing_applies_force_unchanged": True,
            "source_profiles_overwritten": False,
            "prepared_inputs_hash_frozen": True,
            "rollout_execution_authorized": False,
        },
        "paired_corpus_protocol": {
            "policy_step_s": 0.01,
            "steps_per_rollout_exact": 500,
            "states_per_rollout_exact": 500,
            "records_per_state_exact": 2,
            "records": [
                "student_view->teacher_mean",
                "teacher_view->teacher_mean",
            ],
            "label_shared_within_state": True,
            "environment_driver": "teacher_mean+sigma*z",
            "windows": {
                "02": {
                    "split": "TRAIN",
                    "speed_mps": 0.95,
                    "absolute_start_time_s": 119.578,
                    "setup_offset_s": 107.880,
                    "seed": 123,
                    "sigma": 0.0025,
                },
                "04": {
                    "split": "TRAIN",
                    "speed_mps": 1.05,
                    "absolute_start_time_s": 122.189,
                    "setup_offset_s": 107.550,
                    "seed": 124,
                    "sigma": 0.0025,
                },
                "08": {
                    "split": "OFFLINE_VALIDATION_NO_UPDATES",
                    "speed_mps": 1.25,
                    "absolute_start_time_s": 120.390,
                    "setup_offset_s": 106.878,
                    "seed": 125,
                    "sigma": 0.0025,
                },
            },
            "training_trials": ["02", "04"],
            "validation_trials": ["08"],
            "training_records_exact": 2000,
            "validation_records_exact": 1000,
            "group_split_no_leakage_required": True,
            "trial_08_used_for_updates": False,
            "collection_authorized": False,
        },
        "teacher_rollout_gate": {
            "steps_exact": 500,
            "duration_s": 5.0,
            "end_reason_exact": "episode_time_limit",
            "minimum_complete_valid_cycles": 2,
            "max_penetration_m_strictly_less_than": 0.025,
            "zero_required": [
                "action_clipping",
                "timeout",
                "safety_stop",
                "fallback",
                "hard_invalid",
                "nonfinite",
            ],
            "actor_and_full_layout_exact": [35, 84],
            "observation_dtype": "float32",
            "action_shape_and_dtype": "float32[2]",
            "morphology_weight_exact": 0.0,
            "sea_reserve_residual_metrics_all_finite": True,
            "v25_or_v20_observation_fields_allowed": False,
            "ppo_updates_exact": 0,
            "protected_accesses_exact": 0,
            "failure_terminal_outcome": "ERROR_PRIMARY_SPLIT_TEACHER",
            "failure_allows_fit": False,
            "rescue_retry_or_new_windows_allowed": False,
        },
        "one_shot_fit_protocol": {
            "execution_authorized": False,
            "candidate_count_max": 1,
            "sweep_allowed": False,
            "optimizer": "Adam",
            "learning_rate": 0.0001,
            "epochs_max": 300,
            "batch_size": 128,
            "patience": 60,
            "anchor_weight": 0.001,
            "clip_weight": 1.0,
            "logstd_weight": 0.0,
            "seed": 123,
            "training_records_exact": 2000,
            "validation_records_exact": 1000,
            "trainable_scope": "FULL_MEAN_ACTOR",
            "logstd": "FROZEN_BIT_EXACT",
            "critic_and_non_actor_tensors": "UNCHANGED_BIT_EXACT",
            "fail_allows_second_hyperparameter_seed_corpus_or_candidate": False,
        },
        "offline_adaptation_gates": {
            "student_view_validation_rmse_max": 0.01,
            "student_view_validation_abs_error_max": 0.10,
            "teacher_view_validation_rmse_max": 0.005,
            "teacher_view_validation_abs_error_max": 0.05,
            "student_validation_rmse_reduction_vs_source_h0_min_fraction": 0.50,
            "corpus_outputs_finite": True,
            "corpus_output_bounds_inclusive": [-1.0, 1.0],
            "logstd_parameter_and_output_bit_exact": True,
            "non_actor_state_bit_exact": True,
            "save_reload_actor_bit_exact": True,
            "candidate_actor_digest_must_differ_from_h0": True,
            "failure_terminal_outcome": "FAIL_H0_PRIMARY_SPLIT_OFFLINE_ADAPTATION",
            "failure_allows_retry": False,
        },
        "closed_loop_qualification_protocol": {
            "windows_unused_by_fit": True,
            "declared_offset_from_plateau_start_s": 12.0,
            "offset_from_paired_window_start_s": 10.0,
            "windows": {
                "02": {
                    "speed_mps": 0.95,
                    "absolute_start_time_s": 129.578,
                    "setup_offset_s": 117.880,
                    "seed": 126,
                },
                "04": {
                    "speed_mps": 1.05,
                    "absolute_start_time_s": 132.189,
                    "setup_offset_s": 117.550,
                    "seed": 127,
                },
                "08": {
                    "speed_mps": 1.25,
                    "absolute_start_time_s": 130.390,
                    "setup_offset_s": 116.878,
                    "seed": 128,
                },
            },
            "modes_per_window": {
                "deterministic": {"sigma": 0.0, "noise_tape": "ALL_ZERO"},
                "stochastic": {
                    "sigma": 0.005,
                    "noise_tape": "PREREGISTERED_STANDARD_NORMAL",
                },
            },
            "teacher_reference_case_count": 6,
            "candidate_case_count": 6,
            "teacher_reference_served_action": "H0_teacher_mean+sigma*z",
            "candidate_served_action": "candidate_student_mean+sigma*z",
            "execution_order": "ALL_SIX_TEACHER_REFERENCES_THEN_SIX_CANDIDATES",
            "candidate_requires_all_teacher_references_pass": True,
            "condition_matched_same_noise_tape": True,
            "action_replay_allowed": False,
            "candidate_cases_autonomous": True,
            "common_physical_gate": "teacher_rollout_gate",
            "counterfactual_candidate_vs_teacher_rmse_max": 0.015,
            "counterfactual_candidate_vs_teacher_abs_error_max": 0.10,
            "served_action_teacher_dependency_count_exact": 0,
            "reserve_nonregression": {
                "metrics": ["rms", "abs_max"],
                "rule": "candidate<=reference+max(5_percent,5_Nm)",
            },
            "residual_nonregression": {
                "metrics": ["rms", "abs_max"],
                "rule": "candidate<=reference+max(5_percent,1e-6_Nm)",
            },
            "sea_continuous_nonregression": {
                "metrics": ["rms", "abs_max"],
                "scope": "EACH_CONTINUOUS_SEA_METRIC",
                "rule": "candidate<=reference+max(5_percent,1e-6_metric_unit)",
            },
            "count_metrics_nonregression": {
                "metrics": [
                    "saturation",
                    "fallback",
                    "timeout",
                    "hard_invalid",
                    "safety_stop",
                ],
                "rule": "candidate<=reference_no_tolerance",
            },
            "all_six_candidate_cases_must_pass": True,
            "stop_at_first_failure": True,
            "failure_terminal_outcome": "FAIL_H0_PRIMARY_SPLIT_CLOSED_LOOP",
            "qualification_authorized": False,
        },
        "zero_iteration_port_protocol": {
            "reachable_only_after_all_closed_loop_gates_pass": True,
            "port_method": "warm-start-raw",
            "target_trainer": "NEW_ZERO_ITERATION_TRAINER",
            "actor_learner_envrunner_export_bit_exact": True,
            "critic_and_optimizer": "FRESH_NO_H0_RESTORE",
            "save_reload_zero_update_required": True,
            "critic_warmup_authorized": False,
            "critic_updates_authorized": False,
            "ppo_updates_authorized": False,
            "failure_terminal_outcome": "ERROR_H0_PRIMARY_SPLIT_ZERO_ITER_PORT",
            "execution_authorized": False,
        },
        "artifact_contract": {
            "json_strict_no_nan_inf": True,
            "json_atomic_no_clobber": True,
            "dataset_tape_trace_summary_manifest_receipt_ledger_no_clobber": True,
            "npz_and_checkpoint_sha256_and_size_in_atomic_manifest": True,
            "existing_h0_and_profiles_overwrite_allowed": False,
        },
        "terminal_outcomes": {
            "teacher_failure": "ERROR_PRIMARY_SPLIT_TEACHER",
            "offline_failure": "FAIL_H0_PRIMARY_SPLIT_OFFLINE_ADAPTATION",
            "closed_loop_failure": "FAIL_H0_PRIMARY_SPLIT_CLOSED_LOOP",
            "zero_iteration_port_error": "ERROR_H0_PRIMARY_SPLIT_ZERO_ITER_PORT",
            "maximum_success": "H0_PRIMARY_GRF_SPLIT_V1_BASELINE_READY",
            "success_opens_v25_abc_h0_sep_protected_ppo_or_runtime": False,
        },
        "unmet_execution_prerequisites": {
            "teacher_shadow_fsm_source_and_tests_hash_frozen": False,
            "paired_collector_source_and_tests_hash_frozen": False,
            "one_shot_fit_source_and_tests_hash_frozen": False,
            "qualification_driver_source_and_tests_hash_frozen": False,
            "qualification_noise_tapes_materialized_and_hash_frozen": False,
            "new_empty_output_destinations_frozen": False,
            "restore_and_save_reload_preflight_receipt_present": False,
            "separate_adaptation_execution_unlock_present": False,
        },
        "interstage_gate_before_actor_update": {
            "status": "CLOSED_PROTOCOL_ONLY",
            "requirements_currently_met": {
                "paired_corpus_materialized_and_hash_frozen": False,
                "teacher_collection_manifest_strict_atomic_no_clobber": False,
                "all_paired_and_reference_teacher_gates_pass_receipt_present": False,
            },
            "actor_update_may_start": False,
            "future_rule": "ALL_REQUIREMENTS_MUST_BE_TRUE_BEFORE_FIRST_UPDATE",
        },
        "assertions": assertions,
        "freeze_script": source_record(Path(__file__)),
        "historical_protocol_or_terminal_result_rewritten": False,
        "next_stage": (
            "IMPLEMENT_AND_FREEZE_SEPARATE_PRIMARY_SPLIT_ADAPTATION_"
            "EXECUTION_UNLOCK_OR_STOP"
        ),
    }
    encode_json(payload)
    return payload


def _fsync_directory(path: Path) -> None:
    try:
        descriptor = os.open(path, os.O_RDONLY)
    except OSError:
        return
    try:
        os.fsync(descriptor)
    except OSError:
        pass
    finally:
        os.close(descriptor)


def write_json_exclusive(path: Path, payload: Mapping[str, Any]) -> Path:
    if os.path.lexists(path):
        raise H0PrimarySplitProtocolError(f"refusing to clobber: {path}")
    encoded = encode_json(payload)
    descriptor, temporary_raw = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent)
    )
    temporary = Path(temporary_raw)
    descriptor_open = True
    try:
        with os.fdopen(descriptor, "wb") as stream:
            descriptor_open = False
            stream.write(encoded)
            stream.flush()
            os.fsync(stream.fileno())
        if os.path.lexists(path):
            raise H0PrimarySplitProtocolError(f"refusing to clobber: {path}")
        try:
            os.link(temporary, path)
        except FileExistsError as exc:
            raise H0PrimarySplitProtocolError(f"refusing to clobber: {path}") from exc
        _fsync_directory(path.parent)
        return path
    finally:
        if descriptor_open:
            os.close(descriptor)
        try:
            temporary.unlink()
        except FileNotFoundError:
            pass


def preflight_unfrozen() -> dict[str, Any]:
    payload = build_payload(require_destination_unoccupied=True)
    return {
        "status": "H0_PRIMARY_SPLIT_PROTOCOL_READY_UNWRITTEN",
        "destination_unoccupied": True,
        "execution_authorized": False,
        "updates_authorized": False,
        "lock_record_if_frozen": payload_record(DESTINATION, payload),
        "protocol_payload": payload,
    }


def freeze_protocol() -> dict[str, Any]:
    payload = build_payload(require_destination_unoccupied=True)
    write_json_exclusive(DESTINATION, payload)
    return payload


def verify_frozen() -> dict[str, Any]:
    published = strict_json_load(DESTINATION)
    expected = build_payload(require_destination_unoccupied=False)
    if published != expected:
        raise H0PrimarySplitProtocolError("published protocol lock is not reproducible")
    return {
        "status": "PASS_H0_PRIMARY_SPLIT_PROTOCOL_LOCK_VERIFICATION",
        "lock": source_record(DESTINATION),
        "execution_authorized": False,
        "updates_authorized": False,
    }


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--check", action="store_true")
    mode.add_argument("--freeze", action="store_true")
    mode.add_argument("--verify", action="store_true")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        if args.check:
            result = preflight_unfrozen()
        elif args.freeze:
            result = freeze_protocol()
        else:
            result = verify_frozen()
    except Exception as exc:
        print(
            "H0 primary-split protocol freeze failed closed: "
            f"{type(exc).__name__}: {exc}",
            file=os.sys.stderr,
        )
        return 2
    print(json.dumps(result, indent=2, sort_keys=True, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
