"""Freeze structural readiness of the dormant V25 shadow integration.

This gate is deliberately non-numerical.  It binds the frozen V25 profile to
the dormant runtime allowlist, the legacy analogue sensor-only route, the 1 ms
transport contract, and the unchanged 35/84 observation boundary.  It does
not run H0, compare A/B rollouts, expose ``binary_active``, or authorize any
subsequent stage.
"""

from __future__ import annotations

import argparse
import ast
import hashlib
import json
import math
import os
import sys
import tempfile
from pathlib import Path
from typing import Any, Mapping, Sequence


REPO_ROOT = Path(__file__).resolve().parents[1]
VALIDATION_ROOT = REPO_ROOT / "validation"
SCHEMA_VERSION = 1
GATE_ID = "AB06_V25_DORMANT_SHADOW_STRUCTURAL_READINESS"
DESTINATION = (
    VALIDATION_ROOT / "binary_phase_detector_v25_shadow_readiness_receipt.json"
)

V25_PROFILE_SHA256 = (
    "db704e502b99e49bea6d89493812bafdac748f8ce8d3ce28214ff624078539a2"
)
V25_LOCK_SHA256 = (
    "04ecfe68937bc0d4baa3be9ab9b62060b20eb92c2f218f8540db1cebe423d346"
)
LEGACY_ANALOG_PROFILE_SHA256 = (
    "61ea948a3c0613e5c0e684a3197de118c7116e36188fca6993da79ce713fd99e"
)
HISTORICAL_FSM_CONTRACT_ID = (
    "binary_point_v19+functional_contact_fsm_v1_shadow"
)
RUNTIME_V25_CONTRACT_ID = (
    "binary_point_v25+functional_contact_fsm_v1_shadow"
)
SCIENTIFIC_SHADOW_BUNDLE_ID = (
    "primary_grf_split_v1+binary_point_v25+functional_contact_fsm_v1_shadow"
)
TARGET_ACTIVE_BUNDLE_ID = (
    "primary_grf_split_v1+binary_point_v25+functional_contact_fsm_v1"
)
HISTORICAL_RUNNER_SHA256 = (
    "f609533d69bb056e7eeca1933f934b308df8d5d9d3725387ed1027645fb08a43"
)

V25_RUN = (
    VALIDATION_ROOT
    / "binary_phase_detector_v25_geometry_runs"
    / "2026-08-04_local_reach_sweep_dev02_04_08"
)

PINNED: dict[str, tuple[Path, str]] = {
    "v25_candidate_lock": (
        VALIDATION_ROOT
        / "binary_phase_detector_v25_development_candidate_freeze_lock.json",
        V25_LOCK_SHA256,
    ),
    "v25_profile": (
        V25_RUN / "selected_candidate_profile.json",
        V25_PROFILE_SHA256,
    ),
    "legacy_analog_profile": (
        REPO_ROOT
        / "online_grf_profiles"
        / "AB06_SEASEA_stiff321_500_pi_grf_detector_HS-TO.json",
        LEGACY_ANALOG_PROFILE_SHA256,
    ),
    "binary_detector_source": (
        REPO_ROOT / "binary_phase_detector.py",
        "57a313133e1ce5a675b2699e940226325dfa5b2b895c7eb6b17c0892a94263b6",
    ),
    "fsm_v20_source": (
        REPO_ROOT / "Trajectory Generator" / "binary_phase_fsm.py",
        "0f7669b60a72c1b27ee3c4f1a43161eeb9f2d091dff5558cc4fa43f1fce8d9c1",
    ),
    "environment_source": (
        REPO_ROOT / "Trajectory Generator" / "osim_trj_cmc_like.py",
        "6a6847291dc56431f6263ebd8f02c3edceed2d8d657cfde1e94242ab508d95e3",
    ),
    "model_loader_source": (
        REPO_ROOT / "model_loader.py",
        "401beddc52e2dd8ce4a88208cf5b38b036232cb1bc3ea37704e467852f2ace12",
    ),
    "runner_source": (
        REPO_ROOT / "simulation_runner.py",
        "9851d41668fd9a4e16a5761837ed682824bef91a7f07ea46542a0d4b45fd3f18",
    ),
    "simulator_config_source": (
        REPO_ROOT / "config.py",
        "88c120bdf8249143a78cd19a33a4de34c10d4230a2ad6760b33dec9bb51417e3",
    ),
    "path_resolver_source": (
        REPO_ROOT / "path_resolver.py",
        "2a61b8c54ab68c228ea55a6e28b6334ac0a9539da79f86e9a5ef2970bc937a1d",
    ),
    "training_yaml": (
        REPO_ROOT
        / "Trajectory Generator"
        / "baseline_MLP"
        / "training_exnovo_cfg.yaml",
        "5d0930e6317330e714fc26231f82e6d062e0ac7c7877e019d21541cf522b0c60",
    ),
    "training_config_source": (
        REPO_ROOT
        / "Trajectory Generator"
        / "baseline_MLP"
        / "training_config.py",
        "cbc14abec0f696bd3135a459ab6a4b1cab3daf2128178602983943ceeabdfbc2",
    ),
    "training_cli_source": (
        REPO_ROOT
        / "Trajectory Generator"
        / "baseline_MLP"
        / "train_ppo_mlp.py",
        "d94d1ed9c2d9b4af3e06214949929a87b2ca60be026f2ba1791ac7c6703d70cf",
    ),
    "rollout_cli_source": (
        REPO_ROOT
        / "Trajectory Generator"
        / "baseline_MLP"
        / "rollout_eval.py",
        "8b2972368eca069305d125546b4e20f90b1e69a0fc7099f72c817fcd60954815",
    ),
    "shadow_integration_test": (
        VALIDATION_ROOT / "test_binary_phase_fsm_env_v20.py",
        "5c9a9d54680d6cc25c7640fc91b643452d118bdc1efa24d64a9edc425ffe5e82",
    ),
    "runner_contract_test": (
        VALIDATION_ROOT / "test_binary_phase_detector_v19.py",
        "f9d5d50791ed63c1f620504d71abe0cc64b619fd8c790517bebe59e6d2a83cd4",
    ),
    "fsm_contract_test": (
        VALIDATION_ROOT / "test_binary_phase_fsm_v20.py",
        "7f88b17ed7fcfb5d4d0ee10557131156d0aeb1b7c167cf68d31208714c8bad7f",
    ),
    "detector_data_path_test": (
        VALIDATION_ROOT / "test_detector_sensor_data_path.py",
        "9223a8888ea3529eeedaf02a17c294398ebd22232625d92122e577991242e6f5",
    ),
    "runner_contract_receipt": (
        VALIDATION_ROOT / "binary_phase_detector_v19_geometry_receipt.json",
        "7d93ef5f5b5c877246025c0f6d7a607783621e7019065ca63b4b889f5a30e0ff",
    ),
    "layout_evidence": (
        VALIDATION_ROOT
        / "controller_memory_ablation"
        / "2026-07-13_markov35_corrected_full_sigma0005_seed123"
        / "rollout_summary.json",
        "70676e7d2b4e8c1bca185fe23d1d29f96fa83ac34167ba916250eff6e96f601e",
    ),
    "online_grf_source_untouched": (
        REPO_ROOT / "online_grf.py",
        "52e39bf9a3b20dd65242f3f9076d76ed788239fe7c3e5b825bc37a9657c4fefa",
    ),
}


class V25ShadowReadinessError(RuntimeError):
    """Raised when structural shadow readiness cannot be frozen exactly."""


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def _reject_json_constant(value: str) -> None:
    raise V25ShadowReadinessError(f"non-finite JSON constant: {value}")


def _reject_duplicate_pairs(pairs: Sequence[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise V25ShadowReadinessError(f"duplicate JSON object key: {key!r}")
        result[key] = value
    return result


def _finite_tree(value: object) -> bool:
    if value is None or isinstance(value, (bool, int, str)):
        return True
    if isinstance(value, float):
        return math.isfinite(value)
    if isinstance(value, Mapping):
        return all(isinstance(key, str) and _finite_tree(item) for key, item in value.items())
    if isinstance(value, (list, tuple)):
        return all(_finite_tree(item) for item in value)
    return False


def strict_json_load(path: Path, expected_sha256: str | None = None) -> dict[str, Any]:
    if not path.is_file():
        raise V25ShadowReadinessError(f"JSON artifact is missing: {path}")
    if expected_sha256 is not None and sha256_file(path) != expected_sha256:
        raise V25ShadowReadinessError(f"pinned JSON artifact drifted: {path}")
    try:
        payload = json.loads(
            path.read_text(encoding="utf-8"),
            object_pairs_hook=_reject_duplicate_pairs,
            parse_constant=_reject_json_constant,
        )
    except V25ShadowReadinessError:
        raise
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise V25ShadowReadinessError(f"cannot parse strict JSON: {path}") from exc
    if not isinstance(payload, dict):
        raise V25ShadowReadinessError(f"JSON root is not an object: {path}")
    if not _finite_tree(payload):
        raise V25ShadowReadinessError(f"JSON tree contains a non-finite value: {path}")
    return payload


def source_record(path: Path) -> dict[str, Any]:
    resolved = path.resolve()
    if not resolved.is_file():
        raise V25ShadowReadinessError(f"source is missing: {resolved}")
    try:
        relative = resolved.relative_to(REPO_ROOT.resolve()).as_posix()
    except ValueError as exc:
        raise V25ShadowReadinessError(f"source escapes repository: {resolved}") from exc
    return {
        "path": relative,
        "sha256": sha256_file(resolved),
        "size_bytes": int(resolved.stat().st_size),
    }


def encode_json(payload: Mapping[str, Any]) -> bytes:
    if not _finite_tree(payload):
        raise V25ShadowReadinessError("refusing to encode a non-finite payload")
    return (
        json.dumps(dict(payload), indent=2, sort_keys=True, allow_nan=False) + "\n"
    ).encode("utf-8")


def payload_record(path: Path, payload: Mapping[str, Any]) -> dict[str, Any]:
    encoded = encode_json(payload)
    return {
        "path": path.resolve().relative_to(REPO_ROOT.resolve()).as_posix(),
        "sha256": sha256_bytes(encoded),
        "size_bytes": len(encoded),
    }


def _literal_assignment(path: Path, name: str, class_name: str | None = None) -> Any:
    try:
        tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    except (OSError, UnicodeError, SyntaxError) as exc:
        raise V25ShadowReadinessError(f"cannot parse Python source: {path}") from exc
    nodes: Sequence[ast.stmt] = tree.body
    if class_name is not None:
        classes = [
            node
            for node in tree.body
            if isinstance(node, ast.ClassDef) and node.name == class_name
        ]
        if len(classes) != 1:
            raise V25ShadowReadinessError(
                f"expected one class {class_name!r} in {path}"
            )
        nodes = classes[0].body
    matches: list[ast.AST] = []
    for node in nodes:
        if isinstance(node, ast.Assign) and any(
            isinstance(target, ast.Name) and target.id == name for target in node.targets
        ):
            matches.append(node.value)
        elif (
            isinstance(node, ast.AnnAssign)
            and isinstance(node.target, ast.Name)
            and node.target.id == name
            and node.value is not None
        ):
            matches.append(node.value)
    if len(matches) != 1:
        raise V25ShadowReadinessError(
            f"expected one literal assignment {name!r} in {path}"
        )
    try:
        return ast.literal_eval(matches[0])
    except (ValueError, TypeError) as exc:
        raise V25ShadowReadinessError(
            f"assignment {name!r} is not a static literal in {path}"
        ) from exc


def _function_source(path: Path, class_name: str, function_name: str) -> str:
    text = path.read_text(encoding="utf-8")
    tree = ast.parse(text, filename=str(path))
    for node in tree.body:
        if not isinstance(node, ast.ClassDef) or node.name != class_name:
            continue
        matches = [
            item
            for item in node.body
            if isinstance(item, (ast.FunctionDef, ast.AsyncFunctionDef))
            and item.name == function_name
        ]
        if len(matches) != 1:
            break
        segment = ast.get_source_segment(text, matches[0])
        if segment is not None:
            return segment
    raise V25ShadowReadinessError(
        f"cannot isolate {class_name}.{function_name} in {path}"
    )


def _verify_pins() -> tuple[dict[str, dict[str, Any]], dict[str, dict[str, Any]]]:
    records: dict[str, dict[str, Any]] = {}
    json_payloads: dict[str, dict[str, Any]] = {}
    for label, (path, expected_sha256) in PINNED.items():
        record = source_record(path)
        if record["sha256"] != expected_sha256:
            raise V25ShadowReadinessError(f"pinned artifact drifted: {label}")
        records[label] = record
        if path.suffix == ".json":
            json_payloads[label] = strict_json_load(path, expected_sha256)
    return records, json_payloads


def _structural_assertions(
    records: Mapping[str, Mapping[str, Any]],
    payloads: Mapping[str, Mapping[str, Any]],
) -> dict[str, bool]:
    env_path = PINNED["environment_source"][0]
    loader_path = PINNED["model_loader_source"][0]
    runner_path = PINNED["runner_source"][0]
    env_text = env_path.read_text(encoding="utf-8")
    loader_text = loader_path.read_text(encoding="utf-8")
    runner_text = runner_path.read_text(encoding="utf-8")
    training_text = PINNED["training_yaml"][0].read_text(encoding="utf-8")
    rollout_text = PINNED["rollout_cli_source"][0].read_text(encoding="utf-8")
    training_cli_text = PINNED["training_cli_source"][0].read_text(encoding="utf-8")

    allowlist = _literal_assignment(env_path, "_BINARY_PHASE_SHADOW_PROFILE_ALLOWLIST")
    default_mode = _literal_assignment(env_path, "binary_phase_fsm_mode", "CMCEnvConfig")
    default_runtime_contract = _literal_assignment(
        env_path, "binary_phase_event_contract_id", "CMCEnvConfig"
    )
    loader_v25_sha = _literal_assignment(
        loader_path, "_V25_BINARY_SHADOW_PROFILE_SHA256"
    )
    loader_analog_sha = _literal_assignment(
        loader_path, "_V25_LEGACY_ANALOG_DETECTOR_PROFILE_SHA256"
    )

    v25_lock = payloads["v25_candidate_lock"]
    profile = payloads["v25_profile"]
    runner_receipt = payloads["runner_contract_receipt"]
    layout = payloads["layout_evidence"]
    points = profile.get("points")
    lock_implementation = v25_lock.get("implementation", {})
    lock_contracts = v25_lock.get("contracts", {})
    runtime_transport = runner_receipt.get("runtime_transport", {})
    observation_source = _function_source(
        env_path, "CMCLikeProsthesisTrajectoryEnv", "_get_observation"
    )

    expected_v25_allowlist_value = (
        "V25",
        V25_PROFILE_SHA256,
        True,
    )
    exact_roles = (
        isinstance(points, list)
        and len(points) == 2
        and tuple(item.get("name") for item in points if isinstance(item, Mapping))
        == ("left_heel", "left_toe")
    )
    assertions = {
        "all_pins_exact": len(records) == len(PINNED),
        "v25_candidate_lock_exact": v25_lock.get("status")
        == "V25_DEVELOPMENT_CANDIDATE_FROZEN_H0_PROTOCOL_REQUIRED"
        and v25_lock.get("candidate", {}).get("candidate_id")
        == "v25_4b351f67b5b86ab0"
        and v25_lock.get("lifecycle", {}).get("runtime_promoted") is False,
        "v25_profile_exact_two_force_free_points": profile.get("detector_type")
        == "binary_point_clearance_v1"
        and profile.get("contact_rule", {}).get("contact_when")
        == "signed_clearance_le_zero"
        and exact_roles,
        "runtime_v25_allowlist_exact": isinstance(allowlist, dict)
        and allowlist.get(RUNTIME_V25_CONTRACT_ID) == expected_v25_allowlist_value,
        "runtime_and_bundle_contract_ids_distinct": RUNTIME_V25_CONTRACT_ID
        != SCIENTIFIC_SHADOW_BUNDLE_ID
        and not RUNTIME_V25_CONTRACT_ID.startswith("primary_grf_split_v1+")
        and SCIENTIFIC_SHADOW_BUNDLE_ID.startswith("primary_grf_split_v1+"),
        "historical_v19_to_v25_bridge_explicit": lock_contracts.get(
            "fsm_implementation_contract_id"
        )
        == HISTORICAL_FSM_CONTRACT_ID
        and lock_implementation.get("fsm_v20_source", {}).get("sha256")
        == records["fsm_v20_source"]["sha256"]
        and HISTORICAL_FSM_CONTRACT_ID in allowlist
        and RUNTIME_V25_CONTRACT_ID in allowlist,
        "legacy_analog_v25_coexistence_fail_closed": loader_v25_sha
        == V25_PROFILE_SHA256
        and loader_analog_sha == LEGACY_ANALOG_PROFILE_SHA256
        and LEGACY_ANALOG_PROFILE_SHA256 in loader_text
        and "_validate_binary_detector_coexistence" in loader_text
        and "online_grf_detector_force_paths" in loader_text
        and "binary_phase_detector_profile" in loader_text,
        "default_legacy_and_morphology_zero": "phase_fsm_input_mode: legacy_events"
        in training_text
        and "event_contract_id: legacy_events_v1" in training_text
        and "morphology_weight: 0.0" in training_text
        and default_mode == "disabled"
        and default_runtime_contract == "binary_events_disabled_v1",
        "cli_modes_are_disabled_or_shadow_only": 'choices=("disabled", "binary_shadow")'
        in rollout_text
        and 'choices=("disabled", "binary_shadow")' in training_cli_text
        and "binary_active" not in env_text
        and "binary_active" not in rollout_text
        and "binary_active" not in training_cli_text,
        "observation_layout_not_extended_by_shadow": "_binary_phase"
        not in observation_source
        and layout.get("n_actor") == 35
        and layout.get("n_observation") == 84
        and len(layout.get("actor_feature_names", [])) == 35
        and len(layout.get("observation_feature_names", [])) == 84,
        "runner_t0_and_ten_sample_contract": runtime_transport.get("status")
        == "PASS"
        and runtime_transport.get("t0_sampled_but_not_emitted") is True
        and runtime_transport.get("normal_10ms_step_sample_count") == 10
        and runtime_transport.get("interval_semantics")
        == "(previous_sample_time, t_stop]"
        and runner_receipt.get("implementation_sources", {}).get(
            "simulation_runner.py"
        )
        == HISTORICAL_RUNNER_SHA256
        and "_binary_phase_sensor_baseline" in runner_text
        and "_finalize_binary_phase_sensor_segment" in runner_text,
        "legacy_analog_sensor_only_separate_from_primary": (
            "online_grf_detector_force_paths" in loader_text
            and "applies_force=False" in loader_text
            and records["legacy_analog_profile"]["sha256"]
            == LEGACY_ANALOG_PROFILE_SHA256
        ),
        "legacy_analog_four_roles_exact": all(
            role in env_text
            for role in ("left_heel", "left_toe", "right_heel", "right_toe")
        )
        and "len(analog_paths) != 4" in env_text
        and "exactly four analog legacy roles" in env_text,
        "no_h0_or_numerical_gate_claim": True,
    }
    return assertions


def build_receipt_payload(*, require_destination_unoccupied: bool = True) -> dict[str, Any]:
    if require_destination_unoccupied and os.path.lexists(DESTINATION):
        raise V25ShadowReadinessError(f"refusing to clobber: {DESTINATION}")
    records, payloads = _verify_pins()
    assertions = _structural_assertions(records, payloads)
    if not all(assertions.values()):
        raise V25ShadowReadinessError(
            f"V25 shadow structural readiness failed: {assertions}"
        )
    receipt = {
        "schema_version": SCHEMA_VERSION,
        "gate_id": GATE_ID,
        "date": "2026-08-04",
        "status": "V25_SHADOW_STRUCTURALLY_READY_NUMERICAL_AB_UNRUN",
        "structural_readiness": True,
        "numerical_ab_pass_claimed": False,
        "h0_compatibility_claimed": False,
        "candidate": {
            "candidate_id": "v25_4b351f67b5b86ab0",
            "profile": records["v25_profile"],
            "candidate_freeze_lock": records["v25_candidate_lock"],
            "legacy_analog_profile": records["legacy_analog_profile"],
        },
        "contracts": {
            "raw_detector_contract_id": "binary_point_clearance_v1",
            "historical_freeze_fsm_contract_id": HISTORICAL_FSM_CONTRACT_ID,
            "runtime_binary_phase_event_contract_id": RUNTIME_V25_CONTRACT_ID,
            "scientific_shadow_bundle_contract_id": SCIENTIFIC_SHADOW_BUNDLE_ID,
            "target_active_bundle_contract_id": TARGET_ACTIVE_BUNDLE_ID,
            "fsm_geometry_agnostic": True,
            "fsm_source_unchanged_from_historical_freeze": True,
            "historical_freeze_reinterpreted_or_rewritten": False,
            "sample_dt_s": 0.001,
            "policy_step_s": 0.01,
            "samples_per_normal_policy_step": 10,
            "t0_is_non_event_baseline": True,
            "debounce_s": 0.005,
        },
        "dormant_topology": {
            "legacy_analog_loaded": True,
            "legacy_analog_sensor_only": True,
            "legacy_analog_applies_force": False,
            "legacy_analog_distinct_from_primary_grf": True,
            "legacy_analog_roles": [
                "left_heel",
                "left_toe",
                "right_heel",
                "right_toe",
            ],
            "v25_loaded_and_sampled": True,
            "v25_adds_force_or_model_component": False,
            "authoritative_event_source": "legacy_events",
            "binary_phase_fsm_mode": "binary_shadow",
            "morphology_weight": 0.0,
            "actor_observation_count": 35,
            "full_observation_count": 84,
        },
        "implementation": {
            key: records[key]
            for key in (
                "binary_detector_source",
                "fsm_v20_source",
                "environment_source",
                "model_loader_source",
                "runner_source",
                "simulator_config_source",
                "path_resolver_source",
                "training_yaml",
                "training_config_source",
                "training_cli_source",
                "rollout_cli_source",
                "online_grf_source_untouched",
            )
        },
        "test_and_static_evidence": {
            key: records[key]
            for key in (
                "shadow_integration_test",
                "runner_contract_test",
                "fsm_contract_test",
                "detector_data_path_test",
                "runner_contract_receipt",
                "layout_evidence",
            )
        },
        "runner_transport_bridge": {
            "historical_receipt": records["runner_contract_receipt"],
            "historical_runner_sha256_recorded_by_receipt": HISTORICAL_RUNNER_SHA256,
            "current_runner": records["runner_source"],
            "current_transport_test": records["shadow_integration_test"],
            "historical_transport_contract_preserved": True,
            "current_delta": "disabled_publication_guard",
            "historical_receipt_reinterpreted_or_rewritten": False,
        },
        "session_test_attestation": {
            "interpreter": "/opt/anaconda3/envs/envCMC-like/bin/python",
            "command_argv": [
                "/opt/anaconda3/envs/envCMC-like/bin/python",
                "-m",
                "unittest",
                "validation/test_binary_phase_detector_v19.py",
                "validation/test_binary_phase_fsm_v20.py",
                "validation/test_binary_phase_fsm_env_v20.py",
                "validation/test_detector_sensor_data_path.py",
            ],
            "tests_run": 65,
            "failures": 0,
            "errors": 0,
            "skipped": 0,
            "status": "PASS_SESSION_ATTESTATION",
            "independently_reproduced_by_this_static_gate": False,
            "numerical_h0_or_ab_rollouts_included": False,
        },
        "assertions": assertions,
        "authority": {
            "execution_authorized": False,
            "h0_executed": False,
            "binary_active_available": False,
            "training_authorized": False,
            "ppo_updates_authorized": False,
            "h0_sep_authorized": False,
            "protected_trial_access_authorized": False,
            "reserve_trial_access_authorized": False,
            "corridor_authorized": False,
            "runtime_promotion_authorized": False,
            "primary_grf_modification_authorized": False,
            "detector_retuning_authorized": False,
        },
        "freeze_script": source_record(Path(__file__)),
        "next_stage": "FREEZE_NON_EXECUTABLE_H0_V25_ABC_PROTOCOL",
    }
    encode_json(receipt)
    return receipt


def preflight_unfrozen() -> dict[str, Any]:
    receipt = build_receipt_payload(require_destination_unoccupied=True)
    return {
        "status": "V25_SHADOW_STRUCTURAL_RECEIPT_READY_UNWRITTEN",
        "destination_unoccupied": True,
        "receipt_record_if_frozen": payload_record(DESTINATION, receipt),
        "receipt_payload": receipt,
    }


def _fsync_directory(path: Path) -> None:
    flags = os.O_RDONLY | getattr(os, "O_DIRECTORY", 0)
    try:
        descriptor = os.open(str(path), flags)
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
        raise V25ShadowReadinessError(f"refusing to clobber: {path}")
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
            raise V25ShadowReadinessError(f"refusing to clobber: {path}")
        try:
            os.link(temporary, path)
        except FileExistsError as exc:
            raise V25ShadowReadinessError(f"refusing to clobber: {path}") from exc
        _fsync_directory(path.parent)
        return path
    finally:
        if descriptor_open:
            os.close(descriptor)
        try:
            temporary.unlink()
        except FileNotFoundError:
            pass


def freeze_readiness() -> dict[str, Any]:
    receipt = build_receipt_payload(require_destination_unoccupied=True)
    write_json_exclusive(DESTINATION, receipt)
    return receipt


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--check", action="store_true")
    mode.add_argument("--freeze", action="store_true")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        payload = preflight_unfrozen() if args.check else freeze_readiness()
    except Exception as exc:
        print(
            "V25 shadow structural readiness failed closed: "
            f"{type(exc).__name__}: {exc}",
            file=sys.stderr,
        )
        return 2
    print(json.dumps(payload, indent=2, sort_keys=True, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
