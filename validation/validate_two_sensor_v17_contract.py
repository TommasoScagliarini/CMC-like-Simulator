"""Fail-closed static validation for the V17 detector and primary-core lock.

This validator never imports OpenSim, never runs the simulator, and never
opens trial data.  It checks immutable core hashes, V17's exact two-sphere
geometry/configuration, and the currently selected hybrid routing by parsing
the loader/configuration sources without modifying them.
"""

from __future__ import annotations

import argparse
import ast
import json
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path, PurePosixPath, PureWindowsPath
from typing import Any, Mapping, Sequence

import yaml


REPO_ROOT = Path(__file__).resolve().parents[1]
VALIDATION_ROOT = Path(__file__).resolve().parent
for _path in (REPO_ROOT, VALIDATION_ROOT):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

import readiness_gatekeeper as gate  # noqa: E402


EVENT_CONTRACT_ID = "primary_grf_split_v1+two_sensor_highrate_v1"
DEFAULT_V17_CONTRACT = REPO_ROOT / "validation/two_sensor_v17_high_rate_contract.json"
DEFAULT_PRIMARY_LOCK = REPO_ROOT / "validation/primary_grf_core_lock_2026-08-03.json"
EXPECTED_GEOMETRY = {
    "left_heel": {
        "side": "left",
        "frame": "/bodyset/foot_l",
        "location": [-0.0946600475, -0.0353195377, 0.01399567],
        "radius": 0.0229053623,
    },
    "left_toe": {
        "side": "left",
        "frame": "/bodyset/foot_l",
        "location": [0.11574858501553537, -0.051879237903540584, 0.0030479021621026177],
        "radius": 0.0229053623,
    },
}


class V17ContractError(ValueError):
    """Raised when the detector, primary lock, or routing contract drifts."""


def _strict_object(path: Path, label: str) -> dict[str, Any]:
    value = gate.load_json_strict(path)
    if not isinstance(value, Mapping):
        raise V17ContractError(f"{label} must be a JSON object")
    return dict(value)


def _repo_path(value: Any, *, label: str) -> Path:
    if not isinstance(value, str) or not value.strip() or value != value.strip():
        raise V17ContractError(f"{label} must be a trimmed non-empty path")
    if "\\" in value or ":" in value:
        raise V17ContractError(f"{label} must use portable '/' separators")
    posix = PurePosixPath(value)
    windows = PureWindowsPath(value)
    if posix.is_absolute() or windows.is_absolute() or windows.drive:
        raise V17ContractError(f"{label} must be repository-relative")
    if not posix.parts or any(part in {"", ".", ".."} for part in posix.parts):
        raise V17ContractError(f"{label} contains unsafe components")
    path = REPO_ROOT.joinpath(*posix.parts).resolve()
    try:
        path.relative_to(REPO_ROOT)
    except ValueError as exc:
        raise V17ContractError(f"{label} escapes the repository") from exc
    return path


def _assert_close(observed: Any, expected: Any, label: str) -> None:
    if isinstance(expected, list):
        if not isinstance(observed, list) or len(observed) != len(expected):
            raise V17ContractError(f"{label} shape drifted")
        for index, (actual, target) in enumerate(zip(observed, expected)):
            _assert_close(actual, target, f"{label}[{index}]")
        return
    if isinstance(expected, float):
        try:
            actual_float = float(observed)
        except (TypeError, ValueError) as exc:
            raise V17ContractError(f"{label} must be numeric") from exc
        if not math.isfinite(actual_float) or actual_float != expected:
            raise V17ContractError(
                f"{label} drifted: expected {expected!r}, observed {observed!r}"
            )
        return
    if observed != expected:
        raise V17ContractError(
            f"{label} drifted: expected {expected!r}, observed {observed!r}"
        )


def validate_v17_profile(contract: Mapping[str, Any]) -> dict[str, Any]:
    candidate = contract.get("candidate")
    if not isinstance(candidate, Mapping):
        raise V17ContractError("V17 candidate contract is missing")
    if contract.get("event_contract_id") != EVENT_CONTRACT_ID:
        raise V17ContractError("V17 event contract id drifted")
    expected_candidate = {
        "sensor_count": 2,
        "roles": ["left_heel", "left_toe"],
        "applies_force": False,
        "geometry_locked_to_v13": True,
        "sensor_on_threshold_n": 0.5,
        "sensor_off_threshold_n": 0.25,
        "sensor_dwell_s": 0.03,
        "detector_sample_dt_s": 0.001,
        "policy_step_s": 0.01,
    }
    for key, expected in expected_candidate.items():
        _assert_close(candidate.get(key), expected, f"candidate.{key}")
    profile_path = _repo_path(candidate.get("profile_path"), label="candidate.profile_path")
    profile = _strict_object(profile_path, "V17 profile")
    spheres = profile.get("spheres")
    if not isinstance(spheres, list) or len(spheres) != 2:
        raise V17ContractError("V17 profile must contain exactly two spheres")
    by_name = {
        str(sphere.get("name")): sphere
        for sphere in spheres
        if isinstance(sphere, Mapping)
    }
    if set(by_name) != set(EXPECTED_GEOMETRY):
        raise V17ContractError("V17 profile roles must be exactly heel and toe")
    for name, expected in EXPECTED_GEOMETRY.items():
        sphere = by_name[name]
        for key, target in expected.items():
            _assert_close(sphere.get(key), target, f"{name}.{key}")

    metadata = profile.get("metadata")
    if not isinstance(metadata, Mapping):
        raise V17ContractError("V17 profile metadata is missing")
    if metadata.get("event_contract_id") != EVENT_CONTRACT_ID:
        raise V17ContractError("V17 profile event contract id drifted")
    if metadata.get("status") != "development_candidate_not_runtime_or_training_promoted":
        raise V17ContractError("V17 profile has an unsafe promotion status")
    sampling = metadata.get("detector_sampling")
    if not isinstance(sampling, Mapping):
        raise V17ContractError("V17 detector sampling metadata is missing")
    for key, expected in {
        "sample_dt_s": 0.001,
        "policy_step_s": 0.01,
        "sensor_on_threshold_n": 0.5,
        "sensor_off_threshold_n": 0.25,
        "sensor_dwell_s": 0.03,
    }.items():
        _assert_close(sampling.get(key), expected, f"metadata.detector_sampling.{key}")

    comparator_path = _repo_path(
        metadata.get("v13_comparator", {}).get("path"),
        label="metadata.v13_comparator.path",
    )
    comparator = _strict_object(comparator_path, "V13 comparator")
    comparator_spheres = comparator.get("spheres")
    if not isinstance(comparator_spheres, list) or len(comparator_spheres) != 2:
        raise V17ContractError("V13 comparator no longer has exactly two spheres")
    comparator_by_name = {
        str(sphere.get("name")): sphere
        for sphere in comparator_spheres
        if isinstance(sphere, Mapping)
    }
    for name, expected in EXPECTED_GEOMETRY.items():
        if name not in comparator_by_name:
            raise V17ContractError(f"V13 comparator lost {name}")
        for key, target in expected.items():
            _assert_close(comparator_by_name[name].get(key), target, f"V13.{name}.{key}")
    if profile.get("ground") != comparator.get("ground"):
        raise V17ContractError("V17 ground differs from V13")
    if profile.get("material") != comparator.get("material"):
        raise V17ContractError("V17 material differs from V13")
    return {
        "profile_path": profile_path.relative_to(REPO_ROOT).as_posix(),
        "profile_sha256": gate.sha256_file(profile_path),
        "sensor_count": 2,
        "geometry_exact_v13": True,
        "not_promoted": True,
    }


def validate_primary_core_lock(lock: Mapping[str, Any]) -> dict[str, Any]:
    if lock.get("status") != "PRIMARY_CONTRACT_FROZEN_LIMITED_HYBRID_CLAIM":
        raise V17ContractError("primary lock status drifted")
    core = lock.get("scientific_core")
    if not isinstance(core, Mapping) or not core:
        raise V17ContractError("primary scientific core is missing")
    observed: dict[str, Any] = {}
    for name, raw_record in core.items():
        if not isinstance(raw_record, Mapping):
            raise V17ContractError(f"primary core record {name} is invalid")
        path = _repo_path(raw_record.get("path"), label=f"scientific_core.{name}.path")
        expected_hash = raw_record.get("sha256")
        if not gate.is_sha256(expected_hash):
            raise V17ContractError(f"primary core {name} hash is invalid")
        actual_hash = gate.sha256_file(path)
        if actual_hash != expected_hash:
            raise V17ContractError(
                f"primary core {name} hash mismatch: expected {expected_hash}, "
                f"observed {actual_hash}"
            )
        observed[name] = {"path": raw_record["path"], "sha256": actual_hash}
    attestation = lock.get("platform_attestations", {}).get("macos_arm64_dylib")
    if not isinstance(attestation, Mapping):
        raise V17ContractError("macOS dylib attestation is missing")
    dylib = _repo_path(attestation.get("path"), label="macos_arm64_dylib.path")
    if gate.sha256_file(dylib) != attestation.get("sha256"):
        raise V17ContractError("macOS online-GRF dylib attestation drifted")
    exclusions = set(lock.get("scientific_exclusions", ()))
    required_exclusions = {
        "COP representation or sentinel at zero load",
        "technical left-only primary-profile loader option",
        "detector geometry, thresholds, debounce or event timing",
        "full-wrench production certification",
    }
    if not required_exclusions.issubset(exclusions):
        raise V17ContractError("primary scientific exclusions are incomplete")
    routing = lock.get("routing_contract")
    if not isinstance(routing, Mapping) or not routing.get(
        "python_and_yaml_hashes_are_deliberately_not_part_of_scientific_core"
    ):
        raise V17ContractError("primary lock incorrectly expands the scientific core")
    return {
        "core_records": observed,
        "macos_binary_attested": True,
        "claim": "hybrid_left_online_right_prescribed_only",
    }


def _call_name(call: ast.Call) -> str:
    if isinstance(call.func, ast.Name):
        return call.func.id
    if isinstance(call.func, ast.Attribute):
        return call.func.attr
    return ""


def _keyword(call: ast.Call, name: str) -> ast.AST | None:
    return next((item.value for item in call.keywords if item.arg == name), None)


def validate_loader_routing(loader_path: Path) -> dict[str, Any]:
    try:
        tree = ast.parse(loader_path.read_text(encoding="utf-8"), filename=str(loader_path))
    except (OSError, SyntaxError) as exc:
        raise V17ContractError(f"cannot parse model loader: {exc}") from exc
    setup_model = next(
        (
            node
            for node in tree.body
            if isinstance(node, ast.FunctionDef) and node.name == "setup_model"
        ),
        None,
    )
    if setup_model is None:
        raise V17ContractError("model_loader.setup_model is missing")
    has_auto_disable = any(
        isinstance(node, ast.AugAssign)
        and isinstance(node.op, ast.BitOr)
        and isinstance(node.target, ast.Name)
        and node.target.id == "prescribed_grf_disabled_sides"
        and isinstance(node.value, ast.Name)
        and node.value.id == "online_grf_applied_sides"
        for node in ast.walk(setup_model)
    )
    if not has_auto_disable:
        raise V17ContractError("loader no longer auto-disables prescribed applied sides")
    calls = [
        node
        for node in ast.walk(setup_model)
        if isinstance(node, ast.Call) and _call_name(node) == "add_online_grf_forces"
    ]
    primary_calls = [
        call
        for call in calls
        if len(call.args) >= 2
        and isinstance(call.args[1], ast.Name)
        and call.args[1].id == "profile"
    ]
    detector_calls = [
        call
        for call in calls
        if len(call.args) >= 2
        and isinstance(call.args[1], ast.Name)
        and call.args[1].id == "detector_profile"
    ]
    if len(primary_calls) != 1 or len(detector_calls) != 1:
        raise V17ContractError("loader must have one primary and one detector add call")
    primary_apply_sides = _keyword(primary_calls[0], "apply_sides")
    if primary_apply_sides is None or "online_grf_applied_sides" not in ast.unparse(
        primary_apply_sides
    ):
        raise V17ContractError("primary add call no longer uses per-side application")
    detector_applies = _keyword(detector_calls[0], "applies_force")
    if not (
        isinstance(detector_applies, ast.Constant)
        and detector_applies.value is False
    ):
        raise V17ContractError("detector add call must pass applies_force=False")
    detector_prefix = _keyword(detector_calls[0], "name_prefix")
    if not (
        isinstance(detector_prefix, ast.Constant)
        and detector_prefix.value == "online_grf_detector_"
    ):
        raise V17ContractError("detector force namespace drifted")
    has_support_check = any(
        isinstance(node, ast.Call)
        and _call_name(node) == "_validate_hybrid_prescribed_support"
        for node in ast.walk(setup_model)
    )
    if not has_support_check:
        raise V17ContractError("hybrid prescribed-support fail-closed check is missing")
    return {
        "auto_disable_applied_side": True,
        "primary_per_side_application": True,
        "detector_applies_force_false": True,
        "hybrid_support_check": True,
    }


def validate_selected_routing(lock: Mapping[str, Any]) -> dict[str, Any]:
    routing = lock.get("routing_contract")
    if not isinstance(routing, Mapping):
        raise V17ContractError("routing contract is missing")
    config_path = _repo_path(routing.get("configuration"), label="routing.configuration")
    loader_path = _repo_path(routing.get("loader"), label="routing.loader")
    setup_path = _repo_path(routing.get("setup"), label="routing.setup")
    external_path = _repo_path(routing.get("external_loads"), label="routing.external_loads")
    try:
        config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        raise V17ContractError(f"cannot parse selected training config: {exc}") from exc
    if not isinstance(config, Mapping) or not isinstance(config.get("grf"), Mapping):
        raise V17ContractError("selected training config has no grf mapping")
    grf = config["grf"]
    expected_profile = lock["scientific_core"]["primary_profile"]["path"]
    checks = {
        "grf_mode_online_sensor": grf.get("grf_mode") == "online_sensor",
        "primary_profile_locked": grf.get("online_grf_profile") == expected_profile,
        "online_applied_side_left_only": grf.get("online_grf_applied_side") == ["left"],
        "no_explicit_prescribed_disable_drift": grf.get("disable_prescribed_grf_side") == [],
        "legacy_events_default": grf.get("phase_fsm_input_mode") == "legacy_events",
    }
    failed = [name for name, passed in checks.items() if not passed]
    if failed:
        raise V17ContractError(f"selected GRF routing failed: {failed}")

    try:
        setup_root = ET.parse(setup_path).getroot()
        selected_external = str(setup_root.findtext(".//external_loads_xml", "")).strip()
        selected_model = str(setup_root.findtext(".//model_file", "")).strip()
    except (ET.ParseError, OSError) as exc:
        raise V17ContractError(f"cannot parse simulator setup: {exc}") from exc
    if _repo_path(selected_external, label="setup.external_loads_xml") != external_path:
        raise V17ContractError("setup no longer selects the locked ExternalLoads XML")
    locked_model = _repo_path(
        lock["scientific_core"]["runtime_model"]["path"],
        label="scientific_core.runtime_model.path",
    )
    if _repo_path(selected_model, label="setup.model_file") != locked_model:
        raise V17ContractError("setup no longer selects the locked runtime model")

    try:
        external_root = ET.parse(external_path).getroot()
    except (ET.ParseError, OSError) as exc:
        raise V17ContractError(f"cannot parse runtime ExternalLoads XML: {exc}") from exc
    sides = []
    for force in external_root.findall(".//ExternalForce"):
        body = str(force.findtext("applied_to_body", "")).strip().lower()
        if body in {"foot_l", "/bodyset/foot_l"}:
            sides.append("left")
        elif body in {"calcn_r", "/bodyset/calcn_r", "foot_r", "/bodyset/foot_r"}:
            sides.append("right")
    if sides.count("left") != 1 or sides.count("right") != 1:
        raise V17ContractError("runtime ExternalLoads must expose one force per side")
    return {
        **checks,
        **validate_loader_routing(loader_path),
        "right_prescribed_external_force_present": True,
    }


def validate_all(
    *,
    contract_path: Path = DEFAULT_V17_CONTRACT,
    primary_lock_path: Path = DEFAULT_PRIMARY_LOCK,
) -> dict[str, Any]:
    contract = _strict_object(contract_path.resolve(), "V17 contract")
    lock = _strict_object(primary_lock_path.resolve(), "primary core lock")
    split = contract.get("data_split")
    if not isinstance(split, Mapping):
        raise V17ContractError("V17 data split is missing")
    if split.get("development") != ["02", "04", "08"]:
        raise V17ContractError("V17 development split drifted")
    if set(contract.get("development_sources", {})) != {"02", "04", "08"}:
        raise V17ContractError("V17 exposes a non-development source")
    decision = contract.get("decision_contract")
    if not isinstance(decision, Mapping) or any(
        decision.get(key) is not False
        for key in (
            "geometry_threshold_dwell_or_radius_sweep_allowed",
            "fallback_or_retuning_after_failure_allowed",
            "protected_open_allowed_by_this_contract",
            "runtime_or_training_promotion_allowed",
            "positive_morphology_weight_training_allowed",
        )
    ):
        raise V17ContractError("V17 decision contract is not fail-closed")
    result = {
        "schema_version": 1,
        "status": "PASS",
        "event_contract_id": EVENT_CONTRACT_ID,
        "v17_profile": validate_v17_profile(contract),
        "primary_core": validate_primary_core_lock(lock),
        "routing": validate_selected_routing(lock),
        "data_access": {
            "development_sources_exposed": ["02", "04", "08"],
            "protected_sources_exposed": [],
            "reserve_sources_exposed": [],
        },
    }
    json.dumps(result, allow_nan=False)
    return result


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--contract", type=Path, default=DEFAULT_V17_CONTRACT)
    parser.add_argument("--primary-lock", type=Path, default=DEFAULT_PRIMARY_LOCK)
    parser.add_argument(
        "--output",
        type=Path,
        help="optional immutable JSON report under validation/",
    )
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = _parse_args(argv)
    result = validate_all(
        contract_path=args.contract,
        primary_lock_path=args.primary_lock,
    )
    if args.output is not None:
        output = args.output.resolve()
        try:
            output.relative_to(REPO_ROOT / "validation")
        except ValueError as exc:
            raise V17ContractError("report output must stay under validation/") from exc
        gate.write_json_no_clobber(output, result)
        print(f"PASS {output}")
    else:
        print(json.dumps(result, indent=2, sort_keys=True, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
