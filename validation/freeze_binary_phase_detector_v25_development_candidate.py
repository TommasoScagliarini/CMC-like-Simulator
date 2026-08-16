"""Freeze the V25 development candidate before any H0 protocol is opened.

This receipt consumes the completed V25 development sweep as immutable input
for a future, separately frozen H0 A/B/C protocol.  It does not execute H0,
change runtime configuration, open protected data, or promote the detector.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import sys
import tempfile
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Mapping, Sequence, TextIO


REPO_ROOT = Path(__file__).resolve().parents[1]
VALIDATION_ROOT = REPO_ROOT / "validation"
SCHEMA_VERSION = 25
FREEZE_ID = "AB06_BINARY_POINT_V25_DEVELOPMENT_CANDIDATE_GLOBAL_FREEZE"
CANDIDATE_ID = "v25_4b351f67b5b86ab0"
DESTINATION = (
    VALIDATION_ROOT
    / "binary_phase_detector_v25_development_candidate_freeze_lock.json"
)
RUN_DIR = (
    VALIDATION_ROOT
    / "binary_phase_detector_v25_geometry_runs/"
    "2026-08-04_local_reach_sweep_dev02_04_08"
)

PINNED = {
    "v25_manifest": (
        RUN_DIR / "manifest.json",
        "4feeee1a32db9d1b8efc8cef5cdb08c8b35c3726b91cabd7013b52ac11748094",
    ),
    "v25_terminal_decision": (
        RUN_DIR / "terminal_decision.json",
        "f34b805051883b85f26a9c42b6b4601d9c68bf5bf58dda14da82cf69c42db9f0",
    ),
    "v25_profile": (
        RUN_DIR / "selected_candidate_profile.json",
        "db704e502b99e49bea6d89493812bafdac748f8ce8d3ce28214ff624078539a2",
    ),
    "v25_exact_verification": (
        RUN_DIR / "stage1_exact_verification.json",
        "49c71778d526774adc72f3aea68b34292ea38a466b94d35138851b2954cf70b7",
    ),
    "v25_script": (
        VALIDATION_ROOT / "sweep_binary_phase_detector_v25_geometry.py",
        "0b66c1f74e8c9bb658af14099058fba292b70968cfe8039a8d35589c73216299",
    ),
    "v24_receipt": (
        VALIDATION_ROOT
        / "binary_phase_detector_v24_diagnostic_runs/"
        "2026-08-04_trial08_clearance_gap/diagnostic_receipt.json",
        "3c4c0caa78745bafeaae812f332158f44152c4148a99d6aa4fb3fd0ec36c59c2",
    ),
    "fsm_v20": (
        REPO_ROOT / "Trajectory Generator/binary_phase_fsm.py",
        "0f7669b60a72c1b27ee3c4f1a43161eeb9f2d091dff5558cc4fa43f1fce8d9c1",
    ),
    "binary_detector": (
        REPO_ROOT / "binary_phase_detector.py",
        "57a313133e1ce5a675b2699e940226325dfa5b2b895c7eb6b17c0892a94263b6",
    ),
    "online_grf": (
        REPO_ROOT / "online_grf.py",
        "52e39bf9a3b20dd65242f3f9076d76ed788239fe7c3e5b825bc37a9657c4fefa",
    ),
}

EXPECTED_TRIALS = ("02", "04", "08")
EXPECTED_VIEWS = ("plateau_01", "plateau_02", "plateau_03", "plateau_04")
EXPECTED_MODES = ("sequential_1ms", "batched_10ms_same_samples")
EXPECTED_HEEL_LOCATION_M = (
    -0.059315516055,
    -0.06316904531668477,
    0.01716156589300303,
)
EXPECTED_TOE_LOCATION_M = (
    0.135837908089,
    -0.06595974250405881,
    0.005834590883117434,
)


class V25CandidateFreezeError(RuntimeError):
    """Raised when V25 evidence cannot be frozen without ambiguity."""


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _reject_json_constant(value: str) -> None:
    raise V25CandidateFreezeError(f"non-finite JSON constant: {value}")


def _strict_json(path: Path, expected_sha256: str) -> dict[str, Any]:
    if not path.is_file() or sha256_file(path) != expected_sha256:
        raise V25CandidateFreezeError(f"pinned artifact drifted: {path}")
    try:
        payload = json.loads(
            path.read_text(encoding="utf-8"),
            parse_constant=_reject_json_constant,
        )
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise V25CandidateFreezeError(f"cannot parse strict JSON: {path}") from exc
    if not isinstance(payload, dict):
        raise V25CandidateFreezeError(f"JSON root is not an object: {path}")
    return payload


def _source_record(path: Path) -> dict[str, Any]:
    resolved = path.resolve()
    if not resolved.is_file():
        raise V25CandidateFreezeError(f"source is missing: {resolved}")
    return {
        "path": resolved.relative_to(REPO_ROOT.resolve()).as_posix(),
        "sha256": sha256_file(resolved),
        "size_bytes": int(resolved.stat().st_size),
    }


def _finite_tree(value: object) -> bool:
    if isinstance(value, bool) or value is None or isinstance(value, str):
        return True
    if isinstance(value, int):
        return True
    if isinstance(value, float):
        return math.isfinite(value)
    if isinstance(value, Mapping):
        return all(_finite_tree(item) for item in value.values())
    if isinstance(value, (list, tuple)):
        return all(_finite_tree(item) for item in value)
    return False


def _same_numbers(observed: object, expected: Sequence[float]) -> bool:
    return bool(
        isinstance(observed, Sequence)
        and not isinstance(observed, (str, bytes, bytearray))
        and len(observed) == len(expected)
        and all(
            abs(float(left) - float(right)) <= 1e-12
            for left, right in zip(observed, expected)
        )
    )


def _verify_manifest_files(manifest: Mapping[str, Any]) -> dict[str, Any]:
    records = manifest.get("output_files")
    if not isinstance(records, Mapping) or len(records) != 7:
        raise V25CandidateFreezeError("V25 manifest output file map drifted")
    verified: dict[str, Any] = {}
    for name, record in records.items():
        if not isinstance(name, str) or not isinstance(record, Mapping):
            raise V25CandidateFreezeError("malformed V25 manifest source record")
        raw_path = record.get("path")
        if not isinstance(raw_path, str) or Path(raw_path).is_absolute():
            raise V25CandidateFreezeError("non-portable V25 output path")
        path = (REPO_ROOT / raw_path).resolve()
        try:
            path.relative_to(RUN_DIR.resolve())
        except ValueError as exc:
            raise V25CandidateFreezeError("V25 output escapes run directory") from exc
        if (
            not path.is_file()
            or int(record.get("bytes", -1)) != path.stat().st_size
            or record.get("sha256") != sha256_file(path)
        ):
            raise V25CandidateFreezeError(f"V25 output record drifted: {name}")
        verified[name] = _source_record(path)
    return verified


def preflight_unfrozen() -> dict[str, Any]:
    payloads: dict[str, dict[str, Any]] = {}
    sources: dict[str, dict[str, Any]] = {}
    for label, (path, expected_sha256) in PINNED.items():
        sources[label] = _source_record(path)
        if sources[label]["sha256"] != expected_sha256:
            raise V25CandidateFreezeError(f"pinned source drifted: {label}")
        if path.suffix == ".json":
            payloads[label] = _strict_json(path, expected_sha256)

    manifest = payloads["v25_manifest"]
    decision = payloads["v25_terminal_decision"]
    profile = payloads["v25_profile"]
    exact = payloads["v25_exact_verification"]
    v24 = payloads["v24_receipt"]
    selected = manifest.get("selected_verification")
    verifications = exact.get("verifications")
    if not isinstance(selected, Mapping) or not isinstance(verifications, list):
        raise V25CandidateFreezeError("selected V25 verification is missing")
    units = selected.get("units")
    parity = selected.get("parity")
    continuity = selected.get("oracle_stance_continuity")
    data_scope = manifest.get("data_scope")
    points = profile.get("points")
    expected_cover = {
        (trial, view, mode)
        for trial in EXPECTED_TRIALS
        for view in EXPECTED_VIEWS
        for mode in EXPECTED_MODES
    }
    observed_cover = (
        {
            (
                str(unit.get("trial_id")),
                str(unit.get("view_id")),
                str(unit.get("consumption_mode")),
            )
            for unit in units
        }
        if isinstance(units, list)
        else set()
    )
    decision_replicated = all(
        key in manifest and manifest[key] == value
        for key, value in decision.items()
    )
    output_files = _verify_manifest_files(manifest)
    assertions = {
        "manifest_pass": manifest.get("status")
        == "PASS_V25_LOCAL_GEOMETRY_DEVELOPMENT"
        and manifest.get("pass") is True,
        "decision_pass": decision.get("status")
        == "PASS_V25_LOCAL_GEOMETRY_DEVELOPMENT"
        and decision.get("pass") is True,
        "decision_replicated_by_manifest": decision_replicated,
        "candidate_exact": manifest.get("selected_candidate_id") == CANDIDATE_ID
        and selected.get("candidate_id") == CANDIDATE_ID,
        "single_exact_verification": len(verifications) == 1
        and verifications[0] == selected,
        "stage1_unique_eligible": manifest.get("stage1_candidate_count") == 9
        and manifest.get("stage1_screen_eligible_count") == 1
        and manifest.get("stage1_exact_eligible_count") == 1,
        "stage2_closed": manifest.get("stage2_opened") is False
        and manifest.get("stage2_candidate_count") == 0
        and manifest.get("stage2_exact_eligible_count") == 0,
        "twenty_four_units_exact": isinstance(units, list)
        and len(units) == 24
        and selected.get("unit_count") == 24
        and selected.get("unit_pass_count") == 24
        and all(unit.get("pass") is True for unit in units),
        "cartesian_coverage_exact": observed_cover == expected_cover,
        "event_cycle_gates_exact": isinstance(units, list)
        and all(
            unit.get("exact_global_event_order") is True
            and unit.get("cycles", {}).get("expected")
            == unit.get("cycles", {}).get("observed")
            and unit.get("accepted_flight_pass") is True
            and unit.get("heel_strike", {}).get("precision") == 1.0
            and unit.get("heel_strike", {}).get("recall") == 1.0
            and unit.get("toe_off", {}).get("precision") == 1.0
            and unit.get("toe_off", {}).get("recall") == 1.0
            for unit in units
        ),
        "three_trial_parity_exact": isinstance(parity, list)
        and {item.get("trial_id") for item in parity} == set(EXPECTED_TRIALS)
        and all(
            item.get("pass") is True
            and all(
                value is True
                for key, value in item.items()
                if key != "trial_id"
            )
            for item in parity
        ),
        "zero_internal_gaps": isinstance(continuity, Mapping)
        and continuity.get("pass") is True
        and continuity.get("raw_internal_gap_count") == 0
        and continuity.get("debounce_stable_internal_gap_count") == 0,
        "geometry_exact": selected.get("heel", {}).get("reach_m") == 0.025
        and selected.get("toe", {}).get("reach_m") == 0.0275
        and _same_numbers(
            selected.get("heel", {}).get("location_m"), EXPECTED_HEEL_LOCATION_M
        )
        and _same_numbers(
            selected.get("toe", {}).get("location_m"), EXPECTED_TOE_LOCATION_M
        ),
        "profile_exact": profile.get("detector_type")
        == "binary_point_clearance_v1"
        and profile.get("contact_rule", {}).get("contact_when")
        == "signed_clearance_le_zero"
        and isinstance(points, list)
        and len(points) == 2
        and _same_numbers(points[0].get("location"), EXPECTED_HEEL_LOCATION_M)
        and _same_numbers(points[1].get("location"), EXPECTED_TOE_LOCATION_M),
        "development_scope_exact": isinstance(data_scope, Mapping)
        and data_scope.get("pass") is True
        and data_scope.get("opened_development_trials") == list(EXPECTED_TRIALS)
        and data_scope.get("opened_protected_trials") == []
        and data_scope.get("opened_reserve_trials") == []
        and data_scope.get("opened_historical_trials") == [],
        "runtime_and_training_untouched": manifest.get("h0_executed") is False
        and manifest.get("runtime_profile_modified") is False
        and manifest.get("training_started") is False
        and manifest.get("corridor_started") is False
        and manifest.get("ppo_updates") == 0,
        "v24_prerequisite_pass": v24.get("status")
        == "PASS_V24_TRIAL08_GEOMETRIC_CLEARANCE_GAP_CONFIRMED"
        and v24.get("pass") is True,
        "all_evidence_finite": _finite_tree(manifest)
        and _finite_tree(decision)
        and _finite_tree(exact),
        "manifest_files_exact": len(output_files) == 7,
        "freeze_destination_unoccupied": not os.path.lexists(DESTINATION),
    }
    if not all(assertions.values()):
        raise V25CandidateFreezeError(
            f"V25 development freeze preflight failed: {assertions}"
        )
    return {
        "schema_version": SCHEMA_VERSION,
        "freeze_id": FREEZE_ID,
        "status": "V25_DEVELOPMENT_CANDIDATE_FREEZE_READY_UNFROZEN",
        "assertions": assertions,
        "sources": sources,
        "manifest_output_files": output_files,
        "candidate": {
            "candidate_id": CANDIDATE_ID,
            "profile": sources["v25_profile"],
            "heel_location_m": list(EXPECTED_HEEL_LOCATION_M),
            "toe_location_m": list(EXPECTED_TOE_LOCATION_M),
            "heel_reach_m": 0.025,
            "toe_reach_m": 0.0275,
        },
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


def _write_json_exclusive(path: Path, payload: Mapping[str, Any]) -> Path:
    if os.path.lexists(path):
        raise V25CandidateFreezeError(f"refusing to clobber: {path}")
    encoded = (
        json.dumps(dict(payload), indent=2, sort_keys=True, allow_nan=False)
        + "\n"
    ).encode("utf-8")
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
            raise V25CandidateFreezeError(f"refusing to clobber: {path}")
        try:
            os.link(temporary, path)
        except FileExistsError as exc:
            raise V25CandidateFreezeError(f"refusing to clobber: {path}") from exc
        _fsync_directory(path.parent)
        return path
    finally:
        if descriptor_open:
            os.close(descriptor)
        try:
            temporary.unlink()
        except FileNotFoundError:
            pass


def freeze_candidate() -> dict[str, Any]:
    preflight = preflight_unfrozen()
    payload = {
        "schema_version": SCHEMA_VERSION,
        "freeze_id": FREEZE_ID,
        "status": (
            "V25_DEVELOPMENT_CANDIDATE_FROZEN_H0_PROTOCOL_REQUIRED"
        ),
        "pass": True,
        "frozen_at_utc": datetime.now(timezone.utc).isoformat(),
        "candidate": preflight["candidate"],
        "evidence": {
            "manifest": preflight["sources"]["v25_manifest"],
            "terminal_decision": preflight["sources"][
                "v25_terminal_decision"
            ],
            "exact_verification": preflight["sources"][
                "v25_exact_verification"
            ],
            "v24_diagnostic_receipt": preflight["sources"]["v24_receipt"],
            "manifest_output_files": preflight["manifest_output_files"],
        },
        "implementation": {
            "v25_sweep_source": preflight["sources"]["v25_script"],
            "binary_detector_source": preflight["sources"]["binary_detector"],
            "fsm_v20_source": preflight["sources"]["fsm_v20"],
            "online_grf_source_untouched": preflight["sources"]["online_grf"],
        },
        "contracts": {
            "raw_detector_contract_id": "binary_point_clearance_v1",
            "fsm_implementation_contract_id": (
                "binary_point_v19+functional_contact_fsm_v1_shadow"
            ),
            "target_h0_bundle_contract_id": (
                "primary_grf_split_v1+binary_point_v25+"
                "functional_contact_fsm_v1"
            ),
            "default_event_source": "legacy_events",
            "morphology_weight": 0.0,
            "sample_dt_s": 0.001,
            "policy_step_s": 0.01,
        },
        "data_governance": {
            "development_trials_consumed": list(EXPECTED_TRIALS),
            "protected_trials_opened": [],
            "protected_trials_remaining_closed": ["05", "06"],
            "reserve_trials_opened": [],
            "reserve_trials_remaining_closed": ["03", "07"],
            "historical_trial_01_reopened": False,
        },
        "lifecycle": {
            "v25_run_consumed": True,
            "v25_rerun_allowed": False,
            "candidate_reselection_or_retuning_allowed": False,
            "global_development_candidate_frozen": True,
            "runtime_promoted": False,
        },
        "post_pass_scope": {
            "dormant_shadow_integration_implementation_allowed": True,
            "h0_protocol_freeze_allowed": True,
            "h0_execution_allowed": False,
            "protected_trial_access_allowed": False,
            "runtime_promotion_allowed": False,
            "training_allowed": False,
            "corridor_activation_allowed": False,
            "positive_morphology_reward_ppo_allowed": False,
        },
        "next_stage": "FREEZE_H0_A_B_C_PROTOCOL_BEFORE_ANY_H0_EXECUTION",
        "freeze_script": _source_record(Path(__file__)),
        "preflight_assertions": preflight["assertions"],
    }
    json.dumps(payload, allow_nan=False)
    _write_json_exclusive(DESTINATION, payload)
    return payload


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--check", action="store_true")
    mode.add_argument("--freeze", action="store_true")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        payload = preflight_unfrozen() if args.check else freeze_candidate()
    except Exception as exc:
        print(
            f"V25 candidate freeze failed closed: {type(exc).__name__}: {exc}",
            file=sys.stderr,
        )
        return 2
    print(json.dumps(payload, indent=2, sort_keys=True, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
