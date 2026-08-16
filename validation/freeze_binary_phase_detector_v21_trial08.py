"""Freeze V21 before the procedural one-shot challenge on AB06 trial 08.

This command is deliberately non-semantic with respect to trial 08.  It reads
only already-public protocols and V21 development artefacts.  In particular it
does not open the trial-08 preprocessing lock, IK motion, prescribed GRF, or
canonical event ledger.  Those paths are recorded as declarations and can be
verified only after the one-shot opening receipt has consumed the stage.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import tempfile
from pathlib import Path
from typing import Any, Callable, Mapping, Sequence, TextIO


REPO_ROOT = Path(__file__).resolve().parents[1]
VALIDATION_ROOT = REPO_ROOT / "validation"

SCHEMA_VERSION = 1
FREEZE_ID = "AB06_BINARY_POINT_V21_TRIAL08_PROCEDURAL_ONE_SHOT_FREEZE"
CANDIDATE_ID = "v21_678b0b5162b706dd"

FREEZE_PATH = VALIDATION_ROOT / "binary_phase_detector_v21_trial08_freeze_lock.json"
EXECUTION_LEDGER_PATH = (
    VALIDATION_ROOT / "binary_phase_detector_v21_trial08_execution_ledger.json"
)
OUTPUT_DIR = (
    VALIDATION_ROOT
    / "binary_phase_detector_v21_holdout_runs/2026-08-04_trial08_one_shot"
)

V21_RUN_DIR = (
    VALIDATION_ROOT / "binary_phase_detector_v21_runs/2026-08-04_run01"
)
PROFILE_PATH = V21_RUN_DIR / "eligible_finalist_profile.json"
PROFILE_SHA256 = "be8e063304a4798e5fc9947beb69c7b2ad813b4cab65e3bfdb0f2cd7284439bc"
MANIFEST_PATH = V21_RUN_DIR / "manifest.json"
MANIFEST_SHA256 = "ecc89b780a22a0762a121572234cfde2a1062762e03981935f8484ac0b21055e"
FINAL_VERIFICATION_PATH = V21_RUN_DIR / "final_verification.json"
FINAL_VERIFICATION_SHA256 = (
    "9964374bca42d9a491ede1a19360a03c8f96264f63ebb3e66c10f9b1e602a713"
)

TRIAL_ID = "08"
TRIAL_ROLE = "INTERNAL_ONE_SHOT_HOLDOUT_RELATIVE_TO_V21_SWEEP_ONLY"
TRIAL_INTERVAL_S = (10.69, 154.89)
EXPECTED_SAMPLE_COUNT = 144201
SAMPLE_DT_S = 0.001
POLICY_STEP_S = 0.010
DEBOUNCE_S = 0.005

FROZEN_GATE = {
    "required_view_count": 4,
    "required_consumption_modes": [
        "sequential_1ms",
        "batched_10ms_same_samples",
    ],
    "required_unit_count": 8,
    "minimum_complete_cycles_per_view": 10,
    "exact_event_counts_order_and_cycles": True,
    "precision": 1.0,
    "recall": 1.0,
    "minimum_f1": 0.95,
    "minimum_iou": 0.90,
    "maximum_confirmed_hs_error_s": 0.050,
    "maximum_confirmed_to_error_s": 0.080,
    "maximum_delivered_hs_error_s": 0.060,
    "maximum_delivered_to_error_s": 0.090,
    "confirmation_delay_s": DEBOUNCE_S,
    "maximum_delivery_after_confirmation_s": 0.010,
    "minimum_accepted_flight_s": 0.030,
    "sequential_batch_exact_parity": True,
    "fast_fsm_signature_exact": True,
    "two_sensor_channel_gate_each_view": True,
    "all_metrics_finite_strict_json": True,
    "fsm_current_implementation_contract_exact": True,
    "terminal_pending_state_clear_required": True,
    "zero_candidate_cancellations_required": False,
    "candidate_cancellations_scope": (
        "diagnostic parity only, matching the frozen V21/V20 development gate"
    ),
}

POST_PASS_SCOPE = {
    "development_candidate_freeze_allowed": True,
    "h0_integration_implementation_allowed": True,
    "h0_execution_allowed": False,
    "development_candidate_h0_ready_allowed": False,
    "independent_validation_claim_allowed": False,
    "runtime_promotion_allowed": False,
    "training_promotion_allowed": False,
    "positive_morphology_reward_ppo_allowed": False,
    "next_independent_gates": ["05", "06"],
}

CANDIDATE_GEOMETRY = {
    "heel": {
        "lateral_fraction": 0.8030205847202724,
        "location_m": [
            -0.059315516055,
            -0.06316904531668477,
            0.01716156589300303,
        ],
        "reach_m": 0.025,
        "role": "left_heel",
        "surface_location_m": [
            -0.059315516055,
            -0.03816904531668476,
            0.01716156589300303,
        ],
        "x_m": -0.059315516055,
    },
    "toe": {
        "lateral_fraction": 0.5,
        "location_m": [
            0.135837908089,
            -0.06545974250405881,
            0.005834590883117434,
        ],
        "reach_m": 0.027,
        "role": "left_toe",
        "surface_location_m": [
            0.135837908089,
            -0.03845974250405881,
            0.005834590883117434,
        ],
        "x_m": 0.135837908089,
    },
}

CLAIM_LIMIT = (
    "procedural one-shot confirmation relative to the V21 DEV02/04 geometry "
    "sweep; not independent validation"
)

NON_ACTIONS = {
    "primary_grf_modification_allowed": False,
    "cpp_or_contact_modification_allowed": False,
    "sea_semantics_modification_allowed": False,
    "trial_specific_geometry_or_logic_allowed": False,
}

OPENING_CONTRACT = {
    "global_execution_ledger": EXECUTION_LEDGER_PATH.relative_to(
        REPO_ROOT
    ).as_posix(),
    "output_dir": OUTPUT_DIR.relative_to(REPO_ROOT).as_posix(),
    "ledger_and_access_receipt_before_semantic_read": True,
    "one_process_one_candidate_one_execution": True,
    "pass_fail_error_or_interrupt_consumes_stage": True,
    "retry_allowed": False,
    "rescue_reselection_or_retuning_allowed": False,
}

TRIAL08_DECLARATIONS = {
    "preprocessing_lock": {
        "path": (
            "validation/two_sensor_cross_speed_v14_2_runs/"
            "2026-07-22_ab06_cross_speed_v14_2_boundary_recovery/preprocessed/"
            "trial_08/treadmill_08_01_preprocessing_lock.json"
        ),
        "sha256": "c961e191871f0caab54edef82424cab20ca3e2938525af8fd1f2752c092399ca",
        "size_bytes": 5472,
    },
    "ik_motion": {
        "path": (
            "validation/two_sensor_cross_speed_v14_2_runs/"
            "2026-07-22_ab06_cross_speed_v14_2_boundary_recovery/preprocessed/"
            "trial_08/treadmill_08_01_ik.mot"
        ),
        "sha256": "87af7ec5e6530f179bb2fd84c7f2caf78da041f6dcd92966ac8ca1999e1ce14a",
        "size_bytes": 10787093,
    },
    "canonical_oracle": {
        "path": (
            "validation/canonical_event_oracles/2026-08-03_v17_development/"
            "trial_08_canonical_event_ledger.json"
        ),
        "sha256": "aa0cf5a2b044bcf5faecf012e8eac5a3693a48459b9dfabc1317536288021f16",
        "size_bytes": 3377913,
    },
    "model": {
        "path": "models/AB06_SEASEA-raw/osimxml/AB06_SEASEA_marker_calibrated.osim",
        "sha256": "98cfcbc4f7155ea4576f583654fbd50a6e8bd2f2f33ff0894c9f3f24dce5fa8d",
        "size_bytes": 500970,
    },
    "plugin_binary_macos": {
        "path": "plugins/libSEA_Plugin_BlackBox_mCMC_impedence_ff.dylib",
        "sha256": "77390d0f74055fb3419e88637baac1d215b1dd402ee1effe3e8cb14a66caf54b",
        "size_bytes": 218560,
    },
}

# Hashes here cover only inputs that may be verified before trial 08 is opened.
PINNED_PREACCESS_SOURCES = {
    "validation/sweep_binary_phase_detector_v21_geometry.py": (
        "32aeb6dadba42000607e7d1e7a2480d16574b032fb0d59ad2cd5fd6558147a47"
    ),
    "validation/validate_binary_phase_fsm_v20_development.py": (
        "1ce0b82a4e9db3b2d90b4dc132de798f8eb452e784fe2492aa76ccbe10e8e431"
    ),
    "Trajectory Generator/binary_phase_fsm.py": (
        "0f7669b60a72c1b27ee3c4f1a43161eeb9f2d091dff5558cc4fa43f1fce8d9c1"
    ),
    "binary_phase_detector.py": (
        "57a313133e1ce5a675b2699e940226325dfa5b2b895c7eb6b17c0892a94263b6"
    ),
    "validation/binary_phase_fsm_v20_development_receipt.json": (
        "43fe41ba938b020f75604a5c21dbe433a12bc3e8233be47f98bc697d9ac41e2c"
    ),
    "validation/build_canonical_grf_event_oracle.py": (
        "246c7cb326c209fe5bf732e3c5b2d3a9125b33d3cb3208378f7ded0bd7c40a89"
    ),
    "validation/two_sensor_v18_signal_semantics_protocol.json": (
        "7bd6db285da14ae102da9c1c33c377f658887cfac25650254428abf9a380dc9a"
    ),
    "validation/two_sensor_v15_routing_protocol.json": (
        "b18b884239af363dcd08c70101b3416587577d4e5629a124bb843a30caab92a9"
    ),
    "validation/diagnose_two_sensor_v15_routing.py": (
        "0548507a123a5bccf9c3739bc995b0fcf0f5a80021b19fd342cae4502f6a107f"
    ),
    "kinematics_interpolator.py": (
        "424d352a461b424ed8f7e318513a85b75d3a6fb1a00155eab1e885e9d3fd4ede"
    ),
    "config.py": (
        "88c120bdf8249143a78cd19a33a4de34c10d4230a2ad6760b33dec9bb51417e3"
    ),
    "model_loader.py": (
        "fba3f025a83082bb07276770b21f644e3c84750402d97c6305c7ea0eef8ccd76"
    ),
}

DYNAMIC_FREEZE_SOURCES = (
    "validation/freeze_binary_phase_detector_v21_trial08.py",
    "validation/validate_binary_phase_detector_v21_trial08_one_shot.py",
    "validation/test_binary_phase_detector_v21_trial08_one_shot.py",
)


class V21FreezeError(RuntimeError):
    """Raised when V21 cannot be frozen without opening trial 08."""


def _reject_json_constant(value: str) -> None:
    raise V21FreezeError(f"non-finite JSON constant is forbidden: {value}")


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def canonical_sha256(value: object) -> str:
    encoded = json.dumps(
        value,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=True,
        allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _strict_json(path: Path, expected_sha256: str) -> dict[str, Any]:
    if not path.is_file() or sha256_file(path) != expected_sha256:
        raise V21FreezeError(f"hash-pinned JSON drifted: {path}")
    try:
        value = json.loads(
            path.read_text(encoding="utf-8"),
            parse_constant=_reject_json_constant,
        )
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise V21FreezeError(f"cannot read strict JSON: {path}") from exc
    if not isinstance(value, dict):
        raise V21FreezeError(f"JSON root must be an object: {path}")
    return value


def _source_record(path: Path) -> dict[str, Any]:
    resolved = path.resolve()
    if not resolved.is_file():
        raise V21FreezeError(f"source is missing: {resolved}")
    try:
        relative = resolved.relative_to(REPO_ROOT.resolve()).as_posix()
    except ValueError as exc:
        raise V21FreezeError(f"source escapes repository: {resolved}") from exc
    return {
        "path": relative,
        "sha256": sha256_file(resolved),
        "size_bytes": int(resolved.stat().st_size),
    }


def _verify_preaccess_sources() -> dict[str, dict[str, Any]]:
    records: dict[str, dict[str, Any]] = {}
    for relative, expected in PINNED_PREACCESS_SOURCES.items():
        record = _source_record(REPO_ROOT / relative)
        if record["sha256"] != expected:
            raise V21FreezeError(f"pinned source drifted: {relative}")
        records[relative] = record
    for relative in DYNAMIC_FREEZE_SOURCES:
        records[relative] = _source_record(REPO_ROOT / relative)
    return records


def _verify_v21_selection() -> dict[str, Any]:
    profile = _strict_json(PROFILE_PATH, PROFILE_SHA256)
    manifest = _strict_json(MANIFEST_PATH, MANIFEST_SHA256)
    verification = _strict_json(
        FINAL_VERIFICATION_PATH, FINAL_VERIFICATION_SHA256
    )
    points = profile.get("points")
    checks = {
        "candidate_id": manifest.get("selected_candidate_id") == CANDIDATE_ID,
        "eligible_finalist": manifest.get("eligible_finalist") is True,
        "status_trial08_closed": manifest.get("status")
        == "V21_GEOMETRY_SWEEP_ELIGIBLE_FINALIST_DEV02_04_TRIAL08_CLOSED",
        "next_stage": manifest.get("next_stage")
        == "FREEZE_BEFORE_ONE_SHOT_TRIAL08",
        "profile_binding": manifest.get("selected_profile", {}).get("sha256")
        == PROFILE_SHA256,
        "profile_type": profile.get("detector_type")
        == "binary_point_clearance_v1",
        "profile_roles": isinstance(points, list)
        and [item.get("name") for item in points] == ["left_heel", "left_toe"],
        "profile_rule": profile.get("contact_rule", {}).get("contact_when")
        == "signed_clearance_le_zero",
    }
    candidates = verification.get("candidates")
    selected = None
    if isinstance(candidates, list):
        selected = next(
            (
                item
                for item in candidates
                if isinstance(item, Mapping)
                and item.get("candidate_id") == CANDIDATE_ID
            ),
            None,
        )
    checks["verification_candidate_unique"] = (
        isinstance(candidates, list)
        and sum(
            isinstance(item, Mapping)
            and item.get("candidate_id") == CANDIDATE_ID
            for item in candidates
        )
        == 1
    )
    checks["verification_eligible"] = (
        isinstance(selected, Mapping) and selected.get("eligible") is True
    )
    checks["verification_units"] = (
        isinstance(selected, Mapping)
        and selected.get("unit_count") == 16
        and selected.get("unit_pass_count") == 16
    )
    if isinstance(selected, Mapping) and isinstance(points, list):
        selected_locations = [
            selected.get("heel", {}).get("location_m"),
            selected.get("toe", {}).get("location_m"),
        ]
        profile_locations = [item.get("location") for item in points]
        checks["profile_geometry_binding"] = selected_locations == profile_locations
    else:
        checks["profile_geometry_binding"] = False
    checks["development_scope"] = manifest.get("data_access") == {
        "canonical_ledgers_read": ["02", "04"],
        "development_trials_opened": ["02", "04"],
        "prescribed_grf_read": False,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
        "trial_08_opened": False,
    }
    if not all(checks.values()):
        raise V21FreezeError(f"V21 selection binding failed: {checks}")
    assert isinstance(selected, Mapping)
    geometry = {
        "heel": dict(selected["heel"]),
        "toe": dict(selected["toe"]),
    }
    if geometry != CANDIDATE_GEOMETRY:
        raise V21FreezeError("selected V21 geometry drifted from the preregistered finalist")
    return {
        "checks": checks,
        "geometry": geometry,
        "geometry_sha256": canonical_sha256(geometry),
        "profile": _source_record(PROFILE_PATH),
        "manifest": _source_record(MANIFEST_PATH),
        "final_verification": _source_record(FINAL_VERIFICATION_PATH),
    }


def _verify_protocol_declarations() -> dict[str, Any]:
    protocol_path = REPO_ROOT / "validation/two_sensor_v15_routing_protocol.json"
    protocol = _strict_json(
        protocol_path, PINNED_PREACCESS_SOURCES[protocol_path.relative_to(REPO_ROOT).as_posix()]
    )
    observed = protocol.get("preprocessing", {}).get("trials", {}).get(TRIAL_ID)
    if not isinstance(observed, Mapping):
        raise V21FreezeError("V15 protocol lost the trial-08 declarations")
    for label in ("preprocessing_lock", "ik_motion"):
        expected = TRIAL08_DECLARATIONS[label]
        record = observed.get(label)
        if not isinstance(record, Mapping) or {
            "path": record.get("path"),
            "sha256": record.get("sha256"),
        } != {"path": expected["path"], "sha256": expected["sha256"]}:
            raise V21FreezeError(f"trial-08 declaration drifted: {label}")
    if protocol.get("trials", {}).get(TRIAL_ID, {}).get("trial_interval_s") != list(
        TRIAL_INTERVAL_S
    ):
        raise V21FreezeError("trial-08 interval declaration drifted")
    return {
        "protocol": _source_record(protocol_path),
        "declared_without_opening": True,
    }


def build_freeze_payload(*, check_destinations: bool = False) -> dict[str, Any]:
    if check_destinations:
        if os.path.lexists(FREEZE_PATH):
            raise V21FreezeError(f"freeze lock already exists: {FREEZE_PATH}")
        if os.path.lexists(EXECUTION_LEDGER_PATH) or os.path.lexists(OUTPUT_DIR):
            raise V21FreezeError("trial-08 one-shot destination is already consumed")
    selection = _verify_v21_selection()
    protocol = _verify_protocol_declarations()
    sources = _verify_preaccess_sources()
    gate = json.loads(json.dumps(FROZEN_GATE, allow_nan=False))
    payload = {
        "schema_version": SCHEMA_VERSION,
        "freeze_id": FREEZE_ID,
        "freeze_date": "2026-08-04",
        "status": "V21_FROZEN_BEFORE_TRIAL08_PROCEDURAL_ONE_SHOT",
        "frozen_before_trial08_v21_performance_access": True,
        "candidate": {
            "candidate_id": CANDIDATE_ID,
            "profile": selection["profile"],
            "geometry": selection["geometry"],
            "geometry_sha256": selection["geometry_sha256"],
            "fsm": sources["Trajectory Generator/binary_phase_fsm.py"],
            "fsm_current_implementation_contract_id": (
                "binary_point_v19+functional_contact_fsm_v1_shadow"
            ),
            "target_bundle_contract_id_after_h0_integration": (
                "primary_grf_split_v1+binary_point_v21+functional_contact_fsm_v1"
            ),
            "sample_dt_s": SAMPLE_DT_S,
            "policy_step_s": POLICY_STEP_S,
            "debounce_s": DEBOUNCE_S,
            "candidate_count_after_freeze": 1,
        },
        "v21_selection": selection,
        "trial": {
            "trial_id": TRIAL_ID,
            "role": TRIAL_ROLE,
            "scientifically_virgin": False,
            "historical_lineage_acknowledged": ["V14.2", "V17", "V18 preflight"],
            "claim_limit": CLAIM_LIMIT,
            "interval_s": list(TRIAL_INTERVAL_S),
            "sample_dt_s": SAMPLE_DT_S,
            "expected_sample_count": EXPECTED_SAMPLE_COUNT,
            "declared_inputs_not_verified_or_opened_by_freeze": TRIAL08_DECLARATIONS,
            "protocol_declaration_check": protocol,
        },
        "gate": gate,
        "opening_contract": json.loads(
            json.dumps(OPENING_CONTRACT, allow_nan=False)
        ),
        "data_governance": {
            "development_opened_before_freeze": ["02", "04"],
            "trial08_opened_by_this_freeze": False,
            "current_cycle_pre_freeze_nonperformance_metadata_access": {
                "occurred": True,
                "mechanism": "broad source-index search during validator review",
                "observed_fields": [
                    "live_replay_inputs.plugin.binary_path",
                    "live_replay_inputs.plugin.loader_basename",
                ],
                "ik_values_read": False,
                "oracle_or_event_values_read": False,
                "grf_values_read": False,
                "detector_trace_or_metrics_read": False,
                "candidate_or_gate_changed_after_access": False,
                "scientific_performance_blinding_preserved": True,
            },
            "protected_trials_opened": [],
            "reserve_trials_opened": [],
            "protected_trials_remaining_closed": ["05", "06"],
            "reserve_trials_remaining_closed": ["03", "07"],
            "prescribed_grf_required_by_binary_replay": False,
            "external_loads_required_by_binary_replay": False,
        },
        "sources": sources,
        "post_pass_scope": json.loads(
            json.dumps(POST_PASS_SCOPE, allow_nan=False)
        ),
        "non_actions": json.loads(json.dumps(NON_ACTIONS, allow_nan=False)),
    }
    json.dumps(payload, allow_nan=False)
    return payload


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


def _atomic_write_exclusive(path: Path, writer: Callable[[TextIO], None]) -> Path:
    if not path.parent.is_dir():
        raise V21FreezeError(f"output parent does not exist: {path.parent}")
    descriptor, temporary_raw = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent)
    )
    temporary = Path(temporary_raw)
    descriptor_open = True
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8", newline="\n") as stream:
            descriptor_open = False
            writer(stream)
            stream.flush()
            os.fsync(stream.fileno())
        os.link(temporary, path)
        _fsync_directory(path.parent)
        return path
    finally:
        if descriptor_open:
            os.close(descriptor)
        try:
            temporary.unlink()
        except FileNotFoundError:
            pass


def write_json_exclusive(path: Path, payload: Mapping[str, Any]) -> Path:
    def writer(stream: TextIO) -> None:
        json.dump(dict(payload), stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")

    return _atomic_write_exclusive(path, writer)


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_mutually_exclusive_group(required=True)
    modes.add_argument(
        "--check", action="store_true", help="validate freeze inputs without writing"
    )
    modes.add_argument(
        "--freeze", action="store_true", help="publish the immutable freeze lock"
    )
    args = parser.parse_args(argv)
    try:
        if args.freeze:
            payload = build_freeze_payload(check_destinations=True)
            write_json_exclusive(FREEZE_PATH, payload)
            existing_freeze_verified = False
        elif FREEZE_PATH.is_file():
            payload = build_freeze_payload(check_destinations=False)
            existing = _strict_json(FREEZE_PATH, sha256_file(FREEZE_PATH))
            if existing != payload:
                raise V21FreezeError("existing freeze lock differs from expected payload")
            existing_freeze_verified = True
        else:
            payload = build_freeze_payload(check_destinations=True)
            existing_freeze_verified = False
    except Exception as exc:
        print(
            f"V21 trial-08 freeze failed closed: {type(exc).__name__}: {exc}",
            file=os.sys.stderr,
        )
        return 2
    summary = {
        "status": payload["status"],
        "candidate_id": CANDIDATE_ID,
        "trial_role": TRIAL_ROLE,
        "trial08_semantic_access": False,
        "existing_freeze_verified": existing_freeze_verified,
        "freeze_path": FREEZE_PATH.relative_to(REPO_ROOT).as_posix()
        if args.freeze
        else None,
    }
    print(json.dumps(summary, indent=2, sort_keys=True, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
