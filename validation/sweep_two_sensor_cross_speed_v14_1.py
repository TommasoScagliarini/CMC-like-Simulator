"""Fail-closed V14.1 recovery wrapper for the AB06 cross-speed experiment.

V14 was consumed before detector replay because its converter required the
dataset-IK, force-plate, and marker streams to have identical endpoints.  This
module never edits or reuses that destination.  It binds a new one-shot run to
the same V14 scientific core and changes only preprocessing provenance:

* every source must cover a frozen, absolute analysis interval;
* the analysis interval is inset by >=10 ms and snapped inward to the global
  5 ms marker lattice before any new trial is opened;
* the recovery converter preserves all native samples (no crop/resample);
* an access receipt precedes every development decode and a preprocessing lock
  precedes every detector replay;
* a detector and prescribed-GRF warmup certificate protects the continuous FSM
  state before the first scoreable plateau event.

Trial 05 and trial 06 remain behind the original V14 one-shot gates.  Trials
03 and 07 have no semantic access route.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
import sys
import traceback
from contextlib import contextmanager
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any, Iterator, Mapping, Sequence

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
VALIDATION_ROOT = REPO_ROOT / "validation"
TOOLS_ROOT = REPO_ROOT / "tools"
for _path in (REPO_ROOT, VALIDATION_ROOT, TOOLS_ROOT):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

import convert_epic_ab06_tables_recovery as recovery_converter  # noqa: E402
import sweep_two_sensor_cross_speed_v14 as v14  # noqa: E402
import two_sensor_v14_1_warmup as warmup  # noqa: E402


SCHEMA_VERSION = 14
PROTOCOL_ID = (
    "AB06_TWO_SENSOR_CROSS_SPEED_INTERPOLATION_2026-07-22_V14_1_RECOVERY"
)
RECOVERY_ID = (
    "AB06_TWO_SENSOR_CROSS_SPEED_V14_TO_V141_PREPROCESSING_RECOVERY_2026-07-22"
)
DEFAULT_PROTOCOL = VALIDATION_ROOT / "two_sensor_cross_speed_v14_1_protocol.json"
DEFAULT_OUTPUT_DIR = (
    VALIDATION_ROOT
    / "two_sensor_cross_speed_v14_1_runs/"
    "2026-07-22_ab06_cross_speed_v14_1_recovery"
)
DEFAULT_EXECUTION_LEDGER = (
    VALIDATION_ROOT / "two_sensor_cross_speed_v14_1_execution_ledger.json"
)
RECOVERY_LINEAGE = (
    VALIDATION_ROOT / "two_sensor_cross_speed_v141_recovery_lineage.json"
)
PARENT_PROTOCOL = VALIDATION_ROOT / "two_sensor_cross_speed_v14_protocol.json"
PARENT_LEDGER = VALIDATION_ROOT / "two_sensor_cross_speed_v14_execution_ledger.json"
PARENT_OUTPUT_DIR = (
    VALIDATION_ROOT
    / "two_sensor_cross_speed_v14_runs/2026-07-22_ab06_cross_speed_v14"
)

EXPECTED_PARENT_HASHES = {
    "protocol": "bdc44f93ed88e94b5afb867f30bb9b1507ebc44b7883be2c96224c37f0ae026c",
    "ledger": "afe6dfe204d48984aeb7910344707d03a6f8bfdbc49fa77bbf2114bd2438475a",
    "runner": "1f346643efe02cdcc32e2511302db45373c3e2eb076f68af8152459d9e298a6e",
    "tests": "7057133ffb7d40198055b7d21bbdee8e258b143089240a112ed5bc42fdaf023e",
    "converter": "015f7e96f4f3dd6a923e3c1115d4f41d07faf1d997419138e11e9d6f9d8433cb",
    "run_start_receipt": "22c1f986472a96cd8a66b3577e5a9046e2f6a4a0812fd19ea3080154c629b245",
    "failure": "4f72c22183a63a084653081ce4f40bfd6097e8e2dd12abd6ccd694a968eb4b58",
}
EXPECTED_LINEAGE_SHA256 = (
    "60590bbcc69f8095add54e66042c1751216dfeb6d443f2bddc355e392818d139"
)
EXPECTED_RECOVERY_CONVERTER_SHA256 = (
    "aa295bab1af42347efed7d445dacdbee2aede36f5ee4368952f20644d1a55b80"
)
EXPECTED_RECOVERY_CONVERTER_TEST_SHA256 = (
    "ef282bf5c47ab2921f44acc4520552f66766cf8191c8f8dd76580b0c00167669"
)
PARENT_SCIENTIFIC_CORE_SHA256 = (
    "06f70ad7c80fc8a894c49d558ec15e9b2506bb489477a8ec063719491a3a9268"
)
ANALYSIS_TIME_LATTICE_S = 0.005
MINIMUM_EDGE_INSET_S = 0.010
ANALYSIS_INTERVALS_S = {
    "02": [9.875, 153.080],
    "03": [8.865, 154.400],
    "04": [12.485, 156.025],
    "05": [15.245, 158.940],
    "06": [11.165, 155.030],
    "07": [9.855, 153.895],
    "08": [10.690, 154.890],
}

GOLDEN_ROOT = (
    VALIDATION_ROOT
    / "two_sensor_v14_1_preprocessing_golden/2026-07-22_treadmill_01_01"
)
GOLDEN_PATHS = {
    "conversion_manifest": GOLDEN_ROOT / "treadmill_01_01_conversion_manifest.json",
    "execution_receipt": GOLDEN_ROOT / "treadmill_01_01_ik_execution_receipt.json",
    "final_receipt": GOLDEN_ROOT / "treadmill_01_01_ik_receipt.json",
    "parity": GOLDEN_ROOT / "treadmill_01_01_v13_ik_parity.json",
    "generated_ik": GOLDEN_ROOT / "treadmill_01_01_ik.mot",
    "v13_ik": REPO_ROOT / "models/AB06_SEASEA_Threadmill/data/IK_results_AB06_SEASEA.mot",
}
EXPECTED_GOLDEN_HASHES = {
    "conversion_manifest": "2a15c02adab9efc28d8d75319c6c32b0d0654055d0e894367b6cabb6cc1d938a",
    "execution_receipt": "4575cb0a6cf2d140e3852fa41e6bc310576095e0f3d3bc1b4eed29f297290dd8",
    "final_receipt": "0f8337aa5f988938c48865e93cd0dede12a19ef23a548da153a7b9ff1b1db45a",
    "parity": "98285280fd41fe5c415e9b3aa7eb651f55d67847e255da42a68c7e05d6aab678",
    "generated_ik": v14.EXPECTED_GOLDEN_IK_SHA256,
    "v13_ik": v14.EXPECTED_GOLDEN_IK_SHA256,
}

SCIENTIFIC_CORE_KEYS = (
    "split",
    "replay",
    "isolated_parameter_grid",
    "combination_formulas",
    "sampling",
    "unit_boundary_handling",
    "gate_contract",
    "causal_event_diagnostics",
    "detail_reporting",
    "selection",
    "decision_contract",
)


class ProtocolError(v14.ProtocolError):
    """Raised when the V14.1 recovery contract is violated."""


NoClobberError = v14.NoClobberError


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _canonical_sha256(value: Any) -> str:
    raw = json.dumps(
        value, sort_keys=True, separators=(",", ":"), allow_nan=False
    ).encode("utf-8")
    return hashlib.sha256(raw).hexdigest()


def _portable(path: Path) -> str:
    try:
        return path.resolve().relative_to(REPO_ROOT).as_posix()
    except ValueError:
        return path.resolve().as_posix()


def _source_record(path: Path, *, require: bool = True) -> dict[str, str]:
    resolved = path.resolve()
    if not resolved.is_file():
        if require:
            raise ProtocolError(f"missing V14.1 source: {_portable(resolved)}")
        return {
            "path": _portable(resolved),
            "sha256": "<FILL_AFTER_SOURCE_EXISTS>",
        }
    return {"path": _portable(resolved), "sha256": _sha256(resolved)}


def _load_json(path: Path) -> dict[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ProtocolError(f"cannot load V14.1 evidence: {_portable(path)}") from exc
    if not isinstance(payload, dict):
        raise ProtocolError(f"V14.1 evidence is not an object: {_portable(path)}")
    return payload


def _validate_parent_failure() -> dict[str, Any]:
    paths = {
        "protocol": PARENT_PROTOCOL,
        "ledger": PARENT_LEDGER,
        "runner": VALIDATION_ROOT / "sweep_two_sensor_cross_speed_v14.py",
        "tests": VALIDATION_ROOT / "test_two_sensor_cross_speed_v14.py",
        "converter": TOOLS_ROOT / "convert_epic_ab06_tables.py",
        "run_start_receipt": PARENT_OUTPUT_DIR / "run_start_receipt.json",
        "failure": PARENT_OUTPUT_DIR / "failure.json",
    }
    checks = {
        name: path.is_file() and _sha256(path) == EXPECTED_PARENT_HASHES[name]
        for name, path in paths.items()
    }
    inventory = (
        sorted(path.name for path in PARENT_OUTPUT_DIR.iterdir())
        if PARENT_OUTPUT_DIR.is_dir()
        else []
    )
    checks["exact_parent_output_inventory"] = inventory == [
        "failure.json",
        "run_start_receipt.json",
    ]
    failure = _load_json(paths["failure"])
    checks["failed_before_detector_replay"] = (
        failure.get("error")
        == "TableSchemaError: IK, FP, and marker time ranges do not match"
        and failure.get("validation_receipt_exists") is False
        and failure.get("sealed_receipt_exists") is False
    )
    if not all(checks.values()):
        raise ProtocolError(f"parent V14 failure evidence drifted: {checks}")
    return {
        "status": "PASS_PARENT_V14_FAILURE_BOUNDARY_VERIFIED",
        "checks": checks,
        "sources": {name: _source_record(path) for name, path in paths.items()},
        "output_inventory": inventory,
    }


def _validated_recovery_lineage() -> dict[str, Any]:
    if _sha256(RECOVERY_LINEAGE) != EXPECTED_LINEAGE_SHA256:
        raise ProtocolError("V14.1 recovery lineage identity drifted")
    payload = _load_json(RECOVERY_LINEAGE)
    frozen = payload.get("frozen_recovery_change", {})
    warmup = payload.get("warmup_recovery_contract", {})
    if not (
        payload.get("recovery_id") == RECOVERY_ID
        and payload.get("status") == "RECOVERY_PREREGISTRATION_EVIDENCE"
        and frozen.get("frozen_analysis_intervals_s") == ANALYSIS_INTERVALS_S
        and frozen.get("analysis_time_lattice_s") == ANALYSIS_TIME_LATTICE_S
        and frozen.get("minimum_analysis_edge_inset_s") == MINIMUM_EDGE_INSET_S
        and warmup.get("minimum_complete_detector_hs_to_to_hs_cycles") == 1
        and warmup.get("minimum_complete_prescribed_grf_hs_to_to_hs_cycles") == 1
        and warmup.get("warmup_samples_or_events_enter_scoring_denominators")
        is False
    ):
        raise ProtocolError("V14.1 recovery lineage semantics drifted")
    return _source_record(RECOVERY_LINEAGE)


def _validated_recovery_golden() -> dict[str, Any]:
    checks = {
        name: path.is_file() and _sha256(path) == EXPECTED_GOLDEN_HASHES[name]
        for name, path in GOLDEN_PATHS.items()
    }
    checks["live_recovery_converter"] = (
        _sha256(TOOLS_ROOT / "convert_epic_ab06_tables_recovery.py")
        == EXPECTED_RECOVERY_CONVERTER_SHA256
    )
    checks["live_recovery_converter_tests"] = (
        _sha256(VALIDATION_ROOT / "test_convert_epic_ab06_tables_recovery.py")
        == EXPECTED_RECOVERY_CONVERTER_TEST_SHA256
    )
    manifest = _load_json(GOLDEN_PATHS["conversion_manifest"])
    receipt = _load_json(GOLDEN_PATHS["final_receipt"])
    parity = _load_json(GOLDEN_PATHS["parity"])
    coverage = manifest.get("source_time_coverage", {})
    checks.update(
        {
            "coverage": coverage.get("all_sources_cover_required_range") is True,
            "explicit_range": coverage.get("range_origin")
            == "explicit_protocol_range",
            "dataset_ik_not_downstream": coverage.get(
                "dataset_ik_used_downstream"
            )
            is False,
            "final_receipt": receipt.get("status") == "IK_OUTPUT_VERIFIED",
            "byte_parity": parity.get("comparison", {}).get("byte_identical")
            is True,
            "no_adaptive_transform": parity.get("coverage_contract", {}).get(
                "adaptive_crop_resample_or_interpolation_used"
            )
            is False,
        }
    )
    if not all(checks.values()):
        raise ProtocolError(f"V14.1 recovery golden drifted: {checks}")
    return {
        "status": "PASS_VERIFIED_FULL_SPAN_BYTE_PARITY",
        "checks": checks,
        "expected_marker_ik_sha256": v14.EXPECTED_GOLDEN_IK_SHA256,
        "sources": {
            name: _source_record(path) for name, path in GOLDEN_PATHS.items()
        },
    }


def _scientific_core(protocol: Mapping[str, Any]) -> dict[str, Any]:
    core = {key: copy.deepcopy(protocol[key]) for key in SCIENTIFIC_CORE_KEYS}
    # Recovery-only safety evidence is intentionally excluded.  Every V14
    # scoring, selection, and decision field remains byte-for-byte canonical.
    core["unit_boundary_handling"].pop("pre_score_warmup_certificate", None)
    core["selection"]["isolated_arm_root_safety"].pop(
        "warmup_certificate_required", None
    )
    core["selection"]["full_finalist_gate"].pop(
        "warmup_certificate_required", None
    )
    core["trial_plateaus"] = {
        trial_id: copy.deepcopy(trial["plateaus"])
        for trial_id, trial in protocol["trials"].items()
    }
    return core


def _validate_analysis_catalog(
    trials: Mapping[str, Mapping[str, Any]],
) -> None:
    if set(trials) != set(ANALYSIS_INTERVALS_S):
        raise ProtocolError("V14.1 trial catalog differs from frozen trial set")
    for trial_id, trial in trials.items():
        source_start, source_end = map(float, trial["source_trial_interval_s"])
        analysis_start, analysis_end = map(float, trial["trial_interval_s"])
        expected = ANALYSIS_INTERVALS_S[trial_id]
        lattice_start = math.ceil(
            (source_start + MINIMUM_EDGE_INSET_S - 1.0e-12)
            / ANALYSIS_TIME_LATTICE_S
        ) * ANALYSIS_TIME_LATTICE_S
        lattice_end = math.floor(
            (source_end - MINIMUM_EDGE_INSET_S + 1.0e-12)
            / ANALYSIS_TIME_LATTICE_S
        ) * ANALYSIS_TIME_LATTICE_S
        if not (
            abs(analysis_start - expected[0]) <= 1.0e-12
            and abs(analysis_end - expected[1]) <= 1.0e-12
            and abs(analysis_start - lattice_start) <= 1.0e-9
            and abs(analysis_end - lattice_end) <= 1.0e-9
            and analysis_start - source_start >= MINIMUM_EDGE_INSET_S - 1.0e-12
            and source_end - analysis_end >= MINIMUM_EDGE_INSET_S - 1.0e-12
        ):
            raise ProtocolError(f"V14.1 analysis interval drifted for trial {trial_id}")
        for plateau in trial["plateaus"]:
            if not (
                analysis_start < float(plateau["start_s"])
                < float(plateau["end_s"])
                < analysis_end
            ):
                raise ProtocolError(
                    f"V14.1 plateau escapes analysis interval for trial {trial_id}"
                )


def expected_protocol_payload(
    *, require_all_sources: bool = True
) -> dict[str, Any]:
    """Return V14.1 without opening any trial stream."""

    parent_evidence = _validate_parent_failure()
    lineage = _validated_recovery_lineage()
    golden = _validated_recovery_golden()
    parent_protocol = _load_json(PARENT_PROTOCOL)
    live_parent_protocol = v14.expected_protocol_payload(
        require_metadata_audit=True,
        require_all_sources=True,
    )
    if live_parent_protocol != parent_protocol:
        raise ProtocolError("live inherited V14 sources drifted from parent protocol")
    if _canonical_sha256(_scientific_core(parent_protocol)) != PARENT_SCIENTIFIC_CORE_SHA256:
        raise ProtocolError("parent V14 scientific-core hash drifted")

    payload = copy.deepcopy(parent_protocol)
    payload["protocol_id"] = PROTOCOL_ID
    payload["stage"] = (
        "v14_1_preprocessing_recovery_then_cross_speed_development_"
        "then_one_shot_validation_and_sealed"
    )
    payload["objective"] = (
        "Recover the preprocessing-only V14 failure under a new one-shot "
        "identity, then execute the unchanged V14 cross-speed geometry "
        "objective and gates."
    )
    payload["interpretation_limits"] = [
        *parent_protocol["interpretation_limits"],
        "V14.1 changes only the preregistered preprocessing coverage and continuous-state warmup certification; it does not constitute a new scientific search design.",
        "Analysis timestamps remain absolute and are frozen on a global 5 ms lattice; no source-dependent cropping, resampling, interpolation, or time rezeroing is allowed.",
        "Warmup events and samples certify FSM maturity only and never enter any plateau or trial scoring denominator.",
    ]
    for trial_id, trial in payload["trials"].items():
        source_interval = list(trial["trial_interval_s"])
        trial["source_trial_interval_s"] = source_interval
        trial["trial_interval_s"] = list(ANALYSIS_INTERVALS_S[trial_id])
        trial["analysis_interval_origin"] = (
            "frozen_inward_global_5ms_lattice_after_minimum_10ms_edge_inset"
        )
    _validate_analysis_catalog(payload["trials"])

    payload["preprocessing_golden"] = golden
    payload["preprocessing"].update(
        {
            "marker_converter": "tools/convert_epic_ab06_tables_recovery.py",
            "force_plate_converter": "tools/convert_epic_ab06_tables_recovery.py",
            "trial_time_range_source": "frozen_v14_1_analysis_intervals",
            "source_time_contract": (
                "every_source_covers_exact_absolute_analysis_interval"
            ),
            "source_streams_preserved_without_crop_resample_or_interpolation": True,
            "exact_ik_setup_and_simulation_interval_required": True,
            "absolute_timestamps_no_rezero": True,
            "development_trial_access_receipt_before_raw_decode": True,
            "preprocessing_lock_required_before_detector_replay": True,
        }
    )
    payload["unit_boundary_handling"]["pre_score_warmup_certificate"] = {
        "scope": "candidate_x_trial_x_cadence",
        "window_end": "first_scoreable_hs_minus_left_context_90ms",
        "minimum_detector_complete_cycles": 1,
        "minimum_prescribed_complete_cycles": 1,
        "zero_timeout_or_invalid": True,
        "known_nonfailure_state_at_boundary": True,
        "enters_score_denominators": False,
        "v13_failure": "abort_protocol",
        "nonbaseline_failure": "full_gate_false_and_root_safe_rejection",
    }
    payload["selection"]["isolated_arm_root_safety"][
        "warmup_certificate_required"
    ] = True
    payload["selection"]["full_finalist_gate"][
        "warmup_certificate_required"
    ] = True
    payload["execution_destination"] = {
        "canonical_output_dir": _portable(DEFAULT_OUTPUT_DIR),
        "canonical_one_shot_ledger": _portable(DEFAULT_EXECUTION_LEDGER),
        "alternate_output_directory_allowed": False,
        "ledger_written_before_any_development_semantic_decode": True,
    }
    payload["recovery"] = {
        "recovery_id": RECOVERY_ID,
        "lineage": lineage,
        "parent_failure": parent_evidence,
        "parent_scientific_core_schema": 1,
        "parent_scientific_core_sha256": PARENT_SCIENTIFIC_CORE_SHA256,
        "analysis_time_lattice_s": ANALYSIS_TIME_LATTICE_S,
        "minimum_edge_inset_s": MINIMUM_EDGE_INSET_S,
        "frozen_analysis_intervals_s": copy.deepcopy(ANALYSIS_INTERVALS_S),
        "parent_destination_reused": False,
        "parent_ledger_reused": False,
    }
    recovery_sources = {
        "v14_1_harness": VALIDATION_ROOT / "sweep_two_sensor_cross_speed_v14_1.py",
        "v14_1_tests": VALIDATION_ROOT / "test_two_sensor_cross_speed_v14_1.py",
        "v14_1_warmup": VALIDATION_ROOT / "two_sensor_v14_1_warmup.py",
        "v14_1_warmup_tests": VALIDATION_ROOT / "test_two_sensor_v14_1_warmup.py",
        "recovery_converter": TOOLS_ROOT / "convert_epic_ab06_tables_recovery.py",
        "recovery_converter_tests": VALIDATION_ROOT / "test_convert_epic_ab06_tables_recovery.py",
        "recovery_lineage": RECOVERY_LINEAGE,
        "parent_v14_protocol": PARENT_PROTOCOL,
        "parent_v14_ledger": PARENT_LEDGER,
        "runtime_mesh_sweep_core": VALIDATION_ROOT
        / "sweep_two_sensor_mesh_placements_prescribed.py",
        "runtime_dual_stream_core": VALIDATION_ROOT
        / "diagnose_two_sensor_dual_stream_prescribed.py",
        "runtime_threshold_gate_core": VALIDATION_ROOT
        / "sweep_two_sensor_prescribed_thresholds.py",
        "runtime_prescribed_replay_core": VALIDATION_ROOT
        / "validate_two_sensor_prescribed_replay.py",
        "runtime_online_grf_validation_core": VALIDATION_ROOT
        / "validate_online_grf.py",
        "runtime_online_grf_event_core": VALIDATION_ROOT
        / "validate_online_grf_events.py",
        "runtime_mesh_comparison_core": VALIDATION_ROOT
        / "compare_two_sensor_mesh_profiles_prescribed.py",
        "runtime_mesh_audit_core": VALIDATION_ROOT
        / "audit_two_sensor_prescribed_geometry.py",
        "runtime_mesh_builder_core": VALIDATION_ROOT
        / "build_two_sensor_mesh_profile_v4.py",
        "runtime_prosthetic_fsm": REPO_ROOT
        / "Trajectory Generator/prosthetic_phase_fsm.py",
        "runtime_online_grf": REPO_ROOT / "online_grf.py",
        "runtime_model_loader": REPO_ROOT / "model_loader.py",
        "runtime_kinematics_interpolator": REPO_ROOT
        / "kinematics_interpolator.py",
        "runtime_output": REPO_ROOT / "output.py",
        "runtime_config": REPO_ROOT / "config.py",
        "runtime_path_resolver": REPO_ROOT / "path_resolver.py",
    }
    payload["sources"].update(
        {
            name: _source_record(path, require=require_all_sources)
            for name, path in recovery_sources.items()
        }
    )
    if _canonical_sha256(_scientific_core(payload)) != PARENT_SCIENTIFIC_CORE_SHA256:
        raise ProtocolError("V14.1 scientific core is not exact-equal to V14")
    return payload


def load_and_validate_protocol(path: str | Path = DEFAULT_PROTOCOL) -> dict[str, Any]:
    protocol_path = v14.v1.resolve_repo_path(path).resolve()
    if protocol_path != DEFAULT_PROTOCOL.resolve():
        raise ProtocolError("V14.1 execution requires the canonical protocol path")
    raw = _load_json(protocol_path)
    expected = expected_protocol_payload(require_all_sources=True)
    if raw != expected:
        differing = sorted(
            key for key in set(raw) | set(expected) if raw.get(key) != expected.get(key)
        )
        raise ProtocolError(f"V14.1 frozen protocol drifted: {differing}")
    raw["_protocol_path"] = protocol_path.as_posix()
    raw["_protocol_sha256"] = _sha256(protocol_path)
    raw["_primary_event_time_field"] = "confirmed_time_s"
    raw["_diagnostic_event_time_field"] = "event_time_s"
    raw["_phase_reference_mode"] = "validated_event_intervals"
    raw["profile_paths"] = {v14.v1.CURRENT_PROFILE_ID: v14.BASELINE_PROFILE_PATH}
    return raw


@dataclass
class _RuntimeState:
    protocol: Mapping[str, Any]
    output_dir: Path
    stage_access_receipts: dict[str, Path]
    preprocessing_locks: dict[str, Path]
    preprocessing_lock_sha256: dict[str, str]
    prescribed_warmup_events: dict[str, dict[str, np.ndarray]]
    warmup_certificates: dict[str, dict[str, Any]]


_RUNTIME: _RuntimeState | None = None
_PARENT_WRITE_JSON = v14._write_json_exclusive


def _active() -> _RuntimeState:
    if _RUNTIME is None:
        raise ProtocolError("V14.1 runtime hook used outside configured execution")
    return _RUNTIME


def _trial_access_receipt(
    *, protocol: Mapping[str, Any], trial_id: str, stage: str, output_dir: Path
) -> Path:
    ledger = DEFAULT_EXECUTION_LEDGER
    if not ledger.is_file():
        raise ProtocolError("V14.1 ledger must precede development raw access")
    path = output_dir / f"{stage}_trial_{trial_id}_preprocessing_access_receipt.json"
    return _PARENT_WRITE_JSON(
        path,
        {
            "schema_version": SCHEMA_VERSION,
            "status": "V14_1_PREPROCESSING_SEMANTIC_ACCESS_STARTED",
            "stage": stage,
            "trial_id": trial_id,
            "semantic_access_started": True,
            "protocol": {
                "path": _portable(Path(str(protocol["_protocol_path"]))),
                "sha256": protocol["_protocol_sha256"],
                "protocol_id": protocol["protocol_id"],
            },
            "execution_ledger": _source_record(ledger),
            "recovery_lineage": _source_record(RECOVERY_LINEAGE),
            "written_before_raw_identity_check_or_semantic_decode": True,
            "rerun_allowed": False,
        },
    )


def _validate_conversion_manifest(
    path: Path,
    required_range: Sequence[float],
    *,
    expected_trial: str,
    raw_sources: Mapping[str, Mapping[str, Any]],
) -> dict[str, Any]:
    manifest = _load_json(path)
    coverage = manifest.get("source_time_coverage", {})
    streams = coverage.get("streams", {})
    try:
        required = [float(required_range[0]), float(required_range[1])]
    except (IndexError, TypeError, ValueError) as exc:
        raise ProtocolError("invalid V14.1 required conversion range") from exc
    original = manifest.get("recovery_lineage", {}).get("original_converter", {})
    original_path = (
        path.parent / str(original.get("path", ""))
    ).resolve()
    expected_sources = {
        manifest_role: {
            "filename": Path(str(raw_sources[protocol_role]["path"])).name,
            "sha256": str(raw_sources[protocol_role]["sha256"]),
        }
        for manifest_role, protocol_role in (
            ("ik", "ik"),
            ("force_plate", "fp"),
            ("markers", "markers"),
        )
    }
    stream_ranges_ok = bool(
        set(streams) == {"dataset_ik", "force_plate", "markers"}
        and all(
            isinstance(item, Mapping)
            and item.get("covers_required_range") is True
            and isinstance(item.get("rows"), int)
            and not isinstance(item.get("rows"), bool)
            and int(item["rows"]) >= 2
            and isinstance(item.get("time_range_s"), list)
            and len(item["time_range_s"]) == 2
            and all(math.isfinite(float(value)) for value in item["time_range_s"])
            and float(item["time_range_s"][0]) <= required[0] + 1.0e-9
            and float(item["time_range_s"][1]) >= required[1] - 1.0e-9
            for item in streams.values()
        )
    )
    if not (
        len(required) == 2
        and all(math.isfinite(value) for value in required)
        and required[1] > required[0]
        and manifest.get("schema_version") == 1
        and manifest.get("status") == "CONVERTED"
        and manifest.get("trial") == expected_trial
        and manifest.get("sources") == expected_sources
        and manifest.get("time_range_s") == required
        and original_path == (TOOLS_ROOT / "convert_epic_ab06_tables.py").resolve()
        and original.get("sha256") == EXPECTED_PARENT_HASHES["converter"]
        and original_path.is_file()
        and _sha256(original_path) == EXPECTED_PARENT_HASHES["converter"]
        and coverage.get("schema_version") == 1
        and float(coverage.get("endpoint_tolerance_s", math.nan)) == 1.0e-9
        and coverage.get("coverage_rule")
        == "every_source_covers_downstream_required_time_range"
        and coverage.get("range_origin") == "explicit_protocol_range"
        and coverage.get("downstream_required_time_range_s")
        == required
        and coverage.get("all_sources_cover_required_range") is True
        and coverage.get("dataset_ik_used_downstream") is False
        and stream_ranges_ok
        and manifest.get("inverse_kinematics", {}).get("time_range_s")
        == required
    ):
        raise ProtocolError("V14.1 conversion coverage/IK interval drifted")
    return manifest


def _validate_preprocessed_files(
    manifest: Mapping[str, Any],
    *,
    work_dir: Path,
    stem: str,
    trc: Path,
    grf: Path,
    external: Path,
    ik_setup: Path,
    ik_motion: Path,
) -> dict[str, dict[str, str]]:
    """Bind every converted/replayed file to its canonical path and hash."""

    root = work_dir.resolve()
    paths = {
        "dataset_ik": root / f"{stem}_ik_dataset_ab06_seasea.mot",
        "markers_trc": trc.resolve(),
        "grf": grf.resolve(),
        "external_loads": external.resolve(),
        "ik_setup": ik_setup.resolve(),
        "ik_motion": ik_motion.resolve(),
    }
    expected_paths = {
        "dataset_ik": root / f"{stem}_ik_dataset_ab06_seasea.mot",
        "markers_trc": root / f"{stem}.trc",
        "grf": root / f"{stem}_grf.mot",
        "external_loads": root / f"{stem}_ExternalLoads.xml",
        "ik_setup": root / f"{stem}_iksetup.xml",
        "ik_motion": root / f"{stem}_ik.mot",
    }
    if paths != expected_paths or any(not path.is_file() for path in paths.values()):
        raise ProtocolError("V14.1 preprocessed file path/inventory drifted")
    records = {name: _source_record(path) for name, path in paths.items()}
    outputs = manifest.get("outputs", {})
    for role in ("dataset_ik", "markers_trc", "grf", "external_loads"):
        if outputs.get(role) != {
            "path": paths[role].name,
            "sha256": records[role]["sha256"],
        }:
            raise ProtocolError(f"V14.1 manifest output drifted: {role}")
    inverse = manifest.get("inverse_kinematics", {})
    if inverse.get("setup") != {
        "path": paths["ik_setup"].name,
        "sha256": records["ik_setup"]["sha256"],
    }:
        raise ProtocolError("V14.1 IK setup manifest link drifted")
    return records


def _validate_final_ik_receipt(
    receipt_path: Path,
    *,
    work_dir: Path,
    stem: str,
    conversion_manifest: Path,
    ik_setup: Path,
    ik_motion: Path,
    required_range: Sequence[float],
) -> dict[str, Any]:
    """Validate the complete live receipt chain consumed by replay."""

    receipt = receipt_path.resolve()
    root = work_dir.resolve()
    execution = root / f"{stem}_ik_execution_receipt.json"
    expected_receipt = root / f"{stem}_ik_receipt.json"
    if receipt != expected_receipt or not execution.is_file():
        raise ProtocolError("V14.1 IK receipt path/chain is not canonical")
    payload = _load_json(receipt)
    output = payload.get("output_ik", {})
    output_range = output.get("time_range_s")
    marker = payload.get("marker_contract", {})
    if not (
        payload.get("status") == "IK_OUTPUT_VERIFIED"
        and payload.get("trial") == stem
        and payload.get("opensim_version") == "4.5.2"
        and payload.get("conversion_manifest")
        == {
            "path": conversion_manifest.name,
            "sha256": _sha256(conversion_manifest),
        }
        and payload.get("execution_receipt")
        == {"path": execution.name, "sha256": _sha256(execution)}
        and payload.get("setup")
        == {"path": ik_setup.name, "sha256": _sha256(ik_setup)}
        and payload.get("model", {}).get("sha256")
        == v14.metadata.EXPECTED_MARKER_CALIBRATED_MODEL_SHA256
        and payload.get("plugin", {}).get("binary_sha256")
        in set(v14.EXPECTED_PLUGIN_BINARY_SHA256_BY_SUFFIX.values())
        and marker.get("count") == 28
        and marker.get("apply") is True
        and float(marker.get("weight", math.nan)) == 1.0
        and float(marker.get("accuracy", math.nan)) == 1.0e-5
        and output.get("path") == ik_motion.name
        and output.get("sha256") == _sha256(ik_motion)
        and output.get("coordinate_count") == 21
        and isinstance(output.get("rows"), int)
        and not isinstance(output.get("rows"), bool)
        and int(output["rows"]) >= 2
        and isinstance(output_range, list)
        and len(output_range) == 2
        and float(output_range[0]) <= float(required_range[0]) + 1.0e-9
        and float(output_range[1]) >= float(required_range[1]) - 1.0e-9
    ):
        raise ProtocolError("V14.1 final IK receipt chain drifted")
    return payload


def _validated_live_replay_inputs(
    protocol: Mapping[str, Any], setup: Any
) -> dict[str, Any]:
    """Re-hash every repository input reopened by detector replay."""

    model = v14.v1.resolve_repo_path(protocol["model_file"]).resolve()
    reserve = v14.v1.resolve_repo_path(protocol["reserve_actuators_xml"]).resolve()
    load_profile = v14.v1.resolve_repo_path(
        protocol["load_evidence_profile"]
    ).resolve()
    plugin_basename = v14.v1.resolve_repo_path(
        str(protocol["replay"]["sea_plugin"])
    ).resolve()
    if Path(str(protocol["replay"]["sea_plugin"])).suffix:
        raise ProtocolError("V14.1 replay plugin basename gained an extension")
    plugin = v14.converter.resolve_plugin_spec(plugin_basename)
    expected_plugin_hash = v14.EXPECTED_PLUGIN_BINARY_SHA256_BY_SUFFIX.get(
        plugin.binary.suffix.lower()
    )
    records = {
        "model": _source_record(model),
        "reserve_actuators": _source_record(reserve),
        "load_evidence_profile": _source_record(load_profile),
        "plugin": {
            "loader_basename": _portable(plugin.loader_basename),
            "binary_path": _portable(plugin.binary),
            "binary_sha256": _sha256(plugin.binary),
        },
    }
    sources = protocol["sources"]
    if not (
        Path(setup.model_file).resolve() == model
        and Path(setup.reserve_actuators_xml).resolve() == reserve
        and setup.grf_mode == "prescribed"
        and setup.online_grf_profile_file is None
        and plugin.loader_basename == plugin_basename
        and expected_plugin_hash is not None
        and records["plugin"]["binary_sha256"] == expected_plugin_hash
        and records["model"] == sources["marker_calibrated_model"]
        and records["reserve_actuators"] == sources["reserve_actuators"]
        and records["load_evidence_profile"] == sources["load_evidence_profile"]
    ):
        raise ProtocolError("V14.1 live replay model/plugin/profile inputs drifted")
    return records


def _prepare_trial_hook(
    protocol: Mapping[str, Any],
    *,
    trial_id: str,
    stage: str,
    work_dir: Path,
    access_receipt: Path | None,
) -> Any:
    """Recovery equivalent of V14.prepare_trial with receipt-first access."""

    state = _active()
    expected_work_dir = (
        state.output_dir / "preprocessed" / f"trial_{trial_id}"
    ).resolve()
    if work_dir.resolve() != expected_work_dir:
        raise ProtocolError("V14.1 trial preprocessing path is not canonical")
    v14._assert_semantic_access(
        protocol,
        trial_id=trial_id,
        stage=stage,
        access_receipt=access_receipt,
    )
    if stage == "development":
        receipt = _trial_access_receipt(
            protocol=protocol,
            trial_id=trial_id,
            stage=stage,
            output_dir=state.output_dir,
        )
    else:
        if access_receipt is None or not access_receipt.is_file():
            raise ProtocolError(f"{stage} access receipt missing before preprocessing")
        receipt = access_receipt
    state.stage_access_receipts[trial_id] = receipt

    trial = protocol["trials"][trial_id]
    v14._verify_raw_identity(trial["raw_sources"]["conditions"])
    dataset_ik_path = v14._verify_raw_identity(trial["raw_sources"]["ik"])
    fp_path = v14._verify_raw_identity(trial["raw_sources"]["fp"])
    marker_path = v14._verify_raw_identity(trial["raw_sources"]["markers"])
    if work_dir.exists():
        raise NoClobberError(f"trial work directory exists: {_portable(work_dir)}")

    required_range = [float(x) for x in trial["trial_interval_s"]]
    if required_range != ANALYSIS_INTERVALS_S[trial_id]:
        raise ProtocolError("V14.1 prepare range is not the frozen analysis interval")
    stem = f"treadmill_{trial_id}_01"
    model_file = v14.v1.resolve_repo_path(protocol["model_file"]).resolve()
    if _sha256(model_file) != v14.metadata.EXPECTED_MARKER_CALIBRATED_MODEL_SHA256:
        raise ProtocolError("marker-calibrated model hash drifted before V14.1 IK")
    plugin = v14.v1.resolve_repo_path(str(protocol["replay"]["sea_plugin"])).resolve()
    if Path(str(protocol["replay"]["sea_plugin"])).suffix:
        raise ProtocolError("SEA plugin must be an extensionless basename")
    plugin_spec = v14.converter.resolve_plugin_spec(plugin)
    expected_plugin_hash = v14.EXPECTED_PLUGIN_BINARY_SHA256_BY_SUFFIX.get(
        plugin_spec.binary.suffix.lower()
    )
    if expected_plugin_hash is None or _sha256(plugin_spec.binary) != expected_plugin_hash:
        raise ProtocolError("SEA plugin binary identity drifted before V14.1 conversion")

    converted = recovery_converter.convert_trial(
        ik_mat=dataset_ik_path,
        fp_mat=fp_path,
        markers_mat=marker_path,
        output_dir=work_dir,
        trial=stem,
        ik_model=model_file,
        ik_plugin=plugin,
        required_time_range_s=required_range,
    )
    conversion_manifest = Path(str(converted["conversion_manifest"])).resolve()
    if conversion_manifest != work_dir / f"{stem}_conversion_manifest.json":
        raise ProtocolError("V14.1 conversion manifest path is not canonical")
    manifest = _validate_conversion_manifest(
        conversion_manifest,
        required_range,
        expected_trial=stem,
        raw_sources=trial["raw_sources"],
    )
    ik_setup = Path(str(converted["ik_setup_xml"])).resolve()
    ik_motion = work_dir / f"{stem}_ik.mot"
    recovery_converter.run_ik_from_setup(setup_xml=ik_setup, ik_plugin=plugin)
    finalized = recovery_converter.finalize_ik_receipt(
        setup_xml=ik_setup,
        output_ik_mot=ik_motion,
        ik_plugin=plugin,
    )
    ik_receipt = Path(str(finalized["receipt"])).resolve()
    if (
        ik_receipt != work_dir / f"{stem}_ik_receipt.json"
        or not ik_receipt.is_file()
        or finalized.get("status") != "IK_OUTPUT_VERIFIED"
    ):
        raise ProtocolError("final V14.1 marker-based IK receipt is missing")
    _validate_final_ik_receipt(
        ik_receipt,
        work_dir=work_dir,
        stem=stem,
        conversion_manifest=conversion_manifest,
        ik_setup=ik_setup,
        ik_motion=ik_motion,
        required_range=required_range,
    )

    trc = Path(str(converted["trc"])).resolve()
    grf = Path(str(converted["grf_mot"])).resolve()
    external = Path(str(converted["external_loads_xml"])).resolve()
    preprocessed_files = _validate_preprocessed_files(
        manifest,
        work_dir=work_dir,
        stem=stem,
        trc=trc,
        grf=grf,
        external=external,
        ik_setup=ik_setup,
        ik_motion=ik_motion,
    )
    setup = v14.setup_io.build_simulation_setup(
        model_file=model_file,
        kinematics_file=ik_motion,
        external_loads_xml=external,
        reserve_actuators_xml=v14.v1.resolve_repo_path(
            protocol["reserve_actuators_xml"]
        ).resolve(),
        t_start=required_range[0],
        t_end=required_range[1],
        grf_mode="prescribed",
    )
    if [float(setup.t_start), float(setup.t_end)] != required_range:
        raise ProtocolError("V14.1 SimulationSetup interval drifted")
    replay_inputs = _validated_live_replay_inputs(protocol, setup)

    lock = _PARENT_WRITE_JSON(
        work_dir / f"{stem}_preprocessing_lock.json",
        {
            "schema_version": SCHEMA_VERSION,
            "status": "V14_1_PREPROCESSING_FROZEN_BEFORE_DETECTOR_REPLAY",
            "trial_id": trial_id,
            "stage": stage,
            "analysis_interval_s": required_range,
            "absolute_timestamps_no_rezero": True,
            "adaptive_crop_resample_or_interpolation_used": False,
            "dataset_ik_used_downstream": False,
            "all_sources_cover_analysis_interval": True,
            "source_time_coverage": manifest["source_time_coverage"],
            "access_receipt": _source_record(receipt),
            "conversion_manifest": _source_record(conversion_manifest),
            "ik_receipt": _source_record(ik_receipt),
            "ik_execution_receipt": _source_record(
                work_dir / f"{stem}_ik_execution_receipt.json"
            ),
            "ik_motion": _source_record(ik_motion),
            "preprocessed_files": preprocessed_files,
            "live_replay_inputs": replay_inputs,
            "recovery_lineage": _source_record(RECOVERY_LINEAGE),
        },
    )
    state.preprocessing_locks[trial_id] = lock
    state.preprocessing_lock_sha256[trial_id] = _sha256(lock)
    return v14.TrialArtifacts(
        trial_id=trial_id,
        stage=stage,
        setup=setup,
        work_dir=work_dir,
        trc=trc,
        grf=grf,
        external_loads=external,
        ik_setup=ik_setup,
        ik_motion=ik_motion,
        ik_receipt=ik_receipt,
        conversion_manifest=conversion_manifest,
    )


def _validated_preprocessing_lock(
    protocol: Mapping[str, Any], artifacts: Any
) -> Path:
    state = _active()
    trial_id = str(artifacts.trial_id)
    stem = f"treadmill_{trial_id}_01"
    expected_work_dir = (
        state.output_dir / "preprocessed" / f"trial_{trial_id}"
    ).resolve()
    if Path(artifacts.work_dir).resolve() != expected_work_dir:
        raise ProtocolError("V14.1 replay work directory is not canonical")
    lock = state.preprocessing_locks.get(trial_id)
    expected_lock = (
        Path(artifacts.work_dir)
        / f"treadmill_{trial_id}_01_preprocessing_lock.json"
    ).resolve()
    if lock is None or lock.resolve() != expected_lock or not lock.is_file():
        raise ProtocolError("V14.1 preprocessing lock must precede detector replay")
    frozen_hash = state.preprocessing_lock_sha256.get(trial_id)
    if frozen_hash is None or _sha256(lock) != frozen_hash:
        raise ProtocolError("V14.1 preprocessing lock identity drifted")
    payload = _load_json(lock)
    required_range = protocol["trials"][trial_id]["trial_interval_s"]
    receipt = state.stage_access_receipts.get(trial_id)
    if receipt is None or not receipt.is_file():
        raise ProtocolError("V14.1 preprocessing access receipt drifted")
    conversion_manifest = Path(artifacts.conversion_manifest).resolve()
    ik_receipt = Path(artifacts.ik_receipt).resolve()
    ik_motion = Path(artifacts.ik_motion).resolve()
    manifest = _validate_conversion_manifest(
        conversion_manifest,
        required_range,
        expected_trial=stem,
        raw_sources=protocol["trials"][trial_id]["raw_sources"],
    )
    _validate_final_ik_receipt(
        ik_receipt,
        work_dir=Path(artifacts.work_dir),
        stem=stem,
        conversion_manifest=conversion_manifest,
        ik_setup=Path(artifacts.ik_setup),
        ik_motion=ik_motion,
        required_range=required_range,
    )
    preprocessed_files = _validate_preprocessed_files(
        manifest,
        work_dir=Path(artifacts.work_dir),
        stem=stem,
        trc=Path(artifacts.trc),
        grf=Path(artifacts.grf),
        external=Path(artifacts.external_loads),
        ik_setup=Path(artifacts.ik_setup),
        ik_motion=ik_motion,
    )
    replay_inputs = _validated_live_replay_inputs(protocol, artifacts.setup)
    coverage = payload.get("source_time_coverage", {})
    streams = coverage.get("streams", {})
    if not (
        payload.get("status")
        == "V14_1_PREPROCESSING_FROZEN_BEFORE_DETECTOR_REPLAY"
        and payload.get("trial_id") == trial_id
        and payload.get("stage") == artifacts.stage
        and payload.get("analysis_interval_s") == required_range
        and payload.get("absolute_timestamps_no_rezero") is True
        and payload.get("adaptive_crop_resample_or_interpolation_used") is False
        and payload.get("dataset_ik_used_downstream") is False
        and payload.get("all_sources_cover_analysis_interval") is True
        and coverage.get("downstream_required_time_range_s") == required_range
        and coverage.get("all_sources_cover_required_range") is True
        and coverage.get("dataset_ik_used_downstream") is False
        and set(streams) == {"dataset_ik", "force_plate", "markers"}
        and all(
            item.get("covers_required_range") is True
            for item in streams.values()
        )
        and payload.get("access_receipt") == _source_record(receipt)
        and payload.get("conversion_manifest")
        == _source_record(conversion_manifest)
        and payload.get("ik_receipt") == _source_record(ik_receipt)
        and payload.get("ik_execution_receipt")
        == _source_record(
            Path(artifacts.work_dir) / f"{stem}_ik_execution_receipt.json"
        )
        and payload.get("ik_motion") == _source_record(ik_motion)
        and payload.get("preprocessed_files") == preprocessed_files
        and payload.get("live_replay_inputs") == replay_inputs
        and payload.get("recovery_lineage") == _source_record(RECOVERY_LINEAGE)
        and Path(artifacts.setup.kinematics_file).resolve() == ik_motion
        and Path(artifacts.setup.external_loads_xml).resolve()
        == Path(artifacts.external_loads).resolve()
        and float(artifacts.setup.t_start) == float(required_range[0])
        and float(artifacts.setup.t_end) == float(required_range[1])
    ):
        raise ProtocolError("V14.1 preprocessing lock drifted before replay")
    return lock


def _evaluate_trial_hook(
    protocol: Mapping[str, Any], artifacts: Any, *args: Any, **kwargs: Any
) -> Any:
    _validated_preprocessing_lock(protocol, artifacts)
    return _PARENT_EVALUATE_TRIAL(protocol, artifacts, *args, **kwargs)


_PARENT_EVALUATE_TRIAL = v14._evaluate_trial


_PARENT_EVALUATE_CANDIDATE = v14.evaluate_continuous_candidate
_PARENT_RUN_FSM = v14.v1._run_production_fsm
_PARENT_ROOT_SAFE = v14.root_safe_isolated


def _prescribed_warmup_events(
    protocol: Mapping[str, Any], bundle: Any, first_scoreable_hs_s: float
) -> dict[str, np.ndarray]:
    """Return prescribed cycles through the first scoreable HS.

    The short post-HS tail lets the existing sustained-contact extractor retain
    that HS as the closing edge.  It never enters detector state or scoring.
    """

    state = _active()
    trial_id = str(bundle.trial_id)
    cached = state.prescribed_warmup_events.get(trial_id)
    if cached is not None:
        if not np.any(
            np.isclose(
                cached["heel_strike"],
                float(first_scoreable_hs_s),
                rtol=0.0,
                atol=1.0e-12,
            )
        ):
            raise ProtocolError("cached prescribed warmup anchor drifted")
        return cached
    replay = protocol["replay"]
    minimum_contact_s = float(replay["reference_min_contact_duration_s"])
    tail_s = minimum_contact_s + max(float(bundle.sample_dt_s), 0.001)
    source_setup = bundle.shared["setup"]
    warmup_end = min(
        float(source_setup.t_end), float(first_scoreable_hs_s) + tail_s
    )
    warmup_setup = replace(
        source_setup,
        t_start=float(protocol["trials"][trial_id]["trial_interval_s"][0]),
        t_end=warmup_end,
    )
    try:
        events, _provenance = v14.v1._reference_events_from_prescribed_grf(
            warmup_setup,
            threshold_n=float(replay["prescribed_contact_threshold_n"]),
            min_contact_duration_s=minimum_contact_s,
            min_cycle_duration_s=float(replay["reference_min_cycle_duration_s"]),
        )
    except (OSError, ValueError) as exc:
        raise ProtocolError("cannot derive prescribed V14.1 warmup cycles") from exc
    if not np.any(
        np.isclose(
            events["heel_strike"],
            float(first_scoreable_hs_s),
            rtol=0.0,
            atol=1.0e-12,
        )
    ):
        raise ProtocolError("prescribed warmup does not reach first scoreable HS")
    state.prescribed_warmup_events[trial_id] = events
    return events


def _evaluate_candidate_hook(
    protocol: Mapping[str, Any], base: Any, candidate: Any, bundle: Any
) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    """Capture the un-sliced replay and attach a pre-score certificate."""

    captured: dict[str, Any] = {}

    def capture_fsm(*args: Any, **kwargs: Any) -> dict[str, Any]:
        if captured:
            raise ProtocolError("candidate evaluation invoked the FSM more than once")
        replay = _PARENT_RUN_FSM(*args, **kwargs)
        captured["replay"] = replay
        return replay

    previous = v14.v1._run_production_fsm
    v14.v1._run_production_fsm = capture_fsm
    try:
        rows, details = _PARENT_EVALUATE_CANDIDATE(
            protocol, base, candidate, bundle
        )
    finally:
        v14.v1._run_production_fsm = previous
    full_replay = captured.get("replay")
    if not isinstance(full_replay, Mapping):
        raise ProtocolError("V14.1 candidate replay was not captured")
    first_scoreable_hs = float(
        bundle.plateau_references[0]["events"]["heel_strike"][0]
    )
    prescribed = _prescribed_warmup_events(
        protocol, bundle, first_scoreable_hs
    )
    detector_events = [
        dict(item)
        for item in full_replay["accepted"]
        if not (
            str(item.get("event", "")) == "toe_off"
            and float(item.get("segment_valid", 1.0)) == 0.0
        )
    ]
    try:
        certificate = warmup.certify_warmup(
            analysis_start_s=float(bundle.shared["times"][0]),
            first_scoreable_hs_s=first_scoreable_hs,
            sample_dt_s=float(bundle.sample_dt_s),
            sample_times_s=np.asarray(bundle.shared["times"], dtype=float),
            state_ids=np.asarray(full_replay["state_id"], dtype=float),
            detector_accepted_events=detector_events,
            detector_invalid_steps=full_replay["invalid_steps"],
            prescribed_events=prescribed,
        )
    except warmup.WarmupValidationError as exc:
        raise ProtocolError("malformed V14.1 warmup evidence") from exc

    is_v13 = str(candidate.candidate_id) == v14.BASELINE_ID
    for index, row in enumerate(rows):
        try:
            applied = warmup.apply_warmup_certificate(
                row,
                certificate,
                is_v13=is_v13,
                protocol_error_type=ProtocolError,
            )
        except warmup.WarmupValidationError as exc:
            raise ProtocolError("V14.1 warmup certificate validation failed") from exc
        applied.update(
            {
                "warmup_detector_complete_cycle_count": int(
                    certificate["detector"]["complete_cycle_count"]
                ),
                "warmup_prescribed_complete_cycle_count": int(
                    certificate["prescribed"]["complete_cycle_count"]
                ),
                "warmup_cutoff_s": float(certificate["cutoff_s"]),
                "warmup_enters_scoring_denominator": False,
            }
        )
        row.clear()
        row.update(applied)
        detail = details[index]
        detail["row"] = row
        detail["warmup_certificate"] = copy.deepcopy(certificate)
        gate = detail.get("dynamic_gate_result")
        if isinstance(gate, dict):
            gate.setdefault("checks", {})[
                "pre_score_warmup_certificate"
            ] = bool(certificate["ok"])
            gate["ok"] = bool(gate.get("ok") and certificate["ok"])

    key = (
        f"{candidate.candidate_id}::trial_{bundle.trial_id}::"
        f"{bundle.cadence_label}"
    )
    _active().warmup_certificates[key] = copy.deepcopy(certificate)
    return rows, details


def _root_safe_hook(
    candidate_rows: Sequence[Mapping[str, Any]],
    baseline_rows: Sequence[Mapping[str, Any]],
) -> dict[str, Any]:
    result = _PARENT_ROOT_SAFE(candidate_rows, baseline_rows)
    try:
        return warmup.apply_root_safe_warmup(result, candidate_rows)
    except warmup.WarmupValidationError as exc:
        raise ProtocolError("malformed root-safety warmup evidence") from exc


def _augment_payload(path: Path, payload: Mapping[str, Any]) -> dict[str, Any]:
    state = _active()
    result = copy.deepcopy(dict(payload))
    name = path.name
    if name == DEFAULT_EXECUTION_LEDGER.name:
        result["status"] = "V14_1_RECOVERY_CANONICAL_DESTINATION_CONSUMED"
        result["recovery_lineage"] = _source_record(RECOVERY_LINEAGE)
        result["parent_v14_ledger_reused"] = False
    elif name == "run_start_receipt.json":
        result["status"] = "V14_1_RECOVERY_DEVELOPMENT_ACCESS_STARTED"
        result["recovery_lineage"] = _source_record(RECOVERY_LINEAGE)
    elif name == "development_candidate_lock.json":
        expected = set(state.protocol["split"]["DEVELOPMENT"])
        if set(state.preprocessing_locks) & expected != expected:
            raise ProtocolError("development lock precedes all preprocessing locks")
        result["recovery_lineage"] = _source_record(RECOVERY_LINEAGE)
        result["development_access_receipts"] = {
            key: _source_record(state.stage_access_receipts[key])
            for key in sorted(expected)
        }
        result["preprocessing_locks"] = {
            key: _source_record(state.preprocessing_locks[key])
            for key in sorted(expected)
        }
        result["warmup_certificate_required"] = True
        development_ids = set(state.protocol["split"]["DEVELOPMENT"])
        development_certificates = {
            key: value
            for key, value in state.warmup_certificates.items()
            if any(f"trial_{trial_id}::" in key for trial_id in development_ids)
        }
        if not development_certificates:
            raise ProtocolError("development lock lacks warmup certificates")
        result["warmup_certificate_index"] = {
            "count": len(development_certificates),
            "all_pass": all(
                bool(item.get("ok")) for item in development_certificates.values()
            ),
            "canonical_sha256_by_candidate_trial_cadence": {
                key: _canonical_sha256(value)
                for key, value in sorted(development_certificates.items())
            },
        }
    elif name == "validation_access_receipt.json":
        result["status"] = "VALIDATION_OPENED_FOR_SINGLE_AUTHORIZED_V14_1_RECOVERY_RUN"
        result["recovery_lineage"] = _source_record(RECOVERY_LINEAGE)
        result["development_preprocessing_locks"] = {
            key: _source_record(value)
            for key, value in sorted(state.preprocessing_locks.items())
            if key in set(state.protocol["split"]["DEVELOPMENT"])
        }
    elif name == "validation_decision_lock.json":
        trial_id = str(state.protocol["split"]["VALIDATION"][0])
        if trial_id not in state.preprocessing_locks:
            raise ProtocolError("validation decision precedes validation preprocessing lock")
        result["recovery_lineage"] = _source_record(RECOVERY_LINEAGE)
        result["validation_preprocessing_lock"] = _source_record(
            state.preprocessing_locks[trial_id]
        )
        result["warmup_certificate_required"] = True
        validation_certificates = {
            key: value
            for key, value in state.warmup_certificates.items()
            if f"trial_{trial_id}::" in key
        }
        if len(validation_certificates) != 4:
            raise ProtocolError("validation warmup certificate cardinality drifted")
        result["warmup_certificate_index"] = {
            "count": len(validation_certificates),
            "all_pass": all(
                bool(item.get("ok")) for item in validation_certificates.values()
            ),
            "canonical_sha256_by_candidate_trial_cadence": {
                key: _canonical_sha256(value)
                for key, value in sorted(validation_certificates.items())
            },
        }
    elif name == "sealed_access_receipt.json":
        result["status"] = "SEALED_OPENED_FOR_SINGLE_AUTHORIZED_V14_1_RECOVERY_RUN"
        validation_id = str(state.protocol["split"]["VALIDATION"][0])
        result["recovery_lineage"] = _source_record(RECOVERY_LINEAGE)
        result["validation_preprocessing_lock"] = _source_record(
            state.preprocessing_locks[validation_id]
        )
    elif name == "manifest.json":
        if result.get("conclusion") == "V14_CHALLENGER_PASSED_ALL_STAGES":
            result["conclusion"] = "V14_1_RECOVERY_CHALLENGER_PASSED_ALL_STAGES"
        result["recovery"] = {
            "recovery_id": RECOVERY_ID,
            "lineage": _source_record(RECOVERY_LINEAGE),
            "parent_destination_reused": False,
            "parent_ledger_reused": False,
            "preprocessing_access_receipts": {
                key: _source_record(value)
                for key, value in sorted(state.stage_access_receipts.items())
            },
            "preprocessing_locks": {
                key: _source_record(value)
                for key, value in sorted(state.preprocessing_locks.items())
            },
            "warmup_certificate_required": True,
            "warmup_certificate_index": {
                "count": len(state.warmup_certificates),
                "pass_count": sum(
                    bool(item.get("ok"))
                    for item in state.warmup_certificates.values()
                ),
                "fail_count": sum(
                    not bool(item.get("ok"))
                    for item in state.warmup_certificates.values()
                ),
                "canonical_sha256_by_candidate_trial_cadence": {
                    key: _canonical_sha256(value)
                    for key, value in sorted(state.warmup_certificates.items())
                },
            },
        }
    elif name == "failure.json":
        result["status"] = "ERROR_AFTER_V14_1_RECOVERY_DESTINATION_CONSUMED"
        result["recovery_lineage"] = _source_record(RECOVERY_LINEAGE)
    return result


def _write_json_hook(path: Path, payload: Mapping[str, Any]) -> Path:
    return _PARENT_WRITE_JSON(path, _augment_payload(path, payload))


@contextmanager
def _configured_parent(
    protocol: Mapping[str, Any], output_dir: Path
) -> Iterator[None]:
    global _RUNTIME
    if _RUNTIME is not None:
        raise ProtocolError("nested V14.1 execution is forbidden")
    patches = {
        "DEFAULT_PROTOCOL": DEFAULT_PROTOCOL,
        "DEFAULT_OUTPUT_DIR": DEFAULT_OUTPUT_DIR,
        "DEFAULT_EXECUTION_LEDGER": DEFAULT_EXECUTION_LEDGER,
        "prepare_trial": _prepare_trial_hook,
        "_evaluate_trial": _evaluate_trial_hook,
        "evaluate_continuous_candidate": _evaluate_candidate_hook,
        "root_safe_isolated": _root_safe_hook,
        "_write_json_exclusive": _write_json_hook,
    }
    previous = {name: getattr(v14, name) for name in patches}
    _RUNTIME = _RuntimeState(protocol, output_dir, {}, {}, {}, {}, {})
    try:
        for name, value in patches.items():
            setattr(v14, name, value)
        yield
    finally:
        for name, value in previous.items():
            setattr(v14, name, value)
        _RUNTIME = None


def _preflight_no_clobber(output_dir: Path) -> None:
    if output_dir.resolve() != DEFAULT_OUTPUT_DIR.resolve():
        raise ProtocolError("V14.1 execution is bound to its canonical output directory")
    if DEFAULT_EXECUTION_LEDGER.exists():
        raise NoClobberError("V14.1 canonical one-shot ledger already exists")
    if output_dir.exists():
        raise NoClobberError(f"refusing occupied V14.1 output: {_portable(output_dir)}")
    # The failed V14 evidence is immutable and must still be exact immediately
    # before consuming the recovery destination.
    _validate_parent_failure()


def _validate_runtime_protocol(protocol: Mapping[str, Any]) -> None:
    """Reject direct-call bypasses before the one-shot ledger can be written."""

    if not isinstance(protocol, Mapping):
        raise ProtocolError("V14.1 runtime protocol must be a mapping")
    canonical = load_and_validate_protocol(DEFAULT_PROTOCOL)
    if dict(protocol) != canonical:
        raise ProtocolError("V14.1 runtime protocol is not the canonical frozen mapping")
    protocol_path = Path(str(protocol.get("_protocol_path", ""))).resolve()
    if not (
        protocol_path == DEFAULT_PROTOCOL.resolve()
        and protocol.get("_protocol_sha256") == _sha256(DEFAULT_PROTOCOL)
        and protocol.get("protocol_id") == PROTOCOL_ID
    ):
        raise ProtocolError("V14.1 runtime protocol identity drifted")


def run_cross_speed_protocol(
    protocol: Mapping[str, Any], output_dir: Path
) -> dict[str, Any]:
    _validate_runtime_protocol(protocol)
    _preflight_no_clobber(output_dir)
    with _configured_parent(protocol, output_dir):
        v14.run_cross_speed_protocol(protocol, output_dir)
    persisted = _load_json(output_dir / "manifest.json")
    if not (
        persisted.get("protocol", {}).get("protocol_id") == PROTOCOL_ID
        and persisted.get("recovery", {}).get("recovery_id") == RECOVERY_ID
        and isinstance(persisted.get("ok"), bool)
    ):
        raise ProtocolError("persisted V14.1 manifest identity drifted")
    return persisted


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Print or execute the frozen AB06 V14.1 recovery protocol."
    )
    parser.add_argument("--protocol", default=str(DEFAULT_PROTOCOL))
    parser.add_argument("--output-dir", default=str(DEFAULT_OUTPUT_DIR))
    parser.add_argument("--print-protocol-template", action="store_true")
    parser.add_argument("--execute", action="store_true")
    return parser


def _write_failure(output_dir: Path, exc: Exception) -> None:
    if not output_dir.is_dir() or (output_dir / "failure.json").exists():
        return
    _PARENT_WRITE_JSON(
        output_dir / "failure.json",
        {
            "schema_version": SCHEMA_VERSION,
            "status": "ERROR_AFTER_V14_1_RECOVERY_DESTINATION_CONSUMED",
            "ok": False,
            "rerun_allowed": False,
            "validation_receipt_exists": (
                output_dir / "validation_access_receipt.json"
            ).is_file(),
            "sealed_receipt_exists": (
                output_dir / "sealed_access_receipt.json"
            ).is_file(),
            "recovery_lineage": _source_record(RECOVERY_LINEAGE),
            "error": f"{type(exc).__name__}: {exc}",
            "traceback": traceback.format_exc(),
        },
    )


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    if args.print_protocol_template:
        if args.execute:
            parser.error("--print-protocol-template and --execute are mutually exclusive")
        print(
            json.dumps(
                expected_protocol_payload(require_all_sources=False),
                indent=2,
                sort_keys=True,
                allow_nan=False,
            )
        )
        return 0
    if not args.execute:
        parser.error("semantic execution requires the explicit --execute flag")
    output_dir = v14.v1.resolve_repo_path(args.output_dir).resolve()
    output_existed_before_attempt = output_dir.exists()
    ledger_existed_before_attempt = DEFAULT_EXECUTION_LEDGER.exists()
    try:
        _preflight_no_clobber(output_dir)
        protocol = load_and_validate_protocol(args.protocol)
        manifest = run_cross_speed_protocol(protocol, output_dir)
    except NoClobberError as exc:
        if (
            output_dir.resolve() == DEFAULT_OUTPUT_DIR.resolve()
            and not output_existed_before_attempt
            and not ledger_existed_before_attempt
            and output_dir.is_dir()
            and (output_dir / "run_start_receipt.json").is_file()
        ):
            _write_failure(output_dir, exc)
        print(
            json.dumps(
                {
                    "schema_version": SCHEMA_VERSION,
                    "status": "ERROR",
                    "ok": False,
                    "no_clobber": True,
                    "error": f"{type(exc).__name__}: {exc}",
                },
                indent=2,
            )
        )
        return 2
    except Exception as exc:  # pragma: no cover - integration failure path.
        if (
            output_dir.resolve() == DEFAULT_OUTPUT_DIR.resolve()
            and output_dir.is_dir()
            and (output_dir / "run_start_receipt.json").is_file()
        ):
            _write_failure(output_dir, exc)
        print(
            json.dumps(
                {
                    "schema_version": SCHEMA_VERSION,
                    "status": "ERROR",
                    "ok": False,
                    "error": f"{type(exc).__name__}: {exc}",
                },
                indent=2,
            )
        )
        return 1
    print(json.dumps(manifest, indent=2, sort_keys=True, allow_nan=False))
    return 0 if manifest["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
