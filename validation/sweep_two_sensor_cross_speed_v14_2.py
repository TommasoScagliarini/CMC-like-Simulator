"""Fail-closed V14.2 boundary recovery for the AB06 cross-speed sweep.

V14.1 repaired preprocessing, then stopped on the already-open DEV02 before
station sampling because the first prescribed heel strike of plateau 1 had
only 45.851 ms of local context.  V14.2 keeps the 90 ms causal guard and the
V14.1 warm-up certificate.  Before any detector replay it removes the minimum
candidate-independent prescribed-cycle prefix that satisfies both contracts.

The consumed V14 and V14.1 destinations are immutable.  Trial 02 reuses a
byte-locked copy of its V14.1 preprocessing products without raw MAT decode or
IK.  Trials 04/08, and conditionally 05/06, retain the V14.1 receipt-first
preprocessing path.  Trials 03/07 have no execution route.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import shutil
import sys
import traceback
from contextlib import contextmanager
from dataclasses import replace
from pathlib import Path
from typing import Any, Iterator, Mapping, Sequence

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
VALIDATION_ROOT = REPO_ROOT / "validation"
for _path in (REPO_ROOT, VALIDATION_ROOT):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

import sweep_two_sensor_cross_speed_v14_1 as v141  # noqa: E402


v14 = v141.v14
SCHEMA_VERSION = 14
PROTOCOL_ID = (
    "AB06_TWO_SENSOR_CROSS_SPEED_INTERPOLATION_2026-07-22_V14_2_BOUNDARY_RECOVERY"
)
RECOVERY_ID = (
    "AB06_TWO_SENSOR_CROSS_SPEED_V141_TO_V142_BOUNDARY_RECOVERY_2026-07-22"
)
DEFAULT_PROTOCOL = VALIDATION_ROOT / "two_sensor_cross_speed_v14_2_protocol.json"
DEFAULT_OUTPUT_DIR = (
    VALIDATION_ROOT
    / "two_sensor_cross_speed_v14_2_runs/"
    "2026-07-22_ab06_cross_speed_v14_2_boundary_recovery"
)
DEFAULT_EXECUTION_LEDGER = (
    VALIDATION_ROOT / "two_sensor_cross_speed_v14_2_execution_ledger.json"
)
BOUNDARY_LINEAGE = (
    VALIDATION_ROOT / "two_sensor_cross_speed_v142_boundary_recovery_lineage.json"
)
PARENT_PROTOCOL = VALIDATION_ROOT / "two_sensor_cross_speed_v14_1_protocol.json"
PARENT_LEDGER = VALIDATION_ROOT / "two_sensor_cross_speed_v14_1_execution_ledger.json"
PARENT_OUTPUT_DIR = (
    VALIDATION_ROOT
    / "two_sensor_cross_speed_v14_1_runs/"
    "2026-07-22_ab06_cross_speed_v14_1_recovery"
)
PARENT_PREPROCESSED_TRIAL02 = PARENT_OUTPUT_DIR / "preprocessed/trial_02"

EXPECTED_BOUNDARY_LINEAGE_SHA256 = (
    "6902f821f1226c45aee9dc6914bb105f8cc424449151f06febfdd3b8988816d8"
)
EXPECTED_PARENT_HASHES = {
    "runner": "959f36239cda5bbd89dc07be68f2ecf11c3b1686def1b8c006d787c04bde79f8",
    "tests": "5eeb101590a50f3da57c57c107086d3ca56a369a4ff470a1e24a6ed4a1cf0215",
    "warmup": "886d2f89171ba8fedb03befdc752b90eb3a66e085685d3bc040d76a7fbb8c9b6",
    "warmup_tests": "2f1312416dde548adffd423c605fd5ee1fb626b0c876168532c0c3ec5ec052e3",
    "protocol": "e263a624e24a13bafd76bf2858f831184914399d188982deb2520f1f383673f1",
    "ledger": "2db41fde50d009b57a04b9f51b037c0d4c1363ee202aca0f1592c5b85b87847f",
    "lineage": "60590bbcc69f8095add54e66042c1751216dfeb6d443f2bddc355e392818d139",
    "run_start_receipt": "b3b6db3d87349e4d182b0494ca57409567b80fd0641d07d0ef07f80d2b135ba8",
    "failure": "4009aa12a01a1a0943a2525ea29658d6444d35ce09513db9c80625acb085f56d",
    "trial02_access_receipt": "adcab9090ef724621e798a8bf4a5c6988011f9894dc057a101b8b791b126b4e2",
    "trial02_preprocessing_lock": "fa45f3e98b2dc83313c9e8171ed84114edc87078750379c05843a525e3a963d8",
}
PARENT_SOURCE_PATHS = {
    "runner": VALIDATION_ROOT / "sweep_two_sensor_cross_speed_v14_1.py",
    "tests": VALIDATION_ROOT / "test_two_sensor_cross_speed_v14_1.py",
    "warmup": VALIDATION_ROOT / "two_sensor_v14_1_warmup.py",
    "warmup_tests": VALIDATION_ROOT / "test_two_sensor_v14_1_warmup.py",
    "protocol": PARENT_PROTOCOL,
    "ledger": PARENT_LEDGER,
    "lineage": VALIDATION_ROOT / "two_sensor_cross_speed_v141_recovery_lineage.json",
    "run_start_receipt": PARENT_OUTPUT_DIR / "run_start_receipt.json",
    "failure": PARENT_OUTPUT_DIR / "failure.json",
    "trial02_access_receipt": (
        PARENT_OUTPUT_DIR / "development_trial_02_preprocessing_access_receipt.json"
    ),
    "trial02_preprocessing_lock": (
        PARENT_PREPROCESSED_TRIAL02 / "treadmill_02_01_preprocessing_lock.json"
    ),
}
EXPECTED_PARENT_OUTPUT_FILES = {
    "development_trial_02_preprocessing_access_receipt.json": (
        "adcab9090ef724621e798a8bf4a5c6988011f9894dc057a101b8b791b126b4e2"
    ),
    "failure.json": "4009aa12a01a1a0943a2525ea29658d6444d35ce09513db9c80625acb085f56d",
    "preprocessed/trial_02/opensim.log": (
        "e6d5f8205fc91326674cb34cef6b1182b67c9f91046bddcabd35d3da39de7329"
    ),
    "preprocessed/trial_02/treadmill_02_01.trc": (
        "cef30550f636c3d9ef4fa759c54c7cb98f0a9253d43cca777b196af5e7ca27b4"
    ),
    "preprocessed/trial_02/treadmill_02_01_ExternalLoads.xml": (
        "060c8add7ba26cf1ddf956fbc7445950aab0ee5c889e57024bb2d394a70bc429"
    ),
    "preprocessed/trial_02/treadmill_02_01_conversion_manifest.json": (
        "41580d678c6243e3a49a25a2842a9b6d5b5d32e6e65446b09fae29be6b5ba440"
    ),
    "preprocessed/trial_02/treadmill_02_01_grf.mot": (
        "36de805e73e1c95d303ba6265ee262099e0a88fc46e7351bae0cdce3877d23d3"
    ),
    "preprocessed/trial_02/treadmill_02_01_ik.mot": (
        "4018b3001a5d293edda799839158f4c154747af5c908a2eb530dae3e37e5a982"
    ),
    "preprocessed/trial_02/treadmill_02_01_ik_dataset_ab06_seasea.mot": (
        "314101f0dcf010a663699ed7e5d8765bcdca673c8f8d18337cd1a6238615a9f0"
    ),
    "preprocessed/trial_02/treadmill_02_01_ik_execution_receipt.json": (
        "36e578bb9556b692efa789e945b0641fe6b0b4d969d2f7400c98edf34c0ba491"
    ),
    "preprocessed/trial_02/treadmill_02_01_ik_receipt.json": (
        "f5daa2290a030dd2dbf6c4b1908db2e9505b14745b397aa61b070993703e086e"
    ),
    "preprocessed/trial_02/treadmill_02_01_iksetup.xml": (
        "2872322d1f61317c4bcf2c07174f99d612df09771c54f42a87c73a525c80d811"
    ),
    "preprocessed/trial_02/treadmill_02_01_preprocessing_lock.json": (
        "fa45f3e98b2dc83313c9e8171ed84114edc87078750379c05843a525e3a963d8"
    ),
    "run_start_receipt.json": (
        "b3b6db3d87349e4d182b0494ca57409567b80fd0641d07d0ef07f80d2b135ba8"
    ),
}
TRIAL02_REUSABLE_FILES = {
    relative: digest
    for relative, digest in EXPECTED_PARENT_OUTPUT_FILES.items()
    if relative.startswith("preprocessed/trial_02/treadmill_02_01")
    and not relative.endswith("preprocessing_lock.json")
}

FROZEN_BOUNDARY_CONTRACT = {
    "schema_version": 1,
    "selection_source": "prescribed_grf_only_before_station_sampling",
    "candidate_independent": True,
    "cadence_independent": True,
    "minimum_prefix_rule": (
        "first_retained_hs_minus_plateau_start_at_least_90ms"
    ),
    "first_plateau_additional_rule": (
        "at_least_one_complete_prescribed_trial_cycle_closed_by_"
        "first_retained_hs_minus_90ms"
    ),
    "right_boundary_censor_applied_after_left_prefix": True,
    "minimum_scoreable_cycles_after_both_censors": 10,
    "left_excluded_cycles_enter_scoring": False,
    "warmup_cycles_enter_scoring": False,
    "guard_relaxed": False,
}


class ProtocolError(v141.ProtocolError):
    """Raised when the V14.2 frozen recovery contract is violated."""


NoClobberError = v141.NoClobberError
_RAW_WRITE_JSON = v14._write_json_exclusive
_PARENT_RETAIN_SCOREABLE = v14.retain_scoreable_plateau_cycles


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
            raise ProtocolError(f"missing V14.2 source: {_portable(resolved)}")
        return {"path": _portable(resolved), "sha256": "<FILL_AFTER_SOURCE_EXISTS>"}
    return {"path": _portable(resolved), "sha256": _sha256(resolved)}


def _load_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ProtocolError(f"cannot load V14.2 evidence: {_portable(path)}") from exc
    if not isinstance(value, dict):
        raise ProtocolError(f"V14.2 evidence is not an object: {_portable(path)}")
    return value


def _validate_parent_v141_failure() -> dict[str, Any]:
    checks = {
        name: path.is_file() and _sha256(path) == EXPECTED_PARENT_HASHES[name]
        for name, path in PARENT_SOURCE_PATHS.items()
    }
    observed_inventory = {
        path.relative_to(PARENT_OUTPUT_DIR).as_posix(): _sha256(path)
        for path in PARENT_OUTPUT_DIR.rglob("*")
        if path.is_file()
    } if PARENT_OUTPUT_DIR.is_dir() else {}
    checks["exact_parent_output_inventory_and_hashes"] = (
        observed_inventory == EXPECTED_PARENT_OUTPUT_FILES
    )
    failure = _load_json(PARENT_OUTPUT_DIR / "failure.json")
    checks.update(
        {
            "exact_failure": failure.get("error")
            == (
                "ProtocolError: first scoreable HS lacks the frozen 90 ms "
                "continuous-state guard"
            ),
            "rerun_forbidden": failure.get("rerun_allowed") is False,
            "validation_unopened": failure.get("validation_receipt_exists") is False,
            "sealed_unopened": failure.get("sealed_receipt_exists") is False,
            "no_detector_or_candidate_outputs": not any(
                any(token in relative for token in ("metrics", "details", "candidate_lock"))
                for relative in observed_inventory
            ),
            "only_trial02_preprocessed": not any(
                f"trial_{trial_id}" in relative
                for trial_id in ("03", "04", "05", "06", "07", "08")
                for relative in observed_inventory
            ),
        }
    )
    if not all(checks.values()):
        raise ProtocolError(f"parent V14.1 failure evidence drifted: {checks}")
    return {
        "status": "PASS_PARENT_V14_1_PRE_REPLAY_FAILURE_VERIFIED",
        "checks": checks,
        "sources": {
            name: _source_record(path) for name, path in PARENT_SOURCE_PATHS.items()
        },
        "output_inventory_sha256": observed_inventory,
    }


def _validated_boundary_lineage() -> dict[str, str]:
    if (
        not BOUNDARY_LINEAGE.is_file()
        or _sha256(BOUNDARY_LINEAGE) != EXPECTED_BOUNDARY_LINEAGE_SHA256
    ):
        raise ProtocolError("V14.2 boundary lineage identity drifted")
    payload = _load_json(BOUNDARY_LINEAGE)
    contract = payload.get("frozen_boundary_recovery_contract", {})
    diagnosis = payload.get("observed_dev02_boundary_diagnosis", {})
    minimum = diagnosis.get("minimum_valid_prefix_for_plateau_1", {})
    if not (
        payload.get("status") == "BOUNDARY_RECOVERY_PREREGISTRATION_EVIDENCE"
        and payload.get("recovery_id") == RECOVERY_ID
        and contract.get("applies_candidate_independently") is True
        and contract.get("remove_or_relax_90ms_guard") is False
        and minimum.get("excluded_complete_cycle_count") == 2
        and minimum.get("retained_scoreable_cycle_count_before_right_censor") == 19
    ):
        raise ProtocolError("V14.2 boundary lineage semantics drifted")
    return _source_record(BOUNDARY_LINEAGE)


def _normalized_inherited_scientific_core(
    protocol: Mapping[str, Any],
) -> dict[str, Any]:
    core = v141._scientific_core(protocol)
    core["unit_boundary_handling"].pop(
        "v14_2_prescribed_left_prefix_censor", None
    )
    return core


def expected_protocol_payload(*, require_all_sources: bool = True) -> dict[str, Any]:
    """Build V14.2 from the frozen V14.1 JSON without opening new trials."""

    parent_evidence = _validate_parent_v141_failure()
    lineage = _validated_boundary_lineage()
    parent = _load_json(PARENT_PROTOCOL)
    if _sha256(PARENT_PROTOCOL) != EXPECTED_PARENT_HASHES["protocol"]:
        raise ProtocolError("parent V14.1 protocol drifted")
    payload = copy.deepcopy(parent)
    payload["protocol_id"] = PROTOCOL_ID
    payload["stage"] = (
        "v14_2_prescribed_boundary_recovery_then_cross_speed_development_"
        "then_one_shot_validation_and_sealed"
    )
    payload["objective"] = (
        "Apply the preregistered prescribed-only left-boundary prefix censor, "
        "then execute the inherited V14.1 center/radius search and staged gates."
    )
    payload["interpretation_limits"] = [
        *parent["interpretation_limits"],
        "V14.2 changes the scoreable reference-cycle boundary set and is not claimed as exact-core V14/V14.1.",
        "The left prefix is chosen from prescribed GRF only before station sampling and cannot depend on a detector or candidate outcome.",
        "The inherited 90 ms causal guard and warm-up certificate are preserved, not relaxed.",
        "V14.1 DEV02 preprocessing is copied byte-identically; its raw MAT streams and IK are not reopened.",
    ]
    payload["unit_boundary_handling"][
        "v14_2_prescribed_left_prefix_censor"
    ] = copy.deepcopy(FROZEN_BOUNDARY_CONTRACT)
    payload["preprocessing"].update(
        {
            "trial_02_source": "byte_locked_v14_1_preprocessing_products",
            "trial_02_raw_mat_redecode": False,
            "trial_02_inverse_kinematics_rerun": False,
            "trial_02_v14_2_import_receipt_before_copy": True,
        }
    )
    payload["execution_destination"] = {
        "canonical_output_dir": _portable(DEFAULT_OUTPUT_DIR),
        "canonical_one_shot_ledger": _portable(DEFAULT_EXECUTION_LEDGER),
        "alternate_output_directory_allowed": False,
        "ledger_written_before_any_development_import_or_semantic_decode": True,
    }
    payload["recovery"]["v14_2_boundary_recovery"] = {
        "recovery_id": RECOVERY_ID,
        "lineage": lineage,
        "parent_failure": parent_evidence,
        "parent_destination_reused": False,
        "parent_ledger_reused": False,
        "trial_02_preprocessing_reused_byte_identically": True,
        "boundary_contract": copy.deepcopy(FROZEN_BOUNDARY_CONTRACT),
        "boundary_contract_sha256": _canonical_sha256(FROZEN_BOUNDARY_CONTRACT),
        "only_scientific_delta": "prescribed_left_boundary_prefix_censor",
    }
    new_sources = {
        "v14_2_harness": VALIDATION_ROOT / "sweep_two_sensor_cross_speed_v14_2.py",
        "v14_2_tests": VALIDATION_ROOT / "test_two_sensor_cross_speed_v14_2.py",
        "v14_2_boundary_lineage": BOUNDARY_LINEAGE,
        "parent_v14_1_protocol": PARENT_PROTOCOL,
        "parent_v14_1_execution_ledger": PARENT_LEDGER,
        "parent_v14_1_run_start_receipt": PARENT_OUTPUT_DIR / "run_start_receipt.json",
        "parent_v14_1_failure": PARENT_OUTPUT_DIR / "failure.json",
        "parent_v14_1_trial02_preprocessing_lock": (
            PARENT_PREPROCESSED_TRIAL02 / "treadmill_02_01_preprocessing_lock.json"
        ),
    }
    payload["sources"].update(
        {
            name: _source_record(path, require=require_all_sources)
            for name, path in new_sources.items()
        }
    )
    if (
        _normalized_inherited_scientific_core(payload)
        != _normalized_inherited_scientific_core(parent)
    ):
        raise ProtocolError("V14.2 changed inherited scientific fields beyond boundary")
    return payload


def load_and_validate_protocol(path: str | Path = DEFAULT_PROTOCOL) -> dict[str, Any]:
    protocol_path = v14.v1.resolve_repo_path(path).resolve()
    if protocol_path != DEFAULT_PROTOCOL.resolve():
        raise ProtocolError("V14.2 execution requires the canonical protocol path")
    raw = _load_json(protocol_path)
    expected = expected_protocol_payload(require_all_sources=True)
    if raw != expected:
        differing = sorted(
            key for key in set(raw) | set(expected) if raw.get(key) != expected.get(key)
        )
        raise ProtocolError(f"V14.2 frozen protocol drifted: {differing}")
    raw["_protocol_path"] = protocol_path.as_posix()
    raw["_protocol_sha256"] = _sha256(protocol_path)
    raw["_primary_event_time_field"] = "confirmed_time_s"
    raw["_diagnostic_event_time_field"] = "event_time_s"
    raw["_phase_reference_mode"] = "validated_event_intervals"
    raw["profile_paths"] = {v14.v1.CURRENT_PROFILE_ID: v14.BASELINE_PROFILE_PATH}
    return raw


def _validated_event_arrays(
    events: Mapping[str, Sequence[float] | np.ndarray],
    *,
    label: str,
) -> tuple[np.ndarray, np.ndarray]:
    try:
        heel = np.asarray(events["heel_strike"], dtype=float)
        toe = np.asarray(events["toe_off"], dtype=float)
    except (KeyError, TypeError, ValueError) as exc:
        raise ProtocolError(f"{label} event arrays are malformed") from exc
    if (
        heel.ndim != 1
        or toe.ndim != 1
        or heel.size != toe.size + 1
        or heel.size < 2
        or not np.all(np.isfinite(heel))
        or not np.all(np.isfinite(toe))
        or np.any(np.diff(heel) <= 0.0)
        or np.any(np.diff(toe) <= 0.0)
    ):
        raise ProtocolError(f"{label} must be finite HS--TO--HS cycles")
    for index, toe_time in enumerate(toe):
        if not float(heel[index]) < float(toe_time) < float(heel[index + 1]):
            raise ProtocolError(f"{label} is not strictly HS--TO--HS")
    return heel, toe


def _eligible_prescribed_warmup_cycles(
    events: Mapping[str, Sequence[float] | np.ndarray],
    *,
    cutoff_s: float,
) -> list[dict[str, float]]:
    heel, toe = _validated_event_arrays(events, label="trial warmup reference")
    return [
        {
            "opening_hs_s": float(heel[index]),
            "toe_off_s": float(toe_time),
            "closing_hs_s": float(heel[index + 1]),
        }
        for index, toe_time in enumerate(toe)
        if float(heel[index + 1]) <= cutoff_s + v14.NUMERIC_TOLERANCE
    ]


def retain_scoreable_plateau_cycles_v142(
    raw_events: Mapping[str, Sequence[float] | np.ndarray],
    *,
    plateau_start_s: float,
    plateau_end_s: float,
    warmup_prescribed_events: Mapping[str, Sequence[float] | np.ndarray] | None,
    right_observation_margin_s: float = v14.RIGHT_OBSERVATION_MARGIN_S,
    minimum_cycles: int = v14.MINIMUM_SCOREABLE_CYCLES_PER_PLATEAU,
) -> tuple[dict[str, np.ndarray], dict[str, Any]]:
    """Apply the frozen minimum prescribed left prefix, then V14 right censor."""

    start = float(plateau_start_s)
    end = float(plateau_end_s)
    heel, toe = _validated_event_arrays(raw_events, label="plateau reference")
    for index, toe_time in enumerate(toe):
        if not (
            start <= float(heel[index])
            < float(toe_time)
            < float(heel[index + 1])
            <= end
        ):
            raise ProtocolError("raw reference cycle is not fully internal to plateau")

    chosen_prefix: int | None = None
    chosen_warmup: list[dict[str, float]] = []
    warmup_heel = (
        None
        if warmup_prescribed_events is None
        else _validated_event_arrays(
            warmup_prescribed_events, label="trial warmup reference"
        )[0]
    )
    for prefix in range(int(toe.size)):
        anchor = float(heel[prefix])
        if anchor - start + v14.NUMERIC_TOLERANCE < v14.LEFT_CONTEXT_S:
            continue
        if warmup_heel is not None and not np.any(
            np.isclose(
                warmup_heel,
                anchor,
                rtol=0.0,
                atol=v14.NUMERIC_TOLERANCE,
            )
        ):
            continue
        eligible = (
            []
            if warmup_prescribed_events is None
            else _eligible_prescribed_warmup_cycles(
                warmup_prescribed_events,
                cutoff_s=anchor - v14.LEFT_CONTEXT_S,
            )
        )
        if warmup_prescribed_events is not None and not eligible:
            continue
        chosen_prefix = prefix
        chosen_warmup = eligible
        break
    if chosen_prefix is None:
        raise ProtocolError(
            "no prescribed left prefix satisfies the V14.2 guard/warmup contract"
        )

    sliced = {
        "heel_strike": heel[chosen_prefix:].copy(),
        "toe_off": toe[chosen_prefix:].copy(),
    }
    events, access = _PARENT_RETAIN_SCOREABLE(
        sliced,
        plateau_start_s=start,
        plateau_end_s=end,
        right_observation_margin_s=right_observation_margin_s,
        minimum_cycles=minimum_cycles,
    )
    retained = int(access["retained_scoreable_cycle_count"])
    excluded_right = int(access["excluded_right_boundary_cycle_count"])
    if int(toe.size) != chosen_prefix + retained + excluded_right:
        raise ProtocolError("V14.2 left/retained/right cycle accounting drifted")
    excluded_cycles = [
        {
            "opening_hs_s": float(heel[index]),
            "toe_off_s": float(toe[index]),
            "closing_hs_s": float(heel[index + 1]),
        }
        for index in range(chosen_prefix)
    ]
    access.update(
        {
            "raw_complete_cycle_count": int(toe.size),
            "post_left_prefix_complete_cycle_count": int(toe.size - chosen_prefix),
            "excluded_left_boundary_cycle_count": chosen_prefix,
            "excluded_left_boundary_cycles": excluded_cycles,
            "left_boundary_prefix_is_minimum": True,
            "left_boundary_selection_source": "prescribed_grf_only",
            "first_plateau_warmup_cycle_required": (
                warmup_prescribed_events is not None
            ),
            "warmup_anchor_membership_verified": (
                None if warmup_prescribed_events is None else True
            ),
            "eligible_prescribed_warmup_cycle_count_before_cutoff": len(
                chosen_warmup
            ),
            "selected_prescribed_warmup_cycle": (
                None if not chosen_warmup else chosen_warmup[-1]
            ),
            "warmup_cutoff_s": (
                None
                if warmup_prescribed_events is None
                else float(events["heel_strike"][0] - v14.LEFT_CONTEXT_S)
            ),
            "left_retained_right_cycle_identity_ok": True,
            "v14_2_boundary_contract_sha256": _canonical_sha256(
                FROZEN_BOUNDARY_CONTRACT
            ),
        }
    )
    return events, access


def _plateau_references_v142(
    protocol: Mapping[str, Any],
    setup: Any,
    *,
    trial_id: str,
) -> tuple[dict[str, Any], ...]:
    """Derive frozen prescribed references before any detector station sample."""

    replay = protocol["replay"]
    plateaus = protocol["trials"][trial_id]["plateaus"]
    analysis_start = float(protocol["trials"][trial_id]["trial_interval_s"][0])
    first_plateau_end = float(plateaus[0]["end_s"])
    warmup_events, warmup_provenance = v14.v1._reference_events_from_prescribed_grf(
        replace(setup, t_start=analysis_start, t_end=first_plateau_end),
        threshold_n=float(replay["prescribed_contact_threshold_n"]),
        min_contact_duration_s=float(replay["reference_min_contact_duration_s"]),
        min_cycle_duration_s=float(replay["reference_min_cycle_duration_s"]),
    )
    result: list[dict[str, Any]] = []
    for plateau_index, plateau in enumerate(plateaus, start=1):
        start = float(plateau["start_s"])
        end = float(plateau["end_s"])
        raw_events, provenance = v14.v1._reference_events_from_prescribed_grf(
            replace(setup, t_start=start, t_end=end),
            threshold_n=float(replay["prescribed_contact_threshold_n"]),
            min_contact_duration_s=float(replay["reference_min_contact_duration_s"]),
            min_cycle_duration_s=float(replay["reference_min_cycle_duration_s"]),
        )
        events, cycle_access = retain_scoreable_plateau_cycles_v142(
            raw_events,
            plateau_start_s=start,
            plateau_end_s=end,
            warmup_prescribed_events=(
                warmup_events if plateau_index == 1 else None
            ),
        )
        result.append(
            {
                "trial_id": trial_id,
                "plateau_index": plateau_index,
                "speed_mps": float(plateau["speed_mps"]),
                "events": events,
                "dynamic_gate": v14._dynamic_gate(
                    len(events["heel_strike"]), len(events["toe_off"])
                ),
                "reference_provenance": provenance,
                "first_plateau_warmup_reference_provenance": (
                    warmup_provenance if plateau_index == 1 else None
                ),
                **cycle_access,
            }
        )
    return tuple(result)


def _validate_trial02_parent_products() -> dict[str, Path]:
    _validate_parent_v141_failure()
    paths: dict[str, Path] = {}
    for relative, expected in TRIAL02_REUSABLE_FILES.items():
        source = PARENT_OUTPUT_DIR / relative
        if not source.is_file() or _sha256(source) != expected:
            raise ProtocolError(f"V14.1 trial02 reusable product drifted: {relative}")
        paths[Path(relative).name] = source
    if len(paths) != 9:
        raise ProtocolError("V14.2 trial02 reusable product cardinality drifted")
    return paths


def _copy_exclusive(source: Path, destination: Path) -> None:
    if destination.exists():
        raise NoClobberError(f"refusing to overwrite {_portable(destination)}")
    with source.open("rb") as src, destination.open("xb") as dst:
        shutil.copyfileobj(src, dst, length=1024 * 1024)
    if _sha256(destination) != _sha256(source):
        raise ProtocolError(f"byte-copy verification failed: {_portable(destination)}")


def _prepare_trial02_from_parent(
    protocol: Mapping[str, Any],
    *,
    trial_id: str,
    stage: str,
    work_dir: Path,
    access_receipt: Path | None,
) -> Any:
    state = v141._active()
    if trial_id != "02" or stage != "development" or access_receipt is not None:
        raise ProtocolError("V14.2 parent-product import is restricted to DEV02")
    expected_work_dir = (
        state.output_dir / "preprocessed" / "trial_02"
    ).resolve()
    if work_dir.resolve() != expected_work_dir or work_dir.exists():
        raise NoClobberError("V14.2 DEV02 import path is occupied or noncanonical")
    v14._assert_semantic_access(
        protocol,
        trial_id=trial_id,
        stage=stage,
        access_receipt=access_receipt,
    )
    if not DEFAULT_EXECUTION_LEDGER.is_file():
        raise ProtocolError("V14.2 ledger must precede DEV02 preprocessing import")
    import_receipt = _RAW_WRITE_JSON(
        state.output_dir / "development_trial_02_preprocessing_import_receipt.json",
        {
            "schema_version": SCHEMA_VERSION,
            "status": "V14_2_DEV02_LOCKED_PREPROCESSING_IMPORT_STARTED",
            "trial_id": "02",
            "stage": "development",
            "protocol": {
                "path": _portable(Path(str(protocol["_protocol_path"]))),
                "sha256": protocol["_protocol_sha256"],
                "protocol_id": protocol["protocol_id"],
            },
            "execution_ledger": _source_record(DEFAULT_EXECUTION_LEDGER),
            "boundary_lineage": _source_record(BOUNDARY_LINEAGE),
            "parent_preprocessing_lock": _source_record(
                PARENT_PREPROCESSED_TRIAL02
                / "treadmill_02_01_preprocessing_lock.json"
            ),
            "written_before_parent_product_copy_or_semantic_replay": True,
            "prior_hash_only_parent_identity_checks_allowed": True,
            "raw_mat_decode_allowed": False,
            "inverse_kinematics_rerun_allowed": False,
            "rerun_allowed": False,
        },
    )
    state.stage_access_receipts[trial_id] = import_receipt
    sources = _validate_trial02_parent_products()
    work_dir.mkdir(parents=True, exist_ok=False)
    copied: dict[str, Path] = {}
    for name, source in sorted(sources.items()):
        destination = work_dir / name
        _copy_exclusive(source, destination)
        copied[name] = destination

    stem = "treadmill_02_01"
    trial = protocol["trials"][trial_id]
    required_range = [float(value) for value in trial["trial_interval_s"]]
    conversion_manifest = copied[f"{stem}_conversion_manifest.json"]
    manifest = v141._validate_conversion_manifest(
        conversion_manifest,
        required_range,
        expected_trial=stem,
        raw_sources=trial["raw_sources"],
    )
    trc = copied[f"{stem}.trc"]
    grf = copied[f"{stem}_grf.mot"]
    external = copied[f"{stem}_ExternalLoads.xml"]
    ik_setup = copied[f"{stem}_iksetup.xml"]
    ik_motion = copied[f"{stem}_ik.mot"]
    ik_receipt = copied[f"{stem}_ik_receipt.json"]
    v141._validate_final_ik_receipt(
        ik_receipt,
        work_dir=work_dir,
        stem=stem,
        conversion_manifest=conversion_manifest,
        ik_setup=ik_setup,
        ik_motion=ik_motion,
        required_range=required_range,
    )
    preprocessed_files = v141._validate_preprocessed_files(
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
        model_file=v14.v1.resolve_repo_path(protocol["model_file"]).resolve(),
        kinematics_file=ik_motion,
        external_loads_xml=external,
        reserve_actuators_xml=v14.v1.resolve_repo_path(
            protocol["reserve_actuators_xml"]
        ).resolve(),
        t_start=required_range[0],
        t_end=required_range[1],
        grf_mode="prescribed",
    )
    replay_inputs = v141._validated_live_replay_inputs(protocol, setup)
    lock = _RAW_WRITE_JSON(
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
            "access_receipt": _source_record(import_receipt),
            "conversion_manifest": _source_record(conversion_manifest),
            "ik_receipt": _source_record(ik_receipt),
            "ik_execution_receipt": _source_record(
                copied[f"{stem}_ik_execution_receipt.json"]
            ),
            "ik_motion": _source_record(ik_motion),
            "preprocessed_files": preprocessed_files,
            "live_replay_inputs": replay_inputs,
            "recovery_lineage": _source_record(BOUNDARY_LINEAGE),
            "v14_2_parent_product_import": {
                "status": "BYTE_IDENTICAL_PARENT_PRODUCTS_VERIFIED",
                "parent_preprocessing_lock": _source_record(
                    PARENT_PREPROCESSED_TRIAL02
                    / f"{stem}_preprocessing_lock.json"
                ),
                "source_sha256_by_filename": {
                    name: _sha256(source) for name, source in sorted(sources.items())
                },
                "destination_sha256_by_filename": {
                    name: _sha256(path) for name, path in sorted(copied.items())
                },
                "raw_mat_decode_used": False,
                "inverse_kinematics_used": False,
            },
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


def _prepare_trial_hook(
    protocol: Mapping[str, Any],
    *,
    trial_id: str,
    stage: str,
    work_dir: Path,
    access_receipt: Path | None,
) -> Any:
    if trial_id == "02":
        return _prepare_trial02_from_parent(
            protocol,
            trial_id=trial_id,
            stage=stage,
            work_dir=work_dir,
            access_receipt=access_receipt,
        )
    return v141._prepare_trial_hook(
        protocol,
        trial_id=trial_id,
        stage=stage,
        work_dir=work_dir,
        access_receipt=access_receipt,
    )


def _augment_payload(path: Path, payload: Mapping[str, Any]) -> dict[str, Any]:
    result = v141._augment_payload(path, payload)
    name = path.name
    boundary = {
        "recovery_id": RECOVERY_ID,
        "lineage": _source_record(BOUNDARY_LINEAGE),
        "contract": copy.deepcopy(FROZEN_BOUNDARY_CONTRACT),
        "contract_sha256": _canonical_sha256(FROZEN_BOUNDARY_CONTRACT),
    }
    if name == DEFAULT_EXECUTION_LEDGER.name:
        result["status"] = "V14_2_BOUNDARY_RECOVERY_CANONICAL_DESTINATION_CONSUMED"
        result["boundary_recovery"] = boundary
    elif name == "run_start_receipt.json":
        result["status"] = "V14_2_BOUNDARY_RECOVERY_DEVELOPMENT_ACCESS_STARTED"
        result["boundary_recovery"] = boundary
    elif name == "development_candidate_lock.json":
        result["boundary_recovery"] = boundary
        result["trial_02_preprocessing_reused_without_raw_or_ik"] = True
    elif name == "validation_access_receipt.json":
        result["status"] = "VALIDATION_OPENED_FOR_SINGLE_AUTHORIZED_V14_2_RUN"
        result["boundary_recovery"] = boundary
    elif name == "sealed_access_receipt.json":
        result["status"] = "SEALED_OPENED_FOR_SINGLE_AUTHORIZED_V14_2_RUN"
        result["boundary_recovery"] = boundary
    elif name in {"validation_decision_lock.json", "manifest.json"}:
        result["boundary_recovery"] = boundary
    if name == "manifest.json":
        if result.get("conclusion") == "V14_1_RECOVERY_CHALLENGER_PASSED_ALL_STAGES":
            result["conclusion"] = "V14_2_BOUNDARY_RECOVERY_CHALLENGER_PASSED_ALL_STAGES"
        recovery = result.setdefault("recovery", {})
        recovery["recovery_id"] = RECOVERY_ID
        recovery["boundary_recovery"] = boundary
        recovery["trial_02_preprocessing_reused_without_raw_or_ik"] = True
    return result


def _write_json_hook(path: Path, payload: Mapping[str, Any]) -> Path:
    return _RAW_WRITE_JSON(path, _augment_payload(path, payload))


@contextmanager
def _configured_runtime(
    protocol: Mapping[str, Any], output_dir: Path
) -> Iterator[None]:
    if v141._RUNTIME is not None:
        raise ProtocolError("nested V14.2/V14.1 execution is forbidden")
    parent_patches = {
        "DEFAULT_PROTOCOL": DEFAULT_PROTOCOL,
        "DEFAULT_OUTPUT_DIR": DEFAULT_OUTPUT_DIR,
        "DEFAULT_EXECUTION_LEDGER": DEFAULT_EXECUTION_LEDGER,
        "prepare_trial": _prepare_trial_hook,
        "_evaluate_trial": v141._evaluate_trial_hook,
        "evaluate_continuous_candidate": v141._evaluate_candidate_hook,
        "root_safe_isolated": v141._root_safe_hook,
        "_write_json_exclusive": _write_json_hook,
        "_plateau_references": _plateau_references_v142,
    }
    wrapper_patches = {
        "DEFAULT_PROTOCOL": DEFAULT_PROTOCOL,
        "DEFAULT_OUTPUT_DIR": DEFAULT_OUTPUT_DIR,
        "DEFAULT_EXECUTION_LEDGER": DEFAULT_EXECUTION_LEDGER,
        "PROTOCOL_ID": PROTOCOL_ID,
        "RECOVERY_ID": RECOVERY_ID,
        "RECOVERY_LINEAGE": BOUNDARY_LINEAGE,
    }
    previous_parent = {name: getattr(v14, name) for name in parent_patches}
    previous_wrapper = {name: getattr(v141, name) for name in wrapper_patches}
    v141._RUNTIME = v141._RuntimeState(protocol, output_dir, {}, {}, {}, {}, {})
    try:
        for name, value in wrapper_patches.items():
            setattr(v141, name, value)
        for name, value in parent_patches.items():
            setattr(v14, name, value)
        yield
    finally:
        for name, value in previous_parent.items():
            setattr(v14, name, value)
        for name, value in previous_wrapper.items():
            setattr(v141, name, value)
        v141._RUNTIME = None


def _preflight_no_clobber(output_dir: Path) -> None:
    if output_dir.resolve() != DEFAULT_OUTPUT_DIR.resolve():
        raise ProtocolError("V14.2 execution is bound to its canonical output directory")
    if DEFAULT_EXECUTION_LEDGER.exists():
        raise NoClobberError("V14.2 canonical one-shot ledger already exists")
    if output_dir.exists():
        raise NoClobberError(f"refusing occupied V14.2 output: {_portable(output_dir)}")
    if output_dir in {v141.DEFAULT_OUTPUT_DIR, v141.PARENT_OUTPUT_DIR}:
        raise ProtocolError("V14.2 cannot reuse a consumed parent destination")
    _validate_parent_v141_failure()
    _validated_boundary_lineage()


def _validate_runtime_protocol(protocol: Mapping[str, Any]) -> None:
    if not isinstance(protocol, Mapping):
        raise ProtocolError("V14.2 runtime protocol must be a mapping")
    canonical = load_and_validate_protocol(DEFAULT_PROTOCOL)
    if dict(protocol) != canonical:
        raise ProtocolError("V14.2 runtime protocol is not the canonical frozen mapping")
    if not (
        Path(str(protocol.get("_protocol_path", ""))).resolve()
        == DEFAULT_PROTOCOL.resolve()
        and protocol.get("_protocol_sha256") == _sha256(DEFAULT_PROTOCOL)
        and protocol.get("protocol_id") == PROTOCOL_ID
    ):
        raise ProtocolError("V14.2 runtime protocol identity drifted")


def run_cross_speed_protocol(
    protocol: Mapping[str, Any], output_dir: Path
) -> dict[str, Any]:
    _validate_runtime_protocol(protocol)
    _preflight_no_clobber(output_dir)
    with _configured_runtime(protocol, output_dir):
        v14.run_cross_speed_protocol(protocol, output_dir)
    persisted = _load_json(output_dir / "manifest.json")
    boundary = persisted.get("boundary_recovery", {})
    if not (
        persisted.get("protocol", {}).get("protocol_id") == PROTOCOL_ID
        and boundary.get("recovery_id") == RECOVERY_ID
        and isinstance(persisted.get("ok"), bool)
    ):
        raise ProtocolError("persisted V14.2 manifest identity drifted")
    return persisted


def _write_failure(output_dir: Path, exc: Exception) -> None:
    if not output_dir.is_dir() or (output_dir / "failure.json").exists():
        return
    _RAW_WRITE_JSON(
        output_dir / "failure.json",
        {
            "schema_version": SCHEMA_VERSION,
            "status": "ERROR_AFTER_V14_2_BOUNDARY_RECOVERY_DESTINATION_CONSUMED",
            "ok": False,
            "rerun_allowed": False,
            "validation_receipt_exists": (
                output_dir / "validation_access_receipt.json"
            ).is_file(),
            "sealed_receipt_exists": (
                output_dir / "sealed_access_receipt.json"
            ).is_file(),
            "boundary_lineage": _source_record(BOUNDARY_LINEAGE),
            "error": f"{type(exc).__name__}: {exc}",
            "traceback": traceback.format_exc(),
        },
    )


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Print or execute the frozen AB06 V14.2 boundary recovery."
    )
    parser.add_argument("--protocol", default=str(DEFAULT_PROTOCOL))
    parser.add_argument("--output-dir", default=str(DEFAULT_OUTPUT_DIR))
    parser.add_argument("--print-protocol-template", action="store_true")
    parser.add_argument("--execute", action="store_true")
    return parser


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
    output_existed_before = output_dir.exists()
    ledger_existed_before = DEFAULT_EXECUTION_LEDGER.exists()
    try:
        _preflight_no_clobber(output_dir)
        protocol = load_and_validate_protocol(args.protocol)
        manifest = run_cross_speed_protocol(protocol, output_dir)
    except NoClobberError as exc:
        if (
            output_dir.resolve() == DEFAULT_OUTPUT_DIR.resolve()
            and not output_existed_before
            and not ledger_existed_before
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
