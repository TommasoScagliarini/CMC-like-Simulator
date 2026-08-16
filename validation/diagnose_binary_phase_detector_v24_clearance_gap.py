"""Diagnose the V23 trial-08 clearance gap without changing the detector.

This development-only diagnostic replays the hash-pinned V21 detector profile
on the marker/source and runtime models at identical trial-08 IK coordinates.
It samples continuous heel/toe signed clearance around the extra V23 TO/HS
pair, compares the resulting bits with the immutable V23 packed trace, and
quantifies the uniform normal-direction clearance margin required to close the
short false-air gap.  It does not tune geometry or FSM logic and cannot open
protected trials, execute H0, promote runtime/training, or activate corridor or
PPO work.

``--check`` is read-only.  ``--execute-development-diagnostic`` writes one
strict-JSON diagnostic followed by a separate terminal receipt, both with
exclusive no-clobber semantics.  Execution is intentionally explicit.
"""

from __future__ import annotations

import argparse
import base64
import hashlib
import json
import math
import os
import platform
import sys
import tempfile
import xml.etree.ElementTree as ET
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Mapping, Sequence, TextIO

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
VALIDATION_ROOT = REPO_ROOT / "validation"
TRAJECTORY_ROOT = REPO_ROOT / "Trajectory Generator"
for import_root in (REPO_ROOT, VALIDATION_ROOT, TRAJECTORY_ROOT):
    if str(import_root) not in sys.path:
        sys.path.insert(0, str(import_root))


SCHEMA_VERSION = 24
DIAGNOSTIC_ID = "AB06_BINARY_POINT_V24_TRIAL08_CLEARANCE_GAP_DIAGNOSTIC"
TRIAL_ID = "08"
CANDIDATE_ID = "v21_678b0b5162b706dd"
SAMPLE_DT_S = 0.001

# The highlighted interval contains the V23-only false TO/HS pair.  The wider
# fixed context includes the subsequent true swing and oracle TO at 100.168 s.
CORE_WINDOW_START_S = 99.840
CORE_WINDOW_END_S = 99.940
CONTEXT_START_S = 99.750
CONTEXT_END_S = 100.200
V23_FALSE_AIR_ONSET_S = 99.882
V23_FALSE_AIR_LAST_SAMPLE_S = 99.893
V23_RECONTACT_ONSET_S = 99.894
V23_FALSE_AIR_SAMPLE_COUNT = 12
V20_DEBOUNCE_INTERVAL_S = 0.005
V20_DEBOUNCE_INTERVAL_COUNT = 5
V20_CONFIRMATION_RUN_SAMPLES = 6
V23_TRUE_TO_S = 100.147
ORACLE_TRUE_TO_S = 100.168

OUTPUT_DIR = (
    VALIDATION_ROOT
    / "binary_phase_detector_v24_diagnostic_runs"
    / "2026-08-04_trial08_clearance_gap"
)
DIAGNOSTIC_PATH = OUTPUT_DIR / "clearance_gap_diagnostic.json"
RECEIPT_PATH = OUTPUT_DIR / "diagnostic_receipt.json"

PINNED_INPUTS: dict[str, dict[str, str]] = {
    "v23_freeze": {
        "path": "validation/binary_phase_detector_v23_trial08_development_freeze_lock.json",
        "sha256": "6a958e90dcc7370adfae64c8bd7970bce92d9af227c12bf6019a93ae422072fd",
    },
    "v23_decision": {
        "path": (
            "validation/binary_phase_detector_v23_development_runs/"
            "2026-08-04_trial08_assertions_facts_fix/"
            "trial08_development_decision_lock.json"
        ),
        "sha256": "de55bebbd9b1a21bf3aaadd7132cebed34554e14f5034797b159b14969a4510c",
    },
    "v23_execution_ledger": {
        "path": "validation/binary_phase_detector_v23_trial08_development_execution_ledger.json",
        "sha256": "c77dd856503d377570cf095882156e33057cd9071a04f255561d91e3a8e4645c",
    },
    "v23_access_receipt": {
        "path": (
            "validation/binary_phase_detector_v23_development_runs/"
            "2026-08-04_trial08_assertions_facts_fix/"
            "trial08_development_access_receipt.json"
        ),
        "sha256": "c77dd856503d377570cf095882156e33057cd9071a04f255561d91e3a8e4645c",
    },
    "v23_manifest": {
        "path": (
            "validation/binary_phase_detector_v23_development_runs/"
            "2026-08-04_trial08_assertions_facts_fix/manifest.json"
        ),
        "sha256": "333dc729a88e458a81038a9a90155e4cf08b68daed8aa1ee780be974a8b51f1f",
    },
    "v23_units": {
        "path": (
            "validation/binary_phase_detector_v23_development_runs/"
            "2026-08-04_trial08_assertions_facts_fix/unit_metrics.json"
        ),
        "sha256": "29398345fac488f623b4f053cf3b1af69033c54891a579539f4e7a3f3e55e75b",
    },
    "v23_packed_trace": {
        "path": (
            "validation/binary_phase_detector_v23_development_runs/"
            "2026-08-04_trial08_assertions_facts_fix/packed_binary_trace.json"
        ),
        "sha256": "ed52d2044d5ffb6f74fcb03adad2829834ab41c6eb03e4774f058f1b96795181",
    },
    "v23_event_journal": {
        "path": (
            "validation/binary_phase_detector_v23_development_runs/"
            "2026-08-04_trial08_assertions_facts_fix/event_journal.json"
        ),
        "sha256": "00a135b9e4d735ca2969f2fe1a27e0d7ad6e8434893437ea0ab45770ad008694",
    },
    "profile": {
        "path": (
            "validation/binary_phase_detector_v21_runs/2026-08-04_run01/"
            "eligible_finalist_profile.json"
        ),
        "sha256": "be8e063304a4798e5fc9947beb69c7b2ad813b4cab65e3bfdb0f2cd7284439bc",
    },
    "preprocessing_lock": {
        "path": (
            "validation/two_sensor_cross_speed_v14_2_runs/"
            "2026-07-22_ab06_cross_speed_v14_2_boundary_recovery/"
            "preprocessed/trial_08/treadmill_08_01_preprocessing_lock.json"
        ),
        "sha256": "c961e191871f0caab54edef82424cab20ca3e2938525af8fd1f2752c092399ca",
    },
    "ik_motion": {
        "path": (
            "validation/two_sensor_cross_speed_v14_2_runs/"
            "2026-07-22_ab06_cross_speed_v14_2_boundary_recovery/"
            "preprocessed/trial_08/treadmill_08_01_ik.mot"
        ),
        "sha256": "87af7ec5e6530f179bb2fd84c7f2caf78da041f6dcd92966ac8ca1999e1ce14a",
    },
    "canonical_oracle": {
        "path": (
            "validation/canonical_event_oracles/2026-08-03_v17_development/"
            "trial_08_canonical_event_ledger.json"
        ),
        "sha256": "aa0cf5a2b044bcf5faecf012e8eac5a3693a48459b9dfabc1317536288021f16",
    },
    "marker_model": {
        "path": "models/AB06_SEASEA-raw/osimxml/AB06_SEASEA_marker_calibrated.osim",
        "sha256": "98cfcbc4f7155ea4576f583654fbd50a6e8bd2f2f33ff0894c9f3f24dce5fa8d",
    },
    "audit_source_model": {
        "path": "models/AB06_SEASEA_Threadmill/AB06_SEASEA.osim",
        "sha256": "98cfcbc4f7155ea4576f583654fbd50a6e8bd2f2f33ff0894c9f3f24dce5fa8d",
    },
    "runtime_model": {
        "path": "models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi.osim",
        "sha256": "33e67d84bf11740eac509f620a143ad3c57d98c6f765d857e69c1892513de0c1",
    },
    "compatibility_audit": {
        "path": "validation/primary_grf_model_compatibility_audit_2026-07-24.json",
        "sha256": "7bda56ec2a11f4ec78fd9a379ab6f3e24f2fda31714f2fb94e414363057be848",
    },
    "plugin_binary": {
        "path": "plugins/libSEA_Plugin_BlackBox_mCMC_impedence_ff.dylib",
        "sha256": "77390d0f74055fb3419e88637baac1d215b1dd402ee1effe3e8cb14a66caf54b",
    },
    "binary_detector_source": {
        "path": "binary_phase_detector.py",
        "sha256": "57a313133e1ce5a675b2699e940226325dfa5b2b895c7eb6b17c0892a94263b6",
    },
    "binary_fsm_source": {
        "path": "Trajectory Generator/binary_phase_fsm.py",
        "sha256": "0f7669b60a72c1b27ee3c4f1a43161eeb9f2d091dff5558cc4fa43f1fce8d9c1",
    },
    "online_grf_source": {
        "path": "online_grf.py",
        "sha256": "52e39bf9a3b20dd65242f3f9076d76ed788239fe7c3e5b825bc37a9657c4fefa",
    },
    "kinematics_source": {
        "path": "kinematics_interpolator.py",
        "sha256": "424d352a461b424ed8f7e318513a85b75d3a6fb1a00155eab1e885e9d3fd4ede",
    },
    "model_loader_source": {
        "path": "model_loader.py",
        "sha256": "fba3f025a83082bb07276770b21f644e3c84750402d97c6305c7ea0eef8ccd76",
    },
    "config_source": {
        "path": "config.py",
        "sha256": "88c120bdf8249143a78cd19a33a4de34c10d4230a2ad6760b33dec9bb51417e3",
    },
}

SCOPE = {
    "development_only": True,
    "opened_development_trial": ["08"],
    "protected_trials_opened": [],
    "protected_trials_allowed": False,
    "reserve_trials_opened": [],
    "reserve_trials_allowed": False,
    "h0_execution_allowed": False,
    "runtime_promotion_allowed": False,
    "training_allowed": False,
    "corridor_activation_allowed": False,
    "ppo_allowed": False,
    "geometry_or_fsm_tuning_allowed": False,
}

NON_ACTIONS = {
    "primary_grf_modified": False,
    "binary_detector_profile_modified": False,
    "fsm_modified": False,
    "cpp_or_contact_modified": False,
    "sea_semantics_modified": False,
    "h0_executed": False,
    "protected_trials_accessed": False,
    "runtime_or_training_promoted": False,
}


class V24DiagnosticError(RuntimeError):
    """Raised when diagnostic provenance or numerical evidence is invalid."""


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _reject_json_constant(value: str) -> None:
    raise V24DiagnosticError(f"non-finite JSON constant is forbidden: {value}")


def _strict_json(path: Path, *, expected_sha256: str | None = None) -> dict[str, Any]:
    if not path.is_file():
        raise V24DiagnosticError(f"required JSON is missing: {path}")
    observed = sha256_file(path)
    if expected_sha256 is not None and observed != expected_sha256:
        raise V24DiagnosticError(
            f"hash-pinned JSON drifted: {path}: {observed} != {expected_sha256}"
        )
    try:
        value = json.loads(
            path.read_text(encoding="utf-8"), parse_constant=_reject_json_constant
        )
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise V24DiagnosticError(f"cannot read strict JSON: {path}") from exc
    if not isinstance(value, dict):
        raise V24DiagnosticError(f"JSON root must be an object: {path}")
    return value


def _source_record(path: Path) -> dict[str, Any]:
    resolved = path.resolve()
    if not resolved.is_file():
        raise V24DiagnosticError(f"source is missing: {resolved}")
    return {
        "path": resolved.relative_to(REPO_ROOT.resolve()).as_posix(),
        "sha256": sha256_file(resolved),
        "size_bytes": int(resolved.stat().st_size),
    }


def _verify_pinned_inputs() -> tuple[dict[str, Path], dict[str, dict[str, Any]]]:
    paths: dict[str, Path] = {}
    records: dict[str, dict[str, Any]] = {}
    for label, declaration in PINNED_INPUTS.items():
        path = (REPO_ROOT / declaration["path"]).resolve()
        record = _source_record(path)
        if record["sha256"] != declaration["sha256"]:
            raise V24DiagnosticError(f"hash-pinned input drifted: {label}")
        paths[label] = path
        records[label] = record
    return paths, records


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
    """Publish strict JSON atomically without replacing an existing path."""

    target = Path(path)
    if os.path.lexists(target):
        raise V24DiagnosticError(f"refusing to clobber existing path: {target}")
    if not target.parent.is_dir():
        raise V24DiagnosticError(f"output parent is missing: {target.parent}")
    try:
        encoded = (
            json.dumps(
                dict(payload), indent=2, sort_keys=True, allow_nan=False
            )
            + "\n"
        ).encode("utf-8")
    except (TypeError, ValueError) as exc:
        raise V24DiagnosticError("diagnostic payload is not strict JSON") from exc
    descriptor, temporary_raw = tempfile.mkstemp(
        prefix=f".{target.name}.", suffix=".tmp", dir=str(target.parent)
    )
    temporary = Path(temporary_raw)
    descriptor_open = True
    try:
        with os.fdopen(descriptor, "wb") as stream:
            descriptor_open = False
            stream.write(encoded)
            stream.flush()
            os.fsync(stream.fileno())
        if os.path.lexists(target):
            raise V24DiagnosticError(
                f"refusing to clobber existing path: {target}"
            )
        try:
            os.link(temporary, target)
        except FileExistsError as exc:
            raise V24DiagnosticError(
                f"refusing to clobber existing path: {target}"
            ) from exc
        _fsync_directory(target.parent)
        return target
    finally:
        if descriptor_open:
            os.close(descriptor)
        try:
            temporary.unlink()
        except FileNotFoundError:
            pass


def _require_assertions(assertions: Mapping[str, Any], *, label: str) -> None:
    if not assertions or not all(type(value) is bool for value in assertions.values()):
        raise V24DiagnosticError(f"{label} assertions must be nonempty native bools")
    if not all(assertions.values()):
        raise V24DiagnosticError(f"{label} assertions failed: {dict(assertions)}")


def _time_grid(
    start_s: float = CONTEXT_START_S,
    end_s: float = CONTEXT_END_S,
    sample_dt_s: float = SAMPLE_DT_S,
) -> np.ndarray:
    values = (float(start_s), float(end_s), float(sample_dt_s))
    if not all(math.isfinite(value) for value in values):
        raise V24DiagnosticError("diagnostic time-grid values must be finite")
    start, end, dt = values
    if start >= end or dt <= 0.0:
        raise V24DiagnosticError("diagnostic time-grid interval is invalid")
    span = (end - start) / dt
    count_minus_one = int(round(span))
    if abs(span - count_minus_one) > 1e-9:
        raise V24DiagnosticError("diagnostic interval is not aligned to sample dt")
    times = start + np.arange(count_minus_one + 1, dtype=np.float64) * dt
    if (
        not np.all(np.isfinite(times))
        or abs(float(times[0]) - start) > 1e-12
        or abs(float(times[-1]) - end) > 1e-9
        or not np.allclose(np.diff(times), dt, atol=1e-12, rtol=0.0)
    ):
        raise V24DiagnosticError("diagnostic time grid failed exact validation")
    return times


def _decode_v23_bits(payload: Mapping[str, Any]) -> dict[str, Any]:
    if payload.get("encoding") != "base64_of_numpy_packbits":
        raise V24DiagnosticError("unknown V23 packed-bit encoding")
    if payload.get("bitorder") != "little":
        raise V24DiagnosticError("V23 packed-bit order drifted")
    if payload.get("channel_order_per_sample") != ["left_heel", "left_toe"]:
        raise V24DiagnosticError("V23 packed-bit channel order drifted")
    count = int(payload.get("sample_count", -1))
    if count <= 0:
        raise V24DiagnosticError("V23 packed-bit sample count is invalid")
    try:
        raw = base64.b64decode(str(payload["packed_base64"]), validate=True)
    except Exception as exc:
        raise V24DiagnosticError("V23 packed bits are invalid base64") from exc
    if len(raw) != int(payload.get("packed_size_bytes", -1)):
        raise V24DiagnosticError("V23 packed-bit byte count drifted")
    if hashlib.sha256(raw).hexdigest() != payload.get("packed_sha256"):
        raise V24DiagnosticError("V23 packed-bit hash drifted")
    bits = np.unpackbits(np.frombuffer(raw, dtype=np.uint8), bitorder="little")
    required = count * 2
    if bits.size < required or np.any(bits[required:] != 0):
        raise V24DiagnosticError("V23 packed-bit padding is malformed")
    paired = bits[:required].reshape(count, 2).astype(bool)
    grid = payload.get("time_grid")
    if not isinstance(grid, Mapping):
        raise V24DiagnosticError("V23 packed trace time grid is missing")
    times = _time_grid(
        float(grid["start_s"]),
        float(grid["end_s"]),
        float(grid["sample_dt_s"]),
    )
    if times.size != count:
        raise V24DiagnosticError("V23 packed trace count and grid disagree")
    expected_time_hash = hashlib.sha256(
        times.astype("<f8", copy=False).tobytes()
    ).hexdigest()
    if expected_time_hash != grid.get("time_trace_sha256"):
        raise V24DiagnosticError("V23 packed trace time hash drifted")
    return {
        "time_s": times,
        "heel": paired[:, 0],
        "toe": paired[:, 1],
        "bit_trace_sha256": payload["packed_sha256"],
    }


def _false_runs(contact: np.ndarray) -> list[tuple[int, int]]:
    air = ~np.asarray(contact, dtype=bool)
    padded = np.concatenate(([False], air, [False])).astype(np.int8)
    starts = np.flatnonzero(np.diff(padded) == 1)
    ends = np.flatnonzero(np.diff(padded) == -1) - 1
    return [(int(start), int(end)) for start, end in zip(starts, ends)]


def _clearance_gap_metrics(
    times: np.ndarray,
    heel_clearance_m: np.ndarray,
    toe_clearance_m: np.ndarray,
    *,
    debounce_samples: int = V20_CONFIRMATION_RUN_SAMPLES,
) -> dict[str, Any]:
    time_values = np.asarray(times, dtype=float)
    heel = np.asarray(heel_clearance_m, dtype=float)
    toe = np.asarray(toe_clearance_m, dtype=float)
    if (
        time_values.ndim != 1
        or heel.ndim != 1
        or toe.ndim != 1
        or time_values.size < 2
        or heel.shape != time_values.shape
        or toe.shape != time_values.shape
    ):
        raise V24DiagnosticError("clearance arrays must be aligned nonempty vectors")
    if not (
        np.all(np.isfinite(time_values))
        and np.all(np.isfinite(heel))
        and np.all(np.isfinite(toe))
    ):
        raise V24DiagnosticError("clearance diagnostic contains NaN/Inf")
    if not np.allclose(
        np.diff(time_values), SAMPLE_DT_S, atol=1e-12, rtol=0.0
    ):
        raise V24DiagnosticError("clearance diagnostic must use the 1 ms lattice")
    if type(debounce_samples) is not int or debounce_samples <= 0:
        raise V24DiagnosticError("debounce_samples must be a positive integer")

    heel_contact = heel <= 0.0
    toe_contact = toe <= 0.0
    any_contact = heel_contact | toe_contact
    false_runs = _false_runs(any_contact)
    run_records = [
        {
            "start_time_s": float(time_values[start]),
            "end_time_s": float(time_values[end]),
            "sample_count": int(end - start + 1),
            "sampled_duration_s": float((end - start + 1) * SAMPLE_DT_S),
        }
        for start, end in false_runs
    ]
    critical = [
        (start, end)
        for start, end in false_runs
        if float(time_values[start]) <= V23_FALSE_AIR_ONSET_S + 1e-12
        and float(time_values[end]) >= V23_FALSE_AIR_LAST_SAMPLE_S - 1e-12
    ]
    if len(critical) != 1:
        raise V24DiagnosticError(
            f"expected one V23 false-air run, observed {critical}"
        )
    start, end = critical[0]
    if (
        abs(float(time_values[start]) - V23_FALSE_AIR_ONSET_S) > 1e-9
        or abs(float(time_values[end]) - V23_FALSE_AIR_LAST_SAMPLE_S) > 1e-9
        or end - start + 1 != V23_FALSE_AIR_SAMPLE_COUNT
    ):
        raise V24DiagnosticError("V23 critical false-air run boundaries drifted")

    envelope = np.minimum(heel[start : end + 1], toe[start : end + 1])
    full_shift = max(0.0, float(np.max(envelope)))
    limiting_local = int(np.argmax(envelope))
    limiting_index = start + limiting_local
    limiting_sensor = (
        "left_heel"
        if heel[limiting_index] <= toe[limiting_index]
        else "left_toe"
    )

    candidate_shifts = sorted({0.0, *(max(0.0, float(v)) for v in envelope)})
    debounce_shift: float | None = None
    debounce_longest_after: int | None = None
    for shift in candidate_shifts:
        shifted_contact = envelope <= shift
        shifted_runs = _false_runs(shifted_contact)
        longest = max((finish - begin + 1 for begin, finish in shifted_runs), default=0)
        if longest < debounce_samples:
            debounce_shift = float(shift)
            debounce_longest_after = int(longest)
            break
    if debounce_shift is None or debounce_longest_after is None:
        raise V24DiagnosticError("cannot derive finite debounce-breaking margin")

    true_swing_runs = [
        record
        for record in run_records
        if record["start_time_s"] >= V23_RECONTACT_ONSET_S - 1e-12
        and record["end_time_s"] >= ORACLE_TRUE_TO_S - 0.030
    ]
    return {
        "sample_count": int(time_values.size),
        "sample_dt_s": SAMPLE_DT_S,
        "contact_rule": "signed_clearance_le_zero",
        "false_air_runs": run_records,
        "critical_gap": {
            "start_time_s": float(time_values[start]),
            "end_time_s": float(time_values[end]),
            "recontact_onset_s": V23_RECONTACT_ONSET_S,
            "sample_count": int(end - start + 1),
            "heel_clearance_start_mm": float(heel[start] * 1000.0),
            "heel_clearance_end_mm": float(heel[end] * 1000.0),
            "toe_clearance_start_mm": float(toe[start] * 1000.0),
            "toe_clearance_end_mm": float(toe[end] * 1000.0),
            "minimum_sensor_clearance_by_sample_mm": [
                float(value * 1000.0) for value in envelope
            ],
        },
        "full_uniform_normal_shift_to_close_gap_m": full_shift,
        "full_uniform_normal_shift_to_close_gap_mm": full_shift * 1000.0,
        "full_closure_limiting_time_s": float(time_values[limiting_index]),
        "full_closure_limiting_sensor": limiting_sensor,
        "minimum_uniform_shift_to_break_debounce_m": debounce_shift,
        "minimum_uniform_shift_to_break_debounce_mm": debounce_shift * 1000.0,
        "v20_debounce_interval_s": V20_DEBOUNCE_INTERVAL_S,
        "v20_confirmation_run_samples_inclusive": debounce_samples,
        "longest_air_run_after_debounce_shift_samples": debounce_longest_after,
        "true_swing_runs_in_context": true_swing_runs,
        "interpretation_limit": (
            "normal-direction clearance margins are diagnostic counterfactuals, "
            "not authorization to move points or the ground plane"
        ),
    }


def _files_byte_identical(left: Path, right: Path) -> bool:
    if left.stat().st_size != right.stat().st_size:
        return False
    with left.open("rb") as left_stream, right.open("rb") as right_stream:
        while True:
            left_chunk = left_stream.read(1024 * 1024)
            right_chunk = right_stream.read(1024 * 1024)
            if left_chunk != right_chunk:
                return False
            if not left_chunk:
                return True


def _oracle_context(ledger: Mapping[str, Any]) -> dict[str, Any]:
    core = ledger.get("scientific_core")
    if not isinstance(core, Mapping):
        raise V24DiagnosticError("canonical oracle scientific_core is missing")
    events_raw = core.get("events")
    if not isinstance(events_raw, Sequence):
        raise V24DiagnosticError("canonical oracle events are missing")
    events = [
        event
        for event in events_raw
        if isinstance(event, Mapping)
        and 99.5 <= float(event.get("event_time_s", math.nan)) <= 100.3
    ]
    compact = [
        {
            "event": str(event["event"]),
            "event_time_s": float(event["event_time_s"]),
            "event_id": str(event["event_id"]),
            "contact_id": str(event["contact_id"]),
        }
        for event in events
    ]
    assertions = {
        "trial_exact": core.get("trial_id") == TRIAL_ID,
        "sample_dt_exact": float(core.get("sample_dt_s", math.nan))
        == SAMPLE_DT_S,
        "two_neighbor_events": len(compact) == 2,
        "previous_hs_exact": bool(
            len(compact) == 2
            and compact[0]["event"] == "heel_strike"
            and abs(compact[0]["event_time_s"] - 99.662) <= 1e-12
        ),
        "true_to_exact": bool(
            len(compact) == 2
            and compact[1]["event"] == "toe_off"
            and abs(compact[1]["event_time_s"] - ORACLE_TRUE_TO_S) <= 1e-12
        ),
        "same_oracle_stance": bool(
            len(compact) == 2
            and compact[0]["contact_id"] == compact[1]["contact_id"]
        ),
        "no_oracle_event_in_false_gap": not any(
            V23_FALSE_AIR_ONSET_S - 1e-12
            <= item["event_time_s"]
            <= V23_RECONTACT_ONSET_S + 1e-12
            for item in compact
        ),
    }
    _require_assertions(assertions, label="oracle context")
    return {
        "events": compact,
        "assertions": assertions,
        "interpretation": (
            "the V23 OFF/OFF run lies inside one accepted oracle stance; "
            "the following long OFF run is the real swing"
        ),
    }


def preflight_unopened() -> dict[str, Any]:
    """Validate lineage and compatibility declarations without sampling/writes."""

    paths, records = _verify_pinned_inputs()
    decision = _strict_json(
        paths["v23_decision"],
        expected_sha256=PINNED_INPUTS["v23_decision"]["sha256"],
    )
    freeze = _strict_json(
        paths["v23_freeze"],
        expected_sha256=PINNED_INPUTS["v23_freeze"]["sha256"],
    )
    compatibility = _strict_json(
        paths["compatibility_audit"],
        expected_sha256=PINNED_INPUTS["compatibility_audit"]["sha256"],
    )
    preprocessing = _strict_json(
        paths["preprocessing_lock"],
        expected_sha256=PINNED_INPUTS["preprocessing_lock"]["sha256"],
    )
    oracle = _strict_json(
        paths["canonical_oracle"],
        expected_sha256=PINNED_INPUTS["canonical_oracle"]["sha256"],
    )
    packed_payload = _strict_json(
        paths["v23_packed_trace"],
        expected_sha256=PINNED_INPUTS["v23_packed_trace"]["sha256"],
    )
    decoded = _decode_v23_bits(packed_payload)
    marker_equals_source = _files_byte_identical(
        paths["marker_model"], paths["audit_source_model"]
    )
    topology = compatibility.get("topology_checks")
    if not isinstance(topology, Mapping):
        raise V24DiagnosticError("compatibility topology checks are missing")
    compatibility_declarations = {
        "marker_and_audit_source_model_byte_identical": marker_equals_source,
        "marker_and_audit_source_sha_exact": (
            records["marker_model"]["sha256"]
            == records["audit_source_model"]["sha256"]
        ),
        "compatibility_audit_pass": compatibility.get("status") == "PASS",
        "compatibility_decision_exact": compatibility.get("decision")
        == "IK_GEOMETRY_COMPATIBLE_WITH_RUNTIME_MODEL",
        "body_set_match": bool(topology.get("body_set", {}).get("match")),
        "joint_set_match": bool(topology.get("joint_set", {}).get("match")),
        "marker_set_match": bool(topology.get("marker_set", {}).get("match")),
        "contact_geometry_set_match": bool(
            topology.get("contact_geometry_set", {}).get("match")
        ),
        "coordinate_names_match": bool(
            topology.get("coordinate_names", {}).get("match")
        ),
    }
    _require_assertions(
        compatibility_declarations, label="model compatibility declarations"
    )
    lineage_assertions = {
        "v23_terminal_fail_exact": decision.get("status")
        == "FAIL_V23_TRIAL08_OPEN_DEVELOPMENT_REPLAY_TERMINAL",
        "v23_pass_false": decision.get("pass") is False,
        "v23_cycle_consumed": decision.get("cycle_consumed") is True,
        "v23_rerun_forbidden": decision.get("rerun_allowed") is False,
        "candidate_exact": decision.get("candidate_id") == CANDIDATE_ID,
        "v23_unit_result_exact": (
            decision.get("unit_count") == 8
            and decision.get("unit_pass_count") == 6
        ),
        "freeze_candidate_exact": freeze.get("candidate", {}).get("candidate_id")
        == CANDIDATE_ID,
        "ledger_receipt_byte_identical": _files_byte_identical(
            paths["v23_execution_ledger"], paths["v23_access_receipt"]
        ),
        "trial08_preprocessing_exact": preprocessing.get("trial_id") == TRIAL_ID,
        "trial08_already_open_development": True,
        "packed_trace_contains_context": bool(
            float(decoded["time_s"][0]) <= CONTEXT_START_S
            and float(decoded["time_s"][-1]) >= CONTEXT_END_S
        ),
    }
    _require_assertions(lineage_assertions, label="V23 lineage")
    oracle_context = _oracle_context(oracle)
    output_unoccupied = not any(
        os.path.lexists(path)
        for path in (OUTPUT_DIR, DIAGNOSTIC_PATH, RECEIPT_PATH)
    )
    return {
        "schema_version": SCHEMA_VERSION,
        "diagnostic_id": DIAGNOSTIC_ID,
        "status": "V24_CLEARANCE_GAP_DIAGNOSTIC_PREFLIGHT_READY_UNOPENED",
        "scope": dict(SCOPE),
        "non_actions": dict(NON_ACTIONS),
        "lineage_assertions": lineage_assertions,
        "compatibility_declarations": compatibility_declarations,
        "oracle_context": oracle_context,
        "time_grid": {
            "start_s": CONTEXT_START_S,
            "end_s": CONTEXT_END_S,
            "sample_dt_s": SAMPLE_DT_S,
            "sample_count": int(_time_grid().size),
        },
        "paths": {
            "output_dir": OUTPUT_DIR.relative_to(REPO_ROOT).as_posix(),
            "diagnostic": DIAGNOSTIC_PATH.relative_to(REPO_ROOT).as_posix(),
            "receipt": RECEIPT_PATH.relative_to(REPO_ROOT).as_posix(),
            "protected_trials": [],
            "reserve_trials": [],
        },
        "output_unoccupied": output_unoccupied,
        "opensim_sampling_started": False,
        "platform": {
            "system": platform.system(),
            "machine": platform.machine(),
            "numerical_claim_scope": "macOS_arm64",
        },
        "sources": records,
    }


def _sample_model(
    *,
    model_path: Path,
    profile_path: Path,
    ik_path: Path,
    times: np.ndarray,
) -> dict[str, Any]:
    import opensim

    from binary_phase_detector import (
        BinaryPhaseDetectorSampler,
        load_binary_phase_detector_profile,
    )
    from config import SimulatorConfig
    from kinematics_interpolator import KinematicsInterpolator

    profile = load_binary_phase_detector_profile(profile_path)
    model = opensim.Model(str(model_path))
    state = model.initSystem()
    sampler = BinaryPhaseDetectorSampler(model, profile)
    frame = opensim.PhysicalFrame.safeDownCast(
        model.getComponent("/bodyset/foot_l")
    )
    if frame is None:
        raise V24DiagnosticError("cannot resolve runtime foot PhysicalFrame")
    cfg = SimulatorConfig()
    cfg.model_bundle_dir = str(model_path.parent)
    cfg.model_file = str(model_path)
    cfg.kinematics_file = str(ik_path)
    cfg.t_start = float(times[0])
    cfg.t_end = float(times[-1])
    kinematics = KinematicsInterpolator(cfg)
    coordinate_set = model.getCoordinateSet()
    heel = np.empty(times.size, dtype=float)
    toe = np.empty(times.size, dtype=float)
    heel_bits = np.empty(times.size, dtype=bool)
    toe_bits = np.empty(times.size, dtype=bool)
    reach_gain = np.empty(times.size, dtype=float)
    ground_normal = np.asarray(profile.ground.normal, dtype=float)
    local_origin = opensim.Vec3(0.0, 0.0, 0.0)
    local_y = opensim.Vec3(0.0, 1.0, 0.0)
    for row, time_s in enumerate(times):
        q, _qdot, _qddot = kinematics.get(float(time_s))
        state.setTime(float(time_s))
        for index in range(coordinate_set.getSize()):
            coordinate = coordinate_set.get(index)
            name = coordinate.getName()
            if name in q:
                coordinate.setValue(state, float(q[name]), False)
        reading = sampler.sample(state, float(time_s))
        heel[row] = float(reading.signed_clearance_m["left_heel"])
        toe[row] = float(reading.signed_clearance_m["left_toe"])
        heel_bits[row] = reading.contacts["left_heel"]
        toe_bits[row] = reading.contacts["left_toe"]
        ground_origin = frame.findStationLocationInGround(state, local_origin)
        ground_y = frame.findStationLocationInGround(state, local_y)
        direction = np.asarray(
            [
                float(ground_y.get(axis) - ground_origin.get(axis))
                for axis in range(3)
            ],
            dtype=float,
        )
        reach_gain[row] = float(ground_normal @ direction)
    arrays = (heel, toe, reach_gain)
    if any(not np.all(np.isfinite(values)) for values in arrays):
        raise V24DiagnosticError("paired model sampling produced NaN/Inf")
    if np.any(reach_gain <= 0.0):
        raise V24DiagnosticError(
            "increasing plantar reach does not monotonically reduce clearance"
        )
    return {
        "time_s": np.asarray(times, dtype=float),
        "heel_clearance_m": heel,
        "toe_clearance_m": toe,
        "heel_contact": heel_bits,
        "toe_contact": toe_bits,
        "reach_clearance_reduction_per_m": reach_gain,
    }


def _v23_context_bits(
    decoded: Mapping[str, Any], times: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    source_times = np.asarray(decoded["time_s"], dtype=float)
    indices = np.rint((times - source_times[0]) / SAMPLE_DT_S).astype(int)
    if (
        np.any(indices < 0)
        or np.any(indices >= source_times.size)
        or not np.allclose(source_times[indices], times, atol=1e-12, rtol=0.0)
    ):
        raise V24DiagnosticError("diagnostic context is not on V23 trace lattice")
    return (
        np.asarray(decoded["heel"], dtype=bool)[indices],
        np.asarray(decoded["toe"], dtype=bool)[indices],
    )


def _reach_counterfactual(
    times: np.ndarray,
    heel_clearance_m: np.ndarray,
    toe_clearance_m: np.ndarray,
    reach_gain: np.ndarray,
) -> dict[str, Any]:
    start = int(
        np.flatnonzero(
            np.isclose(times, V23_FALSE_AIR_ONSET_S, atol=1e-12, rtol=0.0)
        )[0]
    )
    end = int(
        np.flatnonzero(
            np.isclose(
                times, V23_FALSE_AIR_LAST_SAMPLE_S, atol=1e-12, rtol=0.0
            )
        )[0]
    )
    gain = np.asarray(reach_gain[start : end + 1], dtype=float)
    heel_required = np.maximum(
        np.asarray(heel_clearance_m[start : end + 1], dtype=float), 0.0
    ) / gain
    toe_required = np.maximum(
        np.asarray(toe_clearance_m[start : end + 1], dtype=float), 0.0
    ) / gain
    common_required = np.minimum(heel_required, toe_required)
    return {
        "critical_interval_s": [
            V23_FALSE_AIR_ONSET_S,
            V23_FALSE_AIR_LAST_SAMPLE_S,
        ],
        "reach_gain_minimum": float(np.min(gain)),
        "reach_gain_maximum": float(np.max(gain)),
        "minimum_toe_only_reach_increase_to_close_gap_m": float(
            np.max(toe_required)
        ),
        "minimum_toe_only_reach_increase_to_close_gap_mm": float(
            1000.0 * np.max(toe_required)
        ),
        "minimum_heel_only_reach_increase_to_close_gap_m": float(
            np.max(heel_required)
        ),
        "minimum_heel_only_reach_increase_to_close_gap_mm": float(
            1000.0 * np.max(heel_required)
        ),
        "minimum_common_reach_increase_to_close_gap_m": float(
            np.max(common_required)
        ),
        "minimum_common_reach_increase_to_close_gap_mm": float(
            1000.0 * np.max(common_required)
        ),
        "status": "DIAGNOSTIC_COUNTERFACTUAL_NOT_A_GEOMETRY_SELECTION",
    }


def execute_development_diagnostic() -> dict[str, Any]:
    preflight = preflight_unopened()
    if not preflight["output_unoccupied"]:
        raise V24DiagnosticError("V24 diagnostic destination is already occupied")
    if platform.system() != "Darwin" or platform.machine().lower() not in {
        "arm64",
        "aarch64",
    }:
        raise V24DiagnosticError("V24 numerical diagnostic is macOS-arm64 only")
    paths, records = _verify_pinned_inputs()
    packed_payload = _strict_json(
        paths["v23_packed_trace"],
        expected_sha256=PINNED_INPUTS["v23_packed_trace"]["sha256"],
    )
    decoded = _decode_v23_bits(packed_payload)
    times = _time_grid()

    from model_loader import _load_plugin

    _load_plugin(str(REPO_ROOT / "plugins/SEA_Plugin_BlackBox_mCMC_impedence_ff"))
    marker = _sample_model(
        model_path=paths["marker_model"],
        profile_path=paths["profile"],
        ik_path=paths["ik_motion"],
        times=times,
    )
    runtime = _sample_model(
        model_path=paths["runtime_model"],
        profile_path=paths["profile"],
        ik_path=paths["ik_motion"],
        times=times,
    )
    v23_heel, v23_toe = _v23_context_bits(decoded, times)
    heel_diff = np.abs(
        marker["heel_clearance_m"] - runtime["heel_clearance_m"]
    )
    toe_diff = np.abs(marker["toe_clearance_m"] - runtime["toe_clearance_m"])
    gain_diff = np.abs(
        marker["reach_clearance_reduction_per_m"]
        - runtime["reach_clearance_reduction_per_m"]
    )
    numerical_assertions = {
        "marker_runtime_heel_bits_exact": bool(
            np.array_equal(marker["heel_contact"], runtime["heel_contact"])
        ),
        "marker_runtime_toe_bits_exact": bool(
            np.array_equal(marker["toe_contact"], runtime["toe_contact"])
        ),
        "marker_runtime_clearance_within_1e_12_m": bool(
            max(float(np.max(heel_diff)), float(np.max(toe_diff))) <= 1e-12
        ),
        "marker_runtime_reach_gain_within_1e_12": bool(
            float(np.max(gain_diff)) <= 1e-12
        ),
        "marker_bits_equal_immutable_v23": bool(
            np.array_equal(marker["heel_contact"], v23_heel)
            and np.array_equal(marker["toe_contact"], v23_toe)
        ),
        "runtime_bits_equal_immutable_v23": bool(
            np.array_equal(runtime["heel_contact"], v23_heel)
            and np.array_equal(runtime["toe_contact"], v23_toe)
        ),
        "all_clearances_finite": bool(
            np.all(np.isfinite(marker["heel_clearance_m"]))
            and np.all(np.isfinite(marker["toe_clearance_m"]))
            and np.all(np.isfinite(runtime["heel_clearance_m"]))
            and np.all(np.isfinite(runtime["toe_clearance_m"]))
        ),
    }
    _require_assertions(numerical_assertions, label="paired model sampling")
    gap_metrics = _clearance_gap_metrics(
        times,
        marker["heel_clearance_m"],
        marker["toe_clearance_m"],
    )
    reach_metrics = _reach_counterfactual(
        times,
        marker["heel_clearance_m"],
        marker["toe_clearance_m"],
        marker["reach_clearance_reduction_per_m"],
    )
    samples = []
    for index, time_s in enumerate(times):
        samples.append(
            {
                "time_s": float(time_s),
                "marker_model": {
                    "left_heel_clearance_m": float(
                        marker["heel_clearance_m"][index]
                    ),
                    "left_toe_clearance_m": float(
                        marker["toe_clearance_m"][index]
                    ),
                    "left_heel_contact": bool(marker["heel_contact"][index]),
                    "left_toe_contact": bool(marker["toe_contact"][index]),
                },
                "runtime_model": {
                    "left_heel_clearance_m": float(
                        runtime["heel_clearance_m"][index]
                    ),
                    "left_toe_clearance_m": float(
                        runtime["toe_clearance_m"][index]
                    ),
                    "left_heel_contact": bool(runtime["heel_contact"][index]),
                    "left_toe_contact": bool(runtime["toe_contact"][index]),
                },
                "reach_clearance_reduction_per_m": float(
                    marker["reach_clearance_reduction_per_m"][index]
                ),
            }
        )
    diagnostic = {
        "schema_version": SCHEMA_VERSION,
        "diagnostic_id": DIAGNOSTIC_ID,
        "status": "PASS_V24_TRIAL08_GEOMETRIC_CLEARANCE_GAP_CONFIRMED",
        "pass": True,
        "created_at_utc": _utc_now(),
        "trial_id": TRIAL_ID,
        "trial_role": "ALREADY_OPEN_DEVELOPMENT_NOT_HOLDOUT",
        "candidate_id": CANDIDATE_ID,
        "scope": dict(SCOPE),
        "non_actions": dict(NON_ACTIONS),
        "lineage_assertions": preflight["lineage_assertions"],
        "compatibility_declarations": preflight[
            "compatibility_declarations"
        ],
        "oracle_context": preflight["oracle_context"],
        "numerical_assertions": numerical_assertions,
        "marker_runtime_comparison": {
            "marker_runtime_clearance_equivalent": True,
            "maximum_absolute_heel_clearance_difference_m": float(
                np.max(heel_diff)
            ),
            "maximum_absolute_toe_clearance_difference_m": float(
                np.max(toe_diff)
            ),
            "maximum_absolute_reach_gain_difference": float(np.max(gain_diff)),
            "tolerance_m": 1e-12,
        },
        "gap_metrics": gap_metrics,
        "reach_counterfactual": reach_metrics,
        "samples": samples,
        "conclusion": {
            "geometry_or_kinematic_signal_gap_confirmed": True,
            "marker_runtime_model_mismatch_excluded": True,
            "fsm_created_raw_gap": False,
            "fsm_change_required_by_this_diagnostic": False,
            "recommended_next_stage": (
                "V25_TARGETED_OPEN_DEVELOPMENT_GEOMETRY_SWEEP_DEV02_04_08"
            ),
        },
        "sources": records,
    }
    json.dumps(diagnostic, allow_nan=False)
    OUTPUT_DIR.mkdir(parents=True, exist_ok=False)
    try:
        _write_json_exclusive(DIAGNOSTIC_PATH, diagnostic)
        diagnostic_record = _source_record(DIAGNOSTIC_PATH)
        receipt = {
            "schema_version": SCHEMA_VERSION,
            "diagnostic_id": DIAGNOSTIC_ID,
            "status": "PASS_V24_TRIAL08_GEOMETRIC_CLEARANCE_GAP_CONFIRMED",
            "pass": True,
            "created_at_utc": _utc_now(),
            "trial_id": TRIAL_ID,
            "candidate_id": CANDIDATE_ID,
            "marker_runtime_clearance_equivalent": True,
            "v23_false_air_run_reproduced": True,
            "critical_gap_sample_count": int(
                gap_metrics["critical_gap"]["sample_count"]
            ),
            "full_uniform_normal_shift_to_close_gap_mm": float(
                gap_metrics["full_uniform_normal_shift_to_close_gap_mm"]
            ),
            "minimum_toe_only_reach_increase_to_close_gap_mm": float(
                reach_metrics[
                    "minimum_toe_only_reach_increase_to_close_gap_mm"
                ]
            ),
            "diagnostic": diagnostic_record,
            "script": _source_record(Path(__file__)),
            "scope": dict(SCOPE),
            "non_actions": dict(NON_ACTIONS),
            "next_stage": (
                "V25_TARGETED_OPEN_DEVELOPMENT_GEOMETRY_SWEEP_DEV02_04_08"
            ),
        }
        _write_json_exclusive(RECEIPT_PATH, receipt)
    except BaseException:
        # Preserve any materialized evidence; exclusive semantics intentionally
        # prevent silently converting a partial execution into a rerun.
        raise
    return receipt


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument(
        "--check",
        action="store_true",
        help="validate frozen inputs and scope without sampling or writing",
    )
    group.add_argument(
        "--execute-development-diagnostic",
        action="store_true",
        help="execute the hash-pinned trial-08 diagnostic exactly once",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        result = (
            preflight_unopened()
            if args.check
            else execute_development_diagnostic()
        )
    except Exception as exc:
        print(
            f"V24 clearance diagnostic failed closed: {type(exc).__name__}: {exc}",
            file=sys.stderr,
        )
        return 2
    print(json.dumps(result, indent=2, sort_keys=True, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
