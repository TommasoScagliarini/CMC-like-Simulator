"""Run the frozen V21 detector/FSM challenge on AB06 trial 08 exactly once.

Trial 08 is an internal procedural holdout relative to the V21 DEV02/04 sweep;
it is not scientifically virgin across the older V14/V17 lineage.  ``--check``
never touches trial-08 semantic inputs.  ``--execute-one-shot`` first publishes
an immutable global execution ledger and access receipt, then and only then
opens the trial-08 preprocessing lock, IK motion, and canonical event oracle.

PASS, FAIL, an exception, or an interrupt after the opening ledger permanently
consume this stage.  No candidate selector or tuning parameter is exposed.
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
import traceback
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Mapping, Sequence, TextIO

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
TRAJECTORY_ROOT = REPO_ROOT / "Trajectory Generator"
VALIDATION_ROOT = REPO_ROOT / "validation"
for import_root in (REPO_ROOT, TRAJECTORY_ROOT, VALIDATION_ROOT):
    if str(import_root) not in sys.path:
        sys.path.insert(0, str(import_root))

import freeze_binary_phase_detector_v21_trial08 as freeze  # noqa: E402


SCHEMA_VERSION = 1
RUN_ID = "AB06_BINARY_POINT_V21_TRIAL08_PROCEDURAL_ONE_SHOT"
MODES = ("sequential_1ms", "batched_10ms_same_samples")
ACCESS_RECEIPT_NAME = "trial08_access_receipt.json"
FAILURE_NAME = "failure.json"
DECISION_NAME = "trial08_decision_lock.json"
MANIFEST_NAME = "manifest.json"

PLUGIN_LOADER = "plugins/SEA_Plugin_BlackBox_mCMC_impedence_ff"

_EXECUTION_AUDIT: dict[str, Any] = {
    "stage": "pre_open",
    "trial08_inputs_attempted": [],
    "trial08_inputs_verified": [],
}


class V21Trial08Error(RuntimeError):
    """Raised when the frozen trial-08 contract cannot be preserved."""


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def _audit_stage(stage: str) -> None:
    _EXECUTION_AUDIT["stage"] = str(stage)


def _audit_trial_input(label: str, *, verified: bool) -> None:
    key = "trial08_inputs_verified" if verified else "trial08_inputs_attempted"
    inputs = _EXECUTION_AUDIT[key]
    if label not in inputs:
        inputs.append(str(label))


def _reject_json_constant(value: str) -> None:
    raise V21Trial08Error(f"non-finite JSON constant is forbidden: {value}")


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


def _strict_json(path: Path, *, expected_sha256: str | None = None) -> dict[str, Any]:
    if not path.is_file():
        raise V21Trial08Error(f"required JSON is missing: {path}")
    observed_sha = sha256_file(path)
    if expected_sha256 is not None and observed_sha != expected_sha256:
        raise V21Trial08Error(
            f"hash-pinned JSON drifted: {path}: {observed_sha} != {expected_sha256}"
        )
    try:
        value = json.loads(
            path.read_text(encoding="utf-8"),
            parse_constant=_reject_json_constant,
        )
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise V21Trial08Error(f"cannot read strict JSON: {path}") from exc
    if not isinstance(value, dict):
        raise V21Trial08Error(f"JSON root must be an object: {path}")
    return value


def _resolve_repo_path(relative: str) -> Path:
    value = Path(relative)
    if value.is_absolute() or ".." in value.parts:
        raise V21Trial08Error(f"non-portable repository path: {relative!r}")
    resolved = (REPO_ROOT / value).resolve()
    try:
        resolved.relative_to(REPO_ROOT.resolve())
    except ValueError as exc:
        raise V21Trial08Error(f"path escapes repository: {relative!r}") from exc
    return resolved


def _source_record(path: Path) -> dict[str, Any]:
    resolved = path.resolve()
    if not resolved.is_file():
        raise V21Trial08Error(f"source is missing: {resolved}")
    return {
        "path": resolved.relative_to(REPO_ROOT.resolve()).as_posix(),
        "sha256": sha256_file(resolved),
        "size_bytes": int(resolved.stat().st_size),
    }


def _verify_source_record(record: Mapping[str, Any], *, label: str) -> Path:
    if set(record) != {"path", "sha256", "size_bytes"}:
        raise V21Trial08Error(f"malformed source record: {label}")
    path = _resolve_repo_path(str(record["path"]))
    if not path.is_file():
        raise V21Trial08Error(f"missing source: {label}")
    if int(path.stat().st_size) != int(record["size_bytes"]):
        raise V21Trial08Error(f"source size drifted: {label}")
    if sha256_file(path) != str(record["sha256"]):
        raise V21Trial08Error(f"source hash drifted: {label}")
    return path


def _validate_freeze_lock() -> tuple[dict[str, Any], dict[str, Any]]:
    lock = _strict_json(freeze.FREEZE_PATH)
    checks = {
        "schema": lock.get("schema_version") == freeze.SCHEMA_VERSION,
        "freeze_id": lock.get("freeze_id") == freeze.FREEZE_ID,
        "status": lock.get("status")
        == "V21_FROZEN_BEFORE_TRIAL08_PROCEDURAL_ONE_SHOT",
        "frozen": lock.get("frozen_before_trial08_v21_performance_access") is True,
        "candidate": lock.get("candidate", {}).get("candidate_id")
        == freeze.CANDIDATE_ID,
        "single_candidate": lock.get("candidate", {}).get(
            "candidate_count_after_freeze"
        )
        == 1,
        "candidate_sample_dt": lock.get("candidate", {}).get("sample_dt_s")
        == freeze.SAMPLE_DT_S,
        "candidate_policy_step": lock.get("candidate", {}).get("policy_step_s")
        == freeze.POLICY_STEP_S,
        "candidate_debounce": lock.get("candidate", {}).get("debounce_s")
        == freeze.DEBOUNCE_S,
        "profile": lock.get("candidate", {}).get("profile", {}).get("sha256")
        == freeze.PROFILE_SHA256,
        "trial": lock.get("trial", {}).get("trial_id") == freeze.TRIAL_ID,
        "role": lock.get("trial", {}).get("role") == freeze.TRIAL_ROLE,
        "not_virgin": lock.get("trial", {}).get("scientifically_virgin") is False,
        "claim_limit": lock.get("trial", {}).get("claim_limit")
        == freeze.CLAIM_LIMIT,
        "sample_count": lock.get("trial", {}).get("expected_sample_count")
        == freeze.EXPECTED_SAMPLE_COUNT,
        "interval": lock.get("trial", {}).get("interval_s")
        == list(freeze.TRIAL_INTERVAL_S),
        "modes": lock.get("gate", {}).get("required_consumption_modes")
        == list(MODES),
        "units": lock.get("gate", {}).get("required_unit_count") == 8,
        "gate_exact": lock.get("gate") == freeze.FROZEN_GATE,
        "no_retry": lock.get("opening_contract", {}).get("retry_allowed")
        is False,
        "opening_contract_exact": lock.get("opening_contract")
        == freeze.OPENING_CONTRACT,
        "fixed_ledger": lock.get("opening_contract", {}).get(
            "global_execution_ledger"
        )
        == freeze.EXECUTION_LEDGER_PATH.relative_to(REPO_ROOT).as_posix(),
        "fixed_output": lock.get("opening_contract", {}).get("output_dir")
        == freeze.OUTPUT_DIR.relative_to(REPO_ROOT).as_posix(),
        "no_runtime_promotion": lock.get("post_pass_scope", {}).get(
            "runtime_promotion_allowed"
        )
        is False,
        "no_training_promotion": lock.get("post_pass_scope", {}).get(
            "training_promotion_allowed"
        )
        is False,
        "post_pass_scope_exact": lock.get("post_pass_scope")
        == freeze.POST_PASS_SCOPE,
        "non_actions_exact": lock.get("non_actions") == freeze.NON_ACTIONS,
        "current_fsm_contract": lock.get("candidate", {}).get(
            "fsm_current_implementation_contract_id"
        )
        == "binary_point_v19+functional_contact_fsm_v1_shadow",
        "target_bundle_contract": lock.get("candidate", {}).get(
            "target_bundle_contract_id_after_h0_integration"
        )
        == "primary_grf_split_v1+binary_point_v21+functional_contact_fsm_v1",
        "metadata_access_disclosed": lock.get("data_governance", {})
        .get("current_cycle_pre_freeze_nonperformance_metadata_access", {})
        .get("occurred")
        is True,
        "performance_blinding_preserved": lock.get("data_governance", {})
        .get("current_cycle_pre_freeze_nonperformance_metadata_access", {})
        .get("scientific_performance_blinding_preserved")
        is True,
    }
    if not all(checks.values()):
        raise V21Trial08Error(f"freeze contract drifted: {checks}")

    sources = lock.get("sources")
    if not isinstance(sources, Mapping) or set(sources) != set(
        freeze.PINNED_PREACCESS_SOURCES
    ) | set(freeze.DYNAMIC_FREEZE_SOURCES):
        raise V21Trial08Error("freeze source keyset drifted")
    for label, record in sources.items():
        if not isinstance(record, Mapping):
            raise V21Trial08Error(f"invalid freeze source record: {label}")
        _verify_source_record(record, label=f"freeze.sources.{label}")
        if label in freeze.PINNED_PREACCESS_SOURCES:
            if record.get("path") != label or record.get("sha256") != (
                freeze.PINNED_PREACCESS_SOURCES[label]
            ):
                raise V21Trial08Error(f"freeze source lost static binding: {label}")
        elif record.get("path") != label:
            raise V21Trial08Error(f"dynamic freeze source path drifted: {label}")

    candidate = lock["candidate"]
    for label in ("profile",):
        _verify_source_record(candidate[label], label=f"candidate.{label}")
    if candidate["profile"] != {
        "path": freeze.PROFILE_PATH.relative_to(REPO_ROOT).as_posix(),
        "sha256": freeze.PROFILE_SHA256,
        "size_bytes": int(freeze.PROFILE_PATH.stat().st_size),
    }:
        raise V21Trial08Error("candidate profile static binding drifted")
    if (
        candidate.get("geometry") != freeze.CANDIDATE_GEOMETRY
        or candidate.get("geometry_sha256")
        != canonical_sha256(freeze.CANDIDATE_GEOMETRY)
    ):
        raise V21Trial08Error("candidate geometry static binding drifted")
    selection = lock.get("v21_selection")
    if not isinstance(selection, Mapping):
        raise V21Trial08Error("V21 selection receipt is missing from freeze")
    for label in ("manifest", "final_verification"):
        record = selection.get(label)
        if not isinstance(record, Mapping):
            raise V21Trial08Error(f"freeze lost V21 {label}")
        _verify_source_record(record, label=f"v21_selection.{label}")
    if selection["manifest"].get("path") != freeze.MANIFEST_PATH.relative_to(
        REPO_ROOT
    ).as_posix() or selection["manifest"].get("sha256") != freeze.MANIFEST_SHA256:
        raise V21Trial08Error("V21 manifest static binding drifted")
    if selection["final_verification"].get("path") != (
        freeze.FINAL_VERIFICATION_PATH.relative_to(REPO_ROOT).as_posix()
    ) or selection["final_verification"].get("sha256") != (
        freeze.FINAL_VERIFICATION_SHA256
    ):
        raise V21Trial08Error("V21 final verification static binding drifted")

    declarations = lock.get("trial", {}).get(
        "declared_inputs_not_verified_or_opened_by_freeze"
    )
    if declarations != freeze.TRIAL08_DECLARATIONS:
        raise V21Trial08Error("trial-08 declared input contract drifted")
    # Deliberately do not resolve, stat, hash, or parse any declaration here.
    return lock, checks


def _verify_common_input(declaration: Mapping[str, Any], *, label: str) -> Path:
    """Verify a non-trial-specific runtime file without touching trial-08 data."""

    path = _resolve_repo_path(str(declaration["path"]))
    if (
        not path.is_file()
        or int(path.stat().st_size) != int(declaration["size_bytes"])
        or sha256_file(path) != str(declaration["sha256"])
    ):
        raise V21Trial08Error(f"common runtime input drifted: {label}")
    return path


def _absolute_plugin_loader_from_binary(binary: Path) -> Path:
    binary = binary.resolve()
    binary_name = binary.name
    if not binary_name.startswith("lib") or binary.suffix != ".dylib":
        raise V21Trial08Error("macOS plugin binary name cannot yield a safe loader")
    return binary.with_name(binary_name[len("lib") : -len(binary.suffix)])


def _preflight_common_environment(lock: Mapping[str, Any]) -> dict[str, Any]:
    """Catch API/profile/common-file errors before consuming the one-shot."""

    import opensim
    import sweep_binary_phase_detector_v21_geometry as v21
    import validate_binary_phase_fsm_v20_development as v20
    from binary_phase_detector import (
        BinaryPhaseDetectorSampler,
        load_binary_phase_detector_profile,
    )
    from binary_phase_fsm import BinaryPhaseFSM
    from kinematics_interpolator import KinematicsInterpolator  # noqa: F401
    from model_loader import _load_plugin

    declarations = lock["trial"]["declared_inputs_not_verified_or_opened_by_freeze"]
    model = _verify_common_input(declarations["model"], label="model")
    plugin = _verify_common_input(
        declarations["plugin_binary_macos"], label="plugin_binary_macos"
    )
    profile_path = _verify_source_record(
        lock["candidate"]["profile"], label="candidate.profile.preflight"
    )
    profile = load_binary_phase_detector_profile(profile_path)
    fsm_payload = BinaryPhaseFSM().payload()
    plugin_loader_absolute = _absolute_plugin_loader_from_binary(plugin)
    _load_plugin(str(plugin_loader_absolute))
    model_instance = opensim.Model(str(model))
    model_instance.initSystem()
    BinaryPhaseDetectorSampler(model_instance, profile)
    checks = {
        "binary_sampler_api": callable(getattr(BinaryPhaseDetectorSampler, "sample", None)),
        "profile_roles": [point.name for point in profile.points]
        == ["left_heel", "left_toe"],
        "profile_type": profile.detector_type == "binary_point_clearance_v1",
        "fsm_source": fsm_payload.get("source") == "binary_phase_fsm_v20",
        "fsm_contract": fsm_payload.get("event_contract_id")
        == lock["candidate"]["fsm_current_implementation_contract_id"],
        "fsm_sample_dt": fsm_payload.get("sample_dt_s") == freeze.SAMPLE_DT_S,
        "fsm_debounce": fsm_payload.get("debounce_s") == freeze.DEBOUNCE_S,
        "run_mode_api": callable(getattr(v20, "_run_mode", None)),
        "score_view_api": callable(getattr(v20, "_score_view", None)),
        "channel_gate_api": callable(getattr(v21, "two_sensor_view_gate", None)),
        "fast_fsm_api": callable(getattr(v21, "fast_fsm_events", None)),
        "plugin_loaded_from_verified_absolute_basename": True,
        "model_initialized": True,
        "detector_frames_resolved": True,
    }
    if not all(checks.values()):
        raise V21Trial08Error(f"common environment preflight failed: {checks}")
    return {
        "checks": checks,
        "environment": {
            "python": platform.python_version(),
            "numpy": np.__version__,
            "opensim_python": str(getattr(opensim, "__version__", "unknown")),
            "opensim_build": str(opensim.GetVersionAndDate()),
        },
        "model": _source_record(model),
        "plugin_binary_macos": _source_record(plugin),
        "profile": _source_record(profile_path),
        "plugin_loaded_in_current_process": True,
        "plugin_loader_repo_path": plugin_loader_absolute.relative_to(
            REPO_ROOT
        ).as_posix(),
        "trial08_semantic_access": False,
    }


def preflight_unopened() -> dict[str, Any]:
    system = platform.system()
    machine = platform.machine().lower()
    if system != "Darwin" or machine not in {"arm64", "aarch64"}:
        raise V21Trial08Error(
            "numerical trial-08 replay is currently attested only on macOS arm64"
        )
    lock, checks = _validate_freeze_lock()
    common_environment = _preflight_common_environment(lock)
    if os.path.lexists(freeze.EXECUTION_LEDGER_PATH):
        raise V21Trial08Error("global execution ledger already exists; stage consumed")
    if os.path.lexists(freeze.OUTPUT_DIR):
        raise V21Trial08Error("fixed output directory already exists; stage consumed")
    return {
        "status": "V21_TRIAL08_ONE_SHOT_PREFLIGHT_READY_UNOPENED",
        "candidate_id": freeze.CANDIDATE_ID,
        "trial_role": freeze.TRIAL_ROLE,
        "trial08_semantic_access": False,
        "platform": {
            "system": system,
            "machine": machine,
            "numerical_claim_scope": "macOS-arm64-only",
            "portable_scope": "validator contract and synthetic tests",
        },
        "freeze_lock": _source_record(freeze.FREEZE_PATH),
        "freeze_checks": checks,
        "common_environment": common_environment,
        "lock": lock,
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


def _atomic_write_exclusive(path: Path, writer: Callable[[TextIO], None]) -> Path:
    if not path.parent.is_dir():
        raise V21Trial08Error(f"output parent does not exist: {path.parent}")
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


def _write_json_exclusive(path: Path, payload: Mapping[str, Any]) -> Path:
    def writer(stream: TextIO) -> None:
        json.dump(dict(payload), stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")

    return _atomic_write_exclusive(path, writer)


def _artifact_record(path: Path) -> dict[str, Any]:
    return _source_record(path)


def _opening_payload(preflight: Mapping[str, Any]) -> dict[str, Any]:
    return {
        "schema_version": SCHEMA_VERSION,
        "run_id": RUN_ID,
        "status": "TRIAL08_SEMANTIC_ACCESS_STARTED_STAGE_CONSUMED",
        "opened_at_utc": _utc_now(),
        "process_id": os.getpid(),
        "trial_id": freeze.TRIAL_ID,
        "trial_role": freeze.TRIAL_ROLE,
        "scientifically_virgin": False,
        "candidate_id": freeze.CANDIDATE_ID,
        "freeze_lock": preflight["freeze_lock"],
        "platform": preflight["platform"],
        "common_environment": preflight["common_environment"],
        "fixed_output_dir": freeze.OUTPUT_DIR.relative_to(REPO_ROOT).as_posix(),
        "semantic_access_started": True,
        "stage_consumed": True,
        "rerun_allowed": False,
        "rescue_reselection_or_retuning_allowed": False,
        "pre_freeze_nonperformance_metadata_access": preflight["lock"]
        ["data_governance"]
        ["current_cycle_pre_freeze_nonperformance_metadata_access"],
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
        "prescribed_grf_authorized_for_read": False,
        "external_loads_authorized_for_read": False,
    }


def _open_stage(preflight: Mapping[str, Any]) -> dict[str, Any]:
    freeze.OUTPUT_DIR.parent.mkdir(parents=True, exist_ok=True)
    opening = _opening_payload(preflight)
    _write_json_exclusive(freeze.EXECUTION_LEDGER_PATH, opening)
    freeze.OUTPUT_DIR.mkdir(parents=False, exist_ok=False)
    _fsync_directory(freeze.OUTPUT_DIR.parent)
    _write_json_exclusive(freeze.OUTPUT_DIR / ACCESS_RECEIPT_NAME, opening)
    _audit_stage("opening_receipts_published")
    return opening


def _assert_semantic_access_opened() -> dict[str, Any]:
    """Require the canonical immutable receipts before any trial-08 decode."""

    ledger = _strict_json(freeze.EXECUTION_LEDGER_PATH)
    receipt = _strict_json(freeze.OUTPUT_DIR / ACCESS_RECEIPT_NAME)
    checks = {
        "receipts_exact": ledger == receipt,
        "status": ledger.get("status")
        == "TRIAL08_SEMANTIC_ACCESS_STARTED_STAGE_CONSUMED",
        "semantic_access": ledger.get("semantic_access_started") is True,
        "consumed": ledger.get("stage_consumed") is True,
        "no_retry": ledger.get("rerun_allowed") is False,
        "candidate": ledger.get("candidate_id") == freeze.CANDIDATE_ID,
        "trial": ledger.get("trial_id") == freeze.TRIAL_ID,
        "process_owner": ledger.get("process_id") == os.getpid(),
        "freeze_hash": ledger.get("freeze_lock", {}).get("sha256")
        == sha256_file(freeze.FREEZE_PATH),
    }
    if not all(checks.values()):
        raise V21Trial08Error(f"semantic-access receipt drifted: {checks}")
    return ledger


def _verify_declared_input(
    declaration: Mapping[str, Any], *, label: str
) -> Path:
    _assert_semantic_access_opened()
    path = _resolve_repo_path(str(declaration["path"]))
    if not path.is_file():
        raise V21Trial08Error(f"trial-08 input is missing: {label}")
    _audit_trial_input(label, verified=False)
    if int(path.stat().st_size) != int(declaration["size_bytes"]):
        raise V21Trial08Error(f"trial-08 input size drifted: {label}")
    if sha256_file(path) != str(declaration["sha256"]):
        raise V21Trial08Error(f"trial-08 input hash drifted: {label}")
    _audit_trial_input(label, verified=True)
    return path


def _validate_trial08_replay_inputs(
    lock: Mapping[str, Any]
) -> dict[str, Any]:
    _assert_semantic_access_opened()
    declarations = lock["trial"]["declared_inputs_not_verified_or_opened_by_freeze"]
    paths = {
        label: _verify_declared_input(declarations[label], label=label)
        for label in (
            "preprocessing_lock",
            "ik_motion",
            "model",
            "plugin_binary_macos",
        )
    }
    preprocessing = _strict_json(
        paths["preprocessing_lock"],
        expected_sha256=declarations["preprocessing_lock"]["sha256"],
    )
    checks = {
        "trial": preprocessing.get("trial_id") == freeze.TRIAL_ID,
        "stage": preprocessing.get("stage") == "development",
        "status": preprocessing.get("status")
        == "V14_1_PREPROCESSING_FROZEN_BEFORE_DETECTOR_REPLAY",
        "interval": preprocessing.get("analysis_interval_s")
        == list(freeze.TRIAL_INTERVAL_S),
        "absolute_time": preprocessing.get("absolute_timestamps_no_rezero") is True,
        "coverage": preprocessing.get("all_sources_cover_analysis_interval") is True,
        "no_adaptation": preprocessing.get(
            "adaptive_crop_resample_or_interpolation_used"
        )
        is False,
        "marker_ik_not_used": preprocessing.get("dataset_ik_used_downstream")
        is False,
        "ik_bound": preprocessing.get("preprocessed_files", {})
        .get("ik_motion", {})
        .get("sha256")
        == declarations["ik_motion"]["sha256"],
    }
    live = preprocessing.get("live_replay_inputs", {})
    model_record = live.get("model", {})
    plugin_record = live.get("plugin", {})
    checks["model_bound"] = (
        model_record.get("path") == declarations["model"]["path"]
        and model_record.get("sha256") == declarations["model"]["sha256"]
    )
    checks["plugin_bound"] = (
        plugin_record.get("binary_path")
        == declarations["plugin_binary_macos"]["path"]
        and plugin_record.get("binary_sha256")
        == declarations["plugin_binary_macos"]["sha256"]
        and plugin_record.get("loader_basename") == PLUGIN_LOADER
    )
    if not all(checks.values()):
        raise V21Trial08Error(f"trial-08 preprocessing binding failed: {checks}")
    plugin_loader_absolute = _absolute_plugin_loader_from_binary(
        paths["plugin_binary_macos"]
    )
    _audit_stage("trial08_replay_inputs_verified")
    return {
        "paths": paths,
        "plugin_loader_absolute": plugin_loader_absolute,
        "checks": checks,
        "source_records": {label: _source_record(path) for label, path in paths.items()},
    }


def _time_grid() -> np.ndarray:
    times = freeze.TRIAL_INTERVAL_S[0] + np.arange(
        freeze.EXPECTED_SAMPLE_COUNT, dtype=np.float64
    ) * freeze.SAMPLE_DT_S
    if (
        times.size != freeze.EXPECTED_SAMPLE_COUNT
        or abs(float(times[0]) - freeze.TRIAL_INTERVAL_S[0]) > 1e-12
        or abs(float(times[-1]) - freeze.TRIAL_INTERVAL_S[1]) > 1e-9
        or not np.all(np.isfinite(times))
        or not np.all(np.diff(times) > 0.0)
        or not np.allclose(np.diff(times), freeze.SAMPLE_DT_S, atol=1e-12, rtol=0.0)
    ):
        raise V21Trial08Error("trial-08 1 ms lattice drifted")
    return times


def _pack_binary_bits(heel: np.ndarray, toe: np.ndarray) -> dict[str, Any]:
    heel_bits = np.asarray(heel)
    toe_bits = np.asarray(toe)
    if (
        heel_bits.dtype != np.dtype(bool)
        or toe_bits.dtype != np.dtype(bool)
        or heel_bits.ndim != 1
        or toe_bits.shape != heel_bits.shape
        or heel_bits.size == 0
    ):
        raise V21Trial08Error("packed binary trace requires aligned bool vectors")
    packed = np.packbits(
        np.column_stack((heel_bits, toe_bits)).reshape(-1), bitorder="little"
    )
    raw = packed.tobytes()
    return {
        "packed_bits_base64": base64.b64encode(raw).decode("ascii"),
        "packed_bits_size_bytes": len(raw),
        "bit_trace_sha256": hashlib.sha256(raw).hexdigest(),
    }


def _acquire_trace(
    replay_inputs: Mapping[str, Any], profile_path: Path, *, plugin_preloaded: bool
) -> dict[str, Any]:
    _assert_semantic_access_opened()
    # Imports happen only after the permanent opening receipt exists.
    import opensim
    from binary_phase_detector import (
        BinaryPhaseDetectorSampler,
        load_binary_phase_detector_profile,
    )
    from config import SimulatorConfig
    from kinematics_interpolator import KinematicsInterpolator
    from model_loader import _load_plugin
    from sweep_binary_phase_detector_v21_geometry import SweepProgress

    if sha256_file(profile_path) != freeze.PROFILE_SHA256:
        raise V21Trial08Error("candidate profile drifted immediately before sampling")
    profile = load_binary_phase_detector_profile(profile_path)
    paths = replay_inputs["paths"]
    if not plugin_preloaded:
        _load_plugin(str(replay_inputs["plugin_loader_absolute"]))
    model = opensim.Model(str(paths["model"]))
    state = model.initSystem()
    sampler = BinaryPhaseDetectorSampler(model, profile)

    cfg = SimulatorConfig()
    cfg.model_bundle_dir = str(paths["model"].parent)
    cfg.model_file = str(paths["model"])
    cfg.kinematics_file = str(paths["ik_motion"])
    cfg.t_start, cfg.t_end = freeze.TRIAL_INTERVAL_S
    kinematics = KinematicsInterpolator(cfg)
    _audit_stage("trial08_ik_loaded_trace_sampling")
    times = _time_grid()
    heel = np.empty(times.size, dtype=bool)
    toe = np.empty(times.size, dtype=bool)
    clearance_min = {"left_heel": math.inf, "left_toe": math.inf}
    clearance_max = {"left_heel": -math.inf, "left_toe": -math.inf}
    coordinates = model.getCoordinateSet()
    progress = SweepProgress(
        total=int(times.size),
        label="V21 one-shot trial08",
        non_tty_interval_s=20.0,
    )
    try:
        for row, time_s in enumerate(times):
            q, _qdot, _qddot = kinematics.get(float(time_s))
            state.setTime(float(time_s))
            for index in range(coordinates.getSize()):
                coordinate = coordinates.get(index)
                name = coordinate.getName()
                if name in q:
                    coordinate.setValue(state, float(q[name]), False)
            reading = sampler.sample(state, float(time_s))
            heel_bit = reading.contacts["left_heel"]
            toe_bit = reading.contacts["left_toe"]
            if type(heel_bit) is not bool or type(toe_bit) is not bool:
                raise V21Trial08Error("binary sampler produced a non-native bool")
            heel[row] = heel_bit
            toe[row] = toe_bit
            for role in ("left_heel", "left_toe"):
                clearance = float(reading.signed_clearance_m[role])
                if not math.isfinite(clearance):
                    raise V21Trial08Error("binary sampler produced non-finite clearance")
                clearance_min[role] = min(clearance_min[role], clearance)
                clearance_max[role] = max(clearance_max[role], clearance)
            if row % 100 == 0 or row + 1 == times.size:
                progress.update(row + 1)
    except BaseException:
        progress.finish("FAILED")
        raise
    progress.finish("DONE")
    packed = _pack_binary_bits(heel, toe)
    _audit_stage("trial08_binary_trace_acquired")
    return {
        "time_s": times,
        "heel": heel,
        "toe": toe,
        "sample_count": int(times.size),
        **packed,
        "time_trace_sha256": hashlib.sha256(
            times.astype("<f8", copy=False).tobytes()
        ).hexdigest(),
        "baseline": {
            "time_s": float(times[0]),
            "left_heel_contact": bool(heel[0]),
            "left_toe_contact": bool(toe[0]),
            "event_attributed_at_reset": False,
        },
        "clearance_range_m": {
            role: {"minimum": clearance_min[role], "maximum": clearance_max[role]}
            for role in ("left_heel", "left_toe")
        },
    }


def _validate_oracle_views(
    core: Mapping[str, Any], views: Sequence[Mapping[str, Any]], *, minimum_cycles: int
) -> dict[str, Any]:
    required_keys = {
        "view_id",
        "interval_s",
        "speed_mps",
        "left_boundary",
        "right_boundary",
        "complete_cycles",
        "scoreable_events",
        "counts",
        "consumer_contract",
        "view_sha256",
    }
    event_ids = {
        str(event["event_id"])
        for event in core.get("events", [])
        if isinstance(event, Mapping) and "event_id" in event
    }
    grid_start = float(core["time_grid"]["start_s"])
    grid_end = float(core["time_grid"]["end_s"])
    dt = float(core["sample_dt_s"])
    prior_end = -math.inf
    seen_ids: set[str] = set()
    summaries: list[dict[str, Any]] = []
    for view in views:
        if not isinstance(view, Mapping) or set(view) != required_keys:
            raise V21Trial08Error("canonical oracle view schema drifted")
        view_id = str(view["view_id"])
        if not view_id or view_id in seen_ids:
            raise V21Trial08Error("canonical oracle view ids must be unique")
        seen_ids.add(view_id)
        hash_payload = dict(view)
        observed_view_sha = str(hash_payload.pop("view_sha256"))
        if canonical_sha256(hash_payload) != observed_view_sha:
            raise V21Trial08Error(f"canonical oracle view hash drifted: {view_id}")
        interval = view["interval_s"]
        if not isinstance(interval, list) or len(interval) != 2:
            raise V21Trial08Error(f"canonical oracle interval drifted: {view_id}")
        start, end = (float(value) for value in interval)
        speed = float(view["speed_mps"])
        if not all(math.isfinite(value) for value in (start, end, speed)):
            raise V21Trial08Error(f"canonical oracle view is non-finite: {view_id}")
        if not (grid_start <= start < end <= grid_end) or start < prior_end - 1e-12:
            raise V21Trial08Error(f"canonical oracle view ordering drifted: {view_id}")
        prior_end = end
        for value in (start, end):
            lattice = (value - grid_start) / dt
            if abs(lattice - round(lattice)) > 1e-9:
                raise V21Trial08Error(f"canonical oracle view is off-grid: {view_id}")
        for label, expected_side, expected_time in (
            ("left_boundary", "left", start),
            ("right_boundary", "right", end),
        ):
            boundary = view[label]
            if (
                not isinstance(boundary, Mapping)
                or set(boundary)
                != {
                    "boundary",
                    "time_s",
                    "contact_active",
                    "active_contact_ids",
                    "event_ids_at_boundary",
                }
                or boundary.get("boundary") != expected_side
                or abs(float(boundary.get("time_s", math.nan)) - expected_time)
                > 1e-12
                or type(boundary.get("contact_active")) is not bool
                or not isinstance(boundary.get("active_contact_ids"), list)
                or not isinstance(boundary.get("event_ids_at_boundary"), list)
            ):
                raise V21Trial08Error(
                    f"canonical oracle boundary drifted: {view_id}.{label}"
                )
        cycles = view["complete_cycles"]
        events = view["scoreable_events"]
        counts = view["counts"]
        if (
            not isinstance(cycles, list)
            or not isinstance(events, list)
            or not isinstance(counts, Mapping)
        ):
            raise V21Trial08Error(f"canonical oracle view payload drifted: {view_id}")
        scoreable_ids = [
            str(event.get("event_id", ""))
            for event in events
            if isinstance(event, Mapping)
        ]
        event_times = [
            float(event.get("event_time_s", math.nan))
            for event in events
            if isinstance(event, Mapping)
        ]
        exact_counts = {
            "complete_cycles": len(cycles),
            "heel_strike": sum(
                isinstance(event, Mapping) and event.get("event") == "heel_strike"
                for event in events
            ),
            "toe_off": sum(
                isinstance(event, Mapping) and event.get("event") == "toe_off"
                for event in events
            ),
        }
        if (
            len(scoreable_ids) != len(events)
            or len(set(scoreable_ids)) != len(scoreable_ids)
            or not set(scoreable_ids).issubset(event_ids)
            or len(event_times) != len(events)
            or not all(math.isfinite(value) for value in event_times)
            or any(
                event_times[index] > event_times[index + 1] + 1e-12
                for index in range(len(event_times) - 1)
            )
            or dict(counts) != exact_counts
            or len(cycles) < minimum_cycles
            or view["consumer_contract"]
            != (
                "reuse scoreable_events verbatim at both cadences; never "
                "threshold prescribed GRF in a consumer"
            )
        ):
            raise V21Trial08Error(f"canonical oracle scoreable view drifted: {view_id}")
        summaries.append(
            {
                "view_id": view_id,
                "view_sha256": observed_view_sha,
                "interval_s": [start, end],
                "complete_cycle_count": len(cycles),
                "scoreable_event_count": len(events),
            }
        )
    return {"pass": True, "views": summaries}


def _load_oracle(lock: Mapping[str, Any]) -> dict[str, Any]:
    _assert_semantic_access_opened()
    import build_canonical_grf_event_oracle as canonical_oracle

    declaration = lock["trial"]["declared_inputs_not_verified_or_opened_by_freeze"][
        "canonical_oracle"
    ]
    path = _verify_declared_input(declaration, label="canonical_oracle")
    ledger = _strict_json(path, expected_sha256=declaration["sha256"])
    canonical_oracle.validate_ledger(ledger)
    core = ledger.get("scientific_core")
    if not isinstance(core, Mapping):
        raise V21Trial08Error("canonical oracle scientific core is missing")
    views = core.get("views")
    gate = lock["gate"]
    view_integrity = (
        _validate_oracle_views(
            core,
            views,
            minimum_cycles=int(gate["minimum_complete_cycles_per_view"]),
        )
        if isinstance(views, list)
        else {"pass": False, "views": []}
    )
    checks = {
        "trial": core.get("trial_id") == freeze.TRIAL_ID,
        "contract": core.get("event_contract_id")
        == "primary_grf_split_v1+two_sensor_highrate_v1",
        "sample_dt": float(core.get("sample_dt_s", math.nan))
        == freeze.SAMPLE_DT_S,
        "threshold_n": float(core.get("threshold_n", math.nan)) == 20.0,
        "min_contact_duration_s": float(
            core.get("min_contact_duration_s", math.nan)
        )
        == 0.050,
        "min_cycle_duration_s": float(
            core.get("min_cycle_duration_s", math.nan)
        )
        == 0.30,
        "grid_start": abs(
            float(core.get("time_grid", {}).get("start_s", math.nan))
            - freeze.TRIAL_INTERVAL_S[0]
        )
        <= 1e-12,
        "grid_end": abs(
            float(core.get("time_grid", {}).get("end_s", math.nan))
            - freeze.TRIAL_INTERVAL_S[1]
        )
        <= 1e-9,
        "grid_count": core.get("time_grid", {}).get("sample_count")
        == freeze.EXPECTED_SAMPLE_COUNT,
        "view_count": isinstance(views, list)
        and len(views) == int(gate["required_view_count"]),
        "minimum_cycles": isinstance(views, list)
        and all(
            int(view.get("counts", {}).get("complete_cycles", -1))
            >= int(gate["minimum_complete_cycles_per_view"])
            for view in views
        ),
        "view_integrity": view_integrity["pass"],
    }
    if not all(checks.values()):
        raise V21Trial08Error(f"canonical oracle gate drifted: {checks}")
    _audit_stage("trial08_canonical_oracle_loaded")
    return {
        "ledger": ledger,
        "checks": checks,
        "view_integrity": view_integrity,
        "source": _source_record(path),
    }


def _full_event_contract(events: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    finite = True
    confirmation = True
    delivery = True
    for event in events:
        values = [
            float(event.get("event_time_s", math.nan)),
            float(event.get("confirmed_time_s", math.nan)),
            float(event.get("delivered_time_s", math.nan)),
        ]
        finite = finite and all(math.isfinite(value) for value in values)
        confirmation = confirmation and abs(
            values[1] - values[0] - freeze.DEBOUNCE_S
        ) <= 1e-9
        delivery = delivery and -1e-12 <= values[2] - values[1] <= 0.010 + 1e-12
    return {
        "event_count": len(events),
        "all_event_times_finite": finite,
        "confirmation_delay_exact": confirmation,
        "delivery_delay_bounded": delivery,
        "pass": bool(events and finite and confirmation and delivery),
    }


def _evaluate(
    lock: Mapping[str, Any], trace: Mapping[str, Any], oracle: Mapping[str, Any]
) -> dict[str, Any]:
    import sweep_binary_phase_detector_v21_geometry as v21
    import validate_binary_phase_fsm_v20_development as v20

    views = oracle["ledger"]["scientific_core"]["views"]
    channel_gate = v21.two_sensor_view_gate(
        trace["time_s"], trace["heel"], trace["toe"], views
    )
    mode_results = {mode: v20._run_mode(trace, mode) for mode in MODES}
    sequential = mode_results[MODES[0]]
    batched = mode_results[MODES[1]]
    expected_fsm_contract = lock["candidate"][
        "fsm_current_implementation_contract_id"
    ]
    fsm_contract_by_mode = {
        mode: {
            "source": mode_results[mode]["final_payload"].get("source")
            == "binary_phase_fsm_v20",
            "event_contract_id": mode_results[mode]["final_payload"].get(
                "event_contract_id"
            )
            == expected_fsm_contract,
            "sample_dt_s": mode_results[mode]["final_payload"].get("sample_dt_s")
            == freeze.SAMPLE_DT_S,
            "debounce_s": mode_results[mode]["final_payload"].get("debounce_s")
            == freeze.DEBOUNCE_S,
            "max_delivery_delay_s": mode_results[mode]["final_payload"].get(
                "max_delivery_delay_s"
            )
            == 0.010,
        }
        for mode in MODES
    }
    for record in fsm_contract_by_mode.values():
        record["pass"] = all(record.values())
    fsm_contract = {
        "current_implementation_contract_id": expected_fsm_contract,
        "target_bundle_contract_id_after_h0_integration": lock["candidate"][
            "target_bundle_contract_id_after_h0_integration"
        ],
        "target_contract_is_not_yet_runtime_implemented": True,
        "modes": fsm_contract_by_mode,
        "pass": all(record["pass"] for record in fsm_contract_by_mode.values()),
    }
    terminal_state_by_mode = {
        mode: {
            "pending_event_clear": mode_results[mode]["final_payload"].get(
                "pending_event"
            )
            is None,
            "pending_contact_state_clear": mode_results[mode]["final_payload"].get(
                "pending_contact_state"
            )
            is None,
        }
        for mode in MODES
    }
    for record in terminal_state_by_mode.values():
        record["pass"] = all(record.values())
    terminal_state = {
        "modes": terminal_state_by_mode,
        "pass": all(record["pass"] for record in terminal_state_by_mode.values()),
    }
    fast = v21.fast_fsm_events(trace["time_s"], trace["heel"], trace["toe"])
    parity = {
        "trial_id": freeze.TRIAL_ID,
        "events_exact": sequential["events"] == batched["events"],
        "contact_state_transitions_exact": sequential[
            "contact_state_transitions"
        ]
        == batched["contact_state_transitions"],
        "candidate_cancellations_exact": sequential["candidate_cancellations"]
        == batched["candidate_cancellations"],
        "boundary_snapshots_sha256_exact": sequential[
            "boundary_snapshots_sha256"
        ]
        == batched["boundary_snapshots_sha256"],
        "final_payload_exact": sequential["final_payload"]
        == batched["final_payload"],
        "fast_event_signature_exact": v21.event_signature(fast)
        == v21.event_signature(sequential["events"]),
        "sequential_boundary_snapshots_sha256": sequential[
            "boundary_snapshots_sha256"
        ],
        "batched_boundary_snapshots_sha256": batched[
            "boundary_snapshots_sha256"
        ],
    }
    parity["pass"] = bool(
        all(
            value
            for key, value in parity.items()
            if key
            not in {
                "trial_id",
                "sequential_boundary_snapshots_sha256",
                "batched_boundary_snapshots_sha256",
            }
        )
    )
    full_event = {
        mode: _full_event_contract(mode_results[mode]["events"]) for mode in MODES
    }
    units: list[dict[str, Any]] = []
    for mode in MODES:
        for view in views:
            units.append(
                v20._score_view(
                    trial_id=freeze.TRIAL_ID,
                    mode=mode,
                    trace=trace,
                    events=mode_results[mode]["events"],
                    view=view,
                    parity_pass=bool(parity["pass"]),
                )
            )
    gate = lock["gate"]
    all_pass = bool(
        len(units) == int(gate["required_unit_count"])
        and all(unit["pass"] for unit in units)
        and channel_gate["pass"]
        and parity["pass"]
        and fsm_contract["pass"]
        and terminal_state["pass"]
        and all(item["pass"] for item in full_event.values())
    )
    result = {
        "pass": all_pass,
        "unit_count": len(units),
        "unit_pass_count": sum(bool(unit["pass"]) for unit in units),
        "two_sensor_channel_gate": channel_gate,
        "parity": parity,
        "fsm_contract_binding": fsm_contract,
        "terminal_pending_state_gate": terminal_state,
        "full_event_contract": full_event,
        "units": units,
        "events": {mode: mode_results[mode]["events"] for mode in MODES},
        "diagnostics": {
            mode: {
                "event_count": len(mode_results[mode]["events"]),
                "contact_state_transition_count": len(
                    mode_results[mode]["contact_state_transitions"]
                ),
                "candidate_cancellation_count": len(
                    mode_results[mode]["candidate_cancellations"]
                ),
                "final_payload": mode_results[mode]["final_payload"],
            }
            for mode in MODES
        },
    }
    json.dumps(result, allow_nan=False)
    _audit_stage("trial08_gate_evaluation_complete")
    return result


def _write_result_artifacts(
    *,
    opening: Mapping[str, Any],
    preflight: Mapping[str, Any],
    replay_inputs: Mapping[str, Any],
    trace: Mapping[str, Any],
    oracle: Mapping[str, Any],
    evaluation: Mapping[str, Any],
) -> dict[str, Any]:
    trace_manifest = {
        "schema_version": SCHEMA_VERSION,
        "run_id": RUN_ID,
        "trial_id": freeze.TRIAL_ID,
        "sample_count": trace["sample_count"],
        "start_time_s": float(trace["time_s"][0]),
        "end_time_s": float(trace["time_s"][-1]),
        "sample_dt_s": freeze.SAMPLE_DT_S,
        "bit_trace_sha256": trace["bit_trace_sha256"],
        "time_trace_sha256": trace["time_trace_sha256"],
        "baseline": trace["baseline"],
        "clearance_range_m": trace["clearance_range_m"],
        "raw_trace_persisted": False,
        "raw_binary_bits_persisted": True,
        "prescribed_grf_read": False,
        "external_loads_read": False,
        "sources": replay_inputs["source_records"],
    }
    payloads = {
        "trace_manifest.json": trace_manifest,
        "packed_binary_trace.json": {
            "schema_version": SCHEMA_VERSION,
            "trial_id": freeze.TRIAL_ID,
            "encoding": "base64_of_numpy_packbits",
            "bitorder": "little",
            "channel_order_per_sample": ["left_heel", "left_toe"],
            "sample_count": trace["sample_count"],
            "packed_size_bytes": trace["packed_bits_size_bytes"],
            "packed_sha256": trace["bit_trace_sha256"],
            "time_grid": {
                "start_s": float(trace["time_s"][0]),
                "end_s": float(trace["time_s"][-1]),
                "sample_dt_s": freeze.SAMPLE_DT_S,
                "sample_count": trace["sample_count"],
                "time_trace_sha256": trace["time_trace_sha256"],
            },
            "packed_base64": trace["packed_bits_base64"],
        },
        "oracle_binding.json": {
            "schema_version": SCHEMA_VERSION,
            "trial_id": freeze.TRIAL_ID,
            "checks": oracle["checks"],
            "view_integrity": oracle["view_integrity"],
            "source": oracle["source"],
            "oracle_reconstructed_by_consumer": False,
        },
        "two_sensor_channel_gate.json": {
            "schema_version": SCHEMA_VERSION,
            **evaluation["two_sensor_channel_gate"],
        },
        "scalar_batch_parity.json": {
            "schema_version": SCHEMA_VERSION,
            **evaluation["parity"],
            "full_event_contract": evaluation["full_event_contract"],
            "fsm_contract_binding": evaluation["fsm_contract_binding"],
            "terminal_pending_state_gate": evaluation[
                "terminal_pending_state_gate"
            ],
        },
        "unit_metrics.json": {
            "schema_version": SCHEMA_VERSION,
            "unit_count": evaluation["unit_count"],
            "unit_pass_count": evaluation["unit_pass_count"],
            "units": evaluation["units"],
        },
        "event_journal.json": {
            "schema_version": SCHEMA_VERSION,
            "events": evaluation["events"],
            "diagnostics": evaluation["diagnostics"],
        },
    }
    records: dict[str, Any] = {}
    for name, payload in payloads.items():
        path = _write_json_exclusive(freeze.OUTPUT_DIR / name, payload)
        records[name] = _artifact_record(path)

    passed = bool(evaluation["pass"])
    terminal_status = (
        "PASS_INTERNAL_V21_TRIAL08_CONSUMED_DEVELOPMENT_CANDIDATE_"
        "READY_FOR_H0_INTEGRATION"
        if passed
        else "FAIL_INTERNAL_V21_TRIAL08_CONSUMED_TERMINAL"
    )
    next_stage = (
        "FREEZE_DEVELOPMENT_CANDIDATE_AND_IMPLEMENT_V21_H0_ROUTING"
        if passed
        else "STOP_NO_RETRY_NO_RESELECT_NO_RETUNE"
    )
    manifest = {
        "schema_version": SCHEMA_VERSION,
        "run_id": RUN_ID,
        "status": "V21_TRIAL08_EVIDENCE_COMPLETE_AWAITING_TERMINAL_DECISION",
        "expected_terminal_status": terminal_status,
        "pass": passed,
        "candidate_id": freeze.CANDIDATE_ID,
        "trial_id": freeze.TRIAL_ID,
        "trial_role": freeze.TRIAL_ROLE,
        "scientifically_virgin": False,
        "claim_limit": preflight["lock"]["trial"]["claim_limit"],
        "opening_receipt": _artifact_record(
            freeze.OUTPUT_DIR / ACCESS_RECEIPT_NAME
        ),
        "global_execution_ledger": _artifact_record(
            freeze.EXECUTION_LEDGER_PATH
        ),
        "freeze_lock": preflight["freeze_lock"],
        "platform": preflight["platform"],
        "common_environment": preflight["common_environment"],
        "artifacts": records,
        "data_access": {
            "trial08_opened": True,
            "trial08_inputs_attempted": list(
                _EXECUTION_AUDIT["trial08_inputs_attempted"]
            ),
            "trial08_inputs_verified": list(
                _EXECUTION_AUDIT["trial08_inputs_verified"]
            ),
            "prescribed_grf_read": False,
            "external_loads_read": False,
            "protected_trials_opened": [],
            "reserve_trials_opened": [],
        },
        "primary_grf_modified": False,
        "cpp_or_contact_modified": False,
        "sea_semantics_modified": False,
        "geometry_or_fsm_retuned_after_opening": False,
        "runtime_promoted": False,
        "training_promoted": False,
        "positive_morphology_reward_ppo_run": False,
        "execution_audit": json.loads(
            json.dumps(_EXECUTION_AUDIT, sort_keys=True, allow_nan=False)
        ),
        "next_stage": next_stage,
    }
    manifest_path = _write_json_exclusive(
        freeze.OUTPUT_DIR / MANIFEST_NAME, manifest
    )
    manifest_record = _artifact_record(manifest_path)
    _audit_stage("evidence_manifest_published")
    decision = {
        "schema_version": SCHEMA_VERSION,
        "run_id": RUN_ID,
        "status": terminal_status,
        "pass": passed,
        "decided_at_utc": _utc_now(),
        "stage_consumed": True,
        "rerun_allowed": False,
        "candidate_id": freeze.CANDIDATE_ID,
        "trial_id": freeze.TRIAL_ID,
        "trial_role": freeze.TRIAL_ROLE,
        "scientifically_virgin": False,
        "independent_validation_claim": False,
        "unit_count": evaluation["unit_count"],
        "unit_pass_count": evaluation["unit_pass_count"],
        "channel_gate_pass": evaluation["two_sensor_channel_gate"]["pass"],
        "parity_pass": evaluation["parity"]["pass"],
        "fsm_contract_pass": evaluation["fsm_contract_binding"]["pass"],
        "terminal_pending_state_pass": evaluation[
            "terminal_pending_state_gate"
        ]["pass"],
        "post_pass_scope": preflight["lock"]["post_pass_scope"],
        "evidence_manifest": manifest_record,
        "next_stage": next_stage,
    }
    decision_path = _write_json_exclusive(
        freeze.OUTPUT_DIR / DECISION_NAME, decision
    )
    _audit_stage("terminal_decision_published")
    return {
        "decision": decision,
        "manifest": manifest_record,
        "terminal_decision": _artifact_record(decision_path),
        "opening": dict(opening),
    }


def execute_one_shot() -> dict[str, Any]:
    _EXECUTION_AUDIT.clear()
    _EXECUTION_AUDIT.update(
        {
            "stage": "pre_open",
            "trial08_inputs_attempted": [],
            "trial08_inputs_verified": [],
        }
    )
    preflight = preflight_unopened()
    opening = _open_stage(preflight)
    lock = preflight["lock"]
    replay_inputs = _validate_trial08_replay_inputs(lock)
    profile_path = _verify_source_record(
        lock["candidate"]["profile"], label="candidate.profile.post_open"
    )
    oracle = _load_oracle(lock)
    trace = _acquire_trace(
        replay_inputs,
        profile_path,
        plugin_preloaded=bool(
            preflight["common_environment"]["plugin_loaded_in_current_process"]
        ),
    )
    evaluation = _evaluate(lock, trace, oracle)
    return _write_result_artifacts(
        opening=opening,
        preflight=preflight,
        replay_inputs=replay_inputs,
        trace=trace,
        oracle=oracle,
        evaluation=evaluation,
    )


def _write_failure_after_open(exc: BaseException) -> None:
    if not os.path.lexists(freeze.EXECUTION_LEDGER_PATH):
        return
    if not freeze.OUTPUT_DIR.is_dir():
        return
    if os.path.lexists(freeze.OUTPUT_DIR / DECISION_NAME):
        return
    path = freeze.OUTPUT_DIR / FAILURE_NAME
    if os.path.lexists(path):
        return
    try:
        ledger = _strict_json(freeze.EXECUTION_LEDGER_PATH)
    except BaseException:
        return
    if (
        ledger.get("process_id") != os.getpid()
        or ledger.get("candidate_id") != freeze.CANDIDATE_ID
        or ledger.get("trial_id") != freeze.TRIAL_ID
    ):
        return
    receipt_path = freeze.OUTPUT_DIR / ACCESS_RECEIPT_NAME
    receipt_published = receipt_path.is_file()
    if receipt_published:
        try:
            receipt = _strict_json(receipt_path)
        except BaseException:
            return
        if receipt != ledger:
            return
    payload = {
        "schema_version": SCHEMA_VERSION,
        "run_id": RUN_ID,
        "status": "ERROR_INTERNAL_V21_TRIAL08_CONSUMED",
        "failed_at_utc": _utc_now(),
        "stage_consumed": True,
        "rerun_allowed": False,
        "candidate_id": freeze.CANDIDATE_ID,
        "trial_id": freeze.TRIAL_ID,
        "trial_role": freeze.TRIAL_ROLE,
        "scientifically_virgin": False,
        "exception_type": type(exc).__name__,
        "exception": str(exc),
        "traceback": "".join(traceback.format_exception(exc)),
        "execution_audit": json.loads(
            json.dumps(_EXECUTION_AUDIT, sort_keys=True, allow_nan=False)
        ),
        "access_receipt_published": receipt_published,
        "evidence_manifest_published": os.path.lexists(
            freeze.OUTPUT_DIR / MANIFEST_NAME
        ),
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
        "next_stage": "STOP_NO_RETRY_NO_RESELECT_NO_RETUNE",
    }
    try:
        _write_json_exclusive(path, payload)
    except BaseException:
        pass


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_mutually_exclusive_group(required=True)
    modes.add_argument(
        "--check",
        action="store_true",
        help="preflight freeze and destinations without opening trial 08",
    )
    modes.add_argument(
        "--execute-one-shot",
        action="store_true",
        help="permanently consume and evaluate the fixed trial-08 stage",
    )
    args = parser.parse_args(argv)
    if args.check:
        try:
            result = preflight_unopened()
        except Exception as exc:
            print(
                f"V21 trial-08 preflight failed closed: {type(exc).__name__}: {exc}",
                file=sys.stderr,
            )
            return 2
        public = {key: value for key, value in result.items() if key != "lock"}
        print(json.dumps(public, indent=2, sort_keys=True, allow_nan=False))
        return 0

    opened_before_failure = os.path.lexists(freeze.EXECUTION_LEDGER_PATH)
    try:
        result = execute_one_shot()
    except BaseException as exc:
        opened_after_failure = os.path.lexists(freeze.EXECUTION_LEDGER_PATH)
        _write_failure_after_open(exc)
        if opened_before_failure or opened_after_failure:
            message = "V21 trial-08 one-shot failed closed and is consumed"
        else:
            message = "V21 trial-08 preflight failed before opening; stage not consumed"
        print(f"{message}: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 2
    print(json.dumps(result, indent=2, sort_keys=True, allow_nan=False))
    return 0 if result["decision"]["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
