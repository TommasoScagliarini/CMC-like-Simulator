"""Execute the frozen V22 detector replay on already-open trial 08.

This is a new, explicitly authorized development cycle.  It preserves the
terminal V21 one-shot ERROR and changes only the invalid oracle/trace coverage
precondition: the detector trace must be an exact subgrid that covers every
scoreable oracle view and event; the two global grids need not be equal.

``--check`` is read-only.  ``--execute-development`` publishes a new global
ledger and a byte-identical access receipt before detector replay, evaluates
the unchanged V21 geometry and V20 FSM with the unchanged numerical gates, and
then publishes evidence before the terminal decision.  PASS is development
evidence only; it cannot promote H0, runtime, training, protected data, or PPO.
"""

from __future__ import annotations

import argparse
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

import freeze_binary_phase_detector_v22_trial08_development as freeze  # noqa: E402
import validate_binary_phase_detector_v21_trial08_one_shot as v21_gate  # noqa: E402


SCHEMA_VERSION = 22
RUN_ID = "AB06_BINARY_POINT_V22_TRIAL08_OPEN_DEVELOPMENT_COVERAGE_FIX"
MODES = ("sequential_1ms", "batched_10ms_same_samples")
ACCESS_RECEIPT_NAME = "trial08_development_access_receipt.json"
FAILURE_NAME = "failure.json"
DECISION_NAME = "trial08_development_decision_lock.json"
MANIFEST_NAME = "manifest.json"
PLUGIN_LOADER = "plugins/SEA_Plugin_BlackBox_mCMC_impedence_ff"
CLAIM_LIMIT = (
    "open development replay on already-open trial 08 after the terminal V21 "
    "procedural error; not independent validation and not H0, runtime, "
    "training, protected-trial, corridor, or PPO promotion"
)

_EXECUTION_AUDIT: dict[str, Any] = {
    "stage": "pre_open",
    "development_inputs_attempted": [],
    "development_inputs_verified": [],
}


class V22DevelopmentError(RuntimeError):
    """Raised when the frozen V22 development contract cannot be preserved."""


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def _audit_stage(stage: str) -> None:
    _EXECUTION_AUDIT["stage"] = str(stage)


def _audit_input(label: str, *, verified: bool) -> None:
    key = (
        "development_inputs_verified"
        if verified
        else "development_inputs_attempted"
    )
    values = _EXECUTION_AUDIT[key]
    if label not in values:
        values.append(str(label))


def _reject_json_constant(value: str) -> None:
    raise V22DevelopmentError(f"non-finite JSON constant is forbidden: {value}")


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _strict_json(
    path: Path, *, expected_sha256: str | None = None
) -> dict[str, Any]:
    if not path.is_file():
        raise V22DevelopmentError(f"required JSON is missing: {path}")
    observed_sha = sha256_file(path)
    if expected_sha256 is not None and observed_sha != expected_sha256:
        raise V22DevelopmentError(
            f"hash-pinned JSON drifted: {path}: "
            f"{observed_sha} != {expected_sha256}"
        )
    try:
        value = json.loads(
            path.read_text(encoding="utf-8"),
            parse_constant=_reject_json_constant,
        )
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise V22DevelopmentError(f"cannot read strict JSON: {path}") from exc
    if not isinstance(value, dict):
        raise V22DevelopmentError(f"JSON root must be an object: {path}")
    return value


def _resolve_repo_path(relative: str) -> Path:
    value = Path(relative)
    if value.is_absolute() or ".." in value.parts:
        raise V22DevelopmentError(f"non-portable repository path: {relative!r}")
    resolved = (REPO_ROOT / value).resolve()
    try:
        resolved.relative_to(REPO_ROOT.resolve())
    except ValueError as exc:
        raise V22DevelopmentError(f"path escapes repository: {relative!r}") from exc
    return resolved


def _source_record(path: Path) -> dict[str, Any]:
    resolved = path.resolve()
    if not resolved.is_file():
        raise V22DevelopmentError(f"source is missing: {resolved}")
    return {
        "path": resolved.relative_to(REPO_ROOT.resolve()).as_posix(),
        "sha256": sha256_file(resolved),
        "size_bytes": int(resolved.stat().st_size),
    }


def _verify_record(record: Mapping[str, Any], *, label: str) -> Path:
    if set(record) != {"path", "sha256", "size_bytes"}:
        raise V22DevelopmentError(f"malformed source record: {label}")
    path = _resolve_repo_path(str(record["path"]))
    if not path.is_file():
        raise V22DevelopmentError(f"missing source: {label}")
    if int(path.stat().st_size) != int(record["size_bytes"]):
        raise V22DevelopmentError(f"source size drifted: {label}")
    if sha256_file(path) != str(record["sha256"]):
        raise V22DevelopmentError(f"source hash drifted: {label}")
    return path


def _validate_freeze_lock() -> tuple[dict[str, Any], dict[str, Any]]:
    if not freeze.FREEZE_PATH.is_file():
        raise V22DevelopmentError("V22 development freeze lock is missing")
    lock = _strict_json(freeze.FREEZE_PATH)
    expected = freeze.build_freeze_payload(check_destinations=False)
    checks = {
        "exact_frozen_payload": lock == expected,
        "schema": lock.get("schema_version") == freeze.SCHEMA_VERSION,
        "protocol": lock.get("protocol_id") == freeze.PROTOCOL_ID,
        "status": lock.get("status")
        == "V22_OPEN_DEVELOPMENT_FROZEN_BEFORE_DETECTOR_REPLAY",
        "authorization": lock.get("authorization", {}).get("kind")
        == "explicit_user_authorized_new_development_cycle_after_v21_error",
        "old_error_not_reclassified": lock.get("authorization", {}).get(
            "previous_error_reclassified"
        )
        is False,
        "candidate": lock.get("candidate", {}).get("candidate_id")
        == freeze.CANDIDATE_ID,
        "single_candidate": lock.get("candidate", {}).get("candidate_count") == 1,
        "trial": lock.get("trial", {}).get("trial_id") == freeze.TRIAL_ID,
        "role": lock.get("trial", {}).get("role") == freeze.TRIAL_ROLE,
        "already_open": lock.get("trial", {}).get("already_open_before_v22")
        is True,
        "not_independent": lock.get("trial", {}).get("independent_validation")
        is False,
        "trace_interval": lock.get("trial", {}).get("trace_interval_s")
        == [freeze.TRACE_START_S, freeze.TRACE_END_S],
        "trace_count": lock.get("trial", {}).get("trace_sample_count")
        == freeze.TRACE_SAMPLE_COUNT,
        "gate_unchanged": lock.get("gate") == freeze.v21_freeze.FROZEN_GATE,
        "coverage_pass": lock.get("oracle", {}).get("coverage", {}).get("pass")
        is True,
        "global_grid_equality_not_required": lock.get("correction", {}).get(
            "global_grid_equality_required"
        )
        is False,
        "geometry_unchanged": lock.get("correction", {}).get("geometry_changed")
        is False,
        "fsm_unchanged": lock.get("correction", {}).get("fsm_changed") is False,
        "scorer_gate_unchanged": lock.get("correction", {}).get(
            "scorer_or_numerical_gate_changed"
        )
        is False,
        "no_rethreshold": lock.get("correction", {}).get(
            "oracle_reconstructed_or_rethresholded"
        )
        is False,
        "fixed_ledger": lock.get("opening_contract", {}).get(
            "global_execution_ledger"
        )
        == freeze.EXECUTION_LEDGER_PATH.relative_to(REPO_ROOT).as_posix(),
        "fixed_output": lock.get("opening_contract", {}).get("output_dir")
        == freeze.OUTPUT_DIR.relative_to(REPO_ROOT).as_posix(),
        "receipt_before_replay": lock.get("opening_contract", {}).get(
            "ledger_and_receipt_before_detector_replay"
        )
        is True,
        "single_execution": lock.get("opening_contract", {}).get(
            "single_execution_in_this_development_cycle"
        )
        is True,
        "no_retry": lock.get("opening_contract", {}).get(
            "retry_after_terminal_result_allowed"
        )
        is False,
        "post_scope_exact": lock.get("post_pass_scope") == freeze.POST_PASS_SCOPE,
        "no_h0_execution": lock.get("post_pass_scope", {}).get(
            "h0_execution_allowed"
        )
        is False,
        "no_protected_access": lock.get("post_pass_scope", {}).get(
            "protected_trial_access_allowed"
        )
        is False,
        "no_runtime_promotion": lock.get("post_pass_scope", {}).get(
            "runtime_promotion_allowed"
        )
        is False,
        "no_training_promotion": lock.get("post_pass_scope", {}).get(
            "training_promotion_allowed"
        )
        is False,
        "old_failure_status": lock.get("previous_v21_terminal_error", {})
        .get("checks", {})
        .get("failure_status")
        is True,
        "protected_closed": lock.get("data_governance", {}).get(
            "protected_trials_remaining_closed"
        )
        == ["05", "06"],
        "reserve_closed": lock.get("data_governance", {}).get(
            "reserve_trials_remaining_closed"
        )
        == ["03", "07"],
    }
    if not all(checks.values()):
        raise V22DevelopmentError(f"V22 freeze contract drifted: {checks}")
    for label, record in lock["sources"].items():
        if not isinstance(record, Mapping):
            raise V22DevelopmentError(f"invalid freeze source record: {label}")
        _verify_record(record, label=f"freeze.sources.{label}")
    _verify_record(lock["candidate"]["profile"], label="candidate.profile")
    _verify_record(lock["candidate"]["fsm"], label="candidate.fsm")
    _verify_record(lock["oracle"]["source"], label="canonical_oracle")
    for label, expected_record in lock["previous_v21_terminal_error"][
        "artifacts"
    ].items():
        _verify_record(expected_record, label=f"previous_v21_terminal.{label}")
    return lock, checks


def _absolute_plugin_loader_from_binary(binary: Path) -> Path:
    binary = binary.resolve()
    if not binary.name.startswith("lib") or binary.suffix != ".dylib":
        raise V22DevelopmentError(
            "macOS plugin binary name cannot yield a safe absolute loader"
        )
    return binary.with_name(binary.name[len("lib") : -len(binary.suffix)])


def _preflight_common_environment(lock: Mapping[str, Any]) -> dict[str, Any]:
    """Catch common API/model/profile failures before consuming V22."""

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

    declarations = lock["trial"]["declared_inputs"]
    model = _verify_record(declarations["model"], label="model.preflight")
    plugin = _verify_record(
        declarations["plugin_binary_macos"], label="plugin_binary_macos.preflight"
    )
    profile_path = _verify_record(
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
        "fsm_debounce": fsm_payload.get("debounce_s") == 0.005,
        "run_mode_api": callable(getattr(v20, "_run_mode", None)),
        "score_view_api": callable(getattr(v20, "_score_view", None)),
        "channel_gate_api": callable(getattr(v21, "two_sensor_view_gate", None)),
        "fast_fsm_api": callable(getattr(v21, "fast_fsm_events", None)),
        "legacy_pure_evaluator_api": callable(getattr(v21_gate, "_evaluate", None)),
        "plugin_loaded_from_verified_absolute_basename": True,
        "model_initialized": True,
        "detector_frames_resolved": True,
    }
    if not all(checks.values()):
        raise V22DevelopmentError(
            f"common environment preflight failed: {checks}"
        )
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
        "detector_replay_started": False,
    }


def preflight_unopened() -> dict[str, Any]:
    system = platform.system()
    machine = platform.machine().lower()
    if system != "Darwin" or machine not in {"arm64", "aarch64"}:
        raise V22DevelopmentError(
            "numerical V22 replay is currently attested only on macOS arm64"
        )
    lock, checks = _validate_freeze_lock()
    common_environment = _preflight_common_environment(lock)
    if os.path.lexists(freeze.EXECUTION_LEDGER_PATH):
        raise V22DevelopmentError(
            "V22 global execution ledger already exists; cycle consumed"
        )
    if os.path.lexists(freeze.OUTPUT_DIR):
        raise V22DevelopmentError(
            "V22 fixed output directory already exists; cycle consumed"
        )
    return {
        "status": "V22_TRIAL08_OPEN_DEVELOPMENT_PREFLIGHT_READY_UNOPENED",
        "candidate_id": freeze.CANDIDATE_ID,
        "trial_role": freeze.TRIAL_ROLE,
        "claim_limit": CLAIM_LIMIT,
        "detector_replay_started": False,
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


def _atomic_write_exclusive(
    path: Path, writer: Callable[[TextIO], None]
) -> Path:
    if os.path.lexists(path):
        raise V22DevelopmentError(f"refusing to clobber existing path: {path}")
    if not path.parent.is_dir():
        raise V22DevelopmentError(f"output parent does not exist: {path.parent}")
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
        try:
            os.link(temporary, path)
        except FileExistsError as exc:
            raise V22DevelopmentError(
                f"refusing to clobber concurrently-created path: {path}"
            ) from exc
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
        "status": "V22_DEVELOPMENT_REPLAY_OPENED_CYCLE_CONSUMED",
        "opened_at_utc": _utc_now(),
        "process_id": os.getpid(),
        "trial_id": freeze.TRIAL_ID,
        "trial_role": freeze.TRIAL_ROLE,
        "scientifically_virgin": False,
        "independent_validation": False,
        "candidate_id": freeze.CANDIDATE_ID,
        "claim_limit": CLAIM_LIMIT,
        "freeze_lock": preflight["freeze_lock"],
        "platform": preflight["platform"],
        "common_environment": preflight["common_environment"],
        "fixed_output_dir": freeze.OUTPUT_DIR.relative_to(REPO_ROOT).as_posix(),
        "detector_replay_started": False,
        "cycle_consumed": True,
        "rerun_allowed": False,
        "rescue_reselection_or_retuning_allowed": False,
        "previous_v21_error_reclassified": False,
        "previous_v21_error_artifacts_mutable": False,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
        "prescribed_grf_authorized_for_read": False,
        "external_loads_authorized_for_read": False,
    }


def _open_stage(preflight: Mapping[str, Any]) -> dict[str, Any]:
    freeze.OUTPUT_DIR.parent.mkdir(parents=True, exist_ok=True)
    opening = _opening_payload(preflight)
    _write_json_exclusive(freeze.EXECUTION_LEDGER_PATH, opening)
    try:
        freeze.OUTPUT_DIR.mkdir(parents=False, exist_ok=False)
    except FileExistsError as exc:
        raise V22DevelopmentError(
            "fixed V22 output directory was concurrently occupied"
        ) from exc
    _fsync_directory(freeze.OUTPUT_DIR.parent)
    _write_json_exclusive(freeze.OUTPUT_DIR / ACCESS_RECEIPT_NAME, opening)
    _audit_stage("opening_ledger_and_receipt_published")
    return opening


def _assert_development_opened() -> dict[str, Any]:
    ledger = _strict_json(freeze.EXECUTION_LEDGER_PATH)
    receipt = _strict_json(freeze.OUTPUT_DIR / ACCESS_RECEIPT_NAME)
    checks = {
        "receipts_exact": ledger == receipt,
        "receipt_bytes_exact": freeze.EXECUTION_LEDGER_PATH.read_bytes()
        == (freeze.OUTPUT_DIR / ACCESS_RECEIPT_NAME).read_bytes(),
        "status": ledger.get("status")
        == "V22_DEVELOPMENT_REPLAY_OPENED_CYCLE_CONSUMED",
        "consumed": ledger.get("cycle_consumed") is True,
        "no_retry": ledger.get("rerun_allowed") is False,
        "candidate": ledger.get("candidate_id") == freeze.CANDIDATE_ID,
        "trial": ledger.get("trial_id") == freeze.TRIAL_ID,
        "role": ledger.get("trial_role") == freeze.TRIAL_ROLE,
        "process_owner": ledger.get("process_id") == os.getpid(),
        "freeze_hash": ledger.get("freeze_lock", {}).get("sha256")
        == sha256_file(freeze.FREEZE_PATH),
        "old_error_not_reclassified": ledger.get(
            "previous_v21_error_reclassified"
        )
        is False,
    }
    if not all(checks.values()):
        raise V22DevelopmentError(f"development opening guard failed: {checks}")
    return ledger


def _verify_declared_input(
    declaration: Mapping[str, Any], *, label: str
) -> Path:
    _assert_development_opened()
    _audit_input(label, verified=False)
    path = _verify_record(declaration, label=label)
    _audit_input(label, verified=True)
    return path


def _validate_replay_inputs(lock: Mapping[str, Any]) -> dict[str, Any]:
    _assert_development_opened()
    declarations = lock["trial"]["declared_inputs"]
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
        == [freeze.TRACE_START_S, freeze.TRACE_END_S],
        "absolute_time": preprocessing.get("absolute_timestamps_no_rezero")
        is True,
        "coverage": preprocessing.get("all_sources_cover_analysis_interval")
        is True,
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
        raise V22DevelopmentError(f"trial-08 replay binding failed: {checks}")
    _audit_stage("development_replay_inputs_verified")
    return {
        "paths": paths,
        "plugin_loader_absolute": _absolute_plugin_loader_from_binary(
            paths["plugin_binary_macos"]
        ),
        "checks": checks,
        "source_records": {
            label: _source_record(path) for label, path in paths.items()
        },
    }


def _time_grid() -> np.ndarray:
    """Reuse the frozen V21 pure lattice helper, then bind it to V22."""

    times = v21_gate._time_grid()
    if (
        times.size != freeze.TRACE_SAMPLE_COUNT
        or abs(float(times[0]) - freeze.TRACE_START_S) > 1e-12
        or abs(float(times[-1]) - freeze.TRACE_END_S) > 1e-9
        or not np.all(np.isfinite(times))
        or not np.allclose(
            np.diff(times), freeze.SAMPLE_DT_S, atol=1e-12, rtol=0.0
        )
    ):
        raise V22DevelopmentError("V22 detector replay lattice drifted")
    return times


def _acquire_trace(
    replay_inputs: Mapping[str, Any],
    profile_path: Path,
    *,
    profile_sha256: str,
    plugin_preloaded: bool,
) -> dict[str, Any]:
    _assert_development_opened()
    import opensim
    from binary_phase_detector import (
        BinaryPhaseDetectorSampler,
        load_binary_phase_detector_profile,
    )
    from config import SimulatorConfig
    from kinematics_interpolator import KinematicsInterpolator
    from model_loader import _load_plugin
    from sweep_binary_phase_detector_v21_geometry import SweepProgress

    if sha256_file(profile_path) != profile_sha256:
        raise V22DevelopmentError("candidate profile drifted before sampling")
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
    cfg.t_start = freeze.TRACE_START_S
    cfg.t_end = freeze.TRACE_END_S
    kinematics = KinematicsInterpolator(cfg)
    _audit_stage("development_ik_loaded_trace_sampling")
    times = _time_grid()
    heel = np.empty(times.size, dtype=bool)
    toe = np.empty(times.size, dtype=bool)
    clearance_min = {"left_heel": math.inf, "left_toe": math.inf}
    clearance_max = {"left_heel": -math.inf, "left_toe": -math.inf}
    coordinates = model.getCoordinateSet()
    progress = SweepProgress(
        total=int(times.size),
        label="V22 development trial08",
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
                raise V22DevelopmentError(
                    "binary sampler produced a non-native bool"
                )
            heel[row] = heel_bit
            toe[row] = toe_bit
            for role in ("left_heel", "left_toe"):
                clearance = float(reading.signed_clearance_m[role])
                if not math.isfinite(clearance):
                    raise V22DevelopmentError(
                        "binary sampler produced non-finite clearance"
                    )
                clearance_min[role] = min(clearance_min[role], clearance)
                clearance_max[role] = max(clearance_max[role], clearance)
            if row % 100 == 0 or row + 1 == times.size:
                progress.update(row + 1)
    except BaseException:
        progress.finish("FAILED")
        raise
    progress.finish("DONE")
    packed = v21_gate._pack_binary_bits(heel, toe)
    _audit_stage("development_binary_trace_acquired")
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


def _load_oracle(lock: Mapping[str, Any]) -> dict[str, Any]:
    _assert_development_opened()
    import build_canonical_grf_event_oracle as canonical_oracle

    declaration = lock["trial"]["declared_inputs"]["canonical_oracle"]
    path = _verify_declared_input(declaration, label="canonical_oracle")
    ledger = _strict_json(path, expected_sha256=str(declaration["sha256"]))
    canonical_oracle.validate_ledger(ledger)
    core = ledger.get("scientific_core")
    if not isinstance(core, Mapping):
        raise V22DevelopmentError("canonical oracle scientific core is missing")
    views = core.get("views")
    view_integrity = (
        v21_gate._validate_oracle_views(
            core,
            views,
            minimum_cycles=int(lock["gate"]["minimum_complete_cycles_per_view"]),
        )
        if isinstance(views, list)
        else {"pass": False, "views": []}
    )
    coverage = freeze.validate_oracle_trace_coverage(core)
    grid = core.get("time_grid", {})
    checks = {
        "trial": core.get("trial_id") == freeze.TRIAL_ID,
        "contract": core.get("event_contract_id")
        == "primary_grf_split_v1+two_sensor_highrate_v1",
        "core_sha": ledger.get("scientific_core_sha256")
        == freeze.ORACLE_CORE_SHA256,
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
        "oracle_grid_start": float(grid.get("start_s", math.nan))
        == freeze.ORACLE_START_S,
        "oracle_grid_end": float(grid.get("end_s", math.nan))
        == freeze.ORACLE_END_S,
        "oracle_grid_count": int(grid.get("sample_count", -1))
        == freeze.ORACLE_SAMPLE_COUNT,
        "global_grid_equality_required": False,
        "view_count": isinstance(views, list)
        and len(views) == int(lock["gate"]["required_view_count"]),
        "view_integrity": bool(view_integrity["pass"]),
        "coverage": bool(coverage["pass"]),
        "coverage_exactly_frozen": coverage == lock["oracle"]["coverage"],
        "oracle_not_reconstructed": True,
        "grf_not_rethresholded_by_consumer": True,
    }
    if not all(checks.values()):
        raise V22DevelopmentError(f"canonical oracle gate drifted: {checks}")
    _audit_stage("development_canonical_oracle_loaded_with_coverage")
    return {
        "ledger": ledger,
        "checks": checks,
        "view_integrity": view_integrity,
        "coverage": coverage,
        "source": _source_record(path),
    }


def _evaluate(
    lock: Mapping[str, Any], trace: Mapping[str, Any], oracle: Mapping[str, Any]
) -> dict[str, Any]:
    """Run the unchanged frozen V21/V20 numerical gate on V22 evidence."""

    if lock.get("gate") != freeze.v21_freeze.FROZEN_GATE:
        raise V22DevelopmentError("numerical gate differs from frozen V21")
    result = v21_gate._evaluate(lock, trace, oracle)
    json.dumps(result, allow_nan=False)
    _audit_stage("development_gate_evaluation_complete")
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
    _assert_development_opened()
    trace_manifest = {
        "schema_version": SCHEMA_VERSION,
        "run_id": RUN_ID,
        "trial_id": freeze.TRIAL_ID,
        "trial_role": freeze.TRIAL_ROLE,
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
        "oracle_coverage_binding.json": {
            "schema_version": SCHEMA_VERSION,
            "trial_id": freeze.TRIAL_ID,
            "checks": oracle["checks"],
            "view_integrity": oracle["view_integrity"],
            "coverage": oracle["coverage"],
            "source": oracle["source"],
            "global_grid_equality_required": False,
            "oracle_reconstructed_by_consumer": False,
            "prescribed_grf_rethresholded_by_consumer": False,
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
        "previous_v21_terminal_preservation.json": {
            "schema_version": SCHEMA_VERSION,
            "previous_error_reclassified": False,
            "previous_error_artifacts_mutable": False,
            "previous_v21_terminal_error": preflight["lock"][
                "previous_v21_terminal_error"
            ],
        },
    }
    records: dict[str, Any] = {}
    for name, payload in payloads.items():
        path = _write_json_exclusive(freeze.OUTPUT_DIR / name, payload)
        records[name] = _artifact_record(path)

    passed = bool(evaluation["pass"])
    terminal_status = (
        "PASS_V22_TRIAL08_OPEN_DEVELOPMENT_REPLAY_CANDIDATE_FREEZE_ALLOWED"
        if passed
        else "FAIL_V22_TRIAL08_OPEN_DEVELOPMENT_REPLAY_TERMINAL"
    )
    next_stage = (
        "FREEZE_DEVELOPMENT_CANDIDATE_BUNDLE_ONLY_NO_H0_EXECUTION"
        if passed
        else "STOP_V22_NO_RETRY_NO_RESELECT_NO_RETUNE"
    )
    manifest = {
        "schema_version": SCHEMA_VERSION,
        "run_id": RUN_ID,
        "status": "V22_DEVELOPMENT_EVIDENCE_COMPLETE_AWAITING_TERMINAL_DECISION",
        "expected_terminal_status": terminal_status,
        "pass": passed,
        "candidate_id": freeze.CANDIDATE_ID,
        "trial_id": freeze.TRIAL_ID,
        "trial_role": freeze.TRIAL_ROLE,
        "scientifically_virgin": False,
        "independent_validation": False,
        "claim_limit": CLAIM_LIMIT,
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
        "oracle_grid_equals_trace_grid": False,
        "oracle_scoreable_coverage_pass": oracle["coverage"]["pass"],
        "numerical_gate_changed_from_v21": False,
        "data_access": {
            "trial08_open_development": True,
            "development_inputs_attempted": list(
                _EXECUTION_AUDIT["development_inputs_attempted"]
            ),
            "development_inputs_verified": list(
                _EXECUTION_AUDIT["development_inputs_verified"]
            ),
            "prescribed_grf_read": False,
            "external_loads_read": False,
            "protected_trials_opened": [],
            "reserve_trials_opened": [],
        },
        "previous_v21_error_reclassified": False,
        "previous_v21_error_artifacts_mutable": False,
        "primary_grf_modified": False,
        "cpp_or_contact_modified": False,
        "sea_semantics_modified": False,
        "geometry_or_fsm_retuned_in_v22": False,
        "h0_executed": False,
        "runtime_promoted": False,
        "training_promoted": False,
        "positive_morphology_reward_ppo_run": False,
        "post_pass_scope": preflight["lock"]["post_pass_scope"],
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
        "cycle_consumed": True,
        "rerun_allowed": False,
        "candidate_id": freeze.CANDIDATE_ID,
        "trial_id": freeze.TRIAL_ID,
        "trial_role": freeze.TRIAL_ROLE,
        "scientifically_virgin": False,
        "independent_validation_claim": False,
        "claim_limit": CLAIM_LIMIT,
        "unit_count": evaluation["unit_count"],
        "unit_pass_count": evaluation["unit_pass_count"],
        "channel_gate_pass": evaluation["two_sensor_channel_gate"]["pass"],
        "parity_pass": evaluation["parity"]["pass"],
        "fsm_contract_pass": evaluation["fsm_contract_binding"]["pass"],
        "terminal_pending_state_pass": evaluation[
            "terminal_pending_state_gate"
        ]["pass"],
        "oracle_scoreable_coverage_pass": oracle["coverage"]["pass"],
        "previous_v21_error_reclassified": False,
        "previous_v21_error_artifacts_mutable": False,
        "h0_ready_claim": False,
        "runtime_or_training_promotion": False,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
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


def execute_development() -> dict[str, Any]:
    _EXECUTION_AUDIT.clear()
    _EXECUTION_AUDIT.update(
        {
            "stage": "pre_open",
            "development_inputs_attempted": [],
            "development_inputs_verified": [],
        }
    )
    preflight = preflight_unopened()
    opening = _open_stage(preflight)
    lock = preflight["lock"]
    replay_inputs = _validate_replay_inputs(lock)
    profile_path = _verify_record(
        lock["candidate"]["profile"], label="candidate.profile.post_open"
    )
    oracle = _load_oracle(lock)
    trace = _acquire_trace(
        replay_inputs,
        profile_path,
        profile_sha256=str(lock["candidate"]["profile"]["sha256"]),
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
        or ledger.get("run_id") != RUN_ID
    ):
        return
    receipt_path = freeze.OUTPUT_DIR / ACCESS_RECEIPT_NAME
    receipt_published = receipt_path.is_file()
    if receipt_published:
        try:
            receipt = _strict_json(receipt_path)
        except BaseException:
            return
        if receipt != ledger or receipt_path.read_bytes() != (
            freeze.EXECUTION_LEDGER_PATH.read_bytes()
        ):
            return
    payload = {
        "schema_version": SCHEMA_VERSION,
        "run_id": RUN_ID,
        "status": "ERROR_V22_TRIAL08_OPEN_DEVELOPMENT_REPLAY_CONSUMED",
        "failed_at_utc": _utc_now(),
        "cycle_consumed": True,
        "rerun_allowed": False,
        "candidate_id": freeze.CANDIDATE_ID,
        "trial_id": freeze.TRIAL_ID,
        "trial_role": freeze.TRIAL_ROLE,
        "scientifically_virgin": False,
        "independent_validation": False,
        "claim_limit": CLAIM_LIMIT,
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
        "previous_v21_error_reclassified": False,
        "previous_v21_error_artifacts_mutable": False,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
        "h0_executed": False,
        "runtime_promoted": False,
        "training_promoted": False,
        "next_stage": "STOP_V22_NO_RETRY_NO_RESELECT_NO_RETUNE",
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
        help="validate the frozen V22 contract without opening detector replay",
    )
    modes.add_argument(
        "--execute-development",
        action="store_true",
        help="permanently consume and execute the fixed V22 development replay",
    )
    args = parser.parse_args(argv)
    if args.check:
        try:
            result = preflight_unopened()
        except Exception as exc:
            print(
                f"V22 development preflight failed closed: "
                f"{type(exc).__name__}: {exc}",
                file=sys.stderr,
            )
            return 2
        public = {key: value for key, value in result.items() if key != "lock"}
        print(json.dumps(public, indent=2, sort_keys=True, allow_nan=False))
        return 0

    opened_before_failure = os.path.lexists(freeze.EXECUTION_LEDGER_PATH)
    try:
        result = execute_development()
    except BaseException as exc:
        opened_after_failure = os.path.lexists(freeze.EXECUTION_LEDGER_PATH)
        _write_failure_after_open(exc)
        if opened_before_failure or opened_after_failure:
            message = "V22 development replay failed closed and is consumed"
        else:
            message = "V22 development preflight failed before opening"
        print(f"{message}: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 2
    print(json.dumps(result, indent=2, sort_keys=True, allow_nan=False))
    return 0 if result["decision"]["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
