"""Execute the explicitly authorized V23 open-development replay on trial 08.

V23 preserves the V21 geometry, V20 FSM, canonical oracle, evaluator, and
numerical gates.  Its only procedural correction is architectural: the real
oracle binding is completed before the execution ledger is opened, positive
assertions are kept separate from descriptive facts, and the already-verified
in-memory oracle is passed to the unchanged evaluator.  After opening, source
files are hash-checked again but the oracle is never parsed or rebound.

``--check`` is read-only.  ``--execute-development`` permanently consumes the
single V23 development cycle.  A PASS remains development evidence only and
does not authorize H0 execution, protected data, runtime/training promotion,
corridor activation, or PPO.
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

import freeze_binary_phase_detector_v23_trial08_development as freeze  # noqa: E402
import validate_binary_phase_detector_v21_trial08_one_shot as v21_gate  # noqa: E402


SCHEMA_VERSION = 23
RUN_ID = "AB06_BINARY_POINT_V23_TRIAL08_OPEN_DEVELOPMENT_ASSERTIONS_FACTS_FIX"
MODES = ("sequential_1ms", "batched_10ms_same_samples")
ACCESS_RECEIPT_NAME = "trial08_development_access_receipt.json"
FAILURE_NAME = "failure.json"
DECISION_NAME = "trial08_development_decision_lock.json"
MANIFEST_NAME = "manifest.json"
PLUGIN_LOADER = "plugins/SEA_Plugin_BlackBox_mCMC_impedence_ff"
CLAIM_LIMIT = (
    "open development replay on already-open trial 08 after terminal V21/V22 "
    "procedural errors; not independent validation and not H0, runtime, "
    "training, protected-trial, corridor, or PPO promotion"
)

_EXECUTION_AUDIT: dict[str, Any] = {
    "stage": "pre_open",
    "preopen_inputs_attempted": [],
    "preopen_inputs_verified": [],
    "freeze_payload_binding_verified_preopen": False,
    "live_oracle_binding_calls_preopen": 0,
    "oracle_binding_calls_post_open": 0,
    "post_open_sources_reverified": [],
}


class V23DevelopmentError(RuntimeError):
    """Raised when the frozen V23 development contract cannot be preserved."""


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def _audit_stage(stage: str) -> None:
    _EXECUTION_AUDIT["stage"] = str(stage)


def _audit_input(label: str, *, verified: bool) -> None:
    key = "preopen_inputs_verified" if verified else "preopen_inputs_attempted"
    values = _EXECUTION_AUDIT[key]
    if label not in values:
        values.append(str(label))


def _reset_audit() -> None:
    _EXECUTION_AUDIT.clear()
    _EXECUTION_AUDIT.update(
        {
            "stage": "pre_open",
            "preopen_inputs_attempted": [],
            "preopen_inputs_verified": [],
            "freeze_payload_binding_verified_preopen": False,
            "live_oracle_binding_calls_preopen": 0,
            "oracle_binding_calls_post_open": 0,
            "post_open_sources_reverified": [],
        }
    )


def _reject_json_constant(value: str) -> None:
    raise V23DevelopmentError(f"non-finite JSON constant is forbidden: {value}")


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
        raise V23DevelopmentError(f"required JSON is missing: {path}")
    observed_sha = sha256_file(path)
    if expected_sha256 is not None and observed_sha != expected_sha256:
        raise V23DevelopmentError(
            f"hash-pinned JSON drifted: {path}: "
            f"{observed_sha} != {expected_sha256}"
        )
    try:
        value = json.loads(
            path.read_text(encoding="utf-8"),
            parse_constant=_reject_json_constant,
        )
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise V23DevelopmentError(f"cannot read strict JSON: {path}") from exc
    if not isinstance(value, dict):
        raise V23DevelopmentError(f"JSON root must be an object: {path}")
    return value


def _resolve_repo_path(relative: str) -> Path:
    value = Path(relative)
    if value.is_absolute() or ".." in value.parts:
        raise V23DevelopmentError(f"non-portable repository path: {relative!r}")
    resolved = (REPO_ROOT / value).resolve()
    try:
        resolved.relative_to(REPO_ROOT.resolve())
    except ValueError as exc:
        raise V23DevelopmentError(f"path escapes repository: {relative!r}") from exc
    return resolved


def _source_record(path: Path) -> dict[str, Any]:
    resolved = path.resolve()
    if not resolved.is_file():
        raise V23DevelopmentError(f"source is missing: {resolved}")
    return {
        "path": resolved.relative_to(REPO_ROOT.resolve()).as_posix(),
        "sha256": sha256_file(resolved),
        "size_bytes": int(resolved.stat().st_size),
    }


def _verify_record(record: Mapping[str, Any], *, label: str) -> Path:
    if set(record) != {"path", "sha256", "size_bytes"}:
        raise V23DevelopmentError(f"malformed source record: {label}")
    path = _resolve_repo_path(str(record["path"]))
    if not path.is_file():
        raise V23DevelopmentError(f"missing source: {label}")
    if int(path.stat().st_size) != int(record["size_bytes"]):
        raise V23DevelopmentError(f"source size drifted: {label}")
    if sha256_file(path) != str(record["sha256"]):
        raise V23DevelopmentError(f"source hash drifted: {label}")
    return path


def _require_positive_assertions(
    assertions: Mapping[str, Any], *, label: str
) -> None:
    """Aggregate only positive boolean assertions, never descriptive facts."""

    if not assertions:
        raise V23DevelopmentError(f"{label} assertions are empty")
    if not all(type(value) is bool for value in assertions.values()):
        raise V23DevelopmentError(f"{label} assertions must be native bools")
    if not all(assertions.values()):
        raise V23DevelopmentError(f"{label} assertions failed: {dict(assertions)}")


def _validate_freeze_lock() -> tuple[dict[str, Any], dict[str, Any]]:
    if not freeze.FREEZE_PATH.is_file():
        raise V23DevelopmentError("V23 development freeze lock is missing")
    lock = _strict_json(freeze.FREEZE_PATH)
    expected = freeze.build_freeze_payload(check_destinations=False)
    _EXECUTION_AUDIT["freeze_payload_binding_verified_preopen"] = True
    assertions = {
        "exact_frozen_payload": lock == expected,
        "schema_exact": lock.get("schema_version") == freeze.SCHEMA_VERSION,
        "protocol_exact": lock.get("protocol_id") == freeze.PROTOCOL_ID,
        "status_exact": lock.get("status")
        == "V23_OPEN_DEVELOPMENT_FROZEN_BEFORE_DETECTOR_REPLAY",
        "authorization_exact": lock.get("authorization", {}).get("request")
        == "autorizzo V23",
        "candidate_exact": lock.get("candidate", {}).get("candidate_id")
        == freeze.CANDIDATE_ID,
        "single_candidate": lock.get("candidate", {}).get("candidate_count") == 1,
        "trial_exact": lock.get("trial", {}).get("trial_id") == freeze.TRIAL_ID,
        "role_exact": lock.get("trial", {}).get("role") == freeze.TRIAL_ROLE,
        "already_open": lock.get("trial", {}).get("already_open_before_v23")
        is True,
        "gate_unchanged": lock.get("gate") == freeze.v21_freeze.FROZEN_GATE,
        "assertions_facts_separated": lock.get("correction", {}).get(
            "facts_are_never_aggregated"
        )
        is True,
        "oracle_assertions_pass": lock.get("oracle", {})
        .get("binding", {})
        .get("assertions_pass")
        is True,
        "terminal_decision_last": lock.get("opening_contract", {}).get(
            "terminal_decision_last"
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
        "scope_exact": lock.get("post_pass_scope") == freeze.POST_PASS_SCOPE,
        "no_h0": lock.get("post_pass_scope", {}).get("h0_execution_allowed")
        is False,
        "no_protected": lock.get("post_pass_scope", {}).get(
            "protected_trial_access_allowed"
        )
        is False,
        "no_runtime": lock.get("post_pass_scope", {}).get(
            "runtime_promotion_allowed"
        )
        is False,
        "no_training": lock.get("post_pass_scope", {}).get(
            "training_promotion_allowed"
        )
        is False,
        "no_corridor": lock.get("post_pass_scope", {}).get(
            "corridor_activation_allowed"
        )
        is False,
    }
    _require_positive_assertions(assertions, label="freeze")
    facts = {
        "global_grid_equality_required": lock.get("correction", {}).get(
            "global_grid_equality_required"
        ),
        "scientifically_virgin": lock.get("trial", {}).get(
            "scientifically_virgin"
        ),
        "independent_validation": lock.get("trial", {}).get(
            "independent_validation"
        ),
    }
    if facts != {
        "global_grid_equality_required": False,
        "scientifically_virgin": False,
        "independent_validation": False,
    }:
        raise V23DevelopmentError(f"freeze facts drifted: {facts}")
    for label, record in lock["sources"].items():
        if not isinstance(record, Mapping):
            raise V23DevelopmentError(f"invalid freeze source record: {label}")
        _verify_record(record, label=f"freeze.sources.{label}")
    _verify_record(lock["candidate"]["profile"], label="candidate.profile")
    _verify_record(lock["candidate"]["fsm"], label="candidate.fsm")
    for generation in ("v21", "v22"):
        terminal = lock[f"previous_{generation}_terminal_error"]
        for label, record in terminal["artifacts"].items():
            _verify_record(record, label=f"previous_{generation}.{label}")
    return lock, {"assertions": assertions, "facts": facts}


def _absolute_plugin_loader_from_binary(binary: Path) -> Path:
    binary = binary.resolve()
    if not binary.name.startswith("lib") or binary.suffix != ".dylib":
        raise V23DevelopmentError(
            "macOS plugin binary name cannot yield a safe absolute loader"
        )
    return binary.with_name(binary.name[len("lib") : -len(binary.suffix)])


def _verify_declared_input_preopen(
    declaration: Mapping[str, Any], *, label: str
) -> Path:
    _audit_input(label, verified=False)
    path = _verify_record(declaration, label=label)
    _audit_input(label, verified=True)
    return path


def _load_and_validate_oracle(lock: Mapping[str, Any]) -> dict[str, Any]:
    """Load and fully bind the canonical oracle without opening V23 outputs.

    This function performs no writes and never samples the detector.  The
    returned ledger is the sole in-memory oracle consumed by execution.
    """

    declaration = lock["trial"]["declared_inputs"]["canonical_oracle"]
    path = _verify_record(declaration, label="canonical_oracle.preopen")
    ledger = _strict_json(path, expected_sha256=str(declaration["sha256"]))
    core = ledger.get("scientific_core")
    if not isinstance(core, Mapping):
        raise V23DevelopmentError("canonical oracle scientific_core is missing")
    binding = freeze.validate_oracle_binding(core, ledger)
    assertions = binding.get("assertions")
    facts = binding.get("facts")
    if not isinstance(assertions, Mapping) or not isinstance(facts, Mapping):
        raise V23DevelopmentError("oracle binding assertions/facts are malformed")
    _require_positive_assertions(assertions, label="oracle binding")
    if binding.get("assertions_pass") is not True:
        raise V23DevelopmentError("oracle binding aggregate drifted")
    frozen_binding = lock.get("oracle", {}).get("binding")
    if binding != frozen_binding:
        raise V23DevelopmentError("live pre-open oracle binding differs from freeze")
    if facts.get("global_grid_equality_required") is not False:
        raise V23DevelopmentError("oracle grid-equality fact drifted")
    return {
        "ledger": ledger,
        "assertions": dict(assertions),
        "facts": dict(facts),
        "assertions_pass": True,
        "coverage": binding["coverage"],
        "view_integrity": binding["view_integrity"],
        "source": _source_record(path),
        "bound_preopen": True,
    }


def _preflight_replay_inputs(lock: Mapping[str, Any]) -> dict[str, Any]:
    declarations = lock["trial"]["declared_inputs"]
    paths = {
        label: _verify_declared_input_preopen(declarations[label], label=label)
        for label in (
            "preprocessing_lock",
            "ik_motion",
            "model",
            "plugin_binary_macos",
        )
    }
    preprocessing = _strict_json(
        paths["preprocessing_lock"],
        expected_sha256=str(declarations["preprocessing_lock"]["sha256"]),
    )
    assertions = {
        "trial_exact": preprocessing.get("trial_id") == freeze.TRIAL_ID,
        "development_stage": preprocessing.get("stage") == "development",
        "status_exact": preprocessing.get("status")
        == "V14_1_PREPROCESSING_FROZEN_BEFORE_DETECTOR_REPLAY",
        "interval_exact": preprocessing.get("analysis_interval_s")
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
    assertions["model_bound"] = (
        model_record.get("path") == declarations["model"]["path"]
        and model_record.get("sha256") == declarations["model"]["sha256"]
    )
    assertions["plugin_bound"] = (
        plugin_record.get("binary_path")
        == declarations["plugin_binary_macos"]["path"]
        and plugin_record.get("binary_sha256")
        == declarations["plugin_binary_macos"]["sha256"]
        and plugin_record.get("loader_basename") == PLUGIN_LOADER
    )
    _require_positive_assertions(assertions, label="replay input")
    return {
        "paths": paths,
        "plugin_loader_absolute": _absolute_plugin_loader_from_binary(
            paths["plugin_binary_macos"]
        ),
        "assertions": assertions,
        "source_records": {
            label: _source_record(path) for label, path in paths.items()
        },
    }


def _preflight_common_environment(
    lock: Mapping[str, Any], replay_inputs: Mapping[str, Any]
) -> dict[str, Any]:
    import opensim
    import sweep_binary_phase_detector_v21_geometry as v21
    import validate_binary_phase_fsm_v20_development as v20
    from binary_phase_detector import (
        BinaryPhaseDetectorSampler,
        load_binary_phase_detector_profile,
    )
    from binary_phase_fsm import BinaryPhaseFSM
    from model_loader import _load_plugin

    profile_path = _verify_record(
        lock["candidate"]["profile"], label="candidate.profile.preopen"
    )
    profile = load_binary_phase_detector_profile(profile_path)
    fsm_payload = BinaryPhaseFSM().payload()
    plugin_loader_absolute = replay_inputs["plugin_loader_absolute"]
    _load_plugin(str(plugin_loader_absolute))
    model = opensim.Model(str(replay_inputs["paths"]["model"]))
    model.initSystem()
    BinaryPhaseDetectorSampler(model, profile)
    assertions = {
        "binary_sampler_api": callable(
            getattr(BinaryPhaseDetectorSampler, "sample", None)
        ),
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
        "legacy_evaluator_api": callable(getattr(v21_gate, "_evaluate", None)),
        "plugin_loaded": True,
        "model_initialized": True,
        "detector_frames_resolved": True,
    }
    _require_positive_assertions(assertions, label="common environment")
    return {
        "assertions": assertions,
        "environment": {
            "python": platform.python_version(),
            "numpy": np.__version__,
            "opensim_python": str(getattr(opensim, "__version__", "unknown")),
            "opensim_build": str(opensim.GetVersionAndDate()),
        },
        "model": _source_record(replay_inputs["paths"]["model"]),
        "plugin_binary_macos": _source_record(
            replay_inputs["paths"]["plugin_binary_macos"]
        ),
        "profile": _source_record(profile_path),
        "plugin_loaded_in_current_process": True,
        "plugin_loader_repo_path": plugin_loader_absolute.relative_to(
            REPO_ROOT
        ).as_posix(),
        "detector_replay_started": False,
    }


def _bound_source_records(
    lock: Mapping[str, Any], replay_inputs: Mapping[str, Any], oracle: Mapping[str, Any]
) -> dict[str, dict[str, Any]]:
    records = {
        f"source:{label}": dict(record)
        for label, record in lock["sources"].items()
    }
    records.update(
        {
            "candidate:profile": dict(lock["candidate"]["profile"]),
            "candidate:fsm": dict(lock["candidate"]["fsm"]),
            "oracle:canonical": dict(oracle["source"]),
            "freeze:lock": _source_record(freeze.FREEZE_PATH),
        }
    )
    for label, record in replay_inputs["source_records"].items():
        records[f"replay:{label}"] = dict(record)
    return records


def preflight_unopened() -> dict[str, Any]:
    system = platform.system()
    machine = platform.machine().lower()
    if system != "Darwin" or machine not in {"arm64", "aarch64"}:
        raise V23DevelopmentError(
            "numerical V23 replay is currently attested only on macOS arm64"
        )
    lock, freeze_binding = _validate_freeze_lock()
    replay_inputs = _preflight_replay_inputs(lock)
    oracle = _load_and_validate_oracle(lock)
    _EXECUTION_AUDIT["live_oracle_binding_calls_preopen"] += 1
    _audit_input("canonical_oracle", verified=False)
    _audit_input("canonical_oracle", verified=True)
    common_environment = _preflight_common_environment(lock, replay_inputs)
    if os.path.lexists(freeze.EXECUTION_LEDGER_PATH):
        raise V23DevelopmentError(
            "V23 global execution ledger already exists; cycle consumed"
        )
    if os.path.lexists(freeze.OUTPUT_DIR):
        raise V23DevelopmentError(
            "V23 fixed output directory already exists; cycle consumed"
        )
    sources = _bound_source_records(lock, replay_inputs, oracle)
    _audit_stage("preopen_oracle_bound_and_environment_verified")
    return {
        "status": "V23_TRIAL08_OPEN_DEVELOPMENT_PREFLIGHT_READY_UNOPENED",
        "candidate_id": freeze.CANDIDATE_ID,
        "trial_role": freeze.TRIAL_ROLE,
        "claim_limit": CLAIM_LIMIT,
        "detector_replay_started": False,
        "oracle_bound_preopen": True,
        "platform": {
            "system": system,
            "machine": machine,
            "numerical_claim_scope": "macOS-arm64-only",
            "portable_scope": "validator contract and synthetic tests",
        },
        "freeze_lock": _source_record(freeze.FREEZE_PATH),
        "freeze_binding": freeze_binding,
        "common_environment": common_environment,
        "replay_inputs": replay_inputs,
        "oracle": oracle,
        "bound_source_records": sources,
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
        raise V23DevelopmentError(f"refusing to clobber existing path: {path}")
    if not path.parent.is_dir():
        raise V23DevelopmentError(f"output parent does not exist: {path.parent}")
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
            raise V23DevelopmentError(
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
    oracle = preflight["oracle"]
    return {
        "schema_version": SCHEMA_VERSION,
        "run_id": RUN_ID,
        "status": "V23_DEVELOPMENT_REPLAY_OPENED_CYCLE_CONSUMED",
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
        "preopen_oracle_binding": {
            "bound_preopen": oracle["bound_preopen"],
            "source": oracle["source"],
            "assertions": oracle["assertions"],
            "facts": oracle["facts"],
            "assertions_pass": oracle["assertions_pass"],
            "coverage": oracle["coverage"],
            "view_integrity": oracle["view_integrity"],
        },
        "fixed_output_dir": freeze.OUTPUT_DIR.relative_to(REPO_ROOT).as_posix(),
        "detector_replay_started": False,
        "cycle_consumed": True,
        "rerun_allowed": False,
        "rescue_reselection_or_retuning_allowed": False,
        "previous_v21_error_reclassified": False,
        "previous_v22_error_reclassified": False,
        "previous_terminal_artifacts_mutable": False,
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
        raise V23DevelopmentError(
            "fixed V23 output directory was concurrently occupied"
        ) from exc
    _fsync_directory(freeze.OUTPUT_DIR.parent)
    _write_json_exclusive(freeze.OUTPUT_DIR / ACCESS_RECEIPT_NAME, opening)
    _audit_stage("opening_ledger_and_receipt_published")
    return opening


def _assert_development_opened() -> dict[str, Any]:
    ledger = _strict_json(freeze.EXECUTION_LEDGER_PATH)
    receipt_path = freeze.OUTPUT_DIR / ACCESS_RECEIPT_NAME
    receipt = _strict_json(receipt_path)
    assertions = {
        "receipts_exact": ledger == receipt,
        "receipt_bytes_exact": freeze.EXECUTION_LEDGER_PATH.read_bytes()
        == receipt_path.read_bytes(),
        "status_exact": ledger.get("status")
        == "V23_DEVELOPMENT_REPLAY_OPENED_CYCLE_CONSUMED",
        "consumed": ledger.get("cycle_consumed") is True,
        "no_retry": ledger.get("rerun_allowed") is False,
        "candidate_exact": ledger.get("candidate_id") == freeze.CANDIDATE_ID,
        "trial_exact": ledger.get("trial_id") == freeze.TRIAL_ID,
        "role_exact": ledger.get("trial_role") == freeze.TRIAL_ROLE,
        "process_owner": ledger.get("process_id") == os.getpid(),
        "freeze_hash_exact": ledger.get("freeze_lock", {}).get("sha256")
        == sha256_file(freeze.FREEZE_PATH),
        "oracle_bound_preopen": ledger.get("preopen_oracle_binding", {}).get(
            "bound_preopen"
        )
        is True,
        "oracle_assertions_pass": ledger.get("preopen_oracle_binding", {}).get(
            "assertions_pass"
        )
        is True,
        "old_errors_not_reclassified": (
            ledger.get("previous_v21_error_reclassified") is False
            and ledger.get("previous_v22_error_reclassified") is False
        ),
    }
    _require_positive_assertions(assertions, label="development opening")
    facts = ledger["preopen_oracle_binding"].get("facts", {})
    if facts.get("global_grid_equality_required") is not False:
        raise V23DevelopmentError("opening oracle facts drifted")
    return ledger


def _reverify_preopen_sources(preflight: Mapping[str, Any]) -> dict[str, Any]:
    """After opening, hash-check only; never parse or bind the oracle again."""

    _assert_development_opened()
    records = preflight["bound_source_records"]
    verified: list[str] = []
    for label, record in records.items():
        if not isinstance(record, Mapping):
            raise V23DevelopmentError(f"invalid pre-open source snapshot: {label}")
        _verify_record(record, label=f"post_open.{label}")
        verified.append(str(label))
    if _EXECUTION_AUDIT["oracle_binding_calls_post_open"] != 0:
        raise V23DevelopmentError("oracle was rebound after V23 opening")
    _EXECUTION_AUDIT["post_open_sources_reverified"] = verified
    _audit_stage("post_open_hash_only_reverification_complete")
    return {"pass": True, "verified_labels": verified}


def _time_grid() -> np.ndarray:
    times = v21_gate._time_grid()
    assertions = {
        "count_exact": times.size == freeze.TRACE_SAMPLE_COUNT,
        "start_exact": abs(float(times[0]) - freeze.TRACE_START_S) <= 1e-12,
        "end_exact": abs(float(times[-1]) - freeze.TRACE_END_S) <= 1e-9,
        "finite": bool(np.all(np.isfinite(times))),
        "dt_exact": bool(
            np.allclose(
                np.diff(times), freeze.SAMPLE_DT_S, atol=1e-12, rtol=0.0
            )
        ),
    }
    _require_positive_assertions(assertions, label="detector replay lattice")
    return times


def _acquire_trace(
    replay_inputs: Mapping[str, Any],
    profile_path: Path,
    *,
    profile_sha256: str,
    plugin_preloaded: bool,
) -> dict[str, Any]:
    _assert_development_opened()
    if _EXECUTION_AUDIT["oracle_binding_calls_post_open"] != 0:
        raise V23DevelopmentError("oracle binding occurred after opening")
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
        raise V23DevelopmentError("candidate profile drifted before sampling")
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
        label="V23 development trial08",
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
                raise V23DevelopmentError(
                    "binary sampler produced a non-native bool"
                )
            heel[row] = heel_bit
            toe[row] = toe_bit
            for role in ("left_heel", "left_toe"):
                clearance = float(reading.signed_clearance_m[role])
                if not math.isfinite(clearance):
                    raise V23DevelopmentError(
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


def _evaluate(
    lock: Mapping[str, Any], trace: Mapping[str, Any], oracle: Mapping[str, Any]
) -> dict[str, Any]:
    """Run the unchanged frozen V21/V20 numerical evaluator."""

    _assert_development_opened()
    if lock.get("gate") != freeze.v21_freeze.FROZEN_GATE:
        raise V23DevelopmentError("numerical gate differs from frozen V21")
    if oracle.get("bound_preopen") is not True:
        raise V23DevelopmentError("evaluator received an unbound oracle")
    if _EXECUTION_AUDIT["oracle_binding_calls_post_open"] != 0:
        raise V23DevelopmentError("oracle was rebound after V23 opening")
    result = v21_gate._evaluate(lock, trace, oracle)
    json.dumps(result, allow_nan=False)
    _audit_stage("development_gate_evaluation_complete")
    return result


def _write_result_artifacts(
    *,
    opening: Mapping[str, Any],
    preflight: Mapping[str, Any],
    trace: Mapping[str, Any],
    evaluation: Mapping[str, Any],
    post_open_reverification: Mapping[str, Any],
) -> dict[str, Any]:
    _assert_development_opened()
    oracle = preflight["oracle"]
    replay_inputs = preflight["replay_inputs"]
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
        "oracle_preopen_binding.json": {
            "schema_version": SCHEMA_VERSION,
            "trial_id": freeze.TRIAL_ID,
            "bound_preopen": oracle["bound_preopen"],
            "assertions": oracle["assertions"],
            "facts": oracle["facts"],
            "assertions_pass": oracle["assertions_pass"],
            "coverage": oracle["coverage"],
            "view_integrity": oracle["view_integrity"],
            "source": oracle["source"],
            "post_open_binding_calls": _EXECUTION_AUDIT[
                "oracle_binding_calls_post_open"
            ],
        },
        "post_open_source_reverification.json": {
            "schema_version": SCHEMA_VERSION,
            **post_open_reverification,
            "oracle_reparsed": False,
            "oracle_rebound": False,
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
        "previous_terminal_preservation.json": {
            "schema_version": SCHEMA_VERSION,
            "previous_v21_error_reclassified": False,
            "previous_v22_error_reclassified": False,
            "previous_terminal_artifacts_mutable": False,
            "previous_v21_terminal_error": preflight["lock"][
                "previous_v21_terminal_error"
            ],
            "previous_v22_terminal_error": preflight["lock"][
                "previous_v22_terminal_error"
            ],
        },
    }
    records: dict[str, Any] = {}
    for name, payload in payloads.items():
        path = _write_json_exclusive(freeze.OUTPUT_DIR / name, payload)
        records[name] = _artifact_record(path)

    passed = bool(evaluation["pass"])
    terminal_status = (
        "PASS_V23_TRIAL08_OPEN_DEVELOPMENT_REPLAY_CANDIDATE_FREEZE_ALLOWED"
        if passed
        else "FAIL_V23_TRIAL08_OPEN_DEVELOPMENT_REPLAY_TERMINAL"
    )
    next_stage = (
        "FREEZE_DEVELOPMENT_CANDIDATE_BUNDLE_ONLY_NO_H0_EXECUTION"
        if passed
        else "STOP_V23_NO_RETRY_NO_RESELECT_NO_RETUNE"
    )
    manifest = {
        "schema_version": SCHEMA_VERSION,
        "run_id": RUN_ID,
        "status": "V23_DEVELOPMENT_EVIDENCE_COMPLETE_AWAITING_TERMINAL_DECISION",
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
        "oracle_bound_preopen": True,
        "oracle_binding_assertions_pass": oracle["assertions_pass"],
        "oracle_binding_facts": oracle["facts"],
        "oracle_reparsed_or_rebound_post_open": False,
        "oracle_scoreable_coverage_pass": oracle["coverage"]["pass"],
        "numerical_gate_changed_from_v21": False,
        "data_access": {
            "trial08_open_development": True,
            "preopen_inputs_attempted": list(
                _EXECUTION_AUDIT["preopen_inputs_attempted"]
            ),
            "preopen_inputs_verified": list(
                _EXECUTION_AUDIT["preopen_inputs_verified"]
            ),
            "canonical_oracle_read_preopen": True,
            "prescribed_grf_read": False,
            "external_loads_read": False,
            "protected_trials_opened": [],
            "reserve_trials_opened": [],
        },
        "previous_v21_error_reclassified": False,
        "previous_v22_error_reclassified": False,
        "previous_terminal_artifacts_mutable": False,
        "primary_grf_modified": False,
        "cpp_or_contact_modified": False,
        "sea_semantics_modified": False,
        "geometry_or_fsm_retuned_in_v23": False,
        "h0_executed": False,
        "runtime_promoted": False,
        "training_promoted": False,
        "corridor_activated": False,
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

    # This is intentionally the final filesystem write of a successful run.
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
        "oracle_bound_preopen": True,
        "oracle_rebound_post_open": False,
        "oracle_scoreable_coverage_pass": oracle["coverage"]["pass"],
        "previous_v21_error_reclassified": False,
        "previous_v22_error_reclassified": False,
        "previous_terminal_artifacts_mutable": False,
        "h0_ready_claim": False,
        "runtime_training_or_corridor_promotion": False,
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
    _reset_audit()
    preflight = preflight_unopened()
    opening = _open_stage(preflight)
    post_open_reverification = _reverify_preopen_sources(preflight)
    lock = preflight["lock"]
    replay_inputs = preflight["replay_inputs"]
    profile_path = _verify_record(
        lock["candidate"]["profile"], label="candidate.profile.post_open_hash_only"
    )
    trace = _acquire_trace(
        replay_inputs,
        profile_path,
        profile_sha256=str(lock["candidate"]["profile"]["sha256"]),
        plugin_preloaded=bool(
            preflight["common_environment"]["plugin_loaded_in_current_process"]
        ),
    )
    # The exact object bound before opening is consumed; no oracle loader or
    # binding helper is called below this point.
    evaluation = _evaluate(lock, trace, preflight["oracle"])
    return _write_result_artifacts(
        opening=opening,
        preflight=preflight,
        trace=trace,
        evaluation=evaluation,
        post_open_reverification=post_open_reverification,
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
        "status": "ERROR_V23_TRIAL08_OPEN_DEVELOPMENT_REPLAY_CONSUMED",
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
        "terminal_decision_published": False,
        "oracle_bound_preopen": ledger.get("preopen_oracle_binding", {}).get(
            "bound_preopen"
        )
        is True,
        "oracle_binding_calls_post_open": _EXECUTION_AUDIT[
            "oracle_binding_calls_post_open"
        ],
        "previous_v21_error_reclassified": False,
        "previous_v22_error_reclassified": False,
        "previous_terminal_artifacts_mutable": False,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
        "h0_executed": False,
        "runtime_promoted": False,
        "training_promoted": False,
        "corridor_activated": False,
        "next_stage": "STOP_V23_NO_RETRY_NO_RESELECT_NO_RETUNE",
    }
    try:
        _write_json_exclusive(path, payload)
    except BaseException:
        pass


def _public_preflight(preflight: Mapping[str, Any]) -> dict[str, Any]:
    """Return a JSON-safe summary without the lock, oracle ledger, or Paths."""

    oracle = preflight["oracle"]
    return {
        "status": preflight["status"],
        "candidate_id": preflight["candidate_id"],
        "trial_role": preflight["trial_role"],
        "claim_limit": preflight["claim_limit"],
        "detector_replay_started": preflight["detector_replay_started"],
        "oracle_bound_preopen": preflight["oracle_bound_preopen"],
        "platform": preflight["platform"],
        "freeze_lock": preflight["freeze_lock"],
        "freeze_binding": preflight["freeze_binding"],
        "common_environment": preflight["common_environment"],
        "replay_input_assertions": preflight["replay_inputs"]["assertions"],
        "oracle_binding": {
            "assertions": oracle["assertions"],
            "facts": oracle["facts"],
            "assertions_pass": oracle["assertions_pass"],
            "coverage": oracle["coverage"],
            "view_integrity": oracle["view_integrity"],
            "source": oracle["source"],
        },
        "execution_audit": json.loads(
            json.dumps(_EXECUTION_AUDIT, sort_keys=True, allow_nan=False)
        ),
    }


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_mutually_exclusive_group(required=True)
    modes.add_argument(
        "--check",
        action="store_true",
        help="validate V23 fully, including real oracle binding, without opening",
    )
    modes.add_argument(
        "--execute-development",
        action="store_true",
        help="permanently consume and execute the fixed V23 development replay",
    )
    args = parser.parse_args(argv)
    if args.check:
        _reset_audit()
        try:
            result = preflight_unopened()
        except Exception as exc:
            print(
                f"V23 development preflight failed closed: "
                f"{type(exc).__name__}: {exc}",
                file=sys.stderr,
            )
            return 2
        print(
            json.dumps(
                _public_preflight(result),
                indent=2,
                sort_keys=True,
                allow_nan=False,
            )
        )
        return 0

    opened_before_failure = os.path.lexists(freeze.EXECUTION_LEDGER_PATH)
    try:
        result = execute_development()
    except BaseException as exc:
        opened_after_failure = os.path.lexists(freeze.EXECUTION_LEDGER_PATH)
        _write_failure_after_open(exc)
        if opened_before_failure or opened_after_failure:
            message = "V23 development replay failed closed and is consumed"
        else:
            message = "V23 development preflight failed before opening"
        print(f"{message}: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 2
    print(json.dumps(result, indent=2, sort_keys=True, allow_nan=False))
    return 0 if result["decision"]["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
