"""Build the non-simulation preflight for the V7 early-contact diagnostic."""

from __future__ import annotations

import json
import importlib.metadata
import math
import os
import platform
import sys
from pathlib import Path, PurePosixPath
from typing import Any, Mapping, Sequence


REPO_ROOT = Path(__file__).resolve().parents[1]
VALIDATION_ROOT = REPO_ROOT / "validation"
if str(VALIDATION_ROOT) not in sys.path:
    sys.path.insert(0, str(VALIDATION_ROOT))

import h0_forensic_rollout as forensic  # noqa: E402
import h0_primary_split_v7_early_contact_contract as contract  # noqa: E402


class V7EarlyContactPreflightError(RuntimeError):
    """Raised when any frozen diagnostic prerequisite is invalid."""


def resolve_relative(relative: str | PurePosixPath) -> Path:
    value = PurePosixPath(relative)
    if value.is_absolute() or ".." in value.parts or not value.parts:
        raise V7EarlyContactPreflightError(
            f"non-canonical repository path: {value}"
        )
    return REPO_ROOT.joinpath(*value.parts)


PREFLIGHT_PATH = resolve_relative(contract.PREFLIGHT_RECEIPT_PATH)
LOCK_PATH = resolve_relative(contract.LOCK_PATH)
RUN_ROOT = resolve_relative(contract.RUN_ROOT)
DESTINATION = RUN_ROOT / contract.CASE_ID


def source_record(path: str | Path) -> dict[str, Any]:
    unresolved = Path(path).expanduser()
    if unresolved.is_symlink():
        raise V7EarlyContactPreflightError(
            f"required file must not be a symlink: {unresolved}"
        )
    source = unresolved.resolve()
    if not source.is_file():
        raise V7EarlyContactPreflightError(
            f"required regular non-symlink file is missing: {source}"
        )
    return forensic.artifact_record(source, artifact_root=REPO_ROOT)


def _record_matches(record: Any, path: Path) -> bool:
    return (
        isinstance(record, Mapping)
        and set(record) == {"path", "sha256", "size_bytes"}
        and dict(record) == source_record(path)
    )


def _mapping(path: Path) -> dict[str, Any]:
    value = forensic.strict_json_load(path)
    if not isinstance(value, Mapping):
        raise V7EarlyContactPreflightError(f"expected JSON object: {path}")
    return dict(value)


def _sequence(path: Path) -> list[Any]:
    value = forensic.strict_json_load(path)
    if isinstance(value, (str, bytes)) or not isinstance(value, Sequence):
        raise V7EarlyContactPreflightError(f"expected JSON array: {path}")
    return list(value)


def source_paths() -> dict[str, Path]:
    return {
        name: resolve_relative(relative)
        for name, relative in contract.SOURCE_RELATIVE_PATHS.items()
    }


def input_paths() -> dict[str, Path]:
    return {
        name: resolve_relative(relative)
        for name, relative in contract.INPUT_RELATIVE_PATHS.items()
    }


def _distribution_version(name: str) -> str:
    try:
        value = importlib.metadata.version(name)
    except importlib.metadata.PackageNotFoundError as exc:
        raise V7EarlyContactPreflightError(
            f"required distribution is missing: {name}"
        ) from exc
    if not value:
        raise V7EarlyContactPreflightError(
            f"required distribution has an empty version: {name}"
        )
    return value


def platform_provenance() -> dict[str, Any]:
    return {
        "system": platform.system(),
        "machine": platform.machine(),
        "python_version": platform.python_version(),
        "python_implementation": platform.python_implementation(),
        "python_executable": Path(sys.executable).resolve().as_posix(),
        "distributions": {
            name: _distribution_version(name)
            for name in (
                "gymnasium",
                "numpy",
                "opensim",
                "ray",
                "scipy",
                "torch",
            )
        },
    }


def validate_platform_identity() -> dict[str, Any]:
    path = input_paths()["v3_platform_receipt"]
    receipt = _mapping(path)
    expected_keys = {
        "schema_version",
        "status",
        "passed",
        "protocol_id",
        "revision",
        "numerical_claim",
        "windows_claim",
        "identity",
        "binary_artifacts",
        "actor_updates",
        "critic_updates",
        "ppo_updates",
        "protected_trials_opened",
    }
    if (
        set(receipt) != expected_keys
        or receipt.get("schema_version") != 3
        or receipt.get("status") != "PASS_H0_PRIMARY_SPLIT_V3_PLATFORM"
        or receipt.get("passed") is not True
        or receipt.get("protocol_id")
        != "AB06_H0_PRIMARY_GRF_SPLIT_V3_SEMANTIC_REPLAY"
        or receipt.get("revision") != contract.REVISION
        or receipt.get("numerical_claim") != "macOS-arm64-only"
        or receipt.get("windows_claim")
        != "schema-and-path-compatibility-only"
        or receipt.get("actor_updates") != 0
        or receipt.get("critic_updates") != 0
        or receipt.get("ppo_updates") != 0
        or receipt.get("protected_trials_opened") != []
    ):
        raise V7EarlyContactPreflightError("V3 platform receipt drifted")
    live = platform_provenance()
    if (
        receipt.get("identity") != contract.EXPECTED_PLATFORM_IDENTITY
        or live != contract.EXPECTED_PLATFORM_IDENTITY
    ):
        raise V7EarlyContactPreflightError(
            "live numerical environment differs from the V3 platform identity"
        )
    binaries = receipt.get("binary_artifacts")
    expected_binary_paths = {
        "online_grf_macos_dylib": "plugins/libOnlineGRFContact.dylib",
        "sea_macos_dylib": (
            "plugins/libSEA_Plugin_BlackBox_mCMC_impedence_ff.dylib"
        ),
    }
    if not isinstance(binaries, Mapping) or set(binaries) != set(
        expected_binary_paths
    ):
        raise V7EarlyContactPreflightError(
            "V3 platform binary closure is malformed"
        )
    for name, relative in expected_binary_paths.items():
        if not _record_matches(binaries[name], resolve_relative(relative)):
            raise V7EarlyContactPreflightError(
                f"platform binary changed after V3 attestation: {name}"
            )
    return {
        "receipt": source_record(path),
        "identity": live,
        "binary_artifacts": {
            name: dict(binaries[name]) for name in sorted(binaries)
        },
        "numerical_claim": "macOS-arm64-only",
        "windows_claim": "schema-and-path-compatibility-only",
    }


def validate_primary_core() -> dict[str, Any]:
    path = input_paths()["primary_core_lock"]
    lock = _mapping(path)
    if (
        lock.get("schema_version") != 1
        or lock.get("lock_id")
        != "AB06_PRIMARY_GRF_HYBRID_CORE_LOCK_2026-08-03"
        or lock.get("status")
        != "PRIMARY_CONTRACT_FROZEN_LIMITED_HYBRID_CLAIM"
    ):
        raise V7EarlyContactPreflightError("primary GRF core lock drifted")
    scientific_core = lock.get("scientific_core")
    attestations = lock.get("platform_attestations")
    expected_core_names = {
        "contact_cmake",
        "contact_cpp",
        "contact_header",
        "plugin_interface",
        "primary_profile",
        "runtime_model",
    }
    if (
        not isinstance(scientific_core, Mapping)
        or set(scientific_core) != expected_core_names
        or not isinstance(attestations, Mapping)
        or not isinstance(attestations.get("macos_arm64_dylib"), Mapping)
        or attestations.get("windows_x86_64")
        != "PENDING_VERIFIED_DLL_HASH_AND_PARITY"
    ):
        raise V7EarlyContactPreflightError(
            "primary GRF scientific/platform closure is malformed"
        )

    def attest(raw: Any, label: str) -> dict[str, Any]:
        if (
            not isinstance(raw, Mapping)
            or not isinstance(raw.get("path"), str)
            or not isinstance(raw.get("sha256"), str)
        ):
            raise V7EarlyContactPreflightError(
                f"primary core record is malformed: {label}"
            )
        record = source_record(resolve_relative(str(raw["path"])))
        if (
            record["path"] != raw["path"]
            or record["sha256"] != raw["sha256"]
        ):
            raise V7EarlyContactPreflightError(
                f"primary core live bytes drifted: {label}"
            )
        return record

    live_core = {
        name: attest(scientific_core[name], f"scientific_core.{name}")
        for name in sorted(scientific_core)
    }
    macos_dylib = attest(
        attestations["macos_arm64_dylib"],
        "platform_attestations.macos_arm64_dylib",
    )
    if live_core["primary_profile"]["sha256"] != contract.EXPECTED_INPUT_SHA256[
        "primary_grf_profile"
    ]:
        raise V7EarlyContactPreflightError("primary profile/core binding drifted")
    return {
        "lock": source_record(path),
        "scientific_core": live_core,
        "macos_arm64_dylib": macos_dylib,
        "windows_x86_64": "PENDING_VERIFIED_DLL_HASH_AND_PARITY",
    }


def _finite_vector(value: Any, length: int) -> bool:
    return bool(
        isinstance(value, list)
        and len(value) == length
        and all(
            not isinstance(item, bool)
            and isinstance(item, (int, float))
            and math.isfinite(float(item))
            for item in value
        )
    )


def _expected_input_hashes(paths: Mapping[str, Path]) -> dict[str, Any]:
    records = {name: source_record(path) for name, path in paths.items()}
    for name, digest in contract.EXPECTED_INPUT_SHA256.items():
        if name not in records or records[name]["sha256"] != digest:
            raise V7EarlyContactPreflightError(
                f"frozen input hash drifted: {name}"
            )
    return records


def validate_v6_live_source_closure(
    lock: Mapping[str, Any],
) -> dict[str, dict[str, Any]]:
    """Re-attest every live runtime source frozen by the V6 lock.

    Citing the V6 lock alone would only prove historical bytes.  V7 imports
    and executes that stack, so every recorded path is resolved again and its
    current bytes must still match before the new closure can be frozen.
    """

    records = lock.get("sources")
    if not isinstance(records, Mapping) or len(records) != 26:
        raise V7EarlyContactPreflightError("V6 runtime source closure drifted")
    result: dict[str, dict[str, Any]] = {}
    for name, raw_record in records.items():
        if (
            not isinstance(name, str)
            or not isinstance(raw_record, Mapping)
            or set(raw_record) != {"path", "sha256", "size_bytes"}
            or not isinstance(raw_record.get("path"), str)
        ):
            raise V7EarlyContactPreflightError(
                f"V6 runtime source record malformed: {name!r}"
            )
        path = resolve_relative(str(raw_record["path"]))
        if not _record_matches(raw_record, path):
            raise V7EarlyContactPreflightError(
                f"V6 runtime source changed after freeze: {name}"
            )
        result[name] = dict(raw_record)
    return result


def validate_v3_runtime_closure() -> dict[str, Any]:
    """Use the V3 closure as the exhaustive runtime path inventory.

    Runtime inputs remain byte-exact to V3.  Four sources were intentionally
    revised by the later detector/H0 work; their paths are inherited from V3
    but their *current* bytes become the V7 source closure.
    """

    lock_path = input_paths()["v3_runtime_execution_lock"]
    lock = _mapping(lock_path)
    closure = lock.get("runtime_closure")
    if (
        lock.get("schema_version") != 3
        or lock.get("status") != "H0_PRIMARY_GRF_SPLIT_V3_EXECUTION_FROZEN"
        or lock.get("protocol_id")
        != "AB06_H0_PRIMARY_GRF_SPLIT_V3_SEMANTIC_REPLAY"
        or lock.get("critic_updates") != 0
        or lock.get("ppo_updates") != 0
        or lock.get("protected_trials_opened") != []
        or not isinstance(closure, Mapping)
    ):
        raise V7EarlyContactPreflightError("V3 runtime closure lock drifted")
    source_records = closure.get("runtime_sources")
    input_records = closure.get("runtime_inputs")
    if (
        not isinstance(source_records, Mapping)
        or len(source_records) != 36
        or not isinstance(input_records, Mapping)
        or len(input_records) != 21
    ):
        raise V7EarlyContactPreflightError("V3 runtime closure is incomplete")
    live_sources: dict[str, dict[str, Any]] = {}
    drifted_sources: list[str] = []
    for name, frozen_record in source_records.items():
        if (
            not isinstance(name, str)
            or not isinstance(frozen_record, Mapping)
            or set(frozen_record) != {"path", "sha256", "size_bytes"}
            or not isinstance(frozen_record.get("path"), str)
        ):
            raise V7EarlyContactPreflightError(
                f"V3 runtime source record malformed: {name!r}"
            )
        path = resolve_relative(str(frozen_record["path"]))
        live_record = source_record(path)
        if live_record["path"] != frozen_record["path"]:
            raise V7EarlyContactPreflightError(
                f"V3 runtime source path drifted: {name}"
            )
        if live_record != dict(frozen_record):
            drifted_sources.append(name)
        live_sources[name] = live_record
    expected_source_drift = {
        "actor_fit",
        "binary_phase_adapter",
        "phase_fsm",
        "training_config",
    }
    if set(drifted_sources) != expected_source_drift:
        raise V7EarlyContactPreflightError(
            "V3-to-V7 runtime source drift set is not the audited lineage: "
            f"{sorted(drifted_sources)}"
        )
    verified_inputs: dict[str, dict[str, Any]] = {}
    for name, frozen_record in input_records.items():
        if (
            not isinstance(name, str)
            or not isinstance(frozen_record, Mapping)
            or set(frozen_record) != {"path", "sha256", "size_bytes"}
            or not isinstance(frozen_record.get("path"), str)
        ):
            raise V7EarlyContactPreflightError(
                f"V3 runtime input record malformed: {name!r}"
            )
        path = resolve_relative(str(frozen_record["path"]))
        if not _record_matches(frozen_record, path):
            raise V7EarlyContactPreflightError(
                f"runtime input changed after V3 freeze: {name}"
            )
        verified_inputs[name] = dict(frozen_record)
    return {
        "v3_execution_lock": source_record(lock_path),
        "runtime_source_path_inventory": "V3_FROZEN_PATHS_V7_LIVE_BYTES",
        "runtime_sources": live_sources,
        "runtime_source_count": len(live_sources),
        "v3_to_v7_source_drift": sorted(drifted_sources),
        "runtime_inputs": verified_inputs,
        "runtime_input_count": len(verified_inputs),
        "runtime_inputs_v3_byte_exact": True,
    }


def validate_v5_nominal() -> dict[str, Any]:
    paths = input_paths()
    trace_path = paths["v5_nominal_trace"]
    summary_path = paths["v5_nominal_summary"]
    receipt_path = paths["v5_nominal_receipt"]
    trace = _sequence(trace_path)
    summary = _mapping(summary_path)
    receipt = _mapping(receipt_path)
    artifacts = receipt.get("artifacts")
    if (
        receipt.get("schema_version") != 5
        or receipt.get("status")
        != "PASS_H0_PRIMARY_SPLIT_V5_QUALIFICATION_ROLLOUT"
        or receipt.get("passed") is not True
        or receipt.get("role") != "baseline"
        or receipt.get("case_id") != "deterministic_offset_nominal"
        or receipt.get("actor_updates") != 0
        or receipt.get("critic_updates") != 0
        or receipt.get("ppo_updates") != 0
        or receipt.get("protected_trials_opened") != []
        or not isinstance(artifacts, Mapping)
        or not _record_matches(artifacts.get("trace.json"), trace_path)
        or not _record_matches(artifacts.get("summary.json"), summary_path)
    ):
        raise V7EarlyContactPreflightError("V5 nominal receipt drifted")
    if (
        summary.get("schema_version") != 5
        or summary.get("role") != "baseline"
        or summary.get("case_id") != "deterministic_offset_nominal"
        or summary.get("actor_input_view") != "historical_analog"
        or summary.get("event_contract_id")
        != "primary_grf_split_v1+legacy_events_v1"
        or summary.get("binary_phase_fsm_mode") != "disabled"
        or summary.get("phase_fsm_input_mode") != "legacy_events"
        or summary.get("action_selection") != "deterministic"
        or summary.get("action_seed") is not None
        or summary.get("runtime_seed") != contract.RUNTIME_SEED
        or summary.get("episode_start_offset_s")
        != contract.EPISODE_START_OFFSET_S
        or summary.get("steps") != contract.EXPECTED_STEPS
        or summary.get("end_reason") != "episode_time_limit"
        or summary.get("terminated") is not False
        or summary.get("truncated") is not True
        or summary.get("morphology_weight") != 0.0
        or summary.get("phase_valid_cycle_count")
        != contract.EXPECTED_LEGACY_PHASE_VALID_CYCLE_COUNT
        or summary.get("invalid_event_count")
        != contract.EXPECTED_LEGACY_PHASE_INVALID_EVENT_COUNT
        or summary.get("actor_updates") != 0
        or summary.get("critic_updates") != 0
        or summary.get("ppo_updates") != 0
        or summary.get("protected_trials_opened") != []
    ):
        raise V7EarlyContactPreflightError("V5 nominal summary drifted")
    expected_fields = {
        "step",
        "time_s",
        "actor_input_view",
        "actor_observation",
        "mean_action",
        "standard_normal",
        "raw_action",
        "reward",
        "terminated",
        "truncated",
    }
    if len(trace) != contract.EXPECTED_STEPS:
        raise V7EarlyContactPreflightError("V5 nominal trace length drifted")
    for index, raw_row in enumerate(trace, start=1):
        if not isinstance(raw_row, Mapping):
            raise V7EarlyContactPreflightError(
                f"V5 nominal row {index} is malformed"
            )
        row = dict(raw_row)
        expected_time = contract.EPISODE_START_TIME_S + (
            index * contract.EXPECTED_POLICY_DT_S
        )
        if (
            set(row) != expected_fields
            or row.get("step") != index
            or row.get("actor_input_view") != "historical_analog"
            or not _finite_vector(row.get("actor_observation"), 35)
            or not _finite_vector(row.get("mean_action"), 2)
            or not _finite_vector(row.get("standard_normal"), 2)
            or row.get("standard_normal") != [0.0, 0.0]
            or not _finite_vector(row.get("raw_action"), 2)
            or isinstance(row.get("time_s"), bool)
            or not isinstance(row.get("time_s"), (int, float))
            or not math.isfinite(float(row["time_s"]))
            or abs(float(row["time_s"]) - expected_time)
            > contract.TIME_TOLERANCE_S
            or isinstance(row.get("reward"), bool)
            or not isinstance(row.get("reward"), (int, float))
            or not math.isfinite(float(row["reward"]))
            or type(row.get("terminated")) is not bool
            or type(row.get("truncated")) is not bool
        ):
            raise V7EarlyContactPreflightError(
                f"V5 nominal row schema drifted: {index}"
            )
    if trace[-1]["terminated"] is not False or trace[-1]["truncated"] is not True:
        raise V7EarlyContactPreflightError("V5 nominal terminal row drifted")
    return {
        "case_id": contract.CASE_ID,
        "source_case_id": "deterministic_offset_nominal",
        "rows": len(trace),
        "trace": source_record(trace_path),
        "summary": source_record(summary_path),
        "receipt": source_record(receipt_path),
        "actor_input_view": "historical_analog",
        "source_event_contract_id": "primary_grf_split_v1+legacy_events_v1",
        "reclassification": "V5_PASS_BASELINE_TO_V7_DEVELOPMENT_DIAGNOSTIC",
    }


def _decode_v6_error(message: Any) -> dict[str, Any]:
    prefix = "Actor FSM rejected a V20 active event: "
    if not isinstance(message, str) or not message.startswith(prefix):
        raise V7EarlyContactPreflightError("V6 failure message drifted")
    try:
        value = json.loads(message[len(prefix) :])
    except json.JSONDecodeError as exc:
        raise V7EarlyContactPreflightError(
            "V6 failure context is not strict JSON"
        ) from exc
    forensic.canonical_json_bytes(value)
    if not isinstance(value, Mapping):
        raise V7EarlyContactPreflightError("V6 failure context is not an object")
    return dict(value)


def validate_v6_terminal() -> dict[str, Any]:
    paths = input_paths()
    ledger_path = paths["v6_terminal_ledger"]
    lock_path = paths["v6_execution_lock"]
    failure_path = paths["v6_nominal_failure"]
    last_step_path = paths["v6_nominal_last_step"]
    ledger = _mapping(ledger_path)
    lock = _mapping(lock_path)
    failure = _mapping(failure_path)
    last_step = _mapping(last_step_path)
    if (
        ledger.get("schema_version") != 6
        or ledger.get("status")
        != "FAIL_H0_V6_V25_TEACHER_REPLAY_DEVELOPMENT"
        or ledger.get("passed") is not False
        or ledger.get("completed_cases")
        != ["deterministic_offset_minus_0p20"]
        or ledger.get("next_stage") != "STOP_WITHOUT_RETRY_OR_RETUNING"
        or ledger.get("actor_updates") != 0
        or ledger.get("critic_updates") != 0
        or ledger.get("ppo_updates") != 0
        or ledger.get("protected_trials_opened") != []
        or not _record_matches(ledger.get("execution_lock"), lock_path)
    ):
        raise V7EarlyContactPreflightError("V6 terminal ledger drifted")
    if (
        lock.get("schema_version") != 6
        or lock.get("status")
        != "H0_PRIMARY_SPLIT_V6_V25_TEACHER_REPLAY_FROZEN"
        or lock.get("actor_updates") != 0
        or lock.get("critic_updates") != 0
        or lock.get("ppo_updates") != 0
        or lock.get("protected_trials_opened") != []
        or not isinstance(lock.get("inputs"), Mapping)
    ):
        raise V7EarlyContactPreflightError("V6 execution lock drifted")
    live_sources = validate_v6_live_source_closure(lock)
    if (
        failure.get("status") != "FAIL_H0_V6_V25_TEACHER_REPLAY"
        or failure.get("passed") is not False
        or failure.get("last_completed_step") != 457
        or failure.get("end_reason") != "worker_exception"
        or not isinstance(failure.get("error"), Mapping)
        or failure["error"].get("type") != "ValueError"
        or not isinstance(failure.get("details"), Mapping)
        or failure["details"].get("actor_updates") != 0
        or failure["details"].get("critic_updates") != 0
        or failure["details"].get("ppo_updates") != 0
        or failure["details"].get("protected_trials_opened") != []
    ):
        raise V7EarlyContactPreflightError("V6 nominal failure drifted")
    context = _decode_v6_error(failure["error"].get("message"))
    events = context.get("adapted_events")
    if (
        context.get("invalid_event_type") != "hs_too_early_after_to"
        or context.get("state_name") != "SWING_AFTER_TO"
        or not isinstance(events, list)
        or len(events) != 1
        or not isinstance(events[0], Mapping)
        or events[0].get("event") != "heel_strike"
        or abs(
            float(events[0].get("event_time_s", math.nan))
            - contract.V6_EARLY_HS_ONSET_TIME_S
        )
        > contract.TIME_TOLERANCE_S
    ):
        raise V7EarlyContactPreflightError("V6 early-event context drifted")
    binary = last_step.get("binary_phase_fsm")
    phase = last_step.get("phase_fsm")
    samples = last_step.get("binary_phase_sensor_samples")
    if (
        last_step.get("step") != 457
        or not isinstance(binary, Mapping)
        or not isinstance(phase, Mapping)
        or not isinstance(samples, list)
        or len(samples) != 10
        or binary.get("last_event") != "toe_off"
        or abs(
            float(binary.get("last_to_event_time_s", math.nan))
            - contract.V6_PRECEDING_TO_TIME_S
        )
        > contract.TIME_TOLERANCE_S
        or not isinstance(binary.get("pending_event"), Mapping)
        or binary["pending_event"].get("event") != "heel_strike"
        or binary["pending_event"].get("onset_contact_state") != "TOE"
        or abs(
            float(binary["pending_event"].get("event_time_s", math.nan))
            - contract.V6_EARLY_HS_ONSET_TIME_S
        )
        > contract.TIME_TOLERANCE_S
        or phase.get("state_name") != "SWING_AFTER_TO"
        or abs(
            float(phase.get("swing_elapsed_s", math.nan))
            - (float(last_step["runtime_time_s"]) - contract.V6_PRECEDING_TO_TIME_S)
        )
        > contract.TIME_TOLERANCE_S
    ):
        raise V7EarlyContactPreflightError("V6 last completed step drifted")
    onset_rows = [
        row
        for row in samples
        if isinstance(row, Mapping)
        and abs(
            float(row.get("time_s", math.nan))
            - contract.V6_EARLY_HS_ONSET_TIME_S
        )
        <= contract.TIME_TOLERANCE_S
    ]
    if (
        len(onset_rows) != 1
        or onset_rows[0].get("left_heel_contact") is not False
        or onset_rows[0].get("left_toe_contact") is not True
        or not (
            contract.MIN_SWING_CANDIDATE_S
            <= contract.V6_OBSERVED_SWING_S
            < contract.LEGACY_MIN_SWING_DURATION_S
        )
    ):
        raise V7EarlyContactPreflightError("V6 toe-only onset evidence drifted")
    lock_inputs = lock["inputs"]
    linked_inputs = {
        "source_h0_config": paths["source_h0_config"],
        "source_h0_module_state": paths["source_h0_module_state"],
        "source_h0_module_ctor": paths["source_h0_module_ctor"],
        "source_h0_module_metadata": paths["source_h0_module_metadata"],
        "v25_profile": paths["v25_profile"],
        "primary_grf_profile": paths["primary_grf_profile"],
        "analog_teacher_profile": paths["analog_teacher_profile"],
    }
    for name, path in linked_inputs.items():
        if not _record_matches(lock_inputs.get(name), path):
            raise V7EarlyContactPreflightError(
                f"V6 frozen input lineage drifted: {name}"
            )
    return {
        "status": ledger["status"],
        "ledger": source_record(ledger_path),
        "execution_lock": source_record(lock_path),
        "nominal_failure": source_record(failure_path),
        "nominal_last_completed_step": source_record(last_step_path),
        "last_completed_step": 457,
        "invalid_event_type": context["invalid_event_type"],
        "preceding_to_time_s": contract.V6_PRECEDING_TO_TIME_S,
        "early_hs_onset_time_s": contract.V6_EARLY_HS_ONSET_TIME_S,
        "observed_swing_s": contract.V6_OBSERVED_SWING_S,
        "early_onset_contact_state": "TOE",
        "terminal_failure_preserved": True,
        "retry_authorized": False,
        "live_runtime_sources": live_sources,
    }


def validate_frozen_runtime_inputs() -> dict[str, Any]:
    paths = input_paths()
    v25_freeze = _mapping(paths["v25_candidate_freeze"])
    candidate = v25_freeze.get("candidate")
    governance = v25_freeze.get("data_governance")
    if (
        v25_freeze.get("status")
        != "V25_DEVELOPMENT_CANDIDATE_FROZEN_H0_PROTOCOL_REQUIRED"
        or v25_freeze.get("pass") is not True
        or not isinstance(candidate, Mapping)
        or candidate.get("candidate_id") != "v25_4b351f67b5b86ab0"
        or not _record_matches(candidate.get("profile"), paths["v25_profile"])
        or not isinstance(governance, Mapping)
        or governance.get("protected_trials_opened") != []
        or governance.get("reserve_trials_opened") != []
    ):
        raise V7EarlyContactPreflightError("V25 frozen candidate drifted")
    primary_record = source_record(paths["primary_grf_profile"])
    if (
        primary_record["sha256"]
        != "09e04ab94954703d74acc3a80b24ecefcc07d3fc918c03b9e9df8116a6c1a2b0"
    ):
        raise V7EarlyContactPreflightError("primary GRF frozen profile drifted")
    return {
        "source_h0_id": contract.SOURCE_H0_ID,
        "source_h0_config": source_record(paths["source_h0_config"]),
        "source_h0_module_state": source_record(paths["source_h0_module_state"]),
        "source_h0_module_ctor": source_record(paths["source_h0_module_ctor"]),
        "source_h0_module_metadata": source_record(
            paths["source_h0_module_metadata"]
        ),
        "v25_candidate_id": candidate["candidate_id"],
        "v25_candidate_freeze": source_record(paths["v25_candidate_freeze"]),
        "v25_profile": source_record(paths["v25_profile"]),
        "binary_phase_fsm_mode": "binary_shadow",
        "shadow_event_contract_id": contract.SHADOW_EVENT_CONTRACT_ID,
        "primary_grf_profile": primary_record,
        "analog_teacher_profile": source_record(paths["analog_teacher_profile"]),
        "primary_load_contract_id": contract.PRIMARY_LOAD_CONTRACT_ID,
        "primary_load_evidence_role": contract.PRIMARY_LOAD_EVIDENCE_ROLE,
        "canonical_scientific_oracle": contract.CANONICAL_SCIENTIFIC_ORACLE,
        "primary_online_grf_used_as_event_source": False,
    }


def build_payload(*, require_destinations_absent: bool = True) -> dict[str, Any]:
    sources = source_paths()
    inputs = input_paths()
    source_records = {name: source_record(path) for name, path in sources.items()}
    input_records = _expected_input_hashes(inputs)
    v5 = validate_v5_nominal()
    v6 = validate_v6_terminal()
    runtime = validate_frozen_runtime_inputs()
    runtime_closure = validate_v3_runtime_closure()
    platform_attestation = validate_platform_identity()
    platform_record = dict(platform_attestation["identity"])
    primary_core = validate_primary_core()
    checks = {
        "all_sources_present_and_hashed": len(source_records)
        == len(contract.SOURCE_RELATIVE_PATHS),
        "all_inputs_present_and_hashed": len(input_records)
        == len(contract.INPUT_RELATIVE_PATHS),
        "preregistered_input_hashes_exact": all(
            input_records[name]["sha256"] == digest
            for name, digest in contract.EXPECTED_INPUT_SHA256.items()
        ),
        "v5_nominal_actions_frozen": v5["rows"] == contract.EXPECTED_STEPS,
        "v5_actor_remains_historical_analog": v5["actor_input_view"]
        == "historical_analog",
        "v6_terminal_fail_preserved": v6["terminal_failure_preserved"] is True
        and v6["retry_authorized"] is False,
        "v6_live_runtime_source_closure_exact": len(
            v6["live_runtime_sources"]
        )
        == 26,
        "transitive_runtime_sources_closed_live_v7": runtime_closure[
            "runtime_source_count"
        ]
        == 36,
        "runtime_inputs_v3_byte_exact": runtime_closure[
            "runtime_input_count"
        ]
        == 21
        and runtime_closure["runtime_inputs_v3_byte_exact"] is True,
        "platform_identity_and_packages_exact": platform_record
        == contract.EXPECTED_PLATFORM_IDENTITY,
        "platform_binaries_live_exact": len(
            platform_attestation["binary_artifacts"]
        )
        == 2,
        "primary_scientific_core_live_exact": len(
            primary_core["scientific_core"]
        )
        == 6,
        "primary_macos_dylib_live_exact": primary_core[
            "macos_arm64_dylib"
        ]["sha256"]
        == "5597a59a5368825fd207753b59291e72240c1ece3aa5280804f1e9b9c7d6a2b3",
        "known_early_pair_bound": abs(
            float(v6["observed_swing_s"]) - contract.V6_OBSERVED_SWING_S
        )
        <= contract.TIME_TOLERANCE_S,
        "v25_shadow_only": runtime["binary_phase_fsm_mode"] == "binary_shadow",
        "primary_is_offline_corroboration_only": runtime[
            "primary_load_evidence_role"
        ]
        == contract.PRIMARY_LOAD_EVIDENCE_ROLE
        and runtime["primary_online_grf_used_as_event_source"] is False,
        "canonical_scientific_oracle_preserved": runtime[
            "canonical_scientific_oracle"
        ]
        == contract.CANONICAL_SCIENTIFIC_ORACLE,
        "diagnostic_authorized": contract.AUTHORITY[
            "development_diagnostic_authorized"
        ],
        "binary_active_forbidden": not contract.AUTHORITY[
            "binary_active_execution_authorized"
        ],
        "updates_forbidden": not any(
            contract.AUTHORITY[name]
            for name in (
                "actor_updates_authorized",
                "critic_updates_authorized",
                "ppo_updates_authorized",
            )
        ),
        "protected_closed": not contract.AUTHORITY[
            "protected_trial_access_authorized"
        ],
        "reserve_closed": not contract.AUTHORITY[
            "reserve_trial_access_authorized"
        ],
        "macos_arm64_numeric_scope": platform_record["system"]
        == contract.EXPECTED_PLATFORM_SYSTEM
        and platform_record["machine"] == contract.EXPECTED_PLATFORM_MACHINE,
        "destination_unoccupied": not os.path.lexists(DESTINATION),
        "run_root_unoccupied": not os.path.lexists(RUN_ROOT),
        "execution_lock_unoccupied": not os.path.lexists(LOCK_PATH),
    }
    if require_destinations_absent and not all(checks.values()):
        failed = [name for name, passed in checks.items() if not passed]
        raise V7EarlyContactPreflightError(
            f"V7 early-contact preflight failed: {failed}"
        )
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.PREFLIGHT_PASS_STATUS,
        "passed": all(checks.values()),
        "protocol_id": contract.PROTOCOL_ID,
        "collector_id": contract.COLLECTOR_ID,
        "revision": contract.REVISION,
        "checks": checks,
        "case": dict(contract.CASE),
        "destination": DESTINATION.relative_to(REPO_ROOT).as_posix(),
        "v5_nominal": v5,
        "v6_terminal": v6,
        "frozen_runtime": runtime,
        "runtime_closure": runtime_closure,
        "platform": platform_record,
        "platform_attestation": platform_attestation,
        "primary_core": primary_core,
        "sources": source_records,
        "inputs": input_records,
        "authority": dict(contract.AUTHORITY),
        "simulations_executed": 0,
        "candidate_created": False,
        "candidate_implemented": False,
        "runtime_promoted": False,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
        "next_stage": "FREEZE_V7_EARLY_CONTACT_SHADOW_DIAGNOSTIC",
    }


def publish() -> dict[str, Any]:
    payload = build_payload(require_destinations_absent=True)
    forensic.write_json_exclusive(PREFLIGHT_PATH, payload)
    return payload


def main() -> int:
    try:
        result = publish()
    except Exception as exc:
        print(
            f"V7 early-contact preflight failed: {type(exc).__name__}: {exc}",
            file=sys.stderr,
        )
        return 2
    print(json.dumps(result, indent=2, sort_keys=True, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
