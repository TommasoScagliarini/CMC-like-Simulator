"""Run the single frozen H0/V25 early-contact diagnostic in shadow mode.

The protocol is diagnostic-only.  The original H0 actor remains on the legacy
observation/event path and the exact nominal V5 raw actions are replayed.  V25
and V20 execute only in ``binary_shadow`` so their 1 ms bit stream and events
cannot affect observations, actions, rewards, or dynamics.  Every completed
step is durably published before the scientific gate is evaluated.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import secrets
import subprocess
import sys
import time
from pathlib import Path
from typing import Any, Mapping, Sequence


REPO_ROOT = Path(__file__).resolve().parents[1]
VALIDATION_ROOT = REPO_ROOT / "validation"
TRAJECTORY_ROOT = REPO_ROOT / "Trajectory Generator"
BASELINE_ROOT = TRAJECTORY_ROOT / "baseline_MLP"
for import_root in (VALIDATION_ROOT, BASELINE_ROOT, TRAJECTORY_ROOT, REPO_ROOT):
    if str(import_root) not in sys.path:
        sys.path.insert(0, str(import_root))

import build_h0_primary_split_v7_early_contact_preflight as preflight  # noqa: E402
import freeze_h0_primary_split_v7_early_contact as freezer  # noqa: E402
import h0_forensic_rollout as forensic  # noqa: E402
import h0_primary_grf_split_v6_teacher_replay_contract as v6_contract  # noqa: E402
import h0_primary_split_v7_early_contact_contract as contract  # noqa: E402
import h0_v3_so_recovery_contract as so_recovery  # noqa: E402
import run_h0_v25_abc_preflight as legacy  # noqa: E402
from primary_grf_split_adaptation import array_sha256  # noqa: E402


LOCK = preflight.resolve_relative(contract.LOCK_PATH)
PREFLIGHT_RECEIPT = preflight.resolve_relative(contract.PREFLIGHT_RECEIPT_PATH)
RUN_ROOT = preflight.resolve_relative(contract.RUN_ROOT)
EXECUTION_LEDGER = preflight.resolve_relative(contract.EXECUTION_LEDGER_PATH)
EXECUTION_CLAIM = preflight.resolve_relative(contract.EXECUTION_CLAIM_PATH)
WORKER_CLAIMS_ROOT = preflight.resolve_relative(contract.WORKER_CLAIMS_ROOT)
DESTINATION = RUN_ROOT / contract.CASE_ID
SOURCE_H0_CONFIG = preflight.resolve_relative(
    contract.INPUT_RELATIVE_PATHS["source_h0_config"]
)
SOURCE_H0_MODULE = preflight.resolve_relative(
    contract.INPUT_RELATIVE_PATHS["source_h0_module_state"]
).parent
SOURCE_H0_CTOR = preflight.resolve_relative(
    contract.INPUT_RELATIVE_PATHS["source_h0_module_ctor"]
)
SOURCE_H0_METADATA = preflight.resolve_relative(
    contract.INPUT_RELATIVE_PATHS["source_h0_module_metadata"]
)
V25_PROFILE = preflight.resolve_relative(
    contract.INPUT_RELATIVE_PATHS["v25_profile"]
)
ANALOG_PROFILE = preflight.resolve_relative(
    contract.INPUT_RELATIVE_PATHS["analog_teacher_profile"]
)
BASELINE_TRACE = preflight.resolve_relative(
    contract.INPUT_RELATIVE_PATHS["v5_nominal_trace"]
)
WORKER_TIMEOUT_S = 2400.0
TIME_TOLERANCE_S = 1.0e-12
PRIMARY_ALIGNMENT_ID = contract.PRIMARY_LOAD_ALIGNMENT_ID


class V7EarlyContactExecutionError(RuntimeError):
    """Raised on any provenance, runtime, persistence, or gate failure."""


def _mapping(path: Path) -> dict[str, Any]:
    value = forensic.strict_json_load(path)
    if not isinstance(value, Mapping):
        raise V7EarlyContactExecutionError(f"expected JSON object: {path}")
    return dict(value)


def _sequence(path: Path) -> list[Any]:
    value = forensic.strict_json_load(path)
    if isinstance(value, (str, bytes)) or not isinstance(value, Sequence):
        raise V7EarlyContactExecutionError(f"expected JSON array: {path}")
    return list(value)


def _record(path: Path) -> dict[str, Any]:
    return preflight.source_record(path)


def _record_matches(record: Any, path: Path) -> bool:
    return (
        isinstance(record, Mapping)
        and set(record) == {"path", "sha256", "size_bytes"}
        and dict(record) == _record(path)
    )


def _token_sha256(token: str) -> str:
    if not isinstance(token, str) or len(token) < 32:
        raise V7EarlyContactExecutionError("execution token is malformed")
    return hashlib.sha256(token.encode("utf-8")).hexdigest()


def _payload_sha256(payload: Any) -> str:
    return hashlib.sha256(forensic.canonical_json_bytes(payload)).hexdigest()


def _execution_claim_payload(token_sha256: str) -> dict[str, Any]:
    if len(token_sha256) != 64:
        raise V7EarlyContactExecutionError("execution token digest is malformed")
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": "H0_V7_EARLY_CONTACT_EXECUTION_CLAIMED",
        "protocol_id": contract.PROTOCOL_ID,
        "collector_id": contract.COLLECTOR_ID,
        "execution_token_sha256": token_sha256,
        "execution_lock": _record(LOCK),
        "case_id": contract.CASE_ID,
        "authority": dict(contract.AUTHORITY),
        "candidate_created": False,
        "candidate_implemented": False,
        "runtime_promoted": False,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
    }


def _worker_claim_payload(token_sha256: str) -> dict[str, Any]:
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": "H0_V7_EARLY_CONTACT_WORKER_CLAIMED",
        "protocol_id": contract.PROTOCOL_ID,
        "collector_id": contract.COLLECTOR_ID,
        "case_id": contract.CASE_ID,
        "execution_token_sha256": token_sha256,
        "execution_lock": _record(LOCK),
        "execution_claim": _record(EXECUTION_CLAIM),
        "destination": DESTINATION.relative_to(REPO_ROOT).as_posix(),
        "predecessor_receipts": [],
        "authority": dict(contract.AUTHORITY),
        "candidate_created": False,
        "candidate_implemented": False,
        "runtime_promoted": False,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
    }


def worker_claim_path() -> Path:
    return WORKER_CLAIMS_ROOT / f"01_{contract.CASE_ID}.json"


def _verify_live_runtime_sources(
    lock: Mapping[str, Any],
    receipt: Mapping[str, Any],
) -> None:
    lock_v6 = lock.get("v6_terminal")
    receipt_v6 = receipt.get("v6_terminal")
    if (
        not isinstance(lock_v6, Mapping)
        or not isinstance(receipt_v6, Mapping)
        or forensic.canonical_json_bytes(lock_v6)
        != forensic.canonical_json_bytes(receipt_v6)
    ):
        raise V7EarlyContactExecutionError("V6 terminal closure drifted in V7 lock")
    records = lock_v6.get("live_runtime_sources")
    if not isinstance(records, Mapping) or len(records) != 26:
        raise V7EarlyContactExecutionError("V6 live runtime closure is incomplete")
    for name, record in records.items():
        if not isinstance(record, Mapping) or not isinstance(
            record.get("path"), str
        ):
            raise V7EarlyContactExecutionError(
                f"V6 live runtime record malformed: {name!r}"
            )
        path = preflight.resolve_relative(str(record["path"]))
        if not _record_matches(record, path):
            raise V7EarlyContactExecutionError(
                f"runtime source changed after V7 freeze: {name}"
            )


def _verify_transitive_runtime_closure(
    lock: Mapping[str, Any],
    receipt: Mapping[str, Any],
) -> None:
    lock_closure = lock.get("runtime_closure")
    receipt_closure = receipt.get("runtime_closure")
    if (
        not isinstance(lock_closure, Mapping)
        or not isinstance(receipt_closure, Mapping)
        or forensic.canonical_json_bytes(lock_closure)
        != forensic.canonical_json_bytes(receipt_closure)
    ):
        raise V7EarlyContactExecutionError(
            "V3 transitive runtime closure drifted in V7 lock"
        )
    live_closure = preflight.validate_v3_runtime_closure()
    if forensic.canonical_json_bytes(lock_closure) != forensic.canonical_json_bytes(
        live_closure
    ):
        raise V7EarlyContactExecutionError(
            "transitive runtime bytes changed after V7 freeze"
        )


def _verify_platform_and_primary_closure(
    lock: Mapping[str, Any],
    receipt: Mapping[str, Any],
) -> None:
    for key, live in (
        ("platform_attestation", preflight.validate_platform_identity()),
        ("primary_core", preflight.validate_primary_core()),
    ):
        if (
            lock.get(key) != receipt.get(key)
            or forensic.canonical_json_bytes(lock.get(key))
            != forensic.canonical_json_bytes(live)
        ):
            raise V7EarlyContactExecutionError(
                f"V7 {key} closure changed after freeze"
            )


def _verify_canonical_lock_payload(
    lock: Mapping[str, Any],
    receipt: Mapping[str, Any],
    *,
    preflight_receipt_record: Mapping[str, Any],
) -> None:
    """Reconstruct every lock field, including all static governance blocks."""

    try:
        expected = freezer.build_payload_from_receipt(
            receipt,
            preflight_receipt_record=preflight_receipt_record,
        )
        exact = forensic.canonical_json_bytes(lock) == forensic.canonical_json_bytes(
            expected
        )
    except (KeyError, TypeError, ValueError) as exc:
        raise V7EarlyContactExecutionError(
            "V7 execution lock cannot be reconstructed canonically"
        ) from exc
    if not exact:
        raise V7EarlyContactExecutionError(
            "V7 execution lock differs from the canonical freezer payload"
        )


def verify_lock(*, require_run_root_absent: bool = False) -> dict[str, Any]:
    lock = _mapping(LOCK)
    receipt = _mapping(PREFLIGHT_RECEIPT)
    preflight_receipt_record = _record(PREFLIGHT_RECEIPT)
    _verify_canonical_lock_payload(
        lock,
        receipt,
        preflight_receipt_record=preflight_receipt_record,
    )
    if (
        lock.get("schema_version") != contract.SCHEMA_VERSION
        or lock.get("status") != contract.LOCK_STATUS
        or lock.get("protocol_id") != contract.PROTOCOL_ID
        or lock.get("collector_id") != contract.COLLECTOR_ID
        or lock.get("revision") != contract.REVISION
        or lock.get("run_root") != contract.RUN_ROOT.as_posix()
        or lock.get("execution_ledger")
        != contract.EXECUTION_LEDGER_PATH.as_posix()
        or lock.get("execution_claim")
        != contract.EXECUTION_CLAIM_PATH.as_posix()
        or lock.get("worker_claims_root")
        != contract.WORKER_CLAIMS_ROOT.as_posix()
        or lock.get("case") != dict(contract.CASE)
        or lock.get("execution_order") != [contract.CASE_ID]
        or lock.get("authority") != contract.AUTHORITY
        or lock.get("simulations_executed_at_freeze") != 0
        or lock.get("candidate_created") is not False
        or lock.get("candidate_implemented") is not False
        or lock.get("runtime_promoted") is not False
        or lock.get("actor_updates") != 0
        or lock.get("critic_updates") != 0
        or lock.get("ppo_updates") != 0
        or lock.get("protected_trials_opened") != []
        or lock.get("reserve_trials_opened") != []
    ):
        raise V7EarlyContactExecutionError("V7 execution lock identity drifted")
    if (
        receipt.get("status") != contract.PREFLIGHT_PASS_STATUS
        or receipt.get("passed") is not True
        or not isinstance(receipt.get("checks"), Mapping)
        or not all(value is True for value in receipt["checks"].values())
        or receipt.get("actor_updates") != 0
        or receipt.get("critic_updates") != 0
        or receipt.get("ppo_updates") != 0
        or receipt.get("protected_trials_opened") != []
        or receipt.get("reserve_trials_opened") != []
        or receipt.get("candidate_created") is not False
        or receipt.get("candidate_implemented") is not False
        or receipt.get("runtime_promoted") is not False
        or lock.get("preflight_receipt") != preflight_receipt_record
    ):
        raise V7EarlyContactExecutionError("V7 preflight closure drifted")
    source_paths = preflight.source_paths()
    input_paths = preflight.input_paths()
    for label, paths in (("sources", source_paths), ("inputs", input_paths)):
        records = lock.get(label)
        if not isinstance(records, Mapping) or set(records) != set(paths):
            raise V7EarlyContactExecutionError(f"V7 {label} closure drifted")
        for name, path in paths.items():
            if not _record_matches(records[name], path):
                raise V7EarlyContactExecutionError(
                    f"V7 {label}.{name} record drifted"
                )
    if (
        lock.get("v5_nominal") != receipt.get("v5_nominal")
        or lock.get("frozen_runtime") != receipt.get("frozen_runtime")
        or lock.get("platform") != receipt.get("platform")
        or lock.get("platform") != preflight.platform_provenance()
    ):
        raise V7EarlyContactExecutionError("V7 frozen evidence closure drifted")
    _verify_live_runtime_sources(lock, receipt)
    _verify_transitive_runtime_closure(lock, receipt)
    _verify_platform_and_primary_closure(lock, receipt)
    if require_run_root_absent and os.path.lexists(RUN_ROOT):
        raise V7EarlyContactExecutionError(f"run root already claimed: {RUN_ROOT}")
    return lock


def verify_execution_claim(execution_token: str) -> dict[str, Any]:
    observed = _mapping(EXECUTION_CLAIM)
    expected = _execution_claim_payload(_token_sha256(execution_token))
    if forensic.canonical_json_bytes(observed) != forensic.canonical_json_bytes(
        expected
    ):
        raise V7EarlyContactExecutionError("execution claim/token drifted")
    return observed


def verify_worker_claim(execution_token: str) -> dict[str, Any]:
    execution_claim = verify_execution_claim(execution_token)
    expected = _worker_claim_payload(execution_claim["execution_token_sha256"])
    observed = _mapping(worker_claim_path())
    if forensic.canonical_json_bytes(observed) != forensic.canonical_json_bytes(
        expected
    ):
        raise V7EarlyContactExecutionError("worker claim/token drifted")
    return observed


def load_frozen_baseline() -> list[dict[str, Any]]:
    rows_raw = _sequence(BASELINE_TRACE)
    rows = [dict(row) for row in rows_raw if isinstance(row, Mapping)]
    if len(rows) != len(rows_raw) or len(rows) != contract.EXPECTED_STEPS:
        raise V7EarlyContactExecutionError("nominal V5 trace drifted")
    return rows


def exact_float32_vector(
    observed: Any,
    expected: Any,
    *,
    length: int,
    np: Any,
) -> tuple[bool, str, str]:
    left = np.ascontiguousarray(np.asarray(observed, dtype=np.float32))
    right = np.ascontiguousarray(np.asarray(expected, dtype=np.float32))
    if (
        left.shape != (length,)
        or right.shape != (length,)
        or not np.all(np.isfinite(left))
        or not np.all(np.isfinite(right))
    ):
        raise V7EarlyContactExecutionError("cannot compare malformed vectors")
    return (
        left.tobytes(order="C") == right.tobytes(order="C"),
        array_sha256(left),
        array_sha256(right),
    )


def build_env_config() -> dict[str, Any]:
    case = contract.CASE
    translated = {
        "id": contract.CASE_ID,
        "action_selection": case["action_selection"],
        "offset_s": case["episode_start_offset_s"],
        "seed": case["runtime_seed"],
    }
    previous_config = legacy.H0_CONFIG
    previous_profile = legacy.V25_PROFILE
    previous_analog = legacy.ANALOG_PROFILE
    try:
        legacy.H0_CONFIG = SOURCE_H0_CONFIG
        legacy.V25_PROFILE = V25_PROFILE
        legacy.ANALOG_PROFILE = ANALOG_PROFILE
        result = legacy.build_env_config(case_id="B", condition=translated)
    finally:
        legacy.H0_CONFIG = previous_config
        legacy.V25_PROFILE = previous_profile
        legacy.ANALOG_PROFILE = previous_analog
    if (
        result.get("binary_phase_fsm_mode") != "binary_shadow"
        or result.get("binary_phase_event_contract_id")
        != contract.SHADOW_EVENT_CONTRACT_ID
        or result.get("phase_fsm_input_mode") != "legacy_events"
        or result.get("event_contract_id") != "legacy_events_v1"
        or result.get("online_grf_applied_sides") != ["left"]
        or result.get("reward", {}).get("morphology_weight")
        != contract.MORPHOLOGY_WEIGHT
        or result.get("detector_sample_dt_s") != contract.EXPECTED_SAMPLE_DT_S
        or result.get("segment_duration") != contract.EXPECTED_POLICY_DT_S
        or result.get("episode_duration") != 5.0
    ):
        raise V7EarlyContactExecutionError("V7 shadow environment routing drifted")
    return result


def _sea_fallback_count(payload: Any) -> int:
    if not isinstance(payload, Mapping) or not isinstance(
        payload.get("joints"), Mapping
    ):
        raise V7EarlyContactExecutionError("SEA diagnostics are malformed")
    total = 0
    for joint in ("pros_knee_angle", "pros_ankle_angle"):
        values = payload["joints"].get(joint)
        if not isinstance(values, Mapping):
            raise V7EarlyContactExecutionError(f"SEA diagnostics missing {joint}")
        for field in (
            "tau_input_plugin_fallback_count",
            "motor_accel_plugin_fallback_count",
        ):
            value = values.get(field)
            if type(value) is not int or value < 0:
                raise V7EarlyContactExecutionError(
                    f"SEA fallback counter malformed: {joint}.{field}"
                )
            total += value
    return total


class _PrimaryGRFObservationTap:
    """Observe existing 1 ms primary-GRF calls without a second evaluation."""

    def __init__(self, runtime_runner: Any, *, body_weight_n: float) -> None:
        if not math.isfinite(body_weight_n) or body_weight_n <= 0.0:
            raise V7EarlyContactExecutionError("tap body weight is invalid")
        if "_sample_online_grf" in vars(runtime_runner):
            raise V7EarlyContactExecutionError(
                "primary sampler already has an instance override"
            )
        original = getattr(runtime_runner, "_sample_online_grf", None)
        if not callable(original):
            raise V7EarlyContactExecutionError("primary sampler is unavailable")
        self.runtime_runner = runtime_runner
        self.body_weight_n = float(body_weight_n)
        self.original = original
        self.original_self = getattr(original, "__self__", None)
        self.original_func = getattr(original, "__func__", None)
        self.samples: list[dict[str, Any]] = []
        self.wrapper_call_count = 0
        self.original_call_count = 0
        self.installed = False
        self.restored = False

        def observe(state: Any, time_s: float) -> Any:
            self.wrapper_call_count += 1
            result = self.original(state, time_s)
            self.original_call_count += 1
            if (
                not isinstance(result, tuple)
                or len(result) != 2
                or not isinstance(result[0], Mapping)
                or not isinstance(result[0].get("sides"), Mapping)
                or not isinstance(result[0]["sides"].get("left"), Mapping)
            ):
                raise V7EarlyContactExecutionError(
                    "observed primary GRF return is malformed"
                )
            left = result[0]["sides"]["left"]
            force_raw = left.get("normal_force")
            in_contact = left.get("in_contact")
            if (
                isinstance(time_s, bool)
                or not isinstance(time_s, (int, float))
                or not math.isfinite(float(time_s))
                or isinstance(force_raw, bool)
                or not isinstance(force_raw, (int, float))
                or not math.isfinite(float(force_raw))
                or float(force_raw) < 0.0
                or type(in_contact) is not bool
            ):
                raise V7EarlyContactExecutionError(
                    "observed primary left sample is invalid"
                )
            force_n = float(force_raw)
            self.samples.append(
                {
                    "sampled_time_s": float(time_s),
                    "left_normal_grf_bw": force_n / self.body_weight_n,
                    "left_normal_force_n": force_n,
                    "left_in_contact": in_contact,
                }
            )
            return result

        self.wrapper = observe

    def install(self) -> None:
        if self.installed or self.restored:
            raise V7EarlyContactExecutionError("primary tap install is not one-shot")
        setattr(self.runtime_runner, "_sample_online_grf", self.wrapper)
        if getattr(self.runtime_runner, "_sample_online_grf") is not self.wrapper:
            raise V7EarlyContactExecutionError("primary tap installation failed")
        self.installed = True

    def deliver_step(self, *, start_index: int, delivered_time_s: float) -> list[dict[str, Any]]:
        if not self.installed or self.restored:
            raise V7EarlyContactExecutionError("primary tap is not active")
        if type(start_index) is not int or start_index < 0:
            raise V7EarlyContactExecutionError("primary tap start index is invalid")
        observed = self.samples[start_index:]
        if len(observed) != contract.EXPECTED_PRIMARY_SAMPLES_PER_STEP:
            raise V7EarlyContactExecutionError(
                "primary tap did not observe exactly ten existing evaluations"
            )
        if (
            isinstance(delivered_time_s, bool)
            or not isinstance(delivered_time_s, (int, float))
            or not math.isfinite(float(delivered_time_s))
        ):
            raise V7EarlyContactExecutionError("primary delivery time is invalid")
        return [
            {**dict(sample), "delivered_time_s": float(delivered_time_s)}
            for sample in observed
        ]

    def restore(self) -> dict[str, Any]:
        if not self.installed or self.restored:
            raise V7EarlyContactExecutionError("primary tap restore is not one-shot")
        if getattr(self.runtime_runner, "_sample_online_grf") is not self.wrapper:
            raise V7EarlyContactExecutionError("primary tap identity drifted")
        delattr(self.runtime_runner, "_sample_online_grf")
        restored = getattr(self.runtime_runner, "_sample_online_grf", None)
        descriptor_exact = bool(
            callable(restored)
            and getattr(restored, "__self__", None) is self.original_self
            and getattr(restored, "__func__", None) is self.original_func
        )
        self.restored = True
        if not descriptor_exact:
            raise V7EarlyContactExecutionError(
                "primary sampler descriptor was not restored exactly"
            )
        return {
            "instrumentation_id": "primary_grf_existing_call_observer_v1",
            "installed_after_reset": True,
            "reset_call_count": 0,
            "wrapper_call_count": self.wrapper_call_count,
            "original_call_count": self.original_call_count,
            "one_original_call_per_wrapper_call": (
                self.wrapper_call_count == self.original_call_count
            ),
            "second_primary_evaluations": 0,
            "return_forwarded_unmodified": True,
            "restored_in_finally": True,
            "descriptor_identity_restored": descriptor_exact,
        }


def _minimal_raw_samples(samples: Sequence[Mapping[str, Any]]) -> list[dict[str, Any]]:
    return [
        {
            "time_s": sample["time_s"],
            "left_heel_contact": sample["left_heel_contact"],
            "left_toe_contact": sample["left_toe_contact"],
        }
        for sample in samples
    ]


def _validated_primary_tap_audit(value: Any) -> dict[str, Any]:
    """Require the exact one-call observational instrumentation receipt."""

    expected = {
        "instrumentation_id": "primary_grf_existing_call_observer_v1",
        "installed_after_reset": True,
        "reset_call_count": 0,
        "wrapper_call_count": contract.EXPECTED_PRIMARY_LOAD_SAMPLES,
        "original_call_count": contract.EXPECTED_PRIMARY_LOAD_SAMPLES,
        "one_original_call_per_wrapper_call": True,
        "second_primary_evaluations": 0,
        "return_forwarded_unmodified": True,
        "restored_in_finally": True,
        "descriptor_identity_restored": True,
    }
    if not isinstance(value, Mapping) or dict(value) != expected:
        raise V7EarlyContactExecutionError(
            "persisted primary observation tap audit is not exact"
        )
    return expected


def _audit_persisted_evidence(
    *,
    trace_path: Path,
    summary_path: Path,
    journal_path: Path,
) -> tuple[dict[str, Any], dict[str, Any]]:
    """Rebuild every scientific claim from strict, durable artifacts."""

    trace_raw = _sequence(trace_path)
    summary = _mapping(summary_path)
    journal = _mapping(journal_path)
    if len(trace_raw) != contract.EXPECTED_STEPS or not all(
        isinstance(row, Mapping) for row in trace_raw
    ):
        raise V7EarlyContactExecutionError("persisted trace is incomplete")
    body_weight_raw = journal.get("body_weight_n")
    if (
        isinstance(body_weight_raw, bool)
        or not isinstance(body_weight_raw, (int, float))
        or not math.isfinite(float(body_weight_raw))
        or float(body_weight_raw) <= 0.0
    ):
        raise V7EarlyContactExecutionError("persisted body weight is invalid")
    body_weight_n = float(body_weight_raw)
    tap_audit = _validated_primary_tap_audit(
        journal.get("primary_observation_tap")
    )
    projected: list[dict[str, Any]] = []
    flattened_raw: list[dict[str, Any]] = []
    flattened_primary: list[dict[str, Any]] = []
    flattened_events: list[dict[str, Any]] = []
    reconstructed_policy_steps: list[dict[str, Any]] = []
    penetrations: list[float] = []
    counter_totals = {
        "action_mismatch_count": 0,
        "actor_observation_mismatch_count": 0,
        "actor_mean_mismatch_count": 0,
        "teacher_std_mismatch_count": 0,
        "projected_trace_mismatch_count": 0,
        "time_mismatch_count": 0,
        "step_contract_failure_count": 0,
        "action_clipped_values": 0,
        "timeout_count": 0,
        "hard_invalid_count": 0,
        "nonfinite_count": 0,
        "so_solver_unaccepted_count": 0,
        "sea_plugin_fallback_count": 0,
        "routing_failure_count": 0,
        "control_window_count": 0,
    }
    expected_check_names = {
        "finite",
        "actor_observation_bit_exact",
        "actor_mean_bit_exact",
        "teacher_std_bit_exact",
        "frozen_action_unclipped",
        "projected_v5_row_bit_exact",
        "runtime_time_matches_v5",
        "shadow_routing_exact",
        "ten_raw_samples",
        "ten_control_windows",
        "no_unaccepted_so",
        "no_sea_fallback",
    }
    expected_stat_names = {
        "grf_penetration_m",
        "action_clipped_values",
        "timeout_count",
        "hard_invalid_count",
        "nonfinite_count",
        "routing_failure_count",
        "so_solver_unaccepted_count",
        "sea_plugin_fallback_count",
        "control_window_count",
        "runtime_end_reason",
        "runtime_failure_present",
    }
    final_phase: Mapping[str, Any] | None = None
    final_binary: Mapping[str, Any] | None = None
    final_projection: Mapping[str, Any] | None = None
    final_end_reason: Any = None
    for index, raw_row in enumerate(trace_raw, start=1):
        row = dict(raw_row)
        projection = row.get("projected_v5_row")
        checks = row.get("checks")
        stats = row.get("sufficient_statistics")
        samples = row.get("binary_phase_sensor_samples")
        primary_step = row.get("primary_load_samples")
        binary = row.get("binary_phase_fsm")
        phase = row.get("legacy_phase_fsm")
        so_counters = row.get("so_recovery_counters")
        digests = row.get("digests")
        if (
            row.get("step") != index
            or not isinstance(projection, Mapping)
            or not isinstance(checks, Mapping)
            or set(checks) != expected_check_names
            or not all(type(value) is bool for value in checks.values())
            or not isinstance(stats, Mapping)
            or set(stats) != expected_stat_names
            or not isinstance(samples, list)
            or len(samples) != contract.EXPECTED_SAMPLES_PER_STEP
            or not isinstance(primary_step, list)
            or len(primary_step) != contract.EXPECTED_PRIMARY_SAMPLES_PER_STEP
            or not isinstance(binary, Mapping)
            or not isinstance(binary.get("events_this_step"), list)
            or not isinstance(phase, Mapping)
            or not isinstance(so_counters, Mapping)
            or not isinstance(digests, Mapping)
            or set(digests)
            != {
                "actor_observation",
                "expected_actor_observation",
                "queried_mean",
                "expected_mean",
                "teacher_std",
                "expected_teacher_std",
                "applied_action",
                "frozen_raw_action",
            }
            or not all(
                isinstance(value, str)
                and len(value) == 64
                and all(character in "0123456789abcdef" for character in value)
                for value in digests.values()
            )
        ):
            raise V7EarlyContactExecutionError(
                f"persisted trace row is malformed: {index}"
            )
        integer_stat_names = expected_stat_names - {
            "grf_penetration_m",
            "runtime_end_reason",
            "runtime_failure_present",
        }
        if (
            any(
                type(stats.get(name)) is not int or int(stats[name]) < 0
                for name in integer_stat_names
            )
            or isinstance(stats.get("grf_penetration_m"), bool)
            or not isinstance(stats.get("grf_penetration_m"), (int, float))
            or not math.isfinite(float(stats["grf_penetration_m"]))
            or float(stats["grf_penetration_m"]) < 0.0
            or type(stats.get("runtime_failure_present")) is not bool
            or (
                stats.get("runtime_end_reason") is not None
                and not isinstance(stats.get("runtime_end_reason"), str)
            )
        ):
            raise V7EarlyContactExecutionError(
                f"persisted sufficient statistics are malformed: {index}"
            )
        so_control = so_counters.get("control_window_count")
        hard_so = so_counters.get("unaccepted_hard_so_fallback_count")
        bounded_so = so_counters.get("unaccepted_bounded_ls_count")
        so_unaccepted = (
            hard_so + bounded_so
            if type(hard_so) is int
            and hard_so >= 0
            and type(bounded_so) is int
            and bounded_so >= 0
            else None
        )
        timeout_raw = phase.get("timeout_exceeded", 0.0)
        if (
            type(so_control) is not int
            or so_control < 0
            or so_unaccepted is None
            or isinstance(timeout_raw, bool)
            or not isinstance(timeout_raw, (int, float))
            or not math.isfinite(float(timeout_raw))
            or stats["control_window_count"] != so_control
            or stats["so_solver_unaccepted_count"] != so_unaccepted
            or stats["timeout_count"] != int(float(timeout_raw) > 0.0)
            or stats["hard_invalid_count"]
            != int(stats["runtime_failure_present"])
            or stats["nonfinite_count"] != int(not checks["finite"])
            or stats["routing_failure_count"]
            != int(not checks["shadow_routing_exact"])
            or checks["frozen_action_unclipped"]
            is not (stats["action_clipped_values"] == 0)
            or checks["no_unaccepted_so"]
            is not (stats["so_solver_unaccepted_count"] == 0)
            or checks["no_sea_fallback"]
            is not (stats["sea_plugin_fallback_count"] == 0)
            or checks["ten_control_windows"]
            is not (stats["control_window_count"] == 10)
            or checks["ten_raw_samples"] is not True
            or checks["actor_observation_bit_exact"]
            is not (
                digests["actor_observation"]
                == digests["expected_actor_observation"]
            )
            or checks["actor_mean_bit_exact"]
            is not (digests["queried_mean"] == digests["expected_mean"])
            or checks["teacher_std_bit_exact"]
            is not (digests["teacher_std"] == digests["expected_teacher_std"])
            or digests["teacher_std"]
            != contract.EXPECTED_H0_POLICY_STD_ARRAY_SHA256
            or digests["expected_teacher_std"]
            != contract.EXPECTED_H0_POLICY_STD_ARRAY_SHA256
            or checks["frozen_action_unclipped"]
            is not (digests["applied_action"] == digests["frozen_raw_action"])
        ):
            raise V7EarlyContactExecutionError(
                f"persisted sufficient statistics disagree with row evidence: {index}"
            )
        projected.append(dict(projection))
        for sample_index, sample in enumerate(samples, start=1):
            if (
                not isinstance(sample, Mapping)
                or set(sample)
                != {
                    "time_s",
                    "left_heel_contact",
                    "left_toe_contact",
                }
                or isinstance(sample.get("time_s"), bool)
                or not isinstance(sample.get("time_s"), (int, float))
                or not math.isfinite(float(sample["time_s"]))
                or type(sample.get("left_heel_contact")) is not bool
                or type(sample.get("left_toe_contact")) is not bool
            ):
                raise V7EarlyContactExecutionError(
                    "persisted detector raw sample is malformed: "
                    f"{index}.{sample_index}"
                )
        for sample_index, primary in enumerate(primary_step, start=1):
            global_sample_index = (
                (index - 1) * contract.EXPECTED_PRIMARY_SAMPLES_PER_STEP
                + sample_index
                - 1
            )
            expected_sample_time = (
                contract.EPISODE_START_TIME_S
                + global_sample_index * contract.EXPECTED_SAMPLE_DT_S
            )
            expected_delivery_time = row.get("runtime_time_s")
            if (
                not isinstance(primary, Mapping)
                or set(primary)
                != {
                    "sampled_time_s",
                    "delivered_time_s",
                    "left_normal_grf_bw",
                    "left_normal_force_n",
                    "left_in_contact",
                }
                or any(
                    isinstance(primary.get(name), bool)
                    or not isinstance(primary.get(name), (int, float))
                    or not math.isfinite(float(primary[name]))
                    for name in (
                        "sampled_time_s",
                        "delivered_time_s",
                        "left_normal_grf_bw",
                        "left_normal_force_n",
                    )
                )
                or not math.isclose(
                    float(primary["sampled_time_s"]),
                    expected_sample_time,
                    rel_tol=0.0,
                    abs_tol=contract.TIME_TOLERANCE_S,
                )
                or primary.get("delivered_time_s") != expected_delivery_time
                or float(primary["delivered_time_s"])
                + contract.TIME_TOLERANCE_S
                < float(primary["sampled_time_s"])
                or not (
                    contract.EXPECTED_SAMPLE_DT_S
                    - contract.TIME_TOLERANCE_S
                    <= float(primary["delivered_time_s"])
                    - float(primary["sampled_time_s"])
                    <= contract.EXPECTED_POLICY_DT_S
                    + contract.TIME_TOLERANCE_S
                )
                or float(primary["left_normal_grf_bw"]) < 0.0
                or float(primary["left_normal_force_n"]) < 0.0
                or type(primary.get("left_in_contact")) is not bool
                or not math.isclose(
                    float(primary["left_normal_grf_bw"]) * body_weight_n,
                    float(primary["left_normal_force_n"]),
                    rel_tol=1.0e-10,
                    abs_tol=1.0e-9,
                )
                or (
                    float(primary["left_normal_force_n"])
                    > contract.PRIMARY_FORCE_THRESHOLD_N
                    and primary["left_in_contact"] is not True
                )
            ):
                raise V7EarlyContactExecutionError(
                    "persisted primary 1 ms sample is inconsistent: "
                    f"{index}.{sample_index}"
                )
        flattened_raw.extend(dict(sample) for sample in samples)
        flattened_primary.extend(dict(sample) for sample in primary_step)
        step_events = [dict(event) for event in binary["events_this_step"]]
        flattened_events.extend(step_events)
        reconstructed_policy_steps.append(
            {
                "step": index,
                "time_s": row.get("runtime_time_s"),
                "primary_load_samples": [dict(sample) for sample in primary_step],
                "legacy_online_events": row.get("legacy_online_events"),
                "shadow_events": step_events,
                "legacy_phase_fsm": dict(phase),
                "v20_shadow_fsm": dict(binary),
            }
        )
        penetrations.append(float(stats["grf_penetration_m"]))
        counter_totals["action_mismatch_count"] += int(
            not checks["frozen_action_unclipped"]
        )
        counter_totals["actor_observation_mismatch_count"] += int(
            not checks["actor_observation_bit_exact"]
        )
        counter_totals["actor_mean_mismatch_count"] += int(
            not checks["actor_mean_bit_exact"]
        )
        counter_totals["teacher_std_mismatch_count"] += int(
            not checks["teacher_std_bit_exact"]
        )
        counter_totals["projected_trace_mismatch_count"] += int(
            not checks["projected_v5_row_bit_exact"]
        )
        counter_totals["time_mismatch_count"] += int(
            not checks["runtime_time_matches_v5"]
        )
        counter_totals["step_contract_failure_count"] += int(
            not all(checks.values())
        )
        for name in (
            "action_clipped_values",
            "timeout_count",
            "hard_invalid_count",
            "nonfinite_count",
            "so_solver_unaccepted_count",
            "sea_plugin_fallback_count",
            "routing_failure_count",
            "control_window_count",
        ):
            counter_totals[name] += int(stats[name])
        final_phase = phase
        final_binary = binary
        final_projection = projection
        final_end_reason = stats["runtime_end_reason"]
    frozen_v5 = [dict(row) for row in _sequence(BASELINE_TRACE)]
    if (
        len(frozen_v5) != contract.EXPECTED_STEPS
        or forensic.canonical_json_bytes(projected)
        != forensic.canonical_json_bytes(frozen_v5)
        or summary.get("v5_projected_trace_bit_exact") is not True
        or summary.get("projected_v5_trace_sha256") != _payload_sha256(projected)
        or summary.get("baseline_trace_sha256") != _payload_sha256(frozen_v5)
    ):
        raise V7EarlyContactExecutionError(
            "persisted non-binary trace is not bit-exact to V5"
        )
    if (
        journal.get("schema_version") != contract.SCHEMA_VERSION
        or journal.get("protocol_id") != contract.PROTOCOL_ID
        or journal.get("case_id") != contract.CASE_ID
        or journal.get("sample_dt_s") != contract.EXPECTED_SAMPLE_DT_S
        or journal.get("policy_dt_s") != contract.EXPECTED_POLICY_DT_S
        or not isinstance(journal.get("primary_load_alignment"), Mapping)
        or journal["primary_load_alignment"].get("id")
        != contract.PRIMARY_LOAD_ALIGNMENT_ID
        or journal["primary_load_alignment"].get("source_cadence_s")
        != contract.EXPECTED_SAMPLE_DT_S
        or journal["primary_load_alignment"].get("delivery_cadence_s")
        != contract.EXPECTED_POLICY_DT_S
        or journal["primary_load_alignment"].get("interpolation") is not False
        or not isinstance(journal.get("raw_samples"), list)
        or len(journal["raw_samples"]) != contract.EXPECTED_RAW_SENSOR_SAMPLES
        or not isinstance(journal.get("primary_load_samples"), list)
        or len(journal["primary_load_samples"])
        != contract.EXPECTED_PRIMARY_LOAD_SAMPLES
        or not isinstance(journal.get("shadow_events"), list)
        or not isinstance(journal.get("policy_steps"), list)
        or not isinstance(journal.get("final_v20_payload"), Mapping)
        or not isinstance(journal.get("v20_final_state_audit"), Mapping)
        or not isinstance(journal.get("baseline_raw_sample"), Mapping)
        or journal.get("primary_observation_tap") != tap_audit
        or summary.get("platform") != preflight.platform_provenance()
    ):
        raise V7EarlyContactExecutionError("persisted raw journal schema drifted")
    if (
        trace_raw[0].get("binary_phase_sensor_baseline")
        != journal["baseline_raw_sample"]
        or any(
            row.get("binary_phase_sensor_baseline") is not None
            for row in trace_raw[1:]
        )
        or forensic.canonical_json_bytes(flattened_raw)
        != forensic.canonical_json_bytes(journal["raw_samples"])
        or forensic.canonical_json_bytes(flattened_primary)
        != forensic.canonical_json_bytes(journal["primary_load_samples"])
        or forensic.canonical_json_bytes(flattened_events)
        != forensic.canonical_json_bytes(journal["shadow_events"])
        or forensic.canonical_json_bytes(reconstructed_policy_steps)
        != forensic.canonical_json_bytes(journal["policy_steps"])
        or forensic.canonical_json_bytes(final_binary)
        != forensic.canonical_json_bytes(journal["final_v20_payload"])
        or summary.get("primary_observation_tap") != tap_audit
    ):
        raise V7EarlyContactExecutionError(
            "persisted journal is not reconstructed by the 500 step rows"
        )
    raw_samples = _minimal_raw_samples(journal["raw_samples"])
    rebuilt_v20 = contract.audit_v20_final_state(
        baseline_sample=journal["baseline_raw_sample"],
        policy_boundary_times=[row["time_s"] for row in journal["policy_steps"]],
        final_payload=journal["final_v20_payload"],
        raw_samples=raw_samples,
        shadow_events=journal["shadow_events"],
    )
    rebuilt = contract.classify_contact(
        baseline_sample=journal["baseline_raw_sample"],
        raw_samples=raw_samples,
        primary_load_samples=journal["primary_load_samples"],
        shadow_events=journal["shadow_events"],
        body_weight_n=journal.get("body_weight_n"),
    )
    if (
        forensic.canonical_json_bytes(rebuilt)
        != forensic.canonical_json_bytes(summary.get("contact_classification"))
        or not _record_matches(summary.get("v25_raw_journal"), journal_path)
        or summary.get("v25_raw_journal_payload_sha256")
        != _payload_sha256(journal)
        or forensic.canonical_json_bytes(rebuilt_v20)
        != forensic.canonical_json_bytes(journal["v20_final_state_audit"])
        or forensic.canonical_json_bytes(rebuilt_v20)
        != forensic.canonical_json_bytes(summary.get("v20_final_state_audit"))
    ):
        raise V7EarlyContactExecutionError(
            "persisted contact classification cannot be reproduced"
        )
    if final_phase is None or final_projection is None:
        raise V7EarlyContactExecutionError("persisted final step is absent")
    legacy_invalid = final_phase.get("invalid_event_count")
    legacy_cycles = final_phase.get("valid_cycle_count")
    if (
        isinstance(legacy_invalid, bool)
        or not isinstance(legacy_invalid, (int, float))
        or not math.isfinite(float(legacy_invalid))
        or int(float(legacy_invalid)) != float(legacy_invalid)
        or isinstance(legacy_cycles, bool)
        or not isinstance(legacy_cycles, (int, float))
        or not math.isfinite(float(legacy_cycles))
        or int(float(legacy_cycles)) != float(legacy_cycles)
    ):
        raise V7EarlyContactExecutionError("persisted legacy phase counters malformed")
    reconstructed_summary = {
        "steps": len(trace_raw),
        "control_window_count": counter_totals["control_window_count"],
        "v25_raw_sensor_sample_count": len(flattened_raw),
        "primary_load_sample_count": len(flattened_primary),
        "shadow_event_count": len(flattened_events),
        "end_reason": final_end_reason,
        "terminated": final_projection.get("terminated"),
        "truncated": final_projection.get("truncated"),
        "safety_stop_count": int(bool(final_projection.get("terminated"))),
        "phase_valid_cycle_count": int(float(legacy_cycles)),
        "legacy_phase_invalid_event_count": int(float(legacy_invalid)),
        "grf_penetration_max_m": max(penetrations),
        "invalid_event_count": 0 if rebuilt_v20.get("passed") is True else 1,
        **{
            name: value
            for name, value in counter_totals.items()
            if name != "control_window_count"
        },
    }
    for name, expected in reconstructed_summary.items():
        if summary.get(name) != expected:
            raise V7EarlyContactExecutionError(
                f"summary statistic is not reconstructed from steps: {name}"
            )
    if (
        summary.get("body_weight_n") != journal.get("body_weight_n")
        or summary.get("n_actor") != len(v6_contract.EXPECTED_ACTOR_FEATURE_NAMES)
        or summary.get("n_observation")
        != len(v6_contract.EXPECTED_OBSERVATION_FEATURE_NAMES)
        or summary.get("actor_feature_names")
        != list(v6_contract.EXPECTED_ACTOR_FEATURE_NAMES)
        or summary.get("observation_feature_names")
        != list(v6_contract.EXPECTED_OBSERVATION_FEATURE_NAMES)
        or summary.get("observation_dtype") != contract.EXPECTED_OBSERVATION_DTYPE
        or summary.get("morphology_weight") != contract.MORPHOLOGY_WEIGHT
    ):
        raise V7EarlyContactExecutionError(
            "persisted layout/body-weight summary is not canonical"
        )
    return summary, rebuilt


def _projected_v5_row(
    *,
    step: int,
    time_s: float,
    actor_observation: Any,
    mean_action: Any,
    raw_action: Any,
    reward: float,
    terminated: bool,
    truncated: bool,
) -> dict[str, Any]:
    return {
        "step": step,
        "time_s": float(time_s),
        "actor_input_view": "historical_analog",
        "actor_observation": actor_observation.tolist(),
        "mean_action": mean_action.tolist(),
        "standard_normal": [0.0, 0.0],
        "raw_action": raw_action.tolist(),
        "reward": float(reward),
        "terminated": bool(terminated),
        "truncated": bool(truncated),
    }


def _worker_start_payload(lock: Mapping[str, Any]) -> dict[str, Any]:
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "status": "H0_V7_EARLY_CONTACT_SHADOW_STARTED",
        "protocol_id": contract.PROTOCOL_ID,
        "collector_id": contract.COLLECTOR_ID,
        "case": dict(contract.CASE),
        "behavior": contract.CASE["behavior"],
        "actor_event_source": "legacy_events",
        "detector_mode": "binary_shadow",
        "primary_load_alignment_id": PRIMARY_ALIGNMENT_ID,
        "platform": preflight.platform_provenance(),
        "execution_lock": _record(LOCK),
        "execution_claim": _record(EXECUTION_CLAIM),
        "worker_claim": _record(worker_claim_path()),
        "baseline_trace": _record(BASELINE_TRACE),
        "source_h0_module_state": lock["inputs"]["source_h0_module_state"],
        "source_h0_module_ctor": lock["inputs"]["source_h0_module_ctor"],
        "source_h0_module_metadata": lock["inputs"]["source_h0_module_metadata"],
        "analog_teacher_profile": lock["inputs"]["analog_teacher_profile"],
        "v25_profile": lock["inputs"]["v25_profile"],
        "authority": dict(contract.AUTHORITY),
        "candidate_created": False,
        "candidate_implemented": False,
        "runtime_promoted": False,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
    }


def _execute_worker(
    *,
    baseline_rows: Sequence[Mapping[str, Any]],
    writer: forensic.ForensicRolloutWriter,
) -> dict[str, Any]:
    rollout_eval, np, torch, RLModule, env_factory, _reward = (
        legacy._load_inference_stack()
    )
    runtime_seed = int(contract.CASE["runtime_seed"])
    np.random.seed(runtime_seed)
    torch.manual_seed(runtime_seed)
    module = RLModule.from_checkpoint(SOURCE_H0_MODULE.resolve())
    env_config = build_env_config()
    env = env_factory.make_cmc_env(env_config)
    rows: list[dict[str, Any]] = []
    projected_trace: list[dict[str, Any]] = []
    journal_samples: list[dict[str, Any]] = []
    primary_load_samples: list[dict[str, Any]] = []
    shadow_events: list[dict[str, Any]] = []
    policy_steps: list[dict[str, Any]] = []
    control_window_count = 0
    unaccepted_so_count = 0
    sea_fallback_count = 0
    timeout_count = 0
    hard_invalid_count = 0
    nonfinite_count = 0
    action_clipped_values = 0
    projected_mismatch_count = 0
    actor_observation_mismatch_count = 0
    actor_mean_mismatch_count = 0
    teacher_std_mismatch_count = 0
    action_mismatch_count = 0
    routing_failure_count = 0
    v20_invalid_event_count = 0
    final_binary_payload: dict[str, Any] | None = None
    penetrations: list[float] = []
    terminated = False
    truncated = False
    info: Mapping[str, Any] = {}
    actor_names: tuple[str, ...] = ()
    full_names: tuple[str, ...] = ()
    started = time.monotonic()
    body_weight_n = float("nan")
    primary_tap: _PrimaryGRFObservationTap | None = None
    primary_tap_audit: dict[str, Any] | None = None
    try:
        observation, reset_info = env.reset(seed=runtime_seed)
        observation = np.asarray(observation, dtype=np.float32)
        base_env = env.unwrapped
        actor_names = tuple(str(name) for name in base_env.actor_feature_names)
        full_names = tuple(str(name) for name in base_env.observation_feature_names)
        rollout_eval._validate_module_observation_contract(
            module,
            actor_names,
            full_names,
        )
        body_weight_n = float(getattr(base_env, "_body_weight_n", float("nan")))
        if not math.isfinite(body_weight_n) or body_weight_n <= 0.0:
            raise V7EarlyContactExecutionError("body weight is not finite/positive")
        if (
            observation.shape != (contract.EXPECTED_FULL_FEATURES,)
            or observation.dtype != np.dtype("float32")
            or actor_names != v6_contract.EXPECTED_ACTOR_FEATURE_NAMES
            or full_names != v6_contract.EXPECTED_OBSERVATION_FEATURE_NAMES
            or tuple(env.action_space.shape) != (contract.EXPECTED_ACTION_DIM,)
        ):
            raise V7EarlyContactExecutionError(
                "runtime layout is not the frozen 35/84 float32 contract"
            )
        if (
            reset_info.get("binary_phase_fsm_executed") is not True
            or reset_info.get("binary_phase_fsm_mode") != "binary_shadow"
            or reset_info.get("binary_phase_event_contract_id")
            != contract.SHADOW_EVENT_CONTRACT_ID
            or reset_info.get("phase_fsm_input_mode") != "legacy_events"
            or reset_info.get("event_contract_id") != "legacy_events_v1"
        ):
            raise V7EarlyContactExecutionError("V25 shadow routing failed at reset")
        baseline_sensor = legacy._validate_raw_sample(
            reset_info.get("binary_phase_sensor_baseline"),
            float(reset_info.get("time")),
            "t0",
        )
        reset_fsm = reset_info.get("binary_phase_fsm")
        if (
            not isinstance(reset_fsm, Mapping)
            or reset_fsm.get("events_this_step") != []
        ):
            raise V7EarlyContactExecutionError("V25 attributed an event to t0")
        primary_tap = _PrimaryGRFObservationTap(
            base_env.runner,
            body_weight_n=body_weight_n,
        )
        primary_tap.install()
        if primary_tap.samples:
            raise V7EarlyContactExecutionError(
                "primary tap observed calls before the first policy step"
            )

        for step_index, baseline_raw in enumerate(baseline_rows):
            step = step_index + 1
            baseline = dict(baseline_raw)
            actor_observation = np.ascontiguousarray(
                observation[: contract.EXPECTED_ACTOR_FEATURES],
                dtype=np.float32,
            )
            expected_actor_observation = np.ascontiguousarray(
                np.asarray(baseline["actor_observation"], dtype=np.float32)
            )
            _raw, queried_mean, teacher_std, _innovation = legacy._policy_values(
                module=module,
                obs=observation,
                action_shape=tuple(env.action_space.shape),
                standard_normal=None,
                stochastic=False,
                rollout_eval=rollout_eval,
            )
            queried_mean = np.ascontiguousarray(
                np.asarray(queried_mean, dtype=np.float32)
            )
            teacher_std = np.ascontiguousarray(
                np.asarray(teacher_std, dtype=np.float32)
            )
            expected_teacher_std = np.ascontiguousarray(
                np.full(
                    (contract.EXPECTED_ACTION_DIM,),
                    contract.EXPECTED_H0_POLICY_STD,
                    dtype=np.float32,
                )
            )
            expected_mean = np.ascontiguousarray(
                np.asarray(baseline["mean_action"], dtype=np.float32)
            )
            raw_action = np.ascontiguousarray(
                np.asarray(baseline["raw_action"], dtype=np.float32)
            )
            actor_exact, actor_sha, expected_actor_sha = exact_float32_vector(
                actor_observation,
                expected_actor_observation,
                length=contract.EXPECTED_ACTOR_FEATURES,
                np=np,
            )
            mean_exact, mean_sha, expected_mean_sha = exact_float32_vector(
                queried_mean,
                expected_mean,
                length=contract.EXPECTED_ACTION_DIM,
                np=np,
            )
            std_exact, std_sha, expected_std_sha = exact_float32_vector(
                teacher_std,
                expected_teacher_std,
                length=contract.EXPECTED_ACTION_DIM,
                np=np,
            )
            applied_action = np.ascontiguousarray(
                np.clip(raw_action, env.action_space.low, env.action_space.high),
                dtype=np.float32,
            )
            action_exact, applied_sha, raw_sha = exact_float32_vector(
                applied_action,
                raw_action,
                length=contract.EXPECTED_ACTION_DIM,
                np=np,
            )
            step_action_clipped_values = int(
                np.count_nonzero(applied_action != raw_action)
            )
            action_clipped_values += step_action_clipped_values

            primary_step_start = len(primary_tap.samples)
            observation_after, reward, terminated, truncated, info = env.step(raw_action)
            observation_after = np.asarray(observation_after, dtype=np.float32)
            if not isinstance(info, Mapping):
                raise V7EarlyContactExecutionError(
                    f"environment info malformed at step {step}"
                )
            runtime_time_s = float(info.get("time"))
            samples = info.get("binary_phase_sensor_samples")
            if (
                not isinstance(samples, Sequence)
                or isinstance(samples, (str, bytes))
                or len(samples) != contract.EXPECTED_SAMPLES_PER_STEP
            ):
                raise V7EarlyContactExecutionError(
                    f"step {step} lacks ten V25 samples"
                )
            previous = (
                float(baseline_sensor["time_s"])
                + step_index * contract.EXPECTED_POLICY_DT_S
            )
            validated_samples = [
                legacy._validate_raw_sample(
                    sample,
                    previous + sample_index * contract.EXPECTED_SAMPLE_DT_S,
                    f"step {step} sample {sample_index}",
                )
                for sample_index, sample in enumerate(samples, start=1)
            ]
            step_primary_samples = primary_tap.deliver_step(
                start_index=primary_step_start,
                delivered_time_s=runtime_time_s,
            )
            cached_online_grf = info.get("online_grf")
            cached_left = (
                cached_online_grf.get("left")
                if isinstance(cached_online_grf, Mapping)
                else None
            )
            last_primary = step_primary_samples[-1]
            if (
                not isinstance(cached_left, Mapping)
                or cached_left.get("normal_force")
                != last_primary["left_normal_force_n"]
                or cached_left.get("in_contact")
                is not last_primary["left_in_contact"]
            ):
                raise V7EarlyContactExecutionError(
                    "cached primary endpoint disagrees with the last observed call"
                )
            primary_load_samples.extend(step_primary_samples)
            journal_samples.extend(validated_samples)
            binary_payload = info.get("binary_phase_fsm")
            if not isinstance(binary_payload, Mapping):
                raise V7EarlyContactExecutionError("V20 shadow payload is missing")
            events_this_step = binary_payload.get("events_this_step")
            if (
                not isinstance(events_this_step, Sequence)
                or isinstance(events_this_step, (str, bytes))
                or not all(isinstance(event, Mapping) for event in events_this_step)
            ):
                raise V7EarlyContactExecutionError("V20 shadow events are malformed")
            final_binary_payload = legacy._jsonable(dict(binary_payload))
            shadow_events.extend(
                legacy._jsonable(dict(event)) for event in events_this_step
            )
            routing_exact = (
                info.get("binary_phase_fsm_executed") is True
                and info.get("binary_phase_fsm_mode") == "binary_shadow"
                and info.get("binary_phase_event_contract_id")
                == contract.SHADOW_EVENT_CONTRACT_ID
                and info.get("phase_fsm_input_mode") == "legacy_events"
                and info.get("event_contract_id") == "legacy_events_v1"
                and info.get("online_grf_applied_sides") == ["left"]
                and isinstance(info.get("phase_fsm"), Mapping)
                and info["phase_fsm"].get("event_source") == "legacy_events"
            )
            routing_failure_count += int(not routing_exact)
            reward_terms = info.get("reward_terms")
            if not isinstance(reward_terms, Mapping):
                raise V7EarlyContactExecutionError("reward terms are missing")
            penetration = float(reward_terms.get("grf_penetration_m"))
            if not math.isfinite(penetration) or penetration < 0.0:
                raise V7EarlyContactExecutionError("penetration is malformed")
            penetrations.append(penetration)
            phase = info.get("phase_fsm")
            step_timeout_count = int(
                float(phase.get("timeout_exceeded", 0.0)) > 0.0
            )
            timeout_count += step_timeout_count
            try:
                classified_so = so_recovery.classify_policy_step(
                    info.get("so_solver_audit_entries"),
                    policy_id=v6_contract.SO_POLICY_ID,
                )
            except so_recovery.SORecoveryContractError as exc:
                raise V7EarlyContactExecutionError(str(exc)) from exc
            so_counters = classified_so["counters"]
            control_window_count += int(so_counters["control_window_count"])
            unaccepted_so_count += int(
                so_counters["unaccepted_hard_so_fallback_count"]
                + so_counters["unaccepted_bounded_ls_count"]
            )
            step_sea_fallback = _sea_fallback_count(
                info.get("sea_segment_diagnostics")
            )
            sea_fallback_count += step_sea_fallback
            step_hard_invalid_count = int("failure" in info)
            hard_invalid_count += step_hard_invalid_count
            finite = bool(
                np.all(np.isfinite(actor_observation))
                and np.all(np.isfinite(observation_after))
                and np.all(np.isfinite(queried_mean))
                and np.all(np.isfinite(teacher_std))
                and math.isfinite(float(reward))
            )
            step_nonfinite_count = int(not finite)
            nonfinite_count += step_nonfinite_count
            projected = _projected_v5_row(
                step=step,
                time_s=runtime_time_s,
                actor_observation=actor_observation,
                mean_action=queried_mean,
                raw_action=raw_action,
                reward=float(reward),
                terminated=bool(terminated),
                truncated=bool(truncated),
            )
            projected_exact = (
                forensic.canonical_json_bytes(projected)
                == forensic.canonical_json_bytes(baseline)
            )
            projected_mismatch_count += int(not projected_exact)
            actor_observation_mismatch_count += int(not actor_exact)
            actor_mean_mismatch_count += int(not mean_exact)
            teacher_std_mismatch_count += int(not std_exact)
            action_mismatch_count += int(not action_exact)
            time_exact = bool(
                math.isfinite(runtime_time_s)
                and abs(runtime_time_s - float(baseline["time_s"]))
                <= TIME_TOLERANCE_S
            )
            checks = {
                "finite": finite,
                "actor_observation_bit_exact": actor_exact,
                "actor_mean_bit_exact": mean_exact,
                "teacher_std_bit_exact": std_exact,
                "frozen_action_unclipped": action_exact,
                "projected_v5_row_bit_exact": projected_exact,
                "runtime_time_matches_v5": time_exact,
                "shadow_routing_exact": routing_exact,
                "ten_raw_samples": len(validated_samples)
                == contract.EXPECTED_SAMPLES_PER_STEP,
                "ten_control_windows": so_counters["control_window_count"] == 10,
                "no_unaccepted_so": (
                    so_counters["unaccepted_hard_so_fallback_count"]
                    + so_counters["unaccepted_bounded_ls_count"]
                )
                == 0,
                "no_sea_fallback": step_sea_fallback == 0,
            }
            policy_step = {
                "step": step,
                "time_s": runtime_time_s,
                "primary_load_samples": step_primary_samples,
                "legacy_online_events": legacy._jsonable(
                    info.get("legacy_online_events", [])
                ),
                "shadow_events": legacy._jsonable(events_this_step),
                "legacy_phase_fsm": legacy._jsonable(phase),
                "v20_shadow_fsm": legacy._jsonable(binary_payload),
            }
            policy_steps.append(policy_step)
            forensic_row = {
                "schema_version": contract.SCHEMA_VERSION,
                "case_id": contract.CASE_ID,
                "baseline_time_s": float(baseline["time_s"]),
                "runtime_time_s": runtime_time_s,
                "projected_v5_row": projected,
                "binary_phase_sensor_samples": validated_samples,
                "binary_phase_sensor_baseline": (
                    baseline_sensor if step == 1 else None
                ),
                "primary_load_samples": step_primary_samples,
                "binary_phase_fsm": legacy._jsonable(binary_payload),
                "legacy_phase_fsm": legacy._jsonable(phase),
                "legacy_online_events": policy_step["legacy_online_events"],
                "digests": {
                    "actor_observation": actor_sha,
                    "expected_actor_observation": expected_actor_sha,
                    "queried_mean": mean_sha,
                    "expected_mean": expected_mean_sha,
                    "teacher_std": std_sha,
                    "expected_teacher_std": expected_std_sha,
                    "applied_action": applied_sha,
                    "frozen_raw_action": raw_sha,
                },
                "checks": checks,
                "so_recovery_counters": legacy._jsonable(so_counters),
                "sufficient_statistics": {
                    "grf_penetration_m": penetration,
                    "action_clipped_values": step_action_clipped_values,
                    "timeout_count": step_timeout_count,
                    "hard_invalid_count": step_hard_invalid_count,
                    "nonfinite_count": step_nonfinite_count,
                    "routing_failure_count": int(not routing_exact),
                    "so_solver_unaccepted_count": int(
                        so_counters["unaccepted_hard_so_fallback_count"]
                        + so_counters["unaccepted_bounded_ls_count"]
                    ),
                    "sea_plugin_fallback_count": step_sea_fallback,
                    "control_window_count": int(
                        so_counters["control_window_count"]
                    ),
                    "runtime_end_reason": info.get("end_reason"),
                    "runtime_failure_present": "failure" in info,
                },
            }
            writer.write_step(step, forensic_row)
            rows.append({"step": step, **forensic_row})
            projected_trace.append(projected)
            observation = observation_after
            if step == 1 or step % 10 == 0:
                elapsed = time.monotonic() - started
                eta = elapsed / step * (contract.EXPECTED_STEPS - step)
                print(
                    f"[V7/V25 early-contact shadow] {step:3d}/"
                    f"{contract.EXPECTED_STEPS} elapsed={elapsed:7.1f}s "
                    f"eta={eta:7.1f}s",
                    flush=True,
                )
            if terminated or truncated:
                break
    finally:
        try:
            if primary_tap is not None and primary_tap.installed:
                primary_tap_audit = primary_tap.restore()
        finally:
            env.close()

    raw_samples_minimal = _minimal_raw_samples(journal_samples)
    if final_binary_payload is None or primary_tap_audit is None:
        raise V7EarlyContactExecutionError(
            "final V20 payload or primary observation tap audit is invalid"
        )
    primary_tap_audit = _validated_primary_tap_audit(primary_tap_audit)
    v20_final_state_audit = contract.audit_v20_final_state(
        baseline_sample=baseline_sensor,
        policy_boundary_times=[step["time_s"] for step in policy_steps],
        final_payload=final_binary_payload,
        raw_samples=raw_samples_minimal,
        shadow_events=shadow_events,
    )
    v20_invalid_event_count = int(v20_final_state_audit.get("passed") is not True)
    contact_classification = contract.classify_contact(
        baseline_sample=baseline_sensor,
        raw_samples=raw_samples_minimal,
        primary_load_samples=primary_load_samples,
        shadow_events=shadow_events,
        body_weight_n=body_weight_n,
    )
    journal = {
        "schema_version": contract.SCHEMA_VERSION,
        "protocol_id": contract.PROTOCOL_ID,
        "case_id": contract.CASE_ID,
        "sample_dt_s": contract.EXPECTED_SAMPLE_DT_S,
        "policy_dt_s": contract.EXPECTED_POLICY_DT_S,
        "primary_load_alignment": {
            "id": PRIMARY_ALIGNMENT_ID,
            "source_cadence_s": contract.EXPECTED_SAMPLE_DT_S,
            "delivery_cadence_s": contract.EXPECTED_POLICY_DT_S,
            "interpolation": False,
            "description": (
                "The existing primary GRF evaluation is observed once at each "
                "pre-integration 1 ms control window t0..t_end-1ms. Ten source "
                "samples are persisted separately from detector samples and "
                "delivered as evidence at the enclosing policy boundary."
            ),
        },
        "body_weight_n": body_weight_n,
        "baseline_raw_sample": baseline_sensor,
        "primary_observation_tap": primary_tap_audit,
        "raw_samples": journal_samples,
        "primary_load_samples": primary_load_samples,
        "shadow_events": shadow_events,
        "final_v20_payload": final_binary_payload,
        "v20_final_state_audit": v20_final_state_audit,
        "policy_steps": policy_steps,
    }
    journal_path = forensic.write_json_exclusive(
        writer.run_directory / "v25_raw_journal.json",
        journal,
    )
    journal_record = _record(journal_path)
    baseline_trace = [dict(row) for row in baseline_rows]
    projected_trace_exact = (
        forensic.canonical_json_bytes(projected_trace)
        == forensic.canonical_json_bytes(baseline_trace)
    )
    phase = info.get("phase_fsm", {}) if isinstance(info, Mapping) else {}
    summary = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.DIAGNOSTIC_COLLECTED_STATUS,
        "protocol_id": contract.PROTOCOL_ID,
        "collector_id": contract.COLLECTOR_ID,
        "case_id": contract.CASE_ID,
        "action_selection": contract.CASE["action_selection"],
        "episode_start_offset_s": contract.CASE["episode_start_offset_s"],
        "action_seed": contract.CASE["action_seed"],
        "runtime_seed": contract.CASE["runtime_seed"],
        "behavior": contract.CASE["behavior"],
        "actor_event_source": "legacy_events",
        "source_h0_id": contract.SOURCE_H0_ID,
        "source_observation_contract_id": (
            contract.SOURCE_OBSERVATION_CONTRACT_ID
        ),
        "source_event_contract_id": contract.SOURCE_EVENT_CONTRACT_ID,
        "primary_load_contract_id": contract.PRIMARY_LOAD_CONTRACT_ID,
        "primary_load_evidence_role": contract.PRIMARY_LOAD_EVIDENCE_ROLE,
        "canonical_scientific_oracle": contract.CANONICAL_SCIENTIFIC_ORACLE,
        "primary_online_grf_used_as_event_source": False,
        "binary_phase_fsm_mode": "binary_shadow",
        "binary_phase_event_contract_id": contract.SHADOW_EVENT_CONTRACT_ID,
        "primary_load_alignment_id": PRIMARY_ALIGNMENT_ID,
        "steps": len(rows),
        "control_window_count": control_window_count,
        "v25_raw_sensor_sample_count": len(journal_samples),
        "primary_load_sample_count": len(primary_load_samples),
        "shadow_event_count": len(shadow_events),
        "end_reason": info.get("end_reason") if isinstance(info, Mapping) else None,
        "terminated": bool(terminated),
        "truncated": bool(truncated),
        "phase_valid_cycle_count": int(float(phase.get("valid_cycle_count", 0))),
        "grf_penetration_max_m": max(penetrations, default=0.0),
        "v5_projected_trace_bit_exact": projected_trace_exact,
        "projected_trace_mismatch_count": projected_mismatch_count,
        "actor_observation_mismatch_count": actor_observation_mismatch_count,
        "actor_mean_mismatch_count": actor_mean_mismatch_count,
        "teacher_std_mismatch_count": teacher_std_mismatch_count,
        "action_mismatch_count": action_mismatch_count,
        "time_mismatch_count": sum(
            int(
                abs(float(observed["time_s"]) - float(expected["time_s"]))
                > TIME_TOLERANCE_S
            )
            for observed, expected in zip(projected_trace, baseline_trace)
        ),
        "step_contract_failure_count": sum(
            int(not all(row["checks"].values())) for row in rows
        ),
        "routing_failure_count": routing_failure_count,
        "action_clipped_values": action_clipped_values,
        "timeout_count": timeout_count,
        "safety_stop_count": int(bool(terminated)),
        # This is the V20 diagnostic stream counter.  The historical actor-FSM
        # counter is retained separately and is not reinterpreted by V7.
        "invalid_event_count": v20_invalid_event_count,
        "legacy_phase_invalid_event_count": int(
            float(phase.get("invalid_event_count", 0))
        ),
        "hard_invalid_count": hard_invalid_count,
        "nonfinite_count": nonfinite_count,
        "so_solver_unaccepted_count": unaccepted_so_count,
        "sea_plugin_fallback_count": sea_fallback_count,
        "morphology_weight": env_config["reward"]["morphology_weight"],
        "body_weight_n": body_weight_n,
        "n_actor": len(actor_names),
        "n_observation": len(full_names),
        "observation_dtype": "float32",
        "platform": preflight.platform_provenance(),
        "actor_feature_names": list(actor_names),
        "observation_feature_names": list(full_names),
        "baseline_trace_sha256": _payload_sha256(baseline_trace),
        "projected_v5_trace_sha256": _payload_sha256(projected_trace),
        "v25_raw_journal": journal_record,
        "v25_raw_journal_payload_sha256": _payload_sha256(journal),
        "contact_classification": contact_classification,
        "v20_final_state_audit": v20_final_state_audit,
        "primary_observation_tap": primary_tap_audit,
        "v6_terminal_status": "FAIL_H0_V6_V25_TEACHER_REPLAY_DEVELOPMENT",
        "v6_terminal_ledger_sha256": contract.EXPECTED_INPUT_SHA256[
            "v6_terminal_ledger"
        ],
        "v6_nominal_failure_sha256": contract.EXPECTED_INPUT_SHA256[
            "v6_nominal_failure"
        ],
        "candidate_created": False,
        "candidate_implemented": False,
        "runtime_promoted": False,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
    }
    partial_summary = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": "H0_V7_EARLY_CONTACT_PERSISTED_BEFORE_GATE",
        "protocol_id": contract.PROTOCOL_ID,
        "case_id": contract.CASE_ID,
        "steps": len(rows),
        "expected_steps": contract.EXPECTED_STEPS,
        "v25_raw_journal": journal_record,
        "contact_classification": contact_classification,
        "gate_evaluated": False,
        "candidate_created": False,
        "candidate_implemented": False,
        "runtime_promoted": False,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
    }
    persisted = writer.finalize_before_gate(
        trace=rows,
        partial_summary=partial_summary,
        summary=summary,
    )

    def evaluate_gate(records: dict[str, Any]) -> dict[str, Any]:
        verify_lock(require_run_root_absent=False)
        if _record(journal_path) != journal_record:
            raise V7EarlyContactExecutionError(
                "raw journal changed before the diagnostic gate"
            )
        persisted_summary, rebuilt_classification = _audit_persisted_evidence(
            trace_path=writer.trace_path,
            summary_path=writer.summary_path,
            journal_path=journal_path,
        )
        if (
            forensic.canonical_json_bytes(persisted_summary)
            != forensic.canonical_json_bytes(summary)
            or forensic.canonical_json_bytes(rebuilt_classification)
            != forensic.canonical_json_bytes(contact_classification)
        ):
            raise V7EarlyContactExecutionError(
                "in-memory evidence differs from persisted diagnostic evidence"
            )
        gate = contract.diagnostic_gate(persisted_summary)
        gate["persisted_before_gate"] = {
            **records,
            "v25_raw_journal": journal_record,
        }
        return gate

    writer.run_gate(evaluate_gate)
    gate = _mapping(writer.gate_path)
    if gate.get("passed") is not True:
        writer.publish_failure(
            end_reason="diagnostic_gate_failed",
            error={
                "type": "V7EarlyContactGateFailure",
                "message": "V7 early-contact diagnostic was ambiguous or invalid",
            },
            status=contract.DIAGNOSTIC_FAIL_STATUS,
            details={"gate": _record(writer.gate_path)},
        )
        raise V7EarlyContactExecutionError("V7 early-contact diagnostic failed")
    receipt = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.DIAGNOSTIC_PASS_STATUS,
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "case_id": contract.CASE_ID,
        "decision": contact_classification["decision"],
        "contact_classification": contact_classification,
        "artifacts": writer.artifact_records(),
        "v25_raw_journal": journal_record,
        "persisted_before_gate": persisted,
        "execution_lock": _record(LOCK),
        "execution_claim": _record(EXECUTION_CLAIM),
        "worker_claim": _record(worker_claim_path()),
        "candidate_created": False,
        "candidate_implemented": False,
        "runtime_promoted": False,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
    }
    receipt_path = forensic.write_json_exclusive(
        writer.run_directory / "receipt.json",
        receipt,
    )
    return {**receipt, "receipt": _record(receipt_path)}


def run_worker(*, output_dir: str | Path, execution_token: str) -> dict[str, Any]:
    lock = verify_lock()
    verify_worker_claim(execution_token)
    observed = Path(output_dir).expanduser().resolve()
    if observed != DESTINATION:
        raise V7EarlyContactExecutionError(
            f"non-canonical worker destination: {observed} != {DESTINATION}"
        )
    writer = forensic.ForensicRolloutWriter(DESTINATION, artifact_root=REPO_ROOT)
    try:
        writer.start(_worker_start_payload(lock))
        return _execute_worker(
            baseline_rows=load_frozen_baseline(),
            writer=writer,
        )
    except Exception as exc:
        if writer.run_start_path.is_file() and not os.path.lexists(writer.failure_path):
            try:
                writer.publish_failure(
                    end_reason="worker_exception",
                    error=exc,
                    status=contract.DIAGNOSTIC_FAIL_STATUS,
                    details={
                        "case_id": contract.CASE_ID,
                        "candidate_created": False,
                        "candidate_implemented": False,
                        "runtime_promoted": False,
                        "actor_updates": 0,
                        "critic_updates": 0,
                        "ppo_updates": 0,
                        "protected_trials_opened": [],
                        "reserve_trials_opened": [],
                    },
                )
            except Exception as receipt_error:
                raise V7EarlyContactExecutionError(
                    f"worker failed and failure receipt failed: {receipt_error}"
                ) from exc
        raise


def _worker_command(execution_token: str) -> list[str]:
    return [
        sys.executable,
        str(Path(__file__).resolve()),
        "--worker",
        "--output-dir",
        str(DESTINATION),
        "--execution-token",
        execution_token,
    ]


def _verify_receipt() -> dict[str, Any]:
    live_lock = verify_lock(require_run_root_absent=False)
    receipt_path = DESTINATION / "receipt.json"
    receipt = _mapping(receipt_path)
    trace_path = DESTINATION / "trace.json"
    partial_path = DESTINATION / "partial_summary.json"
    summary_path = DESTINATION / "summary.json"
    gate_path = DESTINATION / "gate.json"
    journal_path = DESTINATION / "v25_raw_journal.json"
    run_start_path = DESTINATION / "run_start.json"
    failure_path = DESTINATION / "failure.json"
    artifacts = receipt.get("artifacts")
    persisted = receipt.get("persisted_before_gate")
    if (
        receipt.get("status") != contract.DIAGNOSTIC_PASS_STATUS
        or receipt.get("passed") is not True
        or receipt.get("protocol_id") != contract.PROTOCOL_ID
        or receipt.get("case_id") != contract.CASE_ID
        or receipt.get("actor_updates") != 0
        or receipt.get("critic_updates") != 0
        or receipt.get("ppo_updates") != 0
        or receipt.get("protected_trials_opened") != []
        or receipt.get("reserve_trials_opened") != []
        or receipt.get("candidate_created") is not False
        or receipt.get("candidate_implemented") is not False
        or receipt.get("runtime_promoted") is not False
        or not _record_matches(receipt.get("execution_lock"), LOCK)
        or not _record_matches(receipt.get("execution_claim"), EXECUTION_CLAIM)
        or not _record_matches(receipt.get("worker_claim"), worker_claim_path())
        or not _record_matches(receipt.get("v25_raw_journal"), journal_path)
        or not isinstance(artifacts, Mapping)
        or set(artifacts)
        != {"run_start", "steps", "trace", "partial_summary", "summary", "gate"}
        or not _record_matches(artifacts.get("run_start"), run_start_path)
        or os.path.lexists(failure_path)
        or not _record_matches(artifacts.get("trace"), trace_path)
        or not _record_matches(artifacts.get("partial_summary"), partial_path)
        or not _record_matches(artifacts.get("summary"), summary_path)
        or not _record_matches(artifacts.get("gate"), gate_path)
        or not isinstance(artifacts.get("steps"), list)
        or len(artifacts["steps"]) != contract.EXPECTED_STEPS
        or not isinstance(persisted, Mapping)
        or set(persisted) != {"trace", "partial_summary", "summary"}
        or not _record_matches(persisted.get("trace"), trace_path)
        or not _record_matches(persisted.get("partial_summary"), partial_path)
        or not _record_matches(persisted.get("summary"), summary_path)
    ):
        raise V7EarlyContactExecutionError("worker receipt drifted")
    if forensic.canonical_json_bytes(_mapping(run_start_path)) != (
        forensic.canonical_json_bytes(_worker_start_payload(live_lock))
    ):
        raise V7EarlyContactExecutionError("worker run_start payload drifted")
    for index, record in enumerate(artifacts["steps"], start=1):
        if not _record_matches(record, DESTINATION / "steps" / f"{index:06d}.json"):
            raise V7EarlyContactExecutionError(
                f"worker step artifact drifted: {index}"
            )
    summary, rebuilt_classification = _audit_persisted_evidence(
        trace_path=trace_path,
        summary_path=summary_path,
        journal_path=journal_path,
    )
    gate = _mapping(gate_path)
    expected_gate = contract.diagnostic_gate(summary)
    gate_persisted = gate.get("persisted_before_gate")
    if (
        gate.get("passed") is not True
        or not isinstance(gate.get("checks"), Mapping)
        or not all(value is True for value in gate["checks"].values())
        or any(gate.get(key) != value for key, value in expected_gate.items())
        or not isinstance(gate_persisted, Mapping)
        or set(gate_persisted)
        != {"trace", "partial_summary", "summary", "v25_raw_journal"}
        or any(
            gate_persisted.get(name) != persisted.get(name)
            for name in ("trace", "partial_summary", "summary")
        )
        or not _record_matches(
            gate_persisted.get("v25_raw_journal"), journal_path
        )
        or forensic.canonical_json_bytes(receipt.get("contact_classification"))
        != forensic.canonical_json_bytes(rebuilt_classification)
        or receipt.get("decision") != rebuilt_classification.get("decision")
        or receipt.get("decision") != gate.get("decision")
    ):
        raise V7EarlyContactExecutionError(
            "worker gate/classification receipt drifted"
        )
    verify_lock(require_run_root_absent=False)
    return receipt


def execute() -> dict[str, Any]:
    verify_lock(require_run_root_absent=True)
    started = time.time()
    execution_token = secrets.token_urlsafe(32)
    token_sha256 = _token_sha256(execution_token)
    forensic.write_json_exclusive(
        EXECUTION_CLAIM,
        _execution_claim_payload(token_sha256),
    )
    forensic.write_json_exclusive(
        worker_claim_path(),
        _worker_claim_payload(token_sha256),
    )
    passed = False
    error: str | None = None
    receipt: dict[str, Any] | None = None
    decision = contract.NO_DECISION
    try:
        completed = subprocess.run(
            _worker_command(execution_token),
            cwd=REPO_ROOT,
            timeout=WORKER_TIMEOUT_S,
            check=False,
        )
        if completed.returncode != 0:
            raise V7EarlyContactExecutionError(
                f"worker {contract.CASE_ID} exited {completed.returncode}"
            )
        receipt = _verify_receipt()
        decision = str(receipt["decision"])
        passed = True
    except Exception as exc:
        error = f"{type(exc).__name__}: {exc}"
    status = (
        contract.DIAGNOSTIC_PASS_STATUS
        if passed
        else contract.DIAGNOSTIC_FAIL_STATUS
    )
    next_stage_by_decision = {
        contract.MIN_SWING_CANDIDATE_DECISION: (
            "FREEZE_BINARY_ACTIVE_MIN_SWING_0P20_DEVELOPMENT_PROTOCOL"
        ),
        contract.ACTOR_CORRECTION_DECISION: (
            "FREEZE_ACTOR_CORRECTION_DEVELOPMENT_PROTOCOL"
        ),
    }
    ledger = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": status,
        "passed": passed,
        "protocol_id": contract.PROTOCOL_ID,
        "started_unix_s": started,
        "completed_unix_s": time.time(),
        "completed_cases": [contract.CASE_ID] if passed else [],
        "receipt": (
            _record(DESTINATION / "receipt.json") if receipt is not None else None
        ),
        "decision": decision,
        "error": error,
        "execution_lock": _record(LOCK),
        "execution_claim": _record(EXECUTION_CLAIM),
        "candidate_created": False,
        "candidate_implemented": False,
        "runtime_promoted": False,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
        "next_stage": (
            next_stage_by_decision.get(decision, "STOP_AMBIGUOUS_FAIL_CLOSED")
            if passed
            else "STOP_WITHOUT_RETRY"
        ),
    }
    forensic.write_json_exclusive(EXECUTION_LEDGER, ledger)
    print(json.dumps(ledger, indent=2, sort_keys=True, allow_nan=False), flush=True)
    if not passed:
        raise V7EarlyContactExecutionError(error or status)
    return ledger


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--worker", action="store_true")
    mode.add_argument("--execute", action="store_true")
    parser.add_argument("--output-dir")
    parser.add_argument("--execution-token")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        if args.execute:
            result = execute()
        else:
            if args.output_dir is None or args.execution_token is None:
                raise V7EarlyContactExecutionError(
                    "--output-dir and supervisor execution token are required"
                )
            result = run_worker(
                output_dir=args.output_dir,
                execution_token=args.execution_token,
            )
    except Exception as exc:
        print(
            f"V7 early-contact diagnostic failed closed: "
            f"{type(exc).__name__}: {exc}",
            file=sys.stderr,
        )
        return 2
    print(json.dumps(result, indent=2, sort_keys=True, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
