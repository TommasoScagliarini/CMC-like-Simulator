"""Canonical semantic-replay adaptation of H0 to ``primary_grf_split_v1``.

V3 never lets the source H0 drive a newly visited state during collection.
Instead, it replays two previously validated 500-step action tapes on the
canonical H0 setup, pairs the current primary-GRF actor view with the exact
historical analog-GRF view, and distils only the two changed input columns.
The third registered tape is opened only after the fitted candidate has been
frozen and is used as a procedural final holdout.

The diagnostic replay mode is intentionally available before protocol freeze;
all mutating protocol stages require the immutable V3 execution lock.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
import numbers
import os
import subprocess
import sys
import tempfile
import time
from pathlib import Path, PurePosixPath
from typing import Any, Mapping, Sequence


REPO_ROOT = Path(__file__).resolve().parents[1]
VALIDATION_ROOT = REPO_ROOT / "validation"
TRAJECTORY_ROOT = REPO_ROOT / "Trajectory Generator"
BASELINE_ROOT = TRAJECTORY_ROOT / "baseline_MLP"
for import_root in (VALIDATION_ROOT, BASELINE_ROOT, TRAJECTORY_ROOT, REPO_ROOT):
    if str(import_root) not in sys.path:
        sys.path.insert(0, str(import_root))

import compare_h0_v25_abc as common_gates  # noqa: E402
import h0_primary_grf_split_v3_freeze_contract as freeze_contract  # noqa: E402
import h0_v3_so_recovery_contract as so_recovery  # noqa: E402
import primary_grf_split_adaptation as split_contract  # noqa: E402
import run_h0_primary_grf_split_v1_adaptation as v1  # noqa: E402
import run_h0_v25_abc_preflight as h0_runtime  # noqa: E402
import target_domain_imitation as imitation  # noqa: E402
import target_domain_markov_adaptation as markov_adaptation  # noqa: E402
import warm_start  # noqa: E402


LOCK = VALIDATION_ROOT / "h0_primary_grf_split_v3_execution_lock.json"
PLAN = (
    REPO_ROOT
    / "reports"
    / "plans"
    / "2026-08-06_protocollo_h0_primary_split_v3_semantic_replay.md"
)
RUNNER = Path(__file__).resolve()
TESTS = VALIDATION_ROOT / "test_h0_primary_grf_split_v3.py"
RUN_ROOT = (
    VALIDATION_ROOT
    / "h0_primary_grf_split_adaptation_runs"
    / "2026-08-06_h0_primary_split_v3_semantic_replay"
)
ATTEMPT_CLAIM = RUN_ROOT / "attempt_claim.json"
CANDIDATE_FREEZE = RUN_ROOT / "adaptation" / "candidate_freeze.json"
HOLDOUT_ACCESS_CLAIM = RUN_ROOT / "holdout_access_claim.json"
PROTOCOL_ID = "AB06_H0_PRIMARY_GRF_SPLIT_V3_SEMANTIC_REPLAY"
CANDIDATE_ID = "H0_primary_split_v3_semantic_replay"
REVISION = "2026-08-06"
H0_MODULE = v1.H0_MODULE
H0_CONFIG = v1.H0_CONFIG
EXPECTED_STEPS = 500
EXPECTED_ACTOR_FEATURES = 35
EXPECTED_FULL_FEATURES = 84
EXPECTED_SIGMA = 0.005
EVENT_CONTRACT = "primary_grf_split_v1+legacy_events_v1"
CANONICAL_OFFSET_S = 1.956870983805102
CANONICAL_SEEDS = (123, 124, 125)
TRAIN_SEEDS = (123, 124)
FINAL_HOLDOUT_SEED = 125
MUTABLE_FEATURE_NAMES = (
    "online_left_normal_grf_bw",
    "online_left_in_contact",
)
TRACE_ROOT_TEMPLATE = (
    "controller_memory_ablation/2026-07-13_markov35_corrected_full_sigma0005_seed{seed}"
)
WORKER_TIMEOUT_S = 2400.0
FIT = {
    "seed": 123,
    "epochs": 400,
    "batch_size": 128,
    "learning_rate": 5.0e-5,
    "validation_fraction": 0.0,
    "patience": 0,
    "clip_weight": 1.0,
    "logstd_weight": 0.0,
    "anchor_weight": 1.0e-2,
    "selection_mode": "fixed_final_epoch",
}
SO_RECOVERY_COUNTER_KEYS = (
    "control_window_count",
    "solver_invocation_count",
    "primary_solver_nonconvergence_count",
    "bounded_ls_invocation_count",
    "selected_bounded_ls_count",
    "verified_bounded_ls_count",
    "verified_bounded_ls_success_count",
    "verified_status0_max_iter_count",
    "unaccepted_hard_so_fallback_count",
    "unaccepted_bounded_ls_count",
    "hard_so_fallback_count",
    "reuse_previous_count",
    "bounded_ls_unsuccessful_count",
    "bounds_violation_count",
    "nonfinite_solver_count",
    "selected_infeasible_count",
    "selected_solution_mismatch_count",
    "residual_contract_mismatch_count",
)
LOCK_AUTHORITY = {
    "historical_action_replay_authorized": True,
    "actor_only_adaptation_authorized": True,
    "ppo_updates_authorized": False,
    "critic_updates_authorized": False,
    "protected_trial_access_authorized": False,
    "primary_grf_modification_authorized": False,
    "sea_semantic_modification_authorized": False,
    "runtime_promotion_authorized": False,
}
LOCK_TOP_LEVEL_KEYS = {
    "schema_version",
    "status",
    "protocol_id",
    "revision",
    "candidate_id",
    "run_root",
    "expected_steps",
    "canonical_seeds",
    "canonical_offset_s",
    "event_contract_id",
    "so_policy_id",
    "so_policy",
    "fit",
    "mutable_feature_names",
    "destinations",
    "authority",
    "sources",
    "inputs",
    "runtime_closure",
    "actor_update_candidate_count",
    "retry_or_retuning_allowed",
    "critic_updates",
    "ppo_updates",
    "protected_trials_opened",
}


class H0PrimarySplitV3Error(RuntimeError):
    """Raised on any fail-closed semantic-replay contract violation."""


def sha256_file(path: str | Path) -> str:
    digest = hashlib.sha256()
    with Path(path).expanduser().resolve().open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _repo_relative_posix(path: str | Path) -> str:
    resolved = Path(path).expanduser().resolve()
    try:
        relative = resolved.relative_to(REPO_ROOT)
    except ValueError as exc:
        raise H0PrimarySplitV3Error(
            f"path is outside the repository: {resolved}"
        ) from exc
    return relative.as_posix()


def source_record(path: str | Path) -> dict[str, Any]:
    resolved = Path(path).expanduser().resolve()
    return {
        "path": _repo_relative_posix(resolved),
        "sha256": sha256_file(resolved),
        "size_bytes": resolved.stat().st_size,
    }


def _external_record(path: str | Path) -> dict[str, Any]:
    resolved = Path(path).expanduser().resolve()
    return {
        "path": resolved.as_posix(),
        "path_scope": "external_absolute",
        "sha256": sha256_file(resolved),
        "size_bytes": resolved.stat().st_size,
    }


def _require_bool(value: Any, label: str) -> bool:
    if type(value) is not bool:
        raise H0PrimarySplitV3Error(f"{label} must be a JSON boolean")
    return value


def _require_counter(value: Any, label: str) -> int:
    if isinstance(value, (bool,)) or not isinstance(value, numbers.Integral):
        raise H0PrimarySplitV3Error(f"{label} must be a non-negative integer")
    result = int(value)
    if result < 0:
        raise H0PrimarySplitV3Error(f"{label} must be a non-negative integer")
    return result


def _require_integral_number(value: Any, label: str) -> int:
    if isinstance(value, (bool,)) or not isinstance(value, numbers.Real):
        raise H0PrimarySplitV3Error(f"{label} must be an integral finite number")
    number = float(value)
    if not math.isfinite(number) or not number.is_integer() or number < 0.0:
        raise H0PrimarySplitV3Error(f"{label} must be an integral finite number")
    return int(number)


def _require_finite_number(value: Any, label: str) -> float:
    if isinstance(value, (bool,)) or not isinstance(value, numbers.Real):
        raise H0PrimarySplitV3Error(f"{label} must be a finite number")
    result = float(value)
    if not math.isfinite(result):
        raise H0PrimarySplitV3Error(f"{label} must be a finite number")
    return result


def _require_finite_vector(value: Any, length: int, label: str) -> list[Any]:
    if not isinstance(value, list) or len(value) != length:
        raise H0PrimarySplitV3Error(f"{label} must contain {length} values")
    for index, item in enumerate(value):
        _require_finite_number(item, f"{label}[{index}]")
    return value


def _require_array_contract(
    value: Any,
    *,
    np: Any,
    label: str,
    dtype: Any,
    shape: tuple[int, ...] | None = None,
    finite: bool = True,
) -> Any:
    array = np.asarray(value)
    expected_dtype = np.dtype(dtype)
    if array.dtype != expected_dtype:
        raise H0PrimarySplitV3Error(f"{label} dtype {array.dtype} != {expected_dtype}")
    if shape is not None and array.shape != shape:
        raise H0PrimarySplitV3Error(f"{label} shape {array.shape} != {shape}")
    if finite and not np.all(np.isfinite(array)):
        raise H0PrimarySplitV3Error(f"{label} contains non-finite values")
    return np.ascontiguousarray(array)


def _require_sha256(value: Any, label: str) -> str:
    if (
        not isinstance(value, str)
        or len(value) != 64
        or any(character not in "0123456789abcdef" for character in value)
    ):
        raise H0PrimarySplitV3Error(f"{label} must be a lowercase SHA-256")
    return value


def _require_finite_json_tree(value: Any, label: str) -> None:
    if value is None or type(value) in {bool, str}:
        return
    if isinstance(value, numbers.Integral):
        return
    if isinstance(value, numbers.Real):
        if not math.isfinite(float(value)):
            raise H0PrimarySplitV3Error(f"{label} contains a non-finite number")
        return
    if isinstance(value, Mapping):
        for key, child in value.items():
            if not isinstance(key, str):
                raise H0PrimarySplitV3Error(f"{label} has a non-string key")
            _require_finite_json_tree(child, f"{label}.{key}")
        return
    if isinstance(value, list):
        for index, child in enumerate(value):
            _require_finite_json_tree(child, f"{label}[{index}]")
        return
    raise H0PrimarySplitV3Error(f"{label} contains a non-JSON value")


def _validate_so_solver_audit_entries(
    value: Any,
    *,
    step_index: int,
    selected_fallback: bool,
) -> tuple[list[dict[str, Any]], int, int]:
    if not isinstance(value, list) or not value:
        raise H0PrimarySplitV3Error("SO solver audit entries are missing")
    normalized: list[dict[str, Any]] = []
    hard_fallback_count = 0
    attempt_count = 0
    for window_offset, raw_window in enumerate(value, start=1):
        label = f"step {step_index} SO window {window_offset}"
        if not isinstance(raw_window, Mapping) or set(raw_window) != {
            "control_window_index",
            "control_window_time_s",
            "selected_feasibility_attempt_index",
            "served_solution_sha256",
            "selected_solver_solution_matches_served",
            "attempts",
        }:
            raise H0PrimarySplitV3Error(f"{label} schema drifted")
        window = dict(raw_window)
        if _require_counter(window["control_window_index"], label) != window_offset:
            raise H0PrimarySplitV3Error(f"{label} index is not contiguous")
        _require_finite_number(window["control_window_time_s"], f"{label} time")
        selected_index = _require_counter(
            window["selected_feasibility_attempt_index"],
            f"{label} selected attempt",
        )
        if selected_index <= 0:
            raise H0PrimarySplitV3Error(f"{label} selected attempt must be positive")
        served_sha = _require_sha256(
            window["served_solution_sha256"], f"{label} served solution"
        )
        if not _require_bool(
            window["selected_solver_solution_matches_served"],
            f"{label} selected/served match",
        ):
            raise H0PrimarySplitV3Error(
                f"{label} selected solver solution does not match served controls"
            )
        raw_attempts = window["attempts"]
        if not isinstance(raw_attempts, list) or not raw_attempts:
            raise H0PrimarySplitV3Error(f"{label} attempts are missing")
        selected_attempts = 0
        selected_attempt_fallback: bool | None = None
        for attempt_offset, raw_attempt in enumerate(raw_attempts, start=1):
            attempt_label = f"{label} attempt {attempt_offset}"
            if not isinstance(raw_attempt, Mapping) or set(raw_attempt) != {
                "attempt_index",
                "feasibility_scale",
                "feasibility_accepted",
                "residual_norm",
                "residual_relative_norm",
                "residual_max_abs",
                "solver_fallback_used",
                "solver_path",
                "selected",
            }:
                raise H0PrimarySplitV3Error(f"{attempt_label} schema drifted")
            attempt = dict(raw_attempt)
            if (
                _require_counter(attempt["attempt_index"], attempt_label)
                != attempt_offset
            ):
                raise H0PrimarySplitV3Error(f"{attempt_label} index is not contiguous")
            for key in (
                "feasibility_scale",
                "residual_norm",
                "residual_relative_norm",
                "residual_max_abs",
            ):
                _require_finite_number(attempt[key], f"{attempt_label}.{key}")
            _require_bool(
                attempt["feasibility_accepted"],
                f"{attempt_label}.feasibility_accepted",
            )
            attempt_fallback = _require_bool(
                attempt["solver_fallback_used"],
                f"{attempt_label}.solver_fallback_used",
            )
            is_selected = _require_bool(
                attempt["selected"], f"{attempt_label}.selected"
            )
            solver_path = attempt["solver_path"]
            if not isinstance(solver_path, Mapping):
                raise H0PrimarySplitV3Error(f"{attempt_label}.solver_path is missing")
            _require_finite_json_tree(solver_path, f"{attempt_label}.solver_path")
            if solver_path.get("schema") != "static_optimization_solver_audit_v1":
                raise H0PrimarySplitV3Error(
                    f"{attempt_label} solver audit schema drifted"
                )
            for key in (
                "input_matrix_finite",
                "input_target_finite",
                "weights_finite",
                "bounds_finite",
                "warm_start_finite",
                "bounded_lsq_used",
                "reuse_previous_solution",
                "hard_fallback",
            ):
                _require_bool(solver_path.get(key), f"{attempt_label}.{key}")
            selected_solution = solver_path.get("selected_solution")
            if not isinstance(selected_solution, Mapping):
                raise H0PrimarySplitV3Error(
                    f"{attempt_label}.selected_solution is missing"
                )
            for key in (
                "output_shape_matches",
                "output_finite",
                "equality_residual_finite",
            ):
                _require_bool(
                    selected_solution.get(key),
                    f"{attempt_label}.selected_solution.{key}",
                )
            selected_sha = _require_sha256(
                selected_solution.get("output_sha256"),
                f"{attempt_label}.selected_solution.output_sha256",
            )
            if is_selected:
                selected_attempts += 1
                selected_attempt_fallback = attempt_fallback
                if attempt_offset != selected_index or selected_sha != served_sha:
                    raise H0PrimarySplitV3Error(
                        f"{attempt_label} does not identify the served solution"
                    )
            hard_fallback_count += int(solver_path["hard_fallback"] is True)
            attempt_count += 1
        if selected_attempts != 1 or selected_attempt_fallback is None:
            raise H0PrimarySplitV3Error(
                f"{label} must identify exactly one selected attempt"
            )
        if (
            window_offset == len(value)
            and selected_attempt_fallback is not selected_fallback
        ):
            raise H0PrimarySplitV3Error(
                f"{label} selected fallback flag disagrees with SO diagnostics"
            )
        normalized.append(copy.deepcopy(window))
    return normalized, attempt_count, hard_fallback_count


def _publish_temporary_exclusive(temporary: Path, destination: Path) -> None:
    """Atomically publish a completed temp file after an exclusive claim.

    The zero-byte claim intentionally remains after a publication failure.  A
    crashed attempt therefore cannot silently reuse the same artifact path.
    This uses only APIs available on both Windows and POSIX; no hard links are
    required.
    """

    flags = os.O_CREAT | os.O_EXCL | os.O_WRONLY | getattr(os, "O_BINARY", 0)
    try:
        descriptor = os.open(str(destination), flags, 0o600)
    except FileExistsError as exc:
        raise H0PrimarySplitV3Error(f"refusing to clobber: {destination}") from exc
    else:
        os.close(descriptor)
    os.replace(str(temporary), str(destination))


def _write_json_exclusive(path: str | Path, payload: Any) -> Path:
    destination = Path(path).expanduser().resolve()
    common_gates.canonical_json_bytes(payload)
    destination.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_raw = tempfile.mkstemp(
        prefix=f".{destination.name}.", suffix=".tmp", dir=destination.parent
    )
    temporary = Path(temporary_raw)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            json.dump(
                payload,
                stream,
                indent=2,
                sort_keys=True,
                ensure_ascii=False,
                allow_nan=False,
            )
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        _publish_temporary_exclusive(temporary, destination)
    finally:
        if temporary.exists():
            temporary.unlink()
    return destination


def _strict_mapping(path: str | Path) -> dict[str, Any]:
    payload = common_gates.strict_json_load(path)
    if not isinstance(payload, Mapping):
        raise H0PrimarySplitV3Error(f"expected JSON object: {path}")
    return dict(payload)


def _strict_sequence(path: str | Path) -> list[Any]:
    payload = common_gates.strict_json_load(path)
    if isinstance(payload, (str, bytes)) or not isinstance(payload, Sequence):
        raise H0PrimarySplitV3Error(f"expected JSON array: {path}")
    return list(payload)


def _write_npz_exclusive(path: str | Path, **arrays: Any) -> Path:
    import numpy as np

    destination = Path(path).expanduser().resolve()
    destination.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_raw = tempfile.mkstemp(
        prefix=f".{destination.name}.", suffix=".tmp", dir=destination.parent
    )
    temporary = Path(temporary_raw)
    try:
        with os.fdopen(descriptor, "wb") as stream:
            np.savez(stream, **arrays)
            stream.flush()
            os.fsync(stream.fileno())
        _publish_temporary_exclusive(temporary, destination)
    finally:
        if temporary.exists():
            temporary.unlink()
    return destination


def _historical_directory(seed: int) -> Path:
    if seed not in CANONICAL_SEEDS:
        raise H0PrimarySplitV3Error(f"unregistered canonical seed {seed}")
    return VALIDATION_ROOT / TRACE_ROOT_TEMPLATE.format(seed=seed)


def historical_inputs(seed: int) -> tuple[list[Any], dict[str, Any], Path, Path]:
    directory = _historical_directory(seed)
    trace_path = directory / "rollout_policy_trace.json"
    summary_path = directory / "rollout_summary.json"
    trace = _strict_sequence(trace_path)
    summary = _strict_mapping(summary_path)
    if (
        len(trace) != EXPECTED_STEPS
        or _require_counter(summary.get("steps"), "historical steps") != EXPECTED_STEPS
    ):
        raise H0PrimarySplitV3Error(f"historical seed {seed} is not 500 steps")
    if (
        summary.get("end_reason") != "episode_time_limit"
        or _require_bool(summary.get("terminated"), "historical terminated")
        is not False
        or _require_bool(summary.get("truncated"), "historical truncated") is not True
        or _require_counter(summary.get("action_seed"), "historical action_seed")
        != seed
        or summary.get("action_selection") != "stochastic"
        or abs(
            _require_finite_number(
                summary.get("episode_start_offset_s"),
                "historical episode_start_offset_s",
            )
            - CANONICAL_OFFSET_S
        )
        > 1.0e-12
    ):
        raise H0PrimarySplitV3Error(f"historical seed {seed} contract drifted")
    raw_names = summary.get("actor_feature_names")
    if not isinstance(raw_names, list) or not all(
        isinstance(item, str) and item for item in raw_names
    ):
        raise H0PrimarySplitV3Error("historical actor feature names are malformed")
    expected_names = tuple(raw_names)
    if len(set(expected_names)) != len(expected_names):
        raise H0PrimarySplitV3Error("historical actor feature names are duplicated")
    previous_time = -math.inf
    for index, row in enumerate(trace, start=1):
        if (
            not isinstance(row, Mapping)
            or _require_counter(row.get("step"), f"historical row {index} step")
            != index
        ):
            raise H0PrimarySplitV3Error(
                f"historical seed {seed} has non-contiguous steps"
            )
        _require_finite_vector(
            row.get("actor_observation_vector_before"),
            EXPECTED_ACTOR_FEATURES,
            f"historical seed {seed} row {index} observation",
        )
        action = _require_finite_vector(
            row.get("raw_policy_action"),
            2,
            f"historical seed {seed} row {index} raw action",
        )
        applied = row.get("applied_policy_action")
        _require_finite_vector(
            applied,
            2,
            f"historical seed {seed} row {index} applied action",
        )
        if applied != action:
            raise H0PrimarySplitV3Error(
                f"historical seed {seed} row {index} is malformed"
            )
        row_time = _require_finite_number(
            row.get("time"), f"historical seed {seed} row {index} time"
        )
        if row_time <= previous_time:
            raise H0PrimarySplitV3Error(
                f"historical seed {seed} timestamps are not strictly monotonic"
            )
        previous_time = row_time
    if len(expected_names) != EXPECTED_ACTOR_FEATURES:
        raise H0PrimarySplitV3Error("historical actor schema is not 35 features")
    _require_counter(
        summary.get("invalid_event_count"), "historical invalid_event_count"
    )
    return trace, summary, trace_path, summary_path


def build_env_config(
    *, seed: int, offset_s: float = CANONICAL_OFFSET_S
) -> dict[str, Any]:
    condition = {
        "id": f"v3_semantic_replay_seed{seed}",
        "action_selection": "stochastic",
        "offset_s": float(offset_s),
        "seed": int(seed),
    }
    config = h0_runtime.build_env_config(case_id="A", condition=condition)
    config.update(
        {
            "episode_start_offset_s": float(offset_s),
            "binary_phase_detector_profile_file": None,
            "binary_phase_fsm_mode": "disabled",
            "binary_phase_event_contract_id": "binary_events_disabled_v1",
            "phase_fsm_input_mode": "legacy_events",
            "event_contract_id": EVENT_CONTRACT,
            "record_outputs": False,
            "save_outputs_on_close": False,
        }
    )
    config["reward"] = dict(config["reward"])
    config["reward"]["morphology_weight"] = 0.0
    return config


def _h0_logits(module: Any, observations: Any, torch: Any):
    import numpy as np

    values = np.asarray(observations, dtype=np.float32)
    if values.ndim == 1:
        values = values.reshape(1, -1)
    module.pi.eval()
    with torch.no_grad():
        logits = module.pi(torch.as_tensor(values, dtype=torch.float32))
    return logits.detach().cpu().numpy().astype(np.float32)


def _is_within(path: Path, root: Path) -> bool:
    try:
        path.relative_to(root)
    except ValueError:
        return False
    return True


def _canonical_replay_destination(seed: int) -> Path:
    if seed not in CANONICAL_SEEDS:
        raise H0PrimarySplitV3Error(f"unregistered canonical seed {seed}")
    return RUN_ROOT / "replay" / f"seed_{seed}"


def _canonical_stage_destinations() -> tuple[Path, ...]:
    return tuple(_canonical_replay_destination(seed) for seed in CANONICAL_SEEDS) + (
        RUN_ROOT / "corpus",
        RUN_ROOT / "adaptation",
        RUN_ROOT / "holdout",
    )


def _attempt_claim_payload() -> dict[str, Any]:
    return {
        "schema_version": 3,
        "status": "H0_PRIMARY_SPLIT_V3_EXECUTION_ATTEMPT_CLAIMED",
        "protocol_id": PROTOCOL_ID,
        "run_root": _repo_relative_posix(RUN_ROOT),
        "execution_lock": source_record(LOCK),
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
    }


def _claim_attempt_root() -> Path:
    RUN_ROOT.parent.mkdir(parents=True, exist_ok=True)
    try:
        RUN_ROOT.mkdir()
    except FileExistsError as exc:
        raise H0PrimarySplitV3Error(
            f"V3 execution attempt is already claimed: {RUN_ROOT}"
        ) from exc
    _write_json_exclusive(ATTEMPT_CLAIM, _attempt_claim_payload())
    return ATTEMPT_CLAIM


def _verify_attempt_claim() -> dict[str, Any]:
    claim = _strict_mapping(ATTEMPT_CLAIM)
    expected = _attempt_claim_payload()
    if common_gates.canonical_json_bytes(claim) != common_gates.canonical_json_bytes(
        expected
    ):
        raise H0PrimarySplitV3Error("V3 execution attempt claim drifted")
    return claim


def _claim_destination(
    path: str | Path,
    *,
    diagnostic: bool,
    canonical_destination: Path | None = None,
) -> Path:
    destination = Path(path).expanduser().resolve()
    resolved_run_root = RUN_ROOT.resolve()
    if diagnostic:
        if _is_within(destination, resolved_run_root):
            raise H0PrimarySplitV3Error(
                "diagnostic artifacts must remain outside the V3 run root"
            )
        if canonical_destination is not None:
            raise H0PrimarySplitV3Error(
                "diagnostic output cannot claim a canonical protocol destination"
            )
    else:
        if canonical_destination is None:
            raise H0PrimarySplitV3Error(
                "protocol output requires an exact canonical destination"
            )
        expected = canonical_destination.expanduser().resolve()
        if destination != expected:
            raise H0PrimarySplitV3Error(
                f"non-canonical V3 destination: {destination} != {expected}"
            )
        _verify_attempt_claim()

    destination.parent.mkdir(parents=True, exist_ok=True)
    try:
        destination.mkdir()
    except FileExistsError as exc:
        raise H0PrimarySplitV3Error(
            f"destination is already claimed/no-clobber: {destination}"
        ) from exc
    return destination


def _replay_gate(
    summary: Mapping[str, Any],
    *,
    so_policy_id: str | None = None,
) -> dict[str, Any]:
    def counter(key: str) -> int | None:
        try:
            return _require_counter(summary.get(key), f"summary.{key}")
        except H0PrimarySplitV3Error:
            return None

    def finite(key: str) -> float | None:
        try:
            return _require_finite_number(summary.get(key), f"summary.{key}")
        except H0PrimarySplitV3Error:
            return None

    selected_policy_id = (
        summary.get("so_policy_id") if so_policy_id is None else so_policy_id
    )
    if selected_policy_id is None:
        selected_policy_id = freeze_contract.STRICT_ZERO_POLICY
    policy = freeze_contract.SO_POLICIES.get(selected_policy_id)
    policy_is_valid = isinstance(policy, Mapping)
    raw_fallbacks_allowed = bool(
        policy_is_valid and policy.get("raw_fallback_count_may_be_nonzero") is True
    )
    status0_policy = (
        selected_policy_id == freeze_contract.VERIFIED_STATUS0_MAX_ITER_POLICY
    )
    penetration = finite("grf_penetration_max_m")
    morphology_weight = finite("morphology_weight")
    steps = counter("steps")
    cycles = counter("phase_valid_cycle_count")
    invalid_events = counter("invalid_event_count")
    historical_invalid_events = counter("historical_invalid_event_count")
    bounded_lsq_invocations = counter("so_solver_bounded_ls_invocation_count")
    verified_bounded_lsq = counter("so_solver_verified_bounded_ls_count")
    raw_fallbacks = counter("fallback_count")
    so_fallbacks = counter("so_fallback_count")
    sea_fallbacks = counter("sea_plugin_fallback_count")
    hard_fallbacks = counter("so_solver_hard_fallback_count")
    unsuccessful_bounded = counter("so_solver_bounded_ls_unsuccessful_count")
    verified_status0 = counter("so_solver_verified_status0_max_iter_count")
    unaccepted_hard = counter("so_solver_unaccepted_hard_fallback_count")
    unaccepted_bounded = counter("so_solver_unaccepted_bounded_ls_count")
    no_unaccepted_solver_fallback = bool(
        policy_is_valid
        and unaccepted_hard == 0
        and unaccepted_bounded == 0
        and sea_fallbacks == 0
        and raw_fallbacks is not None
        and so_fallbacks is not None
        and raw_fallbacks == so_fallbacks + sea_fallbacks
        and verified_bounded_lsq is not None
        and so_fallbacks <= verified_bounded_lsq
    )
    checks = {
        "steps_500": steps == EXPECTED_STEPS,
        "episode_time_limit": summary.get("end_reason") == "episode_time_limit",
        "not_terminated": type(summary.get("terminated")) is bool
        and summary.get("terminated") is False,
        "truncated": type(summary.get("truncated")) is bool
        and summary.get("truncated") is True,
        "cycles_at_least_two": cycles is not None and cycles >= 2,
        "penetration_below_25mm": penetration is not None
        and 0.0 <= penetration < 0.025,
        "zero_clipping": counter("action_clipped_values") == 0,
        "zero_timeouts": counter("timeout_count") == 0,
        "zero_safety_stops": counter("safety_stop_count") == 0,
        "zero_fallbacks": raw_fallbacks == 0
        or (raw_fallbacks_allowed and no_unaccepted_solver_fallback),
        "zero_hard_so_fallbacks": hard_fallbacks == 0
        or (
            status0_policy
            and no_unaccepted_solver_fallback
            and hard_fallbacks is not None
            and hard_fallbacks == verified_status0
        ),
        "zero_reuse_previous": counter("so_solver_reuse_previous_count") == 0,
        "zero_bounded_lsq_failures": unsuccessful_bounded == 0
        or (
            status0_policy
            and no_unaccepted_solver_fallback
            and unsuccessful_bounded is not None
            and unsuccessful_bounded == verified_status0
        ),
        "zero_solver_bounds_violations": counter("so_solver_bounds_violation_count")
        == 0,
        "zero_solver_nonfinite": counter("so_solver_nonfinite_count") == 0,
        "zero_selected_infeasible": counter("so_solver_selected_infeasible_count") == 0,
        "zero_selected_solution_mismatch": counter(
            "so_solver_selected_solution_mismatch_count"
        )
        == 0,
        "zero_residual_contract_mismatch": counter(
            "so_solver_residual_contract_mismatch_count"
        )
        == 0,
        "all_bounded_lsq_verified": bounded_lsq_invocations is not None
        and verified_bounded_lsq is not None
        and bounded_lsq_invocations == verified_bounded_lsq,
        "zero_hard_invalid": counter("hard_invalid_count") == 0,
        "zero_nonfinite": counter("nonfinite_count") == 0,
        "legacy_invalid_matches_history": invalid_events is not None
        and invalid_events == historical_invalid_events,
        "fixed_features_bit_exact": counter("fixed_feature_mismatch_count") == 0,
        "analog_teacher_view_bit_exact": counter("teacher_view_mismatch_count") == 0,
        "times_match_history": counter("time_mismatch_count") == 0,
        "primary_view_differs": (counter("mutable_feature_difference_count") or 0) > 0,
        "actor_layout_35": counter("n_actor") == EXPECTED_ACTOR_FEATURES,
        "observation_layout_84": counter("n_observation") == EXPECTED_FULL_FEATURES,
        "float32_observation": summary.get("observation_dtype") == "float32",
        "primary_left_only": summary.get("online_grf_applied_sides") == ["left"],
        "v25_disabled": summary.get("binary_phase_fsm_mode") == "disabled",
        "event_contract": summary.get("event_contract_id") == EVENT_CONTRACT,
        "morphology_zero": morphology_weight == 0.0,
        "h0_not_behavior": type(summary.get("h0_used_for_behavior")) is bool
        and summary.get("h0_used_for_behavior") is False,
        "no_ppo": counter("ppo_updates") == 0,
    }
    passed = all(checks.values())
    return {
        "schema_version": 3,
        "so_policy_id": selected_policy_id,
        "status": "PASS_H0_PRIMARY_SPLIT_V3_REPLAY"
        if passed
        else "FAIL_H0_PRIMARY_SPLIT_V3_REPLAY",
        "passed": passed,
        "checks": checks,
    }


def run_replay(
    *,
    seed: int,
    output_dir: str | Path,
    diagnostic: bool,
    max_steps: int = EXPECTED_STEPS,
) -> dict[str, Any]:
    selected_so_policy_id = freeze_contract.STRICT_ZERO_POLICY
    if diagnostic and seed not in TRAIN_SEEDS:
        raise H0PrimarySplitV3Error(
            "diagnostic replay is restricted to the preregistered train seeds"
        )
    if not diagnostic:
        execution_lock = verify_lock()
        selected_so_policy_id = execution_lock.get("so_policy_id")
        if selected_so_policy_id not in freeze_contract.SO_POLICIES:
            raise H0PrimarySplitV3Error(
                "V3 execution lock contains an unknown SO policy"
            )
        if seed == FINAL_HOLDOUT_SEED:
            verify_candidate_freeze()
            verify_holdout_access_claim()
        elif seed not in TRAIN_SEEDS:
            raise H0PrimarySplitV3Error(
                "non-holdout replay seed is not train-authorized"
            )
    destination = _claim_destination(
        output_dir,
        diagnostic=diagnostic,
        canonical_destination=(
            None if diagnostic else _canonical_replay_destination(seed)
        ),
    )
    rollout_eval, np, torch, RLModule, env_factory = v1._load_stack()
    trace_rows, historical_summary, trace_path, historical_summary_path = (
        historical_inputs(seed)
    )
    historical_observations_raw = np.asarray(
        [row["actor_observation_vector_before"] for row in trace_rows]
    )
    actions_raw = np.asarray([row["raw_policy_action"] for row in trace_rows])
    if not np.all(np.isfinite(historical_observations_raw)) or not np.all(
        np.isfinite(actions_raw)
    ):
        raise H0PrimarySplitV3Error("historical replay arrays are non-finite")
    historical_observations = np.ascontiguousarray(
        historical_observations_raw, dtype=np.float32
    )
    actions = np.ascontiguousarray(actions_raw, dtype=np.float32)
    for label, values in (
        ("historical observations", historical_observations),
        ("historical actions", actions),
    ):
        if not np.all(np.isfinite(values)):
            raise H0PrimarySplitV3Error(f"{label} overflowed float32")
    h0_module = RLModule.from_checkpoint(H0_MODULE.resolve())
    teacher_logits = _h0_logits(h0_module, historical_observations, torch)
    teacher_means = np.ascontiguousarray(teacher_logits[:, :2], dtype=np.float32)
    teacher_logstd = np.ascontiguousarray(teacher_logits[:, 2:], dtype=np.float32)
    if teacher_logits.shape != (EXPECTED_STEPS, 4) or not np.all(
        np.isfinite(teacher_logits)
    ):
        raise H0PrimarySplitV3Error("H0 teacher logits are malformed/non-finite")
    expected_logstd = np.full_like(teacher_logstd, math.log(EXPECTED_SIGMA))
    if not np.allclose(teacher_logstd, expected_logstd, rtol=0.0, atol=1.0e-7):
        raise H0PrimarySplitV3Error("H0 logstd logits drifted from sigma 0.005")

    config = build_env_config(seed=seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    env = env_factory.make_cmc_env(config)
    target_observations: list[Any] = []
    reconstructed_teacher_observations: list[Any] = []
    times: list[float] = []
    rewards: list[float] = []
    reserve = h0_runtime._empty_accumulator()
    residual = h0_runtime._empty_accumulator()
    sea = h0_runtime._sea_accumulators()
    penetrations: list[float] = []
    clipping_values = 0
    timeout_count = 0
    fallback_count = 0
    so_fallback_count = 0
    sea_plugin_fallback_count = 0
    fallback_journal: list[dict[str, Any]] = []
    solver_audit_journal: list[dict[str, Any]] = []
    solver_audit_attempt_count = 0
    solver_audit_hard_fallback_count = 0
    solver_recovery_totals = {key: 0 for key in SO_RECOVERY_COUNTER_KEYS}
    hard_invalid_count = 0
    invalid_event_count = 0
    fixed_feature_mismatch_count = 0
    teacher_view_mismatch_count = 0
    mutable_feature_difference_count = 0
    time_mismatch_count = 0
    terminated = False
    truncated = False
    final_info: dict[str, Any] = {}
    started = time.monotonic()
    try:
        observation, reset_info = env.reset(seed=seed)
        if not isinstance(reset_info, Mapping):
            raise H0PrimarySplitV3Error("reset info must be an object")
        current_info = dict(reset_info)
        observation = _require_array_contract(
            observation,
            np=np,
            label="reset observation",
            dtype=np.float32,
            shape=(EXPECTED_FULL_FEATURES,),
        )
        observation_dtype = str(observation.dtype)
        base = env.unwrapped
        actor_names = tuple(str(name) for name in base.actor_feature_names)
        full_names = tuple(str(name) for name in base.observation_feature_names)
        rollout_eval._validate_module_observation_contract(
            h0_module, actor_names, full_names
        )
        v1._validate_reset_contract(
            module=h0_module,
            env=env,
            observation=observation,
            actor_names=actor_names,
            full_names=full_names,
        )
        if (
            abs(
                _require_finite_number(reset_info.get("time"), "reset_info.time")
                - (11.99 + CANONICAL_OFFSET_S)
            )
            > 1e-9
        ):
            raise H0PrimarySplitV3Error("canonical absolute reset time drifted")
        if list(base.cfg.online_grf_applied_sides) != ["left"]:
            raise H0PrimarySplitV3Error("primary GRF routing is not left-only")
        if base.env_cfg.binary_phase_detector_profile_file is not None:
            raise H0PrimarySplitV3Error("V25 is loaded during semantic replay")
        mutable_indices = tuple(
            actor_names.index(name) for name in MUTABLE_FEATURE_NAMES
        )
        fixed_indices = tuple(
            index
            for index in range(EXPECTED_ACTOR_FEATURES)
            if index not in mutable_indices
        )
        if mutable_indices != (10, 11):
            raise H0PrimarySplitV3Error(
                f"mutable feature indices drifted: {mutable_indices}"
            )

        shadow_fsm = copy.deepcopy(base._phase_fsm)
        body_weight_n = _require_finite_number(
            base._body_weight_n, "environment body weight"
        )
        if body_weight_n <= 0.0:
            raise H0PrimarySplitV3Error("environment body weight must be positive")
        if max_steps <= 0 or max_steps > EXPECTED_STEPS:
            raise H0PrimarySplitV3Error("max_steps must be in [1, 500]")
        for step_index in range(max_steps):
            target = np.ascontiguousarray(observation[:EXPECTED_ACTOR_FEATURES]).copy()
            historical = historical_observations[step_index]
            paired = split_contract.build_paired_views(
                observation,
                actor_names,
                current_info,
                body_weight_n=body_weight_n,
                reset_boundary=step_index == 0,
                teacher_phase_observation=shadow_fsm.observation(),
            )
            paired_student = _require_array_contract(
                paired.student,
                np=np,
                label="paired student observation",
                dtype=np.float32,
                shape=(EXPECTED_ACTOR_FEATURES,),
            )
            paired_teacher = _require_array_contract(
                paired.teacher,
                np=np,
                label="paired teacher observation",
                dtype=np.float32,
                shape=(EXPECTED_ACTOR_FEATURES,),
            )
            if paired_student.tobytes() != target.tobytes():
                raise H0PrimarySplitV3Error(
                    "paired student view differs from runtime actor prefix"
                )
            if paired_teacher.tobytes() != historical.tobytes():
                teacher_view_mismatch_count += 1
            reconstructed_teacher_observations.append(paired_teacher.copy())
            if (
                target[list(fixed_indices)].tobytes()
                != historical[list(fixed_indices)].tobytes()
            ):
                fixed_feature_mismatch_count += 1
            if (
                target[list(mutable_indices)].tobytes()
                != historical[list(mutable_indices)].tobytes()
            ):
                mutable_feature_difference_count += 1
            target_observations.append(target)
            raw_action = actions[step_index].copy()
            applied = np.clip(
                raw_action, env.action_space.low, env.action_space.high
            ).astype(np.float32)
            clipping_values += int(np.count_nonzero(applied != raw_action))
            observation, reward, terminated, truncated, info = env.step(raw_action)
            observation = _require_array_contract(
                observation,
                np=np,
                label=f"step {step_index + 1} observation",
                dtype=np.float32,
                shape=(EXPECTED_FULL_FEATURES,),
            )
            if str(observation.dtype) != observation_dtype:
                raise H0PrimarySplitV3Error("runtime observation dtype drifted")
            _require_bool(terminated, f"step {step_index + 1} terminated")
            _require_bool(truncated, f"step {step_index + 1} truncated")
            if not isinstance(info, Mapping):
                raise H0PrimarySplitV3Error("runtime info must be an object")
            if info.get("event_contract_id") != EVENT_CONTRACT:
                raise H0PrimarySplitV3Error("runtime event contract drifted")
            if info.get("online_grf_applied_sides") != ["left"]:
                raise H0PrimarySplitV3Error("runtime GRF routing drifted")
            observed_time = _require_finite_number(
                info.get("time"), f"step {step_index + 1} time"
            )
            historical_time = _require_finite_number(
                trace_rows[step_index].get("time"),
                f"historical step {step_index + 1} time",
            )
            time_mismatch_count += int(abs(observed_time - historical_time) > 1e-9)
            times.append(observed_time)
            rewards.append(
                _require_finite_number(reward, f"step {step_index + 1} reward")
            )
            terms = info.get("reward_terms")
            if not isinstance(terms, Mapping):
                raise H0PrimarySplitV3Error("reward terms are missing")
            h0_runtime._accumulate_scalar(reserve, terms["reserve_norm_nm"])
            h0_runtime._accumulate_scalar(residual, terms["residual_norm_nm"])
            penetration = _require_finite_number(
                terms.get("grf_penetration_m"),
                f"step {step_index + 1} grf penetration",
            )
            if penetration < 0.0:
                raise H0PrimarySplitV3Error("penetration is not finite/non-negative")
            penetrations.append(penetration)
            sea_payload = info.get("sea_segment_diagnostics")
            if not isinstance(sea_payload, Mapping) or not isinstance(
                sea_payload.get("joints"), Mapping
            ):
                raise H0PrimarySplitV3Error("SEA fallback diagnostics are missing")
            step_sea_fallback = 0
            for joint in common_gates.JOINTS:
                joint_payload = sea_payload["joints"].get(joint)
                if not isinstance(joint_payload, Mapping):
                    raise H0PrimarySplitV3Error(
                        f"SEA fallback diagnostics missing {joint}"
                    )
                step_sea_fallback += _require_counter(
                    joint_payload.get("tau_input_plugin_fallback_count"),
                    f"step {step_index + 1} {joint} tau-input fallback count",
                ) + _require_counter(
                    joint_payload.get("motor_accel_plugin_fallback_count"),
                    f"step {step_index + 1} {joint} motor-accel fallback count",
                )
            h0_runtime._accumulate_sea(sea, sea_payload)
            sea_plugin_fallback_count += step_sea_fallback
            phase = info.get("phase_fsm")
            if not isinstance(phase, Mapping):
                raise H0PrimarySplitV3Error("phase FSM diagnostics are missing")
            timeout_value = _require_integral_number(
                phase.get("timeout_exceeded"),
                f"step {step_index + 1} timeout_exceeded",
            )
            if timeout_value not in {0, 1}:
                raise H0PrimarySplitV3Error("timeout_exceeded must be binary")
            timeout_count += timeout_value
            invalid_event_count = max(
                invalid_event_count,
                _require_integral_number(
                    phase.get("invalid_event_count"),
                    f"step {step_index + 1} invalid_event_count",
                ),
            )
            so = info.get("so_diagnostics")
            if not isinstance(so, Mapping) or "solver_fallback_used" not in so:
                raise H0PrimarySplitV3Error("SO fallback diagnostic is missing")
            step_so_fallback = int(
                _require_bool(
                    so["solver_fallback_used"],
                    f"step {step_index + 1} solver_fallback_used",
                )
            )
            so_fallback_count += step_so_fallback
            audit_entries, audit_attempts, audit_hard_fallbacks = (
                _validate_so_solver_audit_entries(
                    info.get("so_solver_audit_entries"),
                    step_index=step_index + 1,
                    selected_fallback=bool(step_so_fallback),
                )
            )
            solver_audit_attempt_count += audit_attempts
            solver_audit_hard_fallback_count += audit_hard_fallbacks
            recovery = so_recovery.classify_policy_step(
                info.get("so_solver_audit_entries"),
                policy_id=selected_so_policy_id,
            )
            for key in SO_RECOVERY_COUNTER_KEYS:
                solver_recovery_totals[key] += _require_counter(
                    recovery["counters"].get(key),
                    f"step {step_index + 1} SO recovery counter {key}",
                )
            if (
                recovery["counters"]["control_window_count"] != len(audit_entries)
                or recovery["counters"]["solver_invocation_count"] != audit_attempts
                or recovery["counters"]["hard_so_fallback_count"]
                != audit_hard_fallbacks
            ):
                raise H0PrimarySplitV3Error(
                    "SO audit classifiers disagree on runtime telemetry"
                )
            solver_audit_journal.append(
                {
                    "step": step_index + 1,
                    "time_s": observed_time,
                    "control_windows": audit_entries,
                }
            )
            if step_so_fallback or step_sea_fallback:
                fallback_journal.append(
                    {
                        "step": step_index + 1,
                        "time_s": observed_time,
                        "so_fallback": step_so_fallback,
                        "sea_plugin_fallback_count": step_sea_fallback,
                    }
                )
            hard_invalid_count += int("failure" in info)
            v1._update_shadow_fsm(
                shadow_fsm,
                info=info,
                body_weight_n=body_weight_n,
            )
            final_info = dict(info)
            current_info = dict(info)
            completed = step_index + 1
            if completed == 1 or completed % 25 == 0 or completed == EXPECTED_STEPS:
                elapsed = time.monotonic() - started
                eta = elapsed / completed * (EXPECTED_STEPS - completed)
                print(
                    f"[v3/replay/seed{seed}] {completed:3d}/{EXPECTED_STEPS} "
                    f"elapsed={elapsed:7.1f}s eta={eta:7.1f}s",
                    flush=True,
                )
            if terminated or truncated:
                break
    finally:
        env.close()

    sea_metrics = h0_runtime._finalize_sea(sea)
    aggregate_sea_fallback_count = sum(
        _require_counter(
            sea[joint]["fallback_count"], f"aggregate SEA fallback count {joint}"
        )
        for joint in common_gates.JOINTS
    )
    if aggregate_sea_fallback_count != sea_plugin_fallback_count:
        raise H0PrimarySplitV3Error("SEA fallback aggregate is internally inconsistent")
    fallback_count = so_fallback_count + sea_plugin_fallback_count
    phase = final_info.get("phase_fsm", {})
    if not isinstance(phase, Mapping):
        raise H0PrimarySplitV3Error("final phase FSM diagnostics are missing")
    target_array = _require_array_contract(
        np.asarray(target_observations),
        np=np,
        label="target observation replay",
        dtype=np.float32,
        shape=(len(target_observations), EXPECTED_ACTOR_FEATURES),
    )
    reconstructed_teacher_array = _require_array_contract(
        np.asarray(reconstructed_teacher_observations),
        np=np,
        label="reconstructed teacher replay",
        dtype=np.float32,
        shape=(len(target_observations), EXPECTED_ACTOR_FEATURES),
    )
    times_array = _require_array_contract(
        np.asarray(times, dtype=np.float64),
        np=np,
        label="replay times",
        dtype=np.float64,
        shape=(len(target_observations),),
    )
    rewards_array = _require_array_contract(
        np.asarray(rewards, dtype=np.float64),
        np=np,
        label="replay rewards",
        dtype=np.float64,
        shape=(len(target_observations),),
    )
    finite_count = 0
    summary = {
        "schema_version": 3,
        "so_policy_id": selected_so_policy_id,
        "seed": seed,
        "steps": len(target_array),
        "end_reason": final_info.get("end_reason"),
        "terminated": bool(terminated),
        "truncated": bool(truncated),
        "phase_valid_cycle_count": _require_integral_number(
            phase.get("valid_cycle_count"), "final valid_cycle_count"
        ),
        "invalid_event_count": invalid_event_count,
        "historical_invalid_event_count": _require_counter(
            historical_summary.get("invalid_event_count"),
            "historical invalid_event_count",
        ),
        "grf_penetration_max_m": max(penetrations, default=0.0),
        "action_clipped_values": clipping_values,
        "timeout_count": timeout_count,
        "safety_stop_count": int(bool(terminated)),
        "fallback_count": fallback_count,
        "so_fallback_count": so_fallback_count,
        "sea_plugin_fallback_count": sea_plugin_fallback_count,
        "so_solver_control_window_count": sum(
            len(row["control_windows"]) for row in solver_audit_journal
        ),
        "so_solver_attempt_count": solver_audit_attempt_count,
        "so_solver_hard_fallback_count": solver_audit_hard_fallback_count,
        "so_solver_primary_nonconvergence_count": solver_recovery_totals[
            "primary_solver_nonconvergence_count"
        ],
        "so_solver_bounded_ls_invocation_count": solver_recovery_totals[
            "bounded_ls_invocation_count"
        ],
        "so_solver_selected_bounded_ls_count": solver_recovery_totals[
            "selected_bounded_ls_count"
        ],
        "so_solver_verified_bounded_ls_count": solver_recovery_totals[
            "verified_bounded_ls_count"
        ],
        "so_solver_verified_bounded_ls_success_count": solver_recovery_totals[
            "verified_bounded_ls_success_count"
        ],
        "so_solver_verified_status0_max_iter_count": solver_recovery_totals[
            "verified_status0_max_iter_count"
        ],
        "so_solver_unaccepted_hard_fallback_count": solver_recovery_totals[
            "unaccepted_hard_so_fallback_count"
        ],
        "so_solver_unaccepted_bounded_ls_count": solver_recovery_totals[
            "unaccepted_bounded_ls_count"
        ],
        "so_solver_reuse_previous_count": solver_recovery_totals[
            "reuse_previous_count"
        ],
        "so_solver_bounded_ls_unsuccessful_count": solver_recovery_totals[
            "bounded_ls_unsuccessful_count"
        ],
        "so_solver_bounds_violation_count": solver_recovery_totals[
            "bounds_violation_count"
        ],
        "so_solver_nonfinite_count": solver_recovery_totals["nonfinite_solver_count"],
        "so_solver_selected_infeasible_count": solver_recovery_totals[
            "selected_infeasible_count"
        ],
        "so_solver_selected_solution_mismatch_count": solver_recovery_totals[
            "selected_solution_mismatch_count"
        ],
        "so_solver_residual_contract_mismatch_count": solver_recovery_totals[
            "residual_contract_mismatch_count"
        ],
        "so_solver_audit_schema": "static_optimization_solver_audit_v1",
        "hard_invalid_count": hard_invalid_count,
        "nonfinite_count": finite_count,
        "fixed_feature_mismatch_count": fixed_feature_mismatch_count,
        "teacher_view_mismatch_count": teacher_view_mismatch_count,
        "mutable_feature_difference_count": mutable_feature_difference_count,
        "time_mismatch_count": time_mismatch_count,
        "n_actor": len(actor_names),
        "n_observation": len(full_names),
        "observation_dtype": observation_dtype,
        "actor_feature_names": list(actor_names),
        "observation_feature_names": list(full_names),
        "event_contract_id": EVENT_CONTRACT,
        "binary_phase_fsm_mode": "disabled",
        "online_grf_applied_sides": list(
            final_info.get("online_grf_applied_sides", [])
        ),
        "morphology_weight": float(config["reward"]["morphology_weight"]),
        "episode_return": float(sum(rewards)),
        "episode_metrics": {
            "reserve_norm_nm": h0_runtime._finalize_accumulator(reserve),
            "residual_norm_nm": h0_runtime._finalize_accumulator(residual),
        },
        "sea_episode_metrics": sea_metrics,
        "h0_used_for_behavior": False,
        "historical_action_replay_used_for_behavior": True,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
    }
    arrays_path = _write_npz_exclusive(
        destination / "paired_replay.npz",
        target_observations=target_array,
        reconstructed_teacher_observations=reconstructed_teacher_array,
        historical_observations=historical_observations,
        teacher_means=teacher_means,
        teacher_logstd=teacher_logstd,
        replay_actions=actions,
        times=times_array,
        rewards=rewards_array,
        actor_feature_names=np.asarray(actor_names, dtype="U64"),
    )
    summary_path = _write_json_exclusive(destination / "summary.json", summary)
    gate = _replay_gate(summary, so_policy_id=selected_so_policy_id)
    gate_path = _write_json_exclusive(destination / "gate.json", gate)
    fallback_path = _write_json_exclusive(
        destination / "fallback_journal.json", fallback_journal
    )
    solver_audit_path = _write_json_exclusive(
        destination / "solver_audit_journal.json", solver_audit_journal
    )

    def artifact_record(path: Path) -> dict[str, Any]:
        if _is_within(path.resolve(), REPO_ROOT.resolve()):
            return source_record(path)
        return _external_record(path)

    receipt = {
        "schema_version": 3,
        "status": gate["status"],
        "passed": gate["passed"],
        "seed": seed,
        "diagnostic": diagnostic,
        "execution_lock": None if diagnostic else source_record(LOCK),
        "attempt_claim": None if diagnostic else source_record(ATTEMPT_CLAIM),
        "artifacts": {
            "arrays": artifact_record(arrays_path),
            "summary": artifact_record(summary_path),
            "gate": artifact_record(gate_path),
            "fallback_journal": artifact_record(fallback_path),
            "solver_audit_journal": artifact_record(solver_audit_path),
            "historical_trace": source_record(trace_path),
            "historical_summary": source_record(historical_summary_path),
        },
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
    }
    _write_json_exclusive(destination / "receipt.json", receipt)
    print(json.dumps(receipt, indent=2, sort_keys=True, allow_nan=False), flush=True)
    if not gate["passed"]:
        raise H0PrimarySplitV3Error(f"semantic replay seed {seed} failed")
    return receipt


def verify_lock() -> dict[str, Any]:
    lock = _strict_mapping(LOCK)
    if set(lock) != LOCK_TOP_LEVEL_KEYS:
        raise H0PrimarySplitV3Error("V3 execution lock top-level schema drifted")
    selected_so_policy_id = lock.get("so_policy_id")
    if selected_so_policy_id not in freeze_contract.SO_POLICIES:
        raise H0PrimarySplitV3Error("V3 execution lock SO policy is unknown")
    exact_values = {
        "schema_version": 3,
        "status": "H0_PRIMARY_GRF_SPLIT_V3_EXECUTION_FROZEN",
        "protocol_id": PROTOCOL_ID,
        "revision": REVISION,
        "candidate_id": CANDIDATE_ID,
        "run_root": _repo_relative_posix(RUN_ROOT),
        "expected_steps": EXPECTED_STEPS,
        "canonical_seeds": list(CANONICAL_SEEDS),
        "canonical_offset_s": CANONICAL_OFFSET_S,
        "event_contract_id": EVENT_CONTRACT,
        "so_policy_id": selected_so_policy_id,
        "so_policy": freeze_contract.SO_POLICIES[selected_so_policy_id],
        "fit": FIT,
        "mutable_feature_names": list(MUTABLE_FEATURE_NAMES),
        "destinations": [
            _repo_relative_posix(path) for path in _canonical_stage_destinations()
        ],
        "authority": LOCK_AUTHORITY,
        "actor_update_candidate_count": 1,
        "retry_or_retuning_allowed": False,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
    }
    for key, expected in exact_values.items():
        if common_gates.canonical_json_bytes(lock.get(key)) != (
            common_gates.canonical_json_bytes(expected)
        ):
            raise H0PrimarySplitV3Error(f"V3 execution lock drifted at {key}")

    if (
        tuple(TRAIN_SEEDS) != tuple(freeze_contract.TRAIN_SEEDS)
        or FINAL_HOLDOUT_SEED != freeze_contract.FINAL_HOLDOUT_SEED
    ):
        raise H0PrimarySplitV3Error("V3 runner/freeze seed contract drifted")

    def expected_repo_path(relative: str) -> Path:
        pure = PurePosixPath(relative)
        if pure.is_absolute() or ".." in pure.parts or pure.as_posix() != relative:
            raise H0PrimarySplitV3Error(
                f"freeze contract contains a non-canonical path: {relative}"
            )
        return REPO_ROOT.joinpath(*pure.parts)

    def verify_exact_record_map(
        value: Any,
        expected_relative_paths: Mapping[str, str],
        label: str,
    ) -> None:
        if not isinstance(value, Mapping) or set(value) != set(expected_relative_paths):
            raise H0PrimarySplitV3Error(f"{label} identity schema drifted")
        for key, relative in expected_relative_paths.items():
            _verified_record_path(
                value[key], f"{label}.{key}", expected_repo_path(relative)
            )

    sources = lock.get("sources")
    if not isinstance(sources, Mapping):
        raise H0PrimarySplitV3Error("V3 lock source identity schema drifted")
    verify_exact_record_map(
        sources,
        freeze_contract.LOCK_SOURCE_RELATIVE_PATHS,
        "sources",
    )
    if (
        expected_repo_path(
            freeze_contract.LOCK_SOURCE_RELATIVE_PATHS["protocol_plan"]
        ).resolve()
        != PLAN.resolve()
        or expected_repo_path(
            freeze_contract.LOCK_SOURCE_RELATIVE_PATHS["runner"]
        ).resolve()
        != RUNNER.resolve()
        or expected_repo_path(
            freeze_contract.LOCK_SOURCE_RELATIVE_PATHS["tests"]
        ).resolve()
        != TESTS.resolve()
    ):
        raise H0PrimarySplitV3Error("V3 canonical source paths drifted")

    inputs = lock.get("inputs")
    if not isinstance(inputs, Mapping) or set(inputs) != {
        "h0_config",
        "h0_module_state",
        "h0_module_ctor",
        "h0_module_metadata",
        "historical_inputs",
    }:
        raise H0PrimarySplitV3Error("V3 lock input identity schema drifted")
    _verified_record_path(inputs["h0_config"], "inputs.h0_config", H0_CONFIG)
    _verified_record_path(
        inputs["h0_module_state"],
        "inputs.h0_module_state",
        H0_MODULE / "module_state.pkl",
    )
    _verified_record_path(
        inputs["h0_module_ctor"],
        "inputs.h0_module_ctor",
        H0_MODULE / "class_and_ctor_args.pkl",
    )
    _verified_record_path(
        inputs["h0_module_metadata"],
        "inputs.h0_module_metadata",
        H0_MODULE / "metadata.json",
    )
    histories = inputs["historical_inputs"]
    if not isinstance(histories, Mapping) or set(histories) != {
        str(seed) for seed in CANONICAL_SEEDS
    }:
        raise H0PrimarySplitV3Error("V3 historical input identity schema drifted")
    for seed in CANONICAL_SEEDS:
        history = histories[str(seed)]
        if not isinstance(history, Mapping) or set(history) != {"trace", "summary"}:
            raise H0PrimarySplitV3Error(
                f"V3 historical input schema drifted for seed {seed}"
            )
        directory = _historical_directory(seed)
        _verified_record_path(
            history["trace"],
            f"inputs.historical_inputs.{seed}.trace",
            directory / "rollout_policy_trace.json",
        )
        _verified_record_path(
            history["summary"],
            f"inputs.historical_inputs.{seed}.summary",
            directory / "rollout_summary.json",
        )

    runtime_closure = lock.get("runtime_closure")
    expected_closure_sections = {
        "protocol_sources",
        "runtime_sources",
        "runtime_inputs",
        "evidence_receipts",
        "diagnostic_artifacts",
        "lineage",
    }
    if not isinstance(runtime_closure, Mapping) or set(runtime_closure) != (
        expected_closure_sections
    ):
        raise H0PrimarySplitV3Error("V3 runtime closure schema drifted")
    for section_name, paths in (
        (
            "protocol_sources",
            freeze_contract.PROTOCOL_SOURCE_RELATIVE_PATHS,
        ),
        ("runtime_sources", freeze_contract.RUNTIME_SOURCE_RELATIVE_PATHS),
        ("runtime_inputs", freeze_contract.RUNTIME_INPUT_RELATIVE_PATHS),
        ("evidence_receipts", freeze_contract.EVIDENCE_RELATIVE_PATHS),
        ("lineage", freeze_contract.LINEAGE_RELATIVE_PATHS),
    ):
        verify_exact_record_map(
            runtime_closure[section_name], paths, f"runtime_closure.{section_name}"
        )
    diagnostic_artifacts = runtime_closure["diagnostic_artifacts"]
    expected_diagnostics = freeze_contract.diagnostic_artifact_relative_paths()
    if not isinstance(diagnostic_artifacts, Mapping) or set(
        diagnostic_artifacts
    ) != set(expected_diagnostics):
        raise H0PrimarySplitV3Error("V3 diagnostic closure run set drifted")
    for run_id, paths in expected_diagnostics.items():
        verify_exact_record_map(
            diagnostic_artifacts[run_id],
            paths,
            f"runtime_closure.diagnostic_artifacts.{run_id}",
        )

    unique_records: set[str] = set()

    def verify_tree(value: Any, label: str) -> None:
        if not isinstance(value, Mapping):
            raise H0PrimarySplitV3Error(f"{label} must be a record tree")
        if set(value) == {"path", "sha256", "size_bytes"}:
            artifact = _verified_record_path(value, label)
            unique_records.add(_repo_relative_posix(artifact))
            return
        if not value:
            raise H0PrimarySplitV3Error(f"{label} record tree is empty")
        for key, child in value.items():
            if not isinstance(key, str) or not key:
                raise H0PrimarySplitV3Error(f"{label} has a malformed key")
            verify_tree(child, f"{label}.{key}")

    for section_name, section in (
        ("sources", sources),
        ("inputs", inputs),
        ("runtime_closure", runtime_closure),
    ):
        verify_tree(section, section_name)
    if len(unique_records) < 35:
        raise H0PrimarySplitV3Error(
            f"V3 frozen closure is incomplete: {len(unique_records)} unique records"
        )
    return lock


def _verified_record_path(
    record: Any,
    label: str,
    expected_path: str | Path | None = None,
) -> Path:
    if not isinstance(record, Mapping) or set(record) != {
        "path",
        "sha256",
        "size_bytes",
    }:
        raise H0PrimarySplitV3Error(f"{label} record is malformed")
    raw_path = record.get("path")
    if not isinstance(raw_path, str) or not raw_path:
        raise H0PrimarySplitV3Error(f"{label}.path must be repository-relative")
    if (
        "\\" in raw_path
        or raw_path.startswith("/")
        or ":" in raw_path
        or any(part in {"", ".", ".."} for part in raw_path.split("/"))
    ):
        raise H0PrimarySplitV3Error(f"{label}.path is not canonical POSIX-relative")
    portable = PurePosixPath(raw_path)
    if portable.is_absolute() or portable.as_posix() != raw_path:
        raise H0PrimarySplitV3Error(f"{label}.path is not canonical POSIX-relative")
    path = REPO_ROOT.joinpath(*portable.parts).resolve()
    try:
        path.relative_to(REPO_ROOT)
    except ValueError as exc:
        raise H0PrimarySplitV3Error(f"{label} escapes repository") from exc
    if expected_path is not None and path != Path(expected_path).expanduser().resolve():
        raise H0PrimarySplitV3Error(f"{label} points to a non-canonical path")
    expected_size = _require_counter(record.get("size_bytes"), f"{label}.size_bytes")
    expected_sha = _require_sha256(record.get("sha256"), f"{label}.sha256")
    if not path.is_file() or path.stat().st_size != expected_size:
        raise H0PrimarySplitV3Error(f"{label} artifact drifted")
    if sha256_file(path) != expected_sha:
        raise H0PrimarySplitV3Error(f"{label} artifact drifted")
    return path


def _path_from_record(record: Any, label: str) -> Path:
    return _verified_record_path(record, label)


def _validate_replay_receipt(
    seed: int,
) -> tuple[dict[str, Any], dict[str, Path]]:
    replay_dir = _canonical_replay_destination(seed)
    receipt_path = replay_dir / "receipt.json"
    receipt = _strict_mapping(receipt_path)
    if set(receipt) != {
        "schema_version",
        "status",
        "passed",
        "seed",
        "diagnostic",
        "execution_lock",
        "attempt_claim",
        "artifacts",
        "actor_updates",
        "critic_updates",
        "ppo_updates",
        "protected_trials_opened",
    }:
        raise H0PrimarySplitV3Error(f"replay seed {seed} receipt schema drifted")
    if (
        _require_counter(receipt["schema_version"], "replay schema_version") != 3
        or receipt.get("status") != "PASS_H0_PRIMARY_SPLIT_V3_REPLAY"
        or _require_bool(receipt.get("passed"), "replay passed") is not True
        or _require_counter(receipt.get("seed"), "replay seed") != seed
        or _require_bool(receipt.get("diagnostic"), "replay diagnostic") is not False
        or _require_counter(receipt.get("actor_updates"), "replay actor_updates") != 0
        or _require_counter(receipt.get("critic_updates"), "replay critic_updates") != 0
        or _require_counter(receipt.get("ppo_updates"), "replay ppo_updates") != 0
        or receipt.get("protected_trials_opened") != []
    ):
        raise H0PrimarySplitV3Error(f"replay seed {seed} receipt is non-canonical")
    _verified_record_path(
        receipt.get("execution_lock"), f"replay seed {seed} execution lock", LOCK
    )
    _verified_record_path(
        receipt.get("attempt_claim"),
        f"replay seed {seed} attempt claim",
        ATTEMPT_CLAIM,
    )
    artifacts = receipt.get("artifacts")
    expected_artifact_names = {
        "arrays",
        "summary",
        "gate",
        "fallback_journal",
        "solver_audit_journal",
        "historical_trace",
        "historical_summary",
    }
    if not isinstance(artifacts, Mapping) or set(artifacts) != expected_artifact_names:
        raise H0PrimarySplitV3Error(f"replay seed {seed} artifacts schema drifted")
    expected_paths = {
        "arrays": replay_dir / "paired_replay.npz",
        "summary": replay_dir / "summary.json",
        "gate": replay_dir / "gate.json",
        "fallback_journal": replay_dir / "fallback_journal.json",
        "solver_audit_journal": replay_dir / "solver_audit_journal.json",
        "historical_trace": _historical_directory(seed) / "rollout_policy_trace.json",
        "historical_summary": _historical_directory(seed) / "rollout_summary.json",
    }
    paths = {
        name: _verified_record_path(
            artifacts[name], f"replay seed {seed} {name}", expected
        )
        for name, expected in expected_paths.items()
    }
    summary = _strict_mapping(paths["summary"])
    if _require_counter(summary.get("seed"), "replay summary seed") != seed:
        raise H0PrimarySplitV3Error(f"replay seed {seed} summary identity drifted")
    execution_lock = verify_lock()
    selected_so_policy_id = execution_lock["so_policy_id"]
    if summary.get("so_policy_id") != selected_so_policy_id:
        raise H0PrimarySplitV3Error(f"replay seed {seed} SO policy drifted")
    gate = _strict_mapping(paths["gate"])
    recomputed_gate = _replay_gate(summary, so_policy_id=selected_so_policy_id)
    if common_gates.canonical_json_bytes(gate) != common_gates.canonical_json_bytes(
        recomputed_gate
    ):
        raise H0PrimarySplitV3Error(f"replay seed {seed} gate is not reproducible")
    if gate.get("passed") is not True or gate.get("status") != receipt.get("status"):
        raise H0PrimarySplitV3Error(f"replay seed {seed} gate did not pass")
    fallback_journal = _strict_sequence(paths["fallback_journal"])
    fallback_by_step: dict[int, tuple[int, int]] = {}
    fallback_total = 0
    so_fallback_total = 0
    sea_fallback_total = 0
    previous_fallback_step = 0
    for row_index, raw_row in enumerate(fallback_journal, start=1):
        if not isinstance(raw_row, Mapping) or set(raw_row) != {
            "step",
            "time_s",
            "so_fallback",
            "sea_plugin_fallback_count",
        }:
            raise H0PrimarySplitV3Error(
                f"replay seed {seed} fallback row {row_index} schema drifted"
            )
        fallback_step = _require_counter(raw_row["step"], "fallback step")
        if fallback_step <= previous_fallback_step:
            raise H0PrimarySplitV3Error("fallback journal steps are not monotonic")
        previous_fallback_step = fallback_step
        _require_finite_number(raw_row["time_s"], "fallback time")
        so_value = _require_counter(raw_row["so_fallback"], "SO fallback")
        sea_value = _require_counter(
            raw_row["sea_plugin_fallback_count"], "SEA fallback"
        )
        if so_value not in {0, 1} or (so_value == 0 and sea_value == 0):
            raise H0PrimarySplitV3Error("fallback journal row is non-canonical")
        fallback_by_step[fallback_step] = (so_value, sea_value)
        so_fallback_total += so_value
        sea_fallback_total += sea_value
        fallback_total += so_value + sea_value
    if (
        fallback_total != _require_counter(summary.get("fallback_count"), "fallback")
        or so_fallback_total
        != _require_counter(summary.get("so_fallback_count"), "SO fallback total")
        or sea_fallback_total
        != _require_counter(
            summary.get("sea_plugin_fallback_count"), "SEA fallback total"
        )
    ):
        raise H0PrimarySplitV3Error("fallback journal does not reconcile with summary")
    solver_journal = _strict_sequence(paths["solver_audit_journal"])
    steps = _require_counter(summary.get("steps"), "replay summary steps")
    if any(step > steps for step in fallback_by_step):
        raise H0PrimarySplitV3Error("fallback journal step exceeds replay length")
    if len(solver_journal) != steps:
        raise H0PrimarySplitV3Error(f"replay seed {seed} solver journal is incomplete")
    audit_windows = 0
    audit_attempts = 0
    audit_hard_fallbacks = 0
    recovery_totals = {key: 0 for key in SO_RECOVERY_COUNTER_KEYS}
    for index, row in enumerate(solver_journal, start=1):
        if not isinstance(row, Mapping) or set(row) != {
            "step",
            "time_s",
            "control_windows",
        }:
            raise H0PrimarySplitV3Error(
                f"replay seed {seed} solver journal row {index} drifted"
            )
        if _require_counter(row["step"], "solver journal step") != index:
            raise H0PrimarySplitV3Error("solver journal steps are not contiguous")
        _require_finite_number(row["time_s"], "solver journal time")
        windows, attempts, hard = _validate_so_solver_audit_entries(
            row["control_windows"],
            step_index=index,
            selected_fallback=bool(fallback_by_step.get(index, (0, 0))[0]),
        )
        audit_windows += len(windows)
        audit_attempts += attempts
        audit_hard_fallbacks += hard
        recovery = so_recovery.classify_policy_step(
            row["control_windows"], policy_id=selected_so_policy_id
        )
        for key in SO_RECOVERY_COUNTER_KEYS:
            recovery_totals[key] += _require_counter(
                recovery["counters"].get(key),
                f"replay seed {seed} recovery counter {key}",
            )
    if (
        _require_counter(
            summary.get("so_solver_control_window_count"),
            "summary SO control-window count",
        )
        != audit_windows
        or _require_counter(
            summary.get("so_solver_attempt_count"), "summary SO attempt count"
        )
        != audit_attempts
        or _require_counter(
            summary.get("so_solver_hard_fallback_count"),
            "summary SO hard-fallback count",
        )
        != audit_hard_fallbacks
    ):
        raise H0PrimarySplitV3Error(
            f"replay seed {seed} solver audit aggregate drifted"
        )
    summary_recovery_fields = {
        "primary_solver_nonconvergence_count": (
            "so_solver_primary_nonconvergence_count"
        ),
        "bounded_ls_invocation_count": "so_solver_bounded_ls_invocation_count",
        "selected_bounded_ls_count": "so_solver_selected_bounded_ls_count",
        "verified_bounded_ls_count": "so_solver_verified_bounded_ls_count",
        "verified_bounded_ls_success_count": (
            "so_solver_verified_bounded_ls_success_count"
        ),
        "verified_status0_max_iter_count": (
            "so_solver_verified_status0_max_iter_count"
        ),
        "unaccepted_hard_so_fallback_count": (
            "so_solver_unaccepted_hard_fallback_count"
        ),
        "unaccepted_bounded_ls_count": (
            "so_solver_unaccepted_bounded_ls_count"
        ),
        "reuse_previous_count": "so_solver_reuse_previous_count",
        "bounded_ls_unsuccessful_count": ("so_solver_bounded_ls_unsuccessful_count"),
        "bounds_violation_count": "so_solver_bounds_violation_count",
        "nonfinite_solver_count": "so_solver_nonfinite_count",
        "selected_infeasible_count": "so_solver_selected_infeasible_count",
        "selected_solution_mismatch_count": (
            "so_solver_selected_solution_mismatch_count"
        ),
        "residual_contract_mismatch_count": (
            "so_solver_residual_contract_mismatch_count"
        ),
    }
    for counter_key, summary_key in summary_recovery_fields.items():
        if (
            _require_counter(summary.get(summary_key), summary_key)
            != (recovery_totals[counter_key])
        ):
            raise H0PrimarySplitV3Error(
                f"replay seed {seed} recovery aggregate drifted at {summary_key}"
            )
    return receipt, paths


def finalize_corpus(output_dir: str | Path) -> dict[str, Any]:
    verify_lock()
    destination = _claim_destination(
        output_dir,
        diagnostic=False,
        canonical_destination=RUN_ROOT / "corpus",
    )
    import numpy as np

    groups: list[dict[str, Any]] = []
    input_records: dict[str, Any] = {}
    canonical_names: tuple[str, ...] | None = None
    for seed in TRAIN_SEEDS:
        replay_dir = _canonical_replay_destination(seed)
        _, replay_paths = _validate_replay_receipt(seed)
        arrays_path = replay_paths["arrays"]
        with np.load(arrays_path, allow_pickle=False) as archive:
            expected_keys = {
                "target_observations",
                "reconstructed_teacher_observations",
                "historical_observations",
                "teacher_means",
                "teacher_logstd",
                "replay_actions",
                "times",
                "rewards",
                "actor_feature_names",
            }
            if set(archive.files) != expected_keys:
                raise H0PrimarySplitV3Error(f"replay seed {seed} NPZ schema drifted")
            students = _require_array_contract(
                archive["target_observations"],
                np=np,
                label=f"replay seed {seed} target observations",
                dtype=np.float32,
                shape=(EXPECTED_STEPS, EXPECTED_ACTOR_FEATURES),
            )
            teachers = _require_array_contract(
                archive["reconstructed_teacher_observations"],
                np=np,
                label=f"replay seed {seed} teacher observations",
                dtype=np.float32,
                shape=(EXPECTED_STEPS, EXPECTED_ACTOR_FEATURES),
            )
            means = _require_array_contract(
                archive["teacher_means"],
                np=np,
                label=f"replay seed {seed} teacher means",
                dtype=np.float32,
                shape=(EXPECTED_STEPS, 2),
            )
            _require_array_contract(
                archive["historical_observations"],
                np=np,
                label=f"replay seed {seed} historical observations",
                dtype=np.float32,
                shape=(EXPECTED_STEPS, EXPECTED_ACTOR_FEATURES),
            )
            _require_array_contract(
                archive["teacher_logstd"],
                np=np,
                label=f"replay seed {seed} teacher logstd",
                dtype=np.float32,
                shape=(EXPECTED_STEPS, 2),
            )
            _require_array_contract(
                archive["replay_actions"],
                np=np,
                label=f"replay seed {seed} actions",
                dtype=np.float32,
                shape=(EXPECTED_STEPS, 2),
            )
            _require_array_contract(
                archive["times"],
                np=np,
                label=f"replay seed {seed} times",
                dtype=np.float64,
                shape=(EXPECTED_STEPS,),
            )
            _require_array_contract(
                archive["rewards"],
                np=np,
                label=f"replay seed {seed} rewards",
                dtype=np.float64,
                shape=(EXPECTED_STEPS,),
            )
            names = _require_array_contract(
                archive["actor_feature_names"],
                np=np,
                label=f"replay seed {seed} actor feature names",
                dtype=np.dtype("U64"),
                shape=(EXPECTED_ACTOR_FEATURES,),
                finite=False,
            )
        names_tuple = tuple(names.tolist())
        historical_summary = _strict_mapping(replay_paths["historical_summary"])
        if names_tuple != tuple(historical_summary.get("actor_feature_names", [])):
            raise H0PrimarySplitV3Error(
                f"replay seed {seed} actor names differ from history"
            )
        if canonical_names is None:
            canonical_names = names_tuple
        elif names_tuple != canonical_names:
            raise H0PrimarySplitV3Error("replay actor layouts differ across seeds")
        groups.append(
            {
                "trial_id": str(seed),
                "student_views": students,
                "teacher_views": teachers,
                "teacher_means": means,
                "actor_feature_names": list(names_tuple),
            }
        )
        input_records[str(seed)] = {
            "receipt": source_record(replay_dir / "receipt.json"),
            **{name: source_record(path) for name, path in replay_paths.items()},
        }

    dataset, training_indices, validation_indices, assembly = (
        split_contract.assemble_train_only_dataset(
            groups,
            training_trials=tuple(str(seed) for seed in TRAIN_SEEDS),
        )
    )
    if (
        assembly.get("physical_states") != 1000
        or assembly.get("records") != 2000
        or assembly.get("training_records") != 2000
        or assembly.get("validation_records") != 0
        or assembly.get("partition_mode") != "fixed_final_epoch_train_only"
    ):
        raise H0PrimarySplitV3Error(f"V3 corpus cardinality drifted: {assembly}")
    corpus_path = _write_npz_exclusive(
        destination / "corpus.npz",
        **dataset,
        training_indices=np.asarray(training_indices, dtype=np.int64),
        validation_indices=np.asarray(validation_indices, dtype=np.int64),
    )
    manifest = {
        "schema_version": 3,
        "status": "H0_PRIMARY_SPLIT_V3_CORPUS_FROZEN",
        **assembly,
        "actor_feature_names": dataset["actor_feature_names"].astype(str).tolist(),
        "mutable_feature_names": list(MUTABLE_FEATURE_NAMES),
        "execution_lock": source_record(LOCK),
        "attempt_claim": source_record(ATTEMPT_CLAIM),
        "source_replays": input_records,
        "corpus": source_record(corpus_path),
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
    }
    manifest_path = _write_json_exclusive(destination / "manifest.json", manifest)
    receipt = {
        "schema_version": 3,
        "status": "PASS_H0_PRIMARY_SPLIT_V3_CORPUS_FROZEN",
        "passed": True,
        "corpus": source_record(corpus_path),
        "manifest": source_record(manifest_path),
        "execution_lock": source_record(LOCK),
        "attempt_claim": source_record(ATTEMPT_CLAIM),
        "source_replays": input_records,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
    }
    _write_json_exclusive(destination / "receipt.json", receipt)
    return receipt


def _validate_corpus_receipt() -> tuple[dict[str, Any], dict[str, Any], Path]:
    corpus_dir = RUN_ROOT / "corpus"
    receipt_path = corpus_dir / "receipt.json"
    receipt = _strict_mapping(receipt_path)
    if set(receipt) != {
        "schema_version",
        "status",
        "passed",
        "corpus",
        "manifest",
        "execution_lock",
        "attempt_claim",
        "source_replays",
        "actor_updates",
        "critic_updates",
        "ppo_updates",
        "protected_trials_opened",
    }:
        raise H0PrimarySplitV3Error("V3 corpus receipt schema drifted")
    if (
        _require_counter(receipt["schema_version"], "corpus schema_version") != 3
        or receipt.get("status") != "PASS_H0_PRIMARY_SPLIT_V3_CORPUS_FROZEN"
        or _require_bool(receipt.get("passed"), "corpus passed") is not True
        or _require_counter(receipt.get("actor_updates"), "corpus actor_updates") != 0
        or _require_counter(receipt.get("critic_updates"), "corpus critic_updates") != 0
        or _require_counter(receipt.get("ppo_updates"), "corpus ppo_updates") != 0
        or receipt.get("protected_trials_opened") != []
    ):
        raise H0PrimarySplitV3Error("V3 corpus receipt is non-canonical")
    corpus_path = _verified_record_path(
        receipt.get("corpus"), "V3 corpus", corpus_dir / "corpus.npz"
    )
    manifest_path = _verified_record_path(
        receipt.get("manifest"), "V3 corpus manifest", corpus_dir / "manifest.json"
    )
    _verified_record_path(receipt.get("execution_lock"), "V3 corpus lock", LOCK)
    _verified_record_path(
        receipt.get("attempt_claim"), "V3 corpus attempt claim", ATTEMPT_CLAIM
    )
    manifest = _strict_mapping(manifest_path)
    expected_manifest_keys = {
        "schema_version",
        "status",
        "physical_states",
        "records",
        "training_records",
        "validation_records",
        "training_trials",
        "validation_trials",
        "training_group_count",
        "validation_group_count",
        "partition_mode",
        "observations_sha256",
        "actions_sha256",
        "training_indices_sha256",
        "validation_indices_sha256",
        "actor_feature_names",
        "mutable_feature_names",
        "execution_lock",
        "attempt_claim",
        "source_replays",
        "corpus",
        "actor_updates",
        "critic_updates",
        "ppo_updates",
        "protected_trials_opened",
    }
    if set(manifest) != expected_manifest_keys:
        raise H0PrimarySplitV3Error("V3 corpus manifest schema drifted")
    if (
        _require_counter(manifest["schema_version"], "manifest schema_version") != 3
        or manifest.get("status") != "H0_PRIMARY_SPLIT_V3_CORPUS_FROZEN"
        or manifest.get("mutable_feature_names") != list(MUTABLE_FEATURE_NAMES)
        or manifest.get("partition_mode") != "fixed_final_epoch_train_only"
        or _require_counter(manifest.get("physical_states"), "physical_states") != 1000
        or _require_counter(manifest.get("records"), "records") != 2000
        or _require_counter(manifest.get("training_records"), "training_records")
        != 2000
        or _require_counter(manifest.get("validation_records"), "validation_records")
        != 0
        or manifest.get("training_trials") != [str(seed) for seed in TRAIN_SEEDS]
        or manifest.get("validation_trials") != []
        or _require_counter(manifest.get("actor_updates"), "manifest actor_updates")
        != 0
        or _require_counter(manifest.get("critic_updates"), "manifest critic_updates")
        != 0
        or _require_counter(manifest.get("ppo_updates"), "manifest ppo_updates") != 0
        or manifest.get("protected_trials_opened") != []
    ):
        raise H0PrimarySplitV3Error("V3 corpus manifest is non-canonical")
    for container, label in ((receipt, "receipt"), (manifest, "manifest")):
        _verified_record_path(container["execution_lock"], f"{label} lock", LOCK)
        _verified_record_path(
            container["attempt_claim"], f"{label} attempt claim", ATTEMPT_CLAIM
        )
    if common_gates.canonical_json_bytes(receipt.get("corpus")) != (
        common_gates.canonical_json_bytes(manifest.get("corpus"))
    ):
        raise H0PrimarySplitV3Error("V3 corpus record differs across provenance")
    expected_replays: dict[str, Any] = {}
    for seed in TRAIN_SEEDS:
        replay_dir = _canonical_replay_destination(seed)
        _, replay_paths = _validate_replay_receipt(seed)
        expected_replays[str(seed)] = {
            "receipt": source_record(replay_dir / "receipt.json"),
            **{name: source_record(path) for name, path in replay_paths.items()},
        }
    for container, label in ((receipt, "receipt"), (manifest, "manifest")):
        if common_gates.canonical_json_bytes(container.get("source_replays")) != (
            common_gates.canonical_json_bytes(expected_replays)
        ):
            raise H0PrimarySplitV3Error(f"V3 corpus {label} replay provenance drifted")
    return receipt, manifest, corpus_path


def _module_logits(module_path: Path, observations: Any):
    import numpy as np
    import torch
    from ray.rllib.core.rl_module.rl_module import RLModule

    module = RLModule.from_checkpoint(module_path.resolve())
    module.pi.eval()
    with torch.no_grad():
        logits = module.pi(torch.as_tensor(observations, dtype=torch.float32))
    return logits.detach().cpu().numpy().astype(np.float32)


def adapt_worker(output_dir: str | Path) -> dict[str, Any]:
    verify_lock()
    destination = _claim_destination(
        output_dir,
        diagnostic=False,
        canonical_destination=RUN_ROOT / "adaptation",
    )
    import numpy as np

    _corpus_receipt, corpus_manifest, corpus_path = _validate_corpus_receipt()
    with np.load(corpus_path, allow_pickle=False) as archive:
        expected_keys = {
            "observations",
            "actions",
            "actor_feature_names",
            "trial_ids",
            "view_roles",
            "group_ids",
            "training_indices",
            "validation_indices",
        }
        if set(archive.files) != expected_keys:
            raise H0PrimarySplitV3Error("V3 corpus NPZ schema drifted")
        observations = _require_array_contract(
            archive["observations"],
            np=np,
            label="V3 corpus observations",
            dtype=np.float32,
            shape=(2000, EXPECTED_ACTOR_FEATURES),
        )
        targets = _require_array_contract(
            archive["actions"],
            np=np,
            label="V3 corpus actions",
            dtype=np.float32,
            shape=(2000, 2),
        )
        names = _require_array_contract(
            archive["actor_feature_names"],
            np=np,
            label="V3 corpus actor names",
            dtype=np.dtype("U64"),
            shape=(EXPECTED_ACTOR_FEATURES,),
            finite=False,
        )
        trial_ids = _require_array_contract(
            archive["trial_ids"],
            np=np,
            label="V3 corpus trial IDs",
            dtype=np.dtype("U8"),
            shape=(2000,),
            finite=False,
        )
        roles = _require_array_contract(
            archive["view_roles"],
            np=np,
            label="V3 corpus view roles",
            dtype=np.dtype("U16"),
            shape=(2000,),
            finite=False,
        )
        group_ids = _require_array_contract(
            archive["group_ids"],
            np=np,
            label="V3 corpus group IDs",
            dtype=np.dtype("U32"),
            shape=(2000,),
            finite=False,
        )
        training_indices = _require_array_contract(
            archive["training_indices"],
            np=np,
            label="V3 training indices",
            dtype=np.int64,
            shape=(2000,),
        )
        validation_indices = _require_array_contract(
            archive["validation_indices"],
            np=np,
            label="V3 validation indices",
            dtype=np.int64,
            shape=(0,),
        )
    if (
        tuple(names.tolist()) != tuple(corpus_manifest["actor_feature_names"])
        or set(roles.tolist()) != {"student_view", "teacher_view"}
        or set(trial_ids[training_indices].tolist()) != {"123", "124"}
        or validation_indices.size != 0
        or not np.array_equal(training_indices, np.arange(2000, dtype=np.int64))
        or np.intersect1d(training_indices, validation_indices).size
        or np.any(training_indices < 0)
        or np.any(validation_indices < 0)
        or np.any(training_indices >= len(observations))
        or np.any(validation_indices >= len(observations))
        or len(np.unique(training_indices)) != len(training_indices)
        or len(np.unique(validation_indices)) != len(validation_indices)
        or len(set(group_ids[training_indices].tolist())) != 1000
    ):
        raise H0PrimarySplitV3Error("V3 corpus split/provenance drifted")
    expected_hashes = {
        "observations_sha256": split_contract.array_sha256(observations),
        "actions_sha256": split_contract.array_sha256(targets),
        "training_indices_sha256": split_contract.array_sha256(training_indices),
        "validation_indices_sha256": split_contract.array_sha256(validation_indices),
    }
    if any(corpus_manifest.get(key) != value for key, value in expected_hashes.items()):
        raise H0PrimarySplitV3Error("V3 corpus array digests drifted")
    dataset = {
        "observations": observations,
        "actions": targets,
        "actor_feature_names": names,
    }
    report = imitation.adapt_actor(
        H0_MODULE.resolve(),
        dataset,
        destination,
        seed=FIT["seed"],
        epochs=FIT["epochs"],
        batch_size=FIT["batch_size"],
        learning_rate=FIT["learning_rate"],
        validation_fraction=FIT["validation_fraction"],
        patience=FIT["patience"],
        clip_weight=FIT["clip_weight"],
        logstd_weight=FIT["logstd_weight"],
        anchor_weight=FIT["anchor_weight"],
        freeze_logstd_head=True,
        trainable_first_layer_features=MUTABLE_FEATURE_NAMES,
        selection_mode=FIT["selection_mode"],
    )
    candidate = destination / "rl_module_target_adapted"
    source_logits = _module_logits(H0_MODULE, observations)
    adapted_logits = _module_logits(candidate, observations)
    train_fit_metrics = split_contract.offline_adaptation_gate(
        source_predictions=source_logits[:, :2],
        adapted_predictions=adapted_logits[:, :2],
        targets=targets,
        validation_indices=training_indices,
        view_roles=roles,
        all_adapted_logits=adapted_logits,
    )
    source_state = warm_start.load_module_state(H0_MODULE)
    candidate_state = warm_start.load_module_state(candidate)
    scope = markov_adaptation.selected_column_update_audit(
        source_state,
        candidate_state,
        names.astype(str).tolist(),
        MUTABLE_FEATURE_NAMES,
    )
    logstd_parameter_exact = all(
        np.array_equal(
            warm_start._as_numpy(source_state[key])[2:],
            warm_start._as_numpy(candidate_state[key])[2:],
        )
        for key in ("pi.1.weight", "pi.1.bias")
    )
    checks = {
        **train_fit_metrics["checks"],
        "fixed_final_epoch_mode": report.get("selection_mode") == "fixed_final_epoch",
        "epochs_exact_400": report.get("epochs_run") == FIT["epochs"]
        and report.get("best_epoch") == FIT["epochs"],
        "zero_validation_samples_in_fit": report.get("validation_samples") == 0,
        "only_selected_columns_changed": bool(scope["only_selected_columns_changed"]),
        "logstd_parameters_bit_exact": bool(logstd_parameter_exact),
        "logstd_outputs_bit_exact": bool(
            np.array_equal(source_logits[:, 2:], adapted_logits[:, 2:])
        ),
        "actor_digest_changed": warm_start.actor_state_digest(source_state)
        != warm_start.actor_state_digest(candidate_state),
        "save_reload_actor_exact": bool(report["save_reload"]["exact"]),
        "non_actor_exact_or_absent": report["non_actor_verification"]
        in {"exact", "not_available_in_inference_only_rl_module"},
        "actor_manifest_width_35": len(names) == EXPECTED_ACTOR_FEATURES,
    }
    passed = all(checks.values())
    manifest = {
        "schema_version": 3,
        "candidate_id": CANDIDATE_ID,
        "observation_contract_id": "primary_grf_split_v1",
        "event_contract_id": "legacy_events_v1",
        "actor_feature_count": len(names),
        "actor_feature_names": names.astype(str).tolist(),
        "mutable_feature_names": list(MUTABLE_FEATURE_NAMES),
        "actor_digest": warm_start.actor_state_digest(candidate_state),
        "module_state_sha256": sha256_file(candidate / "module_state.pkl"),
        "execution_lock": source_record(LOCK),
        "attempt_claim": source_record(ATTEMPT_CLAIM),
        "corpus_receipt": source_record(RUN_ROOT / "corpus" / "receipt.json"),
        "corpus_manifest": source_record(RUN_ROOT / "corpus" / "manifest.json"),
        "corpus": source_record(corpus_path),
        "source_h0": {
            "state": source_record(H0_MODULE / "module_state.pkl"),
            "ctor": source_record(H0_MODULE / "class_and_ctor_args.pkl"),
            "metadata": source_record(H0_MODULE / "metadata.json"),
            "config": source_record(H0_CONFIG),
        },
    }
    manifest_path = _write_json_exclusive(
        candidate / warm_start.DEFAULT_ACTOR_FEATURE_MANIFEST_NAME,
        manifest,
    )
    gate = {
        "schema_version": 3,
        "status": "PASS_H0_PRIMARY_SPLIT_V3_OFFLINE"
        if passed
        else "FAIL_H0_PRIMARY_SPLIT_V3_OFFLINE",
        "passed": passed,
        "checks": checks,
        "train_fit_metrics": train_fit_metrics,
        "selected_column_update_audit": scope,
        "fit": FIT,
        "execution_lock": source_record(LOCK),
        "attempt_claim": source_record(ATTEMPT_CLAIM),
        "corpus_receipt": source_record(RUN_ROOT / "corpus" / "receipt.json"),
        "corpus_manifest": source_record(RUN_ROOT / "corpus" / "manifest.json"),
        "corpus": source_record(corpus_path),
        "source_actor_digest": warm_start.actor_state_digest(source_state),
        "candidate_actor_digest": warm_start.actor_state_digest(candidate_state),
        "actor_updates": 1,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
    }
    gate_path = _write_json_exclusive(destination / "offline_gate.json", gate)
    receipt = {
        "schema_version": 3,
        "status": gate["status"],
        "passed": passed,
        "candidate_module_state": source_record(candidate / "module_state.pkl"),
        "candidate_module_ctor": source_record(candidate / "class_and_ctor_args.pkl"),
        "candidate_module_metadata": source_record(candidate / "metadata.json"),
        "actor_feature_manifest": source_record(manifest_path),
        "adaptation_report": source_record(destination / "adaptation_report.json"),
        "offline_gate": source_record(gate_path),
        "corpus": source_record(corpus_path),
        "corpus_receipt": source_record(RUN_ROOT / "corpus" / "receipt.json"),
        "corpus_manifest": source_record(RUN_ROOT / "corpus" / "manifest.json"),
        "execution_lock": source_record(LOCK),
        "attempt_claim": source_record(ATTEMPT_CLAIM),
        "source_h0": {
            "state": source_record(H0_MODULE / "module_state.pkl"),
            "ctor": source_record(H0_MODULE / "class_and_ctor_args.pkl"),
            "metadata": source_record(H0_MODULE / "metadata.json"),
            "config": source_record(H0_CONFIG),
        },
        "actor_updates": 1,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
    }
    _write_json_exclusive(destination / "receipt.json", receipt)
    if not passed:
        raise H0PrimarySplitV3Error("V3 offline adaptation gate failed")
    return receipt


def _validate_candidate_receipt() -> tuple[dict[str, Any], Path]:
    destination = RUN_ROOT / "adaptation"
    _corpus_receipt, corpus_manifest, _corpus_path = _validate_corpus_receipt()
    receipt = _strict_mapping(destination / "receipt.json")
    expected_keys = {
        "schema_version",
        "status",
        "passed",
        "candidate_module_state",
        "candidate_module_ctor",
        "candidate_module_metadata",
        "actor_feature_manifest",
        "adaptation_report",
        "offline_gate",
        "corpus",
        "corpus_receipt",
        "corpus_manifest",
        "execution_lock",
        "attempt_claim",
        "source_h0",
        "actor_updates",
        "critic_updates",
        "ppo_updates",
        "protected_trials_opened",
    }
    if set(receipt) != expected_keys:
        raise H0PrimarySplitV3Error("V3 candidate receipt schema drifted")
    if (
        _require_counter(receipt.get("schema_version"), "candidate schema") != 3
        or receipt.get("status") != "PASS_H0_PRIMARY_SPLIT_V3_OFFLINE"
        or _require_bool(receipt.get("passed"), "candidate passed") is not True
        or _require_counter(receipt.get("actor_updates"), "candidate actor updates")
        != 1
        or _require_counter(receipt.get("critic_updates"), "candidate critic updates")
        != 0
        or _require_counter(receipt.get("ppo_updates"), "candidate PPO updates") != 0
        or receipt.get("protected_trials_opened") != []
    ):
        raise H0PrimarySplitV3Error("V3 candidate receipt is non-canonical")
    candidate = destination / "rl_module_target_adapted"
    expected_records = {
        "candidate_module_state": candidate / "module_state.pkl",
        "candidate_module_ctor": candidate / "class_and_ctor_args.pkl",
        "candidate_module_metadata": candidate / "metadata.json",
        "actor_feature_manifest": (
            candidate / warm_start.DEFAULT_ACTOR_FEATURE_MANIFEST_NAME
        ),
        "adaptation_report": destination / "adaptation_report.json",
        "offline_gate": destination / "offline_gate.json",
        "corpus": RUN_ROOT / "corpus" / "corpus.npz",
        "corpus_receipt": RUN_ROOT / "corpus" / "receipt.json",
        "corpus_manifest": RUN_ROOT / "corpus" / "manifest.json",
        "execution_lock": LOCK,
        "attempt_claim": ATTEMPT_CLAIM,
    }
    for key, expected in expected_records.items():
        _verified_record_path(receipt.get(key), f"candidate {key}", expected)
    source_h0 = receipt.get("source_h0")
    if not isinstance(source_h0, Mapping) or set(source_h0) != {
        "state",
        "ctor",
        "metadata",
        "config",
    }:
        raise H0PrimarySplitV3Error("candidate source H0 provenance drifted")
    for key, expected in {
        "state": H0_MODULE / "module_state.pkl",
        "ctor": H0_MODULE / "class_and_ctor_args.pkl",
        "metadata": H0_MODULE / "metadata.json",
        "config": H0_CONFIG,
    }.items():
        _verified_record_path(source_h0[key], f"candidate source H0 {key}", expected)
    report = _strict_mapping(destination / "adaptation_report.json")
    if (
        report.get("selection_mode") != "fixed_final_epoch"
        or _require_counter(report.get("epochs_run"), "candidate epochs_run")
        != FIT["epochs"]
        or _require_counter(report.get("best_epoch"), "candidate best_epoch")
        != FIT["epochs"]
        or _require_counter(
            report.get("training_samples"), "candidate training samples"
        )
        != 2000
        or _require_counter(
            report.get("validation_samples"), "candidate validation samples"
        )
        != 0
    ):
        raise H0PrimarySplitV3Error("candidate fixed-final-epoch fit drifted")
    gate = _strict_mapping(destination / "offline_gate.json")
    if (
        gate.get("passed") is not True
        or gate.get("status") != receipt.get("status")
        or common_gates.canonical_json_bytes(gate.get("fit"))
        != common_gates.canonical_json_bytes(FIT)
    ):
        raise H0PrimarySplitV3Error("candidate offline gate drifted")
    manifest_path = candidate / warm_start.DEFAULT_ACTOR_FEATURE_MANIFEST_NAME
    manifest = _strict_mapping(manifest_path)
    expected_manifest_keys = {
        "schema_version",
        "candidate_id",
        "observation_contract_id",
        "event_contract_id",
        "actor_feature_count",
        "actor_feature_names",
        "mutable_feature_names",
        "actor_digest",
        "module_state_sha256",
        "execution_lock",
        "attempt_claim",
        "corpus_receipt",
        "corpus_manifest",
        "corpus",
        "source_h0",
    }
    candidate_state = warm_start.load_module_state(candidate)
    candidate_digest = warm_start.actor_state_digest(candidate_state)
    if (
        set(manifest) != expected_manifest_keys
        or _require_counter(manifest.get("schema_version"), "manifest schema") != 3
        or manifest.get("candidate_id") != CANDIDATE_ID
        or manifest.get("observation_contract_id") != "primary_grf_split_v1"
        or manifest.get("event_contract_id") != "legacy_events_v1"
        or _require_counter(
            manifest.get("actor_feature_count"), "manifest actor feature count"
        )
        != EXPECTED_ACTOR_FEATURES
        or manifest.get("actor_feature_names") != corpus_manifest["actor_feature_names"]
        or manifest.get("mutable_feature_names") != list(MUTABLE_FEATURE_NAMES)
        or manifest.get("actor_digest") != candidate_digest
        or manifest.get("module_state_sha256")
        != sha256_file(candidate / "module_state.pkl")
        or gate.get("candidate_actor_digest") != candidate_digest
    ):
        raise H0PrimarySplitV3Error("candidate actor manifest/digest drifted")
    for key, expected in {
        "execution_lock": LOCK,
        "attempt_claim": ATTEMPT_CLAIM,
        "corpus_receipt": RUN_ROOT / "corpus" / "receipt.json",
        "corpus_manifest": RUN_ROOT / "corpus" / "manifest.json",
        "corpus": RUN_ROOT / "corpus" / "corpus.npz",
    }.items():
        _verified_record_path(manifest.get(key), f"candidate manifest {key}", expected)
    if common_gates.canonical_json_bytes(manifest.get("source_h0")) != (
        common_gates.canonical_json_bytes(source_h0)
    ):
        raise H0PrimarySplitV3Error("candidate manifest source H0 drifted")
    return receipt, candidate


def _candidate_freeze_payload() -> dict[str, Any]:
    receipt, candidate = _validate_candidate_receipt()
    gate = _strict_mapping(RUN_ROOT / "adaptation" / "offline_gate.json")
    return {
        "schema_version": 3,
        "status": "H0_PRIMARY_SPLIT_V3_CANDIDATE_FROZEN_BEFORE_HOLDOUT",
        "protocol_id": PROTOCOL_ID,
        "candidate_id": CANDIDATE_ID,
        "train_seeds": list(TRAIN_SEEDS),
        "final_holdout_seed": FINAL_HOLDOUT_SEED,
        "fit": dict(FIT),
        "candidate_receipt": source_record(RUN_ROOT / "adaptation" / "receipt.json"),
        "candidate_offline_gate": source_record(
            RUN_ROOT / "adaptation" / "offline_gate.json"
        ),
        "candidate_module_state": source_record(candidate / "module_state.pkl"),
        "candidate_module_ctor": source_record(candidate / "class_and_ctor_args.pkl"),
        "candidate_module_metadata": source_record(candidate / "metadata.json"),
        "actor_feature_manifest": receipt["actor_feature_manifest"],
        "candidate_actor_digest": gate["candidate_actor_digest"],
        "holdout_accessed_before_freeze": False,
        "actor_updates": 1,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
    }


def freeze_candidate_before_holdout() -> dict[str, Any]:
    if (
        _canonical_replay_destination(FINAL_HOLDOUT_SEED).exists()
        or HOLDOUT_ACCESS_CLAIM.exists()
        or (RUN_ROOT / "holdout").exists()
    ):
        raise H0PrimarySplitV3Error("holdout artifacts exist before candidate freeze")
    payload = _candidate_freeze_payload()
    _write_json_exclusive(CANDIDATE_FREEZE, payload)
    return payload


def verify_candidate_freeze() -> dict[str, Any]:
    observed = _strict_mapping(CANDIDATE_FREEZE)
    expected = _candidate_freeze_payload()
    if common_gates.canonical_json_bytes(observed) != common_gates.canonical_json_bytes(
        expected
    ):
        raise H0PrimarySplitV3Error("V3 candidate freeze drifted")
    return observed


def _holdout_access_payload() -> dict[str, Any]:
    verify_candidate_freeze()
    return {
        "schema_version": 3,
        "status": "H0_PRIMARY_SPLIT_V3_FINAL_HOLDOUT_ACCESS_CLAIMED",
        "protocol_id": PROTOCOL_ID,
        "candidate_freeze": source_record(CANDIDATE_FREEZE),
        "holdout_seed": FINAL_HOLDOUT_SEED,
        "fit_updates_complete_before_access": True,
        "additional_actor_updates_authorized": False,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
    }


def claim_holdout_access() -> dict[str, Any]:
    if _canonical_replay_destination(FINAL_HOLDOUT_SEED).exists():
        raise H0PrimarySplitV3Error("holdout replay exists before access claim")
    payload = _holdout_access_payload()
    _write_json_exclusive(HOLDOUT_ACCESS_CLAIM, payload)
    return payload


def verify_holdout_access_claim() -> dict[str, Any]:
    observed = _strict_mapping(HOLDOUT_ACCESS_CLAIM)
    expected = _holdout_access_payload()
    if common_gates.canonical_json_bytes(observed) != common_gates.canonical_json_bytes(
        expected
    ):
        raise H0PrimarySplitV3Error("V3 holdout access claim drifted")
    return observed


def evaluate_final_holdout(output_dir: str | Path) -> dict[str, Any]:
    verify_lock()
    freeze = verify_candidate_freeze()
    verify_holdout_access_claim()
    destination = _claim_destination(
        output_dir,
        diagnostic=False,
        canonical_destination=RUN_ROOT / "holdout",
    )
    import numpy as np

    _, replay_paths = _validate_replay_receipt(FINAL_HOLDOUT_SEED)
    with np.load(replay_paths["arrays"], allow_pickle=False) as archive:
        students = _require_array_contract(
            archive["target_observations"],
            np=np,
            label="holdout student observations",
            dtype=np.float32,
            shape=(EXPECTED_STEPS, EXPECTED_ACTOR_FEATURES),
        )
        teachers = _require_array_contract(
            archive["reconstructed_teacher_observations"],
            np=np,
            label="holdout teacher observations",
            dtype=np.float32,
            shape=(EXPECTED_STEPS, EXPECTED_ACTOR_FEATURES),
        )
        means = _require_array_contract(
            archive["teacher_means"],
            np=np,
            label="holdout teacher means",
            dtype=np.float32,
            shape=(EXPECTED_STEPS, 2),
        )
        names = _require_array_contract(
            archive["actor_feature_names"],
            np=np,
            label="holdout actor feature names",
            dtype=np.dtype("U64"),
            shape=(EXPECTED_ACTOR_FEATURES,),
            finite=False,
        )
    observations = np.empty(
        (EXPECTED_STEPS * 2, EXPECTED_ACTOR_FEATURES), dtype=np.float32
    )
    targets = np.empty((EXPECTED_STEPS * 2, 2), dtype=np.float32)
    roles = np.empty(EXPECTED_STEPS * 2, dtype="U16")
    observations[0::2] = students
    observations[1::2] = teachers
    targets[0::2] = means
    targets[1::2] = means
    roles[0::2] = split_contract.STUDENT_VIEW
    roles[1::2] = split_contract.TEACHER_VIEW
    indices = np.arange(len(observations), dtype=np.int64)
    _, candidate = _validate_candidate_receipt()
    source_logits = _module_logits(H0_MODULE, observations)
    candidate_logits = _module_logits(candidate, observations)
    metrics = split_contract.offline_adaptation_gate(
        source_predictions=source_logits[:, :2],
        adapted_predictions=candidate_logits[:, :2],
        targets=targets,
        validation_indices=indices,
        view_roles=roles,
        all_adapted_logits=candidate_logits,
    )
    candidate_unchanged = (
        source_record(candidate / "module_state.pkl")
        == freeze["candidate_module_state"]
    )
    checks = {
        **metrics["checks"],
        "holdout_seed_125": FINAL_HOLDOUT_SEED == 125,
        "candidate_frozen_before_holdout": True,
        "candidate_module_unchanged": candidate_unchanged,
        "holdout_records_1000": len(observations) == 1000,
        "actor_layout_35": tuple(names.tolist())
        == tuple(_strict_mapping(replay_paths["summary"])["actor_feature_names"]),
        "no_updates_during_holdout": True,
    }
    passed = all(checks.values())
    gate = {
        "schema_version": 3,
        "status": "PASS_H0_PRIMARY_SPLIT_V3_FINAL_HOLDOUT"
        if passed
        else "FAIL_H0_PRIMARY_SPLIT_V3_FINAL_HOLDOUT",
        "passed": passed,
        "checks": checks,
        "metrics": metrics,
        "candidate_freeze": source_record(CANDIDATE_FREEZE),
        "holdout_access_claim": source_record(HOLDOUT_ACCESS_CLAIM),
        "holdout_replay_receipt": source_record(
            _canonical_replay_destination(FINAL_HOLDOUT_SEED) / "receipt.json"
        ),
        "actor_updates": 1,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
    }
    gate_path = _write_json_exclusive(destination / "gate.json", gate)
    receipt = {
        "schema_version": 3,
        "status": gate["status"],
        "passed": passed,
        "gate": source_record(gate_path),
        "candidate_freeze": source_record(CANDIDATE_FREEZE),
        "holdout_access_claim": source_record(HOLDOUT_ACCESS_CLAIM),
        "holdout_replay_receipt": source_record(
            _canonical_replay_destination(FINAL_HOLDOUT_SEED) / "receipt.json"
        ),
        "actor_updates": 1,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
    }
    _write_json_exclusive(destination / "receipt.json", receipt)
    if not passed:
        raise H0PrimarySplitV3Error("V3 final holdout gate failed")
    return receipt


def _worker_command(*arguments: str) -> list[str]:
    return [sys.executable, str(Path(__file__).resolve()), *arguments]


def _parallel(commands: Sequence[list[str]]) -> None:
    processes = [subprocess.Popen(command, cwd=REPO_ROOT) for command in commands]
    deadline = time.monotonic() + WORKER_TIMEOUT_S
    failure: str | None = None
    try:
        while True:
            codes = [process.poll() for process in processes]
            failed = [code for code in codes if code not in (None, 0)]
            if failed:
                failure = f"worker failures: {failed}"
                break
            if all(code == 0 for code in codes):
                return
            if time.monotonic() >= deadline:
                failure = "parallel worker timeout"
                break
            time.sleep(0.2)
    finally:
        if failure is not None:
            for process in processes:
                if process.poll() is None:
                    process.terminate()
            for process in processes:
                if process.poll() is None:
                    try:
                        process.wait(timeout=5.0)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait(timeout=5.0)
    if failure is not None:
        raise H0PrimarySplitV3Error(failure)


def execute() -> dict[str, Any]:
    verify_lock()
    _claim_attempt_root()
    started = time.time()
    stage = "semantic_replay"
    status = "FAIL_H0_PRIMARY_SPLIT_V3_REPLAY"
    error = None
    passed = False
    try:
        _parallel(
            [
                _worker_command(
                    "--replay-worker",
                    "--seed",
                    str(seed),
                    "--output-dir",
                    str(RUN_ROOT / "replay" / f"seed_{seed}"),
                )
                for seed in TRAIN_SEEDS
            ]
        )
        for seed in TRAIN_SEEDS:
            receipt = _strict_mapping(
                RUN_ROOT / "replay" / f"seed_{seed}" / "receipt.json"
            )
            if receipt.get("passed") is not True:
                raise H0PrimarySplitV3Error(f"replay seed {seed} failed")
        stage = "corpus"
        finalize_corpus(RUN_ROOT / "corpus")
        stage = "adaptation"
        status = "FAIL_H0_PRIMARY_SPLIT_V3_OFFLINE"
        completed = subprocess.run(
            _worker_command(
                "--adapt-worker",
                "--output-dir",
                str(RUN_ROOT / "adaptation"),
            ),
            cwd=REPO_ROOT,
            timeout=WORKER_TIMEOUT_S,
            check=False,
        )
        if completed.returncode != 0:
            raise H0PrimarySplitV3Error(
                f"adaptation worker exited {completed.returncode}"
            )
        _validate_candidate_receipt()
        stage = "candidate_freeze"
        freeze_candidate_before_holdout()
        stage = "final_holdout_replay"
        status = "FAIL_H0_PRIMARY_SPLIT_V3_FINAL_HOLDOUT"
        claim_holdout_access()
        completed = subprocess.run(
            _worker_command(
                "--replay-worker",
                "--seed",
                str(FINAL_HOLDOUT_SEED),
                "--output-dir",
                str(_canonical_replay_destination(FINAL_HOLDOUT_SEED)),
            ),
            cwd=REPO_ROOT,
            timeout=WORKER_TIMEOUT_S,
            check=False,
        )
        if completed.returncode != 0:
            raise H0PrimarySplitV3Error(
                f"final holdout replay worker exited {completed.returncode}"
            )
        stage = "final_holdout_evaluation"
        evaluate_final_holdout(RUN_ROOT / "holdout")
        stage = "final_holdout_complete"
        status = "PASS_H0_PRIMARY_SPLIT_V3_FINAL_HOLDOUT"
        passed = True
    except Exception as exc:
        error = f"{type(exc).__name__}: {exc}"
    ledger = {
        "schema_version": 3,
        "status": status,
        "passed": passed,
        "terminal_stage": stage,
        "error": error,
        "started_unix_s": started,
        "completed_unix_s": time.time(),
        "execution_lock": source_record(LOCK),
        "attempt_claim": source_record(ATTEMPT_CLAIM),
        "corpus_frozen": (RUN_ROOT / "corpus" / "receipt.json").is_file(),
        "candidate_created": (RUN_ROOT / "adaptation" / "receipt.json").is_file(),
        "candidate_frozen_before_holdout": CANDIDATE_FREEZE.is_file(),
        "holdout_access_claimed": HOLDOUT_ACCESS_CLAIM.is_file(),
        "final_holdout_completed": (RUN_ROOT / "holdout" / "receipt.json").is_file(),
        "actor_update_candidates": int(
            (RUN_ROOT / "adaptation" / "adaptation_report.json").is_file()
        ),
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "next_stage": (
            "CANONICAL_CLOSED_LOOP_QUALIFICATION"
            if passed
            else "STOP_WITHOUT_RETRY_OR_RETUNING"
        ),
    }
    _write_json_exclusive(RUN_ROOT / "execution_ledger.json", ledger)
    print(json.dumps(ledger, indent=2, sort_keys=True, allow_nan=False), flush=True)
    if not passed:
        raise H0PrimarySplitV3Error(error or status)
    return ledger


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--diagnostic-replay", action="store_true")
    mode.add_argument("--replay-worker", action="store_true")
    mode.add_argument("--finalize-corpus", action="store_true")
    mode.add_argument("--adapt-worker", action="store_true")
    mode.add_argument("--execute", action="store_true")
    parser.add_argument("--seed", type=int, choices=CANONICAL_SEEDS)
    parser.add_argument("--output-dir")
    parser.add_argument(
        "--max-steps",
        type=int,
        default=EXPECTED_STEPS,
        help="Diagnostic-only shortened replay; protocol workers require 500.",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    if args.execute:
        execute()
        return 0
    if args.finalize_corpus:
        if not args.output_dir:
            raise H0PrimarySplitV3Error("--output-dir is required")
        finalize_corpus(args.output_dir)
        return 0
    if args.adapt_worker:
        if not args.output_dir:
            raise H0PrimarySplitV3Error("--output-dir is required")
        adapt_worker(args.output_dir)
        return 0
    if args.seed is None or not args.output_dir:
        raise H0PrimarySplitV3Error("--seed and --output-dir are required")
    if args.replay_worker and args.max_steps != EXPECTED_STEPS:
        raise H0PrimarySplitV3Error("protocol replay workers require exactly 500 steps")
    run_replay(
        seed=args.seed,
        output_dir=args.output_dir,
        diagnostic=bool(args.diagnostic_replay),
        max_steps=args.max_steps,
    )
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"{type(exc).__name__}: {exc}", file=sys.stderr, flush=True)
        raise SystemExit(2) from exc
