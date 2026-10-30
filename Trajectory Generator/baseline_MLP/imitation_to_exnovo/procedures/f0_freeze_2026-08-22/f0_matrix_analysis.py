"""F0 matrix analyser: per-job and per-start metrics for every rollout family.

Fail-closed and no-clobber. For each described job of the matrix whose output
directory exists, the analyser:
  1. re-verifies the receipt through the driver's ``verify_existing`` (fields,
     status ok, returncode 0, summary.ok, trace present, SHA-256 recomputed);
  2. validates the SUMMARY against an explicit schema (presence, type,
     finiteness, domain of every field the analyser uses);
  3. requires the complete output evidence: six ``.sto`` files with their
     expected columns (plus EVERY column actually consumed, e.g. all
     ``*_reserve_torque`` columns), a time grid consistent with the summary
     step count and finite values; a trace whose every step carries
     raw/applied actions, the actor observation vector, a ``phase_fsm``
     payload and a ``morphology_causal_diagnostics`` mapping. Trace counters
     (FSM events, resync/hs-cancel, causal ledger totals incl.
     ``timeout_transition_count``) must be present where required, finite,
     non-negative integers (bool excluded), with constant
     ``fsm_behaviour_version``/``event_source`` across the trace and coherent
     with the receipt CLI and the pinned config, cumulative counters
     non-decreasing, and the four FSM counters of the summary equal to the
     last trace step. The causal ledger payload is MANDATORY on every step
     when the pinned config selects a causal morphology phase mode; otherwise
     an all-empty ledger is allowed, but once a non-empty payload appears it
     must stay non-empty (continuity) with monotonic counters. Absent is
     never reported as zero. Any violation gives ``FAIL_OUTPUT_INCOMPLETE``
     and excludes the job from the comparisons;
  4. validates every time grid (finite, strictly increasing) and verifies that
     every ``.sto`` shares the kinematics grid BEFORE any cross-series
     computation (tracking of the served reference, SEA power/work);
  5. derives the horizon from the runtime semantics with fail-closed
     provenance (receipt CLI > pinned config > argparse defaults parsed from
     the pinned ``rollout_eval.py``), keeping three quantities apart:
     ``progress_upper_bound`` = ``min(max_steps, ceil(ed/sd))`` (only the
     progress-bar bound of rollout_eval.py:581-587), ``env_horizon_steps_
     expected`` = ``ceil(ed/sd - 0.5)`` (the env ends the episode when
     ``t >= episode_end - 0.5*segment_duration`` with ``t`` advancing by
     ``segment_duration`` per policy step, osim_trj_cmc_like.py
     ``reached_horizon``), and ``expected_recorded_steps`` =
     ``min(loop_cap=max_steps, env_horizon)`` (rollout_eval loops
     ``range(max_steps)``). ``horizon_completed`` is true only when the end
     reason is ``episode_time_limit`` and ``steps == env_horizon_steps_
     expected``; a loop cap below the env horizon is reported as
     ``loop_cap_hit``, never as completion; a ``dataset_end`` end reason means
     the dataset bounds shortened the episode and no equivalence is claimed;
  6. computes the metrics (actor-FSM events with the actual FSM version and
     event source; legacy left-detector events; counters with presence
     flags; morphology settled/discarded; penetration and reserve; actual
     knee/ankle shape and the SERVED REFERENCE from
     ``rollout_episode_kinematics_reference.sto`` (``*_q_ref``); normalized SEA
     controls (``rollout_episode_sea_controls.sto`` holds ``u_sea``); SEA
     torques with sign changes, power and work on the verified common grid)
     and finally rejects any non-finite/overflowed numeric metric through a
     recursive gate (only explicitly nullable fields may be None);
  7. builds per-start comparisons: deltas versus ``B0820_H0`` only for
     ``isometric_comparable`` jobs; historical, 39D and replay rows are
     reported as controls and never declared isometric; stochastic jobs are
     aggregated over development seeds.

Global gate: ``analysis_complete`` is true only when EVERY described job of
the requested perimeter is executed and ``PASS_ANALYSED``;
``executed_subset_complete`` reports the executed subset separately.
Writes ``metrics/f0_matrix_analysis_<stamp>.json`` and ``.md`` (refuses to
overwrite).
"""

from __future__ import annotations

import argparse
import ast
import glob
import json
import math
import sys
import time
from pathlib import Path
from typing import Any, Callable

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import f0_common as C  # noqa: E402
import f0_rollout_matrix as M  # noqa: E402
from f0_replay_analysis import TOL, SAMPLES_PER_STEP, column, read_sto, sto_evidence  # noqa: E402

ISOMETRIC_CLASS = "isometric_comparable"
REFERENCE_CANDIDATE = "B0820_H0"
NON_ISOMETRIC_NOTE = {
    "historical_control": "controllo storico: stesso runtime v3 ma policy di luglio mai addestrata/adattata sotto V26; NON isometrico rispetto alla policy",
    "compatibility_control_39D": "controllo 39D sotto runtime imitativo nativo: return non comparabile (reward imitation); NON isometrico",
    "historical_config_replay_on_current_code": "replay della configurazione storica sul codice HEAD: non isometrico rispetto al runtime target, nessuna riproduzione bit-exact dichiarata",
}

KINEMATICS_FILE = "rollout_episode_kinematics.sto"
REFERENCE_FILE = "rollout_episode_kinematics_reference.sto"
SEA_CONTROLS_FILE = "rollout_episode_sea_controls.sto"
SEA_TORQUES_FILE = "rollout_episode_sea_torques.sto"
GRF_FILE = "rollout_episode_online_grf.sto"
RESERVE_FILE = "rollout_episode_reserve_torques.sto"
EXPECTED_STO: dict[str, tuple[str, ...]] = {
    KINEMATICS_FILE: ("pros_knee_angle", "pros_ankle_angle"),
    REFERENCE_FILE: ("pros_knee_angle_q_ref", "pros_ankle_angle_q_ref"),
    SEA_CONTROLS_FILE: ("pros_knee_angle", "pros_ankle_angle"),
    SEA_TORQUES_FILE: ("SEA_Knee_tau_spring", "SEA_Ankle_tau_spring", "SEA_Knee_tau_motor", "SEA_Ankle_tau_motor"),
    GRF_FILE: ("left_normal_force", "left_penetration", "left_in_contact"),
    RESERVE_FILE: ("pros_knee_angle_reserve_torque", "pros_ankle_angle_reserve_torque"),
}
RESERVE_COLUMN_SUFFIX = "_reserve_torque"
STO_SEMANTICS = {
    KINEMATICS_FILE: "cinematica effettiva delle coordinate protesiche [rad]",
    REFERENCE_FILE: "riferimento servito (q_ref) delle coordinate protesiche [rad] - output.py kinematics_reference",
    SEA_CONTROLS_FILE: "comandi SEA normalizzati u_sea [-1, 1] (output.py:461-462), NON cinematica servita",
    SEA_TORQUES_FILE: "coppie SEA: tau_spring (molla) e tau_motor [Nm]",
    GRF_FILE: "contatto online sinistro: forza normale [N], penetrazione [m], contatto",
    RESERVE_FILE: "coppie reserve per coordinata [Nm]; la norma usa TUTTE le colonne *_reserve_torque",
}
EXPECTED_EVIDENCE: tuple[str, ...] = ("summary", *EXPECTED_STO.keys(), "trace")
assert EXPECTED_STO, "expected series set must be explicit and non-empty"

SUMMARY_SCHEMA: dict[str, str] = {
    "ok": "bool_true",
    "steps": "pos_int",
    "episode_return": "finite_float",
    "reward_mean": "finite_float",
    "end_reason": "nonempty_str",
    "phase_valid_hs_count": "nonneg_int",
    "phase_valid_to_count": "nonneg_int",
    "phase_valid_cycle_count": "nonneg_int",
    "invalid_event_count": "nonneg_int",
    "grf_penetration_max_m": "nonneg_float",
    "reserve_norm_max_nm": "nonneg_float",
    "action_abs_max": "nonneg_float",
    "action_clipped_steps": "nonneg_int",
    "morphology_settled_segments": "nonneg_int",
    "morphology_settled_samples": "nonneg_int",
    "morphology_discarded_segments": "nonneg_int",
    "morphology_discarded_samples": "nonneg_int",
    "grf_penetration_penalty_threshold_m": "pos_float",
    "grf_penetration_termination_m": "pos_float",
    "binary_phase_fsm_mode": "nonempty_str",
    "phase_fsm_input_mode": "nonempty_str",
    "event_contract_id": "nonempty_str",
    "binary_phase_event_contract_id": "nonempty_str",
    "n_actor": "pos_int",
    "n_observation": "pos_int",
}
# ABI of the served policy: action vector exactly 2 (knee, ankle); feature-name lists sized by n_actor/n_observation.
ACTION_SHAPE_EXPECTED = [2]
ACTION_WIDTH = 2
SUMMARY_FEATURE_LISTS = {"actor_feature_names": "n_actor", "observation_feature_names": "n_observation"}
# Event/FSM contract fields that must match the pinned config EXACTLY (no prefix acceptance).
SUMMARY_CONTRACT_FIELDS = ("binary_phase_fsm_mode", "phase_fsm_input_mode", "event_contract_id", "binary_phase_event_contract_id")
BINARY_ACTIVE_MODE = "binary_active"
EVENT_SOURCE_BINARY_ACTIVE = "binary_active_v26"  # binary_phase_adapter_v26.V26_ACTOR_EVENT_SOURCE
TRACE_REQUIRED_KEYS = ("step", "time", "raw_policy_action", "applied_policy_action", "actor_observation_vector_before", "phase_fsm", "morphology_causal_diagnostics")
FSM_REQUIRED_COUNTERS = ("valid_hs_count", "valid_to_count", "valid_cycle_count", "invalid_event_count")
FSM_OPTIONAL_COUNTERS = ("resync_count", "hs_cancelled_count")
FSM_REQUIRED_STRINGS = ("fsm_behaviour_version", "event_source")
FSM_VERSIONS = ("v2", "v3")
CAUSAL_COUNTERS = ("total_cancelled_transition_count", "total_resolved_sample_count", "total_dropped_sample_count", "timeout_transition_count")
SUMMARY_TO_TRACE_COUNTERS = {"phase_valid_hs_count": "valid_hs_count", "phase_valid_to_count": "valid_to_count", "phase_valid_cycle_count": "valid_cycle_count", "invalid_event_count": "invalid_event_count"}
ROLLOUT_EVAL_CLI_FLAGS = {
    "max_steps": "--max-steps",
    "episode_duration": "--episode-duration",
    "segment_duration": "--segment-duration",
    "actor_fsm_version": "--binary-phase-actor-fsm-version",
    "phase_fsm_input_mode": "--phase-fsm-input-mode",
    "event_contract_id": "--event-contract-id",
    "binary_phase_fsm_mode": "--binary-phase-fsm-mode",
    "binary_phase_event_contract_id": "--binary-phase-event-contract-id",
    "include_controller_diagnostic_observation": "--include-controller-diagnostic-observation",
}
CLI_STRING_FLAGS = ("phase_fsm_input_mode", "event_contract_id", "binary_phase_fsm_mode", "binary_phase_event_contract_id")
# argparse BooleanOptionalAction flags: "--flag" -> True, "--no-flag" -> False, no value token
CLI_BOOL_FLAGS = ("include_controller_diagnostic_observation",)
CONFIG_GRF_KEYS = {"phase_fsm_input_mode": "phase_fsm_input_mode", "event_contract_id": "event_contract_id", "binary_phase_fsm_mode": "binary_phase_fsm_mode", "binary_phase_event_contract_id": "binary_phase_event_contract_id"}
HORIZON_END_REASON = "episode_time_limit"
DATASET_END_REASON = "dataset_end"
HORIZON_BOUNDARY_EPS = 1e-9
# Metric leaves allowed to be None (everything else must be a finite number, a string or a bool).
NULLABLE_METRIC_PATHS = (
    "tracking_served_vs_actual.knee.pearson_r",
    "tracking_served_vs_actual.ankle.pearson_r",
    "causal_ledger.failed_closed",
    "causal_ledger.terminal_flushed",
    "events_legacy_left.heel_strike",
    "events_legacy_left.toe_off",
    "horizon.dataset_end_note",
    "fsm_runtime.expected.morphology_phase_mode",
    "fsm_runtime.expected.morphology_causal_event_contract_id",
    "causal_ledger.event_contract_id",
)

ANGLE_DEADBAND_RAD = 1e-3
ANGLE_TURNING_HYSTERESIS_RAD = 0.02
TORQUE_DEADBAND_NM = 2.0
JOINTS = (
    {"joint": "knee", "q": "pros_knee_angle", "q_ref": "pros_knee_angle_q_ref", "u": "pros_knee_angle", "tau_spring": "SEA_Knee_tau_spring", "tau_motor": "SEA_Knee_tau_motor"},
    {"joint": "ankle", "q": "pros_ankle_angle", "q_ref": "pros_ankle_angle_q_ref", "u": "pros_ankle_angle", "tau_spring": "SEA_Ankle_tau_spring", "tau_motor": "SEA_Ankle_tau_motor"},
)


class GridMismatchError(RuntimeError):
    """Raised when a time grid is invalid or two series do not share a grid."""


class ProvenanceError(RuntimeError):
    """Raised when receipt/config/CLI provenance is missing, malformed or inconsistent."""


class MetricsFiniteError(RuntimeError):
    """Raised when a computed metric is non-finite (NaN/Inf/overflow) or unexpectedly None."""


class ComparisonError(RuntimeError):
    """Raised when a stochastic series would mix heterogeneous jobs (family/runtime/class) or duplicate seeds."""


# --- scalar validators ------------------------------------------------------------


def is_finite_float(x: Any) -> bool:
    """Finite number (bool excluded). Python ints are exact and always finite: never cast to float."""
    if isinstance(x, bool):
        return False
    if isinstance(x, int):
        return True
    if isinstance(x, float):
        return math.isfinite(x)
    return False


def is_nonneg_int(x: Any) -> bool:
    """Non-negative integer value: an int (exact, any magnitude) or an integral finite float."""
    if isinstance(x, bool):
        return False
    if isinstance(x, int):
        return x >= 0
    if isinstance(x, float):
        return math.isfinite(x) and x >= 0.0 and x.is_integer()
    return False


def as_int(x: Any) -> int:
    """Exact integer of a counter value (int kept as is; integral float converted); used for
    every difference, sum, max and comparison of counters - no float arithmetic."""
    if isinstance(x, bool) or not isinstance(x, (int, float)):
        raise TypeError(f"not an integer counter value: {x!r}")
    if isinstance(x, int):
        return x
    if not (math.isfinite(x) and x.is_integer()):
        raise TypeError(f"not an integral finite value: {x!r}")
    return int(x)


def check_value(value: Any, kind: str) -> str | None:
    """Return a problem description or None when ``value`` satisfies ``kind``."""
    if kind == "bool_true":
        return None if value is True else f"expected True, got {value!r}"
    if kind == "nonempty_str":
        return None if isinstance(value, str) and value else f"expected non-empty string, got {value!r}"
    if kind == "finite_float":
        return None if is_finite_float(value) else f"expected finite number, got {value!r}"
    if kind == "nonneg_float":
        return None if is_finite_float(value) and value >= 0 else f"expected finite number >= 0, got {value!r}"
    if kind == "pos_float":
        return None if is_finite_float(value) and value > 0 else f"expected finite number > 0, got {value!r}"
    if kind == "nonneg_int":
        return None if is_nonneg_int(value) else f"expected non-negative integer, got {value!r}"
    if kind == "pos_int":
        return None if is_nonneg_int(value) and as_int(value) > 0 else f"expected positive integer, got {value!r}"
    raise ValueError(f"unknown kind {kind}")


def counter(payload: dict[str, Any] | None, key: str) -> dict[str, Any]:
    """Report a counter with an explicit presence flag: absent is not zero; integral values stay exact ints."""
    if not isinstance(payload, dict) or key not in payload or payload.get(key) is None:
        return {"present": False, "value": None}
    value = payload[key]
    if is_nonneg_int(value):
        return {"present": True, "value": as_int(value)}
    return {"present": True, "value": float(value) if isinstance(value, (int, float)) and not isinstance(value, bool) else value}


def assert_finite_metrics(node: Any, path: str = "", nullable: tuple[str, ...] = NULLABLE_METRIC_PATHS) -> None:
    """Recursive gate: every numeric leaf must be finite; None only at nullable
    paths or as the ``value`` of a counter whose ``present`` flag is False."""
    if isinstance(node, dict):
        if set(node.keys()) == {"present", "value"}:
            if node["present"] is False:
                if node["value"] is not None:
                    raise MetricsFiniteError(f"{path}: absent counter carries a value")
                return
            if not is_finite_float(node["value"]):
                raise MetricsFiniteError(f"{path}: counter value not finite: {node['value']!r}")
            return
        for key, value in node.items():
            assert_finite_metrics(value, f"{path}.{key}" if path else str(key), nullable)
        return
    if isinstance(node, (list, tuple)):
        for i, value in enumerate(node):
            assert_finite_metrics(value, f"{path}[{i}]", nullable)
        return
    if node is None:
        if path in nullable:
            return
        raise MetricsFiniteError(f"{path}: unexpected None")
    if isinstance(node, bool) or isinstance(node, str):
        return
    if isinstance(node, (int, np.integer)):
        return  # exact integers are always finite (never cast to float: magnitude above 2**53 must stay exact)
    if isinstance(node, (float, np.floating)):
        if not math.isfinite(float(node)):
            raise MetricsFiniteError(f"{path}: non-finite metric {node!r}")
        return
    raise MetricsFiniteError(f"{path}: unsupported metric type {type(node).__name__}")


# --- grids ---------------------------------------------------------------------------


def validate_grid(t: np.ndarray, label: str) -> None:
    """A time grid must be finite, have at least two samples and be strictly increasing."""
    t = np.asarray(t, dtype=np.float64)
    if t.ndim != 1 or t.size < 2:
        raise GridMismatchError(f"time grid too short ({label}): {t.size} samples")
    if not np.all(np.isfinite(t)):
        raise GridMismatchError(f"time grid contains non-finite values ({label})")
    if not np.all(np.diff(t) > 0.0):
        idx = int(np.argmax(np.diff(t) <= 0.0))
        raise GridMismatchError(f"time grid not strictly increasing ({label}) at index {idx + 1}: {t[idx]!r} -> {t[idx + 1]!r}")


def verify_grid(t_a: np.ndarray, t_b: np.ndarray, label: str) -> None:
    """Fail closed unless both grids are valid, have the same length and identical times."""
    validate_grid(t_a, f"{label} [a]")
    validate_grid(t_b, f"{label} [b]")
    t_a = np.asarray(t_a, dtype=np.float64)
    t_b = np.asarray(t_b, dtype=np.float64)
    if t_a.shape != t_b.shape:
        raise GridMismatchError(f"time grid length mismatch ({label}): {t_a.shape[0]} vs {t_b.shape[0]}")
    if not np.allclose(t_a, t_b, rtol=0.0, atol=TOL):
        idx = int(np.argmax(np.abs(t_a - t_b) > TOL))
        raise GridMismatchError(f"time grid mismatch ({label}) at index {idx}: {t_a[idx]!r} vs {t_b[idx]!r}")


# --- shape metrics --------------------------------------------------------------


def count_sign_changes(x: np.ndarray, deadband: float) -> int:
    sign = 0
    changes = 0
    for v in x:
        s = 1 if v > deadband else (-1 if v < -deadband else 0)
        if s == 0:
            continue
        if sign != 0 and s != sign:
            changes += 1
        sign = s
    return changes


def count_turning_points(q: np.ndarray, hysteresis: float) -> int:
    if len(q) < 2:
        return 0
    direction = 0
    ref = float(q[0])
    count = 0
    for x in q[1:]:
        x = float(x)
        if direction == 0:
            if x - ref >= hysteresis:
                direction, ref = 1, x
            elif ref - x >= hysteresis:
                direction, ref = -1, x
            continue
        if direction > 0:
            if x > ref:
                ref = x
            elif ref - x >= hysteresis:
                count += 1
                direction, ref = -1, x
        else:
            if x < ref:
                ref = x
            elif x - ref >= hysteresis:
                count += 1
                direction, ref = 1, x
    return count


def series_shape(q: np.ndarray, t: np.ndarray) -> dict[str, Any]:
    q = np.asarray(q, dtype=np.float64)
    t = np.asarray(t, dtype=np.float64)
    validate_grid(t, "series_shape")
    if q.shape != t.shape:
        raise GridMismatchError(f"series/time length mismatch: {q.shape[0]} vs {t.shape[0]}")
    if not np.all(np.isfinite(q)):
        raise GridMismatchError("series contains non-finite values")
    v = np.gradient(q, t)
    return {
        "samples": int(len(q)),
        "min": float(np.min(q)),
        "max": float(np.max(q)),
        "rom": float(np.max(q) - np.min(q)),
        "mean": float(np.mean(q)),
        "negative_fraction": float(np.mean(q < 0.0)),
        "positive_fraction": float(np.mean(q > 0.0)),
        "zero_crossings": count_sign_changes(q, ANGLE_DEADBAND_RAD),
        "turning_points": count_turning_points(q, ANGLE_TURNING_HYSTERESIS_RAD),
        "velocity_abs_max": float(np.max(np.abs(v))),
        "velocity_rms": float(np.sqrt(np.mean(np.square(v)))),
    }


def control_shape(u: np.ndarray) -> dict[str, Any]:
    u = np.asarray(u, dtype=np.float64)
    if not np.all(np.isfinite(u)):
        raise GridMismatchError("control series contains non-finite values")
    return {"samples": int(len(u)), "min": float(np.min(u)), "max": float(np.max(u)), "abs_max": float(np.max(np.abs(u))), "mean": float(np.mean(u)), "saturated_fraction_abs_ge_1": float(np.mean(np.abs(u) >= 1.0))}


def torque_shape(tau: np.ndarray, v_joint: np.ndarray, t: np.ndarray) -> dict[str, Any]:
    """Torque shape plus mechanical power/work on a validated common grid - no slicing."""
    tau = np.asarray(tau, dtype=np.float64)
    v_joint = np.asarray(v_joint, dtype=np.float64)
    t = np.asarray(t, dtype=np.float64)
    validate_grid(t, "torque_shape")
    if not (tau.shape == v_joint.shape == t.shape):
        raise GridMismatchError(f"torque/velocity/time length mismatch: {tau.shape[0]}/{v_joint.shape[0]}/{t.shape[0]}")
    if not (np.all(np.isfinite(tau)) and np.all(np.isfinite(v_joint))):
        raise GridMismatchError("torque or velocity contains non-finite values")
    with np.errstate(over="ignore", invalid="ignore"):
        power = tau * v_joint
        dt = np.gradient(t)
        out = {
            "min": float(np.min(tau)),
            "max": float(np.max(tau)),
            "rom": float(np.max(tau) - np.min(tau)),
            "abs_max": float(np.max(np.abs(tau))),
            "mean": float(np.mean(tau)),
            "sign_changes": count_sign_changes(tau, TORQUE_DEADBAND_NM),
            "power_mean_w": float(np.mean(power)),
            "power_abs_max_w": float(np.max(np.abs(power))),
            "work_positive_j": float(np.sum(np.clip(power, 0.0, None) * dt)),
            "work_negative_j": float(np.sum(np.clip(power, None, 0.0) * dt)),
            "work_net_j": float(np.sum(power * dt)),
        }
    return out


def tracking(served: np.ndarray, actual: np.ndarray) -> dict[str, Any]:
    served = np.asarray(served, dtype=np.float64)
    actual = np.asarray(actual, dtype=np.float64)
    if served.shape != actual.shape:
        raise GridMismatchError(f"served/actual length mismatch: {served.shape[0]} vs {actual.shape[0]}")
    if not (np.all(np.isfinite(served)) and np.all(np.isfinite(actual))):
        raise GridMismatchError("served or actual series contains non-finite values")
    err = served - actual
    corr = None
    if len(served) > 1 and np.std(served) > 0 and np.std(actual) > 0:
        corr = float(np.corrcoef(served, actual)[0, 1])
    return {"samples": int(len(served)), "rmse": float(np.sqrt(np.mean(np.square(err)))), "abs_max": float(np.max(np.abs(err))), "pearson_r": corr}


# --- provenance: CLI, pinned config, rollout_eval defaults ----------------------------


def rollout_eval_argparse_defaults(path: Path) -> dict[str, Any]:
    """Parse ``add_argument("--flag", ..., default=<const>)`` calls from the pinned
    rollout_eval.py source (AST; no import of the simulation stack)."""
    tree = ast.parse(path.read_text(encoding="utf-8"))
    defaults: dict[str, Any] = {}
    for node in ast.walk(tree):
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute) and node.func.attr == "add_argument" and node.args and isinstance(node.args[0], ast.Constant):
            flag = node.args[0].value
            for kw in node.keywords:
                if kw.arg == "default" and isinstance(kw.value, ast.Constant):
                    defaults[flag] = kw.value.value
    missing = [flag for flag in ROLLOUT_EVAL_CLI_FLAGS.values() if flag not in defaults]
    if missing:
        raise ProvenanceError(f"rollout_eval.py argparse defaults not found for {missing}")
    return {name: defaults[flag] for name, flag in ROLLOUT_EVAL_CLI_FLAGS.items()}


def cli_values(command: Any, flags: dict[str, str]) -> dict[str, Any]:
    """Extract ``--flag value`` pairs from the receipt command; fail closed on
    missing values, repeated flags with different values or non-parsable tokens."""
    if not isinstance(command, list) or not all(isinstance(tok, str) for tok in command):
        raise ProvenanceError("receipt command is not a list of strings")
    found: dict[str, Any] = {}
    for name, flag in flags.items():
        if name in CLI_BOOL_FLAGS:
            negative = "--no-" + flag[2:]
            bool_values = [tok == flag for tok in command if tok in (flag, negative)]
            if bool_values:
                if any(v != bool_values[0] for v in bool_values):
                    raise ProvenanceError(f"inconsistent CLI: {flag} and {negative} both present")
                found[name] = bool_values[0]
            continue
        positions = [i for i, tok in enumerate(command) if tok == flag or tok.startswith(flag + "=")]
        values = []
        for i in positions:
            tok = command[i]
            if "=" in tok and tok.startswith(flag + "="):
                raw = tok.split("=", 1)[1]
            else:
                if i + 1 >= len(command) or command[i + 1].startswith("--"):
                    raise ProvenanceError(f"malformed CLI: {flag} without value")
                raw = command[i + 1]
            if name == "actor_fsm_version":
                if raw not in FSM_VERSIONS:
                    raise ProvenanceError(f"malformed CLI: {flag}={raw!r} not in {FSM_VERSIONS}")
                values.append(raw)
            elif name in CLI_STRING_FLAGS:
                if not raw:
                    raise ProvenanceError(f"malformed CLI: {flag} with empty value")
                values.append(raw)
            else:
                try:
                    values.append(int(raw) if name == "max_steps" else float(raw))
                except ValueError as exc:
                    raise ProvenanceError(f"malformed CLI: {flag}={raw!r} not numeric") from exc
        if values:
            if any(v != values[0] for v in values):
                raise ProvenanceError(f"inconsistent CLI: {flag} repeated with different values {values}")
            found[name] = values[0]
    return found


def load_pinned_config(receipt: dict[str, Any]) -> tuple[dict[str, Any], dict[str, Any]]:
    rel_cfg = receipt.get("config")
    if not isinstance(rel_cfg, str) or not rel_cfg:
        raise ProvenanceError("receipt has no pinned config path")
    path = C.REPO / rel_cfg
    if not path.is_file():
        raise ProvenanceError(f"pinned config missing: {rel_cfg}")
    sha = C.sha256_file(path)
    if sha != receipt.get("config_sha256"):
        raise ProvenanceError(f"pinned config digest mismatch: receipt {receipt.get('config_sha256')} vs disk {sha}")
    cfg = C.load_yaml(path)
    if not isinstance(cfg, dict):
        raise ProvenanceError("pinned config is not a mapping")
    return cfg, {"config": rel_cfg, "config_sha256": sha}


def verify_rollout_eval_pin(receipt: dict[str, Any], path: Path) -> dict[str, Any]:
    if not path.is_file():
        raise ProvenanceError("rollout_eval.py missing")
    sha = C.sha256_file(path)
    if sha != receipt.get("rollout_eval_sha256"):
        raise ProvenanceError(f"rollout_eval.py digest mismatch: receipt {receipt.get('rollout_eval_sha256')} vs disk {sha}")
    return {"rollout_eval": C.rel(path), "rollout_eval_sha256": sha}


def expected_runtime(cfg: dict[str, Any], defaults: dict[str, Any], cli: dict[str, Any], runtime: str | None = None) -> dict[str, Any]:
    """Expected actor-FSM version, EXACT event/FSM contract identifiers,
    causal-ledger obligation, controller-diagnostic placement and full
    observation width from CLI > pinned config > argparse default (+ registry).

    Production semantics: the actor-facing ``phase_fsm.event_source`` is exactly
    ``binary_active_v26`` when ``binary_phase_fsm_mode == binary_active``
    (binary_phase_adapter_v26.V26_ACTOR_EVENT_SOURCE), otherwise exactly the
    configured ``phase_fsm_input_mode`` (e.g. ``legacy_events``). No prefix
    equivalence is ever inferred. ``include_controller_diagnostic_observation``
    resolved from CLI (BooleanOptionalAction) > config > default must equal the
    registry expectation of the runtime (False for every F0 runtime: the four
    sea_u_abs/saturated diagnostics live in the privileged suffix), otherwise
    the job was launched with the wrong actor contract (fail closed)."""
    grf = cfg.get("grf") if isinstance(cfg.get("grf"), dict) else {}
    reward = cfg.get("reward") if isinstance(cfg.get("reward"), dict) else {}
    flat = C.flatten(cfg)
    diag_keys = [k for k in flat if k.split(".")[-1] == "include_controller_diagnostic_observation"]
    if "include_controller_diagnostic_observation" in cli:
        include_diag, diag_source = bool(cli["include_controller_diagnostic_observation"]), "cli"
    elif diag_keys:
        if len(diag_keys) > 1 or not isinstance(flat[diag_keys[0]], bool):
            raise ProvenanceError(f"config include_controller_diagnostic_observation ambiguous or not boolean: {diag_keys}")
        include_diag, diag_source = flat[diag_keys[0]], "config"
    else:
        include_diag, diag_source = bool(defaults["include_controller_diagnostic_observation"]), "rollout_eval_default"
    runtime_spec = None
    if runtime is not None:
        if runtime not in C.RUNTIMES:
            raise ProvenanceError(f"runtime {runtime!r} unknown to the F0 registry")
        runtime_spec = C.RUNTIMES[runtime]
        if include_diag != runtime_spec["include_controller_diagnostic_observation"]:
            raise ProvenanceError(f"include_controller_diagnostic_observation resolved {include_diag} ({diag_source}) but the registry requires {runtime_spec['include_controller_diagnostic_observation']} for runtime {runtime!r} (actor contract without controller diagnostics)")
    if "actor_fsm_version" in cli:
        version, source = cli["actor_fsm_version"], "cli"
    elif isinstance(grf.get("binary_phase_actor_fsm_version"), str):
        version, source = grf["binary_phase_actor_fsm_version"], "config"
    else:
        version, source = defaults["actor_fsm_version"], "rollout_eval_default"
    if version not in FSM_VERSIONS:
        raise ProvenanceError(f"expected FSM version {version!r} not in {FSM_VERSIONS}")
    contracts: dict[str, str] = {}
    contract_sources: dict[str, str] = {}
    for name, cfg_key in CONFIG_GRF_KEYS.items():
        if name in cli:
            value, src = cli[name], "cli"
        elif cfg_key in grf:
            value, src = grf[cfg_key], "config"
        else:
            value, src = defaults[name], "rollout_eval_default"
        if not isinstance(value, str) or not value:
            raise ProvenanceError(f"expected {name} is not a non-empty string ({src}): {value!r}")
        contracts[name] = value
        contract_sources[name] = src
    mode = contracts["binary_phase_fsm_mode"]
    phase_mode = reward.get("morphology_phase_mode")
    if phase_mode is not None and not isinstance(phase_mode, str):
        raise ProvenanceError("config reward.morphology_phase_mode is not a string")
    causal_contract = reward.get("morphology_causal_event_contract_id")
    if causal_contract is not None and (not isinstance(causal_contract, str) or not causal_contract):
        raise ProvenanceError("config reward.morphology_causal_event_contract_id is not a non-empty string")
    causal_required = isinstance(phase_mode, str) and "causal" in phase_mode
    if causal_required and causal_contract is None:
        raise ProvenanceError("causal morphology mode pinned but reward.morphology_causal_event_contract_id absent from the config")
    return {
        "fsm_behaviour_version": version,
        "version_source": source,
        **contracts,
        "contract_sources": contract_sources,
        "event_source_expected": EVENT_SOURCE_BINARY_ACTIVE if mode == BINARY_ACTIVE_MODE else contracts["phase_fsm_input_mode"],
        "event_source_rule": f"esattamente '{EVENT_SOURCE_BINARY_ACTIVE}' se binary_phase_fsm_mode == '{BINARY_ACTIVE_MODE}', altrimenti esattamente phase_fsm_input_mode; nessuna equivalenza per prefisso",
        "include_controller_diagnostic_observation": include_diag,
        "include_controller_diagnostic_observation_source": diag_source,
        "expected_observation_width": int(runtime_spec["expected_observation_width"]) if runtime_spec else None,
        "runtime": runtime,
        "morphology_phase_mode": phase_mode,
        "morphology_causal_event_contract_id": causal_contract,
        "causal_ledger_required": causal_required,
        "causal_ledger_rule": "obbligatorio e non vuoto a ogni step se reward.morphology_phase_mode contiene 'causal'; altrimenti ammesso tutto vuoto, ma dal primo payload non vuoto in poi continuita' e contatori monotoni; ogni payload attivo porta event_contract_id esattamente uguale a reward.morphology_causal_event_contract_id e costante lungo la trace",
    }


def binary_v3_counters_required(version: Any, event_source: Any) -> bool:
    """resync/hs_cancelled counters are mandatory only on the v3 actor FSM fed by the
    exact binary-active V26 event source."""
    return version == "v3" and event_source == EVENT_SOURCE_BINARY_ACTIVE


def expected_horizon(cfg: dict[str, Any], defaults: dict[str, Any], cli: dict[str, Any]) -> dict[str, Any]:
    """Horizon quantities from the runtime semantics (values: CLI > pinned config > argparse defaults).

    progress_upper_bound    = min(max_steps, ceil(ed/sd))        rollout_eval.py:581-587 (progress bar only)
    env_horizon_steps       = ceil(ed/sd - 0.5)                   env: reached_horizon <=> t >= episode_end - 0.5*sd,
                                                                  t advancing by sd per policy step (osim_trj_cmc_like.py)
    loop_cap                = max_steps                            rollout_eval.py loop range(max_steps)
    expected_recorded_steps = min(loop_cap, env_horizon_steps)
    Dataset bounds (episode_end clamped to t_end) are NOT modelled: a ``dataset_end`` end reason is
    reported as such and never equated to the horizon."""
    sim = cfg.get("simulation") if isinstance(cfg.get("simulation"), dict) else {}
    values: dict[str, Any] = {}
    sources: dict[str, str] = {}
    for name, cfg_key in (("max_steps", None), ("episode_duration", "episode_duration"), ("segment_duration", "segment_duration")):
        if name in cli:
            values[name], sources[name] = cli[name], "cli"
        elif cfg_key is not None and cfg_key in sim:
            values[name], sources[name] = sim[cfg_key], "config"
        else:
            values[name], sources[name] = defaults[name], "rollout_eval_default"
    max_steps, ed, sd = values["max_steps"], values["episode_duration"], values["segment_duration"]
    if not (is_nonneg_int(max_steps) and float(max_steps) > 0):
        raise ProvenanceError(f"max_steps must be a positive integer, got {max_steps!r}")
    if not (is_finite_float(ed) and float(ed) > 0.0):
        raise ProvenanceError(f"episode_duration must be finite and > 0, got {ed!r}")
    if not (is_finite_float(sd) and float(sd) > 0.0):
        raise ProvenanceError(f"segment_duration must be finite and > 0, got {sd!r}")
    ratio = float(ed) / float(sd)
    if not math.isfinite(ratio) or ratio <= 0.0:
        raise ProvenanceError(f"episode_duration/segment_duration ratio invalid: {ratio!r}")
    progress_upper_bound = min(int(max_steps), int(math.ceil(ratio)))
    env_horizon = max(1, int(math.ceil(ratio - 0.5)))
    boundary_sensitive = abs((ratio - 0.5) - round(ratio - 0.5)) < HORIZON_BOUNDARY_EPS
    return {
        "progress_upper_bound": progress_upper_bound,
        "env_horizon_steps_expected": env_horizon,
        "loop_cap": int(max_steps),
        "expected_recorded_steps": min(int(max_steps), env_horizon),
        "loop_cap_below_env_horizon": int(max_steps) < env_horizon,
        "env_horizon_boundary_sensitive": boundary_sensitive,
        "rules": {
            "progress_upper_bound": "min(max_steps, ceil(episode_duration/segment_duration)) - rollout_eval.py:581-587, only the progress-bar bound",
            "env_horizon_steps_expected": "ceil(episode_duration/segment_duration - 0.5): env ends when t >= episode_end - 0.5*segment_duration with t advancing by segment_duration per step (osim_trj_cmc_like.py reached_horizon); dataset bounds not modelled",
            "expected_recorded_steps": "min(loop_cap=max_steps (rollout_eval.py loop range(max_steps)), env_horizon_steps_expected)",
        },
        "values": {"max_steps": int(max_steps), "episode_duration_s": float(ed), "segment_duration_s": float(sd), "ratio": ratio},
        "sources": sources,
    }


def horizon_outcome(summary: dict[str, Any], horizon: dict[str, Any]) -> dict[str, Any]:
    steps = int(summary["steps"])
    end_reason = summary["end_reason"]
    env_h = horizon["env_horizon_steps_expected"]
    cap = horizon["loop_cap"]
    return {
        **horizon,
        "steps_recorded": steps,
        "ended_at_dataset_end": end_reason == DATASET_END_REASON,
        "dataset_end_note": "dataset bounds shortened the episode: no horizon equivalence claimed" if end_reason == DATASET_END_REASON else None,
        "loop_cap_hit": steps == cap and cap < env_h,
        "horizon_completed": end_reason == HORIZON_END_REASON and steps == env_h,
        "horizon_end_reason_but_step_mismatch": end_reason == HORIZON_END_REASON and steps != env_h,
    }


# --- evidence ---------------------------------------------------------------------


def _feature_list_problems(summary: dict[str, Any], candidate: str | None = None) -> list[str]:
    """actor/observation feature-name lists: sized by n_actor/n_observation,
    non-empty unique strings; actor == observation[:n_actor] in exact order;
    no controller diagnostic inside the actor block; actor names exactly equal
    to the candidate's content-addressed manifest."""
    problems: list[str] = []
    for field, size_field in SUMMARY_FEATURE_LISTS.items():
        names = summary.get(field)
        if not isinstance(names, list):
            problems.append(f"{field} missing or not a list")
            continue
        if not all(isinstance(n, str) and n for n in names):
            problems.append(f"{field} contains empty or non-string names")
        if len(set(names)) != len(names):
            problems.append(f"{field} contains duplicate names")
        size = summary.get(size_field)
        if is_nonneg_int(size) and len(names) != int(size):
            problems.append(f"{field} length {len(names)} != {size_field} {int(size)}")
    actor = summary.get("actor_feature_names")
    obs = summary.get("observation_feature_names")
    if isinstance(actor, list) and isinstance(obs, list) and is_nonneg_int(summary.get("n_actor")):
        n = int(summary["n_actor"])
        if obs[:n] != actor:
            problems.append("observation_feature_names[:n_actor] != actor_feature_names (exact order required)")
    if isinstance(actor, list):
        diag = [n for n in actor if n in C.CONTROLLER_DIAGNOSTIC_FEATURES]
        if diag:
            problems.append(f"actor_feature_names contains controller diagnostics {diag} (must sit in the privileged suffix)")
    if candidate is not None and isinstance(actor, list):
        try:
            manifest = C.load_actor_feature_manifest(candidate)
        except (RuntimeError, OSError, ValueError) as exc:
            problems.append(f"actor feature manifest of {candidate} unusable: {exc}")
        else:
            if actor != manifest["actor_feature_names"]:
                problems.append(f"actor_feature_names != content-addressed manifest of {candidate} ({manifest['sha256'][:12]}..., {manifest['actor_feature_count']} names, exact order required)")
    return problems


def summary_evidence_matrix(summary: Any, path: Path, expected: dict[str, Any] | None = None, candidate: str | None = None) -> dict[str, Any]:
    """Summary schema + ABI (action_shape exactly [2], feature lists sized by
    n_actor/n_observation, actor prefix exact, manifest exact, n_actor ==
    registry width, n_observation == runtime full width, controller
    diagnostics excluded from the actor) + EXACT equality of the FSM/event
    contract fields with the pinned config."""
    rec: dict[str, Any] = {"path": C.rel(path), "present": path.is_file(), "sha256": C.sha256_file(path) if path.is_file() else None}
    if not isinstance(summary, dict):
        rec.update({"problems": ["summary is not a mapping"], "valid": False})
        return rec
    candidate_width = int(C.CANDIDATES[candidate]["width"]) if candidate in C.CANDIDATES else None
    problems = []
    for field, kind in SUMMARY_SCHEMA.items():
        if field not in summary:
            problems.append(f"{field} missing")
            continue
        problem = check_value(summary.get(field), kind)
        if problem:
            problems.append(f"{field}: {problem}")
    shape = summary.get("action_shape")
    if not isinstance(shape, list) or [int(x) if is_nonneg_int(x) else x for x in shape] != ACTION_SHAPE_EXPECTED:
        problems.append(f"action_shape {shape!r} != {ACTION_SHAPE_EXPECTED}")
    problems.extend(_feature_list_problems(summary, candidate))
    if candidate_width is not None and is_nonneg_int(summary.get("n_actor")) and int(summary["n_actor"]) != int(candidate_width):
        problems.append(f"n_actor {int(summary['n_actor'])} != registry width {int(candidate_width)} of the receipt candidate")
    if expected is not None:
        for field in SUMMARY_CONTRACT_FIELDS:
            if field in summary and summary.get(field) != expected[field]:
                problems.append(f"{field} {summary.get(field)!r} != pinned config/CLI {expected[field]!r} (exact match required)")
        width = expected.get("expected_observation_width")
        if width is not None and is_nonneg_int(summary.get("n_observation")) and int(summary["n_observation"]) != int(width):
            problems.append(f"n_observation {int(summary['n_observation'])} != expected full observation width {int(width)} of the runtime")
        flag = summary.get("include_controller_diagnostic_observation")
        if not isinstance(flag, bool):
            problems.append("include_controller_diagnostic_observation missing or not boolean in the summary")
        elif flag != expected["include_controller_diagnostic_observation"]:
            problems.append(f"include_controller_diagnostic_observation {flag} != expected {expected['include_controller_diagnostic_observation']} ({expected['include_controller_diagnostic_observation_source']})")
    rec["problems"] = problems
    rec["valid"] = not problems
    rec["abi"] = {"n_actor": summary.get("n_actor"), "n_observation": summary.get("n_observation"), "action_shape": shape, "registry_width": candidate_width, "include_controller_diagnostic_observation": summary.get("include_controller_diagnostic_observation")}
    return rec


def reserve_evidence(path: Path, steps: int | None) -> dict[str, Any]:
    """Reserve file: expected columns plus EVERY ``*_reserve_torque`` column
    actually consumed by the norm must be present, non-empty and finite."""
    rec = sto_evidence(path, EXPECTED_STO[RESERVE_FILE], steps)
    if not rec.get("parse_ok"):
        rec["consumed_columns"] = []
        rec["consumed_nonfinite_count"] = None
        rec["valid"] = False
        return rec
    names, data = read_sto(path)
    consumed = [n for n in names if n.endswith(RESERVE_COLUMN_SUFFIX)]
    rec["consumed_columns"] = consumed
    if not consumed:
        rec["consumed_nonfinite_count"] = None
        rec["valid"] = False
        rec["consumed_problem"] = "no *_reserve_torque column"
        return rec
    block = data[:, [names.index(n) for n in consumed]]
    nonfinite = int(np.count_nonzero(~np.isfinite(block)))
    rec["consumed_nonfinite_count"] = nonfinite
    if nonfinite:
        rec["finite"] = False
        rec["valid"] = False
        rec["consumed_problem"] = f"{nonfinite} non-finite values in consumed reserve columns"
    return rec


def _validate_fsm_payload(fsm: Any, expected: dict[str, Any] | None) -> list[str]:
    problems: list[str] = []
    if not isinstance(fsm, dict):
        return ["phase_fsm is not a mapping"]
    for key in FSM_REQUIRED_STRINGS:
        if not isinstance(fsm.get(key), str) or not fsm.get(key):
            problems.append(f"phase_fsm.{key} missing or not a string")
    version = fsm.get("fsm_behaviour_version")
    if isinstance(version, str) and version not in FSM_VERSIONS:
        problems.append(f"phase_fsm.fsm_behaviour_version unexpected: {version!r}")
    for key in FSM_REQUIRED_COUNTERS:
        if key not in fsm:
            problems.append(f"phase_fsm.{key} missing")
        elif not is_nonneg_int(fsm.get(key)):
            problems.append(f"phase_fsm.{key} not a finite non-negative integer: {fsm.get(key)!r}")
    source = fsm.get("event_source") if isinstance(fsm.get("event_source"), str) else ""
    binary_v3 = binary_v3_counters_required(version, source)
    for key in FSM_OPTIONAL_COUNTERS:
        if key in fsm and fsm.get(key) is not None:
            if not is_nonneg_int(fsm.get(key)):
                problems.append(f"phase_fsm.{key} not a finite non-negative integer: {fsm.get(key)!r}")
        elif binary_v3:
            problems.append(f"phase_fsm.{key} required on v3 with event_source {EVENT_SOURCE_BINARY_ACTIVE!r} but absent")
    if expected is not None:
        if isinstance(version, str) and version != expected["fsm_behaviour_version"]:
            problems.append(f"phase_fsm.fsm_behaviour_version {version!r} != expected {expected['fsm_behaviour_version']!r} ({expected['version_source']})")
        if source != expected["event_source_expected"]:
            problems.append(f"phase_fsm.event_source {source!r} != expected exactly {expected['event_source_expected']!r} (binary_phase_fsm_mode {expected['binary_phase_fsm_mode']!r}, phase_fsm_input_mode {expected['phase_fsm_input_mode']!r})")
    return problems


def _validate_causal_payload(causal: Any, required: bool, expected_contract: str | None = None) -> list[str]:
    if not isinstance(causal, dict):
        return ["morphology_causal_diagnostics is not a mapping"]
    if not causal:
        return ["morphology_causal_diagnostics empty but required by the causal morphology mode"] if required else []
    problems = []
    for key in CAUSAL_COUNTERS:
        if key not in causal:
            problems.append(f"morphology_causal_diagnostics.{key} missing")
        elif not is_nonneg_int(causal.get(key)):
            problems.append(f"morphology_causal_diagnostics.{key} not a finite non-negative integer: {causal.get(key)!r}")
    contract = causal.get("event_contract_id")
    if not isinstance(contract, str) or not contract:
        problems.append("morphology_causal_diagnostics.event_contract_id missing on an active payload")
    elif expected_contract is None:
        problems.append(f"morphology_causal_diagnostics.event_contract_id {contract!r} present but the pinned config has no reward.morphology_causal_event_contract_id")
    elif contract != expected_contract:
        problems.append(f"morphology_causal_diagnostics.event_contract_id {contract!r} != config {expected_contract!r} (exact match required)")
    return problems


def _vector_problems(row: dict[str, Any], step: int, n_actor: int | None) -> tuple[list[str], bool]:
    """Action vectors exactly ACTION_WIDTH wide, observation vector exactly n_actor wide, all finite."""
    problems: list[str] = []
    nonfinite = False
    for key, width in (("raw_policy_action", ACTION_WIDTH), ("applied_policy_action", ACTION_WIDTH), ("actor_observation_vector_before", n_actor)):
        vec = row.get(key)
        if not isinstance(vec, list) or not vec:
            problems.append(f"step {step}: {key} missing or empty")
            continue
        if not all(isinstance(x, (int, float)) and not isinstance(x, bool) for x in vec):
            problems.append(f"step {step}: {key} contains non-numeric values")
        elif not all(math.isfinite(float(x)) for x in vec):
            problems.append(f"step {step}: {key} contains non-finite values")
            nonfinite = True
        if width is None:
            problems.append(f"step {step}: {key} width cannot be checked (summary n_actor invalid)")
        elif len(vec) != int(width):
            problems.append(f"step {step}: {key} width {len(vec)} != expected {int(width)}")
    return problems, nonfinite


def _sequence_problems(rows: list[dict[str, Any]]) -> tuple[list[str], bool]:
    """``step`` must be the exact integer sequence 1..len(rows); ``time`` finite and strictly increasing."""
    problems: list[str] = []
    previous_time: float | None = None
    ok = True
    for i, r in enumerate(rows):
        step = r.get("step")
        if not is_nonneg_int(step) or int(step) != i + 1:
            problems.append(f"row {i}: step {step!r} != expected {i + 1} (exact 1..steps sequence)")
            ok = False
        t = r.get("time")
        if not is_finite_float(t):
            problems.append(f"row {i}: time not finite: {t!r}")
            ok = False
        elif previous_time is not None and float(t) <= previous_time:
            problems.append(f"row {i}: time {t!r} not strictly increasing after {previous_time!r}")
            ok = False
        if is_finite_float(t):
            previous_time = float(t)
    return problems, ok


def trace_evidence_matrix(path: Path, steps: int | None, summary: dict[str, Any] | None = None, expected: dict[str, Any] | None = None, n_actor: int | None = None) -> dict[str, Any]:
    rec: dict[str, Any] = {"path": C.rel(path), "present": path.is_file(), "sha256": C.sha256_file(path) if path.is_file() else None}
    invalid = {"parse_ok": False, "steps": None, "steps_consistent": False, "required_keys_complete": False, "missing_keys": list(TRACE_REQUIRED_KEYS), "payload_problems": [], "payload_problem_count": 0, "finite": False, "observation_widths": [], "sequence_ok": False, "constant_runtime": False, "counters_monotonic": False, "summary_coherent": False, "causal_continuity": False, "causal_contract_constant": False, "valid": False}
    if not rec["present"]:
        rec.update(invalid)
        return rec
    try:
        rows = json.loads(path.read_text(encoding="utf-8"))
        if not isinstance(rows, list) or not rows or not all(isinstance(r, dict) for r in rows):
            raise ValueError("trace is not a non-empty list of mappings")
        rec["parse_ok"] = True
    except Exception as exc:  # noqa: BLE001
        rec.update(invalid)
        rec["parse_error"] = f"{type(exc).__name__}: {exc}"
        return rec
    causal_required = bool(expected and expected.get("causal_ledger_required"))
    expected_contract = expected.get("morphology_causal_event_contract_id") if expected else None
    rec["steps"] = len(rows)
    rec["steps_consistent"] = steps is not None and len(rows) == int(steps)
    missing = sorted({k for r in rows for k in TRACE_REQUIRED_KEYS if k not in r or r[k] is None})
    rec["missing_keys"] = missing
    rec["required_keys_complete"] = not missing
    problems: list[str] = []
    finite = True
    widths: set[int] = set()
    seq_problems, sequence_ok = _sequence_problems(rows)
    problems.extend(seq_problems)
    rec["sequence_ok"] = sequence_ok
    causal_contracts: set[str] = set()
    for i, r in enumerate(rows):
        vec_problems, nonfinite = _vector_problems(r, i, n_actor)
        problems.extend(vec_problems)
        finite = finite and not nonfinite
        obs = r.get("actor_observation_vector_before")
        if isinstance(obs, list):
            widths.add(len(obs))
        fsm_problems = _validate_fsm_payload(r.get("phase_fsm"), expected)
        causal_problems = _validate_causal_payload(r.get("morphology_causal_diagnostics"), causal_required, expected_contract)
        causal = r.get("morphology_causal_diagnostics")
        if isinstance(causal, dict) and causal and isinstance(causal.get("event_contract_id"), str):
            causal_contracts.add(causal["event_contract_id"])
        for p in fsm_problems + causal_problems:
            problems.append(f"step {i}: {p}")
            if "not a finite" in p:
                finite = False
    rec["causal_contract_constant"] = len(causal_contracts) <= 1
    rec["causal_event_contract_ids"] = sorted(causal_contracts)
    if len(causal_contracts) > 1:
        # trace-level finding first: the per-row mismatches that follow would otherwise hide it in the truncated list
        problems.insert(0, f"morphology_causal_diagnostics.event_contract_id changes along the trace: {sorted(causal_contracts)}")
    # causal ledger continuity: once active it can never disappear
    activity = [bool(r.get("morphology_causal_diagnostics")) if isinstance(r.get("morphology_causal_diagnostics"), dict) else None for r in rows]
    first_active = next((i for i, a in enumerate(activity) if a), None)
    continuity = True
    if first_active is not None and any(a is not True for a in activity[first_active:]):
        continuity = False
        gap = next(i for i, a in enumerate(activity[first_active:], start=first_active) if a is not True)
        problems.append(f"morphology_causal_diagnostics active from step {first_active} but empty/invalid at step {gap} (continuity broken)")
    if causal_required and first_active != 0:
        continuity = False
        problems.append("morphology_causal_diagnostics required from step 0 by the causal morphology mode")
    rec["causal_continuity"] = continuity
    rec["causal_ledger_required"] = causal_required
    rec["causal_ledger_first_active_step"] = first_active
    # runtime constancy across the trace
    first_fsm = rows[0].get("phase_fsm") if isinstance(rows[0].get("phase_fsm"), dict) else {}
    versions = {str((r.get("phase_fsm") or {}).get("fsm_behaviour_version")) if isinstance(r.get("phase_fsm"), dict) else "<invalid>" for r in rows}
    sources = {str((r.get("phase_fsm") or {}).get("event_source")) if isinstance(r.get("phase_fsm"), dict) else "<invalid>" for r in rows}
    rec["constant_runtime"] = len(versions) == 1 and len(sources) == 1
    if not rec["constant_runtime"]:
        problems.append(f"fsm_behaviour_version/event_source not constant across the trace: {sorted(versions)} / {sorted(sources)}")
    # cumulative counters must be non-decreasing, EXCEPT valid_hs_count under the v3 heel-strike
    # cancellation contract (prosthetic_phase_fsm.py _cancel_bounced_heel_strike, 1808-1840): a TO inside the
    # minimum stance revokes a young HS -> valid_hs_count -1 in the SAME step as hs_cancelled_count +1 and
    # invalid_event_count +1; accepted HS = valid_hs_count + hs_cancelled_count never decreases. The contract is
    # applied only when hs_cancelled_count is carried on every row (v3); v2/legacy stay strictly monotonic.
    monotonic = True
    fsm_rows = [r.get("phase_fsm") for r in rows]
    fsm_all_dicts = all(isinstance(f, dict) for f in fsm_rows)

    def fsm_series(key: str) -> list[Any]:
        return [f.get(key) for f in fsm_rows] if fsm_all_dicts else []

    # the cancellation contract applies ONLY when the FSM version is constant and exactly "v3" and
    # hs_cancelled_count is an integer on every row; v2/legacy stay strictly monotonic even if the
    # payload happens to carry hs_cancelled_count. All arithmetic is exact integer arithmetic.
    cancel_contract = fsm_all_dicts and versions == {"v3"} and all(is_nonneg_int(f.get("hs_cancelled_count")) for f in fsm_rows)
    rec["hs_cancellation_contract_applied"] = cancel_contract
    rec["hs_cancellations_observed"] = 0
    for key in (*FSM_REQUIRED_COUNTERS, *FSM_OPTIONAL_COUNTERS):
        series = fsm_series(key)
        if not series or not all(is_nonneg_int(v) for v in series):
            continue  # absent optional counters are reported present=false elsewhere; malformed ones already flagged
        ints = [as_int(v) for v in series]
        if key == "valid_hs_count" and cancel_contract:
            cancelled = [as_int(v) for v in fsm_series("hs_cancelled_count")]
            invalid_raw = fsm_series("invalid_event_count")
            if all(is_nonneg_int(v) for v in invalid_raw):
                invalid = [as_int(v) for v in invalid_raw]
                for i in range(1, len(ints)):
                    d_hs = ints[i] - ints[i - 1]
                    d_c = cancelled[i] - cancelled[i - 1]
                    d_i = invalid[i] - invalid[i - 1]
                    if d_hs < 0:
                        if d_hs == -1 and d_c == 1 and d_i == 1:
                            rec["hs_cancellations_observed"] += 1
                        else:
                            monotonic = False
                            problems.append(f"phase_fsm.valid_hs_count decreases by {-d_hs} at row {i} without a matching heel-strike cancellation (hs_cancelled_count +{d_c}, invalid_event_count +{d_i}); allowed only exactly -1 with hs_cancelled_count +1 and invalid_event_count +1 in the same step")
                    if ints[i] + cancelled[i] < ints[i - 1] + cancelled[i - 1]:
                        monotonic = False
                        problems.append(f"phase_fsm.valid_hs_count + hs_cancelled_count (accepted heel strikes) decreases at row {i}")
            continue
        if any(b < a for a, b in zip(ints[:-1], ints[1:])):
            monotonic = False
            problems.append(f"phase_fsm.{key} decreases along the trace")
    if first_active is not None and continuity:
        for key in CAUSAL_COUNTERS:
            series = [r["morphology_causal_diagnostics"].get(key) for r in rows[first_active:]]
            if all(is_nonneg_int(v) for v in series):
                ints = [as_int(v) for v in series]
                if any(b < a for a, b in zip(ints[:-1], ints[1:])):
                    monotonic = False
                    problems.append(f"morphology_causal_diagnostics.{key} decreases along the trace")
    rec["counters_monotonic"] = monotonic
    # summary counters vs the RUNNING MAXIMUM of the trace (rollout_eval.py 736-745 keeps the max of the live
    # value, not the last step: after a terminal cancellation summary != last step); exact integers
    coherent = True
    rec["summary_vs_trace"] = {}
    if isinstance(summary, dict):
        for s_key, t_key in SUMMARY_TO_TRACE_COUNTERS.items():
            series = fsm_series(t_key)
            comparable = is_nonneg_int(summary.get(s_key)) and bool(series) and all(is_nonneg_int(v) for v in series)
            if comparable:
                ints = [as_int(v) for v in series]
                trace_max = max(ints)
                trace_last = ints[-1]
                summary_int = as_int(summary[s_key])
                rec["summary_vs_trace"][s_key] = {"summary": summary_int, "trace_max": trace_max, "trace_last": trace_last}
                if summary_int != trace_max:
                    coherent = False
                    problems.append(f"summary.{s_key}={summary_int} != running max of trace {t_key}={trace_max} (rollout_eval keeps the max; last step {trace_last})")
            else:
                coherent = False
                problems.append(f"summary.{s_key}/trace {t_key} not comparable")
    rec["summary_coherent"] = coherent
    rec["payload_problems"] = problems[:40]
    rec["payload_problem_count"] = len(problems)
    rec["finite"] = finite
    rec["observation_widths"] = sorted(widths)
    rec["fsm_behaviour_version"] = first_fsm.get("fsm_behaviour_version")
    rec["event_source"] = first_fsm.get("event_source")
    rec["has_legacy_online_events_key"] = all("legacy_online_events" in r for r in rows)
    rec["valid"] = bool(rec["steps_consistent"] and not missing and not problems and len(widths) == 1 and finite and sequence_ok)
    return rec


def collect_evidence(out_dir: Path, summary: Any, expected: dict[str, Any] | None, candidate: str | None = None) -> dict[str, Any]:
    ev: dict[str, Any] = {"summary": summary_evidence_matrix(summary, out_dir / M.SUMMARY_FILE, expected, candidate)}
    steps = summary.get("steps") if (isinstance(summary, dict) and is_nonneg_int(summary.get("steps"))) else None
    n_actor = int(summary["n_actor"]) if (isinstance(summary, dict) and is_nonneg_int(summary.get("n_actor")) and int(summary["n_actor"]) > 0) else None
    for fname, cols in EXPECTED_STO.items():
        path = out_dir / "sim_outputs" / fname
        ev[fname] = reserve_evidence(path, steps) if fname == RESERVE_FILE else sto_evidence(path, cols, steps)
        ev[fname]["semantics"] = STO_SEMANTICS[fname]
    ev["trace"] = trace_evidence_matrix(out_dir / M.TRACE_FILE, steps, summary if isinstance(summary, dict) else None, expected, n_actor)
    ev["_missing_or_invalid"] = [item for item in EXPECTED_EVIDENCE if ev[item].get("valid") is not True]
    ev["_complete"] = not ev["_missing_or_invalid"]
    return ev


# --- per-job metrics ----------------------------------------------------------------


def job_metrics(out_dir: Path, summary: dict[str, Any], receipt: dict[str, Any], runtime_label: str | None, provenance: dict[str, Any]) -> dict[str, Any]:
    """Metrics of one job with complete, validated evidence and verified provenance."""
    stos = {fname: read_sto(out_dir / "sim_outputs" / fname) for fname in EXPECTED_STO}
    rows = json.loads((out_dir / M.TRACE_FILE).read_text(encoding="utf-8"))
    last = rows[-1]
    fsm = last["phase_fsm"]
    causal = last["morphology_causal_diagnostics"]
    k_names, k_data = stos[KINEMATICS_FILE]
    t = column(k_names, k_data, "time")
    validate_grid(t, KINEMATICS_FILE)
    grids = {}
    for fname, (names, data) in stos.items():
        if fname == KINEMATICS_FILE:
            continue
        verify_grid(t, column(names, data, "time"), f"{KINEMATICS_FILE} vs {fname}")
        grids[fname] = "identical"
    r_names, r_data = stos[REFERENCE_FILE]
    u_names, u_data = stos[SEA_CONTROLS_FILE]
    tq_names, tq_data = stos[SEA_TORQUES_FILE]
    g_names, g_data = stos[GRF_FILE]
    rs_names, rs_data = stos[RESERVE_FILE]
    version = fsm.get("fsm_behaviour_version")
    metrics: dict[str, Any] = {
        "steps": int(summary["steps"]),
        "horizon": horizon_outcome(summary, provenance["horizon"]),
        "horizon_provenance": {k: provenance[k] for k in ("config", "config_sha256", "rollout_eval", "rollout_eval_sha256")},
        "episode_return": float(summary["episode_return"]),
        "reward_mean": float(summary["reward_mean"]),
        "end_reason": summary["end_reason"],
        "time_grids_verified_against_kinematics": grids,
        "abi": {
            "n_actor": int(summary["n_actor"]),
            "n_observation": int(summary["n_observation"]),
            "n_observation_expected": int(provenance["expected_runtime"]["expected_observation_width"]),
            "action_shape": [int(x) for x in summary["action_shape"]],
            "registry_width": int(provenance["candidate_width"]),
            "actor_prefix_exact": summary["observation_feature_names"][: int(summary["n_actor"])] == summary["actor_feature_names"],
            "actor_feature_manifest": provenance["actor_feature_manifest"]["path"],
            "actor_feature_manifest_sha256": provenance["actor_feature_manifest"]["sha256"],
            "actor_equals_manifest": summary["actor_feature_names"] == provenance["actor_feature_manifest"]["actor_feature_names"],
            "include_controller_diagnostic_observation": bool(summary["include_controller_diagnostic_observation"]),
            "actor_feature_names_sha256": C.sha256_text("\n".join(summary["actor_feature_names"])),
            "observation_feature_names_sha256": C.sha256_text("\n".join(summary["observation_feature_names"])),
        },
        "fsm_runtime": {
            "fsm_behaviour_version": version,
            "event_source": fsm.get("event_source"),
            **{k: summary.get(k) for k in SUMMARY_CONTRACT_FIELDS},
            "runtime_label": runtime_label,
            "expected": provenance["expected_runtime"],
            "note": f"eventi e contatori riferiti alla FSM attore {version} con sorgente eventi {fsm.get('event_source')}",
        },
        "actor_fsm_events_summary": {k: int(summary[k]) for k in SUMMARY_TO_TRACE_COUNTERS},
        "actor_fsm_events_summary_semantics": "rollout_eval.py 736-745: massimo corrente del valore vivo lungo l'episodio (== ultimo step solo senza cancellazioni terminali)",
        "actor_fsm_events_trace_last_step": {k: counter(fsm, k) for k in FSM_REQUIRED_COUNTERS},
        "actor_fsm_events_trace_last_step_semantics": "valori vivi all'ultimo step: valid_hs_count = heel strike attualmente validi (revocabili in FSM v3, prosthetic_phase_fsm.py 1833), gli altri cumulativi",
        "actor_fsm_events_trace_max": {k: {"present": True, "value": max(as_int(r["phase_fsm"][k]) for r in rows)} if all(is_nonneg_int(r["phase_fsm"].get(k)) for r in rows) else {"present": False, "value": None} for k in FSM_REQUIRED_COUNTERS},
        "fsm_counters": {
            "resync_count": counter(fsm, "resync_count"),
            "hs_cancelled_count": counter(fsm, "hs_cancelled_count"),
            "valid_hs_count_live_last_step": counter(fsm, "valid_hs_count"),
            "accepted_hs_count": {"present": True, "value": as_int(fsm["valid_hs_count"]) + as_int(fsm["hs_cancelled_count"])} if (version == "v3" and is_nonneg_int(fsm.get("hs_cancelled_count")) and is_nonneg_int(fsm.get("valid_hs_count"))) else {"present": False, "value": None},
            "accepted_hs_semantics": "accepted_hs_count = valid_hs_count + hs_cancelled_count (heel strike accettati, cumulativo, mai decrescente; interi esatti); presente solo sotto il contratto di cancellazione della FSM v3, assente su v2/legacy",
            "required_on_this_runtime": binary_v3_counters_required(version, fsm.get("event_source")),
            "source_field": "rollout_policy_trace.json[-1].phase_fsm",
        },
        "causal_ledger": {
            **{k: counter(causal, k) for k in CAUSAL_COUNTERS},
            "failed_closed": causal.get("failed_closed") if causal else None,
            "terminal_flushed": causal.get("terminal_flushed") if causal else None,
            "event_contract_id": causal.get("event_contract_id") if causal else None,
            "present": bool(causal),
            "required_by_config": provenance["expected_runtime"]["causal_ledger_required"],
            "source_field": "rollout_policy_trace.json[-1].morphology_causal_diagnostics",
        },
        "morphology_summary": {k: int(summary[k]) for k in ("morphology_settled_segments", "morphology_settled_samples", "morphology_discarded_segments", "morphology_discarded_samples")},
    }
    if all("legacy_online_events" in r for r in rows):
        hs = to = 0
        for r in rows:
            for ev in r.get("legacy_online_events") or []:
                if isinstance(ev, dict) and ev.get("side") == "left":
                    if ev.get("event") == "heel_strike":
                        hs += 1
                    elif ev.get("event") == "toe_off":
                        to += 1
        metrics["events_legacy_left"] = {"present": True, "heel_strike": hs, "toe_off": to}
    else:
        metrics["events_legacy_left"] = {"present": False, "heel_strike": None, "toe_off": None}
    pen = column(g_names, g_data, "left_penetration")
    force = column(g_names, g_data, "left_normal_force")
    contact = column(g_names, g_data, "left_in_contact")
    thr_pen = float(summary["grf_penetration_penalty_threshold_m"])
    metrics["penetration"] = {
        "summary_max_m": float(summary["grf_penetration_max_m"]),
        "series_max_m": float(np.max(pen)),
        "series_max_time_s": float(t[int(np.argmax(pen))]),
        "penalty_threshold_m": thr_pen,
        "termination_m": float(summary["grf_penetration_termination_m"]),
        "fraction_above_penalty_threshold": float(np.mean(pen > thr_pen)),
        "left_normal_force_max_n": float(np.max(force)),
        "left_contact_fraction": float(np.mean(contact > 0.5)),
    }
    reserve_cols = [n for n in rs_names if n.endswith(RESERVE_COLUMN_SUFFIX)]
    if not reserve_cols:
        raise MetricsFiniteError("no *_reserve_torque column in the reserve file")
    block = rs_data[:, [rs_names.index(n) for n in reserve_cols]]
    if not np.all(np.isfinite(block)):
        raise MetricsFiniteError("non-finite values in consumed reserve columns")
    reserve_norm = np.sqrt(np.sum(np.square(block), axis=1))
    metrics["reserve"] = {
        "summary_max_nm": float(summary["reserve_norm_max_nm"]),
        "series_norm_max_nm": float(np.max(reserve_norm)),
        "series_norm_mean_nm": float(np.mean(reserve_norm)),
        "columns": len(reserve_cols),
        "prosthetic_abs_max_nm": float(max(np.max(np.abs(column(rs_names, rs_data, "pros_knee_angle_reserve_torque"))), np.max(np.abs(column(rs_names, rs_data, "pros_ankle_angle_reserve_torque"))))),
    }
    raw = np.asarray([r["raw_policy_action"] for r in rows], dtype=np.float64)
    applied = np.asarray([r["applied_policy_action"] for r in rows], dtype=np.float64)
    metrics["actions"] = {"raw_abs_max": float(np.max(np.abs(raw))), "applied_abs_max": float(np.max(np.abs(applied))), "raw_mean": raw.mean(axis=0).tolist(), "raw_std": raw.std(axis=0).tolist(), "clipped_steps_summary": int(summary["action_clipped_steps"]), "action_abs_max_summary": float(summary["action_abs_max"])}
    metrics["kinematics_actual"] = {}
    metrics["served_reference"] = {}
    metrics["tracking_served_vs_actual"] = {}
    metrics["sea_controls_normalized"] = {}
    metrics["sea_torques"] = {}
    for spec in JOINTS:
        joint = spec["joint"]
        q_actual = column(k_names, k_data, spec["q"])
        q_ref = column(r_names, r_data, spec["q_ref"])
        metrics["kinematics_actual"][joint] = series_shape(q_actual, t)
        metrics["served_reference"][joint] = series_shape(q_ref, t)
        metrics["tracking_served_vs_actual"][joint] = tracking(q_ref, q_actual)
        metrics["sea_controls_normalized"][joint] = control_shape(column(u_names, u_data, spec["u"]))
        v_joint = np.gradient(q_actual, t)
        metrics["sea_torques"][joint] = {"tau_spring": torque_shape(column(tq_names, tq_data, spec["tau_spring"]), v_joint, t), "tau_motor_abs_max": float(np.max(np.abs(column(tq_names, tq_data, spec["tau_motor"]))))}
    assert_finite_metrics(metrics)
    return metrics


def analyse_job(rec: dict[str, Any], *, verify: Callable[[dict[str, Any], Path], Any] = M.verify_existing, rollout_eval_path: Path | None = None) -> dict[str, Any]:
    out_dir = C.REPO / rec["output_dir"]
    result: dict[str, Any] = {k: rec.get(k) for k in ("job_id", "family", "comparison_class", "candidate", "runtime", "start", "action_selection", "seed", "repeat", "output_dir", "historical_reference_summary", "policy_native_to_runtime", "runtime_is_target_v3")}
    result["isometric_comparable"] = rec.get("comparison_class") == ISOMETRIC_CLASS
    result["comparison_note"] = "confronto isometrico (runtime v3 pinnato, contratto 35D, stessa catena B0820)" if result["isometric_comparable"] else NON_ISOMETRIC_NOTE.get(rec.get("comparison_class"), "non isometrico")
    if not out_dir.exists():
        result.update({"executed": False, "verdict": "NOT_EXECUTED", "analysis_complete": False})
        return result
    result["executed"] = True
    try:
        verify(rec, out_dir)
        result["integrity"] = "PASS"
    except RuntimeError as exc:
        result.update({"integrity": f"FAIL: {str(exc)[:500]}", "verdict": "FAIL_VERIFICATION", "analysis_complete": False})
        return result
    try:
        summary = C.read_json(out_dir / M.SUMMARY_FILE)
        receipt = C.read_json(out_dir / M.RECEIPT_FILE)
    except (OSError, ValueError) as exc:
        result.update({"verdict": "FAIL_OUTPUT_INCOMPLETE", "analysis_complete": False, "verdict_reason": f"unreadable summary/receipt: {exc}"})
        return result
    result["receipt"] = {k: receipt.get(k) for k in ("status", "returncode", "duration_s", "summary_sha256", "trace_sha256", "git_head", "config", "config_sha256", "rollout_eval_sha256", "module_state_sha256", "candidate", "schema_version", "provenance_class", "source_closure_unchanged")}
    result["receipt"]["provenance_class_effective"] = receipt.get("provenance_class") or ("B" if receipt.get("schema_version") in M.LEGACY_RECEIPT_SCHEMA_VERSIONS else None)
    try:
        candidate = receipt.get("candidate")
        if not isinstance(candidate, str) or candidate not in C.CANDIDATES:
            raise ProvenanceError(f"receipt candidate {candidate!r} unknown to the F0 registry")
        if candidate != rec.get("candidate"):
            raise ProvenanceError(f"receipt candidate {candidate!r} != job candidate {rec.get('candidate')!r}")
        cfg, cfg_prov = load_pinned_config(receipt)
        eval_path = rollout_eval_path or C.ROLLOUT_EVAL
        eval_prov = verify_rollout_eval_pin(receipt, eval_path)
        defaults = rollout_eval_argparse_defaults(eval_path)
        cli = cli_values(receipt.get("command"), ROLLOUT_EVAL_CLI_FLAGS)
        try:
            manifest = C.load_actor_feature_manifest(candidate)
        except RuntimeError as exc:
            raise ProvenanceError(str(exc)) from exc
        provenance = {**cfg_prov, **eval_prov, "candidate": candidate, "candidate_width": int(C.CANDIDATES[candidate]["width"]), "actor_feature_manifest": manifest, "cli": cli, "rollout_eval_defaults": defaults, "expected_runtime": expected_runtime(cfg, defaults, cli, rec.get("runtime")), "horizon": expected_horizon(cfg, defaults, cli)}
    except (ProvenanceError, OSError, ValueError, SyntaxError) as exc:
        result.update({"verdict": "FAIL_PROVENANCE", "analysis_complete": False, "verdict_reason": f"{type(exc).__name__}: {exc}"})
        return result
    result["provenance"] = {k: provenance[k] for k in ("config", "config_sha256", "rollout_eval", "rollout_eval_sha256", "candidate", "candidate_width", "cli", "expected_runtime")}
    result["provenance"]["actor_feature_manifest"] = {k: provenance["actor_feature_manifest"][k] for k in ("path", "sha256", "actor_feature_count")}
    evidence = collect_evidence(out_dir, summary, provenance["expected_runtime"], candidate)
    result["evidence"] = evidence
    if not evidence["_complete"]:
        result.update({"verdict": "FAIL_OUTPUT_INCOMPLETE", "analysis_complete": False, "verdict_reason": f"missing or invalid evidence: {evidence['_missing_or_invalid']}; summary problems: {evidence['summary'].get('problems', [])[:5]}; trace problems: {evidence['trace'].get('payload_problems', [])[:5]}; reserve: {evidence[RESERVE_FILE].get('consumed_problem')}"})
        return result
    try:
        result["metrics"] = job_metrics(out_dir, summary, receipt, rec.get("runtime"), provenance)
    except (GridMismatchError, MetricsFiniteError, ProvenanceError, RuntimeError, ValueError, KeyError, TypeError) as exc:
        result.update({"verdict": "FAIL_METRICS", "analysis_complete": False, "verdict_reason": f"{type(exc).__name__}: {exc}"})
        return result
    result.update({"verdict": "PASS_ANALYSED", "analysis_complete": True})
    return result


# --- per-start comparisons ------------------------------------------------------


def _pick(m: dict[str, Any]) -> dict[str, Any]:
    return {
        "steps": m["steps"],
        "horizon_completed": m["horizon"]["horizon_completed"],
        "loop_cap_hit": m["horizon"]["loop_cap_hit"],
        "episode_return": m["episode_return"],
        "end_reason": m["end_reason"],
        "fsm_version": m["fsm_runtime"]["fsm_behaviour_version"],
        "event_source": m["fsm_runtime"]["event_source"],
        "valid_cycles": m["actor_fsm_events_summary"]["phase_valid_cycle_count"],
        "valid_hs": m["actor_fsm_events_summary"]["phase_valid_hs_count"],
        "valid_to": m["actor_fsm_events_summary"]["phase_valid_to_count"],
        "invalid_events": m["actor_fsm_events_summary"]["invalid_event_count"],
        "resync_count": m["fsm_counters"]["resync_count"],
        "hs_cancelled_count": m["fsm_counters"]["hs_cancelled_count"],
        "total_cancelled_transition_count": m["causal_ledger"]["total_cancelled_transition_count"],
        "settled_segments": m["morphology_summary"]["morphology_settled_segments"],
        "discarded_samples": m["morphology_summary"]["morphology_discarded_samples"],
        "penetration_max_mm": 1000.0 * float(m["penetration"]["series_max_m"]),
        "reserve_max_nm": m["reserve"]["series_norm_max_nm"],
        "knee_rom_rad": m["kinematics_actual"]["knee"]["rom"],
        "knee_min_rad": m["kinematics_actual"]["knee"]["min"],
        "knee_max_rad": m["kinematics_actual"]["knee"]["max"],
        "ankle_rom_rad": m["kinematics_actual"]["ankle"]["rom"],
        "ankle_negative_fraction": m["kinematics_actual"]["ankle"]["negative_fraction"],
        "ankle_zero_crossings": m["kinematics_actual"]["ankle"]["zero_crossings"],
        "knee_turning_points": m["kinematics_actual"]["knee"]["turning_points"],
        "served_knee_rom_rad": m["served_reference"]["knee"]["rom"],
        "served_ankle_rom_rad": m["served_reference"]["ankle"]["rom"],
        "tracking_knee_rmse_rad": m["tracking_served_vs_actual"]["knee"]["rmse"],
        "tracking_ankle_rmse_rad": m["tracking_served_vs_actual"]["ankle"]["rmse"],
        "knee_tau_spring_abs_max_nm": m["sea_torques"]["knee"]["tau_spring"]["abs_max"],
        "ankle_tau_spring_abs_max_nm": m["sea_torques"]["ankle"]["tau_spring"]["abs_max"],
        "ankle_tau_spring_sign_changes": m["sea_torques"]["ankle"]["tau_spring"]["sign_changes"],
        "knee_work_net_j": m["sea_torques"]["knee"]["tau_spring"]["work_net_j"],
        "ankle_work_net_j": m["sea_torques"]["ankle"]["tau_spring"]["work_net_j"],
        "action_raw_abs_max": m["actions"]["raw_abs_max"],
        "action_clipped_steps": m["actions"]["clipped_steps_summary"],
    }


NUMERIC_DELTA_KEYS = ("episode_return", "steps", "valid_cycles", "invalid_events", "penetration_max_mm", "reserve_max_nm", "knee_rom_rad", "ankle_rom_rad", "ankle_negative_fraction", "settled_segments", "discarded_samples", "served_knee_rom_rad", "served_ankle_rom_rad", "tracking_knee_rmse_rad", "tracking_ankle_rmse_rad", "knee_tau_spring_abs_max_nm", "ankle_tau_spring_abs_max_nm")


def build_comparisons(jobs: list[dict[str, Any]]) -> dict[str, Any]:
    analysed = [j for j in jobs if j.get("verdict") == "PASS_ANALYSED"]
    excluded = [{"job_id": j["job_id"], "verdict": j.get("verdict")} for j in jobs if j.get("executed") and j.get("verdict") != "PASS_ANALYSED"]
    out: dict[str, Any] = {"excluded_from_comparisons": excluded, "isometry_rule": "delta rispetto a B0820_H0 solo per job isometric_comparable (runtime v3 pinnato, 35D, catena B0820); controlli storici, 39D e replay riportati senza delta e mai dichiarati isometrici", "deterministic": {}, "stochastic": {}}
    det_like = [j for j in analysed if j["action_selection"] == "deterministic"]
    for start in sorted({j["start"] for j in det_like}):
        group = [j for j in det_like if j["start"] == start]
        ref = next((j for j in group if j["candidate"] == REFERENCE_CANDIDATE and j["family"] == "det" and j["repeat"] == 1), None)
        ref_pick = _pick(ref["metrics"]) if ref else None
        rows = []
        for j in sorted(group, key=lambda x: (x["family"], x["candidate"], x["repeat"])):
            pick = _pick(j["metrics"])
            row = {"job_id": j["job_id"], "family": j["family"], "candidate": j["candidate"], "runtime": j["runtime"], "comparison_class": j["comparison_class"], "isometric_vs_reference": bool(j["isometric_comparable"] and ref is not None and j["runtime"] == ref["runtime"]), "note": j["comparison_note"], "metrics": pick}
            if row["isometric_vs_reference"] and ref_pick is not None:
                row["delta_vs_B0820_H0"] = {k: ((pick[k] - ref_pick[k]) if (isinstance(pick[k], int) and isinstance(ref_pick[k], int) and not isinstance(pick[k], bool) and not isinstance(ref_pick[k], bool)) else float(pick[k]) - float(ref_pick[k])) if is_finite_float(pick.get(k)) and is_finite_float(ref_pick.get(k)) else None for k in NUMERIC_DELTA_KEYS}
            rows.append(row)
        out["deterministic"][start] = {"reference_candidate": REFERENCE_CANDIDATE if ref else None, "reference_job_id": ref["job_id"] if ref else None, "rows": rows}
    # Stochastic aggregates: ONLY the contemporaneous family "stoch" (development seeds under the pinned v3
    # runtime). A series is (start, candidate) and must be homogeneous in family/runtime/comparison_class with
    # unique, non-sealed seeds; anything else fails closed (ComparisonError). Stochastic jobs of other families
    # (e.g. the replay JUL_H0 +0.20 seed 123 under july_legacy) are reported individually, never fused.
    out["stochastic_rule"] = "aggregati solo per family=stoch (serie contemporanee sotto runtime v3 pinnato), chiave (start, candidato), omogeneita' obbligatoria di family/runtime/comparison_class e seed univoci non sigillati (altrimenti errore); i job stocastici di altre famiglie sono riportati singolarmente in stochastic_controls_individual senza fusione"
    stoch_all = [j for j in analysed if j["action_selection"] == "stochastic"]
    contemporaneous = [j for j in stoch_all if j["family"] == "stoch"]
    out["stochastic_controls_individual"] = {}
    for j in sorted([j for j in stoch_all if j["family"] != "stoch"], key=lambda x: (x["start"], x["job_id"])):
        out["stochastic_controls_individual"].setdefault(j["start"], []).append({"job_id": j["job_id"], "family": j["family"], "candidate": j["candidate"], "runtime": j["runtime"], "comparison_class": j["comparison_class"], "seed": j["seed"], "note": j["comparison_note"], "aggregated": False, "metrics": _pick(j["metrics"])})
    for start in sorted({j["start"] for j in contemporaneous}):
        out["stochastic"][start] = {}
        for cand in sorted({j["candidate"] for j in contemporaneous if j["start"] == start}):
            group = sorted([j for j in contemporaneous if j["start"] == start and j["candidate"] == cand], key=lambda x: x["seed"])
            families = sorted({j["family"] for j in group})
            runtimes = sorted({j["runtime"] for j in group})
            classes = sorted({j["comparison_class"] for j in group})
            seeds = [j["seed"] for j in group]
            if families != ["stoch"] or len(runtimes) != 1 or len(classes) != 1:
                raise ComparisonError(f"stochastic series {cand}@{start} is heterogeneous: families={families} runtimes={runtimes} classes={classes} jobs={[j['job_id'] for j in group]}")
            if len(set(seeds)) != len(seeds):
                raise ComparisonError(f"stochastic series {cand}@{start} has duplicate seeds {seeds}: jobs={[j['job_id'] for j in group]}")
            if any(s in C.SEALED_SEEDS for s in seeds):
                raise ComparisonError(f"stochastic series {cand}@{start} uses a sealed seed: {seeds}")
            picks = [_pick(j["metrics"]) for j in group]
            returns = [p["episode_return"] for p in picks]
            cycles = [p["valid_cycles"] for p in picks]
            out["stochastic"][start][cand] = {
                "series_id": f"{cand}__{runtimes[0]}__{start}__stoch",
                "family": "stoch",
                "runtime": runtimes[0],
                "comparison_class": classes[0],
                "note": group[0]["comparison_note"],
                "fsm_versions": sorted({str(p["fsm_version"]) for p in picks}),
                "seeds": seeds,
                "job_ids": [j["job_id"] for j in group],
                "n": len(group),
                "horizon_completed_fraction": float(np.mean([p["horizon_completed"] for p in picks])),
                "return_mean_min_max": [float(np.mean(returns)), float(np.min(returns)), float(np.max(returns))],
                "cycles_mean_min_max": [float(np.mean(cycles)), int(np.min(cycles)), int(np.max(cycles))],
                "steps_mean": float(np.mean([p["steps"] for p in picks])),
                "penetration_max_mm_mean_max": [float(np.mean([p["penetration_max_mm"] for p in picks])), float(np.max([p["penetration_max_mm"] for p in picks]))],
                "reserve_max_nm_mean_max": [float(np.mean([p["reserve_max_nm"] for p in picks])), float(np.max([p["reserve_max_nm"] for p in picks]))],
                "knee_rom_rad_mean": float(np.mean([p["knee_rom_rad"] for p in picks])),
                "ankle_rom_rad_mean": float(np.mean([p["ankle_rom_rad"] for p in picks])),
                "per_seed": [{"seed": j["seed"], **p} for j, p in zip(group, picks)],
            }
    return out


# --- orchestration -------------------------------------------------------------------


def analyse_matrix(described: list[dict[str, Any]], *, verify: Callable[[dict[str, Any], Path], Any] = M.verify_existing, preflight: dict[str, Any] | None = None, rollout_eval_path: Path | None = None) -> dict[str, Any]:
    jobs = [analyse_job(rec, verify=verify, rollout_eval_path=rollout_eval_path) for rec in described]
    verdicts: dict[str, int] = {}
    for j in jobs:
        verdicts[j["verdict"]] = verdicts.get(j["verdict"], 0) + 1
    executed = [j for j in jobs if j.get("executed")]
    perimeter_complete = bool(jobs) and all(j.get("verdict") == "PASS_ANALYSED" for j in jobs)
    return {
        "schema_version": 4,
        "revision": C.F0_REV,
        "generated_at_utc": C.utc_now(),
        "git": C.git_snapshot(),
        "preflight": preflight,
        "expected_evidence": list(EXPECTED_EVIDENCE),
        "expected_series": {k: list(v) for k, v in EXPECTED_STO.items()},
        "series_semantics": STO_SEMANTICS,
        "summary_schema": SUMMARY_SCHEMA,
        "trace_required_keys": list(TRACE_REQUIRED_KEYS),
        "horizon_semantics": "progress_upper_bound = min(max_steps, ceil(ed/sd)) e' solo il limite della barra di avanzamento; env_horizon_steps_expected = ceil(ed/sd - 0.5) deriva dalla terminazione dell'env (t >= episode_end - 0.5*sd); expected_recorded_steps = min(loop cap, env horizon); horizon_completed solo con end_reason episode_time_limit e step == env horizon; dataset_end non e' equiparato all'orizzonte",
        "fsm_counter_policy": f"valid_*/invalid_event_count sempre richiesti (interi non negativi finiti); resync_count/hs_cancelled_count richiesti solo su FSM v3 con event_source esattamente '{EVENT_SOURCE_BINARY_ACTIVE}', altrimenti riportati con present=false se assenti; mai assente -> zero; versione/sorgente FSM costanti lungo la trace, versione coerente con CLI/config e sorgente ESATTAMENTE uguale all'attesa ('{EVENT_SOURCE_BINARY_ACTIVE}' se binary_phase_fsm_mode == '{BINARY_ACTIVE_MODE}', altrimenti phase_fsm_input_mode; nessun prefisso); contatori cumulativi non decrescenti, con l'unica eccezione del contratto di cancellazione HS della FSM v3 (prosthetic_phase_fsm.py 1808-1840), applicato SOLO se fsm_behaviour_version e' costante ed esattamente 'v3' e hs_cancelled_count e' un intero su ogni riga: valid_hs_count e' il numero di heel strike ATTUALMENTE validi e puo' calare solo di esattamente 1 nello stesso step in cui hs_cancelled_count cresce di 1 e invalid_event_count cresce di 1, con valid_hs_count + hs_cancelled_count (heel strike accettati) mai decrescente; ogni altro calo fallisce; v2/legacy restano strettamente monotoni anche se portano hs_cancelled_count; tutta l'aritmetica dei contatori (differenze, somme, massimi, confronti) e' intera esatta, mai in virgola mobile; contatori del summary uguali al MASSIMO corrente della trace (semantica di rollout_eval.py 736-745), non all'ultimo step; ledger causale obbligatorio a ogni step in modalita' morphology causale, altrimenti continuo dal primo payload attivo, con event_contract_id esattamente uguale a reward.morphology_causal_event_contract_id e costante",
        "abi_policy": f"summary: n_actor/n_observation interi positivi, action_shape esattamente {ACTION_SHAPE_EXPECTED}, actor_feature_names/observation_feature_names di lunghezza n_actor/n_observation con nomi non vuoti e unici, actor_feature_names == observation_feature_names[:n_actor] in ordine esatto, actor_feature_names esattamente uguale al manifest content-addressed del candidato (SHA pinnato nel registro; 35D: manifest markov35 luglio, 39D: manifest V26), nessuna diagnostica {list(C.CONTROLLER_DIAGNOSTIC_FEATURES)} nel blocco actor, n_actor == larghezza di registro del candidato, n_observation == larghezza completa attesa del runtime (84 / 88), include_controller_diagnostic_observation risolto da CLI>config>default uguale al registro del runtime (False); trace: step = sequenza esatta 1..steps, time finito strettamente crescente, actor_observation_vector_before di larghezza n_actor e azioni raw/applied di larghezza {ACTION_WIDTH} a ogni riga",
        "contract_policy": "summary.binary_phase_fsm_mode, phase_fsm_input_mode, event_contract_id e binary_phase_event_contract_id presenti ed esattamente uguali a config pinnata/CLI (default argparse di rollout_eval.py se assenti dalla config)",
        "counters_note": "ogni contatore FSM/ledger riporta {present, value}: un contatore assente non e' mai riportato come zero",
        "finiteness_gate": "tutte le colonne consumate (incluse tutte le *_reserve_torque) devono essere finite; gate ricorsivo finale su ogni metrica numerica (NaN/Inf/overflow -> FAIL_METRICS); None ammesso solo per " + ", ".join(NULLABLE_METRIC_PATHS),
        "completeness_rule": "analysis_complete richiede che TUTTI i job descritti del perimetro siano eseguiti e PASS_ANALYSED; executed_subset_complete riguarda solo i job eseguiti",
        "job_count_total": len(jobs),
        "job_count_executed": len(executed),
        "verdict_counts": verdicts,
        "analysis_complete": perimeter_complete,
        "executed_subset_complete": bool(executed) and all(j.get("verdict") == "PASS_ANALYSED" for j in executed),
        "jobs": jobs,
        "not_executed": [j["job_id"] for j in jobs if not j.get("executed")],
        "comparisons": build_comparisons(jobs),
    }


def _job_row(j: dict[str, Any], f: Callable[..., str]) -> list[Any]:
    head = [j["job_id"], j["comparison_class"], j.get("integrity", "-"), j["verdict"]]
    m = j.get("metrics")
    if not m:
        return head + ["-"] * 22
    ev = m["actor_fsm_events_summary"]
    ka, sr, sea, hz = m["kinematics_actual"], m["served_reference"], m["sea_torques"], m["horizon"]
    return head + [
        f"{m['fsm_runtime']['fsm_behaviour_version']}/{m['fsm_runtime']['event_source']}",
        f"{m['steps']}/{hz['env_horizon_steps_expected']}{' (cap ' + str(hz['loop_cap']) + ')' if hz['loop_cap_below_env_horizon'] else ''}{' completato' if hz['horizon_completed'] else ''}", f(m["episode_return"]), m["end_reason"], f"{ev['phase_valid_hs_count']}/{ev['phase_valid_to_count']}/{ev['phase_valid_cycle_count']}", ev["invalid_event_count"],
        f(m["fsm_counters"]["resync_count"]), f(m["fsm_counters"]["hs_cancelled_count"]), f(m["causal_ledger"]["total_cancelled_transition_count"]),
        f"{m['morphology_summary']['morphology_settled_segments']}/{m['morphology_summary']['morphology_settled_samples']}", f"{m['morphology_summary']['morphology_discarded_segments']}/{m['morphology_summary']['morphology_discarded_samples']}",
        f(1000 * m["penetration"]["series_max_m"], 4), f(m["reserve"]["series_norm_max_nm"], 5),
        f"{f(ka['knee']['min'], 3)}/{f(ka['knee']['max'], 3)}/{f(ka['knee']['rom'], 3)}",
        f"{f(ka['ankle']['min'], 3)}/{f(ka['ankle']['max'], 3)}/{f(ka['ankle']['rom'], 3)}",
        f"{f(ka['ankle']['negative_fraction'], 3)} / {ka['ankle']['zero_crossings']}",
        f"{f(sr['knee']['rom'], 3)} / {f(sr['ankle']['rom'], 3)}",
        f"{f(m['tracking_served_vs_actual']['knee']['rmse'], 3)} / {f(m['tracking_served_vs_actual']['ankle']['rmse'], 3)}",
        f"{f(sea['knee']['tau_spring']['abs_max'], 4)} / {f(sea['ankle']['tau_spring']['abs_max'], 4)}",
        sea["ankle"]["tau_spring"]["sign_changes"],
        f"{f(sea['knee']['tau_spring']['work_net_j'], 4)} / {f(sea['ankle']['tau_spring']['work_net_j'], 4)}",
        f"{f(m['actions']['raw_abs_max'], 4)} / {m['actions']['clipped_steps_summary']}",
    ]


def render_markdown(payload: dict[str, Any], stamp: str) -> str:
    def f(v: Any, nd: int = 5) -> str:
        if isinstance(v, bool):
            return str(v)
        if isinstance(v, float):
            return f"{v:.{nd}g}"
        if isinstance(v, dict) and "present" in v:
            return f(v["value"]) if v["present"] else "assente"
        return str(v)

    lines = [
        f"# F0 — Analisi della matrice di rollout — revisione {payload['revision']} — stamp {stamp}",
        "",
        f"Generato: {payload['generated_at_utc']} — git HEAD `{payload['git']['head'][:12]}`. Job totali {payload['job_count_total']}, eseguiti {payload['job_count_executed']}, verdetti {payload['verdict_counts']}, analisi completa sul perimetro: **{payload['analysis_complete']}**, sottoinsieme eseguito completo: **{payload['executed_subset_complete']}**.",
        "",
        f"Preflight: {payload.get('preflight')}",
        "",
        payload["completeness_rule"] + ". " + payload["horizon_semantics"] + ". " + payload["counters_note"] + ". " + payload["fsm_counter_policy"] + ". " + payload["finiteness_gate"] + ". " + payload["comparisons"]["isometry_rule"] + ".",
        "",
        "Semantica delle serie: " + "; ".join(f"`{k}`: {v}" for k, v in payload["series_semantics"].items()) + ".",
        "",
        "## Job eseguiti (eventi e contatori riferiti alla FSM attore effettiva, colonna «FSM ver./sorgente»; colonna step = registrati/orizzonte env)",
        "",
        C.md_table(
            ["job", "classe", "integrita'", "verdetto", "FSM ver./sorgente", "step/orizzonte env", "return", "fine", "HS/TO/cicli", "inv", "resync", "HS canc.", "cancelled tr.", "settled seg/smp", "disc seg/smp", "penetr. max [mm]", "reserve max [Nm]", "knee min/max/ROM", "ankle min/max/ROM", "ankle neg. fraz. / zero-cross", "ROM servito knee/ankle", "tracking RMSE knee/ankle", "SEA knee/ankle |tau| max", "ankle tau sign ch.", "lavoro netto knee/ankle [J]", "|a| raw max / clip"],
            [_job_row(j, f) for j in payload["jobs"] if j.get("executed")],
        ),
        "",
    ]
    if payload["not_executed"]:
        lines += [f"Job non eseguiti ({len(payload['not_executed'])}) — il gate globale resta incompleto: " + ", ".join(payload["not_executed"]), ""]
    if payload["comparisons"]["excluded_from_comparisons"]:
        lines += ["Esclusi dai confronti (evidenza incompleta, provenienza o verifica fallita): " + ", ".join(f"{e['job_id']} ({e['verdict']})" for e in payload["comparisons"]["excluded_from_comparisons"]), ""]
    lines += ["## Confronti deterministici per start (delta solo per job isometrici rispetto a B0820_H0)", ""]
    for start, block in payload["comparisons"]["deterministic"].items():
        lines += [f"### start {start} — riferimento: {block['reference_job_id'] or 'assente'}", "", C.md_table(
            ["candidato", "famiglia", "classe", "FSM ver.", "isometrico vs H0", "step", "orizzonte completato", "return", "Δ return", "cicli", "Δ cicli", "inv", "resync", "HS canc.", "penetr. [mm]", "Δ penetr.", "reserve [Nm]", "Δ reserve", "knee ROM", "Δ knee ROM", "ankle ROM", "Δ ankle ROM", "ankle neg. fraz.", "ROM servito knee", "tracking RMSE knee", "settled seg", "disc. campioni"],
            [[
                r["candidate"], r["family"], r["comparison_class"], r["metrics"]["fsm_version"], r["isometric_vs_reference"], r["metrics"]["steps"], r["metrics"]["horizon_completed"], f(r["metrics"]["episode_return"]), f((r.get("delta_vs_B0820_H0") or {}).get("episode_return"), 4) if r.get("delta_vs_B0820_H0") else "—",
                r["metrics"]["valid_cycles"], f((r.get("delta_vs_B0820_H0") or {}).get("valid_cycles")) if r.get("delta_vs_B0820_H0") else "—", r["metrics"]["invalid_events"], f(r["metrics"]["resync_count"]), f(r["metrics"]["hs_cancelled_count"]),
                f(r["metrics"]["penetration_max_mm"], 4), f((r.get("delta_vs_B0820_H0") or {}).get("penetration_max_mm"), 3) if r.get("delta_vs_B0820_H0") else "—",
                f(r["metrics"]["reserve_max_nm"], 5), f((r.get("delta_vs_B0820_H0") or {}).get("reserve_max_nm"), 4) if r.get("delta_vs_B0820_H0") else "—",
                f(r["metrics"]["knee_rom_rad"], 3), f((r.get("delta_vs_B0820_H0") or {}).get("knee_rom_rad"), 3) if r.get("delta_vs_B0820_H0") else "—",
                f(r["metrics"]["ankle_rom_rad"], 3), f((r.get("delta_vs_B0820_H0") or {}).get("ankle_rom_rad"), 3) if r.get("delta_vs_B0820_H0") else "—",
                f(r["metrics"]["ankle_negative_fraction"], 3), f(r["metrics"]["served_knee_rom_rad"], 3), f(r["metrics"]["tracking_knee_rmse_rad"], 3), r["metrics"]["settled_segments"], r["metrics"]["discarded_samples"],
            ] for r in block["rows"]],
        ), ""]
    if payload["comparisons"]["stochastic"]:
        lines += ["## Aggregati stocastici per start e candidato (solo famiglia `stoch`, seed di sviluppo; serie omogenee per runtime/classe, seed univoci)", "", payload["comparisons"].get("stochastic_rule", ""), ""]
        for start, cands in payload["comparisons"]["stochastic"].items():
            lines += [f"### start {start}", "", C.md_table(
                ["candidato", "runtime", "classe", "FSM ver.", "seed", "n", "orizzonte completato (fraz.)", "return media/min/max", "cicli media/min/max", "step medi", "penetr. [mm] media/max", "reserve [Nm] media/max", "knee ROM media", "ankle ROM media"],
                [[cand, a["runtime"], a["comparison_class"], "/".join(a["fsm_versions"]), a["seeds"], a["n"], f(a["horizon_completed_fraction"], 3), "/".join(f(x, 5) for x in a["return_mean_min_max"]), "/".join(f(x, 4) for x in a["cycles_mean_min_max"]), f(a["steps_mean"], 4), "/".join(f(x, 4) for x in a["penetration_max_mm_mean_max"]), "/".join(f(x, 5) for x in a["reserve_max_nm_mean_max"]), f(a["knee_rom_rad_mean"], 3), f(a["ankle_rom_rad_mean"], 3)] for cand, a in cands.items()],
            ), ""]
    if payload["comparisons"].get("stochastic_controls_individual"):
        lines += ["## Controlli stocastici individuali (famiglie diverse da `stoch`: riportati singolarmente, mai fusi negli aggregati)", ""]
        for start, items in payload["comparisons"]["stochastic_controls_individual"].items():
            lines += [f"### start {start}", "", C.md_table(
                ["job", "famiglia", "candidato", "runtime", "classe", "seed", "step", "orizzonte completato", "return", "cicli", "penetr. [mm]", "reserve [Nm]", "knee ROM", "ankle ROM"],
                [[it["job_id"], it["family"], it["candidate"], it["runtime"], it["comparison_class"], it["seed"], it["metrics"]["steps"], it["metrics"]["horizon_completed"], f(it["metrics"]["episode_return"]), it["metrics"]["valid_cycles"], f(it["metrics"]["penetration_max_mm"], 4), f(it["metrics"]["reserve_max_nm"], 5), f(it["metrics"]["knee_rom_rad"], 3), f(it["metrics"]["ankle_rom_rad"], 3)] for it in items],
            ), ""]
    return "\n".join(lines)


def write_outputs(payload: dict[str, Any], stamp: str, metrics_dir: Path | None = None) -> tuple[Path, Path]:
    target = Path(metrics_dir) if metrics_dir is not None else C.OUT_METRICS
    out_json = target / f"f0_matrix_analysis_{stamp}.json"
    out_md = target / f"f0_matrix_analysis_{stamp}.md"
    for path in (out_json, out_md):
        if path.exists():
            raise FileExistsError(f"refusing to overwrite existing artefact: {path}")
    C.write_json(out_json, payload)
    C.write_text(out_md, render_markdown(payload, stamp))
    return out_json, out_md


def main(argv: list[str] | None = None) -> int:
    # minimal CLI: no options; --help exits 0 and unknown arguments exit 2 BEFORE any side effect
    # (no directory creation, no interpreter probe, no analysis, no artefact)
    parser = argparse.ArgumentParser(prog="f0_matrix_analysis.py", description="F0 matrix analyser (fail-closed, no-clobber): analyses every described job of the rollout matrix whose output exists and writes metrics/f0_matrix_analysis_<stamp>.{json,md}. No options are accepted.", formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.parse_args(argv)
    C.ensure_out_dirs()
    stamp = time.strftime("%Y%m%d_%H%M%S")
    interpreter = C.select_python()
    described = [M.describe_job(j, interpreter["selected"], None) for j in M.build_jobs()]
    preflight = M.preflight_all(described)  # fail-closed: raises if any existing output is invalid
    payload = analyse_matrix(described, preflight={"checked_existing": preflight["checked_existing"], "verified_identical": len(preflight["verified_identical"]), "invalid": preflight["invalid"]})
    payload["execute_manifests"] = [C.rel(m) for m in sorted(glob.glob(str(C.OUT_ROLLOUTS / "rollout_matrix_manifest_execute_*.json")))]
    payload["status_files"] = [C.rel(s) for s in sorted(glob.glob(str(C.OUT_ROLLOUTS / "rollout_matrix_status_*.json")))]
    out_json, out_md = write_outputs(payload, stamp)
    print(f"[matrix-analysis] written {out_json} / {out_md}; executed={payload['job_count_executed']}/{payload['job_count_total']} verdicts={payload['verdict_counts']} complete={payload['analysis_complete']} executed_subset_complete={payload['executed_subset_complete']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
