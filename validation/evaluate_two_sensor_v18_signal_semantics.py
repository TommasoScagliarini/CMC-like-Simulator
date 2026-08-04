"""Fail-closed V18 comparison of three frozen two-sensor signal semantics.

The runner consumes the immutable V18 raw traces for trials 02 and 04 and the
canonical V17 event ledgers.  It evaluates only necessary event-level gates.
If every semantic fails any necessary gate on selection trial 04, the V18
signal-semantic branch terminates without running a full FSM, opening trial 08,
or touching protected data.  A necessary-gate PASS would still require a new
hash-pinned full-FSM receipt before any finalist or holdout access.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
import sys
import traceback
import uuid
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
VALIDATION_ROOT = REPO_ROOT / "validation"
if str(VALIDATION_ROOT) not in sys.path:
    sys.path.insert(0, str(VALIDATION_ROOT))

import acquire_two_sensor_v18_raw_traces as acquisition  # noqa: E402


SCHEMA_VERSION = 18
ANALYSIS_ID = "AB06_TWO_SENSOR_V18_FROZEN_SIGNAL_SEMANTIC_SELECTION"
SEMANTICS = ("heel_only", "first_stable_regional", "combined_load")
MODES = ("sequential_1ms", "batched_10ms_same_samples")
TRIALS = ("02", "04")
SELECTION_TRIAL = "04"
CHALLENGE_TRIAL = "02"
ON_THRESHOLD_N = 0.5
OFF_THRESHOLD_N = 0.25
DWELL_S = 0.03
DWELL_SAMPLES = 30
POLICY_SAMPLES = 10
HS_TOLERANCE_S = 0.05
TO_TOLERANCE_S = 0.08
DELIVERED_HS_TOLERANCE_S = 0.06
DELIVERED_TO_TOLERANCE_S = 0.09
RIGHT_OBSERVATION_MARGIN_S = 0.06
MAX_DIAGNOSTIC_ASSOCIATION_S = 0.25
DEFAULT_CONTRACT = VALIDATION_ROOT / "two_sensor_v18_selection_analysis_contract.json"
TRACE_ROOT = (
    VALIDATION_ROOT
    / "two_sensor_v18_signal_semantics_runs/"
    "2026-08-03_ab06_dev02_04_v18_selection_traces"
)
DEFAULT_OUTPUT = (
    VALIDATION_ROOT
    / "two_sensor_v18_signal_semantics_runs/"
    "2026-08-03_ab06_dev02_04_v18_semantic_selection"
)


class V18SelectionError(RuntimeError):
    """Raised when V18 selection evidence cannot support a safe decision."""


@dataclass(frozen=True)
class Edge:
    signal: str
    active: bool
    event_time_s: float
    confirmed_time_s: float
    onset_index: int
    confirmed_index: int

    def payload(self) -> dict[str, Any]:
        return {
            "signal": self.signal,
            "edge": "contact_on" if self.active else "contact_off",
            "event_time_s": self.event_time_s,
            "confirmed_time_s": self.confirmed_time_s,
            "onset_index": self.onset_index,
            "confirmed_index": self.confirmed_index,
        }


@dataclass(frozen=True)
class SemanticEvent:
    semantic: str
    event: str
    source_sensor: str
    event_time_s: float
    confirmed_time_s: float
    delivered_time_s: float
    onset_index: int
    confirmed_index: int
    delivered_index: int

    def payload(self) -> dict[str, Any]:
        return {
            "semantic": self.semantic,
            "event": self.event,
            "source_sensor": self.source_sensor,
            "event_time_s": self.event_time_s,
            "confirmed_time_s": self.confirmed_time_s,
            "delivered_time_s": self.delivered_time_s,
            "onset_index": self.onset_index,
            "confirmed_index": self.confirmed_index,
            "delivered_index": self.delivered_index,
        }


class DebouncedLatch:
    def __init__(self, signal: str) -> None:
        self.signal = signal
        self.active = False
        self.pending_target: bool | None = None
        self.pending_index: int | None = None

    def prime(self, force_n: float) -> None:
        self.active = bool(force_n >= ON_THRESHOLD_N)
        self.pending_target = None
        self.pending_index = None

    def update(self, index: int, time_s: float, force_n: float, times: np.ndarray) -> Edge | None:
        requested: bool | None = None
        if self.active and force_n <= OFF_THRESHOLD_N:
            requested = False
        elif not self.active and force_n >= ON_THRESHOLD_N:
            requested = True
        if requested is None:
            self.pending_target = None
            self.pending_index = None
            return None
        if self.pending_target != requested:
            self.pending_target = requested
            self.pending_index = int(index)
        if self.pending_index is None or index - self.pending_index < DWELL_SAMPLES:
            return None
        onset_index = int(self.pending_index)
        self.active = bool(requested)
        self.pending_target = None
        self.pending_index = None
        return Edge(
            signal=self.signal,
            active=bool(requested),
            event_time_s=float(times[onset_index]),
            confirmed_time_s=float(time_s),
            onset_index=onset_index,
            confirmed_index=int(index),
        )


class SemanticEngine:
    def __init__(self, semantic: str, times: np.ndarray) -> None:
        if semantic not in SEMANTICS:
            raise V18SelectionError(f"unsupported V18 semantic: {semantic}")
        self.semantic = semantic
        self.times = times
        self.heel = DebouncedLatch("heel")
        self.toe = DebouncedLatch("toe")
        self.combined = DebouncedLatch("combined")
        self.armed = False
        self.stance = False
        self.clear_since_index: int | None = None
        self.events: list[SemanticEvent] = []
        self.edges: list[Edge] = []
        self._state_digest = hashlib.sha256()

    def prime(self, heel_n: float, toe_n: float) -> None:
        self.heel.prime(heel_n)
        self.toe.prime(toe_n)
        self.combined.prime(heel_n + toe_n)
        raw_clear = (
            heel_n <= OFF_THRESHOLD_N and toe_n <= OFF_THRESHOLD_N
            if self.semantic != "combined_load"
            else heel_n + toe_n <= OFF_THRESHOLD_N
        )
        self.clear_since_index = 0 if raw_clear else None
        self.armed = False
        self.stance = False
        self._update_state_digest(0)

    def _delivery(self, confirmed_index: int) -> tuple[int, float]:
        delivered_index = int(
            math.ceil(confirmed_index / POLICY_SAMPLES) * POLICY_SAMPLES
        )
        delivered_time = float(self.times[0] + delivered_index * acquisition.SAMPLE_DT_S)
        return delivered_index, delivered_time

    def _emit(self, event: str, source: str, edge: Edge) -> None:
        delivered_index, delivered_time = self._delivery(edge.confirmed_index)
        self.events.append(
            SemanticEvent(
                semantic=self.semantic,
                event=event,
                source_sensor=source,
                event_time_s=edge.event_time_s,
                confirmed_time_s=edge.confirmed_time_s,
                delivered_time_s=delivered_time,
                onset_index=edge.onset_index,
                confirmed_index=edge.confirmed_index,
                delivered_index=delivered_index,
            )
        )

    def _update_state_digest(self, index: int) -> None:
        state = (
            index,
            int(self.heel.active),
            int(self.toe.active),
            int(self.combined.active),
            int(self.armed),
            int(self.stance),
            -1 if self.clear_since_index is None else self.clear_since_index,
        )
        self._state_digest.update((",".join(map(str, state)) + "\n").encode("ascii"))

    def consume(self, index: int, heel_n: float, toe_n: float) -> None:
        time_s = float(self.times[index])
        if self.semantic == "combined_load":
            edge = self.combined.update(
                index, time_s, float(heel_n + toe_n), self.times
            )
            if edge is not None:
                self.edges.append(edge)
            raw_clear = heel_n + toe_n <= OFF_THRESHOLD_N
            if raw_clear:
                if self.clear_since_index is None:
                    self.clear_since_index = index
            else:
                self.clear_since_index = None
            if (
                self.clear_since_index is not None
                and index - self.clear_since_index >= DWELL_SAMPLES
                and not self.combined.active
            ):
                self.armed = True
            if edge is not None and not edge.active:
                if self.stance:
                    self._emit("toe_off", "combined", edge)
                    self.stance = False
                self.armed = True
            elif edge is not None and edge.active and self.armed and not self.stance:
                self._emit("heel_strike", "combined", edge)
                self.stance = True
                self.armed = False
            self._update_state_digest(index)
            return

        heel_edge = self.heel.update(index, time_s, float(heel_n), self.times)
        toe_edge = self.toe.update(index, time_s, float(toe_n), self.times)
        new_edges = [edge for edge in (heel_edge, toe_edge) if edge is not None]
        self.edges.extend(new_edges)
        raw_clear = heel_n <= OFF_THRESHOLD_N and toe_n <= OFF_THRESHOLD_N
        if raw_clear:
            if self.clear_since_index is None:
                self.clear_since_index = index
        else:
            self.clear_since_index = None
        if (
            self.clear_since_index is not None
            and index - self.clear_since_index >= DWELL_SAMPLES
            and not self.heel.active
            and not self.toe.active
        ):
            self.armed = True

        falling = [edge for edge in new_edges if not edge.active]
        if falling and not self.heel.active and not self.toe.active:
            if self.stance:
                edge = max(falling, key=lambda item: (item.event_time_s, item.signal))
                self._emit("toe_off", "both_regional_off", edge)
                self.stance = False
            self.armed = True

        rising = [edge for edge in new_edges if edge.active]
        eligible: list[Edge]
        if self.semantic == "heel_only":
            eligible = [edge for edge in rising if edge.signal == "heel"]
        else:
            eligible = rising
        if eligible and self.armed and not self.stance:
            edge = min(
                eligible,
                key=lambda item: (
                    item.event_time_s,
                    0 if item.signal == "heel" else 1,
                ),
            )
            self._emit("heel_strike", edge.signal, edge)
            self.stance = True
            self.armed = False
        self._update_state_digest(index)

    @property
    def state_sha256(self) -> str:
        return self._state_digest.hexdigest()


def _strict_json_object(path: Path, *, label: str) -> dict[str, Any]:
    return acquisition.load_json_object(path, label=label)


def load_and_validate_contract(path: Path = DEFAULT_CONTRACT) -> dict[str, Any]:
    if path.resolve() != DEFAULT_CONTRACT.resolve():
        raise V18SelectionError("V18 selection accepts only its canonical contract")
    raw = _strict_json_object(path, label="V18 selection contract")
    checks = {
        "schema": raw.get("schema_version") == SCHEMA_VERSION,
        "id": raw.get("analysis_id") == ANALYSIS_ID,
        "frozen": raw.get("frozen_before_execution") is True,
        "trials": raw.get("trials") == list(TRIALS),
        "selection": raw.get("selection_trial") == SELECTION_TRIAL,
        "challenge": raw.get("challenge_trial") == CHALLENGE_TRIAL,
        "holdout_closed": raw.get("internal_holdout", {}).get("opened") is False,
        "semantics": raw.get("semantics") == list(SEMANTICS),
        "modes": raw.get("consumption_modes") == list(MODES),
        "on": raw.get("detector", {}).get("on_n") == ON_THRESHOLD_N,
        "off": raw.get("detector", {}).get("off_n") == OFF_THRESHOLD_N,
        "dwell": raw.get("detector", {}).get("dwell_s") == DWELL_S,
    }
    if not all(checks.values()):
        raise V18SelectionError(f"V18 selection contract drifted: {checks}")
    sources = raw.get("sources")
    expected = {
        "main_protocol",
        "analysis_runner",
        "analysis_tests",
        "raw_manifest",
        "raw_trial_02",
        "raw_trial_04",
        "canonical_trial_02",
        "canonical_trial_04",
        "v17_profile",
        "v17_failure_receipt",
    }
    if not isinstance(sources, Mapping) or set(sources) != expected:
        raise V18SelectionError("V18 selection source keyset drifted")
    paths: dict[str, Path] = {}
    for label, record in sources.items():
        try:
            paths[label] = acquisition._require_source(record, label=f"sources.{label}")
        except acquisition.V18AcquisitionError as exc:
            raise V18SelectionError(str(exc)) from exc
    raw["_contract_sha256"] = acquisition.sha256_file(path)
    raw["_source_paths"] = paths
    return raw


def load_trace(path: Path, trial_id: str) -> dict[str, np.ndarray]:
    try:
        with path.open("r", encoding="utf-8", newline="") as stream:
            reader = csv.reader(stream)
            header = next(reader)
    except (OSError, UnicodeError, csv.Error, StopIteration) as exc:
        raise V18SelectionError(f"cannot read V18 trace header: {path}") from exc
    if tuple(header) != acquisition.TRACE_COLUMNS:
        raise V18SelectionError(f"V18 trace columns drifted: {path}")
    try:
        matrix = np.loadtxt(path, delimiter=",", skiprows=1, dtype=float)
    except (OSError, ValueError) as exc:
        raise V18SelectionError(f"cannot parse V18 trace: {path}") from exc
    if matrix.ndim != 2 or matrix.shape[1] != len(acquisition.TRACE_COLUMNS):
        raise V18SelectionError(f"V18 trace shape drifted: {path}")
    arrays = {
        name: np.asarray(matrix[:, index], dtype=float)
        for index, name in enumerate(acquisition.TRACE_COLUMNS)
    }
    acquisition.validate_trace_arrays(
        trial_id=trial_id,
        times=arrays["time_s"],
        heel_force_n=arrays["left_heel_normal_n"],
        toe_force_n=arrays["left_toe_normal_n"],
        heel_penetration_m=arrays["left_heel_penetration_m"],
        toe_penetration_m=arrays["left_toe_penetration_m"],
    )
    return arrays


def process_trace(
    semantic: str,
    mode: str,
    times: np.ndarray,
    heel_n: np.ndarray,
    toe_n: np.ndarray,
) -> dict[str, Any]:
    if mode not in MODES:
        raise V18SelectionError(f"unsupported consumption mode: {mode}")
    engine = SemanticEngine(semantic, times)
    engine.prime(float(heel_n[0]), float(toe_n[0]))
    if mode == "sequential_1ms":
        groups: Iterable[range] = (range(index, index + 1) for index in range(1, len(times)))
    else:
        groups = (
            range(start, min(start + POLICY_SAMPLES, len(times)))
            for start in range(1, len(times), POLICY_SAMPLES)
        )
    for group in groups:
        for index in group:
            engine.consume(index, float(heel_n[index]), float(toe_n[index]))
    return {
        "events": [event.payload() for event in engine.events],
        "edges": [edge.payload() for edge in engine.edges],
        "state_sha256": engine.state_sha256,
    }


def _canonical_sha256(value: Any) -> str:
    payload = json.dumps(
        value, sort_keys=True, separators=(",", ":"), allow_nan=False
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def _ordered_match_count(reference: Sequence[float], predicted: Sequence[float], tolerance: float) -> int:
    ref = [float(value) for value in reference]
    pred = [float(value) for value in predicted]
    i = 0
    j = 0
    matches = 0
    while i < len(ref) and j < len(pred):
        if pred[j] < ref[i] - tolerance:
            j += 1
        elif pred[j] > ref[i] + tolerance:
            i += 1
        else:
            matches += 1
            i += 1
            j += 1
    return matches


def _owned_events(
    events: Sequence[Mapping[str, Any]],
    reference: Mapping[str, Any],
    event_name: str,
) -> list[dict[str, Any]]:
    refs = [
        float(item["event_time_s"])
        for item in reference["scoreable_events"]
        if item["event"] == event_name
    ]
    start_s, end_s = (float(value) for value in reference["interval_s"])
    tolerance = HS_TOLERANCE_S if event_name == "heel_strike" else TO_TOLERANCE_S
    closing_hs = max(
        float(item["event_time_s"])
        for item in reference["scoreable_events"]
        if item["event"] == "heel_strike"
    )
    confirmation_limit = closing_hs + RIGHT_OBSERVATION_MARGIN_S
    primary_start = max(start_s, refs[0] - tolerance)
    primary_end = min(refs[-1] + tolerance, confirmation_limit, end_s - 1e-12)
    return [
        dict(item)
        for item in events
        if item["event"] == event_name
        and float(item["event_time_s"]) >= start_s - 1e-12
        and primary_start - 1e-12
        <= float(item["confirmed_time_s"])
        <= primary_end + 1e-12
    ]


def score_view(
    *,
    trial_id: str,
    semantic: str,
    mode: str,
    view: Mapping[str, Any],
    events: Sequence[Mapping[str, Any]],
) -> dict[str, Any]:
    refs = {
        name: [
            float(item["event_time_s"])
            for item in view["scoreable_events"]
            if item["event"] == name
        ]
        for name in ("heel_strike", "toe_off")
    }
    predicted = {
        name: _owned_events(events, view, name)
        for name in ("heel_strike", "toe_off")
    }
    metrics: dict[str, Any] = {}
    necessary = True
    for name, tolerance, delivered_tolerance in (
        ("heel_strike", HS_TOLERANCE_S, DELIVERED_HS_TOLERANCE_S),
        ("toe_off", TO_TOLERANCE_S, DELIVERED_TO_TOLERANCE_S),
    ):
        reference_times = refs[name]
        items = predicted[name]
        confirmed = [float(item["confirmed_time_s"]) for item in items]
        delivered = [float(item["delivered_time_s"]) for item in items]
        matched = _ordered_match_count(reference_times, confirmed, tolerance)
        exact_count = len(items) == len(reference_times)
        ordered_errors = (
            [value - expected for value, expected in zip(confirmed, reference_times)]
            if exact_count
            else []
        )
        delivered_errors = (
            [value - expected for value, expected in zip(delivered, reference_times)]
            if exact_count
            else []
        )
        delivery_delays = [
            float(item["delivered_time_s"]) - float(item["confirmed_time_s"])
            for item in items
        ]
        event_pass = bool(
            exact_count
            and matched == len(reference_times)
            and ordered_errors
            and max(abs(value) for value in ordered_errors) <= tolerance + 1e-12
            and max(abs(value) for value in delivered_errors)
            <= delivered_tolerance + 1e-12
            and all(-1e-12 <= value <= 0.010 + 1e-12 for value in delivery_delays)
        )
        necessary = necessary and event_pass
        metrics[name] = {
            "reference_count": len(reference_times),
            "predicted_count": len(items),
            "exact_count": exact_count,
            "matched_within_confirmed_tolerance": matched,
            "precision": matched / max(1, len(items)),
            "recall": matched / max(1, len(reference_times)),
            "ordered_confirmed_errors_s": ordered_errors,
            "ordered_delivered_errors_s": delivered_errors,
            "maximum_absolute_confirmed_error_s": (
                max(abs(value) for value in ordered_errors) if ordered_errors else None
            ),
            "maximum_absolute_delivered_error_s": (
                max(abs(value) for value in delivered_errors) if delivered_errors else None
            ),
            "delivery_delay_min_s": min(delivery_delays) if delivery_delays else None,
            "delivery_delay_max_s": max(delivery_delays) if delivery_delays else None,
            "necessary_timing_count_delivery_pass": event_pass,
        }

    merged = sorted(
        [*predicted["heel_strike"], *predicted["toe_off"]],
        key=lambda item: (float(item["confirmed_time_s"]), item["event"]),
    )
    expected_order = [item["event"] for item in view["scoreable_events"]]
    observed_order = [item["event"] for item in merged]
    exact_order = observed_order == expected_order
    complete_cycles = sum(
        observed_order[index : index + 3]
        == ["heel_strike", "toe_off", "heel_strike"]
        for index in range(max(0, len(observed_order) - 2))
    )
    expected_cycles = int(view["counts"]["complete_cycles"])
    toe_clearances: list[float] = []
    for index, item in enumerate(merged):
        if item["event"] != "toe_off":
            continue
        following = next(
            (
                future
                for future in merged[index + 1 :]
                if future["event"] == "heel_strike"
            ),
            None,
        )
        if following is not None:
            toe_clearances.append(
                float(following["event_time_s"])
                - float(item["confirmed_time_s"])
            )
    toe_clear_pass = bool(
        len(toe_clearances) >= expected_cycles
        and min(toe_clearances) >= 0.03 - 1e-12
    )
    necessary = bool(
        necessary
        and exact_order
        and complete_cycles == expected_cycles
        and toe_clear_pass
    )
    return {
        "trial_id": trial_id,
        "view_id": view["view_id"],
        "speed_mps": float(view["speed_mps"]),
        "semantic": semantic,
        "consumption_mode": mode,
        "events": metrics,
        "exact_global_event_order": exact_order,
        "observed_complete_cycle_count": complete_cycles,
        "expected_complete_cycle_count": expected_cycles,
        "minimum_toe_clear_before_next_hs_s": (
            min(toe_clearances) if toe_clearances else None
        ),
        "minimum_toe_clear_pass": toe_clear_pass,
        "necessary_event_gate_pass": necessary,
        "full_fsm_gate_status": "NOT_EXECUTED_UNLESS_NECESSARY_EVENT_GATE_PASSES",
    }


def _nearest(items: Sequence[Mapping[str, Any]], reference_s: float) -> Mapping[str, Any] | None:
    if not items:
        return None
    result = min(items, key=lambda item: abs(float(item["confirmed_time_s"]) - reference_s))
    if abs(float(result["confirmed_time_s"]) - reference_s) > MAX_DIAGNOSTIC_ASSOCIATION_S:
        return None
    return result


def _diagnostic_rows(
    trial_id: str,
    ledger: Mapping[str, Any],
    by_semantic: Mapping[str, Mapping[str, Any]],
) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    regional_edges = by_semantic["heel_only"]["edges"]
    combined_edges = by_semantic["combined_load"]["edges"]
    stable_on = {
        "heel": [item for item in regional_edges if item["signal"] == "heel" and item["edge"] == "contact_on"],
        "toe": [item for item in regional_edges if item["signal"] == "toe" and item["edge"] == "contact_on"],
        "combined": [item for item in combined_edges if item["edge"] == "contact_on"],
    }
    for view in ledger["scientific_core"]["views"]:
        for oracle in view["scoreable_events"]:
            if oracle["event"] != "heel_strike":
                continue
            reference_s = float(oracle["event_time_s"])
            row: dict[str, Any] = {
                "trial_id": trial_id,
                "view_id": view["view_id"],
                "speed_mps": float(view["speed_mps"]),
                "oracle_event_id": oracle["event_id"],
                "oracle_event_time_s": reference_s,
            }
            for signal, items in stable_on.items():
                edge = _nearest(items, reference_s)
                row[f"{signal}_stable_onset_s"] = None if edge is None else float(edge["event_time_s"])
                row[f"{signal}_stable_confirmed_s"] = None if edge is None else float(edge["confirmed_time_s"])
                row[f"{signal}_confirmed_error_s"] = None if edge is None else float(edge["confirmed_time_s"]) - reference_s
            for semantic in SEMANTICS:
                hs_items = [
                    item
                    for item in by_semantic[semantic]["events"]
                    if item["event"] == "heel_strike"
                ]
                event = _nearest(hs_items, reference_s)
                prefix = semantic
                row[f"{prefix}_source_sensor"] = None if event is None else event["source_sensor"]
                row[f"{prefix}_event_time_s"] = None if event is None else float(event["event_time_s"])
                row[f"{prefix}_confirmed_time_s"] = None if event is None else float(event["confirmed_time_s"])
                row[f"{prefix}_delivered_time_s"] = None if event is None else float(event["delivered_time_s"])
                error = None if event is None else float(event["confirmed_time_s"]) - reference_s
                row[f"{prefix}_confirmed_error_s"] = error
                row[f"{prefix}_classification"] = (
                    "missing"
                    if error is None
                    else "early"
                    if error < -HS_TOLERANCE_S
                    else "late"
                    if error > HS_TOLERANCE_S
                    else "within_tolerance"
                )
            rows.append(row)
    return rows


def _atomic_json(path: Path, payload: Mapping[str, Any]) -> Path:
    if path.exists():
        raise V18SelectionError(f"refusing to overwrite {path}")
    temporary = path.with_name(f".{path.name}.{uuid.uuid4().hex}.tmp")
    try:
        with temporary.open("x", encoding="utf-8", newline="\n") as stream:
            json.dump(dict(payload), stream, indent=2, sort_keys=True, allow_nan=False)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if temporary.exists():
            temporary.unlink()
    return path


def _atomic_jsonl(path: Path, rows: Sequence[Mapping[str, Any]]) -> Path:
    if path.exists():
        raise V18SelectionError(f"refusing to overwrite {path}")
    temporary = path.with_name(f".{path.name}.{uuid.uuid4().hex}.tmp")
    try:
        with temporary.open("x", encoding="utf-8", newline="\n") as stream:
            for row in rows:
                stream.write(json.dumps(dict(row), sort_keys=True, allow_nan=False) + "\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if temporary.exists():
            temporary.unlink()
    return path


def _artifact(path: Path) -> dict[str, Any]:
    return {
        "path": path.resolve().relative_to(REPO_ROOT).as_posix(),
        "sha256": acquisition.sha256_file(path),
        "size_bytes": int(path.stat().st_size),
    }


def preflight(contract: Mapping[str, Any]) -> dict[str, Any]:
    raw_manifest = _strict_json_object(contract["_source_paths"]["raw_manifest"], label="raw manifest")
    if raw_manifest.get("status") != "PASS_V18_SELECTION_RAW_TRACE_LOCKED":
        raise V18SelectionError("V18 raw trace gate is not PASS")
    if raw_manifest.get("internal_holdout_sampled") is not False:
        raise V18SelectionError("V18 internal holdout state drifted")
    for trial_id in TRIALS:
        record = raw_manifest.get("traces", {}).get(trial_id, {}).get("artifact", {})
        path = contract["_source_paths"][f"raw_trial_{trial_id}"]
        if record.get("sha256") != acquisition.sha256_file(path):
            raise V18SelectionError(f"raw manifest binding drift for trial {trial_id}")
    return {
        "status": "PASS_V18_SIGNAL_SELECTION_PREFLIGHT",
        "contract_sha256": contract["_contract_sha256"],
        "trials": list(TRIALS),
        "semantics": list(SEMANTICS),
        "consumption_modes": list(MODES),
        "internal_holdout_opened": False,
        "protected_trials_opened": [],
    }


def run_selection(contract: Mapping[str, Any], output_dir: Path = DEFAULT_OUTPUT) -> dict[str, Any]:
    if output_dir.resolve() != DEFAULT_OUTPUT.resolve():
        raise V18SelectionError("V18 selection output path is fixed")
    if output_dir.exists():
        raise V18SelectionError(f"V18 selection output exists: {output_dir}")
    output_dir.mkdir(parents=True, exist_ok=False)
    _atomic_json(
        output_dir / "run_start_receipt.json",
        {
            "schema_version": SCHEMA_VERSION,
            "status": "V18_SIGNAL_SELECTION_STARTED",
            "analysis_id": ANALYSIS_ID,
            "contract_sha256": contract["_contract_sha256"],
            "trials": list(TRIALS),
            "internal_holdout_opened": False,
            "protected_trials_opened": [],
            "rerun_allowed": False,
        },
    )
    try:
        all_units: list[dict[str, Any]] = []
        all_journal: list[dict[str, Any]] = []
        all_diagnostics: list[dict[str, Any]] = []
        parity_records: list[dict[str, Any]] = []
        trial_semantic_results: dict[str, dict[str, dict[str, Any]]] = {}
        for trial_id in TRIALS:
            trace = load_trace(contract["_source_paths"][f"raw_trial_{trial_id}"], trial_id)
            ledger = _strict_json_object(
                contract["_source_paths"][f"canonical_trial_{trial_id}"],
                label=f"canonical trial {trial_id}",
            )
            trial_semantic_results[trial_id] = {}
            for semantic in SEMANTICS:
                mode_results = {
                    mode: process_trace(
                        semantic,
                        mode,
                        trace["time_s"],
                        trace["left_heel_normal_n"],
                        trace["left_toe_normal_n"],
                    )
                    for mode in MODES
                }
                seq = mode_results[MODES[0]]
                batch = mode_results[MODES[1]]
                parity = {
                    "trial_id": trial_id,
                    "semantic": semantic,
                    "events_exact": seq["events"] == batch["events"],
                    "edges_exact": seq["edges"] == batch["edges"],
                    "state_sha256_exact": seq["state_sha256"] == batch["state_sha256"],
                    "sequential_state_sha256": seq["state_sha256"],
                    "batched_state_sha256": batch["state_sha256"],
                }
                parity["pass"] = bool(
                    parity["events_exact"]
                    and parity["edges_exact"]
                    and parity["state_sha256_exact"]
                )
                parity_records.append(parity)
                trial_semantic_results[trial_id][semantic] = seq
                for mode, result in mode_results.items():
                    for event in result["events"]:
                        all_journal.append(
                            {"trial_id": trial_id, "consumption_mode": mode, **event}
                        )
                    for view in ledger["scientific_core"]["views"]:
                        unit = score_view(
                            trial_id=trial_id,
                            semantic=semantic,
                            mode=mode,
                            view=view,
                            events=result["events"],
                        )
                        unit["signal_engine_parity_pass"] = parity["pass"]
                        unit["necessary_event_gate_pass"] = bool(
                            unit["necessary_event_gate_pass"] and parity["pass"]
                        )
                        all_units.append(unit)
            all_diagnostics.extend(
                _diagnostic_rows(
                    trial_id,
                    ledger,
                    trial_semantic_results[trial_id],
                )
            )

        candidate_summary: dict[str, Any] = {}
        selection_eligible: list[str] = []
        for semantic in SEMANTICS:
            selection_units = [
                row
                for row in all_units
                if row["trial_id"] == SELECTION_TRIAL and row["semantic"] == semantic
            ]
            challenge_units = [
                row
                for row in all_units
                if row["trial_id"] == CHALLENGE_TRIAL and row["semantic"] == semantic
            ]
            selection_pass = bool(
                len(selection_units) == 8
                and all(row["necessary_event_gate_pass"] for row in selection_units)
            )
            challenge_pass = bool(
                len(challenge_units) == 8
                and all(row["necessary_event_gate_pass"] for row in challenge_units)
            )
            if selection_pass:
                selection_eligible.append(semantic)
            candidate_summary[semantic] = {
                "selection_trial_04_necessary_event_gate_pass": selection_pass,
                "challenge_trial_02_necessary_event_gate_pass": challenge_pass,
                "selection_failed_units": [
                    f"{row['view_id']}:{row['consumption_mode']}"
                    for row in selection_units
                    if not row["necessary_event_gate_pass"]
                ],
                "challenge_failed_units": [
                    f"{row['view_id']}:{row['consumption_mode']}"
                    for row in challenge_units
                    if not row["necessary_event_gate_pass"]
                ],
            }

        terminal = not selection_eligible
        decision = {
            "schema_version": SCHEMA_VERSION,
            "status": (
                "TERMINAL_NO_V18_SIGNAL_SEMANTIC_PASSES_SELECTION_PREREQUISITES"
                if terminal
                else "EVENT_PREFLIGHT_PASS_FULL_FSM_REQUIRED_BEFORE_ANY_FINALIST"
            ),
            "decision": (
                "STOP_WITHOUT_OPENING_TRIAL_08"
                if terminal
                else "STOP_PENDING_SEPARATE_FULL_FSM_GATE"
            ),
            "selection_trial": SELECTION_TRIAL,
            "challenge_trial": CHALLENGE_TRIAL,
            "selection_eligible_semantics": selection_eligible,
            "finalist_id": None,
            "candidate_summary": candidate_summary,
            "necessary_gate_only": True,
            "full_fsm_not_executed": True,
            "full_fsm_skip_reason": (
                "Every candidate failed a necessary count/order/timing/delivery/clear gate; a full FSM cannot turn a necessary FAIL into an eligible PASS without invalid or rejected events."
                if terminal
                else "A separate hash-pinned full-FSM implementation is mandatory."
            ),
            "internal_holdout_trial_08_opened": False,
            "validation_trial_05_opened": False,
            "sealed_trial_06_opened": False,
            "reserve_trials_03_07_opened": False,
            "h0_or_training_executed": False,
        }

        units_path = _atomic_jsonl(output_dir / "semantic_unit_metrics.jsonl", all_units)
        journal_path = _atomic_jsonl(output_dir / "semantic_event_journal.jsonl", all_journal)
        diagnostics_path = _atomic_jsonl(
            output_dir / "canonical_hs_diagnostics.jsonl", all_diagnostics
        )
        parity_path = _atomic_json(
            output_dir / "signal_engine_parity_receipt.json",
            {
                "schema_version": SCHEMA_VERSION,
                "status": (
                    "PASS_EXACT_SEQUENTIAL_BATCH_SIGNAL_ENGINE_PARITY"
                    if all(row["pass"] for row in parity_records)
                    else "FAIL_SIGNAL_ENGINE_PARITY"
                ),
                "records": parity_records,
            },
        )
        decision_path = _atomic_json(output_dir / "development_decision_lock.json", decision)
        manifest = {
            "schema_version": SCHEMA_VERSION,
            "status": decision["status"],
            "analysis_id": ANALYSIS_ID,
            "contract_sha256": contract["_contract_sha256"],
            "raw_trace_manifest": _artifact(contract["_source_paths"]["raw_manifest"]),
            "artifacts": {
                "unit_metrics": _artifact(units_path),
                "event_journal": _artifact(journal_path),
                "canonical_hs_diagnostics": _artifact(diagnostics_path),
                "parity_receipt": _artifact(parity_path),
                "decision_lock": _artifact(decision_path),
            },
            "cardinality": {
                "unit_rows": len(all_units),
                "event_journal_rows": len(all_journal),
                "canonical_hs_diagnostic_rows": len(all_diagnostics),
                "parity_records": len(parity_records),
            },
            "decision": decision,
            "protected_trials_opened": [],
            "reserve_trials_opened": [],
            "internal_holdout_opened": False,
        }
        manifest_path = _atomic_json(output_dir / "manifest.json", manifest)
        return {"status": manifest["status"], "manifest": _artifact(manifest_path)}
    except Exception as exc:
        failure = output_dir / "failure.json"
        if not failure.exists():
            _atomic_json(
                failure,
                {
                    "schema_version": SCHEMA_VERSION,
                    "status": "ERROR_AFTER_V18_SELECTION_DESTINATION_CONSUMED",
                    "error": f"{type(exc).__name__}: {exc}",
                    "traceback": traceback.format_exc(),
                    "rerun_allowed": False,
                    "internal_holdout_opened": False,
                    "protected_trials_opened": [],
                    "reserve_trials_opened": [],
                },
            )
        raise


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    action = parser.add_mutually_exclusive_group(required=True)
    action.add_argument("--check", action="store_true")
    action.add_argument("--execute", action="store_true")
    parser.add_argument("--contract", type=Path, default=DEFAULT_CONTRACT)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        contract = load_and_validate_contract(args.contract)
        check = preflight(contract)
        if args.check:
            print(json.dumps(check, indent=2, sort_keys=True, allow_nan=False))
            return 0
        result = run_selection(contract, args.output_dir)
        print(json.dumps(result, indent=2, sort_keys=True, allow_nan=False))
        return 0
    except Exception as exc:
        print(f"V18 selection failed closed: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
