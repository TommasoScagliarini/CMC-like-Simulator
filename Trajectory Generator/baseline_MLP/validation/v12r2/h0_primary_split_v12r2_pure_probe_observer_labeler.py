"""Self-contained pure-probe replay and observer-only H0 labelling for V12R2.

The online half of this module is deliberately a passive recorder.  It never
loads H0, queries a policy, serves an action, or calls ``reset``/``step`` on an
environment.  It records the actor state at each action boundary and the next
boundary's physical inputs in a pickle-free NPZ.

The offline half first validates the already-closed probe and the complete NPZ,
then reconstructs the authoritative V10 ``LegacyGaitShadow`` sequentially.
Only after that closure check may it load H0 and issue exactly one mean query
for each recorded actor state.  The H0 counterfactual may change precisely
columns 10 through 24 inclusive; the persisted fit input remains the original
V26 actor observation paired with the same-state H0 mean.
"""

from __future__ import annotations

import dataclasses
import hashlib
import importlib
import json
import math
import os
import sys
import tempfile
from dataclasses import dataclass
from pathlib import Path, PurePath
from typing import Any, Callable, Mapping, Sequence

import numpy as np


def _discover_repo_root(source: Path) -> Path:
    for candidate in source.resolve().parents:
        if (
            (candidate / "AGENTS.md").is_file()
            and (candidate / "validation").is_dir()
            and (candidate / "Trajectory Generator").is_dir()
        ):
            return candidate
    raise RuntimeError("repository root could not be discovered")


REPO_ROOT = _discover_repo_root(Path(__file__))
VALIDATION_ROOT = REPO_ROOT / "validation"
TRAJECTORY_ROOT = REPO_ROOT / "Trajectory Generator"
BASELINE_ROOT = TRAJECTORY_ROOT / "baseline_MLP"
LOCAL_VALIDATION_ROOT = BASELINE_ROOT / "validation"
REVISION_ROOT = Path(__file__).resolve().parent
for _root in (
    REPO_ROOT,
    VALIDATION_ROOT,
    TRAJECTORY_ROOT,
    BASELINE_ROOT,
    LOCAL_VALIDATION_ROOT,
    REVISION_ROOT,
):
    if str(_root) not in sys.path:
        sys.path.insert(0, str(_root))

import h0_forensic_rollout as forensic  # noqa: E402
import h0_primary_split_v10_coherent_teacher as coherent_teacher  # noqa: E402


REPLAY_SCHEMA_ID = "H0_V12R2_OBSERVER_REPLAY_BOUNDARIES_V2"
LABEL_CORPUS_SCHEMA_ID = "H0_V12R2_OBSERVER_LABEL_CORPUS_V1"
ACTOR_FEATURE_COUNT = 35
ACTION_DIM = 2
MUTABLE_ACTOR_COLUMNS = tuple(range(10, 25))
IMMUTABLE_ACTOR_COLUMNS = tuple(
    index for index in range(ACTOR_FEATURE_COUNT) if index not in MUTABLE_ACTOR_COLUMNS
)
TIME_TOLERANCE_S = 1.0e-9

BOUNDARY_ARRAY_DTYPES: Mapping[str, np.dtype[Any]] = {
    "boundary_time_s": np.dtype(np.float64),
    "pros_knee_angle_rad": np.dtype(np.float32),
    "pros_knee_speed_rad_s": np.dtype(np.float32),
    "pros_ankle_angle_rad": np.dtype(np.float32),
    "pros_ankle_speed_rad_s": np.dtype(np.float32),
    "legacy_left_normal_grf_bw": np.dtype(np.float32),
    "legacy_left_in_contact": np.dtype(np.bool_),
}
STEP_ARRAY_DTYPES: Mapping[str, np.dtype[Any]] = {
    "actor_observations": np.dtype(np.float32),
    "previous_penetration_m": np.dtype(np.float64),
}
CONFIG_ARRAY_DTYPES: Mapping[str, np.dtype[Any]] = {
    "legacy_fsm_config_json_utf8": np.dtype(np.uint8),
    "legacy_fsm_config_sha256_ascii": np.dtype(np.uint8),
    "legacy_fsm_module_utf8": np.dtype(np.uint8),
    "legacy_fsm_class_utf8": np.dtype(np.uint8),
}
SCALAR_ARRAY_DTYPES: Mapping[str, np.dtype[Any]] = {
    "body_weight_n": np.dtype(np.float64),
    "event_contract_id_utf8": np.dtype(np.uint8),
}
EVENT_ARRAY_DTYPES: Mapping[str, np.dtype[Any] | str] = {
    "legacy_left_event_boundary_offsets": np.dtype(np.int64),
    "legacy_left_event_side": "U5",
    "legacy_left_event_type": "U11",
    "legacy_left_event_onset_time_s": np.dtype(np.float64),
    "legacy_left_event_confirmed_time_s": np.dtype(np.float64),
    "legacy_left_event_delivered_time_s": np.dtype(np.float64),
    "legacy_left_event_cycle_duration_present": np.dtype(np.bool_),
    "legacy_left_event_cycle_duration_s": np.dtype(np.float64),
    "legacy_left_event_contact_duration_present": np.dtype(np.bool_),
    "legacy_left_event_contact_duration_s": np.dtype(np.float64),
    "legacy_left_event_startup_contact_present": np.dtype(np.bool_),
    "legacy_left_event_startup_contact": np.dtype(np.bool_),
}
REPLAY_ARRAY_NAMES = frozenset(
    (
        *BOUNDARY_ARRAY_DTYPES,
        *STEP_ARRAY_DTYPES,
        *CONFIG_ARRAY_DTYPES,
        *SCALAR_ARRAY_DTYPES,
        *EVENT_ARRAY_DTYPES,
    )
)

LABEL_ARRAY_DTYPES: Mapping[str, np.dtype[Any] | str] = {
    "observations": np.dtype(np.float32),
    "actions": np.dtype(np.float32),
    "reset_mask": np.dtype(np.bool_),
    "previous_penetration_m": np.dtype(np.float64),
    "coverage_distance_rms_z": np.dtype(np.float64),
    "coverage_nearest_reference_index": np.dtype(np.int64),
    "coverage_ood_mask": np.dtype(np.bool_),
    "raw_sample_weights": np.dtype(np.float64),
    "normalized_sample_weights": np.dtype(np.float64),
    "actor_feature_names": "U64",
    "case_ids": "U64",
    "step_indices": np.dtype(np.int64),
    "tranche_ids": "U64",
    "origins": "U160",
}


class V12R2ObserverLabelError(RuntimeError):
    """Raised when replay or labelling cannot remain exact and observer-only."""


@dataclass(frozen=True)
class LoadedReplay:
    """Strictly decoded replay plus canonical configuration provenance."""

    arrays: Mapping[str, np.ndarray]
    config: Mapping[str, Any]
    config_json: bytes
    config_sha256: str
    fsm_module: str
    fsm_class: str
    event_contract_id: str
    n_steps: int
    boundary_count: int
    event_count: int


@dataclass(frozen=True)
class ReplayedTeacherViews:
    """Sequential V10 replay result generated without any policy query."""

    student_observations: np.ndarray
    teacher_observations: np.ndarray
    replayed_boundary_count: int
    changed_only_mutable_count: int
    invariant_columns_byte_exact_count: int
    column_24_changed_count: int


@dataclass(frozen=True)
class CoverageResult:
    """Exact coverage output used by preregistered observer weighting."""

    distance_rms_z: np.ndarray
    nearest_reference_index: np.ndarray
    ood_mask: np.ndarray
    audit: Mapping[str, Any]


@dataclass(frozen=True)
class ObserverLabelResult:
    """Complete in-memory labels and zero-environment execution counters."""

    arrays: Mapping[str, np.ndarray]
    replay: LoadedReplay
    teacher_views: ReplayedTeacherViews
    coverage: CoverageResult
    teacher_query_count: int
    environment_reset_calls: int = 0
    environment_step_calls: int = 0
    action_served_count: int = 0


def _contract_module() -> Any:
    return importlib.import_module("h0_primary_split_v12r2_autonomy_recovery_contract")


def _finite(value: Any, *, label: str, nonnegative: bool = False) -> float:
    if isinstance(value, (bool, np.bool_)):
        raise V12R2ObserverLabelError(f"{label} must be numeric")
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise V12R2ObserverLabelError(f"{label} must be numeric") from exc
    if not math.isfinite(result):
        raise V12R2ObserverLabelError(f"{label} must be finite")
    if nonnegative and result < 0.0:
        raise V12R2ObserverLabelError(f"{label} must be nonnegative")
    return result


def _native_bool(value: Any, *, label: str) -> bool:
    if not isinstance(value, (bool, np.bool_)):
        raise V12R2ObserverLabelError(f"{label} must be boolean")
    return bool(value)


def _utf8_array(value: str, *, label: str) -> np.ndarray:
    if not isinstance(value, str) or not value:
        raise V12R2ObserverLabelError(f"{label} must be nonempty text")
    encoded = value.encode("utf-8")
    return np.frombuffer(encoded, dtype=np.uint8).copy()


def _decode_utf8(array: np.ndarray, *, label: str, ascii_only: bool = False) -> str:
    if array.dtype != np.dtype(np.uint8) or array.ndim != 1 or array.size == 0:
        raise V12R2ObserverLabelError(f"{label} must be a nonempty uint8 vector")
    try:
        result = array.tobytes().decode("ascii" if ascii_only else "utf-8")
    except UnicodeDecodeError as exc:
        raise V12R2ObserverLabelError(f"{label} is not valid text") from exc
    if not result:
        raise V12R2ObserverLabelError(f"{label} must be nonempty")
    return result


def _reject_duplicate_pairs(pairs: Sequence[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise V12R2ObserverLabelError(f"legacy FSM config repeats JSON key {key!r}")
        result[key] = value
    return result


def _reject_json_constant(value: str) -> None:
    raise V12R2ObserverLabelError(
        f"legacy FSM config contains non-finite JSON constant {value}"
    )


def canonical_legacy_fsm_config(runtime_phase_fsm: Any) -> tuple[bytes, str, str]:
    """Return compact canonical JSON for the runtime FSM with legacy routing."""

    config = getattr(runtime_phase_fsm, "config", None)
    if config is None or not dataclasses.is_dataclass(config):
        raise V12R2ObserverLabelError("runtime phase FSM config must be a dataclass")
    try:
        legacy_config = dataclasses.replace(config, event_source="legacy_events")
        payload = dataclasses.asdict(legacy_config)
    except Exception as exc:
        raise V12R2ObserverLabelError(
            "runtime phase FSM config cannot be canonicalized"
        ) from exc
    try:
        encoded = json.dumps(
            payload,
            sort_keys=True,
            separators=(",", ":"),
            ensure_ascii=False,
            allow_nan=False,
        ).encode("utf-8")
    except (TypeError, ValueError, UnicodeError) as exc:
        raise V12R2ObserverLabelError(
            "legacy FSM config is not finite strict JSON"
        ) from exc
    module_name = str(type(runtime_phase_fsm).__module__)
    class_name = str(type(runtime_phase_fsm).__qualname__)
    if not module_name or not class_name or "<locals>" in class_name:
        raise V12R2ObserverLabelError("runtime phase FSM identity is unstable")
    return encoded, module_name, class_name


def _event_timestamp(
    event: Mapping[str, Any],
    *,
    primary: str,
    fallback: str | None,
    label: str,
    default: float | None = None,
) -> float:
    primary_present = primary in event and event.get(primary) is not None
    fallback_present = (
        fallback is not None and fallback in event and event.get(fallback) is not None
    )
    if not primary_present and not fallback_present:
        if default is None:
            raise V12R2ObserverLabelError(f"{label} is missing")
        return float(default)
    primary_value = (
        _finite(event[primary], label=f"{label}.{primary}") if primary_present else None
    )
    fallback_value = (
        _finite(event[fallback], label=f"{label}.{fallback}")
        if fallback_present and fallback is not None
        else None
    )
    if (
        primary_value is not None
        and fallback_value is not None
        and abs(primary_value - fallback_value) > TIME_TOLERANCE_S
    ):
        raise V12R2ObserverLabelError(f"{label} timestamp aliases disagree")
    return float(primary_value if primary_value is not None else fallback_value)


def _observation_fields(info: Mapping[str, Any]) -> tuple[float, float, float, float]:
    observation = info.get("observation")
    if not isinstance(observation, Mapping):
        raise V12R2ObserverLabelError("info.observation must be an object")
    return (
        _finite(observation.get("pros_knee_angle"), label="pros knee angle"),
        _finite(observation.get("pros_knee_angle_vel"), label="pros knee speed"),
        _finite(observation.get("pros_ankle_angle"), label="pros ankle angle"),
        _finite(observation.get("pros_ankle_angle_vel"), label="pros ankle speed"),
    )


class ProbeReplayAccumulator:
    """Passive online accumulator; it owns no environment or policy object."""

    def __init__(
        self,
        *,
        reset_info: Mapping[str, Any],
        body_weight_n: float,
        runtime_phase_fsm: Any,
        event_contract_id: str,
    ) -> None:
        self._body_weight_n = _finite(
            body_weight_n, label="body_weight_n", nonnegative=True
        )
        if self._body_weight_n <= 0.0:
            raise V12R2ObserverLabelError("body_weight_n must be positive")
        if not isinstance(event_contract_id, str) or not event_contract_id:
            raise V12R2ObserverLabelError("event_contract_id must be nonempty")
        self._event_contract_id = event_contract_id
        config_json, fsm_module, fsm_class = canonical_legacy_fsm_config(
            runtime_phase_fsm
        )
        self._config_json = config_json
        self._fsm_module = fsm_module
        self._fsm_class = fsm_class
        self._boundaries: dict[str, list[Any]] = {
            name: [] for name in BOUNDARY_ARRAY_DTYPES
        }
        self._actors: list[np.ndarray] = []
        self._penetrations: list[float] = []
        self._events: list[dict[str, Any]] = []
        self._event_offsets: list[int] = [0]
        self._closed = False
        self._append_boundary(reset_info, reset_boundary=True)

    @property
    def step_count(self) -> int:
        return len(self._actors)

    @property
    def boundary_count(self) -> int:
        return len(self._boundaries["boundary_time_s"])

    def _append_boundary(
        self, info: Mapping[str, Any], *, reset_boundary: bool
    ) -> None:
        if not isinstance(info, Mapping):
            raise V12R2ObserverLabelError("boundary info must be an object")
        boundary = _finite(info.get("time"), label="info.time")
        if self._boundaries["boundary_time_s"] and (
            boundary <= self._boundaries["boundary_time_s"][-1] + TIME_TOLERANCE_S
        ):
            raise V12R2ObserverLabelError(
                "policy boundary times must be strictly increasing"
            )
        knee, knee_speed, ankle, ankle_speed = _observation_fields(info)
        detector = info.get("online_grf_detector")
        if not isinstance(detector, Mapping):
            raise V12R2ObserverLabelError("info.online_grf_detector must be an object")
        if reset_boundary:
            if detector:
                raise V12R2ObserverLabelError("reset detector stream must be empty")
            load_bw = 0.0
            in_contact = False
        else:
            left = detector.get("left")
            if not isinstance(left, Mapping):
                raise V12R2ObserverLabelError("left detector stream is missing")
            normal_force_n = _finite(
                left.get("normal_force"),
                label="left detector normal force",
                nonnegative=True,
            )
            load_bw = normal_force_n / self._body_weight_n
            in_contact = _native_bool(
                left.get("in_contact"), label="left detector contact"
            )

        raw_events = info.get("legacy_online_events")
        if not isinstance(raw_events, Sequence) or isinstance(
            raw_events, (str, bytes, bytearray)
        ):
            raise V12R2ObserverLabelError(
                "info.legacy_online_events must be an ordered sequence"
            )
        if reset_boundary and raw_events:
            raise V12R2ObserverLabelError("reset legacy event stream must be empty")
        previous_onset = (
            float(self._events[-1]["onset_time_s"]) if self._events else None
        )
        for raw_index, raw in enumerate(raw_events):
            label = f"legacy_online_events[{raw_index}]"
            if not isinstance(raw, Mapping):
                raise V12R2ObserverLabelError(f"{label} must be an object")
            side = str(raw.get("side", "")).strip().lower()
            event_type = str(raw.get("event", "")).strip().lower()
            if side not in {"left", "right"}:
                raise V12R2ObserverLabelError(f"{label}.side is unknown")
            if event_type not in {"heel_strike", "toe_off"}:
                raise V12R2ObserverLabelError(f"{label}.event is unknown")
            onset = _event_timestamp(
                raw,
                primary="event_time_s",
                fallback="time",
                label=label,
            )
            confirmed = _event_timestamp(
                raw,
                primary="confirmed_time_s",
                fallback="confirmed_time",
                label=label,
                default=onset,
            )
            delivered = _event_timestamp(
                raw,
                primary="delivered_time_s",
                fallback=None,
                label=label,
                default=boundary,
            )
            if not (
                onset <= confirmed + TIME_TOLERANCE_S
                and confirmed <= delivered + TIME_TOLERANCE_S
                and abs(delivered - boundary) <= TIME_TOLERANCE_S
            ):
                raise V12R2ObserverLabelError(f"{label} timestamps are not causal")
            if side != "left":
                continue
            if (
                previous_onset is not None
                and onset <= previous_onset + TIME_TOLERANCE_S
            ):
                raise V12R2ObserverLabelError(
                    "legacy left event onsets must be strictly increasing"
                )
            previous_onset = onset

            def optional_float(name: str) -> tuple[bool, float]:
                present = name in raw and raw.get(name) is not None
                value = (
                    _finite(raw[name], label=f"{label}.{name}", nonnegative=True)
                    if present
                    else 0.0
                )
                return present, value

            cycle_present, cycle = optional_float("cycle_duration_s")
            contact_present, contact = optional_float("contact_duration_s")
            startup_keys = tuple(
                key
                for key in ("startup_contact", "startup_partial_stance")
                if key in raw and raw.get(key) is not None
            )
            startup_present = bool(startup_keys)
            startup = False
            if startup_present:
                values = [
                    _native_bool(raw[key], label=f"{label}.{key}")
                    for key in startup_keys
                ]
                if len(set(values)) != 1:
                    raise V12R2ObserverLabelError(
                        f"{label} startup flag aliases disagree"
                    )
                startup = values[0]
            self._events.append(
                {
                    "side": "left",
                    "event_type": event_type,
                    "onset_time_s": onset,
                    "confirmed_time_s": confirmed,
                    "delivered_time_s": boundary,
                    "cycle_present": cycle_present,
                    "cycle_duration_s": cycle,
                    "contact_present": contact_present,
                    "contact_duration_s": contact,
                    "startup_present": startup_present,
                    "startup_contact": startup,
                }
            )

        self._boundaries["boundary_time_s"].append(boundary)
        self._boundaries["pros_knee_angle_rad"].append(knee)
        self._boundaries["pros_knee_speed_rad_s"].append(knee_speed)
        self._boundaries["pros_ankle_angle_rad"].append(ankle)
        self._boundaries["pros_ankle_speed_rad_s"].append(ankle_speed)
        self._boundaries["legacy_left_normal_grf_bw"].append(load_bw)
        self._boundaries["legacy_left_in_contact"].append(in_contact)
        self._event_offsets.append(len(self._events))

    def append_step(
        self,
        *,
        actor_observation: Any,
        previous_penetration_m: float,
        next_info: Mapping[str, Any],
    ) -> None:
        """Record one current action state and its resulting next boundary."""

        if self._closed:
            raise V12R2ObserverLabelError("probe replay accumulator is closed")
        actor = np.ascontiguousarray(np.asarray(actor_observation))
        if actor.dtype != np.dtype(np.float32) or actor.shape != (ACTOR_FEATURE_COUNT,):
            raise V12R2ObserverLabelError(
                "actor_observation must have exact dtype float32 and shape (35,)"
            )
        if not np.all(np.isfinite(actor)):
            raise V12R2ObserverLabelError(
                "actor_observation contains non-finite values"
            )
        current = self.boundary_count - 1
        expected = np.asarray(
            [
                self._boundaries["pros_knee_angle_rad"][current],
                self._boundaries["pros_knee_speed_rad_s"][current],
                self._boundaries["pros_ankle_angle_rad"][current],
                self._boundaries["pros_ankle_speed_rad_s"][current],
            ],
            dtype=np.float32,
        )
        if actor[2:6].tobytes(order="C") != expected.tobytes(order="C"):
            raise V12R2ObserverLabelError(
                "actor/current-boundary prosthetic kinematics are not byte exact"
            )
        penetration = _finite(
            previous_penetration_m,
            label="previous_penetration_m",
            nonnegative=True,
        )
        # Append state metadata only after the resulting boundary is valid.
        self._append_boundary(next_info, reset_boundary=False)
        self._actors.append(actor.copy())
        self._penetrations.append(penetration)

    def arrays(self) -> dict[str, np.ndarray]:
        """Close the accumulator and return the exact V12R2 NPZ mapping."""

        if self._closed:
            raise V12R2ObserverLabelError("probe replay accumulator already closed")
        if not self._actors or self.boundary_count != self.step_count + 1:
            raise V12R2ObserverLabelError(
                "replay must contain N states and N+1 boundaries"
            )
        self._closed = True
        arrays: dict[str, np.ndarray] = {
            name: np.ascontiguousarray(values, dtype=dtype)
            for name, dtype in BOUNDARY_ARRAY_DTYPES.items()
            for values in (self._boundaries[name],)
        }
        arrays["actor_observations"] = np.ascontiguousarray(
            self._actors, dtype=np.float32
        )
        arrays["previous_penetration_m"] = np.ascontiguousarray(
            self._penetrations, dtype=np.float64
        )
        config_sha = hashlib.sha256(self._config_json).hexdigest()
        arrays.update(
            {
                "legacy_fsm_config_json_utf8": np.frombuffer(
                    self._config_json, dtype=np.uint8
                ).copy(),
                "legacy_fsm_config_sha256_ascii": np.frombuffer(
                    config_sha.encode("ascii"), dtype=np.uint8
                ).copy(),
                "legacy_fsm_module_utf8": _utf8_array(
                    self._fsm_module, label="legacy FSM module"
                ),
                "legacy_fsm_class_utf8": _utf8_array(
                    self._fsm_class, label="legacy FSM class"
                ),
                "body_weight_n": np.asarray([self._body_weight_n], dtype=np.float64),
                "event_contract_id_utf8": _utf8_array(
                    self._event_contract_id, label="event contract id"
                ),
                "legacy_left_event_boundary_offsets": np.asarray(
                    self._event_offsets, dtype=np.int64
                ),
                "legacy_left_event_side": np.asarray(
                    [event["side"] for event in self._events], dtype="U5"
                ),
                "legacy_left_event_type": np.asarray(
                    [event["event_type"] for event in self._events], dtype="U11"
                ),
                "legacy_left_event_onset_time_s": np.asarray(
                    [event["onset_time_s"] for event in self._events],
                    dtype=np.float64,
                ),
                "legacy_left_event_confirmed_time_s": np.asarray(
                    [event["confirmed_time_s"] for event in self._events],
                    dtype=np.float64,
                ),
                "legacy_left_event_delivered_time_s": np.asarray(
                    [event["delivered_time_s"] for event in self._events],
                    dtype=np.float64,
                ),
                "legacy_left_event_cycle_duration_present": np.asarray(
                    [event["cycle_present"] for event in self._events],
                    dtype=np.bool_,
                ),
                "legacy_left_event_cycle_duration_s": np.asarray(
                    [event["cycle_duration_s"] for event in self._events],
                    dtype=np.float64,
                ),
                "legacy_left_event_contact_duration_present": np.asarray(
                    [event["contact_present"] for event in self._events],
                    dtype=np.bool_,
                ),
                "legacy_left_event_contact_duration_s": np.asarray(
                    [event["contact_duration_s"] for event in self._events],
                    dtype=np.float64,
                ),
                "legacy_left_event_startup_contact_present": np.asarray(
                    [event["startup_present"] for event in self._events],
                    dtype=np.bool_,
                ),
                "legacy_left_event_startup_contact": np.asarray(
                    [event["startup_contact"] for event in self._events],
                    dtype=np.bool_,
                ),
            }
        )
        return arrays

    def write_exclusive(self, path: str | Path) -> Path:
        return write_npz_exclusive(path, self.arrays())


class PureProbeReplayRecorder:
    """Runner-facing recorder with explicit reset/current/next-boundary ordering.

    ``record_reset`` stores the first actor as pending.  Every
    ``record_step_boundary`` commits that pending actor against the new physical
    boundary, then optionally installs the next actor as pending.  The terminal
    call must pass ``actor_observation_or_none=None`` so that N actions produce
    N rows and N+1 boundaries.
    """

    def __init__(
        self,
        *,
        runtime_phase_fsm: Any,
        body_weight_n: float,
        actor_feature_names: Sequence[str],
        event_contract_id: str | None,
    ) -> None:
        coherent_teacher.validate_actor_feature_names(actor_feature_names)
        self._runtime_phase_fsm = runtime_phase_fsm
        self._body_weight_n = body_weight_n
        self._event_contract_id = event_contract_id
        self._accumulator: ProbeReplayAccumulator | None = None
        self._pending_actor: np.ndarray | None = None
        self._pending_penetration_m: float | None = None

    @classmethod
    def from_runtime(
        cls,
        runtime_phase_fsm: Any,
        body_weight_n: float,
        actor_feature_names: Sequence[str],
        *,
        event_contract_id: str | None = None,
    ) -> "PureProbeReplayRecorder":
        return cls(
            runtime_phase_fsm=runtime_phase_fsm,
            body_weight_n=body_weight_n,
            actor_feature_names=actor_feature_names,
            event_contract_id=event_contract_id,
        )

    def record_reset(
        self,
        actor_observation: Any,
        info: Mapping[str, Any],
        *,
        previous_penetration_m: float = 0.0,
    ) -> None:
        if self._accumulator is not None:
            raise V12R2ObserverLabelError("probe reset was already recorded")
        event_contract_id = self._event_contract_id
        if event_contract_id is None:
            raw = info.get("event_contract_id")
            if not isinstance(raw, str) or not raw:
                raise V12R2ObserverLabelError(
                    "event_contract_id must be supplied or present in reset info"
                )
            event_contract_id = raw
        self._accumulator = ProbeReplayAccumulator(
            reset_info=info,
            body_weight_n=self._body_weight_n,
            runtime_phase_fsm=self._runtime_phase_fsm,
            event_contract_id=event_contract_id,
        )
        actor = np.ascontiguousarray(np.asarray(actor_observation))
        if actor.dtype != np.dtype(np.float32) or actor.shape != (ACTOR_FEATURE_COUNT,):
            raise V12R2ObserverLabelError(
                "reset actor must have exact dtype float32 and shape (35,)"
            )
        self._pending_actor = actor.copy()
        self._pending_penetration_m = _finite(
            previous_penetration_m,
            label="reset previous penetration",
            nonnegative=True,
        )

    def record_step_boundary(
        self,
        actor_observation_or_none: Any | None,
        info: Mapping[str, Any],
        *,
        previous_penetration_m: float | None = None,
    ) -> None:
        if (
            self._accumulator is None
            or self._pending_actor is None
            or self._pending_penetration_m is None
        ):
            raise V12R2ObserverLabelError(
                "record_reset must establish a pending actor first"
            )
        self._accumulator.append_step(
            actor_observation=self._pending_actor,
            previous_penetration_m=self._pending_penetration_m,
            next_info=info,
        )
        if actor_observation_or_none is None:
            if previous_penetration_m is not None:
                raise V12R2ObserverLabelError(
                    "terminal boundary must not provide next-row penetration"
                )
            self._pending_actor = None
            self._pending_penetration_m = None
            return
        if previous_penetration_m is None:
            raise V12R2ObserverLabelError(
                "nonterminal boundary requires next-row previous penetration"
            )
        actor = np.ascontiguousarray(np.asarray(actor_observation_or_none))
        if actor.dtype != np.dtype(np.float32) or actor.shape != (ACTOR_FEATURE_COUNT,):
            raise V12R2ObserverLabelError(
                "next actor must have exact dtype float32 and shape (35,)"
            )
        self._pending_actor = actor.copy()
        self._pending_penetration_m = _finite(
            previous_penetration_m,
            label="next-row previous penetration",
            nonnegative=True,
        )

    def arrays(self) -> dict[str, np.ndarray]:
        if self._accumulator is None:
            raise V12R2ObserverLabelError("probe reset was not recorded")
        if self._pending_actor is not None:
            raise V12R2ObserverLabelError(
                "terminal boundary was not recorded; one actor remains pending"
            )
        return self._accumulator.arrays()

    def write_exclusive(self, path: str | Path) -> Path:
        return write_npz_exclusive(path, self.arrays())


def _fsync_directory(path: Path) -> None:
    try:
        descriptor = os.open(str(path), os.O_RDONLY)
    except OSError:  # pragma: no cover - platform dependent
        return
    try:
        os.fsync(descriptor)
    except OSError:  # pragma: no cover - platform dependent
        pass
    finally:
        os.close(descriptor)


def write_npz_exclusive(path: str | Path, arrays: Mapping[str, np.ndarray]) -> Path:
    """Publish one NPZ atomically with an exclusive, fail-closed claim."""

    destination = Path(path).expanduser().resolve()
    destination.parent.mkdir(parents=True, exist_ok=True)
    if os.path.lexists(destination):
        raise V12R2ObserverLabelError(f"refusing to clobber: {destination}")
    descriptor, temporary_raw = tempfile.mkstemp(
        prefix=f".{destination.name}.", suffix=".npz", dir=str(destination.parent)
    )
    os.close(descriptor)
    temporary = Path(temporary_raw)
    try:
        np.savez(temporary, **arrays)
        with temporary.open("rb") as stream:
            os.fsync(stream.fileno())
        flags = os.O_CREAT | os.O_EXCL | os.O_WRONLY | getattr(os, "O_BINARY", 0)
        try:
            claim = os.open(str(destination), flags, 0o600)
        except FileExistsError as exc:
            raise V12R2ObserverLabelError(
                f"refusing to clobber: {destination}"
            ) from exc
        else:
            os.close(claim)
        os.replace(str(temporary), str(destination))
        _fsync_directory(destination.parent)
        return destination
    finally:
        try:
            temporary.unlink()
        except FileNotFoundError:
            pass


# Explicit runner-facing spelling requested by the V12R2 orchestrator.
write_replay_npz_exclusive = write_npz_exclusive


def _dtype_matches(array: np.ndarray, expected: np.dtype[Any] | str) -> bool:
    if isinstance(expected, str) and expected.startswith("U"):
        return array.dtype == np.dtype(expected)
    return array.dtype == expected


def _strict_config(config_json: bytes) -> dict[str, Any]:
    try:
        value = json.loads(
            config_json.decode("utf-8"),
            object_pairs_hook=_reject_duplicate_pairs,
            parse_constant=_reject_json_constant,
        )
    except V12R2ObserverLabelError:
        raise
    except (UnicodeError, json.JSONDecodeError) as exc:
        raise V12R2ObserverLabelError("legacy FSM config JSON is invalid") from exc
    if not isinstance(value, Mapping):
        raise V12R2ObserverLabelError("legacy FSM config JSON must be an object")
    try:
        canonical = json.dumps(
            value,
            sort_keys=True,
            separators=(",", ":"),
            ensure_ascii=False,
            allow_nan=False,
        ).encode("utf-8")
    except (TypeError, ValueError, UnicodeError) as exc:
        raise V12R2ObserverLabelError(
            "legacy FSM config JSON is not canonical"
        ) from exc
    if canonical != config_json:
        raise V12R2ObserverLabelError("legacy FSM config JSON bytes are noncanonical")
    if value.get("event_source") != "legacy_events":
        raise V12R2ObserverLabelError("legacy FSM config event source drifted")
    return dict(value)


def load_probe_replay_strict(
    path: str | Path, *, contract_module: Any | None = None
) -> LoadedReplay:
    """Load the exact replay schema with ``allow_pickle=False`` and no coercion."""

    source = Path(path).expanduser().resolve()
    if not source.is_file():
        raise V12R2ObserverLabelError(f"probe replay is missing: {source}")
    try:
        with np.load(source, allow_pickle=False) as archive:
            if set(archive.files) != set(REPLAY_ARRAY_NAMES):
                missing = sorted(REPLAY_ARRAY_NAMES - set(archive.files))
                extra = sorted(set(archive.files) - REPLAY_ARRAY_NAMES)
                raise V12R2ObserverLabelError(
                    f"probe replay array schema drifted; missing={missing}, extra={extra}"
                )
            arrays = {
                name: np.ascontiguousarray(archive[name].copy())
                for name in archive.files
            }
    except V12R2ObserverLabelError:
        raise
    except Exception as exc:
        raise V12R2ObserverLabelError(
            "probe replay is not a pickle-free numeric NPZ"
        ) from exc
    for declarations in (
        BOUNDARY_ARRAY_DTYPES,
        STEP_ARRAY_DTYPES,
        CONFIG_ARRAY_DTYPES,
        SCALAR_ARRAY_DTYPES,
        EVENT_ARRAY_DTYPES,
    ):
        for name, dtype in declarations.items():
            if not _dtype_matches(arrays[name], dtype):
                raise V12R2ObserverLabelError(
                    f"probe replay {name} dtype {arrays[name].dtype} != {dtype}"
                )

    actor = arrays["actor_observations"]
    penetration = arrays["previous_penetration_m"]
    if actor.ndim != 2 or actor.shape[1] != ACTOR_FEATURE_COUNT or len(actor) < 1:
        raise V12R2ObserverLabelError("actor_observations must have shape (N,35)")
    n_steps = len(actor)
    boundaries = n_steps + 1
    if penetration.shape != (n_steps,):
        raise V12R2ObserverLabelError("previous_penetration_m shape drifted")
    for name in BOUNDARY_ARRAY_DTYPES:
        if arrays[name].shape != (boundaries,):
            raise V12R2ObserverLabelError(f"{name} must have shape (N+1,)")
    if arrays["body_weight_n"].shape != (1,):
        raise V12R2ObserverLabelError("body_weight_n must have shape (1,)")
    config_hash_array = arrays["legacy_fsm_config_sha256_ascii"]
    if config_hash_array.shape != (64,):
        raise V12R2ObserverLabelError(
            "legacy_fsm_config_sha256_ascii must have shape (64,)"
        )
    if not np.all(np.isfinite(actor)) or not np.all(np.isfinite(penetration)):
        raise V12R2ObserverLabelError("probe step arrays contain non-finite values")
    if np.any(penetration < 0.0):
        raise V12R2ObserverLabelError("previous penetration must be nonnegative")
    for name in BOUNDARY_ARRAY_DTYPES:
        if arrays[name].dtype.kind == "f" and not np.all(np.isfinite(arrays[name])):
            raise V12R2ObserverLabelError(f"{name} contains non-finite values")
    times = arrays["boundary_time_s"]
    if not np.all(np.diff(times) > TIME_TOLERANCE_S):
        raise V12R2ObserverLabelError("boundary times are not strictly increasing")
    body_weight = _finite(
        arrays["body_weight_n"][0], label="body_weight_n", nonnegative=True
    )
    if body_weight <= 0.0:
        raise V12R2ObserverLabelError("body_weight_n must be positive")
    if arrays["legacy_left_normal_grf_bw"][0].tobytes() != np.float32(
        0.0
    ).tobytes() or bool(arrays["legacy_left_in_contact"][0]):
        raise V12R2ObserverLabelError("reset detector boundary is not empty")
    if np.any(arrays["legacy_left_normal_grf_bw"] < 0.0):
        raise V12R2ObserverLabelError("legacy normal GRF must be nonnegative")
    boundary_kinematics = np.column_stack(
        [
            arrays["pros_knee_angle_rad"][:-1],
            arrays["pros_knee_speed_rad_s"][:-1],
            arrays["pros_ankle_angle_rad"][:-1],
            arrays["pros_ankle_speed_rad_s"][:-1],
        ]
    ).astype(np.float32, copy=False)
    if actor[:, 2:6].tobytes(order="C") != np.ascontiguousarray(
        boundary_kinematics
    ).tobytes(order="C"):
        raise V12R2ObserverLabelError(
            "actor/boundary prosthetic kinematics are not byte exact"
        )

    config_json = arrays["legacy_fsm_config_json_utf8"].tobytes()
    config = _strict_config(config_json)
    observed_hash = _decode_utf8(
        config_hash_array, label="legacy FSM config SHA", ascii_only=True
    )
    expected_hash = hashlib.sha256(config_json).hexdigest()
    if (
        observed_hash != expected_hash
        or len(observed_hash) != 64
        or any(char not in "0123456789abcdef" for char in observed_hash)
    ):
        raise V12R2ObserverLabelError("legacy FSM config SHA-256 mismatch")
    fsm_module = _decode_utf8(
        arrays["legacy_fsm_module_utf8"], label="legacy FSM module"
    )
    fsm_class = _decode_utf8(arrays["legacy_fsm_class_utf8"], label="legacy FSM class")
    event_contract_id = _decode_utf8(
        arrays["event_contract_id_utf8"], label="event contract id"
    )

    offsets = arrays["legacy_left_event_boundary_offsets"]
    if offsets.shape != (boundaries + 1,):
        raise V12R2ObserverLabelError("event boundary offsets must have shape (N+2,)")
    if offsets[0] != 0 or np.any(np.diff(offsets) < 0):
        raise V12R2ObserverLabelError("event boundary offsets are malformed")
    event_count = int(offsets[-1])
    if offsets[1] != 0:
        raise V12R2ObserverLabelError("reset boundary must own zero events")
    for name in EVENT_ARRAY_DTYPES:
        if name == "legacy_left_event_boundary_offsets":
            continue
        if arrays[name].shape != (event_count,):
            raise V12R2ObserverLabelError(f"{name} must have shape (E,)")
    if any(str(value) != "left" for value in arrays["legacy_left_event_side"]):
        raise V12R2ObserverLabelError("event journal must contain only left events")
    if any(
        str(value) not in {"heel_strike", "toe_off"}
        for value in arrays["legacy_left_event_type"]
    ):
        raise V12R2ObserverLabelError("event journal contains an unknown event")
    onset = arrays["legacy_left_event_onset_time_s"]
    confirmed = arrays["legacy_left_event_confirmed_time_s"]
    delivered = arrays["legacy_left_event_delivered_time_s"]
    if not (
        np.all(np.isfinite(onset))
        and np.all(np.isfinite(confirmed))
        and np.all(np.isfinite(delivered))
        and np.all(onset <= confirmed + TIME_TOLERANCE_S)
        and np.all(confirmed <= delivered + TIME_TOLERANCE_S)
    ):
        raise V12R2ObserverLabelError("event timestamps are non-finite or noncausal")
    if event_count > 1 and not np.all(np.diff(onset) > TIME_TOLERANCE_S):
        raise V12R2ObserverLabelError("left event onsets are not strictly increasing")
    for boundary in range(boundaries):
        start, stop = int(offsets[boundary]), int(offsets[boundary + 1])
        if start < 0 or stop > event_count:
            raise V12R2ObserverLabelError("event offsets escape the journal")
        if stop > start and not np.all(
            np.abs(delivered[start:stop] - times[boundary]) <= TIME_TOLERANCE_S
        ):
            raise V12R2ObserverLabelError(
                "event delivery does not match its owning boundary"
            )
    optional_float_pairs = (
        (
            "legacy_left_event_cycle_duration_present",
            "legacy_left_event_cycle_duration_s",
        ),
        (
            "legacy_left_event_contact_duration_present",
            "legacy_left_event_contact_duration_s",
        ),
    )
    for present_name, value_name in optional_float_pairs:
        present = arrays[present_name]
        value = arrays[value_name]
        if not np.all(np.isfinite(value)) or np.any(value < 0.0):
            raise V12R2ObserverLabelError(f"{value_name} is invalid")
        if np.any(value[~present] != 0.0):
            raise V12R2ObserverLabelError(
                f"{value_name} absent values must be canonical zero"
            )
    startup_present = arrays["legacy_left_event_startup_contact_present"]
    startup = arrays["legacy_left_event_startup_contact"]
    if np.any(startup[~startup_present]):
        raise V12R2ObserverLabelError(
            "absent startup contact values must be canonical false"
        )

    selected_contract = contract_module or _contract_module()
    declared_schema = getattr(selected_contract, "PROBE_REPLAY_SCHEMA", {})
    declared_names = set(
        (
            *declared_schema.get("boundary_arrays", {}),
            *declared_schema.get("step_arrays", {}),
            *declared_schema.get("legacy_fsm_config_arrays", {}),
            *declared_schema.get("scalar_arrays", {}),
            *declared_schema.get("left_event_journal_arrays", {}),
        )
    )
    if (
        declared_schema.get("schema_id") != REPLAY_SCHEMA_ID
        or declared_names != set(REPLAY_ARRAY_NAMES)
        or declared_schema.get("teacher_mutable_actor_columns")
        != list(MUTABLE_ACTOR_COLUMNS)
        or selected_contract.replay_schema_gate(declared_schema).get("passed")
        is not True
    ):
        raise V12R2ObserverLabelError("V12R2 replay contract is not available")
    topology = selected_contract.replay_event_topology_gate(arrays, n_steps=n_steps)
    if topology.get("passed") is not True:
        raise V12R2ObserverLabelError(f"V12R2 replay topology gate failed: {topology}")
    return LoadedReplay(
        arrays=arrays,
        config=config,
        config_json=config_json,
        config_sha256=observed_hash,
        fsm_module=fsm_module,
        fsm_class=fsm_class,
        event_contract_id=event_contract_id,
        n_steps=n_steps,
        boundary_count=boundaries,
        event_count=event_count,
    )


def _default_phase_fsm_factory(
    config: Mapping[str, Any], module_name: str, class_name: str
) -> Any:
    if module_name != "prosthetic_phase_fsm" or class_name != "ProstheticPhaseFSM":
        raise V12R2ObserverLabelError(
            "production replay accepts only prosthetic_phase_fsm.ProstheticPhaseFSM"
        )
    from prosthetic_phase_fsm import ProstheticPhaseFSM, ProstheticPhaseFSMConfig

    try:
        fsm_config = ProstheticPhaseFSMConfig(**dict(config))
        return ProstheticPhaseFSM(fsm_config)
    except Exception as exc:
        raise V12R2ObserverLabelError(
            "canonical legacy FSM cannot be reconstructed"
        ) from exc


def _boundary_events(replay: LoadedReplay, boundary: int) -> list[dict[str, Any]]:
    arrays = replay.arrays
    offsets = arrays["legacy_left_event_boundary_offsets"]
    start, stop = int(offsets[boundary]), int(offsets[boundary + 1])
    result: list[dict[str, Any]] = []
    for index in range(start, stop):
        event: dict[str, Any] = {
            "side": str(arrays["legacy_left_event_side"][index]),
            "event": str(arrays["legacy_left_event_type"][index]),
            "time": float(arrays["legacy_left_event_onset_time_s"][index]),
            "event_time_s": float(arrays["legacy_left_event_onset_time_s"][index]),
            "confirmed_time": float(
                arrays["legacy_left_event_confirmed_time_s"][index]
            ),
            "confirmed_time_s": float(
                arrays["legacy_left_event_confirmed_time_s"][index]
            ),
            "delivered_time_s": float(
                arrays["legacy_left_event_delivered_time_s"][index]
            ),
        }
        for present_name, value_name, target_name in (
            (
                "legacy_left_event_cycle_duration_present",
                "legacy_left_event_cycle_duration_s",
                "cycle_duration_s",
            ),
            (
                "legacy_left_event_contact_duration_present",
                "legacy_left_event_contact_duration_s",
                "contact_duration_s",
            ),
        ):
            if bool(arrays[present_name][index]):
                event[target_name] = float(arrays[value_name][index])
        if bool(arrays["legacy_left_event_startup_contact_present"][index]):
            event["startup_partial_stance"] = bool(
                arrays["legacy_left_event_startup_contact"][index]
            )
        result.append(event)
    return result


def _boundary_info(replay: LoadedReplay, boundary: int) -> dict[str, Any]:
    arrays = replay.arrays
    reset = boundary == 0
    detector: dict[str, Any] = {}
    if not reset:
        detector = {
            "left": {
                # Multiplication followed by V10's division reconstructs the
                # persisted float32 body-weight-normalized value.
                "normal_force": float(arrays["legacy_left_normal_grf_bw"][boundary])
                * float(arrays["body_weight_n"][0]),
                "in_contact": bool(arrays["legacy_left_in_contact"][boundary]),
            }
        }
    return {
        "time": float(arrays["boundary_time_s"][boundary]),
        "observation": {
            "pros_knee_angle": float(arrays["pros_knee_angle_rad"][boundary]),
            "pros_knee_angle_vel": float(arrays["pros_knee_speed_rad_s"][boundary]),
            "pros_ankle_angle": float(arrays["pros_ankle_angle_rad"][boundary]),
            "pros_ankle_angle_vel": float(arrays["pros_ankle_speed_rad_s"][boundary]),
        },
        "online_grf_detector": detector,
        "legacy_online_events": _boundary_events(replay, boundary),
    }


def replay_teacher_views(
    replay: LoadedReplay,
    *,
    phase_fsm_factory: Callable[[Mapping[str, Any], str, str], Any] | None = None,
) -> ReplayedTeacherViews:
    """Replay all N+1 boundaries; build N views and consume the final boundary."""

    factory = phase_fsm_factory or _default_phase_fsm_factory
    try:
        phase_fsm = factory(replay.config, replay.fsm_module, replay.fsm_class)
        shadow = coherent_teacher.LegacyGaitShadow(phase_fsm)
    except V12R2ObserverLabelError:
        raise
    except Exception as exc:
        raise V12R2ObserverLabelError("legacy FSM reconstruction failed") from exc
    students = np.ascontiguousarray(
        replay.arrays["actor_observations"], dtype=np.float32
    )
    teachers: list[np.ndarray] = []
    invariant_exact = 0
    changed_only = 0
    column_24_changed = 0
    body_weight = float(replay.arrays["body_weight_n"][0])
    for boundary in range(replay.n_steps):
        student = students[boundary]
        info = _boundary_info(replay, boundary)
        try:
            teacher = coherent_teacher.build_teacher_view(
                student,
                coherent_teacher.EXPECTED_ACTOR_FEATURE_NAMES,
                info,
                body_weight_n=body_weight,
                shadow=shadow,
                reset_boundary=boundary == 0,
            )
        except Exception as exc:
            raise V12R2ObserverLabelError(
                f"authoritative V10 replay failed at boundary {boundary}"
            ) from exc
        coherent_teacher.assert_coherent_pair(student, teacher)
        invariant_equal = student[list(IMMUTABLE_ACTOR_COLUMNS)].tobytes(
            order="C"
        ) == teacher[list(IMMUTABLE_ACTOR_COLUMNS)].tobytes(order="C")
        if not invariant_equal:
            raise V12R2ObserverLabelError(
                f"teacher changed an immutable column at boundary {boundary}"
            )
        invariant_exact += 1
        changed_only += 1
        column_24_changed += int(student[24].tobytes() != teacher[24].tobytes())
        teachers.append(teacher)

    # The terminal boundary has no action state and therefore no H0 query, but
    # consuming it proves that every persisted event and continuous input is a
    # valid causal continuation of the same shadow.
    try:
        shadow.consume(
            _boundary_info(replay, replay.n_steps),
            body_weight_n=body_weight,
            reset_boundary=False,
        )
    except Exception as exc:
        raise V12R2ObserverLabelError(
            "authoritative V10 replay failed at the terminal boundary"
        ) from exc
    teacher_array = np.ascontiguousarray(teachers, dtype=np.float32)
    if teacher_array.shape != (replay.n_steps, ACTOR_FEATURE_COUNT):
        raise V12R2ObserverLabelError("teacher replay result shape drifted")
    return ReplayedTeacherViews(
        student_observations=students,
        teacher_observations=teacher_array,
        replayed_boundary_count=replay.boundary_count,
        changed_only_mutable_count=changed_only,
        invariant_columns_byte_exact_count=invariant_exact,
        column_24_changed_count=column_24_changed,
    )


def _default_module_loader(path: Path) -> Any:
    from ray.rllib.core.rl_module.rl_module import RLModule

    module = RLModule.from_checkpoint(path)
    module.eval()
    return module


def _default_mean_query(module: Any, teacher_view: np.ndarray) -> np.ndarray:
    import torch
    from ray.rllib.core.columns import Columns

    actor = np.ascontiguousarray(teacher_view, dtype=np.float32)
    tensor = torch.as_tensor(actor[None, :], dtype=torch.float32)
    with torch.no_grad():
        logits = module._policy_logits({Columns.OBS: tensor}).detach().cpu().numpy()
    logits = np.ascontiguousarray(logits, dtype=np.float32)
    if logits.shape != (1, 2 * ACTION_DIM) or not np.all(np.isfinite(logits)):
        raise V12R2ObserverLabelError("H0 policy logits are malformed")
    return np.ascontiguousarray(logits[0, :ACTION_DIM], dtype=np.float32)


def _coverage_from_mapping(value: Any, *, rows: int) -> CoverageResult:
    if isinstance(value, CoverageResult):
        result = value
    elif isinstance(value, Mapping):
        distance = value.get("distance_rms_z", value.get("coverage_distance_rms_z"))
        nearest = value.get(
            "nearest_reference_index", value.get("coverage_nearest_reference_index")
        )
        ood = value.get("ood_mask", value.get("coverage_ood_mask"))
        result = CoverageResult(
            distance_rms_z=np.ascontiguousarray(distance, dtype=np.float64),
            nearest_reference_index=np.ascontiguousarray(nearest, dtype=np.int64),
            ood_mask=np.ascontiguousarray(ood, dtype=np.bool_),
            audit=(
                dict(value["audit"])
                if isinstance(value.get("audit"), Mapping)
                else {
                    key: item
                    for key, item in value.items()
                    if key
                    not in {
                        "distance_rms_z",
                        "coverage_distance_rms_z",
                        "nearest_reference_index",
                        "coverage_nearest_reference_index",
                        "ood_mask",
                        "coverage_ood_mask",
                    }
                }
            ),
        )
    else:
        raise V12R2ObserverLabelError("coverage evaluator returned an unknown value")
    if (
        result.distance_rms_z.shape != (rows,)
        or result.nearest_reference_index.shape != (rows,)
        or result.ood_mask.shape != (rows,)
        or not np.all(np.isfinite(result.distance_rms_z))
        or np.any(result.distance_rms_z < 0.0)
        or np.any(result.nearest_reference_index < 0)
    ):
        raise V12R2ObserverLabelError("coverage arrays are malformed")
    return result


def _default_coverage_evaluator(observations: np.ndarray) -> CoverageResult:
    fitter = importlib.import_module("h0_primary_split_v12r2_recovery_weighted_fitter")
    evaluator = getattr(fitter, "evaluate_observer_coverage", None)
    if not callable(evaluator):
        evaluator = getattr(fitter, "observer_coverage", None)
    if not callable(evaluator):
        raise V12R2ObserverLabelError(
            "V12R2 fitter does not expose observer coverage evaluation"
        )
    return _coverage_from_mapping(evaluator(observations), rows=len(observations))


def observer_episode_weights(
    previous_penetration_m: Any,
    reset_mask: Any,
    coverage_ood_mask: Any,
    *,
    contract_module: Any | None = None,
) -> tuple[np.ndarray, np.ndarray]:
    """Compute the fixed max(reset,recovery,coverage) weights and mass 500."""

    contract = contract_module or _contract_module()
    penetration = np.ascontiguousarray(previous_penetration_m, dtype=np.float64)
    reset = np.ascontiguousarray(reset_mask, dtype=np.bool_)
    ood = np.ascontiguousarray(coverage_ood_mask, dtype=np.bool_)
    if (
        penetration.ndim != 1
        or reset.shape != penetration.shape
        or ood.shape != penetration.shape
        or not np.all(np.isfinite(penetration))
        or np.any(penetration < 0.0)
        or np.count_nonzero(reset) != 1
        or not bool(reset[0])
    ):
        raise V12R2ObserverLabelError("observer episode weighting inputs are malformed")
    lower = float(contract.RECOVERY_WEIGHTING["nominal_upper_m"])
    upper = float(contract.RECOVERY_WEIGHTING["latch_activation_m"])

    def recovery_weight(value: float) -> float:
        if value <= lower:
            return 1.0
        if value >= upper:
            return 100.0
        return 1.0 + 99.0 * ((value - lower) / (upper - lower))

    raw = np.asarray(
        [
            max(
                100.0 if bool(reset[index]) else 1.0,
                recovery_weight(float(penetration[index])),
                100.0 if bool(ood[index]) else 1.0,
            )
            for index in range(len(penetration))
        ],
        dtype=np.float64,
    )
    target_mass = float(contract.RECOVERY_WEIGHTING["episode_target_mass"])
    total = math.fsum(float(value) for value in raw)
    normalized = np.asarray(
        [float(value) * target_mass / total for value in raw], dtype=np.float64
    )
    normalized[-1] = target_mass - math.fsum(float(value) for value in normalized[:-1])
    for _ in range(4):
        observed = math.fsum(float(value) for value in normalized)
        if observed == target_mass:
            break
        direction = math.inf if observed < target_mass else -math.inf
        normalized[-1] = np.nextafter(normalized[-1], direction)
    if (
        not np.all(np.isfinite(raw))
        or np.any(raw <= 0.0)
        or not np.all(np.isfinite(normalized))
        or np.any(normalized <= 0.0)
        or math.fsum(float(value) for value in normalized) != target_mass
    ):
        raise V12R2ObserverLabelError("observer episode weights are malformed")
    return np.ascontiguousarray(raw), np.ascontiguousarray(normalized)


def label_closed_probe_in_memory(
    replay_path: str | Path,
    *,
    probe_closure_validator: Callable[[LoadedReplay], None],
    source_h0_path: str | Path,
    case_id: str,
    probe_stage: str,
    module_loader: Callable[[Path], Any] | None = None,
    mean_query: Callable[[Any, np.ndarray], Any] | None = None,
    coverage_evaluator: Callable[[np.ndarray], Any] | None = None,
    phase_fsm_factory: Callable[[Mapping[str, Any], str, str], Any] | None = None,
    contract_module: Any | None = None,
) -> ObserverLabelResult:
    """Validate closure, replay offline, then load/query H0 exactly N times."""

    if not callable(probe_closure_validator):
        raise V12R2ObserverLabelError("probe closure validator is required")
    contract = contract_module or _contract_module()
    if probe_stage not in tuple(contract.FIT_STAGES):
        raise V12R2ObserverLabelError(f"unknown probe stage: {probe_stage!r}")
    if not isinstance(case_id, str) or not case_id or len(case_id) > 64:
        raise V12R2ObserverLabelError("case_id must be nonempty and <=64 chars")
    replay = load_probe_replay_strict(replay_path, contract_module=contract)
    # This is the hard temporal boundary.  Neither the module loader nor any
    # inference helper is reachable before the caller proves the physical
    # probe receipt/gate/replay are closed and immutable.
    try:
        probe_closure_validator(replay)
    except Exception as exc:
        raise V12R2ObserverLabelError("pure probe is not validly closed") from exc
    teacher_views = replay_teacher_views(replay, phase_fsm_factory=phase_fsm_factory)
    loader = module_loader or _default_module_loader
    query = mean_query or _default_mean_query
    module = loader(Path(source_h0_path).expanduser().resolve())
    actions: list[np.ndarray] = []
    query_count = 0
    for index, teacher_view in enumerate(teacher_views.teacher_observations):
        try:
            mean = np.ascontiguousarray(query(module, teacher_view))
        except Exception as exc:
            raise V12R2ObserverLabelError(
                f"H0 mean query failed for replay row {index}"
            ) from exc
        if mean.dtype != np.dtype(np.float32) or mean.shape != (ACTION_DIM,):
            raise V12R2ObserverLabelError(
                "H0 mean query must return exact float32 shape (2,)"
            )
        if not np.all(np.isfinite(mean)) or np.any(np.abs(mean) > 1.0):
            raise V12R2ObserverLabelError("H0 mean is non-finite or out of bounds")
        actions.append(mean.copy())
        query_count += 1
    if query_count != replay.n_steps:
        raise V12R2ObserverLabelError("H0 teacher query count drifted")
    evaluator = coverage_evaluator or _default_coverage_evaluator
    coverage = _coverage_from_mapping(
        evaluator(teacher_views.student_observations), rows=replay.n_steps
    )
    reset_mask = np.zeros(replay.n_steps, dtype=np.bool_)
    reset_mask[0] = True
    raw_weights, normalized_weights = observer_episode_weights(
        replay.arrays["previous_penetration_m"],
        reset_mask,
        coverage.ood_mask,
        contract_module=contract,
    )
    step_indices = np.arange(1, replay.n_steps + 1, dtype=np.int64)
    arrays: dict[str, np.ndarray] = {
        "observations": teacher_views.student_observations.copy(),
        "actions": np.ascontiguousarray(actions, dtype=np.float32),
        "reset_mask": reset_mask,
        "previous_penetration_m": np.ascontiguousarray(
            replay.arrays["previous_penetration_m"], dtype=np.float64
        ),
        "coverage_distance_rms_z": coverage.distance_rms_z.copy(),
        "coverage_nearest_reference_index": coverage.nearest_reference_index.copy(),
        "coverage_ood_mask": coverage.ood_mask.copy(),
        "raw_sample_weights": raw_weights,
        "normalized_sample_weights": normalized_weights,
        "actor_feature_names": np.asarray(
            coherent_teacher.EXPECTED_ACTOR_FEATURE_NAMES, dtype="U64"
        ),
        "case_ids": np.full(replay.n_steps, case_id, dtype="U64"),
        "step_indices": step_indices,
        "tranche_ids": np.full(
            replay.n_steps, f"observer_probe_{probe_stage}", dtype="U64"
        ),
        "origins": np.asarray(
            [
                f"pure_observer:{probe_stage}:{case_id}:{step}"
                for step in step_indices.tolist()
            ],
            dtype="U160",
        ),
    }
    validate_observer_label_arrays(arrays, rows=replay.n_steps)
    return ObserverLabelResult(
        arrays=arrays,
        replay=replay,
        teacher_views=teacher_views,
        coverage=coverage,
        teacher_query_count=query_count,
    )


def validate_observer_label_arrays(
    arrays: Mapping[str, np.ndarray], *, rows: int
) -> None:
    if set(arrays) != set(LABEL_ARRAY_DTYPES):
        raise V12R2ObserverLabelError("observer label corpus array schema drifted")
    for name, dtype in LABEL_ARRAY_DTYPES.items():
        array = np.asarray(arrays[name])
        if not _dtype_matches(array, dtype):
            raise V12R2ObserverLabelError(f"observer label {name} dtype drifted")
    expected_shapes = {
        "observations": (rows, ACTOR_FEATURE_COUNT),
        "actions": (rows, ACTION_DIM),
        "reset_mask": (rows,),
        "previous_penetration_m": (rows,),
        "coverage_distance_rms_z": (rows,),
        "coverage_nearest_reference_index": (rows,),
        "coverage_ood_mask": (rows,),
        "raw_sample_weights": (rows,),
        "normalized_sample_weights": (rows,),
        "actor_feature_names": (ACTOR_FEATURE_COUNT,),
        "case_ids": (rows,),
        "step_indices": (rows,),
        "tranche_ids": (rows,),
        "origins": (rows,),
    }
    if any(
        np.asarray(arrays[name]).shape != shape
        for name, shape in expected_shapes.items()
    ):
        raise V12R2ObserverLabelError("observer label corpus shape drifted")
    for name in (
        "observations",
        "actions",
        "previous_penetration_m",
        "coverage_distance_rms_z",
        "raw_sample_weights",
        "normalized_sample_weights",
    ):
        if not np.all(np.isfinite(arrays[name])):
            raise V12R2ObserverLabelError(f"observer label {name} is non-finite")
    if tuple(arrays["actor_feature_names"].astype(str).tolist()) != tuple(
        coherent_teacher.EXPECTED_ACTOR_FEATURE_NAMES
    ):
        raise V12R2ObserverLabelError("observer actor feature layout drifted")
    if np.count_nonzero(arrays["reset_mask"]) != 1 or not bool(arrays["reset_mask"][0]):
        raise V12R2ObserverLabelError("observer reset mask drifted")
    if not np.array_equal(arrays["step_indices"], np.arange(1, rows + 1)):
        raise V12R2ObserverLabelError("observer step indices drifted")


def _resolve_repo_path(value: str | PurePath | Path) -> Path:
    path = Path(value)
    return (path if path.is_absolute() else REPO_ROOT / path).resolve()


def _artifact(path: str | Path) -> dict[str, Any]:
    try:
        return forensic.artifact_record(
            _resolve_repo_path(path), artifact_root=REPO_ROOT
        )
    except Exception as exc:
        raise V12R2ObserverLabelError(f"cannot bind artifact: {path}") from exc


def _tree_record(path: str | Path) -> dict[str, Any]:
    try:
        import h0_primary_split_v10s_fit as v10s_fit

        return v10s_fit._tree_record(_resolve_repo_path(path))
    except Exception as exc:
        raise V12R2ObserverLabelError(f"cannot bind artifact tree: {path}") from exc


def _default_probe_closure_validator(
    *,
    contract: Any,
    stage: str,
    probe_evidence: Mapping[str, Any],
    replay_path: Path,
) -> Callable[[LoadedReplay], None]:
    def validate(replay: LoadedReplay) -> None:
        probe_root = _resolve_repo_path(contract.PROBE_ROOT / stage)
        current_replay = _artifact(replay_path)
        current_receipt = _artifact(contract.PROBE_RECEIPT_PATHS[stage])
        current_gate_artifact = _artifact(probe_root / "gate.json")
        gate_payload = _strict_json_mapping(
            probe_root / "gate.json", label="pure probe gate"
        )
        if (
            not isinstance(probe_evidence, Mapping)
            or probe_evidence.get("protocol_id") != contract.PROTOCOL_ID
            or probe_evidence.get("fit_stage") != stage
            or probe_evidence.get("integrity_passed") is not True
            or type(probe_evidence.get("passed")) is not bool
            or probe_evidence.get("probe_step_count") != replay.n_steps
            or probe_evidence.get("replay_schema") != contract.PROBE_REPLAY_SCHEMA
            or (
                probe_evidence.get("passed") is not True
                and not (
                    stage != "p3"
                    and probe_evidence.get("recoverable_for_data_collection") is True
                )
            )
            or probe_evidence.get("replay_payload") != current_replay
            or probe_evidence.get("receipt") != current_receipt
            or probe_evidence.get("gate_artifact") != current_gate_artifact
            or gate_payload.get("protocol_id") != contract.PROTOCOL_ID
            or gate_payload.get("fit_stage") != stage
            or gate_payload.get("integrity_passed") is not True
            or gate_payload.get("passed") is not probe_evidence.get("passed")
            or gate_payload.get("probe_step_count") != replay.n_steps
        ):
            raise V12R2ObserverLabelError("probe evidence does not close this replay")
        if replay.event_contract_id != contract.EVENT_CONTRACT_ID:
            raise V12R2ObserverLabelError("probe replay event contract drifted")

    return validate


def run_observer_label_stage(
    *,
    stage: str,
    probe_evidence: Mapping[str, Any],
    pipeline_claim_path: str | Path,
    worker_claim_path: str | Path,
    replay_path: str | Path | None = None,
    case_id: str | None = None,
    output_dir: str | Path | None = None,
    source_h0_path: str | Path | None = None,
    module_loader: Callable[[Path], Any] | None = None,
    mean_query: Callable[[Any, np.ndarray], Any] | None = None,
    coverage_evaluator: Callable[[np.ndarray], Any] | None = None,
    phase_fsm_factory: Callable[[Mapping[str, Any], str, str], Any] | None = None,
    contract_module: Any | None = None,
    enforce_canonical_destination: bool = True,
) -> dict[str, Any]:
    """Publish one exact observer-only corpus, summary, gate, and receipt."""

    contract = contract_module or _contract_module()
    if stage not in tuple(contract.FIT_STAGES):
        raise V12R2ObserverLabelError(f"unknown label stage: {stage!r}")
    replay_source = _resolve_repo_path(
        contract.PROBE_ROOT / stage / "replay_boundaries.npz"
        if replay_path is None
        else replay_path
    )
    destination = _resolve_repo_path(
        contract.LABEL_ROOT / stage if output_dir is None else output_dir
    )
    resolved_case_id = (
        str(contract.PROBE_CASE["case_id"]) if case_id is None else case_id
    )
    canonical_destination = _resolve_repo_path(contract.LABEL_ROOT / stage)
    if enforce_canonical_destination and destination != canonical_destination:
        raise V12R2ObserverLabelError("observer label destination is not canonical")
    if os.path.lexists(destination):
        raise V12R2ObserverLabelError(
            f"observer label destination occupied/no-clobber: {destination}"
        )
    source_h0 = _resolve_repo_path(
        contract.SOURCE_H0_MODULE_PATH if source_h0_path is None else source_h0_path
    )
    closure = _default_probe_closure_validator(
        contract=contract,
        stage=stage,
        probe_evidence=probe_evidence,
        replay_path=replay_source,
    )
    result = label_closed_probe_in_memory(
        replay_source,
        probe_closure_validator=closure,
        source_h0_path=source_h0,
        case_id=resolved_case_id,
        probe_stage=stage,
        module_loader=module_loader,
        mean_query=mean_query,
        coverage_evaluator=coverage_evaluator,
        phase_fsm_factory=phase_fsm_factory,
        contract_module=contract,
    )
    destination.mkdir(parents=True, exist_ok=False)
    corpus_path = write_npz_exclusive(
        destination / "observer_labels.npz", result.arrays
    )
    coverage_audit = dict(result.coverage.audit)
    coverage = contract.COVERAGE_WEIGHTING
    normalized_mass = math.fsum(result.arrays["normalized_sample_weights"].tolist())
    summary = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": "H0_PRIMARY_SPLIT_V12R2_OBSERVER_LABELS_COMPLETE_UNGATED",
        "protocol_id": contract.PROTOCOL_ID,
        "candidate_fit_stage": stage,
        "probe_passed": probe_evidence.get("passed"),
        "probe_integrity_passed": probe_evidence.get("integrity_passed"),
        "probe_step_count": result.replay.n_steps,
        "probe_candidate_id": probe_evidence.get("candidate_id"),
        "probe_candidate_module_tree_sha256": probe_evidence.get(
            "candidate_module_tree_sha256"
        ),
        "probe_trace": probe_evidence.get("trace"),
        "probe_replay_payload": probe_evidence.get("replay_payload"),
        "probe_receipt": probe_evidence.get("receipt"),
        "probe_gate_artifact": probe_evidence.get("gate_artifact"),
        "probe_replay_schema": contract.PROBE_REPLAY_SCHEMA,
        "labelled_row_count": result.replay.n_steps,
        "teacher_query_count": result.teacher_query_count,
        "same_state_teacher_label_count": result.teacher_query_count,
        "persisted_label_count": result.replay.n_steps,
        "replayed_boundary_count": result.teacher_views.replayed_boundary_count,
        "replay_payload_missing_count": 0,
        "teacher_view_changes_only_10_24_count": (
            result.teacher_views.changed_only_mutable_count
        ),
        "invariant_columns_byte_exact_count": (
            result.teacher_views.invariant_columns_byte_exact_count
        ),
        "teacher_column_24_mutable": True,
        "teacher_column_24_changed_count": result.teacher_views.column_24_changed_count,
        "coverage_distance_count": result.replay.n_steps,
        "coverage_reference_observations_sha256": coverage[
            "reference_observations_sha256"
        ],
        "coverage_normalization_mean_sha256": coverage["normalization_mean_sha256"],
        "coverage_normalization_std_sha256": coverage["normalization_std_sha256"],
        "coverage_reference_features_sha256": coverage[
            "normalized_feature_matrix_sha256"
        ],
        "coverage_loo_p95": coverage["loo_p95"],
        "coverage_new_row_query": coverage["new_row_query"],
        "coverage_ood_row_count": int(np.count_nonzero(result.coverage.ood_mask)),
        "coverage_audit": coverage_audit,
        "previous_penetration_metadata_count": result.replay.n_steps,
        "raw_sample_weight_count": result.replay.n_steps,
        "normalized_sample_weight_count": result.replay.n_steps,
        "reset_row_count": 1,
        "recovery_weighting": contract.RECOVERY_WEIGHTING,
        "normalized_episode_mass": (
            float(contract.RECOVERY_WEIGHTING["episode_target_mass"])
            if math.isclose(
                normalized_mass,
                float(contract.RECOVERY_WEIGHTING["episode_target_mass"]),
                rel_tol=0.0,
                abs_tol=1.0e-12,
            )
            else normalized_mass
        ),
        "label_corpus": _artifact(corpus_path),
        "worker_claim": _artifact(worker_claim_path),
        "teacher_id": contract.TEACHER_ID,
        "teacher_evidence_id": contract.TEACHER_EVIDENCE_ID,
        "teacher_evidence_receipt": contract.TEACHER_EVIDENCE_ARTIFACT,
        "source_h0": _tree_record(source_h0),
        "environment_reset_calls": 0,
        "environment_step_calls": 0,
        "action_served_count": 0,
        "teacher_loaded_after_probe_closed": True,
        "actor_updates": 0,
        "critic_updates": 0,
        "ppo_updates": 0,
        "protected_trials_opened": [],
        "reserve_trials_opened": [],
    }
    summary_path = forensic.write_json_exclusive(destination / "summary.json", summary)
    gate = dict(
        contract.observer_label_gate(
            summary, stage=stage, probe_evidence=probe_evidence
        )
    )
    gate_path = forensic.write_json_exclusive(destination / "gate.json", gate)
    if gate.get("passed") is not True:
        raise V12R2ObserverLabelError(
            f"observer label gate failed: {_artifact(gate_path)}"
        )
    receipt = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.OBSERVER_LABEL_PASS_STATUS,
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "fit_stage": stage,
        "labelled_row_count": result.replay.n_steps,
        "label_corpus": _artifact(corpus_path),
        "pipeline_claim": _artifact(pipeline_claim_path),
        "worker_claim": _artifact(worker_claim_path),
    }
    receipt_path = forensic.write_json_exclusive(destination / "receipt.json", receipt)
    return {
        **receipt,
        "receipt": _artifact(receipt_path),
        "summary": _artifact(summary_path),
        "gate": _artifact(gate_path),
        "environment_reset_calls": 0,
        "environment_step_calls": 0,
        "action_served_count": 0,
    }


def _strict_json_mapping(path: Path, *, label: str) -> dict[str, Any]:
    try:
        value = forensic.strict_json_load(path)
    except Exception as exc:
        raise V12R2ObserverLabelError(f"{label} is not strict JSON") from exc
    if not isinstance(value, Mapping):
        raise V12R2ObserverLabelError(f"{label} must be an object")
    return dict(value)


def verify_observer_label_stage(
    stage: str,
    *,
    contract_module: Any | None = None,
) -> dict[str, Any]:
    """Recompute the canonical label gate, corpus invariants, and receipt.

    Verification is observer-only and issues no policy or environment call.  It
    binds every referenced artifact to its current bytes and re-evaluates the
    pure contract gate from the persisted summary and reconstructed probe
    evidence.
    """

    contract = contract_module or _contract_module()
    if stage not in tuple(contract.FIT_STAGES):
        raise V12R2ObserverLabelError(f"unknown label stage: {stage!r}")
    root = _resolve_repo_path(contract.LABEL_ROOT / stage)
    corpus_path = root / "observer_labels.npz"
    summary_path = root / "summary.json"
    gate_path = root / "gate.json"
    receipt_path = root / "receipt.json"
    summary = _strict_json_mapping(summary_path, label="observer label summary")
    gate = _strict_json_mapping(gate_path, label="observer label gate")
    receipt = _strict_json_mapping(receipt_path, label="observer label receipt")
    try:
        with np.load(corpus_path, allow_pickle=False) as archive:
            if set(archive.files) != set(LABEL_ARRAY_DTYPES):
                raise V12R2ObserverLabelError("observer corpus NPZ schema drifted")
            arrays = {
                name: np.ascontiguousarray(archive[name].copy())
                for name in archive.files
            }
    except V12R2ObserverLabelError:
        raise
    except Exception as exc:
        raise V12R2ObserverLabelError(
            "observer corpus is not a pickle-free NPZ"
        ) from exc
    rows = summary.get("labelled_row_count")
    if type(rows) is not int or rows < 1:
        raise V12R2ObserverLabelError("observer labelled row count is malformed")
    validate_observer_label_arrays(arrays, rows=rows)
    replay = load_probe_replay_strict(
        _resolve_repo_path(contract.PROBE_ROOT / stage / "replay_boundaries.npz"),
        contract_module=contract,
    )
    expected_steps = np.arange(1, rows + 1, dtype=np.int64)
    expected_case = str(contract.PROBE_CASE["case_id"])
    replay_bindings_exact = (
        replay.n_steps == rows
        and arrays["observations"].tobytes(order="C")
        == replay.arrays["actor_observations"].tobytes(order="C")
        and arrays["previous_penetration_m"].tobytes(order="C")
        == replay.arrays["previous_penetration_m"].tobytes(order="C")
        and np.array_equal(
            arrays["reset_mask"],
            np.asarray([index == 0 for index in range(rows)], dtype=np.bool_),
        )
        and np.array_equal(arrays["step_indices"], expected_steps)
        and set(arrays["case_ids"].astype(str).tolist()) == {expected_case}
        and set(arrays["tranche_ids"].astype(str).tolist())
        == {f"observer_probe_{stage}"}
        and arrays["origins"].astype(str).tolist()
        == [
            f"pure_observer:{stage}:{expected_case}:{step}"
            for step in expected_steps.tolist()
        ]
    )
    if not replay_bindings_exact:
        raise V12R2ObserverLabelError(
            "observer corpus no longer matches the closed probe replay"
        )
    expected_ood = arrays["coverage_distance_rms_z"] > float(
        contract.COVERAGE_WEIGHTING["loo_p95"]
    )
    if expected_ood.tobytes(order="C") != arrays["coverage_ood_mask"].tobytes(
        order="C"
    ):
        raise V12R2ObserverLabelError("observer coverage OOD mask drifted")
    expected_raw, expected_normalized = observer_episode_weights(
        arrays["previous_penetration_m"],
        arrays["reset_mask"],
        arrays["coverage_ood_mask"],
        contract_module=contract,
    )
    if expected_raw.tobytes(order="C") != arrays["raw_sample_weights"].tobytes(
        order="C"
    ) or expected_normalized.tobytes(order="C") != arrays[
        "normalized_sample_weights"
    ].tobytes(
        order="C"
    ):
        raise V12R2ObserverLabelError("observer sample weights drifted")

    probe_evidence = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": (
            contract.PURE_PROBE_PASS_STATUS
            if summary.get("probe_passed") is True
            else contract.PURE_PROBE_FAIL_STATUS
        ),
        "passed": summary.get("probe_passed"),
        "integrity_passed": summary.get("probe_integrity_passed"),
        "recoverable_for_data_collection": (
            summary.get("probe_passed") is False
            and summary.get("probe_integrity_passed") is True
            and stage != "p3"
        ),
        "protocol_id": contract.PROTOCOL_ID,
        "fit_stage": stage,
        "probe_step_count": summary.get("probe_step_count"),
        "candidate_id": summary.get("probe_candidate_id"),
        "candidate_module_tree_sha256": summary.get(
            "probe_candidate_module_tree_sha256"
        ),
        "trace": summary.get("probe_trace"),
        "replay_payload": summary.get("probe_replay_payload"),
        "replay_schema": summary.get("probe_replay_schema"),
        "candidate_promoted": False,
        "receipt": summary.get("probe_receipt"),
        "gate_artifact": summary.get("probe_gate_artifact"),
    }
    # The observer gate consumes only this stable evidence subset; ``next_stage``
    # and the detailed probe checks are deliberately irrelevant here.
    expected_gate = dict(
        contract.observer_label_gate(
            summary, stage=stage, probe_evidence=probe_evidence
        )
    )
    if (
        forensic.canonical_json_bytes(gate)
        != forensic.canonical_json_bytes(expected_gate)
        or gate.get("passed") is not True
    ):
        raise V12R2ObserverLabelError("observer label gate recomputation failed")

    pipeline_record = receipt.get("pipeline_claim")
    worker_record = receipt.get("worker_claim")
    expected_receipt = {
        "schema_version": contract.SCHEMA_VERSION,
        "status": contract.OBSERVER_LABEL_PASS_STATUS,
        "passed": True,
        "protocol_id": contract.PROTOCOL_ID,
        "fit_stage": stage,
        "labelled_row_count": rows,
        "label_corpus": _artifact(corpus_path),
        "pipeline_claim": _artifact(contract.PIPELINE_CLAIM_PATH),
        "worker_claim": _artifact(contract.worker_claim_path(f"label_{stage}")),
    }
    if forensic.canonical_json_bytes(receipt) != forensic.canonical_json_bytes(
        expected_receipt
    ):
        raise V12R2ObserverLabelError("minimal observer label receipt drifted")
    current_bindings = (
        summary.get("label_corpus") == expected_receipt["label_corpus"]
        and summary.get("worker_claim") == worker_record
        and pipeline_record == expected_receipt["pipeline_claim"]
        and summary.get("probe_trace")
        == _artifact(contract.PROBE_ROOT / stage / "trace.json")
        and summary.get("probe_replay_payload")
        == _artifact(contract.PROBE_ROOT / stage / "replay_boundaries.npz")
        and summary.get("probe_receipt")
        == _artifact(contract.PROBE_RECEIPT_PATHS[stage])
        and summary.get("probe_gate_artifact")
        == _artifact(contract.PROBE_ROOT / stage / "gate.json")
        and summary.get("source_h0") == _tree_record(contract.SOURCE_H0_MODULE_PATH)
    )
    if not current_bindings:
        raise V12R2ObserverLabelError("observer label artifact binding drifted")
    return {
        **receipt,
        "receipt": _artifact(receipt_path),
        "summary": _artifact(summary_path),
        "gate": _artifact(gate_path),
        "verified": True,
        "environment_reset_calls": 0,
        "environment_step_calls": 0,
        "action_served_count": 0,
    }


__all__ = [
    "ACTOR_FEATURE_COUNT",
    "ACTION_DIM",
    "BOUNDARY_ARRAY_DTYPES",
    "CONFIG_ARRAY_DTYPES",
    "CoverageResult",
    "EVENT_ARRAY_DTYPES",
    "IMMUTABLE_ACTOR_COLUMNS",
    "LABEL_ARRAY_DTYPES",
    "LABEL_CORPUS_SCHEMA_ID",
    "LoadedReplay",
    "MUTABLE_ACTOR_COLUMNS",
    "ObserverLabelResult",
    "ProbeReplayAccumulator",
    "PureProbeReplayRecorder",
    "REPLAY_ARRAY_NAMES",
    "REPLAY_SCHEMA_ID",
    "ReplayedTeacherViews",
    "SCALAR_ARRAY_DTYPES",
    "STEP_ARRAY_DTYPES",
    "V12R2ObserverLabelError",
    "canonical_legacy_fsm_config",
    "label_closed_probe_in_memory",
    "load_probe_replay_strict",
    "observer_episode_weights",
    "replay_teacher_views",
    "run_observer_label_stage",
    "validate_observer_label_arrays",
    "verify_observer_label_stage",
    "write_npz_exclusive",
    "write_replay_npz_exclusive",
]
