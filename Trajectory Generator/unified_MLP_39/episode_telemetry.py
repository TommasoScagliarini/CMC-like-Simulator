"""Opt-in, bounded episode diagnostics; never changes the sampled transition.

RLlib 2.55 keeps ``SingleAgentEpisode.id_`` and cumulative ``t`` across cuts.
The observer lives on the callback, rather than on sampled episode/custom_data,
so lookback buffers and learner batches never contain its ring buffer. Only
already-recorded infos, observations, actions and model outputs are read.
"""
from __future__ import annotations

from collections import OrderedDict, deque
from collections.abc import Mapping
from dataclasses import dataclass, field
import json
import math
import os
from pathlib import Path
import sys
import time
from typing import Any

ENV_VAR = "UNIFIED39_EPISODE_TELEMETRY_DIR"
METRIC_PREFIX = "episode_telemetry"
TAIL_STEPS = 50
MAX_ENV_SLOTS = 128
JOINTS = ("pros_knee_angle", "pros_ankle_angle")
FSM_KEYS = (
    "state_id", "state_name", "expected_next_event", "stance_elapsed_s",
    "swing_elapsed_s", "timeout_exceeded", "timeout_side", "invalid_event_this_step",
    "invalid_event_type", "invalid_event_count", "resync_event_this_step",
    "resync_count", "hs_cancelled_count", "cycle_degraded", "valid_cycle_count",
    "valid_hs_count", "valid_to_count", "fsm_behaviour_version",
)
TERM_KEYS = (
    "segment_completed", "grf_hard_contact_available",
    "grf_penetration_m", "grf_penetration_loss", "grf_ankle_moment_flip_loss",
    "grf_ankle_moment_flip_available",
    "grf_ankle_moment_flip_excess_nm", "grf_ankle_moment_nm",
    "grf_ankle_moment_flip_tau_nm", "grf_ankle_moment_flip_force_n",
    "grf_ankle_moment_flip_cop_z_minus_ankle_z_m",
    "prosthetic_normal_force_bw", "safety_loss", "sound_imitation_loss",
    "served_imitation_loss", "tracking_loss", "command_rate_loss",
    "policy_action_clip_fraction", "policy_action_clip_abs_max",
    "phase_timeout_exceeded", "phase_timeout_side",
)
COMPONENT_KEYS = (
    "reward", "reward_base", "penalty", "imitation_score", "served_imitation_score",
    "grf_ankle_moment_flip_term", "grf_penetration_term", "safety_term",
)
JOINT_SUFFIXES = ("", "_vel", "_served_ref", "_served_ref_vel", "_served_ref_accel",
                  "_previous_endpoint", "_sea_u")
TERMINAL_CONTACT_KEYS = (
    "available", "unavailable_reason", "source", "time_s", "side", "penetration_m", "limit_m", "online_grf",
)


def _number(value: Any) -> float | None:
    try:
        result = float(value)
        return result if math.isfinite(result) else None
    except (TypeError, ValueError, OverflowError):
        return None


def _copy_json(value: Any) -> Any:
    """Copy only selected, small payloads; nonfinite diagnostics become null."""
    if value is None or isinstance(value, (str, bool)):
        return value
    if isinstance(value, Mapping):
        return {str(k): _copy_json(v) for k, v in value.items()}
    if hasattr(value, "tolist"):
        return _copy_json(value.tolist())
    if isinstance(value, (list, tuple)):
        return [_copy_json(v) for v in value]
    return _number(value)


def _select(source: Any, keys) -> dict:
    if not isinstance(source, Mapping):
        return {}
    return {key: _copy_json(source[key]) for key in keys if key in source}


@dataclass
class _EpisodeState:
    episode_id: str
    worker_index: int
    env_index: int
    first_t: int
    last_t: int = 0
    start_time_s: float | None = None
    observed_steps: int = 0
    missing_steps: int = 0
    missing_rows: int = 0
    distribution_rows: int = 0
    completed_segment_rows: int = 0
    partial_segment_rows: int = 0
    unknown_segment_rows: int = 0
    reward_sum: float = 0.0
    peaks: dict = field(default_factory=dict)
    event_counts: dict = field(default_factory=lambda: {
        "resync_rows": 0, "invalid_event_rows": 0, "action_clip_rows": 0,
        "moment_penalty_rows": 0, "penetration_penalty_rows": 0,
    })
    last_resync: dict | None = None
    last_hs_cancel: dict | None = None
    hs_cancel_count: float = 0.0
    joint_errors: dict = field(default_factory=dict)
    policy_versions: dict = field(default_factory=lambda: {
        "first": None, "last": None, "min": None, "max": None,
        "switch_count": 0, "rows_with_version": 0, "rows_without_version": 0,
    })
    tail: deque = field(default_factory=lambda: deque(maxlen=TAIL_STEPS))


class EpisodeTelemetry:
    """One observer per EnvRunner callback, with at most one ring per env slot."""

    def __init__(self, directory: str):
        self.directory = Path(directory)
        self.path_error = None if self.directory.is_absolute() else "telemetry directory must be absolute"
        self.process_id = os.getpid()
        # No uuid/random calls: wall-clock identity does not consume any RNG.
        self.writer_id = f"p{self.process_id}_{time.time_ns()}"
        self._slots: OrderedDict[tuple[int, int], _EpisodeState] = OrderedDict()
        self._last_closed: OrderedDict[tuple[int, int], str] = OrderedDict()
        self._warned: set[str] = set()
        self._file_part = 0
        self.metrics: dict[str, int] = {}

    def _metric(self, key: str, logger, value: int = 1) -> None:
        self.metrics[key] = self.metrics.get(key, 0) + value
        if logger is not None:
            try:
                logger.log_value(f"{METRIC_PREFIX}/{key}", float(value), reduce="sum")
            except Exception as error:
                self._warn("metrics", error)

    def _warn(self, category: str, error: Any) -> None:
        if category in self._warned:
            return
        self._warned.add(category)
        try:
            print(f"[episode_telemetry] {category}: {error}; diagnostic data may be missing; training continues.",
                  file=sys.stderr, flush=True)
        except Exception:
            pass

    @staticmethod
    def _identity(episode, env_runner, env_index) -> tuple[tuple[int, int], str, int]:
        slot = (int(getattr(env_runner, "worker_index", 0) or 0), int(env_index or 0))
        return slot, str(episode.id_), int(episode.t)

    def _state(self, episode, env_runner, env_index, logger) -> _EpisodeState | None:
        slot, episode_id, timestep = self._identity(episode, env_runner, env_index)
        if self._last_closed.get(slot) == episode_id:
            self._metric("duplicate_steps", logger)
            return None
        current = self._slots.get(slot)
        if current is None or current.episode_id != episode_id:
            if current is not None:
                self._metric("abandoned_episodes", logger)
                self._warn("abandoned", "env reset/replacement before on_episode_end")
            current = _EpisodeState(episode_id, *slot, first_t=timestep,
                                    missing_steps=max(0, timestep - 1))
            self._slots[slot] = current
            if len(self._slots) > MAX_ENV_SLOTS:
                self._slots.popitem(last=False)
                self._metric("evicted_episodes", logger)
                self._warn("capacity", "bounded env-slot capacity exceeded")
        self._slots.move_to_end(slot)
        return current

    def observe(self, episode, *, env_runner=None, env_index=0, metrics_logger=None) -> None:
        """One sample, after env.step. Duplicated callback/cut samples are ignored."""
        try:
            state = self._state(episode, env_runner, env_index, metrics_logger)
            if state is None:
                return
            timestep = int(episode.t)
            if timestep <= state.last_t:
                self._metric("duplicate_steps", metrics_logger)
                return
            row = self._row(episode, env_runner)
            if state.last_t:
                state.missing_steps += max(0, timestep - state.last_t - 1)
            state.last_t = timestep
            if state.observed_steps == 0 and timestep == 1:
                state.start_time_s = row["before_time_s"]
            state.observed_steps += 1
            state.missing_rows += bool(row["missing_fields"])
            state.distribution_rows += bool(row["policy_outputs"].get("action_dist_inputs") is not None)
            state.tail.append(row)
            self._accumulate(state, row)
            self._metric("observed_steps", metrics_logger)
            if row["missing_fields"]:
                self._metric("missing_rows", metrics_logger)
            if row["policy_outputs"].get("action_dist_inputs") is not None:
                self._metric("policy_distribution_rows", metrics_logger)
        except Exception as error:
            self._metric("step_errors", metrics_logger)
            self._metric("missing_rows", metrics_logger)
            self._warn("step", error)

    @staticmethod
    def _row(episode, env_runner) -> dict:
        info = episode.get_infos(-1)
        if not isinstance(info, Mapping):
            raise ValueError("last info is not a mapping")
        observation = info.get("observation", {})
        terms = info.get("reward_terms", {})
        fsm = _select(info.get("phase_fsm"), FSM_KEYS)
        raw_action = _copy_json(episode.get_actions(-1))
        # -2 is the observation preceding the just-recorded action, including
        # the first sample after cut: RLlib retains that boundary observation.
        before = episode.get_observations(-2)
        actor_before = _copy_json(before[:39])
        outputs = {}
        available = getattr(episode, "extra_model_outputs", {})
        for key in ("action_dist_inputs", "action_logp", "weights_seq_no"):
            if key in available:
                outputs[key] = _copy_json(episode.get_extra_model_outputs(key, -1))
        if "weights_seq_no" not in outputs:
            outputs["weights_seq_no"] = _number(getattr(env_runner, "_weights_seq_no", None))
        sides = info.get("online_gait", {}).get("sides", {})
        completed = _number(info.get("segment_completed", terms.get("segment_completed")))
        row = {
            "episode_step": int(episode.t),
            "before_time_s": _number(episode.get_infos(-2).get("time")),
            "time_s": _number(info.get("time")),
            "target_time_s": _number(info.get("target_time")),
            # A hard guard can interrupt a 10 ms segment at an integration
            # substep. Keep the policy boundary time and contact time separate.
            # Missing legacy status remains unknown rather than inventing a
            # completed or partial segment for archived infos.
            "segment_completed": bool(completed) if completed in (0.0, 1.0) else None,
            "terminal_contact": _select(info.get("terminal_contact"), TERMINAL_CONTACT_KEYS),
            "episode_start_offset_s": _number(info.get("episode_start_offset_s")),
            "raw_policy_action": raw_action,
            "policy_outputs": outputs,
            "actor_observation_before": actor_before,
            "joints": _select(observation, [j + suffix for j in JOINTS for suffix in JOINT_SUFFIXES]),
            "imitation_target": _select(info.get("imitation_target"), ("q", "qdot", "phase", "source")),
            "phase_fsm": fsm,
            "left_contact": _select(sides.get("left"), ("normal_force_n", "normal_force_bw", "in_contact", "gait_phase")),
            "reward_terms": _select(terms, TERM_KEYS),
            "reward_components": _select(info.get("reward_components"), COMPONENT_KEYS),
            "reference_governor": _select(info.get("reference_governor_diagnostics"),
                ("velocity_limit_fraction", "acceleration_limit_fraction", "jerk_limit_fraction")),
            "policy_segment_values": _copy_json(info.get("policy_segment_values")),
            # Vectorized RLlib infos can contain np.bool_ instead of bool.
            "target_visible": _copy_json(info.get("target_availability", {}).get("visible")),
            "reward": _number(episode.get_rewards(-1)),
            "end_reason": info.get("end_reason"),
        }
        row["missing_fields"] = [key for key, value in (
            ("time_s", row["time_s"]), ("penetration", _number(terms.get("grf_penetration_m"))),
            ("fsm_state", fsm.get("state_id")), ("resync_count", fsm.get("resync_count")),
            ("target_q", row["imitation_target"].get("q")), ("reward", row["reward"]),
            ("actor_observation_before", actor_before if len(actor_before) == 39 else None),
        ) if value is None]
        terminal = row["terminal_contact"]
        if terminal.get("available") and terminal.get("source") == "hard_penetration_guard":
            for key in ("time_s", "penetration_m"):
                if _number(terminal.get(key)) is None:
                    row["missing_fields"].append("terminal_contact_" + key)
        return row

    @staticmethod
    def _accumulate(state: _EpisodeState, row: dict) -> None:
        if row["reward"] is not None:
            state.reward_sum += row["reward"]
        terms, fsm = row["reward_terms"], row["phase_fsm"]
        completed = row.get("segment_completed")
        if completed is True:
            state.completed_segment_rows += 1
        elif completed is False:
            state.partial_segment_rows += 1
        else:
            state.unknown_segment_rows += 1
        terminal = row.get("terminal_contact") or {}
        hard_contact = bool(terminal.get("available")) and terminal.get("source") == "hard_penetration_guard"
        snapshot = terminal.get("online_grf") or {}
        snapshot_left = snapshot.get("left") or {}
        contact_available = completed is not False or hard_contact
        moment_available = _number(terms.get("grf_ankle_moment_flip_available"))
        # Legacy complete rows did not serialize availability. Preserve them;
        # new unavailable diagnostics and partial rows are never measurements.
        moment_available = moment_available > 0 if moment_available is not None else completed is not False
        moment_available = moment_available and contact_available
        version = _number(row["policy_outputs"].get("weights_seq_no"))
        versions = state.policy_versions
        if version is None:
            versions["rows_without_version"] += 1
        else:
            versions["rows_with_version"] += 1
            if versions["first"] is None:
                versions.update(first=version, min=version, max=version)
            elif versions["last"] != version:
                versions["switch_count"] += 1
            versions["last"] = version
            versions["min"] = min(versions["min"], version)
            versions["max"] = max(versions["max"], version)
        maxima = {
            "penetration_m": (terminal.get("penetration_m") if hard_contact else terms.get("grf_penetration_m")) if contact_available else None,
            # Use the explicitly timestamped primary snapshot for hard-contact
            # peaks; the FSM phase beside that force still belongs to the boundary.
            "normal_force_n": (snapshot_left.get("normal_force") if hard_contact else row["left_contact"].get("normal_force_n")) if contact_available else None,
            "moment_flip_loss": terms.get("grf_ankle_moment_flip_loss") if moment_available else None,
            "resync_count": fsm.get("resync_count"),
            "hs_cancelled_count": fsm.get("hs_cancelled_count"),
        }
        for name, value in maxima.items():
            value = _number(value)
            if value is not None and (name not in state.peaks or value > state.peaks[name]["value"]):
                measured_contact = name in ("penetration_m", "normal_force_n", "moment_flip_loss") and hard_contact
                state.peaks[name] = {"value": value, "episode_step": row["episode_step"],
                                     "time_s": _number(terminal.get("time_s")) if measured_contact else row["time_s"],
                                     "phase_fsm_state": fsm.get("state_name")}
                if measured_contact:
                    state.peaks[name].update(source="hard_penetration_guard",
                        segment_completed=completed, policy_boundary_time_s=row["time_s"],
                        phase_fsm_time_s=row["time_s"])
        moment = _number(terms.get("grf_ankle_moment_flip_tau_nm")) if moment_available else None
        if moment is not None and ("ankle_moment_min_nm" not in state.peaks or moment < state.peaks["ankle_moment_min_nm"]["value"]):
            state.peaks["ankle_moment_min_nm"] = {
                "value": moment, "episode_step": row["episode_step"],
                "time_s": _number(terminal.get("time_s")) if hard_contact else row["time_s"],
                "phase_fsm_state": fsm.get("state_name"),
            }
            if hard_contact:
                state.peaks["ankle_moment_min_nm"].update(source="hard_penetration_guard",
                    segment_completed=completed, policy_boundary_time_s=row["time_s"],
                    phase_fsm_time_s=row["time_s"])
        flags = {
            "resync_rows": fsm.get("resync_event_this_step"),
            "invalid_event_rows": fsm.get("invalid_event_this_step"),
            "action_clip_rows": terms.get("policy_action_clip_fraction"),
            "moment_penalty_rows": terms.get("grf_ankle_moment_flip_loss"),
            "penetration_penalty_rows": terms.get("grf_penetration_loss"),
        }
        for key, value in flags.items():
            if (_number(value) or 0) > 0:
                state.event_counts[key] += 1
        if (_number(fsm.get("resync_event_this_step")) or 0) > 0:
            state.last_resync = {"episode_step": row["episode_step"], "time_s": row["time_s"], "state_name": fsm.get("state_name")}
        cancelled = _number(fsm.get("hs_cancelled_count")) or 0
        if cancelled > state.hs_cancel_count:
            state.last_hs_cancel = {"episode_step": row["episode_step"], "time_s": row["time_s"]}
        state.hs_cancel_count = cancelled
        # Partial hard-terminal infos carry a contact snapshot, not a completed
        # policy segment. Their q/target pair must not add a kinematic sample.
        # Reward/outcome and the measured terminal penetration remain counted.
        if completed is False:
            return
        target_q = row["imitation_target"].get("q") or {}
        for joint in JOINTS:
            q, target = _number(row["joints"].get(joint)), _number(target_q.get(joint))
            if q is not None and target is not None:
                acc = state.joint_errors.setdefault(joint, {
                    "count": 0, "sum_squared_rad2": 0.0,
                    "completed_segment_count": 0, "unknown_segment_count": 0,
                })
                acc["count"] += 1
                acc["completed_segment_count" if completed is True else "unknown_segment_count"] += 1
                acc["sum_squared_rad2"] += (q - target) ** 2

    def finish(self, episode, *, env_runner=None, env_index=0, metrics_logger=None,
               prev_episode_chunks=None) -> None:
        """Write all outcomes once, release the ring even on disk/format errors."""
        state = None
        slot = None
        try:
            slot, episode_id, timestep = self._identity(episode, env_runner, env_index)
            if self._last_closed.get(slot) == episode_id:
                self._metric("duplicate_ends", metrics_logger)
                return
            state = self._slots.get(slot)
            if state is None or state.episode_id != episode_id:
                state = None
                self._metric("episodes_dropped", metrics_logger)
                self._warn("missing_episode", "on_episode_end without an observed matching episode")
                return
            self._last_closed[slot] = episode_id
            self._last_closed.move_to_end(slot)
            if len(self._last_closed) > MAX_ENV_SLOTS:
                self._last_closed.popitem(last=False)
            chunks = [*(prev_episode_chunks or []), episode]
            record = {
                "schema": "unified39_episode_telemetry_v1",
                "episode_uid": f"{self.writer_id}:w{slot[0]}:e{slot[1]}:{episode_id}",
                "rllib_episode_id": episode_id, "writer_id": self.writer_id,
                "pid": self.process_id, "worker_index": slot[0], "env_index": slot[1],
                "episode_steps": timestep,
                "episode_start_time_s": state.start_time_s,
                "duration_s": (state.tail[-1]["time_s"] - state.start_time_s
                    if state.start_time_s is not None and state.tail and state.tail[-1]["time_s"] is not None else None),
                "observed_steps": state.observed_steps,
                "missing_steps": max(state.missing_steps + max(0, timestep - state.last_t),
                                     timestep - state.observed_steps),
                "rows_with_missing_fields": state.missing_rows,
                "policy_distribution_rows": state.distribution_rows,
                "completed_segment_rows": state.completed_segment_rows,
                "partial_segment_rows": state.partial_segment_rows,
                "unknown_segment_rows": state.unknown_segment_rows,
                "partial_episode": state.first_t != 1,
                "terminated": bool(episode.is_terminated), "truncated": bool(episode.is_truncated),
                "end_reason": episode.get_infos(-1).get("end_reason"),
                "episode_return": _number(sum(float(chunk.get_return()) for chunk in chunks)),
                "observed_reward_sum": state.reward_sum,
                "episode_start_offset_s": state.tail[-1]["episode_start_offset_s"] if state.tail else None,
                "last_time_s": state.tail[-1]["time_s"] if state.tail else None,
                "terminal_contact_time_s": (_number(state.tail[-1].get("terminal_contact", {}).get("time_s"))
                    if state.tail and state.tail[-1].get("terminal_contact", {}).get("available") else None),
                "peaks": state.peaks, "event_counts": state.event_counts,
                "last_resync": state.last_resync, "last_hs_cancel": state.last_hs_cancel,
                "joint_error_accumulators": state.joint_errors,
                "policy_versions": state.policy_versions,
                "tail_max_steps": TAIL_STEPS, "tail": list(state.tail),
                "action_semantics": "raw_policy_action is the sampled RLModule action before module-to-env clipping; policy_segment_values is the actual commanded reference. No new policy forward is run.",
            }
            if self.path_error:
                raise ValueError(self.path_error)
            self.directory.mkdir(parents=True, exist_ok=True)
            path = self.directory / f"episodes_w{slot[0]}_{self.writer_id}_part{self._file_part}.jsonl"
            serialized = json.dumps(record, separators=(",", ":"), allow_nan=False)
            with path.open("a", encoding="utf-8") as stream:
                stream.write(serialized + "\n")
            self._metric("episodes_written", metrics_logger)
        except Exception as error:
            self._metric("write_errors", metrics_logger)
            self._metric("episodes_dropped", metrics_logger)
            self._warn("write", error)
            # A partial write cannot corrupt a later episode's JSON line.
            self._file_part += 1
        finally:
            if state is not None and slot is not None and self._slots.get(slot) is state:
                del self._slots[slot]
