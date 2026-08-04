#!/usr/bin/env python3
"""Replay the experimental completed-segment corridor on a recorded rollout.

This is an offline validation adapter, not a rollout generator.  It recovers
accepted prosthetic HS/TO timestamps only from monotonic FSM counters and the
elapsed clock of the newly opened state.  Ambiguous or inconsistent traces are
rejected.  The original served references are copied unchanged, passed through
the production completed-segment ledger, and enriched with runtime-compatible
diagnostic payloads for the independent validator.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
from dataclasses import replace
from pathlib import Path
from typing import Any, Mapping, Sequence


SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent
BASELINE_DIR = REPO_ROOT / "Trajectory Generator" / "baseline_MLP"
if str(BASELINE_DIR) not in sys.path:
    sys.path.insert(0, str(BASELINE_DIR))

from experimental_morphology_corridor import (  # noqa: E402
    EXPERIMENTAL_PHASE_MODE,
    CompletedSegmentMorphologyLedger,
)
from reward_function import (  # noqa: E402
    RewardConfig,
    _load_morphology_profile,
    _morphology_canonical_to_phase,
    _morphology_corridor_at,
    _morphology_hard_excursions,
    _morphology_interval_losses,
)


TIME_TOL_S = 1.0e-9
VALUE_TOL = 1.0e-8


def _load_json(path: Path) -> Any:
    with path.open("r", encoding="utf-8") as stream:
        return json.load(stream)


def _write_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="\n") as stream:
        json.dump(value, stream, indent=2, allow_nan=False)
        stream.write("\n")


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _finite(value: Any, *, context: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{context} must be numeric") from exc
    if not math.isfinite(result):
        raise ValueError(f"{context} must be finite")
    return result


def _counter(terms: Mapping[str, Any], key: str, row_index: int) -> int:
    value = _finite(terms.get(key, 0.0), context=f"trace[{row_index}].{key}")
    rounded = int(round(value))
    if rounded < 0 or abs(value - rounded) > VALUE_TOL:
        raise ValueError(f"trace[{row_index}].{key} must be a non-negative integer")
    return rounded


def _rows(value: Any) -> list[dict[str, Any]]:
    if not isinstance(value, list) or not value:
        raise ValueError("rollout trace must be a non-empty JSON list")
    rows: list[dict[str, Any]] = []
    previous_time: float | None = None
    for index, item in enumerate(value):
        if not isinstance(item, Mapping):
            raise ValueError(f"trace[{index}] must be a JSON object")
        row = dict(item)
        time_s = _finite(row.get("time"), context=f"trace[{index}].time")
        if previous_time is not None and time_s <= previous_time + TIME_TOL_S:
            raise ValueError("trace timestamps must be strictly increasing")
        previous_time = time_s
        if not isinstance(row.get("reward_terms"), Mapping):
            raise ValueError(f"trace[{index}].reward_terms is required")
        state = row.get("prosthetic_state")
        if not isinstance(state, Mapping):
            raise ValueError(f"trace[{index}].prosthetic_state is required")
        _finite(
            state.get("pros_knee_angle_served_ref"),
            context=f"trace[{index}] knee served reference",
        )
        _finite(
            state.get("pros_ankle_angle_served_ref"),
            context=f"trace[{index}] ankle served reference",
        )
        rows.append(row)
    return rows


def recover_transition_journal(
    rows: Sequence[Mapping[str, Any]],
) -> tuple[list[list[dict[str, Any]]], dict[str, Any]]:
    """Recover accepted transitions and reject ambiguous FSM histories."""
    journals: list[list[dict[str, Any]]] = [[] for _ in rows]
    previous_hs = 0
    previous_to = 0
    previous_cycles = 0
    active_hs: float | None = None
    active_to: float | None = None
    expected_state = 0
    recovered_events: list[dict[str, Any]] = []

    for index, row in enumerate(rows):
        terms = row["reward_terms"]
        time_s = _finite(row.get("time"), context=f"trace[{index}].time")
        hs_count = _counter(terms, "phase_valid_hs_count", index)
        to_count = _counter(terms, "phase_valid_to_count", index)
        cycle_count = _counter(terms, "phase_valid_cycle_count", index)
        if hs_count < previous_hs or to_count < previous_to:
            raise ValueError(f"FSM accepted-event counter decreased at trace[{index}]")
        if cycle_count < previous_cycles:
            raise ValueError(f"FSM valid-cycle counter decreased at trace[{index}]")
        hs_delta = hs_count - previous_hs
        to_delta = to_count - previous_to
        cycle_delta = cycle_count - previous_cycles
        if hs_delta > 1 or to_delta > 1 or cycle_delta > 1:
            raise ValueError(f"FSM counter skipped a value at trace[{index}]")
        if hs_delta and to_delta:
            raise ValueError(f"HS and TO both increment at trace[{index}]")
        if cycle_delta and not hs_delta:
            raise ValueError(
                f"valid-cycle counter incremented without a closing HS at trace[{index}]"
            )

        journal: list[dict[str, Any]] = []
        if to_delta:
            if active_hs is None or expected_state != 1:
                raise ValueError(f"accepted TO has no active stance at trace[{index}]")
            elapsed = _finite(
                terms.get("phase_swing_elapsed_s", 0.0),
                context=f"trace[{index}].phase_swing_elapsed_s",
            )
            event_time = time_s - max(0.0, elapsed)
            if event_time <= active_hs + TIME_TOL_S or event_time > time_s + TIME_TOL_S:
                raise ValueError(f"invalid recovered TO timestamp at trace[{index}]")
            active_to = float(event_time)
            expected_state = 2
            journal.append(
                {
                    "event": "toe_off",
                    "event_time_s": float(event_time),
                    "from_state_id": 1.0,
                    "to_state_id": 2.0,
                    "closed_segment_type": "stance",
                    "segment_start_time_s": float(active_hs),
                    "segment_end_time_s": float(event_time),
                    "segment_valid": 1.0,
                    "anchor_geometry_valid": 1.0,
                    "opens_segment_type": "swing",
                    "cycle_valid": -1.0,
                    "cycle_reject_reason": "",
                }
            )

        if hs_delta:
            elapsed = _finite(
                terms.get("phase_stance_elapsed_s", 0.0),
                context=f"trace[{index}].phase_stance_elapsed_s",
            )
            event_time = time_s - max(0.0, elapsed)
            if event_time > time_s + TIME_TOL_S:
                raise ValueError(f"invalid recovered HS timestamp at trace[{index}]")
            first_hs = active_hs is None
            if first_hs:
                if expected_state != 0 or cycle_delta:
                    raise ValueError(f"invalid first accepted HS at trace[{index}]")
                transition = {
                    "event": "heel_strike",
                    "event_time_s": float(event_time),
                    "from_state_id": 0.0,
                    "to_state_id": 1.0,
                    "closed_segment_type": "",
                    "segment_start_time_s": -1.0,
                    "segment_end_time_s": float(event_time),
                    "segment_valid": 1.0,
                    "anchor_geometry_valid": 1.0,
                    "opens_segment_type": "stance",
                    "cycle_valid": -1.0,
                    "cycle_reject_reason": "",
                }
            else:
                if (
                    expected_state != 2
                    or active_to is None
                    or not active_hs < active_to < event_time
                ):
                    raise ValueError(
                        f"accepted HS does not close HS-TO-HS at trace[{index}]"
                    )
                cycle_valid = bool(cycle_delta)
                transition = {
                    "event": "heel_strike",
                    "event_time_s": float(event_time),
                    "from_state_id": 2.0,
                    "to_state_id": 1.0,
                    "closed_segment_type": "swing",
                    "segment_start_time_s": float(active_to),
                    "segment_end_time_s": float(event_time),
                    "segment_valid": 1.0,
                    "anchor_geometry_valid": 1.0,
                    "opens_segment_type": "stance",
                    "cycle_valid": float(cycle_valid),
                    "cycle_reject_reason": (
                        "" if cycle_valid else "recovered_cycle_rejection"
                    ),
                }
                if cycle_valid:
                    period = float(event_time - active_hs)
                    stance_fraction = float((active_to - active_hs) / period)
                    logged_period = _finite(
                        terms.get("phase_last_period_s", 0.0),
                        context=f"trace[{index}].phase_last_period_s",
                    )
                    logged_fraction = _finite(
                        terms.get("phase_last_stance_fraction", 0.0),
                        context=f"trace[{index}].phase_last_stance_fraction",
                    )
                    if abs(logged_period - period) > VALUE_TOL:
                        raise ValueError(
                            f"recovered period disagrees with FSM at trace[{index}]"
                        )
                    if abs(logged_fraction - stance_fraction) > VALUE_TOL:
                        raise ValueError(
                            f"recovered stance fraction disagrees at trace[{index}]"
                        )
            journal.append(transition)
            active_hs = float(event_time)
            active_to = None
            expected_state = 1

        timeout = _finite(
            terms.get("phase_timeout_exceeded", 0.0),
            context=f"trace[{index}].phase_timeout_exceeded",
        )
        if timeout > 0.0:
            if journal:
                raise ValueError(
                    f"timeout and accepted transition coincide at trace[{index}]"
                )
            side = _counter(terms, "phase_timeout_side", index)
            closed = "stance" if side == 1 else "swing" if side == 2 else ""
            start = active_hs if closed == "stance" else active_to
            if not closed or start is None:
                raise ValueError(f"ambiguous timeout state at trace[{index}]")
            journal.append(
                {
                    "event": "timeout",
                    "event_time_s": float(time_s),
                    "from_state_id": float(expected_state),
                    "to_state_id": 4.0,
                    "closed_segment_type": closed,
                    "segment_start_time_s": float(start),
                    "segment_end_time_s": float(time_s),
                    "segment_valid": 0.0,
                    "anchor_geometry_valid": 0.0,
                    "opens_segment_type": "",
                    "cycle_valid": 0.0,
                    "cycle_reject_reason": f"phase_timeout:{closed}",
                }
            )
            active_hs = None
            active_to = None
            expected_state = 4

        logged_state = int(
            round(
                _finite(
                    terms.get("phase_fsm_state_id", expected_state),
                    context=f"trace[{index}].phase_fsm_state_id",
                )
            )
        )
        if logged_state != expected_state:
            raise ValueError(
                f"recovered FSM state {expected_state} disagrees with logged "
                f"state {logged_state} at trace[{index}]"
            )
        if expected_state == 1:
            if active_hs is None:
                raise ValueError(f"stance has no recovered HS at trace[{index}]")
            elapsed = _finite(
                terms.get("phase_stance_elapsed_s", 0.0),
                context=f"trace[{index}].phase_stance_elapsed_s",
            )
            if abs((time_s - max(0.0, elapsed)) - active_hs) > VALUE_TOL:
                raise ValueError(
                    f"stance elapsed clock disagrees with recovered HS at trace[{index}]"
                )
        elif expected_state == 2:
            if active_to is None:
                raise ValueError(f"swing has no recovered TO at trace[{index}]")
            elapsed = _finite(
                terms.get("phase_swing_elapsed_s", 0.0),
                context=f"trace[{index}].phase_swing_elapsed_s",
            )
            if abs((time_s - max(0.0, elapsed)) - active_to) > VALUE_TOL:
                raise ValueError(
                    f"swing elapsed clock disagrees with recovered TO at trace[{index}]"
                )
        journals[index] = journal
        for transition in journal:
            recovered_events.append(
                {
                    **transition,
                    "confirmation_row_index": index,
                    "confirmation_step": int(row.get("step", index + 1)),
                    "confirmation_time_s": float(time_s),
                    "confirmation_latency_s": float(
                        time_s - float(transition["event_time_s"])
                    ),
                }
            )
        previous_hs = hs_count
        previous_to = to_count
        previous_cycles = cycle_count

    return journals, {
        "accepted_event_count": len(recovered_events),
        "accepted_hs_count": sum(
            item["event"] == "heel_strike" for item in recovered_events
        ),
        "accepted_to_count": sum(
            item["event"] == "toe_off" for item in recovered_events
        ),
        "timeout_count": sum(item["event"] == "timeout" for item in recovered_events),
        "completed_cycle_count": previous_cycles,
        "events": recovered_events,
        "incomplete_active_segment_type": (
            "stance" if expected_state == 1 else "swing" if expected_state == 2 else ""
        ),
    }


def _segment_payload(
    segment: Any,
    *,
    profile: Mapping[str, Any],
    config: RewardConfig,
    alpha: float,
) -> tuple[dict[str, Any], dict[str, float]]:
    samples: list[dict[str, float]] = []
    totals = {
        "knee_loss": 0.0,
        "ankle_loss": 0.0,
        "inside_score": 0.0,
        "knee_hard_excursion_rad": 0.0,
        "ankle_hard_excursion_rad": 0.0,
    }
    for sample, phase in zip(segment.samples, segment.phases(alpha)):
        corridor = _morphology_corridor_at(profile, phase, config)
        knee_corridor = corridor["pros_knee_angle"]
        ankle_corridor = corridor["pros_ankle_angle"]
        knee_loss, _, knee_excursion = _morphology_interval_losses(
            sample.knee_rad,
            knee_corridor["min_rad"],
            knee_corridor["max_rad"],
        )
        ankle_loss, _, ankle_excursion = _morphology_interval_losses(
            sample.ankle_rad,
            ankle_corridor["min_rad"],
            ankle_corridor["max_rad"],
        )
        knee_hard, ankle_hard = _morphology_hard_excursions(
            sample.knee_rad, sample.ankle_rad, config
        )
        inside_score = 0.5 * (
            float(knee_excursion == 0.0) + float(ankle_excursion == 0.0)
        )
        samples.append(
            {
                "time_s": float(sample.time_s),
                "phase": float(phase),
                "knee_served_ref_rad": float(sample.knee_rad),
                "ankle_served_ref_rad": float(sample.ankle_rad),
                "knee_min_rad": float(knee_corridor["min_rad"]),
                "knee_max_rad": float(knee_corridor["max_rad"]),
                "ankle_min_rad": float(ankle_corridor["min_rad"]),
                "ankle_max_rad": float(ankle_corridor["max_rad"]),
                "knee_loss": float(knee_loss),
                "ankle_loss": float(ankle_loss),
                "knee_excursion_rad": float(knee_excursion),
                "ankle_excursion_rad": float(ankle_excursion),
                "inside_score": float(inside_score),
                "knee_hard_excursion_rad": float(knee_hard),
                "ankle_hard_excursion_rad": float(ankle_hard),
            }
        )
        totals["knee_loss"] += knee_loss
        totals["ankle_loss"] += ankle_loss
        totals["inside_score"] += inside_score
        totals["knee_hard_excursion_rad"] = max(
            totals["knee_hard_excursion_rad"], knee_hard
        )
        totals["ankle_hard_excursion_rad"] = max(
            totals["ankle_hard_excursion_rad"], ankle_hard
        )
    return (
        {
            "segment_type": str(segment.segment_type),
            "start_time_s": float(segment.start_time_s),
            "end_time_s": float(segment.end_time_s),
            "duration_s": float(segment.duration_s),
            "sample_count": len(samples),
            "samples": samples,
        },
        totals,
    )


def replay(
    rows: Sequence[Mapping[str, Any]],
    *,
    config: RewardConfig,
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    profile = _load_morphology_profile(config.morphology_profile)
    if profile is None:
        raise ValueError("experimental replay requires a morphology profile")
    alpha = _morphology_canonical_to_phase(profile, require_event_contract=True)
    journals, event_summary = recover_transition_journal(rows)
    ledger = CompletedSegmentMorphologyLedger(
        max_samples=int(config.morphology_completed_segment_max_samples)
    )
    enriched: list[dict[str, Any]] = []
    settled_segments = 0
    settled_samples = 0
    discarded_segments = 0
    discarded_samples = 0
    morphology_loss_sum = 0.0

    for index, (raw_row, journal) in enumerate(zip(rows, journals)):
        row = dict(raw_row)
        state = row["prosthetic_state"]
        time_s = _finite(row.get("time"), context=f"trace[{index}].time")
        knee = _finite(
            state.get("pros_knee_angle_served_ref"),
            context=f"trace[{index}] knee served reference",
        )
        ankle = _finite(
            state.get("pros_ankle_angle_served_ref"),
            context=f"trace[{index}] ankle served reference",
        )
        update = ledger.update(
            time_s=time_s,
            knee_rad=knee,
            ankle_rad=ankle,
            accepted_transitions=journal,
            episode_ended=index == len(rows) - 1,
        )
        payloads: list[dict[str, Any]] = []
        knee_loss_sum = 0.0
        ankle_loss_sum = 0.0
        inside_sum = 0.0
        hard_knee_max = 0.0
        hard_ankle_max = 0.0
        sample_count = 0
        for segment in update.completed_segments:
            payload, totals = _segment_payload(
                segment, profile=profile, config=config, alpha=alpha
            )
            payloads.append(payload)
            sample_count += int(payload["sample_count"])
            knee_loss_sum += totals["knee_loss"]
            ankle_loss_sum += totals["ankle_loss"]
            inside_sum += totals["inside_score"]
            hard_knee_max = max(hard_knee_max, totals["knee_hard_excursion_rad"])
            hard_ankle_max = max(hard_ankle_max, totals["ankle_hard_excursion_rad"])
        total_loss = 0.5 * (knee_loss_sum + ankle_loss_sum)
        hard_max = max(hard_knee_max, hard_ankle_max)

        phase_fsm = dict(row.get("phase_fsm") or {})
        phase_fsm["accepted_transitions_this_step"] = journal
        row["phase_fsm"] = phase_fsm
        row["morphology_completed_segments"] = payloads
        row["morphology_ledger_diagnostics"] = {
            "discard_reason": str(update.discard_reason),
            "discarded_segment_count": int(update.discarded_segment_count),
            "discarded_sample_count": int(update.discarded_sample_count),
            "overflowed": bool(update.overflowed),
            "nonmonotonic_sample": bool(update.nonmonotonic_sample),
            "pending_sample_count": int(update.pending_sample_count),
            "active_segment_type": str(update.active_segment_type),
            "active_segment_start_time_s": float(update.active_segment_start_time_s),
            "completed_segment_count": len(update.completed_segments),
        }
        reward_terms = dict(row["reward_terms"])
        reward_terms.update(
            {
                "morphology_phase_mode_id": 3.0,
                "morphology_canonical_to_phase": float(alpha),
                "morphology_loss": float(total_loss),
                "morphology_loss_mean": float(
                    total_loss / sample_count if sample_count else 0.0
                ),
                "morphology_knee_loss": float(knee_loss_sum),
                "morphology_ankle_loss": float(ankle_loss_sum),
                "morphology_inside_fraction": float(
                    inside_sum / sample_count if sample_count else 0.0
                ),
                "morphology_settled_this_step": float(len(update.completed_segments)),
                "morphology_settled_sample_count": float(sample_count),
                "morphology_pending_sample_count": float(update.pending_sample_count),
                "morphology_discarded_segment_count": float(
                    update.discarded_segment_count
                ),
                "morphology_discarded_sample_count": float(
                    update.discarded_sample_count
                ),
                "morphology_ledger_overflow": float(update.overflowed),
                "morphology_ledger_nonmonotonic_sample": float(
                    update.nonmonotonic_sample
                ),
                "morphology_hard_violation": float(hard_max > 0.0),
                "morphology_hard_knee_excursion_rad": float(hard_knee_max),
                "morphology_hard_ankle_excursion_rad": float(hard_ankle_max),
                "morphology_hard_max_excursion_rad": float(hard_max),
            }
        )
        row["reward_terms"] = reward_terms
        enriched.append(row)
        settled_segments += len(update.completed_segments)
        settled_samples += sample_count
        discarded_segments += int(update.discarded_segment_count)
        discarded_samples += int(update.discarded_sample_count)
        morphology_loss_sum += total_loss

    return enriched, {
        "phase_mode": EXPERIMENTAL_PHASE_MODE,
        "canonical_to_phase": float(alpha),
        "event_recovery": event_summary,
        "settled_segment_count": settled_segments,
        "settled_sample_count": settled_samples,
        "discarded_segment_count": discarded_segments,
        "discarded_sample_count": discarded_samples,
        "raw_settled_coverage": float(
            settled_samples / max(1, settled_samples + discarded_samples)
        ),
        "morphology_loss_sum": float(morphology_loss_sum),
        "hard_violation_count": int(
            sum(
                float(row["reward_terms"]["morphology_hard_violation"]) > 0.0
                for row in enriched
            )
        ),
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trace", type=Path)
    parser.add_argument("--summary", type=Path, default=None)
    parser.add_argument(
        "--reward-json",
        type=Path,
        default=(
            BASELINE_DIR
            / "experimental_configs"
            / "morphology_completed_segment_shadow.json"
        ),
    )
    parser.add_argument("--output-dir", type=Path, required=True)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    trace_path = args.trace.expanduser().resolve()
    summary_path = (
        args.summary.expanduser().resolve()
        if args.summary is not None
        else trace_path.with_name("rollout_summary.json")
    )
    reward_path = args.reward_json.expanduser().resolve()
    output_dir = args.output_dir.expanduser().resolve()
    rows = _rows(_load_json(trace_path))
    original_summary = _load_json(summary_path)
    if not isinstance(original_summary, Mapping):
        raise ValueError("rollout summary must be a JSON object")
    overrides = _load_json(reward_path)
    if not isinstance(overrides, Mapping):
        raise ValueError("reward override must be a JSON object")
    base_config = RewardConfig.from_mapping(original_summary.get("reward_config"))
    config = replace(base_config, **dict(overrides))
    if config.morphology_phase_mode != EXPERIMENTAL_PHASE_MODE:
        raise ValueError("reward override must select the experimental phase mode")
    if float(config.morphology_weight) != 0.0:
        raise ValueError("offline shadow replay requires morphology_weight=0")
    if float(config.morphology_hard_termination_enabled) != 0.0:
        raise ValueError("offline shadow replay requires hard termination disabled")

    enriched, replay_summary = replay(rows, config=config)
    trace_output = output_dir / "rollout_policy_trace.json"
    summary_output = output_dir / "rollout_summary.json"
    replay_output = output_dir / "completed_segment_replay_summary.json"
    output_dir.mkdir(parents=True, exist_ok=True)
    _write_json(trace_output, enriched)
    output_summary = {
        **dict(original_summary),
        "reward_config": config.to_dict(),
        "morphology_replay": replay_summary,
        "source_trace": str(trace_path),
        "source_trace_sha256": _sha256(trace_path),
        "source_summary": str(summary_path),
        "source_summary_sha256": _sha256(summary_path),
        "reward_override": str(reward_path),
        "reward_override_sha256": _sha256(reward_path),
    }
    _write_json(summary_output, output_summary)
    _write_json(
        replay_output,
        {
            **replay_summary,
            "source_trace": str(trace_path),
            "source_trace_sha256": _sha256(trace_path),
            "output_trace": str(trace_output),
            "served_references_modified": False,
        },
    )
    print(json.dumps(replay_summary, indent=2))
    print(f"trace:   {trace_output}")
    print(f"summary: {replay_output}")


if __name__ == "__main__":
    main()
