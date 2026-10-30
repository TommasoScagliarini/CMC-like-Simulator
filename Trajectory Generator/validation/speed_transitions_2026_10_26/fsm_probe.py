"""Synthetic contact probe of the unchanged V26/v3 FSM, not a walking rollout.

Run from any directory with the project's envCMC-rllib Python. This feeds the
production adapter the same 1 ms batches used at 10 ms policy boundaries.
Continuation after TIMEOUT is diagnostic only: the real reward wrapper stops
the episode at its first timeout. No weights, dynamics, or detector geometry
are exercised here.
"""

from __future__ import annotations

import hashlib
import json
import math
import sys
from dataclasses import asdict
from pathlib import Path

HERE = Path(__file__).resolve().parent
TRAJECTORY = HERE.parents[1]
BASELINE = TRAJECTORY / "baseline_MLP"
sys.path.insert(0, str(BASELINE))
import _bootstrap  # noqa: E402

_bootstrap.ensure_sim_paths()
import win_runtime  # noqa: E402,F401
import yaml  # noqa: E402
from binary_phase import (  # noqa: E402
    BinaryPhaseActiveAdapterV26,
    BinaryPhaseTransferError,
    HeelQualifiedBinaryPhaseFSM,
    HeelQualifiedBinaryPhaseFSMConfig,
)
from prosthetic_phase_fsm import (  # noqa: E402
    ProstheticPhaseFSM,
    ProstheticPhaseFSMConfig,
)

PRESET = BASELINE / "training_exnovo_cfg.yaml"
PERIOD = 1.58
STANCE_FRACTION = 0.68
INITIAL_AIR_S = 0.2


def configuration():
    """Mirror the environment's mapping from the current preset to its FSM."""
    preset = yaml.safe_load(PRESET.read_text(encoding="utf-8"))
    reward, grf = preset["reward"], preset["grf"]
    mapping = {
        "min_stance_duration_s": "phase_min_stance_duration_s",
        "min_swing_duration_s": "phase_min_swing_duration_s",
        "landing_window_start_s": "phase_landing_window_start_s",
        "landing_window_end_s": "phase_landing_window_end_s",
        "stance_hard_timeout_s": "phase_stance_hard_timeout_s",
        "swing_hard_timeout_s": "phase_swing_hard_timeout_s",
        "landing_force_full_credit_bw": "contact_load_target_bw",
        "min_stance_contact_fraction": "phase_min_stance_contact_fraction",
        "min_stance_load_bw_s": "phase_min_stance_load_bw_s",
        "min_cycle_knee_excursion_rad": "phase_min_cycle_knee_excursion_rad",
        "hs_event_credit": "phase_hs_event_credit",
        "toe_off_event_credit": "phase_to_event_credit",
        "cycle_complete_bonus": "phase_cycle_complete_bonus",
        "failure_extra_penalty": "phase_failure_extra_penalty",
    }
    actor = ProstheticPhaseFSMConfig(
        **{target: float(reward[source]) for target, source in mapping.items()},
        event_source="binary_active_v26",
        resync_enabled=grf["binary_phase_actor_fsm_version"] == "v3",
        hs_cancel_enabled=grf["binary_phase_actor_fsm_version"] == "v3",
        detector_sample_dt_s=grf["detector_sample_dt_s"],
        sensor_on_threshold_n=grf["phase_sensor_on_threshold_n"],
        sensor_off_threshold_n=grf["phase_sensor_off_threshold_n"],
        sensor_dwell_s=grf["phase_sensor_dwell_s"],
    )
    binary = HeelQualifiedBinaryPhaseFSMConfig(
        sample_dt_s=grf["detector_sample_dt_s"],
        debounce_s=grf["binary_phase_debounce_s"],
        event_contract_id=grf["binary_phase_event_contract_id"],
    )
    assert grf["binary_phase_invalid_event_policy"] == "reject_continue"
    return actor, binary


def source_time(t: float, schedule: list[tuple[str, float, float]]):
    """Integrate synthetic cadence; keep the initial air interval common."""
    if t < INITIAL_AIR_S:
        return -1.0, "initial_air"
    remaining = t - INITIAL_AIR_S
    source = 0.0
    for name, duration, rate in schedule:
        elapsed = min(max(remaining, 0.0), duration)
        source += elapsed * rate
        if remaining <= duration + 1e-12:
            return source, name
        remaining -= duration
    return source, schedule[-1][0]


def contact_at(t: float, schedule):
    source, stage = source_time(t, schedule)
    if source < 0.0:
        return False, False, -0.2, stage
    phase = (source / PERIOD) % 1.0
    # Heel -> both -> toe -> air. "Both" means two sensors on one foot,
    # never bilateral double support or a claim about balance.
    heel = phase < 0.50
    toe = 0.10 <= phase < STANCE_FRACTION
    knee = -0.4 + 0.3 * math.cos(2.0 * math.pi * phase)
    return heel, toe, knee, stage


def run_case(name: str, schedule, actor_cfg, binary_cfg):
    adapter = BinaryPhaseActiveAdapterV26()
    candidate = adapter.prime(
        binary_fsm=HeelQualifiedBinaryPhaseFSM(binary_cfg),
        phase_fsm=ProstheticPhaseFSM(actor_cfg),
        time_s=0.0,
        heel_contact=False,
        toe_contact=False,
    )
    n_steps = math.ceil((INITIAL_AIR_S + sum(x[1] for x in schedule)) / 0.01)
    transitions = []
    first_timeout = None
    first_recovery = None
    transfer_failure = None
    snapshots = {}
    timeout_count = 0
    for step in range(1, n_steps + 1):
        time_s = step / 100.0
        samples = []
        for sample_index in range((step - 1) * 10 + 1, step * 10 + 1):
            t = sample_index / 1000.0
            heel, toe, _, _ = contact_at(t, schedule)
            samples.append({
                "time_s": t,
                "left_heel_contact": heel,
                "left_toe_contact": toe,
            })
        heel, toe, knee, stage = contact_at(time_s, schedule)
        try:
            candidate = adapter.advance(
                binary_fsm=candidate.binary_fsm,
                phase_fsm=candidate.phase_fsm,
                time_s=time_s,
                previous_time_s=(step - 1) / 100.0,
                sensor_samples=samples,
                normal_force_bw=0.8 if heel or toe else 0.0,
                in_contact=heel or toe,
                prosthetic_knee_angle_rad=knee,
                prosthetic_ankle_angle_rad=0.0,
                invalid_event_mode="drop",
            )
        except (BinaryPhaseTransferError, ValueError) as exc:
            transfer_failure = {"time_s": time_s, "stage": stage,
                                "error_type": type(exc).__name__, "error": str(exc)}
            break
        payload = candidate.phase_payload
        state = payload["state_name"]
        for transition in payload["accepted_transitions_this_step"]:
            transitions.append({"boundary_time_s": time_s, "stage": stage, **transition})
        if state == "TIMEOUT":
            timeout_count += 1
            if first_timeout is None:
                first_timeout = {"time_s": time_s, "stage": stage, "payload": payload}
        elif first_timeout is not None and first_recovery is None:
            first_recovery = {"time_s": time_s, "stage": stage, "state_name": state}
        snapshots[stage] = {
            "time_s": time_s,
            "state_name": state,
            "valid_hs_count": payload["valid_hs_count"],
            "valid_to_count": payload["valid_to_count"],
            "valid_cycle_count": payload["valid_cycle_count"],
            "last_period_s": payload["last_period_s"],
            "invalid_event_count": payload["invalid_event_count"],
        }
    return {
        "case": name,
        "schedule_after_initial_air": [
            {"stage": label, "duration_s": duration, "cadence_multiplier": rate}
            for label, duration, rate in schedule
        ],
        "first_timeout": first_timeout,
        "first_recovery_after_timeout_diagnostic_only": first_recovery,
        "transfer_failure": transfer_failure,
        "timeout_boundary_count": timeout_count,
        "stage_end_snapshots": snapshots,
        "transitions": transitions,
        "final_actor_payload": candidate.phase_payload,
        "final_binary_payload": candidate.binary_payload,
        "would_real_episode_terminate": first_timeout is not None or transfer_failure is not None,
    }


def main():
    actor, binary = configuration()
    scenarios = {
        "constant": [("walk", PERIOD * 6, 1.0)],
        "walk_stop_walk": [
            ("walk_before", PERIOD * 2 + 0.4, 1.0),
            ("hold_heel_and_toe_contact", 3.0, 0.0),
            ("walk_after", PERIOD * 2, 1.0),
        ],
        "cadence_changes": [
            ("nominal_before", PERIOD * 2, 1.0),
            ("slow", PERIOD * 2 / 0.8, 0.8),
            ("fast", PERIOD * 2 / 1.2, 1.2),
            ("nominal_after", PERIOD * 2, 1.0),
        ],
    }
    result = {
        "scope": "synthetic contacts + production binary FSM + production adapter + actor FSM; no network or dynamics",
        "continuation_after_timeout": "diagnostic only; live environment terminates on first timeout",
        "period_nominal_s": PERIOD,
        "stance_fraction": STANCE_FRACTION,
        "normal_load_bw_in_contact": 0.8,
        "initial_air_s": INITIAL_AIR_S,
        "preset_sha256": hashlib.sha256(PRESET.read_bytes()).hexdigest(),
        "source_sha256": {
            str(path.relative_to(TRAJECTORY)): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in (Path(__file__), TRAJECTORY / "phase_tracking" / "binary_phase.py",
                         TRAJECTORY / "phase_tracking" / "prosthetic_phase_fsm.py")
        },
        "actor_config": asdict(actor),
        "binary_config": asdict(binary),
        "cases": {name: run_case(name, schedule, actor, binary) for name, schedule in scenarios.items()},
    }
    output = HERE / "fsm_probe.json"
    output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    for name, case in result["cases"].items():
        first = case["first_timeout"]
        print(name, "first_timeout", None if first is None else first["time_s"],
              "cycles", case["final_actor_payload"]["valid_cycle_count"],
              "recovered", case["first_recovery_after_timeout_diagnostic_only"],
              "transfer_failure", case["transfer_failure"])


if __name__ == "__main__":
    main()
