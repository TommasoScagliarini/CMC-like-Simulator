"""Offline comparison semantics: aligned targets, unequal coverage and finite data."""
import copy
import json
import math

import pytest

from unified_MLP.validation import compare_pretraining_checkpoints as comparison


def fixture_rollout():
    rows = []
    for index, (begin, end, count) in enumerate(((0.0, 0.01, 2), (0.01, 0.03, 8))):
        error = index + 1.0
        row = {
            "step": index + 1, "time": end, "policy_segment_times": [begin, end],
            "_pre_time": begin, "_dt": end - begin,
            "prosthetic_state": {}, "imitation_target_q": {}, "imitation_target_qdot": {},
            "actor_observation_before": {
                "online_left_cycle_duration_s": 0, "online_left_gait_phase_sin": 0,
                "online_left_gait_phase_cos": 1, "gait_phase_sin": 0, "gait_phase_cos": 0,
                "pros_knee_angle": -500.0,
            },
            "phase_fsm": {"valid_hs_count": 0, "valid_to_count": 0, "valid_cycle_count": 0,
                          "invalid_event_count": 0, "state_name": "WAIT_HS"},
            "reward_terms": {"grf_penetration_m": 0.01, "u_abs_max": 0.3,
                             "policy_action_clip_fraction": 0, "reserve_norm_nm": 10},
        }
        for joint in comparison.JOINTS.values():
            row["imitation_target_q"][joint] = 100.0
            row["imitation_target_qdot"][joint] = 0.0
            row["prosthetic_state"].update({joint: 100.0 + error, joint + "_vel": error,
                                          joint + "_served_ref": 100.0 + 2 * error,
                                          joint + "_served_ref_vel": 2 * error})
            prefix = joint + "_sea_"
            row["reward_terms"][prefix + "sample_count"] = count
            for metric, suffix, unit in (
                ("tau_spring", "sum_squares_nm2", "nm"),
                ("torque_error", "sum_squares_nm2", "nm"),
                ("motor_speed", "sum_squares_rad2_s2", "rad_s"),
                ("motor_accel", "sum_squares_rad2_s4", "rad_s2"),
                ("motor_power", "sum_squares_w2", "w"),
            ):
                row["reward_terms"][prefix + metric + "_" + suffix] = count * error ** 2
                row["reward_terms"][prefix + metric + "_abs_max_" + unit] = error
            for field, v in {"tau_input_abs_max_nm": 2, "tau_input_saturation_count": int(index == 0),
                             "tau_input_plugin_fallback_count": 0,
                             "motor_accel_plugin_fallback_count": 0}.items():
                row["reward_terms"][prefix + field] = v
        rows.append(row)
    return {"rows": rows, "summary": {
        "ok": True, "steps": 2, "action_selection": "deterministic",
        "grf_penetration_penalty_threshold_m": 0.02, "episode_return": -2,
        "reward_mean": -1, "end_reason": "episode_time_limit", "terminated": False, "truncated": True,
    }}


def test_tracking_uses_aligned_poststep_data_and_duration_weights():
    rollout = fixture_rollout()
    result = comparison.analyze_window(rollout, rollout["rows"], 0.03, True)
    knee = result["tracking"]["knee"]
    assert knee["actual_vs_prescribed"]["position"]["rmse_rad"] == pytest.approx(math.sqrt(3))
    assert knee["served_vs_prescribed"]["position"]["rmse_rad"] == pytest.approx(math.sqrt(12))
    assert knee["actual_vs_served"]["position"]["rmse_rad"] == pytest.approx(math.sqrt(3))
    assert result["phase"]["online_period_available_fraction"] == 0


def test_sea_aggregates_internal_samples_without_averaging_rms():
    rollout = fixture_rollout()
    result = comparison.analyze_window(rollout, rollout["rows"], 0.03, True)
    assert result["sea"]["knee"]["tau_spring_rms_nm"] == pytest.approx(math.sqrt(3.4))
    assert result["sea"]["knee"]["tau_input_saturation_fraction"] == pytest.approx(0.1)


def test_common_prefix_preserves_coverage_and_does_not_invent_reward():
    long = fixture_rollout()
    short = copy.deepcopy(long)
    short["rows"] = short["rows"][:1]
    a, b = comparison.common_rows(long, short)
    assert len(a) == len(b) == 1
    metrics = comparison.analyze_window(long, a, 0.03, False)
    assert metrics["planned_coverage_fraction"] == pytest.approx(1 / 3)
    assert metrics["episode_coverage_fraction"] == pytest.approx(1 / 3)
    assert metrics["episode_sample_coverage_fraction"] == 0.5
    assert metrics["reward"]["episode_return"] is None


def test_common_prefix_rejects_mismatched_prescribed_or_times():
    first = fixture_rollout()
    second = copy.deepcopy(first)
    second["rows"][0]["imitation_target_q"]["pros_knee_angle"] += 0.1
    with pytest.raises(ValueError, match="prescribed targets differ"):
        comparison.common_rows(first, second)
    second = copy.deepcopy(first)
    second["rows"][0]["time"] += 0.001
    with pytest.raises(ValueError, match="different time grids"):
        comparison.common_rows(first, second)


def test_measured_nonfinite_fails_but_inactive_diagnostics_do_not():
    rollout = fixture_rollout()
    rollout["rows"][0]["reward_terms"]["unused_inactive_loss"] = float("nan")
    comparison.analyze_window(rollout, rollout["rows"], 0.03, True)
    rollout["rows"][0]["imitation_target_q"]["pros_knee_angle"] = float("nan")
    with pytest.raises(ValueError, match="Nonfinite"):
        comparison.analyze_window(rollout, rollout["rows"], 0.03, True)


def test_loader_rejects_incomplete_or_noncontiguous_trace(tmp_path):
    rollout = fixture_rollout()
    summary = tmp_path / "rollout_summary.json"
    trace = tmp_path / "rollout_policy_trace.json"
    summary.write_text(json.dumps(rollout["summary"]), encoding="utf-8")
    trace.write_text(json.dumps(rollout["rows"]), encoding="utf-8")
    assert len(comparison.load_rollout(tmp_path)["rows"]) == 2
    rollout["rows"][1]["policy_segment_times"][0] = 0.015
    trace.write_text(json.dumps(rollout["rows"]), encoding="utf-8")
    with pytest.raises(ValueError, match="discontinuous"):
        comparison.load_rollout(tmp_path)


def bounced_hs_rollout():
    rollout = fixture_rollout()
    rollout["rows"][0]["phase_fsm"].update(valid_hs_count=1, hs_cancelled_count=0)
    rollout["rows"][1]["phase_fsm"].update(valid_hs_count=0, hs_cancelled_count=1,
                                            invalid_event_count=1)
    return rollout


@pytest.mark.parametrize("boundary_only", [False, True])
def test_confirmed_hs_cancellation_preserves_net_fsm_count(boundary_only):
    rollout = bounced_hs_rollout()
    rows = rollout["rows"][1:] if boundary_only else rollout["rows"]
    phase = comparison.analyze_window(rollout, rows, 0.03, not boundary_only)["phase"]
    assert phase["valid_hs_count"] == 0
    assert phase["hs_cancelled_count"] == 1
    assert phase["invalid_event_count"] == 1


@pytest.mark.parametrize("boundary_only", [False, True])
@pytest.mark.parametrize("cancellation_evidence", ["absent", "unchanged", "insufficient"])
def test_hs_decrease_requires_enough_observed_cancellations(boundary_only, cancellation_evidence):
    rollout = bounced_hs_rollout()
    if cancellation_evidence == "absent":
        for row in rollout["rows"]:
            row["phase_fsm"].pop("hs_cancelled_count")
    elif cancellation_evidence == "unchanged":
        rollout["rows"][1]["phase_fsm"]["hs_cancelled_count"] = 0
    else:
        rollout["rows"][0]["phase_fsm"]["valid_hs_count"] = 2
    rows = rollout["rows"][1:] if boundary_only else rollout["rows"]
    with pytest.raises(ValueError, match="Nonmonotonic.*valid_hs_count"):
        comparison.analyze_window(rollout, rows, 0.03, not boundary_only)


@pytest.mark.parametrize("key", ["valid_to_count", "valid_cycle_count", "invalid_event_count",
                                  "hs_cancelled_count"])
def test_other_counter_decreases_are_never_excused_by_hs_cancellation(key):
    rollout = bounced_hs_rollout()
    rollout["rows"][0]["phase_fsm"][key] = 2
    rollout["rows"][1]["phase_fsm"][key] = 1
    with pytest.raises(ValueError, match="Nonmonotonic"):
        comparison.analyze_window(rollout, rollout["rows"][1:], 0.03, False)


@pytest.mark.parametrize("number", [-1, 0.5, float("inf")])
@pytest.mark.parametrize("key", ["valid_hs_count", "valid_to_count", "valid_cycle_count",
                                  "invalid_event_count", "hs_cancelled_count"])
def test_counters_must_be_finite_nonnegative_integers(number, key):
    rollout = bounced_hs_rollout()
    rollout["rows"][0]["phase_fsm"][key] = number
    with pytest.raises(ValueError):
        comparison.analyze_window(rollout, rollout["rows"][1:], 0.03, False)


def test_legacy_trace_does_not_invent_cancellation_telemetry():
    rollout = fixture_rollout()
    phase = comparison.analyze_window(rollout, rollout["rows"], 0.03, True)["phase"]
    assert "hs_cancelled_count" not in phase


def test_cancellation_diagnostic_must_be_present_throughout_observed_window():
    rollout = bounced_hs_rollout()
    rollout["rows"][0]["phase_fsm"].pop("hs_cancelled_count")
    with pytest.raises(ValueError, match="Partially missing"):
        comparison.analyze_window(rollout, rollout["rows"][1:], 0.03, False)


def test_prefix_does_not_read_counter_evidence_from_future_rows():
    rollout = bounced_hs_rollout()
    rollout["rows"][1]["phase_fsm"]["hs_cancelled_count"] = float("nan")
    phase = comparison.analyze_window(rollout, rollout["rows"][:1], 0.03, False)["phase"]
    assert phase["valid_hs_count"] == 1
    assert phase["hs_cancelled_count"] == 0


def test_cancelled_resync_can_leave_net_hs_zero_due_to_runtime_clamp():
    rollout = bounced_hs_rollout()
    rollout["rows"][0]["phase_fsm"]["valid_hs_count"] = 0
    phase = comparison.analyze_window(rollout, rollout["rows"], 0.03, True)["phase"]
    assert phase["valid_hs_count"] == 0
    assert phase["hs_cancelled_count"] == 1
    assert "hs_accepted_count" not in phase
