"""Reward tuning uses the same physical arithmetic online and offline."""
from dataclasses import replace
import json
from pathlib import Path

import pytest

from unified_MLP_39.reward_function import RewardConfig, compute_reward
from unified_MLP_39.tuning_terms import TuningHistory, enrich_tuning_terms


def _cfg(**kwargs):
    return RewardConfig(reward_mode="imitation", oob_weight=0.0, **kwargs)


def _velocity(knee, ankle=0.0):
    return {"pros_knee_angle_vel": knee, "pros_ankle_angle_vel": ankle}


def test_segment_positive_statistics_preserve_sign_and_all_sample_denominator():
    from simulation_runner import SimulationRunner
    samples = []
    for index, torque in enumerate((-12.0, 0.0, 6.0, 18.0)):
        row = dict.fromkeys((
            "tau_input_raw_nm", "tau_input_nm", "tau_input_saturated",
            "torque_error_nm", "motor_speed_rad_s", "motor_accel_rad_s2",
            "joint_power_w", "motor_power_w", "tau_input_plugin_fallback",
            "motor_accel_plugin_fallback",
        ), 0.0)
        row.update(time_s=index * 0.001, tau_spring_nm=torque)
        samples.append({"pros_ankle_angle": row})
    stats = SimulationRunner._summarize_sea_segment_diagnostics(samples)["joints"]["pros_ankle_angle"]
    assert stats["tau_spring_positive_sum_nm"] == 24.0
    assert stats["tau_spring_positive_mean_nm"] == 6.0
    assert stats["tau_spring_positive_max_nm"] == 18.0
    assert stats["tau_spring_positive_sample_count"] == 2.0
    raw = {f"pros_ankle_angle_sea_{key}": value for key, value in stats.items()}
    enriched = enrich_tuning_terms(raw, _cfg(ankle_positive_torque_weight=1.0))
    assert enriched["ankle_positive_torque_loss"] == 0.2


def test_knee_rate_is_joint_specific_uncapped_and_ankle_cost_linear():
    raw = {
        "pros_ankle_angle_sea_tau_spring_positive_mean_nm": 60.0,
        "pros_knee_angle_sea_tau_spring_rate_rms_nm_s": 6000.0,
        "pros_ankle_angle_sea_tau_spring_rate_rms_nm_s": 999999.0,
    }
    cfg = _cfg(ankle_positive_torque_weight=1.0, knee_torque_rate_weight=1.0)
    enriched = enrich_tuning_terms(raw, cfg)
    assert enriched["ankle_positive_torque_loss"] == 2.0
    assert enriched["knee_torque_rate_loss"] == 36.0


def test_nonuniform_sampling_gives_exact_constant_jerk_and_reset_warmup():
    cfg = _cfg(knee_kinematic_jerk_weight=1.0)
    history = TuningHistory()
    history.reset(0.0, _velocity(0.0))
    first = enrich_tuning_terms({}, cfg, time_s=0.01, kinematics=_velocity(0.5 * 6000 * 0.01**2), history=history)
    assert first["knee_kinematic_jerk_warmup"] == 1.0
    second = enrich_tuning_terms({}, cfg, time_s=0.03, kinematics=_velocity(0.5 * 6000 * 0.03**2), history=history)
    assert second["knee_kinematic_jerk_available"] == 1.0
    assert second["knee_kinematic_jerk_rad_s3"] == pytest.approx(6000.0)
    assert second["knee_kinematic_jerk_loss"] == pytest.approx(36.0)
    history.reset(0.0, _velocity(0.0))
    first_again = enrich_tuning_terms({}, cfg, time_s=0.01, kinematics=_velocity(0.3), history=history)
    assert first_again["knee_kinematic_jerk_available"] == 0.0


@pytest.mark.parametrize("acceleration", [0.0, 12.0])
def test_constant_velocity_or_acceleration_has_zero_jerk(acceleration):
    history = TuningHistory()
    cfg = _cfg(knee_kinematic_jerk_weight=1.0, ankle_kinematic_jerk_weight=1.0)
    history.reset(0.0, _velocity(2.0, 3.0))
    for time in (0.01, 0.02, 0.03):
        row = enrich_tuning_terms({}, cfg, time_s=time, kinematics=_velocity(2 + acceleration*time, 3 + acceleration*time), history=history)
    assert row["knee_kinematic_jerk_loss"] == pytest.approx(0.0, abs=1e-20)
    assert row["ankle_kinematic_jerk_loss"] == pytest.approx(0.0, abs=1e-20)


def test_partial_segment_ignores_stale_inputs_and_clears_history():
    history = TuningHistory()
    history.reset(0.0, _velocity(0.0))
    cfg = _cfg(ankle_positive_torque_weight=1.0, knee_kinematic_jerk_weight=1.0)
    row = enrich_tuning_terms({"pros_ankle_angle_sea_tau_spring_positive_mean_nm": 1e6}, cfg, time_s=0.0, history=history, segment_completed=False)
    assert row["ankle_positive_torque_available"] == 0.0
    assert row["ankle_positive_torque_loss"] == 0.0
    assert not history._samples
    reward, components = compute_reward({**row, "safety_loss": 1.0}, cfg)
    assert components["safety_term"] == 2.0
    assert reward == -1.0


def test_missing_nonfinite_inputs_and_invalid_time_fail_when_enabled():
    cfg = _cfg(ankle_positive_torque_weight=1.0)
    for raw in ({}, {"pros_ankle_angle_sea_tau_spring_positive_mean_nm": float("nan")}):
        with pytest.raises(ValueError, match="required ankle_positive_torque"):
            enrich_tuning_terms(raw, cfg)
    with pytest.raises(ValueError, match="required enabled reward term missing"):
        compute_reward({}, cfg)
    with pytest.raises(ValueError, match="positive"):
        compute_reward({}, replace(cfg, ankle_positive_torque_scale_nm=0.0))
    history = TuningHistory()
    history.reset(1.0, _velocity(0.0))
    for time in (1.0, 0.0):
        with pytest.raises(ValueError, match="strictly increasing"):
            enrich_tuning_terms({}, _cfg(knee_kinematic_jerk_weight=1), time_s=time, kinematics=_velocity(0), history=history)


def test_quality_costs_are_post_clip_and_exnovo_ignores_all_tuning_params():
    terms = {"effort_loss": 100.0, "ankle_positive_torque_loss": 2.0}
    cfg = _cfg(ankle_positive_torque_weight=0.25)
    reward, components = compute_reward(terms, cfg)
    assert components["reward_base"] == 0.0
    assert reward == -0.5
    exnovo = replace(cfg, reward_mode="ex_novo", ankle_positive_torque_scale_nm=-1.0)
    assert compute_reward({}, exnovo)[0] == compute_reward({}, replace(exnovo, ankle_positive_torque_weight=0))[0]


def test_online_offline_velocity_history_produces_identical_loss_rows():
    cfg = _cfg(knee_kinematic_jerk_weight=1, ankle_kinematic_jerk_weight=1)
    histories = [TuningHistory(), TuningHistory()]
    for history in histories:
        history.reset(0.0, _velocity(2.0, 3.0))
    for time in (0.01, 0.02, 0.03):
        states = _velocity(2 + time*time, 3 - time*time)
        live = enrich_tuning_terms({}, cfg, time_s=time, kinematics=states, history=histories[0])
        trace = json.loads(json.dumps({"time": time, "prosthetic_state": states}))
        offline = enrich_tuning_terms({}, cfg, time_s=trace["time"], kinematics=trace["prosthetic_state"], history=histories[1])
        assert live == offline


def test_unprimed_history_reports_two_rows_of_warmup_and_missing_is_not_warmup():
    cfg = _cfg(knee_kinematic_jerk_weight=1)
    history = TuningHistory()
    availability = []
    for time in (0.01, 0.02, 0.03):
        row = enrich_tuning_terms({}, cfg, time_s=time, kinematics=_velocity(time*time), history=history)
        availability.append(row["knee_kinematic_jerk_available"])
    assert availability == [0.0, 0.0, 1.0]
    with pytest.raises(ValueError, match="required knee kinematic jerk input unavailable"):
        enrich_tuning_terms({}, cfg, time_s=0.04, kinematics={}, history=history)


def test_inactive_nonfinite_loss_cannot_poison_legacy_reward():
    cfg = _cfg()
    base = compute_reward({}, cfg)[0]
    terms = {"ankle_positive_torque_loss": float("nan"), "knee_torque_rate_loss": float("nan")}
    assert compute_reward(terms, cfg)[0] == base


def test_legacy_best80_zero_weights_reproduce_saved_return_when_fixture_available():
    folder = Path(__file__).resolve().parents[1] / "runs/rollout/MLP39_LEFT_pen2_2026-09-24_best80_deterministic"
    if not folder.is_dir():
        pytest.skip("historical local rollout is not available")
    summary = json.loads((folder / "rollout_summary.json").read_text())
    trace = json.loads((folder / "rollout_policy_trace.json").read_text())
    cfg = RewardConfig.from_mapping(summary["reward_config"])
    result = sum(compute_reward(row["reward_terms"], cfg, reference=row["policy_segment_values"])[0] for row in trace)
    assert result == pytest.approx(summary["episode_return"], abs=1e-10)
