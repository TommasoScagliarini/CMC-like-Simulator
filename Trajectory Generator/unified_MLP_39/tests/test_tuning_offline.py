"""Offline replay checks preserve missing-data and policy-step semantics."""
import copy
from pathlib import Path

import numpy as np
import pytest

from unified_MLP_39.reward_function import RewardConfig, compute_reward
from unified_MLP_39.reward_tuning.analysis import enrich_trace, family_available, source_replay
from unified_MLP_39.reward_tuning.metrics import causal_jerk, highpass, load_dataset, measure


def _dataset(n=100, with_tables=False):
    cfg = RewardConfig(reward_mode="imitation", oob_weight=0.0)
    rows = []
    for index in range(n):
        time = (index + 1) * .01
        state, targets, before = {}, {}, {}
        for joint in ("knee", "ankle"):
            coord = f"pros_{joint}_angle"
            state.update({coord: 0.0, coord + "_served_ref": 0.0,
                          coord + "_vel": time*time, coord + "_served_ref_vel": 0.0})
            before.update({coord + "_vel": (time-.01)**2, coord + "_served_ref_vel": 0.0})
            targets[coord] = 0.0
        rows.append({"step": index+1, "time": time, "segment_completed": True,
                     "policy_segment_times": [index*.01, time],
                     "policy_segment_values": [[0.0, 0.0], [0.0, 0.0]],
                     "actor_observation_before": before, "prosthetic_state": state,
                     "imitation_target_q": targets, "phase_fsm": {"state_id": 2.0},
                     "reward_terms": {"pros_knee_angle_sea_tau_spring_rate_rms_nm_s": 1000.0,
                                      "pros_ankle_angle_imitation_target_phase": .5,
                                      "pros_knee_angle_imitation_target_phase": .5}})
    tables = {}
    if with_tables:
        times = np.arange(0, n*.01, .001)
        tables = {"sea_torques": {"time": times, "SEA_Ankle_tau_spring": np.ones(len(times))*3,
                                 "SEA_Knee_tau_spring": np.sin(20*np.pi*times)},
                  "online_grf": {"time": times, "left_normal_force": np.ones(len(times))*100,
                                 "left_penetration": np.zeros(len(times))}}
    return {"id": "synthetic", "rows": rows, "trace": rows, "tables": tables,
            "summary": {"end_reason": "episode_time_limit", "steps": n,
                        "reward_config": cfg.to_dict(),
                        "episode_return": sum(compute_reward(row["reward_terms"], cfg)[0] for row in rows),
                        "grf_penetration_max_m": 0.0, "phase_valid_cycle_count": 2}}


def test_missing_sto_disables_ankle_family_but_retains_trace_knee_and_jerk():
    dataset = _dataset()
    cfg = RewardConfig.from_mapping(dataset["summary"]["reward_config"])
    terms, error = enrich_trace(dataset, cfg)
    assert error is None
    assert not family_available(("ankle_positive_torque",), terms)
    assert family_available(("knee_torque_rate",), terms)
    assert family_available(("knee_kinematic_jerk", "ankle_kinematic_jerk"), terms)
    assert all(t["ankle_positive_torque_available"] == 0 for t in terms)
    metrics = measure(dataset)
    assert metrics["ankle"] is None and metrics["knee"] is None
    assert metrics["contact"] is None


def test_partial_terminal_preserves_original_replay_and_excludes_stale_diagnostics():
    dataset = _dataset(n=3)
    final = copy.deepcopy(dataset["trace"][-1])
    final.update(time=.03, segment_completed=False, policy_segment_times=[.03,.04])
    final["reward_terms"].update(safety_loss=1.0, pros_knee_angle_sea_tau_spring_rate_rms_nm_s=999999.)
    dataset["trace"] = [*dataset["rows"], final]
    cfg = RewardConfig.from_mapping(dataset["summary"]["reward_config"])
    dataset["summary"]["episode_return"] += compute_reward(final["reward_terms"],cfg)[0]
    dataset["summary"]["end_reason"] = "grf_penetration_hard"
    dataset["summary"]["steps"] = 4
    _, pairs, discrepancy = source_replay(dataset)
    assert len(pairs) == 4 and discrepancy == 0
    terms, _ = enrich_trace(dataset,cfg)
    assert terms[-1]["tuning_segment_completed"] == 0
    assert terms[-1]["knee_torque_rate_available"] == 0
    assert terms[-1]["knee_torque_rate_loss"] == 0
    assert terms[-1]["safety_loss"] == 1
    assert family_available(("knee_torque_rate",),terms)
    metrics = measure(dataset)
    assert metrics["episode"]["completed"] is False
    assert metrics["episode"]["incomplete_rows"] == 1


def test_missing_fsm_does_not_become_zero_swing_positive_torque():
    dataset = _dataset(with_tables=True)
    for row in dataset["rows"]:
        row["phase_fsm"] = {}
    metrics = measure(dataset)
    assert metrics["ankle"]["positive_area_nm_s"] == pytest.approx(3.0)
    assert metrics["ankle"]["swing_positive_area_nm_s"] is None


def test_missing_grf_coverage_is_not_extrapolated_as_known_load():
    dataset = _dataset(with_tables=True)
    grf = dataset["tables"]["online_grf"]
    dataset["tables"]["online_grf"] = {key:value[:100] for key,value in grf.items()}
    try:
        metrics = measure(dataset)
    except ValueError:
        return
    assert metrics["ankle"]["loaded_positive_area_nm_s"] is None
    assert metrics["ankle"]["unloaded_positive_area_nm_s"] is None


@pytest.mark.parametrize("key", ["pros_knee_angle", "pros_knee_angle_vel"])
def test_nonfinite_trace_kinematics_fail_instead_of_reducing_jerk_samples(key):
    dataset = _dataset()
    dataset["rows"][50]["prosthetic_state"][key] = float("nan")
    with pytest.raises(ValueError):
        measure(dataset)


def test_highpass_rejects_nonfinite_and_resolves_high_frequency_content():
    times = np.arange(0, 4, .001)
    values = 2*np.sin(2*np.pi*times) + np.sin(2*np.pi*30*times)
    filtered = highpass(values,times,10)
    assert np.sqrt(np.mean(filtered[500:-500]**2)) == pytest.approx(2**-.5,abs=.02)
    values[20] = np.nan
    with pytest.raises(ValueError):
        highpass(values,times,10)
    assert highpass(np.ones(10),np.arange(10)*.01,10) is None


def test_causal_jerk_matches_online_midpoint_rule():
    times=np.array([0.0,.01,.03,.04])
    jerk=causal_jerk(times,3*times**2)
    assert np.isnan(jerk[:2]).all()
    np.testing.assert_allclose(jerk[2:],[6.,6.],rtol=1e-12)


def test_pair_filter_stop_excludes_future_tail_and_keeps_full_episode_metadata():
    before = _dataset(n=200, with_tables=True)
    after = copy.deepcopy(before)
    for row in after["rows"]:
        if row["time"] > 1.0 + 1e-8:
            row["prosthetic_state"]["pros_knee_angle"] = 10.0
    times = after["tables"]["sea_torques"]["time"]
    after["tables"]["sea_torques"]["SEA_Knee_tau_spring"][times > 1.0 + 1e-8] = 5000
    baseline = measure(before, [.15, 1.0], filter_stop=1.0)
    modified = measure(after, [.15, 1.0], filter_stop=1.0)
    assert baseline == modified
    assert modified["episode"]["duration_s"] == 2.0
    assert modified["episode"]["stop_s"] == 2.0
    assert modified["episode"]["incomplete_rows"] == 0


def test_missing_prescribed_phase_remains_explicitly_unavailable():
    dataset = _dataset()
    for row in dataset["rows"]:
        row["reward_terms"].pop("pros_knee_angle_imitation_target_phase")
    tracking = measure(dataset)["tracking"]["knee"]
    assert tracking["phase_available_samples"] == 0
    assert all(bin["actual_rmse_deg"] is None for bin in tracking["phase_bins"].values())


def test_best80_exact_source_replay_and_half_open_sea_alignment():
    folder=Path(__file__).resolve().parents[1]/"runs/rollout/MLP39_LEFT_pen2_2026-09-24_best80_deterministic"
    if not folder.is_dir():
        pytest.skip("historical local rollout unavailable")
    dataset=load_dataset(folder)
    cfg,_,delta=source_replay(dataset)
    assert abs(delta)<1e-10
    terms,error=enrich_trace(dataset,cfg)
    assert error<1e-5
    assert family_available(("ankle_positive_torque",),terms)
    assert family_available(("knee_torque_rate",),terms)
    measured=measure(dataset)
    # Constant 10 ms policy durations turn mean positive torque into area.
    area=sum(t["ankle_positive_torque_mean_nm"]*.01 for t in terms)
    assert area == pytest.approx(measured["ankle"]["positive_area_nm_s"],abs=1e-8)
    knee_losses=np.array([t["knee_kinematic_jerk_rad_s3"] for t in terms if t["knee_kinematic_jerk_available"]])
    assert np.sqrt(np.mean(knee_losses**2)) == pytest.approx(measured["kinematic_smoothness"]["knee_actual"]["jerk_rms_rad_s3"],rel=1e-10)
