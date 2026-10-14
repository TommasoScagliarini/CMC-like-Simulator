"""Paired, time-aligned response to one small action impulse; no policy updates."""
from __future__ import annotations
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
from matplotlib import pyplot as plt

HERE = Path(__file__).resolve().parent
PACKAGE = HERE.parents[2]
REPO = PACKAGE.parents[1]
RUNS = PACKAGE / "runs/rollout/general_method_diagnosis_2026-09-29"
PLOTS = REPO / "plot/09_29_2026_general_method_diagnosis/dynamic"
HASHES = {}
TIME_TOL = 1e-8
WINDOWS_S = (.05, .1, .2, .5, 1., 2., 4.)


def read(path):
    data = path.read_bytes()
    HASHES[path.relative_to(REPO).as_posix()] = hashlib.sha256(data).hexdigest()
    return data.decode()


def load(case):
    folder = RUNS / case
    return folder, json.loads(read(folder / "rollout_summary.json")), json.loads(read(folder / "rollout_policy_trace.json"))


def table(path):
    content = read(path).split("endheader", 1)[1].strip()
    return dict(zip(content.splitlines()[0].split(), np.loadtxt(io.StringIO(content), skiprows=1).T))


def arr(trace, kind, key):
    return np.asarray([r[kind][key] for r in trace], dtype=float)


def first(mask, times):
    indices = np.flatnonzero(mask)
    return float(times[indices[0]]) if len(indices) else None


def completed_prefix(trace):
    """Only a contiguous prefix of physically completed segments is comparable."""
    for i, row in enumerate(trace):
        start, end = np.asarray(row["policy_segment_times"], float)[[0,-1]]
        actual = float(row["time"])
        complete = (row.get("segment_completed", True) and end > start + TIME_TOL
                    and actual > start + TIME_TOL and abs(actual-end) <= TIME_TOL)
        if not complete:
            if i != len(trace)-1:
                raise AssertionError("Incomplete/nonadvancing segment before the final trace row")
            return i
    return len(trace)


def completion_status(summary, protocol, complete_count):
    requested = int(protocol["steps_requested"])
    reason = summary.get("end_reason")
    full = (complete_count >= 500 and int(summary["steps"]) == 500
            and reason == "episode_time_limit" and not summary.get("terminated", False))
    normal_reasons = {None, "", "episode_time_limit", "max_steps", "max_steps_reached"}
    failed = bool(summary.get("terminated", False) or reason not in normal_reasons)
    reached = bool(complete_count >= requested and not failed)
    status = ("full_episode_completed" if full else "requested_horizon_reached" if reached
              else "physical_failure" if failed else "stopped_before_requested_horizon")
    return {"steps_requested": requested, "steps_recorded": int(summary["steps"]),
            "physically_completed_steps": int(complete_count), "requested_horizon_reached": reached,
            "full_episode_completed_500": bool(full), "failed_during_requested_horizon": failed,
            "early_failure": bool(failed and complete_count < requested),
            "ended_before_requested_horizon": bool(complete_count < requested),
            "evaluation_status": status, "end_reason": reason}


def response_windows(signals, elapsed, decision_elapsed, applied):
    """Report a named horizon only when both kinds of time cover it fully."""
    windows, omitted = {}, []
    max_physical, max_decision = float(elapsed[-1]), float(decision_elapsed[-1])
    for horizon in WINDOWS_S:
        if horizon > min(max_physical,max_decision) + TIME_TOL:
            omitted.append(horizon)
            continue
        physical_mask = (elapsed > TIME_TOL) & (elapsed <= horizon+TIME_TOL)
        decision_mask = (decision_elapsed > TIME_TOL) & (decision_elapsed <= horizon+TIME_TOL)
        if not physical_mask.any() or not decision_mask.any():
            omitted.append(horizon)
            continue
        window = {}
        for key, values in signals.items():
            mask = decision_mask if key in {"mean_norm", "applied_command_norm"} else physical_mask
            selected = values[mask]
            window[key] = {"rms":float(np.sqrt(np.mean(selected**2))), "max_abs":float(np.max(np.abs(selected)))}
        window.update(samples=int(physical_mask.sum()), mean_samples=int(decision_mask.sum()),
                      last_physical_response_time_s=float(elapsed[physical_mask][-1]),
                      last_decision_response_time_s=float(decision_elapsed[decision_mask][-1]),
                      mean_max_gain_over_applied_impulse=float(np.max(signals["mean_norm"][decision_mask])/np.linalg.norm(applied)),
                      applied_command_max_gain_over_applied_impulse=float(np.max(signals["applied_command_norm"][decision_mask])/np.linalg.norm(applied)))
        windows[str(horizon)] = window
    return windows, omitted


def load_governor():
    path = HERE.parent/"governor/analyze_governor.py"
    read(path)
    spec = importlib.util.spec_from_file_location("dynamic_exact_reference_governor",path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def first_response_explanation(governor, case, endpoint_states, b, p, records, protocol):
    """Replay the impulse segment from the same exact pre-impulse filter state."""
    index = int(protocol["impulse_index_zero_based"])
    assert index > 0
    pre = endpoint_states[index-1]
    limits, cfg = case["limits"], case["cfg"]
    assert np.all(np.isinf(case["slew"]))
    t0,t1 = np.asarray(p[index]["policy_segment_times"])[[0,-1]]
    n = max(2,int(np.ceil((t1-t0)/.001)))
    times = np.linspace(t0,t1,n+1)
    arms = {}
    for label,trace in (("baseline",b),("impulse",p)):
        row = trace[index]
        cmd = np.asarray(row["policy_segment_values"][-1],float)
        kin = governor.Kinematics(governor.Base(),governor.JOINTS,
                ref_model=cfg["pros_ref_model"],ref_lpf_cutoff_hz=cfg["pros_ref_cutoff_hz"],
                ref_governor_enable=cfg["pros_ref_governor"],ref_velocity_limits=limits[0],
                ref_acceleration_limits=limits[1],ref_jerk_limits=limits[2],control_dt=.001)
        kin._filt_q,kin._filt_v,kin._filt_a = [pre[k:k+2].copy() for k in (0,2,4)]
        kin.set_segment(np.array([t0,t1]),np.array([pre[:2],cmd]),np.array([pre[2:4],(cmd-pre[:2])/(t1-t0)]))
        states = np.c_[kin._segment_spline(times),kin._segment_spline.derivative(1)(times),
                       kin._segment_spline.derivative(2)(times)]
        wc = 2*np.pi*cfg["pros_ref_cutoff_hz"]
        raw_jerk = wc**3*(cmd-states[:-1,:2])-2*wc**2*states[:-1,2:4]-2*wc*states[:-1,4:6]
        limited = np.abs(raw_jerk)>limits[2]+1e-8
        # Production diagnostic includes the two zero-initialized entries at t0.
        np.testing.assert_allclose(limited.sum()/(2*(n+1)),row["reference_governor_diagnostics"]["jerk_limit_fraction"],atol=1e-12,rtol=0)
        observed = np.asarray([row["prosthetic_state"][f"pros_{j}_angle{s}"] for s in
            ("_served_ref","_served_ref_vel","_served_ref_accel") for j in ("knee","ankle")])
        error = np.abs(states[-1]-observed)
        assert max(error)<5e-5
        arms[label] = {"raw_action":row["raw_policy_action"],"applied_action":row["applied_policy_action"],
            "accepted_endpoint_rad":cmd.tolist(),"served_endpoint_q_v_a":states[-1].tolist(),
            "observed_served_endpoint_q_v_a":observed.tolist(),"replay_abs_error_q_v_a":error.tolist(),
            "raw_jerk_min_by_joint_rad_s3":raw_jerk.min(axis=0).tolist(),
            "raw_jerk_max_by_joint_rad_s3":raw_jerk.max(axis=0).tolist(),
            "jerk_limited_substeps_by_joint":limited.sum(axis=0).tolist(),
            "substeps":n,"recorded_governor_diagnostics":row["reference_governor_diagnostics"]}
    delta = np.asarray(arms["impulse"]["served_endpoint_q_v_a"])-np.asarray(arms["baseline"]["served_endpoint_q_v_a"])
    observed_delta = np.asarray(arms["impulse"]["observed_served_endpoint_q_v_a"])-np.asarray(arms["baseline"]["observed_served_endpoint_q_v_a"])
    next_mean = next_applied = None
    if len(p)>index+1:
        next_mean = (np.asarray(records[index+1]["policy_mean"])-np.asarray(b[index+1]["raw_policy_action"])).tolist()
        next_applied = (np.asarray(p[index+1]["applied_policy_action"])-np.asarray(b[index+1]["applied_policy_action"])).tolist()
    return {"impulse_index_zero_based":index,"time_start_s":float(t0),"time_end_s":float(t1),
        "joint_order":["knee","ankle"],"state_order":["q_knee","q_ankle","qdot_knee","qdot_ankle","qddot_knee","qddot_ankle"],
        "pre_governor_state_q_v_a":pre.tolist(),"cutoff_hz":cfg["pros_ref_cutoff_hz"],
        "limits_velocity_acceleration_jerk":limits.tolist(),"arms":arms,
        "exact_replay_served_delta_q_v_a":delta.tolist(),"recorded_served_delta_q_v_a":observed_delta.tolist(),
        "first_next_decision_mean_delta":next_mean,"first_next_decision_applied_command_delta":next_applied,
        "interpretation_limit":"Same initial governor state within each policy. LEFT/RIGHT have different states and active constraints; "
            "equal raw/applied action impulses need not produce equal reference disturbances. "
            "Exact replay explains only the first segment's reference response, not later physical feedback or global stability."}


def main():
    PLOTS.mkdir(parents=True, exist_ok=True)
    results, pending, explanations, governor_cases = {}, [], {}, {}
    governor = load_governor()
    read(Path(__file__))
    if (HERE/"README.md").exists():
        read(HERE/"README.md")
    for source in ("LEFT", "RIGHT"):
        baseline, bs, bt = load(source + "_baseline")
        baseline_receipt = json.loads(read(baseline / "diagnostic_receipt.json"))
        assert baseline_receipt["checkpoint_unchanged"]
        assert baseline_receipt["comparison_to_archive_prefix_steps"] == 500
        assert all(v["exact"] and v["max_abs"] == 0 for v in baseline_receipt["archive_replay"].values())
        baseline_valid_count = completed_prefix(bt)
        assert baseline_valid_count == 500 and bs["end_reason"] == "episode_time_limit"
        baseline_torque = table(baseline / "sim_outputs/rollout_episode_sea_torques.sto")
        for mode in ("impulse_knee", "impulse_ankle"):
            folder = RUNS / f"{source}_{mode}"
            if not (folder / "diagnostic_receipt.json").exists():
                pending.append(folder.name)
                continue
            _, summary, trace = load(folder.name)
            receipt = json.loads(read(folder / "diagnostic_receipt.json"))
            protocol = json.loads(read(folder / "diagnostic_protocol.json"))
            rec = json.loads(read(folder / "action_interventions.json"))
            assert receipt["checkpoint_unchanged"]
            assert len(rec) == len(trace) == summary["steps"]
            valid_count = completed_prefix(trace)
            n = min(valid_count, baseline_valid_count)
            b, p = bt[:n], trace[:n]
            index = int(protocol["impulse_index_zero_based"])
            assert protocol["source"] == source and protocol["mode"] == mode
            result = {"source":source, "mode":mode, "common_steps":n,
                      "partial_terminal_rows_excluded":len(trace)-valid_count,
                      **completion_status(summary,protocol,valid_count),
                      "baseline_end_reason":bs["end_reason"],
                      "scope":"Paired finite-amplitude response of each frozen policy on its own nominal trajectory; "
                              "not an infinitesimal Jacobian, general stability proof, or matching physical states across LEFT and RIGHT."}
            if n <= index:
                for key in ("actor_observation_vector_before", "raw_policy_action", "policy_segment_values"):
                    np.testing.assert_array_equal([r[key] for r in p], [r[key] for r in b])
                result.update(response_available=False, reason_no_response="No physically completed impulse segment",
                              prefix_exact_steps=n, windows={})
                results[folder.name] = result
                continue
            t = np.array([r["time"] for r in p])
            np.testing.assert_array_equal(t, [r["time"] for r in b])
            np.testing.assert_array_equal([r["policy_segment_times"] for r in p], [r["policy_segment_times"] for r in b])
            assert np.all(np.diff(t) > 0)
            t0 = float(p[index]["policy_segment_times"][0])
            elapsed = t-t0
            decision_elapsed = np.asarray([r["policy_segment_times"][0] for r in p])-t0
            # Inputs and generated commands must match exactly before the intervention.
            for key in ("actor_observation_vector_before", "raw_policy_action", "policy_segment_values"):
                np.testing.assert_array_equal([r[key] for r in p[:index]], [r[key] for r in b[:index]])
            np.testing.assert_array_equal(p[index]["actor_observation_vector_before"],b[index]["actor_observation_vector_before"])
            np.testing.assert_array_equal([r["command"] for r in rec[:n]], [r["raw_policy_action"] for r in p])
            np.testing.assert_array_equal(np.clip([r["command"] for r in rec[:n]],-1,1), [r["applied_policy_action"] for r in p])
            delta_mean = np.asarray([r["policy_mean"] for r in rec[:n]])-np.asarray([r["raw_policy_action"] for r in b])
            # The intervention changes the command, not the mean at its first decision.
            np.testing.assert_array_equal(delta_mean[index], [0., 0.])
            applied = np.asarray(rec[index]["clipped_intervention"])
            if not np.any(applied):
                raise ValueError("Impulse fully clipped; cannot diagnose its propagation")
            interventions=np.asarray([r["intervention"] for r in rec])
            assert np.count_nonzero(np.any(interventions != 0, axis=1))==1
            expected_impulse = np.zeros(2); expected_impulse[0 if mode == "impulse_knee" else 1] = protocol["impulse_normalized"]
            np.testing.assert_allclose(interventions[index],expected_impulse,atol=1e-7,rtol=0)
            np.testing.assert_allclose(applied,expected_impulse,atol=1e-7,rtol=0)
            delta_applied = np.asarray([r["applied_policy_action"] for r in p])-np.asarray([r["applied_policy_action"] for r in b])
            signals = {"mean_norm": np.linalg.norm(delta_mean, axis=1),
                       "applied_command_norm":np.linalg.norm(delta_applied,axis=1)}
            for joint in ("knee", "ankle"):
                coord = f"pros_{joint}_angle"
                for field, suffix, factor in (("q_deg", "", 180/np.pi), ("served_deg", "_served_ref", 180/np.pi),
                                              ("served_vel", "_served_ref_vel", 1), ("served_accel", "_served_ref_accel", 1)):
                    signals[f"{joint}_{field}"] = factor*(arr(p, "prosthetic_state", coord+suffix)-arr(b, "prosthetic_state", coord+suffix))
            for key, field in (("force_n", "grf_ankle_moment_flip_force_n"), ("grf_moment_nm", "grf_ankle_moment_flip_tau_nm"),
                               ("penetration_mm", "grf_penetration_m")):
                signals[key] = (arr(p, "reward_terms", field)-arr(b, "reward_terms", field))*(1000 if key.endswith("mm") else 1)
            fsm_keys = ("state_id", "valid_hs_count", "valid_to_count", "valid_cycle_count", "invalid_event_count", "resync_count")
            mismatch = np.zeros(n, bool)
            for key in fsm_keys:
                mismatch |= arr(p, "phase_fsm", key) != arr(b, "phase_fsm", key)
            windows, omitted = response_windows(signals,elapsed,decision_elapsed,applied)
            if source not in governor_cases:
                case = governor.load_case(source)
                endpoint_states,_,_,replay_verification = governor.replay(case,compare=True)
                governor_cases[source] = (case,endpoint_states,replay_verification)
            case,endpoint_states,replay_verification = governor_cases[source]
            explanation = first_response_explanation(governor,case,endpoint_states,b,p,rec,protocol)
            explanation.update(source=source,mode=mode,baseline_trace=str((baseline/"rollout_policy_trace.json").relative_to(REPO)),
                               impulse_trace=str((folder/"rollout_policy_trace.json").relative_to(REPO)),
                               full_nominal_governor_replay_verification=replay_verification)
            explanations[folder.name] = explanation
            torque = table(folder / "sim_outputs/rollout_episode_sea_torques.sto")
            assert np.all(np.diff(torque["time"])>0) and np.all(np.diff(baseline_torque["time"])>0)
            common = (torque["time"]>=baseline_torque["time"][0]) & (torque["time"]<=min(t[-1],baseline_torque["time"][-1]))
            ts = torque["time"][common]
            torque_deltas = {joint: torque[f"SEA_{joint.title()}_tau_spring"][common]-np.interp(ts,baseline_torque["time"],baseline_torque[f"SEA_{joint.title()}_tau_spring"]) for joint in ("knee","ankle")}
            for horizon,window in windows.items():
                tm = ((ts-t0)>TIME_TOL)&((ts-t0)<=float(horizon)+TIME_TOL)
                for joint,values in torque_deltas.items():
                    window[f"{joint}_tau_spring_nm"] = {"rms":float(np.sqrt(np.mean(values[tm]**2))),
                                                          "max_abs":float(np.max(np.abs(values[tm])))}
                window["torque_samples"] = int(tm.sum())
            result.update({"response_available":True,"time_of_action_s":t0,"prefix_exact_steps":index,
                      "applied_impulse_normalized":applied.tolist(),
                      "observed_common_physical_response_s":float(elapsed[-1]),
                      "observed_common_decision_response_s":float(decision_elapsed[-1]),
                      "observed_common_torque_response_s":float(ts[-1]-t0),
                      "first_fsm_mismatch_after_impulse_s":first(mismatch & (elapsed>TIME_TOL),elapsed),
                      "first_mean_norm_above_impulse_s":first((signals["mean_norm"]>np.linalg.norm(applied)) & (decision_elapsed>1e-8),decision_elapsed),
                      "first_applied_command_norm_above_impulse_s":first((signals["applied_command_norm"]>np.linalg.norm(applied)) & (decision_elapsed>TIME_TOL),decision_elapsed),
                      "first_joint_difference_above_0p1deg_s":first((np.maximum(abs(signals["knee_q_deg"]),abs(signals["ankle_q_deg"]))>.1)&(elapsed>TIME_TOL),elapsed),
                      "first_contact_force_difference_above_1N_s":first((abs(signals["force_n"])>1)&(elapsed>TIME_TOL),elapsed),
                      "windows":windows,"unobserved_requested_window_horizons_s":omitted})
            results[folder.name]=result
            fig, axs=plt.subplots(5,1,figsize=(12,12),sharex=True,layout="constrained")
            axs[0].plot(decision_elapsed,delta_mean[:,0],label="Knee mean (decision time)");axs[0].plot(decision_elapsed,delta_mean[:,1],label="Ankle mean (decision time)")
            axs[0].set_ylabel("Delta action mean")
            axs[1].plot(elapsed,signals["knee_q_deg"],label="Knee actual");axs[1].plot(elapsed,signals["ankle_q_deg"],label="Ankle actual")
            axs[1].set_ylabel("Delta q [deg]")
            axs[2].plot(elapsed,signals["knee_served_accel"],label="Knee served acceleration");axs[2].plot(elapsed,signals["ankle_served_accel"],label="Ankle served acceleration")
            axs[2].set_ylabel("Delta accel [rad/s²]")
            for joint in ("knee","ankle"):axs[3].plot(ts-t0,torque_deltas[joint],label=joint)
            axs[3].set_ylabel("Delta SEA torque [Nm]")
            axs[4].plot(elapsed,signals["force_n"],label="Contact force difference")
            axs[4].set_ylabel("Delta normal force [N]");axs[4].set_xlabel("Time since impulse [s]")
            for ax in axs:
                ax.axvline(0,color="black",linestyle=":");ax.grid(alpha=.2);ax.legend(loc="upper right",fontsize=8);ax.set_xlim(-.1, min(2.,elapsed[-1]))
                if result["first_fsm_mismatch_after_impulse_s"] is not None: ax.axvline(result["first_fsm_mismatch_after_impulse_s"],color="red",linestyle="--",alpha=.5)
            fig.suptitle(f"{source}: {mode}, {protocol['impulse_normalized']:+g} for 10 ms; red = first FSM divergence")
            fig.savefig(PLOTS/f"{folder.name}.png",dpi=150);plt.close(fig)
    HASHES.update(governor.HASHES)
    for path,expected in HASHES.items():
        assert hashlib.sha256((REPO/path).read_bytes()).hexdigest() == expected, f"Input changed: {path}"
    (HERE/"metrics.json").write_text(json.dumps(results,indent=2,allow_nan=False)+"\n")
    (HERE/"first_response_explanation.json").write_text(json.dumps({"status":"preliminary" if pending else "complete",
        "pending_cases":pending,"cases":explanations},indent=2,allow_nan=False)+"\n")
    (HERE/"input_hashes.json").write_text(json.dumps(HASHES,indent=2)+"\n")
    (HERE/"analysis_verification.json").write_text(json.dumps({"input_hashes_unchanged":True,"input_count":len(HASHES),
        "analyzed_cases":list(results),"pending_cases":pending,"no_physical_rollouts":True,"no_training":True,
        "metrics_sha256":hashlib.sha256((HERE/"metrics.json").read_bytes()).hexdigest(),
        "first_response_explanation_sha256":hashlib.sha256((HERE/"first_response_explanation.json").read_bytes()).hexdigest()},indent=2)+"\n")
    print(json.dumps({k:{f:v.get(f) for f in ("evaluation_status","requested_horizon_reached","full_episode_completed_500",
        "observed_common_physical_response_s","observed_common_decision_response_s","first_mean_norm_above_impulse_s",
        "first_fsm_mismatch_after_impulse_s","first_contact_force_difference_above_1N_s")} for k,v in results.items()},indent=2))


if __name__=="__main__":main()
