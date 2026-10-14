"""Offline, reproducible masking rollout diagnosis; never changes source runs.

Run from any directory with Python providing NumPy/SciPy/Matplotlib/Gymnasium.
Optionally add completed controls with --extra label=/absolute/rollout/path.
The simulator, actor, checkpoints and training are never executed here.
"""
from __future__ import annotations

import argparse
import hashlib
import io
import json
import os
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
PACKAGE = HERE.parents[2]
TG = PACKAGE.parent
REPO = TG.parent
PLOTS = REPO / "plot/10_01_2026_masking_diagnosis"
for key in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "VECLIB_MAXIMUM_THREADS"):
    os.environ[key] = "1"
os.environ.setdefault("MPLCONFIGDIR", str(HERE / "mpl_cache"))
sys.path.insert(0, str(TG))

import numpy as np
from scipy.signal import butter, find_peaks, sosfiltfilt, welch
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from unified_MLP_39.reward_function import RewardConfig, compute_reward

SOURCES = {
    "cp90_visible": PACKAGE / "runs/rollout/unified_MLP_39/runs/rollout/MLP39_LEFT_fresh_best90_2026-09-30_deterministic",
    "cp112_hidden": PACKAGE / "runs/rollout/MLP39_LEFT_masking_best112_overall_hidden_2026-10-01",
    "cp184_hidden": PACKAGE / "runs/rollout/MLP39_LEFT_masking_best184_last25_hidden_2026-10-01",
}
LABELS = {"cp90_visible": "CP90 visible", "cp112_hidden": "CP112 hidden", "cp184_hidden": "CP184 hidden"}
COLORS = {"cp90_visible": "#236b4d", "cp112_hidden": "#cc7832", "cp184_hidden": "#8268b2",
          "cp90_hidden": "#76a982", "cp112_visible": "#ac492e", "cp184_visible": "#326ab4"}
WINDOW = (13.19, 17.94)  # 200 ms after physical initialization; common end.
LAG_WINDOW = (13.50, 17.49)  # Fixed support for every candidate lag +/- 0.5 s.
HASHES = {}


def read(path):
    raw = path.read_bytes()
    HASHES[str(path.relative_to(REPO))] = hashlib.sha256(raw).hexdigest()
    return raw.decode("utf-8")


def table(folder, suffix):
    text = read(folder / "sim_outputs" / f"rollout_episode_{suffix}.sto")
    body = text.split("endheader", 1)[1].strip()
    return dict(zip(body.splitlines()[0].split(), np.loadtxt(io.StringIO(body), skiprows=1).T))


def rms(values):
    return float(np.sqrt(np.mean(np.asarray(values) ** 2)))


def corr(a, b):
    return float(np.corrcoef(a, b)[0, 1]) if np.std(a) > 1e-12 and np.std(b) > 1e-12 else None


def mask(time, window=WINDOW):
    return (time >= window[0] - 1e-8) & (time <= window[1] + 1e-8)


def lag_audit(time, actual, target):
    if time[-1] < LAG_WINDOW[1] + .5 - 1e-8:
        return {"available": False, "reason": "Early termination; fixed lag support and +/-0.5 s search unavailable."}
    keep = mask(time, LAG_WINDOW)
    t, a = time[keep], actual[keep]
    candidates = []
    for delay in np.arange(-0.5, 0.5001, .001):
        reference = np.interp(t - delay, time, target)
        candidates.append((rms(a-reference), corr(a, reference), float(delay), reference))
    rmse, correlation, delay, reference = min(candidates, key=lambda row: row[0])
    corr_best = max(candidates, key=lambda row: row[1] if row[1] is not None else -2)
    affine = np.linalg.lstsq(np.c_[reference, np.ones_like(reference)], a, rcond=None)[0]
    return {
        "fixed_support_s": list(LAG_WINDOW), "zero_lag_rmse_deg": rms(a-target[keep]),
        "best_rmse_delay_s": delay, "best_lag_rmse_deg": rmse,
        "best_lag_correlation": correlation,
        "max_correlation_delay_s": corr_best[2], "max_correlation": corr_best[1],
        "best_rmse_lag_affine_target_gain": float(affine[0]),
        "best_rmse_lag_affine_bias_deg": float(affine[1]),
        "best_rmse_lag_affine_residual_rmse_deg": rms(a-(affine[0]*reference+affine[1])),
        "delay_definition": "actual(t) compared with target(t-delay); positive means actual lags target",
        "limit": "Descriptive whole-window translation, not proof of a causal or constant delay.",
    }


def reward_budget(trace, summary, window):
    cfg = RewardConfig.from_mapping(summary["reward_config"])
    components = [compute_reward(row["reward_terms"], cfg, reference=row["policy_segment_values"])[1] for row in trace]
    returned = sum(row["reward"] for row in components)
    np.testing.assert_allclose(returned, summary["episode_return"], rtol=0, atol=1e-7)
    all_indices = np.ones(len(trace), dtype=bool)
    time = np.asarray([r["time"] for r in trace])
    knee_target = -np.degrees([r["imitation_target_q"]["pros_knee_angle"] for r in trace])
    budgets = {}
    for name, keep in {"full_episode": all_indices, "common_window": mask(time,window),
                       "target_knee_over35deg": mask(time,window) & (knee_target > 35),
                       "target_knee_at_most35deg": mask(time,window) & (knee_target <= 35)}.items():
        cc = [c for c, k in zip(components, keep) if k]
        rr = [r["reward_terms"] for r, k in zip(trace, keep) if k]
        s = lambda key: float(sum(c.get(key, 0) for c in cc))
        actual = cfg.blend_imitation * s("imitation_score")
        served = cfg.blend_served_imitation * s("served_imitation_score")
        positive = actual + served
        penalty = s("penalty")
        clipping = s("reward_base") - positive + penalty
        penalty_costs = {key: float(getattr(cfg, key) * sum(row.get(key.replace("_weight", "_loss"), 0) for row in rr))
                         for key in ("effort_weight", "smoothness_weight", "saturation_weight", "command_rate_weight",
                                     "sea_saturation_weight", "sea_torque_error_weight", "sea_tau_spring_effort_weight",
                                     "sea_tau_spring_rate_weight", "sea_motor_speed_weight", "sea_motor_accel_weight",
                                     "sea_motor_power_weight", "policy_action_clip_weight")}
        np.testing.assert_allclose(sum(penalty_costs.values()), penalty, rtol=0, atol=1e-7)
        budgets[name] = {
            "steps": int(keep.sum()), "return": s("reward"), "weighted_actual_score": actual,
            "weighted_served_score": served, "mixed_imitation_before_penalties": positive,
            "base_penalties": penalty, "base_penalty_components": penalty_costs,
            "base_clip_adjustment": clipping, "base_clip_low_steps": sum(c["reward_base"] <= cfg.clip_low+1e-12 for c in cc),
            "reward_base": s("reward_base"), "additional_penalties": s("reward_base")-s("reward"),
            "grf_penetration_term": s("grf_penetration_term"), "grf_ankle_moment_flip_term": s("grf_ankle_moment_flip_term"),
            "oob_term": s("oob_term"), "mean_imitation_score": s("imitation_score")/len(cc) if cc else None,
            "mean_served_imitation_score": s("served_imitation_score")/len(cc) if cc else None,
            "mean_sound_imitation_loss": float(np.mean([r["sound_imitation_loss"] for r in rr])) if rr else None,
            "mean_served_imitation_loss": float(np.mean([r["served_imitation_loss"] for r in rr])) if rr else None,
            "command_rate_subcosts": {key: float(cfg.command_rate_weight/6 * sum(r.get(key, 0) for r in rr)) for key in
                                      ("segment_delta_loss", "qdot_ref_loss", "qddot_ref_loss", "jerk_ref_loss", "reference_governor_loss", "u_rate_loss")},
        }
        np.testing.assert_allclose(sum(budgets[name]["command_rate_subcosts"].values()), penalty_costs["command_rate_weight"], atol=1e-8, rtol=0)
    return budgets, components


def analyze(name, folder):
    summary = json.loads(read(folder / "rollout_summary.json"))
    trace = json.loads(read(folder / "rollout_policy_trace.json"))
    tables = {suffix: table(folder, suffix) for suffix in ("sea_torques", "sea_diagnostics", "kinematics", "kinematics_reference", "online_grf")}
    time = np.asarray([r["time"] for r in trace])
    last_physical = min(t["time"][-1] for t in tables.values())
    window = (WINDOW[0], min(WINDOW[1], np.floor((last_physical+1e-8)*100)/100))
    keep = mask(time,window)
    assert time.min() <= window[0] and keep.sum() > 5
    reward, reward_series = reward_budget(trace, summary, window)
    result = {
        "source": str(folder.relative_to(REPO)), "checkpoint": summary["checkpoint"],
        "steps": len(trace), "episode_return": summary["episode_return"],
        "visible_steps": sum(r["target_availability"]["visible"] for r in trace),
        "window_s": list(window), "common_window_complete": bool(window[1] == WINDOW[1]), "reward": reward, "joints": {},
        "fsm": {k:summary.get(k) for k in ("phase_valid_hs_count", "phase_valid_to_count", "phase_valid_cycle_count", "invalid_event_count", "end_reason")},
        "completed_cycles": [{"time":r["time"], **{k:r["phase_fsm"][k] for k in ("last_period_s", "last_stance_fraction", "cycle_knee_excursion_rad", "cycle_ankle_excursion_rad")}}
                             for r in trace if r["phase_fsm"]["cycle_completed_this_step"]],
        "action_clipped_fraction": summary["action_clipped_fraction"],
        "action_clipped_steps": summary["action_clipped_steps"],
        "reference_jerk_limit_fraction_mean": float(np.mean([r["reward_terms"]["reference_jerk_limit_fraction"] for r in trace])),
        "mean_command_endpoint_step_rad": {},
    }
    series = {"time": time, "reward": reward_series, "tables": tables, "joints": {}}
    for joint in ("knee", "ankle"):
        coord = f"pros_{joint}_angle"
        sign = -1 if joint == "knee" else 1
        actual = sign*np.degrees([r["prosthetic_state"][coord] for r in trace])
        served = sign*np.degrees([r["prosthetic_state"][coord+"_served_ref"] for r in trace])
        target = sign*np.degrees([r["imitation_target_q"][coord] for r in trace])
        velocity = sign*np.degrees([r["prosthetic_state"][coord+"_vel"] for r in trace])
        target_velocity = sign*np.degrees([r["imitation_target_qdot"][coord] for r in trace])
        action_index = 0 if joint == "knee" else 1
        command = sign*np.degrees([r["policy_segment_values"][-1][action_index] for r in trace])
        series["joints"][joint] = {"actual": actual, "served": served, "target": target, "command": command}
        metric = {"rmse_actual_target_deg": rms((actual-target)[keep]),
                  "rmse_served_target_deg": rms((served-target)[keep]),
                  "rmse_actual_served_deg": rms((actual-served)[keep]),
                  "rmse_actual_target_velocity_deg_s": rms((velocity-target_velocity)[keep]),
                  "rmse_command_target_deg": rms((command-target)[keep]),
                  "corr_actual_target": corr(actual[keep], target[keep]),
                  "actual_mean_bias_vs_target_deg": float(np.mean((actual-target)[keep])),
                  "lag": lag_audit(time, actual, target)}
        for channel, values in series["joints"][joint].items():
            metric[channel] = {"min_deg": float(values[keep].min()), "max_deg": float(values[keep].max()),
                               "excursion_deg": float(np.ptp(values[keep])), "sd_deg": float(np.std(values[keep]))}
        target_peaks, _ = find_peaks(target, prominence=10 if joint == "knee" else 8, distance=100)
        metric["at_target_peaks"] = [{"time_s": float(time[i]), "target_deg": float(target[i]), "actual_deg": float(actual[i]), "served_deg": float(served[i])}
                                     for i in target_peaks if keep[i]]
        metric["per_elapsed_second_rmse_actual_target_deg"] = [rms((actual-target)[(time>=13+i-1e-8)&(time<14+i-1e-8)]) if np.any((time>=13+i-1e-8)&(time<14+i-1e-8)) else None for i in range(5)]
        result["joints"][joint] = metric
        result["mean_command_endpoint_step_rad"][joint] = float(np.mean(np.abs(np.diff(np.radians(command)))))
    torque = tables["sea_torques"]
    t = torque["time"]
    dt = float(np.median(np.diff(t)))
    assert np.max(np.abs(np.diff(t)-dt)) < 1e-7
    tk = mask(t,window)
    result["torque"] = {}
    for joint in ("knee", "ankle"):
        raw = torque[f"SEA_{joint.title()}_tau_spring"]
        sig = (-1 if joint == "knee" else 1)*raw
        result["torque"][joint] = {"min_nm":float(sig[tk].min()), "max_nm":float(sig[tk].max()),
                                    "rms_nm":rms(sig[tk]), "positive_area_nm_s":float(np.trapezoid(np.maximum(sig[tk],0),t[tk])),
                                    "negative_area_nm_s":float(np.trapezoid(np.minimum(sig[tk],0),t[tk])),
                                    "positive_fraction":float(np.mean(sig[tk]>0)),
                                    "positive_over8_fraction":float(np.mean(sig[tk]>8)), "highpass_rms_nm": {}}
        for cutoff in (10,20):
            high = sosfiltfilt(butter(4, cutoff, btype="highpass", fs=1/dt, output="sos"), sig)[tk]
            f, psd = welch(high, fs=1/dt, nperseg=min(2048,len(high)))
            band = (f>=cutoff)&(f<=200)
            result["torque"][joint]["highpass_rms_nm"][str(cutoff)] = rms(high)
            result["torque"][joint][f"highpass_{cutoff}_peak_frequency_hz"] = float(f[band][np.argmax(psd[band])])
        diagnostics=tables["sea_diagnostics"]
        dk=mask(diagnostics["time"],window)
        channel_hf={}
        for channel in ("tau_input_plugin","tau_ref","cascade_inner_p_cmd","cascade_inner_i_cmd"):
            values=diagnostics[f"SEA_{joint.title()}_{channel}"]
            diag_dt=float(np.median(np.diff(diagnostics["time"])))
            channel_hf[channel]={}
            for cutoff in (10,20):
                hp=sosfiltfilt(butter(4,cutoff,btype="highpass",fs=1/diag_dt,output="sos"),values)
                low=sosfiltfilt(butter(4,cutoff,btype="lowpass",fs=1/diag_dt,output="sos"),values)
                channel_hf[channel][f"highpass_{cutoff}_rms_nm"]=rms(hp[dk])
                channel_hf[channel][f"residual_after_lowpass_{cutoff}_rms_nm"]=rms((values-low)[dk])
        result["torque"][joint]["controller_channels"]=channel_hf
        result["torque"][joint]["highpass20_per_actual_excursion_nm_per_deg"]=result["torque"][joint]["highpass_rms_nm"]["20"]/result["joints"][joint]["actual"]["excursion_deg"]
    grf = tables["online_grf"]
    gk = mask(grf["time"],window)
    normal, penetration = grf["left_normal_force"], grf["left_penetration"]
    result["contact"] = {"left_normal_impulse_ns":float(np.trapezoid(normal[gk],grf["time"][gk])),
                         "left_force_max_n":float(normal[gk].max()),
                         "left_force_mean_n":float(np.mean(normal[gk])),
                         "left_contact_fraction":float(np.mean(grf["left_in_contact"][gk]>.5)),
                         "left_loaded_over50n_fraction":float(np.mean(normal[gk]>=50)),
                         "left_penetration_max_mm":float(1000*penetration[gk].max()),
                         "left_penetration_whole_max_mm":float(1000*penetration.max())}
    (HERE / f"{name}_metrics.json").write_text(json.dumps(result,indent=2,allow_nan=False)+"\n")
    return result, series


def plot_comparison(metrics, signals):
    original = [n for n in SOURCES if n in signals]
    groups = [(original, "rollout_kinematics_original_three.png")]
    for cp in (90,112,184):
        names = [n for n in (f"cp{cp}_visible",f"cp{cp}_hidden") if n in signals]
        if len(names)==2:
            groups.append((names,f"rollout_kinematics_cp{cp}_visibility.png"))
    saved = []
    for names, filename in groups:
        plot_window=(WINDOW[0],min(metrics[n]["window_s"][1] for n in names))
        fig, axes = plt.subplots(3,2,figsize=(15,10),sharex=True,layout="constrained")
        for j,joint in enumerate(("knee","ankle")):
            base = signals[names[0]]
            axes[0,j].plot(base["time"],base["joints"][joint]["target"],color="black",lw=2,label="Prescribed LEFT target")
            axes[1,j].plot(base["time"],base["joints"][joint]["target"],color="black",lw=2,label="Prescribed LEFT target")
            for name in names:
                s = signals[name]; data=s["joints"][joint]; color=COLORS[name]
                axes[0,j].plot(s["time"],data["actual"],color=color,label=LABELS.get(name,name))
                axes[1,j].plot(s["time"],data["served"],color=color,label=LABELS.get(name,name))
                axes[2,j].plot(s["time"],data["actual"]-data["target"],color=color,label=LABELS.get(name,name))
            axes[0,j].set_title(joint.title());axes[0,j].set_ylabel("Actual [deg]")
            axes[1,j].set_ylabel("Served reference [deg]");axes[2,j].set_ylabel("Actual - target [deg]")
            axes[2,j].set_xlabel("Simulation time [s]")
        for ax in axes.flat:
            ax.set_xlim(*plot_window);ax.grid(alpha=.25);ax.legend(fontsize=8)
        fig.suptitle("Masking diagnosis: kinematics, common time and recorded imitation target")
        path=PLOTS/filename;fig.savefig(path,dpi=160);plt.close(fig);saved.append(str(path.relative_to(REPO)))
    fig,axes=plt.subplots(3,1,figsize=(14,10),sharex=True,layout="constrained")
    for name in original:
        tables=signals[name]["tables"];torque=tables["sea_torques"];grf=tables["online_grf"]
        axes[0].plot(torque["time"],torque["SEA_Ankle_tau_spring"],color=COLORS[name],label=LABELS[name],lw=.9)
        axes[1].plot(torque["time"],-torque["SEA_Knee_tau_spring"],color=COLORS[name],label=LABELS[name],lw=.8)
        axes[2].plot(grf["time"],grf["left_normal_force"],color=COLORS[name],label=LABELS[name],lw=1)
    for ax,title in zip(axes,("Ankle spring torque [Nm]","Knee spring torque [Nm]","Left normal force [N]")):
        ax.set_ylabel(title);ax.legend();ax.grid(alpha=.25);ax.set_xlim(*WINDOW)
    axes[2].set_xlabel("Simulation time [s]")
    path=PLOTS/"rollout_torque_contact_original_three.png";fig.savefig(path,dpi=160);plt.close(fig);saved.append(str(path.relative_to(REPO)))
    fig,axes=plt.subplots(len(original),2,figsize=(15,10),sharex=True,layout="constrained")
    for row,name in enumerate(original):
        s=signals[name];t=s["time"];q=s["joints"]["knee"];rr=s["reward"]
        axes[row,0].plot(t,q["target"],color="black",label="Knee target")
        axes[row,0].plot(t,q["served"],color=COLORS[name],label="Knee served")
        axes[row,0].set_ylabel(f"{LABELS[name]}\nKnee [deg]")
        imitation=np.array([.2*r["imitation_score"]+.8*r["served_imitation_score"] for r in rr])
        cost=np.array([r["penalty"] for r in rr]);base=np.array([r["reward_base"] for r in rr])
        axes[row,1].plot(t,imitation,label="Weighted imitation score",color="#276984")
        axes[row,1].plot(t,cost,label="Base costs",color="#656565",alpha=.8,ls="--")
        axes[row,1].plot(t,base,label="Reward base after clip",color=COLORS[name])
        axes[row,1].fill_between(t,0,1,where=base<=1e-12,color="#d34836",alpha=.15,label="Clipped to zero")
        axes[row,1].set_ylabel("Per-step score / cost");axes[row,1].set_ylim(-.03,1.03)
    for ax in axes.flat:
        ax.set_xlim(*WINDOW);ax.grid(alpha=.25);ax.legend(fontsize=8,loc="upper right")
    for ax in axes[-1]:ax.set_xlabel("Simulation time [s]")
    fig.suptitle("Knee flexion and clipping of the base reward")
    path=PLOTS/"rollout_reward_floor_original_three.png";fig.savefig(path,dpi=160);plt.close(fig);saved.append(str(path.relative_to(REPO)))
    return saved


def paired_prefixes(metrics,signals):
    """Never compare five-second and early-failure integrals as equal windows."""
    result={}
    for cp in (90,112,184):
        names=[f"cp{cp}_visible",f"cp{cp}_hidden"]
        if not all(n in signals for n in names):
            continue
        end=min(metrics[n]["window_s"][1] for n in names)
        window=(WINDOW[0],end)
        pair={"window_s":list(window),"rollouts":{}}
        for name in dict.fromkeys([*names,"cp90_visible"]):
            s=signals[name];keep=mask(s["time"],window)
            result_joints={}
            for joint,data in s["joints"].items():
                result_joints[joint]={"rmse_actual_target_deg":rms((data["actual"]-data["target"])[keep]),
                                      "rmse_served_target_deg":rms((data["served"]-data["target"])[keep]),
                                      "rmse_actual_served_deg":rms((data["actual"]-data["served"])[keep]),
                                      "actual_excursion_deg":float(np.ptp(data["actual"][keep]))}
            rewards=[r["reward"] for r,k in zip(s["reward"],keep) if k]
            pair["rollouts"][name]={"steps":int(keep.sum()),"joints":result_joints,
                                     "prefix_return":float(sum(rewards)),"prefix_reward_mean":float(np.mean(rewards))}
        result[f"cp{cp}"]=pair
    return result


def window_sensitivity(signals):
    window=(13.15,17.95)
    result={"window_s":list(window),"rollouts":{}}
    for name,s in signals.items():
        if s["time"][-1]<window[1]:
            continue
        keep=mask(s["time"],window)
        torque=s["tables"]["sea_torques"];t=torque["time"];tk=mask(t,window)
        dt=float(np.median(np.diff(t)))
        hp=sosfiltfilt(butter(4,20,btype="highpass",fs=1/dt,output="sos"),torque["SEA_Knee_tau_spring"])
        result["rollouts"][name]={"rmse_actual_target_deg":{j:rms((q["actual"]-q["target"])[keep]) for j,q in s["joints"].items()},
                                   "ankle_positive_area_nm_s":float(np.trapezoid(np.maximum(torque["SEA_Ankle_tau_spring"][tk],0),t[tk])),
                                   "ankle_positive_fraction":float(np.mean(torque["SEA_Ankle_tau_spring"][tk]>0)),
                                   "knee_highpass20_rms_nm":rms(hp[tk])}
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--extra",action="append",default=[],help="label=/absolute/rollout/path")
    args=parser.parse_args()
    folders=dict(SOURCES)
    for extra in args.extra:
        name,path=extra.split("=",1);folders[name]=Path(path).resolve()
    PLOTS.mkdir(parents=True,exist_ok=True)
    metrics,signals={},{}
    for name,folder in folders.items():
        metrics[name],signals[name]=analyze(name,folder)
    base=signals["cp90_visible"]
    for name,s in signals.items():
        n=len(s["time"])
        np.testing.assert_allclose(s["time"],base["time"][:n],rtol=0,atol=1e-8)
        for joint in ("knee","ankle"):
            np.testing.assert_allclose(s["joints"][joint]["target"],base["joints"][joint]["target"][:n],rtol=0,atol=1e-8)
    figures=plot_comparison(metrics,signals)
    limits=["Original three rollouts confound checkpoint adaptation and actor visibility; CP90 was visible, CP112/184 hidden.",
            "One deterministic nominal start and seed per policy/visibility; no multi-start or robustness claim.",
            "Same simulation times and recorded targets verified numerically across every included rollout.",
            "Position/velocity traces are at 100 Hz, physical force and torque at 1000 Hz. No units or sign changes in sources.",
            "Displayed knee coordinates and spring torques have canonical sign reversal; ankle retains raw sign.",
            "Excursion is max-min over the common window, not a phase-warped cycle mean.",
            "Highpass measures are zero-phase 4th-order Butterworth applied to full recorded signal then cropped to common window.",
            "Early failures have a shorter per-rollout window and omitted lag metrics; visibility pairs are also compared on an identical surviving prefix.",
            "All reward components are recomputed with the saved configuration and exact per-step clipping, reproducing saved returns."]
    prefixes=paired_prefixes(metrics,signals)
    sensitivity=window_sensitivity(signals)
    output={"window_s":list(WINDOW),"limits":limits,"figures":figures,"rollouts":metrics,"paired_prefix_comparisons":prefixes,"window_sensitivity":sensitivity}
    (HERE/"metrics.json").write_text(json.dumps(output,indent=2,allow_nan=False)+"\n")
    for path in (Path(__file__), PACKAGE/"reward_function.py", PACKAGE/"tuning_terms.py", REPO/"plot/plotter.py"):
        read(path)
    (HERE/"input_hashes.json").write_text(json.dumps(HASHES,indent=2)+"\n")
    (HERE/"window_sensitivity.json").write_text(json.dumps(sensitivity,indent=2)+"\n")
    lines=["# Offline masking rollout diagnosis", "", f"Common window: {WINDOW[0]:.2f}–{WINDOW[1]:.2f} s.","",
           "| Policy | Window end [s] | RMSE knee/ankle [deg] | Actual/served knee [deg] | Knee excursion [deg] | Ankle positive area [Nm s] | Knee >20Hz [Nm RMS] | Return |",
           "|---|---:|---:|---:|---:|---:|---:|---:|"]
    for name,m in metrics.items():
        k,a=m["joints"]["knee"],m["joints"]["ankle"]
        lines.append(f"| {name} | {m['window_s'][1]:.2f} | {k['rmse_actual_target_deg']:.2f}/{a['rmse_actual_target_deg']:.2f} | {k['rmse_actual_served_deg']:.2f} | {k['actual']['excursion_deg']:.2f} | {m['torque']['ankle']['positive_area_nm_s']:.2f} | {m['torque']['knee']['highpass_rms_nm']['20']:.2f} | {m['episode_return']:.2f} |")
    lines += ["", "Returns in the preceding table cover each entire recorded episode; CP184 visible lasts 0.98 s and its return is not comparable to five-second returns.","",
              "## Frozen-policy visibility controls", "", "Both modes use identical checkpoints, initialization, seed and target. RMSE entries are knee/ankle in degrees.","",
              "| Checkpoint | Visible RMSE | Hidden RMSE | Visible/hidden steps | Interpretation |", "|---|---:|---:|---:|---|"]
    descriptions={90:"Large immediate loss when teacher inputs are removed before any masking updates.",112:"Visible imitation largely retained; hidden imitation still poor.",184:"Hidden recovery remains incomplete; restored visibility crosses knee extension guard at 0.98 s."}
    for cp in (90,112,184):
        visible,hidden=f"cp{cp}_visible",f"cp{cp}_hidden"
        if visible not in metrics or hidden not in metrics:continue
        values=[]
        for name in (visible,hidden):
            m=metrics[name]
            values.append('/'.join(f"{m['joints'][j]['rmse_actual_target_deg']:.2f}" for j in ('knee','ankle')) + (" (short prefix)" if not m['common_window_complete'] else ""))
        lines.append(f"| CP{cp} | {values[0]} | {values[1]} | {metrics[visible]['steps']}/{metrics[hidden]['steps']} | {descriptions[cp]} |")
    if "cp184" in prefixes:
        pair=prefixes["cp184"]
        lines += ["",f"CP184 matched surviving prefix {pair['window_s'][0]:.2f}–{pair['window_s'][1]:.2f} s (78 steps):", "",
                  "| Policy | RMSE knee/ankle [deg] | Mean reward |", "|---|---:|---:|"]
        for name,p in pair['rollouts'].items():
            values='/'.join(f"{p['joints'][j]['rmse_actual_target_deg']:.2f}" for j in ('knee','ankle'))
            lines.append(f"| {name} | {values} | {p['prefix_reward_mean']:.4f} |")
        lines += ["", "The terminal knee position is only +0.0004203 rad beyond the raw zero-extension boundary, with served reference +0.0029552 rad. This is a hard-guard crossing, not evidence of a large numerical explosion."]
    lines += ["", "## Reward and torque interpretation", "",
              "The low-level controller follows the served reference within about 0.2 degrees RMS; the hidden kinematic error is already present in that reference.",
              "For the original CP90-visible / CP112-hidden / CP184-hidden trio: knee excursion 41.71 / 19.04 / 21.36 degrees; ankle excursion 30.67 / 8.98 / 17.62 degrees.",
              "Ankle positive-time fraction falls 57.3% / 50.0% / 43.1%, whereas positive area is 2.88 / 8.41 / 3.42 Nm s. These quantify different aspects of the curve.",
              "Knee spring high-frequency RMS >20 Hz is 3.62 / 3.37 / 3.50 Nm, while motor torque high-frequency RMS is 13.24 / 14.25 / 15.32 Nm. Smoother appearance is not a general reduction of motor chattering.",
              "The command-rate penalty increases 78.17 / 87.99 / 96.47 despite flatter served kinematics; nearly all is the segment-delta component. Raw endpoint mean steps and governor jerk limiting also increase.",
              "At CP184, 67/500 steps have base reward clipped to zero, all within the 112 common-window steps whose target knee exceeds 35 degrees. These target-flexion steps yield 5.69 return versus 52.63 for CP90 visible.",
              "Lag-only alignment leaves CP184 RMS errors of 10.45 degrees knee and 7.19 degrees ankle on a fixed central support; the missing amplitude and offset are not removed by a time shift.","",
              "Sensitivity to the previous 13.15–17.95 s window is saved in `window_sensitivity.json`; no new simulation is required.","",
              "Reproduce with `python analyze.py`; optional completed controls: `--extra label=/absolute/rollout/path`.","",*['- '+x for x in limits],""]
    (HERE/"README.md").write_text("\n".join(lines))
    print("\n".join(lines))


if __name__=="__main__":
    main()
