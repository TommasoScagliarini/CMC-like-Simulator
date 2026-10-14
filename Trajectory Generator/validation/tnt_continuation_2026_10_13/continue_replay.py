"""Exploratory continuation of previously inspected TNT data; no neural training."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
import sys

import numpy as np

HERE = Path(__file__).resolve().parent
PREVIOUS = HERE.parent / "tnt_preliminary_2026_10_13"
ROOT = HERE.parents[2]
sys.path.insert(0, str(PREVIOUS))
from tnt_replay import Parameters, make_template, predict, score, duration_stats, write_json, rate_diagnostics
from kinematic_generator import stream_replay, KinematicTNTController


def metrics(r, q, pred, mask):
    result = score(r, q, pred, mask)
    rates = rate_diagnostics(r)
    ratios = [x["early_to_middle_duration_ratio"] for x in rates if "early_to_middle_duration_ratio" in x]
    result["rate_stats"] = {
        "minimum": float(r["rate"].min()), "maximum": float(r["rate"].max()),
        "max_early_middle_ratio": max(ratios, default=None),
        "max_abs_blended_local_gain_alpha025": max((abs(1 - .25*(1+x)) for x in ratios), default=None)}
    return result


def plot_subject(subject, t, q, cases, cutoff):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(4, 1, figsize=(13, 10), constrained_layout=True)
    for name, (r, pred) in cases.items():
        axes[0].plot(t, np.where(r["state"] < 0, np.nan, r["rate"]), label=name, lw=1)
        invalid = np.flatnonzero(r["state"] < 0)
        if len(invalid):
            axes[0].axvline(t[invalid[0]], color="C0", ls=":")
            axes[0].annotate("Original estimator INVALID", (t[invalid[0]], r["rate"][invalid[0]]),
                             xytext=(5, -8), textcoords="offset points", fontsize=8)
    axes[0].set_ylabel("Normalized rate [1/s]")
    axes[0].axvline(cutoff, color="grey", ls=":", label="End of curve calibration")
    axes[0].legend(ncol=3, fontsize=8)
    m = (t >= t[0] + 60) & (t < t[0] + 65)
    r, pred = cases["TNT blended rate"]
    axes[1].plot(t[m], r["T"][m], label="TNT swing progress")
    axes[1].plot(t[m], np.where(r["state"][m] == 0, r["stance_progress"][m], np.nan), label="Stance timer extension")
    axes[1].set_ylabel("Progress")
    axes[1].legend(fontsize=8)
    for j, name in enumerate(["Knee", "Ankle"]):
        ax = axes[j+2]
        ax.plot(t[m], q[m, j], color="black", label="Prescribed target")
        for label in ["TNT blended rate", "Fixed duration"]:
            ax.plot(t[m], cases[label][1][m, j], label=label)
        ax.set_ylabel(name + " [deg]")
        ax.legend(ncol=3, fontsize=8)
    for ax in axes:
        ax.grid(alpha=.25)
        ax.set_xlabel("Recorded time [s]")
    fig.suptitle(subject + ": exploratory TNT continuation, no network or GRF input")
    out = ROOT / "plot/tnt_continuation_2026_10_13"
    out.mkdir(parents=True, exist_ok=True)
    fig.savefig(out / (subject + "_replay_comparison.png"), dpi=150)
    plt.close(fig)


def subject_run(old):
    subject = old["subject"]
    data = np.genfromtxt(PREVIOUS / (subject.lower() + "_prescribed.csv"), delimiter=",", names=True)
    t, theta = data["time_s"], data["thigh_deg"]
    q = np.c_[data["knee_deg"], data["ankle_deg"]]
    params = Parameters(**old["parameters"])
    stance_duration = old["calibration_geometry"]["initial_stance_duration_s"]
    cal, evaluate = t < t[0]+40, t >= t[0]+40
    results, cases = {}, {}
    templates = {}
    for method, label in [("literal", "TNT original rate"), ("blend", "TNT blended rate"), ("duration", "TNT duration rate")]:
        r = stream_replay(t, theta, params, method=method, initial_stance_duration=stance_duration)
        template = make_template(r, q, cal)
        pred = predict(r, template)
        results[method] = metrics(r, q, pred, evaluate)
        templates[method] = template
        cases[label] = r, pred
    duration = float(np.median(duration_stats(cases["TNT blended rate"][0], cal)["swing"]))
    pfd = stream_replay(t, theta, params, pfd_duration=duration, initial_stance_duration=stance_duration)
    pfd_template = make_template(pfd, q, cal)
    pfd_pred = predict(pfd, pfd_template)
    results["pfd"] = metrics(pfd, q, pfd_pred, evaluate)
    cases["Fixed duration"] = pfd, pfd_pred

    receipt = json.loads((PREVIOUS / (subject.lower() + "_extraction_receipt.json")).read_text())
    bundle = {
        "subject": subject, "scope": "Exploratory kinematic controller; paper TNT with explicit blended-rate modification",
        "parameters": old["parameters"], "rate_method": "blend", "rate_alpha": .25,
        "filter_cutoff_hz": 6., "initial_stance_duration_s": stance_duration,
        "angle_definition": receipt["angle_definition"],
        "template": templates["blend"], "joint_names": ["pros_knee_angle", "pros_ankle_angle"],
        "joint_units": "degree", "inputs": receipt["inputs"],
        "calibration_window_s": [float(t[0]), float(t[0]+40)],
        "limitations": "Previously inspected same-trial data; no unseen-subject/condition validation; no GRF or q targets at runtime",
    }
    bundle_path = HERE / (subject.lower() + "_bundle.json")
    write_json(bundle_path, bundle)

    # Verify exact streaming/controller reference parity before a dynamic consumer.
    controller = KinematicTNTController(bundle_path, t[0], theta[0])
    generated = np.array([controller.step(now, angle) for now, angle in zip(t, theta)])
    np.testing.assert_allclose(generated, cases["TNT blended rate"][1], atol=1e-11, rtol=0)

    # Predeclared perturbations on inspected traces, hence stress diagnostics only.
    stresses = {}
    signals = {"clean": theta}
    for seed in [11, 23, 47]:
        signals[f"noise025deg_seed{seed}"] = theta + np.random.default_rng(seed).normal(0, .25, len(theta))
    signals["bias_minus2deg"] = theta - 2
    signals["bias_plus2deg"] = theta + 2
    for name, signal in signals.items():
        r = stream_replay(t, signal, params, method="blend", initial_stance_duration=stance_duration)
        stresses[name] = metrics(r, q, predict(r, templates["blend"]), evaluate)
    for scale in [.75, 1.25]:
        physical_time = np.arange(t[0], t[0] + (t[-1] - t[0])/scale, .01)
        source_time = t[0] + (physical_time-t[0])*scale
        signal = np.interp(source_time, t, theta)
        target = np.column_stack([np.interp(source_time, t, q[:, j]) for j in range(2)])
        r = stream_replay(physical_time, signal, params, method="blend", initial_stance_duration=stance_duration)
        stresses[f"synthetic_speed_factor_{scale}"] = metrics(r, target, predict(r, templates["blend"]), source_time >= t[0]+40)
    starts = []
    for elapsed in [0, 10, 30, 50, 70, 90, 110]:
        start = int(round(elapsed/.01))
        end = min(start + 2000, len(t))
        r = stream_replay(t[start:end], theta[start:end], params, method="blend", initial_stance_duration=stance_duration)
        warm = r["t"] >= r["t"][0]+2.5
        m = metrics(r, q[start:end], predict(r, templates["blend"]), warm)
        starts.append({"elapsed_start_s": elapsed, "warmup_excluded_s": 2.5, "metrics": m})
    r, pred = cases["TNT blended rate"]
    out = {
        "subject": subject, "parameters_unchanged_from_prior": old["parameters"],
        "sample_count": len(t), "comparison": results, "stress_tests": stresses,
        "cold_starts": starts, "streaming_reference_max_error_deg": float(np.max(abs(generated-pred))),
        "bundle": str(bundle_path.relative_to(ROOT)), "pfd_duration_s": duration,
        "full_recording_completed_swings": int(r["completed"][-1]),
        "recommended_dynamic_start_s": next(e["t"] for e in r["events"] if e["event"] == "late_to_stance" and e["swing_id"] >= 3),
        "intercycle_template_spread_deg": templates["blend"]["intercycle_rms_std_deg"],
    }
    names = "time_s,thigh_deg,T,state,rate,raw_rate,stance_progress,knee_target_deg,ankle_target_deg,knee_tnt_deg,ankle_tnt_deg"
    np.savetxt(HERE / (subject.lower()+"_blend_trace.csv"), np.column_stack([t, theta,r["T"],r["state"],r["rate"],r["raw_rate"],r["stance_progress"],q,pred]), delimiter=",", header=names, comments="")
    write_json(HERE / (subject.lower()+"_blend_events.json"), r["events"])
    write_json(HERE / (subject.lower()+"_blend_rates.json"), rate_diagnostics(r))
    plot_subject(subject, t, q, cases, t[0]+40)
    print(subject, "complete",out["full_recording_completed_swings"],"testRMSE",results["blend"]["all"],flush=True)
    return out


def main():
    prior = json.loads((PREVIOUS/"results.json").read_text())
    result = {
        "scope": "Exploratory continuation on previously inspected recordings, not a new blinded holdout",
        "parameter_policy": "Keep prior selected parameters fixed; alpha=.25 is a declared modification, not tuned against target error",
        "curve_fit": "Same first40s only, rebuilt on each generator's own phase axis",
        "new_rate_laws": {"blend": "v_new=(1-alpha)*v_previous+alpha*v_raw, alpha=.25",
                          "duration": "v_new=T_em/(D_early+D_middle)"},
        "blend_local_gain": "For fixed durations: 1-alpha*(1+D_early/D_middle). Does not guarantee global hybrid/noise stability.",
        "results": [subject_run(s) for s in prior["results"]]}
    write_json(HERE/"continuation_results.json", result)


if __name__ == "__main__":
    main()
