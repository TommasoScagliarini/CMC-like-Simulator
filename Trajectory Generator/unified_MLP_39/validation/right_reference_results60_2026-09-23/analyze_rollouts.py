"""Offline RIGHT39 checkpoint58/60 analysis, reusing the verified RIGHT40 helpers.

No training or simulation. Canonical and comparative figures live in root plot/.
Run only after both native deterministic rollouts have completed successfully.
"""
from __future__ import annotations

import argparse
import importlib.util
import json
import os
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
for key in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "VECLIB_MAXIMUM_THREADS", "NUMEXPR_NUM_THREADS"):
    os.environ[key] = "1"
os.environ.setdefault("MPLCONFIGDIR", str(HERE / "mpl_cache"))


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


base = load("right60_verified_helpers", HERE.parent / "right_reference_results_2026-09-23/analyze_rollouts.py")
fsm = load("right60_verified_fsm_helpers", HERE.parent / "right_reference_results_2026-09-23/fsm_audit/analyze.py")
np, h, previous = base.np, base.h, base.previous
base.torch.set_num_threads(1)
base.FIGURES = base.REPO / "plot/09_23_2026_right60_comparison"
NEW = ("right_best58", "right_last60")
base.FOLDERS = {
    **{name: base.PACKAGE / f"runs/rollout/MLP39_right_v26_60iter_2026-09-23_{name.removeprefix('right_')}" for name in NEW},
    **{name: base.FOLDERS[name] for name in ("right_best35", "right_last40", "left_best72", "v26_best87")},
}
base.LABELS.update({"right_best58": "39D RIGHT best58", "right_last60": "39D RIGHT last60"})
base.COLORS.update({"right_best58": "#06784e", "right_last60": "#2677c9", "right_best35": "#a4a548", "right_last40": "#d17a36"})
LIMITS = [
    "One training seed and one deterministic start per checkpoint; no statistical-equivalence or stochastic-robustness test.",
    "Best58 and best35 are training-selected checkpoints, not winners of a separate deterministic evaluation selection.",
    "RIGHT/LEFT changes target template, clock, target velocities and target-derived normalization together.",
    "V26 best87 and LEFT best72 have more training experience; they are quality references, not matched-budget controls.",
    "FSM valid-cycle count is accepted non-degraded HS-to-HS cycles; rejected cycles may still contain limb motion.",
    "Conditional learned standard deviations and clipping probabilities use states from deterministic rollouts only.",
    "Lag alignment is descriptive and does not identify causal delay introduced by the reference filter.",
]


def immutable_summary(label, summary):
    """Resolve old mutable checkpoint paths against their archived file hashes."""
    iteration = {"right_best35": 35, "right_last40": 40}.get(label)
    if iteration is None:
        return summary, None
    original = Path(summary["checkpoint"])
    if not original.is_absolute():
        original = base.REPO / original
    archived = h.read(HERE.parent / "right_reference_results_2026-09-23/rollout_input_hashes.json")
    checkpoint = original.parent / f"milestone_iteration_{iteration:06d}/rl_module_last"
    checked = {}
    for filename in ("module_state.pkl", "class_and_ctor_args.pkl", "unified_contract.json", "training_cfg.resolved.yaml"):
        expected = archived[str((original / filename).relative_to(base.REPO))]
        actual = previous.file_hash(checkpoint / filename)
        assert actual == expected, (label, filename, actual, expected)
        checked[filename] = actual
    return {**summary, "checkpoint": str(checkpoint)}, {
        "original_summary_checkpoint": str(original), "resolved_checkpoint": str(checkpoint),
        "archived_source_hashes_match": checked, "original_summary_unchanged": True,
    }


def reward_decomposition(label, trace, summary, metric):
    """Verify the exact per-step base clipping before summing its correction."""
    module = h.historical_reward if label == "v26_best87" else h.reward_function
    cfg = module.RewardConfig.from_mapping(summary["reward_config"])
    assert cfg.reward_mode == "imitation" and cfg.blend_imitation_tracking == 0
    actual, served, clip_adjustment, clipped_steps = 0.0, 0.0, 0.0, 0
    for row in trace:
        _, components = module.compute_reward(row["reward_terms"], cfg, reference=row["policy_segment_values"])
        a = cfg.blend_imitation * components["imitation_score"]
        s = cfg.blend_served_imitation * components["served_imitation_score"]
        before_clip = a + s - components["penalty"]
        after_clip = min(cfg.clip_high, max(cfg.clip_low, before_clip))
        np.testing.assert_allclose(after_clip, components["reward_base"], atol=1e-12, rtol=0)
        actual += a
        served += s
        clip_adjustment += after_clip - before_clip
        clipped_steps += int(after_clip != before_clip)
    total = metric["reward_components_sums"]
    np.testing.assert_allclose(actual + served - total["penalty"] + clip_adjustment,
                               total["reward_base"], atol=1e-8, rtol=0)
    return {"weighted_actual_imitation": actual, "weighted_served_imitation": served,
            "mixed_imitation_before_penalties": actual + served, "base_penalties": total["penalty"],
            "per_step_base_clip_adjustment": clip_adjustment, "base_clipped_steps": clipped_steps,
            "base_clip_bounds": [cfg.clip_low, cfg.clip_high], "per_step_base_clipping_verified": True,
            "additional_penalties": total["reward_base"] - total["reward"],
            "full_return_reconstruction_error": metric["reward_reconstruction_error"]}


def ankle_moment_pairing(folder, trace, raw):
    """Pair the 1 kHz SEA spring with recorded 100 Hz joint-projected GRF moment."""
    time = np.asarray([row["time"] for row in trace])
    sea = np.interp(time, raw["time"], raw["torques"]["ankle"])
    moment = np.asarray([row["reward_terms"]["grf_ankle_moment_flip_tau_nm"] for row in trace])
    force = np.asarray([row["reward_terms"]["grf_ankle_moment_flip_force_n"] for row in trace])
    diag = previous.read_table(folder, "sea_diagnostics")
    peak = int(np.argmax(raw["torques"]["ankle"]))
    closest = int(np.argmin(abs(time - raw["time"][peak])))
    final = np.flatnonzero(time - raw["time"][0] >= 4 - 1e-8)

    def snapshot(index):
        return {"time_s": float(time[index]), "elapsed_s": float(time[index] - raw["time"][0]),
                "sea_spring_nm": float(sea[index]), "grf_joint_moment_nm": float(moment[index]),
                "normal_force_n": float(force[index]),
                "sea_tau_ref_nm": float(np.interp(time[index], diag.time, diag.data[:, diag.index["SEA_Ankle_tau_ref"]])),
                "ankle_target_deg": float(np.degrees(trace[index]["imitation_target_q"]["pros_ankle_angle"])),
                "ankle_actual_deg": float(np.degrees(trace[index]["prosthetic_state"]["pros_ankle_angle"])),
                "fsm_state": trace[index]["phase_fsm"]["state_name"]}

    return {"nearest_reward_sample_to_sea_peak": snapshot(closest),
            "peak_vs_reward_sample_time_difference_s": float(time[closest] - raw["time"][peak]),
            "last_second_most_negative_grf_sample": snapshot(int(final[np.argmin(moment[final])])),
            "last_second_grf_moment_min_max_nm": [float(moment[final].min()), float(moment[final].max())],
            "scope": "GRF moment is the recorded moment about the ankle projected onto its axis; online_grf.sto moment_xyz is about the ground origin and is not directly interchangeable. SEA is linearly interpolated from 1 kHz STO to reward times. Opposite signs are an association, not isolated causality."}


def summary_markdown(result):
    lines = ["# RIGHT39: rollout deterministici alle iterazioni 58 e 60", "",
             "| Rollout | Passi | Return | RMSE knee / ankle [°] | Pen. max [mm] | Cicli validi | Invalid / resync |",
             "|---|---:|---:|---:|---:|---:|---:|"]
    for name, metric in result["rollouts"].items():
        knee, ankle = [metric["joints"][j]["rmse_actual_target_deg"] for j in base.JOINTS]
        final = metric["fsm_final"]
        lines.append(f"| {base.LABELS[name]} | {metric['steps']} | {metric['episode_return']:.3f} | {knee:.3f} / {ankle:.3f} | {metric['contact']['penetration_peak_mm']:.3f} | {final['valid_cycle_count']} | {final['invalid_event_count']} / {final['resync_count']} |")
    lines += ["", "## Coppie e distribuzione delle azioni", "",
              "| Rollout | Coppia ankle max [Nm] | Chattering knee >20Hz, dopo 0,2s [Nm RMS] | Std knee / ankle |",
              "|---|---:|---:|---:|"]
    for name, metric in result["rollouts"].items():
        std = metric["policy_audit"]["learned_std_mean"]
        lines.append(f"| {base.LABELS[name]} | {metric['sea_torque']['ankle']['whole_episode']['max_nm']:.3f} | {metric['sea_torque']['knee']['after_200ms']['highpass_above_20hz_rms_nm']:.3f} | {std[0]:.3f} / {std[1]:.3f} |")
    lines += ["", "## Contributi alla reward cumulativa", "",
              "| Rollout | Imitazione actual pesata | Imitazione served pesata | Penalità command rate | Penalità GRF ankle |",
              "|---|---:|---:|---:|---:|"]
    for name, metric in result["rollouts"].items():
        terms = metric["reward_decomposition"]
        lines.append(f"| {base.LABELS[name]} | {terms['weighted_actual_imitation']:.3f} | {terms['weighted_served_imitation']:.3f} | {metric['penalty_components_sums']['command_rate_weight']:.3f} | {metric['reward_components_sums']['grf_ankle_moment_flip_term']:.3f} |")
    lines += ["", "Picchi di penetrazione dalla registrazione fisica STO; le metriche reward usano campioni a 100 Hz. Segni delle coppie coerenti con i plot canonici.",
              "", "## Limiti", "", *["- " + line for line in LIMITS], ""]
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--render", action="store_true", help="Render seven canonical figures for each new rollout")
    args = parser.parse_args()
    base.FIGURES.mkdir(parents=True, exist_ok=True)
    result = {"rollouts": {}, "limits": LIMITS, "method": "Verified RIGHT40 metric helpers reused without modification; CPU numerical libraries limited to one thread."}
    traces, physical, summaries, fsm_results = {}, {}, {}, {}
    for label, folder in base.FOLDERS.items():
        metric, trace, summary = h.rollout_metrics(folder, historical=label == "v26_best87", scale=1 if label.startswith("left_") else None)
        summary, checkpoint_resolution = immutable_summary(label, summary)
        if checkpoint_resolution:
            metric["policy_checkpoint_resolution"] = checkpoint_resolution
        extra, raw = previous.physical_metrics(folder, trace)
        metric.update(extra)
        for short, torque in raw["torques"].items():
            maximum = int(np.argmax(torque))
            metric["sea_torque"][short]["maximum_timing"] = {
                "time_s": float(raw["time"][maximum]), "elapsed_s": float(raw["elapsed"][maximum]),
                "value_nm": float(torque[maximum]),
            }
        grf = previous.read_table(folder, "online_grf")
        after = grf.time - grf.time[0] >= .2 - 1e-8
        metric["contact"]["after_200ms"] = {
            "normal_force_peak_n": float(grf.data[after, grf.index["left_normal_force"]].max()),
            "penetration_peak_mm": float(1000 * grf.data[after, grf.index["left_penetration"]].max()),
            "contact_fraction": float(np.mean(grf.data[after, grf.index["left_in_contact"]] > .5)),
        }
        metric["policy_audit"] = base.policy_audit(trace, summary)
        metric["lag_audit"] = base.lag_audit(trace)
        metric["reward_decomposition"] = reward_decomposition(label, trace, summary, metric)
        metric["ankle_moment_pairing"] = ankle_moment_pairing(folder, trace, raw)
        fsm_results[label] = fsm.audit(folder)
        result["rollouts"][label], traces[label], physical[label], summaries[label] = metric, trace, raw, summary
        print(label, "steps", metric["steps"], "return", metric["episode_return"], flush=True)
    for label in (name for name in base.FOLDERS if name.startswith("right_")):
        result["rollouts"][label]["right_input_audit"] = base.right_input_audit(traces[label], traces["v26_best87"], summaries[label])
        if args.render and label in NEW:
            result["rollouts"][label]["canonical_plots"] = base.canonical(label, summaries[label])
    result["comparison_figures"] = [
        base.comparison(traces, physical, (*NEW, "v26_best87"), "right60_vs_v26_deterministic.png"),
        base.comparison(traces, physical, (*NEW, "right_best35", "right_last40"), "right60_vs_right40_deterministic.png"),
        base.comparison(traces, physical, ("right_best58", "left_best72", "v26_best87"), "right60_vs_left72_v26_deterministic.png"),
    ]
    # Read-only provenance includes helper scripts, all metric sources and FSM inputs.
    for path in (Path(__file__), Path(base.__file__), Path(fsm.__file__), Path(previous.__file__), Path(h.__file__), Path(base.lag_helper.__file__)):
        h.INPUT_HASHES[str(path.relative_to(base.REPO))] = previous.file_hash(path)
    h.INPUT_HASHES.update(fsm.HASHES)
    h.dump(HERE / "rollout_metrics.json", result)
    h.dump(HERE / "fsm_metrics.json", {"rollouts": fsm_results, "limits": LIMITS})
    h.dump(HERE / "analysis_input_hashes.json", h.INPUT_HASHES)
    (HERE / "summary.md").write_text(summary_markdown(result), encoding="utf-8")
    print(json.dumps({"ok": True, "rollouts": len(result["rollouts"]), "canonical_sets": 2 if args.render else 0,
                      "comparative_figures": len(result["comparison_figures"])}, indent=2))


if __name__ == "__main__":
    main()
