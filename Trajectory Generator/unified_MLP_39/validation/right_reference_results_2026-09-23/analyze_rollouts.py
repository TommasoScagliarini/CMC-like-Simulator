"""Read-only checkpoint evaluation analysis; no training or simulation.

Reuses the historical reward/physical metrics, keeping signed SEA torques and
the 20 Hz chattering proxy comparable. Canonical figures go to root plot/.
"""
from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
PACKAGE = HERE.parents[1]
REPO = PACKAGE.parent.parent
for name in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "VECLIB_MAXIMUM_THREADS"):
    os.environ.setdefault(name, "1")
os.environ.setdefault("MPLCONFIGDIR", str(HERE / "mpl_cache"))


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


previous = load("right_results_metric_helpers", HERE.parent / "v26_weight_transfer_2026-09-23/analyze.py")
lag_helper = load("right_results_lag_helpers", HERE.parent / "phase_delay_torque_2026-09-22/lag_audit/analyze.py")
h = previous.helpers
np, plt, torch, plotter = previous.np, previous.plt, h.torch, previous.plotter
torch.set_num_threads(1)
JOINTS = h.JOINTS
FIGURES = REPO / "plot/09_23_2026_right_reference_comparison"
FOLDERS = {
    "right_best35": PACKAGE / "runs/rollout/MLP39_right_v26_40iter_2026-09-23_best35",
    "right_last40": PACKAGE / "runs/rollout/MLP39_right_v26_40iter_2026-09-23_last40",
    "left_last40": PACKAGE / "runs/rollout/MLP39_v26_aligned_left_phase_fsmv3_40iter_2026-09-21_last40",
    "left_best72": previous.BEST72,
    "v26_best87": previous.OLD,
}
LABELS = {"right_best35": "39D RIGHT best35", "right_last40": "39D RIGHT last40",
          "left_last40": "39D LEFT last40", "left_best72": "39D LEFT best72", "v26_best87": "V26 agosto best87"}
COLORS = {"right_best35": "#167c5b", "right_last40": "#418fd1", "left_last40": "#dd8653",
          "left_best72": "#b63862", "v26_best87": "#393939"}


def policy_audit(trace, summary):
    checkpoint = Path(summary["checkpoint"])
    if not checkpoint.is_absolute():
        checkpoint = REPO / checkpoint
    for path in checkpoint.iterdir():
        if path.is_file():
            h.INPUT_HASHES[str(path.relative_to(REPO))] = previous.file_hash(path)
    module = h.RLModule.from_checkpoint(str(checkpoint))
    module.eval()
    names = summary["actor_feature_names"][:module._n_actor]
    obs = np.asarray([r["actor_observation_vector_before"][:module._n_actor] for r in trace], dtype=np.float32)

    def forward(x):
        with torch.no_grad():
            return module.forward_inference({"obs": torch.as_tensor(x)})["action_dist_inputs"].numpy()

    logits = forward(obs)
    mean, std = logits[:, :2], np.exp(logits[:, 2:])
    error = float(np.max(abs(mean - np.asarray([r["raw_policy_action"] for r in trace]))))
    assert error < 2e-6, error
    prescribed = [i for i, name in enumerate(names) if name.startswith(("prescribed_left_", "prescribed_right_")) or "imitation_target" in name]
    clock = [names.index("gait_phase_sin"), names.index("gait_phase_cos")]
    assert len(prescribed) == 4
    mutations = {}
    for label, indices in (("four_targets_zero", prescribed), ("clock_zero", clock), ("all_six_zero", prescribed + clock)):
        changed = obs.copy()
        changed[:, indices] = 0
        mutations[label] = np.sqrt(np.mean((forward(changed)[:, :2] - mean) ** 2, axis=0)).tolist()
    return {
        "actor_width": module._n_actor, "full_width": module._n_full,
        "reproduced_action_max_abs_error": error,
        "learned_std_mean": std.mean(axis=0).tolist(),
        "learned_std_min": std.min(axis=0).tolist(), "learned_std_max": std.max(axis=0).tolist(),
        "gaussian_clip_probability_mean": np.mean(h.ndtr((-1 - mean) / std) + 1 - h.ndtr((1 - mean) / std), axis=0).tolist(),
        "deterministic_clip_fraction": np.mean(abs(mean) > 1, axis=0).tolist(),
        "input_zeroing_action_delta_rms": mutations,
        "scope": "Conditional on each deterministic trajectory; not a stochastic robustness experiment or causal isolation of exploration.",
    }


def right_input_audit(trace, historical, summary):
    n = min(len(trace), len(historical))
    np.testing.assert_allclose([r["time"] for r in trace[:n]], [r["time"] for r in historical[:n]], atol=2e-8, rtol=0)
    max_error = 0.0
    for row, old in zip(trace[:n], historical[:n]):
        assert row["target_availability"]["visible"]
        for field in ("imitation_target_q", "imitation_target_qdot"):
            for joint in JOINTS:
                max_error = max(max_error, abs(row[field][joint] - old[field][joint]))
                np.testing.assert_allclose(row[field][joint], old[field][joint], atol=1e-10, rtol=0)
        obs = row["actor_observation_before"]
        for key in ("gait_phase_sin", "gait_phase_cos"):
            np.testing.assert_allclose(obs[key], old["actor_observation_before"][key], atol=1e-7, rtol=0)
    for i in range(1, len(trace)):
        obs = trace[i]["actor_observation_before"]
        for joint, short in zip(JOINTS, ("knee", "ankle")):
            np.testing.assert_allclose(obs[f"prescribed_right_shifted_{short}_q_scaled"], trace[i - 1]["imitation_target_q"][joint], atol=2e-6, rtol=0)
            np.testing.assert_allclose(obs[f"prescribed_right_shifted_{short}_qdot_scaled"], trace[i - 1]["imitation_target_qdot"][joint], atol=2e-6, rtol=0)
    assert summary["episode_start_offset_s"] == 1
    return {"pass": True, "samples": n, "max_target_q_qdot_error": max_error,
            "clock_matches_historical_unshifted_right": True, "six_inputs_visible": True,
            "velocity_divisor": 1, "target_shifts": {"knee": .465, "ankle": .452}}


def lag_audit(trace):
    t = np.asarray([r["time"] for r in trace])
    indices = np.flatnonzero((t >= 13.30 - 1e-8) & (t <= 17.75 + 1e-8))
    if not len(indices) or indices.min() < 20 or indices.max() + 20 >= len(t):
        return {"available": False, "reason": "Incomplete common window"}
    result = {"available": True, "method": "Same fixed window 13.30–17.75 s; lags -200 to +200 ms in 10 ms steps. Descriptive alignment, not a causal filter-delay estimate.", "joints": {}}
    for j, joint in enumerate(JOINTS):
        data = {"target": h.array(trace, "imitation_target_q", joint),
                "served": h.array(trace, "prosthetic_state", joint + "_served_ref"),
                "actual": h.array(trace, "prosthetic_state", joint),
                "endpoint": np.asarray([r["policy_segment_values"][-1][j] for r in trace])}
        result["joints"][joint] = {a + "_to_" + b: lag_helper.lag_stats(data[a], data[b], indices, t)
            for a, b in (("target", "served"), ("served", "actual"), ("target", "endpoint"), ("endpoint", "served"))}
    return result


def canonical(label, summary):
    folder = FOLDERS[label]
    out = REPO / "plot" / ("09_23_2026_39D_" + label)
    out.mkdir(parents=True, exist_ok=True)
    receipt_path = out / "plot_receipt.json"
    trace_sha = previous.file_hash(folder / "rollout_policy_trace.json")
    if receipt_path.exists():
        receipt = h.read(receipt_path)
        assert receipt["trace_sha256"] == trace_sha
        assert len(receipt["figures"]) == 7 and all(Path(p).is_file() for p in receipt["figures"])
        return receipt
    context = plotter.valid_mlp_rollout_context(folder)
    assert context is not None
    from matplotlib.text import Text
    saved = []
    original_save, original_discovery = plotter.save_figure, plotter.latest_mlp_rollout_context
    original_next, argv = plotter.next_output_dir, sys.argv

    def save(fig, out_dir, filename):
        if filename.startswith("07_"):
            for label_object in fig.findobj(match=Text):
                txt = label_object.get_text().replace("Shifted Sound Leg Target", "Prescribed RIGHT Phase Target")
                txt = txt.replace("shifted sound leg target", "prescribed RIGHT phase target")
                txt = txt.replace("shifted sound leg - policy C2", "RIGHT phase target - policy C2")
                label_object.set_text(txt)
        original_save(fig, out_dir, filename)
        saved.append(str(out_dir / filename))

    try:
        plotter.next_output_dir = lambda _: out
        plotter.latest_mlp_rollout_context = lambda: context
        plotter.save_figure = save
        sys.argv = [str(REPO / "plot/plotter.py"), "--mlp", "--results-dir", str(context.sim_outputs_dir),
                    "--prefix", "rollout_episode", "--setup", str(summary["setup_xml_path"]),
                    "--gait-side", "left", "--out-root", str(REPO / "plot")]
        assert plotter.main() in (0, None)
    finally:
        plotter.save_figure, plotter.latest_mlp_rollout_context = original_save, original_discovery
        plotter.next_output_dir, sys.argv = original_next, argv
    assert len(saved) == 7 and all(Path(p).is_file() for p in saved)
    receipt = {"rollout": str(folder), "checkpoint": summary["checkpoint"], "trace_sha256": trace_sha,
               "figures": saved, "target": "Recorded prescribed RIGHT phase target, shifts knee .465 / ankle .452",
               "gait_side": "left", "middle_row_figure7": "Raw sound-side kinematics, context only"}
    h.dump(receipt_path, receipt)
    return receipt


def comparison(traces, physical, labels, filename):
    fig, axes = plt.subplots(3, 2, figsize=(15, 11), layout="constrained")
    for label in labels:
        trace = traces[label]
        elapsed = np.asarray([r["time"] for r in trace]) - trace[0]["policy_segment_times"][0]
        for col, (joint, short) in enumerate(zip(JOINTS, ("knee", "ankle"))):
            axes[0, col].plot(elapsed, plotter.joint_sign(short) * np.degrees(h.array(trace, "prosthetic_state", joint)),
                              label=LABELS[label], color=COLORS[label], lw=1.15)
            axes[1, col].plot(physical[label]["elapsed"], physical[label]["torques"][short],
                              label=LABELS[label], color=COLORS[label], lw=.7)
        axes[2, 0].plot(elapsed, [1000 * r["reward_terms"].get("grf_penetration_m", 0) for r in trace], label=LABELS[label], color=COLORS[label])
        axes[2, 1].plot(elapsed, [r["reward_terms"].get("grf_ankle_moment_flip_tau_nm", 0) for r in trace], label=LABELS[label], color=COLORS[label])
    for source, style, name in (("v26_best87", "--", "Target RIGHT"), ("left_best72", ":", "Target LEFT")):
        if source == "left_best72" and not any(s.startswith("left_") for s in labels):
            continue
        trace = traces[source]
        elapsed = np.asarray([r["time"] for r in trace]) - trace[0]["policy_segment_times"][0]
        for col, (joint, short) in enumerate(zip(JOINTS, ("knee", "ankle"))):
            axes[0, col].plot(elapsed, plotter.joint_sign(short) * np.degrees(h.array(trace, "imitation_target_q", joint)), style, color="black", lw=1, label=name)
    for ax, title, unit in zip(axes.flat, ("Ginocchio", "Caviglia", "Coppia SEA ginocchio", "Coppia SEA caviglia", "Penetrazione (campioni reward)", "Momento GRF caviglia"), ("°", "°", "Nm", "Nm", "mm", "Nm")):
        ax.set(title=title, ylabel=unit, xlabel="Tempo dal reset [s]")
        ax.grid(alpha=.2)
        ax.legend(fontsize=8)
    fig.suptitle("Rollout deterministici, stesso start; V26 best87 e LEFT best72 hanno più esperienza")
    path = FIGURES / filename
    fig.savefig(path, dpi=165)
    plt.close(fig)
    return str(path)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--render", action="store_true")
    args = p.parse_args()
    FIGURES.mkdir(parents=True, exist_ok=True)
    result = {"rollouts": {}, "limits": ["One training seed, one deterministic start; not statistical equivalence.",
        "RIGHT/LEFT intervention includes template, phase clock, target velocities and target-derived normalization.",
        "V26 best87 and LEFT best72 are later checkpoints; no matched-budget historical rollout checkpoint available.",
        "Training completion describes exploratory collection across changing policies, not a frozen-checkpoint robustness test."]}
    traces, physical, summaries = {}, {}, {}
    for label, folder in FOLDERS.items():
        metric, trace, summary = h.rollout_metrics(folder, historical=label == "v26_best87", scale=1 if label.startswith("left_") else None)
        extra, raw = previous.physical_metrics(folder, trace)
        metric.update(extra)
        grf = previous.read_table(folder, "online_grf")
        after = grf.time - grf.time[0] >= .2 - 1e-8
        metric["contact"]["after_200ms"] = {
            "normal_force_peak_n": float(grf.data[after, grf.index["left_normal_force"]].max()),
            "penetration_peak_mm": float(1000 * grf.data[after, grf.index["left_penetration"]].max()),
            "contact_fraction": float(np.mean(grf.data[after, grf.index["left_in_contact"]] > .5)),
        }
        metric["policy_audit"] = policy_audit(trace, summary)
        metric["lag_audit"] = lag_audit(trace)
        result["rollouts"][label], traces[label], physical[label], summaries[label] = metric, trace, raw, summary
        print(label, "steps", metric["steps"], "return", metric["episode_return"], flush=True)
    for label in ("right_best35", "right_last40"):
        result["rollouts"][label]["right_input_audit"] = right_input_audit(traces[label], traces["v26_best87"], summaries[label])
        if args.render:
            result["rollouts"][label]["canonical_plots"] = canonical(label, summaries[label])
    result["comparison_figures"] = [
        comparison(traces, physical, ("right_best35", "right_last40", "v26_best87"), "right_vs_v26_deterministic.png"),
        comparison(traces, physical, ("right_best35", "left_last40", "left_best72"), "right_vs_left_deterministic.png")]
    h.dump(HERE / "rollout_metrics.json", result)
    h.dump(HERE / "rollout_input_hashes.json", h.INPUT_HASHES)
    print(json.dumps({name: {"steps": m["steps"], "return": m["episode_return"],
        "rmse_deg": [m["joints"][j]["rmse_actual_target_deg"] for j in JOINTS],
        "ankle_positive_peak_nm": m["sea_torque"]["ankle"]["whole_episode"]["max_nm"],
        "knee_highpass_rms_nm": m["sea_torque"]["knee"]["after_200ms"]["highpass_above_20hz_rms_nm"],
        "std": m["policy_audit"]["learned_std_mean"]} for name, m in result["rollouts"].items()}, indent=2))


if __name__ == "__main__":
    main()
