"""Offline comparison of V26 weights in unified39; never runs a simulator.

Use --render to add the seven canonical plots under the repository root plot/.
The RIGHT diagnostic is compared with the saved August rollout and LEFT best72.
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
TG = PACKAGE.parent
REPO = TG.parent
for _key in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ.setdefault(_key, "1")
os.environ.setdefault("MPLCONFIGDIR", str(HERE / "mpl_cache"))


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


helpers = load_module("v26_transfer_metric_helpers", PACKAGE / "validation/ablation_report_2026-09-20/analyze_results.py")
import numpy as np
from scipy.signal import butter, sosfiltfilt
import matplotlib.pyplot as plt

helpers.torch.set_num_threads(1)
plotter = load_module("v26_transfer_canonical_plotter", REPO / "plot/plotter.py")
DEFAULT_NEW = PACKAGE / "runs/rollout/V26_best87_in_unified39_right_fsmv3_2026-09-23"
OLD = TG / "runs/rollout/MLP_imitation_native_v26_08-20-2026_june_equiv_100iter_rollout"
BEST72 = PACKAGE / "runs/rollout/MLP39_completion_2026-09-22_best072/deterministic"
ACTOR_PERMUTATION = [0, 1, *range(6, 39), 2, 3, 4, 5]
LABELS = {"v26_historical": "V26 agosto best87", "v26_in_unified39": "V26 in unified39 RIGHT", "unified39_best72_left": "Unified39 best72 LEFT"}


def file_hash(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def tensor_error(a, b):
    delta = np.asarray(a, float) - np.asarray(b, float)
    return {"max_abs": float(np.max(np.abs(delta))), "rms": float(np.sqrt(np.mean(delta * delta)))}


def read_table(folder, suffix):
    path = folder / "sim_outputs" / f"rollout_episode_{suffix}.sto"
    table = plotter.load_table(path)
    if table is None:
        raise FileNotFoundError(path)
    helpers.INPUT_HASHES[str(path.relative_to(REPO))] = file_hash(path)
    if np.any(np.diff(table.time) <= 0):
        raise ValueError(f"Non-monotonic STO times: {path}")
    return table


def torque_window(time, torque, mask):
    t, values = time[mask], torque[mask]
    if len(t) < 20:
        raise ValueError("Insufficient torque samples for the requested window")
    dt = np.diff(t)
    np.testing.assert_allclose(dt, np.median(dt), rtol=0, atol=2e-8)
    rate = np.diff(values) / dt
    sos = butter(4, 20.0, btype="highpass", fs=1.0 / float(np.median(dt)), output="sos")
    highpass = sosfiltfilt(sos, values)
    return {
        "samples": len(t), "start_time_s": float(t[0]), "end_time_s": float(t[-1]),
        "sample_interval_s": float(np.median(dt)), "min_nm": float(values.min()), "max_nm": float(values.max()),
        "rms_nm": float(np.sqrt(np.mean(values ** 2))),
        "positive_fraction": float(np.mean(values > 0)),
        "positive_above_8nm_fraction": float(np.mean(values > 8)),
        "positive_torque_integral_nm_s": float(np.trapezoid(np.maximum(values, 0), t)),
        "torque_rate_rms_nm_s": float(np.sqrt(np.mean(rate ** 2))),
        "total_variation_per_second_nm_s": float(np.abs(np.diff(values)).sum() / (t[-1] - t[0])),
        "highpass_above_20hz_rms_nm": float(np.sqrt(np.mean(highpass ** 2))),
    }


def physical_metrics(folder, trace):
    table = read_table(folder, "sea_torques")
    elapsed = table.time - table.time[0]
    torque, series = {}, {}
    for joint, short in zip(helpers.JOINTS, ("knee", "ankle")):
        column = f"SEA_{short.title()}_tau_spring"
        values = table.data[:, table.index[column]] * plotter.joint_sign(short)
        series[short] = values
        torque[short] = {
            "column": column, "canonical_sign": plotter.joint_sign(short),
            "whole_episode": torque_window(table.time, values, elapsed >= 0),
            "after_200ms": torque_window(table.time, values, elapsed >= .2 - 1e-8),
        }
        if elapsed[-1] >= 4.02:
            torque[short]["from_4s"] = torque_window(table.time, values, elapsed >= 4 - 1e-8)
    grf = read_table(folder, "online_grf")
    contact = {name: grf.data[:, grf.index[name]] for name in ("left_normal_force", "left_penetration", "left_in_contact")}
    last_fsm = trace[-1]["phase_fsm"]
    fsm_keys = ("valid_hs_count", "valid_to_count", "valid_cycle_count", "invalid_event_count", "resync_count", "hs_cancelled_count")
    events = []
    for row in trace:
        fsm = row["phase_fsm"]
        if any(fsm.get(k) for k in ("invalid_event_this_step", "resync_event_this_step", "cycle_rejected_this_step", "hs_cancelled_this_step")):
            events.append({"step": row["step"], "time": row["time"], **{k: fsm.get(k) for k in ("state_name", "invalid_event_type", "resync_event_this_step", "cycle_reject_reason", "hs_cancelled_this_step")}})
    metric = {
        "sea_torque": torque,
        "contact": {
            "normal_force_peak_n": float(contact["left_normal_force"].max()),
            "penetration_peak_mm": 1000 * float(contact["left_penetration"].max()),
            "contact_fraction": float(np.mean(contact["left_in_contact"] > .5)),
            "ankle_moment_flip_penalized_steps": sum(row["reward_terms"].get("grf_ankle_moment_flip_loss", 0) > 0 for row in trace),
        },
        "fsm_final": {key: last_fsm.get(key) for key in fsm_keys},
        "fsm_events": events,
    }
    return metric, {"time": table.time, "elapsed": elapsed, "torques": series}


def matched_trace_comparison(new, old):
    n = min(len(new), len(old))
    a, b = new[:n], old[:n]
    np.testing.assert_allclose([r["time"] for r in a], [r["time"] for r in b], rtol=0, atol=2e-8)
    result = {"common_steps": n, "same_completed_length": len(new) == len(old) == 500, "joint_differences": {}}
    for joint in helpers.JOINTS:
        target_error = tensor_error([r["imitation_target_q"][joint] for r in a], [r["imitation_target_q"][joint] for r in b])
        velocity_error = tensor_error([r["imitation_target_qdot"][joint] for r in a], [r["imitation_target_qdot"][joint] for r in b])
        if max(target_error["max_abs"], velocity_error["max_abs"]) > 2e-8:
            raise ValueError(f"RIGHT target differs from historical trace for {joint}")
        result["joint_differences"][joint] = {
            "target_q_rad": target_error, "target_qdot_rad_s": velocity_error,
            "actual_q_deg": tensor_error(np.degrees([r["prosthetic_state"][joint] for r in a]), np.degrees([r["prosthetic_state"][joint] for r in b])),
            "served_q_deg": tensor_error(np.degrees([r["prosthetic_state"][joint + "_served_ref"] for r in a]), np.degrees([r["prosthetic_state"][joint + "_served_ref"] for r in b])),
        }
    result["raw_action"] = tensor_error([r["raw_policy_action"] for r in a], [r["raw_policy_action"] for r in b])
    new_obs = np.array([r["actor_observation_vector_before"][:39] for r in a])
    old_obs = np.array([r["actor_observation_vector_before"][:39] for r in b])[:, ACTOR_PERMUTATION]
    result["actor_observations_semantically_remapped"] = tensor_error(new_obs, old_obs)
    result["reset_actor_max_abs_difference"] = float(np.abs(new_obs[0] - old_obs[0]).max())
    result["caveat"] = "Closed-loop differences compare states actually visited. Offline same-input functional equivalence is checked separately by probe.py."
    return result


def canonical_render(folder, summary, contract):
    context = plotter.valid_mlp_rollout_context(folder)
    if context is None:
        raise ValueError("Canonical plotting requires complete summary, trace and sim_outputs")
    saved, labels = [], []
    from matplotlib.text import Text
    original_save, original_discovery, argv = plotter.save_figure, plotter.latest_mlp_rollout_context, sys.argv

    def save_figure(fig, out_dir, filename):
        if filename.startswith("07_"):
            for label in fig.findobj(match=Text):
                text = label.get_text()
                text = text.replace("Shifted Sound Leg Target", "Historical Right Phase Target (knee 0.465; ankle 0.452)")
                text = text.replace("shifted sound leg target", "historical right phase target")
                label.set_text(text)
                if text:
                    labels.append(text)
        original_save(fig, out_dir, filename)
        saved.append(str(out_dir / filename))

    try:
        plotter.latest_mlp_rollout_context = lambda: context
        plotter.save_figure = save_figure
        sys.argv = [str(REPO / "plot/plotter.py"), "--mlp", "--results-dir", str(context.sim_outputs_dir),
                    "--prefix", "rollout_episode", "--setup", str(summary["setup_xml_path"]),
                    "--gait-side", "left", "--out-root", str(REPO / "plot")]
        plotter.main()
    finally:
        sys.argv = argv
        plotter.save_figure, plotter.latest_mlp_rollout_context = original_save, original_discovery
    if len(saved) != 7 or any(not Path(path).is_file() for path in saved):
        raise RuntimeError("Expected seven canonical plots")
    receipt = {"rollout": str(folder), "checkpoint": summary["checkpoint"], "diagnostic_contract": contract,
               "figures": saved, "figure7_labels": labels, "figure7_target": "Recorded imitation_target_q: historical RIGHT phase template",
               "figure7_middle_row": "Raw sound-side kinematics for context", "gait_side": "left"}
    text = json.dumps(receipt, indent=2) + "\n"
    (Path(saved[0]).parent / "plot_receipt.json").write_text(text, encoding="utf-8")
    (HERE / "canonical_plot_receipt.json").write_text(text, encoding="utf-8")
    return receipt


def comparison_figure(traces, physical):
    directory = REPO / "plot/09_23_2026_v26_transfer_comparison"
    directory.mkdir(exist_ok=True)
    fig, axes = plt.subplots(3, 2, figsize=(14, 11), layout="constrained")
    for label, trace in traces.items():
        elapsed = np.asarray([r["time"] for r in trace]) - trace[0]["policy_segment_times"][0]
        for col, (joint, short) in enumerate(zip(helpers.JOINTS, ("knee", "ankle"))):
            axes[0, col].plot(elapsed, plotter.joint_sign(short) * np.degrees([r["prosthetic_state"][joint] for r in trace]), label=LABELS[label], linewidth=1.15)
            axes[1, col].plot(physical[label]["elapsed"], physical[label]["torques"][short], label=LABELS[label], linewidth=.65, alpha=.85)
        axes[2, 0].plot(elapsed, [1000 * r["reward_terms"].get("grf_penetration_m", 0) for r in trace], label=LABELS[label])
        axes[2, 1].plot(elapsed, [r["reward_terms"].get("prosthetic_normal_force_bw", 0) for r in trace], label=LABELS[label])
    for source, style, target_label in (("v26_historical", "--", "Target RIGHT storico"), ("unified39_best72_left", ":", "Target LEFT")):
        trace = traces[source]
        elapsed = np.asarray([r["time"] for r in trace]) - trace[0]["policy_segment_times"][0]
        for col, (joint, short) in enumerate(zip(helpers.JOINTS, ("knee", "ankle"))):
            axes[0, col].plot(elapsed, plotter.joint_sign(short) * np.degrees([r["imitation_target_q"][joint] for r in trace]), style, color="black", linewidth=1, label=target_label)
    for ax, title, ylabel in zip(axes.flat, ("Ginocchio", "Caviglia", "Coppia SEA ginocchio", "Coppia SEA caviglia", "Penetrazione registrata dalla reward", "Carico piede sinistro"), ("Angolo [°]", "Angolo [°]", "Nm", "Nm", "mm", "Pesi corporei")):
        ax.set(title=title, ylabel=ylabel, xlabel="Tempo dal reset [s]")
        ax.grid(alpha=.2)
        ax.legend(fontsize=7)
    fig.suptitle("Pesi V26 nel contenitore unified39: rollout deterministico, FSM v3\nBest72 ha un target diverso: confronto descrittivo, non equivalenza statistica")
    path = directory / "comparison.png"
    fig.savefig(path, dpi=170)
    plt.close(fig)
    return str(path)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rollout", type=Path, default=DEFAULT_NEW)
    parser.add_argument("--render", action="store_true")
    args = parser.parse_args()
    folder = args.rollout.resolve()
    summary = helpers.read(folder / "rollout_summary.json")
    checkpoint = Path(summary["checkpoint"])
    if not checkpoint.is_absolute():
        checkpoint = REPO / checkpoint
    contract = helpers.read(checkpoint / "diagnostic_contract.json")
    target_source = contract.get("teacher_source", contract.get("target_channels", {}).get("teacher_source"))
    if target_source != "prescribed_right_phase_template":
        raise ValueError("Expected explicit historical RIGHT diagnostic contract")
    result = {"rollouts": {}, "method": {
        "new_training": False, "new_simulations_by_analysis": 0,
        "torque_sign": "Canonical plot convention: knee inverted, ankle unchanged; spring output torque",
        "chattering_proxy": "Fourth-order Butterworth high-pass above 20 Hz, zero-phase; plus derivative RMS and total variation per second. Identical processing, no claim that all high-frequency content is pathological.",
        "missing_fsm_counters": "Historical missing fields remain null; they are not assumed zero.",
        "limitation": "One deterministic start. This diagnostic does not establish stochastic robustness or statistical training equivalence.",
    }}
    traces, physical, summaries = {}, {}, {}
    folders = {"v26_historical": OLD, "v26_in_unified39": folder, "unified39_best72_left": BEST72}
    for label, location in folders.items():
        metrics, trace, s = helpers.rollout_metrics(location, historical=label == "v26_historical", scale=1 if label == "unified39_best72_left" else None)
        extra, raw = physical_metrics(location, trace)
        metrics.update(extra)
        result["rollouts"][label], traces[label], physical[label], summaries[label] = metrics, trace, raw, s
    result["historical_reproduction"] = matched_trace_comparison(traces["v26_in_unified39"], traces["v26_historical"])
    result["comparison_figure"] = comparison_figure(traces, physical)
    if args.render:
        result["canonical_plots"] = canonical_render(folder, summary, contract)
    (HERE / "metrics.json").write_text(json.dumps(result, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    (HERE / "analysis_input_hashes.json").write_text(json.dumps(helpers.INPUT_HASHES, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"metrics": str(HERE / "metrics.json"), "reproduction": result["historical_reproduction"], "canonical_plots": result.get("canonical_plots", {}).get("figures", [])}, indent=2))


if __name__ == "__main__":
    main()
