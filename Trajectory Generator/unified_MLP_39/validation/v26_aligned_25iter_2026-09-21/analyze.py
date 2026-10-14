"""Audit the completed 25-iteration screening run and its deterministic rollout."""
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
os.environ.setdefault("MPLCONFIGDIR", str(HERE / ".matplotlib"))
os.environ.setdefault("OMP_NUM_THREADS", "1")
sys.path.insert(0, str(TG))

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from tensorboard.backend.event_processing.event_accumulator import EventAccumulator

RUN = PACKAGE / "runs/training/MLP39_v26_aligned_left_phase_fsmv3_25iter_2026-09-21"
PLAN = PACKAGE / "runs/plans/v26_aligned_left_phase_fsmv3_25iter_2026-09-21"
V26 = TG / "runs/training/MLP_imitation_native_v26_08-20-2026_june_equiv_100iter"
PREVIOUS = PACKAGE / "runs/training/MLP39_phase_velocity_scale1_2026-09-18"
ROLLOUT = PACKAGE / "runs/rollout/MLP39_v26_aligned_left_phase_fsmv3_25iter_2026-09-21_best25"
HISTORICAL_ROLLOUT = TG / "runs/rollout/MLP_imitation_native_v26_08-20-2026_june_equiv_100iter_rollout"
JOINTS = ("pros_knee_angle", "pros_ankle_angle")


def read(path):
    return json.loads(path.read_text())


def history(path):
    return [json.loads(line) for line in (path / "train_iterations.jsonl").read_text().splitlines() if line.strip()]


def write(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")


def endings(rows, low, high):
    before = rows[low - 2]["termination_metrics"] if low > 1 else {}
    after = rows[high - 1]["termination_metrics"]
    result = {key.split("/episode_end/")[-1]: int(value - before.get(key, 0))
              for key, value in after.items() if "/episode_end/" in key}
    assert all(value >= 0 for value in result.values())
    return {"counts": result, "recorded_episodes": sum(result.values()),
            "full_fraction": result.get("episode_time_limit", 0) / sum(result.values())}


def training_metrics(path):
    rows = history(path)
    events = EventAccumulator(str(path / "tensorboard"), size_guidance={"scalars": 0})
    events.Reload()
    tags = ["reward_diagnostic/" + joint + "_imitation_position_loss" for joint in JOINTS]
    tags += ["reward/grf_penetration_loss", "reward/imitation_score", "reward/served_imitation_score"]
    scalars = {tag: [float(value.value) for value in events.Scalars(tag)[:25]] for tag in tags}
    blocks = {}
    for low, high in ((1, 10), (11, 20), (21, 25)):
        blocks[f"{low}-{high}"] = {
            **endings(rows, low, high),
            "iteration_mean_return": float(np.mean([row["episode_return_mean"] for row in rows[low - 1:high]])),
            "iteration_mean_length": float(np.mean([row["episode_len_mean"] for row in rows[low - 1:high]])),
        }
    return rows, {"iteration25": {key: rows[24][key] for key in
                                ("episode_return_mean", "episode_len_mean", "num_env_steps_sampled_lifetime")},
                  "blocks": blocks, "tensorboard_smoothed": scalars}


def training_plot(histories, metrics):
    fig, axes = plt.subplots(2, 2, figsize=(12, 8), layout="constrained")
    for name, color, label in (("current", "#2563eb", "39D, configurazione riallineata"),
                               ("v26", "#d97706", "V26 del 20 agosto")):
        rows = histories[name][:25]
        x = [row["iteration"] for row in rows]
        axes[0, 0].plot(x, [row["episode_return_mean"] for row in rows], color=color, label=label)
        axes[0, 1].plot(x, [row["episode_len_mean"] * .01 for row in rows], color=color)
        axes[1, 0].plot(x, [100 * endings(rows, max(1, i - 4), i)["full_fraction"] for i in x], color=color)
        for joint, style, short in zip(JOINTS, ("-", "--"), ("ginocchio", "caviglia")):
            values = metrics[name]["tensorboard_smoothed"]["reward_diagnostic/" + joint + "_imitation_position_loss"]
            axes[1, 1].semilogy(x, values, color=color, linestyle=style, label=label.split(",")[0] + ", " + short)
    axes[0, 0].set_ylabel("Return medio")
    axes[0, 0].legend(fontsize=9)
    axes[0, 1].set_ylabel("Durata media episodi (s)")
    axes[0, 1].axhline(5, color="gray", linestyle=":")
    axes[1, 0].set_ylabel("Episodi completi registrati (%)\nFinestra delle ultime 5 iterazioni")
    axes[1, 0].set_ylim(0, 105)
    axes[1, 1].set_ylabel("Errore quadratico normalizzato\nMedia mobile TensorBoard")
    axes[1, 1].legend(fontsize=8)
    for axis in axes.flat:
        axis.set_xlabel("Iterazione")
        axis.grid(alpha=.2)
    fig.suptitle("Screening a 25 iterazioni — target diversi; stesso numero di iterazioni, campioni differenti")
    fig.savefig(HERE / "training_comparison.png", dpi=160)
    plt.close(fig)


def phase_audit(trace):
    cycles = []
    transitions = []
    for sample in trace:
        phase = sample.get("phase_fsm", {})
        for event in phase.get("accepted_transitions_this_step", []):
            transitions.append({key: event[key] for key in
                                ("event", "event_time_s", "segment_valid")})
        if phase.get("cycle_completed_this_step"):
            cycles.append({"time_s": sample["time"],
                           "period_s": phase["last_period_s"],
                           "stance_fraction": phase["last_stance_fraction"]})
    final = trace[-1].get("phase_fsm", {})
    return {"completed_cycles": cycles, "transitions": transitions,
            "final": {key: final.get(key) for key in
                      ("fsm_behaviour_version", "resync_count", "hs_cancelled_count",
                       "invalid_event_count", "cycle_degraded")}}


def sampling_budget_audit(histories):
    runs = {}
    for name in ("current", "v26"):
        rows = histories[name][:25]
        previous = 0
        batches = []
        for row in rows:
            total = row["num_env_steps_sampled_lifetime"]
            sampled = int(total - previous)
            learner_input = int(row["learner_metrics"][
                "learners/__all_modules__/learner_connector_sum_episodes_length_in"])
            assert sampled == learner_input
            batches.append({"iteration": row["iteration"], "sampled": sampled,
                            "learner_input": learner_input})
            previous = total
        trained = rows[-1]["learner_metrics"][
            "learners/default_policy/num_module_steps_trained_lifetime"]
        runs[name] = {"nominal_batch_size": 4096, "batches": batches,
                      "sampled_total": int(previous),
                      "excess_over_25_nominal_batches": int(previous) - 25 * 4096,
                      "processed_minibatch_samples": int(trained),
                      "minibatch_passes_at_size512": int(trained / 512)}
    windows = {}
    for name, low, high in (("current", 21, 25), ("v26", 16, 20)):
        rows = histories[name]
        before, after = rows[low - 2], rows[high - 1]
        windows[name] = {
            "iterations": [low, high],
            "sample_budget_start": before["num_env_steps_sampled_lifetime"],
            "sample_budget_end": after["num_env_steps_sampled_lifetime"],
            "window_steps": after["num_env_steps_sampled_lifetime"]
                            - before["num_env_steps_sampled_lifetime"],
            **endings(rows, low, high),
        }
    return {"runs": runs, "matched_experience_windows": windows,
            "v26_extra_sampling_explanation": {
                "historical_evidence": "reports/user/2026-08-20_verdetto_fresh_run_june_equivalent_nativo_v26.md",
                "historical_guard_fix": "reports/user/2026-08-21_report_approfondito_dal_training_imitativo_v26_alla_fsm_v3.md",
                "mechanism": "A failed worker leaves the parallel round below the nominal batch threshold; RLlib collects another round and trains on the whole returned batch.",
                "example": "7562 = 2 * (316 + 11 * 315); current healthy round = 316 + 12 * 315 = 4096.",
                "caveat": "Worker-failure mechanism is documented historically and matches installed RLlib source and batch arithmetic; original per-round failure logs are not reconstructed here."}}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--training-only", action="store_true")
    args = parser.parse_args()
    summary = read(RUN / "summary.json")
    supervisor = read(RUN / "supervisor_state.json")
    assert summary["ok"] and summary["iterations_completed"] == 25
    assert summary["initialization_mode"] == "fresh" and not summary["resume_from"]
    assert supervisor["restart_count"] == supervisor["crash_restart_count"] == 0
    assert supervisor["skipped_iterations"] == []
    for path, digest in read(PLAN / "source_hashes.json").items():
        assert hashlib.sha256((REPO / path).read_bytes()).hexdigest() == digest, path
    histories, metrics = {}, {}
    for name, path in (("current", RUN), ("v26", V26), ("previous_scale1", PREVIOUS)):
        histories[name], metrics[name] = training_metrics(path)
    rows = histories["current"]
    budget_audit = sampling_budget_audit(histories)
    write(HERE / "sampling_budget_audit.json", budget_audit)
    assert [row["iteration"] for row in rows] == list(range(1, 26))
    for row in rows:
        audit = row["unified_phase_audit"]
        mask = audit["target_availability_batch"]
        assert mask["visible_steps"] == mask["total_steps"] == 4096
        assert audit["mean_changed"] and audit["critic_changed"]
        assert row["training_health"]["missing_telemetry_rows"] == 0
        assert all(np.isfinite(row[key]) for key in ("policy_loss", "vf_loss", "mean_kl_loss"))
    health = {key: sum(row["training_health"].get(key, 0) or 0 for row in rows)
              for key in ("observed_rows", "missing_telemetry_rows", "resync_event_rows",
                          "phase_timeout_stance_rows", "morphology_causal_contract_failure_rows",
                          "timeout_side_disagreement_rows")}
    matched = min(histories["v26"], key=lambda row: abs(row["num_env_steps_sampled_lifetime"] - 102400))
    result = {"training": metrics, "audit": {"passed": True, "supervisor": supervisor,
              "health": health, "best_iteration": read(RUN / "checkpoint_best_meta.json")["logical_iteration"],
              "elapsed_wall_time_s": summary["elapsed_wall_time_s"],
              "v26_nearest_sample_budget": {key: matched[key] for key in
                    ("iteration", "num_env_steps_sampled_lifetime", "episode_return_mean", "episode_len_mean")}},
              "limitations": ["Single training seed; no statistical equivalence claim.",
                "Historical intermediate V26 checkpoints are unavailable; gait comparison uses V26 best87.",
                "Historical worker deaths can be missing from episode-end counters.",
                "Smoothed training losses are normalized by each target's own range.",
                "The historical run sampled more transitions by iteration25.",
                "Several settings were changed together; this does not isolate one causal parameter."]}
    result["sampling_budget_audit"] = budget_audit
    training_plot(histories, metrics)
    write(HERE / "training_metrics.json", result)
    if args.training_only:
        print("Training audit passed; saved metrics and comparison plot.")
        return

    spec = importlib.util.spec_from_file_location("prior_analysis", PACKAGE / "validation/ablation_report_2026-09-20/analyze_results.py")
    helpers = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(helpers)
    current, trace, current_summary = helpers.rollout_metrics(ROLLOUT, scale=1)
    old, old_trace, _ = helpers.rollout_metrics(HISTORICAL_ROLLOUT, historical=True)
    result["rollout"] = current
    result["historical_best87_rollout"] = old
    result["phase_audit"] = {"current": phase_audit(trace),
                             "v26_best87": phase_audit(old_trace)}
    current_summary = {**current_summary,
                       "checkpoint": str((REPO / current_summary["checkpoint"]).resolve())}
    result["policy_audit"] = helpers.policy_audit(trace, current_summary, slew=False)
    length = min(len(trace), len(old_trace))
    result["matched_rollout_prefix"] = {"steps": length,
        "current": {joint: helpers.joint_metrics(trace[:length], joint) for joint in JOINTS},
        "v26_best87": {joint: helpers.joint_metrics(old_trace[:length], joint) for joint in JOINTS}}
    fig, axes = plt.subplots(2, 2, figsize=(12, 7), layout="constrained", sharex="col")
    for column, (samples, label) in enumerate(((trace, "39D, best25"), (old_trace, "V26, best87"))):
        time = np.asarray([row["time"] for row in samples]) - samples[0]["policy_segment_times"][0]
        for row, joint in enumerate(JOINTS):
            axis = axes[row, column]
            for field, key, legend, color, style in (("imitation_target_q", joint, "Target prescritto", "black", "--"),
                 ("prosthetic_state", joint + "_served_ref", "Reference servita", "#d97706", ":"),
                 ("prosthetic_state", joint, "Giunto simulato", "#2563eb", "-")):
                axis.plot(time, np.rad2deg([sample[field][key] for sample in samples]), label=legend, color=color, linestyle=style)
            axis.set_title(label + (" — ginocchio" if row == 0 else " — caviglia"))
            axis.set_ylabel("Angolo (°)")
            axis.set_xlabel("Tempo dall'inizio episodio (s)")
            axis.grid(alpha=.2)
    axes[0, 0].legend(fontsize=8)
    fig.suptitle("Rollout deterministici, stesso start fisico — ogni policy confrontata con il proprio target")
    fig.savefig(HERE / "rollout_comparison.png", dpi=170)
    plt.close(fig)
    write(HERE / "metrics.json", result)
    write(HERE / "rollout_input_hashes.json", helpers.INPUT_HASHES)
    print(json.dumps({"audit": result["audit"], "rollout": current,
                      "policy_audit": result["policy_audit"]}, indent=2))


if __name__ == "__main__":
    main()
