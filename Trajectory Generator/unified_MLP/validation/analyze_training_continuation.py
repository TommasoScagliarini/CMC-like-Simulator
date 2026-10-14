"""Offline statistics and figures for the completed best98 -> 200 continuation."""
from __future__ import annotations

import csv
import json
import pickle
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from tensorboard.backend.event_processing.event_accumulator import EventAccumulator

import analyze_training_0_100 as previous

U = Path(__file__).resolve().parents[1]
ROOT = U.parents[1]
PARENT = U / "runs/training/pretraining_v1_pen1_pilot_2026-09-14"
RUN = U / "runs/training/pretraining_v1_pen1_best98_to200_2026-09-15"
OUTPUT = U / "runs/validation/pretraining_v1_continuation_200_2026-09-16"
BLOCKS = ((1, 30), (31, 50), (51, 80), (81, 98), (99, 120), (121, 140),
          (141, 160), (161, 180), (181, 200), (191, 200))


def read_history(run):
    return [json.loads(line) for line in (run / "train_iterations.jsonl").read_text().splitlines() if line.strip()]


def block_stats(history, lo, hi, tb):
    rows = history[lo-1:hi]
    old = previous.end_counts(history[lo-2]) if lo > 1 else {}
    latest = previous.end_counts(history[hi-1])
    ends = {key: value-old.get(key, 0) for key, value in latest.items()}
    assert all(n >= 0 for n in ends.values())
    per_start = {}
    for name, key in zip(previous.STARTS, sorted(rows[0]["per_start_metrics"])):
        entries = [r["per_start_metrics"][key] for r in rows]
        n = sum(e["completed_episodes"] for e in entries)
        per_start[name] = {
            "completed_episodes": n,
            "return_mean": sum(e["episode_return_sum"] or 0 for e in entries)/n,
            "duration_mean_s": .01*sum((e["episode_length_mean"] or 0)*e["completed_episodes"] for e in entries)/n,
        }
    assert sum(ends.values()) == sum(e["completed_episodes"] for e in per_start.values())
    return {"first": lo, "last": hi,
            "reported_return_mean": float(np.mean([r["episode_return_mean"] for r in rows])),
            "reported_duration_mean_s": .01*float(np.mean([r["episode_len_mean"] for r in rows])),
            "completed_episodes": sum(ends.values()), "termination_counts": ends,
            "time_limit_fraction": ends.get("episode_time_limit", 0)/sum(ends.values()),
            "per_start": per_start,
            "tb_rolling_component_means": {tag: float(np.mean([values[i] for i in range(lo, hi+1)])) for tag, values in tb.items()},
            "max_minibatch_mean_kl": max(r["max_minibatch_mean_kl_loss"] for r in rows)}


def save(fig, name):
    for extension in ("png", "pdf"):
        fig.savefig(OUTPUT / f"{name}.{extension}", dpi=170, bbox_inches="tight")
    plt.close(fig)


def replay_policy(evaluation, sources):
    result = []
    for case in evaluation["cases"]:
        directory = Path(case["source_directory"])
        summary_path, trace_path = directory / "rollout_summary.json", directory / "rollout_policy_trace.json"
        summary, trace = previous.read(summary_path), previous.read(trace_path)
        weights_path = Path(summary["checkpoint"]) / "module_state.pkl"
        sources.update((summary_path, trace_path, weights_path))
        with weights_path.open("rb") as stream:
            weights = pickle.load(stream)
        obs = np.asarray([row["actor_observation_vector_before"][:35] for row in trace], dtype=np.float32)
        z = np.tanh(obs @ np.asarray(weights["pi_encoder.0.weight"]).T + np.asarray(weights["pi_encoder.0.bias"]))
        z = np.tanh(z @ np.asarray(weights["pi_encoder.2.weight"]).T + np.asarray(weights["pi_encoder.2.bias"]))
        logits = z @ np.asarray(weights["pi.1.weight"]).T + np.asarray(weights["pi.1.bias"])
        raw = np.asarray([row["raw_policy_action"] for row in trace])
        applied = np.asarray([row["applied_policy_action"] for row in trace])
        error = float(np.max(np.abs(raw-logits[:, :2])))
        assert error < 5e-6
        invalid_events, count_before = [], 0
        origin = trace[0]["time"]-.01
        for row in trace:
            fsm = row["phase_fsm"]
            count = int(fsm["invalid_event_count"])
            assert count >= count_before
            if count > count_before:
                invalid_events.append({"step": row["step"], "elapsed_s": row["time"]-origin,
                                       "counter_increment": count-count_before,
                                       "last_reported_type": fsm["invalid_event_type"]})
            count_before = count
        assert sum(event["counter_increment"] for event in invalid_events) == case["full_episode"]["phase"]["invalid_event_count"]
        result.append({"iteration": case["checkpoint_iteration"], "start": case["start"],
                       "samples": len(trace), "mean_replay_max_abs_error": error,
                       "sigma_median_normalized": np.median(np.exp(logits[:, 2:]), axis=0).tolist(),
                       "clipped_step_fraction": float(np.mean(np.any(np.abs(raw-applied) > 1e-6, axis=1))),
                       "invalid_events": invalid_events,
                       "scope": "Saved deterministic states, not the training-state distribution"})
    return result


def main():
    OUTPUT.mkdir(parents=True, exist_ok=True)
    parent, history = read_history(PARENT), read_history(RUN)
    assert len(parent) == 100 and [r["iteration"] for r in history] == list(range(1, 201))
    assert parent[:98] == history[:98]
    assert all(r["unified_phase"] == "imitation" and r["exact_start_balance"]["pass"] for r in history)
    assert all(r["num_env_steps_sampled_lifetime"] == 4608*r["iteration"] for r in history)
    assert all(r["training_health"]["missing_telemetry_rows"] == 0 and r["kl_nonfinite_count"] == 0 for r in history)
    summary = previous.read(RUN / "summary.json")
    sources = {PARENT / "train_iterations.jsonl", RUN / "train_iterations.jsonl", RUN / "summary.json",
               RUN / "training_cfg.resolved.yaml", RUN / "startup_verification.json"}
    tb = {tag: {} for tag in (*previous.COMPONENTS, "env_runners/episode_return_mean")}
    for run, lower, upper in ((PARENT, 1, 98), (RUN, 99, 200)):
        for path in sorted((run / "tensorboard").glob("events*")):
            sources.add(path)
            accumulator = EventAccumulator(str(path), size_guidance={"scalars": 0})
            accumulator.Reload()
            for tag in tb:
                for event in accumulator.Scalars(tag):
                    assert event.step % 4608 == 0
                    iteration = event.step//4608
                    if not lower <= iteration <= upper:
                        continue
                    assert iteration not in tb[tag]
                    assert np.isfinite(event.value)
                    tb[tag][iteration] = event.value
    assert all(sorted(values) == list(range(1, 201)) for values in tb.values())
    assert all(np.isclose(tb["env_runners/episode_return_mean"][r["iteration"]], r["episode_return_mean"], rtol=1e-6) for r in history)
    blocks = [block_stats(history, lo, hi, tb) for lo, hi in BLOCKS]
    total = block_stats(history, 99, 200, tb)
    best = max(history, key=lambda row: row["episode_return_mean"])
    result = {"scope": "Offline analysis of preserved completed training",
              "lineage": "parent iterations1-98 + continuation99-200; parent99-100 are a separate branch",
              "iterations_completed_this_process": summary["iterations_completed_this_process"],
              "new_transitions": 102*4608, "new_transitions_per_start": 102*1536,
              "ancestral_transitions": 200*4608, "all_campaign_transitions_including_discarded_parent99_100": 202*4608,
              "training_wall_time_s": summary["elapsed_wall_time_s"],
              "best_iteration": best["iteration"], "best_return": best["episode_return_mean"],
              "last_return": history[-1]["episode_return_mean"],
              "total_continuation": total, "blocks": blocks,
              "configuration": {k: summary[k] for k in ("lr", "num_epochs", "num_env_runners", "freeze_logstd", "fixed_exploration_sigma", "restart_count", "crash_restart_count", "skipped_iterations")},
              "interpretation_limits": [
                  "Training return/length block means are descriptive averages of reported iteration metrics.",
                  "Reward components are rolling100-step worker aggregates, not exact episode decompositions.",
                  "Episodes can span policy updates; a time-limit ending does not certify gait cycles.",
                  "A single training seed and sparse deterministic checkpoints do not establish statistical robustness.",
                  "Known training/evaluation action-clipping reward mismatch remained unchanged.",
                  "No CP0 deterministic baseline; original CP100 is not an ancestor of the resumed branch.",
              ]}
    evaluation_path = OUTPUT / "milestone_evaluation.json"
    if evaluation_path.exists():
        evaluation = previous.read(evaluation_path)
        assert evaluation["status"] == "complete"
        sources.add(evaluation_path)
        result["policy_replay"] = replay_policy(evaluation, sources)
        result["quality_gates"] = evaluation["quality_gates"]
        fig, axes = plt.subplots(2, 3, figsize=(12, 6.8))
        checkpoints = (98, 100, 198, 200)
        max_cycles = max(c["full_episode"]["phase"]["valid_cycle_count"] for c in evaluation["cases"])
        for column, start in enumerate(previous.STARTS):
            cases = [next(c for c in evaluation["cases"] if c["checkpoint_iteration"] == i and c["start"] == start) for i in checkpoints]
            for row, label in ((0, "Durata [s]"), (1, "Cicli validi")):
                values = [c["full_episode"]["duration_s"] if row == 0 else c["full_episode"]["phase"]["valid_cycle_count"] for c in cases]
                ax = axes[row, column]
                ax.bar(range(4), values, color=["#999999", "#CC79A7", "#0072B2", "#009E73"])
                ax.set(xticks=range(4), xticklabels=[str(i) for i in checkpoints], title=start if row == 0 else "", ylabel=label, ylim=(0, 5.7 if row == 0 else max(2, max_cycles+.7)))
                if row == 1:
                    ax.set_yticks(range(int(max_cycles)+1))
                for i, value in enumerate(values):
                    ax.text(i, value+.05, f"{value:.2f}" if row == 0 else str(int(value)), ha="center")
                ax.grid(alpha=.2, axis="y")
        fig.suptitle("Valutazione deterministica: CP98 di partenza, CP100 precedente, CP198 e CP200\nStessi tre start e seed 123; CP100 appartiene al ramo originale")
        fig.tight_layout(rect=(0, 0, 1, .91))
        save(fig, "deterministic_98_200")
        fig, axes = plt.subplots(2, 3, figsize=(13, 6.5), sharex=True, sharey="row")
        for column, start in enumerate(previous.STARTS):
            traces = {}
            for iteration in (198, 200):
                case = next(c for c in evaluation["cases"] if c["checkpoint_iteration"] == iteration and c["start"] == start)
                traces[iteration] = previous.read(Path(case["source_directory"]) / "rollout_policy_trace.json")
            reference = traces[200]
            origin = reference[0]["time"]-.01
            for row, (joint, label) in enumerate((("pros_knee_angle", "Ginocchio [°]"), ("pros_ankle_angle", "Caviglia [°]"))):
                ax = axes[row, column]
                ax.plot([r["time"]-origin for r in reference], np.rad2deg([r["imitation_target_q"][joint] for r in reference]), color="black", lw=1.7, label="Prescribed")
                for iteration, color in ((198, "#0072B2"), (200, "#D55E00")):
                    trace = traces[iteration]
                    ax.plot([r["time"]-origin for r in trace], np.rad2deg([r["prosthetic_state"][joint] for r in trace]), color=color, lw=1.4, label=f"CP{iteration} effettivo")
                ax.set(xlim=(0, 5), title=start if row == 0 else "", ylabel=label if column == 0 else "", xlabel="Tempo dal reset [s]" if row else "")
                ax.grid(alpha=.2)
        axes[0, 0].legend(fontsize=8, loc="lower left")
        fig.suptitle("Forma della traiettoria: prescribed e movimento effettivo ai checkpoint finali\nStesse finestre complete di 5 s; nessuno sfasamento applicato")
        fig.tight_layout(rect=(0, 0, 1, .91))
        save(fig, "kinematics_198_200")
    result["source_sha256"] = {str(path.relative_to(ROOT)): previous.digest(path) for path in sorted(sources)}
    (OUTPUT / "training_analysis.json").write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False)+"\n")
    with (OUTPUT / "training_series.csv").open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(("iteration", "origin", "reported_return", "reported_duration_s", *tb))
        for row in history:
            i = row["iteration"]
            writer.writerow((i, "parent" if i <= 98 else "continuation", row["episode_return_mean"], .01*row["episode_len_mean"], *(values[i] for values in tb.values())))
    plt.rcParams.update({"font.size": 10, "axes.spines.top": False, "axes.spines.right": False})
    fig, axes = plt.subplots(2, 2, figsize=(13, 8))
    for ax, key, factor, title in ((axes[0, 0], "episode_return_mean", 1, "Return medio riportato"), (axes[0, 1], "episode_len_mean", .01, "Durata media riportata [s]")):
        values = np.asarray([row[key]*factor for row in history])
        ax.plot(range(1, 201), values, color=".75", lw=1, label="Per iterazione")
        ax.plot(range(1, 201), previous.smooth(values), color="#0072B2", lw=2, label="Media mobile 10")
        ax.set(title=title, xlabel="Iterazione globale")
        ax.legend(fontsize=8)
    for tag, label, color in (("reward/served_imitation_score", "Reference/prescribed", "#0072B2"), ("reward/imitation_score", "Actual/prescribed", "#009E73"), ("reward/phase_timeout_penalty_term", "Penalità timeout", "#D55E00")):
        axes[1, 0].plot(range(1, 201), previous.smooth([tb[tag][i] for i in range(1, 201)]), label=label, color=color, lw=2)
    axes[1, 0].set(title="Componenti reward TB: media mobile 10", xlabel="Iterazione globale")
    axes[1, 0].legend(fontsize=8)
    bars = blocks[:-1]
    bottom = np.zeros(len(bars))
    for index, (label, color) in enumerate(zip(("Limite 5 s", "Penetrazione", "Timeout swing", "Altre protezioni"), ("#009E73", "#D55E00", "#CC79A7", "#777777"))):
        values = []
        for block in bars:
            ends = block["termination_counts"]
            counts = (ends.get("episode_time_limit", 0), ends.get("grf_penetration", 0)+ends.get("grf_penetration_hard", 0), ends.get("phase_timeout_swing", 0))
            n = counts[index] if index < 3 else block["completed_episodes"]-sum(counts)
            values.append(100*n/block["completed_episodes"])
        axes[1, 1].bar(range(len(bars)), values, bottom=bottom, label=label, color=color)
        bottom += values
    axes[1, 1].set(xticks=range(len(bars)), xticklabels=[f"{b['first']}–{b['last']}" for b in bars], ylabel="Episodi terminati [%]", title="Esito degli episodi durante il training")
    axes[1, 1].tick_params(axis="x", labelsize=8)
    axes[1, 1].legend(fontsize=8, ncol=2)
    for ax in axes.flat:
        ax.grid(alpha=.2, axis="y")
    for ax in (axes[0, 0], axes[0, 1], axes[1, 0]):
        ax.axvline(98.5, color=".35", ls="--", lw=1)
    fig.suptitle("V1 imitativa: percorso completo 1–200\nLinea tratteggiata: resume da CP98; le vecchie iterazioni 99–100 sono escluse da questa curva")
    fig.tight_layout(rect=(0, 0, 1, .93))
    save(fig, "training_1_200")
    print(json.dumps({"output": str(OUTPUT), "best_iteration": best["iteration"],
                      "new_updates": 102, "evaluated_cases": len(result.get("policy_replay", [])),
                      "continuation_terminations": total["termination_counts"]}, indent=2))


if __name__ == "__main__":
    main()
