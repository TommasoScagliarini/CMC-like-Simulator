"""Offline analysis of the completed, user-paused September 14 V1 campaign.

Reads existing history, TensorBoard and deterministic traces. Does not import
the simulator, start training, sample actions or alter any checkpoint.
"""
from __future__ import annotations

import csv
import hashlib
import json
import pickle
import pickletools
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from tensorboard.backend.event_processing.event_accumulator import EventAccumulator


UNIFIED = Path(__file__).resolve().parents[1]
REPO = UNIFIED.parents[1]
RUN = UNIFIED / "runs/training/pretraining_v1_pen1_pilot_2026-09-14"
PLAN = UNIFIED / "runs/plans/pretraining_50_100_feasibility_2026-09-14"
EVALUATION = UNIFIED / "runs/validation/pretraining_v1_pen1_milestone_100_2026-09-14"
OUTPUT = UNIFIED / "runs/validation/pretraining_v1_global_0_100_2026-09-15"
STARTS = ("early", "center", "late")
BLOCKS = ((1, 10), (11, 30), (31, 50), (51, 70), (71, 90), (91, 100))
COMPONENTS = (
    "reward/reward", "reward/reward_base", "reward/served_imitation_score",
    "reward/imitation_score", "reward/penalty", "reward/grf_penetration_term",
    "reward/safety_term", "reward/phase_timeout_penalty_term",
    "reward/grf_ankle_moment_flip_term", "reward/policy_action_clip_fraction",
    "learners/default_policy/entropy", "learners/default_policy/vf_explained_var",
)


def read(path):
    return json.loads(path.read_text(encoding="utf-8"))


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def end_counts(row):
    return {k.rsplit("/", 1)[-1]: int(v) for k, v in row["termination_metrics"].items()
            if "/episode_end/" in k}


def serialized_boolean(path, key):
    """Read a frozen config flag without unpickling an RLlib algorithm."""
    ops = list(pickletools.genops(path.read_bytes()))
    values = []
    for index, (_, value, _) in enumerate(ops):
        if value != key:
            continue
        following = next(op.name for op, _, _ in ops[index+1:] if op.name != "MEMOIZE")
        assert following in ("NEWTRUE", "NEWFALSE"), (key, following)
        values.append(following == "NEWTRUE")
    assert values and len(set(values)) == 1, (key, values)
    return values[0]


def smooth(values, width=10):
    return np.asarray([np.mean(values[max(0, i-width+1):i+1]) for i in range(len(values))])


def save_figure(fig, name):
    fig.savefig(OUTPUT / f"{name}.png", dpi=170, bbox_inches="tight")
    fig.savefig(OUTPUT / f"{name}.pdf", bbox_inches="tight")
    plt.close(fig)


def main():
    assert read(PLAN / "user_pause.json")["automatic_continuation_authorized"] is False
    history_path = RUN / "train_iterations.jsonl"
    history = [json.loads(line) for line in history_path.read_text().splitlines() if line.strip()]
    assert [r["iteration"] for r in history] == list(range(1, 101))
    assert all(r["unified_phase"] == "imitation" and r["exact_start_balance"]["pass"] for r in history)
    assert all(r["num_env_steps_sampled_lifetime"] == r["iteration"] * 4608 for r in history)
    source_paths = {history_path, PLAN / "training_summary_100.json", EVALUATION / "milestone_evaluation.json"}
    tb = {}
    for path in sorted((RUN / "tensorboard").glob("events*")):
        source_paths.add(path)
        accumulator = EventAccumulator(str(path), size_guidance={"scalars": 0})
        accumulator.Reload()
        for tag in (*COMPONENTS, "env_runners/episode_return_mean"):
            if tag not in accumulator.Tags()["scalars"]:
                continue
            for event in accumulator.Scalars(tag):
                assert event.step % 4608 == 0
                iteration = event.step // 4608
                assert iteration not in tb.setdefault(tag, {}), (tag, iteration)
                assert np.isfinite(event.value)
                tb[tag][iteration] = event.value
    for r in history:
        assert np.isclose(tb["env_runners/episode_return_mean"][r["iteration"]], r["episode_return_mean"], rtol=1e-6)
    blocks = []
    for lo, hi in BLOCKS:
        rows = history[lo-1:hi]
        previous = end_counts(history[lo-2]) if lo > 1 else {}
        last = end_counts(history[hi-1])
        ends = {k: n-previous.get(k, 0) for k, n in last.items()}
        assert all(n >= 0 for n in ends.values())
        per_start = {}
        for name, key in zip(STARTS, sorted(rows[0]["per_start_metrics"])):
            entries = [r["per_start_metrics"][key] for r in rows]
            n = sum(r["completed_episodes"] for r in entries)
            per_start[name] = {
                "completed_episodes": int(n),
                "return_mean": sum(r["episode_return_sum"] or 0 for r in entries) / n,
                "duration_mean_s": .01 * sum((r["episode_length_mean"] or 0) * r["completed_episodes"] for r in entries) / n,
            }
        assert sum(ends.values()) == sum(r["completed_episodes"] for r in per_start.values())
        blocks.append({
            "first": lo, "last": hi,
            "mean_reported_return": float(np.mean([r["episode_return_mean"] for r in rows])),
            "mean_reported_duration_s": float(np.mean([r["episode_len_mean"]*.01 for r in rows])),
            "completed_episodes": sum(ends.values()), "termination_counts": ends,
            "time_limit_fraction": ends.get("episode_time_limit", 0) / sum(ends.values()),
            "per_start": per_start,
            "tb_rolling_component_means": {tag: float(np.mean([tb[tag][i] for i in range(lo, hi+1)])) for tag in COMPONENTS},
        })
    evaluation = read(EVALUATION / "milestone_evaluation.json")
    policy = []
    for case in evaluation["cases"]:
        if case["checkpoint_iteration"] not in (5, 50, 100):
            continue
        directory = Path(case["source_directory"])
        summary_path, trace_path = directory / "rollout_summary.json", directory / "rollout_policy_trace.json"
        summary, rows = read(summary_path), read(trace_path)
        weights_path = Path(summary["checkpoint"]) / "module_state.pkl"
        source_paths.update((summary_path, trace_path, weights_path))
        with weights_path.open("rb") as stream:
            weights = pickle.load(stream)
        obs = np.asarray([r["actor_observation_vector_before"][:35] for r in rows], dtype=np.float32)
        z = np.tanh(obs @ np.asarray(weights["pi_encoder.0.weight"]).T + np.asarray(weights["pi_encoder.0.bias"]))
        z = np.tanh(z @ np.asarray(weights["pi_encoder.2.weight"]).T + np.asarray(weights["pi_encoder.2.bias"]))
        logits = z @ np.asarray(weights["pi.1.weight"]).T + np.asarray(weights["pi.1.bias"])
        raw = np.asarray([r["raw_policy_action"] for r in rows])
        applied = np.asarray([r["applied_policy_action"] for r in rows])
        replay_error = float(np.max(np.abs(raw-logits[:, :2])))
        assert replay_error < 5e-6, "Offline forward does not reproduce recorded deterministic means"
        sigma = np.exp(logits[:, 2:])
        policy.append({"checkpoint": case["checkpoint_iteration"], "start": case["start"],
                       "sigma_median_normalized": np.median(sigma, axis=0).tolist(),
                       "sigma_min_normalized": sigma.min(axis=0).tolist(),
                       "sigma_max_normalized": sigma.max(axis=0).tolist(),
                       "mean_replay_max_abs_error": replay_error,
                       "clipped_step_fraction": float(np.mean(np.any(np.abs(raw-applied) > 1e-6, axis=1))),
                       "scope": "Existing deterministic states only; no stochastic rollout or new action sampling"})
    summaries = []
    for path in (PLAN / "pilot5_snapshot/summary.json", PLAN / "training_summary_50.json", PLAN / "training_summary_100.json"):
        source_paths.add(path)
        s = read(path)
        summaries.append({k: s[k] for k in ("elapsed_wall_time_s", "iterations_completed_this_process", "restart_count", "crash_restart_count")})
    constructor = RUN / "milestone_iteration_000100/checkpoint_last/class_and_ctor_args.pkl"
    source_paths.add(constructor)
    clipping_audit = {
        "checkpoint100_normalize_actions": serialized_boolean(constructor, "normalize_actions"),
        "checkpoint100_clip_actions": serialized_boolean(constructor, "clip_actions"),
        "recorded_training_clip_fraction_all_zero": all(tb["reward/policy_action_clip_fraction"][i] == 0 for i in range(1, 101)),
        "source_interpretation": "RLlib NormalizeAndClipActions uses unsquash_action with normalize_actions=True; unsquash_action clips Box bounds before env.step. rollout_eval passes raw_action to the reward wrapper, which measures raw-vs-bound excursion before the inner action wrapper.",
        "limit": "Both paths bound the physical action; this establishes a reward/telemetry mismatch, not its causal share in gait failures.",
    }
    OUTPUT.mkdir(parents=True, exist_ok=True)
    result = {
        "scope": "Offline analysis only; campaign remains paused",
        "iteration_zero": "Fresh weights established by pilot summary; no evaluated CP0 rollout exists in this analysis",
        "samples_used": 460800, "samples_per_start": 153600,
        "training_process_wall_time_s": sum(s["elapsed_wall_time_s"] for s in summaries),
        "training_processes": summaries, "blocks": blocks,
        "clipping_audit": clipping_audit,
        "policy_replay": policy,
        "kl_spikes_above_0p1": [{"iteration": r["iteration"], "max_minibatch_mean_kl": r["max_minibatch_mean_kl_loss"], "reported_mean_kl": r["mean_kl_loss"]} for r in history if r["max_minibatch_mean_kl_loss"] > .1],
        "interpretation_limits": [
            "Means of reported training metrics are descriptive; no independent replicate confidence intervals.",
            "TB reward components are averages of rolling 100-step worker windows, not exact full-batch or full-episode decompositions.",
            "Completed training episodes may span policy updates. Resume starts new physical episodes at 6 and 51.",
            "Time-limit completion does not establish valid gait cycles.",
            "Sigma replay uses existing deterministic states, not a matched stochastic validation.",
            "No CP0 or dense deterministic evaluation through all 100 checkpoints; no global best-gait claim.",
        ],
        "source_sha256": {str(p.relative_to(REPO)): digest(p) for p in sorted(source_paths)},
    }
    (OUTPUT / "analysis.json").write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False)+"\n", encoding="utf-8")
    with (OUTPUT / "training_series.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        fields = ("iteration", "samples", "reported_return", "reported_duration_s", "max_minibatch_kl", *COMPONENTS)
        writer.writerow(fields)
        for row in history:
            i = row["iteration"]
            writer.writerow((i, i*4608, row["episode_return_mean"], row["episode_len_mean"]*.01, row["max_minibatch_mean_kl_loss"], *(tb[t][i] for t in COMPONENTS)))
    plt.rcParams.update({"font.size": 10, "axes.spines.top": False, "axes.spines.right": False})
    iterations = np.arange(1, 101)
    fig, axes = plt.subplots(2, 2, figsize=(13, 8))
    for ax, key, label, multiplier in ((axes[0, 0], "episode_return_mean", "Return medio riportato", 1), (axes[0, 1], "episode_len_mean", "Durata media riportata [s]", .01)):
        values = np.asarray([r[key]*multiplier for r in history])
        ax.plot(iterations, values, color="0.65", lw=1, label="Per iterazione")
        ax.plot(iterations, smooth(values), color="#0072B2", lw=2, label="Media mobile 10 iterazioni")
        ax.set(title=label, xlabel="Iterazione")
        ax.legend(fontsize=8)
    axes[0, 1].axhline(5, color="0.4", ls=":", lw=1)
    for tag, label, color in (("reward/served_imitation_score", "Score reference servita", "#0072B2"), ("reward/imitation_score", "Score movimento actual", "#009E73"), ("reward/phase_timeout_penalty_term", "Penalità timeout fase", "#D55E00")):
        values = [tb[tag][i] for i in iterations]
        axes[1, 0].plot(iterations, smooth(values), label=label, color=color, lw=2)
    axes[1, 0].set(title="Componenti TB: media mobile 10 iterazioni", xlabel="Iterazione")
    axes[1, 0].legend(fontsize=8)
    colors = ("#009E73", "#D55E00", "#CC79A7", "#777777")
    labels = ("Limite 5 s", "Penetrazione", "Timeout swing", "Altre protezioni")
    bottom = np.zeros(len(blocks))
    for idx, (label, color) in enumerate(zip(labels, colors)):
        values = []
        for b in blocks:
            ends = b["termination_counts"]
            counts = (ends.get("episode_time_limit", 0), ends.get("grf_penetration", 0)+ends.get("grf_penetration_hard", 0), ends.get("phase_timeout_swing", 0))
            count = counts[idx] if idx < 3 else b["completed_episodes"]-sum(counts)
            values.append(100*count/b["completed_episodes"])
        axes[1, 1].bar(np.arange(len(blocks)), values, bottom=bottom, label=label, color=color)
        bottom += values
    axes[1, 1].set(xticks=np.arange(len(blocks)), xticklabels=[f"{b['first']}–{b['last']}" for b in blocks], ylabel="Episodi completati [%]", title="Terminazioni nel training stocastico")
    axes[1, 1].legend(fontsize=8, ncol=2)
    for ax in axes.flat:
        ax.grid(alpha=.2, axis="y")
    for ax in (axes[0, 0], axes[0, 1], axes[1, 0]):
        for x in (5.5, 50.5): ax.axvline(x, color="0.4", ls="--", alpha=.5)
    fig.suptitle("V1 imitativa: percorso completo 1–100\nLinee tratteggiate: resume; CP0 non valutato. Arrivare a 5 s non implica cammino valido.")
    fig.tight_layout(rect=(0, 0, 1, .94))
    save_figure(fig, "training_1_100")
    fig, axes = plt.subplots(2, 3, figsize=(13, 7))
    checkpoints = (5, 49, 50, 98, 100)
    for column, start in enumerate(STARTS):
        cases = [next(c for c in evaluation["cases"] if c["checkpoint_iteration"]==i and c["start"]==start) for i in checkpoints]
        for row, label in ((0, "Durata [s]"), (1, "Cicli validi")):
            values = [c["full_episode"]["duration_s"] if row==0 else c["full_episode"]["phase"]["valid_cycle_count"] for c in cases]
            ax = axes[row, column]
            ax.bar(range(5), values, color=["#999999", "#E69F00", "#0072B2", "#CC79A7", "#009E73"])
            ax.set(xticks=range(5), xticklabels=[str(i) for i in checkpoints], title=start if row==0 else "", xlabel="Checkpoint", ylabel=label)
            ax.set_ylim(0, 5.8 if row==0 else 3.8)
            for i, value in enumerate(values): ax.text(i, value+.05, f"{value:.2f}" if row==0 else str(int(value)), ha="center", fontsize=9)
            ax.grid(axis="y", alpha=.2)
    fig.suptitle("Valutazioni deterministiche: stesso seed, tre start, massimo 5 s\nNessun checkpoint mostrato supera lo screening complessivo")
    fig.tight_layout(rect=(0, 0, 1, .92))
    save_figure(fig, "deterministic_milestones")
    print(json.dumps({"output_dir": str(OUTPUT), "blocks": len(blocks), "source_files": len(source_paths), "replayed_cases": len(policy), "max_replay_error": max(p["mean_replay_max_abs_error"] for p in policy)}, indent=2))


if __name__ == "__main__":
    main()
