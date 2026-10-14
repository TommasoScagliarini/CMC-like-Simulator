"""Audit the 25->40 continuation, matched sample budgets and deterministic rollouts."""
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
PLAN = PACKAGE / "runs/plans/v26_aligned_left_phase_fsmv3_resume25_to40_2026-09-21"
V26 = TG / "runs/training/MLP_imitation_native_v26_08-20-2026_june_equiv_100iter"
JOINTS = ("pros_knee_angle", "pros_ankle_angle")
ROLLOUTS = {
    "iter25": PACKAGE / "runs/rollout/MLP39_v26_aligned_left_phase_fsmv3_25iter_2026-09-21_best25",
    "best29": PACKAGE / "runs/rollout/MLP39_v26_aligned_left_phase_fsmv3_40iter_2026-09-21_best29",
    "last40": PACKAGE / "runs/rollout/MLP39_v26_aligned_left_phase_fsmv3_40iter_2026-09-21_last40",
    "v26_best87": TG / "runs/rollout/MLP_imitation_native_v26_08-20-2026_june_equiv_100iter_rollout",
}


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


previous = load_module("analysis25", PACKAGE / "validation/v26_aligned_25iter_2026-09-21/analyze.py")


def read(path):
    return json.loads(path.read_text())


def write(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")


def block(rows, low, high):
    values = rows[low - 1:high]
    before = rows[low - 2]["num_env_steps_sampled_lifetime"] if low > 1 else 0
    end = rows[high - 1]["num_env_steps_sampled_lifetime"]
    return {"iterations": [low, high], "sample_budget_start": before,
            "sample_budget_end": end, "window_steps": end - before,
            **previous.endings(rows, low, high),
            "iteration_mean_return": float(np.mean([r["episode_return_mean"] for r in values])),
            "iteration_mean_duration_s": float(np.mean([r["episode_len_mean"] * .01 for r in values]))}


def scalars(path, rows):
    events = EventAccumulator(str(path / "tensorboard"), size_guidance={"scalars": 0})
    events.Reload()
    tags = ["reward_diagnostic/" + j + "_imitation_position_loss" for j in JOINTS]
    tags += ["reward/grf_penetration_loss", "reward/imitation_score", "reward/served_imitation_score"]
    result = {}
    for tag in tags:
        by_step = {v.step: float(v.value) for v in events.Scalars(tag)}
        result[tag] = [{"iteration": r["iteration"], "samples": r["num_env_steps_sampled_lifetime"],
                        "value": by_step[int(r["num_env_steps_sampled_lifetime"])]} for r in rows]
    return result


def training_plot(current, old):
    fig, axes = plt.subplots(2, 2, figsize=(12, 8), layout="constrained")
    for rows, name, color in ((current, "39D, fino alla 40", "#2563eb"),
                              (old[:40], "V26, fino alla 40", "#d97706")):
        budgets = np.asarray([r["num_env_steps_sampled_lifetime"] for r in rows]) / 1000
        axes[0, 0].plot(budgets, [r["episode_return_mean"] for r in rows], label=name, color=color)
        axes[0, 1].plot(budgets, [r["episode_len_mean"] * .01 for r in rows], color=color)
        windows = [block(rows, max(1, i - 4), i) for i in range(1, len(rows) + 1)]
        axes[1, 0].plot(budgets, [100 * w["full_fraction"] for w in windows], color=color)
        axes[1, 1].plot(budgets, [10000 * sum(v for k, v in w["counts"].items() if k.startswith("grf_penetration")) / w["window_steps"] for w in windows], color=color)
    labels = ("Ritorno medio", "Durata media episodio (s)",
              "Episodi completi registrati (%)\nUltime 5 iterazioni", "Terminazioni per penetrazione / 10.000 step\nUltime 5 iterazioni")
    for axis, label in zip(axes.flat, labels):
        axis.set_ylabel(label)
        axis.set_xlabel("Transizioni cumulative (migliaia)")
        axis.axvline(102.4, color="gray", linestyle=":", alpha=.6)
        axis.grid(alpha=.2)
    axes[0, 0].legend()
    axes[0, 1].axhline(5, color="gray", linestyle=":")
    axes[1, 0].set_ylim(0, 105)
    fig.suptitle("39D e V26 per quantità di esperienza — linea verticale: ripresa 39D dalla 25")
    fig.savefig(HERE / "training_by_samples.png", dpi=160)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--training-only", action="store_true")
    args = parser.parse_args()
    rows, old = previous.history(RUN), previous.history(V26)
    summary, supervisor = read(RUN / "summary.json"), read(RUN / "supervisor_state.json")
    assert summary["ok"] and summary["iterations_completed"] == 40
    assert summary["restored_logical_iteration"] == 25 and summary["iterations_completed_this_process"] == 15
    assert summary["initialization_mode"] == "resume_from"
    assert supervisor["restart_count"] == supervisor["crash_restart_count"] == 0
    assert supervisor["skipped_iterations"] == []
    assert [r["iteration"] for r in rows] == list(range(1, 41))
    assert rows[:25] == previous.history(PLAN / "pre_resume_25")
    for path, sha in read(PLAN / "source_hashes.json").items():
        assert hashlib.sha256((REPO / path).read_bytes()).hexdigest() == sha, path
    for row in rows:
        mask = row["unified_phase_audit"]["target_availability_batch"]
        assert mask["total_steps"] == mask["visible_steps"] == 4096
        assert row["num_env_steps_sampled_lifetime"] == row["iteration"] * 4096
        assert row["unified_phase_audit"]["mean_changed"] and row["unified_phase_audit"]["critic_changed"]
        assert all(np.isfinite(row[k]) for k in ("policy_loss", "vf_loss", "mean_kl_loss"))
    resume = read(RUN / "resume_audit.json")
    for restored, source in (("restored_actor_digest", "actor_after"), ("restored_mean_digest", "mean_after"), ("restored_critic_digest", "critic_after")):
        assert resume[restored] == rows[24]["unified_phase_audit"][source]
    for key in ("actor", "mean", "critic"):
        assert rows[25]["unified_phase_audit"][key + "_before"] == rows[24]["unified_phase_audit"][key + "_after"]
    selected = (25, 29, 32, 35, 39, 40)
    simple_rows = [{k: row[k] for k in ("iteration", "episode_return_mean", "episode_len_mean", "num_env_steps_sampled_lifetime", "entropy", "mean_kl_loss")}
                   for row in rows]
    result = {
        "audit": {"passed": True, "source_hashes_unchanged": True, "prefix25_unchanged": True,
                  "resume_digests_match": True, "supervisor": supervisor,
                  "total_visible_samples": 40 * 4096, "continuation_elapsed_s": summary["elapsed_wall_time_s"],
                  "finished_at": summary["finished_at"], "best_iteration": read(RUN / "checkpoint_best_meta.json")["logical_iteration"]},
        "training": {"iterations": simple_rows,
                     "blocks": {f"{a}-{b}": block(rows, a, b) for a, b in ((21, 25), (26, 30), (31, 35), (36, 40), (26, 40))},
                     "tensorboard_smoothed": scalars(RUN, rows)},
        "matched_experience": {},
    }
    for i in selected:
        current = rows[i - 1]
        match = min(old, key=lambda r: abs(r["num_env_steps_sampled_lifetime"] - current["num_env_steps_sampled_lifetime"]))
        j = match["iteration"]
        result["matched_experience"][str(i)] = {"current": block(rows, i - 4, i), "v26": block(old, j - 4, j),
            "v26_endpoint": {k: match[k] for k in ("iteration", "episode_return_mean", "episode_len_mean", "num_env_steps_sampled_lifetime")}}
    result["training"]["health"] = {
        f"{a}-{b}": {key: sum(r["training_health"].get(key, 0) or 0 for r in rows[a - 1:b])
                     for key in ("observed_rows", "resync_event_rows", "missing_telemetry_rows", "phase_timeout_stance_rows",
                                 "morphology_causal_contract_failure_rows", "timeout_side_disagreement_rows")}
        for a, b in ((1, 25), (26, 40), (36, 40))}
    training_plot(rows, old)
    write(HERE / "training_metrics.json", result)
    if args.training_only:
        print(json.dumps({"audit": result["audit"], "blocks": result["training"]["blocks"], "matched40": result["matched_experience"]["40"]}, indent=2))
        return

    helpers = load_module("rollout_helpers", PACKAGE / "validation/ablation_report_2026-09-20/analyze_results.py")
    traces = {}
    result["rollouts"] = {}
    for name, folder in ROLLOUTS.items():
        metrics, trace, s = helpers.rollout_metrics(folder, historical=name == "v26_best87", scale=None if name == "v26_best87" else 1)
        metrics["phase_audit"] = previous.phase_audit(trace)
        active_penalties = [r for r in trace if r["reward_terms"].get("grf_ankle_moment_flip_loss", 0) > 0]
        peak = max(trace, key=lambda r: r["reward_terms"].get("grf_ankle_moment_flip_loss", 0))
        metrics["contact_moment_audit"] = {
            "penalized_steps": len(active_penalties),
            "penalized_fraction": len(active_penalties) / len(trace),
            "first_penalized_time_s": active_penalties[0]["time"] if active_penalties else None,
            "last_penalized_time_s": active_penalties[-1]["time"] if active_penalties else None,
            "peak_time_s": peak["time"],
            "peak_terms": {k: v for k, v in peak["reward_terms"].items() if k.startswith("grf_ankle_moment_flip_")},
        }
        if name in ("best29", "last40"):
            s = {**s, "checkpoint": str((REPO / s["checkpoint"]).resolve())}
            metrics["policy_audit"] = helpers.policy_audit(trace, s, slew=False)
        result["rollouts"][name] = metrics
        traces[name] = trace
    shared = min(len(traces[n]) for n in ("iter25", "best29", "last40"))
    for name in ("best29", "last40"):
        for j in JOINTS:
            np.testing.assert_allclose(helpers.array(traces[name][:shared], "imitation_target_q", j),
                                       helpers.array(traces["iter25"][:shared], "imitation_target_q", j), atol=1e-10)
    result["matched_rollout_prefix"] = {"steps": shared,
        "joints": {name: {j: helpers.joint_metrics(trace[:shared], j) for j in JOINTS} for name, trace in traces.items()}}
    fig, axes = plt.subplots(2, 3, figsize=(16, 7), layout="constrained", sharex=True, sharey="row")
    for col, name in enumerate(("iter25", "best29", "last40")):
        trace = traces[name]
        times = np.array([r["time"] for r in trace]) - trace[0]["policy_segment_times"][0]
        for row, j in enumerate(JOINTS):
            axis = axes[row, col]
            for field, key, label, color, style in (("imitation_target_q", j, "Target sinistro", "black", "--"),
                ("prosthetic_state", j + "_served_ref", "Reference servita", "#d97706", ":"),
                ("prosthetic_state", j, "Giunto", "#2563eb", "-")):
                axis.plot(times, np.rad2deg(helpers.array(trace, field, key)), label=label, color=color, linestyle=style)
            axis.set_title(name + (" — ginocchio" if row == 0 else " — caviglia"))
            axis.set_ylabel("Angolo (°)")
            axis.set_xlabel("Tempo dall'inizio episodio (s)")
            axis.grid(alpha=.2)
    axes[0, 0].legend(fontsize=8)
    fig.suptitle("Rollout deterministici allo stesso start — confronto 25, best29 e last40")
    fig.savefig(HERE / "rollout_comparison.png", dpi=160)
    plt.close(fig)
    fig, axes = plt.subplots(2, 1, figsize=(11, 7), layout="constrained", sharex=True)
    for name, color in (("iter25", "#64748b"), ("best29", "#d97706"), ("last40", "#2563eb")):
        trace = traces[name]
        times = np.asarray([r["time"] for r in trace]) - trace[0]["policy_segment_times"][0]
        axes[0].plot(times, [r["reward_terms"]["grf_ankle_moment_flip_tau_nm"] for r in trace], label=name, color=color)
        axes[1].plot(times, [.1 * r["reward_terms"]["grf_ankle_moment_flip_loss"] for r in trace], label=name, color=color)
    axes[0].axhline(-8, color="black", linestyle="--", linewidth=1, label="Soglia −8 Nm (con forza normale ≥50 N)")
    axes[0].set_ylabel("Momento GRF alla caviglia (Nm)")
    axes[0].legend(fontsize=9)
    axes[1].set_ylabel("Penalità di reward per step")
    axes[1].set_xlabel("Tempo dall'inizio episodio (s)")
    for axis in axes:
        axis.grid(alpha=.2)
    fig.suptitle("Penalità sul segno del momento di contatto — configurazione invariata")
    fig.savefig(HERE / "contact_moment_comparison.png", dpi=160)
    plt.close(fig)
    result["limitations"] = ["One seed and one physical start; not a statistical equivalence test.",
        "Historical intermediate V26 checkpoints unavailable; physical reference is best87.",
        "Historic fatal worker errors can be missing from episode counters.",
        "Resume restores learning state, but physical episodes restart; not a bit-identical uninterrupted run.",
        "Best checkpoint is selected by exploratory training return, not deterministic rollout quality."]
    write(HERE / "metrics.json", result)
    write(HERE / "rollout_input_hashes.json", helpers.INPUT_HASHES)
    print(json.dumps({name: {"duration": r["duration_s"], "return": r["episode_return"], "penetration_mm": 1000 * r["grf_penetration_max_m"],
                            "joints": r["joints"], "phase": r["phase_audit"]} for name, r in result["rollouts"].items()}, indent=2))


if __name__ == "__main__":
    main()
