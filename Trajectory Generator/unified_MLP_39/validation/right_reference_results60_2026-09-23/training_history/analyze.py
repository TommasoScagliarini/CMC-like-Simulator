"""Offline RIGHT39 continuation40->60 audit and matched-experience comparisons.

Null episode metrics are kept missing: console projections never become returns
of completed episodes or best-checkpoint candidates. No simulation or training.
"""
from __future__ import annotations

import csv
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import pickle
from statistics import mean, stdev

HERE = Path(__file__).resolve().parent
PACKAGE = HERE.parents[2]
REPO = PACKAGE.parents[1]
PLOT = REPO / "plot/09_23_2026_right60_comparison"
PLAN = PACKAGE / "runs/plans/right_reference_resume60_2026-09-23"
os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("MPLCONFIGDIR", str(HERE / ".matplotlib"))
spec = importlib.util.spec_from_file_location("right60_history_helpers",
    PACKAGE / "validation/right_reference_results_2026-09-23/training_history/analyze.py")
old = importlib.util.module_from_spec(spec)
spec.loader.exec_module(old)
RUNS, TAGS = old.RUNS, old.TAGS
read, history = old.read, old.history
plt = old.plt
EPISODE_SUM_TAGS = [name + "/offset_1p000000s" for name in
    ("episode_start_return_sum", "episode_start_length_sum", "episode_start_episode_count")]


def tb_scalars(path):
    result = old.tb_scalars(path)
    events = old.EventAccumulator(str(path / "tensorboard"), size_guidance={"scalars": 0})
    events.Reload()
    for tag in EPISODE_SUM_TAGS:
        result[tag] = {int(v.step): float(v.value) for v in events.Scalars(tag)}
    return result


def safe_mean(values):
    values = [v for v in values if v is not None and math.isfinite(v)]
    return mean(values) if values else None


def block(rows, low, high, scalars=None):
    selected = rows[low - 1:high]
    before = old.counts(rows[low - 2]) if low > 1 else {}
    after = old.counts(rows[high - 1])
    endings = {k: after.get(k, 0) - before.get(k, 0) for k in sorted(set(before) | set(after))}
    assert all(v >= 0 for v in endings.values()), (low, high, endings)
    n = sum(endings.values())
    start = int(rows[low - 2]["num_env_steps_sampled_lifetime"]) if low > 1 else 0
    end = int(rows[high - 1]["num_env_steps_sampled_lifetime"])
    result = {"iterations": [low, high], "sample_budget_start": start, "sample_budget_end": end,
        "window_steps": end - start, "endings": endings, "recorded_episodes": n,
        "complete_episodes": endings.get("episode_time_limit", 0),
        "completion_fraction": endings.get("episode_time_limit", 0) / n if n else None,
        "penetration_endings": sum(v for k, v in endings.items() if k.startswith("grf_penetration")),
        "available_episode_metric_iterations": sum(r["episode_return_mean"] is not None for r in selected),
        "missing_episode_metric_iterations": [r["iteration"] for r in selected if r["episode_return_mean"] is None]}
    for k in ("episode_return_mean", "episode_len_mean", "entropy", "policy_loss", "vf_loss", "mean_kl_loss"):
        result["mean_" + k] = safe_mean(r[k] for r in selected)
    returns = [r["episode_return_mean"] for r in selected if r["episode_return_mean"] is not None]
    result["episode_return_between_iteration_sample_std"] = stdev(returns) if len(returns) > 1 else None
    result["mean_vf_explained_variance"] = safe_mean(
        r["learner_metrics"]["learners/default_policy/vf_explained_var"] for r in selected)
    result["entropy_equivalent_geometric_sigma_proxy"] = math.exp((result["mean_entropy"] - math.log(2 * math.pi * math.e)) / 2)
    if scalars:
        result["tensorboard_smoothed_means"] = {
            tag: mean(scalars[tag][int(r["num_env_steps_sampled_lifetime"])] for r in selected) for tag in TAGS}
        sums = {tag: sum(scalars[tag].get(int(r["num_env_steps_sampled_lifetime"]), 0) for r in selected)
                for tag in EPISODE_SUM_TAGS}
        count = sums[EPISODE_SUM_TAGS[2]]
        assert count == n, (low, high, count, n)
        weighted_native_return = weighted_native_len = 0.0
        for row in selected:
            i = row["iteration"]
            previous = old.counts(rows[i - 2]) if i > 1 else {}
            current = old.counts(row)
            ended = sum(current.values()) - sum(previous.values())
            if ended:
                assert row["episode_return_mean"] is not None and row["episode_len_mean"] is not None
                weighted_native_return += row["episode_return_mean"] * ended
                weighted_native_len += row["episode_len_mean"] * ended
        # TensorBoard scalar encoding is float32; the JSONL native means have
        # more precision. Keep both and verify the actual episode callback sums.
        result["episode_weighted_return_from_native_iteration_means"] = weighted_native_return / n if n else None
        result["episode_weighted_length_from_native_iteration_means"] = weighted_native_len / n if n else None
        result["episode_weighted_return_from_callback_sums"] = sums[EPISODE_SUM_TAGS[0]] / n if n else None
        result["episode_weighted_length_from_callback_sums"] = sums[EPISODE_SUM_TAGS[1]] / n if n else None
        result["callback_completed_episode_count"] = count
        result["weighted_native_vs_callback_return_abs_delta"] = abs(weighted_native_return - sums[EPISODE_SUM_TAGS[0]]) / n if n else None
        result["weighted_native_vs_callback_length_abs_delta"] = abs(weighted_native_len - sums[EPISODE_SUM_TAGS[1]]) / n if n else None
        # Early historic oversampling and smoothed worker metrics may not share
        # the callback denominator exactly; preserve discrepancies explicitly.
        result["callback_precision_note"] = "Episode callback sums read from TensorBoard float32; authoritative episode-weighted aggregate. Native weighted means retained as cross-check, not substituted if different."
    return result


def endpoint(rows, high):
    best = max((r for r in rows[:high] if r["episode_return_mean"] is not None), key=lambda r: r["episode_return_mean"])
    result = {"last": old.simple(rows[high - 1]), "best_so_far": old.simple(best),
        "cumulative_minibatch_passes": rows[high - 1]["learner_metrics"]["learners/default_policy/num_module_steps_trained_lifetime"] / 512}
    return result


def integrity(rows):
    run = RUNS["right39"]
    summary = read(run / "summary.json")
    status = read(PLAN / "status.json")
    receipt = read(PLAN / "freeze_receipt.json")
    assert receipt["manifest_sha256"] == hashlib.sha256((PLAN / "source_hashes.json").read_bytes()).hexdigest()
    assert receipt["checkpoint_manifest_sha256"] == hashlib.sha256((PLAN / "checkpoint40_hashes.json").read_bytes()).hexdigest()
    verified = {}
    for filename in ("source_hashes.json", "checkpoint40_hashes.json", "archive_hashes.json", "checkpoint60_hashes.json"):
        values = read(PLAN / filename)
        assert all(hashlib.sha256((REPO / p).read_bytes()).hexdigest() == sha for p, sha in values.items())
        verified[filename] = len(values)
    archived = (PLAN / "pre_resume_40/train_iterations.jsonl").read_bytes()
    assert (run / "train_iterations.jsonl").read_bytes().startswith(archived)
    assert history(PLAN / "pre_resume_40") == rows[:40]
    assert summary["ok"] and summary["stop_reason"] == "completed" and summary["iterations_completed"] == 60
    assert summary["restored_logical_iteration"] == 40 and summary["iterations_completed_this_process"] == 20
    assert summary["restart_count"] == summary["crash_restart_count"] == 0 and summary["skipped_iterations"] == []
    assert status["state"] == "completed" and status["returncode"] == 0
    assert [r["iteration"] for r in rows] == list(range(1, 61))
    restore = read(PLAN / "resume_verified.json")
    assert restore["passed"] and restore["weights"]["actor_reinitialized"] is False
    for component in ("actor", "mean", "critic"):
        assert restore["weights"]["restored_" + component + "_digest"] == rows[39]["unified_phase_audit"][component + "_after"] == rows[40]["unified_phase_audit"][component + "_before"]
    optimizer = restore["optimizer"]
    assert optimizer["passed"] and optimizer["ppo_training_state"]["passed"]
    assert optimizer["ppo_training_state"]["source"] == "checkpoint_sidecar"
    for learner in optimizer["learners"]:
        assert learner["passed"] and learner["parameters_unchanged"]
        for item in learner["optimizers"]:
            assert item["state_exact"] and item["state_count"] == item["linked_state_count"] == 12
            assert item["source_digest"] == item["restored_digest"]
    progress = read(PLAN / "optimizer_progress.json")
    assert progress["passed"]
    checkpoint_steps = {}
    for iteration, expected in ((40, 3240), (41, 3321), (60, 4860)):
        path = run / f"milestone_iteration_{iteration:06d}/checkpoint_last/learner_group/learner/state.pkl"
        state = pickle.loads(path.read_bytes())["optimizer"]
        steps = {float(v["step"]) for payload in state.values() for v in payload["state"]["state"].values()}
        assert steps == {expected}
        checkpoint_steps[str(iteration)] = sorted(steps)
    for i, row in enumerate(rows, 1):
        batch = row["unified_phase_audit"]["target_availability_batch"]
        assert batch["total_steps"] == batch["visible_steps"] == 4096
        assert row["num_env_steps_sampled_lifetime"] == i * 4096
        assert row["unified_phase_audit"]["mean_changed"] and row["unified_phase_audit"]["critic_changed"]
        assert all(math.isfinite(row[k]) for k in ("policy_loss", "vf_loss", "entropy", "mean_kl_loss"))
        one = block(rows, i, i)
        if row["episode_return_mean"] is None:
            assert one["recorded_episodes"] == 0 and row["console_episode_metrics"]["estimated"]
    return {"passed": True, "verified_manifests": verified, "prefix40_unchanged": True,
        "resume_actor_mean_critic_exact": True, "adam_steps": checkpoint_steps,
        "ppo_state_restored_exactly": True, "resume_verified": restore,
        "started_at": summary["started_at"], "finished_at": summary["finished_at"],
        "elapsed_wall_time_s": summary["elapsed_wall_time_s"], "restart_count": 0,
        "missing_episode_metrics": [r["iteration"] for r in rows if r["episode_return_mean"] is None],
        "total_visible_steps": 245760,
        "health41to60": {k: sum(r["training_health"][k] for r in rows[40:])
                          for k in ("observed_rows", "missing_telemetry_rows", "resync_event_rows", "phase_timeout_stance_rows")}}


def plots(histories, scalars):
    colors = {"right39": "#2563eb", "left39": "#d97706", "v26": "#15956d"}
    labels = {"right39": "39D RIGHT", "left39": "39D LEFT", "v26": "V26 storico"}
    fig, axes = plt.subplots(3, 2, figsize=(13, 11), layout="constrained")
    fig2, axes2 = plt.subplots(2, 3, figsize=(16, 8), layout="constrained")
    for name, rows in histories.items():
        rows = rows[:60 if name != "v26" else 53]
        x = [r["num_env_steps_sampled_lifetime"] / 1000 for r in rows]
        wins = [block(rows, max(1, i - 4), i) for i in range(1, len(rows) + 1)]
        args = {"color": colors[name], "label": labels[name]}
        axes[0, 0].plot(x, [r["episode_return_mean"] if r["episode_return_mean"] is not None else float("nan") for r in rows], **args)
        axes[0, 1].plot(x, [r["episode_len_mean"] * .01 if r["episode_len_mean"] is not None else float("nan") for r in rows], **args)
        axes[1, 0].plot(x, [w["completion_fraction"] * 100 for w in wins], **args)
        axes[1, 1].plot(x, [w["penetration_endings"] * 10000 / w["window_steps"] for w in wins], **args)
        axes[2, 0].plot(x, [r["entropy"] for r in rows], **args)
        axes[2, 1].plot(x, [r["learner_metrics"]["learners/default_policy/vf_explained_var"] for r in rows], **args)
        for ax, tag in zip(axes2.flat, (TAGS[0], TAGS[1], "reward/imitation_score", "reward/grf_penetration_loss", "reward/grf_ankle_moment_flip_loss", "reward/command_rate_loss")):
            ax.plot(x, [scalars[name][tag][int(r["num_env_steps_sampled_lifetime"])] for r in rows], **args)
    titles = ["Return medio (solo episodi conclusi)", "Durata media episodio [s]", "Completi ultimi5iter [%]",
              "Penetrazioni ultime5 /10.000step", "Entropia", "Varianza spiegata critic"]
    for ax, title in zip(axes.flat, titles):
        ax.set_ylabel(title)
        ax.set_xlabel("Transizioni cumulative [migliaia]")
        ax.grid(alpha=.2)
        ax.axvline(163.84, color="gray", ls="--", alpha=.5)
    axes[1, 0].set_ylim(0, 105)
    axes[0, 0].legend()
    for ax, title in zip(axes2.flat, ("Loss posizione ginocchio normalizzata", "Loss posizione caviglia normalizzata", "Score imitazione effettiva", "Loss penetrazione", "Loss inversione momento caviglia", "Loss variazione comandi")):
        ax.set_ylabel(title)
        ax.set_xlabel("Transizioni cumulative [migliaia]")
        ax.grid(alpha=.2)
        ax.axvline(163.84, color="gray", ls="--", alpha=.5)
    axes2[0, 0].legend()
    fig.suptitle("RIGHT fino60, LEFT60, V26 fino53 — tratteggio: ripresa RIGHT40→60\nI vuoti del return indicano batch senza episodi conclusi, non dati persi")
    fig2.suptitle("Loss medie mobili TensorBoard — LEFT usa target e normalizzazioni diversi")
    fig.savefig(PLOT / "training_by_experience.png", dpi=170)
    fig2.savefig(PLOT / "imitation_and_contact_losses.png", dpi=170)
    plt.close("all")


def main():
    HERE.mkdir(parents=True, exist_ok=True)
    PLOT.mkdir(parents=True, exist_ok=True)
    histories = {name: history(path) for name, path in RUNS.items()}
    scalars = {name: tb_scalars(path) for name, path in RUNS.items()}
    rows = histories["right39"]
    result = {"audit": integrity(rows), "right": {}, "comparisons": {},
        "method": {"counts": "Differences of cumulative ending counters, including across the resume40 boundary.",
            "returns": "Report both arithmetic means of available iteration metrics and episode-weighted callback sum/count aggregates. Nulls and console projections are excluded.",
            "entropy_sigma_proxy": "exp((H-log(2*pi*e))/2); geometric proxy across states/action dimensions, not jointwise std.",
            "jointwise_training_std": "Not serialized as a training scalar. Requires separate checkpoint/trajectory audit.",
            "window_episodes": "Training episodes may span policy updates and iteration boundaries; not independent frozen-policy evaluations."},
        "limitations": ["One training seed per condition and one fixed start; no statistical equivalence claim.",
            "LEFT25->40 had an Adam restore defect; LEFT40->60 had a telemetry interruption after42. RIGHT40->60 restored Adam/PPO correctly without interruption.",
            "Reference intervention changes waveform, clock and range-derived normalization jointly.",
            "Historical V26 uses earlier FSM/failure handling and semantic input order; some fatal worker failures may be absent from historical ending counters.",
            "Completion does not certify cycle validity, kinematics, torques or zero penetration.",
            "Resume preserves learning state but resets physical episodes/RNG continuity; synchronized workers explain empty-completion batches."]}
    for low, high in ((36, 40), (41, 45), (46, 50), (51, 55), (56, 60), (41, 50), (51, 60), (41, 60)):
        result["right"][f"{low}-{high}"] = block(rows, low, high, scalars["right39"])
    result["right_endpoints"] = {str(i): endpoint(rows, i) for i in (40, 50, 58, 60)}
    for high in (40, 60):
        target = high * 4096
        match = min(range(1, len(histories["v26"]) + 1), key=lambda i: abs(histories["v26"][i - 1]["num_env_steps_sampled_lifetime"] - target))
        result["comparisons"][str(high)] = {}
        for name, end in (("right39", high), ("left39", high), ("v26", match)):
            result["comparisons"][str(high)][name] = {
                "endpoint": endpoint(histories[name], end),
                "last5": block(histories[name], end - 4, end, scalars[name]),
                "last10": block(histories[name], end - 9, end, scalars[name]),
                "last20": block(histories[name], end - 19, end, scalars[name])}
    flat = []
    for name, history_rows in histories.items():
        for i, row in enumerate(history_rows[:60], 1):
            single = block(history_rows, i, i)
            rolling = block(history_rows, max(1, i - 4), i)
            flat.append({"run": name, **old.simple(row), "ended_this_iteration": single["recorded_episodes"],
                "complete_this_iteration": single["complete_episodes"], "penetration_endings_this_iteration": single["penetration_endings"],
                "completion_last5": rolling["completion_fraction"],
                "episode_metric_available": row["episode_return_mean"] is not None,
                "console_metric_estimated": row.get("console_episode_metrics", {}).get("estimated", False)})
    with (HERE / "iterations.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=flat[0])
        writer.writeheader()
        writer.writerows(flat)
    old.write_json(HERE / "metrics.json", result)
    old.write_json(HERE / "tensorboard_selected.json", scalars)
    old.write_json(HERE / "input_hashes.json", old.INPUT_HASHES)
    plots(histories, scalars)
    print(json.dumps({"best": result["right_endpoints"]["60"]["best_so_far"],
                     "blocks": result["right"], "matched60": result["comparisons"]["60"]}, indent=2))


if __name__ == "__main__":
    main()
