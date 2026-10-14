"""Offline comparison of fresh RIGHT39, aligned LEFT39 and historical V26.

Reads immutable training artifacts; no policy, environment or checkpoint writes.
Episode counts are differences of cumulative counters, never sums of counters.
"""
from __future__ import annotations

import csv
import hashlib
import json
import math
import os
from pathlib import Path
from statistics import mean
import yaml

HERE = Path(__file__).resolve().parent
PACKAGE = HERE.parents[2]
TG = PACKAGE.parent
REPO = TG.parent
PLOT = REPO / "plot/09_23_2026_right_reference_comparison"
os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("MPLCONFIGDIR", str(HERE / ".matplotlib"))
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from tensorboard.backend.event_processing.event_accumulator import EventAccumulator

RUNS = {
    "right39": PACKAGE / "runs/training/MLP39_v26_aligned_right_phase_fsmv3_40iter_2026-09-23",
    "left39": PACKAGE / "runs/training/MLP39_v26_aligned_left_phase_fsmv3_25iter_2026-09-21",
    "v26": TG / "runs/training/MLP_imitation_native_v26_08-20-2026_june_equiv_100iter",
}
PLAN = PACKAGE / "runs/plans/right_reference_training_2026-09-23"
TAGS = [
    "reward_diagnostic/pros_knee_angle_imitation_position_loss",
    "reward_diagnostic/pros_ankle_angle_imitation_position_loss",
    "reward_diagnostic/pros_knee_angle_imitation_velocity_loss",
    "reward_diagnostic/pros_ankle_angle_imitation_velocity_loss",
    "reward/imitation_score", "reward/served_imitation_score",
    "reward/grf_penetration_loss", "reward/grf_ankle_moment_flip_loss",
    "reward/command_rate_loss",
]
INPUT_HASHES = {}


def read(path):
    data = path.read_bytes()
    INPUT_HASHES[str(path.relative_to(REPO))] = hashlib.sha256(data).hexdigest()
    return json.loads(data)


def history(path):
    file = path / "train_iterations.jsonl"
    data = file.read_bytes()
    INPUT_HASHES[str(file.relative_to(REPO))] = hashlib.sha256(data).hexdigest()
    return [json.loads(line) for line in data.splitlines() if line.strip()]


def counts(row):
    return {k.split("/episode_end/")[-1]: int(v) for k, v in row.get("termination_metrics", {}).items()
            if "/episode_end/" in k}


def block(rows, low, high, scalars=None):
    selected = rows[low - 1:high]
    before = counts(rows[low - 2]) if low > 1 else {}
    after = counts(rows[high - 1])
    endings = {k: after.get(k, 0) - before.get(k, 0) for k in sorted(set(before) | set(after))}
    assert all(v >= 0 for v in endings.values()), (low, high, endings)
    n = sum(endings.values())
    start = int(rows[low - 2]["num_env_steps_sampled_lifetime"]) if low > 1 else 0
    end = int(rows[high - 1]["num_env_steps_sampled_lifetime"])
    result = {
        "iterations": [low, high], "sample_budget_start": start, "sample_budget_end": end,
        "window_steps": end - start, "endings": endings, "recorded_episodes": n,
        "complete_episodes": endings.get("episode_time_limit", 0),
        "full_fraction": endings.get("episode_time_limit", 0) / n if n else None,
        "penetration_endings": sum(v for k, v in endings.items() if k.startswith("grf_penetration")),
        "swing_timeouts": endings.get("phase_timeout_swing", 0),
        "mean_iteration_duration_s": mean(r["episode_len_mean"] * .01 for r in selected),
    }
    for key in ("episode_return_mean", "entropy", "policy_loss", "vf_loss", "mean_kl_loss"):
        result[key if key.startswith("mean_") else "mean_" + key] = mean(r[key] for r in selected)
    result["mean_vf_explained_variance"] = mean(
        r["learner_metrics"]["learners/default_policy/vf_explained_var"] for r in selected)
    # Proxy only: a geometric mean over states and two action dimensions, not a jointwise standard deviation.
    result["entropy_equivalent_geometric_sigma_proxy"] = math.exp(
        (result["mean_entropy"] - math.log(2 * math.pi * math.e)) / 2)
    if scalars:
        result["tensorboard_smoothed_means"] = {
            tag: mean(scalars[tag][int(r["num_env_steps_sampled_lifetime"])] for r in selected)
            for tag in TAGS}
    return result


def tb_scalars(path):
    folder = path / "tensorboard"
    for file in folder.glob("events.out.tfevents.*"):
        INPUT_HASHES[str(file.relative_to(REPO))] = hashlib.sha256(file.read_bytes()).hexdigest()
    events = EventAccumulator(str(folder), size_guidance={"scalars": 0})
    events.Reload()
    return {tag: {int(e.step): float(e.value) for e in events.Scalars(tag)} for tag in TAGS}


def simple(row):
    return {k: row[k] for k in ("iteration", "episode_return_mean", "episode_len_mean",
                                "num_env_steps_sampled_lifetime", "entropy", "vf_loss", "mean_kl_loss")}


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")


def config_audit(histories):
    paths = {
        "left25": RUNS["left39"] / "milestone_iteration_000025/checkpoint_last/training_cfg.resolved.yaml",
        "left40": RUNS["left39"] / "milestone_iteration_000040/checkpoint_last/training_cfg.resolved.yaml",
        "right40": RUNS["right39"] / "checkpoint_last/training_cfg.resolved.yaml",
    }
    configs = {}
    for name, path in paths.items():
        data = path.read_bytes()
        INPUT_HASHES[str(path.relative_to(REPO))] = hashlib.sha256(data).hexdigest()
        configs[name] = yaml.safe_load(data)

    def diff(left, right, path=""):
        if isinstance(left, dict) and isinstance(right, dict):
            result = []
            for key in sorted(set(left) | set(right)):
                if key == "provenance":
                    continue
                result += diff(left.get(key, "<absent>"), right.get(key, "<absent>"),
                               path + "." + key if path else key)
            return result
        return [{"path": path, "left": left, "right": right}] if left != right else []

    compared = diff(configs["left25"], configs["right40"])
    assert {row["path"] for row in compared} == {"simulation.iterations", "target_availability.target_reference"}
    pre_resume = PACKAGE / "runs/plans/v26_aligned_left_phase_fsmv3_resume25_to40_2026-09-21/pre_resume_25"
    archived_rows = history(pre_resume)
    assert archived_rows == histories["left39"][:25]
    left_manifest = read(PACKAGE / "runs/plans/v26_aligned_left_phase_fsmv3_25iter_2026-09-21/source_hashes.json")
    right_manifest = read(PLAN / "source_hashes.json")
    shared = sorted(set(left_manifest) & set(right_manifest))
    return {
        "resolved_configs": {k: str(v.relative_to(REPO)) for k, v in paths.items()},
        "left25_vs_right40_differences_excluding_provenance": compared,
        "left25_vs_left40_differences_excluding_provenance": diff(configs["left25"], configs["left40"]),
        "left_first25_equal_archived_pre_resume_history": True,
        "shared_source_manifest_entries": len(shared),
        "shared_source_entries_changed": [f for f in shared if left_manifest[f] != right_manifest[f]],
        "source_caveat": "The LEFT manifest is small; this does not assert byte-identical full runtimes. Native RIGHT support and diagnostic telemetry were added later. 107 regression/preflight tests are archived in the startup report.",
    }


def plots(histories, result, scalars):
    colors = {"right39": "#2563eb", "left39": "#d97706", "v26": "#15956d"}
    labels = {"right39": "39D RIGHT da zero", "left39": "39D LEFT", "v26": "V26 storico"}
    fig, axes = plt.subplots(2, 2, figsize=(12, 8), layout="constrained")
    fig2, axes2 = plt.subplots(2, 2, figsize=(12, 8), layout="constrained")
    for name, rows in histories.items():
        rows = rows[:40]
        x = [r["num_env_steps_sampled_lifetime"] / 1000 for r in rows]
        rolling = [block(rows, max(1, i - 4), i) for i in range(1, len(rows) + 1)]
        kwargs = dict(color=colors[name], label=labels[name])
        axes[0, 0].plot(x, [r["episode_return_mean"] for r in rows], **kwargs)
        axes[0, 1].plot(x, [r["episode_len_mean"] * .01 for r in rows], **kwargs)
        axes[1, 0].plot(x, [100 * r["full_fraction"] for r in rolling], **kwargs)
        axes[1, 1].plot(x, [r["penetration_endings"] * 10000 / r["window_steps"] for r in rolling], **kwargs)
        axes2[0, 0].plot(x, [r["entropy"] for r in rows], **kwargs)
        axes2[0, 1].plot(x, [r["learner_metrics"]["learners/default_policy/vf_explained_var"] for r in rows], **kwargs)
        for joint, linestyle in (("knee", "-"), ("ankle", "--")):
            tag = f"reward_diagnostic/pros_{joint}_angle_imitation_position_loss"
            axes2[1, 0].semilogy(x, [scalars[name][tag][int(r["num_env_steps_sampled_lifetime"])] for r in rows],
                                color=colors[name], linestyle=linestyle, label=labels[name] + " " + joint)
        tag = "reward/grf_ankle_moment_flip_loss"
        axes2[1, 1].plot(x, [scalars[name][tag][int(r["num_env_steps_sampled_lifetime"])] for r in rows], **kwargs)
    ylabels = ["Return medio di training", "Durata media episodio (s)",
               "Episodi completi (%)\nUltime 5 iterazioni", "Terminazioni per penetrazione / 10.000 step\nUltime 5 iterazioni"]
    ylabels2 = ["Entropia gaussiana delle azioni", "Varianza spiegata dal critic",
                "Loss posizione normalizzata\nMedia mobile TensorBoard", "Loss momento GRF ankle\nMedia mobile TensorBoard"]
    for panels, labels_y in ((axes, ylabels), (axes2, ylabels2)):
        for ax, label in zip(panels.flat, labels_y):
            ax.set_ylabel(label)
            ax.set_xlabel("Transizioni cumulative (migliaia)")
            ax.grid(alpha=.2)
            ax.axvline(102.4, color="gray", linestyle=":", alpha=.6)
            ax.axvline(163.84, color="gray", linestyle="--", alpha=.4)
    axes[0, 0].legend(fontsize=9)
    axes[1, 0].set_ylim(0, 105)
    axes[0, 1].axhline(5, color="gray", linestyle=":")
    axes2[0, 0].legend(fontsize=9)
    axes2[1, 0].legend(fontsize=7, ncol=2)
    fig.suptitle("Apprendimento a pari esperienza — linea puntinata: ripresa LEFT dalla 25")
    fig2.suptitle("Esplorazione e loss — target/normalizzazioni LEFT e RIGHT differenti")
    fig.savefig(PLOT / "training_by_experience.png", dpi=180)
    fig2.savefig(PLOT / "learner_and_losses.png", dpi=180)
    plt.close("all")


def main():
    HERE.mkdir(parents=True, exist_ok=True)
    PLOT.mkdir(parents=True, exist_ok=True)
    all_histories = {name: history(path) for name, path in RUNS.items()}
    histories = {name: rows[:40] for name, rows in all_histories.items()}
    scalars = {name: tb_scalars(path) for name, path in RUNS.items()}
    summary = read(RUNS["right39"] / "summary.json")
    status = read(PLAN / "status.json")
    hashes = read(PLAN / "source_hashes.json")
    mismatches = [f for f, sha in hashes.items() if hashlib.sha256((REPO / f).read_bytes()).hexdigest() != sha]
    assert not mismatches
    assert summary["ok"] and summary["iterations_completed"] == 40
    assert status["state"] == "completed" and status["returncode"] == 0
    assert summary["initialization_mode"] == "fresh" and summary["resume_from"] is None
    assert summary["crash_restart_count"] == summary["restart_count"] == 0
    assert summary["skipped_iterations"] == []
    for name, rows in histories.items():
        assert [r["iteration"] for r in rows] == list(range(1, 41))
        for i, row in enumerate(rows, 1):
            block(rows, i, i)  # monotonic episode counters, including LEFT resume
            assert all(math.isfinite(row[k]) for k in ("policy_loss", "vf_loss", "entropy", "mean_kl_loss"))
            if name != "v26":
                mask = row["unified_phase_audit"]["target_availability_batch"]
                assert mask["total_steps"] == mask["visible_steps"] == 4096
                assert row["num_env_steps_sampled_lifetime"] == i * 4096
                assert row["unified_phase_audit"]["mean_changed"] and row["unified_phase_audit"]["critic_changed"]
    initial_matches = {component: histories["right39"][0]["unified_phase_audit"][component + "_before"] ==
                        histories["left39"][0]["unified_phase_audit"][component + "_before"]
                       for component in ("actor", "mean", "critic")}
    assert all(initial_matches.values())
    result = {
        "scope": "RIGHT fresh iterations1-40; LEFT first40 of eventual100; V26 historical100, primary endpoint33",
        "audit": {"passed": True, "source_hashes_unchanged_count": len(hashes),
                  "right_left_initial_digest_matches": initial_matches,
                  "started_at": summary["started_at"], "finished_at": summary["finished_at"],
                  "elapsed_wall_time_s": summary["elapsed_wall_time_s"],
                  "restart_count": 0, "skipped_iterations": []},
        "runs": {}, "matched_experience": {}, "configuration_audit": config_audit(histories),
        "interpretation_limits": [
            "One training seed per condition; not a statistical equivalence test.",
            "RIGHT changes target waveform, prescribed clock and range-derived normalization together; effects are not separated.",
            "LEFT was resumed after25; RIGHT was uninterrupted. A robustness gap already exists before that resume.",
            "Historical V26 uses FSMv2, earlier failure handling, oversampled batches and another semantic input order.",
            "Historical fatal worker errors can be missing from recorded episode endings.",
            "Completion means reaching500steps, not zero penetration or good torque/kinematics.",
            "Returns use different LEFT vsRIGHT targets; lower return does not imply lower completion.",
            "Return/duration are means of per-iteration metrics; completion denominators are ending-counter deltas.",
            "TensorBoard reward scalars are moving averages over visited training states, not deterministic rollout metrics.",
            "Entropy cannot identify jointwise sigma or prove it caused robustness; geometric sigma is only an entropy proxy.",
            "Historical best87 kinematics use substantially more experience than newbest35; no intermediate V26 checkpoints remain.",
        ]}
    csv_rows = []
    for name, rows in histories.items():
        blocks = {f"{low}-{low+4}": block(rows, low, low + 4, scalars[name]) for low in range(1, 37, 5)}
        best = max(rows, key=lambda r: r["episode_return_mean"])
        thresholds = {}
        for threshold in (.5, .8, .9):
            eligible = [i for i in range(5, len(rows) + 1) if block(rows, i - 4, i)["full_fraction"] >= threshold]
            first = eligible[0] if eligible else None
            thresholds[str(threshold)] = {"first_iteration": first,
                "samples": int(rows[first - 1]["num_env_steps_sampled_lifetime"]) if first else None,
                "window": block(rows, first - 4, first) if first else None,
                "all_later_windows_above": all(i in eligible for i in range(first, len(rows) + 1)) if first else False}
        result["runs"][name] = {"path": str(RUNS[name].relative_to(REPO)), "blocks": blocks,
            "first40_best": simple(best), "iteration40": simple(rows[-1]),
            "completion_threshold_crossings": thresholds, "overall1to40": block(rows, 1, 40),
            "ending_counters_monotonic": True,
            "minibatch_passes_at_iteration40": rows[-1]["learner_metrics"]["learners/default_policy/num_module_steps_trained_lifetime"] / 512,
            "health_totals": {k: (sum(r["training_health"][k] for r in rows)
                                   if all(k in r.get("training_health", {}) for r in rows) else None)
                              for k in ("missing_telemetry_rows", "observed_rows", "resync_event_rows", "phase_timeout_stance_rows")}}
        for i, row in enumerate(rows, 1):
            one = block(rows, i, i)
            window = block(rows, max(1, i - 4), i)
            csv_rows.append({"run": name, **simple(row), "complete_episodes_this_iteration": one["complete_episodes"],
                "ended_episodes_this_iteration": one["recorded_episodes"],
                "penetration_endings_this_iteration": one["penetration_endings"],
                "completion_last5_fraction": window["full_fraction"],
                "completion_last5_count": window["complete_episodes"], "endings_last5_count": window["recorded_episodes"]})
    for endpoint in (15, 20, 25, 30, 35, 40):
        budget = endpoint * 4096
        vi = min(range(len(all_histories["v26"])), key=lambda i: abs(all_histories["v26"][i]["num_env_steps_sampled_lifetime"] - budget)) + 1
        result["matched_experience"][str(endpoint)] = {
            "right39": block(histories["right39"], endpoint - 4, endpoint, scalars["right39"]),
            "left39": block(histories["left39"], endpoint - 4, endpoint, scalars["left39"]),
            "v26": block(all_histories["v26"], vi - 4, vi, scalars["v26"]),
            "v26_best_up_to_endpoint": simple(max(all_histories["v26"][:vi], key=lambda r: r["episode_return_mean"])),
            "v26_minibatch_passes": all_histories["v26"][vi - 1]["learner_metrics"]["learners/default_policy/num_module_steps_trained_lifetime"] / 512,
        }
    with (HERE / "iterations.csv").open("w", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=csv_rows[0])
        writer.writeheader()
        writer.writerows(csv_rows)
    write_json(HERE / "metrics.json", result)
    write_json(HERE / "tensorboard_selected.json", scalars)
    write_json(HERE / "input_hashes.json", INPUT_HASHES)
    plots(histories, result, scalars)
    print(json.dumps({"audit": result["audit"], "matched40": result["matched_experience"]["40"],
                      "matched25": result["matched_experience"]["25"]}, indent=2))


if __name__ == "__main__":
    main()
