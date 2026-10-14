"""Independent offline metrics supplement; preserves the first audit files."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
import statistics


OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[3]
RUN = ROOT / "Trajectory Generator/runs/training/MLP_V26C_J20_sigma0p010_morph_equal5p0_ppo50_2026-09-01"


def stat(series):
    values = [v for _, v in series]
    return {"n": len(values), "min": min(values), "max": max(values), "mean": statistics.mean(values),
            "median": statistics.median(values), "first": {"iteration": series[0][0], "value": series[0][1]},
            "last": {"iteration": series[-1][0], "value": series[-1][1]},
            "first10_observations_mean": statistics.mean(values[:10]),
            "last10_observations_mean": statistics.mean(values[-10:])}


def digest(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    preserved = {p.name: digest(p) for p in OUT.iterdir() if p.is_file() and p.name in {
        "distribution_audit.md", "distribution_summary.json", "per_iteration_offsets.csv", "current_ik_reset_q_qdot.csv", "audit_distribution.py"}}
    summary = json.loads((RUN / "summary.json").read_text())
    rows = [json.loads(l) for l in (RUN / "train_iterations.jsonl").read_text().splitlines() if l.strip()]
    assert len(rows) == 50 and [r["iteration"] for r in rows] == list(range(2, 52))
    labels = list(rows[0]["per_start_metrics"])
    by_start = {}
    for label in labels:
        items = [(r["iteration"], r["per_start_metrics"][label]) for r in rows]
        returns = [(i, x["episode_return_mean"]) for i, x in items if x["completed_episodes"] > 0]
        advantages = [(i, x["advantage_mean"]) for i, x in items]
        return_stats = stat(returns)
        return_stats.update({
            "completed_episode_count": int(sum(x["completed_episodes"] for _, x in items)),
            "weighted_completed_episode_mean": sum(x["episode_return_sum"] or 0 for _, x in items) / sum(x["completed_episodes"] for _, x in items),
            "first10_updates_mean_of_completed_groups": statistics.mean(v for i, v in returns if i <= 11),
            "last10_updates_mean_of_completed_groups": statistics.mean(v for i, v in returns if i >= 42),
            "missing_groups_are_ignored_not_zero_imputed": True,
        })
        return_stats["last10_minus_first10_update_groups"] = return_stats["last10_updates_mean_of_completed_groups"] - return_stats["first10_updates_mean_of_completed_groups"]
        adv_stats = stat(advantages)
        adv_stats.update({"positive_mean_rows": sum(v > 0 for _, v in advantages),
                          "negative_mean_rows": sum(v < 0 for _, v in advantages), "zero_mean_rows": sum(v == 0 for _, v in advantages),
                          "aggregate_positive_transition_fraction": sum(x["advantage_positive_count"] for _, x in items) / sum(x["advantage_count"] for _, x in items)})
        by_start[label] = {"offset_s": summary["episode_start_offset_choices_s"][labels.index(label)],
                           "return": return_stats, "advantage_normalized": adv_stats,
                           "observed_episode_length_mean_values": sorted(set(x["episode_length_mean"] for _, x in items if x["episode_length_mean"] is not None)),
                           "per_update": [{"logical_iteration": i, "completed_episodes": x["completed_episodes"],
                                           "episode_return_mean": x["episode_return_mean"], "advantage_mean": x["advantage_mean"]} for i, x in items]}
    synchronous = all(len({r["per_start_metrics"][label]["completed_episodes"] for label in labels}) == 1 for r in rows)
    complete_iterations = [r["iteration"] for r in rows if rows[0]["per_start_metrics"] and r["per_start_metrics"][labels[0]]["completed_episodes"] > 0]
    empty_iterations = [r["iteration"] for r in rows if r["per_start_metrics"][labels[0]]["completed_episodes"] == 0]
    vf_series = [(r["iteration"], r["learner_metrics"]["learners/default_policy/vf_explained_var"]) for r in rows]
    vf = stat(vf_series)
    vf["negative_iterations"] = [i for i, v in vf_series if v < 0]
    vf["per_update"] = [{"logical_iteration": i, "vf_explained_var": v} for i, v in vf_series]
    parent_dir = Path(summary["resume_from"]).parent
    parent_summary = json.loads((parent_dir / "summary.json").read_text())
    parent_rows = [json.loads(l) for l in (parent_dir / "train_iterations.jsonl").read_text().splitlines() if l.strip()]
    warmup = {
        "source": str(parent_dir.relative_to(ROOT)), "parent_updates": len(parent_rows),
        "freeze_actor": parent_summary["freeze_actor"], "freeze_logstd": parent_summary["freeze_logstd"],
        "exact_start_sampling": parent_summary["exact_start_sampling"],
        "episode_start_offset_s": parent_summary["episode_start_offset_s"],
        "episode_start_offset_choices_s": parent_summary["episode_start_offset_choices_s"],
        "recorded_start_coverage": parent_rows[0]["start_coverage_metrics"],
        "vf_explained_var": parent_rows[0]["learner_metrics"]["learners/default_policy/vf_explained_var"],
        "morphology_knee_weight": parent_summary["reward_config"]["morphology_knee_weight"],
        "morphology_ankle_weight": parent_summary["reward_config"]["morphology_ankle_weight"],
        "actor_freeze_audit": parent_summary["actor_freeze_audit"],
        "evidence": "Only the nominal offset is configured and only nominal4096 transitions are present in start_coverage_metrics.",
    }
    global_adv_means = [sum(r["per_start_metrics"][l]["advantage_sum"] for l in labels) / sum(r["per_start_metrics"][l]["advantage_count"] for l in labels) for r in rows]
    sources = [RUN / "summary.json", RUN / "train_iterations.jsonl", RUN / "checkpoint_best_meta.json",
               parent_dir / "summary.json", parent_dir / "train_iterations.jsonl", parent_dir / "training_cfg.resolved.yaml",
               ROOT / "Trajectory Generator/baseline_MLP/start_condition_metrics.py"]
    result = {
        "audit_date": "2026-10-12", "scope": "Supplement of historical statistics only; no native test, rollout, training, or production change.",
        "run": str(RUN.relative_to(ROOT)), "new_updates": 50, "best_logical_iteration": 47,
        "best47_new_actor_updating_iterations": 46,
        "returns_population": "38 completed-episode groups per offset (four episodes per group), not 50 return observations; 12 updates have no completed episode and null means.",
        "per_start": by_start,
        "completed_episode_synchrony": {"equal_count_each_update_across_three_offsets": synchronous,
                                        "completed_iterations": complete_iterations, "no_completed_iterations": empty_iterations,
                                        "completed_groups_per_offset": len(complete_iterations), "completed_episodes_per_offset": 152,
                                        "episode_lengths_all500": all(x == 500.0 for d in by_start.values() for x in d["observed_episode_length_mean_values"]),
                                        "interpretation": "Synchronized completions follow identical384-step fragments and500-step episodes on four runners per fixed start. This is evidence of repeated schedule/length, not proof of independence of trajectories or reset observations."},
        "vf_explained_var_aggregate": vf, "critic_rewarm_parent": warmup,
        "training_objective_change": {"parent_morphology_weights": [warmup["morphology_knee_weight"], warmup["morphology_ankle_weight"]],
                                      "ppo_morphology_weights": [summary["reward_config"]["morphology_knee_weight"], summary["reward_config"]["morphology_ankle_weight"]]},
        "normalized_advantage_global_mean_max_abs": max(abs(v) for v in global_adv_means),
        "advantage_semantics": "Current start_condition_metrics.py compacts real transitions, globally re-standardizes advantages, then logs those consumed by the learner. Offset sign is relative to the globally standardized batch, not an absolute raw-return or raw-value-error sign. All three starts have equal1536 rows.",
        "causal_limit": "The nominal-only critic warm-up, changed reward weights, low aggregate VF explained variance and offset advantage asymmetry are observed. They do not establish an uncalibrated critic as the cause of weak policy learning. No per-offset VF errors/targets, raw advantages or controlled critic rewarm comparison are available in these records.",
        "first_audit_preserved_sha256": preserved,
        "sources": [{"path": str(p.relative_to(ROOT)), "sha256": digest(p)} for p in sources],
    }
    assert synchronous and len(complete_iterations) == 38
    dest = OUT / "distribution_metrics.json"
    if dest.exists():
        raise FileExistsError("Refusing to overwrite existing metrics supplement")
    dest.write_text(json.dumps(result, indent=2) + "\n")
    assert preserved == {name: digest(OUT / name) for name in preserved}
    print(json.dumps({"new_updates": 50, "best47_new_updates": 46, "completed_groups": len(complete_iterations),
                      "completed_synchronous": synchronous, "vf_explained_var": {k: v for k, v in vf.items() if k != "per_update"},
                      "critic_warmup_start": warmup["episode_start_offset_s"], "first_audit_preserved": True}, indent=2))


if __name__ == "__main__":
    main()
