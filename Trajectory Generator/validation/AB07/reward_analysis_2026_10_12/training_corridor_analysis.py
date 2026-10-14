"""Read saved training telemetry; never load OpenSim or run a policy.

Requires TensorBoard, available in the existing envCMC-rllib environment.
Morphology scalars summarize each runner's last 100 steps in each update,
not the full batch. Ray clears these windows when reducing each result.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
from statistics import mean

from tensorboard.backend.event_processing.event_accumulator import EventAccumulator


REPO = Path(__file__).resolve().parents[4]
DEFAULT_RUN = REPO / "Trajectory Generator/runs/training/MLP_V26C_J20_sigma0p010_morph_equal5p0_ppo50_2026-09-01"
TAGS = {
    "window_morphology_cost_mean": "reward/morphology_term",
    "window_knee_cost_mean": "reward/morphology_knee_term",
    "window_ankle_cost_mean": "reward/morphology_ankle_term",
    "window_reward_mean": "reward/reward",
    "window_reward_base_mean": "reward/reward_base",
    "window_morphology_available_mean": "reward/morphology_available",
    "window_morphology_inside_fraction_mean": "reward/morphology_inside_fraction",
    "batch_reward_sum_tb": "env_runners/console/reward_sum_current",
    "batch_sample_count": "env_runners/console/reward_count_current",
    "completed_episodes": "env_runners/num_episodes",
}


def fingerprint(path: Path) -> dict:
    try:
        label = str(path.relative_to(REPO))
    except ValueError:
        label = str(path)
    return {"path": label, "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}


def historical_repo_path(recorded: str) -> Path:
    """Resolve archived macOS/Windows source paths against the current checkout."""
    normalized = recorded.replace("\\", "/")
    anchor = "Trajectory Generator/"
    if anchor in normalized:
        return REPO / anchor / normalized.split(anchor, 1)[1]
    return REPO / normalized


def aggregate(rows: list[dict]) -> dict:
    cost = mean(row["window_morphology_cost_mean"] for row in rows)
    knee = mean(row["window_knee_cost_mean"] for row in rows)
    ankle = mean(row["window_ankle_cost_mean"] for row in rows)
    reward = mean(row["window_reward_mean"] for row in rows)
    batch_count = math.fsum(row["batch_sample_count"] for row in rows)
    batch_reward = math.fsum(row["batch_reward_sum_tb"] for row in rows)
    return {
        "logical_iteration_first": rows[0]["logical_iteration"],
        "logical_iteration_last": rows[-1]["logical_iteration"],
        "new_update_count": len(rows),
        "new_sample_count": int(batch_count),
        "all_sample_reward_sum_from_tb": batch_reward,
        "all_sample_reward_mean_from_tb": batch_reward / batch_count,
        "completed_episode_count": int(math.fsum(row["completed_episodes"] for row in rows)),
        "completed_episode_return_sum_from_json": math.fsum(
            row["completed_episode_return_sum_json"] for row in rows
        ),
        "mean_of_logged_window_morphology_cost_means": cost,
        "mean_of_logged_window_knee_cost_means": knee,
        "mean_of_logged_window_ankle_cost_means": ankle,
        "ankle_share_of_logged_window_cost": ankle / cost if cost else None,
        "mean_of_logged_window_reward_means": reward,
        "mean_of_logged_window_reward_before_morphology": reward + cost,
        "morphology_fraction_of_window_reward_before_morphology": (
            cost / (reward + cost) if reward + cost else None
        ),
        "logged_window_morphology_cost_mean_min": min(
            row["window_morphology_cost_mean"] for row in rows
        ),
        "logged_window_morphology_cost_mean_max": max(
            row["window_morphology_cost_mean"] for row in rows
        ),
        "full_sample_morphology_cost_sum": None,
        "full_sample_reward_sum_before_morphology": None,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", type=Path, default=DEFAULT_RUN)
    parser.add_argument("--output-dir", type=Path, default=Path(__file__).resolve().parent)
    args = parser.parse_args()
    run = args.run_dir.resolve()
    history_path = run / "train_iterations.jsonl"
    history = [json.loads(line) for line in history_path.read_text().splitlines() if line]
    summary = json.loads((run / "summary.json").read_text())
    meta = json.loads((run / "checkpoint_best_meta.json").read_text())
    rewarm_summary_path = historical_repo_path(summary["resume_from"]).parent / "summary.json"
    rewarm = json.loads(rewarm_summary_path.read_text())
    j20_summary_path = historical_repo_path(rewarm["warm_start_source"]).parent / "summary.json"
    j20 = json.loads(j20_summary_path.read_text())
    j19a_result_path = historical_repo_path(j20["warm_start_source"]).parent / "v26c_j19a_result.json"
    j19a = json.loads(j19a_result_path.read_text())
    events = sorted((run / "tensorboard").glob("events.out.tfevents.*"))
    accumulator = EventAccumulator(str(run / "tensorboard"), size_guidance={"scalars": 0})
    accumulator.Reload()
    scalars = {}
    for field, tag in TAGS.items():
        values = accumulator.Scalars(tag)
        mapping = {event.step: event.value for event in values}
        if len(mapping) != len(values):
            raise ValueError(f"Duplicate TensorBoard step for {tag}; reconcile before aggregation")
        scalars[field] = mapping

    rows = []
    for update_index, record in enumerate(history, start=1):
        step = int(record["num_env_steps_sampled_lifetime"])
        per_start = record["per_start_metrics"].values()
        row = {
            "logical_iteration": int(record["iteration"]),
            "new_update_index": update_index,
            "sampled_steps_lifetime_including_restored": step,
            "source_jsonl_line": update_index,
            "episode_return_mean": record["episode_return_mean"],
            "episode_length_mean": record["episode_len_mean"],
            "console_return_source": record["console_episode_metrics"]["source"],
            "completed_episode_return_sum_json": math.fsum(
                item["episode_return_sum"] or 0.0 for item in per_start
            ),
            **{field: values[step] for field, values in scalars.items()},
        }
        row["batch_reward_mean_tb"] = row["batch_reward_sum_tb"] / row["batch_sample_count"]
        row["window_reward_before_morphology_mean"] = (
            row["window_reward_mean"] + row["window_morphology_cost_mean"]
        )
        rows.append(row)

    best_iteration = int(meta["logical_iteration"])
    through_best = [row for row in rows if row["logical_iteration"] <= best_iteration]
    best = next(row for row in rows if row["logical_iteration"] == best_iteration)
    cumulative = 0
    for row in rows:
        cumulative += int(row["batch_sample_count"])
        row["new_sample_count_cumulative"] = cumulative
    restored_samples = (
        rows[0]["sampled_steps_lifetime_including_restored"] - rows[0]["batch_sample_count"]
    )
    coefficients_error = max(abs(
        row["window_morphology_cost_mean"]
        - row["window_knee_cost_mean"] - row["window_ankle_cost_mean"]
    ) for row in rows)
    if coefficients_error > 1e-7:
        raise ValueError("Morphology total and joint-specific telemetry disagree")

    metrics = {
        "schema_version": 1,
        "scope": "Only the additional ex_novo PPO phase of 1-2 September 2026: 50 new updates (logical2-51), or its first 46 updates through best47. This is not the policy's complete training history; excludes inherited J19A/prior actor adaptation, critic warmup/rewarm, and posttraining rollouts.",
        "run": fingerprint(run / "training_cfg.resolved.yaml"),
        "sources": [fingerprint(path) for path in [
            history_path, run / "summary.json", run / "checkpoint_best_meta.json",
            rewarm_summary_path, j20_summary_path, j19a_result_path,
            REPO / "Trajectory Generator/baseline_MLP/tb_logging.py", *events,
        ]],
        "method": {
            "tensorboard_tags": TAGS,
            "tensorboard_step_axis": "num_env_steps_sampled_lifetime, joined exactly to train_iterations.jsonl",
            "reward_components": "Ray 2.55.1 MeanStats: each EnvRunner retains its last 100 step values; reduce() clears the deque, and distributed merge takes a mean of means. Each update here samples 384 steps per runner across 12 runners. A logged point therefore summarizes the 100-step tails, nominally 1200 of 4608 transitions, not the full batch or complete episodes.",
            "window_aggregation": "Equal arithmetic mean of the saved per-update tail means. Consecutive update tails do not overlap because reduce() clears the values and each runner samples 384 new steps, exceeding the 100-step window. This is not a moving average across training iterations and not a mean over every training transition.",
            "nominal_tail_steps_per_runner_per_update": 100,
            "runner_count": int(summary["num_env_runners"]),
            "steps_per_runner_per_update": 384,
            "nominal_tail_steps_per_update": 1200,
            "nominal_fraction_of_batch_represented": 1200 / 4608,
            "ray_meanstats_evidence": "ray/rllib/utils/metrics/stats/mean.py:17 documents mean-of-means; :52 reduce() calls window_reduce() then _set_values([]). Read-only audit of installed Ray 2.55.1.",
            "batch_reward": "console/reward_sum_current and reward_count_current are unsmoothed update-scoped counters. TensorBoard stores scalar values with float32 rounding.",
            "completed_episode_returns": "Summed per_start_metrics episode_return_sum over completed episodes; episode coverage differs from batch coverage and can cross update boundaries.",
            "morphology_equation": "cost = 2.5*knee_loss + 2.5*ankle_loss; subtract after base reward clipping; values are already weighted costs.",
            "initial_final_best": "Training telemetry at logical iterations 2, 51 and 47 respectively. Iteration 47 reward/morphology metrics describe sampling before update 47; checkpoint_best logical47 stores parameters after that update. They are not deterministic evaluation results of the saved best47 actor.",
            "full_training_morphology_total_recoverable": False,
            "unrecoverable_reason": "No unsmoothed morphology sum/count or complete per-transition training reward trace is retained here. Multiplying the logged 100-step tail means by full batch counts would only extrapolate the unrecorded 284 steps per runner per update.",
            "do_not_infer": "No causal effect of corridor on learned policy or training performance is identifiable from this one run; requires a controlled training ablation.",
            "no_new_training_or_simulation": True,
        },
        "initialization": {
            "analyzed_phase_started_at": summary["started_at"],
            "analyzed_phase_finished_at": summary["finished_at"],
            "scope_is_complete_policy_training_history": False,
            "inherited_actor_previously_trained": True,
            "resume_from": summary["resume_from"],
            "restored_logical_iteration": summary["restored_logical_iteration"],
            "restored_sample_count": int(restored_samples),
            "new_update_count": len(rows),
            "best_logical_iteration": best_iteration,
            "updates_through_best": len(through_best),
            "best_episode_return_mean": summary["best_episode_return_mean"],
            "training_setup": summary["setup_xml_path"],
            "excluded_prior_stage_provenance": [
                {
                    "stage": "Critic rewarm, 31 August 2026",
                    "source": fingerprint(rewarm_summary_path),
                    "initialization_mode": rewarm["initialization_mode"],
                    "actor_source": rewarm["warm_start_source"],
                    "freeze_actor": rewarm["freeze_actor"],
                    "iterations_run": rewarm["iterations_run"],
                },
                {
                    "stage": "J20 critic warmup, 27 August 2026",
                    "source": fingerprint(j20_summary_path),
                    "initialization_mode": j20["initialization_mode"],
                    "actor_source": j20["warm_start_source"],
                    "freeze_actor": j20["freeze_actor"],
                    "iterations_run": j20["iterations_run"],
                },
                {
                    "stage": "J19A actor adaptation and its earlier J18/J8 ancestry",
                    "source": fingerprint(j19a_result_path),
                    "actor_promoted": j19a["actor_promoted"],
                    "best_epoch": j19a["diagnostics"]["best_epoch"],
                    "covered_by_current_metrics": False,
                },
            ],
        },
        "all_50_updates": aggregate(rows),
        "through_best47": aggregate(through_best),
        "first_5_updates": aggregate(rows[:5]),
        "last_5_updates_through_best47": aggregate(through_best[-5:]),
        "last_5_updates": aggregate(rows[-5:]),
        "selected_iterations": {"initial": rows[0], "best47": best, "final": rows[-1]},
        "validation": {
            "history_equals_summary_history": history == summary["history"],
            "all_batches_have_4608_rows": all(row["batch_sample_count"] == 4608 for row in rows),
            "morphology_split_max_abs_discrepancy_tb": coefficients_error,
            "max_window_cost_logical_iteration": max(rows, key=lambda r: r["window_morphology_cost_mean"])["logical_iteration"],
            "min_window_cost_logical_iteration": min(rows, key=lambda r: r["window_morphology_cost_mean"])["logical_iteration"],
        },
    }
    args.output_dir.mkdir(parents=True, exist_ok=True)
    (args.output_dir / "training_corridor_metrics.json").write_text(
        json.dumps(metrics, indent=2, ensure_ascii=False, allow_nan=False) + "\n", encoding="utf-8"
    )
    with (args.output_dir / "training_corridor_timeseries.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    print(json.dumps({"updates": len(rows), "best_logical_iteration": best_iteration,
                      "new_samples": cumulative, "morphology_total_recoverable": False}))


if __name__ == "__main__":
    main()
