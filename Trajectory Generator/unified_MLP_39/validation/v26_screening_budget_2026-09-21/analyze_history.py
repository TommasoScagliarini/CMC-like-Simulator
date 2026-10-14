"""Read historical V26 logs; export screening evidence without running a policy."""
from __future__ import annotations

import csv
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from tensorboard.backend.event_processing.event_accumulator import EventAccumulator
import yaml


OUT = Path(__file__).resolve().parent
TG = OUT.parents[2]
OLD = TG / "runs/training/MLP_imitation_native_v26_08-20-2026_june_equiv_100iter"
NEW = TG / "unified_MLP_39/runs/training/MLP39_phase_velocity_scale1_2026-09-18"


def main():
    history = [json.loads(line) for line in (OLD / "train_iterations.jsonl").read_text().splitlines()]
    events = EventAccumulator(str(OLD / "tensorboard"), size_guidance={"scalars": 0})
    events.Reload()
    tags = {
        "knee_position_loss_smoothed": "reward_diagnostic/pros_knee_angle_imitation_position_loss",
        "ankle_position_loss_smoothed": "reward_diagnostic/pros_ankle_angle_imitation_position_loss",
        "penetration_loss_smoothed": "reward/grf_penetration_loss",
        "imitation_score_smoothed": "reward/imitation_score",
        "episode_len_min": "env_runners/episode_len_min",
    }
    scalar_maps = {name: {event.step: event.value for event in events.Scalars(tag)} for name, tag in tags.items()}
    reasons = ("episode_time_limit", "grf_penetration", "phase_timeout_stance", "phase_timeout_swing")
    prefix = "env_runners/episode_end/"
    rows = []
    previous = {}
    for expected_iteration, item in enumerate(history, 1):
        assert item["iteration"] == expected_iteration
        counts = {key.removeprefix(prefix): value for key, value in item["termination_metrics"].items() if key.startswith(prefix)}
        assert set(counts) <= set(reasons), counts
        row = {key: item[key] for key in ("iteration", "episode_return_mean", "episode_len_mean", "num_env_steps_sampled_lifetime")}
        for reason in reasons:
            delta = counts.get(reason, 0) - previous.get(reason, 0)
            assert delta >= 0 and delta == int(delta)
            row[reason] = int(delta)
        for name, mapping in scalar_maps.items():
            row[name] = mapping[int(item["num_env_steps_sampled_lifetime"])]
        rows.append(row)
        previous = counts
    with (OUT / "v26_history.csv").open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    blocks = []
    for low, high in ((1, 10), (11, 20), (21, 25), (26, 30), (31, 35), (31, 40), (41, 50)):
        counts = {reason: sum(row[reason] for row in rows[low - 1:high]) for reason in reasons}
        blocks.append({"first_iteration": low, "last_iteration": high, **counts,
                       "full_episode_fraction": counts["episode_time_limit"] / sum(counts.values())})
    old_cfg, new_cfg = [yaml.safe_load((path / "training_cfg.resolved.yaml").read_text()) for path in (OLD, NEW)]
    direct_changes = (
        "policy_action_clip_weight", "phase_invalid_event_weight", "phase_contact_validity_weight",
        "phase_timeout_penalty_weight", "prosthetic_joint_range_weight", "grf_ankle_moment_flip_weight",
        "oob_q_min", "prosthetic_joint_q_min", "phase_swing_timeout_s",
    )
    evidence = {
        "training_run": str(OLD.relative_to(TG)),
        "comparison_run": str(NEW.relative_to(TG)),
        "first_full_mean_iteration": next(row["iteration"] for row in rows if row["episode_len_mean"] == 500),
        "blocks": blocks,
        "direct_reward_changes": {key: {"v26": old_cfg["reward"][key], "unified39": new_cfg["reward"][key]} for key in direct_changes},
        "retained_checkpoints": {name: json.loads((OLD / f"checkpoint_{name}_meta.json").read_text())["logical_iteration"] for name in ("best", "last")},
        "limitations": [
            "Episode counts are deltas of recorded cumulative counters; hard worker failures can be absent.",
            "Training episodes use exploratory actions and can span policy updates.",
            "TensorBoard reward diagnostics are smoothed means, not full-rollout errors or maxima.",
            "No intermediate checkpoint or gait rollout was found for iterations 25, 32 or 35.",
            "Zero penetration terminations does not imply zero geometric penetration or zero penalty.",
            "The reward diff above lists direct changes, not all detector/FSM configuration differences.",
        ],
    }
    (OUT / "evidence.json").write_text(json.dumps(evidence, indent=2) + "\n")

    x = [row["iteration"] for row in rows]
    fig, axes = plt.subplots(3, 1, figsize=(11, 10), sharex=True, layout="constrained")
    axes[0].plot(x, [row["episode_len_mean"] for row in rows], label="Durata media registrata", color="#2563eb")
    axes[0].axhline(500, color="#475569", linestyle=":", label="Episodio completo: 500 step")
    axes[0].set_ylabel("Step per episodio")
    axes[0].legend(loc="lower right")
    full_fractions = []
    for i in range(len(rows)):
        window = rows[max(0, i - 4):i + 1]
        total = sum(sum(row[reason] for reason in reasons) for row in window)
        full_fractions.append(100 * sum(row["episode_time_limit"] for row in window) / total)
    axes[1].plot(x, full_fractions, color="#059669", label="Episodi completi / conclusi registrati, ultime 5 iter")
    axes[1].set_ylabel("Completamento (%)")
    axes[1].set_ylim(0, 105)
    axes[1].legend(loc="lower right")
    for key, label, color in (("knee_position_loss_smoothed", "Ginocchio", "#7c3aed"), ("ankle_position_loss_smoothed", "Caviglia", "#ea580c")):
        axes[2].semilogy(x, [row[key] for row in rows], label=label, color=color)
    axes[2].set_ylabel("Errore quadratico normalizzato\n(media mobile TensorBoard)")
    axes[2].set_xlabel("Iterazione di training V26")
    axes[2].legend(loc="upper right")
    for axis in axes:
        axis.axvspan(25, 35, color="#fbbf24", alpha=0.15)
        axis.axvline(32, color="#475569", linestyle="--", alpha=0.65)
        axis.grid(alpha=0.2)
        axis.set_xlim(1, 100)
    fig.suptitle("V26, 20 agosto: primi segnali a 20–25 iter, primo gruppo tutto completo a 32\n"
                 "Training esplorativo; le curve non certificano la qualità del rollout deterministico", fontsize=12)
    fig.savefig(OUT / "v26_screening_history.png", dpi=170)
    plt.close(fig)
    print(json.dumps({"rows": len(rows), "first_full_mean_iteration": evidence["first_full_mean_iteration"], "outputs": ["v26_history.csv", "evidence.json", "v26_screening_history.png"]}))


if __name__ == "__main__":
    main()
