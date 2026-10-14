"""Offline diagnostic replay and compact figures for CP200/250/283/300.

Requires the completed evaluate_training_300.py output. Never launches training
or a simulator, and does not modify checkpoints or runtime configuration.
"""
from __future__ import annotations

import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

import analyze_training_continuation as previous
import evaluate_training_300 as evaluation

OUT = evaluation.OUTPUT
ITERATIONS = (200, 250, 283, 300)
STARTS = ("early", "center", "late")


def read(path):
    return json.loads(path.read_text(encoding="utf-8"))


def save(fig, name):
    for extension in ("png", "pdf"):
        fig.savefig(OUT / f"{name}.{extension}", dpi=170, bbox_inches="tight")
    plt.close(fig)


def main():
    report = read(OUT / "milestone_evaluation.json")
    assert report["status"] == "complete" and len(report["cases"]) == 12
    assert len(report["common_prefix_comparisons"]) == 18
    sources = {OUT / "milestone_evaluation.json"}
    for case in report["cases"]:
        for name, expected in case["source_hashes"].items():
            path = Path(case["source_directory"]) / name
            assert evaluation.metrics.sha256(path) == expected, path
            sources.add(path)
    replays = previous.replay_policy(report, sources)
    cases = {(case["checkpoint_iteration"], case["start"]): case for case in report["cases"]}
    compact = []
    for iteration in ITERATIONS:
        for start in STARTS:
            case = cases[(iteration, start)]
            full = case["full_episode"]
            row = {"iteration": iteration, "start": start,
                   "samples": full["samples"], "duration_s": full["duration_s"],
                   "return": full["reward"]["episode_return"],
                   "valid_cycles": full["phase"]["valid_cycle_count"],
                   "invalid_events": full["phase"]["invalid_event_count"],
                   "first_online_period_s": full["phase"]["first_online_period_available_elapsed_s"],
                   "penetration_max_mm": full["safety"]["penetration_max_mm"],
                   "clipped_steps": full["safety"]["clipped_policy_steps"],
                   "termination": full["termination"]["end_reason"]}
            for joint in ("knee", "ankle"):
                tracking = full["tracking"][joint]
                for relation in ("actual_vs_prescribed", "served_vs_prescribed", "actual_vs_served"):
                    row[f"{joint}_{relation}_rmse_deg"] = tracking[relation]["position"]["rmse_deg"]
                for relation in ("actual_vs_prescribed", "served_vs_prescribed"):
                    row[f"{joint}_{relation}_excursion_deg"] = tracking[relation]["excursion_deg"]
                row[f"{joint}_prescribed_excursion_deg"] = tracking["prescribed_excursion_deg"]
            compact.append(row)
    aggregates = {}
    for iteration in ITERATIONS:
        selected = [row for row in compact if row["iteration"] == iteration]
        aggregates[str(iteration)] = {
            "scope": "Descriptive arithmetic mean/range over the three fixed starts; no confidence interval",
            "completed_5s": sum(row["duration_s"] >= 4.99 for row in selected),
            "total_valid_cycles": sum(row["valid_cycles"] for row in selected),
            "total_invalid_events": sum(row["invalid_events"] for row in selected),
        }
        for key in ("return", "penetration_max_mm", "knee_actual_vs_prescribed_rmse_deg",
                    "ankle_actual_vs_prescribed_rmse_deg", "knee_actual_vs_prescribed_excursion_deg",
                    "knee_actual_vs_served_rmse_deg", "ankle_actual_vs_served_rmse_deg"):
            values = [row[key] for row in selected]
            aggregates[str(iteration)][key] = {"mean": float(np.mean(values)), "min": min(values), "max": max(values)}
    diagnostics = {"cases": compact, "aggregates": aggregates, "policy_replay": replays,
                   "quality_gates": report["quality_gates"],
                   "source_sha256": {str(path): evaluation.metrics.sha256(path) for path in sorted(sources)}}
    evaluation.metrics.write_json(OUT / "evaluation_diagnostics.json", diagnostics)
    evaluation.metrics.analysis.write_csv(OUT / "evaluation_summary.csv", compact)

    plt.rcParams.update({"font.size": 10, "axes.spines.top": False, "axes.spines.right": False})
    fig, axes = plt.subplots(2, 3, figsize=(13, 8))
    panels = (
        ("knee_actual_vs_prescribed_rmse_deg", "Ginocchio: errore actual–prescribed [°]"),
        ("ankle_actual_vs_prescribed_rmse_deg", "Caviglia: errore actual–prescribed [°]"),
        ("knee_actual_vs_prescribed_excursion_deg", "Escursione globale ginocchio [°]"),
        ("invalid_events", "Eventi invalidi FSM"),
        ("penetration_max_mm", "Penetrazione massima [mm]"),
        ("valid_cycles", "Cicli validi FSM"),
    )
    for ax, (key, title) in zip(axes.flat, panels):
        for start, color in zip(STARTS, ("#0072B2", "#D55E00", "#009E73")):
            selected = [next(row for row in compact if row["iteration"] == i and row["start"] == start) for i in ITERATIONS]
            ax.plot(range(4), [row[key] for row in selected], marker="o", lw=1.7, label=start, color=color)
        ax.set(title=title, xticks=range(4), xticklabels=[f"CP{i}" for i in ITERATIONS], ylim=(0, None))
        ax.grid(alpha=.22)
        if key == "knee_actual_vs_prescribed_excursion_deg":
            ax.axhline(compact[0]["knee_prescribed_excursion_deg"], color="black", ls="--", label="Prescribed")
            ax.set_ylim(0, 53)
            ax.legend(fontsize=8)
        if key == "penetration_max_mm":
            ax.axhline(28, color=".4", ls="--", label="Soglia hard")
            ax.set_ylim(0, 30)
            ax.legend(fontsize=8)
        if key in ("invalid_events", "valid_cycles"):
            ax.yaxis.set_major_locator(plt.MaxNLocator(integer=True))
    axes[0, 0].legend(fontsize=8)
    fig.suptitle("V1 35D: confronto deterministico sui tre start\nErrori calcolati agli stessi tempi fisici, senza riallineamento della fase")
    fig.tight_layout(rect=(0, 0, 1, .93))
    save(fig, "deterministic_comparison_200_300")

    fig, axes = plt.subplots(2, 3, figsize=(14, 7), sharex=True, sharey="row")
    for column, start in enumerate(STARTS):
        traces = {i: read(Path(cases[(i, start)]["source_directory"]) / "rollout_policy_trace.json") for i in (200, 283, 300)}
        for row, (joint, label) in enumerate((("pros_knee_angle", "Ginocchio [°]"), ("pros_ankle_angle", "Caviglia [°]"))):
            ax = axes[row, column]
            target = traces[300]
            origin = target[0]["time"] - .01
            ax.plot([r["time"]-origin for r in target], np.rad2deg([r["imitation_target_q"][joint] for r in target]), color="black", lw=1.7, label="Prescribed")
            for iteration, color in ((200, "#999999"), (283, "#0072B2"), (300, "#D55E00")):
                trace = traces[iteration]
                trace_origin = trace[0]["time"] - .01
                ax.plot([r["time"]-trace_origin for r in trace], np.rad2deg([r["prosthetic_state"][joint] for r in trace]), color=color, lw=1.3, label=f"CP{iteration}")
            ax.set(xlim=(0, 5), title=start if row == 0 else "", ylabel=label if column == 0 else "", xlabel="Tempo dal reset [s]" if row else "")
            ax.grid(alpha=.2)
    axes[0, 0].legend(fontsize=8)
    fig.suptitle("Traiettorie effettive: CP200, best training CP283 e ultimo CP300\nConfronto con prescribed alla stessa fase temporale; nessuno sfasamento applicato")
    fig.tight_layout(rect=(0, 0, 1, .92))
    save(fig, "kinematics_200_283_300")
    print(json.dumps({"aggregates": aggregates, "quality_gates": report["quality_gates"]}, indent=2))


if __name__ == "__main__":
    main()
