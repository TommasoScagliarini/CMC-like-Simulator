"""Standalone overview of the frozen pose pool and observed random-pose results."""
from pathlib import Path
import json
import math
import os
import importlib.util

os.environ.setdefault("MPLBACKEND", "Agg")
import matplotlib.pyplot as plt

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]


def main():
    selection = json.loads((HERE / "preflight/selected_cases.json").read_text())
    manifest = json.loads((HERE / "frozen_manifest.json").read_text())
    aggregate = json.loads((HERE / "aggregate_results.json").read_text())
    declared = {c["id"]: c for c in manifest["cases"]}
    eligible, rejected = [], []
    for r in selection["preflight_outcomes"]:
        if not r["id"].startswith("random_"):
            continue
        q = declared[r["id"]]["pose"]
        pair = (q["pros_knee_angle"]*180/math.pi, q["pros_ankle_angle"]*180/math.pi)
        (eligible if r["status"] == "static_eligible" else rejected).append(pair)
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.8), constrained_layout=True)
    ax = axes[0]
    for points, color, marker, label in ((eligible, "#d8d8d8", ".", "Pool ammesso"),
                                          (rejected, "#6e6e6e", "x", "Escluso al reset")):
        if points:
            ax.scatter(*zip(*points), c=color, marker=marker, s=18, label=label)
    styles = {"GAIT_RECOVERY_PASS": ("#16834a", "o", "Recupero qualificato"),
              "GAIT_CRITERION_FAILURE": ("#d58a00", "D", "Criteri gait non soddisfatti"),
              "PHYSICAL_FAILURE": ("#bb302e", "s", "Failure fisico"),
              "TECHNICAL_CENSORED": ("#6656ba", "X", "Censura tecnica"),
              "POSTPROCESSING_UNRESOLVED": ("#6656ba", "P", "Misure irrisolte")}
    for name, (color, marker, label) in styles.items():
        selected = [r for r in aggregate["random_cases"] if r["classification"] == name]
        if not selected:
            continue
        xs = [r["case"]["pose"]["pros_knee_angle"]*180/math.pi for r in selected]
        ys = [r["case"]["pose"]["pros_ankle_angle"]*180/math.pi for r in selected]
        ax.scatter(xs, ys, c=color, marker=marker, s=68, edgecolors="black", linewidths=.5, label=label, zorder=4)
        for r, x, y in zip(selected, xs, ys):
            ax.annotate(r["id"].removeprefix("random_"), (x, y), xytext=(3, 4), textcoords="offset points", fontsize=7)
    ax.set(xlabel="Ginocchio iniziale [°]", ylabel="Caviglia iniziale [°]",
           title="400 proposte e prime 20 ammesse")
    ax.legend(fontsize=8, loc="lower left")
    ax.grid(alpha=.2)
    ax = axes[1]
    frontier = aggregate["completion_vs_strict_frontier"]
    labels = ["5 s completati", "Guardie rispettate\ne 5 s completati", "Recupero gait\nqualificato"]
    values = [frontier["run_completed"], frontier["completed_safely"], aggregate["successes"]]
    bars = ax.bar(labels, values, color=["#5287b3", "#306da0", "#16834a"])
    for bar, value in zip(bars, values):
        ax.text(bar.get_x()+bar.get_width()/2, value+.2, f"{value}/20", ha="center")
    ax.set(ylim=(0, 22), ylabel="Pose selezionate", title="Esiti con policy best47 congelata")
    ax.set_yticks(range(0, 21, 5))
    ax.grid(axis="y", alpha=.2)
    ci = aggregate["success_probability_cp95"]
    foot = "AB06; biologia IK; qdot protesiche zero; 5 s per caso."
    if ci:
        foot += f" Recupero: {100*ci['estimate']:.1f}%, CI esatto95% [{100*ci['two_sided95'][0]:.1f}, {100*ci['two_sided95'][1]:.1f}]%."
    fig.suptitle(foot, fontsize=10)
    folder = REPO / "plot/random_pose_reset_2026_10_13/summary"
    folder.mkdir(parents=True, exist_ok=False)
    fig.savefig(folder / "pose_pool_and_recovery.png", dpi=180)
    fig.savefig(folder / "pose_pool_and_recovery.pdf")
    plt.close(fig)
    failed = [r for r in aggregate["random_cases"] if r["classification"] == "PHYSICAL_FAILURE"]
    if failed:
        spec = importlib.util.spec_from_file_location("pose_plot_saved_reader", HERE / "aggregate_results_v2.py")
        reader = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(reader)
        fig, ax = plt.subplots(figsize=(8, 4.5), constrained_layout=True)
        for result in failed:
            path = HERE / "rollouts" / result["id"] / "sim_outputs/rollout_episode_online_grf.sto"
            if not path.exists():
                continue
            grf, _ = reader.storage(path)
            receipt = json.loads((path.parent.parent / "receipt.json").read_text())
            t0 = receipt["initial_state"]["time_s"]
            ax.plot([t-t0 for t in grf["time"]], [1000*x for x in grf["left_penetration"]], label=result["id"]+" (sample salvati)")
            contacts = result.get("returned_journal_evidence", {}).get("measured_terminal_contacts", [])
            for contact in contacts:
                ax.scatter(contact["physical_sample_time_s"]-t0, 1000*contact["penetration_m"], c="#bb302e", marker="x", s=70, zorder=5)
        ax.axhline(27, color="#888888", linestyle=":", label="Gate iniziale 27 mm")
        ax.axhline(28, color="#d58a00", linestyle="--", label="Limite episodio 28 mm")
        ax.axhline(30, color="#bb302e", linestyle="--", label="Guardia fisica 30 mm")
        ax.set(xlabel="Tempo dal reset [s]", ylabel="Penetrazione primaria sinistra [mm]", title="Interruzione fisica dopo un reset inizialmente ammesso")
        ax.grid(alpha=.2)
        ax.legend(fontsize=8)
        fig.savefig(folder / "failure_contact_guard.png", dpi=180)
        fig.savefig(folder / "failure_contact_guard.pdf")
        plt.close(fig)
    print(folder)


if __name__ == "__main__":
    main()
