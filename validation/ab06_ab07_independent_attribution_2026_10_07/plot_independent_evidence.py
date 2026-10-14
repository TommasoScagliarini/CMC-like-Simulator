"""Plot the measured contrasts frozen in the independent Codex findings.

Audit figures only; no rollouts or operational model edits. Works with the
same Matplotlib API on macOS and Windows. Replicate dots are the three
selected cycles, not confidence intervals or independent subject samples.
"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]


def main() -> None:
    findings = HERE / "codex_independent_findings.json"
    data = json.loads(findings.read_text(encoding="utf-8"))
    facts = data["plot_facts"]
    out = ROOT / "plot/AB06_AB07_independent_attribution_2026_10_07"
    out.mkdir(parents=True, exist_ok=True)
    plt.rcParams.update({"font.size": 11, "axes.titlesize": 13})
    fig, axes = plt.subplots(1, 2, figsize=(13, 5.7), layout="constrained")
    fig.suptitle("AB06–AB07: audit indipendente della preparazione", fontsize=16)
    labels = ["Healthy\nIK originale", "Healthy\nnuova IK comune", "Protesico\nnuova IK comune"]
    values = facts["stance_joint_center_AB07_minus_AB06_mm"]
    colors = ["#4477AA", "#66AADD", "#EE8866"]
    ax = axes[0]
    bars = ax.bar(np.arange(3), values, color=colors, width=0.58)
    ax.axhline(0, color="black", linewidth=0.8)
    for i, (bar, value) in enumerate(zip(bars, values)):
        cycles = facts["per_cycle_stance_joint_center_AB07_minus_AB06_mm"][i]
        ax.scatter(i + np.linspace(-0.13, 0.13, len(cycles)), cycles,
                   color="#333333", s=22, zorder=3)
        ax.annotate(f"{value:+.2f}", (bar.get_x()+bar.get_width()/2, value),
                    xytext=(0, 8 if value >= 0 else -18), textcoords="offset points",
                    ha="center", fontweight="bold")
    ax.set_xticks(np.arange(3), labels)
    ax.set_ylabel("Altezza caviglia AB07 − AB06 in appoggio [mm]")
    ax.set_title("La costruzione e la IK invertono il confronto")
    ax.grid(axis="y", alpha=0.2)
    ax.set_axisbelow(True)
    displayed = [0., *values, *(v for cycle in facts["per_cycle_stance_joint_center_AB07_minus_AB06_mm"] for v in cycle)]
    lo, hi = min(displayed), max(displayed)
    padding = max(4., (hi-lo)*0.2)
    ax.set_ylim(lo-padding, hi+padding)
    ax.text(0.02, 0.03, "Punti: i tre cicli selezionati", transform=ax.transAxes,
            fontsize=9, color="#444444")

    ax = axes[1]
    x = np.arange(2)
    for i, subject in enumerate(("AB06", "AB07")):
        force = facts["reference_FK_mean_force_N"][subject]
        bars = ax.bar(x + (i-0.5)*0.32, force, width=0.32,
                      color=("#4477AA", "#EE8866")[i], label=subject)
        for bar, value in zip(bars, force):
            ax.annotate(f"{value:.1f}", (bar.get_x()+bar.get_width()/2, value),
                        xytext=(0, 5), textcoords="offset points", ha="center", fontsize=10)
    ax.set_xticks(x, ["Modello + IK\nprecedenti", "Modello + IK\nprocedura comune"])
    ax.set_ylabel("Forza verticale media del contatto [N]")
    ax.set_title("Il gap resta identico nel rebuild controllato")
    ax.set_ylim(0, max(max(v) for v in facts["reference_FK_mean_force_N"].values())*1.2)
    ax.grid(axis="y", alpha=0.2)
    ax.set_axisbelow(True)
    ax.legend(frameon=False, loc="upper left")
    fig.supxlabel("Centri articolari funzionalmente omologhi. Forze da replay FK dei riferimenti filtrati; nessuna nuova dinamica.",
                  fontsize=9)
    artifacts = []
    for extension in ("png", "svg"):
        path = out / ("independent_preparation_evidence." + extension)
        fig.savefig(path, dpi=180)
        artifacts.append({"path": path.relative_to(ROOT).as_posix(),
                          "sha256": hashlib.sha256(path.read_bytes()).hexdigest()})
    plt.close(fig)
    print(json.dumps({"source": str(findings), "figures": artifacts}, indent=2))


if __name__ == "__main__":
    main()
