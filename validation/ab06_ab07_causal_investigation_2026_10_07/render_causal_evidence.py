"""Render saved causal-audit evidence; no OpenSim or dynamic execution."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


ROOT = Path(__file__).resolve().parents[2]
AUDIT = Path(__file__).resolve().parent
OUTPUT = ROOT / "plot/AB06_AB07_causal_investigation_2026_10_07"


def fingerprint(path):
    return {"path": path.relative_to(ROOT).as_posix(),
            "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}


def main():
    paths = {"basis": AUDIT / "contact_basis_ablation.json",
             "geometry": AUDIT / "model/factorial_geometry_fk.json",
             "balance": AUDIT / "alternative/vertical_impulse_audit.json"}
    before = {key: fingerprint(path) for key, path in paths.items()}
    evidence = {key: json.loads(path.read_text(encoding="utf-8")) for key, path in paths.items()}
    plt.rcParams.update({"font.size": 10, "axes.spines.top": False, "axes.spines.right": False})
    fig, axes = plt.subplots(1, 3, figsize=(17.5, 5.7), layout="constrained")
    subjects = ("AB06", "AB07")
    case = evidence["basis"]["cases"]["actual"]["subjects"]
    other = [case[subject]["other_spheres_mean_force_n"] for subject in subjects]
    basis = [case[subject]["basis07_mean_force_n"] for subject in subjects]
    ax = axes[0]
    ax.bar(subjects, other, color="#738496", label="Altre 7 sfere")
    ax.bar(subjects, basis, bottom=other, color="#d78132", label="left_basis_07")
    for index, subject in enumerate(subjects):
        ax.text(index, other[index] / 2, f"{other[index]:.1f}", ha="center", va="center", color="white")
        ax.text(index, other[index] + basis[index] / 2, f"{basis[index]:.1f}", ha="center", va="center")
        ax.text(index, other[index] + basis[index] + 8, f"{other[index] + basis[index]:.1f} N", ha="center")
    ax.set_ylim(0, 470)
    ax.set_ylabel("Forza verticale media [N]")
    ax.set_title("1. Da quale componente passa il divario?\nLegge di contatto sulle pose del rollout")
    ax.legend(loc="upper left", frameon=False)
    ax.text(.5, -.15, "Pose congelate: ablazione algebrica,\nnessuna previsione di un nuovo rollout.", transform=ax.transAxes, ha="center", va="top")

    lineage = evidence["geometry"]["registration_lineage_2_by_2"]
    values = np.array([[lineage["G_pre_ground__Q_pre_ground"]["aggregate"]["mean_force_n"],
                        lineage["G_pre_ground__Q_current"]["aggregate"]["mean_force_n"]],
                       [lineage["G_current__Q_pre_ground"]["aggregate"]["mean_force_n"],
                        lineage["G_current__Q_current"]["aggregate"]["mean_force_n"]]])
    ax = axes[1]
    ax.imshow(values, cmap="YlOrBr", vmin=0, vmax=450, aspect="auto")
    ax.set_xticks((0, 1), ("IK precedente", "IK corrente"))
    ax.set_yticks((0, 1), ("Montaggio precedente", "Montaggio registrato"))
    for row in range(2):
        for column in range(2):
            value = values[row, column]
            ax.text(column, row, f"{value:.3f} N", ha="center", va="center",
                    color="white" if value > 300 else "#222222", weight="bold")
    ax.set_title("2. Effetto della registrazione AB07\nStessa anatomia, pose FK diagnostiche")
    ax.text(.5, -.15, "La precedente preparazione sotto-supportava.\nL'incremento non dimostra un errore di registrazione.", transform=ax.transAxes, ha="center", va="top")

    ax = axes[2]
    x = np.arange(2)
    required, applied, reserve = [], [], []
    for subject in subjects:
        actual = evidence["balance"]["subjects"][subject]["cases"]["virtual_actual_saved"]["aggregate"]
        duration = actual["duration_s"]
        required.append(actual["left_required_with_measured_right_and_no_pelvis_support_Ns"] / duration)
        applied.append(actual["actual_left_impulse_Ns"] / duration)
        reserve.append(actual["actual_pelvis_vertical_reserve_mean_N"])
    ax.bar(x - .22, applied, width=.4, color="#477aa8", label="Contatto sinistro")
    ax.bar(x + .22, reserve, width=.4, color="#8d6092", label="Reserve verticale bacino")
    ax.scatter(x, required, s=90, marker="_", linewidths=3, color="#1e604d", label="Sinistra richiesta dal bilancio", zorder=4)
    for index, value in enumerate(reserve):
        ax.text(x[index] + .22, value + (7 if value > 0 else -7), f"{value:+.1f} N",
                ha="center", va="bottom" if value > 0 else "top")
    ax.axhline(0, color="#999999", linewidth=.8)
    ax.set_xticks(x, subjects)
    ax.set_ylim(-185, 480)
    ax.set_ylabel("Forza verticale media [N]")
    ax.set_title("3. Perché il tracking resta possibile?\nBilancio con peso, GRF destra e ΔCOM")
    ax.legend(loc="upper left", fontsize=8, frameon=False)
    ax.text(.5, -.15, "Contatto + reserve ≈ richiesta di supporto.\nChiusura integrata entro 0,066 N·s.", transform=ax.transAxes, ha="center", va="top")
    fig.suptitle("Diagnostica causale AB06 / AB07 — stessi tre cicli nominali già selezionati", fontsize=15)
    OUTPUT.mkdir(parents=True, exist_ok=True)
    outputs = []
    for extension in ("png", "svg"):
        path = OUTPUT / f"causal_evidence.{extension}"
        fig.savefig(path, dpi=170, facecolor="white")
        outputs.append(fingerprint(path))
    plt.close(fig)
    after = {key: fingerprint(path) for key, path in paths.items()}
    assert before == after
    receipt = {"status": "PASS_RENDERED", "scope": __doc__, "sources": before,
               "inputs_unchanged": True, "outputs": outputs,
               "helper": fingerprint(Path(__file__).resolve())}
    (AUDIT / "figure_rendering.json").write_text(json.dumps(receipt, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(receipt, indent=2))


if __name__ == "__main__":
    main()
