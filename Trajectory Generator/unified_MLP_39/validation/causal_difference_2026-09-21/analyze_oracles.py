"""Compare direct-target diagnostic rollouts; never train or alter runtime code."""
from __future__ import annotations

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
sys.path.insert(0, str(TG))
os.environ.setdefault("MPLCONFIGDIR", str(HERE / ".matplotlib"))
os.environ.setdefault("OMP_NUM_THREADS", "1")

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def read(path):
    return json.loads(path.read_text())


def load_helpers():
    path = PACKAGE / "validation/ablation_report_2026-09-20/analyze_results.py"
    spec = importlib.util.spec_from_file_location("rollout_helpers", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def main():
    helpers = load_helpers()
    data = {"arms": {}, "limitations": [
        "Direct endpoints still pass through the original reference governor and SEA; this is not perfect kinematic replay.",
        "One fixed physical start and episode, no action noise, no neural policy and no training.",
        "This isolates a target substitution in the current physical setup, not all changes since August.",
        "Right-arm observation metadata belongs to the left-phase 39D scaffold; no network uses it.",
    ]}
    traces = {}
    for arm in ("left", "right"):
        folder = PACKAGE / f"runs/rollout/causal_difference_2026-09-21_oracle_{arm}"
        intervention = read(folder / "intervention.json")
        assert intervention["diagnostic_only"] and not intervention["learned_policy_used"]
        assert intervention["reference_trace_match"] and intervention["fsm"] == "v3"
        metrics, trace, summary = helpers.rollout_metrics(folder)
        refs = read(folder / "oracle_reference.json")
        assert len(refs) == len(trace)
        for ref, row in zip(refs, trace):
            np.testing.assert_allclose(ref["time_s"], row["time"], atol=1e-9)
            for j in helpers.JOINTS:
                np.testing.assert_allclose(ref["q"][j], row["imitation_target_q"][j], atol=1e-8)
            np.testing.assert_allclose(row["raw_policy_action"], ref["action"], atol=1e-8)
        metrics["intervention"] = intervention
        metrics["contact"] = {
            "penalized_steps": sum(row["reward_terms"]["grf_ankle_moment_flip_loss"] > 0 for row in trace),
            "min_moment_nm": min(row["reward_terms"]["grf_ankle_moment_flip_tau_nm"] for row in trace),
            "max_normal_force_n": max(row["reward_terms"]["grf_ankle_moment_flip_force_n"] for row in trace),
        }
        data["arms"][arm] = metrics
        traces[arm] = trace
    length = min(map(len, traces.values()))
    data["common_prefix"] = {"steps": length, "duration_s": length * .01,
        "joints": {arm: {j: helpers.joint_metrics(trace[:length], j) for j in helpers.JOINTS}
                   for arm, trace in traces.items()}}
    fig, axes = plt.subplots(4, 2, figsize=(14, 12), layout="constrained", sharex=True, sharey="row")
    for col, arm in enumerate(("left", "right")):
        trace = traces[arm]
        time = np.array([r["time"] for r in trace]) - 12.99
        for row, joint in enumerate(helpers.JOINTS):
            for field, key, label, color, style in (
                ("imitation_target_q", joint, "Target", "black", "--"),
                ("prosthetic_state", joint + "_served_ref", "Reference dopo governor", "#d97706", ":"),
                ("prosthetic_state", joint, "Giunto", "#2563eb", "-"),
            ):
                axes[row, col].plot(time, np.rad2deg(helpers.array(trace, field, key)), color=color, linestyle=style, label=label)
        axes[0, col].set_title("Target sinistro, fase sinistra" if arm == "left" else "Target storico destro, sfasato")
        axes[2, col].plot(time, [r["reward_terms"]["grf_ankle_moment_flip_tau_nm"] for r in trace], color="#2563eb")
        axes[2, col].axhline(-8, color="black", linestyle="--", label="Soglia sotto carico ≥50 N")
        axes[3, col].plot(time, [r["reward_terms"]["grf_ankle_moment_flip_force_n"] for r in trace], color="#2563eb")
        axes[3, col].set_xlabel("Tempo dall'inizio episodio (s)")
        for ax in axes[:, col]:
            ax.grid(alpha=.2)
    for ax, label in zip(axes[:, 0], ("Ginocchio (°)", "Caviglia (°)", "Momento GRF (Nm)", "Carico normale (N)")):
        ax.set_ylabel(label)
    axes[0, 0].legend(fontsize=8)
    fig.suptitle("Sostituzione controllata del target: stessi governor, controllori e FSM v3; nessuna rete")
    fig.savefig(HERE / "oracle_comparison.png", dpi=150)
    plt.close(fig)
    (HERE / "oracle_metrics.json").write_text(json.dumps(data, indent=2, allow_nan=False) + "\n")
    hashes = helpers.INPUT_HASHES.copy()
    hashes[str(Path(__file__).resolve().relative_to(REPO))] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    (HERE / "oracle_input_hashes.json").write_text(json.dumps(hashes, indent=2) + "\n")
    print(json.dumps(data, indent=2))


if __name__ == "__main__":
    main()
