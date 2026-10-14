"""Render a morphology corridor example using existing episode telemetry.

Requires NumPy and Matplotlib only; it does not load OpenSim or a checkpoint.
Output stays in the same repository plot/ directory as the other examples.
"""
from __future__ import annotations

import json

import render_reward_analysis as plots
import matplotlib.pyplot as plt
import numpy as np


def main():
    plots.OUT.mkdir(parents=True, exist_ok=True)
    plt.rcParams.update({
        "font.family": "DejaVu Sans", "font.size": 11,
        "axes.spines.top": False, "axes.spines.right": False,
        "pdf.fonttype": 42, "svg.fonttype": "none",
    })
    with (plots.RUN / "runtime_diagnostics.jsonl").open(encoding="utf-8") as stream:
        records = [json.loads(line) for line in stream]
    samples = [sample for record in records for sample in record["morphology_causal_samples"]]
    receipt = json.loads(plots.RECEIPT.read_text(encoding="utf-8"))
    time = np.array([sample["time_s"] - receipt["actual_episode_start_s"] for sample in samples])
    fig, axes = plt.subplots(2, 1, figsize=(11, 6.4), sharex=True)
    fig.subplots_adjust(top=.84, bottom=.16, hspace=.35)
    fig.suptitle("Morphology corridor: illustrative recorded episode", fontsize=17, fontweight="bold")
    for ax, joint, title in zip(axes, ["knee", "ankle"], ["Knee", "Ankle"]):
        q = np.rad2deg([sample[f"{joint}_served_ref_rad"] for sample in samples])
        low = np.rad2deg([sample[f"{joint}_min_rad"] for sample in samples])
        high = np.rad2deg([sample[f"{joint}_max_rad"] for sample in samples])
        outside = np.array([sample[f"{joint}_loss"] > 0 for sample in samples])
        ax.fill_between(time, low, high, color=plots.BLUE, alpha=.16, label="Allowed band")
        ax.plot(time, q, color=plots.BLUE, lw=1.8, label="Served reference")
        ax.scatter(time[outside], q[outside], s=9, color=plots.RED, label="Sample outside band", zorder=4)
        ax.set_title(title, loc="left")
        plots.style(ax, "" if joint == "knee" else "Physical sample time since episode start [s]", "Angle [degrees]")
        ax.set_xlim(0, 6.11)
    axes[0].legend(fontsize=9, ncol=3, loc="upper left")
    fig.text(.09, .04, "Only samples with an assigned valid phase are shown.\nThe unevaluated initial interval does not establish that the reference is inside the band.", color=plots.GREY, fontsize=10)
    for extension in ("png", "svg"):
        fig.savefig(plots.OUT / f"05_corridor_served.{extension}", dpi=170, bbox_inches="tight")
    plt.close(fig)


if __name__ == "__main__":
    main()
