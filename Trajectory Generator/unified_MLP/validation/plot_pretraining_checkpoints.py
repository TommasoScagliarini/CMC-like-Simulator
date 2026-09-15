"""Plot aligned post-step tracking for the deterministic iteration 3/5 comparison.

From the repository root, with matplotlib installed:
    python "Trajectory Generator/unified_MLP/validation/plot_pretraining_checkpoints.py"
Paths can be overridden on macOS and Windows with --rollout-root/--output-dir.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D


UNIFIED_DIR = Path(__file__).resolve().parents[1]
STAGES = (3, 5)
STARTS = ("early", "center", "late")
JOINTS = (("pros_knee_angle", "Ginocchio"), ("pros_ankle_angle", "Caviglia"))
COLORS = {3: "#2765a8", 5: "#c55b28"}


def load_case(root: Path, iteration: int, start: str, date: str) -> dict:
    directory = root / f"pretraining_v1_pen1_iter{iteration}_{start}_{date}"
    summary = json.loads((directory / "rollout_summary.json").read_text(encoding="utf-8"))
    trace = json.loads((directory / "rollout_policy_trace.json").read_text(encoding="utf-8"))
    if not summary.get("ok") or summary.get("action_selection") != "deterministic":
        raise ValueError(f"Not a successful deterministic rollout: {directory}")
    if not trace or len(trace) != summary["steps"]:
        raise ValueError(f"Missing or incomplete trace: {directory}")
    initial_time = float(trace[0]["policy_segment_times"][0])
    times = [float(row["time"]) - initial_time for row in trace]
    if not all(math.isfinite(t) for t in times) or any(b <= a for a, b in zip(times, times[1:])):
        raise ValueError(f"Invalid time axis: {directory}")
    signals = {}
    for joint, _ in JOINTS:
        signals[joint] = {
            "target": [math.degrees(float(row["imitation_target_q"][joint])) for row in trace],
            "actual": [math.degrees(float(row["prosthetic_state"][joint])) for row in trace],
            "served": [math.degrees(float(row["prosthetic_state"][joint + "_served_ref"])) for row in trace],
        }
        if not all(math.isfinite(value) for series in signals[joint].values() for value in series):
            raise ValueError(f"Nonfinite joint signal: {directory} / {joint}")
    return {"summary": summary, "times": times, "signals": signals}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rollout-root", type=Path, default=UNIFIED_DIR / "runs" / "rollout")
    parser.add_argument("--output-dir", type=Path, default=UNIFIED_DIR / "runs" / "validation" / "pretraining_v1_pen1_compare_3_5_2026-09-14")
    parser.add_argument("--date", default="2026-09-14")
    args = parser.parse_args()
    cases = {(iteration, start): load_case(args.rollout_root, iteration, start, args.date)
             for iteration in STAGES for start in STARTS}
    for start in STARTS:
        left, right = (cases[(iteration, start)] for iteration in STAGES)
        if left["summary"]["episode_start_offset_s"] != right["summary"]["episode_start_offset_s"]:
            raise ValueError(f"Different start offsets: {start}")
        common = min(len(left["times"]), len(right["times"]))
        for index in range(common):
            if abs(left["times"][index] - right["times"][index]) > 1e-7:
                raise ValueError(f"Time grids differ: {start}")
            for joint, _ in JOINTS:
                if abs(left["signals"][joint]["target"][index] - right["signals"][joint]["target"][index]) > 1e-6:
                    raise ValueError(f"Prescribed targets differ at the same time: {start}")

    plt.rcParams.update({"font.size": 10, "axes.spines.top": False, "axes.spines.right": False})
    fig, axes = plt.subplots(3, 2, figsize=(13, 10), sharex=True, sharey="col")
    fig.subplots_adjust(left=0.08, right=0.98, bottom=0.145, top=0.845, hspace=0.43, wspace=0.16)
    fig.suptitle("Pretraining imitativo: checkpoint 3 e 5", x=0.08, y=0.975, ha="left", fontsize=18, weight="bold")
    fig.text(0.08, 0.93, "Rollout deterministici AB06 · prescribed sinistro · stessa fisica e stessi start", color="#444444")
    legend = [Line2D([], [], color="#444444", lw=2, label="Prescribed sinistro")]
    for iteration in STAGES:
        legend.extend((Line2D([], [], color=COLORS[iteration], lw=1.8, label=f"Checkpoint {iteration}: actual"),
                       Line2D([], [], color=COLORS[iteration], lw=1.3, ls="--", label=f"Checkpoint {iteration}: served")))
    fig.legend(handles=legend, loc="upper left", bbox_to_anchor=(0.073, 0.914), ncol=3, frameon=False)
    reasons = {"episode_time_limit": "limite 5 s", "phase_timeout:swing": "timeout swing", "phase_timeout:stance": "timeout stance", "grf_penetration": "penetrazione", "grf_penetration_hard": "penetrazione hard"}
    for row_index, start in enumerate(STARTS):
        pair = [cases[(iteration, start)] for iteration in STAGES]
        reference = max(pair, key=lambda case: len(case["times"]))
        for column, (joint, joint_name) in enumerate(JOINTS):
            ax = axes[row_index, column]
            ax.plot(reference["times"], reference["signals"][joint]["target"], color="#444444", lw=2, zorder=2)
            for iteration, case in zip(STAGES, pair):
                ax.plot(case["times"], case["signals"][joint]["actual"], color=COLORS[iteration], lw=1.8, zorder=4)
                ax.plot(case["times"], case["signals"][joint]["served"], color=COLORS[iteration], lw=1.3, ls="--", alpha=0.85, zorder=3)
                if case["times"][-1] < 4.995:
                    ax.axvline(case["times"][-1], color=COLORS[iteration], lw=0.9, ls=":", alpha=0.8)
            ax.set_title(f"{start.capitalize()} · {joint_name}", loc="left", fontsize=11, weight="bold")
            ax.set_ylabel("Angolo [°]")
            ax.set_xlim(0, 5)
            ax.grid(alpha=0.18)
            if row_index == 2:
                ax.set_xlabel("Tempo dal reset [s]")
        footer = []
        for iteration, case in zip(STAGES, pair):
            summary = case["summary"]
            reason = reasons.get(summary["end_reason"], summary["end_reason"])
            footer.append(f"C{iteration}: {case['times'][-1]:.2f} s, {reason}, {summary['phase_valid_cycle_count']} cicli validi")
        footer_y = -0.35 if row_index == 2 else -0.20
        axes[row_index, 0].text(0, footer_y, "  |  ".join(footer), transform=axes[row_index, 0].transAxes, fontsize=8, color="#444444")
    fig.text(0.08, 0.035, "Target, actual e served sono allineati al tempo post-step. Le curve terminano con l'episodio; nessuna estrapolazione oltre lo stop.", fontsize=9, color="#555555")
    args.output_dir.mkdir(parents=True, exist_ok=True)
    for suffix in ("png", "pdf"):
        path = args.output_dir / f"tracking_angles.{suffix}"
        fig.savefig(path, dpi=170, facecolor="white")
        print(path)
    plt.close(fig)


if __name__ == "__main__":
    main()
