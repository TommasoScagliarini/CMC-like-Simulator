"""Render the seven existing canonical figures for one explicitly named case.

No latest-rollout lookup is used. Kinematic reference panels use the recorded
served reference; healthy/source overlays remain diagnostic comparisons.
"""
from __future__ import annotations

import argparse
import csv
import json
import re
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
sys.path.insert(0, str(ROOT / "Trajectory Generator/baseline_MLP"))
import _bootstrap

_bootstrap.ensure_sim_paths()
import win_runtime  # noqa: F401

sys.path.insert(0, str(ROOT / "plot"))
import plotter


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--case", required=True, help="Case directory, e.g. control_smoke")
    args = parser.parse_args()
    if re.fullmatch(r"[A-Za-z0-9_-]+", args.case) is None:
        parser.error("--case must be a single case directory name")
    case = HERE / args.case
    summary_path = case / "rollout_summary.json"
    trace_path = case / "rollout_policy_trace.json"
    results = case / "sim_outputs"
    for path in (summary_path, trace_path, case / "setup.xml", case / "journal.jsonl",
                 results / "rollout_episode_kinematics_reference.sto"):
        if not path.is_file():
            raise FileNotFoundError(path)
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    context = plotter.MlpRolloutContext(
        rollout_dir=case, sim_outputs_dir=results, summary_path=summary_path,
        trace_path=trace_path, summary=summary,
        reward_mode=str(summary.get("reward_config", {}).get("reward_mode", "unknown")),
    )
    tables = plotter.load_tables(results, "rollout_episode")
    time_range = plotter.infer_result_time_range(tables)
    if time_range is None:
        raise ValueError("No recorded time range")
    cycles = set()
    with (case / "journal.jsonl").open(encoding="utf-8") as source:
        for line in source:
            fsm = json.loads(line).get("phase_fsm") or {}
            if float(fsm.get("cycle_completed_this_step", 0.0)) <= 0:
                continue
            end = float(fsm["last_valid_hs_time_s"])
            start = end - float(fsm["last_period_s"])
            if time_range[0] <= start < end <= time_range[1]:
                cycles.add((start, end))
    out = ROOT / "plot/speed_transitions_2026_10_26" / args.case
    out.mkdir(parents=True, exist_ok=False)
    events = out / "validated_fsm_cycles.csv"
    with events.open("w", newline="", encoding="utf-8") as target:
        writer = csv.writer(target)
        writer.writerow(("side", "cycle_start", "cycle_end"))
        writer.writerows(("left", start, end) for start, end in sorted(cycles))
    plotter.latest_mlp_rollout_context = lambda: context
    plotter.next_output_dir = lambda _root: out
    sys.argv = ["plotter.py", "--mlp", "--setup", str(case / "setup.xml"),
                "--results-dir", str(results), "--prefix", "rollout_episode",
                "--out-root", str(out.parent), "--events", str(events),
                "--gait-side", "left"]
    result = plotter.main()
    figures = sorted(path.name for path in out.glob("*.png"))
    if len(figures) != 7:
        raise RuntimeError(f"Expected seven canonical figures, found {len(figures)}")
    (out / "plot_manifest.json").write_text(json.dumps({
        "case": args.case, "rollout_dir": str(case),
        "recorded_time_range_s": list(time_range), "figures": figures,
        "valid_complete_fsm_cycles": len(cycles),
        "served_reference_source": str(results / "rollout_episode_kinematics_reference.sto"),
        "notes": [
            "Selected case explicitly; no latest-rollout lookup.",
            "Gait plots use recorded valid actor-FSM cycles within the recorded interval.",
            "Healthy and original IK overlays are diagnostic, not speed-matched ground truth.",
            "Missing inputs or complete cycles are annotated in missing_channels.txt.",
            "Plot existence does not imply protocol completion or gait success.",
        ],
    }, indent=2) + "\n", encoding="utf-8")
    return result


if __name__ == "__main__":
    raise SystemExit(main())
