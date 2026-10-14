"""Render the seven canonical figures for one explicit unified 39D rollout.

Reuse the simulator plotter without its legacy latest-rollout discovery.
Canonical figures live under the repository's root plot/ directory.
Figure 7 reads the recorded imitation target; its labels must identify the
left phase template, while the middle row remains the raw sound leg.
"""
from __future__ import annotations

import argparse
import importlib.util
import json
from pathlib import Path
import sys


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rollout", type=Path, required=True)
    args = parser.parse_args()
    rollout = args.rollout.resolve()
    summary = json.loads((rollout / "rollout_summary.json").read_text())
    contract = json.loads(
        (Path(summary["checkpoint"]) / "unified_contract.json").read_text()
    )
    if contract["target_channels"]["teacher_source"] != "prescribed_left_phase_template":
        raise ValueError("Expected the prescribed-left phase template contract")

    repo = Path(__file__).resolve().parents[3]
    spec = importlib.util.spec_from_file_location(
        "cmc_canonical_plotter", repo / "plot" / "plotter.py"
    )
    plotter = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = plotter
    spec.loader.exec_module(plotter)
    context = plotter.valid_mlp_rollout_context(rollout)
    if context is None:
        raise ValueError(f"Incomplete rollout: {rollout}")

    from matplotlib.text import Text

    original_save = plotter.save_figure
    original_discovery = plotter.latest_mlp_rollout_context
    saved = []
    figure7_labels = []

    def save_figure(fig, out_dir, filename):
        if filename.startswith("07_"):
            # Values already come from imitation_target_q in this exact trace.
            # Only historical labels refer incorrectly to a shifted right leg.
            for label in fig.findobj(match=Text):
                text = label.get_text()
                text = text.replace("Shifted Sound Leg Target", "Prescribed Left Phase Target")
                text = text.replace("shifted sound leg target", "prescribed left phase target")
                text = text.replace("shifted sound leg - policy C2", "left phase target - policy C2")
                label.set_text(text)
                if text:
                    figure7_labels.append(text)
        original_save(fig, out_dir, filename)
        saved.append(str(out_dir / filename))

    old_argv = sys.argv
    try:
        plotter.latest_mlp_rollout_context = lambda: context
        plotter.save_figure = save_figure
        sys.argv = [
            str(repo / "plot" / "plotter.py"), "--mlp",
            "--results-dir", str(context.sim_outputs_dir),
            "--prefix", "rollout_episode",
            "--setup", str(summary["setup_xml_path"]),
            "--gait-side", "left",
            "--out-root", str(repo / "plot"),
        ]
        result = plotter.main()
    finally:
        sys.argv = old_argv
        plotter.save_figure = original_save
        plotter.latest_mlp_rollout_context = original_discovery
    if len(saved) != 7:
        raise RuntimeError(f"Expected seven canonical figures, got {len(saved)}")
    receipt = {
        "rollout": str(rollout), "checkpoint": summary["checkpoint"],
        "teacher_source": contract["target_channels"]["teacher_source"],
        "gait_side": "left", "figures": saved,
        "figure7_labels": figure7_labels,
        "figure7_middle_row": "raw sound leg, for context only",
        "figure7_target": "recorded imitation_target_q (prescribed left phase template)",
    }
    receipt_text = json.dumps(receipt, indent=2) + "\n"
    (rollout / "plots").mkdir(parents=True, exist_ok=True)
    (rollout / "plots" / "plot_receipt.json").write_text(receipt_text)
    (Path(saved[0]).parent / "plot_receipt.json").write_text(receipt_text)
    return result


if __name__ == "__main__":
    raise SystemExit(main())
