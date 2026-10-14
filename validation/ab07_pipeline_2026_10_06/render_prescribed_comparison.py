"""Render a seventh prescribed diagnostic: healthy, new IK and actual motion.

No policy traces or neural-network outputs are used.  The six other canonical
figures are produced by plot/plotter.py.  This image must live below root plot/.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from validate_bundle import REPO, read_storage, sha256


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--results-dir", required=True, type=Path)
    parser.add_argument("--prefix", default="sim_output")
    parser.add_argument("--reference", required=True, type=Path)
    parser.add_argument("--healthy", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--receipt", type=Path)
    parser.add_argument("--diagnostic-v26", action="store_true",
                        help="Label the figure as a V26 prescribed diagnostic without a learned policy")
    args = parser.parse_args()
    output = args.output.resolve()
    if not output.is_relative_to(REPO / "plot"):
        parser.error("--output must be below the repository's root plot/ directory")
    actual_path = args.results_dir / f"{args.prefix}_kinematics.sto"
    tables = {"actual": read_storage(actual_path), "IK": read_storage(args.reference),
              "healthy": read_storage(args.healthy)}
    time = tables["actual"][1][:, 0]
    series = {}
    for key, (labels, values, degrees) in tables.items():
        if values[0, 0] > time[0] + 1e-8 or values[-1, 0] < time[-1] - 1e-8:
            raise ValueError(f"{key} does not cover the actual motion window")
        scale = 1.0 if degrees else 180.0 / np.pi
        # Match plot/plotter.py joint_sign: knee flexion is -native angle.
        series[key] = {name: np.interp(time, values[:, 0], values[:, i]) * scale
                      * (-1.0 if name in {"pros_knee_angle", "knee_angle_r"} else 1.0)
                       for i, name in enumerate(labels) if i}
    fig, axes = plt.subplots(3, 2, figsize=(14, 10), sharex=True)
    metrics = {}
    for column, (label, prosthetic, biological) in enumerate([
        ("Knee", "pros_knee_angle", "knee_angle_r"),
        ("Ankle", "pros_ankle_angle", "ankle_angle_r"),
    ]):
        axes[0, column].set_title(label)
        for row, coordinate in [(0, prosthetic), (1, biological)]:
            for key, color, style in [("healthy", "0.5", ":"), ("IK", "tab:orange", "--"),
                                       ("actual", "tab:blue", "-")]:
                axes[row, column].plot(time, series[key][coordinate], color=color,
                                       linestyle=style, label={"IK": "New prosthetic-model IK",
                                                             "healthy": "Healthy dataset remap"}.get(key, key.title()))
        actual_error = series["actual"][prosthetic] - series["IK"][prosthetic]
        adaptation = series["IK"][prosthetic] - series["healthy"][prosthetic]
        axes[2, column].plot(time, actual_error, label="Actual − new IK", color="tab:blue")
        axes[2, column].plot(time, adaptation, label="New IK − healthy left", color="tab:orange")
        axes[2, column].axhline(0, linewidth=0.8, color="black", alpha=0.4)
        axes[2, column].set_xlabel("Time [s]")
        metrics[prosthetic] = {"actual_vs_new_ik_rmse_deg": float(np.sqrt(np.mean(actual_error ** 2))),
                               "actual_vs_new_ik_max_abs_deg": float(np.max(np.abs(actual_error))),
                               "new_ik_vs_healthy_rmse_deg": float(np.sqrt(np.mean(adaptation ** 2)))}
        biological_error = series["actual"][biological] - series["IK"][biological]
        metrics[biological] = {"actual_vs_new_ik_rmse_deg": float(np.sqrt(np.mean(biological_error ** 2))),
                               "actual_vs_new_ik_max_abs_deg": float(np.max(np.abs(biological_error)))}
    for row, label in enumerate(["Prosthetic left angle [deg]", "Biological right angle [deg]", "Difference [deg]"]):
        axes[row, 0].set_ylabel(label)
    for ax in axes.flat:
        ax.grid(alpha=0.25)
        ax.legend(fontsize=8)
    title = ("AB07 V26 prescribed diagnostic (no learned policy): healthy, raw IK and actual"
             if args.diagnostic_v26 else "AB07 prescribed validation: healthy dataset, new IK and simulated motion")
    fig.suptitle(title + "\n"
                 "Knee flexion positive (−native coordinate); ankle native sign; angles in degrees")
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=150)
    plt.close(fig)
    receipt = {"status": "PASS", "output": str(output), "sha256": sha256(output),
               "inputs_sha256": {str(p): sha256(p) for p in [actual_path, args.reference, args.healthy]},
               "time_range_s": time[[0, -1]].tolist(), "rows": len(time), "metrics": metrics,
               "display_units": "deg", "display_sign_relative_to_native": {"knee": -1.0, "ankle": 1.0},
               "diagnostic_v26": args.diagnostic_v26,
               "scope": "Prescribed diagnostic; no policy, teacher or training involved"}
    if args.receipt:
        args.receipt.parent.mkdir(parents=True, exist_ok=True)
        args.receipt.write_text(json.dumps(receipt, indent=2, allow_nan=False) + "\n")
    print(json.dumps(receipt, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
