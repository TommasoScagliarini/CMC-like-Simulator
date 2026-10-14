"""Render saved detector geometry diagnostics and per-stance comparisons."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(Path(__file__).resolve().parent))
from analyze_prescribed_v26 import fingerprint, oracle_cycles, stats


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--diagnostic-dir", type=Path,
                        default=REPO / "validation/ab07_detector_investigation_2026_10_06")
    parser.add_argument("--plot-dir", type=Path,
                        default=REPO / "plot/AB07_detector_investigation_2026_10_06")
    args = parser.parse_args()
    source = args.diagnostic_dir / "sensor_pose_comparison.csv"
    groups = {}
    with source.open() as stream:
        for row in csv.DictReader(stream):
            groups.setdefault(row["case"], []).append([float(row[k]) for k in
                           ("time_s", "heel_clearance_m", "toe_clearance_m", "nominal_fz_n")])
    groups = {name: np.asarray(rows) for name, rows in groups.items()}
    oracle = REPO / "results/AB07_prescribed_2026_10_06/sim_output_gait_events.csv"
    cycles, _ = oracle_cycles(oracle, 10.36, 19.369)
    result = {"schema_version": 1, "scope": "Saved-pose scientific plots; no simulation",
              "sources": {"pose_csv": fingerprint(source), "oracle": fingerprint(oracle),
                          "script": fingerprint(Path(__file__))},
              "cases_during_nominal_complete_stances": {}}
    for name, data in groups.items():
        if name.startswith("AB06"):
            continue
        entries = []
        for cycle in cycles:
            keep = (data[:, 0] >= cycle["hs_s"]) & (data[:, 0] < cycle["to_s"])
            q = data[keep]
            if not len(q):
                continue
            contact = q[:, 1:3] <= 0
            entries.append({"nominal_cycle": cycle, "samples": len(q),
                            "heel_contact_fraction": float(np.mean(contact[:, 0])),
                            "toe_contact_fraction": float(np.mean(contact[:, 1])),
                            "any_contact_fraction": float(np.mean(np.any(contact, axis=1))),
                            "heel_clearance_m": stats(q[:, 1]), "toe_clearance_m": stats(q[:, 2])})
        result["cases_during_nominal_complete_stances"][name] = entries
    args.plot_dir.mkdir(parents=True, exist_ok=True)
    figure, axes = plt.subplots(4, 1, figsize=(12.5, 12), constrained_layout=True)
    panels = [
        ("AB06 reference IK, shared 6 Hz filter", "AB06_new_ik_filtered"),
        ("AB07 reference IK, shared 6 Hz filter", "AB07_new_ik_filtered"),
        ("AB07 actual prescribed CMC, measured GRF applied", "AB07_actual_prescribed"),
        ("AB07 actual V26 diagnostic, online left GRF applied", "AB07_actual_v26"),
    ]
    for axis, (title, name) in zip(axes, panels):
        data = groups[name]
        time = data[:, 0]
        axis.fill_between(time, -30, 65, where=data[:, 3] > 20, color="#dbe5ee", alpha=0.65,
                          step="mid", label="Measured left GRF >20 N (nominal support)")
        axis.plot(time, data[:, 1]*1000, color="#1565c0", lw=1.3, label="Heel point")
        axis.plot(time, data[:, 2]*1000, color="#dd6800", lw=1.3, label="Toe point")
        axis.axhline(0, color="#202020", lw=1, label="Frozen detector plane; contact at/below 0")
        axis.set_ylim(-30, 65)
        axis.set_xlim(time[0], time[-1])
        axis.set_ylabel("Signed clearance [mm]")
        axis.set_title(title, loc="left", fontsize=11)
        axis.set_xlabel("Acquisition time [s]")
        axis.grid(alpha=0.2)
    axes[0].legend(loc="upper right", fontsize=8, ncol=2)
    figure.suptitle("Frozen heel/toe detector: geometry before event interpretation\n"
                    "AB06 and AB07 use different acquisition windows; upper excursions above 65 mm are clipped", fontsize=13)
    plot = args.plot_dir / "01_sensor_clearance_and_nominal_support.png"
    figure.savefig(plot, dpi=160)
    plt.close(figure)
    result["plot"] = fingerprint(plot)
    result["limitations"] = [
        "No waveform matching across subjects is implied by these panels.",
        "Blue shading is measured nominal support, not proof of heel contact on the prosthetic hardware.",
        "The plain/V26 panel comparison changes both the reference path and applied GRF; isolated joint counterfactuals are in the geometry receipt.",
        "Clearance is for frozen detector stations, not all physical sole vertices.",
    ]
    out = args.diagnostic_dir / "stance_comparison_and_plot_receipt.json"
    out.write_text(json.dumps(result, indent=2, allow_nan=False)+"\n", encoding="utf-8")
    print(json.dumps({"nominal_complete_cycles": len(cycles), "plot": str(plot), "receipt": str(out)}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
