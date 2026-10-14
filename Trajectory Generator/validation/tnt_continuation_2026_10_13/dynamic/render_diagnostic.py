"""Render seven canonical diagnostic figures for one explicitly selected run.

Figure 7 compares the served reference with recorded same-time prescribed
prosthetic kinematics, not a GRF-derived imitation target. Figures 3/4 use
complete minimum-to-minimum cycles of the recorded actual thigh, labelled as
kinematic cycles rather than heel-strike cycles. No simulation is run here.
"""
from __future__ import annotations

import argparse
import importlib.util
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[3]
sys.path.insert(0, str(HERE))
from run_tnt_diagnostic import fingerprint, local_path, serial, write_json


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", required=True, type=Path)
    parser.add_argument("--outdir", required=True, type=Path)
    args = parser.parse_args()
    run, out = args.run.resolve(), args.outdir.resolve()
    if not out.is_relative_to(REPO / "plot"):
        parser.error("The seven canonical plots must live under repository plot/")
    if out.exists():
        parser.error("Plot directory must be new")
    import matplotlib
    matplotlib.use("Agg")
    import numpy as np
    from scipy.signal import find_peaks
    sys.path.insert(0, str(REPO))
    spec = importlib.util.spec_from_file_location("tnt_canonical_plotter", REPO / "plot/plotter.py")
    plotter = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = plotter
    spec.loader.exec_module(plotter)
    receipt_path, trace_path = run / "receipt.json", run / "generator_trace.json"
    receipt = json.loads(receipt_path.read_text(encoding="utf-8"))
    trace = json.loads(trace_path.read_text(encoding="utf-8"))
    if len(trace) < 2:
        raise ValueError("At least two completed samples are needed for diagnostic plots")
    before = [fingerprint(p) for p in sorted(run.rglob("*")) if p.is_file()]
    cfg = plotter.SimulatorConfig(**receipt["effective_simulator_config"])
    tables = plotter.load_tables(run / "sim_outputs", cfg.output_prefix)
    missing = plotter.MissingReport()
    missing.add("Cycle panels use actual-thigh minima, not measured or predicted heel strikes")
    t = np.array([r["time"] for r in trace])
    theta = np.array([r["thigh_actual_deg"] for r in trace])
    peaks, _ = find_peaks(-theta, prominence=5, distance=max(1, int(round(.4 / np.median(np.diff(t))))))
    starts = t[peaks]
    cycles = list(zip(starts[:-1].tolist(), starts[1:].tolist()))
    # Keep only cycles fully covered by every native channel used in cycle plots.
    support = [tables[k] for k in ("states", "sea_torques", "power") if tables[k] is not None]
    if support:
        lo, hi = max(x.time[0] for x in support), min(x.time[-1] for x in support)
        cycles = [(a, b) for a, b in cycles if lo <= a < b <= hi]
    events = {"left": cycles}
    healthy_dir = local_path(cfg.model_bundle_dir) / "data/healthy"
    healthy = plotter.load_healthy_data(healthy_dir, missing)
    outer = plotter.outer_loop_subtitle(cfg)
    inner = plotter.inner_loop_subtitle(plotter.load_sea_params(cfg), cfg)
    original_save = plotter.save_figure

    def labelled_save(fig, destination, filename):
        original_title = fig._suptitle.get_text() if fig._suptitle else ""
        if filename.startswith("07_"):
            original_title = "Served generator vs same-time prescribed prosthetic q (diagnostic only)"
            replacements = {
                "shifted sound leg - policy C2": "prescribed q - served generator",
                "shifted sound leg target": "same-time prescribed prosthetic q",
                "policy C2 served ref": "served generator reference",
                "policy C2 and target": "served generator and prescribed q",
            }
            for ax in fig.axes:
                ylabel = ax.get_ylabel()
                for old, new in replacements.items():
                    ylabel = ylabel.replace(old, new)
                ax.set_ylabel(ylabel)
                for line in ax.lines:
                    label = line.get_label()
                    for old, new in replacements.items():
                        label = label.replace(old, new)
                    line.set_label(label)
                if ax.get_legend() is not None:
                    ax.legend(loc="best", fontsize=8)
            filename = "07_generator_vs_prescribed_error.png"
        if filename.startswith(("03_", "04_")):
            original_title += " | actual-thigh minimum-to-minimum cycles"
            for ax in fig.axes:
                if ax.get_xlabel() == "gait cycle [%]":
                    ax.set_xlabel("actual-thigh cycle [%]")
        fig.suptitle(f"Training-free {receipt['mode']} | {receipt['status']}\n{original_title}", fontsize=12, y=.995)
        for artist, y in zip([x for x in fig.texts if x is not fig._suptitle and x.get_position()[1] > .9], (.943, .921)):
            artist.set_position((artist.get_position()[0], y))
        fig.tight_layout(rect=(0, 0, 1, .865))
        original_save(fig, destination, filename)

    def prescribed_series(rows, coord, side, missing_report):
        values = [(r["time"], r["prescribed_same_time_q_rad"][coord]) for r in rows]
        return np.array([v[0] for v in values]), np.array([v[1] for v in values]) * plotter.figure7_policy_sign(side), "same-time prescribed prosthetic q"

    def no_extrapolation(series, time):
        return np.interp(time, series[0], series[1], left=np.nan, right=np.nan)

    plotter.save_figure = labelled_save
    plotter.shifted_healthy_target_series = prescribed_series
    plotter.interpolate_like = no_extrapolation
    out.mkdir(parents=True)
    plotter.plot_figure_1(tables, out, missing, outer, inner)
    plotter.plot_figure_2(tables, out, missing, outer, inner)
    plotter.plot_figure_3(tables, events, "left", healthy, out, missing, outer, inner)
    plotter.plot_figure_4(tables, events, "left", healthy, out, missing, outer, inner)
    plotter.plot_figure_5(tables, plotter.load_sea_f_opt(cfg, missing), out, missing, outer, inner)
    plotter.plot_figure_6(tables, None, out, missing, outer, inner)
    context = plotter.MlpRolloutContext(run, run / "sim_outputs", receipt_path, trace_path, receipt,
                                      "training-free " + receipt["mode"])
    plotter.plot_figure_7(tables, context, out, missing, outer, inner)
    pngs = sorted(out.glob("*.png"))
    if len(pngs) != 7:
        raise AssertionError(f"Expected exactly seven canonical figures, observed {len(pngs)}")
    if before != [fingerprint(p) for p in sorted(run.rglob("*")) if p.is_file()]:
        raise AssertionError("Finalized run artifacts changed while plotting")
    missing.write(out / "missing_channels.txt")
    write_json(out / "plot_manifest.json", {
        "source_receipt": fingerprint(receipt_path), "source_trace": fingerprint(trace_path),
        "renderer": fingerprint(Path(__file__)), "root_plotter": fingerprint(REPO / "plot/plotter.py"),
        "cycle_definition": "Offline minima of actual thigh; prominence5deg, minsep0.4s; not HS",
        "complete_cycles_s": cycles, "missing_channels": missing.items,
        "figure7_target": "Same-time prescribed prosthetic q, recorded at generator decision times, diagnostic only",
        "files": [fingerprint(p) for p in pngs], "run_artifacts_unchanged": True,
        "visual_review": "pending",
    })
    print(json.dumps(serial({"plot_dir": out, "canonical_figures": len(pngs), "complete_cycles": len(cycles)})))


if __name__ == "__main__":
    main()
