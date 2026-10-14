"""Plot four explicitly selected finalized saved force traces and event clocks.

No force integration, event re-detection, FK, IK, dynamics or profile fitting.
"""
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import numpy as np

HERE = Path(__file__).resolve().parent
AUDIT = HERE.parent
ROOT = AUDIT.parents[1]
PLOT = ROOT / "plot/AB07_pylon_necessity_screening_2026_10_07"
IDS = ["P4_A1", "P0_M10", "P3_A1", "P2_A0"]


def read(p):
    return json.loads(p.read_text(), parse_constant=lambda s: (_ for _ in ()).throw(ValueError(s)))


def fp(p):
    p = p.resolve()
    return {"path": p.relative_to(ROOT).as_posix(), "sha256": hashlib.sha256(p.read_bytes()).hexdigest()}


def bound(reference):
    p = ROOT / reference["path"]
    if fp(p)["sha256"] != reference["sha256"]:
        raise ValueError("Changed finalized source: " + str(p))
    return p


def storage(p, column):
    with p.open() as f:
        for line in f:
            words = line.split()
            if words and words[0].lower() == "time":
                names = words
                break
        else:
            raise ValueError("Missing storage header")
        if len(names) != len(set(names)) or column not in names:
            raise ValueError("Invalid storage columns")
        rows = np.asarray([[float(x) for x in line.split()] for line in f if line.strip()])
    if rows.ndim != 2 or rows.shape[1] != len(names) or not np.isfinite(rows).all() or np.any(np.diff(rows[:, 0]) <= 0):
        raise ValueError("Invalid saved force table")
    return rows[:, 0], rows[:, names.index(column)]


def main():
    aggregate_path = AUDIT / "phase_timing/aggregate_phase_metrics_v2.json"
    aggregate = read(aggregate_path)
    if aggregate["status"] != "all18_finalized_descriptive_phase_diagnostics" or len(aggregate["cases"]) != 18:
        raise ValueError("Require finalized eighteen-case phase ledger")
    selection_path = ROOT / "Trajectory Generator/validation/paired_prescribed_AB06_AB07_2026_10_07/selection.json"
    chosen = read(selection_path)["subjects"]["AB07"]
    cycles = chosen["primary_cycles"]
    start, end = cycles[0]["hs_s"], cycles[-1]["next_hs_s"]
    sources = [fp(aggregate_path), fp(selection_path), fp(bound(aggregate["phase_protocol"]))]
    fig, axes = plt.subplots(4, 1, figsize=(12.5, 10.5), sharex=True)
    summaries = {}
    for ax, name in zip(axes, IDS):
        phase_path = bound(aggregate["cases"][name]["metrics"])
        phase = read(phase_path)
        if phase["primary_reference"]["cycles"] != cycles or phase["phase_protocol"] != aggregate["phase_protocol"]:
            raise ValueError("Different frozen nominal cycles/protocol")
        receipt_path = bound(phase["receipt"])
        receipt = read(receipt_path)
        numeric_path = bound(phase["swing_impulse_fraction"]["source_metrics"])
        numeric = read(numeric_path)
        if numeric["receipt"]["sha256"] != fp(receipt_path)["sha256"] or numeric["windows"]["primary_all"]["coverage"] != "complete":
            raise ValueError("Require complete matched physical primary support")
        force_path = bound(phase["raw_force_source"])
        measured_path = bound(receipt["source_files"]["prescribed_grf"])
        t, fy = storage(force_path, "left_force_y")
        mt, mfy = storage(measured_path, chosen["left_force_column"])
        if t[0] > start or t[-1] < end or mt[0] > start or mt[-1] < end:
            raise ValueError("Saved trace does not cover exact primary window")
        bw = float(numeric["BW_N"])
        if not np.isfinite(bw) or bw <= 0:
            raise ValueError("Invalid captured body weight")
        actual = (t >= start) & (t <= end)
        measured = (mt >= start) & (mt <= end)
        ax.plot(mt[measured]-start, mfy[measured]/bw, color="#81888E", linewidth=1, alpha=.85)
        ax.plot(t[actual]-start, fy[actual]/bw, color="#28688F", linewidth=1.25)
        for cycle in cycles:
            ax.axvspan(cycle["hs_s"]-start, cycle["to_s"]-start, color="#DDD6C9", alpha=.3)
            ax.axvline(cycle["hs_s"]-start, color="#6E746F", linewidth=.65)
            ax.axvline(cycle["to_s"]-start, color="#6E746F", linestyle="--", linewidth=.65)
        ax.axvline(end-start, color="#6E746F", linewidth=.65)
        ax.axhline(20/bw, color="#A89F95", linestyle=":", linewidth=.8)
        channel = phase["primary_channels"]["primary_force_20N_crossings"]
        if channel["status"] != "fully_observed_primary_window":
            raise ValueError("Phase force support differs from finalized complete window")
        event_summary = {}
        for kind, marker, color in [("heel_strike", "^", "#AF4C37"), ("toe_off", "v", "#724F94")]:
            e = channel["comparison"][kind]["channels"]["onset"]
            event_times = np.asarray(e["observed_times_s"])
            in_window = event_times[(event_times >= start) & (event_times <= end)]
            ax.scatter(in_window-start, np.full(len(in_window), 20/bw), marker=marker, color=color, s=30, zorder=5)
            event_summary[kind] = {"observed_candidate_times_s": e["observed_times_s"], "matching": e["matching"],
                                   "unmatched_complete_guard_s": e["unmatched_reference_fully_observed_matching_guard_s"],
                                   "unmatched_censored_s": e["unmatched_reference_censored_or_unknown_s"]}
        hs, to = event_summary["heel_strike"]["matching"], event_summary["toe_off"]["matching"]
        case = phase["case"]
        ax.set_title(f"{name}: {case['length_mm']:.3f} mm / {case['static_ankle_offset_deg']:+.2f}°    "
                     f"20 N crossings HS {hs['predicted_count']} (matched {hs['matched_count']}), TO {to['predicted_count']} (matched {to['matched_count']})",
                     loc="left", fontsize=10, fontweight="semibold")
        ax.set_ylabel("Vertical force / BW", fontsize=9)
        ax.grid(axis="y", color="#E5E8EA", linewidth=.6)
        ax.spines[["right", "top"]].set_visible(False)
        ax.tick_params(labelsize=8)
        summaries[name] = event_summary
        sources += [fp(p) for p in [phase_path, receipt_path, numeric_path, force_path, measured_path]]
    axes[-1].set_xlabel("Time since first fixed nominal primary HS [s]", fontsize=10)
    axes[-1].set_xlim(0, end-start)
    fig.suptitle("Loaded contact and event timing remain separate diagnostics", y=.99, fontsize=15, fontweight="semibold")
    fig.legend(handles=[Line2D([], [], color="#28688F", label="Saved actual virtual contact"),
                        Line2D([], [], color="#81888E", label="Raw healthy measured GRF"),
                        Line2D([], [], color="#AF4C37", marker="^", linestyle="", label="Recorded force HS onset"),
                        Line2D([], [], color="#724F94", marker="v", linestyle="", label="Recorded force TO onset")],
               loc="upper center", bbox_to_anchor=(.5, .963), ncol=2, fontsize=9, frameon=False)
    fig.text(.07, .021, "Shaded windows: measured nominal stance; solid/dashed boundaries: nominal HS/TO. Crossing markers use the frozen phase JSON, not re-detection.\n"
             "HS/TO inherited onset pairing tolerances: 120/180 ms; closing HS lacks the complete late guard even when matched.\n"
             "These four descriptive cases were selected after the full sweep. Mean-load agreement does not validate phase, mounting or clinical suitability.", fontsize=8.3)
    fig.subplots_adjust(left=.075, right=.985, bottom=.12, top=.865, hspace=.38)
    outputs = []
    for suffix in ["png", "svg"]:
        p = PLOT / ("temporal_contact_tradeoffs."+suffix)
        if p.exists():
            raise FileExistsError(p)
        fig.savefig(p, dpi=220, metadata={"Date": None} if suffix == "svg" else None)
        outputs.append(fp(p))
    sources = list({x["path"]:x for x in sources}.values())
    if any(fp(ROOT / x["path"]) != x for x in sources):
        raise ValueError("Final source changed while plotting")
    result = {"created_utc":datetime.now(timezone.utc).isoformat(),"status":"rendered_pending_visual_review","helper":fp(Path(__file__)),
              "sources":sources,"figures":outputs,"case_ids":IDS,"event_summaries":summaries,"new_native_FK_IK":False,"new_Claude_branch_read":False,
              "method":"Saved native waveform samples only; no boundary extrapolation, force integral or event re-detection. Frozen onset clocks reused."}
    p=PLOT / "temporal_contact_tradeoffs_receipt.json"
    with p.open("x") as f:
        json.dump(result,f,indent=2,allow_nan=False);f.write("\n")
    print(json.dumps({"receipt":fp(p),"figures":outputs}))


if __name__ == "__main__":
    main()
