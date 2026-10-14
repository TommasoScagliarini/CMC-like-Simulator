"""Scientific figures from finalized saved-data statistics, without OpenSim.

No run discovery, integration, IK, profile fitting or partial-cycle completion.
"""
from __future__ import annotations

import argparse
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
MAIN_IDS = [f"P{i}_A{a}" for i in range(7) for a in range(2)]
POSE_IDS = ["P0_M10", "P0_M5", "P0_M2", "P0_A0", "P0_A1"]
COLORS = ("#28688F", "#C36A29")


def read(path: Path):
    return json.loads(path.read_text(encoding="utf-8"),
                      parse_constant=lambda x: (_ for _ in ()).throw(ValueError(x)))


def fingerprint(path: Path):
    return {"path": path.resolve().relative_to(ROOT).as_posix(),
            "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}


def check_reference(reference):
    p = ROOT / reference["path"]
    actual = fingerprint(p)
    if actual["sha256"] != reference["sha256"]:
        raise ValueError("Finalized source binding changed: " + reference["path"])
    if "size_bytes" in reference and p.stat().st_size != reference["size_bytes"]:
        raise ValueError("Finalized source size changed: " + reference["path"])
    return p, actual


def eligible(row):
    """All plotted comparable metrics require the full same three cycles."""
    return row["complete_primary_cycles"] == 3 and row["primary_support_complete"]


def number(row, name):
    value = row.get(name)
    supported = row.get("reference_support_complete", False) if name == "reference_left_mean_BW" else eligible(row)
    if not supported or value is None:
        return np.nan
    value = float(value)
    if not np.isfinite(value):
        raise ValueError(f"Nonfinite finalized scalar: {row['id']}/{name}")
    return value


def style_axis(ax, title, ylabel):
    ax.set_title(title, loc="left", fontsize=10.5, fontweight="semibold", pad=8)
    ax.set_ylabel(ylabel, fontsize=9)
    ax.grid(axis="both", color="#E5E8EA", linewidth=0.65)
    for name in ("top", "right"):
        ax.spines[name].set_visible(False)
    ax.tick_params(labelsize=8)
    ax.set_axisbelow(True)
    ax.axhline(0, color="#81898E", linewidth=0.7)


def series(ax, rows, metric, label, color, *, linestyle="-", marker="o"):
    return ax.plot([r["length_mm"] for r in rows],
                   [number(r, metric) for r in rows], label=label,
                   color=color, linestyle=linestyle, marker=marker,
                   linewidth=1.6, markersize=4.5, zorder=3)


def benchmark(ax, control, metric):
    value = number(control, metric)
    if np.isfinite(value):
        ax.axhline(value, color="#5E6366", linewidth=1.1, linestyle=(0, (6, 3)))


PANELS = [
    ("A  Left vertical contact", "Mean force / body weight", "left_mean_BW"),
    ("B  Vertical reserve impulse", "Impulse / (body weight × duration)", "reserve_positive_fraction"),
    ("C  Vertical reserve magnitude", "RMS force / body weight", "reserve_rms_BW"),
    ("D  Vertical reserve work", "Absolute work / mass [J/kg]", "reserve_absolute_work_J_per_kg"),
    ("E  Prosthetic knee tracking", "Angle RMS error [deg]", "knee_q_RMS_deg"),
    ("F  Prosthetic ankle tracking", "Angle RMS error [deg]", "ankle_q_RMS_deg"),
    ("G  Pelvis vertical tracking", "Position RMS error [mm]", "pelvis_y_q_RMS_mm"),
    ("H  Vertical impulse closure", "Residual / (body weight × duration)", "closure_fraction"),
    ("I  Contact in nominal swing", "Fraction of left impulse [%]", "swing_impulse_percent"),
]


def main_grid_figure(rows):
    fig, axes = plt.subplots(3, 3, figsize=(13.6, 11.7), constrained_layout=False)
    control = rows["AB06_control"]
    for index, (ax, (title, ylabel, metric)) in enumerate(zip(axes.flat, PANELS)):
        style_axis(ax, title, ylabel)
        for angle, color in enumerate(COLORS):
            group = [rows[f"P{i}_A{angle}"] for i in range(7)]
            label = "Static calibration 0°" if angle == 0 else "Static calibration +4.73065°"
            series(ax, group, metric, label, color)
            if index == 0:
                series(ax, group, "reference_left_mean_BW", "Own filtered reference FK", color,
                       linestyle=":", marker="")
            if index == 1:
                series(ax, group, "reserve_negative_fraction", "Negative reserve impulse", color,
                       linestyle=":", marker="v")
        benchmark(ax, control, metric)
        if index == 1:
            benchmark(ax, control, "reserve_negative_fraction")
        if index == 0:
            values = [number(rows[name], "measured_left_mean_BW") for name in MAIN_IDS]
            value = next((v for v in values if np.isfinite(v)), np.nan)
            if np.isfinite(value):
                ax.axhline(value, color="#9B9790", linewidth=1.0, linestyle="-.")
        ax.set_xlabel("Mount length [mm]", fontsize=9)
        ax.set_xticks([385.89929908923726, 395, 403.4784289483052, 413],
                      ["385.90", "395", "403.48", "413"])
    handles = [Line2D([], [], color=COLORS[0], marker="o", label="Static calibration 0°"),
               Line2D([], [], color=COLORS[1], marker="o", label="Static calibration +4.73065°"),
               Line2D([], [], color="#5E6366", linestyle=(0, (6, 3)), label="AB06 control, own cycles"),
               Line2D([], [], color="#777777", linestyle=":", label="Dotted: reference FK (A), negative impulse (B)"),
               Line2D([], [], color="#9B9790", linestyle="-.", label="Healthy measured AB07 comparator (A)")]
    fig.suptitle("AB07 pylon screening: seven lengths × two coherent preparations", y=0.988,
                 fontsize=15, fontweight="semibold")
    fig.text(0.5, 0.953, "Three fixed nominal cycles · prescribed reference · physical SEA spring/plugin · fixed contact profile",
             ha="center", fontsize=10, color="#4A5257")
    fig.legend(handles=handles, loc="upper center", bbox_to_anchor=(0.5, 0.934),
               ncol=2, fontsize=9, frameon=False)
    missing = [name for name in MAIN_IDS if not eligible(rows[name])]
    footer = ("Work = generalized pelvis force × pelvis speed. Healthy force and virtual hardware are different contact systems. "
              "Dotted FK retains its own complete reference support. No clinical optimum or causal shares.\n" + runtime_note({name: rows[name] for name in MAIN_IDS + ["AB06_control"]}))
    if missing:
        footer += "\nNo common three-cycle point: " + ", ".join(missing) + ". Gaps retain missing data."
    fig.text(0.06, 0.018, footer, va="bottom", fontsize=8.4, color="#4A5257", wrap=True)
    fig.subplots_adjust(left=0.065, right=0.985, bottom=0.11, top=0.84, wspace=0.31, hspace=0.40)
    return fig


def pose_control_figure(rows):
    panels = [PANELS[i] for i in (0, 1, 3, 4, 5, 6)]
    fig, axes = plt.subplots(2, 3, figsize=(13.6, 8.3), constrained_layout=False)
    group = [rows[name] for name in POSE_IDS]
    current = rows["P4_A1"]
    x = [r["static_ankle_offset_deg"] for r in group]
    for index, (ax, (title, ylabel, metric)) in enumerate(zip(axes.flat, panels)):
        pose_title = f"{chr(65 + index)}  {title.split('  ', 1)[1]}"
        style_axis(ax, pose_title, ylabel)
        ax.plot(x, [number(r, metric) for r in group], color="#28688F", marker="o",
                markersize=5.2, linewidth=1.65)
        if index == 0:
            ax.plot(x, [number(r, "reference_left_mean_BW") for r in group],
                    color="#28688F", linestyle=":", linewidth=1.4)
        if index == 1:
            ax.plot(x, [number(r, "reserve_negative_fraction") for r in group],
                    color="#28688F", linestyle=":", marker="v", linewidth=1.4)
        value = number(current, metric)
        if np.isfinite(value):
            ax.axhline(value, color="#C36A29", linewidth=1.2, linestyle=(0, (6, 3)))
        ax.set_xlabel("Static calibration ankle offset [deg]", fontsize=9)
        ax.set_xticks([-10, -5, -2, 0, 4.730648501276871], ["−10", "−5", "−2", "0", "+4.73"])
    fig.suptitle("Proportional AB07 pylon: separately predefined orientation controls", y=0.986,
                 fontsize=14, fontweight="semibold")
    fig.text(0.5, 0.94, "Mount length 385.8993 mm · three negative offsets plus the two main preparations · same nominal cycles",
             ha="center", fontsize=10, color="#4A5257")
    fig.legend(handles=[Line2D([], [], color="#28688F", marker="o", label="Own coherent short-pylon model and full IK"),
                        Line2D([], [], color="#C36A29", linestyle=(0, (6, 3)), label="Current 403.4784 mm / +4.73065° dynamic comparator"),
                        Line2D([], [], color="#777777", linestyle=":", label="Dotted: reference FK (A), negative impulse (B)")],
               loc="upper center", bbox_to_anchor=(0.5, 0.915), fontsize=9, frameon=False)
    missing = [name for name in POSE_IDS if not eligible(rows[name])]
    footer = "Coherent recalibration/IK controls test this preparation and controller. Marker fit does not independently identify hardware pose."
    footer += "\nSuccess would be conditional; failure in three controls cannot prove universal impossibility."
    footer += "\n" + runtime_note({name: rows[name] for name in POSE_IDS + ["P4_A1"]})
    if missing:
        footer += " No common three-cycle point: " + ", ".join(missing) + "."
    fig.text(0.06, 0.018, footer, va="bottom", fontsize=8.4, color="#4A5257")
    fig.subplots_adjust(left=0.065, right=0.985, bottom=0.15, top=0.80, wspace=0.30, hspace=0.39)
    return fig


def runtime_note(rows):
    failed = [name for name, row in rows.items()
              if row["runtime_status"] != "completed" or row["close_error"]]
    if failed:
        result = "Recorded three-cycle physics can be shown despite runtime failure. Non-completed/I/O outcomes: " + ", ".join(failed) + "."
    else:
        result = "All represented runtime receipts completed; each point still requires three fully supported nominal cycles."
    prefix_difference = [name for name, row in rows.items()
                         if row["control_prefix_applicable"] and not row["control_prefix_exact"]]
    if prefix_difference:
        result += "\nRecorded control-prefix differences retained: " + ", ".join(prefix_difference) + "; see finalized diagnostic."
    return result


def q_rms_primary(window, coordinate, requested_unit):
    metric = window["tracking"]["coordinates"][coordinate]["actual_minus_served"]["position"]
    value, unit = float(metric["rms"]), metric["unit"]
    conversion = {("deg", "deg"): 1.0, ("rad", "deg"): 180.0 / np.pi,
                  ("m", "mm"): 1000.0, ("mm", "mm"): 1.0}
    if (unit, requested_unit) not in conversion:
        raise ValueError(f"Tracking unit mismatch: {coordinate} {unit} → {requested_unit}")
    return value * conversion[unit, requested_unit]


def adapt_finalized_metrics(path: Path, reference_main: Path, reference_supplement: Path):
    """Project bound finalized statistics, without re-reading numerical STO."""
    aggregate = read(path)
    expected_ids = set(MAIN_IDS + ["P0_M2", "P0_M5", "P0_M10", "AB06_control"])
    if aggregate["status"] != "all_18_finalized_descriptive_metrics" or set(aggregate["cases"]) != expected_ids:
        raise ValueError("Require exactly all eighteen finalized descriptive outcomes")
    if aggregate["clinical_gate_or_forcefit_used"] or aggregate["new_Claude_branch_read"]:
        raise ValueError("Unreviewed scientific scope of aggregate")
    sources = [fingerprint(path), fingerprint(reference_main), fingerprint(reference_supplement)]
    for field in ("analysis_protocol", "dynamic_protocol"):
        _, actual = check_reference(aggregate[field]); sources.append(actual)
    dynamic = read(ROOT / aggregate["dynamic_protocol"]["path"])
    if set(dynamic["cases"]) != expected_ids:
        raise ValueError("Aggregate differs from prospective case set")
    selection_path = ROOT / "Trajectory Generator/validation/paired_prescribed_AB06_AB07_2026_10_07/selection.json"
    selection = read(selection_path)["subjects"]
    sources.append(fingerprint(selection_path))
    references = {}
    reference_profiles = {}
    expected_B = selection["AB07"]["primary_cycles"]
    expected_knots = sorted({c[k] for c in expected_B for k in ("hs_s", "to_s", "next_hs_s")})
    expected_bounds = [expected_knots[0], expected_knots[-1]]
    for p, expected_status in ((reference_main, "PASS_reference_FK_screening_only"),
                               (reference_supplement, "PASS_supplement_reference_FK_only")):
        reference = read(p)
        if reference["status"] != expected_status or not reference["inputs_integrity"]["all_unchanged"]:
            raise ValueError("Reference FK statistics not finalized/integrity-bound")
        if set(references).intersection(reference["cases"]):
            raise ValueError("Reference cases overlap")
        grid = reference["methodology"]["grid"]
        if grid["primary_s"] != expected_bounds or grid["exact7events_s"] != expected_knots or len(expected_knots) != 7:
            raise ValueError("Reference FK window/events differ from frozen primary selection")
        profile = reference["methodology"].get("fixed_contact_profile", reference["methodology"].get("fixed_profile"))
        if not profile:
            raise ValueError("Reference FK contact profile binding absent")
        _, actual = check_reference(profile); sources.append(actual)
        reference_profiles.update({name: profile["sha256"] for name in reference["cases"]})
        references.update(reference["cases"])
    if set(references) != expected_ids - {"AB06_control"}:
        raise ValueError("All seventeen coherent AB07 reference statistics required")
    rows = {}
    for name, entry in aggregate["cases"].items():
        metrics_path, current = check_reference(entry["metrics"])
        sources.append(current)
        data = read(metrics_path)
        if data["case_id"] != name or data["case"] != dynamic["cases"][name] or data["case"] != entry["case"]:
            raise ValueError("Finalized case identity/prospective binding differs")
        if data["dynamic_protocol"] != aggregate["dynamic_protocol"] or data["analysis_protocol"] != aggregate["analysis_protocol"]:
            raise ValueError("Mixed analysis/runtime protocol in aggregate")
        if data["runtime"] != entry["runtime"] or data["windows"].get("primary_all") != entry["primary_all"]:
            raise ValueError("Aggregate does not contain exact finalized case statistics")
        receipt_path, actual = check_reference(data["receipt"])
        sources.append(actual)
        receipt = read(receipt_path)
        if not receipt.get("finished_utc") or receipt["status"] not in ("completed", "interrupted", "failed"):
            raise ValueError("Active or unrecognized runtime outcome")
        if data["runtime"]["status"] != receipt["status"] or not data["run_artifacts_unchanged"]["passed"]:
            raise ValueError("Runtime outcome/input integrity mismatch")
        windows = data["windows"]
        cycles = selection[entry["subject"]]["primary_cycles"]
        full_cycles = sum(windows.get(f"cycle_{i}", {}).get("coverage") == "complete" for i in range(1, 4))
        window = windows.get("primary_all")
        complete = bool(window and window["coverage"] == "complete" and data.get("numerical_data_finite"))
        row = {"id": name, "subject": entry["subject"], "length_mm": entry["case"]["length_mm"],
               "static_ankle_offset_deg": entry["case"]["static_ankle_offset_deg"],
               "runtime_status": receipt["status"], "close_error": receipt.get("close_error"),
               "complete_primary_cycles": full_cycles, "primary_support_complete": complete,
               "reference_support_complete": False,
               "analysis_status": data["analysis_status"],
               "control_prefix_applicable": data["prefix_comparison"]["applicable"],
               "control_prefix_exact": data["prefix_comparison"].get("all_available_tables_exact_numeric_prefix", False)}
        for _, _, metric in PANELS:
            row[metric] = None
        row.update(reference_left_mean_BW=None, measured_left_mean_BW=None, reserve_negative_fraction=None)
        if name != "AB06_control":
            reference = references[name]
            if reference["model"] != entry["case"]["model"] or reference["full_IK"] != entry["case"]["IK"]:
                raise ValueError("Reference FK is not the same coherent model/full IK")
            if receipt["profile_association"]["online_grf_profile_file"]["sha256"] != reference_profiles[name]:
                raise ValueError("Reference FK/runtime fixed contact profiles differ")
            reference_bw = data.get("BW_N")
            if reference_bw is not None and float(reference_bw) > 0 and np.isfinite(float(reference_bw)):
                row["reference_left_mean_BW"] = reference["contact"]["aggregate"]["mean_force_N"] / float(reference_bw)
                row["reference_support_complete"] = True
        if eligible(row):
            expected = [cycles[0]["hs_s"], cycles[-1]["next_hs_s"]]
            if window["requested_range_s"] != expected or window["observed_intersection_range_s"] != expected:
                raise ValueError("Three-cycle window differs from frozen exact selection")
            if window["boundary_clamping_or_extrapolation_used"]:
                raise ValueError("Extrapolated primary window")
            duration, bw, mass = window["requested_duration_s"], float(data["BW_N"]), float(data["mass_kg"])
            if duration <= 0 or bw <= 0 or mass <= 0:
                raise ValueError("Invalid duration/mass/body weight")
            reserve = window["reserves"]["per_coordinate"]["pelvis_ty"]
            if reserve["unit"] != "N" or reserve["impulse_unit"] != "Ns":
                raise ValueError("Vertical reserve must be a force with force impulse")
            row.update(left_mean_BW=window["forces"]["left_online"]["mean_BW"],
                       measured_left_mean_BW=window["forces"]["left_measured"]["mean_BW"],
                       reserve_positive_fraction=reserve["impulse"]["positive"] / (bw * duration),
                       reserve_negative_fraction=reserve["impulse"]["negative"] / (bw * duration),
                       reserve_rms_BW=reserve["force_or_torque"]["rms_BW"],
                       reserve_absolute_work_J_per_kg=reserve["work_J_per_kg"]["absolute"],
                       knee_q_RMS_deg=q_rms_primary(window, "pros_knee_angle", "deg"),
                       ankle_q_RMS_deg=q_rms_primary(window, "pros_ankle_angle", "deg"),
                       pelvis_y_q_RMS_mm=q_rms_primary(window, "pelvis_ty", "mm"))
            balance = window["vertical_balance"]
            if balance["available"]:
                row["closure_fraction"] = balance["closure_residual_over_BW_duration"]
            total = window["forces"]["left_online"]["impulse_Ns"]
            swings = [windows[f"cycle_{i}_swing"] for i in range(1, 4)]
            if total != 0 and all(w["coverage"] == "complete" for w in swings):
                row["swing_impulse_percent"] = 100 * sum(w["forces"]["left_online"]["impulse_Ns"] for w in swings) / total
        rows[name] = row
    sources = list({s["path"]: s for s in sources}.values())
    provenance = {"data": "Exact finalized aggregate and per-case metric JSON; scalar normalization/projection only, no numerical STO reanalysis",
                  "reference": "Finalized coherent own-model/full-IK fixed-profile FK, separate from forward dynamic contact",
                  "phase": "Same three frozen measured nominal cycles per subject; runtime status independent of physical recorded support",
                  "work": "Absolute integral of actual pelvis generalized force × actual pelvis speed, normalized by own model mass",
                  "tracking": "Actual minus actually served reference, unit fields validated; already-deg metrics never converted twice",
                  "new_Claude_branch_read": False}
    return rows, sources, provenance


def render(metrics_path: Path, reference_main: Path, reference_supplement: Path):
    rows, source_fingerprints, provenance = adapt_finalized_metrics(metrics_path, reference_main, reference_supplement)
    if set(rows) != set(MAIN_IDS + ["P0_M2", "P0_M5", "P0_M10", "AB06_control"]):
        raise ValueError("All eighteen finalized outcomes must be represented")
    plt.rcParams.update({"font.family": "DejaVu Sans", "svg.fonttype": "none",
                         "axes.formatter.use_mathtext": True})
    PLOT.mkdir(parents=True, exist_ok=True)
    receipt_path = PLOT / "aggregate_figure_receipt.json"
    if receipt_path.exists():
        raise FileExistsError("Preserve existing aggregate figure receipt")
    outputs = []
    for name, figure in [("main_grid_physical_screening", main_grid_figure(rows)),
                         ("proportional_pylon_pose_controls", pose_control_figure(rows))]:
        for suffix in ("png", "svg"):
            target = PLOT / f"{name}.{suffix}"
            if target.exists():
                raise FileExistsError("Preserve existing figure: " + str(target))
            metadata = {"Creator": "Codex saved-data scientific figures", "Date": None} if suffix == "svg" else {"Software": "Codex saved-data scientific figures"}
            figure.savefig(target, dpi=220, metadata=metadata)
            outputs.append(fingerprint(target))
        plt.close(figure)
    for source in source_fingerprints:
        if fingerprint(ROOT / source["path"]) != source:
            raise AssertionError("Finalized statistics/source changed while rendering")
    receipt = {"created_utc": datetime.now(timezone.utc).isoformat(),
               "status": "rendered_pending_visual_review", "helper": fingerprint(Path(__file__)),
               "input_statistic_JSON": fingerprint(metrics_path), "sources": source_fingerprints,
               "provenance": provenance, "figures": outputs,
               "displayed_scalar_rows": rows,
               "limitations": ["All plotted actual comparable points use three supported nominal cycles; missing outcomes remain missing.",
                               "Dotted reference FK is independently available on the same complete nominal window even when actual dynamics does not reach it.",
                               "No force fitting, dynamics, FK, clinical optimization or causal percentages performed by this plotter."]}
    with receipt_path.open("x", encoding="utf-8") as stream:
        json.dump(receipt, stream, indent=2, ensure_ascii=False, allow_nan=False)
        stream.write("\n")
    print(json.dumps({"receipt": fingerprint(receipt_path), "figures": outputs}, ensure_ascii=False))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--metrics", type=Path, required=True, help="Exact finalized aggregate_metrics.json; no implicit latest-run lookup")
    parser.add_argument("--reference-main", type=Path, required=True, help="Finalized main14 coherent reference-FK statistic JSON")
    parser.add_argument("--reference-supplement", type=Path, required=True, help="Finalized supplementary3 coherent reference-FK statistic JSON")
    args = parser.parse_args()
    render(args.metrics.resolve(), args.reference_main.resolve(), args.reference_supplement.resolve())


if __name__ == "__main__":
    main()
