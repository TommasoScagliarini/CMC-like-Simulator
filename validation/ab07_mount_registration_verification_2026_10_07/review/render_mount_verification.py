"""Render the completed mount/marker/IK audit from saved JSON only.

No OpenSim import, FK, IK, inference, dynamics or edits to source artifacts.
The figure and receipt are written exclusively under root plot/.
"""
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import tempfile

os.environ.setdefault("MPLCONFIGDIR", str(Path(tempfile.gettempdir()) / "cmc_ab07_mount_plot_cache"))
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

HERE = Path(__file__).resolve().parent
AUDIT = HERE.parent
ROOT = AUDIT.parents[1]
OUTPUT = ROOT / "plot/AB07_mount_registration_verification_2026_10_07"


def read(path):
    return json.loads(path.read_text(), parse_constant=lambda value: (_ for _ in ()).throw(ValueError(value)))


def fingerprint(path):
    return {"path": path.relative_to(ROOT).as_posix(), "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}


def main():
    comparison_path = AUDIT / "variant_contact_comparison.json"
    constraints_path = AUDIT / "constraints/static_geometry_constraints.json"
    healthy_path = ROOT / "validation/ab06_ab07_independent_attribution_2026_10_07/healthy_ik/supplemental_healthy_geometry.json"
    paths = [comparison_path, constraints_path, healthy_path, Path(__file__)]
    before = {p.relative_to(ROOT).as_posix(): fingerprint(p)["sha256"] for p in paths}
    comparison, constraints, healthy = read(comparison_path), read(constraints_path), read(healthy_path)
    if comparison["status"] != "PASS_reference_preparation_FK_only" or constraints["status"] != "PASS" or healthy["status"] != "PASS":
        raise ValueError("Render only finalized, passed source artifacts")
    if not comparison["inputs_integrity"]["all_unchanged"] or not constraints["inputs_integrity"]["all_unchanged"] or not healthy["inputs_integrity"]["unchanged"]:
        raise ValueError("Source integrity not confirmed")

    ids = [["L0_A0", "L0_A1"], ["L1_A0", "L1_A1"]]
    scalar = comparison["scalar_comparison"]
    force = np.array([[scalar[name]["contact_mean_force_N"] for name in row] for row in ids])
    height = 1000 * np.array([[scalar[name]["ankle_center_Y_mean_stance_m"] for name in row] for row in ids])
    cases = comparison["cases"]
    lengths = [1000 * cases[name]["case"]["distal_mount_length_m"] for name in ("L0_A0", "L1_A0")]
    angle = cases["L1_A1"]["case"]["static_ankle_offset_deg"]
    sensitivity = constraints["AB07_target_sensitivity"]
    expected = ["baseline", "donor_pitch_minus2deg", "donor_pitch_plus2deg", "donor_height_minus5mm", "donor_height_plus5mm"]
    if [row["case"] for row in sensitivity] != expected:
        raise ValueError("Unexpected declared target sensitivity cases")
    extra = 1000 * np.array([row["extra_distal_length_m"] for row in sensitivity])
    healthy_contrast = healthy["subject_difference_budgets"]["stance"]["fresh_healthy"]["joint_center_xyz_m_time_weighted_mean"]["healthy_AB07_minus_AB06"][1]
    prepared_A_Y = healthy["subjects"]["AB06"]["cases"]["prepared_current"]["aggregate"]["stance"]["joint_center_Y_time_weighted_mean_m"]
    contrasts = 1000 * np.array([healthy_contrast,
                                scalar["L0_A0"]["ankle_center_Y_mean_stance_m"] - prepared_A_Y,
                                scalar["L1_A1"]["ankle_center_Y_mean_stance_m"] - prepared_A_Y])
    if not np.isfinite(force).all() or not np.isfinite(height).all() or not np.isfinite(extra).all() or not np.isfinite(contrasts).all():
        raise ValueError("Nonfinite figure data")

    OUTPUT.mkdir(parents=True, exist_ok=True)
    png = OUTPUT / "mount_registration_2x2_and_target_sensitivity.png"
    svg = png.with_suffix(".svg")
    receipt = OUTPUT / "figure_receipt.json"
    if any(path.exists() for path in (png, svg, receipt)):
        raise FileExistsError("Preserve previous figure artifacts")
    plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 10,
                         "axes.spines.top": False, "axes.spines.right": False,
                         "svg.fonttype": "none", "savefig.facecolor": "white"})
    fig = plt.figure(figsize=(14, 11), facecolor="white")
    grid = fig.add_gridspec(2, 2, left=.08, right=.94, top=.825, bottom=.22,
                           wspace=.42, hspace=.82)
    axes = [fig.add_subplot(grid[i // 2, i % 2]) for i in range(4)]

    def cells(ax, values, title, cmap, precision):
        view = ax.imshow(values, cmap=cmap, aspect="equal")
        ax.set_xticks([0, 1], ["0°", f"+{angle:.3f}°"])
        ax.set_yticks([0, 1], [f"Short\n{lengths[0]:.3f} mm", f"Long\n{lengths[1]:.3f} mm"])
        ax.set_xlabel("Static ankle calibration offset", labelpad=10)
        ax.set_title(title, loc="left", fontweight="bold", pad=15)
        for i in range(2):
            for j in range(2):
                value = values[i, j]
                fraction = (value - values.min()) / max(float(np.ptp(values)), 1e-12)
                color = "white" if fraction < .55 else "#151515"
                text = f"{value:.{precision}f}"
                if (i, j) == (1, 1):
                    text += "\nCurrent"
                ax.text(j, i, text, ha="center", va="center", color=color,
                        fontsize=12, fontweight="bold", linespacing=1.6)
        ax.set_xticks(np.arange(-.5, 2, 1), minor=True)
        ax.set_yticks(np.arange(-.5, 2, 1), minor=True)
        ax.grid(which="minor", color="white", linewidth=3)
        ax.tick_params(which="minor", length=0)
        fig.colorbar(view, ax=ax, shrink=.75, pad=.04)

    cells(axes[0], force, "A  Mean left contact force (N)", "viridis", 3)
    cells(axes[1], height, "B  Stance ankle-center Y (mm)", "cividis", 3)
    ax = axes[2]
    labels = ["Donor\nbaseline", "Pitch\n−2°", "Pitch\n+2°", "Clearance\n−5 mm", "Clearance\n+5 mm"]
    bars = ax.bar(range(5), extra, color=["#304d6d"] + ["#aab7c4"] * 4, width=.68)
    ax.axhline(extra[0], color="#304d6d", linestyle="--", linewidth=1, zorder=0)
    ax.set_xticks(range(5), labels)
    ax.set_ylabel("Extra distal mount length (mm)")
    ax.set_title("C  Sensitivity to donor targets", loc="left", fontweight="bold", pad=15)
    ax.set_ylim(min(0., float(extra.min()) * 1.2), max(float(extra.max()) * 1.20, 1.))
    ax.grid(axis="y", color="#e6e6e6", linewidth=.7)
    ax.set_axisbelow(True)
    for bar, value in zip(bars, extra):
        ax.annotate(f"{value:.3f}", (bar.get_x() + bar.get_width() / 2, value),
                    xytext=(0, 5 if value >= 0 else -12), textcoords="offset points", ha="center", fontsize=10)

    ax = axes[3]
    bars = ax.bar(range(3), contrasts, width=.62, color=["#788f76", "#a7b69b", "#bd704d"])
    ax.axhline(0, color="#666666", linewidth=1)
    ax.set_xticks(range(3), ["Healthy\nfresh IK", "Prepared\nshort, 0°", "Prepared\nlong, +offset"])
    ax.set_ylabel("AB07 − AB06 stance ankle-center Y (mm)")
    ax.set_title("D  Subject height contrast", loc="left", fontweight="bold", pad=15)
    ax.set_ylim(-13, 14)
    ax.grid(axis="y", color="#e6e6e6", linewidth=.7)
    ax.set_axisbelow(True)
    for bar, value in zip(bars, contrasts):
        ax.annotate(f"{value:+.6f}", (bar.get_x() + bar.get_width() / 2, value),
                    xytext=(0, 6 if value >= 0 else -16), textcoords="offset points", ha="center", fontsize=10)
    ax.text(.5, -.30,
            f"Short prepared − healthy contrast: {contrasts[1] - contrasts[0]:+.6f} mm",
            transform=ax.transAxes, ha="center", fontsize=9, color="#444444")

    start, end = comparison["methodology"]["grid"]["primary_s"]
    fig.suptitle("AB07 mount registration: preparation response and target dependence",
                 x=.08, y=.96, ha="left", fontsize=16, fontweight="bold")
    fig.text(.08, .90,
             f"Reference FK • own freshly fitted 12 markers + complete IK per cell • fixed contact profile • "
             f"three frozen measured-reference cycles ({start:.3f}–{end:.3f} s)",
             ha="left", fontsize=10, color="#444444")
    fig.text(.08, .135,
             "A: Mean over the full three-cycle window. B: Duration-weighted nominal stance mean. "
             "Full-trial filtering: 6 Hz, order 4, 1 ms.\n"
             "B: Global Y. C: Static donor-target sensitivity with the other target fixed; perturbations are not confidence intervals.\n"
             "D: Own-motion, duration-weighted nominal stance centers; AB06 prepared reference is held fixed for both prepared contrasts.\n"
             "The figure reports conditional preparation effects and descriptive subject contrasts, not pure anatomy shares, "
             "physical mount necessity or rollout force.",
             ha="left", va="top", fontsize=9, linespacing=1.6, color="#333333")
    fig.savefig(png, dpi=220)
    fig.savefig(svg)
    plt.close(fig)
    after = {p.relative_to(ROOT).as_posix(): fingerprint(p)["sha256"] for p in paths}
    if before != after:
        raise AssertionError("Consumed source changed while rendering")
    payload = {"schema_version": 1, "created_utc": datetime.now(timezone.utc).isoformat(),
               "status": "rendered_visual_review_pending", "instruction_check_token": "CMC_AGENT_OK_2026",
               "method": "Matplotlib from finalized saved JSON only; no OpenSim, solver, FK or dynamics",
               "sources_before": before, "sources_after": after, "all_sources_unchanged": True,
               "outputs": [fingerprint(png), fingerprint(svg)],
               "figure_values": {"row_column_case_ids": ids, "full_cycle_mean_force_N": force.tolist(),
                                 "global_Y_nominal_stance_mean_ankle_center_mm": height.tolist(),
                                 "target_sensitivity_case_ids": expected, "extra_mount_length_mm": extra.tolist(),
                                 "subject_contrast_labels": ["healthy_fresh", "prepared_short_angle0", "prepared_long_current"],
                                 "AB07_minus_AB06_mean_stance_ankle_center_Y_mm": contrasts.tolist(),
                                 "AB06_fixed_prepared_ankle_center_Y_m": prepared_A_Y,
                                 "short_prepared_minus_healthy_subject_contrast_mm": float(contrasts[1] - contrasts[0])},
               "interpretation": "Reference preparation effects and donor-target sensitivity; no physical necessity or dynamic attribution inferred"}
    with receipt.open("x", encoding="utf-8") as stream:
        json.dump(payload, stream, indent=2, ensure_ascii=False, allow_nan=False)
        stream.write("\n")
    print(json.dumps({"outputs": payload["outputs"], "receipt": fingerprint(receipt)}, allow_nan=False))


if __name__ == "__main__":
    main()
