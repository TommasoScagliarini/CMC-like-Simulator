"""Plot the three frozen static mounts using saved rigid transforms only.

No native FK, IK, model writes, contact-force calculations or dynamics reads.
"""
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import struct

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
PLOT = ROOT / "plot/AB07_standing_geometry_fit_2026_10_07"
RECEIPT = HERE / "calibration_geometry_plot_receipt_v2.json"
SOURCES = {
    "helper": Path(__file__),
    "two_region_fit": HERE / "fit_geometry_receipt_completed.json",
    "four_region_fit": HERE / "four_roi_support/fit_receipt.json",
    "four_region_qualification": HERE / "four_roi_support/saved_geometry_qualification.json",
    "four_region_protocol": HERE / "four_roi_support/protocol.json",
    "comparison_400": HERE / "static_mount_comparison.json",
    "fixed_STL": ROOT / "Geometry/AM_foot_l.STL",
}
EXPECTED = {
    "two_region_fit": "230ea4bf5b985537fe5f5fd2a1ec277743db92dee9f59935b5b3a32a53d236cc",
    "four_region_fit": "cb82cf29c331e18d980d03a1044e42276779bbfdf524e77b4d6736f16fc6b7fd",
    "four_region_qualification": "fae2a52fd939993a6d4a511b2cceba30e882f6ed1a3555c30d40595a5dc52c1a",
    "four_region_protocol": "6e22be63038f45038ab6b43d4648053d2fa6be6e3788fcff4b11d7f98cd71eb7",
    "fixed_STL": "fcfc4d7a90c4ccd3bedb501ec3e50d4337aa9ca6e8438b58cc6be00f47a689e9",
}


def fp(p):
    return {"path": str(p.resolve().relative_to(ROOT)),
            "sha256": hashlib.sha256(p.read_bytes()).hexdigest(),
            "size_bytes": p.stat().st_size}


def rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])


def main():
    outputs = [PLOT / ("calibration_geometry_v2." + ext) for ext in ("png", "svg")]
    if RECEIPT.exists() or any(p.exists() for p in outputs):
        raise FileExistsError("Saved figures and receipt are immutable")
    before = {role: fp(p) for role, p in SOURCES.items()}
    for role, digest in EXPECTED.items():
        if before[role]["sha256"] != digest:
            raise ValueError("Frozen geometry source changed: " + role)
    a = json.loads(SOURCES["two_region_fit"].read_text())
    b = json.loads(SOURCES["four_region_fit"].read_text())
    c = json.loads(SOURCES["comparison_400"].read_text())
    proto = json.loads(SOURCES["four_region_protocol"].read_text())
    qual = json.loads(SOURCES["four_region_qualification"].read_text())
    if a["status"] != "completed" or b["strict_verdict"] != "PASS" or qual["status"] != "PASS":
        raise ValueError("Final geometry qualification is required")
    raw = SOURCES["fixed_STL"].read_bytes()
    count = struct.unpack_from("<I", raw, 80)[0]
    if len(raw) != 84 + count * 50 or not count:
        raise ValueError("Exact binary STL required")
    dtype = np.dtype([("normal", "<f4", (3,)), ("vertices", "<f4", (3, 3)), ("attribute", "<u2")])
    local = np.unique(np.ndarray((count,), dtype=dtype, buffer=raw, offset=84)["vertices"].reshape(-1, 3).astype(float), axis=0)
    r2 = np.array(a["geometry"]["foot_rotation_ground"])
    qa2 = a["reference_coordinate_maps"]["prepared_deg_m"]["pros_ankle_angle"]
    v400 = c["numerical_values"]
    # All first-fit orientations are unchanged. Pin rotation is local +Z.
    r400 = r2 @ rz(math.radians(v400["static_400_ankle_angle_deg"] - qa2))
    cases = [
        {"id": "400_REFERENCE", "title": "Riferimento 400 mm", "L_mm": 400., "alpha_deg": 0.,
         "qankle_deg": v400["static_400_ankle_angle_deg"], "R": r400,
         "p": np.array(v400["static_400_ankle_XYZ_m"]), "knee": np.array(v400["static_400_knee_XYZ_m"])},
        {"id": "TWO_ROI", "title": "Fit tallone + avampiede", "L_mm": a["mounts"]["distal_pylon_length_m"] * 1000,
         "alpha_deg": 0., "qankle_deg": qa2, "R": r2,
         "p": np.array(a["geometry"]["fitted_ankle_XYZ_m"]), "knee": np.array(a["geometry"]["fitted_knee_XYZ_m"])},
        {"id": "FOUR_ROI", "title": "Fit quattro regioni", "L_mm": b["mounts"]["pylon_length_m"] * 1000,
         "alpha_deg": b["mounts"]["osseo_parent_orientation_X_deg"],
         "qankle_deg": b["reference_coordinate_maps"]["prepared_deg_m"]["pros_ankle_angle"],
         "R": np.array(b["geometry"]["foot_rotation_ground"]),
         "p": np.array(b["geometry"]["foot_origin_XYZ_m"]), "knee": np.array(b["geometry"]["fitted_knee_XYZ_m"])},
    ]
    healthy = np.array(a["geometry"]["healthy_knee_target_XYZ_m"])
    regions = {}
    for name, rule in proto["four_ROI"].items():
        mask = local[:, 0] <= proto["heel_X_max_m"] if rule["region"] == "heel" else local[:, 0] >= proto["forefoot_X_min_m"]
        mask &= local[:, 2] <= rule["Z_max_m"] if "Z_max_m" in rule else local[:, 2] >= rule["Z_min_m"]
        regions[name] = mask
    numerical = {}
    for item in cases:
        world = local @ item["R"].T + item["p"]
        item["world"] = world
        item["contacts"] = np.flatnonzero(np.abs(world[:, 1]) <= 1e-10)
        item["witnesses"] = [int(np.flatnonzero(mask)[np.argmin(world[mask, 1])]) for mask in regions.values()]
        item["roi_mm"] = {name: float(world[mask, 1].min() * 1000) for name, mask in regions.items()}
        numerical[item["id"]] = {"length_mm": item["L_mm"], "mount_alpha_deg": item["alpha_deg"],
            "static_ankle_angle_deg": item["qankle_deg"],
            "static_ankle_offset_from_healthy_deg": item["qankle_deg"] - a["reference_coordinate_maps"]["healthy_deg_m"]["ankle_angle_l"],
            "knee_Y_mm": float(item["knee"][1] * 1000), "ankle_Y_mm": float(item["p"][1] * 1000),
            "whole_STL_min_Y_mm": float(world[:, 1].min() * 1000), "ROI_min_Y_mm": item["roi_mm"],
            "actual_contact_vertices": len(item["contacts"])}
    checks = {
        "400_whole_STL_matches_root_saved_summary": abs(numerical["400_REFERENCE"]["whole_STL_min_Y_mm"] / 1000 - v400["static_400_whole_mesh_minY_m"]) < 1e-12,
        "two_ROI_mesh_matches_saved_native": abs(numerical["TWO_ROI"]["whole_STL_min_Y_mm"] / 1000 - a["geometry"]["whole_STL_min_Y_m"]) < 1e-12,
        "four_ROI_matches_saved_native": all(abs(numerical["FOUR_ROI"]["ROI_min_Y_mm"][name] / 1000 - b["geometry"]["ROI_min_Y_m"][name]) < 1e-12 for name in regions),
        "all_saved_transform_geometry_finite": all(np.all(np.isfinite(item["world"])) for item in cases),
    }
    if not all(checks.values()):
        raise ValueError("Saved rigid geometry failed reproducibility checks")
    plt.rcParams.update({"font.size": 10, "svg.fonttype": "none"})
    fig, axes = plt.subplots(2, 3, figsize=(14, 10), gridspec_kw={"height_ratios": [1.45, 1]})
    colors = ["#c45e26", "#22719f", "#19865d"]
    all_x = np.concatenate([(item["world"][:, 0] - healthy[0]) * 1000 for item in cases])
    x_limits = (float(all_x.min() - 12), float(all_x.max() + 12))
    for j, (item, color) in enumerate(zip(cases, colors)):
        world, knee, ankle = item["world"], item["knee"], item["p"]
        x = (world[:, 0] - healthy[0]) * 1000
        z = (world[:, 2] - healthy[2]) * 1000
        y = world[:, 1] * 1000
        ax = axes[0, j]
        ax.scatter(x, y, s=.9, color=color, alpha=.45, rasterized=False)
        ax.plot((np.array([knee[0], ankle[0]]) - healthy[0]) * 1000, np.array([knee[1], ankle[1]]) * 1000, color=color, linewidth=3)
        ax.scatter((np.array([knee[0], ankle[0]]) - healthy[0]) * 1000, np.array([knee[1], ankle[1]]) * 1000, color=color, s=35, zorder=5)
        ax.axhline(healthy[1] * 1000, color="#545454", linestyle="--", linewidth=1)
        ax.axhline(0, color="#333333", linewidth=1)
        ax.set_xlim(*x_limits); ax.set_ylim(-12, 510)
        ax.set_xlabel("X rispetto al ginocchio healthy [mm]")
        ax.set_ylabel("Y nel frame modello [mm]" if j == 0 else "")
        ax.set_title(item["title"] + f"\nL={item['L_mm']:.3f} mm; montaggio X={item['alpha_deg']:+.3f}°", fontsize=12)
        ax.text(.04, .87, f"Ginocchio {knee[1]*1000:.3f} mm", transform=ax.transAxes, color=color)
        ax.text(.04, .75, "L = offset del montaggio distale\n" + f"Caviglia statica {item['qankle_deg']:.3f}°\n" + f"Offset da healthy {numerical[item['id']]['static_ankle_offset_from_healthy_deg']:+.3f}°", transform=ax.transAxes, fontsize=9)
        ax.grid(alpha=.18)
        ax = axes[1, j]
        ax.scatter(z, y, s=1, color=color, alpha=.35)
        ax.scatter(z[item["witnesses"]], y[item["witnesses"]], color=color, marker="x", s=35, linewidths=1.5, label="Minimi delle quattro regioni")
        if len(item["contacts"]):
            ax.scatter(z[item["contacts"]], y[item["contacts"]], facecolors="none", edgecolors="#222222", s=60, linewidths=1.2)
        ax.axhline(0, color="#222222", linewidth=1.3)
        ax.axhline(-9.503954760384832, color="#9355a3", linestyle=":", linewidth=1)
        ax.axhline(14.8208231, color="#c09426", linestyle=":", linewidth=1)
        ax.set_xlim(-110, 90); ax.set_ylim(-15, 95)
        ax.set_xlabel("Z rispetto al ginocchio healthy [mm]")
        ax.set_ylabel("Y piede [mm]" if j == 0 else "")
        gaps = list(item["roi_mm"].values())
        ax.text(.035, .98, f"Minimi laterale / mediale [mm]\nTallone: {gaps[0]:.6f} / {gaps[1]:.6f}\nAvampiede: {gaps[2]:.6f} / {gaps[3]:.6f}\nContatti esatti: {len(item['contacts'])}", va="top", transform=ax.transAxes, fontsize=8.5,
                bbox={"facecolor": "white", "edgecolor": "none", "alpha": .88})
        ax.grid(alpha=.18)
    fig.suptitle("AB07: stessa posa healthy acquisita, tre montaggi statici\nGinocchio healthy Y=474.956 mm; identica mesh protesica", fontsize=15, y=.99)
    fig.text(.04, .02, "Y=0 è una convenzione benchmark, non un pavimento misurato. Piani invariati: forza −9.504 mm (viola), detector +14.821 mm (oro).\n"
             "Due regioni: contatto solo mediale. Quattro regioni: massimo gap 0.000845 mm, tre contatti non allineati; la mesh curva non è tutta planare.\n"
             "L=|translation.Y| di pros_ankle; non è la lunghezza del corpo osseo_pylon. Connessione osseo_pylon→ginocchio invariata: 205 mm.\n"
             "Geometria salvata, senza nuove FK/IK/dinamiche o indicazione di capacità di carico/necessità anatomica.", fontsize=9)
    fig.tight_layout(rect=[.015, .115, .995, .935], h_pad=2.5)
    PLOT.mkdir(parents=True, exist_ok=True)
    for p in outputs:
        fig.savefig(p, dpi=180 if p.suffix == ".png" else None)
    plt.close(fig)
    after = {role: fp(p) for role, p in SOURCES.items()}
    checks["sources_unchanged_before_after_render"] = before == after
    if not all(checks.values()):
        raise ValueError("Source changed during rendering")
    result = {"created_utc": datetime.now(timezone.utc).isoformat(), "status": "PASS", "checks": checks,
        "sources_before": before, "sources_after": after, "figures": [fp(p) for p in outputs],
        "static_geometry_values": numerical, "new_native_FK_IK_fit_or_integration_calls": 0,
        "force_event_reserve_data_read": False, "new_Claude_contents_read": False,
        "visual_review": "pending", "render_revision": "V2 expands common side-view X bounds to include every saved STL vertex; V1 preserved.", "limits": ["Saved static geometry only; no load/stability or physical dimension identification.",
        "Plot is supplementary calibration illustration; the canonical seven rollout plots are separate."]}
    with RECEIPT.open("x") as stream:
        stream.write(json.dumps(result, indent=2, allow_nan=False) + "\n")
    print(json.dumps({"status": result["status"], "receipt": fp(RECEIPT), "figures": result["figures"], "checks": checks}, indent=2))


if __name__ == "__main__":
    main()
