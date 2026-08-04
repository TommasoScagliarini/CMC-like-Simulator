"""Plot a readable summary of the frozen V4 fixed-profile comparison.

This is post-processing only.  It reads the completed manifest and does not
sample OpenSim, rerun the FSM, or alter the preregistered decision.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MANIFEST = (
    REPO_ROOT
    / "validation/two_sensor_mesh_profile_v4_fixed_replay_runs/"
    "2026-07-21_ab06_50_100_fixed_v4/manifest.json"
)
DEFAULT_OUTPUT = (
    REPO_ROOT
    / "plot/07_21_2026_two_sensor_v4_mesh_fixed_replay/v4/"
    "03_fixed_replay_summary.png"
)
PROFILE_IDS = ("current_geometry", "v4_mesh_geometry")
COLORS = {"current_geometry": "#4C78A8", "v4_mesh_geometry": "#E45756"}
LABELS = {"current_geometry": "current profile", "v4_mesh_geometry": "V4 mesh"}


def _load_manifest(path: Path) -> dict[str, Any]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if payload.get("conclusion") != "V4_PRESCRIBED_FIXED_GATE_FAIL_NOT_PROMOTABLE":
        raise ValueError("expected the completed frozen V4 fixed-replay manifest")
    if payload.get("data_access", {}).get("sealed_block_opened") is not False:
        raise ValueError("refusing a manifest that does not prove the sealed block stayed closed")
    for cadence in ("primary_10ms", "sensitivity_1ms"):
        access = payload["data_access"]["cadences"][cadence]
        if access["samples_at_or_after_100_s"] != 0:
            raise ValueError("manifest contains samples at or after 100 s")
    return payload


def _cycles(
    manifest: Mapping[str, Any],
    profile_id: str,
) -> list[Mapping[str, Any]]:
    return list(
        manifest["details"]["primary_10ms"][profile_id][
            "regional_continuity_diagnostics_not_gate"
        ]["cycles"]
    )


def _finite_cycle_values(
    cycles: Sequence[Mapping[str, Any]],
    key: str,
) -> tuple[np.ndarray, np.ndarray]:
    pairs = [
        (int(item["cycle_index"]), float(item[key]))
        for item in cycles
        if item.get(key) is not None and np.isfinite(float(item[key]))
    ]
    if not pairs:
        return np.asarray([], dtype=int), np.asarray([], dtype=float)
    return (
        np.asarray([item[0] for item in pairs], dtype=int),
        np.asarray([item[1] for item in pairs], dtype=float),
    )


def plot_summary(manifest: Mapping[str, Any], output: Path) -> dict[str, Any]:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    rows = manifest["metrics"]["primary_10ms"]
    figure, axes = plt.subplots(2, 2, figsize=(15, 10))

    event_axis = axes[0, 0]
    x = np.arange(2, dtype=float)
    width = 0.24
    reference = [
        int(rows["current_geometry"]["reference_hs_count"]),
        int(rows["current_geometry"]["reference_to_count"]),
    ]
    event_axis.bar(x - width, reference, width, color="0.65", label="prescribed")
    for offset, profile_id in ((0.0, "current_geometry"), (width, "v4_mesh_geometry")):
        values = [
            int(rows[profile_id]["predicted_hs_count"]),
            int(rows[profile_id]["predicted_to_count"]),
        ]
        bars = event_axis.bar(
            x + offset,
            values,
            width,
            color=COLORS[profile_id],
            label=LABELS[profile_id],
        )
        event_axis.bar_label(bars, padding=2)
    event_axis.set_xticks(x, ("Heel Strike", "Toe Off"))
    event_axis.set_ylabel("event count")
    event_axis.set_title("Accepted confirmed events — 10 ms")
    event_axis.legend(loc="best")
    event_axis.grid(axis="y", alpha=0.25)

    phase_axis = axes[0, 1]
    phase_metrics = ("confirmed_fsm_stance_f1", "confirmed_fsm_stance_iou")
    x_phase = np.arange(len(phase_metrics), dtype=float)
    for offset, profile_id in ((-0.18, "current_geometry"), (0.18, "v4_mesh_geometry")):
        values = [float(rows[profile_id][key]) for key in phase_metrics]
        bars = phase_axis.bar(
            x_phase + offset,
            values,
            0.36,
            color=COLORS[profile_id],
            label=LABELS[profile_id],
        )
        phase_axis.bar_label(bars, fmt="%.3f", padding=2)
    phase_axis.axhline(0.95, color="black", linestyle="--", linewidth=0.9, label="F1 gate")
    phase_axis.axhline(0.90, color="0.35", linestyle=":", linewidth=1.0, label="IoU gate")
    phase_axis.set_xticks(x_phase, ("stance F1", "stance IoU"))
    phase_axis.set_ylim(0.0, 1.08)
    phase_axis.set_title("Confirmed FSM phase quality — 10 ms")
    phase_axis.legend(loc="best", fontsize=8)
    phase_axis.grid(axis="y", alpha=0.25)

    onset_axis = axes[1, 0]
    for profile_id in PROFILE_IDS:
        cycles = _cycles(manifest, profile_id)
        indices, onset = _finite_cycle_values(cycles, "toe_on_after_hs_s")
        onset_axis.plot(
            indices + 1,
            1000.0 * onset,
            "o-",
            markersize=3,
            linewidth=1.1,
            color=COLORS[profile_id],
            label=f"{LABELS[profile_id]}: toe-on",
        )
        duration_indices, duration = _finite_cycle_values(cycles, "toe_contact_duration_s")
        onset_axis.plot(
            duration_indices + 1,
            1000.0 * duration,
            linestyle="--",
            linewidth=1.0,
            color=COLORS[profile_id],
            alpha=0.8,
            label=f"{LABELS[profile_id]}: toe duration",
        )
    onset_axis.set_xlabel("prescribed cycle")
    onset_axis.set_ylabel("time [ms]")
    onset_axis.set_title("Toe engagement within prescribed stance — diagnostic")
    onset_axis.legend(loc="best", fontsize=8)
    onset_axis.grid(alpha=0.25)

    gap_axis = axes[1, 1]
    for profile_id in PROFILE_IDS:
        cycles = _cycles(manifest, profile_id)
        indices, gap = _finite_cycle_values(cycles, "heel_off_to_toe_on_gap_s")
        gap_axis.plot(
            indices + 1,
            1000.0 * gap,
            "o-",
            markersize=3,
            linewidth=1.1,
            color=COLORS[profile_id],
            label=f"{LABELS[profile_id]}: heel→toe gap",
        )
        interior_indices, interior = _finite_cycle_values(
            cycles, "maximum_interior_both_off_gap_s"
        )
        gap_axis.plot(
            interior_indices + 1,
            1000.0 * interior,
            linestyle="--",
            linewidth=1.0,
            color=COLORS[profile_id],
            alpha=0.8,
            label=f"{LABELS[profile_id]}: interior both-OFF",
        )
    gap_axis.axhline(0.0, color="black", linewidth=0.8)
    gap_axis.set_xlabel("prescribed cycle")
    gap_axis.set_ylabel("unsupported gap [ms]")
    gap_axis.set_title("Heel-to-toe contact continuity — diagnostic")
    gap_axis.legend(loc="best", fontsize=8)
    gap_axis.grid(alpha=0.25)

    figure.suptitle(
        "Experimental V4 mesh detector — fixed prescribed design replay: FAIL / not promotable\n"
        "50 common cycles; thresholds 0.5/0.25 N; dwell 30 ms; sealed block untouched"
    )
    figure.tight_layout(rect=(0.0, 0.0, 1.0, 0.94))
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=180)
    plt.close(figure)

    return {
        "output": str(output.resolve()),
        "status": manifest["status"],
        "v4_hs_count": int(rows["v4_mesh_geometry"]["predicted_hs_count"]),
        "v4_to_count": int(rows["v4_mesh_geometry"]["predicted_to_count"]),
        "sealed_block_opened": bool(manifest["data_access"]["sealed_block_opened"]),
    }


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", default=str(DEFAULT_MANIFEST))
    parser.add_argument("--output", default=str(DEFAULT_OUTPUT))
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    manifest_path = Path(args.manifest).resolve()
    output_path = Path(args.output).resolve()
    result = plot_summary(_load_manifest(manifest_path), output_path)
    print(json.dumps(result, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
