"""Plot the frozen V5 P35 comparison at 10 ms and 1 ms.

This is read-only post-processing: the only input is the already completed V5
manifest.  It does not import OpenSim, replay the detector, alter the FSM, or
write into the frozen run directory.  The destination directory is strict
no-clobber and must not exist before execution.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from typing import Any, Mapping, Sequence


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MANIFEST = (
    REPO_ROOT
    / "validation/two_sensor_timing_placement_sweep_runs/"
    "2026-07-22_ab06_50_100_x2_depth_micro_v5/manifest.json"
)
DEFAULT_OUTPUT_DIR = (
    REPO_ROOT
    / "plot/07_22_2026_two_sensor_timing_placement_v5_multiresolution"
)
OUTPUT_NAME = "01_p35_10ms_vs_1ms_validation.png"
EXPECTED_PROTOCOL_ID = (
    "AB06_TWO_SENSOR_TIMING_PLACEMENT_DEVELOPMENT_2026-07-22_V5"
)
EXPECTED_MANIFEST_SHA256 = (
    "243900a5c10cea9a930f23fb71d8484be5276340743dbe5459b62327fee362bc"
)
EXPECTED_WINNER_ID = "H02_X2_F80_P35p00"
RESOLUTION_KEYS = ("10 ms", "1 ms")
COLORS = ("#2A9D8F", "#E76F51")


class ManifestError(ValueError):
    """Raised when the input is not the frozen completed V5 manifest."""


class NoClobberError(RuntimeError):
    """Raised before writing if the dedicated output directory exists."""


def _required(mapping: Mapping[str, Any], key: str, context: str) -> Any:
    if key not in mapping:
        raise ManifestError(f"missing {context}.{key}")
    return mapping[key]


def _number(mapping: Mapping[str, Any], key: str, context: str) -> float:
    value = _required(mapping, key, context)
    if isinstance(value, bool):
        raise ManifestError(f"{context}.{key} must be numeric")
    try:
        return float(value)
    except (TypeError, ValueError) as exc:
        raise ManifestError(f"{context}.{key} must be numeric") from exc


def _integer(mapping: Mapping[str, Any], key: str, context: str) -> int:
    value = _number(mapping, key, context)
    integer = int(value)
    if float(integer) != value:
        raise ManifestError(f"{context}.{key} must be an integer")
    return integer


def _winner_row(rows: Any, candidate_id: str, context: str) -> Mapping[str, Any]:
    if isinstance(rows, Mapping):
        keyed = rows.get(candidate_id)
        matches = (
            [keyed]
            if isinstance(keyed, Mapping)
            and keyed.get("candidate_id") == candidate_id
            else []
        )
    elif isinstance(rows, list):
        matches = [
            row
            for row in rows
            if isinstance(row, Mapping) and row.get("candidate_id") == candidate_id
        ]
    else:
        raise ManifestError(f"{context} must be a list or object")
    if len(matches) != 1:
        raise ManifestError(
            f"expected exactly one {candidate_id} row in {context}; got {len(matches)}"
        )
    return matches[0]


def load_frozen_manifest(path: Path = DEFAULT_MANIFEST) -> dict[str, Any]:
    """Load and identify the exact immutable V5 result used by this figure."""

    try:
        raw = path.read_bytes()
    except OSError as exc:
        raise ManifestError(f"cannot read manifest: {path}") from exc
    digest = hashlib.sha256(raw).hexdigest()
    if digest != EXPECTED_MANIFEST_SHA256:
        raise ManifestError(
            "V5 manifest hash drifted; refusing to plot a different result "
            f"({digest})"
        )
    try:
        manifest = json.loads(raw)
    except json.JSONDecodeError as exc:
        raise ManifestError(f"invalid JSON manifest: {path}") from exc
    if not isinstance(manifest, dict):
        raise ManifestError("manifest root must be an object")
    protocol = manifest.get("protocol", {})
    if protocol.get("protocol_id") != EXPECTED_PROTOCOL_ID:
        raise ManifestError("unexpected V5 protocol id")
    if manifest.get("selected_pair", {}).get("candidate_id") != EXPECTED_WINNER_ID:
        raise ManifestError("unexpected selected V5 pair")
    if manifest.get("detector_contract", {}).get("sensors_per_pair") != 2:
        raise ManifestError("V5 result does not preserve the two-sensor contract")
    return manifest


def extract_comparison(manifest: Mapping[str, Any]) -> dict[str, Any]:
    """Extract only the scalar P35 metrics required by the summary plot."""

    selected = _required(manifest, "selected_pair", "manifest")
    if not isinstance(selected, Mapping):
        raise ManifestError("manifest.selected_pair must be an object")
    candidate_id = str(_required(selected, "candidate_id", "selected_pair"))
    if candidate_id != EXPECTED_WINNER_ID:
        raise ManifestError(f"expected P35 winner, got {candidate_id}")

    primary = _required(manifest, "primary_10ms", "manifest")
    sensitivity = _required(
        manifest, "conditional_winner_current_1ms", "manifest"
    )
    if not isinstance(primary, Mapping) or not isinstance(sensitivity, Mapping):
        raise ManifestError("V5 cadence sections must be objects")
    if sensitivity.get("executed") is not True:
        raise ManifestError("V5 1 ms sensitivity was not executed")
    row_10 = _winner_row(primary.get("rows"), candidate_id, "primary_10ms.rows")
    row_1 = _winner_row(
        sensitivity.get("rows"),
        candidate_id,
        "conditional_winner_current_1ms.rows",
    )

    details_10 = primary.get("details", {}).get(candidate_id, {})
    details_1 = sensitivity.get("details", {}).get(candidate_id, {})
    if not isinstance(details_10, Mapping) or not isinstance(details_1, Mapping):
        raise ManifestError("winner details must be objects")
    diagnostics_10 = details_10.get("primary_event_diagnostics", {})
    diagnostics_1 = details_1.get("primary_event_diagnostics", {})
    if not isinstance(diagnostics_10, Mapping) or not isinstance(
        diagnostics_1, Mapping
    ):
        raise ManifestError("primary event diagnostics must be objects")
    hs_10 = diagnostics_10.get("heel_strike", {})
    hs_1 = diagnostics_1.get("heel_strike", {})
    to_10 = diagnostics_10.get("toe_off", {})
    to_1 = diagnostics_1.get("toe_off", {})
    if not all(isinstance(item, Mapping) for item in (hs_10, hs_1, to_10, to_1)):
        raise ManifestError("HS/TO diagnostics must be objects")
    hs_tolerance_s = _number(hs_10, "tolerance_s", "10ms.hs")
    to_tolerance_s = _number(to_10, "tolerance_s", "10ms.to")
    if _number(hs_1, "tolerance_s", "1ms.hs") != hs_tolerance_s:
        raise ManifestError("HS tolerance changed between cadences")
    if _number(to_1, "tolerance_s", "1ms.to") != to_tolerance_s:
        raise ManifestError("TO tolerance changed between cadences")

    gate_10 = primary.get("selection", {}).get("candidate_gates", {}).get(
        candidate_id, {}
    )
    gate_1 = sensitivity.get("assessment", {}).get("gate", {})
    if not isinstance(gate_10, Mapping) or not isinstance(gate_1, Mapping):
        raise ManifestError("cadence gates must be objects")
    release_threshold_s = _number(
        gate_10, "minimum_forefoot_release_margin_s", "10ms.gate"
    )
    if (
        _number(gate_1, "minimum_forefoot_release_margin_s", "1ms.gate")
        != release_threshold_s
    ):
        raise ManifestError("release-margin threshold changed between cadences")

    semantic_10 = details_10.get("semantic_gate", {})
    semantic_1 = details_1.get("semantic_gate", {})
    if not isinstance(semantic_10, Mapping) or not isinstance(
        semantic_1, Mapping
    ):
        raise ManifestError("semantic gates must be objects")
    expected_cycles = _integer(
        semantic_10, "expected_complete_cycles", "10ms.semantic_gate"
    )
    if (
        _integer(semantic_1, "expected_complete_cycles", "1ms.semantic_gate")
        != expected_cycles
    ):
        raise ManifestError("expected cycle count changed between cadences")

    rows: list[dict[str, Any]] = []
    for label, row, gate in (
        ("10 ms", row_10, gate_10),
        ("1 ms", row_1, gate_1),
    ):
        rows.append(
            {
                "label": label,
                "sample_dt_ms": 1000.0
                * _number(row, "sample_dt_s", f"{label}.row"),
                "hs_max_ms": 1000.0
                * _number(row, "max_abs_hs_error_s", f"{label}.row"),
                "to_max_ms": 1000.0
                * _number(row, "max_abs_toe_off_error_s", f"{label}.row"),
                "both_off_duration_ms": 1000.0
                * _number(
                    row, "maximum_interior_both_off_gap_s", f"{label}.row"
                ),
                "both_off_samples": _integer(
                    row,
                    "transfer_both_latches_off_sample_count",
                    f"{label}.row",
                ),
                "early_to_count": _integer(
                    row, "to_candidates_before_min_stance_count", f"{label}.row"
                ),
                "release_margin_ms": 1000.0
                * _number(
                    row, "minimum_forefoot_release_margin_s", f"{label}.row"
                ),
                "valid_cycles": _integer(
                    row, "observed_valid_cycle_count", f"{label}.row"
                ),
                "accepted_hs": _integer(
                    row, "predicted_hs_count", f"{label}.row"
                ),
                "accepted_to": _integer(
                    row, "predicted_to_count", f"{label}.row"
                ),
                "invalid_transitions": _integer(
                    row,
                    "invalid_or_timeout_transition_count",
                    f"{label}.row",
                ),
                "unaccepted_events": _integer(
                    row,
                    "unaccepted_sensor_gait_event_count",
                    f"{label}.row",
                ),
                "gate_pass": gate.get("ok") is True,
            }
        )

    reference_hs = _integer(row_10, "reference_hs_count", "10ms.row")
    reference_to = _integer(row_10, "reference_to_count", "10ms.row")
    if (
        _integer(row_1, "reference_hs_count", "1ms.row") != reference_hs
        or _integer(row_1, "reference_to_count", "1ms.row") != reference_to
    ):
        raise ManifestError("reference event counts changed between cadences")

    contract = _required(manifest, "detector_contract", "manifest")
    if not isinstance(contract, Mapping) or contract.get("sensors_per_pair") != 2:
        raise ManifestError("detector is not exactly one heel plus one forefoot sensor")
    if contract.get("sensor_roles") != ["heel", "forefoot"]:
        raise ManifestError("unexpected detector sensor roles")

    return {
        "candidate_id": candidate_id,
        "rows": rows,
        "thresholds": {
            "hs_max_ms": 1000.0 * hs_tolerance_s,
            "to_max_ms": 1000.0 * to_tolerance_s,
            "both_off_duration_ms": 0.0,
            "both_off_samples": 0,
            "early_to_count": 0,
            "release_margin_ms": 1000.0 * release_threshold_s,
            "valid_cycles": expected_cycles,
            "accepted_hs": reference_hs,
            "accepted_to": reference_to,
        },
        "sensors_per_pair": 2,
        "status": str(manifest.get("status")),
        "conclusion": str(manifest.get("conclusion")),
    }


def _bar_labels(axis: Any, bars: Any, fmt: str = "{:.1f}") -> None:
    for bar in bars:
        value = float(bar.get_height())
        axis.annotate(
            fmt.format(value),
            (bar.get_x() + bar.get_width() / 2.0, value),
            xytext=(0, 4),
            textcoords="offset points",
            ha="center",
            va="bottom",
            fontsize=9,
            fontweight="bold",
        )


def _single_metric_panel(
    axis: Any,
    rows: Sequence[Mapping[str, Any]],
    *,
    key: str,
    title: str,
    ylabel: str,
    threshold: float,
    criterion: str,
) -> None:
    values = [float(row[key]) for row in rows]
    bars = axis.bar(RESOLUTION_KEYS, values, color=COLORS, width=0.58)
    _bar_labels(axis, bars)
    axis.axhline(
        threshold,
        color="#303030",
        linestyle="--",
        linewidth=1.2,
        label=f"soglia {criterion} {threshold:g}",
    )
    top = max(values + [threshold, 1.0])
    axis.set_ylim(0.0, top * 1.24)
    axis.set_ylabel(ylabel)
    axis.set_title(title, fontweight="bold")
    axis.grid(axis="y", alpha=0.22)
    axis.legend(loc="upper left", fontsize=8)


def plot_comparison(comparison: Mapping[str, Any], output_dir: Path) -> Path:
    """Create the six-panel comparison in a brand-new directory."""

    if output_dir.exists():
        raise NoClobberError(f"refusing to overwrite existing path: {output_dir}")

    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np

    rows = list(comparison["rows"])
    thresholds = comparison["thresholds"]
    figure, axes = plt.subplots(2, 3, figsize=(16, 9.6))

    _single_metric_panel(
        axes[0, 0],
        rows,
        key="hs_max_ms",
        title="Errore massimo Heel Strike",
        ylabel="errore assoluto [ms]",
        threshold=float(thresholds["hs_max_ms"]),
        criterion="≤",
    )
    _single_metric_panel(
        axes[0, 1],
        rows,
        key="to_max_ms",
        title="Errore massimo Toe Off",
        ylabel="errore assoluto [ms]",
        threshold=float(thresholds["to_max_ms"]),
        criterion="≤",
    )
    _single_metric_panel(
        axes[0, 2],
        rows,
        key="both_off_duration_ms",
        title="Massimo intervallo heel+toe OFF",
        ylabel="durata [ms]",
        threshold=float(thresholds["both_off_duration_ms"]),
        criterion="=",
    )
    axes[0, 2].text(
        0.98,
        0.92,
        "deve essere esattamente zero",
        transform=axes[0, 2].transAxes,
        ha="right",
        va="top",
        fontsize=8,
        color="#555555",
    )

    violation_axis = axes[1, 0]
    x = np.arange(2, dtype=float)
    width = 0.34
    both_off = [int(row["both_off_samples"]) for row in rows]
    early_to = [int(row["early_to_count"]) for row in rows]
    bars_off = violation_axis.bar(
        x - width / 2.0,
        both_off,
        width,
        color="#457B9D",
        label="campioni heel+toe OFF",
    )
    bars_early = violation_axis.bar(
        x + width / 2.0,
        early_to,
        width,
        color="#F4A261",
        label="candidati TO precoci",
    )
    _bar_labels(violation_axis, bars_off, "{:.0f}")
    _bar_labels(violation_axis, bars_early, "{:.0f}")
    violation_axis.axhline(0.0, color="#303030", linestyle="--", linewidth=1.2)
    violation_axis.set_xticks(x, RESOLUTION_KEYS)
    violation_axis.set_ylim(0.0, max(both_off + early_to + [1]) * 1.28)
    violation_axis.set_ylabel("conteggio; soglia = 0")
    violation_axis.set_title("Violazioni discrete", fontweight="bold")
    violation_axis.grid(axis="y", alpha=0.22)
    violation_axis.legend(loc="upper left", fontsize=8)

    _single_metric_panel(
        axes[1, 1],
        rows,
        key="release_margin_ms",
        title="Margine di rilascio forefoot",
        ylabel="margine minimo [ms]",
        threshold=float(thresholds["release_margin_ms"]),
        criterion="≥",
    )

    count_axis = axes[1, 2]
    categories = ("cicli validi", "HS accettati", "TO accettati")
    count_keys = ("valid_cycles", "accepted_hs", "accepted_to")
    expected = [int(thresholds[key]) for key in count_keys]
    x_counts = np.arange(len(categories), dtype=float)
    width_counts = 0.34
    for index, row in enumerate(rows):
        values = [int(row[key]) for key in count_keys]
        bars = count_axis.bar(
            x_counts + (index - 0.5) * width_counts,
            values,
            width_counts,
            color=COLORS[index],
            label=row["label"],
        )
        _bar_labels(count_axis, bars, "{:.0f}")
    for index, target in enumerate(expected):
        count_axis.hlines(
            target,
            index - 0.42,
            index + 0.42,
            color="#303030",
            linestyle="--",
            linewidth=1.1,
        )
    count_axis.set_xticks(x_counts, categories)
    count_axis.set_ylim(0.0, max(expected) * 1.16)
    count_axis.set_ylabel("conteggio; soglia = valore esatto")
    count_axis.set_title("Cicli ed eventi confermati", fontweight="bold")
    count_axis.grid(axis="y", alpha=0.22)
    count_axis.legend(loc="lower right", fontsize=8)
    count_axis.text(
        0.02,
        0.96,
        (
            "invalid / non accettati: "
            f"{rows[0]['invalid_transitions']}/{rows[0]['unaccepted_events']} "
            "(10 ms), "
            f"{rows[1]['invalid_transitions']}/{rows[1]['unaccepted_events']} "
            "(1 ms)"
        ),
        transform=count_axis.transAxes,
        ha="left",
        va="top",
        fontsize=8,
    )

    verdict = " | ".join(
        f"{row['label']}: {'PASS' if row['gate_pass'] else 'FAIL'}" for row in rows
    )
    figure.suptitle(
        "Detector a due sensori — P35, confronto multirisoluzione\n"
        f"{verdict}  ·  1 heel + 1 forefoot  ·  blocco development 50–100 s",
        fontsize=15,
        fontweight="bold",
    )
    figure.text(
        0.5,
        0.015,
        (
            "Le linee tratteggiate sono le soglie del medesimo gate V5. "
            "A 1 ms i conteggi finali restano corretti, ma compaiono una "
            "discontinuità di 6 ms, un TO precoce e margine di rilascio insufficiente."
        ),
        ha="center",
        va="bottom",
        fontsize=9,
        color="#404040",
    )
    figure.tight_layout(rect=(0.02, 0.055, 0.98, 0.91))

    output_dir.mkdir(parents=True, exist_ok=False)
    output = output_dir / OUTPUT_NAME
    figure.savefig(output, dpi=190, bbox_inches="tight")
    plt.close(figure)
    return output


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    manifest_path = args.manifest.expanduser().resolve()
    output_dir = args.output_dir.expanduser().resolve()
    manifest = load_frozen_manifest(manifest_path)
    comparison = extract_comparison(manifest)
    output = plot_comparison(comparison, output_dir)
    print(
        json.dumps(
            {
                "input_manifest": str(manifest_path),
                "input_sha256": EXPECTED_MANIFEST_SHA256,
                "output": str(output),
                "candidate_id": comparison["candidate_id"],
                "sensors_per_pair": comparison["sensors_per_pair"],
                "cadence_gate_pass": {
                    row["label"]: row["gate_pass"] for row in comparison["rows"]
                },
            },
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
