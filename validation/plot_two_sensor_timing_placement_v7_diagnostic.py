"""Create a readable diagnostic summary of the frozen V7 placement sweep.

The V7 sweep stores ``999`` as a sentinel when event counts do not match.  A
sentinel is not a timing error and must never become a bar in this figure.
Missing events are instead reported explicitly beside the corresponding
cadence.  This module is read-only post-processing: it does not replay the
detector, import OpenSim, alter the frozen manifest, or write in the frozen run
directory.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
from typing import Any, Mapping, Sequence


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MANIFEST = (
    REPO_ROOT
    / "validation/two_sensor_timing_placement_sweep_runs/"
    "2026-07-22_ab06_50_100_multires_geometry_v7/manifest.json"
)
DEFAULT_OUTPUT_DIR = (
    REPO_ROOT
    / "plot/07_22_2026_two_sensor_timing_placement_v7_diagnostic"
)
OUTPUT_NAME = "01_two_sensor_v7_multiresolution_diagnostic.png"
EXPECTED_MANIFEST_SHA256 = (
    "e76e91bb888387ac50549225c5b80bb21eb1da6111162961a399eff3d9e4d661"
)
EXPECTED_PROTOCOL_ID = (
    "AB06_TWO_SENSOR_TIMING_PLACEMENT_DEVELOPMENT_2026-07-22_V7"
)
EXPECTED_STATUS = "FAIL"
EXPECTED_CONCLUSION = "NO_STRICT_MULTIRESOLUTION_DEVELOPMENT_WINNER"
CADENCES = (
    ("10 ms", "runtime_10ms", "#2A9D8F"),
    ("1 ms", "fine_1ms", "#E76F51"),
)
SENTINEL_FLOOR = 900.0


class ManifestError(ValueError):
    """Raised if the input is not the exact completed V7 manifest."""


class NoClobberError(RuntimeError):
    """Raised before writing when the dedicated destination already exists."""


def _required(mapping: Mapping[str, Any], key: str, context: str) -> Any:
    if key not in mapping:
        raise ManifestError(f"missing {context}.{key}")
    return mapping[key]


def _number(mapping: Mapping[str, Any], key: str, context: str) -> float:
    value = _required(mapping, key, context)
    if isinstance(value, bool):
        raise ManifestError(f"{context}.{key} must be numeric")
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise ManifestError(f"{context}.{key} must be numeric") from exc
    if not math.isfinite(number):
        raise ManifestError(f"{context}.{key} must be finite")
    return number


def _integer(mapping: Mapping[str, Any], key: str, context: str) -> int:
    number = _number(mapping, key, context)
    result = int(number)
    if float(result) != number:
        raise ManifestError(f"{context}.{key} must be an integer")
    return result


def _short_label(candidate_id: str) -> str:
    prefix = "H02_X"
    if candidate_id.startswith(prefix):
        return candidate_id[len(prefix) :].replace("p", ".").replace("_", "/")
    if candidate_id == "v5_h02_x2_f80_p35_comparator":
        return "V5 comparator"
    if candidate_id == "current_geometry":
        return "current"
    return candidate_id


def load_frozen_manifest(path: Path = DEFAULT_MANIFEST) -> dict[str, Any]:
    """Load and identify the immutable V7 result used by this diagnostic."""

    try:
        raw = path.read_bytes()
    except OSError as exc:
        raise ManifestError(f"cannot read manifest: {path}") from exc
    digest = hashlib.sha256(raw).hexdigest()
    if digest != EXPECTED_MANIFEST_SHA256:
        raise ManifestError(
            "V7 manifest hash drifted; refusing to plot a different result "
            f"({digest})"
        )
    try:
        manifest = json.loads(raw)
    except json.JSONDecodeError as exc:
        raise ManifestError(f"invalid JSON manifest: {path}") from exc
    if not isinstance(manifest, dict):
        raise ManifestError("manifest root must be an object")
    if manifest.get("protocol", {}).get("protocol_id") != EXPECTED_PROTOCOL_ID:
        raise ManifestError("unexpected V7 protocol id")
    if manifest.get("status") != EXPECTED_STATUS or manifest.get("ok") is not False:
        raise ManifestError("V7 result is not the expected formal FAIL")
    if manifest.get("conclusion") != EXPECTED_CONCLUSION:
        raise ManifestError("unexpected V7 conclusion")
    selected = manifest.get("selected_pair", {})
    if selected.get("candidate_id") is not None:
        raise ManifestError("V7 unexpectedly contains a selected candidate")
    contract = manifest.get("detector_contract", {})
    if contract.get("sensors_per_pair") != 2:
        raise ManifestError("V7 does not preserve the two-sensor contract")
    if contract.get("sensor_roles") != ["heel", "forefoot"]:
        raise ManifestError("V7 sensor roles are not heel plus forefoot")
    return manifest


def _timing_value_ms(
    row: Mapping[str, Any],
    *,
    metric_key: str,
    reference_key: str,
    predicted_key: str,
    context: str,
) -> tuple[float | None, int]:
    reference = _integer(row, reference_key, context)
    predicted = _integer(row, predicted_key, context)
    missing = max(reference - predicted, 0)
    value_s = _number(row, metric_key, context)
    if value_s >= SENTINEL_FLOOR or missing:
        return None, missing
    return 1000.0 * value_s, 0


def extract_diagnostic(manifest: Mapping[str, Any]) -> dict[str, Any]:
    """Extract paired scalar diagnostics without exposing sentinel values."""

    contract = _required(manifest, "detector_contract", "manifest")
    if not isinstance(contract, Mapping):
        raise ManifestError("manifest.detector_contract must be an object")
    if contract.get("sensors_per_pair") != 2:
        raise ManifestError("expected exactly two detector sensors per pair")

    cadence_rows: dict[str, list[Mapping[str, Any]]] = {}
    for label, section_key, _color in CADENCES:
        section = _required(manifest, section_key, "manifest")
        if not isinstance(section, Mapping):
            raise ManifestError(f"manifest.{section_key} must be an object")
        rows = _required(section, "rows", section_key)
        if not isinstance(rows, list) or not rows:
            raise ManifestError(f"{section_key}.rows must be a non-empty list")
        if not all(isinstance(row, Mapping) for row in rows):
            raise ManifestError(f"{section_key}.rows contains a non-object")
        cadence_rows[label] = rows

    ids_10 = [str(_required(row, "candidate_id", "10ms.row")) for row in cadence_rows["10 ms"]]
    ids_1 = [str(_required(row, "candidate_id", "1ms.row")) for row in cadence_rows["1 ms"]]
    if ids_10 != ids_1 or len(ids_10) != len(set(ids_10)):
        raise ManifestError("candidate identities/order differ between cadences")

    expected_dt = {"10 ms": 0.01, "1 ms": 0.001}
    paired: list[dict[str, Any]] = []
    for index, candidate_id in enumerate(ids_10):
        cadence_values: dict[str, dict[str, Any]] = {}
        for label, _section_key, _color in CADENCES:
            row = cadence_rows[label][index]
            context = f"{label}.{candidate_id}"
            if not math.isclose(
                _number(row, "sample_dt_s", context),
                expected_dt[label],
                rel_tol=0.0,
                abs_tol=1.0e-12,
            ):
                raise ManifestError(f"unexpected sample cadence for {context}")
            hs_max_ms, missing_hs = _timing_value_ms(
                row,
                metric_key="max_abs_hs_error_s",
                reference_key="reference_hs_count",
                predicted_key="predicted_hs_count",
                context=context,
            )
            to_max_ms, missing_to = _timing_value_ms(
                row,
                metric_key="max_abs_toe_off_error_s",
                reference_key="reference_to_count",
                predicted_key="predicted_to_count",
                context=context,
            )
            clear_ms = 1000.0 * _number(
                row,
                "minimum_causal_toe_clear_before_next_hs_onset_s",
                context,
            )
            # Values at machine epsilon below zero represent the zero boundary.
            if abs(clear_ms) < 1.0e-8:
                clear_ms = 0.0
            cadence_values[label] = {
                "accepted_hs": _integer(row, "predicted_hs_count", context),
                "valid_cycles": _integer(
                    row, "observed_valid_cycle_count", context
                ),
                "hs_max_ms": hs_max_ms,
                "missing_hs": missing_hs,
                "to_max_ms": to_max_ms,
                "missing_to": missing_to,
                "both_off_samples": _integer(
                    row,
                    "transfer_both_latches_off_sample_count",
                    context,
                ),
                "causal_clear_ms": max(0.0, clear_ms),
                "recontact_episodes": _integer(
                    row, "toe_latch_recontact_episode_count", context
                ),
            }
        paired.append(
            {
                "candidate_id": candidate_id,
                "label": _short_label(candidate_id),
                "cadences": cadence_values,
            }
        )

    if any(
        value in (999.0, 999000.0)
        for item in paired
        for cadence in item["cadences"].values()
        for value in cadence.values()
    ):
        raise ManifestError("sentinel leaked into extracted plot diagnostics")

    return {
        "candidates": paired,
        "status": str(_required(manifest, "status", "manifest")),
        "conclusion": str(_required(manifest, "conclusion", "manifest")),
        "sensor_contract": "1 heel + 1 forefoot",
        "sensors_per_pair": 2,
        "thresholds": {
            "accepted_hs": 51,
            "valid_cycles": 50,
            "hs_max_ms": 50.0,
            "to_max_ms": 80.0,
            "both_off_samples": 0,
            "causal_clear_ms": 30.0,
        },
    }


def _annotate_bars(axis: Any, bars: Any, *, decimals: int = 0) -> None:
    for bar in bars:
        height = float(bar.get_height())
        axis.annotate(
            f"{height:.{decimals}f}",
            (bar.get_x() + bar.get_width() / 2.0, height),
            xytext=(0, 3),
            textcoords="offset points",
            ha="center",
            va="bottom",
            fontsize=6.5,
            rotation=90 if height >= 100.0 else 0,
        )


def _paired_bars(
    axis: Any,
    candidates: Sequence[Mapping[str, Any]],
    *,
    key: str,
    title: str,
    ylabel: str,
    threshold: float | None = None,
    threshold_relation: str = "",
    decimals: int = 0,
    missing_key: str | None = None,
) -> None:
    import numpy as np

    positions = np.arange(len(candidates), dtype=float)
    width = 0.36
    finite_values: list[float] = []
    missing_annotations: list[tuple[float, str, str]] = []
    for cadence_index, (label, _section, color) in enumerate(CADENCES):
        offset = (cadence_index - 0.5) * width
        values: list[float] = []
        for index, candidate in enumerate(candidates):
            cadence = candidate["cadences"][label]
            raw = cadence[key]
            if raw is None:
                values.append(float("nan"))
                count = int(cadence[missing_key]) if missing_key else 0
                event = "HS" if missing_key == "missing_hs" else "TO"
                missing_annotations.append(
                    (positions[index] + offset, f"−{count} {event}", color)
                )
            else:
                value = float(raw)
                values.append(value)
                finite_values.append(value)
        bars = axis.bar(
            positions + offset,
            values,
            width,
            color=color,
            label=label,
            alpha=0.92,
        )
        _annotate_bars(axis, bars, decimals=decimals)

    if threshold is not None:
        axis.axhline(
            threshold,
            color="#303030",
            linestyle="--",
            linewidth=1.1,
            label=f"soglia {threshold_relation} {threshold:g}",
        )
    top = max(finite_values + ([threshold] if threshold is not None else []) + [1.0])
    axis.set_ylim(0.0, top * (1.38 if missing_annotations else 1.22))
    for x_value, text_value, color in missing_annotations:
        axis.annotate(
            text_value,
            (x_value, top * 1.07),
            ha="center",
            va="bottom",
            fontsize=7,
            fontweight="bold",
            color=color,
            rotation=90,
        )
    axis.set_title(title, fontweight="bold")
    axis.set_ylabel(ylabel)
    axis.grid(axis="y", alpha=0.22)


def plot_diagnostic(diagnostic: Mapping[str, Any], output_dir: Path) -> Path:
    """Render the six requested metrics into a brand-new directory."""

    if output_dir.exists():
        raise NoClobberError(f"refusing to overwrite existing path: {output_dir}")

    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np

    candidates = list(diagnostic["candidates"])
    thresholds = diagnostic["thresholds"]
    labels = [str(candidate["label"]) for candidate in candidates]
    positions = np.arange(len(candidates), dtype=float)
    figure, axes = plt.subplots(3, 2, figsize=(20, 15.5), sharex=True)

    counts_axis = axes[0, 0]
    width = 0.19
    count_specs = (
        ("10 ms", "accepted_hs", -1.5, "#2A9D8F", "10 ms · HS"),
        ("10 ms", "valid_cycles", -0.5, "#76C7B7", "10 ms · cicli"),
        ("1 ms", "accepted_hs", 0.5, "#E76F51", "1 ms · HS"),
        ("1 ms", "valid_cycles", 1.5, "#F4A261", "1 ms · cicli"),
    )
    for cadence, key, multiplier, color, legend in count_specs:
        values = [candidate["cadences"][cadence][key] for candidate in candidates]
        bars = counts_axis.bar(
            positions + multiplier * width,
            values,
            width,
            color=color,
            label=legend,
        )
        _annotate_bars(counts_axis, bars)
    counts_axis.axhline(
        thresholds["accepted_hs"],
        color="#365486",
        linestyle=":",
        linewidth=1.1,
    )
    counts_axis.axhline(
        thresholds["valid_cycles"],
        color="#303030",
        linestyle="--",
        linewidth=1.1,
    )
    counts_axis.set_ylim(44.0, 53.4)
    counts_axis.set_title("Eventi HS e cicli completi", fontweight="bold")
    counts_axis.set_ylabel("conteggio (attesi: 51 HS, 50 cicli)")
    counts_axis.grid(axis="y", alpha=0.22)
    counts_axis.legend(loc="lower left", ncol=2, fontsize=8)

    _paired_bars(
        axes[0, 1],
        candidates,
        key="hs_max_ms",
        title="Errore massimo Heel Strike — solo confronti completi",
        ylabel="errore assoluto [ms]",
        threshold=thresholds["hs_max_ms"],
        threshold_relation="≤",
        decimals=1,
        missing_key="missing_hs",
    )
    _paired_bars(
        axes[1, 0],
        candidates,
        key="to_max_ms",
        title="Errore massimo Toe Off",
        ylabel="errore assoluto [ms]",
        threshold=thresholds["to_max_ms"],
        threshold_relation="≤",
        decimals=1,
        missing_key="missing_to",
    )
    _paired_bars(
        axes[1, 1],
        candidates,
        key="both_off_samples",
        title="Trasferimento: heel e forefoot entrambi OFF",
        ylabel="campioni (richiesto: 0)",
        threshold=thresholds["both_off_samples"],
        threshold_relation="=",
    )
    _paired_bars(
        axes[2, 0],
        candidates,
        key="causal_clear_ms",
        title="Clearance causale del forefoot prima del successivo HS",
        ylabel="margine minimo [ms]",
        threshold=thresholds["causal_clear_ms"],
        threshold_relation="≥",
        decimals=0,
    )
    _paired_bars(
        axes[2, 1],
        candidates,
        key="recontact_episodes",
        title="Ricontatti forefoot durante swing (diagnostica)",
        ylabel="episodi; non è un gate autonomo",
    )

    for axis in axes.flat:
        axis.set_xticks(positions, labels, rotation=38, ha="right")
    axes[0, 1].legend(loc="upper right", fontsize=8)

    figure.suptitle(
        "V7 — detector semplice a due sensori (1 heel + 1 forefoot)\n"
        "ESITO: FAIL · nessun candidato supera insieme i gate a 10 ms e 1 ms",
        fontsize=16,
        fontweight="bold",
        color="#9B2226",
    )
    figure.text(
        0.5,
        0.012,
        (
            "Le annotazioni −N HS/TO sostituiscono la sentinella 999: in assenza "
            "di conteggi completi non viene mostrato un falso errore temporale. "
            "La mesh è usata solo offline per il posizionamento; ogni coppia "
            "valutata resta composta da due sole sfere detector."
        ),
        ha="center",
        va="bottom",
        fontsize=9.5,
        color="#404040",
    )
    figure.tight_layout(rect=(0.02, 0.055, 0.98, 0.925))

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
    diagnostic = extract_diagnostic(load_frozen_manifest(manifest_path))
    output = plot_diagnostic(diagnostic, output_dir)
    print(
        json.dumps(
            {
                "input_manifest": str(manifest_path),
                "input_sha256": EXPECTED_MANIFEST_SHA256,
                "output": str(output),
                "status": diagnostic["status"],
                "conclusion": diagnostic["conclusion"],
                "sensor_contract": diagnostic["sensor_contract"],
                "sentinel_bars_rendered": False,
            },
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
