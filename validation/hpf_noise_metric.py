"""
hpf_noise_metric.py
===================
Post-processing script that computes the HPF-energy-ratio noise metric
on tau_input from existing .sto files.

The metric is:

    noise_hpf = energy(HPF(tau_input)) / energy(tau_input)

where HPF is a 4th-order Butterworth high-pass filter at a configurable
cutoff (default 50 Hz).  This measures the fraction of motor torque
energy that sits above the cutoff — i.e. chattering / high-frequency
noise — without being sensitive to phase lag like the old 1-R² metric.

Usage
-----
    # Analyze current results/ directory
    python validation/hpf_noise_metric.py

    # Analyze a specific sweep run
    python validation/hpf_noise_metric.py \
        --results-dir results/_sea_driver_sweep_20260418_052206/runs/screen_kk250_ka500_wn700_z0p7_td0

    # Custom cutoff and transient skip
    python validation/hpf_noise_metric.py --cutoff-hz 30 --skip-s 0.1

    # Re-analyze all sweep CSV rows and append HPF columns
    python validation/hpf_noise_metric.py --sweep-csv results/_sea_driver_sweep_20260418_052206/sweep_results.csv
"""
from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path

import numpy as np
from scipy.signal import butter, sosfiltfilt

REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from output import read_sto  # noqa: E402


def hpf_energy_ratio(
    signal: np.ndarray,
    dt: float,
    cutoff_hz: float = 50.0,
    order: int = 4,
) -> float:
    """
    Compute the fraction of signal energy above cutoff_hz.

    Uses a zero-phase Butterworth high-pass filter (sosfiltfilt) so there
    is no phase distortion.  Returns a value in [0, 1].
    """
    total_energy = float(np.sum(signal ** 2))
    if total_energy < 1e-30:
        return 0.0

    fs = 1.0 / dt
    nyquist = fs / 2.0
    if cutoff_hz >= nyquist * 0.95:
        # Cutoff too close to Nyquist — can't filter meaningfully
        return float("nan")

    sos = butter(order, cutoff_hz / nyquist, btype="high", output="sos")
    filtered = sosfiltfilt(sos, signal)
    hf_energy = float(np.sum(filtered ** 2))
    return hf_energy / total_energy


def analyze_sto(
    sea_torques_path: Path,
    sea_controls_path: Path,
    cutoff_hz: float = 50.0,
    skip_s: float = 0.1,
    f_opt: dict[str, float] | None = None,
) -> dict[str, float]:
    """
    Compute HPF noise metrics from a pair of .sto files.

    Returns a dict with keys like:
        knee_hpf_noise, ankle_hpf_noise,
        knee_hpf_noise_tau_input, ankle_hpf_noise_tau_input
    """
    results: dict[str, float] = {}

    if not sea_torques_path.is_file():
        return results
    if not sea_controls_path.is_file():
        return results

    time_t, cols_t, data_t, _ = read_sto(str(sea_torques_path))
    time_c, cols_c, data_c, _ = read_sto(str(sea_controls_path))

    if len(time_t) < 20:
        return results

    dt = float(time_t[1] - time_t[0]) if len(time_t) > 1 else 0.001

    # Skip initial transient
    mask = time_t >= (time_t[0] + skip_s)
    time_t = time_t[mask]
    data_t = data_t[mask]

    mask_c = time_c >= (time_c[0] + skip_s)
    time_c = time_c[mask_c]
    data_c = data_c[mask_c]

    if len(time_t) < 20 or len(time_c) < 2:
        return results

    col_idx_t = {name: i for i, name in enumerate(cols_t)}
    col_idx_c = {name: i for i, name in enumerate(cols_c)}

    joints = [
        ("knee", "SEA_Knee", "pros_knee_angle"),
        ("ankle", "SEA_Ankle", "pros_ankle_angle"),
    ]

    for label, sea_name, coord_name in joints:
        # tau_input (motor torque)
        tau_input_col = col_idx_t.get(f"{sea_name}_tau_motor")
        if tau_input_col is not None:
            tau_input = data_t[:, tau_input_col]
            ratio = hpf_energy_ratio(tau_input, dt, cutoff_hz)
            results[f"{label}_hpf_noise_tau_input"] = ratio

        # tau_spring (spring torque = SEA output)
        tau_spring_col = col_idx_t.get(f"{sea_name}_tau_spring")
        if tau_spring_col is not None:
            tau_spring = data_t[:, tau_spring_col]
            ratio = hpf_energy_ratio(tau_spring, dt, cutoff_hz)
            results[f"{label}_hpf_noise_tau_spring"] = ratio

        # tau_ref (control * F_opt) — if F_opt known
        control_col = col_idx_c.get(coord_name)
        fopt = (f_opt or {}).get(sea_name)
        if control_col is not None and fopt is not None:
            control_interp = np.interp(time_t, time_c, data_c[:, control_col])
            tau_ref = control_interp * fopt
            ratio = hpf_energy_ratio(tau_ref, dt, cutoff_hz)
            results[f"{label}_hpf_noise_tau_ref"] = ratio

    return results


def load_f_opt_from_model(model_path: Path) -> dict[str, float]:
    """Read F_opt for each SEA from the model XML."""
    from xml.etree import ElementTree as ET

    f_opt: dict[str, float] = {}
    if not model_path.is_file():
        return f_opt
    try:
        root = ET.parse(model_path).getroot()
    except ET.ParseError:
        return f_opt

    for element in root.iter():
        tag = element.tag.rsplit("}", 1)[-1]
        if tag != "SeriesElasticActuator":
            continue
        name = element.attrib.get("name", "")
        for child in element:
            child_tag = child.tag.rsplit("}", 1)[-1]
            if child_tag == "optimal_force" and child.text:
                try:
                    f_opt[name] = float(child.text.strip())
                except ValueError:
                    pass
    return f_opt


def analyze_single_dir(
    results_dir: Path,
    prefix: str,
    cutoff_hz: float,
    skip_s: float,
    f_opt: dict[str, float] | None,
) -> None:
    """Analyze a single results directory and print metrics."""
    torques_path = results_dir / f"{prefix}_sea_torques.sto"
    controls_path = results_dir / f"{prefix}_sea_controls.sto"

    metrics = analyze_sto(torques_path, controls_path, cutoff_hz, skip_s, f_opt)

    if not metrics:
        print(f"No data found in {results_dir}")
        return

    print(f"\nHPF noise metric (cutoff={cutoff_hz} Hz, skip={skip_s} s)")
    print(f"Directory: {results_dir}\n")
    print(f"{'Metric':<35s} {'Value':>10s}")
    print("-" * 47)
    for key, value in sorted(metrics.items()):
        if math.isnan(value):
            print(f"{key:<35s} {'NaN':>10s}")
        else:
            print(f"{key:<35s} {value:10.4f}  ({value * 100:.1f}%)")


def analyze_sweep_csv(
    csv_path: Path,
    cutoff_hz: float,
    skip_s: float,
    f_opt: dict[str, float] | None,
    prefix: str,
) -> None:
    """
    Re-analyze all runs referenced by a sweep CSV and write an augmented CSV
    with HPF noise columns appended.
    """
    sweep_dir = csv_path.parent
    runs_dir = sweep_dir / "runs"

    with csv_path.open("r", encoding="utf-8") as fh:
        reader = csv.DictReader(fh)
        original_fields = list(reader.fieldnames or [])
        rows = list(reader)

    hpf_fields = [
        "knee_hpf_noise_tau_input",
        "ankle_hpf_noise_tau_input",
        "knee_hpf_noise_tau_spring",
        "ankle_hpf_noise_tau_spring",
        "worst_hpf_noise_tau_input",
    ]
    out_fields = original_fields + [f for f in hpf_fields if f not in original_fields]

    analyzed = 0
    for row in rows:
        run_name = row.get("name", "")
        run_dir = runs_dir / run_name
        torques = run_dir / f"{prefix}_sea_torques.sto"
        controls = run_dir / f"{prefix}_sea_controls.sto"

        if not torques.is_file():
            for field in hpf_fields:
                row[field] = ""
            continue

        metrics = analyze_sto(torques, controls, cutoff_hz, skip_s, f_opt)
        for field in hpf_fields:
            if field == "worst_hpf_noise_tau_input":
                k = metrics.get("knee_hpf_noise_tau_input", float("nan"))
                a = metrics.get("ankle_hpf_noise_tau_input", float("nan"))
                val = max(k, a) if math.isfinite(k) and math.isfinite(a) else float("nan")
                row[field] = f"{val:.6f}" if math.isfinite(val) else ""
            elif field in metrics:
                val = metrics[field]
                row[field] = f"{val:.6f}" if math.isfinite(val) else ""
            else:
                row[field] = ""
        analyzed += 1

    out_path = csv_path.with_name(csv_path.stem + "_hpf.csv")
    with out_path.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=out_fields)
        writer.writeheader()
        writer.writerows(rows)

    print(f"Analyzed {analyzed}/{len(rows)} runs")
    print(f"Augmented CSV: {out_path}")

    # Summary statistics
    valid_knee = [
        float(r["knee_hpf_noise_tau_input"])
        for r in rows
        if r.get("knee_hpf_noise_tau_input", "").strip()
    ]
    valid_ankle = [
        float(r["ankle_hpf_noise_tau_input"])
        for r in rows
        if r.get("ankle_hpf_noise_tau_input", "").strip()
    ]
    if valid_knee:
        arr = np.array(valid_knee)
        print(f"\nKnee HPF noise:  min={arr.min():.4f}  "
              f"median={np.median(arr):.4f}  max={arr.max():.4f}")
    if valid_ankle:
        arr = np.array(valid_ankle)
        print(f"Ankle HPF noise: min={arr.min():.4f}  "
              f"median={np.median(arr):.4f}  max={arr.max():.4f}")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Compute HPF-energy-ratio noise metric on SEA tau_input"
    )
    parser.add_argument(
        "--results-dir",
        default="results",
        help="Directory with .sto files (default: results/)",
    )
    parser.add_argument(
        "--prefix",
        default="sim_output",
        help="STO file prefix (default: sim_output)",
    )
    parser.add_argument(
        "--cutoff-hz",
        type=float,
        default=50.0,
        help="HPF cutoff frequency in Hz (default: 50)",
    )
    parser.add_argument(
        "--skip-s",
        type=float,
        default=0.1,
        help="Seconds to skip at the start for transient (default: 0.1)",
    )
    parser.add_argument(
        "--model",
        default=None,
        help="Model .osim file for F_opt lookup (default: from config.py)",
    )
    parser.add_argument(
        "--sweep-csv",
        default=None,
        help="Path to sweep_results.csv to re-analyze all runs and produce "
             "an augmented CSV with HPF columns.",
    )
    args = parser.parse_args()

    # Load F_opt
    if args.model:
        model_path = Path(args.model)
    else:
        from config import SimulatorConfig
        model_path = REPO_ROOT / SimulatorConfig().model_file

    f_opt = load_f_opt_from_model(model_path)
    if f_opt:
        print(f"F_opt: {f_opt}")

    if args.sweep_csv:
        analyze_sweep_csv(
            Path(args.sweep_csv),
            args.cutoff_hz,
            args.skip_s,
            f_opt,
            args.prefix,
        )
    else:
        analyze_single_dir(
            Path(args.results_dir),
            args.prefix,
            args.cutoff_hz,
            args.skip_s,
            f_opt,
        )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
