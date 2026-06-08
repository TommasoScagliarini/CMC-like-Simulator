"""Audit physical viability of saved prescribed/sensor/active onlineGRF runs.

This validator is read-only: it consumes simulator outputs and compares every
run against the IK reference and prescribed GRF oracle. It intentionally keeps
sensor accuracy separate from active-contact physical viability.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from config import SimulatorConfig
from kinematics_interpolator import KinematicsInterpolator
from output import _read_storage_table
from path_resolver import resolve_repo_path
from setup_io import read_setup_xml
from validation.validate_online_grf import _external_grf, _external_wrench, _wrench_metrics


def _finite(value: float) -> float | None:
    return float(value) if math.isfinite(float(value)) else None


def _slope(time: np.ndarray, values: np.ndarray) -> float:
    if len(time) < 2 or float(time[-1] - time[0]) <= 0.0:
        return float("nan")
    return float(np.polyfit(time - time[0], values, 1)[0])


def _integral(values: np.ndarray, time: np.ndarray) -> float:
    trapezoid = getattr(np, "trapezoid", None)
    if trapezoid is None:
        trapezoid = np.trapz
    return float(trapezoid(values, time))


def _stats(values: np.ndarray) -> dict[str, float | None]:
    finite = np.asarray(values, dtype=float)
    finite = finite[np.isfinite(finite)]
    if finite.size == 0:
        return {"max_abs": None, "mean_abs": None, "p95_abs": None}
    absolute = np.abs(finite)
    return {
        "max_abs": float(np.max(absolute)),
        "mean_abs": float(np.mean(absolute)),
        "p95_abs": float(np.percentile(absolute, 95)),
    }


def _find_output(run_dir: Path, suffix: str, required: bool = True) -> Path | None:
    matches = sorted(run_dir.glob(f"*_{suffix}"))
    if suffix == "states.sto":
        matches = [path for path in matches if not path.name.endswith("_sea_states.sto")]
    if not matches:
        if required:
            raise FileNotFoundError(f"No *_{suffix} found in {run_dir}")
        return None
    if len(matches) > 1:
        raise ValueError(f"Multiple *_{suffix} files found in {run_dir}: {matches}")
    return matches[0]


def _read_status(run_dir: Path) -> dict[str, str]:
    path = _find_output(run_dir, "run_status.txt", required=False)
    if path is None:
        return {"status": "missing"}
    result: dict[str, str] = {}
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        key, separator, value = line.partition("=")
        if separator:
            result[key.strip()] = value.strip()
    return result


def _column(
    columns: list[str],
    data: np.ndarray,
    name: str,
    *,
    required: bool = True,
) -> np.ndarray | None:
    try:
        return data[:, columns.index(name)]
    except ValueError:
        if required:
            raise ValueError(f"Missing required column {name!r}.") from None
        return None


def _make_kinematics(setup) -> KinematicsInterpolator:
    cfg = SimulatorConfig()
    cfg.model_bundle_dir = str(setup.model_file.parent)
    cfg.model_file = str(setup.model_file)
    cfg.kinematics_file = str(setup.kinematics_file)
    cfg.t_start = float(setup.t_start)
    cfg.t_end = float(setup.t_end)
    return KinematicsInterpolator(cfg)


def _reference_pelvis_ty(
    kinematics: KinematicsInterpolator,
    times: np.ndarray,
) -> np.ndarray:
    return np.asarray(
        [kinematics.get(float(time))[0]["pelvis_ty"] for time in times],
        dtype=float,
    )


def _prescribed_impulse(setup, times: np.ndarray) -> dict[str, float]:
    reference = _external_grf(setup, times)
    return {
        side: _integral(reference[side][:, 1], times)
        for side in ("left", "right")
    }


def _run_metrics(
    label: str,
    run_dir: Path,
    setup,
    kinematics: KinematicsInterpolator,
    fall_threshold: float,
) -> tuple[dict[str, Any], dict[str, np.ndarray]]:
    states_path = _find_output(run_dir, "states.sto")
    times, state_columns, states = _read_storage_table(str(states_path))
    pelvis_ty = _column(state_columns, states, "pelvis_ty_q")
    reference_ty = _reference_pelvis_ty(kinematics, times)
    sink = reference_ty - pelvis_ty

    result: dict[str, Any] = {
        "label": label,
        "directory": str(run_dir.resolve()),
        "status": _read_status(run_dir),
        "samples": int(len(times)),
        "time_range_s": [float(times[0]), float(times[-1])],
        "duration_s": float(times[-1] - times[0]) if len(times) > 1 else 0.0,
        "pelvis_ty": {
            "min_m": float(np.min(pelvis_ty)),
            "reference_min_m": float(np.min(reference_ty)),
            "drift_rms_m": float(np.sqrt(np.mean((pelvis_ty - reference_ty) ** 2))),
            "end_drift_m": float(pelvis_ty[-1] - reference_ty[-1]),
            "max_sink_m": float(max(0.0, np.max(sink))),
            "sink_slope_m_per_s": _finite(_slope(times, sink)),
            "fall_threshold_m": float(fall_threshold),
            "fraction_below_fall_threshold": float(np.mean(pelvis_ty < fall_threshold)),
        },
        "prescribed_vertical_impulse_ns": _prescribed_impulse(setup, times),
    }

    reserve_path = _find_output(run_dir, "reserve_torques.sto", required=False)
    reserve_ty = np.full_like(times, np.nan)
    if reserve_path is not None:
        reserve_time, reserve_columns, reserve_data = _read_storage_table(str(reserve_path))
        source = _column(
            reserve_columns,
            reserve_data,
            "pelvis_ty_reserve_torque",
            required=False,
        )
        if source is not None:
            reserve_ty = np.interp(times, reserve_time, source)
    result["pelvis_ty_reserve"] = {
        **_stats(reserve_ty),
        "slope_abs_per_s": _finite(_slope(times, np.abs(reserve_ty))),
    }

    recruitment_path = _find_output(run_dir, "recruitment.sto", required=False)
    recruitment: dict[str, Any] = {}
    if recruitment_path is not None:
        rec_time, rec_columns, rec_data = _read_storage_table(str(recruitment_path))
        for name in ("tau_reserve_norm", "unactuated_reserve_norm"):
            values = _column(rec_columns, rec_data, name, required=False)
            if values is not None:
                recruitment[name] = {
                    **_stats(values),
                    "slope_abs_per_s": _finite(_slope(rec_time, np.abs(values))),
                }
    result["recruitment"] = recruitment

    grf_path = _find_output(run_dir, "online_grf.sto", required=False)
    total_vertical = np.full_like(times, np.nan)
    if grf_path is not None:
        grf_time, grf_columns, grf_data = _read_storage_table(str(grf_path))
        total_vertical = np.zeros_like(times)
        result["online_grf"] = {}
        predicted_wrench: dict[str, dict[str, np.ndarray]] = {}
        online_impulse_total = 0.0
        oracle_impulse_total = 0.0
        for side in ("left", "right"):
            predicted_wrench[side] = {
                key: np.column_stack(
                    [
                        _column(grf_columns, grf_data, f"{side}_{key}_{axis}")
                        for axis in ("x", "y", "z")
                    ]
                )
                for key in ("force", "moment", "cop")
            }
            force_y = _column(grf_columns, grf_data, f"{side}_force_y")
            penetration = _column(grf_columns, grf_data, f"{side}_penetration")
            slip = _column(grf_columns, grf_data, f"{side}_slip_speed")
            in_contact = _column(grf_columns, grf_data, f"{side}_in_contact") > 0.5
            impulse = _integral(force_y, grf_time)
            oracle_impulse = float(result["prescribed_vertical_impulse_ns"][side])
            online_impulse_total += impulse
            oracle_impulse_total += oracle_impulse
            contact_penetration = penetration[in_contact]
            contact_slip = slip[in_contact]
            result["online_grf"][side] = {
                "vertical_impulse_ns": impulse,
                "prescribed_impulse_ratio": (
                    impulse / oracle_impulse if abs(oracle_impulse) > 1e-12 else None
                ),
                "peak_vertical_force_n": float(np.max(force_y)),
                "contact_fraction": float(np.mean(in_contact)),
                "penetration_max_m": float(np.max(penetration)),
                "penetration_p95_contact_m": (
                    float(np.percentile(contact_penetration, 95))
                    if contact_penetration.size
                    else 0.0
                ),
                "slip_speed_p95_contact_m_per_s": (
                    float(np.percentile(contact_slip, 95))
                    if contact_slip.size
                    else 0.0
                ),
            }
            total_vertical += np.interp(times, grf_time, force_y)
        result["online_grf"]["total"] = {
            "vertical_impulse_ns": online_impulse_total,
            "prescribed_impulse_ratio": (
                online_impulse_total / oracle_impulse_total
                if abs(oracle_impulse_total) > 1e-12
                else None
            ),
            "peak_vertical_force_n": float(np.max(total_vertical)),
        }
        result["online_grf_wrench"] = _wrench_metrics(
            _external_wrench(setup, grf_time),
            predicted_wrench,
            grf_time,
            np.ones(len(grf_time), dtype=bool),
            20.0,
        )

    traces = {
        "time": times,
        "sink": sink,
        "reserve_ty": reserve_ty,
        "total_vertical": total_vertical,
    }
    return result, traces


def _comparisons(runs: dict[str, dict[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    active = runs.get("online")
    if active is None:
        return result
    active_p95 = active["pelvis_ty_reserve"]["p95_abs"]
    for baseline_name in ("sensor", "prescribed"):
        baseline = runs.get(baseline_name)
        if baseline is None:
            continue
        baseline_p95 = baseline["pelvis_ty_reserve"]["p95_abs"]
        result[f"online_vs_{baseline_name}"] = {
            "same_time_window": bool(
                np.allclose(
                    active["time_range_s"],
                    baseline["time_range_s"],
                    rtol=0.0,
                    atol=1.0e-9,
                )
            ),
            "pelvis_ty_reserve_p95_ratio": (
                active_p95 / baseline_p95
                if active_p95 is not None
                and baseline_p95 is not None
                and baseline_p95 > 1e-12
                else None
            ),
            "max_sink_difference_m": (
                active["pelvis_ty"]["max_sink_m"]
                - baseline["pelvis_ty"]["max_sink_m"]
            ),
        }
    return result


def _plot(path: Path, traces: dict[str, dict[str, np.ndarray]]) -> None:
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("[forward-drift] matplotlib unavailable; skipping plot.")
        return

    figure, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=False)
    for label, trace in traces.items():
        relative_time = trace["time"] - trace["time"][0]
        axes[0].plot(relative_time, trace["sink"], label=label)
        axes[1].plot(relative_time, trace["reserve_ty"], label=label)
        axes[2].plot(relative_time, trace["total_vertical"], label=label)
    axes[0].set_ylabel("pelvis_ty sink [m]")
    axes[1].set_ylabel("pelvis_ty reserve [N]")
    axes[2].set_ylabel("online vertical GRF [N]")
    axes[2].set_xlabel("time from run start [s]")
    for axis in axes:
        axis.grid(True, alpha=0.3)
        axis.legend()
    figure.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(path, dpi=160)
    plt.close(figure)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--setup", required=True)
    parser.add_argument("--online-dir", required=True)
    parser.add_argument("--sensor-dir", default="")
    parser.add_argument("--prescribed-dir", default="")
    parser.add_argument("--fall-threshold", type=float, default=0.55)
    parser.add_argument(
        "--report",
        default="results/online_grf_forward_drift.json",
    )
    parser.add_argument("--plot", default="")
    args = parser.parse_args()

    setup = read_setup_xml(args.setup)
    if setup.external_loads_xml is None:
        raise ValueError("Forward-drift validation requires prescribed ExternalLoads.")
    kinematics = _make_kinematics(setup)
    directories = {"online": args.online_dir}
    if args.sensor_dir:
        directories["sensor"] = args.sensor_dir
    if args.prescribed_dir:
        directories["prescribed"] = args.prescribed_dir

    runs: dict[str, dict[str, Any]] = {}
    traces: dict[str, dict[str, np.ndarray]] = {}
    for label, raw_directory in directories.items():
        run_dir = resolve_repo_path(raw_directory)
        runs[label], traces[label] = _run_metrics(
            label,
            run_dir,
            setup,
            kinematics,
            args.fall_threshold,
        )

    report = {
        "setup": str(resolve_repo_path(args.setup)),
        "runs": runs,
        "comparisons": _comparisons(runs),
    }
    report_path = resolve_repo_path(args.report)
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    if args.plot:
        _plot(resolve_repo_path(args.plot), traces)
    print(f"onlineGRF forward-drift report: {report_path.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
