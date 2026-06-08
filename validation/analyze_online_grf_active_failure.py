"""Diagnose why an active onlineGRF run differs from prescribed dynamics."""

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

from output import _read_storage_table
from path_resolver import resolve_repo_path


AXES = ("x", "y", "z")
SIDES = {
    "left": ("ground_force1", "ground_torque1"),
    "right": ("ground_force2", "ground_torque2"),
}


def _find(run_dir: Path, suffix: str) -> Path:
    matches = sorted(run_dir.glob(f"*_{suffix}"))
    if suffix == "states.sto":
        matches = [path for path in matches if not path.name.endswith("_sea_states.sto")]
    if len(matches) != 1:
        raise ValueError(f"Expected one *_{suffix} in {run_dir}, found {matches}")
    return matches[0]


def _columns(path: Path) -> tuple[np.ndarray, dict[str, np.ndarray]]:
    time, names, data = _read_storage_table(str(path))
    return time, {name: data[:, index] for index, name in enumerate(names)}


def _interp(
    source_time: np.ndarray,
    columns: dict[str, np.ndarray],
    target_time: np.ndarray,
    names: list[str],
) -> np.ndarray:
    return np.column_stack(
        [np.interp(target_time, source_time, columns[name]) for name in names]
    )


def _stats(values: np.ndarray) -> dict[str, Any]:
    array = np.asarray(values, dtype=float)
    absolute = np.abs(array)
    return {
        "mean": np.mean(array, axis=0).tolist(),
        "mean_abs": np.mean(absolute, axis=0).tolist(),
        "p95_abs": np.percentile(absolute, 95, axis=0).tolist(),
        "max_abs": np.max(absolute, axis=0).tolist(),
        "rms": np.sqrt(np.mean(array * array, axis=0)).tolist(),
    }


def _correlation(a: np.ndarray, b: np.ndarray) -> list[float | None]:
    result: list[float | None] = []
    for axis in range(a.shape[1]):
        if np.std(a[:, axis]) < 1e-12 or np.std(b[:, axis]) < 1e-12:
            result.append(None)
        else:
            result.append(float(np.corrcoef(a[:, axis], b[:, axis])[0, 1]))
    return result


def _comparison(predicted: np.ndarray, reference: np.ndarray) -> dict[str, Any]:
    error = predicted - reference
    return {
        "predicted": _stats(predicted),
        "reference": _stats(reference),
        "error": _stats(error),
        "correlation": _correlation(predicted, reference),
    }


def _integral(time: np.ndarray, values: np.ndarray) -> list[float]:
    trapezoid = getattr(np, "trapezoid", None)
    if trapezoid is None:
        trapezoid = np.trapz
    return [float(trapezoid(values[:, axis], time)) for axis in range(3)]


def _online(run_dir: Path) -> tuple[np.ndarray, dict[str, dict[str, np.ndarray]]]:
    time, columns = _columns(_find(run_dir, "online_grf.sto"))
    result: dict[str, dict[str, np.ndarray]] = {}
    for side in SIDES:
        result[side] = {
            key: np.column_stack(
                [columns[f"{side}_{key}_{axis}"] for axis in AXES]
            )
            for key in ("force", "moment", "cop")
        }
        for key in ("normal_force", "penetration", "slip_speed", "in_contact"):
            result[side][key] = columns[f"{side}_{key}"]
    return time, result


def _prescribed(
    grf_file: Path,
    target_time: np.ndarray,
) -> dict[str, dict[str, np.ndarray]]:
    source_time, columns = _columns(grf_file)
    result: dict[str, dict[str, np.ndarray]] = {}
    for side, (force_prefix, torque_prefix) in SIDES.items():
        force = _interp(
            source_time,
            columns,
            target_time,
            [f"{force_prefix}_v{axis}" for axis in AXES],
        )
        point = _interp(
            source_time,
            columns,
            target_time,
            [f"{force_prefix}_p{axis}" for axis in AXES],
        )
        free_torque = _interp(
            source_time,
            columns,
            target_time,
            [f"{torque_prefix}_{axis}" for axis in AXES],
        )
        result[side] = {
            "force": force,
            "cop": point,
            "free_torque": free_torque,
            "moment": np.cross(point, force) + free_torque,
        }
    return result


def _net(source: dict[str, dict[str, np.ndarray]], key: str) -> np.ndarray:
    return sum(source[side][key] for side in SIDES)


def _reserve_ranking(
    active_dir: Path,
    sensor_dir: Path,
) -> tuple[list[dict[str, Any]], dict[str, np.ndarray]]:
    active_time, active = _columns(_find(active_dir, "reserve_torques.sto"))
    sensor_time, sensor = _columns(_find(sensor_dir, "reserve_torques.sto"))
    ranking = []
    traces: dict[str, np.ndarray] = {}
    for name, active_values in active.items():
        sensor_values = np.interp(active_time, sensor_time, sensor[name])
        active_p95 = float(np.percentile(np.abs(active_values), 95))
        sensor_p95 = float(np.percentile(np.abs(sensor_values), 95))
        delta = active_values - sensor_values
        ranking.append(
            {
                "coordinate": name.removesuffix("_reserve_torque"),
                "active_p95_abs": active_p95,
                "sensor_p95_abs": sensor_p95,
                "active_sensor_ratio": (
                    active_p95 / sensor_p95 if sensor_p95 > 1e-9 else None
                ),
                "delta_p95_abs": float(np.percentile(np.abs(delta), 95)),
            }
        )
        traces[name.removesuffix("_reserve_torque")] = delta
    ranking.sort(key=lambda item: item["delta_p95_abs"], reverse=True)
    return ranking, traces


def _tau_ranking(active_dir: Path, sensor_dir: Path) -> list[dict[str, Any]]:
    active_time, active = _columns(_find(active_dir, "tau_bio.sto"))
    sensor_time, sensor = _columns(_find(sensor_dir, "tau_bio.sto"))
    ranking = []
    for name, active_values in active.items():
        sensor_values = np.interp(active_time, sensor_time, sensor[name])
        delta = active_values - sensor_values
        ranking.append(
            {
                "coordinate": name,
                "active_p95_abs": float(np.percentile(np.abs(active_values), 95)),
                "sensor_p95_abs": float(np.percentile(np.abs(sensor_values), 95)),
                "delta_p95_abs": float(np.percentile(np.abs(delta), 95)),
            }
        )
    ranking.sort(key=lambda item: item["delta_p95_abs"], reverse=True)
    return ranking


def _state_ranking(active_dir: Path, sensor_dir: Path) -> list[dict[str, Any]]:
    active_time, active = _columns(_find(active_dir, "states.sto"))
    sensor_time, sensor = _columns(_find(sensor_dir, "states.sto"))
    ranking = []
    for name, active_values in active.items():
        if not name.endswith(("_q", "_qdot")):
            continue
        sensor_values = np.interp(active_time, sensor_time, sensor[name])
        delta = active_values - sensor_values
        ranking.append(
            {
                "state": name,
                "delta_mean_abs": float(np.mean(np.abs(delta))),
                "delta_p95_abs": float(np.percentile(np.abs(delta), 95)),
                "delta_max_abs": float(np.max(np.abs(delta))),
            }
        )
    ranking.sort(key=lambda item: item["delta_p95_abs"], reverse=True)
    return ranking


def _scalar_correlation(a: np.ndarray, b: np.ndarray) -> float | None:
    if np.std(a) < 1e-12 or np.std(b) < 1e-12:
        return None
    return float(np.corrcoef(a, b)[0, 1])


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--active-dir", required=True)
    parser.add_argument("--sensor-dir", required=True)
    parser.add_argument("--grf-file", required=True)
    parser.add_argument(
        "--report",
        default="results/online_grf_active_failure_analysis.json",
    )
    args = parser.parse_args()

    active_dir = resolve_repo_path(args.active_dir)
    sensor_dir = resolve_repo_path(args.sensor_dir)
    active_time, active = _online(active_dir)
    sensor_time, sensor_raw = _online(sensor_dir)
    sensor = {
        side: {
            key: (
                np.column_stack(
                    [
                        np.interp(active_time, sensor_time, values[:, axis])
                        for axis in range(3)
                    ]
                )
                if values.ndim == 2
                else np.interp(active_time, sensor_time, values)
            )
            for key, values in sensor_raw[side].items()
        }
        for side in SIDES
    }
    prescribed = _prescribed(resolve_repo_path(args.grf_file), active_time)

    force_report: dict[str, Any] = {}
    moment_report: dict[str, Any] = {}
    cop_report: dict[str, Any] = {}
    for side in SIDES:
        force_report[side] = {
            "active_vs_prescribed": _comparison(
                active[side]["force"], prescribed[side]["force"]
            ),
            "sensor_vs_prescribed": _comparison(
                sensor[side]["force"], prescribed[side]["force"]
            ),
            "active_vs_sensor": _comparison(
                active[side]["force"], sensor[side]["force"]
            ),
            "active_impulse_ns": _integral(active_time, active[side]["force"]),
            "prescribed_impulse_ns": _integral(
                active_time, prescribed[side]["force"]
            ),
        }
        moment_report[side] = {
            "active_vs_prescribed": _comparison(
                active[side]["moment"], prescribed[side]["moment"]
            ),
            "sensor_vs_prescribed": _comparison(
                sensor[side]["moment"], prescribed[side]["moment"]
            ),
        }
        cop_report[side] = {
            "active_vs_prescribed": _comparison(
                active[side]["cop"], prescribed[side]["cop"]
            ),
            "sensor_vs_prescribed": _comparison(
                sensor[side]["cop"], prescribed[side]["cop"]
            ),
        }
        force_report[side]["active_vs_sensor_contact_state"] = {
            key: {
                "active": _stats(active[side][key][:, None]),
                "sensor": _stats(sensor[side][key][:, None]),
                "delta": _stats(
                    (active[side][key] - sensor[side][key])[:, None]
                ),
            }
            for key in ("normal_force", "penetration", "slip_speed", "in_contact")
        }

    reserve_ranking, reserve_delta = _reserve_ranking(active_dir, sensor_dir)
    net_active_force = _net(active, "force")
    net_sensor_force = _net(sensor, "force")
    net_prescribed_force = _net(prescribed, "force")
    external_force_deficit = net_prescribed_force - net_active_force
    pelvis_reserve_correlations = {}
    for coord, axis in (("pelvis_tx", 0), ("pelvis_ty", 1), ("pelvis_tz", 2)):
        delta = reserve_delta.get(coord)
        pelvis_reserve_correlations[coord] = {
            "with_prescribed_minus_active_force": (
                _scalar_correlation(delta, external_force_deficit[:, axis])
                if delta is not None
                else None
            ),
            "with_sensor_minus_active_online_force": (
                _scalar_correlation(delta, net_sensor_force[:, axis] - net_active_force[:, axis])
                if delta is not None
                else None
            ),
        }

    report = {
        "active_dir": str(active_dir),
        "sensor_dir": str(sensor_dir),
        "grf_file": str(resolve_repo_path(args.grf_file)),
        "samples": int(len(active_time)),
        "time_range_s": [float(active_time[0]), float(active_time[-1])],
        "force_axes": list(AXES),
        "force": force_report,
        "net_force": {
            "active_vs_prescribed": _comparison(
                net_active_force, net_prescribed_force
            ),
            "sensor_online_vs_prescribed": _comparison(
                net_sensor_force, net_prescribed_force
            ),
            "active_vs_sensor_online": _comparison(
                net_active_force, net_sensor_force
            ),
            "active_impulse_ns": _integral(active_time, net_active_force),
            "prescribed_impulse_ns": _integral(active_time, net_prescribed_force),
        },
        "moment_about_ground_origin": moment_report,
        "cop": cop_report,
        "reserve_delta_ranking": reserve_ranking,
        "tau_bio_delta_ranking": _tau_ranking(active_dir, sensor_dir),
        "state_delta_ranking": _state_ranking(active_dir, sensor_dir),
        "pelvis_reserve_delta_correlations": pelvis_reserve_correlations,
    }
    report_path = resolve_repo_path(args.report)
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
