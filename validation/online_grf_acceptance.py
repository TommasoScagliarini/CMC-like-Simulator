"""Aggregate onlineGRF validation reports into an explicit PASS/FAIL gate."""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from path_resolver import resolve_repo_path


def _load(path: str) -> dict[str, Any]:
    return json.loads(resolve_repo_path(path).read_text(encoding="utf-8"))


def _criterion(
    criteria: list[dict[str, Any]],
    name: str,
    value: Any,
    *,
    minimum: float | None = None,
    maximum: float | None = None,
    expected: Any | None = None,
) -> None:
    passed = value is not None
    if passed and isinstance(value, float):
        passed = math.isfinite(value)
    if passed and minimum is not None:
        passed = float(value) >= minimum
    if passed and maximum is not None:
        passed = float(value) <= maximum
    if passed and expected is not None:
        passed = value == expected
    criteria.append(
        {
            "name": name,
            "value": value,
            "minimum": minimum,
            "maximum": maximum,
            "expected": expected,
            "pass": bool(passed),
        }
    )


def _finite_extreme(values: Any, mode: str) -> float | None:
    if not isinstance(values, list) or not values:
        return None
    numbers = [float(value) for value in values]
    if not all(math.isfinite(value) for value in numbers):
        return None
    return max(numbers) if mode == "max" else min(numbers)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--plugin-report", required=True)
    parser.add_argument("--sensor-report", required=True)
    parser.add_argument("--forward-report", required=True)
    parser.add_argument(
        "--thresholds",
        default="validation/online_grf_acceptance_thresholds.json",
    )
    parser.add_argument(
        "--report",
        default="results/online_grf_acceptance.json",
    )
    args = parser.parse_args()

    thresholds = _load(args.thresholds)
    plugin = _load(args.plugin_report)
    sensor = _load(args.sensor_report)
    forward = _load(args.forward_report)
    active = forward.get("runs", {}).get("online", {})
    comparisons = forward.get("comparisons", {})
    active_sensor = comparisons.get("online_vs_sensor", {})
    criteria: list[dict[str, Any]] = []

    _criterion(
        criteria,
        "plugin.max_abs_error_n",
        plugin.get("max_abs_error_n"),
        maximum=thresholds["plugin_max_abs_error_n"],
    )
    for side in ("left", "right"):
        holdout = sensor.get("holdout_metrics", {}).get(side, {})
        physical = sensor.get("holdout_physical_metrics", {}).get(side, {})
        wrench = sensor.get("holdout_wrench_metrics", {}).get(side, {})
        nrmse = holdout.get("nrmse_peak", [None, None, None])
        correlation = holdout.get("correlation", [None, None, None])
        _criterion(
            criteria,
            f"sensor.{side}.vertical_nrmse",
            nrmse[1],
            maximum=thresholds["sensor_holdout_vertical_nrmse_max"],
        )
        _criterion(
            criteria,
            f"sensor.{side}.vertical_correlation",
            correlation[1],
            minimum=thresholds["sensor_holdout_vertical_correlation_min"],
        )
        _criterion(
            criteria,
            f"sensor.{side}.contact_f1",
            holdout.get("vertical_contact_f1"),
            minimum=thresholds["sensor_holdout_contact_f1_min"],
        )
        _criterion(
            criteria,
            f"sensor.{side}.penetration_max_m",
            physical.get("penetration_max_m"),
            maximum=thresholds["sensor_holdout_penetration_max_m"],
        )
        _criterion(
            criteria,
            f"sensor.{side}.force_nrmse_max",
            _finite_extreme(wrench.get("force", {}).get("nrmse_peak"), "max"),
            maximum=thresholds["sensor_holdout_force_nrmse_max"],
        )
        _criterion(
            criteria,
            f"sensor.{side}.force_correlation_min",
            _finite_extreme(wrench.get("force", {}).get("correlation"), "min"),
            minimum=thresholds["sensor_holdout_force_correlation_min"],
        )
        _criterion(
            criteria,
            f"sensor.{side}.moment_nrmse_max",
            _finite_extreme(wrench.get("moment", {}).get("nrmse_peak"), "max"),
            maximum=thresholds["sensor_holdout_moment_nrmse_max"],
        )
        _criterion(
            criteria,
            f"sensor.{side}.moment_correlation_min",
            _finite_extreme(wrench.get("moment", {}).get("correlation"), "min"),
            minimum=thresholds["sensor_holdout_moment_correlation_min"],
        )
        cop_rmse = wrench.get("cop", {}).get("rmse_m")
        _criterion(
            criteria,
            f"sensor.{side}.cop_horizontal_rmse_max_m",
            (
                max(float(cop_rmse[0]), float(cop_rmse[2]))
                if isinstance(cop_rmse, list) and len(cop_rmse) == 3
                else None
            ),
            maximum=thresholds["sensor_holdout_cop_horizontal_rmse_max_m"],
        )
        events = wrench.get("event_timing", {})
        for event in ("heel_strike", "toe_off"):
            _criterion(
                criteria,
                f"sensor.{side}.{event}_max_error_s",
                events.get(event, {}).get("nearest_event_max_s"),
                maximum=thresholds[f"sensor_holdout_{event}_max_error_s"],
            )

    _criterion(
        criteria,
        "active.status",
        active.get("status", {}).get("status"),
        expected="complete",
    )
    _criterion(
        criteria,
        "active.duration_s",
        active.get("duration_s"),
        minimum=thresholds["active_minimum_duration_s"],
    )
    pelvis = active.get("pelvis_ty", {})
    _criterion(
        criteria,
        "active.max_sink_m",
        pelvis.get("max_sink_m"),
        maximum=thresholds["active_max_sink_m"],
    )
    _criterion(
        criteria,
        "active.sink_slope_m_per_s",
        abs(float(pelvis.get("sink_slope_m_per_s", float("nan")))),
        maximum=thresholds["active_sink_slope_m_per_s_max"],
    )
    online_grf = active.get("online_grf", {})
    active_wrench = active.get("online_grf_wrench", {})
    for side in ("left", "right"):
        item = online_grf.get(side, {})
        _criterion(
            criteria,
            f"active.{side}.penetration_max_m",
            item.get("penetration_max_m"),
            maximum=thresholds["active_penetration_max_m"],
        )
        _criterion(
            criteria,
            f"active.{side}.impulse_ratio",
            item.get("prescribed_impulse_ratio"),
            minimum=thresholds["active_side_impulse_ratio_min"],
            maximum=thresholds["active_side_impulse_ratio_max"],
        )
        wrench = active_wrench.get(side, {})
        _criterion(
            criteria,
            f"active.{side}.force_nrmse_max",
            _finite_extreme(wrench.get("force", {}).get("nrmse_peak"), "max"),
            maximum=thresholds["active_force_nrmse_max"],
        )
        _criterion(
            criteria,
            f"active.{side}.force_correlation_min",
            _finite_extreme(wrench.get("force", {}).get("correlation"), "min"),
            minimum=thresholds["active_force_correlation_min"],
        )
        _criterion(
            criteria,
            f"active.{side}.moment_nrmse_max",
            _finite_extreme(wrench.get("moment", {}).get("nrmse_peak"), "max"),
            maximum=thresholds["active_moment_nrmse_max"],
        )
        cop_rmse = wrench.get("cop", {}).get("rmse_m")
        _criterion(
            criteria,
            f"active.{side}.cop_horizontal_rmse_max_m",
            (
                max(float(cop_rmse[0]), float(cop_rmse[2]))
                if isinstance(cop_rmse, list) and len(cop_rmse) == 3
                else None
            ),
            maximum=thresholds["active_cop_horizontal_rmse_max_m"],
        )
    _criterion(
        criteria,
        "active.total_impulse_ratio",
        online_grf.get("total", {}).get("prescribed_impulse_ratio"),
        minimum=thresholds["active_total_impulse_ratio_min"],
        maximum=thresholds["active_total_impulse_ratio_max"],
    )
    _criterion(
        criteria,
        "active_vs_sensor.same_time_window",
        active_sensor.get("same_time_window"),
        expected=True,
    )
    _criterion(
        criteria,
        "active_vs_sensor.pelvis_ty_reserve_p95_ratio",
        active_sensor.get("pelvis_ty_reserve_p95_ratio"),
        maximum=thresholds["active_reserve_p95_ratio_vs_sensor_max"],
    )
    _criterion(
        criteria,
        "active.pelvis_ty_reserve_abs_growth_slope",
        active.get("pelvis_ty_reserve", {}).get("slope_abs_per_s"),
        maximum=thresholds["active_reserve_abs_growth_slope_max"],
    )

    failed = [item for item in criteria if not item["pass"]]
    report = {
        "verdict": "PASS" if not failed else "FAIL",
        "passed": len(criteria) - len(failed),
        "failed": len(failed),
        "sources": {
            "plugin_report": str(resolve_repo_path(args.plugin_report)),
            "sensor_report": str(resolve_repo_path(args.sensor_report)),
            "forward_report": str(resolve_repo_path(args.forward_report)),
            "sensor_input_profile": sensor.get("input_profile"),
            "forward_profile": forward.get("profile"),
        },
        "thresholds": thresholds,
        "criteria": criteria,
    }
    report_path = resolve_repo_path(args.report)
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0 if not failed else 1


if __name__ == "__main__":
    raise SystemExit(main())
