"""
validate_sim_results.py
=======================
Numerical checks for detecting tautological SEA simulation results.

The script reads simulator outputs from results/ and reports whether the SEA
states look plugin-driven or algebraically forced to match the torque command.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple
from xml.etree import ElementTree as ET

import numpy as np

import sys

REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from config import SimulatorConfig  # noqa: E402
from output import read_sto  # noqa: E402


ROTATION_HINTS = (
    "angle",
    "tilt",
    "list",
    "rotation",
    "extension",
    "bending",
    "flexion",
    "adduction",
)


@dataclass
class Table:
    path: Path
    time: np.ndarray
    columns: List[str]
    data: np.ndarray
    in_degrees: bool

    def __post_init__(self) -> None:
        self.index = {name: i for i, name in enumerate(self.columns)}

    def series(self, candidates: Sequence[str]) -> Optional[Tuple[np.ndarray, np.ndarray, str]]:
        for name in candidates:
            idx = self.index.get(name)
            if idx is not None:
                return self.time, self.data[:, idx], name
        return None


@dataclass
class Check:
    status: str
    name: str
    detail: str


def resolve_path(raw: str | Path) -> Path:
    path = Path(raw)
    return path if path.is_absolute() else REPO_ROOT / path


def load_table(path: Path) -> Optional[Table]:
    if not path.is_file():
        return None
    time, cols, data, in_degrees = read_sto(str(path))
    return Table(path, time, cols, data, in_degrees)


def load_required_tables(results_dir: Path, prefix: str) -> Dict[str, Optional[Table]]:
    suffixes = {
        "sea_torques": "sea_torques",
        "sea_controls": "sea_controls",
        "sea_states": "sea_states",
        "sea_derivatives": "sea_derivatives",
        "sea_diagnostics": "sea_diagnostics",
        "states": "states",
        "kinematics": "kinematics",
        "reserve_torques": "reserve_torques",
        "power": "power",
    }
    return {
        key: load_table(results_dir / f"{prefix}_{suffix}.sto")
        for key, suffix in suffixes.items()
    }


def load_run_status(results_dir: Path, prefix: str) -> Dict[str, str]:
    path = results_dir / f"{prefix}_run_status.txt"
    if not path.is_file():
        return {}
    values: Dict[str, str] = {}
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        if "=" not in raw_line:
            continue
        key, value = raw_line.split("=", 1)
        values[key.strip()] = value.strip()
    return values


def validate_run_status(
    results_dir: Path,
    prefix: str,
    checks: List[Check],
) -> None:
    status = load_run_status(results_dir, prefix)
    if not status:
        add_check(
            checks,
            "WARN",
            "run status",
            "missing run status file; cannot tell whether outputs are complete",
        )
        return
    if status.get("status") != "complete":
        detail = (
            f"simulation status={status.get('status', 'unknown')}, "
            f"t={status.get('t', '?')} of {status.get('t_end', '?')}; "
            f"{status.get('error', 'no error detail')}"
        )
        add_check(checks, "FAIL", "run status", detail)
        return
    add_check(
        checks,
        "PASS",
        "run status",
        f"simulation complete at t={status.get('t', '?')}",
    )


def xml_local_name(tag: str) -> str:
    return tag.rsplit("}", 1)[-1]


def parse_sea_props(model_path: Path, sea_names: Iterable[str]) -> Dict[str, Dict[str, float]]:
    tree = ET.parse(model_path)
    wanted = set(sea_names)
    props: Dict[str, Dict[str, float]] = {}
    for element in tree.getroot().iter():
        if xml_local_name(element.tag) != "SeriesElasticActuator":
            continue
        sea_name = element.attrib.get("name", "")
        if sea_name not in wanted:
            continue
        values = {
            xml_local_name(child.tag): (child.text or "").strip()
            for child in element
        }

        def as_float(key: str) -> float:
            return float(values[key])

        props[sea_name] = {
            "K": as_float("stiffness"),
            "F_opt": as_float("optimal_force"),
            "Kp": as_float("Kp"),
            "Kd": as_float("Kd"),
            "Bm": as_float("motor_damping"),
            "Jm": as_float("motor_inertia"),
        }
    missing = wanted - set(props)
    if missing:
        raise ValueError(f"Missing SeriesElasticActuator(s) in model XML: {sorted(missing)}")
    return props


def is_rotation_coord(coord: str, cfg: SimulatorConfig) -> bool:
    if coord in cfg.translation_coords:
        return False
    return any(hint in coord for hint in ROTATION_HINTS)


def interp(source_time: np.ndarray, source_values: np.ndarray, target_time: np.ndarray) -> np.ndarray:
    return np.interp(target_time, source_time, source_values)


def stats(values: np.ndarray) -> Dict[str, float]:
    arr = np.asarray(values, dtype=float)
    return {
        "max_abs": float(np.nanmax(np.abs(arr))),
        "rms": float(np.sqrt(np.nanmean(arr * arr))),
        "mean_abs": float(np.nanmean(np.abs(arr))),
    }


def format_stats(values: np.ndarray, unit: str = "") -> str:
    s = stats(values)
    suffix = f" {unit}" if unit else ""
    return (
        f"max_abs={s['max_abs']:.6e}{suffix}, "
        f"rms={s['rms']:.6e}{suffix}, "
        f"mean_abs={s['mean_abs']:.6e}{suffix}"
    )


def add_check(checks: List[Check], status: str, name: str, detail: str) -> None:
    checks.append(Check(status, name, detail))


def get_series(table: Optional[Table], candidates: Sequence[str]) -> Optional[Tuple[np.ndarray, np.ndarray, str]]:
    if table is None:
        return None
    return table.series(candidates)


def validate_sea_tautology(
    cfg: SimulatorConfig,
    tables: Dict[str, Optional[Table]],
    sea_props: Dict[str, Dict[str, float]],
    checks: List[Check],
) -> None:
    sides = [
        (cfg.sea_knee_name, cfg.pros_coords[0]),
        (cfg.sea_ankle_name, cfg.pros_coords[1]),
    ]
    for sea_name, coord_name in sides:
        torques = get_series(tables["sea_torques"], [f"{sea_name}_tau_spring"])
        controls = get_series(tables["sea_controls"], [coord_name])
        states_angle = get_series(tables["sea_states"], [f"{sea_name}_motor_angle"])
        states_speed = get_series(tables["sea_states"], [f"{sea_name}_motor_speed"])
        joint_angle = get_series(tables["states"], [f"{coord_name}_q"])
        joint_speed = get_series(tables["states"], [f"{coord_name}_qdot"])
        if None in (torques, controls, states_angle, states_speed, joint_angle, joint_speed):
            add_check(checks, "FAIL", f"{sea_name} required channels", "missing SEA validation channel(s)")
            continue

        torque_time, tau_spring, _ = torques
        control_time, control, _ = controls
        ma_time, theta_m, _ = states_angle
        ms_time, omega_m, _ = states_speed
        q_time, q, _ = joint_angle
        qdot_time, qdot, _ = joint_speed

        props = sea_props[sea_name]
        tau_ref = interp(control_time, control, torque_time) * props["F_opt"]
        theta_m_i = interp(ma_time, theta_m, torque_time)
        omega_m_i = interp(ms_time, omega_m, torque_time)
        q_i = interp(q_time, q, torque_time)
        qdot_i = interp(qdot_time, qdot, torque_time)

        tau_error = tau_ref - tau_spring
        algebraic_state_error = theta_m_i - (q_i + tau_ref / props["K"])
        speed_error = omega_m_i - qdot_i

        tau_s = stats(tau_error)
        alg_s = stats(algebraic_state_error)
        speed_s = stats(speed_error)

        if tau_s["max_abs"] < 1e-4:
            status = "FAIL"
            reason = "tau_ref and tau_spring match at numerical precision"
        elif tau_s["rms"] < 1e-2:
            status = "WARN"
            reason = "very small torque tracking error"
        else:
            status = "PASS"
            reason = "non-trivial torque tracking error"
        add_check(checks, status, f"{sea_name} tau_ref - tau_spring", f"{reason}; {format_stats(tau_error, 'Nm')}")

        if alg_s["max_abs"] < 1e-6:
            status = "FAIL"
            reason = "motor_angle equals q + tau_ref/K"
        elif alg_s["rms"] < 1e-4:
            status = "WARN"
            reason = "motor_angle is very close to algebraic equilibrium"
        else:
            status = "PASS"
            reason = "motor_angle is not algebraically constrained"
        add_check(checks, status, f"{sea_name} algebraic motor state", f"{reason}; {format_stats(algebraic_state_error, 'rad')}")

        if speed_s["max_abs"] < 1e-10:
            status = "FAIL"
            reason = "motor_speed equals joint speed exactly"
        elif speed_s["rms"] < 1e-3:
            status = "WARN"
            reason = "motor_speed nearly equals joint speed"
        else:
            status = "PASS"
            reason = "motor_speed differs from joint speed"
        add_check(checks, status, f"{sea_name} motor_speed - qdot", f"{reason}; {format_stats(speed_error, 'rad/s')}")


def validate_reserves(
    cfg: SimulatorConfig,
    tables: Dict[str, Optional[Table]],
    checks: List[Check],
) -> None:
    for coord_name in cfg.pros_coords:
        series = get_series(tables["reserve_torques"], [f"{coord_name}_reserve_torque"])
        if series is None:
            add_check(checks, "WARN", f"{coord_name} reserve torque", "missing reserve torque channel")
            continue
        _time, values, _name = series
        s = stats(values)
        if s["max_abs"] == 0.0:
            status = "WARN"
            reason = "prosthetic reserve is exactly zero; OK only if SEA path is validated"
        else:
            status = "PASS"
            reason = "prosthetic reserve has non-zero diagnostic values"
        add_check(checks, status, f"{coord_name} reserve torque", f"{reason}; {format_stats(values, 'Nm')}")


def ref_series(reference: Table, coord_name: str, cfg: SimulatorConfig) -> Optional[Tuple[np.ndarray, np.ndarray, str]]:
    candidates = [coord_name, f"{coord_name}/value", f"/jointset/{coord_name}/{coord_name}/value"]
    result = reference.series(candidates)
    if result is None:
        return None
    time, values, name = result
    if reference.in_degrees and is_rotation_coord(coord_name, cfg):
        values = np.deg2rad(values)
    return time, values, name


def validate_tracking(
    cfg: SimulatorConfig,
    tables: Dict[str, Optional[Table]],
    reference: Optional[Table],
    checks: List[Check],
) -> None:
    if reference is None:
        add_check(checks, "WARN", "kinematic tracking", "reference kinematics not available")
        return
    source = tables["kinematics"] or tables["states"]
    if source is None:
        add_check(checks, "WARN", "kinematic tracking", "simulated kinematics not available")
        return

    if source is tables["kinematics"]:
        coord_names = [
            name for name in source.columns
            if ref_series(reference, name, cfg) is not None
        ]
    else:
        coord_names = [
            name[:-2] for name in source.columns
            if name.endswith("_q")
        ]

    preferred = list(cfg.pros_coords) + list(cfg.translation_coords)
    ordered = preferred + [
        name for name in coord_names
        if name not in preferred
    ]

    for coord_name in ordered:
        if source is tables["kinematics"]:
            sim = source.series([coord_name])
        else:
            sim = source.series([f"{coord_name}_q"])
        ref = ref_series(reference, coord_name, cfg)
        if sim is None or ref is None:
            add_check(checks, "WARN", f"{coord_name} tracking", "missing simulated or reference channel")
            continue
        sim_t, sim_y, _sim_name = sim
        ref_t, ref_y, _ref_name = ref
        err = sim_y - interp(ref_t, ref_y, sim_t)
        if is_rotation_coord(coord_name, cfg):
            err_out = np.rad2deg(err)
            unit = "deg"
            rms_warn = 0.1
            rms_fail = 10.0
            max_fail = 30.0
        else:
            err_out = err
            unit = "m"
            rms_warn = 1e-3
            rms_fail = 0.02
            max_fail = 0.05
        s = stats(err_out)
        if not np.all(np.isfinite(err_out)):
            status = "FAIL"
            reason = "non-finite tracking error"
        elif s["rms"] >= rms_fail or s["max_abs"] >= max_fail:
            status = "FAIL"
            reason = "tracking error exceeds stability threshold"
        elif s["rms"] < rms_warn:
            status = "WARN"
            reason = "very tight tracking; result is controller-following, not predictive"
        else:
            status = "PASS"
            reason = "non-zero tracking error"
        add_check(checks, status, f"{coord_name} output vs IK", f"{reason}; {format_stats(err_out, unit)}")


def validate_power_and_controls(
    cfg: SimulatorConfig,
    tables: Dict[str, Optional[Table]],
    checks: List[Check],
) -> None:
    for sea_name, coord_name in (
        (cfg.sea_knee_name, cfg.pros_coords[0]),
        (cfg.sea_ankle_name, cfg.pros_coords[1]),
    ):
        control = get_series(tables["sea_controls"], [coord_name])
        if control is not None:
            _time, values, _name = control
            max_u = float(np.nanmax(np.abs(values)))
            if max_u >= 0.999:
                status = "WARN"
                reason = "SEA control reaches saturation"
            else:
                status = "PASS"
                reason = "SEA control below saturation"
            add_check(checks, status, f"{sea_name} control saturation", f"{reason}; max |u|={max_u:.6f}")

        joint_power = get_series(tables["power"], [f"{sea_name}_joint_power"])
        motor_power = get_series(tables["power"], [f"{sea_name}_motor_power"])
        if joint_power is None or motor_power is None:
            add_check(checks, "WARN", f"{sea_name} power", "missing joint or motor power channel")
            continue
        power_t, jp, _ = joint_power
        mp_t, mp, _ = motor_power
        mp_i = interp(mp_t, mp, power_t)
        diff = mp_i - jp
        if np.nanstd(jp) > 0 and np.nanstd(mp_i) > 0:
            corr = float(np.corrcoef(jp, mp_i)[0, 1])
        else:
            corr = float("nan")
        add_check(
            checks,
            "PASS",
            f"{sea_name} motor vs joint power",
            f"motor-joint diff {format_stats(diff, 'W')}, corr={corr:.6f}",
        )


def validate_sea_derivatives(
    cfg: SimulatorConfig,
    tables: Dict[str, Optional[Table]],
    checks: List[Check],
) -> None:
    table = tables["sea_derivatives"]
    if table is None:
        add_check(checks, "WARN", "SEA plugin derivatives", "missing sim_output_sea_derivatives.sto")
        return
    for sea_name in (cfg.sea_knee_name, cfg.sea_ankle_name):
        angle_dot = table.series([f"{sea_name}_motor_angle_dot"])
        speed_dot = table.series([f"{sea_name}_motor_speed_dot"])
        if angle_dot is None or speed_dot is None:
            add_check(checks, "WARN", f"{sea_name} derivatives", "missing derivative channel")
            continue
        _t, angle_values, _ = angle_dot
        _t, speed_values, _ = speed_dot
        if not np.all(np.isfinite(angle_values)) or not np.all(np.isfinite(speed_values)):
            add_check(checks, "FAIL", f"{sea_name} derivatives", "non-finite plugin derivatives")
        elif stats(speed_values)["max_abs"] == 0.0:
            add_check(checks, "WARN", f"{sea_name} derivatives", "motor_speed_dot is exactly zero")
        else:
            add_check(checks, "PASS", f"{sea_name} derivatives", f"finite plugin derivatives; speed_dot {format_stats(speed_values, 'rad/s^2')}")


def validate_sea_interface_diagnostics(
    cfg: SimulatorConfig,
    tables: Dict[str, Optional[Table]],
    checks: List[Check],
) -> None:
    table = tables.get("sea_diagnostics")
    if table is None:
        add_check(
            checks,
            "WARN",
            "SEA diagnostics",
            "missing sim_output_sea_diagnostics.sto; plugin/Python tau_input interface not certified",
        )
        return

    for sea_name in (cfg.sea_knee_name, cfg.sea_ankle_name):
        python_tau = table.series([f"{sea_name}_tau_input_python"])
        plugin_tau = table.series([f"{sea_name}_tau_input_plugin"])
        tau_diff = table.series([f"{sea_name}_tau_input_plugin_minus_python"])
        tau_error = table.series([f"{sea_name}_tau_error"])
        tau_ff = table.series([f"{sea_name}_tau_ff_cmd"])
        outer_pd = table.series([f"{sea_name}_outer_pd_cmd"])
        tau_input_raw = table.series([f"{sea_name}_tau_input_raw"])
        inner_prop = table.series([f"{sea_name}_inner_prop_term"])
        inner_damp = table.series([f"{sea_name}_inner_damp_term"])
        accel_numerator = table.series([f"{sea_name}_motor_accel_numerator"])
        motor_speed = table.series([f"{sea_name}_motor_speed"])
        speed_dot = table.series([f"{sea_name}_motor_speed_dot_plugin"])
        saturated = table.series([f"{sea_name}_tau_input_saturated"])

        required = (python_tau, plugin_tau, tau_diff, tau_error, motor_speed, speed_dot)
        if any(item is None for item in required):
            add_check(
                checks,
                "FAIL",
                f"{sea_name} SEA diagnostics",
                "missing required diagnostic channel(s)",
            )
            continue

        _time, python_values, _ = python_tau
        _time, plugin_values, _ = plugin_tau
        _time, diff_values, _ = tau_diff
        _time, tau_error_values, _ = tau_error
        _time, speed_values, _ = motor_speed
        _time, speed_dot_values, _ = speed_dot

        if not np.all(np.isfinite(plugin_values)):
            add_check(
                checks,
                "FAIL",
                f"{sea_name} plugin tau_input output",
                "non-finite or missing plugin tau_input output",
            )
        elif not np.all(np.isfinite(python_values)):
            add_check(
                checks,
                "FAIL",
                f"{sea_name} Python tau_input recompute",
                "non-finite Python tau_input recompute",
            )
        elif stats(diff_values)["max_abs"] > 1e-4:
            add_check(
                checks,
                "FAIL",
                f"{sea_name} plugin/Python tau_input agreement",
                "plugin output does not match independently recomputed law; "
                f"{format_stats(diff_values, 'Nm')}",
            )
        else:
            add_check(
                checks,
                "PASS",
                f"{sea_name} plugin/Python tau_input agreement",
                "plugin output matches independently recomputed SEA law; "
                f"{format_stats(diff_values, 'Nm')}",
            )

        if not np.all(np.isfinite(tau_error_values)):
            add_check(checks, "FAIL", f"{sea_name} tau_error diagnostic", "non-finite tau_ref - tau_spring")
        else:
            add_check(
                checks,
                "PASS",
                f"{sea_name} tau_error diagnostic",
                f"finite tau_ref - tau_spring; {format_stats(tau_error_values, 'Nm')}",
            )

        if not np.all(np.isfinite(speed_values)) or not np.all(np.isfinite(speed_dot_values)):
            add_check(checks, "FAIL", f"{sea_name} motor speed diagnostics", "non-finite motor speed or speed_dot")
        else:
            add_check(
                checks,
                "PASS",
                f"{sea_name} motor speed diagnostics",
                f"motor_speed {format_stats(speed_values, 'rad/s')}; "
                f"speed_dot {format_stats(speed_dot_values, 'rad/s^2')}",
            )

        if saturated is None:
            add_check(checks, "WARN", f"{sea_name} tau_input saturation", "missing saturation flag")
            continue
        _time, sat_values, _ = saturated
        sat_count = int(np.nansum(np.asarray(sat_values) > 0.5))
        sat_pct = sat_count / max(1, len(sat_values)) * 100.0
        sat_mask = np.asarray(sat_values) > 0.5
        if sat_count:
            add_check(
                checks,
                "WARN",
                f"{sea_name} tau_input saturation",
                f"tau_input reaches +/-500 Nm in {sat_count} samples ({sat_pct:.3f}%); tuning issue, not an interface failure",
            )
        else:
            add_check(
                checks,
                "PASS",
                f"{sea_name} tau_input saturation",
                "tau_input never reaches the +/-500 Nm clamp",
            )

        source_channels = (tau_ff, outer_pd, tau_input_raw, inner_prop, inner_damp, accel_numerator)
        if any(item is None for item in source_channels):
            add_check(
                checks,
                "WARN",
                f"{sea_name} saturation source terms",
                "missing extended SEA diagnostic source channels",
            )
            continue

        _time, tau_ff_values, _ = tau_ff
        _time, outer_pd_values, _ = outer_pd
        _time, raw_values, _ = tau_input_raw
        _time, prop_values, _ = inner_prop
        _time, damp_values, _ = inner_damp
        _time, numerator_values, _ = accel_numerator

        if not all(np.all(np.isfinite(values)) for values in (
            tau_ff_values,
            outer_pd_values,
            raw_values,
            prop_values,
            damp_values,
            numerator_values,
        )):
            add_check(
                checks,
                "FAIL",
                f"{sea_name} saturation source terms",
                "non-finite extended SEA diagnostic values",
            )
            continue

        raw_excess = np.maximum(np.abs(raw_values) - 500.0, 0.0)
        if sat_count:
            detail = (
                f"sat median |tau_ff|={np.median(np.abs(tau_ff_values[sat_mask])):.3f} Nm, "
                f"|outer_PD|={np.median(np.abs(outer_pd_values[sat_mask])):.3f} Nm, "
                f"|inner_prop|={np.median(np.abs(prop_values[sat_mask])):.3f} Nm, "
                f"|inner_damp|={np.median(np.abs(damp_values[sat_mask])):.3f} Nm; "
                f"raw excess {format_stats(raw_excess, 'Nm')}; "
                f"motor numerator {format_stats(numerator_values, 'Nm')}"
            )
            add_check(checks, "PASS", f"{sea_name} saturation source terms", detail)
        else:
            detail = (
                f"raw command below clamp; raw {format_stats(raw_values, 'Nm')}; "
                f"motor numerator {format_stats(numerator_values, 'Nm')}"
            )
            add_check(checks, "PASS", f"{sea_name} saturation source terms", detail)


def status_rank(status: str) -> int:
    return {"FAIL": 2, "WARN": 1, "PASS": 0}.get(status, 0)


def render_report(checks: List[Check], results_dir: Path, reference: Optional[Path]) -> str:
    worst = max(checks, key=lambda item: status_rank(item.status)).status if checks else "PASS"
    lines = [
        f"# Validazione simulatore - {datetime.now().strftime('%Y-%m-%d %H:%M')}",
        "",
        f"Risultato complessivo: **{worst}**",
        "",
        f"- Results dir: `{results_dir}`",
        f"- Reference: `{reference}`" if reference else "- Reference: not provided",
        "",
        "## Checks",
        "",
        "| Status | Check | Detail |",
        "|---|---|---|",
    ]
    for check in checks:
        detail = check.detail.replace("|", "\\|")
        lines.append(f"| {check.status} | {check.name} | {detail} |")
    lines.append("")
    if worst == "FAIL":
        lines.extend([
            "## Interpretazione",
            "",
            "Almeno un controllo critico indica che il risultato non e validabile.",
            "Un FAIL su `run status` indica output parziali; un FAIL su tracking",
            "indica divergenza dinamica; un FAIL sulle metriche SEA indica possibile",
            "tautologia o derivate plugin non finite.",
            "",
        ])
    return "\n".join(lines)


def print_summary(checks: List[Check]) -> None:
    for check in checks:
        print(f"[{check.status}] {check.name}: {check.detail}")
    counts = {status: sum(1 for check in checks if check.status == status) for status in ("PASS", "WARN", "FAIL")}
    print(
        f"Summary: PASS={counts['PASS']}, WARN={counts['WARN']}, "
        f"FAIL={counts['FAIL']}"
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Validate simulator outputs for SEA tautology and tracking diagnostics."
    )
    parser.add_argument("--results-dir", default="results")
    parser.add_argument("--prefix", default="sim_output")
    parser.add_argument("--model", default=SimulatorConfig.model_file)
    parser.add_argument("--reference", default=SimulatorConfig.kinematics_file)
    parser.add_argument("--out", default=None)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    cfg = SimulatorConfig()
    results_dir = resolve_path(args.results_dir)
    model_path = resolve_path(args.model)
    reference_path = resolve_path(args.reference) if args.reference else None

    if not results_dir.is_dir():
        raise FileNotFoundError(f"Results directory not found: {results_dir}")

    tables = load_required_tables(results_dir, args.prefix)
    reference = load_table(reference_path) if reference_path and reference_path.is_file() else None
    sea_props = parse_sea_props(
        model_path,
        [cfg.sea_knee_name, cfg.sea_ankle_name],
    )

    checks: List[Check] = []
    validate_run_status(results_dir, args.prefix, checks)
    validate_sea_tautology(cfg, tables, sea_props, checks)
    validate_sea_derivatives(cfg, tables, checks)
    validate_sea_interface_diagnostics(cfg, tables, checks)
    validate_reserves(cfg, tables, checks)
    validate_tracking(cfg, tables, reference, checks)
    validate_power_and_controls(cfg, tables, checks)

    print_summary(checks)

    report = render_report(checks, results_dir, reference_path)
    if args.out:
        out_path = resolve_path(args.out)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        out_path.write_text(report, encoding="utf-8")
        print(f"Report written to: {out_path}")

    return 1 if any(check.status == "FAIL" for check in checks) else 0


if __name__ == "__main__":
    raise SystemExit(main())
