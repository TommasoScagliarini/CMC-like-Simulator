"""
plotter.py
==========
Read simulator .sto outputs and save ankle/knee diagnostic PNG plots.

This script is intentionally a consumer of existing result channels. It does
not derive missing simulator channels such as SEA motor states, reserve
actuator torque, power, or gait-cycle events. Healthy reference files are
loaded from data/health or data/healthy when available; reference velocity and
power are derived only for plotting. Missing channels are reported and
annotated in the generated figures.
"""

from __future__ import annotations

import argparse
import csv
import re
import sys
from xml.etree import ElementTree as ET
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from output import read_sto  # noqa: E402
from config import SimulatorConfig  # noqa: E402


GAIT_GRID = np.linspace(0.0, 100.0, 101)
HEALTHY_COLOR = "#ffa500"
HEALTHY_LINESTYLE = "--"

SIDES = [
    {
        "key": "ankle",
        "title": "Ankle",
        "coord": "pros_ankle_angle",
        "sea": "SEA_Ankle",
    },
    {
        "key": "knee",
        "title": "Knee",
        "coord": "pros_knee_angle",
        "sea": "SEA_Knee",
    },
]


@dataclass
class StoTable:
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
class HealthyData:
    directory: Path
    kinematics: StoTable
    actuator_forces: StoTable
    notes: List[str]


class MissingReport:
    def __init__(self) -> None:
        self._items: List[str] = []
        self._seen: set[str] = set()

    def add(self, item: str) -> None:
        if item not in self._seen:
            self._seen.add(item)
            self._items.append(item)

    def extend(self, items: Iterable[str]) -> None:
        for item in items:
            self.add(item)

    @property
    def items(self) -> List[str]:
        return list(self._items)

    def write(self, path: Path) -> None:
        if self._items:
            text = "\n".join(f"- {item}" for item in self._items) + "\n"
        else:
            text = "No missing channels.\n"
        path.write_text(text, encoding="utf-8")


def resolve_project_path(raw: str | Path) -> Path:
    path = Path(raw)
    if path.is_absolute():
        return path
    return REPO_ROOT / path


def next_output_dir(out_root: Path) -> Path:
    out_root.mkdir(parents=True, exist_ok=True)
    date_str = datetime.now().strftime("%d_%m_%Y")
    pattern = re.compile(rf"^{re.escape(date_str)} - (\d+)$")
    existing = []
    for child in out_root.iterdir():
        if not child.is_dir():
            continue
        match = pattern.match(child.name)
        if match:
            existing.append(int(match.group(1)))
    idx = max(existing, default=0) + 1
    out_dir = out_root / f"{date_str} - {idx}"
    out_dir.mkdir(parents=False, exist_ok=False)
    return out_dir


def load_table(path: Path) -> Optional[StoTable]:
    if not path.is_file():
        return None
    time, columns, data, in_degrees = read_sto(str(path))
    return StoTable(path, time, columns, data, in_degrees)


def default_healthy_dir() -> Optional[Path]:
    for rel_path in ("data/health", "data/healthy"):
        path = REPO_ROOT / rel_path
        if path.is_dir():
            return path
    return None


def load_healthy_data(
    healthy_dir: Optional[Path],
    missing: MissingReport,
) -> Optional[HealthyData]:
    if healthy_dir is None:
        missing.add("healthy data: directory not found; expected data/health or data/healthy")
        return None
    if not healthy_dir.is_dir():
        missing.add(f"healthy data: directory not found: {healthy_dir}")
        return None

    kinematics_files = sorted(healthy_dir.glob("*Kinematics_q.sto"))
    force_files = sorted(healthy_dir.glob("*Actuation_force.sto"))
    if not kinematics_files:
        missing.add(f"healthy data: missing *Kinematics_q.sto in {healthy_dir}")
        return None
    if not force_files:
        missing.add(f"healthy data: missing *Actuation_force.sto in {healthy_dir}")
        return None

    kin = load_table(kinematics_files[0])
    forces = load_table(force_files[0])
    if kin is None or forces is None:
        missing.add(f"healthy data: could not load tables from {healthy_dir}")
        return None

    notes = [
        "healthy velocity derived from Kinematics_q finite differences",
        "healthy power derived as healthy torque * derived healthy velocity",
    ]
    return HealthyData(healthy_dir, kin, forces, notes)


def load_tables(results_dir: Path, prefix: str) -> Dict[str, Optional[StoTable]]:
    suffixes = {
        "sea_torques": "sea_torques",
        "sea_controls": "sea_controls",
        "states": "states",
        "reserve_torques": "reserve_torques",
        "sea_states": "sea_states",
        "power": "power",
    }
    return {
        key: load_table(results_dir / f"{prefix}_{suffix}.sto")
        for key, suffix in suffixes.items()
    }


def annotate_missing(ax: plt.Axes, message: str) -> None:
    ax.text(
        0.5,
        0.5,
        f"missing:\n{message}",
        ha="center",
        va="center",
        transform=ax.transAxes,
        fontsize=10,
        color="crimson",
    )
    ax.grid(True, alpha=0.25)


def note_missing(
    missing: MissingReport,
    figure: str,
    side: str,
    channel: str,
    detail: str,
) -> None:
    missing.add(f"{figure} / {side} / {channel}: {detail}")


def find_series(
    table: Optional[StoTable],
    candidates: Sequence[str],
    missing: MissingReport,
    figure: str,
    side: str,
    channel: str,
    file_label: str,
) -> Optional[Tuple[np.ndarray, np.ndarray, str]]:
    if table is None:
        note_missing(missing, figure, side, channel, f"file not found: {file_label}")
        return None
    found = table.series(candidates)
    if found is None:
        note_missing(
            missing,
            figure,
            side,
            channel,
            "columns not found: " + ", ".join(candidates),
        )
    return found


def joint_sign(side_key: str) -> float:
    """Plot knee joint angle/velocity with the requested sign convention."""
    return -1.0 if side_key == "knee" else 1.0


def apply_joint_sign(
    series: Optional[Tuple[np.ndarray, np.ndarray, str]],
    side_key: str,
) -> Optional[Tuple[np.ndarray, np.ndarray, str]]:
    if series is None:
        return None
    time, values, column = series
    return time, values * joint_sign(side_key), column


def convert_angle_values(values: np.ndarray, in_degrees: bool) -> np.ndarray:
    return np.deg2rad(values) if in_degrees else values


def unique_time_series(time: np.ndarray, values: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    unique_time, unique_idx = np.unique(time, return_index=True)
    return unique_time, values[unique_idx]


def healthy_angle_series(
    healthy: Optional[HealthyData],
    coord: str,
    side_key: str,
    missing: MissingReport,
    figure: str,
) -> Optional[Tuple[np.ndarray, np.ndarray, str]]:
    if healthy is None:
        return None
    found = healthy.kinematics.series([coord])
    if found is None:
        note_missing(missing, figure, side_key, "healthy angle", f"column not found: {coord}")
        return None
    time, values, column = found
    values = convert_angle_values(values, healthy.kinematics.in_degrees)
    time, values = unique_time_series(time, values)
    return apply_joint_sign((time, values, column), side_key)


def healthy_velocity_series(
    healthy: Optional[HealthyData],
    coord: str,
    side_key: str,
    missing: MissingReport,
    figure: str,
) -> Optional[Tuple[np.ndarray, np.ndarray, str]]:
    angle = healthy_angle_series(healthy, coord, side_key, missing, figure)
    if angle is None:
        return None
    time, values, column = angle
    if len(time) < 2:
        note_missing(missing, figure, side_key, "healthy velocity", "not enough kinematic samples")
        return None
    velocity = np.gradient(values, time)
    return time, velocity, f"{column}_derived_qdot"


def healthy_torque_series(
    healthy: Optional[HealthyData],
    coord: str,
    side_key: str,
    missing: MissingReport,
    figure: str,
) -> Optional[Tuple[np.ndarray, np.ndarray, str]]:
    if healthy is None:
        return None
    candidates = [
        f"reserve_{coord}",
        coord,
        f"{coord}_torque",
        f"{side_key}_torque",
    ]
    found = healthy.actuator_forces.series(candidates)
    if found is None:
        note_missing(
            missing,
            figure,
            side_key,
            "healthy torque",
            "columns not found: " + ", ".join(candidates),
        )
    if found is None:
        return None
    time, values, column = found
    time, values = unique_time_series(time, values)
    return apply_joint_sign((time, values, column), side_key)


def healthy_power_series(
    healthy: Optional[HealthyData],
    coord: str,
    side_key: str,
    missing: MissingReport,
    figure: str,
) -> Optional[Tuple[np.ndarray, np.ndarray, str]]:
    torque = healthy_torque_series(healthy, coord, side_key, missing, figure)
    velocity = healthy_velocity_series(healthy, coord, side_key, missing, figure)
    if torque is None or velocity is None:
        return None
    target_velocity = np.interp(torque[0], velocity[0], velocity[1])
    return torque[0], torque[1] * target_velocity, "healthy_joint_power_derived"


def torque_candidates(coord: str, sea: str) -> List[str]:
    return [
        f"{sea}_tau_spring",
        f"{coord}_tau_spring",
        f"{sea}_sea_torque",
        f"{coord}_sea_torque",
        f"{sea}_torque",
        f"{coord}_torque",
        f"{sea}_tau",
        f"{coord}_tau",
    ]


def control_candidates(coord: str) -> List[str]:
    return [coord, f"{coord}_u", f"{coord}_control", f"{coord}_control_input"]


def reserve_candidates(coord: str) -> List[str]:
    return [
        f"{coord}_reserve",
        f"{coord}_reserve_torque",
        f"{coord}_tau_reserve",
        f"{coord}_reserve_actuator",
    ]


def q_candidates(coord: str) -> List[str]:
    return [f"{coord}_q"]


def qdot_candidates(coord: str) -> List[str]:
    return [f"{coord}_qdot"]


def motor_angle_candidates(coord: str, sea: str) -> List[str]:
    return [
        f"{sea}_motor_angle",
        f"{coord}_motor_angle",
        f"{sea}/motor_angle",
        f"{coord}/motor_angle",
    ]


def motor_speed_candidates(coord: str, sea: str) -> List[str]:
    return [
        f"{sea}_motor_speed",
        f"{coord}_motor_speed",
        f"{sea}/motor_speed",
        f"{coord}/motor_speed",
    ]


def joint_power_candidates(coord: str, sea: str, side_key: str) -> List[str]:
    return [
        f"{sea}_joint_power",
        f"{coord}_joint_power",
        f"{coord}_power",
        f"{side_key}_power",
        f"{side_key}_joint_power",
    ]


def motor_power_candidates(coord: str, sea: str, side_key: str) -> List[str]:
    return [
        f"{sea}_motor_power",
        f"{coord}_motor_power",
        f"{side_key}_motor_power",
    ]


def motor_torque_candidates(coord: str, sea: str) -> List[str]:
    return [
        f"{sea}_tau_motor",
        f"{coord}_tau_motor",
        f"{sea}_tau_input",
        f"{coord}_tau_input",
    ]


def xml_local_name(tag: str) -> str:
    return tag.rsplit("}", 1)[-1]


def child_text(element: ET.Element, child_name: str) -> Optional[str]:
    for child in list(element):
        if xml_local_name(child.tag) == child_name:
            return child.text.strip() if child.text is not None else ""
    return None


def load_sea_f_opt(cfg: SimulatorConfig, missing: MissingReport) -> Dict[str, float]:
    model_path = resolve_project_path(cfg.model_file)
    sea_names = [cfg.sea_knee_name, cfg.sea_ankle_name]
    values: Dict[str, float] = {}
    if not model_path.is_file():
        missing.add(f"figure 5 / all / tau_ref: model file not found: {model_path}")
        return values
    try:
        root = ET.parse(model_path).getroot()
    except ET.ParseError as exc:
        missing.add(f"figure 5 / all / tau_ref: could not parse model XML: {exc}")
        return values
    for element in root.iter():
        if xml_local_name(element.tag) != "SeriesElasticActuator":
            continue
        name = element.attrib.get("name")
        if name not in sea_names:
            continue
        text = child_text(element, "optimal_force")
        if text is None or text == "":
            missing.add(f"figure 5 / {name} / tau_ref: missing <optimal_force>")
            continue
        try:
            values[name] = float(text)
        except ValueError:
            missing.add(f"figure 5 / {name} / tau_ref: invalid <optimal_force> value '{text}'")
    for name in sea_names:
        if name not in values:
            missing.add(f"figure 5 / {name} / tau_ref: SEA not found in model XML")
    return values


def plot_time_series(
    ax: plt.Axes,
    series: Optional[Tuple[np.ndarray, np.ndarray, str]],
    label: str,
    color: str,
    missing_message: str,
) -> bool:
    if series is None:
        annotate_missing(ax, missing_message)
        return False
    time, values, column = series
    ax.plot(time, values, label=label or column, color=color, linewidth=1.3)
    ax.grid(True, alpha=0.25)
    if label:
        ax.legend(loc="best", fontsize=8)
    return True


def finalize_time_axes(fig: plt.Figure, axes: np.ndarray, title: str) -> None:
    fig.suptitle(title, fontsize=14)
    for ax in axes[-1, :]:
        ax.set_xlabel("time [s]")
    fig.tight_layout(rect=(0.0, 0.0, 1.0, 0.97))


def save_figure(fig: plt.Figure, out_dir: Path, filename: str) -> None:
    fig.savefig(out_dir / filename, dpi=150)
    plt.close(fig)


def interpolate_like(
    source: Tuple[np.ndarray, np.ndarray, str],
    target_time: np.ndarray,
) -> np.ndarray:
    return np.interp(target_time, source[0], source[1])


def plot_figure_1(
    tables: Dict[str, Optional[StoTable]],
    out_dir: Path,
    missing: MissingReport,
) -> None:
    fig, axes = plt.subplots(4, 2, figsize=(14, 11), sharex=False)
    row_labels = [
        "SEA torque [N*m]",
        "control input u",
        "reserve actuator",
        "SEA torque + reserve",
    ]
    for row, label in enumerate(row_labels):
        axes[row, 0].set_ylabel(label)

    for col, side in enumerate(SIDES):
        key = side["key"]
        title = side["title"]
        coord = side["coord"]
        sea = side["sea"]
        axes[0, col].set_title(title)

        torque = find_series(
            tables["sea_torques"],
            torque_candidates(coord, sea),
            missing,
            "figure 1",
            key,
            "SEA torque",
            "sim_output_sea_torques.sto",
        )
        torque = apply_joint_sign(torque, key)
        control = find_series(
            tables["sea_controls"],
            control_candidates(coord),
            missing,
            "figure 1",
            key,
            "control input",
            "sim_output_sea_controls.sto",
        )
        control = apply_joint_sign(control, key)
        reserve = find_series(
            tables["reserve_torques"],
            reserve_candidates(coord),
            missing,
            "figure 1",
            key,
            "reserve actuator",
            "sim_output_reserve_torques.sto",
        )
        reserve = apply_joint_sign(reserve, key)

        plot_time_series(axes[0, col], torque, "SEA torque", "tab:blue", "SEA torque")
        plot_time_series(axes[1, col], control, "control input", "tab:green", "control input")
        plot_time_series(axes[2, col], reserve, "reserve actuator", "tab:orange", "reserve actuator")

        plotted_overlay = False
        if torque is not None:
            time, values, _ = torque
            axes[3, col].plot(time, values, label="SEA torque", color="tab:blue", linewidth=1.3)
            plotted_overlay = True
        if reserve is not None:
            time, values, _ = reserve
            axes[3, col].plot(time, values, label="reserve", color="tab:orange", linewidth=1.3)
            plotted_overlay = True
        if not plotted_overlay:
            annotate_missing(axes[3, col], "SEA torque and reserve actuator")
        elif reserve is None:
            axes[3, col].text(
                0.02,
                0.92,
                "missing: reserve actuator",
                transform=axes[3, col].transAxes,
                color="crimson",
                fontsize=9,
                va="top",
            )
        axes[3, col].grid(True, alpha=0.25)
        if plotted_overlay:
            axes[3, col].legend(loc="best", fontsize=8)

    finalize_time_axes(fig, axes, "Time Signals: SEA, Control, Reserve")
    save_figure(fig, out_dir, "01_time_sea_control_reserve.png")


def plot_figure_2(
    tables: Dict[str, Optional[StoTable]],
    out_dir: Path,
    missing: MissingReport,
) -> None:
    fig, axes = plt.subplots(5, 2, figsize=(14, 13), sharex=False)
    row_labels = [
        "joint angle [rad]",
        "joint velocity [rad/s]",
        "SEA motor angle",
        "SEA motor speed",
        "motor + joint angle",
    ]
    for row, label in enumerate(row_labels):
        axes[row, 0].set_ylabel(label)

    for col, side in enumerate(SIDES):
        key = side["key"]
        title = side["title"]
        coord = side["coord"]
        sea = side["sea"]
        axes[0, col].set_title(title)

        joint_q = find_series(
            tables["states"],
            q_candidates(coord),
            missing,
            "figure 2",
            key,
            "joint angle",
            "sim_output_states.sto",
        )
        joint_q = apply_joint_sign(joint_q, key)
        joint_qdot = find_series(
            tables["states"],
            qdot_candidates(coord),
            missing,
            "figure 2",
            key,
            "joint velocity",
            "sim_output_states.sto",
        )
        joint_qdot = apply_joint_sign(joint_qdot, key)
        motor_q = find_series(
            tables["sea_states"],
            motor_angle_candidates(coord, sea),
            missing,
            "figure 2",
            key,
            "SEA motor angle",
            "sim_output_sea_states.sto",
        )
        motor_q = apply_joint_sign(motor_q, key)
        motor_qdot = find_series(
            tables["sea_states"],
            motor_speed_candidates(coord, sea),
            missing,
            "figure 2",
            key,
            "SEA motor speed",
            "sim_output_sea_states.sto",
        )
        motor_qdot = apply_joint_sign(motor_qdot, key)

        plot_time_series(axes[0, col], joint_q, "joint angle", "tab:blue", "joint angle")
        plot_time_series(axes[1, col], joint_qdot, "joint velocity", "tab:green", "joint velocity")
        plot_time_series(axes[2, col], motor_q, "motor angle", "tab:orange", "SEA motor angle")
        plot_time_series(axes[3, col], motor_qdot, "motor speed", "tab:red", "SEA motor speed")

        plotted_overlay = False
        if joint_q is not None:
            time, values, _ = joint_q
            axes[4, col].plot(time, values, label="joint angle", color="tab:blue", linewidth=1.3)
            plotted_overlay = True
        if motor_q is not None:
            time, values, _ = motor_q
            axes[4, col].plot(time, values, label="motor angle", color="tab:orange", linewidth=1.3)
            plotted_overlay = True
        if not plotted_overlay:
            annotate_missing(axes[4, col], "motor angle and joint angle")
        elif motor_q is None:
            axes[4, col].text(
                0.02,
                0.92,
                "missing: SEA motor angle",
                transform=axes[4, col].transAxes,
                color="crimson",
                fontsize=9,
                va="top",
            )
        axes[4, col].grid(True, alpha=0.25)
        if plotted_overlay:
            axes[4, col].legend(loc="best", fontsize=8)

    finalize_time_axes(fig, axes, "Time Signals: Joint and SEA Motor States")
    save_figure(fig, out_dir, "02_time_joint_motor_states.png")


def load_events(path: Optional[Path], missing: MissingReport) -> Dict[str, List[Tuple[float, float]]]:
    if path is None:
        missing.add("gait cycle events: not provided; pass --events side,cycle_start,cycle_end CSV")
        return {}
    if not path.is_file():
        missing.add(f"gait cycle events: file not found: {path}")
        return {}

    events: Dict[str, List[Tuple[float, float]]] = {
        "ankle": [],
        "knee": [],
        "left": [],
        "right": [],
        "all": [],
    }
    with path.open(newline="", encoding="utf-8-sig") as fh:
        reader = csv.DictReader(fh)
        if reader.fieldnames is None:
            missing.add(f"gait cycle events: empty CSV: {path}")
            return {}
        fields = {field.lower(): field for field in reader.fieldnames}
        side_field = fields.get("side")
        start_field = fields.get("cycle_start") or fields.get("start") or fields.get("start_time")
        end_field = fields.get("cycle_end") or fields.get("end") or fields.get("end_time")
        if not side_field or not start_field or not end_field:
            missing.add(
                "gait cycle events: CSV must include side,cycle_start,cycle_end columns"
            )
            return {}
        for row_num, row in enumerate(reader, start=2):
            side = row.get(side_field, "").strip().lower()
            try:
                start = float(row[start_field])
                end = float(row[end_field])
            except (TypeError, ValueError):
                missing.add(f"gait cycle events: invalid numeric row {row_num}")
                continue
            if end <= start:
                missing.add(f"gait cycle events: cycle_end <= cycle_start at row {row_num}")
                continue
            if side in {"ankle", "knee", "left", "right"}:
                events[side].append((start, end))
            elif side == "all":
                events["all"].append((start, end))
            else:
                missing.add(f"gait cycle events: unsupported side '{side}' at row {row_num}")
    return events


def cycles_for(
    side_key: str,
    events: Dict[str, List[Tuple[float, float]]],
    gait_side: str,
) -> List[Tuple[float, float]]:
    preferred = (
        events.get("left", []) + events.get("right", [])
        if gait_side == "all"
        else events.get(gait_side, [])
    )
    return (
        events.get(side_key, [])
        + preferred
        + events.get("all", [])
    )


def resample_by_cycles(
    time: np.ndarray,
    values: np.ndarray,
    cycles: Sequence[Tuple[float, float]],
) -> Optional[np.ndarray]:
    valid = []
    for start, end in cycles:
        if start < time[0] or end > time[-1] or end <= start:
            continue
        target_t = start + (GAIT_GRID / 100.0) * (end - start)
        valid.append(np.interp(target_t, time, values))
    if not valid:
        return None
    return np.vstack(valid)


def plot_mean_std_over_gait(
    ax: plt.Axes,
    cycles_data: np.ndarray,
    color: str,
    label: str,
    ylabel: str,
    linestyle: str = "-",
) -> None:
    mean = np.mean(cycles_data, axis=0)
    std = np.std(cycles_data, axis=0)
    ax.plot(GAIT_GRID, mean, color=color, label=label, linewidth=1.4, linestyle=linestyle)
    if cycles_data.shape[0] > 1:
        ax.fill_between(GAIT_GRID, mean - std, mean + std, color=color, alpha=0.2)
    ax.set_xlabel("gait cycle [%]")
    ax.set_ylabel(ylabel)
    ax.grid(True, alpha=0.25)
    ax.legend(loc="best", fontsize=8)


def plot_torque_angle_mean_std(
    ax: plt.Axes,
    angle_cycles: np.ndarray,
    torque_cycles: np.ndarray,
    color: str,
    label: str,
    linestyle: str = "-",
) -> None:
    mean_angle = np.mean(angle_cycles, axis=0)
    mean_torque = np.mean(torque_cycles, axis=0)
    torque_std = np.std(torque_cycles, axis=0)
    ax.plot(mean_angle, mean_torque, color=color, label=label, linewidth=1.4, linestyle=linestyle)
    if torque_cycles.shape[0] > 1:
        ax.fill_between(mean_angle, mean_torque - torque_std, mean_torque + torque_std, color=color, alpha=0.2)
    ax.set_xlabel("joint angle [rad]")
    ax.set_ylabel("SEA torque [N*m]")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="best", fontsize=8)


def gait_series_or_missing(
    ax: plt.Axes,
    series: Optional[Tuple[np.ndarray, np.ndarray, str]],
    cycles: Sequence[Tuple[float, float]],
    missing_label: str,
    missing: MissingReport,
    figure: str,
    side_key: str,
    channel: str,
) -> Optional[np.ndarray]:
    if not cycles:
        note_missing(missing, figure, side_key, channel, "gait cycle events not available")
        annotate_missing(ax, "gait cycle events")
        return None
    if series is None:
        annotate_missing(ax, missing_label)
        return None
    time, values, _ = series
    cycles_data = resample_by_cycles(time, values, cycles)
    if cycles_data is None:
        note_missing(missing, figure, side_key, channel, "no valid cycles inside signal time range")
        annotate_missing(ax, "valid gait cycles inside signal time range")
        return None
    return cycles_data


def plot_power_over_gait(
    ax: plt.Axes,
    joint_power: Optional[Tuple[np.ndarray, np.ndarray, str]],
    motor_power: Optional[Tuple[np.ndarray, np.ndarray, str]],
    healthy_power: Optional[Tuple[np.ndarray, np.ndarray, str]],
    cycles: Sequence[Tuple[float, float]],
    missing: MissingReport,
    figure: str,
    side_key: str,
) -> None:
    if not cycles:
        note_missing(missing, figure, side_key, "power", "gait cycle events not available")
        annotate_missing(ax, "gait cycle events")
        return

    plotted = False
    missing_labels = []
    for label, color, linestyle, series in [
        ("joint power", "tab:red", "-", joint_power),
        ("motor power", "tab:purple", "-", motor_power),
        ("healthy power", HEALTHY_COLOR, HEALTHY_LINESTYLE, healthy_power),
    ]:
        if series is None:
            missing_labels.append(label)
            continue
        cycles_data = resample_by_cycles(series[0], series[1], cycles)
        if cycles_data is None:
            note_missing(
                missing,
                figure,
                side_key,
                label,
                "no valid cycles inside signal time range",
            )
            missing_labels.append(label)
            continue
        plot_mean_std_over_gait(ax, cycles_data, color, label, "power [W]", linestyle=linestyle)
        plotted = True

    if not plotted:
        annotate_missing(ax, "power")
    elif missing_labels:
        ax.text(
            0.02,
            0.92,
            "missing: " + ", ".join(missing_labels),
            transform=ax.transAxes,
            color="crimson",
            fontsize=9,
            va="top",
        )


def plot_figure_3(
    tables: Dict[str, Optional[StoTable]],
    events: Dict[str, List[Tuple[float, float]]],
    gait_side: str,
    healthy: Optional[HealthyData],
    out_dir: Path,
    missing: MissingReport,
) -> None:
    fig, axes = plt.subplots(2, 2, figsize=(14, 8), sharex=False)
    for col, side in enumerate(SIDES):
        key = side["key"]
        coord = side["coord"]
        sea = side["sea"]
        axes[0, col].set_title(side["title"])
        cycles = cycles_for(key, events, gait_side)

        torque = find_series(
            tables["sea_torques"],
            torque_candidates(coord, sea),
            missing,
            "figure 3",
            key,
            "SEA torque",
            "sim_output_sea_torques.sto",
        )
        torque = apply_joint_sign(torque, key)
        angle = find_series(
            tables["states"],
            q_candidates(coord),
            missing,
            "figure 3",
            key,
            "joint angle",
            "sim_output_states.sto",
        )
        angle = apply_joint_sign(angle, key)
        joint_power = find_series(
            tables["power"],
            joint_power_candidates(coord, sea, key),
            missing,
            "figure 3",
            key,
            "joint power",
            "sim_output_power.sto",
        )
        motor_power = find_series(
            tables["power"],
            motor_power_candidates(coord, sea, key),
            missing,
            "figure 3",
            key,
            "motor power",
            "sim_output_power.sto",
        )
        healthy_angle = healthy_angle_series(healthy, coord, key, missing, "figure 3")
        healthy_torque = healthy_torque_series(healthy, coord, key, missing, "figure 3")
        healthy_power = healthy_power_series(healthy, coord, key, missing, "figure 3")

        if not cycles:
            note_missing(missing, "figure 3", key, "torque-angle", "gait cycle events not available")
            annotate_missing(axes[0, col], "gait cycle events")
        elif torque is not None and angle is not None:
            torque_cycles = resample_by_cycles(torque[0], torque[1], cycles)
            angle_cycles = resample_by_cycles(angle[0], angle[1], cycles)
            if torque_cycles is not None and angle_cycles is not None:
                plot_torque_angle_mean_std(axes[0, col], angle_cycles, torque_cycles, "tab:blue", "prosthetic")
            else:
                note_missing(missing, "figure 3", key, "torque-angle", "no valid cycles inside signal time range")
                annotate_missing(axes[0, col], "valid gait cycles inside signal time range")
        else:
            annotate_missing(axes[0, col], "SEA torque or joint angle")

        if cycles and healthy_torque is not None and healthy_angle is not None:
            healthy_torque_cycles = resample_by_cycles(healthy_torque[0], healthy_torque[1], cycles)
            healthy_angle_cycles = resample_by_cycles(healthy_angle[0], healthy_angle[1], cycles)
            if healthy_torque_cycles is not None and healthy_angle_cycles is not None:
                plot_torque_angle_mean_std(
                    axes[0, col],
                    healthy_angle_cycles,
                    healthy_torque_cycles,
                    HEALTHY_COLOR,
                    "healthy",
                    linestyle=HEALTHY_LINESTYLE,
                )

        plot_power_over_gait(
            axes[1, col],
            joint_power,
            motor_power,
            healthy_power,
            cycles,
            missing,
            "figure 3",
            key,
        )

    fig.suptitle("Gait Cycle: Torque-Angle and Power", fontsize=14)
    fig.tight_layout(rect=(0.0, 0.0, 1.0, 0.95))
    save_figure(fig, out_dir, "03_gaitcycle_torque_angle_power.png")


def plot_figure_4(
    tables: Dict[str, Optional[StoTable]],
    events: Dict[str, List[Tuple[float, float]]],
    gait_side: str,
    healthy: Optional[HealthyData],
    out_dir: Path,
    missing: MissingReport,
) -> None:
    fig, axes = plt.subplots(3, 2, figsize=(14, 9), sharex=False)
    for col, side in enumerate(SIDES):
        key = side["key"]
        coord = side["coord"]
        sea = side["sea"]
        axes[0, col].set_title(side["title"])
        cycles = cycles_for(key, events, gait_side)

        angle = find_series(
            tables["states"],
            q_candidates(coord),
            missing,
            "figure 4",
            key,
            "joint angle",
            "sim_output_states.sto",
        )
        angle = apply_joint_sign(angle, key)
        velocity = find_series(
            tables["states"],
            qdot_candidates(coord),
            missing,
            "figure 4",
            key,
            "joint velocity",
            "sim_output_states.sto",
        )
        velocity = apply_joint_sign(velocity, key)
        joint_power = find_series(
            tables["power"],
            joint_power_candidates(coord, sea, key),
            missing,
            "figure 4",
            key,
            "joint power",
            "sim_output_power.sto",
        )
        motor_power = find_series(
            tables["power"],
            motor_power_candidates(coord, sea, key),
            missing,
            "figure 4",
            key,
            "motor power",
            "sim_output_power.sto",
        )
        healthy_angle = healthy_angle_series(healthy, coord, key, missing, "figure 4")
        healthy_velocity = healthy_velocity_series(healthy, coord, key, missing, "figure 4")
        healthy_power = healthy_power_series(healthy, coord, key, missing, "figure 4")

        angle_cycles = gait_series_or_missing(
            axes[0, col],
            angle,
            cycles,
            "joint angle",
            missing,
            "figure 4",
            key,
            "joint angle",
        )
        if angle_cycles is not None:
            plot_mean_std_over_gait(axes[0, col], angle_cycles, "tab:blue", "prosthetic angle", "joint angle [rad]")
        if cycles and healthy_angle is not None:
            healthy_angle_cycles = resample_by_cycles(healthy_angle[0], healthy_angle[1], cycles)
            if healthy_angle_cycles is not None:
                plot_mean_std_over_gait(
                    axes[0, col],
                    healthy_angle_cycles,
                    HEALTHY_COLOR,
                    "healthy angle",
                    "joint angle [rad]",
                    linestyle=HEALTHY_LINESTYLE,
                )

        velocity_cycles = gait_series_or_missing(
            axes[1, col],
            velocity,
            cycles,
            "joint velocity",
            missing,
            "figure 4",
            key,
            "joint velocity",
        )
        if velocity_cycles is not None:
            plot_mean_std_over_gait(axes[1, col], velocity_cycles, "tab:green", "prosthetic velocity", "joint velocity [rad/s]")
        if cycles and healthy_velocity is not None:
            healthy_velocity_cycles = resample_by_cycles(healthy_velocity[0], healthy_velocity[1], cycles)
            if healthy_velocity_cycles is not None:
                plot_mean_std_over_gait(
                    axes[1, col],
                    healthy_velocity_cycles,
                    HEALTHY_COLOR,
                    "healthy velocity",
                    "joint velocity [rad/s]",
                    linestyle=HEALTHY_LINESTYLE,
                )

        plot_power_over_gait(
            axes[2, col],
            joint_power,
            motor_power,
            healthy_power,
            cycles,
            missing,
            "figure 4",
            key,
        )

    fig.suptitle("Gait Cycle: Joint Angle, Velocity, Power", fontsize=14)
    fig.tight_layout(rect=(0.0, 0.0, 1.0, 0.95))
    save_figure(fig, out_dir, "04_gaitcycle_joint_velocity_power.png")


def plot_figure_5(
    tables: Dict[str, Optional[StoTable]],
    sea_f_opt: Dict[str, float],
    out_dir: Path,
    missing: MissingReport,
) -> None:
    fig, axes = plt.subplots(3, 2, figsize=(14, 10), sharex=False)
    axes[0, 0].set_ylabel("tau_input [N*m]")
    axes[1, 0].set_ylabel("tau_ref - tau_spring [N*m]")
    axes[2, 0].set_ylabel("tau_ref - tau_input [N*m]")

    for col, side in enumerate(SIDES):
        key = side["key"]
        coord = side["coord"]
        sea = side["sea"]
        axes[0, col].set_title(side["title"])

        tau_input = find_series(
            tables["sea_torques"],
            motor_torque_candidates(coord, sea),
            missing,
            "figure 5",
            key,
            "tau_input",
            "sim_output_sea_torques.sto",
        )
        tau_input = apply_joint_sign(tau_input, key)
        tau_spring = find_series(
            tables["sea_torques"],
            torque_candidates(coord, sea),
            missing,
            "figure 5",
            key,
            "tau_spring",
            "sim_output_sea_torques.sto",
        )
        tau_spring = apply_joint_sign(tau_spring, key)
        control = find_series(
            tables["sea_controls"],
            control_candidates(coord),
            missing,
            "figure 5",
            key,
            "tau_ref control",
            "sim_output_sea_controls.sto",
        )
        control = apply_joint_sign(control, key)

        plot_time_series(
            axes[0, col],
            tau_input,
            "tau_input",
            "tab:purple",
            "tau_input",
        )

        f_opt = sea_f_opt.get(sea)
        if tau_spring is None or control is None or f_opt is None:
            if f_opt is None:
                note_missing(missing, "figure 5", key, "tau_ref", f"missing F_opt for {sea}")
            annotate_missing(axes[1, col], "tau_ref or tau_spring")
        else:
            time, spring_values, _ = tau_spring
            control_values = interpolate_like(control, time)
            tau_ref = control_values * f_opt
            error = tau_ref - spring_values
            axes[1, col].plot(
                time,
                error,
                label="tau_ref - tau_spring",
                color="tab:red",
                linewidth=1.3,
            )
            axes[1, col].axhline(0.0, color="0.3", linewidth=0.8, linestyle="--")
            axes[1, col].grid(True, alpha=0.25)
            axes[1, col].legend(loc="best", fontsize=8)

        if tau_input is None or control is None or f_opt is None:
            annotate_missing(axes[2, col], "tau_ref or tau_input")
        else:
            time, input_values, _ = tau_input
            control_values = interpolate_like(control, time)
            tau_ref = control_values * f_opt
            error = tau_ref - input_values
            axes[2, col].plot(
                time,
                error,
                label="tau_ref - tau_input",
                color="tab:orange",
                linewidth=1.3,
            )
            axes[2, col].axhline(0.0, color="0.3", linewidth=0.8, linestyle="--")
            axes[2, col].grid(True, alpha=0.25)
            axes[2, col].legend(loc="best", fontsize=8)

    finalize_time_axes(fig, axes, "SEA Motor Torque and Tracking Error")
    save_figure(fig, out_dir, "05_time_tau_input_tracking_error.png")


def parse_args() -> argparse.Namespace:
    default_cfg = SimulatorConfig()
    parser = argparse.ArgumentParser(
        description="Plot ankle/knee diagnostics from simulator .sto outputs."
    )
    parser.add_argument("--results-dir", default="results", help="Directory containing .sto results.")
    parser.add_argument("--out-root", default="plot", help="Root directory for dated PNG folders.")
    parser.add_argument("--events", default=None, help="CSV with side,cycle_start,cycle_end gait-cycle events.")
    parser.add_argument(
        "--healthy-dir",
        default=None,
        help="Directory containing healthy *Kinematics_q.sto and *Actuation_force.sto.",
    )
    parser.add_argument("--prefix", default="sim_output", help="Result file prefix.")
    parser.add_argument(
        "--gait-side",
        default=default_cfg.plot_gait_side,
        choices=["left", "right", "all"],
        help="Gait event side to use for ankle/knee gait-cycle plots.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    results_dir = resolve_project_path(args.results_dir)
    out_root = resolve_project_path(args.out_root)
    if args.events:
        events_path = resolve_project_path(args.events)
    else:
        default_events = results_dir / f"{args.prefix}_gait_events.csv"
        events_path = default_events if default_events.is_file() else None

    missing = MissingReport()

    if not results_dir.is_dir():
        raise FileNotFoundError(f"Results directory not found: {results_dir}")

    healthy_dir = resolve_project_path(args.healthy_dir) if args.healthy_dir else default_healthy_dir()
    healthy = load_healthy_data(healthy_dir, missing)
    if healthy is not None:
        print(f"Healthy overlay loaded from: {healthy.directory}")
        for note in healthy.notes:
            print(f"  - {note}")

    out_dir = next_output_dir(out_root)
    tables = load_tables(results_dir, args.prefix)
    events = load_events(events_path, missing)
    sea_f_opt = load_sea_f_opt(SimulatorConfig(), missing)

    plot_figure_1(tables, out_dir, missing)
    plot_figure_2(tables, out_dir, missing)
    plot_figure_3(tables, events, args.gait_side, healthy, out_dir, missing)
    plot_figure_4(tables, events, args.gait_side, healthy, out_dir, missing)
    plot_figure_5(tables, sea_f_opt, out_dir, missing)

    missing_path = out_dir / "missing_channels.txt"
    missing.write(missing_path)

    if missing.items:
        print("Missing channels / unavailable inputs:")
        for item in missing.items:
            print(f"  - {item}")
    else:
        print("No missing channels.")

    print(f"Plots saved to: {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
