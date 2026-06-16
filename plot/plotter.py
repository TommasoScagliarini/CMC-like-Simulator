"""
plotter.py
==========
Read simulator .sto outputs and save ankle/knee diagnostic PNG plots.

This script is intentionally a consumer of existing result channels. It does
not derive missing simulator channels such as SEA motor states, reserve
actuator torque, power, or gait-cycle events. Healthy reference files are
loaded from the active model bundle first, with a fallback to repo-level
data/health or data/healthy when available; reference velocity and power are
derived only for plotting. Missing channels are reported and annotated in the
generated figures.
"""

from __future__ import annotations

import argparse
import copy
import csv
import json
import os
import re
import sys
import tempfile
from xml.etree import ElementTree as ET
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

os.environ.setdefault("MPLCONFIGDIR", str(Path(tempfile.gettempdir()) / "cmc_like_matplotlib"))
os.environ.setdefault("XDG_CACHE_HOME", str(Path(tempfile.gettempdir()) / "cmc_like_cache"))

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent
TRAJ_GEN_ROLLOUT_ROOT = REPO_ROOT / "Trajectory Generator" / "runs" / "rollout"
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from output import read_sto  # noqa: E402
from config import SimulatorConfig  # noqa: E402
from kinematics_interpolator import KinematicsInterpolator  # noqa: E402
from path_resolver import (  # noqa: E402
    normalize_cli_existing_path,
    resolve_repo_path,
    resolve_simulator_paths,
)
from setup_io import read_last_setup_path, read_setup_xml  # noqa: E402


GAIT_GRID = np.linspace(0.0, 100.0, 101)
HEALTHY_COLOR = "#ffa500"
HEALTHY_LINESTYLE = "--"
TAU_INPUT_SATURATION_NM = 500.0
SATURATION_MARKER_COLOR = "crimson"

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


@dataclass
class MlpRolloutContext:
    rollout_dir: Path
    sim_outputs_dir: Path
    summary_path: Path
    trace_path: Path
    summary: Dict[str, object]
    reward_mode: str


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
    return resolve_repo_path(raw)


def plot_output_index(dirname: str, date_str: str) -> Optional[int]:
    """Extract N from MM_DD_YYYY_N, allowing descriptive suffixes after N."""
    match = re.match(rf"^{re.escape(date_str)}_(\d+)(?:$|[\s_-].*)", dirname)
    if match is None:
        return None
    return int(match.group(1))


def next_output_dir(out_root: Path) -> Path:
    out_root.mkdir(parents=True, exist_ok=True)
    date_str = datetime.now().strftime("%m_%d_%Y")
    existing = []
    for child in out_root.iterdir():
        if not child.is_dir():
            continue
        idx = plot_output_index(child.name, date_str)
        if idx is not None:
            existing.append(idx)
    idx = max(existing, default=0) + 1
    out_dir = out_root / f"{date_str}_{idx}"
    out_dir.mkdir(parents=False, exist_ok=False)
    return out_dir


def load_table(path: Path) -> Optional[StoTable]:
    if not path.is_file():
        return None
    time, columns, data, in_degrees = read_sto(str(path))
    return StoTable(path, time, columns, data, in_degrees)


def default_healthy_dir(cfg: SimulatorConfig) -> Optional[Path]:
    return resolve_simulator_paths(cfg).healthy_dir


def apply_setup_to_config(cfg: SimulatorConfig, setup_path: Path) -> None:
    """Point plotting config at the same files used by a simulator setup XML."""
    setup = read_setup_xml(setup_path)
    cfg.model_bundle_dir = str(setup.model_file.parent)
    cfg.model_file = str(setup.model_file)
    cfg.kinematics_file = str(setup.kinematics_file)
    cfg.external_loads_xml = str(setup.external_loads_xml)
    cfg.reserve_actuators_xml = str(setup.reserve_actuators_xml)
    cfg.t_start = setup.t_start
    cfg.t_end = setup.t_end


def fallback_setup_path(setup_path: Path) -> Optional[Path]:
    """Recover from a stale last-setup filename when the bundle has one setup XML."""
    if setup_path.is_file():
        return setup_path
    if not setup_path.parent.is_dir():
        return None

    candidates = sorted(setup_path.parent.glob("*setup*.xml"))
    return candidates[0] if len(candidates) == 1 else None


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


def load_reference_kinematics(
    reference_path: Optional[str],
    cfg: SimulatorConfig,
    missing: MissingReport,
) -> Optional[KinematicsInterpolator]:
    """Load the active kinematic reference with the same preprocessing as the simulator."""
    raw_path = reference_path or cfg.kinematics_file
    if not raw_path:
        missing.add("reference kinematics: no reference path configured")
        return None

    ref_cfg = copy.deepcopy(cfg)
    if reference_path is not None:
        ref_cfg.kinematics_file = str(resolve_reference_path(raw_path, cfg))
    else:
        ref_cfg.kinematics_file = str(resolve_simulator_paths(cfg).kinematics_path)
    try:
        return KinematicsInterpolator(ref_cfg)
    except Exception as exc:
        missing.add(f"reference kinematics: could not load {ref_cfg.kinematics_file}: {exc}")
        return None


def resolve_reference_path(raw: str | Path, cfg: SimulatorConfig) -> Path:
    """Resolve explicit reference IK paths like the simulator resolves bundle inputs."""
    path = Path(raw)
    if path.is_absolute():
        return path
    repo_candidate = REPO_ROOT / path
    if repo_candidate.is_file():
        return repo_candidate
    return resolve_simulator_paths(cfg).model_bundle_dir / path


def load_tables(results_dir: Path, prefix: str) -> Dict[str, Optional[StoTable]]:
    suffixes = {
        "sea_torques": "sea_torques",
        "sea_controls": "sea_controls",
        "states": "states",
        "kinematics_reference": "kinematics_reference",
        "reserve_torques": "reserve_torques",
        "sea_states": "sea_states",
        "sea_diagnostics": "sea_diagnostics",
        "power": "power",
    }
    return {
        key: load_table(results_dir / f"{prefix}_{suffix}.sto")
        for key, suffix in suffixes.items()
    }


def read_json_dict(path: Path) -> Dict[str, object]:
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return {}
    return data if isinstance(data, dict) else {}


def rollout_timestamp(rollout_dir: Path, summary_path: Path) -> float:
    try:
        return summary_path.stat().st_mtime
    except OSError:
        pass
    try:
        return rollout_dir.stat().st_mtime
    except OSError:
        return 0.0


def valid_mlp_rollout_context(rollout_dir: Path) -> Optional[MlpRolloutContext]:
    summary_path = rollout_dir / "rollout_summary.json"
    trace_path = rollout_dir / "rollout_policy_trace.json"
    sim_outputs_dir = rollout_dir / "sim_outputs"
    if not summary_path.is_file() or not trace_path.is_file():
        return None
    if not sim_outputs_dir.is_dir():
        return None

    summary = read_json_dict(summary_path)
    if summary.get("ok") is False:
        return None
    reward = summary.get("reward_config")
    reward_mode = "unknown"
    if isinstance(reward, dict):
        reward_mode = str(reward.get("reward_mode", "unknown"))
    return MlpRolloutContext(
        rollout_dir=rollout_dir.resolve(),
        sim_outputs_dir=sim_outputs_dir.resolve(),
        summary_path=summary_path.resolve(),
        trace_path=trace_path.resolve(),
        summary=summary,
        reward_mode=reward_mode,
    )


def latest_mlp_rollout_context(
    rollout_root: Path = TRAJ_GEN_ROLLOUT_ROOT,
) -> MlpRolloutContext:
    if not rollout_root.is_dir():
        raise FileNotFoundError(f"MLP rollout root not found: {rollout_root}")

    candidates: List[Tuple[float, str, MlpRolloutContext]] = []
    for child in rollout_root.iterdir():
        if not child.is_dir():
            continue
        context = valid_mlp_rollout_context(child)
        if context is None:
            continue
        candidates.append(
            (
                rollout_timestamp(context.rollout_dir, context.summary_path),
                context.rollout_dir.name.lower(),
                context,
            )
        )
    if not candidates:
        raise FileNotFoundError(
            f"No valid MLP rollout found in {rollout_root}. "
            "Expected rollout_summary.json, rollout_policy_trace.json and sim_outputs/."
        )
    candidates.sort(key=lambda item: (item[0], item[1]))
    return candidates[-1][2]


def infer_result_time_range(tables: Dict[str, Optional[StoTable]]) -> Optional[Tuple[float, float]]:
    for table in tables.values():
        if table is not None and table.time.size:
            return float(table.time[0]), float(table.time[-1])
    return None


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


def reference_angle_series(
    reference: Optional[KinematicsInterpolator],
    time: np.ndarray,
    coord: str,
    side_key: str,
    missing: MissingReport,
    figure: str,
) -> Optional[Tuple[np.ndarray, np.ndarray, str]]:
    if reference is None:
        return None
    if coord not in reference.coord_names:
        note_missing(missing, figure, side_key, "kinematic ref", f"coordinate not found in reference: {coord}")
        return None
    try:
        values = np.array([reference.get(float(t))[0][coord] for t in time], dtype=float)
    except Exception as exc:
        note_missing(missing, figure, side_key, "kinematic ref", f"could not evaluate reference: {exc}")
        return None
    return apply_joint_sign((time, values, f"{coord}_q_ref"), side_key)


def recorded_reference_angle_series(
    table: Optional[StoTable],
    time: np.ndarray,
    coord: str,
    side_key: str,
) -> Optional[Tuple[np.ndarray, np.ndarray, str]]:
    """Return the reference actually served to the controller, when recorded."""
    if table is None:
        return None
    series = table.series([f"{coord}_q_ref"])
    if series is None:
        return None
    source_time, source_values, column = series
    values = np.interp(time, source_time, source_values)
    return apply_joint_sign((time, values, column), side_key)


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


def sound_q_candidates(side_key: str) -> List[str]:
    raw_coord = "knee_angle_r" if side_key == "knee" else "ankle_angle_r"
    return [f"{raw_coord}_q", raw_coord]


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


def saturation_times(
    tables: Dict[str, Optional[StoTable]],
    sea: str,
) -> np.ndarray:
    """Return times where the SEA motor torque reaches the plugin clamp."""
    diagnostics = tables.get("sea_diagnostics")
    if diagnostics is not None:
        flag = diagnostics.series([f"{sea}_tau_input_saturated"])
        if flag is not None:
            time, values, _ = flag
            return np.asarray(time)[np.asarray(values) > 0.5]
        tau_input = diagnostics.series([
            f"{sea}_tau_input_plugin",
            f"{sea}_tau_input_python",
        ])
        if tau_input is not None:
            time, values, _ = tau_input
            return np.asarray(time)[
                np.abs(values) >= TAU_INPUT_SATURATION_NM - 1e-6
            ]

    torques = tables.get("sea_torques")
    if torques is None:
        return np.array([], dtype=float)
    tau_input = torques.series(motor_torque_candidates("", sea))
    if tau_input is None:
        return np.array([], dtype=float)
    time, values, _ = tau_input
    return np.asarray(time)[np.abs(values) >= TAU_INPUT_SATURATION_NM - 1e-6]


def saturation_cycle_percentages(
    times: np.ndarray,
    cycles: Sequence[Tuple[float, float]],
) -> np.ndarray:
    """Project absolute saturation times into gait-cycle percentage."""
    percentages: List[float] = []
    for sat_time in times:
        for start, end in cycles:
            if start <= sat_time <= end and end > start:
                percentages.append((sat_time - start) / (end - start) * 100.0)
                break
    return np.array(percentages, dtype=float)


def mark_time_saturations(ax: plt.Axes, times: np.ndarray) -> None:
    """Draw vertical markers for tau_input clamp events on time plots."""
    if times.size == 0:
        return
    for sat_time in times:
        ax.axvline(
            sat_time,
            color=SATURATION_MARKER_COLOR,
            linestyle=":",
            linewidth=0.8,
            alpha=0.28,
        )
    ax.text(
        0.02,
        0.84,
        f"tau_input sat: {times.size}",
        transform=ax.transAxes,
        color=SATURATION_MARKER_COLOR,
        fontsize=8,
        va="top",
    )


def mark_gait_saturations(
    ax: plt.Axes,
    percentages: Optional[np.ndarray],
) -> None:
    """Draw vertical saturation markers on gait-cycle percentage plots."""
    if percentages is None or percentages.size == 0:
        return
    for pct in percentages:
        ax.axvline(
            pct,
            color=SATURATION_MARKER_COLOR,
            linestyle=":",
            linewidth=0.8,
            alpha=0.28,
        )
    ax.text(
        0.02,
        0.84,
        f"tau_input sat: {percentages.size}",
        transform=ax.transAxes,
        color=SATURATION_MARKER_COLOR,
        fontsize=8,
        va="top",
    )


def xml_local_name(tag: str) -> str:
    return tag.rsplit("}", 1)[-1]


def child_text(element: ET.Element, child_name: str) -> Optional[str]:
    for child in list(element):
        if xml_local_name(child.tag) == child_name:
            return child.text.strip() if child.text is not None else ""
    return None


def load_sea_params(cfg: SimulatorConfig) -> Dict[str, Dict[str, float]]:
    """Load inner-loop properties from the model XML for each SEA actuator."""
    model_path = resolve_simulator_paths(cfg).model_path
    sea_names = [cfg.sea_knee_name, cfg.sea_ankle_name]
    params: Dict[str, Dict[str, float]] = {}
    if not model_path.is_file():
        return params
    try:
        root = ET.parse(model_path).getroot()
    except ET.ParseError:
        return params
    for element in root.iter():
        if xml_local_name(element.tag) != "SeriesElasticActuator":
            continue
        name = element.attrib.get("name")
        if name not in sea_names:
            continue
        p: Dict[str, float] = {}
        for prop in ("stiffness", "Kp", "Kd", "Ki", "integral_torque_limit"):
            text = child_text(element, prop)
            if text:
                try:
                    p[prop] = float(text)
                except ValueError:
                    pass
        if p:
            params[name] = p
    return params


def outer_loop_subtitle(cfg: SimulatorConfig) -> str:
    """Build a subtitle string showing the active prosthetic outer-loop controller."""
    mode = str(getattr(cfg, "sea_outer_controller_mode", "pd")).upper()
    if mode == "CASCADE":
        parts = ["Outer loop: prosthetic CASCADE position-P / velocity-PI"]
    else:
        parts = [f"Outer loop: prosthetic {mode} torque"]
    for coord, label in [("pros_knee_angle", "Knee"), ("pros_ankle_angle", "Ankle")]:
        gains: List[str] = []
        if mode == "CASCADE":
            kp_outer = getattr(cfg, "sea_cascade_kp_outer", {}).get(coord)
            kp_inner = getattr(cfg, "sea_cascade_kp_inner", {}).get(coord)
            ki_inner = getattr(cfg, "sea_cascade_ki_inner", {}).get(coord)
            i_torque_limit = getattr(
                cfg,
                "sea_cascade_inner_i_torque_limit",
                {},
            ).get(coord)
            if kp_outer is not None:
                gains.append(f"Kp_outer={kp_outer:g}")
            if kp_inner is not None:
                gains.append(f"Kp_inner={kp_inner:g}")
            if ki_inner is not None:
                gains.append(f"Ki_inner={ki_inner:g}")
            if i_torque_limit is not None:
                gains.append(f"Ilim_tau={i_torque_limit:g}")
        else:
            kp = cfg.sea_kp.get(coord)
            kd = cfg.sea_kd.get(coord)
            ki = getattr(cfg, "sea_ki", {}).get(coord)
            integral_limit = getattr(cfg, "sea_integral_limit", {}).get(coord)
            if kp is not None:
                gains.append(f"Kp={kp:g}")
            if kd is not None:
                gains.append(f"Kd={kd:g}")
            if mode == "PID" and ki is not None:
                gains.append(f"Ki={ki:g}")
            if mode == "PID" and integral_limit is not None:
                gains.append(f"Ilim={integral_limit:g}")
        if gains:
            parts.append(f"{label} " + ", ".join(gains))
    if mode == "PID":
        leak = getattr(cfg, "sea_integral_leak_s_inv", None)
        if leak is not None:
            parts.append(f"leak={leak:g} 1/s")
    return "   |   ".join(parts)


def plugin_display_name(cfg: SimulatorConfig) -> str:
    configured = str(getattr(cfg, "plugin_name", "") or "").strip()
    configured_name = Path(configured).name if configured else "unknown"
    try:
        plugin_path = resolve_simulator_paths(cfg).plugin_path
    except Exception:
        return f"{configured_name} (configured, unresolved)"

    candidates = [plugin_path]
    if not plugin_path.suffix:
        candidates.extend(
            [
                plugin_path.with_suffix(".dll"),
                plugin_path.with_suffix(".dylib"),
                plugin_path.with_suffix(".so"),
                plugin_path.parent / f"lib{plugin_path.name}.dylib",
                plugin_path.parent / f"lib{plugin_path.name}.so",
            ]
        )
    for candidate in candidates:
        if candidate.is_file():
            return candidate.name
    return f"{plugin_path.name} (configured, file not found)"


def inner_loop_subtitle(sea_params: Dict[str, Dict[str, float]], cfg: SimulatorConfig) -> str:
    """Build a subtitle string showing the active plugin and SEA inner-loop gains."""
    has_pi = any(abs(float(p.get("Ki", 0.0) or 0.0)) > 0.0 for p in sea_params.values())
    plugin_label = "PI plugin" if has_pi else f"plugin {plugin_display_name(cfg)}"
    parts = [f"Inner loop: SEA {plugin_label}"]
    for sea_name, label in [(cfg.sea_knee_name, "Knee"), (cfg.sea_ankle_name, "Ankle")]:
        p = sea_params.get(sea_name, {})
        k = p.get("stiffness", cfg.sea_stiffness.get(sea_name))
        kp = p.get("Kp")
        kd = p.get("Kd")
        ki = p.get("Ki")
        i_limit = p.get("integral_torque_limit")
        params: List[str] = []
        if k is not None:
            params.append(f"K={k:g}")
        if kp is not None:
            params.append(f"Kp={kp:g}")
        if kd is not None:
            params.append(f"Kd={kd:g}")
        if ki is not None and abs(float(ki)) > 0.0:
            params.append(f"Ki={ki:g}")
        if i_limit is not None and abs(float(ki or 0.0)) > 0.0:
            params.append(f"Ilim_tau={i_limit:g}")
        if params:
            parts.append(f"{label} " + ", ".join(params))
    return "   |   ".join(parts)


def apply_figure_header(
    fig: plt.Figure,
    title: str,
    outer_subtitle: str = "",
    inner_subtitle: str = "",
) -> float:
    """Draw a 3-line header and return the top margin reserved for plots."""
    fig.suptitle(title, fontsize=14, y=0.992)

    subtitle_lines = [line for line in (outer_subtitle, inner_subtitle) if line]
    if not subtitle_lines:
        return 0.965

    y_positions = [0.965, 0.941]
    for y, line in zip(y_positions, subtitle_lines):
        fig.text(
            0.5,
            y,
            line,
            ha="center",
            va="top",
            fontsize=8.8,
            color="0.35",
        )
    return 0.885 if len(subtitle_lines) == 2 else 0.925


def load_sea_f_opt(cfg: SimulatorConfig, missing: MissingReport) -> Dict[str, float]:
    model_path = resolve_simulator_paths(cfg).model_path
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


def finalize_time_axes(
    fig: plt.Figure,
    axes: np.ndarray,
    title: str,
    outer_subtitle: str = "",
    inner_subtitle: str = "",
) -> None:
    top_margin = apply_figure_header(fig, title, outer_subtitle, inner_subtitle)
    for ax in axes[-1, :]:
        ax.set_xlabel("time [s]")
    fig.tight_layout(rect=(0.0, 0.0, 1.0, top_margin))


def save_figure(fig: plt.Figure, out_dir: Path, filename: str) -> None:
    fig.savefig(out_dir / filename, dpi=150)
    plt.close(fig)


def interpolate_like(
    source: Tuple[np.ndarray, np.ndarray, str],
    target_time: np.ndarray,
) -> np.ndarray:
    return np.interp(target_time, source[0], source[1])


def load_policy_trace(path: Path, missing: MissingReport) -> List[dict]:
    if not path.is_file():
        missing.add(f"figure 7 / all / policy trace: file not found: {path}")
        return []
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        missing.add(f"figure 7 / all / policy trace: could not read JSON: {exc}")
        return []
    if not isinstance(data, list):
        missing.add("figure 7 / all / policy trace: expected a JSON list")
        return []
    return [item for item in data if isinstance(item, dict)]


def policy_trace_series(
    trace: Sequence[dict],
    coord_index: int,
    side_key: str,
    missing: MissingReport,
) -> Optional[Tuple[np.ndarray, np.ndarray, str]]:
    times: List[float] = []
    values: List[float] = []
    for item in trace:
        segment_times = item.get("policy_segment_times")
        segment_values = item.get("policy_segment_values")
        if not isinstance(segment_times, list) or not isinstance(segment_values, list):
            continue
        for t_raw, row in zip(segment_times, segment_values):
            if not isinstance(row, list) or len(row) <= coord_index:
                continue
            try:
                t = float(t_raw)
                value = float(row[coord_index])
            except (TypeError, ValueError):
                continue
            if np.isfinite(t) and np.isfinite(value):
                times.append(t)
                values.append(value)

    if not times:
        note_missing(
            missing,
            "figure 7",
            side_key,
            "policy trajectory",
            "no finite policy_segment_times/policy_segment_values in trace",
        )
        return None

    time_arr = np.asarray(times, dtype=float)
    value_arr = np.asarray(values, dtype=float)
    order = np.argsort(time_arr)
    time_arr, value_arr = unique_time_series(time_arr[order], value_arr[order])
    return apply_joint_sign((time_arr, value_arr, "policy_segment_values"), side_key)


def figure7_policy_sign(side_key: str) -> float:
    return joint_sign(side_key)


def served_policy_reference_series(
    tables: Dict[str, Optional[StoTable]],
    coord: str,
    side_key: str,
    missing: MissingReport,
) -> Optional[Tuple[np.ndarray, np.ndarray, str]]:
    table = tables.get("kinematics_reference")
    if table is None:
        note_missing(
            missing,
            "figure 7",
            side_key,
            "policy C2 served reference",
            "file not found: rollout_episode_kinematics_reference.sto",
        )
        return None
    series = table.series([f"{coord}_q_ref"])
    if series is None:
        note_missing(
            missing,
            "figure 7",
            side_key,
            "policy C2 served reference",
            f"column not found: {coord}_q_ref",
        )
        return None
    time, values, column = series
    time, values = unique_time_series(time, values)
    return time, values * figure7_policy_sign(side_key), column


def shifted_healthy_target_series(
    trace: Sequence[dict],
    coord: str,
    side_key: str,
    missing: MissingReport,
) -> Optional[Tuple[np.ndarray, np.ndarray, str]]:
    times: List[float] = []
    values: List[float] = []
    for item in trace:
        target = item.get("imitation_target_q")
        if not isinstance(target, dict) or coord not in target:
            continue
        try:
            t = float(item.get("time"))
            value = float(target[coord])
        except (TypeError, ValueError):
            continue
        if np.isfinite(t) and np.isfinite(value):
            times.append(t)
            values.append(value)

    if not times:
        note_missing(
            missing,
            "figure 7",
            side_key,
            "shifted healthy target",
            f"no finite imitation_target_q[{coord!r}] in policy trace",
        )
        return None

    time_arr = np.asarray(times, dtype=float)
    value_arr = np.asarray(values, dtype=float)
    order = np.argsort(time_arr)
    time_arr, value_arr = unique_time_series(time_arr[order], value_arr[order])
    return (
        time_arr,
        value_arr * figure7_policy_sign(side_key),
        f"imitation_target_q[{coord}]",
    )


def plot_figure_1(
    tables: Dict[str, Optional[StoTable]],
    out_dir: Path,
    missing: MissingReport,
    outer_subtitle: str = "",
    inner_subtitle: str = "",
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

    finalize_time_axes(
        fig,
        axes,
        "Time Signals: SEA, Control, Reserve",
        outer_subtitle,
        inner_subtitle,
    )
    save_figure(fig, out_dir, "01_time_sea_control_reserve.png")


def plot_figure_2(
    tables: Dict[str, Optional[StoTable]],
    out_dir: Path,
    missing: MissingReport,
    outer_subtitle: str = "",
    inner_subtitle: str = "",
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
        sat_times = saturation_times(tables, sea)
        mark_time_saturations(axes[3, col], sat_times)

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

    finalize_time_axes(
        fig,
        axes,
        "Time Signals: Joint and SEA Motor States",
        outer_subtitle,
        inner_subtitle,
    )
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
    if not any(events.values()):
        missing.add(f"gait cycle events: no valid cycles found in {path}")
    return events


def describe_events(events: Dict[str, List[Tuple[float, float]]]) -> str:
    parts = []
    for side in ("left", "right", "ankle", "knee", "all"):
        count = len(events.get(side, []))
        if count:
            parts.append(f"{side}: {count}")
    return ", ".join(parts) if parts else "no valid cycles"


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
    saturation_percentages: Optional[np.ndarray] = None,
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
    if plotted:
        mark_gait_saturations(ax, saturation_percentages)


def plot_figure_3(
    tables: Dict[str, Optional[StoTable]],
    events: Dict[str, List[Tuple[float, float]]],
    gait_side: str,
    healthy: Optional[HealthyData],
    out_dir: Path,
    missing: MissingReport,
    outer_subtitle: str = "",
    inner_subtitle: str = "",
) -> None:
    fig, axes = plt.subplots(2, 2, figsize=(14, 8), sharex=False)
    for col, side in enumerate(SIDES):
        key = side["key"]
        coord = side["coord"]
        sea = side["sea"]
        axes[0, col].set_title(side["title"])
        cycles = cycles_for(key, events, gait_side)
        sat_pct = saturation_cycle_percentages(
            saturation_times(tables, sea),
            cycles,
        )

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
            saturation_percentages=sat_pct,
        )

    top_margin = apply_figure_header(
        fig,
        "Gait Cycle: Torque-Angle and Power",
        outer_subtitle,
        inner_subtitle,
    )
    fig.tight_layout(rect=(0.0, 0.0, 1.0, top_margin))
    save_figure(fig, out_dir, "03_gaitcycle_torque_angle_power.png")


def plot_figure_4(
    tables: Dict[str, Optional[StoTable]],
    events: Dict[str, List[Tuple[float, float]]],
    gait_side: str,
    healthy: Optional[HealthyData],
    out_dir: Path,
    missing: MissingReport,
    outer_subtitle: str = "",
    inner_subtitle: str = "",
) -> None:
    fig, axes = plt.subplots(3, 2, figsize=(14, 9), sharex=False)
    for col, side in enumerate(SIDES):
        key = side["key"]
        coord = side["coord"]
        sea = side["sea"]
        axes[0, col].set_title(side["title"])
        cycles = cycles_for(key, events, gait_side)
        sat_pct = saturation_cycle_percentages(
            saturation_times(tables, sea),
            cycles,
        )

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
            saturation_percentages=sat_pct,
        )

    top_margin = apply_figure_header(
        fig,
        "Gait Cycle: Joint Angle, Velocity, Power",
        outer_subtitle,
        inner_subtitle,
    )
    fig.tight_layout(rect=(0.0, 0.0, 1.0, top_margin))
    save_figure(fig, out_dir, "04_gaitcycle_joint_velocity_power.png")


def plot_figure_5(
    tables: Dict[str, Optional[StoTable]],
    sea_f_opt: Dict[str, float],
    out_dir: Path,
    missing: MissingReport,
    outer_subtitle: str = "",
    inner_subtitle: str = "",
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
        sat_times = saturation_times(tables, sea)

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

        for row in range(3):
            mark_time_saturations(axes[row, col], sat_times)

    finalize_time_axes(
        fig,
        axes,
        "SEA Motor Torque and Tracking Error",
        outer_subtitle,
        inner_subtitle,
    )
    save_figure(fig, out_dir, "05_time_tau_input_tracking_error.png")


def plot_figure_6(
    tables: Dict[str, Optional[StoTable]],
    reference: Optional[KinematicsInterpolator],
    out_dir: Path,
    missing: MissingReport,
    outer_subtitle: str = "",
    inner_subtitle: str = "",
) -> None:
    fig, axes = plt.subplots(3, 2, figsize=(14, 10), sharex=False)
    row_labels = [
        "kinematic ref [rad]",
        "simulated joint angle [rad]",
        "simulation - kin ref [rad]",
    ]
    for row, label in enumerate(row_labels):
        axes[row, 0].set_ylabel(label)

    for col, side in enumerate(SIDES):
        key = side["key"]
        coord = side["coord"]
        axes[0, col].set_title(side["title"])

        joint_q = find_series(
            tables["states"],
            q_candidates(coord),
            missing,
            "figure 6",
            key,
            "joint angle",
            "sim_output_states.sto",
        )
        joint_q = apply_joint_sign(joint_q, key)

        kin_ref = None
        kin_ref_label = "served kinematic ref"
        if joint_q is not None:
            kin_ref = recorded_reference_angle_series(
                tables["kinematics_reference"],
                joint_q[0],
                coord,
                key,
            )
            if kin_ref is None:
                kin_ref_label = "prescribed kinematic ref (fallback)"
                kin_ref = reference_angle_series(
                    reference,
                    joint_q[0],
                    coord,
                    key,
                    missing,
                    "figure 6",
                )
        elif reference is None:
            note_missing(missing, "figure 6", key, "kinematic ref", "reference kinematics not available")

        if kin_ref is None:
            annotate_missing(axes[0, col], "kinematic ref")
        else:
            time, values, _ = kin_ref
            axes[0, col].plot(
                time,
                values,
                label=kin_ref_label,
                color="tab:orange",
                linewidth=1.3,
            )
            axes[0, col].grid(True, alpha=0.25)
            axes[0, col].legend(loc="best", fontsize=8)

        if joint_q is None:
            annotate_missing(axes[1, col], "simulated joint angle")
        else:
            time, values, _ = joint_q
            axes[1, col].plot(
                time,
                values,
                label="simulated joint angle",
                color="tab:blue",
                linewidth=1.3,
            )
            axes[1, col].grid(True, alpha=0.25)
            axes[1, col].legend(loc="best", fontsize=8)

        if joint_q is None:
            note_missing(missing, "figure 6", key, "simulation - kin ref", "simulated joint angle not available")
            annotate_missing(axes[2, col], "simulated joint angle")
        else:
            if kin_ref is None:
                annotate_missing(axes[2, col], "kinematic ref")
            else:
                time, joint_values, _ = joint_q
                _, ref_values, _ = kin_ref
                error = joint_values - ref_values
                axes[2, col].plot(
                    time,
                    error,
                    label=f"simulation - {kin_ref_label}",
                    color="tab:red",
                    linewidth=1.3,
                )
                axes[2, col].axhline(0.0, color="0.3", linewidth=0.8, linestyle="--")
                axes[2, col].grid(True, alpha=0.25)
                axes[2, col].legend(loc="best", fontsize=8)

    finalize_time_axes(
        fig,
        axes,
        "Time Signals: Kinematic Reference vs Simulated Joint Angle",
        outer_subtitle,
        inner_subtitle,
    )
    save_figure(fig, out_dir, "06_time_joint_ref_sea_error.png")


def plot_figure_7(
    tables: Dict[str, Optional[StoTable]],
    mlp_context: MlpRolloutContext,
    out_dir: Path,
    missing: MissingReport,
    outer_subtitle: str = "",
    inner_subtitle: str = "",
) -> None:
    trace = load_policy_trace(mlp_context.trace_path, missing)
    fig, axes = plt.subplots(3, 2, figsize=(14, 10), sharex=False)
    row_labels = [
        "policy C2 and target [rad]",
        "sound leg raw [rad]",
        "shifted sound leg - policy C2 [rad]",
    ]
    for row, label in enumerate(row_labels):
        axes[row, 0].set_ylabel(label)

    reward_mode = mlp_context.reward_mode
    for col, side in enumerate(SIDES):
        key = side["key"]
        coord = side["coord"]
        axes[0, col].set_title(side["title"])
        policy = served_policy_reference_series(
            tables,
            coord,
            key,
            missing,
        )
        shifted_target = shifted_healthy_target_series(
            trace,
            coord,
            key,
            missing,
        )
        healthy_raw = find_series(
            tables["states"],
            sound_q_candidates(key),
            missing,
            "figure 7",
            key,
            "sound leg raw angle",
            "rollout_episode_states.sto",
        )
        healthy_raw = apply_joint_sign(healthy_raw, key)

        if policy is None:
            annotate_missing(axes[0, col], "policy C2 served reference")
        else:
            time, values, _ = policy
            axes[0, col].plot(
                time,
                values,
                label=f"policy C2 served ref ({reward_mode})",
                color="tab:blue",
                linewidth=1.3,
            )
            if shifted_target is not None:
                target_values = interpolate_like(shifted_target, time)
                axes[0, col].plot(
                    time,
                    target_values,
                    label="shifted sound leg target",
                    color=HEALTHY_COLOR,
                    linestyle=HEALTHY_LINESTYLE,
                    linewidth=1.2,
                )
            axes[0, col].grid(True, alpha=0.25)
            axes[0, col].legend(loc="best", fontsize=8)

        if healthy_raw is None:
            annotate_missing(axes[1, col], "sound leg raw angle")
        else:
            time, values, column = healthy_raw
            axes[1, col].plot(
                time,
                values,
                label=f"sound leg raw ({column})",
                color=HEALTHY_COLOR,
                linestyle=HEALTHY_LINESTYLE,
                linewidth=1.3,
            )
            axes[1, col].grid(True, alpha=0.25)
            axes[1, col].legend(loc="best", fontsize=8)

        if policy is None:
            annotate_missing(axes[2, col], "policy C2 served reference")
        elif shifted_target is None:
            annotate_missing(axes[2, col], "shifted sound leg target")
        else:
            time, policy_values, _ = policy
            target_values = interpolate_like(shifted_target, time)
            error = target_values - policy_values
            axes[2, col].plot(
                time,
                error,
                label=f"shifted sound leg - policy C2 ({reward_mode})",
                color="tab:red",
                linewidth=1.3,
            )
            axes[2, col].axhline(0.0, color="0.3", linewidth=0.8, linestyle="--")
            axes[2, col].grid(True, alpha=0.25)
            axes[2, col].legend(loc="best", fontsize=8)

    finalize_time_axes(
        fig,
        axes,
        f"MLP C2-Filtered Policy vs Shifted Sound Leg Target (reward_mode={reward_mode})",
        outer_subtitle,
        inner_subtitle,
    )
    save_figure(fig, out_dir, "07_mlp_policy_vs_sound_leg_error.png")


def parse_args() -> argparse.Namespace:
    default_cfg = SimulatorConfig()
    parser = argparse.ArgumentParser(
        description="Plot ankle/knee diagnostics from simulator .sto outputs."
    )
    parser.add_argument(
        "--mlp",
        action="store_true",
        help=(
            "Use the latest baseline-MLP rollout in "
            "Trajectory Generator/runs/rollout as input."
        ),
    )
    parser.add_argument("--results-dir", default=None, help="Directory containing .sto results.")
    parser.add_argument("--out-root", default="plot", help="Root directory for dated PNG folders.")
    parser.add_argument("--events", default=None, help="CSV with side,cycle_start,cycle_end gait-cycle events.")
    parser.add_argument(
        "--healthy-dir",
        default=None,
        help="Directory containing healthy *Kinematics_q.sto and *Actuation_force.sto.",
    )
    parser.add_argument(
        "--setup",
        default=None,
        help=(
            "Simulator setup XML to use for model and reference kinematics. "
            "Defaults to the last setup loaded by the simulator when available."
        ),
    )
    parser.add_argument(
        "--no-last-setup",
        action="store_true",
        help="Ignore .simulator_last_setup.json and use config.py/CLI paths only.",
    )
    parser.add_argument("--prefix", default=None, help="Result file prefix.")
    parser.add_argument(
        "--model-bundle",
        default=None,
        help="Override model bundle directory from config.py.",
    )
    parser.add_argument(
        "--model",
        default=None,
        help="Override model file (filename in bundle or explicit path).",
    )
    parser.add_argument(
        "--gait-side",
        default=default_cfg.plot_gait_side,
        choices=["left", "right", "all"],
        help="Gait event side to use for ankle/knee gait-cycle plots.",
    )
    parser.add_argument(
        "--reference",
        default=None,
        help="Reference IK .sto file for kinematic comparison; defaults to config kinematics_file.",
    )
    parser.add_argument(
        "--sea-outer-controller",
        choices=["pd", "pid", "cascade"],
        default=None,
        help="Prosthetic outer controller mode used for the run.",
    )
    parser.add_argument("--sea-kp-knee", type=float, default=None)
    parser.add_argument("--sea-kd-knee", type=float, default=None)
    parser.add_argument("--sea-ki-knee", type=float, default=None)
    parser.add_argument("--sea-integral-limit-knee", type=float, default=None)
    parser.add_argument("--sea-kp-ankle", type=float, default=None)
    parser.add_argument("--sea-kd-ankle", type=float, default=None)
    parser.add_argument("--sea-ki-ankle", type=float, default=None)
    parser.add_argument("--sea-integral-limit-ankle", type=float, default=None)
    parser.add_argument("--sea-integral-leak", type=float, default=None)
    parser.add_argument("--sea-cascade-kp-outer-knee", type=float, default=None)
    parser.add_argument("--sea-cascade-kp-inner-knee", type=float, default=None)
    parser.add_argument("--sea-cascade-ki-inner-knee", type=float, default=None)
    parser.add_argument("--sea-cascade-inner-i-torque-limit-knee", type=float, default=None)
    parser.add_argument("--sea-cascade-kp-outer-ankle", type=float, default=None)
    parser.add_argument("--sea-cascade-kp-inner-ankle", type=float, default=None)
    parser.add_argument("--sea-cascade-ki-inner-ankle", type=float, default=None)
    parser.add_argument("--sea-cascade-inner-i-torque-limit-ankle", type=float, default=None)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    mlp_context: Optional[MlpRolloutContext] = None
    if args.mlp:
        mlp_context = latest_mlp_rollout_context()
        print(f"MLP rollout loaded from: {mlp_context.rollout_dir}")
        print(f"  - reward_mode: {mlp_context.reward_mode}")

    if args.results_dir is not None:
        results_dir = resolve_project_path(args.results_dir)
    elif mlp_context is not None:
        results_dir = mlp_context.sim_outputs_dir
    else:
        results_dir = resolve_project_path("results")
    prefix = args.prefix or ("rollout_episode" if mlp_context is not None else "sim_output")
    out_root = resolve_project_path(args.out_root)
    if args.events:
        events_path = resolve_project_path(args.events)
    else:
        default_events = results_dir / f"{prefix}_gait_events.csv"
        events_path = default_events if default_events.is_file() else None

    missing = MissingReport()

    if not results_dir.is_dir():
        raise FileNotFoundError(f"Results directory not found: {results_dir}")

    cfg = SimulatorConfig()
    setup_path = resolve_project_path(args.setup) if args.setup else None
    if setup_path is None and not args.no_last_setup:
        setup_path = read_last_setup_path()
    if setup_path is not None:
        if not args.setup:
            recovered_setup = fallback_setup_path(setup_path)
            if recovered_setup is not None and recovered_setup != setup_path:
                print(f"Last setup path not found, using bundle setup: {recovered_setup}")
                setup_path = recovered_setup
        try:
            apply_setup_to_config(cfg, setup_path)
            print(f"Simulator setup loaded from: {setup_path}")
        except Exception as exc:
            if args.setup:
                raise
            missing.add(f"simulator setup: could not load last setup {setup_path}: {exc}")

    if args.model_bundle is not None:
        cfg.model_bundle_dir = args.model_bundle
        if args.model is None:
            cfg.model_file = ""
    if args.model is not None:
        cfg.model_file = normalize_cli_existing_path(args.model)
    if args.sea_outer_controller is not None:
        cfg.sea_outer_controller_mode = args.sea_outer_controller
    if args.sea_kp_knee is not None:
        cfg.sea_kp[cfg.pros_coords[0]] = args.sea_kp_knee
    if args.sea_kd_knee is not None:
        cfg.sea_kd[cfg.pros_coords[0]] = args.sea_kd_knee
    if args.sea_ki_knee is not None:
        cfg.sea_ki[cfg.pros_coords[0]] = args.sea_ki_knee
    if args.sea_integral_limit_knee is not None:
        cfg.sea_integral_limit[cfg.pros_coords[0]] = args.sea_integral_limit_knee
    if args.sea_kp_ankle is not None:
        cfg.sea_kp[cfg.pros_coords[1]] = args.sea_kp_ankle
    if args.sea_kd_ankle is not None:
        cfg.sea_kd[cfg.pros_coords[1]] = args.sea_kd_ankle
    if args.sea_ki_ankle is not None:
        cfg.sea_ki[cfg.pros_coords[1]] = args.sea_ki_ankle
    if args.sea_integral_limit_ankle is not None:
        cfg.sea_integral_limit[cfg.pros_coords[1]] = args.sea_integral_limit_ankle
    if args.sea_integral_leak is not None:
        cfg.sea_integral_leak_s_inv = args.sea_integral_leak
    if args.sea_cascade_kp_outer_knee is not None:
        cfg.sea_cascade_kp_outer[cfg.pros_coords[0]] = args.sea_cascade_kp_outer_knee
    if args.sea_cascade_kp_inner_knee is not None:
        cfg.sea_cascade_kp_inner[cfg.pros_coords[0]] = args.sea_cascade_kp_inner_knee
    if args.sea_cascade_ki_inner_knee is not None:
        cfg.sea_cascade_ki_inner[cfg.pros_coords[0]] = args.sea_cascade_ki_inner_knee
    if args.sea_cascade_inner_i_torque_limit_knee is not None:
        cfg.sea_cascade_inner_i_torque_limit[cfg.pros_coords[0]] = (
            args.sea_cascade_inner_i_torque_limit_knee
        )
    if args.sea_cascade_kp_outer_ankle is not None:
        cfg.sea_cascade_kp_outer[cfg.pros_coords[1]] = args.sea_cascade_kp_outer_ankle
    if args.sea_cascade_kp_inner_ankle is not None:
        cfg.sea_cascade_kp_inner[cfg.pros_coords[1]] = args.sea_cascade_kp_inner_ankle
    if args.sea_cascade_ki_inner_ankle is not None:
        cfg.sea_cascade_ki_inner[cfg.pros_coords[1]] = args.sea_cascade_ki_inner_ankle
    if args.sea_cascade_inner_i_torque_limit_ankle is not None:
        cfg.sea_cascade_inner_i_torque_limit[cfg.pros_coords[1]] = (
            args.sea_cascade_inner_i_torque_limit_ankle
        )

    healthy_dir = resolve_project_path(args.healthy_dir) if args.healthy_dir else default_healthy_dir(cfg)
    healthy = load_healthy_data(healthy_dir, missing)
    if healthy is not None:
        print(f"Healthy overlay loaded from: {healthy.directory}")
        for note in healthy.notes:
            print(f"  - {note}")

    out_dir = next_output_dir(out_root)
    tables = load_tables(results_dir, prefix)
    result_time_range = infer_result_time_range(tables)
    if result_time_range is not None:
        cfg.t_start, cfg.t_end = result_time_range
        print(f"Result time range: {cfg.t_start:.3f} - {cfg.t_end:.3f} s")
    events = load_events(events_path, missing)
    if events_path is not None:
        print(f"Gait events loaded from: {events_path}")
        print(f"  - {describe_events(events)}")
    sea_f_opt = load_sea_f_opt(cfg, missing)
    sea_params = load_sea_params(cfg)
    reference_path = (
        resolve_reference_path(args.reference, cfg)
        if args.reference is not None
        else resolve_simulator_paths(cfg).kinematics_path
    )
    print(f"Reference kinematics: {reference_path}")
    reference = load_reference_kinematics(args.reference, cfg, missing)
    outer_subtitle = outer_loop_subtitle(cfg)
    inner_subtitle = inner_loop_subtitle(sea_params, cfg)

    plot_figure_1(tables, out_dir, missing, outer_subtitle, inner_subtitle)
    plot_figure_2(tables, out_dir, missing, outer_subtitle, inner_subtitle)
    plot_figure_3(
        tables,
        events,
        args.gait_side,
        healthy,
        out_dir,
        missing,
        outer_subtitle,
        inner_subtitle,
    )
    plot_figure_4(
        tables,
        events,
        args.gait_side,
        healthy,
        out_dir,
        missing,
        outer_subtitle,
        inner_subtitle,
    )
    plot_figure_5(tables, sea_f_opt, out_dir, missing, outer_subtitle, inner_subtitle)
    plot_figure_6(tables, reference, out_dir, missing, outer_subtitle, inner_subtitle)
    if mlp_context is not None:
        plot_figure_7(
            tables,
            mlp_context,
            out_dir,
            missing,
            outer_subtitle,
            inner_subtitle,
        )

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
