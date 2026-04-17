"""
output.py
=========
Centralised I/O for simulation results.

- ``write_sto`` / ``read_sto``: OpenSim .sto file writer and parser.
- ``OutputRecorder``: manages pre-allocated NumPy buffers, per-step
  recording, and batch save of all enabled output files.

Every other module that needs to read or write .sto data imports from here.
"""

from __future__ import annotations

import os
import csv
import re
from typing import Dict, List, Optional, TYPE_CHECKING

import numpy as np

from config import SimulatorConfig

try:
    import opensim
except ModuleNotFoundError:
    opensim = None

if TYPE_CHECKING:
    from model_loader import SimulationContext


# ─────────────────────────────────────────────────────────────────────────────
#  .sto writer
# ─────────────────────────────────────────────────────────────────────────────
def write_sto(
    filepath:    str,
    header_name: str,
    time:        np.ndarray,
    col_names:   List[str],
    data:        np.ndarray,
    in_degrees:  bool = False,
) -> None:
    """
    Write a minimal OpenSim .sto / Storage file.

    Parameters
    ----------
    filepath    : output path
    header_name : value of the first header line (arbitrary label)
    time        : shape (N,)
    col_names   : length n_cols
    data        : shape (N, n_cols)
    in_degrees  : whether the file header should say inDegrees=yes
    """
    n_rows, n_cols = data.shape
    assert len(col_names) == n_cols, "col_names / data column count mismatch"
    assert len(time) == n_rows,      "time / data row count mismatch"

    deg_str = "yes" if in_degrees else "no"

    with open(filepath, "w") as fh:
        fh.write(f"{header_name}\n")
        fh.write("version=1\n")
        fh.write(f"nRows={n_rows}\n")
        fh.write(f"nColumns={n_cols + 1}\n")
        fh.write(f"inDegrees={deg_str}\n")
        fh.write("\n")
        fh.write("endheader\n")
        fh.write("time\t" + "\t".join(col_names) + "\n")
        for i in range(n_rows):
            row = f"{time[i]:.8f}\t" + "\t".join(
                f"{v:.8f}" for v in data[i]
            )
            fh.write(row + "\n")


# ─────────────────────────────────────────────────────────────────────────────
#  .sto reader
# ─────────────────────────────────────────────────────────────────────────────
def read_sto(path: str):
    """
    Parse an OpenSim .sto file (version=1, tab-separated).

    Returns
    -------
    times       : np.ndarray shape (n_frames,)
    col_names   : list[str]  (excluding 'time')
    data        : np.ndarray shape (n_frames, n_cols)
    in_degrees  : bool
    """
    with open(path) as f:
        in_degrees = False
        for line in f:
            stripped = line.strip()
            if stripped.lower().startswith("indegrees"):
                in_degrees = "yes" in stripped.lower()
            if stripped == "endheader":
                break

        header_line = f.readline().strip()
        col_names = header_line.split("\t")  # first is "time"

        rows = []
        for line in f:
            vals = line.strip().split("\t")
            if len(vals) == len(col_names):
                rows.append([float(v) for v in vals])

    arr = np.array(rows)
    times = arr[:, 0]
    data = arr[:, 1:]
    col_names = col_names[1:]  # drop "time"
    return times, col_names, data, in_degrees


# ─────────────────────────────────────────────────────────────────────────────
#  Output recorder
# ─────────────────────────────────────────────────────────────────────────────
def _read_storage_table(path: str):
    """Read whitespace-delimited OpenSim Storage/.mot data."""
    with open(path) as f:
        for line in f:
            if line.strip().lower() == "endheader":
                break

        header_line = ""
        for line in f:
            if line.strip():
                header_line = line.strip()
                break
        if not header_line:
            raise ValueError(f"No data header found in Storage file: {path}")

        col_names = re.split(r"\s+", header_line)
        rows = []
        for line in f:
            stripped = line.strip()
            if not stripped:
                continue
            vals = re.split(r"\s+", stripped)
            if len(vals) == len(col_names):
                rows.append([float(v) for v in vals])

    if not rows:
        raise ValueError(f"No numeric rows found in Storage file: {path}")

    arr = np.array(rows, dtype=float)
    return arr[:, 0], col_names[1:], arr[:, 1:]


def _cycles_from_vertical_grf(
    time: np.ndarray,
    vertical_grf: np.ndarray,
    threshold: float,
    t_start: float,
    t_end: float,
) -> List[tuple]:
    """Return complete heel-strike-to-heel-strike cycles from GRF crossings."""
    edges: List[float] = []
    for i in range(1, len(time)):
        was_contact = vertical_grf[i - 1] > threshold
        is_contact = vertical_grf[i] > threshold
        if was_contact or not is_contact:
            continue
        denom = vertical_grf[i] - vertical_grf[i - 1]
        frac = 0.0 if abs(denom) < 1e-12 else (threshold - vertical_grf[i - 1]) / denom
        edge_time = time[i - 1] + frac * (time[i] - time[i - 1])
        if t_start <= edge_time <= t_end:
            edges.append(float(edge_time))

    return [
        (edges[i], edges[i + 1])
        for i in range(len(edges) - 1)
        if t_start <= edges[i] < edges[i + 1] <= t_end
    ]


def _write_gait_events_csv(cfg: SimulatorConfig, ctx: "SimulationContext") -> None:
    """Write gait-cycle events inferred from GRF vertical threshold crossings."""
    out = cfg.output_dir
    pfx = cfg.output_prefix
    path = os.path.join(out, f"{pfx}_gait_events.csv")
    threshold = float(cfg.grf_contact_threshold_n)
    rows = []

    grf_file = getattr(ctx, "grf_data_file", "")
    grf_columns = getattr(ctx, "grf_vertical_force_columns", {})

    if grf_file and os.path.isfile(grf_file) and grf_columns:
        time, col_names, data = _read_storage_table(grf_file)
        col_idx = {name: i for i, name in enumerate(col_names)}
        for side in ("left", "right"):
            source_col = grf_columns.get(side)
            idx = col_idx.get(source_col) if source_col else None
            if idx is None:
                continue
            cycles = _cycles_from_vertical_grf(
                time,
                data[:, idx],
                threshold,
                cfg.t_start,
                cfg.t_end,
            )
            for start, end in cycles:
                rows.append(
                    {
                        "side": side,
                        "cycle_start": f"{start:.8f}",
                        "cycle_end": f"{end:.8f}",
                        "source_force": source_col,
                        "threshold_n": f"{threshold:.8f}",
                    }
                )

    with open(path, "w", newline="") as fh:
        writer = csv.DictWriter(
            fh,
            fieldnames=[
                "side",
                "cycle_start",
                "cycle_end",
                "source_force",
                "threshold_n",
            ],
        )
        writer.writeheader()
        writer.writerows(rows)

    counts = {side: sum(1 for row in rows if row["side"] == side) for side in ("left", "right")}
    print(
        f"  -> Gait events : {path} "
        f"(left={counts['left']}, right={counts['right']}, threshold={threshold:g} N)"
    )


class OutputRecorder:
    """
    Pre-allocates output buffers, records one row per simulation step,
    and writes all enabled .sto files at the end of the simulation.

    Parameters
    ----------
    cfg          : SimulatorConfig
    ctx          : SimulationContext
    sea_pros_map : list of (sea_name, coord_name) pairs
    sea_props    : dict  {sea_name: {K, Kp, Kd, Bm, F_opt, impedance, ...}}
    """

    def __init__(
        self,
        cfg: SimulatorConfig,
        ctx: SimulationContext,
        sea_pros_map: List[tuple],
        sea_props: Dict[str, dict],
    ) -> None:
        self._cfg = cfg
        self._ctx = ctx
        self._sea_pros_map = sea_pros_map
        self._sea_props = sea_props

        n_steps  = int((cfg.t_end - cfg.t_start) / cfg.dt) + 2
        n_coords = len(ctx.coord_names)
        n_sea    = len(sea_pros_map)
        n_res_all = len(getattr(ctx, "reserve_all_names", ctx.reserve_names))

        self._rec_time          = np.full(n_steps, np.nan)
        self._rec_q             = np.full((n_steps, n_coords),       np.nan)
        self._rec_qdot          = np.full((n_steps, n_coords),       np.nan)
        self._rec_qddot         = np.full((n_steps, n_coords),       np.nan)
        self._rec_activations   = np.full((n_steps, ctx.n_muscles),  np.nan)
        self._rec_u_res         = np.full((n_steps, ctx.n_reserves), np.nan)
        self._rec_reserve_controls = np.full((n_steps, n_res_all),   np.nan)
        self._rec_reserve_torques  = np.full((n_steps, n_res_all),   np.nan)
        self._rec_tau_bio       = np.full((n_steps, ctx.n_bio),      np.nan)
        self._rec_sea_controls  = np.full((n_steps, 2),              np.nan)
        self._rec_muscle_forces = np.full((n_steps, ctx.n_muscles),  np.nan)
        self._rec_sea_torques   = np.full((n_steps, n_sea * 2),      np.nan)
        self._rec_sea_states    = np.full((n_steps, n_sea * 2),      np.nan)
        self._rec_sea_derivatives = np.full((n_steps, n_sea * 2),    np.nan)
        self._rec_sea_diagnostics = np.full((n_steps, n_sea * 21),   np.nan)
        self._rec_power         = np.full((n_steps, n_sea * 2),      np.nan)
        self._rec_recruitment   = np.full((n_steps, 12),             np.nan)
        self._step_count        = 0

    @property
    def step_count(self) -> int:
        return self._step_count

    def record(
        self,
        t:       float,
        state:   opensim.State,
        a:       np.ndarray,
        u_res:   np.ndarray,
        tau_bio: np.ndarray,
        u_sea:   Dict[str, float],
        udot:    np.ndarray,
        controls: opensim.Vector,
        q_ref: Optional[Dict[str, float]] = None,
        qdot_ref: Optional[Dict[str, float]] = None,
        tau_pros_ff: Optional[Dict[str, float]] = None,
        sea_derivatives: Optional[np.ndarray] = None,
        sea_plugin_outputs: Optional[np.ndarray] = None,
        so_diagnostics: Optional[dict] = None,
    ) -> None:
        """Append one row to all output buffers."""
        ctx = self._ctx
        cfg = self._cfg
        k   = self._step_count

        self._rec_time[k] = t

        coord_set = ctx.model.getCoordinateSet()
        for i, name in enumerate(ctx.coord_names):
            coord = coord_set.get(name)
            self._rec_q[k, i]     = coord.getValue(state)
            self._rec_qdot[k, i]  = coord.getSpeedValue(state)
            self._rec_qddot[k, i] = udot[ctx.coord_mob_idx[name]]

        self._rec_activations[k]  = a
        self._rec_u_res[k]        = u_res
        self._rec_tau_bio[k]      = tau_bio
        self._rec_sea_controls[k, 0] = u_sea.get(cfg.pros_coords[0], 0.0)
        self._rec_sea_controls[k, 1] = u_sea.get(cfg.pros_coords[1], 0.0)

        reserve_all_names = getattr(ctx, "reserve_all_names", ctx.reserve_names)
        reserve_all_f_opt = getattr(ctx, "reserve_all_f_opt", ctx.reserve_f_opt)
        reserve_all_ctrl_idx = getattr(ctx, "reserve_all_ctrl_idx", ctx.reserve_ctrl_idx)
        for j, reserve_name in enumerate(reserve_all_names):
            ctrl_idx = reserve_all_ctrl_idx.get(reserve_name)
            if ctrl_idx is None or ctrl_idx < 0:
                continue
            ctrl = controls.get(ctrl_idx)
            self._rec_reserve_controls[k, j] = ctrl
            self._rec_reserve_torques[k, j] = ctrl * reserve_all_f_opt[j]

        # ── Muscle forces: F_i = a_i * F_max_i ─────────────────────────────
        self._rec_muscle_forces[k] = a * ctx.f_max

        # ── SEA torques: spring (downstream) + motor (upstream) ─────────────
        sv = ctx.model.getStateVariableValues(state)
        for i, (sea_name, coord_name) in enumerate(self._sea_pros_map):
            props = self._sea_props.get(sea_name)
            if props is None:
                continue
            K     = props["K"]
            Kp    = props["Kp"]
            Kd    = props["Kd"]
            Bm    = props["Bm"]
            Jm    = props.get("Jm", np.nan)
            F_opt = props["F_opt"]

            ma_idx = ctx.sea_motor_angle_sv_idx[sea_name]
            ms_idx = ctx.sea_motor_speed_sv_idx.get(sea_name)
            theta_m = sv.get(ma_idx)
            omega_m = sv.get(ms_idx) if ms_idx is not None else 0.0

            theta_j = coord_set.get(coord_name).getValue(state)
            omega_j = coord_set.get(coord_name).getSpeedValue(state)

            tau_spring = K * (theta_m - theta_j)

            u = u_sea.get(coord_name, 0.0)
            tau_ref = u * F_opt
            q_ref_value = (q_ref or {}).get(coord_name, theta_j)
            qdot_ref_value = (qdot_ref or {}).get(coord_name, omega_j)
            outer_kp = cfg.sea_kp.get(coord_name, 0.0)
            outer_kd = cfg.sea_kd.get(coord_name, 0.0)
            outer_pd_cmd = (
                outer_kp * (q_ref_value - theta_j)
                + outer_kd * (qdot_ref_value - omega_j)
            )
            tau_ff_cmd = (tau_pros_ff or {}).get(coord_name, np.nan)
            tau_cmd_raw = (
                tau_ff_cmd + outer_pd_cmd
                if np.isfinite(tau_ff_cmd)
                else np.nan
            )
            tau_ref_minus_tau_cmd = (
                tau_ref - tau_cmd_raw
                if np.isfinite(tau_cmd_raw)
                else np.nan
            )
            if props["impedance"]:
                theta_m_ref = theta_j + tau_ref / K
                omega_m_ref = omega_j
                tau_ff = tau_spring + Bm * omega_m
                inner_prop_term = Kp * (theta_m_ref - theta_m)
                inner_damp_term = Kd * (omega_m_ref - omega_m)
                tau_input_raw = (tau_ff
                                 + inner_prop_term
                                 + inner_damp_term)
            else:
                inner_prop_term = Kp * (tau_ref - tau_spring)
                inner_damp_term = -Kd * omega_m
                tau_input_raw = inner_prop_term + inner_damp_term

            tau_input_python = max(-500.0, min(500.0, tau_input_raw))
            tau_input_plugin = np.nan
            motor_angle_dot_plugin = np.nan
            motor_speed_dot_plugin = np.nan
            if sea_plugin_outputs is not None and len(sea_plugin_outputs) >= (i + 1) * 3:
                base = i * 3
                tau_input_plugin = float(sea_plugin_outputs[base])
                motor_angle_dot_plugin = float(sea_plugin_outputs[base + 1])
                motor_speed_dot_plugin = float(sea_plugin_outputs[base + 2])

            tau_input = (
                tau_input_plugin
                if np.isfinite(tau_input_plugin)
                else tau_input_python
            )
            tau_input_diff = (
                tau_input_plugin - tau_input_python
                if np.isfinite(tau_input_plugin)
                else np.nan
            )
            tau_error = tau_ref - tau_spring
            is_tau_input_saturated = (
                1.0 if abs(tau_input) >= 500.0 - 1e-9 else 0.0
            )
            motor_accel_numerator = tau_input - tau_spring - Bm * omega_m

            self._rec_sea_torques[k, i * 2]     = tau_spring
            self._rec_sea_torques[k, i * 2 + 1] = tau_input
            self._rec_sea_states[k, i * 2]       = theta_m
            self._rec_sea_states[k, i * 2 + 1]   = omega_m
            self._rec_power[k, i * 2]            = tau_spring * omega_j
            self._rec_power[k, i * 2 + 1]        = tau_input * omega_m
            diag_base = i * 21
            self._rec_sea_diagnostics[k, diag_base:diag_base + 21] = np.array([
                tau_ref,
                tau_ff_cmd,
                outer_pd_cmd,
                tau_cmd_raw,
                tau_ref_minus_tau_cmd,
                tau_spring,
                tau_error,
                tau_input_raw,
                tau_input_python,
                tau_input_plugin,
                tau_input_diff,
                is_tau_input_saturated,
                inner_prop_term,
                inner_damp_term,
                omega_m,
                motor_angle_dot_plugin,
                motor_speed_dot_plugin,
                motor_accel_numerator,
                Jm,
                Bm,
                K,
            ])

        if sea_derivatives is not None:
            self._rec_sea_derivatives[k, :len(sea_derivatives)] = sea_derivatives

        # ── Recruitment diagnostics ─────────────────────────────────────────
        diag = so_diagnostics
        if diag:
            self._rec_recruitment[k] = np.array([
                diag.get("tau_target_norm", np.nan),
                diag.get("tau_muscle_norm", np.nan),
                diag.get("tau_reserve_norm", np.nan),
                diag.get("muscle_share", np.nan),
                diag.get("muscle_capable_share", np.nan),
                diag.get("muscle_capable_reserve_norm", np.nan),
                diag.get("unactuated_reserve_norm", np.nan),
                diag.get("reserve_control_norm", np.nan),
                diag.get("activation_mean", np.nan),
                diag.get("activation_nonzero_fraction", np.nan),
                diag.get("residual_norm", np.nan),
                diag.get("equilibrium_failures", np.nan),
            ])
            interval = self._cfg.recruitment_diagnostics_interval
            if interval > 0 and k % interval == 0:
                print(
                    f"[Recruit t={t:.3f}] "
                    f"muscle_share={diag.get('muscle_share', np.nan):.3f}  "
                    f"capable_share={diag.get('muscle_capable_share', np.nan):.3f}  "
                    f"|tau_m|={diag.get('tau_muscle_norm', np.nan):.2f}  "
                    f"|tau_res|={diag.get('tau_reserve_norm', np.nan):.2f}  "
                    f"act>thr={diag.get('activation_nonzero_fraction', np.nan):.2f}  "
                    f"|u_res|={diag.get('reserve_control_norm', np.nan):.3f}",
                    flush=True,
                )

        self._step_count += 1

    def save_results(self) -> None:
        """Write all enabled output files as OpenSim-compatible .sto files."""
        cfg = self._cfg
        ctx = self._ctx
        k   = self._step_count
        out = cfg.output_dir
        pfx = cfg.output_prefix

        if cfg.save_activations:
            path = os.path.join(out, f"{pfx}_activations.sto")
            write_sto(
                path, "Activations",
                self._rec_time[:k], ctx.muscle_names,
                self._rec_activations[:k],
            )
            print(f"  -> Activations : {path}")

        if cfg.save_sea_controls:
            path = os.path.join(out, f"{pfx}_sea_controls.sto")
            write_sto(
                path, "SEAControls",
                self._rec_time[:k], cfg.pros_coords,
                self._rec_sea_controls[:k],
            )
            print(f"  -> SEA ctrl    : {path}")

        reserve_all_names = getattr(ctx, "reserve_all_names", ctx.reserve_names)
        reserve_all_coords = getattr(ctx, "reserve_all_coord_names", reserve_all_names)
        reserve_control_cols = [
            f"{coord_name}_reserve_control"
            for coord_name in reserve_all_coords
        ]
        reserve_torque_cols = [
            f"{coord_name}_reserve_torque"
            for coord_name in reserve_all_coords
        ]

        if cfg.save_reserve_controls and reserve_control_cols:
            path = os.path.join(out, f"{pfx}_reserve_controls.sto")
            write_sto(
                path, "ReserveControls",
                self._rec_time[:k], reserve_control_cols,
                self._rec_reserve_controls[:k],
            )
            print(f"  -> Reserve ctrl: {path}")

        if cfg.save_reserve_torques and reserve_torque_cols:
            path = os.path.join(out, f"{pfx}_reserve_torques.sto")
            write_sto(
                path, "ReserveTorques",
                self._rec_time[:k], reserve_torque_cols,
                self._rec_reserve_torques[:k],
            )
            print(f"  -> Reserve tau : {path}")

        if cfg.save_kinematics:
            path = os.path.join(out, f"{pfx}_kinematics.sto")
            write_sto(
                path, "Kinematics_q",
                self._rec_time[:k], ctx.coord_names,
                self._rec_q[:k],
                in_degrees=False,
            )
            print(f"  -> Kinematics  : {path}")

        if cfg.save_tau_bio:
            path = os.path.join(out, f"{pfx}_tau_bio.sto")
            write_sto(
                path, "GeneralisedForces_bio",
                self._rec_time[:k], ctx.bio_coord_names,
                self._rec_tau_bio[:k],
            )
            print(f"  -> Tau_bio     : {path}")

        if cfg.save_muscle_forces:
            path = os.path.join(out, f"{pfx}_muscle_forces.sto")
            write_sto(
                path, "MuscleForces",
                self._rec_time[:k], ctx.muscle_names,
                self._rec_muscle_forces[:k],
            )
            print(f"  -> Muscle F    : {path}")

        if cfg.save_sea_torques:
            sea_col_names = []
            for sea_name, coord_name in self._sea_pros_map:
                sea_col_names.append(f"{sea_name}_tau_spring")
                sea_col_names.append(f"{sea_name}_tau_motor")
            path = os.path.join(out, f"{pfx}_sea_torques.sto")
            write_sto(
                path, "SEATorques",
                self._rec_time[:k], sea_col_names,
                self._rec_sea_torques[:k],
            )
            print(f"  -> SEA torques : {path}")

        if cfg.save_sea_states:
            sea_state_cols = []
            for sea_name, coord_name in self._sea_pros_map:
                sea_state_cols.append(f"{sea_name}_motor_angle")
                sea_state_cols.append(f"{sea_name}_motor_speed")
            path = os.path.join(out, f"{pfx}_sea_states.sto")
            write_sto(
                path, "SEAStates",
                self._rec_time[:k], sea_state_cols,
                self._rec_sea_states[:k],
                in_degrees=False,
            )
            print(f"  -> SEA states  : {path}")

        if getattr(cfg, "save_sea_derivatives", False):
            sea_derivative_cols = []
            for sea_name, coord_name in self._sea_pros_map:
                sea_derivative_cols.append(f"{sea_name}_motor_angle_dot")
                sea_derivative_cols.append(f"{sea_name}_motor_speed_dot")
            path = os.path.join(out, f"{pfx}_sea_derivatives.sto")
            write_sto(
                path, "SEADerivatives",
                self._rec_time[:k], sea_derivative_cols,
                self._rec_sea_derivatives[:k],
                in_degrees=False,
            )
            print(f"  -> SEA derivs  : {path}")

        if getattr(cfg, "save_sea_diagnostics", False):
            sea_diagnostic_cols = []
            for sea_name, coord_name in self._sea_pros_map:
                sea_diagnostic_cols.extend([
                    f"{sea_name}_tau_ref",
                    f"{sea_name}_tau_ff_cmd",
                    f"{sea_name}_outer_pd_cmd",
                    f"{sea_name}_tau_cmd_raw",
                    f"{sea_name}_tau_ref_minus_tau_cmd_raw",
                    f"{sea_name}_tau_spring_state",
                    f"{sea_name}_tau_error",
                    f"{sea_name}_tau_input_raw",
                    f"{sea_name}_tau_input_python",
                    f"{sea_name}_tau_input_plugin",
                    f"{sea_name}_tau_input_plugin_minus_python",
                    f"{sea_name}_tau_input_saturated",
                    f"{sea_name}_inner_prop_term",
                    f"{sea_name}_inner_damp_term",
                    f"{sea_name}_motor_speed",
                    f"{sea_name}_motor_angle_dot_plugin",
                    f"{sea_name}_motor_speed_dot_plugin",
                    f"{sea_name}_motor_accel_numerator",
                    f"{sea_name}_motor_inertia",
                    f"{sea_name}_motor_damping",
                    f"{sea_name}_spring_stiffness",
                ])
            path = os.path.join(out, f"{pfx}_sea_diagnostics.sto")
            write_sto(
                path, "SEADiagnostics",
                self._rec_time[:k], sea_diagnostic_cols,
                self._rec_sea_diagnostics[:k],
                in_degrees=False,
            )
            print(f"  -> SEA diag    : {path}")

        if cfg.save_power:
            power_cols = []
            for sea_name, coord_name in self._sea_pros_map:
                power_cols.append(f"{sea_name}_joint_power")
                power_cols.append(f"{sea_name}_motor_power")
            path = os.path.join(out, f"{pfx}_power.sto")
            write_sto(
                path, "SEAPower",
                self._rec_time[:k], power_cols,
                self._rec_power[:k],
            )
            print(f"  -> SEA power   : {path}")

        if cfg.save_states:
            state_col_names = []
            state_data = np.empty((k, len(ctx.coord_names) * 3))
            for i, name in enumerate(ctx.coord_names):
                state_col_names.extend([
                    f"{name}_q", f"{name}_qdot", f"{name}_qddot",
                ])
                state_data[:, i * 3]     = self._rec_q[:k, i]
                state_data[:, i * 3 + 1] = self._rec_qdot[:k, i]
                state_data[:, i * 3 + 2] = self._rec_qddot[:k, i]
            path = os.path.join(out, f"{pfx}_states.sto")
            write_sto(
                path, "CoordinateStates",
                self._rec_time[:k], state_col_names,
                state_data,
                in_degrees=False,
            )
            print(f"  -> States      : {path}")

        if cfg.save_recruitment_diagnostics:
            path = os.path.join(out, f"{pfx}_recruitment.sto")
            write_sto(
                path, "RecruitmentDiagnostics",
                self._rec_time[:k],
                [
                    "tau_target_norm",
                    "tau_muscle_norm",
                    "tau_reserve_norm",
                    "muscle_share",
                    "muscle_capable_share",
                    "muscle_capable_reserve_norm",
                    "unactuated_reserve_norm",
                    "reserve_control_norm",
                    "activation_mean",
                    "activation_nonzero_fraction",
                    "residual_norm",
                    "equilibrium_failures",
                ],
                self._rec_recruitment[:k],
            )
            print(f"  -> Recruitment: {path}")

        if cfg.save_gait_events:
            _write_gait_events_csv(cfg, ctx)

        print(f"\n[Runner] All results saved to: {os.path.abspath(out)}")
