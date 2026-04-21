"""
prosthesis_controller.py
========================
High-level (outer) controller for the two SEA actuators.

Architecture
------------
The SEA plugin implements a *low-level* PD torque loop in C++.  It accepts a
normalised command signal  u ∈ [−1, +1]  whose physical meaning is:

    τ_commanded = u · F_opt_sea   [N·m]

This module implements the *high-level* position-tracking loop that runs at
the Python level:

    e_q     = q_ref    − q_current
    e_qdot  = qdot_ref − qdot_current
    τ_cmd   = Kp · e_q + Kd · e_qdot
    u       = clip(τ_cmd / F_opt, −1, +1)

No inverse-dynamics feed-forward term is used in this controller: the SEA
command is generated from the prosthetic tracking controller alone.

The resulting u values are injected into the OpenSim control vector at the
indices assigned to SEA_knee and SEA_ankle before the model is realised to
acceleration.

Replacing this with a more sophisticated controller (impedance, torque-feed-
forward, etc.) only requires editing :meth:`compute`.
"""

from __future__ import annotations

from typing import Dict

import numpy as np
import opensim

from config import SimulatorConfig
from model_loader import SimulationContext


class ProsthesisController:
    """
    High-level position-tracking PD controller for both SEA actuators.

    Parameters
    ----------
    cfg : SimulatorConfig
        Provides SEA gain tables (sea_kp / sea_kd) and coordinate names.
    ctx : SimulationContext
        Provides control vector indices (sea_ctrl_idx) and the model reference.
    """

    def __init__(self, cfg: SimulatorConfig, ctx: SimulationContext) -> None:
        self._cfg = cfg
        self._ctx = ctx
        self._pros_coords = cfg.pros_coords     # ["pros_knee_angle", "pros_ankle_angle"]
        self._sea_names   = [cfg.sea_knee_name, cfg.sea_ankle_name]

        # Pre-fetch coordinate objects once to avoid repeated string lookups
        # inside the hot path.
        coord_set = ctx.model.getCoordinateSet()
        self._coords: Dict[str, opensim.Coordinate] = {
            name: coord_set.get(name) for name in self._pros_coords
        }

    # ─────────────────────────────────────────────────────────────────────────
    #  Public API
    # ─────────────────────────────────────────────────────────────────────────
    def compute(
        self,
        state: opensim.State,
        q_ref:    Dict[str, float],
        qdot_ref: Dict[str, float],
        controls: opensim.Vector,
    ) -> Dict[str, float]:
        """
        Compute and inject SEA control signals into *controls*.

        The controls Vector is modified **in-place** at the SEA indices.

        Parameters
        ----------
        state    : current OpenSim State (must be realised to Velocity)
        q_ref    : reference positions   {coord_name: value [rad]}
        qdot_ref : reference velocities  {coord_name: value [rad/s]}
        controls : model control Vector (modified in-place)

        Returns
        -------
        u_dict : {coord_name: u} for logging / debugging
        """
        u_dict: Dict[str, float] = {}
        for sea_name, coord_name in zip(self._sea_names, self._pros_coords):
            coord = self._coords[coord_name]

            # ── Read current state ──────────────────────────────────────────
            # These calls require state ≥ Stage::Velocity (guaranteed by caller)
            q_cur    = coord.getValue(state)       # [rad]
            qdot_cur = coord.getSpeedValue(state)  # [rad/s]

            # ── PD error ────────────────────────────────────────────────────
            e_q    = q_ref.get(coord_name,    0.0) - q_cur
            e_qdot = qdot_ref.get(coord_name, 0.0) - qdot_cur

            # ── Control law ─────────────────────────────────────────────────
            kp = self._cfg.sea_kp.get(coord_name, 5.0)
            kd = self._cfg.sea_kd.get(coord_name, 0.5)

            f_opt = self._ctx.sea_f_opt.get(sea_name, 1.0)
            outer_pd_cmd = kp * e_q + kd * e_qdot
            scale = self._select_feasibility_scale(
                state,
                sea_name,
                coord_name,
                outer_pd_cmd,
                f_opt,
            )
            tau_cmd = scale * outer_pd_cmd
            u_raw = tau_cmd / f_opt if f_opt > 1e-10 else 0.0
            u     = float(np.clip(u_raw, -1.0, 1.0))

            # ── Inject into control Vector ──────────────────────────────────
            # Set the control before any downstream Dynamics-stage plugin output
            # reads it through getControl(state).
            ctrl_idx = self._ctx.sea_ctrl_idx[sea_name]
            controls.set(ctrl_idx, u)

            u_dict[coord_name] = u
            u_dict[f"{coord_name}_feasibility_scale"] = scale

        return u_dict

    def _select_feasibility_scale(
        self,
        state: opensim.State,
        sea_name: str,
        coord_name: str,
        outer_pd_cmd: float,
        f_opt: float,
    ) -> float:
        """Return the largest PD scale predicted to keep the SEA unsaturated."""
        if not getattr(self._cfg, "enable_sea_feasibility_scaling", False):
            return 1.0

        props = self._ctx.sea_props.get(sea_name, {})
        K = float(props.get("K", 0.0))
        Kp_inner = float(props.get("Kp", 0.0))
        Kd_inner = float(props.get("Kd", 0.0))
        Bm = float(props.get("Bm", 0.0))
        impedance = bool(props.get("impedance", False))
        if f_opt <= 1e-10 or K <= 1e-10:
            return 1.0

        coord = self._coords[coord_name]
        theta_j = coord.getValue(state)
        omega_j = coord.getSpeedValue(state)
        sv = self._ctx.model.getStateVariableValues(state)
        ma_idx = self._ctx.sea_motor_angle_sv_idx.get(sea_name)
        ms_idx = self._ctx.sea_motor_speed_sv_idx.get(sea_name)
        if ma_idx is None:
            return 1.0
        theta_m = sv.get(ma_idx)
        omega_m = sv.get(ms_idx) if ms_idx is not None else 0.0
        tau_spring = K * (theta_m - theta_j)

        raw_scales = getattr(self._cfg, "sea_feasibility_scales", [1.0])
        scales = [
            float(value) for value in raw_scales
            if np.isfinite(float(value)) and 0.0 <= float(value) <= 1.0
        ]
        if not scales:
            scales = [1.0]

        tau_limit = float(getattr(
            self._cfg, "sea_feasibility_tau_input_limit", 450.0
        ))
        u_limit = float(getattr(self._cfg, "sea_feasibility_u_limit", 0.95))

        best_scale = scales[-1]
        best_violation = float("inf")
        for scale in scales:
            tau_ref = float(np.clip(
                (scale * outer_pd_cmd) / f_opt,
                -1.0,
                1.0,
            )) * f_opt
            if impedance:
                theta_m_ref = theta_j + tau_ref / K
                tau_ff_inner = tau_spring + Bm * omega_m
                tau_input_raw = (
                    tau_ff_inner
                    + Kp_inner * (theta_m_ref - theta_m)
                    + Kd_inner * (omega_j - omega_m)
                )
            else:
                tau_input_raw = (
                    Kp_inner * (tau_ref - tau_spring)
                    - Kd_inner * omega_m
                )
            u_abs = abs((scale * outer_pd_cmd) / f_opt)
            violation = max(
                0.0,
                abs(tau_input_raw) - tau_limit,
                u_abs - u_limit,
            )
            if violation < best_violation:
                best_violation = violation
                best_scale = scale
            if violation <= 0.0:
                return scale
        return best_scale
