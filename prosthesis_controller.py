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
    τ_pd    = Kp · e_q + Kd · e_qdot
    τ_cmd   = τ_ff + τ_pd
    u       = clip(τ_cmd / F_opt, −1, +1)

If a feed-forward torque is supplied, it is added in physical torque units
before the final normalisation by the SEA optimal force.  This matches the
plugin source, where the low-level torque reference is ``tau_ref = u * F_opt``.

The resulting u values are injected into the OpenSim control vector at the
indices assigned to SEA_knee and SEA_ankle before the model is realised to
acceleration.

Replacing this with a more sophisticated controller (impedance, torque-feed-
forward, etc.) only requires editing :meth:`compute`.
"""

from __future__ import annotations

from typing import Dict, Optional

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
        tau_ff:   Optional[Dict[str, float]] = None,
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
        tau_ff   : optional feed-forward torque {coord_name: N*m}

        Returns
        -------
        u_dict : {coord_name: u} for logging / debugging
        """
        u_dict: Dict[str, float] = {}
        tau_ff = tau_ff or {}

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
            #tau_cmd = tau_ff.get(coord_name, 0.0) + kp * e_q + kd * e_qdot
            tau_cmd = kp * e_q + kd * e_qdot
            u_raw = tau_cmd / f_opt if f_opt > 1e-10 else 0.0
            u     = float(np.clip(u_raw, -1.0, 1.0))

            # ── Inject into control Vector ──────────────────────────────────
            # CRITICAL ORDER: controls must be set BEFORE model.realizeAcceleration(state)
            # is called.  The SEA plugin reads its control signal via getControl(state)
            # inside computeActuation(), which is invoked during Stage::Dynamics.
            ctrl_idx = self._ctx.sea_ctrl_idx[sea_name]
            controls.set(ctrl_idx, u)

            u_dict[coord_name] = u

        return u_dict
