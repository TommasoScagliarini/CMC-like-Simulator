"""
outer_loop.py
=============
Outer kinematic tracking loop (biological joints only).

Role in the pipeline
--------------------
This is the first of the two biological control layers, analogous to the
"tracking controller" in CMC:

    q_ddot_des[i] = q_ddot_ref[i]
                    + Kp[i] · (q_ref[i]  − q_current[i])
                    + Kd[i] · (qdot_ref[i] − qdot_current[i])

The desired accelerations are later fed to the InverseDynamics module to
compute the generalised forces that muscles and reserve actuators must produce.

Coordinate scope
----------------
Only *biological* coordinates (those NOT in cfg.pros_coords) are processed.
Prosthetic coordinates are handled exclusively by ProsthesisController.

Gain selection
--------------
Per-coordinate overrides in cfg.tracking_kp / tracking_kd take precedence
over the global defaults (cfg.default_tracking_kp / kd).  Pelvis translations
should be set to 0 because no net muscle force can drive them; residual errors
are absorbed by the reserve actuators in the SO step.
"""

from __future__ import annotations

from typing import Dict

import opensim

from config import SimulatorConfig
from model_loader import SimulationContext


class OuterLoop:
    """
    Biological kinematic tracking PD controller.

    Parameters
    ----------
    cfg : SimulatorConfig
    ctx : SimulationContext
    """

    def __init__(self, cfg: SimulatorConfig, ctx: SimulationContext) -> None:
        self._cfg = cfg
        self._ctx = ctx

        # Pre-fetch coordinate objects and gains for bio coords
        coord_set = ctx.model.getCoordinateSet()
        self._coords: Dict[str, opensim.Coordinate] = {
            name: coord_set.get(name) for name in ctx.bio_coord_names
        }

        # Build per-coordinate gain lookups (override > default)
        self._kp: Dict[str, float] = {
            name: cfg.tracking_kp.get(name, cfg.default_tracking_kp)
            for name in ctx.bio_coord_names
        }
        self._kd: Dict[str, float] = {
            name: cfg.tracking_kd.get(name, cfg.default_tracking_kd)
            for name in ctx.bio_coord_names
        }

    # ─────────────────────────────────────────────────────────────────────────
    #  Public API
    # ─────────────────────────────────────────────────────────────────────────
    def compute_desired_accelerations(
        self,
        state:    opensim.State,
        q_ref:    Dict[str, float],
        qdot_ref: Dict[str, float],
        qddot_ref: Dict[str, float],
    ) -> Dict[str, float]:
        """
        Compute q̈_des for all biological coordinates.

        Parameters
        ----------
        state     : current OpenSim State (must be ≥ Stage::Velocity)
        q_ref     : reference positions   {coord → [rad] or [m]}
        qdot_ref  : reference velocities  {coord → [rad/s] or [m/s]}
        qddot_ref : reference accelerations from spline  {coord → [rad/s²] …}

        Returns
        -------
        qddot_des : {coord_name → desired acceleration}
        """
        qddot_des: Dict[str, float] = {}

        for name, coord in self._coords.items():
            # ── Current state (Stage::Velocity already ensured by caller) ───
            q_cur    = coord.getValue(state)       # [rad] or [m]
            qdot_cur = coord.getSpeedValue(state)  # [rad/s] or [m/s]

            # ── Tracking errors ──────────────────────────────────────────────
            e_q    = q_ref.get(name,    q_cur)    - q_cur
            e_qdot = qdot_ref.get(name, qdot_cur) - qdot_cur

            # ── PD feed-forward control law ──────────────────────────────────
            # The feed-forward term (qddot_ref) handles nominal motion.
            # PD terms correct deviations from the reference trajectory.
            qddot_des[name] = (
                qddot_ref.get(name, 0.0)
                + self._kp[name] * e_q
                + self._kd[name] * e_qdot
            )

        return qddot_des
