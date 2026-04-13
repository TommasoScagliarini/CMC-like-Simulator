"""
inverse_dynamics.py
===================
Partial inverse-dynamics computation for biological AND prosthetic DOFs.

Method: "zero-actuator residual + mass-matrix projection"
-----------------------------------------------------------
At each frame we want the vector of generalised forces that actuators must
produce to achieve the desired accelerations q̈_des.

    Step 1 – zero ALL actuator contributions:
             - Zero all controls (muscles, reserves, SEA)
             - Zero SEA spring forces (set motor_angle = theta_joint)
             Compute q̈₀ via realizeDynamics + InverseDynamicsSolver.

    Step 2 – acceleration deficit (ALL DOFs):
        Δq̈[i]  =  q̈_des[i] − q̈₀[i]

    Step 3 – project through mass matrix:
        τ  =  M · Δq̈

    Step 4 – split:
        τ_bio  = τ[bio_indices]    → muscles + reserves (QP)
        τ_pros = τ[pros_indices]   → SEA actuators

Why not realizeAcceleration?
----------------------------
realizeAcceleration triggers computeStateVariableDerivatives() in the
SEA plugin, which crashes.  The bypass (realizeDynamics + ID solver +
multiplyByM) avoids this entirely.
"""

from __future__ import annotations

from typing import Dict, List, Tuple

import numpy as np
import opensim

from config import SimulatorConfig
from model_loader import SimulationContext


# ─────────────────────────────────────────────────────────────────────────────
#  Module-level helpers (shared with simulation_runner)
# ─────────────────────────────────────────────────────────────────────────────
def build_mass_matrix(
    matter: opensim.SimbodyMatterSubsystem,
    state:  opensim.State,
    n_mob:  int,
    e_vec:  opensim.Vector,
    Me_vec: opensim.Vector,
) -> np.ndarray:
    """
    Build the full mass matrix M as a numpy array.

    Uses n_mob calls to matter.multiplyByM (one per column).
    Requires state ≥ Stage::Position.
    """
    M = np.zeros((n_mob, n_mob))
    for j in range(n_mob):
        for k in range(n_mob):
            e_vec.set(k, 0.0)
        e_vec.set(j, 1.0)
        matter.multiplyByM(state, e_vec, Me_vec)
        for i in range(n_mob):
            M[i, j] = Me_vec.get(i)
    return M


def compute_udot_bypass(
    matter: opensim.SimbodyMatterSubsystem,
    model:  opensim.Model,
    state:  opensim.State,
    n_mob:  int,
    e_vec:  opensim.Vector,
    Me_vec: opensim.Vector,
) -> np.ndarray:
    """
    Compute generalised accelerations q̈ WITHOUT realizeAcceleration.

    Steps:
      1. realizeDynamics  — computes all forces, no zdot
      2. ID_solver.solve(state, q̈=0) → residual
      3. Build M via multiplyByM
      4. q̈ = solve(M, −residual)
    """
    model.realizeDynamics(state)

    id_solver = opensim.InverseDynamicsSolver(model)
    zero_udot = opensim.Vector(n_mob, 0.0)
    residual  = id_solver.solve(state, zero_udot)

    M = build_mass_matrix(matter, state, n_mob, e_vec, Me_vec)

    neg_res = np.array([-residual.get(i) for i in range(n_mob)])
    udot = np.linalg.solve(M, neg_res)
    return udot


class InverseDynamicsComputer:
    """
    Frame-by-frame ID using the zero-actuator + M·Δq̈ approach.

    Computes required forces for BOTH biological and prosthetic DOFs.
    """

    def __init__(self, cfg: SimulatorConfig, ctx: SimulationContext) -> None:
        self._ctx = ctx
        self._cfg = cfg

        self._n_mob = ctx.n_mob
        self._n_bio = ctx.n_bio

        # Indices of bio_coords within the full mobility vector
        self._bio_mob_indices = np.array([
            ctx.coord_mob_idx[name] for name in ctx.bio_coord_names
        ], dtype=int)

        # Indices of pros_coords within the full mobility vector
        self._pros_mob_indices = ctx.pros_mob_indices

        # Simbody matter subsystem
        self._matter = ctx.model.getMatterSubsystem()

        # Reusable scratch vectors for build_mass_matrix
        self._e_vec  = opensim.Vector(self._n_mob, 0.0)
        self._Me_vec = opensim.Vector(self._n_mob, 0.0)

        # SEA coordinate names (for zeroing springs)
        self._sea_names = [cfg.sea_knee_name, cfg.sea_ankle_name]
        self._pros_coords = cfg.pros_coords

    # ─────────────────────────────────────────────────────────────────────────
    #  Public API
    # ─────────────────────────────────────────────────────────────────────────
    def compute_tau(
        self,
        state:          opensim.State,
        controls:       opensim.Vector,
        qddot_des:      Dict[str, float],
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Compute the generalised forces for bio DOFs (muscles + reserves)
        and pros DOFs (SEA actuators).

        Parameters
        ----------
        state          : current OpenSim State ≥ Stage::Velocity.
        controls       : full control Vector (temporarily zeroed, restored on exit).
        qddot_des      : desired accelerations for ALL coords {name → rad/s²}

        Returns
        -------
        tau_bio  : np.ndarray shape (n_bio,)
        tau_pros : np.ndarray shape (n_pros,)
        """
        model = self._ctx.model
        ctx   = self._ctx
        n_mob = self._n_mob

        # ── Step 1a: save and zero ALL controls ──────────────────────────────
        n_ctrl = controls.size()
        saved_ctrl = [controls.get(i) for i in range(n_ctrl)]
        for i in range(n_ctrl):
            controls.set(i, 0.0)
        model.setControls(state, controls)

        # ── Step 1b: zero SEA springs (motor_angle = theta_joint) ────────────
        #    In non-impedance mode, computeActuation = K*(theta_m - theta_j).
        #    With u=0 but theta_m ≠ theta_j, the spring still applies force.
        #    We must zero this to get a true "zero-actuator" baseline.
        sv = model.getStateVariableValues(state)
        saved_motor = {}
        coord_set = model.getCoordinateSet()
        for sea_name, coord_name in zip(self._sea_names, self._pros_coords):
            ma_idx = ctx.sea_motor_angle_sv_idx.get(sea_name)
            ms_idx = ctx.sea_motor_speed_sv_idx.get(sea_name)
            if ma_idx is not None:
                saved_motor[sea_name] = (
                    sv.get(ma_idx),
                    sv.get(ms_idx) if ms_idx is not None else 0.0,
                )
                theta_j = coord_set.get(coord_name).getValue(state)
                sv.set(ma_idx, theta_j)      # zero spring deflection
                if ms_idx is not None:
                    sv.set(ms_idx, 0.0)       # zero motor speed
        model.setStateVariableValues(state, sv)

        # ── Step 2: compute q̈₀ (no realizeAcceleration) ─────────────────────
        qdot0 = compute_udot_bypass(
            self._matter, model, state, n_mob,
            self._e_vec, self._Me_vec,
        )

        # ── Step 3: restore controls and SEA motor state ─────────────────────
        for i in range(n_ctrl):
            controls.set(i, saved_ctrl[i])

        sv = model.getStateVariableValues(state)
        for sea_name, (saved_ma, saved_ms) in saved_motor.items():
            ma_idx = ctx.sea_motor_angle_sv_idx[sea_name]
            sv.set(ma_idx, saved_ma)
            ms_idx = ctx.sea_motor_speed_sv_idx.get(sea_name)
            if ms_idx is not None:
                sv.set(ms_idx, saved_ms)
        model.setStateVariableValues(state, sv)

        # ── Step 4: Δq̈ for ALL DOFs ─────────────────────────────────────────
        delta_udot = np.zeros(n_mob)
        for name in ctx.coord_names:
            idx = ctx.coord_mob_idx[name]
            delta_udot[idx] = qddot_des.get(name, 0.0) - qdot0[idx]

        # ── Step 5: τ = M · Δq̈ ──────────────────────────────────────────────
        # Re-realize after setStateVariableValues invalidated the cache
        model.realizePosition(state)
        M = build_mass_matrix(
            self._matter, state, n_mob,
            self._e_vec, self._Me_vec,
        )
        tau_full = M @ delta_udot

        # ── Step 6: split bio / pros ─────────────────────────────────────────
        tau_bio  = tau_full[self._bio_mob_indices]
        tau_pros = tau_full[self._pros_mob_indices]

        return tau_bio, tau_pros
