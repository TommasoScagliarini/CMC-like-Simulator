"""
inverse_dynamics.py
===================
Partial inverse-dynamics computation for the biological degrees of freedom.

Method: "zero-actuator residual + mass-matrix projection"
-----------------------------------------------------------
At each frame we want the vector of generalised forces  τ_bio  that muscles
and reserve actuators must produce to achieve the desired accelerations
q̈_des (bio DOFs).  The derivation is:

    Equation of motion (full system):
        M(q) · q̈  =  τ_muscles + τ_SEA + τ_GRF + τ_gravity + τ_passive

    Step 1 – zero ALL actuator controls (muscles, reserves, SEA).
             Compute q̈₀ via realizeDynamics + InverseDynamicsSolver.

    Step 2 – acceleration deficit (bio DOFs only, prosthetic DOFs set to 0):
        Δq̈[i]  =  q̈_des[i] − q̈₀[i]    for i ∈ bio_coords
        Δq̈[j]  =  0                      for j ∈ pros_coords

    Step 3 – project through mass matrix:
        τ_bio[i]  =  (M · Δq̈)[i]        for i ∈ bio_coords

Why not realizeAcceleration?
----------------------------
realizeAcceleration computes BOTH q̈ AND zdot (auxiliary state variable
derivatives).  The SEA BlackBox plugin's computeStateVariableDerivatives()
crashes when its internal state variables are in an unexpected configuration.

Bypass:  realizeDynamics (forces only, no zdot)
       + InverseDynamicsSolver.solve (uses calcResidualForce, no zdot)
       + multiplyByM (pure mass-matrix algebra, no zdot)

All three are safe and produce identical q̈ values.
"""

from __future__ import annotations

from typing import Dict, List

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
    For gait2392 (n_mob=21) this is negligible (~μs per call).
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

    The state must already have controls set via model.setControls().

    Steps:
      1. realizeDynamics  — computes all forces, no zdot
      2. ID_solver.solve(state, q̈=0) → residual = C + G − f_applied
      3. Build M via multiplyByM
      4. q̈ = solve(M, −residual)

    Returns
    -------
    udot : np.ndarray shape (n_mob,)
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
    Frame-by-frame partial ID using the zero-actuator + M·Δq̈ approach.

    Parameters
    ----------
    cfg : SimulatorConfig
    ctx : SimulationContext
    """

    def __init__(self, cfg: SimulatorConfig, ctx: SimulationContext) -> None:
        self._ctx = ctx
        self._cfg = cfg

        self._n_mob = ctx.n_mob
        self._n_bio = ctx.n_bio

        # Indices of bio_coords within the full mobility vector (numpy for fancy indexing)
        self._bio_mob_indices = np.array([
            ctx.coord_mob_idx[name] for name in ctx.bio_coord_names
        ], dtype=int)

        # Simbody matter subsystem — used for multiplyByM
        self._matter = ctx.model.getMatterSubsystem()

        # Reusable scratch vectors for build_mass_matrix
        self._e_vec  = opensim.Vector(self._n_mob, 0.0)
        self._Me_vec = opensim.Vector(self._n_mob, 0.0)

    # ─────────────────────────────────────────────────────────────────────────
    #  Public API
    # ─────────────────────────────────────────────────────────────────────────
    def compute_tau_bio(
        self,
        state:          opensim.State,
        controls:       opensim.Vector,
        qddot_des_bio:  Dict[str, float],
    ) -> np.ndarray:
        """
        Compute the generalised forces that muscles + reserves must produce.

        Parameters
        ----------
        state          : current OpenSim State; must be ≥ Stage::Velocity.
                         On exit: Stage::Dynamics (never Acceleration).
        controls       : full control Vector.  All entries are temporarily
                         zeroed; original values are restored on exit.
        qddot_des_bio  : desired accelerations for bio coords {name → rad/s²}

        Returns
        -------
        tau_bio : np.ndarray shape (n_bio,)
        """
        model = self._ctx.model
        n_mob = self._n_mob

        # ── Step 1: save and zero ALL controls ───────────────────────────────
        n_ctrl = controls.size()
        saved = [controls.get(i) for i in range(n_ctrl)]
        for i in range(n_ctrl):
            controls.set(i, 0.0)
        model.setControls(state, controls)

        # ── Step 2: compute q̈₀ (no realizeAcceleration) ─────────────────────
        qdot0 = compute_udot_bypass(
            self._matter, model, state, n_mob,
            self._e_vec, self._Me_vec,
        )

        # ── Step 3: restore controls ────────────────────────────────────────
        for i in range(n_ctrl):
            controls.set(i, saved[i])

        # ── Step 4: Δq̈ (bio = desired − q̈₀, pros = 0) ─────────────────────
        delta_udot = np.zeros(n_mob)
        for name, idx in zip(self._ctx.bio_coord_names, self._bio_mob_indices):
            delta_udot[idx] = qddot_des_bio.get(name, 0.0) - qdot0[idx]

        # ── Step 5: τ = M · Δq̈ ──────────────────────────────────────────────
        # M depends on q only → same as in step 2 → rebuild (cheap, 21 calls)
        M = build_mass_matrix(
            self._matter, state, n_mob,
            self._e_vec, self._Me_vec,
        )
        tau_full = M @ delta_udot

        # ── Step 6: extract bio rows ────────────────────────────────────────
        return tau_full[self._bio_mob_indices]