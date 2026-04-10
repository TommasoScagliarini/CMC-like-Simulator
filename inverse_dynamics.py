"""
inverse_dynamics.py
===================
Partial inverse-dynamics computation for the biological degrees of freedom.

Method: "zero-muscle residual + mass-matrix projection"
---------------------------------------------------------
At each frame we want the vector of generalised forces  τ_bio  that muscles
and reserve actuators must produce to achieve the desired accelerations
q̈_des (bio DOFs).  The derivation is:

    Equation of motion (full system):
        M(q) · q̈  =  τ_muscles + τ_SEA + τ_GRF + τ_gravity + τ_passive

    Step 1 – zero all MUSCLE controls, keep SEA controls and all other forces:
        M · q̈₀  =  τ_SEA + τ_GRF + τ_gravity + τ_passive
        → realise to acceleration → read q̈₀

    Step 2 – acceleration deficit (bio DOFs only; prosthetic DOFs set to 0):
        Δq̈[i]  =  q̈_des[i] − q̈₀[i]    for i ∈ bio_coords
        Δq̈[j]  =  0                      for j ∈ pros_coords

    Step 3 – project through mass matrix:
        τ_required  =  M · Δq̈
        τ_bio[i]    =  τ_required[i]      for i ∈ bio_coords

This automatically accounts for inter-DOF coupling in M without requiring
an explicit GRF projection step.

Simbody API note
----------------
``SimbodyMatterSubsystem.multiplyByM(state, v, Mv)`` multiplies v by the
full mass matrix M and writes the result into Mv.  Both v and Mv are
opensim.Vector objects of size n_mob.  The state must be realised to at
least Stage::Position before calling this.

If ``multiplyByM`` is not in your SWIG binding (rare), uncomment and use the
``_tau_via_id_solver`` fallback that uses ``opensim.InverseDynamicsSolver``.
"""

from __future__ import annotations

from typing import Dict, List

import numpy as np
import opensim

from config import SimulatorConfig
from model_loader import SimulationContext


class InverseDynamicsComputer:
    """
    Frame-by-frame partial ID using the zero-muscle + M·Δq̈ approach.

    Parameters
    ----------
    cfg : SimulatorConfig
    ctx : SimulationContext
    """

    def __init__(self, cfg: SimulatorConfig, ctx: SimulationContext) -> None:
        self._ctx  = ctx
        self._cfg  = cfg

        model      = ctx.model
        coord_set  = model.getCoordinateSet()

        # Pre-fetch coordinate objects for reading acceleration values
        self._all_coords: List[opensim.Coordinate] = [
            coord_set.get(name) for name in ctx.coord_names
        ]
        self._n_mob   = ctx.n_mob
        self._n_bio   = ctx.n_bio

        # Indices of bio_coords within the full coord list (= mobility vector)
        self._bio_mob_indices: List[int] = [
            ctx.coord_mob_idx[name] for name in ctx.bio_coord_names
        ]
        # Indices of pros_coords (to leave Δq̈ = 0 there)
        self._pros_mob_indices: List[int] = [
            ctx.coord_mob_idx[name] for name in ctx.pros_coord_names
        ]

        # Simbody matter subsystem – used for M·v product
        self._matter = model.getMatterSubsystem()

        # Reusable SimTK::Vector objects (avoid re-allocation every step)
        self._delta_udot  = opensim.Vector(self._n_mob, 0.0)
        self._M_delta_udot = opensim.Vector(self._n_mob, 0.0)

        # Zero-control vector for the muscle-zeroing step
        self._zero_muscle_controls = opensim.Vector(ctx.n_controls, 0.0)

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
        state          : current OpenSim State; must be ≥ Stage::Velocity on entry.
                         On exit the state will have been realised to Acceleration
                         (Stage is NOT rolled back; caller must be aware).
        controls       : full control Vector already containing SEA commands.
                         Muscle entries will be temporarily zeroed.
        qddot_des_bio  : desired accelerations for bio coords {name → [rad/s²]}

        Returns
        -------
        tau_bio : np.ndarray shape (n_bio,)
            Required generalised forces [N·m or N] for each bio coordinate,
            ordered consistently with ctx.bio_coord_names.
        """
        model = self._ctx.model

        # ── Step 1: temporarily zero muscle controls ─────────────────────────
        # We keep SEA controls intact (they are already in `controls`).
        # We do NOT use a copy of controls here; we restore muscle controls
        # after the ID realisation.
        #
        # IMPORTANT: GRF forces are applied through ExternalLoads which are
        # part of the model's ForceSet.  They are evaluated automatically when
        # the state is realised to Stage::Dynamics — no explicit setup needed.
        saved_muscle_controls = {
            name: controls.get(self._ctx.muscle_ctrl_idx[name])
            for name in self._ctx.muscle_names
        }
        for name in self._ctx.muscle_names:
            controls.set(self._ctx.muscle_ctrl_idx[name], 0.0)

        # Apply zeroed-muscle controls to the state cache
        # After setControls, any call to realizeAcceleration will use exactly
        # these control values (the stage cache is now valid).
        model.setControls(state, controls)

        # ── Step 2: realise to Acceleration → q̈₀ ───────────────────────────
        model.realizeAcceleration(state)

        # Read q̈₀ for every DOF (ordered as in coord_names = mobility order)
        q_ddot_0 = np.array([
            coord.getAccelerationValue(state)
            for coord in self._all_coords
        ])

        # ── Step 3: restore muscle controls for subsequent SO step ───────────
        for name, val in saved_muscle_controls.items():
            controls.set(self._ctx.muscle_ctrl_idx[name], val)
        # (We do NOT re-apply controls here; the SO step will set the final
        #  muscle controls and call setControls before the next realisation.)

        # ── Step 4: build Δq̈ vector (bio deficit, zeros elsewhere) ──────────
        for i in range(self._n_mob):
            self._delta_udot.set(i, 0.0)

        for name, idx in zip(self._ctx.bio_coord_names, self._bio_mob_indices):
            des = qddot_des_bio.get(name, 0.0)
            self._delta_udot.set(idx, des - q_ddot_0[idx])

        # ── Step 5: τ_required = M · Δq̈ ────────────────────────────────────
        # multiplyByM requires state ≥ Stage::Position (guaranteed post-realize)
        # M is the generalised mass matrix of the full Simbody system.
        # The result encodes how much extra generalised force muscles must add
        # to cover the acceleration deficit, accounting for all inertial coupling.
        try:
            self._matter.multiplyByM(
                state,
                self._delta_udot,
                self._M_delta_udot,
            )
        except AttributeError:
            # Fallback for rare SWIG bindings that do not expose multiplyByM.
            # Uses the InverseDynamicsSolver instead.
            self._M_delta_udot = self._tau_via_id_solver(state, qddot_des_bio)

        # ── Step 6: extract bio-DOF rows ────────────────────────────────────
        tau_bio = np.array([
            self._M_delta_udot.get(idx)
            for idx in self._bio_mob_indices
        ])

        return tau_bio

    # ─────────────────────────────────────────────────────────────────────────
    #  Fallback (uncomment / call if multiplyByM is unavailable)
    # ─────────────────────────────────────────────────────────────────────────
    def _tau_via_id_solver(
        self,
        state: opensim.State,
        qddot_des_bio: Dict[str, float],
    ) -> opensim.Vector:
        """
        Alternative ID via ``opensim.InverseDynamicsSolver``.

        The solver computes  τ = M·q̈_des + C·q̇ + G − τ_applied  directly,
        accounting for all applied forces (GRF, gravity, passive forces).
        """
        n_mob = self._n_mob
        model = self._ctx.model

        id_solver = opensim.InverseDynamicsSolver(model)

        # Build desired udot vector (bio = desired, pros = current)
        udot_des = opensim.Vector(n_mob, 0.0)
        for name in self._ctx.bio_coord_names:
            idx = self._ctx.coord_mob_idx[name]
            udot_des.set(idx, qddot_des_bio.get(name, 0.0))
        for coord in [
            model.getCoordinateSet().get(n)
            for n in self._ctx.pros_coord_names
        ]:
            idx = self._ctx.coord_mob_idx[coord.getName()]
            udot_des.set(idx, coord.getAccelerationValue(state))

        tau_out = opensim.Vector(n_mob, 0.0)
        id_solver.solve(state, udot_des, tau_out)
        return tau_out
