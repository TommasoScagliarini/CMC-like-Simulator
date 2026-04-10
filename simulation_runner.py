"""
simulation_runner.py
====================
Main integration loop.  Ties every component together and advances the
simulation one Δt at a time.

Integration strategy
--------------------
OpenSim's ``Manager`` is used to manage the simulation state and output
storage.  However, the ``Manager.integrate()`` method in OpenSim 4.x does not
provide a per-step callback in Python (without SWIG directors).

We therefore use a **custom semi-explicit Euler loop** that:

  1. Reads the current OpenSim State.
  2. Computes controls (SEA + biological SO) using all sub-components.
  3. Sets those controls into the state.
  4. Calls ``model.realizeAcceleration(state)`` to get q̈.
  5. Advances q and q̇ with a first-order Euler step.
  6. Writes the new (q, q̇, t) back into the OpenSim State and calls
     ``manager.takeOneStep(…)`` so that the Manager records the trajectory.

The Manager integrator is configured with ``SemiExplicitEuler2`` (single-stage,
so controls are evaluated exactly once per time step — consistent with our
zero-order-hold update scheme).

Step order inside each iteration
---------------------------------
  A. SET STATE   – write (q_ref or q_propagated, q̇, t) into state.
  B. REALISE V   – model.realizeVelocity(state)
  C. SEA CTRL    – ProsthesisController.compute() → fills controls[SEA_idx]
  D. OUTER LOOP  – OuterLoop.compute_desired_accelerations()
  E. ID          – InverseDynamicsComputer.compute_tau_bio()
                   (temporarily zeros muscle controls, realises to Accel,
                    reads q̈₀, restores muscle controls)
  F. SO          – StaticOptimizer.solve() → a, u_res
  G. APPLY CTRL  – StaticOptimizer.apply_to_controls()
  H. REALISE A   – model.realizeAcceleration(state)  [with final controls]
  I. INTEGRATE   – Euler: q(t+dt) = q(t) + q̇·dt
                          q̇(t+dt) = q̇(t) + q̈·dt
  J. RECORD      – append data row to results buffers

Output files
------------
All results are written as OpenSim-compatible .sto files into cfg.output_dir.
"""

from __future__ import annotations

import os
import time as _time
from typing import Dict, List

import numpy as np
import opensim

from config import SimulatorConfig
from model_loader import SimulationContext
from kinematics_interpolator import KinematicsInterpolator
from prosthesis_controller import ProsthesisController
from outer_loop import OuterLoop
from inverse_dynamics import InverseDynamicsComputer
from static_optimization import StaticOptimizer


class SimulationRunner:
    """
    Orchestrates the full simulation loop.

    Parameters
    ----------
    cfg : SimulatorConfig
    ctx : SimulationContext    (already built by model_loader.setup_model)
    kin : KinematicsInterpolator
    """

    def __init__(
        self,
        cfg: SimulatorConfig,
        ctx: SimulationContext,
        kin: KinematicsInterpolator,
    ) -> None:
        self._cfg = cfg
        self._ctx = ctx
        self._kin = kin

        # ── Instantiate sub-components ────────────────────────────────────────
        self._prosthesis_ctrl = ProsthesisController(cfg, ctx)
        self._outer_loop      = OuterLoop(cfg, ctx)
        self._id_computer     = InverseDynamicsComputer(cfg, ctx)
        self._so              = StaticOptimizer(cfg, ctx)

        # ── Output buffers (pre-allocated for efficiency) ─────────────────────
        n_steps  = int((cfg.t_end - cfg.t_start) / cfg.dt) + 2
        n_coords = len(ctx.coord_names)

        self._rec_time          = np.full(n_steps, np.nan)
        self._rec_q             = np.full((n_steps, n_coords),         np.nan)
        self._rec_qdot          = np.full((n_steps, n_coords),         np.nan)
        self._rec_activations   = np.full((n_steps, ctx.n_muscles),   np.nan)
        self._rec_u_res         = np.full((n_steps, ctx.n_reserves),  np.nan)
        self._rec_tau_bio       = np.full((n_steps, ctx.n_bio),       np.nan)
        self._rec_sea_controls  = np.full((n_steps, 2),               np.nan)
        self._step_count        = 0

        # ── Make output directory ─────────────────────────────────────────────
        os.makedirs(cfg.output_dir, exist_ok=True)

    # ─────────────────────────────────────────────────────────────────────────
    #  Public entry point
    # ─────────────────────────────────────────────────────────────────────────
    def run(self) -> None:
        """Execute the full simulation loop from t_start to t_end."""
        cfg   = self._cfg
        ctx   = self._ctx
        model = ctx.model
        state = ctx.state

        dt     = cfg.dt
        t      = cfg.t_start
        t_end  = cfg.t_end

        # ── Initialise Manager ────────────────────────────────────────────────
        # SemiExplicitEuler2 is a single-stage integrator: forces (and thus
        # controls) are evaluated exactly once per step, which is consistent
        # with our zero-order-hold control update between steps.
        manager = opensim.Manager(model)
        manager.setIntegratorMethod(
            opensim.Manager.IntegratorMethod_SemiExplicitEuler2
        )
        manager.setIntegratorMaximumStepSize(dt)
        manager.setIntegratorMinimumStepSize(dt * 0.01)
        manager.setIntegratorAccuracy(1e-4)

        # ── Set initial state from reference kinematics ───────────────────────
        q0, qdot0, _ = self._kin.get(t)
        self._set_state(state, q0, qdot0, t)
        model.equilibrateMuscles(state)   # re-equilibrate at start position
        manager.initialize(state)

        # Pre-allocate the full OpenSim controls vector (reused every step)
        controls = opensim.Vector(ctx.n_controls, 0.0)

        # ── Main loop ─────────────────────────────────────────────────────────
        wall_t0     = _time.perf_counter()
        step        = 0
        n_steps_est = int((t_end - t) / dt)

        print(
            f"\n[Runner] Starting simulation  t ∈ [{t:.3f}, {t_end:.3f}] s  "
            f"dt={dt:.4f} s  (~{n_steps_est} steps)\n"
        )

        while t < t_end - dt * 0.5:

            # ═══════════════════════════════════════════════════════════════
            # A. Ensure Manager state is current
            # ═══════════════════════════════════════════════════════════════
            state = manager.getState()

            # ═══════════════════════════════════════════════════════════════
            # B. Realise to Velocity
            #    Required by ProsthesisController and OuterLoop (they read
            #    Coordinate values and speeds).
            # ═══════════════════════════════════════════════════════════════
            model.realizeVelocity(state)

            # ═══════════════════════════════════════════════════════════════
            # C. Reference kinematics at current time
            # ═══════════════════════════════════════════════════════════════
            q_ref, qdot_ref, qddot_ref = self._kin.get(t)

            # ═══════════════════════════════════════════════════════════════
            # D. SEA high-level controller
            #    Computes u ∈ [−1,+1] for each SEA and writes them into
            #    `controls`.  Must happen BEFORE ID so that the SEA torque
            #    is included in the zero-muscle acceleration q̈₀.
            # ═══════════════════════════════════════════════════════════════
            u_sea = self._prosthesis_ctrl.compute(
                state, q_ref, qdot_ref, controls
            )

            # ═══════════════════════════════════════════════════════════════
            # E. Outer loop – desired biological accelerations
            # ═══════════════════════════════════════════════════════════════
            qddot_des_bio = self._outer_loop.compute_desired_accelerations(
                state, q_ref, qdot_ref, qddot_ref
            )

            # ═══════════════════════════════════════════════════════════════
            # F. Inverse dynamics → required bio generalised forces τ_bio
            #
            #    CRITICAL ORDER:
            #      - SEA controls must already be in `controls` (step D)
            #      - ID internally zeros muscle controls, realises, reads q̈₀,
            #        then restores them.  On exit, `controls` still has the
            #        SEA commands and zero muscle commands (SO fills them next).
            # ═══════════════════════════════════════════════════════════════
            tau_bio = self._id_computer.compute_tau_bio(
                state, controls, qddot_des_bio
            )

            # ═══════════════════════════════════════════════════════════════
            # G. Static Optimisation → muscle activations + reserve controls
            #
            #    State must be ≥ Stage::Velocity for moment-arm computation.
            #    After the ID step, the state has been realised to Acceleration,
            #    so this is guaranteed.
            # ═══════════════════════════════════════════════════════════════
            a, u_res = self._so.solve(state, tau_bio)

            # ═══════════════════════════════════════════════════════════════
            # H. Apply all controls to the model
            #
            #    After setControls(state, controls), any subsequent call to
            #    realizeAcceleration will use exactly these control values
            #    (the controls cache is marked valid).
            # ═══════════════════════════════════════════════════════════════
            self._so.apply_to_controls(a, u_res, controls)
            model.setControls(state, controls)

            # ═══════════════════════════════════════════════════════════════
            # I. Record current state BEFORE advancing time
            # ═══════════════════════════════════════════════════════════════
            self._record(t, state, a, u_res, tau_bio, u_sea)

            # ═══════════════════════════════════════════════════════════════
            # J. Advance simulation by one Δt
            #
            #    manager.integrate(t + dt) uses the controls currently set in
            #    the state (step H) for the SemiExplicitEuler2 single-stage
            #    step.  The Manager also stores the trajectory internally for
            #    later export.
            # ═══════════════════════════════════════════════════════════════
            try:
                state = manager.integrate(t + dt)
            except RuntimeError as exc:
                print(f"\n[Runner] Integration failed at t={t:.4f} s: {exc}")
                print("[Runner] Saving partial results …")
                break

            t    = state.getTime()
            step += 1

            # Progress report every 50 steps
            if step % 50 == 0:
                elapsed  = _time.perf_counter() - wall_t0
                progress = (t - cfg.t_start) / (t_end - cfg.t_start) * 100
                eta      = elapsed / max(progress, 1e-3) * (100 - progress)
                print(
                    f"  t={t:.3f}/{t_end:.3f} s  "
                    f"({progress:.1f}%)  "
                    f"elapsed={elapsed:.1f}s  ETA={eta:.0f}s"
                )

        # ── Save outputs ──────────────────────────────────────────────────────
        elapsed_total = _time.perf_counter() - wall_t0
        print(
            f"\n[Runner] Simulation complete. "
            f"{step} steps, {elapsed_total:.1f} s wall time."
        )
        self._save_results(manager)

    # ─────────────────────────────────────────────────────────────────────────
    #  Private helpers
    # ─────────────────────────────────────────────────────────────────────────
    def _set_state(
        self,
        state:  opensim.State,
        q:      Dict[str, float],
        qdot:   Dict[str, float],
        t:      float,
    ) -> None:
        """
        Write (q, q̇, t) into an OpenSim State via its state-variable vector.

        This is the cleanest way to set positions and velocities atomically:
        model.setStateVariableValues() invalidates all dependent caches (forces,
        accelerations) so subsequent realisations start from a clean slate.
        """
        ctx   = self._ctx
        model = ctx.model

        sv = model.getStateVariableValues(state)

        for coord_name, val in q.items():
            idx = ctx.q_sv_idx.get(coord_name)
            if idx is not None:
                sv.set(idx, val)

        for coord_name, val in qdot.items():
            idx = ctx.qdot_sv_idx.get(coord_name)
            if idx is not None:
                sv.set(idx, val)

        model.setStateVariableValues(state, sv)
        state.setTime(t)

    def _record(
        self,
        t:       float,
        state:   opensim.State,
        a:       np.ndarray,
        u_res:   np.ndarray,
        tau_bio: np.ndarray,
        u_sea:   Dict[str, float],
    ) -> None:
        """Append one row to all output buffers."""
        ctx = self._ctx
        k   = self._step_count

        self._rec_time[k] = t

        for i, coord in enumerate(
            [ctx.model.getCoordinateSet().get(n) for n in ctx.coord_names]
        ):
            self._rec_q[k, i]    = coord.getValue(state)
            self._rec_qdot[k, i] = coord.getSpeedValue(state)

        self._rec_activations[k]  = a
        self._rec_u_res[k]        = u_res
        self._rec_tau_bio[k]      = tau_bio
        self._rec_sea_controls[k, 0] = u_sea.get(
            self._cfg.pros_coords[0], 0.0
        )
        self._rec_sea_controls[k, 1] = u_sea.get(
            self._cfg.pros_coords[1], 0.0
        )

        self._step_count += 1

    def _save_results(self, manager: opensim.Manager) -> None:
        """Write all output files."""
        cfg = self._cfg
        ctx = self._ctx
        k   = self._step_count   # number of completed steps
        out = cfg.output_dir
        pfx = cfg.output_prefix

        # ── 1. Full OpenSim trajectory (Manager-stored) ───────────────────────
        # This is the canonical OpenSim output that can be loaded in the GUI.
        storage = manager.getStatesTrajectory()  # opensim.StatesTrajectory
        # Export as .sto via Storage (legacy API, widely compatible)
        # Convert StatesTrajectory → Storage
        sto_path = os.path.join(out, f"{pfx}_states.sto")
        states_table = opensim.StatesTrajectoryReporter.convertToTable(
            storage, ctx.model
        )
        opensim.STOFileAdapter.write(states_table, sto_path)
        print(f"  → States   : {sto_path}")

        # ── 2. Muscle activations ─────────────────────────────────────────────
        if cfg.save_activations:
            path = os.path.join(out, f"{pfx}_activations.sto")
            _write_sto(
                path,
                header_name="Activations",
                time=self._rec_time[:k],
                col_names=ctx.muscle_names,
                data=self._rec_activations[:k],
            )
            print(f"  → Activations: {path}")

        # ── 3. SEA controls ───────────────────────────────────────────────────
        if cfg.save_sea_controls:
            path = os.path.join(out, f"{pfx}_sea_controls.sto")
            _write_sto(
                path,
                header_name="SEAControls",
                time=self._rec_time[:k],
                col_names=cfg.pros_coords,
                data=self._rec_sea_controls[:k],
            )
            print(f"  → SEA ctrl : {path}")

        # ── 4. Kinematics ─────────────────────────────────────────────────────
        if cfg.save_kinematics:
            path = os.path.join(out, f"{pfx}_kinematics.sto")
            _write_sto(
                path,
                header_name="Kinematics_q",
                time=self._rec_time[:k],
                col_names=ctx.coord_names,
                data=self._rec_q[:k],
                in_degrees=False,   # stored in radians
            )
            print(f"  → Kinematics: {path}")

        # ── 5. Bio generalised forces ─────────────────────────────────────────
        if cfg.save_tau_bio:
            path = os.path.join(out, f"{pfx}_tau_bio.sto")
            _write_sto(
                path,
                header_name="GeneralisedForces_bio",
                time=self._rec_time[:k],
                col_names=ctx.bio_coord_names,
                data=self._rec_tau_bio[:k],
            )
            print(f"  → Tau_bio  : {path}")

        print(f"\n[Runner] All results saved to: {os.path.abspath(out)}")


# ─────────────────────────────────────────────────────────────────────────────
#  Utility: write an OpenSim .sto file
# ─────────────────────────────────────────────────────────────────────────────
def _write_sto(
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
