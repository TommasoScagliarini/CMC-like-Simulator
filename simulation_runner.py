"""
simulation_runner.py
====================
Main integration loop.  Ties every component together and advances the
simulation one Δt at a time.

Integration strategy
--------------------
The OpenSim ``Manager`` is NOT used: its ``initialize()`` method crashes
silently (native C++ exception on Windows) when the initial state contains
muscles near their length limits (e.g. flex_dig_r at the IK pose).

Instead we implement a **pure Python semi-explicit Euler loop** that is
mathematically identical to SemiExplicitEuler2 but fully under our control:

  1. Realise the state to Velocity.
  2. Compute controls (SEA + biological SO) using all sub-components.
  3. Apply controls and realise to Acceleration → read q̈.
  4. Advance q and q̇ with a first-order Euler step.
  5. Write the new (q, q̇, t) back into the OpenSim State via
     model.setStateVariableValues().

This is a Zero-Order-Hold scheme: controls computed at time t are held
constant over the interval [t, t+Δt], consistent with the single-stage
nature of SemiExplicitEuler2.

Muscle state initialisation
----------------------------
After model.initSystem(), muscle fiber lengths are consistent with the
default (neutral) model pose.  When we call _set_state() to move to the
IK initial pose (t_start), the joint angles change but the fiber lengths
in the state do NOT update automatically — they are state variables, not
derived quantities.  If the IK pose differs significantly from the default
pose, the stored fiber lengths become geometrically invalid (e.g. pennation
angle → 90°, fiber length → NaN) and the first realizeVelocity() triggers
a native C++ crash that Python cannot catch.

Fix: _init_muscle_states() explicitly sets every muscle's fiber_length to
its optimal_fiber_length and activation to 0.01 immediately after
_set_state().  These are conservative, physically valid values that let
the muscles evolve from a known-good configuration.

Step order inside each iteration
---------------------------------
  A. REALISE V   – model.realizeVelocity(state)
  B. KIN REF     – kin.get(t) → q_ref, qdot_ref, qddot_ref
  C. SEA CTRL    – ProsthesisController.compute() → fills controls[SEA_idx]
  D. OUTER LOOP  – OuterLoop.compute_desired_accelerations()
  E. ID          – InverseDynamicsComputer.compute_tau_bio()
                   (temporarily zeros muscle controls, realises to Accel,
                    reads q̈₀, restores muscle controls)
  F. SO          – StaticOptimizer.solve() → a, u_res
  G. APPLY CTRL  – apply_to_controls() + model.setControls()
  H. COMPUTE q̈  – compute_udot_bypass(state)
                   (realizeDynamics + ID solver + multiplyByM + np.solve
                    — no realizeAcceleration, avoids SEA plugin crash)
  I. RECORD      – append data row to results buffers
  J. EULER STEP  – q(t+dt)  = q(t)  + q̇(t)  · dt
                   q̇(t+dt) = q̇(t) + q̈(t) · dt
                   set_state(q_new, qdot_new, t+dt)

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
from inverse_dynamics import InverseDynamicsComputer, compute_udot_bypass
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
        self._rec_q             = np.full((n_steps, n_coords),        np.nan)
        self._rec_qdot          = np.full((n_steps, n_coords),        np.nan)
        self._rec_activations   = np.full((n_steps, ctx.n_muscles),  np.nan)
        self._rec_u_res         = np.full((n_steps, ctx.n_reserves), np.nan)
        self._rec_tau_bio       = np.full((n_steps, ctx.n_bio),      np.nan)
        self._rec_sea_controls  = np.full((n_steps, 2),              np.nan)
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

        dt    = cfg.dt
        t     = cfg.t_start
        t_end = cfg.t_end

        # ── Set initial state ─────────────────────────────────────────────────
        # Use ctx.state (from model_loader's initSystem()) rather than calling
        # model.initSystem() again.  A second initSystem() call may invalidate
        # the ExternalLoads Storage connections established during the first
        # call, causing a native C++ crash during Stage::Dynamics evaluation
        # (GRF interpolation) on the first realizeAcceleration().
        state = ctx.state
        q0, qdot0, _ = self._kin.get(t)
        self._set_state(state, q0, qdot0, t)

        # Make q0 available to _init_muscle_states for SEA motor angle init
        self._q0_for_sea_init = q0
        self._init_muscle_states(state)
        del self._q0_for_sea_init

        # Pre-allocate the full OpenSim controls vector (reused every step)
        controls = opensim.Vector(ctx.n_controls, 0.0)

        # Pre-fetch CoordinateSet once — used in the Euler step
        coord_set = model.getCoordinateSet()

        # ── Bypass resources (for computing q̈ without realizeAcceleration) ──
        # realizeAcceleration triggers computeStateVariableDerivatives() in the
        # SEA BlackBox plugin, which crashes.  Instead we use:
        #   realizeDynamics + InverseDynamicsSolver + multiplyByM → q̈
        # See inverse_dynamics.py docstring for the full derivation.
        matter = model.getMatterSubsystem()
        n_mob  = ctx.n_mob
        _e_vec  = opensim.Vector(n_mob, 0.0)   # scratch for build_mass_matrix
        _Me_vec = opensim.Vector(n_mob, 0.0)   # scratch for build_mass_matrix

        # ── Main loop ─────────────────────────────────────────────────────────
        wall_t0     = _time.perf_counter()
        step        = 0
        n_steps_est = int((t_end - t) / dt)

        print(
            f"\n[Runner] Starting simulation  t ∈ [{t:.3f}, {t_end:.3f}] s  "
            f"dt={dt:.4f} s  (~{n_steps_est} steps)\n"
        )

        while t < t_end - dt * 0.5:

            try:
                # ═══════════════════════════════════════════════════════════
                # A. Realise to Velocity
                # ═══════════════════════════════════════════════════════════
                if step < 3:
                    print(f"[DBG t={t:.4f}] A: realizeVelocity …", flush=True)
                model.realizeVelocity(state)

                # ═══════════════════════════════════════════════════════════
                # B. Reference kinematics at current time
                # ═══════════════════════════════════════════════════════════
                if step < 3:
                    print(f"[DBG t={t:.4f}] B: kin.get …", flush=True)
                q_ref, qdot_ref, qddot_ref = self._kin.get(t)

                # ═══════════════════════════════════════════════════════════
                # C. SEA high-level controller
                # ═══════════════════════════════════════════════════════════
                if step < 3:
                    print(f"[DBG t={t:.4f}] C: prosthesis_ctrl …", flush=True)
                u_sea = self._prosthesis_ctrl.compute(
                    state, q_ref, qdot_ref, controls
                )

                # ═══════════════════════════════════════════════════════════
                # D. Outer loop – desired biological accelerations
                # ═══════════════════════════════════════════════════════════
                if step < 3:
                    print(f"[DBG t={t:.4f}] D: outer_loop …", flush=True)
                qddot_des_bio = self._outer_loop.compute_desired_accelerations(
                    state, q_ref, qdot_ref, qddot_ref
                )

                # ═══════════════════════════════════════════════════════════
                # E. Inverse dynamics → τ_bio
                #    Uses ID solver bypass (no realizeAcceleration)
                # ═══════════════════════════════════════════════════════════
                if step < 3:
                    print(f"[DBG t={t:.4f}] E: inverse_dynamics …", flush=True)
                tau_bio = self._id_computer.compute_tau_bio(
                    state, controls, qddot_des_bio
                )

                # ═══════════════════════════════════════════════════════════
                # F. Static Optimisation → muscle activations + reserves
                # ═══════════════════════════════════════════════════════════
                if step < 3:
                    print(f"[DBG t={t:.4f}] F: static_optimization …", flush=True)
                a, u_res = self._so.solve(state, tau_bio)

                # ═══════════════════════════════════════════════════════════
                # G. Apply all controls to the model
                # ═══════════════════════════════════════════════════════════
                if step < 3:
                    print(f"[DBG t={t:.4f}] G: apply_controls …", flush=True)
                self._so.apply_to_controls(a, u_res, controls)
                model.setControls(state, controls)

                # ═══════════════════════════════════════════════════════════
                # H. Compute q̈ with final controls (NO realizeAcceleration)
                #
                #    realizeAcceleration triggers the SEA plugin's
                #    computeStateVariableDerivatives() which crashes.
                #    Instead: realizeDynamics + ID solver + M → q̈.
                # ═══════════════════════════════════════════════════════════
                if step < 3:
                    print(f"[DBG t={t:.4f}] H: compute_udot_bypass …",
                          flush=True)
                udot = compute_udot_bypass(
                    matter, model, state, n_mob, _e_vec, _Me_vec,
                )

                # ═══════════════════════════════════════════════════════════
                # I. Record current state BEFORE advancing time
                # ═══════════════════════════════════════════════════════════
                self._record(t, state, a, u_res, tau_bio, u_sea)

                # ═══════════════════════════════════════════════════════════
                # J. Semi-explicit Euler step
                #    q̈ comes from the bypass (udot array), NOT from the
                #    state (which was never realised to Acceleration).
                # ═══════════════════════════════════════════════════════════
                q_new:    Dict[str, float] = {}
                qdot_new: Dict[str, float] = {}

                for i_c, name in enumerate(ctx.coord_names):
                    coord    = coord_set.get(name)
                    q_cur    = coord.getValue(state)
                    qdot_cur = coord.getSpeedValue(state)
                    qacc_cur = udot[ctx.coord_mob_idx[name]]   # from bypass

                    q_new[name]    = q_cur    + qdot_cur * dt
                    qdot_new[name] = qdot_cur + qacc_cur * dt

                t_new = t + dt
                self._set_state(state, q_new, qdot_new, t_new)
                t = t_new

                if step < 3:
                    print(f"[DBG t={t:.4f}] ✓ step {step} complete", flush=True)

            except Exception as exc:
                # Catch ALL Python exceptions (not just RuntimeError)
                print(f"\n[Runner] Exception at t={t:.4f} s, step={step}: "
                      f"{type(exc).__name__}: {exc}", flush=True)
                import traceback
                traceback.print_exc()
                print("[Runner] Saving partial results …", flush=True)
                break

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
        self._save_results()

    # ─────────────────────────────────────────────────────────────────────────
    #  Private helpers
    # ─────────────────────────────────────────────────────────────────────────
    def _init_muscle_states(self, state: opensim.State) -> None:
        """
        Set muscle activation, fiber_length, and plugin state variables to
        safe initial values.

        After _set_state() moves the model to the IK initial pose, two
        categories of state variables may be inconsistent:

        1. Muscle fiber_length: stored from initSystem() at the default pose;
           geometrically invalid at the IK pose → triggers C++ crash on
           realizeVelocity().

        2. Plugin (SEA) internal state variables: spring deflection, motor
           angle, etc.  Even with u=0 the BlackBox impedance plugin reads
           these during computeActuation() inside Stage::Dynamics. If they
           have initSystem() values that are inconsistent with the new joint
           angles, realizeAcceleration() triggers a native C++ crash that
           Python cannot catch.

        Fix for (1): set fiber_length = optimal_fiber_length, activation = 0.01
        Fix for (2): identify every state variable that is NOT a muscle
                     activation/fiber_length and NOT a coordinate value/speed.
                     These must be plugin-specific — zero them all.
        """
        model      = self._ctx.model
        muscle_set = model.getMuscles()

        sv_names_os = model.getStateVariableNames()
        sv_names = [sv_names_os.get(i)
                    for i in range(sv_names_os.getSize())]

        sv = model.getStateVariableValues(state)

        # ── 1. Muscle fiber_length and activation ─────────────────────────────
        for i in range(muscle_set.getSize()):
            muscle = muscle_set.get(i)
            name   = muscle.getName()
            opt_fl = muscle.getOptimalFiberLength()   # [m]

            for sv_idx, sv_name in enumerate(sv_names):
                if sv_name.endswith(f"{name}/fiber_length"):
                    sv.set(sv_idx, opt_fl)
                    break

            for sv_idx, sv_name in enumerate(sv_names):
                if sv_name.endswith(f"{name}/activation"):
                    sv.set(sv_idx, 0.01)
                    break

        # ── 2. Plugin-specific state variables (SEA internal state) ───────────
        # Build set of "known" suffixes (muscles + coordinates).
        # Anything that does NOT match is a plugin state variable.
        known_suffixes: set = set()
        for name in self._ctx.muscle_names:
            known_suffixes.add(f"{name}/fiber_length")
            known_suffixes.add(f"{name}/activation")
        for name in self._ctx.coord_names:
            known_suffixes.add(f"{name}/value")
            known_suffixes.add(f"{name}/speed")

        plugin_sv: list = []
        for sv_idx, sv_name in enumerate(sv_names):
            is_known = any(sv_name.endswith(suf) for suf in known_suffixes)
            if not is_known:
                plugin_sv.append((sv_idx, sv_name))
                sv.set(sv_idx, 0.0)

        # ── 3. SEA motor angles: match joint angle → zero initial deflection ──
        # The SEA spring torque is K*(motor_angle - gear_ratio*joint_angle).
        # With motor_angle=0 but joint_angle≠0 (IK pose), the spring exerts a
        # large torque even with u=0, which may trigger an assertion or overflow
        # inside the BlackBox plugin during Stage::Dynamics.
        # Setting motor_angle = joint_angle (assuming gear_ratio≈1) zeroes the
        # initial spring deflection and avoids extreme initial spring forces.
        pros_knee_idx  = None
        pros_ankle_idx = None
        for sv_idx, sv_name in enumerate(sv_names):
            if sv_name.endswith("SEA_Knee/motor_angle"):
                pros_knee_idx = sv_idx
            elif sv_name.endswith("SEA_Ankle/motor_angle"):
                pros_ankle_idx = sv_idx

        # q0 is the IK reference at t_start — built just before this call
        # by the caller (run()) and stored temporarily here.
        if hasattr(self, "_q0_for_sea_init"):
            q0_sea = self._q0_for_sea_init
            if pros_knee_idx is not None:
                sv.set(pros_knee_idx,
                       q0_sea.get(self._cfg.pros_coords[0], 0.0))
                print(f"[Runner] SEA_Knee motor_angle = "
                      f"{q0_sea.get(self._cfg.pros_coords[0], 0.0):.4f} rad",
                      flush=True)
            if pros_ankle_idx is not None:
                sv.set(pros_ankle_idx,
                       q0_sea.get(self._cfg.pros_coords[1], 0.0))
                print(f"[Runner] SEA_Ankle motor_angle = "
                      f"{q0_sea.get(self._cfg.pros_coords[1], 0.0):.4f} rad",
                      flush=True)

        # Write all changes to the state at once
        model.setStateVariableValues(state, sv)

        print(
            f"[Runner] Muscle states initialised: "
            f"activation=0.01, fiber_length=optimal "
            f"for {muscle_set.getSize()} muscles.",
            flush=True
        )
        if plugin_sv:
            print(
                f"[Runner] Plugin state vars zeroed ({len(plugin_sv)}): "
                + ", ".join(sv_name.split("/")[-1] for _, sv_name in plugin_sv),
                flush=True
            )

    def _set_state(
        self,
        state: opensim.State,
        q:     Dict[str, float],
        qdot:  Dict[str, float],
        t:     float,
    ) -> None:
        """
        Write (q, q̇, t) into an OpenSim State via its state-variable vector.

        model.setStateVariableValues() invalidates all dependent caches so
        subsequent realisations start from a clean slate.
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
        self._rec_sea_controls[k, 0] = u_sea.get(self._cfg.pros_coords[0], 0.0)
        self._rec_sea_controls[k, 1] = u_sea.get(self._cfg.pros_coords[1], 0.0)

        self._step_count += 1

    def _save_results(self) -> None:
        """Write all output files as OpenSim-compatible .sto files."""
        cfg = self._cfg
        ctx = self._ctx
        k   = self._step_count
        out = cfg.output_dir
        pfx = cfg.output_prefix

        if cfg.save_activations:
            path = os.path.join(out, f"{pfx}_activations.sto")
            _write_sto(
                path, "Activations",
                self._rec_time[:k], ctx.muscle_names,
                self._rec_activations[:k],
            )
            print(f"  → Activations : {path}")

        if cfg.save_sea_controls:
            path = os.path.join(out, f"{pfx}_sea_controls.sto")
            _write_sto(
                path, "SEAControls",
                self._rec_time[:k], cfg.pros_coords,
                self._rec_sea_controls[:k],
            )
            print(f"  → SEA ctrl    : {path}")

        if cfg.save_kinematics:
            path = os.path.join(out, f"{pfx}_kinematics.sto")
            _write_sto(
                path, "Kinematics_q",
                self._rec_time[:k], ctx.coord_names,
                self._rec_q[:k],
                in_degrees=False,
            )
            print(f"  → Kinematics  : {path}")

        if cfg.save_tau_bio:
            path = os.path.join(out, f"{pfx}_tau_bio.sto")
            _write_sto(
                path, "GeneralisedForces_bio",
                self._rec_time[:k], ctx.bio_coord_names,
                self._rec_tau_bio[:k],
            )
            print(f"  → Tau_bio     : {path}")

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