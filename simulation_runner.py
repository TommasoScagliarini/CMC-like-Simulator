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
  3. Apply controls and compute actual q̈ via bypass (no realizeAcceleration).
  4. Record current state (activations, tau_bio, SEA controls, kinematics).
  5. Advance q and q̇ with a semi-explicit Euler step.  The IK data are a
     reference for the controllers, not a forced output trajectory.

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
  C. OUTER LOOP  – OuterLoop.compute_desired_accelerations() [bio DOFs]
  D. ID          – InverseDynamicsComputer.compute_tau() [bio + pros]
                   (zeros controls + SEA springs, computes q̈₀, restores)
  E. SEA CTRL    – ProsthesisController.compute() with ID feed-forward
  F. SO          – StaticOptimizer.solve() → a, u_res  (bio DOFs)
  G. APPLY CTRL  – apply_to_controls() + set SEA controls + setControls()
  H. COMPUTE q̈  – compute_udot_bypass(state)
  I. RECORD      – append data row to results buffers
  J. EULER STEP  – integrate simulated q and qdot, update SEA motor state

Output files
------------
All results are written as OpenSim-compatible .sto files into cfg.output_dir.
"""

from __future__ import annotations

import os
import time as _time
from typing import Dict

import numpy as np
import opensim

from config import SimulatorConfig
from model_loader import SimulationContext
from kinematics_interpolator import KinematicsInterpolator
from prosthesis_controller import ProsthesisController
from outer_loop import OuterLoop
from inverse_dynamics import InverseDynamicsComputer, compute_udot_bypass
from static_optimization import StaticOptimizer
from output import OutputRecorder


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

        # SEA name ↔ pros coordinate mapping
        self._sea_pros_map = list(zip(
            [cfg.sea_knee_name, cfg.sea_ankle_name],
            cfg.pros_coords,
        ))

        # ── Cache SEA plugin properties (for tau_input computation) ──────────
        # These are parsed from the .osim XML in model_loader. On Windows,
        # OpenSim/Python may not expose custom plugin properties through
        # getPropertyByName(), even though the plugin itself uses them.
        self._sea_props: Dict[str, dict] = {}
        for sea_name in [cfg.sea_knee_name, cfg.sea_ankle_name]:
            if sea_name not in ctx.sea_props:
                raise RuntimeError(
                    f"[Runner] Missing cached SEA properties for '{sea_name}'"
                )
            self._sea_props[sea_name] = dict(ctx.sea_props[sea_name])
            print(f"[Runner] SEA '{sea_name}' props: {self._sea_props[sea_name]}")

        # ── Output recorder ───────────────────────────────────────────────────
        self._recorder = OutputRecorder(
            cfg, ctx, self._sea_pros_map, self._sea_props,
        )

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

        # Pre-fetch CoordinateSet once, used in the Euler step
        coord_set = model.getCoordinateSet()

        # Bypass resources for computing q̈ without realizeAcceleration.
        # The SEA plugin can crash while computing auxiliary state derivatives;
        # compute_udot_bypass uses realizeDynamics + ID solver + M instead.
        matter = model.getMatterSubsystem()
        n_mob  = ctx.n_mob
        _e_vec  = opensim.Vector(n_mob, 0.0)
        _Me_vec = opensim.Vector(n_mob, 0.0)

        # ── Main loop ─────────────────────────────────────────────────────────
        wall_t0     = _time.perf_counter()
        step        = 0
        n_steps_est = int((t_end - t) / dt)

        print(
            f"\n[Runner] Starting simulation  t in [{t:.3f}, {t_end:.3f}] s  "
            f"dt={dt:.4f} s  (~{n_steps_est} steps)\n"
        )

        while t < t_end - dt * 0.5:

            try:
                # ═══════════════════════════════════════════════════════════
                # A. Realise to Velocity
                # ═══════════════════════════════════════════════════════════
                if step < 3:
                    print(f"[DBG t={t:.4f}] A: realizeVelocity ...", flush=True)
                model.realizeVelocity(state)

                # ═══════════════════════════════════════════════════════════
                # B. Reference kinematics at current time
                # ═══════════════════════════════════════════════════════════
                if step < 3:
                    print(f"[DBG t={t:.4f}] B: kin.get ...", flush=True)
                q_ref, qdot_ref, qddot_ref = self._kin.get(t)

                # ═══════════════════════════════════════════════════════════
                # C. Outer loop – desired biological accelerations
                # ═══════════════════════════════════════════════════════════
                if step < 3:
                    print(f"[DBG t={t:.4f}] C: outer_loop ...", flush=True)
                qddot_des_bio = self._outer_loop.compute_desired_accelerations(
                    state, q_ref, qdot_ref, qddot_ref
                )

                # Build desired accelerations for ALL DOFs (bio + pros).
                # Pros DOFs use the reference accelerations only for the ID
                # feed-forward estimate; the applied SEA command comes from
                # ProsthesisController, and the state is integrated dynamically.
                qddot_des_all = dict(qddot_des_bio)
                for coord_name in cfg.pros_coords:
                    qddot_des_all[coord_name] = qddot_ref.get(coord_name, 0.0)

                if cfg.use_muscles_in_so:
                    self._so.prepare_muscle_baseline(state)
                    model.realizeVelocity(state)

                # ═══════════════════════════════════════════════════════════
                # D. Inverse dynamics → τ_bio + τ_pros feed-forward estimate
                #    Zeros controls AND SEA springs, computes q̈₀, then
                #    τ = M · (q̈_des − q̈₀) for ALL DOFs.
                # ═══════════════════════════════════════════════════════════
                if step < 3:
                    print(f"[DBG t={t:.4f}] D: inverse_dynamics ...", flush=True)
                tau_bio, tau_pros_ff = self._id_computer.compute_tau(
                    state, controls, qddot_des_all
                )
                model.realizeVelocity(state)

                # ═══════════════════════════════════════════════════════════
                # E. SEA high-level controller
                #    Feed-forward holds the nominal inverse-dynamics torque;
                #    the PD terms correct residual tracking error.
                # ═══════════════════════════════════════════════════════════
                tau_pros_ff_by_coord = {
                    coord_name: float(tau_pros_ff[i])
                    for i, coord_name in enumerate(ctx.pros_coord_names)
                }
                if step < 3:
                    print(f"[DBG t={t:.4f}] E: prosthesis_ctrl ...", flush=True)
                u_sea = self._prosthesis_ctrl.compute(
                    state, q_ref, qdot_ref, controls,
                    tau_ff=tau_pros_ff_by_coord,
                )
                tau_sea_cmd = np.array([
                    u_sea.get(coord_name, 0.0) * ctx.sea_f_opt[sea_name]
                    for sea_name, coord_name in self._sea_pros_map
                ])

                # ═══════════════════════════════════════════════════════════
                # F. Static Optimisation → muscle activations + reserves
                # ═══════════════════════════════════════════════════════════
                if step < 3:
                    sea_str = ", ".join(f"{k}={v:.4f}" for k, v in u_sea.items())
                    print(f"[DBG t={t:.4f}] F: u_sea = {{{sea_str}}}",
                          flush=True)
                    print(f"[DBG t={t:.4f}] F: static_optimization ...", flush=True)
                a, u_res = self._so.solve(state, tau_bio)

                # ═══════════════════════════════════════════════════════════
                # G. Apply muscle + reserve controls (SEA already set above)
                # ═══════════════════════════════════════════════════════════
                if step < 3:
                    print(f"[DBG t={t:.4f}] G: apply_controls ...", flush=True)
                self._so.apply_to_controls(a, u_res, controls, state)
                self._update_sea_motor_state(state, tau_sea_cmd)
                model.realizeVelocity(state)
                model.setControls(state, controls)

                # ═══════════════════════════════════════════════════════════
                # H. Compute q̈ with final controls (NO realizeAcceleration)
                #
                #    q̈ comes from the bypass, so output kinematics are produced
                #    by the simulated dynamics rather than copied from kin.get().
                # ═══════════════════════════════════════════════════════════
                if step < 3:
                    print(f"[DBG t={t:.4f}] H: compute_udot_bypass ...",
                          flush=True)
                udot = compute_udot_bypass(
                    matter, model, state, n_mob, _e_vec, _Me_vec,
                )
                if not np.all(np.isfinite(udot)):
                    bad_coords = [
                        name for name in ctx.coord_names
                        if not np.isfinite(udot[ctx.coord_mob_idx[name]])
                    ]
                    raise FloatingPointError(
                        f"Non-finite accelerations at t={t:.4f}: {bad_coords[:5]}"
                    )

                # ═══════════════════════════════════════════════════════════
                # I. Record current state BEFORE advancing time
                # ═══════════════════════════════════════════════════════════
                self._recorder.record(
                    t, state, a, u_res, tau_bio, u_sea, udot,
                    controls,
                    so_diagnostics=self._so.last_diagnostics,
                )

                # ═══════════════════════════════════════════════════════════
                # J. Semi-explicit Euler step + SEA motor state update
                # ═══════════════════════════════════════════════════════════
                q_new:    Dict[str, float] = {}
                qdot_new: Dict[str, float] = {}

                for name in ctx.coord_names:
                    coord    = coord_set.get(name)
                    q_cur    = coord.getValue(state)
                    qdot_cur = coord.getSpeedValue(state)
                    qacc_cur = udot[ctx.coord_mob_idx[name]]

                    qdot_next      = qdot_cur + qacc_cur * dt
                    q_new[name]    = q_cur + qdot_next * dt
                    qdot_new[name] = qdot_next

                t_new = t + dt
                self._set_state(state, q_new, qdot_new, t_new)
                self._update_sea_motor_state(state, tau_sea_cmd)

                t = t_new

                if step < 3:
                    print(f"[DBG t={t:.4f}] step {step} complete", flush=True)

            except Exception as exc:
                print(f"\n[Runner] Exception at t={t:.4f} s, step={step}: "
                      f"{type(exc).__name__}: {exc}", flush=True)
                import traceback
                traceback.print_exc()
                print("[Runner] Saving partial results ...", flush=True)
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
        self._recorder.save_results()

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

    def _update_sea_motor_state(
        self,
        state:       opensim.State,
        tau_sea_cmd: np.ndarray,
    ) -> None:
        """
        Set SEA motor_angle to the equilibrium position that produces
        the requested SEA spring torque at the current joint angle.

        In non-impedance mode:  actuation = K * (motor_angle - theta_joint)
        For actuation = τ_required:
            motor_angle = theta_joint + τ_required / K

        In impedance mode: actuation = F_opt * u (independent of motor_angle),
        but we still update motor_angle for consistency.
        """
        ctx   = self._ctx
        model = ctx.model
        coord_set = model.getCoordinateSet()

        sv = model.getStateVariableValues(state)

        for i, (sea_name, coord_name) in enumerate(self._sea_pros_map):
            ma_idx = ctx.sea_motor_angle_sv_idx.get(sea_name)
            ms_idx = ctx.sea_motor_speed_sv_idx.get(sea_name)
            if ma_idx is None:
                continue

            theta_j = coord_set.get(coord_name).getValue(state)
            tau_req = tau_sea_cmd[i]
            K = self._sea_stiffness(sea_name, coord_name)

            # Equilibrium: spring deflection = τ_required / K
            theta_m_eq = theta_j + tau_req / K if K > 1e-10 else theta_j
            sv.set(ma_idx, theta_m_eq)

            # Motor speed: approximate from joint speed (tracks joint)
            if ms_idx is not None:
                omega_j = coord_set.get(coord_name).getSpeedValue(state)
                sv.set(ms_idx, omega_j)

        model.setStateVariableValues(state, sv)

    def _sea_stiffness(self, sea_name: str, coord_name: str) -> float:
        """Return SEA stiffness from parsed plugin props, with config fallback."""
        props = self._sea_props.get(sea_name)
        if props is not None and "K" in props:
            return float(props["K"])

        stiffness = self._cfg.sea_stiffness
        if isinstance(stiffness, dict):
            if sea_name in stiffness:
                return float(stiffness[sea_name])
            if coord_name in stiffness:
                return float(stiffness[coord_name])
            raise KeyError(
                f"Missing SEA stiffness for '{sea_name}' / '{coord_name}'"
            )
        return float(stiffness)

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

