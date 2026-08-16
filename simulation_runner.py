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

Two-level loop (CMC-like)
-------------------------
When use_control_window=True the loop has two levels:

  Outer (control) step — once every T_control:
    A. REALISE V   – model.realizeVelocity(state)
    B. KIN REF     – kin.get(t) → q_ref, qdot_ref, qddot_ref
    C. OUTER LOOP  – OuterLoop.compute_desired_accelerations()  [bio]
    D. ID          – InverseDynamicsComputer.compute_tau()      [bio target]
    E. SEA CTRL    – ProsthesisController.compute()
    F. SO          – StaticOptimizer.solve() → a, u_res         [bio]
    G. APPLY CTRL  – apply_to_controls() + setControls()

  Inner (integration) substep — repeats N = T_control / integration_dt times,
  with controls held constant:
    H. COMPUTE q̈  – compute_udot_bypass(state)        (re-evaluated)
    I. RECORD      – append data row at t_substep
    J. EULER STEP  – advance q, q̇, SEA motor state by integration_dt

When use_control_window=False the loop degenerates to T_control = dt and
N = 1 substep, which reproduces the original single-step behaviour exactly.

Output files
------------
All results are written as OpenSim-compatible .sto files into cfg.output_dir.
"""

from __future__ import annotations

import copy
import os
import json
import time as _time
from typing import Dict, Mapping

import numpy as np
import opensim

from binary_phase_detector import BinaryPhaseDetectorSampler
from config import SimulatorConfig
from model_loader import SimulationContext
from kinematics_interpolator import KinematicsInterpolator
from prosthesis_controller import ProsthesisController
from outer_loop import OuterLoop
from inverse_dynamics import InverseDynamicsComputer, compute_udot_bypass
from static_optimization import StaticOptimizer
from output import OutputRecorder
from online_grf import (
    StreamingGaitEventDetector,
    online_grf_sensor_channels,
    read_online_grf,
)


class SegmentWallClockTimeout(RuntimeError):
    """Raised when ``step_until`` exceeds its per-call wall-clock budget.

    Used by RL adapters to truncate a *degenerate* episode (a simulation state
    that integrates pathologically slowly, e.g. Static Optimization falling back
    to bounded least-squares) instead of letting a single slow worker stall the
    whole synchronous-sampling iteration. This is a deliberate guard, not a
    numerical fault, so callers should always truncate gracefully on it,
    independent of ``fail_fast``.
    """


class PhaseSensorSamplingError(RuntimeError):
    """Raised when the causal 1 ms heel/toe sample contract is violated."""


class BinaryPhaseSensorTransportError(RuntimeError):
    """Raised when the force-free binary sample contract is violated."""


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
        if cfg.sea_forward_mode not in {"plugin", "ideal_torque"}:
            raise ValueError(
                "sea_forward_mode must be 'plugin' or 'ideal_torque', "
                f"got {cfg.sea_forward_mode!r}"
            )
        integration_scheme = getattr(
            cfg, "integration_scheme", "semi_implicit_euler"
        )
        if integration_scheme not in {"semi_implicit_euler", "rk4_bypass"}:
            raise ValueError(
                "integration_scheme must be 'semi_implicit_euler' or "
                f"'rk4_bypass', got {integration_scheme!r}"
            )
        if cfg.sea_forward_mode == "plugin" and cfg.dt > 0.001:
            print(
                "[Runner] WARNING: plugin SEA dynamics are stiff; "
                "dt > 0.001 s can destabilize the validated path.",
                flush=True,
            )

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

        self._sea_integrated_state_items: list[tuple[str, str, int, str]] = []
        for sea_name, _coord_name in self._sea_pros_map:
            component_path = f"/forceset/{sea_name}"
            component = ctx.model.getComponent(component_path)
            state_map = getattr(ctx, "sea_state_sv_idx", {}).get(sea_name, {})
            for state_name, sv_idx in sorted(state_map.items(), key=lambda item: item[1]):
                output_name = f"{state_name}_dot"
                try:
                    component.getOutput(output_name)
                except Exception as exc:
                    raise RuntimeError(
                        "[Runner] SEA state variable lacks derivative output: "
                        f"{component_path}/{state_name} requires output "
                        f"'{output_name}' for {cfg.integration_scheme}"
                    ) from exc
                self._sea_integrated_state_items.append(
                    (sea_name, state_name, sv_idx, output_name)
                )
        print(
            "[Runner] SEA RK4 state variables: "
            f"{[(sea, name) for sea, name, _idx, _out in self._sea_integrated_state_items]}",
            flush=True,
        )

        # ── Output recorder ───────────────────────────────────────────────────
        self._recorder = OutputRecorder(
            cfg, ctx, self._sea_pros_map, self._sea_props,
        )

        # ── Make output directory ─────────────────────────────────────────────
        os.makedirs(cfg.output_dir, exist_ok=True)

        self._runtime_state: opensim.State | None = None
        self._runtime_controls: opensim.Vector | None = None
        self._runtime_time: float | None = None
        self._runtime_step: int = 0
        self._last_step_info: dict = {}
        self._phase_sensor_sample_dt_s = float(
            getattr(cfg, "detector_sample_dt_s", 0.001)
        )
        if (
            not np.isfinite(self._phase_sensor_sample_dt_s)
            or self._phase_sensor_sample_dt_s <= 0.0
        ):
            raise ValueError("detector_sample_dt_s must be finite and positive.")
        self._phase_sensor_last_sample_time_s: float | None = None
        self._phase_sensor_sampling_enabled = bool(
            len(getattr(ctx, "online_grf_detector_force_paths", [])) == 2
        )
        binary_profile = getattr(ctx, "binary_phase_detector_profile", None)
        self._binary_phase_detector_sampler = (
            None
            if binary_profile is None
            else BinaryPhaseDetectorSampler(ctx.model, binary_profile)
        )
        self._binary_phase_sensor_sample_dt_s = self._phase_sensor_sample_dt_s
        self._binary_phase_sensor_last_sample_time_s: float | None = None
        self._binary_phase_sensor_sampling_enabled = bool(
            self._binary_phase_detector_sampler is not None
        )
        self._binary_phase_sensor_baseline: dict | None = None
        self._online_event_detector = None
        self._online_event_force_paths = list(
            getattr(ctx, "online_grf_detector_force_paths", [])
            or getattr(ctx, "online_grf_force_paths", [])
        )
        self._online_event_force_sides = dict(
            getattr(ctx, "online_grf_detector_force_sides", {})
            or getattr(ctx, "online_grf_force_sides", {})
        )
        if self._online_event_force_paths:
            confirmation_threshold = float(
                getattr(cfg, "online_grf_hs_confirmation_threshold_n", 0.0)
            )
            if confirmation_threshold <= 0.0:
                confirmation_threshold = float(
                    getattr(
                        ctx,
                        (
                            "online_grf_detector_hs_confirmation_threshold_n"
                            if getattr(ctx, "online_grf_detector_force_paths", [])
                            else "online_grf_hs_confirmation_threshold_n"
                        ),
                        0.0,
                    )
                )
            self._online_event_detector = StreamingGaitEventDetector(
                cfg.grf_contact_threshold_n,
                cfg.grf_min_contact_duration_s,
                cfg.grf_min_cycle_duration_s,
                (
                    confirmation_threshold
                    if confirmation_threshold > 0.0
                    else None
                ),
            )

    # ─────────────────────────────────────────────────────────────────────────
    #  Public entry point
    # ─────────────────────────────────────────────────────────────────────────
    @property
    def state(self) -> opensim.State:
        """Current mutable OpenSim state for step-wise integrations."""
        if self._runtime_state is None:
            raise RuntimeError("Runner state is not initialised; call reset_to_time().")
        return self._runtime_state

    @property
    def current_time(self) -> float:
        """Current simulation time for step-wise integrations."""
        if self._runtime_time is None:
            raise RuntimeError("Runner time is not initialised; call reset_to_time().")
        return self._runtime_time

    @property
    def last_step_info(self) -> dict:
        """Diagnostics from the latest public step_until() control window."""
        return dict(self._last_step_info)

    def reset_to_time(self, t: float) -> opensim.State:
        """
        Reset state, controls, controller memory, SO warm start, and bypass
        resources for interactive/RL use without rebuilding the OpenSim model.
        """
        cfg = self._cfg
        ctx = self._ctx
        model = ctx.model
        state = ctx.state
        t = float(t)

        q0, qdot0, _ = self._kin.get(t)
        self._set_state(state, q0, qdot0, t)

        self._q0_for_sea_init = q0
        self._qdot0_for_sea_init = qdot0
        try:
            self._init_muscle_states(state)
        finally:
            if hasattr(self, "_q0_for_sea_init"):
                del self._q0_for_sea_init
            if hasattr(self, "_qdot0_for_sea_init"):
                del self._qdot0_for_sea_init

        self._runtime_state = state
        self._runtime_controls = opensim.Vector(ctx.n_controls, 0.0)
        self._runtime_time = t
        self._runtime_step = 0
        self._last_step_info = {}
        self._phase_sensor_last_sample_time_s = None
        self._binary_phase_sensor_last_sample_time_s = None
        self._binary_phase_sensor_baseline = None
        if self._online_event_detector is not None:
            self._online_event_detector.reset()

        self._matter = model.getMatterSubsystem()
        self._n_mob = ctx.n_mob
        self._e_vec = opensim.Vector(ctx.n_mob, 0.0)
        self._Me_vec = opensim.Vector(ctx.n_mob, 0.0)

        self._prosthesis_ctrl.reset()
        self._so.remember_solution(
            np.full(ctx.n_muscles, cfg.muscle_min_activation),
            np.zeros(ctx.n_reserves),
        )
        model.realizeVelocity(state)
        model.setControls(state, self._runtime_controls)
        if self._phase_sensor_sampling_enabled:
            # Establish and validate the open-left endpoint.  This sample is
            # intentionally not returned to the policy step and cannot earn
            # detector/event credit; the first emitted sample is t + 1 ms.
            self._sample_phase_detector_channels(state, t)
            self._phase_sensor_last_sample_time_s = t
        if self._binary_phase_sensor_sampling_enabled:
            # Establish the same open-left endpoint for the two raw bits.  The
            # t0 reading is diagnostic only and is never emitted as a sample,
            # edge, HS, or TO.
            self._sample_binary_phase_detector_channels(state, t)
            self._binary_phase_sensor_last_sample_time_s = t
            self._binary_phase_sensor_baseline = dict(
                self._last_step_info["binary_phase_sensor_diagnostics"]
            )
            self._last_step_info["binary_phase_sensor_baseline"] = dict(
                self._binary_phase_sensor_baseline
            )
        return state

    def step_until(
        self,
        t_stop: float,
        record: bool = True,
        wall_timeout_s: float = 0.0,
    ) -> dict:
        """
        Advance the already-reset runner up to t_stop using the CMC-like loop.
        This is the public stepping API for trajectory-level RL adapters.

        ``wall_timeout_s`` (> 0) caps the real wall-clock time spent inside this
        call. The budget is checked between control sub-steps; if a degenerate
        simulation state makes the integration crawl, the call raises
        :class:`SegmentWallClockTimeout` so an RL adapter can truncate the
        episode instead of letting one slow worker gate synchronous sampling.
        """
        cfg = self._cfg
        if self._runtime_state is None or self._runtime_controls is None:
            self.reset_to_time(cfg.t_start)

        state = self.state
        controls = self._runtime_controls
        t = self.current_time
        t_stop = float(t_stop)
        if t_stop < t - 1e-12:
            raise ValueError(
                f"t_stop ({t_stop:.6g}) is earlier than current time ({t:.6g})."
            )

        _t_control, h_sub_nominal, n_substeps = self._control_loop_timing()
        step_online_events: list[dict] = []
        sea_segment_samples: list[dict[str, dict[str, float]]] = []
        phase_sensor_samples: list[dict[str, float]] = []
        binary_phase_sensor_samples: list[dict[str, float | bool]] = []
        so_solver_audit_entries: list[dict] = []
        phase_segment_start_time_s = float(t)
        wall_timeout_s = float(wall_timeout_s)
        wall_deadline = (
            _time.monotonic() + wall_timeout_s if wall_timeout_s > 0.0 else None
        )

        while t < t_stop - 1e-12:
            (
                a, u_res, u_sea,
                tau_bio, tau_pros_ff_by_coord, tau_sea_cmd,
                q_ref_win, qdot_ref_win,
            ) = self._compute_controls_for_window(state, controls, t)
            so_solver_audit_entries.append(
                {
                    "control_window_index": len(so_solver_audit_entries) + 1,
                    "control_window_time_s": float(t),
                    "selected_feasibility_attempt_index": self._so.last_diagnostics.get(
                        "selected_feasibility_attempt_index"
                    ),
                    "served_solution_sha256": self._so.last_diagnostics.get(
                        "served_solution_sha256"
                    ),
                    "selected_solver_solution_matches_served": (
                        self._so.last_diagnostics.get(
                            "selected_solver_solution_matches_served"
                        )
                    ),
                    "attempts": copy.deepcopy(
                        self._so.last_diagnostics.get("solver_attempt_audit", [])
                    ),
                }
            )
            self._last_step_info = {
                "time": t,
                "u_sea": dict(u_sea),
                "q_ref": dict(q_ref_win),
                "qdot_ref": dict(qdot_ref_win),
                "tau_pros_ff": dict(tau_pros_ff_by_coord),
                "so_diagnostics": dict(self._so.last_diagnostics),
                "so_solver_audit_entries": copy.deepcopy(
                    so_solver_audit_entries
                ),
            }

            for _sub in range(n_substeps):
                if t >= t_stop - 1e-12:
                    break
                if wall_deadline is not None and _time.monotonic() > wall_deadline:
                    raise SegmentWallClockTimeout(
                        f"step_until exceeded its {wall_timeout_s:g} s wall-clock "
                        f"budget at sim time t={t:.6g}s (target {t_stop:.6g}s)."
                    )
                h_step = min(h_sub_nominal, t_stop - t)
                udot, sea_plugin_outputs, sea_derivatives = (
                    self._integrate_evaluate(state, controls, t)
                )
                sea_segment_samples.append(
                    self._sea_runtime_diagnostics(
                        state,
                        u_sea,
                        sea_plugin_outputs,
                    )
                )
                online_grf, online_events = self._sample_online_grf(state, t)
                if online_grf is not None:
                    step_online_events.extend(online_events)
                    self._last_step_info["online_grf"] = self._online_grf_info(
                        online_grf
                    )
                    self._last_step_info["online_events"] = list(step_online_events)

                if record:
                    self._recorder.record(
                        t, state, a, u_res, tau_bio, u_sea, udot,
                        controls,
                        q_ref=q_ref_win,
                        qdot_ref=qdot_ref_win,
                        tau_pros_ff=tau_pros_ff_by_coord,
                        sea_derivatives=sea_derivatives,
                        sea_plugin_outputs=sea_plugin_outputs,
                        so_diagnostics=self._so.last_diagnostics,
                        online_grf=online_grf,
                        online_events=online_events,
                    )

                if getattr(
                    cfg,
                    "integration_scheme",
                    "semi_implicit_euler",
                ) == "rk4_bypass":
                    t = self._advance_rk4_bypass_state(state, controls, h_step)
                elif cfg.sea_forward_mode == "plugin":
                    t = self._advance_plugin_state_substeps(
                        state, controls, udot, h_step
                    )
                else:
                    t = self._advance_legacy_euler_state(
                        state, controls, udot, tau_sea_cmd, h_step
                    )

                self._runtime_time = t
                self._runtime_step += 1
                self._append_phase_sensor_sample(
                    state,
                    t,
                    phase_sensor_samples,
                )
                self._append_binary_phase_sensor_sample(
                    state,
                    t,
                    binary_phase_sensor_samples,
                )

        self._last_step_info["sea_segment_diagnostics"] = (
            self._summarize_sea_segment_diagnostics(sea_segment_samples)
        )
        self._finalize_phase_sensor_segment(
            segment_start_time_s=phase_segment_start_time_s,
            t_stop=t_stop,
            samples=phase_sensor_samples,
        )
        self._last_step_info["phase_sensor_samples"] = [
            dict(sample) for sample in phase_sensor_samples
        ]
        self._finalize_binary_phase_sensor_segment(
            segment_start_time_s=phase_segment_start_time_s,
            t_stop=t_stop,
            samples=binary_phase_sensor_samples,
        )
        if self._binary_phase_sensor_sampling_is_enabled():
            self._last_step_info["binary_phase_sensor_samples"] = [
                dict(sample) for sample in binary_phase_sensor_samples
            ]
            if self._binary_phase_sensor_baseline is not None:
                self._last_step_info["binary_phase_sensor_baseline"] = dict(
                    self._binary_phase_sensor_baseline
                )
        return self.last_step_info

    def save_results(self) -> None:
        """Public wrapper around the output recorder."""
        self._recorder.save_results()

    def reset_outputs(self) -> None:
        """Clear recorded samples while keeping the same model/context."""
        self._recorder = OutputRecorder(
            self._cfg, self._ctx, self._sea_pros_map, self._sea_props,
        )

    def run(self) -> None:
        """Execute the full simulation loop from t_start to t_end."""
        cfg   = self._cfg
        ctx   = self._ctx
        model = ctx.model

        # Two-level loop parameters. When use_control_window=False the outer
        # loop runs at integration_dt (legacy single-step behaviour).
        use_window = bool(getattr(cfg, "use_control_window", False))
        h_int = float(getattr(cfg, "integration_dt", cfg.dt))
        if use_window:
            T_control = float(getattr(cfg, "T_control", h_int))
            if T_control < h_int:
                T_control = h_int
        else:
            T_control = h_int

        # N substeps per control window (integer; rounded to keep T_control exact)
        n_substeps = max(1, int(round(T_control / h_int)))
        # Effective dt that each substep advances by — keeps T_control exact
        # even when T_control / h_int is not perfectly divisible.
        h_sub = T_control / n_substeps

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
        self._qdot0_for_sea_init = qdot0
        try:
            self._init_muscle_states(state)
        finally:
            del self._q0_for_sea_init
            del self._qdot0_for_sea_init

        # Pre-allocate the full OpenSim controls vector (reused every step)
        controls = opensim.Vector(ctx.n_controls, 0.0)

        # Pre-fetch CoordinateSet once, used in the Euler step
        coord_set = model.getCoordinateSet()

        # Bypass resources for computing q̈ without realizeAcceleration.
        # The SEA plugin can crash while computing auxiliary state derivatives;
        # compute_udot_bypass uses realizeDynamics + ID solver + M instead.
        # Stored on self so the per-substep evaluator can reuse them.
        self._matter = model.getMatterSubsystem()
        self._n_mob  = ctx.n_mob
        self._e_vec  = opensim.Vector(ctx.n_mob, 0.0)
        self._Me_vec = opensim.Vector(ctx.n_mob, 0.0)

        # ── Main loop ─────────────────────────────────────────────────────────
        wall_t0     = _time.perf_counter()
        step        = 0
        n_windows_est = int((t_end - t) / T_control)
        n_steps_est = n_windows_est * n_substeps
        failure_exc: Exception | None = None
        failure_t = t
        failure_step = step

        print(
            f"\n[Runner] Starting simulation  t in [{t:.3f}, {t_end:.3f}] s  "
            f"T_control={T_control:.4f} s  h_sub={h_sub:.4f} s  "
            f"(~{n_windows_est} windows × {n_substeps} substeps "
            f"= {n_steps_est} steps)\n"
        )
        self._write_progress_heartbeat("simulation", t, step, 0.0)

        while t < t_end - h_sub * 0.5:

            try:
                # ═══════════════════════════════════════════════════════════
                # OUTER (CONTROL) STEP — once per T_control
                # Computes muscle activations, reserves, and SEA commands
                # that will be held constant during the integration substeps.
                # ═══════════════════════════════════════════════════════════
                (
                    a, u_res, u_sea,
                    tau_bio, tau_pros_ff_by_coord, tau_sea_cmd,
                    q_ref_win, qdot_ref_win,
                ) = self._compute_controls_for_window(state, controls, t)

                # ═══════════════════════════════════════════════════════════
                # INNER (INTEGRATION) SUBSTEPS — n_substeps × h_sub = T_control
                # Controls held constant; bypass re-evaluates udot at each
                # substep state so the integrator sees the true response.
                # ═══════════════════════════════════════════════════════════
                for _sub in range(n_substeps):
                    udot, sea_plugin_outputs, sea_derivatives = (
                        self._integrate_evaluate(state, controls, t)
                    )
                    online_grf, online_events = self._sample_online_grf(state, t)

                    self._recorder.record(
                        t, state, a, u_res, tau_bio, u_sea, udot,
                        controls,
                        q_ref=q_ref_win,
                        qdot_ref=qdot_ref_win,
                        tau_pros_ff=tau_pros_ff_by_coord,
                        sea_derivatives=sea_derivatives,
                        sea_plugin_outputs=sea_plugin_outputs,
                        so_diagnostics=self._so.last_diagnostics,
                        online_grf=online_grf,
                        online_events=online_events,
                    )

                    if getattr(
                        cfg,
                        "integration_scheme",
                        "semi_implicit_euler",
                    ) == "rk4_bypass":
                        t_new = self._advance_rk4_bypass_state(
                            state, controls, h_sub
                        )
                    elif cfg.sea_forward_mode == "plugin":
                        t_new = self._advance_plugin_state_substeps(
                            state, controls, udot, h_sub
                        )
                    else:
                        q_new:    Dict[str, float] = {}
                        qdot_new: Dict[str, float] = {}
                        for name in ctx.coord_names:
                            coord    = coord_set.get(name)
                            q_cur    = coord.getValue(state)
                            qdot_cur = coord.getSpeedValue(state)
                            qacc_cur = udot[ctx.coord_mob_idx[name]]

                            qdot_next      = qdot_cur + qacc_cur * h_sub
                            q_new[name]    = q_cur + qdot_next * h_sub
                            qdot_new[name] = qdot_next

                        t_new = t + h_sub
                        self._set_state(state, q_new, qdot_new, t_new)
                        self._update_sea_motor_state(state, tau_sea_cmd)

                    t = t_new
                    step += 1

            except Exception as exc:
                print(f"\n[Runner] Exception at t={t:.4f} s, step={step}: "
                      f"{type(exc).__name__}: {exc}", flush=True)
                import traceback
                traceback.print_exc()
                print("[Runner] Saving partial results ...", flush=True)
                failure_exc = exc
                failure_t = t
                failure_step = step
                break

            if step <= n_substeps:
                print(f"[Runner] t={t:.4f} - first window OK", flush=True)
                self._write_progress_heartbeat(
                    "simulation",
                    t,
                    step,
                    _time.perf_counter() - wall_t0,
                )

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
                self._write_progress_heartbeat("simulation", t, step, elapsed)

        # ── Save outputs ──────────────────────────────────────────────────────
        elapsed_total = _time.perf_counter() - wall_t0
        if failure_exc is None:
            print(
                f"\n[Runner] Simulation complete. "
                f"{step} steps, {elapsed_total:.1f} s wall time."
            )
        else:
            print(
                f"\n[Runner] Simulation stopped early at t={failure_t:.4f} s, "
                f"step={failure_step}, after {elapsed_total:.1f} s wall time."
            )
        self._recorder.save_results()
        status_t = t if failure_exc is None else failure_t
        status_step = step if failure_exc is None else failure_step
        self._write_run_status(failure_exc, status_t, status_step, elapsed_total)
        self._write_progress_heartbeat(
            "complete" if failure_exc is None else "failed",
            status_t,
            status_step,
            elapsed_total,
        )
        if failure_exc is not None:
            raise RuntimeError(
                "Simulation stopped early; partial results were saved to "
                f"{os.path.abspath(self._cfg.output_dir)}"
            ) from failure_exc

    # ─────────────────────────────────────────────────────────────────────────
    #  Private helpers
    # ─────────────────────────────────────────────────────────────────────────
    def _control_loop_timing(self) -> tuple[float, float, int]:
        """Return (T_control, h_sub, n_substeps) for the configured loop."""
        cfg = self._cfg
        use_window = bool(getattr(cfg, "use_control_window", False))
        h_int = float(getattr(cfg, "integration_dt", cfg.dt))
        if use_window:
            t_control = float(getattr(cfg, "T_control", h_int))
            if t_control < h_int:
                t_control = h_int
        else:
            t_control = h_int

        n_substeps = max(1, int(round(t_control / h_int)))
        h_sub = t_control / n_substeps
        return t_control, h_sub, n_substeps

    def _advance_legacy_euler_state(
        self,
        state: opensim.State,
        controls: opensim.Vector,
        udot: np.ndarray,
        tau_sea_cmd: np.ndarray,
        dt: float,
    ) -> float:
        """Legacy non-plugin fallback retained for run() parity."""
        ctx = self._ctx
        coord_set = ctx.model.getCoordinateSet()
        q_new: Dict[str, float] = {}
        qdot_new: Dict[str, float] = {}
        for name in ctx.coord_names:
            coord = coord_set.get(name)
            q_cur = coord.getValue(state)
            qdot_cur = coord.getSpeedValue(state)
            qacc_cur = udot[ctx.coord_mob_idx[name]]

            qdot_next = qdot_cur + qacc_cur * dt
            q_new[name] = q_cur + qdot_next * dt
            qdot_new[name] = qdot_next

        t_new = float(state.getTime()) + dt
        self._set_state(state, q_new, qdot_new, t_new)
        self._update_sea_motor_state(state, tau_sea_cmd)
        ctx.model.realizeVelocity(state)
        ctx.model.setControls(state, controls)
        return t_new

    def _compute_controls_for_window(
        self,
        state:    opensim.State,
        controls: opensim.Vector,
        t:        float,
    ) -> tuple:
        """
        Compute all control quantities for the current control window.

        Mutates `controls` in place (muscle activations, reserves, SEA u).
        Mutates `state` (prepare_muscle_baseline updates fiber lengths;
        ID computer transiently zeros and restores controls/SEA springs).
        Returns the held-constant quantities the inner integration loop
        needs for recording.
        """
        cfg   = self._cfg
        ctx   = self._ctx
        model = ctx.model

        # A. Realise to Velocity
        model.realizeVelocity(state)

        # B. Reference kinematics at window start
        q_ref, qdot_ref, qddot_ref = self._kin.get(t)

        # C. Outer loop – raw desired biological accelerations. The feasibility
        # guard below may scale only the PD correction around qddot_ref.
        qddot_des_bio_raw = self._outer_loop.compute_desired_accelerations(
            state, q_ref, qdot_ref, qddot_ref
        )

        if cfg.use_muscles_in_so:
            self._so.prepare_muscle_baseline(state)
            model.realizeVelocity(state)

        if getattr(cfg, "enable_so_feasibility_backtracking", True):
            raw_scales = getattr(cfg, "so_backtracking_scales", [1.0])
        else:
            raw_scales = [1.0]
        scales = []
        for value in raw_scales:
            scale = float(value)
            if np.isfinite(scale) and scale >= 0.0:
                scales.append(scale)
        if not scales:
            scales = [1.0]

        abs_tol = float(getattr(cfg, "so_residual_abs_tol", 1e-6))
        rel_tol = float(getattr(cfg, "so_residual_rel_tol", 1e-3))
        best_candidate = None
        best_metric = float("inf")
        best_abs = float("inf")
        solver_attempt_audit: list[dict] = []

        # D/F. ID + SO feasibility backtracking. Each candidate recomputes the
        # full inverse-dynamics vector so tau_bio matches the selected biological
        # request. The prosthetic slice is retained only as an oracle diagnostic:
        # it is no longer fed into the SEA outer controller.
        for attempt_idx, scale in enumerate(scales, start=1):
            qddot_des_all = {}
            for coord_name in ctx.bio_coord_names:
                qddot_ref_value = qddot_ref.get(coord_name, 0.0)
                qddot_raw_value = qddot_des_bio_raw.get(
                    coord_name,
                    qddot_ref_value,
                )
                qddot_des_all[coord_name] = (
                    qddot_ref_value
                    + scale * (qddot_raw_value - qddot_ref_value)
                )
            for coord_name in cfg.pros_coords:
                qddot_des_all[coord_name] = qddot_ref.get(coord_name, 0.0)

            tau_bio_candidate, tau_pros_ff_candidate = (
                self._id_computer.compute_tau(state, controls, qddot_des_all)
            )
            model.realizeVelocity(state)
            a_candidate, u_res_candidate = self._so.solve(
                state,
                tau_bio_candidate,
                feasibility_scale=scale,
            )

            diag_candidate = dict(self._so.last_diagnostics)
            residual_norm = float(
                diag_candidate.get("residual_norm", float("inf"))
            )
            residual_rel = float(
                diag_candidate.get("residual_relative_norm", float("inf"))
            )
            residual_max_abs = float(
                diag_candidate.get("residual_max_abs", float("inf"))
            )
            feasible = (
                residual_norm <= abs_tol
                or residual_max_abs <= abs_tol
                or residual_rel <= rel_tol
            )
            diag_candidate["feasibility_attempts"] = float(attempt_idx)
            diag_candidate["feasibility_accepted"] = 1.0 if feasible else 0.0
            solver_attempt_audit.append(
                {
                    "attempt_index": int(attempt_idx),
                    "feasibility_scale": float(scale),
                    "feasibility_accepted": bool(feasible),
                    "residual_norm": residual_norm,
                    "residual_relative_norm": residual_rel,
                    "residual_max_abs": residual_max_abs,
                    "solver_fallback_used": diag_candidate.get(
                        "solver_fallback_used"
                    ),
                    "solver_path": dict(diag_candidate.get("solver_path", {})),
                }
            )

            metric = residual_rel if np.isfinite(residual_rel) else float("inf")
            abs_metric = (
                residual_norm if np.isfinite(residual_norm) else float("inf")
            )
            if (
                metric < best_metric
                or (
                    np.isclose(metric, best_metric, rtol=0.0, atol=1e-12)
                    and abs_metric < best_abs
                )
            ):
                best_metric = metric
                best_abs = abs_metric
                best_candidate = (
                    a_candidate.copy(),
                    u_res_candidate.copy(),
                    tau_bio_candidate.copy(),
                    tau_pros_ff_candidate.copy(),
                    diag_candidate,
                    True if feasible else False,
                    scale,
                    residual_norm,
                    residual_rel,
                )

            if feasible:
                break

        if best_candidate is None:
            raise FloatingPointError(
                f"No finite SO feasibility candidate at t={t:.4f}"
            )

        (
            a, u_res, tau_bio, tau_pros_ff,
            selected_diag, feasibility_accepted, selected_scale,
            selected_residual_norm, selected_residual_rel,
        ) = best_candidate
        selected_diag["feasibility_accepted"] = (
            1.0 if feasibility_accepted else 0.0
        )
        selected_diag["selected_feasibility_attempt_index"] = int(
            selected_diag["feasibility_attempts"]
        )
        for audit_entry in solver_attempt_audit:
            audit_entry["selected"] = bool(
                audit_entry["attempt_index"]
                == selected_diag["selected_feasibility_attempt_index"]
            )
        selected_diag["solver_attempt_audit"] = solver_attempt_audit
        served_solution_sha256 = self._so._array_sha256(
            np.concatenate([a, u_res])
        )
        selected_solver_solution_sha256 = (
            selected_diag.get("solver_path", {})
            .get("selected_solution", {})
            .get("output_sha256")
        )
        selected_diag["served_solution_sha256"] = served_solution_sha256
        selected_diag["selected_solver_solution_matches_served"] = bool(
            served_solution_sha256 is not None
            and served_solution_sha256 == selected_solver_solution_sha256
        )
        self._so.last_diagnostics = selected_diag
        self._so.remember_solution(a, u_res)

        if not feasibility_accepted:
            warn_count = getattr(self, "_so_backtracking_warning_count", 0) + 1
            self._so_backtracking_warning_count = warn_count
            if warn_count <= 10 or warn_count % 100 == 0:
                print(
                    "[Runner] WARNING: SO feasibility backtracking did not "
                    f"meet tolerance at t={t:.4f}; using scale="
                    f"{selected_scale:.6g}, |res|={selected_residual_norm:.3g}, "
                    f"rel={selected_residual_rel:.3g}",
                    flush=True,
                )

        # E. SEA high-level controller (writes SEA u into `controls`)
        tau_pros_ff_by_coord = {
            coord_name: float(tau_pros_ff[i])
            for i, coord_name in enumerate(ctx.pros_coord_names)
        }
        u_sea = self._prosthesis_ctrl.compute(
            state, q_ref, qdot_ref, controls,
        )
        tau_sea_cmd = np.array([
            u_sea.get(coord_name, 0.0) * ctx.sea_f_opt[sea_name]
            for sea_name, coord_name in self._sea_pros_map
        ])

        # G. Apply muscle + reserve controls (SEA already set above)
        self._so.apply_to_controls(a, u_res, controls, state)
        if cfg.sea_forward_mode == "ideal_torque":
            self._update_sea_motor_state(state, tau_sea_cmd)
        model.realizeVelocity(state)
        model.setControls(state, controls)

        return (
            a, u_res, u_sea,
            tau_bio, tau_pros_ff_by_coord, tau_sea_cmd,
            q_ref, qdot_ref,
        )

    @staticmethod
    def _online_grf_info(grf: dict) -> dict:
        """Return JSON-friendly aggregate GRF and per-sensor detector data.

        The existing ``left``/``right`` payload is preserved.  ``sensors`` is
        additive and exposes the heel/toe normal load, penetration and contact
        flag without forwarding any contact force into the model dynamics.
        """
        result = {}
        for side in ("left", "right"):
            item = grf["sides"][side]
            result[side] = {
                "force": np.asarray(item["force"], dtype=float).tolist(),
                "moment": np.asarray(item["moment"], dtype=float).tolist(),
                "cop": np.asarray(item["cop"], dtype=float).tolist(),
                "normal_force": float(item["normal_force"]),
                "penetration": float(item["penetration"]),
                "slip_speed": float(item["slip_speed"]),
                "in_contact": bool(item["in_contact"]),
            }
        result["sensors"] = online_grf_sensor_channels(grf)
        return result

    def _sample_phase_detector_channels(
        self,
        state: opensim.State,
        time_s: float,
    ) -> dict[str, float]:
        """Read the two non-force-applying prosthetic detector channels.

        This deliberately bypasses ``StreamingGaitEventDetector``: it is a
        sample transport for the high-rate FSM, not a second event producer.
        The physical primary GRF stream is neither read nor modified here.
        """
        ctx = self._ctx
        detector_paths = list(
            getattr(ctx, "online_grf_detector_force_paths", [])
        )
        if len(detector_paths) != 2:
            raise PhaseSensorSamplingError(
                "High-rate phase sampling requires exactly two detector "
                f"spheres, got {len(detector_paths)}."
            )
        detector_grf = read_online_grf(
            ctx.model,
            state,
            detector_paths,
            ctx.online_grf_detector_force_sides,
        )
        channels = online_grf_sensor_channels(detector_grf)
        required_roles = {"left_heel", "left_toe"}
        actual_roles = set(channels)
        if actual_roles != required_roles:
            raise PhaseSensorSamplingError(
                "High-rate phase sampling requires exactly left_heel and "
                f"left_toe detector roles, got {sorted(actual_roles)}."
            )

        sample: dict[str, float] = {"time_s": float(time_s)}
        for role, output_name in (
            ("left_heel", "left_heel_normal_n"),
            ("left_toe", "left_toe_normal_n"),
        ):
            channel = channels.get(role)
            if not isinstance(channel, Mapping):
                raise PhaseSensorSamplingError(
                    f"Detector channel {role!r} is missing or malformed."
                )
            try:
                value = float(channel["normal_load_n"])
            except (KeyError, TypeError, ValueError) as exc:
                raise PhaseSensorSamplingError(
                    f"Detector channel {role!r} has no numeric normal load."
                ) from exc
            if not np.isfinite(value) or value < 0.0:
                raise PhaseSensorSamplingError(
                    f"Detector channel {role!r} is non-finite or negative "
                    f"at t={float(time_s):.9g}."
                )
            sample[output_name] = value

        if not np.isfinite(sample["time_s"]):
            raise PhaseSensorSamplingError(
                "Detector sample timestamp must be finite."
            )
        self._last_step_info["online_grf_detector"] = self._online_grf_info(
            detector_grf
        )
        return sample

    def _phase_sensor_sampling_is_enabled(self) -> bool:
        """Return whether the configured detector is the two-sensor contract."""
        configured = getattr(self, "_phase_sensor_sampling_enabled", None)
        if configured is not None:
            return bool(configured)
        return bool(
            len(getattr(self._ctx, "online_grf_detector_force_paths", [])) == 2
        )

    def _append_phase_sensor_sample(
        self,
        state: opensim.State,
        time_s: float,
        samples: list[dict[str, float]],
    ) -> None:
        """Append one sample on the reset-anchored detector lattice.

        Integrator states before the next 1 ms boundary are ignored.  Crossing
        a boundary, revisiting it, or moving backwards is a hard error because
        interpolation would silently change the detector semantics.
        """
        if not self._phase_sensor_sampling_is_enabled():
            return
        current = float(time_s)
        if not np.isfinite(current):
            raise PhaseSensorSamplingError(
                "Detector sample timestamp must be finite."
            )
        previous = self._phase_sensor_last_sample_time_s
        if previous is None:
            raise PhaseSensorSamplingError(
                "Detector sampling baseline is missing; reset_to_time() must "
                "establish the open-left endpoint first."
            )
        dt = float(self._phase_sensor_sample_dt_s)
        tolerance = max(1e-12, dt * 1e-7)
        delta = current - float(previous)
        if delta < -tolerance:
            raise PhaseSensorSamplingError(
                "Detector sample timestamps are non-monotonic: "
                f"{current:.12g} follows {float(previous):.12g}."
            )
        if abs(delta) <= tolerance:
            raise PhaseSensorSamplingError(
                f"Duplicate detector sample timestamp {current:.12g}."
            )

        expected = float(previous) + dt
        if current < expected - tolerance:
            return
        if current > expected + tolerance:
            raise PhaseSensorSamplingError(
                "Missing detector sample: expected "
                f"{expected:.12g}, reached {current:.12g}."
            )

        sample = self._sample_phase_detector_channels(state, expected)
        for key in (
            "time_s",
            "left_heel_normal_n",
            "left_toe_normal_n",
        ):
            try:
                value = float(sample[key])
            except (KeyError, TypeError, ValueError) as exc:
                raise PhaseSensorSamplingError(
                    f"Detector sample field {key!r} is missing or malformed."
                ) from exc
            if not np.isfinite(value) or (key != "time_s" and value < 0.0):
                raise PhaseSensorSamplingError(
                    f"Detector sample field {key!r} is non-finite or invalid."
                )
            sample[key] = value
        if samples and sample["time_s"] <= samples[-1]["time_s"] + tolerance:
            raise PhaseSensorSamplingError(
                "Detector segment contains duplicate or non-monotonic samples."
            )
        samples.append(sample)
        self._phase_sensor_last_sample_time_s = expected

    def _finalize_phase_sensor_segment(
        self,
        *,
        segment_start_time_s: float,
        t_stop: float,
        samples: list[dict[str, float]],
    ) -> None:
        """Validate the open-left/closed-right segment before publishing it."""
        if not self._phase_sensor_sampling_is_enabled():
            if samples:
                raise PhaseSensorSamplingError(
                    "Detector samples exist without a configured detector."
                )
            return
        start = float(segment_start_time_s)
        stop = float(t_stop)
        dt = float(self._phase_sensor_sample_dt_s)
        tolerance = max(1e-12, dt * 1e-7)
        duration = stop - start
        if duration < -tolerance:
            raise PhaseSensorSamplingError(
                "Detector segment end precedes its start."
            )
        if abs(duration) <= tolerance:
            if samples:
                raise PhaseSensorSamplingError(
                    "A zero-duration detector segment must contain no samples."
                )
            return

        count_float = duration / dt
        expected_count = int(round(count_float))
        if expected_count <= 0 or abs(count_float - expected_count) > 1e-7:
            raise PhaseSensorSamplingError(
                "Detector segment is not aligned to detector_sample_dt_s: "
                f"duration={duration:.12g}, dt={dt:.12g}."
            )
        if len(samples) != expected_count:
            raise PhaseSensorSamplingError(
                "Detector segment is incomplete: expected "
                f"{expected_count} samples, got {len(samples)}."
            )
        for index, sample in enumerate(samples, start=1):
            expected_time = start + index * dt
            try:
                sample_time = float(sample["time_s"])
                heel_force = float(sample["left_heel_normal_n"])
                toe_force = float(sample["left_toe_normal_n"])
            except (KeyError, TypeError, ValueError) as exc:
                raise PhaseSensorSamplingError(
                    "Detector segment contains a malformed sample."
                ) from exc
            if (
                not np.isfinite(sample_time)
                or not np.isfinite(heel_force)
                or not np.isfinite(toe_force)
                or heel_force < 0.0
                or toe_force < 0.0
            ):
                raise PhaseSensorSamplingError(
                    "Detector segment contains a non-finite or invalid sample."
                )
            if abs(sample_time - expected_time) > tolerance:
                raise PhaseSensorSamplingError(
                    "Detector segment contains a duplicate, non-monotonic, "
                    "missing, or off-grid timestamp."
                )
        expected_first = start + dt
        expected_last = stop
        if (
            abs(float(samples[0]["time_s"]) - expected_first) > tolerance
            or abs(float(samples[-1]["time_s"]) - expected_last) > tolerance
        ):
            raise PhaseSensorSamplingError(
                "Detector segment violates (previous_time, t_stop] boundaries."
            )
        if (
            self._phase_sensor_last_sample_time_s is None
            or abs(self._phase_sensor_last_sample_time_s - stop) > tolerance
        ):
            raise PhaseSensorSamplingError(
                "Detector endpoint was not sampled exactly once."
            )

    def _binary_phase_sensor_sampling_is_enabled(self) -> bool:
        """Return whether the force-free two-point sampler is configured."""
        configured = getattr(
            self,
            "_binary_phase_sensor_sampling_enabled",
            None,
        )
        if configured is not None:
            return bool(configured)
        return bool(
            getattr(self, "_binary_phase_detector_sampler", None) is not None
        )

    def _sample_binary_phase_detector_channels(
        self,
        state: opensim.State,
        time_s: float,
    ) -> dict[str, float | bool]:
        """Read two point/plane switches without assigning gait semantics."""
        sampler = getattr(self, "_binary_phase_detector_sampler", None)
        if sampler is None:
            raise BinaryPhaseSensorTransportError(
                "Binary detector sampler is missing."
            )
        reading = sampler.sample(state, time_s)
        sample = reading.public_sample()
        expected_fields = {
            "time_s",
            "left_heel_contact",
            "left_toe_contact",
        }
        if set(sample) != expected_fields:
            raise BinaryPhaseSensorTransportError(
                "Binary detector sample fields differ from the frozen schema."
            )
        try:
            timestamp = float(sample["time_s"])
        except (TypeError, ValueError) as exc:
            raise BinaryPhaseSensorTransportError(
                "Binary detector timestamp is malformed."
            ) from exc
        if not np.isfinite(timestamp):
            raise BinaryPhaseSensorTransportError(
                "Binary detector timestamp must be finite."
            )
        sample["time_s"] = timestamp
        for field in ("left_heel_contact", "left_toe_contact"):
            if type(sample[field]) is not bool:
                raise BinaryPhaseSensorTransportError(
                    f"Binary detector field {field!r} must be a Python bool."
                )
        diagnostics = reading.diagnostics()
        json.dumps(diagnostics, allow_nan=False)
        self._last_step_info["binary_phase_sensor_diagnostics"] = diagnostics
        return sample

    def _append_binary_phase_sensor_sample(
        self,
        state: opensim.State,
        time_s: float,
        samples: list[dict[str, float | bool]],
    ) -> None:
        """Append one raw binary sample on the reset-anchored 1 ms lattice."""
        if not self._binary_phase_sensor_sampling_is_enabled():
            return
        try:
            current = float(time_s)
        except (TypeError, ValueError) as exc:
            raise BinaryPhaseSensorTransportError(
                "Binary detector sample timestamp must be numeric."
            ) from exc
        if not np.isfinite(current):
            raise BinaryPhaseSensorTransportError(
                "Binary detector sample timestamp must be finite."
            )
        previous = self._binary_phase_sensor_last_sample_time_s
        if previous is None:
            raise BinaryPhaseSensorTransportError(
                "Binary detector sampling baseline is missing; "
                "reset_to_time() must establish t0 first."
            )
        dt = float(self._binary_phase_sensor_sample_dt_s)
        tolerance = max(1e-12, dt * 1e-7)
        delta = current - float(previous)
        if delta < -tolerance:
            raise BinaryPhaseSensorTransportError(
                "Binary detector sample timestamps are non-monotonic: "
                f"{current:.12g} follows {float(previous):.12g}."
            )
        if abs(delta) <= tolerance:
            raise BinaryPhaseSensorTransportError(
                f"Duplicate binary detector timestamp {current:.12g}."
            )

        expected = float(previous) + dt
        if current < expected - tolerance:
            return
        if current > expected + tolerance:
            raise BinaryPhaseSensorTransportError(
                "Missing binary detector sample: expected "
                f"{expected:.12g}, reached {current:.12g}."
            )

        sample = self._sample_binary_phase_detector_channels(state, expected)
        try:
            sample_time = float(sample["time_s"])
        except (KeyError, TypeError, ValueError) as exc:
            raise BinaryPhaseSensorTransportError(
                "Binary detector sample timestamp is missing or malformed."
            ) from exc
        if not np.isfinite(sample_time):
            raise BinaryPhaseSensorTransportError(
                "Binary detector sample timestamp must be finite."
            )
        sample["time_s"] = sample_time
        for field in ("left_heel_contact", "left_toe_contact"):
            if field not in sample or type(sample[field]) is not bool:
                raise BinaryPhaseSensorTransportError(
                    f"Binary detector field {field!r} must be a Python bool."
                )
        if set(sample) != {
            "time_s",
            "left_heel_contact",
            "left_toe_contact",
        }:
            raise BinaryPhaseSensorTransportError(
                "Binary detector sample contains unexpected fields."
            )
        if samples and sample_time <= float(samples[-1]["time_s"]) + tolerance:
            raise BinaryPhaseSensorTransportError(
                "Binary detector segment contains duplicate or "
                "non-monotonic samples."
            )
        samples.append(sample)
        self._binary_phase_sensor_last_sample_time_s = expected

    def _finalize_binary_phase_sensor_segment(
        self,
        *,
        segment_start_time_s: float,
        t_stop: float,
        samples: list[dict[str, float | bool]],
    ) -> None:
        """Fail closed before publishing one open-left/closed-right bit batch."""
        if not self._binary_phase_sensor_sampling_is_enabled():
            if samples:
                raise BinaryPhaseSensorTransportError(
                    "Binary detector samples exist without a configured profile."
                )
            return
        start = float(segment_start_time_s)
        stop = float(t_stop)
        dt = float(self._binary_phase_sensor_sample_dt_s)
        tolerance = max(1e-12, dt * 1e-7)
        if not np.isfinite(start) or not np.isfinite(stop):
            raise BinaryPhaseSensorTransportError(
                "Binary detector segment boundaries must be finite."
            )
        duration = stop - start
        if duration < -tolerance:
            raise BinaryPhaseSensorTransportError(
                "Binary detector segment end precedes its start."
            )
        if abs(duration) <= tolerance:
            if samples:
                raise BinaryPhaseSensorTransportError(
                    "A zero-duration binary detector segment must be empty."
                )
            return

        count_float = duration / dt
        expected_count = int(round(count_float))
        if expected_count <= 0 or abs(count_float - expected_count) > 1e-7:
            raise BinaryPhaseSensorTransportError(
                "Binary detector segment is not aligned to "
                "detector_sample_dt_s."
            )
        if len(samples) != expected_count:
            raise BinaryPhaseSensorTransportError(
                "Binary detector segment is incomplete: expected "
                f"{expected_count} samples, got {len(samples)}."
            )
        exact_fields = {
            "time_s",
            "left_heel_contact",
            "left_toe_contact",
        }
        for index, sample in enumerate(samples, start=1):
            if set(sample) != exact_fields:
                raise BinaryPhaseSensorTransportError(
                    "Binary detector segment contains a malformed sample."
                )
            try:
                sample_time = float(sample["time_s"])
            except (TypeError, ValueError) as exc:
                raise BinaryPhaseSensorTransportError(
                    "Binary detector segment contains a malformed timestamp."
                ) from exc
            if not np.isfinite(sample_time):
                raise BinaryPhaseSensorTransportError(
                    "Binary detector segment contains a non-finite timestamp."
                )
            if (
                type(sample["left_heel_contact"]) is not bool
                or type(sample["left_toe_contact"]) is not bool
            ):
                raise BinaryPhaseSensorTransportError(
                    "Binary detector segment contacts must be Python bools."
                )
            expected_time = start + index * dt
            if abs(sample_time - expected_time) > tolerance:
                raise BinaryPhaseSensorTransportError(
                    "Binary detector segment contains a duplicate, missing, "
                    "non-monotonic, or off-grid timestamp."
                )
        if (
            abs(float(samples[0]["time_s"]) - (start + dt)) > tolerance
            or abs(float(samples[-1]["time_s"]) - stop) > tolerance
        ):
            raise BinaryPhaseSensorTransportError(
                "Binary detector segment violates (previous_time, t_stop] "
                "boundaries."
            )
        if (
            self._binary_phase_sensor_last_sample_time_s is None
            or abs(self._binary_phase_sensor_last_sample_time_s - stop)
            > tolerance
        ):
            raise BinaryPhaseSensorTransportError(
                "Binary detector endpoint was not sampled exactly once."
            )

    def _sample_online_grf(
        self,
        state: opensim.State,
        t: float,
    ) -> tuple[dict | None, list[dict]]:
        """Read calculated GRF, validate active contacts, and detect events."""
        ctx = self._ctx
        force_paths = getattr(ctx, "online_grf_force_paths", [])
        if not force_paths:
            return None, []

        grf = read_online_grf(
            ctx.model,
            state,
            force_paths,
            ctx.online_grf_force_sides,
        )
        vertical_forces = {}
        for side in ("left", "right"):
            item = grf["sides"][side]
            force = np.asarray(item["force"], dtype=float)
            scalar_values = np.asarray(
                [
                    item["normal_force"],
                    item["penetration"],
                    item["slip_speed"],
                ],
                dtype=float,
            )
            if not np.all(np.isfinite(force)) or not np.all(np.isfinite(scalar_values)):
                raise FloatingPointError(
                    f"Non-finite online GRF at t={t:.4f} for {side}."
                )
            vertical_forces[side] = float(force[1])

        detector_grf = None
        detector_paths = getattr(ctx, "online_grf_detector_force_paths", [])
        if detector_paths:
            detector_grf = read_online_grf(
                ctx.model,
                state,
                detector_paths,
                ctx.online_grf_detector_force_sides,
            )
            detector_event_sides = {
                str(side).strip().lower()
                for side in getattr(
                    ctx,
                    "online_grf_detector_force_sides",
                    {},
                ).values()
                if str(side).strip().lower() in {"left", "right"}
            }
            for side in ("left", "right"):
                item = detector_grf["sides"][side]
                force = np.asarray(item["force"], dtype=float)
                scalar_values = np.asarray(
                    [
                        item["normal_force"],
                        item["penetration"],
                        item["slip_speed"],
                    ],
                    dtype=float,
                )
                if not np.all(np.isfinite(force)) or not np.all(
                    np.isfinite(scalar_values)
                ):
                    raise FloatingPointError(
                        f"Non-finite detector online GRF at t={t:.4f} for {side}."
                    )
                # A detector profile may intentionally cover only one side
                # (for example the prosthetic left foot).  It owns event
                # detection only for configured sides; uncovered sides keep
                # their primary-GRF event signal instead of being zeroed by
                # the empty aggregate returned by ``read_online_grf``.
                if side in detector_event_sides:
                    vertical_forces[side] = float(force[1])

        grf_mode = str(getattr(ctx, "grf_mode", "prescribed")).strip().lower()
        if grf_mode == "online":
            force_applying_sides = {"left", "right"}
        elif grf_mode == "online_sensor":
            force_applying_sides = {
                str(side).strip().lower()
                for side in getattr(ctx, "online_grf_applied_sides", [])
                if str(side).strip().lower() in {"left", "right"}
            }
        else:
            force_applying_sides = set()

        if force_applying_sides:
            mass = float(ctx.model.getTotalMass(state))
            max_force = (
                float(getattr(self._cfg, "online_grf_max_force_bw", 5.0))
                * mass
                * 9.80665
            )
            side_forces = [
                np.asarray(grf["sides"][side]["force"], dtype=float)
                for side in sorted(force_applying_sides)
            ]
            peak = max(
                *(float(np.linalg.norm(force)) for force in side_forces),
                float(np.linalg.norm(np.sum(side_forces, axis=0))),
            )
            if peak > max_force:
                raise FloatingPointError(
                    f"Online GRF exceeds {self._cfg.online_grf_max_force_bw:g} BW "
                    f"at t={t:.4f}: {peak:.1f} N > {max_force:.1f} N."
                )
            max_penetration = float(
                getattr(self._cfg, "online_grf_max_penetration_m", 0.03)
            )
            for side in sorted(force_applying_sides):
                penetration = float(grf["sides"][side]["penetration"])
                if penetration > max_penetration:
                    raise FloatingPointError(
                        f"Online GRF penetration exceeds {max_penetration:g} m "
                        f"at t={t:.4f} for {side}: {penetration:.6f} m."
                    )

        events = []
        if self._online_event_detector is not None:
            events = self._online_event_detector.update(t, vertical_forces)
        if detector_grf is not None:
            self._last_step_info["online_grf_detector"] = self._online_grf_info(
                detector_grf
            )
        return grf, events

    def _integrate_evaluate(
        self,
        state:    opensim.State,
        controls: opensim.Vector,
        t:        float,
    ) -> tuple:
        """
        Compute q̈ via bypass and read SEA plugin outputs at the current state.

        Called once per integration substep with held-constant `controls`.
        Re-evaluates the dynamics so the integrator sees the true response
        of the model at the current state (rather than reusing the udot
        computed at window start).
        """
        cfg   = self._cfg
        ctx   = self._ctx
        model = ctx.model

        model.realizeVelocity(state)
        model.setControls(state, controls)

        udot = compute_udot_bypass(
            self._matter, model, state, self._n_mob,
            self._e_vec, self._Me_vec,
        )
        if cfg.sea_forward_mode == "plugin":
            sea_plugin_outputs = self._sea_plugin_outputs_from_outputs(state)
            sea_derivatives = self._sea_integrated_state_derivatives_from_outputs(
                state
            )
        else:
            sea_plugin_outputs = np.full(len(self._sea_pros_map) * 3, np.nan)
            sea_derivatives = np.full(
                len(self._sea_integrated_state_items), np.nan
            )

        if not np.all(np.isfinite(udot)):
            bad_coords = [
                name for name in ctx.coord_names
                if not np.isfinite(udot[ctx.coord_mob_idx[name]])
            ]
            raise FloatingPointError(
                f"Non-finite accelerations at t={t:.4f}: {bad_coords[:5]}"
            )

        return udot, sea_plugin_outputs, sea_derivatives

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
        pros_knee_idx = pros_ankle_idx = None
        pros_knee_speed_idx = pros_ankle_speed_idx = None
        for sv_idx, sv_name in enumerate(sv_names):
            if sv_name.endswith("SEA_Knee/motor_angle"):
                pros_knee_idx = sv_idx
            elif sv_name.endswith("SEA_Ankle/motor_angle"):
                pros_ankle_idx = sv_idx
            elif sv_name.endswith("SEA_Knee/motor_speed"):
                pros_knee_speed_idx = sv_idx
            elif sv_name.endswith("SEA_Ankle/motor_speed"):
                pros_ankle_speed_idx = sv_idx

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
        if hasattr(self, "_qdot0_for_sea_init"):
            qdot0_sea = self._qdot0_for_sea_init
            if pros_knee_speed_idx is not None:
                sv.set(
                    pros_knee_speed_idx,
                    qdot0_sea.get(self._cfg.pros_coords[0], 0.0),
                )
            if pros_ankle_speed_idx is not None:
                sv.set(
                    pros_ankle_speed_idx,
                    qdot0_sea.get(self._cfg.pros_coords[1], 0.0),
                )

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

    def _component_output_float(
        self,
        state: opensim.State,
        component_path: str,
        output_name: str,
        required: bool = True,
    ) -> float:
        """Read a scalar component output without entering Acceleration stage."""
        try:
            component = self._ctx.model.getComponent(component_path)
            output = component.getOutput(output_name)
            return float(output.getValueAsString(state))
        except Exception as exc:
            if not required:
                return float("nan")
            raise RuntimeError(
                f"Could not read output '{output_name}' from "
                f"'{component_path}'"
            ) from exc

    def _sea_runtime_diagnostics(
        self,
        state: opensim.State,
        u_sea: Dict[str, float],
        sea_plugin_outputs: np.ndarray,
    ) -> dict[str, dict[str, float]]:
        """Return physical SEA diagnostics for one integration substep."""
        ctx = self._ctx
        sv = ctx.model.getStateVariableValues(state)
        coord_set = ctx.model.getCoordinateSet()
        diagnostics: dict[str, dict[str, float]] = {}
        for i, (sea_name, coord_name) in enumerate(self._sea_pros_map):
            props = self._sea_props[sea_name]
            K = float(props["K"])
            Kp = float(props["Kp"])
            Kd = float(props["Kd"])
            Ki = float(props.get("Ki", 0.0))
            integral_limit = max(
                0.0,
                float(props.get("integral_torque_limit", 100.0)),
            )
            Bm = float(props["Bm"])
            Jm = max(1e-9, float(props["Jm"]))
            F_opt = float(props["F_opt"])

            coord = coord_set.get(coord_name)
            theta_j = float(coord.getValue(state))
            omega_j = float(coord.getSpeedValue(state))
            theta_m = float(sv.get(ctx.sea_motor_angle_sv_idx[sea_name]))
            ms_idx = ctx.sea_motor_speed_sv_idx.get(sea_name)
            omega_m = float(sv.get(ms_idx)) if ms_idx is not None else 0.0
            tau_spring = K * (theta_m - theta_j)
            tau_ref = float(u_sea.get(coord_name, 0.0)) * F_opt

            if bool(props.get("impedance", False)):
                theta_m_ref = theta_j + tau_ref / max(abs(K), 1e-9)
                tau_input_raw = (
                    tau_spring
                    + Bm * omega_m
                    + Kp * (theta_m_ref - theta_m)
                    + Kd * (omega_j - omega_m)
                )
            else:
                xi_idx = ctx.sea_state_sv_idx.get(sea_name, {}).get(
                    "torque_error_integral"
                )
                xi = float(sv.get(xi_idx)) if xi_idx is not None else 0.0
                integral_torque = float(
                    np.clip(Ki * xi, -integral_limit, integral_limit)
                )
                tau_input_raw = (
                    tau_ref
                    + Kp * (tau_ref - tau_spring)
                    + integral_torque
                    - Kd * omega_m
                )

            base = i * 3
            tau_input_from_plugin = bool(
                len(sea_plugin_outputs) > base
                and np.isfinite(sea_plugin_outputs[base])
            )
            tau_input = (
                float(sea_plugin_outputs[base])
                if tau_input_from_plugin
                else float(np.clip(tau_input_raw, -500.0, 500.0))
            )
            motor_accel_from_plugin = bool(
                len(sea_plugin_outputs) > base + 2
                and np.isfinite(sea_plugin_outputs[base + 2])
            )
            motor_accel = (
                float(sea_plugin_outputs[base + 2])
                if motor_accel_from_plugin
                else (tau_input - tau_spring - Bm * omega_m) / Jm
            )
            diagnostics[coord_name] = {
                "time_s": float(state.getTime()),
                "tau_spring_nm": float(tau_spring),
                "tau_input_raw_nm": float(tau_input_raw),
                "tau_input_nm": float(tau_input),
                "tau_input_saturated": float(abs(tau_input) >= 500.0 - 1e-9),
                "torque_error_nm": float(tau_ref - tau_spring),
                "motor_speed_rad_s": float(omega_m),
                "motor_accel_rad_s2": float(motor_accel),
                "joint_power_w": float(tau_spring * omega_j),
                "motor_power_w": float(tau_input * omega_m),
                "tau_input_plugin_fallback": float(
                    not tau_input_from_plugin
                ),
                "motor_accel_plugin_fallback": float(
                    not motor_accel_from_plugin
                ),
            }
        return diagnostics

    @staticmethod
    def _summarize_sea_segment_diagnostics(
        samples: list[dict[str, dict[str, float]]],
    ) -> dict:
        """Reduce substep SEA samples to compact segment-level RMS/max metrics."""
        if not samples:
            return {"sample_count": 0, "joints": {}}
        coord_names = sorted(
            {
                coord_name
                for sample in samples
                for coord_name in sample
            }
        )
        joints: dict[str, dict[str, float]] = {}
        for coord_name in coord_names:
            rows = [
                sample[coord_name]
                for sample in samples
                if coord_name in sample
            ]
            if not rows:
                continue

            def values(key: str) -> np.ndarray:
                return np.asarray([float(row[key]) for row in rows], dtype=float)

            time_s = values("time_s")
            tau_spring = values("tau_spring_nm")
            raw = values("tau_input_raw_nm")
            tau_input = values("tau_input_nm")
            saturated = values("tau_input_saturated")
            torque_error = values("torque_error_nm")
            motor_speed = values("motor_speed_rad_s")
            motor_accel = values("motor_accel_rad_s2")
            joint_power = values("joint_power_w")
            motor_power = values("motor_power_w")
            tau_input_fallback = values("tau_input_plugin_fallback")
            motor_accel_fallback = values("motor_accel_plugin_fallback")

            def rms(data: np.ndarray) -> float:
                if data.size == 0:
                    return 0.0
                return float(np.sqrt(np.mean(np.square(data))))

            dt = np.diff(time_s)
            tau_spring_diff = np.diff(tau_spring)
            valid_dt = dt > 1e-12
            tau_spring_rate = (
                tau_spring_diff[valid_dt] / dt[valid_dt]
                if np.any(valid_dt)
                else np.asarray([], dtype=float)
            )

            joints[coord_name] = {
                "sample_count": float(len(rows)),
                # Sufficient statistics are published so zero-update validation
                # can form episode-level RMS/count values without reconstructing
                # or approximating the simulator substep trace.
                "tau_spring_sum_squares_nm2": float(
                    np.sum(np.square(tau_spring))
                ),
                "tau_spring_rms_nm": rms(tau_spring),
                "tau_spring_abs_max_nm": float(np.max(np.abs(tau_spring))),
                "tau_spring_rate_sample_count": float(tau_spring_rate.size),
                "tau_spring_rate_sum_squares_nm2_s2": float(
                    np.sum(np.square(tau_spring_rate))
                ),
                "tau_spring_rate_rms_nm_s": rms(tau_spring_rate),
                "tau_spring_rate_abs_max_nm_s": (
                    float(np.max(np.abs(tau_spring_rate)))
                    if tau_spring_rate.size
                    else 0.0
                ),
                "tau_input_raw_rms_nm": rms(raw),
                "tau_input_raw_abs_max_nm": float(np.max(np.abs(raw))),
                "tau_input_abs_max_nm": float(np.max(np.abs(tau_input))),
                "tau_input_saturation_count": int(np.count_nonzero(saturated)),
                "tau_input_saturation_fraction": float(np.mean(saturated)),
                "torque_error_sum_squares_nm2": float(
                    np.sum(np.square(torque_error))
                ),
                "torque_error_rms_nm": rms(torque_error),
                "torque_error_abs_max_nm": float(
                    np.max(np.abs(torque_error))
                ),
                "motor_speed_sum_squares_rad2_s2": float(
                    np.sum(np.square(motor_speed))
                ),
                "motor_speed_rms_rad_s": rms(motor_speed),
                "motor_speed_abs_max_rad_s": float(np.max(np.abs(motor_speed))),
                "motor_accel_sum_squares_rad2_s4": float(
                    np.sum(np.square(motor_accel))
                ),
                "motor_accel_rms_rad_s2": rms(motor_accel),
                "motor_accel_abs_max_rad_s2": float(np.max(np.abs(motor_accel))),
                "joint_power_rms_w": rms(joint_power),
                "joint_power_mean_w": float(np.mean(joint_power)),
                "joint_power_positive_mean_w": float(
                    np.mean(np.maximum(joint_power, 0.0))
                ),
                "joint_power_negative_mean_w": float(
                    np.mean(np.minimum(joint_power, 0.0))
                ),
                "motor_power_sum_squares_w2": float(
                    np.sum(np.square(motor_power))
                ),
                "motor_power_rms_w": rms(motor_power),
                "motor_power_abs_max_w": float(np.max(np.abs(motor_power))),
                "tau_input_plugin_fallback_count": int(
                    np.count_nonzero(tau_input_fallback)
                ),
                "motor_accel_plugin_fallback_count": int(
                    np.count_nonzero(motor_accel_fallback)
                ),
            }
        return {"sample_count": int(len(samples)), "joints": joints}

    def _sea_plugin_outputs_from_outputs(self, state: opensim.State) -> np.ndarray:
        """Return tau_input and motor derivatives exposed by the SEA plugin."""
        values = np.full(len(self._sea_pros_map) * 3, np.nan)
        for i, (sea_name, _coord_name) in enumerate(self._sea_pros_map):
            component_path = f"/forceset/{sea_name}"
            base = i * 3
            values[base] = self._component_output_float(
                state, component_path, "tau_input", required=False
            )
            values[base + 1] = self._component_output_float(
                state, component_path, "motor_angle_dot"
            )
            values[base + 2] = self._component_output_float(
                state, component_path, "motor_speed_dot"
            )

        derivative_values = np.ravel(
            [[values[i * 3 + 1], values[i * 3 + 2]]
             for i in range(len(self._sea_pros_map))]
        )
        if not np.all(np.isfinite(derivative_values)):
            raise FloatingPointError(
                "Non-finite SEA plugin output derivatives: "
                f"{values.tolist()}"
            )
        return values

    def _sea_derivatives_from_plugin_outputs(
        self,
        sea_plugin_outputs: np.ndarray,
    ) -> np.ndarray:
        """Extract motor_angle_dot and motor_speed_dot from plugin outputs."""
        values = np.full(len(self._sea_pros_map) * 2, np.nan)
        for i in range(len(self._sea_pros_map)):
            values[i * 2] = sea_plugin_outputs[i * 3 + 1]
            values[i * 2 + 1] = sea_plugin_outputs[i * 3 + 2]
        return values

    def _sea_state_derivatives_from_outputs(self, state: opensim.State) -> np.ndarray:
        """Return motor_angle_dot and motor_speed_dot exposed by the SEA plugin."""
        return self._sea_derivatives_from_plugin_outputs(
            self._sea_plugin_outputs_from_outputs(state)
        )

    def _sea_integrated_state_derivatives_from_outputs(
        self,
        state: opensim.State,
    ) -> np.ndarray:
        """Return derivatives for every SEA state integrated by the bypass."""
        values = np.zeros(len(self._sea_integrated_state_items), dtype=float)
        for i, (sea_name, _state_name, _sv_idx, output_name) in enumerate(
            self._sea_integrated_state_items
        ):
            component_path = f"/forceset/{sea_name}"
            values[i] = self._component_output_float(
                state, component_path, output_name
            )
        if not np.all(np.isfinite(values)):
            raise FloatingPointError(
                "Non-finite SEA integrated state derivatives: "
                f"{values.tolist()}"
            )
        return values

    def _advance_plugin_state_substeps(
        self,
        state: opensim.State,
        controls: opensim.Vector,
        udot_step: np.ndarray,
        dt: float,
    ) -> float:
        """
        Advance coordinates once while substepping SEA motor states.

        The CMC-like controller and SO solve produce one acceleration vector for
        the step. Recomputing biological accelerations inside SEA substeps would
        reuse stale controls in states they were not optimized for, which can
        create artificial drift in lightly constrained coordinates. The SEA
        motor state still uses the C++ plugin derivatives at every substep.
        """
        start_time, start_values = self._snapshot_state_variables(state)
        requested = max(1, int(getattr(self._cfg, "sea_motor_substeps", 1)))
        max_substeps = max(
            requested,
            int(getattr(self._cfg, "sea_motor_max_substeps", requested)),
        )

        substeps = requested
        last_error: Exception | None = None
        while substeps <= max_substeps:
            self._restore_state_variables(state, start_time, start_values)
            try:
                return self._advance_plugin_state_fixed_substeps(
                    state, controls, udot_step, dt, substeps
                )
            except FloatingPointError as exc:
                last_error = exc
                if substeps >= max_substeps:
                    break
                next_substeps = min(max_substeps, substeps * 2)
                print(
                    "[Runner] Plugin substep retry at "
                    f"t={start_time:.4f}: {substeps} -> {next_substeps} "
                    f"substeps ({exc})",
                    flush=True,
                )
                substeps = next_substeps

        self._restore_state_variables(state, start_time, start_values)
        raise FloatingPointError(
            "Plugin forward step did not remain finite at "
            f"t={start_time:.4f} even with {substeps} substeps. "
            f"Last error: {last_error}"
        ) from last_error

    def _advance_rk4_bypass_state(
        self,
        state: opensim.State,
        controls: opensim.Vector,
        dt: float,
    ) -> float:
        """
        Advance coordinates and SEA motor states with explicit RK4.

        All acceleration evaluations use compute_udot_bypass, so this keeps the
        native Acceleration stage out of the path while reducing the Euler
        single-step error that can amplify infeasible high-gain corrections.
        """
        ctx = self._ctx
        model = ctx.model
        coord_set = model.getCoordinateSet()
        n_coords = len(ctx.coord_names)
        n_plugin_states = len(self._sea_integrated_state_items)

        start_time, start_values = self._snapshot_state_variables(state)

        y0 = np.zeros(n_coords * 2 + n_plugin_states)
        for i, name in enumerate(ctx.coord_names):
            coord = coord_set.get(name)
            y0[i] = coord.getValue(state)
            y0[n_coords + i] = coord.getSpeedValue(state)

        sv = model.getStateVariableValues(state)
        sea_base = n_coords * 2
        for i, (_sea_name, _state_name, sv_idx, _output_name) in enumerate(
            self._sea_integrated_state_items
        ):
            y0[sea_base + i] = sv.get(sv_idx)

        def write_stage(y_values: np.ndarray, time_value: float) -> None:
            q_stage = {
                name: float(y_values[i])
                for i, name in enumerate(ctx.coord_names)
            }
            qdot_stage = {
                name: float(y_values[n_coords + i])
                for i, name in enumerate(ctx.coord_names)
            }
            self._set_state(state, q_stage, qdot_stage, time_value)

            sea_values: Dict[int, float] = {}
            for i, (_sea_name, _state_name, sv_idx, _output_name) in enumerate(
                self._sea_integrated_state_items
            ):
                sea_values[sv_idx] = float(y_values[sea_base + i])
            self._set_sea_state_values(state, sea_values)

        def rhs(y_values: np.ndarray, time_value: float) -> np.ndarray:
            if not np.all(np.isfinite(y_values)):
                raise FloatingPointError(
                    f"Non-finite RK4 stage state at t={time_value:.4f}"
                )
            self._restore_state_variables(state, start_time, start_values)
            write_stage(y_values, time_value)

            model.realizeVelocity(state)
            model.setControls(state, controls)
            udot = compute_udot_bypass(
                self._matter, model, state, self._n_mob,
                self._e_vec, self._Me_vec,
            )
            if not np.all(np.isfinite(udot)):
                bad_coords = [
                    name for name in ctx.coord_names
                    if not np.isfinite(udot[ctx.coord_mob_idx[name]])
                ]
                raise FloatingPointError(
                    "Non-finite RK4 accelerations at "
                    f"t={time_value:.4f}: {bad_coords[:5]}"
                )

            if self._cfg.sea_forward_mode == "plugin":
                sea_derivatives = self._sea_integrated_state_derivatives_from_outputs(
                    state
                )
            else:
                sea_derivatives = np.zeros(n_plugin_states)

            dy = np.zeros_like(y_values)
            dy[:n_coords] = y_values[n_coords:n_coords * 2]
            for i, name in enumerate(ctx.coord_names):
                dy[n_coords + i] = udot[ctx.coord_mob_idx[name]]
            dy[sea_base:sea_base + n_plugin_states] = sea_derivatives[
                :n_plugin_states
            ]

            if not np.all(np.isfinite(dy)):
                raise FloatingPointError(
                    f"Non-finite RK4 derivative at t={time_value:.4f}"
                )
            return dy

        h = float(dt)
        k1 = rhs(y0, start_time)
        k2 = rhs(y0 + 0.5 * h * k1, start_time + 0.5 * h)
        k3 = rhs(y0 + 0.5 * h * k2, start_time + 0.5 * h)
        k4 = rhs(y0 + h * k3, start_time + h)
        y_next = y0 + (h / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)

        if not np.all(np.isfinite(y_next)):
            self._restore_state_variables(state, start_time, start_values)
            raise FloatingPointError(
                f"Non-finite RK4 state update at t={start_time:.4f}"
            )

        self._restore_state_variables(state, start_time, start_values)
        write_stage(y_next, start_time + h)
        model.realizeVelocity(state)
        return float(state.getTime())

    def _advance_plugin_state_fixed_substeps(
        self,
        state: opensim.State,
        controls: opensim.Vector,
        udot_step: np.ndarray,
        dt: float,
        substeps: int,
    ) -> float:
        """Advance one main step with an already chosen substep count."""
        ctx = self._ctx
        model = self._ctx.model
        coord_set = model.getCoordinateSet()

        h = dt / substeps

        for substep in range(substeps):
            model.realizeVelocity(state)
            model.setControls(state, controls)
            model.realizeDynamics(state)

            sea_derivatives = self._sea_state_derivatives_from_outputs(state)
            sea_integrated_derivatives = (
                self._sea_integrated_state_derivatives_from_outputs(state)
            )

            q_new: Dict[str, float] = {}
            qdot_new: Dict[str, float] = {}
            for name in ctx.coord_names:
                coord = coord_set.get(name)
                q_cur = coord.getValue(state)
                qdot_cur = coord.getSpeedValue(state)
                qacc_cur = udot_step[ctx.coord_mob_idx[name]]

                qdot_next = qdot_cur + qacc_cur * h
                q_new[name] = q_cur + qdot_next * h
                qdot_new[name] = qdot_next

            sv = model.getStateVariableValues(state)
            sea_next: Dict[int, float] = {}
            for (
                deriv_idx,
                (_sea_name, state_name, sv_idx, _output_name),
            ) in enumerate(self._sea_integrated_state_items):
                if state_name in {"motor_angle", "motor_speed"}:
                    continue
                value_next = sv.get(sv_idx) + sea_integrated_derivatives[deriv_idx] * h
                if not np.isfinite(value_next):
                    raise FloatingPointError(
                        "Non-finite SEA state integration during plugin "
                        f"substep {substep + 1}/{substeps} for "
                        f"{_sea_name}/{state_name}"
                    )
                sea_next[sv_idx] = float(value_next)

            for i, (sea_name, _coord_name) in enumerate(self._sea_pros_map):
                ma_idx = ctx.sea_motor_angle_sv_idx.get(sea_name)
                ms_idx = ctx.sea_motor_speed_sv_idx.get(sea_name)
                if ma_idx is None or ms_idx is None:
                    continue

                omega_next = sv.get(ms_idx) + sea_derivatives[i * 2 + 1] * h
                theta_next = sv.get(ma_idx) + omega_next * h
                if not (np.isfinite(theta_next) and np.isfinite(omega_next)):
                    raise FloatingPointError(
                        "Non-finite SEA motor integration during plugin "
                        f"substep {substep + 1}/{substeps} for {sea_name}"
                    )
                sea_next[ms_idx] = float(omega_next)
                sea_next[ma_idx] = float(theta_next)

            t_next = float(state.getTime()) + h
            self._set_state(state, q_new, qdot_new, t_next)
            self._set_sea_state_values(state, sea_next)

        return float(state.getTime())

    def _snapshot_state_variables(
        self,
        state: opensim.State,
    ) -> tuple[float, list[float]]:
        """Capture time and all state variables before a retryable step."""
        model = self._ctx.model
        sv = model.getStateVariableValues(state)
        n_sv = model.getStateVariableNames().getSize()
        values = [float(sv.get(i)) for i in range(n_sv)]
        return float(state.getTime()), values

    def _restore_state_variables(
        self,
        state: opensim.State,
        time_value: float,
        values: list[float],
    ) -> None:
        """Restore a state snapshot after a failed plugin substep attempt."""
        model = self._ctx.model
        sv = model.getStateVariableValues(state)
        for i, value in enumerate(values):
            sv.set(i, float(value))
        model.setStateVariableValues(state, sv)
        state.setTime(float(time_value))

    def _write_progress_heartbeat(
        self,
        phase: str,
        t_value: float,
        step: int,
        wall_time: float,
    ) -> None:
        """Publish atomic progress when supervised through CMC_SIM_HEARTBEAT_FILE."""
        raw_path = os.environ.get("CMC_SIM_HEARTBEAT_FILE", "").strip()
        if not raw_path:
            return
        path = os.path.abspath(raw_path)
        os.makedirs(os.path.dirname(path), exist_ok=True)
        now = _time.time()
        duration = max(0.0, float(self._cfg.t_end - self._cfg.t_start))
        progress = (
            100.0 * (float(t_value) - float(self._cfg.t_start)) / duration
            if duration > 0.0
            else 100.0
        )
        payload = {
            "phase": str(phase),
            "progress": max(0.0, min(100.0, progress)),
            "timeout_s": 0.0,
            "started_at_unix_s": now,
            "updated_at_unix_s": now,
            "pid": os.getpid(),
            "simulation_time_s": float(t_value),
            "step": int(step),
            "wall_time_s": float(wall_time),
        }
        temporary = f"{path}.{os.getpid()}.tmp"
        with open(temporary, "w", encoding="utf-8") as fh:
            json.dump(payload, fh, indent=2)
        os.replace(temporary, path)

    def _write_run_status(
        self,
        failure_exc: Exception | None,
        t_value: float,
        step: int,
        wall_time: float,
    ) -> None:
        """Write a status file so partial outputs are never ambiguous."""
        status_path = os.path.join(
            self._cfg.output_dir,
            f"{self._cfg.output_prefix}_run_status.txt",
        )
        ok = failure_exc is None
        with open(status_path, "w", encoding="utf-8") as fh:
            fh.write(f"status={'complete' if ok else 'failed'}\n")
            fh.write(f"t={float(t_value):.10g}\n")
            fh.write(f"step={int(step)}\n")
            fh.write(f"wall_time_s={float(wall_time):.10g}\n")
            fh.write(f"t_start={float(self._cfg.t_start):.10g}\n")
            fh.write(f"t_end={float(self._cfg.t_end):.10g}\n")
            fh.write(f"dt={float(self._cfg.dt):.10g}\n")
            fh.write(
                f"use_control_window="
                f"{bool(getattr(self._cfg, 'use_control_window', False))}\n"
            )
            fh.write(
                f"T_control="
                f"{float(getattr(self._cfg, 'T_control', self._cfg.dt)):.10g}\n"
            )
            fh.write(
                f"integration_dt="
                f"{float(getattr(self._cfg, 'integration_dt', self._cfg.dt)):.10g}\n"
            )
            fh.write(
                f"integration_scheme="
                f"{getattr(self._cfg, 'integration_scheme', 'semi_implicit_euler')}\n"
            )
            fh.write(f"sea_forward_mode={self._cfg.sea_forward_mode}\n")
            fh.write(
                f"grf_mode={getattr(self._ctx, 'grf_mode', 'prescribed')}\n"
            )
            disabled_sides = getattr(
                self._ctx, "prescribed_grf_disabled_sides", []
            )
            fh.write(
                "prescribed_grf_disabled_sides="
                + ",".join(sorted(str(side) for side in disabled_sides))
                + "\n"
            )
            applied_sides = getattr(self._ctx, "online_grf_applied_sides", [])
            fh.write(
                "online_grf_applied_sides="
                + ",".join(sorted(str(side) for side in applied_sides))
                + "\n"
            )
            online_profile = getattr(self._ctx, "online_grf_profile_file", "")
            if online_profile:
                fh.write(f"online_grf_profile_file={online_profile}\n")
            fh.write(
                f"online_grf_max_penetration_m="
                f"{float(getattr(self._cfg, 'online_grf_max_penetration_m', 0.03)):.10g}\n"
            )
            fh.write(f"sea_motor_substeps={self._cfg.sea_motor_substeps}\n")
            fh.write(
                f"sea_motor_max_substeps={self._cfg.sea_motor_max_substeps}\n"
            )
            fh.write(
                f"enable_grf_contact_filter="
                f"{bool(getattr(self._cfg, 'enable_grf_contact_filter', False))}\n"
            )
            fh.write(
                f"grf_contact_threshold_n="
                f"{float(getattr(self._cfg, 'grf_contact_threshold_n', 20.0)):.10g}\n"
            )
            if self._online_event_detector is not None:
                fh.write(
                    f"online_grf_hs_confirmation_threshold_n="
                    f"{self._online_event_detector.confirmation_threshold_n:.10g}\n"
                )
            fh.write(
                f"grf_min_contact_duration_s="
                f"{float(getattr(self._cfg, 'grf_min_contact_duration_s', 0.0)):.10g}\n"
            )
            grf_source = getattr(self._ctx, "grf_unfiltered_data_file", "")
            grf_loaded = getattr(self._ctx, "grf_data_file", "")
            grf_filter_report = getattr(self._ctx, "grf_filter_report_file", "")
            if grf_source:
                fh.write(f"grf_source_file={grf_source}\n")
            if grf_loaded:
                fh.write(f"grf_loaded_file={grf_loaded}\n")
            if grf_filter_report:
                fh.write(f"grf_filter_report={grf_filter_report}\n")
            if failure_exc is not None:
                fh.write(f"error_type={type(failure_exc).__name__}\n")
                fh.write(f"error={failure_exc}\n")
        print(f"  -> Run status  : {status_path}", flush=True)

    def _set_sea_state_values(
        self,
        state: opensim.State,
        sea_values: Dict[int, float] | None,
    ) -> None:
        """Write integrated SEA state variables back to the OpenSim State."""
        if not sea_values:
            return
        sv = self._ctx.model.getStateVariableValues(state)
        for sv_idx, value in sea_values.items():
            sv.set(sv_idx, float(value))
        self._ctx.model.setStateVariableValues(state, sv)

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
