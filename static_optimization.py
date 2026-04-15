"""
static_optimization.py
=======================
Frame-by-frame Static Optimisation (SO) using a Quadratic Program (QP).

Problem formulation
-------------------
Given the required bio-DOF generalised forces  τ_des  (from InverseDynamics),
find muscle activations  a ∈ [a_min, 1]^n_muscles  and reserve controls
u_res ∈ [−u_max, u_max]^n_reserves  that:

    minimise    Σ_i a_i²  +  w · Σ_j u_res_j²

    subject to  A_muscle · (a - a_min)
              + R_res · (u_res ⊙ F_opt)  =  τ_des
                a_min  ≤  a_i  ≤  1
               −u_max ≤ u_res_j ≤ u_max    (reserve control bounds)

Matrices
---------
A_muscle  : (n_bio × n_muscles) torque-per-activation map [N·m]
             built from Thelen equilibrium tendon-force changes at the
             current state. This replaces the old moment_arm * Fmax
             approximation, which was not consistent with the OpenSim muscle
             states used during forward dynamics.
R_res      : (n_bio × n_reserves) identity-like mapping  [N·m]
             R_res[i,j] = F_opt_j  if reserve j controls bio coord i, else 0
F_opt      : absorbed into R_res already

Solver choice
-------------
"slsqp"  → scipy.optimize.minimize (always available, no extra deps).
"osqp"   → qpsolvers with OSQP backend (faster, needs `pip install qpsolvers[osqp]`).

Warm-starting
-------------
The previous step's solution is used as x0 for SLSQP.  For dense gait data
(dt ≈ 10 ms) the solution changes little between frames, so the solver
converges in very few iterations after the first step.

Muscle mapping
--------------
Thelen2003Muscle force is not an algebraic function of excitation alone; the
instantaneous joint force is carried by tendon force, which depends on fiber
length state.  Therefore the optimizer prepares a quasi-static muscle baseline
at each frame, then estimates each muscle column by changing activation and
recomputing that muscle's equilibrium fiber length.  During forward dynamics
the selected muscle force is applied on the OpenSim muscle actuator through
actuation override; this keeps recruitment muscle-first without repeatedly
rewriting the simulated fiber state after the optimization solve.
Possible optimisations (not implemented here):
  - Cache R when the configuration changes slowly (check ‖Δq‖ < threshold).
  - Use finite-difference moment-arm approximation for speed.
"""

from __future__ import annotations

import warnings
from typing import Dict, List, Optional, Tuple

import numpy as np
from scipy.optimize import lsq_linear, minimize, OptimizeResult

import opensim

from config import SimulatorConfig
from model_loader import SimulationContext


class StaticOptimizer:
    """
    QP-based static optimisation, one frame at a time.

    Parameters
    ----------
    cfg : SimulatorConfig
    ctx : SimulationContext
    """

    def __init__(self, cfg: SimulatorConfig, ctx: SimulationContext) -> None:
        self._cfg = cfg
        self._ctx = ctx

        n_m  = ctx.n_muscles
        n_r  = ctx.n_reserves
        n_x  = n_m + n_r          # total decision variables

        self._activation_min = float(cfg.muscle_min_activation)
        self._activation_max = float(cfg.muscle_max_activation)
        self._activation_span = max(
            self._activation_max - self._activation_min,
            1e-8,
        )

        # ── Cost-function weight matrix ──────────────────────────────────────
        # P = diag([1,...,1, w,...,w])   (used in  (1/2) x^T P x)
        self._weights = np.ones(n_x)
        self._weights[:n_m] = cfg.muscle_activation_weight
        self._weights[n_m:] = cfg.reserve_weight

        # ── Variable bounds ──────────────────────────────────────────────────
        self._reserve_u_bounds = np.full(n_r, cfg.reserve_u_max, dtype=float)
        for j, bio_row in enumerate(ctx.reserve_bio_row):
            coord_name = ctx.bio_coord_names[bio_row]
            if any(
                coord_name.startswith(prefix)
                for prefix in cfg.unactuated_reserve_coord_prefixes
            ):
                self._reserve_u_bounds[j] = cfg.unactuated_reserve_u_max

        self._bounds: List[Tuple[float, float]] = (
            [(self._activation_min, self._activation_max)] * n_m
            + [
                (-float(u_max), float(u_max))
                for u_max in self._reserve_u_bounds
            ]
        )

        # ── Build constant part of the constraint matrix (reserve block) ────
        # R_res[i, j] = F_opt[j]  if reserve j controls bio coord i, else 0.
        # This block does NOT change with the model state.
        self._R_res_scaled = np.zeros((ctx.n_bio, n_r))
        for j, (bio_row, f_opt) in enumerate(
            zip(ctx.reserve_bio_row, ctx.reserve_f_opt)
        ):
            self._R_res_scaled[bio_row, j] = f_opt

        # ── Pre-fetch muscle & coordinate objects for moment-arm loop ────────
        muscle_set = ctx.model.getMuscles()
        coord_set  = ctx.model.getCoordinateSet()

        self._muscles: List[opensim.Muscle] = [
            muscle_set.get(name) for name in ctx.muscle_names
        ]
        self._bio_coords: List[opensim.Coordinate] = [
            coord_set.get(name) for name in ctx.bio_coord_names
        ]
        self._residual_reserve_rows = np.array([
            any(
                coord_name.startswith(prefix)
                for prefix in cfg.unactuated_reserve_coord_prefixes
            )
            for coord_name in ctx.bio_coord_names
        ], dtype=bool)

        # ── Warm-start: initial guess (uniform low activation) ───────────────
        self._x_prev = np.zeros(n_x)
        self._x_prev[:n_m] = self._activation_min

        self._last_muscle_matrix = np.zeros((ctx.n_bio, n_m))
        self._last_baseline_forces = np.zeros(n_m)
        self._last_force_gains = np.zeros(n_m)
        self._last_muscle_override_forces = np.zeros(n_m)
        self._last_equilibrium_failures = 0
        self._baseline_time: Optional[float] = None
        self._baseline_active = False
        self.last_diagnostics: Dict[str, float] = {}

        # ── Solver selection ─────────────────────────────────────────────────
        self._use_osqp = (cfg.qp_solver == "osqp")
        if self._use_osqp:
            try:
                import qpsolvers as _qp   # noqa: F401
            except ImportError:
                warnings.warn(
                    "qpsolvers not found; falling back to scipy SLSQP. "
                    "Install with: pip install qpsolvers[osqp]",
                    RuntimeWarning,
                )
                self._use_osqp = False

    # ─────────────────────────────────────────────────────────────────────────
    #  Public API
    # ─────────────────────────────────────────────────────────────────────────
    def solve(
        self,
        state:    opensim.State,
        tau_bio:  np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Solve the QP for the current state.

        Parameters
        ----------
        state   : OpenSim State realised to at least Stage::Velocity.
                  Muscle moment arms are computed via the model geometry,
                  which requires Velocity realisation.
        tau_bio : required generalised forces for bio coords  shape (n_bio,)

        Returns
        -------
        a       : muscle activations  shape (n_muscles,)  ∈ [a_min, a_max]
        u_res   : reserve controls    shape (n_reserves,) ∈ [−u_max, u_max]
        """
        # ── Build muscle/reserve constraint matrix ──────────────────────────
        A = self._build_constraint_matrix(state)
        A_muscle = A[:, :self._ctx.n_muscles]

        # The muscle columns describe torque from (activation - a_min), while
        # the optimizer variable is the absolute activation a. Move the
        # baseline term to the right-hand side for the linear constraint.
        if self._cfg.use_muscles_in_so:
            activation_floor = np.full(
                self._ctx.n_muscles,
                self._activation_min,
            )
            b = tau_bio + A_muscle @ activation_floor
        else:
            b = tau_bio

        # ── Dispatch to selected solver ──────────────────────────────────────
        if self._use_osqp:
            x, success = self._solve_osqp(A, b)
        else:
            x, success = self._solve_slsqp(A, b)

        if not success:
            warnings.warn(
                "[StaticOptimizer] QP did not converge. "
                "Using bounded least-squares fallback.",
                RuntimeWarning,
            )
            x = self._solve_bounded_least_squares(A, b)

        self._x_prev = x.copy()

        n_m = self._ctx.n_muscles
        activations = x[:n_m]
        u_res = x[n_m:]
        self._update_diagnostics(tau_bio, A_muscle, activations, u_res)
        return activations, u_res

    # ─────────────────────────────────────────────────────────────────────────
    #  Private helpers
    # ─────────────────────────────────────────────────────────────────────────
    def _build_constraint_matrix(self, state: opensim.State) -> np.ndarray:
        """
        Build the full constraint matrix  A = [R_bio_scaled | R_res_scaled]
        of shape (n_bio, n_muscles + n_reserves).

        The left block maps absolute muscle activation to torque around the
        per-frame minimum-activation baseline. The solve shifts that baseline
        onto the right-hand side so the actual muscle torque is
        A_muscle · (a - a_min).

        State must be ≥ Stage::Velocity for computeMomentArm to work.
        """
        ctx = self._ctx
        n_bio = ctx.n_bio
        n_m   = ctx.n_muscles

        R_muscle_scaled = np.zeros((n_bio, n_m))
        if self._cfg.use_muscles_in_so:
            R_muscle_scaled = self._build_equilibrium_muscle_matrix(state)
        self._last_muscle_matrix = R_muscle_scaled

        # -- Concatenate muscle and reserve blocks ---------------------------
        A = np.hstack([R_muscle_scaled, self._R_res_scaled])
        return A

    def prepare_muscle_baseline(self, state: opensim.State) -> None:
        """
        Put muscles in a minimum-activation equilibrium state for the current
        coordinates before inverse dynamics computes the zero-actuator baseline.
        """
        if not self._cfg.use_muscles_in_so:
            return

        current_time = float(state.getTime())
        if self._baseline_active and self._baseline_time == current_time:
            return

        model = self._ctx.model
        model.realizePosition(state)
        self._disable_muscle_actuation_overrides(state)
        failures = 0
        for muscle in self._muscles:
            muscle.setActivation(state, self._activation_min)
        for muscle in self._muscles:
            if not self._safe_compute_equilibrium(state, muscle):
                failures += 1

        model.realizeVelocity(state)
        self._last_equilibrium_failures = failures
        self._baseline_time = current_time
        self._baseline_active = True

    def apply_activations_to_state(
        self,
        state: opensim.State,
        activations: np.ndarray,
    ) -> None:
        """Write optimized activations and muscle actuation to the OpenSim state."""
        if not self._cfg.use_muscles_in_so:
            return

        for muscle, activation in zip(self._muscles, activations):
            bounded = float(np.clip(
                activation,
                self._activation_min,
                self._activation_max,
            ))
            muscle.setActivation(state, bounded)

        if self._cfg.muscle_force_application == "override_actuation":
            self._apply_muscle_actuation_overrides(state, activations)
        else:
            failures = 0
            for muscle in self._muscles:
                if not self._safe_compute_equilibrium(state, muscle):
                    failures += 1
            self._last_equilibrium_failures += failures

        self._ctx.model.realizeVelocity(state)
        self._baseline_active = False

    def _build_equilibrium_muscle_matrix(
        self,
        state: opensim.State,
    ) -> np.ndarray:
        """
        Build torque-per-activation columns from Thelen equilibrium tendon
        force, holding all other muscles at the minimum-activation baseline.
        """
        ctx = self._ctx
        model = ctx.model
        n_bio = ctx.n_bio
        n_m = ctx.n_muscles
        A_muscle = np.zeros((n_bio, n_m))

        self.prepare_muscle_baseline(state)
        model.realizeDynamics(state)

        baseline_fiber_lengths = np.zeros(n_m)
        baseline_forces = np.zeros(n_m)
        for j, muscle in enumerate(self._muscles):
            baseline_fiber_lengths[j] = self._safe_muscle_fiber_length(
                muscle, state
            )
            baseline_forces[j] = self._safe_muscle_actuation(muscle, state)

        failures = 0
        force_gains = np.zeros(n_m)
        for j, muscle in enumerate(self._muscles):
            muscle.setActivation(state, self._activation_max)
            if self._safe_compute_equilibrium(state, muscle):
                model.realizeDynamics(state)
                high_force = self._safe_muscle_actuation(muscle, state)
            else:
                high_force = baseline_forces[j]
                failures += 1

            force_gain = max(
                0.0,
                (high_force - baseline_forces[j]) / self._activation_span,
            )
            if np.isfinite(force_gain) and force_gain > 0.0:
                force_gains[j] = force_gain
                for i, coord in enumerate(self._bio_coords):
                    A_muscle[i, j] = muscle.computeMomentArm(state, coord) * force_gain

            self._set_muscle_state_values(
                state,
                muscle.getName(),
                activation=self._activation_min,
                fiber_length=baseline_fiber_lengths[j],
            )

        model.realizeVelocity(state)
        self._last_baseline_forces = baseline_forces
        self._last_force_gains = force_gains
        self._last_equilibrium_failures += failures
        self._baseline_active = True
        self._baseline_time = float(state.getTime())
        return A_muscle

    def _apply_muscle_actuation_overrides(
        self,
        state: opensim.State,
        activations: np.ndarray,
    ) -> None:
        """
        Apply the SO muscle contribution on OpenSim muscle actuators directly.

        The ID baseline already includes the minimum-activation equilibrium
        muscle force, so the override force is the baseline force plus the
        linearized increment selected by the optimizer.
        """
        delta = np.clip(
            activations - self._activation_min,
            0.0,
            self._activation_span,
        )
        forces = self._last_baseline_forces + self._last_force_gains * delta
        forces = np.where(np.isfinite(forces), forces, self._last_baseline_forces)
        forces = np.maximum(forces, 0.0)

        for muscle, force in zip(self._muscles, forces):
            muscle.setOverrideActuation(state, float(force))
            muscle.overrideActuation(state, True)

        self._last_muscle_override_forces = forces.copy()

    def _disable_muscle_actuation_overrides(self, state: opensim.State) -> None:
        for muscle in self._muscles:
            muscle.overrideActuation(state, False)

    def _safe_compute_equilibrium(
        self,
        state: opensim.State,
        muscle: opensim.Muscle,
    ) -> bool:
        try:
            muscle.computeEquilibrium(state)
            fiber_length = self._safe_muscle_fiber_length(muscle, state)
            if np.isfinite(fiber_length) and fiber_length > 1e-8:
                return True
        except Exception:
            pass

        self._set_muscle_state_values(
            state,
            muscle.getName(),
            activation=float(muscle.getActivation(state)),
            fiber_length=muscle.getOptimalFiberLength(),
        )
        return False

    def _set_muscle_state_values(
        self,
        state: opensim.State,
        muscle_name: str,
        *,
        activation: Optional[float] = None,
        fiber_length: Optional[float] = None,
    ) -> None:
        sv = self._ctx.model.getStateVariableValues(state)
        if activation is not None:
            idx = self._ctx.muscle_activation_sv_idx[muscle_name]
            sv.set(idx, float(activation))
        if fiber_length is not None and np.isfinite(fiber_length):
            idx = self._ctx.muscle_fiber_length_sv_idx[muscle_name]
            sv.set(idx, float(max(fiber_length, 1e-8)))
        self._ctx.model.setStateVariableValues(state, sv)

    def _safe_muscle_actuation(
        self,
        muscle: opensim.Muscle,
        state: opensim.State,
    ) -> float:
        try:
            actuation = float(muscle.getActuation(state))
        except Exception:
            return 0.0
        return actuation if np.isfinite(actuation) else 0.0

    def _safe_muscle_fiber_length(
        self,
        muscle: opensim.Muscle,
        state: opensim.State,
    ) -> float:
        try:
            fiber_length = float(muscle.getFiberLength(state))
        except Exception:
            fiber_length = muscle.getOptimalFiberLength()
        if not np.isfinite(fiber_length) or fiber_length <= 1e-8:
            fiber_length = muscle.getOptimalFiberLength()
        return fiber_length

    def _update_diagnostics(
        self,
        tau_bio: np.ndarray,
        A_muscle: np.ndarray,
        activations: np.ndarray,
        u_res: np.ndarray,
    ) -> None:
        muscle_delta = np.maximum(0.0, activations - self._activation_min)
        tau_muscle = A_muscle @ muscle_delta
        tau_reserve = self._R_res_scaled @ u_res
        residual = tau_bio - tau_muscle - tau_reserve

        muscle_norm = float(np.linalg.norm(tau_muscle))
        reserve_norm = float(np.linalg.norm(tau_reserve))
        denom = muscle_norm + reserve_norm + 1e-12

        row_capacity = np.linalg.norm(A_muscle, axis=1)
        muscle_capable = (
            (row_capacity > self._cfg.muscle_row_capacity_threshold)
            & ~self._residual_reserve_rows
        )
        if np.any(muscle_capable):
            capable_muscle_norm = float(np.linalg.norm(tau_muscle[muscle_capable]))
            capable_reserve_norm = float(np.linalg.norm(tau_reserve[muscle_capable]))
            capable_denom = capable_muscle_norm + capable_reserve_norm + 1e-12
            capable_share = capable_muscle_norm / capable_denom
        else:
            capable_muscle_norm = 0.0
            capable_reserve_norm = 0.0
            capable_share = 0.0

        self.last_diagnostics = {
            "tau_target_norm": float(np.linalg.norm(tau_bio)),
            "tau_muscle_norm": muscle_norm,
            "tau_reserve_norm": reserve_norm,
            "muscle_share": muscle_norm / denom,
            "muscle_capable_share": capable_share,
            "muscle_capable_reserve_norm": capable_reserve_norm,
            "unactuated_reserve_norm": float(np.linalg.norm(
                tau_reserve[~muscle_capable]
            )),
            "reserve_control_norm": float(np.linalg.norm(u_res)),
            "activation_mean": float(np.mean(activations)),
            "activation_nonzero_fraction": float(np.mean(
                activations > self._cfg.muscle_active_threshold
            )),
            "residual_norm": float(np.linalg.norm(residual)),
            "equilibrium_failures": float(self._last_equilibrium_failures),
        }

    # ── SLSQP via scipy ──────────────────────────────────────────────────────
    def _solve_slsqp(
        self,
        A: np.ndarray,
        b: np.ndarray,
    ) -> Tuple[np.ndarray, bool]:
        """
        Solve  min (1/2) x^T diag(w) x   s.t.  A x = b,  lb ≤ x ≤ ub.

        The QP cost is convex, the equality constraint is linear, so SLSQP
        always finds the global minimum (if feasible).
        """
        w = self._weights

        def obj(x: np.ndarray) -> float:
            return 0.5 * float(w @ (x * x))

        def obj_grad(x: np.ndarray) -> np.ndarray:
            return w * x

        constraint = {
            "type": "eq",
            "fun":  lambda x: A @ x - b,
            "jac":  lambda x: A,          # constant Jacobian
        }

        result: OptimizeResult = minimize(
            obj,
            x0           = self._x_prev,
            jac          = obj_grad,
            method       = "SLSQP",
            bounds       = self._bounds,
            constraints  = [constraint],
            options      = {
                "maxiter": self._cfg.qp_max_iter,
                "ftol":    1e-8,
                "disp":    False,
            },
        )
        return result.x, result.success

    def _solve_bounded_least_squares(
        self,
        A: np.ndarray,
        b: np.ndarray,
    ) -> np.ndarray:
        """
        Robust fallback for frames where SLSQP cannot satisfy the equality
        constraints exactly.

        The variable transform y=sqrt(w)*x keeps the fallback close to the QP
        objective: among near-feasible controls, reserve usage remains costly.
        """
        if not np.all(np.isfinite(A)) or not np.all(np.isfinite(b)):
            warnings.warn(
                "[StaticOptimizer] Non-finite SO inputs. Reusing previous solution.",
                RuntimeWarning,
            )
            return self._x_prev.copy()

        scale = np.sqrt(self._weights)
        A_scaled = A / scale[np.newaxis, :]

        lb = np.array([lo for lo, _ in self._bounds], dtype=float) * scale
        ub = np.array([hi for _, hi in self._bounds], dtype=float) * scale

        result = lsq_linear(
            A_scaled,
            b,
            bounds=(lb, ub),
            lsmr_tol="auto",
            max_iter=self._cfg.qp_max_iter,
        )
        return result.x / scale

    # ── OSQP via qpsolvers ───────────────────────────────────────────────────
    def _solve_osqp(
        self,
        A: np.ndarray,
        b: np.ndarray,
    ) -> Tuple[np.ndarray, bool]:
        """
        Solve the same QP using OSQP (faster for large muscle sets).

        Problem in standard QP form:
            min   (1/2) x^T P x
            s.t.  A_eq x = b_eq
                  lb ≤ x ≤ ub

        where P = diag(weights).
        """
        import qpsolvers
        import scipy.sparse as sp

        n_x  = len(self._weights)
        P = sp.diags(self._weights, format="csc")
        q = np.zeros(n_x)   # linear cost term  (none here)

        # Build combined constraint matrix for qpsolvers:
        # equality + box bounds encoded as inequalities
        A_sp = sp.csc_matrix(A)
        lb   = np.concatenate([
            self._activation_min * np.ones(self._ctx.n_muscles),
            -self._reserve_u_bounds,
        ])
        ub   = np.concatenate([
            self._activation_max * np.ones(self._ctx.n_muscles),
            self._reserve_u_bounds,
        ])

        try:
            x_sol = qpsolvers.solve_qp(
                P     = P,
                q     = q,
                A     = A_sp,
                b     = b,
                lb    = lb,
                ub    = ub,
                solver = "osqp",
                initvals = self._x_prev,
            )
            success = x_sol is not None
            if not success:
                x_sol = self._x_prev.copy()
        except Exception as e:
            warnings.warn(f"[StaticOptimizer] OSQP failed: {e}. Using SLSQP fallback.")
            return self._solve_slsqp(A, b)

        return x_sol, success

    # ─────────────────────────────────────────────────────────────────────────
    #  Inject solution into control Vector
    # ─────────────────────────────────────────────────────────────────────────
    def apply_to_controls(
        self,
        a:        np.ndarray,
        u_res:    np.ndarray,
        controls: opensim.Vector,
        state:    Optional[opensim.State] = None,
    ) -> None:
        """
        Write the SO solution into the model's control Vector in-place.

        Parameters
        ----------
        a        : muscle activations  shape (n_muscles,)
        u_res    : reserve controls    shape (n_reserves,)
        controls : model control Vector (modified in-place)
        state    : optional OpenSim state; if provided, muscle activations and
                   muscle actuator overrides are updated for immediate force
                   output.
        """
        ctx = self._ctx

        if state is not None:
            self.apply_activations_to_state(state, a)

        for i, name in enumerate(ctx.muscle_names):
            controls.set(ctx.muscle_ctrl_idx[name], float(a[i]))

        for j, name in enumerate(ctx.reserve_names):
            controls.set(ctx.reserve_ctrl_idx[name], float(u_res[j]))
