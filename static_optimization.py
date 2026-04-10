"""
static_optimization.py
=======================
Frame-by-frame Static Optimisation (SO) using a Quadratic Program (QP).

Problem formulation
-------------------
Given the required bio-DOF generalised forces  τ_des  (from InverseDynamics),
find muscle activations  a ∈ [0, 1]^n_muscles  and reserve controls
u_res ∈ [−u_max, u_max]^n_reserves  that:

    minimise    Σ_i a_i²  +  w · Σ_j u_res_j²

    subject to  R · (a ⊙ F_max)  +  R_res · (u_res ⊙ F_opt)  =  τ_des
                0  ≤  a_i    ≤  1           (muscle activation bounds)
               −u_max ≤ u_res_j ≤ u_max    (reserve control bounds)

Matrices
---------
R          : (n_bio × n_muscles)  moment-arm matrix  [m]
             R[i,j] = moment arm of muscle j at bio coordinate i
F_max      : (n_muscles,)         max isometric force [N]  — from model
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

Moment-arm computation
-----------------------
Muscle.computeMomentArm(state, Coordinate) is called for every (muscle, coord)
pair at each time step.  This is the main computational cost.  Possible
optimisations (not implemented here):
  - Cache R when the configuration changes slowly (check ‖Δq‖ < threshold).
  - Use finite-difference moment-arm approximation for speed.
"""

from __future__ import annotations

import warnings
from typing import Dict, List, Optional, Tuple

import numpy as np
from scipy.optimize import minimize, OptimizeResult

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

        # ── Cost-function weight matrix ──────────────────────────────────────
        # P = diag([1,...,1, w,...,w])   (used in  (1/2) x^T P x)
        self._weights = np.ones(n_x)
        self._weights[n_m:] = cfg.reserve_weight

        # ── Variable bounds ──────────────────────────────────────────────────
        u_max = cfg.reserve_u_max
        self._bounds: List[Tuple[float, float]] = (
            [(0.0, 1.0)] * n_m          # muscle activations ∈ [0, 1]
            + [(-u_max, u_max)] * n_r   # reserve normalised control
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

        # ── Warm-start: initial guess (uniform low activation) ───────────────
        self._x_prev = np.zeros(n_x)
        self._x_prev[:n_m] = 0.01      # small non-zero to avoid zero-gradient

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
        a       : muscle activations  shape (n_muscles,)  ∈ [0, 1]
        u_res   : reserve controls    shape (n_reserves,) ∈ [−u_max, u_max]
        """
        # ── Build moment-arm matrix R and scaled constraint matrix A ─────────
        A = self._build_constraint_matrix(state)

        # ── Dispatch to selected solver ──────────────────────────────────────
        if self._use_osqp:
            x, success = self._solve_osqp(A, tau_bio)
        else:
            x, success = self._solve_slsqp(A, tau_bio)

        if not success:
            # Fallback: return previous solution and log a warning
            warnings.warn(
                f"[StaticOptimizer] QP did not converge. Reusing previous solution.",
                RuntimeWarning,
            )
            x = self._x_prev.copy()

        self._x_prev = x.copy()

        n_m = self._ctx.n_muscles
        return x[:n_m], x[n_m:]

    # ─────────────────────────────────────────────────────────────────────────
    #  Private helpers
    # ─────────────────────────────────────────────────────────────────────────
    def _build_constraint_matrix(self, state: opensim.State) -> np.ndarray:
        """
        Build the full constraint matrix  A = [R_bio_scaled | R_res_scaled]
        of shape (n_bio, n_muscles + n_reserves).

        The left block is  R_bio_scaled[i,j] = R[i,j] · F_max[j]   [N·m]
        so that the constraint reads   A · x = τ_des   with  x = [a; u_res].

        State must be ≥ Stage::Velocity for computeMomentArm to work.
        """
        ctx = self._ctx
        n_bio = ctx.n_bio
        n_m   = ctx.n_muscles

        # -- Muscle moment-arm block (changes every step) --------------------
        # R[i, j] = computeMomentArm(state, bio_coord_i)  for muscle j
        # Multiply column j by F_max[j] to get force → torque mapping.
        R_muscle_scaled = np.zeros((n_bio, n_m))
        for j, muscle in enumerate(self._muscles):
            for i, coord in enumerate(self._bio_coords):
                # computeMomentArm takes an opensim.Coordinate and the state.
                # Positive sign convention: positive moment arm means the
                # muscle *flexes* the joint (increases coord value).
                R_muscle_scaled[i, j] = (
                    muscle.computeMomentArm(state, coord)   # [m]
                    * ctx.f_max[j]                           # [N]  → [N·m]
                )

        # -- Concatenate muscle and reserve blocks ---------------------------
        A = np.hstack([R_muscle_scaled, self._R_res_scaled])
        return A

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
        n_bio = self._ctx.n_bio
        u_max = self._cfg.reserve_u_max

        P = sp.diags(self._weights, format="csc")
        q = np.zeros(n_x)   # linear cost term  (none here)

        # Build combined constraint matrix for qpsolvers:
        # equality + box bounds encoded as inequalities
        A_sp = sp.csc_matrix(A)
        lb   = np.concatenate([np.zeros(self._ctx.n_muscles),
                               -u_max * np.ones(self._ctx.n_reserves)])
        ub   = np.concatenate([np.ones(self._ctx.n_muscles),
                                u_max * np.ones(self._ctx.n_reserves)])

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
    ) -> None:
        """
        Write the SO solution into the model's control Vector in-place.

        Parameters
        ----------
        a        : muscle activations  shape (n_muscles,)
        u_res    : reserve controls    shape (n_reserves,)
        controls : model control Vector (modified in-place)
        """
        ctx = self._ctx

        for i, name in enumerate(ctx.muscle_names):
            controls.set(ctx.muscle_ctrl_idx[name], float(a[i]))

        for j, name in enumerate(ctx.reserve_names):
            controls.set(ctx.reserve_ctrl_idx[name], float(u_res[j]))
