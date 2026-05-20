"""
prosthesis_controller.py
========================
High-level (outer) controller for the two SEA actuators.

Architecture
------------
The SEA plugin implements a *low-level* PD torque loop in C++.  It accepts a
normalised command signal  u ∈ [−1, +1]  whose physical meaning is:

    τ_commanded = u · F_opt_sea   [N·m]

This module implements the *high-level* position-tracking loop that runs at
the Python level:

    e_q     = q_ref    − q_current
    e_qdot  = qdot_ref − qdot_current
    τ_cmd   = Kp · e_q + Kd · e_qdot
    u       = clip(τ_cmd / F_opt, −1, +1)

No inverse-dynamics feed-forward term is used in this controller: the SEA
command is generated from the prosthetic tracking controller alone.

The resulting u values are injected into the OpenSim control vector at the
indices assigned to SEA_knee and SEA_ankle before the model is realised to
acceleration.

Replacing this with a more sophisticated controller (impedance, torque-feed-
forward, etc.) only requires editing :meth:`compute`.
"""

from __future__ import annotations

from typing import Dict, Tuple

import numpy as np
import opensim

from config import SimulatorConfig
from model_loader import SimulationContext


class ProsthesisController:
    """
    High-level position-tracking PD controller for both SEA actuators.

    Parameters
    ----------
    cfg : SimulatorConfig
        Provides SEA gain tables (sea_kp / sea_kd) and coordinate names.
    ctx : SimulationContext
        Provides control vector indices (sea_ctrl_idx) and the model reference.
    """

    def __init__(self, cfg: SimulatorConfig, ctx: SimulationContext) -> None:
        self._cfg = cfg
        self._ctx = ctx
        self._pros_coords = cfg.pros_coords     # ["pros_knee_angle", "pros_ankle_angle"]
        self._sea_names   = [cfg.sea_knee_name, cfg.sea_ankle_name]

        # Pre-fetch coordinate objects once to avoid repeated string lookups
        # inside the hot path.
        coord_set = ctx.model.getCoordinateSet()
        self._coords: Dict[str, opensim.Coordinate] = {
            name: coord_set.get(name) for name in self._pros_coords
        }
        self._integral_error: Dict[str, float] = {
            name: 0.0 for name in self._pros_coords
        }
        self._cascade_integral_error: Dict[str, float] = {
            name: 0.0 for name in self._pros_coords
        }
        self._u_filtered: Dict[str, float] = {
            name: 0.0 for name in self._pros_coords
        }
        self._u_filtered_initialised: Dict[str, bool] = {
            name: False for name in self._pros_coords
        }
        self._last_time: float | None = None

    # ─────────────────────────────────────────────────────────────────────────
    #  Public API
    # ─────────────────────────────────────────────────────────────────────────
    def compute(
        self,
        state: opensim.State,
        q_ref:    Dict[str, float],
        qdot_ref: Dict[str, float],
        controls: opensim.Vector,
    ) -> Dict[str, float]:
        """
        Compute and inject SEA control signals into *controls*.

        The controls Vector is modified **in-place** at the SEA indices.

        Parameters
        ----------
        state    : current OpenSim State (must be realised to Velocity)
        q_ref    : reference positions   {coord_name: value [rad]}
        qdot_ref : reference velocities  {coord_name: value [rad/s]}
        controls : model control Vector (modified in-place)

        Returns
        -------
        u_dict : {coord_name: u} for logging / debugging
        """
        u_dict: Dict[str, float] = {}
        dt_control = self._control_dt(state)
        mode = self._controller_mode()
        for sea_name, coord_name in zip(self._sea_names, self._pros_coords):
            coord = self._coords[coord_name]

            # ── Read current state ──────────────────────────────────────────
            # These calls require state ≥ Stage::Velocity (guaranteed by caller)
            q_cur    = coord.getValue(state)       # [rad]
            qdot_cur = coord.getSpeedValue(state)  # [rad/s]

            # ── Tracking error ──────────────────────────────────────────────
            e_q    = q_ref.get(coord_name,    0.0) - q_cur
            e_qdot = qdot_ref.get(coord_name, 0.0) - qdot_cur

            f_opt = self._ctx.sea_f_opt.get(sea_name, 1.0)
            cascade_qdot_ref = qdot_ref.get(coord_name, 0.0)
            cascade_velocity_error = e_qdot
            cascade_inner_p_cmd = 0.0
            cascade_inner_i_cmd = 0.0
            cascade_xi_v = 0.0
            cascade_i_clamped = False
            cascade_anti_windup_active = False

            # ── Control law ─────────────────────────────────────────────────
            if mode == "cascade":
                self._integral_error[coord_name] = 0.0
                kp_outer = self._cfg.sea_cascade_kp_outer.get(coord_name, 0.0)
                kp_inner = self._cfg.sea_cascade_kp_inner.get(coord_name, 0.0)
                ki_inner = self._cfg.sea_cascade_ki_inner.get(coord_name, 0.0)
                cascade_qdot_ref = qdot_ref.get(coord_name, 0.0) + kp_outer * e_q
                cascade_velocity_error = cascade_qdot_ref - qdot_cur
                cascade_inner_p_cmd = kp_inner * cascade_velocity_error
                p_cmd = cascade_inner_p_cmd
                d_cmd = 0.0
                pd_cmd = p_cmd
                (
                    cascade_xi_v,
                    cascade_inner_i_cmd,
                    cascade_i_clamped,
                    cascade_anti_windup_active,
                ) = self._cascade_integral_step(
                    state,
                    sea_name,
                    coord_name,
                    cascade_velocity_error,
                    cascade_inner_p_cmd,
                    ki_inner,
                    f_opt,
                    dt_control,
                )
                integral_error = cascade_xi_v
                i_cmd = cascade_inner_i_cmd
                integral_clamped = cascade_i_clamped
                anti_windup_active = cascade_anti_windup_active
            else:
                self._cascade_integral_error[coord_name] = 0.0
                kp = self._cfg.sea_kp.get(coord_name, 5.0)
                kd = self._cfg.sea_kd.get(coord_name, 0.5)
                ki = self._cfg.sea_ki.get(coord_name, 0.0)
                p_cmd = kp * e_q
                d_cmd = kd * e_qdot
                pd_cmd = p_cmd + d_cmd
                (
                    integral_error,
                    i_cmd,
                    integral_clamped,
                    anti_windup_active,
                ) = self._pid_integral_step(
                    state,
                    sea_name,
                    coord_name,
                    e_q,
                    pd_cmd,
                    ki,
                    f_opt,
                    dt_control,
                    mode,
                )
            outer_cmd_unscaled = pd_cmd + i_cmd
            scale = self._select_feasibility_scale(
                state,
                sea_name,
                coord_name,
                outer_cmd_unscaled,
                f_opt,
            )
            tau_cmd = scale * outer_cmd_unscaled
            u_raw_unclipped = tau_cmd / f_opt if f_opt > 1e-10 else 0.0
            u_raw = float(np.clip(u_raw_unclipped, -1.0, 1.0))
            u_filt, lpf_alpha = self._apply_u_lpf(
                coord_name,
                u_raw,
                dt_control,
            )
            u = u_filt

            # ── Inject into control Vector ──────────────────────────────────
            # Set the control before any downstream Dynamics-stage plugin output
            # reads it through getControl(state).
            ctrl_idx = self._ctx.sea_ctrl_idx[sea_name]
            controls.set(ctrl_idx, u)

            u_dict[coord_name] = u
            u_dict[f"{coord_name}_u_raw"] = u_raw
            u_dict[f"{coord_name}_u_lpf_alpha"] = lpf_alpha
            u_dict[f"{coord_name}_feasibility_scale"] = scale
            u_dict[f"{coord_name}_outer_p_cmd"] = p_cmd
            u_dict[f"{coord_name}_outer_d_cmd"] = d_cmd
            u_dict[f"{coord_name}_outer_i_cmd"] = i_cmd
            u_dict[f"{coord_name}_outer_pd_unscaled_cmd"] = pd_cmd
            u_dict[f"{coord_name}_outer_pd_cmd"] = scale * pd_cmd
            u_dict[f"{coord_name}_outer_cmd_unscaled"] = outer_cmd_unscaled
            u_dict[f"{coord_name}_outer_cmd"] = tau_cmd
            u_dict[f"{coord_name}_outer_integral_error"] = integral_error
            u_dict[f"{coord_name}_outer_integral_clamped"] = (
                1.0 if integral_clamped else 0.0
            )
            u_dict[f"{coord_name}_outer_anti_windup_active"] = (
                1.0 if anti_windup_active else 0.0
            )
            u_dict[f"{coord_name}_outer_controller_mode_id"] = {
                "pd": 0.0,
                "pid": 1.0,
                "cascade": 2.0,
            }[mode]
            u_dict[f"{coord_name}_cascade_qdot_ref"] = cascade_qdot_ref
            u_dict[f"{coord_name}_cascade_velocity_error"] = (
                cascade_velocity_error
            )
            u_dict[f"{coord_name}_cascade_inner_p_cmd"] = cascade_inner_p_cmd
            u_dict[f"{coord_name}_cascade_inner_i_cmd"] = cascade_inner_i_cmd
            u_dict[f"{coord_name}_cascade_xi_v"] = cascade_xi_v
            u_dict[f"{coord_name}_cascade_i_clamped"] = (
                1.0 if cascade_i_clamped else 0.0
            )
            u_dict[f"{coord_name}_cascade_anti_windup_active"] = (
                1.0 if cascade_anti_windup_active else 0.0
            )

        return u_dict

    def _controller_mode(self) -> str:
        """Return the validated high-level SEA controller mode."""
        mode = str(getattr(self._cfg, "sea_outer_controller_mode", "pd")).lower()
        if mode not in {"pd", "pid", "cascade"}:
            raise ValueError(
                "sea_outer_controller_mode must be 'pd', 'pid', or "
                "'cascade', "
                f"got {mode!r}"
            )
        return mode

    def _control_dt(self, state: opensim.State) -> float:
        """Return elapsed controller time and reset integrators on time rewind."""
        fallback_dt = float(getattr(
            self._cfg,
            "T_control",
            getattr(self._cfg, "dt", 0.001),
        ))
        if not np.isfinite(fallback_dt) or fallback_dt < 0.0:
            fallback_dt = 0.0

        t = float(state.getTime())
        if self._last_time is None:
            dt_control = fallback_dt
        elif t < self._last_time - 1e-12:
            self.reset()
            dt_control = fallback_dt
        else:
            dt_control = max(0.0, t - self._last_time)

        if not np.isfinite(dt_control):
            dt_control = fallback_dt
        self._last_time = t
        return dt_control

    def reset(self) -> None:
        """Clear controller memory."""
        for coord_name in self._integral_error:
            self._integral_error[coord_name] = 0.0
        for coord_name in self._cascade_integral_error:
            self._cascade_integral_error[coord_name] = 0.0
        for coord_name in self._u_filtered:
            self._u_filtered[coord_name] = 0.0
            self._u_filtered_initialised[coord_name] = False
        self._last_time = None

    def _pid_integral_step(
        self,
        state: opensim.State,
        sea_name: str,
        coord_name: str,
        e_q: float,
        pd_cmd: float,
        ki: float,
        f_opt: float,
        dt_control: float,
        mode: str,
    ) -> Tuple[float, float, bool, bool]:
        """Update the bounded integral term for PID mode."""
        if mode != "pid" or abs(ki) <= 1e-12:
            self._integral_error[coord_name] = 0.0
            return 0.0, 0.0, False, False

        previous = self._integral_error.get(coord_name, 0.0)
        leak = max(0.0, float(getattr(self._cfg, "sea_integral_leak_s_inv", 0.0)))
        leak_factor = max(0.0, 1.0 - leak * max(0.0, dt_control))
        leaked = previous * leak_factor
        increment = e_q * max(0.0, dt_control)
        raw_candidate = leaked + increment
        limited_candidate, integral_clamped = self._clamp_integral(
            coord_name,
            raw_candidate,
        )
        candidate_i_cmd = ki * limited_candidate
        candidate_cmd = pd_cmd + candidate_i_cmd
        scale = self._select_feasibility_scale(
            state,
            sea_name,
            coord_name,
            candidate_cmd,
            f_opt,
        )
        u_raw = (scale * candidate_cmd) / f_opt if f_opt > 1e-10 else 0.0
        integral_push = ki * increment
        anti_windup_active = (
            abs(u_raw) > 1.0
            and abs(integral_push) > 0.0
            and (scale * candidate_cmd) * integral_push > 0.0
        )

        if anti_windup_active:
            integral_error, leaked_clamped = self._clamp_integral(coord_name, leaked)
            integral_clamped = integral_clamped or leaked_clamped
        else:
            integral_error = limited_candidate

        self._integral_error[coord_name] = integral_error
        return (
            integral_error,
            ki * integral_error,
            integral_clamped,
            anti_windup_active,
        )

    def _cascade_integral_step(
        self,
        state: opensim.State,
        sea_name: str,
        coord_name: str,
        e_v: float,
        p_cmd: float,
        ki_inner: float,
        f_opt: float,
        dt_control: float,
    ) -> Tuple[float, float, bool, bool]:
        """Update the bounded velocity-error integral for cascade mode."""
        if abs(ki_inner) <= 1e-12:
            self._cascade_integral_error[coord_name] = 0.0
            return 0.0, 0.0, False, False

        previous = self._cascade_integral_error.get(coord_name, 0.0)
        increment = e_v * max(0.0, dt_control)
        raw_candidate = previous + increment
        limited_candidate, integral_clamped = (
            self._clamp_cascade_integral_torque(
                coord_name,
                raw_candidate,
                ki_inner,
            )
        )
        candidate_i_cmd = ki_inner * limited_candidate
        candidate_cmd = p_cmd + candidate_i_cmd
        scale = self._select_feasibility_scale(
            state,
            sea_name,
            coord_name,
            candidate_cmd,
            f_opt,
        )
        u_raw = (scale * candidate_cmd) / f_opt if f_opt > 1e-10 else 0.0
        integral_push = ki_inner * increment
        anti_windup_active = (
            abs(u_raw) > 1.0
            and abs(integral_push) > 0.0
            and (scale * candidate_cmd) * integral_push > 0.0
        )

        if anti_windup_active:
            integral_error, previous_clamped = (
                self._clamp_cascade_integral_torque(
                    coord_name,
                    previous,
                    ki_inner,
                )
            )
            integral_clamped = integral_clamped or previous_clamped
        else:
            integral_error = limited_candidate

        self._cascade_integral_error[coord_name] = integral_error
        return (
            integral_error,
            ki_inner * integral_error,
            integral_clamped,
            anti_windup_active,
        )

    def _apply_u_lpf(
        self,
        coord_name: str,
        u_raw: float,
        dt_control: float,
    ) -> Tuple[float, float]:
        """First-order LPF on the outer SEA command u.

        Returns the filtered command and the discrete blend factor alpha used
        (alpha == 1.0 means filter bypassed).
        """
        cutoff_cfg = getattr(self._cfg, "sea_u_lpf_cutoff_hz", {}) or {}
        fc = float(cutoff_cfg.get(coord_name, 0.0))
        if not np.isfinite(fc) or fc <= 0.0 or not np.isfinite(dt_control) or dt_control <= 0.0:
            self._u_filtered[coord_name] = u_raw
            self._u_filtered_initialised[coord_name] = True
            return u_raw, 1.0
        tau_filter = 1.0 / (2.0 * np.pi * fc)
        alpha = dt_control / (dt_control + tau_filter)
        if alpha < 0.0:
            alpha = 0.0
        elif alpha > 1.0:
            alpha = 1.0
        if not self._u_filtered_initialised[coord_name]:
            self._u_filtered[coord_name] = u_raw
            self._u_filtered_initialised[coord_name] = True
            return u_raw, alpha
        prev = self._u_filtered[coord_name]
        u_filt = prev + alpha * (u_raw - prev)
        u_filt = float(np.clip(u_filt, -1.0, 1.0))
        self._u_filtered[coord_name] = u_filt
        return u_filt, alpha

    def _clamp_integral(
        self,
        coord_name: str,
        value: float,
    ) -> Tuple[float, bool]:
        limit_cfg = getattr(self._cfg, "sea_integral_limit", {})
        limit = float(limit_cfg.get(coord_name, 0.0))
        if not np.isfinite(limit) or limit <= 0.0:
            return 0.0, abs(value) > 0.0
        clipped = float(np.clip(value, -limit, limit))
        return clipped, abs(clipped - value) > 1e-12

    def _clamp_cascade_integral_torque(
        self,
        coord_name: str,
        value: float,
        ki_inner: float,
    ) -> Tuple[float, bool]:
        limit_cfg = getattr(
            self._cfg,
            "sea_cascade_inner_i_torque_limit",
            {},
        )
        torque_limit = float(limit_cfg.get(coord_name, 0.0))
        if (
            not np.isfinite(torque_limit)
            or torque_limit <= 0.0
            or abs(ki_inner) <= 1e-12
        ):
            return 0.0, abs(value) > 0.0
        xi_limit = torque_limit / abs(ki_inner)
        clipped = float(np.clip(value, -xi_limit, xi_limit))
        return clipped, abs(clipped - value) > 1e-12

    def _select_feasibility_scale(
        self,
        state: opensim.State,
        sea_name: str,
        coord_name: str,
        outer_cmd: float,
        f_opt: float,
    ) -> float:
        """Return the largest PD scale predicted to keep the SEA unsaturated."""
        if not getattr(self._cfg, "enable_sea_feasibility_scaling", False):
            return 1.0

        props = self._ctx.sea_props.get(sea_name, {})
        K = float(props.get("K", 0.0))
        Kp_inner = float(props.get("Kp", 0.0))
        Kd_inner = float(props.get("Kd", 0.0))
        Bm = float(props.get("Bm", 0.0))
        impedance = bool(props.get("impedance", False))
        if f_opt <= 1e-10 or K <= 1e-10:
            return 1.0

        coord = self._coords[coord_name]
        theta_j = coord.getValue(state)
        omega_j = coord.getSpeedValue(state)
        sv = self._ctx.model.getStateVariableValues(state)
        ma_idx = self._ctx.sea_motor_angle_sv_idx.get(sea_name)
        ms_idx = self._ctx.sea_motor_speed_sv_idx.get(sea_name)
        if ma_idx is None:
            return 1.0
        theta_m = sv.get(ma_idx)
        omega_m = sv.get(ms_idx) if ms_idx is not None else 0.0
        tau_spring = K * (theta_m - theta_j)

        raw_scales = getattr(self._cfg, "sea_feasibility_scales", [1.0])
        scales = [
            float(value) for value in raw_scales
            if np.isfinite(float(value)) and 0.0 <= float(value) <= 1.0
        ]
        if not scales:
            scales = [1.0]

        tau_limit = float(getattr(
            self._cfg, "sea_feasibility_tau_input_limit", 450.0
        ))
        u_limit = float(getattr(self._cfg, "sea_feasibility_u_limit", 0.95))

        best_scale = scales[-1]
        best_violation = float("inf")
        for scale in scales:
            tau_ref = float(np.clip(
                (scale * outer_cmd) / f_opt,
                -1.0,
                1.0,
            )) * f_opt
            if impedance:
                theta_m_ref = theta_j + tau_ref / K
                tau_ff_inner = tau_spring + Bm * omega_m
                tau_input_raw = (
                    tau_ff_inner
                    + Kp_inner * (theta_m_ref - theta_m)
                    + Kd_inner * (omega_j - omega_m)
                )
            else:
                tau_input_raw = (
                    tau_ref
                    + Kp_inner * (tau_ref - tau_spring)
                    - Kd_inner * omega_m
                )
            u_abs = abs((scale * outer_cmd) / f_opt)
            violation = max(
                0.0,
                abs(tau_input_raw) - tau_limit,
                u_abs - u_limit,
            )
            if violation < best_violation:
                best_violation = violation
                best_scale = scale
            if violation <= 0.0:
                return scale
        return best_scale
