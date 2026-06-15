"""
osim_trj_cmc_like.py
====================
Gymnasium environment for trajectory-level RL on top of the CMC-like
OpenSim simulator.

The policy does not actuate muscles, reserves, or SEA controls directly.
Instead it emits a short prosthetic kinematic-reference segment for:

    - pros_knee_angle
    - pros_ankle_angle

The segment is interpolated and injected as q_ref/qdot_ref/qddot_ref for the
prosthetic coordinates only. Every non-prosthetic coordinate continues to use
the original kinematic reference file loaded by KinematicsInterpolator. The
existing CMC-like stack then advances the model:

    kin_ref + policy prosthetic segment
      -> biological outer loop
      -> inverse dynamics
      -> static optimization
      -> prosthesis outer controller
      -> SEA plugin dynamics

This file is intentionally a separate RL adapter. It does not change the
validated SimulationRunner path and does not modify the C++ SEA plugin.
"""

from __future__ import annotations

import copy
import os
import traceback
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Iterable, Mapping, Optional, Sequence, Tuple

import numpy as np
from scipy.interpolate import BPoly, CubicHermiteSpline, CubicSpline, PchipInterpolator

try:
    from gymnasium import Env, spaces
except ModuleNotFoundError:  # pragma: no cover - small import-time fallback.
    class Env:  # type: ignore[no-redef]
        metadata: dict = {}

        def reset(self, seed=None, options=None):
            return None

    class _Box:
        def __init__(self, low, high, shape=None, dtype=np.float32):
            self.low = np.asarray(low, dtype=dtype)
            self.high = np.asarray(high, dtype=dtype)
            self.shape = tuple(shape or self.low.shape)
            self.dtype = dtype

        def sample(self):
            return np.random.uniform(self.low, self.high, self.shape).astype(
                self.dtype
            )

    class _Spaces:
        Box = _Box

    spaces = _Spaces()  # type: ignore[assignment]

from config import SimulatorConfig
from kinematics_interpolator import KinematicsInterpolator
from model_loader import setup_model
from path_resolver import normalize_cli_existing_path
from setup_io import read_last_setup_path, read_setup_xml
from simulation_runner import SegmentWallClockTimeout, SimulationRunner


CoordDict = Dict[str, float]


@dataclass(frozen=True)
class CMCEnvConfig:
    """
    RL adapter settings.

    Parameters
    ----------
    setup_xml_path
        Simulator setup XML. If omitted, the persisted last setup is used.
    segment_duration
        Duration advanced by one RL action [s].
    policy_knots
        Number of future knots emitted by the policy. The environment prepends
        a continuity anchor at the current target, so the interpolator sees
        policy_knots + 1 points.
    action_mode
        "delta": action in [-1, 1] is an offset around kin_ref.
        "absolute": action in [-1, 1] is mapped to absolute coord bounds.
        "raw": action is already an absolute trajectory in radians.
    max_delta_rad
        Max offset from kin_ref when action_mode == "delta".
    absolute_bounds_rad
        Bounds used by action_mode == "absolute"; keys are prosthetic coord
        names and values are (low, high) in radians.
    random_init
        Start each episode from a random time in the setup window.
    episode_start_offset_s
        Deterministic offset from setup t_start when random_init is disabled.
    episode_duration
        Optional maximum episode duration [s]. If omitted, use setup t_end.
    rebuild_model_on_reset
        Rebuild OpenSim model/context on reset. Disabled by default because
        PPO-style training needs cheap resets; enable only if a model/plugin
        state proves non-reusable.
    fail_fast
        If True, simulation exceptions are re-raised. If False, they become
        truncated=True and the full Python traceback is stored in info.
    record_outputs
        If True, keep the standard OutputRecorder samples and optionally save
        them on close.
    save_outputs_on_close
        Save recorder outputs from the current episode when close() is called.
    grf_mode
        Optional override of the setup GRF mode: prescribed, online_sensor, or
        online. If omitted, use the setup XML/default simulator mode.
    online_grf_profile_file
        Optional onlineGRF profile override, required by online_sensor/online.
    include_online_grf_observation
        Add normalized online GRF, contact, gait-event pulses, and heel-strike
        gait phase to the policy observation. Disabled by default so existing
        checkpoints keep their observation contract.
    prescribed_grf_disabled_sides
        Diagnostic opt-in. Keep prescribed GRF data as an oracle, but remove
        the selected side's ExternalForce from the model dynamics.
    """

    setup_xml_path: Optional[str] = None
    segment_duration: float = 0.05
    policy_knots: int = 4
    action_mode: str = "absolute"
    max_delta_rad: float | Mapping[str, float] | Sequence[float] = 0.35
    # Absolute action bounds [rad] per prosthetic coord (used by
    # action_mode="absolute"): the policy emits an ABSOLUTE trajectory, not a
    # deviation from the prescribed IK. Bounds give ex-novo exploration room
    # beyond the experimental IK range while staying inside the anti-divergence
    # truncation guards (truncation_bounds_rad): pros_knee_angle is
    # flexion-negative (IK ~[-1.06,-0.13]); pros_ankle_angle ~[-0.13,0.40].
    absolute_bounds_rad: Optional[Mapping[str, Tuple[float, float]]] = field(
        default_factory=lambda: {
            "pros_knee_angle": (-1.5, 0.0),
            "pros_ankle_angle": (-0.7, 0.7),
        }
    )
    # Simulator-side reference low-pass. The policy only emits a raw smooth
    # prosthetic reference; the simulator band-limits it to the same cutoff used
    # for the experimental IK (KinematicsInterpolator) before the prosthesis
    # controller tracks it. This removes the high-frequency content that excites
    # the ~35 Hz knee SEA resonance (knee saturating limit-cycle), so generated
    # trajectories load the SEA command like real IK does. Cutoff None -> reuse
    # cfg.kinematics_lowpass_cutoff_hz. See
    # reports/user/2026-06-01_knee_saturazione_env_rl_limit_cycle.md.
    enable_pros_ref_lpf: bool = True
    pros_ref_model: str = "second_order"
    pros_ref_lpf_cutoff_hz: Optional[float] = None
    pros_ref_lpf_zeta: float = 1.0
    # Hard reference governor applied inside the continuous reference model.
    # Limits include a small margin over the 6 Hz-filtered sound-leg IK maxima,
    # so feasible imitation is preserved while policy-generated steps cannot
    # excite the SEA with non-experimental velocity/acceleration transients.
    enable_pros_ref_governor: bool = True
    pros_ref_velocity_limit_rad_s: Mapping[str, float] = field(
        default_factory=lambda: {
            "pros_knee_angle": 6.0,
            "pros_ankle_angle": 3.5,
        }
    )
    pros_ref_acceleration_limit_rad_s2: Mapping[str, float] = field(
        default_factory=lambda: {
            "pros_knee_angle": 60.0,
            "pros_ankle_angle": 55.0,
        }
    )
    pros_ref_jerk_limit_rad_s3: Mapping[str, float] = field(
        default_factory=lambda: {
            "pros_knee_angle": 3000.0,
            "pros_ankle_angle": 2750.0,
        }
    )
    # Sound-side (biological) gait-cycle phase clock. The sound leg follows the
    # prescribed GRF/kinematics — a fixed clock — so its heel strikes are
    # deterministic and recoverable offline from the prescribed vertical GRF. The
    # env builds a sawtooth phase phi in [0,1) that resets at each sound heel
    # strike (shifted by gait_clock_phase_offset, a cycle fraction in [0,1) so the
    # reset point is tunable: 0.0=heel strike, ~0.6=toe-off, ...) and exposes
    # (sin, cos)(2*pi*phi) in the observation. This is the pacemaker the policy
    # reads to keep the (free) prosthetic leg coordinated with the sound leg. It
    # is INDEPENDENT of the online GRF event detector (unreliable on the
    # prosthetic side); the sound side is prescribed and deterministic.
    gait_clock_enable: bool = True
    gait_clock_side: str = "right"
    gait_clock_phase_offset: float = 0.0
    random_init: bool = False
    episode_start_offset_s: float = 0.0
    episode_duration: Optional[float] = None
    rebuild_model_on_reset: bool = False
    fail_fast: bool = True
    # Wall-clock budget (real seconds) for a single env step's simulation
    # (``runner.step_until`` over one segment). A *degenerate* state can make the
    # CMC-like integration crawl (e.g. Static Optimization bounded least-squares
    # fallback); when one segment exceeds this budget the episode is truncated
    # gracefully (``end_reason="step_wall_timeout"``) regardless of ``fail_fast``,
    # so one slow worker cannot stall synchronous RL sampling. A healthy segment
    # runs in well under a second; the default is hugely generous. 0 disables.
    step_wall_timeout_s: float = 30.0
    record_outputs: bool = False
    save_outputs_on_close: bool = False
    output_dir: Optional[str] = None
    output_prefix: str = "rl_episode"
    grf_mode: Optional[str] = None
    online_grf_profile_file: Optional[str] = None
    include_online_grf_observation: bool = False
    prescribed_grf_disabled_sides: Sequence[str] = ()
    # Hybrid GRF: sides whose online contact is APPLIED (not just sensed), so the
    # prosthetic ankle/knee work against a real ground reaction. Prescribed is
    # auto-disabled on these sides by the loader. Use with grf_mode=online_sensor.
    online_grf_applied_sides: Sequence[str] = ()
    # Penetration shaping on the APPLIED prosthetic contact. The contact already
    # bounds penetration physically; these are a light penalty + a clean
    # termination as a safety net. Thresholds sit above v2's normal operating
    # penetration (~17 mm) and below the runner's hard abort (online_grf_max_
    # penetration_m, default 30 mm). Reserves are NOT penalised (uncontrollable).
    grf_penetration_penalty_threshold_m: float = 0.020
    grf_penetration_termination_m: float = 0.028
    reward_grf_penetration_weight: float = 5.0
    pelvis_min_height: float = 0.55
    # Per-joint divergence guard on the *simulated* prosthetic angle q [rad].
    # Terminate (anti-divergence) if a prosthetic coordinate leaves these bounds.
    # Wide on purpose: catches a blown-up simulation, not gait-band deviations
    # (those are shaped softly in the reward, not terminated). Keyed by coord name
    # so knee/ankle can be asymmetric.
    # pros_knee_angle is flexion-negative here (IK ~[-1.06,-0.13]), pros_ankle_angle
    # ~[-0.13, 0.40]; bounds are wide guards around those measured ranges.
    truncation_bounds_rad: Mapping[str, Tuple[float, float]] = field(
        default_factory=lambda: {
            "pros_knee_angle": (-2.0, 0.0),
            "pros_ankle_angle": (-0.9, 0.9),
        }
    )
    reward_tracking_weight: float = 8.0
    reward_reference_weight: float = 6.0
    reward_bio_weight: float = 2.0
    reward_effort_weight: float = 0.05
    reward_smoothness_weight: float = 0.1
    reward_saturation_weight: float = 0.1
    reward_safety_weight: float = 2.0
    truncation_penalty: float = 1.0
    sea_u_saturation_threshold: float = 0.98
    # Observation is split into a REALISTIC "actor" prefix (signals a real
    # instrumented prosthesis could sense: prosthetic joint encoders, SEA motor
    # states, prosthetic-foot load, the controller's own clock/command memory) and
    # a PRIVILEGED "critic-only" suffix (full pelvis/contralateral state + IK
    # reference). When False (default) the env emits ONLY the actor prefix -> a
    # realistic Box(n_actor) that trains on the stock DefaultModelConfig pipeline.
    # When True the env emits the full superset Box(n_full) = [actor | privileged];
    # this REQUIRES the custom asymmetric RLModule (policy reads obs[:n_actor], the
    # value head reads the full vector). The full ordered dict is always kept in
    # info["observation"] regardless of this flag.
    critic_privileged_observation: bool = False
    actor_cyclic_phase_only: bool = False
    include_reference_state_observation: bool = False
    # Sound-leg imitation target (used only when reward_function shapes the reward
    # in "imitation" mode; the env always emits ``sound_imitation_loss`` so the
    # ex-novo reward is unchanged). A periodic phase-normalized template is built
    # from complete sound-leg gait cycles. Per-joint shifts are calibrated against
    # the measured prosthetic-side IK; the scalar shift remains the fallback.
    imitation_phase_shift: float = 0.5
    imitation_phase_shifts: Mapping[str, float] = field(
        default_factory=lambda: {
            "pros_knee_angle": 0.465,
            "pros_ankle_angle": 0.452,
        }
    )
    imitation_phase_samples: int = 200
    imitation_initialize_to_target: bool = False
    imitation_vel_weight: float = 0.02
    imitation_sound_coords: Mapping[str, str] = field(
        default_factory=lambda: {
            "pros_knee_angle": "knee_angle_r",
            "pros_ankle_angle": "ankle_angle_r",
        }
    )
    # Physical command-rate normalizers. Losses are dimensionless and emitted
    # separately so reward_function.py can tune them without changing the plant.
    command_delta_scale_rad: Mapping[str, float] = field(
        default_factory=lambda: {
            "pros_knee_angle": 0.05,
            "pros_ankle_angle": 0.04,
        }
    )
    command_velocity_scale_rad_s: Mapping[str, float] = field(
        default_factory=lambda: {
            "pros_knee_angle": 6.0,
            "pros_ankle_angle": 5.0,
        }
    )
    command_acceleration_scale_rad_s2: Mapping[str, float] = field(
        default_factory=lambda: {
            "pros_knee_angle": 200.0,
            "pros_ankle_angle": 200.0,
        }
    )
    command_jerk_scale_rad_s3: Mapping[str, float] = field(
        default_factory=lambda: {
            "pros_knee_angle": 3000.0,
            "pros_ankle_angle": 2750.0,
        }
    )
    sea_u_rate_scale: float = 0.2
    # SEA stress normalizers for the segment-level diagnostics returned by the
    # SimulationRunner. The motor torque clamp itself is fixed by the plugin.
    sea_tau_input_soft_limit_nm: float = 400.0
    sea_tau_input_clamp_nm: float = 500.0
    sea_motor_speed_scale_rad_s: float = 100.0
    sea_motor_accel_scale_rad_s2: float = 50000.0
    sea_motor_power_scale_w: float = 50000.0


class ProstheticSegmentKinematics:
    """
    Kinematics adapter that overrides only prosthetic reference coordinates.

    Non-prosthetic q/qdot/qddot are always delegated to the base
    KinematicsInterpolator, so the biological side keeps following kin_ref.
    """

    def __init__(
        self,
        base: KinematicsInterpolator,
        pros_coords: Sequence[str],
        *,
        ref_lpf_enable: bool = True,
        ref_model: str = "second_order",
        ref_lpf_cutoff_hz: float = 6.0,
        ref_lpf_zeta: float = 1.0,
        ref_governor_enable: bool = True,
        ref_velocity_limits: Sequence[float] | None = None,
        ref_acceleration_limits: Sequence[float] | None = None,
        ref_jerk_limits: Sequence[float] | None = None,
        control_dt: float = 0.001,
    ) -> None:
        self._base = base
        self._pros_coords = tuple(pros_coords)
        self._segment_t0: float | None = None
        self._segment_t1: float | None = None
        self._segment_spline: PchipInterpolator | CubicHermiteSpline | BPoly | None = None
        self._last_anchor = np.zeros(len(self._pros_coords), dtype=float)
        # Simulator-side reference low-pass: the policy emits a raw smooth target
        # trajectory; the simulator band-limits it to ref_lpf_cutoff_hz (matching
        # the experimental IK low-pass) with a 2nd-order reference model. The
        # reference fed to the prosthesis controller then has the same spectral
        # content as real IK. Filter state (filtered position/velocity) is carried
        # across segments so served position and velocity are continuous; the
        # optional governor bounds velocity and acceleration.
        self._ref_lpf_enable = bool(ref_lpf_enable) and float(ref_lpf_cutoff_hz) > 0.0
        self._ref_model = str(ref_model).strip().lower()
        if self._ref_model not in {"second_order", "butterworth3_jerk_limited"}:
            raise ValueError(
                "ref_model must be 'second_order' or 'butterworth3_jerk_limited'."
            )
        self._ref_lpf_wn = 2.0 * np.pi * float(ref_lpf_cutoff_hz)
        self._ref_lpf_zeta = float(ref_lpf_zeta)
        self._ref_lpf_dt = float(control_dt) if control_dt and control_dt > 0.0 else 0.001
        self._ref_governor_enable = bool(ref_governor_enable)
        self._ref_velocity_limits = self._positive_limits(
            ref_velocity_limits, "ref_velocity_limits"
        )
        self._ref_acceleration_limits = self._positive_limits(
            ref_acceleration_limits, "ref_acceleration_limits"
        )
        self._ref_jerk_limits = self._positive_limits(
            ref_jerk_limits, "ref_jerk_limits"
        )
        self._filt_q: np.ndarray | None = None
        self._filt_v: np.ndarray | None = None
        self._filt_a: np.ndarray | None = None
        self._last_governor_diagnostics: dict[str, object] = {}

    def _positive_limits(
        self,
        values: Sequence[float] | None,
        label: str,
    ) -> np.ndarray:
        if values is None:
            return np.full(len(self._pros_coords), np.inf, dtype=float)
        limits = np.asarray(values, dtype=float)
        expected = (len(self._pros_coords),)
        if limits.shape != expected or not np.all(np.isfinite(limits)):
            raise ValueError(f"{label} must contain {expected[0]} finite values.")
        if np.any(limits <= 0.0):
            raise ValueError(f"{label} values must be > 0.")
        return limits

    @property
    def coord_names(self) -> list[str]:
        return list(self._base.coord_names)

    def clear_segment(self) -> None:
        self._segment_t0 = None
        self._segment_t1 = None
        self._segment_spline = None
        self._filt_q = None
        self._filt_v = None
        self._filt_a = None
        self._last_governor_diagnostics = {}

    @property
    def reference_governor_diagnostics(self) -> dict[str, object]:
        diagnostics: dict[str, object] = {}
        for key, value in self._last_governor_diagnostics.items():
            diagnostics[key] = value.copy() if isinstance(value, np.ndarray) else value
        return diagnostics

    def current_target(self, t: float) -> np.ndarray:
        q, _, _ = self.get(t)
        return np.array([q[name] for name in self._pros_coords], dtype=float)

    def set_segment(
        self,
        times: np.ndarray,
        values: np.ndarray,
        derivatives: np.ndarray | None = None,
    ) -> None:
        times = np.asarray(times, dtype=float)
        values = np.asarray(values, dtype=float)

        if times.ndim != 1 or len(times) < 2:
            raise ValueError("Segment times must be a 1D array with >=2 points.")
        if values.shape != (len(times), len(self._pros_coords)):
            raise ValueError(
                "Segment values must have shape "
                f"({len(times)}, {len(self._pros_coords)}), got {values.shape}."
            )
        if np.any(np.diff(times) <= 0.0):
            raise ValueError("Segment times must be strictly increasing.")
        if not np.all(np.isfinite(values)):
            raise ValueError("Segment values contain non-finite entries.")
        if derivatives is not None:
            derivatives = np.asarray(derivatives, dtype=float)
            if derivatives.shape != values.shape:
                raise ValueError(
                    "Segment derivatives must match values shape "
                    f"{values.shape}, got {derivatives.shape}."
                )
            if not np.all(np.isfinite(derivatives)):
                raise ValueError("Segment derivatives contain non-finite entries.")

        self._segment_t0 = float(times[0])
        self._segment_t1 = float(times[-1])

        # Raw target trajectory produced by the policy (the "smooth reference").
        if derivatives is None:
            raw_spline: PchipInterpolator | CubicHermiteSpline = PchipInterpolator(
                times,
                values,
                axis=0,
                extrapolate=True,
            )
        else:
            raw_spline = CubicHermiteSpline(
                times,
                values,
                derivatives,
                axis=0,
                extrapolate=True,
            )

        if not self._ref_lpf_enable:
            self._segment_spline = raw_spline
            self._last_anchor = values[-1].copy()
            self._last_governor_diagnostics = {}
            return

        if self._ref_model == "butterworth3_jerk_limited":
            self._set_third_order_segment(times, values, derivatives, raw_spline)
            return

        # Simulator-side 6 Hz band-limiting via a 2nd-order reference model:
        #   qf_ddot = wn^2 (q_target - qf) - 2*zeta*wn*qf_dot
        # integrated over the segment from the carried filter state. The served
        # spline is built from the filtered (qf, qf_dot) samples, so q/qdot/qddot
        # stay mutually consistent and band-limited like the experimental IK.
        if self._filt_q is None or self._filt_v is None:
            self._filt_q = np.asarray(raw_spline(times[0]), dtype=float).copy()
            if derivatives is not None:
                self._filt_v = np.asarray(derivatives[0], dtype=float).copy()
            else:
                self._filt_v = np.asarray(
                    raw_spline.derivative(1)(times[0]), dtype=float
                ).copy()
            if self._ref_governor_enable:
                self._filt_v = np.clip(
                    self._filt_v,
                    -self._ref_velocity_limits,
                    self._ref_velocity_limits,
                )

        t0 = float(times[0])
        t1 = float(times[-1])
        n = max(2, int(np.ceil((t1 - t0) / self._ref_lpf_dt)))
        t_fine = np.linspace(t0, t1, n + 1)
        dt = (t1 - t0) / n
        wn = self._ref_lpf_wn
        two_zeta_wn = 2.0 * self._ref_lpf_zeta * wn
        qf = self._filt_q.copy()
        vf = self._filt_v.copy()
        q_hist = np.empty((n + 1, len(self._pros_coords)), dtype=float)
        v_hist = np.empty_like(q_hist)
        a_hist = np.zeros_like(q_hist)
        target_error_hist = np.zeros_like(q_hist)
        velocity_limited = np.zeros_like(q_hist, dtype=bool)
        acceleration_limited = np.zeros_like(q_hist, dtype=bool)
        q_hist[0] = qf
        v_hist[0] = vf
        target_error_hist[0] = np.asarray(raw_spline(t_fine[0]), dtype=float) - qf
        for i in range(1, n + 1):
            q_target = np.asarray(raw_spline(t_fine[i]), dtype=float)
            af = wn * wn * (q_target - qf) - two_zeta_wn * vf
            if self._ref_governor_enable:
                limited_af = np.clip(
                    af,
                    -self._ref_acceleration_limits,
                    self._ref_acceleration_limits,
                )
                acceleration_limited[i] = np.abs(limited_af - af) > 1e-12
                af = limited_af
            previous_vf = vf.copy()
            next_vf = previous_vf + af * dt
            if self._ref_governor_enable:
                limited_vf = np.clip(
                    next_vf,
                    -self._ref_velocity_limits,
                    self._ref_velocity_limits,
                )
                velocity_limited[i] = np.abs(limited_vf - next_vf) > 1e-12
                next_vf = limited_vf
            vf = next_vf
            # Trapezoidal position update keeps q and qdot mutually consistent.
            # CubicHermiteSpline then reproduces the bounded acceleration instead
            # of creating a 4x qddot spike from a semi-implicit position update.
            qf = qf + 0.5 * (previous_vf + vf) * dt
            q_hist[i] = qf
            v_hist[i] = vf
            a_hist[i] = (vf - previous_vf) / dt
            target_error_hist[i] = q_target - qf

        self._segment_spline = CubicHermiteSpline(
            t_fine, q_hist, v_hist, axis=0, extrapolate=True
        )
        self._filt_q = qf
        self._filt_v = vf
        self._last_anchor = q_hist[-1].copy()
        self._last_governor_diagnostics = {
            "target_error_rms_rad": np.sqrt(
                np.mean(np.square(target_error_hist), axis=0)
            ),
            "velocity_limit_fraction": float(np.mean(velocity_limited)),
            "acceleration_limit_fraction": float(np.mean(acceleration_limited)),
            "served_velocity_abs_max_rad_s": np.max(np.abs(v_hist), axis=0),
            "served_acceleration_abs_max_rad_s2": np.max(np.abs(a_hist), axis=0),
            "served_jerk_abs_max_rad_s3": np.zeros(len(self._pros_coords)),
            "jerk_limit_fraction": 0.0,
        }

    def _set_third_order_segment(
        self,
        times: np.ndarray,
        values: np.ndarray,
        derivatives: np.ndarray | None,
        raw_spline: PchipInterpolator | CubicHermiteSpline,
    ) -> None:
        """Serve a C2, jerk-limited third-order reference over one policy step."""
        if self._filt_q is None or self._filt_v is None or self._filt_a is None:
            self._filt_q = np.asarray(raw_spline(times[0]), dtype=float).copy()
            if derivatives is not None:
                self._filt_v = np.asarray(derivatives[0], dtype=float).copy()
            else:
                self._filt_v = np.asarray(
                    raw_spline.derivative(1)(times[0]), dtype=float
                ).copy()
            # There is no previously served acceleration at reset. Starting from
            # zero avoids turning the first policy endpoint into an artificial
            # qddot impulse; the jerk-limited model then builds acceleration
            # continuously from the physical reset pose and velocity.
            self._filt_a = np.zeros(len(self._pros_coords), dtype=float)
            if self._ref_governor_enable:
                self._filt_v = np.clip(
                    self._filt_v, -self._ref_velocity_limits, self._ref_velocity_limits
                )
                self._filt_a = np.clip(
                    self._filt_a,
                    -self._ref_acceleration_limits,
                    self._ref_acceleration_limits,
                )

        t0 = float(times[0])
        t1 = float(times[-1])
        n = max(2, int(np.ceil((t1 - t0) / self._ref_lpf_dt)))
        t_fine = np.linspace(t0, t1, n + 1)
        dt = (t1 - t0) / n
        wc = self._ref_lpf_wn
        q_command = np.asarray(values[-1], dtype=float)

        qf = self._filt_q.copy()
        vf = self._filt_v.copy()
        af = self._filt_a.copy()
        q_hist = np.empty((n + 1, len(self._pros_coords)), dtype=float)
        v_hist = np.empty_like(q_hist)
        a_hist = np.empty_like(q_hist)
        j_hist = np.zeros_like(q_hist)
        target_error_hist = np.empty_like(q_hist)
        velocity_limited = np.zeros_like(q_hist, dtype=bool)
        acceleration_limited = np.zeros_like(q_hist, dtype=bool)
        jerk_limited = np.zeros_like(q_hist, dtype=bool)
        q_hist[0], v_hist[0], a_hist[0] = qf, vf, af
        target_error_hist[0] = q_command - qf

        for i in range(1, n + 1):
            raw_jerk = (
                wc**3 * (q_command - qf)
                - 2.0 * wc**2 * vf
                - 2.0 * wc * af
            )
            if self._ref_governor_enable:
                jerk = np.clip(raw_jerk, -self._ref_jerk_limits, self._ref_jerk_limits)
                jerk_limited[i] = np.abs(jerk - raw_jerk) > 1e-12
                a_low = np.maximum(
                    -self._ref_acceleration_limits,
                    af - self._ref_jerk_limits * dt,
                )
                a_high = np.minimum(
                    self._ref_acceleration_limits,
                    af + self._ref_jerk_limits * dt,
                )
                # For linearly changing acceleration, constrain the endpoint
                # acceleration so the trapezoidal velocity update stays bounded.
                a_high = np.minimum(
                    a_high,
                    2.0 * (self._ref_velocity_limits - vf) / dt - af,
                )
                a_low = np.maximum(
                    a_low,
                    2.0 * (-self._ref_velocity_limits - vf) / dt - af,
                )
                feasible = a_low <= a_high
                raw_a_next = af + jerk * dt
                a_next = np.where(
                    feasible,
                    np.minimum(np.maximum(raw_a_next, a_low), a_high),
                    np.clip(
                        raw_a_next,
                        -self._ref_acceleration_limits,
                        self._ref_acceleration_limits,
                    ),
                )
                acceleration_limited[i] = np.abs(a_next - raw_a_next) > 1e-12
                jerk = (a_next - af) / dt
            else:
                jerk = raw_jerk
                a_next = af + jerk * dt

            v_next = vf + 0.5 * (af + a_next) * dt
            if self._ref_governor_enable:
                limited_v = np.clip(
                    v_next, -self._ref_velocity_limits, self._ref_velocity_limits
                )
                velocity_limited[i] = np.abs(limited_v - v_next) > 1e-12
                v_next = limited_v
            q_next = qf + vf * dt + 0.5 * af * dt**2 + jerk * dt**3 / 6.0

            qf, vf, af = q_next, v_next, a_next
            q_hist[i], v_hist[i], a_hist[i], j_hist[i] = qf, vf, af, jerk
            target_error_hist[i] = q_command - qf

        self._segment_spline = BPoly.from_derivatives(
            t_fine,
            [[q_hist[i], v_hist[i], a_hist[i]] for i in range(len(t_fine))],
        )
        self._filt_q, self._filt_v, self._filt_a = qf, vf, af
        self._last_anchor = q_hist[-1].copy()
        self._last_governor_diagnostics = {
            "reference_model": self._ref_model,
            "target_error_rms_rad": np.sqrt(np.mean(np.square(target_error_hist), axis=0)),
            "velocity_limit_fraction": float(np.mean(velocity_limited)),
            "acceleration_limit_fraction": float(np.mean(acceleration_limited)),
            "jerk_limit_fraction": float(np.mean(jerk_limited)),
            "served_velocity_abs_max_rad_s": np.max(np.abs(v_hist), axis=0),
            "served_acceleration_abs_max_rad_s2": np.max(np.abs(a_hist), axis=0),
            "served_jerk_abs_max_rad_s3": np.max(np.abs(j_hist), axis=0),
        }

    def get(self, t: float) -> tuple[CoordDict, CoordDict, CoordDict]:
        q, qdot, qddot = self._base.get(t)

        if self._segment_spline is None:
            return q, qdot, qddot
        if self._segment_t0 is None or self._segment_t1 is None:
            return q, qdot, qddot
        if t < self._segment_t0 - 1e-12 or t > self._segment_t1 + 1e-12:
            return q, qdot, qddot

        q_values = np.asarray(self._segment_spline(t), dtype=float)
        qd_values = np.asarray(self._segment_spline.derivative(1)(t), dtype=float)
        qdd_values = np.asarray(self._segment_spline.derivative(2)(t), dtype=float)

        for i, coord_name in enumerate(self._pros_coords):
            q[coord_name] = float(q_values[i])
            qdot[coord_name] = float(qd_values[i])
            qddot[coord_name] = float(qdd_values[i])
        return q, qdot, qddot


class GaitPhaseClock:
    """Phase clock locked to the sound-side gait cycle.

    The sound (biological) leg follows the prescribed GRF/kinematics — a fixed
    clock — so its heel strikes are deterministic and recoverable offline from
    the prescribed vertical GRF. This builds a sawtooth gait phase ``phi`` in
    ``[0, 1)`` that resets at each sound heel strike (optionally shifted by
    ``phase_offset``, a cycle fraction, so the reset point can be tuned to
    toe-off / mid-stance) and exposes ``(sin, cos)(2*pi*phi)``. It is the
    pacemaker the policy reads to keep the free prosthetic leg coordinated
    (anti-phase) with the sound leg. It does NOT depend on the online GRF event
    detector (which is unreliable on the prosthetic side).

    Out of the detected-strike range the phase is extrapolated with the nearest
    cycle period, so a long episode that runs past the last detected strike keeps
    a continuous, well-defined phase.
    """

    def __init__(self, heel_strike_times, *, phase_offset: float = 0.0) -> None:
        hs = np.asarray(
            sorted(float(t) for t in heel_strike_times), dtype=float
        )
        if hs.size:
            hs = hs[np.isfinite(hs)]
        if hs.size > 1:
            # Drop non-increasing duplicates defensively.
            keep = np.concatenate(([True], np.diff(hs) > 1e-9))
            hs = hs[keep]
        self._hs = hs
        self._offset = float(phase_offset) % 1.0
        self.available = bool(self._hs.size >= 2)

    @property
    def heel_strike_times(self) -> np.ndarray:
        return self._hs

    @property
    def n_cycles(self) -> int:
        return int(max(0, self._hs.size - 1))

    @property
    def mean_period(self) -> float:
        if self._hs.size < 2:
            return 0.0
        return float(np.mean(np.diff(self._hs)))

    def _phase_cycle_raw(self, t: float) -> float:
        """Fraction since the bracketing heel strike. May be <0 (before the
        first strike) or >=1 (after the last), extrapolated with the nearest
        cycle period; ``phase`` wraps it back into ``[0, 1)``."""
        hs = self._hs
        t = float(t)
        if t < hs[0]:
            period = hs[1] - hs[0]
            return (t - hs[0]) / period
        if t >= hs[-1]:
            period = hs[-1] - hs[-2]
            return (t - hs[-1]) / period
        k = int(np.searchsorted(hs, t, side="right")) - 1
        period = hs[k + 1] - hs[k]
        return (t - hs[k]) / period

    def phase(self, t: float) -> float:
        """Sound-side gait phase in ``[0, 1)`` (0 at the reset point)."""
        if not self.available:
            return 0.0
        x = self._phase_cycle_raw(t) - self._offset
        return float(x - np.floor(x))

    def raw_phase(self, t: float) -> float:
        """Heel-strike-relative phase in ``[0, 1)`` without display offset."""
        if not self.available:
            return 0.0
        x = self._phase_cycle_raw(t)
        return float(x - np.floor(x))

    def local_period(self, t: float) -> float:
        """Period of the cycle bracketing *t*, extrapolated at the edges."""
        if not self.available:
            return 0.0
        hs = self._hs
        t = float(t)
        if t < hs[0]:
            return float(hs[1] - hs[0])
        if t >= hs[-1]:
            return float(hs[-1] - hs[-2])
        k = int(np.searchsorted(hs, t, side="right")) - 1
        return float(hs[k + 1] - hs[k])

    def phase_sin_cos(self, t: float) -> tuple[float, float]:
        ang = 2.0 * np.pi * self.phase(t)
        return float(np.sin(ang)), float(np.cos(ang))


class PhaseBasedImitationTarget:
    """Periodic sound-leg target indexed by local gait phase.

    Complete sound-side cycles in the requested simulation window are resampled
    on a common phase grid and averaged. Periodic cubic splines then provide
    mutually consistent position and velocity targets without querying before
    the IK domain or clamping an initial prefix.
    """

    def __init__(
        self,
        base_kin: KinematicsInterpolator,
        gait_clock: GaitPhaseClock,
        sound_coords: Mapping[str, str],
        phase_shifts: Mapping[str, float],
        *,
        fallback_phase_shift: float = 0.5,
        phase_samples: int = 200,
        time_window: tuple[float, float] | None = None,
    ) -> None:
        self._base_kin = base_kin
        self._clock = gait_clock
        self._sound_coords = dict(sound_coords)
        self._phase_shifts = {
            coord: float(phase_shifts.get(coord, fallback_phase_shift)) % 1.0
            for coord in sound_coords
        }
        self._splines: dict[str, CubicSpline] = {}
        self._cycle_count = 0
        self._period_mean = 0.0
        self._period_min = 0.0
        self._period_max = 0.0

        n_phase = max(16, int(phase_samples))
        if not gait_clock.available:
            return
        hs = gait_clock.heel_strike_times
        t0, t1 = base_kin.time_bounds
        cycles = [
            (float(a), float(b))
            for a, b in zip(hs[:-1], hs[1:])
            if a >= t0 - 1e-9 and b <= t1 + 1e-9 and b > a
        ]
        if time_window is not None:
            w0, w1 = (float(time_window[0]), float(time_window[1]))
            local = [
                cycle
                for cycle in cycles
                if cycle[1] >= w0 - 1e-9 and cycle[0] <= w1 + 1e-9
            ]
            if local:
                cycles = local
        if not cycles:
            return

        periods = np.asarray([b - a for a, b in cycles], dtype=float)
        median_period = float(np.median(periods))
        if median_period > 0.0:
            keep = np.abs(periods - median_period) <= 0.25 * median_period
            filtered = [cycle for cycle, use in zip(cycles, keep) if bool(use)]
            if filtered:
                cycles = filtered
                periods = np.asarray([b - a for a, b in cycles], dtype=float)

        phase = np.linspace(0.0, 1.0, n_phase, endpoint=False)
        samples: dict[str, list[np.ndarray]] = {
            coord: [] for coord in self._sound_coords
        }
        for start, end in cycles:
            period = end - start
            rows = {coord: np.empty(n_phase, dtype=float) for coord in samples}
            for i, phi in enumerate(phase):
                q, _, _ = base_kin.get(start + float(phi) * period)
                for coord, sound_name in self._sound_coords.items():
                    if sound_name in q:
                        rows[coord][i] = q[sound_name]
                    else:
                        rows[coord][i] = np.nan
            for coord, values in rows.items():
                if np.all(np.isfinite(values)):
                    samples[coord].append(values)

        phase_periodic = np.append(phase, 1.0)
        for coord, cycle_values in samples.items():
            if not cycle_values:
                continue
            mean_values = np.mean(np.vstack(cycle_values), axis=0)
            periodic_values = np.append(mean_values, mean_values[0])
            self._splines[coord] = CubicSpline(
                phase_periodic,
                periodic_values,
                bc_type="periodic",
            )

        self._cycle_count = len(cycles)
        self._period_mean = float(np.mean(periods))
        self._period_min = float(np.min(periods))
        self._period_max = float(np.max(periods))

    @property
    def available(self) -> bool:
        return bool(self._splines)

    def get(self, t: float) -> tuple[CoordDict, CoordDict, dict[str, float]]:
        """Return prosthetic target q, qdot, and target phase per coordinate."""
        q_target: CoordDict = {}
        qdot_target: CoordDict = {}
        phases: dict[str, float] = {}
        if not self.available:
            return q_target, qdot_target, phases

        raw_phase = self._clock.raw_phase(t)
        period = max(self._clock.local_period(t), 1e-9)
        for coord, spline in self._splines.items():
            target_phase = (raw_phase - self._phase_shifts[coord]) % 1.0
            q_target[coord] = float(spline(target_phase))
            qdot_target[coord] = float(spline(target_phase, 1) / period)
            phases[coord] = float(target_phase)
        return q_target, qdot_target, phases

    def summary(self) -> dict[str, object]:
        return {
            "available": bool(self.available),
            "cycle_count": int(self._cycle_count),
            "period_mean_s": float(self._period_mean),
            "period_min_s": float(self._period_min),
            "period_max_s": float(self._period_max),
            "phase_shifts": dict(self._phase_shifts),
            "coordinates": sorted(self._splines),
        }


class CMCLikeProsthesisTrajectoryEnv(Env):
    """
    RL environment that asks a policy for prosthetic trajectory segments.

    Action
    ------
    A Box with shape (policy_knots, 2). Columns follow cfg.pros_coords:
    [pros_knee_angle, pros_ankle_angle].

    In the default "absolute" mode, each action value is in [-1, 1] and is
    mapped to an ABSOLUTE prosthetic angle (a generated trajectory, not a
    deviation from the prescribed IK), per coordinate, over
    cfg.absolute_bounds_rad:

        q_policy(t_k) = low_j + 0.5 * (action[k, j] + 1) * (high_j - low_j)

    ("delta" mode — q_kin_ref + action * max_delta_rad — and "raw" mode remain
    available for diagnostics.) The environment prepends a continuity anchor at
    the current prosthetic target and interpolates the full knot set with PCHIP.

    Observation
    -----------
    Compact state vector with phase, prosthetic state, current prosthetic
    target, tracking error, SEA motor state, selected biological context, and
    previous target endpoint.
    """

    metadata = {"render_modes": []}

    def __init__(
        self,
        env_config: Optional[CMCEnvConfig] = None,
        simulator_config: Optional[SimulatorConfig] = None,
        **kwargs,
    ) -> None:
        super().__init__()
        if env_config is not None and kwargs:
            raise ValueError("Pass either env_config or keyword config values.")
        self.env_cfg = env_config or CMCEnvConfig(**kwargs)
        if self.env_cfg.policy_knots < 1:
            raise ValueError("policy_knots must be >= 1.")
        if self.env_cfg.segment_duration <= 0.0:
            raise ValueError("segment_duration must be > 0.")

        self._base_simulator_config = copy.deepcopy(
            simulator_config or SimulatorConfig()
        )
        self.cfg: SimulatorConfig
        self.ctx = None
        self.base_kin: KinematicsInterpolator
        self.kin: ProstheticSegmentKinematics
        self.runner: SimulationRunner
        self._rng = np.random.default_rng()
        self._episode_start = 0.0
        self._episode_end = 0.0
        self.t = 0.0
        self._step_index = 0
        self._last_u_sea: Dict[str, float] = {}
        self._last_u_rate_loss = 0.0
        self._last_policy_endpoint = np.zeros(2, dtype=float)
        self._last_policy_knots = np.zeros((self.env_cfg.policy_knots, 2))
        self._last_command_rate_terms: dict[str, float] = {}
        self._last_sea_segment_diagnostics: dict = {}
        self._observation_feature_names: tuple[str, ...] | None = None
        self._actor_feature_names: tuple[str, ...] | None = None
        self._privileged_feature_names: tuple[str, ...] | None = None
        self._n_actor: int = 0
        self._last_info: dict = {}
        self._body_weight_n = 1.0
        self._online_grf: dict = {}
        self._online_events: list[dict] = []
        self._online_gait_sides: dict[str, dict[str, float | None]] = {}
        self._gait_clock: GaitPhaseClock | None = None
        self._imitation_target: PhaseBasedImitationTarget | None = None
        self._closed = False

        self._delta_scale = self._resolve_delta_scale()
        self.action_space = self._build_action_space()

        self._build_simulator()
        self._initialise_episode()
        obs, _ = self._get_observation()
        self.observation_space = spaces.Box(
            low=-np.inf,
            high=np.inf,
            shape=obs.shape,
            dtype=np.float32,
        )

    @property
    def observation_feature_names(self) -> tuple[str, ...]:
        """Full ordered feature names (actor prefix followed by privileged suffix)."""
        if self._observation_feature_names is None:
            raise RuntimeError("Observation schema has not been initialised yet.")
        return self._observation_feature_names

    @property
    def actor_feature_names(self) -> tuple[str, ...]:
        """Realistic feature names making up the actor prefix obs[:n_actor]."""
        if self._actor_feature_names is None:
            raise RuntimeError("Observation schema has not been initialised yet.")
        return self._actor_feature_names

    @property
    def privileged_feature_names(self) -> tuple[str, ...]:
        """Privileged (critic-only) feature names, i.e. obs[n_actor:n_obs]."""
        if self._privileged_feature_names is None:
            raise RuntimeError("Observation schema has not been initialised yet.")
        return self._privileged_feature_names

    @property
    def n_actor(self) -> int:
        """Length of the realistic actor prefix in the full observation vector."""
        if self._observation_feature_names is None:
            raise RuntimeError("Observation schema has not been initialised yet.")
        return self._n_actor

    @property
    def n_obs(self) -> int:
        """Length of the full (actor + privileged) observation vector."""
        if self._observation_feature_names is None:
            raise RuntimeError("Observation schema has not been initialised yet.")
        return len(self._observation_feature_names)

    def reset(self, seed=None, options=None):
        try:
            super().reset(seed=seed)
        except Exception:
            pass
        if seed is not None:
            self._rng = np.random.default_rng(seed)

        if (
            getattr(self, "runner", None) is not None
            and self.env_cfg.rebuild_model_on_reset
        ):
            self._build_simulator()
        self._initialise_episode()
        obs, obs_dict = self._get_observation()
        info = {
            "time": self.t,
            "observation": obs_dict,
            "observation_feature_names": self.observation_feature_names,
            "reset_diagnostics": self._reset_diagnostics_payload(),
            "grf_mode": self.cfg.grf_mode,
            "prescribed_grf_disabled_sides": list(
                getattr(self.cfg, "prescribed_grf_disabled_sides", [])
            ),
            "online_grf_applied_sides": list(
                getattr(self.cfg, "online_grf_applied_sides", [])
            ),
            "gait_clock": self._gait_clock_summary(),
            "imitation_target": self._imitation_target_summary(),
        }
        info.update(self._online_info_payload())
        return obs, info

    def step(self, action):
        action_arr = self._validate_action(action)
        self._last_policy_knots = action_arr.copy()
        target_t = min(self.t + self.env_cfg.segment_duration, self._episode_end)

        segment_times, segment_values, segment_derivatives = (
            self._action_to_segment(action_arr, target_t)
        )
        self.kin.set_segment(segment_times, segment_values, segment_derivatives)
        self._last_command_rate_terms = self._command_rate_terms(
            segment_times,
            segment_values,
            segment_derivatives,
        )
        self._last_policy_endpoint = segment_values[-1].copy()

        failure: Exception | None = None
        failure_traceback = ""
        wall_timeout = False
        step_info: dict = {}

        try:
            step_info = self.runner.step_until(
                target_t,
                record=self.env_cfg.record_outputs,
                wall_timeout_s=self.env_cfg.step_wall_timeout_s,
            )
            self.t = self.runner.current_time
            next_u_sea = dict(step_info.get("u_sea", {}))
            self._last_u_rate_loss = self._u_rate_loss(
                self._last_u_sea,
                next_u_sea,
            )
            self._last_u_sea = next_u_sea
            self._last_sea_segment_diagnostics = dict(
                step_info.get("sea_segment_diagnostics", {})
            )
            self._step_index += 1
        except SegmentWallClockTimeout as exc:
            # Deliberate guard: a degenerate, pathologically slow segment. Always
            # truncate gracefully (even with fail_fast) so a single slow worker
            # cannot gate synchronous RL sampling.
            wall_timeout = True
            failure = exc
            failure_traceback = traceback.format_exc()
        except Exception as exc:  # Native OpenSim faults cannot always be caught.
            if self.env_cfg.fail_fast:
                raise
            failure = exc
            failure_traceback = traceback.format_exc()

        self._update_online_gait_state(step_info)
        obs, obs_dict = self._get_observation()
        reward, reward_terms = self._get_reward(obs_dict)
        unsafe_reason = self._unsafe_end_reason(obs_dict)
        reached_horizon = self.t >= self._episode_end - 1e-12

        # Gymnasium semantics:
        # - unsafe states belong to the task/MDP and are true terminations;
        # - time limits and numerical failures are external truncations.
        # Unsafe events take priority if they coincide with the horizon.
        if unsafe_reason is not None:
            terminated = True
            truncated = False
            end_reason = unsafe_reason
        elif wall_timeout:
            terminated = False
            truncated = True
            end_reason = "step_wall_timeout"
        elif failure is not None:
            terminated = False
            truncated = True
            end_reason = "numerical_failure"
        elif reached_horizon:
            terminated = False
            truncated = True
            end_reason = self._horizon_end_reason()
        else:
            terminated = False
            truncated = False
            end_reason = None

        safety_loss = (
            float(self.env_cfg.truncation_penalty)
            if unsafe_reason is not None
            else 0.0
        )
        if safety_loss:
            reward = float(reward - self.env_cfg.reward_safety_weight * safety_loss)

        # Light, graded penalty on prosthetic-contact penetration beyond the soft
        # threshold (punch-through shaping). Reserves are deliberately NOT
        # penalised: they reflect the missing sound-side support, not policy
        # quality. Large penetration is handled by the termination above.
        penetration_m = self._applied_penetration_max()
        penetration_loss = max(
            0.0, penetration_m - self.env_cfg.grf_penetration_penalty_threshold_m
        )
        if penetration_loss:
            reward = float(
                reward
                - self.env_cfg.reward_grf_penetration_weight * penetration_loss
            )
        reward_terms.update(
            {
                "safety_loss": float(safety_loss),
                "grf_penetration_m": float(penetration_m),
                "grf_penetration_loss": float(penetration_loss),
                "terminated": float(bool(terminated)),
                "truncated": float(bool(truncated)),
            }
        )

        info = {
            "time": self.t,
            "target_time": target_t,
            "observation": obs_dict,
            "observation_feature_names": self.observation_feature_names,
            "reward_terms": reward_terms,
            "log": self._log_scalars(reward_terms),
            "policy_segment_times": segment_times.copy(),
            "policy_segment_values": segment_values.copy(),
            "policy_segment_derivatives": segment_derivatives.copy(),
            "reference_governor_diagnostics": (
                self.kin.reference_governor_diagnostics
            ),
            "imitation_target": self._imitation_target_payload(),
            "sea_segment_diagnostics": copy.deepcopy(
                self._last_sea_segment_diagnostics
            ),
            "end_reason": end_reason,
            "grf_mode": self.cfg.grf_mode,
            "prescribed_grf_disabled_sides": list(
                getattr(self.cfg, "prescribed_grf_disabled_sides", [])
            ),
            "online_grf_applied_sides": list(
                getattr(self.cfg, "online_grf_applied_sides", [])
            ),
        }
        info.update(self._online_info_payload())
        if failure is not None:
            info["failure"] = f"{type(failure).__name__}: {failure}"
            info["failure_traceback"] = failure_traceback
        self._last_info = info
        return obs, float(reward), bool(terminated), bool(truncated), info

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        if (
            self.env_cfg.record_outputs
            and self.env_cfg.save_outputs_on_close
            and getattr(self, "runner", None) is not None
        ):
            self.runner.save_results()

    # ------------------------------------------------------------------
    # Setup and episode initialisation
    # ------------------------------------------------------------------
    def _build_simulator(self) -> None:
        cfg = copy.deepcopy(self._base_simulator_config)
        setup_path = self._resolve_setup_path()
        if setup_path is not None:
            setup = read_setup_xml(setup_path)
            self._apply_setup_to_config(cfg, setup)
        self._apply_grf_overrides(cfg)

        if self.env_cfg.output_dir is not None:
            cfg.output_dir = self.env_cfg.output_dir
        else:
            cfg.output_dir = os.path.join("results", "_rl_cmc_like_env")
        cfg.output_prefix = self.env_cfg.output_prefix
        self._validate_runtime_path(cfg)

        if not self.env_cfg.record_outputs:
            self._disable_output_files(cfg)

        self.cfg = cfg
        self.ctx = setup_model(cfg)
        if (
            self.env_cfg.include_online_grf_observation
            and not getattr(self.ctx, "online_grf_force_paths", [])
        ):
            raise ValueError(
                "include_online_grf_observation=True requires grf_mode "
                "'online_sensor' or 'online' with a valid onlineGRF profile."
            )
        self._body_weight_n = max(
            1e-9,
            float(self.ctx.model.getTotalMass(self.ctx.state)) * 9.80665,
        )
        self.base_kin = KinematicsInterpolator(cfg)
        ref_cutoff = self.env_cfg.pros_ref_lpf_cutoff_hz
        if ref_cutoff is None:
            # Match the experimental IK band by default.
            ref_cutoff = float(getattr(cfg, "kinematics_lowpass_cutoff_hz", 6.0))
        self.kin = ProstheticSegmentKinematics(
            self.base_kin,
            cfg.pros_coords,
            ref_lpf_enable=self.env_cfg.enable_pros_ref_lpf,
            ref_model=self.env_cfg.pros_ref_model,
            ref_lpf_cutoff_hz=ref_cutoff,
            ref_lpf_zeta=self.env_cfg.pros_ref_lpf_zeta,
            ref_governor_enable=self.env_cfg.enable_pros_ref_governor,
            ref_velocity_limits=[
                self.env_cfg.pros_ref_velocity_limit_rad_s[name]
                for name in cfg.pros_coords
            ],
            ref_acceleration_limits=[
                self.env_cfg.pros_ref_acceleration_limit_rad_s2[name]
                for name in cfg.pros_coords
            ],
            ref_jerk_limits=[
                self.env_cfg.pros_ref_jerk_limit_rad_s3[name]
                for name in cfg.pros_coords
            ],
            control_dt=float(getattr(cfg, "dt", 0.001)),
        )
        self.runner = SimulationRunner(cfg, self.ctx, self.kin)
        self._gait_clock = self._build_gait_clock()
        self._imitation_target = PhaseBasedImitationTarget(
            self.base_kin,
            self._gait_clock,
            self.env_cfg.imitation_sound_coords,
            self.env_cfg.imitation_phase_shifts,
            fallback_phase_shift=float(self.env_cfg.imitation_phase_shift),
            phase_samples=int(self.env_cfg.imitation_phase_samples),
            time_window=(float(cfg.t_start), float(cfg.t_end)),
        )

    @staticmethod
    def _validate_runtime_path(cfg: SimulatorConfig) -> None:
        if cfg.sea_forward_mode != "plugin":
            raise ValueError(
                "CMCLikeProsthesisTrajectoryEnv supports only "
                "sea_forward_mode='plugin'."
            )
        if getattr(cfg, "integration_scheme", "") != "rk4_bypass":
            raise ValueError(
                "CMCLikeProsthesisTrajectoryEnv supports only "
                "integration_scheme='rk4_bypass'."
            )

    def _resolve_setup_path(self) -> Optional[Path]:
        raw = self.env_cfg.setup_xml_path
        if raw:
            return Path(raw)
        last = read_last_setup_path()
        return last if last is not None else None

    @staticmethod
    def _apply_setup_to_config(cfg: SimulatorConfig, setup) -> None:
        cfg.model_file = str(setup.model_file)
        cfg.kinematics_file = str(setup.kinematics_file)
        cfg.external_loads_xml = (
            "" if setup.external_loads_xml is None else str(setup.external_loads_xml)
        )
        cfg.reserve_actuators_xml = str(setup.reserve_actuators_xml)
        cfg.t_start = float(setup.t_start)
        cfg.t_end = float(setup.t_end)
        cfg.model_bundle_dir = str(setup.model_file.parent)
        cfg.grf_mode = str(getattr(setup, "grf_mode", "prescribed"))
        profile = getattr(setup, "online_grf_profile_file", None)
        cfg.online_grf_profile_file = "" if profile is None else str(profile)

    def _apply_grf_overrides(self, cfg: SimulatorConfig) -> None:
        if self.env_cfg.grf_mode is not None:
            cfg.grf_mode = str(self.env_cfg.grf_mode).strip().lower()
        if self.env_cfg.online_grf_profile_file is not None:
            cfg.online_grf_profile_file = normalize_cli_existing_path(
                self.env_cfg.online_grf_profile_file
            )
        cfg.prescribed_grf_disabled_sides = [
            str(side).strip().lower()
            for side in self.env_cfg.prescribed_grf_disabled_sides
            if str(side).strip()
        ]
        cfg.online_grf_applied_sides = [
            str(side).strip().lower()
            for side in self.env_cfg.online_grf_applied_sides
            if str(side).strip()
        ]

    @staticmethod
    def _disable_output_files(cfg: SimulatorConfig) -> None:
        for name in (
            "save_activations",
            "save_kinematics",
            "save_sea_controls",
            "save_tau_bio",
            "save_muscle_forces",
            "save_sea_torques",
            "save_states",
            "save_reserve_controls",
            "save_reserve_torques",
            "save_sea_states",
            "save_sea_derivatives",
            "save_sea_diagnostics",
            "save_so_torque_diagnostics",
            "save_power",
            "save_gait_events",
            "save_recruitment_diagnostics",
        ):
            if hasattr(cfg, name):
                setattr(cfg, name, False)

    def _build_gait_clock(self) -> GaitPhaseClock:
        """Build the sound-side gait-phase clock from the prescribed GRF.

        Disabled or missing data -> an unavailable clock whose phase is a
        constant 0.0 (harmless extra observation features), so the env still runs
        in pure-online GRF setups without a prescribed sound-side force.
        """
        if not self.env_cfg.gait_clock_enable:
            return GaitPhaseClock([])
        side = str(self.env_cfg.gait_clock_side).strip().lower()
        hs = self._load_sound_heel_strikes(side)
        return GaitPhaseClock(
            hs, phase_offset=float(self.env_cfg.gait_clock_phase_offset)
        )

    def _load_sound_heel_strikes(self, side: str) -> list[float]:
        """Sound-side heel-strike times [s] from the prescribed vertical GRF.

        Reuses the exact threshold-crossing detection the simulator uses for its
        own gait-event export (``output._cycles_from_vertical_grf``), so the RL
        clock and the simulator agree on what a heel strike is. Detection spans
        the full GRF file (not just the setup window) so random-init / long
        episodes anywhere in the dataset get a valid phase.
        """
        ctx = self.ctx
        cfg = self.cfg
        grf_file = getattr(ctx, "grf_data_file", "")
        grf_columns = getattr(ctx, "grf_vertical_force_columns", {})
        source_col = (
            grf_columns.get(side) if isinstance(grf_columns, Mapping) else None
        )
        if not (grf_file and os.path.isfile(grf_file) and source_col):
            return []
        # Lazy import to avoid any import-order coupling with the simulator I/O.
        from output import _cycles_from_vertical_grf, _read_storage_table

        time, col_names, data = _read_storage_table(grf_file)
        col_idx = {name: i for i, name in enumerate(col_names)}
        idx = col_idx.get(source_col)
        if idx is None or time.size < 2:
            return []
        cycles = _cycles_from_vertical_grf(
            time,
            data[:, idx],
            float(cfg.grf_contact_threshold_n),
            float(time[0]),
            float(time[-1]),
            float(getattr(cfg, "grf_min_contact_duration_s", 0.0)),
            float(getattr(cfg, "grf_min_cycle_duration_s", 0.0)),
        )
        if not cycles:
            return []
        # cycles[i] = (heel_strike_i, heel_strike_{i+1}, contact_duration_i);
        # the heel-strike edges are the cycle boundaries.
        return [float(cycles[0][0])] + [float(c[1]) for c in cycles]

    def _gait_clock_summary(self) -> dict:
        clock = self._gait_clock
        if clock is None:
            return {"available": False, "n_cycles": 0, "mean_period_s": 0.0}
        return {
            "available": bool(clock.available),
            "n_cycles": int(clock.n_cycles),
            "mean_period_s": float(clock.mean_period),
            "side": str(self.env_cfg.gait_clock_side),
            "phase_offset": float(self.env_cfg.gait_clock_phase_offset),
            "phase": float(clock.phase(self.t)),
        }

    def _imitation_target_summary(self) -> dict:
        target = self._imitation_target
        if target is None:
            return {"available": False}
        return target.summary()

    def imitation_target(
        self,
        t: float,
    ) -> tuple[CoordDict, CoordDict, dict[str, float]]:
        """Return the periodic phase-based imitation target at time *t*."""
        target = self._imitation_target
        if target is not None and target.available:
            return target.get(t)

        q_sound, qd_sound, _ = self.base_kin.get(t)
        q_target: CoordDict = {}
        qdot_target: CoordDict = {}
        for coord_name, sound_name in self.env_cfg.imitation_sound_coords.items():
            if sound_name in q_sound:
                q_target[coord_name] = float(q_sound[sound_name])
                qdot_target[coord_name] = float(qd_sound[sound_name])
        return q_target, qdot_target, {}

    def _imitation_target_payload(self) -> dict:
        q_target, qdot_target, phases = self.imitation_target(self.t)
        return {
            "q": q_target,
            "qdot": qdot_target,
            "phase": phases,
        }

    def _reset_diagnostics_payload(self) -> dict:
        """Return aligned target, prescribed, served, physical, and SEA reset state."""
        ctx = self.ctx
        if ctx is None:
            return {}

        sv = ctx.model.getStateVariableValues(self.runner.state)
        q_target, qdot_target, target_phases = self.imitation_target(self.t)
        q_base, qdot_base, _ = self.base_kin.get(self.t)
        q_ref, qdot_ref, qddot_ref = self.kin.get(self.t)

        joints: dict[str, dict[str, float]] = {}
        for coord_name in self.cfg.pros_coords:
            q_actual = float(sv.get(ctx.q_sv_idx[coord_name]))
            qdot_actual = float(sv.get(ctx.qdot_sv_idx[coord_name]))
            joints[coord_name] = {
                "target_q": float(q_target.get(coord_name, q_actual)),
                "target_qdot": float(qdot_target.get(coord_name, qdot_actual)),
                "target_phase": float(target_phases.get(coord_name, 0.0)),
                "prescribed_q": float(q_base.get(coord_name, q_actual)),
                "prescribed_qdot": float(qdot_base.get(coord_name, qdot_actual)),
                "served_q": float(q_ref.get(coord_name, q_actual)),
                "served_qdot": float(qdot_ref.get(coord_name, qdot_actual)),
                "served_qddot": float(qddot_ref.get(coord_name, 0.0)),
                "actual_q": q_actual,
                "actual_qdot": qdot_actual,
            }

        sea: dict[str, dict[str, float]] = {}
        for sea_name in (self.cfg.sea_knee_name, self.cfg.sea_ankle_name):
            values: dict[str, float] = {}
            ma_idx = ctx.sea_motor_angle_sv_idx.get(sea_name)
            ms_idx = ctx.sea_motor_speed_sv_idx.get(sea_name)
            if ma_idx is not None:
                values["motor_angle"] = float(sv.get(ma_idx))
            if ms_idx is not None:
                values["motor_speed"] = float(sv.get(ms_idx))
            sea[sea_name] = values

        gait_clock = self._gait_clock
        return {
            "time": float(self.t),
            "episode_start_offset_s": float(self.env_cfg.episode_start_offset_s),
            "imitation_initialize_to_target": bool(
                self.env_cfg.imitation_initialize_to_target
            ),
            "gait_phase": (
                float(gait_clock.phase(self.t)) if gait_clock is not None else 0.0
            ),
            "gait_raw_phase": (
                float(gait_clock.raw_phase(self.t))
                if gait_clock is not None
                else 0.0
            ),
            "joints": joints,
            "sea": sea,
        }

    def _initialise_episode(self) -> None:
        cfg = self.cfg
        ctx = self.ctx
        if ctx is None:
            raise RuntimeError("Simulation context was not built.")

        max_start = cfg.t_end
        if self.env_cfg.episode_duration is not None:
            max_start = cfg.t_end - self.env_cfg.episode_duration
        max_start = max(cfg.t_start, max_start)

        if self.env_cfg.random_init and max_start > cfg.t_start:
            self._episode_start = float(self._rng.uniform(cfg.t_start, max_start))
        else:
            requested_start = float(cfg.t_start) + max(
                0.0, float(self.env_cfg.episode_start_offset_s)
            )
            self._episode_start = min(requested_start, float(max_start))

        if self.env_cfg.episode_duration is None:
            self._episode_end = float(cfg.t_end)
        else:
            self._episode_end = min(
                float(cfg.t_end),
                self._episode_start + float(self.env_cfg.episode_duration),
            )

        self.t = self._episode_start
        self._step_index = 0
        self.kin.clear_segment()
        if self.env_cfg.imitation_initialize_to_target:
            t_next = min(self.t + float(cfg.dt), self._episode_end)
            if t_next > self.t:
                q0, qd0, _ = self.imitation_target(self.t)
                q1, qd1, _ = self.imitation_target(t_next)
                if all(name in q0 and name in q1 for name in cfg.pros_coords):
                    values = np.asarray(
                        [
                            [q0[name] for name in cfg.pros_coords],
                            [q1[name] for name in cfg.pros_coords],
                        ],
                        dtype=float,
                    )
                    derivatives = np.asarray(
                        [
                            [qd0[name] for name in cfg.pros_coords],
                            [qd1[name] for name in cfg.pros_coords],
                        ],
                        dtype=float,
                    )
                    self.kin.set_segment(
                        np.asarray([self.t, t_next], dtype=float),
                        values,
                        derivatives,
                    )

        self.runner.reset_to_time(self.t)
        if self.env_cfg.record_outputs:
            self.runner.reset_outputs()

        q_ref, _, _ = self.kin.get(self.t)
        self._last_policy_endpoint = np.array(
            [q_ref[name] for name in cfg.pros_coords],
            dtype=float,
        )
        self._last_policy_knots[:] = 0.0
        self._last_u_sea = {}
        self._last_u_rate_loss = 0.0
        self._last_command_rate_terms = {}
        self._last_sea_segment_diagnostics = {}
        self._last_info = {}
        self._reset_online_gait_state()

    # ------------------------------------------------------------------
    # Action mapping
    # ------------------------------------------------------------------
    def _build_action_space(self):
        mode = self.env_cfg.action_mode.lower()
        shape = (self.env_cfg.policy_knots, 2)
        if mode in {"delta", "absolute"}:
            return spaces.Box(low=-1.0, high=1.0, shape=shape, dtype=np.float32)
        if mode == "raw":
            return spaces.Box(low=-np.inf, high=np.inf, shape=shape, dtype=np.float32)
        raise ValueError("action_mode must be 'delta', 'absolute', or 'raw'.")

    def _resolve_delta_scale(self) -> np.ndarray:
        value = self.env_cfg.max_delta_rad
        pros = self._base_simulator_config.pros_coords
        if isinstance(value, Mapping):
            return np.array([float(value[name]) for name in pros], dtype=float)
        arr = np.asarray(value, dtype=float)
        if arr.ndim == 0:
            return np.full(2, float(arr), dtype=float)
        if arr.shape != (2,):
            raise ValueError("max_delta_rad must be scalar, mapping, or length-2.")
        return arr.astype(float)

    def _validate_action(self, action) -> np.ndarray:
        arr = np.asarray(action, dtype=float)
        expected = (self.env_cfg.policy_knots, 2)
        if arr.shape != expected:
            raise ValueError(f"Action shape must be {expected}, got {arr.shape}.")
        if not np.all(np.isfinite(arr)):
            raise ValueError("Action contains non-finite entries.")
        return arr

    def _action_to_segment(
        self,
        action: np.ndarray,
        target_t: float,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        cfg = self.cfg
        n = self.env_cfg.policy_knots
        future = self.t + (
            np.arange(1, n + 1, dtype=float) / float(n)
        ) * max(float(target_t) - self.t, self.cfg.dt)

        mode = self.env_cfg.action_mode.lower()
        if mode == "delta":
            values = np.empty((n, 2), dtype=float)
            for k, tk in enumerate(future):
                q_base, _, _ = self.base_kin.get(float(tk))
                for j, coord_name in enumerate(cfg.pros_coords):
                    values[k, j] = (
                        q_base[coord_name] + action[k, j] * self._delta_scale[j]
                    )
        elif mode == "absolute":
            bounds = self.env_cfg.absolute_bounds_rad
            if bounds is None:
                raise ValueError(
                    "absolute_bounds_rad is required when action_mode='absolute'."
                )
            values = np.empty((n, 2), dtype=float)
            for j, coord_name in enumerate(cfg.pros_coords):
                low, high = bounds[coord_name]
                values[:, j] = low + 0.5 * (action[:, j] + 1.0) * (high - low)
        elif mode == "raw":
            values = action.copy()
        else:
            raise ValueError(f"Unsupported action_mode: {self.env_cfg.action_mode}")

        q_anchor, qdot_anchor, _ = self.kin.get(self.t)
        anchor = np.array([q_anchor[name] for name in cfg.pros_coords], dtype=float)
        anchor_derivative = np.array(
            [qdot_anchor[name] for name in cfg.pros_coords],
            dtype=float,
        )
        times = np.concatenate(([self.t], future))
        segment_values = np.vstack((anchor, values))
        segment_derivatives = np.gradient(segment_values, times, axis=0)
        segment_derivatives[0, :] = anchor_derivative
        return times, segment_values, segment_derivatives

    def _coord_scales(self, values: Mapping[str, float], fallback: float) -> np.ndarray:
        return np.asarray(
            [
                max(1e-9, float(values.get(coord_name, fallback)))
                for coord_name in self.cfg.pros_coords
            ],
            dtype=float,
        )

    @staticmethod
    def _bounded_normalized_square(values: np.ndarray, scales: np.ndarray) -> float:
        normalized = np.asarray(values, dtype=float) / np.asarray(scales, dtype=float)
        return float(np.mean(np.clip(np.square(normalized), 0.0, 25.0)))

    def _command_rate_terms(
        self,
        times: np.ndarray,
        values: np.ndarray,
        derivatives: np.ndarray,
    ) -> dict[str, float]:
        delta_scales = self._coord_scales(
            self.env_cfg.command_delta_scale_rad,
            0.05,
        )
        velocity_scales = self._coord_scales(
            self.env_cfg.command_velocity_scale_rad_s,
            5.0,
        )
        acceleration_scales = self._coord_scales(
            self.env_cfg.command_acceleration_scale_rad_s2,
            200.0,
        )
        jerk_scales = self._coord_scales(
            self.env_cfg.command_jerk_scale_rad_s3,
            3000.0,
        )
        segment_delta_loss = self._bounded_normalized_square(
            np.diff(values, axis=0),
            delta_scales,
        )
        served_derivatives = np.empty_like(derivatives)
        served_accelerations = np.empty_like(derivatives)
        for i, time_value in enumerate(times):
            _, qdot_ref, qddot_ref = self.kin.get(float(time_value))
            served_derivatives[i, :] = [
                qdot_ref[name] for name in self.cfg.pros_coords
            ]
            served_accelerations[i, :] = [
                qddot_ref[name] for name in self.cfg.pros_coords
            ]
        qdot_ref_loss = self._bounded_normalized_square(
            served_derivatives,
            velocity_scales,
        )
        qddot_ref_loss = self._bounded_normalized_square(
            served_accelerations,
            acceleration_scales,
        )
        terms = {
            "segment_delta_loss": float(segment_delta_loss),
            "qdot_ref_loss": float(qdot_ref_loss),
            "qddot_ref_loss": float(qddot_ref_loss),
        }
        governor = self.kin.reference_governor_diagnostics
        target_error_rms = np.asarray(
            governor.get(
                "target_error_rms_rad",
                np.zeros(len(self.cfg.pros_coords)),
            ),
            dtype=float,
        )
        velocity_limit_fraction = float(
            governor.get("velocity_limit_fraction", 0.0)
        )
        acceleration_limit_fraction = float(
            governor.get("acceleration_limit_fraction", 0.0)
        )
        jerk_limit_fraction = float(governor.get("jerk_limit_fraction", 0.0))
        served_jerk_max = np.asarray(
            governor.get(
                "served_jerk_abs_max_rad_s3",
                np.zeros(len(self.cfg.pros_coords)),
            ),
            dtype=float,
        )
        jerk_ref_loss = self._bounded_normalized_square(
            served_jerk_max,
            jerk_scales,
        )
        terms["jerk_ref_loss"] = float(jerk_ref_loss)
        # Penalize only actual hard-limit intervention. The target-error RMS is
        # logged diagnostically but includes the intended phase lag of the 6 Hz
        # reference model and therefore is not itself an incompatibility.
        terms["reference_governor_loss"] = float(
            (
                velocity_limit_fraction
                + acceleration_limit_fraction
                + jerk_limit_fraction
            )
            / 3.0
        )
        terms["reference_velocity_limit_fraction"] = velocity_limit_fraction
        terms["reference_acceleration_limit_fraction"] = (
            acceleration_limit_fraction
        )
        terms["reference_jerk_limit_fraction"] = jerk_limit_fraction
        served_velocity_max = np.asarray(
            governor.get(
                "served_velocity_abs_max_rad_s",
                np.zeros(len(self.cfg.pros_coords)),
            ),
            dtype=float,
        )
        served_acceleration_max = np.asarray(
            governor.get(
                "served_acceleration_abs_max_rad_s2",
                np.zeros(len(self.cfg.pros_coords)),
            ),
            dtype=float,
        )
        for i, coord_name in enumerate(self.cfg.pros_coords):
            terms[f"{coord_name}_reference_target_error_rms_rad"] = float(
                target_error_rms[i]
            )
            terms[f"{coord_name}_reference_velocity_abs_max_rad_s"] = float(
                served_velocity_max[i]
            )
            terms[f"{coord_name}_reference_acceleration_abs_max_rad_s2"] = float(
                served_acceleration_max[i]
            )
            terms[f"{coord_name}_reference_jerk_abs_max_rad_s3"] = float(
                served_jerk_max[i]
            )
        return terms

    def _u_rate_loss(
        self,
        previous: Mapping[str, float],
        current: Mapping[str, float],
    ) -> float:
        if not previous:
            return 0.0
        scale = max(1e-9, float(self.env_cfg.sea_u_rate_scale))
        delta = np.asarray(
            [
                float(current.get(coord_name, 0.0))
                - float(previous.get(coord_name, 0.0))
                for coord_name in self.cfg.pros_coords
            ],
            dtype=float,
        )
        return float(np.mean(np.clip(np.square(delta / scale), 0.0, 25.0)))

    # ------------------------------------------------------------------
    # Observation, reward, termination
    # ------------------------------------------------------------------
    def _get_observation(self) -> tuple[np.ndarray, dict]:
        ctx = self.ctx
        if ctx is None:
            raise RuntimeError("Simulation context was not built.")

        model = ctx.model
        state = self.runner.state
        sv = model.getStateVariableValues(state)
        q_ref, qdot_ref, qddot_ref = self.kin.get(self.t)
        q_base, _, _ = self.base_kin.get(self.t)

        def q(name: str) -> float:
            return float(sv.get(ctx.q_sv_idx[name]))

        def qd(name: str) -> float:
            return float(sv.get(ctx.qdot_sv_idx[name]))

        # The observation is split into a REALISTIC "actor" prefix (only signals a
        # real instrumented prosthesis could sense) and a PRIVILEGED "critic-only"
        # suffix (full physical state + IK reference). The actor dict is built first
        # so its features always occupy the prefix obs[:n_actor]; an asymmetric
        # critic reads the full vector while the policy reads only the prefix. See
        # the 2026-06-10 observation-space report.
        actor: dict[str, float] = {}
        priv: dict[str, float] = {}

        # --- ACTOR (realistic) ---------------------------------------------
        if not self.env_cfg.actor_cyclic_phase_only:
            duration = max(self._episode_end - self._episode_start, self.cfg.dt)
            phase = (self.t - self._episode_start) / duration
            actor["phase"] = phase
            actor["phase_sin"] = float(np.sin(2.0 * np.pi * phase))
            actor["phase_cos"] = float(np.cos(2.0 * np.pi * phase))

        # Sound-side gait-cycle clock (the pacemaker): deterministic phase from the
        # prescribed sound-leg heel strikes, exposed as (sin, cos) to avoid the
        # 1->0 wrap discontinuity. Constant 0 when unavailable. NOTE: this is the
        # ONE known privileged input kept in the actor on an INTERIM basis -- it
        # reads the contralateral (sound) leg. To be migrated to ipsilateral-load /
        # IMU entrainment once the online GRF detector is validated; do not treat it
        # as a fully realistic signal.
        gait_phase = (
            self._gait_clock.phase(self.t)
            if self._gait_clock is not None
            else 0.0
        )
        gait_angle = 2.0 * np.pi * gait_phase
        if not self.env_cfg.actor_cyclic_phase_only:
            actor["gait_phase"] = float(gait_phase)
        actor["gait_phase_sin"] = float(np.sin(gait_angle))
        actor["gait_phase_cos"] = float(np.cos(gait_angle))

        # Prosthetic joints: measured angle + velocity (joint/motor encoder).
        for coord_name in self.cfg.pros_coords:
            actor[coord_name] = q(coord_name)
            actor[f"{coord_name}_vel"] = qd(coord_name)

        # SEA motor internal states (motor encoder).
        for sea_name in (self.cfg.sea_knee_name, self.cfg.sea_ankle_name):
            ma_idx = ctx.sea_motor_angle_sv_idx.get(sea_name)
            ms_idx = ctx.sea_motor_speed_sv_idx.get(sea_name)
            if ma_idx is not None:
                actor[f"{sea_name}_motor_angle"] = float(sv.get(ma_idx))
            if ms_idx is not None:
                actor[f"{sea_name}_motor_speed"] = float(sv.get(ms_idx))

        # Prosthetic-foot load (instrumented foot). The prosthesis is on the LEFT
        # side, so only ``online_left_*`` is sensable on-device; ``online_right_*``
        # (sound foot) is privileged and lives in the critic suffix below.
        gait = None
        if self.env_cfg.include_online_grf_observation:
            gait = self._online_gait_info()
            left_info = gait["sides"]["left"]
            actor["online_left_normal_grf_bw"] = float(
                left_info["normal_force_bw"]
            )
            actor["online_left_in_contact"] = float(left_info["in_contact"])
            actor["online_left_heel_strike"] = float(left_info["heel_strike"])
            actor["online_left_toe_off"] = float(left_info["toe_off"])
            left_phase = float(left_info["gait_phase"])
            if self.env_cfg.actor_cyclic_phase_only:
                left_phase_angle = 2.0 * np.pi * left_phase
                actor["online_left_gait_phase_sin"] = float(np.sin(left_phase_angle))
                actor["online_left_gait_phase_cos"] = float(np.cos(left_phase_angle))
            else:
                actor["online_left_gait_phase"] = left_phase
            actor["online_left_cycle_duration_s"] = float(
                left_info["cycle_duration_s"]
            )

        # Command memory (controller's own last command / internal state).
        for i, coord_name in enumerate(self.cfg.pros_coords):
            actor[f"{coord_name}_previous_endpoint"] = self._last_policy_endpoint[i]
            if self.env_cfg.include_reference_state_observation:
                actor[f"{coord_name}_served_ref"] = float(q_ref[coord_name])
                actor[f"{coord_name}_served_ref_vel"] = float(qdot_ref[coord_name])
                actor[f"{coord_name}_served_ref_accel"] = float(qddot_ref[coord_name])
            sea_u = float(self._last_u_sea.get(coord_name, 0.0))
            sea_u_abs = abs(sea_u)
            actor[f"{coord_name}_sea_u"] = sea_u
            actor[f"{coord_name}_sea_u_abs"] = sea_u_abs
            actor[f"{coord_name}_sea_u_saturated"] = float(
                sea_u_abs >= self.env_cfg.sea_u_saturation_threshold
            )

        # --- PRIVILEGED (critic-only) --------------------------------------
        # IK reference for the prosthetic joints. Privileged AND anti-ex-novo, so
        # out of the actor. Whether it ALSO leaves the critic is deferred to the
        # reward/policy redesign; for now it stays in the privileged bucket. (Moving
        # this triplet back to the actor later is just relocating these three lines.)
        for coord_name in self.cfg.pros_coords:
            priv[f"{coord_name}_target"] = q_ref[coord_name]
            priv[f"{coord_name}_target_vel"] = qdot_ref[coord_name]
            priv[f"{coord_name}_tracking_error"] = (
                q_ref[coord_name] - actor[coord_name]
            )

        # Biological context: full pelvis 6-DOF + contralateral joints, not sensable
        # from the prosthesis alone.
        for coord_name in self._bio_context_coords():
            if coord_name in ctx.q_sv_idx:
                priv[coord_name] = q(coord_name)
                priv[f"{coord_name}_vel"] = qd(coord_name)
                priv[f"{coord_name}_kin_ref"] = q_base.get(coord_name, 0.0)

        # Sound-side foot load.
        if self.env_cfg.include_online_grf_observation and gait is not None:
            right_info = gait["sides"]["right"]
            priv["online_right_normal_grf_bw"] = float(
                right_info["normal_force_bw"]
            )
            priv["online_right_in_contact"] = float(right_info["in_contact"])
            priv["online_right_heel_strike"] = float(right_info["heel_strike"])
            priv["online_right_toe_off"] = float(right_info["toe_off"])
            priv["online_right_gait_phase"] = float(right_info["gait_phase"])
            priv["online_right_cycle_duration_s"] = float(
                right_info["cycle_duration_s"]
            )

        full_obs = {**actor, **priv}
        actor_names = tuple(actor.keys())
        full_names = tuple(full_obs.keys())
        if self._observation_feature_names is None:
            self._actor_feature_names = actor_names
            self._privileged_feature_names = tuple(priv.keys())
            self._observation_feature_names = full_names
            self._n_actor = len(actor_names)
        else:
            if full_names != self._observation_feature_names:
                raise RuntimeError(
                    "Observation schema changed during the episode: "
                    f"{full_names} != {self._observation_feature_names}"
                )
            if actor_names != self._actor_feature_names:
                raise RuntimeError(
                    "Actor observation schema changed during the episode: "
                    f"{actor_names} != {self._actor_feature_names}"
                )

        full_arr = np.asarray(
            [full_obs[name] for name in self._observation_feature_names],
            dtype=np.float32,
        )
        # Symmetric (realistic) mode returns only the actor prefix; asymmetric mode
        # returns the full superset. The DICT is always the full ordered obs, so
        # info["observation"] stays complete for diagnostics/termination either way.
        if self.env_cfg.critic_privileged_observation:
            arr = full_arr
        else:
            arr = full_arr[: self._n_actor]
        return arr, full_obs

    def _reset_online_gait_state(self) -> None:
        self._online_grf = {}
        self._online_events = []
        self._online_gait_sides = {
            side: {
                "last_heel_strike_time": None,
                "last_toe_off_time": None,
                "cycle_duration_s": 0.0,
            }
            for side in ("left", "right")
        }

    def _update_online_gait_state(self, step_info: Mapping[str, object]) -> None:
        online_grf = step_info.get("online_grf")
        self._online_grf = dict(online_grf) if isinstance(online_grf, Mapping) else {}

        events = step_info.get("online_events")
        self._online_events = [
            dict(event)
            for event in events
            if isinstance(event, Mapping)
        ] if isinstance(events, Sequence) else []

        for event in self._online_events:
            side = str(event.get("side", "")).lower()
            if side not in self._online_gait_sides:
                continue
            event_name = str(event.get("event", "")).lower()
            event_time = float(event.get("time", self.t))
            if event_name == "heel_strike":
                self._online_gait_sides[side]["last_heel_strike_time"] = event_time
                cycle_duration = event.get("cycle_duration_s")
                if cycle_duration is not None and float(cycle_duration) > 0.0:
                    self._online_gait_sides[side]["cycle_duration_s"] = float(
                        cycle_duration
                    )
            elif event_name == "toe_off":
                self._online_gait_sides[side]["last_toe_off_time"] = event_time

    def _online_gait_info(self) -> dict:
        event_names = {
            side: {
                str(event.get("event", "")).lower()
                for event in self._online_events
                if str(event.get("side", "")).lower() == side
            }
            for side in ("left", "right")
        }
        sides: dict[str, dict[str, float | bool | None]] = {}
        for side in ("left", "right"):
            grf_side = self._online_grf.get(side, {})
            if not isinstance(grf_side, Mapping):
                grf_side = {}
            normal_force_n = float(grf_side.get("normal_force", 0.0))
            state = self._online_gait_sides.get(side, {})
            last_hs = state.get("last_heel_strike_time")
            cycle_duration = float(state.get("cycle_duration_s") or 0.0)
            if last_hs is None or cycle_duration <= 0.0:
                phase = 0.0
            else:
                phase = float(
                    np.clip((self.t - float(last_hs)) / cycle_duration, 0.0, 1.0)
                )
            sides[side] = {
                "normal_force_n": normal_force_n,
                "normal_force_bw": normal_force_n / self._body_weight_n,
                "in_contact": bool(grf_side.get("in_contact", False)),
                "heel_strike": "heel_strike" in event_names[side],
                "toe_off": "toe_off" in event_names[side],
                "last_heel_strike_time": last_hs,
                "last_toe_off_time": state.get("last_toe_off_time"),
                "cycle_duration_s": cycle_duration,
                "gait_phase": phase,
            }
        return {
            "available": bool(getattr(self.ctx, "online_grf_force_paths", [])),
            "sides": sides,
        }

    def _online_info_payload(self) -> dict:
        if not getattr(self.ctx, "online_grf_force_paths", []):
            return {}
        return {
            "online_grf": copy.deepcopy(self._online_grf),
            "online_events": copy.deepcopy(self._online_events),
            "online_gait": self._online_gait_info(),
        }

    def _bio_context_coords(self) -> Iterable[str]:
        # The prosthesis is on the left side, but the left hip remains a
        # biological joint above the prosthetic knee and is therefore useful
        # context/reward signal rather than a policy-controlled coordinate.
        preferred = (
            "pelvis_tx",
            "pelvis_ty",
            "pelvis_tz",
            "pelvis_tilt",
            "pelvis_list",
            "pelvis_rotation",
            "hip_flexion_r",
            "knee_angle_r",
            "ankle_angle_r",
            "hip_flexion_l",
            "hip_adduction_l",
        )
        pros = set(self.cfg.pros_coords)
        return [name for name in preferred if name not in pros]

    def _sea_reward_terms(self) -> dict[str, float]:
        """Normalize segment-level physical SEA diagnostics into reward losses."""
        diagnostics = self._last_sea_segment_diagnostics
        joints = diagnostics.get("joints") if isinstance(diagnostics, Mapping) else None
        if not isinstance(joints, Mapping):
            return {
                "sea_saturation_loss": 0.0,
                "sea_torque_error_loss": 0.0,
                "sea_motor_speed_loss": 0.0,
                "sea_motor_accel_loss": 0.0,
                "sea_motor_power_loss": 0.0,
                "sea_tau_input_saturation_fraction": 0.0,
            }

        soft = float(self.env_cfg.sea_tau_input_soft_limit_nm)
        clamp = max(soft + 1e-9, float(self.env_cfg.sea_tau_input_clamp_nm))
        speed_scale = max(1e-9, float(self.env_cfg.sea_motor_speed_scale_rad_s))
        accel_scale = max(1e-9, float(self.env_cfg.sea_motor_accel_scale_rad_s2))
        power_scale = max(1e-9, float(self.env_cfg.sea_motor_power_scale_w))
        losses = {
            "sea_saturation_loss": [],
            "sea_torque_error_loss": [],
            "sea_motor_speed_loss": [],
            "sea_motor_accel_loss": [],
            "sea_motor_power_loss": [],
            "sea_tau_input_saturation_fraction": [],
        }
        terms: dict[str, float] = {}
        for sea_name, coord_name in zip(
            (self.cfg.sea_knee_name, self.cfg.sea_ankle_name),
            self.cfg.pros_coords,
        ):
            values = joints.get(coord_name)
            if not isinstance(values, Mapping):
                continue
            raw_rms = float(values.get("tau_input_raw_rms_nm", 0.0))
            sat_fraction = float(values.get("tau_input_saturation_fraction", 0.0))
            torque_error_rms = float(values.get("torque_error_rms_nm", 0.0))
            motor_speed_rms = float(values.get("motor_speed_rms_rad_s", 0.0))
            motor_accel_rms = float(values.get("motor_accel_rms_rad_s2", 0.0))
            motor_power_rms = float(values.get("motor_power_rms_w", 0.0))
            proximity = max(0.0, raw_rms - soft) / (clamp - soft)
            f_opt = max(1e-9, float(self.ctx.sea_f_opt.get(sea_name, 1.0)))

            losses["sea_saturation_loss"].append(
                min(25.0, proximity * proximity + sat_fraction)
            )
            losses["sea_torque_error_loss"].append(
                min(25.0, (torque_error_rms / f_opt) ** 2)
            )
            losses["sea_motor_speed_loss"].append(
                min(25.0, (motor_speed_rms / speed_scale) ** 2)
            )
            losses["sea_motor_accel_loss"].append(
                min(25.0, (motor_accel_rms / accel_scale) ** 2)
            )
            losses["sea_motor_power_loss"].append(
                min(25.0, (motor_power_rms / power_scale) ** 2)
            )
            losses["sea_tau_input_saturation_fraction"].append(sat_fraction)
            prefix = f"{coord_name}_sea"
            for key, value in values.items():
                try:
                    terms[f"{prefix}_{key}"] = float(value)
                except (TypeError, ValueError):
                    continue

        for key, values in losses.items():
            terms[key] = float(np.mean(values)) if values else 0.0
        return terms

    def _get_reward(self, obs: Mapping[str, float]) -> tuple[float, dict]:
        ctx = self.ctx
        if ctx is None:
            raise RuntimeError("Simulation context was not built.")

        q_ref, qdot_ref, _ = self.kin.get(self.t)
        q_base, _, _ = self.base_kin.get(self.t)
        sv = ctx.model.getStateVariableValues(self.runner.state)

        pros_q_loss = 0.0
        pros_v_loss = 0.0
        reference_loss = 0.0
        for coord_name in self.cfg.pros_coords:
            q_cur = float(sv.get(ctx.q_sv_idx[coord_name]))
            qd_cur = float(sv.get(ctx.qdot_sv_idx[coord_name]))
            pros_q_loss += (q_ref[coord_name] - q_cur) ** 2
            pros_v_loss += 0.02 * (qdot_ref[coord_name] - qd_cur) ** 2
            reference_loss += (q_base[coord_name] - q_cur) ** 2
        tracking_loss = pros_q_loss + pros_v_loss

        bio_loss = 0.0
        bio_count = 0
        for coord_name in self._bio_context_coords():
            if coord_name in ctx.q_sv_idx:
                q_cur = float(sv.get(ctx.q_sv_idx[coord_name]))
                bio_loss += (q_base.get(coord_name, q_cur) - q_cur) ** 2
                bio_count += 1
        if bio_count:
            bio_loss /= float(bio_count)

        u_values = [
            self._last_u_sea.get(coord_name, 0.0)
            for coord_name in self.cfg.pros_coords
        ]
        u_values_arr = np.asarray(u_values, dtype=float)
        u_abs = np.abs(u_values_arr)
        effort_loss = float(np.mean(np.square(u_values_arr)))
        sat_den = max(1e-6, 1.0 - self.env_cfg.sea_u_saturation_threshold)
        saturation = np.clip(
            (u_abs - self.env_cfg.sea_u_saturation_threshold) / sat_den,
            0.0,
            1.0,
        )
        saturation_loss = float(np.mean(np.square(saturation)))
        saturation_fraction = float(
            np.mean(u_abs >= self.env_cfg.sea_u_saturation_threshold)
        )

        if self._last_policy_knots.shape[0] > 1:
            smoothness_loss = float(
                np.mean(np.square(np.diff(self._last_policy_knots, axis=0)))
            )
        else:
            smoothness_loss = 0.0
        command_rate_terms = dict(self._last_command_rate_terms)
        command_rate_terms["u_rate_loss"] = float(self._last_u_rate_loss)
        command_rate_loss = float(
            np.mean(
                [
                    command_rate_terms.get("segment_delta_loss", 0.0),
                    command_rate_terms.get("qdot_ref_loss", 0.0),
                    command_rate_terms.get("qddot_ref_loss", 0.0),
                    command_rate_terms.get("jerk_ref_loss", 0.0),
                    command_rate_terms.get("reference_governor_loss", 0.0),
                    command_rate_terms.get("u_rate_loss", 0.0),
                ]
            )
        )
        sea_reward_terms = self._sea_reward_terms()

        tracking_score = 1.0 / (
            1.0 + self.env_cfg.reward_tracking_weight * tracking_loss
        )
        reference_score = 1.0 / (
            1.0 + self.env_cfg.reward_reference_weight * reference_loss
        )
        bio_score = 1.0 / (1.0 + self.env_cfg.reward_bio_weight * bio_loss)
        penalty = (
            self.env_cfg.reward_effort_weight * effort_loss
            + self.env_cfg.reward_smoothness_weight * smoothness_loss
            + self.env_cfg.reward_saturation_weight * saturation_loss
        )
        reward = float(
            np.clip(
                0.25 * tracking_score
                + 0.55 * reference_score
                + 0.20 * bio_score
                - penalty,
                0.0,
                1.0,
            )
        )

        # Periodic, phase-based sound-leg imitation target. Position and velocity
        # come from the same periodic spline, so the target is coherent at the
        # episode start and never needs an out-of-domain time clamp.
        q_sound, qd_sound, target_phases = self.imitation_target(self.t)
        vel_w = float(self.env_cfg.imitation_vel_weight)
        sound_imitation_loss = 0.0
        served_imitation_loss = 0.0
        imitation_coord_terms: dict[str, float] = {}
        for coord_name in self.cfg.pros_coords:
            if coord_name not in q_sound:
                continue
            q_cur = float(sv.get(ctx.q_sv_idx[coord_name]))
            qd_cur = float(sv.get(ctx.qdot_sv_idx[coord_name]))
            coord_loss = (q_sound[coord_name] - q_cur) ** 2
            coord_loss += vel_w * (qd_sound[coord_name] - qd_cur) ** 2
            sound_imitation_loss += coord_loss
            served_coord_loss = (q_sound[coord_name] - q_ref[coord_name]) ** 2
            served_coord_loss += vel_w * (
                qd_sound[coord_name] - qdot_ref[coord_name]
            ) ** 2
            served_imitation_loss += served_coord_loss
            imitation_coord_terms[f"{coord_name}_imitation_loss"] = float(coord_loss)
            imitation_coord_terms[
                f"{coord_name}_served_imitation_loss"
            ] = float(served_coord_loss)
            imitation_coord_terms[f"{coord_name}_imitation_target_q"] = float(
                q_sound[coord_name]
            )
            imitation_coord_terms[f"{coord_name}_imitation_target_qdot"] = float(
                qd_sound[coord_name]
            )
            imitation_coord_terms[f"{coord_name}_imitation_target_phase"] = float(
                target_phases.get(coord_name, 0.0)
            )

        terms = {
            "tracking_loss": float(tracking_loss),
            "reference_loss": float(reference_loss),
            "bio_loss": float(bio_loss),
            "effort_loss": float(effort_loss),
            "smoothness_loss": float(smoothness_loss),
            "command_rate_loss": float(command_rate_loss),
            "saturation_loss": float(saturation_loss),
            "sound_imitation_loss": float(sound_imitation_loss),
            "served_imitation_loss": float(served_imitation_loss),
            "u_abs_max": float(np.max(u_abs)) if u_abs.size else 0.0,
            "u_saturation_fraction": float(saturation_fraction),
            "tracking_score": float(tracking_score),
            "reference_score": float(reference_score),
            "bio_score": float(bio_score),
            "penalty": float(penalty),
        }
        terms.update(command_rate_terms)
        terms.update(sea_reward_terms)
        terms.update(imitation_coord_terms)
        return reward, terms

    @staticmethod
    def _log_scalars(reward_terms: Mapping[str, float]) -> dict[str, float]:
        return {f"Reward/{key}": float(value) for key, value in reward_terms.items()}

    def _horizon_end_reason(self) -> str:
        if self._episode_end >= float(self.cfg.t_end) - 1e-12:
            return "dataset_end"
        return "episode_time_limit"

    def _applied_penetration_max(self) -> float:
        """Max contact penetration over the APPLIED online-GRF sides [m]."""
        sides = [
            str(side).strip().lower()
            for side in getattr(self.cfg, "online_grf_applied_sides", [])
            if str(side).strip()
        ]
        if not sides:
            return 0.0
        values = []
        for side in sides:
            grf_side = self._online_grf.get(side, {})
            if isinstance(grf_side, Mapping):
                values.append(float(grf_side.get("penetration", 0.0)))
        return max(values) if values else 0.0

    def _unsafe_end_reason(self, obs: Mapping[str, float]) -> str | None:
        # Termination reads the physical STATE directly (like _get_reward), not the
        # observation array: in realistic (actor-only) mode pelvis_ty and the
        # privileged coords are not in the policy's obs, but "fall"/"joint_divergence"
        # must trigger identically. ``obs`` is unused and kept for the call signature.
        ctx = self.ctx
        sv = None
        if ctx is not None and getattr(self, "runner", None) is not None:
            sv = ctx.model.getStateVariableValues(self.runner.state)

        if sv is not None and "pelvis_ty" in ctx.q_sv_idx:
            pelvis_ty = float(sv.get(ctx.q_sv_idx["pelvis_ty"]))
            if pelvis_ty < self.env_cfg.pelvis_min_height:
                return "fall"

        if (
            self._applied_penetration_max()
            > self.env_cfg.grf_penetration_termination_m
        ):
            return "grf_penetration"

        bounds = self.env_cfg.truncation_bounds_rad or {}
        if sv is not None:
            for coord_name in self.cfg.pros_coords:
                if coord_name not in ctx.q_sv_idx:
                    continue
                lo_hi = bounds.get(coord_name)
                if lo_hi is None:
                    continue
                value = float(sv.get(ctx.q_sv_idx[coord_name]))
                low, high = lo_hi
                if value < low or value > high:
                    return f"joint_divergence:{coord_name}"
        return None

    def _is_truncated(self, obs: Mapping[str, float]) -> bool:
        """Backward-compatible unsafe-state probe.

        Historically these unsafe states were labelled as truncations. Keep the
        helper for external diagnostics, but ``step`` now reports them as true
        Gymnasium terminations.
        """
        return self._unsafe_end_reason(obs) is not None


# Backward-compatible name close to the original file.
ProsthesisTrajectory = CMCLikeProsthesisTrajectoryEnv


if __name__ == "__main__":
    env = CMCLikeProsthesisTrajectoryEnv(
        CMCEnvConfig(
            segment_duration=0.02,
            policy_knots=2,
            episode_duration=0.04,
            record_outputs=False,
        )
    )
    obs, info = env.reset(seed=1)
    print("obs", obs.shape, "t", info["time"])
    obs, reward, terminated, truncated, info = env.step(env.action_space.sample())
    print(
        "step",
        obs.shape,
        f"reward={reward:.6f}",
        "terminated",
        terminated,
        "truncated",
        truncated,
        "t",
        info["time"],
    )
