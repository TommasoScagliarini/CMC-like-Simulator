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
from scipy.interpolate import CubicHermiteSpline, PchipInterpolator

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
from simulation_runner import SimulationRunner


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
    """

    setup_xml_path: Optional[str] = None
    segment_duration: float = 0.05
    policy_knots: int = 4
    action_mode: str = "delta"
    max_delta_rad: float | Mapping[str, float] | Sequence[float] = 0.35
    absolute_bounds_rad: Optional[Mapping[str, Tuple[float, float]]] = None
    # Simulator-side reference low-pass. The policy only emits a raw smooth
    # prosthetic reference; the simulator band-limits it to the same cutoff used
    # for the experimental IK (KinematicsInterpolator) before the prosthesis
    # controller tracks it. This removes the high-frequency content that excites
    # the ~35 Hz knee SEA resonance (knee saturating limit-cycle), so generated
    # trajectories load the SEA command like real IK does. Cutoff None -> reuse
    # cfg.kinematics_lowpass_cutoff_hz. See
    # reports/user/2026-06-01_knee_saturazione_env_rl_limit_cycle.md.
    enable_pros_ref_lpf: bool = True
    pros_ref_lpf_cutoff_hz: Optional[float] = None
    pros_ref_lpf_zeta: float = 1.0
    random_init: bool = False
    episode_duration: Optional[float] = None
    rebuild_model_on_reset: bool = False
    fail_fast: bool = True
    record_outputs: bool = False
    save_outputs_on_close: bool = False
    output_dir: Optional[str] = None
    output_prefix: str = "rl_episode"
    grf_mode: Optional[str] = None
    online_grf_profile_file: Optional[str] = None
    include_online_grf_observation: bool = False
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
        ref_lpf_cutoff_hz: float = 6.0,
        ref_lpf_zeta: float = 1.0,
        control_dt: float = 0.001,
    ) -> None:
        self._base = base
        self._pros_coords = tuple(pros_coords)
        self._segment_t0: float | None = None
        self._segment_t1: float | None = None
        self._segment_spline: PchipInterpolator | CubicHermiteSpline | None = None
        self._last_anchor = np.zeros(len(self._pros_coords), dtype=float)
        # Simulator-side reference low-pass: the policy emits a raw smooth target
        # trajectory; the simulator band-limits it to ref_lpf_cutoff_hz (matching
        # the experimental IK low-pass) with a 2nd-order reference model. The
        # reference fed to the prosthesis controller then has the same spectral
        # content as real IK and never excites the ~35 Hz knee SEA resonance.
        # Filter state (filtered position/velocity) is carried across segments so
        # the served q/qdot/qddot are continuous and C2.
        self._ref_lpf_enable = bool(ref_lpf_enable) and float(ref_lpf_cutoff_hz) > 0.0
        self._ref_lpf_wn = 2.0 * np.pi * float(ref_lpf_cutoff_hz)
        self._ref_lpf_zeta = float(ref_lpf_zeta)
        self._ref_lpf_dt = float(control_dt) if control_dt and control_dt > 0.0 else 0.001
        self._filt_q: np.ndarray | None = None
        self._filt_v: np.ndarray | None = None

    @property
    def coord_names(self) -> list[str]:
        return list(self._base.coord_names)

    def clear_segment(self) -> None:
        self._segment_t0 = None
        self._segment_t1 = None
        self._segment_spline = None
        self._filt_q = None
        self._filt_v = None

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
        q_hist[0] = qf
        v_hist[0] = vf
        for i in range(1, n + 1):
            q_target = np.asarray(raw_spline(t_fine[i]), dtype=float)
            af = wn * wn * (q_target - qf) - two_zeta_wn * vf
            vf = vf + af * dt          # semi-implicit Euler (stable for wn*dt<<1)
            qf = qf + vf * dt
            q_hist[i] = qf
            v_hist[i] = vf

        self._segment_spline = CubicHermiteSpline(
            t_fine, q_hist, v_hist, axis=0, extrapolate=True
        )
        self._filt_q = qf
        self._filt_v = vf
        self._last_anchor = q_hist[-1].copy()

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


class CMCLikeProsthesisTrajectoryEnv(Env):
    """
    RL environment that asks a policy for prosthetic trajectory segments.

    Action
    ------
    A Box with shape (policy_knots, 2). Columns follow cfg.pros_coords:
    [pros_knee_angle, pros_ankle_angle].

    In the default "delta" mode, each action value is in [-1, 1] and is
    converted to:

        q_policy(t_k) = q_kin_ref(t_k) + action[k, j] * max_delta_rad[j]

    The environment prepends a continuity anchor at the current prosthetic
    target and interpolates the full knot set with PCHIP.

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
        self._last_policy_endpoint = np.zeros(2, dtype=float)
        self._last_policy_knots = np.zeros((self.env_cfg.policy_knots, 2))
        self._observation_feature_names: tuple[str, ...] | None = None
        self._last_info: dict = {}
        self._body_weight_n = 1.0
        self._online_grf: dict = {}
        self._online_events: list[dict] = []
        self._online_gait_sides: dict[str, dict[str, float | None]] = {}
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
        if self._observation_feature_names is None:
            raise RuntimeError("Observation schema has not been initialised yet.")
        return self._observation_feature_names

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
            "grf_mode": self.cfg.grf_mode,
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
        self._last_policy_endpoint = segment_values[-1].copy()

        failure: Exception | None = None
        failure_traceback = ""
        step_info: dict = {}

        try:
            step_info = self.runner.step_until(
                target_t,
                record=self.env_cfg.record_outputs,
            )
            self.t = self.runner.current_time
            self._last_u_sea = dict(step_info.get("u_sea", {}))
            self._step_index += 1
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
        reward_terms.update(
            {
                "safety_loss": float(safety_loss),
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
            "end_reason": end_reason,
            "grf_mode": self.cfg.grf_mode,
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
            ref_lpf_cutoff_hz=ref_cutoff,
            ref_lpf_zeta=self.env_cfg.pros_ref_lpf_zeta,
            control_dt=float(getattr(cfg, "dt", 0.001)),
        )
        self.runner = SimulationRunner(cfg, self.ctx, self.kin)

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
            self._episode_start = float(cfg.t_start)

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

        self.runner.reset_to_time(self.t)
        if self.env_cfg.record_outputs:
            self.runner.reset_outputs()

        q_ref, _, _ = self.base_kin.get(self.t)
        self._last_policy_endpoint = np.array(
            [q_ref[name] for name in cfg.pros_coords],
            dtype=float,
        )
        self._last_policy_knots[:] = 0.0
        self._last_u_sea = {}
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
        q_ref, qdot_ref, _ = self.kin.get(self.t)
        q_base, _, _ = self.base_kin.get(self.t)

        def q(name: str) -> float:
            return float(sv.get(ctx.q_sv_idx[name]))

        def qd(name: str) -> float:
            return float(sv.get(ctx.qdot_sv_idx[name]))

        obs: dict[str, float] = {}
        duration = max(self._episode_end - self._episode_start, self.cfg.dt)
        phase = (self.t - self._episode_start) / duration
        obs["phase"] = phase
        obs["phase_sin"] = float(np.sin(2.0 * np.pi * phase))
        obs["phase_cos"] = float(np.cos(2.0 * np.pi * phase))

        for coord_name in self.cfg.pros_coords:
            obs[coord_name] = q(coord_name)
            obs[f"{coord_name}_vel"] = qd(coord_name)
            obs[f"{coord_name}_target"] = q_ref[coord_name]
            obs[f"{coord_name}_target_vel"] = qdot_ref[coord_name]
            obs[f"{coord_name}_tracking_error"] = (
                q_ref[coord_name] - obs[coord_name]
            )

        for sea_name in (self.cfg.sea_knee_name, self.cfg.sea_ankle_name):
            ma_idx = ctx.sea_motor_angle_sv_idx.get(sea_name)
            ms_idx = ctx.sea_motor_speed_sv_idx.get(sea_name)
            if ma_idx is not None:
                obs[f"{sea_name}_motor_angle"] = float(sv.get(ma_idx))
            if ms_idx is not None:
                obs[f"{sea_name}_motor_speed"] = float(sv.get(ms_idx))

        for coord_name in self._bio_context_coords():
            if coord_name in ctx.q_sv_idx:
                obs[coord_name] = q(coord_name)
                obs[f"{coord_name}_vel"] = qd(coord_name)
                obs[f"{coord_name}_kin_ref"] = q_base.get(coord_name, 0.0)

        for i, coord_name in enumerate(self.cfg.pros_coords):
            obs[f"{coord_name}_previous_endpoint"] = self._last_policy_endpoint[i]
            sea_u = float(self._last_u_sea.get(coord_name, 0.0))
            sea_u_abs = abs(sea_u)
            obs[f"{coord_name}_sea_u"] = sea_u
            obs[f"{coord_name}_sea_u_abs"] = sea_u_abs
            obs[f"{coord_name}_sea_u_saturated"] = float(
                sea_u_abs >= self.env_cfg.sea_u_saturation_threshold
            )

        if self.env_cfg.include_online_grf_observation:
            gait = self._online_gait_info()
            for side in ("left", "right"):
                side_info = gait["sides"][side]
                prefix = f"online_{side}"
                obs[f"{prefix}_normal_grf_bw"] = float(
                    side_info["normal_force_bw"]
                )
                obs[f"{prefix}_in_contact"] = float(side_info["in_contact"])
                obs[f"{prefix}_heel_strike"] = float(side_info["heel_strike"])
                obs[f"{prefix}_toe_off"] = float(side_info["toe_off"])
                obs[f"{prefix}_gait_phase"] = float(side_info["gait_phase"])
                obs[f"{prefix}_cycle_duration_s"] = float(
                    side_info["cycle_duration_s"]
                )

        names = tuple(obs.keys())
        if self._observation_feature_names is None:
            self._observation_feature_names = names
        elif names != self._observation_feature_names:
            raise RuntimeError(
                "Observation schema changed during the episode: "
                f"{names} != {self._observation_feature_names}"
            )

        arr = np.asarray(
            [obs[name] for name in self._observation_feature_names],
            dtype=np.float32,
        )
        return arr, obs

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
        terms = {
            "tracking_loss": float(tracking_loss),
            "reference_loss": float(reference_loss),
            "bio_loss": float(bio_loss),
            "effort_loss": float(effort_loss),
            "smoothness_loss": float(smoothness_loss),
            "saturation_loss": float(saturation_loss),
            "u_abs_max": float(np.max(u_abs)) if u_abs.size else 0.0,
            "u_saturation_fraction": float(saturation_fraction),
            "tracking_score": float(tracking_score),
            "reference_score": float(reference_score),
            "bio_score": float(bio_score),
            "penalty": float(penalty),
        }
        return reward, terms

    @staticmethod
    def _log_scalars(reward_terms: Mapping[str, float]) -> dict[str, float]:
        return {f"Reward/{key}": float(value) for key, value in reward_terms.items()}

    def _horizon_end_reason(self) -> str:
        if self._episode_end >= float(self.cfg.t_end) - 1e-12:
            return "dataset_end"
        return "episode_time_limit"

    def _unsafe_end_reason(self, obs: Mapping[str, float]) -> str | None:
        pelvis_ty = obs.get("pelvis_ty")
        if pelvis_ty is not None and pelvis_ty < self.env_cfg.pelvis_min_height:
            return "fall"

        bounds = self.env_cfg.truncation_bounds_rad or {}
        for coord_name in self.cfg.pros_coords:
            value = obs.get(coord_name)
            if value is None:
                continue
            lo_hi = bounds.get(coord_name)
            if lo_hi is None:
                continue
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
