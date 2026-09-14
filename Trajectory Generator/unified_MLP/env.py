"""Unified 35D environment with same-time prescribed left-leg supervision.

The simulator, SEA dynamics, governor, GRF and phase detectors are inherited.
Only the offline imitation target and two disabled clock slots differ.
"""

from __future__ import annotations

from typing import Any

from unified_MLP._bootstrap import ensure_sim_paths

ensure_sim_paths()

from baseline_MLP import win_runtime  # noqa: F401 (Torch before OpenSim on Windows)
import numpy as np

from osim_trj_cmc_like import CMCEnvConfig, CMCLikeProsthesisTrajectoryEnv


TARGET_COORDINATES = ("pros_knee_angle", "pros_ankle_angle")
CLOCK_FEATURES = ("gait_phase_sin", "gait_phase_cos")
ENV_INVARIANTS: dict[str, Any] = {
    "segment_duration": 0.01,
    "policy_knots": 1,
    "action_mode": "absolute",
    "gait_clock_enable": False,
    "actor_cyclic_phase_only": True,
    "include_imitation_target_observation": False,
    "include_reference_state_observation": True,
    "include_controller_state_observation": True,
    "include_controller_diagnostic_observation": False,
    "deployable_minimal_observation": False,
    "critic_privileged_observation": True,
    "include_online_grf_observation": True,
    "imitation_initialize_to_target": False,
    "grf_mode": "online_sensor",
    "binary_phase_fsm_mode": "binary_active",
    "binary_phase_actor_fsm_version": "v3",
    "binary_phase_event_contract_id": "binary_point_v25+heel_qualified_fsm_v2",
    "binary_phase_invalid_event_policy": "reject_continue",
    "phase_fsm_input_mode": "legacy_events",
    "event_contract_id": "legacy_events_v1",
    "detector_sample_dt_s": 0.001,
    "binary_phase_debounce_s": 0.005,
}


def validate_unified_env_config(cfg: CMCEnvConfig) -> None:
    """Reject switches that would change the common v1 environment contract."""
    for name, expected in ENV_INVARIANTS.items():
        actual = getattr(cfg, name)
        if actual != expected:
            raise ValueError(f"unified v1 requires {name}={expected!r}, got {actual!r}")
    if tuple(cfg.online_grf_applied_sides) != ("left",):
        raise ValueError("unified v1 requires online_grf_applied_sides=['left']")
    if tuple(cfg.prescribed_grf_disabled_sides):
        raise ValueError("unified v1 uses the loader's automatic left GRF routing")


class UnifiedProsthesisEnv(CMCLikeProsthesisTrajectoryEnv):
    """One environment contract for imitation and ex-novo PPO."""

    def __init__(self, env_config: CMCEnvConfig) -> None:
        validate_unified_env_config(env_config)
        super().__init__(env_config)

    def imitation_target(self, t: float):
        """Read q and qdot from prescribed left coordinates at simulation time."""
        sample_time = float(t)
        if not np.isfinite(sample_time):
            raise ValueError("prescribed-left target time must be finite")
        lower, upper = self.base_kin.time_bounds
        if sample_time < float(lower) - 1e-9 or sample_time > float(upper) + 1e-9:
            raise ValueError("prescribed-left target time is outside the IK domain")
        q_base, qdot_base, _ = self.base_kin.get(sample_time)
        q_target: dict[str, float] = {}
        qdot_target: dict[str, float] = {}
        for name in TARGET_COORDINATES:
            for source, destination, kind in (
                (q_base, q_target, "q"), (qdot_base, qdot_target, "qdot")
            ):
                if name not in source:
                    raise ValueError(f"prescribed-left target is missing {kind}[{name}]")
                value = source[name]
                if isinstance(value, (bool, np.bool_)):
                    raise ValueError(f"prescribed-left {kind}[{name}] must be numeric")
                value = float(value)
                if not np.isfinite(value):
                    raise ValueError(f"prescribed-left {kind}[{name}] must be finite")
                destination[name] = value
        # No phase transform, cycle average, side mapping or future target.
        return q_target, qdot_target, {}

    def _imitation_target_summary(self) -> dict:
        return {
            "available": True,
            "source": "prescribed_left_base_kin",
            "coordinates": list(TARGET_COORDINATES),
            "time_alignment": "simulation_time",
            "position_units": "rad",
            "velocity_units": "rad/s",
            "actor_target_observation": False,
            "position_ranges_rad": dict(self._imitation_position_ranges_rad),
            "velocity_ranges_rad_s": dict(self._imitation_velocity_ranges_rad_s),
        }

    def _imitation_target_payload(self) -> dict:
        payload = super()._imitation_target_payload()
        payload["source"] = "prescribed_left_base_kin"
        payload["time_alignment"] = "simulation_time"
        return payload

    def _get_observation(self):
        observation, values = super()._get_observation()
        if self.n_actor != 35 or self.actor_feature_names[:2] != CLOCK_FEATURES:
            raise RuntimeError("unified v1 requires the baseline 35D actor feature order")
        observation = observation.copy()
        values = dict(values)
        # sin(0), cos(0) would be (0, 1): v1 explicitly reserves both slots as 0.
        for index, name in enumerate(CLOCK_FEATURES):
            observation[index] = 0.0
            values[name] = 0.0
        return observation, values
