"""39D environment with selectable prescribed phase targets and one shared mask."""
from __future__ import annotations

from unified_MLP_39.env_v1 import (
    UnifiedProsthesisEnv as V1Env, CMCLikeProsthesisTrajectoryEnv,
    ENV_INVARIANTS, TARGET_COORDINATES, CLOCK_FEATURES, validate_unified_env_config,
)
from unified_MLP_39.target_availability import (
    EpisodeAvailability, target_feature_names, extend_observation, velocity_scale,
    DEFAULT_TARGET_VELOCITY_SCALE,
)
from unified_MLP_39.phase_target import (
    PrescribedLeftPhaseTarget, PrescribedRightV26PhaseTarget,
    TARGET_SOURCE, RIGHT_TARGET_SOURCE,
)


class UnifiedProsthesisEnv(V1Env):
    # Retain the LEFT default for lightweight callers constructing via __new__.
    target_reference = "left"

    def __init__(self, env_config, *, availability_probability=0.0, mask_seed=123, reward_mode="imitation",
                 target_velocity_scale=DEFAULT_TARGET_VELOCITY_SCALE, target_reference="left", target_mask_mode="stage_default"):
        if target_reference not in ("left", "right_v26"):
            raise ValueError("target_reference must be 'left' or 'right_v26'")
        self.target_reference = target_reference
        self._target_availability = EpisodeAvailability(availability_probability, mask_seed, reward_mode, target_mask_mode)
        self.target_velocity_scale = velocity_scale(target_velocity_scale)
        super().__init__(env_config)

    @property
    def actor_feature_names(self):
        return super().actor_feature_names + target_feature_names(self.target_reference)

    @property
    def observation_feature_names(self):
        base = super().observation_feature_names
        return base[:35] + target_feature_names(self.target_reference) + base[35:]

    @property
    def n_actor(self):
        return 39

    @property
    def n_obs(self):
        return len(self.observation_feature_names)

    def _initialise_episode(self):
        self._target_availability.begin_episode()
        return super()._initialise_episode()

    def reset(self, seed=None, options=None):
        self._target_availability.reset_seed(seed)
        observation, info = super().reset(seed=seed, options=options)
        info["target_availability"] = self._availability_info()
        return observation, info

    def step(self, action):
        observation, reward, terminated, truncated, info = super().step(action)
        self._target_availability.record_transition()
        info["target_availability"] = self._availability_info()
        return observation, reward, terminated, truncated, info

    def _availability_info(self):
        state = self._target_availability
        return {"visible": state.visible, "episode_probability": state.episode_probability,
                "episode_draw": state.draws, **state.transition_counts()}

    def _build_imitation_error_ranges(self):
        # Called by the shared model loader after base_kin/cfg/ctx are ready,
        # and again on model rebuild. Build before the inherited range sampler
        # dispatches to imitation_target(). Keep the shared clock disabled; this
        # wrapper supplies the selected clock through the six-channel input mask.
        if self.target_reference == "right_v26":
            self._prescribed_right_teacher = PrescribedRightV26PhaseTarget(
                self.base_kin, self._load_sound_heel_strikes("right"),
                sound_coordinates=self.env_cfg.imitation_sound_coords,
                phase_samples=self.env_cfg.imitation_phase_samples,
                time_window=(self.cfg.t_start, self.cfg.t_end),
            )
            self._prescribed_phase_teacher = self._prescribed_right_teacher
        else:
            self._prescribed_left_teacher = PrescribedLeftPhaseTarget(
                self.base_kin, self._load_sound_heel_strikes("left"),
                phase_samples=self.env_cfg.imitation_phase_samples,
                time_window=(self.cfg.t_start, self.cfg.t_end),
            )
            self._prescribed_phase_teacher = self._prescribed_left_teacher
        return super()._build_imitation_error_ranges()

    def imitation_target(self, t):
        return self._prescribed_phase_teacher.get(t)

    def _target_time_alignment(self):
        return ("prescribed_right_gait_phase" if self.target_reference == "right_v26"
                else "prescribed_left_gait_phase")

    def _imitation_target_summary(self):
        return {
            **self._prescribed_phase_teacher.summary(),
            "time_alignment": self._target_time_alignment(),
            "position_units": "rad", "velocity_units": "rad/s",
            "actor_target_observation": True,
            "actor_target_visible": self._target_availability.visible,
            "actor_clock_visible": self._target_availability.visible,
            "target_velocity_scale": self.target_velocity_scale,
            "position_ranges_rad": dict(self._imitation_position_ranges_rad),
            "velocity_ranges_rad_s": dict(self._imitation_velocity_ranges_rad_s),
        }

    def _imitation_target_payload(self):
        payload = CMCLikeProsthesisTrajectoryEnv._imitation_target_payload(self)
        source = RIGHT_TARGET_SOURCE if self.target_reference == "right_v26" else TARGET_SOURCE
        payload.update(source=source, time_alignment=self._target_time_alignment())
        return payload

    def _gait_clock_summary(self):
        if self.target_reference != "right_v26":
            return super()._gait_clock_summary()
        clock = self._prescribed_phase_teacher.clock
        return {"available": clock.available, "side": "right", "phase_offset": 0.0,
                "phase": clock.phase(self.t), "actor_visible": self._target_availability.visible}

    def _get_observation(self):
        # Keep the base class's internal 35D schema caches intact. The public
        # properties expose the new 39D prefix and 88D complete observation.
        base, _ = CMCLikeProsthesisTrajectoryEnv._get_observation(self)
        if self._n_actor != 35 or self._actor_feature_names[:2] != CLOCK_FEATURES:
            raise RuntimeError("variant39 requires the unchanged V1 actor prefix")
        q, qdot, phases = self.imitation_target(self.t)
        raw = [q[TARGET_COORDINATES[0]], qdot[TARGET_COORDINATES[0]],
               q[TARGET_COORDINATES[1]], qdot[TARGET_COORDINATES[1]]]
        observation = extend_observation(
            base, raw, self._target_availability.visible,
            phase=(self._prescribed_phase_teacher.clock.phase(self.t)
                   if self.target_reference == "right_v26" else phases[TARGET_COORDINATES[0]]),
            target_velocity_scale=self.target_velocity_scale,
        )
        return observation, dict(zip(self.observation_feature_names, map(float, observation)))
