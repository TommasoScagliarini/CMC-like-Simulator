"""39D extension of the preserved V1 environment, materialized only in phase 5."""
from __future__ import annotations

from unified_MLP_39.env_v1 import (
    UnifiedProsthesisEnv as V1Env, CMCLikeProsthesisTrajectoryEnv,
    ENV_INVARIANTS, TARGET_COORDINATES, CLOCK_FEATURES, validate_unified_env_config,
)
from unified_MLP_39.target_availability import (
    EpisodeAvailability, TARGET_FEATURES, extend_observation,
)


class UnifiedProsthesisEnv(V1Env):
    def __init__(self, env_config, *, availability_probability=0.0, mask_seed=123, reward_mode="imitation"):
        self._target_availability = EpisodeAvailability(availability_probability, mask_seed, reward_mode)
        super().__init__(env_config)

    @property
    def actor_feature_names(self):
        return super().actor_feature_names + TARGET_FEATURES

    @property
    def observation_feature_names(self):
        base = super().observation_feature_names
        return base[:35] + TARGET_FEATURES + base[35:]

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

    def _imitation_target_summary(self):
        result = super()._imitation_target_summary()
        result["actor_target_observation"] = True
        result["actor_target_visible"] = self._target_availability.visible
        return result

    def _get_observation(self):
        # Keep the base class's internal 35D schema caches intact. The public
        # properties expose the new 39D prefix and 88D complete observation.
        base, _ = CMCLikeProsthesisTrajectoryEnv._get_observation(self)
        if self._n_actor != 35 or self._actor_feature_names[:2] != CLOCK_FEATURES:
            raise RuntimeError("variant39 requires the unchanged V1 actor prefix")
        q, qdot, _ = self.imitation_target(self.t)
        raw = [q[TARGET_COORDINATES[0]], qdot[TARGET_COORDINATES[0]],
               q[TARGET_COORDINATES[1]], qdot[TARGET_COORDINATES[1]]]
        observation = extend_observation(base, raw, self._target_availability.visible)
        return observation, dict(zip(self.observation_feature_names, map(float, observation)))
