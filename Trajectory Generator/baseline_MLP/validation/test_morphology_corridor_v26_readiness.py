"""OpenSim-free readiness gates for the additive V26 morphology corridor.

The tests exercise the real frozen V26 FSM/adapter journals and the production
reward wrapper. They do not start PPO, execute a qualifying rollout, enable a
positive morphology weight, or write/promote a checkpoint.
"""

from __future__ import annotations

import copy
import hashlib
import math
import struct
import sys
import unittest
from pathlib import Path
from typing import Any, Callable, Mapping, Sequence

import numpy as np
import yaml


REPO_ROOT = Path(__file__).resolve().parents[3]
TRAJECTORY_ROOT = REPO_ROOT / "Trajectory Generator"
BASELINE_ROOT = TRAJECTORY_ROOT / "baseline_MLP"
for root in (TRAJECTORY_ROOT, BASELINE_ROOT):
    if str(root) not in sys.path:
        sys.path.insert(0, str(root))

from binary_phase_adapter_v26 import BinaryPhaseActiveAdapterV26  # noqa: E402
from binary_phase_fsm_v26 import (  # noqa: E402
    HeelQualifiedBinaryPhaseFSM,
    HeelQualifiedBinaryPhaseFSMConfig,
    V26_EVENT_CONTRACT_ID,
)
from experimental_morphology_corridor import (  # noqa: E402
    CAUSAL_DELAYED_PHASE_MODE,
    CausalDelayedMorphologyBuffer,
    extract_v26_morphology_runtime,
)
from prosthetic_phase_fsm import (  # noqa: E402
    ProstheticPhaseFSM,
    ProstheticPhaseFSMConfig,
)
import reward_function  # noqa: E402


PROFILE = (
    BASELINE_ROOT
    / "morphology_profiles"
    / "ab06_prosthetic_event_warped_mean_std_corridor.json"
)
V25_PROFILE = (
    REPO_ROOT
    / "validation"
    / "binary_phase_detector_v25_geometry_runs"
    / "2026-08-04_local_reach_sweep_dev02_04_08"
    / "selected_candidate_profile.json"
)
V26_CANDIDATE = (
    BASELINE_ROOT
    / "experimental_configs"
    / "morphology_event_anchored_causal_v26_candidate.yaml"
)


def _samples(
    start_ms: int,
    contact_at_ms: Callable[[int], tuple[bool, bool]],
) -> list[dict[str, Any]]:
    output: list[dict[str, Any]] = []
    for millisecond in range(start_ms + 1, start_ms + 11):
        heel, toe = contact_at_ms(millisecond)
        output.append(
            {
                "time_s": millisecond / 1000.0,
                "left_heel_contact": bool(heel),
                "left_toe_contact": bool(toe),
            }
        )
    return output


def _runtime_trace(
    contact_at_ms: Callable[[int], tuple[bool, bool]],
    *,
    end_ms: int,
    baseline_contact: tuple[bool, bool] = (False, False),
    stance_timeout_s: float = 1.0,
) -> list[dict[str, Any]]:
    adapter = BinaryPhaseActiveAdapterV26()
    binary = HeelQualifiedBinaryPhaseFSM(
        HeelQualifiedBinaryPhaseFSMConfig(
            event_contract_id=V26_EVENT_CONTRACT_ID,
        )
    )
    phase = ProstheticPhaseFSM(
        ProstheticPhaseFSMConfig(
            event_source="binary_active_v26",
            min_stance_duration_s=0.02,
            min_swing_duration_s=0.02,
            stance_hard_timeout_s=stance_timeout_s,
            swing_hard_timeout_s=1.0,
            min_stance_contact_fraction=0.0,
            min_stance_load_bw_s=0.0,
            min_cycle_knee_excursion_rad=0.0,
        )
    )
    current = adapter.prime(
        binary_fsm=binary,
        phase_fsm=phase,
        time_s=0.0,
        heel_contact=baseline_contact[0],
        toe_contact=baseline_contact[1],
    )
    infos: list[dict[str, Any]] = []
    for start_ms in range(0, end_ms, 10):
        batch = _samples(start_ms, contact_at_ms)
        final_contact = bool(
            batch[-1]["left_heel_contact"] or batch[-1]["left_toe_contact"]
        )
        current = adapter.advance(
            binary_fsm=current.binary_fsm,
            phase_fsm=current.phase_fsm,
            time_s=(start_ms + 10) / 1000.0,
            previous_time_s=start_ms / 1000.0,
            sensor_samples=batch,
            normal_force_bw=0.8 if final_contact else 0.0,
            in_contact=final_contact,
            prosthetic_knee_angle_rad=-0.25,
            prosthetic_ankle_angle_rad=0.05,
        )
        boundary = (start_ms + 10) / 1000.0
        last = start_ms + 10 == end_ms
        infos.append(
            {
                "time": boundary,
                "observation": {
                    "pros_knee_angle_served_ref": 0.35 + boundary,
                    "pros_ankle_angle_served_ref": -0.70 - boundary,
                },
                "reward_terms": {
                    "tracking_loss": 0.125,
                    "terminated": 0.0,
                    "truncated": float(last),
                },
                "phase_fsm": copy.deepcopy(current.phase_payload),
                "binary_phase_fsm_mode": "binary_active",
                "binary_phase_event_contract_id": V26_EVENT_CONTRACT_ID,
                "binary_phase_fsm": copy.deepcopy(current.binary_payload),
                "binary_phase_active_adapter": copy.deepcopy(current.adapter_payload),
                "policy_segment_values": np.asarray(
                    [[-0.25, 0.05]],
                    dtype=float,
                ),
                "end_reason": "dataset_end" if last else None,
            }
        )
        if current.phase_payload["state_name"] == "TIMEOUT":
            break
    return infos


class _ScriptedV26Env(reward_function.gym.Env):
    action_space = reward_function.gym.spaces.Box(
        low=-1.0,
        high=1.0,
        shape=(2,),
        dtype=np.float32,
    )
    observation_space = reward_function.gym.spaces.Box(
        low=-np.inf,
        high=np.inf,
        shape=(1,),
        dtype=np.float32,
    )

    def __init__(self, infos: Sequence[Mapping[str, Any]]) -> None:
        super().__init__()
        self._infos = [copy.deepcopy(dict(info)) for info in infos]
        self._index = 0
        self.actions: list[np.ndarray] = []

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        self._index = 0
        self.actions = []
        return np.zeros(1, dtype=np.float32), {}

    def step(self, action):
        info = copy.deepcopy(self._infos[self._index])
        self.actions.append(np.asarray(action, dtype=float).copy())
        self._index += 1
        raw = info["reward_terms"]
        terminated = bool(raw.get("terminated", 0.0))
        truncated = bool(raw.get("truncated", 0.0))
        return (
            np.asarray([float(self._index)], dtype=np.float32),
            -99.0,
            terminated,
            truncated,
            info,
        )


def _reward_config(*, causal: bool) -> reward_function.RewardConfig:
    return reward_function.RewardConfig(
        reward_mode="ex_novo",
        blend_tracking=1.0,
        blend_reference=0.0,
        blend_bio=0.0,
        blend_contact_load=0.0,
        blend_contact_support_to=0.0,
        blend_phase_regular=0.0,
        blend_phase_event_progress=0.0,
        blend_landing_window_contact=0.0,
        effort_weight=0.0,
        smoothness_weight=0.0,
        saturation_weight=0.0,
        safety_weight=0.0,
        grf_penetration_weight=0.0,
        prosthetic_joint_range_weight=0.0,
        oob_weight=0.0,
        morphology_profile=str(PROFILE) if causal else "",
        morphology_phase_mode=(
            CAUSAL_DELAYED_PHASE_MODE if causal else "legacy_cycle_fraction"
        ),
        morphology_reward_delay_s=0.04,
        morphology_max_delivery_latency_s=0.01,
        morphology_causal_event_contract_id=V26_EVENT_CONTRACT_ID,
        morphology_causal_allow_effects=0.0,
        morphology_weight=0.0,
        morphology_hard_termination_enabled=0.0,
    )


def _run_wrapper(
    infos: Sequence[Mapping[str, Any]],
    *,
    causal: bool,
) -> tuple[list[float], list[dict[str, Any]], list[np.ndarray]]:
    env = _ScriptedV26Env(infos)
    wrapper = reward_function.RewardShapingWrapper(
        env,
        _reward_config(causal=causal),
    )
    wrapper.reset()
    rewards: list[float] = []
    outputs: list[dict[str, Any]] = []
    for index in range(len(infos)):
        action = np.asarray([0.1 + index * 0.01, -0.2], dtype=np.float32)
        _obs, reward, _terminated, _truncated, info = wrapper.step(action)
        rewards.append(float(reward))
        outputs.append(info)
    return rewards, outputs, env.actions


class MorphologyCorridorV26ReadinessTests(unittest.TestCase):
    @staticmethod
    def nominal_trace() -> list[dict[str, Any]]:
        def contact_at_ms(ms: int) -> tuple[bool, bool]:
            if ms == 10:
                return True, False  # pending HS, cancelled at 11 ms
            if 21 <= ms <= 49:
                return True, False  # accepted HS onset 21 ms
            if 50 <= ms <= 70:
                return False, False  # accepted TO onset 50 ms
            if ms >= 71:
                return True, True  # next accepted HS onset 71 ms
            return False, False

        return _runtime_trace(contact_at_ms, end_ms=90)

    def test_v26_candidate_is_promoted_post_q2_and_guard_preserved(self) -> None:
        """Governance after decision (ii) of 2026-08-21: the V26 causal corridor
        candidate is promoted post-Q2 and the active config runs it at the
        sanctioned A/B weight; the runtime Q2 guard itself is preserved."""
        candidate = yaml.safe_load(V26_CANDIDATE.read_text(encoding="utf-8"))
        active = yaml.safe_load(
            (BASELINE_ROOT / "training_exnovo_cfg.yaml").read_text(encoding="utf-8")
        )
        self.assertEqual(
            candidate["candidate"]["status"],
            "v26_promoted_post_q2_2026-08-21",
        )
        self.assertTrue(candidate["candidate"]["ppo_updates_authorized"])
        self.assertTrue(candidate["candidate"]["qualifying_rollouts_authorized"])
        self.assertTrue(candidate["candidate"]["active_training_config_replaced"])
        self.assertEqual(
            candidate["grf"]["binary_phase_event_contract_id"],
            V26_EVENT_CONTRACT_ID,
        )
        weight = float(candidate["reward"]["morphology_weight"])
        self.assertIn(weight, candidate["invariants"]["positive_ab_weights_after_q2"])
        self.assertGreater(weight, 0.0)
        self.assertEqual(candidate["reward"]["morphology_causal_allow_effects"], 1.0)
        self.assertEqual(float(active["reward"]["morphology_weight"]), weight)
        self.assertEqual(active["reward"]["morphology_causal_allow_effects"], 1.0)
        self.assertEqual(
            active["reward"]["morphology_phase_mode"],
            candidate["reward"]["morphology_phase_mode"],
        )
        self.assertEqual(active["grf"]["binary_phase_actor_fsm_version"], "v3")
        self.assertEqual(
            hashlib.sha256(PROFILE.read_bytes()).hexdigest(),
            candidate["reward"]["morphology_profile_sha256"],
        )
        self.assertEqual(
            hashlib.sha256(V25_PROFILE.read_bytes()).hexdigest(),
            candidate["grf"]["binary_phase_detector_profile_sha256"],
        )
        resolved = reward_function.RewardConfig.from_mapping(candidate["reward"])
        self.assertEqual(resolved.morphology_phase_mode, CAUSAL_DELAYED_PHASE_MODE)
        self.assertEqual(
            resolved.morphology_causal_event_contract_id,
            V26_EVENT_CONTRACT_ID,
        )
        self.assertEqual(resolved.morphology_reward_delay_s, 0.04)

        # The runtime Q2 guard is untouched: positive weight without the
        # explicit authorization flag still fails closed...
        blocked = _reward_config(causal=True)
        blocked.morphology_weight = 0.0025
        blocked.morphology_causal_allow_effects = 0.0
        with self.assertRaisesRegex(ValueError, "Q2"):
            reward_function.RewardShapingWrapper(
                _ScriptedV26Env(()),
                blocked,
            )
        # ...and the promoted configuration constructs.
        promoted = _reward_config(causal=True)
        promoted.morphology_weight = weight
        promoted.morphology_causal_allow_effects = 1.0
        reward_function.RewardShapingWrapper(_ScriptedV26Env(()), promoted)

    def test_weight_zero_is_bit_exact_and_never_multiplies_nan(self) -> None:
        cfg = _reward_config(causal=False)
        cfg.morphology_weight = 0.0
        base_reward, _ = reward_function.compute_reward(
            {"tracking_loss": 0.125},
            cfg,
        )
        with_nan, components = reward_function.compute_reward(
            {"tracking_loss": 0.125, "morphology_loss": float("nan")},
            cfg,
        )
        self.assertEqual(
            struct.pack("!d", base_reward),
            struct.pack("!d", with_nan),
        )
        self.assertTrue(math.isfinite(with_nan))
        self.assertEqual(components["morphology_term"], 0.0)
        self.assertEqual(components["morphology_loss"], 0.0)
        self.assertEqual(components["morphology_loss_input_nonfinite"], 1.0)

    def test_v26_live_wrapper_matches_offline_replay_and_zero_branch(self) -> None:
        infos = self.nominal_trace()
        causal_rewards, live, causal_actions = _run_wrapper(infos, causal=True)
        baseline_rewards, _baseline, baseline_actions = _run_wrapper(
            infos,
            causal=False,
        )
        self.assertEqual(len(causal_rewards), len(baseline_rewards))
        for causal_reward, baseline_reward in zip(
            causal_rewards,
            baseline_rewards,
        ):
            self.assertEqual(
                struct.pack("!d", causal_reward),
                struct.pack("!d", baseline_reward),
            )
        for causal_action, baseline_action in zip(
            causal_actions,
            baseline_actions,
        ):
            np.testing.assert_array_equal(causal_action, baseline_action)

        cfg = _reward_config(causal=True)
        alpha = reward_function._morphology_canonical_to_phase(
            reward_function._load_morphology_profile(PROFILE),
            require_event_contract=True,
        )
        stance_fraction = cfg.prosthetic_stance_phase_end
        offline = CausalDelayedMorphologyBuffer(
            delay_s=cfg.morphology_reward_delay_s,
            canonical_to_phase=alpha,
            nominal_stance_duration_s=(cfg.phase_period_nominal_s * stance_fraction),
            nominal_swing_duration_s=(
                cfg.phase_period_nominal_s * (1.0 - stance_fraction)
            ),
            max_delivery_latency_s=cfg.morphology_max_delivery_latency_s,
            event_contract_id=V26_EVENT_CONTRACT_ID,
        )
        offline_samples: list[tuple[float, float, bool]] = []
        for info in infos:
            runtime = extract_v26_morphology_runtime(info)
            obs = info["observation"]
            phase = info["phase_fsm"]
            use_robust = bool(
                phase["duration_history_count"] > 0.0
                and phase["robust_stance_duration_s"] > 0.0
                and phase["robust_swing_duration_s"] > 0.0
            )
            update = offline.update(
                time_s=info["time"],
                knee_rad=obs["pros_knee_angle_served_ref"],
                ankle_rad=obs["pros_ankle_angle_served_ref"],
                accepted_transitions=runtime.accepted_transitions,
                confirmed_detector_transitions=runtime.detector_transitions,
                pending_transition=runtime.pending_transition,
                cancelled_transitions=runtime.cancelled_transitions,
                episode_ended=bool(info.get("end_reason")),
                actor_state_name=runtime.actor_state_name,
                partial_stance_active=runtime.partial_stance_active,
                stance_duration_s=(
                    phase["robust_stance_duration_s"] if use_robust else None
                ),
                swing_duration_s=(
                    phase["robust_swing_duration_s"] if use_robust else None
                ),
            )
            self.assertFalse(update.failed_closed, update.failure_reason)
            offline_samples.extend(
                (
                    item.sample.time_s,
                    item.phase,
                    item.terminal_flush,
                )
                for item in update.resolved_samples
            )

        live_samples = [
            (
                item["time_s"],
                item["phase"],
                item["terminal_flush"],
            )
            for info in live
            for item in info["morphology_causal_samples"]
        ]
        self.assertEqual(len(live_samples), len(offline_samples))
        for observed, expected in zip(live_samples, offline_samples):
            self.assertAlmostEqual(observed[0], expected[0], places=12)
            self.assertAlmostEqual(observed[1], expected[1], places=12)
            self.assertEqual(observed[2], expected[2])
        self.assertGreater(len(live_samples), 0)
        self.assertTrue(
            any(
                info["morphology_causal_diagnostics"]["cancelled_transition_count"] == 1
                for info in live
            )
        )
        self.assertTrue(live[-1]["morphology_causal_diagnostics"]["terminal_flushed"])
        self.assertFalse(live[-1]["morphology_causal_diagnostics"]["failed_closed"])

    def test_wait_hs_drops_only_causally_unanchorable_prefix(self) -> None:
        infos = _runtime_trace(
            lambda _ms: (False, False),
            end_ms=70,
        )
        _rewards, outputs, _actions = _run_wrapper(infos, causal=True)
        self.assertTrue(
            any(
                output["reward_terms"]["morphology_causal_dropped_wait_hs_count"] > 0.0
                for output in outputs
            )
        )
        self.assertEqual(
            [
                sample
                for output in outputs
                for sample in output["morphology_causal_samples"]
            ],
            [],
        )
        self.assertTrue(
            outputs[-1]["morphology_causal_diagnostics"]["terminal_flushed"]
        )
        self.assertFalse(outputs[-1]["morphology_causal_diagnostics"]["failed_closed"])

    def test_v26_partial_stance_to_opens_swing_without_fabricated_hs(self) -> None:
        infos = _runtime_trace(
            lambda _ms: (False, False),
            end_ms=30,
            baseline_contact=(False, True),
        )
        first = extract_v26_morphology_runtime(infos[0])
        self.assertEqual(len(first.accepted_transitions), 1)
        transition = first.accepted_transitions[0]
        self.assertEqual(transition["event"], "toe_off")
        self.assertEqual(transition["segment_valid"], 0.0)
        self.assertEqual(transition["from_state_id"], 1.0)
        self.assertEqual(transition["to_state_id"], 2.0)
        _rewards, outputs, _actions = _run_wrapper(infos, causal=True)
        self.assertFalse(outputs[-1]["morphology_causal_diagnostics"]["failed_closed"])
        for output in outputs:
            for sample in output["morphology_causal_samples"]:
                self.assertEqual(sample["segment_type"], "swing")
                self.assertGreaterEqual(sample["time_s"], transition["event_time_s"])

    def test_v26_timeout_forces_terminal_flush(self) -> None:
        infos = _runtime_trace(
            lambda _ms: (True, False),
            end_ms=80,
            stance_timeout_s=0.03,
        )
        self.assertEqual(infos[-1]["phase_fsm"]["state_name"], "TIMEOUT")
        infos[-1]["reward_terms"]["truncated"] = 0.0
        infos[-1]["end_reason"] = None
        _rewards, outputs, _actions = _run_wrapper(infos, causal=True)
        terminal = outputs[-1]["morphology_causal_diagnostics"]
        self.assertTrue(terminal["terminal_flushed"])
        self.assertEqual(terminal["timeout_transition_count"], 1)
        self.assertFalse(terminal["failed_closed"])

    def test_terminal_flush_drops_only_v26_pending_onset_tail(self) -> None:
        infos = _runtime_trace(
            lambda ms: (ms < 50, False),
            end_ms=50,
        )
        pending = infos[-1]["binary_phase_fsm"]["pending_event"]
        self.assertEqual(pending["event"], "toe_off")
        self.assertAlmostEqual(pending["event_time_s"], 0.05)
        _rewards, outputs, _actions = _run_wrapper(infos, causal=True)
        terminal = outputs[-1]["morphology_causal_diagnostics"]
        self.assertTrue(terminal["terminal_flushed"])
        self.assertEqual(terminal["dropped_pending_sample_count"], 1)
        self.assertEqual(terminal["drop_reason"], "episode_end_pending_transition")
        self.assertEqual(terminal["pending_sample_count"], 0)
        terminal_times = [
            sample["time_s"]
            for sample in outputs[-1]["morphology_causal_samples"]
            if sample["terminal_flush"]
        ]
        self.assertEqual(terminal_times, [0.02, 0.03, 0.04])

    def test_cross_lineage_runtime_is_rejected_without_reward_nan(self) -> None:
        info = copy.deepcopy(self.nominal_trace()[0])
        info["phase_fsm"]["event_source"] = "binary_active"
        with self.assertRaisesRegex(ValueError, "v26_actor_event_source_mismatch"):
            extract_v26_morphology_runtime(info)
        _rewards, outputs, _actions = _run_wrapper((info,), causal=True)
        diagnostics = outputs[0]["morphology_causal_diagnostics"]
        self.assertTrue(diagnostics["failed_closed"])
        self.assertIn("v26_runtime:", diagnostics["failure_reason"])
        self.assertTrue(math.isfinite(outputs[0]["reward_components"]["reward"]))


if __name__ == "__main__":
    unittest.main()
