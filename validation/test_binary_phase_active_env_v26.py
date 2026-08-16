from __future__ import annotations

import copy
import sys
import unittest
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace


REPO_ROOT = Path(__file__).resolve().parents[1]
TRAJECTORY_ROOT = REPO_ROOT / "Trajectory Generator"
for root in (REPO_ROOT, TRAJECTORY_ROOT):
    if str(root) not in sys.path:
        sys.path.insert(0, str(root))

from binary_phase_adapter import BinaryPhaseActiveAdapter  # noqa: E402
from binary_phase_adapter_v26 import (  # noqa: E402
    BinaryPhaseActiveAdapterV26,
)
from binary_phase_fsm_v26 import (  # noqa: E402
    HeelQualifiedBinaryPhaseFSM,
    HeelQualifiedBinaryPhaseFSMConfig,
    V26_EVENT_CONTRACT_ID,
)
from osim_trj_cmc_like import (  # noqa: E402
    CMCEnvConfig,
    CMCLikeProsthesisTrajectoryEnv,
    _validate_binary_phase_active_configuration,
)
from prosthetic_phase_fsm import (  # noqa: E402
    BINARY_ACTIVE_V26_ADAPTER_SOURCE,
    WAIT_HS,
    ProstheticPhaseFSM,
    ProstheticPhaseFSMConfig,
)


V25_PROFILE = (
    REPO_ROOT
    / "validation"
    / "binary_phase_detector_v25_geometry_runs"
    / "2026-08-04_local_reach_sweep_dev02_04_08"
    / "selected_candidate_profile.json"
)
ANALOG_PROFILE = (
    REPO_ROOT
    / "online_grf_profiles"
    / "AB06_SEASEA_stiff321_500_pi_grf_detector_HS-TO.json"
)


def detector_samples(heel: bool, toe: bool) -> list[dict]:
    return [
        {
            "time_s": index / 1000,
            "left_heel_contact": heel,
            "left_toe_contact": toe,
        }
        for index in range(1, 11)
    ]


class _StateValues:
    @staticmethod
    def get(index):
        return {"knee": -0.25, "ankle": 0.05}[index]


class _Model:
    @staticmethod
    def getStateVariableValues(_state):
        return _StateValues()


class BinaryPhaseActiveEnvV26Tests(unittest.TestCase):
    @staticmethod
    def config() -> CMCEnvConfig:
        return CMCEnvConfig(
            phase_fsm_input_mode="legacy_events",
            event_contract_id="legacy_events_v1",
            online_grf_detector_profile_file=str(ANALOG_PROFILE),
            binary_phase_detector_profile_file=str(V25_PROFILE),
            binary_phase_fsm_mode="binary_active",
            binary_phase_event_contract_id=V26_EVENT_CONTRACT_ID,
            binary_phase_debounce_s=0.005,
            detector_sample_dt_s=0.001,
            segment_duration=0.01,
            phase_min_swing_duration_s=0.25,
        )

    def shell(
        self,
        *,
        heel: bool,
        toe: bool,
        legacy_events=(),
        primary_in_contact: bool = False,
    ):
        env = CMCLikeProsthesisTrajectoryEnv.__new__(
            CMCLikeProsthesisTrajectoryEnv
        )
        env.env_cfg = self.config()
        env._binary_phase_fsm = HeelQualifiedBinaryPhaseFSM(
            HeelQualifiedBinaryPhaseFSMConfig(
                event_contract_id=V26_EVENT_CONTRACT_ID
            )
        )
        env._phase_fsm = ProstheticPhaseFSM(
            ProstheticPhaseFSMConfig(
                event_source="binary_active_v26",
                min_swing_duration_s=0.25,
            )
        )
        env._binary_phase_active_adapter = BinaryPhaseActiveAdapterV26()
        env._binary_phase_fsm_payload = env._binary_phase_fsm.payload()
        env._phase_fsm_payload = env._phase_fsm.payload()
        env._binary_phase_active_adapter_payload = {}
        env._binary_phase_sensor_previous_time_s = 0.0
        env._binary_phase_sensor_samples = detector_samples(heel, toe)
        env._binary_phase_sensor_baseline = {}
        env._legacy_online_events = [copy.deepcopy(x) for x in legacy_events]
        env._online_events = []
        env._online_gait_sides = {
            side: {
                "last_heel_strike_time": None,
                "last_toe_off_time": None,
                "cycle_duration_s": 0.0,
            }
            for side in ("left", "right")
        }
        env._body_weight_n = 1000.0
        env._online_grf = {}
        env._online_grf_detector = {}
        env.t = 0.0
        env.ctx = SimpleNamespace(
            model=_Model(),
            q_sv_idx={
                "pros_knee_angle": "knee",
                "pros_ankle_angle": "ankle",
            },
            online_grf_force_paths=(),
        )
        env.runner = SimpleNamespace(
            state=object(),
            last_step_info={
                "binary_phase_sensor_baseline": {
                    "time_s": 0.0,
                    "contacts": {"left_heel": False, "left_toe": False},
                }
            },
        )
        env._physical_online_grf_sides = lambda required=False: {
            "left": {
                "normal_force": 800.0 if primary_in_contact else 0.0,
                "in_contact": primary_in_contact,
            }
        }
        env._prime_binary_phase_fsm_from_runner()
        env.t = 0.010
        return env

    def test_configuration_dispatches_exact_v26_tuple(self) -> None:
        cfg = self.config()
        _validate_binary_phase_active_configuration(
            cfg,
            detector_dt=cfg.detector_sample_dt_s,
        )
        env = CMCLikeProsthesisTrajectoryEnv.__new__(
            CMCLikeProsthesisTrajectoryEnv
        )
        env.env_cfg = cfg
        self.assertEqual(env._phase_fsm_config().event_source, "binary_active_v26")
        self.assertIsInstance(
            env._binary_phase_fsm_config(),
            HeelQualifiedBinaryPhaseFSMConfig,
        )

        invalid = replace(cfg, binary_phase_event_contract_id="unfrozen_v26")
        with self.assertRaises(ValueError):
            _validate_binary_phase_active_configuration(
                invalid,
                detector_dt=invalid.detector_sample_dt_s,
            )

    def test_toe_only_is_not_an_event_even_with_primary_load(self) -> None:
        legacy_left = {
            "side": "left",
            "event": "heel_strike",
            "time": 0.004,
            "source": "legacy",
        }
        env = self.shell(
            heel=False,
            toe=True,
            legacy_events=(legacy_left,),
            primary_in_contact=True,
        )
        env._update_binary_active_phase_path()
        self.assertEqual(env._binary_phase_fsm_payload["raw_contact_state"], "TOE")
        self.assertFalse(env._binary_phase_fsm_payload["in_contact"])
        self.assertEqual(env._phase_fsm_payload["state_id"], float(WAIT_HS))
        self.assertEqual(env._online_events, [])
        self.assertEqual(env._legacy_online_events, [legacy_left])

    def test_heel_event_uses_v26_and_right_legacy_is_preserved(self) -> None:
        legacy_right = {
            "side": "right",
            "event": "toe_off",
            "time": 0.004,
            "source": "legacy",
        }
        env = self.shell(
            heel=True,
            toe=False,
            legacy_events=(legacy_right,),
            primary_in_contact=True,
        )
        env._update_binary_active_phase_path()
        left = [x for x in env._online_events if x["side"] == "left"]
        right = [x for x in env._online_events if x["side"] == "right"]
        self.assertEqual(len(left), 1)
        self.assertEqual(left[0]["event"], "heel_strike")
        self.assertEqual(left[0]["source"], BINARY_ACTIVE_V26_ADAPTER_SOURCE)
        self.assertEqual(left[0]["event_contract_id"], V26_EVENT_CONTRACT_ID)
        self.assertEqual(right, [legacy_right])
        self.assertEqual(
            env._binary_phase_active_adapter_payload["actor_event_source"],
            "binary_active_v26",
        )

    def test_cross_lineage_adapter_fails_without_partial_commit(self) -> None:
        env = self.shell(heel=True, toe=False)
        env._binary_phase_active_adapter = BinaryPhaseActiveAdapter()
        binary_before = copy.deepcopy(env._binary_phase_fsm.payload())
        phase_before = copy.deepcopy(env._phase_fsm.payload())
        with self.assertRaises((TypeError, ValueError)):
            env._update_binary_active_phase_path()
        self.assertEqual(env._binary_phase_fsm.payload(), binary_before)
        self.assertEqual(env._phase_fsm.payload(), phase_before)
        self.assertEqual(env._online_events, [])


if __name__ == "__main__":
    unittest.main()
