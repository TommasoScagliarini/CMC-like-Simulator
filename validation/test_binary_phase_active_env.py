"""Environment-level contract tests for the V25/V20 active event path.

The tests use a lightweight, OpenSim-free environment shell.  They exercise
configuration/routing and the atomic policy-boundary commit without running a
simulation, H0, PPO, or any protected trial.
"""

from __future__ import annotations

import copy
import inspect
import sys
import unittest
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace


REPO_ROOT = Path(__file__).resolve().parents[1]
TRAJECTORY_ROOT = REPO_ROOT / "Trajectory Generator"
for import_root in (REPO_ROOT, TRAJECTORY_ROOT):
    if str(import_root) not in sys.path:
        sys.path.insert(0, str(import_root))

from binary_phase_adapter import (  # noqa: E402
    BINARY_ACTIVE_ADAPTER_SOURCE,
    BINARY_ACTIVE_EVENT_CONTRACT_ID,
    BinaryPhaseActiveAdapter,
)
from binary_phase_fsm import BinaryPhaseFSM, BinaryPhaseFSMConfig  # noqa: E402
from osim_trj_cmc_like import (  # noqa: E402
    CMCEnvConfig,
    CMCLikeProsthesisTrajectoryEnv,
    _validate_binary_phase_active_configuration,
)
from prosthetic_phase_fsm import (  # noqa: E402
    STANCE_AFTER_HS,
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
V19_PROFILE = (
    REPO_ROOT
    / "validation"
    / "experimental_detector_profiles"
    / "two_point_binary_v19_outsole_25mm.json"
)
ANALOG_PROFILE = (
    REPO_ROOT
    / "online_grf_profiles"
    / "AB06_SEASEA_stiff321_500_pi_grf_detector_HS-TO.json"
)
SHADOW_CONTRACT = "binary_point_v25+functional_contact_fsm_v1_shadow"


def samples(
    *,
    heel: bool,
    toe: bool,
    start_ms: int = 0,
) -> list[dict[str, object]]:
    return [
        {
            "time_s": index / 1000.0,
            "left_heel_contact": heel,
            "left_toe_contact": toe,
        }
        for index in range(start_ms + 1, start_ms + 11)
    ]


class _StateValues:
    def __init__(self) -> None:
        self._values = {"knee": -0.25, "ankle": 0.05}

    def get(self, index):
        return self._values[index]


class _Model:
    @staticmethod
    def getStateVariableValues(_state):
        return _StateValues()


class _MalformedEventBinaryFSM(BinaryPhaseFSM):
    """Produce a valid detector transition with a deliberately bad source."""

    def update_policy_step(self, **kwargs):
        payload = super().update_policy_step(**kwargs)
        for event in payload["events_this_step"]:
            event["source"] = "malformed_non_v20_source"
        return payload


class BinaryPhaseActiveEnvTests(unittest.TestCase):
    @staticmethod
    def active_config() -> CMCEnvConfig:
        return CMCEnvConfig(
            phase_fsm_input_mode="legacy_events",
            event_contract_id="legacy_events_v1",
            online_grf_detector_profile_file=str(ANALOG_PROFILE),
            binary_phase_detector_profile_file=str(V25_PROFILE),
            binary_phase_fsm_mode="binary_active",
            binary_phase_event_contract_id=BINARY_ACTIVE_EVENT_CONTRACT_ID,
            binary_phase_debounce_s=0.005,
            detector_sample_dt_s=0.001,
            segment_duration=0.01,
        )

    @staticmethod
    def analog_routes() -> tuple[tuple[str, ...], dict[str, str]]:
        names = (
            "online_grf_detector_left_heel",
            "online_grf_detector_left_toe",
            "online_grf_detector_right_heel",
            "online_grf_detector_right_toe",
        )
        return (
            tuple(f"/forceset/{name}" for name in names),
            {
                name: ("left" if "_left_" in name else "right")
                for name in names
            },
        )

    def active_env(
        self,
        *,
        baseline_heel: bool = False,
        baseline_toe: bool = False,
        sample_heel: bool = True,
        sample_toe: bool = False,
        legacy_events=(),
        primary_in_contact: bool | None = None,
    ):
        env = CMCLikeProsthesisTrajectoryEnv.__new__(
            CMCLikeProsthesisTrajectoryEnv
        )
        env.env_cfg = self.active_config()
        adapter = BinaryPhaseActiveAdapter()
        # Accept either private spelling while keeping the test focused on the
        # published environment entry point.
        env._binary_phase_adapter = adapter
        env._binary_phase_active_adapter = adapter
        env._binary_phase_fsm = BinaryPhaseFSM(
            BinaryPhaseFSMConfig(
                event_contract_id=BINARY_ACTIVE_EVENT_CONTRACT_ID
            )
        )
        env._phase_fsm = ProstheticPhaseFSM(
            ProstheticPhaseFSMConfig(event_source="binary_active")
        )
        env._binary_phase_fsm_payload = env._binary_phase_fsm.payload()
        env._phase_fsm_payload = env._phase_fsm.payload()
        env._binary_phase_sensor_previous_time_s = 0.0
        env._binary_phase_sensor_samples = samples(
            heel=sample_heel,
            toe=sample_toe,
        )
        env._legacy_online_events = [copy.deepcopy(item) for item in legacy_events]
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
                    "contacts": {
                        "left_heel": baseline_heel,
                        "left_toe": baseline_toe,
                    },
                }
            },
        )
        contact = (
            bool(sample_heel or sample_toe)
            if primary_in_contact is None
            else bool(primary_in_contact)
        )
        normal_force = 800.0 if contact else 0.0
        env._physical_online_grf_sides = lambda required=False: {
            "left": {
                "normal_force": normal_force,
                "in_contact": contact,
            }
        }
        env._prime_binary_phase_fsm_from_runner()
        env.t = 0.010
        return env

    def test_active_configuration_accepts_only_exact_v25_contract(self) -> None:
        cfg = self.active_config()
        result = _validate_binary_phase_active_configuration(
            cfg,
            detector_dt=cfg.detector_sample_dt_s,
        )
        self.assertIsNotNone(result)

        env = CMCLikeProsthesisTrajectoryEnv.__new__(
            CMCLikeProsthesisTrajectoryEnv
        )
        env.env_cfg = cfg
        self.assertEqual(env._phase_fsm_config().event_source, "binary_active")

        paths, sides = self.analog_routes()
        env.ctx = SimpleNamespace(
            binary_phase_detector_profile=SimpleNamespace(
                points=(
                    SimpleNamespace(name="left_heel"),
                    SimpleNamespace(name="left_toe"),
                )
            ),
            online_grf_detector_force_paths=paths,
            online_grf_detector_force_sides=sides,
        )
        env._validate_binary_phase_sensor_setup()

    def test_active_configuration_rejects_nonconforming_profile_or_contract(
        self,
    ) -> None:
        valid = self.active_config()
        invalid = {
            "shadow_contract": replace(
                valid,
                binary_phase_event_contract_id=SHADOW_CONTRACT,
            ),
            "v19_profile": replace(
                valid,
                binary_phase_detector_profile_file=str(V19_PROFILE),
            ),
            "missing_binary_profile": replace(
                valid,
                binary_phase_detector_profile_file=None,
            ),
            "missing_legacy_analog": replace(
                valid,
                online_grf_detector_profile_file=None,
            ),
            "wrong_legacy_contract": replace(
                valid,
                event_contract_id="primary_grf_split_v1",
            ),
            "wrong_debounce": replace(valid, binary_phase_debounce_s=0.004),
        }
        for label, cfg in invalid.items():
            with self.subTest(label=label):
                with self.assertRaises(ValueError):
                    _validate_binary_phase_active_configuration(
                        cfg,
                        detector_dt=cfg.detector_sample_dt_s,
                    )

    def test_active_routes_only_v20_left_and_preserves_right_legacy(self) -> None:
        legacy_left = {
            "side": "left",
            "event": "heel_strike",
            "time": 0.004,
            "source": "legacy_analog",
        }
        legacy_right = {
            "side": "right",
            "event": "toe_off",
            "time": 0.005,
            "source": "legacy_analog",
        }
        env = self.active_env(legacy_events=(legacy_left, legacy_right))
        legacy_before = copy.deepcopy(env._legacy_online_events)

        env._update_binary_active_phase_path(
            require_batch=True,
            require_primary_sample=True,
        )

        self.assertEqual(env._legacy_online_events, legacy_before)
        left = [event for event in env._online_events if event["side"] == "left"]
        right = [event for event in env._online_events if event["side"] == "right"]
        self.assertEqual(len(left), 1)
        self.assertEqual(left[0]["event"], "heel_strike")
        self.assertEqual(left[0]["source"], BINARY_ACTIVE_ADAPTER_SOURCE)
        self.assertEqual(
            left[0]["event_contract_id"],
            BINARY_ACTIVE_EVENT_CONTRACT_ID,
        )
        self.assertNotIn(legacy_left, env._online_events)
        self.assertEqual(right, [legacy_right])
        self.assertEqual(env._phase_fsm_payload["state_id"], float(STANCE_AFTER_HS))
        self.assertEqual(env._phase_fsm_payload["valid_hs_count"], 1.0)
        self.assertEqual(
            env._online_gait_sides["left"]["last_heel_strike_time"],
            0.001,
        )
        self.assertEqual(
            env._online_gait_sides["right"]["last_toe_off_time"],
            0.005,
        )

    def test_active_event_is_delivered_before_same_policy_step_observation(
        self,
    ) -> None:
        env = self.active_env()
        env._update_binary_active_phase_path(
            require_batch=True,
            require_primary_sample=True,
        )

        self.assertEqual(env._binary_phase_sensor_previous_time_s, 0.010)
        transition = env._phase_fsm_payload["accepted_transitions_this_step"]
        self.assertEqual(len(transition), 1)
        self.assertAlmostEqual(transition[0]["event_time_s"], 0.001, places=12)
        self.assertAlmostEqual(
            transition[0]["confirmed_time_s"], 0.006, places=12
        )
        self.assertAlmostEqual(
            transition[0]["delivered_time_s"], 0.010, places=12
        )
        self.assertEqual(env._phase_fsm.observation()["phase_fsm_stance_after_hs"], 1.0)
        self.assertEqual(env._online_events[0]["event"], "heel_strike")

        step_source = inspect.getsource(CMCLikeProsthesisTrajectoryEnv.step)
        self.assertLess(
            step_source.index("_update_binary_active_phase_path"),
            step_source.index("_get_observation"),
        )

    def test_active_has_no_left_legacy_or_primary_contact_fallback(self) -> None:
        legacy_left = {
            "side": "left",
            "event": "heel_strike",
            "time": 0.004,
            "source": "prescribed_or_analog",
        }
        env = self.active_env(
            sample_heel=False,
            sample_toe=False,
            legacy_events=(legacy_left,),
            primary_in_contact=True,
        )
        env._update_binary_active_phase_path(
            require_batch=True,
            require_primary_sample=True,
        )

        self.assertEqual(env._binary_phase_fsm_payload["events_this_step"], [])
        self.assertEqual(env._phase_fsm_payload["state_id"], float(WAIT_HS))
        self.assertEqual(env._phase_fsm_payload["valid_hs_count"], 0.0)
        self.assertEqual(
            [event for event in env._online_events if event["side"] == "left"],
            [],
        )
        self.assertEqual(env._legacy_online_events, [legacy_left])

    def assert_failed_update_is_atomic(self, env, exception_type) -> None:
        binary_object = env._binary_phase_fsm
        phase_object = env._phase_fsm
        binary_before = copy.deepcopy(binary_object.payload())
        phase_before = copy.deepcopy(phase_object.payload())
        binary_payload_before = copy.deepcopy(env._binary_phase_fsm_payload)
        phase_payload_before = copy.deepcopy(env._phase_fsm_payload)
        boundary_before = env._binary_phase_sensor_previous_time_s
        active_before = copy.deepcopy(env._online_events)
        gait_before = copy.deepcopy(env._online_gait_sides)

        with self.assertRaises(exception_type):
            env._update_binary_active_phase_path(
                require_batch=True,
                require_primary_sample=True,
            )

        self.assertIs(env._binary_phase_fsm, binary_object)
        self.assertIs(env._phase_fsm, phase_object)
        self.assertEqual(env._binary_phase_fsm.payload(), binary_before)
        self.assertEqual(env._phase_fsm.payload(), phase_before)
        self.assertEqual(env._binary_phase_fsm_payload, binary_payload_before)
        self.assertEqual(env._phase_fsm_payload, phase_payload_before)
        self.assertEqual(env._binary_phase_sensor_previous_time_s, boundary_before)
        self.assertEqual(env._online_events, active_before)
        self.assertEqual(env._online_gait_sides, gait_before)

    def test_active_malformed_batch_and_event_roll_back_atomically(self) -> None:
        malformed_batch = self.active_env()
        malformed_batch._binary_phase_sensor_samples[4][
            "left_heel_contact"
        ] = 1
        self.assert_failed_update_is_atomic(malformed_batch, TypeError)

        malformed_event = self.active_env()
        bad_binary = _MalformedEventBinaryFSM(
            BinaryPhaseFSMConfig(
                event_contract_id=BINARY_ACTIVE_EVENT_CONTRACT_ID
            )
        )
        bad_binary.reset(
            time_s=0.0,
            heel_contact=False,
            toe_contact=False,
        )
        malformed_event._binary_phase_fsm = bad_binary
        malformed_event._binary_phase_fsm_payload = bad_binary.payload()
        self.assert_failed_update_is_atomic(malformed_event, ValueError)


if __name__ == "__main__":
    unittest.main()
