"""Validate the strictly separated primary-GRF and detector data paths.

The virtual heel/toe contacts remain sensor-only.  Aggregate load/contact
evidence comes from the force-applying primary GRF, while detector channels may
only drive their local event path.
"""

from __future__ import annotations

import ast
import json
import sys
import unittest
from unittest import mock
from pathlib import Path
from types import SimpleNamespace

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))
TRAJECTORY_GENERATOR = REPO_ROOT / "Trajectory Generator"
if str(TRAJECTORY_GENERATOR) not in sys.path:
    sys.path.insert(0, str(TRAJECTORY_GENERATOR))

import online_grf  # noqa: E402
import simulation_runner  # noqa: E402
from model_loader import (  # noqa: E402
    _primary_profile_required_sides,
    _validate_hybrid_prescribed_support,
)
from online_grf import (  # noqa: E402
    flatten_online_grf,
    load_online_grf_profile,
    online_grf_sensor_channels,
    online_grf_sensor_role,
    read_online_grf,
)
from simulation_runner import SimulationRunner  # noqa: E402
from osim_trj_cmc_like import (  # noqa: E402
    CMCEnvConfig,
    CMCLikeProsthesisTrajectoryEnv,
)


DETECTOR_PROFILE = (
    REPO_ROOT
    / "online_grf_profiles"
    / "AB06_SEASEA_stiff321_500_pi_grf_detector_HS-TO.json"
)
V17_DETECTOR_PROFILE = (
    REPO_ROOT
    / "validation"
    / "experimental_detector_profiles"
    / "two_sensor_v17_high_rate_v13_geometry.json"
)


def _side_payload(normal_force: float, penetration: float) -> dict:
    return {
        "force": np.asarray([0.0, normal_force, 0.0]),
        "moment": np.zeros(3),
        "cop": np.asarray([0.1, 0.0, -0.1]),
        "normal_force": float(normal_force),
        "penetration": float(penetration),
        "slip_speed": 0.0,
        "in_contact": bool(penetration > 0.0),
    }


def _synthetic_grf() -> dict:
    return {
        "sides": {
            "left": _side_payload(32.0, 0.002),
            "right": _side_payload(0.0, 0.0),
        },
        "spheres": {
            "online_grf_detector_left_heel": {
                "side": "left",
                "normal_force": 8.0,
                "penetration": 0.0,
                "in_contact": False,
            },
            "online_grf_detector_left_toe": {
                "side": "left",
                "normal_load_n": 24.0,
                "penetration_m": 0.002,
                "in_contact": True,
            },
        },
    }


class DetectorSensorDataPathTests(unittest.TestCase):
    def test_high_rate_env_contract_rejects_parameter_or_profile_drift(self) -> None:
        contract = dict(
            phase_fsm_input_mode="two_sensor",
            event_contract_id="primary_grf_split_v1+two_sensor_highrate_v1",
            online_grf_detector_profile_file=str(V17_DETECTOR_PROFILE),
            phase_sensor_on_threshold_n=0.5,
            phase_sensor_off_threshold_n=0.25,
            phase_sensor_dwell_s=0.03,
            detector_sample_dt_s=0.001,
            segment_duration=0.01,
        )
        invalid = (
            ({**contract, "phase_sensor_on_threshold_n": 5.0}, "on/off/dwell"),
            ({**contract, "segment_duration": 0.05}, "segment_duration"),
            ({**contract, "detector_sample_dt_s": 0.002}, "ten detector samples"),
            ({**contract, "event_contract_id": "legacy_events_v1"}, "event_contract_id"),
            (
                {**contract, "online_grf_detector_profile_file": str(DETECTOR_PROFILE)},
                "profile hash mismatch",
            ),
        )
        for values, message in invalid:
            with self.subTest(values=values):
                with self.assertRaisesRegex(ValueError, message):
                    CMCLikeProsthesisTrajectoryEnv(
                        env_config=CMCEnvConfig(**values)
                    )

    def test_prefixed_components_resolve_to_explicit_sensor_roles(self) -> None:
        self.assertEqual(
            online_grf_sensor_role("online_grf_detector_left_heel", "left"),
            "left_heel",
        )
        self.assertEqual(
            online_grf_sensor_role("online_grf_detector_left_toe", "left"),
            "left_toe",
        )
        self.assertIsNone(
            online_grf_sensor_role("online_grf_detector_left_midfoot", "left")
        )

    def test_per_sphere_channels_keep_heel_and_toe_separate(self) -> None:
        channels = online_grf_sensor_channels(_synthetic_grf())
        self.assertEqual(set(channels), {"left_heel", "left_toe"})
        self.assertEqual(channels["left_heel"]["normal_load_n"], 8.0)
        self.assertEqual(channels["left_heel"]["penetration_m"], 0.0)
        self.assertFalse(channels["left_heel"]["in_contact"])
        self.assertEqual(channels["left_toe"]["normal_load_n"], 24.0)
        self.assertEqual(channels["left_toe"]["penetration_m"], 0.002)
        self.assertTrue(channels["left_toe"]["in_contact"])

    def test_nonfinite_sensor_channel_fails_closed(self) -> None:
        grf = _synthetic_grf()
        grf["spheres"]["online_grf_detector_left_heel"][
            "normal_force"
        ] = float("nan")
        with self.assertRaises(FloatingPointError):
            online_grf_sensor_channels(grf)

    def test_read_online_grf_propagates_component_records_per_sensor(self) -> None:
        class FakeForce:
            def __init__(self, name: str) -> None:
                self._name = name

            def getName(self) -> str:
                return self._name

        class FakeForceType:
            @staticmethod
            def safeDownCast(component):
                return component

        class FakeModel:
            def __init__(self, components: dict[str, FakeForce]) -> None:
                self.components = components

            def realizeDynamics(self, _state) -> None:
                return None

            def getComponent(self, path: str) -> FakeForce:
                return self.components[path]

        heel_path = "/forceset/online_grf_detector_left_heel"
        toe_path = "/forceset/online_grf_detector_left_toe"
        heel = FakeForce("online_grf_detector_left_heel")
        toe = FakeForce("online_grf_detector_left_toe")
        model = FakeModel({heel_path: heel, toe_path: toe})

        def record_values(force, _state) -> np.ndarray:
            values = np.zeros(18, dtype=float)
            if force is heel:
                values[1] = 7.5
                values[9] = 7.5
                values[10] = 0.001
            else:
                values[1] = 21.0
                values[9] = 21.0
                values[10] = 0.003
            values[15:18] = [0.0, 1.0, 0.0]
            return values

        with (
            mock.patch.object(online_grf.opensim, "Force", FakeForceType),
            mock.patch.object(online_grf, "_record_values", record_values),
        ):
            grf = read_online_grf(
                model,
                object(),
                [heel_path, toe_path],
                {heel.getName(): "left", toe.getName(): "left"},
            )

        self.assertEqual(grf["sides"]["left"]["normal_force"], 28.5)
        self.assertEqual(grf["sensors"]["left_heel"]["normal_load_n"], 7.5)
        self.assertEqual(grf["sensors"]["left_toe"]["normal_load_n"], 21.0)
        self.assertEqual(grf["sensors"]["left_heel"]["penetration_m"], 0.001)
        self.assertEqual(grf["sensors"]["left_toe"]["penetration_m"], 0.003)
        self.assertTrue(grf["sensors"]["left_heel"]["in_contact"])
        self.assertTrue(grf["sensors"]["left_toe"]["in_contact"])
        self.assertTrue(np.all(np.isfinite(grf["sides"]["right"]["cop"])))
        self.assertTrue(np.allclose(grf["sides"]["right"]["cop"], 0.0))

    def test_runner_payload_is_additive_and_json_safe(self) -> None:
        grf = _synthetic_grf()
        info = SimulationRunner._online_grf_info(grf)

        # Legacy aggregate payload remains available with the same field names.
        self.assertEqual(info["left"]["normal_force"], 32.0)
        self.assertEqual(info["left"]["penetration"], 0.002)
        self.assertTrue(info["left"]["in_contact"])
        self.assertEqual(info["right"]["normal_force"], 0.0)

        self.assertEqual(
            set(info["sensors"]),
            {"left_heel", "left_toe"},
        )
        self.assertEqual(info["sensors"]["left_heel"]["kind"], "heel")
        self.assertEqual(info["sensors"]["left_toe"]["kind"], "toe")
        json.dumps(info, allow_nan=True)

        # The additive per-sensor view does not alter the legacy flat recorder.
        self.assertEqual(flatten_online_grf(grf).shape, (26,))

    def test_left_only_detector_does_not_zero_right_event_force(self) -> None:
        primary_grf = {
            "sides": {
                "left": _side_payload(120.0, 0.002),
                "right": _side_payload(310.0, 0.003),
            },
            "spheres": {},
        }
        detector_grf = _synthetic_grf()
        event_detector = mock.Mock()
        event_detector.update.return_value = []

        runner = object.__new__(SimulationRunner)
        runner._ctx = SimpleNamespace(
            model=object(),
            grf_mode="prescribed",
            online_grf_force_paths=["/forceset/primary_left"],
            online_grf_force_sides={"primary_left": "left"},
            online_grf_detector_force_paths=[
                "/forceset/online_grf_detector_left_heel",
                "/forceset/online_grf_detector_left_toe",
            ],
            online_grf_detector_force_sides={
                "online_grf_detector_left_heel": "left",
                "online_grf_detector_left_toe": "left",
            },
        )
        runner._cfg = SimpleNamespace()
        runner._online_event_detector = event_detector
        runner._last_step_info = {}

        with mock.patch(
            "simulation_runner.read_online_grf",
            side_effect=[primary_grf, detector_grf],
        ):
            returned_grf, events = runner._sample_online_grf(object(), 1.25)

        self.assertIs(returned_grf, primary_grf)
        self.assertEqual(events, [])
        event_detector.update.assert_called_once()
        event_forces = event_detector.update.call_args.args[1]
        self.assertEqual(event_forces["left"], 32.0)
        self.assertEqual(event_forces["right"], 310.0)
        self.assertIn("online_grf_detector", runner._last_step_info)

    def test_env_gait_load_and_contact_ignore_detector_aggregate(self) -> None:
        env = object.__new__(CMCLikeProsthesisTrajectoryEnv)
        env.ctx = SimpleNamespace(
            online_grf_force_paths=["/forceset/primary_left"],
            online_grf_force_sides={"primary_left": "left"},
        )
        env.t = 4.0
        env._body_weight_n = 100.0
        env._online_events = []
        env._online_gait_sides = {
            "left": {
                "last_heel_strike_time": None,
                "last_toe_off_time": None,
                "cycle_duration_s": None,
            },
            "right": {
                "last_heel_strike_time": None,
                "last_toe_off_time": None,
                "cycle_duration_s": None,
            },
        }
        env._online_grf = {
            "left": _side_payload(120.0, 0.002),
            "right": _side_payload(310.0, 0.003),
        }
        env._online_grf_detector = {
            "left": _side_payload(999.0, 0.0),
            "right": _side_payload(0.0, 0.0),
            "sensors": {
                "left_heel": {"normal_load_n": 7.0},
                "left_toe": {"normal_load_n": 11.0},
            },
        }

        gait = env._online_gait_info()

        self.assertEqual(gait["sides"]["left"]["normal_force_n"], 120.0)
        self.assertEqual(gait["sides"]["left"]["normal_force_bw"], 1.2)
        self.assertTrue(gait["sides"]["left"]["in_contact"])
        self.assertEqual(gait["sides"]["right"]["normal_force_n"], 310.0)
        self.assertEqual(gait["sides"]["right"]["normal_force_bw"], 3.1)
        self.assertTrue(gait["sides"]["right"]["in_contact"])

        env._online_grf_detector["left"] = _side_payload(0.0, 0.005)
        env._online_grf_detector["right"] = _side_payload(888.0, 0.005)
        changed_detector_gait = env._online_gait_info()
        for side in ("left", "right"):
            for field in ("normal_force_n", "normal_force_bw", "in_contact"):
                self.assertEqual(
                    changed_detector_gait["sides"][side][field],
                    gait["sides"][side][field],
                )
        self.assertEqual(
            env._online_load_contact_observation_features(
                changed_detector_gait
            ),
            env._online_load_contact_observation_features(gait),
        )

    def test_detector_is_never_a_fallback_for_missing_primary_load(self) -> None:
        env = object.__new__(CMCLikeProsthesisTrajectoryEnv)
        env.ctx = SimpleNamespace(
            online_grf_force_paths=["/forceset/primary_left"]
        )
        env._online_grf = {}
        env._online_grf_detector = {
            "left": _side_payload(999.0, 0.005),
            "right": _side_payload(888.0, 0.005),
        }

        self.assertEqual(env._physical_online_grf_sides(), {})
        with self.assertRaisesRegex(
            RuntimeError,
            "Primary online-GRF sample is required",
        ):
            env._physical_online_grf_sides(required=True)

    def test_required_primary_rejects_empty_and_nonfinite_aggregates(self) -> None:
        env = object.__new__(CMCLikeProsthesisTrajectoryEnv)
        env.ctx = SimpleNamespace(
            online_grf_force_paths=["/forceset/primary_left"],
            online_grf_force_sides={"primary_left": "left"},
        )
        env._online_grf_detector = {
            "left": _side_payload(999.0, 0.005),
        }

        for malformed in (
            {},
            {"normal_force": 10.0, "in_contact": True},
            _side_payload(float("nan"), 0.002),
            {**_side_payload(10.0, 0.002), "in_contact": 1},
            {**_side_payload(10.0, 0.002), "force": {"bad": "shape"}},
            {**_side_payload(10.0, 0.002), "normal_force": True},
        ):
            env._online_grf = {"left": malformed}
            with self.subTest(malformed=malformed):
                with self.assertRaisesRegex(
                    RuntimeError,
                    "missing or malformed",
                ):
                    env._physical_online_grf_sides(required=True)

    def test_left_only_primary_is_complete_for_left_hybrid_contract(self) -> None:
        env = object.__new__(CMCLikeProsthesisTrajectoryEnv)
        env.ctx = SimpleNamespace(
            online_grf_force_paths=["/forceset/primary_left"],
            online_grf_force_sides={"primary_left": "left"},
        )
        env._online_grf = {"left": _side_payload(120.0, 0.002)}
        env._online_grf_detector = {
            "right": _side_payload(999.0, 0.005),
        }

        self.assertIs(
            env._physical_online_grf_sides(required=True),
            env._online_grf,
        )

    def _hybrid_runner(
        self,
        *,
        left_force_n: float = 100.0,
        right_force_n: float = 100.0,
        left_penetration_m: float = 0.002,
        right_penetration_m: float = 0.002,
    ) -> tuple[SimulationRunner, dict]:
        runner = object.__new__(SimulationRunner)
        runner._ctx = SimpleNamespace(
            grf_mode="online_sensor",
            online_grf_applied_sides=["left"],
            online_grf_force_paths=["/forceset/primary_left"],
            online_grf_force_sides={"primary_left": "left"},
            online_grf_detector_force_paths=[],
            model=SimpleNamespace(getTotalMass=lambda _state: 10.0),
        )
        runner._cfg = SimpleNamespace(
            online_grf_max_force_bw=5.0,
            online_grf_max_penetration_m=0.015,
        )
        runner._online_event_detector = None
        runner._last_step_info = {}
        grf = {
            "sides": {
                "left": _side_payload(left_force_n, left_penetration_m),
                "right": _side_payload(right_force_n, right_penetration_m),
            },
            "spheres": {},
        }
        return runner, grf

    def test_hybrid_applied_side_keeps_online_force_safety_gate(self) -> None:
        runner, grf = self._hybrid_runner(left_force_n=1000.0)
        with (
            mock.patch.object(
                simulation_runner,
                "read_online_grf",
                return_value=grf,
            ),
            self.assertRaisesRegex(FloatingPointError, "exceeds 5 BW"),
        ):
            runner._sample_online_grf(object(), 1.0)

    def test_hybrid_safety_ignores_nonapplied_online_sensor_side(self) -> None:
        runner, grf = self._hybrid_runner(
            left_force_n=100.0,
            right_force_n=1000.0,
            right_penetration_m=0.100,
        )
        with mock.patch.object(
            simulation_runner,
            "read_online_grf",
            return_value=grf,
        ):
            sampled, events = runner._sample_online_grf(object(), 1.0)
        self.assertIs(sampled, grf)
        self.assertEqual(events, [])

    def test_hybrid_applied_side_keeps_penetration_safety_gate(self) -> None:
        runner, grf = self._hybrid_runner(left_penetration_m=0.016)
        with (
            mock.patch.object(
                simulation_runner,
                "read_online_grf",
                return_value=grf,
            ),
            self.assertRaisesRegex(
                FloatingPointError,
                "penetration exceeds 0.015 m",
            ),
        ):
            runner._sample_online_grf(object(), 1.0)

    def test_fsm_uses_primary_load_and_high_rate_detector_batch(self) -> None:
        class FakeStateValues:
            @staticmethod
            def get(index: int) -> float:
                return {10: 0.15, 11: -0.05}[index]

        phase_fsm = mock.Mock()
        phase_fsm.update_policy_step.return_value = {
            "accepted_transitions_this_step": [],
        }

        env = object.__new__(CMCLikeProsthesisTrajectoryEnv)
        env.env_cfg = SimpleNamespace(phase_fsm_input_mode="shadow")
        env.ctx = SimpleNamespace(
            model=SimpleNamespace(
                getStateVariableValues=lambda _state: FakeStateValues()
            ),
            q_sv_idx={
                "pros_knee_angle": 10,
                "pros_ankle_angle": 11,
            },
        )
        env.runner = SimpleNamespace(state=object())
        env.t = 2.5
        env._body_weight_n = 100.0
        env._phase_fsm = phase_fsm
        env._phase_fsm_payload = {}
        env._phase_sensor_previous_time_s = 2.49
        env._phase_sensor_samples = [
            {
                "time_s": 2.49 + (index + 1) * 0.001,
                "left_heel_normal_n": 7.0,
                "left_toe_normal_n": 11.0,
            }
            for index in range(10)
        ]
        env._legacy_online_events = []
        env._online_events = []
        env._online_gait_sides = {
            "left": {
                "last_heel_strike_time": None,
                "last_toe_off_time": None,
                "cycle_duration_s": None,
            },
            "right": {
                "last_heel_strike_time": None,
                "last_toe_off_time": None,
                "cycle_duration_s": None,
            },
        }
        env._online_grf = {
            "left": _side_payload(80.0, 0.002),
            "right": _side_payload(40.0, 0.001),
        }
        env._online_grf_detector = {
            "left": _side_payload(900.0, 0.0),
            "right": _side_payload(0.0, 0.0),
            "sensors": {
                "left_heel": {"normal_load_n": 7.0},
                "left_toe": {"normal_load_n": 11.0},
            },
        }

        env._update_phase_fsm()

        phase_fsm.update.assert_not_called()
        phase_fsm.update_policy_step.assert_called_once()
        call = phase_fsm.update_policy_step.call_args.kwargs
        self.assertEqual(call["normal_force_bw"], 0.8)
        self.assertTrue(call["in_contact"])
        self.assertEqual(call["previous_time_s"], 2.49)
        self.assertEqual(call["sensor_samples"], env._phase_sensor_samples)
        self.assertEqual(env._phase_sensor_previous_time_s, 2.5)

    def test_failed_segment_does_not_mask_original_failure_with_missing_primary(
        self,
    ) -> None:
        class FakeStateValues:
            @staticmethod
            def get(index: int) -> float:
                return {10: 0.15, 11: -0.05}[index]

        phase_fsm = mock.Mock()
        phase_fsm.update.return_value = {
            "accepted_transitions_this_step": [],
        }
        env = object.__new__(CMCLikeProsthesisTrajectoryEnv)
        env.env_cfg = SimpleNamespace(phase_fsm_input_mode="legacy_events")
        env.ctx = SimpleNamespace(
            online_grf_force_paths=["/forceset/primary_left"],
            model=SimpleNamespace(
                getStateVariableValues=lambda _state: FakeStateValues()
            ),
            q_sv_idx={
                "pros_knee_angle": 10,
                "pros_ankle_angle": 11,
            },
        )
        env.runner = SimpleNamespace(state=object())
        env.t = 2.5
        env._body_weight_n = 100.0
        env._phase_fsm = phase_fsm
        env._phase_fsm_payload = {}
        env._legacy_online_events = []
        env._online_events = []
        env._online_grf = {}
        env._online_grf_detector = {}
        env._online_gait_sides = {
            side: {
                "last_heel_strike_time": None,
                "last_toe_off_time": None,
                "cycle_duration_s": None,
            }
            for side in ("left", "right")
        }

        env._update_phase_fsm(require_primary_sample=False)
        phase_fsm.update.assert_called_once()
        with self.assertRaisesRegex(
            RuntimeError,
            "Primary online-GRF sample is required",
        ):
            env._update_phase_fsm(require_primary_sample=True)

    def test_detector_profile_has_distinct_reasonable_left_sensors(self) -> None:
        profile = load_online_grf_profile(DETECTOR_PROFILE)
        by_name = {sphere.name: sphere for sphere in profile.spheres}
        self.assertIn("left_heel", by_name)
        self.assertIn("left_toe", by_name)
        heel = by_name["left_heel"]
        toe = by_name["left_toe"]
        self.assertEqual(heel.side, "left")
        self.assertEqual(toe.side, "left")
        self.assertEqual(heel.frame, toe.frame)
        separation = float(
            np.linalg.norm(np.asarray(heel.location) - np.asarray(toe.location))
        )
        self.assertGreater(separation, heel.radius + toe.radius)

    def test_left_only_profile_is_opt_in_for_detector_loading(self) -> None:
        payload = json.loads(DETECTOR_PROFILE.read_text(encoding="utf-8"))
        payload["spheres"] = [
            sphere for sphere in payload["spheres"] if sphere["side"] == "left"
        ]

        with self.assertRaisesRegex(ValueError, "missing required sphere side.*right"):
            online_grf.OnlineGRFProfile.from_mapping(payload)

        detector = online_grf.OnlineGRFProfile.from_mapping(
            payload,
            required_sides=("left",),
        )
        self.assertEqual(len(detector.spheres), 2)
        self.assertEqual({sphere.side for sphere in detector.spheres}, {"left"})

    def test_model_loader_relaxes_primary_contract_only_for_hybrid_sides(
        self,
    ) -> None:
        self.assertEqual(
            _primary_profile_required_sides("online_sensor", {"left"}),
            ("left",),
        )
        self.assertEqual(
            _primary_profile_required_sides("online_sensor", {"right"}),
            ("right",),
        )
        self.assertEqual(
            _primary_profile_required_sides(
                "online_sensor",
                {"right", "left"},
            ),
            ("left", "right"),
        )
        self.assertEqual(
            _primary_profile_required_sides("online_sensor", set()),
            ("left", "right"),
        )
        self.assertEqual(
            _primary_profile_required_sides("online", set()),
            ("left", "right"),
        )

    def test_hybrid_requires_prescribed_support_on_complementary_side(
        self,
    ) -> None:
        _validate_hybrid_prescribed_support({"left"}, {"right"})
        _validate_hybrid_prescribed_support({"right"}, {"left"})
        _validate_hybrid_prescribed_support({"left", "right"}, set())
        with self.assertRaisesRegex(ValueError, "missing prescribed.*right"):
            _validate_hybrid_prescribed_support({"left"}, set())

    def test_model_loader_relaxes_left_requirement_for_detector(self) -> None:
        tree = ast.parse(
            (REPO_ROOT / "model_loader.py").read_text(encoding="utf-8")
        )
        matching_calls: list[ast.Call] = []
        for node in ast.walk(tree):
            if not isinstance(node, ast.Call):
                continue
            if not isinstance(node.func, ast.Name):
                continue
            if node.func.id != "load_online_grf_profile":
                continue
            keywords = {item.arg: item.value for item in node.keywords if item.arg}
            required = keywords.get("required_sides")
            if required is not None:
                matching_calls.append(node)

        detector_calls = [
            call
            for call in matching_calls
            if isinstance(call.args[0], ast.Name)
            and call.args[0].id == "online_grf_detector_profile_file"
        ]
        self.assertEqual(len(detector_calls), 1)
        required = next(
            item.value
            for item in detector_calls[0].keywords
            if item.arg == "required_sides"
        )
        self.assertIsInstance(required, ast.Tuple)
        self.assertEqual(
            [item.value for item in required.elts if isinstance(item, ast.Constant)],
            ["left"],
        )

    def test_every_detector_component_sets_applies_force_false(self) -> None:
        profile = load_online_grf_profile(DETECTOR_PROFILE)

        class FakeSocket:
            def connect(self, _frame) -> None:
                return None

        class FakeForce:
            def setName(self, name: str) -> None:
                self.name = name

            def updSocket(self, _name: str) -> FakeSocket:
                return FakeSocket()

        class FakeOpenSimObjectType:
            @staticmethod
            def newInstanceOfType(_name: str) -> FakeForce:
                return FakeForce()

        class FakeForceType:
            @staticmethod
            def safeDownCast(component):
                return component

        class FakeModel:
            def getComponent(self, _path: str) -> object:
                return object()

        for sphere in profile.spheres:
            property_values: dict[str, object] = {}

            def capture_property(_component, name: str, value) -> None:
                property_values[name] = value

            with (
                mock.patch.object(
                    online_grf.opensim,
                    "OpenSimObject",
                    FakeOpenSimObjectType,
                ),
                mock.patch.object(online_grf.opensim, "Force", FakeForceType),
                mock.patch.object(online_grf, "_set_property", capture_property),
            ):
                online_grf._new_online_grf_force(
                    FakeModel(),
                    sphere,
                    profile,
                    False,
                    name_prefix="online_grf_detector_",
                )

            self.assertIn("appliesForce", property_values)
            self.assertIs(property_values["appliesForce"], False)

    def test_model_loader_instantiates_detector_profile_sensor_only(self) -> None:
        """Guard the production call, not just the profile JSON contents."""
        tree = ast.parse(
            (REPO_ROOT / "model_loader.py").read_text(encoding="utf-8")
        )
        detector_calls: list[ast.Call] = []
        for node in ast.walk(tree):
            if not isinstance(node, ast.Call):
                continue
            if not isinstance(node.func, ast.Name):
                continue
            if node.func.id != "add_online_grf_forces":
                continue
            keywords = {item.arg: item.value for item in node.keywords if item.arg}
            prefix = keywords.get("name_prefix")
            if (
                isinstance(prefix, ast.Constant)
                and prefix.value == "online_grf_detector_"
            ):
                detector_calls.append(node)

        self.assertEqual(len(detector_calls), 1)
        keywords = {
            item.arg: item.value
            for item in detector_calls[0].keywords
            if item.arg
        }
        applies_force = keywords.get("applies_force")
        self.assertIsInstance(applies_force, ast.Constant)
        self.assertIs(applies_force.value, False)

    def test_two_sensor_env_profile_contract_fails_before_first_segment(self) -> None:
        env = object.__new__(CMCLikeProsthesisTrajectoryEnv)
        env.env_cfg = SimpleNamespace(phase_fsm_input_mode="two_sensor")
        env.ctx = SimpleNamespace(
            online_grf_detector_force_paths=[
                "/forceset/online_grf_detector_left_heel",
                "/forceset/online_grf_detector_left_toe",
            ],
            online_grf_detector_force_sides={
                "online_grf_detector_left_heel": "left",
                "online_grf_detector_left_toe": "left",
            },
        )
        env._validate_phase_sensor_setup()

        env.ctx.online_grf_detector_force_paths.pop()
        with self.assertRaisesRegex(ValueError, "exactly two"):
            env._validate_phase_sensor_setup()

        env.ctx.online_grf_detector_force_paths.extend(
            [
                "/forceset/online_grf_detector_left_toe",
                "/forceset/online_grf_detector_right_heel",
                "/forceset/online_grf_detector_right_toe",
            ]
        )
        with self.assertRaisesRegex(ValueError, "exactly two"):
            env._validate_phase_sensor_setup()


if __name__ == "__main__":
    unittest.main()
