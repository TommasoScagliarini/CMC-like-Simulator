from __future__ import annotations

import copy
import inspect
import sys
import tempfile
import unittest
from unittest import mock
from pathlib import Path
from types import SimpleNamespace


REPO_ROOT = Path(__file__).resolve().parents[1]
TRAJECTORY_ROOT = REPO_ROOT / "Trajectory Generator"
for import_root in (REPO_ROOT, TRAJECTORY_ROOT):
    if str(import_root) not in sys.path:
        sys.path.insert(0, str(import_root))

try:
    import model_loader as model_loader_module  # noqa: E402
    import osim_trj_cmc_like as env_module  # noqa: E402
    from binary_phase_fsm import BinaryPhaseFSM  # noqa: E402
    from osim_trj_cmc_like import (  # noqa: E402
        CMCEnvConfig,
        CMCLikeProsthesisTrajectoryEnv,
        _validate_binary_phase_sampled_disabled_configuration,
        _validate_binary_phase_shadow_configuration,
    )
    from model_loader import (  # noqa: E402
        _validate_binary_detector_coexistence,
    )
    from simulation_runner import SimulationRunner  # noqa: E402
except ModuleNotFoundError as exc:  # pragma: no cover - platform test skip.
    model_loader_module = None
    env_module = None
    BinaryPhaseFSM = None
    CMCEnvConfig = None
    CMCLikeProsthesisTrajectoryEnv = None
    _validate_binary_phase_shadow_configuration = None
    _validate_binary_phase_sampled_disabled_configuration = None
    _validate_binary_detector_coexistence = None
    SimulationRunner = None
    IMPORT_ERROR = exc
else:
    IMPORT_ERROR = None


V19_PROFILE = (
    REPO_ROOT
    / "validation"
    / "experimental_detector_profiles"
    / "two_point_binary_v19_outsole_25mm.json"
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
V19_CONTRACT = "binary_point_v19+functional_contact_fsm_v1_shadow"
V25_CONTRACT = "binary_point_v25+functional_contact_fsm_v1_shadow"


def samples(heel: bool = False, toe: bool = False) -> list[dict]:
    return [
        {
            "time_s": index / 1000,
            "left_heel_contact": heel,
            "left_toe_contact": toe,
        }
        for index in range(1, 11)
    ]


@unittest.skipIf(IMPORT_ERROR is not None, f"runtime dependency unavailable: {IMPORT_ERROR}")
class BinaryPhaseFSMEnvV20Tests(unittest.TestCase):
    @staticmethod
    def shadow_config(
        *,
        profile: Path,
        contract: str,
        analog_profile: Path | None = None,
    ) -> CMCEnvConfig:
        return CMCEnvConfig(
            phase_fsm_input_mode="legacy_events",
            event_contract_id="legacy_events_v1",
            online_grf_detector_profile_file=(
                None if analog_profile is None else str(analog_profile)
            ),
            binary_phase_detector_profile_file=str(profile),
            binary_phase_fsm_mode="binary_shadow",
            binary_phase_event_contract_id=contract,
            binary_phase_debounce_s=0.005,
            detector_sample_dt_s=0.001,
            segment_duration=0.01,
        )

    def shadow_env(self):
        env = CMCLikeProsthesisTrajectoryEnv.__new__(
            CMCLikeProsthesisTrajectoryEnv
        )
        env.env_cfg = SimpleNamespace(
            binary_phase_fsm_mode="binary_shadow",
            binary_phase_event_contract_id=(
                "binary_point_v19+functional_contact_fsm_v1_shadow"
            ),
        )
        env._binary_phase_fsm = BinaryPhaseFSM()
        env._binary_phase_fsm.reset(
            time_s=0.0,
            heel_contact=False,
            toe_contact=False,
        )
        env._binary_phase_fsm_payload = env._binary_phase_fsm.payload()
        env._binary_phase_sensor_previous_time_s = 0.0
        env._binary_phase_sensor_samples = samples(heel=True)
        env.t = 0.010
        env._online_events = [
            {"side": "left", "event": "heel_strike", "source": "legacy"}
        ]
        env._legacy_online_events = copy.deepcopy(env._online_events)
        env._phase_fsm_payload = {"authoritative": "legacy"}
        return env

    @staticmethod
    def analog_routes() -> tuple[tuple[str, ...], dict[str, str]]:
        names = (
            "online_grf_detector_left_heel",
            "online_grf_detector_left_toe",
            "online_grf_detector_right_heel",
            "online_grf_detector_right_toe",
        )
        paths = tuple(f"/forceset/{name}" for name in names)
        sides = {
            name: ("left" if "_left_" in name else "right")
            for name in names
        }
        return paths, sides

    def v25_setup_env(
        self,
        *,
        paths: tuple[str, ...] | None = None,
        sides: dict[str, str] | None = None,
    ):
        canonical_paths, canonical_sides = self.analog_routes()
        env = CMCLikeProsthesisTrajectoryEnv.__new__(
            CMCLikeProsthesisTrajectoryEnv
        )
        env.env_cfg = self.shadow_config(
            profile=V25_PROFILE,
            contract=V25_CONTRACT,
            analog_profile=ANALOG_PROFILE,
        )
        env.ctx = SimpleNamespace(
            binary_phase_detector_profile=SimpleNamespace(
                points=(
                    SimpleNamespace(name="left_heel"),
                    SimpleNamespace(name="left_toe"),
                )
            ),
            online_grf_detector_force_paths=(
                canonical_paths if paths is None else paths
            ),
            online_grf_detector_force_sides=(
                canonical_sides if sides is None else sides
            ),
        )
        return env

    def test_defaults_leave_historical_fsm_authoritative(self) -> None:
        cfg = CMCEnvConfig()
        self.assertEqual(cfg.phase_fsm_input_mode, "legacy_events")
        self.assertEqual(cfg.event_contract_id, "legacy_events_v1")
        self.assertEqual(cfg.binary_phase_fsm_mode, "disabled")
        self.assertIsNone(cfg.binary_phase_detector_profile_file)

    def test_v25_shadow_allows_frozen_legacy_analog_detector(self) -> None:
        cfg = self.shadow_config(
            profile=V25_PROFILE,
            contract=V25_CONTRACT,
            analog_profile=ANALOG_PROFILE,
        )
        label, sha256, requires_analog = (
            _validate_binary_phase_shadow_configuration(
                cfg,
                detector_dt=cfg.detector_sample_dt_s,
            )
        )
        self.assertEqual(label, "V25")
        self.assertEqual(
            sha256,
            "db704e502b99e49bea6d89493812bafdac748f8ce8d3ce28214ff624078539a2",
        )
        self.assertTrue(requires_analog)
        self.v25_setup_env()._validate_binary_phase_sensor_setup()

    def test_v25_missing_analog_profile_fails_closed_in_env_and_loader(self) -> None:
        cfg = self.shadow_config(
            profile=V25_PROFILE,
            contract=V25_CONTRACT,
        )
        with self.assertRaisesRegex(ValueError, "requires the frozen analog"):
            _validate_binary_phase_shadow_configuration(
                cfg,
                detector_dt=cfg.detector_sample_dt_s,
            )
        with self.assertRaisesRegex(ValueError, "requires the analog legacy"):
            _validate_binary_detector_coexistence(V25_PROFILE, None)

    def test_v25_wrong_analog_path_fails_closed_in_env_and_loader(self) -> None:
        cfg = self.shadow_config(
            profile=V25_PROFILE,
            contract=V25_CONTRACT,
            analog_profile=V19_PROFILE,
        )
        with self.assertRaisesRegex(ValueError, "analog legacy profile path mismatch"):
            _validate_binary_phase_shadow_configuration(
                cfg,
                detector_dt=cfg.detector_sample_dt_s,
            )
        with self.assertRaisesRegex(ValueError, "analog legacy profile path mismatch"):
            _validate_binary_detector_coexistence(V25_PROFILE, V19_PROFILE)

    def test_v25_analog_hash_drift_fails_closed_in_env_and_loader(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            drifted_path = Path(temp_dir) / ANALOG_PROFILE.name
            drifted_path.write_bytes(b"{}\n")
            cfg = self.shadow_config(
                profile=V25_PROFILE,
                contract=V25_CONTRACT,
                analog_profile=drifted_path,
            )
            with mock.patch.object(
                env_module,
                "_V25_LEGACY_ANALOG_DETECTOR_PROFILE_PATH",
                drifted_path,
            ):
                with self.assertRaisesRegex(
                    ValueError,
                    "analog legacy profile hash mismatch",
                ):
                    _validate_binary_phase_shadow_configuration(
                        cfg,
                        detector_dt=cfg.detector_sample_dt_s,
                    )
            with mock.patch.object(
                model_loader_module,
                "_V25_LEGACY_ANALOG_DETECTOR_PROFILE_PATH",
                drifted_path,
            ):
                with self.assertRaisesRegex(
                    ValueError,
                    "analog legacy profile hash mismatch",
                ):
                    _validate_binary_detector_coexistence(
                        V25_PROFILE,
                        drifted_path,
                    )

    def test_v19_keeps_historical_analog_coexistence_guard(self) -> None:
        cfg = self.shadow_config(
            profile=V19_PROFILE,
            contract=V19_CONTRACT,
            analog_profile=ANALOG_PROFILE,
        )
        with self.assertRaisesRegex(ValueError, "V19 forbids"):
            _validate_binary_phase_shadow_configuration(
                cfg,
                detector_dt=cfg.detector_sample_dt_s,
            )

        env = CMCLikeProsthesisTrajectoryEnv.__new__(
            CMCLikeProsthesisTrajectoryEnv
        )
        env.env_cfg = self.shadow_config(
            profile=V19_PROFILE,
            contract=V19_CONTRACT,
        )
        env.ctx = SimpleNamespace(
            binary_phase_detector_profile=SimpleNamespace(
                points=(
                    SimpleNamespace(name="left_heel"),
                    SimpleNamespace(name="left_toe"),
                )
            ),
            online_grf_detector_force_paths=("legacy_heel", "legacy_toe"),
        )
        with self.assertRaisesRegex(ValueError, "V19 cannot coexist"):
            env._validate_binary_phase_sensor_setup()

    def test_v25_shadow_contract_and_profile_hash_fail_closed(self) -> None:
        unknown = self.shadow_config(
            profile=V25_PROFILE,
            contract="binary_point_v26+functional_contact_fsm_v1_shadow",
        )
        with self.assertRaisesRegex(ValueError, "allowlisted"):
            _validate_binary_phase_shadow_configuration(
                unknown,
                detector_dt=unknown.detector_sample_dt_s,
            )

        with tempfile.NamedTemporaryFile(suffix=".json") as altered:
            altered.write(b"{}\n")
            altered.flush()
            drifted = self.shadow_config(
                profile=Path(altered.name),
                contract=V25_CONTRACT,
                analog_profile=ANALOG_PROFILE,
            )
            with self.assertRaisesRegex(ValueError, "V25 profile hash mismatch"):
                _validate_binary_phase_shadow_configuration(
                    drifted,
                    detector_dt=drifted.detector_sample_dt_s,
                )

    def test_v25_shadow_cannot_replace_legacy_authority(self) -> None:
        base = self.shadow_config(
            profile=V25_PROFILE,
            contract=V25_CONTRACT,
            analog_profile=ANALOG_PROFILE,
        )
        for cfg, message in (
            (
                CMCEnvConfig(
                    **{
                        **base.__dict__,
                        "phase_fsm_input_mode": "two_sensor",
                    }
                ),
                "phase_fsm_input_mode='legacy_events'",
            ),
            (
                CMCEnvConfig(
                    **{
                        **base.__dict__,
                        "event_contract_id": "primary_grf_split_v1",
                    }
                ),
                "event_contract_id='legacy_events_v1'",
            ),
        ):
            with self.subTest(message=message):
                with self.assertRaisesRegex(ValueError, message):
                    _validate_binary_phase_shadow_configuration(
                        cfg,
                        detector_dt=cfg.detector_sample_dt_s,
                    )

    def test_shadow_is_absent_from_observation_builder(self) -> None:
        observation_source = inspect.getsource(
            CMCLikeProsthesisTrajectoryEnv._get_observation
        )
        self.assertNotIn("_binary_phase", observation_source)

    def test_disabled_mode_adds_no_binary_info_keys(self) -> None:
        env = CMCLikeProsthesisTrajectoryEnv.__new__(
            CMCLikeProsthesisTrajectoryEnv
        )
        env.env_cfg = SimpleNamespace(
            binary_phase_fsm_mode="disabled",
            binary_phase_event_contract_id="binary_events_disabled_v1",
        )
        self.assertEqual(env._binary_phase_info_payload(), {})

        runner = SimulationRunner.__new__(SimulationRunner)
        runner._runtime_state = object()
        runner._runtime_controls = object()
        runner._runtime_time = 0.0
        runner._cfg = SimpleNamespace(
            dt=0.001,
            integration_dt=0.001,
            use_control_window=False,
        )
        runner._last_step_info = {}
        runner._phase_sensor_sampling_enabled = False
        runner._binary_phase_sensor_sampling_enabled = False
        runner._binary_phase_sensor_baseline = None
        info = runner.step_until(0.0, record=False)
        self.assertNotIn("binary_phase_sensor_samples", info)
        self.assertNotIn("binary_phase_sensor_baseline", info)

    def test_sampled_disabled_is_v25_only_and_never_executes_v20(self) -> None:
        cfg = self.shadow_config(
            profile=V25_PROFILE,
            contract=V25_CONTRACT,
            analog_profile=ANALOG_PROFILE,
        )
        cfg = SimpleNamespace(**{
            **vars(cfg),
            "binary_phase_fsm_mode": "disabled",
        })
        result = _validate_binary_phase_sampled_disabled_configuration(
            cfg,
            detector_dt=cfg.detector_sample_dt_s,
        )
        self.assertEqual(result[0], "V25")

        invalid = SimpleNamespace(**{
            **vars(cfg),
            "binary_phase_event_contract_id": V19_CONTRACT,
            "binary_phase_detector_profile_file": str(V19_PROFILE),
            "online_grf_detector_profile_file": None,
        })
        with self.assertRaisesRegex(ValueError, "case A"):
            _validate_binary_phase_sampled_disabled_configuration(
                invalid,
                detector_dt=invalid.detector_sample_dt_s,
            )

        env = CMCLikeProsthesisTrajectoryEnv.__new__(
            CMCLikeProsthesisTrajectoryEnv
        )
        env.env_cfg = cfg
        env._binary_phase_fsm = BinaryPhaseFSM()
        before = env._binary_phase_fsm.payload()
        env._binary_phase_fsm_payload = copy.deepcopy(before)
        env._binary_phase_sensor_baseline = {
            "time_s": 0.0,
            "left_heel_contact": False,
            "left_toe_contact": False,
        }
        env._binary_phase_sensor_previous_time_s = 0.0
        env._binary_phase_sensor_samples = samples(heel=True)
        env.t = 0.010
        env._update_binary_phase_fsm(require_batch=True)
        self.assertEqual(env._binary_phase_fsm.payload(), before)
        self.assertEqual(env._binary_phase_sensor_previous_time_s, 0.010)
        payload = env._binary_phase_info_payload()
        self.assertFalse(payload["binary_phase_fsm_executed"])
        self.assertNotIn("binary_phase_fsm", payload)
        self.assertEqual(len(payload["binary_phase_sensor_samples"]), 10)
        self.assertEqual(
            set(payload["binary_phase_sensor_baseline"]),
            {"time_s", "left_heel_contact", "left_toe_contact"},
        )

    def test_loader_allows_only_v25_to_coexist_with_legacy_detector(self) -> None:
        _validate_binary_detector_coexistence(V25_PROFILE, ANALOG_PROFILE)
        with self.assertRaisesRegex(ValueError, "only with the frozen V25"):
            _validate_binary_detector_coexistence(V19_PROFILE, ANALOG_PROFILE)
        _validate_binary_detector_coexistence(V19_PROFILE, None)

    def _assert_v25_routing_rejected(
        self,
        paths: tuple[str, ...],
        sides: dict[str, str],
    ) -> None:
        with self.assertRaisesRegex(ValueError, "exactly four analog legacy roles"):
            self.v25_setup_env(paths=paths, sides=sides)._validate_binary_phase_sensor_setup()

    def test_v25_incomplete_analog_routing_fails_closed(self) -> None:
        paths, sides = self.analog_routes()
        self._assert_v25_routing_rejected(paths[:-1], sides)

    def test_v25_duplicate_analog_routing_fails_closed(self) -> None:
        paths, sides = self.analog_routes()
        duplicated = (*paths[:-1], paths[0])
        self._assert_v25_routing_rejected(duplicated, sides)

    def test_v25_unknown_analog_routing_fails_closed(self) -> None:
        paths, sides = self.analog_routes()
        unknown_name = "online_grf_detector_right_midfoot"
        unknown_paths = (*paths[:-1], f"/forceset/{unknown_name}")
        unknown_sides = {**sides, unknown_name: "right"}
        self._assert_v25_routing_rejected(unknown_paths, unknown_sides)

    def test_shadow_update_cannot_mutate_active_stream_or_legacy_payload(self) -> None:
        env = self.shadow_env()
        active_before = copy.deepcopy(env._online_events)
        legacy_before = copy.deepcopy(env._legacy_online_events)
        historical_fsm_before = copy.deepcopy(env._phase_fsm_payload)
        env._update_binary_phase_fsm(require_batch=True)

        self.assertEqual(env._online_events, active_before)
        self.assertEqual(env._legacy_online_events, legacy_before)
        self.assertEqual(env._phase_fsm_payload, historical_fsm_before)
        self.assertEqual(env._binary_phase_sensor_previous_time_s, 0.010)
        self.assertEqual(
            env._binary_phase_fsm_payload["events_this_step"][0]["event"],
            "heel_strike",
        )

    def test_malformed_shadow_batch_is_atomic_and_does_not_advance_boundary(self) -> None:
        env = self.shadow_env()
        env._binary_phase_sensor_samples[4]["left_toe_contact"] = 0
        fsm_before = env._binary_phase_fsm.payload()
        active_before = copy.deepcopy(env._online_events)
        with self.assertRaises(TypeError):
            env._update_binary_phase_fsm(require_batch=True)
        self.assertEqual(env._binary_phase_fsm.payload(), fsm_before)
        self.assertEqual(env._binary_phase_sensor_previous_time_s, 0.0)
        self.assertEqual(env._online_events, active_before)

    def test_numerical_failure_does_not_consume_shadow_batch(self) -> None:
        env = self.shadow_env()
        fsm_before = env._binary_phase_fsm.payload()
        env._update_binary_phase_fsm(require_batch=False)
        self.assertEqual(env._binary_phase_fsm.payload(), fsm_before)
        self.assertEqual(env._binary_phase_sensor_previous_time_s, 0.0)

    def test_t0_baseline_is_required_and_emits_no_event(self) -> None:
        env = self.shadow_env()
        env.t = 4.125
        env.runner = SimpleNamespace(
            last_step_info={
                "binary_phase_sensor_baseline": {
                    "time_s": 4.125,
                    "contacts": {
                        "left_heel": False,
                        "left_toe": True,
                    },
                }
            }
        )
        env._prime_binary_phase_fsm_from_runner()
        payload = env._binary_phase_fsm_payload
        self.assertEqual(payload["raw_contact_state"], "TOE")
        self.assertTrue(payload["in_contact"])
        self.assertEqual(payload["events_this_step"], [])
        self.assertEqual(env._binary_phase_sensor_previous_time_s, 4.125)

        env.runner = SimpleNamespace(last_step_info={})
        with self.assertRaises(RuntimeError):
            env._prime_binary_phase_fsm_from_runner()

    def test_diagnostic_info_is_deep_copied(self) -> None:
        env = self.shadow_env()
        info = env._binary_phase_info_payload()
        info["binary_phase_sensor_samples"][0]["left_heel_contact"] = False
        info["binary_phase_fsm"]["event_count"]["heel_strike"] = 999
        self.assertTrue(
            env._binary_phase_sensor_samples[0]["left_heel_contact"]
        )
        self.assertNotEqual(
            env._binary_phase_fsm_payload["event_count"]["heel_strike"], 999
        )


if __name__ == "__main__":
    unittest.main()
