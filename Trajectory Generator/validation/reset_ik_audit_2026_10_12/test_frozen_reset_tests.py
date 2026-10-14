"""Offline checks of the audit boundary; no OpenSim, Torch or Ray imports."""
from __future__ import annotations

import importlib.util
import tempfile
import unittest
from dataclasses import dataclass, FrozenInstanceError
from pathlib import Path
from types import SimpleNamespace

SPEC = importlib.util.spec_from_file_location("frozen_reset_harness", Path(__file__).with_name("frozen_reset_tests.py"))
harness = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(harness)


class Vector:
    def __init__(self, values):
        self.values = values

    def get(self, index):
        return self.values[index]

    def getSize(self):
        return len(self.values)


def fake_environment(fail=False):
    coordinates = ("pelvis_tx", *harness.JOINTS)
    q = {"pelvis_tx": 0.6, "pros_knee_angle": -0.4, "pros_ankle_angle": 0.1}
    qdot = {"pelvis_tx": 1.2, "pros_knee_angle": -0.8, "pros_ankle_angle": 0.3}
    values = Vector([0.0] * 8)
    names = Vector(["q/" + key for key in coordinates] + ["v/" + key for key in coordinates] + ["SEA_Knee/motor_speed", "SEA_Ankle/motor_speed"])
    model = SimpleNamespace(getStateVariableValues=lambda state: values, getStateVariableNames=lambda: names)
    kin = SimpleNamespace(get=lambda t: (q, qdot, {key: 0.0 for key in q}))
    runner = SimpleNamespace(_kin=kin, state=object())

    def reset(t):
        q0, qd0, _ = runner._kin.get(t)
        if fail:
            raise RuntimeError("fake reset failed")
        values.values[:] = [q0[key] for key in coordinates] + [qd0[key] for key in coordinates] + [qd0[key] for key in harness.JOINTS]
        return runner.state

    runner.reset_to_time = reset
    ctx = SimpleNamespace(model=model, q_sv_idx=dict(zip(coordinates, range(3))),
                          qdot_sv_idx=dict(zip(coordinates, range(3, 6))),
                          sea_motor_speed_sv_idx={"SEA_Knee": 6, "SEA_Ankle": 7})
    return SimpleNamespace(runner=runner, ctx=ctx, cfg=SimpleNamespace(sea_knee_name="SEA_Knee", sea_ankle_name="SEA_Ankle")), qdot


class AuditBoundaryTests(unittest.TestCase):
    def test_worker_accepts_only_watchdog_precreated_directory(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "case"
            path.mkdir()
            (path / "watchdog_state.json").write_text('{"phase":"startup"}', encoding="utf-8")
            harness.prepare_worker_output(path)
            self.assertEqual(list(path.iterdir()), [path / "watchdog_state.json"])
            for name in ("receipt.json", "rollout_summary.json", "sim_outputs", "unrecognized.txt"):
                artifact = path / name
                if name == "sim_outputs":
                    artifact.mkdir()
                else:
                    artifact.write_text("preserve me", encoding="utf-8")
                with self.assertRaises(FileExistsError):
                    harness.prepare_worker_output(path)
                self.assertTrue(artifact.exists())
                artifact.rmdir() if artifact.is_dir() else artifact.unlink()

    def test_passive_offset_replaces_frozen_config_without_mutating_source(self):
        @dataclass(frozen=True)
        class Config:
            episode_start_offset_s: float = 1.0
            grf_mode: str = "online_sensor"
        source = Config()
        base = SimpleNamespace(env_cfg=source)
        with self.assertRaises(FrozenInstanceError):
            source.episode_start_offset_s = 2.0
        harness.set_probe_start_offset(base, 2.0)
        self.assertEqual(source.episode_start_offset_s, 1.0)
        self.assertEqual(base.env_cfg.episode_start_offset_s, 2.0)
        self.assertEqual(base.env_cfg.grf_mode, source.grf_mode)
        self.assertIsNot(base.env_cfg, source)

    def test_zero_speed_preserves_biology_and_future_reference(self):
        base, source_qdot = fake_environment()
        original_get = base.runner._kin.get
        receipt = {}
        harness.install_reset_observer(base, {"velocity": "prosthetic_zero"}, receipt)
        base.runner.reset_to_time(1.0)
        record = receipt["reset_reads"][0]
        self.assertEqual(record["actual_qdot"]["pelvis_tx"], 1.2)
        self.assertEqual(record["sea_motor_speed"], {"SEA_Knee": 0.0, "SEA_Ankle": 0.0})
        self.assertEqual(record["position_max_error_rad_or_m"], 0.0)
        self.assertEqual(source_qdot["pros_knee_angle"], -0.8)
        self.assertIs(base.runner._kin.get, original_get)
        self.assertEqual(base.runner._kin.get(1.01)[1], source_qdot)
        self.assertEqual(record["served_reference_after_reset"]["qdot"], source_qdot)

    def test_nominal_hook_preserves_unmodified_initial_values(self):
        base, source_qdot = fake_environment()
        receipt = {}
        harness.install_reset_observer(base, {"velocity": "ik"}, receipt)
        base.runner.reset_to_time(1.0)
        self.assertEqual(receipt["reset_reads"][0]["actual_qdot"], source_qdot)

    def test_delta_is_limited_to_requested_prosthetic_speed(self):
        base, _ = fake_environment()
        receipt = {}
        harness.install_reset_observer(base, {"velocity": "prosthetic_delta", "delta_rad_s": {"pros_knee_angle": 0.25}}, receipt)
        base.runner.reset_to_time(1.0)
        record = receipt["reset_reads"][0]
        self.assertAlmostEqual(record["actual_qdot"]["pros_knee_angle"], -0.55)
        self.assertEqual(record["actual_qdot"]["pros_ankle_angle"], 0.3)
        self.assertTrue(record["sea_speed_matches_joint"])

    def test_getter_restored_if_native_reset_raises(self):
        base, _ = fake_environment(fail=True)
        original_get = base.runner._kin.get
        harness.install_reset_observer(base, {"velocity": "prosthetic_zero"}, {})
        with self.assertRaisesRegex(RuntimeError, "fake reset"):
            base.runner.reset_to_time(1.0)
        self.assertIs(base.runner._kin.get, original_get)

    def test_no_synthetic_heel_strike_from_initial_contact(self):
        cycles = harness.sustained_cycles([0, 1, 2, 3, 4, 5, 6], [30, 0, 0, 30, 0, 30, 0], min_contact=0, min_cycle=0)
        self.assertEqual(len(cycles), 1)
        self.assertAlmostEqual(cycles[0][0], 2 + 2 / 3)
        self.assertAlmostEqual(cycles[0][1], 4 + 2 / 3)

    def test_rejects_duration_that_would_clamp_an_offset(self):
        case = dict(id="outside", offset_s=7.0, velocity="ik")
        with self.assertRaisesRegex(ValueError, "clamped"):
            harness.validate_cases([case], 11.99, 21, 5)

    def test_rejects_biological_velocity_intervention(self):
        case = dict(id="invalid", offset_s=1.0, velocity="prosthetic_delta", delta_rad_s={"pelvis_tx": 0.1})
        with self.assertRaisesRegex(ValueError, "prosthetic"):
            harness.validate_cases([case], 11.99, 21, 5)

    def test_missing_or_nonfinite_storage_is_not_treated_as_zero(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "bad.sto"
            path.write_text("endheader\ntime x\n0 nan\n", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "Invalid Storage"):
                harness.storage(path)


if __name__ == "__main__":
    unittest.main()
