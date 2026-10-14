"""Preparation contract checks with fake tool output; never launch OpenSim."""

from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
from xml.etree import ElementTree as ET


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from scripts import run_opensim_sea_pipeline as runner
from tools import build_abx_seasea_pipeline as builder


class PreparationContractTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory(prefix="abx_preparation_test_")
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)

    def ik_arguments(self) -> tuple[object, ...]:
        return (
            sys.executable,
            self.root / "pipeline",
            self.root / "model.osim",
            self.root / "markers.trc",
            self.root / "grf.mot",
            self.root / "support",
            self.root / "plugin.dylib",
            (1.0, 2.0),
            self.root / "ik.log",
        )

    def test_builder_executes_ik_only_and_pins_cli(self) -> None:
        generated = self.root / "pipeline" / "02_ik" / "full" / "IK_full_1p000_2p000.mot"
        generated.parent.mkdir(parents=True)
        generated.write_text("fake marker IK output\n", encoding="utf-8")
        cli = self.root / "native_opensim_cmd"
        with patch.object(builder, "run_logged", return_value=0) as run:
            result = builder.run_ik_pipeline(*self.ik_arguments(), opensim_cmd=cli)
        self.assertEqual(result, generated)
        command = run.call_args.args[0]
        self.assertIn("--ik-only", command)
        self.assertNotIn("--dry-run", command)
        self.assertEqual(command[command.index("--opensim-cmd") + 1], str(cli))

    def test_failed_ik_does_not_fall_back_to_dataset(self) -> None:
        with patch.object(builder, "run_logged", return_value=7):
            with self.assertRaisesRegex(RuntimeError, "code 7"):
                builder.run_ik_pipeline(*self.ik_arguments())

    def test_marker_metrics_follow_exact_motion_despite_stale_companion(self) -> None:
        current = "IK_full_10p360_153p415"
        for stem, rms, maximum in (
            ("IK_full_10p359_153p415", 0.9, 1.2),
            (current, 0.01, 0.04),
        ):
            (self.root / f"{stem}_ik_marker_errors.sto").write_text(
                f"endheader\ntime rms_error max_error\n10.36 {rms} {maximum}\n",
                encoding="utf-8",
            )
        self.assertEqual(runner.parse_ik_marker_error_files(self.root, current), (0.01, 0.04))
        self.assertEqual(runner.parse_ik_marker_error_files(self.root, "missing_motion"), (None, None))

    def test_missing_ik_output_is_a_failure_even_after_zero_exit(self) -> None:
        stale_wrong_range = self.root / "pipeline" / "02_ik" / "IK_full_0p000_0p500.mot"
        stale_wrong_range.parent.mkdir(parents=True)
        stale_wrong_range.write_text("old unrelated output\n", encoding="utf-8")
        with patch.object(builder, "run_logged", return_value=0):
            with self.assertRaisesRegex(FileNotFoundError, "IK_full_1p000_2p000"):
                builder.run_ik_pipeline(*self.ik_arguments())

    def test_matlab_nojvm_is_opt_in_for_conversion_and_overlay(self) -> None:
        subject_root = self.root / "session"
        id_file = subject_root / "treadmill" / "id" / "treadmill_01_01.mat"
        id_file.parent.mkdir(parents=True)
        id_file.write_bytes(b"fake file; mocked MATLAB never reads it")
        with patch.object(builder, "run_logged", return_value=0) as run:
            builder.run_matlab_convert(
                "matlab", subject_root, "static", "static01", self.root / "converted",
                self.root / "model.osim", "AB07_SEASEA", self.root / "convert.log",
            )
            self.assertEqual(run.call_args.args[0][:2], ["matlab", "-batch"])
            builder.run_matlab_convert(
                "matlab", subject_root, "treadmill", "treadmill_01_01", self.root / "converted",
                self.root / "model.osim", "AB07_SEASEA", self.root / "convert.log", nojvm=True,
            )
            self.assertEqual(run.call_args.args[0][:3], ["matlab", "-nojvm", "-batch"])
            builder.run_matlab_healthy_overlay(
                "matlab", "AB07", subject_root, "treadmill", "treadmill_01_01",
                self.root / "healthy", self.root / "healthy.log", nojvm=True,
            )
            self.assertEqual(run.call_args.args[0][:3], ["matlab", "-nojvm", "-batch"])

    def create_runner_inputs(self) -> runner.ToolConfig:
        inputs = self.root / "sources"
        inputs.mkdir()
        model = inputs / "AB07_SEASEA_stiff321_500_pi.osim"
        model.write_text(
            '<OpenSimDocument><Model><MarkerSet><objects><Marker name="L_Heel"/>'
            '</objects></MarkerSet><BodySet><objects><Body name="foot_l"/>'
            '<Body name="calcn_r"/></objects></BodySet></Model></OpenSimDocument>',
            encoding="utf-8",
        )
        trc = inputs / "markers.trc"
        trc.write_text("Fake TRC used only by mocked tool\n", encoding="utf-8")
        grf = inputs / "grf.mot"
        grf.write_text(
            "endheader\ntime\t"
            + "\t".join(
                f"{prefix}{axis}"
                for prefix in (
                    "ground_force1_v", "ground_force1_p", "ground_torque1_",
                    "ground_force2_v", "ground_force2_p", "ground_torque2_",
                )
                for axis in "xyz"
            )
            + "\n",
            encoding="utf-8",
        )
        support = inputs / "support"
        support.mkdir()
        for filename in ("CMC_Actuators.xml", "CMC_Tasks - modified Kp_Kv.xml"):
            (support / filename).write_text("<OpenSimDocument/>\n", encoding="utf-8")
        return runner.ToolConfig(
            opensim_cmd=None,
            plugin=inputs / "fake_plugin.dylib",
            model=model,
            marker_set=model,
            trc=trc,
            grf=grf,
            support_data=support,
            skip_scaling=True,
            ik_only=True,
            full_time_range=(1.0, 2.0),
        )

    def test_ik_only_stops_before_id_rra_and_smoke(self) -> None:
        cfg = self.create_runner_inputs()
        paths = runner.create_paths(self.root / "pipeline")

        def fake_ik(setup: Path, worker: Path, config: runner.ToolConfig, label: str) -> runner.CommandResult:
            tool = ET.parse(setup).getroot().find("InverseKinematicsTool")
            self.assertIsNotNone(tool)
            self.assertEqual(label, "ik_full")
            output_dir = Path(tool.findtext("results_directory"))
            output_dir.mkdir(parents=True, exist_ok=True)
            output = output_dir / tool.findtext("output_motion_file")
            output.write_text("endheader\ntime\tpros_knee_angle\n1\t0\n2\t1\n", encoding="utf-8")
            return runner.CommandResult(0, "fake completed marker IK", 0.0)

        with patch.object(runner, "run_opensim_tool", side_effect=fake_ik) as tool_run:
            with patch.object(runner, "run_smoke_test") as smoke:
                code = runner.run_pipeline(cfg, paths, run_smoke=True)
        self.assertEqual(code, 0)
        tool_run.assert_called_once()
        smoke.assert_not_called()
        self.assertTrue((paths.ik_dir / "IK_full_1p000_2p000.mot").is_file())
        self.assertFalse((paths.id_dir / "ID_setup.xml").exists())
        self.assertFalse((paths.rra_dir / "RRA_Actuators.xml").exists())

    def test_bundle_uses_exact_directory_and_single_pi_model(self) -> None:
        cfg = self.create_runner_inputs()
        bundle = self.root / "models" / "AB07_SEASEA_Threadmill"
        files = builder.build_bundle(
            "AB07", "treadmill", cfg.model, cfg.trc, cfg.grf,
            cfg.support_data, bundle, 1.0, 2.0,
            model_stem="AB07_SEASEA_stiff321_500_pi",
        )
        self.assertEqual(files["model"].name, "AB07_SEASEA_stiff321_500_pi.osim")
        self.assertEqual(files["setup"].name, "AB07_SEASEA_stiff321_500_pi_setup.xml")
        self.assertEqual(files["kinematics"].name, "IK_results_AB07_SEASEA.mot")
        self.assertEqual(files["grf"].name, "AB07_SEASEA_GRF_FullSpan.mot")
        self.assertEqual(list(bundle.glob("*.osim")), [files["model"]])
        setup = ET.parse(files["setup"]).getroot().find("CMC_Simulator_Setup")
        self.assertEqual(Path(setup.findtext("model_file")).resolve(), files["model"].resolve())
        self.assertEqual(Path(setup.findtext("kinematics_file")).resolve(), files["kinematics"].resolve())
        self.assertEqual(files["model"].read_bytes(), cfg.model.read_bytes())


if __name__ == "__main__":
    unittest.main()
