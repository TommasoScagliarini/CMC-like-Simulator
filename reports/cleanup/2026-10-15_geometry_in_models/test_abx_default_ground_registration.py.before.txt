"""Public ABx preparation always registers hardware and computes marker IK.

All models/data here are temporary existence fixtures. Native fit, MATLAB,
marker calibration, IK and subprocess calls are mocked; no pipeline is run.
"""
from __future__ import annotations

from contextlib import ExitStack, redirect_stderr, redirect_stdout
from copy import deepcopy
import hashlib
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

REPO = Path(__file__).resolve().parents[1]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))
from tools import build_abx_seasea_pipeline as builder


class DefaultGroundPreparationTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="abx_default_ground_")
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.models = self.root / "models"
        self.donor = self.models / "AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi.osim"
        self.healthy = self.models / "AB06-raw/osimxml/AB06.osim"
        self.static = self.models / "AB06_SEASEA-raw/data/converted/static/static_01/static_01_ik_dataset_ab06_seasea.mot"
        self.plugin = self.root / "frozen_plugin.library"
        self.support = self.root / "support"
        self.support.mkdir()
        for path in (self.donor, self.healthy, self.static, self.plugin):
            self.placeholder(path)

    def placeholder(self, path):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(b"temporary existence fixture; never passed to a native tool")

    def argv(self, subject="AB07", *extra):
        return ["build_abx_seasea_pipeline.py", "--subject", subject,
                "--models-dir", str(self.models), "--plugin", str(self.plugin),
                "--support-data", str(self.support), "--output-models-dir", str(self.root / "output_models"),
                "--results-dir", str(self.root / "output_results"), *extra]

    def parse(self, subject="AB07", *extra):
        with patch.object(sys, "argv", self.argv(subject, *extra)):
            return builder.parse_args()

    def discovery(self, subject="AB07"):
        session = self.models / f"{subject}-raw/10_02_18"
        static = builder.TrialSelection("static", "static01", session / "static/markers/static01.mat", None,
                                        session / "static/ik/static01.mat", session / "static/id/static01.mat")
        trial = builder.TrialSelection("treadmill", "treadmill_01_01", session / "treadmill/markers/trial.mat",
                                       session / "treadmill/fp/trial.mat", session / "treadmill/ik/trial.mat",
                                       session / "treadmill/id/trial.mat")
        return builder.SubjectDiscovery(subject, self.models / f"{subject}-raw", session,
                                        self.models / f"{subject}-raw/osimxml/{subject.lower()}.osim", static, trial)

    def resolved_payload_path(self, value):
        path = Path(value)
        return (path if path.is_absolute() else REPO / path).resolve()

    def test_ab06_ab07_ab08_share_default_reference_contract(self):
        for subject in ("AB06", "AB07", "AB08"):
            with self.subTest(subject=subject):
                args = self.parse(subject)
                self.assertIsNone(args.donor_model)
                self.assertEqual(builder.default_donor_model(args.models_dir).resolve(), self.donor.resolve())
                self.assertEqual(args.fit_reference_model.resolve(), self.healthy.resolve())
                self.assertEqual(args.ground_reference_motion.resolve(), self.static.resolve())
                self.assertEqual(args.ground_plane_profile.resolve(), builder.DEFAULT_GROUND_PLANE_PROFILE.resolve())
                self.assertFalse(args.skip_opensim_ik)

    def test_models_dir_default_references_stay_local_and_accept_healthy_filename_case(self):
        self.healthy.unlink()
        lowercase = self.healthy.with_name("ab06.osim")
        self.placeholder(lowercase)
        args = self.parse("AB08")
        # Mac/Windows may expose the preferred spelling for the same inode.
        self.assertTrue(args.fit_reference_model.samefile(lowercase))
        self.assertEqual(args.ground_reference_motion.resolve(), self.static.resolve())
        self.assertEqual(builder.default_donor_model(args.models_dir).resolve(), self.donor.resolve())
        for path in (args.fit_reference_model, args.ground_reference_motion):
            self.assertIn(self.models.resolve(), path.resolve().parents)

    def test_missing_pi_donor_never_falls_back_to_legacy_raw_model(self):
        self.donor.unlink()
        legacy = self.models / "AB06_SEASEA-raw/osimxml/AB06_SEASEA.osim"
        self.placeholder(legacy)
        self.assertEqual(builder.default_donor_model(self.models), self.donor)
        with patch.object(sys, "argv", self.argv("AB07", "--validate-only")), \
             patch.object(builder, "discover_subject", return_value=self.discovery()), \
             patch.object(builder, "validate_discovery", side_effect=AssertionError("legacy donor reached validation")):
            with self.assertRaisesRegex(FileNotFoundError, "Donor SEASEA model"):
                builder.main()

    def test_explicit_reference_plane_mesh_overrides_are_preserved(self):
        values = {"fit-reference-model": self.root / "other/healthy.osim",
                  "ground-reference-motion": self.root / "other/static.mot",
                  "ground-plane-profile": self.root / "other/plane.json",
                  "foot-mesh": self.root / "other/foot.STL"}
        extra = [item for flag, path in values.items() for item in ("--" + flag, str(path))]
        args = self.parse("AB07", *extra)
        for flag, path in values.items():
            self.assertEqual(getattr(args, flag.replace("-", "_")).resolve(), path.resolve())

    def test_custom_donor_requires_both_explicit_anatomical_and_static_references(self):
        custom = self.root / "custom/donor.osim"
        for extra in ([], ["--fit-reference-model", str(self.healthy)],
                      ["--ground-reference-motion", str(self.static)]):
            with self.subTest(extra=extra), redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit) as error:
                    self.parse("AB08", "--donor-model", str(custom), *extra)
                self.assertEqual(error.exception.code, 2)

    def test_custom_donor_with_explicit_pair_and_canonical_donor_with_defaults_are_allowed(self):
        custom = self.root / "custom/donor.osim"
        healthy = self.root / "custom/healthy.osim"
        static = self.root / "custom/static.mot"
        args = self.parse("AB08", "--donor-model", str(custom), "--fit-reference-model", str(healthy),
                          "--ground-reference-motion", str(static))
        self.assertEqual(args.donor_model, custom)
        self.assertEqual(args.fit_reference_model, healthy)
        self.assertEqual(args.ground_reference_motion, static)
        self.assertEqual(args.ground_plane_profile, builder.DEFAULT_GROUND_PLANE_PROFILE)
        canonical = self.parse("AB07", "--donor-model", str(self.donor))
        self.assertEqual(canonical.fit_reference_model.resolve(), self.healthy.resolve())
        self.assertEqual(canonical.ground_reference_motion.resolve(), self.static.resolve())

    def test_skip_marker_ik_rejected_even_without_registration_flags(self):
        errors = io.StringIO()
        with redirect_stderr(errors), self.assertRaises(SystemExit) as error:
            self.parse("AB06", "--skip-opensim-ik")
        self.assertEqual(error.exception.code, 2)
        self.assertIn("requires a new marker IK", errors.getvalue())

    def test_validate_only_exposes_resolved_contract_without_build_or_native_execution(self):
        output = io.StringIO()
        with ExitStack() as stack:
            stack.enter_context(patch.object(sys, "argv", self.argv("AB08", "--validate-only")))
            stack.enter_context(patch.object(builder, "discover_subject", return_value=self.discovery("AB08")))
            stack.enter_context(patch.object(builder, "python_has_opensim", return_value=True))
            for name in ("freeze_preparation_references", "build_subject", "graft_seasea_model", "run_matlab_convert", "run_ground_registration",
                         "run_marker_calibration", "run_ik_pipeline", "run_smoke"):
                stack.enter_context(patch.object(builder, name, side_effect=AssertionError(f"unexpected {name}")))
            stack.enter_context(patch.object(builder.subprocess, "run", side_effect=AssertionError("unexpected subprocess")))
            stack.enter_context(redirect_stdout(output))
            self.assertEqual(builder.main(), 0)
        payload = json.loads(output.getvalue())
        preparation = payload["preparation"]
        self.assertTrue(preparation["anatomical_fitting_required"])
        self.assertTrue(preparation["ground_registration_required"])
        self.assertEqual(preparation["ik_method"], "opensim_marker_ik")
        self.assertFalse(preparation["dataset_ik_fallback"])
        expected = {"reference_donor": self.donor, "reference_healthy_model": self.healthy,
                    "reference_static_motion": self.static, "ground_plane_profile": builder.DEFAULT_GROUND_PLANE_PROFILE,
                    "foot_mesh": REPO / "Geometry/AM_foot_l.STL"}
        for name, path in expected.items():
            self.assertEqual(self.resolved_payload_path(preparation[name]), path.resolve())
        self.assertFalse((self.root / "output_models").exists())
        self.assertFalse((self.root / "output_results").exists())

    def test_missing_common_inputs_fail_before_preflight_or_build(self):
        plane, mesh = self.root / "plane.json", self.root / "foot.STL"
        common = {self.healthy: "Healthy fitting reference model", self.static: "Donor static registration motion",
                  plane: "Shared ground registration profile", mesh: "Fixed foot hardware STL", self.donor: "Donor SEASEA model"}
        for missing, label in common.items():
            for validate_only in (False, True):
                with self.subTest(missing=missing.name, validate_only=validate_only):
                    for path in common:
                        if path == missing:
                            if path.exists():
                                path.unlink()
                        else:
                            self.placeholder(path)
                    extra = ["--ground-plane-profile", str(plane), "--foot-mesh", str(mesh)]
                    if validate_only:
                        extra.append("--validate-only")
                    with ExitStack() as stack:
                        stack.enter_context(patch.object(sys, "argv", self.argv("AB07", *extra)))
                        stack.enter_context(patch.object(builder, "discover_subject", return_value=self.discovery()))
                        for name in ("validate_discovery", "require_python_opensim", "freeze_preparation_references", "build_subject", "graft_seasea_model",
                                     "run_matlab_convert", "run_ground_registration", "run_marker_calibration", "run_ik_pipeline"):
                            stack.enter_context(patch.object(builder, name, side_effect=AssertionError(f"missing input reached {name}")))
                        stack.enter_context(patch.object(builder.subprocess, "run", side_effect=AssertionError("unexpected subprocess")))
                        with self.assertRaisesRegex(FileNotFoundError, label):
                            builder.main()
                    self.assertFalse((self.root / "output_models").exists())
                    self.assertFalse((self.root / "output_results").exists())

    def test_reference_snapshot_preserves_role_bytes_after_originals_are_rewritten(self):
        args = self.parse()
        sources = {"donor_model": self.donor, "healthy_model": self.healthy, "static_motion": self.static}
        originals = {}
        for role, path in sources.items():
            originals[role] = (role + " original common benchmark bytes").encode()
            path.write_bytes(originals[role])
        snapshot_donor = builder.freeze_preparation_references(args, self.donor)
        snapshots = {"donor_model": snapshot_donor, "healthy_model": args.fit_reference_model,
                     "static_motion": args.ground_reference_motion}
        manifest = args.preparation_references
        persisted = json.loads(self.resolved_payload_path(manifest["manifest"]).read_text())
        self.assertEqual(persisted, {name: value for name, value in manifest.items() if name != "manifest"})
        self.assertEqual(manifest["method"], "one_reference_snapshot_per_build")
        self.assertEqual(set(manifest["inputs"]), set(sources))
        for role, source in sources.items():
            snapshot = snapshots[role]
            entry = manifest["inputs"][role]
            self.assertNotEqual(snapshot.resolve(), source.resolve())
            self.assertEqual(snapshot.name, source.name)
            self.assertEqual(snapshot.read_bytes(), originals[role])
            self.assertEqual(entry["sha256"], hashlib.sha256(originals[role]).hexdigest())
            self.assertEqual(self.resolved_payload_path(entry["source_file"]), source.resolve())
            self.assertEqual(self.resolved_payload_path(entry["snapshot_file"]), snapshot.resolve())
            source.write_bytes((role + " overwritten by later subject preparation").encode())
        for role, snapshot in snapshots.items():
            self.assertEqual(snapshot.read_bytes(), originals[role])
        self.assertEqual(args.ground_plane_profile.resolve(), builder.DEFAULT_GROUND_PLANE_PROFILE.resolve())

    def test_all_subjects_use_one_snapshot_even_when_ab06_rewrites_original_references(self):
        calls = []
        originals = {"donor_model": self.donor.read_bytes(), "healthy_model": self.healthy.read_bytes(),
                     "static_motion": self.static.read_bytes()}
        argv = self.argv()
        argv[1:3] = ["--all-subjects"]

        def build(discovery, args, donor, plugin, support):
            references = {"donor_model": donor, "healthy_model": args.fit_reference_model,
                          "static_motion": args.ground_reference_motion}
            calls.append({"subject": discovery.subject, "paths": references,
                          "bytes": {role: path.read_bytes() for role, path in references.items()},
                          "manifest": deepcopy(args.preparation_references)})
            if discovery.subject == "AB06":
                for path in (self.donor, self.healthy, self.static):
                    path.write_bytes(b"rewritten original after reference snapshot")
            return builder.BuildReport(subject=discovery.subject, task="treadmill", trial="treadmill_01_01",
                                       static_trial="static01", status="complete",
                                       preparation_references=deepcopy(args.preparation_references))

        with ExitStack() as stack:
            stack.enter_context(patch.object(sys, "argv", argv))
            stack.enter_context(patch.object(builder, "discover_subjects", return_value=["AB06", "AB07", "AB08"]))
            stack.enter_context(patch.object(builder, "discover_subject", side_effect=lambda models, subject, *rest: self.discovery(subject)))
            stack.enter_context(patch.object(builder, "require_python_opensim", return_value=None))
            freeze = stack.enter_context(patch.object(builder, "freeze_preparation_references", wraps=builder.freeze_preparation_references))
            stack.enter_context(patch.object(builder, "build_subject", side_effect=build))
            for name in ("graft_seasea_model", "run_matlab_convert", "run_ground_registration", "run_marker_calibration", "run_ik_pipeline", "run_smoke"):
                stack.enter_context(patch.object(builder, name, side_effect=AssertionError(f"unexpected native operation {name}")))
            stack.enter_context(patch.object(builder.subprocess, "run", side_effect=AssertionError("unexpected subprocess")))
            stack.enter_context(redirect_stdout(io.StringIO()))
            self.assertEqual(builder.main(), 0)
            freeze.assert_called_once()
        self.assertEqual([call["subject"] for call in calls], ["AB06", "AB07", "AB08"])
        for call in calls:
            self.assertEqual(call["paths"], calls[0]["paths"])
            self.assertEqual(call["bytes"], originals)
            self.assertEqual(call["manifest"], calls[0]["manifest"])
        self.assertEqual(len(list((self.root / "output_results").glob("abx_reference_*/manifest.json"))), 1)
        self.assertFalse((self.root / "output_models").exists())

    def route_until_failure(self, failing_stage):
        """Exercise actual main/build_subject routing with every tool mocked."""
        events = []
        captured = {}
        static_files = {name: self.root / "converted_static" / name for name in ("dir", "ik_dataset", "trc", "grf")}
        trial_files = {name: self.root / "converted_trial" / name for name in ("dir", "ik_dataset", "trc", "grf")}

        def graft(*args, **kwargs):
            events.append("anatomical_fit")
            captured["graft"] = (args, kwargs)
            return 58.8, 58.8, 9.0, []

        def convert(*args, **kwargs):
            events.append(f"convert_{args[2]}")

        def ground(*args, **kwargs):
            events.append("ground_registration")
            captured["ground"] = args
            if failing_stage == "ground":
                raise RuntimeError("sentinel failed ground registration")
            return {"method": "donor_relative_static_hardware_ground_v1"}

        def calibrate(*args, **kwargs):
            events.append("marker_calibration")
            captured["calibration"] = args

        def ik(*args, **kwargs):
            events.append("new_marker_ik")
            captured["ik"] = args
            raise RuntimeError("sentinel failed new marker IK")

        with ExitStack() as stack:
            stack.enter_context(patch.object(sys, "argv", self.argv()))
            stack.enter_context(patch.object(builder, "discover_subject", return_value=self.discovery()))
            stack.enter_context(patch.object(builder, "require_python_opensim", return_value=None))
            stack.enter_context(patch.object(builder, "graft_seasea_model", side_effect=graft))
            stack.enter_context(patch.object(builder, "run_matlab_convert", side_effect=convert))
            stack.enter_context(patch.object(builder, "converted_trial_files", side_effect=[static_files, trial_files]))
            stack.enter_context(patch.object(builder, "run_ground_registration", side_effect=ground))
            stack.enter_context(patch.object(builder, "run_marker_calibration", side_effect=calibrate))
            stack.enter_context(patch.object(builder, "mean_marker_errors", return_value=(0.01, 0.0003)))
            stack.enter_context(patch.object(builder, "read_time_range", return_value=(10.36, 153.415)))
            stack.enter_context(patch.object(builder, "run_ik_pipeline", side_effect=ik))
            for name in ("build_bundle", "run_matlab_healthy_overlay", "run_smoke", "run_rra_pipeline", "run_logged"):
                stack.enter_context(patch.object(builder, name, side_effect=AssertionError(f"failed preparation reached {name}")))
            stack.enter_context(patch.object(builder.subprocess, "run", side_effect=AssertionError("unexpected subprocess")))
            stack.enter_context(redirect_stdout(io.StringIO()))
            stack.enter_context(redirect_stderr(io.StringIO()))
            with self.assertRaisesRegex(RuntimeError, "sentinel failed"):
                builder.main()
        self.assertFalse((self.root / "output_models").exists())
        self.assertFalse((self.root / "output_results/abx_seasea_last_build_summary.json").exists())
        return events, captured, static_files, trial_files

    def test_main_registers_before_calibration_and_new_ik_failure_aborts_without_fallback(self):
        events, captured, static, trial = self.route_until_failure("ik")
        self.assertEqual(events, ["anatomical_fit", "convert_static", "convert_treadmill", "ground_registration",
                                  "marker_calibration", "new_marker_ik"])
        graft_args, graft_kwargs = captured["graft"]
        ground, calibration, ik = captured["ground"], captured["calibration"], captured["ik"]
        self.assertEqual(graft_kwargs["fit_reference_model"].name, self.healthy.name)
        self.assertEqual(graft_kwargs["fit_reference_model"].read_bytes(), self.healthy.read_bytes())
        self.assertEqual(ground[1], graft_args[3])
        self.assertEqual(ground[2], static["ik_dataset"])
        self.assertEqual(ground[3], graft_args[2])
        self.assertEqual(ground[3].read_bytes(), self.donor.read_bytes())
        self.assertEqual(ground[4].name, self.static.name)
        self.assertEqual(ground[4].read_bytes(), self.static.read_bytes())
        self.assertEqual(ground[6], builder.DEFAULT_GROUND_PLANE_PROFILE.resolve())
        self.assertEqual(calibration[1], ground[7])
        self.assertEqual(calibration[3], ground[8])
        self.assertEqual(ik[2], calibration[4])
        self.assertEqual(ik[3], trial["trc"])

    def test_failed_ground_registration_aborts_before_marker_calibration_and_ik(self):
        events, _, _, _ = self.route_until_failure("ground")
        self.assertEqual(events, ["anatomical_fit", "convert_static", "convert_treadmill", "ground_registration"])

    def test_direct_build_cannot_bypass_required_references_or_new_ik_before_mutation(self):
        original = self.parse()
        cases = [("skip_opensim_ik", True)] + [(name, None) for name in
                ("fit_reference_model", "ground_reference_motion", "ground_plane_profile")]
        for field, value in cases:
            with self.subTest(field=field):
                args = deepcopy(original)
                setattr(args, field, value)
                with patch.object(builder, "graft_seasea_model", side_effect=AssertionError("invalid input reached model mutation")), \
                     patch.object(builder, "ensure_overwrite_policy", side_effect=AssertionError("invalid input reached overwrite policy")):
                    with self.assertRaises(ValueError):
                        builder.build_subject(self.discovery(), args, self.donor, self.plugin, self.support)
                self.assertFalse((self.root / "output_models").exists())


if __name__ == "__main__":
    unittest.main()
