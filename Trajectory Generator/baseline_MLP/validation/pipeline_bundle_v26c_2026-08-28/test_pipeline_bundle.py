#!/usr/bin/env python3
"""Structural and byte-integrity tests for the V26C validation bundle."""

from __future__ import annotations

import json
import re
import subprocess
import sys
import tempfile
import threading
import unittest
from pathlib import Path, PurePosixPath


HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))
sys.dont_write_bytecode = True

import materialize_pipeline_bundle as bundle  # noqa: E402


class PipelineBundleTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.repo = bundle.find_repo_root(HERE)
        cls.bundle_manifest = json.loads(
            bundle.BUNDLE_MANIFEST.read_text(encoding="utf-8")
        )
        cls.in_place_manifest = json.loads(
            bundle.IN_PLACE_MANIFEST.read_text(encoding="utf-8")
        )

    def test_bundle_is_sibling_of_sealed_stage(self) -> None:
        sealed = HERE.parent / "v26c_july_replica_2026-08-26"
        self.assertTrue(sealed.is_dir())
        self.assertEqual(HERE.parent, sealed.parent)
        self.assertNotIn(sealed, HERE.parents)
        self.assertNotIn(HERE, sealed.parents)

    def test_manifest_schemas_and_policy(self) -> None:
        self.assertEqual(self.bundle_manifest["schema"], bundle.BUNDLE_SCHEMA)
        self.assertEqual(self.in_place_manifest["schema"], bundle.IN_PLACE_SCHEMA)
        policy = self.bundle_manifest["compatibility_policy"]
        self.assertTrue(policy["historical_paths_retained"])
        self.assertTrue(policy["production_runtime_retained"])
        self.assertTrue(policy["canonical_reports_retained"])
        self.assertFalse(policy["snapshot_is_runtime_canonical"])
        self.assertFalse(policy["symlinks_used"])

    def test_manifest_is_path_independent(self) -> None:
        drive = re.compile(r"^[A-Za-z]:[\\/]")

        def visit(value: object) -> None:
            if isinstance(value, dict):
                for child in value.values():
                    visit(child)
            elif isinstance(value, list):
                for child in value:
                    visit(child)
            elif isinstance(value, str):
                self.assertFalse(value.startswith("/"), value)
                self.assertIsNone(drive.match(value), value)

        visit(self.bundle_manifest)
        visit(self.in_place_manifest)
        serialized = json.dumps(self.bundle_manifest, sort_keys=True)
        self.assertNotIn("generated_at", serialized)
        self.assertNotIn("mtime", serialized)

    def test_repository_snapshot_paths_are_safe(self) -> None:
        for entry in self.bundle_manifest["entries"]:
            source = PurePosixPath(entry["original_repo_relpath"])
            target = PurePosixPath(entry["bundle_relpath"])
            self.assertFalse(source.is_absolute())
            self.assertFalse(target.is_absolute())
            self.assertNotIn("..", source.parts)
            self.assertNotIn("..", target.parts)
            self.assertEqual(target.parts[:2], ("objects", "sha256"))
            self.assertEqual(target.parts[2], entry["sha256"][:2])
            self.assertEqual(target.parts[3], entry["sha256"])
            self.assertLess(len(str(target)), 100)
        for unsafe in ("../escape", "/absolute", "C:/windows", "a\\b"):
            with self.assertRaises(bundle.BundleError):
                bundle.from_posix(HERE, unsafe)

    def test_audited_source_closure_is_exact(self) -> None:
        try:
            current = {
                spec.repo_relpath for spec in bundle.collect_source_specs(self.repo)
            }
        except bundle.BundleError as exc:
            self.skipTest(f"optional live source workspace is incomplete: {exc}")
        recorded = {
            entry["original_repo_relpath"]
            for entry in self.bundle_manifest["entries"]
        }
        self.assertEqual(current, recorded)

    def test_required_operational_and_analysis_anchors_are_present(self) -> None:
        recorded = {
            entry["original_repo_relpath"]
            for entry in self.bundle_manifest["entries"]
        }
        required = {
            "Trajectory Generator/baseline_MLP/train_ppo_mlp.py",
            "Trajectory Generator/baseline_MLP/tb_logging.py",
            "Trajectory Generator/baseline_MLP/asymmetric_rl_module.py",
            "Trajectory Generator/baseline_MLP/warm_start.py",
            "Trajectory Generator/baseline_MLP/target_domain_dagger.py",
            "Trajectory Generator/baseline_MLP/configure_actor_exploration.py",
            "Trajectory Generator/runs/training/MLP_ExNovo_B0820_fsmv3_fixedcorridor_50iter/training_cfg.resolved.yaml",
            "Trajectory Generator/runs/training/MLP_imitation_native_v26_08-20-2026_june_equiv_100iter/rl_module_best/module_state.pkl",
            "Trajectory Generator/runs/training/target_domain_markov35_phase_aligned_scaled_full_r32_alt8_2026-07-13/rl_module_target_adapted/module_state.pkl",
            "Trajectory Generator/runs/rollout/validation/v26b_bridge_runs/2026-08-24_V26B_anchors_r1/student/V1_35D_transplant/rl_module/module_state.pkl",
            "models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi_setup.xml",
            "models/AB06_SEASEA_Threadmill/data/IK_results_AB06_SEASEA.mot",
            "models/AB06_SEASEA_Threadmill/data/AB06_SEASEA_GRF_FullSpan.mot",
            "plugins/libSEA_Plugin_BlackBox_mCMC_impedence_ff.dylib",
            "plugins/libOnlineGRFContact.dylib",
            "Geometry/AM_foot_l.STL",
            "Geometry/osseo_femur.STL",
            "Geometry/pors_tibia.STL",
            "Geometry/transfemur_l.stl",
            "reports/user/2026-08-28_guida_ricostruzione_pipeline_exnovo_training_ready_da_imitativo_v26_agosto.md",
            "reports/user/2026-08-23_f1_s1_protocollo_tooling_dryrun.md",
            "reports/plans/2026-08-22_piano_operativo_recupero_pipeline_exnovo_v26_fsmv3_morphology.md",
            "validation/verify_h1_readiness.py",
        }
        self.assertEqual(set(), required - recorded)

    def test_bundle_pins_the_in_place_manifest(self) -> None:
        entries = self.bundle_manifest["related_manifests"]
        self.assertEqual(1, len(entries))
        entry = entries[0]
        self.assertEqual(bundle.IN_PLACE_MANIFEST.name, entry["bundle_relpath"])
        self.assertEqual(
            bundle.sha256_file(bundle.IN_PLACE_MANIFEST), entry["sha256"]
        )
        self.assertEqual(bundle.IN_PLACE_MANIFEST.stat().st_size, entry["size_bytes"])

    def test_external_j5_log_is_recovered(self) -> None:
        entries = self.bundle_manifest["external_entries"]
        self.assertEqual(1, len(entries))
        entry = entries[0]
        self.assertEqual(bundle.J5_LOG_SHA256, entry["sha256"])
        self.assertEqual(bundle.J5_LOG_BUNDLE_RELPATH, entry["bundle_relpath"])
        path = bundle.from_posix(HERE, entry["bundle_relpath"])
        self.assertEqual(bundle.J5_LOG_SHA256, bundle.sha256_file(path))

    def test_in_place_inventory_matches_workspace_file_set(self) -> None:
        current = {
            bundle.repo_relpath(path, self.repo)
            for path, _role in bundle._iter_in_place_files(self.repo)
        }
        recorded = {
            entry["repo_relpath"]
            for entry in self.in_place_manifest["entries"]
        }
        self.assertEqual(current, recorded)
        telemetry = (
            "Trajectory Generator/baseline_MLP/validation/"
            "test_training_health_telemetry.py"
        )
        self.assertIn(telemetry, recorded)
        self.assertIn(
            "Trajectory Generator/baseline_MLP/validation/README.md", recorded
        )
        self.assertTrue(
            any(
                path.startswith(
                    "Trajectory Generator/baseline_MLP/validation/"
                    "v26c_july_replica_2026-08-26/"
                )
                for path in recorded
            )
        )

    def test_ephemeral_files_are_excluded(self) -> None:
        paths = [
            PurePosixPath(entry["repo_relpath"])
            for entry in self.in_place_manifest["entries"]
        ]
        for path in paths:
            self.assertFalse(set(path.parts) & bundle.EXCLUDED_DIR_NAMES, path)
            self.assertNotIn(path.name, bundle.EXCLUDED_FILE_NAMES)
            self.assertNotIn(path.suffix.lower(), bundle.EXCLUDED_SUFFIXES)

    def test_snapshot_contains_no_symlinks(self) -> None:
        self.assertTrue(bundle.OBJECTS_DIR.is_dir())
        links = [path for path in bundle.OBJECTS_DIR.rglob("*") if path.is_symlink()]
        self.assertEqual([], links)

    def test_no_clobber_is_concurrency_safe_and_preserves_different_files(self) -> None:
        payload = b"v26c-no-clobber-race-test-" * 65536
        with tempfile.TemporaryDirectory(prefix="v26c-bundle-test-") as temp:
            root = Path(temp)
            source = root / "source.bin"
            destination = root / "destination.bin"
            source.write_bytes(payload)
            barrier = threading.Barrier(6)
            successes: list[bool] = []
            errors: list[Exception] = []

            def copy() -> None:
                try:
                    barrier.wait()
                    bundle._copy_no_clobber(source, destination)
                    successes.append(True)
                except bundle.BundleError as exc:
                    # A loser may observe the winning process's incomplete
                    # exclusive-create and fail safely; it must never delete it.
                    errors.append(exc)

            threads = [threading.Thread(target=copy) for _ in range(6)]
            for thread in threads:
                thread.start()
            for thread in threads:
                thread.join()
            self.assertTrue(successes)
            self.assertTrue(destination.is_file())
            self.assertEqual(bundle.sha256_file(source), bundle.sha256_file(destination))

            different = root / "different.bin"
            different.write_bytes(b"keep-existing-bytes")
            with self.assertRaises(bundle.BundleError):
                bundle._copy_no_clobber(source, different)
            self.assertEqual(b"keep-existing-bytes", different.read_bytes())

    def test_repository_ignore_rules_reinclude_the_byte_archive(self) -> None:
        ignore = (self.repo / ".gitignore").read_text(encoding="utf-8")
        root = (
            "Trajectory Generator/baseline_MLP/validation/"
            "pipeline_bundle_v26c_2026-08-28"
        )
        self.assertIn(root + "/repository_snapshot/", ignore)
        self.assertIn("!" + root + "/objects/**", ignore)

    def test_git_attributes_preserve_pins_across_platforms(self) -> None:
        first_object = self.bundle_manifest["entries"][0]["bundle_relpath"]
        paths = [
            "Trajectory Generator/baseline_MLP/train_ppo_mlp.py",
            "Trajectory Generator/baseline_MLP/validation/"
            "test_training_health_telemetry.py",
            "Trajectory Generator/baseline_MLP/validation/"
            "v26c_july_replica_2026-08-26/"
            "v26c_j21_training_ready_attestation.py",
            "Trajectory Generator/baseline_MLP/validation/"
            "pipeline_bundle_v26c_2026-08-28/" + first_object,
        ]
        result = subprocess.run(
            ["git", "check-attr", "text", "eol", "--", *paths],
            cwd=self.repo,
            check=True,
            capture_output=True,
            text=True,
        )
        lines = result.stdout.splitlines()
        self.assertIn(paths[0] + ": text: set", lines)
        self.assertIn(paths[0] + ": eol: lf", lines)
        self.assertIn(paths[1] + ": text: set", lines)
        self.assertIn(paths[1] + ": eol: lf", lines)
        self.assertIn(paths[2] + ": text: unset", lines)
        self.assertIn(paths[3] + ": text: unset", lines)

    def test_roles_distinguish_operational_inputs_from_context_extras(self) -> None:
        by_path = {
            entry["original_repo_relpath"]: entry
            for entry in self.bundle_manifest["entries"]
        }
        b0820 = (
            "Trajectory Generator/runs/training/"
            "MLP_ExNovo_B0820_fsmv3_fixedcorridor_50iter/"
        )
        self.assertIn(
            "runtime_configuration_anchor",
            by_path[b0820 + "training_cfg.resolved.yaml"]["roles"],
        )
        milestone = next(
            entry
            for path, entry in by_path.items()
            if path.startswith(b0820 + "milestone_iteration_")
        )
        self.assertEqual(["context_snapshot_extra"], milestone["roles"])
        parent_state = (
            "Trajectory Generator/runs/training/"
            "MLP_imitation_native_v26_08-20-2026_june_equiv_100iter/"
            "rl_module_best/module_state.pkl"
        )
        self.assertIn("operational_input", by_path[parent_state]["roles"])

    def test_windows_path_policy_is_explicit(self) -> None:
        policy = self.bundle_manifest["compatibility_policy"]["windows_checkout"]
        self.assertEqual(248, policy["legacy_path_budget_chars"])
        self.assertGreater(policy["short_root_max_chars_without_long_path_support"], 0)
        self.assertIn("core.longpaths", policy["requirement"])

    def test_declared_gaps_are_not_silenced(self) -> None:
        gaps = {entry["id"]: entry["status"] for entry in self.bundle_manifest["gaps"]}
        self.assertEqual("missing", gaps["windows_plugin_binaries"])
        self.assertEqual("not_created_historically", gaps["ad_hoc_analysis_scripts"])
        self.assertEqual("not_closed", gaps["opensim_visual_meshes"])
        self.assertEqual("partial", gaps["environment_reproducibility"])

    def test_every_object_and_in_place_artefact_matches_sha256(self) -> None:
        counts = bundle.verify(self.repo, full=True, check_live_sources=False)
        self.assertGreater(counts["source_entry_count"], 50)
        self.assertGreater(counts["object_count"], 50)
        self.assertGreater(counts["object_bytes"], 0)
        self.assertEqual(1, counts["external_file_count"])
        self.assertGreater(counts["in_place_file_count"], 100)

    def test_archive_verification_is_standalone(self) -> None:
        counts = bundle.verify(None, full=False, check_live_sources=False)
        self.assertEqual(
            self.bundle_manifest["summary"]["object_count"],
            counts["object_count"],
        )
        policy = self.bundle_manifest["compatibility_policy"]["live_source_audit"]
        self.assertFalse(policy["default"])
        self.assertEqual("host_source_workspace_only", policy["scope"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
