"""Model-layout migration: legacy inputs, portable paths and namespace safety."""
from __future__ import annotations

import sys
import json
import hashlib
import tempfile
import unittest
from dataclasses import replace
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

import workspace_bootstrap

workspace_bootstrap.ensure_simulator_paths()

from config import SimulatorConfig
from path_resolver import (
    BINARY_DETECTOR_PROFILE_PATH,
    LEGACY_BINARY_DETECTOR_PROFILE,
    LEGACY_MODEL_DIRECTORIES,
    ONLINE_GRF_DIR,
    ONLINE_GRF_PROFILES_DIR,
    ONLINE_GRF_PROFILE_FILENAMES,
    SHARED_GEOMETRY_DIR,
    PLUGIN_DIRECTORIES,
    _resolve_bundle_input_path,
    normalize_cli_existing_path,
    remap_model_path,
    remap_plugin_path,
    remap_binary_detector_profile_path,
    remap_online_grf_path,
    resolve_plugin_path,
    resolve_repo_path,
    resolve_simulator_paths,
)
from setup_io import DEFAULT_SETUP_XML_PATH, read_last_setup_path, read_setup_xml


class ModelLayoutPathsTests(unittest.TestCase):
    def test_shared_geometry_legacy_and_current_addresses(self):
        self.assertFalse((ROOT / "Geometry").exists())
        meshes = sorted(SHARED_GEOMETRY_DIR.glob("*"))
        self.assertEqual(len(meshes), 8)
        for mesh in meshes:
            legacy = "Geometry/" + mesh.name
            for raw in (
                legacy, "./" + legacy, legacy.replace("/", "\\"),
                str(ROOT / legacy),
                "models/Geometry/" + mesh.name,
                "C:/archive/CMC-like-Simulator - Claude/" + legacy,
                "//server/share/CMC-like-Simulator/" + legacy,
            ):
                with self.subTest(raw=raw):
                    self.assertEqual(resolve_repo_path(raw), mesh)
                    self.assertEqual(normalize_cli_existing_path(raw), str(mesh))
        self.assertEqual(resolve_repo_path("Geometry"), SHARED_GEOMETRY_DIR)

    def test_archived_and_existing_external_geometry_are_not_redirected(self):
        for raw in (
            "validation/frozen/Geometry/AM_foot_l.STL",
            str(ROOT / "validation/frozen/Geometry/AM_foot_l.STL"),
            "C:/validation/CMC-like-Simulator/Geometry/AM_foot_l.STL",
            "C:/other_project/Geometry/AM_foot_l.STL",
        ):
            with self.subTest(raw=raw):
                self.assertIsNone(remap_model_path(raw))
        with tempfile.TemporaryDirectory() as work:
            custom = Path(work) / "CMC-like-Simulator/Geometry/custom.STL"
            custom.parent.mkdir(parents=True)
            custom.write_text("external test fixture")
            self.assertIsNone(remap_model_path(custom))
            self.assertEqual(resolve_repo_path(custom), custom)

    def test_each_moved_directory_and_portable_separators(self):
        for old, new in LEGACY_MODEL_DIRECTORIES.items():
            for tail in ("", "/data/sample.mot"):
                legacy = "models/" + old + tail
                expected = ROOT / "models" / (new + tail)
                for text in (legacy, legacy.replace("/", "\\"), "./" + legacy):
                    with self.subTest(text=text):
                        self.assertEqual(resolve_repo_path(text), expected)
                        self.assertEqual(resolve_repo_path(expected), expected)

    def test_saved_absolute_checkout_paths(self):
        tail = "AB07_SEASEA_stiff321_500_pi_setup.xml"
        old = "models/AB07_SEASEA_Threadmill/" + tail
        expected = ROOT / "models/AB07/AB07_SEASEA_Threadmill" / tail
        examples = [
            str(ROOT / old),
            "C:/archive/CMC-like-Simulator - Claude/" + old,
            "C:\\archive\\CMC-like-Simulator\\" + old.replace("/", "\\"),
            "//server/share/CMC-like-Simulator - Claude/" + old,
            "/old-machine/CMC-like-Simulator/" + old,
        ]
        for text in examples:
            with self.subTest(text=text):
                self.assertEqual(remap_model_path(text), expected)
                self.assertEqual(normalize_cli_existing_path(text), str(expected))
                self.assertEqual(_resolve_bundle_input_path(ROOT / "elsewhere", text), expected)

    def test_nested_archives_and_unknown_foreign_roots_are_not_redirected(self):
        tail = "models/AB06_SEASEA_Threadmill/model.osim"
        examples = [
            "validation/stage3/build/" + tail,
            str(ROOT / "validation/stage3/baseline" / tail),
            "C:/foreign_project/" + tail,
            "models/unknown_subject/model.osim",
        ]
        for text in examples:
            with self.subTest(text=text):
                self.assertIsNone(remap_model_path(text))

    def test_existing_external_checkout_keeps_its_own_model(self):
        with tempfile.TemporaryDirectory() as work:
            custom = Path(work) / "CMC-like-Simulator/models/AB06_SEASEA_Threadmill/custom.osim"
            custom.parent.mkdir(parents=True)
            custom.write_text("external test fixture")
            self.assertIsNone(remap_model_path(custom))
            self.assertEqual(resolve_repo_path(custom), custom)

    def test_bundle_relative_inputs_and_seasea_are_unchanged(self):
        bundle = ROOT / "models/AB06/AB06_SEASEA_Threadmill"
        self.assertEqual(_resolve_bundle_input_path(bundle, "data/IK_results_AB06_SEASEA.mot"),
                         bundle / "data/IK_results_AB06_SEASEA.mot")
        self.assertEqual(resolve_repo_path("models/SEASEA/Adjusted_SEASEA.osim"),
                         ROOT / "models/SEASEA/Adjusted_SEASEA.osim")

    def test_default_setup_and_remembered_ab07_setup_resolve_all_inputs(self):
        # Exercise a saved legacy address without depending on the user's
        # currently selected subject or changing their remembered setup.
        with tempfile.TemporaryDirectory() as work:
            saved = Path(work) / "last_setup.json"
            saved.write_text(json.dumps({"setup_xml_path":
                "models/AB07_SEASEA_Threadmill/AB07_SEASEA_stiff321_500_pi_setup.xml"}))
            with patch("setup_io.LAST_SETUP_STATE_PATH", saved):
                ab07_setup = read_last_setup_path()
        for subject, setup_path in [("AB06", DEFAULT_SETUP_XML_PATH), ("AB07", ab07_setup)]:
            with self.subTest(subject=subject):
                self.assertIsNotNone(setup_path)
                setup = read_setup_xml(setup_path)
                for value in (setup.model_file, setup.kinematics_file,
                              setup.external_loads_xml, setup.reserve_actuators_xml):
                    self.assertTrue(value.is_file(), value)
                self.assertIn("models/" + subject + "/", setup.model_file.as_posix())
        paths = resolve_simulator_paths(SimulatorConfig())
        for value in (paths.model_path, paths.kinematics_path,
                      paths.external_loads_path, paths.reserve_actuators_path):
            self.assertTrue(value.is_file(), value)

    def test_historical_setup_bytes_work_without_rewriting_the_saved_input(self):
        # Copy a former setup to a temporary path, preserving its old model
        # addresses: the loader must map them without editing the document.
        old_bytes = b'''<?xml version="1.0" encoding="utf-8"?>
<OpenSimDocument Version="40000">
  <CMC_Simulator_Setup>
    <model_file>models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi.osim</model_file>
    <kinematics_file>models/AB06_SEASEA_Threadmill/data/IK_results_AB06_SEASEA.mot</kinematics_file>
    <external_loads_xml>models/AB06_SEASEA_Threadmill/data/ExternalForces.xml</external_loads_xml>
    <reserve_actuators_xml>models/AB06_SEASEA_Threadmill/data/CMC_Actuators.xml</reserve_actuators_xml>
    <t_start>11.99</t_start>
    <t_end>21.0</t_end>
  </CMC_Simulator_Setup>
</OpenSimDocument>'''
        with tempfile.TemporaryDirectory() as work:
            source = Path(work) / "old_setup.xml"
            source.write_bytes(old_bytes)
            setup = read_setup_xml(source)
            self.assertTrue(setup.model_file.is_file())
            self.assertTrue(setup.kinematics_file.is_file())
            self.assertEqual(source.read_bytes(), old_bytes)


class BinaryDetectorProfilePathsTests(unittest.TestCase):
    def test_saved_profile_addresses_resolve_to_the_original_operational_bytes(self):
        expected = BINARY_DETECTOR_PROFILE_PATH
        self.assertEqual(hashlib.sha256(expected.read_bytes()).hexdigest(),
                         "db704e502b99e49bea6d89493812bafdac748f8ce8d3ce28214ff624078539a2")
        current = str(expected.relative_to(ROOT))
        for relative in (LEGACY_BINARY_DETECTOR_PROFILE,
                         "online_grf_profiles/binary_detector/operational/heel_toe_v25.json",
                         current):
            for raw in (relative, "./" + relative, relative.replace("/", "\\"),
                        str(ROOT / relative),
                        "C:/archive/CMC-like-Simulator - Claude/" + relative,
                        "C:\\Users\\test\\CMC-like-Simulator\\" + relative.replace("/", "\\"),
                        "//server/share/CMC-like-Simulator/" + relative,
                        "/old-machine/CMC-like-Simulator/" + relative):
                with self.subTest(raw=raw):
                    self.assertEqual(resolve_repo_path(raw), expected)
                    self.assertEqual(normalize_cli_existing_path(raw), str(expected))
                    self.assertEqual(_resolve_bundle_input_path(ROOT / "elsewhere", raw),
                                     expected)

    def test_saved_simulator_config_loads_profile_without_rewriting_the_address(self):
        for raw in (LEGACY_BINARY_DETECTOR_PROFILE,
                    "online_grf_profiles/binary_detector/operational/heel_toe_v25.json",
                    "C:/old/CMC-like-Simulator/" + LEGACY_BINARY_DETECTOR_PROFILE):
            with self.subTest(raw=raw):
                cfg = replace(SimulatorConfig(), binary_phase_detector_profile_file=raw)
                paths = resolve_simulator_paths(cfg)
                self.assertEqual(paths.binary_phase_detector_profile_path,
                                 BINARY_DETECTOR_PROFILE_PATH)
                self.assertEqual(cfg.binary_phase_detector_profile_file, raw)

    def test_other_candidates_and_nested_snapshots_keep_their_own_identity(self):
        for raw in (
            "validation/frozen/" + LEGACY_BINARY_DETECTOR_PROFILE,
            str(ROOT / "validation/snapshot" / LEGACY_BINARY_DETECTOR_PROFILE),
            "validation/other_campaign/selected_candidate_profile.json",
            "online_grf_profiles/binary_detector/experimental/heel_toe_v25.json",
            "C:/other_project/" + LEGACY_BINARY_DETECTOR_PROFILE,
            "C:/validation/CMC-like-Simulator/" + LEGACY_BINARY_DETECTOR_PROFILE,
            "C:/runs/CMC-like-Simulator/" + LEGACY_BINARY_DETECTOR_PROFILE,
        ):
            with self.subTest(raw=raw):
                self.assertIsNone(remap_binary_detector_profile_path(raw))

    def test_existing_external_profile_is_not_replaced_by_the_local_geometry(self):
        with tempfile.TemporaryDirectory() as work:
            for relative in (LEGACY_BINARY_DETECTOR_PROFILE,
                             "online_grf_profiles/binary_detector/operational/heel_toe_v25.json",
                             str(BINARY_DETECTOR_PROFILE_PATH.relative_to(ROOT))):
                external = Path(work) / "CMC-like-Simulator" / relative
                external.parent.mkdir(parents=True, exist_ok=True)
                external.write_text('{"custom_external_geometry":true}')
                self.assertIsNone(remap_binary_detector_profile_path(external))
                self.assertEqual(resolve_repo_path(external), external)
                self.assertEqual(normalize_cli_existing_path(external), str(external))


class OnlineGRFLayoutPathsTests(unittest.TestCase):
    def test_operational_profiles_accept_saved_and_current_portable_addresses(self):
        self.assertEqual(len(ONLINE_GRF_PROFILE_FILENAMES), 3)
        for filename in ONLINE_GRF_PROFILE_FILENAMES:
            expected = ONLINE_GRF_PROFILES_DIR / filename
            self.assertTrue(expected.is_file(), expected)
            for relative in ("online_grf_profiles/" + filename,
                             "online_GRF/profiles/" + filename):
                for raw in (relative, "./" + relative, relative.replace("/", "\\"),
                            str(ROOT / relative),
                            "C:/archive/CMC-like-Simulator - Claude/" + relative,
                            "C:\\Users\\test\\CMC-like-Simulator\\" + relative.replace("/", "\\"),
                            "//server/share/CMC-like-Simulator/" + relative,
                            "/old-machine/CMC-like-Simulator/" + relative):
                    with self.subTest(raw=raw):
                        self.assertEqual(remap_online_grf_path(raw), expected)
                        self.assertEqual(resolve_repo_path(raw), expected)
                        self.assertEqual(normalize_cli_existing_path(raw), str(expected))
                        self.assertEqual(_resolve_bundle_input_path(ROOT / "elsewhere", raw),
                                         expected)

    def test_saved_simulator_config_preserves_its_profile_addresses(self):
        for filename in ONLINE_GRF_PROFILE_FILENAMES:
            for prefix in ("online_grf_profiles/",
                           "C:/archive/CMC-like-Simulator/online_grf_profiles/"):
                raw = prefix + filename
                with self.subTest(raw=raw):
                    cfg = replace(SimulatorConfig(), online_grf_profile_file=raw,
                                  online_grf_detector_profile_file=raw)
                    paths = resolve_simulator_paths(cfg)
                    expected = ONLINE_GRF_PROFILES_DIR / filename
                    self.assertEqual(paths.online_grf_profile_path, expected)
                    self.assertEqual(paths.online_grf_detector_profile_path, expected)
                    self.assertEqual(cfg.online_grf_profile_file, raw)
                    self.assertEqual(cfg.online_grf_detector_profile_file, raw)

    def test_moved_tool_and_detector_resources_keep_their_relative_suffix(self):
        for suffix in (
            "binary_detector/operational/heel_toe_v25.json",
            "calibration/generate_online_grf_profile.py",
            "contact/primary_grf_candidate_grid_v1.json",
            "detector/validate_online_grf_events.py",
            "validation/online_grf_acceptance_thresholds.json",
        ):
            expected = ONLINE_GRF_DIR / suffix
            self.assertTrue(expected.is_file(), expected)
            for namespace in ("online_grf_profiles/", "online_GRF/"):
                with self.subTest(suffix=suffix, namespace=namespace):
                    self.assertEqual(resolve_repo_path(namespace + suffix), expected)

    def test_retired_profiles_nested_snapshots_and_foreign_projects_are_not_substituted(self):
        filename = sorted(ONLINE_GRF_PROFILE_FILENAMES)[0]
        for relative in ("online_grf_profiles/" + filename,
                         "online_GRF/profiles/" + filename):
            for raw in ("validation/frozen/" + relative,
                        str(ROOT / "runs/snapshot" / relative),
                        "C:/other_project/" + relative,
                        "C:/validation/CMC-like-Simulator/" + relative,
                        "C:/runs/CMC-like-Simulator/" + relative,
                        "C:/snapshots/CMC-like-Simulator/" + relative):
                with self.subTest(raw=raw):
                    self.assertIsNone(remap_online_grf_path(raw))
        self.assertIsNone(remap_online_grf_path("online_grf_profiles/deleted_profile.json"))

    def test_existing_external_profiles_keep_their_own_bytes(self):
        filename = sorted(ONLINE_GRF_PROFILE_FILENAMES)[0]
        with tempfile.TemporaryDirectory() as work:
            for relative in ("online_grf_profiles/" + filename,
                             "online_GRF/profiles/" + filename,
                             "online_grf_profiles/contact/primary_grf_candidate_grid_v1.json"):
                external = Path(work) / "CMC-like-Simulator" / relative
                external.parent.mkdir(parents=True, exist_ok=True)
                payload = b'{"custom_external_profile":true}'
                external.write_bytes(payload)
                with self.subTest(relative=relative):
                    self.assertIsNone(remap_online_grf_path(external))
                    self.assertEqual(resolve_repo_path(external), external)
                    self.assertEqual(normalize_cli_existing_path(external), str(external))
                    self.assertEqual(external.read_bytes(), payload)


class PluginLayoutPathsTests(unittest.TestCase):
    def test_current_plugins_accept_legacy_and_grouped_portable_addresses(self):
        for stem, directory in PLUGIN_DIRECTORIES.items():
            for filename in (stem, "lib" + stem, stem + ".dll",
                             "lib" + stem + ".dylib", "lib" + stem + ".so"):
                expected = ROOT / "plugins" / directory / filename
                for relative in ("plugins/" + filename,
                                 "plugins/" + directory + "/" + filename):
                    for raw in (relative, "./" + relative,
                                relative.replace("/", "\\"), str(ROOT / relative)):
                        with self.subTest(raw=raw):
                            self.assertEqual(remap_plugin_path(raw), expected)
                            self.assertEqual(resolve_repo_path(raw), expected)
                            self.assertEqual(normalize_cli_existing_path(raw), str(expected))
                            self.assertEqual(resolve_plugin_path(raw), expected.with_name(stem))

    def test_saved_foreign_checkout_plugins_resolve_on_this_host(self):
        for stem, directory in PLUGIN_DIRECTORIES.items():
            expected = ROOT / "plugins" / directory / stem
            for prefix in ("C:/archive/CMC-like-Simulator - Claude/",
                           "C:\\archive\\CMC-like-Simulator\\",
                           "//server/share/CMC-like-Simulator/",
                           "/old-machine/CMC-like-Simulator - Claude/"):
                with self.subTest(prefix=prefix, stem=stem):
                    self.assertEqual(resolve_plugin_path(prefix + "plugins/" + stem), expected)

    def test_old_implementations_unknown_roots_and_archives_are_not_substituted(self):
        filename = "libSEA_Plugin_BlackBox_mCMC_impedence_ff.dylib"
        for raw in (
            "plugins/SEA_Plugin_BlackBox_mCMC_impedence",
            "plugins/libSEA_Plugin_BlackBox_mCMC_impedence_PID.dylib",
            "plugins/" + filename + ".no_pi_backup_20260517",
            "plugins/other/" + filename,
            "plugins/SEA/OnlineGRFContact",
            "validation/frozen/plugins/" + filename,
            str(ROOT / "validation/frozen/plugins" / filename),
            "C:/validation/CMC-like-Simulator/plugins/" + filename,
            "C:/runs/CMC-like-Simulator/plugins/" + filename,
            "C:/other_project/plugins/" + filename,
        ):
            with self.subTest(raw=raw):
                self.assertIsNone(remap_plugin_path(raw))

    def test_existing_external_libraries_and_extension_free_stems_keep_their_checkout(self):
        stem = "SEA_Plugin_BlackBox_mCMC_impedence_ff"
        with tempfile.TemporaryDirectory() as work:
            directory = Path(work) / "CMC-like-Simulator/plugins"
            directory.mkdir(parents=True)
            library = directory / ("lib" + stem + ".dylib")
            library.write_bytes(b"external test fixture")
            for raw in (library, directory / stem, directory / (stem + ".dll")):
                with self.subTest(raw=raw):
                    self.assertIsNone(remap_plugin_path(raw))
                    self.assertEqual(resolve_repo_path(raw), raw)
                    self.assertEqual(resolve_plugin_path(raw), directory / stem)

    def test_saved_plugin_settings_equal_new_defaults_without_rewriting_configuration(self):
        old = replace(
            SimulatorConfig(),
            plugin_name="C:\\archive\\CMC-like-Simulator\\plugins\\"
                        "SEA_Plugin_BlackBox_mCMC_impedence_ff.dll",
            online_grf_contact_plugin="plugins/libOnlineGRFContact.dylib",
        )
        before = (old.plugin_name, old.online_grf_contact_plugin)
        self.assertEqual(resolve_simulator_paths(old), resolve_simulator_paths(SimulatorConfig()))
        self.assertEqual((old.plugin_name, old.online_grf_contact_plugin), before)

    def test_visualizer_resolves_legacy_plugin_stem_without_a_cwd_dependency(self):
        from visualize import _resolve_project_path
        for stem, directory in PLUGIN_DIRECTORIES.items():
            expected = ROOT / "plugins" / directory / stem
            self.assertEqual(_resolve_project_path("plugins/" + stem, library_basename=True),
                             str(expected))

    def test_model_loader_maps_current_plugins_and_preserves_other_library_requests(self):
        from model_loader import _load_plugin
        with patch("model_loader.opensim.LoadOpenSimLibrary") as loader, \
                patch("model_loader._configure_windows_dll_search_path"):
            for stem, directory in PLUGIN_DIRECTORIES.items():
                for raw in ("plugins/" + stem,
                            "C:/archive/CMC-like-Simulator/plugins/" + stem + ".dll",
                            "plugins/lib" + stem + ".dylib"):
                    _load_plugin(raw)
                    loader.assert_called_with(str(ROOT / "plugins" / directory / stem))
            for raw in ("custom_library", "plugins/SEA_Plugin_BlackBox_mCMC_impedence",
                        "validation/frozen/plugins/OnlineGRFContact",
                        "/external/custom_library"):
                _load_plugin(raw)
                loader.assert_called_with(raw)


if __name__ == "__main__":
    unittest.main(verbosity=2)
