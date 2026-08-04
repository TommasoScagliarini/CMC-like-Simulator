"""Pure golden tests for the portable EPIC AB06 table converter.

Only ``treadmill_01_01``, whose full interval was already consumed by V13, is
decoded.  No later treadmill trial is opened or referenced by these tests.
"""

from __future__ import annotations

import hashlib
import json
import shutil
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
from xml.etree import ElementTree as ET

import numpy as np
from scipy.io import savemat

from tools import convert_epic_ab06_tables as subject


REPO_ROOT = Path(__file__).resolve().parents[1]
RAW_TRIAL_ROOT = REPO_ROOT / "models/AB06-raw/10_09_18/treadmill"
TRIAL = "treadmill_01_01"
IK_MAT = RAW_TRIAL_ROOT / "ik" / f"{TRIAL}.mat"
FP_MAT = RAW_TRIAL_ROOT / "fp" / f"{TRIAL}.mat"
MARKERS_MAT = RAW_TRIAL_ROOT / "markers" / f"{TRIAL}.mat"
MATLAB_OUTPUT = (
    REPO_ROOT
    / "models/AB06_SEASEA-raw/data/converted/treadmill"
    / TRIAL
)
MATLAB_IK = MATLAB_OUTPUT / f"{TRIAL}_ik_dataset_ab06_seasea.mot"
MATLAB_GRF = MATLAB_OUTPUT / f"{TRIAL}_grf.mot"
MATLAB_TRC = MATLAB_OUTPUT / f"{TRIAL}.trc"
IK_MODEL = (
    REPO_ROOT
    / "models/AB06_SEASEA-raw/osimxml/AB06_SEASEA_marker_calibrated.osim"
)
IK_PLUGIN = (
    REPO_ROOT / "plugins/libSEA_Plugin_BlackBox_mCMC_impedence_ff.dylib"
)
V13_IK = REPO_ROOT / "models/AB06_SEASEA_Threadmill/data/IK_results_AB06_SEASEA.mot"
V13_IK_SHA256 = "ce4a948fd8f01f34ff32e4680b8a082f2e44de155aed89b7e4a55d37016c3596"


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _read_storage(path: Path) -> tuple[np.ndarray, tuple[str, ...], np.ndarray]:
    lines = path.read_text(encoding="utf-8", errors="strict").splitlines()
    try:
        header_index = lines.index("endheader")
    except ValueError as exc:
        raise AssertionError(f"missing endheader in {path}") from exc
    label_index = next(
        index
        for index in range(header_index + 1, len(lines))
        if lines[index].strip()
    )
    labels = tuple(lines[label_index].split())
    values = np.loadtxt(path, skiprows=label_index + 1, ndmin=2)
    return values[:, 0], labels[1:], values[:, 1:]


def _read_trc(path: Path) -> tuple[tuple[str, ...], np.ndarray]:
    lines = path.read_text(encoding="utf-8", errors="strict").splitlines()
    frame_index = next(
        index for index, line in enumerate(lines) if line.startswith("Frame#")
    )
    markers = tuple(
        item for item in lines[frame_index].split("\t")[2:] if item.strip()
    )
    values = np.loadtxt(path, skiprows=frame_index + 2, ndmin=2)
    return markers, values


class ConvertEpicAb06TablesTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.ik_table = subject.decode_mcos_table(IK_MAT)
        cls.fp_table = subject.decode_mcos_table(FP_MAT)
        cls.marker_table = subject.decode_mcos_table(MARKERS_MAT)

    def test_mcos_decoder_recovers_frozen_trial01_schemas(self) -> None:
        self.assertEqual(self.ik_table.names, subject.IK_SOURCE_COLUMNS)
        self.assertEqual(self.ik_table.nrows, 28_612)
        self.assertEqual(self.ik_table.column("Header")[[0, -1]].tolist(), [11.99, 155.045])

        self.assertEqual(self.fp_table.names, subject.FP_SOURCE_COLUMNS)
        self.assertEqual(self.fp_table.nrows, 143_056)
        self.assertEqual(self.fp_table.column("Header")[[0, -1]].tolist(), [11.99, 155.045])

        self.assertEqual(self.marker_table.nrows, 28_612)
        self.assertEqual(
            set(self.marker_table.names),
            subject.MARKER_REQUIRED_COLUMNS | subject.MARKER_ALLOWED_EXTRA_COLUMNS,
        )

    def test_ik_mapping_is_numerically_identical_to_matlab_golden(self) -> None:
        converted = subject.extract_coordinate_data(self.ik_table)
        time, labels, values = _read_storage(MATLAB_IK)
        self.assertEqual(converted.labels, labels)
        np.testing.assert_array_equal(converted.time, time)
        np.testing.assert_array_equal(converted.values, values)

    def test_grf_mapping_matches_matlab_golden_to_print_precision(self) -> None:
        converted = subject.extract_grf_data(self.fp_table)
        time, labels, values = _read_storage(MATLAB_GRF)
        self.assertEqual(converted.labels, labels)
        np.testing.assert_array_equal(converted.time, time)
        np.testing.assert_allclose(converted.values, values, rtol=0.0, atol=5.1e-11)
        np.testing.assert_array_equal(converted.values[:, [12, 14, 15, 17]], 0.0)

    def test_marker_mapping_matches_matlab_trc_to_print_precision(self) -> None:
        converted = subject.extract_marker_data(self.marker_table)
        markers, golden = _read_trc(MATLAB_TRC)
        self.assertEqual(converted.names, markers)
        self.assertEqual(converted.units, "mm")
        self.assertAlmostEqual(converted.rate_hz, 200.0)
        np.testing.assert_array_equal(converted.time, golden[:, 1])
        np.testing.assert_allclose(
            converted.values.reshape(converted.time.size, -1),
            golden[:, 2:],
            rtol=0.0,
            atol=5.1e-11,
            equal_nan=True,
        )

    def test_full_writer_roundtrip_and_portable_external_loads(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary) / "converted_trial01"
            result = subject.convert_trial(
                ik_mat=IK_MAT,
                fp_mat=FP_MAT,
                markers_mat=MARKERS_MAT,
                output_dir=output,
                trial=TRIAL,
                ik_model=IK_MODEL,
                ik_plugin=IK_PLUGIN,
            )
            self.assertEqual(result["status"], "ok")
            generated_ik = output / f"{TRIAL}_ik_dataset_ab06_seasea.mot"
            generated_grf = output / f"{TRIAL}_grf.mot"
            generated_trc = output / f"{TRIAL}.trc"
            generated_external = output / f"{TRIAL}_ExternalLoads.xml"
            generated_setup = output / f"{TRIAL}_iksetup.xml"
            generated_manifest = output / f"{TRIAL}_conversion_manifest.json"

            expected_ik = _read_storage(MATLAB_IK)
            observed_ik = _read_storage(generated_ik)
            self.assertEqual(expected_ik[1], observed_ik[1])
            np.testing.assert_array_equal(expected_ik[0], observed_ik[0])
            np.testing.assert_array_equal(expected_ik[2], observed_ik[2])

            expected_grf = _read_storage(MATLAB_GRF)
            observed_grf = _read_storage(generated_grf)
            self.assertEqual(expected_grf[1], observed_grf[1])
            np.testing.assert_array_equal(expected_grf[0], observed_grf[0])
            np.testing.assert_array_equal(expected_grf[2], observed_grf[2])

            expected_markers, expected_trc = _read_trc(MATLAB_TRC)
            observed_markers, observed_trc = _read_trc(generated_trc)
            self.assertEqual(expected_markers, observed_markers)
            self.assertTrue(
                np.array_equal(expected_trc, observed_trc, equal_nan=True)
            )

            root = ET.parse(generated_external).getroot()
            loads = list(root.iter("ExternalForce"))
            self.assertEqual(len(loads), 2)
            self.assertEqual(
                [item.findtext("applied_to_body") for item in loads],
                ["foot_l", "calcn_r"],
            )
            self.assertEqual(
                [item.findtext("force_identifier") for item in loads],
                ["ground_force1_v", "ground_force2_v"],
            )
            self.assertEqual(
                root.findtext("./ExternalLoads/datafile"), generated_grf.name
            )
            xml_text = generated_external.read_text(encoding="utf-8")
            self.assertNotIn(str(output.resolve()), xml_text)

            for portable_output in (generated_grf, generated_trc):
                self.assertNotIn(
                    str(output.resolve()),
                    portable_output.read_text(encoding="utf-8"),
                )

            setup_root = ET.parse(generated_setup).getroot()
            tasks = list(setup_root.iter("IKMarkerTask"))
            self.assertEqual(
                tuple(task.get("name") for task in tasks), subject.MARKER_NAMES
            )
            self.assertEqual(
                [task.findtext("apply") for task in tasks], ["true"] * 28
            )
            self.assertEqual(
                [task.findtext("weight") for task in tasks], ["1"] * 28
            )
            self.assertEqual(
                setup_root.findtext(".//accuracy"), "1e-05"
            )
            self.assertEqual(
                setup_root.findtext(".//time_range"), "11.99 155.045"
            )
            self.assertFalse(
                Path(setup_root.findtext(".//model_file", "")).is_absolute()
            )
            self.assertFalse(
                Path(setup_root.findtext(".//marker_file", "")).is_absolute()
            )

            manifest = json.loads(generated_manifest.read_text(encoding="utf-8"))
            inverse = manifest["inverse_kinematics"]
            self.assertEqual(
                inverse["required_opensim_version"],
                subject.REQUIRED_OPENSIM_VERSION,
            )
            self.assertEqual(inverse["accuracy"], subject.IK_ACCURACY)
            self.assertEqual(inverse["time_range_s"], [11.99, 155.045])
            self.assertEqual(len(inverse["marker_tasks"]), 28)
            self.assertIsNone(inverse["output_ik_sha256"])
            self.assertEqual(inverse["setup"]["sha256"], _sha256(generated_setup))
            self.assertEqual(inverse["model"]["sha256"], _sha256(IK_MODEL))
            self.assertEqual(
                manifest["outputs"]["dataset_ik"]["sha256"],
                _sha256(generated_ik),
            )

            # Exercise the causal run -> execution receipt -> final receipt
            # chain without invoking the OpenSim solver in this pure unit test.
            # The fake tool writes a schema-valid conversion output; a separate
            # integration run is responsible for reproducing the frozen V13 IK.
            generated_opensim_ik = output / f"{TRIAL}_ik.mot"
            class FakeIkTool:
                def __init__(self, setup_name: str) -> None:
                    self.setup_name = setup_name

                def run(self) -> bool:
                    shutil.copyfile(generated_ik, generated_opensim_ik)
                    return True

            class FakeOpenSim:
                InverseKinematicsTool = FakeIkTool

                def LoadOpenSimLibrary(self, path: str) -> None:
                    self.loaded_plugin = path

            fake_opensim = FakeOpenSim()
            with patch.object(
                subject,
                "_load_pinned_opensim",
                return_value=(fake_opensim, "4.5.2"),
            ):
                run_result = subject.run_ik_from_setup(
                    setup_xml=generated_setup,
                    ik_plugin=IK_PLUGIN,
                )
                execution_receipt = (
                    output / f"{TRIAL}_ik_execution_receipt.json"
                )
                held_receipt = execution_receipt.with_suffix(".held")
                execution_receipt.rename(held_receipt)
                with self.assertRaises(subject.ConversionError):
                    subject.finalize_ik_receipt(
                        setup_xml=generated_setup,
                        output_ik_mot=generated_opensim_ik,
                        ik_plugin=IK_PLUGIN,
                    )
                held_receipt.rename(execution_receipt)
                receipt_result = subject.finalize_ik_receipt(
                    setup_xml=generated_setup,
                    output_ik_mot=generated_opensim_ik,
                    ik_plugin=IK_PLUGIN,
                    opensim_version="4.5.2",
            )
            receipt = output / f"{TRIAL}_ik_receipt.json"
            receipt_payload = json.loads(receipt.read_text(encoding="utf-8"))
            self.assertEqual(receipt_result["status"], "IK_OUTPUT_VERIFIED")
            self.assertEqual(
                run_result["execution_receipt"],
                execution_receipt.resolve().as_posix(),
            )
            self.assertEqual(
                receipt_payload["execution_receipt"]["sha256"],
                _sha256(execution_receipt),
            )
            self.assertEqual(_sha256(V13_IK), V13_IK_SHA256)
            self.assertEqual(
                receipt_payload["output_ik"]["sha256"], _sha256(generated_ik)
            )
            self.assertEqual(
                receipt_payload["plugin"]["binary_sha256"], _sha256(IK_PLUGIN)
            )
            with patch.object(
                subject,
                "_load_pinned_opensim",
                return_value=(fake_opensim, "4.5.2"),
            ):
                with self.assertRaises(subject.NoClobberError):
                    subject.finalize_ik_receipt(
                        setup_xml=generated_setup,
                        output_ik_mot=generated_opensim_ik,
                        ik_plugin=IK_PLUGIN,
                        opensim_version="4.5.2",
                    )

    def test_no_clobber_runs_before_any_source_decode(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary) / "already_exists"
            output.mkdir()
            with patch.object(subject, "decode_mcos_table") as decode:
                with self.assertRaises(subject.NoClobberError):
                    subject.convert_trial(
                        ik_mat=IK_MAT,
                        fp_mat=FP_MAT,
                        markers_mat=MARKERS_MAT,
                        output_dir=output,
                        trial=TRIAL,
                    )
            decode.assert_not_called()
            self.assertEqual(list(output.iterdir()), [])

    def test_decoder_fails_closed_on_plain_matlab_matrix(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            source = Path(temporary) / "not_a_table.mat"
            savemat(source, {"data": np.arange(6, dtype=float).reshape(2, 3)})
            with self.assertRaises(subject.TableSchemaError):
                subject.decode_mcos_table(source)

    def test_individual_writers_refuse_existing_files(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            target = Path(temporary) / "occupied.mot"
            target.write_text("owned by user\n", encoding="utf-8")
            data = subject.extract_coordinate_data(self.ik_table)
            with self.assertRaises(subject.NoClobberError):
                subject.write_coordinate_mot(target, data)
            self.assertEqual(target.read_text(encoding="utf-8"), "owned by user\n")


if __name__ == "__main__":
    unittest.main()
