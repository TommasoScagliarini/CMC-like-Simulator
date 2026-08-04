"""Pure and synthetic tests for the V14.1 recovery wrapper."""

from __future__ import annotations

import inspect
import io
import json
import copy
import sys
import tempfile
import unittest
from contextlib import redirect_stdout
from pathlib import Path
from types import ModuleType, SimpleNamespace
from typing import Any
from unittest import mock

import numpy as np


VALIDATION_ROOT = Path(__file__).resolve().parent
if str(VALIDATION_ROOT) not in sys.path:
    sys.path.insert(0, str(VALIDATION_ROOT))


class _ForbiddenOpenSim(ModuleType):
    def __getattr__(self, name: str) -> Any:
        raise AssertionError(f"pure V14.1 test touched opensim.{name}")


with mock.patch.dict(sys.modules, {"opensim": _ForbiddenOpenSim("opensim")}):
    import sweep_two_sensor_cross_speed_v14_1 as subject  # noqa: E402


def _synthetic_raw_sources() -> dict[str, dict[str, object]]:
    return {
        role: {
            "path": f"/synthetic/{role}/treadmill_02_01.mat",
            "sha256": f"sha-{role}",
            "size_bytes": 1,
        }
        for role in ("ik", "fp", "markers", "conditions")
    }


def _manifest_sources(
    raw_sources: dict[str, dict[str, object]],
) -> dict[str, dict[str, str]]:
    return {
        manifest_role: {
            "filename": Path(str(raw_sources[protocol_role]["path"])).name,
            "sha256": str(raw_sources[protocol_role]["sha256"]),
        }
        for manifest_role, protocol_role in (
            ("ik", "ik"),
            ("force_plate", "fp"),
            ("markers", "markers"),
        )
    }


class RecoveryProtocolTest(unittest.TestCase):
    def test_parent_failure_and_golden_are_exact(self) -> None:
        parent = subject._validate_parent_failure()
        golden = subject._validated_recovery_golden()
        self.assertEqual(
            parent["status"], "PASS_PARENT_V14_FAILURE_BOUNDARY_VERIFIED"
        )
        self.assertTrue(all(parent["checks"].values()))
        self.assertTrue(all(golden["checks"].values()))

    def test_protocol_preserves_normalized_scientific_core(self) -> None:
        parent = subject._load_json(subject.PARENT_PROTOCOL)
        payload = subject.expected_protocol_payload(require_all_sources=False)
        self.assertEqual(
            subject._scientific_core(payload), subject._scientific_core(parent)
        )
        self.assertEqual(
            subject._canonical_sha256(subject._scientific_core(payload)),
            subject.PARENT_SCIENTIFIC_CORE_SHA256,
        )

    def test_analysis_intervals_are_frozen_inward_lattice_points(self) -> None:
        payload = subject.expected_protocol_payload(require_all_sources=False)
        subject._validate_analysis_catalog(payload["trials"])
        for trial_id, expected in subject.ANALYSIS_INTERVALS_S.items():
            trial = payload["trials"][trial_id]
            self.assertEqual(trial["trial_interval_s"], expected)
            source_start, source_end = trial["source_trial_interval_s"]
            self.assertGreaterEqual(
                expected[0] - source_start, subject.MINIMUM_EDGE_INSET_S - 1e-12
            )
            self.assertGreaterEqual(
                source_end - expected[1], subject.MINIMUM_EDGE_INSET_S - 1e-12
            )
            for value in expected:
                ticks = value / subject.ANALYSIS_TIME_LATTICE_S
                self.assertAlmostEqual(ticks, round(ticks), places=8)

    def test_plateau_definitions_are_identical_to_parent(self) -> None:
        parent = subject._load_json(subject.PARENT_PROTOCOL)
        payload = subject.expected_protocol_payload(require_all_sources=False)
        for trial_id in parent["trials"]:
            self.assertEqual(
                payload["trials"][trial_id]["plateaus"],
                parent["trials"][trial_id]["plateaus"],
            )

    def test_10ms_ticks_are_subset_of_1ms_ticks(self) -> None:
        for start, end in subject.ANALYSIS_INTERVALS_S.values():
            primary = {
                round(start + index * 0.010, 9)
                for index in range(int((end - start) // 0.010) + 1)
            }
            fine = {
                round(start + index * 0.001, 9)
                for index in range(int((end - start) // 0.001) + 1)
            }
            self.assertTrue(primary <= fine)

    def test_recovery_paths_are_distinct_from_consumed_v14(self) -> None:
        self.assertNotEqual(subject.DEFAULT_PROTOCOL, subject.PARENT_PROTOCOL)
        self.assertNotEqual(subject.DEFAULT_EXECUTION_LEDGER, subject.PARENT_LEDGER)
        self.assertNotEqual(subject.DEFAULT_OUTPUT_DIR, subject.PARENT_OUTPUT_DIR)
        self.assertEqual(
            sorted(path.name for path in subject.PARENT_OUTPUT_DIR.iterdir()),
            ["failure.json", "run_start_receipt.json"],
        )

    def test_protocol_loader_rejects_alternate_path_before_read(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            alternate = Path(tmp) / "protocol.json"
            alternate.write_text("{}\n", encoding="utf-8")
            with self.assertRaisesRegex(subject.ProtocolError, "canonical"):
                subject.load_and_validate_protocol(alternate)

    def test_runtime_guard_rejects_mutated_split_before_execution(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            protocol_path = Path(tmp) / "protocol.json"
            protocol_path.write_text("{}\n", encoding="utf-8")
            canonical = {
                "protocol_id": subject.PROTOCOL_ID,
                "split": {"DEVELOPMENT": ["02", "04", "08"]},
                "_protocol_path": protocol_path.resolve().as_posix(),
                "_protocol_sha256": subject._sha256(protocol_path),
            }
            mutated = copy.deepcopy(canonical)
            mutated["split"]["DEVELOPMENT"] = ["03"]
            with (
                mock.patch.object(subject, "DEFAULT_PROTOCOL", protocol_path),
                mock.patch.object(
                    subject, "load_and_validate_protocol", return_value=canonical
                ),
            ):
                subject._validate_runtime_protocol(canonical)
                with self.assertRaisesRegex(subject.ProtocolError, "canonical"):
                    subject._validate_runtime_protocol(mutated)

    def test_alternate_output_with_sentinel_receipt_is_byte_immutable(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            canonical = root / "canonical"
            alternate = root / "alternate"
            alternate.mkdir()
            receipt = alternate / "run_start_receipt.json"
            receipt.write_bytes(b"sentinel\n")
            before = {path.name: path.read_bytes() for path in alternate.iterdir()}
            with (
                mock.patch.object(subject, "DEFAULT_OUTPUT_DIR", canonical),
                mock.patch.object(
                    subject, "DEFAULT_EXECUTION_LEDGER", root / "ledger.json"
                ),
                redirect_stdout(io.StringIO()),
            ):
                code = subject.main(
                    ["--execute", "--output-dir", str(alternate)]
                )
            after = {path.name: path.read_bytes() for path in alternate.iterdir()}
        self.assertEqual(code, 1)
        self.assertEqual(after, before)
        self.assertNotIn("failure.json", after)

    def test_consumed_canonical_retry_does_not_add_failure(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            canonical = root / "canonical"
            canonical.mkdir()
            (canonical / "run_start_receipt.json").write_bytes(b"sentinel\n")
            ledger = root / "ledger.json"
            ledger.write_bytes(b"consumed\n")
            before = {path.name: path.read_bytes() for path in canonical.iterdir()}
            with (
                mock.patch.object(subject, "DEFAULT_OUTPUT_DIR", canonical),
                mock.patch.object(subject, "DEFAULT_EXECUTION_LEDGER", ledger),
                redirect_stdout(io.StringIO()),
            ):
                code = subject.main(
                    ["--execute", "--output-dir", str(canonical)]
                )
            after = {path.name: path.read_bytes() for path in canonical.iterdir()}
        self.assertEqual(code, 2)
        self.assertEqual(after, before)
        self.assertNotIn("failure.json", after)

    def test_midrun_no_clobber_persists_failure_once(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            canonical = root / "canonical"
            ledger = root / "ledger.json"

            def fail_after_start(*_args: object) -> dict[str, object]:
                canonical.mkdir()
                (canonical / "run_start_receipt.json").write_text(
                    "{}\n", encoding="utf-8"
                )
                ledger.write_text("{}\n", encoding="utf-8")
                raise subject.NoClobberError("synthetic mid-run collision")

            with (
                mock.patch.object(subject, "DEFAULT_OUTPUT_DIR", canonical),
                mock.patch.object(subject, "DEFAULT_EXECUTION_LEDGER", ledger),
                mock.patch.object(subject, "_preflight_no_clobber"),
                mock.patch.object(subject, "load_and_validate_protocol", return_value={}),
                mock.patch.object(
                    subject, "run_cross_speed_protocol", side_effect=fail_after_start
                ),
                redirect_stdout(io.StringIO()),
            ):
                code = subject.main(
                    ["--execute", "--output-dir", str(canonical)]
                )
            failure = json.loads((canonical / "failure.json").read_text())
        self.assertEqual(code, 2)
        self.assertEqual(
            failure["status"],
            "ERROR_AFTER_V14_1_RECOVERY_DESTINATION_CONSUMED",
        )
        self.assertFalse(failure["rerun_allowed"])

    def test_completed_scientific_fail_returns_nonzero(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            canonical = root / "canonical"
            with (
                mock.patch.object(subject, "DEFAULT_OUTPUT_DIR", canonical),
                mock.patch.object(
                    subject, "DEFAULT_EXECUTION_LEDGER", root / "ledger.json"
                ),
                mock.patch.object(subject, "_preflight_no_clobber"),
                mock.patch.object(subject, "load_and_validate_protocol", return_value={}),
                mock.patch.object(
                    subject, "run_cross_speed_protocol", return_value={"ok": False}
                ),
                redirect_stdout(io.StringIO()),
            ):
                code = subject.main(
                    ["--execute", "--output-dir", str(canonical)]
                )
        self.assertEqual(code, 1)

    def test_prepare_writes_receipt_before_raw_identity_or_decode(self) -> None:
        source = inspect.getsource(subject._prepare_trial_hook)
        receipt = source.index("state.stage_access_receipts[trial_id] = receipt")
        raw_identity = source.index("v14._verify_raw_identity")
        decode = source.index("recovery_converter.convert_trial")
        self.assertLess(receipt, raw_identity)
        self.assertLess(raw_identity, decode)
        self.assertIn("required_time_range_s=required_range", source)

    def test_preprocessing_lock_is_written_before_prepare_returns(self) -> None:
        source = inspect.getsource(subject._prepare_trial_hook)
        lock_write = source.index('f"{stem}_preprocessing_lock.json"')
        register = source.index("state.preprocessing_locks[trial_id] = lock")
        returns = source.index("return v14.TrialArtifacts")
        self.assertLess(lock_write, register)
        self.assertLess(register, returns)

    def test_conversion_manifest_accepts_exact_coverage(self) -> None:
        required = [1.0, 2.0]
        raw_sources = _synthetic_raw_sources()
        payload = {
            "schema_version": 1,
            "status": "CONVERTED",
            "trial": "treadmill_02_01",
            "sources": _manifest_sources(raw_sources),
            "time_range_s": required,
            "recovery_lineage": {
                "original_converter": {
                    "path": str(subject.TOOLS_ROOT / "convert_epic_ab06_tables.py"),
                    "sha256": subject.EXPECTED_PARENT_HASHES["converter"],
                }
            },
            "source_time_coverage": {
                "schema_version": 1,
                "endpoint_tolerance_s": 1.0e-9,
                "coverage_rule": "every_source_covers_downstream_required_time_range",
                "range_origin": "explicit_protocol_range",
                "downstream_required_time_range_s": required,
                "all_sources_cover_required_range": True,
                "dataset_ik_used_downstream": False,
                "streams": {
                    name: {
                        "covers_required_range": True,
                        "rows": 3,
                        "time_range_s": required,
                    }
                    for name in ("dataset_ik", "force_plate", "markers")
                },
            },
            "inverse_kinematics": {"time_range_s": required},
        }
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "manifest.json"
            path.write_text(json.dumps(payload), encoding="utf-8")
            observed = subject._validate_conversion_manifest(
                path,
                required,
                expected_trial="treadmill_02_01",
                raw_sources=raw_sources,
            )
            wrong_trial = copy.deepcopy(payload)
            wrong_trial["trial"] = "treadmill_04_01"
            path.write_text(json.dumps(wrong_trial), encoding="utf-8")
            with self.assertRaises(subject.ProtocolError):
                subject._validate_conversion_manifest(
                    path,
                    required,
                    expected_trial="treadmill_02_01",
                    raw_sources=raw_sources,
                )
            wrong_source = copy.deepcopy(payload)
            wrong_source["sources"]["force_plate"]["sha256"] = "wrong"
            path.write_text(json.dumps(wrong_source), encoding="utf-8")
            with self.assertRaises(subject.ProtocolError):
                subject._validate_conversion_manifest(
                    path,
                    required,
                    expected_trial="treadmill_02_01",
                    raw_sources=raw_sources,
                )
        self.assertEqual(observed["time_range_s"], required)

    def test_conversion_manifest_rejects_uncovered_stream(self) -> None:
        required = [1.0, 2.0]
        raw_sources = _synthetic_raw_sources()
        payload = {
            "schema_version": 1,
            "status": "CONVERTED",
            "trial": "treadmill_02_01",
            "sources": _manifest_sources(raw_sources),
            "time_range_s": required,
            "recovery_lineage": {
                "original_converter": {
                    "path": str(subject.TOOLS_ROOT / "convert_epic_ab06_tables.py"),
                    "sha256": subject.EXPECTED_PARENT_HASHES["converter"],
                }
            },
            "source_time_coverage": {
                "schema_version": 1,
                "endpoint_tolerance_s": 1.0e-9,
                "coverage_rule": "every_source_covers_downstream_required_time_range",
                "range_origin": "explicit_protocol_range",
                "downstream_required_time_range_s": required,
                "all_sources_cover_required_range": True,
                "dataset_ik_used_downstream": False,
                "streams": {
                    "dataset_ik": {
                        "covers_required_range": True,
                        "rows": 3,
                        "time_range_s": required,
                    },
                    "force_plate": {
                        "covers_required_range": False,
                        "rows": 3,
                        "time_range_s": required,
                    },
                    "markers": {
                        "covers_required_range": True,
                        "rows": 3,
                        "time_range_s": required,
                    },
                },
            },
            "inverse_kinematics": {"time_range_s": required},
        }
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "manifest.json"
            path.write_text(json.dumps(payload), encoding="utf-8")
            with self.assertRaises(subject.ProtocolError):
                subject._validate_conversion_manifest(
                    path,
                    required,
                    expected_trial="treadmill_02_01",
                    raw_sources=raw_sources,
                )

    def test_evaluate_requires_matching_preprocessing_lock(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            work_dir = root / "preprocessed" / "trial_02"
            work_dir.mkdir(parents=True)
            stem = "treadmill_02_01"
            ik_receipt = work_dir / f"{stem}_ik_receipt.json"
            ik_receipt.write_text("{}\n", encoding="utf-8")
            ik_motion = work_dir / f"{stem}_ik.mot"
            ik_motion.write_text("ik\n", encoding="utf-8")
            dataset_ik = work_dir / f"{stem}_ik_dataset_ab06_seasea.mot"
            trc = work_dir / f"{stem}.trc"
            grf = work_dir / f"{stem}_grf.mot"
            external = work_dir / f"{stem}_ExternalLoads.xml"
            ik_setup = work_dir / f"{stem}_iksetup.xml"
            for path in (dataset_ik, trc, grf, external, ik_setup):
                path.write_text(f"{path.name}\n", encoding="utf-8")
            access_receipt = root / "access.json"
            access_receipt.write_text("{}\n", encoding="utf-8")
            required = [1.0, 2.0]
            raw_sources = _synthetic_raw_sources()
            coverage = {
                "schema_version": 1,
                "endpoint_tolerance_s": 1.0e-9,
                "coverage_rule": "every_source_covers_downstream_required_time_range",
                "range_origin": "explicit_protocol_range",
                "downstream_required_time_range_s": required,
                "all_sources_cover_required_range": True,
                "dataset_ik_used_downstream": False,
                "streams": {
                    name: {
                        "covers_required_range": True,
                        "rows": 3,
                        "time_range_s": required,
                    }
                    for name in ("dataset_ik", "force_plate", "markers")
                },
            }
            conversion_manifest = work_dir / f"{stem}_conversion_manifest.json"
            manifest = {
                "schema_version": 1,
                "status": "CONVERTED",
                "trial": stem,
                "sources": _manifest_sources(raw_sources),
                "time_range_s": required,
                "recovery_lineage": {
                    "original_converter": {
                        "path": str(
                            subject.TOOLS_ROOT / "convert_epic_ab06_tables.py"
                        ),
                        "sha256": subject.EXPECTED_PARENT_HASHES["converter"],
                    }
                },
                "source_time_coverage": coverage,
                "outputs": {
                    role: {
                        "path": path.name,
                        "sha256": subject._sha256(path),
                    }
                    for role, path in (
                        ("dataset_ik", dataset_ik),
                        ("markers_trc", trc),
                        ("grf", grf),
                        ("external_loads", external),
                    )
                },
                "inverse_kinematics": {
                    "time_range_s": required,
                    "setup": {
                        "path": ik_setup.name,
                        "sha256": subject._sha256(ik_setup),
                    },
                },
            }
            conversion_manifest.write_text(
                json.dumps(manifest),
                encoding="utf-8",
            )
            execution_receipt = work_dir / f"{stem}_ik_execution_receipt.json"
            execution_receipt.write_text("{}\n", encoding="utf-8")
            ik_receipt.write_text(
                json.dumps(
                    {
                        "status": "IK_OUTPUT_VERIFIED",
                        "trial": stem,
                        "opensim_version": "4.5.2",
                        "conversion_manifest": {
                            "path": conversion_manifest.name,
                            "sha256": subject._sha256(conversion_manifest),
                        },
                        "execution_receipt": {
                            "path": execution_receipt.name,
                            "sha256": subject._sha256(execution_receipt),
                        },
                        "setup": {
                            "path": ik_setup.name,
                            "sha256": subject._sha256(ik_setup),
                        },
                        "model": {
                            "sha256": subject.v14.metadata.EXPECTED_MARKER_CALIBRATED_MODEL_SHA256
                        },
                        "plugin": {
                            "binary_sha256": next(
                                iter(
                                    subject.v14.EXPECTED_PLUGIN_BINARY_SHA256_BY_SUFFIX.values()
                                )
                            )
                        },
                        "marker_contract": {
                            "count": 28,
                            "apply": True,
                            "weight": 1.0,
                            "accuracy": 1.0e-5,
                        },
                        "output_ik": {
                            "path": ik_motion.name,
                            "sha256": subject._sha256(ik_motion),
                            "coordinate_count": 21,
                            "rows": 3,
                            "time_range_s": required,
                        },
                    }
                ),
                encoding="utf-8",
            )
            protocol = {
                "trials": {
                    "02": {
                        "trial_interval_s": required,
                        "raw_sources": raw_sources,
                    }
                },
            }
            artifacts = SimpleNamespace(
                trial_id="02",
                stage="development",
                work_dir=work_dir,
                ik_receipt=ik_receipt,
                ik_motion=ik_motion,
                conversion_manifest=conversion_manifest,
                trc=trc,
                grf=grf,
                external_loads=external,
                ik_setup=ik_setup,
                setup=SimpleNamespace(
                    kinematics_file=ik_motion,
                    external_loads_xml=external,
                    t_start=required[0],
                    t_end=required[1],
                ),
            )
            state = subject._RuntimeState(protocol, root, {}, {}, {}, {}, {})
            with mock.patch.object(subject, "_RUNTIME", state):
                with self.assertRaises(subject.ProtocolError):
                    subject._evaluate_trial_hook(protocol, artifacts)

            lock = work_dir / "treadmill_02_01_preprocessing_lock.json"
            lock.write_text(
                json.dumps(
                    {
                        "status": "V14_1_PREPROCESSING_FROZEN_BEFORE_DETECTOR_REPLAY",
                        "trial_id": "02",
                        "stage": "development",
                        "analysis_interval_s": required,
                        "absolute_timestamps_no_rezero": True,
                        "adaptive_crop_resample_or_interpolation_used": False,
                        "dataset_ik_used_downstream": False,
                        "all_sources_cover_analysis_interval": True,
                        "source_time_coverage": coverage,
                        "access_receipt": subject._source_record(access_receipt),
                        "conversion_manifest": subject._source_record(
                            conversion_manifest
                        ),
                        "ik_receipt": subject._source_record(ik_receipt),
                        "ik_execution_receipt": subject._source_record(
                            execution_receipt
                        ),
                        "ik_motion": subject._source_record(ik_motion),
                        "preprocessed_files": subject._validate_preprocessed_files(
                            manifest,
                            work_dir=work_dir,
                            stem=stem,
                            trc=trc,
                            grf=grf,
                            external=external,
                            ik_setup=ik_setup,
                            ik_motion=ik_motion,
                        ),
                        "live_replay_inputs": {"synthetic": True},
                        "recovery_lineage": subject._source_record(
                            subject.RECOVERY_LINEAGE
                        ),
                    }
                ),
                encoding="utf-8",
            )
            state.stage_access_receipts["02"] = access_receipt
            state.preprocessing_locks["02"] = lock
            state.preprocessing_lock_sha256["02"] = subject._sha256(lock)
            with (
                mock.patch.object(subject, "_RUNTIME", state),
                mock.patch.object(subject, "_PARENT_EVALUATE_TRIAL", return_value="ok"),
                mock.patch.object(
                    subject,
                    "_validated_live_replay_inputs",
                    return_value={"synthetic": True},
                ),
            ):
                self.assertEqual(
                    subject._evaluate_trial_hook(protocol, artifacts), "ok"
                )
            grf.write_text("tampered after lock\n", encoding="utf-8")
            with mock.patch.object(subject, "_RUNTIME", state):
                with self.assertRaises(subject.ProtocolError):
                    subject._evaluate_trial_hook(protocol, artifacts)

    def test_configured_parent_restores_all_globals(self) -> None:
        names = (
            "DEFAULT_PROTOCOL",
            "DEFAULT_OUTPUT_DIR",
            "DEFAULT_EXECUTION_LEDGER",
            "prepare_trial",
            "_evaluate_trial",
            "evaluate_continuous_candidate",
            "root_safe_isolated",
            "_write_json_exclusive",
        )
        before = {name: getattr(subject.v14, name) for name in names}
        with tempfile.TemporaryDirectory() as tmp:
            with subject._configured_parent({}, Path(tmp)):
                self.assertIs(subject.v14.prepare_trial, subject._prepare_trial_hook)
        self.assertEqual(before, {name: getattr(subject.v14, name) for name in names})
        self.assertIsNone(subject._RUNTIME)

    def test_parent_holdout_and_reserve_access_logic_is_reused(self) -> None:
        run_source = inspect.getsource(subject.run_cross_speed_protocol)
        self.assertIn("v14.run_cross_speed_protocol", run_source)
        self.assertNotIn("VALIDATION", inspect.getsource(subject._prepare_trial_hook))
        payload = subject.expected_protocol_payload(require_all_sources=False)
        self.assertEqual(payload["split"]["VALIDATION"], ["05"])
        self.assertEqual(payload["split"]["SEALED"], ["06"])
        self.assertEqual(payload["split"]["RESERVE"], ["03", "07"])


class WarmupRuntimeIntegrationTest(unittest.TestCase):
    @staticmethod
    def _bundle() -> SimpleNamespace:
        times = np.arange(0.0, 2.0001, 0.1)
        states = np.zeros(times.size)
        states[(times >= 0.2) & (times < 0.5)] = 1
        states[(times >= 0.5) & (times < 0.8)] = 2
        states[times >= 0.8] = 1
        return SimpleNamespace(
            trial_id="02",
            cadence_label="runtime_10ms",
            sample_dt_s=0.1,
            plateau_references=(
                {"events": {"heel_strike": np.asarray([1.09, 1.8])}},
            ),
            shared={"times": times, "state_ids": states},
        )

    @staticmethod
    def _replay(*, complete: bool = True) -> dict[str, object]:
        accepted = [
            {
                "event": "heel_strike",
                "event_time_s": 0.18,
                "confirmed_time_s": 0.20,
            },
            {
                "event": "toe_off",
                "event_time_s": 0.48,
                "confirmed_time_s": 0.50,
            },
        ]
        if complete:
            accepted.append(
                {
                    "event": "heel_strike",
                    "event_time_s": 0.78,
                    "confirmed_time_s": 0.80,
                }
            )
        bundle = WarmupRuntimeIntegrationTest._bundle()
        return {
            "accepted": accepted,
            "invalid_steps": [],
            "state_id": bundle.shared["state_ids"],
        }

    @staticmethod
    def _parent_rows() -> tuple[list[dict[str, object]], list[dict[str, object]]]:
        rows = [
            {
                "candidate_id": "challenger",
                "v14_stage_unit": f"u{index}",
                "unit_full_gate_ok": True,
            }
            for index in range(4)
        ]
        details = [
            {
                "row": row,
                "dynamic_gate_result": {"ok": True, "checks": {}},
            }
            for row in rows
        ]
        return rows, details

    def test_candidate_hook_saves_certificate_without_changing_denominators(self) -> None:
        bundle = self._bundle()
        runtime = subject._RuntimeState({}, Path("."), {}, {}, {}, {}, {})

        def parent_evaluate(*_args: object) -> tuple[list[dict[str, object]], list[dict[str, object]]]:
            subject.v14.v1._run_production_fsm()
            return self._parent_rows()

        with (
            mock.patch.object(subject, "_RUNTIME", runtime),
            mock.patch.object(subject, "_PARENT_RUN_FSM", return_value=self._replay()),
            mock.patch.object(subject, "_PARENT_EVALUATE_CANDIDATE", side_effect=parent_evaluate),
            mock.patch.object(
                subject,
                "_prescribed_warmup_events",
                return_value={
                    "heel_strike": np.asarray([0.05, 0.75, 1.09]),
                    "toe_off": np.asarray([0.30, 0.90]),
                },
            ),
        ):
            rows, details = subject._evaluate_candidate_hook(
                {}, None, SimpleNamespace(candidate_id="challenger"), bundle
            )
        self.assertTrue(all(row["warmup_ok"] for row in rows))
        self.assertTrue(
            all(row["warmup_enters_scoring_denominator"] is False for row in rows)
        )
        self.assertTrue(
            all(
                detail["dynamic_gate_result"]["checks"][
                    "pre_score_warmup_certificate"
                ]
                for detail in details
            )
        )
        self.assertEqual(len(runtime.warmup_certificates), 1)

    def test_v13_warmup_failure_aborts_protocol(self) -> None:
        bundle = self._bundle()
        runtime = subject._RuntimeState({}, Path("."), {}, {}, {}, {}, {})

        def parent_evaluate(*_args: object) -> tuple[list[dict[str, object]], list[dict[str, object]]]:
            subject.v14.v1._run_production_fsm()
            return self._parent_rows()

        with (
            mock.patch.object(subject, "_RUNTIME", runtime),
            mock.patch.object(
                subject, "_PARENT_RUN_FSM", return_value=self._replay(complete=False)
            ),
            mock.patch.object(subject, "_PARENT_EVALUATE_CANDIDATE", side_effect=parent_evaluate),
            mock.patch.object(
                subject,
                "_prescribed_warmup_events",
                return_value={
                    "heel_strike": np.asarray([0.05, 0.75, 1.09]),
                    "toe_off": np.asarray([0.30, 0.90]),
                },
            ),
        ):
            with self.assertRaisesRegex(subject.ProtocolError, "V13 warm-up"):
                subject._evaluate_candidate_hook(
                    {},
                    None,
                    SimpleNamespace(candidate_id=subject.v14.BASELINE_ID),
                    bundle,
                )

    def test_root_safe_hook_conjoins_warmup(self) -> None:
        rows = [{"v14_stage_unit": "u", "warmup_ok": False}]
        with mock.patch.object(subject, "_PARENT_ROOT_SAFE", return_value={"ok": True}):
            result = subject._root_safe_hook(rows, rows)
        self.assertFalse(result["ok"])
        self.assertFalse(result["warmup"]["ok"])


if __name__ == "__main__":
    unittest.main()
